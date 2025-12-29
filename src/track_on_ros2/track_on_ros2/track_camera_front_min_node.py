#!/usr/bin/env python3
"""
ROS2节点：前端摄像头最简关键点跟踪（仅2D像素坐标 + 3D JSON 打印）

新增特性：
- 3D 打印为 JSON（每帧）
- 不可见点也打印 JSON（Mode 2）
- 修复未定义变量（intrinsics_file, depth_topic_str）
- 稳定的 tracking 初始化
"""

import os
import sys
import cv2
import json
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Header, Bool
import matplotlib.pyplot as plt
import re
import yaml

#############################
# 路径搜索 tracking_module
#############################

def find_track_on_path():
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    path = current_file_dir
    for _ in range(10):
        src_track_on = os.path.join(path, 'src', 'track_on')
        if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
            return src_track_on
        if os.path.basename(path) == 'src':
            track_on = os.path.join(path, 'track_on')
            if os.path.isdir(track_on) and os.path.isfile(os.path.join(track_on, 'tracking_module.py')):
                return track_on
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent

    # AMENT_PREFIX_PATH 搜索
    for env_var in ['AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH']:
        env_path = os.environ.get(env_var, '')
        if env_path:
            for prefix_path in env_path.split(':'):
                if 'tracking_with_cameara_ws' in prefix_path:
                    ws_root = prefix_path.split('tracking_with_cameara_ws')[0] + 'tracking_with_cameara_ws'
                    src_track_on = os.path.join(ws_root, 'src', 'track_on')
                    if os.path.isdir(src_track_on):
                        return src_track_on

    # fallback: 使用相对路径从当前文件位置查找
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    # 尝试从当前文件向上查找工作空间根目录
    ws_root = current_file_dir
    for _ in range(5):
        if os.path.basename(ws_root) == 'tracking_with_cameara_ws':
            known_path = os.path.join(ws_root, 'src', 'track_on')
            if os.path.isdir(known_path):
                return known_path
        ws_root = os.path.dirname(ws_root)
        if ws_root == os.path.dirname(ws_root):
            break

    return None


track_on_path = find_track_on_path()
if track_on_path and track_on_path not in sys.path:
    sys.path.insert(0, track_on_path)

from tracking_module import TrackingModule  # noqa: E402


#############################
# RobotLib 路径和初始化
#############################

def _ensure_robotlib_visible(robot_lib_path: str):
    """确保 RobotLib 在路径中可见"""
    if not robot_lib_path:
        return
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if robot_lib_path not in ld_path.split(':'):
        os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
    if robot_lib_path not in sys.path:
        sys.path.append(robot_lib_path)


def _init_robot_arm_servo(robot_ip: str, robot_lib_path: str, component_type: int, logger):
    """
    初始化机器人手臂位姿（使用 set_arm_servo_angle）
    Args:
        robot_ip: 机器人IP地址
        robot_lib_path: RobotLib库路径
        component_type: 组件类型 (1: 左臂, 2: 右臂)
        logger: ROS2 logger
    Returns:
        Robot实例或None
    """
    try:
        _ensure_robotlib_visible(robot_lib_path)
        from RobotLib import Robot  # type: ignore
        
        logger.info(f"连接机器人: {robot_ip}")
        robot = Robot(robot_ip, '', '')
        
        # 启用手臂并设置为伺服模式
        logger.info(f"启用手臂组件 {component_type} 并设置为伺服模式...")
        success = robot.set_arm_enable(component_type, True)
        if not success:
            logger.warn(f"启用手臂失败: component_type={component_type}")
            return None
        
        success = robot.set_arm_mode(component_type, 1)
        if not success:
            logger.warn(f"设置手臂模式失败: component_type={component_type}")
            return None
        
        # 设置初始位姿（使用 set_arm_servo_angle）右手的初始位置
        positions = [-0.6006123423576355, 0.11826176196336746, 0.028828054666519165, 
                     1.8238468170166016, -1.4655508995056152, 0.1113307848572731, 
                     0.38727688789367676]
        
        speed = 0.1
        acc = 1.0
        wait = True
        
        logger.info(f"设置手臂初始位姿（使用 set_arm_servo_angle）: {positions}")
        success = robot.set_arm_servo_angle(component_type, positions, speed, acc, wait)
        if success:
            logger.info("手臂初始位姿设置成功")
            time.sleep(1.5)  # 等待到位
        else:
            logger.warn("手臂初始位姿设置失败")
        
        return robot
    except Exception as e:
        logger.error(f"初始化机器人失败: {e}")
        return None


#########################################
# 主节点类
#########################################

class TrackCameraFrontMinNode(Node):

    def _load_config(self):
        """加载配置文件"""
        # 获取包路径
        try:
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory('track_on_ros2')
            config_file = os.path.join(package_path, 'config', 'config.yaml')
        except Exception:
            # 如果找不到包路径，使用相对路径
            current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(current_dir, 'config', 'config.yaml')

        if not os.path.exists(config_file):
            self.get_logger().error(f'配置文件不存在: {config_file}')
            return None

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'成功加载配置文件: {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {e}')
            return None

    def __init__(self):
        super().__init__('track_camera_front_min_node')

        # 加载配置文件
        self.config = self._load_config()
        if self.config is None:
            self.get_logger().error('无法加载配置文件，使用默认参数')
            self.config = {}

        ######## 机器人初始化参数（在最开始） ########
        # 从配置文件读取机器人参数默认值
        default_robot_ip = self.config.get('robot', {}).get('ip', '192.168.22.63:50051')
        default_robot_component_type = self.config.get('robot', {}).get('component_type', 2)
        default_init_robot_arm = self.config.get('robot', {}).get('init_robot_arm', True)

        self.declare_parameter('robot_ip', default_robot_ip)
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            default_lib = os.path.join(get_package_share_directory('Monte_api_ros2'), 'lib')
        except Exception:
            # fallback路径
            default_lib = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
                'src', 'Monte_api_ros2', 'lib'
            )
        self.declare_parameter('robot_lib_path', default_lib)
        self.declare_parameter('robot_component_type', default_robot_component_type)  # 1: 左臂, 2: 右臂
        self.declare_parameter('init_robot_arm', default_init_robot_arm)  # 是否初始化机器人手臂

        # 读取机器人参数
        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        robot_component_type = int(self.get_parameter('robot_component_type').value)
        init_robot_arm = bool(self.get_parameter('init_robot_arm').value)

        # 初始化机器人手臂（在最开始）
        self.robot = None
        if init_robot_arm:
            self.get_logger().info("=" * 50)
            self.get_logger().info("初始化机器人手臂位姿（使用 set_arm_servo_angle）...")
            self.robot = _init_robot_arm_servo(robot_ip, robot_lib_path, robot_component_type, self.get_logger())
            if self.robot:
                self.get_logger().info("机器人手臂初始化完成")
            else:
                self.get_logger().warn("机器人手臂初始化失败，继续运行但无法控制手臂")
            self.get_logger().info("=" * 50)

        # 根据配置文件确定相机配置
        camera_selection = self.config.get('camera', {}).get('selection', 1)
        if camera_selection == 1:
            camera_config = self.config.get('left_camera', {})
            camera_name = "左相机"
        elif camera_selection == 2:
            camera_config = self.config.get('right_camera', {})
            camera_name = "右相机"
        else:
            self.get_logger().warn(f"未知的相机选择: {camera_selection}，使用默认配置")
            camera_config = self.config.get('right_camera', {})
            camera_name = "右相机（默认）"

        self.get_logger().info(f"使用{camera_name}配置")

        # 根据相机选择设置frame_id
        if camera_selection == 1:
            self.camera_frame_id = "left_camera_color_optical_frame"
        else:
            self.camera_frame_id = "right_camera_color_optical_frame"

        # 从配置文件读取默认值
        default_checkpoint = self.config.get('tracking', {}).get('checkpoint_path', '')
        default_camera_topic = camera_config.get('camera_topic', '/right/color/video')
        default_publish_visualization = self.config.get('tracking', {}).get('publish_visualization', True)
        default_show_interactive_window = self.config.get('tracking', {}).get('show_interactive_window', True)

        ######## 参数 ########
        self.declare_parameter('checkpoint_path', default_checkpoint)
        self.declare_parameter('camera_topic', default_camera_topic)
        self.declare_parameter('publish_visualization', default_publish_visualization)
        self.declare_parameter('show_interactive_window', default_show_interactive_window)

        # 3D 相关
        # 根据相机选择设置内参文件和深度话题
        ws_root = self._get_workspace_root()
        default_intrinsics = camera_config.get('intrinsics_file', 'config/camera_rhead_front_intrinsics_02_10.txt')
        if ws_root and not os.path.isabs(default_intrinsics):
            default_intrinsics = os.path.join(ws_root, default_intrinsics)

        default_depth_topic = camera_config.get('depth_topic', '/right/depth/stream')

        self.declare_parameter('intrinsics_file', default_intrinsics)
        self.declare_parameter('depth_topic', default_depth_topic)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('print_3d', True)
        self.declare_parameter('print_3d_interval', 1)
        
        # 深度验证和运动预测参数
        self.declare_parameter('use_depth_validation', True)  # 是否使用深度验证
        self.declare_parameter('background_depth_threshold', 2.5)  # 背景深度阈值（米）
        self.declare_parameter('max_depth_change', 0.4)  # 允许的最大深度变化（米）
        self.declare_parameter('motion_prediction_enabled', True)  # 是否启用运动预测
        self.declare_parameter('search_radius', 25)  # 搜索半径（像素）
        self.declare_parameter('max_motion_step', 20)  # 最大运动步长（像素）

        # 参数读取
        checkpoint_path = self.get_parameter('checkpoint_path').value
        camera_topic = self.get_parameter('camera_topic').value
        publish_visualization = self.get_parameter('publish_visualization').value
        self.show_interactive_window = self.get_parameter('show_interactive_window').value

        intrinsics_file = self.get_parameter('intrinsics_file').value
        depth_topic = self.get_parameter('depth_topic').value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.print_3d = bool(self.get_parameter('print_3d').value)
        self.print_3d_interval = int(self.get_parameter('print_3d_interval').value)
        self.print_3d_interval = max(1, self.print_3d_interval)
        self.print_3d_counter = 0
        
        # 深度验证参数
        self.use_depth_validation = bool(self.get_parameter('use_depth_validation').value)
        self.background_depth_threshold = float(self.get_parameter('background_depth_threshold').value)
        self.max_depth_change = float(self.get_parameter('max_depth_change').value)
        self.motion_prediction_enabled = bool(self.get_parameter('motion_prediction_enabled').value)
        self.search_radius = int(self.get_parameter('search_radius').value)
        self.max_motion_step = int(self.get_parameter('max_motion_step').value)

        ### 保存关键变量（原代码缺失的） ###
        self.intrinsics_file = intrinsics_file
        self.depth_topic_str = depth_topic

        # 检查 checkpoint
        if not checkpoint_path or not os.path.isfile(checkpoint_path):
            self.get_logger().error(f"模型检查点不存在: {checkpoint_path}")
            raise FileNotFoundError(checkpoint_path)

        # 初始化 TrackingModule
        self.get_logger().info(f"加载模型: {checkpoint_path}")
        self.tracker = TrackingModule(checkpoint_path)

        # 读取相机内参
        self.fx = self.fy = self.cx = self.cy = None
        try:
            if intrinsics_file and os.path.isfile(intrinsics_file):
                self.fx, self.fy, self.cx, self.cy = self._load_intrinsics(intrinsics_file)
                self.get_logger().info(
                    f"内参加载成功 fx={self.fx} fy={self.fy} cx={self.cx} cy={self.cy}"
                )
        except Exception as e:
            self.get_logger().warn(f"加载相机内参失败：{e}")

        #######################
        # 深度订阅
        #######################
        self.latest_depth = None
        try:
            self.depth_sub = self.create_subscription(
                Image, depth_topic, self.depth_callback, qos_profile_sensor_data
            )
            self.get_logger().info(f"订阅深度话题: {depth_topic}")
        except Exception as e:
            self.get_logger().warn(f"深度订阅失败: {e}")

        #######################
        # 图像订阅
        #######################
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f"订阅颜色图像话题: {camera_topic}")

        #######################git tag v1.1.0
        # 可视化发布
        #######################
        if publish_visualization:
            self.vis_image_pub = self.create_publisher(Image, "tracking/visualization", 10)
        else:
            self.vis_image_pub = None

        #######################
        # 状态
        #######################
        self.selected_points = []
        self.tracking_started = False
        self.first_frame_captured = False
        self.frame_count = 0
        self.current_frame = None
        self.colors = self._generate_colors(200)
        self.should_exit = False
        
        # 运动预测和深度验证状态
        self.previous_points = None  # 上一帧的点
        self.previous_visibility = None  # 上一帧的可见性
        self.initial_depths = {}  # 初始深度记录 {point_id: depth}
        self.depth_history = {}  # 深度历史 {point_id: [depth1, depth2, ...]}
        self._depths_recorded = False  # 是否已记录初始深度
        # 粗定位状态
        self.waiting_for_coarse = False
        self.coarse_completed = False

        if self.show_interactive_window:
            cv2.namedWindow("Front Tracking")
            cv2.setMouseCallback("Front Tracking", self.mouse_callback)

        #######################
        # 服务接口
        #######################
        from track_on_ros2_srv.srv import SetKeypoints, ControlTracking, ResetTracking
        self.create_service(SetKeypoints, "tracking/set_keypoints", self.set_keypoints_callback)
        self.create_service(ControlTracking, "tracking/control", self.control_tracking_callback)
        self.create_service(ResetTracking, "tracking/reset", self.reset_tracking_callback)

        # Keypoints 发布
        try:
            from track_on_ros2_msgs.msg import Keypoints
            self.Keypoints = Keypoints
        except ImportError:
            self.get_logger().error("无法导入 track_on_ros2_msgs，请先构建工作空间")
            raise

        self.keypoints_pub = self.create_publisher(self.Keypoints, "tracking/keypoints", 10)
        # 3D点云发布（PointCloud）
        self.pc_pub = self.create_publisher(PointCloud, "tracking/points3d", 10)
        # 粗定位 2D keypoint 发布（通知）和监听粗定位完成
        self.coarse_kp_pub = self.create_publisher(self.Keypoints, "tracking/coarse_keypoints", 10)
        self.coarse_done_sub = self.create_subscription(Bool, "tracking/coarse_done", self._coarse_done_callback, 10)

        self.get_logger().info("前端摄像头跟踪节点已启动（含 JSON 3D 打印）")
    ############################################
    # 图像回调
    ############################################
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像解码失败: {e}")
            return
        self.process_frame(frame, msg.header)

    ############################################
    # 处理图像帧（核心）
    ############################################
    def process_frame(self, frame, header):
        self.current_frame = frame.copy()
        display = frame.copy()

        # 未开始跟踪：绘制选点
        if not self.tracking_started:
            for i, (x, y) in enumerate(self.selected_points):
                color = self.colors[i % len(self.colors)]
                cv2.circle(display, (int(x), int(y)), 8, color, -1)
                cv2.putText(display, f"{i+1}", (int(x)+10, int(y)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 初始化跟踪
        if self.tracking_started and not self.first_frame_captured:
            if len(self.selected_points) > 0:
                try:
                    queries = np.array(self.selected_points, dtype=np.float32)
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    init_out = self.tracker.initialize_tracking(queries, rgb)

                    if isinstance(init_out, (tuple, list)) and len(init_out) >= 2:
                        points = np.array(init_out[0], dtype=np.float32)
                        visibility = np.array(init_out[1], dtype=bool)
                    else:
                        points, visibility = init_out

                    self.first_frame_captured = True
                    # 记录初始深度
                    if self.use_depth_validation and self.latest_depth is not None:
                        self._record_initial_depths(points, visibility)
                        self._depths_recorded = True
                    self.publish_keypoints(points, visibility, header)
                    self._compute_and_log_3d(points, visibility)
                    self._publish_points3d(points, visibility, header)
                except Exception as e:
                    self.get_logger().error(f"初始化跟踪失败: {e}")
                    return

        # 跟踪下一帧
        if self.tracking_started and self.first_frame_captured:
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                out = self.tracker.track_next_frame(rgb)
                if isinstance(out, (tuple, list)) and len(out) >= 2:
                    points = np.array(out[0], dtype=np.float32)
                    visibility = np.array(out[1], dtype=bool)
                else:
                    points, visibility = out

                # 深度验证和修正
                if self.use_depth_validation and self.latest_depth is not None:
                    points, visibility = self._validate_and_correct_tracking(
                        points, visibility, frame
                    )
                
                # 保存当前帧用于下一帧的运动预测
                self.previous_points = points.copy()
                self.previous_visibility = visibility.copy()

            except Exception as e:
                self.get_logger().error(f"跟踪失败: {e}")
                return

            # 绘制跟踪结果
            for i, (x, y) in enumerate(points):
                visible = bool(visibility[i])
                color = self.colors[i % len(self.colors)]
                if visible:
                    cv2.circle(display, (int(x), int(y)), 8, color, -1)
                else:
                    cv2.circle(display, (int(x), int(y)), 8, color, 2)
                cv2.putText(display, f"{i+1}", (int(x)+10, int(y)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 发布 keypoints
            self.publish_keypoints(points, visibility, header)

            # 打印3D坐标（文本）
            self._compute_and_log_3d(points, visibility)
            # 发布3D点云
            self._publish_points3d(points, visibility, header)

            self.frame_count += 1

        # 显示窗口
        if self.show_interactive_window:
            cv2.imshow("Front Tracking", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.should_exit = True
            elif key == ord(" ") and len(self.selected_points) > 0:
                self.start_tracking()
            elif key == ord("r"):
                self.reset_tracking()
            elif key == ord("c"):
                # 停止跟踪并清除点，允许重新选择
                self.stop_and_clear_points()

        # 发布可视化图像
        if self.vis_image_pub is not None:
            img_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            img_msg.header = header
            self.vis_image_pub.publish(img_msg)

    ############################################
    # 开始 / 重置 跟踪
    ############################################
    def start_tracking(self):
        if not self.selected_points:
            self.get_logger().warn("没有关键点，无法开始跟踪")
            return
        self.tracking_started = True
        self.first_frame_captured = False
        self.frame_count = 0
        self.get_logger().info("开始跟踪")

    def reset_tracking(self):
        try:
            self.tracker.reset()
        except:
            pass
        self.tracking_started = False
        self.first_frame_captured = False
        self.selected_points = []
        self.frame_count = 0
        # 重置深度验证状态
        self.previous_points = None
        self.previous_visibility = None
        self.initial_depths = {}
        self.depth_history = {}
        self._depths_recorded = False
        self.get_logger().info("跟踪已重置")

    def _coarse_done_callback(self, msg: Bool):
        try:
            if msg.data:
                self.waiting_for_coarse = False
                self.coarse_completed = True
                self.get_logger().info("已收到粗定位完成通知：现在可以选择跟踪关键点 (tracking/keypoints)")
        except Exception as e:
            self.get_logger().warn(f"处理 coarse_done 回调失败: {e}")

    def stop_and_clear_points(self):
        """停止跟踪并清除所有点，允许重新选择关键点"""
        try:
            self.tracker.reset()
        except:
            pass
        self.tracking_started = False
        self.first_frame_captured = False
        self.selected_points = []
        self.frame_count = 0
        # 重置深度验证状态
        self.previous_points = None
        self.previous_visibility = None
        self.initial_depths = {}
        self.depth_history = {}
        self._depths_recorded = False
        self.get_logger().info("已停止跟踪并清除所有点，可以重新点击选择关键点")

    ############################################
    # 服务回调
    ############################################
    def set_keypoints_callback(self, request, response):
        try:
            if self.tracking_started:
                response.success = False
                response.message = "跟踪已在进行中，请先重置"
                return response
            if self.current_frame is None:
                response.success = False
                response.message = "没有可用的图像帧"
                return response
            if len(request.x) != len(request.y):
                response.success = False
                response.message = "x和y数量不一致"
                return response
            h, w = self.current_frame.shape[:2]
            pts = []
            for i in range(len(request.x)):
                x = request.x[i]
                y = request.y[i]
                if 0 <= x < w and 0 <= y < h:
                    pts.append([float(x), float(y)])
                else:
                    self.get_logger().warn(f"关键点 ({x},{y}) 超出范围 ({w},{h})，已忽略")
            if not pts:
                response.success = False
                response.message = "没有有效的关键点"
                return response
            self.selected_points = pts
            response.success = True
            response.message = f"已设置 {len(pts)} 个关键点"
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"设置失败: {e}"
            return response

    def control_tracking_callback(self, request, response):
        try:
            if request.command == "start":
                if not self.selected_points:
                    response.success = False
                    response.message = "请先设置关键点"
                elif self.tracking_started:
                    response.success = False
                    response.message = "跟踪已在进行中"
                else:
                    self.start_tracking()
                    response.success = True
                    response.message = "跟踪已开始"
            elif request.command == "stop":
                if not self.tracking_started:
                    response.success = False
                    response.message = "跟踪未在进行中"
                else:
                    self.tracking_started = False
                    response.success = True
                    response.message = "跟踪已停止"
            else:
                response.success = False
                response.message = "未知命令: start/stop"
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"控制失败: {e}"
            return response

    def reset_tracking_callback(self, request, response):
        try:
            self.reset_tracking()
            response.success = True
            response.message = "跟踪已重置"
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"重置失败: {e}"
            return response

    ############################################
    # Keypoints 发布
    ############################################
    def publish_keypoints(self, points, visibility, header):
        from track_on_ros2_msgs.msg import Keypoint
        msg = self.Keypoints()
        msg.header = header
        msg.num_keypoints = len(points)

        for i in range(len(points)):
            kp = Keypoint()
            kp.id = i
            kp.x = float(points[i, 0])
            kp.y = float(points[i, 1])
            kp.visible = bool(visibility[i])
            msg.keypoints.append(kp)

        self.keypoints_pub.publish(msg)

    ############################################
    # 颜色生成（用于可视化）
    ############################################
    def _generate_colors(self, num_colors: int):
        """生成BGR颜色列表。若matplotlib不可用则退化为固定颜色。"""
        try:
            cmap = plt.cm.get_cmap('tab20', min(num_colors, 20))
            colors = []
            for i in range(num_colors):
                rgba = cmap(i % cmap.N)
                bgr = (int(rgba[2] * 255), int(rgba[1] * 255), int(rgba[0] * 255))
                colors.append(bgr)
            return colors
        except Exception:
            return [(0, 255, 0)] * num_colors

    ############################################
    # JSON 3D 打印（完整模式：所有点都打印）
    ############################################
    def _print_json_3d(self, points, visibility):
        if not self.print_3d:
            return

        # 深度和内参未准备好
        if self.fx is None or self.latest_depth is None:
            return

        logs = []
        n = len(points)

        for i in range(n):
            u = float(points[i, 0])
            v = float(points[i, 1])
            vis = bool(visibility[i])

            if not vis:
                logs.append({
                    "id": i,
                    "u": None, "v": None,
                    "X": None, "Y": None, "Z": None,
                    "visible": False
                })
                continue

            Z = self._depth_at(u, v)
            if Z is None or Z <= 0:
                logs.append({
                    "id": i,
                    "u": u, "v": v,
                    "X": None, "Y": None, "Z": None,
                    "visible": True
                })
                continue

            X = (u - self.cx) / self.fx * Z
            Y = (v - self.cy) / self.fy * Z

            logs.append({
                "id": i,
                "u": u, "v": v,
                "X": float(X),
                "Y": float(Y),
                "Z": float(Z),
                "visible": True
            })

        # 每个点打印一行 JSON
        for item in logs:
            print(json.dumps(item))

    ############################################
    # 文本 3D 打印（参考 front_node 逻辑）
    ############################################
    def _compute_and_log_3d(self, points, visibility):
        if not self.print_3d:
            return
        # 频率控制
        self.print_3d_counter += 1
        if self.print_3d_counter % self.print_3d_interval != 0:
            return
        # 前置检查
        if any(v is None for v in [self.fx, self.fy, self.cx, self.cy]):
            self.get_logger().warn("3D未启用: 内参未就绪，请确认 intrinsics_file 是否正确")
            return
        if self.latest_depth is None:
            self.get_logger().warn(f"3D未启用: 尚未接收到深度帧，depth_topic={self.depth_topic_str}")
            return

        logs = []
        n = len(points)
        for i in range(n):
            if not bool(visibility[i]):
                logs.append(f"点{i+1}: 不可见")
                continue
            u = float(points[i, 0]); v = float(points[i, 1])
            Z = self._depth_at(u, v, ksize=5)
            if Z is None or Z <= 0:
                logs.append(f"点{i+1}: 深度无效 (u={u:.1f}, v={v:.1f})")
                continue
            X = (u - self.cx) / self.fx * Z
            Y = (v - self.cy) / self.fy * Z
            logs.append(f"点{i+1}: X={X:.4f}m Y={Y:.4f}m Z={Z:.4f}m (u={u:.1f}, v={v:.1f})")
        if logs:
            self.get_logger().info("━━━ 关键点3D坐标 ━━━")
            for l in logs:
                self.get_logger().info(l)
            self.get_logger().info(f"内参: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    ############################################
    # 发布3D点云
    ############################################
    def _publish_points3d(self, points, visibility, header):
        # 内参或深度未就绪则不发布
        if any(v is None for v in [self.fx, self.fy, self.cx, self.cy]) or self.latest_depth is None:
            return
        try:
            pc = PointCloud()
            pc.header = Header()
            pc.header.stamp = header.stamp
            pc.header.frame_id = self.camera_frame_id

            pts = []
            ids = []
            us = []
            vs = []
            visibles = []

            n = len(points)
            for i in range(n):
                vis = bool(visibility[i])
                u = float(points[i, 0])
                v = float(points[i, 1])
                if not vis:
                    # 不可见点跳过（也可改为填充 NaN）
                    continue
                Z = self._depth_at(u, v, ksize=5)
                if Z is None or Z <= 0:
                    continue
                X = (u - self.cx) / self.fx * Z
                Y = (v - self.cy) / self.fy * Z
                p = Point32()
                p.x = float(X)
                p.y = float(Y)
                p.z = float(Z)
                pts.append(p)
                ids.append(float(i))
                us.append(u)
                vs.append(v)
                visibles.append(1.0)

            pc.points = pts

            # 附加通道：id、u、v、visible
            ch_id = ChannelFloat32(name='id', values=ids)
            ch_u = ChannelFloat32(name='u', values=us)
            ch_v = ChannelFloat32(name='v', values=vs)
            ch_vis = ChannelFloat32(name='visible', values=visibles)
            pc.channels = [ch_id, ch_u, ch_v, ch_vis]

            self.pc_pub.publish(pc)
        except Exception as e:
            self.get_logger().warn(f"发布3D点云失败: {e}")

    ############################################
    # 深度图回调
    ############################################
    def depth_callback(self, msg: Image):
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"深度图解码失败: {e}")
            return

        # 统一转换为 float32（米）
        if arr.dtype == np.uint16:
            depth_m = arr.astype(np.float32) * self.depth_scale
        else:
            depth_m = arr.astype(np.float32)

        depth_m = np.clip(depth_m, 0, 1000)  # 简单清理
        self.latest_depth = depth_m

    ############################################
    # 获取某像素邻域深度
    ############################################
    def _depth_at(self, u, v, ksize=5):
        if self.latest_depth is None:
            return None
        H, W = self.latest_depth.shape
        ui = int(round(u))
        vi = int(round(v))
        if ui < 0 or vi < 0 or ui >= W or vi >= H:
            return None

        r = ksize // 2
        region = self.latest_depth[max(0, vi-r):min(H, vi+r+1),
                                   max(0, ui-r):min(W, ui+r+1)]
        vals = region[region > 0]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    ############################################
    # 深度验证和运动预测方法
    ############################################
    def _record_initial_depths(self, points, visibility):
        """记录初始跟踪点的深度信息"""
        if self.latest_depth is None:
            return
        
        self.initial_depths = {}
        self.depth_history = {}
        
        for i, (x, y) in enumerate(points):
            if visibility[i]:
                depth = self._depth_at(x, y)
                if depth is not None and depth < self.background_depth_threshold:
                    self.initial_depths[i] = depth
                    self.depth_history[i] = [depth]

    def _validate_and_correct_tracking(self, points, visibility, frame):
        """
        综合验证和修正跟踪结果
        1. 检测背景跟丢
        2. 检测深度异常变化
        3. 使用运动模型预测修正
        """
        if self.latest_depth is None or self.current_frame is None:
            return points, visibility
        
        corrected_points = points.copy()
        corrected_visibility = visibility.copy()
        h, w = frame.shape[:2]
        
        for i in range(len(points)):
            if not visibility[i]:
                continue
            
            x, y = float(points[i][0]), float(points[i][1])
            current_depth = self._depth_at(x, y)
            
            if current_depth is None:
                continue
            
            # 检查1: 深度是否突然变大（跟丢到背景）
            # 初始深度只用于第一次背景判别
            is_background = current_depth > self.background_depth_threshold
            
            # 检查2: 深度变化是否过大（使用最近N帧的滑动中值作为参考）
            depth_change_ok = True
            reference_depth = None
            
            # 优先使用深度历史的滑动中值
            if i in self.depth_history and len(self.depth_history[i]) >= 3:
                # 使用最近3帧的中值作为参考
                recent_depths = self.depth_history[i][-3:]
                reference_depth = float(np.median(recent_depths))
                depth_change = abs(current_depth - reference_depth)
                depth_change_ok = depth_change <= self.max_depth_change
            elif i in self.depth_history and len(self.depth_history[i]) > 0:
                # 如果历史不足3帧，使用平均值
                reference_depth = np.mean(self.depth_history[i])
                depth_change = abs(current_depth - reference_depth)
                depth_change_ok = depth_change <= self.max_depth_change
            elif i in self.initial_depths:
                # 如果没有历史，使用初始深度（仅用于粗分类）
                reference_depth = self.initial_depths[i]
                # 对于初始帧，放宽限制（允许更大的变化）
                depth_change = abs(current_depth - reference_depth)
                depth_change_ok = depth_change <= self.max_depth_change * 1.5
            
            # 如果检测到问题，尝试修正
            if is_background or not depth_change_ok:
                corrected_point = self._correct_with_motion_prediction(
                    i, x, y, current_depth, frame, is_background
                )
                
                if corrected_point is not None:
                    corrected_points[i] = np.array(corrected_point, dtype=np.float32)
                    # 验证修正后的深度
                    corrected_depth = self._depth_at(corrected_point[0], corrected_point[1])
                    if corrected_depth is not None and corrected_depth < self.background_depth_threshold:
                        # 更新深度历史
                        if i in self.depth_history:
                            self.depth_history[i].append(corrected_depth)
                            # 保持历史长度
                            if len(self.depth_history[i]) > 5:
                                self.depth_history[i].pop(0)
                        self.get_logger().debug(
                            f"点{i+1}已修正: ({x:.1f},{y:.1f})->({corrected_point[0]:.1f},{corrected_point[1]:.1f}), "
                            f"深度: {current_depth:.3f}m->{corrected_depth:.3f}m"
                        )
                    else:
                        # 修正后深度仍异常，标记为不可见
                        corrected_visibility[i] = False
                        self.get_logger().warn(
                            f"点{i+1}修正后深度仍异常，标记为不可见"
                        )
                else:
                    # 无法修正，标记为不可见
                    corrected_visibility[i] = False
                    self.get_logger().warn(
                        f"点{i+1}深度异常({current_depth:.3f}m)且无法修正，标记为不可见"
                    )
            else:
                # 深度正常，更新历史
                if i in self.depth_history:
                    self.depth_history[i].append(current_depth)
                    if len(self.depth_history[i]) > 5:
                        self.depth_history[i].pop(0)
    
        return corrected_points, corrected_visibility

    def _correct_with_motion_prediction(self, point_id, x, y, current_depth, frame, is_background):
        """
        使用运动模型预测修正跟踪点位置
        返回修正后的位置 (x, y) 或 None
        """
        if not self.motion_prediction_enabled:
            return None
        
        if self.previous_points is None or point_id >= len(self.previous_points):
            return None
        
        if self.previous_visibility is None or not self.previous_visibility[point_id]:
            return None
        
        # 计算运动速度
        prev_x, prev_y = float(self.previous_points[point_id][0]), float(self.previous_points[point_id][1])
        dx = x - prev_x
        dy = y - prev_y
        motion_magnitude = np.sqrt(dx*dx + dy*dy)
        
        # 如果运动过大，可能是跟丢，使用更保守的预测
        if motion_magnitude > self.max_motion_step:
            # 限制运动步长
            dx = dx / motion_magnitude * self.max_motion_step
            dy = dy / motion_magnitude * self.max_motion_step
        
        # 预测位置（基于运动方向）
        predicted_x = prev_x + dx
        predicted_y = prev_y + dy
        
        # 目标深度：优先使用最近N帧的滑动中值
        target_depth = None
        if point_id in self.depth_history and len(self.depth_history[point_id]) >= 3:
            # 使用最近3帧的中值
            recent_depths = self.depth_history[point_id][-3:]
            target_depth = float(np.median(recent_depths))
        elif point_id in self.depth_history and len(self.depth_history[point_id]) > 0:
            # 如果历史不足3帧，使用平均值
            target_depth = np.mean(self.depth_history[point_id])
        elif point_id in self.initial_depths:
            # 最后回退到初始深度（仅用于第一次搜索）
            target_depth = self.initial_depths[point_id]
        
        # 在预测位置附近搜索有效点
        h, w = frame.shape[:2]
        best_point = None
        best_score = float('inf')
        
        search_step = max(3, self.search_radius // 8)  # 搜索步长
        
        for sx in range(int(predicted_x - self.search_radius), 
                        int(predicted_x + self.search_radius), search_step):
            for sy in range(int(predicted_y - self.search_radius),
                            int(predicted_y + self.search_radius), search_step):
                if 0 <= sx < w and 0 <= sy < h:
                    d = self._depth_at(sx, sy)
                    if d is not None and d < self.background_depth_threshold:
                        # 计算得分：深度匹配度 + 距离预测位置的距离
                        depth_score = 0
                        if target_depth is not None:
                            depth_score = abs(d - target_depth)
                        
                        dist_score = np.sqrt((sx - predicted_x)**2 + (sy - predicted_y)**2)
                        total_score = depth_score * 0.7 + dist_score * 0.3
                        
                        if total_score < best_score:
                            best_score = total_score
                            best_point = (sx, sy)
        
        return best_point

    ############################################
    # 鼠标点击添加关键点
    ############################################
    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # 如果正在等待粗定位完成，忽略点击
        if getattr(self, "waiting_for_coarse", False):
            self.get_logger().info("正在等待粗定位完成，点击被忽略")
            return

        # 如果还未进行粗定位（coarse_completed==False）且未开始图像跟踪
        if not getattr(self, "coarse_completed", False) and not self.tracking_started:
            try:
                # 发布 2D coarse Keypoint（通知）
                kp_msg = self.Keypoints()
                kp_msg.header = Header()
                kp_msg.header.stamp = self.get_clock().now().to_msg()
                kp_msg.num_keypoints = 1
                from track_on_ros2_msgs.msg import Keypoint
                kp = Keypoint()
                kp.id = 0
                kp.x = float(x)
                kp.y = float(y)
                kp.visible = True
                kp_msg.keypoints.append(kp)
                self.coarse_kp_pub.publish(kp_msg)

                # 计算 3D（若可用）并发布到 tracking/points3d （points3d_tf_to_arm_base_node 订阅）
                Z = self._depth_at(x, y, ksize=5)
                pc = PointCloud()
                pc.header = Header()
                pc.header.stamp = kp_msg.header.stamp
                pc.header.frame_id = self.camera_frame_id
                if Z is not None and all(v is not None for v in [self.fx, self.fy, self.cx, self.cy]):
                    X = (float(x) - self.cx) / self.fx * Z
                    Y = (float(y) - self.cy) / self.fy * Z
                    p = Point32(x=float(X), y=float(Y), z=float(Z))
                    pc.points = [p]
                else:
                    pc.points = []
                # 标记为粗定位点（在 channel 中添加 coarse=1.0）
                try:
                    ch = ChannelFloat32(name='coarse', values=[1.0])
                    pc.channels = [ch]
                except Exception:
                    pass
                try:
                    self.pc_pub.publish(pc)
                except Exception:
                    pass

                self.waiting_for_coarse = True
                self.get_logger().info(f"已发布粗定位点 (u={x}, v={y}, Z={Z}), 等待粗定位完成...")
            except Exception as e:
                self.get_logger().warn(f"发布粗定位点失败: {e}")
            return

        # 否则按原逻辑把点击当作跟踪点选择（用于 tracking/keypoints）
        if not self.tracking_started:
            self.selected_points.append([float(x), float(y)])
            self.get_logger().info(f"添加点 {len(self.selected_points)}: ({x},{y})")

    ############################################
    # 获取工作空间根目录
    ############################################
    def _get_workspace_root(self):
        """获取工作空间根目录（tracking_with_cameara_ws）"""
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        path = current_file_dir
        for _ in range(10):
            if os.path.basename(path) == 'tracking_with_cameara_ws':
                return path
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent
        return None

    ############################################
    # 加载相机内参
    ############################################
    def _load_intrinsics(self, path):
        """
        从文本文件中解析 3x3 K 矩阵（像素单位）。
        期望格式：
        K:
        fx 0 cx
        0 fy cy
        0 0 1
        允许有注释与其它文本，将只解析 K: 后面的三行。
        """
        with open(path, 'r') as f:
            lines = [ln.strip() for ln in f.readlines() if ln.strip()]
        # 找到 'K:' 行
        k_idx = -1
        for i, ln in enumerate(lines):
            if ln.startswith('K'):
                # 允许 'K:' 或 'K :' 等
                if ln.replace(' ', '').startswith('K:'):
                    k_idx = i
                    break
        if k_idx == -1 or k_idx + 3 >= len(lines):
            # fallback：旧的正则方式，但限定只取前三行数字
            nums = []
            for ln in lines:
                if ln.startswith('#'):
                    continue
                nums.extend([float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", ln)])
                if len(nums) >= 9:
                    break
            if len(nums) >= 9:
                fx, _, cx, _, fy, cy, *_ = nums
                return fx, fy, cx, cy
            raise RuntimeError('相机内参文件格式不正确，未找到 K: 3 行')
        # 解析 K: 后三行
        k_rows = []
        for ln in lines[k_idx+1:k_idx+4]:
            vals = [float(x) for x in ln.replace(',', ' ').split() if re.match(r"[-+]?\d*\.\d+|[-+]?\d+", x)]
            if len(vals) != 3:
                raise RuntimeError('K 矩阵每行应有 3 个数')
            k_rows.append(vals)
        fx = k_rows[0][0]
        cx = k_rows[0][2]
        fy = k_rows[1][1]
        cy = k_rows[1][2]
        return fx, fy, cx, cy

############################################
# 主入口
############################################
def main(args=None):
    rclpy.init(args=args)
    node = TrackCameraFrontMinNode()
    try:
        if node.show_interactive_window:
            while rclpy.ok() and not node.should_exit:
                rclpy.spin_once(node, timeout_sec=0.01)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()