#!/usr/bin/env python3
"""
ROS2节点：前端摄像头颜色图像话题关键点跟踪

订阅 /camera_head_front/color/video 话题，实现点击选择关键点并跟踪

    track_camera_front_node_12_3.py保存版本
    更新日期：2025-12-03
    能够实现3D点返回以及跟踪
    
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import re

# 查找并导入 track_on/tracking_module

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
    for env_var in ['AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH']:
        env_path = os.environ.get(env_var, '')
        if env_path:
            for prefix_path in env_path.split(':'):
                if 'tracking_with_cameara_ws' in prefix_path:
                    ws_root = prefix_path[:prefix_path.index('tracking_with_cameara_ws') + len('tracking_with_cameara_ws')]
                    src_track_on = os.path.join(ws_root, 'src', 'track_on')
                    if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
                        return src_track_on
    cwd = os.getcwd()
    if 'tracking_with_cameara_ws' in cwd:
        ws_root = cwd[:cwd.index('tracking_with_cameara_ws') + len('tracking_with_cameara_ws')]
        src_track_on = os.path.join(ws_root, 'src', 'track_on')
        if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
            return src_track_on
    known_path = '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/track_on'
    if os.path.isdir(known_path) and os.path.isfile(os.path.join(known_path, 'tracking_module.py')):
        return known_path
    return None

track_on_path = find_track_on_path()
if track_on_path and track_on_path not in sys.path:
    sys.path.insert(0, track_on_path)

from tracking_module import TrackingModule  # 可能抛出 ImportError，由上层捕获


class TrackCameraFrontNode(Node):
    """ROS2节点：订阅前端摄像头颜色图像话题进行关键点跟踪"""

    def __init__(self):
        super().__init__('track_camera_front_node')

        # 参数
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('camera_topic', '/camera_head_front/color/video')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('show_interactive_window', True)
        # 新增：3D计算相关参数
        self.declare_parameter('intrinsics_file', '')
        self.declare_parameter('depth_topic', '/camera_head_front/depth/stream')
        self.declare_parameter('camera_info_topic', '/camera_head_front/color/camera_info')
        self.declare_parameter('print_3d', True)
        self.declare_parameter('print_3d_interval', 1)  # 每N帧打印一次3D坐标
        self.declare_parameter('depth_scale', 0.001)  # 16UC1常用mm->m比例

        checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        publish_visualization = self.get_parameter('publish_visualization').get_parameter_value().bool_value
        show_interactive_window = self.get_parameter('show_interactive_window').get_parameter_value().bool_value
        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        print_3d = self.get_parameter('print_3d').get_parameter_value().bool_value
        print_3d_interval = self.get_parameter('print_3d_interval').get_parameter_value().integer_value
        depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value

        if not checkpoint_path or not os.path.isfile(checkpoint_path):
            self.get_logger().error(f'检查点文件不存在: {checkpoint_path}')
            raise FileNotFoundError(f'检查点文件不存在: {checkpoint_path}')

        # 初始化跟踪器
        self.get_logger().info(f'正在加载模型检查点: {checkpoint_path}')
        self.tracker = TrackingModule(checkpoint_path)
        self.get_logger().info('模型加载成功')

        # 3D参数与内参加载
        self.print_3d = print_3d
        self.print_3d_interval = max(1, print_3d_interval)  # 至少为1
        self.depth_scale = float(depth_scale) if depth_scale else 0.001
        self.print_3d_counter = 0  # 用于控制打印频率
        self.fx = self.fy = self.cx = self.cy = None
        if intrinsics_file and os.path.isfile(intrinsics_file):
            try:
                self.fx, self.fy, self.cx, self.cy = self._load_intrinsics(intrinsics_file)
                self.get_logger().info(f"已加载相机内参 fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            except Exception as e:
                self.get_logger().warn(f'加载内参失败: {e}')
        else:
            # 尝试从 CameraInfo 动态获取
            self.get_logger().warn('未提供intrinsics_file，尝试从 CameraInfo 话题获取相机内参')
            self.camera_info_ready = False
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self.camera_info_callback,
                qos_profile_sensor_data
            )

        # 深度帧订阅与缓存
        self.latest_depth = None  # 以米为单位的float32数组
        self.latest_depth_shape = None
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f'订阅深度话题: {depth_topic} (用于3D计算)')

        # 状态
        self.bridge = CvBridge()
        self.selected_points = []
        self.tracking_started = False
        self.first_frame_captured = False
        self.frame_count = 0
        self.current_frame = None
        self.colors = self._generate_colors(100)
        self.show_interactive_window = show_interactive_window
        self.window_name = "Front Camera Tracking - Click points, SPACE start, Q quit"
        self.should_exit = False

        # 订阅前端摄像头颜色图像
        self.get_logger().info(f'订阅前端摄像头颜色图像话题: {camera_topic}')
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # 发布关键点
        try:
            from track_on_ros2_msgs.msg import Keypoints
            self.Keypoints = Keypoints
        except ImportError:
            self.get_logger().error('无法导入 track_on_ros2_msgs，请先构建并 source 工作空间')
            raise
        self.keypoints_pub = self.create_publisher(self.Keypoints, 'tracking/keypoints', 10)

        # 可视化发布
        if publish_visualization:
            self.vis_image_pub = self.create_publisher(Image, 'tracking/visualization', 10)
        else:
            self.vis_image_pub = None

        # 服务
        try:
            from track_on_ros2_srv.srv import SetKeypoints, ControlTracking, ResetTracking
        except ImportError:
            self.get_logger().error('无法导入 track_on_ros2_srv，请先构建并 source 工作空间')
            raise
        self.set_keypoints_srv = self.create_service(SetKeypoints, 'tracking/set_keypoints', self.set_keypoints_callback)
        self.control_tracking_srv = self.create_service(ControlTracking, 'tracking/control', self.control_tracking_callback)
        self.reset_tracking_srv = self.create_service(ResetTracking, 'tracking/reset', self.reset_tracking_callback)

        # 交互窗口
        if self.show_interactive_window:
            cv2.namedWindow(self.window_name)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.get_logger().info('交互窗口已启用: 点击添加点, 空格开始, r重置, q退出')

        self.get_logger().info('前端摄像头跟踪节点已启动')

    def camera_info_callback(self, msg: CameraInfo):
        """从 CameraInfo 设置内参，一次性获取后注销订阅"""
        try:
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self.camera_info_ready = True
            self.get_logger().info(f"从CameraInfo获取内参 fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            # 注销订阅，避免重复处理
            if hasattr(self, 'camera_info_sub') and self.camera_info_sub is not None:
                try:
                    self.destroy_subscription(self.camera_info_sub)
                except Exception:
                    pass
                self.camera_info_sub = None
        except Exception as e:
            self.get_logger().warn(f"解析CameraInfo失败: {e}")

    def _load_intrinsics(self, path):
        """从ost.txt或类似格式文件中读取内参，返回 fx, fy, cx, cy"""
        fx = fy = cx = cy = None
        with open(path, 'r') as f:
            lines = [ln.strip() for ln in f.readlines() if ln.strip()]
        # 先尝试解析K矩阵
        try:
            # 找到以 'K:' 开头的行
            idx = next(i for i,l in enumerate(lines) if l.startswith('K:'))
            # 接下来的三行是矩阵
            row0 = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", lines[idx+1])]
            row1 = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", lines[idx+2])]
            # row2 = lines[idx+3]
            fx = row0[0]; cx = row0[2]
            fy = row1[1]; cy = row1[2]
            return fx, fy, cx, cy
        except Exception:
            pass
        # 再尝试解析YAML风格: camera_matrix: data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        try:
            for i,l in enumerate(lines):
                if 'camera_matrix' in l and 'data' in l:
                    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", l)]
                    if len(nums) >= 9:
                        fx = nums[0]; cx = nums[2]; fy = nums[4]; cy = nums[5]
                        return fx, fy, cx, cy
                if re.match(r"\s*data\s*:\s*\[", l):
                    # 下一行或本行包含数据
                    nums = []
                    j = i
                    while j < len(lines) and ']' not in lines[j]:
                        nums += [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", lines[j])]
                        j += 1
                    if j < len(lines):
                        nums += [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", lines[j])]
                    if len(nums) >= 9:
                        fx = nums[0]; cx = nums[2]; fy = nums[4]; cy = nums[5]
                        return fx, fy, cx, cy
        except Exception:
            pass
        raise RuntimeError('无法从内参文件解析出 fx, fy, cx, cy')

    def depth_callback(self, msg: Image):
        """缓存最近的深度图（单位: 米）。
        兼容以下情况：
        - 16UC1/mono16: 通常为毫米，需要乘以 depth_scale (默认0.001)
        - 32FC1: 可能是米，也可能是毫米（部分驱动），通过中值启发式判断
        - 其他编码: 依据数组dtype进行处理
        """
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            enc = msg.encoding.lower() if hasattr(msg, 'encoding') and isinstance(msg.encoding, str) else ''

            depth_m = None

            # 特殊情况：Y16有时被解码为两个uint8通道
            if arr.ndim == 3 and arr.shape[2] == 2 and arr.dtype == np.uint8:
                # 组合为uint16（小端序）
                depth_u16 = (arr[:, :, 1].astype(np.uint16) << 8) | arr[:, :, 0].astype(np.uint16)
                depth_m = depth_u16.astype(np.float32) * float(self.depth_scale)
            # 情况1：uint16 → 视为毫米
            elif arr.dtype == np.uint16 or ('16u' in enc and 'c1' in enc) or enc in ('y16', 'y16c1'):
                depth_m = arr.astype(np.float32) * float(self.depth_scale)
            else:
                # 转成float32便于后续处理
                arr_f = arr.astype(np.float32)
                if enc in ('32fc1', 'type_32fc1'):
                    # 启发式判断单位：如果中值>20，极可能是毫米
                    med = float(np.nanmedian(arr_f)) if arr_f.size > 0 else 0.0
                    if np.isfinite(med) and med > 20.0:
                        depth_m = arr_f * float(self.depth_scale)
                    else:
                        depth_m = arr_f  # 认为已是米
                else:
                    # 未知编码：再用启发式
                    med = float(np.nanmedian(arr_f)) if arr_f.size > 0 else 0.0
                    if np.isfinite(med) and med > 20.0:
                        depth_m = arr_f * float(self.depth_scale)
                    else:
                        depth_m = arr_f

            # 极端值/无效值处理
            depth_m = np.where(np.isfinite(depth_m), depth_m, 0).astype(np.float32)

            # 兜底：若深度中值异常大（>20m且<10000），自动按毫米→米缩放
            try:
                med_m = float(np.nanmedian(depth_m)) if depth_m.size > 0 else 0.0
                if np.isfinite(med_m) and med_m > 20.0 and med_m < 10000.0:
                    self.get_logger().warn(
                        f"检测到异常深度中值={med_m:.3f}m，自动按毫米→米缩放(×0.001)。如需关闭，请显式设置depth_scale并确保编码正确。"
                    )
                    depth_m *= 0.001
            except Exception:
                pass

            self.latest_depth = depth_m
            self.latest_depth_shape = depth_m.shape
        except Exception as e:
            self.get_logger().error(f'处理深度消息失败: {e}')

    def _depth_at(self, u, v, ksize=5):
        """返回像素(u,v)邻域内的中值深度(米)。无效返回None
        
        Args:
            u: 像素x坐标
            v: 像素y坐标
            ksize: 邻域大小（奇数）
            
        Returns:
            深度值（米），或None（无效）
        """
        if self.latest_depth is None:
            return None
        
        # 处理3维或2维深度图
        depth_shape = self.latest_depth.shape
        if len(depth_shape) == 3:
            h, w, c = depth_shape
            depth_2d = self.latest_depth[:, :, 0]  # 取第一个通道
        elif len(depth_shape) == 2:
            h, w = depth_shape
            depth_2d = self.latest_depth
        else:
            return None
        
        # 如颜色与深度分辨率不同，则将颜色坐标按比例映射到深度分辨率
        if self.current_frame is not None:
            ch, cw = self.current_frame.shape[:2]
            if cw > 0 and ch > 0 and (cw != w or ch != h):
                u = float(u) * (w / float(cw))
                v = float(v) * (h / float(ch))
        
        u_i = int(round(u))
        v_i = int(round(v))
        
        # 检查坐标是否在图像范围内
        if u_i < 0 or v_i < 0 or u_i >= w or v_i >= h:
            return None
        
        # 提取邻域
        r = ksize // 2
        u0 = max(0, u_i - r)
        u1 = min(w, u_i + r + 1)
        v0 = max(0, v_i - r)
        v1 = min(h, v_i + r + 1)
        
        patch = depth_2d[v0:v1, u0:u1].reshape(-1)
        
        # 过滤有效的深度值
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        
        if patch.size == 0:
            return None
        
        # 返回中值深度
        return float(np.median(patch))

    def _get_3d_coords(self, points, visibility):
        """计算关键点的3D坐标（相机坐标系，单位: 米）
        
        Args:
            points: 关键点2D坐标数组 (N, 2)
            visibility: 关键点可见性数组 (N,)
            
        Returns:
            list: 包含3D坐标信息的字典列表
                [{'id': i, 'visible': bool, 'x': float, 'y': float, 'z': float, 
                  'u': float, 'v': float, 'valid': bool}, ...]
        """
        coords_3d = []
        
        for i in range(len(points)):
            info = {
                'id': i,
                'visible': bool(visibility[i]),
                'u': float(points[i, 0]),
                'v': float(points[i, 1]),
                'x': None,
                'y': None,
                'z': None,
                'valid': False
            }
            
            if not info['visible']:
                coords_3d.append(info)
                continue
            
            # 检查内参和深度图是否就绪
            if any(v is None for v in [self.fx, self.fy, self.cx, self.cy]):
                coords_3d.append(info)
                continue
            
            if self.latest_depth is None:
                coords_3d.append(info)
                continue
            
            # 获取深度值
            Z = self._depth_at(info['u'], info['v'], ksize=5)
            if Z is None or Z <= 0:
                coords_3d.append(info)
                continue
            
            # 计算3D坐标
            X = (info['u'] - self.cx) / self.fx * Z
            Y = (info['v'] - self.cy) / self.fy * Z
            
            info['x'] = X
            info['y'] = Y
            info['z'] = Z
            info['valid'] = True
            
            coords_3d.append(info)
        
        return coords_3d

    def _compute_and_log_3d(self, points, visibility):
        """根据深度与内参，计算并打印3D坐标（以相机坐标系，单位: 米）"""
        if not self.print_3d:
            return
        
        # 控制打印频率
        self.print_3d_counter += 1
        if self.print_3d_counter % self.print_3d_interval != 0:
            return
        
        # 获取3D坐标
        coords_3d = self._get_3d_coords(points, visibility)
        
        if not coords_3d:
            return
        
        # 检查是否有有效的3D坐标
        has_valid = any(c['valid'] for c in coords_3d)
        if not has_valid and not any(c['visible'] for c in coords_3d):
            return
        
        # 打印日志
        self.get_logger().info("╔═══════════════════════════════════════════════════════════╗")
        self.get_logger().info("║           关键点3D坐标 (相机坐标系，单位: 米)              ║")
        self.get_logger().info("╠═══════════════════════════════════════════════════════════╣")
        
        for coord in coords_3d:
            if not coord['visible']:
                self.get_logger().info(f"║ 点{coord['id']+1}: 不可见                                          ║")
            elif not coord['valid']:
                self.get_logger().info(f"║ 点{coord['id']+1}: 深度无效 (像素: {coord['u']:.1f}, {coord['v']:.1f})           ║")
            else:
                self.get_logger().info(
                    f"║ 点{coord['id']+1}: X={coord['x']:7.4f}m Y={coord['y']:7.4f}m Z={coord['z']:7.4f}m ║"
                )
                self.get_logger().info(
                    f"║      像素坐标: u={coord['u']:7.1f}, v={coord['v']:7.1f}                    ║"
                )
        
        self.get_logger().info("╠═══════════════════════════════════════════════════════════╣")
        self.get_logger().info(
            f"║ 相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}  ║"
        )
        self.get_logger().info("╚═══════════════════════════════════════════════════════════╝")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.tracking_started:
            self.selected_points.append([x, y])
            self.get_logger().info(f'已选择点 {len(self.selected_points)}: ({x}, {y})')

    def _generate_colors(self, num_colors):
        cmap = plt.cm.get_cmap('tab20', min(num_colors, 20))
        colors = []
        for i in range(num_colors):
            rgba = cmap(i % cmap.N)
            bgr = (int(rgba[2] * 255), int(rgba[1] * 255), int(rgba[0] * 255))
            colors.append(bgr)
        return colors

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'处理图像消息时出错: {e}')

    def process_frame(self, frame, header: Header):
        self.current_frame = frame.copy()
        display_frame = frame.copy()

        if not self.tracking_started:
            # 显示已选择的点
            for i, point in enumerate(self.selected_points):
                x, y = int(point[0]), int(point[1])
                color = self.colors[i % len(self.colors)]
                cv2.circle(display_frame, (x, y), 8, color, -1)
                cv2.putText(display_frame, f"{i+1}", (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            tip = (
                f"Selected {len(self.selected_points)} points. Press SPACE to start tracking"
                if self.show_interactive_window else
                f"Selected {len(self.selected_points)} points. Call /tracking/control to start"
            ) if len(self.selected_points) > 0 else (
                "Click to select points, then press SPACE" if self.show_interactive_window else
                "Call /tracking/set_keypoints to select points"
            )
            cv2.putText(display_frame, tip, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if len(self.selected_points)>0 else (0,255,255), 2)
        else:
            # 跟踪模式
            if not self.first_frame_captured and len(self.selected_points) > 0:
                # 初始化跟踪
                queries = np.array(self.selected_points, dtype=np.float32)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                try:
                    init_out = self.tracker.initialize_tracking(queries, frame_rgb)
                    # 兼容返回超过两个元素的情况，只取前两个(points, visibility)
                    if isinstance(init_out, (list, tuple)) and len(init_out) >= 2:
                        points = np.array(init_out[0], dtype=np.float32)
                        visibility = np.array(init_out[1], dtype=bool)
                    else:
                        points, visibility = init_out
                    self.first_frame_captured = True
                    self.get_logger().info(f'跟踪已初始化，共 {len(queries)} 个关键点')
                    self.publish_keypoints(points, visibility, header)
                    # 计算并打印3D坐标
                    self._compute_and_log_3d(points, visibility)
                except Exception as e:
                    self.get_logger().error(f'初始化跟踪时出错: {e}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())
                    self.tracking_started = False
                    return
            if self.first_frame_captured:
                # 跟踪下一帧
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                try:
                    out = self.tracker.track_next_frame(frame_rgb)
                    if isinstance(out, (list, tuple)) and len(out) >= 2:
                        points = np.array(out[0], dtype=np.float32)
                        visibility = np.array(out[1], dtype=bool)
                    else:
                        points, visibility = out
                    # 绘制跟踪结果
                    for i in range(len(points)):
                        x, y = int(points[i, 0]), int(points[i, 1])
                        color = self.colors[i % len(self.colors)]
                        if visibility[i]:
                            cv2.circle(display_frame, (x, y), 8, color, -1)
                            cv2.circle(display_frame, (x, y), 12, color, 2)
                        else:
                            cv2.circle(display_frame, (x, y), 8, color, 2)
                        cv2.putText(display_frame, f"{i+1}", (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(display_frame, f"Frame: {self.frame_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    self.publish_keypoints(points, visibility, header)
                    # 计算并打印3D坐标（跟踪阶段）
                    self._compute_and_log_3d(points, visibility)
                    self.frame_count += 1
                except Exception as e:
                    self.get_logger().error(f'跟踪时出错: {e}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())

        # 显示窗口并处理键盘输入
        if self.show_interactive_window:
            cv2.imshow(self.window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('收到退出信号 (q键)')
                self.should_exit = True
            elif key == ord(' '):
                if len(self.selected_points) > 0 and not self.tracking_started:
                    self.tracking_started = True
                    self.first_frame_captured = False
                    self.frame_count = 0
                    self.get_logger().info('开始跟踪...')
                elif self.tracking_started:
                    self.get_logger().info('跟踪已在进行中...')
                else:
                    self.get_logger().warn('请先选择至少一个关键点')
            elif key == ord('r'):
                if self.tracking_started:
                    self.tracker.reset()
                self.tracking_started = False
                self.first_frame_captured = False
                self.selected_points = []
                self.frame_count = 0
                self.get_logger().info('已重置，请重新选择关键点')

        # 发布可视化图像
        if self.vis_image_pub is not None:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding='bgr8')
                vis_msg.header = header
                self.vis_image_pub.publish(vis_msg)
            except Exception as e:
                self.get_logger().error(f'发布可视化图像时出错: {e}')

    def publish_keypoints(self, points, visibility, header):
        from track_on_ros2_msgs.msg import Keypoint
        keypoints_msg = self.Keypoints()
        keypoints_msg.header = header
        keypoints_msg.num_keypoints = len(points)
        for i in range(len(points)):
            kp = Keypoint()
            kp.id = i
            kp.x = float(points[i, 0])
            kp.y = float(points[i, 1])
            kp.visible = bool(visibility[i])
            keypoints_msg.keypoints.append(kp)
        self.keypoints_pub.publish(keypoints_msg)

    def set_keypoints_callback(self, request, response):
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
            response.message = "x和y坐标数量不匹配"
            return response
        h, w = self.current_frame.shape[:2]
        self.selected_points = []
        for i in range(len(request.x)):
            x, y = request.x[i], request.y[i]
            if 0 <= x < w and 0 <= y < h:
                self.selected_points.append([x, y])
            else:
                self.get_logger().warn(f'关键点 ({x}, {y}) 超出图像范围 ({w}, {h})，已忽略')
        if len(self.selected_points) == 0:
            response.success = False
            response.message = "没有有效的关键点"
            return response
        response.success = True
        response.message = f"已设置 {len(self.selected_points)} 个关键点"
        self.get_logger().info(response.message)
        return response

    def control_tracking_callback(self, request, response):
        if request.command == "start":
            if len(self.selected_points) == 0:
                response.success = False
                response.message = "请先设置关键点"
                return response
            if self.tracking_started:
                response.success = False
                response.message = "跟踪已在进行中"
                return response
            self.tracking_started = True
            self.first_frame_captured = False
            self.frame_count = 0
            response.success = True
            response.message = "跟踪已开始"
            self.get_logger().info(response.message)
        elif request.command == "stop":
            if not self.tracking_started:
                response.success = False
                response.message = "跟踪未在进行中"
                return response
            self.tracking_started = False
            response.success = True
            response.message = "跟踪已停止"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"未知命令: {request.command}，支持的命令: start, stop"
        return response

    def reset_tracking_callback(self, request, response):
        if self.tracking_started:
            self.tracker.reset()
        self.tracking_started = False
        self.first_frame_captured = False
        self.selected_points = []
        self.frame_count = 0
        response.success = True
        response.message = "跟踪已重置"
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        if self.show_interactive_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrackCameraFrontNode()
        if node.show_interactive_window:
            while rclpy.ok() and not node.should_exit:
                rclpy.spin_once(node, timeout_sec=0.01)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

