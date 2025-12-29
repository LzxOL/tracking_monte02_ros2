#!/usr/bin/env python3
"""
ROS2节点：实时摄像头关键点跟踪

功能:
    1. 订阅摄像头图像话题或直接读取摄像头
    2. 通过服务接收初始关键点选择
    3. 实时跟踪关键点并发布结果
    4. 发布可视化图像
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import matplotlib.pyplot as plt

# 添加track_on路径以导入TrackingModule
# 获取工作空间路径（支持开发环境和安装环境）
def find_track_on_path():
    """查找track_on模块路径"""
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 方法1: 从当前文件位置向上查找工作空间
    # 在开发环境: track_on_ros2/track_on_ros2/track_camera_node.py -> src/track_on
    # 在安装环境: install/track_on_ros2/lib/track_on_ros2/track_camera_node.py -> src/track_on
    path = current_file_dir
    for _ in range(10):  # 最多向上查找10层
        # 检查当前路径是否是工作空间根目录
        src_track_on = os.path.join(path, 'src', 'track_on')
        if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
            return src_track_on
        
        # 检查当前路径的src/track_on
        if os.path.basename(path) == 'src':
            track_on = os.path.join(path, 'track_on')
            if os.path.isdir(track_on) and os.path.isfile(os.path.join(track_on, 'tracking_module.py')):
                return track_on
        
        parent = os.path.dirname(path)
        if parent == path:  # 到达根目录
            break
        path = parent
    
    # 方法2: 从环境变量获取工作空间路径
    # AMENT_PREFIX_PATH 或 COLCON_PREFIX_PATH 可能包含工作空间路径
    for env_var in ['AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH']:
        env_path = os.environ.get(env_var, '')
        if env_path:
            # 通常是 install 目录，需要找到 src 目录
            for prefix_path in env_path.split(':'):
                if 'tracking_with_cameara_ws' in prefix_path:
                    # 找到工作空间根目录
                    ws_root = prefix_path[:prefix_path.index('tracking_with_cameara_ws') + len('tracking_with_cameara_ws')]
                    src_track_on = os.path.join(ws_root, 'src', 'track_on')
                    if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
                        return src_track_on
    
    # 方法3: 从当前工作目录查找
    cwd = os.getcwd()
    if 'tracking_with_cameara_ws' in cwd:
        ws_root = cwd[:cwd.index('tracking_with_cameara_ws') + len('tracking_with_cameara_ws')]
        src_track_on = os.path.join(ws_root, 'src', 'track_on')
        if os.path.isdir(src_track_on) and os.path.isfile(os.path.join(src_track_on, 'tracking_module.py')):
            return src_track_on
    
    # 方法4: 使用相对路径从当前文件位置查找（作为最后手段）
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = current_file_dir
    for _ in range(5):
        if os.path.basename(ws_root) == 'tracking_with_cameara_ws':
            known_path = os.path.join(ws_root, 'src', 'track_on')
        if os.path.isdir(known_path) and os.path.isfile(os.path.join(known_path, 'tracking_module.py')):
            return known_path
        ws_root = os.path.dirname(ws_root)
        if ws_root == os.path.dirname(ws_root):
            break
    
    return None

track_on_path = find_track_on_path()
if track_on_path:
    if track_on_path not in sys.path:
        sys.path.insert(0, track_on_path)
    try:
        from tracking_module import TrackingModule
    except ImportError as e:
        print(f"错误: 无法导入 tracking_module")
        print(f"track_on_path: {track_on_path}")
        print(f"tracking_module.py 存在: {os.path.isfile(os.path.join(track_on_path, 'tracking_module.py'))}")
        print(f"当前 sys.path 前5个: {sys.path[:5]}")
        raise
else:
    raise ImportError(
        "无法找到 track_on 模块。请确保：\n"
        "1. track_on 目录位于工作空间的 src 目录下\n"
        "2. 工作空间已正确构建和source\n"
        f"当前文件位置: {os.path.abspath(__file__)}\n"
        f"当前工作目录: {os.getcwd()}"
    )


class TrackCameraNode(Node):
    """ROS2节点：摄像头关键点跟踪"""
    
    def __init__(self):
        super().__init__('track_camera_node')
        
        # 声明参数
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('use_camera_topic', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('show_interactive_window', True)
        
        # 获取参数
        checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        use_camera_topic = self.get_parameter('use_camera_topic').get_parameter_value().bool_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        publish_visualization = self.get_parameter('publish_visualization').get_parameter_value().bool_value
        show_interactive_window = self.get_parameter('show_interactive_window').get_parameter_value().bool_value
        
        # 检查检查点路径
        if not checkpoint_path or not os.path.isfile(checkpoint_path):
            self.get_logger().error(f'检查点文件不存在: {checkpoint_path}')
            raise FileNotFoundError(f'检查点文件不存在: {checkpoint_path}')
        
        # 初始化跟踪模块
        self.get_logger().info(f'正在加载模型检查点: {checkpoint_path}')
        try:
            self.tracker = TrackingModule(checkpoint_path)
            self.get_logger().info('模型加载成功')
        except Exception as e:
            self.get_logger().error(f'加载模型失败: {e}')
            raise
        
        # 初始化状态
        self.bridge = CvBridge()
        self.selected_points = []
        self.tracking_started = False
        self.first_frame_captured = False
        self.frame_count = 0
        self.current_frame = None
        self.colors = self._generate_colors(100)
        self.show_interactive_window = show_interactive_window
        self.window_name = "Camera Tracking - Click points, press SPACE to start, Q to quit"
        self.should_exit = False
        
        # 如果启用交互窗口，创建窗口和鼠标回调
        if self.show_interactive_window:
            cv2.namedWindow(self.window_name)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.get_logger().info('交互窗口已启用')
            self.get_logger().info('使用说明:')
            self.get_logger().info('  1. 在窗口中点击鼠标左键选择要跟踪的关键点')
            self.get_logger().info('  2. 按空格键开始跟踪')
            self.get_logger().info('  3. 按 \'r\' 键重置并重新选择点')
            self.get_logger().info('  4. 按 \'q\' 键退出')
        
        # 如果使用摄像头话题，创建订阅者
        if use_camera_topic:
            self.get_logger().info(f'订阅摄像头话题: {camera_topic}')
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                qos_profile_sensor_data
            )
            self.cap = None
        else:
            # 直接打开摄像头
            self.get_logger().info(f'直接打开摄像头: {camera_id}')
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                self.get_logger().error(f'无法打开摄像头 {camera_id}')
                raise RuntimeError(f'无法打开摄像头 {camera_id}')
            
            # 创建定时器定期读取摄像头
            self.timer = self.create_timer(0.033, self.camera_timer_callback)  # ~30 FPS
        
        # 创建发布者
        try:
            from track_on_ros2_msgs.msg import Keypoints
            self.Keypoints = Keypoints
        except ImportError:
            self.get_logger().error('无法导入 track_on_ros2_msgs，请先构建消息包')
            raise
        
        self.keypoints_pub = self.create_publisher(
            Keypoints,
            'tracking/keypoints',
            10
        )
        
        if publish_visualization:
            self.vis_image_pub = self.create_publisher(
                Image,
                'tracking/visualization',
                10
            )
        else:
            self.vis_image_pub = None
        
        # 创建服务：设置初始关键点
        try:
            from track_on_ros2_srv.srv import SetKeypoints
            self.SetKeypoints = SetKeypoints
        except ImportError:
            self.get_logger().error('无法导入 track_on_ros2_srv，请先构建服务包')
            raise
        
        self.set_keypoints_srv = self.create_service(
            SetKeypoints,
            'tracking/set_keypoints',
            self.set_keypoints_callback
        )
        
        # 创建服务：开始/停止跟踪
        from track_on_ros2_srv.srv import ControlTracking
        self.ControlTracking = ControlTracking
        
        self.control_tracking_srv = self.create_service(
            ControlTracking,
            'tracking/control',
            self.control_tracking_callback
        )
        
        # 创建服务：重置跟踪
        from track_on_ros2_srv.srv import ResetTracking
        self.ResetTracking = ResetTracking
        
        self.reset_tracking_srv = self.create_service(
            ResetTracking,
            'tracking/reset',
            self.reset_tracking_callback
        )
        
        self.get_logger().info('跟踪节点已启动')
        self.get_logger().info('使用服务 /tracking/set_keypoints 设置初始关键点')
        self.get_logger().info('使用服务 /tracking/control 开始/停止跟踪')
        self.get_logger().info('使用服务 /tracking/reset 重置跟踪')
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数，用于选择关键点"""
        if event == cv2.EVENT_LBUTTONDOWN and not self.tracking_started:
            self.selected_points.append([x, y])
            self.get_logger().info(f'已选择点 {len(self.selected_points)}: ({x}, {y})')
    
    def _generate_colors(self, num_colors):
        """生成不同颜色的列表（BGR格式，用于OpenCV）"""
        cmap = plt.cm.get_cmap('tab20', min(num_colors, 20))
        colors = []
        for i in range(num_colors):
            rgba = cmap(i % cmap.N)
            # 转换为BGR格式（OpenCV使用BGR）
            bgr = (int(rgba[2] * 255), int(rgba[1] * 255), int(rgba[0] * 255))
            colors.append(bgr)
        return colors
    
    def image_callback(self, msg):
        """处理订阅的图像消息"""
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'处理图像消息时出错: {e}')
    
    def camera_timer_callback(self):
        """定时器回调：从摄像头读取帧"""
        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = 'camera'
                self.process_frame(frame, header)
            else:
                self.get_logger().warn('无法读取摄像头画面')
    
    def process_frame(self, frame, header):
        """处理一帧图像"""
        self.current_frame = frame.copy()
        display_frame = frame.copy()
        
        # 如果还没有开始跟踪，显示已选择的点
        if not self.tracking_started:
            # 在帧上绘制已选择的点
            for i, point in enumerate(self.selected_points):
                x, y = int(point[0]), int(point[1])
                color = self.colors[i % len(self.colors)]
                cv2.circle(display_frame, (x, y), 8, color, -1)
                cv2.putText(display_frame, f"{i+1}", (x+10, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 显示提示信息
            if len(self.selected_points) > 0:
                if self.show_interactive_window:
                    cv2.putText(display_frame, 
                               f"Selected {len(self.selected_points)} points. Press SPACE to start tracking",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, 
                               f"Selected {len(self.selected_points)} points. Call /tracking/control to start",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                if self.show_interactive_window:
                    cv2.putText(display_frame, 
                               "Click to select points, then press SPACE",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(display_frame, 
                               "Call /tracking/set_keypoints to select points",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 如果已经开始跟踪，进行跟踪并显示结果
        else:
            if not self.first_frame_captured:
                # 初始化跟踪
                if len(self.selected_points) > 0:
                    queries = np.array(self.selected_points, dtype=np.float32)
                    # 转换BGR到RGB
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    try:
                        points, visibility = self.tracker.initialize_tracking(queries, frame_rgb)
                        self.first_frame_captured = True
                        self.get_logger().info(f'跟踪已初始化，共 {len(queries)} 个关键点')
                        # 发布第一帧的结果
                        self.publish_keypoints(points, visibility, header)
                    except Exception as e:
                        self.get_logger().error(f'初始化跟踪时出错: {e}')
                        self.tracking_started = False
                        return
            
            # 跟踪当前帧
            if self.first_frame_captured:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                try:
                    points, visibility = self.tracker.track_next_frame(frame_rgb)
                    
                    # 在帧上绘制跟踪结果
                    for i in range(len(points)):
                        x, y = int(points[i, 0]), int(points[i, 1])
                        color = self.colors[i % len(self.colors)]
                        
                        if visibility[i]:
                            # 可见的点：实心圆
                            cv2.circle(display_frame, (x, y), 8, color, -1)
                            cv2.circle(display_frame, (x, y), 12, color, 2)
                        else:
                            # 不可见的点：空心圆
                            cv2.circle(display_frame, (x, y), 8, color, 2)
                        
                        # 显示点编号
                        cv2.putText(display_frame, f"{i+1}", (x+10, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # 显示帧计数
                    cv2.putText(display_frame, f"Frame: {self.frame_count}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # 发布关键点
                    self.publish_keypoints(points, visibility, header)
                    
                    self.frame_count += 1
                    
                except Exception as e:
                    self.get_logger().error(f'跟踪时出错: {e}')
        
        # 显示交互窗口
        if self.show_interactive_window:
            cv2.imshow(self.window_name, display_frame)
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('收到退出信号 (q键)')
                self.should_exit = True
            elif key == ord(' '):  # 空格键
                if len(self.selected_points) > 0 and not self.tracking_started:
                    self.tracking_started = True
                    self.first_frame_captured = False
                    self.frame_count = 0
                    self.get_logger().info('开始跟踪...')
                elif self.tracking_started:
                    self.get_logger().info('跟踪已在进行中...')
                else:
                    self.get_logger().warn('请先选择至少一个关键点')
            elif key == ord('r'):  # 重置
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
        """发布关键点消息"""
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
        """服务回调：设置初始关键点"""
        if self.tracking_started:
            response.success = False
            response.message = "跟踪已在进行中，请先重置"
            return response
        
        # 检查是否有当前帧
        if self.current_frame is None:
            response.success = False
            response.message = "没有可用的图像帧"
            return response
        
        # 验证关键点坐标
        if len(request.x) != len(request.y):
            response.success = False
            response.message = "x和y坐标数量不匹配"
            return response
        
        # 检查坐标是否在图像范围内
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
        """服务回调：控制跟踪（开始/停止）"""
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
        """服务回调：重置跟踪"""
        if self.tracking_started:
            self.tracker.reset()
            self.tracking_started = False
            self.first_frame_captured = False
            self.selected_points = []
            self.frame_count = 0
            response.success = True
            response.message = "跟踪已重置"
            self.get_logger().info(response.message)
        else:
            response.success = True
            response.message = "跟踪未在进行中，已清除选择的关键点"
            self.selected_points = []
        
        return response
    
    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        if self.show_interactive_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrackCameraNode()
        # 如果启用了交互窗口，需要定期检查退出标志
        if node.show_interactive_window:
            while rclpy.ok() and not node.should_exit:
                rclpy.spin_once(node, timeout_sec=0.01)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

