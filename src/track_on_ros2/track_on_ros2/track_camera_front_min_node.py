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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import re

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

    # fallback
    known_path = '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/track_on'
    if os.path.isdir(known_path):
        return known_path

    return None


track_on_path = find_track_on_path()
if track_on_path and track_on_path not in sys.path:
    sys.path.insert(0, track_on_path)

from tracking_module import TrackingModule  # noqa: E402


#########################################
# 主节点类
#########################################

class TrackCameraFrontMinNode(Node):

    def __init__(self):
        super().__init__('track_camera_front_min_node')

        ######## 参数 ########
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('show_interactive_window', True)

        # 3D 相关
        # 默认从指定文件读取相机内参（像素单位）
        self.declare_parameter('intrinsics_file', '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/camera_rhead_front_intrinsics_02_10.txt')
        self.declare_parameter('depth_topic', '/right/depth/stream')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('print_3d', True)
        self.declare_parameter('print_3d_interval', 1)

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

        #######################
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
        self.get_logger().info("跟踪已重置")

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
            pc.header.frame_id = "right_camera_color_optical_frame"

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
    # 鼠标点击添加关键点
    ############################################
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.tracking_started:
            self.selected_points.append([x, y])
            self.get_logger().info(f"添加点 {len(self.selected_points)}: ({x},{y})")

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
