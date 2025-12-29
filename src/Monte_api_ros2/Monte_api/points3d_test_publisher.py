#!/usr/bin/env python3
import os
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32


def _ensure_robotlib_visible(robot_lib_path: str):
    if not robot_lib_path:
        return
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if robot_lib_path not in ld_path.split(':'):
        os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
    if robot_lib_path not in sys.path:
        sys.path.append(robot_lib_path)


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (qx,qy,qz,qw) to rotation matrix"""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        return np.eye(3, dtype=float)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),     2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),         1 - 2*(xx + yy)]
    ], dtype=float)
    return R


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll-pitch-yaw (XYZ intrinsic) to rotation matrix (ROS convention)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rotation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    return Rz @ Ry @ Rx


def load_wrist_extrinsic(path: str):
    """Load camera→wrist extrinsic from YAML-like file"""
    vals = {}
    with open(path, 'r') as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            if ':' in ln:
                k, v = ln.split(':', 1)
                k = k.strip()
                v = v.strip()
                if k in ('joint_r7_wrist_roll', 'joint_l7_wrist_roll'):
                    continue
                try:
                    vals[k] = float(v)
                except ValueError:
                    pass
    tx = float(vals.get('x', 0.0))
    ty = float(vals.get('y', 0.0))
    tz = float(vals.get('z', 0.0))
    roll = float(vals.get('roll', 0.0))
    pitch = float(vals.get('pitch', 0.0))
    yaw = float(vals.get('yaw', 0.0))
    R = rpy_to_rot(roll, pitch, yaw)
    t = np.array([tx, ty, tz], dtype=float)
    return R, t


def invert_rt(R: np.ndarray, t: np.ndarray):
    """Invert rigid transform: [R|t] => [R.T | -R.T @ t]"""
    Rt = R.T
    tinv = -Rt @ t
    return Rt, tinv


class Points3DTestPublisher(Node):
    """
    测试节点：optical → (camera) → wrist → base
    """

    def __init__(self):
        super().__init__('points3d_test_publisher')

        # 声明参数 #0: X=0.0180 Y=0.1162 Z=0.7720
        # self.declare_parameter('x', 0.1534)
        # self.declare_parameter('y', -0.0333)
        # self.declare_parameter('z', 0.6010)
        self.declare_parameter('x', 0.0180)
        self.declare_parameter('y', 0.1162)
        self.declare_parameter('z', 0.7720)
        self.declare_parameter('id', 1)
        self.declare_parameter('publish_rate', 5.0)

        self.declare_parameter('input_topic', 'tracking/points3d')
        self.declare_parameter('base_topic', 'tracking/points3d_in_arm_base')
        self.declare_parameter('source_frame', 'right_camera_color_optical_frame')
        self.declare_parameter('wrist_frame', 'joint_r7_wrist_roll')
        self.declare_parameter('target_frame', 'link_r0_arm_base')
        self.declare_parameter('subscribe_base', True)

        ws_root = self._get_workspace_root()
        default_extrinsic = os.path.join(ws_root, 'config', 'joint_r7_wrist_roll.txt') if ws_root else ''
        self.declare_parameter('wrist_extrinsic_file', default_extrinsic)
        self.declare_parameter('invert_extrinsic', False)
        self.declare_parameter('apply_optical_to_camera_rotation', False)

        try:
            from ament_index_python.packages import get_package_share_directory
            default_lib = os.path.join(get_package_share_directory('Monte_api_ros2'), 'lib')
        except Exception:
            default_lib = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
        self.declare_parameter('robot_ip', '192.168.22.63:50051')
        self.declare_parameter('robot_lib_path', default_lib)

        # 读取参数
        self.pt_x = float(self.get_parameter('x').value)
        self.pt_y = float(self.get_parameter('y').value)
        self.pt_z = float(self.get_parameter('z').value)
        self.pt_id = int(self.get_parameter('id').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.input_topic = self.get_parameter('input_topic').value
        self.base_topic = self.get_parameter('base_topic').value
        self.source_frame = self.get_parameter('source_frame').value
        self.wrist_frame = self.get_parameter('wrist_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.subscribe_base = bool(self.get_parameter('subscribe_base').value)

        self.wrist_extrinsic_file = self.get_parameter('wrist_extrinsic_file').value
        self.invert_extrinsic = bool(self.get_parameter('invert_extrinsic').value)

        # 发布者 / 订阅者
        self.pub = self.create_publisher(PointCloud, self.input_topic, 10)
        if self.subscribe_base:
            self.sub_base = self.create_subscription(
                PointCloud, self.base_topic, self.cb_base, qos_profile_sensor_data
            )
        else:
            self.sub_base = None

        # 载入外参：camera → wrist
        try:
            R_ext, t_ext = load_wrist_extrinsic(self.wrist_extrinsic_file)
            if self.invert_extrinsic:
                R_ext, t_ext = invert_rt(R_ext, t_ext)

            self.R_cam2wrist = R_ext
            self.t_cam2wrist = t_ext
            self.get_logger().info(f'✅ 加载相机→wrist 外参成功: t={self.t_cam2wrist}, R=\n{self.R_cam2wrist}')
        except Exception as e:
            self.get_logger().error(f'❌ 外参加载失败，使用单位变换: {e}')
            self.R_cam2wrist = np.eye(3)
            self.t_cam2wrist = np.zeros(3)

        # RobotLib 连接
        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        _ensure_robotlib_visible(robot_lib_path)

        try:
            from RobotLib import Robot
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('✅ Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'❌ 导入 RobotLib 失败: {e}')
            self.robot = None

        # 定时器
        period = 1.0 / max(1e-3, self.publish_rate)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f'📌 发布固定点到 {self.input_topic}, point=({self.pt_x},{self.pt_y},{self.pt_z})'
        )

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

    def _optical_to_camera(self, pts: np.ndarray) -> np.ndarray:
        # Astra / RealSense optical == camera? Assume identity
        return pts

    def timer_cb(self):
        # 发布 PointCloud（用于可视化）
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source_frame
        msg.points = [Point32(x=self.pt_x, y=self.pt_y, z=self.pt_z)]
        ch = ChannelFloat32()
        ch.name = 'id'
        ch.values = [float(self.pt_id)]
        msg.channels = [ch]
        self.pub.publish(msg)

        # Step 0: 光学系（假设 = 相机系）
        p_optical = np.array([self.pt_x, self.pt_y, self.pt_z])
        self.get_logger().info(f'[TEST] 光学/相机系: {p_optical}')

        # Step 1: camera → wrist
        p_wrist = self.R_cam2wrist @ p_optical + self.t_cam2wrist
        self.get_logger().info(f'[TEST] 手腕系 (wrist): {p_wrist}')

        # Step 2: wrist → base
        if self.robot is not None:
            ok, tf_bw = self.robot.get_tf_transform(self.target_frame, self.wrist_frame)
            if ok and tf_bw is not None and len(tf_bw) >= 7:
                tx, ty, tz, qw, qx, qy, qz = [float(tf_bw[i]) for i in range(7)]
                R_wb = quat_to_rot(qx, qy, qz, qw)  # wrist → base
                t_wb = np.array([tx, ty, tz])
                p_base = R_wb @ p_wrist + t_wb
                self.get_logger().info(f'[TEST] 基座系 (base): {p_base}')
            else:
                self.get_logger().warn('[TEST] 获取 TF 失败')
        else:
            self.get_logger().warn('[TEST] Robot 未连接')

    def cb_base(self, msg: PointCloud):
        if not msg.points:
            return
        p = msg.points[0]
        self.get_logger().info(f'[SUB BASE] {msg.header.frame_id}: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')


def main(args=None):
    rclpy.init(args=args)
    node = Points3DTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()