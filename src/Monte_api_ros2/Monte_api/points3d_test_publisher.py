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
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    return Rz @ Ry @ Rx


def load_wrist_extrinsic(path: str):
    vals = {}
    with open(path, 'r') as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            if ':' in ln:
                k, v = ln.split(':', 1)
                k = k.strip(); v = v.strip()
                if k in ('joint_r7_wrist_roll','joint_l7_wrist_roll'):
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
    Rt = R.T
    tinv = -Rt @ t
    return Rt, tinv


class Points3DTestPublisher(Node):
    """
    参考 points3d_tf_to_arm_base_node 的逻辑：
    - 周期性向 tracking/points3d 发布固定点 (光学坐标)
    - 同时在本节点内部按与转换节点一致的链路计算并打印：
        源(光学) → 相机(笛卡尔, 可选) → 手腕 → 基座(link_r0_arm_base)
    - 也订阅 tracking/points3d_in_arm_base 用于对照（若转换节点在运行）
    """

    def __init__(self):
        super().__init__('points3d_test_publisher')

        # 固定点参数
        self.declare_parameter('x', 0.2230)
        self.declare_parameter('y', -0.1214)
        self.declare_parameter('z', 0.2500)
        self.declare_parameter('id', 0)
        self.declare_parameter('publish_rate', 5.0)

        # 坐标/话题参数
        self.declare_parameter('input_topic', 'tracking/points3d')
        self.declare_parameter('base_topic', 'tracking/points3d_in_arm_base')
        self.declare_parameter('source_frame', 'right_camera_color_optical_frame')
        self.declare_parameter('wrist_frame', 'joint_r7_wrist_roll')
        self.declare_parameter('target_frame', 'link_r0_arm_base')
        self.declare_parameter('subscribe_base', True)

        # 外参与选项
        self.declare_parameter('wrist_extrinsic_file', '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/joint_r7_wrist_roll.txt')
        self.declare_parameter('invert_extrinsic', False)
        self.declare_parameter('apply_optical_to_camera_rotation', True)

        # RobotLib
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
        self.apply_optical_to_camera_rotation = bool(self.get_parameter('apply_optical_to_camera_rotation').value)

        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value

        # 发布/订阅
        self.pub = self.create_publisher(PointCloud, self.input_topic, 10)
        if self.subscribe_base:
            self.sub_base = self.create_subscription(
                PointCloud, self.base_topic, self.cb_base, qos_profile_sensor_data
            )
        else:
            self.sub_base = None

        # 载入外参
        try:
            R_ext, t_ext = load_wrist_extrinsic(self.wrist_extrinsic_file)
            if self.invert_extrinsic:
                R_ext, t_ext = invert_rt(R_ext, t_ext)
            self.R_src2wrist = R_ext
            self.t_src2wrist = t_ext
            self.get_logger().info(
                f'外参 OK: t=({self.t_src2wrist[0]:.6f},{self.t_src2wrist[1]:.6f},{self.t_src2wrist[2]:.6f})')
        except Exception as e:
            self.get_logger().error(f'解析外参失败: {e}')
            self.R_src2wrist = np.eye(3, dtype=float)
            self.t_src2wrist = np.zeros(3, dtype=float)

        # 连接 RobotLib
        _ensure_robotlib_visible(robot_lib_path)
        try:
            from RobotLib import Robot  # type: ignore
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'导入/连接 RobotLib 失败: {e}')
            self.robot = None

        # 定时器
        period = 1.0 / max(1e-3, self.publish_rate)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f'发布固定点到 {self.input_topic}, frame="{self.source_frame}", '
            f'point=({self.pt_x:.4f}, {self.pt_y:.4f}, {self.pt_z:.4f}), id={self.pt_id}')
        if self.sub_base is not None:
            self.get_logger().info(f'订阅转换后的点: {self.base_topic}')

    def _optical_to_camera(self, pts: np.ndarray) -> np.ndarray:
        if not self.apply_optical_to_camera_rotation:
            return pts
        R_cam_opt = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]], dtype=float)
        return (R_cam_opt @ pts.T).T

    def timer_cb(self):
        # 构造并发布源(光学)点
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source_frame
        msg.points = [Point32(x=self.pt_x, y=self.pt_y, z=self.pt_z)]
        ch = ChannelFloat32(); ch.name = 'id'; ch.values = [float(self.pt_id)]
        msg.channels = [ch]
        self.pub.publish(msg)

        # 在本节点内部做同样的变换链路并打印结果
        src = np.array([[self.pt_x, self.pt_y, self.pt_z]], dtype=float)
        self.get_logger().info(f'[TEST] 源(光学) #{self.pt_id}: X={self.pt_x:.6f} Y={self.pt_y:.6f} Z={self.pt_z:.6f}')

        cam = self._optical_to_camera(src)
        if self.apply_optical_to_camera_rotation:
            x, y, z = cam[0]
            self.get_logger().info(f'[TEST] 源(相机笛卡尔) #{self.pt_id}: X={x:.6f} Y={y:.6f} Z={z:.6f}')
            src_pts = cam
        else:
            src_pts = src

        wrist = (self.R_src2wrist @ src_pts.T).T + self.t_src2wrist.reshape(1, 3)
        wx, wy, wz = wrist[0]
        self.get_logger().info(f'[TEST] 手腕 {self.wrist_frame} #{self.pt_id}: X={wx:.6f} Y={wy:.6f} Z={wz:.6f}')

        if self.robot is not None:
            try:
                ok, tf_base_wrist = self.robot.get_tf_transform(self.target_frame, self.wrist_frame)
                # print(f"get_tf_transform success: {ok}, transform: {tf_base_wrist}")
                if ok and tf_base_wrist is not None and len(tf_base_wrist) >= 7:
                    tx, ty, tz, qx, qy, qz, qw = [float(tf_base_wrist[i]) for i in range(7)]
                    R_base_wrist = quat_to_rot(qx, qy, qz, qw)
                    t_base_wrist = np.array([tx, ty, tz], dtype=float)
                    base = (R_base_wrist @ wrist.T).T + t_base_wrist.reshape(1, 3)
                    bx, by, bz = base[0]
                    self.get_logger().info(f'[TEST] 基座 {self.target_frame} #{self.pt_id}: X={bx:.6f} Y={by:.6f} Z={bz:.6f}')
                else:
                    self.get_logger().warn(f'[TEST] 获取TF失败 {self.target_frame} <- {self.wrist_frame}')
            except Exception as e:
                self.get_logger().warn(f'[TEST] TF异常: {e}')

    def cb_base(self, msg: PointCloud):
        if not msg.points:
            return
        p = msg.points[0]
        self.get_logger().info(f'[SUB BASE] {msg.header.frame_id} #{self.pt_id}: X={p.x:.6f} Y={p.y:.6f} Z={p.z:.6f}')


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
