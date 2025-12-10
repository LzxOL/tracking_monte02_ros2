import os
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
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
    # 归一化
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        return np.eye(3, dtype=float)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    # 旋转矩阵
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
    # ZYX (yaw->pitch->roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    return Rz @ Ry @ Rx


def load_wrist_extrinsic(path: str):
    """从 joint_*_wrist_roll.txt 解析外参，返回 (R_ext, t_ext)。
    约定：该外参表示 T_{wrist<-camera_source}，即把相机(source_frame)坐标下的点
    变换到关节 wrist 坐标系。
    文件格式示例：
    joint_r7_wrist_roll:
        x: ...
        y: ...
        z: ...
        roll: ...
        pitch: ...
        yaw: ...
    """
    try:
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
                    # 顶层 key 跳过（例如 joint_r7_wrist_roll:）
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
    except Exception as e:
        raise RuntimeError(f'解析外参文件失败: {e}')


def invert_rt(R: np.ndarray, t: np.ndarray):
    Rt = R.T
    tinv = -Rt @ t
    return Rt, tinv


class Points3DTFToArmBaseNode(Node):
    """
    订阅 tracking/points3d (sensor_msgs/PointCloud)，
    处理流程：
      1) 要求输入点的 frame_id == source_frame（optical）
      2) 外参 T_{wrist<-source}：p_wrist = R_ext·p_src + t_ext
      3) RobotLib TF T_{base<-wrist}：p_base = R·p_wrist + t
    打印三坐标系下的点，并可选发布到 tracking/points3d_in_arm_base。
    """
    def __init__(self):
        super().__init__('points3d_tf_to_arm_base_node')

        # 参数：RobotLib 连接
        self.declare_parameter('robot_ip', '192.168.22.63:50051')
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            default_lib = os.path.join(get_package_share_directory('Monte_api_ros2'), 'lib')
        except Exception:
            default_lib = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
        self.declare_parameter('robot_lib_path', default_lib)

        # 话题/坐标系参数（右手）
        self.declare_parameter('input_topic', 'tracking/points3d')
        self.declare_parameter('source_frame', 'right_camera_color_optical_frame')
        self.declare_parameter('wrist_frame', 'joint_r7_wrist_roll')
        self.declare_parameter('target_frame', 'link_r0_arm_base')
        self.declare_parameter('publish_transformed', True)
        self.declare_parameter('print_limit', 10)
        # 外参文件 + 方向 + 坐标系约定
        self.declare_parameter('wrist_extrinsic_file', '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/joint_r7_wrist_roll.txt')
        self.declare_parameter('invert_extrinsic', False)  # 若文件给的是 T_{source<-wrist}，则置 True 取逆
        # 若外参以相机笛卡尔坐标（x前,y左,z上）为源，而输入为光学坐标（x右,y下,z前），需先做 optical->camera 固定旋转
        self.declare_parameter('apply_optical_to_camera_rotation', True)

        # 读取参数
        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        self.input_topic = self.get_parameter('input_topic').value
        self.source_frame = self.get_parameter('source_frame').value
        self.wrist_frame = self.get_parameter('wrist_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.publish_transformed = bool(self.get_parameter('publish_transformed').value)
        self.print_limit = int(self.get_parameter('print_limit').value)
        wrist_extrinsic_file = self.get_parameter('wrist_extrinsic_file').value
        self.invert_extrinsic = bool(self.get_parameter('invert_extrinsic').value)
        self.apply_optical_to_camera_rotation = bool(self.get_parameter('apply_optical_to_camera_rotation').value)

        self.get_logger().info(f'Robot IP: {robot_ip}')
        self.get_logger().info(f'RobotLib Path: {robot_lib_path}')
        self.get_logger().info(f'外参文件: {wrist_extrinsic_file}, invert={self.invert_extrinsic}')
        self.get_logger().info(
            f"订阅: {self.input_topic}, 流程: {self.target_frame} <- {self.wrist_frame} <-ext- {self.source_frame}")

        # 载入外参（相机source -> 手腕wrist）
        try:
            R_ext, t_ext = load_wrist_extrinsic(wrist_extrinsic_file)
            if self.invert_extrinsic:
                R_ext, t_ext = invert_rt(R_ext, t_ext)
            self.R_src2wrist, self.t_src2wrist = R_ext, t_ext
            self.get_logger().info(
                f'外参 OK: t=({self.t_src2wrist[0]:.6f},{self.t_src2wrist[1]:.6f},{self.t_src2wrist[2]:.6f})')
        except Exception as e:
            self.get_logger().error(str(e))
            self.R_src2wrist = np.eye(3, dtype=float)
            self.t_src2wrist = np.zeros(3, dtype=float)

        # 导入并连接 RobotLib
        _ensure_robotlib_visible(robot_lib_path)
        try:
            from RobotLib import Robot  # type: ignore
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'导入/连接 RobotLib 失败: {e}')
            self.robot = None

        # 订阅与发布
        self.sub = self.create_subscription(PointCloud, self.input_topic, self.cb_points, 10)
        self.pub = None
        if self.publish_transformed:
            self.pub = self.create_publisher(PointCloud, 'tracking/points3d_in_arm_base', 10)

    def _transform_pts(self, pts: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """批量点变换: p_out = R @ p_in + t"""
        return (R @ pts.T).T + t.reshape(1, 3)

    def _extract_channel(self, msg: PointCloud, name: str):
        try:
            for ch in msg.channels:
                if ch.name == name:
                    return ch.values
        except Exception:
            pass
        return None

    def _log_points(self, pts: np.ndarray, frame_name: str, prefix: str, ids=None, limit: int = 10):
        n = pts.shape[0]
        m = min(limit, n)
        if m <= 0:
            return
        self.get_logger().info(f"{prefix} 坐标系={frame_name} 点(前{m}/{n}个):")
        for i in range(m):
            x, y, z = pts[i]
            tag = f"#{int(ids[i])}" if ids is not None and i < len(ids) else f"#{i}"
            self.get_logger().info(f"  {tag}: X={x:.4f} Y={y:.4f} Z={z:.4f}")

    def cb_points(self, msg: PointCloud):
        if self.robot is None:
            return

        # 收集输入点
        if not msg.points:
            return
        pts = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=float)
        msg_frame = msg.header.frame_id

        # Step0: 要求输入点就是 source_frame（optical）
        if (not msg_frame) or (msg_frame != self.source_frame):
            self.get_logger().warn(f'收到的点云 frame_id={msg_frame}，但预期为 {self.source_frame}，丢弃本帧')
            return

        # 打印 源坐标（source_frame, optical）
        ids = self._extract_channel(msg, 'id')
        self._log_points(pts, self.source_frame, '源(光学)', ids, self.print_limit)

        # 可选：将光学坐标转为相机笛卡尔坐标（x前,y左,z上），再应用外参
        if self.apply_optical_to_camera_rotation:
            # R_cam<-optical (REP-103):
            R_cam_opt = np.array([[0, 0, 1],
                                  [-1, 0, 0],
                                  [0, -1, 0]], dtype=float)
            pts_cam = (R_cam_opt @ pts.T).T
            self._log_points(pts_cam, f'{self.source_frame}_camera', '源(相机笛卡尔)', ids, self.print_limit)
            src_pts = pts_cam
        else:
            src_pts = pts

        # Step1: 使用外参：source -> wrist
        pts_wrist = self._transform_pts(src_pts, self.R_src2wrist, self.t_src2wrist)
        # 打印 手腕坐标（wrist_frame）
        self._log_points(pts_wrist, self.wrist_frame, '手腕', ids, self.print_limit)

        # Step2: 用 RobotLib TF：target <- wrist
        try:
            ok, tf_base_wrist = self.robot.get_tf_transform(self.target_frame, self.wrist_frame)
            if not ok or tf_base_wrist is None or len(tf_base_wrist) < 7:
                self.get_logger().warn(f'获取TF失败 {self.target_frame} <- {self.wrist_frame}')
                return
            tx, ty, tz, qx, qy, qz, qw = [float(tf_base_wrist[i]) for i in range(7)]
            R_base_wrist = quat_to_rot(qx, qy, qz, qw)
            t_base_wrist = np.array([tx, ty, tz], dtype=float)
        except Exception as e:
            self.get_logger().warn(f'TF异常 {self.target_frame} <- {self.wrist_frame}: {e}')
            return

        pts_base = self._transform_pts(pts_wrist, R_base_wrist, t_base_wrist)

        # 打印（基座）
        n_print = min(self.print_limit, pts_base.shape[0])
        if n_print > 0:
            self.get_logger().info(
                f"转换到 {self.target_frame} 坐标系的点(前{n_print}/{pts_base.shape[0]}个):")
            for i in range(n_print):
                x, y, z = pts_base[i]
                self.get_logger().info(f"  #{i}: X={x:.4f} Y={y:.4f} Z={z:.4f}")

        # 发布
        if self.pub is not None:
            out_msg = PointCloud()
            out_msg.header = msg.header
            out_msg.header.frame_id = self.target_frame
            out_msg.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts_base]
            out_msg.channels = msg.channels
            try:
                self.pub.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f"发布转换点云失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Points3DTFToArmBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
