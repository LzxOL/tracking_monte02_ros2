import os
import sys
import math
import time
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
      2) 外参 T_{wrist<-camera}：p_wrist = R_ext·p_camera + t_ext
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
        # 粗定位控制参数
        self.declare_parameter('enable_coarse_move', True)
        self.declare_parameter('component_type', 2)  # 1: 左臂  2: 右臂
        self.declare_parameter('distance_stop', 0.5)  # m
        self.declare_parameter('step_size', 0.10)     # 每步前进距离 m
        self.declare_parameter('cmd_interval', 0.3)   # 连续set_arm_position发送最小间隔 s（当wait=False）
        self.declare_parameter('max_speed', 0.10)     # m/s
        self.declare_parameter('max_acc', 0.50)       # m/s^2
        self.declare_parameter('use_wait', False)     # set_arm_position 的 wait
        self.declare_parameter('target_id', -1)       # 若>=0，则优先匹配channel 'id'
        # 粗定位前：末端左右转（关节角）预调整
        self.declare_parameter('servo_yaw_enabled', True)
        self.declare_parameter('servo_yaw_gain', 0.8)          # rad/m，基于基座系y偏差
        self.declare_parameter('servo_yaw_max', 0.35)          # rad，单次最大偏航
        self.declare_parameter('servo_cmd_interval', 0.5)      # s，最小发送间隔
        self.declare_parameter('servo_speed', 0.1)             # set_arm_servo_angle 的 speed
        self.declare_parameter('servo_acc', 1.0)               # set_arm_servo_angle 的 acc
        self.declare_parameter(
            'servo_base_positions',
            [-0.6544687747955322, 0.1876031905412674, -0.01836462691426277,
             1.8717538118362427, 0.16210661828517914, 0.45163217186927795, -0.23318371176719666]
        )
        # 外参文件 + 方向 + 坐标系约定
        self.declare_parameter('wrist_extrinsic_file', '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/joint_r7_wrist_roll.txt')
        self.declare_parameter('invert_extrinsic', False)  # 若文件给的是 T_{source<-wrist}，则置 True 取逆
        # 若外参以相机笛卡尔坐标（x前,y左,z上）为源，而输入为光学坐标（x右,y下,z前），需先做 optical->camera 固定旋转
        self.declare_parameter('apply_optical_to_camera_rotation', False)  # 与参考代码一致，默认 False

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

        # 载入外参（相机 -> 手腕）
        try:
            R_ext, t_ext = load_wrist_extrinsic(wrist_extrinsic_file)
            if self.invert_extrinsic:
                R_ext, t_ext = invert_rt(R_ext, t_ext)
            self.R_cam2wrist = R_ext
            self.t_cam2wrist = t_ext
            self.get_logger().info(
                f'外参 OK: t=({self.t_cam2wrist[0]:.6f},{self.t_cam2wrist[1]:.6f},{self.t_cam2wrist[2]:.6f})')
        except Exception as e:
            self.get_logger().error(str(e))
            self.R_cam2wrist = np.eye(3, dtype=float)
            self.t_cam2wrist = np.zeros(3, dtype=float)

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

        # ---- 粗定位控制相关 ----
        self.enable_coarse_move = bool(self.get_parameter('enable_coarse_move').value)

        # >>> 新增：安全与失败计数 <<<
        self._last_log_ts = 0.0
        self._move_fail_count = 0
        self._max_fail_count = 3  # 连续失败3次就暂停

        self.component_type = int(self.get_parameter('component_type').value)
        self.distance_stop = float(self.get_parameter('distance_stop').value)
        self.step_size = float(self.get_parameter('step_size').value)
        self.cmd_interval = float(self.get_parameter('cmd_interval').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acc = float(self.get_parameter('max_acc').value)
        self.use_wait = bool(self.get_parameter('use_wait').value)
        self.target_id = int(self.get_parameter('target_id').value)
        self.servo_yaw_enabled = bool(self.get_parameter('servo_yaw_enabled').value)
        self.servo_yaw_gain = float(self.get_parameter('servo_yaw_gain').value)
        self.servo_yaw_max = float(self.get_parameter('servo_yaw_max').value)
        self.servo_cmd_interval = float(self.get_parameter('servo_cmd_interval').value)
        self.servo_speed = float(self.get_parameter('servo_speed').value)
        self.servo_acc = float(self.get_parameter('servo_acc').value)
        self.servo_base_positions = list(self.get_parameter('servo_base_positions').value)

        # 最新目标点（Base系）与平滑缓存
        self.target_point = None
        self.last_cmd_ts = 0.0
        self.cached_quat_wxyz = None  # 保持当前姿态
        self.last_servo_cmd_ts = 0.0

        if self.enable_coarse_move:
            self.timer_ctrl = self.create_timer(0.05, self.control_loop)  # 20Hz
            self.get_logger().info(f"粗定位控制已启用: stop={self.distance_stop}m, step={self.step_size}m, wait={self.use_wait}")

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

        if not msg.points:
            return

        # Step 0: 提取光学坐标系下的点
        pts_optical = []
        for p in msg.points:
            pts_optical.append(np.array([p.x, p.y, p.z], dtype=float))
        msg_frame = msg.header.frame_id

        if (not msg_frame) or (msg_frame != self.source_frame):
            self.get_logger().warn(f'收到的点云 frame_id={msg_frame}，但预期为 {self.source_frame}，丢弃本帧')
            return

        ids = self._extract_channel(msg, 'id')
        # self._log_points(np.array(pts_optical), self.source_frame, '源(光学)', ids, self.print_limit)

        # Step 0: optical → camera（默认恒等，与参考代码一致）
        pts_camera = []
        if self.apply_optical_to_camera_rotation:
            R_cam_from_opt = np.array([[0,  0, 1],
                                       [-1, 0, 0],
                                       [0, -1, 0]], dtype=float)
            for p_opt in pts_optical:
                p_cam = R_cam_from_opt @ p_opt
                pts_camera.append(p_cam)
            # self._log_points(np.array(pts_camera), f'{self.source_frame}_camera', '源(相机)', ids, self.print_limit)
        else:
            pts_camera = pts_optical  # 与参考代码 _optical_to_camera() 行为一致

        # Step 1: camera → wrist
        pts_wrist = []
        for p_cam in pts_camera:
            p_wr = self.R_cam2wrist @ p_cam + self.t_cam2wrist
            pts_wrist.append(p_wr)
        # self._log_points(np.array(pts_wrist), self.wrist_frame, '手腕', ids, self.print_limit)

        # Step 2: wrist → base
        try:
            ok, tf_bw = self.robot.get_tf_transform(self.target_frame, self.wrist_frame)
            if not ok or tf_bw is None or len(tf_bw) < 7:
                self.get_logger().warn(f'获取TF失败 {self.target_frame} <- {self.wrist_frame}')
                return
            # [tx, ty, tz, qw, qx, qy, qz]
            tx, ty, tz, qw, qx, qy, qz = [float(tf_bw[i]) for i in range(7)]
            R_wb = quat_to_rot(qx, qy, qz, qw)  # wrist → base
            t_wb = np.array([tx, ty, tz], dtype=float)

            pts_base = []
            for p_wr in pts_wrist:
                p_b = R_wb @ p_wr + t_wb
                pts_base.append(p_b)

        except Exception as e:
            self.get_logger().warn(f'TF异常 {self.target_frame} <- {self.wrist_frame}: {e}')
            return

        # 打印基座坐标
        n_print = min(self.print_limit, len(pts_base))
        if n_print > 0:
            self.get_logger().info(f"转换到 {self.target_frame} 坐标系的点(前{n_print}/{len(pts_base)}个):")
            for i in range(n_print):
                x, y, z = pts_base[i]
                tag = f"#{int(ids[i])}" if ids is not None and i < len(ids) else f"#{i}"
                self.get_logger().info(f"  {tag}: X={x:.4f} Y={y:.4f} Z={z:.4f}")

        # 发布
        if self.pub is not None:
            out_msg = PointCloud()
            out_msg.header = msg.header
            out_msg.header.frame_id = self.target_frame
            out_msg.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts_base]
            out_msg.channels = msg.channels
            self.pub.publish(out_msg)

        # 缓存基座系点用于粗定位控制
        if self.enable_coarse_move:
            self.latest_pts_base = pts_base
            self.latest_ids = ids
            # 选择目标点：优先匹配target_id，否则取第一个
            target = None
            if self.target_id >= 0 and ids is not None:
                for i, p in enumerate(pts_base):
                    try:
                        if int(ids[i]) == self.target_id:
                            target = p
                            break
                    except Exception:
                        continue
            if target is None and len(pts_base) > 0:
                target = pts_base[0]

            # >>> 新增：安全过滤 <<<
            valid_target = None
            if target is not None:
                target = np.array(target, dtype=float)
                dist_from_base = np.linalg.norm(target)
                z_val = target[2]

                # 条件1: 距离 <= 2.0m
                # 条件2: Z轴的上下限 -1.0 <= Z <= 0.0 
                if dist_from_base <= 2.0 and (-1.0 <= z_val <= 0.0):
                    valid_target = target
                else:
                    self.get_logger().warn(
                        f'目标点被过滤: distance={dist_from_base:.3f}m, Z={z_val:.3f} (要求: dist≤2.0m 且 -1.0≤Z≤0.0)'
                    )
            # <<<

            self.target_point = valid_target
            # 在进入粗定位前，先做基于末端关节的左右旋转微调（绕URDF Z）
            if self.servo_yaw_enabled and valid_target is not None:
                self._run_servo_yaw_adjust(valid_target)

    # ------------------- 末端粗定位控制（基于 set_arm_position） -------------------
    def _get_arm_position(self):
        """读取当前末端位姿(基于对应臂的base)，返回 (ok, p_base_xyz, quat_wxyz)
        依照接口文档：pose = [x,y,z,w,x,y,z]
        """
        try:
            ok, pose = self.robot.get_arm_position(self.component_type)
            if ok and pose and len(pose) >= 7:
                x, y, z, qw, qx, qy, qz = [float(v) for v in pose[:7]]
                return True, np.array([x, y, z], dtype=float), np.array([qw, qx, qy, qz], dtype=float)
        except Exception as e:
            self.get_logger().warn(f'get_arm_position 异常: {e}')
        return False, None, None

    def _build_pose_wxyz(self, p_xyz, quat_wxyz):
        return [float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]), float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])]

    def _run_servo_yaw_adjust(self, target_base: np.ndarray):
        """
        基于基座系目标点 y 偏差，调用 set_arm_servo_angle 调整末端最后关节水平左右转。
        正值让相机向右，负值向左。
        """
        now = time.time()
        if (now - self.last_servo_cmd_ts) < self.servo_cmd_interval:
            return
        if not hasattr(self, 'servo_base_positions') or len(self.servo_base_positions) < 7:
            self.get_logger().warn('servo_base_positions 长度不足，跳过servo调整')
            return
        try:
            dy = float(target_base[1])
        except Exception:
            return

        positions = list(self.servo_base_positions)
        delta_yaw = -self.servo_yaw_gain * dy  # 目标在左( +y )则朝左转(负)，在右则朝右转(正)
        delta_yaw = float(np.clip(delta_yaw, -self.servo_yaw_max, self.servo_yaw_max))
        positions[-1] = positions[-1] + delta_yaw

        try:
            self.robot.set_arm_servo_angle(self.component_type, positions, float(self.servo_speed), float(self.servo_acc), True)
            self.last_servo_cmd_ts = now
            self.get_logger().info(f'[servo yaw] dy={dy:.3f}, delta={delta_yaw:.3f}, cmd sent')
        except Exception as e:
            self.get_logger().warn(f'set_arm_servo_angle 失败: {e}')

    def control_loop(self):
        if not self.enable_coarse_move or self.robot is None:
            return
        if self.target_point is None:
            return
        ok, p_curr, quat_wxyz = self._get_arm_position()
        if not ok:
            return
        # 首次缓存姿态，粗定位阶段保持
        if self.cached_quat_wxyz is None:
            self.cached_quat_wxyz = quat_wxyz

        # 误差与距离
        e = self.target_point - p_curr
        d = float(np.linalg.norm(e))

        # 日志：1Hz
        if not hasattr(self, '_last_log_ts'):
            self._last_log_ts = 0.0
        now = time.time()
        if now - self._last_log_ts > 1.0:
            self.get_logger().info(f'[粗定位] dist={d:.3f} m, curr=({p_curr[0]:.3f},{p_curr[1]:.3f},{p_curr[2]:.3f}), target=({self.target_point[0]:.3f},{self.target_point[1]:.3f},{self.target_point[2]:.3f})')
            self._last_log_ts = now

        # 到达阈值则不再发送
        if d <= self.distance_stop:
            return

        # 计算一步目标（不越过阈值）
        if d > 1e-6:
            step = min(self.step_size, max(0.0, d - self.distance_stop))
            p_next = p_curr + (e / d) * step
        else:
            p_next = p_curr.copy()

        pose_next = self._build_pose_wxyz(p_next, self.cached_quat_wxyz)

        # 频率/等待控制
        try:
            cmd_ok = False
            if self.use_wait:
                # 阻塞式：调用即等待完成
                self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), True)
                cmd_ok = True
                self.last_cmd_ts = now
            else:
                if (now - self.last_cmd_ts) >= self.cmd_interval:
                    self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), False)
                    cmd_ok = True
                    self.last_cmd_ts = now

            if cmd_ok:
                self._move_fail_count = 0  # 成功则清零
            else:
                # 非发送时机，不算失败
                pass

        except Exception as e:
            self.get_logger().warn(f'set_arm_position 失败: {e}')
            self._move_fail_count += 1

            if self._move_fail_count >= self._max_fail_count:
                self.get_logger().error(f'❌ 粗定位连续失败 {self._max_fail_count} 次，暂停控制')
                self.enable_coarse_move = False  # 自动关闭


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