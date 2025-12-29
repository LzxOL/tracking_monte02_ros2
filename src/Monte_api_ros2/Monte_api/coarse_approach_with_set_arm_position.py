#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
粗定位节点：让指定机械臂末端朝目标点(已在Base坐标系)逼近，直到距离<=阈值(默认0.5m)即停止。
控制接口：RobotLib.set_arm_position(component_type, pose[wxyz], max_speed, max_acc, wait)

目标点来源：订阅 tracking/points3d_in_arm_base (sensor_msgs/PointCloud)
- 若提供channel 'id'，可用 target_id 过滤；否则取第一个点。

注意：该节点仅做“粗定位”，保持当前末端姿态不变，只在位置上逼近目标。
"""

import os
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud


def _ensure_robotlib_visible(robot_lib_path: str):
    if not robot_lib_path:
        return
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if robot_lib_path not in ld_path.split(':'):
        os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
    if robot_lib_path not in sys.path:
        sys.path.append(robot_lib_path)


class CoarseApproachWithSetArmPosition(Node):
    def __init__(self):
        super().__init__('coarse_approach_with_set_arm_position')

        # 话题/坐标/控制参数
        self.declare_parameter('target_topic', 'tracking/points3d_in_arm_base')
        self.declare_parameter('target_frame', 'link_r0_arm_base')
        self.declare_parameter('component_type', 1)  # 1:左臂 2:右臂
        self.declare_parameter('distance_stop', 0.5)  # m
        self.declare_parameter('step_size', 0.10)     # 每步前进距离 m
        self.declare_parameter('step_interval', 0.3)  # 每步发送间隔 s（仅当 wait=False 时生效）
        self.declare_parameter('max_speed', 0.10)     # m/s
        self.declare_parameter('max_acc', 0.50)       # m/s^2
        self.declare_parameter('timeout_sec', 120.0)  # 最大粗定位时长
        self.declare_parameter('use_wait', False)     # set_arm_position 的 wait
        self.declare_parameter('smooth_alpha', 0.4)   # 目标点指数平滑系数
        self.declare_parameter('target_id', -1)       # 若>=0，则匹配channel 'id'

        # RobotLib 连接参数
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            default_lib = os.path.join(get_package_share_directory('Monte_api_ros2'), 'lib')
        except Exception:
            default_lib = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
        self.declare_parameter('robot_ip', '192.168.22.63:50051')
        self.declare_parameter('robot_lib_path', default_lib)

        # 读取参数
        self.target_topic = self.get_parameter('target_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.component_type = int(self.get_parameter('component_type').value)
        self.d_stop = float(self.get_parameter('distance_stop').value)
        self.step_size = float(self.get_parameter('step_size').value)
        self.step_interval = float(self.get_parameter('step_interval').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acc = float(self.get_parameter('max_acc').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.use_wait = bool(self.get_parameter('use_wait').value)
        self.alpha = float(self.get_parameter('smooth_alpha').value)
        self.target_id = int(self.get_parameter('target_id').value)
        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value

        # 目标点缓存（指数平滑）
        self.target_point = None   # 最新的原始目标
        self.target_smooth = None  # 平滑后的目标

        # 订阅目标点
        self.sub = self.create_subscription(PointCloud, self.target_topic, self.cb_target, qos_profile_sensor_data)

        # 连接 RobotLib
        _ensure_robotlib_visible(robot_lib_path)
        try:
            from RobotLib import Robot  # type: ignore
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'导入/连接 RobotLib 失败: {e}')
            self.robot = None

        # 末端姿态缓存（保持姿态）
        self.current_quat_wxyz = None

        # 控制时序
        self.last_step_ts = 0.0
        self.start_ts = time.time()
        self.reached = False

        # 定时器：主循环
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(
            f'粗定位启动: topic={self.target_topic}, frame={self.target_frame}, comp={self.component_type}, stop={self.d_stop}m')

    # ------------------- 数据与工具 -------------------
    def _extract_channel(self, msg: PointCloud, name: str):
        try:
            for ch in msg.channels:
                if ch.name == name:
                    return ch.values
        except Exception:
            pass
        return None

    def cb_target(self, msg: PointCloud):
        if not msg.points:
            return
        # 选点：优先按channel id匹配
        sel = None
        if self.target_id >= 0:
            ids = self._extract_channel(msg, 'id')
            if ids is not None:
                for i, p in enumerate(msg.points):
                    try:
                        if int(ids[i]) == self.target_id:
                            sel = p
                            break
                    except Exception:
                        continue
        if sel is None:
            sel = msg.points[0]
        p = np.array([sel.x, sel.y, sel.z], dtype=float)
        self.target_point = p
        # 指数平滑
        if self.target_smooth is None:
            self.target_smooth = p.copy()
        else:
            self.target_smooth = self.alpha * p + (1 - self.alpha) * self.target_smooth

    def _get_arm_position(self):
        """读取当前末端位姿(基于对应臂的base)，返回 (ok, p_base_xyz, quat_wxyz)
        依照接口文档：pose = [x,y,z,w,x,y,z]
        """
        try:
            ok, pose = self.robot.get_arm_position(self.component_type)
            if ok and pose and len(pose) >= 7:
                x, y, z, qw, qx, qy, qz = map(float, pose[:7])
                return True, np.array([x, y, z], dtype=float), np.array([qw, qx, qy, qz], dtype=float)
        except Exception as e:
            self.get_logger().warn(f'get_arm_position 异常: {e}')
        return False, None, None

    def _build_pose_wxyz(self, p_xyz, quat_wxyz):
        return [float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]), float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])]

    # ------------------- 控制主循环 -------------------
    def control_loop(self):
        if self.robot is None:
            return
        if self.target_smooth is None:
            return
        # 获取当前末端位姿
        ok, p_curr, quat_wxyz = self._get_arm_position()
        if not ok:
            return
        if self.current_quat_wxyz is None:
            self.current_quat_wxyz = quat_wxyz

        # 误差与距离
        e = self.target_smooth - p_curr
        d = float(np.linalg.norm(e))

        # 打印距离（1Hz）
        now = time.time()
        if int((now - self.start_ts) * 1) != int((now - self.start_ts - 0.05) * 1):
            self.get_logger().info(f'[粗定位] dist={d:.3f} m, curr=({p_curr[0]:.3f},{p_curr[1]:.3f},{p_curr[2]:.3f}) target=({self.target_smooth[0]:.3f},{self.target_smooth[1]:.3f},{self.target_smooth[2]:.3f})')

        # 到达判定
        if d <= self.d_stop:
            if not self.reached:
                self.get_logger().info(f'[粗定位] 已进入 {self.d_stop:.2f} m 阈值，停止发送')
                self.reached = True
            return

        # 超时判定
        if now - self.start_ts > self.timeout_sec:
            self.get_logger().warn('[粗定位] 超时未到达，停止')
            return

        # 计算一步目标
        if d > 1e-6:
            step = min(self.step_size, max(0.0, d - self.d_stop))
            p_next = p_curr + (e / d) * step
        else:
            p_next = p_curr.copy()

        pose_next = self._build_pose_wxyz(p_next, self.current_quat_wxyz)

        # 发送命令（限速/加速度由API内部处理，这里传入参数）
        try:
            if self.use_wait:
                self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), True)
                self.last_step_ts = time.time()
            else:
                # 非阻塞：限制发送频率
                if (now - self.last_step_ts) >= self.step_interval:
                    self.robot.set_arm_position(self.component_type, pose_next, float(self.max_speed), float(self.max_acc), False)
                    self.last_step_ts = now
        except Exception as e:
            self.get_logger().warn(f'set_arm_position 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CoarseApproachWithSetArmPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

