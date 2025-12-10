import os
import sys
import time
import math
import rclpy
from rclpy.node import Node

"""
一个简单的控制节点，参考 demo/monte02_api_example/test_arm_move_with_arm_position_linzhaoxian.py
- 仅操作左臂（component=1，可通过参数修改）
- 步骤：启用 -> 设伺服模式 -> 移到安全位姿 -> 读取末端位姿 -> 微调后下发 -> 再读取确认
运行示例：
  ros2 run Monte_api_ros2 arm_control_node \
    --ros-args \
    -p robot_ip:=192.168.22.63:50051 \
    -p robot_lib_path:=/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib \
    -p component:=1 \
    -p interactive:=false
"""


def _ensure_robotlib_visible(robot_lib_path: str):
    if not robot_lib_path:
        return
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if robot_lib_path not in ld_path.split(':'):
        os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
    if robot_lib_path not in sys.path:
        sys.path.append(robot_lib_path)


def getch_block():
    try:
        import getch  # type: ignore
        return getch.getch()
    except Exception:
        # 无 getch 时，退化为回车继续
        input('按回车继续...')
        return '\n'


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        # 参数
        # 参考 linzhaoxian 脚本默认地址
        self.declare_parameter('robot_ip', '192.168.22.63:50051')
        # 默认按照安装后的 share 目录查找 lib；若失败，退回源码相对目录
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            default_lib = os.path.join(get_package_share_directory('Monte_api_ros2'), 'lib')
        except Exception:
            default_lib = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
        self.declare_parameter('robot_lib_path', default_lib)
        self.declare_parameter('component', 2)  # 1 左臂, 2 右臂
        self.declare_parameter('interactive', True)  # 是否每步等待按键

        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        self.component = int(self.get_parameter('component').value)
        self.interactive = bool(self.get_parameter('interactive').value)

        self.get_logger().info(f'Robot IP: {robot_ip}')
        self.get_logger().info(f'RobotLib Path: {robot_lib_path}')
        self.get_logger().info(f'Component: {self.component}, interactive={self.interactive}')

        # 让 RobotLib 可见
        _ensure_robotlib_visible(robot_lib_path)
        try:
            from RobotLib import Robot  # type: ignore
        except Exception as e:
            self.get_logger().error(f'导入 RobotLib 失败: {e}')
            raise

        # 连接机器人
        try:
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'连接机器人失败: {e}')
            raise

        # 执行控制流程
        try:
            self.run_sequence()
        except Exception as e:
            self.get_logger().error(f'控制流程异常: {e}')
        finally:
            # 节点结束
            pass

    def wait_key(self, tip: str):
        if self.interactive:
            self.get_logger().info(tip + ' 按任意键继续...')
            getch_block()
        else:
            self.get_logger().info(tip)

    def run_sequence(self):
        comp = self.component

        # 1) 仅启用指定手臂并设置为伺服模式
        self.wait_key('启用手臂并设置伺服模式')
        success = self.robot.set_arm_enable(comp, True)
        self.get_logger().info(f'set_arm_enable success: {success}')
        success = self.robot.set_arm_mode(comp, 1)  # 1: servo motion mode
        self.get_logger().info(f'set_arm_mode success: {success}, mode=1')

        # 2) 可选：先到一个简单安全位姿（与示例类似）
        self.wait_key('移动到一个安全位姿')
        positions = [0, 0, 0, 1.57, 0, 0, 0]
        speed = 0.1
        acc = 1.0
        wait = True
        success = self.robot.set_arm_servo_angle(comp, positions, speed, acc, wait)
        self.get_logger().info(f'set_arm_servo_angle success: {success}')
        time.sleep(1.5)

        # 3) 读取 arm 坐标系下末端位姿
        self.wait_key('读取当前末端位姿(get_arm_position)')
        success, pose = self.robot.get_arm_position(comp)
        self.get_logger().info(f'get_arm_position success: {success}, pose: {pose}')

        # 4) 微调并下发位姿（示例对 y 增加 0.05）
        self.wait_key('下发末端位姿(对 y += 0.05)')
        try:
            pose[1] += 0.05
        except Exception:
            self.get_logger().warn('读取到的位姿不可修改(可能为None)，跳过设置')
            return
        success = self.robot.set_arm_position(comp, pose, speed, acc, wait)
        self.get_logger().info(f'set_arm_position success: {success}, pose: {pose}')
        time.sleep(1.5)

        # 5) 再读一次进行确认
        self.wait_key('再次读取末端位姿进行确认')
        success, pose2 = self.robot.get_arm_position(comp)
        self.get_logger().info(f'get_arm_position (after) success: {success}, pose: {pose2}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    # 不需要 spin，顺序执行完成即退出
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

