import os
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

"""
   运行: ros2 run Monte_api_ros2 head_base_tf_node
"""

class HeadBaseTFNode(Node):
    """周期性获取 parent_frame -> child_frame 的 TF，并打印与发布。
    默认 parent_frame=link_t0_base, child_frame=link_h2_head。
    依赖 demo/monte02_api_example/lib 下的 RobotLib。
    """

    def __init__(self) -> None:
        super().__init__('head_base_tf_node')

        # 参数
        self.declare_parameter('robot_ip', '192.168.22.63:50051')
        self.declare_parameter('robot_lib_path', '/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib')
        self.declare_parameter('parent_frame', 'link_l0_arm_base')
        self.declare_parameter('child_frame', 'left_camera_color_optical_frame')
        self.declare_parameter('rate', 2.0)  # Hz

        robot_ip = self.get_parameter('robot_ip').value
        robot_lib_path = self.get_parameter('robot_lib_path').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        rate = float(self.get_parameter('rate').value)

        self.get_logger().info(f'Robot IP: {robot_ip}')
        self.get_logger().info(f'RobotLib Path: {robot_lib_path}')
        self.get_logger().info(f'Query TF: {self.parent_frame} -> {self.child_frame} at {rate} Hz')

        # 导入并连接 RobotLib
        self.robot = None
        try:
            if robot_lib_path:
                # 让 .so 可见
                ld_path = os.environ.get('LD_LIBRARY_PATH', '')
                if robot_lib_path not in ld_path.split(':'):
                    os.environ['LD_LIBRARY_PATH'] = robot_lib_path + (':' + ld_path if ld_path else '')
                if robot_lib_path not in sys.path:
                    sys.path.append(robot_lib_path)
            from RobotLib import Robot  # type: ignore
            self.robot = Robot(robot_ip, '', '')
            self.get_logger().info('Robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to import/connect RobotLib: {e}')
            self.robot = None

        # 发布一个可选话题，方便其他节点订阅
        self.pub = self.create_publisher(TransformStamped, 'head_base_transform', 10)
        # 同时通过 tf2 广播该变换，供其他节点通过 TF 树查询
        self.tf_broadcaster = TransformBroadcaster(self)

        self._last_print = 0.0
        period = 1.0 / max(0.1, rate)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        if self.robot is None:
            return
        try:
            ok, tf = self.robot.get_tf_transform(self.parent_frame, self.child_frame)
            if not ok or tf is None:
                self.get_logger().warn(f'Failed to get tf {self.parent_frame} -> {self.child_frame}')
                return
            # 期望 tf: [x,y,z,qx,qy,qz,qw]
            msg = TransformStamped()
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = self.parent_frame
            msg.child_frame_id = self.child_frame
            if len(tf) >= 7:
                msg.transform.translation.x = float(tf[0])
                msg.transform.translation.y = float(tf[1])
                msg.transform.translation.z = float(tf[2])
                msg.transform.rotation.x = float(tf[3])
                msg.transform.rotation.y = float(tf[4])
                msg.transform.rotation.z = float(tf[5])
                msg.transform.rotation.w = float(tf[6])
            self.pub.publish(msg)
            # 通过 tf2 广播
            self.tf_broadcaster.sendTransform(msg)

            # 控制打印频率（每秒最多打印一次，避免刷屏）
            t = time.time()
            if t - self._last_print >= 1.0:
                if len(tf) >= 7:
                    x, y, z, qx, qy, qz, qw = [float(tf[i]) for i in range(7)]
                    self.get_logger().info(
                        f'TF {self.parent_frame} -> {self.child_frame}: '
                        f'p=({x:.3f}, {y:.3f}, {z:.3f}), '
                        f'q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
                    )
                else:
                    self.get_logger().info(f'TF {self.parent_frame} -> {self.child_frame}: {tf}')
                self._last_print = t
        except Exception as e:
            self.get_logger().error(f'Error while querying TF: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HeadBaseTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

