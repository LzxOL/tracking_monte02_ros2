from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

# 一个统一的 launch 文件，可启动 TF 查询节点与臂控制节点，并可在命令行传参
# 用法示例：
#   1) 仅启动 TF 节点（默认）
#      ros2 launch Monte_api_ros2 monte_api.launch.py
#   2) 指定机器人 IP、库路径
#      ros2 launch Monte_api_ros2 monte_api.launch.py robot_ip:=192.168.22.63:50051 \
#           robot_lib_path:="/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib"
#   3) 启动控制节点（并关闭 TF 节点）
#      ros2 launch Monte_api_ros2 monte_api.launch.py run_arm_control:=true run_tf_node:=false interactive:=false component:=1
#   4) 同时启动两个节点
#      ros2 launch Monte_api_ros2 monte_api.launch.py run_arm_control:=true


def generate_launch_description() -> LaunchDescription:
    # ---------- 通用参数 ----------
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.22.63:50051',
        description='Robot gRPC server ip:port'
    )
    robot_lib_path_arg = DeclareLaunchArgument(
        'robot_lib_path',
        # 默认按当前工作区源码路径（请按需修改）
        default_value='/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib',
        description='Path to RobotLib .so directory'
    )

    # ---------- TF 节点参数 ----------
    run_tf_node_arg = DeclareLaunchArgument(
        'run_tf_node', default_value='true', description='Whether to run head_base_tf_node'
    )
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value='link_t0_base', description='TF parent frame'
    )
    child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value='link_h2_head', description='TF child frame'
    )
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='2.0', description='Query rate (Hz)'
    )

    # ---------- 控制节点参数 ----------
    run_arm_control_arg = DeclareLaunchArgument(
        'run_arm_control', default_value='false', description='Whether to run arm_control_node'
    )
    component_arg = DeclareLaunchArgument(
        'component', default_value='1', description='Arm component: 1 left, 2 right'
    )
    interactive_arg = DeclareLaunchArgument(
        'interactive', default_value='true', description='Whether to wait for key press between steps'
    )

    # LaunchConfigurations
    robot_ip = LaunchConfiguration('robot_ip')
    robot_lib_path = LaunchConfiguration('robot_lib_path')

    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    rate = LaunchConfiguration('rate')

    component = LaunchConfiguration('component')
    interactive = LaunchConfiguration('interactive')

    run_tf_node = LaunchConfiguration('run_tf_node')
    run_arm_control = LaunchConfiguration('run_arm_control')

    # 节点定义
    tf_node = Node(
        package='Monte_api_ros2',
        executable='head_base_tf_node',
        name='head_base_tf_node',
        output='screen',
        condition=IfCondition(run_tf_node),
        parameters=[{
            'robot_ip': robot_ip,
            'robot_lib_path': robot_lib_path,
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'rate': rate,
        }]
    )

    arm_ctrl_node = Node(
        package='Monte_api_ros2',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen',
        condition=IfCondition(run_arm_control),
        parameters=[{
            'robot_ip': robot_ip,
            'robot_lib_path': robot_lib_path,
            'component': component,
            'interactive': interactive,
        }]
    )

    return LaunchDescription([
        # 声明参数
        robot_ip_arg,
        robot_lib_path_arg,
        run_tf_node_arg,
        parent_frame_arg,
        child_frame_arg,
        rate_arg,
        run_arm_control_arg,
        component_arg,
        interactive_arg,
        # 节点
        tf_node,
        arm_ctrl_node,
    ])

