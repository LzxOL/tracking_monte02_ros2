from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 仅启动臂控制节点的 launch 文件
# 用法：
#   ros2 launch Monte_api_ros2 arm_control.launch.py \
#       robot_ip:=192.168.22.63:50051 \
#       robot_lib_path:=/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib \
#       component:=1 interactive:=false

def generate_launch_description() -> LaunchDescription:
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.22.63:50051',
        description='Robot gRPC server ip:port'
    )
    robot_lib_path_arg = DeclareLaunchArgument(
        'robot_lib_path',
        default_value='/home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/Monte_api_ros2/lib',
        description='Path to RobotLib .so directory'
    )
    component_arg = DeclareLaunchArgument(
        'component', default_value='1', description='Arm component: 1 left, 2 right'
    )
    interactive_arg = DeclareLaunchArgument(
        'interactive', default_value='true', description='Whether to wait for key press between steps'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_lib_path = LaunchConfiguration('robot_lib_path')
    component = LaunchConfiguration('component')
    interactive = LaunchConfiguration('interactive')

    arm_ctrl_node = Node(
        package='Monte_api_ros2',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'robot_lib_path': robot_lib_path,
            'component': component,
            'interactive': interactive,
        }]
    )

    return LaunchDescription([
        robot_ip_arg, robot_lib_path_arg,
        component_arg, interactive_arg,
        arm_ctrl_node,
    ])

