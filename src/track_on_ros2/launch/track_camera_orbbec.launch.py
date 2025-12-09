from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'checkpoint_path',
            description='模型检查点文件路径',
        ),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/camera/color/image_raw',
            description='Orbbec 彩色图像话题',
        ),
        DeclareLaunchArgument(
            'publish_visualization',
            default_value='true',
            description='是否发布可视化图像',
        ),
        DeclareLaunchArgument(
            'show_interactive_window',
            default_value='true',
            description='是否显示交互窗口（可以点击选择关键点）',
        ),
        Node(
            package='track_on_ros2',
            executable='track_camera_orbbec_node',
            name='track_camera_orbbec_node',
            parameters=[{
                'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                'color_topic': LaunchConfiguration('color_topic'),
                'publish_visualization': LaunchConfiguration('publish_visualization'),
                'show_interactive_window': LaunchConfiguration('show_interactive_window'),
            }],
            output='screen',
        ),
    ])

