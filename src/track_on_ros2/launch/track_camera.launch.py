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
            'camera_id',
            default_value='0',
            description='摄像头ID（当use_camera_topic=false时）',
        ),
        DeclareLaunchArgument(
            'use_camera_topic',
            default_value='false',
            description='是否使用摄像头话题',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='摄像头图像话题名称',
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
            executable='track_camera_node',
            name='track_camera_node',
            parameters=[{
                'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                'camera_id': LaunchConfiguration('camera_id'),
                'use_camera_topic': LaunchConfiguration('use_camera_topic'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'publish_visualization': LaunchConfiguration('publish_visualization'),
                'show_interactive_window': LaunchConfiguration('show_interactive_window'),
            }],
            output='screen',
        ),
    ])

