'''
Description: Launch file for video_click_node
Version: V1.0
Author: hongyuan.liu@corenetic.ai
Date: 2025-12-02
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera_head_front/color/video',
        description='图像话题名称'
    )

    # 创建节点
    video_click_node = Node(
        package='robot_video_client',
        executable='video_click_node',
        name='video_click_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic')
        }],
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        video_click_node
    ])

