'''
Description: 
Version: V1.0
Author: hongyuan.liu@corenetic.ai
Date: 2025-03-21 09:19:41
LastEditors: hongyuan.liu@corenetic.ai
LastEditTime: 2025-04-21 14:11:50
Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_path = get_package_share_directory('robot_video_client')
    config_file = os.path.join(package_path, 'config', 'video_config.json')
    dds_config_file = os.path.join(package_path, 'config', 'dds_config.json')
    logger_config_file = os.path.join(package_path, 'config', 'logger_config.json')

    launch_description = LaunchDescription()

    robot_video_server_node = Node(
        package='robot_video_client',
        executable='robot_video_client',
        name='robot_video_client',
        parameters=[{'config_file': config_file},
                    {'dds_config_file': dds_config_file},
                    {'logger_config_file': logger_config_file}],
        output='screen'
    )
    
    launch_description.add_action(robot_video_server_node)

    return launch_description