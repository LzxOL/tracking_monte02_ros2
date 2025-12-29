import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_workspace_root():
    """获取工作空间根目录"""
    current_file = os.path.abspath(__file__)
    path = os.path.dirname(current_file)
    for _ in range(10):
        if os.path.basename(path) == 'tracking_with_cameara_ws':
            return path
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def generate_launch_description():
    ws_root = get_workspace_root()
    if ws_root:
        default_checkpoint_path_str = os.path.join(ws_root, 'src', 'track_on', 'checkpoints', 'track_on_checkpoint.pt')
        default_intrinsics_file_str = os.path.join(ws_root, 'config', 'camera_head_front_intrinsics.txt')
    else:
        default_checkpoint_path_str = ''
        default_intrinsics_file_str = ''
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value=default_checkpoint_path_str,
            description='模型检查点文件路径',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera_head_front/color/video',
            description='前端摄像头彩色图像话题',
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
        DeclareLaunchArgument(
            'intrinsics_file',
            default_value=default_intrinsics_file_str,
            description='相机内参文件路径',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera_head_front/depth/stream',
            description='深度图像话题',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_head_front/color/camera_info',
            description='相机信息话题（如果未提供内参文件，从此话题获取）',
        ),
        DeclareLaunchArgument(
            'print_3d',
            default_value='true',
            description='是否打印3D坐标',
        ),
        DeclareLaunchArgument(
            'print_3d_interval',
            default_value='1',
            description='每N帧打印一次3D坐标',
        ),
        DeclareLaunchArgument(
            'depth_scale',
            default_value='0.001',
            description='深度缩放因子（16UC1通常为0.001，从毫米转为米）',
        ),
        Node(
            package='track_on_ros2',
            executable='track_camera_front_node',
            name='track_camera_front_node',
            parameters=[{
                'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'publish_visualization': LaunchConfiguration('publish_visualization'),
                'show_interactive_window': LaunchConfiguration('show_interactive_window'),
                'intrinsics_file': LaunchConfiguration('intrinsics_file'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'print_3d': LaunchConfiguration('print_3d'),
                'print_3d_interval': LaunchConfiguration('print_3d_interval'),
                'depth_scale': LaunchConfiguration('depth_scale'),
            }],
            output='screen',
        ),
    ])

