import os
import yaml
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


def load_config():
    """加载配置文件"""
    ws_root = get_workspace_root()
    if ws_root:
        config_file = os.path.join(ws_root, 'src', 'track_on_ros2', 'config', 'config.yaml')
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            except Exception:
                pass
    return {}


def generate_launch_description():
    ws_root = get_workspace_root()
    config = load_config()

    # 从配置文件获取默认值
    camera_selection = config.get('camera', {}).get('selection', 1)
    if camera_selection == 1:
        camera_config = config.get('left_camera', {})
    else:
        camera_config = config.get('right_camera', {})

    checkpoint_default = config.get('tracking', {}).get('checkpoint_path', '')
    if ws_root and not os.path.isabs(checkpoint_default):
        checkpoint_default = os.path.join(ws_root, checkpoint_default)

    camera_topic_default = camera_config.get('camera_topic', '/right/color/video')

    # 从配置文件获取机器人参数默认值
    robot_ip_default = config.get('robot', {}).get('ip', '192.168.22.63:50051')
    robot_component_type_default = str(config.get('robot', {}).get('component_type', 2))
    init_robot_arm_default = str(config.get('robot', {}).get('init_robot_arm', True)).lower()

    # Launch arguments
    checkpoint_path_arg = DeclareLaunchArgument(
        'checkpoint_path', default_value=checkpoint_default,
        description='TrackOn 模型检查点（ckp）路径')

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value=camera_topic_default,
        description='前端摄像头颜色图像话题')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value=robot_ip_default,
        description='机器人IP地址和端口')

    robot_component_type_arg = DeclareLaunchArgument(
        'robot_component_type', default_value=robot_component_type_default,
        description='机器人组件类型 (1: 左臂, 2: 右臂)')

    init_robot_arm_arg = DeclareLaunchArgument(
        'init_robot_arm', default_value=init_robot_arm_default,
        description='是否初始化机器人手臂')

    publish_visualization_arg = DeclareLaunchArgument(
        'publish_visualization', default_value='true',
        description='是否发布叠加关键点的可视化图像')

    show_interactive_window_arg = DeclareLaunchArgument(
        'show_interactive_window', default_value='true',
        description='是否显示交互窗口（点击选择关键点、空格开始）')

    # Launch configurations
    checkpoint_path = LaunchConfiguration('checkpoint_path')
    camera_topic = LaunchConfiguration('camera_topic')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_component_type = LaunchConfiguration('robot_component_type')
    init_robot_arm = LaunchConfiguration('init_robot_arm')
    publish_visualization = LaunchConfiguration('publish_visualization')
    show_interactive_window = LaunchConfiguration('show_interactive_window')

    node = Node(
        package='track_on_ros2',
        executable='track_camera_front_min_node',
        name='track_camera_front_min_node',
        output='screen',
        parameters=[{
            'checkpoint_path': checkpoint_path,
            'camera_topic': camera_topic,
            'robot_ip': robot_ip,
            'robot_component_type': robot_component_type,
            'init_robot_arm': init_robot_arm,
            'publish_visualization': publish_visualization,
            'show_interactive_window': show_interactive_window,
        }]
    )

    return LaunchDescription([
        checkpoint_path_arg,
        camera_topic_arg,
        robot_ip_arg,
        robot_component_type_arg,
        init_robot_arm_arg,
        publish_visualization_arg,
        show_interactive_window_arg,
        node
    ])

