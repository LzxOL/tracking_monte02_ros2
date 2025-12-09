# TrackOn ROS2 节点

ROS2节点实现实时摄像头关键点跟踪功能。

## 功能

- 订阅摄像头图像话题或直接读取摄像头
- 通过ROS2服务接收初始关键点选择
- 实时跟踪关键点并发布结果
- 发布可视化图像

## 构建

### 前置要求

确保已安装必要的依赖：
- `python3-empy` 或 `empy` Python 包（版本 3.3.4）
- ROS2 Humble 及其依赖

### 构建步骤

首先构建消息和服务包，然后构建主节点包：

```bash
cd /home/root1/Corenetic/code/project/tracking_with_cameara_ws

# 如果遇到 empy 模块问题，使用系统 Python 构建消息和服务包
CMAKE_PREFIX_PATH=/opt/ros/humble PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  colcon build --packages-select track_on_ros2_msgs track_on_ros2_srv \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

# 构建主节点包
colcon build --packages-select track_on_ros2

# Source 工作空间
source install/setup.bash
```

**注意**：如果使用 anaconda 环境，可能需要指定系统 Python 来构建消息和服务包，以避免版本冲突。

## 使用方法

### 1. 启动节点

#### 方式1：使用启动文件（推荐）
```bash
# 直接读取摄像头
ros2 launch track_on_ros2 track_camera.launch.py \
  checkpoint_path:=/path/to/checkpoints/track_on_checkpoint.pt \
  camera_id:=0 \
  use_camera_topic:=false

# 订阅摄像头话题
ros2 launch track_on_ros2 track_camera.launch.py \
  checkpoint_path:=/path/to/checkpoints/track_on_checkpoint.pt \
  use_camera_topic:=true \
  camera_topic:=/camera/image_raw
```

#### 方式2：直接运行节点
```bash
# 直接读取摄像头
ros2 run track_on_ros2 track_camera_node \
  --ros-args \
  -p checkpoint_path:=/path/to/checkpoints/track_on_checkpoint.pt \
  -p camera_id:=0 \
  -p use_camera_topic:=false

# 订阅摄像头话题
ros2 run track_on_ros2 track_camera_node \
  --ros-args \
  -p checkpoint_path:=/path/to/checkpoints/track_on_checkpoint.pt \
  -p use_camera_topic:=true \
  -p camera_topic:=/camera/image_raw
```

### 2. 设置初始关键点

通过服务设置要跟踪的关键点：

```bash
ros2 service call /tracking/set_keypoints track_on_ros2_srv/srv/SetKeypoints \
  "{x: [100.0, 200.0, 300.0], y: [150.0, 250.0, 350.0]}"
```

### 3. 开始跟踪

```bash
ros2 service call /tracking/control track_on_ros2_srv/srv/ControlTracking \
  "{command: 'start'}"
```

### 4. 停止跟踪

```bash
ros2 service call /tracking/control track_on_ros2_srv/srv/ControlTracking \
  "{command: 'stop'}"
```

### 5. 重置跟踪

```bash
ros2 service call /tracking/reset track_on_ros2_srv/srv/ResetTracking
```

## 话题

### 发布的话题

- `/tracking/keypoints` (track_on_ros2_msgs/msg/Keypoints): 跟踪的关键点位置和可见性
- `/tracking/visualization` (sensor_msgs/msg/Image): 可视化图像（可选）

### 订阅的话题

- `/camera/image_raw` (sensor_msgs/msg/Image): 摄像头图像（当use_camera_topic=true时）

## 服务

- `/tracking/set_keypoints` (track_on_ros2_srv/srv/SetKeypoints): 设置初始关键点
- `/tracking/control` (track_on_ros2_srv/srv/ControlTracking): 控制跟踪（开始/停止）
- `/tracking/reset` (track_on_ros2_srv/srv/ResetTracking): 重置跟踪状态

## 参数

- `checkpoint_path` (string, 必需): 模型检查点文件路径
- `camera_id` (int, 默认: 0): 摄像头ID（当use_camera_topic=false时）
- `use_camera_topic` (bool, 默认: false): 是否使用摄像头话题
- `camera_topic` (string, 默认: '/camera/image_raw'): 摄像头图像话题名称
- `publish_visualization` (bool, 默认: true): 是否发布可视化图像
- `show_interactive_window` (bool, 默认: true): 是否显示交互窗口（可以点击选择关键点）

## 交互窗口操作

当 `show_interactive_window=true` 时，会显示一个可交互的窗口：

- **点击鼠标左键**: 在图像上选择要跟踪的关键点
- **按空格键**: 开始跟踪已选择的关键点
- **按 'r' 键**: 重置跟踪并清除已选择的关键点
- **按 'q' 键**: 退出节点

## 查看可视化结果

```bash
ros2 run rqt_image_view rqt_image_view /tracking/visualization
```

## 查看关键点消息

```bash
ros2 topic echo /tracking/keypoints
```

