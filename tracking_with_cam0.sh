#!/bin/bash

# 工作空间路径
WS_PATH="/home/root1/Corenetic/code/project/tracking_with_cameara_ws"

# 模型 checkpoint 路径
CHECKPOINT_PATH="$WS_PATH/src/track_on/checkpoints/track_on_checkpoint.pt"

# 本地相机 ID
CAMERA_ID=0

echo "========================"
echo " 启动 TrackOn (本地摄像头模式)"
echo "========================"

# 切换到工作空间
cd $WS_PATH

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 加载工作空间环境
source install/setup.bash

# 启动 track_on_ros2 本地摄像头版本
echo ">>> 启动 TrackOn（camera_id=$CAMERA_ID）..."
ros2 launch track_on_ros2 track_camera.launch.py \
  checkpoint_path:=$CHECKPOINT_PATH \
  camera_id:=$CAMERA_ID

echo "========================"
echo " TrackOn 本地摄像头模式已退出"
echo "========================"
