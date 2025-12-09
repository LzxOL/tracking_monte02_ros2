#!/bin/bash

# 工作空间路径
WS_PATH="/home/root1/Corenetic/code/project/tracking_with_cameara_ws"

# 模型 checkpoint 路径
CHECKPOINT_PATH="$WS_PATH/src/track_on/checkpoints/track_on_checkpoint.pt"

# 相机话题
CAMERA_TOPIC="/camera/color/image_raw"

echo "========================"
echo " 启动 ROS2 + Orbbec + TrackOn"
echo "========================"

# 切换到工作空间
cd $WS_PATH

# 加载 ROS2 Humble 环境
source /opt/ros/humble/setup.bash

# 加载工作空间环境
source install/setup.bash

# 启动 Orbbec 相机驱动
echo ">>> 启动 Orbbec Gemini 330 相机节点..."
ros2 launch orbbec_camera gemini_330_series.launch.py &
CAMERA_PID=$!

# 等待相机初始化
echo ">>> 等待相机初始化 5 秒..."
sleep 5

# 启动 TrackOn 跟踪节点
echo ">>> 启动 TrackOn 跟踪节点..."
ros2 launch track_on_ros2 track_camera_orbbec.launch.py \
  checkpoint_path:=$CHECKPOINT_PATH \
  camera_topic:=$CAMERA_TOPIC &
TRACK_PID=$!

echo "========================"
echo "   所有节点已启动!"
echo "   Camera PID: $CAMERA_PID"
echo "   Tracker PID: $TRACK_PID"
echo "========================"

# 等待两个后台进程
wait $CAMERA_PID $TRACK_PID
