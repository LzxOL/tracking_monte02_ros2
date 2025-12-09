#!/usr/bin/env bash
# 启动前端摄像头跟踪系统
# 1. 启动 robot_video_client 节点
# 2. 延迟5秒
# 3. 启动 track_camera_front 节点

set -euo pipefail

# 获取脚本所在目录（工作空间根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 添加组播路由，避免视频组播收不到
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eno1 || true

# 检查是否已source工作空间
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "错误: 请先 source 工作空间的 setup.bash"
    echo "运行: source install/setup.bash"
    exit 1
fi

echo "=========================================="
echo "启动前端摄像头跟踪系统"
echo "=========================================="

# 启动 robot_video_client 节点（后台运行）
echo "[1/3] 启动 robot_video_client 节点..."
ros2 launch robot_video_client robot_video_client.launch.py &
ROBOT_VIDEO_PID=$!

# 等待5秒
echo "[2/3] 等待 5 秒..."
sleep 5

# 启动 track_camera_front 节点（前台运行）
echo "[3/3] 启动 track_camera_front 节点..."
ros2 launch track_on_ros2 track_camera_front.launch.py

# 如果 track_camera_front 退出，清理后台进程
echo "track_camera_front 节点已退出，正在清理..."
kill $ROBOT_VIDEO_PID 2>/dev/null || true
wait $ROBOT_VIDEO_PID 2>/dev/null || true

echo "所有节点已退出"

