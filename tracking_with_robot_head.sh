#!/usr/bin/env bash
# 启动前端摄像头跟踪系统
# 1. source 工作空间环境
# 2. 启动 robot_video_client 节点
# 3. 延迟5秒
# 4. 启动 track_camera_front_min 节点

set -euo pipefail

# 获取脚本所在目录（工作空间根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ============================
# 新增：自动 source 工作空间
# ============================
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "[INFO] 自动 source install/setup.bash"
    echo "[INFO] 自动 source install/setup.bash"

    set +u
    set +u
    source "${SCRIPT_DIR}/install/setup.bash"
    set -u
    set -u
else
    echo "错误: 找不到 ${SCRIPT_DIR}/install/setup.bash"
    echo "请先编译: colcon build"
    exit 1
fi

# 添加组播路由，避免视频接收失败
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eno1 || true

echo "=========================================="
echo "启动前端摄像头跟踪系统"
echo "=========================================="

# 启动 robot_video_client
echo "[1/3] 启动 robot_video_client 节点..."
ros2 launch robot_video_client robot_video_client.launch.py &
ROBOT_VIDEO_PID=$!

# 等待 5 秒
echo "[2/3] 等待 5 秒..."
sleep 5

# 启动 track_camera_front_min（前台运行）
echo "[3/3] 启动 track_camera_front_min 节点..."
ros2 launch track_on_ros2 track_camera_front_min.launch.py

# track_camera_front_min 退出时清理 robot_video_client
echo "track_camera_front_min 节点已退出，正在清理 robot_video_client..."
kill $ROBOT_VIDEO_PID 2>/dev/null || true
wait $ROBOT_VIDEO_PID 2>/dev/null || true

echo "所有节点已退出"
