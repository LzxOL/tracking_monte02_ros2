#!/usr/bin/env bash
# 启动完整的跟踪与机械臂控制系统
# 1. 启动视频流节点 (robot_video_client)
# 2. 检查视频流是否正常
# 3. 启动 Tracking 节点 (track_camera_front_min)
# 4. 启动 3D-TF 转换与机械臂控制节点 (points3d_tf_to_arm_base_node)

set -euo pipefail

# 获取脚本所在目录（工作空间根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ============================
# 自动 source 工作空间
# ============================
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "[INFO] 自动 source install/setup.bash"
    set +u
    source "${SCRIPT_DIR}/install/setup.bash"
    set -u
else
    echo "错误: 找不到 ${SCRIPT_DIR}/install/setup.bash"
    echo "请先编译: colcon build"
    exit 1
fi

# 添加组播路由，避免视频接收失败
echo "[INFO] 添加组播路由..."
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eno1 || true

echo "=========================================="
echo "启动完整的跟踪与机械臂控制系统"
echo "=========================================="

# ============================
# 步骤 1: 启动视频流节点
# ============================
echo "[1/3] 启动 robot_video_client 节点..."
ros2 launch robot_video_client robot_video_client.launch.py &
ROBOT_VIDEO_PID=$!
echo "robot_video_client PID: $ROBOT_VIDEO_PID"

# 等待 5 秒
echo "[1/3] 等待 5 秒，让视频流节点初始化..."
sleep 5

# ============================
# 步骤 2: 启动 Tracking 节点
# ============================
echo "[2/3] 启动 track_camera_front_min 节点..."
ros2 launch track_on_ros2 track_camera_front_min.launch.py &
TRACKING_PID=$!
echo "track_camera_front_min PID: $TRACKING_PID"

# 等待 5 秒
echo "[2/3] 等待 5 秒，让 Tracking 节点初始化..."
sleep 5

# ============================
# 步骤 3: 启动 3D-TF 转换与机械臂控制节点
# ============================
echo "[3/3] 启动 points3d_tf_to_arm_base_node 节点..."
echo "=========================================="
echo "所有节点已启动，points3d_tf_to_arm_base_node 在前台运行"
echo "按 Ctrl+C 退出所有节点"
echo "=========================================="

# 前台运行 points3d_tf_to_arm_base_node
ros2 run Monte_api_ros2 points3d_tf_to_arm_base_node

# ============================
# 清理：当 points3d_tf_to_arm_base_node 退出时，清理其他节点
# ============================
echo ""
echo "points3d_tf_to_arm_base_node 节点已退出，正在清理其他节点..."
kill $TRACKING_PID 2>/dev/null || true
kill $ROBOT_VIDEO_PID 2>/dev/null || true
wait $TRACKING_PID 2>/dev/null || true
wait $ROBOT_VIDEO_PID 2>/dev/null || true

echo "所有节点已退出"

