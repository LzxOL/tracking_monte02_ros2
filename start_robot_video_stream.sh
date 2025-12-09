#!/usr/bin/env bash
# 启动相机视频流（robot_video_client）
# 1) 添加组播路由到网卡 eno1
# 2) source 工作空间
# 3) 启动 robot_video_client launch

set -euo pipefail

# 工作空间根目录（本脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[1/3] 添加组播路由到 eno1: 224.5.6.7/32"
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eno1 || true

echo "[2/3] source 工作空间: $SCRIPT_DIR/install/setup.bash"
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "未找到 install/setup.bash，请先在工作空间执行: colcon build"
  exit 1
fi
# 暂时关闭 nounset，避免 setup.bash 引用未定义变量(COLCON_TRACE)报错
set +u
# shellcheck disable=SC1090
export COLCON_TRACE=${COLCON_TRACE:-0}
source "$SCRIPT_DIR/install/setup.bash"
# 重新开启 nounset
set -u

echo "[3/3] 启动 robot_video_client"
exec ros2 launch robot_video_client robot_video_client.launch.py

