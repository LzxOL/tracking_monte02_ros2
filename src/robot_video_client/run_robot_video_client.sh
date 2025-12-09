#!/usr/bin/env bash
set -euo pipefail

# Resolve workspace directory (script location)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$WS_DIR"

echo "[1/3] colcon build..."
colcon build

echo "[2/3] source install/setup.bash..."
# setup scripts may reference optional env vars; relax nounset temporarily
set +u
source "$WS_DIR/install/setup.bash"
set -u

echo "[3/3] ros2 launch robot_video_client robot_video_client.launch.py ${*}"
ros2 launch robot_video_client robot_video_client.launch.py "$@"

