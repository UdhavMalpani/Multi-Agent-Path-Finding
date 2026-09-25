#!/usr/bin/env bash
# ==============================================================================
# Launch MAPF ROS2 + Gazebo Simulator
# ==============================================================================
set -eo pipefail
set +u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$ROOT_DIR/mapf_ros2_ws"

if [ ! -f "$ROS_WS/run.sh" ]; then
  echo "Error: run.sh not found at $ROS_WS/run.sh"
  exit 1
fi

exec "$ROS_WS/run.sh" "$@"
