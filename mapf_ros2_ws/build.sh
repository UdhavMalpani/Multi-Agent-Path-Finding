#!/usr/bin/env bash
# build.sh — Build the mapf_ros ROS2 package
set -euo pipefail
CYAN='\033[0;36m'; GREEN='\033[0;32m'; NC='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${CYAN}[MAPF]${NC} Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

echo -e "${CYAN}[MAPF]${NC} Building mapf_ros package..."
cd "$SCRIPT_DIR"
colcon build --symlink-install --packages-select mapf_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo -e "${GREEN}[ OK ]${NC} Build complete!"
echo -e "${CYAN}[MAPF]${NC} Run: ./run.sh"
