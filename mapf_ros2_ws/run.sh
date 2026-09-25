#!/usr/bin/env bash
# run.sh — Launch the full MAPF simulation (Gazebo + RViz2 + all nodes)
set -eo pipefail
CYAN='\033[0;36m'; GREEN='\033[0;32m'; NC='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Parse optional arguments
AGENTS=${1:-100}
GRID=${2:-20}
OBSTACLES=${3:-12}

echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  MAPF Simulator — ROS2 Humble + Gazebo 11             ${NC}"
echo -e "${GREEN}  Agents: ${AGENTS}  Grid: ${GRID}×${GRID}  Obstacles: ${OBSTACLES}%    ${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}  Browser dashboard:${NC} http://localhost:8765"
echo -e "${CYAN}  RViz2:${NC}             opens automatically"
echo -e "${CYAN}  Gazebo:${NC}            opens automatically"
echo ""

ros2 launch mapf_ros mapf_full.launch.py \
  agent_count:="$AGENTS" \
  grid_size:="$GRID" \
  obstacle_density:="0.$(printf '%02d' "$OBSTACLES")"
