#!/usr/bin/env bash
# ==============================================================================
# MAPF Simulator Launcher
# Starts a local server and opens the Multi-Agent Pathfinding Simulator in your browser.
# ==============================================================================

set -e

# Determine script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$SCRIPT_DIR"
ROS_WS="$(cd "$SCRIPT_DIR/../mapf_ros2_ws" && pwd)"

# ANSI color codes
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check if user requested ROS 2 / Gazebo mode
if [[ "${1:-}" == "--ros" || "${1:-}" == "-r" ]]; then
  echo -e "${CYAN}Launching ROS 2 / Gazebo MAPF Simulator...${NC}"
  if [ -f "$ROS_WS/run.sh" ]; then
    shift || true
    exec "$ROS_WS/run.sh" "$@"
  else
    echo -e "${RED}Error: ROS workspace run script not found at $ROS_WS/run.sh${NC}"
    exit 1
  fi
fi

if [ ! -f "$SIM_DIR/index.html" ]; then
  echo -e "${RED}Error: Simulation files not found at $SIM_DIR${NC}"
  exit 1
fi

# Find an available port starting from 8080
PORT=8080
while ss -tuln | grep -q ":${PORT} "; do
  PORT=$((PORT + 1))
done

echo -e "${GREEN}======================================================${NC}"
echo -e "${GREEN}       Multi-Agent Pathfinding (MAPF) Simulator       ${NC}"
echo -e "${GREEN}======================================================${NC}"
echo -e " Serving from : ${CYAN}${SIM_DIR}${NC}"
echo -e " URL          : ${GREEN}http://localhost:${PORT}${NC}"
echo -e " Hint         : Run ${YELLOW}mapf-sim --ros${NC} if you want the ROS2/Gazebo sim."
echo -e " Press ${YELLOW}Ctrl+C${NC} to stop the server."
echo -e "${GREEN}======================================================${NC}"

# Open in browser after a short delay so the server starts first
(sleep 0.6 && xdg-open "http://localhost:${PORT}" >/dev/null 2>&1 || true) &

# Start python static server strictly in SIM_DIR
exec python3 -m http.server "$PORT" --directory "$SIM_DIR"
