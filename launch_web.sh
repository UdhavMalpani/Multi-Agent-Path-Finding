#!/usr/bin/env bash
# ==============================================================================
# Launch MAPF Web-Based Simulator
# ==============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT_DIR/multi-agent-pathfinding/launch.sh" "$@"
