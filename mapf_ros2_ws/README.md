# MAPF Simulator — ROS2 Humble + Gazebo 11

Multi-Agent Path Finding for 100 differential-drive robots.
Uses **Priority-Based Search (PBS)** + **Space-Time A*** for collision-free planning.

---

## Quick Start

### Step 1 — Install ROS2 Humble + Gazebo packages
```bash
chmod +x install_ros.sh && ./install_ros.sh
```
> Requires `sudo`. Takes ~5 min on first run.

### Step 2 — Build
```bash
chmod +x build.sh && ./build.sh
```

### Step 3 — Run
```bash
chmod +x run.sh && ./run.sh
# Optionally with custom args:
./run.sh 100 20 12    # 100 agents, 20x20 grid, 12% obstacles
```

This opens **Gazebo** + **RViz2** simultaneously.

---

## Architecture

```
mapf_ros2_ws/
├── install_ros.sh             # One-time ROS2 install
├── build.sh                   # colcon build helper
├── run.sh                     # Launch helper
└── src/mapf_ros/
    ├── mapf_ros/
    │   ├── core/              # Pure Python MAPF algorithms (no ROS deps)
    │   │   ├── grid.py        # Grid environment
    │   │   ├── astar.py       # Space-Time A* + ReservationTable
    │   │   ├── agent.py       # Agent state machine
    │   │   └── mapf_coordinator.py  # PBS planner
    │   ├── mapf_planner_node.py     # ROS2 node: runs PBS, publishes paths
    │   ├── agent_controller_node.py # ROS2 node: P-controller → cmd_vel
    │   └── world_spawner_node.py    # ROS2 node: spawns obstacles + robots
    ├── launch/mapf_full.launch.py   # Master launch file
    ├── worlds/mapf_world.world      # Gazebo world
    ├── config/mapf_params.yaml      # Tunable parameters
    └── rviz/mapf.rviz               # Pre-configured RViz2 layout
```

---

## ROS2 Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/mapf/grid_map` | `nav_msgs/OccupancyGrid` | planner | 2D obstacle map |
| `/mapf/agent_{i}/plan` | `nav_msgs/Path` | planner | Per-agent path |
| `/mapf/status` | `std_msgs/String` (JSON) | planner | Live stats |
| `/mapf/visualization` | `visualization_msgs/MarkerArray` | planner | RViz goals + obstacles |
| `/agent_{i}/cmd_vel` | `geometry_msgs/Twist` | controller | Drive commands |
| `/agent_{i}/odom` | `nav_msgs/Odometry` | Gazebo | Pose feedback |

---

## Configuration

Edit `config/mapf_params.yaml` or pass as launch args:

```bash
ros2 launch mapf_ros mapf_full.launch.py \
  agent_count:=50 \
  grid_size:=15 \
  obstacle_density:=0.08 \
  random_seed:=123
```

---

## Monitoring

```bash
# Watch live status
ros2 topic echo /mapf/status

# Check all topics
ros2 topic list | grep -E "(mapf|agent)"

# Force replan
ros2 topic pub /mapf/trigger_replan std_msgs/Bool "data: true" --once
```

Browser dashboard also available at **http://localhost:8765**

---

## Algorithm Reference

| Component | Algorithm |
|-----------|-----------|
| Individual routing | A* with Manhattan heuristic |
| Multi-agent planning | Priority-Based Search (PBS) |
| Conflict avoidance | Space-Time A* + ReservationTable |
| Conflict detection | Vertex conflicts (same cell, same time) |
| Conflict resolution | Lower-priority agent replans in-place |
| Physical control | P-controller: heading error → ω, distance → v |

---

## Phase 2 Roadmap

- [ ] Conflict-Based Search (CBS) for optimal solutions
- [ ] ORCA real-time velocity obstacles
- [ ] Dynamic obstacle injection (`/mapf/add_obstacle` topic)
- [ ] Multi-floor / 3D environments
- [ ] Nav2 integration for more sophisticated local planning
