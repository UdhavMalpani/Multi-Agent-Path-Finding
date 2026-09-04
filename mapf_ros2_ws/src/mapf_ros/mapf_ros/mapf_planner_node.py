#!/usr/bin/env python3
"""
mapf_planner_node.py — ROS2 Node: MAPF Planner

Responsibilities:
  1. Generate the grid (obstacles) with a configured seed.
  2. Run Priority-Based Search (PBS) to plan paths for all agents.
  3. Publish:
       /mapf/grid_map          nav_msgs/OccupancyGrid   (once on startup)
       /mapf/agent_{i}/plan    nav_msgs/Path            (per-agent path)
       /mapf/status            std_msgs/String          (JSON stats, 2 Hz)
       /mapf/visualization     visualization_msgs/MarkerArray (for RViz)
  4. Subscribe:
       /mapf/trigger_replan    std_msgs/Bool            (force replan)
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg            import OccupancyGrid, Path
from geometry_msgs.msg       import PoseStamped, Quaternion
from std_msgs.msg            import String, Bool, Header
from visualization_msgs.msg  import MarkerArray, Marker
from builtin_interfaces.msg  import Time as RosTime

from mapf_ros.core import Grid, Agent, AgentState, MAPFCoordinator


def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2)
    q.z = math.sin(yaw / 2)
    return q


class MAPFPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__("mapf_planner")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("agent_count",      100)
        self.declare_parameter("grid_size",         20)
        self.declare_parameter("obstacle_density",  0.12)
        self.declare_parameter("random_seed",       42)
        self.declare_parameter("max_time_horizon", 400)
        self.declare_parameter("frame_id",         "map")

        self.agent_count      = self.get_parameter("agent_count").value
        self.grid_size        = self.get_parameter("grid_size").value
        self.obstacle_density = self.get_parameter("obstacle_density").value
        self.seed             = self.get_parameter("random_seed").value
        self.frame_id         = self.get_parameter("frame_id").value

        self.get_logger().info(
            f"MAPF Planner starting: {self.agent_count} agents, "
            f"{self.grid_size}×{self.grid_size} grid, "
            f"{self.obstacle_density*100:.0f}% obstacles"
        )

        # ── Latching QoS (transient local) for map + plans ────────────────
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._map_pub   = self.create_publisher(OccupancyGrid,  "/mapf/grid_map",      latch_qos)
        self._vis_pub   = self.create_publisher(MarkerArray,    "/mapf/visualization",  10)
        self._stat_pub  = self.create_publisher(String,         "/mapf/status",         10)

        # Per-agent plan publishers (created dynamically after planning)
        self._plan_pubs: dict = {}

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Bool, "/mapf/trigger_replan",
                                 self._replan_callback, 10)

        # ── State ─────────────────────────────────────────────────────────
        self.grid:  Grid              = None
        self.agents: list             = []
        self.coordinator: MAPFCoordinator = None
        self._planned = False

        # ── Init ──────────────────────────────────────────────────────────
        self._build_scenario()

        # Publish status at 2 Hz
        self.create_timer(0.5, self._publish_status)
        # Publish RViz markers at 1 Hz
        self.create_timer(1.0, self._publish_markers)

    # ── Scenario construction ──────────────────────────────────────────────

    def _build_scenario(self) -> None:
        self.get_logger().info("Building scenario…")
        t0 = time.perf_counter()

        self.grid = Grid(self.grid_size, self.grid_size)

        # Generate obstacles (protect nothing yet; starts/goals chosen after)
        self.grid.generate_obstacles(
            density=self.obstacle_density,
            seed=self.seed,
        )

        # Pick starts & goals
        starts, goals = self.grid.random_walkable_pairs(
            self.agent_count, seed=self.seed
        )
        actual = len(starts)
        if actual < self.agent_count:
            self.get_logger().warn(
                f"Only {actual}/{self.agent_count} agents could be placed "
                "(grid too small or too crowded)"
            )

        self.agents = [
            Agent(i, starts[i], goals[i], priority=i)
            for i in range(actual)
        ]

        # Run PBS
        self.get_logger().info(f"Running PBS for {actual} agents…")
        self.coordinator = MAPFCoordinator(self.grid, self.agents)
        self.coordinator.plan_all(start_time=0)

        t1 = time.perf_counter()
        stats = self.coordinator.get_stats()
        self.get_logger().info(
            f"PBS done in {(t1-t0)*1000:.0f}ms — "
            f"{stats.get('ARRIVED',0)+stats.get('MOVING',0)} paths found, "
            f"{stats.get('STUCK',0)} stuck"
        )

        self._planned = True

        # Create per-agent plan publishers and publish paths
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        for agent in self.agents:
            topic = f"/mapf/agent_{agent.id}/plan"
            if topic not in self._plan_pubs:
                self._plan_pubs[topic] = self.create_publisher(Path, topic, latch_qos)
            self._plan_pubs[topic].publish(self._agent_path_msg(agent))

        # Publish map
        self._map_pub.publish(self._build_map_msg())
        self.get_logger().info("Grid map published.")

    # ── Message builders ───────────────────────────────────────────────────

    def _build_map_msg(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.info.resolution = 1.0           # 1 metre per cell
        msg.info.width      = self.grid.cols
        msg.info.height     = self.grid.rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.to_occupancy_data()
        return msg

    def _agent_path_msg(self, agent: Agent) -> Path:
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wp in agent.path:
            ps = PoseStamped()
            ps.header = msg.header
            wx, wy = Grid.cell_to_world(wp["x"], wp["y"])
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.1
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    # ── RViz markers ──────────────────────────────────────────────────────

    def _publish_markers(self) -> None:
        if not self._planned:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Goal markers — spheres at goal positions
        for ag in self.agents:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = now
            m.ns     = "goals"
            m.id     = ag.id
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            wx, wy   = Grid.cell_to_world(ag.goal["x"], ag.goal["y"])
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            # Colour by ID (HSV-like hue spacing)
            hue = (ag.id * 137.5) % 360.0
            r, g, b = _hue_to_rgb(hue)
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.8
            ma.markers.append(m)

        # Obstacle cubes
        for ox, oy in self.grid.obstacle_positions():
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = now
            m.ns     = "obstacles"
            m.id     = oy * self.grid.cols + ox
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(ox) + 0.5
            m.pose.position.y = float(oy) + 0.5
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.95
            m.scale.z = 0.3
            m.color.r = 0.15; m.color.g = 0.25; m.color.b = 0.45; m.color.a = 0.9
            ma.markers.append(m)

        self._vis_pub.publish(ma)

    # ── Status publishing ──────────────────────────────────────────────────

    def _publish_status(self) -> None:
        if not self._planned:
            return
        stats = self.coordinator.get_stats()
        stats["timestamp"] = time.time()
        agents_json = [ag.to_dict() for ag in self.agents]
        payload = {"stats": stats, "agents": agents_json}
        msg = String()
        msg.data = json.dumps(payload)
        self._stat_pub.publish(msg)

    # ── Replan callback ───────────────────────────────────────────────────

    def _replan_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("Replan triggered externally.")
            self._build_scenario()


# ── HSV hue → RGB helper ──────────────────────────────────────────────────

def _hue_to_rgb(hue: float):
    import colorsys
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 0.9)
    return r, g, b


def main(args=None):
    rclpy.init(args=args)
    node = MAPFPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
