#!/usr/bin/env python3
"""
agent_controller_node.py — ROS2 Node: Physical Agent Controller

A single node that controls ALL agents in Gazebo simultaneously.

For each agent i it:
  - Subscribes  /mapf/agent_{i}/plan  → receives the planned waypoints
  - Subscribes  /agent_{i}/odom       → receives current pose from Gazebo
  - Publishes   /agent_{i}/cmd_vel    → sends Twist commands to Gazebo

Control law (P-controller):
  1. Compute heading to next waypoint.
  2. If |heading_error| > threshold: rotate in-place.
  3. Else: drive forward with speed proportional to distance.
  4. When within ARRIVE_DIST of waypoint: advance to next.
  5. Stop when all waypoints consumed.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg      import Path, Odometry
from geometry_msgs.msg import Twist

# Control gains
KP_LINEAR  = 0.8   # forward speed per metre of distance
KP_ANGULAR = 2.5   # angular speed per radian of heading error
MAX_LINEAR = 0.45  # m/s
MAX_ANGULAR= 1.5   # rad/s
ARRIVE_DIST = 0.12 # metres — snap to waypoint when within this radius
HEADING_THRESHOLD = 0.15  # rad — start moving forward when aligned


def _normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class AgentControllerNode(Node):

    def __init__(self) -> None:
        super().__init__("agent_controller")

        self.declare_parameter("agent_count", 100)
        self.agent_count = self.get_parameter("agent_count").value

        # Latching QoS to receive plans that were published before we started
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Per-agent state
        self._waypoints:   list = [[] for _ in range(self.agent_count)]  # list of (wx,wy)
        self._wp_idx:      list = [0  for _ in range(self.agent_count)]
        self._current_x:   list = [None] * self.agent_count
        self._current_y:   list = [None] * self.agent_count
        self._current_yaw: list = [0.0] * self.agent_count
        self._arrived:     list = [False] * self.agent_count

        # Publishers and subscribers
        self._cmd_pubs  = []
        self._plan_subs = []
        self._odom_subs = []

        for i in range(self.agent_count):
            # cmd_vel publisher
            self._cmd_pubs.append(
                self.create_publisher(Twist, f"/agent_{i}/cmd_vel", 10)
            )
            # plan subscriber (latched)
            self._plan_subs.append(
                self.create_subscription(
                    Path, f"/mapf/agent_{i}/plan",
                    lambda msg, idx=i: self._plan_callback(msg, idx),
                    latch_qos,
                )
            )
            # odom subscriber
            self._odom_subs.append(
                self.create_subscription(
                    Odometry, f"/agent_{i}/odom",
                    lambda msg, idx=i: self._odom_callback(msg, idx),
                    10,
                )
            )

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"Agent controller ready for {self.agent_count} agents."
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _plan_callback(self, msg: Path, idx: int) -> None:
        waypoints = []
        for pose in msg.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            waypoints.append((wx, wy))
        self._waypoints[idx] = waypoints
        self._wp_idx[idx]    = 0
        self._arrived[idx]   = False
        self.get_logger().debug(
            f"Agent {idx}: received plan with {len(waypoints)} waypoints"
        )

    def _odom_callback(self, msg: Odometry, idx: int) -> None:
        self._current_x[idx] = msg.pose.pose.position.x
        self._current_y[idx] = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw[idx] = math.atan2(siny_cosp, cosy_cosp)

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        for i in range(self.agent_count):
            self._control_agent(i)

    def _control_agent(self, idx: int) -> None:
        twist = Twist()  # default: zero velocity

        if self._arrived[idx]:
            self._cmd_pubs[idx].publish(twist)
            return

        wps = self._waypoints[idx]
        if not wps:
            self._cmd_pubs[idx].publish(twist)
            return

        cx = self._current_x[idx]
        cy = self._current_y[idx]
        if cx is None or cy is None:
            # Odometry not yet received
            self._cmd_pubs[idx].publish(twist)
            return

        wp_idx = self._wp_idx[idx]
        if wp_idx >= len(wps):
            self._arrived[idx] = True
            self._cmd_pubs[idx].publish(twist)
            return

        tx, ty = wps[wp_idx]
        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_DIST:
            # Reached this waypoint — advance
            self._wp_idx[idx] += 1
            if self._wp_idx[idx] >= len(wps):
                self._arrived[idx] = True
                self.get_logger().info(f"Agent {idx}: ARRIVED at goal ✓")
            self._cmd_pubs[idx].publish(twist)
            return

        # Compute heading error
        target_heading  = math.atan2(dy, dx)
        heading_error   = _normalize_angle(target_heading - self._current_yaw[idx])

        if abs(heading_error) > HEADING_THRESHOLD:
            # Rotate in place first
            twist.angular.z = max(-MAX_ANGULAR,
                                  min(MAX_ANGULAR, KP_ANGULAR * heading_error))
            twist.linear.x  = 0.0
        else:
            # Move forward + fine angular correction
            twist.linear.x  = min(MAX_LINEAR, KP_LINEAR * dist)
            twist.angular.z = max(-MAX_ANGULAR,
                                  min(MAX_ANGULAR, KP_ANGULAR * heading_error))

        self._cmd_pubs[idx].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AgentControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
