#!/usr/bin/env python3
"""
world_spawner_node.py — ROS2 Node: Gazebo World Spawner

Spawns all entities in Gazebo after the world is ready:
  1. Waits for /spawn_entity service (confirms Gazebo is running).
  2. Subscribes to /mapf/status to get agent start positions.
  3. Spawns obstacle boxes at grid obstacle positions.
  4. Spawns mapf_bot robot models at each agent's start position.
  5. Exits after all entities are spawned.

Robot SDF is generated dynamically per-agent with correct namespace
and colour so each robot is visually distinct.
"""

import math
import os
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg    import String
from ament_index_python.packages import get_package_share_directory
import json


def _hue_to_rgba_str(hue: float) -> str:
    """HSV hue (0-360) → Gazebo RGBA string '0.3 0.8 0.9 1.0'."""
    import colorsys
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 0.85)
    return f"{r:.3f} {g:.3f} {b:.3f} 1.0"


def _make_obstacle_sdf(ox: float, oy: float, entity_id: int) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstacle_{entity_id}">
    <static>true</static>
    <pose>{ox + 0.5} {oy + 0.5} 0.15 0 0 0</pose>
    <link name="link">
      <collision name="col">
        <geometry><box><size>0.95 0.95 0.30</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>0.95 0.95 0.30</size></box></geometry>
        <material>
          <ambient>0.08 0.15 0.30 1.0</ambient>
          <diffuse>0.10 0.20 0.40 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _make_robot_sdf(agent_id: int, namespace: str, colour_rgba: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{namespace}">
    <pose>0 0 0.12 0 0 0</pose>

    <link name="base_footprint">
      <pose>0 0 0 0 0 0</pose>
      <inertial><mass>0.001</mass></inertial>
    </link>

    <link name="base_link">
      <pose>0 0 0.08 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia><ixx>0.13</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.13</iyy><iyz>0</iyz><izz>0.21</izz></inertia>
      </inertial>
      <visual name="body">
        <geometry><cylinder><radius>0.22</radius><length>0.16</length></cylinder></geometry>
        <material>
          <ambient>{colour_rgba}</ambient>
          <diffuse>{colour_rgba}</diffuse>
          <specular>0.2 0.2 0.2 1.0</specular>
        </material>
      </visual>
      <collision name="body_col">
        <geometry><cylinder><radius>0.22</radius><length>0.16</length></cylinder></geometry>
      </collision>
    </link>

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent><child>base_link</child>
    </joint>

    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0.0 0.20 0.05 1.5708 0 0</pose>
      <inertial><mass>0.5</mass>
        <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
      </inertial>
      <visual name="vis">
        <geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent><child>left_wheel</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>

    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0.0 -0.20 0.05 1.5708 0 0</pose>
      <inertial><mass>0.5</mass>
        <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
      </inertial>
      <visual name="vis">
        <geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent><child>right_wheel</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>

    <!-- Rear caster -->
    <link name="caster">
      <pose>-0.18 0 0.025 0 0 0</pose>
      <inertial><mass>0.1</mass></inertial>
      <visual name="vis">
        <geometry><sphere><radius>0.025</radius></sphere></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient></material>
      </visual>
      <collision name="col">
        <geometry><sphere><radius>0.025</radius></sphere></geometry>
        <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <joint name="caster_joint" type="ball">
      <parent>base_link</parent><child>caster</child>
    </joint>

    <!-- Differential Drive Plugin -->
    <plugin name="diff_drive_{agent_id}" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/{namespace}</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>20</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.40</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>/{namespace}/odom</odometry_frame>
      <robot_base_frame>/{namespace}/base_footprint</robot_base_frame>
    </plugin>

  </model>
</sdf>"""


class WorldSpawnerNode(Node):

    def __init__(self) -> None:
        super().__init__("world_spawner")

        self.declare_parameter("agent_count",     100)
        self.declare_parameter("grid_size",        20)
        self.declare_parameter("obstacle_density", 0.12)
        self.declare_parameter("random_seed",      42)

        self.agent_count      = self.get_parameter("agent_count").value
        self.grid_size        = self.get_parameter("grid_size").value
        self.obstacle_density = self.get_parameter("obstacle_density").value
        self.seed             = self.get_parameter("random_seed").value

        # We receive the agent start/goal positions from the planner via /mapf/status
        self._agent_starts: dict = {}  # agent_id → (wx, wy)
        self._received_status = False

        self._status_sub = self.create_subscription(
            String, "/mapf/status", self._status_callback, 10
        )

        # Wait for Gazebo spawn service
        self._spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for /spawn_entity service (Gazebo)…")

        # Trigger spawn after receiving first status + service ready
        self._spawn_done = False
        self.create_timer(1.0, self._try_spawn)

    def _status_callback(self, msg: String) -> None:
        if self._received_status:
            return
        try:
            data = json.loads(msg.data)
            for ag in data.get("agents", []):
                wx = float(ag["grid_x"]) + 0.5
                wy = float(ag["grid_y"]) + 0.5
                self._agent_starts[ag["id"]] = (wx, wy)
            self._received_status = True
            self.get_logger().info(
                f"Received positions for {len(self._agent_starts)} agents."
            )
        except Exception as e:
            self.get_logger().error(f"Status parse error: {e}")

    def _try_spawn(self) -> None:
        if self._spawn_done:
            return
        if not self._spawn_client.service_is_ready():
            self.get_logger().info("Waiting for Gazebo /spawn_entity…", throttle_duration_sec=5)
            return
        if not self._received_status:
            self.get_logger().info("Waiting for /mapf/status…", throttle_duration_sec=5)
            return

        self.get_logger().info("Gazebo ready — spawning entities…")
        self._spawn_done = True
        self._do_spawn()

    def _do_spawn(self) -> None:
        from mapf_ros.core import Grid

        grid = Grid(self.grid_size, self.grid_size)
        grid.generate_obstacles(density=self.obstacle_density, seed=self.seed)

        # ── Spawn obstacles ────────────────────────────────────────────────
        obstacles = grid.obstacle_positions()
        self.get_logger().info(f"Spawning {len(obstacles)} obstacle blocks…")
        for eid, (ox, oy) in enumerate(obstacles):
            sdf = _make_obstacle_sdf(float(ox), float(oy), eid)
            self._spawn_sync(f"obstacle_{eid}", sdf, 0.0, 0.0, 0.0)

        # ── Spawn robots ───────────────────────────────────────────────────
        self.get_logger().info(f"Spawning {len(self._agent_starts)} robots…")
        for agent_id, (wx, wy) in self._agent_starts.items():
            ns     = f"agent_{agent_id}"
            hue    = (agent_id * 137.5) % 360.0
            colour = _hue_to_rgba_str(hue)
            sdf    = _make_robot_sdf(agent_id, ns, colour)
            self._spawn_sync(ns, sdf, wx, wy, 0.12)

        self.get_logger().info("✅ All entities spawned successfully!")

    def _spawn_sync(self, name: str, sdf: str, x: float, y: float, z: float) -> None:
        req = SpawnEntity.Request()
        req.name              = name
        req.xml               = sdf
        req.robot_namespace   = name
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.w = 1.0
        req.reference_frame  = "world"

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self.get_logger().debug(f"Spawned: {name}")
        else:
            err = future.result().status_message if future.result() else "timeout"
            self.get_logger().warn(f"Spawn failed for {name}: {err}")


def main(args=None):
    rclpy.init(args=args)
    node = WorldSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
