#!/usr/bin/env python3
"""
mapf_full.launch.py — Master launch file.

Launches in order:
  1. Gazebo 11 with mapf_world.world
  2. mapf_planner_node  — runs PBS, publishes paths + grid map
  3. world_spawner_node — waits for Gazebo, spawns obstacles + robots
  4. agent_controller_node — P-controller for all agents
  5. RViz2              — pre-configured visualisation

Usage:
  ros2 launch mapf_ros mapf_full.launch.py \
    agent_count:=100 grid_size:=20 obstacle_density:=0.12
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess, LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = get_package_share_directory("mapf_ros")

    # ── Declare arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("agent_count",      default_value="100",
                              description="Number of agents"),
        DeclareLaunchArgument("grid_size",         default_value="20",
                              description="Grid cols = rows"),
        DeclareLaunchArgument("obstacle_density",  default_value="0.12",
                              description="Obstacle density 0.0–0.35"),
        DeclareLaunchArgument("random_seed",       default_value="42",
                              description="RNG seed for reproducibility"),
        DeclareLaunchArgument("use_rviz",          default_value="true",
                              description="Launch RViz2"),
        DeclareLaunchArgument("use_gui",           default_value="true",
                              description="Show Gazebo GUI"),
    ]

    agent_count      = LaunchConfiguration("agent_count")
    grid_size        = LaunchConfiguration("grid_size")
    obstacle_density = LaunchConfiguration("obstacle_density")
    random_seed      = LaunchConfiguration("random_seed")
    use_rviz         = LaunchConfiguration("use_rviz")
    use_gui          = LaunchConfiguration("use_gui")

    # Shared params dict
    common_params = [{"agent_count":      agent_count},
                     {"grid_size":        grid_size},
                     {"obstacle_density": obstacle_density},
                     {"random_seed":      random_seed}]

    # ── Gazebo ────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_share, "worlds", "mapf_world.world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world":   world_file,
            "verbose": "false",
            "gui":     use_gui,
        }.items(),
    )

    # ── MAPF Planner (runs PBS immediately) ───────────────────────────────
    planner_node = Node(
        package="mapf_ros",
        executable="mapf_planner",
        name="mapf_planner",
        output="screen",
        parameters=common_params,
    )

    # ── World Spawner (waits for Gazebo, then spawns everything) ──────────
    # Delay 3s to give Gazebo time to fully initialise
    spawner_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="mapf_ros",
                executable="world_spawner",
                name="world_spawner",
                output="screen",
                parameters=common_params,
            )
        ],
    )

    # ── Agent Controller (starts after spawner has had time to work) ──────
    controller_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="mapf_ros",
                executable="agent_controller",
                name="agent_controller",
                output="screen",
                parameters=[{"agent_count": agent_count}],
            )
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "rviz", "mapf.rviz")
    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=None,  # always launch
            )
        ],
    )

    # ── Log startup info ───────────────────────────────────────────────────
    log_info = LogInfo(msg=[
        "\n\n",
        "╔══════════════════════════════════════════════╗\n",
        "║     MAPF Simulator — ROS2 Humble + Gazebo   ║\n",
        "║  Agents: ", agent_count,
        "  Grid: ", grid_size, "×", grid_size, "\n",
        "║  Browser dashboard: http://localhost:8765   ║\n",
        "╚══════════════════════════════════════════════╝\n",
    ])

    return LaunchDescription([
        *args,
        log_info,
        gazebo,
        planner_node,
        spawner_node,
        controller_node,
        rviz_node,
    ])
