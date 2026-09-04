"""
agent.py — Agent Model (Python / ROS2-ready)
"""

from __future__ import annotations
from enum import Enum
from typing import List, Optional


class AgentState(str, Enum):
    IDLE     = "IDLE"
    PLANNING = "PLANNING"
    MOVING   = "MOVING"
    WAITING  = "WAITING"
    ARRIVED  = "ARRIVED"
    STUCK    = "STUCK"


class Agent:
    """Represents one autonomous agent in the MAPF system.

    Path is a list of {"x", "y", "t"} dicts produced by Space-Time A*.
    The agent tracks its logical (grid) position and computes the next
    waypoint for the physical controller.
    """

    def __init__(self, agent_id: int, start: dict, goal: dict, priority: int) -> None:
        self.id       = agent_id
        self.start    = dict(start)
        self.goal     = dict(goal)
        self.priority = priority

        # Current logical grid position
        self.grid_x: int = start["x"]
        self.grid_y: int = start["y"]

        # Planned space-time path: [{"x","y","t"}, ...]
        self.path: List[dict] = []
        self.path_index: int  = 0

        self.state: AgentState = AgentState.IDLE
        self.start_time: int   = 0

        # Stats
        self.steps_taken: int      = 0
        self.wait_steps:  int      = 0
        self.replan_count: int     = 0
        self.conflicts_involved: int = 0

        # For the physical controller: target waypoint in world coords
        # Updated by the controller when waypoint is reached
        self._current_waypoint_idx: int = 0

    # ── Path assignment ────────────────────────────────────────────────────

    def assign_path(self, path: Optional[List[dict]]) -> None:
        self.path       = path or []
        self.path_index = 0
        if not self.path:
            self.state = AgentState.STUCK
        else:
            self.state = AgentState.MOVING

    def has_remaining_path(self) -> bool:
        return self.path_index < len(self.path) - 1

    # ── Logical step (advances grid position per timestep) ─────────────────

    def step(self, current_t: int) -> None:
        if self.state in (AgentState.ARRIVED, AgentState.STUCK):
            return
        if not self.path:
            self.state = AgentState.STUCK
            return

        idx = self.path_index
        if idx >= len(self.path):
            self.state = AgentState.ARRIVED
            return

        entry = self.path[idx]
        if entry["t"] > current_t:
            self.state = AgentState.WAITING
            return

        moved = (entry["x"] != self.grid_x or entry["y"] != self.grid_y)
        self.grid_x = entry["x"]
        self.grid_y = entry["y"]

        if moved:
            self.steps_taken += 1
        else:
            self.wait_steps += 1

        if self.path_index + 1 < len(self.path):
            self.path_index += 1

        if self.grid_x == self.goal["x"] and self.grid_y == self.goal["y"]:
            self.state = AgentState.ARRIVED
        else:
            self.state = AgentState.MOVING if moved else AgentState.WAITING

    # ── Physical controller waypoint helpers ───────────────────────────────

    def next_world_waypoint(self) -> Optional[tuple]:
        """Return (world_x, world_y) of the next waypoint in the plan.

        Returns None if the agent has completed its path.
        """
        if not self.path:
            return None
        idx = min(self._current_waypoint_idx, len(self.path) - 1)
        p   = self.path[idx]
        return float(p["x"]) + 0.5, float(p["y"]) + 0.5

    def advance_waypoint(self) -> None:
        """Called by controller when current waypoint is reached."""
        if self._current_waypoint_idx < len(self.path) - 1:
            self._current_waypoint_idx += 1

    @property
    def progress(self) -> float:
        if len(self.path) <= 1:
            return 1.0 if self.state == AgentState.ARRIVED else 0.0
        return self.path_index / (len(self.path) - 1)

    # ── Serialization ─────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "id":       self.id,
            "state":    self.state.value,
            "grid_x":   self.grid_x,
            "grid_y":   self.grid_y,
            "goal_x":   self.goal["x"],
            "goal_y":   self.goal["y"],
            "priority": self.priority,
            "progress": round(self.progress, 3),
            "steps":    self.steps_taken,
            "waits":    self.wait_steps,
            "replans":  self.replan_count,
        }

    def reset(self, new_start: dict, new_goal: dict, priority: int) -> None:
        self.__init__(self.id, new_start, new_goal, priority)
