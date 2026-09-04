"""
mapf_coordinator.py — Priority-Based Search (PBS) MAPF Planner
Identical algorithm to the JS mapf.js version; no ROS dependencies.
"""

from __future__ import annotations
import logging
from typing import Dict, List, Optional, Tuple

from .grid  import Grid
from .astar import AStar, ReservationTable
from .agent import Agent, AgentState

logger = logging.getLogger(__name__)


class MAPFCoordinator:
    """Priority-Based Search coordinator.

    Planning:
      - Agents are sorted by priority (lower int = higher priority = plans first).
      - Each agent plans a Space-Time A* path avoiding earlier agents' reservations.

    Runtime conflict detection:
      - After every timestep, vertex conflicts are detected.
      - The lower-priority conflicting agent is replanned from its current position.

    Statistics collected:
      total_plans, total_replans, conflict_count
    """

    MAX_TIME = 400  # space-time horizon (timesteps)

    def __init__(self, grid: Grid, agents: List[Agent]) -> None:
        self.grid         = grid
        self.agents       = agents
        self.astar        = AStar(grid)
        self.reservations = ReservationTable()

        self.total_plans:    int = 0
        self.total_replans:  int = 0
        self.conflict_count: int = 0

    # ── Initial PBS planning ───────────────────────────────────────────────

    def plan_all(self, start_time: int = 0) -> None:
        """Run Priority-Based Search for all agents from scratch."""
        self.reservations.clear()
        ordered = sorted(self.agents, key=lambda a: a.priority)
        for agent in ordered:
            self._plan_agent(agent, start_time)
        logger.info(
            "PBS complete: %d agents planned, %d stuck",
            sum(1 for a in self.agents if a.state != AgentState.STUCK),
            sum(1 for a in self.agents if a.state == AgentState.STUCK),
        )

    def _plan_agent(self, agent: Agent, start_time: int) -> None:
        """(Re)plan a single agent; frees its old reservations first."""
        self.reservations.free_agent(agent.id)

        path = self.astar.find_path_space_time(
            agent.start["x"], agent.start["y"],
            agent.goal["x"],  agent.goal["y"],
            self.reservations,
            start_time=start_time,
            max_time=self.MAX_TIME,
        )

        agent.start_time = start_time
        agent.assign_path(path)

        if path:
            self.reservations.reserve_path(path, agent.id, self.MAX_TIME)
        else:
            logger.warning("Agent %d: no path found (STUCK)", agent.id)

        self.total_plans += 1

    # ── Runtime step ──────────────────────────────────────────────────────

    def step(self, current_t: int) -> List[dict]:
        """Advance all agents one timestep; detect and resolve conflicts.

        Returns list of conflict dicts detected at this step.
        """
        for agent in self.agents:
            agent.step(current_t)

        conflicts = self._detect_conflicts(current_t + 1)
        if conflicts:
            self._resolve_conflicts(conflicts, current_t + 1)

        return conflicts

    # ── Conflict detection ─────────────────────────────────────────────────

    def _detect_conflicts(self, t: int) -> List[dict]:
        """Detect vertex conflicts at timestep t."""
        pos_map: Dict[Tuple[int,int], int] = {}
        conflicts = []

        for agent in self.agents:
            if agent.state in (AgentState.ARRIVED, AgentState.STUCK):
                continue
            pos = (agent.grid_x, agent.grid_y)
            if pos in pos_map:
                conflicts.append({
                    "type":   "vertex",
                    "agent_a": pos_map[pos],
                    "agent_b": agent.id,
                    "x": agent.grid_x,
                    "y": agent.grid_y,
                    "t": t,
                })
            else:
                pos_map[pos] = agent.id

        self.conflict_count += len(conflicts)
        return conflicts

    def _resolve_conflicts(self, conflicts: List[dict], t: int) -> None:
        """Lower-priority agent replans from current position."""
        for conflict in conflicts:
            a = self._get_agent(conflict["agent_a"])
            b = self._get_agent(conflict["agent_b"])
            if a is None or b is None:
                continue

            loser = a if a.priority > b.priority else b
            loser.start = {"x": loser.grid_x, "y": loser.grid_y}
            loser.replan_count      += 1
            loser.conflicts_involved += 1
            self.total_replans      += 1

            self._plan_agent(loser, t)
            logger.debug("Conflict @ (%d,%d,t=%d): agent %d replanned",
                         conflict["x"], conflict["y"], t, loser.id)

    # ── Stats ──────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        states = {s.value: 0 for s in AgentState}
        total_steps = total_wait = 0
        for ag in self.agents:
            states[ag.state.value] += 1
            total_steps += ag.steps_taken
            total_wait  += ag.wait_steps
        return {
            **states,
            "total_agents":   len(self.agents),
            "total_plans":    self.total_plans,
            "total_replans":  self.total_replans,
            "conflict_count": self.conflict_count,
            "total_steps":    total_steps,
            "total_wait":     total_wait,
        }

    def all_done(self) -> bool:
        return all(a.state in (AgentState.ARRIVED, AgentState.STUCK)
                   for a in self.agents)

    def _get_agent(self, agent_id: int) -> Optional[Agent]:
        for a in self.agents:
            if a.id == agent_id:
                return a
        return None

    def reset(self) -> None:
        self.reservations.clear()
        self.total_plans    = 0
        self.total_replans  = 0
        self.conflict_count = 0
