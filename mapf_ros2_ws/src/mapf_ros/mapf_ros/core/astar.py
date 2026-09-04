"""
astar.py — Space-Time A* + Reservation Table (Python)
Mirrors the JS implementation: MinHeap + standard A* + space-time variant.
"""

from __future__ import annotations
import heapq
from typing import Dict, List, Optional, Set, Tuple


# ── Reservation Table ──────────────────────────────────────────────────────

class ReservationTable:
    """Space-time reservation table shared by all agents during PBS planning."""

    def __init__(self) -> None:
        self._vertices: Dict[Tuple[int,int,int], int] = {}  # (x,y,t) → agent_id
        self._edges:    Dict[Tuple[int,int,int,int,int], int] = {}  # (x1,y1,x2,y2,t) → agent_id

    def clear(self) -> None:
        self._vertices.clear()
        self._edges.clear()

    def reserve_path(self, path: List[dict], agent_id: int, max_time: int = 400) -> None:
        """Reserve all (x,y,t) triples along a space-time path."""
        if not path:
            return
        for i, node in enumerate(path):
            x, y, t = node["x"], node["y"], node["t"]
            self._vertices[(x, y, t)] = agent_id
            if i < len(path) - 1:
                nx, ny, nt = path[i+1]["x"], path[i+1]["y"], path[i+1]["t"]
                # Forward edge
                self._edges[(x, y, nx, ny, t)] = agent_id
                # Block the swap (edge conflict)
                self._edges[(nx, ny, x, y, t)] = agent_id
        # Reserve goal indefinitely to block others from passing through
        last = path[-1]
        for tt in range(last["t"] + 1, max_time + 1):
            self._vertices[(last["x"], last["y"], tt)] = agent_id

    def is_vertex_reserved(self, x: int, y: int, t: int) -> bool:
        return (x, y, t) in self._vertices

    def is_edge_reserved(self, fx: int, fy: int, tx: int, ty: int, t: int) -> bool:
        return (fx, fy, tx, ty, t) in self._edges

    def free_agent(self, agent_id: int) -> None:
        """Remove all reservations belonging to agent_id (for replanning)."""
        self._vertices = {k: v for k, v in self._vertices.items() if v != agent_id}
        self._edges    = {k: v for k, v in self._edges.items()    if v != agent_id}

    def who_owns(self, x: int, y: int, t: int) -> Optional[int]:
        return self._vertices.get((x, y, t))


# ── A* Engine ─────────────────────────────────────────────────────────────

class AStar:
    """A* pathfinder on a Grid instance.

    Provides both standard A* (no time) and space-time A* (with reservations).
    """

    def __init__(self, grid) -> None:
        self.grid = grid

    @staticmethod
    def _heuristic(ax: int, ay: int, bx: int, by: int) -> int:
        return abs(ax - bx) + abs(ay - by)

    def find_path(self, sx: int, sy: int, gx: int, gy: int) -> Optional[List[Tuple[int,int]]]:
        """Standard A* without time dimension.

        Returns list of (x, y) from start (exclusive) to goal (inclusive), or None.
        """
        grid = self.grid
        if not grid.is_walkable(gx, gy):
            return None

        # heap entries: (f, g, x, y)
        open_heap: List[Tuple] = []
        heapq.heappush(open_heap, (self._heuristic(sx, sy, gx, gy), 0, sx, sy))

        came_from: Dict[Tuple[int,int], Optional[Tuple[int,int]]] = {(sx, sy): None}
        g_score:   Dict[Tuple[int,int], int] = {(sx, sy): 0}

        while open_heap:
            _, g, cx, cy = heapq.heappop(open_heap)

            if cx == gx and cy == gy:
                return self._reconstruct(came_from, gx, gy)

            if g > g_score.get((cx, cy), float("inf")):
                continue  # stale entry

            for nx, ny in grid.get_neighbors(cx, cy):
                ng = g + 1
                if ng < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = ng
                    came_from[(nx, ny)] = (cx, cy)
                    h = self._heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (ng + h, ng, nx, ny))

        return None

    def find_path_space_time(
        self,
        sx: int, sy: int,
        gx: int, gy: int,
        reservations: ReservationTable,
        start_time:   int = 0,
        max_time:     int = 400,
    ) -> Optional[List[dict]]:
        """Space-Time A* with reservation avoidance.

        Returns list of {"x", "y", "t"} dicts from start_time to arrival, or None.
        """
        grid = self.grid
        if not grid.is_walkable(gx, gy):
            return None

        # heap: (f, g, x, y, t)
        h0 = self._heuristic(sx, sy, gx, gy)
        open_heap: List[Tuple] = [(h0, 0, sx, sy, start_time)]

        came_from: Dict[Tuple, Optional[Tuple]] = {(sx, sy, start_time): None}
        g_score:   Dict[Tuple, int] = {(sx, sy, start_time): 0}

        while open_heap:
            _, g, cx, cy, ct = heapq.heappop(open_heap)

            if cx == gx and cy == gy:
                return self._reconstruct_st(came_from, cx, cy, ct, start_time)

            if ct >= max_time:
                continue
            if g > g_score.get((cx, cy, ct), float("inf")):
                continue  # stale

            # Expand: neighbours + wait-in-place
            moves = list(grid.get_neighbors(cx, cy)) + [(cx, cy)]
            for nx, ny in moves:
                nt = ct + 1
                # Vertex conflict check
                if reservations.is_vertex_reserved(nx, ny, nt):
                    continue
                # Edge (swap) conflict check
                if reservations.is_edge_reserved(cx, cy, nx, ny, ct):
                    continue

                ng = g + 1
                state = (nx, ny, nt)
                if ng < g_score.get(state, float("inf")):
                    g_score[state] = ng
                    came_from[state] = (cx, cy, ct)
                    h = self._heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_heap, (ng + h, ng, nx, ny, nt))

        return None

    # ── Path reconstruction ────────────────────────────────────────────────

    @staticmethod
    def _reconstruct(came_from: dict, x: int, y: int) -> List[Tuple[int,int]]:
        path = []
        curr: Optional[Tuple] = (x, y)
        while curr is not None:
            path.append(curr)
            curr = came_from.get(curr)
        path.reverse()
        return path[1:]  # exclude start cell

    @staticmethod
    def _reconstruct_st(
        came_from: dict,
        x: int, y: int, t: int,
        start_time: int,
    ) -> List[dict]:
        path = []
        state: Optional[Tuple] = (x, y, t)
        while state is not None:
            path.append({"x": state[0], "y": state[1], "t": state[2]})
            state = came_from.get(state)
        path.reverse()
        return path
