"""
grid.py — Grid Environment (Python port, ROS2-ready)
Identical logic to the browser JS version; no ROS dependencies.
"""

from __future__ import annotations
import random
from collections import deque
from typing import List, Tuple, Set, Optional

CELL_EMPTY    = 0
CELL_OBSTACLE = 1


class Grid:
    """2-D grid environment for MAPF.

    Coordinates:
      x → column (east), y → row (south), origin top-left.
    World coordinates (Gazebo):
      world_x = x + 0.5  (cell centre)
      world_y = y + 0.5
    """

    def __init__(self, cols: int, rows: int) -> None:
        self.cols = cols
        self.rows = rows
        self._cells: bytearray = bytearray(cols * rows)  # all EMPTY

    # ── Accessors ──────────────────────────────────────────────────────────

    def _idx(self, x: int, y: int) -> int:
        return y * self.cols + x

    def get_cell(self, x: int, y: int) -> int:
        return self._cells[self._idx(x, y)]

    def set_cell(self, x: int, y: int, ctype: int) -> None:
        self._cells[self._idx(x, y)] = ctype

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.cols and 0 <= y < self.rows

    def is_walkable(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self._cells[self._idx(x, y)] != CELL_OBSTACLE

    def is_obstacle(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self._cells[self._idx(x, y)] == CELL_OBSTACLE

    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """4-directional walkable neighbours."""
        result = []
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny):
                result.append((nx, ny))
        return result

    # ── Obstacle generation ────────────────────────────────────────────────

    def clear(self) -> None:
        self._cells = bytearray(len(self._cells))

    def generate_obstacles(self, density: float, protect: Set[str] | None = None,
                           seed: int | None = None) -> None:
        """Random obstacle placement, protecting specific cells."""
        rng = random.Random(seed)
        self.clear()
        if protect is None:
            protect = set()

        total   = self.cols * self.rows
        count   = int(total * density)
        placed  = 0
        attempts = 0
        max_att  = count * 25

        while placed < count and attempts < max_att:
            attempts += 1
            x = rng.randrange(self.cols)
            y = rng.randrange(self.rows)
            key = f"{x},{y}"
            if key not in protect and self._cells[self._idx(x, y)] == CELL_EMPTY:
                self._cells[self._idx(x, y)] = CELL_OBSTACLE
                placed += 1

    # ── Connectivity ───────────────────────────────────────────────────────

    def is_connected(self, x1: int, y1: int, x2: int, y2: int) -> bool:
        """BFS connectivity check between two walkable cells."""
        if not self.is_walkable(x1, y1) or not self.is_walkable(x2, y2):
            return False
        visited: Set[str] = {f"{x1},{y1}"}
        q: deque = deque([(x1, y1)])
        while q:
            cx, cy = q.popleft()
            if cx == x2 and cy == y2:
                return True
            for nx, ny in self.get_neighbors(cx, cy):
                k = f"{nx},{ny}"
                if k not in visited:
                    visited.add(k)
                    q.append((nx, ny))
        return False

    def reachable_cells(self, sx: int, sy: int) -> List[dict]:
        """All cells reachable from (sx, sy) via BFS."""
        visited: Set[str] = {f"{sx},{sy}"}
        q: deque = deque([(sx, sy)])
        result = []
        while q:
            cx, cy = q.popleft()
            result.append({"x": cx, "y": cy})
            for nx, ny in self.get_neighbors(cx, cy):
                k = f"{nx},{ny}"
                if k not in visited:
                    visited.add(k)
                    q.append((nx, ny))
        return result

    # ── Random position sampling ───────────────────────────────────────────

    def random_walkable_cell(self, exclude: Set[str] | None = None,
                             rng: random.Random | None = None) -> Optional[dict]:
        if rng is None:
            rng = random.Random()
        if exclude is None:
            exclude = set()
        walkable = [
            {"x": x, "y": y}
            for y in range(self.rows)
            for x in range(self.cols)
            if self.is_walkable(x, y) and f"{x},{y}" not in exclude
        ]
        if not walkable:
            return None
        return rng.choice(walkable)

    def random_walkable_pairs(self, n: int, seed: int | None = None,
                              max_tries: int = 40) -> Tuple[List[dict], List[dict]]:
        """Pick n (start, goal) pairs that are pairwise reachable."""
        rng = random.Random(seed)
        starts, goals = [], []
        used: Set[str] = set()

        for _ in range(n):
            s = g = None
            for attempt in range(max_tries):
                s = self.random_walkable_cell(exclude=used, rng=rng)
                if s is None:
                    break
                tmp = set(used) | {f"{s['x']},{s['y']}"}
                g = self.random_walkable_cell(exclude=tmp, rng=rng)
                if g and self.is_connected(s["x"], s["y"], g["x"], g["y"]):
                    break
                g = None

            if s and g:
                used.add(f"{s['x']},{s['y']}")
                used.add(f"{g['x']},{g['y']}")
                starts.append(s)
                goals.append(g)

        return starts, goals

    # ── Coordinate helpers ─────────────────────────────────────────────────

    @staticmethod
    def cell_to_world(cx: int, cy: int) -> Tuple[float, float]:
        """Grid cell index → Gazebo world XY (cell centre, 1m cells)."""
        return float(cx) + 0.5, float(cy) + 0.5

    @staticmethod
    def world_to_cell(wx: float, wy: float) -> Tuple[int, int]:
        return int(wx), int(wy)

    # ── Occupancy grid (for ROS OccupancyGrid msg) ─────────────────────────

    def to_occupancy_data(self) -> List[int]:
        """Returns flat list of ROS OccupancyGrid data (0=free, 100=occupied)."""
        return [100 if c == CELL_OBSTACLE else 0 for c in self._cells]

    def obstacle_positions(self) -> List[Tuple[int, int]]:
        """List of (x, y) for every obstacle cell."""
        return [
            (x, y)
            for y in range(self.rows)
            for x in range(self.cols)
            if self.is_obstacle(x, y)
        ]
