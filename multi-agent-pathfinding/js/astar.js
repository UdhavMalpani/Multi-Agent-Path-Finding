/**
 * astar.js — A* Pathfinding Engine
 * Provides both standard A* and space-time A* (for MAPF conflict avoidance).
 * Space-time A* nodes = (x, y, t) so agents can plan around reservations.
 */

'use strict';

// ---------------------------------------------------------------------------
// MinHeap — generic binary min-heap keyed by a numeric priority
// ---------------------------------------------------------------------------
class MinHeap {
  constructor() {
    this._data = [];
  }

  get size() { return this._data.length; }
  isEmpty() { return this._data.length === 0; }

  push(item) {
    this._data.push(item);
    this._bubbleUp(this._data.length - 1);
  }

  pop() {
    if (this.isEmpty()) return null;
    const top = this._data[0];
    const last = this._data.pop();
    if (this._data.length > 0) {
      this._data[0] = last;
      this._siftDown(0);
    }
    return top;
  }

  peek() { return this._data[0] ?? null; }

  _bubbleUp(i) {
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this._data[parent].f <= this._data[i].f) break;
      [this._data[parent], this._data[i]] = [this._data[i], this._data[parent]];
      i = parent;
    }
  }

  _siftDown(i) {
    const n = this._data.length;
    while (true) {
      let smallest = i;
      const l = 2 * i + 1, r = 2 * i + 2;
      if (l < n && this._data[l].f < this._data[smallest].f) smallest = l;
      if (r < n && this._data[r].f < this._data[smallest].f) smallest = r;
      if (smallest === i) break;
      [this._data[smallest], this._data[i]] = [this._data[i], this._data[smallest]];
      i = smallest;
    }
  }
}

// ---------------------------------------------------------------------------
// AStar — pathfinding on a Grid instance
// ---------------------------------------------------------------------------
class AStar {
  /**
   * @param {Grid} grid
   */
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Manhattan distance heuristic.
   */
  heuristic(ax, ay, bx, by) {
    return Math.abs(ax - bx) + Math.abs(ay - by);
  }

  /**
   * Standard A* — no time dimension, no reservation table.
   * Returns array of {x, y} from start (exclusive) to goal (inclusive),
   * or null if unreachable.
   */
  findPath(sx, sy, gx, gy) {
    const grid = this.grid;
    if (!grid.isWalkable(gx, gy)) return null;

    const openSet = new MinHeap();
    const cameFrom = new Map();
    const gScore = new Map();

    const startKey = `${sx},${sy}`;
    gScore.set(startKey, 0);
    openSet.push({ x: sx, y: sy, f: this.heuristic(sx, sy, gx, gy), g: 0 });

    while (!openSet.isEmpty()) {
      const curr = openSet.pop();
      if (curr.x === gx && curr.y === gy) return this._reconstructPath(cameFrom, curr.x, curr.y);

      for (const [nx, ny] of grid.getNeighbors(curr.x, curr.y)) {
        const tentativeG = curr.g + 1;
        const nKey = `${nx},${ny}`;
        if (tentativeG < (gScore.get(nKey) ?? Infinity)) {
          gScore.set(nKey, tentativeG);
          cameFrom.set(nKey, { x: curr.x, y: curr.y });
          const h = this.heuristic(nx, ny, gx, gy);
          // Tie-break: prefer nodes closer to goal (smaller h)
          openSet.push({ x: nx, y: ny, f: tentativeG + h + h * 0.001, g: tentativeG });
        }
      }
    }
    return null; // No path
  }

  /**
   * Space-Time A* — plans around a reservation table.
   * Nodes are (x, y, t). Agents can "wait" in place (stay = move to same cell).
   *
   * @param {number} sx - start x
   * @param {number} sy - start y
   * @param {number} gx - goal x
   * @param {number} gy - goal y
   * @param {ReservationTable} reservations
   * @param {number} startTime - timestep when agent departs
   * @param {number} maxTime - maximum timestep horizon
   * @returns {Array<{x,y}>|null} path (t=startTime...) or null
   */
  findPathSpaceTime(sx, sy, gx, gy, reservations, startTime = 0, maxTime = 300) {
    const grid = this.grid;
    if (!grid.isWalkable(gx, gy)) return null;

    // Key: "x,y,t"
    const openSet = new MinHeap();
    const cameFrom = new Map();  // key → {x, y, t}
    const gScore = new Map();

    const startKey = `${sx},${sy},${startTime}`;
    gScore.set(startKey, 0);
    const h0 = this.heuristic(sx, sy, gx, gy);
    openSet.push({ x: sx, y: sy, t: startTime, g: 0, f: h0 });

    while (!openSet.isEmpty()) {
      const curr = openSet.pop();
      const { x, y, t } = curr;

      // Reached goal: reconstruct
      if (x === gx && y === gy) {
        return this._reconstructPathST(cameFrom, x, y, t, startTime);
      }

      if (t >= maxTime) continue;

      // Expand: 4-directional moves + wait-in-place
      const moves = [...grid.getNeighbors(x, y), [x, y]]; // wait = [x,y]
      for (const [nx, ny] of moves) {
        const nt = t + 1;
        // Check vertex reservation at (nx, ny, nt)
        if (reservations.isVertexReserved(nx, ny, nt)) continue;
        // Check edge reservation: swapping positions
        if (reservations.isEdgeReserved(x, y, nx, ny, t)) continue;

        const tentativeG = curr.g + 1;
        const nKey = `${nx},${ny},${nt}`;
        if (tentativeG < (gScore.get(nKey) ?? Infinity)) {
          gScore.set(nKey, tentativeG);
          cameFrom.set(nKey, { x, y, t });
          const h = this.heuristic(nx, ny, gx, gy);
          openSet.push({ x: nx, y: ny, t: nt, g: tentativeG, f: tentativeG + h });
        }
      }
    }
    return null;
  }

  _reconstructPath(cameFrom, x, y) {
    const path = [];
    let key = `${x},${y}`;
    while (cameFrom.has(key)) {
      path.push({ x, y });
      const prev = cameFrom.get(key);
      x = prev.x; y = prev.y;
      key = `${x},${y}`;
    }
    return path.reverse();
  }

  _reconstructPathST(cameFrom, x, y, t, startTime) {
    const path = [];
    let key = `${x},${y},${t}`;
    while (cameFrom.has(key)) {
      path.unshift({ x, y, t });
      const prev = cameFrom.get(key);
      x = prev.x; y = prev.y; t = prev.t;
      key = `${x},${y},${t}`;
    }
    // Include start position at startTime
    path.unshift({ x, y, t: startTime });
    return path;
  }
}

// ---------------------------------------------------------------------------
// ReservationTable — tracks which (x,y,t) cells are claimed by agents
// ---------------------------------------------------------------------------
class ReservationTable {
  constructor() {
    this._vertices = new Map();  // "x,y,t" → agentId
    this._edges    = new Map();  // "x1,y1,x2,y2,t" → agentId
  }

  clear() { this._vertices.clear(); this._edges.clear(); }

  /**
   * Reserve all cells in a space-time path for an agent.
   * Also reserves the goal cell for all future timesteps up to maxTime
   * so agents don't path through it later.
   */
  reservePath(path, agentId, maxTime = 300) {
    if (!path || path.length === 0) return;
    for (let i = 0; i < path.length; i++) {
      const { x, y, t } = path[i];
      this._vertices.set(`${x},${y},${t}`, agentId);
      if (i < path.length - 1) {
        const { x: nx, y: ny, t: nt } = path[i + 1];
        // Forward edge: agent moves from (x,y) to (nx,ny) at time t
        this._edges.set(`${x},${y},${nx},${ny},${t}`, agentId);
        // Reverse edge: block the swap
        this._edges.set(`${nx},${ny},${x},${y},${t}`, agentId);
      }
    }
    // Reserve goal indefinitely to avoid other agents walking through
    const last = path[path.length - 1];
    for (let tt = last.t + 1; tt <= maxTime; tt++) {
      this._vertices.set(`${last.x},${last.y},${tt}`, agentId);
    }
  }

  isVertexReserved(x, y, t) {
    return this._vertices.has(`${x},${y},${t}`);
  }

  isEdgeReserved(fromX, fromY, toX, toY, t) {
    return this._edges.has(`${fromX},${fromY},${toX},${toY},${t}`);
  }

  whoOwns(x, y, t) {
    return this._vertices.get(`${x},${y},${t}`) ?? null;
  }

  /** Remove all reservations for a given agent (for replanning). */
  freeAgent(agentId) {
    for (const [k, v] of this._vertices) if (v === agentId) this._vertices.delete(k);
    for (const [k, v] of this._edges)    if (v === agentId) this._edges.delete(k);
  }
}
