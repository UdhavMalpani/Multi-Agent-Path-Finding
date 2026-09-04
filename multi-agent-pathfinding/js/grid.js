/**
 * grid.js — Grid Environment
 * Represents the 2D navigable world: cells, obstacles, connectivity.
 */

'use strict';

const CELL = Object.freeze({
  EMPTY:    0,
  OBSTACLE: 1,
  START:    2,
  GOAL:     3,
});

class Grid {
  /**
   * @param {number} cols  - number of columns (x-axis)
   * @param {number} rows  - number of rows (y-axis)
   */
  constructor(cols, rows) {
    this.cols = cols;
    this.rows = rows;
    // Flat array: cells[y * cols + x]
    this._cells = new Uint8Array(cols * rows); // all EMPTY
  }

  // -------------------------------------------------------------------------
  // Cell accessors
  // -------------------------------------------------------------------------

  getCell(x, y) {
    return this._cells[y * this.cols + x];
  }

  setCell(x, y, type) {
    this._cells[y * this.cols + x] = type;
  }

  inBounds(x, y) {
    return x >= 0 && y >= 0 && x < this.cols && y < this.rows;
  }

  isWalkable(x, y) {
    return this.inBounds(x, y) && this._cells[y * this.cols + x] !== CELL.OBSTACLE;
  }

  isObstacle(x, y) {
    return this.inBounds(x, y) && this._cells[y * this.cols + x] === CELL.OBSTACLE;
  }

  /**
   * 4-directional neighbors (no diagonals) that are walkable.
   * @returns {Array<[number, number]>}
   */
  getNeighbors(x, y) {
    const dirs = [[0,-1],[0,1],[-1,0],[1,0]];
    const result = [];
    for (const [dx, dy] of dirs) {
      const nx = x + dx, ny = y + dy;
      if (this.isWalkable(nx, ny)) result.push([nx, ny]);
    }
    return result;
  }

  // -------------------------------------------------------------------------
  // Obstacle generation
  // -------------------------------------------------------------------------

  /**
   * Clear the entire grid (set all to EMPTY).
   */
  clear() {
    this._cells.fill(CELL.EMPTY);
  }

  /**
   * Generate random obstacles.
   * Uses a density factor (0–1). Applies random-walk noise to avoid
   * fully disconnecting the grid. Protects a set of positions.
   *
   * @param {number} density     - fraction of cells to block (0–0.35 recommended)
   * @param {Set<string>} protect - set of "x,y" keys to never obstruct
   */
  generateObstacles(density, protect = new Set()) {
    this.clear();
    const total = this.cols * this.rows;
    const count = Math.floor(total * density);

    let placed = 0;
    let attempts = 0;
    const maxAttempts = count * 20;

    while (placed < count && attempts < maxAttempts) {
      attempts++;
      const x = Math.floor(Math.random() * this.cols);
      const y = Math.floor(Math.random() * this.rows);
      const key = `${x},${y}`;
      if (!protect.has(key) && this._cells[y * this.cols + x] === CELL.EMPTY) {
        this._cells[y * this.cols + x] = CELL.OBSTACLE;
        placed++;
      }
    }
  }

  /**
   * Add rectangular wall segments (for structured environments).
   */
  addWallRect(x, y, w, h) {
    for (let cy = y; cy < y + h; cy++) {
      for (let cx = x; cx < x + w; cx++) {
        if (this.inBounds(cx, cy)) this._cells[cy * this.cols + cx] = CELL.OBSTACLE;
      }
    }
  }

  // -------------------------------------------------------------------------
  // Connectivity check (BFS)
  // -------------------------------------------------------------------------

  /**
   * Check whether two cells are reachable from each other using BFS.
   */
  isConnected(x1, y1, x2, y2) {
    if (!this.isWalkable(x1, y1) || !this.isWalkable(x2, y2)) return false;
    const visited = new Set();
    const queue = [[x1, y1]];
    visited.add(`${x1},${y1}`);
    while (queue.length > 0) {
      const [cx, cy] = queue.shift();
      if (cx === x2 && cy === y2) return true;
      for (const [nx, ny] of this.getNeighbors(cx, cy)) {
        const key = `${nx},${ny}`;
        if (!visited.has(key)) {
          visited.add(key);
          queue.push([nx, ny]);
        }
      }
    }
    return false;
  }

  /**
   * Returns all walkable cells reachable from (sx, sy).
   * @returns {Array<{x, y}>}
   */
  reachableCells(sx, sy) {
    const visited = new Set();
    const queue = [[sx, sy]];
    const result = [];
    const startKey = `${sx},${sy}`;
    visited.add(startKey);
    while (queue.length > 0) {
      const [cx, cy] = queue.shift();
      result.push({ x: cx, y: cy });
      for (const [nx, ny] of this.getNeighbors(cx, cy)) {
        const key = `${nx},${ny}`;
        if (!visited.has(key)) {
          visited.add(key);
          queue.push([nx, ny]);
        }
      }
    }
    return result;
  }

  // -------------------------------------------------------------------------
  // Position sampling
  // -------------------------------------------------------------------------

  /**
   * Pick a random walkable cell not in the excluded set.
   * @param {Set<string>} exclude - "x,y" keys to skip
   * @returns {{x, y}|null}
   */
  randomWalkableCell(exclude = new Set()) {
    const walkable = [];
    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        if (this.isWalkable(x, y) && !exclude.has(`${x},${y}`)) {
          walkable.push({ x, y });
        }
      }
    }
    if (walkable.length === 0) return null;
    return walkable[Math.floor(Math.random() * walkable.length)];
  }

  /**
   * Pick N distinct random walkable cells, ensuring each pair is connected.
   * Tries up to maxRetries times before giving up on connectivity check.
   */
  randomWalkablePairs(n, obstacleDensity = 0) {
    const starts = [];
    const goals  = [];
    const usedKeys = new Set();

    for (let i = 0; i < n; i++) {
      let s = null, g = null;
      let tries = 0;
      do {
        s = this.randomWalkableCell(usedKeys);
        if (!s) break;
        const tempExclude = new Set([...usedKeys, `${s.x},${s.y}`]);
        g = this.randomWalkableCell(tempExclude);
        tries++;
      } while (g && !this.isConnected(s.x, s.y, g.x, g.y) && tries < 30);

      if (s && g) {
        usedKeys.add(`${s.x},${s.y}`);
        usedKeys.add(`${g.x},${g.y}`);
        starts.push(s);
        goals.push(g);
      }
    }
    return { starts, goals };
  }

  // -------------------------------------------------------------------------
  // Serialization (for debugging)
  // -------------------------------------------------------------------------

  toAscii() {
    let out = '';
    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const c = this._cells[y * this.cols + x];
        out += c === CELL.OBSTACLE ? '█' : '·';
      }
      out += '\n';
    }
    return out;
  }
}
