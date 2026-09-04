/**
 * agent.js — Agent Class
 * Each agent has a unique ID, start/goal positions, a planned space-time path,
 * and a state machine driving its behaviour.
 *
 * State Machine:
 *   IDLE → PLANNING → MOVING ↔ WAITING → ARRIVED
 *                  ↘ STUCK (if no path found)
 */

'use strict';

const AgentState = Object.freeze({
  IDLE:     'IDLE',
  PLANNING: 'PLANNING',
  MOVING:   'MOVING',
  WAITING:  'WAITING',
  ARRIVED:  'ARRIVED',
  STUCK:    'STUCK',
});

class Agent {
  /**
   * @param {number} id      - unique integer id
   * @param {{x,y}} start    - starting grid cell
   * @param {{x,y}} goal     - target grid cell
   * @param {number} priority - lower number = higher priority during PBS
   */
  constructor(id, start, goal, priority) {
    this.id       = id;
    this.start    = { ...start };
    this.goal     = { ...goal  };
    this.priority = priority;

    // Current discrete grid position (updated each timestep)
    this.gridX = start.x;
    this.gridY = start.y;

    // Render interpolation (pixel-space lerp between grid steps)
    this.renderX = start.x;
    this.renderY = start.y;

    // Planned space-time path: [{x, y, t}, ...]
    // Index 0 is start at t=startTime, last is goal.
    this.path = [];

    // Index into path array for the current simulation timestep
    this.pathIndex = 0;

    // Current simulation timestep at which agent was placed
    this.startTime = 0;

    this.state = AgentState.IDLE;
    this.replanCount = 0;

    // Colour (HSL hue, 0–360) — assigned externally for visual differentiation
    this.hue = (id * 137.5) % 360; // golden-angle spacing

    // Statistics
    this.stepsTaken = 0;
    this.waitSteps  = 0;
    this.conflictsInvolved = 0;

    // Trail of recent grid positions for rendering
    this._trail = [];
    this._trailMaxLen = 12;
  }

  // -------------------------------------------------------------------------
  // Path management
  // -------------------------------------------------------------------------

  /** Assign a newly-planned space-time path. */
  assignPath(path) {
    this.path = path ?? [];
    this.pathIndex = 0;
    this.state = this.path.length > 0 ? AgentState.MOVING : AgentState.STUCK;
  }

  /** True if agent still has steps to take. */
  hasRemainingPath() {
    return this.pathIndex < this.path.length - 1;
  }

  /**
   * Advance agent by one simulation timestep.
   * @param {number} currentT - the current global timestep
   */
  step(currentT) {
    if (this.state === AgentState.ARRIVED || this.state === AgentState.STUCK) return;
    if (this.path.length === 0) { this.state = AgentState.STUCK; return; }

    // Find the path entry for this timestep
    const idx = this.pathIndex;
    if (idx >= this.path.length) {
      this.state = AgentState.ARRIVED;
      return;
    }

    const entry = this.path[idx];
    if (entry.t > currentT) {
      // Agent hasn't started yet (shouldn't happen in normal flow)
      this.state = AgentState.WAITING;
      return;
    }

    // Move to the cell in the path at or before currentT
    const newX = entry.x;
    const newY = entry.y;

    const moved = (newX !== this.gridX || newY !== this.gridY);
    if (moved) {
      this._trail.push({ x: this.gridX, y: this.gridY });
      if (this._trail.length > this._trailMaxLen) this._trail.shift();
      this.stepsTaken++;
    } else {
      this.waitSteps++;
    }

    this.gridX = newX;
    this.gridY = newY;

    // Advance path index for next timestep
    if (idx + 1 < this.path.length) {
      this.pathIndex++;
    }

    // Check arrival
    if (this.gridX === this.goal.x && this.gridY === this.goal.y) {
      this.state = AgentState.ARRIVED;
    } else {
      this.state = moved ? AgentState.MOVING : AgentState.WAITING;
    }
  }

  /**
   * Update render interpolation position.
   * @param {number} alpha - 0=prev position, 1=current grid position
   */
  updateRenderPosition(alpha) {
    this.renderX += (this.gridX - this.renderX) * alpha;
    this.renderY += (this.gridY - this.renderY) * alpha;
  }

  /** Snap render position immediately (used on reset). */
  snapRenderPosition() {
    this.renderX = this.gridX;
    this.renderY = this.gridY;
  }

  /** Reset agent to initial conditions (for scenario reset). */
  reset(newStart, newGoal, priority) {
    this.start    = { ...newStart };
    this.goal     = { ...newGoal  };
    this.priority = priority;
    this.gridX    = newStart.x;
    this.gridY    = newStart.y;
    this.renderX  = newStart.x;
    this.renderY  = newStart.y;
    this.path     = [];
    this.pathIndex = 0;
    this.startTime = 0;
    this.state    = AgentState.IDLE;
    this.replanCount = 0;
    this.stepsTaken  = 0;
    this.waitSteps   = 0;
    this.conflictsInvolved = 0;
    this._trail   = [];
  }

  get trail() { return this._trail; }

  /** Progress 0→1 along path. */
  get progress() {
    if (this.path.length <= 1) return this.state === AgentState.ARRIVED ? 1 : 0;
    return this.pathIndex / (this.path.length - 1);
  }
}
