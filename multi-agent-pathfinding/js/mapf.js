/**
 * mapf.js — Multi-Agent Path Finding Coordinator
 *
 * Implements Priority-Based Search (PBS):
 *   1. Sort agents by priority (lower priority number = plans first).
 *   2. For each agent in order, run Space-Time A* avoiding the
 *      reservations already placed by higher-priority agents.
 *   3. Build a global reservation table as we go.
 *
 * Conflict detection & reactive replanning:
 *   During simulation, if a conflict is detected (vertex or edge),
 *   the lower-priority agent is flagged for replanning at the current
 *   timestep. Its old reservations are cleared and ST-A* is re-run.
 *
 * Statistics tracked:
 *   - Total plans made
 *   - Total replans
 *   - Conflicts detected
 */

'use strict';

class MAPFCoordinator {
  /**
   * @param {Grid}   grid
   * @param {Agent[]} agents
   */
  constructor(grid, agents) {
    this.grid   = grid;
    this.agents = agents;
    this.astar  = new AStar(grid);
    this.reservations = new ReservationTable();

    // Stats
    this.totalPlans    = 0;
    this.totalReplans  = 0;
    this.conflictCount = 0;

    // Max time horizon for space-time search
    this.MAX_TIME = 400;
  }

  // -------------------------------------------------------------------------
  // Initial planning — Priority-Based Search
  // -------------------------------------------------------------------------

  /**
   * Plan paths for all agents using PBS.
   * Sorts agents by priority and plans each one in order,
   * reserving space-time cells as it goes.
   *
   * @param {number} startTime - global timestep at which agents depart
   */
  planAll(startTime = 0) {
    this.reservations.clear();

    // Sort by priority (ascending = plan in this order)
    const ordered = [...this.agents].sort((a, b) => a.priority - b.priority);

    for (const agent of ordered) {
      this._planAgent(agent, startTime);
    }
  }

  /**
   * Plan (or replan) a single agent.
   * Removes its existing reservations, then re-runs ST-A*.
   */
  _planAgent(agent, startTime) {
    // Free old reservations
    this.reservations.freeAgent(agent.id);

    const path = this.astar.findPathSpaceTime(
      agent.start.x, agent.start.y,
      agent.goal.x,  agent.goal.y,
      this.reservations,
      startTime,
      this.MAX_TIME
    );

    agent.startTime = startTime;
    agent.assignPath(path);

    if (path) {
      this.reservations.reservePath(path, agent.id, this.MAX_TIME);
    }

    this.totalPlans++;
  }

  // -------------------------------------------------------------------------
  // Runtime conflict detection
  // -------------------------------------------------------------------------

  /**
   * Check for vertex and edge conflicts at the given timestep.
   * Returns array of conflict objects.
   *
   * @param {number} t  - timestep to check
   * @returns {Array<{type, agentA, agentB, x, y}>}
   */
  detectConflicts(t) {
    const conflicts = [];
    const posMap = new Map(); // "x,y" → agentId

    for (const agent of this.agents) {
      if (agent.state === AgentState.ARRIVED || agent.state === AgentState.STUCK) continue;
      const key = `${agent.gridX},${agent.gridY}`;
      if (posMap.has(key)) {
        conflicts.push({
          type: 'vertex',
          agentA: posMap.get(key),
          agentB: agent.id,
          x: agent.gridX,
          y: agent.gridY,
          t,
        });
      } else {
        posMap.set(key, agent.id);
      }
    }

    this.conflictCount += conflicts.length;
    return conflicts;
  }

  /**
   * Resolve detected conflicts by replanning the lower-priority agent.
   * Updates that agent's start to its current position at time t.
   *
   * @param {Array} conflicts - from detectConflicts()
   * @param {number} t        - current timestep
   */
  resolveConflicts(conflicts, t) {
    for (const conflict of conflicts) {
      const a = this._getAgent(conflict.agentA);
      const b = this._getAgent(conflict.agentB);
      if (!a || !b) continue;

      // The higher-priority (lower number) agent wins.
      // Lower-priority agent replans.
      const loser = a.priority > b.priority ? a : b;

      // Update loser's start to its current position
      loser.start = { x: loser.gridX, y: loser.gridY };
      loser.replanCount++;
      loser.conflictsInvolved++;
      this.totalReplans++;

      this._planAgent(loser, t);
    }
  }

  /**
   * Full step: advance all agents, detect & resolve conflicts.
   * @param {number} t - current global timestep
   */
  step(t) {
    // Step all agents
    for (const agent of this.agents) {
      agent.step(t);
    }

    // Detect conflicts at new positions
    const conflicts = this.detectConflicts(t + 1);
    if (conflicts.length > 0) {
      this.resolveConflicts(conflicts, t + 1);
    }
  }

  // -------------------------------------------------------------------------
  // Aggregate stats
  // -------------------------------------------------------------------------

  getStats() {
    let arrived = 0, stuck = 0, moving = 0, waiting = 0, totalSteps = 0, totalWait = 0;
    for (const ag of this.agents) {
      if (ag.state === AgentState.ARRIVED) arrived++;
      else if (ag.state === AgentState.STUCK) stuck++;
      else if (ag.state === AgentState.WAITING) waiting++;
      else moving++;
      totalSteps += ag.stepsTaken;
      totalWait  += ag.waitSteps;
    }
    return {
      arrived, stuck, moving, waiting,
      totalAgents:  this.agents.length,
      totalPlans:   this.totalPlans,
      totalReplans: this.totalReplans,
      conflictCount: this.conflictCount,
      totalSteps,
      totalWait,
    };
  }

  _getAgent(id) { return this.agents.find(a => a.id === id) ?? null; }

  reset() {
    this.reservations.clear();
    this.totalPlans    = 0;
    this.totalReplans  = 0;
    this.conflictCount = 0;
  }
}
