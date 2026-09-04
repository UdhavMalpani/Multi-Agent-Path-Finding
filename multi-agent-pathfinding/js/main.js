/**
 * main.js — Bootstrap & Game Loop
 *
 * Owns the simulation lifecycle:
 *   - Initialise grid, agents, MAPF coordinator, renderer, UI
 *   - Run requestAnimationFrame loop
 *   - Advance discrete timesteps at the configured speed
 *   - Expose public API consumed by ui.js
 */

'use strict';

class Simulation {
  constructor() {
    // ---- Config (can be changed via UI) ----
    this.agentCount      = 100;
    this.gridCols        = 60;
    this.gridRows        = 60;
    this.obstacleDensity = 0.12;  // 12% obstacles by default
    this.stepsPerSecond  = 3;     // discrete timesteps per second
    this.showTrails      = true;

    // ---- Runtime ----
    this.grid     = null;
    this.agents   = [];
    this.mapf     = null;
    this.renderer = null;
    this.ui       = null;

    this.isRunning        = false;
    this.currentTimestep  = 0;
    this._lastStepTime    = 0;   // ms timestamp of last discrete step
    this._rafId           = null;
    this._planningWorking = false;
  }

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------

  init() {
    const canvas = document.getElementById('sim-canvas');
    this.grid     = new Grid(this.gridCols, this.gridRows);
    this.renderer = new Renderer(canvas, this.grid);
    this.ui       = new UI(this);

    this._buildScenario();
    this._startRenderLoop();

    // Show "ready" state
    this.ui.updateStats();
  }

  /** Tear down and rebuild everything from scratch. */
  reset() {
    this.isRunning       = false;
    this.currentTimestep = 0;
    this._lastStepTime   = 0;

    this.grid = new Grid(this.gridCols, this.gridRows);
    this.renderer.grid = this.grid;
    this.renderer.clearHeatmap();
    this.renderer.resize();

    this._buildScenario();
    this.ui.updateStats();
  }

  resume() {
    this.isRunning     = true;
    this._lastStepTime = performance.now();
  }

  pause() {
    this.isRunning = false;
  }

  /** Advance exactly one discrete timestep (for manual stepping while paused). */
  stepOnce() {
    this._advanceTimestep();
    this.ui.updateStats();
  }

  // -------------------------------------------------------------------------
  // Scenario construction
  // -------------------------------------------------------------------------

  _buildScenario() {
    this._planningWorking = true;
    this._showPlanningOverlay(true);

    // Generate obstacles — protect a margin around the border
    const protect = new Set();
    // No specific cells to protect at start; obstacle generator avoids chosen
    // starts/goals automatically below.
    this.grid.generateObstacles(this.obstacleDensity, protect);

    // Pick starts & goals
    const { starts, goals } = this.grid.randomWalkablePairs(this.agentCount);
    const actualCount = starts.length;

    // Build protect set with starts and goals so obstacles aren't placed there
    // (obstacles were already generated — now mark them in grid for reference)
    for (let i = 0; i < actualCount; i++) {
      protect.add(`${starts[i].x},${starts[i].y}`);
      protect.add(`${goals[i].x},${goals[i].y}`);
    }

    // Create agents
    this.agents = [];
    for (let i = 0; i < actualCount; i++) {
      const priority = i;  // agent i has priority i (0 = highest)
      this.agents.push(new Agent(i, starts[i], goals[i], priority));
    }

    // Create MAPF coordinator and run PBS planning
    this.mapf = new MAPFCoordinator(this.grid, this.agents);

    // Planning can be slow for 100 agents; run in a setTimeout so UI updates first
    setTimeout(() => {
      try {
        this.mapf.planAll(0);
      } catch (e) {
        console.error('Planning error:', e);
      }
      this._planningWorking = false;
      this._showPlanningOverlay(false);
      this.ui.updateStats();
    }, 30);
  }

  _showPlanningOverlay(visible) {
    const el = document.getElementById('planning-overlay');
    if (el) el.style.display = visible ? 'flex' : 'none';
  }

  // -------------------------------------------------------------------------
  // Render loop
  // -------------------------------------------------------------------------

  _startRenderLoop() {
    const loop = (timestamp) => {
      this._rafId = requestAnimationFrame(loop);

      if (this.isRunning && !this._planningWorking) {
        const stepIntervalMs = 1000 / this.stepsPerSecond;
        const elapsed = timestamp - this._lastStepTime;

        if (elapsed >= stepIntervalMs) {
          const steps = Math.floor(elapsed / stepIntervalMs);
          for (let i = 0; i < Math.min(steps, 5); i++) {  // cap at 5 steps/frame
            this._advanceTimestep();
          }
          this._lastStepTime = timestamp;
        }
      }

      // Interpolation alpha: how far into the current step we are
      const stepIntervalMs = 1000 / this.stepsPerSecond;
      const alpha = this.isRunning
        ? Math.min((performance.now() - this._lastStepTime) / stepIntervalMs, 1)
        : 0;

      // Record heatmap
      if (this.renderer.showHeatmap) {
        this.renderer.recordAgentPositions(this.agents);
      }

      // Draw
      const effectiveAgents = this.showTrails ? this.agents : this.agents.map(a => ({
        ...a, trail: [], _trail: [], get trail() { return []; }
      }));

      this.renderer.draw(this.agents, alpha, timestamp);
    };

    this._rafId = requestAnimationFrame(loop);
  }

  // -------------------------------------------------------------------------
  // Discrete timestep advancement
  // -------------------------------------------------------------------------

  _advanceTimestep() {
    // Check if all arrived
    const allDone = this.agents.every(
      a => a.state === AgentState.ARRIVED || a.state === AgentState.STUCK
    );
    if (allDone) {
      this.isRunning = false;
      return;
    }

    this.mapf.step(this.currentTimestep);
    this.currentTimestep++;
  }

  // -------------------------------------------------------------------------
  // Public setters (called by UI)
  // -------------------------------------------------------------------------

  setSpeed(stepsPerSec) {
    this.stepsPerSecond = stepsPerSec;
  }

  setAgentCount(n) {
    this.agentCount = Math.max(1, Math.min(n, 150));
  }

  setObstacleDensity(d) {
    this.obstacleDensity = Math.max(0, Math.min(d, 0.35));
  }

  setGridSize(size) {
    this.gridCols = size;
    this.gridRows = size;
  }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
let sim;

document.addEventListener('DOMContentLoaded', () => {
  sim = new Simulation();
  sim.init();
  // Expose globally for debugging
  window.sim = sim;
});
