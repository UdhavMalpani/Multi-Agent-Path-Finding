/**
 * ui.js — UI Controls & Stats Panel
 * Wires up all DOM controls and updates the stats sidebar.
 */

'use strict';

class UI {
  constructor(simulation) {
    this.sim = simulation;
    this._bind();
    this._statsInterval = null;
  }

  _bind() {
    const sim = this.sim;

    // Play / Pause
    const playBtn = document.getElementById('btn-play');
    playBtn.addEventListener('click', () => {
      if (sim.isRunning) {
        sim.pause();
        playBtn.textContent = '▶  Play';
        playBtn.classList.remove('active');
      } else {
        sim.resume();
        playBtn.textContent = '⏸  Pause';
        playBtn.classList.add('active');
      }
    });

    // Step (advance one timestep while paused)
    document.getElementById('btn-step').addEventListener('click', () => {
      if (!sim.isRunning) sim.stepOnce();
    });

    // Reset
    document.getElementById('btn-reset').addEventListener('click', () => {
      sim.reset();
      playBtn.textContent = '▶  Play';
      playBtn.classList.remove('active');
    });

    // Speed slider
    const speedSlider = document.getElementById('slider-speed');
    const speedLabel  = document.getElementById('label-speed');
    speedSlider.addEventListener('input', () => {
      const v = parseFloat(speedSlider.value);
      speedLabel.textContent = `${v.toFixed(1)}×`;
      sim.setSpeed(v);
    });

    // Agent count slider
    const agentSlider = document.getElementById('slider-agents');
    const agentLabel  = document.getElementById('label-agents');
    agentSlider.addEventListener('input', () => {
      const v = parseInt(agentSlider.value);
      agentLabel.textContent = v;
    });
    agentSlider.addEventListener('change', () => {
      sim.setAgentCount(parseInt(agentSlider.value));
      sim.reset();
    });

    // Obstacle density
    const obstSlider = document.getElementById('slider-obstacles');
    const obstLabel  = document.getElementById('label-obstacles');
    obstSlider.addEventListener('input', () => {
      const v = parseInt(obstSlider.value);
      obstLabel.textContent = `${v}%`;
    });
    obstSlider.addEventListener('change', () => {
      sim.setObstacleDensity(parseInt(obstSlider.value) / 100);
      sim.reset();
    });

    // Grid size
    const gridSlider = document.getElementById('slider-grid');
    const gridLabel  = document.getElementById('label-grid');
    gridSlider.addEventListener('input', () => {
      const v = parseInt(gridSlider.value);
      gridLabel.textContent = `${v}×${v}`;
    });
    gridSlider.addEventListener('change', () => {
      sim.setGridSize(parseInt(gridSlider.value));
      sim.reset();
    });

    // Heatmap toggle
    document.getElementById('btn-heatmap').addEventListener('click', (e) => {
      sim.renderer.showHeatmap = !sim.renderer.showHeatmap;
      e.target.classList.toggle('active', sim.renderer.showHeatmap);
      if (!sim.renderer.showHeatmap) sim.renderer.clearHeatmap();
    });

    // Trails toggle
    document.getElementById('btn-trails').addEventListener('click', (e) => {
      sim.showTrails = !sim.showTrails;
      e.target.classList.toggle('active', sim.showTrails);
    });

    // Window resize
    window.addEventListener('resize', () => sim.renderer.resize());

    // Start stats refresh
    this._statsInterval = setInterval(() => this.updateStats(), 250);
  }

  updateStats() {
    const s = this.sim.mapf?.getStats() ?? null;
    if (!s) return;

    const set = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.textContent = val;
    };

    const pct = (n, d) => d > 0 ? ((n / d) * 100).toFixed(1) + '%' : '0%';

    set('stat-arrived',   s.arrived);
    set('stat-stuck',     s.stuck);
    set('stat-moving',    s.moving);
    set('stat-waiting',   s.waiting);
    set('stat-total',     s.totalAgents);
    set('stat-plans',     s.totalPlans);
    set('stat-replans',   s.totalReplans);
    set('stat-conflicts', s.conflictCount);
    set('stat-steps',     s.totalSteps);
    set('stat-timestep',  this.sim.currentTimestep);
    set('stat-arrived-pct', pct(s.arrived, s.totalAgents));

    // Progress bar
    const bar = document.getElementById('progress-bar');
    if (bar) {
      bar.style.width = pct(s.arrived, s.totalAgents);
    }

    // Status badge
    const badge = document.getElementById('status-badge');
    if (badge) {
      if (!this.sim.isRunning && this.sim.currentTimestep === 0) {
        badge.textContent = 'READY';
        badge.className = 'badge badge-idle';
      } else if (this.sim.isRunning) {
        badge.textContent = s.arrived === s.totalAgents ? 'COMPLETE' : 'RUNNING';
        badge.className   = s.arrived === s.totalAgents ? 'badge badge-done' : 'badge badge-running';
      } else {
        badge.textContent = 'PAUSED';
        badge.className   = 'badge badge-paused';
      }
    }
  }

  destroy() {
    clearInterval(this._statsInterval);
  }
}
