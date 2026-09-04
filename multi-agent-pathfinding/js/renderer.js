/**
 * renderer.js — Canvas Rendering Engine
 *
 * Draws the MAPF simulation at 60fps using:
 *   - Grid background with subtle lines
 *   - Obstacles as dark blocks with gradient
 *   - Goal markers as pulsing rings
 *   - Agent trails (fading polyline)
 *   - Agents as smooth circles (lerp interpolated)
 *   - Optional heatmap overlay
 *   - HUD layer (grid coordinates on hover)
 */

'use strict';

class Renderer {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {Grid} grid
   */
  constructor(canvas, grid) {
    this.canvas = canvas;
    this.ctx    = canvas.getContext('2d');
    this.grid   = grid;

    this.cellSize = 0; // computed in resize()
    this.offsetX  = 0;
    this.offsetY  = 0;

    // Heatmap: grid-flat array of visit counts
    this._heatmap = new Float32Array(grid.cols * grid.rows);
    this._heatmapMax = 1;
    this.showHeatmap = false;

    // Animation phase for pulsing goals
    this._phase = 0;

    // Hover cell
    this._hoverCell = null;
    this._setupMouseTrack();

    this.resize();
  }

  // -------------------------------------------------------------------------
  // Layout
  // -------------------------------------------------------------------------

  resize() {
    // Canvas fills its container (set by CSS)
    const w = this.canvas.clientWidth  || this.canvas.width;
    const h = this.canvas.clientHeight || this.canvas.height;
    this.canvas.width  = w;
    this.canvas.height = h;

    // Fit grid to canvas with some padding
    const pad = 10;
    const cellW = (w - pad * 2) / this.grid.cols;
    const cellH = (h - pad * 2) / this.grid.rows;
    this.cellSize = Math.min(cellW, cellH);

    // Center the grid
    this.offsetX = (w - this.cellSize * this.grid.cols) / 2;
    this.offsetY = (h - this.cellSize * this.grid.rows) / 2;
  }

  // -------------------------------------------------------------------------
  // Main draw
  // -------------------------------------------------------------------------

  /**
   * Draw one frame.
   * @param {Agent[]} agents
   * @param {number}  alpha     - sub-step interpolation [0,1] for smooth motion
   * @param {number}  timestamp - milliseconds (for animations)
   */
  draw(agents, alpha, timestamp) {
    this._phase = (timestamp / 1200) % (Math.PI * 2);

    const ctx = this.ctx;
    const { canvas, cellSize, offsetX, offsetY, grid } = this;

    // Clear
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Background
    ctx.fillStyle = '#0a0a14';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    this._drawGrid(ctx, offsetX, offsetY, cellSize, grid);
    this._drawObstacles(ctx, offsetX, offsetY, cellSize, grid);

    if (this.showHeatmap) {
      this._drawHeatmap(ctx, offsetX, offsetY, cellSize, grid);
    }

    this._drawGoals(ctx, offsetX, offsetY, cellSize, agents);
    this._drawStarts(ctx, offsetX, offsetY, cellSize, agents);
    this._drawTrails(ctx, offsetX, offsetY, cellSize, agents);
    this._drawAgents(ctx, offsetX, offsetY, cellSize, agents, alpha);

    if (this._hoverCell) {
      this._drawHoverHighlight(ctx, offsetX, offsetY, cellSize);
    }
  }

  // -------------------------------------------------------------------------
  // Sub-draw routines
  // -------------------------------------------------------------------------

  _drawGrid(ctx, ox, oy, cs, grid) {
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    ctx.lineWidth   = 0.5;

    for (let x = 0; x <= grid.cols; x++) {
      ctx.beginPath();
      ctx.moveTo(ox + x * cs, oy);
      ctx.lineTo(ox + x * cs, oy + grid.rows * cs);
      ctx.stroke();
    }
    for (let y = 0; y <= grid.rows; y++) {
      ctx.beginPath();
      ctx.moveTo(ox,                   oy + y * cs);
      ctx.lineTo(ox + grid.cols * cs,  oy + y * cs);
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawObstacles(ctx, ox, oy, cs, grid) {
    ctx.save();
    for (let y = 0; y < grid.rows; y++) {
      for (let x = 0; x < grid.cols; x++) {
        if (grid.isObstacle(x, y)) {
          const px = ox + x * cs;
          const py = oy + y * cs;
          // Fill with dark blue-grey
          const grad = ctx.createLinearGradient(px, py, px + cs, py + cs);
          grad.addColorStop(0, '#1a2340');
          grad.addColorStop(1, '#0e1520');
          ctx.fillStyle = grad;
          ctx.fillRect(px + 0.5, py + 0.5, cs - 1, cs - 1);
          // Subtle border
          ctx.strokeStyle = 'rgba(100,140,200,0.15)';
          ctx.lineWidth   = 0.5;
          ctx.strokeRect(px + 0.5, py + 0.5, cs - 1, cs - 1);
        }
      }
    }
    ctx.restore();
  }

  _drawHeatmap(ctx, ox, oy, cs, grid) {
    ctx.save();
    ctx.globalAlpha = 0.35;
    const max = this._heatmapMax || 1;
    for (let y = 0; y < grid.rows; y++) {
      for (let x = 0; x < grid.cols; x++) {
        if (grid.isObstacle(x, y)) continue;
        const v = this._heatmap[y * grid.cols + x] / max;
        if (v < 0.01) continue;
        // Interpolate: cold=blue → warm=red
        const hue = (1 - v) * 240; // 240=blue, 0=red
        ctx.fillStyle = `hsl(${hue},100%,50%)`;
        ctx.fillRect(ox + x * cs, oy + y * cs, cs, cs);
      }
    }
    ctx.restore();
  }

  _drawGoals(ctx, ox, oy, cs, agents) {
    ctx.save();
    const r   = cs * 0.35;
    const pulse = 0.12 * Math.sin(this._phase);

    for (const ag of agents) {
      if (ag.state === AgentState.ARRIVED) continue;
      const px = ox + (ag.goal.x + 0.5) * cs;
      const py = oy + (ag.goal.y + 0.5) * cs;

      ctx.beginPath();
      ctx.arc(px, py, r * (1 + pulse), 0, Math.PI * 2);
      ctx.strokeStyle = `hsla(${ag.hue},90%,65%,0.7)`;
      ctx.lineWidth   = cs * 0.06;
      ctx.stroke();

      // Inner dot
      ctx.beginPath();
      ctx.arc(px, py, r * 0.22, 0, Math.PI * 2);
      ctx.fillStyle = `hsla(${ag.hue},90%,70%,0.5)`;
      ctx.fill();
    }
    ctx.restore();
  }

  _drawStarts(ctx, ox, oy, cs, agents) {
    // Small diamond at start positions for IDLE/PLANNING agents
    ctx.save();
    for (const ag of agents) {
      if (ag.state !== AgentState.IDLE && ag.state !== AgentState.PLANNING) continue;
      const px = ox + (ag.start.x + 0.5) * cs;
      const py = oy + (ag.start.y + 0.5) * cs;
      const s  = cs * 0.22;
      ctx.beginPath();
      ctx.moveTo(px,     py - s);
      ctx.lineTo(px + s, py);
      ctx.lineTo(px,     py + s);
      ctx.lineTo(px - s, py);
      ctx.closePath();
      ctx.strokeStyle = `hsla(${ag.hue},80%,60%,0.5)`;
      ctx.lineWidth   = 1;
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawTrails(ctx, ox, oy, cs, agents) {
    ctx.save();
    ctx.lineCap  = 'round';
    ctx.lineJoin = 'round';

    for (const ag of agents) {
      const trail = ag.trail;
      if (trail.length < 2) continue;

      for (let i = 1; i < trail.length; i++) {
        const alpha = (i / trail.length) * 0.35;
        ctx.beginPath();
        ctx.strokeStyle = `hsla(${ag.hue},85%,60%,${alpha})`;
        ctx.lineWidth   = cs * 0.12;
        const ax = ox + (trail[i-1].x + 0.5) * cs;
        const ay = oy + (trail[i-1].y + 0.5) * cs;
        const bx = ox + (trail[i].x   + 0.5) * cs;
        const by = oy + (trail[i].y   + 0.5) * cs;
        ctx.moveTo(ax, ay);
        ctx.lineTo(bx, by);
        ctx.stroke();
      }

      // Extend trail to current render position
      if (trail.length > 0) {
        const last = trail[trail.length - 1];
        ctx.beginPath();
        ctx.strokeStyle = `hsla(${ag.hue},85%,60%,0.35)`;
        ctx.lineWidth   = cs * 0.12;
        ctx.moveTo(ox + (last.x + 0.5) * cs, oy + (last.y + 0.5) * cs);
        ctx.lineTo(ox + (ag.renderX + 0.5) * cs, oy + (ag.renderY + 0.5) * cs);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  _drawAgents(ctx, ox, oy, cs, agents, alpha) {
    ctx.save();
    // Lerp render positions toward grid positions
    const lerpSpeed = 0.22 + alpha * 0.3;
    for (const ag of agents) {
      ag.updateRenderPosition(lerpSpeed);
    }

    // Draw in priority order (higher priority on top)
    const sorted = [...agents].sort((a, b) => b.priority - a.priority);

    for (const ag of sorted) {
      const px = ox + (ag.renderX + 0.5) * cs;
      const py = oy + (ag.renderY + 0.5) * cs;
      const r  = cs * 0.38;

      // Glow effect
      if (ag.state === AgentState.MOVING) {
        ctx.save();
        ctx.shadowColor = `hsla(${ag.hue},100%,60%,0.6)`;
        ctx.shadowBlur  = cs * 0.5;
        ctx.beginPath();
        ctx.arc(px, py, r, 0, Math.PI * 2);
        ctx.fillStyle = `hsla(${ag.hue},90%,55%,0.2)`;
        ctx.fill();
        ctx.restore();
      }

      // Main circle
      const grad = ctx.createRadialGradient(px - r*0.2, py - r*0.2, 0, px, py, r);
      const lightness = ag.state === AgentState.ARRIVED  ? 80 :
                        ag.state === AgentState.STUCK    ? 30 :
                        ag.state === AgentState.WAITING  ? 50 : 65;
      const sat       = ag.state === AgentState.STUCK ? 20 : 90;

      grad.addColorStop(0, `hsla(${ag.hue},${sat}%,${lightness + 15}%,1)`);
      grad.addColorStop(1, `hsla(${ag.hue},${sat}%,${lightness - 10}%,1)`);

      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fillStyle = grad;
      ctx.fill();

      // Border ring
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = `hsla(${ag.hue},100%,85%,0.6)`;
      ctx.stroke();

      // State indicator: small dot bottom-right
      if (ag.state === AgentState.WAITING) {
        ctx.beginPath();
        ctx.arc(px + r*0.55, py + r*0.55, r*0.25, 0, Math.PI*2);
        ctx.fillStyle = '#f59e0b';
        ctx.fill();
      } else if (ag.state === AgentState.STUCK) {
        ctx.beginPath();
        ctx.arc(px + r*0.55, py + r*0.55, r*0.25, 0, Math.PI*2);
        ctx.fillStyle = '#ef4444';
        ctx.fill();
      } else if (ag.state === AgentState.ARRIVED) {
        ctx.beginPath();
        ctx.arc(px + r*0.55, py + r*0.55, r*0.25, 0, Math.PI*2);
        ctx.fillStyle = '#22c55e';
        ctx.fill();
      }

      // Agent ID text (if cells are large enough)
      if (cs >= 18) {
        ctx.fillStyle = 'rgba(255,255,255,0.9)';
        ctx.font      = `bold ${Math.floor(cs * 0.28)}px Inter, sans-serif`;
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(ag.id, px, py);
      }
    }
    ctx.restore();
  }

  _drawHoverHighlight(ctx, ox, oy, cs) {
    const { x, y } = this._hoverCell;
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.lineWidth   = 1;
    ctx.strokeRect(ox + x * cs + 0.5, oy + y * cs + 0.5, cs - 1, cs - 1);
    ctx.restore();
  }

  // -------------------------------------------------------------------------
  // Heatmap management
  // -------------------------------------------------------------------------

  recordAgentPositions(agents) {
    for (const ag of agents) {
      if (ag.state === AgentState.IDLE || ag.state === AgentState.PLANNING) continue;
      const idx = ag.gridY * this.grid.cols + ag.gridX;
      this._heatmap[idx]++;
      if (this._heatmap[idx] > this._heatmapMax) this._heatmapMax = this._heatmap[idx];
    }
  }

  clearHeatmap() {
    this._heatmap.fill(0);
    this._heatmapMax = 1;
  }

  // -------------------------------------------------------------------------
  // Mouse tracking
  // -------------------------------------------------------------------------

  _setupMouseTrack() {
    this.canvas.addEventListener('mousemove', (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const cx = Math.floor((mx - this.offsetX) / this.cellSize);
      const cy = Math.floor((my - this.offsetY) / this.cellSize);
      if (this.grid.inBounds(cx, cy)) {
        this._hoverCell = { x: cx, y: cy };
      } else {
        this._hoverCell = null;
      }
    });
    this.canvas.addEventListener('mouseleave', () => {
      this._hoverCell = null;
    });
  }

  /**
   * Convert canvas pixel to grid cell.
   * @returns {{x,y}|null}
   */
  pixelToCell(px, py) {
    const cx = Math.floor((px - this.offsetX) / this.cellSize);
    const cy = Math.floor((py - this.offsetY) / this.cellSize);
    return this.grid.inBounds(cx, cy) ? { x: cx, y: cy } : null;
  }
}
