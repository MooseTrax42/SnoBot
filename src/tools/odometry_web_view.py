"""
Live odometry web viewer.

Starts:
1) SBCP control loop to read encoders/IMU and update odometry.
2) A tiny local web server with a live map page at http://localhost:3000.

Usage:
    python src/tools/odometry_web_view.py --port COM12
"""

from __future__ import annotations

import argparse
import asyncio
import heapq
import json
import math
import platform
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Add src/ to path so local modules can be imported when running from repo root.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sbcp.control_loop import ControlLoop
from sbcp.transport import AsyncSerialTransport
from sban.perimeter import PerimeterRecorder, RecorderState
from sban.planning import CoveragePlanner, PlannerConfig
from sban.localization.odometry import Pose

Point = Tuple[float, float]
ENABLE_NAV = False
NAV_MAX_CELLS = 80_000
NAV_MAX_EXPANSIONS = 80_000
NAV_MIN_RESOLUTION = 0.6


DEFAULT_SERIAL_PORT = "COM12" if platform.system() == "Windows" else "/dev/ttyACM0"
DEFAULT_BAUD = 115200


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>SnoBot Perimeter Mapper</title>
  <style>
    @import url("https://fonts.googleapis.com/css2?family=Newsreader:wght@400;600&family=Space+Grotesk:wght@400;600;700&display=swap");
    :root {
      --bg: #f2efe6;
      --panel: #fffdf6;
      --ink: #111827;
      --muted: #4b5563;
      --accent: #0f766e;
      --accent-2: #2f6fed;
      --robot: #ef4444;
      --grid: #d8d2c4;
      --outer: #0f766e;
      --hole: #f97316;
      --pass: #94a3b8;
      --path: #0ea5a4;
      --ok: #16a34a;
      --bad: #b91c1c;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Space Grotesk", "Trebuchet MS", sans-serif;
      background:
        radial-gradient(1200px 600px at 15% -10%, #fff3c7 0%, transparent 55%),
        radial-gradient(900px 500px at 100% 10%, #e4f2ff 0%, transparent 55%),
        var(--bg);
      color: var(--ink);
    }
    .wrap {
      max-width: 1200px;
      margin: 0 auto;
      padding: 18px;
      display: grid;
      gap: 14px;
    }
    .panel {
      background: var(--panel);
      border: 2px solid #d5ccb9;
      border-radius: 14px;
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.08);
      padding: 14px 16px;
    }
    .top {
      display: grid;
      gap: 12px;
      grid-template-columns: 1.5fr 1fr 1fr;
      align-items: center;
    }
    h1 {
      margin: 0;
      font-family: "Newsreader", serif;
      font-size: 1.5rem;
      letter-spacing: 0.01em;
    }
    .sub {
      margin: 4px 0 0;
      color: var(--muted);
      font-size: 0.95rem;
    }
    .status {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      font-size: 0.95rem;
      color: var(--muted);
      justify-content: flex-start;
    }
    .dot {
      width: 11px;
      height: 11px;
      border-radius: 999px;
      background: var(--bad);
      box-shadow: 0 0 0 3px rgba(185, 28, 28, 0.12);
    }
    .dot.ok {
      background: var(--ok);
      box-shadow: 0 0 0 3px rgba(22, 163, 74, 0.14);
    }
    .controls {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    .grid {
      display: grid;
      grid-template-columns: minmax(260px, 320px) 1fr;
      gap: 14px;
    }
    .card {
      border: 1px solid #e2ddd1;
      border-radius: 12px;
      padding: 12px;
      background: #fffaf0;
      display: grid;
      gap: 10px;
    }
    .card h2 {
      margin: 0;
      font-size: 1rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    canvas {
      width: 100%;
      height: min(70vh, 720px);
      border-radius: 12px;
      border: 2px solid #ddd3be;
      background: #fffef9;
      display: block;
    }
    .stats {
      display: grid;
      grid-template-columns: repeat(3, minmax(120px, 1fr));
      gap: 8px 12px;
      font-size: 0.92rem;
      margin-top: 10px;
    }
    .kv { color: var(--muted); }
    .kv b { color: var(--ink); }
    .row {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      align-items: center;
    }
    button {
      border: 0;
      border-radius: 9px;
      padding: 9px 12px;
      font-weight: 700;
      cursor: pointer;
      color: #fff;
      background: #1f2937;
    }
    button.alt { background: var(--accent); }
    button.ghost {
      background: transparent;
      color: var(--ink);
      border: 1px solid #d1c8b5;
    }
    button.primary { background: var(--accent-2); }
    input[type="range"] {
      width: 100%;
    }
    input[type="number"] {
      width: 90px;
      padding: 6px 8px;
      border-radius: 8px;
      border: 1px solid #d3c9b6;
      font-weight: 600;
      background: #fff;
    }
    select {
      padding: 6px 8px;
      border-radius: 8px;
      border: 1px solid #d3c9b6;
      font-weight: 600;
      background: #fff;
    }
    label.toggle {
      display: inline-flex;
      gap: 6px;
      align-items: center;
      font-size: 0.9rem;
      color: var(--muted);
    }
    kbd {
      background: #111827;
      color: #fff;
      padding: 2px 6px;
      border-radius: 6px;
      font-size: 0.8rem;
      font-weight: 700;
      letter-spacing: 0.05em;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 999px;
      background: #e7e2d3;
      color: #2f2a20;
      font-size: 0.8rem;
      font-weight: 700;
    }
    .muted { color: var(--muted); font-size: 0.88rem; }
    .log {
      background: #f7f2e7;
      border: 1px solid #e2ddd1;
      border-radius: 10px;
      padding: 10px;
      max-height: 160px;
      overflow-y: auto;
      white-space: pre-wrap;
      font-size: 0.85rem;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="panel top">
      <div>
        <h1>SnoBot Perimeter Mapper</h1>
        <div class="sub">Keyboard drive, perimeter + holes, and coverage preview.</div>
      </div>
      <div class="status">
        <span id="statusDot" class="dot"></span>
        <div>
          <div id="statusText">Connecting...</div>
          <div class="sub" id="statusSub">Waiting for telemetry</div>
        </div>
      </div>
      <div class="controls">
        <button id="manualBtn" class="ghost" type="button">Manual Mode</button>
        <button id="resetFaultsBtn" class="ghost" type="button">Reset Faults</button>
        <button id="resetBtn" class="alt" type="button">Reset Odometry</button>
        <button id="fitBtn" type="button">Fit View</button>
      </div>
    </div>

    <div class="panel grid">
      <div class="left">
        <section class="card">
          <h2>Teleop</h2>
          <div class="muted">Drive with <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or arrows.</div>
          <label class="toggle">
            <input id="teleopToggle" type="checkbox" />
            Enable keyboard control
          </label>
          <label class="toggle">
            <input id="sandboxToggle" type="checkbox" />
            Sandbox mode (no hardware)
          </label>
          <div class="row">
            <span class="muted">Speed</span>
            <input id="speedRange" type="range" min="0.1" max="1.35" step="0.05" value="0.4" />
            <span class="pill" id="speedVal">0.40 m/s</span>
          </div>
          <div class="row">
            <span class="muted">Turn</span>
            <input id="turnRange" type="range" min="0.4" max="3.14" step="0.1" value="1.2" />
            <span class="pill" id="turnVal">1.20 rad/s</span>
          </div>
          <div class="row">
            <button id="stopBtn" class="ghost" type="button">Stop</button>
          </div>
        </section>

        <section class="card">
          <h2>Perimeter + Holes</h2>
          <div class="row">
            <button id="outerBtn" class="alt" type="button">Record Outer</button>
            <button id="holeBtn" type="button">Record Hole</button>
          </div>
          <div class="row">
            <button id="recStopBtn" class="ghost" type="button">Stop Recording</button>
            <button id="clearBtn" class="ghost" type="button">Clear</button>
          </div>
          <div class="muted" id="recStatus">Recorder: idle</div>
          <div class="muted">Outer: <b id="outerCount">0</b> pts</div>
          <div class="muted">Holes: <b id="holeCount">0</b></div>
        </section>

        <section class="card">
          <h2>Coverage Preview</h2>
          <div class="row">
            <span class="muted">Line spacing</span>
            <input id="spacingInput" type="number" min="0.2" step="0.05" value="0.45" />
            <span class="muted">m</span>
          </div>
          <div class="row">
            <span class="muted">Sweep angle</span>
            <input id="angleInput" type="number" min="0" max="180" step="5" value="0" />
            <span class="muted">deg</span>
          </div>
          <div class="row">
            <span class="muted">Pattern</span>
            <select id="patternSelect">
              <option value="hybrid" selected>Hybrid (edge + fill)</option>
              <option value="scanline">Scanline only</option>
              <option value="contour">Contour only</option>
            </select>
          </div>
          <div class="row">
            <span class="muted">Edge loops</span>
            <input id="edgeInput" type="number" min="0" max="8" step="1" value="2" />
          </div>
          <div class="row">
            <button id="planBtn" class="primary" type="button">Generate Passes</button>
            <button id="simBtn" class="ghost" type="button">Play Preview</button>
            <button id="exportBtn" class="ghost" type="button">Download Route</button>
            <button id="execBtn" class="alt" type="button">Execute Route</button>
          </div>
          <div class="row">
            <label class="toggle"><input id="showPerimeter" type="checkbox" checked />Show perimeter</label>
            <label class="toggle"><input id="showCoverage" type="checkbox" checked />Show passes</label>
            <label class="toggle"><input id="showRoute" type="checkbox" checked />Show route</label>
            <label class="toggle"><input id="showOdom" type="checkbox" checked />Show odometry</label>
          </div>
          <div class="row">
            <label class="toggle"><input id="allowHwExec" type="checkbox" />Allow hardware execute</label>
          </div>
        </section>

        <section class="card">
          <h2>Console</h2>
          <div id="log" class="log">UI loaded.</div>
        </section>
      </div>

      <div class="right">
        <canvas id="map"></canvas>
        <div class="stats">
          <div class="kv">X: <b id="xv">0.000</b> m</div>
          <div class="kv">Y: <b id="yv">0.000</b> m</div>
          <div class="kv">Heading: <b id="hv">0.0</b> deg</div>
          <div class="kv">Distance: <b id="dv">0.000</b> m</div>
          <div class="kv">Updates: <b id="uv">0</b></div>
          <div class="kv">Meters/tick: <b id="mtv">0.000000</b></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const state = {
      path: [],
      pose: null,
      perimeter: {
        outer: [],
        holes: [],
        recording_path: [],
        rec_state: "idle",
        rec_stage: "outer"
      },
      coverage: {
        passes: [],
        line_spacing_m: 0.45,
        sweep_angle_rad: 0.0,
        route: []
      },
      plan: null,
      connected: false,
      lastError: null
    };

    const ui = {
      showPerimeter: true,
      showCoverage: true,
      showRoute: true,
      showOdom: true
    };

    const sim = {
      active: false,
      speed: 0.35,
      dist: 0,
      segments: [],
      total: 0,
      point: null
    };

    const MAX_LINEAR_VEL = 1.35;
    const MAX_ANGULAR_VEL = 3.14;

    const teleop = {
      enabled: false,
      speed: 0.4,
      turn: 1.2,
      moving: false
    };

    const pressed = new Set();

    const c = document.getElementById("map");
    const ctx = c.getContext("2d");
    const statusDot = document.getElementById("statusDot");
    const statusText = document.getElementById("statusText");
    const statusSub = document.getElementById("statusSub");
    const sandboxToggle = document.getElementById("sandboxToggle");
    const allowHwExec = document.getElementById("allowHwExec");
    const execBtn = document.getElementById("execBtn");
    let execActive = false;
    const logEl = document.getElementById("log");
    const logBuffer = [];
    const LOG_LIMIT = 140;
    const boundsBuffer = [];
    const ORIGIN = { x: 0, y: 0 };
    let lastPlanTs = 0;
    let lastPlanError = null;
    let lastPlanSummaryTs = 0;

    function on(id, ev, fn) {
      const el = document.getElementById(id);
      if (el) {
        el.addEventListener(ev, fn);
      }
      return el;
    }

      function log(msg) {
        if (!logEl) return;
        const t = new Date().toLocaleTimeString();
        logBuffer.unshift(`[${t}] ${msg}`);
        if (logBuffer.length > LOG_LIMIT) {
          logBuffer.length = LOG_LIMIT;
        }
        logEl.textContent = logBuffer.join("\\n");
      }

      function logPlanSummary(plan, spacingOverride, angleOverride) {
        const planPattern = plan.pattern || "unknown";
        const planEdge = (typeof plan.edge_passes === "number") ? plan.edge_passes : "?";
        const spacing = (typeof spacingOverride === "number")
          ? spacingOverride
          : (state.coverage && typeof state.coverage.line_spacing_m === "number")
            ? state.coverage.line_spacing_m
            : null;
        const angleDeg = (typeof angleOverride === "number")
          ? angleOverride
          : (state.coverage && typeof state.coverage.sweep_angle_rad === "number")
            ? (state.coverage.sweep_angle_rad * 180 / Math.PI)
            : null;
        const spacingTxt = (spacing !== null) ? `, spacing ${spacing.toFixed(2)}m` : "";
        const angleTxt = (angleDeg !== null) ? `, angle ${angleDeg.toFixed(1)}deg` : "";
        log(`Passes generated (${planPattern}, edge ${planEdge}${spacingTxt}${angleTxt}) -> passes=${plan.passes || 0}, route_pts=${plan.route_pts || 0}`);
        if (plan.audit) {
          const a = plan.audit;
          log(`Audit: coverage ${a.coverage_pct}% (max_gap ${a.max_gap_m}m, samples ${a.samples}, step ${a.step_m}m)`);
        }
      }

    window.addEventListener("error", (e) => {
      setStatus(false, "UI error", e.message || "script error");
      log(`UI error: ${e.message || "script error"}`);
    });
    window.addEventListener("unhandledrejection", (e) => {
      setStatus(false, "UI error", String(e.reason || "unhandled rejection"));
      log(`UI error: ${String(e.reason || "unhandled rejection")}`);
    });

    function resizeCanvas() {
      const dpr = window.devicePixelRatio || 1;
      const w = c.clientWidth;
      const h = c.clientHeight;
      c.width = Math.floor(w * dpr);
      c.height = Math.floor(h * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }
    window.addEventListener("resize", resizeCanvas);

    function setStatus(connected, text, sub) {
      statusDot.classList.toggle("ok", connected);
      statusText.textContent = text;
      statusSub.textContent = sub || "";
    }

    function worldToCanvas(x, y, bounds, w, h) {
      const margin = 35;
      const spanX = Math.max(bounds.maxX - bounds.minX, 0.5);
      const spanY = Math.max(bounds.maxY - bounds.minY, 0.5);
      const sx = (w - margin * 2) / spanX;
      const sy = (h - margin * 2) / spanY;
      const s = Math.max(Math.min(sx, sy), 1e-6);
      const ox = (w - spanX * s) / 2 - bounds.minX * s;
      const oy = (h - spanY * s) / 2 + bounds.maxY * s;
      return [x * s + ox, -y * s + oy];
    }

    function gatherPoints() {
      boundsBuffer.length = 0;
      const pts = boundsBuffer;
      if (ui.showOdom) {
        pts.push(...state.path);
      }
      if (ui.showPerimeter) {
        if (state.perimeter.outer) pts.push(...state.perimeter.outer);
        for (const h of state.perimeter.holes || []) {
          pts.push(...h);
        }
        pts.push(...(state.perimeter.recording_path || []));
      }
      if (ui.showCoverage) {
        for (const p of state.coverage.passes || []) {
          pts.push(...p);
        }
      }
      if (ui.showRoute && state.coverage.route) {
        pts.push(...state.coverage.route);
      }
      if (sim.point) {
        pts.push(sim.point);
      }
      if (state.pose) {
        pts.push(state.pose);
      }
      if (!pts.length) {
        pts.push(ORIGIN);
      }
      return pts;
    }

    function computeBounds() {
      const pts = gatherPoints();
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      for (const p of pts) {
        minX = Math.min(minX, p.x);
        minY = Math.min(minY, p.y);
        maxX = Math.max(maxX, p.x);
        maxY = Math.max(maxY, p.y);
      }
      return { minX, minY, maxX, maxY };
    }

    function drawGrid(w, h) {
      ctx.strokeStyle = "rgba(216, 210, 196, 0.9)";
      ctx.lineWidth = 1;
      for (let x = 0; x <= w; x += 40) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
      }
      for (let y = 0; y <= h; y += 40) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }
    }

    function drawPolyline(points, bounds, w, h, style) {
      if (!points || points.length < 2) return;
      ctx.save();
      ctx.strokeStyle = style.color;
      ctx.lineWidth = style.width || 2;
      if (style.dash) ctx.setLineDash(style.dash);
      ctx.beginPath();
      const [x0, y0] = worldToCanvas(points[0].x, points[0].y, bounds, w, h);
      ctx.moveTo(x0, y0);
      for (let i = 1; i < points.length; i++) {
        const [x, y] = worldToCanvas(points[i].x, points[i].y, bounds, w, h);
        ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();
    }

    function drawRobot(pose, bounds, w, h, color) {
      const [cx, cy] = worldToCanvas(pose.x, pose.y, bounds, w, h);
      const size = 10;
      const th = pose.theta || 0;
      const p1 = [cx + Math.cos(th) * size * 1.6, cy - Math.sin(th) * size * 1.6];
      const p2 = [cx + Math.cos(th + 2.5) * size, cy - Math.sin(th + 2.5) * size];
      const p3 = [cx + Math.cos(th - 2.5) * size, cy - Math.sin(th - 2.5) * size];

      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(p1[0], p1[1]);
      ctx.lineTo(p2[0], p2[1]);
      ctx.lineTo(p3[0], p3[1]);
      ctx.closePath();
      ctx.fill();
    }

    function drawCoverage(bounds, w, h) {
      if (!ui.showCoverage) return;
      for (const pass of state.coverage.passes || []) {
        drawPolyline(pass, bounds, w, h, { color: "rgba(148, 163, 184, 0.9)", width: 2 });
      }
    }

    function drawRoute(bounds, w, h) {
      if (!ui.showRoute) return;
      if (state.coverage.route && state.coverage.route.length > 1) {
        drawPolyline(state.coverage.route, bounds, w, h, { color: "rgba(47, 111, 237, 0.8)", width: 2.5 });
      }
    }

    function drawPerimeter(bounds, w, h) {
      if (!ui.showPerimeter) return;
      if (state.perimeter.outer && state.perimeter.outer.length > 1) {
        drawPolyline(state.perimeter.outer, bounds, w, h, { color: "#0f766e", width: 3 });
      }
      for (const hole of state.perimeter.holes || []) {
        drawPolyline(hole, bounds, w, h, { color: "#f97316", width: 2, dash: [6, 5] });
      }
      if (state.perimeter.recording_path && state.perimeter.recording_path.length > 1) {
        drawPolyline(state.perimeter.recording_path, bounds, w, h, { color: "#1d4ed8", width: 2 });
      }
    }

    function drawOdom(bounds, w, h) {
      if (!ui.showOdom) return;
      drawPolyline(state.path, bounds, w, h, { color: "#0ea5a4", width: 2 });
    }

    function draw() {
      const w = c.clientWidth;
      const h = c.clientHeight;
      ctx.clearRect(0, 0, w, h);
      drawGrid(w, h);

      const bounds = computeBounds();
      drawCoverage(bounds, w, h);
      drawRoute(bounds, w, h);
      drawPerimeter(bounds, w, h);
      drawOdom(bounds, w, h);

      if (sim.point) {
        drawRobot({x: sim.point.x, y: sim.point.y, theta: 0}, bounds, w, h, "rgba(79, 70, 229, 0.7)");
      }

      if (state.pose) {
        drawRobot(state.pose, bounds, w, h, "#ef4444");
      }
    }

    function setText(id, value) {
      document.getElementById(id).textContent = value;
    }

    function updateStats(data) {
      const pose = data.pose || { x: 0, y: 0, theta: 0 };
      setText("xv", pose.x.toFixed(3));
      setText("yv", pose.y.toFixed(3));
      setText("hv", (pose.theta * 180 / Math.PI).toFixed(1));
      setText("dv", (data.total_distance_m || 0).toFixed(3));
      setText("uv", String(data.update_count || 0));
      setText("mtv", (data.meters_per_tick || 0).toFixed(6));
    }

    function updatePerimeterUI() {
      const outerCount = state.perimeter.outer ? state.perimeter.outer.length : 0;
      const holeCount = state.perimeter.holes ? state.perimeter.holes.length : 0;
      setText("outerCount", String(outerCount));
      setText("holeCount", String(holeCount));
      const recState = state.perimeter.rec_state || "idle";
      const stage = state.perimeter.rec_stage || "outer";
      document.getElementById("recStatus").textContent = `Recorder: ${recState} (${stage})`;
    }

    function buildSimPath() {
      const segments = sim.segments;
      segments.length = 0;
      let total = 0;
      const route = state.coverage.route && state.coverage.route.length > 1
        ? [state.coverage.route]
        : (state.coverage.passes || []);
      for (const pass of route) {
        if (!pass || pass.length < 2) continue;
        for (let i = 1; i < pass.length; i++) {
          const a = pass[i - 1];
          const b = pass[i];
          const len = Math.hypot(b.x - a.x, b.y - a.y);
          if (len > 1e-6) {
            segments.push({ a, b, len, start: total, end: total + len });
            total += len;
          }
        }
      }
      sim.total = total;
      sim.dist = 0;
      sim.point = null;
    }

    function applyCoverage(coverage) {
      if (coverage === undefined || coverage === null) return;
      state.coverage = coverage;
      buildSimPath();
    }

    function updateSim(dt) {
      if (!sim.active || sim.total <= 0) {
        sim.point = null;
        return;
      }
      sim.dist += sim.speed * dt;
      if (sim.dist > sim.total) {
        sim.dist = 0;
      }
      const d = sim.dist;
      for (const seg of sim.segments) {
        if (d <= seg.end) {
          const t = (d - seg.start) / seg.len;
          sim.point = {
            x: seg.a.x + (seg.b.x - seg.a.x) * t,
            y: seg.a.y + (seg.b.y - seg.a.y) * t
          };
          return;
        }
      }
    }

    async function postJson(url, body) {
      const res = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: body ? JSON.stringify(body) : null
      });
      if (!res.ok) {
        throw new Error(await res.text());
      }
    }

    function computeTeleop() {
      let v = 0;
      let w = 0;
      if (pressed.has("w") || pressed.has("arrowup")) v += teleop.speed;
      if (pressed.has("s") || pressed.has("arrowdown")) v -= teleop.speed;
      if (pressed.has("a") || pressed.has("arrowleft")) w += teleop.turn;
      if (pressed.has("d") || pressed.has("arrowright")) w -= teleop.turn;
      return { v, w };
    }

    setInterval(async () => {
      if (!teleop.enabled) return;
      const { v, w } = computeTeleop();
      const moving = Math.abs(v) > 1e-3 || Math.abs(w) > 1e-3;
      try {
        if (moving) {
          await postJson("/cmd/vel", { v, w });
          teleop.moving = true;
        } else if (teleop.moving) {
          await postJson("/cmd/stop");
          teleop.moving = false;
        }
      } catch (err) {
        setStatus(false, "Command error", String(err));
      }
    }, 100);

    window.addEventListener("keydown", (e) => {
      if (e.repeat) return;
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(key)) {
        pressed.add(key);
      }
    });
    window.addEventListener("keyup", (e) => {
      const key = e.key.toLowerCase();
      pressed.delete(key);
    });
    window.addEventListener("blur", () => pressed.clear());

    const es = new EventSource("/events");
    es.onmessage = (evt) => {
      lastEventTs = Date.now();
      const data = JSON.parse(evt.data);
      state.path = data.path || [];
      state.pose = data.pose || null;
      state.connected = !!data.connected;
      state.lastError = data.last_error || null;
      state.perimeter = data.perimeter || state.perimeter;
      applyCoverage(data.coverage);
      state.plan = data.plan || state.plan;
      if (typeof data.sandbox === "boolean" && sandboxToggle) {
        sandboxToggle.checked = data.sandbox;
      }
      updateStats(data);
      updatePerimeterUI();
        if (data.plan) {
          if (typeof data.plan.ts === "number" && data.plan.ts !== lastPlanTs) {
            lastPlanTs = data.plan.ts;
            if (data.plan.error && data.plan.error !== lastPlanError) {
              lastPlanError = data.plan.error;
              log(`Planner error: ${data.plan.error}`);
            } else if (!data.plan.error && data.plan.ts !== lastPlanSummaryTs) {
              lastPlanSummaryTs = data.plan.ts;
              logPlanSummary(data.plan);
            }
          }
        }
      if (state.connected) {
        const modeText = data.sandbox ? "Sandbox mode" : `Recorder: ${state.perimeter.rec_state || "idle"}`;
        setStatus(true, "Live", modeText);
      } else if (data.worker_alive === false) {
        setStatus(false, "Telemetry stopped", "Check server log");
      } else if (state.lastError) {
        setStatus(false, "Disconnected", state.lastError);
      } else {
        setStatus(false, "Connecting...", "Waiting for telemetry");
      }
      draw();
    };
    es.onerror = () => {
      setStatus(false, "Stream disconnected");
      log("SSE disconnected");
    };

    on("fitBtn", "click", () => draw());
    on("resetBtn", "click", async () => {
      try {
        await postJson("/reset");
        log("Odometry reset");
      } catch (err) {
        setStatus(false, "Reset error", String(err));
        log(`Reset error: ${String(err)}`);
      }
    });
    on("manualBtn", "click", async () => {
      try {
        await postJson("/mode/manual");
        log("Mode set to MANUAL");
      } catch (err) {
        setStatus(false, "Mode error", String(err));
        log(`Mode error: ${String(err)}`);
      }
    });
    on("resetFaultsBtn", "click", async () => {
      try {
        await postJson("/cmd/reset_faults");
        log("Fault reset requested");
      } catch (err) {
        setStatus(false, "Fault reset error", String(err));
        log(`Fault reset error: ${String(err)}`);
      }
    });
    on("outerBtn", "click", async () => {
      try {
        await postJson("/rec/start_outer");
        log("Recording OUTER perimeter");
      } catch (err) {
        setStatus(false, "Recorder error", String(err));
        log(`Recorder error: ${String(err)}`);
      }
    });
    on("holeBtn", "click", async () => {
      try {
        await postJson("/rec/start_hole");
        log("Recording HOLE perimeter");
      } catch (err) {
        setStatus(false, "Recorder error", String(err));
        log(`Recorder error: ${String(err)}`);
      }
    });
    on("recStopBtn", "click", async () => {
      try {
        await postJson("/rec/stop");
        log("Recording stopped");
      } catch (err) {
        setStatus(false, "Recorder error", String(err));
        log(`Recorder error: ${String(err)}`);
      }
    });
    on("clearBtn", "click", async () => {
      try {
        await postJson("/rec/clear");
        log("Cleared perimeters");
      } catch (err) {
        setStatus(false, "Recorder error", String(err));
        log(`Recorder error: ${String(err)}`);
      }
    });
    function sleep(ms) {
      return new Promise((resolve) => setTimeout(resolve, ms));
    }
    async function fetchPlanUpdate(prevTs) {
      let data = null;
      for (let i = 0; i < 10; i++) {
        await sleep(120);
        const res = await fetch("/health");
        if (!res.ok) continue;
        data = await res.json();
        const ts = (data.plan && data.plan.ts) ? data.plan.ts : 0;
        if (ts && ts !== prevTs) return data;
        if (data.plan && data.plan.error) return data;
      }
      return data;
    }
    on("planBtn", "click", async () => {
      const spacing = parseFloat(document.getElementById("spacingInput").value || "0.45");
      const angle = parseFloat(document.getElementById("angleInput").value || "0");
      const patternEl = document.getElementById("patternSelect");
      const edgeEl = document.getElementById("edgeInput");
      const pattern = patternEl ? patternEl.value : "hybrid";
      const edgePasses = parseInt(edgeEl ? edgeEl.value : "2", 10);
      try {
        const prevTs = (state.plan && state.plan.ts) ? state.plan.ts : 0;
          await postJson("/plan", { line_spacing_m: spacing, sweep_angle_deg: angle, pattern, edge_passes: edgePasses });
          const data = await fetchPlanUpdate(prevTs) || await (await fetch("/health")).json();
          applyCoverage(data.coverage);
          state.plan = data.plan || state.plan;
          const plan = data.plan || {};
          const ts = (plan && typeof plan.ts === "number") ? plan.ts : 0;
          if (ts && ts !== prevTs && !plan.error) {
            lastPlanSummaryTs = ts;
            logPlanSummary(plan, spacing, angle);
          } else if (!plan.error) {
            log("Planning... waiting for result");
          }
          if (plan.error) {
            log(`Planner: ${plan.error}`);
          }
        } catch (err) {
          setStatus(false, "Planner error", String(err));
          log(`Planner error: ${String(err)}`);
        }
      });
    on("simBtn", "click", () => {
      const hasRoute = state.coverage.route && state.coverage.route.length > 1;
      const hasPasses = state.coverage.passes && state.coverage.passes.length > 0;
      if (!hasRoute && !hasPasses) {
        setStatus(false, "Preview", "Generate passes first");
        log("Preview blocked: no route/passes");
        return;
      }
      sim.active = !sim.active;
      buildSimPath();
      document.getElementById("simBtn").textContent = sim.active ? "Pause Preview" : "Play Preview";
      log(sim.active ? "Preview started" : "Preview paused");
    });
    on("exportBtn", "click", async () => {
      try {
        const res = await fetch("/export");
        if (!res.ok) throw new Error(await res.text());
        const blob = await res.blob();
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `route_${Date.now()}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        log("Route exported");
      } catch (err) {
        setStatus(false, "Export error", String(err));
        log(`Export error: ${String(err)}`);
      }
    });
    if (execBtn) {
      execBtn.addEventListener("click", async () => {
        try {
          if (!execActive) {
            await postJson("/exec/start", {
              allow: allowHwExec ? allowHwExec.checked : false,
              speed: teleop.speed,
              max_w: teleop.turn
            });
            execActive = true;
            execBtn.textContent = "Stop Execute";
            log("Execution started");
          } else {
            await postJson("/exec/stop", {});
            execActive = false;
            execBtn.textContent = "Execute Route";
            log("Execution stopped");
          }
        } catch (err) {
          setStatus(false, "Execute error", String(err));
          log(`Execute error: ${String(err)}`);
        }
      });
    }

    on("teleopToggle", "change", async (e) => {
      teleop.enabled = e.target.checked;
      if (teleop.enabled) {
        try {
          await postJson("/mode/manual");
          log("Teleop enabled");
        } catch (err) {
          setStatus(false, "Mode error", String(err));
          log(`Mode error: ${String(err)}`);
        }
      } else {
        log("Teleop disabled");
      }
    });
    if (sandboxToggle) {
      sandboxToggle.addEventListener("change", async (e) => {
        try {
          await postJson("/sandbox", { enabled: e.target.checked });
          log(e.target.checked ? "Sandbox enabled" : "Sandbox disabled");
        } catch (err) {
          setStatus(false, "Sandbox error", String(err));
          log(`Sandbox error: ${String(err)}`);
        }
      });
    }
    on("speedRange", "input", (e) => {
      teleop.speed = Math.max(0.0, Math.min(MAX_LINEAR_VEL, parseFloat(e.target.value)));
      document.getElementById("speedVal").textContent = `${teleop.speed.toFixed(2)} m/s`;
      if (execActive) {
        postJson("/exec/config", { speed: teleop.speed, max_w: teleop.turn }).catch(() => {});
      }
    });
    on("turnRange", "input", (e) => {
      teleop.turn = Math.max(0.0, Math.min(MAX_ANGULAR_VEL, parseFloat(e.target.value)));
      document.getElementById("turnVal").textContent = `${teleop.turn.toFixed(2)} rad/s`;
      if (execActive) {
        postJson("/exec/config", { speed: teleop.speed, max_w: teleop.turn }).catch(() => {});
      }
    });
    on("stopBtn", "click", async () => {
      pressed.clear();
      try {
        await postJson("/cmd/stop");
      } catch (err) {
        setStatus(false, "Stop error", String(err));
      }
    });
    on("showPerimeter", "change", (e) => {
      ui.showPerimeter = e.target.checked;
      draw();
    });
    on("showCoverage", "change", (e) => {
      ui.showCoverage = e.target.checked;
      draw();
    });
    on("showRoute", "change", (e) => {
      ui.showRoute = e.target.checked;
      draw();
    });
    on("showOdom", "change", (e) => {
      ui.showOdom = e.target.checked;
      draw();
    });

    let lastTs = performance.now();
    function tick(ts) {
      const dt = (ts - lastTs) / 1000;
      lastTs = ts;
      updateSim(dt);
      draw();
      requestAnimationFrame(tick);
    }
    resizeCanvas();
    requestAnimationFrame(tick);

    let lastEventTs = 0;
    let lastHealthOk = false;
    setStatus(false, "UI ready", "Waiting for events");
    log("UI ready");
    setInterval(async () => {
      const now = Date.now();
      if (now - lastEventTs < 2500) {
        lastHealthOk = true;
        return;
      }
      try {
        const res = await fetch("/health");
        if (!res.ok) throw new Error("health failed");
        const data = await res.json();
        state.path = data.path || [];
        state.pose = data.pose || null;
        state.connected = !!data.connected;
        state.lastError = data.last_error || null;
        state.perimeter = data.perimeter || state.perimeter;
        applyCoverage(data.coverage);
        state.plan = data.plan || state.plan;
        updateStats(data);
        updatePerimeterUI();
        draw();
        if (!lastHealthOk) log("Health fallback active");
        lastHealthOk = true;
      } catch (err) {
        if (lastHealthOk) log("Health fallback failed");
        lastHealthOk = false;
        setStatus(false, "Disconnected", "No events / health");
      }
    }, 2000);
  </script>
</body>
</html>"""


class SharedState:
    def __init__(self, max_points: int = 20000):
        self._lock = threading.Lock()
        self._max_points = max_points
        self._path: List[Dict[str, float]] = []
        self._pose: Optional[Dict[str, float]] = None
        self._total_distance_m: float = 0.0
        self._update_count: int = 0
        self._meters_per_tick: float = 0.0
        self._connected: bool = False
        self._last_error: Optional[str] = None
        self._outer: Optional[List[Dict[str, float]]] = None
        self._holes: List[List[Dict[str, float]]] = []
        self._recording_path: List[Dict[str, float]] = []
        self._rec_state: str = "idle"
        self._rec_stage: str = "outer"
        self._coverage_passes: List[List[Dict[str, float]]] = []
        self._coverage_line_spacing: float = 0.45
        self._coverage_sweep_angle: float = 0.0
        self._coverage_route: List[Dict[str, float]] = []
        self._plan_info: Dict[str, Any] = {
            "passes": 0,
            "route_pts": 0,
            "error": None,
            "ts": 0.0,
            "pattern": None,
            "edge_passes": None,
            "audit": None,
        }
        self._sandbox: bool = False
        self._worker_alive: bool = False

    def set_connection(self, connected: bool, error: Optional[str] = None) -> None:
        with self._lock:
            self._connected = connected
            self._last_error = error

    def clear_path(self) -> None:
        with self._lock:
            self._path = []

    def clear_perimeters(self) -> None:
        with self._lock:
            self._outer = None
            self._holes = []
            self._recording_path = []
            self._rec_state = "idle"
            self._rec_stage = "outer"

    def set_perimeters(
        self,
        outer: Optional[List[Dict[str, float]]],
        holes: List[List[Dict[str, float]]],
        rec_state: str,
        rec_stage: str,
        recording_path: List[Dict[str, float]],
    ) -> None:
        with self._lock:
            self._outer = outer
            self._holes = holes
            self._rec_state = rec_state
            self._rec_stage = rec_stage
            self._recording_path = recording_path

    def clear_coverage(self) -> None:
        with self._lock:
            self._coverage_passes = []
            self._coverage_route = []

    def set_coverage(
        self,
        passes: List[List[Dict[str, float]]],
        line_spacing: float,
        sweep_angle: float,
        route: Optional[List[Dict[str, float]]] = None,
        error: Optional[str] = None,
        pattern: Optional[str] = None,
        edge_passes: Optional[int] = None,
        audit: Optional[Dict[str, Any]] = None,
    ) -> None:
        max_passes = 20000
        max_points = 500000
        max_route = 200000
        total_points = sum(len(p) for p in passes)
        pass_count = len(passes)
        route_count = len(route) if route is not None else 0
        if passes and (pass_count > max_passes or total_points > max_points):
            passes = []
            error = error or f"coverage too large (passes={pass_count}, points={total_points})"
        if route is not None and route_count > max_route:
            route = []
            error = error or f"route too large ({route_count} pts)"
        with self._lock:
            self._coverage_passes = passes
            self._coverage_line_spacing = line_spacing
            self._coverage_sweep_angle = sweep_angle
            if route is not None:
                self._coverage_route = route
            self._plan_info = {
                "passes": len(passes),
                "route_pts": len(route or []),
                "error": error,
                "ts": time.time(),
                "pattern": pattern,
                "edge_passes": edge_passes,
                "audit": audit,
            }

    def set_sandbox(self, enabled: bool) -> None:
        with self._lock:
            self._sandbox = enabled

    def is_sandbox(self) -> bool:
        with self._lock:
            return self._sandbox

    def set_worker_alive(self, alive: bool) -> None:
        with self._lock:
            self._worker_alive = alive

    def update(
        self,
        pose: Optional[Any],
        stats: Optional[Dict[str, Any]],
        connected: bool,
        error: Optional[str] = None,
    ) -> None:
        with self._lock:
            self._connected = connected
            self._last_error = error

            if pose is not None:
                pose_dict = {"x": float(pose.x), "y": float(pose.y), "theta": float(pose.theta)}
                self._pose = pose_dict
                self._append_path_point(pose_dict["x"], pose_dict["y"])

            if stats:
                self._total_distance_m = float(stats.get("total_distance_m", self._total_distance_m))
                self._update_count = int(stats.get("update_count", self._update_count))
                self._meters_per_tick = float(stats.get("meters_per_tick", self._meters_per_tick))

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "path": list(self._path),
                "pose": dict(self._pose) if self._pose is not None else None,
                "total_distance_m": self._total_distance_m,
                "update_count": self._update_count,
                "meters_per_tick": self._meters_per_tick,
                "connected": self._connected,
                "last_error": self._last_error,
                "perimeter": {
                    "outer": list(self._outer) if self._outer is not None else [],
                    "holes": [list(h) for h in self._holes],
                    "recording_path": list(self._recording_path),
                    "rec_state": self._rec_state,
                    "rec_stage": self._rec_stage,
                },
                "coverage": {
                    "passes": [list(p) for p in self._coverage_passes],
                    "line_spacing_m": self._coverage_line_spacing,
                    "sweep_angle_rad": self._coverage_sweep_angle,
                    "route": list(self._coverage_route),
                },
                "plan": dict(self._plan_info),
                "sandbox": self._sandbox,
                "worker_alive": self._worker_alive,
                "ts": time.time(),
            }

    def snapshot_lite(self, path_limit: int = 600) -> Dict[str, Any]:
        with self._lock:
            path = self._path[-path_limit:] if path_limit > 0 else []
            return {
                "path": list(path),
                "pose": dict(self._pose) if self._pose is not None else None,
                "total_distance_m": self._total_distance_m,
                "update_count": self._update_count,
                "meters_per_tick": self._meters_per_tick,
                "connected": self._connected,
                "last_error": self._last_error,
                "perimeter": {
                    "outer": list(self._outer) if self._outer is not None else [],
                    "holes": [list(h) for h in self._holes],
                    "recording_path": list(self._recording_path),
                    "rec_state": self._rec_state,
                    "rec_stage": self._rec_stage,
                },
                # Omit coverage in SSE to keep payloads small; UI keeps last known coverage.
                "plan": dict(self._plan_info),
                "sandbox": self._sandbox,
                "worker_alive": self._worker_alive,
                "ts": time.time(),
            }

    def _append_path_point(self, x: float, y: float) -> None:
        if not self._path:
            self._path.append({"x": x, "y": y})
            return

        last = self._path[-1]
        if math.hypot(x - last["x"], y - last["y"]) < 0.004:
            return

        self._path.append({"x": x, "y": y})
        if len(self._path) > self._max_points:
            self._path = self._path[-self._max_points :]


def _waypoints_to_points(waypoints: List[Any]) -> List[Dict[str, float]]:
    return [{"x": float(w.x), "y": float(w.y)} for w in waypoints]


def _perimeter_to_points(perimeter: Any) -> Optional[List[Dict[str, float]]]:
    if perimeter is None:
        return None
    return _waypoints_to_points(perimeter.waypoints)


def _coverage_to_passes(coverage: Any) -> List[List[Dict[str, float]]]:
    if coverage is None:
        return []
    passes = []
    for p in coverage.passes:
        pts = [{"x": float(pt.x), "y": float(pt.y)} for pt in p.points]
        if pts:
            passes.append(pts)
    return passes


def _simplify_poly(points: List[Point], tol: float) -> List[Point]:
    if len(points) < 4:
        return points

    def dist_point_seg(p: Point, a: Point, b: Point) -> float:
        q = _nearest_point_on_segment(p, a, b)
        return math.hypot(p[0] - q[0], p[1] - q[1])

    def rdp(pts: List[Point]) -> List[Point]:
        if len(pts) < 3:
            return pts
        a = pts[0]
        b = pts[-1]
        max_d = -1.0
        idx = -1
        for i in range(1, len(pts) - 1):
            d = dist_point_seg(pts[i], a, b)
            if d > max_d:
                max_d = d
                idx = i
        if max_d > tol:
            left = rdp(pts[: idx + 1])
            right = rdp(pts[idx:])
            return left[:-1] + right
        return [a, b]

    return rdp(points)


def _segment_intersects(a: Point, b: Point, c: Point, d: Point) -> bool:
    def ccw(p1, p2, p3) -> bool:
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])
    return (ccw(a, c, d) != ccw(b, c, d)) and (ccw(a, b, c) != ccw(a, b, d))


def _self_intersections(poly: List[Point]) -> int:
    n = len(poly)
    if n < 4:
        return 0
    count = 0
    for i in range(n):
        a1 = poly[i]
        a2 = poly[(i + 1) % n]
        for j in range(i + 1, n):
            if abs(i - j) <= 1 or (i == 0 and j == n - 1):
                continue
            b1 = poly[j]
            b2 = poly[(j + 1) % n]
            if _segment_intersects(a1, a2, b1, b2):
                count += 1
    return count


def _polygon_area(poly: List[Point]) -> float:
    n = len(poly)
    if n < 3:
        return 0.0
    a = 0.0
    for i in range(n):
        j = (i + 1) % n
        a += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    return abs(a) / 2.0


def _finite_point(p: Point) -> bool:
    return math.isfinite(p[0]) and math.isfinite(p[1])


def _filter_finite(poly: List[Point]) -> List[Point]:
    return [p for p in poly if _finite_point(p)]


def _poly_bounds(poly: List[Point]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return (min(xs), min(ys), max(xs), max(ys))


def _dedupe_poly(points: List[Point], tol: float = 1e-6) -> List[Point]:
    if not points:
        return points
    out = [points[0]]
    for p in points[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > tol:
            out.append(p)
    return out


def _prune_poly_loops(poly: List[Point], tol: float, min_length: float) -> List[Point]:
    if len(poly) < 4:
        return poly

    route = [{"x": p[0], "y": p[1]} for p in poly]

    def dist(a, b) -> float:
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    changed = True
    out = list(route)
    while changed:
        changed = False
        cell: Dict[Tuple[int, int], int] = {}
        acc = [0.0]
        for i in range(1, len(out)):
            acc.append(acc[-1] + dist(out[i - 1], out[i]))
        for i, p in enumerate(out):
            key = (int(p["x"] / tol), int(p["y"] / tol))
            if key in cell:
                j = cell[key]
                # Preserve terminal closure (end near start) for polygons.
                if j == 0 and i == len(out) - 1:
                    continue
                if i - j > 2 and (acc[i] - acc[j]) > min_length and dist(out[i], out[j]) < tol:
                    out = out[: j + 1] + out[i:]
                    changed = True
                    break
            else:
                cell[key] = i
    return [(p["x"], p["y"]) for p in out]


def _convex_hull(points: List[Point]) -> List[Point]:
    if len(points) <= 3:
        return points
    pts = sorted(set(points))
    if len(pts) <= 3:
        return pts

    def cross(o, a, b) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def _build_route_points(
    coverage: Any,
    start_pose: Optional[Pose],
    turn_offset: float,
    outer_poly: Optional[List[Point]],
    hole_polys: List[List[Point]],
    nav: Optional[Dict[str, Any]] = None,
    smooth_route: bool = False,
) -> List[Dict[str, float]]:
    if coverage is None or not coverage.passes:
        return []

    passes = list(coverage.passes)

    if start_pose is not None and outer_poly:
        start_point = (start_pose.x, start_pose.y)
        if not _point_in_poly(start_point, outer_poly):
            nx, ny = _nearest_point_on_polygon(start_point, outer_poly)
            start_pose = Pose(x=nx, y=ny, theta=start_pose.theta)
        else:
            for hole in hole_polys:
                if _point_in_poly(start_point, hole):
                    nx, ny = _nearest_point_on_polygon(start_point, outer_poly)
                    start_pose = Pose(x=nx, y=ny, theta=start_pose.theta)
                    break

    def pass_kind(p: Any) -> str:
        return str(p.metadata.get("pattern", "scanline")).lower()

    def pass_start_distance(p: Any, point: Point) -> float:
        pts = p.points
        if not pts:
            return float("inf")
        kind = pass_kind(p)
        if kind == "contour":
            return min(math.hypot(pt.x - point[0], pt.y - point[1]) for pt in pts)
        p0 = pts[0]
        p1 = pts[-1]
        return min(
            math.hypot(p0.x - point[0], p0.y - point[1]),
            math.hypot(p1.x - point[0], p1.y - point[1]),
        )

    def rotate_contour_points(pts: List[Any], start: Point) -> List[Any]:
        if len(pts) < 3:
            return pts
        best_idx = 0
        best_d = float("inf")
        for i, pt in enumerate(pts):
            d = math.hypot(pt.x - start[0], pt.y - start[1])
            if d < best_d:
                best_d = d
                best_idx = i
        if best_idx == 0:
            return pts
        return pts[best_idx:] + pts[:best_idx]

    contour_passes = [p for p in passes if pass_kind(p) == "contour"]
    scanline_passes = [p for p in passes if pass_kind(p) != "contour"]

    if contour_passes and scanline_passes and start_pose is not None:
        start_xy = (start_pose.x, start_pose.y)
        dist_contour = min(pass_start_distance(p, start_xy) for p in contour_passes)
        dist_scanline = min(pass_start_distance(p, start_xy) for p in scanline_passes)
        if dist_scanline <= dist_contour:
            first_group = list(scanline_passes)
            second_group = list(contour_passes)
        else:
            first_group = list(contour_passes)
            second_group = list(scanline_passes)
        # Rotate within the starting group so we begin closest to the start pose.
        best = float("inf")
        start_idx = 0
        for i, p in enumerate(first_group):
            d = pass_start_distance(p, start_xy)
            if d < best:
                best = d
                start_idx = i
        first_group = first_group[start_idx:] + first_group[:start_idx]
        ordered = first_group + second_group
    else:
        ordered = passes
        if start_pose is not None:
            start_xy = (start_pose.x, start_pose.y)
            best = float("inf")
            start_idx = 0
            for i, p in enumerate(ordered):
                d = pass_start_distance(p, start_xy)
                if d < best:
                    best = d
                    start_idx = i
            ordered = ordered[start_idx:] + ordered[:start_idx]
    route: List[Dict[str, float]] = []
    last_pass_heading: Optional[float] = None
    spacing = max(turn_offset * 2.0, 0.1)

    if start_pose is not None:
        route.append({"x": float(start_pose.x), "y": float(start_pose.y)})

    def append_point(x: float, y: float) -> None:
        if route and math.hypot(route[-1]["x"] - x, route[-1]["y"] - y) < 1e-6:
            return
        route.append({"x": x, "y": y})

    for idx, p in enumerate(ordered):
        pts = list(p.points)

        if not pts:
            continue
        if route:
            cur = (route[-1]["x"], route[-1]["y"])
            if pass_kind(p) == "contour":
                pts = rotate_contour_points(pts, cur)
            else:
                d0 = math.hypot(pts[0].x - cur[0], pts[0].y - cur[1])
                d1 = math.hypot(pts[-1].x - cur[0], pts[-1].y - cur[1])
                if d1 < d0:
                    pts = list(reversed(pts))
        pass_start_heading: Optional[float] = None
        pass_end_heading: Optional[float] = None
        if len(pts) >= 2:
            pass_start_heading = math.atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x)
            pass_end_heading = math.atan2(pts[-1].y - pts[-2].y, pts[-1].x - pts[-2].x)

        start_pt = pts[0]

        if route:
            last = route[-1]
            connector = _connector_path(
                (last["x"], last["y"]),
                (start_pt.x, start_pt.y),
                outer_poly,
                hole_polys,
                turn_offset,
                nav,
                start_heading=last_pass_heading,
                end_heading=pass_start_heading,
            )
            connector = _simplify_connector(connector, spacing)
            for cx, cy in connector:
                append_point(cx, cy)

        for pt in pts:
            append_point(float(pt.x), float(pt.y))
        last_pass_heading = pass_end_heading

    if route:
        raw_route = route
        if len(route) <= 50000:
            spike_tol = max(0.06, spacing * 0.15)
            spike_len = max(0.35, spacing * 0.9)
            route = _prune_spikes(route, tol=spike_tol, max_len=spike_len)
            loop_tol = max(0.12, spacing * 0.25)
            loop_min = max(0.6, spacing * 1.2)
            route = _prune_loops(route, tol=loop_tol, min_length=loop_min)
        if len(route) >= 3:
            # Coalesce dense micro-segments before filleting so infill corners can round visibly.
            min_pt = max(0.02, spacing * 0.03)
            pts_clean: List[Point] = []
            for p in route:
                pt = (float(p["x"]), float(p["y"]))
                if not pts_clean or math.hypot(pt[0] - pts_clean[-1][0], pt[1] - pts_clean[-1][1]) >= min_pt:
                    pts_clean.append(pt)
            pts_clean = _simplify_collinear(pts_clean, math.radians(3.0), min_pt)
            if len(pts_clean) >= 2:
                route = [{"x": p[0], "y": p[1]} for p in pts_clean]
        # Always apply geometric corner rounding so dense infill routes are rounded too.
        round_radius = max(0.10, min(spacing * 0.85, 0.70))
        round_step = max(0.03, min(spacing * 0.15, 0.08))
        rounded = _round_route_corners(
            route,
            radius=round_radius,
            step_len=round_step,
            outer=outer_poly,
            holes=hole_polys,
        )
        if not outer_poly or _route_segments_clear(rounded, outer_poly, hole_polys):
            route = rounded
        if outer_poly and not _route_segments_clear(route, outer_poly, hole_polys):
            route = raw_route

    if outer_poly and smooth_route:
        route = _prune_loops(route, tol=0.35, min_length=1.0)
        route = _smooth_route(route, outer_poly, hole_polys, passes=2)

    return route


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
    t = max(0.0, min(1.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def _simplify_collinear(points: List[Point], angle_tol_rad: float, min_dist: float) -> List[Point]:
    if len(points) < 3:
        return points
    out = [points[0]]
    for i in range(1, len(points) - 1):
        a = out[-1]
        b = points[i]
        c = points[i + 1]
        if math.hypot(b[0] - a[0], b[1] - a[1]) < min_dist:
            continue
        if math.hypot(c[0] - b[0], c[1] - b[1]) < min_dist:
            continue
        v1x = b[0] - a[0]
        v1y = b[1] - a[1]
        v2x = c[0] - b[0]
        v2y = c[1] - b[1]
        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cosang = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (n1 * n2)))
        ang = math.acos(cosang)
        if ang < angle_tol_rad or abs(math.pi - ang) < angle_tol_rad:
            continue
        out.append(b)
    out.append(points[-1])
    return out


def _simplify_connector(connector: List[Point], spacing: float) -> List[Point]:
    if not connector or len(connector) < 3:
        return connector

    min_dist = max(0.02, spacing * 0.05)
    angle_tol = math.radians(6.0)

    cleaned: List[Point] = []
    for p in connector:
        if not cleaned:
            cleaned.append(p)
            continue
        if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) < min_dist:
            continue
        cleaned.append(p)

    cleaned = _simplify_collinear(cleaned, angle_tol, min_dist)

    if len(cleaned) >= 4:
        tol = max(0.1, spacing * 0.2)
        min_len = max(0.4, spacing * 0.8)
        as_dict = [{"x": p[0], "y": p[1]} for p in cleaned]
        pruned = _prune_loops(as_dict, tol=tol, min_length=min_len)
        cleaned = [(p["x"], p["y"]) for p in pruned]

    return cleaned


def _line_intersection(p: Point, r: Point, q: Point, s: Point) -> Optional[Point]:
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-9:
        return None
    qp = (q[0] - p[0], q[1] - p[1])
    t = (qp[0] * s[1] - qp[1] * s[0]) / den
    return (p[0] + t * r[0], p[1] + t * r[1])


def _round_route_corners(
    route: List[Dict[str, float]],
    radius: float,
    step_len: float,
    outer: Optional[List[Point]] = None,
    holes: Optional[List[List[Point]]] = None,
) -> List[Dict[str, float]]:
    if len(route) < 3 or radius <= 1e-4:
        return route

    pts: List[Point] = [(float(p["x"]), float(p["y"])) for p in route]
    out: List[Point] = [pts[0]]
    holes_local = holes or []
    min_corner_deg = 4.0
    max_corner_deg = 179.4

    for i in range(1, len(pts) - 1):
        p0 = pts[i - 1]
        p1 = pts[i]
        p2 = pts[i + 1]

        d1 = (p1[0] - p0[0], p1[1] - p0[1])
        d2 = (p2[0] - p1[0], p2[1] - p1[1])
        l1 = math.hypot(d1[0], d1[1])
        l2 = math.hypot(d2[0], d2[1])
        if l1 < 1e-5 or l2 < 1e-5:
            out.append(p1)
            continue

        u1 = (d1[0] / l1, d1[1] / l1)
        u2 = (d2[0] / l2, d2[1] / l2)
        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        phi = math.acos(dot)
        phi_deg = math.degrees(phi)
        if phi_deg < min_corner_deg or phi_deg > max_corner_deg:
            out.append(p1)
            continue

        tan_half = math.tan(phi / 2.0)
        if abs(tan_half) < 1e-6:
            out.append(p1)
            continue

        max_r = min(radius, 0.45 * l1, 0.45 * l2)
        if max_r <= 1e-4:
            out.append(p1)
            continue
        t = max_r / tan_half
        if t <= 1e-4:
            out.append(p1)
            continue

        start = (p1[0] - u1[0] * t, p1[1] - u1[1] * t)
        end = (p1[0] + u2[0] * t, p1[1] + u2[1] * t)

        turn = u1[0] * u2[1] - u1[1] * u2[0]
        if abs(turn) < 1e-6:
            out.append(p1)
            continue

        if turn > 0.0:
            n1 = (-u1[1], u1[0])
            n2 = (-u2[1], u2[0])
        else:
            n1 = (u1[1], -u1[0])
            n2 = (u2[1], -u2[0])

        center = _line_intersection(start, n1, end, n2)
        if center is None:
            out.append(p1)
            continue

        rs = math.hypot(start[0] - center[0], start[1] - center[1])
        re = math.hypot(end[0] - center[0], end[1] - center[1])
        if rs < 1e-4 or abs(rs - re) > max(0.03, rs * 0.05):
            out.append(p1)
            continue

        a0 = math.atan2(start[1] - center[1], start[0] - center[0])
        a1 = math.atan2(end[1] - center[1], end[0] - center[0])
        if turn > 0.0:
            while a1 <= a0:
                a1 += 2.0 * math.pi
        else:
            while a1 >= a0:
                a1 -= 2.0 * math.pi
        span = a1 - a0
        arc_len = abs(span) * rs
        steps = max(1, int(arc_len / max(step_len, 0.03)))

        corner_pts: List[Point] = [start]
        for k in range(1, steps):
            t_arc = k / steps
            a = a0 + span * t_arc
            corner_pts.append((center[0] + rs * math.cos(a), center[1] + rs * math.sin(a)))
        corner_pts.append(end)

        if outer is not None:
            prev = out[-1]
            valid = True
            for cp in corner_pts:
                if math.hypot(cp[0] - prev[0], cp[1] - prev[1]) < 1e-4:
                    continue
                if not _segment_clear(prev, cp, outer, holes_local):
                    valid = False
                    break
                prev = cp
            if not valid:
                out.append(p1)
                continue

        for cp in corner_pts:
            if math.hypot(cp[0] - out[-1][0], cp[1] - out[-1][1]) < 1e-4:
                continue
            out.append(cp)

    if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) > 1e-6:
        out.append(pts[-1])

    return [{"x": p[0], "y": p[1]} for p in out]


def _prune_spikes(route: List[Dict[str, float]], tol: float, max_len: float) -> List[Dict[str, float]]:
    if len(route) < 3:
        return route
    out: List[Dict[str, float]] = []
    i = 0
    while i < len(route):
        if i + 2 < len(route):
            a = route[i]
            b = route[i + 1]
            c = route[i + 2]
            ab = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            bc = math.hypot(b["x"] - c["x"], b["y"] - c["y"])
            ac = math.hypot(a["x"] - c["x"], a["y"] - c["y"])
            if ac < tol and (ab + bc) < max_len:
                out.append(a)
                i += 2
                continue
        out.append(route[i])
        i += 1
    if out and route:
        if math.hypot(out[-1]["x"] - route[-1]["x"], out[-1]["y"] - route[-1]["y"]) > 1e-6:
            out.append(route[-1])
    return out


def _route_segments_clear(route: List[Dict[str, float]], outer: Optional[List[Point]], holes: List[List[Point]]) -> bool:
    if not outer or len(route) < 2:
        return True
    for i in range(1, len(route)):
        a = (route[i - 1]["x"], route[i - 1]["y"])
        b = (route[i]["x"], route[i]["y"])
        if not _segment_clear(a, b, outer, holes):
            return False
    return True


def _coverage_audit(
    coverage: Any,
    outer: Optional[List[Point]],
    holes: List[List[Point]],
    spacing: float,
    max_samples: int = 20000,
) -> Optional[Dict[str, Any]]:
    if coverage is None or not coverage.passes or not outer:
        return None

    min_x, min_y, max_x, max_y = _poly_bounds(outer)
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x <= 0 or span_y <= 0:
        return None

    step = max(spacing / 3.0, 0.15)
    est = int((span_x / step + 1) * (span_y / step + 1))
    if est > max_samples:
        scale = math.sqrt(est / max_samples)
        step *= scale

    segments: List[Tuple[Point, Point]] = []
    for p in coverage.passes:
        pts = p.points
        for i in range(1, len(pts)):
            segments.append(((pts[i - 1].x, pts[i - 1].y), (pts[i].x, pts[i].y)))
    if not segments:
        return None

    threshold = spacing * 0.4
    samples = 0
    missed = 0
    max_gap = 0.0
    y = min_y
    while y <= max_y:
        x = min_x
        while x <= max_x:
            if _point_in_poly((x, y), outer):
                inside_hole = False
                for h in holes:
                    if _point_in_poly((x, y), h):
                        inside_hole = True
                        break
                if not inside_hole:
                    samples += 1
                    best = float("inf")
                    for a, b in segments:
                        d = _point_segment_distance((x, y), a, b)
                        if d < best:
                            best = d
                            if best <= threshold:
                                break
                    if best > threshold:
                        missed += 1
                    if best > max_gap:
                        max_gap = best
                    if samples >= max_samples:
                        break
            x += step
        if samples >= max_samples:
            break
        y += step

    if samples == 0:
        return None
    coverage_pct = 100.0 * (samples - missed) / samples
    return {
        "samples": samples,
        "missed": missed,
        "max_gap_m": round(max_gap, 3),
        "coverage_pct": round(coverage_pct, 1),
        "step_m": round(step, 3),
    }


def _nearest_vertex(p: Point, poly: List[Point]) -> Point:
    best = poly[0]
    best_d = float("inf")
    for v in poly:
        d = math.hypot(v[0] - p[0], v[1] - p[1])
        if d < best_d:
            best_d = d
            best = v
    return best


def _nearest_point_on_segment(p: Point, a: Point, b: Point) -> Point:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-12:
        return a
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
    t = max(0.0, min(1.0, t))
    return (ax + t * dx, ay + t * dy)


def _nearest_point_on_polygon(p: Point, poly: List[Point]) -> Point:
    best = poly[0]
    best_d = float("inf")
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        q = _nearest_point_on_segment(p, a, b)
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d < best_d:
            best_d = d
            best = q
    return best


def _point_in_poly(p: Point, poly: List[Point]) -> bool:
    x, y = p
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    def ccw(p1, p2, p3) -> bool:
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])

    return (ccw(a, c, d) != ccw(b, c, d)) and (ccw(a, b, c) != ccw(a, b, d))


def _segment_intersects_poly(a: Point, b: Point, poly: List[Point]) -> bool:
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]
        if _segments_intersect(a, b, p1, p2):
            return True
    return False


def _segment_hits_holes(a: Point, b: Point, holes: List[List[Point]]) -> bool:
    for h in holes:
        if _segment_intersects_poly(a, b, h):
            return True
    return False


def _segment_clear(a: Point, b: Point, outer: Optional[List[Point]], holes: List[List[Point]]) -> bool:
    if outer is None:
        return True
    samples = 6
    for i in range(samples + 1):
        t = i / samples
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if not _point_in_poly((x, y), outer):
            return False
        for h in holes:
            if _point_in_poly((x, y), h):
                return False
    return True


def _boundary_path(poly: List[Point], i: int, j: int, direction: int) -> List[Point]:
    n = len(poly)
    if n == 0:
        return []
    path = [poly[i]]
    idx = i
    while idx != j:
        idx = (idx + direction) % n
        path.append(poly[idx])
    return path


def _path_length(path: List[Point]) -> float:
    total = 0.0
    for i in range(1, len(path)):
        total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return total


def _connector_path(
    start: Point,
    end: Point,
    outer: Optional[List[Point]],
    holes: List[List[Point]],
    turn_offset: float,
    nav: Optional[Dict[str, Any]],
    start_heading: Optional[float] = None,
    end_heading: Optional[float] = None,
) -> List[Point]:
    if math.hypot(end[0] - start[0], end[1] - start[1]) < 1e-6:
        return []

    if start_heading is not None and end_heading is not None:
        # Prefer a smooth cubic connector when pass headings are known.
        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        if dist > 1e-6:
            min_handle = max(0.06, min(turn_offset * 0.20, 0.16))
            max_handle = max(min_handle, min(turn_offset * 1.2, dist * 0.45))
            sample_step = max(0.05, min(turn_offset * 0.28, 0.16))
            samples = max(6, int(dist / sample_step))
            for scale in (1.0, 0.8, 0.65, 0.5):
                handle = max(min_handle, max_handle * scale)
                c1 = (
                    start[0] + math.cos(start_heading) * handle,
                    start[1] + math.sin(start_heading) * handle,
                )
                c2 = (
                    end[0] - math.cos(end_heading) * handle,
                    end[1] - math.sin(end_heading) * handle,
                )

                curve: List[Point] = []
                prev = start
                valid = True
                for i in range(1, samples + 1):
                    t = i / samples
                    omt = 1.0 - t
                    bx = (
                        omt * omt * omt * start[0]
                        + 3.0 * omt * omt * t * c1[0]
                        + 3.0 * omt * t * t * c2[0]
                        + t * t * t * end[0]
                    )
                    by = (
                        omt * omt * omt * start[1]
                        + 3.0 * omt * omt * t * c1[1]
                        + 3.0 * omt * t * t * c2[1]
                        + t * t * t * end[1]
                    )
                    p = (bx, by)
                    if not _segment_clear(prev, p, outer, holes):
                        valid = False
                        break
                    curve.append(p)
                    prev = p
                if valid and curve:
                    return curve

        radius = max(0.1, min(turn_offset, 1.2))
        sx = start[0] + math.cos(start_heading) * radius
        sy = start[1] + math.sin(start_heading) * radius
        ex = end[0] - math.cos(end_heading) * radius
        ey = end[1] - math.sin(end_heading) * radius
        spt = (sx, sy)
        ept = (ex, ey)
        if _segment_clear(start, spt, outer, holes) and _segment_clear(spt, ept, outer, holes) and _segment_clear(ept, end, outer, holes):
            return [spt, ept, end]

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    seg_len = math.hypot(dx, dy)
    if seg_len > 1e-6:
        nx = -dy / seg_len
        ny = dx / seg_len
        offset = max(0.1, min(turn_offset, 0.8))
        mid1 = (start[0] + nx * offset, start[1] + ny * offset)
        mid2 = (end[0] + nx * offset, end[1] + ny * offset)
        if _segment_clear(start, mid1, outer, holes) and _segment_clear(mid1, mid2, outer, holes) and _segment_clear(mid2, end, outer, holes):
            return [mid1, mid2, end]

    if _segment_clear(start, end, outer, holes):
        return [end]

    if ENABLE_NAV and nav is not None:
        path = _astar_path(start, end, nav, outer, holes)
        if path:
            return path

    if not outer:
        return [end]

    n = len(outer)
    if n < 3:
        return [end]

    best_path: Optional[List[Point]] = None
    best_len = float("inf")

    for i in range(n):
        vi = outer[i]
        if not _segment_clear(start, vi, outer, holes):
            continue
        for j in range(n):
            vj = outer[j]
            if not _segment_clear(end, vj, outer, holes):
                continue
            cw = _boundary_path(outer, i, j, 1)
            ccw = _boundary_path(outer, i, j, -1)
            for path in (cw, ccw):
                length = math.hypot(start[0] - vi[0], start[1] - vi[1]) + _path_length(path) + math.hypot(end[0] - vj[0], end[1] - vj[1])
                if length < best_len:
                    best_len = length
                    best_path = path

    if best_path is None:
        return [end]

    out: List[Point] = []
    out.append(best_path[0])
    for p in best_path[1:]:
        out.append(p)
    out.append(end)
    return out


def _build_nav_grid(
    outer: List[Point],
    holes: List[List[Point]],
    resolution: float,
    max_cells: int = 250_000,
) -> Optional[Dict[str, Any]]:
    min_x = min(p[0] for p in outer)
    max_x = max(p[0] for p in outer)
    min_y = min(p[1] for p in outer)
    max_y = max(p[1] for p in outer)

    res = max(0.2, min(resolution, 0.8))
    width = max(2, int(math.ceil((max_x - min_x) / res)) + 1)
    height = max(2, int(math.ceil((max_y - min_y) / res)) + 1)

    if width * height > max_cells:
        return None

    grid = bytearray(width * height)
    for iy in range(height):
        y = min_y + iy * res
        for ix in range(width):
            x = min_x + ix * res
            if not _point_in_poly((x, y), outer):
                continue
            inside_hole = False
            for h in holes:
                if _point_in_poly((x, y), h):
                    inside_hole = True
                    break
            if not inside_hole:
                grid[iy * width + ix] = 1

    return {
        "grid": grid,
        "min_x": min_x,
        "min_y": min_y,
        "res": res,
        "width": width,
        "height": height,
    }


def _grid_to_point(ix: int, iy: int, nav: Dict[str, Any]) -> Point:
    return (nav["min_x"] + ix * nav["res"], nav["min_y"] + iy * nav["res"])


def _nearest_walkable_cell(p: Point, nav: Dict[str, Any]) -> Optional[Tuple[int, int]]:
    grid = nav["grid"]
    width = nav["width"]
    height = nav["height"]
    best = None
    best_d = float("inf")
    for iy in range(height):
        for ix in range(width):
            if not grid[iy * width + ix]:
                continue
            x, y = _grid_to_point(ix, iy, nav)
            d = (x - p[0]) ** 2 + (y - p[1]) ** 2
            if d < best_d:
                best_d = d
                best = (ix, iy)
    return best


def _astar_path(
    start: Point,
    end: Point,
    nav: Dict[str, Any],
    outer: Optional[List[Point]],
    holes: List[List[Point]],
) -> List[Point]:
    grid = nav["grid"]
    width = nav["width"]
    height = nav["height"]
    start_cell = _nearest_walkable_cell(start, nav)
    end_cell = _nearest_walkable_cell(end, nav)
    if start_cell is None or end_cell is None:
        return []

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    total = width * height
    start_idx = start_cell[1] * width + start_cell[0]
    end_idx = end_cell[1] * width + end_cell[0]
    g_score = [math.inf] * total
    came_from = [-1] * total
    closed = bytearray(total)
    g_score[start_idx] = 0.0

    open_set: List[Tuple[float, int]] = []
    heapq.heappush(open_set, (heuristic(start_cell, end_cell), start_idx))
    max_steps = min(total * 4, NAV_MAX_EXPANSIONS)
    steps = 0

    neighbors = [
        (-1, -1, math.sqrt(2)), (0, -1, 1.0), (1, -1, math.sqrt(2)),
        (-1, 0, 1.0),                      (1, 0, 1.0),
        (-1, 1, math.sqrt(2)),  (0, 1, 1.0), (1, 1, math.sqrt(2)),
    ]

    while open_set:
        _, current = heapq.heappop(open_set)
        if closed[current]:
            continue
        closed[current] = 1
        steps += 1
        if steps > max_steps:
            return []
        if current == end_idx:
            break

        cx = current % width
        cy = current // width
        cur_pt = _grid_to_point(cx, cy, nav)
        for dx, dy, cost in neighbors:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            nidx = ny * width + nx
            if closed[nidx]:
                continue
            if not grid[nidx]:
                continue
            nxt_pt = _grid_to_point(nx, ny, nav)
            if not _segment_clear(cur_pt, nxt_pt, outer, holes):
                continue
            tentative = g_score[current] + cost
            if tentative < g_score[nidx]:
                came_from[nidx] = current
                g_score[nidx] = tentative
                f_score = tentative + heuristic((nx, ny), end_cell)
                heapq.heappush(open_set, (f_score, nidx))

    if end_idx != start_idx and came_from[end_idx] < 0:
        return []

    path_cells = [end_idx]
    guard = 0
    while path_cells[-1] != start_idx:
        cur = path_cells[-1]
        nxt = came_from[cur]
        if nxt < 0:
            return []
        path_cells.append(nxt)
        guard += 1
        if guard > total:
            return []
    path_cells.reverse()

    path_pts = [_grid_to_point(idx % width, idx // width, nav) for idx in path_cells]
    # Smooth: greedy line-of-sight shortcutting.
    smoothed: List[Point] = []
    i = 0
    while i < len(path_pts):
        smoothed.append(path_pts[i])
        j = len(path_pts) - 1
        while j > i + 1:
            if _segment_clear(path_pts[i], path_pts[j], outer, holes):
                break
            j -= 1
        i = j
    if smoothed and smoothed[-1] != end:
        smoothed.append(end)
    return smoothed


def _prune_loops(route: List[Dict[str, float]], tol: float, min_length: float) -> List[Dict[str, float]]:
    if len(route) < 4:
        return route

    def dist(a, b) -> float:
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    changed = True
    out = list(route)
    while changed:
        changed = False
        cell = {}
        acc = [0.0]
        for i in range(1, len(out)):
            acc.append(acc[-1] + dist(out[i - 1], out[i]))
        for i, p in enumerate(out):
            key = (int(p["x"] / tol), int(p["y"] / tol))
            if key in cell:
                j = cell[key]
                if i - j > 2 and (acc[i] - acc[j]) > min_length and dist(out[i], out[j]) < tol:
                    out = out[: j + 1] + out[i:]
                    changed = True
                    break
            else:
                cell[key] = i
    return out


def _smooth_route(
    route: List[Dict[str, float]],
    outer: List[Point],
    holes: List[List[Point]],
    passes: int = 2,
) -> List[Dict[str, float]]:
    if len(route) < 3:
        return route

    def clear(a, b) -> bool:
        return _segment_clear(a, b, outer, holes)

    pts = [(p["x"], p["y"]) for p in route]
    for _ in range(passes):
        smoothed = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if clear(pts[i], pts[j]):
                    break
                j -= 1
            smoothed.append(pts[j])
            i = j
        pts = smoothed

    return [{"x": p[0], "y": p[1]} for p in pts]


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _route_cumulative_lengths(route: List[Point]) -> List[float]:
    if not route:
        return []
    out = [0.0]
    total = 0.0
    for i in range(1, len(route)):
        total += math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
        out.append(total)
    return out


def _route_point_at_s(route: List[Point], route_s: List[float], s_query: float) -> Point:
    if not route:
        return (0.0, 0.0)
    if len(route) == 1 or len(route_s) != len(route):
        return route[-1]

    total = route_s[-1]
    if total <= 1e-6:
        return route[-1]

    s = max(0.0, min(total, s_query))
    for i in range(1, len(route)):
        s0 = route_s[i - 1]
        s1 = route_s[i]
        seg_len = s1 - s0
        if seg_len <= 1e-9:
            continue
        if s <= s1 or i == len(route) - 1:
            t = (s - s0) / seg_len
            ax, ay = route[i - 1]
            bx, by = route[i]
            return (ax + (bx - ax) * t, ay + (by - ay) * t)
    return route[-1]


def _pure_pursuit_target(
    route: List[Point],
    route_s: List[float],
    pose_xy: Point,
    lookahead: float,
    prev_s: float,
    search_window: float,
    backtrack: float,
    goal_tolerance: float,
) -> tuple[Point, bool, float]:
    if len(route) < 2 or len(route_s) != len(route):
        return (route[-1] if route else (pose_xy[0], pose_xy[1])), True, 0.0

    total = route_s[-1]
    if total <= 1e-6:
        return route[-1], True, total

    px, py = pose_xy
    lo = max(0.0, prev_s - max(0.0, backtrack))
    hi = min(total, prev_s + max(search_window, lookahead * 2.5, 1.0))
    if hi <= lo + 1e-6:
        hi = min(total, lo + max(lookahead, 0.5))

    best_dist = float("inf")
    best_s = lo

    for i in range(1, len(route)):
        s0 = route_s[i - 1]
        s1 = route_s[i]
        if s1 < lo or s0 > hi:
            continue
        ax, ay = route[i - 1]
        bx, by = route[i]
        dx = bx - ax
        dy = by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-12:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
        t = max(0.0, min(1.0, t))
        proj_s = s0 + t * (s1 - s0)
        if proj_s < lo or proj_s > hi:
            continue
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        d = math.hypot(px - proj_x, py - proj_y)
        if d < best_dist:
            best_dist = d
            best_s = proj_s

    progress_s = min(total, max(lo, best_s))
    target_s = min(total, progress_s + max(lookahead, 0.1))
    target = _route_point_at_s(route, route_s, target_s)

    dist_to_goal = math.hypot(route[-1][0] - px, route[-1][1] - py)
    done = (
        progress_s >= (total - max(lookahead * 0.5, 0.12))
        and dist_to_goal <= max(goal_tolerance, lookahead * 0.5)
    )
    return target, done, progress_s


class TelemetryWorker:
    def __init__(
        self,
        port: str,
        baud: int,
        shared: SharedState,
        stop_event: threading.Event,
        publish_rate_hz: float,
        subscribe_rate_hz: float,
        telemetry_poll_rate_hz: float,
    ):
        self.port = port
        self.baud = baud
        self.shared = shared
        self.stop_event = stop_event
        self.publish_rate_hz = publish_rate_hz
        self.subscribe_rate_hz = subscribe_rate_hz
        self.telemetry_poll_rate_hz = telemetry_poll_rate_hz
        self.reset_event = threading.Event()
        self.thread = threading.Thread(target=self._run_thread, daemon=True, name="odometry-worker")
        self._cmd_lock = threading.Lock()
        self._cmd_queue: List[Dict[str, Any]] = []
        self._cmd_queue_max = 200
        self._recorder: Optional[PerimeterRecorder] = None
        self._outer = None
        self._holes: List[Any] = []
        self._rec_stage = "outer"
        self._planner = CoveragePlanner()
        self._sandbox_lock = threading.Lock()
        self._sandbox_enabled = False
        self._sim_pose = Pose()
        self._sim_v = 0.0
        self._sim_w = 0.0
        self._sim_total_distance = 0.0
        self._sim_update_count = 0
        self._exec_active = False
        self._exec_route: List[Point] = []
        self._exec_route_s: List[float] = []
        self._exec_progress_s = 0.0
        self._exec_allow_hw = False
        self._exec_speed = 0.4
        self._exec_max_w = 1.2
        self._exec_lookahead = 0.8
        self._exec_progress_window = 6.0
        self._exec_progress_backtrack = 0.20
        self._exec_goal_tolerance = 0.25
        self._exec_turn_in_place = False
        self._exec_turn_start_angle = math.radians(100.0)
        self._exec_turn_resume_angle = math.radians(40.0)
        self._exec_pivot_kp = 1.4
        self._exec_pivot_kd = 0.35
        self._exec_pivot_max_scale = 0.6
        self._exec_pivot_min_scale = 0.28
        self._exec_pivot_stall_omega = 0.10
        self._exec_pivot_stall_boost_scale = 0.85
        self._exec_turn_exit_slack = math.radians(6.0)
        self._exec_track_kp = 0.95
        self._exec_track_kd = 0.28
        self._exec_w_filter_tau = 0.45
        self._exec_heading_deadband = math.radians(4.5)
        self._exec_min_forward_speed = 0.12
        self._exec_min_forward_scale = 0.24
        self._exec_turn_slow_angle = math.radians(95.0)
        self._exec_track_min_w_scale = 0.08
        self._exec_track_min_w_angle = math.radians(16.0)
        self._exec_lookahead_speed_gain = 0.40
        self._exec_lateral_accel_max = 0.65
        self._exec_w_filtered = 0.0
        self._exec_last_theta: Optional[float] = None
        self._exec_last_time: Optional[float] = None

    def start(self) -> None:
        self.thread.start()

    def request_reset(self) -> None:
        self.reset_event.set()

    def enqueue(self, cmd: Dict[str, Any]) -> None:
        with self._cmd_lock:
            if cmd.get("type") == "vel":
                # Keep only the latest velocity command to avoid unbounded growth.
                replaced = False
                for i in range(len(self._cmd_queue) - 1, -1, -1):
                    if self._cmd_queue[i].get("type") == "vel":
                        self._cmd_queue[i] = cmd
                        replaced = True
                        break
                if not replaced:
                    self._cmd_queue.append(cmd)
            else:
                self._cmd_queue.append(cmd)
            if len(self._cmd_queue) > self._cmd_queue_max:
                self._cmd_queue = self._cmd_queue[-self._cmd_queue_max :]

    def _drain_cmds(self) -> List[Dict[str, Any]]:
        with self._cmd_lock:
            cmds = list(self._cmd_queue)
            self._cmd_queue.clear()
        return cmds

    def set_sandbox(self, enabled: bool) -> None:
        with self._sandbox_lock:
            self._sandbox_enabled = enabled

    def is_sandbox(self) -> bool:
        with self._sandbox_lock:
            return self._sandbox_enabled

    def _capture_completed_perimeter(self) -> None:
        if self._recorder is None:
            return
        if self._recorder.get_state() != RecorderState.DONE:
            return
        perimeter = self._recorder.get_perimeter()
        if perimeter is not None:
            if self._rec_stage == "outer":
                self._outer = perimeter
                self._rec_stage = "holes"
            else:
                self._holes.append(perimeter)
        self._recorder.reset()

    def _recording_snapshot(self) -> tuple[str, str, List[Dict[str, float]]]:
        if self._recorder is None:
            return ("idle", self._rec_stage, [])
        rec_state = self._recorder.get_state().value
        recording_path = []
        if rec_state == RecorderState.RECORDING.value:
            recording_path = _waypoints_to_points(self._recorder.get_waypoints())
        return (rec_state, self._rec_stage, recording_path)

    def _run_thread(self) -> None:
        self.shared.set_worker_alive(True)
        try:
            asyncio.run(self._run())
        except Exception as exc:
            self.shared.set_connection(False, f"telemetry worker crashed: {exc}")
        finally:
            self.shared.set_worker_alive(False)

    async def _run(self) -> None:
        while not self.stop_event.is_set():
            if self.is_sandbox():
                await self._run_sandbox()
            else:
                await self._run_hardware()

    async def _run_hardware(self) -> None:
        control: Optional[ControlLoop] = None
        try:
            self.shared.set_connection(False, "connecting")
            transport = AsyncSerialTransport(
                port=self.port,
                baud=self.baud,
                timeout=0.05,
                read_mode="auto",
                use_rx_thread=(platform.system() != "Windows"),
            )
            control = ControlLoop(
                transport,
                publish_rate_hz=self.publish_rate_hz,
                subscribe_rate_hz=self.subscribe_rate_hz,
                min_send_interval_ms=5.0,
                telemetry_poll_rate_hz=self.telemetry_poll_rate_hz,
                enable_odometry=True,
            )
            await control.start()
            ok = await control.initialize()
            if not ok:
                raise RuntimeError("ControlLoop initialize() failed")

            control.set_mode("MANUAL")
            self._recorder = PerimeterRecorder(control_loop=control)
            self._outer = None
            self._holes = []
            self._rec_stage = "outer"
            self.shared.set_connection(True, None)
            self.shared.set_sandbox(False)

            while not self.stop_event.is_set():
                if self.is_sandbox():
                    break
                self._process_commands(control)
                self._handle_reset(control)
                self._tick_recorder()

                pose = control.get_pose()
                stats = control.get_odometry_stats()
                self.shared.update(pose=pose, stats=stats, connected=True, error=None)
                self._publish_perimeter_snapshot()
                if pose is not None:
                    self._exec_tick(pose, control)
                await asyncio.sleep(1.0 / 15.0)

        except Exception as exc:
            self.shared.set_connection(False, str(exc))
            await asyncio.sleep(0.8)
        finally:
            if control is not None:
                try:
                    await control.stop()
                except Exception:
                    pass

    async def _run_sandbox(self) -> None:
        self.shared.set_connection(True, None)
        self.shared.set_sandbox(True)
        self._recorder = PerimeterRecorder(control_loop=self)
        self._outer = None
        self._holes = []
        self._rec_stage = "outer"
        self._sim_pose = Pose()
        self._sim_v = 0.0
        self._sim_w = 0.0
        self._sim_total_distance = 0.0
        self._sim_update_count = 0
        last_time = time.time()

        while not self.stop_event.is_set() and self.is_sandbox():
            now = time.time()
            dt = max(0.0, min(now - last_time, 0.2))
            last_time = now

            self._process_commands(None)
            if self.reset_event.is_set():
                self.reset_odometry(0.0, 0.0, 0.0)
                self.shared.clear_path()
                self.reset_event.clear()

            if self._recorder is not None and self._recorder.is_recording():
                self._recorder.update()
                self._capture_completed_perimeter()

            # Simple kinematic update.
            dx = self._sim_v * math.cos(self._sim_pose.theta) * dt
            dy = self._sim_v * math.sin(self._sim_pose.theta) * dt
            dtheta = self._sim_w * dt
            self._sim_pose = Pose(
                x=self._sim_pose.x + dx,
                y=self._sim_pose.y + dy,
                theta=self._sim_pose.theta + dtheta,
            )
            self._sim_total_distance += math.hypot(dx, dy)
            self._sim_update_count += 1

            stats = {
                "total_distance_m": self._sim_total_distance,
                "update_count": self._sim_update_count,
                "meters_per_tick": 0.0,
                "last_update_time": now,
            }
            self.shared.update(pose=self._sim_pose, stats=stats, connected=True, error=None)
            self._publish_perimeter_snapshot()
            self._exec_tick(self._sim_pose, None)
            await asyncio.sleep(1.0 / 20.0)

        self.shared.set_sandbox(False)

    def _process_commands(self, control: Optional[ControlLoop]) -> None:
        for cmd in self._drain_cmds():
            ctype = cmd.get("type")
            if ctype == "vel":
                v = float(cmd.get("v", 0.0))
                w = float(cmd.get("w", 0.0))
                if control is not None:
                    control.set_velocity(v, w)
                else:
                    self._sim_v = v
                    self._sim_w = w
            elif ctype == "stop":
                if control is not None:
                    control.stop_motion()
                else:
                    self._sim_v = 0.0
                    self._sim_w = 0.0
            elif ctype == "mode_manual":
                if control is not None:
                    control.set_mode("MANUAL")
            elif ctype == "reset_faults":
                if control is not None:
                    control.reset_faults()
            elif ctype == "rec_start_outer":
                if self._recorder is not None:
                    self._rec_stage = "outer"
                    self._recorder.start_recording(reset_odometry=True)
            elif ctype == "rec_start_hole":
                if self._recorder is not None:
                    if self._outer is not None:
                        self._rec_stage = "holes"
                    self._recorder.start_recording(reset_odometry=False)
            elif ctype == "rec_stop":
                if self._recorder is not None:
                    self._recorder.stop_recording()
                    self._capture_completed_perimeter()
            elif ctype == "rec_clear":
                self._outer = None
                self._holes = []
                self._rec_stage = "outer"
                self._exec_route = []
                self._exec_route_s = []
                self._exec_progress_s = 0.0
                self._exec_active = False
                self._exec_turn_in_place = False
                self._exec_w_filtered = 0.0
                self._exec_last_theta = None
                self._exec_last_time = None
                if self._recorder is not None:
                    if self._recorder.is_recording():
                        self._recorder.stop_recording()
                    if self._recorder.get_state() != RecorderState.IDLE:
                        self._recorder.reset()
                self.shared.clear_coverage()
            elif ctype == "plan":
                spacing = float(cmd.get("line_spacing_m", 0.45))
                angle_deg = float(cmd.get("sweep_angle_deg", 0.0))
                pattern = str(cmd.get("pattern", "scanline")).strip().lower()
                if pattern not in ("scanline", "hybrid", "contour"):
                    pattern = "scanline"
                try:
                    edge_passes = int(cmd.get("edge_passes", 2))
                except (TypeError, ValueError):
                    edge_passes = 2
                edge_passes = max(0, min(edge_passes, 12))
                if self._outer is not None:
                    try:
                        planner = CoveragePlanner(PlannerConfig(
                            line_spacing_m=spacing,
                            sweep_angle_rad=math.radians(angle_deg),
                            pattern=pattern,
                            edge_passes=edge_passes,
                        ))
                        outer_poly = [(wp.x, wp.y) for wp in self._outer.waypoints] if self._outer is not None else None
                        if outer_poly:
                            outer_poly = _filter_finite(_dedupe_poly(outer_poly))
                            if len(outer_poly) > 1 and outer_poly[0] == outer_poly[-1]:
                                outer_poly = outer_poly[:-1]

                        hole_polys = [
                            [(wp.x, wp.y) for wp in h.waypoints]
                            for h in self._holes
                            if h is not None
                        ]
                        cleaned_holes = []
                        for hp in hole_polys:
                            hp = _filter_finite(_dedupe_poly(hp))
                            if hp and len(hp) > 1 and hp[0] == hp[-1]:
                                hp = hp[:-1]
                            cleaned_holes.append(hp)
                        hole_polys = cleaned_holes

                        if not outer_poly or len(outer_poly) < 3:
                            snap = self.shared.snapshot()
                            path_pts = snap.get("path", [])
                            if len(path_pts) >= 3:
                                outer_poly = [(p["x"], p["y"]) for p in path_pts]
                                outer_poly = _filter_finite(_dedupe_poly(outer_poly))
                                if len(outer_poly) > 1 and outer_poly[0] == outer_poly[-1]:
                                    outer_poly = outer_poly[:-1]
                            if not outer_poly or len(outer_poly) < 3:
                                self.shared.set_coverage(
                                    passes=[],
                                    line_spacing=spacing,
                                    sweep_angle=math.radians(angle_deg),
                                    route=[],
                                    error="outer perimeter invalid (too few points)",
                                    pattern=pattern,
                                    edge_passes=edge_passes,
                                )
                                self._exec_route = []
                                self._exec_route_s = []
                                self._exec_progress_s = 0.0
                                continue

                        # Downsample overly dense perimeters to avoid planner blowups.
                        max_pts = 1500
                        if len(outer_poly) > max_pts:
                            stride = max(2, int(math.ceil(len(outer_poly) / max_pts)))
                            outer_poly = outer_poly[::stride]

                        min_x, min_y, max_x, max_y = _poly_bounds(outer_poly)
                        span_x = max_x - min_x
                        span_y = max_y - min_y
                        max_span = max(span_x, span_y)
                        max_coord = max(abs(min_x), abs(max_x), abs(min_y), abs(max_y))
                        print(
                            f"[plan] pts={len(outer_poly)} span={max_span:.2f} "
                            f"coord={max_coord:.2f} spacing={spacing:.2f}"
                        )
                        if max_coord > 10_000 or max_span > 500:
                            self.shared.set_coverage(
                                passes=[],
                                line_spacing=spacing,
                                sweep_angle=math.radians(angle_deg),
                                route=[],
                                error=f"perimeter too large for planner (span={max_span:.1f}m, coord={max_coord:.1f}m)",
                                pattern=pattern,
                                edge_passes=edge_passes,
                            )
                            self._exec_route = []
                            self._exec_route_s = []
                            self._exec_progress_s = 0.0
                            continue
                        if spacing <= 0:
                            self.shared.set_coverage(
                                passes=[],
                                line_spacing=spacing,
                                sweep_angle=math.radians(angle_deg),
                                route=[],
                                error="line spacing must be > 0",
                                pattern=pattern,
                                edge_passes=edge_passes,
                            )
                            self._exec_route = []
                            self._exec_route_s = []
                            self._exec_progress_s = 0.0
                            continue

                        est_lines = int(math.ceil(max_span / spacing)) + 1
                        if est_lines > 12000:
                            self.shared.set_coverage(
                                passes=[],
                                line_spacing=spacing,
                                sweep_angle=math.radians(angle_deg),
                                route=[],
                                error=f"planner guard: estimated scanlines too high ({est_lines})",
                                pattern=pattern,
                                edge_passes=edge_passes,
                            )
                            self._exec_route = []
                            self._exec_route_s = []
                            self._exec_progress_s = 0.0
                            continue

                        diag_area = _polygon_area(outer_poly) if outer_poly else 0.0
                        diag_self = _self_intersections(outer_poly) if outer_poly and len(outer_poly) <= 800 else -1
                        diag_pts = len(outer_poly) if outer_poly else 0

                        if diag_area < (spacing * spacing * 0.2):
                            self.shared.set_coverage(
                                passes=[],
                                line_spacing=spacing,
                                sweep_angle=math.radians(angle_deg),
                                route=[],
                                error=f"perimeter area too small for spacing (area={diag_area:.3f})",
                                pattern=pattern,
                                edge_passes=edge_passes,
                            )
                            self._exec_route = []
                            self._exec_route_s = []
                            self._exec_progress_s = 0.0
                            continue

                        simplify_tol = max(0.05, spacing * 0.25)
                        loop_tol = max(0.45, spacing * 0.8)
                        loop_min = max(0.9, spacing * 2.0)

                        def make_perimeter(poly: List[Point]):
                            from sban.perimeter.perimeter import Perimeter, Waypoint
                            wps = [
                                Waypoint(x=x, y=y, heading=0.0, distance=0.0, timestamp=time.time())
                                for x, y in poly
                            ]
                            return Perimeter(
                                waypoints=wps,
                                created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
                                sample_distance_m=self._outer.sample_distance_m,
                                total_distance_m=self._outer.total_distance_m,
                                loop_closed=self._outer.loop_closed,
                                closure_error_m=self._outer.closure_error_m,
                                metadata=dict(self._outer.metadata),
                            )

                        candidates: List[Tuple[str, List[Point]]] = []

                        def add_candidate(label: str, poly: Optional[List[Point]]) -> None:
                            if not poly:
                                return
                            poly = _dedupe_poly(poly)
                            if len(poly) > 1 and poly[0] == poly[-1]:
                                poly = poly[:-1]
                            if len(poly) < 3:
                                return
                            if candidates and poly == candidates[-1][1]:
                                return
                            candidates.append((label, poly))

                        add_candidate("raw", outer_poly)
                        pruned: Optional[List[Point]] = None
                        if outer_poly:
                            pruned = _prune_poly_loops(outer_poly, tol=loop_tol, min_length=loop_min)
                            if pruned != outer_poly:
                                add_candidate("pruned", pruned)

                        if outer_poly and (diag_self > 0 or len(outer_poly) > 250):
                            simp = _simplify_poly(outer_poly, simplify_tol)
                            add_candidate("simplified", simp)
                            if pruned is not None and pruned != outer_poly:
                                simp2 = _simplify_poly(pruned, simplify_tol)
                                add_candidate("pruned+simplified", simp2)

                        if outer_poly:
                            hull = _convex_hull(outer_poly)
                            add_candidate("hull", hull)

                        coverage = None
                        passes: List[List[Dict[str, float]]] = []
                        used = "raw"
                        outer_poly_used = outer_poly
                        for label, poly in candidates:
                            coverage_try = planner.plan(make_perimeter(poly), holes=self._holes)
                            passes_try = _coverage_to_passes(coverage_try)
                            if passes_try:
                                coverage = coverage_try
                                passes = passes_try
                                used = label
                                outer_poly_used = poly
                                break
                            if coverage is None:
                                coverage = coverage_try
                                outer_poly_used = poly
                                used = label

                        start_pose = control.get_pose() if control is not None else self._sim_pose
                        nav = None
                        if ENABLE_NAV and outer_poly_used:
                            nav = _build_nav_grid(
                                outer_poly_used,
                                hole_polys,
                                resolution=max(NAV_MIN_RESOLUTION, spacing),
                                max_cells=NAV_MAX_CELLS,
                            )
                        route = _build_route_points(
                            coverage,
                            start_pose,
                            turn_offset=spacing / 2.0,
                            outer_poly=outer_poly_used,
                            hole_polys=hole_polys,
                            nav=nav,
                        )
                        audit = _coverage_audit(coverage, outer_poly_used, hole_polys, spacing)
                        tried = ",".join(label for label, _ in candidates) if candidates else "none"
                        error = None
                        if not passes:
                            error = (
                                f"no passes (raw_pts={diag_pts}, area={diag_area:.2f}, "
                                f"span={max_span:.2f}, self_x={diag_self}, tried={tried})"
                            )
                        self.shared.set_coverage(
                            passes=passes,
                            line_spacing=spacing,
                            sweep_angle=math.radians(angle_deg),
                            route=route,
                            error=error,
                            pattern=pattern,
                            edge_passes=edge_passes,
                            audit=audit,
                        )
                        self._exec_route = [(p["x"], p["y"]) for p in route]
                        self._exec_route_s = _route_cumulative_lengths(self._exec_route)
                        self._exec_progress_s = 0.0
                    except Exception as exc:
                        self.shared.set_coverage(
                            passes=[],
                            line_spacing=spacing,
                            sweep_angle=math.radians(angle_deg),
                            route=[],
                            error=str(exc),
                            pattern=pattern,
                            edge_passes=edge_passes,
                        )
                        self._exec_route = []
                        self._exec_route_s = []
                        self._exec_progress_s = 0.0
                else:
                    self.shared.set_coverage(
                        passes=[],
                        line_spacing=spacing,
                        sweep_angle=math.radians(angle_deg),
                        route=[],
                        error="no outer perimeter",
                        pattern=pattern,
                        edge_passes=edge_passes,
                    )
                    self._exec_route = []
                    self._exec_route_s = []
                    self._exec_progress_s = 0.0
            elif ctype == "exec_start":
                if self._exec_route and self._exec_route_s:
                    self._exec_active = True
                    self._exec_turn_in_place = False
                    self._exec_w_filtered = 0.0
                    self._exec_last_theta = None
                    self._exec_last_time = None
                    self._exec_progress_s = 0.0
            elif ctype == "exec_stop":
                self._exec_active = False
                self._exec_turn_in_place = False
                self._exec_w_filtered = 0.0
                self._exec_last_theta = None
                self._exec_last_time = None
                self._exec_progress_s = 0.0
                if control is not None:
                    control.stop_motion()
                else:
                    self._sim_v = 0.0
                    self._sim_w = 0.0
            elif ctype == "exec_config":
                speed = cmd.get("speed")
                max_w = cmd.get("max_w")
                if speed is not None:
                    try:
                        self._exec_speed = max(0.0, min(1.35, float(speed)))
                    except (TypeError, ValueError):
                        pass
                if max_w is not None:
                    try:
                        self._exec_max_w = max(0.0, min(3.14, float(max_w)))
                    except (TypeError, ValueError):
                        pass
            elif ctype == "exec_allow":
                self._exec_allow_hw = bool(cmd.get("allow", False))

    def _handle_reset(self, control: ControlLoop) -> None:
        if self.reset_event.is_set():
            control.reset_odometry(0.0, 0.0, 0.0)
            self.shared.clear_path()
            self.reset_event.clear()

    def _exec_tick(self, pose: Pose, control: Optional[ControlLoop]) -> None:
        if (
            not self._exec_active
            or not self._exec_route
            or not self._exec_route_s
            or len(self._exec_route_s) != len(self._exec_route)
            or pose is None
        ):
            return
        if control is not None and not self._exec_allow_hw:
            return

        now = time.time()
        omega_est = 0.0
        dt_s = 1.0 / 15.0
        if self._exec_last_theta is not None and self._exec_last_time is not None:
            dt = now - self._exec_last_time
            if dt > 1e-3:
                dt_s = dt
                dtheta = _normalize_angle(pose.theta - self._exec_last_theta)
                omega_est = dtheta / dt
        self._exec_last_theta = pose.theta
        self._exec_last_time = now

        lookahead = max(
            0.35,
            min(2.2, self._exec_lookahead + self._exec_lookahead_speed_gain * max(0.0, self._exec_speed)),
        )
        target, done, progress_s = _pure_pursuit_target(
            self._exec_route,
            self._exec_route_s,
            (pose.x, pose.y),
            lookahead,
            self._exec_progress_s,
            search_window=self._exec_progress_window,
            backtrack=self._exec_progress_backtrack,
            goal_tolerance=self._exec_goal_tolerance,
        )
        self._exec_progress_s = progress_s
        if done:
            self._exec_active = False
            self._exec_turn_in_place = False
            self._exec_w_filtered = 0.0
            self._exec_last_theta = None
            self._exec_last_time = None
            self._exec_progress_s = 0.0
            if control is not None:
                control.stop_motion()
            else:
                self._sim_v = 0.0
                self._sim_w = 0.0
            return

        dx = target[0] - pose.x
        dy = target[1] - pose.y
        heading = math.atan2(dy, dx)
        alpha = _normalize_angle(heading - pose.theta)
        abs_alpha = abs(alpha)
        was_pivot = self._exec_turn_in_place

        # Differential-drive corner behavior:
        # if heading error is very large, pivot in place using opposite wheel motion.
        if self._exec_turn_in_place:
            if abs_alpha <= self._exec_turn_resume_angle:
                self._exec_turn_in_place = False
            elif (
                abs_alpha <= (self._exec_turn_resume_angle + self._exec_turn_exit_slack)
                and abs(omega_est) < self._exec_pivot_stall_omega
            ):
                # If we are very close to the exit threshold but no longer rotating,
                # release pivot mode so the follower can continue advancing.
                self._exec_turn_in_place = False
        elif abs_alpha >= self._exec_turn_start_angle:
            self._exec_turn_in_place = True
        if was_pivot != self._exec_turn_in_place:
            self._exec_w_filtered = 0.0

        if self._exec_turn_in_place:
            v = 0.0
            pivot_max_w = max(0.0, self._exec_max_w * self._exec_pivot_max_scale)
            pivot_min_w = min(pivot_max_w, max(0.0, self._exec_max_w * self._exec_pivot_min_scale))
            w_cmd = (self._exec_pivot_kp * alpha) - (self._exec_pivot_kd * omega_est)
            if abs(w_cmd) < pivot_min_w and abs_alpha > self._exec_turn_resume_angle:
                w_cmd = math.copysign(pivot_min_w, alpha)
            if (
                abs(omega_est) < self._exec_pivot_stall_omega
                and abs_alpha > (self._exec_turn_resume_angle + self._exec_turn_exit_slack)
            ):
                stall_kick = max(pivot_min_w, pivot_max_w * self._exec_pivot_stall_boost_scale)
                w_cmd = math.copysign(max(abs(w_cmd), stall_kick), alpha)
            w = max(-pivot_max_w, min(pivot_max_w, w_cmd))
            self._exec_w_filtered = w
        else:
            base_v = self._exec_speed
            slow = max(self._exec_min_forward_scale, 1.0 - min(abs_alpha / self._exec_turn_slow_angle, 1.0))
            v = base_v * slow
            if base_v > 1e-4:
                v = max(v, min(base_v, self._exec_min_forward_speed))
            curvature = abs(math.sin(alpha)) / max(lookahead, 0.25)
            if self._exec_lateral_accel_max > 1e-4 and curvature > 1e-4:
                v_limit = math.sqrt(self._exec_lateral_accel_max / curvature)
                v = min(v, v_limit)
            w_cmd = (
                self._exec_track_kp * max(v, 0.1) * math.sin(alpha) / max(lookahead, 0.3)
            ) - (self._exec_track_kd * omega_est)
            if abs_alpha < self._exec_heading_deadband and abs(omega_est) < 0.08:
                w_cmd = 0.0
            elif abs_alpha > self._exec_track_min_w_angle:
                min_track_w = min(self._exec_max_w, max(0.10, self._exec_max_w * self._exec_track_min_w_scale))
                if abs(w_cmd) < min_track_w:
                    w_cmd = math.copysign(min_track_w, alpha)
            w_cmd = max(
                -self._exec_max_w,
                min(self._exec_max_w, w_cmd),
            )
            tau = max(0.02, self._exec_w_filter_tau)
            beta = max(0.0, min(1.0, dt_s / (tau + dt_s)))
            self._exec_w_filtered += beta * (w_cmd - self._exec_w_filtered)
            w = self._exec_w_filtered

        if control is not None:
            control.set_velocity(v, w)
        else:
            self._sim_v = v
            self._sim_w = w

    def _tick_recorder(self) -> None:
        if self._recorder is not None and self._recorder.is_recording():
            self._recorder.update()
            self._capture_completed_perimeter()

    def _publish_perimeter_snapshot(self) -> None:
        outer_pts = _perimeter_to_points(self._outer)
        hole_pts = [_perimeter_to_points(h) for h in self._holes if h is not None]
        rec_state, rec_stage, recording_path = self._recording_snapshot()
        self.shared.set_perimeters(
            outer=outer_pts,
            holes=[h for h in hole_pts if h is not None],
            rec_state=rec_state,
            rec_stage=rec_stage,
            recording_path=recording_path,
        )

    # SimControlLoop compatibility for PerimeterRecorder.
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self._sim_pose = Pose(x=x, y=y, theta=theta)
        self._sim_total_distance = 0.0
        self._sim_update_count = 0

    def get_pose(self) -> Pose:
        return self._sim_pose


def make_handler(shared: SharedState, worker: TelemetryWorker, stop_event: threading.Event):
    class Handler(BaseHTTPRequestHandler):
        def _read_json(self) -> Dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0:
                return {}
            body = self.rfile.read(length)
            try:
                return json.loads(body.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                return {}

        def _send_text(self, code: int, text: str = "") -> None:
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(text.encode("utf-8"))))
            self.end_headers()
            if text:
                self.wfile.write(text.encode("utf-8"))

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                content = HTML_PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if self.path == "/health":
                payload = json.dumps(shared.snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return

            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()

                try:
                    while not stop_event.is_set():
                        payload = json.dumps(shared.snapshot_lite(), separators=(",", ":"))
                        msg = f"data: {payload}\n\n".encode("utf-8")
                        self.wfile.write(msg)
                        self.wfile.flush()
                        time.sleep(1.0 / 15.0)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

            if self.path == "/export":
                snap = shared.snapshot()
                payload = {
                    "version": "1.0",
                    "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "perimeter": snap.get("perimeter", {}),
                    "coverage": snap.get("coverage", {}),
                    "plan": snap.get("plan", {}),
                }
                data = json.dumps(payload, indent=2).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Content-Disposition", "attachment; filename=route.json")
                self.end_headers()
                self.wfile.write(data)
                return

            self.send_error(404, "Not Found")

        def do_POST(self) -> None:
            if self.path == "/reset":
                worker.request_reset()
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/cmd/vel":
                data = self._read_json()
                worker.enqueue({"type": "vel", "v": data.get("v", 0.0), "w": data.get("w", 0.0)})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/cmd/stop":
                worker.enqueue({"type": "stop"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/mode/manual":
                worker.enqueue({"type": "mode_manual"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/cmd/reset_faults":
                worker.enqueue({"type": "reset_faults"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/rec/start_outer":
                worker.enqueue({"type": "rec_start_outer"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/rec/start_hole":
                worker.enqueue({"type": "rec_start_hole"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/rec/stop":
                worker.enqueue({"type": "rec_stop"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/rec/clear":
                worker.enqueue({"type": "rec_clear"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/plan":
                snap = shared.snapshot()
                if not snap.get("connected") and not snap.get("sandbox"):
                    self._send_text(409, "Not connected. Enable Sandbox mode.")
                    return
                if not snap.get("perimeter", {}).get("outer"):
                    self._send_text(409, "Record an outer perimeter first.")
                    return
                data = self._read_json()
                worker.enqueue({
                    "type": "plan",
                    "line_spacing_m": data.get("line_spacing_m", 0.6),
                    "sweep_angle_deg": data.get("sweep_angle_deg", 0.0),
                    "pattern": data.get("pattern", "scanline"),
                    "edge_passes": data.get("edge_passes", 2),
                })
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/sandbox":
                data = self._read_json()
                enabled = bool(data.get("enabled", False))
                worker.set_sandbox(enabled)
                shared.set_sandbox(enabled)
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/exec/start":
                data = self._read_json()
                allow_hw = bool(data.get("allow", False))
                if not shared.is_sandbox() and not allow_hw:
                    self._send_text(409, "Hardware execute disabled")
                    return
                worker.enqueue(
                    {
                        "type": "exec_config",
                        "speed": data.get("speed", 0.4),
                        "max_w": data.get("max_w", 1.2),
                    }
                )
                worker.enqueue({"type": "exec_allow", "allow": allow_hw})
                worker.enqueue({"type": "exec_start"})
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/exec/config":
                data = self._read_json()
                worker.enqueue(
                    {
                        "type": "exec_config",
                        "speed": data.get("speed", 0.4),
                        "max_w": data.get("max_w", 1.2),
                    }
                )
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/exec/stop":
                worker.enqueue({"type": "exec_stop"})
                self.send_response(204)
                self.end_headers()
                return
            self.send_error(404, "Not Found")

        def log_message(self, format: str, *args: Any) -> None:
            # Keep stdout focused on robot status.
            return

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SnoBot live odometry web viewer")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="Serial port (e.g., COM12)")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int, help="Serial baud rate")
    parser.add_argument("--host", default="127.0.0.1", help="Host bind address")
    parser.add_argument("--web-port", default=3000, type=int, help="Web server port")
    parser.add_argument("--publish-rate", default=15.0, type=float, help="INTENT publish rate (Hz)")
    parser.add_argument("--subscribe-rate", default=20.0, type=float, help="Host subscriber poll rate (Hz)")
    parser.add_argument("--telemetry-rate", default=20.0, type=float, help="STATUS poll rate to MCU (Hz)")
    parser.add_argument("--sandbox", action="store_true", help="Run without hardware (simulated odometry)")
    parser.add_argument(
        "--open-browser",
        dest="open_browser",
        action="store_true",
        help="Open the live page automatically",
    )
    parser.add_argument(
        "--no-open-browser",
        dest="open_browser",
        action="store_false",
        help="Do not open a browser window automatically",
    )
    parser.set_defaults(open_browser=True)
    parser.add_argument(
        "--browser-app",
        dest="browser_app",
        action="store_true",
        help="Open browser as a standalone app window when possible",
    )
    parser.add_argument(
        "--no-browser-app",
        dest="browser_app",
        action="store_false",
        help="Open browser as a normal tab/window",
    )
    parser.set_defaults(browser_app=True)
    return parser.parse_args()


def _open_standalone_browser(url: str) -> bool:
    system = platform.system()
    candidates: List[List[str]] = []

    if system == "Windows":
        edge = shutil.which("msedge")
        chrome = shutil.which("chrome")
        if edge:
            candidates.append([edge, f"--app={url}"])
        if chrome:
            candidates.append([chrome, f"--app={url}"])
        candidates.extend(
            [
                [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", f"--app={url}"],
                [r"C:\Program Files\Microsoft\Edge\Application\msedge.exe", f"--app={url}"],
                [r"C:\Program Files\Google\Chrome\Application\chrome.exe", f"--app={url}"],
                [r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe", f"--app={url}"],
            ]
        )
    elif system == "Darwin":
        candidates.append(["open", "-a", "Google Chrome", "--args", f"--app={url}"])
    else:
        for exe in ("google-chrome", "chromium-browser", "chromium", "microsoft-edge"):
            found = shutil.which(exe)
            if found:
                candidates.append([found, f"--app={url}"])

    for cmd in candidates:
        try:
            subprocess.Popen(cmd)
            return True
        except Exception:
            continue

    return False


def open_browser(url: str, app_mode: bool) -> None:
    if app_mode and _open_standalone_browser(url):
        return

    try:
        webbrowser.open(url, new=1)
    except Exception:
        pass


def main() -> int:
    args = parse_args()
    shared = SharedState()
    stop_event = threading.Event()
    worker = TelemetryWorker(
        args.port,
        args.baud,
        shared,
        stop_event,
        publish_rate_hz=args.publish_rate,
        subscribe_rate_hz=args.subscribe_rate,
        telemetry_poll_rate_hz=args.telemetry_rate,
    )
    if args.sandbox:
        worker.set_sandbox(True)
        shared.set_sandbox(True)
    worker.start()

    handler = make_handler(shared, worker, stop_event)
    server = ThreadingHTTPServer((args.host, args.web_port), handler)
    url = f"http://{args.host}:{args.web_port}"
    print(f"[odometry-web] Serving {url}")
    print(f"[odometry-web] Reading robot telemetry from {args.port} @ {args.baud} baud")
    print("[odometry-web] Press Ctrl+C to stop")
    if args.open_browser:
        open_browser(url, app_mode=args.browser_app)

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            server.shutdown()
        except Exception:
            pass
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
