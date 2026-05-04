/**
 * p5: визуализация и UI. Физика в physics.js.
 */

import {
  MU,
  MU_DEG,
  METHOD_LABELS,
  cloneSimState,
  createInitialState,
  stepByMethod,
  syncVerletPrev,
  totalEnergy,
} from "./physics.js";

const PHYS_HZ = 60;
const PHYS_DT = 1 / PHYS_HZ;
const METHODS = ["explicit", "implicit", "semi", "verlet"];
const METHOD_COLORS = {
  explicit: [255, 118, 118],
  implicit: [118, 186, 255],
  semi: [156, 255, 132],
  verlet: [255, 214, 102],
};

let canvasW = 1080;
let canvasH = 640;
const plotH = 200;
const simH = canvasH - plotH;

/** @type {Record<string, import('./physics.js').SimState>} */
let states = {};
let sceneId = "corner2";
let acc = 0;
let lastT = 0;
let paused = false;

/** @type {Record<string, number[]>} */
const energyHist = Object.fromEntries(METHODS.map((m) => [m, []]));
const E_HIST_MAX = 520;

/** @type {{ x: number; y: number }[]} */
let trail = [];

function el(id) {
  return document.getElementById(id);
}

function readUi() {
  const dtMs = +el("dt").value;
  return {
    scene: el("sceneSelect").value,
    displayMethod: el("displayMethodSelect").value,
    dt: Math.max(0.0008, Math.min(0.045, dtMs / 1000)),
    substeps: Math.max(1, Math.min(16, +el("substeps").value | 0)),
  };
}

function bindRange(id, fmt) {
  const node = el(id);
  const out = el(`${id}Val`);
  if (!node || !out) return;
  const sync = () => {
    out.textContent = fmt ? fmt(+node.value) : String(node.value);
  };
  node.addEventListener("input", sync);
  sync();
}

function resetAll() {
  const u = readUi();
  sceneId = u.scene;
  const base = createInitialState(sceneId, { canvasW, canvasH: simH });
  states = {};
  for (const m of METHODS) {
    states[m] = cloneSimState(base);
    if (m === "verlet") syncVerletPrev(states[m], u.dt / u.substeps);
  }
  for (const m of METHODS) energyHist[m].length = 0;
  trail = [];
  acc = 0;
}

function pushEnergySamples() {
  for (const m of METHODS) {
    const arr = energyHist[m];
    arr.push(totalEnergy(states[m]));
    if (arr.length > E_HIST_MAX) arr.shift();
  }
}

function recordTrail(u) {
  const s = states[u.displayMethod];
  if (s.sceneId === "twoBodies") {
    const cx = (s.x[0] + s.x[1]) * 0.5;
    const cy = (s.y[0] + s.y[1]) * 0.5;
    trail.push({ x: cx, y: cy });
  } else {
    trail.push({ x: s.x[0], y: s.y[0] });
  }
  if (trail.length > 1400) trail.shift();
}

function hasNaNState() {
  for (const m of METHODS) {
    const s = states[m];
    for (let i = 0; i < s.n; i++) {
      if (!Number.isFinite(s.x[i]) || !Number.isFinite(s.y[i])) return true;
    }
  }
  return false;
}

function drawAxesWorld(s) {
  stroke(48, 58, 76);
  strokeWeight(2);
  const O = s.origin;
  if (s.sceneId === "corner2") {
    line(O.x, O.y, canvasW, O.y);
    line(O.x, O.y, O.x + 720 * Math.cos(-MU), O.y + 720 * Math.sin(-MU));
    noStroke();
    fill(40, 48, 62, 90);
    arc(O.x, O.y, 72, 72, -MU, 0);
  } else if (s.sceneId === "corner3") {
    line(O.x, O.y, O.x, O.y - 420);
    line(O.x, O.y, canvasW, O.y);
    const a = s.anchorPoints[1];
    const wallLen = 120;
    line(
      a.x - wallLen * Math.cos(MU),
      a.y - wallLen * Math.sin(MU),
      a.x + wallLen * Math.cos(MU),
      a.y + wallLen * Math.sin(MU),
    );
  } else if (s.sceneId === "twoBodies") {
    const leftX = s.anchorPoints[0].x;
    line(leftX, 12, leftX, simH - 4);
    const a = s.anchorPoints[1];
    const wallLen = 220;
    const wx = Math.cos(MU);
    const wy = Math.sin(MU);
    line(a.x - wallLen * wx, a.y - wallLen * wy, a.x + wallLen * wx, a.y + wallLen * wy);
  } else {
    line(40, O.y, canvasW, O.y);
  }
}

function drawSpring(xa, ya, xb, yb) {
  stroke(120, 130, 150);
  strokeWeight(1.4);
  noFill();
  const segments = 16;
  beginShape();
  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    let px = xa + (xb - xa) * t;
    let py = ya + (yb - ya) * t;
    if (i > 0 && i < segments) {
      const dx = xb - xa;
      const dy = yb - ya;
      const len = Math.hypot(dx, dy) || 1;
      const nx = -dy / len;
      const ny = dx / len;
      const amp = i % 2 === 0 ? 9 : -9;
      px += nx * amp;
      py += ny * amp;
    }
    vertex(px, py);
  }
  endShape();
}

function drawSceneMain(u) {
  background(14, 18, 26);
  const s0 = states[u.displayMethod];
  drawAxesWorld(s0);

  for (const p of s0.anchorPoints) {
    fill(82, 92, 110);
    noStroke();
    circle(p.x, p.y, 10);
  }

  for (const sp of s0.springs) {
    if (sp.type === "anchor") {
      drawSpring(sp.ax, sp.ay, s0.x[sp.i], s0.y[sp.i]);
    } else {
      drawSpring(s0.x[sp.i], s0.y[sp.i], s0.x[sp.j], s0.y[sp.j]);
    }
  }

  for (const m of METHODS) {
    const s = states[m];
    const c = METHOD_COLORS[m];
    const alpha = m === u.displayMethod ? 255 : 70;
    if (s.n === 1) {
      fill(c[0], c[1], c[2], alpha);
      stroke(12, 14, 20, alpha);
      strokeWeight(m === u.displayMethod ? 2 : 1);
      circle(s.x[0], s.y[0], m === u.displayMethod ? 22 : 14);
    } else {
      for (let i = 0; i < s.n; i++) {
        fill(c[0], c[1], c[2], alpha);
        stroke(12, 14, 20, alpha);
        strokeWeight(m === u.displayMethod ? 2 : 1);
        circle(s.x[i], s.y[i], m === u.displayMethod ? 18 : 11);
      }
    }
  }

  noStroke();
  fill(230, 234, 242);
  textSize(13);
  textAlign(LEFT, TOP);
  text(`μ = ${MU_DEG}° (все подзадачи)`, 18, 14);
  text(`Сцена: ${sceneLabel(sceneId)}`, 18, 34);
  text(`Пауза: ${paused ? "да" : "нет"}`, 18, 54);
}

function sceneLabel(id) {
  if (id === "corner2") return "угол (2 пружины)";
  if (id === "corner3") return "три пружины (одна масса)";
  return "две точки (две массы, три пружины, μ)";
}

function drawPlots(u) {
  const y0 = simH + 8;
  const trajW = canvasW * 0.42;
  const eW = canvasW - trajW - 32;
  const eX = trajW + 24;
  const plotInnerH = plotH - 28;

  noStroke();
  fill(18, 22, 32);
  rect(0, simH, canvasW, plotH);

  stroke(52, 62, 82);
  strokeWeight(1);
  noFill();
  rect(10, y0, trajW - 20, plotInnerH, 6);
  rect(eX, y0, eW - 14, plotInnerH, 6);

  drawTrajectoryMini(16, y0 + 4, trajW - 32, plotInnerH - 8, u);
  drawEnergyMini(eX + 6, y0 + 4, eW - 26, plotInnerH - 8, u);
}

function drawTrajectoryMini(x, y, w, h, u) {
  if (trail.length < 2) {
    fill(140, 150, 170);
    noStroke();
    textSize(12);
    textAlign(LEFT, TOP);
    text("Траектория свободной точки", x, y + 4);
    return;
  }
  let minX = trail[0].x;
  let maxX = trail[0].x;
  let minY = trail[0].y;
  let maxY = trail[0].y;
  for (const p of trail) {
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  }
  const pad = 28;
  minX -= pad;
  maxX += pad;
  minY -= pad;
  maxY += pad;
  const sx = w / Math.max(1e-6, maxX - minX);
  const sy = h / Math.max(1e-6, maxY - minY);
  const s = Math.min(sx, sy);
  const cx = (minX + maxX) * 0.5;
  const cy = (minY + maxY) * 0.5;
  const ox = x + w * 0.5;
  const oy = y + h * 0.5;

  noFill();
  stroke(90, 110, 140, 90);
  strokeWeight(1);
  for (let i = 1; i < trail.length; i++) {
    const a = trail[i - 1];
    const b = trail[i];
    line(ox + (a.x - cx) * s, oy + (a.y - cy) * s, ox + (b.x - cx) * s, oy + (b.y - cy) * s);
  }
  stroke(METHOD_COLORS[u.displayMethod][0], METHOD_COLORS[u.displayMethod][1], METHOD_COLORS[u.displayMethod][2]);
  strokeWeight(2);
  const last = trail[trail.length - 1];
  noFill();
  circle(ox + (last.x - cx) * s, oy + (last.y - cy) * s, 6);

  fill(200, 208, 220);
  noStroke();
  textSize(11);
  textAlign(LEFT, TOP);
  text(`Траектория: ${METHOD_LABELS[u.displayMethod]}`, x, y - 2);
}

function drawEnergyMini(x, y, w, h, u) {
  const s0 = states[u.displayMethod];
  const e0 = s0.initialEnergy;
  let vmin = e0;
  let vmax = e0;
  for (const m of METHODS) {
    for (const v of energyHist[m]) {
      vmin = Math.min(vmin, v);
      vmax = Math.max(vmax, v);
    }
  }
  if (Math.abs(vmax - vmin) < 1e-6) {
    vmin -= 1;
    vmax += 1;
  }
  const pad = (vmax - vmin) * 0.08;
  vmin -= pad;
  vmax += pad;

  noStroke();
  fill(200, 208, 220);
  textSize(11);
  textAlign(LEFT, TOP);
  text("Полная энергия E(t), все методы", x, y - 2);

  for (const m of METHODS) {
    const arr = energyHist[m];
    if (arr.length < 2) continue;
    const c = METHOD_COLORS[m];
    stroke(c[0], c[1], c[2], 230);
    strokeWeight(m === u.displayMethod ? 2.2 : 1.2);
    noFill();
    beginShape();
    for (let i = 0; i < arr.length; i++) {
      const t = arr.length > 1 ? i / (arr.length - 1) : 0;
      const px = x + t * w;
      const py = y + h - ((arr[i] - vmin) / (vmax - vmin)) * h;
      vertex(px, py);
    }
    endShape();
  }

  stroke(255, 255, 255, 55);
  strokeWeight(1);
  const yRef = y + h - ((e0 - vmin) / (vmax - vmin)) * h;
  line(x, yRef, x + w, yRef);

  fill(160, 170, 190);
  noStroke();
  textSize(10);
  textAlign(RIGHT, TOP);
  text(`E₀=${e0.toFixed(2)}`, x + w, y + h - 12);
}

function formatStats(u) {
  const s = states[u.displayMethod];
  const E = totalEnergy(s);
  const drift = E - s.initialEnergy;
  return (
    `Метод отображения: ${METHOD_LABELS[u.displayMethod]}\n` +
    `E: ${E.toFixed(4)}  ΔE: ${drift.toFixed(4)}\n` +
    `Δt подшаг: ${u.dt.toFixed(4)}  подшагов: ${u.substeps}\n` +
    `Частицы: ${s.n}`
  );
}

function setupLegend() {
  const leg = el("legend");
  if (!leg) return;
  leg.innerHTML = "";
  for (const m of METHODS) {
    const row = document.createElement("div");
    row.className = "legend-row";
    const sw = document.createElement("span");
    sw.className = "swatch";
    const c = METHOD_COLORS[m];
    sw.style.background = `rgb(${c[0]},${c[1]},${c[2]})`;
    const lab = document.createElement("span");
    lab.textContent = METHOD_LABELS[m];
    row.appendChild(sw);
    row.appendChild(lab);
    leg.appendChild(row);
  }
}

function setup() {
  const cnv = createCanvas(canvasW, canvasH);
  cnv.parent("p5canvas");
  bindRange("dt", (v) => (v / 1000).toFixed(4));
  bindRange("substeps");
  el("btnReset").addEventListener("click", resetAll);
  el("sceneSelect").addEventListener("change", resetAll);
  setupLegend();
  resetAll();
  lastT = millis() / 1000;
}

function draw() {
  const now = millis() / 1000;
  let frameDt = Math.min(0.09, now - lastT);
  lastT = now;
  const u = readUi();

  if (!paused) {
    acc += frameDt;
    while (acc >= PHYS_DT) {
      const h = u.dt / u.substeps;
      for (let k = 0; k < u.substeps; k++) {
        for (const m of METHODS) {
          stepByMethod(states[m], m, h);
        }
      }
      pushEnergySamples();
      recordTrail(u);
      acc -= PHYS_DT;
    }
  }

  if (hasNaNState()) resetAll();

  drawSceneMain(u);
  drawPlots(u);

  const st = el("statsText");
  if (st) st.textContent = formatStats(u);
}

function keyPressed() {
  if (key === " ") {
    paused = !paused;
    return false;
  }
}

window.setup = setup;
window.draw = draw;
window.keyPressed = keyPressed;
