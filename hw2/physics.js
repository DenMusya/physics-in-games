/**
 * 2D деформируемая оболочка: рёбра + площадь. PBD, XPBD, Projective Dynamics, VBD.
 * Без отрисовки; указатель для захвата задаётся через sim.pointerX / sim.pointerY.
 */

const PHYS_TAU = Math.PI * 2;
const PHYS_FIXED_DT = 1 / 60;

const sim = {
  verts: [],
  edges: [],
  restArea: 0,
  groundY: 0,
  grabIndex: -1,
  grabOx: 0,
  grabOy: 0,
  pointerX: 0,
  pointerY: 0,
  paused: false,
  areaLambda: 0,
  scene: "free",
  acc: 0,
  lastT: 0,
};

function polygonSignedArea(v) {
  const n = v.length;
  let s = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    s += v[i].x * v[j].y - v[j].x * v[i].y;
  }
  return 0.5 * s;
}

function buildCircleBlob(cx, cy, r, n, scene) {
  const verts = [];
  for (let i = 0; i < n; i++) {
    const a = -Math.PI / 2 + (PHYS_TAU * i) / n;
    const x = cx + r * Math.cos(a);
    const y = cy + r * Math.sin(a);
    verts.push({
      x,
      y,
      px: x,
      py: y,
      vx: 0,
      vy: 0,
      invMass: 1,
      pinned: false,
    });
  }

  if (scene === "pinned") {
    const order = verts
      .map((p, i) => ({ i, y: p.y }))
      .sort((a, b) => a.y - b.y);
    const i0 = order[0].i;
    const i1 = order[1].i;
    verts[i0].pinned = true;
    verts[i0].invMass = 0;
    verts[i1].pinned = true;
    verts[i1].invMass = 0;
  }

  const edges = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const a = verts[i];
    const b = verts[j];
    const L = Math.hypot(b.x - a.x, b.y - a.y);
    edges.push({ i, j, rest: L, lambda: 0 });
  }

  const restArea = polygonSignedArea(verts);
  return { verts, edges, restArea };
}

function syncVerletState() {
  for (const p of sim.verts) {
    p.px = p.x;
    p.py = p.y;
    p.vx = 0;
    p.vy = 0;
  }
}

function resetSimulation(canvasW, canvasH, u) {
  sim.scene = u.scene;
  sim.groundY = canvasH - 40;
  const cx = canvasW * 0.5;
  let cy = canvasH * 0.38;
  if (u.scene === "squash") cy = sim.groundY - u.blobRadius * 0.82;

  const { verts, edges, restArea } = buildCircleBlob(cx, cy, u.blobRadius, u.nVerts, u.scene);
  sim.verts = verts;
  sim.edges = edges;
  sim.restArea = restArea;
  sim.grabIndex = -1;
  sim.areaLambda = 0;
  syncVerletState();
}

function resetLambdas() {
  for (const e of sim.edges) e.lambda = 0;
  sim.areaLambda = 0;
}

function alphaCompliance(complianceUi, invDt2, stiffScale) {
  const c = Math.max(0, complianceUi) * 1e-8;
  return (c * invDt2) / (Math.max(20, stiffScale) * 0.01);
}

function solveEdgeConstraint(e, alpha) {
  const v = sim.verts;
  const a = v[e.i];
  const b = v[e.j];
  let dx = b.x - a.x;
  let dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-12) return;
  const nx = dx / len;
  const ny = dy / len;
  const C = len - e.rest;
  const w1 = a.pinned ? 0 : a.invMass;
  const w2 = b.pinned ? 0 : b.invMass;
  const wsum = w1 + w2;
  if (wsum <= 0) return;

  const dL = (-C - alpha * e.lambda) / (wsum + alpha);
  e.lambda += dL;
  if (!a.pinned) {
    a.x -= nx * dL * w1;
    a.y -= ny * dL * w1;
  }
  if (!b.pinned) {
    b.x += nx * dL * w2;
    b.y += ny * dL * w2;
  }
}

function solveEdgePBD(e, omega) {
  const v = sim.verts;
  const a = v[e.i];
  const b = v[e.j];
  let dx = b.x - a.x;
  let dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-12) return;
  const nx = dx / len;
  const ny = dy / len;
  const C = len - e.rest;
  const w1 = a.pinned ? 0 : a.invMass;
  const w2 = b.pinned ? 0 : b.invMass;
  const wsum = w1 + w2;
  if (wsum <= 0) return;
  const corr = (-C / wsum) * omega;
  if (!a.pinned) {
    a.x -= nx * corr * w1;
    a.y -= ny * corr * w1;
  }
  if (!b.pinned) {
    b.x += nx * corr * w2;
    b.y += ny * corr * w2;
  }
}

function solveAreaConstraint(alpha) {
  const v = sim.verts;
  const n = v.length;
  const A = polygonSignedArea(v);
  const C = A - sim.restArea;

  const gx = new Float64Array(n);
  const gy = new Float64Array(n);
  let denom = 0;
  for (let k = 0; k < n; k++) {
    const im = (k - 1 + n) % n;
    const ip = (k + 1) % n;
    gx[k] = 0.5 * (v[ip].y - v[im].y);
    gy[k] = 0.5 * (v[im].x - v[ip].x);
    const w = v[k].pinned ? 0 : v[k].invMass;
    denom += w * (gx[k] * gx[k] + gy[k] * gy[k]);
  }
  if (denom < 1e-20) return;

  const dL = (-C - alpha * sim.areaLambda) / (denom + alpha);
  sim.areaLambda += dL;
  for (let k = 0; k < n; k++) {
    if (v[k].pinned) continue;
    const w = v[k].invMass;
    v[k].x += w * dL * gx[k];
    v[k].y += w * dL * gy[k];
  }
}

function solveAreaPBD(omega) {
  const v = sim.verts;
  const n = v.length;
  const A = polygonSignedArea(v);
  const C = A - sim.restArea;
  const gx = new Float64Array(n);
  const gy = new Float64Array(n);
  let denom = 0;
  for (let k = 0; k < n; k++) {
    const im = (k - 1 + n) % n;
    const ip = (k + 1) % n;
    gx[k] = 0.5 * (v[ip].y - v[im].y);
    gy[k] = 0.5 * (v[im].x - v[ip].x);
    const w = v[k].pinned ? 0 : v[k].invMass;
    denom += w * (gx[k] * gx[k] + gy[k] * gy[k]);
  }
  if (denom < 1e-20) return;
  const dL = (-C / denom) * omega;
  for (let k = 0; k < n; k++) {
    if (v[k].pinned) continue;
    const w = v[k].invMass;
    v[k].x += w * dL * gx[k];
    v[k].y += w * dL * gy[k];
  }
}

function applyGrab() {
  if (sim.grabIndex >= 0) {
    const g = sim.verts[sim.grabIndex];
    g.x = sim.pointerX + sim.grabOx;
    g.y = sim.pointerY + sim.grabOy;
  }
}

function omegaFromCompliance(complianceUi, stiffScale) {
  const t = Math.max(0, complianceUi) / 400;
  const base = (Math.max(20, stiffScale) / 100) * (1 - 0.85 * t);
  return Math.min(1.05, Math.max(0.08, base));
}

function stepProjectiveDynamics(iters, u, inX, inY) {
  const v = sim.verts;
  const n = v.length;
  const wEdge = 0.35 * (Math.max(20, u.stiffScale) / 100) * (1 / (1 + u.complianceEdge * 0.004));
  const wInBase = 0.14 * (1 + u.complianceEdge * 0.002);

  for (let k = 0; k < iters; k++) {
    const sumX = new Float64Array(n);
    const sumY = new Float64Array(n);
    const sumW = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      if (v[i].pinned) continue;
      const wi = wInBase * v[i].invMass;
      sumX[i] = wi * inX[i];
      sumY[i] = wi * inY[i];
      sumW[i] = wi;
    }
    for (const e of sim.edges) {
      const a = v[e.i];
      const b = v[e.j];
      const ax = a.x;
      const ay = a.y;
      const bx = b.x;
      const by = b.y;
      let dx = bx - ax;
      let dy = by - ay;
      const len = Math.hypot(dx, dy);
      if (len < 1e-12) continue;
      const nx = dx / len;
      const ny = dy / len;
      const L = e.rest;
      const mx = (ax + bx) * 0.5;
      const my = (ay + by) * 0.5;
      const pix = mx - nx * (L * 0.5);
      const piy = my - ny * (L * 0.5);
      const pjx = mx + nx * (L * 0.5);
      const pjy = my + ny * (L * 0.5);
      const we = wEdge;
      if (!a.pinned) {
        sumX[e.i] += we * pix;
        sumY[e.i] += we * piy;
        sumW[e.i] += we;
      }
      if (!b.pinned) {
        sumX[e.j] += we * pjx;
        sumY[e.j] += we * pjy;
        sumW[e.j] += we;
      }
    }
    for (let i = 0; i < n; i++) {
      if (v[i].pinned) {
        v[i].x = inX[i];
        v[i].y = inY[i];
        continue;
      }
      if (sumW[i] < 1e-12) continue;
      v[i].x = sumX[i] / sumW[i];
      v[i].y = sumY[i] / sumW[i];
    }
    solveAreaPBD(omegaFromCompliance(u.complianceVol, u.stiffScale) * 0.65);
    groundProject();
    applyGrab();
  }
}

/** Итеративная релаксация длины ребра (веса invMass). */
function relaxEdgeLength(e, relax) {
  const v = sim.verts;
  const a = v[e.i];
  const b = v[e.j];
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-12) return;
  const w1 = a.pinned ? 0 : a.invMass;
  const w2 = b.pinned ? 0 : b.invMass;
  const ws = w1 + w2;
  if (ws <= 0) return;
  let diff = ((len - e.rest) / (len * ws)) * relax;
  if (!a.pinned) {
    a.x -= w1 * dx * diff;
    a.y -= w1 * dy * diff;
  }
  if (!b.pinned) {
    b.x += w2 * dx * diff;
    b.y += w2 * dy * diff;
  }
}

function stepVBD(iters, u) {
  const v = sim.verts;
  const n = v.length;
  const alpha = Math.min(
    0.58,
    Math.max(0.3, omegaFromCompliance(u.complianceEdge, u.stiffScale) * 0.52),
  );
  const oA = omegaFromCompliance(u.complianceVol, u.stiffScale) * 0.16;
  const edgeTame = 0.32;

  function vertexDualTarget(i) {
    if (v[i].pinned) return;
    const im = (i - 1 + n) % n;
    const ip = (i + 1) % n;
    const p = v[i];
    const vim = v[im];
    const vip = v[ip];
    const ePrev = sim.edges[(i - 1 + n) % n];
    const eNext = sim.edges[i];
    let d1x = p.x - vim.x;
    let d1y = p.y - vim.y;
    let l1 = Math.hypot(d1x, d1y);
    if (l1 < 1e-10) {
      d1x = 1;
      d1y = 0;
      l1 = 1;
    }
    const t1x = vim.x + (d1x / l1) * ePrev.rest;
    const t1y = vim.y + (d1y / l1) * ePrev.rest;
    let d2x = p.x - vip.x;
    let d2y = p.y - vip.y;
    let l2 = Math.hypot(d2x, d2y);
    if (l2 < 1e-10) {
      d2x = -1;
      d2y = 0;
      l2 = 1;
    }
    const t2x = vip.x + (d2x / l2) * eNext.rest;
    const t2y = vip.y + (d2y / l2) * eNext.rest;
    const tx = 0.5 * (t1x + t2x);
    const ty = 0.5 * (t1y + t2y);
    p.x += (tx - p.x) * alpha;
    p.y += (ty - p.y) * alpha;
  }

  for (let k = 0; k < iters; k++) {
    for (let i = 0; i < n; i++) vertexDualTarget(i);
    for (let i = n - 1; i >= 0; i--) vertexDualTarget(i);
    for (const e of sim.edges) relaxEdgeLength(e, edgeTame);
    solveAreaPBD(oA);
    groundProject();
    applyGrab();
  }
}

function groundProject() {
  const gy = sim.groundY;
  const pad = 3;
  for (const p of sim.verts) {
    if (p.pinned) continue;
    if (p.y + pad > gy) p.y = gy - pad;
  }
}

function groundVelocityResponse(restitution, friction) {
  const gy = sim.groundY;
  const eps = 2;
  const pad = 3;
  for (const p of sim.verts) {
    if (p.pinned) continue;
    if (p.y + pad < gy - eps) continue;
    if (p.vy > 0.35) p.vy = -restitution * p.vy;
    else if (p.vy >= 0) p.vy = 0;
    p.vx *= Math.max(0, 1 - friction);
  }
}

function integrate(h, g) {
  for (const p of sim.verts) {
    if (p.pinned) {
      p.vx = 0;
      p.vy = 0;
      continue;
    }
    p.vy += g * h;
    p.px = p.x;
    p.py = p.y;
    p.x += p.vx * h;
    p.y += p.vy * h;
  }
}

function velocityUpdate(h) {
  const inv = 1 / h;
  for (const p of sim.verts) {
    if (p.pinned) continue;
    p.vx = (p.x - p.px) * inv;
    p.vy = (p.y - p.py) * inv;
  }
}

function stepPhysics(h, u) {
  const invDt2 = 1 / (h * h);
  const aEdge = alphaCompliance(u.complianceEdge, invDt2, u.stiffScale);
  const aVol = alphaCompliance(u.complianceVol, invDt2, u.stiffScale);
  const iters = u.iterations;
  const rest = u.restitution;
  const fric = u.friction;
  const method = u.method || "xpbd";

  integrate(h, u.gravity);

  if (sim.grabIndex >= 0) {
    const g = sim.verts[sim.grabIndex];
    g.vx = 0;
    g.vy = 0;
  }

  const v = sim.verts;
  const n = v.length;
  const inX = new Float64Array(n);
  const inY = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    inX[i] = v[i].x;
    inY[i] = v[i].y;
  }

  const oE = omegaFromCompliance(u.complianceEdge, u.stiffScale);
  const oV = omegaFromCompliance(u.complianceVol, u.stiffScale);

  if (method === "pd") {
    stepProjectiveDynamics(iters, u, inX, inY);
  } else if (method === "vbd") {
    stepVBD(iters, u);
  } else if (method === "pbd") {
    resetLambdas();
    for (let k = 0; k < iters; k++) {
      for (const e of sim.edges) solveEdgePBD(e, oE);
      solveAreaPBD(oV);
      groundProject();
      applyGrab();
    }
  } else {
    resetLambdas();
    for (let k = 0; k < iters; k++) {
      for (const e of sim.edges) solveEdgeConstraint(e, aEdge);
      solveAreaConstraint(aVol);
      groundProject();
      applyGrab();
    }
  }

  velocityUpdate(h);
  groundVelocityResponse(rest, fric);
}

function tryGrab(mx, my) {
  const rGrab = 22;
  let best = -1;
  let bestD = 1e9;
  for (let i = 0; i < sim.verts.length; i++) {
    const p = sim.verts[i];
    const d = Math.hypot(p.x - mx, p.y - my);
    if (d < rGrab && d < bestD) {
      bestD = d;
      best = i;
    }
  }
  sim.grabIndex = best;
  if (best >= 0) {
    const p = sim.verts[best];
    sim.grabOx = p.x - mx;
    sim.grabOy = p.y - my;
  }
}

function physicsStepFixedFrame(u, substeps) {
  const sub = Math.max(1, substeps);
  const h = PHYS_FIXED_DT / sub;
  for (let s = 0; s < sub; s++) stepPhysics(h, u);
}

function vertsHaveNaN() {
  for (const p of sim.verts) {
    if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return true;
  }
  return false;
}
