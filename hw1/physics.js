/**
 * Пружинные системы (2D): силы, энергия, интеграторы.
 * Без p5 — только численная физика.
 */

export const MU_DEG = 110;
export const MU = (MU_DEG * Math.PI) / 180;

/** @typedef {{ x: number; y: number }} Vec2 */

/** @typedef {{ type: 'anchor'; ax: number; ay: number; i: number; k: number; L0: number }} SpringAnchor */
/** @typedef {{ type: 'pair'; i: number; j: number; k: number; L0: number }} SpringPair */
/** @typedef {SpringAnchor | SpringPair} Spring */

/**
 * @typedef {{
 *   sceneId: string;
 *   n: number;
 *   mass: Float64Array;
 *   pinned: Uint8Array;
 *   x: Float64Array;
 *   y: Float64Array;
 *   vx: Float64Array;
 *   vy: Float64Array;
 *   px: Float64Array;
 *   py: Float64Array;
 *   springs: Spring[];
 *   anchorPoints: Vec2[];
 *   origin: Vec2;
 *   g: number;
 *   initialEnergy: number;
 * }} SimState */

function hypot(dx, dy) {
  return Math.hypot(dx, dy);
}

function springForceScalar(dx, dy, k, L0, outFx, outFy) {
  const d = hypot(dx, dy);
  if (d < 1e-14) {
    outFx[0] = 0;
    outFy[0] = 0;
    return;
  }
  const s = k * (d - L0);
  const inv = s / d;
  outFx[0] = inv * dx;
  outFy[0] = inv * dy;
}

/**
 * Ускорения для всех частиц (гравитация + пружины).
 * @param {SimState} s
 * @param {Float64Array} x
 * @param {Float64Array} y
 * @param {Float64Array} ax
 * @param {Float64Array} ay
 */
export function accelerationsAt(s, x, y, ax, ay) {
  const n = s.n;
  for (let i = 0; i < n; i++) {
    ax[i] = 0;
    ay[i] = 0;
  }
  const tmpFx = [0];
  const tmpFy = [0];

  for (const sp of s.springs) {
    if (sp.type === "anchor") {
      const dx = sp.ax - x[sp.i];
      const dy = sp.ay - y[sp.i];
      springForceScalar(dx, dy, sp.k, sp.L0, tmpFx, tmpFy);
      const im = 1 / s.mass[sp.i];
      ax[sp.i] += tmpFx[0] * im;
      ay[sp.i] += tmpFy[0] * im;
    } else {
      const i = sp.i;
      const j = sp.j;
      const dx = x[j] - x[i];
      const dy = y[j] - y[i];
      springForceScalar(dx, dy, sp.k, sp.L0, tmpFx, tmpFy);
      const imi = 1 / s.mass[i];
      const imj = 1 / s.mass[j];
      ax[i] += tmpFx[0] * imi;
      ay[i] += tmpFy[0] * imi;
      ax[j] -= tmpFx[0] * imj;
      ay[j] -= tmpFy[0] * imj;
    }
  }

  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) continue;
    ay[i] += s.g;
  }
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) {
      ax[i] = 0;
      ay[i] = 0;
    }
  }
}

export function kineticEnergy(s) {
  let t = 0;
  for (let i = 0; i < s.n; i++) {
    if (s.pinned[i]) continue;
    const vx = s.vx[i];
    const vy = s.vy[i];
    t += 0.5 * s.mass[i] * (vx * vx + vy * vy);
  }
  return t;
}

export function springPotential(s, x, y) {
  let u = 0;
  for (const sp of s.springs) {
    if (sp.type === "anchor") {
      const d = hypot(sp.ax - x[sp.i], sp.ay - y[sp.i]);
      const e = d - sp.L0;
      u += 0.5 * sp.k * e * e;
    } else {
      const d = hypot(x[sp.j] - x[sp.i], y[sp.j] - y[sp.i]);
      const e = d - sp.L0;
      u += 0.5 * sp.k * e * e;
    }
  }
  return u;
}

export function gravityPotential(s, y) {
  let u = 0;
  for (let i = 0; i < s.n; i++) {
    if (s.pinned[i]) continue;
    u -= s.mass[i] * s.g * y[i];
  }
  return u;
}

export function totalEnergy(s) {
  return kineticEnergy(s) + springPotential(s, s.x, s.y) + gravityPotential(s, s.y);
}

const _ax = new Float64Array(8);
const _ay = new Float64Array(8);
const _x1 = new Float64Array(8);
const _y1 = new Float64Array(8);
const _x0 = new Float64Array(8);
const _y0 = new Float64Array(8);

/**
 * Явный Эйлер: x += dt v, v += dt a(x)
 * @param {SimState} s
 * @param {number} dt
 */
export function stepExplicitEuler(s, dt) {
  const n = s.n;
  accelerationsAt(s, s.x, s.y, _ax, _ay);
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) continue;
    const vx0 = s.vx[i];
    const vy0 = s.vy[i];
    s.vx[i] += _ax[i] * dt;
    s.vy[i] += _ay[i] * dt;
    s.x[i] += vx0 * dt;
    s.y[i] += vy0 * dt;
  }
}

/**
 * Полуявный (симплектический) Эйлер: v += dt a(x), x += dt v
 * @param {SimState} s
 * @param {number} dt
 */
export function stepSemiImplicitEuler(s, dt) {
  const n = s.n;
  accelerationsAt(s, s.x, s.y, _ax, _ay);
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) continue;
    s.vx[i] += _ax[i] * dt;
    s.vy[i] += _ay[i] * dt;
    s.x[i] += s.vx[i] * dt;
    s.y[i] += s.vy[i] * dt;
  }
}

/**
 * Верле: x_{n+1} = 2 x_n - x_{n-1} + dt^2 a(x_n); затем v из разности.
 * @param {SimState} s
 * @param {number} dt
 */
export function stepVerlet(s, dt) {
  const n = s.n;
  const dt2 = dt * dt;
  const inv2dt = 1 / (2 * dt);
  accelerationsAt(s, s.x, s.y, _ax, _ay);
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) continue;
    const xPrev = s.px[i];
    const yPrev = s.py[i];
    const xCurr = s.x[i];
    const yCurr = s.y[i];
    const nx = 2 * xCurr - xPrev + _ax[i] * dt2;
    const ny = 2 * yCurr - yPrev + _ay[i] * dt2;
    s.px[i] = xCurr;
    s.py[i] = yCurr;
    s.x[i] = nx;
    s.y[i] = ny;
    s.vx[i] = (nx - xPrev) * inv2dt;
    s.vy[i] = (ny - yPrev) * inv2dt;
  }
}

/**
 * Неявный Эйлер: x1 = x0 + dt v0 + dt^2 a(x1) — сходящаяся фиксированная точка.
 * @param {SimState} s
 * @param {number} dt
 */
export function stepImplicitEuler(s, dt) {
  const n = s.n;
  const dt2 = dt * dt;
  const invDt = 1 / dt;
  for (let i = 0; i < n; i++) {
    _x0[i] = s.x[i];
    _y0[i] = s.y[i];
    _x1[i] = s.x[i];
    _y1[i] = s.y[i];
  }
  const iters = 16;
  for (let k = 0; k < iters; k++) {
    accelerationsAt(s, _x1, _y1, _ax, _ay);
    for (let i = 0; i < n; i++) {
      if (s.pinned[i]) {
        _x1[i] = _x0[i];
        _y1[i] = _y0[i];
        continue;
      }
      _x1[i] = _x0[i] + s.vx[i] * dt + _ax[i] * dt2;
      _y1[i] = _y0[i] + s.vy[i] * dt + _ay[i] * dt2;
    }
  }
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) continue;
    s.vx[i] = (_x1[i] - _x0[i]) * invDt;
    s.vy[i] = (_y1[i] - _y0[i]) * invDt;
    s.x[i] = _x1[i];
    s.y[i] = _y1[i];
  }
}

/** Синхронизация предыдущих позиций для Верле после смены метода/сброса */
export function syncVerletPrev(s, dt) {
  const n = s.n;
  accelerationsAt(s, s.x, s.y, _ax, _ay);
  for (let i = 0; i < n; i++) {
    if (s.pinned[i]) {
      s.px[i] = s.x[i];
      s.py[i] = s.y[i];
      continue;
    }
    s.px[i] = s.x[i] - s.vx[i] * dt + 0.5 * _ax[i] * dt * dt;
    s.py[i] = s.y[i] - s.vy[i] * dt + 0.5 * _ay[i] * dt * dt;
  }
}

/**
 * @param {string} sceneId 'corner2' | 'corner3' | 'twoBodies'
 * @param {{ canvasW: number; canvasH: number }} canvas
 * @returns {SimState}
 */
export function createInitialState(sceneId, canvas) {
  const g = 420;
  const origin = { x: canvas.canvasW * 0.22, y: canvas.canvasH - 72 };
  const arm = 200;

  if (sceneId === "corner2") {
    const n = 1;
    const mass = new Float64Array([1.2]);
    const pinned = new Uint8Array([0]);
    const x = new Float64Array([origin.x + 120]);
    const y = new Float64Array([origin.y - 140]);
    const vx = new Float64Array([40]);
    const vy = new Float64Array([0]);
    const px = new Float64Array(n);
    const py = new Float64Array(n);
    const ax1 = origin.x + arm * Math.cos(-MU);
    const ay1 = origin.y + arm * Math.sin(-MU);
    const ax2 = origin.x + arm;
    const ay2 = origin.y;
    const k1 = 2.8;
    const k2 = 2.8;
    const L1 = hypot(ax1 - x[0], ay1 - y[0]);
    const L2 = hypot(ax2 - x[0], ay2 - y[0]);
    const springs = /** @type {Spring[]} */ ([
      { type: "anchor", ax: ax1, ay: ay1, i: 0, k: k1, L0: L1 * 0.92 },
      { type: "anchor", ax: ax2, ay: ay2, i: 0, k: k2, L0: L2 * 0.92 },
    ]);
    const s = {
      sceneId,
      n,
      mass,
      pinned,
      x,
      y,
      vx,
      vy,
      px,
      py,
      springs,
      anchorPoints: [
        { x: ax1, y: ay1 },
        { x: ax2, y: ay2 },
      ],
      origin,
      g,
      initialEnergy: 0,
    };
    s.initialEnergy = totalEnergy(/** @type {SimState} */ (s));
    return /** @type {SimState} */ (s);
  }

  if (sceneId === "corner3") {
    /** Одна масса, три пружины к трём якорям (как GamesPhysics/ThreeSprings), угол μ. */
    const n = 1;
    const mass = new Float64Array([1.0]);
    const pinned = new Uint8Array([0]);
    const x = new Float64Array([origin.x + 160]);
    const y = new Float64Array([origin.y - 160]);
    const vx = new Float64Array([25]);
    const vy = new Float64Array([-15]);
    const px = new Float64Array(n);
    const py = new Float64Array(n);
    const ax1 = origin.x;
    const ay1 = origin.y - arm;
    const ax3 = origin.x + arm;
    const ay3 = origin.y;
    const ax2 = origin.x + arm * Math.sin(MU) + 200;
    const ay2 = origin.y - arm * Math.cos(MU) - 200;
    const k = 2.2;
    const L1 = hypot(ax1 - x[0], ay1 - y[0]);
    const L2 = hypot(ax2 - x[0], ay2 - y[0]);
    const L3 = hypot(ax3 - x[0], ay3 - y[0]);
    const springs = /** @type {Spring[]} */ ([
      { type: "anchor", ax: ax1, ay: ay1, i: 0, k, L0: L1 * 0.9 },
      { type: "anchor", ax: ax2, ay: ay2, i: 0, k, L0: L2 * 0.9 },
      { type: "anchor", ax: ax3, ay: ay3, i: 0, k, L0: L3 * 0.9 },
    ]);
    const s = {
      sceneId,
      n,
      mass,
      pinned,
      x,
      y,
      vx,
      vy,
      px,
      py,
      springs,
      anchorPoints: [
        { x: ax1, y: ay1 },
        { x: ax2, y: ay2 },
        { x: ax3, y: ay3 },
      ],
      origin,
      g,
      initialEnergy: 0,
    };
    s.initialEnergy = totalEnergy(/** @type {SimState} */ (s));
    return /** @type {SimState} */ (s);
  }

  if (sceneId === "twoBodies") {
    /**
     * «Две точки связанные пружинкой» по схеме задания: две массы, три пружины —
     * стена слева |—m₁—m₂—/ наклонная стена; μ — угол между вертикалью вверх от m₂
     * и осью третьей пружины (по часовой от (0,-1) к направлению к якорю).
     */
    const n = 2;
    const mass = new Float64Array([1.0, 1.0]);
    const pinned = new Uint8Array([0, 0]);
    const wallX = origin.x;
    const yChain = origin.y - 110;
    const L12 = 92;
    const L23 = 92;
    const L3 = 155;
    const dir3x = Math.sin(MU);
    const dir3y = -Math.cos(MU);
    const axLeft = wallX;
    const ayLeft = yChain;
    const x0 = wallX + L12;
    const y0 = yChain;
    const x1 = wallX + L12 + L23;
    const y1 = yChain;
    const axRight = x1 + L3 * dir3x;
    const ayRight = y1 + L3 * dir3y;
    const x = new Float64Array([x0, x1]);
    const y = new Float64Array([y0, y1]);
    const vx = new Float64Array([12, -8]);
    const vy = new Float64Array([0, -10]);
    const px = new Float64Array(n);
    const py = new Float64Array(n);
    const k = 2.6;
    const springs = /** @type {Spring[]} */ ([
      { type: "anchor", ax: axLeft, ay: ayLeft, i: 0, k, L0: L12 },
      { type: "pair", i: 0, j: 1, k, L0: L23 },
      { type: "anchor", ax: axRight, ay: ayRight, i: 1, k, L0: L3 },
    ]);
    const s = {
      sceneId,
      n,
      mass,
      pinned,
      x,
      y,
      vx,
      vy,
      px,
      py,
      springs,
      anchorPoints: [
        { x: axLeft, y: ayLeft },
        { x: axRight, y: ayRight },
      ],
      origin,
      g: 0,
      initialEnergy: 0,
    };
    s.initialEnergy = totalEnergy(/** @type {SimState} */ (s));
    return /** @type {SimState} */ (s);
  }

  throw new Error(`Unknown scene: ${sceneId}`);
}

/**
 * @param {SimState} s
 * @param {string} method 'explicit' | 'implicit' | 'semi' | 'verlet'
 * @param {number} dt
 */
export function stepByMethod(s, method, dt) {
  if (method === "explicit") stepExplicitEuler(s, dt);
  else if (method === "implicit") stepImplicitEuler(s, dt);
  else if (method === "semi") stepSemiImplicitEuler(s, dt);
  else if (method === "verlet") stepVerlet(s, dt);
}

/** @param {SimState} s @returns {SimState} */
export function cloneSimState(s) {
  const n = s.n;
  const springs = s.springs.map((sp) =>
    sp.type === "anchor"
      ? { type: "anchor", ax: sp.ax, ay: sp.ay, i: sp.i, k: sp.k, L0: sp.L0 }
      : { type: "pair", i: sp.i, j: sp.j, k: sp.k, L0: sp.L0 },
  );
  const anchorPoints = s.anchorPoints.map((p) => ({ x: p.x, y: p.y }));
  return {
    sceneId: s.sceneId,
    n,
    mass: Float64Array.from(s.mass),
    pinned: Uint8Array.from(s.pinned),
    x: Float64Array.from(s.x),
    y: Float64Array.from(s.y),
    vx: Float64Array.from(s.vx),
    vy: Float64Array.from(s.vy),
    px: Float64Array.from(s.px),
    py: Float64Array.from(s.py),
    springs,
    anchorPoints,
    origin: { x: s.origin.x, y: s.origin.y },
    g: s.g,
    initialEnergy: s.initialEnergy,
  };
}

export const METHOD_LABELS = {
  explicit: "Явный Эйлер",
  implicit: "Неявный Эйлер",
  semi: "Полуявный (симплектический) Эйлер",
  verlet: "Верле",
};
