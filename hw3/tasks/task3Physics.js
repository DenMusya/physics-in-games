const SOLVERS = ["XPBD", "Sequential Impulses"];

class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }
}

function makeBody(x, y, width, height, angle, vx = 0, vy = 0, omega = 0, isStatic = false) {
  const mass = isStatic ? Infinity : (width * height) / 2600;
  const inertia = isStatic
    ? Infinity
    : (mass * (width * width + height * height)) / 12;

  return {
    id: -1,
    x,
    y,
    prevX: x,
    prevY: y,
    angle,
    prevAngle: angle,
    vx,
    vy,
    omega,
    width,
    height,
    isStatic,
    sleeping: false,
    sleepFrames: 0,
    invMass: isStatic ? 0 : 1 / mass,
    invInertia: isStatic ? 0 : 1 / inertia,
    cache: {
      exx: 1,
      exy: 0,
      eyx: 0,
      eyy: 1,
      corners: [new Vec2(), new Vec2(), new Vec2(), new Vec2()],
      aabb: { minX: 0, maxX: 0, minY: 0, maxY: 0 },
    },
  };
}

export function createCollisionDetectionState(bodyCount = 10, solverIndex = 0) {
  const largeMode = bodyCount >= 1000;
  const bounds = largeMode ? { x: 780, y: 560 } : { x: 480, y: 340 };
  const bodies = [
    ...createDynamicBodies(bodyCount, bounds),
    ...createStaticBox(bounds),
  ];

  bodies.forEach((body, id) => {
    body.id = id;
    updateBodyCache(body);
  });

  return {
    bodyCount,
    solverIndex,
    solver: SOLVERS[solverIndex],
    gravity: largeMode ? 900 : 1400,
    substeps: largeMode ? 6 : 4,
    iterations: largeMode ? 4 : 6,
    positionIterations: largeMode ? 4 : 4,
    restitution: 0,
    friction: 0.85,
    linearDamping: largeMode ? 6.5 : 2.2,
    angularDamping: largeMode ? 9.5 : 3.2,
    baumgarte: largeMode ? 0.04 : 0.08,
    slop: largeMode ? 0.7 : 1.2,
    sleepSpeed: largeMode ? 30 : 8,
    sleepOmega: largeMode ? 0.65 : 0.18,
    cellSize: largeMode ? 34 : 150,
    bounds,
    bodies,
    contacts: [],
    candidatePairs: [],
    gridCells: 0,
    pairsTested: 0,
    satCalls: 0,
    constraintSolves: 0,
    avgPenetration: 0,
    avgSpeed: 0,
    avgVy: 0,
    maxSpeed: 0,
    sleepingCount: 0,
    elapsed: 0,
  };
}

function createStaticBox(bounds) {
  const thickness = 44;

  return [
    makeBody(0, bounds.y, bounds.x * 2, thickness, 0, 0, 0, 0, true),
    makeBody(0, -bounds.y, bounds.x * 2, thickness, 0, 0, 0, 0, true),
    makeBody(-bounds.x, 0, thickness, bounds.y * 2, 0, 0, 0, 0, true),
    makeBody(bounds.x, 0, thickness, bounds.y * 2, 0, 0, 0, 0, true),
  ];
}

function createDynamicBodies(count, bounds) {
  if (count <= 10) {
    return [
      makeBody(-210, -230, 90, 42, 0.2, 16, 0, 0.22),
      makeBody(-95, -260, 76, 56, -0.3, -10, 0, -0.18),
      makeBody(30, -220, 104, 36, 0.35, 12, 0, 0.16),
      makeBody(155, -270, 68, 68, -0.15, -14, 0, -0.12),
      makeBody(-160, -145, 112, 34, 0.45, 8, 0, 0.14),
      makeBody(-35, -165, 72, 72, -0.4, -6, 0, -0.16),
      makeBody(110, -140, 118, 38, 0.1, 10, 0, 0.1),
      makeBody(210, -185, 82, 48, -0.5, -10, 0, -0.16),
      makeBody(-85, -70, 92, 40, 0.15, 8, 0, 0.08),
      makeBody(45, -80, 86, 56, -0.25, -8, 0, -0.12),
    ];
  }

  const bodies = [];
  const columns = 50;
  const spacingX = 28;
  const spacingY = 22;
  const startX = -((columns - 1) * spacingX) * 0.5;
  const startY = -bounds.y + 120;

  for (let i = 0; i < count; i += 1) {
    const col = i % columns;
    const row = Math.floor(i / columns);
    const jitterX = (seededNoise(i, 1) - 0.5) * 3;
    const jitterY = (seededNoise(i, 2) - 0.5) * 2;
    const width = 11 + Math.floor(seededNoise(i, 3) * 7);
    const height = 9 + Math.floor(seededNoise(i, 4) * 7);
    const angle = (seededNoise(i, 5) - 0.5) * 0.18;
    const vx = (seededNoise(i, 6) - 0.5) * 3;
    const omega = (seededNoise(i, 7) - 0.5) * 0.08;

    bodies.push(
      makeBody(
        startX + col * spacingX + jitterX,
        startY + row * spacingY + jitterY,
        width,
        height,
        angle,
        vx,
        0,
        omega
      )
    );
  }

  return bodies;
}

function seededNoise(i, salt) {
  const x = Math.sin(i * 127.1 + salt * 311.7) * 43758.5453123;
  return x - Math.floor(x);
}

export function cycleCollisionSolver(state) {
  state.solverIndex = (state.solverIndex + 1) % SOLVERS.length;
  state.solver = SOLVERS[state.solverIndex];
}

export function stepCollisionDetection(state, dt) {
  const frameDt = Math.min(dt, 1 / 60);
  const h = frameDt / state.substeps;

  for (let step = 0; step < state.substeps; step += 1) {
    beginFrame(state, h);
    integrateBodies(state.bodies, state.gravity, h);
    runCollisionPipeline(state);
    wakeImpactedBodies(state);

    if (state.solver === "XPBD") {
      solveXpbd(state, h);
      reconstructVelocities(state.bodies, h);
      removeRestingNormalVelocity(state);
      solveFriction(state);
    } else {
      solveSequentialImpulses(state, h);
      removeRestingNormalVelocity(state);
      solvePositionProjection(state);
    }

    applyDamping(state.bodies, state, h);
    sleepRestingBodies(state);
  }

  collectMetrics(state);
}

export function bodyCorners(body) {
  return body.cache.corners.map((corner) => new Vec2(corner.x, corner.y));
}

function beginFrame(state, h) {
  state.elapsed += h;
  state.contacts = [];
  state.candidatePairs = [];
  state.gridCells = 0;
  state.pairsTested = 0;
  state.satCalls = 0;
  state.constraintSolves = 0;
  state.avgPenetration = 0;
}

function integrateBodies(bodies, gravity, h) {
  for (const body of bodies) {
    body.prevX = body.x;
    body.prevY = body.y;
    body.prevAngle = body.angle;

    if (!body.isStatic && !body.sleeping) {
      body.vy += gravity * h;
      body.x += body.vx * h;
      body.y += body.vy * h;
      body.angle += body.omega * h;
    }

    updateBodyCache(body);
  }
}

function runCollisionPipeline(state) {
  const broadphase = buildStableGridPairs(state.bodies, state.cellSize);
  state.candidatePairs = broadphase.pairs;
  state.gridCells = broadphase.gridCells;

  const contacts = [];
  let penetrationSum = 0;
  for (const pair of state.candidatePairs) {
    const aIndex = Math.floor(pair / state.bodies.length);
    const bIndex = pair - aIndex * state.bodies.length;
    state.pairsTested += 1;
    const a = state.bodies[aIndex];
    const b = state.bodies[bIndex];

    if (!aabbOverlap(a.cache.aabb, b.cache.aabb)) continue;

    state.satCalls += 1;
    const contact = satRectangles(a, b);
    if (!contact || contact.depth <= state.slop) continue;

    contact.a = aIndex;
    contact.b = bIndex;
    contact.lambdaN = 0;
    contact.lambdaT = 0;
    contacts.push(contact);
    penetrationSum += contact.depth;
  }

  state.contacts = contacts;
  state.avgPenetration = contacts.length ? penetrationSum / contacts.length : 0;
}

function buildStableGridPairs(bodies, cellSize) {
  const pairSet = new Set();
  const grid = new Map();
  const n = bodies.length;

  for (let i = 0; i < bodies.length; i += 1) {
    if (bodies[i].isStatic) continue;
    const aabb = bodies[i].cache.aabb;
    const minX = Math.floor(aabb.minX / cellSize);
    const maxX = Math.floor(aabb.maxX / cellSize);
    const minY = Math.floor(aabb.minY / cellSize);
    const maxY = Math.floor(aabb.maxY / cellSize);

    for (let x = minX; x <= maxX; x += 1) {
      for (let y = minY; y <= maxY; y += 1) {
        const key = x * 1000000 + y;
        if (!grid.has(key)) grid.set(key, []);
        grid.get(key).push(i);
      }
    }
  }

  for (const bucket of grid.values()) {
    for (let i = 0; i < bucket.length; i += 1) {
      for (let j = i + 1; j < bucket.length; j += 1) {
        const a = Math.min(bucket[i], bucket[j]);
        const b = Math.max(bucket[i], bucket[j]);
        if (bodies[a].isStatic && bodies[b].isStatic) continue;
        pairSet.add(a * n + b);
      }
    }
  }

  for (let i = 0; i < bodies.length; i += 1) {
    const body = bodies[i];
    if (body.isStatic) continue;
    for (let j = 0; j < bodies.length; j += 1) {
      if (!bodies[j].isStatic) continue;
      if (!aabbOverlap(body.cache.aabb, bodies[j].cache.aabb)) continue;
      const a = Math.min(i, j);
      const b = Math.max(i, j);
      pairSet.add(a * n + b);
    }
  }

  return {
    gridCells: grid.size,
    pairs: [...pairSet],
  };
}

function solveXpbd(state, h) {
  const alpha = 0.00005 / (h * h);

  for (let iter = 0; iter < state.iterations; iter += 1) {
    for (const contact of state.contacts) {
      const a = state.bodies[contact.a];
      const b = state.bodies[contact.b];
      solveXpbdContact(a, b, contact, alpha, state.slop);
      state.constraintSolves += 1;
    }
  }
}

function solveXpbdContact(a, b, contact, alpha, slop) {
  updateBodyCache(a);
  updateBodyCache(b);

  const depth = overlapOnAxis(a.cache.corners, b.cache.corners, contact.nx, contact.ny);
  const c = Math.max(0, depth - slop);
  if (c === 0) return;

  const k = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
  if (k <= 1e-9) return;

  const oldLambda = contact.lambdaN;
  contact.lambdaN = Math.max(0, oldLambda + (c - alpha * oldLambda) / (k + alpha));
  const lambda = clamp(contact.lambdaN - oldLambda, -2, 2);
  applyPositionImpulse(a, b, contact.px, contact.py, contact.nx * lambda, contact.ny * lambda);
}

function solveSequentialImpulses(state, h) {
  for (let iter = 0; iter < state.iterations; iter += 1) {
    for (const contact of state.contacts) {
      solveVelocityContact(state.bodies[contact.a], state.bodies[contact.b], contact, state, h);
      state.constraintSolves += 1;
    }
  }
}

function solveVelocityContact(a, b, contact, state, h) {
  const rv = relativeVelocity(a, b, contact.px, contact.py);
  const vn = dot(rv.x, rv.y, contact.nx, contact.ny);
  const positionalBias = Math.min(
    20,
    state.baumgarte * Math.max(0, contact.depth - state.slop) / h
  );
  const kN = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
  if (kN <= 1e-9) return;

  const oldLambdaN = contact.lambdaN;
  const bounce = vn < -80 ? -(1 + state.restitution) * vn : -vn;
  const removeVelocity = bounce + positionalBias;
  contact.lambdaN = Math.max(0, oldLambdaN + clamp(removeVelocity / kN, -30, 30));
  const normalImpulse = contact.lambdaN - oldLambdaN;
  applyVelocityImpulse(
    a,
    b,
    contact.px,
    contact.py,
    contact.nx * normalImpulse,
    contact.ny * normalImpulse
  );

  const tangent = relativeVelocity(a, b, contact.px, contact.py);
  let tx = tangent.x - contact.nx * dot(tangent.x, tangent.y, contact.nx, contact.ny);
  let ty = tangent.y - contact.ny * dot(tangent.x, tangent.y, contact.nx, contact.ny);
  const len = Math.hypot(tx, ty);
  if (len <= 1e-9) return;
  tx /= len;
  ty /= len;

  const kT = effectiveMass(a, b, contact.px, contact.py, tx, ty);
  if (kT <= 1e-9) return;

  const rvT = relativeVelocity(a, b, contact.px, contact.py);
  const vt = dot(rvT.x, rvT.y, tx, ty);
  const oldLambdaT = contact.lambdaT;
  const maxFriction = state.friction * contact.lambdaN;
  contact.lambdaT = clamp(oldLambdaT - vt / kT, -maxFriction, maxFriction);
  const tangentImpulse = contact.lambdaT - oldLambdaT;
  applyVelocityImpulse(a, b, contact.px, contact.py, tx * tangentImpulse, ty * tangentImpulse);
}

function solvePositionProjection(state) {
  for (let iter = 0; iter < state.positionIterations; iter += 1) {
    for (const contact of state.contacts) {
      const a = state.bodies[contact.a];
      const b = state.bodies[contact.b];
      updateBodyCache(a);
      updateBodyCache(b);

      const depth = overlapOnAxis(a.cache.corners, b.cache.corners, contact.nx, contact.ny);
      const c = Math.max(0, depth - state.slop);
      if (c === 0) continue;

      const k = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
      if (k <= 1e-9) continue;

      const lambda = Math.min(c / k, 0.7);
      applyPositionImpulse(a, b, contact.px, contact.py, contact.nx * lambda, contact.ny * lambda);
    }
  }
}

function solveFriction(state) {
  for (const contact of state.contacts) {
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    const rv = relativeVelocity(a, b, contact.px, contact.py);
    let tx = rv.x - contact.nx * dot(rv.x, rv.y, contact.nx, contact.ny);
    let ty = rv.y - contact.ny * dot(rv.x, rv.y, contact.nx, contact.ny);
    const len = Math.hypot(tx, ty);
    if (len <= 1e-9) continue;
    tx /= len;
    ty /= len;

    const k = effectiveMass(a, b, contact.px, contact.py, tx, ty);
    if (k <= 1e-9) continue;

    const rvT = relativeVelocity(a, b, contact.px, contact.py);
    const vt = dot(rvT.x, rvT.y, tx, ty);
    const lambda = clamp(-vt / k, -state.friction * contact.lambdaN, state.friction * contact.lambdaN);
    applyVelocityImpulse(a, b, contact.px, contact.py, tx * lambda, ty * lambda);
  }
}

function removeRestingNormalVelocity(state) {
  for (const contact of state.contacts) {
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    const rv = relativeVelocity(a, b, contact.px, contact.py);
    const vn = dot(rv.x, rv.y, contact.nx, contact.ny);
    const k = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
    if (k <= 1e-9) continue;

    // Position correction should not turn into visible bounce. Clamp small
    // contact-normal motion to zero so resting stacks can actually settle.
    if (Math.abs(vn) < 180 || vn > 0) {
      const lambda = clamp(-vn / k, -25, 25);
      applyVelocityImpulse(a, b, contact.px, contact.py, contact.nx * lambda, contact.ny * lambda);
    }
  }
}

function applyDamping(bodies, state, h) {
  const linear = Math.max(0, 1 - state.linearDamping * h);
  const angular = Math.max(0, 1 - state.angularDamping * h);

  for (const body of bodies) {
    if (body.isStatic) continue;
    body.vx *= linear;
    body.vy *= linear;
    body.omega *= angular;
  }
}

function sleepRestingBodies(state) {
  const touching = new Set();

  for (const contact of state.contacts) {
    touching.add(contact.a);
    touching.add(contact.b);
  }

  for (let i = 0; i < state.bodies.length; i += 1) {
    const body = state.bodies[i];
    if (body.isStatic) continue;
    if (!touching.has(i)) {
      body.sleepFrames = 0;
      body.sleeping = false;
      continue;
    }
    const speed = Math.hypot(body.vx, body.vy);
    if (speed < state.sleepSpeed && Math.abs(body.omega) < state.sleepOmega) {
      body.sleepFrames += 1;
      if (body.sleepFrames > 8) {
        body.sleeping = true;
        body.vx = 0;
        body.vy = 0;
        body.omega = 0;
      }
    } else {
      body.sleepFrames = 0;
      body.sleeping = false;
    }
  }
}

function wakeImpactedBodies(state) {
  for (const contact of state.contacts) {
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    const rv = relativeVelocity(a, b, contact.px, contact.py);
    const impactSpeed = Math.abs(dot(rv.x, rv.y, contact.nx, contact.ny));
    if (impactSpeed < state.sleepSpeed * 1.5) continue;

    if (!a.isStatic) {
      a.sleeping = false;
      a.sleepFrames = 0;
    }
    if (!b.isStatic) {
      b.sleeping = false;
      b.sleepFrames = 0;
    }
  }
}

function reconstructVelocities(bodies, h) {
  for (const body of bodies) {
    if (body.isStatic || body.sleeping) {
      body.vx = 0;
      body.vy = 0;
      body.omega = 0;
      continue;
    }

    body.vx = (body.x - body.prevX) / h;
    body.vy = (body.y - body.prevY) / h;
    body.omega = (body.angle - body.prevAngle) / h;
  }
}

function collectMetrics(state) {
  let speedSum = 0;
  let vySum = 0;
  let count = 0;
  let sleepingCount = 0;
  state.maxSpeed = 0;

  for (const body of state.bodies) {
    if (body.isStatic) continue;
    if (body.sleeping) sleepingCount += 1;
    const speed = Math.hypot(body.vx, body.vy);
    speedSum += speed;
    vySum += body.vy;
    state.maxSpeed = Math.max(state.maxSpeed, speed);
    count += 1;
  }

  state.avgSpeed = count ? speedSum / count : 0;
  state.avgVy = count ? vySum / count : 0;
  state.sleepingCount = sleepingCount;
}

function satRectangles(a, b) {
  const axes = [
    [a.cache.exx, a.cache.exy],
    [a.cache.eyx, a.cache.eyy],
    [b.cache.exx, b.cache.exy],
    [b.cache.eyx, b.cache.eyy],
  ];
  let bestDepth = Infinity;
  let nx = 0;
  let ny = 0;

  for (const axis of axes) {
    const depth = overlapOnAxis(a.cache.corners, b.cache.corners, axis[0], axis[1]);
    if (depth <= 0) return null;
    if (depth < bestDepth) {
      bestDepth = depth;
      nx = axis[0];
      ny = axis[1];
    }
  }

  if (dot(b.x - a.x, b.y - a.y, nx, ny) < 0) {
    nx *= -1;
    ny *= -1;
  }

  const point = contactPoint(a, b);
  return { px: point.x, py: point.y, nx, ny, depth: bestDepth };
}

function contactPoint(a, b) {
  let x = 0;
  let y = 0;
  let count = 0;

  for (const p of a.cache.corners) {
    if (pointInBody(p.x, p.y, b)) {
      x += p.x;
      y += p.y;
      count += 1;
    }
  }
  for (const p of b.cache.corners) {
    if (pointInBody(p.x, p.y, a)) {
      x += p.x;
      y += p.y;
      count += 1;
    }
  }

  if (count > 0) return { x: x / count, y: y / count };
  return { x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5 };
}

function pointInBody(x, y, body) {
  const dx = x - body.x;
  const dy = y - body.y;
  const localX = dot(dx, dy, body.cache.exx, body.cache.exy);
  const localY = dot(dx, dy, body.cache.eyx, body.cache.eyy);

  return (
    Math.abs(localX) <= body.width * 0.5 + 1e-6 &&
    Math.abs(localY) <= body.height * 0.5 + 1e-6
  );
}

function overlapOnAxis(cornersA, cornersB, ax, ay) {
  let minA = Infinity;
  let maxA = -Infinity;
  let minB = Infinity;
  let maxB = -Infinity;

  for (const p of cornersA) {
    const d = dot(p.x, p.y, ax, ay);
    minA = Math.min(minA, d);
    maxA = Math.max(maxA, d);
  }
  for (const p of cornersB) {
    const d = dot(p.x, p.y, ax, ay);
    minB = Math.min(minB, d);
    maxB = Math.max(maxB, d);
  }

  return Math.min(maxA, maxB) - Math.max(minA, minB);
}

function updateBodyCache(body) {
  const c = Math.cos(body.angle);
  const s = Math.sin(body.angle);
  const hx = body.width * 0.5;
  const hy = body.height * 0.5;

  body.cache.exx = c;
  body.cache.exy = s;
  body.cache.eyx = -s;
  body.cache.eyy = c;

  setCorner(body.cache.corners[0], body, -hx, -hy);
  setCorner(body.cache.corners[1], body, hx, -hy);
  setCorner(body.cache.corners[2], body, hx, hy);
  setCorner(body.cache.corners[3], body, -hx, hy);
  updateAabb(body.cache.aabb, body.cache.corners);
}

function setCorner(out, body, localX, localY) {
  out.x = body.x + body.cache.exx * localX + body.cache.eyx * localY;
  out.y = body.y + body.cache.exy * localX + body.cache.eyy * localY;
}

function updateAabb(aabb, corners) {
  aabb.minX = Math.min(corners[0].x, corners[1].x, corners[2].x, corners[3].x);
  aabb.maxX = Math.max(corners[0].x, corners[1].x, corners[2].x, corners[3].x);
  aabb.minY = Math.min(corners[0].y, corners[1].y, corners[2].y, corners[3].y);
  aabb.maxY = Math.max(corners[0].y, corners[1].y, corners[2].y, corners[3].y);
}

function aabbOverlap(a, b) {
  return !(
    a.maxX < b.minX ||
    b.maxX < a.minX ||
    a.maxY < b.minY ||
    b.maxY < a.minY
  );
}

function effectiveMass(a, b, px, py, nx, ny) {
  const rax = px - a.x;
  const ray = py - a.y;
  const rbx = px - b.x;
  const rby = py - b.y;
  const raCrossN = cross(rax, ray, nx, ny);
  const rbCrossN = cross(rbx, rby, nx, ny);

  return (
    activeInvMass(a) +
    activeInvMass(b) +
    raCrossN * raCrossN * activeInvInertia(a) +
    rbCrossN * rbCrossN * activeInvInertia(b)
  );
}

function applyPositionImpulse(a, b, px, py, ix, iy) {
  const rax = px - a.x;
  const ray = py - a.y;
  const rbx = px - b.x;
  const rby = py - b.y;

  a.x -= ix * activeInvMass(a);
  a.y -= iy * activeInvMass(a);
  a.angle -= cross(rax, ray, ix, iy) * activeInvInertia(a);
  b.x += ix * activeInvMass(b);
  b.y += iy * activeInvMass(b);
  b.angle += cross(rbx, rby, ix, iy) * activeInvInertia(b);
  updateBodyCache(a);
  updateBodyCache(b);
}

function applyVelocityImpulse(a, b, px, py, ix, iy) {
  const rax = px - a.x;
  const ray = py - a.y;
  const rbx = px - b.x;
  const rby = py - b.y;

  a.vx -= ix * activeInvMass(a);
  a.vy -= iy * activeInvMass(a);
  a.omega -= cross(rax, ray, ix, iy) * activeInvInertia(a);
  b.vx += ix * activeInvMass(b);
  b.vy += iy * activeInvMass(b);
  b.omega += cross(rbx, rby, ix, iy) * activeInvInertia(b);
}

function relativeVelocity(a, b, px, py) {
  const rax = px - a.x;
  const ray = py - a.y;
  const rbx = px - b.x;
  const rby = py - b.y;

  return {
    x: b.vx - b.omega * rby - (a.vx - a.omega * ray),
    y: b.vy + b.omega * rbx - (a.vy + a.omega * rax),
  };
}

function dot(ax, ay, bx, by) {
  return ax * bx + ay * by;
}

function cross(ax, ay, bx, by) {
  return ax * by - ay * bx;
}

function activeInvMass(body) {
  return body.sleeping ? 0 : body.invMass;
}

function activeInvInertia(body) {
  return body.sleeping ? 0 : body.invInertia;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}
