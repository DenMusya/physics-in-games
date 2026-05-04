const BROADPHASES = ["Sweep and Prune", "LBVH"];

class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }
}

function makeBody(x, y, width, height, angle, vx = 0, vy = 0, omega = 0, isStatic = false) {
  const mass = isStatic ? Infinity : Math.max(0.4, (width * height) / 1800);
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

export function createTask4State(broadphaseIndex = 0) {
  const bounds = { x: 760, y: 500 };
  const bodies = [
    ...createBodies(bounds),
    makeBody(0, bounds.y, bounds.x * 2, 44, 0, 0, 0, 0, true),
    makeBody(0, -bounds.y, bounds.x * 2, 44, 0, 0, 0, 0, true),
    makeBody(-bounds.x, 0, 44, bounds.y * 2, 0, 0, 0, 0, true),
    makeBody(bounds.x, 0, 44, bounds.y * 2, 0, 0, 0, 0, true),
  ];

  bodies.forEach((body, id) => {
    body.id = id;
    updateBodyCache(body);
  });

  return {
    broadphaseIndex,
    broadphase: BROADPHASES[broadphaseIndex],
    bounds,
    bodies,
    contacts: [],
    candidatePairs: [],
    gravity: 950,
    substeps: 5,
    iterations: 5,
    slop: 1,
    compliance: 0.00004,
    muStatic: 0.9,
    muDynamic: 0.55,
    linearDamping: 4.5,
    angularDamping: 6,
    sleepSpeed: 18,
    sleepOmega: 0.35,
    pairsTested: 0,
    satCalls: 0,
    constraintSolves: 0,
    avgPenetration: 0,
    staticFrictionContacts: 0,
    dynamicFrictionContacts: 0,
    sleepingCount: 0,
    maxSpeed: 0,
    avgSpeed: 0,
    elapsed: 0,
  };
}

export function cycleTask4Broadphase(state) {
  state.broadphaseIndex = (state.broadphaseIndex + 1) % BROADPHASES.length;
  state.broadphase = BROADPHASES[state.broadphaseIndex];
}

export function stepTask4(state, dt) {
  const frameDt = Math.min(dt, 1 / 60);
  const h = frameDt / state.substeps;

  for (let step = 0; step < state.substeps; step += 1) {
    beginFrame(state, h);
    integrateBodies(state.bodies, state.gravity, h);
    buildContacts(state);
    solveXpbdContacts(state, h);
    reconstructVelocities(state.bodies, h);
    removeRestingNormalVelocity(state);
    solveStaticDynamicFriction(state);
    applyDamping(state.bodies, state, h);
    sleepRestingBodies(state);
  }

  collectMetrics(state);
}

export function task4BodyCorners(body) {
  return body.cache.corners;
}

function createBodies(bounds) {
  const bodies = [];
  const columns = 22;
  const spacingX = 60;
  const spacingY = 42;
  const startX = -((columns - 1) * spacingX) * 0.5;
  const startY = -bounds.y + 90;

  for (let i = 0; i < 180; i += 1) {
    const col = i % columns;
    const row = Math.floor(i / columns);
    const width = 14 + seededNoise(i, 1) * 76;
    const height = 10 + seededNoise(i, 2) * 58;
    const angle = (seededNoise(i, 3) - 0.5) * 0.45;
    const vx = (seededNoise(i, 4) - 0.5) * 8;
    const omega = (seededNoise(i, 5) - 0.5) * 0.18;

    bodies.push(
      makeBody(
        startX + col * spacingX + (seededNoise(i, 6) - 0.5) * 8,
        startY + row * spacingY + (seededNoise(i, 7) - 0.5) * 5,
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

function beginFrame(state, h) {
  state.elapsed += h;
  state.contacts = [];
  state.candidatePairs = [];
  state.pairsTested = 0;
  state.satCalls = 0;
  state.constraintSolves = 0;
  state.avgPenetration = 0;
  state.staticFrictionContacts = 0;
  state.dynamicFrictionContacts = 0;
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

function buildContacts(state) {
  const pairs =
    state.broadphase === "Sweep and Prune"
      ? sweepAndPrunePairs(state.bodies)
      : lbvhPairs(state.bodies, state.bounds);

  const contacts = [];
  let penetrationSum = 0;

  for (const pair of pairs) {
    const aIndex = Math.floor(pair / state.bodies.length);
    const bIndex = pair - aIndex * state.bodies.length;
    const a = state.bodies[aIndex];
    const b = state.bodies[bIndex];
    state.pairsTested += 1;

    if (!aabbOverlap(a.cache.aabb, b.cache.aabb)) continue;

    state.satCalls += 1;
    const contact = satRectangles(a, b);
    if (!contact || contact.depth <= state.slop) continue;

    contact.a = aIndex;
    contact.b = bIndex;
    contact.lambdaN = 0;
    contacts.push(contact);
    penetrationSum += contact.depth;
  }

  state.candidatePairs = pairs;
  state.contacts = contacts;
  state.avgPenetration = contacts.length ? penetrationSum / contacts.length : 0;
}

function sweepAndPrunePairs(bodies) {
  const dynamic = [];
  const pairs = [];
  const n = bodies.length;

  for (let i = 0; i < bodies.length; i += 1) {
    if (!bodies[i].isStatic) dynamic.push(i);
  }

  dynamic.sort((a, b) => bodies[a].cache.aabb.minX - bodies[b].cache.aabb.minX);

  for (let i = 0; i < dynamic.length; i += 1) {
    const a = dynamic[i];
    const aabbA = bodies[a].cache.aabb;

    for (let j = i + 1; j < dynamic.length; j += 1) {
      const b = dynamic[j];
      if (bodies[b].cache.aabb.minX > aabbA.maxX) break;
      if (aabbOverlap(aabbA, bodies[b].cache.aabb)) pairs.push(a * n + b);
    }

    for (let s = 0; s < bodies.length; s += 1) {
      if (!bodies[s].isStatic) continue;
      if (aabbOverlap(aabbA, bodies[s].cache.aabb)) {
        pairs.push(Math.min(a, s) * n + Math.max(a, s));
      }
    }
  }

  return pairs;
}

function lbvhPairs(bodies, bounds) {
  const leaves = [];
  const pairs = [];
  const n = bodies.length;

  for (let i = 0; i < bodies.length; i += 1) {
    if (bodies[i].isStatic) continue;
    const aabb = bodies[i].cache.aabb;
    const cx = (aabb.minX + aabb.maxX) * 0.5;
    const cy = (aabb.minY + aabb.maxY) * 0.5;
    leaves.push({ body: i, code: morton2D(cx, cy, bounds) });
  }

  leaves.sort((a, b) => a.code - b.code);
  const root = buildBvhNode(leaves, 0, leaves.length, bodies);
  if (!root) return pairs;

  for (const leaf of leaves) {
    queryBvh(root, leaf.body, bodies[leaf.body].cache.aabb, bodies, n, pairs);
    for (let s = 0; s < bodies.length; s += 1) {
      if (!bodies[s].isStatic) continue;
      if (aabbOverlap(bodies[leaf.body].cache.aabb, bodies[s].cache.aabb)) {
        pairs.push(Math.min(leaf.body, s) * n + Math.max(leaf.body, s));
      }
    }
  }

  return [...new Set(pairs)];
}

function buildBvhNode(leaves, start, end, bodies) {
  if (start >= end) return null;
  if (end - start === 1) {
    const body = leaves[start].body;
    return { body, left: null, right: null, aabb: cloneAabb(bodies[body].cache.aabb) };
  }

  const mid = Math.floor((start + end) * 0.5);
  const left = buildBvhNode(leaves, start, mid, bodies);
  const right = buildBvhNode(leaves, mid, end, bodies);
  return { body: -1, left, right, aabb: unionAabb(left.aabb, right.aabb) };
}

function queryBvh(node, bodyIndex, queryAabb, bodies, bodyCount, pairs) {
  if (!node || !aabbOverlap(node.aabb, queryAabb)) return;
  if (node.body >= 0) {
    if (node.body > bodyIndex && aabbOverlap(bodies[node.body].cache.aabb, queryAabb)) {
      pairs.push(bodyIndex * bodyCount + node.body);
    }
    return;
  }

  queryBvh(node.left, bodyIndex, queryAabb, bodies, bodyCount, pairs);
  queryBvh(node.right, bodyIndex, queryAabb, bodies, bodyCount, pairs);
}

function solveXpbdContacts(state, h) {
  const alpha = state.compliance / (h * h);

  for (let iter = 0; iter < state.iterations; iter += 1) {
    for (const contact of state.contacts) {
      solveXpbdContact(state.bodies[contact.a], state.bodies[contact.b], contact, alpha, state.slop);
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

function removeRestingNormalVelocity(state) {
  for (const contact of state.contacts) {
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    const rv = relativeVelocity(a, b, contact.px, contact.py);
    const vn = dot(rv.x, rv.y, contact.nx, contact.ny);
    const k = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
    if (k <= 1e-9) continue;

    if (Math.abs(vn) < 160 || vn > 0) {
      const lambda = clamp(-vn / k, -25, 25);
      applyVelocityImpulse(a, b, contact.px, contact.py, contact.nx * lambda, contact.ny * lambda);
    }
  }
}

function solveStaticDynamicFriction(state) {
  for (const contact of state.contacts) {
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    const rv = relativeVelocity(a, b, contact.px, contact.py);
    let tx = rv.x - contact.nx * dot(rv.x, rv.y, contact.nx, contact.ny);
    let ty = rv.y - contact.ny * dot(rv.x, rv.y, contact.nx, contact.ny);
    const tangentSpeed = Math.hypot(tx, ty);
    if (tangentSpeed <= 1e-6) continue;
    tx /= tangentSpeed;
    ty /= tangentSpeed;

    const k = effectiveMass(a, b, contact.px, contact.py, tx, ty);
    if (k <= 1e-9) continue;

    const desiredStaticLambda = tangentSpeed / k;
    const staticLimit = state.muStatic * contact.lambdaN;

    if (desiredStaticLambda <= staticLimit) {
      applyVelocityImpulse(a, b, contact.px, contact.py, -tx * desiredStaticLambda, -ty * desiredStaticLambda);
      state.staticFrictionContacts += 1;
      continue;
    }

    const dynamicLambda = state.muDynamic * contact.lambdaN;
    applyVelocityImpulse(a, b, contact.px, contact.py, -tx * dynamicLambda, -ty * dynamicLambda);
    state.dynamicFrictionContacts += 1;
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

function collectMetrics(state) {
  let speedSum = 0;
  let count = 0;
  let sleeping = 0;
  state.maxSpeed = 0;

  for (const body of state.bodies) {
    if (body.isStatic) continue;
    if (body.sleeping) sleeping += 1;
    const speed = Math.hypot(body.vx, body.vy);
    speedSum += speed;
    state.maxSpeed = Math.max(state.maxSpeed, speed);
    count += 1;
  }

  state.avgSpeed = count ? speedSum / count : 0;
  state.sleepingCount = sleeping;
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

function aabbOverlap(a, b) {
  return !(
    a.maxX < b.minX ||
    b.maxX < a.minX ||
    a.maxY < b.minY ||
    b.maxY < a.minY
  );
}

function cloneAabb(aabb) {
  return { minX: aabb.minX, maxX: aabb.maxX, minY: aabb.minY, maxY: aabb.maxY };
}

function unionAabb(a, b) {
  return {
    minX: Math.min(a.minX, b.minX),
    maxX: Math.max(a.maxX, b.maxX),
    minY: Math.min(a.minY, b.minY),
    maxY: Math.max(a.maxY, b.maxY),
  };
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

function morton2D(x, y, bounds) {
  const nx = clamp(Math.floor(((x + bounds.x) / (bounds.x * 2)) * 1023), 0, 1023);
  const ny = clamp(Math.floor(((y + bounds.y) / (bounds.y * 2)) * 1023), 0, 1023);
  return spreadBits(nx) | (spreadBits(ny) << 1);
}

function spreadBits(v) {
  let x = v & 1023;
  x = (x | (x << 16)) & 0x030000ff;
  x = (x | (x << 8)) & 0x0300f00f;
  x = (x | (x << 4)) & 0x030c30c3;
  x = (x | (x << 2)) & 0x09249249;
  return x;
}

function seededNoise(i, salt) {
  const x = Math.sin(i * 127.1 + salt * 311.7) * 43758.5453123;
  return x - Math.floor(x);
}

function activeInvMass(body) {
  return body.sleeping ? 0 : body.invMass;
}

function activeInvInertia(body) {
  return body.sleeping ? 0 : body.invInertia;
}

function dot(ax, ay, bx, by) {
  return ax * bx + ay * by;
}

function cross(ax, ay, bx, by) {
  return ax * by - ay * bx;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}
