const SUPPORT_EPS = 1e-9;

export function createTask5State() {
  const bounds = { x: 520, y: 330 };
  const bodies = [
    ...createPyramid(bounds),
    makeBox(0, bounds.y - 20, bounds.x * 2, 40, 0, true),
    makeBox(-bounds.x, 0, 40, bounds.y * 2, 0, true),
    makeBox(bounds.x, 0, 40, bounds.y * 2, 0, true),
    makeCircle(-390, bounds.y - 40 - 38, 34),
    makeCapsule(255, bounds.y - 40 - 24, 95, 18, 0),
    makeRegularPolygon(430, bounds.y - 40 - 48, 5, 42, 0.3),
  ];

  bodies.forEach((body, id) => {
    body.id = id;
    updateBodyCache(body);
  });

  return {
    bounds,
    bodies,
    contacts: [],
    contactCache: new Map(),
    drag: null,
    simplex: [],
    candidatePairs: [],
    gravity: 700,
    substeps: 8,
    iterations: 10,
    dragStrength: 0.1,
    dragMaxCorrection: 12,
    slop: 0.25,
    compliance: 0.00001,
    muStatic: 0.95,
    muDynamic: 0.55,
    linearDamping: 14,
    angularDamping: 32,
    sleepSpeed: 18,
    sleepOmega: 0.35,
    sleepFramesRequired: 8,
    maxLinearSpeed: 110,
    maxAngularSpeed: 1.8,
    cellSize: 128,
    gridCells: 0,
    gravityRampTime: 0.3,
    stackSettleTime: 1.1,
    gjkTests: 0,
    epaCalls: 0,
    clippedContacts: 0,
    constraintSolves: 0,
    staticFrictionContacts: 0,
    dynamicFrictionContacts: 0,
    sleepingCount: 0,
    avgSpeed: 0,
    maxSpeed: 0,
    elapsed: 0,
  };
}

export function stepTask5(state, dt) {
  const frameDt = Math.min(dt, 1 / 60);
  const h = frameDt / state.substeps;

  for (let step = 0; step < state.substeps; step += 1) {
    beginFrame(state, h);
    integrateBodies(state.bodies, rampedGravity(state), h);
    solveMouseDrag(state);
    findContacts(state);
    wakeBodiesInActiveContacts(state);
    solveXpbdContacts(state, h);
    reconstructVelocities(state.bodies, h);
    removeNormalVelocity(state);
    solveStaticDynamicFriction(state);
    applyDamping(state.bodies, state, h);
    clampBodyVelocities(state.bodies, state);
    stabilizeRestingVelocities(state);
    stabilizeStackStartup(state);
    keepBodiesInsideContainer(state.bodies, state.bounds);
  }

  collectMetrics(state);
}

function rampedGravity(state) {
  return state.gravity * clamp(state.elapsed / state.gravityRampTime, 0, 1);
}

export function beginTask5Drag(state, x, y) {
  const body = pickBody(state, x, y);
  if (!body) return false;

  wakeBody(body);
  const c = Math.cos(body.angle);
  const s = Math.sin(body.angle);
  const dx = x - body.x;
  const dy = y - body.y;
  state.drag = {
    bodyId: body.id,
    targetX: x,
    targetY: y,
    localX: c * dx + s * dy,
    localY: -s * dx + c * dy,
  };
  return true;
}

export function updateTask5Drag(state, x, y) {
  if (!state.drag) return;
  state.drag.targetX = x;
  state.drag.targetY = y;
}

export function endTask5Drag(state) {
  state.drag = null;
}

function createPyramid(bounds) {
  const bodies = [];
  const w = 64;
  const h = 32;
  const rows = 4;
  const floorY = bounds.y - 40;
  const overlap = 0.27;
  const offsetX = 8;

  for (let row = 0; row < rows; row += 1) {
    const count = rows - row;
    const y = floorY - h * 0.5 + overlap - row * (h - overlap);
    const startX = -((count - 1) * (w + 0.5)) * 0.5 + offsetX;

    for (let i = 0; i < count; i += 1) {
      const body = makeBox(startX + i * (w + 0.5), y, w, h, 0);
      body.stableStack = true;
      bodies.push(body);
    }
  }

  return bodies;
}

function makeBaseBody(x, y, vx, vy, isStatic) {
  return {
    id: -1,
    x,
    y,
    prevX: x,
    prevY: y,
    angle: 0,
    prevAngle: 0,
    vx,
    vy,
    omega: 0,
    isStatic,
    sleeping: false,
    sleepFrames: 0,
    invMass: 0,
    invInertia: 0,
    cache: { aabb: { minX: 0, maxX: 0, minY: 0, maxY: 0 }, vertices: [] },
  };
}

function makeBox(x, y, w, h, angle = 0, isStatic = false, vx = 0, vy = 0) {
  const body = makeBaseBody(x, y, vx, vy, isStatic);
  body.kind = "polygon";
  body.angle = angle;
  body.prevAngle = angle;
  body.local = [
    { x: -w * 0.5, y: -h * 0.5 },
    { x: w * 0.5, y: -h * 0.5 },
    { x: w * 0.5, y: h * 0.5 },
    { x: -w * 0.5, y: h * 0.5 },
  ];
  setMass(body, isStatic, w * h, (w * w + h * h) / 12);
  return body;
}

function makeRegularPolygon(x, y, sides, radius, angle = 0, isStatic = false, vx = 0, vy = 0) {
  const body = makeBaseBody(x, y, vx, vy, isStatic);
  body.kind = "polygon";
  body.angle = angle;
  body.prevAngle = angle;
  body.local = [];
  for (let i = 0; i < sides; i += 1) {
    const a = -Math.PI * 0.5 + (i / sides) * Math.PI * 2;
    body.local.push({ x: Math.cos(a) * radius, y: Math.sin(a) * radius });
  }
  setMass(body, isStatic, radius * radius * sides, radius * radius * 0.5);
  return body;
}

function makeCircle(x, y, radius, isStatic = false, vx = 0, vy = 0) {
  const body = makeBaseBody(x, y, vx, vy, isStatic);
  body.kind = "circle";
  body.radius = radius;
  setMass(body, isStatic, Math.PI * radius * radius, radius * radius * 0.5);
  return body;
}

function makeCapsule(x, y, length, radius, angle = 0, isStatic = false, vx = 0, vy = 0) {
  const body = makeBaseBody(x, y, vx, vy, isStatic);
  body.kind = "capsule";
  body.length = length;
  body.radius = radius;
  body.angle = angle;
  body.prevAngle = angle;
  setMass(body, isStatic, length * radius * 2 + Math.PI * radius * radius, (length * length) / 12 + radius * radius);
  return body;
}

function setMass(body, isStatic, area, inertiaScale) {
  if (isStatic) {
    body.invMass = 0;
    body.invInertia = 0;
    return;
  }

  const mass = Math.max(0.5, area / 1800);
  body.invMass = 1 / mass;
  body.invInertia = 1 / Math.max(1, mass * inertiaScale);
}

function pickBody(state, x, y) {
  for (let i = state.bodies.length - 1; i >= 0; i -= 1) {
    const body = state.bodies[i];
    if (body.isStatic) continue;
    if (!pointInAabb(x, y, body.cache.aabb)) continue;
    if (pointInBody(body, x, y)) return body;
  }
  return null;
}

function pointInBody(body, x, y) {
  if (body.kind === "circle") {
    return Math.hypot(x - body.x, y - body.y) <= body.radius;
  }

  if (body.kind === "capsule") {
    const axis = { x: Math.cos(body.angle), y: Math.sin(body.angle) };
    const ax = body.x - axis.x * body.length * 0.5;
    const ay = body.y - axis.y * body.length * 0.5;
    const bx = body.x + axis.x * body.length * 0.5;
    const by = body.y + axis.y * body.length * 0.5;
    return pointSegmentDistance(x, y, ax, ay, bx, by) <= body.radius;
  }

  return pointInPolygon(body.cache.vertices, x, y);
}

function pointInAabb(x, y, aabb) {
  return x >= aabb.minX && x <= aabb.maxX && y >= aabb.minY && y <= aabb.maxY;
}

function pointInPolygon(vertices, x, y) {
  let sign = 0;
  for (let i = 0; i < vertices.length; i += 1) {
    const a = vertices[i];
    const b = vertices[(i + 1) % vertices.length];
    const c = cross(b.x - a.x, b.y - a.y, x - a.x, y - a.y);
    if (Math.abs(c) < SUPPORT_EPS) continue;
    const nextSign = Math.sign(c);
    if (sign && nextSign !== sign) return false;
    sign = nextSign;
  }
  return true;
}

function pointSegmentDistance(px, py, ax, ay, bx, by) {
  const abx = bx - ax;
  const aby = by - ay;
  const t = clamp(((px - ax) * abx + (py - ay) * aby) / Math.max(SUPPORT_EPS, abx * abx + aby * aby), 0, 1);
  const x = ax + abx * t;
  const y = ay + aby * t;
  return Math.hypot(px - x, py - y);
}

function beginFrame(state, h) {
  state.elapsed += h;
  state.contacts = [];
  state.candidatePairs = [];
  state.gjkTests = 0;
  state.epaCalls = 0;
  state.clippedContacts = 0;
  state.constraintSolves = 0;
  state.staticFrictionContacts = 0;
  state.dynamicFrictionContacts = 0;
  state.gridCells = 0;
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

function solveMouseDrag(state) {
  if (!state.drag) return;

  const body = state.bodies[state.drag.bodyId];
  if (!body || body.isStatic) {
    state.drag = null;
    return;
  }

  wakeBody(body);
  const c = Math.cos(body.angle);
  const s = Math.sin(body.angle);
  const anchorX = body.x + c * state.drag.localX - s * state.drag.localY;
  const anchorY = body.y + s * state.drag.localX + c * state.drag.localY;
  let dx = state.drag.targetX - anchorX;
  let dy = state.drag.targetY - anchorY;
  const len = Math.hypot(dx, dy);

  if (len > state.dragMaxCorrection) {
    const scale = state.dragMaxCorrection / len;
    dx *= scale;
    dy *= scale;
  }

  const ix = dx * state.dragStrength;
  const iy = dy * state.dragStrength;
  body.x += ix;
  body.y += iy;
  body.angle += cross(state.drag.localX, state.drag.localY, ix, iy) * body.invInertia * 0.25;
  updateBodyCache(body);
}

function wakeBody(body) {
  body.sleeping = false;
  body.sleepFrames = 0;
}

function wakeBodiesInActiveContacts(state) {
  let changed = true;
  while (changed) {
    changed = false;
    for (const contact of state.contacts) {
      const a = state.bodies[contact.a];
      const b = state.bodies[contact.b];
      if (!a || !b || (a.isStatic && b.isStatic)) continue;

      const aAwake = !a.isStatic && !a.sleeping;
      const bAwake = !b.isStatic && !b.sleeping;

      if (aAwake && !b.isStatic && b.sleeping) {
        wakeBody(b);
        changed = true;
      }
      if (bAwake && !a.isStatic && a.sleeping) {
        wakeBody(a);
        changed = true;
      }
    }
  }
}

function findContacts(state) {
  const bodies = state.bodies;
  const broadphase = buildHashBroadphasePairs(bodies, state.cellSize);
  state.candidatePairs = broadphase.pairs;
  state.gridCells = broadphase.gridCells;

  for (const pair of state.candidatePairs) {
      const i = Math.floor(pair / bodies.length);
      const j = pair - i * bodies.length;
      const a = bodies[i];
      const b = bodies[j];
      if (a.isStatic && b.isStatic) continue;
      if (!aabbOverlap(a.cache.aabb, b.cache.aabb)) continue;

      state.gjkTests += 1;
      const gjk = gjkIntersect(a, b);
      if (gjk.hit) {
        state.epaCalls += 1;
        epaPenetration(a, b, gjk.simplex);
      }

      const stableContact = satPolygonContact(a, b);
      if (
        !stableContact ||
        !isFiniteVec(stableContact.normal) ||
        !Number.isFinite(stableContact.depth) ||
        stableContact.depth <= state.slop
      ) {
        continue;
      }

      const points = planeClipping(a, b, stableContact.normal);
      state.clippedContacts += points.length;

      const point = averageContactPoint(points, a, b, stableContact.normal);
      if (!isFiniteVec(point)) continue;
      state.contacts.push({
        key: `${i}:${j}`,
        a: i,
        b: j,
        px: point.x,
        py: point.y,
        nx: stableContact.normal.x,
        ny: stableContact.normal.y,
        depth: stableContact.depth,
        weight: 1,
        lambdaN: 0,
      });
  }
}

function buildHashBroadphasePairs(bodies, cellSize) {
  const grid = new Map();
  const pairSet = new Set();
  const n = bodies.length;

  for (let i = 0; i < bodies.length; i += 1) {
    const aabb = bodies[i].cache.aabb;
    const minX = Math.floor(aabb.minX / cellSize);
    const maxX = Math.floor(aabb.maxX / cellSize);
    const minY = Math.floor(aabb.minY / cellSize);
    const maxY = Math.floor(aabb.maxY / cellSize);

    for (let y = minY; y <= maxY; y += 1) {
      for (let x = minX; x <= maxX; x += 1) {
        const key = `${x},${y}`;
        let bucket = grid.get(key);
        if (!bucket) {
          bucket = [];
          grid.set(key, bucket);
        }
        bucket.push(i);
      }
    }
  }

  for (const bucket of grid.values()) {
    for (let i = 0; i < bucket.length; i += 1) {
      for (let j = i + 1; j < bucket.length; j += 1) {
        const a = Math.min(bucket[i], bucket[j]);
        const b = Math.max(bucket[i], bucket[j]);
        if (a !== b) pairSet.add(a * n + b);
      }
    }
  }

  return { pairs: [...pairSet].sort((a, b) => a - b), gridCells: grid.size };
}

function averageContactPoint(points, a, b, normal) {
  let x = 0;
  let y = 0;
  let count = 0;

  for (const p of points) {
    if (!isFiniteVec(p)) continue;
    x += p.x;
    y += p.y;
    count += 1;
  }

  if (count > 0) return { x: x / count, y: y / count };

  const pa = supportPoint(a, normal);
  const pb = supportPoint(b, neg(normal));
  return { x: (pa.x + pb.x) * 0.5, y: (pa.y + pb.y) * 0.5 };
}

function gjkIntersect(a, b) {
  let direction = { x: b.x - a.x || 1, y: b.y - a.y || 0 };
  const simplex = [support(a, b, direction)];
  direction = neg(simplex[0]);

  for (let iter = 0; iter < 24; iter += 1) {
    const p = support(a, b, direction);
    if (dot(p, direction) <= 0) return { hit: false, simplex };

    simplex.push(p);
    if (handleSimplex(simplex, direction)) return { hit: true, simplex };
  }

  return { hit: false, simplex };
}

function handleSimplex(simplex, direction) {
  const a = simplex[simplex.length - 1];
  const ao = neg(a);

  if (simplex.length === 2) {
    const b = simplex[0];
    const ab = sub(b, a);
    const next = tripleProduct(ab, ao, ab);
    direction.x = Math.abs(next.x) < SUPPORT_EPS && Math.abs(next.y) < SUPPORT_EPS ? -ab.y : next.x;
    direction.y = Math.abs(next.x) < SUPPORT_EPS && Math.abs(next.y) < SUPPORT_EPS ? ab.x : next.y;
    return false;
  }

  const b = simplex[1];
  const c = simplex[0];
  const ab = sub(b, a);
  const ac = sub(c, a);
  const abPerp = tripleProduct(ac, ab, ab);
  const acPerp = tripleProduct(ab, ac, ac);

  if (dot(abPerp, ao) > 0) {
    simplex.splice(0, 1);
    direction.x = abPerp.x;
    direction.y = abPerp.y;
    return false;
  }

  if (dot(acPerp, ao) > 0) {
    simplex.splice(1, 1);
    direction.x = acPerp.x;
    direction.y = acPerp.y;
    return false;
  }

  return true;
}

function epaPenetration(a, b, simplex) {
  let polytope = simplex.slice();
  ensureCcw(polytope);

  for (let iter = 0; iter < 32; iter += 1) {
    const edge = closestEdge(polytope);
    const p = support(a, b, edge.normal);
    const distance = dot(p, edge.normal);

    if (distance - edge.distance < 0.5) {
      const normal = edge.normal;
      if (dot(normal, { x: b.x - a.x, y: b.y - a.y }) < 0) {
        normal.x *= -1;
        normal.y *= -1;
      }
      return { normal, depth: distance };
    }

    polytope.splice(edge.index + 1, 0, p);
  }

  return null;
}

function closestEdge(polytope) {
  let best = { distance: Infinity, normal: { x: 1, y: 0 }, index: 0 };

  for (let i = 0; i < polytope.length; i += 1) {
    const j = (i + 1) % polytope.length;
    const a = polytope[i];
    const b = polytope[j];
    const e = sub(b, a);
    let normal = normalize({ x: e.y, y: -e.x });
    let distance = dot(normal, a);
    if (distance < 0) {
      normal = neg(normal);
      distance = -distance;
    }

    if (distance < best.distance) best = { distance, normal, index: i };
  }

  return best;
}

function planeClipping(a, b, normal) {
  if (a.kind !== "polygon" || b.kind !== "polygon") {
    return [{ x: (supportPoint(a, normal).x + supportPoint(b, neg(normal)).x) * 0.5, y: (supportPoint(a, normal).y + supportPoint(b, neg(normal)).y) * 0.5 }];
  }

  const ref = bestReferenceEdge(a, normal);
  const inc = bestReferenceEdge(b, neg(normal));
  const side = normalize(sub(ref.b, ref.a));
  let points = [inc.a, inc.b];
  points = clipSegment(points, neg(side), -dot(neg(side), ref.a));
  points = clipSegment(points, side, dot(side, ref.b));

  return points.length ? points : [{ x: (ref.a.x + ref.b.x) * 0.5, y: (ref.a.y + ref.b.y) * 0.5 }];
}

function bestReferenceEdge(body, normal) {
  const vertices = body.cache.vertices;
  let bestIndex = 0;
  let bestDot = -Infinity;

  for (let i = 0; i < vertices.length; i += 1) {
    const a = vertices[i];
    const b = vertices[(i + 1) % vertices.length];
    const edge = sub(b, a);
    const edgeNormal = normalize({ x: edge.y, y: -edge.x });
    const d = dot(edgeNormal, normal);
    if (d > bestDot) {
      bestDot = d;
      bestIndex = i;
    }
  }

  return { a: vertices[bestIndex], b: vertices[(bestIndex + 1) % vertices.length] };
}

function clipSegment(points, normal, offset) {
  if (points.length < 2) return points;
  const out = [];
  const d0 = dot(normal, points[0]) - offset;
  const d1 = dot(normal, points[1]) - offset;

  if (d0 <= 0) out.push(points[0]);
  if (d1 <= 0) out.push(points[1]);
  if (d0 * d1 < 0) {
    const t = d0 / (d0 - d1);
    out.push({
      x: points[0].x + (points[1].x - points[0].x) * t,
      y: points[0].y + (points[1].y - points[0].y) * t,
    });
  }

  return out.slice(0, 2);
}

function satPolygonContact(a, b) {
  const axes = polygonAxes(a).concat(polygonAxes(b));
  let bestDepth = Infinity;
  let bestNormal = { x: 1, y: 0 };

  for (const axis of axes) {
    const depth = overlapProjected(a.cache.vertices, b.cache.vertices, axis);
    if (depth <= 0) return null;
    if (depth < bestDepth) {
      bestDepth = depth;
      bestNormal = axis;
    }
  }

  if (dot(bestNormal, { x: b.x - a.x, y: b.y - a.y }) < 0) {
    bestNormal = neg(bestNormal);
  }

  return { normal: bestNormal, depth: bestDepth };
}

function polygonAxes(body) {
  const axes = [];
  const vertices = body.cache.vertices;

  for (let i = 0; i < vertices.length; i += 1) {
    const a = vertices[i];
    const b = vertices[(i + 1) % vertices.length];
    const edge = sub(b, a);
    axes.push(normalize({ x: edge.y, y: -edge.x }));
  }

  return axes;
}

function overlapProjected(aVertices, bVertices, axis) {
  let minA = Infinity;
  let maxA = -Infinity;
  let minB = Infinity;
  let maxB = -Infinity;

  for (const p of aVertices) {
    const d = dot(p, axis);
    minA = Math.min(minA, d);
    maxA = Math.max(maxA, d);
  }
  for (const p of bVertices) {
    const d = dot(p, axis);
    minB = Math.min(minB, d);
    maxB = Math.max(maxB, d);
  }

  return Math.min(maxA, maxB) - Math.max(minA, minB);
}

function support(a, b, direction) {
  const pa = supportPoint(a, direction);
  const pb = supportPoint(b, neg(direction));
  return sub(pa, pb);
}

function supportPoint(body, direction) {
  const d = normalize(direction);

  if (body.kind === "circle") {
    return { x: body.x + d.x * body.radius, y: body.y + d.y * body.radius };
  }

  if (body.kind === "capsule") {
    const axis = { x: Math.cos(body.angle), y: Math.sin(body.angle) };
    const cap = dot(axis, d) >= 0 ? body.length * 0.5 : -body.length * 0.5;
    return {
      x: body.x + axis.x * cap + d.x * body.radius,
      y: body.y + axis.y * cap + d.y * body.radius,
    };
  }

  let best = body.cache.vertices[0];
  let bestDot = dot(best, d);
  for (const vertex of body.cache.vertices) {
    const value = dot(vertex, d);
    if (value > bestDot) {
      bestDot = value;
      best = vertex;
    }
  }
  return { x: best.x, y: best.y };
}

function solveXpbdContacts(state, h) {
  const alpha = state.compliance / (h * h);
  for (let iter = 0; iter < state.iterations; iter += 1) {
    for (const contact of state.contacts) {
      solveContact(state.bodies[contact.a], state.bodies[contact.b], contact, alpha, state.slop);
      state.constraintSolves += 1;
    }
  }
}

function solveContact(a, b, contact, alpha, slop) {
  if (!isFiniteContact(contact)) return;
  updateBodyCache(a);
  updateBodyCache(b);
  const depth = currentDepthAlongNormal(a, b, contact.nx, contact.ny);
  const c = Math.max(0, depth * (contact.weight || 1) - slop);
  if (c === 0) return;

  const k = effectiveMass(a, b, contact.px, contact.py, contact.nx, contact.ny);
  if (!Number.isFinite(k) || k <= 1e-9) return;

  const old = contact.lambdaN;
  contact.lambdaN = Math.max(0, old + (c - alpha * old) / (k + alpha));
  const lambda = clamp(contact.lambdaN - old, -2, 2);
  if (!Number.isFinite(lambda)) return;
  applyPositionImpulse(a, b, contact.px, contact.py, contact.nx * lambda, contact.ny * lambda);
}

function currentDepthAlongNormal(a, b, nx, ny) {
  const normal = { x: nx, y: ny };
  if (a.kind === "polygon" && b.kind === "polygon") {
    return overlapProjected(a.cache.vertices, b.cache.vertices, normal);
  }

  const pa = supportPoint(a, normal);
  const pb = supportPoint(b, neg(normal));
  const depth = dot(sub(pa, pb), normal);
  return Number.isFinite(depth) ? depth : 0;
}

function removeNormalVelocity(state) {
  for (const c of state.contacts) {
    const a = state.bodies[c.a];
    const b = state.bodies[c.b];
    const rv = relativeVelocity(a, b, c.px, c.py);
    const vn = dot(rv, { x: c.nx, y: c.ny });
    const k = effectiveMass(a, b, c.px, c.py, c.nx, c.ny);
    if (k <= 1e-9) continue;

    if (Math.abs(vn) < 160 || vn > 0) {
      const lambda = clamp(-vn / k, -25, 25);
      applyVelocityImpulse(a, b, c.px, c.py, c.nx * lambda, c.ny * lambda);
    }
  }
}

function solveStaticDynamicFriction(state) {
  for (const c of state.contacts) {
    const a = state.bodies[c.a];
    const b = state.bodies[c.b];
    const rv = relativeVelocity(a, b, c.px, c.py);
    let tangent = { x: rv.x - c.nx * dot(rv, { x: c.nx, y: c.ny }), y: rv.y - c.ny * dot(rv, { x: c.nx, y: c.ny }) };
    const speed = length(tangent);
    if (speed <= 1e-6) continue;
    tangent = scale(tangent, 1 / speed);

    const k = effectiveMass(a, b, c.px, c.py, tangent.x, tangent.y);
    if (k <= 1e-9) continue;

    const desired = speed / k;
    const staticLimit = state.muStatic * c.lambdaN;
    if (desired <= staticLimit) {
      applyVelocityImpulse(a, b, c.px, c.py, -tangent.x * desired, -tangent.y * desired);
      state.staticFrictionContacts += 1;
    } else {
      const dynamic = state.muDynamic * c.lambdaN;
      applyVelocityImpulse(a, b, c.px, c.py, -tangent.x * dynamic, -tangent.y * dynamic);
      state.dynamicFrictionContacts += 1;
    }
  }
}

function reconstructVelocities(bodies, h) {
  for (const b of bodies) {
    if (b.isStatic || b.sleeping) {
      b.vx = 0;
      b.vy = 0;
      b.omega = 0;
      continue;
    }
    b.vx = (b.x - b.prevX) / h;
    b.vy = (b.y - b.prevY) / h;
    b.omega = (b.angle - b.prevAngle) / h;
  }
}

function applyDamping(bodies, state, h) {
  const linear = Math.max(0, 1 - state.linearDamping * h);
  const angular = Math.max(0, 1 - state.angularDamping * h);
  for (const b of bodies) {
    if (b.isStatic) continue;
    b.vx *= linear;
    b.vy *= linear;
    b.omega *= angular;
  }
}

function clampBodyVelocities(bodies, state) {
  for (const b of bodies) {
    if (b.isStatic || b.sleeping) continue;
    const speed = Math.hypot(b.vx, b.vy);
    if (speed > state.maxLinearSpeed) {
      const s = state.maxLinearSpeed / speed;
      b.vx *= s;
      b.vy *= s;
    }
    b.omega = clamp(b.omega, -state.maxAngularSpeed, state.maxAngularSpeed);
  }
}

function stabilizeRestingVelocities(state) {
  const touching = new Set();
  const staticSupport = new Set();
  for (const contact of state.contacts) {
    touching.add(contact.a);
    touching.add(contact.b);
    const a = state.bodies[contact.a];
    const b = state.bodies[contact.b];
    if (a?.isStatic && !b?.isStatic) staticSupport.add(contact.b);
    if (b?.isStatic && !a?.isStatic) staticSupport.add(contact.a);
  }

  for (let i = 0; i < state.bodies.length; i += 1) {
    const body = state.bodies[i];
    if (body.isStatic || !touching.has(i) || state.drag?.bodyId === i) continue;

    const speed = Math.hypot(body.vx, body.vy);
    if (speed < 3) {
      body.vx = 0;
      body.vy = 0;
    }
    if (Math.abs(body.omega) < 0.5) {
      body.omega = 0;
    }

    if (body.stableStack && staticSupport.has(i)) {
      body.vx *= 0.35;
      body.omega *= 0.18;
      if (Math.abs(body.vx) < 1.2) body.vx = 0;
      if (Math.abs(body.omega) < 0.25) body.omega = 0;
    }
  }
}

function stabilizeStackStartup(state) {
  if (state.elapsed >= state.stackSettleTime) return;
  const blend = 1 - state.elapsed / state.stackSettleTime;
  const maxLinear = 20 * blend + 8;
  const maxAngular = 0.35 * blend + 0.15;

  for (const body of state.bodies) {
    if (!body.stableStack || state.drag?.bodyId === body.id) continue;

    const speed = Math.hypot(body.vx, body.vy);
    if (speed > maxLinear) {
      const s = maxLinear / speed;
      body.vx *= s;
      body.vy *= s;
    }
    body.omega = clamp(body.omega, -maxAngular, maxAngular);
  }
}

function keepBodiesInsideContainer(bodies, bounds) {
  const left = -bounds.x + 40;
  const right = bounds.x - 40;
  const floor = bounds.y - 40;

  for (const b of bodies) {
    if (b.isStatic || b.sleeping) continue;
    updateBodyCache(b);

    if (b.cache.aabb.minX < left) {
      b.x += left - b.cache.aabb.minX;
      if (b.vx < 0) b.vx = 0;
    }
    if (b.cache.aabb.maxX > right) {
      b.x -= b.cache.aabb.maxX - right;
      if (b.vx > 0) b.vx = 0;
    }
    if (b.cache.aabb.maxY > floor) {
      b.y -= b.cache.aabb.maxY - floor;
      if (b.vy > 0) b.vy = 0;
    }

    updateBodyCache(b);
  }
}

function sleepBodies(state) {
  const touching = new Set();
  for (const contact of state.contacts) {
    touching.add(contact.a);
    touching.add(contact.b);
  }

  for (let i = 0; i < state.bodies.length; i += 1) {
    const b = state.bodies[i];
    if (b.isStatic) continue;
    if (!touching.has(i)) {
      wakeBody(b);
      continue;
    }
    const speed = Math.hypot(b.vx, b.vy);
    if (speed < state.sleepSpeed && Math.abs(b.omega) < state.sleepOmega) {
      b.sleepFrames += 1;
      if (b.sleepFrames > state.sleepFramesRequired) {
        b.sleeping = true;
        b.vx = 0;
        b.vy = 0;
        b.omega = 0;
      }
    } else {
      b.sleepFrames = 0;
      b.sleeping = false;
    }
  }
}

function collectMetrics(state) {
  let sum = 0;
  let count = 0;
  let sleeping = 0;
  state.maxSpeed = 0;
  for (const body of state.bodies) {
    if (body.isStatic) continue;
    if (body.sleeping) sleeping += 1;
    const speed = Math.hypot(body.vx, body.vy);
    sum += speed;
    state.maxSpeed = Math.max(state.maxSpeed, speed);
    count += 1;
  }
  state.avgSpeed = count ? sum / count : 0;
  state.sleepingCount = sleeping;
}

function updateBodyCache(body) {
  if (body.kind === "polygon") {
    const c = Math.cos(body.angle);
    const s = Math.sin(body.angle);
    body.cache.vertices.length = 0;
    for (const p of body.local) {
      body.cache.vertices.push({ x: body.x + c * p.x - s * p.y, y: body.y + s * p.x + c * p.y });
    }
    updateAabbFromPoints(body.cache.aabb, body.cache.vertices);
    return;
  }

  if (body.kind === "circle") {
    body.cache.vertices.length = 0;
    for (let i = 0; i < 16; i += 1) {
      const a = (i / 16) * Math.PI * 2;
      body.cache.vertices.push({
        x: body.x + Math.cos(a) * body.radius,
        y: body.y + Math.sin(a) * body.radius,
      });
    }
    updateAabbFromPoints(body.cache.aabb, body.cache.vertices);
    return;
  }

  const axis = { x: Math.cos(body.angle), y: Math.sin(body.angle) };
  const normal = { x: -axis.y, y: axis.x };
  const left = { x: body.x - axis.x * body.length * 0.5, y: body.y - axis.y * body.length * 0.5 };
  const right = { x: body.x + axis.x * body.length * 0.5, y: body.y + axis.y * body.length * 0.5 };
  body.cache.vertices.length = 0;

  for (let i = 0; i <= 6; i += 1) {
    const a = Math.PI * 0.5 + (i / 6) * Math.PI;
    body.cache.vertices.push({
      x: left.x + (axis.x * Math.cos(a) + normal.x * Math.sin(a)) * body.radius,
      y: left.y + (axis.y * Math.cos(a) + normal.y * Math.sin(a)) * body.radius,
    });
  }
  for (let i = 0; i <= 6; i += 1) {
    const a = -Math.PI * 0.5 + (i / 6) * Math.PI;
    body.cache.vertices.push({
      x: right.x + (axis.x * Math.cos(a) + normal.x * Math.sin(a)) * body.radius,
      y: right.y + (axis.y * Math.cos(a) + normal.y * Math.sin(a)) * body.radius,
    });
  }

  updateAabbFromPoints(body.cache.aabb, body.cache.vertices);
}

function updateAabbFromPoints(aabb, points) {
  aabb.minX = Infinity;
  aabb.maxX = -Infinity;
  aabb.minY = Infinity;
  aabb.maxY = -Infinity;
  for (const p of points) {
    aabb.minX = Math.min(aabb.minX, p.x);
    aabb.maxX = Math.max(aabb.maxX, p.x);
    aabb.minY = Math.min(aabb.minY, p.y);
    aabb.maxY = Math.max(aabb.maxY, p.y);
  }
}

function aabbOverlap(a, b) {
  return !(a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY);
}

function effectiveMass(a, b, px, py, nx, ny) {
  if (![px, py, nx, ny].every(Number.isFinite)) return 0;
  const rax = px - a.x;
  const ray = py - a.y;
  const rbx = px - b.x;
  const rby = py - b.y;
  const raCrossN = cross(rax, ray, nx, ny);
  const rbCrossN = cross(rbx, rby, nx, ny);
  return activeInvMass(a) + activeInvMass(b) + raCrossN * raCrossN * activeInvInertia(a) + rbCrossN * rbCrossN * activeInvInertia(b);
}

function applyPositionImpulse(a, b, px, py, ix, iy) {
  if (![px, py, ix, iy].every(Number.isFinite)) return;
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
  if (![px, py, ix, iy].every(Number.isFinite)) return;
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
  return { x: b.vx - b.omega * rby - (a.vx - a.omega * ray), y: b.vy + b.omega * rbx - (a.vy + a.omega * rax) };
}

function activeInvMass(body) {
  return body.sleeping ? 0 : body.invMass;
}

function activeInvInertia(body) {
  return body.sleeping ? 0 : body.invInertia;
}

function ensureCcw(polytope) {
  let area = 0;
  for (let i = 0; i < polytope.length; i += 1) {
    const a = polytope[i];
    const b = polytope[(i + 1) % polytope.length];
    area += cross(a.x, a.y, b.x, b.y);
  }
  if (area < 0) polytope.reverse();
}

function sub(a, b) {
  return { x: a.x - b.x, y: a.y - b.y };
}

function neg(v) {
  return { x: -v.x, y: -v.y };
}

function scale(v, s) {
  return { x: v.x * s, y: v.y * s };
}

function length(v) {
  return Math.hypot(v.x, v.y);
}

function normalize(v) {
  const len = length(v);
  if (len < SUPPORT_EPS) return { x: 1, y: 0 };
  return { x: v.x / len, y: v.y / len };
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

function cross(ax, ay, bx, by) {
  return ax * by - ay * bx;
}

function tripleProduct(a, b, c) {
  const ac = dot(a, c);
  const bc = dot(b, c);
  return { x: b.x * ac - a.x * bc, y: b.y * ac - a.y * bc };
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function isFiniteVec(v) {
  return v && Number.isFinite(v.x) && Number.isFinite(v.y);
}

function isFiniteContact(c) {
  return (
    Number.isFinite(c.px) &&
    Number.isFinite(c.py) &&
    Number.isFinite(c.nx) &&
    Number.isFinite(c.ny) &&
    Number.isFinite(c.depth)
  );
}
