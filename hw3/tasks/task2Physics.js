import { rectangleInertia } from "../physics.js";

class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  add(v) {
    return new Vec2(this.x + v.x, this.y + v.y);
  }

  sub(v) {
    return new Vec2(this.x - v.x, this.y - v.y);
  }

  scale(s) {
    return new Vec2(this.x * s, this.y * s);
  }

  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  length() {
    return Math.hypot(this.x, this.y);
  }
}

function cross(a, b) {
  return a.x * b.y - a.y * b.x;
}

function rotate(v, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return new Vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function springKinematics(body, spring) {
  const r = rotate(body.attachLocal, body.angle);
  const attachWorld = body.pos.add(r);
  const delta = attachWorld.sub(spring.anchor);
  const length = delta.length() || 1;
  const n = delta.scale(1 / length);
  const attachVelocity = body.vel.add(new Vec2(-body.omega * r.y, body.omega * r.x));
  const rn = cross(r, n);
  const invEffectiveMass = body.invMass + rn * rn * body.invInertia;

  return {
    r,
    attachWorld,
    n,
    length,
    stretch: length - spring.restLength,
    constraintVelocity: attachVelocity.dot(n),
    effectiveMass: 1 / invEffectiveMass,
  };
}

export function createSpringBodyState() {
  const mass = 2;
  const width = 180;
  const height = 56;
  const inertia = rectangleInertia(mass, width, height);

  return {
    kind: "force",
    title: "Task 2.1: Hooke spring as external force",
    body: {
      pos: new Vec2(0, 20),
      vel: new Vec2(25, 0),
      angle: 0.35,
      omega: 0,
      width,
      height,
      mass,
      invMass: 1 / mass,
      inertia,
      invInertia: 1 / inertia,
      attachLocal: new Vec2(65, -18),
    },
    spring: {
      anchor: new Vec2(0, -185),
      restLength: 210,
      stiffness: 8,
      damping: 7,
    },
    lastForce: new Vec2(),
    lastCorrection: 0,
    maxDt: 1 / 120,
  };
}

export function createProjectedSpringState() {
  const state = createSpringBodyState();
  return {
    ...state,
    kind: "projection",
    title: "Task 2.2: simplified stable spring projection",
    spring: {
      ...state.spring,
      stiffness: 2,
      damping: 0.35,
      correctionRate: 0.18,
      maxCorrection: 2.5,
    },
  };
}

export function stepSpringBody(state, dt) {
  const { body, spring } = state;
  const h = Math.min(dt, state.maxDt);
  const k = springKinematics(body, spring);
  const force = k.n.scale(
    -spring.stiffness * k.stretch -
      spring.damping * k.constraintVelocity
  );
  const torque = cross(k.r, force);

  body.vel = body.vel.add(force.scale(body.invMass * h));
  body.omega += torque * body.invInertia * h;

  body.pos = body.pos.add(body.vel.scale(h));
  body.angle += body.omega * h;

  state.lastForce = force;
  state.lastCorrection = 0;
}

export function stepProjectedSpring(state, dt) {
  const { body, spring } = state;
  const h = Math.min(dt, state.maxDt);
  let k = springKinematics(body, spring);

  const dampingImpulse = clamp(
    -k.constraintVelocity * k.effectiveMass * spring.damping,
    -4,
    4
  );
  const velocityImpulse = k.n.scale(dampingImpulse);
  body.vel = body.vel.add(velocityImpulse.scale(body.invMass));
  body.omega += body.invInertia * cross(k.r, velocityImpulse);

  body.pos = body.pos.add(body.vel.scale(h));
  body.angle += body.omega * h;

  k = springKinematics(body, spring);
  const correction = clamp(
    -k.stretch * spring.correctionRate,
    -spring.maxCorrection,
    spring.maxCorrection
  );
  const positionLambda = correction * k.effectiveMass;
  const positionImpulse = k.n.scale(positionLambda);

  body.pos = body.pos.add(positionImpulse.scale(body.invMass));
  body.angle += body.invInertia * cross(k.r, positionImpulse);

  state.lastForce = velocityImpulse.scale(1 / h);
  state.lastCorrection = Math.abs(correction);
}

export function springBodyMetrics(state) {
  const { body, spring } = state;
  const k = springKinematics(body, spring);

  return {
    title: state.title,
    kind: state.kind,
    attachWorld: k.attachWorld,
    springLength: k.length,
    stretch: k.stretch,
    effectiveMass: k.effectiveMass,
    stiffness: spring.stiffness,
    damping: spring.damping,
    forceMag: state.lastForce.length(),
    correction: state.lastCorrection,
    potentialEnergy: 0.5 * spring.stiffness * k.stretch * k.stretch,
    kineticEnergy:
      0.5 * body.mass * body.vel.dot(body.vel) +
      0.5 * body.inertia * body.omega * body.omega,
  };
}
