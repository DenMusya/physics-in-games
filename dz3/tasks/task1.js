import { rectangleInertia } from "../physics.js";

const MODES = [
  { id: "explicit", label: "A: Explicit Symplectic Euler" },
  { id: "modified", label: "B: Semi-implicit / modified" },
];

export function createTask1(p) {
  let modeIndex = 0;
  let body;

  function makeBody() {
    const mass = 2;
    const width = 220;
    const height = 70;
    const inertiaZ = rectangleInertia(mass, width, height);
    const inertiaX = (mass * height * height) / 12;
    const inertiaY = (mass * width * width) / 12;
    const omega = 2.4;

    return {
      x: 0,
      y: 0,
      width,
      height,
      mass,
      inertiaZ,
      inertiaX,
      inertiaY,
      angle: 0.35,
      omega,
      initialAngularMomentum: inertiaZ * omega,
      initialEnergy: 0.5 * inertiaZ * omega * omega,
    };
  }

  function metrics() {
    const angularMomentum = body.inertiaZ * body.omega;
    const energy = 0.5 * body.inertiaZ * body.omega * body.omega;

    return {
      worldL: angularMomentum,
      localL: angularMomentum,
      energy,
    };
  }

  return {
    init() {
      body = makeBody();
    },

    reset() {
      body = makeBody();
    },

    update(dt) {
      const asymmetry =
        (body.inertiaY - body.inertiaX) / (body.inertiaY + body.inertiaX);
      const numericalTorque =
        asymmetry * body.inertiaZ * body.omega * body.omega * Math.sin(2 * body.angle);

      if (MODES[modeIndex].id === "explicit") {
        const alpha = numericalTorque / body.inertiaZ;
        body.omega += alpha * dt;
      } else {
        const alpha = numericalTorque / body.inertiaZ;
        const damping = 0.45;
        body.omega = (body.omega + 0.35 * alpha * dt) / (1 + damping * dt);
      }

      body.angle += body.omega * dt;
    },

    render() {
      drawBody();
      drawOverlay();
    },

    keyPressed(key) {
      if (key.toLowerCase() === "m") {
        modeIndex = (modeIndex + 1) % MODES.length;
        this.reset();
      }
      if (key.toLowerCase() === "r") this.reset();
    },
  };

  function drawBody() {
    p.push();
    p.translate(p.width / 2 + body.x, p.height / 2 + body.y);
    p.rotate(body.angle);
    p.rectMode(p.CENTER);
    p.strokeWeight(2);
    p.stroke(147, 197, 253);
    p.fill(59, 130, 246);
    p.rect(0, 0, body.width, body.height, 8);

    p.stroke(254, 240, 138);
    p.line(0, 0, body.width * 0.36, 0);
    p.fill(254, 240, 138);
    p.noStroke();
    p.circle(body.width * 0.36, 0, 8);
    p.pop();
  }

  function drawOverlay() {
    const m = metrics();

    p.push();
    p.noStroke();
    p.fill(15, 23, 42, 220);
    p.rect(16, 16, 490, 280, 8);
    p.fill(229, 231, 235);
    p.textSize(14);
    p.textAlign(p.LEFT, p.TOP);
    p.text("Task 1: 2D rectangular rigid body", 32, 30);
    p.text(`Mode: ${MODES[modeIndex].label}`, 32, 56);
    p.text("Strictly 2D: angular momentum is the z scalar", 32, 80);
    p.text("Press M to switch mode, R to reset", 32, 104);
    p.text(`I body x/y/z: ${body.inertiaX.toFixed(2)} / ${body.inertiaY.toFixed(2)} / ${body.inertiaZ.toFixed(2)}`, 32, 138);
    p.text(`L world z initial: ${body.initialAngularMomentum.toFixed(4)}`, 32, 168);
    p.text(`L world z current: ${m.worldL.toFixed(4)}`, 32, 192);
    p.text(`L local z initial: ${body.initialAngularMomentum.toFixed(4)}`, 32, 216);
    p.text(`L local z current: ${m.localL.toFixed(4)}`, 32, 240);
    p.text(`Energy: ${m.energy.toFixed(4)} | initial ${body.initialEnergy.toFixed(4)}`, 32, 264);
    p.pop();
  }
}
