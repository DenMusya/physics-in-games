/**
 * p5: холст, ввод, UI; шаг симуляции в physics.js.
 */

let canvasW = 1040;
let canvasH = 620;

function el(id) {
  return document.getElementById(id);
}

function bindRange(id, fmt) {
  const node = el(id);
  const out = el(id + "Val");
  if (!node || !out) return;
  const sync = () => {
    out.textContent = fmt ? fmt(+node.value) : String(node.value);
  };
  node.addEventListener("input", sync);
  sync();
}

function readUi() {
  return {
    method: el("methodSelect") ? el("methodSelect").value : "xpbd",
    iterations: +el("iterations").value,
    substeps: +el("substeps").value,
    complianceEdge: +el("complianceEdge").value,
    complianceVol: +el("complianceVol").value,
    stiffScale: +el("stiffScale").value,
    friction: +el("friction").value / 100,
    restitution: +el("restitution").value / 100,
    gravity: (+el("gravity").value / 100) * 520,
    nVerts: +el("nVerts").value,
    blobRadius: +el("blobRadius").value,
    scene: el("sceneSelect").value,
  };
}

function formatStats(u) {
  const v = sim.verts;
  const A = polygonSignedArea(v);
  const strain = sim.restArea !== 0 ? A / sim.restArea : 1;
  const names = {
    pbd: "PBD",
    xpbd: "XPBD",
    pd: "Projective Dynamics",
    vbd: "VBD",
  };
  const mLabel = names[u.method] || u.method;
  return (
    `Метод: ${mLabel}\n` +
    `Вершин: ${v.length}  Рёбер: ${sim.edges.length}\n` +
    `Площадь: ${A.toFixed(1)} / ${sim.restArea.toFixed(1)} (${(strain * 100).toFixed(2)}%)\n` +
    `Пауза: ${sim.paused ? "да" : "нет"}`
  );
}

function drawScene() {
  background(18, 24, 38);
  stroke(42, 56, 72);
  strokeWeight(2);
  line(0, sim.groundY, canvasW, sim.groundY);

  const v = sim.verts;

  noStroke();
  fill(80, 140, 200, 55);
  beginShape();
  for (const p of v) vertex(p.x, p.y);
  endShape(CLOSE);

  for (const e of sim.edges) {
    const a = v[e.i];
    const b = v[e.j];
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    const t = len / (e.rest || 1);
    let col = color(90, 120, 160, 160);
    if (t > 1.06) col = color(255, 170, 90, 200);
    if (t > 1.15) col = color(255, 100, 100, 210);
    stroke(col);
    strokeWeight(1.5);
    line(a.x, a.y, b.x, b.y);
  }

  for (const p of v) {
    stroke(0, 0, 0, 90);
    strokeWeight(1);
    if (p.pinned) {
      fill(255, 120, 90);
    } else {
      fill(130, 210, 160);
    }
    circle(p.x, p.y, 9);
    if (p.pinned) {
      noFill();
      stroke(255, 255, 255, 200);
      strokeWeight(2);
      circle(p.x, p.y, 12);
    }
  }

  if (sim.grabIndex >= 0) {
    const p = v[sim.grabIndex];
    noFill();
    stroke(255, 255, 255, 180);
    strokeWeight(2);
    drawingContext.setLineDash([6, 4]);
    circle(p.x, p.y, 16);
    drawingContext.setLineDash([]);
  }
}

function setup() {
  const cnv = createCanvas(canvasW, canvasH);
  cnv.parent("p5canvas");

  bindRange("iterations");
  bindRange("substeps");
  bindRange("complianceEdge", (v) => v.toFixed(0));
  bindRange("complianceVol", (v) => v.toFixed(0));
  bindRange("stiffScale", (v) => v.toFixed(0));
  bindRange("friction", (v) => (v / 100).toFixed(2));
  bindRange("restitution", (v) => (v / 100).toFixed(2));
  bindRange("gravity", (v) => (v / 100).toFixed(2));
  bindRange("nVerts");
  bindRange("blobRadius");

  el("btnReset").addEventListener("click", () => resetSimulation(canvasW, canvasH, readUi()));
  el("sceneSelect").addEventListener("change", () => resetSimulation(canvasW, canvasH, readUi()));
  el("methodSelect").addEventListener("change", syncVerletState);

  resetSimulation(canvasW, canvasH, readUi());
  sim.lastT = millis() / 1000;
}

function draw() {
  const now = millis() / 1000;
  let dt = Math.min(0.08, now - sim.lastT);
  sim.lastT = now;

  sim.pointerX = mouseX;
  sim.pointerY = mouseY;

  if (!sim.paused) {
    sim.acc += dt;
    while (sim.acc >= PHYS_FIXED_DT) {
      const u = readUi();
      physicsStepFixedFrame(u, u.substeps);
      sim.acc -= PHYS_FIXED_DT;
    }
  }

  if (vertsHaveNaN()) {
    resetSimulation(canvasW, canvasH, readUi());
  }

  drawScene();

  const st = el("statsText");
  if (st) st.textContent = formatStats(readUi());
}

function mousePressed() {
  if (mouseX < 0 || mouseX > canvasW || mouseY < 0 || mouseY > canvasH) return;
  tryGrab(mouseX, mouseY);
}

function mouseReleased() {
  sim.grabIndex = -1;
}

function touchStarted() {
  if (mouseX >= 0 && mouseX <= canvasW && mouseY >= 0 && mouseY <= canvasH) tryGrab(mouseX, mouseY);
  return false;
}

function touchEnded() {
  sim.grabIndex = -1;
  return false;
}

function keyPressed() {
  if (key === " ") {
    sim.paused = !sim.paused;
    return false;
  }
}
