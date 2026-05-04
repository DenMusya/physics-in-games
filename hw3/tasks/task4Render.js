export function drawTask4(p, state) {
  const view = makeView(p, state.bounds);
  drawContainer(p, state.bounds, view);
  drawGrid(p, state.bounds, view);

  for (const body of state.bodies) {
    if (body.isStatic) continue;
    drawBody(p, body, view);
  }

  drawFrame(p, state.bounds, view);
  drawOverlay(p, state);
}

function drawContainer(p, bounds, view) {
  const topLeft = toScreen(view, { x: -bounds.x, y: -bounds.y });
  const width = bounds.x * 2 * view.scale;
  const height = bounds.y * 2 * view.scale;

  p.push();
  p.noStroke();
  p.fill(2, 6, 23, 180);
  p.rect(topLeft.x, topLeft.y, width, height, 16);
  p.fill(15, 23, 42, 220);
  p.rect(topLeft.x + 8, topLeft.y + 8, width - 16, height - 16, 12);
  p.pop();
}

function drawGrid(p, bounds, view) {
  const step = 100;

  p.push();
  p.stroke(51, 65, 85, 110);
  p.strokeWeight(1);
  for (let x = -bounds.x; x <= bounds.x; x += step) {
    const a = toScreen(view, { x, y: -bounds.y });
    const b = toScreen(view, { x, y: bounds.y });
    p.line(a.x, a.y, b.x, b.y);
  }
  for (let y = -bounds.y; y <= bounds.y; y += step) {
    const a = toScreen(view, { x: -bounds.x, y });
    const b = toScreen(view, { x: bounds.x, y });
    p.line(a.x, a.y, b.x, b.y);
  }
  p.pop();
}

function drawBody(p, body, view) {
  const c = body.cache.corners;
  const a = toScreen(view, c[0]);
  const b = toScreen(view, c[1]);
  const d = toScreen(view, c[2]);
  const e = toScreen(view, c[3]);
  const area = body.width * body.height;
  const alpha = area > 2500 ? 185 : 135;

  p.push();
  p.stroke(147, 197, 253, 180);
  p.strokeWeight(Math.max(0.5, view.scale));
  p.fill(area > 2500 ? [14, 165, 233, alpha] : [37, 99, 235, alpha]);
  p.quad(a.x, a.y, b.x, b.y, d.x, d.y, e.x, e.y);
  p.pop();
}

function drawFrame(p, bounds, view) {
  const topLeft = toScreen(view, { x: -bounds.x, y: -bounds.y });
  const width = bounds.x * 2 * view.scale;
  const height = bounds.y * 2 * view.scale;

  p.push();
  p.noFill();
  p.stroke(226, 232, 240);
  p.strokeWeight(4);
  p.rect(topLeft.x, topLeft.y, width, height, 16);
  p.stroke(16, 185, 129, 150);
  p.strokeWeight(1.5);
  p.rect(topLeft.x + 7, topLeft.y + 7, width - 14, height - 14, 11);
  p.pop();
}

function drawOverlay(p, state) {
  p.push();
  p.noStroke();
  p.fill(15, 23, 42, 225);
  p.rect(16, 16, 510, 410, 8);
  p.fill(229, 231, 235);
  p.textSize(14);
  p.textAlign(p.LEFT, p.TOP);
  p.text("Task 4: varied rigid bodies + advanced broadphase", 32, 30);
  p.text("B: Sweep and Prune / LBVH, R: reset", 32, 56);
  p.text(`broadphase: ${state.broadphase}`, 32, 88);
  p.text("solver: XPBD position constraints", 32, 112);
  p.text(`mu static / dynamic: ${state.muStatic.toFixed(2)} / ${state.muDynamic.toFixed(2)}`, 32, 136);
  p.text(`static / dynamic friction contacts: ${state.staticFrictionContacts} / ${state.dynamicFrictionContacts}`, 32, 160);
  p.text(`bodies: ${state.bodies.length - 4} dynamic | sleeping: ${state.sleepingCount}`, 32, 192);
  p.text(`pairs tested: ${state.pairsTested} | candidates: ${state.candidatePairs.length}`, 32, 216);
  p.text(`SAT calls: ${state.satCalls} | contacts: ${state.contacts.length}`, 32, 240);
  p.text(`constraint solves: ${state.constraintSolves}`, 32, 264);
  p.text(`avg penetration: ${state.avgPenetration.toFixed(3)}`, 32, 288);
  p.text(`avg / max speed: ${state.avgSpeed.toFixed(2)} / ${state.maxSpeed.toFixed(2)}`, 32, 312);
  p.text(`substeps / iterations: ${state.substeps} / ${state.iterations}`, 32, 336);
  p.text(`t: ${state.elapsed.toFixed(2)}s`, 32, 360);
  p.pop();
}

function makeView(p, bounds) {
  const marginX = 560;
  const marginY = 70;
  const availableW = Math.max(320, p.width - marginX);
  const availableH = Math.max(260, p.height - marginY);
  const scale = Math.min(1, availableW / (bounds.x * 2), availableH / (bounds.y * 2));

  return {
    cx: p.width / 2 + 190,
    cy: p.height / 2,
    scale,
  };
}

function toScreen(view, v) {
  return {
    x: view.cx + v.x * view.scale,
    y: view.cy + v.y * view.scale,
  };
}
