export function drawCollisionDetection(p, state) {
  const view = makeView(p, state.bounds);
  drawContainer(p, state.bounds, view);
  drawGrid(p, state.bounds, view);

  for (let i = 0; i < state.bodies.length; i += 1) {
    if (state.bodies[i].isStatic) continue;
    drawBody(p, state.bodies[i], view, state.bodyCount >= 1000);
  }

  drawContainerFrame(p, state.bounds, view);
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
  const step = 80;

  p.push();
  p.stroke(51, 65, 85, 120);
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

function drawBody(p, body, view, fastMode) {
  const corners = body.cache.corners;
  const c0 = toScreen(view, corners[0]);
  const c1 = toScreen(view, corners[1]);
  const c2 = toScreen(view, corners[2]);
  const c3 = toScreen(view, corners[3]);

  p.push();
  p.strokeWeight(body.isStatic ? 1.5 : Math.max(0.4, view.scale));
  if (body.isStatic) {
    p.stroke(203, 213, 225);
    p.fill(71, 85, 105, 210);
  } else {
    if (fastMode) p.noStroke();
    else p.stroke(147, 197, 253);
    p.fill(37, 99, 235, stateFillAlpha(view));
  }
  p.quad(c0.x, c0.y, c1.x, c1.y, c2.x, c2.y, c3.x, c3.y);
  p.pop();
}

function drawContainerFrame(p, bounds, view) {
  const topLeft = toScreen(view, { x: -bounds.x, y: -bounds.y });
  const width = bounds.x * 2 * view.scale;
  const height = bounds.y * 2 * view.scale;

  p.push();
  p.noFill();
  p.stroke(226, 232, 240);
  p.strokeWeight(4);
  p.rect(topLeft.x, topLeft.y, width, height, 16);
  p.stroke(59, 130, 246, 130);
  p.strokeWeight(1.5);
  p.rect(topLeft.x + 7, topLeft.y + 7, width - 14, height - 14, 11);
  p.pop();
}

function stateFillAlpha(view) {
  return view.scale < 0.75 ? 185 : 135;
}

function drawOverlay(p, state) {
  p.push();
  p.noStroke();
  p.fill(15, 23, 42, 220);
  p.rect(16, 16, 470, 430, 8);
  p.fill(229, 231, 235);
  p.textSize(14);
  p.textAlign(p.LEFT, p.TOP);
  p.text("Task 3: rectangle rigid body collisions", 32, 30);
  p.text("Pipeline: stable uniform grid -> SAT -> solver", 32, 56);
  p.text("M: solver, N: 10/1000 rectangles, R: reset", 32, 82);
  p.text(`solver: ${state.solver}`, 32, 112);
  p.text(`gravity acceleration: (0, ${state.gravity})`, 32, 136);
  p.text(`cell size / occupied grid cells: ${state.cellSize.toFixed(1)} / ${state.gridCells}`, 32, 160);
  p.text(`avg vy / avg speed: ${state.avgVy.toFixed(2)} / ${state.avgSpeed.toFixed(2)}`, 32, 184);
  p.text(`max speed: ${state.maxSpeed.toFixed(2)} | t: ${state.elapsed.toFixed(2)}s`, 32, 208);
  p.text(`bodies: ${state.bodyCount} dynamic | substeps/iterations: ${state.substeps} / ${state.iterations}`, 32, 232);
  p.text(`candidate pairs: ${state.candidatePairs.length}`, 32, 256);
  p.text(`pairs tested: ${state.pairsTested}`, 32, 280);
  p.text(`SAT calls: ${state.satCalls}`, 32, 304);
  p.text(`constraint solves: ${state.constraintSolves}`, 32, 328);
  p.text(`contacts: ${state.contacts.length} | sleeping: ${state.sleepingCount}`, 32, 352);
  p.text(`avg penetration: ${state.avgPenetration.toFixed(3)}`, 32, 376);
  p.text(`friction / damping: ${state.friction.toFixed(2)} / ${state.linearDamping.toFixed(2)}`, 32, 400);
  p.pop();
}

function makeView(p, bounds) {
  const marginX = 520;
  const marginY = 70;
  const availableW = Math.max(320, p.width - marginX);
  const availableH = Math.max(260, p.height - marginY);
  const scale = Math.min(1, availableW / (bounds.x * 2), availableH / (bounds.y * 2));

  return {
    cx: p.width / 2 + 170,
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
