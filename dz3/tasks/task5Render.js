export function drawTask5(p, state) {
  const view = makeView(p, state.bounds);
  drawContainer(p, state.bounds, view);

  for (const body of state.bodies) {
    if (body.isStatic) continue;
    drawBody(p, body, view);
  }

  drawDrag(p, state, view);
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

function drawBody(p, body, view) {
  p.push();
  p.stroke(196, 181, 253, 190);
  p.strokeWeight(Math.max(0.5, view.scale));
  p.fill(body.kind === "circle" ? [244, 114, 182, 150] : [124, 58, 237, 145]);

  if (body.kind === "circle") {
    const c = toScreen(view, body);
    p.circle(c.x, c.y, body.radius * 2 * view.scale);
  } else if (body.kind === "capsule") {
    drawCapsule(p, body, view);
  } else {
    p.beginShape();
    for (const vertex of body.cache.vertices) {
      const s = toScreen(view, vertex);
      p.vertex(s.x, s.y);
    }
    p.endShape(p.CLOSE);
  }

  p.pop();
}

function drawCapsule(p, body, view) {
  const axis = { x: Math.cos(body.angle), y: Math.sin(body.angle) };
  const a = toScreen(view, {
    x: body.x - axis.x * body.length * 0.5,
    y: body.y - axis.y * body.length * 0.5,
  });
  const b = toScreen(view, {
    x: body.x + axis.x * body.length * 0.5,
    y: body.y + axis.y * body.length * 0.5,
  });

  p.strokeWeight(body.radius * 2 * view.scale);
  p.line(a.x, a.y, b.x, b.y);
  p.noStroke();
  p.circle(a.x, a.y, body.radius * 2 * view.scale);
  p.circle(b.x, b.y, body.radius * 2 * view.scale);
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
  p.stroke(168, 85, 247, 150);
  p.strokeWeight(1.5);
  p.rect(topLeft.x + 7, topLeft.y + 7, width - 14, height - 14, 11);
  p.pop();
}

function drawDrag(p, state, view) {
  if (!state.drag) return;
  const body = state.bodies[state.drag.bodyId];
  if (!body) return;

  const center = toScreen(view, body);
  const target = toScreen(view, { x: state.drag.targetX, y: state.drag.targetY });

  p.push();
  p.stroke(34, 211, 238, 210);
  p.strokeWeight(2);
  p.line(center.x, center.y, target.x, target.y);
  p.noStroke();
  p.fill(34, 211, 238);
  p.circle(target.x, target.y, 9);
  p.pop();
}

function drawOverlay(p, state) {
  p.push();
  p.noStroke();
  p.fill(15, 23, 42, 225);
  p.rect(16, 16, 510, 390, 8);
  p.fill(229, 231, 235);
  p.textSize(14);
  p.textAlign(p.LEFT, p.TOP);
  p.text("Extra 2D: GJK + EPA + Plane Clipping", 32, 30);
  p.text("Shapes: spheres, capsules, polygons", 32, 56);
  p.text("Pyramid stability: XPBD + friction", 32, 82);
  p.text("GJK support mapping for all shape types", 32, 112);
  p.text(`GJK tests / EPA calls: ${state.gjkTests} / ${state.epaCalls}`, 32, 136);
  p.text(`plane clipped points: ${state.clippedContacts}`, 32, 160);
  p.text(`contacts: ${state.contacts.length} | solves: ${state.constraintSolves}`, 32, 184);
  p.text(`static / dynamic friction: ${state.staticFrictionContacts} / ${state.dynamicFrictionContacts}`, 32, 208);
  p.text(`sleeping bodies: ${state.sleepingCount}`, 32, 232);
  p.text(`avg / max speed: ${state.avgSpeed.toFixed(2)} / ${state.maxSpeed.toFixed(2)}`, 32, 256);
  p.text(`substeps / iterations: ${state.substeps} / ${state.iterations}`, 32, 280);
  p.text(`Mouse: drag any body | R: reset`, 32, 312);
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
