export function drawSpringBodyExperiment(p, state, metrics) {
  const { body, spring } = state;
  const anchor = toScreen(p, spring.anchor);
  const attach = toScreen(p, metrics.attachWorld);
  const center = toScreen(p, body.pos);

  p.push();
  p.stroke(55, 65, 92);
  p.strokeWeight(1);
  for (let x = -400; x <= 400; x += 40) {
    const a = toScreen(p, { x, y: -260 });
    const b = toScreen(p, { x, y: 260 });
    p.line(a.x, a.y, b.x, b.y);
  }
  for (let y = -240; y <= 240; y += 40) {
    const a = toScreen(p, { x: -420, y });
    const b = toScreen(p, { x: 420, y });
    p.line(a.x, a.y, b.x, b.y);
  }
  p.pop();

  p.push();
  p.stroke(250, 204, 21);
  p.strokeWeight(3);
  p.line(anchor.x, anchor.y, attach.x, attach.y);
  p.noStroke();
  p.fill(250, 204, 21);
  p.circle(anchor.x, anchor.y, 12);
  p.circle(attach.x, attach.y, 9);
  p.pop();

  p.push();
  p.translate(center.x, center.y);
  p.rotate(body.angle);
  p.rectMode(p.CENTER);
  p.stroke(147, 197, 253);
  p.strokeWeight(2);
  p.fill(37, 99, 235);
  p.rect(0, 0, body.width, body.height, 8);
  p.noStroke();
  p.fill(254, 240, 138);
  p.circle(body.attachLocal.x, body.attachLocal.y, 8);
  p.pop();

  drawOverlay(p, state, metrics);
}

function drawOverlay(p, state, metrics) {
  const { body, spring } = state;

  p.push();
  p.noStroke();
  p.fill(15, 23, 42, 220);
  p.rect(16, 16, 470, 340, 8);
  p.fill(229, 231, 235);
  p.textSize(14);
  p.textAlign(p.LEFT, p.TOP);
  p.text(metrics.title, 32, 30);
  const formula =
    metrics.kind === "projection"
      ? "velocity damping + clamped position projection"
      : "F = -k C n - c dot(v_attach, n) n";
  p.text(formula, 32, 56);
  p.text("Symplectic Euler: v/omega first, then x/angle", 32, 82);
  p.text("Press M to switch baseline, R to reset", 32, 108);
  p.text(`attach local: (${body.attachLocal.x.toFixed(1)}, ${body.attachLocal.y.toFixed(1)})`, 32, 136);
  p.text(`k/c: ${metrics.stiffness.toFixed(2)} / ${metrics.damping.toFixed(2)}`, 32, 160);
  p.text(`effective mass check: ${metrics.effectiveMass.toFixed(4)}`, 32, 184);
  p.text(`spring length: ${metrics.springLength.toFixed(2)} / rest ${spring.restLength.toFixed(2)}`, 32, 208);
  p.text(`stretch: ${metrics.stretch.toFixed(2)}`, 32, 232);
  p.text(`force magnitude: ${metrics.forceMag.toFixed(2)}`, 32, 256);
  p.text(`kinetic energy: ${metrics.kineticEnergy.toFixed(2)}`, 32, 280);
  p.text(`total energy: ${(metrics.kineticEnergy + metrics.potentialEnergy).toFixed(2)}`, 32, 304);
  if (metrics.kind === "projection") {
    p.text(`clamped correction: ${metrics.correction.toFixed(3)}`, 32, 328);
  }
  p.pop();
}

function toScreen(p, v) {
  return {
    x: p.width / 2 + v.x,
    y: p.height / 2 + v.y,
  };
}
