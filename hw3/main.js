import { SceneManager } from "./sceneManager.js";
import { createTask1 } from "./tasks/task1.js";
import { createTask2 } from "./tasks/task2.js";
import { createTask3 } from "./tasks/task3.js";
import { createTask4 } from "./tasks/task4.js";
import { createTask5 } from "./tasks/task5.js";

const FIXED_DT = 1 / 120;

new p5((p) => {
  let sceneManager;
  let accumulator = 0;
  let lastTime = 0;

  p.setup = () => {
    p.createCanvas(window.innerWidth, window.innerHeight);
    p.angleMode(p.RADIANS);
    p.textFont("monospace");

    sceneManager = new SceneManager();
    sceneManager.addScene("task1", createTask1(p));
    sceneManager.addScene("task2", createTask2(p));
    sceneManager.addScene("task3", createTask3(p));
    sceneManager.addScene("task4", createTask4(p));
    sceneManager.addScene("task5", createTask5(p));
    sceneManager.setScene("task1");

    lastTime = p.millis() / 1000;
  };

  p.draw = () => {
    const now = p.millis() / 1000;
    const frameDt = Math.min(now - lastTime, 0.05);
    lastTime = now;
    accumulator += frameDt;

    while (accumulator >= FIXED_DT) {
      sceneManager.update(FIXED_DT);
      accumulator -= FIXED_DT;
    }

    p.background(17, 24, 39);
    sceneManager.render();
    drawSceneControls(p, sceneManager.currentName);
  };

  p.keyPressed = () => {
    if (p.key === "1") sceneManager.setScene("task1");
    if (p.key === "2") sceneManager.setScene("task2");
    if (p.key === "3") sceneManager.setScene("task3");
    if (p.key === "4") sceneManager.setScene("task4");
    if (p.key === "5") sceneManager.setScene("task5");
    sceneManager.keyPressed(p.key);
  };

  p.mousePressed = () => {
    sceneManager.mousePressed(p.mouseX, p.mouseY);
  };

  p.mouseDragged = () => {
    sceneManager.mouseDragged(p.mouseX, p.mouseY);
  };

  p.mouseReleased = () => {
    sceneManager.mouseReleased(p.mouseX, p.mouseY);
  };

  p.windowResized = () => {
    p.resizeCanvas(window.innerWidth, window.innerHeight);
  };
});

function drawSceneControls(p, currentName) {
  p.push();
  p.noStroke();
  p.fill(229, 231, 235);
  p.textSize(14);
  p.textAlign(p.LEFT, p.BOTTOM);
  p.text(`Scene: ${currentName} | Press 1, 2, 3, 4, 5 to switch`, 16, p.height - 16);
  p.pop();
}
