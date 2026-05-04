import { beginTask5Drag, createTask5State, endTask5Drag, stepTask5, updateTask5Drag } from "./task5Physics.js";
import { drawTask5 } from "./task5Render.js";

export function createTask5(p) {
  let state;

  return {
    init() {
      state = createTask5State();
    },

    update(dt) {
      stepTask5(state, dt);
    },

    render() {
      drawTask5(p, state);
    },

    reset() {
      this.init();
    },

    keyPressed(key) {
      if (key.toLowerCase() === "r") this.reset();
    },

    mousePressed(x, y) {
      beginTask5Drag(state, ...toWorld(p, state, x, y));
    },

    mouseDragged(x, y) {
      updateTask5Drag(state, ...toWorld(p, state, x, y));
    },

    mouseReleased() {
      endTask5Drag(state);
    },
  };
}

function toWorld(p, state, x, y) {
  const marginX = 560;
  const marginY = 70;
  const availableW = Math.max(320, p.width - marginX);
  const availableH = Math.max(260, p.height - marginY);
  const scale = Math.min(1, availableW / (state.bounds.x * 2), availableH / (state.bounds.y * 2));
  const cx = p.width / 2 + 190;
  const cy = p.height / 2;

  return [(x - cx) / scale, (y - cy) / scale];
}
