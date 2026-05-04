import {
  createTask4State,
  cycleTask4Broadphase,
  stepTask4,
} from "./task4Physics.js";
import { drawTask4 } from "./task4Render.js";

export function createTask4(p) {
  let state;

  return {
    init() {
      state = createTask4State(state?.broadphaseIndex ?? 0);
    },

    update(dt) {
      stepTask4(state, dt);
    },

    render() {
      drawTask4(p, state);
    },

    reset() {
      this.init();
    },

    keyPressed(key) {
      if (key.toLowerCase() === "r") this.reset();
      if (key.toLowerCase() === "b") cycleTask4Broadphase(state);
    },
  };
}
