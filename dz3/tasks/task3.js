import {
  createCollisionDetectionState,
  cycleCollisionSolver,
  stepCollisionDetection,
} from "./task3Physics.js";
import { drawCollisionDetection } from "./task3Render.js";

export function createTask3(p) {
  let state;
  let bodyCount = 10;

  return {
    init() {
      state = createCollisionDetectionState(bodyCount, state?.solverIndex ?? 0);
    },

    update(dt) {
      stepCollisionDetection(state, dt);
    },

    render() {
      drawCollisionDetection(p, state);
    },

    reset() {
      this.init();
    },

    keyPressed(key) {
      if (key.toLowerCase() === "r") this.reset();
      if (key.toLowerCase() === "m") cycleCollisionSolver(state);
      if (key.toLowerCase() === "n") {
        bodyCount = bodyCount === 10 ? 1000 : 10;
        this.reset();
      }
    },
  };
}
