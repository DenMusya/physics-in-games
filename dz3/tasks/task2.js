import {
  createProjectedSpringState,
  createSpringBodyState,
  springBodyMetrics,
  stepProjectedSpring,
  stepSpringBody,
} from "./task2Physics.js";
import { drawSpringBodyExperiment } from "./task2Render.js";

const MODES = [
  {
    create: createSpringBodyState,
    step: stepSpringBody,
  },
  {
    create: createProjectedSpringState,
    step: stepProjectedSpring,
  },
];

export function createTask2(p) {
  let modeIndex = 0;
  let state;

  return {
    init() {
      state = MODES[modeIndex].create();
    },

    update(dt) {
      MODES[modeIndex].step(state, dt);
    },

    render() {
      drawSpringBodyExperiment(p, state, springBodyMetrics(state));
    },

    reset() {
      this.init();
    },

    keyPressed(key) {
      if (key.toLowerCase() === "r") this.reset();
      if (key.toLowerCase() === "m") {
        modeIndex = (modeIndex + 1) % MODES.length;
        this.reset();
      }
    },
  };
}
