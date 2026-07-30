import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import type { Ball } from './types';
import {
  createBallRuntime,
  stepWorld,
  type SimEvent,
  type World,
} from './world';

export type TrajectoryOutcome = 'sink' | 'death' | 'rest' | 'truncated';

export interface Trajectory {
  /** Sampled ball centres, starting at the launch point. */
  points: Vec2[];
  outcome: TrajectoryOutcome;
  /** Simulated seconds covered by `points`. */
  duration: number;
}

export interface TrajectoryOptions {
  /** Simulated seconds to look ahead. */
  maxTime: number;
  /** Simulation step for the preview. Coarser than the live step, on purpose. */
  timeStep: number;
  /** One point is emitted every this many steps. */
  sampleEvery: number;
}

export const DEFAULT_TRAJECTORY_OPTIONS: TrajectoryOptions = {
  maxTime: 1.4,
  timeStep: 1 / 120,
  sampleEvery: 3,
};

/**
 * Runs the real integrator forward on a throwaway copy of the world, so the
 * aiming preview can never diverge from the physics the player actually gets.
 * The world and ball passed in are left untouched.
 */
export const predictTrajectory = (
  world: World,
  ball: Ball,
  impulse: Vec2,
  options: Partial<TrajectoryOptions> = {},
): Trajectory => {
  const opts = { ...DEFAULT_TRAJECTORY_OPTIONS, ...options };

  const sandbox: World = {
    ...world,
    // Every mutable member is copied, so previewing a shot cannot bank the
    // stars it would pass through, throw a switch or shatter a block.
    collectibles: world.collectibles.map((c) => ({ ...c })),
    switches: world.switches.map((sw) => ({ ...sw })),
    breakables: { ...world.breakables },
    shapeCache: undefined,
    config: { ...world.config, timeStep: opts.timeStep },
  };

  const probe: Ball = { ...ball, velocity: impulse, atRest: false, restTimer: 0 };
  const runtime = createBallRuntime();
  const events: SimEvent[] = [];

  const points: Vec2[] = [probe.position];
  const totalSteps = Math.max(1, Math.round(opts.maxTime / opts.timeStep));
  let outcome: TrajectoryOutcome = 'truncated';
  let steps = 0;

  for (let i = 0; i < totalSteps; i++) {
    events.length = 0;
    stepWorld(sandbox, probe, runtime, events);
    steps++;

    if (runtime.sunk) {
      points.push(probe.position);
      outcome = 'sink';
      break;
    }
    if (!runtime.alive) {
      points.push(probe.position);
      outcome = 'death';
      break;
    }
    if (probe.atRest) {
      points.push(probe.position);
      outcome = 'rest';
      break;
    }
    if (i % opts.sampleEvery === 0) points.push(probe.position);
  }

  // Always finish on the true endpoint, so the drawn line does not stop short
  // of where the ball actually gets to.
  const last = points[points.length - 1];
  if (!last || last.x !== probe.position.x || last.y !== probe.position.y) {
    points.push(probe.position);
  }

  return { points, outcome, duration: steps * opts.timeStep };
};

/** Total path length of a sampled trajectory, in world units. */
export const trajectoryLength = (trajectory: Trajectory): number => {
  let total = 0;
  for (let i = 1; i < trajectory.points.length; i++) {
    total += V.distance(trajectory.points[i - 1]!, trajectory.points[i]!);
  }
  return total;
};
