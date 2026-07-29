import { TAU } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import type { Ball } from '../physics/types';
import {
  createBall,
  createBallRuntime,
  launchBall,
  stepWorld,
  type SimEvent,
  type World,
} from '../physics/world';
import { BALL_RADIUS, compileLevel, levelMaxPower, type LevelDef } from './level';

export interface Shot {
  /** Radians. */
  angle: number;
  /** 0..1, scaled by the level's maximum launch speed. */
  power: number;
}

export interface SolveOptions {
  /** Aim directions sampled per node. */
  angleSamples: number;
  /** Power levels sampled per node, spread over `minPower`..1. */
  powerSamples: number;
  minPower: number;
  /** Maximum strokes to search. */
  maxStrokes: number;
  /** Simulated seconds allowed per shot. */
  shotTime: number;
  /** Simulation step used while searching. Coarser than gameplay, for speed. */
  timeStep: number;
  /** Rest positions within this distance of each other are treated as one. */
  mergeRadius: number;
  /** Nodes carried into the next stroke, best-first by distance to the hole. */
  beamWidth: number;
}

export const DEFAULT_SOLVE_OPTIONS: SolveOptions = {
  angleSamples: 72,
  powerSamples: 5,
  minPower: 0.25,
  maxStrokes: 3,
  shotTime: 12,
  timeStep: 1 / 120,
  mergeRadius: 45,
  beamWidth: 6,
};

export interface SolveResult {
  solved: boolean;
  /** Strokes used by the shortest solution found, or 0 when unsolved. */
  strokes: number;
  shots: Shot[];
  /** Closest the search ever got to the hole, in world units. */
  closestApproach: number;
  /** Total shots simulated — useful for tuning search cost. */
  simulated: number;
  /**
   * Ids of every star touched by any shot the search tried, whether or not
   * that shot was part of a solution. Used to prove no star is stranded.
   */
  starsSeen: Set<string>;
}

type ShotOutcome = 'sink' | 'rest' | 'death' | 'timeout';

interface ShotSim {
  outcome: ShotOutcome;
  position: Vec2;
  time: number;
  closest: number;
  /** Ids of stars this shot passed through. */
  stars: string[];
}

/** Simulates one shot from a resting ball and reports where it ends up. */
const simulateShot = (
  world: World,
  from: Vec2,
  startTime: number,
  impulse: Vec2,
  opts: SolveOptions,
): ShotSim => {
  const sandbox: World = {
    ...world,
    // A fresh copy each shot, so one shot's pickups do not hide a star from
    // the next.
    collectibles: world.collectibles.map((c) => ({ ...c, collected: false })),
    config: { ...world.config, timeStep: opts.timeStep },
    time: startTime,
    shapeCache: undefined,
  };
  const ball: Ball = createBall(from, BALL_RADIUS);
  const runtime = createBallRuntime();
  launchBall(ball, runtime, impulse);

  const events: SimEvent[] = [];
  const steps = Math.round(opts.shotTime / opts.timeStep);
  let closest = V.distance(from, world.hole.position);
  const stars: string[] = [];

  const finish = (outcome: ShotOutcome, closestDistance: number): ShotSim => ({
    outcome,
    position: ball.position,
    time: sandbox.time,
    closest: closestDistance,
    stars,
  });

  for (let i = 0; i < steps; i++) {
    events.length = 0;
    stepWorld(sandbox, ball, runtime, events);
    for (const event of events) if (event.type === 'collect') stars.push(event.id);
    closest = Math.min(closest, V.distance(ball.position, world.hole.position));
    if (runtime.sunk) return finish('sink', 0);
    if (!runtime.alive) return finish('death', closest);
    if (ball.atRest) return finish('rest', closest);
  }
  return finish('timeout', closest);
};

interface SearchNode {
  position: Vec2;
  time: number;
  shots: Shot[];
}

/**
 * Breadth-first beam search over shots, used to prove every shipped hole is
 * actually completable within a sane stroke count. It is a validation tool, not
 * an in-game hint system: it plays far more shots than a person would.
 */
export const solveLevel = (
  level: LevelDef,
  overrides: Partial<SolveOptions> = {},
): SolveResult => {
  const opts = { ...DEFAULT_SOLVE_OPTIONS, ...overrides };
  const world = compileLevel(level);
  const maxPower = levelMaxPower(level);

  let frontier: SearchNode[] = [{ position: level.tee, time: 0, shots: [] }];
  let closestApproach = V.distance(level.tee, world.hole.position);
  let simulated = 0;
  const starsSeen = new Set<string>();

  for (let stroke = 1; stroke <= opts.maxStrokes; stroke++) {
    const candidates: Array<SearchNode & { score: number }> = [];

    for (const node of frontier) {
      for (let a = 0; a < opts.angleSamples; a++) {
        const angle = (a / opts.angleSamples) * TAU;
        for (let p = 0; p < opts.powerSamples; p++) {
          const power =
            opts.powerSamples === 1
              ? 1
              : opts.minPower + ((1 - opts.minPower) * p) / (opts.powerSamples - 1);
          const impulse = V.fromAngle(angle, power * maxPower);
          const sim = simulateShot(world, node.position, node.time, impulse, opts);
          simulated++;
          closestApproach = Math.min(closestApproach, sim.closest);
          for (const id of sim.stars) starsSeen.add(id);

          const shots = [...node.shots, { angle, power }];
          if (sim.outcome === 'sink') {
            return { solved: true, strokes: stroke, shots, closestApproach: 0, simulated, starsSeen };
          }
          if (sim.outcome === 'rest') {
            candidates.push({
              position: sim.position,
              time: sim.time,
              shots,
              score: V.distance(sim.position, world.hole.position),
            });
          }
        }
      }
    }

    // Merge nearby rest positions, then keep the most promising few.
    candidates.sort((x, y) => x.score - y.score);
    const kept: Array<SearchNode & { score: number }> = [];
    for (const candidate of candidates) {
      if (kept.length >= opts.beamWidth) break;
      const duplicate = kept.some(
        (k) => V.distance(k.position, candidate.position) < opts.mergeRadius,
      );
      if (!duplicate) kept.push(candidate);
    }
    if (kept.length === 0) break;
    frontier = kept;
  }

  return { solved: false, strokes: 0, shots: [], closestApproach, simulated, starsSeen };
};

/**
 * Every star id the search can reach within `maxStrokes`.
 *
 * Chapters unlock on star count, so a star nobody can collect is not a missed
 * bonus — it is a hole in the progression that can strand a player short of
 * the next chapter.
 */
export const reachableStars = (
  level: LevelDef,
  overrides: Partial<SolveOptions> = {},
): Set<string> => {
  // Stop at the first sink so the search keeps exploring instead of returning
  // as soon as it finds a solution.
  const world = compileLevel(level);
  const opts = { ...DEFAULT_SOLVE_OPTIONS, ...overrides };
  const seen = new Set<string>();
  const maxPower = levelMaxPower(level);

  let frontier: Array<{ position: Vec2; time: number }> = [{ position: level.tee, time: 0 }];

  for (let stroke = 1; stroke <= opts.maxStrokes; stroke++) {
    const rests: Array<{ position: Vec2; time: number; score: number }> = [];

    for (const node of frontier) {
      for (let a = 0; a < opts.angleSamples; a++) {
        const angle = (a / opts.angleSamples) * TAU;
        for (let p = 0; p < opts.powerSamples; p++) {
          const power =
            opts.powerSamples === 1
              ? 1
              : opts.minPower + ((1 - opts.minPower) * p) / (opts.powerSamples - 1);
          const sim = simulateShot(
            world,
            node.position,
            node.time,
            V.fromAngle(angle, power * maxPower),
            opts,
          );
          for (const id of sim.stars) seen.add(id);
          if (sim.outcome === 'rest') {
            rests.push({ position: sim.position, time: sim.time, score: -sim.stars.length });
          }
        }
      }
    }

    if (seen.size >= world.collectibles.length) break;

    // Spread out: prefer rest positions far from each other, so later strokes
    // explore new parts of the hole rather than crowding one corner.
    rests.sort((x, y) => x.score - y.score);
    const kept: typeof rests = [];
    for (const candidate of rests) {
      if (kept.length >= opts.beamWidth) break;
      if (!kept.some((k) => V.distance(k.position, candidate.position) < opts.mergeRadius)) {
        kept.push(candidate);
      }
    }
    if (kept.length === 0) break;
    frontier = kept;
  }

  return seen;
};
