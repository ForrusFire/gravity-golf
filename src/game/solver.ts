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
import { DEFAULT_PHYSICS } from '../physics/types';
import { BALL_RADIUS, compileLevel, levelMaxPower, shotImpulse, type LevelDef } from './level';
import { settledTee } from './session';

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
  /** Simulation step used while searching. Matches gameplay by default. */
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
  // Deliberately the gameplay step, not a cheaper one. Gravity slingshots are
  // chaotic: a coarser step finds solutions that do not survive being replayed
  // at the real step, which would make "this hole is completable" a claim about
  // a simulation nobody plays.
  timeStep: DEFAULT_PHYSICS.timeStep,
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

/**
 * Everything a shot can change about the board and carry into the next stroke.
 *
 * The game persists all of it across a hole, so the search has to as well: a
 * locked cup cannot be opened without banking stars over several strokes, and a
 * gate thrown on stroke one has to still be open on stroke two.
 */
export interface BoardState {
  collected: ReadonlySet<string>;
  switchesOn: ReadonlySet<string>;
  /** World time each held switch springs back at, for switches still counting. */
  switchTimers: Readonly<Record<string, number>>;
  breakables: Readonly<Record<string, number>>;
}

const initialBoardState = (world: World): BoardState => ({
  collected: new Set(world.collectibles.filter((c) => c.collected).map((c) => c.id)),
  switchesOn: new Set(world.switches.filter((s) => s.on).map((s) => s.id)),
  switchTimers: Object.fromEntries(
    world.switches.filter((s) => s.offAt !== undefined).map((s) => [s.id, s.offAt as number]),
  ),
  breakables: { ...world.breakables },
});

interface ShotSim {
  outcome: ShotOutcome;
  position: Vec2;
  time: number;
  closest: number;
  /** Ids of stars this shot passed through. */
  stars: string[];
  /** The board as this shot left it. */
  state: BoardState;
}

/** Simulates one shot from a resting ball and reports where it ends up. */
const simulateShot = (
  world: World,
  from: Vec2,
  startTime: number,
  impulse: Vec2,
  opts: SolveOptions,
  state: BoardState,
): ShotSim => {
  // A fresh copy each shot so sibling branches cannot see each other's changes,
  // seeded from the branch's own accumulated state.
  const sandbox: World = {
    ...world,
    collectibles: world.collectibles.map((c) => ({ ...c, collected: state.collected.has(c.id) })),
    switches: world.switches.map((sw) => ({
      ...sw,
      on: state.switchesOn.has(sw.id),
      offAt: state.switchTimers[sw.id],
    })),
    breakables: { ...state.breakables },
    revision: 0,
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
    state: initialBoardState(sandbox),
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
  state: BoardState;
}

/** Distinguishes two rest positions that differ only in what the board is doing. */
const stateKey = (state: BoardState): string =>
  [
    [...state.collected].sort().join('|'),
    [...state.switchesOn].sort().join('|'),
    // Rounded: two branches whose timers differ by a hundredth of a second are
    // the same node for search purposes, and treating them as distinct would
    // hand every held switch its own reserved beam slot.
    Object.keys(state.switchTimers)
      .sort()
      .map((id) => `${id}:${Math.round((state.switchTimers[id] as number) * 4)}`)
      .join('|'),
    Object.keys(state.breakables)
      .sort()
      .map((id) => `${id}:${state.breakables[id]}`)
      .join('|'),
  ].join('/');

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

  // The game settles the ball before handing over control, so searching from
  // the authored tee would solve a hole that is not the one being played.
  const start = settledTee(level);
  let frontier: SearchNode[] = [
    { position: start, time: 0, shots: [], state: initialBoardState(world) },
  ];
  let closestApproach = V.distance(start, world.hole.position);
  let simulated = 0;
  const starsSeen = new Set<string>();

  const required = world.hole.requiresStars ?? 0;
  /**
   * Distance to the hole, except while the cup is still locked — then the stars
   * are the objective and parking next to a cup that will not open is no
   * progress at all.
   */
  const scoreNode = (position: Vec2, state: BoardState): number => {
    const held = state.collected.size;
    if (required > 0 && held < required) {
      let nearest = Infinity;
      for (const item of world.collectibles) {
        if (state.collected.has(item.id)) continue;
        nearest = Math.min(nearest, V.distance(position, item.position));
      }
      return (required - held) * 100000 + (Number.isFinite(nearest) ? nearest : 0);
    }
    return V.distance(position, world.hole.position);
  };

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
          // Identical arithmetic to PlaySession.shoot, so the shots this
          // search returns are bit-for-bit the shots a player would take.
          const impulse = shotImpulse(V.fromAngle(angle), power, maxPower);
          const sim = simulateShot(world, node.position, node.time, impulse, opts, node.state);
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
              state: sim.state,
              score: scoreNode(sim.position, sim.state),
            });
          }
        }
      }
    }

    // Merge nearby rest positions, then keep the most promising few.
    candidates.sort((x, y) => x.score - y.score);
    const kept: Array<SearchNode & { score: number }> = [];
    const seenStates = new Set<string>();
    const take = (candidate: (typeof candidates)[number]): void => {
      kept.push(candidate);
      seenStates.add(stateKey(candidate.state));
    };
    for (const candidate of candidates) {
      if (kept.length >= opts.beamWidth) break;
      // Two balls at the same spot are only the same node if the board around
      // them matches — one may have a gate open that the other does not.
      const key = stateKey(candidate.state);
      const duplicate = kept.some(
        (k) =>
          stateKey(k.state) === key &&
          V.distance(k.position, candidate.position) < opts.mergeRadius,
      );
      if (!duplicate) take(candidate);
    }
    // Reserve room for board states the distance-ranked beam crowded out. A shot
    // that throws a switch usually parks the ball somewhere worse, so without
    // this the search would keep discarding the only branch that can finish.
    for (const candidate of candidates) {
      if (kept.length >= opts.beamWidth * 2) break;
      if (!seenStates.has(stateKey(candidate.state))) take(candidate);
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

  const base = initialBoardState(world);
  /**
   * Machine state carries forward (a gate opened on stroke one stays open), but
   * pickups are cleared every shot: this asks whether each star is *reachable*,
   * so having already banked one must not hide it from a later branch.
   */
  const carry = (state: BoardState): BoardState => ({ ...state, collected: new Set<string>() });

  let frontier: Array<{ position: Vec2; time: number; state: BoardState }> = [
    { position: settledTee(level), time: 0, state: carry(base) },
  ];

  for (let stroke = 1; stroke <= opts.maxStrokes; stroke++) {
    const rests: Array<{ position: Vec2; time: number; state: BoardState; score: number }> = [];

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
            shotImpulse(V.fromAngle(angle), power, maxPower),
            opts,
            node.state,
          );
          for (const id of sim.stars) seen.add(id);
          if (sim.outcome === 'rest') {
            rests.push({
              position: sim.position,
              time: sim.time,
              state: carry(sim.state),
              score: -sim.stars.length,
            });
          }
        }
      }
    }

    if (seen.size >= world.collectibles.length) break;

    // Spread out: prefer rest positions far from each other, so later strokes
    // explore new parts of the hole rather than crowding one corner.
    rests.sort((x, y) => x.score - y.score);
    const kept: typeof rests = [];
    const seenStates = new Set<string>();
    for (const candidate of rests) {
      if (kept.length >= opts.beamWidth) break;
      const key = stateKey(candidate.state);
      if (
        !kept.some(
          (k) =>
            stateKey(k.state) === key &&
            V.distance(k.position, candidate.position) < opts.mergeRadius,
        )
      ) {
        kept.push(candidate);
        seenStates.add(key);
      }
    }
    // Same reservation as the solver: a branch that opened something up must
    // survive the cut even when it parked the ball in a worse place.
    for (const candidate of rests) {
      if (kept.length >= opts.beamWidth * 2) break;
      const key = stateKey(candidate.state);
      if (!seenStates.has(key)) {
        kept.push(candidate);
        seenStates.add(key);
      }
    }
    if (kept.length === 0) break;
    frontier = kept;
  }

  return seen;
};
