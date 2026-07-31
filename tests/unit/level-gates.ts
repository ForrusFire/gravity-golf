import { describe, expect, it } from 'vitest';
import { TAU } from '../../src/core/math';
import * as V from '../../src/core/vec2';
import type { LevelDef } from '../../src/game/level';
import { compileLevel } from '../../src/game/level';
import { ALL_LEVELS } from '../../src/game/levels';
import { PlaySession } from '../../src/game/session';
import { reachableStars, solveLevel, type Shot } from '../../src/game/solver';

/**
 * The per-level verification gates, factored out so they can be run in slices.
 *
 * These are by far the slowest thing in the suite and they grow with every
 * chapter. Vitest parallelises across *files*, not within one, so a single file
 * looping over every level pins one worker at the tail while the others sit
 * idle. Splitting the catalogue lets the pool share the work.
 */
export const SHARDS = 3;

/** Interleaved, not blocked, so every shard gets a mix of cheap and expensive. */
export const shardOf = (index: number): LevelDef[] =>
  ALL_LEVELS.filter((_, i) => i % SHARDS === index);

/** Runs the session until it needs input again, or the hole is finished. */
export const settle = (session: PlaySession, maxSeconds = 40): void => {
  const steps = Math.round(maxSeconds * 60);
  for (let i = 0; i < steps; i++) {
    if (session.state === 'sunk') return;
    if (session.state === 'aiming' && session.canShoot) return;
    session.update(1 / 60);
  }
};

/** Plays a level through a real session using the given shots. */
export const playShots = (session: PlaySession, shots: Shot[]): void => {
  for (const shot of shots) {
    settle(session);
    if (session.state === 'sunk') return;
    session.shoot(V.fromAngle(shot.angle), shot.power);
    settle(session);
  }
  settle(session);
};

/**
 * Every hole in `levels` has a solution within par, and that exact sequence of
 * shots sinks when replayed through a real session.
 *
 * This is what makes "every hole is completable" mean anything. The solver
 * shares the game's integrator *and* its timestep, and starts from the same
 * settled tee, so a solution it finds is a sequence of shots a player could
 * actually take. Gravity slingshots are chaotic: when the solver ran at a
 * coarser step, only half its solutions survived being replayed here.
 */
export const registerReplayGate = (slice: string, levels: LevelDef[]): void => {
  describe(`solver solutions replay through the real game (${slice})`, () => {
    for (const level of levels) {
      it(`${level.id} "${level.name}" sinks when its solution is replayed`, () => {
        // A little wider than the default: the search stands in for a skilled
        // player, and a few holes have a solution a coarser sweep misses.
        const solution = solveLevel(level, {
          angleSamples: 90,
          powerSamples: 6,
          maxStrokes: level.par,
          beamWidth: 6,
        });
        expect(solution.solved, `no solution found within par ${level.par}`).toBe(true);

        const session = new PlaySession(level);
        playShots(session, solution.shots);

        expect(session.state, `replaying ${solution.shots.length} shot(s) did not sink`).toBe(
          'sunk',
        );
        expect(session.result).not.toBeNull();
        expect(session.result!.strokes).toBeLessThanOrEqual(level.par);
      });
    }
  });
};

/**
 * Every star in `levels` can actually be collected.
 *
 * Chapters unlock on star totals, so an uncollectable star is a progression bug,
 * not a missed bonus.
 */
export const registerStarGate = (slice: string, levels: LevelDef[]): void => {
  describe(`star reachability (${slice})`, () => {
    for (const level of levels) {
      it(`${level.id} "${level.name}" has three collectable stars`, () => {
        const world = compileLevel(level);
        const expected = world.collectibles.map((c) => c.id);
        const reached = reachableStars(level, {
          angleSamples: 96,
          powerSamples: 6,
          maxStrokes: Math.max(3, level.par),
          beamWidth: 6,
          // Coverage, not proof: a coarser step samples twice as many shots for
          // the same time, and whether a star is reachable does not hinge on
          // integration precision the way sinking a putt does.
          timeStep: 1 / 120,
        });
        const missing = expected.filter((id) => !reached.has(id));
        expect(missing, `stars never reached by any sampled shot: ${missing.join(', ')}`).toEqual(
          [],
        );
      });
    }
  });
};

const ANGLES = 36;
const POWERS = [0.3, 0.55, 0.8, 1];

/** How often a blind opening shot sinks the hole outright. */
export const sinkRate = (level: LevelDef, angles = ANGLES, powers = POWERS): number => {
  let sinks = 0;
  let total = 0;
  for (let a = 0; a < angles; a++) {
    for (const power of powers) {
      const session = new PlaySession(level);
      // Let the tee settle, then take one shot and see where it ends up.
      for (let i = 0; i < 300 && !session.canShoot; i++) session.update(1 / 60);
      session.shoot(V.fromAngle((a / angles) * TAU), power);
      for (let i = 0; i < 900 && session.state === 'flying'; i++) session.update(1 / 60);
      total++;
      if (session.state === 'sunk') sinks++;
    }
  }
  return sinks / total;
};

/**
 * No hole in `levels` is won by accident.
 *
 * This catches a failure the completability gate cannot see: a hole that is
 * *too* winnable. Three holes shipped with the cup sitting at the natural
 * resting point of the whole course, so anything that settled simply rolled in —
 * one of them scored on a quarter of all blind shots. The solver was perfectly
 * happy, because "can this be finished" was never the question.
 */
export const registerForgivenessGate = (slice: string, levels: LevelDef[]): void => {
  describe(`holes are not won by accident (${slice})`, () => {
    // Chapter 1 is the tutorial and is meant to be generous — the point there is
    // that a reasonable-looking shot works.
    for (const level of levels.filter((l) => l.chapter > 0)) {
      it(`${level.id} "${level.name}" rarely sinks on a blind shot`, () => {
        const rate = sinkRate(level);
        expect(
          rate,
          `${(rate * 100).toFixed(1)}% of blind opening shots sink this hole`,
        ).toBeLessThan(0.07);
      });
    }
  });
};
