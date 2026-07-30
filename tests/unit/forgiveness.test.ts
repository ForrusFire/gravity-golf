import { describe, expect, it } from 'vitest';
import { TAU } from '../../src/core/math';
import * as V from '../../src/core/vec2';
import { ALL_LEVELS } from '../../src/game/levels';
import { PlaySession } from '../../src/game/session';

/**
 * How often a blind opening shot sinks the hole outright.
 *
 * This catches a failure the completability suite cannot see: a hole that is
 * *too* winnable. Three holes shipped with the cup sitting at the natural
 * resting point of the whole course, so anything that settled simply rolled in —
 * one of them scored on a quarter of all blind shots. The solver was perfectly
 * happy, because "can this be finished" was never the question.
 */
const sinkRate = (levelId: string, angles: number, powers: number[]): number => {
  const level = ALL_LEVELS.find((l) => l.id === levelId)!;
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

const ANGLES = 36;
const POWERS = [0.3, 0.55, 0.8, 1];

describe('holes are not won by accident', () => {
  // Chapter 1 is the tutorial and is meant to be generous — the point there is
  // that a reasonable-looking shot works.
  for (const level of ALL_LEVELS.filter((l) => l.chapter > 0)) {
    it(`${level.id} "${level.name}" rarely sinks on a blind shot`, () => {
      const rate = sinkRate(level.id, ANGLES, POWERS);
      expect(rate, `${(rate * 100).toFixed(1)}% of blind opening shots sink this hole`).toBeLessThan(
        0.07,
      );
    });
  }

  it('still lets the opening tutorial hole be generous', () => {
    // The guard must not push the first hole into being unfriendly.
    expect(sinkRate('c1-1', ANGLES, POWERS)).toBeGreaterThan(0.02);
  });
});
