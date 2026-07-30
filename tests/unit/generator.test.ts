import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import {
  dailyId,
  dailySeed,
  generateLevel,
  type GeneratedLevel,
} from '../../src/game/generator';
import { compileLevel, validateLevel } from '../../src/game/level';
import { PlaySession } from '../../src/game/session';
import { solveLevel } from '../../src/game/solver';

const SEEDS = [1, 4242, 99999, 0x9e3779b9];

describe('dailySeed', () => {
  it('is stable for a given UTC day', () => {
    const a = new Date(Date.UTC(2026, 3, 20, 3, 0, 0));
    const b = new Date(Date.UTC(2026, 3, 20, 21, 30, 0));
    expect(dailySeed(a)).toBe(dailySeed(b));
  });

  it('differs across days', () => {
    expect(dailySeed(new Date(Date.UTC(2026, 3, 20)))).not.toBe(
      dailySeed(new Date(Date.UTC(2026, 3, 21))),
    );
  });

  it('produces a stable, sortable id', () => {
    expect(dailyId(new Date(Date.UTC(2026, 0, 5)))).toBe('daily-2026-01-05');
  });
});

describe('generateLevel', () => {
  const generated: GeneratedLevel[] = [];

  for (const seed of SEEDS) {
    it(`seed ${seed} produces a valid, completable hole`, () => {
      const result = generateLevel(seed);
      expect(result.level, `no candidate passed after ${result.attempts} attempts`).not.toBeNull();
      const level = result.level!;
      generated.push(level);

      // Same standard as a handmade hole.
      const errors = validateLevel(level).filter((i) => i.severity === 'error');
      expect(errors.map((e) => e.message)).toEqual([]);
      expect(compileLevel(level).collectibles).toHaveLength(3);
      expect(level.par).toBeGreaterThanOrEqual(2);
      expect(level.par).toBeLessThanOrEqual(4);

      // And the same proof: a solution exists and actually sinks when replayed.
      const solution = solveLevel(level, { maxStrokes: level.par });
      expect(solution.solved).toBe(true);

      const session = new PlaySession(level);
      for (const shot of solution.shots) {
        for (let i = 0; i < 2400 && !session.canShoot && session.state !== 'sunk'; i++) {
          session.update(1 / 60);
        }
        if (session.state === 'sunk') break;
        session.shoot(V.fromAngle(shot.angle), shot.power);
        for (let i = 0; i < 2400 && session.state === 'flying'; i++) session.update(1 / 60);
      }
      for (let i = 0; i < 2400 && session.state === 'flying'; i++) session.update(1 / 60);
      expect(session.state).toBe('sunk');
    });
  }

  it('is deterministic for a seed', () => {
    const a = generateLevel(31337).level;
    const b = generateLevel(31337).level;
    expect(a).not.toBeNull();
    expect(JSON.stringify(a)).toBe(JSON.stringify(b));
  });

  it('produces different holes for different seeds', () => {
    const ids = new Set(generated.map((level) => JSON.stringify(level.bodies)));
    expect(ids.size).toBe(generated.length);
  });

  it('never leaves the direct line to the hole unobstructed', () => {
    for (const level of generated) {
      const blockers = (level.bodies ?? []).filter((b) => !b.id.startsWith('ledge-'));
      expect(blockers.length).toBeGreaterThan(0);
      // A blocker wall is always placed between the two ledges.
      expect(blockers.some((b) => b.id === 'blocker')).toBe(true);
    }
  });

  it('never gates a hole behind a switch the ball cannot reach', () => {
    // A gate keyed to a pad that was never placed would be an unopenable wall.
    for (const level of generated) {
      const gated = (level.bodies ?? []).filter((b) => b.removedBy !== undefined);
      const padIds = new Set((level.switches ?? []).map((sw) => sw.id));
      for (const body of gated) {
        const keys = typeof body.removedBy === 'string' ? [body.removedBy] : body.removedBy!;
        for (const key of keys) expect(padIds.has(key)).toBe(true);
      }
    }
  });

  it('keeps generated holes free of clock-dependent state', () => {
    // The verified solution only replays if the hole reacts to the ball rather
    // than to the time on the clock, so no motion and no held switches.
    for (const level of generated) {
      for (const body of level.bodies ?? []) {
        expect(body.motion).toBeUndefined();
        expect(body.pulse).toBeUndefined();
      }
      for (const pad of level.switches ?? []) expect(pad.holdTime).toBeUndefined();
      expect(level.hole.motion).toBeUndefined();
    }
  });

  it('serialises to JSON and back without losing anything', () => {
    const level = generated[0]!;
    const round = JSON.parse(JSON.stringify(level)) as GeneratedLevel;
    expect(round.id).toBe(level.id);
    expect(round.par).toBe(level.par);
    expect(compileLevel(round).bodies).toHaveLength(compileLevel(level).bodies.length);
    expect(compileLevel(round).switches).toHaveLength(compileLevel(level).switches.length);
    // Cached daily holes travel through localStorage as JSON.
    expect(validateLevel(round).filter((i) => i.severity === 'error')).toEqual([]);
  });
});
