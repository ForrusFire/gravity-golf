import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { ALL_LEVELS, CHAPTERS, levelById, nextLevel } from '../../src/game/levels';
import {
  BALL_RADIUS,
  compileLevel,
  levelHoleRadius,
  validateLevel,
  validateLevels,
  type LevelDef,
} from '../../src/game/level';
import { PlaySession } from '../../src/game/session';
import { reachableStars } from '../../src/game/solver';

describe('level catalogue', () => {
  it('has levels', () => {
    expect(ALL_LEVELS.length).toBeGreaterThan(0);
  });

  it('passes validation with no errors', () => {
    const issues = validateLevels(ALL_LEVELS);
    const errors = issues.filter((i) => i.severity === 'error');
    expect(errors.map((e) => `${e.levelId}: ${e.message}`)).toEqual([]);
  });

  it('has unique level ids', () => {
    const ids = ALL_LEVELS.map((l) => l.id);
    expect(new Set(ids).size).toBe(ids.length);
  });

  it('assigns every level to a declared chapter', () => {
    const chapterIndexes = new Set(CHAPTERS.map((c) => c.index));
    for (const level of ALL_LEVELS) {
      expect(chapterIndexes.has(level.chapter)).toBe(true);
    }
  });

  it('orders levels by chapter', () => {
    const chapters = ALL_LEVELS.map((l) => l.chapter);
    expect([...chapters].sort((a, b) => a - b)).toEqual(chapters);
  });

  it('gives every level exactly three stars', () => {
    for (const level of ALL_LEVELS) {
      expect(level.stars ?? []).toHaveLength(3);
    }
  });

  it('compiles every level into a world', () => {
    for (const level of ALL_LEVELS) {
      const world = compileLevel(level);
      expect(world.hole.radius).toBeGreaterThan(BALL_RADIUS);
      expect(world.collectibles).toHaveLength(3);
      expect(world.collectibles.every((c) => !c.collected)).toBe(true);
    }
  });

  it('looks levels up by id and walks the sequence', () => {
    const first = ALL_LEVELS[0]!;
    expect(levelById(first.id)).toBe(first);
    expect(levelById('nope')).toBeUndefined();
    expect(nextLevel(first.id)).toBe(ALL_LEVELS[1]);
    expect(nextLevel(ALL_LEVELS[ALL_LEVELS.length - 1]!.id)).toBeUndefined();
  });

  it('keeps the tee clear of the hole', () => {
    for (const level of ALL_LEVELS) {
      expect(V.distance(level.tee, level.hole.position)).toBeGreaterThan(
        levelHoleRadius(level) * 3,
      );
    }
  });
});

describe('validateLevel', () => {
  const base: LevelDef = {
    id: 'test',
    name: 'Test',
    chapter: 0,
    par: 2,
    tee: V.vec(-200, 0),
    hole: { position: V.vec(200, 0) },
    bounds: { minX: -400, minY: -300, maxX: 400, maxY: 300 },
    stars: [],
  };

  it('accepts a well-formed level', () => {
    expect(validateLevel(base).filter((i) => i.severity === 'error')).toEqual([]);
  });

  it('rejects a tee outside the bounds', () => {
    const issues = validateLevel({ ...base, tee: V.vec(-9000, 0) });
    expect(issues.some((i) => i.message.includes('tee is outside'))).toBe(true);
  });

  it('rejects a tee buried in a body', () => {
    const issues = validateLevel({
      ...base,
      bodies: [
        {
          id: 'p',
          shape: { kind: 'circle', center: V.vec(-200, 0), radius: 60 },
          material: 'rock',
        },
      ],
    });
    expect(issues.some((i) => i.message.includes('tee overlaps'))).toBe(true);
  });

  it('rejects a hole buried in a body', () => {
    const issues = validateLevel({
      ...base,
      bodies: [
        {
          id: 'p',
          shape: { kind: 'circle', center: V.vec(200, 0), radius: 60 },
          material: 'rock',
        },
      ],
    });
    expect(issues.some((i) => i.message.includes('hole overlaps'))).toBe(true);
  });

  it('rejects a par below one', () => {
    expect(validateLevel({ ...base, par: 0 }).some((i) => i.message.includes('par'))).toBe(true);
  });

  it('rejects more than three stars', () => {
    const issues = validateLevel({
      ...base,
      stars: [V.vec(0, 0), V.vec(20, 20), V.vec(40, 40), V.vec(60, 60)],
    });
    expect(issues.some((i) => i.message.includes('too many stars'))).toBe(true);
  });

  it('rejects duplicate body ids', () => {
    const issues = validateLevel({
      ...base,
      bodies: [
        { id: 'dup', shape: { kind: 'circle', center: V.vec(0, 0), radius: 10 }, material: 'rock' },
        { id: 'dup', shape: { kind: 'circle', center: V.vec(50, 0), radius: 10 }, material: 'rock' },
      ],
    });
    expect(issues.some((i) => i.message.includes('duplicate body id'))).toBe(true);
  });

  it('rejects an inverted bounds rect', () => {
    const issues = validateLevel({
      ...base,
      bounds: { minX: 400, minY: 300, maxX: -400, maxY: -300 },
    });
    expect(issues.some((i) => i.message.includes('bounds'))).toBe(true);
  });

  it('flags a tee sitting inside a hazard zone', () => {
    const issues = validateLevel({
      ...base,
      zones: [
        {
          id: 'haz',
          kind: 'hazard',
          area: { kind: 'circle', center: V.vec(-200, 0), radius: 50 },
        },
      ],
    });
    expect(issues.some((i) => i.message.includes('hazard'))).toBe(true);
  });

  it('detects duplicate ids across the whole catalogue', () => {
    const issues = validateLevels([base, base]);
    expect(issues.some((i) => i.message === 'duplicate level id')).toBe(true);
  });
});

describe('tee placement', () => {
  it('never lets the ball reach the cup without a shot being taken', () => {
    const freebies: string[] = [];
    for (const level of ALL_LEVELS) {
      const session = new PlaySession(level);
      // Ten seconds of doing nothing: the ball may settle, but must not sink.
      for (let i = 0; i < 600; i++) session.update(1 / 60);
      if (session.state === 'sunk') freebies.push(level.id);
    }
    expect(freebies).toEqual([]);
  });

  it('starts every hole with a ball that is ready to shoot', () => {
    const notReady: string[] = [];
    for (const level of ALL_LEVELS) {
      const session = new PlaySession(level);
      // The ball settles up front, so control is available immediately.
      if (!session.canShoot) notReady.push(level.id);
    }
    expect(notReady).toEqual([]);
  });
});

describe('star reachability', () => {
  // Chapters unlock on star totals, so an uncollectable star is a progression
  // bug, not a missed bonus.
  for (const level of ALL_LEVELS) {
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
      expect(missing, `stars never reached by any sampled shot: ${missing.join(', ')}`).toEqual([]);
    });
  }
});
