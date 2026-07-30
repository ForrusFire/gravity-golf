import { describe, expect, it } from 'vitest';
import { TAU, angleDelta } from '../../src/core/math';
import * as V from '../../src/core/vec2';
import { GhostRunner } from '../../src/game/ghost';
import type { LevelDef } from '../../src/game/level';
import { PlaySession, type SessionEvent } from '../../src/game/session';
import {
  BALL_SKINS,
  DEFAULT_SKIN,
  isSkinUnlocked,
  nextLockedSkin,
  resolveSkin,
} from '../../src/game/skins';

const baseLevel: LevelDef = {
  id: 'test-1',
  name: 'Test Hole',
  chapter: 0,
  par: 3,
  tee: V.vec(-300, 0),
  hole: { position: V.vec(300, 0), radius: 16, captureSpeed: 400 },
  bounds: { minX: -600, minY: -400, maxX: 600, maxY: 400 },
  boundsMode: 'open',
  ambientDrag: 0,
  stars: [],
};

const run = (session: PlaySession, seconds: number): SessionEvent[] => {
  const events: SessionEvent[] = [];
  const steps = Math.round(seconds * 60);
  for (let i = 0; i < steps; i++) events.push(...session.update(1 / 60));
  return events;
};

/** Plays until the session wants input again, or the hole is done. */
const settle = (session: PlaySession, maxSeconds = 30): void => {
  const steps = Math.round(maxSeconds * 60);
  for (let i = 0; i < steps; i++) {
    if (session.state === 'sunk') return;
    if (session.state === 'aiming' && session.canShoot) return;
    session.update(1 / 60);
  }
};

describe('undo', () => {
  // A floor to roll along, so shots actually come to rest and the ball has a
  // sequence of distinct resting places to unwind through.
  const rolling: LevelDef = {
    ...baseLevel,
    hole: { position: V.vec(9000, 9000), radius: 16 },
    bounds: { minX: -600, minY: -400, maxX: 600, maxY: 400 },
    boundsMode: 'wall',
    ambientDrag: 0.5,
    uniformGravity: V.vec(0, 500),
    tee: V.vec(-300, -20),
    bodies: [
      {
        id: 'floor',
        shape: { kind: 'capsule', a: V.vec(-560, 40), b: V.vec(560, 40), radius: 12 },
        material: 'rock',
      },
    ],
  };

  it('is unavailable before any shot is taken', () => {
    expect(new PlaySession(rolling).canUndo).toBe(false);
  });

  it('restores the ball, the stroke count and the clock', () => {
    const session = new PlaySession(rolling);
    const origin = session.ball.position;

    session.shoot(V.vec(1, 0), 0.5);
    settle(session);
    expect(session.strokes).toBe(1);
    expect(session.ball.position).not.toEqual(origin);

    expect(session.canUndo).toBe(true);
    expect(session.undo()).toBe(true);
    expect(session.strokes).toBe(0);
    expect(session.ball.position).toEqual(origin);
    expect(session.world.time).toBe(0);
    expect(session.canShoot).toBe(true);
    expect(session.canUndo).toBe(false);
  });

  it('works mid-flight, cancelling the shot in progress', () => {
    const session = new PlaySession(rolling);
    const origin = session.ball.position;
    session.shoot(V.vec(1, 0), 0.8);
    run(session, 0.3);
    expect(session.state).toBe('flying');

    expect(session.undo()).toBe(true);
    expect(session.state).toBe('aiming');
    expect(session.ball.position).toEqual(origin);
    expect(V.length(session.ball.velocity)).toBe(0);
  });

  it('unwinds several shots one at a time', () => {
    const session = new PlaySession(rolling);
    const positions = [session.ball.position];
    for (let i = 0; i < 3; i++) {
      session.shoot(V.vec(1, 0), 0.4);
      settle(session);
      positions.push(session.ball.position);
    }
    expect(session.strokes).toBe(3);

    for (let i = 3; i > 0; i--) {
      expect(session.undo()).toBe(true);
      expect(session.strokes).toBe(i - 1);
      expect(session.ball.position).toEqual(positions[i - 1]);
    }
    expect(session.undo()).toBe(false);
  });

  it('gives back stars the undone shot collected', () => {
    // On the rolling path, not above it: the ball settles a little over y=20.
    const level: LevelDef = { ...rolling, stars: [V.vec(-250, 20)] };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.5);
    settle(session);
    expect(session.starCount).toBe(1);

    session.undo();
    expect(session.starCount).toBe(0);
    expect(session.world.collectibles.every((c) => !c.collected)).toBe(true);
  });

  it('drops the recorded shot so a re-shot run is not double-counted', () => {
    const session = new PlaySession(rolling);
    session.shoot(V.vec(1, 0), 0.5);
    settle(session);
    expect(session.shotList).toHaveLength(1);
    session.undo();
    expect(session.shotList).toHaveLength(0);
  });

  it('is cleared by a restart', () => {
    const session = new PlaySession(rolling);
    session.shoot(V.vec(1, 0), 0.5);
    settle(session);
    session.restart();
    expect(session.canUndo).toBe(false);
    expect(session.shotList).toEqual([]);
  });
});

describe('feats', () => {
  it('awards a clean sink for a shot that touches nothing', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    expect(session.result).not.toBeNull();
    expect(session.result!.feats).toContain('clean');
  });

  it('awards all three stars as a perfect run', () => {
    const level: LevelDef = { ...baseLevel, stars: [V.vec(0, 0), V.vec(100, 0), V.vec(200, 0)] };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    expect(session.result!.feats).toContain('allStars');
  });

  it('awards a full orbit for a shot that circles a planet', () => {
    // The ball starts perched on the far side of a rock, so it is held against
    // the planet's pull instead of falling straight onto it, then is launched
    // tangentially into an orbit that light drag decays into the cup.
    const G = 12000;
    const BODY = 22;
    const PERCH = 260;
    const level: LevelDef = {
      ...baseLevel,
      tee: V.vec(PERCH + 22, 0),
      hole: { position: V.vec(0, 0), radius: 34, captureSpeed: 5000 },
      bounds: { minX: -1400, minY: -1400, maxX: 1400, maxY: 1400 },
      ambientDrag: 0.12,
      shotTimeout: 90,
      bodies: [
        {
          id: 'core',
          shape: { kind: 'circle', center: V.vec(0, 0), radius: BODY },
          material: 'rock',
          gravity: { strength: G, range: 0 },
        },
        {
          id: 'perch',
          shape: { kind: 'circle', center: V.vec(PERCH, 0), radius: 14 },
          material: 'rock',
        },
      ],
    };

    const session = new PlaySession(level);
    const radius = V.length(session.ball.position);
    // sqrt(g * R^2 / r) is the circular-orbit speed at this radius.
    const speed = Math.sqrt((G * BODY * BODY) / radius);
    session.shoot(V.vec(0, 1), speed / session.maxPower);

    // Measure the swept angle independently, so the assertion is about the
    // shot really going round rather than about the tracker agreeing with
    // itself.
    let swept = 0;
    let last = Math.atan2(session.ball.position.y, session.ball.position.x);
    for (let i = 0; i < 60 * 90; i++) {
      session.update(1 / 60);
      const angle = Math.atan2(session.ball.position.y, session.ball.position.x);
      swept += angleDelta(last, angle);
      last = angle;
      if (session.state !== 'flying') break;
    }

    expect(Math.abs(swept), 'the shot should complete at least one revolution').toBeGreaterThan(TAU);
    expect(session.result, 'the orbiting ball should reach the cup').not.toBeNull();
    expect(session.result!.feats).toContain('orbit');
  });

  it('does not award an orbit for a straight putt', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    expect(session.result!.feats).not.toContain('orbit');
  });

  it('reports no feats when the shot is unremarkable', () => {
    const level: LevelDef = {
      ...baseLevel,
      // Rolling along a surface disqualifies "clean".
      tee: V.vec(-300, -20),
      bodies: [
        {
          id: 'floor',
          shape: { kind: 'capsule', a: V.vec(-500, 0), b: V.vec(500, 0), radius: 10 },
          material: 'rock',
        },
      ],
      uniformGravity: V.vec(0, 400),
    };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, -0.05), 0.35);
    run(session, 8);
    if (session.result) expect(session.result.feats).not.toContain('clean');
  });

  it('records the shots taken so a run can be replayed', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    expect(session.result!.shots).toHaveLength(1);
    expect(session.result!.shots[0]!.power).toBeCloseTo(0.35, 6);
    expect(session.result!.shots[0]!.angle).toBeCloseTo(0, 6);
  });
});

describe('GhostRunner', () => {
  it('is invisible with no recorded run', () => {
    expect(new GhostRunner(baseLevel, []).visible).toBe(false);
  });

  it('replays a recorded run and finishes where the original did', () => {
    const original = new PlaySession(baseLevel);
    original.shoot(V.vec(1, 0), 0.35);
    run(original, 4);
    const shots = original.result!.shots;

    const ghost = new GhostRunner(baseLevel, shots);
    expect(ghost.visible).toBe(true);
    for (let i = 0; i < 60 * 5; i++) ghost.advance(1 / 60);

    // The recorded run sank, so the ghost's run ends too.
    expect(ghost.visible).toBe(false);
    expect(ghost.strokes).toBe(1);
  });

  it('builds a trail as it goes', () => {
    const original = new PlaySession(baseLevel);
    original.shoot(V.vec(1, 0), 0.35);
    run(original, 4);

    const ghost = new GhostRunner(baseLevel, original.result!.shots);
    for (let i = 0; i < 30; i++) ghost.advance(1 / 60);
    expect(ghost.trail.length).toBeGreaterThan(0);
  });

  it('does not run away with extra shots it was never given', () => {
    const level: LevelDef = { ...baseLevel, hole: { position: V.vec(9000, 9000), radius: 16 } };
    const ghost = new GhostRunner(level, [{ angle: 0, power: 0.1 }]);
    for (let i = 0; i < 60 * 40; i++) ghost.advance(1 / 60);
    // One recorded shot means one shot played, then the ghost bows out.
    expect(ghost.strokes).toBeLessThanOrEqual(1);
    expect(ghost.visible).toBe(false);
  });
});

describe('ball skins', () => {
  it('starts with only the default unlocked', () => {
    expect(isSkinUnlocked(DEFAULT_SKIN, 0)).toBe(true);
    expect(BALL_SKINS.filter((skin) => isSkinUnlocked(skin, 0))).toEqual([DEFAULT_SKIN]);
  });

  it('unlocks progressively with stars', () => {
    const atFifty = BALL_SKINS.filter((skin) => isSkinUnlocked(skin, 50));
    expect(atFifty.length).toBeGreaterThan(1);
    expect(atFifty.length).toBeLessThan(BALL_SKINS.length);
    expect(BALL_SKINS.every((skin) => isSkinUnlocked(skin, 999))).toBe(true);
  });

  it('falls back to the default for a locked or unknown choice', () => {
    const locked = BALL_SKINS[BALL_SKINS.length - 1]!;
    expect(resolveSkin(locked.id, 0)).toBe(DEFAULT_SKIN);
    expect(resolveSkin('does-not-exist', 999)).toBe(DEFAULT_SKIN);
    expect(resolveSkin(locked.id, locked.starsRequired)).toBe(locked);
  });

  it('names the next skin to work toward, and none when all are held', () => {
    expect(nextLockedSkin(0)?.starsRequired).toBeGreaterThan(0);
    expect(nextLockedSkin(999)).toBeUndefined();
  });

  it('orders skins by the stars they need', () => {
    const required = BALL_SKINS.map((skin) => skin.starsRequired);
    expect([...required].sort((a, b) => a - b)).toEqual(required);
  });
});
