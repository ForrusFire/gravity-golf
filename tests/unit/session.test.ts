import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { BALL_RADIUS, type LevelDef } from '../../src/game/level';
import { PlaySession, medalFor, settledTee, type SessionEvent } from '../../src/game/session';

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

/** Steps the session in fixed 1/60 slices, collecting every event. */
const run = (session: PlaySession, seconds: number): SessionEvent[] => {
  const events: SessionEvent[] = [];
  const steps = Math.round(seconds * 60);
  for (let i = 0; i < steps; i++) events.push(...session.update(1 / 60));
  return events;
};

describe('medalFor', () => {
  it('ranks results against par', () => {
    expect(medalFor(1, 3)).toBe('ace');
    expect(medalFor(2, 3)).toBe('gold');
    expect(medalFor(3, 3)).toBe('silver');
    expect(medalFor(5, 3)).toBe('bronze');
    expect(medalFor(6, 3)).toBe('none');
  });

  it('treats a one-stroke finish as an ace even when par is 1', () => {
    expect(medalFor(1, 1)).toBe('ace');
  });

  it('never ranks a zero-stroke finish above an ace', () => {
    expect(medalFor(0, 3)).toBe('ace');
  });
});

describe('zero-stroke finishes', () => {
  it('scores a ball that rolls in unaided as one stroke, not zero', () => {
    // A tee perched right above the cup: the ball reaches it without a shot.
    const level: LevelDef = {
      ...baseLevel,
      tee: V.vec(0, -60),
      hole: { position: V.vec(0, 0), radius: 20, captureSpeed: 400 },
      bodies: [
        {
          id: 'ground',
          shape: { kind: 'circle', center: V.vec(0, 700), radius: 600 },
          material: 'rock',
          gravity: { strength: 400, range: 0 },
        },
      ],
    };
    const session = new PlaySession(level);
    run(session, 4);

    expect(session.result).not.toBeNull();
    expect(session.result!.strokes).toBe(1);
    expect(session.result!.medal).toBe('ace');
    expect(session.strokes).toBe(1);
  });
});

describe('settledTee', () => {
  it('leaves a tee in open space where it was authored', () => {
    expect(settledTee(baseLevel)).toEqual(baseLevel.tee);
  });

  it('drops a floating tee onto the surface below it', () => {
    const level: LevelDef = {
      ...baseLevel,
      tee: V.vec(0, -200),
      bodies: [
        {
          id: 'ground',
          shape: { kind: 'circle', center: V.vec(0, 700), radius: 500 },
          material: 'rock',
          gravity: { strength: 500, range: 0 },
        },
      ],
    };
    const settled = settledTee(level);
    expect(settled.y).toBeGreaterThan(-200);
    // Comes to rest on the surface, not inside it.
    expect(V.distance(settled, V.vec(0, 700))).toBeCloseTo(500 + BALL_RADIUS, 0);
  });
});

describe('PlaySession', () => {
  it('starts ready to shoot with no strokes taken', () => {
    const session = new PlaySession(baseLevel);
    expect(session.state).toBe('aiming');
    expect(session.canShoot).toBe(true);
    expect(session.strokes).toBe(0);
  });

  it('counts a stroke and starts flying on a shot', () => {
    const session = new PlaySession(baseLevel);
    const events = session.shoot(V.vec(1, 0), 0.5);
    expect(events).toHaveLength(1);
    expect(session.strokes).toBe(1);
    expect(session.state).toBe('flying');
    expect(session.canShoot).toBe(false);
  });

  it('ignores a second shot while the ball is in flight', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.5);
    expect(session.shoot(V.vec(0, 1), 1)).toEqual([]);
    expect(session.strokes).toBe(1);
  });

  it('ignores a zero-power or zero-direction shot', () => {
    const session = new PlaySession(baseLevel);
    expect(session.shoot(V.vec(1, 0), 0)).toEqual([]);
    expect(session.shoot(V.vec(0, 0), 1)).toEqual([]);
    expect(session.strokes).toBe(0);
  });

  it('clamps power above 1', () => {
    const session = new PlaySession(baseLevel);
    const [event] = session.shoot(V.vec(1, 0), 5);
    expect(event && event.type === 'shot' && event.power).toBe(1);
    expect(V.length(session.ball.velocity)).toBeCloseTo(session.maxPower, 6);
  });

  it('sinks the ball and reports a result', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.35);
    const events = run(session, 4);
    const sunk = events.find((e) => e.type === 'sunk');
    expect(sunk).toBeDefined();
    expect(session.state).toBe('sunk');
    expect(session.result).not.toBeNull();
    expect(session.result!.strokes).toBe(1);
    expect(session.result!.medal).toBe('ace');
    expect(session.result!.par).toBe(3);
  });

  it('stops updating once the hole is finished', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    const position = session.ball.position;
    expect(run(session, 2)).toEqual([]);
    expect(session.ball.position).toEqual(position);
  });

  it('returns to aiming when the ball settles', () => {
    const level: LevelDef = {
      ...baseLevel,
      hole: { position: V.vec(3000, 3000), radius: 16 },
      bodies: [
        {
          id: 'ground',
          shape: { kind: 'circle', center: V.vec(0, 700), radius: 500 },
          material: 'sand',
          gravity: { strength: 600, range: 0 },
        },
      ],
      tee: V.vec(0, 180),
    };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, -0.4), 0.4);
    const events = run(session, 12);
    expect(events.some((e) => e.type === 'settled')).toBe(true);
    expect(session.state).toBe('aiming');
    expect(session.canShoot).toBe(true);
    expect(session.strokes).toBe(1);
  });

  it('adds a penalty stroke on death and respawns at the shot origin', () => {
    const level: LevelDef = {
      ...baseLevel,
      bodies: [
        {
          id: 'sun',
          shape: { kind: 'circle', center: V.vec(0, 0), radius: 60 },
          material: 'lava',
        },
      ],
    };
    const session = new PlaySession(level);
    const origin = session.ball.position;
    session.shoot(V.vec(1, 0), 0.5);
    const events = run(session, 3);

    const died = events.find((e) => e.type === 'died');
    expect(died).toBeDefined();
    expect(died && died.type === 'died' && died.cause).toBe('hazard');
    // One stroke for the shot, one for the death.
    expect(session.strokes).toBe(2);
    expect(events.some((e) => e.type === 'respawn')).toBe(true);
    expect(session.state).toBe('aiming');
    expect(session.ball.position).toEqual(origin);
    expect(session.canShoot).toBe(true);
  });

  it('reports an out-of-bounds death', () => {
    const level: LevelDef = { ...baseLevel, boundsMode: 'kill' };
    const session = new PlaySession(level);
    session.shoot(V.vec(0, -1), 1);
    const events = run(session, 3);
    const died = events.find((e) => e.type === 'died');
    expect(died && died.type === 'died' && died.cause).toBe('out-of-bounds');
  });

  it('banks stars when the ball settles safely', () => {
    const level: LevelDef = {
      ...baseLevel,
      stars: [V.vec(0, 0)],
      hole: { position: V.vec(300, 0), radius: 16, captureSpeed: 400 },
    };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.35);
    const events = run(session, 4);
    expect(events.some((e) => e.type === 'star')).toBe(true);
    expect(session.starCount).toBe(1);
    expect(session.result!.stars).toBe(1);
  });

  it('takes stars back when the shot that collected them is fatal', () => {
    const level: LevelDef = {
      ...baseLevel,
      stars: [V.vec(-100, 0)],
      bodies: [
        {
          id: 'sun',
          shape: { kind: 'circle', center: V.vec(0, 0), radius: 60 },
          material: 'lava',
        },
      ],
    };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.5);
    const events = run(session, 3);

    expect(events.some((e) => e.type === 'star')).toBe(true);
    expect(events.some((e) => e.type === 'star-lost')).toBe(true);
    expect(session.starCount).toBe(0);
    // The star is back on the board to be collected again.
    expect(session.world.collectibles.every((c) => !c.collected)).toBe(true);
  });

  it('abandons a shot that never settles', () => {
    const level: LevelDef = {
      ...baseLevel,
      boundsMode: 'open',
      shotTimeout: 2,
      hole: { position: V.vec(9000, 9000), radius: 16 },
    };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.2);
    const events = run(session, 4);
    expect(events.some((e) => e.type === 'timeout')).toBe(true);
    // The shot still counts, but there is no extra penalty on top.
    expect(session.strokes).toBe(1);
    expect(session.state).toBe('aiming');
  });

  it('exposes a shot clock that fills as the shot runs long', () => {
    const level: LevelDef = {
      ...baseLevel,
      shotTimeout: 4,
      hole: { position: V.vec(9000, 9000), radius: 16 },
    };
    const session = new PlaySession(level);
    expect(session.shotClock).toBe(0);
    session.shoot(V.vec(1, 0), 0.2);
    run(session, 2);
    expect(session.shotClock).toBeGreaterThan(0.4);
    expect(session.shotClock).toBeLessThanOrEqual(1);
  });

  it('restarts back to a clean slate', () => {
    const level: LevelDef = { ...baseLevel, stars: [V.vec(0, 0)] };
    const session = new PlaySession(level);
    session.shoot(V.vec(1, 0), 0.35);
    run(session, 4);
    expect(session.strokes).toBe(1);

    session.restart();
    expect(session.strokes).toBe(0);
    expect(session.state).toBe('aiming');
    expect(session.starCount).toBe(0);
    expect(session.result).toBeNull();
    expect(session.world.time).toBe(0);
    expect(session.world.collectibles.every((c) => !c.collected)).toBe(true);
    expect(session.ball.position).toEqual(settledTee(level));
  });

  it('predicts a trajectory without disturbing play', () => {
    const session = new PlaySession(baseLevel);
    const before = session.ball.position;
    const path = session.predict(V.vec(1, 0), 0.5);
    expect(path.points.length).toBeGreaterThan(1);
    expect(session.ball.position).toEqual(before);
    expect(session.strokes).toBe(0);
  });

  it('survives a huge frame delta without fast-forwarding the world', () => {
    const session = new PlaySession(baseLevel);
    session.shoot(V.vec(1, 0), 0.3);
    const before = session.world.time;
    session.update(30);
    // The frame is clamped, so one stall cannot skip the ball across the hole.
    expect(session.world.time - before).toBeLessThan(0.2);
  });
});
