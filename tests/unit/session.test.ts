import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { BALL_RADIUS, type LevelDef } from '../../src/game/level';
import { PlaySession, medalFor, settledTee, type SessionEvent } from '../../src/game/session';
import { ALL_LEVELS } from '../../src/game/levels';

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
  it('never lets a waiting ball roll into the cup unaided', () => {
    // A tee perched right above the cup. This used to score a free hole — the
    // ball dropped in while the player was still aiming, and only a one-stroke
    // floor kept it from reading as better than an ace. A waiting ball now
    // stays where it lies, so the finish has to be played.
    // (The floor itself is still covered by medalFor's zero-stroke case.)
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

    expect(session.result).toBeNull();
    expect(session.strokes).toBe(0);
    expect(session.canShoot).toBe(true);
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

describe('a ball waiting to be struck', () => {
  it('stays exactly where it lies on every hole, however long the aim', () => {
    // A human takes seconds to line up a shot. Before the hold, nine holes let
    // the field drag the waiting ball off its tee — and once it passed rest
    // speed the player was locked out of shooting until it stopped somewhere
    // else. Every other test fired on the first frame and never noticed.
    const drifted: string[] = [];
    for (const level of ALL_LEVELS) {
      const session = new PlaySession(level);
      const start = session.ball.position;
      for (let i = 0; i < 600; i++) {
        session.update(1 / 60);
        if (!session.canShoot) break;
      }
      const moved = V.distance(start, session.ball.position);
      if (!session.canShoot || moved > 0.5) {
        drifted.push(`${level.id} canShoot=${session.canShoot} moved=${moved.toFixed(1)}`);
      }
    }
    expect(drifted).toEqual([]);
  });

  it('does not fast-forward the next shot after a long aim', () => {
    // The aiming loop used to take one physics step per frame while banking
    // the rest, then spend the backlog on the shot: aim for ten seconds and the
    // first half-second of flight covered three seconds of simulation.
    const elapsedAfter = (aimSeconds: number): number => {
      const session = new PlaySession(ALL_LEVELS[0]!);
      for (let i = 0; i < aimSeconds * 60; i++) session.update(1 / 60);
      const before = session.world.time;
      session.shoot(V.vec(1, -0.3), 0.6);
      for (let i = 0; i < 30; i++) session.update(1 / 60);
      return session.world.time - before;
    };
    expect(elapsedAfter(10)).toBeCloseTo(elapsedAfter(0), 2);
    expect(elapsedAfter(10)).toBeCloseTo(0.5, 2);
  });

  it('keeps the world clock at full speed while aiming', () => {
    // The same loop ran moving cups and pulsing walls at a quarter of their
    // real tempo while the player was reading them, then at full speed once
    // they fired — so every timing puzzle showed the wrong rhythm.
    const session = new PlaySession(ALL_LEVELS[0]!);
    for (let i = 0; i < 120; i++) session.update(1 / 60);
    expect(session.world.time).toBeCloseTo(2, 2);
  });

  it('is carried by a moving platform it rests on', () => {
    // The hold is kinematic, not a freeze: a ball on a lift still rides it.
    const level: LevelDef = {
      id: 'lift-test',
      name: 'Lift',
      chapter: 0,
      par: 3,
      tee: V.vec(0, 270),
      hole: { position: V.vec(500, -300) },
      bounds: { minX: -600, minY: -500, maxX: 600, maxY: 500 },
      boundsMode: 'open',
      uniformGravity: V.vec(0, 300),
      bodies: [
        {
          id: 'lift',
          shape: { kind: 'circle', center: V.vec(0, 300), radius: 20 },
          material: 'metal',
          motion: { kind: 'oscillate', from: V.vec(0, 300), to: V.vec(0, 0), period: 6, phase: 0 },
        },
      ],
      stars: [],
    };
    const session = new PlaySession(level);
    const start = session.ball.position.y;
    for (let i = 0; i < 90; i++) session.update(1 / 60);
    expect(session.canShoot).toBe(true);
    // Rising with the platform, not left hanging where it started.
    expect(session.ball.position.y).toBeLessThan(start - 50);
  });
});

describe('replaying a run on a hole that moves', () => {
  // Plays a session forward in display frames until it wants a shot again.
  const untilReady = (session: PlaySession): void => {
    for (let i = 0; i < 60 * 40 && !(session.canShoot || session.state === 'sunk'); i++) {
      session.update(1 / 60);
    }
  };

  it('strikes each shot at its recorded world time and ends exactly where the run did', () => {
    // Every hole with something on a clock — moving cup, orbiting or pulsing
    // bodies — where the instant of the strike changes the shot.
    const moving = ALL_LEVELS.filter(
      (level) => level.hole.motion || (level.bodies ?? []).some((b) => b.motion || b.pulse),
    ).slice(0, 6);
    expect(moving.length).toBeGreaterThan(0);

    for (const level of moving) {
      const live = new PlaySession(level);
      // A player who thinks for a while, takes two shots, and undoes one.
      for (let i = 0; i < 151; i++) live.update(1 / 60);
      live.shoot(V.fromAngle(-0.6), 0.55);
      untilReady(live);
      if (live.state === 'sunk') continue;
      for (let i = 0; i < 47; i++) live.update(1 / 60);
      live.shoot(V.fromAngle(0.4), 0.4);
      untilReady(live);
      if (live.canUndo) live.undo();
      for (let i = 0; i < 33; i++) live.update(1 / 60);
      live.shoot(V.fromAngle(-1.1), 0.7);
      untilReady(live);
      const shots = live.shotList;
      expect(shots.every((s) => s.time !== undefined)).toBe(true);

      const replay = new PlaySession(level);
      for (const shot of shots) {
        untilReady(replay);
        replay.holdUntil(shot.time!);
        expect(replay.world.time).toBeCloseTo(shot.time!, 9);
        replay.shoot(V.fromAngle(shot.angle), shot.power);
      }
      untilReady(replay);

      expect(replay.state, level.id).toBe(live.state);
      expect(replay.strokes, level.id).toBe(live.strokes);
      expect(replay.ball.position.x, level.id).toBeCloseTo(live.ball.position.x, 6);
      expect(replay.ball.position.y, level.id).toBeCloseTo(live.ball.position.y, 6);
    }
  }, 120000);
});

describe('a shot that is abandoned', () => {
  it('gives back the stars it picked up, so they can be collected again', () => {
    // Nothing pulls and nothing slows the ball: it sails through the star and
    // never settles, so the shot clock runs out.
    const level: LevelDef = {
      id: 'test-timeout-star',
      name: 'Drift',
      chapter: 1,
      par: 3,
      tee: V.vec(0, 0),
      hole: { position: V.vec(0, 900) },
      bounds: { minX: -5000, minY: -5000, maxX: 5000, maxY: 5000 },
      boundsMode: 'wall',
      ambientDrag: 0,
      shotTimeout: 2,
      stars: [V.vec(120, 0)],
    };
    const session = new PlaySession(level);
    const events: SessionEvent[] = [];
    const run = (seconds: number): void => {
      for (let i = 0; i < seconds * 60; i++) events.push(...session.update(1 / 60));
    };
    run(0.2);
    events.push(...session.shoot(V.vec(1, 0), 0.4));
    run(6);

    expect(events.some((e) => e.type === 'star')).toBe(true);
    expect(events.some((e) => e.type === 'timeout')).toBe(true);
    expect(events.some((e) => e.type === 'star-lost')).toBe(true);
    expect(session.world.collectibles[0]!.collected).toBe(false);
    expect(session.starCount).toBe(0);

    // And it is there to be had on the next attempt.
    while (!session.canShoot) run(0.1);
    events.length = 0;
    events.push(...session.shoot(V.vec(1, 0), 0.4));
    run(1);
    expect(events.some((e) => e.type === 'star')).toBe(true);
  });
});

describe('feats for the later mechanics', () => {
  const corridor = (extra: Partial<LevelDef>): LevelDef => ({
    id: 'test-feats',
    name: 'Corridor',
    chapter: 1,
    par: 3,
    tee: V.vec(0, 0),
    hole: { position: V.vec(600, 0), captureSpeed: 2000 },
    bounds: { minX: -2000, minY: -2000, maxX: 2000, maxY: 2000 },
    boundsMode: 'wall',
    ambientDrag: 0,
    ...extra,
  });
  const play = (level: LevelDef) => {
    const session = new PlaySession(level);
    for (let i = 0; i < 20; i++) session.update(1 / 60);
    session.shoot(V.vec(1, 0), 0.3);
    for (let i = 0; i < 60 * 8 && session.state !== 'sunk'; i++) session.update(1 / 60);
    return session;
  };

  it('credits a sink through a portal', () => {
    const session = play(
      corridor({
        portals: [
          { id: 'w', from: V.vec(150, 0), to: V.vec(350, 0), radius: 24, twist: 0, boost: 1, bidirectional: false },
        ],
      }),
    );
    expect(session.state).toBe('sunk');
    expect(session.result!.feats).toContain('wormhole');
  });

  it('credits a sink out of a boost ring', () => {
    const session = play(
      corridor({
        boosters: [{ id: 'b', position: V.vec(200, 0), radius: 24, direction: V.vec(1, 0), speed: 400 }],
      }),
    );
    expect(session.state).toBe('sunk');
    expect(session.result!.feats).toContain('afterburner');
  });

  it('credits a switch thrown and a sink on the same shot', () => {
    const session = play(
      corridor({
        switches: [{ id: 's', position: V.vec(250, 0), radius: 20, once: true }],
      }),
    );
    expect(session.state).toBe('sunk');
    expect(session.result!.feats).toContain('oneTwo');
    expect(session.result!.feats).not.toContain('wormhole');
  });
});
