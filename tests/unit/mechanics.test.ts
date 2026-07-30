import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import type { Vec2 } from '../../src/core/vec2';
import { DEFAULT_PHYSICS, type Ball } from '../../src/physics/types';
import {
  createBall,
  createBallRuntime,
  createWorld,
  holePositionAt,
  holeVelocityAt,
  isBodyActive,
  launchBall,
  stepWorld,
  type BallRuntime,
  type SimEvent,
  type World,
} from '../../src/physics/world';
import {
  BALL_RADIUS,
  booster,
  bridge,
  compileLevel,
  crystal,
  gate,
  membrane,
  switchPad,
  timedPad,
  validateLevel,
  type LevelDef,
} from '../../src/game/level';
import { PlaySession } from '../../src/game/session';

const makeWorld = (overrides: Partial<World> = {}): World =>
  createWorld({
    hole: { position: V.vec(9999, 9999), radius: 12, captureSpeed: 200 },
    bounds: { minX: -2000, minY: -2000, maxX: 2000, maxY: 2000 },
    boundsMode: 'open',
    config: { ...DEFAULT_PHYSICS, ambientDrag: 0 },
    ...overrides,
  });

const run = (world: World, ball: Ball, runtime: BallRuntime, seconds: number): SimEvent[] => {
  const events: SimEvent[] = [];
  const steps = Math.round(seconds / world.config.timeStep);
  for (let i = 0; i < steps; i++) stepWorld(world, ball, runtime, events);
  return events;
};

/** A ball launched from `from` toward `to` at `speed`. */
const shoot = (from: Vec2, to: Vec2, speed: number): [Ball, BallRuntime] => {
  const ball = createBall(from, BALL_RADIUS);
  const runtime = createBallRuntime();
  launchBall(ball, runtime, V.mul(V.normalize(V.sub(to, from)), speed));
  return [ball, runtime];
};

describe('switch pads', () => {
  it('fires once when the ball passes through a latching pad', () => {
    const world = makeWorld({ switches: [{ ...switchPad('sw', V.vec(0, 0), 30), on: false }] });
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    const events = run(world, ball, runtime, 2);
    const fired = events.filter((e) => e.type === 'switch');
    expect(fired).toHaveLength(1);
    expect(fired[0]).toMatchObject({ id: 'sw', on: true });
    expect(world.switches[0]!.on).toBe(true);
  });

  it('does not re-fire a latching pad on a second pass', () => {
    const world = makeWorld({ switches: [{ ...switchPad('sw', V.vec(0, 0), 30), on: false }] });
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    // Send it back the other way through the same pad.
    launchBall(ball, runtime, V.vec(-400, 0));
    const events = run(world, ball, runtime, 2);
    expect(events.filter((e) => e.type === 'switch')).toHaveLength(0);
    expect(world.switches[0]!.on).toBe(true);
  });

  it('toggles a non-latching pad on every pass', () => {
    const world = makeWorld({
      switches: [{ ...switchPad('sw', V.vec(0, 0), 30, false), on: false }],
    });
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(world.switches[0]!.on).toBe(true);
    launchBall(ball, runtime, V.vec(-400, 0));
    run(world, ball, runtime, 2);
    expect(world.switches[0]!.on).toBe(false);
  });

  it('fires once per pass, not once per step inside the pad', () => {
    // A slow ball spends many steps inside the pad; the edge-triggered check
    // must not treat each of them as a fresh throw.
    const world = makeWorld({
      switches: [{ ...switchPad('sw', V.vec(0, 0), 60, false), on: false }],
    });
    const [ball, runtime] = shoot(V.vec(-200, 0), V.vec(200, 0), 40);
    const events = run(world, ball, runtime, 9);
    expect(events.filter((e) => e.type === 'switch')).toHaveLength(1);
  });
});

describe('gates and bridges', () => {
  const doorWorld = (): World =>
    makeWorld({
      bodies: [gate('door', V.vec(0, -200), V.vec(0, 200), 'sw')],
      switches: [{ ...switchPad('sw', V.vec(-150, 150), 30), on: false }],
    });

  it('blocks the ball while the switch is off', () => {
    const world = doorWorld();
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeLessThan(0);
  });

  it('lets the ball through once the switch is thrown', () => {
    const world = doorWorld();
    world.switches[0]!.on = true;
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeGreaterThan(200);
  });

  it('treats a bridge as the exact inverse of a gate', () => {
    const span = bridge('span', V.vec(0, -200), V.vec(0, 200), 'sw');
    const world = makeWorld({
      bodies: [span],
      switches: [{ ...switchPad('sw', V.vec(-150, 150), 30), on: false }],
    });
    expect(isBodyActive(world, span)).toBe(false);
    world.switches[0]!.on = true;
    expect(isBodyActive(world, span)).toBe(true);
  });

  it('rejects a level whose gate names a switch that does not exist', () => {
    const level: LevelDef = {
      id: 'bad',
      name: 'Bad',
      chapter: 0,
      par: 2,
      tee: V.vec(-200, 0),
      hole: { position: V.vec(200, 0) },
      bounds: { minX: -400, minY: -300, maxX: 400, maxY: 300 },
      bodies: [gate('door', V.vec(0, -100), V.vec(0, 100), 'nope')],
      stars: [],
    };
    const issues = validateLevel(level);
    expect(issues.some((i) => i.severity === 'error' && i.message.includes('nope'))).toBe(true);
  });
});

describe('breakable blocks', () => {
  it('survives the first hit and shatters on the last', () => {
    const world = makeWorld({ bodies: [crystal('block', V.vec(0, 0), 40, 2)] });
    world.breakables = { block: 2 };

    const first = (() => {
      const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
      return run(world, ball, runtime, 1.5);
    })();
    expect(first.some((e) => e.type === 'bounce')).toBe(true);
    expect(first.some((e) => e.type === 'shatter')).toBe(false);
    expect(world.breakables.block).toBe(1);

    const second = (() => {
      const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
      return run(world, ball, runtime, 1.5);
    })();
    const shattered = second.filter((e) => e.type === 'shatter');
    expect(shattered).toHaveLength(1);
    expect(shattered[0]).toMatchObject({ bodyId: 'block' });
    expect(world.breakables.block).toBe(0);
  });

  it('stops colliding once broken, and bumps the shape cache revision', () => {
    const block = crystal('block', V.vec(0, 0), 40, 1);
    const world = makeWorld({ bodies: [block] });
    world.breakables = { block: 1 };
    const before = world.revision ?? 0;

    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);

    expect(isBodyActive(world, block)).toBe(false);
    expect(world.revision ?? 0).toBeGreaterThan(before);
    // It passed straight on through rather than being turned around.
    expect(ball.position.x).toBeGreaterThan(200);
  });

  it('does not chip away at a block from resting contact', () => {
    // Gravity presses a settled ball into whatever it is lying on every step. If
    // that counted as a hit, a crystal floor would crumble under a still ball.
    const world = makeWorld({
      bodies: [crystal('block', V.vec(0, 100), 60, 3)],
      uniformGravity: V.vec(0, 400),
    });
    world.breakables = { block: 3 };
    // Placed already in contact and motionless, so nothing here is an impact.
    const ball = createBall(V.vec(0, 100 - 60 - BALL_RADIUS), BALL_RADIUS);
    const runtime = createBallRuntime();
    run(world, ball, runtime, 4);
    expect(ball.atRest).toBe(true);
    expect(world.breakables.block).toBe(3);
  });
});

describe('locked holes', () => {
  const lockedLevel: LevelDef = {
    id: 'locked',
    name: 'Locked',
    chapter: 0,
    par: 3,
    tee: V.vec(-300, 0),
    hole: { position: V.vec(300, 0), radius: 20, captureSpeed: 400, requiresStars: 2 },
    bounds: { minX: -600, minY: -400, maxX: 600, maxY: 400 },
    boundsMode: 'open',
    ambientDrag: 0,
    stars: [V.vec(-100, 0), V.vec(100, 0)],
  };

  it('refuses the ball and reports what is still owed', () => {
    const world = compileLevel({ ...lockedLevel, stars: [V.vec(0, 500), V.vec(0, -500)] });
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 120);
    const events = run(world, ball, runtime, 6);
    expect(runtime.sunk).toBe(false);
    const locked = events.filter((e) => e.type === 'locked');
    expect(locked.length).toBeGreaterThan(0);
    expect(locked[0]).toMatchObject({ needed: 2 });
  });

  it('accepts the ball once every star is in hand', () => {
    // Stars sit on the line to the cup, so one roll collects both and unlocks it.
    const world = compileLevel(lockedLevel);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 120);
    const events = run(world, ball, runtime, 6);
    expect(events.filter((e) => e.type === 'collect')).toHaveLength(2);
    expect(events.some((e) => e.type === 'locked')).toBe(false);
    expect(runtime.sunk).toBe(true);
  });
});

describe('undo restores board state', () => {
  const level: LevelDef = {
    id: 'undo-switch',
    name: 'Undo Switch',
    chapter: 0,
    par: 3,
    tee: V.vec(-300, 0),
    hole: { position: V.vec(0, 9999), radius: 16, captureSpeed: 400 },
    bounds: { minX: -600, minY: -400, maxX: 600, maxY: 400 },
    boundsMode: 'open',
    ambientDrag: 0.6,
    bodies: [crystal('block', V.vec(200, 0), 30, 3)],
    switches: [switchPad('sw', V.vec(0, 0), 40)],
    stars: [],
  };

  const settle = (session: PlaySession, maxSeconds = 30): void => {
    const steps = Math.round(maxSeconds * 60);
    for (let i = 0; i < steps; i++) {
      if (session.state === 'aiming' && session.canShoot) return;
      session.update(1 / 60);
    }
  };

  it('puts a thrown switch and a chipped block back', () => {
    const session = new PlaySession(level);
    settle(session);
    expect(session.world.switches[0]!.on).toBe(false);
    expect(session.world.breakables.block).toBe(3);

    session.shoot(V.vec(1, 0), 1);
    settle(session);
    expect(session.world.switches[0]!.on).toBe(true);
    expect(session.world.breakables.block).toBe(2);
    expect(session.strokes).toBe(1);

    expect(session.undo()).toBe(true);
    expect(session.world.switches[0]!.on).toBe(false);
    expect(session.world.breakables.block).toBe(3);
    expect(session.strokes).toBe(0);
  });

  it('re-arms the pad so it can be thrown again after an undo', () => {
    const session = new PlaySession(level);
    settle(session);
    session.shoot(V.vec(1, 0), 1);
    settle(session);
    session.undo();
    settle(session);

    const events = [];
    session.shoot(V.vec(1, 0), 1);
    for (let i = 0; i < 600; i++) {
      events.push(...session.update(1 / 60));
      if (session.state === 'aiming' && session.canShoot) break;
    }
    expect(events.filter((e) => e.type === 'switch')).toHaveLength(1);
    expect(session.world.switches[0]!.on).toBe(true);
  });
});

describe('timed switch pads', () => {
  const timedWorld = (holdTime: number): World =>
    makeWorld({
      switches: [{ ...timedPad('sw', V.vec(0, 0), 30, holdTime), on: false }],
      bodies: [gate('door', V.vec(400, -200), V.vec(400, 200), 'sw')],
    });

  it('springs back after the hold expires, on its own', () => {
    const world = timedWorld(1);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);

    // 263 units to the near edge of the pad at 400/s: struck by ~0.66s.
    const early = run(world, ball, runtime, 0.9);
    expect(early.filter((e) => e.type === 'switch')).toHaveLength(1);
    expect(world.switches[0]!.on).toBe(true);

    // The ball is long past the pad by now; nothing but the clock turns it off.
    const later = run(world, ball, runtime, 1.5);
    const off = later.filter((e) => e.type === 'switch');
    expect(off).toHaveLength(1);
    expect(off[0]).toMatchObject({ id: 'sw', on: false });
    expect(world.switches[0]!.on).toBe(false);
  });

  it('holds the gate open long enough for a ball to get through', () => {
    // 400 units at 400/s is one second of travel.
    const world = timedWorld(4);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(500, 0), 400);
    run(world, ball, runtime, 3);
    expect(ball.position.x).toBeGreaterThan(400);
  });

  it('closes the gate in time to stop a slow ball', () => {
    const world = timedWorld(0.4);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(500, 0), 200);
    run(world, ball, runtime, 6);
    expect(world.switches[0]!.on).toBe(false);
    expect(ball.position.x).toBeLessThan(400);
  });

  it('can be thrown again after springing back', () => {
    const world = timedWorld(0.5);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(world.switches[0]!.on).toBe(false);

    launchBall(ball, runtime, V.vec(-400, 0));
    const again = run(world, ball, runtime, 2);
    expect(again.some((e) => e.type === 'switch' && e.on)).toBe(true);
  });
});

describe('multi-switch gating', () => {
  const bothWorld = (): World =>
    makeWorld({
      bodies: [gate('vault', V.vec(0, -200), V.vec(0, 200), ['sw-a', 'sw-b'])],
      switches: [
        { ...switchPad('sw-a', V.vec(-400, 200), 30), on: false },
        { ...switchPad('sw-b', V.vec(-400, -200), 30), on: false },
      ],
    });

  it('stays shut while only one switch is on', () => {
    const world = bothWorld();
    world.switches[0]!.on = true;
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeLessThan(0);
  });

  it('opens only once every switch is on', () => {
    const world = bothWorld();
    world.switches[0]!.on = true;
    world.switches[1]!.on = true;
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeGreaterThan(200);
  });

  it('requires every switch for a bridge too', () => {
    const span = bridge('span', V.vec(0, -200), V.vec(0, 200), ['sw-a', 'sw-b']);
    const world = makeWorld({
      bodies: [span],
      switches: [
        { ...switchPad('sw-a', V.vec(-400, 200), 30), on: true },
        { ...switchPad('sw-b', V.vec(-400, -200), 30), on: false },
      ],
    });
    expect(isBodyActive(world, span)).toBe(false);
    world.switches[1]!.on = true;
    expect(isBodyActive(world, span)).toBe(true);
  });
});

describe('one-way membranes', () => {
  const skinWorld = (): World =>
    makeWorld({ bodies: [membrane('skin', V.vec(0, -200), V.vec(0, 200), V.vec(1, 0))] });

  it('lets the ball through in the allowed direction', () => {
    const world = skinWorld();
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeGreaterThan(200);
  });

  it('stops the ball coming back the other way', () => {
    const world = skinWorld();
    const [ball, runtime] = shoot(V.vec(300, 0), V.vec(-300, 0), 400);
    run(world, ball, runtime, 2);
    expect(ball.position.x).toBeGreaterThan(0);
  });

  it('holds up a ball resting against it', () => {
    // Gravity pushes left, against the membrane, so the ball must not seep past.
    const world = makeWorld({
      bodies: [membrane('skin', V.vec(0, -200), V.vec(0, 200), V.vec(1, 0))],
      uniformGravity: V.vec(-400, 0),
    });
    const ball = createBall(V.vec(40, 0), BALL_RADIUS);
    const runtime = createBallRuntime();
    run(world, ball, runtime, 5);
    expect(ball.position.x).toBeGreaterThan(0);
  });
});

describe('boost rings', () => {
  const ringWorld = (direction: V.Vec2, speed: number): World =>
    makeWorld({ boosters: [booster('ring', V.vec(0, 0), direction, speed, 30)] });

  it('overwrites velocity with its own heading and speed', () => {
    // Absolute, not additive: the whole point is that the exit is a promise the
    // player can plan around regardless of how they arrived.
    const world = ringWorld(V.vec(0, -1), 500);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    const events = run(world, ball, runtime, 0.9);

    const fired = events.filter((e) => e.type === 'boost');
    expect(fired).toHaveLength(1);
    expect(fired[0]).toMatchObject({ id: 'ring', speed: 500 });
    expect(ball.position.y).toBeLessThan(-100);
  });

  it('gives the same exit however fast the ball arrives', () => {
    const exits: V.Vec2[] = [];
    for (const entrySpeed of [120, 400, 900]) {
      const world = ringWorld(V.vec(1, -1), 600);
      const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), entrySpeed);
      // Step until the ring fires, then read the velocity it handed over.
      const steps = Math.round(3 / world.config.timeStep);
      for (let i = 0; i < steps; i++) {
        const events: SimEvent[] = [];
        stepWorld(world, ball, runtime, events);
        if (events.some((e) => e.type === 'boost')) break;
      }
      exits.push(ball.velocity);
    }
    for (const exit of exits) {
      expect(V.length(exit)).toBeCloseTo(600, 3);
      expect(V.angleOf(exit)).toBeCloseTo(V.angleOf(V.vec(1, -1)), 6);
    }
  });

  it('fires once per pass, not once per step inside the ring', () => {
    // The ring points across the ball's path, so it lingers inside. Re-firing
    // every step would trap it there forever.
    const world = ringWorld(V.vec(0, -1), 30);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 200);
    const events = run(world, ball, runtime, 2);
    expect(events.filter((e) => e.type === 'boost')).toHaveLength(1);
  });

  it('re-arms once the ball has left', () => {
    const world = ringWorld(V.vec(0, -1), 400);
    const [ball, runtime] = shoot(V.vec(-300, 0), V.vec(300, 0), 400);
    run(world, ball, runtime, 1.2);
    expect(V.distance(ball.position, V.vec(0, 0))).toBeGreaterThan(200);

    launchBall(ball, runtime, V.normalize(V.sub(V.vec(0, 0), ball.position)));
    ball.position = V.vec(0, -200);
    ball.velocity = V.vec(0, 400);
    const again = run(world, ball, runtime, 1.5);
    expect(again.filter((e) => e.type === 'boost')).toHaveLength(1);
  });

  it('wakes a ball that had come to rest inside it', () => {
    const world = ringWorld(V.vec(1, 0), 350);
    const ball = createBall(V.vec(0, 0), BALL_RADIUS);
    const runtime = createBallRuntime();
    expect(ball.atRest).toBe(true);
    run(world, ball, runtime, 0.5);
    expect(ball.atRest).toBe(false);
    expect(ball.position.x).toBeGreaterThan(100);
  });

  it('rejects a ring with no exit direction or no speed', () => {
    const base: LevelDef = {
      id: 'bad',
      name: 'Bad',
      chapter: 0,
      par: 2,
      tee: V.vec(-200, 0),
      hole: { position: V.vec(200, 0) },
      bounds: { minX: -400, minY: -300, maxX: 400, maxY: 300 },
      stars: [],
    };
    const still = validateLevel({
      ...base,
      boosters: [{ id: 'r', position: V.vec(0, 0), radius: 30, direction: V.ZERO, speed: 400 }],
    });
    expect(still.some((i) => i.severity === 'error' && i.message.includes('exit direction'))).toBe(
      true,
    );

    const dead = validateLevel({
      ...base,
      boosters: [{ id: 'r', position: V.vec(0, 0), radius: 30, direction: V.vec(1, 0), speed: 0 }],
    });
    expect(dead.some((i) => i.severity === 'error' && i.message.includes('exit speed'))).toBe(true);
  });
});

describe('moving cups', () => {
  /** A cup sliding back and forth along y = 0 between x = 200 and x = 600. */
  const slidingHole = (period: number) => ({
    position: V.vec(200, 0),
    motion: {
      kind: 'oscillate' as const,
      from: V.vec(200, 0),
      to: V.vec(600, 0),
      period,
      phase: 0,
    },
    radius: 20,
    captureSpeed: 260,
  });

  it('reports where the cup is now, not where it started', () => {
    const world = makeWorld({ hole: slidingHole(4) });
    expect(holePositionAt(world, 0).x).toBeCloseTo(200, 3);
    expect(holePositionAt(world, 2).x).toBeCloseTo(600, 3);
    expect(holePositionAt(world, 4).x).toBeCloseTo(200, 3);
    // Static holes must be completely unaffected by the clock.
    const still = makeWorld({});
    expect(holePositionAt(still, 99)).toEqual(still.hole.position);
    expect(holeVelocityAt(still, 99)).toEqual(V.ZERO);
  });

  it('captures a ball that meets it, wherever it has got to', () => {
    const world = makeWorld({ hole: slidingHole(4) });
    // Two seconds in the cup is at x = 600, nowhere near where it started.
    world.time = 2;
    const ball = createBall(V.vec(600, 0), BALL_RADIUS);
    const runtime = createBallRuntime();
    const events = run(world, ball, runtime, 0.2);
    expect(events.some((e) => e.type === 'sink')).toBe(true);
    expect(runtime.sunk).toBe(true);
  });

  it('will not swallow a ball parked on its track', () => {
    // The degenerate strategy a moving cup invites: sit on the rails and let the
    // cup drive over you. Capture is measured against the cup, so a fast cup
    // sweeping a still ball is a near miss, not a hole in one.
    const world = makeWorld({ hole: slidingHole(1.4) });
    const ball = createBall(V.vec(400, 0), BALL_RADIUS);
    const runtime = createBallRuntime();
    const events = run(world, ball, runtime, 3);
    expect(runtime.sunk).toBe(false);
    expect(events.some((e) => e.type === 'lipout')).toBe(true);
  });

  it('still lets a slow cup collect a still ball', () => {
    // The same rule, the other way round: creep the cup along and it does drop.
    const world = makeWorld({ hole: slidingHole(30) });
    const ball = createBall(V.vec(220, 0), BALL_RADIUS);
    const runtime = createBallRuntime();
    run(world, ball, runtime, 3);
    expect(runtime.sunk).toBe(true);
  });

  it('rejects a cup whose declared position is not where its path starts', () => {
    const base: LevelDef = {
      id: 'bad',
      name: 'Bad',
      chapter: 0,
      par: 2,
      tee: V.vec(-300, 0),
      hole: {
        position: V.vec(0, 0),
        motion: {
          kind: 'oscillate',
          from: V.vec(100, 0),
          to: V.vec(200, 0),
          period: 4,
          phase: 0,
        },
      },
      bounds: { minX: -400, minY: -300, maxX: 400, maxY: 300 },
      stars: [],
    };
    const issues = validateLevel(base);
    expect(issues.some((i) => i.severity === 'error' && i.message.includes('motion starts'))).toBe(
      true,
    );
  });

  it('rejects a cup that leaves the field part-way round its lap', () => {
    const issues = validateLevel({
      id: 'bad',
      name: 'Bad',
      chapter: 0,
      par: 2,
      tee: V.vec(-300, 0),
      hole: {
        position: V.vec(0, -900),
        motion: { kind: 'orbit', center: V.vec(0, 0), radius: 900, speed: 1, phase: -Math.PI / 2 },
      },
      bounds: { minX: -400, minY: -300, maxX: 400, maxY: 300 },
      stars: [],
    });
    expect(issues.some((i) => i.message.includes('outside the level bounds'))).toBe(true);
  });
});
