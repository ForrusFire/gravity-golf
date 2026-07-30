import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import type { Vec2 } from '../../src/core/vec2';
import { DEFAULT_PHYSICS, type Ball } from '../../src/physics/types';
import {
  createBall,
  createBallRuntime,
  createWorld,
  isBodyActive,
  launchBall,
  stepWorld,
  type BallRuntime,
  type SimEvent,
  type World,
} from '../../src/physics/world';
import {
  BALL_RADIUS,
  bridge,
  compileLevel,
  crystal,
  gate,
  switchPad,
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
