import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import type { Vec2 } from '../../src/core/vec2';
import { rectPolygon } from '../../src/physics/geometry';
import { DEFAULT_PHYSICS, type Ball, type Body } from '../../src/physics/types';
import {
  accelerationAt,
  createBall,
  createBallRuntime,
  createWorld,
  gravityAt,
  launchBall,
  stepWorld,
  type BallRuntime,
  type SimEvent,
  type World,
} from '../../src/physics/world';

const planet = (center: Vec2, radius: number, strength: number, range = 0): Body => ({
  id: `planet-${center.x}-${center.y}`,
  shape: { kind: 'circle', center, radius },
  material: 'rock',
  gravity: { strength, range },
});

const makeWorld = (overrides: Partial<World> = {}): World =>
  createWorld({
    hole: { position: V.vec(9999, 9999), radius: 12, captureSpeed: 200 },
    bounds: { minX: -2000, minY: -2000, maxX: 2000, maxY: 2000 },
    config: { ...DEFAULT_PHYSICS, ambientDrag: 0 },
    ...overrides,
  });

/** Runs the world for `seconds`, returning every event emitted. */
const run = (
  world: World,
  ball: Ball,
  runtime: BallRuntime,
  seconds: number,
): SimEvent[] => {
  const events: SimEvent[] = [];
  const steps = Math.round(seconds / world.config.timeStep);
  for (let i = 0; i < steps; i++) stepWorld(world, ball, runtime, events);
  return events;
};

describe('gravity field', () => {
  it('equals the configured strength at the surface', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), 100, 400)] });
    const a = gravityAt(world, V.vec(100, 0));
    expect(V.length(a)).toBeCloseTo(400, 6);
    expect(a.x).toBeCloseTo(-400, 6);
  });

  it('falls off with the inverse square of distance', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), 100, 400)] });
    expect(V.length(gravityAt(world, V.vec(200, 0)))).toBeCloseTo(100, 6);
    expect(V.length(gravityAt(world, V.vec(400, 0)))).toBeCloseTo(25, 6);
  });

  it('stays finite at the body centre', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), 100, 400)] });
    const a = gravityAt(world, V.vec(0.0001, 0));
    expect(Number.isFinite(V.length(a))).toBe(true);
    expect(V.length(a)).toBeLessThan(400 / (0.35 * 0.35) + 1);
  });

  it('repels when strength is negative', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), 100, -400)] });
    expect(gravityAt(world, V.vec(100, 0)).x).toBeCloseTo(400, 6);
  });

  it('sums contributions from several bodies and cancels at a Lagrange point', () => {
    const world = makeWorld({
      bodies: [planet(V.vec(-100, 0), 50, 300), planet(V.vec(100, 0), 50, 300)],
    });
    expect(V.length(gravityAt(world, V.vec(0, 0)))).toBeCloseTo(0, 6);
  });

  it('respects a finite range', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), 50, 400, 300)] });
    expect(V.length(gravityAt(world, V.vec(400, 0)))).toBe(0);
    // Inside the fade band but not yet cut off.
    expect(V.length(gravityAt(world, V.vec(250, 0)))).toBeGreaterThan(0);
  });

  it('adds uniform gravity everywhere, independent of distance', () => {
    const world = makeWorld({ uniformGravity: V.vec(0, 400) });
    expect(gravityAt(world, V.vec(0, 0))).toEqual({ x: 0, y: 400 });
    expect(gravityAt(world, V.vec(99999, -99999))).toEqual({ x: 0, y: 400 });
  });

  it('sums uniform gravity with body gravity', () => {
    const world = makeWorld({
      uniformGravity: V.vec(0, 100),
      bodies: [planet(V.vec(0, 0), 100, 400)],
    });
    // At the surface directly above, the body pulls down 400 and the uniform
    // field adds another 100.
    expect(gravityAt(world, V.vec(0, -100)).y).toBeCloseTo(500, 6);
  });

  it('scales uniform gravity inside a gravityScale zone', () => {
    const world = makeWorld({
      uniformGravity: V.vec(0, 400),
      zones: [
        {
          id: 'null',
          kind: 'gravityScale',
          area: { kind: 'circle', center: V.vec(0, 0), radius: 50 },
          scale: 0,
        },
      ],
    });
    expect(gravityAt(world, V.vec(0, 0)).y).toBe(0);
    expect(gravityAt(world, V.vec(500, 0)).y).toBeCloseTo(400, 6);
  });

  it('is scaled by gravityScale zones', () => {
    const world = makeWorld({
      bodies: [planet(V.vec(0, 0), 100, 400)],
      zones: [
        {
          id: 'null',
          kind: 'gravityScale',
          area: { kind: 'circle', center: V.vec(300, 0), radius: 60 },
          scale: 0,
        },
      ],
    });
    expect(V.length(gravityAt(world, V.vec(300, 0)))).toBe(0);
    expect(V.length(gravityAt(world, V.vec(500, 0)))).toBeGreaterThan(0);
  });
});

describe('force zones', () => {
  it('applies wind inside its area only', () => {
    const world = makeWorld({
      zones: [
        {
          id: 'wind',
          kind: 'wind',
          area: rectPolygon(V.vec(0, 0), 100, 100),
          force: V.vec(50, 0),
        },
      ],
    });
    expect(accelerationAt(world, V.vec(0, 0)).x).toBeCloseTo(50, 9);
    expect(accelerationAt(world, V.vec(500, 0)).x).toBeCloseTo(0, 9);
  });

  it('pushes outward from a vortex and swirls tangentially', () => {
    const world = makeWorld({
      zones: [
        {
          id: 'vortex',
          kind: 'vortex',
          area: { kind: 'circle', center: V.vec(0, 0), radius: 200 },
          strength: 100,
          swirl: 50,
        },
      ],
    });
    const a = accelerationAt(world, V.vec(50, 0));
    expect(a.x).toBeCloseTo(100, 9);
    expect(a.y).toBeCloseTo(50, 9);
  });

  it('nebula drag slows the ball faster than open space', () => {
    const nebula = makeWorld({
      zones: [
        {
          id: 'neb',
          kind: 'nebula',
          area: { kind: 'circle', center: V.vec(0, 0), radius: 5000 },
          drag: 3,
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(300, 0));
    run(nebula, ball, rt, 0.5);
    expect(V.length(ball.velocity)).toBeLessThan(300 * Math.exp(-3 * 0.5) + 5);
    expect(V.length(ball.velocity)).toBeGreaterThan(0);
  });
});

describe('ball motion', () => {
  it('falls toward a planet', () => {
    // Surface gravity 500 at r=60 gives ~20 units/s^2 at 300 away, so the drop
    // over 0.4s is about 0.5 * 20 * 0.16 = 1.6 units.
    const world = makeWorld({ bodies: [planet(V.vec(0, 300), 60, 500)] });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    ball.atRest = false;
    run(world, ball, rt, 0.4);
    expect(ball.position.y).toBeCloseTo(1.6, 1);
    expect(ball.velocity.y).toBeCloseTo(8, 0);
  });

  it('holds a circular orbit without spiralling in or out', () => {
    // v = sqrt(g_surface * R^2 / r) for a circular orbit at radius r.
    const R = 80;
    const g = 600;
    const r = 260;
    const speed = Math.sqrt((g * R * R) / r);
    const world = makeWorld({ bodies: [planet(V.vec(0, 0), R, g)] });
    const ball = createBall(V.vec(r, 0), 6);
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(0, speed));

    let minR = Infinity;
    let maxR = 0;
    const steps = Math.round(6 / world.config.timeStep);
    const events: SimEvent[] = [];
    for (let i = 0; i < steps; i++) {
      stepWorld(world, ball, rt, events);
      const d = V.length(ball.position);
      minR = Math.min(minR, d);
      maxR = Math.max(maxR, d);
    }
    // Leapfrog integration should keep the orbit within a couple of percent.
    expect(minR).toBeGreaterThan(r * 0.97);
    expect(maxR).toBeLessThan(r * 1.03);
    expect(rt.alive).toBe(true);
  });

  it('bounces off a surface, losing energy to restitution', () => {
    const world = makeWorld({ bodies: [{ ...planet(V.vec(0, 200), 60, 0), material: 'rock' }] });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(0, 400));
    const events = run(world, ball, rt, 1.5);
    const bounces = events.filter((e) => e.type === 'bounce');
    expect(bounces.length).toBeGreaterThan(0);
    expect(ball.velocity.y).toBeLessThan(0);
    expect(V.length(ball.velocity)).toBeLessThan(400);
  });

  it('bounces higher off a bouncy surface than off sand', () => {
    const speedAfter = (material: 'bouncy' | 'sand'): number => {
      const world = makeWorld({ bodies: [{ ...planet(V.vec(0, 200), 60, 0), material }] });
      const ball = createBall(V.vec(0, 0));
      const rt = createBallRuntime();
      launchBall(ball, rt, V.vec(0, 400));
      run(world, ball, rt, 0.5);
      return Math.abs(ball.velocity.y);
    };
    expect(speedAfter('bouncy')).toBeGreaterThan(speedAfter('sand'));
  });

  it('comes to rest on a planet and emits a rest event', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 300), 100, 500)] });
    const ball = createBall(V.vec(0, 150));
    const rt = createBallRuntime();
    ball.atRest = false;
    const events = run(world, ball, rt, 12);
    expect(events.some((e) => e.type === 'rest')).toBe(true);
    expect(ball.atRest).toBe(true);
    expect(V.length(ball.velocity)).toBe(0);
    // Resting on the surface, not sunk into it.
    expect(V.distance(ball.position, V.vec(0, 300))).toBeCloseTo(100 + ball.radius, 0);
  });

  it('never sinks below a surface it is resting on', () => {
    const world = makeWorld({ bodies: [planet(V.vec(0, 400), 120, 800)] });
    const ball = createBall(V.vec(0, 100));
    const rt = createBallRuntime();
    ball.atRest = false;
    const events: SimEvent[] = [];
    const steps = Math.round(15 / world.config.timeStep);
    for (let i = 0; i < steps; i++) {
      stepWorld(world, ball, rt, events);
      expect(V.distance(ball.position, V.vec(0, 400))).toBeGreaterThan(120 - 1);
    }
  });

  it('does not tunnel through a thin wall at high speed', () => {
    const wall: Body = {
      id: 'wall',
      shape: { kind: 'capsule', a: V.vec(-200, 300), b: V.vec(200, 300), radius: 4 },
      material: 'rock',
    };
    const world = makeWorld({ bodies: [wall] });
    const ball = createBall(V.vec(0, 0), 7);
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(0, 2000));
    run(world, ball, rt, 1);
    expect(ball.position.y).toBeLessThan(300);
  });

  it('clamps speed to maxSpeed', () => {
    const world = makeWorld();
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(99999, 0));
    run(world, ball, rt, 0.1);
    expect(V.length(ball.velocity)).toBeLessThanOrEqual(world.config.maxSpeed + 1e-6);
  });

  it('tracks per-shot distance and peak speed', () => {
    const world = makeWorld();
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(200, 0));
    run(world, ball, rt, 1);
    expect(rt.distance).toBeCloseTo(200, 0);
    expect(rt.peakSpeed).toBeCloseTo(200, 0);
  });
});

describe('moving bodies', () => {
  it('transfers momentum from a moving platform', () => {
    const platform: Body = {
      id: 'platform',
      shape: rectPolygon(V.vec(0, 200), 120, 10),
      material: 'bouncy',
      motion: { kind: 'oscillate', from: V.vec(0, 200), to: V.vec(400, 200), period: 4, phase: 0 },
    };
    const world = makeWorld({ bodies: [platform] });
    const ball = createBall(V.vec(20, 150));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(0, 300));
    run(world, ball, rt, 0.6);
    // The platform sweeps in +x, so the ball should be carried that way.
    expect(ball.velocity.x).toBeGreaterThan(0);
  });

  it('flings the ball off a spinning bar', () => {
    const bar: Body = {
      id: 'bar',
      shape: rectPolygon(V.vec(0, 0), 100, 8),
      material: 'metal',
      motion: { kind: 'spin', speed: 6, phase: 0 },
    };
    const world = makeWorld({ bodies: [bar] });
    const ball = createBall(V.vec(60, -40));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(0, 60));
    run(world, ball, rt, 1.2);
    expect(V.length(ball.velocity)).toBeGreaterThan(60);
  });
});

describe('hole', () => {
  it('captures a slow ball', () => {
    const world = makeWorld({
      hole: { position: V.vec(200, 0), radius: 14, captureSpeed: 220 },
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(150, 0));
    const events = run(world, ball, rt, 3);
    expect(events.some((e) => e.type === 'sink')).toBe(true);
    expect(rt.sunk).toBe(true);
    expect(ball.position).toEqual(V.vec(200, 0));
  });

  it('lips out a fast ball', () => {
    const world = makeWorld({
      hole: { position: V.vec(200, 0), radius: 14, captureSpeed: 120 },
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(900, 0));
    const events = run(world, ball, rt, 1);
    expect(events.some((e) => e.type === 'lipout')).toBe(true);
    expect(rt.sunk).toBe(false);
  });

  it('stops simulating once sunk', () => {
    const world = makeWorld({ hole: { position: V.vec(100, 0), radius: 14, captureSpeed: 300 } });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(150, 0));
    run(world, ball, rt, 2);
    const sunkAt = ball.position;
    run(world, ball, rt, 2);
    expect(ball.position).toEqual(sunkAt);
  });
});

describe('hazards and bounds', () => {
  it('kills the ball on a deadly material', () => {
    const sun: Body = {
      id: 'sun',
      shape: { kind: 'circle', center: V.vec(200, 0), radius: 50 },
      material: 'lava',
    };
    const world = makeWorld({ bodies: [sun] });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(400, 0));
    const events = run(world, ball, rt, 2);
    const death = events.find((e) => e.type === 'death');
    expect(death).toBeDefined();
    expect(death && death.type === 'death' && death.cause).toBe('hazard');
    expect(rt.alive).toBe(false);
  });

  it('kills the ball inside a hazard zone', () => {
    const world = makeWorld({
      zones: [
        { id: 'spikes', kind: 'hazard', area: rectPolygon(V.vec(300, 0), 40, 200) },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(400, 0));
    const events = run(world, ball, rt, 2);
    expect(events.some((e) => e.type === 'death')).toBe(true);
  });

  it('reports out-of-bounds when leaving a kill boundary', () => {
    const world = makeWorld({
      bounds: { minX: -300, minY: -300, maxX: 300, maxY: 300 },
      boundsMode: 'kill',
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(600, 0));
    const events = run(world, ball, rt, 2);
    const death = events.find((e) => e.type === 'death');
    expect(death && death.type === 'death' && death.cause).toBe('out-of-bounds');
  });

  it('keeps the ball inside a walled boundary', () => {
    const world = makeWorld({
      bounds: { minX: -300, minY: -300, maxX: 300, maxY: 300 },
      boundsMode: 'wall',
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(600, 220));
    run(world, ball, rt, 4);
    expect(rt.alive).toBe(true);
    expect(ball.position.x).toBeLessThanOrEqual(300);
    expect(ball.position.x).toBeGreaterThanOrEqual(-300);
    expect(ball.position.y).toBeLessThanOrEqual(300);
    expect(ball.position.y).toBeGreaterThanOrEqual(-300);
  });

  it('never leaves the world in open bounds mode', () => {
    const world = makeWorld({ boundsMode: 'open' });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(600, 0));
    run(world, ball, rt, 3);
    expect(rt.alive).toBe(true);
    // Travels straight past the boundary at x=2000 without dying.
    expect(ball.position.x).toBeCloseTo(1800, 0);
  });
});

describe('portals and collectibles', () => {
  it('teleports the ball and preserves speed', () => {
    const world = makeWorld({
      portals: [
        {
          id: 'p1',
          from: V.vec(200, 0),
          to: V.vec(-500, 0),
          radius: 20,
          twist: 0,
          boost: 1,
          bidirectional: false,
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(300, 0));
    const events = run(world, ball, rt, 1);
    expect(events.some((e) => e.type === 'portal')).toBe(true);
    expect(ball.position.x).toBeLessThan(0);
    expect(V.length(ball.velocity)).toBeCloseTo(300, 0);
  });

  it('applies twist and boost on exit', () => {
    const world = makeWorld({
      portals: [
        {
          id: 'p1',
          from: V.vec(200, 0),
          to: V.vec(-500, 0),
          radius: 20,
          twist: Math.PI / 2,
          boost: 2,
          bidirectional: false,
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(300, 0));
    run(world, ball, rt, 0.9);
    expect(V.length(ball.velocity)).toBeGreaterThan(500);
    expect(Math.abs(ball.velocity.y)).toBeGreaterThan(Math.abs(ball.velocity.x));
  });

  it('does not immediately re-enter an overlapping portal pair', () => {
    const world = makeWorld({
      portals: [
        {
          id: 'p1',
          from: V.vec(100, 0),
          to: V.vec(140, 0),
          radius: 30,
          twist: 0,
          boost: 1,
          bidirectional: true,
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(300, 0));
    const events = run(world, ball, rt, 2);
    // Cooldown must stop an infinite ping-pong between the two mouths.
    expect(events.filter((e) => e.type === 'portal').length).toBeLessThan(6);
    expect(rt.alive).toBe(true);
  });

  it('collects stars once', () => {
    const world = makeWorld({
      collectibles: [{ id: 's1', position: V.vec(200, 0), radius: 14, collected: false }],
    });
    const ball = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    launchBall(ball, rt, V.vec(300, 0));
    const events = run(world, ball, rt, 3);
    expect(events.filter((e) => e.type === 'collect')).toHaveLength(1);
    expect(world.collectibles[0]!.collected).toBe(true);
  });
});

describe('determinism', () => {
  it('produces identical results for identical inputs', () => {
    const build = () => {
      const world = makeWorld({
        bodies: [
          planet(V.vec(300, 100), 70, 700),
          {
            id: 'mover',
            shape: rectPolygon(V.vec(150, -100), 80, 10),
            material: 'bouncy',
            motion: {
              kind: 'oscillate',
              from: V.vec(150, -100),
              to: V.vec(150, 100),
              period: 3,
              phase: 0,
            },
          },
        ],
      });
      const ball = createBall(V.vec(0, 0));
      const rt = createBallRuntime();
      launchBall(ball, rt, V.vec(420, 30));
      return { world, ball, rt };
    };

    const a = build();
    const b = build();
    const eventsA = run(a.world, a.ball, a.rt, 4);
    const eventsB = run(b.world, b.ball, b.rt, 4);

    expect(a.ball.position).toEqual(b.ball.position);
    expect(a.ball.velocity).toEqual(b.ball.velocity);
    expect(eventsA.length).toBe(eventsB.length);
  });
});
