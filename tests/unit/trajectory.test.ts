import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { predictTrajectory, trajectoryLength } from '../../src/physics/trajectory';
import { DEFAULT_PHYSICS } from '../../src/physics/types';
import {
  createBall,
  createBallRuntime,
  createWorld,
  stepWorld,
  type World,
} from '../../src/physics/world';

const makeWorld = (overrides: Partial<World> = {}): World =>
  createWorld({
    hole: { position: V.vec(9999, 9999), radius: 12, captureSpeed: 200 },
    bounds: { minX: -3000, minY: -3000, maxX: 3000, maxY: 3000 },
    config: { ...DEFAULT_PHYSICS, ambientDrag: 0 },
    ...overrides,
  });

describe('predictTrajectory', () => {
  it('traces a straight line in empty space', () => {
    const world = makeWorld();
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(300, 0), { maxTime: 1 });
    expect(path.points.length).toBeGreaterThan(3);
    for (const p of path.points) expect(p.y).toBeCloseTo(0, 6);
    expect(path.points[path.points.length - 1]!.x).toBeGreaterThan(200);
  });

  it('curves toward a planet', () => {
    const world = makeWorld({
      bodies: [
        {
          id: 'p',
          shape: { kind: 'circle', center: V.vec(300, 300), radius: 80 },
          material: 'rock',
          gravity: { strength: 900, range: 0 },
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(300, 0), { maxTime: 1 });
    const last = path.points[path.points.length - 1]!;
    expect(last.y).toBeGreaterThan(5);
  });

  it('reports a sink outcome when the shot goes in', () => {
    const world = makeWorld({
      hole: { position: V.vec(200, 0), radius: 14, captureSpeed: 300 },
    });
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(200, 0), { maxTime: 3 });
    expect(path.outcome).toBe('sink');
  });

  it('reports a death outcome when the shot is fatal', () => {
    const world = makeWorld({
      bodies: [
        {
          id: 'sun',
          shape: { kind: 'circle', center: V.vec(200, 0), radius: 50 },
          material: 'lava',
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(400, 0), { maxTime: 3 });
    expect(path.outcome).toBe('death');
  });

  it('truncates at the look-ahead limit', () => {
    const world = makeWorld();
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(100, 0), { maxTime: 0.5 });
    expect(path.outcome).toBe('truncated');
    expect(path.duration).toBeCloseTo(0.5, 2);
  });

  it('leaves the world and ball untouched', () => {
    const world = makeWorld({
      collectibles: [{ id: 's', position: V.vec(150, 0), radius: 14, collected: false }],
    });
    const ball = createBall(V.vec(0, 0));
    const timeBefore = world.time;
    predictTrajectory(world, ball, V.vec(400, 0), { maxTime: 2 });
    expect(world.time).toBe(timeBefore);
    expect(world.collectibles[0]!.collected).toBe(false);
    expect(ball.position).toEqual(V.vec(0, 0));
    expect(ball.velocity).toEqual(V.ZERO);
  });

  it('is deterministic', () => {
    const world = makeWorld({
      bodies: [
        {
          id: 'p',
          shape: { kind: 'circle', center: V.vec(400, 120), radius: 90 },
          material: 'rock',
          gravity: { strength: 800, range: 0 },
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const a = predictTrajectory(world, ball, V.vec(350, -60), { maxTime: 2 });
    const b = predictTrajectory(world, ball, V.vec(350, -60), { maxTime: 2 });
    expect(a.points).toEqual(b.points);
    expect(a.outcome).toBe(b.outcome);
  });

  it('measures path length', () => {
    const world = makeWorld();
    const ball = createBall(V.vec(0, 0));
    const path = predictTrajectory(world, ball, V.vec(100, 0), { maxTime: 1 });
    expect(trajectoryLength(path)).toBeCloseTo(100, 0);
  });

  it('predicts the same path the simulation produces', () => {
    const world = makeWorld({
      bodies: [
        {
          id: 'p',
          shape: { kind: 'circle', center: V.vec(300, 250), radius: 70 },
          material: 'rock',
          gravity: { strength: 700, range: 0 },
        },
      ],
    });
    const ball = createBall(V.vec(0, 0));
    const predicted = predictTrajectory(world, ball, V.vec(400, -50), {
      maxTime: 1,
      timeStep: DEFAULT_PHYSICS.timeStep,
      sampleEvery: 1,
    });
    const end = predicted.points[predicted.points.length - 1]!;

    // Same impulse through the live stepper must land in the same place.
    const liveBall = createBall(V.vec(0, 0));
    const rt = createBallRuntime();
    liveBall.velocity = V.vec(400, -50);
    liveBall.atRest = false;
    const steps = Math.round(1 / DEFAULT_PHYSICS.timeStep);
    for (let i = 0; i < steps; i++) stepWorld(world, liveBall, rt, []);
    expect(liveBall.position.x).toBeCloseTo(end.x, 3);
    expect(liveBall.position.y).toBeCloseTo(end.y, 3);
  });
});
