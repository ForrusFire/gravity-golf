import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { bodyShapeAt, shapeCenter, shapeRadius } from '../../src/physics/geometry';
import { BALL_RADIUS } from '../../src/game/level';
import { createShowcase } from '../../src/game/showcase';

describe('the title-screen showcase', () => {
  it('never flies the ball through a body, or lets two bodies overlap', () => {
    // It is drawn, not simulated, so nothing else stops it clipping a planet.
    const showcase = createShowcase();
    const bodies = showcase.world.bodies;
    for (let t = 0; t < 120; t += 0.05) {
      const ball = showcase.ballAt(t);
      const shapes = bodies.map((b) => bodyShapeAt(b, t));
      for (const shape of shapes) {
        const gap = V.distance(ball, shapeCenter(shape)) - shapeRadius(shape) - BALL_RADIUS;
        expect(gap).toBeGreaterThan(4);
      }
      for (let i = 0; i < shapes.length; i++) {
        for (let j = i + 1; j < shapes.length; j++) {
          const a = shapes[i]!;
          const b = shapes[j]!;
          const gap = V.distance(shapeCenter(a), shapeCenter(b)) - shapeRadius(a) - shapeRadius(b);
          expect(gap).toBeGreaterThan(0);
        }
      }
    }
  });

  it('loops: the ball comes back to where it started', () => {
    const showcase = createShowcase();
    const start = showcase.ballAt(0);
    const later = showcase.ballAt(10);
    expect(V.distance(start, later)).toBeLessThan(0.01);
    // And the trail ends at the ball.
    const trail = showcase.trailAt(3);
    expect(V.distance(trail[trail.length - 1]!, showcase.ballAt(3))).toBeLessThan(1e-9);
  });
});
