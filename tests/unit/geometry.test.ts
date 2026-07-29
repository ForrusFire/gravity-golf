import { describe, expect, it } from 'vitest';
import { TAU } from '../../src/core/math';
import * as V from '../../src/core/vec2';
import {
  bodyShapeAt,
  bodyVelocityAt,
  closestPointOnSegment,
  closestSurfacePoint,
  containsPoint,
  motionOffset,
  polygonSignedArea2,
  rectPolygon,
  regularPolygon,
  shapeBounds,
  shapeCenter,
  shapeRadius,
  withCcwWinding,
  worldVertices,
} from '../../src/physics/geometry';
import type { Body, CapsuleShape, CircleShape } from '../../src/physics/types';

const circle: CircleShape = { kind: 'circle', center: V.vec(0, 0), radius: 10 };

describe('closestPointOnSegment', () => {
  it('projects onto the interior of the segment', () => {
    expect(closestPointOnSegment(V.vec(0, 0), V.vec(10, 0), V.vec(5, 5))).toEqual({ x: 5, y: 0 });
  });

  it('clamps to the endpoints', () => {
    expect(closestPointOnSegment(V.vec(0, 0), V.vec(10, 0), V.vec(-5, 5))).toEqual({ x: 0, y: 0 });
    expect(closestPointOnSegment(V.vec(0, 0), V.vec(10, 0), V.vec(50, 5))).toEqual({ x: 10, y: 0 });
  });

  it('handles a degenerate zero-length segment', () => {
    expect(closestPointOnSegment(V.vec(2, 2), V.vec(2, 2), V.vec(9, 9))).toEqual({ x: 2, y: 2 });
  });
});

describe('closestSurfacePoint — circle', () => {
  it('reports a positive distance outside', () => {
    const q = closestSurfacePoint(circle, V.vec(25, 0));
    expect(q.distance).toBeCloseTo(15, 12);
    expect(q.inside).toBe(false);
    expect(q.normal).toEqual({ x: 1, y: 0 });
    expect(q.point).toEqual({ x: 10, y: 0 });
  });

  it('reports a negative distance inside', () => {
    const q = closestSurfacePoint(circle, V.vec(3, 0));
    expect(q.distance).toBeCloseTo(-7, 12);
    expect(q.inside).toBe(true);
    expect(q.normal).toEqual({ x: 1, y: 0 });
  });

  it('survives a query exactly at the centre', () => {
    const q = closestSurfacePoint(circle, V.vec(0, 0));
    expect(q.distance).toBeCloseTo(-10, 12);
    expect(V.length(q.normal)).toBeCloseTo(1, 12);
  });
});

describe('closestSurfacePoint — capsule', () => {
  const capsule: CapsuleShape = {
    kind: 'capsule',
    a: V.vec(-20, 0),
    b: V.vec(20, 0),
    radius: 5,
  };

  it('measures distance from the shaft', () => {
    const q = closestSurfacePoint(capsule, V.vec(0, 12));
    expect(q.distance).toBeCloseTo(7, 12);
    expect(q.normal).toEqual({ x: 0, y: 1 });
  });

  it('measures distance from a rounded cap', () => {
    const q = closestSurfacePoint(capsule, V.vec(30, 0));
    expect(q.distance).toBeCloseTo(5, 12);
  });

  it('detects points inside the shaft', () => {
    expect(containsPoint(capsule, V.vec(0, 2))).toBe(true);
    expect(containsPoint(capsule, V.vec(0, 8))).toBe(false);
  });
});

describe('polygons', () => {
  it('normalizes winding to counter-clockwise', () => {
    const cw = {
      kind: 'polygon' as const,
      center: V.vec(0, 0),
      rotation: 0,
      vertices: [V.vec(-10, 10), V.vec(10, 10), V.vec(10, -10), V.vec(-10, -10)],
    };
    expect(polygonSignedArea2(cw.vertices)).toBeLessThan(0);
    expect(polygonSignedArea2(withCcwWinding(cw).vertices)).toBeGreaterThan(0);
  });

  it('produces outward normals for an axis-aligned rect', () => {
    const rect = rectPolygon(V.vec(0, 0), 10, 5);
    const right = closestSurfacePoint(rect, V.vec(30, 0));
    expect(right.normal.x).toBeCloseTo(1, 12);
    expect(right.distance).toBeCloseTo(20, 12);

    const above = closestSurfacePoint(rect, V.vec(0, -25));
    expect(above.normal.y).toBeCloseTo(-1, 12);
    expect(above.distance).toBeCloseTo(20, 12);
  });

  it('reports interior points with a negative distance', () => {
    const rect = rectPolygon(V.vec(0, 0), 10, 5);
    const q = closestSurfacePoint(rect, V.vec(0, 0));
    expect(q.inside).toBe(true);
    expect(q.distance).toBeCloseTo(-5, 12);
    expect(containsPoint(rect, V.vec(9, 4))).toBe(true);
    expect(containsPoint(rect, V.vec(11, 0))).toBe(false);
  });

  it('applies rotation to world vertices', () => {
    const rect = rectPolygon(V.vec(0, 0), 10, 5, Math.PI / 2);
    const verts = worldVertices(rect);
    const maxY = Math.max(...verts.map((v) => v.y));
    const maxX = Math.max(...verts.map((v) => v.x));
    expect(maxY).toBeCloseTo(10, 9);
    expect(maxX).toBeCloseTo(5, 9);
  });

  it('keeps normals outward after rotation', () => {
    const rect = rectPolygon(V.vec(0, 0), 10, 5, 0.9);
    for (let a = 0; a < TAU; a += 0.2) {
      const p = V.fromAngle(a, 60);
      const q = closestSurfacePoint(rect, p);
      expect(q.inside).toBe(false);
      // The outward normal must point broadly toward the exterior query point.
      expect(V.dot(q.normal, V.normalize(V.sub(p, rect.center)))).toBeGreaterThan(0);
    }
  });

  it('builds regular polygons with the requested circumradius', () => {
    const hex = regularPolygon(V.vec(5, 5), 20, 6);
    expect(hex.vertices).toHaveLength(6);
    for (const v of hex.vertices) expect(V.length(v)).toBeCloseTo(20, 9);
    expect(shapeRadius(hex)).toBeCloseTo(20, 9);
  });
});

describe('shape helpers', () => {
  it('returns centres', () => {
    expect(shapeCenter(circle)).toEqual({ x: 0, y: 0 });
    expect(
      shapeCenter({ kind: 'capsule', a: V.vec(0, 0), b: V.vec(10, 10), radius: 1 }),
    ).toEqual({ x: 5, y: 5 });
  });

  it('returns radii', () => {
    expect(shapeRadius(circle)).toBe(10);
    expect(shapeRadius({ kind: 'capsule', a: V.vec(-10, 0), b: V.vec(10, 0), radius: 3 })).toBe(13);
  });

  it('computes bounds', () => {
    expect(shapeBounds(circle)).toEqual({ minX: -10, minY: -10, maxX: 10, maxY: 10 });
    const rect = rectPolygon(V.vec(100, 50), 10, 5);
    expect(shapeBounds(rect)).toEqual({ minX: 90, minY: 45, maxX: 110, maxY: 55 });
  });
});

describe('motion', () => {
  it('orbits at the configured radius', () => {
    const offset = motionOffset(
      { kind: 'orbit', center: V.vec(0, 0), radius: 50, speed: 1, phase: 0 },
      0,
    );
    expect(offset).toEqual({ x: 50, y: 0 });
    const quarter = motionOffset(
      { kind: 'orbit', center: V.vec(0, 0), radius: 50, speed: 1, phase: 0 },
      Math.PI / 2,
    );
    expect(quarter.x).toBeCloseTo(0, 9);
    expect(quarter.y).toBeCloseTo(50, 9);
  });

  it('oscillates between endpoints and returns', () => {
    const spec = {
      kind: 'oscillate' as const,
      from: V.vec(0, 0),
      to: V.vec(100, 0),
      period: 4,
      phase: 0,
    };
    expect(motionOffset(spec, 0).x).toBeCloseTo(0, 9);
    expect(motionOffset(spec, 2).x).toBeCloseTo(100, 9);
    expect(motionOffset(spec, 4).x).toBeCloseTo(0, 9);
  });

  it('moves a body shape along its path', () => {
    const body: Body = {
      id: 'mover',
      shape: { kind: 'circle', center: V.vec(0, 0), radius: 10 },
      material: 'rock',
      motion: { kind: 'oscillate', from: V.vec(0, 0), to: V.vec(100, 0), period: 4, phase: 0 },
    };
    expect(shapeCenter(bodyShapeAt(body, 2)).x).toBeCloseTo(100, 9);
  });

  it('spins a polygon about its centre', () => {
    const body: Body = {
      id: 'spinner',
      shape: rectPolygon(V.vec(0, 0), 40, 4),
      material: 'metal',
      motion: { kind: 'spin', speed: 1, phase: 0 },
    };
    const shape = bodyShapeAt(body, Math.PI / 2);
    expect(shape.kind).toBe('polygon');
    const verts = worldVertices(shape as ReturnType<typeof rectPolygon>);
    expect(Math.max(...verts.map((v) => v.y))).toBeCloseTo(40, 6);
  });

  it('derives surface velocity for linear motion', () => {
    const body: Body = {
      id: 'mover',
      shape: { kind: 'circle', center: V.vec(0, 0), radius: 10 },
      material: 'rock',
      motion: { kind: 'oscillate', from: V.vec(0, 0), to: V.vec(100, 0), period: 4, phase: 0 },
    };
    // Fastest at the midpoint of the travel: amplitude(50) * omega(2pi/4).
    const v = bodyVelocityAt(body, 1, V.vec(50, 0));
    expect(v.x).toBeCloseTo(50 * (TAU / 4), 3);
    expect(v.y).toBeCloseTo(0, 6);
  });

  it('derives tangential velocity for spin', () => {
    const body: Body = {
      id: 'spinner',
      shape: rectPolygon(V.vec(0, 0), 40, 4),
      material: 'metal',
      motion: { kind: 'spin', speed: 2, phase: 0 },
    };
    const v = bodyVelocityAt(body, 0, V.vec(10, 0));
    expect(v.x).toBeCloseTo(0, 9);
    expect(v.y).toBeCloseTo(20, 9);
  });

  it('reports zero surface velocity for static bodies', () => {
    const body: Body = { id: 'static', shape: circle, material: 'rock' };
    expect(bodyVelocityAt(body, 3, V.vec(10, 0))).toEqual({ x: 0, y: 0 });
  });
});
