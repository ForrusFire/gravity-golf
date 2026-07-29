import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';

describe('vec2', () => {
  it('adds, subtracts and scales', () => {
    expect(V.add(V.vec(1, 2), V.vec(3, 4))).toEqual({ x: 4, y: 6 });
    expect(V.sub(V.vec(1, 2), V.vec(3, 4))).toEqual({ x: -2, y: -2 });
    expect(V.mul(V.vec(1, -2), 3)).toEqual({ x: 3, y: -6 });
    expect(V.addScaled(V.vec(1, 1), V.vec(2, 3), 2)).toEqual({ x: 5, y: 7 });
  });

  it('computes dot and cross products', () => {
    expect(V.dot(V.vec(1, 0), V.vec(0, 1))).toBe(0);
    expect(V.dot(V.vec(2, 3), V.vec(4, 5))).toBe(23);
    expect(V.cross(V.vec(1, 0), V.vec(0, 1))).toBe(1);
  });

  it('measures length and distance', () => {
    expect(V.length(V.vec(3, 4))).toBe(5);
    expect(V.lengthSq(V.vec(3, 4))).toBe(25);
    expect(V.distance(V.vec(1, 1), V.vec(4, 5))).toBe(5);
    expect(V.distanceSq(V.vec(1, 1), V.vec(4, 5))).toBe(25);
  });

  it('normalizes and returns zero for degenerate input', () => {
    expect(V.normalize(V.vec(10, 0))).toEqual({ x: 1, y: 0 });
    expect(V.normalize(V.vec(0, 0))).toEqual({ x: 0, y: 0 });
    expect(V.length(V.normalize(V.vec(3, 4)))).toBeCloseTo(1, 12);
  });

  it('clamps length only when longer than the limit', () => {
    expect(V.clampLength(V.vec(3, 4), 10)).toEqual({ x: 3, y: 4 });
    const clamped = V.clampLength(V.vec(30, 40), 5);
    expect(V.length(clamped)).toBeCloseTo(5, 12);
    expect(clamped.x / clamped.y).toBeCloseTo(0.75, 12);
  });

  it('rotates by an angle and round-trips', () => {
    const rotated = V.rotate(V.vec(1, 0), Math.PI / 2);
    expect(rotated.x).toBeCloseTo(0, 12);
    expect(rotated.y).toBeCloseTo(1, 12);
    const back = V.rotate(rotated, -Math.PI / 2);
    expect(V.equalsApprox(back, V.vec(1, 0), 1e-12)).toBe(true);
  });

  it('builds vectors from angles and reads them back', () => {
    const v = V.fromAngle(0.7, 3);
    expect(V.length(v)).toBeCloseTo(3, 12);
    expect(V.angleOf(v)).toBeCloseTo(0.7, 12);
  });

  it('reflects about a normal, preserving speed', () => {
    const reflected = V.reflect(V.vec(1, -1), V.vec(0, 1));
    expect(reflected.x).toBeCloseTo(1, 12);
    expect(reflected.y).toBeCloseTo(1, 12);
    expect(V.length(reflected)).toBeCloseTo(V.length(V.vec(1, -1)), 12);
  });

  it('perp is orthogonal and preserves length', () => {
    const p = V.perp(V.vec(3, 4));
    expect(V.dot(p, V.vec(3, 4))).toBeCloseTo(0, 12);
    expect(V.length(p)).toBeCloseTo(5, 12);
  });

  it('lerps between endpoints', () => {
    expect(V.lerpVec(V.vec(0, 0), V.vec(10, 20), 0.5)).toEqual({ x: 5, y: 10 });
    expect(V.lerpVec(V.vec(0, 0), V.vec(10, 20), 0)).toEqual({ x: 0, y: 0 });
    expect(V.lerpVec(V.vec(0, 0), V.vec(10, 20), 1)).toEqual({ x: 10, y: 20 });
  });
});
