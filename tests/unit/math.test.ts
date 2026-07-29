import { describe, expect, it } from 'vitest';
import {
  angleDelta,
  clamp,
  clamp01,
  damp,
  invLerp,
  lerp,
  remap,
  smoothstep,
  wrapAngle,
} from '../../src/core/math';
import { Rng, hashSeed } from '../../src/core/rng';

describe('math helpers', () => {
  it('clamps', () => {
    expect(clamp(5, 0, 10)).toBe(5);
    expect(clamp(-5, 0, 10)).toBe(0);
    expect(clamp(50, 0, 10)).toBe(10);
    expect(clamp01(1.5)).toBe(1);
  });

  it('lerps and inverse-lerps consistently', () => {
    expect(lerp(0, 10, 0.25)).toBe(2.5);
    expect(invLerp(0, 10, 2.5)).toBe(0.25);
    expect(invLerp(5, 5, 5)).toBe(0);
  });

  it('remaps across ranges and clamps outside them', () => {
    expect(remap(5, 0, 10, 0, 100)).toBe(50);
    expect(remap(-5, 0, 10, 0, 100)).toBe(0);
    expect(remap(15, 0, 10, 0, 100)).toBe(100);
  });

  it('smoothstep is clamped and symmetric about the midpoint', () => {
    expect(smoothstep(0)).toBe(0);
    expect(smoothstep(1)).toBe(1);
    expect(smoothstep(-1)).toBe(0);
    expect(smoothstep(0.5)).toBeCloseTo(0.5, 12);
    expect(smoothstep(0.25) + smoothstep(0.75)).toBeCloseTo(1, 12);
  });

  it('wraps angles into (-PI, PI]', () => {
    expect(wrapAngle(0)).toBeCloseTo(0, 12);
    expect(wrapAngle(Math.PI * 3)).toBeCloseTo(Math.PI, 12);
    expect(wrapAngle(-Math.PI * 3)).toBeCloseTo(Math.PI, 12);
    expect(angleDelta(0.1, 0.2)).toBeCloseTo(0.1, 12);
    expect(angleDelta(3.1, -3.1)).toBeCloseTo(0.0831853, 5);
  });

  it('damp approaches the target and is frame-rate independent', () => {
    const oneBigStep = damp(0, 100, 5, 1);
    let stepped = 0;
    for (let i = 0; i < 100; i++) stepped = damp(stepped, 100, 5, 0.01);
    expect(stepped).toBeCloseTo(oneBigStep, 9);
  });
});

describe('Rng', () => {
  it('is deterministic for a given seed', () => {
    const a = new Rng(1234);
    const b = new Rng(1234);
    for (let i = 0; i < 50; i++) expect(a.next()).toBe(b.next());
  });

  it('differs across seeds', () => {
    const a = new Rng(1);
    const b = new Rng(2);
    expect(a.next()).not.toBe(b.next());
  });

  it('stays inside [0, 1)', () => {
    const rng = new Rng(99);
    for (let i = 0; i < 2000; i++) {
      const v = rng.next();
      expect(v).toBeGreaterThanOrEqual(0);
      expect(v).toBeLessThan(1);
    }
  });

  it('generates integers within the inclusive range', () => {
    const rng = new Rng(7);
    const seen = new Set<number>();
    for (let i = 0; i < 500; i++) {
      const v = rng.int(3, 6);
      expect(Number.isInteger(v)).toBe(true);
      expect(v).toBeGreaterThanOrEqual(3);
      expect(v).toBeLessThanOrEqual(6);
      seen.add(v);
    }
    expect(seen.size).toBe(4);
  });

  it('picks from arrays and rejects empty ones', () => {
    const rng = new Rng(11);
    expect(['a', 'b']).toContain(rng.pick(['a', 'b']));
    expect(() => rng.pick([])).toThrow();
  });

  it('hashes strings to stable seeds', () => {
    expect(hashSeed('level-1')).toBe(hashSeed('level-1'));
    expect(hashSeed('level-1')).not.toBe(hashSeed('level-2'));
  });
});
