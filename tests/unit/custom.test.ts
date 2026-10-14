import { describe, expect, it } from 'vitest';
import {
  CUSTOM_LIMITS,
  customToLevel,
  decodeCustomHole,
  emptyCustomHole,
  encodeCustomHole,
  sanitizeCustomHole,
} from '../../src/game/custom';
import { validateLevel } from '../../src/game/level';
import { solveLevel } from '../../src/game/solver';

describe('custom holes', () => {
  it('round-trips through a link code', () => {
    const hole = emptyCustomHole();
    hole.name = 'Über hole ✨';
    hole.items.push({ t: 'wall', x1: -100, y1: -50, x2: 100, y2: -50 });
    hole.stars.push([0, -200]);
    const code = encodeCustomHole(hole);
    expect(code).toMatch(/^[A-Za-z0-9_-]+$/);
    expect(decodeCustomHole(code)).toEqual(hole);
  });

  it('rejects anything that is not a hole, without throwing', () => {
    for (const junk of ['', 'abc', '!!!!!!!!!!', 'eyJ0ZWUiOjF9', 'x'.repeat(7000)]) {
      expect(decodeCustomHole(junk)).toBeNull();
    }
    expect(sanitizeCustomHole(null)).toBeNull();
    expect(sanitizeCustomHole({ tee: [0, 0] })).toBeNull();
  });

  it('clamps what a hand-edited link could ask for', () => {
    const hole = sanitizeCustomHole({
      tee: [99999, -99999],
      cup: [0, 0],
      par: 40,
      name: 'x'.repeat(200) + '\u0007',
      items: [
        ...Array.from({ length: 100 }, () => ({ t: 'planet', x: 0, y: 0, r: 5000 })),
        { t: 'nonsense', x: 1, y: 2, r: 3 },
      ],
      stars: [[0, 0], [1, 1], [2, 2], [3, 3]],
    })!;
    expect(hole.tee).toEqual([720, -440]);
    expect(hole.par).toBe(CUSTOM_LIMITS.maxPar);
    expect(hole.name.length).toBeLessThanOrEqual(CUSTOM_LIMITS.nameLength);
    expect(hole.items.length).toBe(CUSTOM_LIMITS.items);
    expect(hole.items.every((i) => 'r' in i && i.r === CUSTOM_LIMITS.maxRadius)).toBe(true);
    expect(hole.stars.length).toBe(3);
  });

  it('starts the editor on a hole that is valid and can be sunk', () => {
    const level = customToLevel(emptyCustomHole());
    expect(validateLevel(level)).toEqual([]);
    const solution = solveLevel(level, { angleSamples: 72, powerSamples: 5, maxStrokes: 3 });
    expect(solution.solved).toBe(true);
  }, 60000);

  it('carries portals and boost rings into the hole', () => {
    const hole = emptyCustomHole();
    hole.items.push({ t: 'portal', x1: -200, y1: -200, x2: 200, y2: -200 });
    hole.items.push({ t: 'boost', x: 0, y: -100, a: Math.PI / 2 });
    const back = decodeCustomHole(encodeCustomHole(hole))!;
    const level = customToLevel(back);
    expect(level.portals).toHaveLength(1);
    expect(level.portals![0]!.to).toEqual({ x: 200, y: -200 });
    expect(level.boosters).toHaveLength(1);
    expect(level.boosters![0]!.direction.y).toBeCloseTo(1, 9);
    // Neither is a solid body.
    expect(level.bodies).toHaveLength(2);
    expect(validateLevel(level)).toEqual([]);
  });
});
