import { describe, expect, it } from 'vitest';
import { ALL_LEVELS } from '../../src/game/levels';
import {
  ROUND_LENGTH,
  buildRound,
  dailyRoundSeed,
  roundFinished,
  roundPar,
  roundStars,
  roundStrokes,
} from '../../src/game/round';

describe('buildRound', () => {
  it('draws a full round of distinct holes', () => {
    const round = buildRound(1234, ALL_LEVELS);
    expect(round.holes).toHaveLength(ROUND_LENGTH);
    expect(new Set(round.holes.map((h) => h.level.id)).size).toBe(ROUND_LENGTH);
    expect(round.index).toBe(0);
  });

  it('is deterministic, so a shared round is the same round', () => {
    const a = buildRound(99, ALL_LEVELS);
    const b = buildRound(99, ALL_LEVELS);
    expect(a.holes.map((h) => h.level.id)).toEqual(b.holes.map((h) => h.level.id));
  });

  it('gives different seeds different rounds', () => {
    const a = buildRound(1, ALL_LEVELS).holes.map((h) => h.level.id);
    const b = buildRound(2, ALL_LEVELS).holes.map((h) => h.level.id);
    expect(a).not.toEqual(b);
  });

  it('ramps through the chapters instead of opening on the finale', () => {
    // Nine holes drawn uniformly would usually be nine late ones, since most
    // chapters are late chapters. A round should start where the campaign does.
    for (const seed of [7, 42, 1001, 250000]) {
      const chapters = buildRound(seed, ALL_LEVELS).holes.map((h) => h.level.chapter);
      expect(chapters[0], `seed ${seed}`).toBe(0);
      // Non-decreasing across the round.
      expect([...chapters].sort((x, y) => x - y), `seed ${seed}`).toEqual(chapters);
    }
  });

  it('copes with fewer chapters than holes by looping round them', () => {
    const twoChapters = ALL_LEVELS.filter((l) => l.chapter <= 1);
    const round = buildRound(5, twoChapters);
    expect(round.holes).toHaveLength(ROUND_LENGTH);
    expect(new Set(round.holes.map((h) => h.level.id)).size).toBe(ROUND_LENGTH);
  });

  it('gives up gracefully when there are not enough holes at all', () => {
    const round = buildRound(5, ALL_LEVELS.slice(0, 3));
    expect(round.holes.length).toBeLessThanOrEqual(3);
    expect(round.holes.length).toBeGreaterThan(0);
  });
});

describe('round scoring', () => {
  it('counts only the holes actually played', () => {
    const round = buildRound(11, ALL_LEVELS);
    expect(roundStrokes(round)).toBe(0);
    expect(roundPar(round)).toBe(0);
    expect(roundFinished(round)).toBe(false);

    round.holes[0]!.strokes = 4;
    round.holes[0]!.stars = 2;
    round.holes[1]!.strokes = 2;
    round.index = 2;

    expect(roundStrokes(round)).toBe(6);
    // Par covers the two played holes, not all nine — otherwise a round in
    // progress always reads as miles under par.
    expect(roundPar(round)).toBe(round.holes[0]!.level.par + round.holes[1]!.level.par);
    expect(roundStars(round)).toBe(2);
    expect(roundFinished(round)).toBe(false);
  });

  it('is finished once every hole has been played', () => {
    const round = buildRound(12, ALL_LEVELS);
    round.index = round.holes.length;
    expect(roundFinished(round)).toBe(true);
  });
});

describe('dailyRoundSeed', () => {
  it('is stable for a UTC day and differs across days', () => {
    const a = dailyRoundSeed(new Date(Date.UTC(2026, 6, 30, 1)));
    const b = dailyRoundSeed(new Date(Date.UTC(2026, 6, 30, 23)));
    const c = dailyRoundSeed(new Date(Date.UTC(2026, 6, 31, 1)));
    expect(a).toBe(b);
    expect(a).not.toBe(c);
  });
});
