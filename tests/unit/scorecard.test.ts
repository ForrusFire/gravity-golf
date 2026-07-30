import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import type { LevelDef } from '../../src/game/level';
import { MemoryStorage, ProgressStore } from '../../src/game/progress';
import type { HoleResult } from '../../src/game/session';
import { buildScorecard, isLevelUnlocked, scorecardTotals } from '../../src/ui/screens';

const level = (id: string, par: number, chapter = 0): LevelDef => ({
  id,
  name: id.toUpperCase(),
  chapter,
  par,
  tee: V.vec(-100, 0),
  hole: { position: V.vec(100, 0) },
  bounds: { minX: -200, minY: -200, maxX: 200, maxY: 200 },
});

const result = (levelId: string, strokes: number, par: number, stars: number): HoleResult => ({
  levelId,
  strokes,
  par,
  stars,
  medal: strokes < par ? 'gold' : strokes === par ? 'silver' : 'bronze',
  time: 10,
  longestShot: 100,
  feats: [],
  shots: [],
  hinted: false,
});

const LEVELS = [level('a', 3), level('b', 4), level('c', 2)];

describe('buildScorecard', () => {
  it('marks unplayed holes with no score', () => {
    const rows = buildScorecard(LEVELS, new ProgressStore(new MemoryStorage()));
    expect(rows).toHaveLength(3);
    for (const row of rows) {
      expect(row.played).toBe(false);
      expect(row.strokes).toBeNull();
      expect(row.stars).toBe(0);
      expect(row.medal).toBe('none');
    }
  });

  it('reports best scores for played holes', () => {
    const progress = new ProgressStore(new MemoryStorage());
    progress.submit(result('a', 2, 3, 3));
    const rows = buildScorecard(LEVELS, progress);
    expect(rows[0]).toMatchObject({ played: true, strokes: 2, stars: 3, medal: 'gold' });
    expect(rows[1]!.played).toBe(false);
  });

  it('does not count an attempted-but-unfinished hole as played', () => {
    const progress = new ProgressStore(new MemoryStorage());
    progress.countAttempt('a');
    const rows = buildScorecard(LEVELS, progress);
    expect(rows[0]!.played).toBe(false);
    expect(rows[0]!.strokes).toBeNull();
  });

  it('keeps hole ordering and indexes', () => {
    const rows = buildScorecard(LEVELS, new ProgressStore(new MemoryStorage()));
    expect(rows.map((r) => r.level.id)).toEqual(['a', 'b', 'c']);
    expect(rows.map((r) => r.index)).toEqual([0, 1, 2]);
  });
});

describe('scorecardTotals', () => {
  it('is empty for a fresh save', () => {
    const totals = scorecardTotals(buildScorecard(LEVELS, new ProgressStore(new MemoryStorage())));
    expect(totals).toEqual({
      holesPlayed: 0,
      holesTotal: 3,
      strokes: 0,
      par: 0,
      stars: 0,
      starsTotal: 9,
    });
  });

  it('totals only the holes actually completed', () => {
    const progress = new ProgressStore(new MemoryStorage());
    progress.submit(result('a', 2, 3, 3));
    progress.submit(result('c', 4, 2, 1));

    const totals = scorecardTotals(buildScorecard(LEVELS, progress));
    expect(totals.holesPlayed).toBe(2);
    expect(totals.strokes).toBe(6);
    // Par covers only the completed holes, so "to par" is a fair comparison.
    expect(totals.par).toBe(5);
    expect(totals.stars).toBe(4);
  });

  it('counts stars from holes that were never finished', () => {
    const progress = new ProgressStore(new MemoryStorage());
    progress.submit(result('a', 3, 3, 2));
    const totals = scorecardTotals(buildScorecard(LEVELS, progress));
    expect(totals.stars).toBe(2);
    expect(totals.starsTotal).toBe(9);
  });
});

describe('isLevelUnlocked', () => {
  it('always unlocks the first hole', () => {
    expect(isLevelUnlocked(0, LEVELS, new ProgressStore(new MemoryStorage()))).toBe(true);
  });

  it('unlocks a hole once the previous one is completed', () => {
    const progress = new ProgressStore(new MemoryStorage());
    expect(isLevelUnlocked(1, LEVELS, progress)).toBe(false);
    progress.submit(result('a', 3, 3, 0));
    expect(isLevelUnlocked(1, LEVELS, progress)).toBe(true);
    expect(isLevelUnlocked(2, LEVELS, progress)).toBe(false);
  });
});
