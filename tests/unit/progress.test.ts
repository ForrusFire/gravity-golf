import { describe, expect, it } from 'vitest';
import {
  DEFAULT_SETTINGS,
  MemoryStorage,
  ProgressStore,
  STORAGE_KEY,
  applyResult,
  betterMedal,
  emptyProgress,
  parseProgress,
  type StorageLike,
} from '../../src/game/progress';
import type { HoleResult } from '../../src/game/session';

const result = (overrides: Partial<HoleResult> = {}): HoleResult => ({
  levelId: 'c1-1',
  strokes: 3,
  par: 3,
  stars: 2,
  medal: 'silver',
  time: 20,
  longestShot: 400,
  feats: [],
  shots: [],
  hinted: false,
  ...overrides,
});

describe('betterMedal', () => {
  it('keeps the higher-ranked medal', () => {
    expect(betterMedal('none', 'bronze')).toBe('bronze');
    expect(betterMedal('gold', 'silver')).toBe('gold');
    expect(betterMedal('ace', 'gold')).toBe('ace');
    expect(betterMedal('silver', 'silver')).toBe('silver');
  });
});

describe('parseProgress', () => {
  it('returns defaults for missing data', () => {
    expect(parseProgress(null)).toEqual(emptyProgress());
  });

  it('returns defaults for unparseable JSON', () => {
    expect(parseProgress('{{{not json')).toEqual(emptyProgress());
  });

  it('returns defaults for a JSON primitive', () => {
    expect(parseProgress('42')).toEqual(emptyProgress());
    expect(parseProgress('null')).toEqual(emptyProgress());
  });

  it('drops level records that are not objects', () => {
    const parsed = parseProgress(JSON.stringify({ levels: { a: 'nope', b: 5, c: null } }));
    expect(parsed.levels).toEqual({});
  });

  it('keeps valid level records', () => {
    const parsed = parseProgress(
      JSON.stringify({
        levels: { 'c1-1': { bestStrokes: 2, bestStars: 3, bestMedal: 'gold', bestTime: 12, completed: true, attempts: 4 } },
      }),
    );
    expect(parsed.levels['c1-1']).toEqual({
      bestStrokes: 2,
      bestShots: [],
      bestStars: 3,
      bestMedal: 'gold',
      bestTime: 12,
      completed: true,
      attempts: 4,
    });
  });

  it('clamps out-of-range values', () => {
    const parsed = parseProgress(
      JSON.stringify({
        levels: { x: { bestStrokes: 5.7, bestStars: 99, bestMedal: 'platinum', completed: true, attempts: -3 } },
      }),
    );
    expect(parsed.levels['x']!.bestStrokes).toBe(5);
    expect(parsed.levels['x']!.bestStars).toBe(3);
    expect(parsed.levels['x']!.bestMedal).toBe('none');
    expect(parsed.levels['x']!.attempts).toBe(0);
  });

  it('keeps an attempted-but-unfinished record with no stroke score', () => {
    const parsed = parseProgress(
      JSON.stringify({ levels: { x: { bestStrokes: null, attempts: 2, completed: true } } }),
    );
    expect(parsed.levels['x']!.attempts).toBe(2);
    expect(parsed.levels['x']!.bestStrokes).toBe(Infinity);
    // Cannot be "completed" with no stroke count.
    expect(parsed.levels['x']!.completed).toBe(false);
  });

  it('sanitises settings and falls back for unknown values', () => {
    const parsed = parseProgress(
      JSON.stringify({ settings: { sfxVolume: 9, musicVolume: -2, aimMode: 'wat', highContrast: 'yes' } }),
    );
    expect(parsed.settings.sfxVolume).toBe(1);
    expect(parsed.settings.musicVolume).toBe(0);
    expect(parsed.settings.aimMode).toBe('slingshot');
    expect(parsed.settings.highContrast).toBe(false);
    expect(parsed.settings.screenShake).toBe(DEFAULT_SETTINGS.screenShake);
  });

  it('rejects NaN and Infinity in totals', () => {
    const parsed = parseProgress(JSON.stringify({ totalStrokes: 'x', totalShots: -5 }));
    expect(parsed.totalStrokes).toBe(0);
    expect(parsed.totalShots).toBe(0);
  });
});

describe('applyResult', () => {
  it('records a first completion', () => {
    const data = applyResult(emptyProgress(), result());
    expect(data.levels['c1-1']).toMatchObject({
      bestStrokes: 3,
      bestStars: 2,
      bestMedal: 'silver',
      completed: true,
      attempts: 1,
    });
    expect(data.totalStrokes).toBe(3);
  });

  it('keeps the better values across runs', () => {
    let data = applyResult(emptyProgress(), result({ strokes: 4, stars: 1, medal: 'bronze', time: 30 }));
    data = applyResult(data, result({ strokes: 2, stars: 3, medal: 'gold', time: 15 }));
    expect(data.levels['c1-1']).toMatchObject({
      bestStrokes: 2,
      bestStars: 3,
      bestMedal: 'gold',
      bestTime: 15,
      attempts: 2,
    });
  });

  it('never regresses a best score after a worse run', () => {
    let data = applyResult(emptyProgress(), result({ strokes: 2, stars: 3, medal: 'gold', time: 10 }));
    data = applyResult(data, result({ strokes: 8, stars: 0, medal: 'none', time: 90 }));
    expect(data.levels['c1-1']).toMatchObject({
      bestStrokes: 2,
      bestStars: 3,
      bestMedal: 'gold',
      bestTime: 10,
    });
  });

  it('accumulates lifetime totals', () => {
    let data = applyResult(emptyProgress(), result({ strokes: 3, time: 10 }));
    data = applyResult(data, result({ levelId: 'c1-2', strokes: 5, time: 20 }));
    expect(data.totalStrokes).toBe(8);
    expect(data.totalPlayTime).toBe(30);
  });
});

describe('ProgressStore', () => {
  it('round-trips through storage', () => {
    const storage = new MemoryStorage();
    const first = new ProgressStore(storage);
    first.submit(result({ strokes: 2, stars: 3, medal: 'gold' }));

    const second = new ProgressStore(storage);
    expect(second.isCompleted('c1-1')).toBe(true);
    expect(second.recordOf('c1-1')!.bestStrokes).toBe(2);
    expect(second.totalStars()).toBe(3);
    expect(second.completedCount()).toBe(1);
  });

  it('serialises Infinity as null rather than breaking JSON', () => {
    const storage = new MemoryStorage();
    const store = new ProgressStore(storage);
    store.countAttempt('c9-9');
    const raw = storage.getItem(STORAGE_KEY)!;
    expect(raw).toContain('null');
    expect(() => JSON.parse(raw)).not.toThrow();

    const reloaded = new ProgressStore(storage);
    expect(reloaded.recordOf('c9-9')!.attempts).toBe(1);
    expect(reloaded.isCompleted('c9-9')).toBe(false);
  });

  it('counts attempts without marking the level complete', () => {
    const store = new ProgressStore(new MemoryStorage());
    store.countAttempt('c1-1');
    store.countAttempt('c1-1');
    expect(store.recordOf('c1-1')!.attempts).toBe(2);
    expect(store.isCompleted('c1-1')).toBe(false);
    expect(store.completedCount()).toBe(0);
  });

  it('updates and persists settings', () => {
    const storage = new MemoryStorage();
    const store = new ProgressStore(storage);
    store.updateSettings({ sfxVolume: 0.25, aimMode: 'direct' });
    expect(new ProgressStore(storage).settings.sfxVolume).toBe(0.25);
    expect(new ProgressStore(storage).settings.aimMode).toBe('direct');
  });

  it('clamps settings written through the store', () => {
    const store = new ProgressStore(new MemoryStorage());
    expect(store.updateSettings({ musicVolume: 42 }).musicVolume).toBe(1);
  });

  it('resets everything', () => {
    const storage = new MemoryStorage();
    const store = new ProgressStore(storage);
    store.submit(result());
    store.reset();
    expect(store.completedCount()).toBe(0);
    expect(new ProgressStore(storage).completedCount()).toBe(0);
  });

  it('keeps working when storage throws on read and write', () => {
    const hostile: StorageLike = {
      getItem() {
        throw new Error('blocked');
      },
      setItem() {
        throw new Error('quota exceeded');
      },
    };
    const store = new ProgressStore(hostile);
    expect(() => store.submit(result())).not.toThrow();
    // Progress still holds for the current session, it just is not persisted.
    expect(store.isCompleted('c1-1')).toBe(true);
  });
});

describe('campaign scoping', () => {
  // Daily and random holes are stored like any other level. They must not
  // count toward campaign totals or, worse, unlock chapters.
  const campaign = ['c1-1', 'c1-2', 'c1-3'];

  const seeded = (): ProgressStore => {
    const store = new ProgressStore(new MemoryStorage());
    store.submit(result({ levelId: 'c1-1', strokes: 2, stars: 2 }));
    store.submit(result({ levelId: 'daily-2026-07-29', strokes: 3, stars: 3 }));
    store.submit(result({ levelId: 'gen-12345', strokes: 4, stars: 3 }));
    return store;
  };

  it('counts every hole when unscoped', () => {
    expect(seeded().completedCount()).toBe(3);
    expect(seeded().totalStars()).toBe(8);
  });

  it('counts only campaign holes when scoped', () => {
    expect(seeded().completedCount(campaign)).toBe(1);
    expect(seeded().totalStars(campaign)).toBe(2);
  });

  it('ignores ids that have never been played', () => {
    expect(seeded().completedCount(['c9-9'])).toBe(0);
    expect(seeded().totalStars(['c9-9'])).toBe(0);
  });
});

describe('best round', () => {
  it('starts empty and keeps only the best', () => {
    const store = new ProgressStore(new MemoryStorage());
    expect(store.bestRound).toBeNull();

    expect(store.submitRound(40)).toBe(true);
    expect(store.bestRound).toBe(40);

    expect(store.submitRound(44)).toBe(false);
    expect(store.bestRound).toBe(40);

    expect(store.submitRound(37)).toBe(true);
    expect(store.bestRound).toBe(37);
  });

  it('survives a reload, and a save from before rounds existed reads as none', () => {
    const storage = new MemoryStorage();
    new ProgressStore(storage).submitRound(33);
    expect(new ProgressStore(storage).bestRound).toBe(33);

    // An older save simply has no such field; that is "no round yet", not a
    // reason to throw away everything else in it.
    expect(parseProgress(JSON.stringify({ version: 1, levels: {} })).bestRound).toBeNull();
    expect(parseProgress(JSON.stringify({ version: 1, bestRound: -5 })).bestRound).toBeNull();
  });
});
