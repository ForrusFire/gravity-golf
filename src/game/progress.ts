import { FEATS, type FeatId, type HoleResult, type Medal, type ShotRecord } from './session';

export interface LevelRecord {
  /** Fewest strokes ever taken on this hole. */
  bestStrokes: number;
  /** The shots of the best run, replayed as a ghost on later attempts. */
  bestShots: ShotRecord[];
  /** Most stars collected in a single completed run. */
  bestStars: number;
  bestMedal: Medal;
  /** Fastest completion in seconds. */
  bestTime: number;
  completed: boolean;
  attempts: number;
}

export interface Settings {
  sfxVolume: number;
  musicVolume: number;
  /** How far ahead the aiming preview simulates, in seconds. 0 hides it. */
  aimAssist: number;
  screenShake: boolean;
  /** Higher-contrast palette that does not rely on hue alone. */
  highContrast: boolean;
  reducedMotion: boolean;
  showGravityField: boolean;
  /** 'slingshot' pulls back from the ball; 'direct' aims at the cursor. */
  aimMode: 'slingshot' | 'direct';
  leftHanded: boolean;
  /** Race a translucent replay of your best run on this hole. */
  showGhost: boolean;
  /** Chosen ball appearance. Falls back to the default when not yet unlocked. */
  ballSkin: string;
}

export interface ProgressData {
  version: number;
  levels: Record<string, LevelRecord>;
  settings: Settings;
  /** Style awards earned at least once, across every hole. */
  feats: FeatId[];
  /** Cumulative strokes across all completed holes. */
  totalStrokes: number;
  totalShots: number;
  totalDeaths: number;
  totalPlayTime: number;
  /** Fewest strokes in a completed round, or null before the first one. */
  bestRound: number | null;
}

export const SCHEMA_VERSION = 1;
export const STORAGE_KEY = 'gravity-golf/progress';

export const DEFAULT_SETTINGS: Settings = {
  sfxVolume: 0.7,
  musicVolume: 0.35,
  aimAssist: 1.4,
  screenShake: true,
  highContrast: false,
  reducedMotion: false,
  showGravityField: true,
  aimMode: 'slingshot',
  leftHanded: false,
  showGhost: true,
  ballSkin: 'classic',
};

export const emptyProgress = (): ProgressData => ({
  version: SCHEMA_VERSION,
  levels: {},
  settings: { ...DEFAULT_SETTINGS },
  feats: [],
  totalStrokes: 0,
  totalShots: 0,
  totalDeaths: 0,
  totalPlayTime: 0,
  bestRound: null,
});

const MEDAL_RANK: Record<Medal, number> = { none: 0, bronze: 1, silver: 2, gold: 3, ace: 4 };

export const betterMedal = (a: Medal, b: Medal): Medal => (MEDAL_RANK[a] >= MEDAL_RANK[b] ? a : b);

const isFiniteNumber = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);

/** Keeps only well-formed shots; a corrupt ghost must not break playback. */
const sanitizeShots = (raw: unknown): ShotRecord[] => {
  if (!Array.isArray(raw)) return [];
  const shots: ShotRecord[] = [];
  for (const entry of raw.slice(0, 24)) {
    if (typeof entry !== 'object' || entry === null) continue;
    const shot = entry as Partial<ShotRecord>;
    if (!isFiniteNumber(shot.angle) || !isFiniteNumber(shot.power)) continue;
    shots.push({ angle: shot.angle, power: Math.max(0, Math.min(1, shot.power)) });
  }
  return shots;
};

const sanitizeRecord = (raw: unknown): LevelRecord | null => {
  if (typeof raw !== 'object' || raw === null) return null;
  const r = raw as Partial<LevelRecord>;
  const attempts = isFiniteNumber(r.attempts) ? Math.max(0, Math.floor(r.attempts)) : 0;
  // An unfinished level has no stroke record — it is serialised as null and
  // only worth keeping for its attempt count.
  const hasStrokes = isFiniteNumber(r.bestStrokes) && r.bestStrokes >= 0;
  if (!hasStrokes && attempts === 0) return null;
  return {
    bestStrokes: hasStrokes ? Math.floor(r.bestStrokes as number) : Infinity,
    bestShots: sanitizeShots(r.bestShots),
    bestStars: isFiniteNumber(r.bestStars) ? Math.max(0, Math.min(3, Math.floor(r.bestStars))) : 0,
    bestMedal: (r.bestMedal && r.bestMedal in MEDAL_RANK ? r.bestMedal : 'none') as Medal,
    bestTime: isFiniteNumber(r.bestTime) && r.bestTime > 0 ? r.bestTime : Infinity,
    completed: r.completed === true && hasStrokes,
    attempts,
  };
};

const sanitizeSettings = (raw: unknown): Settings => {
  const s = (typeof raw === 'object' && raw !== null ? raw : {}) as Partial<Settings>;
  const num = (v: unknown, fallback: number, min: number, max: number): number =>
    isFiniteNumber(v) ? Math.max(min, Math.min(max, v)) : fallback;
  return {
    sfxVolume: num(s.sfxVolume, DEFAULT_SETTINGS.sfxVolume, 0, 1),
    musicVolume: num(s.musicVolume, DEFAULT_SETTINGS.musicVolume, 0, 1),
    aimAssist: num(s.aimAssist, DEFAULT_SETTINGS.aimAssist, 0, 4),
    screenShake: s.screenShake !== false,
    highContrast: s.highContrast === true,
    reducedMotion: s.reducedMotion === true,
    showGravityField: s.showGravityField !== false,
    aimMode: s.aimMode === 'direct' ? 'direct' : 'slingshot',
    leftHanded: s.leftHanded === true,
    showGhost: s.showGhost !== false,
    ballSkin: typeof s.ballSkin === 'string' ? s.ballSkin : DEFAULT_SETTINGS.ballSkin,
  };
};

/**
 * Rebuilds a valid progress object from whatever is in storage. Anything
 * unparseable, corrupt or from a future version degrades to defaults rather
 * than throwing — losing progress is bad, but a game that will not boot is
 * worse.
 */
export const parseProgress = (raw: string | null): ProgressData => {
  if (!raw) return emptyProgress();
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return emptyProgress();
  }
  if (typeof parsed !== 'object' || parsed === null) return emptyProgress();

  const data = parsed as Partial<ProgressData>;
  const result = emptyProgress();
  result.settings = sanitizeSettings(data.settings);

  if (typeof data.levels === 'object' && data.levels !== null) {
    for (const [id, value] of Object.entries(data.levels)) {
      const record = sanitizeRecord(value);
      if (record) result.levels[id] = record;
    }
  }

  if (Array.isArray(data.feats)) {
    const known = new Set(Object.keys(FEATS));
    result.feats = data.feats.filter(
      (id): id is FeatId => typeof id === 'string' && known.has(id),
    );
  }

  result.totalStrokes = isFiniteNumber(data.totalStrokes) ? Math.max(0, data.totalStrokes) : 0;
  result.totalShots = isFiniteNumber(data.totalShots) ? Math.max(0, data.totalShots) : 0;
  result.totalDeaths = isFiniteNumber(data.totalDeaths) ? Math.max(0, data.totalDeaths) : 0;
  result.totalPlayTime = isFiniteNumber(data.totalPlayTime) ? Math.max(0, data.totalPlayTime) : 0;
  // Absent in saves from before rounds existed, which is exactly "no round yet".
  result.bestRound =
    isFiniteNumber(data.bestRound) && data.bestRound > 0 ? Math.floor(data.bestRound) : null;
  return result;
};

/** Folds a completed hole into the saved record, keeping the player's bests. */
export const applyResult = (data: ProgressData, result: HoleResult): ProgressData => {
  const previous = data.levels[result.levelId];
  // The ghost should show the run worth beating, so the shot list travels with
  // the stroke record rather than being overwritten by the latest attempt.
  const improved = !previous || result.strokes < previous.bestStrokes;
  const record: LevelRecord = previous
    ? {
        bestStrokes: Math.min(previous.bestStrokes, result.strokes),
        bestShots: improved ? result.shots : previous.bestShots,
        bestStars: Math.max(previous.bestStars, result.stars),
        bestMedal: betterMedal(previous.bestMedal, result.medal),
        bestTime: Math.min(previous.bestTime, result.time),
        completed: true,
        attempts: previous.attempts + 1,
      }
    : {
        bestStrokes: result.strokes,
        bestShots: result.shots,
        bestStars: result.stars,
        bestMedal: result.medal,
        bestTime: result.time,
        completed: true,
        attempts: 1,
      };

  const feats = [...new Set([...data.feats, ...result.feats])];

  return {
    ...data,
    levels: { ...data.levels, [result.levelId]: record },
    feats,
    totalStrokes: data.totalStrokes + result.strokes,
    totalPlayTime: data.totalPlayTime + result.time,
  };
};

/** Simple storage port so tests (and private-mode browsers) do not blow up. */
export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

export class MemoryStorage implements StorageLike {
  private map = new Map<string, string>();
  getItem(key: string): string | null {
    return this.map.get(key) ?? null;
  }
  setItem(key: string, value: string): void {
    this.map.set(key, value);
  }
}

/** Returns localStorage when it is usable, and an in-memory stand-in otherwise. */
export const defaultStorage = (): StorageLike => {
  try {
    if (typeof localStorage === 'undefined') return new MemoryStorage();
    const probe = '__gg_probe__';
    localStorage.setItem(probe, '1');
    localStorage.removeItem(probe);
    return localStorage;
  } catch {
    // Safari private mode and some embedded webviews throw on write.
    return new MemoryStorage();
  }
};

export class ProgressStore {
  private data: ProgressData;

  constructor(private readonly storage: StorageLike = defaultStorage()) {
    this.data = parseProgress(this.safeRead());
  }

  private safeRead(): string | null {
    try {
      return this.storage.getItem(STORAGE_KEY);
    } catch {
      return null;
    }
  }

  get(): ProgressData {
    return this.data;
  }

  get settings(): Settings {
    return this.data.settings;
  }

  /** Style awards earned at least once. */
  earnedFeats(): FeatId[] {
    return [...this.data.feats];
  }

  /** Seconds of play banked across every completed hole. */
  get totalPlayTime(): number {
    return this.data.totalPlayTime;
  }

  /** Fewest strokes in a completed round, or null before the first one. */
  get bestRound(): number | null {
    return this.data.bestRound;
  }

  /** Records a finished round, keeping only the best. Returns true if it beat it. */
  submitRound(strokes: number): boolean {
    const beat = this.data.bestRound === null || strokes < this.data.bestRound;
    if (beat) {
      this.data.bestRound = strokes;
      this.save();
    }
    return beat;
  }

  /** The best run's shots for a hole, for the ghost to replay. */
  bestShots(levelId: string): ShotRecord[] {
    return this.data.levels[levelId]?.bestShots ?? [];
  }

  recordOf(levelId: string): LevelRecord | undefined {
    return this.data.levels[levelId];
  }

  isCompleted(levelId: string): boolean {
    return this.data.levels[levelId]?.completed === true;
  }

  starsFor(levelId: string): number {
    return this.data.levels[levelId]?.bestStars ?? 0;
  }

  /**
   * Stars collected. Pass the campaign's level ids to exclude generated holes:
   * a daily challenge must not count toward unlocking a chapter, and it must
   * not inflate the "x of y holes" readout either.
   */
  totalStars(ids?: readonly string[]): number {
    let total = 0;
    for (const [id, record] of Object.entries(this.data.levels)) {
      if (ids && !ids.includes(id)) continue;
      total += record.bestStars;
    }
    return total;
  }

  /** Completed holes, optionally scoped to a set of level ids. */
  completedCount(ids?: readonly string[]): number {
    let total = 0;
    for (const [id, record] of Object.entries(this.data.levels)) {
      if (ids && !ids.includes(id)) continue;
      if (record.completed) total++;
    }
    return total;
  }

  submit(result: HoleResult): LevelRecord {
    this.data = applyResult(this.data, result);
    this.save();
    return this.data.levels[result.levelId]!;
  }

  countAttempt(levelId: string): void {
    const existing = this.data.levels[levelId];
    if (existing) {
      existing.attempts++;
    } else {
      this.data.levels[levelId] = {
        bestStrokes: Infinity,
        bestShots: [],
        bestStars: 0,
        bestMedal: 'none',
        bestTime: Infinity,
        completed: false,
        attempts: 1,
      };
    }
    this.save();
  }

  countShot(): void {
    this.data.totalShots++;
  }

  countDeath(): void {
    this.data.totalDeaths++;
  }

  updateSettings(patch: Partial<Settings>): Settings {
    this.data.settings = sanitizeSettings({ ...this.data.settings, ...patch });
    this.save();
    return this.data.settings;
  }

  reset(): void {
    this.data = emptyProgress();
    this.save();
  }

  save(): void {
    try {
      // Infinity is not JSON-representable; store unset bests as null.
      this.storage.setItem(
        STORAGE_KEY,
        JSON.stringify(this.data, (_key, value) =>
          typeof value === 'number' && !Number.isFinite(value) ? null : value,
        ),
      );
    } catch {
      // Storage full or blocked — progress stays in memory for this session.
    }
  }
}
