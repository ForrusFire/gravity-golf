import { Rng, hashSeed } from '../core/rng';
import type { LevelDef } from './level';

/** Holes in a round. Nine, like a front nine. */
export const ROUND_LENGTH = 9;

export interface RoundHole {
  level: LevelDef;
  /** Strokes taken, or null while the hole is still to play. */
  strokes: number | null;
  stars: number;
}

export interface RoundState {
  seed: number;
  holes: RoundHole[];
  /** Index of the hole being played. Equal to `holes.length` once finished. */
  index: number;
}

/**
 * Picks a round's holes from the campaign, deterministically from `seed`.
 *
 * Campaign holes rather than generated ones for two reasons: they are instant,
 * where nine generated holes would mean fifteen seconds of verification before
 * the first shot; and they are already proven completable, so a shared round is
 * a fair round. The spread across chapters is deliberate — nine holes drawn
 * uniformly would usually be nine hard ones, since most chapters are late ones.
 */
export const buildRound = (seed: number, levels: readonly LevelDef[]): RoundState => {
  const rng = new Rng(seed);
  const byChapter = new Map<number, LevelDef[]>();
  for (const level of levels) {
    const list = byChapter.get(level.chapter);
    if (list) list.push(level);
    else byChapter.set(level.chapter, [level]);
  }

  const chapters = [...byChapter.keys()].sort((a, b) => a - b);
  const picked: LevelDef[] = [];
  const used = new Set<string>();

  // Walk the chapters in order, taking one hole from each, and looping round if
  // there are fewer chapters than holes. A round therefore ramps in difficulty
  // the way the campaign does instead of opening on the finale.
  for (let i = 0; picked.length < ROUND_LENGTH && i < ROUND_LENGTH * 4; i++) {
    const chapter = chapters[i % chapters.length];
    const pool = (byChapter.get(chapter as number) ?? []).filter((l) => !used.has(l.id));
    if (pool.length === 0) continue;
    const level = pool[rng.int(0, pool.length - 1)] as LevelDef;
    used.add(level.id);
    picked.push(level);
  }

  return {
    seed,
    holes: picked.map((level) => ({ level, strokes: null, stars: 0 })),
    index: 0,
  };
};

/** Total strokes so far. Unplayed holes count as nothing, not as par. */
export const roundStrokes = (round: RoundState): number =>
  round.holes.reduce((sum, hole) => sum + (hole.strokes ?? 0), 0);

/** Par for the holes actually played, so the comparison is a fair one. */
export const roundPar = (round: RoundState): number =>
  round.holes.reduce((sum, hole) => sum + (hole.strokes === null ? 0 : hole.level.par), 0);

export const roundStars = (round: RoundState): number =>
  round.holes.reduce((sum, hole) => sum + hole.stars, 0);

export const roundFinished = (round: RoundState): boolean => round.index >= round.holes.length;

/** A fresh seed for a one-off round. */
export const randomRoundSeed = (): number => Math.floor(Math.random() * 0xffffffff);

/** Everyone gets the same round on a given day. */
export const dailyRoundSeed = (date: Date): number =>
  hashSeed(
    `gravity-golf-round-${date.getUTCFullYear()}-${date.getUTCMonth() + 1}-${date.getUTCDate()}`,
  );
