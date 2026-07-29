import type { LevelDef } from '../level';
import { CHAPTER_1 } from './chapter1';

export interface Chapter {
  index: number;
  name: string;
  subtitle: string;
  /** Stars needed across earlier chapters before this one opens. */
  starsRequired: number;
  levels: LevelDef[];
}

export const CHAPTERS: Chapter[] = [
  {
    index: 0,
    name: 'Orbital Basics',
    subtitle: 'Learn to read a gravity well',
    starsRequired: 0,
    levels: CHAPTER_1,
  },
];

export const ALL_LEVELS: LevelDef[] = CHAPTERS.flatMap((c) => c.levels);

export const levelById = (id: string): LevelDef | undefined =>
  ALL_LEVELS.find((level) => level.id === id);

export const levelIndex = (id: string): number => ALL_LEVELS.findIndex((level) => level.id === id);

export const nextLevel = (id: string): LevelDef | undefined => {
  const index = levelIndex(id);
  return index >= 0 ? ALL_LEVELS[index + 1] : undefined;
};

export const chapterOf = (level: LevelDef): Chapter | undefined =>
  CHAPTERS.find((c) => c.index === level.chapter);
