import type { LevelDef } from '../level';
import { CHAPTER_1 } from './chapter1';
import { CHAPTER_2 } from './chapter2';
import { CHAPTER_3 } from './chapter3';
import { CHAPTER_4 } from './chapter4';
import { CHAPTER_5 } from './chapter5';
import { CHAPTER_6 } from './chapter6';
import { CHAPTER_7 } from './chapter7';

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
  {
    index: 1,
    name: 'Deep Field',
    subtitle: 'Hazards, strange surfaces and moving debris',
    starsRequired: 0,
    levels: CHAPTER_2,
  },
  {
    index: 2,
    name: 'Event Horizon',
    subtitle: 'Gravity stops being friendly',
    starsRequired: 6,
    levels: CHAPTER_3,
  },
  {
    index: 3,
    name: 'Machinery',
    subtitle: 'Lifts, arms, wormholes and conveyors',
    starsRequired: 14,
    levels: CHAPTER_4,
  },
  {
    index: 4,
    name: 'Singularity',
    subtitle: 'Everything at once',
    starsRequired: 26,
    levels: CHAPTER_5,
  },
  {
    index: 5,
    name: 'Machine Shop',
    subtitle: 'Switches, bridges and breakable blocks',
    starsRequired: 40,
    levels: CHAPTER_6,
  },
  {
    index: 6,
    name: 'Inversion',
    subtitle: 'Gravity itself becomes the puzzle',
    starsRequired: 56,
    levels: CHAPTER_7,
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
