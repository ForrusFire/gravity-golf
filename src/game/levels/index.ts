import type { LevelDef } from '../level';
import { CHAPTER_1 } from './chapter1';
import { CHAPTER_2 } from './chapter2';
import { CHAPTER_3 } from './chapter3';
import { CHAPTER_4 } from './chapter4';
import { CHAPTER_5 } from './chapter5';
import { CHAPTER_6 } from './chapter6';
import { CHAPTER_7 } from './chapter7';
import { CHAPTER_8 } from './chapter8';
import { CHAPTER_9 } from './chapter9';
import { CHAPTER_10 } from './chapter10';
import { CHAPTER_11 } from './chapter11';
import { CHAPTER_12 } from './chapter12';
import { CHAPTER_13 } from './chapter13';
import { CHAPTER_14 } from './chapter14';

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
  {
    index: 7,
    name: 'Clockwork',
    subtitle: 'Timed gates, one-way membranes and multi-lock vaults',
    starsRequired: 74,
    levels: CHAPTER_8,
  },
  {
    index: 8,
    name: 'Launch Control',
    subtitle: 'Boost rings that fire you where they please',
    starsRequired: 92,
    levels: CHAPTER_9,
  },
  {
    index: 9,
    name: 'Moving Targets',
    subtitle: 'The cup itself will not stand still',
    starsRequired: 110,
    levels: CHAPTER_10,
  },
  {
    index: 10,
    name: 'Rhythm',
    subtitle: 'Barriers that blink on a clock of their own',
    starsRequired: 128,
    levels: CHAPTER_11,
  },
  {
    index: 11,
    name: 'Approach',
    subtitle: 'Cups that only take the ball one way',
    starsRequired: 146,
    levels: CHAPTER_12,
  },
  {
    index: 12,
    name: 'Toll Roads',
    subtitle: 'Stars stop being a bonus and become the road',
    starsRequired: 164,
    levels: CHAPTER_13,
  },
  {
    index: 13,
    name: 'Tides',
    subtitle: 'The fields themselves come and go',
    starsRequired: 182,
    levels: CHAPTER_14,
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
