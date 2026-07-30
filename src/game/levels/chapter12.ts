import { vec } from '../../core/vec2';
import {
  blackHole,
  booster,
  bumper,
  crystal,
  gate,
  membrane,
  onSurface,
  planet,
  pulsingWall,
  rock,
  sun,
  switchPad,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

/** Straight down, straight left, and so on — the headings the cups demand. */
const DOWN = vec(0, 1);
const UP = vec(0, -1);
const LEFT = vec(-1, 0);

/**
 * Chapter 12 — Approach.
 * Cups with a mouth. Arriving slowly is no longer enough; the ball has to be
 * travelling the right way when it gets there, which turns the last few metres
 * of every shot into the part that matters. The wedge drawn on each cup is the
 * side you have to come from.
 */
export const CHAPTER_12: LevelDef[] = [
  {
    id: 'c12-1',
    name: 'Letterbox',
    chapter: 11,
    par: 3,
    tee: vec(-620, 300),
    hole: {
      position: vec(560, 300),
      // Straight down: rolling along the floor into it will not do.
      approach: { direction: DOWN, tolerance: 0.7 },
    },
    bounds: BOUNDS,
    hint: 'This cup only takes the ball from above. Drop in, do not roll in.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor-l', vec(-720, 330), vec(200, 330), 12),
      wall('floor-r', vec(400, 330), vec(720, 330), 12),
      bumper('kick', vec(-240, 150), 42),
    ],
    stars: [vec(-240, 30), vec(160, 60), vec(480, 120)],
  },
  {
    id: 'c12-2',
    name: 'Back Door',
    chapter: 11,
    par: 3,
    tee: vec(-640, 0),
    hole: {
      position: onSurface(vec(480, 0), 56, 0, 3),
      // Leftwards: the cup faces away, so the ball has to get past and come back.
      approach: { direction: LEFT, tolerance: 0.75 },
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The mouth faces the far wall. You have to come back at it.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(480, 0), 56, 620, { range: 360, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-260, 140), 62, 620, { range: 360, style: 'moon' }),
      rock('post', vec(120, -220), 28),
    ],
    stars: [vec(-260, -180), vec(140, 200), vec(620, -220)],
  },
  {
    id: 'c12-3',
    name: 'Uppercut',
    chapter: 11,
    par: 3,
    tee: vec(-660, 320),
    hole: {
      // Placed in the band where a ball fired from the ring is still rising but
      // has slowed to capture speed. Without gravity there is nothing to bleed
      // the ring's 680 off, and "arrive slowly, still going up" has no window at
      // all — the first cut of this hole only solved under one sampling of the
      // search, which is another way of saying it was luck.
      position: vec(350, -290),
      approach: { direction: UP, tolerance: 0.9 },
      radius: 20,
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The ring throws you up. Meet the cup near the top of the arc.',
    ambientDrag: 0.06,
    uniformGravity: vec(0, 340),
    bodies: [
      wall('floor', vec(-720, 350), vec(60, 350), 12),
      rock('post', vec(-200, 300), 26),
      sun('lamp', vec(600, 60), 32, 380, { range: 230 }),
    ],
    boosters: [booster('ring', vec(160, 300), vec(0.2, -1), 680, 32)],
    stars: [vec(160, 300), vec(-320, 120), vec(560, -280)],
  },
  {
    id: 'c12-4',
    name: 'Threading',
    chapter: 11,
    par: 4,
    tee: vec(-680, 300),
    hole: {
      position: vec(600, 300),
      approach: { direction: DOWN, tolerance: 0.55 },
    },
    bounds: BOUNDS,
    hint: 'A narrow mouth behind a blinking wall. Both have to line up.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 340),
    bodies: [
      wall('floor-l', vec(-720, 330), vec(260, 330), 12),
      wall('floor-r', vec(460, 330), vec(720, 330), 12),
      pulsingWall('beat', vec(180, 318), vec(180, 20), { period: 3.6, duty: 0.5 }),
      crystal('pane', vec(-140, 200), 40, 2),
      bumper('kick', vec(-400, 160), 40),
    ],
    // Clear of the pane: a star inside a block is not a bonus, it is a puzzle
    // about a puzzle.
    stars: [vec(-400, 40), vec(-140, 100), vec(360, 80)],
  },
  {
    id: 'c12-5',
    name: 'Service Entrance',
    chapter: 11,
    par: 4,
    tee: vec(-660, 340),
    hole: {
      position: onSurface(vec(560, 100), 56, -Math.PI / 2, 3),
      approach: { direction: DOWN, tolerance: 0.6 },
    },
    bounds: BOUNDS,
    hint: 'Through the membrane, over the top, and straight down onto it.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 100), 56, 660, { range: 380, material: 'sand', style: 'sand' }),
      planet('core', vec(-120, 60), 68, 700, { range: 420 }),
      membrane('skin', vec(220, 440), vec(220, 120), vec(1, 0)),
      gate('vault', vec(220, 100), vec(220, -200), 'sw-vault'),
      blackHole('maw', vec(120, -320), 26, 850, { range: 280 }),
    ],
    switches: [switchPad('sw-vault', vec(-360, -220), 30)],
    stars: [vec(-360, -220), vec(320, 300), vec(430, -200)],
  },
  {
    id: 'c12-6',
    name: 'Final Approach',
    chapter: 11,
    par: 5,
    tee: vec(-690, 340),
    hole: {
      position: vec(560, -240),
      motion: {
        kind: 'oscillate',
        from: vec(560, -240),
        to: vec(220, -240),
        period: 7,
        phase: 0,
      },
      approach: { direction: UP, tolerance: 0.6 },
      requiresStars: 3,
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Moving, sealed, and it only takes the ball from below. Good luck.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-280, 380), 12),
      wall('ceiling', vec(160, -290), vec(720, -290), 12),
      planet('core', vec(-60, 60), 70, 700, { range: 440 }),
      sun('flare', vec(340, 180), 30, 380, { range: 230 }),
      pulsingWall('shutter', vec(120, -280), vec(120, 60), { period: 4.4, duty: 0.55 }),
    ],
    boosters: [booster('ring', vec(-420, 120), vec(0.75, -1), 680, 32)],
    // None of them inside the ring: with a sealed cup, a star you can only take
    // by being launched across the map costs the position you need for the rest.
    stars: [vec(-540, 60), vec(-160, -300), vec(420, 60)],
  },
];
