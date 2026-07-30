import { vec } from '../../core/vec2';
import {
  blackHole,
  booster,
  bumper,
  crystal,
  gate,
  planet,
  rock,
  sun,
  switchPad,
  timedPad,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

/**
 * Chapter 10 — Moving Targets.
 * The cup itself travels. Every hole up to now could be solved by working out
 * where to aim; these have to be solved by working out *when*. The dashed track
 * shows the cup's whole lap, so it is a timing puzzle rather than a guess.
 */
export const CHAPTER_10: LevelDef[] = [
  {
    id: 'c10-1',
    name: 'Metronome',
    chapter: 9,
    par: 3,
    tee: vec(-620, 300),
    hole: {
      position: vec(320, 306),
      // Slow and level: this hole only has to teach that the cup does not wait.
      motion: {
        kind: 'oscillate',
        from: vec(320, 306),
        to: vec(620, 306),
        period: 6,
        phase: 0,
      },
    },
    bounds: BOUNDS,
    hint: 'The cup slides along the dashed track. Time it, do not chase it.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor', vec(-720, 330), vec(720, 330), 12),
      bumper('kick', vec(-260, 160), 42),
      rock('post', vec(60, 280), 26),
    ],
    stars: [vec(-260, 60), vec(120, 120), vec(500, 160)],
  },
  {
    id: 'c10-2',
    name: 'Carousel',
    chapter: 9,
    par: 3,
    tee: vec(-640, 0),
    hole: {
      position: vec(560, 0),
      motion: { kind: 'orbit', center: vec(340, 0), radius: 220, speed: 0.7, phase: 0 },
    },
    bounds: BOUNDS,
    hint: 'The cup circles. Meet it on the near side rather than following it.',
    ambientDrag: 0.1,
    bodies: [
      planet('hub', vec(340, 0), 74, 700, { range: 420, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-280, 140), 60, 600, { range: 340, style: 'moon' }),
    ],
    stars: [vec(-280, -180), vec(60, 220), vec(340, -300)],
  },
  {
    id: 'c10-3',
    name: 'Pendulum',
    chapter: 9,
    par: 3,
    tee: vec(-660, 300),
    hole: {
      position: vec(200, -260),
      motion: {
        kind: 'oscillate',
        from: vec(200, -260),
        to: vec(620, -260),
        period: 4.5,
        phase: 0,
      },
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'A ring gets you up there. The cup decides whether you arrive in time.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 300),
    bodies: [
      wall('floor', vec(-720, 340), vec(-140, 340), 12),
      wall('ceiling', vec(140, -290), vec(720, -290), 12),
      sun('lamp', vec(-40, 60), 32, 380, { range: 230 }),
    ],
    boosters: [booster('ring', vec(-260, 120), vec(1, -1), 700, 32)],
    stars: [vec(-260, 120), vec(340, 60), vec(560, -120)],
  },
  {
    id: 'c10-4',
    name: 'Shell Game',
    chapter: 9,
    par: 4,
    tee: vec(-680, 0),
    hole: {
      position: vec(420, -240),
      motion: { kind: 'orbit', center: vec(420, 0), radius: 240, speed: -0.85, phase: -Math.PI / 2 },
    },
    bounds: BOUNDS,
    hint: 'The fastest cup yet, and it runs the wrong way round.',
    ambientDrag: 0.1,
    bodies: [
      planet('core', vec(420, 0), 76, 720, { range: 440, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-260, 0), 64, 640, { range: 360, style: 'moon' }),
      rock('post-a', vec(-40, -260), 28),
      rock('post-b', vec(-40, 260), 28),
    ],
    stars: [vec(-260, -240), vec(-260, 240), vec(80, 0)],
  },
  {
    id: 'c10-5',
    name: 'Clockface',
    chapter: 9,
    par: 4,
    tee: vec(-660, 340),
    hole: {
      position: vec(420, 260),
      motion: { kind: 'orbit', center: vec(420, 0), radius: 260, speed: 0.6, phase: Math.PI / 2 },
    },
    bounds: BOUNDS,
    hint: 'The pad opens the way in, but only for as long as the ring is lit.',
    ambientDrag: 0.1,
    bodies: [
      planet('core', vec(420, 0), 70, 700, { range: 420, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-320, 120), 62, 620, { range: 340, style: 'moon' }),
      gate('vault', vec(100, -440), vec(100, -60), 'sw-clock'),
      gate('vault-b', vec(100, 440), vec(100, 60), 'sw-clock'),
      crystal('pane', vec(100, 0), 44, 2),
    ],
    switches: [timedPad('sw-clock', vec(-380, -220), 32, 7)],
    stars: [vec(-380, -220), vec(-100, 300), vec(420, -320)],
  },
  {
    id: 'c10-6',
    name: 'Last Orbit',
    chapter: 9,
    par: 5,
    tee: vec(-690, 340),
    hole: {
      // Kept clear of the gate at x = 300: a cup that orbits *through* the barrier
      // it is meant to be locked behind is a puzzle nobody can read.
      position: vec(520, -180),
      motion: { kind: 'orbit', center: vec(520, 0), radius: 180, speed: -0.55, phase: -Math.PI / 2 },
      requiresStars: 3,
    },
    bounds: BOUNDS,
    // Walls, not a kill box: with a sealed cup the route needs several strokes,
    // and a field where almost every shot dies leaves nowhere to play on from.
    boundsMode: 'wall',
    hint: 'Everything the game has, and a cup that never stops moving.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-320, 380), 12),
      planet('core', vec(520, 0), 76, 740, { range: 460, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-180, 40), 62, 640, { range: 360, style: 'moon' }),
      blackHole('maw', vec(120, -300), 28, 900, { range: 300 }),
      sun('flare', vec(120, 300), 30, 380, { range: 230 }),
      gate('vault', vec(300, -180), vec(300, 180), 'sw-vault'),
    ],
    switches: [switchPad('sw-vault', vec(-420, -220), 32)],
    boosters: [booster('ring', vec(-440, 140), vec(0.7, -1), 660, 32)],
    // Deliberately none of them inside the ring: on a sealed cup, a star you can
    // only collect by being launched across the map costs you the position you
    // need to collect the others.
    stars: [vec(-420, -220), vec(-140, 300), vec(120, 0)],
  },
];
