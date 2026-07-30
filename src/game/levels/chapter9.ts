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
  rock,
  sun,
  switchPad,
  timedPad,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

/**
 * Chapter 9 — Launch Control.
 * Boost rings fire the ball out at a fixed heading and speed, whatever it came
 * in doing. That makes them the one thing on a hole you can aim *at* instead of
 * *with*: get into the ring however you like and the exit is guaranteed. The
 * chapter builds from a single ring to a course where the rings are the route.
 */
export const CHAPTER_9: LevelDef[] = [
  {
    id: 'c9-1',
    name: 'Launch Control',
    chapter: 8,
    par: 2,
    tee: vec(-620, 340),
    hole: { position: vec(600, 336) },
    bounds: BOUNDS,
    hint: 'The ring fires you out at its own speed. Just get into it.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor-l', vec(-720, 360), vec(-160, 360), 12),
      wall('floor-r', vec(200, 360), vec(720, 360), 12),
      wall('lip', vec(-160, 360), vec(-160, 300), 12),
    ],
    boosters: [booster('ring', vec(-40, 140), vec(1, -0.42), 620, 34)],
    stars: [vec(-40, 140), vec(300, -60), vec(520, 220)],
  },
  {
    id: 'c9-2',
    name: 'Relay',
    chapter: 8,
    par: 3,
    tee: vec(-640, 0),
    hole: { position: onSurface(vec(560, 60), 54, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'One ring feeds the next. Reach the first and the rest is a chain.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 60), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-300, 180), 60, 600, { range: 340, style: 'moon' }),
      rock('post', vec(120, 220), 30),
    ],
    boosters: [
      booster('r1', vec(-140, -180), vec(1, 0.75), 560, 32),
      booster('r2', vec(220, 120), vec(0.9, -1), 620, 32),
    ],
    stars: [vec(-140, -180), vec(220, 120), vec(430, -260)],
  },
  {
    id: 'c9-3',
    name: 'Skeet',
    chapter: 8,
    par: 3,
    tee: vec(-660, 320),
    hole: { position: vec(640, -300) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'A ring fires at a fixed speed — sometimes that is more than you want.',
    ambientDrag: 0.14,
    bodies: [
      planet('core', vec(-60, 20), 70, 700, { range: 440 }),
      wall('shelf', vec(560, -270), vec(720, -270), 12),
      // The ring fires hard, so the landing has to be earned: clear the lip on
      // the way down rather than skidding along a shelf into the cup.
      wall('lip', vec(560, -282), vec(560, -370), 10),
      sun('lamp', vec(300, 140), 32, 380, { range: 230 }),
      bumper('kick', vec(320, -140), 40),
    ],
    // Fast and steep: the shelf has to be reached on the way down, not up.
    boosters: [booster('ring', vec(-340, -160), vec(1, -0.55), 780, 32)],
    stars: [vec(-340, -160), vec(60, -320), vec(560, -100)],
  },
  {
    id: 'c9-4',
    name: 'Cannonade',
    chapter: 8,
    par: 4,
    tee: vec(-680, 340),
    hole: { position: onSurface(vec(600, -140), 56, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'The pad turns the near ring on. Until then it is not there at all.',
    ambientDrag: 0.1,
    uniformGravity: vec(0, 260),
    bodies: [
      wall('floor', vec(-720, 380), vec(-120, 380), 12),
      planet('anchor', vec(600, -140), 56, 700, { range: 400, material: 'sand', style: 'sand' }),
      // Withheld until the pad is thrown: the bridge into the ring's mouth.
      gate('shutter', vec(140, 440), vec(140, 60), 'sw-open'),
      crystal('pane', vec(-40, 240), 42, 2),
      membrane('skin', vec(340, 440), vec(340, 120), vec(1, 0)),
    ],
    switches: [switchPad('sw-open', vec(-300, 180), 30)],
    boosters: [booster('ring', vec(240, 300), vec(0.85, -1), 700, 32)],
    stars: [vec(-300, 180), vec(240, 300), vec(470, -320)],
  },
  {
    id: 'c9-5',
    name: 'Countdown',
    chapter: 8,
    par: 4,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(590, 0), 54, Math.PI, 3) },
    bounds: BOUNDS,
    hint: 'The ring only exists while the clock runs. Be in it before it stops.',
    ambientDrag: 0.08,
    bodies: [
      planet('anchor', vec(590, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('core', vec(-80, 0), 74, 720, { range: 460 }),
      blackHole('maw', vec(220, -280), 28, 900, { range: 300 }),
      gate('vault', vec(380, -200), vec(380, 200), 'sw-clock'),
      bumper('kick', vec(-300, 260), 40),
    ],
    switches: [timedPad('sw-clock', vec(-320, -240), 32, 6)],
    boosters: [booster('ring', vec(60, 260), vec(1, -0.35), 660, 32)],
    stars: [vec(-320, -240), vec(60, 260), vec(430, 250)],
  },
  {
    id: 'c9-6',
    name: 'Escape Velocity',
    chapter: 8,
    par: 5,
    tee: vec(-690, 340),
    hole: { position: onSurface(vec(620, -60), 56, Math.PI, 3), requiresStars: 3 },
    bounds: BOUNDS,
    hint: 'Every star, and the rings are the road. Nothing here is optional.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-260, 380), 12),
      planet('anchor', vec(620, -60), 56, 660, { range: 380, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-160, 60), 64, 660, { range: 380, style: 'moon' }),
      // Seals the whole lower field, so the only way across is the high line the
      // second ring fires along. Without it the cup is a straight putt from the
      // tee and every ring on the hole is decoration.
      wall('curtain', vec(300, 440), vec(300, -170), 12),
      sun('flare', vec(160, 300), 32, 380, { range: 230 }),
    ],
    boosters: [
      booster('r1', vec(-400, 140), vec(0.8, -1), 640, 32),
      booster('r2', vec(60, -300), vec(1, 0.3), 700, 32),
    ],
    stars: [vec(-400, 140), vec(60, -300), vec(470, 120)],
  },
];
