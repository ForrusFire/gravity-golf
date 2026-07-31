import { vec } from '../../core/vec2';
import {
  blackHole,
  booster,
  bumper,
  crystal,
  onSurface,
  planet,
  pulsingWall,
  tollGate,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

const DOWN = vec(0, 1);

/**
 * Chapter 13 — Toll Roads.
 * Barriers that lift once you have collected enough stars. Every chapter so far
 * treated stars as a bonus you could ignore; here they are the road. The star
 * pips drawn on each gate are what it still wants, so the route is readable
 * before the first shot rather than discovered by bouncing off it.
 */
export const CHAPTER_13: LevelDef[] = [
  {
    id: 'c13-1',
    name: 'Toll Road',
    chapter: 12,
    par: 3,
    tee: vec(-640, 300),
    hole: { position: vec(620, 306) },
    bounds: BOUNDS,
    hint: 'The gate wants one star before it lifts. Go and fetch it.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor', vec(-720, 330), vec(720, 330), 12),
      // Full height. A toll you can loft over is scenery, and the very hole
      // meant to teach the mechanic would teach the opposite.
      tollGate('toll', vec(180, 318), vec(180, -440), 1),
      bumper('kick', vec(-320, 150), 42),
    ],
    // The first star is the fare; the other two are still just bonuses.
    stars: [vec(-320, 20), vec(-80, 200), vec(460, 140)],
  },
  {
    id: 'c13-2',
    name: 'Two Tolls',
    chapter: 12,
    par: 3,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(560, 0), 54, 0, 3) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Two gates, one star each. They lift in order as you collect.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-280, 180), 62, 620, { range: 360, style: 'moon' }),
      tollGate('toll-a', vec(-40, -440), vec(-40, 440), 1),
      tollGate('toll-b', vec(260, -440), vec(260, 440), 2),
    ],
    stars: [vec(-280, -180), vec(120, 280), vec(420, -260)],
  },
  {
    id: 'c13-3',
    name: 'Turnpike',
    chapter: 12,
    par: 4,
    tee: vec(-680, 340),
    hole: { position: vec(620, -300) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The ring is behind the toll, and the shelf is behind the ring.',
    ambientDrag: 0.1,
    // Gravity and a floor on both sides. The first cut had neither past the
    // toll, so a ball that paid its way through had nothing to land on and
    // simply drifted until the shot timed out — no rest, no second stroke, no
    // way to finish.
    uniformGravity: vec(0, 320),
    bodies: [
      wall('floor-l', vec(-720, 370), vec(-160, 370), 12),
      wall('floor-r', vec(40, 370), vec(720, 370), 12),
      wall('shelf', vec(480, -250), vec(720, -250), 12),
      wall('lip', vec(480, -262), vec(480, -340), 10),
      tollGate('toll', vec(-60, 440), vec(-60, -440), 2),
      bumper('bounce', vec(-380, 180), 40),
    ],
    boosters: [booster('ring', vec(220, 240), vec(0.45, -1), 720, 32)],
    stars: [vec(-380, 40), vec(-260, -180), vec(220, 240)],
  },
  {
    id: 'c13-4',
    name: 'Customs',
    chapter: 12,
    par: 4,
    tee: vec(-660, 300),
    hole: {
      position: vec(600, 306),
      approach: { direction: DOWN, tolerance: 0.7 },
    },
    bounds: BOUNDS,
    hint: 'Pay the toll, then drop in from above. Neither one is optional.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 360),
    bodies: [
      wall('floor-l', vec(-720, 330), vec(240, 330), 12),
      wall('floor-r', vec(440, 330), vec(720, 330), 12),
      tollGate('toll', vec(80, 318), vec(80, -440), 2),
      crystal('pane', vec(-180, 180), 40, 2),
      bumper('kick', vec(-420, 160), 40),
    ],
    stars: [vec(-420, 40), vec(-180, 80), vec(300, 120)],
  },
  {
    id: 'c13-5',
    name: 'Checkpoint',
    chapter: 12,
    par: 5,
    tee: vec(-660, 0),
    hole: {
      position: vec(560, 0),
      motion: { kind: 'orbit', center: vec(360, 0), radius: 200, speed: 0.65, phase: 0 },
    },
    bounds: BOUNDS,
    hint: 'A moving cup behind a toll. Two stars buy the way through.',
    ambientDrag: 0.1,
    bodies: [
      planet('hub', vec(360, 0), 70, 700, { range: 420, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-300, 120), 62, 620, { range: 340, style: 'moon' }),
      // One unbroken wall. The first cut left a one-way membrane across the
      // middle of the toll line, which let the ball through without paying and
      // made both gates decoration.
      tollGate('toll', vec(80, -440), vec(80, 440), 2),
    ],
    stars: [vec(-300, -220), vec(-120, 260), vec(-500, 180)],
  },
  {
    id: 'c13-6',
    name: 'The Last Gate',
    chapter: 12,
    par: 5,
    tee: vec(-690, 340),
    hole: {
      position: onSurface(vec(600, -80), 56, Math.PI, 3),
      requiresStars: 3,
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Every star pays for something, and the cup wants them all again.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-300, 380), 12),
      planet('anchor', vec(600, -80), 56, 700, { range: 400, material: 'sand', style: 'sand' }),
      planet('core', vec(-140, 40), 66, 680, { range: 400 }),
      tollGate('toll-a', vec(140, -440), vec(140, 440), 1),
      tollGate('toll-b', vec(380, -440), vec(380, -140), 2),
      pulsingWall('shutter', vec(380, -140), vec(380, 440), { period: 4, duty: 0.5 }),
      blackHole('maw', vec(140, -320), 26, 850, { range: 280 }),
    ],
    boosters: [booster('ring', vec(-460, 140), vec(0.85, -1), 660, 32)],
    stars: [vec(-460, -180), vec(-140, 300), vec(240, 300)],
  },
];
