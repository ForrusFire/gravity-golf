import { vec } from '../../core/vec2';
import {
  blackHole,
  booster,
  bumper,
  gate,
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

/**
 * Chapter 11 — Rhythm.
 * Barriers that blink in and out on their own clock. Chapter 8's gates waited
 * for the player; these do not wait for anybody, so the shot has to fit inside a
 * beat that was going to happen either way. A ring around each one counts down
 * to its next change, so the timing is on screen rather than in the head.
 */
export const CHAPTER_11: LevelDef[] = [
  {
    id: 'c11-1',
    name: 'On the Beat',
    chapter: 10,
    par: 3,
    tee: vec(-640, 300),
    hole: { position: vec(620, 306) },
    bounds: BOUNDS,
    hint: 'The wall comes and goes on its own. The ring shows when.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor', vec(-720, 330), vec(720, 330), 12),
      // Half on, half off, and slow enough to walk through the idea once.
      pulsingWall('beat', vec(80, 318), vec(80, 60), { period: 4, duty: 0.5 }),
      bumper('kick', vec(-300, 150), 42),
    ],
    stars: [vec(-300, 20), vec(80, -60), vec(440, 160)],
  },
  {
    id: 'c11-2',
    name: 'Off Beat',
    chapter: 10,
    par: 3,
    tee: vec(-660, 300),
    hole: { position: vec(640, 306) },
    bounds: BOUNDS,
    hint: 'Two walls, opposite phases. Only one gap is ever open.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 360),
    bodies: [
      wall('floor', vec(-720, 330), vec(720, 330), 12),
      // Phase-shifted by half a cycle, so the pair alternates.
      pulsingWall('a', vec(-100, 318), vec(-100, 40), { period: 3.4, duty: 0.5, phase: 0 }),
      pulsingWall('b', vec(240, 318), vec(240, 40), { period: 3.4, duty: 0.5, phase: 0.5 }),
      bumper('kick', vec(-380, 160), 40),
    ],
    stars: [vec(-380, 40), vec(70, 180), vec(460, 120)],
  },
  {
    id: 'c11-3',
    name: 'Strobe',
    chapter: 10,
    par: 3,
    tee: vec(-640, 0),
    hole: { position: onSurface(vec(560, 0), 54, 0, 3) },
    bounds: BOUNDS,
    hint: 'A short window on a long cycle. Wait for it, then commit.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-260, 160), 62, 620, { range: 360, style: 'moon' }),
      // Open only a third of the time, so the gap has to be aimed for.
      pulsingWall('shutter', vec(120, -440), vec(120, -60), { period: 3, duty: 0.65 }),
      pulsingWall('shutter-b', vec(120, 440), vec(120, 60), { period: 3, duty: 0.65 }),
      rock('post', vec(120, 0), 26),
    ],
    // Clear of the shutter's own line: a star inside a wall is only collectable
    // on the half of the cycle the wall is gone, which reads as a bug.
    stars: [vec(-260, -200), vec(210, 260), vec(400, -240)],
  },
  {
    id: 'c11-4',
    name: 'Lighthouse',
    chapter: 10,
    par: 4,
    tee: vec(-680, 340),
    hole: { position: vec(650, -300) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The sun blinks too. Cross while it is dark.',
    ambientDrag: 0.1,
    bodies: [
      planet('core', vec(-80, 40), 70, 700, { range: 440 }),
      wall('shelf', vec(520, -270), vec(720, -270), 12),
      wall('lip', vec(520, -282), vec(520, -360), 10),
      // A hazard on a clock: lethal for two seconds in every three.
      { ...sun('beacon', vec(300, -80), 34, 400, { range: 240 }), pulse: { period: 3, duty: 0.66, phase: 0 } },
      bumper('kick', vec(300, 200), 40),
    ],
    stars: [vec(-300, 120), vec(300, -300), vec(560, -60)],
  },
  {
    id: 'c11-5',
    name: 'Syncopation',
    chapter: 10,
    par: 4,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(590, 0), 54, Math.PI, 3) },
    bounds: BOUNDS,
    hint: 'A ring fires you at a fixed speed. The wall decides if you get through.',
    ambientDrag: 0.08,
    bodies: [
      planet('anchor', vec(590, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('core', vec(-120, 0), 70, 700, { range: 440 }),
      pulsingWall('shutter', vec(340, -200), vec(340, 200), { period: 3.6, duty: 0.55 }),
      blackHole('maw', vec(120, -300), 26, 850, { range: 280 }),
    ],
    boosters: [booster('ring', vec(60, 220), vec(1, -0.5), 640, 32)],
    stars: [vec(60, 220), vec(-120, -280), vec(460, 250)],
  },
  {
    id: 'c11-6',
    name: 'Downbeat',
    chapter: 10,
    par: 5,
    tee: vec(-690, 340),
    hole: {
      position: vec(560, -200),
      motion: {
        kind: 'oscillate',
        from: vec(560, -200),
        to: vec(560, 200),
        period: 8,
        phase: 0,
      },
      requiresStars: 3,
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'A moving cup behind a blinking wall. Everything here is on a clock.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-300, 380), 12),
      planet('core', vec(180, 40), 72, 720, { range: 440 }),
      gate('vault', vec(380, -440), vec(380, -280), 'sw-vault'),
      pulsingWall('shutter', vec(380, -260), vec(380, 260), { period: 4.2, duty: 0.5 }),
      sun('flare', vec(-160, -260), 30, 380, { range: 230 }),
    ],
    switches: [switchPad('sw-vault', vec(-440, -160), 32)],
    boosters: [booster('ring', vec(-420, 160), vec(0.9, -1), 680, 32)],
    stars: [vec(-440, -160), vec(-420, 160), vec(120, 300)],
  },
];
