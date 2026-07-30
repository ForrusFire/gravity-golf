import { vec } from '../../core/vec2';
import {
  blackHole,
  bridge,
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
 * Chapter 8 — Clockwork.
 * The switches in chapter 6 stayed thrown forever, which made them a checklist.
 * These ones spring back, so the route becomes a schedule: the gate is only open
 * while the ball is still travelling. Two other ideas share the chapter — gates
 * that need every pad thrown at once, and membranes you can only cross one way.
 */
export const CHAPTER_8: LevelDef[] = [
  {
    id: 'c8-1',
    name: 'Sprung',
    chapter: 7,
    par: 3,
    tee: vec(-640, 340),
    hole: { position: vec(620, 336) },
    bounds: BOUNDS,
    hint: 'The pad springs back. The ring shows how long the gate stays open.',
    ambientDrag: 0.1,
    uniformGravity: vec(0, 380),
    bodies: [
      wall('floor', vec(-720, 360), vec(720, 360), 12),
      // Tall enough that the ball must go through, not over.
      gate('door', vec(180, 348), vec(180, -60), 'sw-door'),
      bumper('kick', vec(-300, 240), 40),
    ],
    // Generous: this hole only has to teach that the clock exists.
    switches: [timedPad('sw-door', vec(-400, 320), 34, 3.2)],
    stars: [vec(-400, 320), vec(0, 200), vec(420, 250)],
  },
  {
    id: 'c8-2',
    name: 'One Way Out',
    chapter: 7,
    par: 3,
    tee: vec(-640, 0),
    hole: { position: onSurface(vec(560, 0), 54, 0, 3) },
    bounds: BOUNDS,
    hint: 'The arrows show which way you may cross. There is no going back.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-280, 160), 62, 620, { range: 360, style: 'moon' }),
      // Crossable rightward only, so an overcooked shot cannot be walked back.
      membrane('skin-a', vec(60, -440), vec(60, -60), vec(1, 0)),
      membrane('skin-b', vec(60, 440), vec(60, 60), vec(1, 0)),
      rock('post', vec(60, 0), 26),
    ],
    stars: [vec(-280, -180), vec(300, 220), vec(300, -220)],
  },
  {
    id: 'c8-3',
    name: 'Two Keys',
    chapter: 7,
    par: 4,
    tee: vec(-660, 300),
    hole: { position: onSurface(vec(580, 140), 56, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'The gate needs both pads thrown. Neither one alone will do it.',
    ambientDrag: 0.12,
    bodies: [
      planet('anchor', vec(580, 140), 56, 620, { range: 360, material: 'sand', style: 'sand' }),
      planet('hub', vec(-80, -40), 74, 700, { range: 440 }),
      // Both switches, so this is a sequence rather than a single pass.
      gate('vault', vec(300, -440), vec(300, 60), ['sw-a', 'sw-b']),
      bumper('kick', vec(-360, -240), 40),
    ],
    switches: [switchPad('sw-a', vec(-360, 260), 30), switchPad('sw-b', vec(140, 300), 30)],
    stars: [vec(-360, 260), vec(140, 300), vec(420, -280)],
  },
  {
    id: 'c8-4',
    name: 'Tempo',
    chapter: 7,
    par: 4,
    tee: vec(-680, 340),
    hole: { position: vec(650, -300) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'A short fuse. Take the fast line or you will watch it close.',
    ambientDrag: 0.08,
    bodies: [
      planet('core', vec(-40, 40), 72, 720, { range: 460 }),
      gate('shutter', vec(420, -440), vec(420, -40), 'sw-fast'),
      wall('shelf', vec(480, -280), vec(720, -280), 12),
      sun('lamp', vec(420, 200), 32, 380, { range: 220 }),
    ],
    switches: [timedPad('sw-fast', vec(-380, -180), 32, 2.4)],
    stars: [vec(-380, -180), vec(-40, 320), vec(560, -180)],
  },
  {
    id: 'c8-5',
    name: 'Ratchet',
    chapter: 7,
    par: 4,
    tee: vec(-660, 340),
    hole: { position: vec(640, 336) },
    bounds: BOUNDS,
    hint: 'Through the membrane, throw the pad, and the bridge is yours.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 360),
    bodies: [
      wall('floor-l', vec(-720, 360), vec(-60, 360), 12),
      wall('floor-r', vec(340, 360), vec(720, 360), 12),
      membrane('skin', vec(-60, 360), vec(-60, 60), vec(1, 0)),
      bridge('span', vec(-60, 360), vec(340, 360), 'sw-span'),
      bumper('kick', vec(-380, 180), 42),
      crystal('pane', vec(160, 200), 40, 2),
    ],
    switches: [switchPad('sw-span', vec(-200, 120), 30)],
    // Just above the pane, not inside it: a star you cannot see is not a bonus.
    stars: [vec(-200, 120), vec(170, 110), vec(520, 220)],
  },
  {
    id: 'c8-6',
    name: 'Clockwork',
    chapter: 7,
    par: 6,
    tee: vec(-690, 0),
    hole: { position: onSurface(vec(610, 0), 56, Math.PI, 3) },
    bounds: BOUNDS,
    hint: 'Both clocks have to be running when you reach the vault.',
    ambientDrag: 0.08,
    bodies: [
      planet('anchor', vec(610, 0), 56, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-400, 0), 58, 620, { range: 320, style: 'moon' }),
      blackHole('maw', vec(60, -260), 32, 1100, { range: 340 }),
      sun('flare', vec(60, 260), 32, 400, { range: 250 }),
      // Held open only while both clocks are still running.
      gate('vault', vec(420, -190), vec(420, 190), ['sw-a', 'sw-b']),
      bumper('kick', vec(-220, 300), 42),
    ],
    switches: [
      // Long enough that throwing one and then running the corridor is a plan
      // rather than a coin flip, short enough that you cannot dawdle.
      timedPad('sw-a', vec(-380, -250), 32, 9),
      timedPad('sw-b', vec(-120, 0), 30, 9),
    ],
    stars: [vec(-380, -250), vec(-120, 0), vec(500, 250)],
  },
];
