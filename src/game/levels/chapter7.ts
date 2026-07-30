import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import {
  blackHole,
  bridge,
  bumper,
  crystal,
  gate,
  onSurface,
  planet,
  rock,
  sun,
  switchPad,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

/**
 * Chapter 7 — Inversion.
 * Gravity itself becomes the puzzle: zones that reverse it, amplify it and
 * cancel it, over a course built from everything that came before.
 */
export const CHAPTER_7: LevelDef[] = [
  {
    id: 'c7-1',
    name: 'Upside Down',
    chapter: 6,
    par: 3,
    tee: vec(-620, 340),
    hole: { position: vec(640, 178) },
    bounds: BOUNDS,
    hint: 'Inside the pink field, down becomes up. Ride it, then step off.',
    ambientDrag: 0.14,
    uniformGravity: vec(0, 420),
    bodies: [
      wall('floor', vec(-720, 366), vec(-200, 366), 12),
      // Low enough to clear on a flat shot, high enough that you cannot roll under.
      wall('divider', vec(0, 440), vec(0, 120), 12),
      wall('shelf', vec(440, 200), vec(720, 200), 12),
      wall('backstop', vec(700, 188), vec(700, 60), 12),
    ],
    zones: [
      {
        id: 'flip',
        kind: 'gravityScale',
        area: rectPolygon(vec(300, 0), 130, 440),
        // Negative scale reverses every field inside the zone.
        scale: -1,
      },
    ],
    stars: [vec(-360, 160), vec(300, -240), vec(510, 90)],
  },
  {
    id: 'c7-2',
    name: 'Heavy Water',
    chapter: 6,
    par: 3,
    tee: vec(-640, -280),
    hole: { position: onSurface(vec(520, 180), 56, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'Gravity triples in the amber field, and barely pulls in the green one.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(520, 180), 56, 620, { range: 360, material: 'sand', style: 'sand' }),
      planet('core', vec(-60, 60), 76, 640, { range: 620 }),
      rock('post', vec(240, -260), 30),
    ],
    zones: [
      {
        id: 'heavy',
        kind: 'gravityScale',
        area: { kind: 'circle', center: vec(180, 100), radius: 170 },
        scale: 3,
      },
      {
        id: 'calm',
        kind: 'gravityScale',
        area: { kind: 'circle', center: vec(-300, -160), radius: 140 },
        scale: 0.2,
      },
    ],
    stars: [vec(-300, -160), vec(180, 100), vec(420, -280)],
  },
  {
    id: 'c7-3',
    name: 'Reversal',
    chapter: 6,
    par: 4,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(580, 0), 54, 0, 3) },
    bounds: BOUNDS,
    hint: 'The pad flips the field. You may need it flipped both ways.',
    ambientDrag: 0.12,
    bodies: [
      planet('anchor', vec(580, 0), 54, 600, { range: 340, material: 'sand', style: 'sand' }),
      planet('core', vec(-40, 0), 80, 780, { range: 520 }),
      gate('shutter-a', vec(240, -440), vec(240, -120), 'sw-flip'),
      bridge('shutter-b', vec(240, 440), vec(240, 120), 'sw-flip'),
      bumper('kick', vec(-300, 260), 40),
    ],
    switches: [switchPad('sw-flip', vec(-300, -240), 30, false)],
    stars: [vec(-300, -240), vec(-40, 300), vec(400, 240)],
  },
  {
    id: 'c7-4',
    name: 'Tug of War',
    chapter: 6,
    par: 4,
    tee: vec(0, 380),
    hole: { position: vec(0, -370) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'One core pulls, one pushes. The quiet line runs between them.',
    ambientDrag: 0.1,
    bodies: [
      planet('pull', vec(-220, 0), 74, 820, { range: 500 }),
      planet('push', vec(220, 0), 74, -820, { range: 500, material: 'ice', style: 'ice' }),
      wall('cap-l', vec(-440, -300), vec(-120, -300), 12),
      wall('cap-r', vec(120, -300), vec(440, -300), 12),
      rock('pin', vec(0, 120), 26),
    ],
    stars: [vec(-380, 200), vec(380, 200), vec(0, -160)],
  },
  {
    id: 'c7-5',
    name: 'Kaleidoscope',
    chapter: 6,
    par: 4,
    tee: vec(-660, 340),
    hole: { position: vec(640, 340) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Wormholes, and a pink field where up and down swap. Plan the exit.',
    ambientDrag: 0.08,
    uniformGravity: vec(0, 320),
    bodies: [
      wall('floor-l', vec(-720, 366), vec(-260, 366), 12),
      wall('floor-r', vec(260, 366), vec(720, 366), 12),
      wall('pillar', vec(0, 440), vec(0, 40), 12, { material: 'ice', style: 'ice' }),
      crystal('pane', vec(-160, 120), 42, 2),
    ],
    zones: [
      {
        id: 'flip',
        kind: 'gravityScale',
        area: rectPolygon(vec(160, -160), 150, 270),
        scale: -1.1,
      },
    ],
    portals: [
      {
        id: 'w1',
        from: vec(-420, 40),
        to: vec(430, -240),
        radius: 30,
        twist: 0.5,
        boost: 1,
        bidirectional: true,
      },
    ],
    // Deliberately not on the portal mouth: a star sitting inside a wormhole is
    // collected or teleported past depending on sub-step timing.
    stars: [vec(-330, 230), vec(160, -160), vec(520, 100)],
  },
  {
    id: 'c7-6',
    name: 'Grand Finale',
    chapter: 6,
    par: 6,
    tee: vec(-690, 0),
    hole: {
      position: onSurface(vec(600, 0), 56, Math.PI, 3),
      requiresStars: 3,
    },
    bounds: BOUNDS,
    hint: 'Throw the switch, thread the corridor, then punch through the flip.',
    ambientDrag: 0.08,
    bodies: [
      planet('anchor', vec(600, 0), 56, 620, { range: 340, material: 'sand', style: 'sand' }),
      // The corridor along y = 0 runs between the two hazards. Off it, you die.
      blackHole('maw', vec(80, -250), 34, 1200, { range: 360 }),
      sun('flare', vec(80, 250), 34, 420, { range: 260 }),
      planet('feeder', vec(-380, 0), 60, 640, { range: 340, style: 'moon' }),
      bumper('kick', vec(-200, 300), 42),
      // Seals the corridor until the pad is thrown, and the last star with it.
      gate('vault', vec(460, -170), vec(460, 170), 'sw-vault'),
    ],
    switches: [switchPad('sw-vault', vec(-380, -260), 30)],
    zones: [
      {
        // Reverses the anchor's own pull, so the last stretch pushes you away
        // instead of drawing you in. You have to arrive with something left.
        id: 'flip',
        kind: 'gravityScale',
        area: rectPolygon(vec(370, 0), 70, 440),
        scale: -1,
      },
    ],
    stars: [vec(-380, -260), vec(80, 0), vec(520, 210)],
  },
];
