import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import { blackHole, bumper, onSurface, planet, rock, sun, wall, type LevelDef } from '../level';

const BOUNDS = { minX: -700, minY: -430, maxX: 700, maxY: 430 };

/**
 * Chapter 3 — Event Horizon.
 * Gravity stops being friendly: black holes that swallow the ball, bodies that
 * push instead of pull, and pockets of space where gravity does not apply.
 */
export const CHAPTER_3: LevelDef[] = [
  {
    id: 'c3-1',
    name: 'First Singularity',
    chapter: 2,
    par: 3,
    tee: vec(-560, 300),
    // Sunk into the right-hand floor, so a ball that arrives rolling can slow
    // down enough to drop. A hole hanging in open space can only be lipped.
    hole: { position: vec(540, 336) },
    bounds: BOUNDS,
    hint: 'A black hole pulls hardest close in. Cross it fast or go around.',
    ambientDrag: 0.1,
    bodies: [
      blackHole('bh', vec(0, -40), 34, 1500, { range: 560 }),
      wall('floor-l', vec(-640, 350), vec(-180, 350), 12),
      wall('floor-r', vec(180, 350), vec(640, 350), 12),
    ],
    stars: [vec(-300, 120), vec(0, 250), vec(300, 120)],
  },
  {
    id: 'c3-2',
    name: 'Repulsor',
    chapter: 2,
    par: 3,
    tee: vec(-600, 0),
    hole: { position: onSurface(vec(520, 0), 60, 0, 3) },
    bounds: BOUNDS,
    hint: 'That blue core pushes instead of pulling. Ride the shove.',
    ambientDrag: 0.14,
    bodies: [
      // Negative surface gravity: the field points outward.
      planet('rep', vec(-40, 0), 70, -820, { range: 520, style: 'ice', material: 'ice' }),
      planet('anchor', vec(520, 0), 60, 620, { range: 400 }),
      wall('rail-top', vec(-300, -260), vec(300, -260), 12),
      wall('rail-bottom', vec(-300, 260), vec(300, 260), 12),
    ],
    stars: [vec(-300, -140), vec(-300, 140), vec(240, 0)],
  },
  {
    id: 'c3-3',
    name: 'Null Space',
    chapter: 2,
    par: 3,
    tee: vec(-620, -280),
    hole: { position: vec(620, 300) },
    bounds: BOUNDS,
    hint: 'Inside the green field, gravity does not apply. Straight lines only.',
    ambientDrag: 0.08,
    bodies: [
      planet('big', vec(0, 60), 120, 1000, { range: 900 }),
      wall('shelf', vec(360, 380), vec(680, 380), 12),
    ],
    zones: [
      {
        id: 'null-1',
        kind: 'gravityScale',
        area: rectPolygon(vec(-60, -260), 320, 90),
        scale: 0,
      },
      {
        id: 'amp',
        kind: 'gravityScale',
        area: { kind: 'circle', center: vec(380, 120), radius: 130 },
        scale: 2.2,
      },
    ],
    stars: [vec(-260, -260), vec(140, -260), vec(420, 260)],
  },
  {
    id: 'c3-4',
    name: 'Tidal Lock',
    chapter: 2,
    par: 4,
    tee: vec(-560, 276),
    hole: { position: vec(550, 286) },
    bounds: BOUNDS,
    hint: 'Two wells guard the crossing. Go under them, or well over.',
    ambientDrag: 0.1,
    bodies: [
      planet('core', vec(0, 760), 300, 700, { range: 1000 }),
      blackHole('bh-l', vec(-190, 40), 38, 1700, { range: 420 }),
      blackHole('bh-r', vec(190, 40), 38, 1700, { range: 420 }),
      wall('ledge-l', vec(-680, 300), vec(-400, 300), 12),
      wall('ledge-r', vec(400, 300), vec(680, 300), 12),
      rock('pillar', vec(0, 330), 46),
    ],
    stars: [vec(-330, 160), vec(0, -220), vec(330, 160)],
  },
  {
    id: 'c3-5',
    name: 'Chain Reaction',
    chapter: 2,
    par: 4,
    tee: vec(-640, 340),
    hole: { position: vec(640, -340) },
    bounds: BOUNDS,
    hint: 'Four wells in a row. Let each one hand you to the next.',
    ambientDrag: 0.06,
    bodies: [
      planet('c1', vec(-380, 120), 62, 760, { range: 340, style: 'moon' }),
      planet('c2', vec(-120, -120), 62, 760, { range: 340, style: 'moon' }),
      planet('c3', vec(160, 120), 62, 760, { range: 340, style: 'moon' }),
      planet('c4', vec(420, -120), 62, 760, { range: 340, style: 'moon' }),
      sun('hazard', vec(20, 330), 44, 500, { range: 300 }),
    ],
    stars: [vec(-250, 0), vec(20, 0), vec(290, 0)],
  },
  {
    id: 'c3-6',
    name: 'The Maw',
    chapter: 2,
    par: 4,
    tee: vec(-640, 0),
    hole: { position: vec(640, 0) },
    bounds: BOUNDS,
    hint: 'There is a safe line. It is not the short one.',
    ambientDrag: 0.09,
    bodies: [
      blackHole('maw', vec(0, 0), 46, 2100, { range: 700 }),
      bumper('b-top', vec(0, -330), 46),
      bumper('b-bottom', vec(0, 330), 46),
      wall('fence-l', vec(-330, -420), vec(-330, -200), 12),
      wall('fence-r', vec(330, 420), vec(330, 200), 12),
    ],
    stars: [vec(-180, -240), vec(180, 240), vec(-460, 240)],
  },
];
