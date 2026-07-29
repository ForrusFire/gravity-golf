import { TAU } from '../../core/math';
import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import { bumper, onSurface, planet, rock, sun, wall, type LevelDef } from '../level';

const BOUNDS = { minX: -680, minY: -420, maxX: 680, maxY: 420 };

/**
 * Chapter 2 — Deep Field.
 * Introduces things that can kill you and surfaces that behave differently:
 * suns, ice, sand, moving debris, pocket gravity wells and solar wind.
 */
export const CHAPTER_2: LevelDef[] = [
  {
    id: 'c2-1',
    name: 'Hot Corner',
    chapter: 1,
    par: 3,
    tee: vec(-560, 300),
    hole: { position: vec(560, -300) },
    bounds: BOUNDS,
    hint: 'Suns destroy the ball. Give them a wide berth.',
    ambientDrag: 0.1,
    bodies: [
      sun('star', vec(0, 0), 62, 900, { range: 620 }),
      planet('p1', vec(-260, -180), 74, 620, { range: 460, style: 'moon' }),
      planet('p2', vec(260, 180), 74, 620, { range: 460, style: 'moon' }),
    ],
    stars: [vec(-350, 30), vec(360, -40), vec(150, 340)],
  },
  {
    id: 'c2-2',
    name: 'Ice Rink',
    chapter: 1,
    par: 3,
    tee: vec(-540, -180),
    hole: { position: vec(540, 250) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Ice keeps almost all your speed. Plan two bounces ahead.',
    ambientDrag: 0.08,
    bodies: [
      planet('core', vec(0, 60), 90, 520, { range: 700 }),
      wall('ice-1', vec(-380, 380), vec(380, 380), 16, { material: 'ice', style: 'ice' }),
      wall('ice-2', vec(-420, -360), vec(120, -360), 16, { material: 'ice', style: 'ice' }),
      wall('ice-3', vec(300, -60), vec(300, -330), 14, { material: 'ice', style: 'ice' }),
    ],
    stars: [vec(-150, -300), vec(160, 250), vec(430, -220)],
  },
  {
    id: 'c2-3',
    name: 'Dust Bowl',
    chapter: 1,
    par: 3,
    tee: vec(-560, -60),
    hole: { position: onSurface(vec(380, 120), 96, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'Sand eats momentum. Fly over it, or accept the slow way round.',
    ambientDrag: 0.1,
    bodies: [
      planet('dune', vec(380, 120), 96, 640, { range: 560, material: 'sand', style: 'sand' }),
      planet('rock', vec(-260, 90), 76, 560, { range: 460 }),
      wall('lip', vec(60, 40), vec(60, -200), 12),
    ],
    stars: [vec(-260, -180), vec(90, -290), vec(380, -80)],
  },
  {
    id: 'c2-4',
    name: 'Asteroid Belt',
    chapter: 1,
    par: 3,
    tee: vec(-570, 0),
    hole: { position: vec(570, 0) },
    bounds: BOUNDS,
    hint: 'The belt turns. Wait for a gap before you commit.',
    ambientDrag: 0.12,
    bodies: [
      planet('core', vec(0, 0), 68, 700, { range: 520 }),
      ...[0, 1, 2, 3, 4].map((i) =>
        rock(`belt-${i}`, vec(0, 0), 22, {
          motion: {
            kind: 'orbit',
            center: vec(0, 0),
            radius: 240,
            speed: 0.75,
            phase: (i / 5) * TAU,
          },
        }),
      ),
    ],
    stars: [vec(0, -330), vec(0, 330), vec(300, -170)],
  },
  {
    id: 'c2-5',
    name: 'Stepping Stones',
    chapter: 1,
    par: 3,
    tee: onSurface(vec(-460, 200), 52, -Math.PI / 2, 10),
    hole: { position: onSurface(vec(470, -180), 52, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'Each well only reaches so far. Hop from pull to pull.',
    ambientDrag: 0.22,
    bodies: [
      planet('s1', vec(-460, 200), 52, 520, { range: 300, style: 'moon' }),
      planet('s2', vec(-150, -60), 52, 520, { range: 300, style: 'moon' }),
      planet('s3', vec(160, 180), 52, 520, { range: 300, style: 'moon' }),
      planet('s4', vec(470, -180), 52, 520, { range: 300, style: 'moon' }),
    ],
    stars: [vec(-300, 60), vec(0, 60), vec(320, 0)],
  },
  {
    id: 'c2-6',
    name: 'Solar Wind',
    chapter: 1,
    par: 4,
    tee: vec(-580, 320),
    hole: { position: vec(580, -320) },
    bounds: BOUNDS,
    hint: 'The stream pushes constantly. Aim into it, not with it.',
    ambientDrag: 0.14,
    bodies: [
      planet('anchor', vec(-180, -120), 80, 640, { range: 480 }),
      bumper('b1', vec(240, 120), 40),
      wall('baffle', vec(420, 420), vec(420, 60), 12),
    ],
    zones: [
      {
        id: 'stream',
        kind: 'wind',
        area: rectPolygon(vec(60, 0), 320, 420),
        force: vec(0, -260),
      },
    ],
    stars: [vec(-380, 60), vec(60, -300), vec(300, -180)],
  },
];
