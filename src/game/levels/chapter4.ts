import { TAU } from '../../core/math';
import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import { bumper, onSurface, planet, rock, sun, wall, type LevelDef } from '../level';

const BOUNDS = { minX: -700, minY: -430, maxX: 700, maxY: 430 };

/**
 * Chapter 4 — Machinery.
 * The course starts moving: lifts, spinning arms, wormholes, conveyors and
 * clockwork that has to be read before it can be beaten.
 */
export const CHAPTER_4: LevelDef[] = [
  {
    id: 'c4-1',
    name: 'Elevator',
    chapter: 3,
    par: 3,
    tee: vec(-560, 250),
    hole: { position: vec(560, -290) },
    bounds: BOUNDS,
    hint: 'The lift carries the ball with it. Ride up, then shoot.',
    ambientDrag: 0.12,
    bodies: [
      planet('ground', vec(0, 1300), 950, 420, { range: 1900 }),
      {
        id: 'lift',
        shape: rectPolygon(vec(0, 260), 90, 12),
        material: 'metal',
        style: 'wall',
        motion: {
          kind: 'oscillate',
          from: vec(0, 280),
          to: vec(0, -180),
          period: 5,
          phase: 0,
        },
      },
      wall('shelf', vec(300, -240), vec(660, -240), 12),
      wall('kerb', vec(-260, 300), vec(-260, 200), 10),
    ],
    stars: [vec(0, 40), vec(-380, 120), vec(420, -320)],
  },
  {
    id: 'c4-2',
    name: 'Windmill',
    chapter: 3,
    par: 3,
    tee: vec(-600, -260),
    hole: { position: onSurface(vec(520, 200), 70, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'Time the gap. The arms hit harder than they look.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(520, 200), 70, 640, { range: 460 }),
      planet('feeder', vec(-320, 60), 60, 560, { range: 380, style: 'moon' }),
      {
        id: 'arm-a',
        shape: rectPolygon(vec(60, -60), 170, 10),
        material: 'metal',
        style: 'wall',
        motion: { kind: 'spin', speed: 1.5, phase: 0 },
      },
      {
        id: 'arm-b',
        shape: rectPolygon(vec(60, -60), 10, 170),
        material: 'metal',
        style: 'wall',
        motion: { kind: 'spin', speed: 1.5, phase: 0 },
      },
    ],
    stars: [vec(-320, -230), vec(60, 260), vec(400, -200)],
  },
  {
    id: 'c4-3',
    name: 'Wormhole',
    chapter: 3,
    par: 3,
    tee: vec(-600, 300),
    hole: { position: vec(600, -300) },
    bounds: BOUNDS,
    hint: 'Portals keep your speed. Line up the exit, not just the entrance.',
    ambientDrag: 0.12,
    bodies: [
      planet('p1', vec(-300, 60), 66, 600, { range: 400, style: 'moon' }),
      planet('p2', vec(320, -60), 66, 600, { range: 400, style: 'moon' }),
      wall('block', vec(40, 430), vec(40, -170), 14),
      wall('shelf', vec(360, -240), vec(660, -240), 12),
    ],
    portals: [
      {
        id: 'w1',
        from: vec(-140, 300),
        to: vec(220, -300),
        radius: 30,
        twist: 0,
        boost: 1,
        bidirectional: true,
      },
    ],
    stars: [vec(-420, 140), vec(-140, 180), vec(500, -120)],
  },
  {
    id: 'c4-4',
    name: 'Conveyor',
    chapter: 3,
    par: 3,
    tee: vec(-620, 330),
    hole: { position: vec(600, 330) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The boost lanes only push one way. Use them, do not fight them.',
    ambientDrag: 0.18,
    bodies: [
      planet('ground', vec(0, 1300), 940, 400, { range: 1900 }),
      wall('step-1', vec(-260, 200), vec(-40, 200), 12),
      wall('step-2', vec(120, 40), vec(360, 40), 12),
      bumper('b1', vec(440, 200), 38),
    ],
    zones: [
      {
        id: 'lane-up',
        kind: 'boost',
        area: rectPolygon(vec(-150, 120), 120, 190),
        force: vec(90, -560),
      },
      {
        id: 'lane-right',
        kind: 'boost',
        area: rectPolygon(vec(240, -60), 150, 90),
        force: vec(620, -60),
      },
    ],
    stars: [vec(-150, 20), vec(240, -170), vec(520, 120)],
  },
  {
    id: 'c4-5',
    name: 'Clockwork',
    chapter: 3,
    par: 4,
    tee: vec(-620, 0),
    hole: { position: onSurface(vec(480, 0), 58, 0, 3) },
    bounds: BOUNDS,
    hint: 'One orbiting moon, one turning gate. They are not in step.',
    ambientDrag: 0.09,
    bodies: [
      planet('core', vec(-40, 0), 84, 760, { range: 520 }),
      planet('moon', vec(0, 0), 34, 420, {
        range: 200,
        style: 'moon',
        motion: { kind: 'orbit', center: vec(-40, 0), radius: 210, speed: 1.1, phase: 0 },
      }),
      planet('anchor', vec(480, 0), 58, 560, { range: 340 }),
      {
        id: 'gate',
        shape: rectPolygon(vec(260, 0), 12, 150),
        material: 'metal',
        style: 'wall',
        motion: { kind: 'spin', speed: -1.3, phase: TAU * 0.1 },
      },
    ],
    stars: [vec(-40, -280), vec(-40, 280), vec(360, -200)],
  },
  {
    id: 'c4-6',
    name: 'Pinball',
    chapter: 3,
    par: 4,
    tee: vec(-640, 340),
    hole: { position: vec(0, 386) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Everything here gives energy back. Aim for a soft landing.',
    ambientDrag: 0.16,
    bodies: [
      planet('ground', vec(0, 1300), 900, 380, { range: 1900 }),
      bumper('b1', vec(-260, 120), 44),
      bumper('b2', vec(80, -60), 44),
      bumper('b3', vec(360, 140), 44),
      bumper('b4', vec(-60, 260), 34),
      rock('post', vec(240, 330), 28),
      sun('flare', vec(-400, -240), 40, 460, { range: 260 }),
      {
        id: 'flipper',
        shape: rectPolygon(vec(-380, -20), 120, 10),
        material: 'bouncy',
        style: 'bumper',
        motion: { kind: 'spin', speed: 2.2, phase: 0 },
      },
    ],
    portals: [
      {
        id: 'kicker',
        from: vec(560, -300),
        to: vec(-500, -120),
        radius: 32,
        twist: 0.6,
        boost: 1.15,
        bidirectional: false,
      },
    ],
    stars: [vec(-260, -180), vec(220, -240), vec(520, 20)],
  },
];
