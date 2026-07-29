import { TAU } from '../../core/math';
import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import { BALL_RADIUS, bumper, onSurface, planet, rock, wall, type LevelDef } from '../level';

const BOUNDS = { minX: -640, minY: -400, maxX: 640, maxY: 400 };

/**
 * A very large, very distant body below the play area. Its huge radius makes
 * the surface read as gently curved ground and its gravity nearly uniform —
 * familiar footing for the tutorial holes before space gets strange.
 */
const GROUND = { center: vec(0, 2600), radius: 2280 };

/** A point resting on the ground surface at world x. */
const groundAt = (x: number, gap = BALL_RADIUS + 1): { x: number; y: number } => {
  const dx = x - GROUND.center.x;
  const dy = Math.sqrt(Math.max(0, GROUND.radius * GROUND.radius - dx * dx));
  return { x, y: GROUND.center.y - dy - gap };
};

const groundBody = (gravity = 300) =>
  planet('ground', GROUND.center, GROUND.radius, gravity, { style: 'planet' });

/**
 * Chapter 1 — Orbital Basics.
 * Teaches, in order: aiming, gravity as a curve, slingshotting round a planet,
 * bouncing, threading two wells, and reading a moving obstacle.
 */
export const CHAPTER_1: LevelDef[] = [
  {
    id: 'c1-1',
    name: 'First Light',
    chapter: 0,
    par: 2,
    tee: groundAt(-430),
    hole: { position: groundAt(440, 2) },
    bounds: BOUNDS,
    // An enclosed practice green: overhitting on the very first hole should
    // bounce off the back wall, not cost a penalty stroke.
    boundsMode: 'wall',
    hint: 'Drag back from the ball and release to putt.',
    bodies: [groundBody()],
    // Low along the ground, then one that rewards getting the ball airborne.
    stars: [vec(-150, 300), vec(0, 40), vec(220, 290)],
  },
  {
    id: 'c1-2',
    name: 'Small Pull',
    chapter: 0,
    par: 2,
    tee: vec(-470, -180),
    hole: { position: vec(450, 240) },
    bounds: BOUNDS,
    hint: 'Gravity curves your shot. Aim wide and let the planet do the rest.',
    ambientDrag: 0.12,
    bodies: [planet('p1', vec(40, 120), 100, 700, { range: 640 })],
    stars: [vec(-190, -110), vec(120, -140), vec(330, 60)],
  },
  {
    id: 'c1-3',
    name: 'Slingshot',
    chapter: 0,
    par: 2,
    tee: vec(-500, -250),
    hole: { position: vec(480, -250) },
    bounds: BOUNDS,
    hint: 'Swing around a planet to steal a change of direction.',
    ambientDrag: 0.1,
    bodies: [
      planet('p1', vec(0, 90), 115, 1000, { range: 720 }),
      wall('gate-l', vec(-150, -400), vec(-150, -150), 10),
      wall('gate-r', vec(150, -400), vec(150, -150), 10),
    ],
    stars: [vec(0, -120), vec(-320, 60), vec(320, 60)],
  },
  {
    id: 'c1-4',
    name: 'Bumper Alley',
    chapter: 0,
    par: 3,
    tee: groundAt(-480),
    hole: { position: groundAt(470, 2) },
    bounds: BOUNDS,
    hint: 'Bumpers give back more than you put in.',
    bodies: [
      groundBody(280),
      wall('divider', vec(0, 380), vec(0, 40), 12),
      bumper('b1', vec(-210, 90), 44),
      bumper('b2', vec(180, 10), 44),
      bumper('b3', vec(340, 190), 34),
    ],
    stars: [vec(-330, 120), vec(0, -110), vec(300, 60)],
  },
  {
    id: 'c1-5',
    name: 'Two Moons',
    chapter: 0,
    par: 3,
    tee: vec(-520, 0),
    hole: { position: onSurface(vec(240, 0), 34, 0, 4) },
    bounds: BOUNDS,
    hint: 'Between two wells there is always a quiet line.',
    ambientDrag: 0.1,
    bodies: [
      planet('m1', vec(0, -250), 85, 820, { range: 560, style: 'moon' }),
      planet('m2', vec(0, 250), 85, 820, { range: 560, style: 'moon' }),
      rock('perch', vec(240, 0), 34),
    ],
    stars: [vec(-250, 0), vec(0, 0), vec(390, -150)],
  },
  {
    id: 'c1-6',
    name: 'The Narrows',
    chapter: 0,
    par: 3,
    tee: vec(-540, -300),
    hole: { position: vec(540, 300) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Walls are your friend. Bank the shot.',
    ambientDrag: 0.12,
    bodies: [
      planet('core', vec(0, 0), 75, 760, { range: 430 }),
      wall('w1', vec(-270, -400), vec(-270, -140), 12),
      wall('w2', vec(270, 400), vec(270, 140), 12),
      wall('w3', vec(-110, 210), vec(110, 210), 12),
      wall('w4', vec(-110, -210), vec(110, -210), 12),
      {
        id: 'spinner',
        shape: rectPolygon(vec(0, 0), 160, 9),
        material: 'metal',
        style: 'wall',
        motion: { kind: 'spin', speed: 0.9, phase: TAU * 0.25 },
      },
    ],
    stars: [vec(-400, 140), vec(0, -320), vec(400, -80)],
  },
];
