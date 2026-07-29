import { TAU } from '../../core/math';
import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import {
  blackHole,
  bumper,
  onSurface,
  planet,
  rock,
  sun,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -740, minY: -450, maxX: 740, maxY: 450 };

/**
 * Chapter 5 — Singularity.
 * The finale. Every mechanic returns at once, and the holes assume you can
 * already read a gravity field at a glance.
 */
export const CHAPTER_5: LevelDef[] = [
  {
    id: 'c5-1',
    name: 'Binary',
    chapter: 4,
    par: 4,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(560, 0), 54, 0, 3) },
    bounds: BOUNDS,
    hint: 'Two suns orbiting each other. The safe gap moves.',
    ambientDrag: 0.08,
    bodies: [
      sun('sun-a', vec(0, 0), 46, 820, {
        range: 460,
        motion: { kind: 'orbit', center: vec(-20, 0), radius: 150, speed: 0.85, phase: 0 },
      }),
      sun('sun-b', vec(0, 0), 46, 820, {
        range: 460,
        motion: { kind: 'orbit', center: vec(-20, 0), radius: 150, speed: 0.85, phase: Math.PI },
      }),
      planet('anchor', vec(560, 0), 54, 620, { range: 380 }),
      planet('feeder', vec(-380, 160), 58, 560, { range: 340, style: 'moon' }),
    ],
    stars: [vec(-20, -330), vec(-20, 330), vec(330, -180)],
  },
  {
    id: 'c5-2',
    name: 'Cat and Mouse',
    chapter: 4,
    par: 4,
    tee: vec(-680, 340),
    hole: { position: vec(660, -360) },
    bounds: BOUNDS,
    hint: 'The nebula robs your speed. Get through it with something left.',
    ambientDrag: 0.06,
    bodies: [
      planet('p1', vec(-360, 40), 70, 700, { range: 420, style: 'moon' }),
      planet('p2', vec(240, -140), 70, 700, { range: 420, style: 'moon' }),
      blackHole('bh', vec(-40, 260), 32, 1400, { range: 400 }),
      wall('shelf', vec(380, -300), vec(700, -300), 12),
      bumper('b1', vec(460, 160), 40),
    ],
    zones: [
      {
        id: 'fog',
        kind: 'nebula',
        area: { kind: 'circle', center: vec(-60, -120), radius: 210 },
        drag: 2.4,
      },
    ],
    stars: [vec(-60, -120), vec(-420, -260), vec(240, 300)],
  },
  {
    id: 'c5-3',
    name: 'Maelstrom',
    chapter: 4,
    par: 4,
    tee: vec(-560, 380),
    hole: { position: vec(600, 386) },
    bounds: BOUNDS,
    hint: 'The vortex spins you outward. Enter on the side you want to leave.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 360),
    bodies: [
      wall('floor-l', vec(-720, 400), vec(-150, 400), 12),
      wall('floor-r', vec(250, 400), vec(720, 400), 12),
      rock('eye', vec(50, 250), 34),
      bumper('kick', vec(-330, 120), 40),
    ],
    zones: [
      {
        id: 'swirl',
        kind: 'vortex',
        area: { kind: 'circle', center: vec(50, 250), radius: 230 },
        strength: 70,
        swirl: 280,
      },
    ],
    stars: [vec(50, 20), vec(-380, -160), vec(430, 120)],
  },
  {
    id: 'c5-4',
    name: 'Gauntlet',
    chapter: 4,
    par: 5,
    tee: vec(-690, 380),
    hole: { position: vec(690, -380) },
    bounds: BOUNDS,
    hint: 'Four rooms, one route. Take it a room at a time.',
    ambientDrag: 0.14,
    bodies: [
      planet('g1', vec(-420, 120), 62, 700, { range: 340, style: 'moon' }),
      planet('g2', vec(-60, -160), 62, 700, { range: 340, style: 'moon' }),
      planet('g3', vec(300, 140), 62, 700, { range: 340, style: 'moon' }),
      sun('haz-1', vec(-230, -60), 34, 400, { range: 220 }),
      sun('haz-2', vec(130, 40), 34, 400, { range: 220 }),
      blackHole('haz-3', vec(500, -120), 28, 1200, { range: 300 }),
      wall('wall-1', vec(-560, -450), vec(-560, -180), 12),
      wall('wall-2', vec(560, 450), vec(560, 180), 12),
      wall('shelf', vec(430, -300), vec(700, -300), 12),
    ],
    stars: [vec(-230, 300), vec(130, -320), vec(560, 60)],
  },
  {
    id: 'c5-5',
    name: 'Hall of Mirrors',
    chapter: 4,
    par: 4,
    tee: vec(-660, 380),
    hole: { position: vec(660, 380) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Every wall here is ice. Speed is the problem, not the answer.',
    ambientDrag: 0.05,
    bodies: [
      planet('core', vec(0, -40), 88, 720, { range: 560 }),
      wall('m1', vec(-420, -160), vec(-160, -400), 14, { material: 'ice', style: 'ice' }),
      wall('m2', vec(420, -160), vec(160, -400), 14, { material: 'ice', style: 'ice' }),
      wall('m3', vec(-460, 200), vec(-200, 200), 14, { material: 'ice', style: 'ice' }),
      wall('m4', vec(460, 200), vec(200, 200), 14, { material: 'ice', style: 'ice' }),
      wall('sink-l', vec(-680, 420), vec(-300, 420), 14, { material: 'sand', style: 'sand' }),
      wall('sink-r', vec(300, 420), vec(680, 420), 14, { material: 'sand', style: 'sand' }),
    ],
    stars: [vec(0, -300), vec(-300, 20), vec(300, 20)],
  },
  {
    id: 'c5-6',
    name: 'Event Horizon',
    chapter: 4,
    par: 5,
    tee: vec(-690, 0),
    hole: { position: onSurface(vec(600, 0), 56, 0, 3) },
    bounds: BOUNDS,
    hint: 'Everything at once. Read the field, pick your moment.',
    ambientDrag: 0.07,
    bodies: [
      blackHole('maw', vec(0, 0), 44, 1900, { range: 560 }),
      planet('anchor', vec(600, 0), 56, 620, { range: 360 }),
      planet('feeder', vec(-440, -180), 60, 620, { range: 360, style: 'moon' }),
      rock('orbiter', vec(0, 0), 26, {
        motion: { kind: 'orbit', center: vec(0, 0), radius: 260, speed: -1.05, phase: TAU * 0.3 },
      }),
      bumper('b1', vec(-160, 340), 42),
      bumper('b2', vec(220, -340), 42),
      {
        id: 'gate',
        shape: rectPolygon(vec(380, 0), 12, 130),
        material: 'metal',
        style: 'wall',
        motion: { kind: 'spin', speed: 1.4, phase: 0 },
      },
    ],
    zones: [
      {
        id: 'calm',
        kind: 'gravityScale',
        area: { kind: 'circle', center: vec(-250, 200), radius: 150 },
        scale: 0.15,
      },
    ],
    stars: [vec(-250, 200), vec(0, -330), vec(430, 250)],
  },
];
