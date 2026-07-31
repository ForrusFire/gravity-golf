import { vec } from '../../core/vec2';
import { rectPolygon } from '../../physics/geometry';
import {
  blackHole,
  booster,
  bumper,
  onSurface,
  planet,
  pulsingWall,
  rock,
  sun,
  tollGate,
  wall,
  type LevelDef,
} from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

const DOWN = vec(0, 1);

/**
 * Chapter 14 — Tides.
 * Until now a hole's *fields* were the one fixed thing: walls could vanish and
 * cups could travel, but wind blew and gravity pulled whatever the clock said.
 * Here the fields breathe. A gust that comes and goes, a hazard field only
 * lethal half the time, a gravity well that switches off — each wears the same
 * countdown ring as a pulsing wall and goes faint rather than invisible, so what
 * is coming is always on screen.
 */
export const CHAPTER_14: LevelDef[] = [
  {
    id: 'c14-1',
    name: 'Gust',
    chapter: 13,
    par: 3,
    tee: vec(-640, 300),
    hole: { position: vec(620, 306) },
    bounds: BOUNDS,
    hint: 'The wind blows in bursts. Wait one out, or ride it.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 360),
    bodies: [
      wall('floor', vec(-720, 330), vec(720, 330), 12),
      bumper('kick', vec(-320, 150), 42),
      rock('post', vec(180, 280), 26),
    ],
    zones: [
      {
        id: 'gust',
        kind: 'wind',
        area: rectPolygon(vec(120, 60), 260, 380),
        // Hard enough to matter, brief enough to wait out.
        force: vec(-620, 0),
        pulse: { period: 4.5, duty: 0.5, phase: 0 },
      },
    ],
    stars: [vec(-320, 30), vec(120, -160), vec(480, 160)],
  },
  {
    id: 'c14-2',
    name: 'Slack Water',
    chapter: 13,
    par: 3,
    tee: vec(-660, 0),
    hole: { position: onSurface(vec(560, 0), 54, 0, 3) },
    bounds: BOUNDS,
    hint: 'The pull switches off for a moment. That moment is the shot.',
    ambientDrag: 0.1,
    bodies: [
      planet('anchor', vec(560, 0), 54, 620, { range: 340, material: 'sand', style: 'sand' }),
      planet('core', vec(-40, 0), 78, 780, { range: 520 }),
      rock('post', vec(260, -240), 28),
    ],
    zones: [
      {
        // Cancels the core's pull across the middle of the field, on and off.
        id: 'slack',
        kind: 'gravityScale',
        area: { kind: 'circle', center: vec(-40, 0), radius: 280 },
        scale: 0.1,
        pulse: { period: 5, duty: 0.45, phase: 0 },
      },
    ],
    stars: [vec(-40, -300), vec(-40, 300), vec(400, 220)],
  },
  {
    id: 'c14-3',
    name: 'Shallows',
    chapter: 13,
    par: 3,
    tee: vec(-660, 320),
    hole: { position: vec(640, -280) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'The red field is only deadly half the time. Cross while it is faint.',
    ambientDrag: 0.1,
    uniformGravity: vec(0, 300),
    bodies: [
      wall('floor', vec(-720, 350), vec(-120, 350), 12),
      wall('shelf', vec(500, -250), vec(720, -250), 12),
      wall('lip', vec(500, -262), vec(500, -340), 10),
    ],
    zones: [
      {
        id: 'tide',
        kind: 'hazard',
        area: rectPolygon(vec(160, -40), 130, 400),
        pulse: { period: 3.6, duty: 0.5, phase: 0 },
      },
    ],
    boosters: [booster('ring', vec(-300, 140), vec(0.55, -1), 720, 32)],
    stars: [vec(-300, 140), vec(160, -320), vec(560, -60)],
  },
  {
    id: 'c14-4',
    name: 'Spring Tide',
    chapter: 13,
    par: 4,
    tee: vec(-680, 300),
    hole: {
      position: vec(600, 306),
      approach: { direction: DOWN, tolerance: 0.7 },
    },
    bounds: BOUNDS,
    hint: 'Two gusts, opposite phases. One of them is always pushing.',
    ambientDrag: 0.12,
    uniformGravity: vec(0, 340),
    bodies: [
      wall('floor-l', vec(-720, 330), vec(240, 330), 12),
      wall('floor-r', vec(440, 330), vec(720, 330), 12),
      pulsingWall('beat', vec(60, 318), vec(60, 40), { period: 4, duty: 0.5 }),
      bumper('kick', vec(-420, 150), 40),
    ],
    zones: [
      {
        id: 'flood',
        kind: 'wind',
        area: rectPolygon(vec(-220, 40), 180, 340),
        force: vec(420, -120),
        pulse: { period: 4, duty: 0.5, phase: 0 },
      },
      {
        id: 'ebb',
        kind: 'wind',
        area: rectPolygon(vec(300, 40), 180, 340),
        force: vec(-420, -120),
        pulse: { period: 4, duty: 0.5, phase: 0.5 },
      },
    ],
    stars: [vec(-420, 40), vec(60, -140), vec(300, 100)],
  },
  {
    id: 'c14-5',
    name: 'Undertow',
    chapter: 13,
    par: 4,
    tee: vec(-660, 0),
    hole: {
      position: vec(560, 0),
      motion: { kind: 'orbit', center: vec(380, 0), radius: 180, speed: 0.6, phase: 0 },
    },
    bounds: BOUNDS,
    hint: 'A whirlpool that comes and goes, and a cup that never stops.',
    ambientDrag: 0.08,
    bodies: [
      planet('hub', vec(380, 0), 66, 680, { range: 400, material: 'sand', style: 'sand' }),
      planet('feeder', vec(-320, 140), 62, 620, { range: 340, style: 'moon' }),
      tollGate('toll', vec(60, -440), vec(60, 440), 1),
    ],
    zones: [
      {
        id: 'eddy',
        kind: 'vortex',
        area: { kind: 'circle', center: vec(180, -60), radius: 180 },
        strength: -260,
        swirl: 320,
        pulse: { period: 4.8, duty: 0.5, phase: 0 },
      },
    ],
    stars: [vec(-320, -180), vec(-140, 240), vec(180, -60)],
  },
  {
    id: 'c14-6',
    name: 'High Water',
    chapter: 13,
    par: 5,
    tee: vec(-690, 340),
    hole: {
      position: onSurface(vec(600, -60), 56, Math.PI, 3),
      requiresStars: 3,
    },
    bounds: BOUNDS,
    boundsMode: 'wall',
    hint: 'Everything on a clock at once. Read the rings before you pull back.',
    ambientDrag: 0.08,
    bodies: [
      wall('floor', vec(-720, 380), vec(-300, 380), 12),
      planet('anchor', vec(600, -60), 56, 700, { range: 400, material: 'sand', style: 'sand' }),
      planet('core', vec(-120, 40), 66, 680, { range: 400 }),
      pulsingWall('shutter', vec(260, -440), vec(260, 40), { period: 4.2, duty: 0.5 }),
      blackHole('maw', vec(260, 300), 26, 850, { range: 280 }),
      sun('flare', vec(-120, -300), 30, 380, { range: 230 }),
    ],
    zones: [
      {
        id: 'surge',
        kind: 'wind',
        area: rectPolygon(vec(60, 60), 200, 340),
        force: vec(0, -520),
        pulse: { period: 5.2, duty: 0.45, phase: 0.2 },
      },
    ],
    boosters: [booster('ring', vec(-460, 140), vec(0.85, -1), 660, 32)],
    stars: [vec(-460, -180), vec(60, 260), vec(420, 260)],
  },
];
