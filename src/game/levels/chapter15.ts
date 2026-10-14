import { vec } from '../../core/vec2';
import { bumper, onSurface, planet, rock, sun, wall, type LevelDef } from '../level';

const BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

/**
 * Solid walls just inside the edges. Down can point at any side here, so any
 * side may be the floor the ball comes to rest on — and the boundary itself
 * is frictionless, so a ball against it would never settle.
 */
const box = (): ReturnType<typeof wall>[] => [
  wall('box-floor', vec(-708, 428), vec(708, 428), 10),
  wall('box-ceiling', vec(-708, -428), vec(708, -428), 10),
  wall('box-west', vec(-708, -428), vec(-708, 428), 10),
  wall('box-east', vec(708, -428), vec(708, 428), 10),
];

/**
 * Chapter 15 — Tumble.
 * Every hole so far has had a fixed "down". Here it turns while the ball waits:
 * the background pull sweeps round the compass as you line up, and holds the
 * moment you strike. The dial in the corner shows where it points now and
 * where it is heading. A floor becomes a wall becomes a ceiling, so the same
 * aim from the same spot is a different shot a few seconds later — the
 * question stops being only "where" and becomes "when".
 */
export const CHAPTER_15: LevelDef[] = [
  {
    id: 'c15-1',
    name: 'Quarter Turn',
    chapter: 14,
    par: 2,
    tee: vec(-600, 311),
    hole: { position: vec(560, 312) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    // A tilted floor rolls a ball a long way before it settles.
    shotTimeout: 30,
    hint: 'Down turns while you wait — watch the dial, and strike when the floor is the one you want.',
    ambientDrag: 0.16,
    uniformGravity: vec(0, 340),
    gravitySpin: { speed: 0.22 },
    bodies: [
      ...box(),
      wall('floor', vec(-698, 330), vec(698, 330), 10),
      wall('column', vec(0, 330), vec(0, -80), 14),
      wall('lip-l', vec(492, 330), vec(492, 250), 9),
      wall('lip-r', vec(628, 330), vec(628, 250), 9),
    ],
    stars: [vec(-300, 120), vec(0, -200), vec(320, 100)],
  },
  {
    id: 'c15-2',
    name: 'Weathervane',
    chapter: 14,
    par: 2,
    tee: onSurface(vec(-260, 80), 110, -Math.PI / 2),
    hole: { position: onSurface(vec(420, -60), 56, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'The planet holds you while down turns. Choose the moment the gap tilts your way.',
    ambientDrag: 0.1,
    uniformGravity: vec(0, 110),
    gravitySpin: { speed: 0.3, phase: 1 },
    bodies: [
      planet('home', vec(-260, 80), 110, 900, { range: 520 }),
      planet('anchor', vec(420, -60), 56, 640, { range: 300, material: 'sand', style: 'sand' }),
      rock('stone', vec(90, -220), 24, { style: 'asteroid' }),
    ],
    stars: [vec(-260, -150), vec(80, 40), vec(420, 140)],
  },
  {
    id: 'c15-3',
    name: 'Tumble Dryer',
    chapter: 14,
    par: 3,
    tee: vec(-560, 410),
    hole: { position: vec(0, -404) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    // A tilted floor rolls a ball a long way before it settles.
    shotTimeout: 30,
    hint: 'The cup is in the ceiling. Wait for up to become down, then drop in.',
    ambientDrag: 0.18,
    uniformGravity: vec(0, 240),
    gravitySpin: { speed: 0.36 },
    bodies: [
      ...box(),
      wall('lip-l', vec(-72, -418), vec(-72, -350), 9),
      wall('lip-r', vec(72, -418), vec(72, -350), 9),
      bumper('drum-a', vec(-280, 40), 46),
      bumper('drum-b', vec(280, 40), 46),
      sun('pilot', vec(0, 120), 34, 260, { range: 200 }),
    ],
    stars: [vec(-420, -200), vec(0, -40), vec(420, 240)],
  },
  {
    id: 'c15-4',
    name: 'Clockwork',
    chapter: 14,
    par: 3,
    tee: onSurface(vec(-480, 160), 70, -Math.PI / 2),
    hole: { position: onSurface(vec(470, 120), 62, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    hint: 'A sun at the hub, and the pull around it turning like a clock hand. Pick your hour.',
    ambientDrag: 0.1,
    uniformGravity: vec(0, 95),
    gravitySpin: { speed: 0.42, phase: 0 },
    bodies: [
      planet('west', vec(-480, 160), 70, 700, { range: 380, style: 'moon' }),
      planet('north', vec(0, -250), 64, 650, { range: 340 }),
      planet('east', vec(470, 120), 62, 640, { range: 330, material: 'sand', style: 'sand' }),
      sun('hub', vec(0, 90), 40, 300, { range: 220 }),
    ],
    stars: [vec(-240, -120), vec(0, -380), vec(250, -100)],
  },
  {
    id: 'c15-5',
    name: 'Hourglass',
    chapter: 14,
    par: 3,
    tee: vec(-560, 410),
    hole: { position: onSurface(vec(480, 0), 44, -Math.PI / 2, 3) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    // A tilted floor rolls a ball a long way before it settles.
    shotTimeout: 30,
    hint: 'One narrow neck between the chambers. Turn the glass, then pour yourself through.',
    ambientDrag: 0.16,
    uniformGravity: vec(0, 300),
    gravitySpin: { speed: 0.3 },
    bodies: [
      ...box(),
      wall('upper', vec(0, -418), vec(0, -105), 12),
      wall('lower', vec(0, 418), vec(0, 105), 12),
      planet('perch', vec(480, 0), 44, 620, { range: 260, material: 'sand', style: 'sand' }),
      bumper('spill', vec(-300, -120), 40),
    ],
    stars: [vec(-300, 200), vec(0, 0), vec(300, 250)],
  },
  {
    id: 'c15-6',
    name: 'Revolution',
    chapter: 14,
    par: 4,
    tee: vec(560, 410),
    hole: { position: vec(-680, 0) },
    bounds: BOUNDS,
    boundsMode: 'wall',
    // A tilted floor rolls a ball a long way before it settles.
    shotTimeout: 30,
    hint: 'Faster now. The cup is in the west wall — strike while it is the floor.',
    ambientDrag: 0.24,
    uniformGravity: vec(0, 280),
    gravitySpin: { speed: 0.5 },
    bodies: [
      ...box(),
      wall('lip-n', vec(-698, -72), vec(-620, -72), 9),
      wall('lip-s', vec(-698, 72), vec(-620, 72), 9),
      sun('core', vec(0, 0), 46, 260, { range: 160 }),
      rock('ne', vec(330, -230), 40, { style: 'asteroid' }),
      rock('sw', vec(-330, 230), 40, { style: 'asteroid' }),
      rock('nw', vec(-360, -260), 30),
    ],
    stars: [vec(300, 160), vec(0, -300), vec(-460, 0)],
  },
];
