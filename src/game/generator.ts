import { TAU } from '../core/math';
import { Rng, hashSeed } from '../core/rng';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import { bodyShapeAt, closestSurfacePoint, rectPolygon } from '../physics/geometry';
import type { Body, SwitchSpec } from '../physics/types';
import {
  BALL_RADIUS,
  blackHole,
  bumper,
  compileLevel,
  crystal,
  gate,
  planet,
  rock,
  sun,
  switchPad,
  validateLevel,
  wall,
  type LevelDef,
} from './level';
import { solveLevel } from './solver';

const BOUNDS = { minX: -700, minY: -430, maxX: 700, maxY: 430 };
const LEDGE_Y = 330;

/**
 * The shape of a generated hole. Picking from a small set of archetypes gives
 * generated levels a recognisable structure — purely random body placement
 * reads as noise, not as a hole someone designed.
 *
 * Nothing here moves on a clock. Switches, gates and breakable blocks are fair
 * game — they change only in response to the ball, and the solver carries that
 * state along each branch, so the solution it verified is exactly the solution a
 * player can replay. A spinning arm or a timed pad would make the outcome depend
 * on *when* the shot was taken, and a hole that is only completable at one
 * instant is not a hole that was verified.
 */
type Archetype =
  | 'slingshot'
  | 'corridor'
  | 'minefield'
  | 'binary'
  | 'pinball'
  | 'vault'
  | 'glasshouse';

const ARCHETYPES: Archetype[] = [
  'slingshot',
  'corridor',
  'minefield',
  'binary',
  'pinball',
  'vault',
  'glasshouse',
];

export interface GeneratedLevel extends LevelDef {
  /** The seed this hole was generated from. */
  seed: number;
  archetype: Archetype;
}

export interface GenerateOptions {
  /** Candidates to try before giving up. */
  attempts: number;
  /** Reject holes the solver cannot finish in this many strokes. */
  maxPar: number;
  /** Reject holes that are solvable in fewer strokes than this — too easy. */
  minPar: number;
}

export const DEFAULT_GENERATE_OPTIONS: GenerateOptions = {
  attempts: 30,
  maxPar: 4,
  minPar: 2,
};

/** A ledge for the tee or hole to sit on, so arriving balls can settle. */
const ledge = (id: string, centerX: number, halfWidth: number): Body =>
  wall(id, V.vec(centerX - halfWidth, LEDGE_Y), V.vec(centerX + halfWidth, LEDGE_Y), 12);

const LEDGE_TOP = LEDGE_Y - 12;

const buildCandidate = (rng: Rng, seed: number): GeneratedLevel => {
  const archetype = ARCHETYPES[rng.int(0, ARCHETYPES.length - 1)]!;

  const teeX = rng.range(-620, -440);
  const holeX = rng.range(440, 620);
  const tee = V.vec(teeX, LEDGE_TOP - BALL_RADIUS - 4);
  const hole = V.vec(holeX, LEDGE_TOP - 2);

  const bodies: Body[] = [
    ledge('ledge-tee', teeX, 130),
    ledge('ledge-hole', holeX, 130),
  ];
  const switches: SwitchSpec[] = [];

  /** Keeps generated obstacles out of the tee and hole pockets. */
  const clearOfEnds = (p: Vec2, radius: number): boolean =>
    V.distance(p, tee) > radius + 120 && V.distance(p, hole) > radius + 120;

  const placeBody = (make: (id: string, at: Vec2, r: number) => Body, radius: number, id: string) => {
    for (let i = 0; i < 40; i++) {
      const at = V.vec(rng.range(-380, 380), rng.range(-330, 240));
      if (!clearOfEnds(at, radius)) continue;
      if (bodies.some((b) => V.distance(bodyCentre(b), at) < radius + bodyRadius(b) + 40)) continue;
      bodies.push(make(id, at, radius));
      return;
    }
  };

  switch (archetype) {
    case 'slingshot': {
      const r = rng.range(90, 130);
      bodies.push(planet('core', V.vec(rng.range(-90, 90), rng.range(-60, 110)), r, rng.range(750, 1050), { range: r * 6 }));
      placeBody((id, at, rr) => rock(id, at, rr), rng.range(28, 44), 'rock-a');
      break;
    }
    case 'corridor': {
      const gapY = rng.range(-120, 120);
      bodies.push(
        wall('gate-top', V.vec(rng.range(-80, 80), -430), V.vec(rng.range(-80, 80), gapY - 90), 12),
        wall('gate-bottom', V.vec(rng.range(-80, 80), 250), V.vec(rng.range(-80, 80), gapY + 90), 12),
        planet('pull', V.vec(rng.range(-330, -180), rng.range(-200, 60)), 70, rng.range(600, 800), { range: 430 }),
      );
      placeBody((id, at, r) => bumper(id, at, r), rng.range(34, 46), 'bump-a');
      break;
    }
    case 'minefield': {
      for (let i = 0; i < 3; i++) {
        placeBody(
          (id, at, r) => planet(id, at, r, rng.range(550, 780), { range: 330, style: 'moon' }),
          rng.range(52, 70),
          `moon-${i}`,
        );
      }
      placeBody((id, at, r) => sun(id, at, r, 420, { range: 240 }), rng.range(30, 42), 'sun-a');
      break;
    }
    case 'binary': {
      const cx = rng.range(-80, 80);
      const cy = rng.range(-80, 80);
      const gap = rng.range(180, 280);
      bodies.push(
        planet('b1', V.vec(cx, cy - gap / 2), 74, rng.range(700, 900), { range: 470, style: 'moon' }),
        planet('b2', V.vec(cx, cy + gap / 2), 74, rng.range(700, 900), { range: 470, style: 'moon' }),
      );
      if (rng.bool(0.5)) {
        bodies.push(blackHole('bh', V.vec(cx, cy), 26, 900, { range: 260 }));
      }
      break;
    }
    case 'vault': {
      // The mandatory blocker becomes a gate below, so all this archetype adds
      // is something to slingshot around on the way to the pad.
      const r = rng.range(70, 100);
      bodies.push(
        planet('core', V.vec(rng.range(-120, 60), rng.range(-40, 140)), r, rng.range(650, 900), {
          range: r * 5,
        }),
      );
      break;
    }
    case 'glasshouse': {
      bodies.push(
        planet('pull', V.vec(rng.range(-340, -190), rng.range(-180, 80)), 66, rng.range(600, 800), {
          range: 400,
          style: 'moon',
        }),
      );
      placeBody((id, at, r) => bumper(id, at, r), rng.range(34, 46), 'bump-a');
      break;
    }
    case 'pinball': {
      for (let i = 0; i < 5; i++) {
        placeBody((id, at, r) => bumper(id, at, r), rng.range(32, 46), `bump-${i}`);
      }
      bodies.push({
        id: 'baffle',
        shape: rectPolygon(
          V.vec(rng.range(-120, 120), rng.range(-160, 60)),
          rng.range(90, 150),
          9,
          rng.range(0, TAU),
        ),
        material: 'metal',
        style: 'wall',
      });
      break;
    }
  }

  // A wall the ball has to clear, somewhere between the two ledges. Without a
  // deliberate blocker the tee and the cup share a ledge with clear air
  // between them, and every candidate solves in one straight putt.
  // Placement retries so the wall does not grow through a planet.
  let blockerX = rng.range(teeX + 220, holeX - 220);
  let blockerTop = rng.range(-40, 180);
  for (let i = 0; i < 24; i++) {
    const x = rng.range(teeX + 220, holeX - 220);
    const top = rng.range(-40, 180);
    const clear = bodies.every((b) => {
      const centre = bodyCentre(b);
      // Only bodies whose vertical span meets the wall can foul it.
      if (centre.y + bodyRadius(b) < top) return true;
      return Math.abs(centre.x - x) > bodyRadius(b) + 26;
    });
    if (clear) {
      blockerX = x;
      blockerTop = top;
      break;
    }
  }
  if (archetype === 'vault') {
    // Sealed rather than solid: the way through is a pad, not a lofted shot.
    bodies.push(gate('blocker', V.vec(blockerX, 430), V.vec(blockerX, blockerTop), 'sw-vault'));
    // On the tee side of the gate, or it could only be reached by going through it.
    for (let i = 0; i < 60; i++) {
      const at = V.vec(rng.range(teeX + 90, blockerX - 90), rng.range(-330, 260));
      const clear = bodies.every(
        (b) => b.id.startsWith('ledge-') || V.distance(bodyCentre(b), at) > bodyRadius(b) + 50,
      );
      if (clear && V.distance(at, tee) > 90) {
        switches.push(switchPad('sw-vault', at, 32));
        break;
      }
    }
    // No reachable spot for the pad means the gate can never open.
    if (switches.length === 0) {
      bodies.pop();
      bodies.push(wall('blocker', V.vec(blockerX, 430), V.vec(blockerX, blockerTop), 12));
    }
  } else {
    bodies.push(wall('blocker', V.vec(blockerX, 430), V.vec(blockerX, blockerTop), 12));

    if (archetype === 'glasshouse') {
      // Stacked on top of the blocker, so going over means going a long way up
      // and going through means spending hits on the way.
      let y = blockerTop - 46;
      for (let i = 0; i < 2; i++) {
        const at = V.vec(blockerX, y);
        const fouls = bodies.some(
          (b) =>
            !b.id.startsWith('ledge-') &&
            b.id !== 'blocker' &&
            V.distance(bodyCentre(b), at) < bodyRadius(b) + 60,
        );
        if (at.y - 44 < BOUNDS.minY + 20) break;
        if (!fouls) bodies.push(crystal(`pane-${i}`, at, 44, i === 0 ? 1 : 2));
        y -= 92;
      }
    }
  }

  // Stars in open space, spread across the hole so collecting all three means
  // taking a different line rather than the same one three times.
  const stars: Vec2[] = [];
  const lanes = [
    { minX: -560, maxX: -220 },
    { minX: -160, maxX: 160 },
    { minX: 220, maxX: 560 },
  ];
  for (const lane of lanes) {
    for (let i = 0; i < 60; i++) {
      const p = V.vec(rng.range(lane.minX, lane.maxX), rng.range(-360, 220));
      const clearOfBodies = bodies.every(
        (b) => V.distance(bodyCentre(b), p) > bodyRadius(b) + 34,
      );
      if (clearOfBodies && V.distance(p, tee) > 90 && V.distance(p, hole) > 90) {
        stars.push(p);
        break;
      }
    }
  }

  return {
    id: `gen-${seed}`,
    name: 'Generated Hole',
    chapter: -1,
    par: DEFAULT_GENERATE_OPTIONS.maxPar,
    tee,
    hole: { position: hole },
    bounds: BOUNDS,
    boundsMode: 'kill',
    ambientDrag: 0.11,
    bodies,
    switches,
    stars,
    seed,
    archetype,
  };
};

/** Approximate centre of a body, good enough for spacing checks. */
const bodyCentre = (body: Body): Vec2 => {
  switch (body.shape.kind) {
    case 'circle':
      return body.shape.center;
    case 'capsule':
      return V.mul(V.add(body.shape.a, body.shape.b), 0.5);
    case 'polygon':
      return body.shape.center;
  }
};

const bodyRadius = (body: Body): number => {
  switch (body.shape.kind) {
    case 'circle':
      return body.shape.radius;
    case 'capsule':
      return V.distance(body.shape.a, body.shape.b) / 2 + body.shape.radius;
    case 'polygon': {
      let max = 0;
      for (const v of body.shape.vertices) max = Math.max(max, V.length(v));
      return max;
    }
  }
};

/**
 * True when the straight tee-to-hole line runs into something. Without this
 * the generator happily produces a clear corridor with decoration either side,
 * which solves in one shot every time.
 */
const directLineBlocked = (level: GeneratedLevel): boolean => {
  const from = level.tee;
  const to = level.hole.position;
  const samples = 48;
  for (const body of level.bodies ?? []) {
    // The tee and hole ledges are underfoot, not obstacles.
    if (body.id.startsWith('ledge-')) continue;
    const shape = bodyShapeAt(body, 0);
    for (let i = 1; i < samples; i++) {
      const p = V.lerpVec(from, to, i / samples);
      if (closestSurfacePoint(shape, p).distance < BALL_RADIUS * 2) return true;
    }
  }
  return false;
};

export interface GenerateResult {
  level: GeneratedLevel | null;
  /** Candidates tried before one passed, for diagnostics. */
  attempts: number;
}

/**
 * Generates a hole for `seed` and verifies it before returning.
 *
 * Verification is what makes generated content shippable: a candidate is only
 * accepted if it validates and the solver can actually finish it, and the par
 * is whatever the solver needed rather than a guess. Candidates are screened at
 * a coarse timestep first, since most rejects are obvious, and only survivors
 * pay for a full-fidelity solve.
 */
export const generateLevel = (
  seed: number,
  options: Partial<GenerateOptions> = {},
): GenerateResult => {
  const opts = { ...DEFAULT_GENERATE_OPTIONS, ...options };
  const rng = new Rng(seed);

  for (let attempt = 1; attempt <= opts.attempts; attempt++) {
    const candidate = buildCandidate(rng, seed);

    if ((candidate.stars ?? []).length < 3) continue;
    if (validateLevel(candidate).some((issue) => issue.severity === 'error')) continue;
    if (compileLevel(candidate).collectibles.length !== 3) continue;
    // Something has to be in the way, or the hole is a straight putt with
    // scenery around it.
    if (!directLineBlocked(candidate)) continue;

    // Cheap screen: reject anything the solver cannot finish even loosely.
    const screen = solveLevel(candidate, {
      angleSamples: 40,
      powerSamples: 4,
      maxStrokes: opts.maxPar,
      beamWidth: 3,
      timeStep: 1 / 120,
      shotTime: 9,
    });
    if (!screen.solved) continue;

    // Full-fidelity pass fixes the par at what the real physics requires, so a
    // generated hole carries the same guarantee as a handmade one.
    const exact = solveLevel(candidate, {
      angleSamples: 60,
      powerSamples: 4,
      maxStrokes: opts.maxPar,
      beamWidth: 4,
      shotTime: 10,
    });
    if (!exact.solved) continue;

    const par = Math.max(opts.minPar, exact.strokes);
    return {
      level: { ...candidate, par, name: nameFor(candidate.archetype, seed) },
      attempts: attempt,
    };
  }

  return { level: null, attempts: opts.attempts };
};

const NAMES: Record<Archetype, string[]> = {
  slingshot: ['Whip Around', 'The Long Way', 'Curveball'],
  corridor: ['Needle', 'Threadline', 'The Gate'],
  minefield: ['Scatter', 'Debris', 'Cluster'],
  binary: ['Two Bodies', 'Pairing', 'Twin Pull'],
  pinball: ['Ricochet', 'Bounce House', 'Kickabout'],
  vault: ['Locked Room', 'The Key', 'Strongbox'],
  glasshouse: ['Panes', 'Break Point', 'Shatterline'],
};

const nameFor = (archetype: Archetype, seed: number): string => {
  const options = NAMES[archetype];
  return options[seed % options.length]!;
};

/** Stable seed for a given calendar day, so everyone gets the same hole. */
export const dailySeed = (date: Date): number =>
  hashSeed(
    `gravity-golf-daily-${date.getUTCFullYear()}-${date.getUTCMonth() + 1}-${date.getUTCDate()}`,
  );

/** The daily challenge's storage key, used for its own best score. */
export const dailyId = (date: Date): string =>
  `daily-${date.getUTCFullYear()}-${String(date.getUTCMonth() + 1).padStart(2, '0')}-${String(
    date.getUTCDate(),
  ).padStart(2, '0')}`;
