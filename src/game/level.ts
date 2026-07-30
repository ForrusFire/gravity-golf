import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import {
  bodyShapeAt,
  closestSurfacePoint,
  containsPoint,
  shapeCenter,
  type Aabb,
} from '../physics/geometry';
import { DEFAULT_PHYSICS, type Body, type Portal, type SwitchSpec, type Zone } from '../physics/types';
import { createWorld, type BoundsMode, type World } from '../physics/world';

/** Authored, serialisable description of a hole. Compiled into a `World`. */
export interface LevelDef {
  id: string;
  name: string;
  /** Chapter index, 0-based. Groups levels on the map screen. */
  chapter: number;
  /** Expected stroke count. Beating it earns a better medal. */
  par: number;
  tee: Vec2;
  hole: {
    position: Vec2;
    radius?: number;
    captureSpeed?: number;
    /** Stars needed in hand before the cup will accept the ball. */
    requiresStars?: number;
  };
  bounds: Aabb;
  boundsMode?: BoundsMode;
  bodies?: Body[];
  zones?: Zone[];
  portals?: Portal[];
  switches?: SwitchSpec[];
  /** Optional bonus pickups, at most three per hole. */
  stars?: Vec2[];
  /** One-line tip shown the first time the hole is played. */
  hint?: string;
  /** Overrides the ambient "space dust" drag for this hole. */
  ambientDrag?: number;
  /**
   * A constant acceleration applied everywhere on the hole. Use this for a
   * plain "down" instead of parking a huge planet off-screen.
   */
  uniformGravity?: Vec2;
  /** Maximum launch speed in units/s. */
  maxPower?: number;
  /** Seconds a shot may run before it is abandoned and replayed. */
  shotTimeout?: number;
}

export const DEFAULT_HOLE_RADIUS = 15;
export const DEFAULT_CAPTURE_SPEED = 320;
export const DEFAULT_MAX_POWER = 900;
export const DEFAULT_SHOT_TIMEOUT = 22;
export const BALL_RADIUS = 7;

export const levelMaxPower = (level: LevelDef): number => level.maxPower ?? DEFAULT_MAX_POWER;
export const levelShotTimeout = (level: LevelDef): number =>
  level.shotTimeout ?? DEFAULT_SHOT_TIMEOUT;
export const levelHoleRadius = (level: LevelDef): number =>
  level.hole.radius ?? DEFAULT_HOLE_RADIUS;

/** Builds a fresh, mutable `World` from a level definition. */
export const compileLevel = (level: LevelDef): World =>
  createWorld({
    bodies: (level.bodies ?? []).map((b) => ({ ...b })),
    zones: (level.zones ?? []).map((z) => ({ ...z })),
    portals: (level.portals ?? []).map((p) => ({ ...p })),
    switches: (level.switches ?? []).map((sw) => ({ ...sw, on: false })),
    breakables: Object.fromEntries(
      (level.bodies ?? [])
        .filter((b) => b.hitsToBreak !== undefined)
        .map((b) => [b.id, b.hitsToBreak as number]),
    ),
    collectibles: (level.stars ?? []).map((position, i) => ({
      id: `${level.id}-star-${i}`,
      position,
      radius: 13,
      collected: false,
    })),
    hole: {
      position: level.hole.position,
      radius: levelHoleRadius(level),
      captureSpeed: level.hole.captureSpeed ?? DEFAULT_CAPTURE_SPEED,
      requiresStars: level.hole.requiresStars ?? 0,
    },
    bounds: level.bounds,
    boundsMode: level.boundsMode ?? 'kill',
    uniformGravity: level.uniformGravity ?? V.ZERO,
    config: { ...DEFAULT_PHYSICS, ambientDrag: level.ambientDrag ?? DEFAULT_PHYSICS.ambientDrag },
    time: 0,
  });

/* -------------------------------------------------------------- validation */

export interface ValidationIssue {
  levelId: string;
  severity: 'error' | 'warning';
  message: string;
}

const inBounds = (b: Aabb, p: Vec2, margin = 0): boolean =>
  p.x >= b.minX + margin &&
  p.x <= b.maxX - margin &&
  p.y >= b.minY + margin &&
  p.y <= b.maxY - margin;

/**
 * Static checks that catch the level-authoring mistakes which are invisible
 * until someone plays the hole: a tee buried in a planet, a hole nobody can
 * reach, a star outside the play area.
 */
export const validateLevel = (level: LevelDef): ValidationIssue[] => {
  const issues: ValidationIssue[] = [];
  const err = (message: string) => issues.push({ levelId: level.id, severity: 'error', message });
  const warn = (message: string) =>
    issues.push({ levelId: level.id, severity: 'warning', message });

  if (!level.id) err('missing id');
  if (!level.name) err('missing name');
  if (level.par < 1) err(`par must be at least 1 (got ${level.par})`);
  if (level.par > 12) warn(`par of ${level.par} is unusually high`);

  const b = level.bounds;
  if (b.minX >= b.maxX || b.minY >= b.maxY) err('bounds are inverted or empty');
  if (!inBounds(b, level.tee)) err('tee is outside the level bounds');
  if (!inBounds(b, level.hole.position)) err('hole is outside the level bounds');

  const holeRadius = levelHoleRadius(level);
  if (holeRadius < BALL_RADIUS + 2) err(`hole radius ${holeRadius} is too small for the ball`);

  if (V.distance(level.tee, level.hole.position) < holeRadius * 3) {
    warn('tee is almost on top of the hole');
  }

  // The tee needs room for the whole ball; the hole only needs to not be
  // buried, since a hole sitting flush on the ground is exactly right.
  const clearance = (p: Vec2, what: string, minDistance: number): void => {
    for (const body of level.bodies ?? []) {
      const shape = bodyShapeAt(body, 0);
      const q = closestSurfacePoint(shape, p);
      if (q.distance < minDistance) {
        err(`${what} overlaps body "${body.id}"`);
      }
    }
    for (const zone of level.zones ?? []) {
      if (zone.kind === 'hazard' && containsPoint(zone.area, p)) {
        err(`${what} sits inside hazard zone "${zone.id}"`);
      }
    }
  };
  clearance(level.tee, 'tee', BALL_RADIUS);
  clearance(level.hole.position, 'hole', 0);

  // A hole further above the ground than the ball's own radius can still be
  // legitimate in open space, but buried under a surface never is.
  for (const body of level.bodies ?? []) {
    if (closestSurfacePoint(bodyShapeAt(body, 0), level.hole.position).distance < 0) {
      err(`hole is buried inside body "${body.id}"`);
    }
  }

  const stars = level.stars ?? [];
  if (stars.length > 3) err(`too many stars (${stars.length}), maximum is 3`);
  stars.forEach((s, i) => {
    if (!inBounds(b, s)) err(`star ${i} is outside the level bounds`);
    for (const body of level.bodies ?? []) {
      if (closestSurfacePoint(bodyShapeAt(body, 0), s).distance < 0) {
        err(`star ${i} is buried inside body "${body.id}"`);
      }
    }
    for (let j = i + 1; j < stars.length; j++) {
      if (V.distance(s, stars[j]!) < 20) warn(`stars ${i} and ${j} nearly overlap`);
    }
  });

  const ids = new Set<string>();
  for (const body of level.bodies ?? []) {
    if (ids.has(body.id)) err(`duplicate body id "${body.id}"`);
    ids.add(body.id);
  }
  for (const zone of level.zones ?? []) {
    if (ids.has(zone.id)) err(`duplicate zone id "${zone.id}"`);
    ids.add(zone.id);
  }
  for (const pad of level.switches ?? []) {
    if (ids.has(pad.id)) err(`duplicate switch id "${pad.id}"`);
    ids.add(pad.id);
    if (!inBounds(b, pad.position)) err(`switch "${pad.id}" is outside the level bounds`);
    if (pad.radius < BALL_RADIUS) warn(`switch "${pad.id}" is narrower than the ball`);
  }

  // A gate keyed to a switch that does not exist can never open, which would
  // silently make the hole unfinishable.
  const switchIds = new Set((level.switches ?? []).map((pad) => pad.id));
  for (const body of level.bodies ?? []) {
    const gatedBy = (ids: string | string[] | undefined): string[] =>
      ids === undefined ? [] : typeof ids === 'string' ? [ids] : ids;
    for (const id of gatedBy(body.removedBy)) {
      if (!switchIds.has(id)) err(`body "${body.id}" is removed by unknown switch "${id}"`);
    }
    for (const id of gatedBy(body.addedBy)) {
      if (!switchIds.has(id)) err(`body "${body.id}" is added by unknown switch "${id}"`);
    }
    if (Array.isArray(body.addedBy) && body.addedBy.length === 0) {
      err(`body "${body.id}" is added by an empty switch list, so it can never appear`);
    }
    if (body.hitsToBreak !== undefined && body.hitsToBreak < 1) {
      err(`body "${body.id}" has hitsToBreak below one`);
    }
  }

  const required = level.hole.requiresStars ?? 0;
  if (required > (level.stars ?? []).length) {
    err(`hole needs ${required} stars but the level only has ${(level.stars ?? []).length}`);
  }

  for (const portal of level.portals ?? []) {
    if (ids.has(portal.id)) err(`duplicate portal id "${portal.id}"`);
    ids.add(portal.id);
    if (portal.radius < BALL_RADIUS) warn(`portal "${portal.id}" is narrower than the ball`);
    if (!inBounds(b, portal.from) || !inBounds(b, portal.to)) {
      err(`portal "${portal.id}" has a mouth outside the level bounds`);
    }
  }

  // A hole with no gravity anywhere and no walls is a straight-line putt.
  const hasGravity =
    (level.bodies ?? []).some((body) => body.gravity && body.gravity.strength !== 0) ||
    (level.uniformGravity !== undefined && V.lengthSq(level.uniformGravity) > 0);
  if (!hasGravity && (level.zones ?? []).length === 0 && level.chapter > 0) {
    warn('no gravity sources or zones — the hole may be trivial');
  }

  return issues;
};

/** Convenience wrapper used by tests and the level editor. */
export const validateLevels = (levels: readonly LevelDef[]): ValidationIssue[] => {
  const issues: ValidationIssue[] = [];
  const seen = new Set<string>();
  for (const level of levels) {
    if (seen.has(level.id)) {
      issues.push({ levelId: level.id, severity: 'error', message: 'duplicate level id' });
    }
    seen.add(level.id);
    issues.push(...validateLevel(level));
  }
  return issues;
};

/* ----------------------------------------------------------- authoring DSL */

/** Terse constructors so level data reads like a description of the hole. */
export const planet = (
  id: string,
  center: Vec2,
  radius: number,
  gravity: number,
  opts: Partial<Body> & { range?: number } = {},
): Body => {
  const { range, ...rest } = opts;
  return {
    id,
    shape: { kind: 'circle', center, radius },
    material: 'rock',
    style: 'planet',
    gravity: { strength: gravity, range: range ?? 0 },
    ...rest,
  };
};

export const rock = (id: string, center: Vec2, radius: number, opts: Partial<Body> = {}): Body => ({
  id,
  shape: { kind: 'circle', center, radius },
  material: 'rock',
  style: 'asteroid',
  ...opts,
});

export const wall = (
  id: string,
  a: Vec2,
  b: Vec2,
  thickness = 8,
  opts: Partial<Body> = {},
): Body => ({
  id,
  shape: { kind: 'capsule', a, b, radius: thickness },
  material: 'rock',
  style: 'wall',
  ...opts,
});

export const bumper = (id: string, center: Vec2, radius: number, opts: Partial<Body> = {}): Body => ({
  id,
  shape: { kind: 'circle', center, radius },
  material: 'bouncy',
  style: 'bumper',
  ...opts,
});

export const sun = (
  id: string,
  center: Vec2,
  radius: number,
  gravity: number,
  opts: Partial<Body> & { range?: number } = {},
): Body => {
  const { range, ...rest } = opts;
  return {
    id,
    shape: { kind: 'circle', center, radius },
    material: 'lava',
    style: 'sun',
    gravity: { strength: gravity, range: range ?? 0 },
    ...rest,
  };
};

export const blackHole = (
  id: string,
  center: Vec2,
  radius: number,
  gravity: number,
  opts: Partial<Body> & { range?: number } = {},
): Body => {
  const { range, ...rest } = opts;
  return {
    id,
    shape: { kind: 'circle', center, radius },
    material: 'void',
    style: 'blackhole',
    gravity: { strength: gravity, range: range ?? 0 },
    ...rest,
  };
};

/**
 * The launch velocity for a shot. Both the game and the solver must derive it
 * the same way, down to the last bit: the solver's guarantee is that its exact
 * shot list replays, and gravity slingshots amplify a 1e-16 difference in the
 * impulse into metres of divergence within two shots.
 */
export const shotImpulse = (direction: Vec2, power: number, maxPower: number): Vec2 => {
  const dir = V.normalize(direction);
  const clamped = power < 0 ? 0 : power > 1 ? 1 : power;
  return V.mul(dir, clamped * maxPower);
};

/** A switch pad the ball throws by passing through it. */
export const switchPad = (
  id: string,
  position: Vec2,
  radius = 26,
  once = true,
): SwitchSpec => ({ id, position, radius, once });

/**
 * A pad that springs back after `holdTime` seconds. Whatever it controls is only
 * open while the ball is still in flight, which makes the route a schedule.
 */
export const timedPad = (
  id: string,
  position: Vec2,
  radius: number,
  holdTime: number,
): SwitchSpec => ({ id, position, radius, once: true, holdTime });

/** A barrier that vanishes while every named switch is on. */
export const gate = (
  id: string,
  a: Vec2,
  b: Vec2,
  switchId: string | string[],
  thickness = 12,
): Body => ({
  id,
  shape: { kind: 'capsule', a, b, radius: thickness },
  material: 'metal',
  style: 'wall',
  removedBy: switchId,
});

/** A bridge that appears once every named switch is on. */
export const bridge = (
  id: string,
  a: Vec2,
  b: Vec2,
  switchId: string | string[],
  thickness = 12,
): Body => ({
  id,
  shape: { kind: 'capsule', a, b, radius: thickness },
  material: 'rock',
  style: 'wall',
  addedBy: switchId,
});

/**
 * A membrane the ball can only cross in the direction of `through`. Useful for
 * handing out a route that cannot be taken back.
 */
export const membrane = (
  id: string,
  a: Vec2,
  b: Vec2,
  through: Vec2,
  thickness = 10,
): Body => ({
  id,
  shape: { kind: 'capsule', a, b, radius: thickness },
  material: 'bouncy',
  style: 'ice',
  oneWay: V.normalize(through),
});

/** A block that shatters after `hits` impacts. */
export const crystal = (
  id: string,
  center: Vec2,
  radius: number,
  hits = 1,
  opts: Partial<Body> = {},
): Body => ({
  id,
  shape: { kind: 'circle', center, radius },
  material: 'metal',
  style: 'crystal',
  hitsToBreak: hits,
  ...opts,
});

/** Where a body sits at t=0, accounting for its motion. */
export const bodyOrigin = (body: Body): Vec2 => shapeCenter(bodyShapeAt(body, 0));

/**
 * A point sitting on a circular body's surface at `angle` (radians, 0 = right,
 * measured clockwise in screen space). Use this to place tees, holes and stars
 * on the ground instead of eyeballing coordinates.
 */
export const onSurface = (
  center: Vec2,
  radius: number,
  angle: number,
  gap = BALL_RADIUS + 1,
): Vec2 => ({
  x: center.x + Math.cos(angle) * (radius + gap),
  y: center.y + Math.sin(angle) * (radius + gap),
});

/** The angle from a body's centre to `p` — the inverse of `onSurface`. */
export const angleFrom = (center: Vec2, p: Vec2): number =>
  Math.atan2(p.y - center.y, p.x - center.x);
