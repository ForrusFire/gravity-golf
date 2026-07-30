import type { Vec2 } from '../core/vec2';

/* ------------------------------------------------------------------ shapes */

export interface CircleShape {
  kind: 'circle';
  center: Vec2;
  radius: number;
}

/**
 * A thick line segment (capsule). Used for walls, bars and paddles — it has no
 * corners to snag on, which keeps bounces predictable.
 */
export interface CapsuleShape {
  kind: 'capsule';
  a: Vec2;
  b: Vec2;
  radius: number;
}

/** A convex polygon given by vertices in local space, in any winding order. */
export interface PolygonShape {
  kind: 'polygon';
  center: Vec2;
  vertices: readonly Vec2[];
  /** Radians. Applied about `center`. */
  rotation: number;
}

export type Shape = CircleShape | CapsuleShape | PolygonShape;

/* --------------------------------------------------------------- materials */

/** Named surface presets. Levels reference these; `MATERIALS` holds the values. */
export type MaterialId = 'rock' | 'bouncy' | 'ice' | 'sand' | 'sticky' | 'metal' | 'lava' | 'void';

export interface Material {
  id: MaterialId;
  /** 0 = no bounce, 1 = perfectly elastic. Values > 1 add energy (bumpers). */
  restitution: number;
  /** Tangential velocity retained per impact, 0 = full stop, 1 = frictionless. */
  tangentRetention: number;
  /** Extra rolling drag applied while resting on this surface, per second. */
  rollingDrag: number;
  /** Contact with this surface destroys the ball. */
  deadly: boolean;
}

export const MATERIALS: Record<MaterialId, Material> = {
  rock: { id: 'rock', restitution: 0.42, tangentRetention: 0.82, rollingDrag: 1.6, deadly: false },
  bouncy: { id: 'bouncy', restitution: 1.06, tangentRetention: 0.96, rollingDrag: 0.4, deadly: false },
  ice: { id: 'ice', restitution: 0.55, tangentRetention: 0.995, rollingDrag: 0.08, deadly: false },
  sand: { id: 'sand', restitution: 0.08, tangentRetention: 0.42, rollingDrag: 5.5, deadly: false },
  sticky: { id: 'sticky', restitution: 0.0, tangentRetention: 0.06, rollingDrag: 14, deadly: false },
  metal: { id: 'metal', restitution: 0.62, tangentRetention: 0.9, rollingDrag: 1.0, deadly: false },
  lava: { id: 'lava', restitution: 0.3, tangentRetention: 0.7, rollingDrag: 2, deadly: true },
  void: { id: 'void', restitution: 0, tangentRetention: 0, rollingDrag: 0, deadly: true },
};

/* ----------------------------------------------------------------- gravity */

export interface GravitySpec {
  /** Acceleration in units/s² at the body's surface. Negative values repel. */
  strength: number;
  /**
   * Distance beyond which the body exerts no pull. `0` means unlimited.
   * Finite ranges make levels readable — players can see where a pull ends.
   */
  range: number;
}

/* ---------------------------------------------------------------- movement */

export type MotionSpec =
  | {
      kind: 'orbit';
      center: Vec2;
      radius: number;
      /** Radians per second. Negative reverses direction. */
      speed: number;
      /** Starting angle in radians. */
      phase: number;
    }
  | {
      kind: 'oscillate';
      /** Travel is `from` -> `to` -> `from`, eased sinusoidally. */
      from: Vec2;
      to: Vec2;
      /** Seconds for a full there-and-back cycle. */
      period: number;
      /** 0..1 offset into the cycle. */
      phase: number;
    }
  | {
      kind: 'spin';
      /** Radians per second about the body's own centre. */
      speed: number;
      phase: number;
    };

/* ------------------------------------------------------------------ bodies */

/** Anything the ball can hit and/or be pulled by. */
export interface Body {
  id: string;
  shape: Shape;
  material: MaterialId;
  gravity?: GravitySpec;
  motion?: MotionSpec;
  /**
   * Present until the named switch is thrown — a barrier that opens.
   * Physics and rendering both ignore it once the switch is on.
   */
  removedBy?: string | string[];
  /**
   * Absent until the named switch is thrown — a bridge that appears. Several
   * ids mean all of them, so a hole can demand a sequence rather than a single
   * pass.
   */
  addedBy?: string | string[];
  /**
   * A one-way membrane: the ball is only stopped when travelling *against* this
   * direction. Lets a hole hand out a route you cannot take back.
   */
  oneWay?: Vec2;
  /**
   * Impacts this body survives before it shatters. Gives a hole destructible
   * geometry, so the course can change shape as it is played.
   */
  hitsToBreak?: number;
  /**
   * Presentation hint only — physics never reads it.
   * Lets the renderer distinguish a planet from a wall from a bumper.
   */
  style?: BodyStyle;
}

export type BodyStyle =
  | 'planet'
  | 'crystal'
  | 'moon'
  | 'wall'
  | 'bumper'
  | 'asteroid'
  | 'sun'
  | 'blackhole'
  | 'ice'
  | 'sand'
  | 'goo';

/* ------------------------------------------------------------------- zones */

/** Non-colliding areas of effect. */
export type Zone =
  | { id: string; kind: 'wind'; area: Shape; force: Vec2 }
  | { id: string; kind: 'boost'; area: Shape; force: Vec2; }
  /** Radial push (positive) or pull (negative) about the area's centre. */
  | { id: string; kind: 'vortex'; area: Shape; strength: number; swirl: number }
  /** Multiplies velocity by `drag` per second — nebula / slow field. */
  | { id: string; kind: 'nebula'; area: Shape; drag: number }
  /** Instant loss on entry. */
  | { id: string; kind: 'hazard'; area: Shape }
  /** Scales all gravity felt inside — for null zones and gravity amplifiers. */
  | { id: string; kind: 'gravityScale'; area: Shape; scale: number };

/* ---------------------------------------------------------------- switches */

/**
 * A pad the ball throws by passing through it. Bodies referencing its id are
 * added or removed when it fires — the only mechanic that lets a shot change
 * the shape of the course.
 */
export interface SwitchSpec {
  id: string;
  position: Vec2;
  radius: number;
  /** Once thrown it stays thrown. Otherwise passing through toggles it. */
  once: boolean;
  /**
   * Seconds the switch stays on before springing back. Absent means forever.
   * A held switch turns a route into a route *and* a schedule: the gate is only
   * open while the ball is still travelling.
   */
  holdTime?: number;
}

/* ----------------------------------------------------------------- portals */

export interface Portal {
  id: string;
  /** Entry disc. */
  from: Vec2;
  /** Exit disc. */
  to: Vec2;
  radius: number;
  /** Exit velocity is rotated by this many radians. */
  twist: number;
  /** Exit speed multiplier. */
  boost: number;
  /** When true the pairing works in both directions. */
  bidirectional: boolean;
}

/**
 * A ring that fires the ball out along a fixed direction at a fixed speed.
 *
 * Deliberately absolute rather than additive: entry speed and angle are
 * irrelevant, so a booster is a promise the player can plan around — "get into
 * that ring however you like and you leave *there*, at *that* speed".
 */
export interface Booster {
  id: string;
  position: Vec2;
  radius: number;
  /** Unit vector the ball leaves along. */
  direction: Vec2;
  /** Exit speed in units per second. */
  speed: number;
}

/* -------------------------------------------------------------------- ball */

export interface Ball {
  position: Vec2;
  velocity: Vec2;
  radius: number;
  /** Seconds the ball has been below the rest threshold while touching a surface. */
  restTimer: number;
  /** True once the ball has settled and a new shot may be taken. */
  atRest: boolean;
}

/* ------------------------------------------------------------------ tuning */

export interface PhysicsConfig {
  /** Fixed simulation step in seconds. */
  timeStep: number;
  /** Velocity lost per second to "space dust" — keeps orbits from lasting forever. */
  ambientDrag: number;
  /** Below this speed a contacting ball starts settling. */
  restSpeed: number;
  /** Seconds below `restSpeed` before the ball is declared at rest. */
  restDelay: number;
  /** Hard cap on speed, to keep collisions solvable. */
  maxSpeed: number;
  /** Extra separation applied on contact to stop re-penetration jitter. */
  contactSlop: number;
  /** Gravity is softened within this fraction of a body's radius. */
  gravitySoftening: number;
}

export const DEFAULT_PHYSICS: PhysicsConfig = {
  timeStep: 1 / 240,
  ambientDrag: 0.05,
  restSpeed: 7,
  restDelay: 0.35,
  maxSpeed: 2400,
  contactSlop: 0.05,
  gravitySoftening: 0.35,
};
