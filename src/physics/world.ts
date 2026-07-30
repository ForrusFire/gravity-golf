import { clamp, smoothstep } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import {
  bodyShapeAt,
  bodyVelocityAt,
  closestSurfacePoint,
  containsPoint,
  motionOffset,
  shapeCenter,
  shapeRadius,
  type Aabb,
} from './geometry';
import {
  DEFAULT_PHYSICS,
  MATERIALS,
  type Ball,
  type Body,
  type MaterialId,
  type MotionSpec,
  type PhysicsConfig,
  type PulseSpec,
  type Booster,
  type Portal,
  type Shape,
  type Zone,
} from './types';

export interface Hole {
  /** Where the cup sits at t=0. With `motion` set this is derived from it. */
  position: Vec2;
  /**
   * Optional path the cup travels. Everything that reads the cup's location must
   * go through `holePositionAt`, or half the game will aim at where it started.
   */
  motion?: MotionSpec;
  radius: number;
  /** The ball only drops if it enters slower than this. Faster balls lip out. */
  captureSpeed: number;
  /**
   * Stars that must be in hand before the cup will accept the ball. Turns the
   * bonus pickups into the objective on holes that want it.
   */
  requiresStars?: number;
  /**
   * A cup that only accepts the ball arriving on a particular heading, like a
   * letterbox. Turns "get there slowly" into "get there slowly *from there*".
   */
  approach?: ApproachSpec;
}

export interface ApproachSpec {
  /** Unit vector the ball must be travelling along to drop in. */
  direction: Vec2;
  /** Half-angle of the accepted cone, in radians. */
  tolerance: number;
}

export interface Collectible {
  id: string;
  position: Vec2;
  radius: number;
  collected: boolean;
}

export type BoundsMode = 'kill' | 'wall' | 'open';

/** Runtime state of a switch pad. */
export interface SwitchState {
  id: string;
  position: Vec2;
  radius: number;
  once: boolean;
  on: boolean;
  /** Seconds it stays on once thrown. Absent means indefinitely. */
  holdTime?: number;
  /** World time at which a held switch springs back. */
  offAt?: number;
}

export interface World {
  bodies: Body[];
  zones: Zone[];
  portals: Portal[];
  switches: SwitchState[];
  boosters: Booster[];
  /**
   * Impacts left before each breakable body shatters, keyed by body id. Held
   * here rather than on the body so a preview or a search can copy the mutable
   * state cheaply and leave the level definition alone.
   */
  breakables: Record<string, number>;
  collectibles: Collectible[];
  hole: Hole;
  bounds: Aabb;
  boundsMode: BoundsMode;
  /**
   * A constant acceleration applied everywhere, for holes that just want a
   * "down". Cheaper and more honest than parking an enormous planet off-screen,
   * and unlike a body it is unaffected by distance.
   */
  uniformGravity: Vec2;
  config: PhysicsConfig;
  /** Simulation clock in seconds; drives every moving body. */
  time: number;
  /** Internal per-instant shape cache. Never write this from outside. */
  shapeCache?: ShapeCache;
  /** Bumped when switch or breakable state changes; invalidates the cache. */
  revision?: number;
}

/** Where the cup is at time `t`. Static holes ignore `t` entirely. */
export const holePositionAt = (world: World, t = world.time): Vec2 =>
  world.hole.motion ? motionOffset(world.hole.motion, t) : world.hole.position;

/** How fast the cup itself is travelling at time `t`. Zero for a static hole. */
export const holeVelocityAt = (world: World, t = world.time): Vec2 => {
  const motion = world.hole.motion;
  if (!motion || motion.kind === 'spin') return V.ZERO;
  const h = 1 / 600;
  const before = motionOffset(motion, t - h);
  const after = motionOffset(motion, t + h);
  return { x: (after.x - before.x) / (2 * h), y: (after.y - before.y) / (2 * h) };
};

/**
 * Whether a body currently exists. Gated bodies let a switch open a barrier or
 * drop a bridge; a shattered breakable is gone for the rest of the hole.
 */
export const isBodyActive = (world: World, body: Body, t = world.time): boolean => {
  if (body.hitsToBreak !== undefined && (world.breakables[body.id] ?? 0) <= 0) return false;
  // All named switches must be on: a body can demand a sequence, not just a pass.
  if (body.removedBy !== undefined && allSwitchesOn(world, body.removedBy)) return false;
  if (body.addedBy !== undefined && !allSwitchesOn(world, body.addedBy)) return false;
  if (body.pulse && !pulseOn(body.pulse, t)) return false;
  return true;
};

/** Where a pulsing body is in its cycle: 0 just appeared, 1 about to change. */
export const pulsePhase = (pulse: PulseSpec, t: number): number => {
  if (pulse.period <= 0) return 0;
  const cycle = (((t / pulse.period + pulse.phase) % 1) + 1) % 1;
  const duty = clamp(pulse.duty, 0, 1);
  return cycle < duty
    ? (duty > 0 ? cycle / duty : 1)
    : (duty < 1 ? (cycle - duty) / (1 - duty) : 1);
};

/** True while a pulsing body exists. */
export const pulseOn = (pulse: PulseSpec, t: number): boolean => {
  if (pulse.period <= 0) return true;
  const cycle = (((t / pulse.period + pulse.phase) % 1) + 1) % 1;
  return cycle < clamp(pulse.duty, 0, 1);
};

const allSwitchesOn = (world: World, ids: string | string[]): boolean => {
  if (typeof ids === 'string') return world.switches.some((s) => s.id === ids && s.on);
  return ids.every((id) => world.switches.some((s) => s.id === id && s.on));
};

interface ResolvedBody {
  body: Body;
  shape: Shape;
  center: Vec2;
  radius: number;
}

interface ShapeCache {
  time: number;
  /** Identity check, so a copied world with different bodies rebuilds. */
  source: Body[];
  /**
   * Bumped whenever a switch fires or a breakable shatters, so the cached set
   * of active bodies cannot outlive the state it was built from.
   */
  revision: number;
  items: ResolvedBody[];
}

/**
 * Bodies resolved to their world-space shapes at time `t`.
 *
 * Gravity, collision and surface-velocity queries all want the same answer
 * many times per instant — the field overlay alone asks hundreds of times per
 * frame. Resolving once per instant turns that into a single pass.
 */
const resolveBodies = (world: World, t: number): ResolvedBody[] => {
  const cache = world.shapeCache;
  const revision = world.revision ?? 0;
  if (cache && cache.time === t && cache.source === world.bodies && cache.revision === revision) {
    return cache.items;
  }

  const items: ResolvedBody[] = [];
  for (const body of world.bodies) {
    if (!isBodyActive(world, body, t)) continue;
    const shape = bodyShapeAt(body, t);
    items.push({ body, shape, center: shapeCenter(shape), radius: shapeRadius(shape) });
  }
  world.shapeCache = { time: t, source: world.bodies, revision, items };
  return items;
};

export type DeathCause = 'hazard' | 'out-of-bounds' | 'crushed';

export type SimEvent =
  | { type: 'bounce'; point: Vec2; normal: Vec2; speed: number; material: MaterialId; bodyId: string }
  | { type: 'collect'; id: string; position: Vec2 }
  | { type: 'sink'; position: Vec2; speed: number }
  | { type: 'lipout'; position: Vec2; speed: number }
  | { type: 'death'; position: Vec2; cause: DeathCause }
  | { type: 'portal'; from: Vec2; to: Vec2; portalId: string }
  | { type: 'switch'; id: string; position: Vec2; on: boolean }
  | { type: 'boost'; id: string; position: Vec2; direction: Vec2; speed: number }
  | { type: 'shatter'; bodyId: string; position: Vec2 }
  | { type: 'locked'; position: Vec2; needed: number }
  | { type: 'rest'; position: Vec2 };

export const createBall = (position: Vec2, radius = 7): Ball => ({
  position,
  velocity: V.ZERO,
  radius,
  restTimer: 0,
  atRest: true,
});

/** Extra per-ball simulation state the physics owns but the game reads. */
export interface BallRuntime {
  portalCooldown: number;
  alive: boolean;
  sunk: boolean;
  /** Material of the last surface touched — drives footstep-style audio and VFX. */
  lastMaterial: MaterialId | null;
  /** Seconds since the ball was last in contact with any surface. */
  airTime: number;
  /** Peak speed reached since the last shot, for scoring flourishes. */
  peakSpeed: number;
  /** Distance travelled since the last shot. */
  distance: number;
  /** Switch ids the ball is currently overlapping, so each pass fires once. */
  insideSwitches: Set<string>;
  /** True while sitting in a cup that refused the ball, so it complains once. */
  inLockedCup: boolean;
  /** Booster ids the ball is currently inside, so each pass fires once. */
  insideBoosters: Set<string>;
}

export const createBallRuntime = (): BallRuntime => ({
  portalCooldown: 0,
  alive: true,
  sunk: false,
  lastMaterial: null,
  airTime: 0,
  peakSpeed: 0,
  distance: 0,
  insideSwitches: new Set(),
  inLockedCup: false,
  insideBoosters: new Set(),
});

export const createWorld = (init: Partial<World> & Pick<World, 'hole' | 'bounds'>): World => ({
  bodies: [],
  zones: [],
  portals: [],
  collectibles: [],
  switches: [],
  boosters: [],
  breakables: {},
  boundsMode: 'kill',
  uniformGravity: V.ZERO,
  config: DEFAULT_PHYSICS,
  time: 0,
  ...init,
});

/* ----------------------------------------------------------------- gravity */

/** Multiplier applied to gravity at `p` by any overlapping gravityScale zone. */
export const gravityScaleAt = (world: World, p: Vec2): number => {
  let scale = 1;
  for (const zone of world.zones) {
    if (zone.kind !== 'gravityScale') continue;
    if (containsPoint(zone.area, p)) scale *= zone.scale;
  }
  return scale;
};

/**
 * Gravitational acceleration at `p`. `strength` is surface gravity, so a body's
 * pull is `strength` at its own radius and falls off with the inverse square.
 */
export const gravityAt = (world: World, p: Vec2, t = world.time): Vec2 => {
  let ax = world.uniformGravity.x;
  let ay = world.uniformGravity.y;
  const soft = world.config.gravitySoftening;

  for (const resolved of resolveBodies(world, t)) {
    const g = resolved.body.gravity;
    if (!g || g.strength === 0) continue;
    const center = resolved.center;
    const dx = center.x - p.x;
    const dy = center.y - p.y;
    const distSq = dx * dx + dy * dy;
    if (distSq < 1e-9) continue;
    const dist = Math.sqrt(distSq);
    if (g.range > 0 && dist > g.range) continue;

    const r = Math.max(resolved.radius, 1);
    // Softened denominator: keeps the pull finite as the ball approaches the core.
    const denom = Math.max(dist, r * soft);
    let mag = (g.strength * r * r) / (denom * denom);

    // Fade out over the final 25% of the range so the edge is not a hard cliff.
    if (g.range > 0) {
      const fadeStart = g.range * 0.75;
      if (dist > fadeStart) mag *= 1 - smoothstep((dist - fadeStart) / (g.range - fadeStart));
    }

    ax += (dx / dist) * mag;
    ay += (dy / dist) * mag;
  }

  const scale = gravityScaleAt(world, p);
  return { x: ax * scale, y: ay * scale };
};

/** Total acceleration: gravity plus every force zone acting at `p`. */
export const accelerationAt = (world: World, p: Vec2, t = world.time): Vec2 => {
  const g = gravityAt(world, p, t);
  let ax = g.x;
  let ay = g.y;

  for (const zone of world.zones) {
    switch (zone.kind) {
      case 'wind':
      case 'boost': {
        if (!containsPoint(zone.area, p)) continue;
        ax += zone.force.x;
        ay += zone.force.y;
        break;
      }
      case 'vortex': {
        if (!containsPoint(zone.area, p)) continue;
        const c = shapeCenter(zone.area);
        const dx = p.x - c.x;
        const dy = p.y - c.y;
        const dist = Math.hypot(dx, dy);
        if (dist < 1e-6) continue;
        const nx = dx / dist;
        const ny = dy / dist;
        ax += nx * zone.strength - ny * zone.swirl;
        ay += ny * zone.strength + nx * zone.swirl;
        break;
      }
      default:
        break;
    }
  }

  return { x: ax, y: ay };
};

/** Combined per-second velocity damping at `p` (ambient plus nebula zones). */
const dragAt = (world: World, p: Vec2): number => {
  let drag = world.config.ambientDrag;
  for (const zone of world.zones) {
    if (zone.kind !== 'nebula') continue;
    if (containsPoint(zone.area, p)) drag += zone.drag;
  }
  return drag;
};

/* --------------------------------------------------------------- collision */

interface Contact {
  body: Body;
  query: ReturnType<typeof closestSurfacePoint>;
  penetration: number;
}

/**
 * Bodies within this distance of the ball count as touching even without
 * overlap. Without a skin, a ball settling on a surface alternates between
 * "pushed out" and "falling" every step and never registers as supported.
 */
const CONTACT_SKIN = 0.75;

/** Speed kept when a ball smashes through the last hit of a breakable block. */
const BREAKTHROUGH_RETENTION = 0.72;

const findDeepestContact = (
  world: World,
  ball: Ball,
  t: number,
  skin = CONTACT_SKIN,
): Contact | null => {
  let best: Contact | null = null;
  const reach = ball.radius + skin;
  for (const resolved of resolveBodies(world, t)) {
    // Cheap reject on the bounding circle before the exact surface query.
    const dx = resolved.center.x - ball.position.x;
    const dy = resolved.center.y - ball.position.y;
    const far = resolved.radius + reach;
    if (dx * dx + dy * dy > far * far) continue;

    // A one-way membrane exists only for a ball travelling against it. A ball
    // at rest on one has a zero dot product, so it still gets held up.
    const oneWay = resolved.body.oneWay;
    if (oneWay && V.dot(ball.velocity, oneWay) > 0) continue;

    const query = closestSurfacePoint(resolved.shape, ball.position);
    if (query.distance >= reach) continue;
    const penetration = ball.radius - query.distance;
    if (!best || penetration > best.penetration) best = { body: resolved.body, query, penetration };
  }
  return best;
};

/**
 * Resolves contact with the deepest overlapping body, then re-checks a few
 * times so wedges between two bodies settle instead of jittering.
 */
interface CollisionResult {
  touched: boolean;
  killed: boolean;
  /** Material of the surface touched this step, if any. */
  material: MaterialId | null;
}

const resolveCollisions = (
  world: World,
  ball: Ball,
  t: number,
  events: SimEvent[],
): CollisionResult => {
  let touched = false;
  let material: MaterialId | null = null;

  for (let iteration = 0; iteration < 4; iteration++) {
    const contact = findDeepestContact(world, ball, t);
    if (!contact) break;
    // Only the first pass reports contact; later passes exist to untangle
    // wedges between two bodies.
    touched = true;
    material = contact.body.material;

    const surface = MATERIALS[contact.body.material];
    const normal = contact.query.normal;

    if (surface.deadly) {
      events.push({ type: 'death', position: ball.position, cause: 'hazard' });
      return { touched: true, killed: true, material };
    }

    // Push out of the surface — only when genuinely overlapping, so a ball
    // resting inside the contact skin is not shoved away every step.
    if (contact.penetration > 0) {
      const push = contact.penetration + world.config.contactSlop;
      ball.position = {
        x: ball.position.x + normal.x * push,
        y: ball.position.y + normal.y * push,
      };
    }

    // Work in the surface's frame so moving platforms transfer momentum.
    const surfaceVel = bodyVelocityAt(contact.body, t, contact.query.point);
    const rel = V.sub(ball.velocity, surfaceVel);
    const vn = V.dot(rel, normal);

    if (vn < 0) {
      const tangent = { x: rel.x - normal.x * vn, y: rel.y - normal.y * vn };
      const impactSpeed = -vn;

      if (impactSpeed >= world.config.restSpeed) {
        if (contact.body.hitsToBreak !== undefined) {
          const left = (world.breakables[contact.body.id] ?? 0) - 1;
          world.breakables[contact.body.id] = left;
          world.revision = (world.revision ?? 0) + 1;
          if (left <= 0) {
            events.push({
              type: 'shatter',
              bodyId: contact.body.id,
              position: contact.query.point,
            });
            // The block gave way, so the ball carries on through instead of
            // bouncing off something that no longer exists — otherwise a
            // breakable wall would cost a stroke per pane just to walk through
            // the hole it made. Punching through costs speed, so a run of
            // blocks is never free.
            ball.velocity = V.mul(ball.velocity, BREAKTHROUGH_RETENTION);
            break;
          }
        }

        // A genuine impact: bounce, and lose some speed along the surface.
        const bounced = impactSpeed * surface.restitution;
        ball.velocity = V.add(
          {
            x: tangent.x * surface.tangentRetention + normal.x * bounced,
            y: tangent.y * surface.tangentRetention + normal.y * bounced,
          },
          surfaceVel,
        );
        events.push({
          type: 'bounce',
          point: contact.query.point,
          normal,
          speed: impactSpeed,
          material: surface.id,
          bodyId: contact.body.id,
        });
      } else {
        // Resting or rolling contact: gravity presses the ball into the surface
        // every step, so treating that as an impact would apply the tangential
        // loss hundreds of times a second and stop a rolling ball dead. Cancel
        // only the motion into the surface and let rollingDrag, which is
        // per-second, do the slowing.
        ball.velocity = V.add(tangent, surfaceVel);
      }
    }

    if (iteration === 0) {
      // Rolling friction: bleed off tangential speed while in contact.
      const decay = Math.exp(-surface.rollingDrag * world.config.timeStep);
      const relNow = V.sub(ball.velocity, surfaceVel);
      const vnNow = V.dot(relNow, normal);
      const tangential = { x: relNow.x - normal.x * vnNow, y: relNow.y - normal.y * vnNow };
      ball.velocity = V.add(
        { x: tangential.x * decay + normal.x * vnNow, y: tangential.y * decay + normal.y * vnNow },
        surfaceVel,
      );
    }
  }

  return { touched, killed: false, material };
};

/* ----------------------------------------------------------------- sensors */

const checkPortals = (world: World, ball: Ball, rt: BallRuntime, events: SimEvent[]): void => {
  if (rt.portalCooldown > 0) return;
  for (const portal of world.portals) {
    const entries: Array<{ from: Vec2; to: Vec2; twist: number }> = [
      { from: portal.from, to: portal.to, twist: portal.twist },
    ];
    if (portal.bidirectional) {
      entries.push({ from: portal.to, to: portal.from, twist: -portal.twist });
    }
    for (const entry of entries) {
      if (V.distanceSq(ball.position, entry.from) > portal.radius * portal.radius) continue;
      const rotated = V.rotate(ball.velocity, entry.twist);
      ball.velocity = V.mul(rotated, portal.boost);
      // Exit offset along travel direction keeps the ball clear of the mouth.
      const dir = V.normalize(ball.velocity);
      ball.position = V.addScaled(entry.to, dir, portal.radius * 0.75);
      rt.portalCooldown = 0.25;
      ball.atRest = false;
      ball.restTimer = 0;
      events.push({ type: 'portal', from: entry.from, to: entry.to, portalId: portal.id });
      return;
    }
  }
};

/**
 * Fires any booster the ball has entered, overwriting its velocity.
 *
 * Edge-triggered like a switch pad: a ball that stalls inside a ring would
 * otherwise be re-fired every step and never leave.
 */
const checkBoosters = (
  world: World,
  ball: Ball,
  runtime: BallRuntime,
  events: SimEvent[],
): void => {
  for (const booster of world.boosters) {
    const reach = booster.radius + ball.radius;
    const inside = V.distanceSq(ball.position, booster.position) <= reach * reach;
    if (!inside) {
      runtime.insideBoosters.delete(booster.id);
      continue;
    }
    if (runtime.insideBoosters.has(booster.id)) continue;

    runtime.insideBoosters.add(booster.id);
    const dir = V.normalize(booster.direction);
    ball.velocity = V.mul(dir, booster.speed);
    ball.atRest = false;
    ball.restTimer = 0;
    events.push({
      type: 'boost',
      id: booster.id,
      position: booster.position,
      direction: dir,
      speed: booster.speed,
    });
  }
};

/**
 * Fires any switch the ball is inside. A latching switch stays on; a toggle
 * flips, but only once per pass, so sitting inside one does not strobe the
 * course between states.
 */
const checkSwitches = (
  world: World,
  ball: Ball,
  runtime: BallRuntime,
  events: SimEvent[],
): void => {
  for (const pad of world.switches) {
    // A held switch springs back on its own, whether or not the ball is near it.
    if (pad.on && pad.offAt !== undefined && world.time >= pad.offAt) {
      pad.on = false;
      pad.offAt = undefined;
      world.revision = (world.revision ?? 0) + 1;
      events.push({ type: 'switch', id: pad.id, position: pad.position, on: false });
    }

    const reach = pad.radius + ball.radius;
    const inside = V.distanceSq(ball.position, pad.position) <= reach * reach;
    const wasInside = runtime.insideSwitches.has(pad.id);

    if (inside && !wasInside) {
      runtime.insideSwitches.add(pad.id);
      // A held switch can always be re-thrown; that is the point of holding it.
      if (pad.once && pad.on && pad.holdTime === undefined) continue;
      pad.on = pad.once ? true : !pad.on;
      pad.offAt = pad.on && pad.holdTime !== undefined ? world.time + pad.holdTime : undefined;
      world.revision = (world.revision ?? 0) + 1;
      events.push({ type: 'switch', id: pad.id, position: pad.position, on: pad.on });
    } else if (!inside && wasInside) {
      runtime.insideSwitches.delete(pad.id);
    }
  }
};

const checkCollectibles = (world: World, ball: Ball, events: SimEvent[]): void => {
  for (const item of world.collectibles) {
    if (item.collected) continue;
    const r = item.radius + ball.radius;
    if (V.distanceSq(ball.position, item.position) <= r * r) {
      item.collected = true;
      events.push({ type: 'collect', id: item.id, position: item.position });
    }
  }
};

const checkHazardZones = (world: World, ball: Ball, events: SimEvent[]): boolean => {
  for (const zone of world.zones) {
    if (zone.kind !== 'hazard') continue;
    if (containsPoint(zone.area, ball.position)) {
      events.push({ type: 'death', position: ball.position, cause: 'hazard' });
      return true;
    }
  }
  return false;
};

/**
 * Hole capture. Slow balls drop; fast balls are nudged by the rim and lip out,
 * which is what makes near misses feel like near misses.
 */
const checkHole = (
  world: World,
  ball: Ball,
  runtime: BallRuntime,
  dt: number,
  events: SimEvent[],
): boolean => {
  const hole = world.hole;
  const holeAt = holePositionAt(world);
  const dist = V.distance(ball.position, holeAt);
  // Speed *relative to the cup*. On a moving hole the absolute speed is the
  // wrong question: it would let a player park the ball on the track and let the
  // cup drive over and swallow it, which beats every moving hole without a plan.
  // For a static cup this is exactly the ball's own speed, as before.
  const speed = V.length(V.sub(ball.velocity, holeVelocityAt(world)));

  // A locked cup refuses the ball until enough stars are in hand.
  const required = hole.requiresStars ?? 0;
  if (required > 0) {
    const held = world.collectibles.filter((c) => c.collected).length;
    if (held < required) {
      // Edge-triggered: a ball parked in a sealed cup overlaps it for hundreds
      // of steps, and one refusal per arrival is the honest signal.
      const inside = dist <= hole.radius;
      if (inside && !runtime.inLockedCup) {
        events.push({ type: 'locked', position: holeAt, needed: required - held });
      }
      runtime.inLockedCup = inside;
      return false;
    }
  }
  if (dist > hole.radius) runtime.inLockedCup = false;

  // Computed once and used for both the funnel and the capture. A directional
  // cup that still sucks at a ball it will not accept would shake a trapped ball
  // about until its heading happened to line up, which beats the mechanic on a
  // technicality. The mouth pulls only from the side it opens onto.
  const accepted = approachAccepted(hole.approach, ball, world, speed);

  const funnelRadius = hole.radius * 2.4;
  if (accepted && dist < funnelRadius) {
    // Rim funnel: strengthens as the ball nears the centre, and fades out as
    // the ball gets faster. A slow ball that grazes the rim drops in; a fast
    // one is barely deflected and skips straight over, which is the whole
    // point of having a capture speed.
    const closeness = 1 - dist / funnelRadius;
    const slowness = clamp(1 - speed / (hole.captureSpeed * 1.6), 0, 1);
    const pull = closeness * slowness * 1400;
    if (pull > 0) {
      const dir = V.normalize(V.sub(holeAt, ball.position));
      ball.velocity = V.addScaled(ball.velocity, dir, pull * dt);
    }
  }

  if (dist <= hole.radius) {
    if (speed <= hole.captureSpeed && accepted) {
      events.push({ type: 'sink', position: holeAt, speed });
      return true;
    }
    // Edge-triggered like the sealed cup: a ball that comes to rest against a
    // directional cup overlaps it for hundreds of steps, and one rejection per
    // arrival is the honest signal rather than a stream of them.
    if (!runtime.inLockedCup) events.push({ type: 'lipout', position: ball.position, speed });
    runtime.inLockedCup = true;
  }
  return false;
};

/**
 * Whether the ball is arriving on the heading a directional cup demands.
 *
 * A ball creeping about inside the cup is not "arriving" from anywhere, so it is
 * refused rather than let in on a technicality — otherwise nudging it around the
 * rim until the direction happened to line up would beat the mechanic.
 */
const approachAccepted = (
  approach: ApproachSpec | undefined,
  ball: Ball,
  world: World,
  speed: number,
): boolean => {
  if (!approach) return true;
  if (speed < world.config.restSpeed) return false;
  const relative = V.sub(ball.velocity, holeVelocityAt(world));
  const heading = V.normalize(relative);
  const wanted = V.normalize(approach.direction);
  return V.dot(heading, wanted) >= Math.cos(approach.tolerance);
};

const checkBounds = (world: World, ball: Ball, events: SimEvent[]): boolean => {
  const b = world.bounds;
  const r = ball.radius;
  if (world.boundsMode === 'open') return false;

  if (world.boundsMode === 'wall') {
    let p = ball.position;
    let v = ball.velocity;
    const bounce = 0.55;
    if (p.x - r < b.minX) {
      p = { x: b.minX + r, y: p.y };
      if (v.x < 0) v = { x: -v.x * bounce, y: v.y };
    } else if (p.x + r > b.maxX) {
      p = { x: b.maxX - r, y: p.y };
      if (v.x > 0) v = { x: -v.x * bounce, y: v.y };
    }
    if (p.y - r < b.minY) {
      p = { x: p.x, y: b.minY + r };
      if (v.y < 0) v = { x: v.x, y: -v.y * bounce };
    } else if (p.y + r > b.maxY) {
      p = { x: p.x, y: b.maxY - r };
      if (v.y > 0) v = { x: v.x, y: -v.y * bounce };
    }
    ball.position = p;
    ball.velocity = v;
    return false;
  }

  const p = ball.position;
  if (p.x < b.minX - r || p.x > b.maxX + r || p.y < b.minY - r || p.y > b.maxY + r) {
    events.push({ type: 'death', position: p, cause: 'out-of-bounds' });
    return true;
  }
  return false;
};

/* -------------------------------------------------------------------- step */

/**
 * Advances the ball by one `config.timeStep`, subdividing internally when the
 * ball is moving fast enough to tunnel. Mutates `ball` and `runtime`; every
 * notable occurrence is appended to `events`.
 *
 * Integration is kick-drift-kick leapfrog, which conserves orbital energy far
 * better than Euler at the same step size — orbits stay closed instead of
 * spiralling out.
 */
export const stepWorld = (
  world: World,
  ball: Ball,
  runtime: BallRuntime,
  events: SimEvent[],
): void => {
  const cfg = world.config;
  const frame = cfg.timeStep;

  if (!runtime.alive || runtime.sunk) {
    world.time += frame;
    return;
  }

  const speed = V.length(ball.velocity);
  // Never let the ball travel more than 40% of its radius per substep.
  const maxTravel = ball.radius * 0.4;
  const substeps = clamp(Math.ceil((speed * frame) / Math.max(maxTravel, 0.001)), 1, 24);
  const h = frame / substeps;

  for (let i = 0; i < substeps; i++) {
    const t = world.time;

    // Kick.
    const a0 = accelerationAt(world, ball.position, t);
    let v = V.addScaled(ball.velocity, a0, h / 2);

    // Drift.
    const prevPosition = ball.position;
    ball.position = V.addScaled(ball.position, v, h);
    world.time = t + h;

    // Kick.
    const a1 = accelerationAt(world, ball.position, world.time);
    v = V.addScaled(v, a1, h / 2);

    // Damping.
    const decay = Math.exp(-dragAt(world, ball.position) * h);
    ball.velocity = V.clampLength(V.mul(v, decay), cfg.maxSpeed);

    runtime.distance += V.distance(prevPosition, ball.position);
    const currentSpeed = V.length(ball.velocity);
    if (currentSpeed > runtime.peakSpeed) runtime.peakSpeed = currentSpeed;

    const collision = resolveCollisions(world, ball, world.time, events);
    if (collision.killed) {
      runtime.alive = false;
      return;
    }
    const touched = collision.touched;

    if (touched) {
      runtime.airTime = 0;
      runtime.lastMaterial = collision.material ?? runtime.lastMaterial;
    } else {
      runtime.airTime += h;
    }

    if (runtime.portalCooldown > 0) runtime.portalCooldown = Math.max(0, runtime.portalCooldown - h);
    checkPortals(world, ball, runtime, events);
    checkBoosters(world, ball, runtime, events);
    checkSwitches(world, ball, runtime, events);
    checkCollectibles(world, ball, events);

    if (checkHazardZones(world, ball, events)) {
      runtime.alive = false;
      return;
    }
    if (checkBounds(world, ball, events)) {
      runtime.alive = false;
      return;
    }
    if (checkHole(world, ball, runtime, h, events)) {
      runtime.sunk = true;
      ball.velocity = V.ZERO;
      // Snap to where the cup is *now*, which on a moving hole is not where
      // it started.
      ball.position = holePositionAt(world);
      return;
    }

    // Settling: the ball must be slow *and* supported to count as at rest.
    const restSpeed = V.length(ball.velocity);
    if (restSpeed < cfg.restSpeed && touched) {
      ball.restTimer += h;
      if (ball.restTimer >= cfg.restDelay && !ball.atRest) {
        ball.atRest = true;
        ball.velocity = V.ZERO;
        events.push({ type: 'rest', position: ball.position });
      }
    } else {
      ball.restTimer = 0;
      if (restSpeed > cfg.restSpeed * 1.5) ball.atRest = false;
    }
  }
};

/** Launches the ball, clearing per-shot runtime stats. */
export const launchBall = (ball: Ball, runtime: BallRuntime, impulse: Vec2): void => {
  ball.velocity = impulse;
  ball.atRest = false;
  ball.restTimer = 0;
  runtime.airTime = 0;
  runtime.peakSpeed = V.length(impulse);
  runtime.distance = 0;
};
