import { TAU, angleDelta, clamp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import { bodyShapeAt, closestSurfacePoint, shapeCenter, shapeRadius } from '../physics/geometry';
import { MATERIALS } from '../physics/types';
import {
  predictTrajectory,
  type Trajectory,
  type TrajectoryOptions,
} from '../physics/trajectory';
import type { Ball } from '../physics/types';
import {
  createBall,
  createBallRuntime,
  launchBall,
  stepWorld,
  type BallRuntime,
  type DeathCause,
  type SimEvent,
  type World,
} from '../physics/world';
import {
  BALL_RADIUS,
  compileLevel,
  levelMaxPower,
  levelShotTimeout,
  shotImpulse,
  type LevelDef,
} from './level';

export type SessionState =
  | 'aiming'
  | 'flying'
  /** Brief pause after a death, before the ball is put back. */
  | 'respawning'
  | 'sunk';

export type SessionEvent =
  | { type: 'shot'; power: number; direction: Vec2; strokes: number }
  | { type: 'bounce'; point: Vec2; normal: Vec2; speed: number; material: string }
  | { type: 'star'; id: string; position: Vec2; total: number }
  | { type: 'star-lost'; id: string }
  | { type: 'portal'; from: Vec2; to: Vec2 }
  | { type: 'switch'; id: string; position: Vec2; on: boolean }
  | { type: 'shatter'; bodyId: string; position: Vec2 }
  | { type: 'locked'; position: Vec2; needed: number }
  | { type: 'lipout'; position: Vec2 }
  | { type: 'settled'; position: Vec2 }
  | { type: 'died'; position: Vec2; cause: DeathCause; strokes: number }
  | { type: 'timeout'; strokes: number }
  | { type: 'respawn'; position: Vec2 }
  | { type: 'sunk'; strokes: number; stars: number; result: HoleResult };

export type Medal = 'ace' | 'gold' | 'silver' | 'bronze' | 'none';

/** A stylish way to finish a hole. Recognising these rewards flair. */
export type FeatId = 'orbit' | 'clean' | 'ricochet' | 'longRange' | 'grazer' | 'allStars';

export interface Feat {
  id: FeatId;
  label: string;
  description: string;
}

export const FEATS: Record<FeatId, Feat> = {
  orbit: {
    id: 'orbit',
    label: 'Full Orbit',
    description: 'Went all the way around a body before sinking',
  },
  clean: {
    id: 'clean',
    label: 'Clean Sink',
    description: 'Sank it without touching a single surface',
  },
  ricochet: {
    id: 'ricochet',
    label: 'Ricochet',
    description: 'Sank it after four or more bounces',
  },
  longRange: {
    id: 'longRange',
    label: 'Long Range',
    description: 'Sank it from over 1200 units away',
  },
  grazer: {
    id: 'grazer',
    label: 'Grazer',
    description: 'Skimmed a hazard on the way in',
  },
  allStars: {
    id: 'allStars',
    label: 'Perfect Run',
    description: 'Collected all three stars',
  },
};

/** A shot as taken, enough to replay it exactly. */
export interface ShotRecord {
  /** Radians. */
  angle: number;
  /** 0..1. */
  power: number;
}

/** Everything undo needs to put a hole back the way it was. */
interface Snapshot {
  ballPosition: Vec2;
  worldTime: number;
  strokes: number;
  playTime: number;
  longestShot: number;
  safePosition: Vec2;
  banked: string[];
  collected: string[];
  /** Ids of switches that were on. */
  switchesOn: string[];
  /** Deadlines for held switches, so undo restores a countdown mid-flight. */
  switchTimers: Record<string, number>;
  /** Hits left on each breakable. */
  breakables: Record<string, number>;
}

interface FeatTracker {
  /** Radians swept around each gravity body, keyed by body id. */
  swept: Map<string, number>;
  /** Last measured angle to each gravity body. */
  lastAngle: Map<string, number>;
  bounces: number;
  touched: boolean;
  /** Closest approach to a deadly body, in units of that body's radius. */
  closestHazard: number;
  distance: number;
}

const newFeatTracker = (): FeatTracker => ({
  swept: new Map(),
  lastAngle: new Map(),
  bounces: 0,
  touched: false,
  closestHazard: Infinity,
  distance: 0,
});

export interface HoleResult {
  levelId: string;
  strokes: number;
  par: number;
  stars: number;
  medal: Medal;
  /** Seconds of wall-clock play, excluding time spent aiming. */
  time: number;
  /** Longest single shot in world units. */
  longestShot: number;
  /** Style awards earned on the shot that sank it. */
  feats: FeatId[];
  /** The shots taken, so a best run can be replayed as a ghost. */
  shots: ShotRecord[];
  /**
   * True when the player asked for a line on this attempt. A hinted run still
   * counts and still earns its medal — but it does not earn style feats and is
   * not kept as the ghost, because neither would be the player's own.
   */
  hinted: boolean;
}

export const medalFor = (strokes: number, par: number): Medal => {
  // A hole cannot be finished in fewer than one stroke; treat anything lower
  // as one rather than inventing a medal better than an ace.
  if (strokes <= 1) return 'ace';
  if (strokes < par) return 'gold';
  if (strokes === par) return 'silver';
  if (strokes <= par + 2) return 'bronze';
  return 'none';
};

/** Seconds the ball stays gone before it is put back after a death. */
const RESPAWN_DELAY = 0.75;

/** How long the tee is allowed to settle before play starts. */
const SETTLE_SECONDS = 6;

/**
 * Drops the ball from the authored tee and returns where it comes to rest.
 *
 * A tee placed a little off a planet's surface would otherwise start the hole
 * mid-fall, with the player unable to shoot until it lands. Settling up front
 * makes every hole start with a ball that is ready to hit, wherever the author
 * put it. Runs on a throwaway world, so nothing observable is consumed.
 */
export const settledTee = (level: LevelDef): Vec2 => {
  const sandbox = compileLevel(level);
  const ball = createBall(level.tee, BALL_RADIUS);
  ball.atRest = false;
  const runtime = createBallRuntime();
  const events: SimEvent[] = [];
  const steps = Math.round(SETTLE_SECONDS / sandbox.config.timeStep);

  for (let i = 0; i < steps; i++) {
    stepWorld(sandbox, ball, runtime, events);
    // A tee that kills the ball or sinks it is a level bug, not something to
    // paper over — leave it where the author put it and let validation shout.
    if (!runtime.alive || runtime.sunk) return level.tee;
    if (ball.atRest) return ball.position;
  }
  // Free-floating tee in open space: play from exactly where it was authored.
  return level.tee;
};

/**
 * Owns everything about playing one hole: the world, the ball, stroke count,
 * star banking, deaths and respawns. Rendering and input sit on top of this and
 * never touch the physics directly.
 */
export class PlaySession {
  readonly level: LevelDef;
  readonly world: World;
  ball: Ball;
  runtime: BallRuntime;

  state: SessionState = 'aiming';
  strokes = 0;
  /** Elapsed play time in seconds, excluding aiming. */
  playTime = 0;
  /** Total seconds this hole has been open, aiming included. */
  totalTime = 0;
  shotTimer = 0;
  respawnTimer = 0;
  longestShot = 0;
  result: HoleResult | null = null;

  /** Where the ball goes back to after a death or a timed-out shot. */
  private safePosition: Vec2;
  private pendingStars: string[] = [];
  private bankedStars = new Set<string>();
  private accumulator = 0;
  private readonly simEvents: SimEvent[] = [];

  /** One entry per shot taken, for undo. */
  private history: Snapshot[] = [];
  /** The shots taken so far, so a completed run can be replayed as a ghost. */
  private shots: ShotRecord[] = [];
  /** Style tracking for the current shot. */
  private tracker = newFeatTracker();
  private earnedFeats: FeatId[] = [];
  /** Hints taken on this attempt. Undo does not clear it — it was still seen. */
  hintsUsed = 0;

  constructor(level: LevelDef) {
    this.level = level;
    this.world = compileLevel(level);
    this.ball = createBall(settledTee(level), BALL_RADIUS);
    this.runtime = createBallRuntime();
    this.safePosition = this.ball.position;
  }

  /** True when there is a previous shot to take back. */
  get canUndo(): boolean {
    return this.history.length > 0 && (this.state === 'aiming' || this.state === 'flying');
  }

  /** The shots taken so far. Copied, so callers cannot mutate history. */
  get shotList(): ShotRecord[] {
    return this.shots.map((shot) => ({ ...shot }));
  }

  /**
   * Takes back the last shot, restoring the ball, the clock, the stroke count
   * and any stars it collected.
   *
   * This is a convenience, not an advantage: restarting the hole was already
   * free, so undo only saves replaying the shots that came before. Making it
   * free keeps players experimenting instead of grinding.
   */
  undo(): boolean {
    const snapshot = this.history.pop();
    if (!snapshot) return false;

    this.ball = createBall(snapshot.ballPosition, BALL_RADIUS);
    this.ball.atRest = true;
    this.runtime = createBallRuntime();
    this.world.time = snapshot.worldTime;
    this.world.shapeCache = undefined;
    for (const item of this.world.collectibles) {
      item.collected = snapshot.collected.includes(item.id);
    }
    // Switches and shattered blocks are part of the board, so undo has to put
    // the course back as well as the ball.
    for (const pad of this.world.switches) {
      pad.on = snapshot.switchesOn.includes(pad.id);
      pad.offAt = snapshot.switchTimers[pad.id];
    }
    this.world.breakables = { ...snapshot.breakables };
    this.world.revision = (this.world.revision ?? 0) + 1;

    this.strokes = snapshot.strokes;
    this.playTime = snapshot.playTime;
    this.longestShot = snapshot.longestShot;
    this.safePosition = snapshot.safePosition;
    this.bankedStars = new Set(snapshot.banked);
    this.pendingStars = [];
    this.shots = this.shots.slice(0, snapshot.strokes);
    this.earnedFeats = [];
    this.tracker = newFeatTracker();

    this.state = 'aiming';
    this.shotTimer = 0;
    this.respawnTimer = 0;
    this.accumulator = 0;
    this.result = null;
    return true;
  }

  get starCount(): number {
    return this.bankedStars.size;
  }

  get totalStars(): number {
    return this.world.collectibles.length;
  }

  get canShoot(): boolean {
    return this.state === 'aiming' && this.ball.atRest;
  }

  get maxPower(): number {
    return levelMaxPower(this.level);
  }

  /** Fraction of the shot clock consumed, 0..1. Drives the on-screen warning. */
  get shotClock(): number {
    if (this.state !== 'flying') return 0;
    return clamp(this.shotTimer / levelShotTimeout(this.level), 0, 1);
  }

  /**
   * Launches the ball. `direction` need not be normalised; `power` is 0..1 and
   * is scaled by the level's maximum launch speed.
   */
  shoot(direction: Vec2, power: number): SessionEvent[] {
    if (!this.canShoot) return [];
    const dir = V.normalize(direction);
    if (dir.x === 0 && dir.y === 0) return [];

    const clamped = clamp(power, 0, 1);
    if (clamped <= 0.001) return [];

    // Snapshot before mutating anything, so undo can put it all back.
    this.history.push({
      ballPosition: this.ball.position,
      worldTime: this.world.time,
      strokes: this.strokes,
      playTime: this.playTime,
      longestShot: this.longestShot,
      safePosition: this.safePosition,
      banked: [...this.bankedStars],
      collected: this.world.collectibles.filter((c) => c.collected).map((c) => c.id),
      switchesOn: this.world.switches.filter((sw) => sw.on).map((sw) => sw.id),
      switchTimers: Object.fromEntries(
        this.world.switches
          .filter((sw) => sw.offAt !== undefined)
          .map((sw) => [sw.id, sw.offAt as number]),
      ),
      breakables: { ...this.world.breakables },
    });

    this.strokes++;
    this.safePosition = this.ball.position;
    this.shotTimer = 0;
    this.state = 'flying';
    this.tracker = newFeatTracker();
    this.shots.push({ angle: V.angleOf(dir), power: clamped });
    launchBall(this.ball, this.runtime, shotImpulse(dir, clamped, this.maxPower));

    return [{ type: 'shot', power: clamped, direction: dir, strokes: this.strokes }];
  }

  /** Preview of where the current aim would send the ball. */
  predict(direction: Vec2, power: number, options?: Partial<TrajectoryOptions>): Trajectory {
    return predictTrajectory(
      this.world,
      this.ball,
      shotImpulse(direction, power, this.maxPower),
      options,
    );
  }

  /** Advances the simulation. `dt` is real elapsed seconds; clamped internally. */
  update(dt: number): SessionEvent[] {
    const events: SessionEvent[] = [];
    // A long frame (tab switch, breakpoint) must not fast-forward the world.
    const frame = clamp(dt, 0, 0.1);
    this.totalTime += frame;

    if (this.state === 'sunk') return events;

    if (this.state === 'respawning') {
      this.respawnTimer -= frame;
      if (this.respawnTimer <= 0) {
        this.respawn();
        events.push({ type: 'respawn', position: this.ball.position });
      }
      return events;
    }

    if (this.state === 'flying') {
      this.playTime += frame;
      this.shotTimer += frame;
    }

    const step = this.world.config.timeStep;
    this.accumulator += frame;
    // Cap the catch-up work so a slow frame cannot spiral.
    let budget = Math.ceil(0.1 / step);

    while (this.accumulator >= step && budget-- > 0) {
      this.accumulator -= step;
      this.simEvents.length = 0;
      stepWorld(this.world, this.ball, this.runtime, this.simEvents);
      this.trackStyle();
      this.drainSimEvents(events);
      if (this.state !== 'flying') break;
    }

    if (this.state === 'flying' && this.shotTimer >= levelShotTimeout(this.level)) {
      events.push({ type: 'timeout', strokes: this.strokes });
      this.beginRespawn();
    }

    return events;
  }

  /** Puts the ball back at the tee and clears the scorecard. */
  restart(): void {
    this.world.time = 0;
    for (const c of this.world.collectibles) c.collected = false;
    for (const pad of this.world.switches) pad.on = false;
    for (const body of this.level.bodies ?? []) {
      if (body.hitsToBreak !== undefined) this.world.breakables[body.id] = body.hitsToBreak;
    }
    this.world.revision = (this.world.revision ?? 0) + 1;
    this.world.shapeCache = undefined;
    this.ball = createBall(settledTee(this.level), BALL_RADIUS);
    this.runtime = createBallRuntime();
    this.state = 'aiming';
    this.strokes = 0;
    this.playTime = 0;
    this.totalTime = 0;
    this.shotTimer = 0;
    this.respawnTimer = 0;
    this.longestShot = 0;
    this.result = null;
    this.safePosition = this.ball.position;
    this.pendingStars = [];
    this.bankedStars.clear();
    this.accumulator = 0;
    this.history = [];
    this.shots = [];
    this.earnedFeats = [];
    this.hintsUsed = 0;
    this.tracker = newFeatTracker();
  }

  /* ------------------------------------------------------------- internals */

  /**
   * Accumulates the shape of the current shot: how far it has swung around each
   * body, whether it has touched anything, and how close it came to a hazard.
   * Read once on sinking to work out which style awards it earned.
   */
  private trackStyle(): void {
    if (this.state !== 'flying') return;
    const tracker = this.tracker;
    tracker.distance = this.runtime.distance;
    if (this.runtime.airTime === 0) tracker.touched = true;

    for (const body of this.world.bodies) {
      const shape = bodyShapeAt(body, this.world.time);
      const centre = shapeCenter(shape);
      const angle = Math.atan2(this.ball.position.y - centre.y, this.ball.position.x - centre.x);
      const previous = tracker.lastAngle.get(body.id);
      tracker.lastAngle.set(body.id, angle);

      if (body.gravity && body.gravity.strength > 0 && previous !== undefined) {
        // Signed, so swinging out and back cancels instead of counting twice.
        const swept = (tracker.swept.get(body.id) ?? 0) + angleDelta(previous, angle);
        tracker.swept.set(body.id, swept);
      }

      if (MATERIALS[body.material].deadly) {
        const radius = Math.max(shapeRadius(shape), 1);
        const surfaceGap = closestSurfacePoint(shape, this.ball.position).distance;
        tracker.closestHazard = Math.min(tracker.closestHazard, surfaceGap / radius);
      }
    }
  }

  /** Which style awards the shot that just sank the ball earned. */
  private collectFeats(): FeatId[] {
    const feats: FeatId[] = [];
    const tracker = this.tracker;

    for (const swept of tracker.swept.values()) {
      if (Math.abs(swept) >= TAU) {
        feats.push('orbit');
        break;
      }
    }
    if (!tracker.touched) feats.push('clean');
    if (tracker.bounces >= 4) feats.push('ricochet');
    if (tracker.distance >= 1200) feats.push('longRange');
    // Within a third of a hazard's own radius of its surface counts as a graze.
    if (tracker.closestHazard <= 0.33) feats.push('grazer');
    if (this.bankedStars.size === this.totalStars && this.totalStars > 0) feats.push('allStars');

    return feats;
  }

  private drainSimEvents(out: SessionEvent[]): void {
    for (const event of this.simEvents) {
      switch (event.type) {
        case 'bounce':
          this.tracker.bounces++;
          out.push({
            type: 'bounce',
            point: event.point,
            normal: event.normal,
            speed: event.speed,
            material: event.material,
          });
          break;

        case 'collect':
          this.pendingStars.push(event.id);
          out.push({
            type: 'star',
            id: event.id,
            position: event.position,
            total: this.bankedStars.size + this.pendingStars.length,
          });
          break;

        case 'portal':
          out.push({ type: 'portal', from: event.from, to: event.to });
          break;

        case 'switch':
          out.push({ type: 'switch', id: event.id, position: event.position, on: event.on });
          break;

        case 'shatter':
          out.push({ type: 'shatter', bodyId: event.bodyId, position: event.position });
          break;

        case 'locked':
          out.push({ type: 'locked', position: event.position, needed: event.needed });
          break;

        case 'lipout':
          out.push({ type: 'lipout', position: event.position });
          break;

        case 'rest':
          this.bankPendingStars();
          this.longestShot = Math.max(this.longestShot, this.runtime.distance);
          this.safePosition = event.position;
          this.state = 'aiming';
          out.push({ type: 'settled', position: event.position });
          break;

        case 'death':
          // Stars picked up during a fatal shot do not count — finish the shot.
          for (const id of this.pendingStars) {
            const item = this.world.collectibles.find((c) => c.id === id);
            if (item) item.collected = false;
            out.push({ type: 'star-lost', id });
          }
          this.pendingStars = [];
          // A death costs a penalty stroke on top of the shot that caused it.
          this.strokes++;
          out.push({
            type: 'died',
            position: event.position,
            cause: event.cause,
            strokes: this.strokes,
          });
          this.beginRespawn();
          break;

        case 'sink': {
          this.bankPendingStars();
          this.longestShot = Math.max(this.longestShot, this.runtime.distance);
          this.state = 'sunk';
          // The ball can reach the cup without a shot if it rolls in off the
          // tee. Scoring that as zero strokes would report a result better
          // than a hole in one, so the floor is one.
          const strokes = Math.max(1, this.strokes);
          this.strokes = strokes;
          const hinted = this.hintsUsed > 0;
          this.earnedFeats = hinted ? [] : this.collectFeats();
          this.result = {
            levelId: this.level.id,
            strokes,
            par: this.level.par,
            stars: this.bankedStars.size,
            medal: medalFor(strokes, this.level.par),
            time: this.playTime,
            longestShot: this.longestShot,
            feats: this.earnedFeats,
            // A ghost you race should be a run you played.
            shots: hinted ? [] : this.shotList,
            hinted,
          };
          out.push({
            type: 'sunk',
            strokes,
            stars: this.bankedStars.size,
            result: this.result,
          });
          break;
        }
      }
      if (this.state === 'sunk' || this.state === 'respawning') break;
    }
  }

  private bankPendingStars(): void {
    for (const id of this.pendingStars) this.bankedStars.add(id);
    this.pendingStars = [];
  }

  private beginRespawn(): void {
    this.state = 'respawning';
    this.respawnTimer = RESPAWN_DELAY;
    this.ball.velocity = V.ZERO;
  }

  private respawn(): void {
    this.ball = createBall(this.safePosition, BALL_RADIUS);
    this.runtime = createBallRuntime();
    this.ball.atRest = true;
    this.state = 'aiming';
    this.shotTimer = 0;
    this.accumulator = 0;
  }
}
