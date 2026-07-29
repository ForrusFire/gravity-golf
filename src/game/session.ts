import { clamp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
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
  | { type: 'lipout'; position: Vec2 }
  | { type: 'settled'; position: Vec2 }
  | { type: 'died'; position: Vec2; cause: DeathCause; strokes: number }
  | { type: 'timeout'; strokes: number }
  | { type: 'respawn'; position: Vec2 }
  | { type: 'sunk'; strokes: number; stars: number; result: HoleResult };

export type Medal = 'ace' | 'gold' | 'silver' | 'bronze' | 'none';

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
}

export const medalFor = (strokes: number, par: number): Medal => {
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

  constructor(level: LevelDef) {
    this.level = level;
    this.world = compileLevel(level);
    this.ball = createBall(settledTee(level), BALL_RADIUS);
    this.runtime = createBallRuntime();
    this.safePosition = this.ball.position;
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

    this.strokes++;
    this.safePosition = this.ball.position;
    this.shotTimer = 0;
    this.state = 'flying';
    launchBall(this.ball, this.runtime, V.mul(dir, clamped * this.maxPower));

    return [{ type: 'shot', power: clamped, direction: dir, strokes: this.strokes }];
  }

  /** Preview of where the current aim would send the ball. */
  predict(direction: Vec2, power: number, options?: Partial<TrajectoryOptions>): Trajectory {
    const dir = V.normalize(direction);
    const impulse = V.mul(dir, clamp(power, 0, 1) * this.maxPower);
    return predictTrajectory(this.world, this.ball, impulse, options);
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
  }

  /* ------------------------------------------------------------- internals */

  private drainSimEvents(out: SessionEvent[]): void {
    for (const event of this.simEvents) {
      switch (event.type) {
        case 'bounce':
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
          this.result = {
            levelId: this.level.id,
            strokes: this.strokes,
            par: this.level.par,
            stars: this.bankedStars.size,
            medal: medalFor(this.strokes, this.level.par),
            time: this.playTime,
            longestShot: this.longestShot,
          };
          out.push({
            type: 'sunk',
            strokes: this.strokes,
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
