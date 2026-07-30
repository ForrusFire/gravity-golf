import type { Vec2 } from '../core/vec2';
import * as V from '../core/vec2';
import type { LevelDef } from './level';
import { PlaySession, type ShotRecord } from './session';

/**
 * Replays a recorded run alongside the live one, so a player has their own best
 * attempt to race rather than only a number to beat.
 *
 * It drives a second PlaySession with the recorded shots, firing each one as
 * soon as that session is ready to shoot. Because the physics is deterministic,
 * a static hole reproduces the original run exactly. On a hole with moving parts
 * the ghost can drift and even die — it is cosmetic, so it simply stops being
 * drawn rather than pretending.
 */
export class GhostRunner {
  private session: PlaySession;
  private shots: ShotRecord[];
  private index = 0;
  private finished = false;
  private failed = false;
  /** Recent positions, for the ghost's trail. */
  private trailPoints: Vec2[] = [];
  private trailClock = 0;

  constructor(level: LevelDef, shots: readonly ShotRecord[]) {
    this.session = new PlaySession(level);
    this.shots = shots.map((shot) => ({ ...shot }));
  }

  /** False once the ghost has finished its run or gone off the rails. */
  get visible(): boolean {
    return !this.finished && !this.failed && this.shots.length > 0;
  }

  get position(): Vec2 {
    return this.session.ball.position;
  }

  get trail(): Vec2[] {
    return this.trailPoints;
  }

  /** Strokes the ghost has played so far — for a "you vs best" readout. */
  get strokes(): number {
    return this.session.strokes;
  }

  advance(dt: number): void {
    if (!this.visible) return;

    // Fire the next recorded shot the moment the ghost is able to.
    if (this.session.canShoot) {
      const shot = this.shots[this.index];
      if (!shot) {
        this.finished = true;
        return;
      }
      this.index++;
      this.session.shoot(V.fromAngle(shot.angle), shot.power);
    }

    this.session.update(dt);

    if (this.session.state === 'sunk') {
      this.finished = true;
      return;
    }
    // A drifted ghost that dies or runs out of recorded shots is not worth
    // showing; the live run is the one that matters.
    if (this.session.strokes > this.shots.length) {
      this.failed = true;
      return;
    }

    this.trailClock += dt;
    if (this.trailClock >= 1 / 30) {
      this.trailClock = 0;
      this.trailPoints.push(this.session.ball.position);
      if (this.trailPoints.length > 22) this.trailPoints.shift();
    }
  }
}
