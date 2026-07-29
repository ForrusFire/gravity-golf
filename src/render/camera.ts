import { clamp, damp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import type { Aabb } from '../physics/geometry';

/**
 * A 2D camera with smoothed follow, fit-to-bounds zoom and a decaying shake
 * offset. World units map to CSS pixels; the renderer applies device pixel
 * ratio separately.
 */
export class Camera {
  position: Vec2 = V.ZERO;
  zoom = 1;

  /** Where the camera is easing toward. */
  target: Vec2 = V.ZERO;
  targetZoom = 1;

  viewportWidth = 1;
  viewportHeight = 1;

  private shakeAmount = 0;
  private shakeTime = 0;
  private shakeOffset: Vec2 = V.ZERO;

  /** How quickly the camera closes the gap to its target, per second. */
  followRate = 6;
  zoomRate = 4;

  /** Clamps the view to these world bounds when set. */
  limits: Aabb | null = null;

  setViewport(width: number, height: number): void {
    this.viewportWidth = Math.max(1, width);
    this.viewportHeight = Math.max(1, height);
  }

  /** Chooses a zoom that fits `bounds` in view, with `padding` world units spare. */
  fit(bounds: Aabb, padding = 40, maxZoom = 1.6): number {
    const w = bounds.maxX - bounds.minX + padding * 2;
    const h = bounds.maxY - bounds.minY + padding * 2;
    return Math.min(this.viewportWidth / w, this.viewportHeight / h, maxZoom);
  }

  snapTo(position: Vec2, zoom = this.targetZoom): void {
    this.target = position;
    this.targetZoom = zoom;
    this.position = position;
    this.zoom = zoom;
    this.applyLimits();
  }

  update(dt: number): void {
    this.position = {
      x: damp(this.position.x, this.target.x, this.followRate, dt),
      y: damp(this.position.y, this.target.y, this.followRate, dt),
    };
    this.zoom = damp(this.zoom, this.targetZoom, this.zoomRate, dt);
    this.applyLimits();

    if (this.shakeTime > 0) {
      this.shakeTime = Math.max(0, this.shakeTime - dt);
      const t = this.shakeTime;
      // Two out-of-phase sinusoids read as a jolt rather than a vibration.
      const decay = t * t;
      this.shakeOffset = {
        x: Math.sin(t * 63) * this.shakeAmount * decay,
        y: Math.cos(t * 47) * this.shakeAmount * decay,
      };
      if (this.shakeTime === 0) this.shakeOffset = V.ZERO;
    }
  }

  /** Adds a camera jolt. Repeat calls take the strongest, they do not stack. */
  shake(amount: number, duration = 0.35): void {
    if (amount <= this.shakeAmount && this.shakeTime > 0) return;
    this.shakeAmount = amount;
    this.shakeTime = duration;
  }

  clearShake(): void {
    this.shakeAmount = 0;
    this.shakeTime = 0;
    this.shakeOffset = V.ZERO;
  }

  /** The camera centre including shake — what the renderer should draw around. */
  get eye(): Vec2 {
    return V.add(this.position, this.shakeOffset);
  }

  worldToScreen(p: Vec2): Vec2 {
    const eye = this.eye;
    return {
      x: (p.x - eye.x) * this.zoom + this.viewportWidth / 2,
      y: (p.y - eye.y) * this.zoom + this.viewportHeight / 2,
    };
  }

  screenToWorld(p: Vec2): Vec2 {
    const eye = this.eye;
    return {
      x: (p.x - this.viewportWidth / 2) / this.zoom + eye.x,
      y: (p.y - this.viewportHeight / 2) / this.zoom + eye.y,
    };
  }

  /** The world-space rectangle currently visible, expanded by `margin`. */
  visibleBounds(margin = 0): Aabb {
    const eye = this.eye;
    const halfW = this.viewportWidth / 2 / this.zoom + margin;
    const halfH = this.viewportHeight / 2 / this.zoom + margin;
    return {
      minX: eye.x - halfW,
      minY: eye.y - halfH,
      maxX: eye.x + halfW,
      maxY: eye.y + halfH,
    };
  }

  private applyLimits(): void {
    const limits = this.limits;
    if (!limits) return;

    const halfW = this.viewportWidth / 2 / this.zoom;
    const halfH = this.viewportHeight / 2 / this.zoom;
    const spanX = limits.maxX - limits.minX;
    const spanY = limits.maxY - limits.minY;

    // When the level is narrower than the view, centre it instead of clamping.
    const x =
      spanX <= halfW * 2
        ? (limits.minX + limits.maxX) / 2
        : clamp(this.position.x, limits.minX + halfW, limits.maxX - halfW);
    const y =
      spanY <= halfH * 2
        ? (limits.minY + limits.maxY) / 2
        : clamp(this.position.y, limits.minY + halfH, limits.maxY - halfH);

    this.position = { x, y };
  }
}
