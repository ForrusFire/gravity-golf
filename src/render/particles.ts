import { TAU } from '../core/math';
import { Rng } from '../core/rng';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';

export interface Particle {
  x: number;
  y: number;
  vx: number;
  vy: number;
  life: number;
  maxLife: number;
  size: number;
  color: string;
  /** Velocity retained per second. */
  drag: number;
  /** Extra downward-in-screen drift, for sparks that should settle. */
  gravity: number;
  shape: 'dot' | 'spark' | 'ring';
  spin: number;
  angle: number;
}

export interface EmitOptions {
  count: number;
  color: string;
  speed: number;
  speedVariance?: number;
  life: number;
  lifeVariance?: number;
  size: number;
  sizeVariance?: number;
  /** Centre direction of the spray. Omit for a full circle. */
  direction?: Vec2;
  /** Half-angle of the spray in radians. */
  spread?: number;
  drag?: number;
  gravity?: number;
  shape?: Particle['shape'];
  /** Initial velocity added to every particle, e.g. the ball's own motion. */
  inherit?: Vec2;
}

/**
 * Fixed-capacity particle pool. Allocating up front keeps the render loop free
 * of GC pauses, and the oldest particle is recycled when the pool is full.
 */
export class ParticleSystem {
  private particles: Particle[] = [];
  private nextIndex = 0;
  private rng: Rng;

  constructor(readonly capacity = 900, seed = 1337) {
    this.rng = new Rng(seed);
    for (let i = 0; i < capacity; i++) {
      this.particles.push({
        x: 0,
        y: 0,
        vx: 0,
        vy: 0,
        life: 0,
        maxLife: 1,
        size: 1,
        color: '#fff',
        drag: 1,
        gravity: 0,
        shape: 'dot',
        spin: 0,
        angle: 0,
      });
    }
  }

  get active(): number {
    let count = 0;
    for (const p of this.particles) if (p.life > 0) count++;
    return count;
  }

  clear(): void {
    for (const p of this.particles) p.life = 0;
  }

  emit(origin: Vec2, options: EmitOptions): void {
    const {
      count,
      color,
      speed,
      speedVariance = 0.4,
      life,
      lifeVariance = 0.3,
      size,
      sizeVariance = 0.4,
      direction,
      spread = Math.PI,
      drag = 0.35,
      gravity = 0,
      shape = 'dot',
      inherit,
    } = options;

    const baseAngle = direction ? V.angleOf(direction) : 0;

    for (let i = 0; i < count; i++) {
      const p = this.particles[this.nextIndex]!;
      this.nextIndex = (this.nextIndex + 1) % this.particles.length;

      const angle = direction
        ? baseAngle + this.rng.range(-spread, spread)
        : this.rng.range(0, TAU);
      const magnitude = speed * (1 + this.rng.range(-speedVariance, speedVariance));

      p.x = origin.x;
      p.y = origin.y;
      p.vx = Math.cos(angle) * magnitude + (inherit?.x ?? 0);
      p.vy = Math.sin(angle) * magnitude + (inherit?.y ?? 0);
      p.maxLife = Math.max(0.05, life * (1 + this.rng.range(-lifeVariance, lifeVariance)));
      p.life = p.maxLife;
      p.size = Math.max(0.4, size * (1 + this.rng.range(-sizeVariance, sizeVariance)));
      p.color = color;
      p.drag = drag;
      p.gravity = gravity;
      p.shape = shape;
      p.angle = angle;
      p.spin = this.rng.range(-6, 6);
    }
  }

  update(dt: number): void {
    for (const p of this.particles) {
      if (p.life <= 0) continue;
      p.life -= dt;
      if (p.life <= 0) continue;
      const decay = Math.exp(-p.drag * dt);
      p.vx *= decay;
      p.vy = p.vy * decay + p.gravity * dt;
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      p.angle += p.spin * dt;
    }
  }

  /** Draws every live particle. The context must already be in world space. */
  draw(ctx: CanvasRenderingContext2D): void {
    ctx.save();
    for (const p of this.particles) {
      if (p.life <= 0) continue;
      const t = p.life / p.maxLife;
      ctx.globalAlpha = t < 0.25 ? t / 0.25 : 1;
      ctx.fillStyle = p.color;
      ctx.strokeStyle = p.color;

      switch (p.shape) {
        case 'dot': {
          const r = p.size * t;
          ctx.beginPath();
          ctx.arc(p.x, p.y, Math.max(0.2, r), 0, TAU);
          ctx.fill();
          break;
        }
        case 'spark': {
          const len = p.size * 3 * t;
          const dx = Math.cos(p.angle) * len;
          const dy = Math.sin(p.angle) * len;
          ctx.lineWidth = Math.max(0.4, p.size * 0.5);
          ctx.lineCap = 'round';
          ctx.beginPath();
          ctx.moveTo(p.x - dx, p.y - dy);
          ctx.lineTo(p.x + dx, p.y + dy);
          ctx.stroke();
          break;
        }
        case 'ring': {
          const r = p.size * (1.6 - t);
          ctx.lineWidth = Math.max(0.4, p.size * 0.28 * t);
          ctx.beginPath();
          ctx.arc(p.x, p.y, Math.max(0.2, r), 0, TAU);
          ctx.stroke();
          break;
        }
      }
    }
    ctx.restore();
  }

  /* ---------------------------------------------------- gameplay presets */

  burst(position: Vec2, color: string, strength = 1): void {
    this.emit(position, {
      count: Math.round(10 * strength),
      color,
      speed: 90 * strength,
      life: 0.45,
      size: 2.4,
      shape: 'dot',
    });
  }

  impact(position: Vec2, normal: Vec2, speed: number, color: string): void {
    const strength = Math.min(1, speed / 500);
    this.emit(position, {
      count: Math.round(3 + 9 * strength),
      color,
      speed: 40 + 160 * strength,
      life: 0.3 + 0.25 * strength,
      size: 1.6 + strength,
      direction: normal,
      spread: 1.1,
      shape: 'spark',
      drag: 2.2,
    });
  }

  trailPuff(position: Vec2, velocity: Vec2, color: string): void {
    this.emit(position, {
      count: 1,
      color,
      speed: 12,
      life: 0.5,
      size: 2,
      drag: 3,
      inherit: V.mul(velocity, -0.05),
      shape: 'dot',
    });
  }

  starPop(position: Vec2, color: string): void {
    this.emit(position, {
      count: 16,
      color,
      speed: 140,
      life: 0.6,
      size: 2.6,
      shape: 'spark',
      drag: 2.4,
    });
    this.emit(position, { count: 1, color, speed: 0, life: 0.5, size: 16, shape: 'ring', drag: 0 });
  }

  sinkCelebration(position: Vec2, color: string): void {
    this.emit(position, {
      count: 40,
      color,
      speed: 220,
      life: 1.1,
      size: 3,
      shape: 'spark',
      drag: 1.6,
    });
    this.emit(position, { count: 3, color, speed: 0, life: 0.9, size: 22, shape: 'ring', drag: 0 });
  }

  explosion(position: Vec2, color: string): void {
    this.emit(position, {
      count: 36,
      color,
      speed: 260,
      life: 0.8,
      size: 3.2,
      shape: 'spark',
      drag: 1.8,
    });
    this.emit(position, {
      count: 18,
      color,
      speed: 90,
      life: 1.2,
      size: 5,
      shape: 'dot',
      drag: 1.1,
    });
  }
}
