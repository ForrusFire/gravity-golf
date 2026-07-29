import { TAU } from '../core/math';
import { Rng } from '../core/rng';
import type { Vec2 } from '../core/vec2';
import type { Palette } from './palette';

interface Star {
  x: number;
  y: number;
  size: number;
  brightness: number;
  /** Parallax factor: 0 is infinitely far, 1 moves with the world. */
  depth: number;
  twinklePhase: number;
  twinkleRate: number;
}

interface NebulaBlob {
  x: number;
  y: number;
  radius: number;
  depth: number;
  color: string;
}

/**
 * Parallax background. Stars live on a repeating tile in "sky space" so the
 * field is infinite without storing an infinite number of stars.
 */
export class Starfield {
  private stars: Star[] = [];
  private nebulae: NebulaBlob[] = [];
  private time = 0;

  constructor(
    private readonly tileSize = 1400,
    seed = 20240,
    starCount = 260,
  ) {
    const rng = new Rng(seed);
    for (let i = 0; i < starCount; i++) {
      const depth = rng.range(0.05, 0.55);
      this.stars.push({
        x: rng.range(0, tileSize),
        y: rng.range(0, tileSize),
        // Nearer stars are drawn larger and brighter.
        size: rng.range(0.5, 1.6) * (0.6 + depth),
        brightness: rng.range(0.25, 1) * (0.5 + depth),
        depth,
        twinklePhase: rng.range(0, TAU),
        twinkleRate: rng.range(0.4, 1.8),
      });
    }
    for (let i = 0; i < 7; i++) {
      this.nebulae.push({
        x: rng.range(0, tileSize),
        y: rng.range(0, tileSize),
        radius: rng.range(280, 620),
        depth: rng.range(0.04, 0.16),
        color: rng.bool() ? 'a' : 'b',
      });
    }
  }

  update(dt: number): void {
    this.time += dt;
  }

  /**
   * Draws the field for a camera centred on `eye`. Runs in screen space, so the
   * caller must not have a world transform applied.
   */
  draw(
    ctx: CanvasRenderingContext2D,
    eye: Vec2,
    width: number,
    height: number,
    zoom: number,
    palette: Palette,
    twinkle: boolean,
  ): void {
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, palette.spaceTop);
    gradient.addColorStop(1, palette.spaceBottom);
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);

    this.drawLayer(this.nebulae, eye, width, height, zoom, (blob, sx, sy, scale) => {
      const r = blob.radius * scale;
      const grad = ctx.createRadialGradient(sx, sy, 0, sx, sy, r);
      grad.addColorStop(0, blob.color === 'a' ? palette.nebulaA : palette.nebulaB);
      grad.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, TAU);
      ctx.fill();
    });

    ctx.fillStyle = palette.star;
    this.drawLayer(this.stars, eye, width, height, zoom, (star, sx, sy, scale) => {
      let alpha = star.brightness;
      if (twinkle) {
        alpha *= 0.72 + 0.28 * Math.sin(this.time * star.twinkleRate + star.twinklePhase);
      }
      ctx.globalAlpha = alpha;
      const r = star.size * Math.max(0.6, scale);
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, TAU);
      ctx.fill();
    });
    ctx.globalAlpha = 1;
  }

  /** Tiles one parallax layer across the viewport and draws each item once. */
  private drawLayer<T extends { x: number; y: number; depth: number }>(
    items: T[],
    eye: Vec2,
    width: number,
    height: number,
    zoom: number,
    drawItem: (item: T, sx: number, sy: number, scale: number) => void,
  ): void {
    const tile = this.tileSize;
    for (const item of items) {
      const scale = zoom * (0.55 + item.depth);
      // Offset the layer by a fraction of the camera position for parallax.
      const baseX = item.x - eye.x * item.depth * zoom;
      const baseY = item.y - eye.y * item.depth * zoom;
      const step = tile;

      // Wrap into the visible range, then tile forward.
      let startX = ((baseX % step) + step) % step;
      let startY = ((baseY % step) + step) % step;
      startX -= step;
      startY -= step;

      for (let sx = startX; sx < width + step; sx += step) {
        for (let sy = startY; sy < height + step; sy += step) {
          if (sx < -step || sy < -step) continue;
          drawItem(item, sx, sy, scale);
        }
      }
    }
  }
}
