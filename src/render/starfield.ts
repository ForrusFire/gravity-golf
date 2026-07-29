import { TAU } from '../core/math';
import { Rng } from '../core/rng';
import type { Vec2 } from '../core/vec2';
import type { Palette } from './palette';

interface LayerSpec {
  /** Parallax factor: 0 is infinitely far, 1 moves with the world. */
  depth: number;
  stars: number;
  nebulae: number;
  minSize: number;
  maxSize: number;
  minAlpha: number;
  maxAlpha: number;
}

/**
 * Three depth bands rather than per-star depth. Baking each band into one
 * bitmap is what makes the background cheap to draw; the difference from
 * continuous parallax is not visible at these speeds.
 */
const LAYERS: LayerSpec[] = [
  { depth: 0.08, stars: 130, nebulae: 5, minSize: 0.5, maxSize: 1.1, minAlpha: 0.2, maxAlpha: 0.5 },
  { depth: 0.2, stars: 80, nebulae: 2, minSize: 0.8, maxSize: 1.6, minAlpha: 0.35, maxAlpha: 0.75 },
  { depth: 0.42, stars: 45, nebulae: 0, minSize: 1.1, maxSize: 2.2, minAlpha: 0.5, maxAlpha: 1 },
];

interface BakedLayer {
  depth: number;
  canvas: HTMLCanvasElement | OffscreenCanvas | null;
}

/**
 * Parallax background.
 *
 * Each depth band is rasterised once into a square tile and then blitted,
 * instead of re-rasterising hundreds of arcs and radial gradients every frame.
 * Radial-gradient fills are the single most expensive thing a 2D canvas does,
 * and the background used to redraw dozens of screen-sized ones per frame.
 */
export class Starfield {
  private layers: BakedLayer[] = [];
  private bakedFor: string | null = null;
  private time = 0;

  constructor(
    private readonly tileSize = 1024,
    private readonly seed = 20240,
  ) {}

  update(dt: number): void {
    this.time += dt;
  }

  /** Discards baked tiles so the next draw re-bakes (e.g. on palette change). */
  invalidate(): void {
    this.bakedFor = null;
  }

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

    this.bake(palette);

    const tile = this.tileSize;
    ctx.save();
    for (const layer of this.layers) {
      if (!layer.canvas) continue;
      // A slow global shimmer costs nothing, unlike per-star twinkling.
      ctx.globalAlpha = twinkle ? 0.86 + 0.14 * Math.sin(this.time * 0.7 + layer.depth * 9) : 1;

      const shiftX = -eye.x * layer.depth * zoom;
      const shiftY = -eye.y * layer.depth * zoom;
      const startX = (((shiftX % tile) + tile) % tile) - tile;
      const startY = (((shiftY % tile) + tile) % tile) - tile;

      for (let x = startX; x < width; x += tile) {
        for (let y = startY; y < height; y += tile) {
          ctx.drawImage(layer.canvas as CanvasImageSource, x, y);
        }
      }
    }
    ctx.restore();
    ctx.globalAlpha = 1;
  }

  /** Rasterises each depth band once for the given palette. */
  private bake(palette: Palette): void {
    const key = `${palette.star}|${palette.nebulaA}|${palette.nebulaB}`;
    if (this.bakedFor === key) return;
    this.bakedFor = key;

    const size = this.tileSize;
    this.layers = LAYERS.map((spec, index) => {
      const canvas = createCanvas(size, size);
      const ctx = canvas?.getContext('2d') as CanvasRenderingContext2D | null;
      if (!canvas || !ctx) return { depth: spec.depth, canvas: null };

      const rng = new Rng(this.seed + index * 7919);

      for (let i = 0; i < spec.nebulae; i++) {
        const cx = rng.range(0, size);
        const cy = rng.range(0, size);
        const r = rng.range(size * 0.22, size * 0.45);
        const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, r);
        grad.addColorStop(0, rng.bool() ? palette.nebulaA : palette.nebulaB);
        grad.addColorStop(1, 'rgba(0,0,0,0)');
        ctx.fillStyle = grad;
        ctx.fillRect(cx - r, cy - r, r * 2, r * 2);
      }

      ctx.fillStyle = palette.star;
      for (let i = 0; i < spec.stars; i++) {
        // Keep stars clear of the tile seam so the repeat is not obvious.
        const x = rng.range(2, size - 2);
        const y = rng.range(2, size - 2);
        ctx.globalAlpha = rng.range(spec.minAlpha, spec.maxAlpha);
        ctx.beginPath();
        ctx.arc(x, y, rng.range(spec.minSize, spec.maxSize), 0, TAU);
        ctx.fill();
      }
      ctx.globalAlpha = 1;

      return { depth: spec.depth, canvas };
    });
  }
}

const createCanvas = (width: number, height: number): HTMLCanvasElement | OffscreenCanvas | null => {
  if (typeof OffscreenCanvas !== 'undefined') return new OffscreenCanvas(width, height);
  if (typeof document === 'undefined') return null;
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  return canvas;
};
