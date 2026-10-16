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
/**
 * Nebula tints per sky, so each chapter has a sky of its own. The first is the
 * palette's own; the rest rotate through hues that keep the same weight.
 */
const SKIES: Array<[string, string] | null> = [
  null,
  ['rgba(150, 60, 120, 0.26)', 'rgba(60, 80, 170, 0.22)'],
  ['rgba(40, 110, 150, 0.26)', 'rgba(30, 140, 120, 0.18)'],
  ['rgba(170, 80, 50, 0.22)', 'rgba(110, 50, 140, 0.24)'],
  ['rgba(60, 60, 170, 0.28)', 'rgba(140, 60, 170, 0.2)'],
  ['rgba(30, 120, 110, 0.22)', 'rgba(80, 60, 150, 0.24)'],
  ['rgba(160, 50, 90, 0.24)', 'rgba(170, 110, 50, 0.16)'],
];

/** Not every star is white: a sky of one colour reads as a texture. */
const STAR_TINTS = ['#ffffff', '#ffffff', '#ffffff', '#cfe0ff', '#bcd2ff', '#ffe9c4', '#ffd6c8'];

export class Starfield {
  private layers: BakedLayer[] = [];
  private bakedFor: string | null = null;
  private time = 0;
  private variant = 0;
  private useSkyTints = true;

  constructor(
    private readonly tileSize = 1024,
    private readonly seed = 20240,
  ) {}

  /**
   * Picks the sky: a different scatter of stars and, unless the palette must
   * stay as it is (high contrast), a different nebula tint.
   */
  setVariant(variant: number, tinted = true): void {
    this.variant = Math.max(0, Math.floor(variant));
    this.useSkyTints = tinted;
  }

  update(dt: number): void {
    this.time += dt;
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
    // Drawn in device pixels, tiles copied 1:1 at whole-pixel positions. A
    // tile scaled up to the backing store is a filtered blit of the whole
    // screen per layer; on a device rasterising on the CPU that alone took a
    // frame from 17ms to 50ms.
    const ratio = Math.max(1, ctx.getTransform().a);
    const pxWidth = Math.ceil(width * ratio);
    const pxHeight = Math.ceil(height * ratio);
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const gradient = ctx.createLinearGradient(0, 0, 0, pxHeight);
    gradient.addColorStop(0, palette.spaceTop);
    gradient.addColorStop(1, palette.spaceBottom);
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, pxWidth, pxHeight);

    this.bake(palette, ratio);

    const tile = this.tilePixels;
    for (const layer of this.layers) {
      if (!layer.canvas) continue;
      // A slow global shimmer costs nothing, unlike per-star twinkling.
      ctx.globalAlpha = twinkle ? 0.86 + 0.14 * Math.sin(this.time * 0.7 + layer.depth * 9) : 1;

      const shiftX = -eye.x * layer.depth * zoom * ratio;
      const shiftY = -eye.y * layer.depth * zoom * ratio;
      const startX = Math.round((((shiftX % tile) + tile) % tile) - tile);
      const startY = Math.round((((shiftY % tile) + tile) % tile) - tile);

      for (let x = startX; x < pxWidth; x += tile) {
        if (x + tile <= 0) continue;
        for (let y = startY; y < pxHeight; y += tile) {
          if (y + tile <= 0) continue;
          ctx.drawImage(layer.canvas as CanvasImageSource, x, y);
        }
      }
    }
    ctx.restore();
    ctx.globalAlpha = 1;
  }

  private vignette: { key: string; canvas: HTMLCanvasElement | OffscreenCanvas | null } | null =
    null;

  /**
   * Darkens the corners so the eye settles on the middle of the hole, the way a
   * lens does. Baked per viewport size: a screen-sized radial gradient is the
   * most expensive single fill a 2D canvas does, and it never changes.
   */
  drawVignette(ctx: CanvasRenderingContext2D, width: number, height: number): void {
    const key = `${Math.round(width)}x${Math.round(height)}`;
    if (this.vignette?.key !== key) {
      const canvas = createCanvas(Math.max(1, Math.round(width)), Math.max(1, Math.round(height)));
      const vctx = canvas?.getContext('2d') as CanvasRenderingContext2D | null;
      if (canvas && vctx) {
        const r = Math.hypot(width, height) / 2;
        const grad = vctx.createRadialGradient(width / 2, height / 2, r * 0.45, width / 2, height / 2, r);
        grad.addColorStop(0, 'rgba(0,0,0,0)');
        grad.addColorStop(1, 'rgba(0,0,6,0.55)');
        vctx.fillStyle = grad;
        vctx.fillRect(0, 0, width, height);
      }
      this.vignette = { key, canvas };
    }
    if (this.vignette.canvas) {
      ctx.drawImage(this.vignette.canvas as CanvasImageSource, 0, 0, width, height);
    }
  }

  /** Device pixels per tile edge, as last baked. */
  private tilePixels = 1024;

  /**
   * Rasterises each depth band once for the given palette, at the backing
   * store's own resolution so drawing is a plain copy. The tile is capped at
   * 1.5x its CSS size in pixels — on a 2x screen it covers a little less sky
   * per repeat, rather than costing 16MB a layer.
   */
  private bake(palette: Palette, ratio: number): void {
    const sky = this.useSkyTints ? SKIES[this.variant % SKIES.length] : null;
    const nebulaA = sky?.[0] ?? palette.nebulaA;
    const nebulaB = sky?.[1] ?? palette.nebulaB;
    const key = `${palette.star}|${nebulaA}|${nebulaB}|${this.variant}|${ratio}`;
    if (this.bakedFor === key) return;
    this.bakedFor = key;
    const baseSeed = this.seed + this.variant * 104729;

    const pixels = Math.round(this.tileSize * Math.min(ratio, 1.5));
    this.tilePixels = pixels;
    // The content is laid out in CSS units over the tile.
    const size = pixels / ratio;
    this.layers = LAYERS.map((spec, index) => {
      const canvas = createCanvas(pixels, pixels);
      const ctx = canvas?.getContext('2d') as CanvasRenderingContext2D | null;
      if (!canvas || !ctx) return { depth: spec.depth, canvas: null };
      ctx.scale(ratio, ratio);

      const rng = new Rng(baseSeed + index * 7919);

      for (let i = 0; i < spec.nebulae; i++) {
        const cx = rng.range(0, size);
        const cy = rng.range(0, size);
        const r = rng.range(size * 0.22, size * 0.45);
        const color = rng.bool() ? nebulaA : nebulaB;
        // Painted toroidally — once per neighbouring tile. A nebula near the
        // edge of the tile used to be cut off flat, and since the tile repeats
        // across the screen that cut showed as a hard vertical seam on every
        // hole.
        for (const ox of [-size, 0, size]) {
          for (const oy of [-size, 0, size]) {
            const x = cx + ox;
            const y = cy + oy;
            if (x + r < 0 || x - r > size || y + r < 0 || y - r > size) continue;
            const grad = ctx.createRadialGradient(x, y, 0, x, y, r);
            grad.addColorStop(0, color);
            grad.addColorStop(1, 'rgba(0,0,0,0)');
            ctx.fillStyle = grad;
            ctx.fillRect(x - r, y - r, r * 2, r * 2);
          }
        }
      }

      // Galaxies far off in the deepest band: the sky goes further than stars.
      if (index === 0) {
        for (let i = 0; i < 2; i++) {
          paintGalaxy(ctx, rng.range(120, size - 120), rng.range(120, size - 120), rng);
        }
      }

      for (let i = 0; i < spec.stars; i++) {
        // Keep stars clear of the tile seam so the repeat is not obvious.
        const x = rng.range(2, size - 2);
        const y = rng.range(2, size - 2);
        ctx.globalAlpha = rng.range(spec.minAlpha, spec.maxAlpha);
        ctx.fillStyle = index === 0 ? palette.star : rng.pick(STAR_TINTS);
        ctx.beginPath();
        ctx.arc(x, y, rng.range(spec.minSize, spec.maxSize), 0, TAU);
        ctx.fill();
      }

      // A few bright stars in the middle band, with the cross of spikes a
      // telescope gives them.
      if (index === 1) {
        for (let i = 0; i < 3; i++) {
          paintBrightStar(ctx, rng.range(40, size - 40), rng.range(40, size - 40), rng);
        }
      }
      ctx.globalAlpha = 1;

      return { depth: spec.depth, canvas };
    });
  }
}

/** A distant galaxy: a small tilted ellipse of light with a brighter core. */
const paintGalaxy = (ctx: CanvasRenderingContext2D, x: number, y: number, rng: Rng): void => {
  const r = rng.range(22, 40);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(rng.range(0, TAU));
  ctx.scale(1, rng.range(0.3, 0.5));
  const hue = rng.bool() ? '200, 210, 255' : '255, 225, 205';
  const halo = ctx.createRadialGradient(0, 0, 0, 0, 0, r);
  halo.addColorStop(0, `rgba(${hue}, 0.32)`);
  halo.addColorStop(0.18, `rgba(${hue}, 0.12)`);
  halo.addColorStop(1, `rgba(${hue}, 0)`);
  ctx.fillStyle = halo;
  ctx.fillRect(-r, -r, r * 2, r * 2);
  ctx.restore();
};

const paintBrightStar = (ctx: CanvasRenderingContext2D, x: number, y: number, rng: Rng): void => {
  // Cool and dim on purpose: nothing in the sky may look like a star to
  // collect, which are gold and five-pointed.
  const tint = rng.pick(['200, 215, 255', '185, 205, 255', '215, 225, 255']);
  const glow = ctx.createRadialGradient(x, y, 0, x, y, 7);
  glow.addColorStop(0, `rgba(${tint}, 0.6)`);
  glow.addColorStop(1, `rgba(${tint}, 0)`);
  ctx.globalAlpha = 1;
  ctx.fillStyle = glow;
  ctx.fillRect(x - 7, y - 7, 14, 14);
  const spike = rng.range(6, 11);
  for (const [dx, dy] of [
    [1, 0],
    [0, 1],
  ] as const) {
    const line = ctx.createLinearGradient(x - dx * spike, y - dy * spike, x + dx * spike, y + dy * spike);
    line.addColorStop(0, `rgba(${tint}, 0)`);
    line.addColorStop(0.5, `rgba(${tint}, 0.45)`);
    line.addColorStop(1, `rgba(${tint}, 0)`);
    ctx.strokeStyle = line;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x - dx * spike, y - dy * spike);
    ctx.lineTo(x + dx * spike, y + dy * spike);
    ctx.stroke();
  }
};

const createCanvas = (width: number, height: number): HTMLCanvasElement | OffscreenCanvas | null => {
  if (typeof OffscreenCanvas !== 'undefined') return new OffscreenCanvas(width, height);
  if (typeof document === 'undefined') return null;
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  return canvas;
};
