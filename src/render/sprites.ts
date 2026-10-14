import { TAU } from '../core/math';
import { Rng, hashSeed } from '../core/rng';
import type { BodyStyle } from '../physics/types';
import type { BodyColors } from './palette';
import type { PaintRequest, PaintResponse } from '../workers/paint.worker';

type AnyCanvas = HTMLCanvasElement | OffscreenCanvas;

/**
 * The direction light falls from, shared by every painted body so the whole
 * scene reads as lit by one star. Upper-left, the convention the eye expects.
 */
export const LIGHT = { x: -0.62, y: -0.78 };

/** Extra room around the disc for the atmosphere halo. */
const HALO = 0.22;

const createCanvas = (size: number): AnyCanvas | null => {
  if (typeof OffscreenCanvas !== 'undefined') return new OffscreenCanvas(size, size);
  if (typeof document === 'undefined') return null;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  return canvas;
};

export interface PlanetSprite {
  canvas: AnyCanvas | ImageBitmap;
  /** World units from the sprite's edge to the body's centre. */
  half: number;
}

/**
 * Paints circular bodies once and hands back the bitmap.
 *
 * Proper planet shading — a terminator, an atmosphere, surface texture — is a
 * dozen gradient fills per body. Done live every frame that would eat the
 * frame budget several times over; done once and blitted it costs a single
 * drawImage. Sprites are rendered at the on-screen resolution, bucketed so a
 * zoom does not repaint every frame, and evicted when the level changes.
 */
export class PlanetPainter {
  /** Per body, its sprites by resolution bucket. */
  private cache = new Map<string, Map<number, PlanetSprite | null>>();
  /** Better-resolution repaints waiting for spare frame time. */
  private queue = new Map<string, () => void>();
  /** Big paints in flight on the worker, by queue key. */
  private inFlight = new Map<string, { entry: Map<number, PlanetSprite | null>; bucket: number }>();
  private worker: Worker | null | undefined;

  clear(): void {
    this.cache.clear();
    this.queue.clear();
    // Anything still painting for the old hole is dropped when it arrives.
    this.inFlight.clear();
  }

  private paintWorker(): Worker | null {
    if (this.worker !== undefined) return this.worker;
    this.worker = null;
    if (typeof Worker === 'undefined' || typeof OffscreenCanvas === 'undefined') return null;
    try {
      const worker = new Worker(new URL('../workers/paint.worker.ts', import.meta.url), {
        type: 'module',
      });
      worker.onmessage = (event: MessageEvent<PaintResponse>) => {
        const { key, bitmap, half } = event.data;
        const job = this.inFlight.get(key);
        if (!job) {
          bitmap?.close();
          return;
        }
        this.inFlight.delete(key);
        job.entry.set(job.bucket, bitmap ? { canvas: bitmap, half } : null);
        trim(job.entry);
      };
      worker.onerror = () => {
        // Fall back to painting here; nothing in flight will ever come back.
        this.worker = null;
        this.inFlight.clear();
      };
      this.worker = worker;
    } catch {
      this.worker = null;
    }
    return this.worker;
  }

  /** Repaints held back from earlier frames, until `budgetMs` is spent. */
  pump(budgetMs = 2): void {
    if (this.queue.size === 0) return;
    const start = performance.now();
    for (const [key, paint] of this.queue) {
      this.queue.delete(key);
      paint();
      if (performance.now() - start > budgetMs) break;
    }
  }

  /**
   * The sprite for a body at roughly this resolution. When the exact bucket is
   * not painted yet but another is, that one is returned scaled and the new
   * one is queued — so a zoom never stalls a frame on a fresh paint.
   * `immediate` paints synchronously regardless, for warming up on load.
   */
  sprite(
    id: string,
    style: BodyStyle,
    colors: BodyColors,
    radius: number,
    pixelsPerUnit: number,
    immediate = false,
  ): PlanetSprite | null {
    // Bucket the resolution in steps of ~19% so zooming reuses bitmaps.
    const bucket = Math.pow(1.19, Math.round(Math.log(Math.max(pixelsPerUnit, 0.05)) / Math.log(1.19)));
    const baseKey = `${id}|${style}|${radius}|${colors.fill}`;
    let entry = this.cache.get(baseKey);
    if (!entry) {
      entry = new Map();
      this.cache.set(baseKey, entry);
    }
    if (entry.has(bucket)) return entry.get(bucket) ?? null;

    const paintNow = (): PlanetSprite | null => {
      const sprite = paintSprite(id, style, colors, radius, bucket);
      entry.set(bucket, sprite);
      trim(entry);
      return sprite;
    };
    const queueKey = `${baseKey}|${bucket}`;

    let nearest: PlanetSprite | null = null;
    let nearestGap = Infinity;
    for (const [b, sprite] of entry) {
      if (!sprite) continue;
      const gap = Math.abs(Math.log(b / bucket));
      if (gap < nearestGap) {
        nearestGap = gap;
        nearest = sprite;
      }
    }
    // A big sprite goes to the worker; until it is back the caller gets the
    // nearest resolution there is, or nothing and draws the body live.
    const pixels = Math.ceil(radius * (1 + HALO) * 2 * bucket);
    const worker = pixels > BIG_SPRITE ? this.paintWorker() : null;
    if (worker) {
      if (!this.inFlight.has(queueKey)) {
        this.inFlight.set(queueKey, { entry, bucket });
        const request: PaintRequest = { key: queueKey, id, style, colors, radius, bucket };
        worker.postMessage(request);
      }
      return nearest;
    }
    if (immediate || !nearest) return paintNow();
    if (!this.queue.has(queueKey)) this.queue.set(queueKey, () => void paintNow());
    return nearest;
  }
}

export const paintSprite = (
  id: string,
  style: BodyStyle,
  colors: BodyColors,
  radius: number,
  bucket: number,
): PlanetSprite | null => {
  const half = radius * (1 + HALO);
  const size = Math.ceil(half * 2 * bucket);
  // Past this a sprite costs more memory than it saves; draw those live.
  if (size > 2048 || size < 4) return null;
  const canvas = createCanvas(size);
  const ctx = canvas?.getContext('2d') as CanvasRenderingContext2D | null;
  if (!canvas || !ctx) return null;
  ctx.scale(bucket, bucket);
  ctx.translate(half, half);
  paintBody(ctx, style, colors, radius, hashSeed(id));
  return { canvas, half };
};

/** Sprites wider than this many pixels are painted on the worker. */
const BIG_SPRITE = 520;

/** Keep a few resolutions per body; the rest are memory for nothing. */
const trim = (entry: Map<number, PlanetSprite | null>): void => {
  while (entry.size > 3) {
    const oldest = entry.keys().next().value as number;
    const sprite = entry.get(oldest);
    if (sprite && 'close' in sprite.canvas) sprite.canvas.close();
    entry.delete(oldest);
  }
};

/** Paints a body centred on the origin. */
const paintBody = (
  ctx: CanvasRenderingContext2D,
  style: BodyStyle,
  colors: BodyColors,
  r: number,
  seed: number,
): void => {
  const rng = new Rng(seed);

  // Atmosphere halo, outside the disc. Gas and ice worlds get a thicker one;
  // airless rock gets almost none, which is half of what makes it read as rock.
  const haze = style === 'planet' || style === 'ice' || style === 'goo' ? 0.85 : 0.35;
  if (haze > 0 && style !== 'crystal') {
    const halo = ctx.createRadialGradient(0, 0, r * 0.92, 0, 0, r * (1 + HALO));
    halo.addColorStop(0, withAlpha(colors.rim, 0.32 * haze));
    halo.addColorStop(1, withAlpha(colors.rim, 0));
    ctx.fillStyle = halo;
    ctx.beginPath();
    ctx.arc(0, 0, r * (1 + HALO), 0, TAU);
    ctx.fill();
  }

  ctx.save();
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, TAU);
  ctx.clip();

  // Base: brightest toward the light.
  const base = ctx.createRadialGradient(
    LIGHT.x * r * 0.45,
    LIGHT.y * r * 0.45,
    r * 0.05,
    0,
    0,
    r * 1.05,
  );
  base.addColorStop(0, lighten(colors.fill, 0.18));
  base.addColorStop(0.55, colors.fill);
  base.addColorStop(1, colors.fillDark);
  ctx.fillStyle = base;
  ctx.fillRect(-r, -r, r * 2, r * 2);

  switch (style) {
    case 'planet':
      paintBands(ctx, colors, r, rng);
      break;
    case 'sand':
      paintDunes(ctx, colors, r, rng);
      break;
    case 'moon':
    case 'asteroid':
      paintCraters(ctx, colors, r, rng, style === 'asteroid' ? 1.4 : 1);
      break;
    case 'ice':
      paintIce(ctx, colors, r, rng);
      break;
    case 'goo':
      paintGoo(ctx, colors, r, rng);
      break;
    case 'crystal':
      paintFacets(ctx, colors, r, rng);
      break;
    default:
      break;
  }

  // The terminator: the side facing away from the light falls into night.
  // This single gradient is what turns a flat disc into a sphere.
  const night = ctx.createRadialGradient(
    LIGHT.x * r * 0.55,
    LIGHT.y * r * 0.55,
    r * 0.55,
    LIGHT.x * r * 0.2,
    LIGHT.y * r * 0.2,
    r * 1.45,
  );
  night.addColorStop(0, 'rgba(2, 4, 16, 0)');
  night.addColorStop(1, 'rgba(2, 4, 16, 0.72)');
  ctx.fillStyle = night;
  ctx.fillRect(-r, -r, r * 2, r * 2);

  // Specular glint, strongest on the glossy materials.
  const gloss = style === 'ice' || style === 'crystal' || style === 'goo' ? 0.5 : 0.16;
  const glint = ctx.createRadialGradient(
    LIGHT.x * r * 0.5,
    LIGHT.y * r * 0.5,
    0,
    LIGHT.x * r * 0.5,
    LIGHT.y * r * 0.5,
    r * 0.42,
  );
  glint.addColorStop(0, `rgba(255,255,255,${gloss})`);
  glint.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = glint;
  ctx.fillRect(-r, -r, r * 2, r * 2);
  ctx.restore();

  // Rim light: a thin bright crescent on the lit limb only.
  ctx.save();
  ctx.lineWidth = Math.max(1, r * 0.045);
  const lightAngle = Math.atan2(LIGHT.y, LIGHT.x);
  const rim = ctx.createLinearGradient(LIGHT.x * r, LIGHT.y * r, -LIGHT.x * r, -LIGHT.y * r);
  rim.addColorStop(0, withAlpha(colors.rim, 0.95));
  rim.addColorStop(0.55, withAlpha(colors.rim, 0.12));
  rim.addColorStop(1, withAlpha(colors.rim, 0));
  ctx.strokeStyle = rim;
  ctx.beginPath();
  ctx.arc(0, 0, r - ctx.lineWidth / 2, lightAngle - 2.1, lightAngle + 2.1);
  ctx.stroke();
  ctx.restore();
};

/** Soft latitude bands, like weather on a gas or ocean world. */
const paintBands = (ctx: CanvasRenderingContext2D, colors: BodyColors, r: number, rng: Rng) => {
  const tilt = rng.range(-0.35, 0.35);
  ctx.save();
  ctx.rotate(tilt);
  const bands = rng.int(5, 8);
  for (let i = 0; i < bands; i++) {
    const y = rng.range(-r, r);
    const h = rng.range(0.05, 0.16) * r;
    ctx.fillStyle = withAlpha(rng.bool() ? colors.rim : colors.fillDark, rng.range(0.1, 0.24));
    ctx.beginPath();
    ctx.ellipse(rng.range(-0.15, 0.15) * r, y, r * 1.3, h, 0, 0, TAU);
    ctx.fill();
  }
  // A storm, now and then.
  if (rng.bool(0.6)) {
    const a = rng.range(0, TAU);
    const d = rng.range(0.2, 0.55) * r;
    ctx.fillStyle = withAlpha(colors.rim, 0.22);
    ctx.beginPath();
    ctx.ellipse(Math.cos(a) * d, Math.sin(a) * d, r * 0.16, r * 0.09, 0, 0, TAU);
    ctx.fill();
  }
  ctx.restore();
};

/** Wind-blown ridges for sand. */
const paintDunes = (ctx: CanvasRenderingContext2D, colors: BodyColors, r: number, rng: Rng) => {
  ctx.save();
  ctx.rotate(rng.range(-0.6, 0.6));
  ctx.lineWidth = Math.max(1, r * 0.035);
  for (let y = -r; y < r; y += r * rng.range(0.13, 0.2)) {
    ctx.strokeStyle = withAlpha(colors.fillDark, 0.32);
    ctx.beginPath();
    for (let x = -r; x <= r; x += r * 0.1) {
      const wave = Math.sin(x / (r * 0.18) + y) * r * 0.035;
      if (x === -r) ctx.moveTo(x, y + wave);
      else ctx.lineTo(x, y + wave);
    }
    ctx.stroke();
  }
  ctx.restore();
};

/** Craters with a lit rim and a shadowed floor, all facing the one light. */
const paintCraters = (
  ctx: CanvasRenderingContext2D,
  colors: BodyColors,
  r: number,
  rng: Rng,
  density: number,
) => {
  const count = Math.round(rng.int(5, 9) * density);
  for (let i = 0; i < count; i++) {
    const a = rng.range(0, TAU);
    const d = Math.sqrt(rng.next()) * r * 0.85;
    const x = Math.cos(a) * d;
    const y = Math.sin(a) * d;
    const cr = rng.range(0.05, 0.17) * r;
    ctx.fillStyle = withAlpha(colors.fillDark, 0.55);
    ctx.beginPath();
    ctx.arc(x, y, cr, 0, TAU);
    ctx.fill();
    // Lip on the far side catches the light; the near wall is in shade.
    ctx.strokeStyle = withAlpha(colors.rim, 0.4);
    ctx.lineWidth = Math.max(0.8, cr * 0.22);
    const away = Math.atan2(-LIGHT.y, -LIGHT.x);
    ctx.beginPath();
    ctx.arc(x, y, cr, away - 1.1, away + 1.1);
    ctx.stroke();
  }
  // Fine regolith speckle.
  for (let i = 0; i < 40; i++) {
    const a = rng.range(0, TAU);
    const d = Math.sqrt(rng.next()) * r;
    ctx.fillStyle = withAlpha(rng.bool() ? colors.rim : colors.fillDark, 0.14);
    ctx.fillRect(Math.cos(a) * d, Math.sin(a) * d, r * 0.02 + 0.6, r * 0.02 + 0.6);
  }
};

/** Pressure cracks across a frozen surface. */
const paintIce = (ctx: CanvasRenderingContext2D, colors: BodyColors, r: number, rng: Rng) => {
  ctx.strokeStyle = withAlpha(colors.rim, 0.45);
  ctx.lineWidth = Math.max(0.8, r * 0.018);
  for (let i = 0; i < 6; i++) {
    let x = rng.range(-r, r) * 0.7;
    let y = rng.range(-r, r) * 0.7;
    ctx.beginPath();
    ctx.moveTo(x, y);
    for (let j = 0; j < 4; j++) {
      x += rng.range(-0.3, 0.3) * r;
      y += rng.range(-0.3, 0.3) * r;
      ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
};

/** Glossy bubbles in something you would rather not touch. */
const paintGoo = (ctx: CanvasRenderingContext2D, colors: BodyColors, r: number, rng: Rng) => {
  for (let i = 0; i < 9; i++) {
    const a = rng.range(0, TAU);
    const d = Math.sqrt(rng.next()) * r * 0.8;
    const br = rng.range(0.06, 0.16) * r;
    ctx.fillStyle = withAlpha(colors.rim, 0.22);
    ctx.beginPath();
    ctx.arc(Math.cos(a) * d, Math.sin(a) * d, br, 0, TAU);
    ctx.fill();
  }
};

/** Cut facets, so a breakable block reads as glass rather than as rock. */
const paintFacets = (ctx: CanvasRenderingContext2D, colors: BodyColors, r: number, rng: Rng) => {
  const n = 7;
  const offset = rng.range(0, TAU);
  for (let i = 0; i < n; i++) {
    const a0 = offset + (i / n) * TAU;
    const a1 = offset + ((i + 1) / n) * TAU;
    const facing = Math.cos((a0 + a1) / 2 - Math.atan2(LIGHT.y, LIGHT.x));
    ctx.fillStyle = facing > 0 ? withAlpha('#ffffff', 0.1 + facing * 0.2) : withAlpha(colors.fillDark, -facing * 0.4);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(a0) * r * 1.1, Math.sin(a0) * r * 1.1);
    ctx.lineTo(Math.cos(a1) * r * 1.1, Math.sin(a1) * r * 1.1);
    ctx.closePath();
    ctx.fill();
  }
  ctx.strokeStyle = withAlpha(colors.rim, 0.5);
  ctx.lineWidth = Math.max(0.8, r * 0.025);
  ctx.beginPath();
  ctx.arc(0, 0, r * 0.5, 0, TAU);
  ctx.stroke();
};

/* ---------------------------------------------------------------- colour */

const parseHex = (hex: string): [number, number, number] | null => {
  const m = /^#([0-9a-f]{6})$/i.exec(hex.trim());
  if (!m) return null;
  const v = parseInt(m[1]!, 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
};

/** `color` at `alpha`, accepting a hex colour or an rgba() string. */
export const withAlpha = (color: string, alpha: number): string => {
  const a = Math.max(0, Math.min(1, alpha));
  const rgb = parseHex(color);
  if (rgb) return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${a})`;
  const m = /^rgba?\(([^)]+)\)$/.exec(color.trim());
  if (m) {
    const [r, g, b] = m[1]!.split(',').map((v) => v.trim());
    return `rgba(${r},${g},${b},${a})`;
  }
  return color;
};

/** Mixes a hex colour toward white by `amount`. */
export const lighten = (hex: string, amount: number): string => {
  const rgb = parseHex(hex);
  if (!rgb) return hex;
  const mix = (c: number) => Math.round(c + (255 - c) * amount);
  return `rgb(${mix(rgb[0])},${mix(rgb[1])},${mix(rgb[2])})`;
};
