import { TAU } from '../core/math';
import { bodyShapeAt, worldVertices } from '../physics/geometry';
import type { Shape, Zone } from '../physics/types';
import type { LevelDef } from '../game/level';
import { colorsForBody, styleForBody } from './palette';

/** Zone tints, matching the in-game colour language at a whisper. */
const ZONE_TINT: Record<Zone['kind'], string> = {
  wind: 'rgba(120,200,255,0.16)',
  boost: 'rgba(166,242,89,0.18)',
  vortex: 'rgba(199,146,255,0.18)',
  nebula: 'rgba(110,140,220,0.16)',
  hazard: 'rgba(255,92,114,0.24)',
  gravityScale: 'rgba(255,120,200,0.16)',
};

const trace = (ctx: CanvasRenderingContext2D, shape: Shape): void => {
  ctx.beginPath();
  switch (shape.kind) {
    case 'circle':
      ctx.arc(shape.center.x, shape.center.y, shape.radius, 0, TAU);
      break;
    case 'capsule': {
      // Thick round-capped line: close enough at thumbnail scale.
      ctx.moveTo(shape.a.x, shape.a.y);
      ctx.lineTo(shape.b.x, shape.b.y);
      break;
    }
    case 'polygon': {
      const verts = worldVertices(shape);
      verts.forEach((v, i) => (i === 0 ? ctx.moveTo(v.x, v.y) : ctx.lineTo(v.x, v.y)));
      ctx.closePath();
      break;
    }
  }
};

const paint = (
  ctx: CanvasRenderingContext2D,
  shape: Shape,
  color: string | CanvasGradient,
): void => {
  trace(ctx, shape);
  if (shape.kind === 'capsule') {
    ctx.strokeStyle = color;
    ctx.lineWidth = shape.radius * 2;
    ctx.lineCap = 'round';
    ctx.stroke();
  } else {
    ctx.fillStyle = color;
    ctx.fill();
  }
};

/**
 * A little map of a hole: its worlds, hazards, tee and cup, fitted to the
 * canvas. Enough to recognise a hole at a glance on the select screen — the
 * names alone stop meaning anything after the first dozen.
 */
export const drawThumbnail = (canvas: HTMLCanvasElement, level: LevelDef, scale = 2): void => {
  const width = 160;
  const height = 90;
  canvas.width = Math.round(width * scale);
  canvas.height = Math.round(height * scale);
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const b = level.bounds;
  const pad = 30;
  const zoom = Math.min(
    canvas.width / (b.maxX - b.minX + pad * 2),
    canvas.height / (b.maxY - b.minY + pad * 2),
  );
  ctx.fillStyle = '#070917';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.translate(canvas.width / 2, canvas.height / 2);
  ctx.scale(zoom, zoom);
  ctx.translate(-(b.minX + b.maxX) / 2, -(b.minY + b.maxY) / 2);

  for (const zone of level.zones ?? []) paint(ctx, zone.area, ZONE_TINT[zone.kind]);

  for (const body of level.bodies ?? []) {
    const shape = bodyShapeAt(body, 0);
    const colors = colorsForBody(body.style, body.material);
    if (body.addedBy) {
      // Not there yet: an outline, as in the game.
      ctx.globalAlpha = 0.35;
      paint(ctx, shape, colors.rim);
      ctx.globalAlpha = 1;
      continue;
    }
    if (shape.kind === 'circle' && styleForBody(body.style, body.material) !== 'wall') {
      const r = shape.radius;
      const grad = ctx.createRadialGradient(
        shape.center.x - r * 0.35,
        shape.center.y - r * 0.4,
        r * 0.1,
        shape.center.x,
        shape.center.y,
        r,
      );
      grad.addColorStop(0, colors.fill);
      grad.addColorStop(1, colors.fillDark);
      paint(ctx, shape, grad);
    } else {
      paint(ctx, shape, colors.fill);
    }
  }

  // Portals as linked violet rings, boost rings as lime ones.
  {
    const px0 = 1 / zoom;
    ctx.lineWidth = 1.6 * px0 * scale;
    for (const portal of level.portals ?? []) {
      ctx.strokeStyle = 'rgba(199,146,255,0.35)';
      ctx.beginPath();
      ctx.moveTo(portal.from.x, portal.from.y);
      ctx.lineTo(portal.to.x, portal.to.y);
      ctx.stroke();
      ctx.strokeStyle = '#c792ff';
      for (const end of [portal.from, portal.to]) {
        ctx.beginPath();
        ctx.arc(end.x, end.y, Math.max(portal.radius, 4 * px0 * scale), 0, TAU);
        ctx.stroke();
      }
    }
    ctx.strokeStyle = '#a6f259';
    for (const boost of level.boosters ?? []) {
      ctx.beginPath();
      ctx.arc(boost.position.x, boost.position.y, Math.max(boost.radius, 4 * px0 * scale), 0, TAU);
      ctx.stroke();
    }
  }

  // Stars, tee and cup sized in screen pixels so they read at any hole size.
  const px = 1 / zoom;
  ctx.fillStyle = '#ffd25a';
  for (const star of level.stars ?? []) {
    ctx.beginPath();
    ctx.arc(star.x, star.y, 2.6 * px * scale, 0, TAU);
    ctx.fill();
  }
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(level.tee.x, level.tee.y, 3 * px * scale, 0, TAU);
  ctx.fill();

  const cup = level.hole.position;
  ctx.strokeStyle = '#66e6b8';
  ctx.lineWidth = 1.6 * px * scale;
  ctx.beginPath();
  ctx.arc(cup.x, cup.y, 5 * px * scale, 0, TAU);
  ctx.stroke();
  ctx.fillStyle = '#66e6b8';
  ctx.beginPath();
  ctx.arc(cup.x, cup.y, 2 * px * scale, 0, TAU);
  ctx.fill();
};
