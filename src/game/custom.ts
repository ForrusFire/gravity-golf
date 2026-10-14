import { vec } from '../core/vec2';
import {
  blackHole,
  bumper,
  planet,
  rock,
  sun,
  wall,
  type LevelDef,
} from './level';
import type { Body } from '../physics/types';

/**
 * Player-made holes.
 *
 * Kept as their own small format rather than as a `LevelDef`: it is what goes
 * into a link, so it should be short, and it is what comes *out* of a link, so
 * every field is checked and clamped on the way in. A shared hole can only ever
 * describe things the editor could have made.
 */

/** The editing area, the same size as most campaign holes. */
export const CUSTOM_BOUNDS = { minX: -720, minY: -440, maxX: 720, maxY: 440 };

export type CustomBodyKind =
  | 'planet'
  | 'moon'
  | 'ice'
  | 'sand'
  | 'goo'
  | 'sun'
  | 'blackhole'
  | 'bumper'
  | 'rock';

export type CustomItem =
  | { t: CustomBodyKind; x: number; y: number; r: number }
  | { t: 'wall'; x1: number; y1: number; x2: number; y2: number };

export interface CustomHole {
  v: 1;
  name: string;
  par: number;
  tee: [number, number];
  cup: [number, number];
  items: CustomItem[];
  stars: Array<[number, number]>;
  /** A plain "down" everywhere, for holes built on floors and walls. */
  down: boolean;
  /** Bounce off the edges instead of being lost past them. */
  walls: boolean;
}

export const CUSTOM_LIMITS = {
  items: 40,
  stars: 3,
  nameLength: 28,
  minRadius: 12,
  maxRadius: 220,
  minPar: 1,
  maxPar: 9,
};

export const BODY_KINDS: readonly CustomBodyKind[] = [
  'planet',
  'moon',
  'ice',
  'sand',
  'goo',
  'sun',
  'blackhole',
  'bumper',
  'rock',
];

export const emptyCustomHole = (): CustomHole => ({
  v: 1,
  name: 'My hole',
  par: 3,
  // Resting on the first world, and the cup sat on the second.
  tee: [-560, 42],
  cup: [560, 37],
  items: [
    { t: 'planet', x: -560, y: 140, r: 90 },
    { t: 'sand', x: 560, y: 110, r: 70 },
  ],
  stars: [],
  down: false,
  walls: false,
});

/* ------------------------------------------------------------ to a hole */

const bodyFor = (item: CustomItem, index: number): Body => {
  const id = `custom-${index}`;
  if (item.t === 'wall') {
    return wall(id, vec(item.x1, item.y1), vec(item.x2, item.y2), 10);
  }
  const at = vec(item.x, item.y);
  const r = item.r;
  // Gravity grows with size, but gently: a big world should pull harder, not
  // swallow the whole hole.
  const reach = (k: number) => Math.min(900, r * k + 120);
  switch (item.t) {
    case 'planet':
      return planet(id, at, r, 760, { range: reach(4.2) });
    case 'moon':
      return planet(id, at, r, 600, { range: reach(3.6), style: 'moon' });
    case 'ice':
      return planet(id, at, r, 680, { range: reach(3.8), material: 'ice', style: 'ice' });
    case 'sand':
      return planet(id, at, r, 640, { range: reach(3.6), material: 'sand', style: 'sand' });
    case 'goo':
      return planet(id, at, r, 620, { range: reach(3.4), material: 'sticky', style: 'goo' });
    case 'sun':
      return sun(id, at, r, 420, { range: reach(4) });
    case 'blackhole':
      return blackHole(id, at, r, 1300, { range: reach(7) });
    case 'bumper':
      return bumper(id, at, r);
    case 'rock':
      return rock(id, at, r, { style: 'asteroid' });
  }
};

/** Builds a playable hole from a custom one. */
export const customToLevel = (hole: CustomHole): LevelDef & { custom: CustomHole } => ({
  id: 'custom',
  name: hole.name || 'Custom hole',
  chapter: 0,
  par: hole.par,
  tee: vec(hole.tee[0], hole.tee[1]),
  hole: { position: vec(hole.cup[0], hole.cup[1]) },
  bounds: CUSTOM_BOUNDS,
  boundsMode: hole.walls ? 'wall' : 'kill',
  ambientDrag: 0.12,
  uniformGravity: hole.down ? vec(0, 340) : undefined,
  bodies: hole.items.map(bodyFor),
  stars: hole.stars.map(([x, y]) => vec(x, y)),
  custom: hole,
});

/* ----------------------------------------------------------- sanitising */

const num = (raw: unknown, min: number, max: number, fallback: number): number =>
  typeof raw === 'number' && Number.isFinite(raw) ? Math.min(max, Math.max(min, raw)) : fallback;

const xOf = (raw: unknown) => Math.round(num(raw, CUSTOM_BOUNDS.minX, CUSTOM_BOUNDS.maxX, 0));
const yOf = (raw: unknown) => Math.round(num(raw, CUSTOM_BOUNDS.minY, CUSTOM_BOUNDS.maxY, 0));

const point = (raw: unknown): [number, number] | null =>
  Array.isArray(raw) && raw.length === 2 ? [xOf(raw[0]), yOf(raw[1])] : null;

/** Turns anything into a valid custom hole, or null when it is not one at all. */
export const sanitizeCustomHole = (raw: unknown): CustomHole | null => {
  if (typeof raw !== 'object' || raw === null) return null;
  const data = raw as Record<string, unknown>;
  const tee = point(data.tee);
  const cup = point(data.cup);
  if (!tee || !cup) return null;

  const items: CustomItem[] = [];
  for (const entry of Array.isArray(data.items) ? data.items.slice(0, CUSTOM_LIMITS.items) : []) {
    if (typeof entry !== 'object' || entry === null) continue;
    const item = entry as Record<string, unknown>;
    if (item.t === 'wall') {
      items.push({ t: 'wall', x1: xOf(item.x1), y1: yOf(item.y1), x2: xOf(item.x2), y2: yOf(item.y2) });
    } else if (typeof item.t === 'string' && (BODY_KINDS as readonly string[]).includes(item.t)) {
      items.push({
        t: item.t as CustomBodyKind,
        x: xOf(item.x),
        y: yOf(item.y),
        r: Math.round(num(item.r, CUSTOM_LIMITS.minRadius, CUSTOM_LIMITS.maxRadius, 40)),
      });
    }
  }

  const stars: Array<[number, number]> = [];
  for (const entry of Array.isArray(data.stars) ? data.stars.slice(0, CUSTOM_LIMITS.stars) : []) {
    const p = point(entry);
    if (p) stars.push(p);
  }

  const name =
    typeof data.name === 'string'
      ? data.name.replace(/[\u0000-\u001f]/g, '').trim().slice(0, CUSTOM_LIMITS.nameLength)
      : '';

  return {
    v: 1,
    name: name || 'Custom hole',
    par: Math.round(num(data.par, CUSTOM_LIMITS.minPar, CUSTOM_LIMITS.maxPar, 3)),
    tee,
    cup,
    items,
    stars,
    down: data.down === true,
    walls: data.walls === true,
  };
};

/* ------------------------------------------------------------- encoding */

const toBase64Url = (text: string): string => {
  const bytes = new TextEncoder().encode(text);
  let binary = '';
  for (const b of bytes) binary += String.fromCharCode(b);
  return btoa(binary).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
};

const fromBase64Url = (code: string): string => {
  const padded = code.replace(/-/g, '+').replace(/_/g, '/');
  const binary = atob(padded + '='.repeat((4 - (padded.length % 4)) % 4));
  const bytes = Uint8Array.from(binary, (c) => c.charCodeAt(0));
  return new TextDecoder().decode(bytes);
};

/** The link-safe code for a custom hole. */
export const encodeCustomHole = (hole: CustomHole): string => toBase64Url(JSON.stringify(hole));

/** Reads a code back, or null when it is not a hole. Never throws. */
export const decodeCustomHole = (code: string): CustomHole | null => {
  if (!/^[A-Za-z0-9_-]{8,6000}$/.test(code)) return null;
  try {
    return sanitizeCustomHole(JSON.parse(fromBase64Url(code)));
  } catch {
    return null;
  }
};
