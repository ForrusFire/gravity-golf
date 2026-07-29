/** A 2D vector. Treated as immutable by every helper in this module. */
export interface Vec2 {
  readonly x: number;
  readonly y: number;
}

/** A mutable 2D vector, used in hot loops where allocation matters. */
export interface MutVec2 {
  x: number;
  y: number;
}

export const vec = (x: number, y: number): Vec2 => ({ x, y });
export const ZERO: Vec2 = { x: 0, y: 0 };

export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const mul = (a: Vec2, s: number): Vec2 => ({ x: a.x * s, y: a.y * s });
export const neg = (a: Vec2): Vec2 => ({ x: -a.x, y: -a.y });

/** a + b * s — the fused form avoids an intermediate allocation. */
export const addScaled = (a: Vec2, b: Vec2, s: number): Vec2 => ({
  x: a.x + b.x * s,
  y: a.y + b.y * s,
});

export const dot = (a: Vec2, b: Vec2): number => a.x * b.x + a.y * b.y;

/** 2D analogue of the cross product: the z component of a×b. */
export const cross = (a: Vec2, b: Vec2): number => a.x * b.y - a.y * b.x;

export const lengthSq = (a: Vec2): number => a.x * a.x + a.y * a.y;
export const length = (a: Vec2): number => Math.hypot(a.x, a.y);

export const distanceSq = (a: Vec2, b: Vec2): number => {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return dx * dx + dy * dy;
};

export const distance = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y);

/** Unit vector, or ZERO when the input has (near) zero length. */
export const normalize = (a: Vec2): Vec2 => {
  const len = Math.hypot(a.x, a.y);
  if (len < 1e-12) return ZERO;
  return { x: a.x / len, y: a.y / len };
};

/** Rescales to `max` only if longer than `max`. */
export const clampLength = (a: Vec2, max: number): Vec2 => {
  const len = Math.hypot(a.x, a.y);
  if (len <= max || len < 1e-12) return a;
  const s = max / len;
  return { x: a.x * s, y: a.y * s };
};

/** Rotate 90° counter-clockwise in a y-down screen space. */
export const perp = (a: Vec2): Vec2 => ({ x: -a.y, y: a.x });

export const rotate = (a: Vec2, radians: number): Vec2 => {
  const c = Math.cos(radians);
  const s = Math.sin(radians);
  return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
};

export const fromAngle = (radians: number, len = 1): Vec2 => ({
  x: Math.cos(radians) * len,
  y: Math.sin(radians) * len,
});

export const angleOf = (a: Vec2): number => Math.atan2(a.y, a.x);

export const lerpVec = (a: Vec2, b: Vec2, t: number): Vec2 => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});

/** Reflects `v` about the plane whose unit normal is `n`. */
export const reflect = (v: Vec2, n: Vec2): Vec2 => {
  const d = 2 * dot(v, n);
  return { x: v.x - d * n.x, y: v.y - d * n.y };
};

export const equalsApprox = (a: Vec2, b: Vec2, epsilon = 1e-9): boolean =>
  Math.abs(a.x - b.x) <= epsilon && Math.abs(a.y - b.y) <= epsilon;

export const clone = (a: Vec2): MutVec2 => ({ x: a.x, y: a.y });
