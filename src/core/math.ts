export const TAU = Math.PI * 2;

export const clamp = (v: number, min: number, max: number): number =>
  v < min ? min : v > max ? max : v;

export const clamp01 = (v: number): number => clamp(v, 0, 1);

export const lerp = (a: number, b: number, t: number): number => a + (b - a) * t;

/** Inverse lerp; returns 0 when the range is degenerate. */
export const invLerp = (a: number, b: number, v: number): number =>
  Math.abs(b - a) < 1e-12 ? 0 : (v - a) / (b - a);

export const remap = (v: number, inA: number, inB: number, outA: number, outB: number): number =>
  lerp(outA, outB, clamp01(invLerp(inA, inB, v)));

export const smoothstep = (t: number): number => {
  const x = clamp01(t);
  return x * x * (3 - 2 * x);
};

export const easeOutCubic = (t: number): number => 1 - Math.pow(1 - clamp01(t), 3);
export const easeInCubic = (t: number): number => Math.pow(clamp01(t), 3);
export const easeOutBack = (t: number): number => {
  const c1 = 1.70158;
  const c3 = c1 + 1;
  const x = clamp01(t);
  return 1 + c3 * Math.pow(x - 1, 3) + c1 * Math.pow(x - 1, 2);
};
export const easeInOutQuad = (t: number): number => {
  const x = clamp01(t);
  return x < 0.5 ? 2 * x * x : 1 - Math.pow(-2 * x + 2, 2) / 2;
};

/** Frame-rate independent exponential approach. `rate` is the fraction closed per second. */
export const damp = (current: number, target: number, rate: number, dt: number): number =>
  lerp(current, target, 1 - Math.exp(-rate * dt));

/** Wraps an angle into (-PI, PI]. */
export const wrapAngle = (a: number): number => {
  let x = (a + Math.PI) % TAU;
  if (x <= 0) x += TAU;
  return x - Math.PI;
};

/** Shortest signed angular delta from `a` to `b`. */
export const angleDelta = (a: number, b: number): number => wrapAngle(b - a);

export const sign = (v: number): number => (v > 0 ? 1 : v < 0 ? -1 : 0);

export const approxEqual = (a: number, b: number, epsilon = 1e-9): boolean =>
  Math.abs(a - b) <= epsilon;

export const roundTo = (v: number, decimals: number): number => {
  const f = Math.pow(10, decimals);
  return Math.round(v * f) / f;
};
