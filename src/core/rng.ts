/**
 * Deterministic PRNG (mulberry32). Seeded generators keep level decoration,
 * particle jitter and tests reproducible across runs and machines.
 */
export class Rng {
  private state: number;

  constructor(seed = 0x9e3779b9) {
    // Avoid the degenerate all-zero state.
    this.state = (seed >>> 0) || 0x9e3779b9;
  }

  /** Uniform in [0, 1). */
  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  /** Uniform in [min, max). */
  range(min: number, max: number): number {
    return min + this.next() * (max - min);
  }

  /** Uniform integer in [min, max]. */
  int(min: number, max: number): number {
    return Math.floor(this.range(min, max + 1));
  }

  bool(probability = 0.5): boolean {
    return this.next() < probability;
  }

  pick<T>(items: readonly T[]): T {
    if (items.length === 0) throw new Error('Rng.pick: empty array');
    return items[this.int(0, items.length - 1)]!;
  }

  /** Approximately normal via the sum of 3 uniforms (Bates), mean 0, sd ~1. */
  gaussian(): number {
    return (this.next() + this.next() + this.next() - 1.5) * 1.9556;
  }
}

/** Stable string -> 32-bit seed, so level ids can drive deterministic decoration. */
export const hashSeed = (text: string): number => {
  let h = 2166136261 >>> 0;
  for (let i = 0; i < text.length; i++) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return h >>> 0;
};
