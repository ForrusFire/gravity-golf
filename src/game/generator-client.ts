import { generateLevel, type GeneratedLevel } from './generator';
import type { GenerateResponse } from '../workers/generate.worker';

const CACHE_PREFIX = 'gravity-golf/generated/';

/** Reads a previously generated hole for `key`, if one was stored. */
export const loadCached = (key: string): GeneratedLevel | null => {
  try {
    const raw = localStorage.getItem(CACHE_PREFIX + key);
    if (!raw) return null;
    const parsed = JSON.parse(raw) as GeneratedLevel;
    // A stored hole from an older build may not match the current shape.
    if (!parsed?.id || !parsed.tee || !parsed.hole?.position || !Array.isArray(parsed.bodies)) {
      return null;
    }
    return parsed;
  } catch {
    return null;
  }
};

const storeCached = (key: string, level: GeneratedLevel): void => {
  try {
    localStorage.setItem(CACHE_PREFIX + key, JSON.stringify(level));
  } catch {
    // Storage full or blocked; the hole simply regenerates next time.
  }
};

/**
 * Generates a hole off the main thread, falling back to synchronous generation
 * where workers are unavailable. Results are cached by `cacheKey` so the daily
 * challenge is only ever generated once per day per device.
 */
export class LevelGenerator {
  private worker: Worker | null = null;
  private nextRequestId = 1;
  private pending = new Map<number, (level: GeneratedLevel | null) => void>();

  private ensureWorker(): Worker | null {
    if (this.worker) return this.worker;
    if (typeof Worker === 'undefined') return null;
    try {
      this.worker = new Worker(new URL('../workers/generate.worker.ts', import.meta.url), {
        type: 'module',
      });
      this.worker.onmessage = (event: MessageEvent<GenerateResponse>) => {
        const resolve = this.pending.get(event.data.requestId);
        if (!resolve) return;
        this.pending.delete(event.data.requestId);
        resolve((event.data.level as GeneratedLevel) ?? null);
      };
      this.worker.onerror = () => {
        // Fail every outstanding request rather than hanging the UI forever.
        for (const resolve of this.pending.values()) resolve(null);
        this.pending.clear();
        this.worker?.terminate();
        this.worker = null;
      };
      return this.worker;
    } catch {
      return null;
    }
  }

  async generate(seed: number, cacheKey?: string): Promise<GeneratedLevel | null> {
    if (cacheKey) {
      const cached = loadCached(cacheKey);
      if (cached) return cached;
    }

    const level = await this.run(seed);
    if (level && cacheKey) storeCached(cacheKey, level);
    return level;
  }

  private run(seed: number): Promise<GeneratedLevel | null> {
    const worker = this.ensureWorker();
    if (!worker) {
      // No worker available: generate inline. Slower and blocking, but a
      // missing feature is worse than a brief stall.
      return Promise.resolve(generateLevel(seed).level);
    }

    const requestId = this.nextRequestId++;
    return new Promise((resolve) => {
      this.pending.set(requestId, resolve);
      worker.postMessage({ requestId, seed });
    });
  }

  dispose(): void {
    this.worker?.terminate();
    this.worker = null;
    this.pending.clear();
  }
}
