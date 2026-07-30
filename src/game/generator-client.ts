import { generateLevel, type GeneratedLevel } from './generator';
import { findHint, type HintRequest, type HintResult } from './hint';
import type { WorkerResponse } from '../workers/generate.worker';

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
 * Runs the solver off the main thread — generating a hole, or finding a hint —
 * falling back to synchronous work where workers are unavailable. Generated
 * holes are cached by `cacheKey` so the daily challenge is only ever generated
 * once per day per device.
 */
export class LevelGenerator {
  private worker: Worker | null = null;
  private nextRequestId = 1;
  private pending = new Map<number, (value: never) => void>();

  private ensureWorker(): Worker | null {
    if (this.worker) return this.worker;
    if (typeof Worker === 'undefined') return null;
    try {
      this.worker = new Worker(new URL('../workers/generate.worker.ts', import.meta.url), {
        type: 'module',
      });
      this.worker.onmessage = (event: MessageEvent<WorkerResponse>) => {
        const message = event.data;
        const resolve = this.pending.get(message.requestId) as
          | ((value: unknown) => void)
          | undefined;
        if (!resolve) return;
        this.pending.delete(message.requestId);
        resolve(message.kind === 'hint' ? message.result : (message.level ?? null));
      };
      this.worker.onerror = () => {
        // Fail every outstanding request rather than hanging the UI forever.
        for (const resolve of this.pending.values()) (resolve as (v: unknown) => void)(null);
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
    return this.dispatch(worker, (requestId) => ({ requestId, kind: 'generate', seed }));
  }

  /** The best next shot from where the ball is, or null if the search found none. */
  hint(request: HintRequest): Promise<HintResult | null> {
    const worker = this.ensureWorker();
    if (!worker) return Promise.resolve(findHint(request));
    return this.dispatch(worker, (requestId) => ({ requestId, kind: 'hint', hint: request }));
  }

  private dispatch<T>(worker: Worker, build: (requestId: number) => unknown): Promise<T | null> {
    const requestId = this.nextRequestId++;
    return new Promise((resolve) => {
      this.pending.set(requestId, resolve as (value: never) => void);
      worker.postMessage(build(requestId));
    });
  }

  dispose(): void {
    this.worker?.terminate();
    this.worker = null;
    this.pending.clear();
  }
}
