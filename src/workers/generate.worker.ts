/// <reference lib="webworker" />
import { generateLevel } from '../game/generator';

export interface GenerateRequest {
  requestId: number;
  seed: number;
}

export interface GenerateResponse {
  requestId: number;
  level: unknown | null;
  attempts: number;
  error?: string;
}

/**
 * Generating a hole means running the solver until a candidate passes, which
 * takes a second or two. On the main thread that is a frozen page, so it runs
 * here instead. The game modules are free of DOM access, so they import into a
 * worker unchanged.
 */
self.onmessage = (event: MessageEvent<GenerateRequest>) => {
  const { requestId, seed } = event.data;
  try {
    const result = generateLevel(seed);
    const response: GenerateResponse = {
      requestId,
      level: result.level,
      attempts: result.attempts,
    };
    (self as unknown as Worker).postMessage(response);
  } catch (error) {
    const response: GenerateResponse = {
      requestId,
      level: null,
      attempts: 0,
      error: error instanceof Error ? error.message : String(error),
    };
    (self as unknown as Worker).postMessage(response);
  }
};
