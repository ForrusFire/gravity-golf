/// <reference lib="webworker" />
import { generateLevel } from '../game/generator';
import { findHint, type HintRequest, type HintResult } from '../game/hint';

export interface GenerateRequest {
  requestId: number;
  kind?: 'generate';
  seed: number;
}

export interface HintWorkRequest {
  requestId: number;
  kind: 'hint';
  hint: HintRequest;
}

export type WorkerRequest = GenerateRequest | HintWorkRequest;

export interface GenerateResponse {
  requestId: number;
  kind?: 'generate';
  level: unknown | null;
  attempts: number;
  error?: string;
}

export interface HintResponse {
  requestId: number;
  kind: 'hint';
  result: HintResult | null;
  error?: string;
}

export type WorkerResponse = GenerateResponse | HintResponse;

const post = (message: WorkerResponse): void =>
  (self as unknown as Worker).postMessage(message);

/**
 * Both jobs here mean running the solver — generating a hole until a candidate
 * passes, or searching for a playable line from where the ball is standing. Each
 * takes a second or more, which on the main thread is a frozen page. The game
 * modules are free of DOM access, so they import into a worker unchanged.
 */
self.onmessage = (event: MessageEvent<WorkerRequest>) => {
  const request = event.data;
  if (request.kind === 'hint') {
    try {
      post({ requestId: request.requestId, kind: 'hint', result: findHint(request.hint) });
    } catch (error) {
      post({
        requestId: request.requestId,
        kind: 'hint',
        result: null,
        error: error instanceof Error ? error.message : String(error),
      });
    }
    return;
  }

  try {
    const result = generateLevel(request.seed);
    post({ requestId: request.requestId, level: result.level, attempts: result.attempts });
  } catch (error) {
    post({
      requestId: request.requestId,
      level: null,
      attempts: 0,
      error: error instanceof Error ? error.message : String(error),
    });
  }
};
