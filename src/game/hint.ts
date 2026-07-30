import type { LevelDef } from './level';
import { boardStateOf, solveLevel, type BoardState, type Shot } from './solver';
import type { World } from '../physics/world';
import type { Vec2 } from '../core/vec2';

/**
 * A board snapshot flattened for `postMessage`. `BoardState` holds Sets, which
 * structured clone would survive but JSON round-trips would not, and the hint
 * has to work identically whether it ran in a worker or fell back to inline.
 */
export interface HintRequestState {
  collected: string[];
  switchesOn: string[];
  switchTimers: Record<string, number>;
  breakables: Record<string, number>;
}

export interface HintRequest {
  level: LevelDef;
  position: Vec2;
  time: number;
  state: HintRequestState;
  /** Strokes the hint may spend. Usually what is left before par. */
  maxStrokes: number;
}

export interface HintResult {
  /** The shot to play now, or null when the search found nothing at all. */
  shot: Shot | null;
  /**
   * True when this shot is the first of a line that actually sinks. False means
   * it is the best progress the search found — worth playing, not a promise.
   */
  sinks: boolean;
  /** Strokes the found solution needs in total, 0 when it does not sink. */
  strokes: number;
}

/**
 * How long the hint search may run. Past a couple of seconds a player assumes it
 * is broken, and the search has a usable answer long before it has an optimal
 * one, so the ceiling costs less than the wait would.
 */
const HINT_BUDGET_MS = 2200;

export const serializeBoardState = (world: World): HintRequestState => {
  const state = boardStateOf(world);
  return {
    collected: [...state.collected],
    switchesOn: [...state.switchesOn],
    switchTimers: { ...state.switchTimers },
    breakables: { ...state.breakables },
  };
};

const deserializeBoardState = (state: HintRequestState): BoardState => ({
  collected: new Set(state.collected),
  switchesOn: new Set(state.switchesOn),
  switchTimers: { ...state.switchTimers },
  breakables: { ...state.breakables },
});

/**
 * The best next shot from where the ball is standing.
 *
 * Runs at the gameplay timestep rather than a cheaper one. A hint is a promise
 * that this exact aim and power leads somewhere, and the player replays it from
 * exactly this position — at a coarser step the ball would diverge and the
 * promise would be false, which is worse than offering no hint at all. Sampling
 * is trimmed instead, since a person is waiting.
 */
export const findHint = (request: HintRequest): HintResult => {
  const result = solveLevel(request.level, {
    angleSamples: 72,
    powerSamples: 5,
    minPower: 0.15,
    // Two strokes is the useful horizon. The hint only has to name the *next*
    // shot, and knowing it sets up a second one is enough to trust it; a third
    // ply more than doubles the wait for an answer the player will not use.
    maxStrokes: Math.max(1, Math.min(2, request.maxStrokes)),
    beamWidth: 3,
    shotTime: 10,
    timeBudgetMs: HINT_BUDGET_MS,
    from: {
      position: request.position,
      time: request.time,
      state: deserializeBoardState(request.state),
    },
  });
  if (result.solved) {
    return { shot: result.shots[0] ?? null, sinks: true, strokes: result.strokes };
  }
  return { shot: result.bestEffort[0] ?? null, sinks: false, strokes: 0 };
};
