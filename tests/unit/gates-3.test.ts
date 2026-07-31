import {
  SHARDS,
  registerForgivenessGate,
  registerReplayGate,
  registerStarGate,
  shardOf,
} from './level-gates';

// One slice of the per-level verification gates. See level-gates.ts for why
// these are split across files rather than looped in one.
const SLICE = `shard 3/${SHARDS}`;
const LEVELS = shardOf(2);

registerReplayGate(SLICE, LEVELS);
registerStarGate(SLICE, LEVELS);
registerForgivenessGate(SLICE, LEVELS);
