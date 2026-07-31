import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { ALL_LEVELS } from '../../src/game/levels';
import { MemoryStorage, ProgressStore } from '../../src/game/progress';
import { PlaySession } from '../../src/game/session';
import { solveLevel } from '../../src/game/solver';
import { isLevelUnlocked } from '../../src/ui/screens';
import { playShots, settle } from './level-gates';

describe('campaign progression', () => {
  it('unlocks the next hole each time one is completed', () => {
    const progress = new ProgressStore(new MemoryStorage());

    // Only the first hole is open on a fresh save.
    expect(isLevelUnlocked(0, ALL_LEVELS, progress)).toBe(true);
    expect(isLevelUnlocked(1, ALL_LEVELS, progress)).toBe(false);

    for (let i = 0; i < 4; i++) {
      const level = ALL_LEVELS[i]!;
      expect(isLevelUnlocked(i, ALL_LEVELS, progress)).toBe(true);

      const solution = solveLevel(level, { maxStrokes: level.par });
      expect(solution.solved).toBe(true);

      const session = new PlaySession(level);
      playShots(session, solution.shots);
      expect(session.result).not.toBeNull();
      progress.submit(session.result!);

      expect(progress.isCompleted(level.id)).toBe(true);
      expect(isLevelUnlocked(i + 1, ALL_LEVELS, progress)).toBe(true);
    }

    expect(progress.completedCount()).toBe(4);
    // A hole further down the list is still locked.
    expect(isLevelUnlocked(6, ALL_LEVELS, progress)).toBe(false);
  });

  it('keeps a best score across repeated plays of the same hole', () => {
    const level = ALL_LEVELS[0]!;
    const progress = new ProgressStore(new MemoryStorage());
    const solution = solveLevel(level, { maxStrokes: level.par });
    expect(solution.solved).toBe(true);

    const first = new PlaySession(level);
    playShots(first, solution.shots);
    progress.submit(first.result!);
    const best = progress.recordOf(level.id)!.bestStrokes;

    // Play it badly: waste a stroke before taking the solution.
    const second = new PlaySession(level);
    second.shoot(V.vec(-1, 0), 0.2);
    settle(second);
    playShots(second, solution.shots);
    if (second.result) progress.submit(second.result);

    expect(progress.recordOf(level.id)!.bestStrokes).toBe(best);
    expect(progress.recordOf(level.id)!.attempts).toBeGreaterThanOrEqual(1);
  });
});
