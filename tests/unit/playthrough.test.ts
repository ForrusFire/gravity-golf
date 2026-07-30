import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { ALL_LEVELS } from '../../src/game/levels';
import { MemoryStorage, ProgressStore } from '../../src/game/progress';
import { PlaySession } from '../../src/game/session';
import { solveLevel, type Shot } from '../../src/game/solver';
import { isLevelUnlocked } from '../../src/ui/screens';

/** Runs the session until it needs input again, or the hole is finished. */
const settle = (session: PlaySession, maxSeconds = 40): void => {
  const steps = Math.round(maxSeconds * 60);
  for (let i = 0; i < steps; i++) {
    if (session.state === 'sunk') return;
    if (session.state === 'aiming' && session.canShoot) return;
    session.update(1 / 60);
  }
};

/** Plays a level through a real session using the given shots. */
const playShots = (session: PlaySession, shots: Shot[]): void => {
  for (const shot of shots) {
    settle(session);
    if (session.state === 'sunk') return;
    session.shoot(V.fromAngle(shot.angle), shot.power);
    settle(session);
  }
  settle(session);
};

describe('solver solutions replay through the real game', () => {
  // This is what makes "every hole is completable" mean anything. The solver
  // shares the game's integrator *and* its timestep, and starts from the same
  // settled tee, so a solution it finds is a sequence of shots a player could
  // actually take. Gravity slingshots are chaotic: when the solver ran at a
  // coarser step, only half its solutions survived being replayed here.
  for (const level of ALL_LEVELS) {
    it(`${level.id} "${level.name}" sinks when its solution is replayed`, () => {
      // A little wider than the default: the search stands in for a skilled
      // player, and a few holes have a solution that a coarser sweep misses.
      const solution = solveLevel(level, {
        angleSamples: 90,
        powerSamples: 6,
        maxStrokes: level.par,
        beamWidth: 6,
      });
      expect(solution.solved, `no solution found within par ${level.par}`).toBe(true);

      const session = new PlaySession(level);
      playShots(session, solution.shots);

      expect(session.state, `replaying ${solution.shots.length} shot(s) did not sink`).toBe('sunk');
      expect(session.result).not.toBeNull();
      expect(session.result!.strokes).toBeLessThanOrEqual(level.par);
    });
  }
});

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
