import { describe, expect, it } from 'vitest';
import * as V from '../../src/core/vec2';
import { findHint, serializeBoardState } from '../../src/game/hint';
import { compileLevel, type LevelDef } from '../../src/game/level';
import { ALL_LEVELS, levelById } from '../../src/game/levels';
import { PlaySession, settledTee } from '../../src/game/session';
import { DEFAULT_SOLVE_OPTIONS, boardStateOf, solveLevel } from '../../src/game/solver';

/** Plays until the session wants input again, or the hole is finished. */
const settle = (session: PlaySession, maxSeconds = 40): void => {
  const steps = Math.round(maxSeconds * 60);
  for (let i = 0; i < steps; i++) {
    if (session.state === 'sunk') return;
    if (session.state === 'aiming' && session.canShoot) return;
    session.update(1 / 60);
  }
};

const hintFrom = (level: LevelDef, session: PlaySession, maxStrokes = 3) =>
  findHint({
    level,
    position: session.ball.position,
    time: session.world.time,
    state: serializeBoardState(session.world),
    maxStrokes,
  });

describe('findHint', () => {
  it('finds a line from the tee that sinks when it is actually played', () => {
    // The whole promise of a hint: play this exact aim and power and the ball
    // goes where it said. It only holds because the search runs at the
    // gameplay timestep — a coarser one would diverge within a shot.
    const level = levelById('c1-1')!;
    const session = new PlaySession(level);
    settle(session);

    const hint = hintFrom(level, session);
    expect(hint.shot).not.toBeNull();
    expect(hint.sinks).toBe(true);
    expect(hint.strokes).toBeGreaterThan(0);

    for (let stroke = 0; stroke < hint.strokes + 1; stroke++) {
      if (session.state === 'sunk') break;
      settle(session);
      const next = hintFrom(level, session);
      if (!next.shot) break;
      session.shoot(V.fromAngle(next.shot.angle), next.shot.power);
      for (let i = 0; i < 2400 && session.state === 'flying'; i++) session.update(1 / 60);
    }
    for (let i = 0; i < 2400 && session.state === 'flying'; i++) session.update(1 / 60);
    expect(session.state).toBe('sunk');
  });

  it('searches from where the ball is, not from the tee', () => {
    const level = levelById('c1-3')!;
    const session = new PlaySession(level);
    settle(session);
    const atTee = hintFrom(level, session);

    // Move the ball somewhere else entirely and ask again.
    const moved = findHint({
      level,
      position: V.vec(level.hole.position.x - 160, level.hole.position.y - 160),
      time: 0,
      state: serializeBoardState(session.world),
      maxStrokes: 3,
    });

    expect(atTee.shot).not.toBeNull();
    expect(moved.shot).not.toBeNull();
    expect(moved.shot!.angle).not.toBeCloseTo(atTee.shot!.angle, 3);
  });

  it('respects board state, so a thrown switch stays thrown', () => {
    const level = levelById('c6-1')!;
    const world = compileLevel(level);
    const before = serializeBoardState(world);
    expect(before.switchesOn).toEqual([]);

    world.switches[0]!.on = true;
    const after = serializeBoardState(world);
    expect(after.switchesOn).toEqual(['sw-door']);

    // With the gate open the search is solving a different hole, and should say
    // so rather than replaying a line that assumes a wall in the way.
    const opened = findHint({
      level,
      position: settledTee(level),
      time: 0,
      state: after,
      maxStrokes: 3,
    });
    expect(opened.shot).not.toBeNull();
  });

  it('offers the best line on when nothing sinks in the strokes allowed', () => {
    const level = levelById('c5-6')!;
    const session = new PlaySession(level);
    settle(session);
    // One stroke on a par-5 finale: there is no winning line, but standing
    // still is not advice.
    const hint = hintFrom(level, session, 1);
    expect(hint.shot).not.toBeNull();
    if (!hint.sinks) expect(hint.strokes).toBe(0);
  });

  it('never claims a sink it did not find', () => {
    const level = levelById('c2-1')!;
    const session = new PlaySession(level);
    settle(session);
    const hint = hintFrom(level, session);
    // sinks and strokes have to agree — a UI that says "sinks it in 0" is a bug.
    expect(hint.sinks).toBe(hint.strokes > 0);
  });
});

describe('the hint always has something to say', () => {
  it('offers a shot on every hole in the game', () => {
    // Two holes used to return nothing at all: they have nothing to land on, so
    // no shot ever came to rest and the beam emptied on the first stroke. A
    // hint button that silently does nothing on some holes is worse than none.
    const silent: string[] = [];
    for (const level of ALL_LEVELS) {
      const world = compileLevel(level);
      const hint = findHint({
        level,
        position: settledTee(level),
        time: 0,
        state: serializeBoardState(world),
        maxStrokes: 1,
      });
      if (!hint.shot) silent.push(level.id);
    }
    expect(silent).toEqual([]);
  }, 120000);

  it('answers within its time budget even on the heaviest hole', () => {
    const level = levelById('c8-6')!;
    const world = compileLevel(level);
    const started = performance.now();
    const hint = findHint({
      level,
      position: settledTee(level),
      time: 0,
      state: serializeBoardState(world),
      maxStrokes: 3,
    });
    const elapsed = performance.now() - started;
    expect(hint.shot).not.toBeNull();
    // The 2.2s budget plus room for the shot in flight when the clock runs out.
    expect(elapsed).toBeLessThan(6000);
  }, 60000);
});

describe('solver mid-hole starts', () => {
  it('reports a best-effort line even when it cannot solve', () => {
    const level = levelById('c5-6')!;
    const result = solveLevel(level, { maxStrokes: 1, angleSamples: 24, powerSamples: 3 });
    expect(result.solved).toBe(false);
    expect(result.shots).toEqual([]);
    expect(result.bestEffort.length).toBeGreaterThan(0);
  });

  it('leaves verification searches unbounded, so results stay machine-independent', () => {
    // A time budget makes the answer depend on how fast the box is. Only the
    // hint may opt in; the default must have no ceiling at all.
    expect(DEFAULT_SOLVE_OPTIONS.timeBudgetMs).toBeUndefined();
  });

  it('starts from the tee when no start is given', () => {
    const level = levelById('c1-1')!;
    const world = compileLevel(level);
    const fromTee = solveLevel(level, { angleSamples: 36, powerSamples: 3 });
    const explicit = solveLevel(level, {
      angleSamples: 36,
      powerSamples: 3,
      from: { position: settledTee(level), time: 0, state: boardStateOf(world) },
    });
    expect(explicit.solved).toBe(fromTee.solved);
    expect(explicit.shots).toEqual(fromTee.shots);
  });
});
