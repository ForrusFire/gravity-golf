import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    globals: true,
    environment: 'node',
    include: ['tests/unit/**/*.test.ts'],
    // The level solvability suites brute-force thousands of shots each.
    testTimeout: 60_000,
    // One fork per core. The heavy gates are CPU-bound and sharded across
    // files, so the pool is the thing that decides wall-clock time.
    pool: 'forks',
    poolOptions: { forks: { maxForks: 4 } },
  },
});
