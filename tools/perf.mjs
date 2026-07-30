/**
 * Measures the game's own per-frame CPU cost on the busiest holes.
 *
 *   node tools/perf.mjs
 *
 * It instruments update() and draw() rather than timing requestAnimationFrame
 * deltas. Frame deltas are dominated by the compositor, which in a headless,
 * software-rasterised container stalls for reasons that have nothing to do with
 * the game — measuring them tells you about the container, not the code.
 */
import { chromium } from '@playwright/test';
import { resolveChromium } from '../tests/browser-path.ts';

/** Per-frame CPU budget for the game itself, well inside a 16.7ms frame. */
const BUDGET_MS = 6;
// The busiest holes in the game: most bodies, most zones, most overdraw.
const LEVELS = ['c5-6', 'c7-6', 'c4-6', 'c7-1', 'c5-4', 'c6-6', 'c2-4', 'c1-1'];
const SAMPLE_MS = 2500;

const executablePath = resolveChromium();
const browser = await chromium.launch({
  args: ['--mute-audio'],
  ...(executablePath ? { executablePath } : {}),
});
const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });

let worst = 0;
for (const id of LEVELS) {
  await page.evaluate((levelId) => window.gravityGolf.debugPlay(levelId), id);
  // Take a big shot so physics, trail and particles are all live while sampling.
  await page.evaluate(() => window.gravityGolf.session?.shoot({ x: 1, y: -0.3 }, 0.9));
  await page.waitForTimeout(250);

  const stats = await page.evaluate(
    (sampleMs) =>
      new Promise((resolve) => {
        const app = window.gravityGolf;
        const renderer = app.renderer;
        const originalDraw = renderer.draw.bind(renderer);
        const originalUpdate = app.update.bind(app);
        const totals = [];
        let pending = 0;
        let peakParticles = 0;

        app.update = (dt) => {
          const t0 = performance.now();
          originalUpdate(dt);
          pending = performance.now() - t0;
        };
        renderer.draw = (...args) => {
          const t0 = performance.now();
          originalDraw(...args);
          totals.push(pending + (performance.now() - t0));
          peakParticles = Math.max(peakParticles, renderer.particles.active);
          pending = 0;
        };

        setTimeout(() => {
          renderer.draw = originalDraw;
          app.update = originalUpdate;
          const sorted = [...totals].sort((a, b) => a - b);
          resolve({
            frames: sorted.length,
            median: sorted[Math.floor(sorted.length / 2)] ?? 0,
            p95: sorted[Math.floor(sorted.length * 0.95)] ?? 0,
            max: sorted[sorted.length - 1] ?? 0,
            peakParticles,
          });
        }, sampleMs);
      }),
    SAMPLE_MS,
  );

  worst = Math.max(worst, stats.p95);
  const flag = stats.p95 > BUDGET_MS ? '  <-- OVER BUDGET' : '';
  console.log(
    `${id.padEnd(6)} frames ${String(stats.frames).padStart(4)}  ` +
      `median ${stats.median.toFixed(2)}ms  p95 ${stats.p95.toFixed(2)}ms  ` +
      `max ${stats.max.toFixed(2)}ms  peak particles ${stats.peakParticles}${flag}`,
  );
}

await browser.close();
console.log(`\nworst p95: ${worst.toFixed(2)}ms of game CPU per frame (budget ${BUDGET_MS}ms)`);
process.exit(worst > BUDGET_MS ? 1 : 0);
