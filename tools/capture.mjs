/**
 * Captures a screenshot of each hole, for eyeballing level layouts and
 * rendering without playing through the whole campaign.
 *
 *   node tools/capture.mjs [outputDir] [levelId ...]
 *
 * Requires a preview server on http://127.0.0.1:4173 (npm run preview).
 */
import { mkdirSync } from 'node:fs';
import { chromium } from '@playwright/test';
import { resolveChromium } from '../tests/browser-path.ts';

const outDir = process.argv[2] ?? 'captures';
const only = process.argv.slice(3);
mkdirSync(outDir, { recursive: true });

const executablePath = resolveChromium();
const browser = await chromium.launch({
  args: ['--mute-audio'],
  ...(executablePath ? { executablePath } : {}),
});
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });

const errors = [];
page.on('console', (m) => m.type() === 'error' && errors.push(m.text()));
page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));

await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });

const levelIds = await page.evaluate(() => {
  const app = window.gravityGolf;
  return app.levelIds();
});

for (const id of levelIds) {
  if (only.length > 0 && !only.includes(id)) continue;
  await page.evaluate((levelId) => window.gravityGolf.debugPlay(levelId), id);
  // Let the camera ease in and one animation cycle play out.
  await page.waitForTimeout(700);
  await page.screenshot({ path: `${outDir}/${id}.png` });
}

console.log(`captured ${only.length || levelIds.length} level(s) to ${outDir}/`);
console.log('errors:', errors.length ? errors : 'none');
await browser.close();
