import { mkdirSync } from 'node:fs';
import { chromium } from '@playwright/test';
import { resolveChromium } from '../tests/browser-path.ts';
const out = process.argv[2] ?? 'ui-shots';
mkdirSync(out, { recursive: true });
const executablePath = resolveChromium();
const browser = await chromium.launch({ args:['--mute-audio'], ...(executablePath?{executablePath}:{}) });
const page = await browser.newPage({ viewport: { width: 1280, height: 860 } });
const errors=[]; page.on('pageerror',e=>errors.push(e.message));
await page.goto('http://127.0.0.1:4173/', { waitUntil:'networkidle' });

// Seed some progress so the screens have content.
await page.evaluate(() => {
  const levels = {};
  const ids = window.gravityGolf.levelIds();
  const data = { version:1, levels, settings:{}, totalStrokes:0, totalShots:0, totalDeaths:0, totalPlayTime:0 };
  ids.slice(0,9).forEach((id,i)=>{ levels[id] = { bestStrokes: 2+(i%3), bestStars: (i%4), bestMedal: ['gold','silver','ace','bronze'][i%4], bestTime: 20+i, completed:true, attempts:i+1 }; });
  localStorage.setItem('gravity-golf/progress', JSON.stringify(data));
});
await page.reload({ waitUntil:'networkidle' });
await page.waitForTimeout(400);
await page.screenshot({ path:`${out}/title.png` });

await page.getByRole('button', { name:'Select hole' }).click();
await page.waitForTimeout(300);
await page.screenshot({ path:`${out}/levels.png` });
await page.getByRole('button', { name:'Back' }).click();

await page.getByRole('button', { name:'Scorecard' }).click();
await page.waitForTimeout(300);
await page.screenshot({ path:`${out}/scorecard.png` });
await page.getByRole('button', { name:'Back' }).click();

await page.getByRole('button', { name:'Settings' }).click();
await page.waitForTimeout(300);
await page.screenshot({ path:`${out}/settings.png` });
await page.getByRole('button', { name:'Back' }).click();

await page.getByRole('button', { name:/^(Play|Continue)$/ }).click();
await page.waitForTimeout(500);
await page.evaluate(() => { const s = window.gravityGolf.session; s.ball.position = { x:s.world.hole.position.x, y:s.world.hole.position.y - s.world.hole.radius*0.4 }; });
await page.waitForTimeout(1600);
await page.screenshot({ path:`${out}/results.png` });
console.log('errors:', errors.length?errors:'none');
await browser.close();
