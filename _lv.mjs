import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ ...(executablePath?{executablePath}:{}) });
const D='/tmp/claude-0/-home-user/5d7b8882-f533-5ead-8fb3-8720590302f5/scratchpad/polish';
const page = await b.newPage({ viewport: { width: 1280, height: 800 } });
await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });
await page.evaluate(() => {
  const ids = window.gravityGolf.levelIds();
  const medals = ['ace','gold','silver','bronze','none'];
  const levels = {};
  ids.slice(0, 10).forEach((id, i) => { levels[id] = { bestStrokes: 3, bestShots: [], bestStars: i % 4, bestMedal: medals[i % 5], bestTime: 20, completed: true, attempts: 1 }; });
  localStorage.setItem('gravity-golf/progress', JSON.stringify({ version: 1, levels, feats: [], totalStrokes: 0, totalShots: 0, totalDeaths: 0, totalPlayTime: 0 }));
});
await page.reload({ waitUntil: 'networkidle' });
await page.getByRole('button', { name: 'Select hole' }).click();
await page.waitForTimeout(500);
await page.screenshot({ path: `${D}/levels2.png` });
await b.close();
