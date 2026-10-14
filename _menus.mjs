import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ ...(executablePath?{executablePath}:{}) });
const D='/tmp/claude-0/-home-user/5d7b8882-f533-5ead-8fb3-8720590302f5/scratchpad/polish';
const page = await b.newPage({ viewport: { width: 1280, height: 800 } });
await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });
for (const name of ['Settings','How to play','Scorecard']) {
  await page.getByRole('button', { name, exact: true }).click();
  await page.waitForTimeout(500);
  await page.screenshot({ path: `${D}/menu-${name.replace(/ /g,'')}.png` });
  await page.keyboard.press('Escape'); await page.waitForTimeout(300);
  if (await page.getByRole('button', { name: 'Back', exact: true }).isVisible().catch(()=>false)) await page.getByRole('button', { name: 'Back', exact: true }).click();
  await page.waitForTimeout(300);
}
await page.goto('http://127.0.0.1:4173/?hole=c2-3', { waitUntil: 'networkidle' });
await page.waitForTimeout(1500);
await page.keyboard.press('Escape'); await page.waitForTimeout(500);
await page.screenshot({ path: `${D}/menu-pause.png` });
await b.close();
