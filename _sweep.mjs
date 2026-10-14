import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ ...(executablePath?{executablePath}:{}) });
const D='/tmp/claude-0/-home-user/5d7b8882-f533-5ead-8fb3-8720590302f5/scratchpad/polish';
const [w,h] = (process.env.VP ?? '1280x800').split('x').map(Number);
const page = await b.newPage({ viewport: { width: w, height: h } });
for (const hole of process.argv.slice(2)) {
  await page.goto(`http://127.0.0.1:4173/?hole=${hole}`, { waitUntil: 'networkidle' });
  await page.waitForTimeout(Number(process.env.WAIT ?? 1500));
  await page.screenshot({ path: `${D}/sw-${hole}.png` });
}
await b.close();
