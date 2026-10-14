import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ ...(executablePath?{executablePath}:{}) });
const D='/tmp/claude-0/-home-user/5d7b8882-f533-5ead-8fb3-8720590302f5/scratchpad/polish';
for (const [w,h,n] of [[1280,800,'desk'],[390,844,'phone']]) {
const page = await b.newPage({ viewport: { width: w, height: h } });
await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });
await page.getByRole('button', { name: 'Play', exact: true }).click();
await page.waitForTimeout(3400);
await page.screenshot({ path: `${D}/tut-${n}.png` });
await page.close();
}
await b.close();
