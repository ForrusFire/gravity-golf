import { mkdirSync } from 'node:fs';
import { chromium, devices } from '@playwright/test';
import { resolveChromium } from '../tests/browser-path.ts';
const out = process.argv[2] ?? 'variants';
mkdirSync(out, { recursive: true });
const executablePath = resolveChromium();
const browser = await chromium.launch({ args:['--mute-audio'], ...(executablePath?{executablePath}:{}) });

// Mobile portrait
const m = await browser.newContext({ ...devices['Pixel 5'] });
const mp = await m.newPage();
await mp.goto('http://127.0.0.1:4173/', { waitUntil:'networkidle' });
await mp.getByRole('button', { name:/^(Play|Continue)$/ }).click();
await mp.waitForTimeout(900);
await mp.screenshot({ path:`${out}/mobile-play.png` });
await mp.getByRole('button', { name:'Pause' }).click();
await mp.waitForTimeout(300);
await mp.screenshot({ path:`${out}/mobile-pause.png` });

// High contrast desktop
const d = await browser.newContext({ viewport:{width:1280,height:800} });
const dp = await d.newPage();
await dp.goto('http://127.0.0.1:4173/', { waitUntil:'networkidle' });
await dp.evaluate(() => {
  const raw = JSON.parse(localStorage.getItem('gravity-golf/progress') ?? '{}');
  raw.settings = { ...(raw.settings ?? {}), highContrast: true };
  localStorage.setItem('gravity-golf/progress', JSON.stringify(raw));
});
await dp.reload({ waitUntil:'networkidle' });
await dp.evaluate(() => window.gravityGolf.debugPlay('c3-6'));
await dp.waitForTimeout(900);
await dp.screenshot({ path:`${out}/high-contrast.png` });
console.log('ok');
await browser.close();
