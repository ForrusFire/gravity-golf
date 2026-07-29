import { mkdirSync } from 'node:fs';
import { chromium } from '@playwright/test';
import { resolveChromium } from '../tests/browser-path.ts';
const out = process.argv[2] ?? 'daily';
mkdirSync(out, { recursive: true });
const executablePath = resolveChromium();
const browser = await chromium.launch({ args:['--mute-audio'], ...(executablePath?{executablePath}:{}) });
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
const errs=[]; page.on('pageerror', e=>errs.push(String(e)));
await page.goto('http://127.0.0.1:4173/', { waitUntil:'networkidle' });
await page.getByRole('button', { name:/^Daily challenge/ }).click();
await page.waitForTimeout(400);
await page.screenshot({ path:`${out}/loading.png` });
for (let i=0;i<120;i++){ const s = await page.evaluate(()=>window.gravityGolf.session?.level?.id ?? null); if (s) break; await page.waitForTimeout(500); }
await page.waitForTimeout(900);
await page.screenshot({ path:`${out}/daily.png` });
const info = await page.evaluate(()=>{const l=window.gravityGolf.session.level; return {id:l.id,name:l.name,par:l.par,bodies:(l.bodies||[]).length};});
console.log('daily:', JSON.stringify(info), 'errors:', errs.length?errs:'none');
await browser.close();
