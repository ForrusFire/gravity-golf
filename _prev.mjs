import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ ...(executablePath?{executablePath}:{}) });
const D='/tmp/claude-0/-home-user/5d7b8882-f533-5ead-8fb3-8720590302f5/scratchpad/polish';
const page = await b.newPage({ viewport: { width: 1280, height: 800 } });
await page.goto('http://127.0.0.1:4173/?hole=c1-3', { waitUntil: 'networkidle' });
await page.waitForTimeout(2500);
await page.evaluate(() => window.gravityGolf.shoot({x:1,y:0.25}, 0.55));
for (let i=0;i<40;i++){ await page.waitForTimeout(250); const st = await page.evaluate(()=>window.gravityGolf.session.canShoot); if (st) break; }
await page.locator('canvas').focus();
await page.keyboard.press('KeyZ');
await page.waitForTimeout(600);
const scr = await page.evaluate(() => window.gravityGolf.renderer.camera.worldToScreen(window.gravityGolf.session.ball.position));
await page.mouse.move(scr.x, scr.y); await page.mouse.down();
await page.mouse.move(scr.x - 50, scr.y - 20, {steps:8});
await page.waitForTimeout(300);
await page.screenshot({ path: `${D}/prev.png` });
await b.close();
