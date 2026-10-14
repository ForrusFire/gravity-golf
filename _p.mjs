import { chromium } from '@playwright/test';
import { resolveChromium } from './tests/browser-path.ts';
const executablePath = resolveChromium();
const b = await chromium.launch({ args:['--mute-audio'], ...(executablePath?{executablePath}:{}) });
const page = await b.newPage({ viewport: { width: 1600, height: 900 } });
await page.goto('http://127.0.0.1:4173/', { waitUntil: 'networkidle' });
for (let k=0;k<3;k++){
await page.evaluate(() => window.gravityGolf.debugPlay('c4-6'));
await page.evaluate(() => window.gravityGolf.session?.shoot({ x: 1, y: -0.3 }, 0.9));
const r = await page.evaluate(() => new Promise((res) => {
  const app = window.gravityGolf; const ren = app.renderer;
  const od = ren.draw.bind(ren), ou = app.update.bind(app);
  const log = [];
  app.update = (dt) => { const t=performance.now(); ou(dt); const u=performance.now()-t; if (u>10) log.push(['u',u.toFixed(1),app.session?.state, app.session?.world.time.toFixed(2)]); };
  ren.draw = (...a) => { const t=performance.now(); od(...a); const d=performance.now()-t; if (d>10) log.push(['d',d.toFixed(1)]); };
  setTimeout(()=>{ren.draw=od; app.update=ou; res(log);}, 4000);
}));
console.log(JSON.stringify(r));
}
await b.close();
