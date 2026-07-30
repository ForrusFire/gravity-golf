import { expect, test, type Page } from '@playwright/test';

/**
 * The service worker only exists in production builds, which is what the
 * Playwright webServer serves. These tests are the only thing standing between
 * "we ship a service worker" and "we ship a service worker that works".
 */
/**
 * Waits until the worker has activated *and* finished precaching.
 * `serviceWorker.ready` can resolve while the worker is still activating, so
 * polling the cache itself is the only signal that actually means "installed".
 */
const waitForPrecache = async (page: Page): Promise<string[]> => {
  await page.evaluate(() => navigator.serviceWorker.ready);
  let cached: string[] = [];
  await expect
    .poll(
      async () => {
        cached = await page.evaluate(async () => {
          const names = await caches.keys();
          if (names.length === 0) return [];
          const cache = await caches.open(names[0]!);
          return (await cache.keys()).map((request) => new URL(request.url).pathname);
        });
        return cached.length;
      },
      { timeout: 15_000 },
    )
    .toBeGreaterThanOrEqual(4);
  return cached;
};

test.describe('offline support', () => {
  test('serves a web app manifest and icons', async ({ page }) => {
    await page.goto('/');

    const manifestHref = await page.getAttribute('link[rel="manifest"]', 'href');
    expect(manifestHref).toBeTruthy();

    const manifest = await page.evaluate(async (href) => {
      const response = await fetch(href!);
      return { ok: response.ok, body: await response.json() };
    }, manifestHref);

    expect(manifest.ok).toBe(true);
    expect(manifest.body.name).toBe('Gravity Golf');
    expect(manifest.body.icons.length).toBeGreaterThan(0);
    // A maskable icon keeps platforms from cropping into the artwork.
    expect(manifest.body.icons.some((i: { purpose?: string }) => i.purpose === 'maskable')).toBe(
      true,
    );

    for (const icon of manifest.body.icons) {
      const response = await page.request.get(new URL(icon.src, page.url()).toString());
      expect(response.ok(), `icon ${icon.src} should be served`).toBe(true);
    }
  });

  test('registers a service worker and precaches the whole build', async ({ page }) => {
    await page.goto('/');
    const cached = await waitForPrecache(page);

    expect(cached.some((path) => path.endsWith('.js'))).toBe(true);
    expect(cached.some((path) => path.endsWith('.css'))).toBe(true);
    // The manifest and icons live in publicDir and never reach the bundle, so
    // they are the ones most likely to be silently left out.
    expect(cached.some((path) => path.endsWith('.webmanifest'))).toBe(true);
    expect(cached.some((path) => path.endsWith('icon.svg'))).toBe(true);

    await expect
      .poll(async () => page.evaluate(async () => (await navigator.serviceWorker.ready).active?.state))
      .toBe('activated');
  });

  test('still boots and plays with the network cut off', async ({ page, context }) => {
    await page.goto('/');
    await waitForPrecache(page);

    await context.setOffline(true);
    await page.reload();

    // The shell alone is not enough: the bundle has to come out of the cache
    // too, or the page renders an empty loading screen.
    await expect(page.locator('body')).not.toHaveClass(/loading/);
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
    await page.getByRole('button', { name: /^(Play|Continue)$/ }).click();
    await expect(page.locator('.hud')).not.toHaveClass(/hud--hidden/);
    await expect(page.locator('.hud__level')).toHaveText('First Light');

    await context.setOffline(false);
  });

  test('opens a shared link with the network cut off', async ({ page, context }) => {
    // The precache stores the shell under './', with no query string. A shared
    // link carries one, so the navigation fallback has to ignore it — otherwise
    // every shared link is a dead page offline.
    await page.goto('/');
    await waitForPrecache(page);

    await context.setOffline(true);
    await page.goto('/?hole=c2-3');

    await expect(page.locator('body')).not.toHaveClass(/loading/);
    await expect(page.locator('.hud')).not.toHaveClass(/hud--hidden/);
    await expect(page.locator('.hud__level')).toHaveText('Dust Bowl');

    await context.setOffline(false);
  });
});
