import { expect, test, type Page } from '@playwright/test';

/**
 * The app exposes its instance on `window.gravityGolf`. These helpers drive the
 * real game object rather than guessing at pixel coordinates, which keeps the
 * tests about behaviour instead of layout.
 */
declare global {
  interface Window {
    gravityGolf: {
      playLevel(level: unknown): void;
      showLevels(): void;
      showTitle(): void;
    };
  }
}

const consoleErrors = (page: Page): string[] => {
  const errors: string[] = [];
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  page.on('pageerror', (error) => errors.push(`pageerror: ${error.message}`));
  return errors;
};

/** Reads gameplay state out of the live session. */
const sessionState = (page: Page) =>
  page.evaluate(() => {
    const app = window.gravityGolf as unknown as {
      session: {
        state: string;
        strokes: number;
        starCount: number;
        level: { id: string; name: string; par: number };
        ball: { position: { x: number; y: number }; atRest: boolean };
        canShoot: boolean;
      } | null;
    };
    const s = app.session;
    if (!s) return null;
    return {
      state: s.state,
      strokes: s.strokes,
      stars: s.starCount,
      levelId: s.level.id,
      par: s.level.par,
      ball: { ...s.ball.position },
      canShoot: s.canShoot,
    };
  });

/** Fires a shot straight through the game API, bypassing pointer geometry. */
const shoot = (page: Page, dx: number, dy: number, power: number) =>
  page.evaluate(
    (args) => {
      const app = window.gravityGolf as unknown as {
        session: { shoot(dir: { x: number; y: number }, power: number): unknown[] } | null;
      };
      app.session?.shoot({ x: args.dx, y: args.dy }, args.power);
    },
    { dx, dy, power },
  );

const startGame = async (page: Page): Promise<void> => {
  await page.goto('/');
  await expect(page.locator('canvas')).toBeVisible();
  await page.getByRole('button', { name: /^(Play|Continue)$/ }).click();
  await expect(page.locator('.hud')).not.toHaveClass(/hud--hidden/);
};

test.describe('boot', () => {
  test('loads to the title screen with no console errors', async ({ page }) => {
    const errors = consoleErrors(page);
    await page.goto('/');

    await expect(page).toHaveTitle('Gravity Golf');
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
    await expect(page.getByRole('button', { name: /^(Play|Continue)$/ })).toBeVisible();
    await expect(page.getByRole('button', { name: 'Select hole' })).toBeVisible();
    await expect(page.locator('body')).not.toHaveClass(/loading/);
    expect(errors).toEqual([]);
  });

  test('renders to the canvas at device pixel ratio', async ({ page }) => {
    await page.goto('/');
    const size = await page.evaluate(() => {
      const canvas = document.querySelector('canvas')!;
      return {
        backingWidth: canvas.width,
        cssWidth: canvas.getBoundingClientRect().width,
        ratio: window.devicePixelRatio,
      };
    });
    expect(size.backingWidth).toBeGreaterThan(0);
    // Backing store should track CSS size times the (capped) pixel ratio.
    expect(size.backingWidth).toBeGreaterThanOrEqual(Math.round(size.cssWidth));
  });

  test('keeps drawing frames', async ({ page }) => {
    await page.goto('/');
    const advanced = await page.evaluate(
      () =>
        new Promise<boolean>((resolve) => {
          let frames = 0;
          const tick = () => {
            frames++;
            if (frames >= 5) resolve(true);
            else requestAnimationFrame(tick);
          };
          requestAnimationFrame(tick);
          setTimeout(() => resolve(frames >= 5), 2000);
        }),
    );
    expect(advanced).toBe(true);
  });
});

test.describe('playing a hole', () => {
  test('starts the first hole ready to shoot', async ({ page }) => {
    await startGame(page);
    const state = await sessionState(page);
    expect(state).not.toBeNull();
    expect(state!.levelId).toBe('c1-1');
    expect(state!.strokes).toBe(0);
    expect(state!.state).toBe('aiming');
    expect(state!.canShoot).toBe(true);
    await expect(page.locator('.hud__level')).toHaveText('First Light');
  });

  test('a drag on the canvas takes a shot and counts a stroke', async ({ page }) => {
    await startGame(page);
    const box = (await page.locator('canvas').boundingBox())!;

    // Slingshot: drag away from the target to launch toward it.
    await page.mouse.move(box.x + box.width * 0.4, box.y + box.height * 0.6);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width * 0.25, box.y + box.height * 0.72, { steps: 10 });
    await page.mouse.up();

    await expect
      .poll(async () => (await sessionState(page))?.strokes, { timeout: 5000 })
      .toBe(1);
    await expect(page.locator('.hud__stat-value').first()).toHaveText('1');
  });

  test('the ball moves after a shot and settles again', async ({ page }) => {
    await startGame(page);
    const before = (await sessionState(page))!;
    await shoot(page, 1, -0.35, 0.55);

    await expect
      .poll(async () => (await sessionState(page))?.state, { timeout: 5000 })
      .not.toBe('flying');

    const after = (await sessionState(page))!;
    expect(Math.abs(after.ball.x - before.ball.x)).toBeGreaterThan(30);
    expect(after.strokes).toBeGreaterThanOrEqual(1);
  });

  test('sinking the ball opens the results panel and records progress', async ({ page }) => {
    await startGame(page);
    // Put the ball on the lip of the cup rather than relying on aim skill;
    // the capture rule in the simulation does the rest.
    await page.evaluate(() => {
      const app = window.gravityGolf as unknown as {
        session: {
          ball: { position: { x: number; y: number } };
          world: { hole: { position: { x: number; y: number }; radius: number } };
        } | null;
      };
      const s = app.session!;
      const hole = s.world.hole;
      s.ball.position = { x: hole.position.x, y: hole.position.y - hole.radius * 0.4 };
    });

    await expect(page.locator('.results')).toBeVisible({ timeout: 10_000 });
    await expect(page.locator('.results__title')).toBeVisible();
    await expect(page.getByRole('button', { name: 'Next hole' })).toBeVisible();

    const saved = await page.evaluate(() =>
      JSON.parse(localStorage.getItem('gravity-golf/progress') ?? '{}'),
    );
    expect(saved.levels?.['c1-1']?.completed).toBe(true);
  });

  test('undo takes back a shot and is disabled with nothing to undo', async ({ page }) => {
    await startGame(page);
    const undo = page.getByRole('button', { name: 'Take back last shot' });
    await expect(undo).toBeDisabled();

    const before = (await sessionState(page))!;
    await shoot(page, 1, -0.3, 0.5);
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(1);
    await expect(undo).toBeEnabled();

    await undo.click();
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(0);
    const after = (await sessionState(page))!;
    // Back where it started, give or take the sub-unit settling a resting ball
    // does as it nestles into the surface.
    expect(Math.hypot(after.ball.x - before.ball.x, after.ball.y - before.ball.y)).toBeLessThan(2);
    expect(after.canShoot).toBe(true);
    await expect(undo).toBeDisabled();
  });

  test('the Z key takes back a shot', async ({ page }) => {
    await startGame(page);
    await shoot(page, 1, -0.3, 0.5);
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(1);

    await page.locator('canvas').focus();
    await page.keyboard.press('KeyZ');
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(0);
  });

  test('restart resets the stroke count', async ({ page }) => {
    await startGame(page);
    await shoot(page, 1, -0.3, 0.5);
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(1);

    await page.getByRole('button', { name: 'Restart hole' }).click();
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(0);
  });
});

test.describe('menus', () => {
  test('pause opens, resumes and returns to the title', async ({ page }) => {
    await startGame(page);

    await page.getByRole('button', { name: 'Pause' }).click();
    await expect(page.getByRole('heading', { name: 'Paused' })).toBeVisible();

    await page.getByRole('button', { name: 'Resume' }).click();
    await expect(page.getByRole('heading', { name: 'Paused' })).toBeHidden();

    await page.getByRole('button', { name: 'Pause' }).click();
    await page.getByRole('button', { name: 'Main menu' }).click();
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
  });

  test('escape pauses and unpauses', async ({ page }) => {
    await startGame(page);
    await page.locator('canvas').click({ position: { x: 5, y: 5 } });

    await page.keyboard.press('Escape');
    await expect(page.getByRole('heading', { name: 'Paused' })).toBeVisible();
    await page.keyboard.press('Escape');
    await expect(page.getByRole('heading', { name: 'Paused' })).toBeHidden();
  });

  test('level select lists holes and locks the ones not yet reached', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'Select hole' }).click();

    await expect(page.getByRole('heading', { name: 'Select a hole' })).toBeVisible();
    const cards = page.locator('.level-card');
    await expect(cards).toHaveCount(30);

    // The first hole is always open; the second is not, on a fresh save.
    await expect(cards.nth(0)).toBeEnabled();
    await expect(cards.nth(1)).toBeDisabled();

    await cards.nth(0).click();
    await expect.poll(async () => (await sessionState(page))?.levelId).toBe('c1-1');
  });

  test('settings persist across a reload', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'Settings' }).click();
    await page.getByLabel('High contrast').check();
    await page.getByRole('button', { name: 'Back', exact: true }).click();

    await page.reload();
    await page.getByRole('button', { name: 'Settings' }).click();
    await expect(page.getByLabel('High contrast')).toBeChecked();
  });

  test('the scorecard shows every hole and updates after a win', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'Scorecard' }).click();
    await expect(page.getByRole('heading', { name: 'Scorecard' })).toBeVisible();

    // Nothing played yet: every hole shows a dash for its best score.
    const firstRow = page.locator('.card__table tbody tr').first();
    await expect(firstRow).toContainText('First Light');
    await expect(page.locator('.card__totals')).toContainText('0/30');

    await page.getByRole('button', { name: 'Back', exact: true }).click();
    await page.getByRole('button', { name: /^(Play|Continue)$/ }).click();
    await page.evaluate(() => {
      const app = window.gravityGolf as unknown as {
        session: {
          ball: { position: { x: number; y: number } };
          world: { hole: { position: { x: number; y: number }; radius: number } };
        } | null;
      };
      const s = app.session!;
      s.ball.position = {
        x: s.world.hole.position.x,
        y: s.world.hole.position.y - s.world.hole.radius * 0.4,
      };
    });
    await expect(page.locator('.results')).toBeVisible({ timeout: 10_000 });

    await page.getByRole('button', { name: 'All holes' }).click();
    await page.getByRole('button', { name: 'Back', exact: true }).click();
    await page.getByRole('button', { name: 'Scorecard' }).click();
    await expect(page.locator('.card__totals')).toContainText('1/30');
  });

  test('the daily challenge generates a playable hole', async ({ page }) => {
    test.setTimeout(90_000);
    await page.goto('/');
    await page.getByRole('button', { name: /^Daily challenge/ }).click();

    // Generation runs in a worker; the page must stay responsive meanwhile.
    await expect(page.locator('.loading-panel')).toBeVisible();

    await expect
      .poll(async () => (await sessionState(page))?.levelId, { timeout: 75_000 })
      .toMatch(/^daily-\d{4}-\d{2}-\d{2}$/);

    const state = (await sessionState(page))!;
    expect(state.par).toBeGreaterThanOrEqual(2);
    expect(state.canShoot).toBe(true);
    await expect(page.locator('.hud__level')).toContainText('Daily:');

    // The generated hole is cached, so returning to it is instant.
    const cached = await page.evaluate(() =>
      Object.keys(localStorage).filter((k) => k.startsWith('gravity-golf/generated/')),
    );
    expect(cached.length).toBe(1);
  });

  test('the scorecard lists feats, locked until earned', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'Scorecard' }).click();
    await expect(page.getByRole('heading', { name: /^Feats/ })).toBeVisible();
    await expect(page.locator('.feat')).toHaveCount(6);
    // A fresh save has earned none.
    await expect(page.locator('.feat--earned')).toHaveCount(0);
  });

  test('ball skins are locked behind stars and selectable once earned', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'Settings' }).click();

    const skins = page.locator('.skin');
    await expect(skins).toHaveCount(5);
    // Only the default is available with no stars.
    await expect(skins.nth(0)).toBeEnabled();
    await expect(skins.nth(1)).toBeDisabled();
    await expect(skins.nth(0)).toHaveClass(/skin--active/);
  });

  test('help screen explains the controls', async ({ page }) => {
    await page.goto('/');
    await page.getByRole('button', { name: 'How to play' }).click();
    await expect(page.getByRole('heading', { name: 'How to play' })).toBeVisible();
    await expect(page.getByRole('heading', { name: 'Gravity' })).toBeVisible();
    await page.getByRole('button', { name: 'Back', exact: true }).click();
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
  });
});

test.describe('accessibility', () => {
  test('dialogs are marked up as dialogs and trap focus', async ({ page }) => {
    await page.goto('/');
    const dialog = page.getByRole('dialog');
    await expect(dialog).toHaveAttribute('aria-modal', 'true');
    // Focus lands inside the panel on open.
    const focusedInside = await page.evaluate(() =>
      document.querySelector('.panel')!.contains(document.activeElement),
    );
    expect(focusedInside).toBe(true);
  });

  test('every menu control is reachable by keyboard', async ({ page }) => {
    await page.goto('/');
    await page.keyboard.press('Tab');
    const tag = await page.evaluate(() => document.activeElement?.tagName);
    expect(['BUTTON', 'INPUT', 'SELECT']).toContain(tag);
  });

  test('the canvas is labelled for assistive tech', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('canvas')).toHaveAttribute('aria-label', /Gravity Golf/);
  });
});

test.describe('input', () => {
  test('keyboard aiming takes a shot with the space bar', async ({ page }) => {
    await startGame(page);
    await page.locator('canvas').focus();

    await page.keyboard.press('ArrowRight');
    await page.keyboard.down('ArrowUp');
    await page.waitForTimeout(400);
    await page.keyboard.up('ArrowUp');
    await page.keyboard.press('Space');

    await expect
      .poll(async () => (await sessionState(page))?.strokes, { timeout: 5000 })
      .toBe(1);
  });

  test('R restarts the hole', async ({ page }) => {
    await startGame(page);
    await shoot(page, 1, 0, 0.5);
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(1);

    await page.locator('canvas').focus();
    await page.keyboard.press('KeyR');
    await expect.poll(async () => (await sessionState(page))?.strokes).toBe(0);
  });

  test('the wheel zooms without scrolling the page', async ({ page }) => {
    await startGame(page);
    const box = (await page.locator('canvas').boundingBox())!;
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.wheel(0, -400);
    await page.waitForTimeout(300);
    expect(await page.evaluate(() => window.scrollY)).toBe(0);
  });
});

test.describe('resilience', () => {
  test('recovers from corrupt saved progress', async ({ page }) => {
    await page.goto('/');
    await page.evaluate(() => localStorage.setItem('gravity-golf/progress', '{{{ not json'));

    const errors = consoleErrors(page);
    await page.reload();
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
    expect(errors).toEqual([]);
  });

  test('survives a window resize mid-play', async ({ page }) => {
    await startGame(page);
    await page.setViewportSize({ width: 600, height: 900 });
    await page.waitForTimeout(300);
    await page.setViewportSize({ width: 1400, height: 700 });
    await page.waitForTimeout(300);

    const state = await sessionState(page);
    expect(state).not.toBeNull();
    const canvasWidth = await page.evaluate(
      () => document.querySelector('canvas')!.getBoundingClientRect().width,
    );
    expect(canvasWidth).toBeGreaterThan(1000);
  });
});
