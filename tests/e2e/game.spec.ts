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
      debugPlay(id: string): void;
      levelIds(): string[];
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
    // Derived, not hard-coded: adding a chapter should not fail this test.
    const holeCount = await page.evaluate(() => window.gravityGolf.levelIds().length);
    expect(holeCount).toBeGreaterThan(0);
    const cards = page.locator('.level-card');
    await expect(cards).toHaveCount(holeCount);

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
    const holeCount = await page.evaluate(() => window.gravityGolf.levelIds().length);
    await expect(page.locator('.card__totals')).toContainText(`0/${holeCount}`);

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
    await expect(page.locator('.card__totals')).toContainText(`1/${holeCount}`);
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
    // The colour language the later chapters depend on has to be documented.
    // Exact: "Violet is machinery" would otherwise match "Machinery" too.
    await expect(page.getByRole('heading', { name: 'Machinery', exact: true })).toBeVisible();
    await expect(
      page.getByRole('heading', { name: 'Reading the field', exact: true }),
    ).toBeVisible();
    await expect(page.getByRole('heading', { name: 'Sealed cups', exact: true })).toBeVisible();
    await expect(page.locator('.help__swatch')).toHaveCount(7);
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

test.describe('finishing the campaign', () => {
  /** Marks every campaign hole complete except `except`. */
  const seedProgress = async (page: Page, except: string): Promise<void> => {
    await page.goto('/');
    await page.evaluate((skip) => {
      const ids = window.gravityGolf.levelIds().filter((id) => id !== skip);
      const levels: Record<string, unknown> = {};
      for (const id of ids) {
        levels[id] = {
          bestStrokes: 3,
          bestShots: [],
          bestStars: 3,
          bestMedal: 'silver',
          bestTime: 20,
          completed: true,
          attempts: 1,
        };
      }
      localStorage.setItem(
        'gravity-golf/progress',
        JSON.stringify({ version: 1, levels, feats: [], totalStrokes: 0, totalShots: 0, totalDeaths: 0, totalPlayTime: 0 }),
      );
    }, except);
  };

  /** Drops the ball straight into the cup and waits for the hole to resolve. */
  const sinkIt = async (page: Page): Promise<void> => {
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
  };

  test('celebrates the last hole rather than shrugging at it', async ({ page }) => {
    const errors = consoleErrors(page);
    await seedProgress(page, 'c1-1');
    await page.goto('/?hole=c1-1');
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c1-1',
    );

    await sinkIt(page);
    await expect(page.locator('.results--finale')).toBeVisible({ timeout: 10000 });
    await expect(page.getByRole('heading', { name: 'Course complete' })).toBeVisible();
    // The finale replaces the per-hole panel, so there is no Next hole button.
    await expect(page.getByRole('button', { name: 'Next hole' })).toHaveCount(0);
    expect(errors).toEqual([]);
  });

  test('fires on the last hole finished, not the last hole in order', async ({ page }) => {
    // The player who leaves hole 3 for last should still get the finale there.
    await seedProgress(page, 'c1-3');
    await page.goto('/?hole=c1-3');
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c1-3',
    );

    await sinkIt(page);
    await expect(page.locator('.results--finale')).toBeVisible({ timeout: 10000 });
  });

  test('does not fire again on a replay once the course is done', async ({ page }) => {
    await seedProgress(page, 'c1-1');
    await page.goto('/?hole=c1-1');
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c1-1',
    );
    await sinkIt(page);
    await expect(page.locator('.results--finale')).toBeVisible({ timeout: 10000 });

    // Play it again: an ordinary hole now, with an ordinary results panel.
    await page.evaluate(() => window.gravityGolf.debugPlay('c1-1'));
    await expect.poll(async () => (await sessionState(page))?.strokes, { timeout: 10000 }).toBe(0);
    await sinkIt(page);
    await expect(page.locator('.results')).toBeVisible({ timeout: 10000 });
    await expect(page.locator('.results--finale')).toHaveCount(0);
  });
});

test.describe('shared links', () => {
  test('a hole link opens that hole straight away', async ({ page }) => {
    const errors = consoleErrors(page);
    await page.goto('/?hole=c3-2');

    // Straight into play: no title screen, no level select.
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c3-2',
    );
    await expect(page.locator('.hud')).not.toHaveClass(/hud--hidden/);
    expect(errors).toEqual([]);
  });

  test('a link ignores the star gates that would normally lock the hole', async ({ page }) => {
    // A fresh save has no stars, so this hole is locked in the level select.
    // Someone who followed a link was sent there on purpose.
    await page.goto('/?hole=c9-1');
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c9-1',
    );
  });

  test('a nonsense link falls back to the title screen', async ({ page }) => {
    const errors = consoleErrors(page);
    await page.goto('/?hole=..%2F..%2Fetc%2Fpasswd');
    await expect(page.getByRole('heading', { name: /GRAVITY/ })).toBeVisible();
    expect(errors).toEqual([]);
  });

  test('the address bar tracks the hole and clears on the way out', async ({ page }) => {
    await startGame(page);
    await expect.poll(async () => new URL(page.url()).search, { timeout: 10000 }).toBe('?hole=c1-1');

    // Back to a menu, the link goes: a refresh from here should not drop the
    // player back into the hole they just left.
    await page.evaluate(() => window.gravityGolf.showLevels());
    await expect.poll(async () => new URL(page.url()).search).toBe('');
  });

  test('offers a share button that copies the link', async ({ page, context }) => {
    await context.grantPermissions(['clipboard-read', 'clipboard-write']);
    await page.goto('/?hole=c1-1');
    await expect.poll(async () => (await sessionState(page))?.levelId, { timeout: 10000 }).toBe(
      'c1-1',
    );

    await page.locator('canvas').press('Escape');
    await page.getByRole('button', { name: 'Share this hole' }).click();
    const copied = await page.evaluate(() => navigator.clipboard.readText());
    expect(copied).toContain('?hole=c1-1');
  });
});

test.describe('hints', () => {
  test('suggests a line, arms the aim, and marks the run as hinted', async ({ page }) => {
    const errors = consoleErrors(page);
    await startGame(page);
    await expect.poll(async () => (await sessionState(page))?.canShoot).toBe(true);

    await page.getByRole('button', { name: 'Show a suggested line' }).click();

    // The solver runs in a worker; the button is disabled until it answers.
    await expect
      .poll(
        async () =>
          page.evaluate(() => {
            const app = window.gravityGolf as unknown as { session: { hintsUsed: number } | null };
            return app.session?.hintsUsed ?? 0;
          }),
        { timeout: 20000 },
      )
      .toBe(1);

    // The hint armed an aim, so the shot key alone plays it.
    await page.locator('canvas').press('Space');
    await expect.poll(async () => (await sessionState(page))?.strokes, { timeout: 10000 }).toBe(1);

    expect(errors).toEqual([]);
  });

  test('a hinted run keeps its medal but not its feats', async ({ page }) => {
    await startGame(page);
    await expect.poll(async () => (await sessionState(page))?.canShoot).toBe(true);

    // Mark the run hinted, then drop the ball into the cup and let the app's own
    // loop finish the hole, so this exercises the real completion path.
    await page.evaluate(() => {
      const app = window.gravityGolf as unknown as {
        session: {
          hintsUsed: number;
          ball: { position: { x: number; y: number } };
          world: { hole: { position: { x: number; y: number }; radius: number } };
        } | null;
      };
      const s = app.session!;
      s.hintsUsed = 1;
      s.ball.position = {
        x: s.world.hole.position.x,
        y: s.world.hole.position.y - s.world.hole.radius * 0.4,
      };
    });

    await expect(page.locator('.results')).toBeVisible({ timeout: 10_000 });
    await expect(page.locator('.results__hinted')).toContainText('Hint used');
    // The medal still stands: a hint costs the flourishes, not the score.
    await expect(page.locator('.results__medal')).toBeVisible();
    await expect(page.locator('.results__feats')).toHaveCount(0);
  });
});

test.describe('state mechanics', () => {
  /** Plays the named hole through the debug hook and waits for control. */
  const openHole = async (page: Page, id: string): Promise<void> => {
    await startGame(page);
    await page.evaluate((levelId) => window.gravityGolf.debugPlay(levelId), id);
    await expect
      .poll(async () => (await sessionState(page))?.levelId, { timeout: 5000 })
      .toBe(id);
    await expect.poll(async () => (await sessionState(page))?.canShoot, { timeout: 5000 }).toBe(true);
  };

  const boardState = (page: Page) =>
    page.evaluate(() => {
      const app = window.gravityGolf as unknown as {
        session: {
          world: { switches: Array<{ id: string; on: boolean }>; breakables: Record<string, number> };
        } | null;
      };
      const world = app.session?.world;
      if (!world) return null;
      return {
        switches: world.switches.map((s) => ({ id: s.id, on: s.on })),
        breakables: { ...world.breakables },
      };
    });

  test('throwing a switch drops the bridge it controls', async ({ page }) => {
    const errors = consoleErrors(page);
    await openHole(page, 'c6-2');

    const before = await boardState(page);
    expect(before?.switches).toEqual([{ id: 'sw-span', on: false }]);

    // The pad sits up and to the right of the tee.
    await shoot(page, 0.55, -1, 0.62);
    await expect
      .poll(async () => (await boardState(page))?.switches.some((s) => s.on), { timeout: 15000 })
      .toBe(true);

    expect(errors).toEqual([]);
  });

  test('a sealed cup refuses the ball and says why', async ({ page }) => {
    await openHole(page, 'c6-4');

    const locked = await page.evaluate(() => {
      const app = window.gravityGolf as unknown as {
        session: { world: { hole: { requiresStars?: number } } } | null;
      };
      return app.session?.world.hole.requiresStars ?? 0;
    });
    expect(locked).toBe(3);

    // Nothing can be sunk here until all three stars are banked, so a long run
    // of play must leave the hole unfinished rather than quietly completing.
    await shoot(page, 1, 0.1, 1);
    await page.waitForTimeout(4000);
    const state = await sessionState(page);
    expect(state?.state).not.toBe('sunk');
  });

  test('a held switch springs back on its own', async ({ page }) => {
    await openHole(page, 'c8-1');

    const before = await boardState(page);
    expect(before?.switches).toEqual([{ id: 'sw-door', on: false }]);

    // The pad sits to the right along the floor; a firm roll reaches it.
    await shoot(page, 1, 0, 0.5);
    await expect
      .poll(async () => (await boardState(page))?.switches[0]?.on, { timeout: 15000 })
      .toBe(true);
    // Nothing touches it again — only its own 3.2s clock turns it off.
    await expect
      .poll(async () => (await boardState(page))?.switches[0]?.on, { timeout: 15000 })
      .toBe(false);
  });

  test('a crystal block loses a hit when struck', async ({ page }) => {
    await openHole(page, 'c6-3');

    const before = await boardState(page);
    expect(before?.breakables).toMatchObject({ c1: 2, c2: 2, c3: 1 });

    await shoot(page, 1, 0, 1);
    await expect
      .poll(
        async () => {
          const now = await boardState(page);
          if (!now) return 99;
          return Object.values(now.breakables).reduce((sum, n) => sum + n, 0);
        },
        { timeout: 15000 },
      )
      .toBeLessThan(5);
  });
});
