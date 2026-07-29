import { existsSync, readdirSync } from 'node:fs';
import { join } from 'node:path';

/**
 * Locates a usable Chromium.
 *
 * Some CI images pre-install browsers under PLAYWRIGHT_BROWSERS_PATH at a
 * revision that does not match the Playwright package, and downloading is
 * disabled. Returning that binary explicitly keeps the suite runnable there,
 * while returning `undefined` lets Playwright use its own managed browser
 * everywhere else.
 */
export const resolveChromium = (): string | undefined => {
  const explicit = process.env.CHROMIUM_PATH;
  if (explicit && existsSync(explicit)) return explicit;

  const root = process.env.PLAYWRIGHT_BROWSERS_PATH;
  if (!root || !existsSync(root)) return undefined;

  let entries: string[];
  try {
    entries = readdirSync(root);
  } catch {
    return undefined;
  }

  // Prefer the full browser over the headless shell: the game needs a real
  // canvas and audio stack.
  const candidates = entries
    .filter((name) => name.startsWith('chromium-'))
    .concat(entries.filter((name) => name.startsWith('chromium_headless_shell-')));

  for (const dir of candidates) {
    for (const binary of ['chrome-linux/chrome', 'chrome-linux/headless_shell']) {
      const full = join(root, dir, binary);
      if (existsSync(full)) return full;
    }
  }
  return undefined;
};
