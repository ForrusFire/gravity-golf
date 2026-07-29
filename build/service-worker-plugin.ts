import { createHash } from 'node:crypto';
import { existsSync, readdirSync, statSync } from 'node:fs';
import { join, relative, sep } from 'node:path';
import type { Plugin } from 'vite';

/** Every file under `dir`, as paths relative to it, using forward slashes. */
const walk = (dir: string, root = dir): string[] => {
  const out: string[] = [];
  for (const entry of readdirSync(dir)) {
    const full = join(dir, entry);
    if (statSync(full).isDirectory()) out.push(...walk(full, root));
    else out.push(relative(root, full).split(sep).join('/'));
  }
  return out;
};

/**
 * Emits a service worker that precaches the exact files this build produced.
 *
 * Vite hashes asset filenames, so a hand-written precache list goes stale the
 * moment anything changes. Generating it from the bundle keeps the two in step,
 * and hashing the contents into the cache name means a new build always
 * replaces the old cache rather than serving a mix of the two.
 */
export const serviceWorkerPlugin = (): Plugin => {
  let publicDir = '';

  return {
  name: 'gravity-golf:service-worker',
  apply: 'build',

  configResolved(config) {
    publicDir = config.publicDir;
  },

  generateBundle(_options, bundle) {
    const assets: string[] = ['./'];
    const hash = createHash('sha256');

    // Files in publicDir are copied verbatim and never appear in the bundle,
    // so the manifest and icons have to be gathered from disk or the installed
    // game would be missing exactly the files that make it installable.
    if (publicDir && existsSync(publicDir)) {
      for (const file of walk(publicDir)) {
        assets.push(`./${file}`);
        hash.update(file);
      }
    }

    for (const [fileName, output] of Object.entries(bundle)) {
      // Source maps are large and only fetched by devtools.
      if (fileName.endsWith('.map')) continue;
      assets.push(`./${fileName}`);
      const content =
        output.type === 'chunk' ? output.code : typeof output.source === 'string'
          ? output.source
          : Buffer.from(output.source);
      hash.update(fileName);
      hash.update(content);
    }

    assets.sort();
    const version = hash.digest('hex').slice(0, 12);

    this.emitFile({
      type: 'asset',
      fileName: 'sw.js',
      source: serviceWorkerSource(version, assets),
    });
  },
  };
};

const serviceWorkerSource = (version: string, assets: string[]): string => `/* Generated at build time. Do not edit. */
const CACHE = 'gravity-golf-${version}';
const ASSETS = ${JSON.stringify(assets, null, 2)};

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches
      .open(CACHE)
      // One missing asset must not fail the whole install and leave the game
      // uncacheable, so each is added independently.
      .then((cache) => Promise.all(ASSETS.map((url) => cache.add(url).catch(() => undefined))))
      .then(() => self.skipWaiting()),
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((keys) => Promise.all(keys.filter((key) => key !== CACHE).map((key) => caches.delete(key))))
      .then(() => self.clients.claim()),
  );
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  if (request.method !== 'GET') return;

  const url = new URL(request.url);
  if (url.origin !== self.location.origin) return;

  // ignoreVary matters: a precached response stored by cache.add() carries the
  // server's Vary header, and the browser sends a different Origin header for a
  // module script than for that plain fetch. Without this, every asset misses
  // the cache offline and the game boots to a blank shell.
  const MATCH = { ignoreVary: true };

  // Navigations fall back to the cached shell so the game opens offline.
  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request).catch(() =>
        caches
          .match('./', MATCH)
          .then((cached) => cached ?? caches.match('./index.html', MATCH))
          .then(
            (cached) => cached ?? new Response('Offline', { status: 503, statusText: 'Offline' }),
          ),
      ),
    );
    return;
  }

  event.respondWith(
    caches.match(request, MATCH).then((cached) => {
      if (cached) return cached;
      return fetch(request)
        .then((response) => {
          // Only cache complete, same-origin successes.
          if (response.ok && response.type === 'basic') {
            const copy = response.clone();
            caches.open(CACHE).then((cache) => cache.put(request, copy)).catch(() => undefined);
          }
          return response;
        })
        .catch(() => new Response('Offline', { status: 503, statusText: 'Offline' }));
    }),
  );
});
`;
