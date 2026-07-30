import { describe, expect, it } from 'vitest';
import { formatDeepLink, linkForLevel, parseDeepLink, shareUrl } from '../../src/game/deeplink';

describe('parseDeepLink', () => {
  it('reads a campaign hole', () => {
    expect(parseDeepLink('?hole=c3-2')).toEqual({ kind: 'campaign', levelId: 'c3-2' });
    expect(parseDeepLink('?hole=c10-6')).toEqual({ kind: 'campaign', levelId: 'c10-6' });
  });

  it('reads a generated seed', () => {
    expect(parseDeepLink('?seed=123456')).toEqual({ kind: 'generated', seed: 123456 });
    expect(parseDeepLink('?seed=0')).toEqual({ kind: 'generated', seed: 0 });
  });

  it('ignores anything it does not recognise', () => {
    // A link is untrusted input: it arrives from whatever someone pasted.
    for (const search of [
      '',
      '?',
      '?other=1',
      '?hole=',
      '?hole=../../etc/passwd',
      '?hole=<script>',
      '?hole=daily-2026-01-01',
      '?seed=abc',
      '?seed=-1',
      '?seed=1e9',
      '?seed=99999999999',
    ]) {
      expect(parseDeepLink(search), search).toBeNull();
    }
  });

  it('rejects a seed the generator could not reproduce', () => {
    // Past 2^32 the RNG wraps, so the link would not rebuild the hole it names.
    expect(parseDeepLink('?seed=4294967295')).toEqual({ kind: 'generated', seed: 0xffffffff });
    expect(parseDeepLink('?seed=4294967296')).toBeNull();
  });

  it('prefers a campaign hole when a link carries both', () => {
    expect(parseDeepLink('?hole=c1-1&seed=42')).toEqual({ kind: 'campaign', levelId: 'c1-1' });
  });
});

describe('links round-trip', () => {
  it('formats back to something it can parse', () => {
    for (const link of [
      { kind: 'campaign', levelId: 'c7-4' },
      { kind: 'generated', seed: 987654 },
    ] as const) {
      expect(parseDeepLink(formatDeepLink(link))).toEqual(link);
    }
  });

  it('has no query string for nothing', () => {
    expect(formatDeepLink(null)).toBe('');
  });
});

describe('linkForLevel', () => {
  it('links a campaign hole by id', () => {
    expect(linkForLevel({ id: 'c2-5' })).toEqual({ kind: 'campaign', levelId: 'c2-5' });
  });

  it('links a generated hole by seed, whatever it has been renamed to', () => {
    // The daily is re-id'd to `daily-YYYY-MM-DD`, but the seed is what rebuilds
    // it — and unlike the date, it means the same hole tomorrow.
    expect(linkForLevel({ id: 'daily-2026-07-30', seed: 4242 })).toEqual({
      kind: 'generated',
      seed: 4242,
    });
    expect(linkForLevel({ id: 'gen-77', seed: 77 })).toEqual({ kind: 'generated', seed: 77 });
  });

  it('gives up on a hole that cannot be rebuilt', () => {
    expect(linkForLevel({ id: 'mystery' })).toBeNull();
  });
});

describe('shareUrl', () => {
  it('keeps the page it was served from', () => {
    const location = { origin: 'https://example.com', pathname: '/gravity-golf/' };
    expect(shareUrl({ kind: 'campaign', levelId: 'c1-1' }, location)).toBe(
      'https://example.com/gravity-golf/?hole=c1-1',
    );
  });
});
