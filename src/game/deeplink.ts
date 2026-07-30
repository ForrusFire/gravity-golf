/**
 * Sharable links to a specific hole.
 *
 * A web game that cannot be linked to is a game nobody passes on. Two shapes,
 * both readable at a glance in an address bar:
 *
 *   ?hole=c3-2     a campaign hole
 *   ?seed=1234567  a generated hole, rebuilt from its seed
 *
 * Seeds work because generation is deterministic and verified: the same seed
 * yields the same hole on every device, and it was proven completable before it
 * was ever shown, so a shared link cannot hand someone an impossible course.
 */
export type DeepLink =
  | { kind: 'campaign'; levelId: string }
  | { kind: 'generated'; seed: number };

/** Campaign ids are `c<chapter>-<hole>`; anything else is not one of ours. */
const CAMPAIGN_ID = /^c\d{1,3}-\d{1,3}$/;

/** Reads a deep link out of a query string, or null when there is not one. */
export const parseDeepLink = (search: string): DeepLink | null => {
  let params: URLSearchParams;
  try {
    params = new URLSearchParams(search);
  } catch {
    return null;
  }

  const hole = params.get('hole');
  if (hole && CAMPAIGN_ID.test(hole)) return { kind: 'campaign', levelId: hole };

  const seed = params.get('seed');
  if (seed !== null && /^\d{1,10}$/.test(seed)) {
    const value = Number(seed);
    // Beyond 2^32 the generator's RNG wraps, so the link would not reproduce
    // the hole it claims to.
    if (Number.isSafeInteger(value) && value >= 0 && value <= 0xffffffff) {
      return { kind: 'generated', seed: value };
    }
  }

  return null;
};

/** The query string for a link, including the leading `?`. Empty when unshareable. */
export const formatDeepLink = (link: DeepLink | null): string => {
  if (!link) return '';
  return link.kind === 'campaign' ? `?hole=${link.levelId}` : `?seed=${link.seed}`;
};

/**
 * A link for the hole being played, or null when it cannot be rebuilt from one.
 *
 * Generated holes carry their seed, so a daily or a random hole shares as a seed
 * link rather than as "today's daily" — which would be a different hole tomorrow
 * and so would not show a friend what you meant.
 */
export const linkForLevel = (level: {
  id: string;
  seed?: number;
}): DeepLink | null => {
  if (CAMPAIGN_ID.test(level.id)) return { kind: 'campaign', levelId: level.id };
  if (typeof level.seed === 'number' && Number.isSafeInteger(level.seed)) {
    return { kind: 'generated', seed: level.seed };
  }
  return null;
};

/** The full URL to share, built from the page's own location. */
export const shareUrl = (link: DeepLink, location: { origin: string; pathname: string }): string =>
  `${location.origin}${location.pathname}${formatDeepLink(link)}`;
