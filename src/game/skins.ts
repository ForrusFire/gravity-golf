/**
 * Ball appearances, unlocked by collecting stars.
 *
 * Stars previously only gated chapters, which meant that once a chapter was
 * open there was no reason to chase the ones you had missed. Cosmetics give
 * them a second, permanent purpose.
 */
export interface BallSkin {
  id: string;
  name: string;
  /** Stars needed across the campaign to unlock it. */
  starsRequired: number;
  /** Centre highlight and body colour of the ball. */
  highlight: string;
  body: string;
  /** Glow around the ball. */
  glow: string;
  /** Colour of the flight trail. */
  trail: string;
}

export const BALL_SKINS: BallSkin[] = [
  {
    id: 'classic',
    name: 'Classic',
    starsRequired: 0,
    highlight: '#ffffff',
    body: '#b9c6dd',
    glow: 'rgba(160, 220, 255, 0.55)',
    trail: 'rgba(150, 210, 255, 0.55)',
  },
  {
    id: 'comet',
    name: 'Comet',
    starsRequired: 12,
    highlight: '#ffffff',
    body: '#8fd8ff',
    glow: 'rgba(90, 209, 255, 0.7)',
    trail: 'rgba(120, 225, 255, 0.7)',
  },
  {
    id: 'ember',
    name: 'Ember',
    starsRequired: 30,
    highlight: '#fff3d0',
    body: '#ff9f45',
    glow: 'rgba(255, 140, 60, 0.7)',
    trail: 'rgba(255, 170, 90, 0.65)',
  },
  {
    id: 'neon',
    name: 'Neon',
    starsRequired: 50,
    highlight: '#eaffef',
    body: '#4dffa8',
    glow: 'rgba(60, 255, 160, 0.7)',
    trail: 'rgba(90, 255, 180, 0.65)',
  },
  {
    id: 'void',
    name: 'Void',
    starsRequired: 75,
    highlight: '#d8c4ff',
    body: '#6d47c4',
    glow: 'rgba(160, 90, 255, 0.75)',
    trail: 'rgba(190, 130, 255, 0.7)',
  },
];

export const DEFAULT_SKIN = BALL_SKINS[0]!;

export const isSkinUnlocked = (skin: BallSkin, stars: number): boolean =>
  stars >= skin.starsRequired;

/**
 * The skin to draw with. Falls back to the default when the chosen one is
 * unknown or not yet unlocked, so a reset or an edited save cannot leave the
 * ball invisible.
 */
export const resolveSkin = (id: string, stars: number): BallSkin => {
  const skin = BALL_SKINS.find((candidate) => candidate.id === id);
  if (!skin || !isSkinUnlocked(skin, stars)) return DEFAULT_SKIN;
  return skin;
};

/** The next skin still to unlock, for a "keep collecting" hint. */
export const nextLockedSkin = (stars: number): BallSkin | undefined =>
  BALL_SKINS.find((skin) => !isSkinUnlocked(skin, stars));
