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
  /** The trail cycles through the spectrum instead of holding one colour. */
  rainbow?: boolean;
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
  {
    id: 'solar',
    name: 'Solar',
    starsRequired: 110,
    highlight: '#fffbe0',
    body: '#ffc53d',
    glow: 'rgba(255, 200, 70, 0.75)',
    trail: 'rgba(255, 210, 100, 0.7)',
  },
  {
    id: 'glacier',
    name: 'Glacier',
    starsRequired: 150,
    highlight: '#ffffff',
    body: '#d7f4ff',
    glow: 'rgba(200, 240, 255, 0.75)',
    trail: 'rgba(210, 245, 255, 0.7)',
  },
  {
    id: 'nebula',
    name: 'Nebula',
    starsRequired: 200,
    highlight: '#ffe0f6',
    body: '#ff6fcf',
    glow: 'rgba(255, 110, 210, 0.75)',
    trail: 'rgba(255, 140, 220, 0.7)',
  },
  {
    id: 'aurora',
    name: 'Aurora',
    starsRequired: 250,
    highlight: '#ffffff',
    body: '#9fffe0',
    glow: 'rgba(150, 255, 220, 0.8)',
    trail: 'rgba(150, 255, 220, 0.7)',
    rainbow: true,
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
