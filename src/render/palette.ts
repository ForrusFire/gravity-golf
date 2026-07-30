import type { BodyStyle, MaterialId } from '../physics/types';

export interface Palette {
  spaceTop: string;
  spaceBottom: string;
  nebulaA: string;
  nebulaB: string;
  star: string;
  fieldLine: string;
  ball: string;
  ballGlow: string;
  trail: string;
  hole: string;
  holeRim: string;
  collectible: string;
  collectibleGlow: string;
  aim: string;
  aimWeak: string;
  aimStrong: string;
  danger: string;
  text: string;
  textDim: string;
  accent: string;
  accentAlt: string;
  panel: string;
  panelBorder: string;
}

export const SPACE_PALETTE: Palette = {
  spaceTop: '#05060f',
  spaceBottom: '#0d0a20',
  nebulaA: 'rgba(88, 61, 168, 0.30)',
  nebulaB: 'rgba(29, 96, 148, 0.24)',
  star: 'rgba(226, 232, 255, 0.9)',
  fieldLine: 'rgba(140, 170, 255, 0.42)',
  ball: '#f7fafc',
  ballGlow: 'rgba(160, 220, 255, 0.55)',
  trail: 'rgba(150, 210, 255, 0.55)',
  hole: '#05070d',
  holeRim: '#66e6b8',
  collectible: '#ffd257',
  collectibleGlow: 'rgba(255, 210, 87, 0.45)',
  aim: 'rgba(232, 240, 255, 0.85)',
  aimWeak: '#7fe3a6',
  aimStrong: '#ff6b6b',
  danger: '#ff5c72',
  text: '#eef2ff',
  textDim: 'rgba(215, 223, 250, 0.62)',
  accent: '#5ad1ff',
  accentAlt: '#c792ff',
  panel: 'rgba(12, 14, 32, 0.88)',
  panelBorder: 'rgba(120, 150, 240, 0.28)',
};

/**
 * Higher-contrast variant. Every gameplay-critical colour also differs in
 * lightness, so the game stays readable without relying on hue alone.
 */
export const HIGH_CONTRAST_PALETTE: Palette = {
  ...SPACE_PALETTE,
  spaceTop: '#000000',
  spaceBottom: '#050510',
  nebulaA: 'rgba(70, 50, 140, 0.16)',
  nebulaB: 'rgba(20, 70, 120, 0.14)',
  fieldLine: 'rgba(190, 210, 255, 0.6)',
  ball: '#ffffff',
  ballGlow: 'rgba(255, 255, 255, 0.7)',
  holeRim: '#00ffa3',
  collectible: '#ffe600',
  aimWeak: '#00ff9c',
  aimStrong: '#ff2d55',
  danger: '#ff2d55',
  text: '#ffffff',
  textDim: 'rgba(255, 255, 255, 0.8)',
  accent: '#4fd8ff',
  panel: 'rgba(0, 0, 0, 0.94)',
  panelBorder: 'rgba(255, 255, 255, 0.5)',
};

export interface BodyColors {
  fill: string;
  fillDark: string;
  rim: string;
  glow: string | null;
}

const BODY_COLORS: Record<BodyStyle, BodyColors> = {
  planet: { fill: '#4a7fd4', fillDark: '#23386e', rim: '#8fc0ff', glow: 'rgba(90,150,255,0.28)' },
  moon: { fill: '#98a3c4', fillDark: '#4a5170', rim: '#d5dcf2', glow: 'rgba(180,195,235,0.22)' },
  crystal: { fill: '#b58cff', fillDark: '#5b3a9e', rim: '#e6d6ff', glow: 'rgba(181,140,255,0.38)' },
  wall: { fill: '#5b6486', fillDark: '#333a52', rim: '#98a2c6', glow: null },
  // Deliberately not orange: bumpers must never be mistaken for a sun, which
  // is the difference between a free bounce and a destroyed ball.
  bumper: { fill: '#28c9a4', fillDark: '#0d6b57', rim: '#9dffe6', glow: 'rgba(40,201,164,0.35)' },
  asteroid: { fill: '#7a6a58', fillDark: '#40372d', rim: '#b7a389', glow: null },
  sun: { fill: '#ffb037', fillDark: '#e0561d', rim: '#fff2c4', glow: 'rgba(255,140,40,0.5)' },
  blackhole: { fill: '#08060f', fillDark: '#000000', rim: '#a06bff', glow: 'rgba(150,80,255,0.45)' },
  ice: { fill: '#8fdcf5', fillDark: '#3d7f9c', rim: '#e2fbff', glow: 'rgba(150,230,255,0.3)' },
  sand: { fill: '#d8bd83', fillDark: '#8a7245', rim: '#f2e3bd', glow: null },
  goo: { fill: '#7bd66a', fillDark: '#3c7833', rim: '#c6f5bd', glow: 'rgba(120,220,100,0.3)' },
};

const MATERIAL_FALLBACK: Record<MaterialId, BodyStyle> = {
  rock: 'planet',
  bouncy: 'bumper',
  ice: 'ice',
  sand: 'sand',
  sticky: 'goo',
  metal: 'wall',
  lava: 'sun',
  void: 'blackhole',
};

export const colorsForBody = (style: BodyStyle | undefined, material: MaterialId): BodyColors =>
  BODY_COLORS[style ?? MATERIAL_FALLBACK[material]];

/** Interpolates the aim guide from safe green to dangerous red as power rises. */
export const powerColor = (power: number, palette: Palette): string => {
  const t = Math.max(0, Math.min(1, power));
  const parse = (hex: string): [number, number, number] => [
    parseInt(hex.slice(1, 3), 16),
    parseInt(hex.slice(3, 5), 16),
    parseInt(hex.slice(5, 7), 16),
  ];
  const [r1, g1, b1] = parse(palette.aimWeak);
  const [r2, g2, b2] = parse(palette.aimStrong);
  const mix = (a: number, b: number): number => Math.round(a + (b - a) * t);
  return `rgb(${mix(r1, r2)}, ${mix(g1, g2)}, ${mix(b1, b2)})`;
};
