import type { Medal } from '../game/session';

const SVG = 'http://www.w3.org/2000/svg';

/** Metal per medal: face light, face dark, rim, ribbon. */
const METAL: Record<Medal, [string, string, string, string]> = {
  ace: ['#fff4b8', '#f0a91c', '#ffe27a', '#7c4dff'],
  gold: ['#ffe9a0', '#d99a22', '#ffd25a', '#2f7fe0'],
  silver: ['#f4f7ff', '#9aa6c4', '#dfe5f5', '#3a8f78'],
  bronze: ['#ffd2a8', '#b06a35', '#e9a876', '#8a3a4a'],
  none: ['#d8def5', '#6d7799', '#b9c2e0', '#4a5272'],
};

let uid = 0;

/**
 * A drawn medal — a ribboned disc with a star, or a trophy cup for a hole in
 * one — so the result looks the same on every device, unlike an emoji.
 */
export const medalBadge = (medal: Medal, size = 64): SVGSVGElement => {
  const [light, dark, rim, ribbon] = METAL[medal];
  const id = `medal-${++uid}`;
  const svg = document.createElementNS(SVG, 'svg');
  svg.setAttribute('viewBox', '0 0 64 64');
  svg.setAttribute('width', String(size));
  svg.setAttribute('height', String(size));
  svg.setAttribute('aria-hidden', 'true');
  svg.classList.add('medal', `medal--${medal}`);
  const face =
    medal === 'ace'
      ? // A trophy cup.
        `<path d="M20 12h24v10a12 12 0 0 1-24 0z" fill="url(#${id}-f)" stroke="${rim}" stroke-width="1.5"/>
         <path d="M20 15h-6a6 6 0 0 0 8 9M44 15h6a6 6 0 0 1-8 9" fill="none" stroke="${rim}" stroke-width="2.5" stroke-linecap="round"/>
         <rect x="29" y="33" width="6" height="9" fill="${dark}"/>
         <path d="M21 50h22l-3-8H24z" fill="url(#${id}-f)" stroke="${rim}" stroke-width="1.5" stroke-linejoin="round"/>
         <path d="M32 15l2.4 4.9 5.4.8-3.9 3.8.9 5.4-4.8-2.6-4.8 2.6.9-5.4-3.9-3.8 5.4-.8z" fill="#fff" opacity="0.92"/>
         <path d="M23 14q2 8 5 11" stroke="#fff" stroke-width="2" opacity="0.5" fill="none" stroke-linecap="round"/>`
      : `<path d="M22 4l6 18h8l6-18z" fill="${ribbon}"/>
         <path d="M22 4l5 18h3L26 4z" fill="#fff" opacity="0.18"/>
         <circle cx="32" cy="38" r="20" fill="url(#${id}-f)" stroke="${rim}" stroke-width="2"/>
         <circle cx="32" cy="38" r="15" fill="none" stroke="${dark}" stroke-width="1.2" opacity="0.55"/>
         <path d="M32 27l3.2 6.6 7.2 1-5.2 5.1 1.2 7.2-6.4-3.4-6.4 3.4 1.2-7.2-5.2-5.1 7.2-1z" fill="${light}" stroke="${dark}" stroke-width="1" stroke-linejoin="round"/>
         <path d="M18 32a15 15 0 0 1 10-11" stroke="#fff" stroke-width="2.4" opacity="0.55" fill="none" stroke-linecap="round"/>`;
  svg.innerHTML = `<defs><linearGradient id="${id}-f" x1="0" y1="0" x2="0.4" y2="1">
      <stop offset="0" stop-color="${light}"/><stop offset="1" stop-color="${dark}"/>
    </linearGradient></defs>${face}`;
  return svg;
};
