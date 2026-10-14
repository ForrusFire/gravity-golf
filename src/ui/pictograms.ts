const SVG = 'http://www.w3.org/2000/svg';

/**
 * Small pictures of the things the help screen describes, in the colours the
 * game draws them in — so the legend teaches what is actually on screen.
 */
const ART = {
  aim: `<circle cx="20" cy="12" r="4" fill="#e9eeff"/><path d="M17 15 7 23" stroke="#4fd8ff" stroke-width="2" stroke-dasharray="2 3" stroke-linecap="round"/><circle cx="20" cy="12" r="7" fill="none" stroke="#ffd25a" stroke-width="2" stroke-dasharray="30 44" stroke-linecap="round"/>`,
  gravity: `<circle cx="14" cy="16" r="7" fill="#4a7fd4"/><path d="M8 14c3-2 9-2 12 0" stroke="#8fc0ff" stroke-width="1.2" opacity=".6"/><path d="M4 4l4 4M24 4l-4 4M4 26l3-3M26 26l-3-3" stroke="#8ca4ff" stroke-width="1.8" stroke-linecap="round"/>`,
  star: `<path d="M14 3l3.2 6.6 7.2 1-5.2 5.1 1.2 7.2L14 19.5l-6.4 3.4 1.2-7.2L3.6 10.6l7.2-1z" fill="#ffd25a" stroke="#fff3c4" stroke-width="1" stroke-linejoin="round"/>`,
  hole: `<circle cx="13" cy="20" r="5.5" fill="#05060d" stroke="#66e6b8" stroke-width="2"/><path d="M13 20V4" stroke="#c9d2ef" stroke-width="1.6"/><path d="M13 4l9 3-9 3z" fill="#66e6b8"/>`,
  directional: `<path d="M14 14 4 6v16z" fill="#66e6b8" opacity=".25"/><circle cx="16" cy="14" r="5.5" fill="#05060d" stroke="#66e6b8" stroke-width="2"/><path d="M3 14h7M8 11l3 3-3 3" stroke="#66e6b8" stroke-width="1.8" fill="none" stroke-linecap="round"/>`,
  moving: `<path d="M5 22a10 10 0 0 1 18 0" stroke="#66e6b8" stroke-width="1.5" stroke-dasharray="2 3" fill="none"/><circle cx="20" cy="12" r="4.5" fill="#05060d" stroke="#66e6b8" stroke-width="2"/><path d="M8 9l3-3" stroke="#66e6b8" stroke-width="1.6" stroke-linecap="round"/>`,
  hazard: `<circle cx="14" cy="14" r="6.5" fill="#ffb037"/><g stroke="#ff5c72" stroke-width="1.8" stroke-linecap="round"><path d="M14 2v3M14 23v3M2 14h3M23 14h3M5.5 5.5l2 2M20.5 20.5l2 2M5.5 22.5l2-2M20.5 7.5l2-2"/></g>`,
  undo: `<path d="M10 16 5 11l5-5" stroke="#e9eeff" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"/><path d="M5 11h11a6 6 0 0 1 0 12h-3" stroke="#e9eeff" stroke-width="2" fill="none" stroke-linecap="round"/>`,
  hint: `<path d="M10.5 21h7M11.5 24.5h5" stroke="#ffd25a" stroke-width="2" stroke-linecap="round"/><path d="M14 3a7 7 0 0 0-4.2 12.6c.8.7 1.2 1.5 1.2 2.4h6c0-.9.4-1.7 1.2-2.4A7 7 0 0 0 14 3z" fill="#ffd25a" opacity=".85"/>`,
  tides: `<rect x="4" y="5" width="20" height="18" rx="2" fill="#ff5c72" opacity=".18" stroke="#ff5c72" stroke-width="1.4" stroke-dasharray="3 2"/><path d="M14 9a5 5 0 1 1-.01 0" stroke="#66e6b8" stroke-width="2" fill="none" stroke-linecap="round"/>`,
  pulse: `<rect x="11" y="3" width="6" height="22" rx="3" fill="#5b6486" stroke="#98a2c6" stroke-width="1.2"/><circle cx="14" cy="14" r="11" fill="none" stroke="#ffa05a" stroke-width="2" stroke-dasharray="48 70" transform="rotate(-90 14 14)"/>`,
  timer: `<circle cx="14" cy="14" r="5" fill="none" stroke="#c792ff" stroke-width="2"/><circle cx="14" cy="14" r="10" fill="none" stroke="#66e6b8" stroke-width="2.4" stroke-dasharray="40 63" transform="rotate(-90 14 14)"/>`,
  sealed: `<circle cx="14" cy="17" r="7" fill="#05060d" stroke="#ffd25a" stroke-width="2"/><path d="M8.5 15h11M8.5 19h11" stroke="#ffd25a" stroke-width="1.6"/><circle cx="10" cy="5" r="1.6" fill="#ffd25a"/><circle cx="14" cy="5" r="1.6" fill="#ffd25a"/><circle cx="18" cy="5" r="1.6" fill="#ffd25a"/>`,
  toll: `<rect x="3" y="11" width="22" height="7" rx="3.5" fill="#5b6486" stroke="#98a2c6" stroke-width="1.2"/><path d="M10 12l1.2 2.4 2.6.4-1.9 1.8.5 2.6-2.4-1.2-2.4 1.2.5-2.6-1.9-1.8 2.6-.4zM18 12l1.2 2.4 2.6.4-1.9 1.8.5 2.6-2.4-1.2-2.4 1.2.5-2.6-1.9-1.8 2.6-.4z" fill="#ffd25a"/>`,
  keyboard: `<rect x="2.5" y="7" width="23" height="14" rx="2.5" fill="none" stroke="#e9eeff" stroke-width="1.6"/><path d="M6 11h2M10 11h2M14 11h2M18 11h4M6 15h2M20 15h2M9 17.5h10" stroke="#e9eeff" stroke-width="1.6" stroke-linecap="round"/>`,
  camera: `<circle cx="12" cy="12" r="6" fill="none" stroke="#e9eeff" stroke-width="1.8"/><path d="M16.5 16.5 23 23" stroke="#e9eeff" stroke-width="2.2" stroke-linecap="round"/><path d="M9.5 12h5M12 9.5v5" stroke="#4fd8ff" stroke-width="1.6" stroke-linecap="round"/>`,
} as const;

export type PictogramName = keyof typeof ART;

export const pictogram = (name: PictogramName): SVGSVGElement => {
  const svg = document.createElementNS(SVG, 'svg');
  svg.setAttribute('viewBox', '0 0 28 28');
  svg.setAttribute('width', '28');
  svg.setAttribute('height', '28');
  svg.setAttribute('aria-hidden', 'true');
  svg.innerHTML = ART[name];
  return svg;
};
