type Child = Node | string | number | null | undefined | false;

export interface ElementOptions {
  class?: string;
  text?: string;
  html?: string;
  title?: string;
  id?: string;
  /** Applied via setAttribute — use for aria-*, role, data-*, type, etc. */
  attrs?: Record<string, string | number | boolean | null | undefined>;
  style?: Partial<CSSStyleDeclaration>;
  on?: Partial<{
    [K in keyof HTMLElementEventMap]: (event: HTMLElementEventMap[K]) => void;
  }>;
}

/** Terse, typed element construction. */
export const el = <K extends keyof HTMLElementTagNameMap>(
  tag: K,
  options: ElementOptions = {},
  children: Child[] = [],
): HTMLElementTagNameMap[K] => {
  const node = document.createElement(tag);
  if (options.class) node.className = options.class;
  if (options.id) node.id = options.id;
  if (options.text !== undefined) node.textContent = options.text;
  if (options.html !== undefined) node.innerHTML = options.html;
  if (options.title) node.title = options.title;

  for (const [key, value] of Object.entries(options.attrs ?? {})) {
    if (value === null || value === undefined || value === false) continue;
    node.setAttribute(key, String(value));
  }
  if (options.style) Object.assign(node.style, options.style);
  for (const [type, handler] of Object.entries(options.on ?? {})) {
    node.addEventListener(type, handler as EventListener);
  }

  for (const child of children) {
    if (child === null || child === undefined || child === false) continue;
    node.append(typeof child === 'string' || typeof child === 'number' ? String(child) : child);
  }
  return node;
};

export const clear = (node: Element): void => {
  while (node.firstChild) node.removeChild(node.firstChild);
};

/** A button that is reachable by keyboard and announces itself correctly. */
export const button = (
  label: string,
  onClick: () => void,
  options: ElementOptions = {},
): HTMLButtonElement =>
  el(
    'button',
    {
      ...options,
      class: `btn ${options.class ?? ''}`.trim(),
      attrs: { type: 'button', ...options.attrs },
      on: { click: onClick, ...options.on },
    },
    [label],
  );

export const icon = (glyph: string, label: string): HTMLSpanElement =>
  el('span', { class: 'icon', text: glyph, attrs: { 'aria-hidden': 'true', title: label } });

/** Formats seconds as m:ss. */
export const formatTime = (seconds: number): string => {
  if (!Number.isFinite(seconds)) return '--:--';
  const total = Math.max(0, Math.floor(seconds));
  const m = Math.floor(total / 60);
  const s = total % 60;
  return `${m}:${s.toString().padStart(2, '0')}`;
};

/**
 * Renders a stroke count relative to par, e.g. "-2", "E", "+3".
 * Before the first stroke there is no score to report yet, so it shows a dash.
 */
export const formatToPar = (strokes: number, par: number): string => {
  if (strokes === 0) return '—';
  const diff = strokes - par;
  if (diff === 0) return 'E';
  return diff > 0 ? `+${diff}` : `${diff}`;
};

export const starRow = (earned: number, total = 3): HTMLElement =>
  el(
    'span',
    {
      class: 'stars',
      attrs: { 'aria-label': `${earned} of ${total} stars` },
    },
    Array.from({ length: total }, (_, i) =>
      el('span', { class: `star ${i < earned ? 'star--on' : ''}`, text: '★' }),
    ),
  );
