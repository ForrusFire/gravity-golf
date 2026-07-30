import type { Chapter } from '../game/levels';
import type { LevelDef } from '../game/level';
import type { ProgressStore, Settings } from '../game/progress';
import { FEATS, type FeatId, type HoleResult, type Medal } from '../game/session';
import { BALL_SKINS, isSkinUnlocked } from '../game/skins';
import { button, clear, el, formatTime, formatToPar, starRow } from './dom';

const MEDAL_LABEL: Record<Medal, string> = {
  ace: 'Hole in one',
  gold: 'Under par',
  silver: 'Par',
  bronze: 'Over par',
  none: 'Completed',
};

const MEDAL_GLYPH: Record<Medal, string> = {
  ace: '🏆',
  gold: '🥇',
  silver: '🥈',
  bronze: '🥉',
  none: '⛳',
};

/**
 * A single full-screen overlay. Only one is visible at a time; showing a panel
 * moves focus into it and Escape closes it when it is dismissable.
 */
export class Overlay {
  readonly root: HTMLElement;
  private panel: HTMLElement;
  private onEscape: (() => void) | null = null;
  private lastFocused: Element | null = null;

  constructor() {
    this.panel = el('div', {
      class: 'panel',
      attrs: { role: 'dialog', 'aria-modal': 'true', tabindex: '-1' },
    });
    this.root = el('div', { class: 'overlay overlay--hidden' }, [this.panel]);
    this.root.addEventListener('keydown', (event) => {
      if (event.key === 'Escape' && this.onEscape) {
        event.stopPropagation();
        this.onEscape();
      }
      if (event.key === 'Tab') this.trapFocus(event);
    });
  }

  get visible(): boolean {
    return !this.root.classList.contains('overlay--hidden');
  }

  show(content: HTMLElement, options: { onEscape?: () => void; wide?: boolean } = {}): void {
    this.lastFocused = document.activeElement;
    clear(this.panel);
    this.panel.classList.toggle('panel--wide', options.wide === true);
    this.panel.append(content);
    this.onEscape = options.onEscape ?? null;
    this.root.classList.remove('overlay--hidden');
    // Focus the first control so keyboard users land inside the dialog.
    const focusable = this.focusable();
    (focusable[0] ?? this.panel).focus();
  }

  hide(): void {
    if (!this.visible) return;
    this.root.classList.add('overlay--hidden');
    this.onEscape = null;
    clear(this.panel);
    if (this.lastFocused instanceof HTMLElement) this.lastFocused.focus();
  }

  private focusable(): HTMLElement[] {
    return Array.from(
      this.panel.querySelectorAll<HTMLElement>(
        'button:not([disabled]), input, select, [tabindex]:not([tabindex="-1"])',
      ),
    );
  }

  private trapFocus(event: KeyboardEvent): void {
    const items = this.focusable();
    if (items.length === 0) return;
    const first = items[0]!;
    const last = items[items.length - 1]!;
    if (event.shiftKey && document.activeElement === first) {
      event.preventDefault();
      last.focus();
    } else if (!event.shiftKey && document.activeElement === last) {
      event.preventDefault();
      first.focus();
    }
  }
}

/* ----------------------------------------------------------------- title */

export interface TitleCallbacks {
  onPlay(): void;
  onLevels(): void;
  onSettings(): void;
  onHelp(): void;
  onScorecard(): void;
  onDaily(): void;
  onRandom(): void;
}

export const titleScreen = (
  callbacks: TitleCallbacks,
  progress: ProgressStore,
  campaignIds: readonly string[],
  dailyDone = false,
): HTMLElement => {
  const totalLevels = campaignIds.length;
  const completed = progress.completedCount(campaignIds);
  const stars = progress.totalStars(campaignIds);
  return el('div', { class: 'title' }, [
    el('h1', { class: 'title__logo' }, [
      el('span', { class: 'title__logo-main', text: 'GRAVITY' }),
      el('span', { class: 'title__logo-sub', text: 'GOLF' }),
    ]),
    el('p', {
      class: 'title__tagline',
      text: 'Putt through orbital mechanics. Let the planets do the work.',
    }),
    el('div', { class: 'title__buttons' }, [
      button(completed > 0 ? 'Continue' : 'Play', callbacks.onPlay, { class: 'btn--primary' }),
      button('Select hole', callbacks.onLevels),
      button(dailyDone ? "Daily challenge ✓" : 'Daily challenge', callbacks.onDaily),
      button('Random hole', callbacks.onRandom),
      button('Scorecard', callbacks.onScorecard),
      button('How to play', callbacks.onHelp),
      button('Settings', callbacks.onSettings),
    ]),
    el('p', {
      class: 'title__progress',
      text: `${completed} / ${totalLevels} holes · ${stars} ★ collected`,
    }),
  ]);
};

/* ---------------------------------------------------------- level select */

export interface LevelSelectCallbacks {
  onPick(level: LevelDef): void;
  onBack(): void;
}

/** A hole is playable once the one before it is done. */
export const isLevelUnlocked = (
  index: number,
  ordered: LevelDef[],
  progress: ProgressStore,
): boolean => {
  if (index === 0) return true;
  const previous = ordered[index - 1];
  return previous ? progress.isCompleted(previous.id) : false;
};

export const levelSelectScreen = (
  chapters: Chapter[],
  ordered: LevelDef[],
  progress: ProgressStore,
  callbacks: LevelSelectCallbacks,
): HTMLElement => {
  const body = el('div', { class: 'levels' });

  const campaignIds = ordered.map((level) => level.id);

  for (const chapter of chapters) {
    // Chapter gates count campaign stars only — a generated hole is optional
    // content and must not unlock the main course.
    const starsHeld = progress.totalStars(campaignIds);
    const chapterLocked = starsHeld < chapter.starsRequired;

    body.append(
      el('div', { class: 'levels__chapter-head' }, [
        el('h3', { text: `${chapter.index + 1}. ${chapter.name}` }),
        el('span', {
          class: 'levels__chapter-sub',
          text: chapterLocked
            ? `Locked — needs ${chapter.starsRequired} ★ (you have ${starsHeld})`
            : chapter.subtitle,
        }),
      ]),
    );

    const grid = el('div', { class: 'levels__grid' });
    for (const level of chapter.levels) {
      const index = ordered.indexOf(level);
      const record = progress.recordOf(level.id);
      const unlocked = !chapterLocked && isLevelUnlocked(index, ordered, progress);
      const best = record?.completed ? record.bestStrokes : null;

      const card = el(
        'button',
        {
          class: `level-card ${unlocked ? '' : 'level-card--locked'} ${
            record?.completed ? 'level-card--done' : ''
          }`.trim(),
          attrs: {
            type: 'button',
            disabled: !unlocked,
            'aria-label': unlocked
              ? `${level.name}, par ${level.par}${best ? `, best ${best} strokes` : ''}`
              : `${level.name}, locked`,
          },
          on: { click: () => unlocked && callbacks.onPick(level) },
        },
        [
          el('span', { class: 'level-card__num', text: unlocked ? String(index + 1) : '🔒' }),
          el('span', { class: 'level-card__name', text: level.name }),
          el('span', { class: 'level-card__meta' }, [
            el('span', { text: `Par ${level.par}` }),
            best !== null ? el('span', { class: 'level-card__best', text: `Best ${best}` }) : null,
          ]),
          starRow(record?.bestStars ?? 0),
          record?.bestMedal && record.bestMedal !== 'none'
            ? el('span', { class: 'level-card__medal', text: MEDAL_GLYPH[record.bestMedal] })
            : null,
        ],
      );
      grid.append(card);
    }
    body.append(grid);
  }

  return el('div', { class: 'screen' }, [
    el('div', { class: 'screen__head' }, [
      el('h2', { text: 'Select a hole' }),
      button('Back', callbacks.onBack, { class: 'btn--ghost' }),
    ]),
    body,
  ]);
};

/* --------------------------------------------------------------- results */

export interface ResultsCallbacks {
  onNext(): void;
  onRetry(): void;
  onLevels(): void;
  hasNext: boolean;
  isNewBest: boolean;
  previousBest: number | null;
}

export const resultsScreen = (
  result: HoleResult,
  levelName: string,
  callbacks: ResultsCallbacks,
): HTMLElement =>
  el('div', { class: 'results' }, [
    el('div', { class: 'results__medal', text: MEDAL_GLYPH[result.medal] }),
    el('h2', { class: 'results__title', text: MEDAL_LABEL[result.medal] }),
    el('p', { class: 'results__level', text: levelName }),
    callbacks.isNewBest ? el('p', { class: 'results__best', text: '★ New personal best!' }) : null,
    el('div', { class: 'results__grid' }, [
      statBlock('Strokes', String(result.strokes)),
      statBlock('Par', String(result.par)),
      statBlock('To par', formatToPar(result.strokes, result.par)),
      statBlock('Time', formatTime(result.time)),
    ]),
    el('div', { class: 'results__stars' }, [
      starRow(result.stars),
      el('span', {
        class: 'results__stars-label',
        text: result.stars === 3 ? 'All stars collected' : `${result.stars} of 3 stars`,
      }),
    ]),
    result.feats.length > 0
      ? el(
          'div',
          { class: 'results__feats' },
          result.feats.map((id) =>
            el('span', { class: 'feat-chip', title: FEATS[id].description }, [
              el('span', { class: 'feat-chip__icon', text: '✦' }),
              FEATS[id].label,
            ]),
          ),
        )
      : null,
    // Said plainly rather than hidden: the medal stands, the style awards do not.
    result.hinted
      ? el('p', {
          class: 'results__hinted',
          text: '💡 Hint used — no feats or ghost recorded for this run',
        })
      : null,
    callbacks.previousBest !== null
      ? el('p', { class: 'results__previous', text: `Previous best: ${callbacks.previousBest}` })
      : null,
    el('div', { class: 'results__buttons' }, [
      callbacks.hasNext
        ? button('Next hole', callbacks.onNext, { class: 'btn--primary' })
        : button('Back to holes', callbacks.onLevels, { class: 'btn--primary' }),
      button('Retry', callbacks.onRetry),
      callbacks.hasNext ? button('All holes', callbacks.onLevels, { class: 'btn--ghost' }) : null,
    ]),
  ]);

const statBlock = (label: string, value: string): HTMLElement =>
  el('div', { class: 'stat-block' }, [
    el('span', { class: 'stat-block__value', text: value }),
    el('span', { class: 'stat-block__label', text: label }),
  ]);

/* ----------------------------------------------------------------- pause */

export interface PauseCallbacks {
  onResume(): void;
  onRetry(): void;
  onLevels(): void;
  onSettings(): void;
  onTitle(): void;
}

export const pauseScreen = (level: LevelDef, callbacks: PauseCallbacks): HTMLElement =>
  el('div', { class: 'menu' }, [
    el('h2', { text: 'Paused' }),
    el('p', { class: 'menu__sub', text: `${level.name} · Par ${level.par}` }),
    el('div', { class: 'menu__buttons' }, [
      button('Resume', callbacks.onResume, { class: 'btn--primary' }),
      button('Restart hole', callbacks.onRetry),
      button('Select hole', callbacks.onLevels),
      button('Settings', callbacks.onSettings),
      button('Main menu', callbacks.onTitle, { class: 'btn--ghost' }),
    ]),
  ]);

/* -------------------------------------------------------------- settings */

export const settingsScreen = (
  settings: Settings,
  stars: number,
  onChange: (patch: Partial<Settings>) => void,
  onBack: () => void,
  onResetProgress: () => void,
): HTMLElement => {
  const slider = (
    label: string,
    value: number,
    min: number,
    max: number,
    step: number,
    apply: (v: number) => void,
    format: (v: number) => string = (v) => `${Math.round(v * 100)}%`,
  ): HTMLElement => {
    const readout = el('span', { class: 'setting__value', text: format(value) });
    const input = el('input', {
      class: 'setting__slider',
      attrs: { type: 'range', min, max, step, value, 'aria-label': label },
      on: {
        input: (event) => {
          const v = Number((event.target as HTMLInputElement).value);
          readout.textContent = format(v);
          apply(v);
        },
      },
    });
    return el('label', { class: 'setting' }, [
      el('span', { class: 'setting__label', text: label }),
      input,
      readout,
    ]);
  };

  const toggle = (
    label: string,
    value: boolean,
    apply: (v: boolean) => void,
    description?: string,
  ): HTMLElement =>
    el('label', { class: 'setting setting--toggle' }, [
      el('span', { class: 'setting__label' }, [
        label,
        description ? el('small', { class: 'setting__desc', text: description }) : null,
      ]),
      el('input', {
        attrs: { type: 'checkbox', checked: value, 'aria-label': label },
        on: {
          change: (event) => apply((event.target as HTMLInputElement).checked),
        },
      }),
    ]);

  return el('div', { class: 'screen' }, [
    el('div', { class: 'screen__head' }, [
      el('h2', { text: 'Settings' }),
      button('Back', onBack, { class: 'btn--ghost' }),
    ]),
    el('div', { class: 'settings' }, [
      el('h3', { text: 'Audio' }),
      slider('Sound effects', settings.sfxVolume, 0, 1, 0.05, (v) => onChange({ sfxVolume: v })),
      slider('Music', settings.musicVolume, 0, 1, 0.05, (v) => onChange({ musicVolume: v })),

      el('h3', { text: 'Aiming' }),
      slider(
        'Trajectory preview',
        settings.aimAssist,
        0,
        4,
        0.2,
        (v) => onChange({ aimAssist: v }),
        (v) => (v === 0 ? 'Off' : `${v.toFixed(1)}s ahead`),
      ),
      el('label', { class: 'setting' }, [
        el('span', { class: 'setting__label', text: 'Aim style' }),
        el(
          'select',
          {
            class: 'setting__select',
            attrs: { 'aria-label': 'Aim style' },
            on: {
              change: (event) =>
                onChange({
                  aimMode: (event.target as HTMLSelectElement).value as Settings['aimMode'],
                }),
            },
          },
          [
            el('option', {
              attrs: { value: 'slingshot', selected: settings.aimMode === 'slingshot' },
              text: 'Slingshot (drag back)',
            }),
            el('option', {
              attrs: { value: 'direct', selected: settings.aimMode === 'direct' },
              text: 'Direct (drag toward)',
            }),
          ],
        ),
      ]),

      toggle(
        'Race your best run',
        settings.showGhost,
        (v) => onChange({ showGhost: v }),
        'Replays your best attempt as a faint ghost ball',
      ),

      el('h3', { text: 'Ball' }),
      el('div', { class: 'skins' }, BALL_SKINS.map((skin) => {
        const unlocked = isSkinUnlocked(skin, stars);
        return el(
          'button',
          {
            class: `skin ${settings.ballSkin === skin.id && unlocked ? 'skin--active' : ''} ${
              unlocked ? '' : 'skin--locked'
            }`.trim(),
            attrs: {
              type: 'button',
              disabled: !unlocked,
              'aria-label': unlocked
                ? `${skin.name} ball`
                : `${skin.name} ball, locked, needs ${skin.starsRequired} stars`,
            },
            on: { click: () => unlocked && onChange({ ballSkin: skin.id }) },
          },
          [
            el('span', {
              class: 'skin__swatch',
              style: {
                background: `radial-gradient(circle at 35% 32%, ${skin.highlight}, ${skin.body})`,
                boxShadow: `0 0 12px ${skin.glow}`,
              },
            }),
            el('span', { class: 'skin__name', text: skin.name }),
            el('span', {
              class: 'skin__req',
              text: unlocked ? 'Unlocked' : `${skin.starsRequired} ★`,
            }),
          ],
        );
      })),

      el('h3', { text: 'Display' }),
      toggle(
        'Gravity field overlay',
        settings.showGravityField,
        (v) => onChange({ showGravityField: v }),
        'Shows which way each point in space pulls',
      ),
      toggle('Screen shake', settings.screenShake, (v) => onChange({ screenShake: v })),
      toggle(
        'Reduced motion',
        settings.reducedMotion,
        (v) => onChange({ reducedMotion: v }),
        'Calms background animation and effects',
      ),
      toggle(
        'High contrast',
        settings.highContrast,
        (v) => onChange({ highContrast: v }),
        'Brighter palette, less reliance on colour alone',
      ),

      el('h3', { text: 'Data' }),
      el('div', { class: 'settings__danger' }, [
        button('Reset all progress', onResetProgress, { class: 'btn--danger' }),
      ]),
    ]),
  ]);
};

/* ------------------------------------------------------------- scorecard */

export interface ScorecardRow {
  level: LevelDef;
  index: number;
  played: boolean;
  strokes: number | null;
  stars: number;
  medal: Medal;
}

export const buildScorecard = (levels: LevelDef[], progress: ProgressStore): ScorecardRow[] =>
  levels.map((level, index) => {
    const record = progress.recordOf(level.id);
    const played = record?.completed === true && Number.isFinite(record.bestStrokes);
    return {
      level,
      index,
      played,
      strokes: played ? record!.bestStrokes : null,
      stars: record?.bestStars ?? 0,
      medal: record?.bestMedal ?? 'none',
    };
  });

export interface ScorecardTotals {
  holesPlayed: number;
  holesTotal: number;
  strokes: number;
  /** Par for the holes actually completed, so the total is a fair comparison. */
  par: number;
  stars: number;
  starsTotal: number;
}

export const scorecardTotals = (rows: ScorecardRow[]): ScorecardTotals => {
  let holesPlayed = 0;
  let strokes = 0;
  let par = 0;
  let stars = 0;
  for (const row of rows) {
    stars += row.stars;
    if (!row.played || row.strokes === null) continue;
    holesPlayed++;
    strokes += row.strokes;
    par += row.level.par;
  }
  return {
    holesPlayed,
    holesTotal: rows.length,
    strokes,
    par,
    stars,
    starsTotal: rows.length * 3,
  };
};

export const scorecardScreen = (
  rows: ScorecardRow[],
  chapters: Chapter[],
  earnedFeats: readonly FeatId[],
  onBack: () => void,
): HTMLElement => {
  const totals = scorecardTotals(rows);
  const body = el('div', { class: 'card' });

  for (const chapter of chapters) {
    const chapterRows = rows.filter((row) => row.level.chapter === chapter.index);
    if (chapterRows.length === 0) continue;

    const table = el('table', { class: 'card__table' }, [
      el('caption', { class: 'card__caption', text: `${chapter.index + 1}. ${chapter.name}` }),
      el('thead', {}, [
        el('tr', {}, [
          el('th', { attrs: { scope: 'col' }, text: 'Hole' }),
          el('th', { attrs: { scope: 'col' }, text: 'Par' }),
          el('th', { attrs: { scope: 'col' }, text: 'Best' }),
          el('th', { attrs: { scope: 'col' }, text: 'To par' }),
          el('th', { attrs: { scope: 'col' }, text: 'Stars' }),
        ]),
      ]),
      el(
        'tbody',
        {},
        chapterRows.map((row) =>
          el('tr', { class: row.played ? '' : 'card__row--unplayed' }, [
            el('th', { attrs: { scope: 'row' } }, [
              el('span', { class: 'card__num', text: `${row.index + 1}` }),
              row.level.name,
            ]),
            el('td', { text: String(row.level.par) }),
            el('td', { text: row.strokes === null ? '—' : String(row.strokes) }),
            el('td', {
              class:
                row.strokes === null
                  ? ''
                  : row.strokes < row.level.par
                    ? 'is-under'
                    : row.strokes > row.level.par
                      ? 'is-over'
                      : '',
              text: row.strokes === null ? '—' : formatToPar(row.strokes, row.level.par),
            }),
            el('td', {}, [starRow(row.stars)]),
          ]),
        ),
      ),
    ]);
    body.append(table);
  }

  const diff = totals.strokes - totals.par;
  return el('div', { class: 'screen' }, [
    el('div', { class: 'screen__head' }, [
      el('h2', { text: 'Scorecard' }),
      button('Back', onBack, { class: 'btn--ghost' }),
    ]),
    el('div', { class: 'card__totals' }, [
      statBlock('Holes', `${totals.holesPlayed}/${totals.holesTotal}`),
      statBlock('Strokes', String(totals.strokes)),
      statBlock(
        'To par',
        totals.holesPlayed === 0 ? '—' : diff === 0 ? 'E' : diff > 0 ? `+${diff}` : `${diff}`,
      ),
      statBlock('Stars', `${totals.stars}/${totals.starsTotal}`),
    ]),
    body,
    el('h3', { text: `Feats — ${earnedFeats.length} of ${Object.keys(FEATS).length}` }),
    el(
      'div',
      { class: 'feats' },
      Object.values(FEATS).map((feat) => {
        const earned = earnedFeats.includes(feat.id);
        return el('div', { class: `feat ${earned ? 'feat--earned' : ''}`.trim() }, [
          el('span', { class: 'feat__icon', text: earned ? '✦' : '·' }),
          el('div', {}, [
            el('span', { class: 'feat__label', text: feat.label }),
            el('small', { class: 'feat__desc', text: feat.description }),
          ]),
        ]);
      }),
    ),
  ]);
};

/* --------------------------------------------------------------- loading */

/** Shown while a hole is being generated and verified in the background. */
export const loadingScreen = (message: string, detail: string): HTMLElement =>
  el('div', { class: 'loading-panel', attrs: { role: 'status', 'aria-live': 'polite' } }, [
    el('div', { class: 'loading-panel__orbit' }, [el('i'), el('i')]),
    el('h2', { text: message }),
    el('p', { class: 'loading-panel__detail', text: detail }),
  ]);

/* ------------------------------------------------------------------ help */

export const helpScreen = (onBack: () => void): HTMLElement =>
  el('div', { class: 'screen' }, [
    el('div', { class: 'screen__head' }, [
      el('h2', { text: 'How to play' }),
      button('Back', onBack, { class: 'btn--ghost' }),
    ]),
    el('div', { class: 'help' }, [
      helpItem('🎯', 'Aim', 'Drag back from the ball and release, like a slingshot. The further you pull, the harder you hit.'),
      helpItem('🪐', 'Gravity', 'Planets bend your shot. The faint field lines show which way space pulls at every point.'),
      helpItem('⭐', 'Stars', 'Three stars are hidden on every hole. They only count if the ball survives the shot.'),
      helpItem('⛳', 'The hole', 'Arrive slowly. Come in too fast and the ball rims out.'),
      helpItem('🎯', 'Moving cups', 'Some cups travel along a dashed track. What matters is your speed relative to the cup, so parking on the rails and waiting will not do it.'),
      helpItem('☠️', 'Hazards', 'Suns, black holes and spikes destroy the ball. That costs a penalty stroke. Anything lethal wears a red serrated corona.'),
      helpItem('↩️', 'Undo', 'Take a shot back any time with Z. Restarting was already free, so undo costs nothing.'),
      helpItem('💡', 'Stuck?', 'The 💡 button finds a line from where the ball is and sets your aim to match. The hole still counts, but a hinted run earns no style feats and is not kept as your ghost.'),
    ]),

    // From chapter 6 on, holes have state. The colour language that carries it
    // has to be written down somewhere the player can find it.
    el('h3', { text: 'Machinery' }),
    el('div', { class: 'help' }, [
      // Pads and crystal share the violet family on purpose: one is a ring you
      // pass through and the other a solid you hit, so they can never be
      // mistaken for each other, and a sixth hue would cost more than it buys.
      helpSwatch('#c792ff', 'Violet is machinery', 'A dashed ring on the ground is a switch pad — roll through it to open a gate or drop a bridge. A bridge that has not appeared yet is drawn as an outline.'),
      helpItem('⏱️', 'Timed pads', 'A pad with a ring around it springs back when the ring runs out, so whatever it opened is only open while the ball is still moving. Some vaults need every pad thrown at once.'),
      helpSwatch('#b58cff', 'Crystal blocks', 'A solid violet block shatters after a set number of hits — the cracks show what is left. The hit that breaks it lets you punch straight through.'),
      helpItem('🔒', 'Sealed cups', 'A cup drawn in amber with bars across it will not take the ball until every star is collected. The pips above it count what is still owed.'),
      helpSwatch('#8fdcf5', 'Membranes', 'Chevrons show the one direction you may cross. There is no coming back through one.'),
      helpSwatch('#a6f259', 'Boost rings', 'A lime ring fires the ball out along its arrows at its own fixed speed, whatever speed you arrived at. Get into it however you like — the exit is always the same.'),
    ]),

    el('h3', { text: 'Reading the field' }),
    el('div', { class: 'help' }, [
      helpSwatch('#ff7ae0', 'Pink', 'Gravity is reversed inside. The chevrons rise, because everything else does too.'),
      helpSwatch('#ffa05a', 'Amber', 'Gravity is stronger inside. Skirt it, or commit to it.'),
      helpSwatch('#78ffd2', 'Green', 'Gravity is weaker inside — near enough to a dead calm.'),
    ]),

    el('h3', { text: 'Controls' }),
    el('div', { class: 'help' }, [
      helpItem('⌨️', 'Keyboard', 'Arrows aim and set power, Space shoots, Z takes a shot back, H asks for a line, R restarts, G toggles the field, C recentres, Esc pauses.'),
      helpItem('🖱️', 'Camera', 'Scroll or pinch to zoom, right-drag or two-finger drag to look around.'),
    ]),
  ]);

const helpItem = (glyph: string, title: string, body: string): HTMLElement =>
  el('div', { class: 'help__item' }, [
    el('span', { class: 'help__glyph', text: glyph }),
    el('div', {}, [el('h4', { text: title }), el('p', { text: body })]),
  ]);

/**
 * A legend row whose glyph is the actual colour being described. The colour is
 * the thing being taught, so an emoji stand-in would defeat the purpose.
 */
const helpSwatch = (color: string, title: string, body: string): HTMLElement =>
  el('div', { class: 'help__item' }, [
    el('span', { class: 'help__glyph' }, [
      el('i', { class: 'help__swatch', style: { background: color } }),
    ]),
    el('div', {}, [el('h4', { text: title }), el('p', { text: body })]),
  ]);
