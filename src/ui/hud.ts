import { clamp } from '../core/math';
import type { LevelDef } from '../game/level';
import type { PlaySession } from '../game/session';
import { icon, type IconName } from './icons';
import { button, clear, el, formatToPar } from './dom';

export interface HudCallbacks {
  onPause(): void;
  onRetry(): void;
  onUndo(): void;
  onToggleField(): void;
  /** Ask the solver for a playable line from where the ball is. */
  onHint(): void;
}

/**
 * The in-game overlay: hole name, stroke count, stars, the shot clock and the
 * transient toast line. DOM rather than canvas, so it scales with the browser's
 * font settings and is readable by assistive tech.
 */
export class Hud {
  readonly root: HTMLElement;

  private levelName: HTMLElement;
  private parLabel: HTMLElement;
  private strokeValue: HTMLElement;
  private toParValue: HTMLElement;
  private starHolder: HTMLElement;
  private clockFill: HTMLElement;
  private clockWrap: HTMLElement;
  private toast: HTMLElement;
  private hint: HTMLElement;
  private intro: HTMLElement;
  private introKicker: HTMLElement;
  private introName: HTMLElement;
  private introPar: HTMLElement;
  private undoButton: HTMLButtonElement;
  /** Not the level tip above — this asks the solver for a line. */
  private hintButton: HTMLButtonElement;
  private hintBusy = false;

  private toastTimer = 0;
  private hintTimer = 0;

  constructor(callbacks: HudCallbacks) {
    this.levelName = el('span', { class: 'hud__level' });
    this.parLabel = el('span', { class: 'hud__par' });
    this.strokeValue = el('span', { class: 'hud__stat-value', text: '0' });
    this.toParValue = el('span', { class: 'hud__stat-value hud__stat-value--par', text: 'E' });
    this.starHolder = el('span', { class: 'hud__stars' });
    this.clockFill = el('i', { class: 'hud__clock-fill' });
    this.clockWrap = el(
      'div',
      {
        class: 'hud__clock',
        attrs: { role: 'progressbar', 'aria-label': 'Shot time remaining' },
      },
      [this.clockFill],
    );
    this.toast = el('div', { class: 'hud__toast', attrs: { role: 'status', 'aria-live': 'polite' } });
    this.hint = el('div', { class: 'hud__hint' });
    this.introKicker = el('span', { class: 'hud__intro-kicker' });
    this.introName = el('span', { class: 'hud__intro-name' });
    this.introPar = el('span', { class: 'hud__intro-par' });
    // A live region, so the hole is announced as well as shown.
    this.intro = el('div', { class: 'hud__intro', attrs: { role: 'status', 'aria-live': 'polite' } }, [
      this.introKicker,
      this.introName,
      this.introPar,
    ]);

    this.undoButton = withIcon('undo', button('', callbacks.onUndo, {
      class: 'btn--icon',
      title: 'Take back last shot (Z)',
      attrs: { 'aria-label': 'Take back last shot' },
    }));

    this.hintButton = withIcon('hint', button('', callbacks.onHint, {
      class: 'btn--icon',
      title: 'Show a line (H)',
      attrs: { 'aria-label': 'Show a suggested line' },
    }));

    this.root = el('div', { class: 'hud' }, [
      el('div', { class: 'hud__top' }, [
        el('div', { class: 'hud__title' }, [this.levelName, this.parLabel]),
        el('div', { class: 'hud__stats' }, [
          el('span', { class: 'hud__stat' }, [
            el('span', { class: 'hud__stat-label', text: 'Strokes' }),
            this.strokeValue,
          ]),
          el('span', { class: 'hud__stat' }, [
            el('span', { class: 'hud__stat-label', text: 'To par' }),
            this.toParValue,
          ]),
          this.starHolder,
        ]),
        el('div', { class: 'hud__actions' }, [
          this.hintButton,
          this.undoButton,
          withIcon('restart', button('', callbacks.onRetry, {
            class: 'btn--icon',
            title: 'Restart hole (R)',
            attrs: { 'aria-label': 'Restart hole' },
          })),
          withIcon('field', button('', callbacks.onToggleField, {
            class: 'btn--icon',
            title: 'Toggle gravity field (G)',
            attrs: { 'aria-label': 'Toggle gravity field overlay' },
          })),
          withIcon('pause', button('', callbacks.onPause, {
            class: 'btn--icon',
            title: 'Pause (Esc)',
            attrs: { 'aria-label': 'Pause' },
          })),
        ]),
      ]),
      this.clockWrap,
      this.intro,
      el('div', { class: 'hud__bottom' }, [this.hint, this.toast]),
    ]);
  }

  /** Disables the hint button and marks it working while the solver runs. */
  setHintBusy(busy: boolean): void {
    this.hintBusy = busy;
    this.hintButton.disabled = busy;
    this.hintButton.classList.toggle('btn--busy', busy);
  }

  setLevel(level: LevelDef, showHint: boolean, kicker = ''): void {
    this.levelName.textContent = level.name;
    // The title card: the hole announced, then out of the way.
    this.introKicker.textContent = kicker;
    this.introName.textContent = level.name;
    this.introPar.textContent = `Par ${level.par}`;
    this.intro.classList.remove('hud__intro--show');
    void this.intro.offsetWidth;
    this.intro.classList.add('hud__intro--show');
    this.parLabel.textContent = `Par ${level.par}`;
    clear(this.hint);
    if (showHint && level.hint) {
      this.hint.textContent = level.hint;
      this.hint.classList.add('hud__hint--visible');
      this.hintTimer = 6;
    } else {
      this.hint.classList.remove('hud__hint--visible');
      this.hintTimer = 0;
    }
  }

  /** Refreshes the readouts from the live session. */
  sync(session: PlaySession): void {
    const strokes = String(session.strokes);
    if (this.strokeValue.textContent !== strokes) {
      // A bump on change, so a new stroke registers without reading it.
      this.strokeValue.textContent = strokes;
      bump(this.strokeValue);
    }
    const toPar = formatToPar(session.strokes, session.level.par);
    this.toParValue.textContent = toPar;
    const played = session.strokes > 0;
    this.toParValue.classList.toggle('is-over', played && session.strokes > session.level.par);
    this.toParValue.classList.toggle('is-under', played && session.strokes < session.level.par);

    const total = session.totalStars;
    const earned = session.starCount;
    if (this.starHolder.childElementCount !== total) {
      clear(this.starHolder);
      for (let i = 0; i < total; i++) {
        this.starHolder.append(el('span', { class: 'star', text: '★' }));
      }
    }
    this.starHolder.setAttribute('aria-label', `${earned} of ${total} stars collected`);
    Array.from(this.starHolder.children).forEach((node, i) => {
      const on = i < earned;
      if (on && !node.classList.contains('star--on')) bump(node as HTMLElement);
      node.classList.toggle('star--on', on);
    });

    this.undoButton.disabled = !session.canUndo;
    this.hintButton.disabled = this.hintBusy || !session.canShoot;
    this.hintButton.classList.toggle('btn--busy', this.hintBusy);

    // The shot clock only appears once a shot is running long.
    const clock = session.shotClock;
    const visible = clock > 0.55;
    this.clockWrap.classList.toggle('hud__clock--visible', visible);
    if (visible) {
      const remaining = clamp(1 - (clock - 0.55) / 0.45, 0, 1);
      this.clockFill.style.transform = `scaleX(${remaining})`;
      this.clockWrap.setAttribute('aria-valuenow', String(Math.round(remaining * 100)));
      this.clockWrap.classList.toggle('is-critical', remaining < 0.3);
    }
  }

  /**
   * A star flying from where it was picked up to its slot in the readout, so
   * the eye follows it to the count. `from` is in viewport pixels.
   */
  flyStar(from: { x: number; y: number }, slot: number): void {
    const target = this.starHolder.children[slot] as HTMLElement | undefined;
    if (!target || typeof document === 'undefined') return;
    const to = target.getBoundingClientRect();
    const host = this.root.getBoundingClientRect();
    const star = el('span', { class: 'hud__flying-star', text: '★', attrs: { 'aria-hidden': 'true' } });
    star.style.left = `${from.x}px`;
    star.style.top = `${from.y}px`;
    this.root.append(star);
    const dx = to.left + to.width / 2 - host.left - from.x;
    const dy = to.top + to.height / 2 - host.top - from.y;
    const animation = star.animate?.(
      [
        { transform: 'translate(-50%, -50%) scale(1.8)', opacity: 1 },
        { transform: `translate(calc(-50% + ${dx * 0.35}px), calc(-50% + ${dy * 0.2 - 40}px)) scale(1.4)`, opacity: 1, offset: 0.35 },
        { transform: `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px)) scale(0.8)`, opacity: 0.9 },
      ],
      { duration: 650, easing: 'cubic-bezier(0.5, 0, 0.6, 1)' },
    );
    if (!animation) {
      star.remove();
      return;
    }
    animation.onfinish = () => {
      star.remove();
      bump(target);
    };
  }

  /** Shows a short message near the bottom of the screen. */
  showToast(message: string, seconds = 1.8, tone: 'neutral' | 'good' | 'bad' = 'neutral'): void {
    this.toast.textContent = message;
    this.toast.className = `hud__toast hud__toast--visible hud__toast--${tone}`;
    this.toastTimer = seconds;
  }

  update(dt: number): void {
    if (this.toastTimer > 0) {
      this.toastTimer -= dt;
      if (this.toastTimer <= 0) this.toast.classList.remove('hud__toast--visible');
    }
    if (this.hintTimer > 0) {
      this.hintTimer -= dt;
      if (this.hintTimer <= 0) this.hint.classList.remove('hud__hint--visible');
    }
  }

  setVisible(visible: boolean): void {
    this.root.classList.toggle('hud--hidden', !visible);
    // Faded out is not enough: its buttons would still take clicks and focus.
    this.root.inert = !visible;
  }
}

/** Restarts an element's bump animation. */
const bump = (node: HTMLElement): void => {
  node.classList.remove('is-bumped');
  void node.offsetWidth;
  node.classList.add('is-bumped');
};

/** Puts an icon in a button that is labelled for assistive tech elsewhere. */
const withIcon = (name: IconName, node: HTMLButtonElement): HTMLButtonElement => {
  node.append(icon(name));
  return node;
};
