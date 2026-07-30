import { clamp } from '../core/math';
import type { LevelDef } from '../game/level';
import type { PlaySession } from '../game/session';
import { button, clear, el, formatToPar } from './dom';

export interface HudCallbacks {
  onPause(): void;
  onRetry(): void;
  onUndo(): void;
  onToggleField(): void;
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
  private undoButton: HTMLButtonElement;

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

    this.undoButton = button('↶', callbacks.onUndo, {
      class: 'btn--icon',
      title: 'Take back last shot (Z)',
      attrs: { 'aria-label': 'Take back last shot' },
    });

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
          this.undoButton,
          button('↺', callbacks.onRetry, {
            class: 'btn--icon',
            title: 'Restart hole (R)',
            attrs: { 'aria-label': 'Restart hole' },
          }),
          button('⊞', callbacks.onToggleField, {
            class: 'btn--icon',
            title: 'Toggle gravity field (G)',
            attrs: { 'aria-label': 'Toggle gravity field overlay' },
          }),
          button('❚❚', callbacks.onPause, {
            class: 'btn--icon',
            title: 'Pause (Esc)',
            attrs: { 'aria-label': 'Pause' },
          }),
        ]),
      ]),
      this.clockWrap,
      el('div', { class: 'hud__bottom' }, [this.hint, this.toast]),
    ]);
  }

  setLevel(level: LevelDef, showHint: boolean): void {
    this.levelName.textContent = level.name;
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
    this.strokeValue.textContent = String(session.strokes);
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
      node.classList.toggle('star--on', i < earned);
    });

    this.undoButton.disabled = !session.canUndo;

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
  }
}
