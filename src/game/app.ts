import { AudioEngine } from '../audio/audio';
import { clamp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import { HIGH_CONTRAST_PALETTE, SPACE_PALETTE } from '../render/palette';
import { Renderer, type AimState, type RenderOptions, type Scene } from '../render/renderer';
import { el } from '../ui/dom';
import { Hud } from '../ui/hud';
import { InputController } from '../ui/input';
import {
  Overlay,
  helpScreen,
  isLevelUnlocked,
  buildScorecard,
  levelSelectScreen,
  pauseScreen,
  scorecardScreen,
  resultsScreen,
  settingsScreen,
  titleScreen,
} from '../ui/screens';
import { BALL_RADIUS, type LevelDef } from './level';
import { ALL_LEVELS, CHAPTERS, nextLevel } from './levels';
import { ProgressStore, type Settings } from './progress';
import { PlaySession, type SessionEvent } from './session';

type AppScreen =
  | 'title'
  | 'levels'
  | 'play'
  | 'paused'
  | 'results'
  | 'settings'
  | 'help'
  | 'scorecard';

const TRAIL_LENGTH = 34;
/** Trail samples are spaced by time, not frames, so it looks the same at any FPS. */
const TRAIL_INTERVAL = 1 / 60;

export class GameApp {
  private readonly renderer: Renderer;
  private readonly audio = new AudioEngine();
  private readonly progress: ProgressStore;
  private readonly overlay = new Overlay();
  private readonly hud: Hud;
  private readonly input: InputController;

  private session: PlaySession | null = null;
  private screen: AppScreen = 'title';
  /** Where Settings/Help should return to. */
  private returnScreen: AppScreen = 'title';

  private trail: Vec2[] = [];
  private trailClock = 0;
  private aimState: AimState | null = null;
  private previewCache: { key: string; points: Vec2[]; outcome: AimState['previewOutcome'] } | null =
    null;

  private cameraOffset: Vec2 = V.ZERO;
  private manualZoom = 1;
  private baseZoom = 1;

  private lastFrameTime = 0;
  private rafHandle = 0;
  private running = false;
  private hintShownFor = new Set<string>();
  private disposed = false;

  constructor(
    private readonly container: HTMLElement,
    progress = new ProgressStore(),
  ) {
    this.progress = progress;

    const canvas = el('canvas', {
      class: 'game-canvas',
      attrs: { 'aria-label': 'Gravity Golf play area', tabindex: '0' },
    });
    this.renderer = new Renderer(canvas);

    this.hud = new Hud({
      onPause: () => this.pause(),
      onRetry: () => this.retry(),
      onToggleField: () =>
        this.applySettings({ showGravityField: !this.progress.settings.showGravityField }),
    });

    this.container.append(canvas, this.hud.root, this.overlay.root);

    this.input = new InputController(
      canvas,
      {
        toWorld: (p) => this.renderer.camera.screenToWorld(p),
        ballPosition: () => this.session?.ball.position ?? null,
        canShoot: () => this.screen === 'play' && this.session?.canShoot === true,
        onShoot: (direction, power) => this.shoot(direction, power),
        onPan: (delta) => this.pan(delta),
        onZoom: (factor) => this.zoom(factor),
        onCancel: () => {
          this.aimState = null;
        },
        onFirstInteraction: () => void this.startAudio(),
      },
      { aimMode: this.progress.settings.aimMode },
    );

    this.applyAudioSettings();
    this.bindGlobalKeys();
    this.handleResize();
    window.addEventListener('resize', this.handleResize);
    document.addEventListener('visibilitychange', this.handleVisibility);

    this.showTitle();
  }

  /* ---------------------------------------------------------- lifecycle */

  start(): void {
    if (this.running || this.disposed) return;
    this.running = true;
    this.lastFrameTime = performance.now();
    this.rafHandle = requestAnimationFrame(this.frame);
  }

  stop(): void {
    this.running = false;
    if (this.rafHandle) cancelAnimationFrame(this.rafHandle);
    this.rafHandle = 0;
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.stop();
    this.input.dispose();
    this.audio.dispose();
    window.removeEventListener('resize', this.handleResize);
    document.removeEventListener('visibilitychange', this.handleVisibility);
  }

  private handleResize = (): void => {
    const rect = this.container.getBoundingClientRect();
    this.renderer.resize(rect.width || window.innerWidth, rect.height || window.innerHeight);
    if (this.session) this.refitCamera();
  };

  private handleVisibility = (): void => {
    if (document.hidden && this.screen === 'play') this.pause();
  };

  private frame = (now: number): void => {
    if (!this.running) return;
    const dt = clamp((now - this.lastFrameTime) / 1000, 0, 0.1);
    this.lastFrameTime = now;
    this.update(dt);
    this.render(dt);
    this.rafHandle = requestAnimationFrame(this.frame);
  };

  /* ------------------------------------------------------------- update */

  private update(dt: number): void {
    this.input.update(dt);
    this.hud.update(dt);

    const settings = this.progress.settings;
    const session = this.session;

    if (session && this.screen === 'play') {
      const events = session.update(dt);
      this.handleSessionEvents(events);
      this.updateAim();
      this.updateTrail(dt, session);
      this.updateCamera(dt, session);
      this.hud.sync(session);
    }

    if (!settings.reducedMotion) this.renderer.starfield.update(dt);
    this.renderer.particles.update(dt);
    this.renderer.camera.update(dt);
  }

  private updateTrail(dt: number, session: PlaySession): void {
    if (session.state !== 'flying') {
      if (this.trail.length > 0) this.trail.shift();
      return;
    }
    this.trailClock += dt;
    while (this.trailClock >= TRAIL_INTERVAL) {
      this.trailClock -= TRAIL_INTERVAL;
      this.trail.push(session.ball.position);
      if (this.trail.length > TRAIL_LENGTH) this.trail.shift();
    }
  }

  private updateCamera(dt: number, session: PlaySession): void {
    void dt;
    const camera = this.renderer.camera;
    const settings = this.progress.settings;

    camera.targetZoom = this.baseZoom * this.manualZoom;
    const focus = session.state === 'flying' ? session.ball.position : session.ball.position;
    camera.target = V.add(focus, this.cameraOffset);

    // While the ball is flying, ease out so more of the field is visible.
    if (session.state === 'flying') {
      const speed = V.length(session.ball.velocity);
      const zoomOut = clamp(1 - speed / 2600, 0.82, 1);
      camera.targetZoom *= zoomOut;
    }
    if (!settings.screenShake) camera.clearShake();
  }

  private updateAim(): void {
    const session = this.session;
    const input = this.input.aim;
    if (!session || !input || !session.canShoot) {
      this.aimState = null;
      return;
    }

    const settings = this.progress.settings;
    const origin = session.ball.position;
    let preview: Vec2[] = [];
    let outcome: AimState['previewOutcome'] = 'truncated';

    if (settings.aimAssist > 0 && input.power > 0.02) {
      // Recomputing a full simulation every frame is wasteful while the aim
      // barely moves, so quantise the inputs and cache on that key.
      const key = `${Math.round(V.angleOf(input.direction) * 200)}:${Math.round(
        input.power * 200,
      )}:${Math.round(session.world.time * 20)}:${Math.round(origin.x)}:${Math.round(origin.y)}`;
      if (this.previewCache?.key === key) {
        preview = this.previewCache.points;
        outcome = this.previewCache.outcome;
      } else {
        const trajectory = session.predict(input.direction, input.power, {
          maxTime: settings.aimAssist,
        });
        preview = trajectory.points;
        outcome = trajectory.outcome;
        this.previewCache = { key, points: preview, outcome };
      }
    }

    this.aimState = {
      origin,
      direction: input.direction,
      power: input.power,
      preview,
      previewOutcome: outcome,
    };
  }

  private handleSessionEvents(events: SessionEvent[]): void {
    const session = this.session;
    if (!session) return;
    const palette = this.palette();
    const particles = this.renderer.particles;
    const camera = this.renderer.camera;
    const shakeAllowed = this.progress.settings.screenShake;

    for (const event of events) {
      switch (event.type) {
        case 'shot':
          this.audio.play('putt', event.power);
          this.progress.countShot();
          particles.emit(session.ball.position, {
            count: 10,
            color: palette.accent,
            speed: 70 * event.power + 20,
            life: 0.35,
            size: 2,
            direction: V.neg(event.direction),
            spread: 0.9,
            shape: 'spark',
          });
          break;

        case 'bounce':
          this.audio.playImpact(event.material, event.speed);
          particles.impact(event.point, event.normal, event.speed, palette.trail);
          if (shakeAllowed && event.speed > 320) camera.shake(event.speed / 260, 0.25);
          break;

        case 'star':
          this.audio.play('star');
          particles.starPop(event.position, palette.collectible);
          this.hud.showToast(`Star ${event.total} of ${session.totalStars}`, 1.4, 'good');
          break;

        case 'star-lost':
          break;

        case 'portal':
          this.audio.play('portal');
          particles.burst(event.from, palette.accentAlt, 1.4);
          particles.burst(event.to, palette.accentAlt, 1.4);
          break;

        case 'lipout':
          this.audio.play('lipout');
          this.hud.showToast('Too fast!', 1.2, 'bad');
          break;

        case 'settled':
          this.trail = [];
          break;

        case 'died': {
          this.audio.play('death');
          this.progress.countDeath();
          particles.explosion(event.position, palette.danger);
          if (shakeAllowed) camera.shake(9, 0.45);
          this.trail = [];
          const reason =
            event.cause === 'out-of-bounds' ? 'Lost in deep space' : 'Ball destroyed';
          this.hud.showToast(`${reason} · +1 stroke`, 2, 'bad');
          break;
        }

        case 'timeout':
          this.hud.showToast('Shot abandoned — the ball never settled', 2.2, 'bad');
          this.trail = [];
          break;

        case 'respawn':
          particles.burst(event.position, palette.accent, 1.2);
          break;

        case 'sunk':
          this.audio.play('sink');
          particles.sinkCelebration(session.world.hole.position, palette.holeRim);
          if (shakeAllowed) camera.shake(4, 0.3);
          this.finishHole();
          break;
      }
    }
  }

  /* --------------------------------------------------------------- flow */

  private shoot(direction: Vec2, power: number): void {
    const session = this.session;
    if (!session) return;
    this.aimState = null;
    this.previewCache = null;
    this.trail = [];
    this.handleSessionEvents(session.shoot(direction, power));
  }

  playLevel(level: LevelDef): void {
    this.session = new PlaySession(level);
    this.progress.countAttempt(level.id);
    this.screen = 'play';
    this.overlay.hide();
    this.hud.setVisible(true);
    this.hud.setLevel(level, !this.hintShownFor.has(level.id));
    this.hintShownFor.add(level.id);
    this.hud.sync(this.session);

    this.trail = [];
    this.aimState = null;
    this.previewCache = null;
    this.cameraOffset = V.ZERO;
    this.manualZoom = 1;
    this.renderer.particles.clear();
    this.renderer.resetDecor();
    this.refitCamera();
    this.renderer.camera.snapTo(level.tee, this.baseZoom);
    this.renderer.canvas.focus();
  }

  private refitCamera(): void {
    const session = this.session;
    if (!session) return;
    // Fit the level, but never zoom so far out that the ball becomes a speck.
    this.baseZoom = clamp(this.renderer.camera.fit(session.world.bounds, 60, 1.4), 0.35, 1.4);
    this.renderer.camera.limits = session.world.bounds;
  }

  private finishHole(): void {
    const session = this.session;
    if (!session?.result) return;
    const result = session.result;
    const previous = this.progress.recordOf(result.levelId);
    const previousBest =
      previous && previous.completed && Number.isFinite(previous.bestStrokes)
        ? previous.bestStrokes
        : null;
    const isNewBest = previousBest === null || result.strokes < previousBest;

    this.progress.submit(result);
    const upcoming = nextLevel(result.levelId);

    // A beat of celebration before the panel covers the screen.
    window.setTimeout(() => {
      if (this.disposed || this.screen !== 'play') return;
      this.screen = 'results';
      this.hud.setVisible(false);
      this.overlay.show(
        resultsScreen(result, session.level.name, {
          hasNext: upcoming !== undefined,
          isNewBest,
          previousBest,
          onNext: () => upcoming && this.playLevel(upcoming),
          onRetry: () => this.playLevel(session.level),
          onLevels: () => this.showLevels(),
        }),
        {},
      );
    }, 900);
  }

  private retry(): void {
    const session = this.session;
    if (!session) return;
    this.audio.play('back');
    this.playLevel(session.level);
  }

  private pause(): void {
    const session = this.session;
    if (!session || this.screen !== 'play') return;
    this.screen = 'paused';
    this.audio.play('click');
    this.overlay.show(
      pauseScreen(session.level, {
        onResume: () => this.resume(),
        onRetry: () => this.retry(),
        onLevels: () => this.showLevels(),
        onSettings: () => this.showSettings('paused'),
        onTitle: () => this.showTitle(),
      }),
      { onEscape: () => this.resume() },
    );
  }

  private resume(): void {
    if (this.screen !== 'paused') return;
    this.screen = 'play';
    this.overlay.hide();
    this.hud.setVisible(true);
    this.renderer.canvas.focus();
  }

  showTitle(): void {
    this.screen = 'title';
    this.session = null;
    this.hud.setVisible(false);
    this.renderer.camera.limits = null;
    this.overlay.show(
      titleScreen(
        {
          onPlay: () => this.playNextUnfinished(),
          onLevels: () => this.showLevels(),
          onSettings: () => this.showSettings('title'),
          onHelp: () => this.showHelp('title'),
          onScorecard: () => this.showScorecard('title'),
        },
        this.progress,
        ALL_LEVELS.length,
      ),
      {},
    );
  }

  private playNextUnfinished(): void {
    const next =
      ALL_LEVELS.find(
        (level, index) =>
          !this.progress.isCompleted(level.id) &&
          isLevelUnlocked(index, ALL_LEVELS, this.progress),
      ) ?? ALL_LEVELS[0]!;
    this.playLevel(next);
  }

  showLevels(): void {
    this.screen = 'levels';
    this.session = null;
    this.hud.setVisible(false);
    this.overlay.show(
      levelSelectScreen(CHAPTERS, ALL_LEVELS, this.progress, {
        onPick: (level) => this.playLevel(level),
        onBack: () => this.showTitle(),
      }),
      { onEscape: () => this.showTitle(), wide: true },
    );
  }

  private showSettings(from: AppScreen): void {
    this.returnScreen = from;
    this.screen = 'settings';
    this.overlay.show(
      settingsScreen(
        this.progress.settings,
        (patch) => this.applySettings(patch),
        () => this.leaveSubScreen(),
        () => this.confirmResetProgress(),
      ),
      { onEscape: () => this.leaveSubScreen(), wide: true },
    );
  }

  private showScorecard(from: AppScreen): void {
    this.returnScreen = from;
    this.screen = 'scorecard';
    this.overlay.show(
      scorecardScreen(buildScorecard(ALL_LEVELS, this.progress), CHAPTERS, () =>
        this.leaveSubScreen(),
      ),
      { onEscape: () => this.leaveSubScreen(), wide: true },
    );
  }

  private showHelp(from: AppScreen): void {
    this.returnScreen = from;
    this.screen = 'help';
    this.overlay.show(helpScreen(() => this.leaveSubScreen()), {
      onEscape: () => this.leaveSubScreen(),
      wide: true,
    });
  }

  private leaveSubScreen(): void {
    this.audio.play('back');
    if (this.returnScreen === 'paused' && this.session) {
      this.screen = 'paused';
      this.pauseFromSubScreen();
    } else if (this.returnScreen === 'levels') {
      this.showLevels();
    } else {
      this.showTitle();
    }
  }

  private pauseFromSubScreen(): void {
    const session = this.session;
    if (!session) {
      this.showTitle();
      return;
    }
    this.overlay.show(
      pauseScreen(session.level, {
        onResume: () => this.resume(),
        onRetry: () => this.retry(),
        onLevels: () => this.showLevels(),
        onSettings: () => this.showSettings('paused'),
        onTitle: () => this.showTitle(),
      }),
      { onEscape: () => this.resume() },
    );
  }

  private confirmResetProgress(): void {
    // eslint-disable-next-line no-alert
    const confirmed = window.confirm(
      'Reset all progress? Every best score, star and unlock will be erased.',
    );
    if (!confirmed) return;
    this.progress.reset();
    this.hintShownFor.clear();
    this.applyAudioSettings();
    this.showTitle();
  }

  private applySettings(patch: Partial<Settings>): void {
    const settings = this.progress.updateSettings(patch);
    this.input.config.aimMode = settings.aimMode;
    this.applyAudioSettings();
    this.previewCache = null;
  }

  private applyAudioSettings(): void {
    const settings = this.progress.settings;
    this.audio.setSfxVolume(settings.sfxVolume);
    this.audio.setMusicVolume(settings.musicVolume);
  }

  private async startAudio(): Promise<void> {
    await this.audio.unlock();
    this.applyAudioSettings();
    if (this.progress.settings.musicVolume > 0) this.audio.startMusic();
  }

  /* ------------------------------------------------------------- camera */

  private pan(deltaScreen: Vec2): void {
    const zoom = this.renderer.camera.zoom || 1;
    this.cameraOffset = V.sub(this.cameraOffset, V.mul(deltaScreen, 1 / zoom));
    // Keep the manual offset modest so the ball never leaves the screen.
    this.cameraOffset = V.clampLength(this.cameraOffset, 420);
  }

  private zoom(factor: number): void {
    this.manualZoom = clamp(this.manualZoom * factor, 0.6, 3);
  }

  private bindGlobalKeys(): void {
    window.addEventListener('keydown', (event) => {
      if (event.defaultPrevented) return;
      const target = event.target as HTMLElement | null;
      if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;

      switch (event.code) {
        case 'Escape':
          if (this.screen === 'play') {
            event.preventDefault();
            this.pause();
          } else if (this.screen === 'paused') {
            event.preventDefault();
            this.resume();
          }
          break;
        case 'KeyR':
          if (this.screen === 'play') {
            event.preventDefault();
            this.retry();
          }
          break;
        case 'KeyG':
          if (this.screen === 'play' || this.screen === 'paused') {
            event.preventDefault();
            this.applySettings({ showGravityField: !this.progress.settings.showGravityField });
          }
          break;
        case 'KeyC':
          if (this.screen === 'play') {
            event.preventDefault();
            this.cameraOffset = V.ZERO;
            this.manualZoom = 1;
          }
          break;
      }
    });
  }

  /* ------------------------------------------------------------- render */

  private palette() {
    return this.progress.settings.highContrast ? HIGH_CONTRAST_PALETTE : SPACE_PALETTE;
  }

  private render(dt: number): void {
    void dt;
    const session = this.session;
    const options: RenderOptions = {
      palette: this.palette(),
      showGravityField: this.progress.settings.showGravityField,
      reducedMotion: this.progress.settings.reducedMotion,
      aimAssist: this.progress.settings.aimAssist,
    };

    if (!session) {
      // Menus still get a live starfield behind them.
      this.renderer.draw(
        {
          world: EMPTY_SCENE_WORLD,
          ballPosition: V.ZERO,
          ballRadius: BALL_RADIUS,
          ballVisible: false,
          trail: [],
          aim: null,
          time: performance.now() / 1000,
          levelId: '',
        },
        { ...options, showGravityField: false },
      );
      return;
    }

    const scene: Scene = {
      world: session.world,
      ballPosition: session.ball.position,
      ballRadius: session.ball.radius,
      ballVisible: session.state !== 'respawning',
      trail: this.trail,
      aim: this.screen === 'play' ? this.aimState : null,
      time: session.totalTime,
      levelId: session.level.id,
    };
    this.renderer.draw(scene, options);
  }
}

/** A minimal world so menu backgrounds can reuse the same draw path. */
const EMPTY_SCENE_WORLD = {
  bodies: [],
  zones: [],
  portals: [],
  collectibles: [],
  hole: { position: { x: 0, y: -100000 }, radius: 1, captureSpeed: 1 },
  bounds: { minX: -1, minY: -1, maxX: 1, maxY: 1 },
  boundsMode: 'open' as const,
  uniformGravity: { x: 0, y: 0 },
  config: {
    timeStep: 1 / 240,
    ambientDrag: 0,
    restSpeed: 1,
    restDelay: 1,
    maxSpeed: 1,
    contactSlop: 0,
    gravitySoftening: 1,
  },
  time: 0,
};
