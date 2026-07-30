import { AudioEngine } from '../audio/audio';
import { clamp, damp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import { HIGH_CONTRAST_PALETTE, SPACE_PALETTE } from '../render/palette';
import { Renderer, type AimState, type RenderOptions, type Scene } from '../render/renderer';
import { el } from '../ui/dom';
import { Hud } from '../ui/hud';
import { InputController } from '../ui/input';
import {
  Overlay,
  campaignCompleteScreen,
  type CampaignSummary,
  helpScreen,
  isLevelUnlocked,
  buildScorecard,
  loadingScreen,
  levelSelectScreen,
  pauseScreen,
  scorecardScreen,
  scorecardTotals,
  resultsScreen,
  roundCompleteScreen,
  settingsScreen,
  titleScreen,
} from '../ui/screens';
import { holePositionAt } from '../physics/world';
import { BALL_RADIUS, type LevelDef } from './level';
import { ALL_LEVELS, CHAPTERS, nextLevel } from './levels';
import { dailyId, dailySeed } from './generator';
import { LevelGenerator } from './generator-client';
import {
  linkForLevel,
  parseDeepLink,
  shareUrl,
  type DeepLink,
} from './deeplink';
import { GhostRunner } from './ghost';
import {
  buildRound,
  randomRoundSeed,
  roundFinished,
  roundStrokes,
  type RoundState,
} from './round';
import { serializeBoardState } from './hint';
import { ProgressStore, type Settings } from './progress';
import { FEATS, PlaySession, type HoleResult, type SessionEvent } from './session';
import { resolveSkin } from './skins';

type AppScreen =
  | 'title'
  | 'levels'
  | 'play'
  | 'paused'
  | 'results'
  | 'settings'
  | 'help'
  | 'scorecard';

/** Campaign level ids, so generated holes stay out of campaign counters. */
const CAMPAIGN_IDS = ALL_LEVELS.map((level) => level.id);

/** The ball stops being a legible object below roughly this scale. */
const MIN_ZOOM = 0.55;

const TRAIL_LENGTH = 34;
/** Trail samples are spaced by time, not frames, so it looks the same at any FPS. */
const TRAIL_INTERVAL = 1 / 60;

/** How much the clock slows during a tense approach to the cup. */
const SLOWMO_SCALE = 0.32;
/** Distance from the cup, in hole radii, where the slow-motion band begins. */
const SLOWMO_RADII = 4.5;
/** Seconds the camera spends showing the hole before play starts. */
const FLYBY_SECONDS = 1.1;

export class GameApp {
  private readonly renderer: Renderer;
  private readonly audio = new AudioEngine();
  private readonly progress: ProgressStore;
  private readonly overlay = new Overlay();
  private readonly hud: Hud;
  private readonly input: InputController;
  private readonly generator = new LevelGenerator();

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
  private generating = false;
  /** The round in progress, or null when playing a single hole. */
  private round: RoundState | null = null;
  /** The suggested line currently on screen, and the shot it came from. */
  private hintLine: { path: Vec2[]; strokes: number } | null = null;
  private hintPending = false;
  private disposed = false;

  private ghost: GhostRunner | null = null;
  /** Eased 0..1; 1 is full speed. Drives the approach slow-motion. */
  private timeScale = 1;
  /** Counts down at level start while the camera shows the hole. */
  private flybyTimer = 0;

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
      onUndo: () => this.undo(),
      onToggleField: () =>
        this.applySettings({ showGravityField: !this.progress.settings.showGravityField }),
      onHint: () => void this.requestHint(),
    });

    this.container.append(canvas, this.hud.root, this.overlay.root);

    this.input = new InputController(
      canvas,
      {
        canShoot: () => this.screen === 'play' && this.session?.canShoot === true,
        onShoot: (direction, power) => this.shoot(direction, power),
        onPan: (delta) => this.pan(delta),
        onZoom: (factor) => this.zoom(factor),
        onCancel: () => {
          this.aimState = null;
        },
        onFirstInteraction: () => void this.startAudio(),
        onInteract: () => {
          // Any input means the player is ready; do not make them wait.
          this.flybyTimer = 0;
        },
      },
      { aimMode: this.progress.settings.aimMode },
    );

    this.applyAudioSettings();
    this.bindGlobalKeys();
    this.handleResize();
    window.addEventListener('resize', this.handleResize);
    document.addEventListener('visibilitychange', this.handleVisibility);

    // A shared link goes straight to the hole it names; the title screen is what
    // you get when nobody sent you anywhere.
    const link = parseDeepLink(window.location.search);
    if (link) void this.openDeepLink(link);
    else this.showTitle();
  }

  /**
   * Opens a hole from a shared link.
   *
   * Deliberately bypasses the star gates. Someone who followed a link to hole 41
   * was sent there on purpose, and refusing them because of their own save file
   * would make every shared link a dead end for most of the people who click it.
   */
  private async openDeepLink(link: DeepLink): Promise<void> {
    if (link.kind === 'campaign') {
      const level = ALL_LEVELS.find((l) => l.id === link.levelId);
      if (level) {
        this.playLevel(level);
        return;
      }
      // A link to a hole this build does not have. Say so rather than hanging.
      this.showTitle();
      this.hud.showToast('That hole is not in this version', 2.6, 'bad');
      return;
    }
    if (link.kind === 'round') {
      this.playRound(link.seed);
      return;
    }
    await this.playGenerated(
      link.seed,
      undefined,
      'Rebuilding a shared hole',
      'Generating the same course from its seed and checking it can be finished.',
    );
  }

  /**
   * Points the address bar at the hole being played, so the URL is always worth
   * copying. `replaceState`, not `pushState`: the back button should leave the
   * game, not walk back through every hole played this session.
   */
  private syncUrl(level: LevelDef & { seed?: number }): void {
    // Mid-round the shareable thing is the round, not hole four of it.
    const search = this.round
      ? `?round=${this.round.seed}`
      : (() => {
          const link = linkForLevel(level);
          return link ? shareUrl(link, window.location) : null;
        })();
    if (!search) return;
    try {
      window.history.replaceState(
        null,
        '',
        search.startsWith('?')
          ? `${window.location.origin}${window.location.pathname}${search}`
          : search,
      );
    } catch {
      // Blocked in a sandboxed frame or a file:// page. Nothing depends on it.
    }
  }

  /**
   * Drops the hole from the address bar on the way back to a menu, so a refresh
   * from the menu does not throw the player into the hole they just left.
   */
  private clearUrl(): void {
    if (!window.location.search) return;
    try {
      window.history.replaceState(null, '', window.location.pathname);
    } catch {
      // Same sandboxing caveat as syncUrl; nothing depends on it.
    }
  }

  /** Copies a link to the hole being played, falling back to a visible URL. */
  private async shareCurrentHole(): Promise<void> {
    const level = this.session?.level as (LevelDef & { seed?: number }) | undefined;
    const link = level ? linkForLevel(level) : null;
    if (!link) return;

    const url = shareUrl(link, window.location);
    try {
      await navigator.clipboard.writeText(url);
      this.hud.showToast('Link copied', 1.8, 'good');
    } catch {
      // No clipboard permission, or an insecure context. Showing the link is
      // still better than silently doing nothing.
      this.hud.showToast(url, 6);
    }
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
    this.generator.dispose();
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
      // The clock slows on a tense approach, so a near miss is something you
      // watch rather than something that already happened.
      const scaled = dt * this.updateTimeScale(dt, session);
      const events = session.update(scaled);
      this.handleSessionEvents(events);
      this.ghost?.advance(scaled);
      this.updateAim();
      this.updateTrail(scaled, session);
      this.updateCamera(dt, session);
      this.hud.sync(session);
      if (this.flybyTimer > 0) this.flybyTimer = Math.max(0, this.flybyTimer - dt);
    }

    if (!settings.reducedMotion) this.renderer.starfield.update(dt);
    this.renderer.particles.update(dt);
    this.renderer.camera.update(dt);
  }

  /**
   * Eases the simulation clock down while the ball is closing on the cup at a
   * speed that could actually drop, and back up otherwise. Purely presentation:
   * the simulation is fixed-step, so slowing it changes how much sim time a
   * real frame buys, never the trajectory.
   */
  private updateTimeScale(dt: number, session: PlaySession): number {
    let target = 1;

    if (session.state === 'flying' && !this.progress.settings.reducedMotion) {
      const hole = session.world.hole;
      const distance = V.distance(session.ball.position, holePositionAt(session.world));
      const speed = V.length(session.ball.velocity);
      const nearCup = distance < hole.radius * SLOWMO_RADII;
      // Only slow down when a drop is plausible; a screamer flying past the cup
      // is not a tense moment, it is a miss.
      const catchable = speed < hole.captureSpeed * 2.2;
      if (nearCup && catchable) target = SLOWMO_SCALE;
    }

    // Ease so the transition reads as a swell rather than a stutter.
    const rate = target < this.timeScale ? 14 : 6;
    this.timeScale = damp(this.timeScale, target, rate, dt);
    return this.timeScale;
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

    // Opening flyby: frame the whole hole first, then settle onto the ball.
    // Holes are wider than the screen, so without this the player's first look
    // at the layout is the moment they have already committed to a shot.
    if (this.flybyTimer > 0) {
      const bounds = session.world.bounds;
      camera.target = {
        x: (bounds.minX + bounds.maxX) / 2,
        y: (bounds.minY + bounds.maxY) / 2,
      };
      camera.targetZoom = camera.fit(bounds, 40, 1.4);
      return;
    }

    let focus = session.ball.position;
    if (session.state !== 'flying') {
      // While aiming, lean the view toward the hole so the player can see where
      // they are shooting. Capped by the viewport so the ball never slides off.
      const lead = V.sub(holePositionAt(session.world), focus);
      const maxLead = (camera.viewportWidth / Math.max(camera.zoom, 0.01)) * 0.22;
      focus = V.add(focus, V.clampLength(V.mul(lead, 0.35), maxLead));
    }
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

        case 'boost':
          this.audio.play('bumper', 1);
          particles.burst(event.position, '#a6f259', 2.2);
          if (shakeAllowed) camera.shake(2.5, 0.16);
          break;

        case 'switch':
          this.audio.play(event.on ? 'unlock' : 'back');
          particles.burst(event.position, event.on ? palette.holeRim : palette.textDim, 1.6);
          // The off case matters too: a held pad springing back mid-flight is the
          // reason a shot suddenly fails, and silence there reads as a bug.
          this.hud.showToast(
            event.on ? 'Switch thrown' : 'Switch released',
            1.3,
            event.on ? 'good' : 'bad',
          );
          break;

        case 'shatter':
          this.audio.play('bumper', 1);
          particles.explosion(event.position, palette.accentAlt);
          if (shakeAllowed) camera.shake(3, 0.22);
          break;

        case 'locked':
          this.audio.play('lipout');
          this.hud.showToast(
            `The cup is sealed — ${event.needed} more star${event.needed === 1 ? '' : 's'}`,
            1.8,
            'bad',
          );
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

        case 'sunk': {
          this.audio.play('sink');
          particles.sinkCelebration(holePositionAt(session.world), palette.holeRim);
          if (shakeAllowed) camera.shake(4, 0.3);
          const feat = event.result.feats[0];
          if (feat) this.hud.showToast(FEATS[feat].label, 2.4, 'good');
          this.finishHole();
          break;
        }
      }
    }
  }

  /* --------------------------------------------------------------- flow */

  private shoot(direction: Vec2, power: number): void {
    const session = this.session;
    if (!session) return;
    this.aimState = null;
    this.previewCache = null;
    this.clearHintLine();
    this.trail = [];
    this.handleSessionEvents(session.shoot(direction, power));
  }

  playLevel(level: LevelDef): void {
    this.session = new PlaySession(level);
    this.syncUrl(level);
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
    this.clearHintLine();
    this.cameraOffset = V.ZERO;
    this.manualZoom = 1;
    this.timeScale = 1;
    this.renderer.particles.clear();
    this.renderer.resetDecor();
    this.refitCamera();

    // Race your own best run, when there is one recorded and it is wanted.
    const bestShots = this.progress.bestShots(level.id);
    this.ghost =
      this.progress.settings.showGhost && bestShots.length > 0
        ? new GhostRunner(level, bestShots)
        : null;

    const flyby = this.progress.settings.reducedMotion ? 0 : FLYBY_SECONDS;
    this.flybyTimer = flyby;
    if (flyby > 0) {
      const bounds = this.session.world.bounds;
      this.renderer.camera.snapTo(
        { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 },
        this.renderer.camera.fit(bounds, 40, 1.4),
      );
    } else {
      this.renderer.camera.snapTo(this.session.ball.position, this.baseZoom);
    }
    this.renderer.canvas.focus();
  }

  /** Takes back the last shot. */
  private undo(): void {
    const session = this.session;
    if (!session?.canUndo) return;
    if (!session.undo()) return;
    this.audio.play('back');
    this.trail = [];
    this.aimState = null;
    this.previewCache = null;
    this.clearHintLine();
    this.timeScale = 1;
    this.hud.sync(session);
    this.hud.showToast('Shot taken back', 1.2);
  }

  /**
   * Asks the solver for a playable line from where the ball is standing, then
   * draws it and arms the aim to match.
   *
   * The aim is a starting point, not a shot: the player can nudge it or ignore
   * it. What it costs is honesty about the run — a hinted hole still earns its
   * medal, but not the style feats, and it is not kept as the ghost.
   */
  private async requestHint(): Promise<void> {
    const session = this.session;
    if (!session || this.hintPending || !session.canShoot || this.screen !== 'play') return;

    this.hintPending = true;
    this.hud.setHintBusy(true);
    this.hud.showToast('Looking for a line…', 1.6);

    // Captured before the await: if any of it has moved on by the time the
    // solver answers, the answer is about a hole that no longer exists.
    const level = session.level;
    const position = session.ball.position;
    const strokes = session.strokes;

    try {
      const result = await this.generator.hint({
        level,
        position,
        time: session.world.time,
        state: serializeBoardState(session.world),
        maxStrokes: Math.max(1, level.par - strokes),
      });

      if (this.disposed) return;
      const current = this.session;
      if (!current || current.level !== level || current.strokes !== strokes || !current.canShoot) {
        return;
      }

      if (!result?.shot) {
        this.hud.showToast('No line found from here — try a restart', 2.4, 'bad');
        return;
      }

      const direction = V.fromAngle(result.shot.angle);
      current.hintsUsed++;
      this.input.setAim(direction, result.shot.power);
      const preview = current.predict(direction, result.shot.power, { maxTime: 10 });
      this.hintLine = { path: preview.points, strokes: result.strokes };
      this.audio.play('unlock');
      this.hud.showToast(
        result.sinks
          ? `This line sinks it in ${result.strokes}`
          : 'No sink from here — this is the best line on',
        2.6,
        result.sinks ? 'good' : undefined,
      );
    } finally {
      this.hintPending = false;
      if (!this.disposed) this.hud.setHintBusy(false);
    }
  }

  /** Drops the suggested line. It described one shot from one place. */
  private clearHintLine(): void {
    this.hintLine = null;
  }

  private refitCamera(): void {
    const session = this.session;
    if (!session) return;
    // Fit the whole hole when it fits comfortably. On a narrow screen — a phone
    // in portrait against a landscape hole — fitting the full width would
    // shrink the ball to a few pixels, so the zoom floors out and the camera
    // pans with the ball instead.
    this.baseZoom = clamp(this.renderer.camera.fit(session.world.bounds, 60, 1.4), MIN_ZOOM, 1.4);
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

    // Captured either side of the submit: the finale is the moment the *last*
    // unfinished hole is completed, which need not be the last hole in order.
    const wasComplete = this.campaignComplete();
    this.progress.submit(result);
    const justFinished = !wasComplete && this.campaignComplete();
    const upcoming = nextLevel(result.levelId);

    // A beat of celebration before the panel covers the screen.
    window.setTimeout(() => {
      if (this.disposed || this.screen !== 'play') return;

      // Mid-round, the next hole follows straight on: a round is one continuous
      // run, and a results panel between every hole would break it into nine.
      if (this.round) {
        this.advanceRound(result);
        return;
      }

      this.screen = 'results';
      this.hud.setVisible(false);

      if (justFinished) {
        this.audio.play('unlock');
        this.overlay.show(
          campaignCompleteScreen(this.campaignSummary(), {
            onScorecard: () => this.showScorecard('title'),
            onLevels: () => this.showLevels(),
            onTitle: () => this.showTitle(),
          }),
          { onEscape: () => this.showTitle() },
        );
        return;
      }

      this.overlay.show(
        resultsScreen(result, session.level.name, {
          hasNext: upcoming !== undefined,
          isNewBest,
          previousBest,
          onNext: () => upcoming && this.playLevel(upcoming),
          onRetry: () => this.playLevel(session.level),
          onLevels: () => this.showLevels(),
          onShare: linkForLevel(session.level) ? () => void this.shareCurrentHole() : undefined,
        }),
        {},
      );
    }, 900);
  }

  /** True once every campaign hole has been finished at least once. */
  private campaignComplete(): boolean {
    return this.progress.completedCount(CAMPAIGN_IDS) >= CAMPAIGN_IDS.length;
  }

  private campaignSummary(): CampaignSummary {
    const rows = buildScorecard(ALL_LEVELS, this.progress);
    return {
      totals: scorecardTotals(rows),
      feats: this.progress.earnedFeats().length,
      featsTotal: Object.keys(FEATS).length,
      playTime: this.progress.totalPlayTime,
      underPar: rows.filter((row) => row.strokes !== null && row.strokes < row.level.par).length,
    };
  }

  /**
   * Starts a nine-hole round drawn from `seed`.
   *
   * A round is scored as a whole, so its holes deliberately do not touch the
   * campaign's per-hole records — a hole you happened to draw badly should not
   * overwrite the best you ever played it.
   */
  playRound(seed: number): void {
    this.round = buildRound(seed, ALL_LEVELS);
    if (this.round.holes.length === 0) {
      this.round = null;
      this.showTitle();
      return;
    }
    this.playRoundHole();
  }

  private playRoundHole(): void {
    const round = this.round;
    if (!round) return;
    const hole = round.holes[round.index];
    if (!hole) return;
    this.playLevel(hole.level);
    this.hud.showToast(`Round · hole ${round.index + 1} of ${round.holes.length}`, 2.2);
  }

  /** Records the hole just finished and moves the round on. */
  private advanceRound(result: HoleResult): void {
    const round = this.round;
    if (!round) return;
    const hole = round.holes[round.index];
    if (hole) {
      hole.strokes = result.strokes;
      hole.stars = result.stars;
    }
    round.index++;

    if (!roundFinished(round)) {
      this.playRoundHole();
      return;
    }

    const strokes = roundStrokes(round);
    const previousBest = this.progress.bestRound;
    this.progress.submitRound(strokes);
    this.audio.play('unlock');
    this.screen = 'results';
    this.hud.setVisible(false);
    this.round = null;
    this.overlay.show(
      roundCompleteScreen(round, previousBest, {
        onAgain: () => this.playRound(randomRoundSeed()),
        onShare: () => void this.shareRound(round.seed),
        onTitle: () => this.showTitle(),
      }),
      { onEscape: () => this.showTitle(), wide: true },
    );
  }

  /** Copies a link that rebuilds this exact round. */
  private async shareRound(seed: number): Promise<void> {
    const url = `${window.location.origin}${window.location.pathname}?round=${seed}`;
    try {
      await navigator.clipboard.writeText(url);
      this.hud.showToast('Link copied', 1.8, 'good');
    } catch {
      this.hud.showToast(url, 6);
    }
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
        onShare: linkForLevel(session.level) ? () => void this.shareCurrentHole() : undefined,
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
    this.clearUrl();
    // Walking away from a round ends it; otherwise finishing some unrelated
    // hole later would quietly count towards it.
    this.round = null;
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
          onDaily: () => void this.playDaily(),
          onRandom: () => void this.playRandom(),
          onRound: () => this.playRound(randomRoundSeed()),
        },
        this.progress,
        CAMPAIGN_IDS,
        this.progress.isCompleted(dailyId(new Date())),
      ),
      {},
    );
  }

  /* --------------------------------------------------- generated holes */

  /**
   * The daily challenge: one generated hole per calendar day, the same for
   * everyone. It is cached after the first generation, so it is only ever
   * verified once per device per day.
   */
  private async playDaily(): Promise<void> {
    const today = new Date();
    await this.playGenerated(
      dailySeed(today),
      dailyId(today),
      'Building today\u2019s hole',
      'Generating a course and checking it can actually be finished.',
    );
  }

  /** An endless supply of one-off holes. Not cached — every press is new. */
  private async playRandom(): Promise<void> {
    // Math.random is fine here: the seed only has to differ between presses.
    const seed = Math.floor(Math.random() * 0xffffffff);
    await this.playGenerated(
      seed,
      undefined,
      'Building a hole',
      'Generating a course and checking it can actually be finished.',
    );
  }

  private async playGenerated(
    seed: number,
    cacheKey: string | undefined,
    message: string,
    detail: string,
  ): Promise<void> {
    if (this.generating) return;
    this.generating = true;
    const from = this.screen;
    this.overlay.show(loadingScreen(message, detail), {});

    try {
      const level = await this.generator.generate(seed, cacheKey);
      if (this.disposed) return;

      if (!level) {
        this.overlay.show(
          loadingScreen('Could not build a hole', 'Something went wrong. Please try again.'),
          {},
        );
        window.setTimeout(() => {
          if (!this.disposed && this.screen === from) this.showTitle();
        }, 1600);
        return;
      }

      this.playLevel(cacheKey ? { ...level, id: cacheKey, name: `Daily: ${level.name}` } : level);
    } finally {
      this.generating = false;
    }
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
    this.clearUrl();
    // Walking away from a round ends it; otherwise finishing some unrelated
    // hole later would quietly count towards it.
    this.round = null;
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
        this.progress.totalStars(CAMPAIGN_IDS),
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
      scorecardScreen(
        buildScorecard(ALL_LEVELS, this.progress),
        CHAPTERS,
        this.progress.earnedFeats(),
        () => this.leaveSubScreen(),
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
        onShare: linkForLevel(session.level) ? () => void this.shareCurrentHole() : undefined,
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
    this.clearHintLine();
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
        case 'KeyZ':
        case 'Backspace':
          if (this.screen === 'play') {
            event.preventDefault();
            this.undo();
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
        case 'KeyH':
          if (this.screen === 'play') {
            event.preventDefault();
            void this.requestHint();
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
          skin: resolveSkin(this.progress.settings.ballSkin, this.progress.totalStars(CAMPAIGN_IDS)),
          ghost: null,
          hintPath: null,
        },
        { ...options, showGravityField: false },
      );
      return;
    }

    const ghost = this.ghost;
    const scene: Scene = {
      world: session.world,
      ballPosition: session.ball.position,
      ballRadius: session.ball.radius,
      ballVisible: session.state !== 'respawning',
      trail: this.trail,
      aim: this.screen === 'play' ? this.aimState : null,
      time: session.totalTime,
      levelId: session.level.id,
      skin: resolveSkin(this.progress.settings.ballSkin, this.progress.totalStars(CAMPAIGN_IDS)),
      ghost:
        ghost && ghost.visible
          ? { position: ghost.position, trail: ghost.trail }
          : null,
      hintPath: this.screen === 'play' ? (this.hintLine?.path ?? null) : null,
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
  switches: [],
  boosters: [],
  breakables: {},
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
