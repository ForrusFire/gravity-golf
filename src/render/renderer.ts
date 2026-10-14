import { TAU, clamp } from '../core/math';
import { Rng, hashSeed } from '../core/rng';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import {
  bodyShapeAt,
  containsPoint,
  shapeBounds,
  shapeCenter,
  shapeRadius,
  worldVertices,
  type Aabb,
} from '../physics/geometry';
import { MATERIALS, type Body, type Shape, type Zone } from '../physics/types';
import {
  gravityAt,
  holePositionAt,
  isBodyActive,
  pulsePhase,
  zoneActive,
  type World,
} from '../physics/world';
import { Camera } from './camera';
import { ParticleSystem } from './particles';
import { colorsForBody, powerColor, styleForBody, type BodyColors, type Palette } from './palette';
import { LIGHT, PlanetPainter, lighten, withAlpha } from './sprites';
import { Starfield } from './starfield';
import type { BallSkin } from '../game/skins';

export interface AimState {
  origin: Vec2;
  direction: Vec2;
  power: number;
  /** Sampled preview path, world space. Empty hides the guide. */
  preview: Vec2[];
  previewOutcome: 'sink' | 'death' | 'rest' | 'truncated';
}

export interface RenderOptions {
  palette: Palette;
  showGravityField: boolean;
  reducedMotion: boolean;
  /** 0 disables the aim preview line entirely. */
  aimAssist: number;
}

export interface Scene {
  world: World;
  ballPosition: Vec2;
  ballRadius: number;
  ballVisible: boolean;
  /** Recent ball positions, oldest first. */
  trail: Vec2[];
  aim: AimState | null;
  /** Seconds since the level started, for animation phase. */
  time: number;
  levelId: string;
  /** Ball appearance. */
  skin: BallSkin;
  /** A replay of the player's best run, drawn behind the live ball. */
  ghost: { position: Vec2; trail: Vec2[] } | null;
  /** The path of a suggested shot, drawn while the hint is on screen. */
  hintPath: Vec2[] | null;
  /** The ball is waiting for a shot: draw a beacon so it is easy to find. */
  ballReady?: boolean;
  /** A menu backdrop: no cup to point at. */
  backdrop?: boolean;
}

const overlaps = (a: Aabb, b: Aabb): boolean =>
  a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;

/** Boost rings. A hue nothing else in the game uses. */
const BOOST_COLOR = '#a6f259';

/** Only solid ground gets cratered — a star or a goo blob should not. */
const ROCKY_STYLES = new Set<string>(['planet', 'moon', 'asteroid', 'sand']);

/** Styles with a pre-painted sprite. Suns, black holes and bumpers move or glow, so stay live. */
const PAINTED_STYLES = new Set<string>(['planet', 'moon', 'asteroid', 'sand', 'ice', 'goo', 'crystal']);

/** Deterministic surface detail so a given planet always looks the same. */
interface BodyDecor {
  craters: Array<{ x: number; y: number; r: number }>;
}

export class Renderer {
  readonly camera = new Camera();
  readonly particles = new ParticleSystem();
  readonly starfield = new Starfield();

  private ctx: CanvasRenderingContext2D;
  private pixelRatio = 1;
  private decorCache = new Map<string, BodyDecor>();
  private painter = new PlanetPainter();
  private flashState: { color: string; amount: number; start: number | null } | null = null;

  constructor(readonly canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d', { alpha: false });
    if (!ctx) throw new Error('Canvas 2D context is unavailable');
    this.ctx = ctx;
  }

  /** Sizes the backing store to the element, accounting for HiDPI displays. */
  resize(cssWidth: number, cssHeight: number, pixelRatio = window.devicePixelRatio || 1): void {
    // Cap the ratio: a 3x buffer on a large screen costs more than it shows.
    this.pixelRatio = clamp(pixelRatio, 1, 2);
    this.canvas.width = Math.max(1, Math.round(cssWidth * this.pixelRatio));
    this.canvas.height = Math.max(1, Math.round(cssHeight * this.pixelRatio));
    this.canvas.style.width = `${cssWidth}px`;
    this.canvas.style.height = `${cssHeight}px`;
    this.camera.setViewport(cssWidth, cssHeight);
  }

  get width(): number {
    return this.camera.viewportWidth;
  }

  get height(): number {
    return this.camera.viewportHeight;
  }

  draw(scene: Scene, options: RenderOptions): void {
    const ctx = this.ctx;
    ctx.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
    ctx.clearRect(0, 0, this.width, this.height);

    this.starfield.draw(
      ctx,
      this.camera.eye,
      this.width,
      this.height,
      this.camera.zoom,
      options.palette,
      !options.reducedMotion,
    );

    ctx.save();
    this.applyWorldTransform(ctx);

    const view = this.camera.visibleBounds(120);

    if (options.showGravityField) {
      this.drawGravityField(ctx, scene.world, options.palette, scene.time, !options.reducedMotion);
    }
    this.drawBoundary(ctx, scene.world, options.palette, scene.time);
    for (const zone of scene.world.zones) {
      this.drawZone(ctx, zone, scene.time, options.palette, scene.world.time);
    }
    for (const portal of scene.world.portals) {
      this.drawPortal(ctx, portal.from, portal.to, portal.radius, scene.time, options.palette);
    }

    for (const body of scene.world.bodies) {
      const shape = bodyShapeAt(body, scene.world.time);
      if (!overlaps(shapeBounds(shape), view)) continue;

      if (!isBodyActive(scene.world, body)) {
        // A bridge waiting on a switch is drawn as an outline. Telegraphing
        // what will appear is the difference between a puzzle and a surprise.
        if (body.addedBy) this.drawPendingBody(ctx, shape, scene.time, options.palette);
        // The same for a pulsing body between beats — but it also gets a filling
        // ring showing how much of the gap is left, so the rhythm is something
        // to play to rather than something to be caught out by.
        if (body.pulse) {
          this.drawPendingBody(ctx, shape, scene.time, options.palette);
          this.drawPulseClock(ctx, shape, pulsePhase(body.pulse, scene.world.time), false);
        }
        continue;
      }
      this.drawBody(ctx, body, shape, options.palette, scene.world, scene.time);
      if (body.pulse) {
        this.drawPulseClock(ctx, shape, pulsePhase(body.pulse, scene.world.time), true);
      }
      if (body.needsStars !== undefined) {
        const owed =
          body.needsStars - scene.world.collectibles.filter((c) => c.collected).length;
        this.drawToll(ctx, shape, Math.max(0, owed), options.palette);
      }
    }

    for (const boost of scene.world.boosters) {
      this.drawBooster(ctx, boost, scene.time);
    }

    for (const pad of scene.world.switches) {
      this.drawSwitch(ctx, pad, scene.time, scene.world.time, options.palette);
    }

    // After the bodies: a cup sunk into the ground must not be painted over by
    // the ground itself.
    this.drawHole(ctx, scene.world, scene.time, options.palette);

    this.drawCollectibles(ctx, scene, options.palette);
    this.particles.draw(ctx);

    // Above the ghost but below the ball: it is advice about the next shot, not
    // a record of a past one.
    if (scene.hintPath) this.drawHintPath(ctx, scene.hintPath, scene.time, options.palette);

    // The ghost goes underneath, so it can never be mistaken for the live ball.
    if (scene.ghost) this.drawGhost(ctx, scene.ghost, scene.ballRadius, options.palette);

    if (scene.ballVisible) {
      this.drawTrail(ctx, scene.trail, scene.ballRadius, scene.skin.trail);
      if (scene.ballReady && !scene.aim && !options.reducedMotion) {
        this.drawBeacon(ctx, scene.ballPosition, scene.ballRadius, scene.time, scene.skin);
      }
      this.drawBall(ctx, scene.ballPosition, scene.ballRadius, scene.time, scene.skin);
    }

    if (scene.aim && options.aimAssist > 0) {
      this.drawAim(ctx, scene.aim, scene.ballRadius, scene.time, options.palette);
    }

    ctx.restore();

    // Screen space, after the world transform is popped.
    this.starfield.drawVignette(ctx, this.width, this.height);
    if (!options.reducedMotion) this.drawFlash(ctx, scene.time);
    if (!scene.backdrop) this.drawOffscreenMarker(ctx, scene, options.palette);
  }

  /**
   * An edge arrow pointing at the hole when it is off-screen.
   *
   * On a narrow screen the camera cannot show a whole hole at a legible scale,
   * so without this the player is aiming at a target they cannot see.
   */
  private drawOffscreenMarker(
    ctx: CanvasRenderingContext2D,
    scene: Scene,
    palette: Palette,
  ): void {
    const hole = holePositionAt(scene.world);
    const screen = this.camera.worldToScreen(hole);
    const margin = 46;
    const onScreen =
      screen.x >= margin &&
      screen.x <= this.width - margin &&
      screen.y >= margin &&
      screen.y <= this.height - margin;
    if (onScreen) return;

    const cx = this.width / 2;
    const cy = this.height / 2;
    const dir = V.normalize({ x: screen.x - cx, y: screen.y - cy });
    if (dir.x === 0 && dir.y === 0) return;

    // Walk from the centre to the first viewport edge the direction crosses.
    const halfW = cx - margin;
    const halfH = cy - margin;
    const scale = Math.min(
      Math.abs(dir.x) > 1e-6 ? halfW / Math.abs(dir.x) : Infinity,
      Math.abs(dir.y) > 1e-6 ? halfH / Math.abs(dir.y) : Infinity,
    );
    const x = cx + dir.x * scale;
    const y = cy + dir.y * scale;

    const distance = Math.round(V.distance(scene.ballPosition, hole));

    ctx.save();
    ctx.translate(x, y);

    ctx.fillStyle = palette.panel;
    ctx.strokeStyle = palette.holeRim;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(0, 0, 17, 0, TAU);
    ctx.fill();
    ctx.stroke();

    ctx.rotate(Math.atan2(dir.y, dir.x));
    ctx.fillStyle = palette.holeRim;
    ctx.beginPath();
    ctx.moveTo(11, 0);
    ctx.lineTo(-2, -7);
    ctx.lineTo(-2, 7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    ctx.save();
    ctx.fillStyle = palette.textDim;
    ctx.font = '600 11px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    // Keep the label inside the viewport when the arrow hugs an edge.
    const labelY = clamp(y + 30, 14, this.height - 14);
    ctx.fillText(`${distance}m`, clamp(x, 22, this.width - 22), labelY);
    ctx.restore();
  }

  private applyWorldTransform(ctx: CanvasRenderingContext2D): void {
    const eye = this.camera.eye;
    ctx.translate(this.width / 2, this.height / 2);
    ctx.scale(this.camera.zoom, this.camera.zoom);
    ctx.translate(-eye.x, -eye.y);
  }

  /* ------------------------------------------------------------- gravity */

  /**
   * Motes drifting down the local field, over faint guide strokes. Direction
   * and strength read at a glance — which way space pulls and how hard — and
   * the motion makes the invisible field feel like a current, which is the
   * whole game.
   */
  private drawGravityField(
    ctx: CanvasRenderingContext2D,
    world: World,
    palette: Palette,
    time: number,
    animate: boolean,
  ): void {
    const view = this.camera.visibleBounds(0);
    // Keep the on-screen spacing roughly constant as the player zooms.
    const spacing = 46 / this.camera.zoom;
    const startX = Math.floor(view.minX / spacing) * spacing;
    const startY = Math.floor(view.minY / spacing) * spacing;
    const len = spacing * 0.42;
    const unit = 1 / this.camera.zoom;

    ctx.save();
    ctx.strokeStyle = palette.fieldLine;
    ctx.lineCap = 'round';

    for (let x = startX; x <= view.maxX + spacing; x += spacing) {
      for (let y = startY; y <= view.maxY + spacing; y += spacing) {
        // A stable per-cell hash: jitter breaks the grid up so it reads as a
        // field rather than graph paper, and staggers the motes' phases.
        const ix = Math.round(x / spacing);
        const iy = Math.round(y / spacing);
        const h = Math.sin(ix * 127.1 + iy * 311.7) * 43758.5453;
        const hash = h - Math.floor(h);
        const px = x + (hash - 0.5) * spacing * 0.5;
        const py = y + (((hash * 7.13) % 1) - 0.5) * spacing * 0.5;
        const g = gravityAt(world, { x: px, y: py });
        const magnitude = Math.hypot(g.x, g.y);
        // Below this the field is not worth the visual noise.
        if (magnitude < 6) continue;

        // Log scaling: real fields span orders of magnitude across one screen.
        const strength = clamp(Math.log10(magnitude) / 2.8, 0.1, 1);
        const dx = g.x / magnitude;
        const dy = g.y / magnitude;

        ctx.globalAlpha = strength * 0.08;
        ctx.lineWidth = 1.2 * unit;
        ctx.beginPath();
        ctx.moveTo(px - dx * len, py - dy * len);
        ctx.lineTo(px + dx * len, py + dy * len);
        ctx.stroke();

        // Where the pull is only the hole's steady background gravity, the
        // guide says enough; motes there just look like rain. They are for
        // the wells, where the field bends.
        const ux = g.x - world.uniformGravity.x;
        const uy = g.y - world.uniformGravity.y;
        if (ux * ux + uy * uy < 36) continue;

        // The mote: quicker where the pull is stronger.
        const phase = animate ? (time * (0.35 + strength * 0.9) + hash) % 1 : 0.7;
        const head = len * (phase * 2 - 1);
        const tail = head - len * 0.7;
        ctx.globalAlpha = strength * 0.42 * Math.sin(phase * Math.PI);
        ctx.lineWidth = (1.3 + strength * 1.1) * unit;
        ctx.beginPath();
        ctx.moveTo(px + dx * tail, py + dy * tail);
        ctx.lineTo(px + dx * head, py + dy * head);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  /**
   * The edge of play. A soft glowing seam rather than a dashed rectangle: red
   * and breathing when crossing it loses the ball, cool and still when it is a
   * wall to bank off.
   */
  private drawBoundary(
    ctx: CanvasRenderingContext2D,
    world: World,
    palette: Palette,
    time: number,
  ): void {
    if (world.boundsMode === 'open') return;
    const b = world.bounds;
    const w = b.maxX - b.minX;
    const h = b.maxY - b.minY;
    const unit = 1 / this.camera.zoom;
    const deadly = world.boundsMode !== 'wall';
    const color = deadly ? palette.danger : palette.accent;
    const pulse = deadly ? 0.8 + Math.sin(time * 2.2) * 0.2 : 0.7;

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineJoin = 'round';
    for (const [width, alpha] of [
      [18, 0.05],
      [8, 0.1],
      [2, 0.5],
    ] as const) {
      ctx.globalAlpha = alpha * pulse;
      ctx.lineWidth = width * unit;
      ctx.strokeRect(b.minX, b.minY, w, h);
    }

    // Corner brackets, so the extent reads even when an edge is off-screen.
    const arm = Math.min(w, h) * 0.06;
    ctx.globalAlpha = 0.85 * pulse;
    ctx.lineWidth = 3 * unit;
    ctx.lineCap = 'round';
    ctx.beginPath();
    for (const [cx, cy, sx, sy] of [
      [b.minX, b.minY, 1, 1],
      [b.maxX, b.minY, -1, 1],
      [b.minX, b.maxY, 1, -1],
      [b.maxX, b.maxY, -1, -1],
    ] as const) {
      ctx.moveTo(cx + sx * arm, cy);
      ctx.lineTo(cx, cy);
      ctx.lineTo(cx, cy + sy * arm);
    }
    ctx.stroke();
    ctx.restore();
  }

  /* --------------------------------------------------------------- zones */

  private drawZone(
    ctx: CanvasRenderingContext2D,
    zone: Zone,
    time: number,
    palette: Palette,
    worldTime = time,
  ): void {
    // A pulsing zone still has to be visible while it is off, or the player is
    // asked to plan around something that is not on screen. It goes faint, and
    // wears the same countdown ring as a pulsing wall.
    const on = zoneActive(zone, worldTime);
    ctx.save();
    if (!on) ctx.globalAlpha = 0.3;
    ctx.save();
    this.tracePath(ctx, zone.area);

    switch (zone.kind) {
      case 'hazard': {
        ctx.fillStyle = 'rgba(255, 70, 100, 0.14)';
        ctx.fill();
        ctx.strokeStyle = palette.danger;
        ctx.lineWidth = 2 / this.camera.zoom;
        ctx.setLineDash([9 / this.camera.zoom, 7 / this.camera.zoom]);
        ctx.lineDashOffset = -time * 24;
        ctx.stroke();
        break;
      }
      case 'nebula': {
        const c = shapeCenter(zone.area);
        const r = shapeRadius(zone.area);
        const grad = ctx.createRadialGradient(c.x, c.y, 0, c.x, c.y, Math.max(r, 1));
        grad.addColorStop(0, 'rgba(150, 110, 210, 0.34)');
        grad.addColorStop(1, 'rgba(150, 110, 210, 0.05)');
        ctx.fillStyle = grad;
        ctx.fill();
        break;
      }
      case 'wind':
      case 'boost': {
        ctx.fillStyle = 'rgba(90, 209, 255, 0.10)';
        ctx.fill();
        ctx.clip();
        this.drawFlowArrows(ctx, zone.area, zone.force, time, palette);
        break;
      }
      case 'vortex': {
        const c = shapeCenter(zone.area);
        const r = shapeRadius(zone.area);
        ctx.strokeStyle = 'rgba(199, 146, 255, 0.5)';
        ctx.lineWidth = 2 / this.camera.zoom;
        for (let i = 0; i < 3; i++) {
          const phase = time * 0.9 * Math.sign(zone.swirl || 1) + (i * TAU) / 3;
          ctx.beginPath();
          for (let t = 0; t <= 1.001; t += 0.05) {
            const radius = r * t;
            const angle = phase + t * 5;
            const x = c.x + Math.cos(angle) * radius;
            const y = c.y + Math.sin(angle) * radius;
            if (t === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          }
          ctx.stroke();
        }
        break;
      }
      case 'gravityScale': {
        // Three distinct readings, because a zone that reverses gravity and a
        // zone that merely softens it demand opposite shots. Reversal also gets
        // a glyph, so it is not colour alone that tells them apart.
        const reversed = zone.scale < 0;
        const tint = reversed ? '#ff7ae0' : zone.scale < 1 ? '#78ffd2' : '#ffa05a';
        ctx.strokeStyle = tint;
        ctx.globalAlpha = 0.55;
        ctx.lineWidth = (reversed ? 3 : 2) / this.camera.zoom;
        const dash = reversed ? [14, 7] : [6, 6];
        ctx.setLineDash(dash.map((d) => d / this.camera.zoom));
        ctx.stroke();
        ctx.globalAlpha = reversed ? 0.11 : 0.09;
        ctx.fillStyle = tint;
        ctx.fill();
        if (reversed) this.drawReversalGlyphs(ctx, zone.area, time, tint);
        break;
      }
    }
    ctx.restore();

    if (zone.pulse) {
      this.drawPulseClock(ctx, zone.area, pulsePhase(zone.pulse, worldTime), on);
    }
    ctx.restore();
  }

  /**
   * Chevrons drifting upward through a reversal zone. Shape, not just hue: the
   * one thing a player must know here is "things fall the other way".
   */
  private drawReversalGlyphs(
    ctx: CanvasRenderingContext2D,
    area: Shape,
    time: number,
    tint: string,
  ): void {
    const bounds = shapeBounds(area);
    const spacing = 76;
    // Rising, so the drift itself reads as "up".
    const drift = spacing - ((time * 46) % spacing);
    const w = 11;

    ctx.strokeStyle = tint;
    ctx.globalAlpha = 0.42;
    ctx.lineWidth = 2.2 / this.camera.zoom;
    ctx.lineCap = 'round';
    ctx.setLineDash([]);
    ctx.beginPath();
    for (let x = bounds.minX + spacing / 2; x < bounds.maxX; x += spacing) {
      for (let y = bounds.maxY - drift; y > bounds.minY; y -= spacing) {
        if (!containsPoint(area, { x, y })) continue;
        ctx.moveTo(x - w, y + w * 0.6);
        ctx.lineTo(x, y - w * 0.6);
        ctx.lineTo(x + w, y + w * 0.6);
      }
    }
    ctx.stroke();
  }

  private drawFlowArrows(
    ctx: CanvasRenderingContext2D,
    area: Shape,
    force: Vec2,
    time: number,
    palette: Palette,
  ): void {
    const bounds = shapeBounds(area);
    const dir = V.normalize(force);
    if (dir.x === 0 && dir.y === 0) return;
    const spacing = 42;
    const drift = ((time * 60) % spacing) - spacing;

    ctx.strokeStyle = palette.accent;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = 2 / this.camera.zoom;
    ctx.lineCap = 'round';

    const perp = V.perp(dir);
    const cx = (bounds.minX + bounds.maxX) / 2;
    const cy = (bounds.minY + bounds.maxY) / 2;
    const span = Math.hypot(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY) / 2;

    const head = 5;
    for (let lane = -span; lane <= span; lane += spacing) {
      for (let along = -span + drift; along <= span; along += spacing) {
        const x = cx + dir.x * along + perp.x * lane;
        const y = cy + dir.y * along + perp.y * lane;
        const tipX = x + dir.x * 14;
        const tipY = y + dir.y * 14;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(tipX, tipY);
        // Arrowheads: the tick alone shows the axis but not which way it pushes.
        ctx.moveTo(tipX - dir.x * head + perp.x * head, tipY - dir.y * head + perp.y * head);
        ctx.lineTo(tipX, tipY);
        ctx.lineTo(tipX - dir.x * head - perp.x * head, tipY - dir.y * head - perp.y * head);
        ctx.stroke();
      }
    }
  }

  /* ------------------------------------------------------------- portals */

  /**
   * Two linked rings joined by a faint tether, so the pairing is obvious even
   * when the mouths are on opposite sides of the level.
   */
  private drawPortal(
    ctx: CanvasRenderingContext2D,
    from: Vec2,
    to: Vec2,
    radius: number,
    time: number,
    palette: Palette,
  ): void {
    ctx.save();

    ctx.strokeStyle = palette.accentAlt;
    ctx.globalAlpha = 0.18;
    ctx.lineWidth = 1.5 / this.camera.zoom;
    ctx.setLineDash([8 / this.camera.zoom, 10 / this.camera.zoom]);
    ctx.lineDashOffset = -time * 30;
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();
    ctx.setLineDash([]);

    const mouth = (p: Vec2, phase: number): void => {
      const grad = ctx.createRadialGradient(p.x, p.y, radius * 0.2, p.x, p.y, radius * 1.8);
      grad.addColorStop(0, 'rgba(199, 146, 255, 0.45)');
      grad.addColorStop(1, 'rgba(199, 146, 255, 0)');
      ctx.globalAlpha = 1;
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius * 1.8, 0, TAU);
      ctx.fill();

      ctx.strokeStyle = palette.accentAlt;
      for (let ring = 0; ring < 3; ring++) {
        // Rings pulse outward on a loop to suggest a one-way flow.
        const t = ((time * 0.7 + phase + ring / 3) % 1 + 1) % 1;
        ctx.globalAlpha = 0.75 * (1 - t);
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius * (0.35 + t * 0.75), 0, TAU);
        ctx.stroke();
      }

      ctx.globalAlpha = 0.95;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, TAU);
      ctx.stroke();
    };

    mouth(from, 0);
    mouth(to, 0.5);
    ctx.restore();
  }

  /* -------------------------------------------------------------- bodies */

  private tracePath(ctx: CanvasRenderingContext2D, shape: Shape): void {
    ctx.beginPath();
    switch (shape.kind) {
      case 'circle':
        ctx.arc(shape.center.x, shape.center.y, shape.radius, 0, TAU);
        break;
      case 'capsule': {
        const axis = V.normalize(V.sub(shape.b, shape.a));
        const n = V.perp(axis);
        const r = shape.radius;
        const startAngle = Math.atan2(n.y, n.x);
        ctx.moveTo(shape.a.x + n.x * r, shape.a.y + n.y * r);
        ctx.lineTo(shape.b.x + n.x * r, shape.b.y + n.y * r);
        ctx.arc(shape.b.x, shape.b.y, r, startAngle, startAngle + Math.PI, true);
        ctx.lineTo(shape.a.x - n.x * r, shape.a.y - n.y * r);
        ctx.arc(shape.a.x, shape.a.y, r, startAngle + Math.PI, startAngle, true);
        break;
      }
      case 'polygon': {
        const verts = worldVertices(shape);
        if (verts.length === 0) break;
        ctx.moveTo(verts[0]!.x, verts[0]!.y);
        for (let i = 1; i < verts.length; i++) ctx.lineTo(verts[i]!.x, verts[i]!.y);
        ctx.closePath();
        break;
      }
    }
  }

  private decorFor(body: Body, radius: number): BodyDecor {
    const cached = this.decorCache.get(body.id);
    if (cached) return cached;
    const rng = new Rng(hashSeed(body.id));
    const craters: BodyDecor['craters'] = [];
    const count = radius > 30 ? rng.int(3, 6) : 0;
    for (let i = 0; i < count; i++) {
      const a = rng.range(0, TAU);
      const d = rng.range(0.15, 0.68) * radius;
      craters.push({
        x: Math.cos(a) * d,
        y: Math.sin(a) * d,
        r: rng.range(0.06, 0.17) * radius,
      });
    }
    const decor = { craters };
    this.decorCache.set(body.id, decor);
    return decor;
  }

  /** A body that a switch has yet to bring into existence. */
  /**
   * A ring around a pulsing body counting down to its next change. Amber while
   * it is solid and about to go, green while it is gone and about to return —
   * so the answer to "can I be there in time" is on screen, not in the player's
   * head.
   */
  private drawPulseClock(
    ctx: CanvasRenderingContext2D,
    shape: Shape,
    phase: number,
    solid: boolean,
  ): void {
    const center = shapeCenter(shape);
    // Capped, not proportional: a long wall's own radius is half its length, and
    // a countdown ring that size swamps the hole it is annotating.
    const radius = clamp(shapeRadius(shape) + 12, 22, 46);
    ctx.save();
    ctx.lineWidth = 3.5;
    ctx.lineCap = 'butt';
    ctx.globalAlpha = solid ? 0.5 : 0.75;
    ctx.strokeStyle = solid ? '#ffa05a' : '#66e6b8';
    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, -Math.PI / 2, -Math.PI / 2 + TAU * (1 - phase));
    ctx.stroke();
    ctx.restore();
  }

  /**
   * Star pips on a toll gate, counting what is still owed. Without them the wall
   * is indistinguishable from a permanent one and the stars look optional.
   */
  private drawToll(
    ctx: CanvasRenderingContext2D,
    shape: Shape,
    owed: number,
    palette: Palette,
  ): void {
    if (owed <= 0) return;
    const center = shapeCenter(shape);
    ctx.save();
    ctx.fillStyle = palette.collectible;
    ctx.globalAlpha = 0.95;
    const spread = 13;
    for (let i = 0; i < owed; i++) {
      const x = center.x + (i - (owed - 1) / 2) * spread;
      this.traceStar(ctx, x, center.y, 5.5);
      ctx.fill();
    }
    ctx.restore();
  }

  private drawPendingBody(
    ctx: CanvasRenderingContext2D,
    shape: Shape,
    time: number,
    palette: Palette,
  ): void {
    ctx.save();
    ctx.globalAlpha = 0.3 + Math.sin(time * 2.6) * 0.1;
    ctx.strokeStyle = palette.accent;
    ctx.lineWidth = 2 / this.camera.zoom;
    ctx.setLineDash([7 / this.camera.zoom, 6 / this.camera.zoom]);
    this.tracePath(ctx, shape);
    ctx.stroke();
    ctx.restore();
  }

  /** A switch pad, lit once thrown. */
  /**
   * A boost ring. Lime and arrow-shaped: it is the only thing in the game that
   * *gives* the ball speed on a heading of its own choosing, so it gets a hue
   * nothing else uses and a chevron stack nobody could read as decoration.
   */
  private drawBooster(
    ctx: CanvasRenderingContext2D,
    boost: { position: Vec2; radius: number; direction: Vec2; speed: number },
    time: number,
  ): void {
    const { x, y } = boost.position;
    const angle = Math.atan2(boost.direction.y, boost.direction.x);
    ctx.save();

    const grad = ctx.createRadialGradient(x, y, boost.radius * 0.2, x, y, boost.radius * 1.9);
    grad.addColorStop(0, 'rgba(166,242,89,0.24)');
    grad.addColorStop(1, 'rgba(166,242,89,0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(x, y, boost.radius * 1.9, 0, TAU);
    ctx.fill();

    ctx.strokeStyle = BOOST_COLOR;
    ctx.lineWidth = 3;
    ctx.globalAlpha = 0.9;
    // The mouth faces back the way the ball leaves: an open arc you fly out of.
    ctx.beginPath();
    ctx.arc(x, y, boost.radius, angle + 0.62, angle - 0.62);
    ctx.stroke();

    ctx.translate(x, y);
    ctx.rotate(angle);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    // Chevrons marching outward, so the heading reads at a glance and the ring
    // looks alive rather than like another piece of scenery.
    const march = (time * 2.2) % 1;
    for (let i = 0; i < 3; i++) {
      const t = (i + march) / 3;
      const r = boost.radius * (0.15 + t * 1.0);
      ctx.globalAlpha = 0.9 * (1 - Math.abs(t - 0.5) * 1.2);
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(r - 9, -9);
      ctx.lineTo(r, 0);
      ctx.lineTo(r - 9, 9);
      ctx.stroke();
    }
    ctx.restore();
  }

  private drawSwitch(
    ctx: CanvasRenderingContext2D,
    pad: { position: Vec2; radius: number; on: boolean; holdTime?: number; offAt?: number },
    time: number,
    worldTime: number,
    palette: Palette,
  ): void {
    // Violet, not the cyan accent: an unthrown pad and a teal bumper are often
    // side by side, and mistaking machinery for a bounce surface loses a stroke.
    const color = pad.on ? palette.holeRim : palette.accentAlt;
    ctx.save();

    const grad = ctx.createRadialGradient(
      pad.position.x,
      pad.position.y,
      0,
      pad.position.x,
      pad.position.y,
      pad.radius * 2,
    );
    grad.addColorStop(0, pad.on ? 'rgba(102,230,184,0.34)' : 'rgba(199,146,255,0.22)');
    grad.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(pad.position.x, pad.position.y, pad.radius * 2, 0, TAU);
    ctx.fill();

    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    if (!pad.on) {
      // An unthrown switch pulses, so it reads as something to go and hit.
      ctx.globalAlpha = 0.55 + Math.sin(time * 3.4) * 0.25;
      ctx.setLineDash([6, 5]);
      ctx.lineDashOffset = -time * 18;
    }
    ctx.beginPath();
    ctx.arc(pad.position.x, pad.position.y, pad.radius, 0, TAU);
    ctx.stroke();

    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.fillStyle = color;
    if (pad.on) {
      ctx.beginPath();
      ctx.arc(pad.position.x, pad.position.y, pad.radius * 0.36, 0, TAU);
      ctx.fill();
    } else {
      // A hollow chevron: "throw me".
      const r = pad.radius * 0.4;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(pad.position.x - r, pad.position.y + r * 0.5);
      ctx.lineTo(pad.position.x, pad.position.y - r * 0.6);
      ctx.lineTo(pad.position.x + r, pad.position.y + r * 0.5);
      ctx.stroke();
    }

    // A held switch shows how long is left, because the whole point of it is
    // that the route it opens is temporary.
    if (pad.holdTime !== undefined) {
      const remaining = pad.on && pad.offAt !== undefined ? pad.offAt - worldTime : 0;
      const fraction = clamp(remaining / pad.holdTime, 0, 1);
      ctx.globalAlpha = 1;
      ctx.lineWidth = 4;
      ctx.lineCap = 'butt';
      ctx.strokeStyle = 'rgba(255,255,255,0.14)';
      ctx.beginPath();
      ctx.arc(pad.position.x, pad.position.y, pad.radius * 1.45, 0, TAU);
      ctx.stroke();
      if (fraction > 0) {
        // Draining clockwise from the top, so it reads as a clock, not a meter.
        ctx.strokeStyle = fraction < 0.3 ? palette.aimStrong : palette.holeRim;
        ctx.beginPath();
        ctx.arc(
          pad.position.x,
          pad.position.y,
          pad.radius * 1.45,
          -Math.PI / 2,
          -Math.PI / 2 + TAU * fraction,
        );
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  private drawBody(
    ctx: CanvasRenderingContext2D,
    body: Body,
    shape: Shape,
    palette: Palette,
    world: World,
    time: number,
  ): void {
    const style = styleForBody(body.style, body.material);
    const colors = colorsForBody(body.style, body.material);
    const center = shapeCenter(shape);
    const radius = Math.max(shapeRadius(shape), 1);

    ctx.save();

    // A body far larger than the screen (a planet used as ground) shows only a
    // sliver of its surface. Shading it with a body-sized gradient costs a full
    // screen of gradient rasterisation to produce a flat-looking result, so
    // oversized bodies get flat fills and no glow.
    const screenRadius = radius * this.camera.zoom;
    const oversized = screenRadius > Math.max(this.width, this.height);

    if (colors.glow && !oversized) {
      // A star breathes; nothing else does.
      const breathe = style === 'sun' ? 1 + Math.sin(time * 1.7) * 0.05 : 1;
      const glowRadius = radius * (style === 'sun' ? 2.6 : 1.7) * breathe;
      const grad = ctx.createRadialGradient(center.x, center.y, radius * 0.7, center.x, center.y, glowRadius);
      grad.addColorStop(0, colors.glow);
      grad.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(center.x, center.y, glowRadius, 0, TAU);
      ctx.fill();
    }

    // Ordinary worlds are painted once, properly, and blitted.
    let painted = false;
    if (shape.kind === 'circle' && !oversized && PAINTED_STYLES.has(style)) {
      const sprite = this.painter.sprite(
        body.id,
        style,
        colors,
        radius,
        this.camera.zoom * this.pixelRatio,
      );
      if (sprite) {
        ctx.drawImage(
          sprite.canvas,
          center.x - sprite.half,
          center.y - sprite.half,
          sprite.half * 2,
          sprite.half * 2,
        );
        painted = true;
      }
    }

    if (!painted) {
      if (style === 'sun' && shape.kind === 'circle' && !oversized) {
        this.drawSunSurface(ctx, center, radius, colors, time);
      } else if (shape.kind === 'circle' && !oversized) {
        const grad = ctx.createRadialGradient(
          center.x + LIGHT.x * radius * 0.4,
          center.y + LIGHT.y * radius * 0.4,
          radius * 0.1,
          center.x,
          center.y,
          radius,
        );
        grad.addColorStop(0, lighten(colors.fill, 0.15));
        grad.addColorStop(0.6, colors.fill);
        grad.addColorStop(1, colors.fillDark);
        ctx.fillStyle = grad;
        this.tracePath(ctx, shape);
        ctx.fill();
      } else if (shape.kind !== 'circle') {
        this.drawSlab(ctx, shape, colors);
      } else {
        ctx.fillStyle = colors.fill;
        this.tracePath(ctx, shape);
        ctx.fill();
      }
    }

    if (style === 'blackhole') {
      this.drawAccretion(ctx, center, radius, colors, time);
    } else if (
      !painted &&
      shape.kind === 'circle' &&
      radius > 24 &&
      ROCKY_STYLES.has(style)
    ) {
      const decor = this.decorFor(body, radius);
      ctx.save();
      this.tracePath(ctx, shape);
      ctx.clip();
      ctx.fillStyle = colors.fillDark;
      ctx.globalAlpha = 0.55;
      for (const crater of decor.craters) {
        ctx.beginPath();
        ctx.arc(center.x + crater.x, center.y + crater.y, crater.r, 0, TAU);
        ctx.fill();
      }
      ctx.restore();
    }

    if (body.hitsToBreak !== undefined) {
      const left = world.breakables[body.id] ?? body.hitsToBreak;
      const damage = 1 - left / body.hitsToBreak;
      if (damage > 0) {
        // Cracks widen as the block takes hits, so its remaining life is
        // readable without a number on screen.
        ctx.save();
        this.tracePath(ctx, shape);
        ctx.clip();
        ctx.strokeStyle = colors.rim;
        ctx.globalAlpha = 0.35 + damage * 0.5;
        ctx.lineWidth = 1 + damage * 2;
        const cracks = Math.max(2, Math.round(damage * 5));
        for (let i = 0; i < cracks; i++) {
          const a = (i / cracks) * TAU + 0.4;
          ctx.beginPath();
          ctx.moveTo(center.x, center.y);
          ctx.lineTo(center.x + Math.cos(a) * radius, center.y + Math.sin(a) * radius);
          ctx.stroke();
        }
        ctx.restore();
      }
    }

    if (style === 'bumper') {
      // Concentric rings read as "springy" without relying on hue.
      ctx.globalAlpha = 0.55;
      ctx.strokeStyle = colors.rim;
      ctx.lineWidth = Math.max(1, radius * 0.06);
      for (const scale of [0.66, 0.4]) {
        ctx.beginPath();
        ctx.arc(center.x, center.y, radius * scale, 0, TAU);
        ctx.stroke();
      }
      // Glossy, like a pinball bumper cap.
      ctx.globalAlpha = 0.5;
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.ellipse(
        center.x + LIGHT.x * radius * 0.42,
        center.y + LIGHT.y * radius * 0.42,
        radius * 0.28,
        radius * 0.13,
        Math.atan2(LIGHT.y, LIGHT.x) + Math.PI / 2,
        0,
        TAU,
      );
      ctx.fill();
    }

    // The edge the ball actually collides with. Painted worlds carry their own
    // rim light, so only a whisper of outline to keep the night side legible.
    ctx.globalAlpha = painted ? 0.28 : style === 'sun' ? 0.6 : 1;
    ctx.strokeStyle = colors.rim;
    ctx.lineWidth = Math.max(1.2, Math.min(radius * 0.035, 4));
    this.tracePath(ctx, shape);
    ctx.stroke();
    ctx.globalAlpha = 1;

    // A one-way membrane wears arrows pointing the way you may cross it. Without
    // them it is a wall that inexplicably lets the ball through half the time.
    if (body.oneWay) this.drawOneWayArrows(ctx, shape, body.oneWay, colors.rim);

    // Anything that destroys the ball wears a serrated corona. It is a shape
    // cue, not a colour cue, so it survives a colourblind player and a
    // washed-out screen alike.
    if (MATERIALS[body.material].deadly) {
      // Clear the black hole's accretion rings, which reach to 1.6x.
      const inset = style === 'blackhole' ? 1.75 : 1.06;
      this.drawDangerCorona(ctx, center, radius, palette, inset);
    }

    ctx.restore();
  }

  /**
   * A star: white-hot core, limb-darkened edge, and slow churning granules.
   * Drawn live because it moves — a sun that sits still looks like a sticker.
   */
  private drawSunSurface(
    ctx: CanvasRenderingContext2D,
    c: Vec2,
    r: number,
    colors: BodyColors,
    time: number,
  ): void {
    const core = ctx.createRadialGradient(c.x, c.y, 0, c.x, c.y, r);
    core.addColorStop(0, '#fffbe8');
    core.addColorStop(0.35, lighten(colors.fill, 0.35));
    core.addColorStop(0.8, colors.fill);
    core.addColorStop(1, colors.fillDark);
    ctx.fillStyle = core;
    ctx.beginPath();
    ctx.arc(c.x, c.y, r, 0, TAU);
    ctx.fill();

    if (this.camera.zoom * r < 10) return;
    ctx.save();
    ctx.beginPath();
    ctx.arc(c.x, c.y, r, 0, TAU);
    ctx.clip();
    ctx.globalCompositeOperation = 'lighter';
    ctx.fillStyle = 'rgba(255,220,140,0.16)';
    for (let i = 0; i < 7; i++) {
      const a = i * 2.39996 + time * (0.18 + (i % 3) * 0.07);
      const d = r * (0.25 + ((i * 0.37) % 0.5));
      const s = r * (0.16 + 0.05 * Math.sin(time * 2.1 + i));
      ctx.beginPath();
      ctx.arc(c.x + Math.cos(a) * d, c.y + Math.sin(a) * d, s, 0, TAU);
      ctx.fill();
    }
    ctx.restore();
  }

  /**
   * A black hole: a swirling accretion disc and a thin photon ring. The motion
   * is the warning — it reads as something that takes, not something to land on.
   */
  private drawAccretion(
    ctx: CanvasRenderingContext2D,
    c: Vec2,
    r: number,
    colors: BodyColors,
    time: number,
  ): void {
    ctx.save();
    ctx.lineCap = 'round';
    const bands = [
      { scale: 1.22, width: 0.14, alpha: 0.85, speed: 1.6, dash: 0.9 },
      { scale: 1.4, width: 0.09, alpha: 0.5, speed: 1.0, dash: 1.4 },
      { scale: 1.58, width: 0.06, alpha: 0.28, speed: 0.6, dash: 2.0 },
    ];
    for (const band of bands) {
      const rr = r * band.scale;
      const dash = rr * band.dash;
      ctx.globalAlpha = band.alpha;
      ctx.strokeStyle = colors.rim;
      ctx.lineWidth = Math.max(1.2, r * band.width);
      ctx.setLineDash([dash, dash * 0.6]);
      ctx.lineDashOffset = -time * band.speed * rr;
      ctx.beginPath();
      ctx.arc(c.x, c.y, rr, 0, TAU);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    // The photon ring: the last light before the edge.
    ctx.globalAlpha = 0.9;
    ctx.strokeStyle = '#efe4ff';
    ctx.lineWidth = Math.max(1, r * 0.05);
    ctx.beginPath();
    ctx.arc(c.x, c.y, r * 1.04, 0, TAU);
    ctx.stroke();
    ctx.restore();
  }

  /**
   * Walls, beams and slabs: lit across their thickness so they read as solid
   * bevelled metal rather than flat strips.
   */
  private drawSlab(ctx: CanvasRenderingContext2D, shape: Shape, colors: BodyColors): void {
    let from: Vec2;
    let to: Vec2;
    if (shape.kind === 'capsule') {
      const axis = V.normalize(V.sub(shape.b, shape.a));
      let n = V.perp(axis);
      // Face the bright edge toward the light.
      if (n.x * LIGHT.x + n.y * LIGHT.y < 0) n = V.neg(n);
      const mid = V.lerpVec(shape.a, shape.b, 0.5);
      from = V.addScaled(mid, n, shape.radius);
      to = V.addScaled(mid, n, -shape.radius);
    } else {
      const center = shapeCenter(shape);
      const r = shapeRadius(shape);
      from = V.addScaled(center, LIGHT, r);
      to = V.addScaled(center, LIGHT, -r);
    }
    const grad = ctx.createLinearGradient(from.x, from.y, to.x, to.y);
    grad.addColorStop(0, lighten(colors.fill, 0.3));
    grad.addColorStop(0.3, colors.fill);
    grad.addColorStop(1, colors.fillDark);
    ctx.fillStyle = grad;
    this.tracePath(ctx, shape);
    ctx.fill();

    // A bright line just inside the lit edge: the bevel.
    if (shape.kind === 'capsule' && shape.radius * this.camera.zoom > 3) {
      const axis = V.normalize(V.sub(shape.b, shape.a));
      let n = V.perp(axis);
      if (n.x * LIGHT.x + n.y * LIGHT.y < 0) n = V.neg(n);
      const inset = shape.radius * 0.55;
      ctx.save();
      ctx.globalAlpha = 0.35;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = Math.max(0.8, shape.radius * 0.14);
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(shape.a.x + n.x * inset, shape.a.y + n.y * inset);
      ctx.lineTo(shape.b.x + n.x * inset, shape.b.y + n.y * inset);
      ctx.stroke();
      ctx.restore();
    }
  }

  /** Chevrons along a membrane, pointing the direction the ball may cross. */
  private drawOneWayArrows(
    ctx: CanvasRenderingContext2D,
    shape: Shape,
    through: Vec2,
    color: string,
  ): void {
    if (shape.kind !== 'capsule') return;
    const dir = V.normalize(through);
    const along = V.normalize(V.sub(shape.b, shape.a));
    const length = V.distance(shape.a, shape.b);
    const spacing = 46;
    const count = Math.max(1, Math.floor(length / spacing));
    const w = 7;

    ctx.save();
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.9;
    ctx.lineWidth = 2.2;
    ctx.lineCap = 'round';
    ctx.beginPath();
    for (let i = 0; i < count; i++) {
      const t = ((i + 0.5) / count) * length;
      const cx = shape.a.x + along.x * t;
      const cy = shape.a.y + along.y * t;
      // A chevron: two strokes meeting at a tip that points the allowed way.
      const tip = { x: cx + dir.x * w, y: cy + dir.y * w };
      for (const side of [1, -1]) {
        const base = {
          x: cx - dir.x * w * 0.4 + along.x * w * side,
          y: cy - dir.y * w * 0.4 + along.y * w * side,
        };
        ctx.moveTo(base.x, base.y);
        ctx.lineTo(tip.x, tip.y);
      }
    }
    ctx.stroke();
    ctx.restore();
  }

  /**
   * The line the hint suggests. Gold and marching, so it is obviously an
   * annotation rather than the aim guide the player is driving themselves.
   */
  private drawHintPath(
    ctx: CanvasRenderingContext2D,
    path: Vec2[],
    time: number,
    palette: Palette,
  ): void {
    if (path.length < 2) return;
    ctx.save();
    ctx.strokeStyle = palette.collectible;
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 2.4 / this.camera.zoom;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.setLineDash([11 / this.camera.zoom, 8 / this.camera.zoom]);
    ctx.lineDashOffset = -time * 44;
    ctx.beginPath();
    ctx.moveTo(path[0]!.x, path[0]!.y);
    for (let i = 1; i < path.length; i++) ctx.lineTo(path[i]!.x, path[i]!.y);
    ctx.stroke();

    // A dot at the far end, so the line has a destination rather than just
    // trailing off the screen.
    const end = path[path.length - 1]!;
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.fillStyle = palette.collectible;
    ctx.beginPath();
    ctx.arc(end.x, end.y, 4.5 / this.camera.zoom, 0, TAU);
    ctx.fill();
    ctx.restore();
  }

  private drawDangerCorona(
    ctx: CanvasRenderingContext2D,
    center: Vec2,
    radius: number,
    palette: Palette,
    innerScale = 1.06,
  ): void {
    const spikes = Math.max(8, Math.round(radius / 5));
    const inner = radius * innerScale;
    const outer = inner + radius * 0.24;
    ctx.save();
    ctx.strokeStyle = palette.danger;
    ctx.lineWidth = Math.max(1.4, radius * 0.05);
    ctx.lineCap = 'round';
    ctx.globalAlpha = 0.85;
    ctx.beginPath();
    for (let i = 0; i < spikes; i++) {
      const a = (i / spikes) * TAU;
      ctx.moveTo(center.x + Math.cos(a) * inner, center.y + Math.sin(a) * inner);
      ctx.lineTo(center.x + Math.cos(a) * outer, center.y + Math.sin(a) * outer);
    }
    ctx.stroke();
    ctx.restore();
  }

  /* ---------------------------------------------------------- hole, ball */

  private drawHole(
    ctx: CanvasRenderingContext2D,
    world: World,
    time: number,
    palette: Palette,
  ): void {
    const radius = world.hole.radius;
    // Never `hole.position`: on a moving hole that is only where it started.
    const position = holePositionAt(world);
    ctx.save();

    // A cup that will not accept the ball yet has to say so before the player
    // wastes a stroke finding out. Amber rather than green, plus a bar across
    // the mouth: the state is legible without relying on colour.
    const owed = Math.max(
      0,
      (world.hole.requiresStars ?? 0) - world.collectibles.filter((c) => c.collected).length,
    );
    const locked = owed > 0;
    const rim = locked ? palette.collectible : palette.holeRim;
    const glow = locked ? '255, 210, 87' : '102, 230, 184';

    // A cup that moves without a visible path is not a puzzle, it is a guess.
    if (world.hole.motion) this.drawHolePath(ctx, world, palette);
    // Likewise a cup that refuses every heading but one.
    if (world.hole.approach) this.drawApproachCone(ctx, position, radius, world.hole.approach, rim);

    const pulse = 1 + Math.sin(time * 2.4) * 0.06;
    const grad = ctx.createRadialGradient(
      position.x,
      position.y,
      radius * 0.2,
      position.x,
      position.y,
      radius * 3.2,
    );
    grad.addColorStop(0, `rgba(${glow}, 0.35)`);
    grad.addColorStop(1, `rgba(${glow}, 0)`);
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius * 3.2, 0, TAU);
    ctx.fill();

    ctx.fillStyle = palette.hole;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius, 0, TAU);
    ctx.fill();

    ctx.strokeStyle = rim;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius, 0, TAU);
    ctx.stroke();

    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius * 1.6 * pulse, 0, TAU);
    ctx.stroke();

    if (locked) {
      // Bars across the mouth, and one pip per star still owed.
      ctx.globalAlpha = 0.9;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (const t of [-0.34, 0.34]) {
        const y = position.y + radius * t;
        const half = Math.sqrt(Math.max(0, radius * radius - (radius * t) ** 2)) * 0.94;
        ctx.moveTo(position.x - half, y);
        ctx.lineTo(position.x + half, y);
      }
      ctx.stroke();

      ctx.fillStyle = rim;
      ctx.globalAlpha = 1;
      const spread = radius * 0.5;
      for (let i = 0; i < owed; i++) {
        const x = position.x + (i - (owed - 1) / 2) * spread;
        ctx.beginPath();
        ctx.arc(x, position.y - radius * 2.1, radius * 0.16, 0, TAU);
        ctx.fill();
      }
    }

    // Flag, so the target reads instantly even against a busy background.
    ctx.globalAlpha = 1;
    const flagHeight = radius * 3.4;
    ctx.strokeStyle = palette.textDim;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(position.x, position.y);
    ctx.lineTo(position.x, position.y - flagHeight);
    ctx.stroke();

    const wave = Math.sin(time * 3) * radius * 0.14;
    ctx.fillStyle = rim;
    ctx.beginPath();
    ctx.moveTo(position.x, position.y - flagHeight);
    ctx.quadraticCurveTo(
      position.x + radius * 1.1,
      position.y - flagHeight + radius * 0.35 + wave,
      position.x,
      position.y - flagHeight + radius * 0.9,
    );
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  /**
   * The mouth of a directional cup: a wedge on the side the ball has to come
   * from, with an arrow through it showing the way in.
   */
  private drawApproachCone(
    ctx: CanvasRenderingContext2D,
    position: Vec2,
    radius: number,
    approach: { direction: Vec2; tolerance: number },
    color: string,
  ): void {
    const heading = Math.atan2(approach.direction.y, approach.direction.x);
    // The mouth faces back along the heading: that is the side you arrive from.
    const mouth = heading + Math.PI;
    const inner = radius * 1.25;
    const outer = radius * 2.9;

    ctx.save();
    ctx.translate(position.x, position.y);

    ctx.fillStyle = color;
    ctx.globalAlpha = 0.16;
    ctx.beginPath();
    ctx.arc(0, 0, outer, mouth - approach.tolerance, mouth + approach.tolerance);
    ctx.arc(0, 0, inner, mouth + approach.tolerance, mouth - approach.tolerance, true);
    ctx.closePath();
    ctx.fill();

    ctx.globalAlpha = 0.75;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8;
    ctx.stroke();

    // An arrow along the heading, so the wedge reads as "in" not "out".
    ctx.rotate(heading);
    ctx.globalAlpha = 0.95;
    ctx.lineWidth = 2.4;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(-outer * 0.86, 0);
    ctx.lineTo(-inner * 0.9, 0);
    ctx.moveTo(-inner * 1.34, -6);
    ctx.lineTo(-inner * 0.9, 0);
    ctx.lineTo(-inner * 1.34, 6);
    ctx.stroke();
    ctx.restore();
  }

  /** The track a moving cup runs along, sampled straight from its motion. */
  private drawHolePath(ctx: CanvasRenderingContext2D, world: World, palette: Palette): void {
    const motion = world.hole.motion;
    if (!motion) return;
    const period =
      motion.kind === 'orbit'
        ? Math.abs(motion.speed) > 1e-6
          ? TAU / Math.abs(motion.speed)
          : 0
        : motion.kind === 'oscillate'
          ? motion.period
          : 0;
    if (period <= 0) return;

    ctx.save();
    ctx.strokeStyle = palette.holeRim;
    ctx.globalAlpha = 0.25;
    ctx.lineWidth = 1.6 / this.camera.zoom;
    ctx.setLineDash([7 / this.camera.zoom, 7 / this.camera.zoom]);
    ctx.beginPath();
    const samples = 64;
    for (let i = 0; i <= samples; i++) {
      const p = holePositionAt(world, (i / samples) * period);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  private drawCollectibles(ctx: CanvasRenderingContext2D, scene: Scene, palette: Palette): void {
    for (const item of scene.world.collectibles) {
      if (item.collected) continue;
      const bob = Math.sin(scene.time * 2.2 + item.position.x * 0.02) * 3;
      const p = { x: item.position.x, y: item.position.y + bob };
      const spin = scene.time * 1.4;

      ctx.save();
      const grad = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, item.radius * 2.6);
      grad.addColorStop(0, palette.collectibleGlow);
      grad.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(p.x, p.y, item.radius * 2.6, 0, TAU);
      ctx.fill();

      ctx.translate(p.x, p.y);
      ctx.rotate(spin);
      ctx.fillStyle = palette.collectible;
      ctx.beginPath();
      this.traceStar(ctx, 0, 0, item.radius);
      ctx.fill();
      ctx.restore();
    }
  }

  /** A five-pointed star, so a toll pip is the same shape as the thing it wants. */
  private traceStar(ctx: CanvasRenderingContext2D, cx: number, cy: number, radius: number): void {
    const spikes = 5;
    const inner = radius * 0.44;
    ctx.beginPath();
    for (let i = 0; i < spikes * 2; i++) {
      const r = i % 2 === 0 ? radius : inner;
      const a = (i / (spikes * 2)) * TAU - Math.PI / 2;
      const x = cx + Math.cos(a) * r;
      const y = cy + Math.sin(a) * r;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.closePath();
  }

  private drawTrail(
    ctx: CanvasRenderingContext2D,
    trail: Vec2[],
    ballRadius: number,
    color: string,
  ): void {
    if (trail.length < 2) return;
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    // Additive, so overlapping loops brighten like a long exposure.
    ctx.globalCompositeOperation = 'lighter';
    ctx.strokeStyle = color;
    for (let i = 1; i < trail.length; i++) {
      const t = i / trail.length;
      const a = trail[i - 1]!;
      const b = trail[i]!;
      // A wide soft glow...
      ctx.globalAlpha = t * t * 0.22;
      ctx.lineWidth = ballRadius * 2.6 * t;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      // ...around a hot thin core.
      ctx.globalAlpha = t * 0.55;
      ctx.lineWidth = ballRadius * 0.7 * t;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  private drawBall(
    ctx: CanvasRenderingContext2D,
    position: Vec2,
    radius: number,
    time: number,
    skin: BallSkin,
  ): void {
    const { x, y } = position;
    ctx.save();
    const glow = ctx.createRadialGradient(x, y, radius * 0.5, x, y, radius * 3.6);
    glow.addColorStop(0, skin.glow);
    glow.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(x, y, radius * 3.6, 0, TAU);
    ctx.fill();

    // Lit from the same star as every planet, so the ball sits in the scene.
    const grad = ctx.createRadialGradient(
      x + LIGHT.x * radius * 0.4,
      y + LIGHT.y * radius * 0.4,
      radius * 0.05,
      x,
      y,
      radius * 1.05,
    );
    grad.addColorStop(0, skin.highlight);
    grad.addColorStop(0.6, skin.body);
    grad.addColorStop(1, 'rgba(10,14,30,1)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, TAU);
    ctx.fill();

    // A hard specular pin: the thing that makes a ball look polished.
    ctx.fillStyle = 'rgba(255,255,255,0.9)';
    ctx.beginPath();
    ctx.arc(x + LIGHT.x * radius * 0.45, y + LIGHT.y * radius * 0.45, radius * 0.2, 0, TAU);
    ctx.fill();

    ctx.globalAlpha = 0.5 + Math.sin(time * 4) * 0.15;
    ctx.strokeStyle = skin.glow;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.arc(x, y, radius + 1.8, 0, TAU);
    ctx.stroke();
    ctx.restore();
  }

  /** Slow ripples off a ball that is waiting to be struck. */
  private drawBeacon(
    ctx: CanvasRenderingContext2D,
    position: Vec2,
    radius: number,
    time: number,
    skin: BallSkin,
  ): void {
    ctx.save();
    ctx.strokeStyle = skin.highlight;
    ctx.lineWidth = 1.5 / Math.max(this.camera.zoom, 0.5);
    for (let i = 0; i < 2; i++) {
      const t = (time / 1.8 + i * 0.5) % 1;
      ctx.globalAlpha = (1 - t) * 0.45;
      ctx.beginPath();
      ctx.arc(position.x, position.y, radius * (1.6 + t * 3.2), 0, TAU);
      ctx.stroke();
    }
    ctx.restore();
  }

  /**
   * The player's best run, drawn as a hollow outline. Deliberately unfilled and
   * dim: it has to be legible without ever competing with the live ball for
   * attention.
   */
  private drawGhost(
    ctx: CanvasRenderingContext2D,
    ghost: { position: Vec2; trail: Vec2[] },
    radius: number,
    palette: Palette,
  ): void {
    ctx.save();
    ctx.globalAlpha = 0.32;
    ctx.strokeStyle = palette.textDim;
    ctx.lineCap = 'round';

    for (let i = 1; i < ghost.trail.length; i++) {
      const t = i / ghost.trail.length;
      ctx.globalAlpha = t * 0.22;
      ctx.lineWidth = radius * 0.8 * t;
      ctx.beginPath();
      ctx.moveTo(ghost.trail[i - 1]!.x, ghost.trail[i - 1]!.y);
      ctx.lineTo(ghost.trail[i]!.x, ghost.trail[i]!.y);
      ctx.stroke();
    }

    // Dashed, so at a glance it reads as a replay rather than a second ball.
    ctx.globalAlpha = 0.72;
    ctx.lineWidth = 1.8;
    ctx.setLineDash([3, 2.5]);
    ctx.beginPath();
    ctx.arc(ghost.position.x, ghost.position.y, radius + 1, 0, TAU);
    ctx.stroke();
    ctx.restore();
  }

  /* ----------------------------------------------------------------- aim */

  private drawAim(
    ctx: CanvasRenderingContext2D,
    aim: AimState,
    ballRadius: number,
    time: number,
    palette: Palette,
  ): void {
    const color = powerColor(aim.power, palette);
    const unit = 1 / Math.max(this.camera.zoom, 0.35);
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // Pull-back: a tapering band behind the ball, like a stretched sling.
    const pull = 20 + aim.power * 70;
    const back = V.addScaled(aim.origin, aim.direction, -pull);
    const side = V.perp(aim.direction);
    const width = ballRadius * (0.9 + aim.power * 0.5);
    const grad = ctx.createLinearGradient(aim.origin.x, aim.origin.y, back.x, back.y);
    grad.addColorStop(0, withAlpha(color, 0.5));
    grad.addColorStop(1, withAlpha(color, 0));
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(aim.origin.x + side.x * width, aim.origin.y + side.y * width);
    ctx.lineTo(back.x, back.y);
    ctx.lineTo(aim.origin.x - side.x * width, aim.origin.y - side.y * width);
    ctx.closePath();
    ctx.fill();

    // Predicted path: a soft glow along the line, with travelling beads on
    // top so the direction of travel is legible at a glance.
    if (aim.preview.length > 1) {
      const fadeLength = 13 * 60;
      ctx.strokeStyle = color;
      let travelled = 0;
      ctx.lineWidth = 5 * unit;
      for (let i = 1; i < aim.preview.length; i++) {
        const a = aim.preview[i - 1]!;
        const b = aim.preview[i]!;
        travelled += V.distance(a, b);
        ctx.globalAlpha = 0.14 * clamp(1 - travelled / fadeLength, 0.15, 1);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }

      const drift = (time * 2) % 1;
      ctx.fillStyle = color;
      let accumulated = 0;
      const spacing = 13;
      for (let i = 1; i < aim.preview.length; i++) {
        const a = aim.preview[i - 1]!;
        const b = aim.preview[i]!;
        const segment = V.distance(a, b);
        if (segment < 1e-6) continue;
        let along = spacing * drift - accumulated;
        while (along < segment) {
          if (along >= 0) {
            const t = along / segment;
            const x = a.x + (b.x - a.x) * t;
            const y = a.y + (b.y - a.y) * t;
            const fade = 1 - (accumulated + along) / fadeLength;
            ctx.globalAlpha = clamp(fade, 0.12, 0.95);
            ctx.beginPath();
            ctx.arc(x, y, 2.6 * clamp(fade + 0.3, 0.6, 1.1), 0, TAU);
            ctx.fill();
          }
          along += spacing;
        }
        accumulated += segment;
      }

      // Mark where the preview ends, and what happens there.
      const end = aim.preview[aim.preview.length - 1]!;
      ctx.globalAlpha = 0.9;
      if (aim.previewOutcome === 'sink') {
        ctx.strokeStyle = palette.holeRim;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.arc(end.x, end.y, 14 + Math.sin(time * 8) * 2, 0, TAU);
        ctx.stroke();
      } else if (aim.previewOutcome === 'death') {
        ctx.strokeStyle = palette.danger;
        ctx.lineWidth = 2.5;
        const s = 8;
        ctx.beginPath();
        ctx.moveTo(end.x - s, end.y - s);
        ctx.lineTo(end.x + s, end.y + s);
        ctx.moveTo(end.x + s, end.y - s);
        ctx.lineTo(end.x - s, end.y + s);
        ctx.stroke();
      }
    }

    // Power ring around the ball, on a faint track so a low-power shot still
    // shows how much room is left.
    const ringRadius = ballRadius + 7;
    ctx.globalAlpha = 0.18;
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.arc(aim.origin.x, aim.origin.y, ringRadius, 0, TAU);
    ctx.stroke();
    ctx.globalAlpha = 0.95;
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.arc(
      aim.origin.x,
      aim.origin.y,
      ringRadius,
      -Math.PI / 2,
      -Math.PI / 2 + TAU * aim.power,
    );
    ctx.stroke();

    // The number, behind the ball where the finger is not.
    const labelAt = V.addScaled(aim.origin, aim.direction, -(pull + 16 * unit));
    ctx.globalAlpha = 0.92;
    ctx.font = `700 ${12 * unit}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.lineWidth = 3 * unit;
    ctx.strokeStyle = 'rgba(4,6,16,0.8)';
    const label = `${Math.round(aim.power * 100)}%`;
    ctx.strokeText(label, labelAt.x, labelAt.y);
    ctx.fillStyle = color;
    ctx.fillText(label, labelAt.x, labelAt.y);

    ctx.restore();
  }

  /**
   * A brief full-screen wash of colour, for the moments that deserve one.
   * Timed on the scene clock, so a paused game holds it rather than skipping.
   */
  flash(color: string, amount = 0.35): void {
    this.flashState = { color, amount, start: null };
  }

  private drawFlash(ctx: CanvasRenderingContext2D, time: number): void {
    const flash = this.flashState;
    if (!flash) return;
    if (flash.start === null || time < flash.start) flash.start = time;
    const alpha = flash.amount * Math.exp(-(time - flash.start) * 4.5);
    if (alpha < 0.01) {
      this.flashState = null;
      return;
    }
    ctx.save();
    ctx.globalCompositeOperation = 'lighter';
    ctx.globalAlpha = alpha;
    ctx.fillStyle = flash.color;
    ctx.fillRect(0, 0, this.width, this.height);
    ctx.restore();
  }

  /** Clears cached per-body decoration. Call when switching levels. */
  resetDecor(): void {
    this.decorCache.clear();
    this.painter.clear();
  }
}
