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
import { gravityAt, isBodyActive, type World } from '../physics/world';
import { Camera } from './camera';
import { ParticleSystem } from './particles';
import { colorsForBody, powerColor, type Palette } from './palette';
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
}

const overlaps = (a: Aabb, b: Aabb): boolean =>
  a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;

/** Boost rings. A hue nothing else in the game uses. */
const BOOST_COLOR = '#a6f259';

/** Only solid ground gets cratered — a star or a goo blob should not. */
const ROCKY_STYLES = new Set<string>(['planet', 'moon', 'asteroid', 'sand']);

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

    if (options.showGravityField) this.drawGravityField(ctx, scene.world, options.palette);
    this.drawBoundary(ctx, scene.world, options.palette);
    for (const zone of scene.world.zones) this.drawZone(ctx, zone, scene.time, options.palette);
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
        continue;
      }
      this.drawBody(ctx, body, shape, options.palette, scene.world);
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
      this.drawBall(ctx, scene.ballPosition, scene.ballRadius, scene.time, scene.skin);
    }

    if (scene.aim && options.aimAssist > 0) {
      this.drawAim(ctx, scene.aim, scene.ballRadius, scene.time, options.palette);
    }

    ctx.restore();

    // Screen space, after the world transform is popped.
    this.drawOffscreenMarker(ctx, scene, options.palette);
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
    const hole = scene.world.hole.position;
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
   * A coarse grid of short strokes aligned with the local gravity direction.
   * It makes the invisible field readable at a glance, which is the whole game.
   */
  private drawGravityField(ctx: CanvasRenderingContext2D, world: World, palette: Palette): void {
    const view = this.camera.visibleBounds(0);
    // Keep the on-screen spacing roughly constant as the player zooms.
    const spacing = 52 / this.camera.zoom;
    const startX = Math.floor(view.minX / spacing) * spacing;
    const startY = Math.floor(view.minY / spacing) * spacing;

    ctx.save();
    ctx.strokeStyle = palette.fieldLine;
    ctx.fillStyle = palette.fieldLine;
    ctx.lineCap = 'round';
    ctx.lineWidth = 1.5 / this.camera.zoom;

    const maxLength = spacing * 0.4;
    const headRadius = 1.7 / this.camera.zoom;

    for (let x = startX; x <= view.maxX + spacing; x += spacing) {
      for (let y = startY; y <= view.maxY + spacing; y += spacing) {
        const p = { x, y };
        const g = gravityAt(world, p);
        const magnitude = Math.hypot(g.x, g.y);
        // Below this the field is not worth the visual noise.
        if (magnitude < 6) continue;

        // Log scaling: real fields span orders of magnitude across one screen.
        const strength = clamp(Math.log10(magnitude) / 2.8, 0.1, 1);
        const len = maxLength * (0.35 + 0.65 * strength);
        const nx = (g.x / magnitude) * len;
        const ny = (g.y / magnitude) * len;

        // Tail fades in from behind, and a dot marks the pull direction, so a
        // still frame shows which way space is pulling rather than just where.
        ctx.globalAlpha = strength * 0.3;
        ctx.beginPath();
        ctx.moveTo(x - nx, y - ny);
        ctx.lineTo(x + nx * 0.55, y + ny * 0.55);
        ctx.stroke();

        ctx.globalAlpha = strength * 0.6;
        ctx.beginPath();
        ctx.arc(x + nx, y + ny, headRadius * (0.6 + strength * 0.7), 0, TAU);
        ctx.fill();
      }
    }
    ctx.restore();
  }

  private drawBoundary(ctx: CanvasRenderingContext2D, world: World, palette: Palette): void {
    if (world.boundsMode === 'open') return;
    const b = world.bounds;
    ctx.save();
    ctx.lineWidth = 3 / this.camera.zoom;
    if (world.boundsMode === 'wall') {
      ctx.strokeStyle = palette.panelBorder;
      ctx.setLineDash([]);
    } else {
      ctx.strokeStyle = palette.danger;
      ctx.globalAlpha = 0.45;
      ctx.setLineDash([14 / this.camera.zoom, 12 / this.camera.zoom]);
    }
    ctx.strokeRect(b.minX, b.minY, b.maxX - b.minX, b.maxY - b.minY);
    ctx.restore();
  }

  /* --------------------------------------------------------------- zones */

  private drawZone(
    ctx: CanvasRenderingContext2D,
    zone: Zone,
    time: number,
    palette: Palette,
  ): void {
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
    world?: World,
  ): void {
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
      const glowRadius = radius * (body.style === 'sun' ? 2.6 : 1.7);
      const grad = ctx.createRadialGradient(center.x, center.y, radius * 0.7, center.x, center.y, glowRadius);
      grad.addColorStop(0, colors.glow);
      grad.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(center.x, center.y, glowRadius, 0, TAU);
      ctx.fill();
    }

    // Body fill: lit from the upper-left so shapes read as solid.
    if (shape.kind === 'circle' && !oversized) {
      const grad = ctx.createRadialGradient(
        center.x - radius * 0.35,
        center.y - radius * 0.35,
        radius * 0.1,
        center.x,
        center.y,
        radius,
      );
      grad.addColorStop(0, colors.fill);
      grad.addColorStop(1, colors.fillDark);
      ctx.fillStyle = grad;
    } else {
      ctx.fillStyle = colors.fill;
    }

    this.tracePath(ctx, shape);
    ctx.fill();

    if (body.style === 'blackhole') {
      // Accretion ring instead of surface detail.
      ctx.strokeStyle = colors.rim;
      ctx.lineWidth = Math.max(1.5, radius * 0.1);
      ctx.globalAlpha = 0.8;
      ctx.beginPath();
      ctx.arc(center.x, center.y, radius * 1.25, 0, TAU);
      ctx.stroke();
      ctx.globalAlpha = 0.35;
      ctx.beginPath();
      ctx.arc(center.x, center.y, radius * 1.6, 0, TAU);
      ctx.stroke();
    } else if (shape.kind === 'circle' && radius > 24 && ROCKY_STYLES.has(body.style ?? 'planet')) {
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

    if (body.hitsToBreak !== undefined && world) {
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

    if (body.style === 'bumper') {
      // Concentric rings read as "springy" without relying on hue.
      ctx.globalAlpha = 0.55;
      ctx.strokeStyle = colors.rim;
      ctx.lineWidth = Math.max(1, radius * 0.06);
      for (const scale of [0.66, 0.4]) {
        ctx.beginPath();
        ctx.arc(center.x, center.y, radius * scale, 0, TAU);
        ctx.stroke();
      }
    }

    ctx.globalAlpha = 1;
    ctx.strokeStyle = colors.rim;
    ctx.lineWidth = Math.max(1.2, radius * 0.035);
    this.tracePath(ctx, shape);
    ctx.stroke();

    // A one-way membrane wears arrows pointing the way you may cross it. Without
    // them it is a wall that inexplicably lets the ball through half the time.
    if (body.oneWay) this.drawOneWayArrows(ctx, shape, body.oneWay, colors.rim);

    // Anything that destroys the ball wears a serrated corona. It is a shape
    // cue, not a colour cue, so it survives a colourblind player and a
    // washed-out screen alike.
    if (MATERIALS[body.material].deadly) {
      // Clear the black hole's accretion rings, which reach to 1.6x.
      const inset = body.style === 'blackhole' ? 1.75 : 1.06;
      this.drawDangerCorona(ctx, center, radius, palette, inset);
    }

    ctx.restore();
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
    const { position, radius } = world.hole;
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
      const spikes = 5;
      const outer = item.radius;
      const inner = item.radius * 0.44;
      for (let i = 0; i < spikes * 2; i++) {
        const r = i % 2 === 0 ? outer : inner;
        const a = (i / (spikes * 2)) * TAU - Math.PI / 2;
        const x = Math.cos(a) * r;
        const y = Math.sin(a) * r;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }

  private drawTrail(
    ctx: CanvasRenderingContext2D,
    trail: Vec2[],
    ballRadius: number,
    color: string,
  ): void {
    if (trail.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    for (let i = 1; i < trail.length; i++) {
      const t = i / trail.length;
      ctx.globalAlpha = t * 0.5;
      ctx.lineWidth = ballRadius * 1.4 * t;
      ctx.beginPath();
      ctx.moveTo(trail[i - 1]!.x, trail[i - 1]!.y);
      ctx.lineTo(trail[i]!.x, trail[i]!.y);
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
    ctx.save();
    const glow = ctx.createRadialGradient(
      position.x,
      position.y,
      radius * 0.5,
      position.x,
      position.y,
      radius * 3.4,
    );
    glow.addColorStop(0, skin.glow);
    glow.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius * 3.4, 0, TAU);
    ctx.fill();

    const grad = ctx.createRadialGradient(
      position.x - radius * 0.35,
      position.y - radius * 0.4,
      radius * 0.1,
      position.x,
      position.y,
      radius,
    );
    grad.addColorStop(0, skin.highlight);
    grad.addColorStop(1, skin.body);
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius, 0, TAU);
    ctx.fill();

    ctx.globalAlpha = 0.55 + Math.sin(time * 4) * 0.12;
    ctx.strokeStyle = skin.glow;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius + 1.5, 0, TAU);
    ctx.stroke();
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
    ctx.save();

    // Pull-back indicator behind the ball, like a stretched sling.
    const back = V.addScaled(aim.origin, aim.direction, -(20 + aim.power * 70));
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(aim.origin.x, aim.origin.y);
    ctx.lineTo(back.x, back.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // Predicted path as travelling dots — direction is legible at a glance.
    if (aim.preview.length > 1) {
      const drift = (time * 2) % 1;
      ctx.globalAlpha = 0.95;
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
            const fade = 1 - (accumulated + along) / (spacing * 60);
            ctx.globalAlpha = clamp(fade, 0.12, 0.95);
            ctx.beginPath();
            ctx.arc(x, y, 2.4, 0, TAU);
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

    // Power ring around the ball.
    ctx.globalAlpha = 0.9;
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(
      aim.origin.x,
      aim.origin.y,
      ballRadius + 6,
      -Math.PI / 2,
      -Math.PI / 2 + TAU * aim.power,
    );
    ctx.stroke();

    ctx.restore();
  }

  /** Clears cached per-body decoration. Call when switching levels. */
  resetDecor(): void {
    this.decorCache.clear();
  }
}
