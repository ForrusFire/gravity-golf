import { TAU, clamp } from '../core/math';
import { Rng, hashSeed } from '../core/rng';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import {
  bodyShapeAt,
  shapeBounds,
  shapeCenter,
  shapeRadius,
  worldVertices,
  type Aabb,
} from '../physics/geometry';
import { MATERIALS, type Body, type Shape, type Zone } from '../physics/types';
import { gravityAt, type World } from '../physics/world';
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
}

const overlaps = (a: Aabb, b: Aabb): boolean =>
  a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;

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
      this.drawBody(ctx, body, shape, options.palette);
    }

    // After the bodies: a cup sunk into the ground must not be painted over by
    // the ground itself.
    this.drawHole(ctx, scene.world, scene.time, options.palette);

    this.drawCollectibles(ctx, scene, options.palette);
    this.particles.draw(ctx);

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
        ctx.strokeStyle = zone.scale < 1 ? 'rgba(120, 255, 210, 0.55)' : 'rgba(255, 160, 90, 0.55)';
        ctx.lineWidth = 2 / this.camera.zoom;
        ctx.setLineDash([6 / this.camera.zoom, 6 / this.camera.zoom]);
        ctx.stroke();
        ctx.globalAlpha = 0.09;
        ctx.fillStyle = zone.scale < 1 ? '#78ffd2' : '#ffa05a';
        ctx.fill();
        break;
      }
    }
    ctx.restore();
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

  private drawBody(
    ctx: CanvasRenderingContext2D,
    body: Body,
    shape: Shape,
    palette: Palette,
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

    const pulse = 1 + Math.sin(time * 2.4) * 0.06;
    const grad = ctx.createRadialGradient(
      position.x,
      position.y,
      radius * 0.2,
      position.x,
      position.y,
      radius * 3.2,
    );
    grad.addColorStop(0, 'rgba(102, 230, 184, 0.35)');
    grad.addColorStop(1, 'rgba(102, 230, 184, 0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius * 3.2, 0, TAU);
    ctx.fill();

    ctx.fillStyle = palette.hole;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius, 0, TAU);
    ctx.fill();

    ctx.strokeStyle = palette.holeRim;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius, 0, TAU);
    ctx.stroke();

    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(position.x, position.y, radius * 1.6 * pulse, 0, TAU);
    ctx.stroke();

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
    ctx.fillStyle = palette.holeRim;
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
