import { TAU, clamp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import {
  CUSTOM_BOUNDS,
  CUSTOM_LIMITS,
  customToLevel,
  emptyCustomHole,
  sanitizeCustomHole,
  type CustomBodyKind,
  type CustomHole,
  type CustomItem,
} from '../game/custom';
import { compileLevel, validateLevel } from '../game/level';
import type { World } from '../physics/world';
import type { Camera } from '../render/camera';
import { button, el } from './dom';

export type EditorTool =
  | CustomBodyKind
  | 'wall'
  | 'portal'
  | 'boost'
  | 'star'
  | 'tee'
  | 'cup'
  | 'move'
  | 'erase';

const TOOLS: Array<{ id: EditorTool; label: string; hint: string }> = [
  { id: 'move', label: 'Move', hint: 'Drag anything to move it' },
  { id: 'planet', label: 'Planet', hint: 'Click, or drag out to size' },
  { id: 'moon', label: 'Moon', hint: 'Click, or drag out to size' },
  { id: 'ice', label: 'Ice', hint: 'Slippery: keeps the ball rolling' },
  { id: 'sand', label: 'Sand', hint: 'Soaks up speed' },
  { id: 'goo', label: 'Goo', hint: 'Sticky: the ball stops dead' },
  { id: 'sun', label: 'Sun', hint: 'Deadly, and it pulls' },
  { id: 'blackhole', label: 'Black hole', hint: 'Deadly, and it pulls hard' },
  { id: 'bumper', label: 'Bumper', hint: 'Bounces back harder' },
  { id: 'rock', label: 'Rock', hint: 'Solid, no pull' },
  { id: 'wall', label: 'Wall', hint: 'Drag from one end to the other' },
  { id: 'portal', label: 'Portal', hint: 'Drag from the way in to the way out' },
  { id: 'boost', label: 'Boost', hint: 'Click, or drag the way it should fire' },
  { id: 'star', label: 'Star', hint: 'Up to three' },
  { id: 'tee', label: 'Tee', hint: 'Where the ball starts' },
  { id: 'cup', label: 'Cup', hint: 'Where it has to end' },
  { id: 'erase', label: 'Erase', hint: 'Click a thing to remove it' },
];

/** The size a body gets from a plain click. */
const DEFAULT_RADIUS: Record<CustomBodyKind, number> = {
  planet: 80,
  moon: 50,
  ice: 60,
  sand: 60,
  goo: 50,
  sun: 36,
  blackhole: 28,
  bumper: 38,
  rock: 26,
};

const DRAFT_KEY = 'gravity-golf/custom-draft';

export interface EditorHost {
  canvas: HTMLCanvasElement;
  camera: Camera;
  onTest(hole: CustomHole): void;
  onShare(hole: CustomHole): void;
  onExit(): void;
  /** A click for the UI, so the editor sounds like the rest of the game. */
  onClick?(): void;
}

type Grab =
  | { kind: 'item'; index: number; offset: Vec2 }
  | { kind: 'star'; index: number; offset: Vec2 }
  | { kind: 'tee' | 'cup'; offset: Vec2 };

/**
 * The hole editor: a toolbar, a status line, and the pointer handling on the
 * canvas. The hole it edits is the same small format a link carries, and it
 * is rebuilt into a real world on every change, so what is drawn is exactly
 * what will be played.
 */
export class HoleEditor {
  readonly root: HTMLElement;
  hole: CustomHole;
  world: World;
  tool: EditorTool = 'planet';

  private toolbar: HTMLElement;
  private statusLine: HTMLElement;
  private nameInput: HTMLInputElement;
  private parValue: HTMLElement;
  private downToggle: HTMLInputElement;
  private wallsToggle: HTMLInputElement;
  private turnToggle: HTMLInputElement;
  private pending: CustomItem | null = null;
  private pendingStart: Vec2 | null = null;
  private grab: Grab | null = null;
  private pointerId: number | null = null;
  /** Fingers on the canvas, in screen space, for pinch-zoom. */
  private touches = new Map<number, Vec2>();
  private pinch: { distance: number; center: Vec2 } | null = null;
  /** How much history there was when the current press began. */
  private historyAtDown = 0;
  private attached = false;
  /** Earlier versions of the hole, newest last, for undo. */
  private history: CustomHole[] = [];
  private undoButton!: HTMLButtonElement;
  private keepButton!: HTMLButtonElement;

  constructor(
    private readonly host: EditorHost,
    hole?: CustomHole,
    /**
     * Editing a hole that came from a link. Nothing is saved until it is kept,
     * so opening someone's link can never replace your own draft.
     */
    public shared = false,
  ) {
    this.hole = hole ? structuredClone(hole) : (loadDraft(DRAFT_KEY) ?? emptyCustomHole());
    this.world = compileLevel(customToLevel(this.hole));

    this.toolbar = el('div', { class: 'editor__tools', attrs: { role: 'toolbar', 'aria-label': 'Editor tools' } });
    this.statusLine = el('div', { class: 'editor__status', attrs: { role: 'status', 'aria-live': 'polite' } });
    this.nameInput = el('input', {
      class: 'editor__name',
      attrs: { type: 'text', maxlength: String(CUSTOM_LIMITS.nameLength), 'aria-label': 'Hole name' },
      on: {
        // One undo step for a whole rename, not one per letter.
        focus: () => this.checkpoint(),
        input: () => {
          this.hole.name = this.nameInput.value;
          this.changed(false);
        },
      },
    });
    this.parValue = el('span', { class: 'editor__par-value' });
    this.downToggle = el('input', {
      attrs: { type: 'checkbox', 'aria-label': 'Gravity pulls down everywhere' },
      on: {
        change: () => {
          this.checkpoint();
          this.hole.down = this.downToggle.checked;
          if (!this.hole.down) this.hole.turn = false;
          this.changed();
        },
      },
    });
    this.wallsToggle = el('input', {
      attrs: { type: 'checkbox', 'aria-label': 'Edges are walls' },
      on: {
        change: () => {
          this.checkpoint();
          this.hole.walls = this.wallsToggle.checked;
          this.changed();
        },
      },
    });

    this.turnToggle = el('input', {
      attrs: { type: 'checkbox', 'aria-label': 'Down turns while the ball waits' },
      on: {
        change: () => {
          this.checkpoint();
          this.hole.turn = this.turnToggle.checked;
          // Turning needs a down to turn.
          if (this.hole.turn) this.hole.down = true;
          this.changed();
        },
      },
    });

    const actions = el('div', { class: 'editor__actions' }, [
      this.nameInput,
      el('span', { class: 'editor__par' }, [
        button('−', () => this.setPar(this.hole.par - 1), { class: 'btn--icon', attrs: { 'aria-label': 'Lower par' } }),
        this.parValue,
        button('+', () => this.setPar(this.hole.par + 1), { class: 'btn--icon', attrs: { 'aria-label': 'Raise par' } }),
      ]),
      el('label', { class: 'editor__toggle' }, [this.downToggle, el('span', { text: 'Down' })]),
      el('label', { class: 'editor__toggle' }, [this.wallsToggle, el('span', { text: 'Walls' })]),
      el('label', { class: 'editor__toggle' }, [this.turnToggle, el('span', { text: 'Turn' })]),
      button('Fit', () => this.frame(), {
        class: 'btn--ghost',
        attrs: { title: 'Show the whole board (pinch or scroll to zoom)' },
      }),
      (this.undoButton = button('Undo', () => this.undo(), {
        class: 'btn--ghost',
        attrs: { title: 'Undo (Ctrl+Z)' },
      })),
      button('Clear', () => this.clear(), { class: 'btn--ghost' }),
      (this.keepButton = button('Keep as mine', () => this.keep(), {
        class: 'btn--ghost',
        attrs: { title: 'Make this hole your own draft' },
      })),
      // A copy: the check that follows takes a while, and the hole may be
      // edited meanwhile.
      button('Share', () => this.host.onShare(structuredClone(this.hole)), { class: 'btn--ghost' }),
      button('Test', () => this.test(), { class: 'btn--primary' }),
      button('Exit', () => this.host.onExit(), { class: 'btn--ghost' }),
    ]);

    this.root = el('div', { class: 'editor' }, [this.toolbar, this.statusLine, actions]);
    this.renderToolbar();
    this.syncFields();
    this.undoButton.disabled = true;
    this.keepButton.hidden = !shared;
    this.status(
      shared
        ? 'Editing a shared hole — changes are not saved unless you keep it'
        : TOOLS.find((t) => t.id === this.tool)!.hint,
    );
  }

  attach(container: HTMLElement): void {
    if (this.attached) return;
    this.attached = true;
    container.append(this.root);
    const c = this.host.canvas;
    c.addEventListener('pointerdown', this.onDown);
    c.addEventListener('pointermove', this.onMove);
    c.addEventListener('pointerup', this.onUp);
    c.addEventListener('pointercancel', this.onCancel);
    c.addEventListener('lostpointercapture', this.onCancel);
    c.addEventListener('wheel', this.onWheel, { passive: false });
    window.addEventListener('keydown', this.onKey);
    this.frame();
  }

  detach(): void {
    if (!this.attached) return;
    this.attached = false;
    this.root.remove();
    const c = this.host.canvas;
    c.removeEventListener('pointerdown', this.onDown);
    c.removeEventListener('pointermove', this.onMove);
    c.removeEventListener('pointerup', this.onUp);
    c.removeEventListener('pointercancel', this.onCancel);
    c.removeEventListener('lostpointercapture', this.onCancel);
    c.removeEventListener('wheel', this.onWheel);
    window.removeEventListener('keydown', this.onKey);
    // A finger still down as the editor closed never sends its pointerup here;
    // forget it, or the editor ignores every touch when it opens again.
    this.pointerId = null;
    this.pendingStart = null;
    this.pending = null;
    this.grab = null;
    this.touches.clear();
    this.pinch = null;
  }

  /** Frames the editing area, leaving room for the bars. */
  frame(): void {
    const camera = this.host.camera;
    const zoom = camera.fit(CUSTOM_BOUNDS, 70, 1.2);
    camera.limits = null;
    camera.snapTo({ x: 0, y: 10 / Math.max(zoom, 0.01) }, zoom);
  }

  status(message: string, tone: 'neutral' | 'bad' | 'good' = 'neutral'): void {
    this.statusLine.textContent = message;
    this.statusLine.dataset.tone = tone;
  }

  /** Problems that would stop the hole being played, worst first. */
  problems(): string[] {
    const issues = validateLevel(customToLevel(this.hole)).map((i) => i.message);
    return issues;
  }

  /** Draws the in-progress placement and handles, in world space. */
  drawOverlay(ctx: CanvasRenderingContext2D, zoom: number): void {
    const unit = 1 / Math.max(zoom, 0.2);
    ctx.save();
    ctx.lineWidth = 1.5 * unit;
    ctx.setLineDash([6 * unit, 5 * unit]);
    ctx.strokeStyle = 'rgba(79, 216, 255, 0.85)';
    const p = this.pending;
    if (p) {
      ctx.beginPath();
      if (p.t === 'wall' || p.t === 'portal') {
        ctx.moveTo(p.x1, p.y1);
        ctx.lineTo(p.x2, p.y2);
        if (p.t === 'portal') {
          ctx.moveTo(p.x1 + 28, p.y1);
          ctx.arc(p.x1, p.y1, 28, 0, TAU);
          ctx.moveTo(p.x2 + 28, p.y2);
          ctx.arc(p.x2, p.y2, 28, 0, TAU);
        }
      } else if (p.t === 'boost') {
        ctx.arc(p.x, p.y, 26, 0, TAU);
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(p.a) * 60, p.y + Math.sin(p.a) * 60);
      } else {
        ctx.arc(p.x, p.y, p.r, 0, TAU);
      }
      ctx.stroke();
    }
    // Tee: a labelled ring, so it can be found and moved like everything else.
    ctx.setLineDash([]);
    ctx.strokeStyle = 'rgba(255,255,255,0.6)';
    ctx.beginPath();
    ctx.arc(this.hole.tee[0], this.hole.tee[1], 14, 0, TAU);
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.font = `700 ${11 * unit}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.fillText('TEE', this.hole.tee[0], this.hole.tee[1] - 20);
    ctx.restore();
  }

  dispose(): void {
    this.detach();
  }

  /* ------------------------------------------------------------ pointer */

  private screenPoint(event: PointerEvent | WheelEvent): Vec2 {
    const rect = this.host.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  private worldPoint(event: PointerEvent): Vec2 {
    const p = this.host.camera.screenToWorld(this.screenPoint(event));
    return {
      x: clamp(Math.round(p.x), CUSTOM_BOUNDS.minX, CUSTOM_BOUNDS.maxX),
      y: clamp(Math.round(p.y), CUSTOM_BOUNDS.minY, CUSTOM_BOUNDS.maxY),
    };
  }

  /** Pointer capture, which throws for a pointer the browser no longer knows. */
  private capture(pointerId: number, on: boolean): void {
    try {
      if (on) this.host.canvas.setPointerCapture?.(pointerId);
      else this.host.canvas.releasePointerCapture?.(pointerId);
    } catch {
      // Already gone: nothing to hold or let go of.
    }
  }

  /** Zoom limits: from a little wider than the board to close work. */
  private zoomRange(): [number, number] {
    const fit = this.host.camera.fit(CUSTOM_BOUNDS, 70, 1.2);
    return [fit * 0.85, Math.max(fit * 4, 1.6)];
  }

  private onWheel = (event: WheelEvent): void => {
    event.preventDefault();
    const camera = this.host.camera;
    const [min, max] = this.zoomRange();
    const zoom = clamp(camera.zoom * Math.exp(-event.deltaY * 0.0015), min, max);
    camera.zoomAround(this.screenPoint(event), zoom);
  };

  /** Two fingers down: whatever the first one started is not an edit. */
  private startPinch(): void {
    if (this.pointerId !== null) {
      this.capture(this.pointerId, false);
      this.pointerId = null;
      // The first finger may already have placed or moved something.
      if (this.history.length > this.historyAtDown) {
        this.hole = this.history.pop()!;
        this.changed();
        this.undoButton.disabled = this.history.length === 0;
      }
    }
    this.endGesture();
    const [a, b] = [...this.touches.values()];
    this.pinch = { distance: V.distance(a!, b!), center: V.lerpVec(a!, b!, 0.5) };
  }

  private movePinch(): void {
    const pinch = this.pinch;
    if (!pinch) return;
    const [a, b] = [...this.touches.values()];
    const distance = V.distance(a!, b!);
    const center = V.lerpVec(a!, b!, 0.5);
    const camera = this.host.camera;
    const [min, max] = this.zoomRange();
    const zoom =
      pinch.distance > 1 ? clamp(camera.zoom * (distance / pinch.distance), min, max) : camera.zoom;
    camera.zoomAround(pinch.center, zoom, center);
    this.pinch = { distance, center };
  }

  private onDown = (event: PointerEvent): void => {
    if (event.pointerType === 'touch') {
      this.touches.set(event.pointerId, this.screenPoint(event));
      if (this.touches.size === 2) {
        this.startPinch();
        return;
      }
      if (this.touches.size > 2 || this.pinch) return;
    }
    if (event.button !== 0 || this.pointerId !== null) return;
    this.historyAtDown = this.history.length;
    const p = this.worldPoint(event);
    this.pointerId = event.pointerId;
    this.capture(event.pointerId, true);
    const tool = this.tool;

    if (tool === 'move') {
      this.grab = this.hitTest(p);
      if (this.grab) this.checkpoint();
      else this.status('Nothing there to move');
      return;
    }
    if (tool === 'erase') {
      this.erase(p);
      return;
    }
    if (tool === 'star') {
      this.checkpoint();
      if (this.hole.stars.length >= CUSTOM_LIMITS.stars) this.hole.stars.shift();
      this.hole.stars.push([p.x, p.y]);
      this.changed();
      return;
    }
    if (tool === 'tee' || tool === 'cup') {
      this.checkpoint();
      this.hole[tool] = [p.x, p.y];
      this.grab = { kind: tool, offset: V.ZERO };
      this.changed();
      return;
    }
    if (this.hole.items.length >= CUSTOM_LIMITS.items) {
      this.status(`That is the limit — ${CUSTOM_LIMITS.items} things to a hole`, 'bad');
      return;
    }
    this.pendingStart = p;
    this.pending =
      tool === 'wall' || tool === 'portal'
        ? { t: tool, x1: p.x, y1: p.y, x2: p.x, y2: p.y }
        : tool === 'boost'
          ? { t: 'boost', x: p.x, y: p.y, a: 0 }
          : { t: tool, x: p.x, y: p.y, r: DEFAULT_RADIUS[tool] };
  };

  private onMove = (event: PointerEvent): void => {
    if (this.touches.has(event.pointerId)) {
      this.touches.set(event.pointerId, this.screenPoint(event));
      if (this.pinch && this.touches.size >= 2) {
        this.movePinch();
        return;
      }
    }
    if (event.pointerId !== this.pointerId) return;
    const p = this.worldPoint(event);
    const pending = this.pending;
    if (pending && this.pendingStart) {
      if (pending.t === 'wall' || pending.t === 'portal') {
        pending.x2 = p.x;
        pending.y2 = p.y;
      } else if (pending.t === 'boost') {
        if (V.distance(p, this.pendingStart) > 8) {
          pending.a = Math.atan2(p.y - this.pendingStart.y, p.x - this.pendingStart.x);
        }
      } else {
        const d = V.distance(p, this.pendingStart);
        // A plain click keeps the default size; a drag sets it.
        if (d > 8) pending.r = Math.round(clamp(d, CUSTOM_LIMITS.minRadius, CUSTOM_LIMITS.maxRadius));
      }
      return;
    }
    const grab = this.grab;
    if (!grab) return;
    const target = inBounds(V.add(p, grab.offset));
    if (grab.kind === 'item') {
      const item = this.hole.items[grab.index];
      if (!item) return;
      if (item.t === 'wall' || item.t === 'portal') {
        const mid = { x: (item.x1 + item.x2) / 2, y: (item.y1 + item.y2) / 2 };
        // Both ends stay inside, or the hole would come back bent from a link.
        const b = CUSTOM_BOUNDS;
        const dx = clamp(
          Math.round(target.x - mid.x),
          b.minX - Math.min(item.x1, item.x2),
          b.maxX - Math.max(item.x1, item.x2),
        );
        const dy = clamp(
          Math.round(target.y - mid.y),
          b.minY - Math.min(item.y1, item.y2),
          b.maxY - Math.max(item.y1, item.y2),
        );
        item.x1 += dx;
        item.x2 += dx;
        item.y1 += dy;
        item.y2 += dy;
      } else {
        item.x = Math.round(target.x);
        item.y = Math.round(target.y);
      }
    } else if (grab.kind === 'star') {
      this.hole.stars[grab.index] = [Math.round(target.x), Math.round(target.y)];
    } else {
      this.hole[grab.kind] = [Math.round(target.x), Math.round(target.y)];
    }
    this.changed(false);
  };

  private onUp = (event: PointerEvent): void => {
    this.liftTouch(event);
    if (event.pointerId !== this.pointerId) return;
    this.pointerId = null;
    this.capture(event.pointerId, false);
    const pending = this.pending;
    this.pending = null;
    this.pendingStart = null;
    if (pending) {
      if (pending.t === 'wall' && Math.hypot(pending.x2 - pending.x1, pending.y2 - pending.y1) < 24) {
        this.status('Drag further to draw a wall');
        return;
      }
      if (pending.t === 'portal' && Math.hypot(pending.x2 - pending.x1, pending.y2 - pending.y1) < 90) {
        this.status('Drag the way out further from the way in');
        return;
      }
      this.checkpoint();
      this.hole.items.push(pending);
      this.changed();
    }
    if (this.grab) {
      this.grab = null;
      this.changed();
    }
  };

  /** A finger off the glass; the pinch ends once every finger is. */
  private liftTouch(event: PointerEvent): void {
    if (!this.touches.delete(event.pointerId)) return;
    if (this.touches.size < 2 && this.pinch) {
      const rest = [...this.touches.values()];
      // The pinch stays on while any finger does, so one left resting on the
      // glass cannot start drawing by accident.
      if (rest.length === 0) this.pinch = null;
      else this.pinch = { distance: 0, center: rest[0]! };
    }
  }

  private onCancel = (event: PointerEvent): void => {
    this.liftTouch(event);
    if (event.pointerId !== this.pointerId) return;
    this.pointerId = null;
    this.pending = null;
    this.pendingStart = null;
    this.grab = null;
  };

  /** The topmost thing under `p`: markers first, then items newest first. */
  private hitTest(p: Vec2): Grab | null {
    const near = (x: number, y: number, r: number) => V.distance(p, { x, y }) <= r;
    if (near(this.hole.cup[0], this.hole.cup[1], 20)) {
      return { kind: 'cup', offset: V.sub({ x: this.hole.cup[0], y: this.hole.cup[1] }, p) };
    }
    if (near(this.hole.tee[0], this.hole.tee[1], 20)) {
      return { kind: 'tee', offset: V.sub({ x: this.hole.tee[0], y: this.hole.tee[1] }, p) };
    }
    for (let i = this.hole.stars.length - 1; i >= 0; i--) {
      const [x, y] = this.hole.stars[i]!;
      if (near(x, y, 20)) return { kind: 'star', index: i, offset: V.sub({ x, y }, p) };
    }
    for (let i = this.hole.items.length - 1; i >= 0; i--) {
      const item = this.hole.items[i]!;
      if (item.t === 'wall' || item.t === 'portal') {
        const a = { x: item.x1, y: item.y1 };
        const b = { x: item.x2, y: item.y2 };
        const reach = item.t === 'portal' ? 30 : 18;
        const onEnd = item.t === 'portal' && (V.distance(p, a) <= reach || V.distance(p, b) <= reach);
        if (onEnd || (item.t === 'wall' && distanceToSegment(p, a, b) <= reach)) {
          const mid = V.lerpVec(a, b, 0.5);
          return { kind: 'item', index: i, offset: V.sub(mid, p) };
        }
      } else if (near(item.x, item.y, (item.t === 'boost' ? 26 : item.r) + 6)) {
        return { kind: 'item', index: i, offset: V.sub({ x: item.x, y: item.y }, p) };
      }
    }
    return null;
  }

  private erase(p: Vec2): void {
    const hit = this.hitTest(p);
    if (!hit) {
      this.status('Nothing there to erase');
      return;
    }
    if (hit.kind === 'tee' || hit.kind === 'cup') {
      this.status('Every hole needs its tee and its cup — move them instead', 'bad');
      return;
    }
    this.checkpoint();
    if (hit.kind === 'star') this.hole.stars.splice(hit.index, 1);
    else if (hit.kind === 'item') this.hole.items.splice(hit.index, 1);
    this.changed();
  }

  /* -------------------------------------------------------------- state */

  /** Remembers the hole as it is, before an edit. */
  private checkpoint(): void {
    this.history.push(structuredClone(this.hole));
    if (this.history.length > 80) this.history.shift();
    this.undoButton.disabled = false;
  }

  /** Steps back one edit. */
  undo(): void {
    const previous = this.history.pop();
    if (!previous) return;
    this.endGesture();
    this.hole = previous;
    this.changed();
    this.undoButton.disabled = this.history.length === 0;
    this.status('Undone');
  }

  private onKey = (event: KeyboardEvent): void => {
    const target = event.target as HTMLElement | null;
    if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;
    // Z or Ctrl/Cmd+Z; with Shift it is redo elsewhere, so leave it be.
    if (event.code === 'KeyZ' && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      this.undo();
    }
  };

  private setPar(par: number): void {
    this.checkpoint();
    this.hole.par = clamp(Math.round(par), CUSTOM_LIMITS.minPar, CUSTOM_LIMITS.maxPar);
    this.changed();
  }

  /** Drops a drag or placement in progress: it was about the hole as it was. */
  private endGesture(): void {
    this.grab = null;
    this.pending = null;
    this.pendingStart = null;
  }

  /** Makes a shared hole your own draft, after asking if that replaces one. */
  private keep(): void {
    if (!this.shared) return;
    if (loadDraft(DRAFT_KEY)) {
      // eslint-disable-next-line no-alert
      if (!window.confirm('Replace your own hole with this one?')) return;
    }
    this.shared = false;
    this.keepButton.hidden = true;
    saveDraft(DRAFT_KEY, this.hole);
    this.status('Kept — this is your hole now', 'good');
  }

  private clear(): void {
    this.checkpoint();
    this.endGesture();
    this.hole = emptyCustomHole();
    this.changed();
    this.status('Started a fresh hole');
  }

  private test(): void {
    const problems = this.problems();
    if (problems.length > 0) {
      this.status(`Fix this first: ${problems[0]}`, 'bad');
      return;
    }
    this.host.onTest(structuredClone(this.hole));
  }

  /** After any edit: rebuild the world, save the draft, refresh the bars. */
  private changed(refreshFields = true): void {
    this.world = compileLevel(customToLevel(this.hole));
    if (!this.shared) saveDraft(DRAFT_KEY, this.hole);
    if (refreshFields) this.syncFields();
  }

  private syncFields(): void {
    if (document.activeElement !== this.nameInput) this.nameInput.value = this.hole.name;
    this.parValue.textContent = `Par ${this.hole.par}`;
    this.downToggle.checked = this.hole.down;
    this.wallsToggle.checked = this.hole.walls;
    this.turnToggle.checked = this.hole.turn === true;
  }

  private renderToolbar(): void {
    this.toolbar.replaceChildren(
      ...TOOLS.map((tool) =>
        button(
          tool.label,
          () => {
            this.tool = tool.id;
            this.host.onClick?.();
            this.renderToolbar();
            this.status(tool.hint);
          },
          {
            class: `editor__tool ${this.tool === tool.id ? 'editor__tool--active' : ''}`.trim(),
            attrs: { 'aria-pressed': String(this.tool === tool.id), title: tool.hint },
          },
        ),
      ),
    );
  }
}

const inBounds = (p: Vec2): Vec2 => ({
  x: clamp(p.x, CUSTOM_BOUNDS.minX, CUSTOM_BOUNDS.maxX),
  y: clamp(p.y, CUSTOM_BOUNDS.minY, CUSTOM_BOUNDS.maxY),
});

const distanceToSegment = (p: Vec2, a: Vec2, b: Vec2): number => {
  const ab = V.sub(b, a);
  const len = V.lengthSq(ab);
  const t = len === 0 ? 0 : clamp(V.dot(V.sub(p, a), ab) / len, 0, 1);
  return V.distance(p, V.addScaled(a, ab, t));
};

const loadDraft = (key: string): CustomHole | null => {
  try {
    const raw = localStorage.getItem(key);
    return raw ? sanitizeCustomHole(JSON.parse(raw)) : null;
  } catch {
    return null;
  }
};

const saveDraft = (key: string, hole: CustomHole): void => {
  try {
    localStorage.setItem(key, JSON.stringify(hole));
  } catch {
    // Private mode or a full disk: the hole still plays, it just will not be
    // waiting next time.
  }
};
