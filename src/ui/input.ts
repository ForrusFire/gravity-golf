import { TAU, clamp } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';

export interface AimInput {
  /** Unit vector the ball would travel along. */
  direction: Vec2;
  /** 0..1. */
  power: number;
  /** True while the player is actively dragging or holding a key. */
  active: boolean;
  source: 'pointer' | 'keyboard';
}

export interface InputCallbacks {
  canShoot(): boolean;
  onShoot(direction: Vec2, power: number): void;
  onPan(deltaScreen: Vec2): void;
  onZoom(factor: number, focusScreen: Vec2): void;
  onCancel(): void;
  /** Fired on the first interaction of any kind, to unlock audio. */
  onFirstInteraction?(): void;
}

export interface InputConfig {
  /** Drag distance in screen pixels that corresponds to full power. */
  maxDragPixels: number;
  aimMode: 'slingshot' | 'direct';
  /** Radians per second while an aim key is held. */
  keyboardTurnRate: number;
  /** Power units per second while a power key is held. */
  keyboardPowerRate: number;
}

export const DEFAULT_INPUT_CONFIG: InputConfig = {
  maxDragPixels: 180,
  aimMode: 'slingshot',
  keyboardTurnRate: 1.6,
  keyboardPowerRate: 0.9,
};

/**
 * Translates pointer, touch, wheel and keyboard events into a single aim
 * state. Pointer capture keeps a drag alive even when it leaves the canvas,
 * so releasing off-screen still takes the shot the player set up.
 */
export class InputController {
  config: InputConfig;

  private dragging = false;
  private dragPointerId: number | null = null;
  private dragStart: Vec2 = V.ZERO;
  private dragCurrent: Vec2 = V.ZERO;

  private panPointerId: number | null = null;
  private panLast: Vec2 = V.ZERO;

  /** Active touch points, for pinch-zoom. */
  private touches = new Map<number, Vec2>();
  private pinchDistance = 0;

  private keyAngle = 0;
  private keyPower = 0.5;
  private keyboardActive = false;
  private keysDown = new Set<string>();
  private interacted = false;
  private disposed = false;

  private readonly listeners: Array<() => void> = [];

  constructor(
    private readonly element: HTMLElement,
    private readonly callbacks: InputCallbacks,
    config: Partial<InputConfig> = {},
  ) {
    this.config = { ...DEFAULT_INPUT_CONFIG, ...config };
    this.attach();
  }

  /** Current aim, or null when the player is not aiming. */
  get aim(): AimInput | null {
    if (this.dragging) {
      const drag = V.sub(this.dragCurrent, this.dragStart);
      const distance = V.length(drag);
      if (distance < 4) {
        return {
          direction: this.lastDirection,
          power: 0,
          active: true,
          source: 'pointer',
        };
      }
      // Slingshot: pull away from the target, like drawing a bow.
      const direction =
        this.config.aimMode === 'slingshot' ? V.normalize(V.neg(drag)) : V.normalize(drag);
      this.lastDirection = direction;
      return {
        direction,
        power: clamp(distance / this.config.maxDragPixels, 0, 1),
        active: true,
        source: 'pointer',
      };
    }

    if (this.keyboardActive) {
      return {
        direction: V.fromAngle(this.keyAngle),
        power: this.keyPower,
        active: true,
        source: 'keyboard',
      };
    }

    return null;
  }

  private lastDirection: Vec2 = { x: 1, y: 0 };

  /** Advances held-key aiming. Call once per frame. */
  update(dt: number): void {
    if (!this.keyboardActive) return;
    const turn = this.config.keyboardTurnRate;
    if (this.keysDown.has('ArrowLeft') || this.keysDown.has('KeyA')) this.keyAngle -= turn * dt;
    if (this.keysDown.has('ArrowRight') || this.keysDown.has('KeyD')) this.keyAngle += turn * dt;
    this.keyAngle = ((this.keyAngle % TAU) + TAU) % TAU;

    const rate = this.config.keyboardPowerRate;
    if (this.keysDown.has('ArrowUp') || this.keysDown.has('KeyW')) {
      this.keyPower = clamp(this.keyPower + rate * dt, 0, 1);
    }
    if (this.keysDown.has('ArrowDown') || this.keysDown.has('KeyS')) {
      this.keyPower = clamp(this.keyPower - rate * dt, 0, 1);
    }
  }

  /** Begins keyboard aiming from wherever the player last aimed. */
  startKeyboardAim(): void {
    if (!this.callbacks.canShoot()) return;
    this.keyboardActive = true;
    this.keyAngle = V.angleOf(this.lastDirection);
  }

  cancelAim(): void {
    this.dragging = false;
    this.dragPointerId = null;
    this.keyboardActive = false;
    this.callbacks.onCancel();
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    for (const off of this.listeners) off();
    this.listeners.length = 0;
    this.keysDown.clear();
  }

  /* ------------------------------------------------------------- wiring */

  private on<K extends keyof HTMLElementEventMap>(
    target: HTMLElement | Window | Document,
    type: K | string,
    handler: (event: never) => void,
    options?: AddEventListenerOptions,
  ): void {
    const listener = handler as EventListener;
    target.addEventListener(type, listener, options);
    this.listeners.push(() => target.removeEventListener(type, listener, options));
  }

  private attach(): void {
    this.on<'pointerdown'>(this.element, 'pointerdown', (e: PointerEvent) => this.onPointerDown(e));
    this.on<'pointermove'>(this.element, 'pointermove', (e: PointerEvent) => this.onPointerMove(e));
    this.on<'pointerup'>(this.element, 'pointerup', (e: PointerEvent) => this.onPointerUp(e));
    this.on<'pointercancel'>(this.element, 'pointercancel', (e: PointerEvent) =>
      this.onPointerCancel(e),
    );
    this.on(this.element, 'contextmenu', (e: Event) => e.preventDefault());
    this.on<'wheel'>(this.element, 'wheel', (e: WheelEvent) => this.onWheel(e), { passive: false });
    this.on<'keydown'>(window, 'keydown', (e: KeyboardEvent) => this.onKeyDown(e));
    this.on<'keyup'>(window, 'keyup', (e: KeyboardEvent) => this.onKeyUp(e));
    // A drag that ends outside the window must not leave the aim stuck on.
    this.on(window, 'blur', () => {
      this.keysDown.clear();
      if (this.dragging) this.cancelAim();
    });
  }

  private localPoint(event: PointerEvent | WheelEvent): Vec2 {
    const rect = this.element.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  private firstInteraction(): void {
    if (this.interacted) return;
    this.interacted = true;
    this.callbacks.onFirstInteraction?.();
  }

  private onPointerDown(event: PointerEvent): void {
    this.firstInteraction();
    const point = this.localPoint(event);

    if (event.pointerType === 'touch') {
      this.touches.set(event.pointerId, point);
      if (this.touches.size === 2) {
        // Second finger down: switch from aiming to pinch-zoom / pan.
        this.dragging = false;
        this.dragPointerId = null;
        this.pinchDistance = this.currentPinchDistance();
        this.callbacks.onCancel();
        return;
      }
    }

    // Secondary/middle button pans.
    if (event.button === 2 || event.button === 1) {
      this.panPointerId = event.pointerId;
      this.panLast = point;
      this.element.setPointerCapture?.(event.pointerId);
      event.preventDefault();
      return;
    }

    if (event.button !== 0) return;
    if (!this.callbacks.canShoot()) return;

    this.keyboardActive = false;
    this.dragging = true;
    this.dragPointerId = event.pointerId;
    this.dragStart = point;
    this.dragCurrent = point;
    this.element.setPointerCapture?.(event.pointerId);
    event.preventDefault();
  }

  private onPointerMove(event: PointerEvent): void {
    const point = this.localPoint(event);

    if (event.pointerType === 'touch' && this.touches.has(event.pointerId)) {
      this.touches.set(event.pointerId, point);
      if (this.touches.size === 2) {
        const distance = this.currentPinchDistance();
        if (this.pinchDistance > 1 && distance > 1) {
          this.callbacks.onZoom(distance / this.pinchDistance, this.pinchCenter());
        }
        this.pinchDistance = distance;
        return;
      }
    }

    if (this.panPointerId === event.pointerId) {
      this.callbacks.onPan(V.sub(point, this.panLast));
      this.panLast = point;
      return;
    }

    if (this.dragging && this.dragPointerId === event.pointerId) {
      this.dragCurrent = point;
    }
  }

  private onPointerUp(event: PointerEvent): void {
    this.touches.delete(event.pointerId);
    if (this.touches.size < 2) this.pinchDistance = 0;

    if (this.panPointerId === event.pointerId) {
      this.panPointerId = null;
      this.element.releasePointerCapture?.(event.pointerId);
      return;
    }

    if (!this.dragging || this.dragPointerId !== event.pointerId) return;

    const aim = this.aim;
    this.dragging = false;
    this.dragPointerId = null;
    this.element.releasePointerCapture?.(event.pointerId);

    if (aim && aim.power > 0.02 && this.callbacks.canShoot()) {
      this.callbacks.onShoot(aim.direction, aim.power);
    } else {
      this.callbacks.onCancel();
    }
  }

  private onPointerCancel(event: PointerEvent): void {
    this.touches.delete(event.pointerId);
    if (this.panPointerId === event.pointerId) this.panPointerId = null;
    if (this.dragPointerId === event.pointerId) {
      this.dragging = false;
      this.dragPointerId = null;
      this.callbacks.onCancel();
    }
  }

  private currentPinchDistance(): number {
    const points = [...this.touches.values()];
    if (points.length < 2) return 0;
    return V.distance(points[0]!, points[1]!);
  }

  private pinchCenter(): Vec2 {
    const points = [...this.touches.values()];
    if (points.length < 2) return V.ZERO;
    return V.mul(V.add(points[0]!, points[1]!), 0.5);
  }

  private onWheel(event: WheelEvent): void {
    event.preventDefault();
    this.firstInteraction();
    const factor = Math.exp(-event.deltaY * 0.0015);
    this.callbacks.onZoom(factor, this.localPoint(event));
  }

  private onKeyDown(event: KeyboardEvent): void {
    // Never swallow keys aimed at a focused control.
    const target = event.target as HTMLElement | null;
    if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;

    this.firstInteraction();
    this.keysDown.add(event.code);

    const aimKeys = ['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'KeyA', 'KeyD', 'KeyW', 'KeyS'];
    if (aimKeys.includes(event.code)) {
      if (!this.keyboardActive && this.callbacks.canShoot()) {
        // Resume from wherever the player last aimed.
        this.keyboardActive = true;
        this.keyAngle = V.angleOf(this.lastDirection);
      }
      if (this.keyboardActive) event.preventDefault();
      return;
    }

    if (event.code === 'Space' || event.code === 'Enter') {
      if (this.keyboardActive && this.callbacks.canShoot()) {
        event.preventDefault();
        const direction = V.fromAngle(this.keyAngle);
        this.lastDirection = direction;
        this.keyboardActive = false;
        this.callbacks.onShoot(direction, this.keyPower);
      } else if (this.callbacks.canShoot()) {
        event.preventDefault();
        this.startKeyboardAim();
      }
      return;
    }

    if (event.code === 'Escape' && (this.keyboardActive || this.dragging)) {
      this.cancelAim();
    }
  }

  private onKeyUp(event: KeyboardEvent): void {
    this.keysDown.delete(event.code);
  }
}
