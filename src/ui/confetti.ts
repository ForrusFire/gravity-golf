import { TAU } from '../core/math';
import { Rng } from '../core/rng';

const COLORS = ['#66e6b8', '#4fd8ff', '#ffd25a', '#c792ff', '#ff7ae0', '#ffffff'];

interface Piece {
  x: number;
  y: number;
  vx: number;
  vy: number;
  spin: number;
  angle: number;
  /** Phase of the tumble that makes a flat piece flash as it turns. */
  flip: number;
  size: number;
  color: string;
}

/**
 * A burst of confetti over everything, for a result worth it. It is a canvas
 * of its own on top of the page, so it plays over the results panel rather
 * than under it, and it cleans itself up when the last piece is gone.
 */
export const launchConfetti = (host: HTMLElement, amount = 1): void => {
  if (typeof document === 'undefined') return;
  const canvas = document.createElement('canvas');
  canvas.className = 'confetti';
  canvas.setAttribute('aria-hidden', 'true');
  host.append(canvas);
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    canvas.remove();
    return;
  }

  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = host.clientWidth || window.innerWidth;
  const height = host.clientHeight || window.innerHeight;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  ctx.scale(ratio, ratio);

  const rng = new Rng((Date.now() & 0xffff) + 7);
  const pieces: Piece[] = [];
  const count = Math.round(140 * amount);
  for (let i = 0; i < count; i++) {
    // Two cannons from the lower corners, angled in toward the middle.
    const left = i % 2 === 0;
    const angle = (left ? -1.05 : -2.09) + rng.range(-0.4, 0.4);
    const speed = rng.range(700, 1250) * Math.min(1.2, height / 800 + 0.3);
    pieces.push({
      x: left ? -10 : width + 10,
      y: height * 0.92,
      vx: Math.cos(angle) * speed,
      vy: Math.sin(angle) * speed,
      spin: rng.range(-9, 9),
      angle: rng.range(0, TAU),
      flip: rng.range(0, TAU),
      size: rng.range(5, 10),
      color: COLORS[rng.int(0, COLORS.length - 1)]!,
    });
  }

  let last = performance.now();
  const started = last;
  const frame = (now: number): void => {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    const age = (now - started) / 1000;
    ctx.clearRect(0, 0, width, height);
    let alive = 0;
    for (const p of pieces) {
      p.vy += 950 * dt;
      // Paper drags hard: it flies out fast and then flutters down.
      const drag = Math.exp(-1.3 * dt);
      p.vx *= drag;
      p.vy = Math.min(p.vy * drag, 260 + Math.sin(p.flip) * 60);
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      p.angle += p.spin * dt;
      p.flip += dt * 7;
      if (p.y > height + 20) continue;
      alive++;
      ctx.save();
      ctx.globalAlpha = Math.min(1, Math.max(0, 4.2 - age));
      ctx.translate(p.x, p.y);
      ctx.rotate(p.angle);
      ctx.scale(1, Math.cos(p.flip));
      ctx.fillStyle = p.color;
      ctx.fillRect(-p.size / 2, -p.size * 0.3, p.size, p.size * 0.6);
      ctx.restore();
    }
    if (alive > 0 && age < 4.4) requestAnimationFrame(frame);
    else canvas.remove();
  };
  requestAnimationFrame(frame);
};
