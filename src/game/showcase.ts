import { TAU } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import type { World } from '../physics/world';
import { compileLevel, planet, rock, type LevelDef } from './level';

/**
 * The scene behind the title screen: a little system with a ball looping
 * round its largest world. Nothing is played — the ball's path is a closed
 * ellipse worked out on the spot — so it can run forever without drifting,
 * crashing or costing a physics step, and it looks the same every visit.
 */
const SHOWCASE: LevelDef = {
  id: 'showcase',
  name: 'Showcase',
  chapter: 0,
  par: 1,
  tee: V.vec(0, 0),
  // Parked far away: the title has nothing to aim at.
  hole: { position: V.vec(0, -100000) },
  bounds: { minX: -2000, minY: -2000, maxX: 2000, maxY: 2000 },
  boundsMode: 'open',
  bodies: [
    planet('show-giant', V.vec(0, 40), 130, 1000),
    planet('show-moon', V.vec(0, 40), 26, 0, {
      style: 'moon',
      motion: { kind: 'orbit', center: V.vec(0, 40), radius: 600, speed: 0.09, phase: 2.2 },
    }),
    planet('show-ice', V.vec(-620, -320), 62, 0, { style: 'ice', material: 'ice' }),
    planet('show-sand', V.vec(480, -100), 55, 0, { style: 'sand', material: 'sand' }),
    rock('show-rock-a', V.vec(380, 330), 22, { style: 'asteroid' }),
    rock('show-rock-b', V.vec(430, 370), 13, { style: 'asteroid' }),
    planet('show-goo', V.vec(-640, 330), 40, 0, { style: 'goo', material: 'sticky' }),
    planet('show-crystal', V.vec(-260, -600), 30, 0, { style: 'crystal' }),
  ],
};

/** Semi-axes and tilt of the ball's loop. Periapsis (~193) clears the giant's 130 radius. */
const ORBIT = { a: 330, b: 300, tilt: -0.42, period: 10 };
const TRAIL_SECONDS = 1.6;
const TRAIL_POINTS = 48;

export interface Showcase {
  world: World;
  /** The ball at `time` seconds. */
  ballAt(time: number): Vec2;
  /** The trail behind the ball at `time`, oldest first. */
  trailAt(time: number): Vec2[];
  /** Where the camera looks at `time`: a slow drift round the system. */
  eyeAt(time: number): Vec2;
}

export const createShowcase = (): Showcase => {
  const world = compileLevel(SHOWCASE);
  const center = SHOWCASE.bodies![0]!.shape.kind === 'circle'
    ? SHOWCASE.bodies![0]!.shape.center
    : V.ZERO;
  // The focus sits at the planet, as an orbit's should, so the ball whips
  // round close in and dawdles out on the far side.
  const focal = Math.sqrt(ORBIT.a * ORBIT.a - ORBIT.b * ORBIT.b);
  const cos = Math.cos(ORBIT.tilt);
  const sin = Math.sin(ORBIT.tilt);

  const ballAt = (time: number): Vec2 => {
    // Solve Kepler's equation for the eccentric anomaly: a few Newton steps
    // are plenty at this eccentricity.
    const e = focal / ORBIT.a;
    const mean = ((time / ORBIT.period) % 1) * TAU;
    let E = mean;
    for (let i = 0; i < 5; i++) E -= (E - e * Math.sin(E) - mean) / (1 - e * Math.cos(E));
    const x = ORBIT.a * Math.cos(E) - focal;
    const y = ORBIT.b * Math.sin(E);
    return { x: center.x + x * cos - y * sin, y: center.y + x * sin + y * cos };
  };

  return {
    world,
    ballAt,
    trailAt: (time: number) => {
      const points: Vec2[] = [];
      for (let i = TRAIL_POINTS; i >= 0; i--) {
        points.push(ballAt(time - (i / TRAIL_POINTS) * TRAIL_SECONDS));
      }
      return points;
    },
    eyeAt: (time: number) => ({
      x: center.x + Math.sin(time * 0.05) * 90,
      y: center.y + Math.cos(time * 0.04) * 50,
    }),
  };
};
