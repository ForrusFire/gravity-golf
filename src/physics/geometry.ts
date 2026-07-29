import { TAU } from '../core/math';
import * as V from '../core/vec2';
import type { Vec2 } from '../core/vec2';
import type { Body, CapsuleShape, CircleShape, MotionSpec, PolygonShape, Shape } from './types';

/** Result of querying the closest point on a shape's surface. */
export interface SurfaceQuery {
  /** The closest point on the surface. */
  point: Vec2;
  /** Unit outward normal (points from the shape toward the query point). */
  normal: Vec2;
  /** Signed distance: negative when the query point is inside the shape. */
  distance: number;
  inside: boolean;
}

export interface Aabb {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

/* ---------------------------------------------------------------- polygons */

/** Twice the signed area; positive for counter-clockwise winding in y-down space. */
export const polygonSignedArea2 = (vertices: readonly Vec2[]): number => {
  let sum = 0;
  for (let i = 0; i < vertices.length; i++) {
    const a = vertices[i]!;
    const b = vertices[(i + 1) % vertices.length]!;
    sum += a.x * b.y - b.x * a.y;
  }
  return sum;
};

/**
 * Returns the polygon with counter-clockwise winding, so edge normals computed
 * as `perp(edge)` always point outward. Cheap to call at level-load time.
 */
export const withCcwWinding = (shape: PolygonShape): PolygonShape =>
  polygonSignedArea2(shape.vertices) >= 0
    ? shape
    : { ...shape, vertices: [...shape.vertices].reverse() };

/** Local vertices transformed into world space. */
export const worldVertices = (shape: PolygonShape): Vec2[] => {
  const c = Math.cos(shape.rotation);
  const s = Math.sin(shape.rotation);
  return shape.vertices.map((v) => ({
    x: shape.center.x + v.x * c - v.y * s,
    y: shape.center.y + v.x * s + v.y * c,
  }));
};

/** Builds a regular n-gon with the given circumradius. */
export const regularPolygon = (
  center: Vec2,
  radius: number,
  sides: number,
  rotation = 0,
): PolygonShape => {
  const vertices: Vec2[] = [];
  for (let i = 0; i < sides; i++) {
    const a = (i / sides) * TAU;
    vertices.push({ x: Math.cos(a) * radius, y: Math.sin(a) * radius });
  }
  return withCcwWinding({ kind: 'polygon', center, vertices, rotation });
};

/** Builds an axis-aligned rectangle (before `rotation` is applied). */
export const rectPolygon = (
  center: Vec2,
  halfWidth: number,
  halfHeight: number,
  rotation = 0,
): PolygonShape =>
  withCcwWinding({
    kind: 'polygon',
    center,
    vertices: [
      { x: -halfWidth, y: -halfHeight },
      { x: halfWidth, y: -halfHeight },
      { x: halfWidth, y: halfHeight },
      { x: -halfWidth, y: halfHeight },
    ],
    rotation,
  });

/* ----------------------------------------------------------------- queries */

/** Closest point to `p` on segment `a`-`b`. */
export const closestPointOnSegment = (a: Vec2, b: Vec2, p: Vec2): Vec2 => {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby;
  if (lenSq < 1e-12) return a;
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return { x: a.x + abx * t, y: a.y + aby * t };
};

const queryFromPoint = (nearest: Vec2, p: Vec2, thickness: number, fallback: Vec2): SurfaceQuery => {
  const dx = p.x - nearest.x;
  const dy = p.y - nearest.y;
  const len = Math.hypot(dx, dy);
  const normal = len < 1e-9 ? fallback : { x: dx / len, y: dy / len };
  const distance = len - thickness;
  return {
    point: { x: nearest.x + normal.x * thickness, y: nearest.y + normal.y * thickness },
    normal,
    distance,
    inside: distance < 0,
  };
};

const queryCircle = (shape: CircleShape, p: Vec2): SurfaceQuery =>
  queryFromPoint(shape.center, p, shape.radius, { x: 0, y: -1 });

const queryCapsule = (shape: CapsuleShape, p: Vec2): SurfaceQuery => {
  const nearest = closestPointOnSegment(shape.a, shape.b, p);
  const axis = V.normalize(V.sub(shape.b, shape.a));
  const fallback = axis.x === 0 && axis.y === 0 ? { x: 0, y: -1 } : V.perp(axis);
  return queryFromPoint(nearest, p, shape.radius, fallback);
};

const queryPolygon = (shape: PolygonShape, p: Vec2): SurfaceQuery => {
  const verts = worldVertices(shape);
  const n = verts.length;
  if (n === 0) {
    return { point: shape.center, normal: { x: 0, y: -1 }, distance: Infinity, inside: false };
  }
  if (n < 3) {
    const nearest = closestPointOnSegment(verts[0]!, verts[n - 1]!, p);
    return queryFromPoint(nearest, p, 0, { x: 0, y: -1 });
  }

  let bestOutsideDistSq = Infinity;
  let bestOutside: Vec2 = verts[0]!;
  let bestOutsideNormal: Vec2 = { x: 0, y: -1 };

  // Deepest (least negative) edge plane, used when the point is inside.
  let bestInsideDist = -Infinity;
  let bestInsideNormal: Vec2 = { x: 0, y: -1 };
  let bestInsidePoint: Vec2 = verts[0]!;
  let isInside = true;

  for (let i = 0; i < n; i++) {
    const a = verts[i]!;
    const b = verts[(i + 1) % n]!;
    // CCW winding in y-down space -> outward normal is the right-hand perp.
    const edge = { x: b.x - a.x, y: b.y - a.y };
    const edgeLen = Math.hypot(edge.x, edge.y);
    if (edgeLen < 1e-12) continue;
    const normal = { x: edge.y / edgeLen, y: -edge.x / edgeLen };
    const signedDist = (p.x - a.x) * normal.x + (p.y - a.y) * normal.y;
    if (signedDist > 0) isInside = false;

    if (signedDist > bestInsideDist) {
      bestInsideDist = signedDist;
      bestInsideNormal = normal;
      bestInsidePoint = { x: p.x - normal.x * signedDist, y: p.y - normal.y * signedDist };
    }

    const nearest = closestPointOnSegment(a, b, p);
    const dSq = V.distanceSq(nearest, p);
    if (dSq < bestOutsideDistSq) {
      bestOutsideDistSq = dSq;
      bestOutside = nearest;
      bestOutsideNormal = normal;
    }
  }

  if (isInside) {
    return {
      point: bestInsidePoint,
      normal: bestInsideNormal,
      distance: bestInsideDist,
      inside: true,
    };
  }
  return queryFromPoint(bestOutside, p, 0, bestOutsideNormal);
};

/** Closest surface point, outward normal and signed distance for any shape. */
export const closestSurfacePoint = (shape: Shape, p: Vec2): SurfaceQuery => {
  switch (shape.kind) {
    case 'circle':
      return queryCircle(shape, p);
    case 'capsule':
      return queryCapsule(shape, p);
    case 'polygon':
      return queryPolygon(shape, p);
  }
};

export const containsPoint = (shape: Shape, p: Vec2): boolean =>
  closestSurfacePoint(shape, p).distance < 0;

/** The shape's representative centre — used for gravity and rendering. */
export const shapeCenter = (shape: Shape): Vec2 => {
  switch (shape.kind) {
    case 'circle':
      return shape.center;
    case 'capsule':
      return { x: (shape.a.x + shape.b.x) / 2, y: (shape.a.y + shape.b.y) / 2 };
    case 'polygon':
      return shape.center;
  }
};

/** Distance from the centre to the farthest surface point. */
export const shapeRadius = (shape: Shape): number => {
  switch (shape.kind) {
    case 'circle':
      return shape.radius;
    case 'capsule':
      return V.distance(shape.a, shape.b) / 2 + shape.radius;
    case 'polygon': {
      let max = 0;
      for (const v of shape.vertices) max = Math.max(max, V.length(v));
      return max;
    }
  }
};

export const shapeBounds = (shape: Shape): Aabb => {
  switch (shape.kind) {
    case 'circle':
      return {
        minX: shape.center.x - shape.radius,
        minY: shape.center.y - shape.radius,
        maxX: shape.center.x + shape.radius,
        maxY: shape.center.y + shape.radius,
      };
    case 'capsule':
      return {
        minX: Math.min(shape.a.x, shape.b.x) - shape.radius,
        minY: Math.min(shape.a.y, shape.b.y) - shape.radius,
        maxX: Math.max(shape.a.x, shape.b.x) + shape.radius,
        maxY: Math.max(shape.a.y, shape.b.y) + shape.radius,
      };
    case 'polygon': {
      const verts = worldVertices(shape);
      let minX = Infinity;
      let minY = Infinity;
      let maxX = -Infinity;
      let maxY = -Infinity;
      for (const v of verts) {
        if (v.x < minX) minX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.x > maxX) maxX = v.x;
        if (v.y > maxY) maxY = v.y;
      }
      return { minX, minY, maxX, maxY };
    }
  }
};

/* ---------------------------------------------------------------- movement */

/** Where a moving body's centre sits at time `t`. Pure — no hidden state. */
export const motionOffset = (motion: MotionSpec, t: number): Vec2 => {
  switch (motion.kind) {
    case 'orbit': {
      const a = motion.phase + motion.speed * t;
      return {
        x: motion.center.x + Math.cos(a) * motion.radius,
        y: motion.center.y + Math.sin(a) * motion.radius,
      };
    }
    case 'oscillate': {
      const cycles = motion.period > 1e-6 ? t / motion.period : 0;
      // Sine easing gives smooth reversals at both ends of the travel.
      const s = 0.5 - 0.5 * Math.cos((cycles + motion.phase) * TAU);
      return V.lerpVec(motion.from, motion.to, s);
    }
    case 'spin':
      return { x: 0, y: 0 };
  }
};

export const motionRotation = (motion: MotionSpec, t: number): number =>
  motion.kind === 'spin' ? motion.phase + motion.speed * t : 0;

/** Angular velocity in rad/s contributed by the motion. */
export const motionAngularVelocity = (motion: MotionSpec): number =>
  motion.kind === 'spin' ? motion.speed : 0;

const translateShape = (shape: Shape, d: Vec2): Shape => {
  if (d.x === 0 && d.y === 0) return shape;
  switch (shape.kind) {
    case 'circle':
      return { ...shape, center: V.add(shape.center, d) };
    case 'capsule':
      return { ...shape, a: V.add(shape.a, d), b: V.add(shape.b, d) };
    case 'polygon':
      return { ...shape, center: V.add(shape.center, d) };
  }
};

const rotateShape = (shape: Shape, radians: number): Shape => {
  if (radians === 0) return shape;
  switch (shape.kind) {
    case 'circle':
      return shape;
    case 'capsule': {
      const mid = shapeCenter(shape);
      return {
        ...shape,
        a: V.add(mid, V.rotate(V.sub(shape.a, mid), radians)),
        b: V.add(mid, V.rotate(V.sub(shape.b, mid), radians)),
      };
    }
    case 'polygon':
      return { ...shape, rotation: shape.rotation + radians };
  }
};

/** The body's shape in world space at time `t`, with its motion applied. */
export const bodyShapeAt = (body: Body, t: number): Shape => {
  if (!body.motion) return body.shape;
  if (body.motion.kind === 'spin') {
    return rotateShape(body.shape, motionRotation(body.motion, t));
  }
  // Positional motion is expressed relative to the shape's authored centre.
  const target = motionOffset(body.motion, t);
  return translateShape(body.shape, V.sub(target, shapeCenter(body.shape)));
};

/**
 * Velocity of the body's surface at world point `p`, time `t`.
 * Central differences keep this correct for every motion kind without
 * hand-derived derivatives.
 */
export const bodyVelocityAt = (body: Body, t: number, p: Vec2): Vec2 => {
  const motion = body.motion;
  if (!motion) return V.ZERO;
  if (motion.kind === 'spin') {
    const center = shapeCenter(bodyShapeAt(body, t));
    const r = V.sub(p, center);
    const w = motionAngularVelocity(motion);
    return { x: -w * r.y, y: w * r.x };
  }
  const h = 1 / 600;
  const before = motionOffset(motion, t - h);
  const after = motionOffset(motion, t + h);
  return { x: (after.x - before.x) / (2 * h), y: (after.y - before.y) / (2 * h) };
};
