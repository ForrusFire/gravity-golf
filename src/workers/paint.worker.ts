/// <reference lib="webworker" />
import type { BodyStyle } from '../physics/types';
import type { BodyColors } from '../render/palette';
import { paintSprite } from '../render/sprites';

export interface PaintRequest {
  key: string;
  id: string;
  style: BodyStyle;
  colors: BodyColors;
  radius: number;
  bucket: number;
}

export interface PaintResponse {
  key: string;
  bitmap: ImageBitmap | null;
  half: number;
}

/**
 * Paints big planet sprites off the main thread. A large world is a couple of
 * megapixels of gradient work — tens of milliseconds that would otherwise land
 * in a single frame as a visible hitch.
 */
self.onmessage = (event: MessageEvent<PaintRequest>) => {
  const { key, id, style, colors, radius, bucket } = event.data;
  const sprite = paintSprite(id, style, colors, radius, bucket);
  const canvas = sprite?.canvas;
  const bitmap =
    canvas && 'transferToImageBitmap' in canvas ? canvas.transferToImageBitmap() : null;
  const response: PaintResponse = { key, bitmap, half: sprite?.half ?? 0 };
  (self as unknown as DedicatedWorkerGlobalScope).postMessage(response, bitmap ? [bitmap] : []);
};
