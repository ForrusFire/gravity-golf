import { clamp } from '../core/math';
import { Rng } from '../core/rng';

export type SfxName =
  | 'putt'
  | 'bounce'
  | 'bounceSoft'
  | 'bumper'
  | 'star'
  | 'sink'
  | 'death'
  | 'portal'
  | 'lipout'
  | 'click'
  | 'back'
  | 'unlock';

/**
 * Everything is synthesised at runtime — no audio files to load, no licensing,
 * and pitch can follow gameplay values like impact speed continuously.
 *
 * The AudioContext is created lazily on the first user gesture, because
 * browsers refuse to start audio before one.
 */
export class AudioEngine {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private sfxBus: GainNode | null = null;
  private musicBus: GainNode | null = null;
  private noiseBuffer: AudioBuffer | null = null;
  private musicTimer: ReturnType<typeof setInterval> | null = null;
  private musicStep = 0;
  private rng = new Rng(4242);

  private sfxVolume = 0.7;
  private musicVolume = 0.35;
  private muted = false;
  private failed = false;

  /** True once the context exists and is running. */
  get ready(): boolean {
    return this.ctx !== null && this.ctx.state === 'running';
  }

  get unavailable(): boolean {
    return this.failed;
  }

  /** Safe to call repeatedly; must be called from a user gesture handler. */
  async unlock(): Promise<void> {
    if (this.failed) return;
    try {
      if (!this.ctx) this.init();
      if (this.ctx && this.ctx.state === 'suspended') await this.ctx.resume();
    } catch {
      this.failed = true;
    }
  }

  private init(): void {
    const Ctor: typeof AudioContext | undefined =
      typeof AudioContext !== 'undefined'
        ? AudioContext
        : (globalThis as { webkitAudioContext?: typeof AudioContext }).webkitAudioContext;
    if (!Ctor) {
      this.failed = true;
      return;
    }

    const ctx = new Ctor();
    this.ctx = ctx;

    this.master = ctx.createGain();
    this.master.gain.value = this.muted ? 0 : 1;
    this.master.connect(ctx.destination);

    this.sfxBus = ctx.createGain();
    this.sfxBus.gain.value = this.sfxVolume;
    this.sfxBus.connect(this.master);

    this.musicBus = ctx.createGain();
    this.musicBus.gain.value = this.musicVolume;
    // A gentle low-pass keeps the pad behind the sound effects.
    const filter = ctx.createBiquadFilter();
    filter.type = 'lowpass';
    filter.frequency.value = 1800;
    this.musicBus.connect(filter);
    filter.connect(this.master);

    // One second of white noise, reused for every percussive effect.
    const length = Math.floor(ctx.sampleRate);
    const buffer = ctx.createBuffer(1, length, ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < length; i++) data[i] = this.rng.range(-1, 1);
    this.noiseBuffer = buffer;
  }

  setSfxVolume(value: number): void {
    this.sfxVolume = clamp(value, 0, 1);
    if (this.sfxBus) this.sfxBus.gain.value = this.sfxVolume;
  }

  setMusicVolume(value: number): void {
    this.musicVolume = clamp(value, 0, 1);
    if (this.musicBus) this.musicBus.gain.value = this.musicVolume;
  }

  setMuted(muted: boolean): void {
    this.muted = muted;
    if (this.master && this.ctx) {
      this.master.gain.setTargetAtTime(muted ? 0 : 1, this.ctx.currentTime, 0.02);
    }
  }

  /* ------------------------------------------------------------ voices */

  private tone(
    frequency: number,
    duration: number,
    options: {
      type?: OscillatorType;
      gain?: number;
      sweepTo?: number;
      delay?: number;
      attack?: number;
      bus?: GainNode | null;
    } = {},
  ): void {
    const ctx = this.ctx;
    const bus = options.bus ?? this.sfxBus;
    if (!ctx || !bus) return;

    const {
      type = 'sine',
      gain = 0.3,
      sweepTo,
      delay = 0,
      attack = 0.005,
    } = options;
    const start = ctx.currentTime + delay;

    const osc = ctx.createOscillator();
    osc.type = type;
    osc.frequency.setValueAtTime(Math.max(20, frequency), start);
    if (sweepTo !== undefined) {
      osc.frequency.exponentialRampToValueAtTime(Math.max(20, sweepTo), start + duration);
    }

    const envelope = ctx.createGain();
    envelope.gain.setValueAtTime(0.0001, start);
    envelope.gain.exponentialRampToValueAtTime(Math.max(0.0002, gain), start + attack);
    envelope.gain.exponentialRampToValueAtTime(0.0001, start + duration);

    osc.connect(envelope);
    envelope.connect(bus);
    osc.start(start);
    osc.stop(start + duration + 0.05);
  }

  private noise(
    duration: number,
    options: {
      gain?: number;
      filterFrom?: number;
      filterTo?: number;
      delay?: number;
      type?: BiquadFilterType;
    } = {},
  ): void {
    const ctx = this.ctx;
    const bus = this.sfxBus;
    if (!ctx || !bus || !this.noiseBuffer) return;

    const { gain = 0.2, filterFrom = 2400, filterTo = 400, delay = 0, type = 'bandpass' } = options;
    const start = ctx.currentTime + delay;

    const source = ctx.createBufferSource();
    source.buffer = this.noiseBuffer;
    source.loop = true;

    const filter = ctx.createBiquadFilter();
    filter.type = type;
    filter.Q.value = 1.2;
    filter.frequency.setValueAtTime(filterFrom, start);
    filter.frequency.exponentialRampToValueAtTime(Math.max(40, filterTo), start + duration);

    const envelope = ctx.createGain();
    envelope.gain.setValueAtTime(0.0001, start);
    envelope.gain.exponentialRampToValueAtTime(Math.max(0.0002, gain), start + 0.004);
    envelope.gain.exponentialRampToValueAtTime(0.0001, start + duration);

    source.connect(filter);
    filter.connect(envelope);
    envelope.connect(bus);
    source.start(start);
    source.stop(start + duration + 0.05);
  }

  /* ------------------------------------------------------------- effects */

  /**
   * Plays a named effect. `intensity` (0..1) scales volume and brightness, so a
   * glancing bounce and a hard slam are audibly different.
   */
  play(name: SfxName, intensity = 0.6): void {
    if (!this.ctx || this.failed) return;
    const i = clamp(intensity, 0, 1);

    switch (name) {
      case 'putt':
        this.noise(0.09, { gain: 0.06 + 0.1 * i, filterFrom: 1200 + 2200 * i, filterTo: 300 });
        this.tone(140 + 200 * i, 0.16, { type: 'triangle', gain: 0.16 + 0.12 * i, sweepTo: 70 });
        break;

      case 'bounce':
        this.tone(180 + 620 * i, 0.1, { type: 'square', gain: 0.04 + 0.1 * i, sweepTo: 90 + 180 * i });
        this.noise(0.06, { gain: 0.03 + 0.07 * i, filterFrom: 900 + 2600 * i, filterTo: 400 });
        break;

      case 'bounceSoft':
        this.noise(0.12, { gain: 0.03 + 0.06 * i, filterFrom: 500 + 700 * i, filterTo: 160, type: 'lowpass' });
        break;

      case 'bumper':
        this.tone(520, 0.12, { type: 'square', gain: 0.1 + 0.1 * i, sweepTo: 1180 });
        this.tone(780, 0.16, { type: 'sine', gain: 0.06 + 0.06 * i, delay: 0.03, sweepTo: 1560 });
        break;

      case 'star': {
        // Rising major arpeggio.
        const base = 880;
        [1, 1.26, 1.5, 2].forEach((ratio, index) => {
          this.tone(base * ratio, 0.22, {
            type: 'triangle',
            gain: 0.13,
            delay: index * 0.045,
            attack: 0.004,
          });
        });
        break;
      }

      case 'sink': {
        const base = 392;
        [1, 1.25, 1.5, 2, 2.5].forEach((ratio, index) => {
          this.tone(base * ratio, 0.55, {
            type: 'sine',
            gain: 0.15,
            delay: index * 0.07,
            attack: 0.01,
          });
        });
        this.noise(0.3, { gain: 0.05, filterFrom: 400, filterTo: 120, type: 'lowpass' });
        break;
      }

      case 'death':
        this.tone(320, 0.5, { type: 'sawtooth', gain: 0.14, sweepTo: 55 });
        this.noise(0.45, { gain: 0.16, filterFrom: 1800, filterTo: 90, type: 'lowpass' });
        break;

      case 'portal':
        this.tone(240, 0.3, { type: 'sine', gain: 0.12, sweepTo: 1400 });
        this.tone(360, 0.34, { type: 'triangle', gain: 0.08, sweepTo: 1800, delay: 0.02 });
        break;

      case 'lipout':
        this.tone(700, 0.14, { type: 'sine', gain: 0.11, sweepTo: 420 });
        this.tone(520, 0.16, { type: 'sine', gain: 0.08, sweepTo: 300, delay: 0.05 });
        break;

      case 'click':
        this.tone(660, 0.06, { type: 'square', gain: 0.07 });
        break;

      case 'back':
        this.tone(420, 0.08, { type: 'square', gain: 0.06, sweepTo: 280 });
        break;

      case 'unlock':
        [523, 659, 784, 1047].forEach((f, index) => {
          this.tone(f, 0.4, { type: 'triangle', gain: 0.13, delay: index * 0.09 });
        });
        break;
    }
  }

  /** Bounce sound tuned by the surface that was hit. */
  playImpact(material: string, speed: number): void {
    const intensity = clamp(speed / 600, 0.08, 1);
    if (intensity < 0.1) return;
    switch (material) {
      case 'bouncy':
        this.play('bumper', intensity);
        break;
      case 'sand':
      case 'sticky':
        this.play('bounceSoft', intensity);
        break;
      case 'ice':
        this.play('bounce', intensity * 0.8);
        break;
      default:
        this.play('bounce', intensity);
    }
  }

  /* -------------------------------------------------------------- music */

  /**
   * A slow generative pad. Each tick plays one note from a fixed pentatonic
   * set, so it never resolves and never grates.
   */
  startMusic(): void {
    if (!this.ctx || this.musicTimer || this.failed) return;
    const scale = [196, 220, 261.63, 293.66, 329.63, 392, 440];
    this.musicTimer = setInterval(() => {
      if (!this.ctx || this.musicVolume <= 0.001) return;
      const note = scale[this.rng.int(0, scale.length - 1)]!;
      const octave = this.rng.bool(0.25) ? 2 : 1;
      this.tone(note * octave, 3.2, {
        type: 'sine',
        gain: 0.08,
        attack: 0.9,
        bus: this.musicBus,
      });
      // Every fourth tick adds a low root for a sense of pulse.
      if (this.musicStep % 4 === 0) {
        this.tone(98, 4.5, { type: 'sine', gain: 0.09, attack: 1.2, bus: this.musicBus });
      }
      this.musicStep++;
    }, 1600);
  }

  stopMusic(): void {
    if (this.musicTimer) {
      clearInterval(this.musicTimer);
      this.musicTimer = null;
    }
  }

  dispose(): void {
    this.stopMusic();
    try {
      void this.ctx?.close();
    } catch {
      // Already closed.
    }
    this.ctx = null;
  }
}
