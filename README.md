# Gravity Golf

A 2D golf game played through orbital mechanics. Putt the ball across space,
slingshot it around planets, thread it past black holes, and sink it — in as
few strokes as you can.

Built in TypeScript on a custom physics engine, with no runtime dependencies.
It renders to a single HTML canvas and ships as a static site.

```
npm install
npm run dev      # play it at http://localhost:5173
```

## Playing

- **Aim** by dragging back from the ball and releasing, like a slingshot. The
  further you pull, the harder you hit.
- **Gravity bends every shot.** The faint field overlay shows which way space
  pulls at each point; the dot on each tick marks the direction of the pull.
- **Three stars** are hidden on every hole. They only count if the ball
  survives the shot that collected them.
- **Arrive slowly.** A ball that reaches the cup too fast rims out.
- **Hazards** — suns, black holes, spikes — destroy the ball and cost a penalty
  stroke. Anything lethal is drawn with a red serrated corona.

Keyboard: arrows aim and set power, `Space` shoots, `R` restarts,
`G` toggles the gravity overlay, `C` recentres the camera, `Esc` pauses.
Mouse/touch: scroll or pinch to zoom, right-drag or two-finger drag to pan.

## The course

30 holes across five chapters, each introducing one idea at a time:

| Chapter | Introduces |
| --- | --- |
| 1. Orbital Basics | Aiming, gravity as a curve, slingshots, bumpers |
| 2. Deep Field | Suns, ice, sand, orbiting debris, pocket wells, solar wind |
| 3. Event Horizon | Black holes, repulsors, null and amplified gravity |
| 4. Machinery | Lifts, spinning arms, wormholes, conveyors, clockwork |
| 5. Singularity | Binary suns, nebulae, vortices, and everything at once |

## Daily challenge and random holes

Beyond the campaign there is a generated hole a day — the same one for
everyone, derived from the date — plus an endless "random hole" button.

Generated holes are held to the same bar as handmade ones: a candidate is only
accepted if it validates *and* the solver can finish it, and its par is
whatever the solver actually needed rather than a guess. Verification takes a
second or two, so it runs in a Web Worker and the daily is cached after its
first generation. Generated holes use only static bodies, which keeps the
verified solution exactly reproducible — a hole that is completable only at one
instant of a spinning arm's cycle is not a hole that was verified.

## How it is put together

```
src/
  core/       vector and scalar maths, seeded PRNG
  physics/    shapes, world stepper, trajectory prediction  (no DOM)
  game/       level data, validation, play session, progress, solver
  render/     canvas renderer, camera, particles, starfield, palette
  audio/      procedural WebAudio synthesis (no audio files)
  ui/         DOM overlays: HUD, menus, input handling
```

`physics/` and most of `game/` are free of browser APIs, which is what makes
the simulation testable in plain Node and identical between the live game and
the aiming preview.

**Integration.** The stepper uses kick-drift-kick leapfrog rather than Euler,
because Euler pumps energy into an orbit and makes it spiral outward at any
step size worth running in a game. Substeps scale with speed so a fast ball
cannot tunnel through a thin wall.

**Contact.** Collisions use a speculative contact skin. Without it, a ball
settling on a surface alternates between "pushed out" and "falling" forever and
never registers as at rest.

**Prediction.** The aiming preview runs the real integrator on a throwaway copy
of the world, so what it draws cannot drift from what the ball actually does.

## Tests

```
npm run verify     # typecheck + unit tests + production build
npm test           # unit tests only
npm run test:e2e   # Playwright, desktop and mobile viewports
```

Two of the suites are worth calling out:

- **Every hole is proven completable.** A beam search finds a solution within
  par for each level, and that exact sequence of shots is then replayed through
  a real play session and must sink. The search shares the game's integrator,
  its timestep and its settled tee, so a solution is a shot list a player could
  actually take. This matters more than it sounds: gravity slingshots are
  chaotic, and when the search ran at a coarser step only half its solutions
  survived being replayed at gameplay precision.
- **Every star is proven collectable.** Chapters unlock on star totals, so a
  star no shot can reach could strand a player short of the next chapter.
- **Level validation** rejects buried tees, holes sunk inside a body, stars
  outside the play area and duplicate ids — the authoring mistakes that are
  invisible until someone plays the hole.

Useful during development:

```
npm run preview                     # needed by both tools below
node tools/capture.mjs captures     # screenshot every hole
node tools/perf.mjs                 # per-frame CPU cost on the busiest holes
```

`tools/perf.mjs` instruments `update()` and `draw()` rather than timing frame
deltas: in a headless, software-rasterised container the frame delta mostly
measures the compositor, not the game.

## Offline

The production build emits a service worker that precaches every file it
produced, so the game installs as a PWA and plays with no network at all.
The precache list is generated from the bundle rather than hand-written, and
the cache name is derived from a hash of the contents, so a new build always
replaces the old cache instead of serving a mix of the two.

## Accessibility

- Full keyboard play, and menus that trap focus and are labelled as dialogs.
- A high-contrast palette, and a reduced-motion setting.
- Lethal objects are marked by shape as well as colour.
- The trajectory preview length is adjustable, including off.

## `legacy-cpp/`

The repository began as a C++/SFML prototype with a partial physics engine and
an empty `main`. It is kept there for reference; nothing in the current game
depends on it.
