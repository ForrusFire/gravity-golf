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
- **Change the course.** Roll through a violet switch pad to open a gate or drop
  a bridge; a bridge that has not appeared yet is outlined so you can plan
  around it. Crystal blocks shatter after a set number of hits, and the hit that
  breaks one lets you punch straight through.
- **Beat the clock.** A pad with a ring around it springs back when the ring
  runs out, so whatever it opened is only open while the ball is still in flight.
  Some vaults need every pad thrown at once.
- **One-way membranes** are marked with chevrons showing the direction you may
  cross. There is no going back through one.
- **Some cups are sealed.** A cup barred and drawn in amber will not accept the
  ball until you have banked every star, with a pip above it for each one still
  owed.
- **Take a shot back** whenever you like. Restarting the hole was already free,
  so undo costs nothing but the tedium of replaying the shots before it.
- **Race your best run.** Once you have finished a hole, a faint ghost replays
  your best attempt beside you.
- **Feats** reward style: a full orbit, a clean sink that touches nothing, a
  ricochet, a graze past a hazard. Collecting stars unlocks ball skins.

Keyboard: arrows aim and set power, `Space` shoots, `Z` takes back a shot,
`R` restarts, `G` toggles the gravity overlay, `C` recentres the camera,
`Esc` pauses.
Mouse/touch: scroll or pinch to zoom, right-drag or two-finger drag to pan.

## The course

48 holes across eight chapters, each introducing one idea at a time:

| Chapter | Introduces |
| --- | --- |
| 1. Orbital Basics | Aiming, gravity as a curve, slingshots, bumpers |
| 2. Deep Field | Suns, ice, sand, orbiting debris, pocket wells, solar wind |
| 3. Event Horizon | Black holes, repulsors, null and amplified gravity |
| 4. Machinery | Lifts, spinning arms, wormholes, conveyors, clockwork |
| 5. Singularity | Binary suns, nebulae, vortices, and everything at once |
| 6. Machine Shop | Switches, gates, drawbridges, breakable crystal, sealed cups |
| 7. Inversion | Reversed, amplified and cancelled gravity |
| 8. Clockwork | Timed gates, one-way membranes, multi-lock vaults |

Chapters 1–5 are pure ballistics: you read the field and pick a line. From
chapter 6 the hole itself has state, and a shot can change it — which is why
undo and the shot-by-shot board snapshot exist. Chapter 6's switches stay thrown
forever, which makes them a checklist; chapter 8's spring back, which turns the
route into a schedule.

Gravity fields are colour- and shape-coded, never colour alone: a **pink** zone
with chevrons rising through it reverses gravity, **amber** amplifies it, and
**green** damps it.

## Daily challenge and random holes

Beyond the campaign there is a generated hole a day — the same one for
everyone, derived from the date — plus an endless "random hole" button.

Generated holes are held to the same bar as handmade ones: a candidate is only
accepted if it validates *and* the solver can finish it, and its par is
whatever the solver actually needed rather than a guess. Verification takes a
second or two, so it runs in a Web Worker and the daily is cached after its
first generation.

Seven archetypes keep them recognisable as designed holes rather than noise —
slingshot, corridor, minefield, binary, pinball, vault (the blocker is a gate
you have to find the pad for) and glasshouse (break through, or go a long way
over). They react to the ball but never to the clock, which is what keeps the
verified solution exactly reproducible: a hole completable only at one instant
of a spinning arm's cycle is not a hole that was verified, so nothing moving and
no timed pads.

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

**Contact, again.** Sustained contact is not an impact. Gravity presses a
resting ball into the surface every step, so applying the impact response there
too multiplied the tangential loss hundreds of times a second and stopped a
rolling ball dead in a tenth of a second. Only genuine impacts bounce; rolling
is slowed by a per-second drag, which is also what finally makes ice, sand and
rock feel different underfoot.

**Presentation clocks.** The approach slow-motion scales how much simulated
time a real frame buys. The simulation itself is fixed-step, so slowing it down
changes what you see and never where the ball goes.

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
- **The search carries board state.** A switch thrown on stroke one is still
  thrown on stroke two, its spring-back deadline travels with it, and stars bank
  across strokes — without that, a sealed
  cup is unsolvable by construction and the completability proof would quietly
  exclude the holes that need it most. The beam also reserves slots for distinct
  board states, because the shot that opens a gate usually parks the ball
  somewhere worse and would otherwise be the first thing discarded.
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
