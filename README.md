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
- **Some cups have a mouth.** A wedge drawn on the cup is the side you have to
  arrive from; it will not funnel you in from anywhere else.
- **Some cups move**, along a dashed track that shows the whole lap. What counts
  is your speed *relative to the cup* — parking on the rails and letting it drive
  over you is a lip-out, not a hole in one.
- **Hazards** — suns, black holes, spikes — destroy the ball and cost a penalty
  stroke. Anything lethal is drawn with a red serrated corona.
- **Change the course.** Roll through a violet switch pad to open a gate or drop
  a bridge; a bridge that has not appeared yet is outlined so you can plan
  around it. Crystal blocks shatter after a set number of hits, and the hit that
  breaks one lets you punch straight through.
- **Beat the clock.** A pad with a ring around it springs back when the ring
  runs out, so whatever it opened is only open while the ball is still in flight.
  Some vaults need every pad thrown at once.
- **Toll gates** carry star pips and lift once you have collected that many. On
  those holes the stars stop being a bonus and become the road.
- **Pulsing walls** blink in and out on a clock of their own, and nothing you do
  changes them. Each wears a ring counting down to its next change — amber while
  it is solid, green while it is gone.
- **Whole fields can blink too** — a wind that gusts, a hazard field that is only
  deadly half the time, a gravity well that switches off. They fade rather than
  vanish, so you can plan around something that is not currently there.
- **One-way membranes** are marked with chevrons showing the direction you may
  cross. There is no going back through one.
- **Boost rings** fire the ball out along their arrows at their own fixed speed,
  whatever speed it arrived at — so they are the one thing on a hole you aim
  *at* rather than *with*.
- **Some cups are sealed.** A cup barred and drawn in amber will not accept the
  ball until you have banked every star, with a pip above it for each one still
  owed.
- **Stuck? Ask for a line.** The 💡 button runs the solver from wherever the ball
  is standing and draws a shot worth playing, setting your aim to match. The hole
  still counts and still earns its medal — a hinted run just earns no style feats
  and is not kept as your ghost.
- **Take a shot back** whenever you like. Restarting the hole was already free,
  so undo costs nothing but the tedium of replaying the shots before it.
- **Race your best run.** Once you have finished a hole, a faint ghost replays
  your best attempt beside you.
- **Feats** reward style: a full orbit, a clean sink that touches nothing, a
  ricochet, a graze past a hazard. Collecting stars unlocks ball skins.

Keyboard: arrows aim and set power, `Space` shoots, `Z` takes back a shot,
`H` asks for a line, `R` restarts, `G` toggles the gravity overlay,
`C` recentres the camera, `Esc` pauses.
Mouse/touch: scroll or pinch to zoom, right-drag or two-finger drag to pan.

## The course

84 holes across fourteen chapters, each introducing one idea at a time:

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
| 9. Launch Control | Boost rings — the route becomes a relay |
| 10. Moving Targets | The cup itself travels; the puzzle becomes *when* |
| 11. Rhythm | Barriers blinking on a clock that waits for nobody |
| 12. Approach | Cups that only take the ball from one direction |
| 13. Toll Roads | Stars stop being a bonus and become the road |
| 14. Tides | The fields themselves come and go |

Finish every hole and the course closes with a card of the whole run: total
strokes against par, holes under par, stars, feats and time.

Chapters 1–5 are pure ballistics: you read the field and pick a line. From
chapter 6 the hole itself has state, and a shot can change it — which is why
undo and the shot-by-shot board snapshot exist. Chapter 6's switches stay thrown
forever, which makes them a checklist; chapter 8's spring back, which turns the
route into a schedule.

Gravity fields are colour- and shape-coded, never colour alone: a **pink** zone
with chevrons rising through it reverses gravity, **amber** amplifies it, and
**green** damps it.

## Rounds

**Round of the day** deals nine campaign holes — the same nine for everybody,
derived from the date, so cards are worth comparing. It and runs them back to back as one
continuous score — no panel between holes, one card at the end, and a personal
best across every round you have played. The holes are drawn deterministically
from a seed and walk up through the chapters, so a round ramps the way the
campaign does instead of opening on a finale. Nine holes drawn uniformly would
mostly be late ones, since most chapters are late chapters.

Campaign holes rather than generated ones, for two reasons: nine generated holes
would mean fifteen seconds of verification before the first shot, and campaign
holes are already proven completable, so a shared round is a fair round. A round
shares as `?round=<seed>` and scores as a whole — its holes deliberately leave
the campaign's per-hole records alone, since a hole you happened to draw badly
should not overwrite the best you ever played it.

## Sharing a hole

Every hole has a link. The address bar tracks whatever you are playing, and
**Share this hole** on the pause and results screens copies it:

```
https://…/?hole=c7-4      a campaign hole
https://…/?seed=1837291  a generated hole, rebuilt from its seed
https://…/?round=4242    a nine-hole round
```

Seed links work because generation is deterministic *and* verified — the same
seed yields the same hole on every device, and it was proven completable before
it was ever shown, so a link cannot hand someone an impossible course. That
covers the daily too: it shares as its seed rather than as a date, which would
be a different hole tomorrow.

A link opens its hole immediately and ignores the star gates. Someone who
followed a link to hole 41 was sent there on purpose, and refusing them because
of their own save file would make every shared link a dead end for most of the
people who click it. Links are untrusted input and are parsed as such; anything
unrecognised falls through to the title screen. They work offline, which needs
the service worker's navigation fallback to ignore the query string — there is a
test for that, because the failure mode is every shared link being a dead page.

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

The per-level gates — replay, star reachability and forgiveness — are the slowest
thing here and grow with every chapter. Vitest parallelises across *files*, not
within one, so a single file looping over every hole pinned one worker at the
tail while the others sat idle. They are sharded across `gates-*.test.ts` instead,
interleaved so each slice gets a mix of cheap and expensive holes. On a
four-core box that took the suite from 377s to 331s; the remainder is simply
CPU-bound, so the honest way to make it faster is to do less work, not to
schedule it better.

Three of the suites are worth calling out:

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
- **The hint is held to the same standard as the verifier.** It runs the same
  search at the same timestep, from wherever the ball is standing, so the aim and
  power it hands over are exactly the ones that produced the line it drew. A
  cheaper timestep would diverge within one shot and the hint would be a lie. It
  is the only search allowed a wall-clock budget, because a time-bounded search
  gives different answers on different machines and "this hole is completable"
  must not depend on how fast the box was.
- **Star reachability carries collected stars forward.** It used to clear them
  between shots, which looked safer — "having banked one must not hide it from a
  later branch" — but became wrong the moment collecting a star could open a
  gate. Every star behind a toll was reported unreachable. Clearing was never
  needed anyway: the seen-set accumulates across the whole search.
- **No hole can be won by accident.** A blind sweep of opening shots must sink
  each hole (outside the tutorial chapter) less than 7% of the time. This catches
  what completability cannot: three holes had shipped with the cup sitting at the
  natural resting point of the course, so anything that settled simply rolled in
  — one scored on a quarter of all blind shots. The solver was perfectly happy,
  because "can this be finished" was never the question.
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
