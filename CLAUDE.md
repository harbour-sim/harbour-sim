# Harbour Sim — mooring simulator

Rust + macroquad 0.4.15 + Rapier 2D, compiled to WebAssembly and served via
GitHub Pages. Top-down simulator of a small vessel docking in a harbour under
engine — currently a proof of concept: the boat lies alongside a fixed quay
under adjustable wind and current. The modeled vessel is, for now, always a
small cruising sailboat (sails furled/down throughout — no sail force
modeled; wind is purely an external load, same as it would be on any small
boat lying to it). Supporting other small-vessel types alongside the
sailboat (see Roadmap) is the agreed direction, not yet built — nothing in
`sim-core` or the renderer should be read as a permanent sailboat-only
decision. The goal is mooring manoeuvres with placeable ropes (bow/stern
lines, springs) under different conditions. Ropes are in (2026-08-20 —
see Mooring lines below); scenarios and scoring are future work.

Boilerplate and pipeline are copied from **dannyrhubarb/pegasus** (2026-08) —
when in doubt about a pattern or a CI gotcha, that repo's CLAUDE.md is the
richer reference; anything imported here follows the same rules.

> **Keep this file current.** Update CLAUDE.md as part of every commit that
> changes architecture, adds a system, renames constants, fixes a gotcha, or
> reveals a lesson. Don't batch it up.

## Build & deploy
```bash
cargo build               # native dev build (opens a window when run)
cargo test --workspace    # --workspace is required or sim-core's tests are skipped
cargo clippy --workspace --all-targets -- -D warnings   # CI gate
cargo check --target wasm32-unknown-unknown             # what actually deploys
```
Deploy is automatic: any push to `main` triggers `.github/workflows/deploy.yml`.
One-time repo setup: **Settings → Pages → Source = "GitHub Actions"** (do NOT
switch it to the `gh-pages` branch — that bypasses the pipeline and serves the
branch with Jekyll defaults).

### Deploy pipeline & PR previews (inherited from Pegasus)
The published site lives on the **`gh-pages` state branch**: the `main` build
at the root, one per-PR preview in `pr-<n>/` (served at
`https://<owner>.github.io/harbour-sim/pr-<n>/` — asset URLs in `index.html`
are relative, which is what makes subdirectory serving work). Four workflows,
sharing two composite actions (`.github/actions/build-site` = wasm build +
icons + revision injection; `.github/actions/sync-pages-branch` = commit into
`gh-pages` with a push-retry loop for concurrent deploys):
- `deploy.yml` (**Main deploy**, push to `main`, also `workflow_run` on
  **Fast forward** completing — see Fast-forward merges below for why):
  build → sync branch root (live `pr-*/` previews are kept). **Deploys the
  main TIP at run time, not the pushed sha** (gotcha, seen live
  2026-08-03): push runs can start out of order — the run for an older
  commit sat queued ~11 min, started 21 s after the newer commit's run,
  cancelled it via `cancel-in-progress`, and synced the OLD build over the
  site root (the About-page merge vanished from the live site). Checking
  out `origin/main` at run start makes straggler runs redeploy current
  content instead, so run ordering can't regress the site — the same
  property that lets a `workflow_run`-triggered build deploy correctly
  without caring what triggered it.
- `preview-deploy.yml` (**Preview deploy**, PR opened/synchronize/reopened):
  build (revision label `<head-sha>-pr-<n>`) → sync `pr-<n>/` → sticky PR
  comment (`<!-- preview-env -->` marker) with the preview URL. Skipped for
  fork PRs (read-only token).
- `preview-teardown.yml` (**Preview teardown**, PR closed): delete `pr-<n>/`.
- `publish-pages.yml` (**Publish Pages**): the *only* workflow that calls
  `deploy-pages`. Triggered by `workflow_run` on the three above (must match
  their `name:` strings exactly — a workflow that pushes to `gh-pages`
  without being listed here lands on the branch and is never deployed).
  **Gotcha (from Pegasus)**: the auto-created `github-pages` environment only
  allows deployments from `main`, so PR-triggered workflows can't deploy
  directly; `workflow_run` workflows execute from the default branch, which
  passes the protection. Also: pushes made with `GITHUB_TOKEN` don't trigger
  `push` workflows (recursion guard), so an `on: push: branches: [gh-pages]`
  publisher would never fire — `workflow_run` is load-bearing. The Pages API
  intermittently rejects rapid-succession deployments, so the deploy step
  retries once after 30 s.
  **Gotcha (learned here, 2026-08-02)**: `workflow_run` triggers only fire
  if the workflow file exists on the DEFAULT branch. On this then-new repo,
  `main` was an empty root while the boilerplate PR was open — preview
  deploys completed but nothing ever published, and the Pages site 404'd.
  Publish Pages only checks out `gh-pages` (no game code), so its copy was
  committed straight to `main` ahead of the first merge; **keep the `main`
  and feature-branch copies identical** so merges are a no-op for it. It
  also carries a `workflow_dispatch` escape hatch (the job's `if:` passes it
  explicitly) to republish the current `gh-pages` state on demand.

`ci.yml` runs on PRs: wasm `cargo check`, clippy `-D warnings`, tests.

### Fast-forward merges (sequoia-pgp/fast-forward)
Two workflows wrap the marketplace **Fast Forward Merge** action so PRs can
land on `main` with `git merge --ff-only` semantics (linear history, commit
shas preserved — matches the curate-then-rebase-merge rule under Git
workflow): `fast-forward-check.yml` runs on PR open/reopen/synchronize
targeting `main` with `merge: false` and comments when the action's result is
non-zero (`comment: on-error` — usually because a rebase on `main` is due,
but also the action's own infrastructure errors, e.g. a failed fetch/clone,
read the same in the comment); `fast-forward.yml` runs on an `issue_comment`
containing `/fast-forward` on a still-**open** PR (`github.event.issue.state
== 'open'`, a best-effort filter against commenting on an already-closed
PR — not a substitute for the action's own authorization check, see below)
and does the merge. Both pin `sequoia-pgp/fast-forward` to its `v1.0.0`
commit sha rather than `@v1`, since **`v1` is a mutable branch, not an
immutable tag** (`git ls-remote` confirms it — only `v1.0.0` is a real tag),
and the merge step hands the action a repo-write token. The action's own
README states it checks that the commenter is authorized to push before
merging, which is why this doesn't also split the workflow into a
GITHUB_TOKEN preflight job before the token-bearing step — that would
duplicate a check the action already makes, for a project this size.
Gotcha: `issue_comment` workflows execute the file from the DEFAULT branch,
so the `/fast-forward` trigger only works once the workflow is on `main`
(same class of gotcha as `workflow_run` above, under Publish Pages).

**Deploying after a fast-forward merge, without a PAT.** The merge push in
`fast-forward.yml` uses the plain default `GITHUB_TOKEN` (no secret to
provision or rotate) — deliberately, even though such pushes are subject to
the recursion guard already documented for Publish Pages (`GITHUB_TOKEN`
pushes don't trigger `push` workflows), which would otherwise mean a
fast-forward merge never fires `deploy.yml`. The fix reuses the exact
mechanism Publish Pages already depends on for the same reason: `deploy.yml`
also triggers on `workflow_run` for **Fast forward** completing (gated
`github.event.workflow_run.conclusion == 'success'`) instead of relying on
the push event. `workflow_run` fires on the upstream workflow's own
completion, not on what its `GITHUB_TOKEN` did internally, and
`fast-forward.yml`'s trigger — a human's PR comment — was never
`GITHUB_TOKEN`-authored to begin with, so the chain isn't blocked. An
earlier version of this design used a `FAST_FORWARD_PAT` secret instead (PAT
pushes aren't subject to the recursion guard either) — dropped because it
tied deploys to a specific person's account (rotation, offboarding, must
itself satisfy `main`'s branch protection) for a problem `workflow_run`
already solves structurally, the same way Publish Pages does.

## Project structure
- `sim-core/` — the **`harbour-sim-core` library crate** (workspace member):
  the whole deterministic half. **Nothing in it may depend on macroquad or
  any nondeterminism**; it uses `glam` (pinned to the version macroquad
  0.4.15 re-exports, so `Vec2` unifies across the boundary) + `rapier2d`.
  - `sim-core/src/sim.rs` — `Sim` (Rapier world: the marina's boundary
    polyline, jetties, mooring poles, the moored fleet — DYNAMIC bodies
    on their own mooring lines since 2026-08-20 — plus the boat),
    `Env` (wind/current), all physics constants, harbour geometry
    constants, unit tests.
  - `sim-core/src/keel.rs` — `KeelProfile` (piecewise-linear underwater
    lateral-area-per-length curve along the hull) and `KeelDerived` (area,
    centre of lateral resistance, yaw damping integral — derived from the
    curve by integration, see Simulation model below).
  - `sim-core/src/line.rs` — mooring lines (2026-08-20): the nylon
    load-elongation curve, `Fairlead` (deck positions read off
    `HULL_PTS`), `Anchor`, `Line`/`LineState`, the crew's `LineCommand`
    orders, and the handling constants (reach, pass speed, haul/pay
    rates). Pure functions + a command applier; the forces themselves
    are applied by `sim.rs`'s `tick`. See Mooring lines below.
  - `sim-core/src/boat.rs` — `BoatDesign` (2026-08-04): the parameter
    bundle the keel editor edits and `Sim::new_with_design` consumes — a
    `KeelProfile`, a `RudderDesign` (blade position/dimensions/end-plate
    flag, per-preset since 2026-08-04 — see the Rudder bullet under
    Simulation model), and `displacement_kg`. Four presets named after REAL
    boats (published specs, sources and the shared-hull caveat in
    `docs/reference-boats.md`): `hallberg_rassy_38()` (default — fin +
    skeg middle configuration, 8.5 t), `oday_39()` (fin + spade, 8.165 t
    — the rudder-sizing anchor: its blade is the one with real published
    dimensions),
    `elan_impression_394()` (2026-08-04, modern shallow-bodied cruiser,
    8.0 t — smallest lateral plane and least yaw damping of the four;
    its spec pages 403'd so the figures were triangulated from search
    excerpts and validated by the D/L=222 consistency check, see
    reference-boats.md), and `alajuela_38()` (heavy full keel, 11.8 t).
    The curve's unit makes the
    naming honest: area-per-length at a station IS local draught (m), so
    presets are capped at each boat's real draft (unit-tested). NOT a
    ship-type abstraction — hull outline/windage/engine stay the single
    shared sailboat (see Roadmap); a design varies keel, rudder and
    weight on it.
- `src/main.rs` — macroquad frontend: input, fixed-timestep loop with render
  interpolation, top-down rendering (water/ripples, the Hinsholmen scenery —
  grass/tree road shore with quay apron NW, wooded rocky hill shore SE,
  plank pontoons, mooring poles, moored boats drawn at the LIVE poses
  sim-core gives them, with their crossed stern lines and breast lines
  drawn by the same rope path as the player's (they are the same
  `Line`s), rounded silt-ringed
  bay head NE, open sea SW with a skerry chain at the world's edge — and
  the player's boat), HUD
  (wind/current dials, throttle/rudder sliders, SOG readout, key help),
  keel design editor overlay (`E`). All static scenery (jetty list, poles,
  moored fleet, both shore polylines, world bounds) is fetched from
  sim-core ONCE before the loop; curved shores render via
  `offset_polyline` + `draw_strip` (quad strips between polylines), and
  everything repeated (boats, poles, trees, ripples) is visibility-culled
  per frame against the camera circle — the marina is ~400 m long and
  only a stretch is ever on screen. Scenery scatter (trees) uses the same
  deterministic hash idiom as the ripples, minus the time term — cosmetic
  only, like everything render-side.
- `src/mooring.rs` — the LINES mode frontend (2026-08-20): mode state,
  the one-grab-at-a-time gesture machine (lead a line / haul), world-space
  hit-testing for fairleads, anchors and ropes, and all the rope drawing
  (slack bights, load colouring, a line still going ashore). Turns
  gestures into `LineCommand`s and hands them to `tick` through
  `InputState` — it never touches physics. See Frontend conventions. The
  crew's pass speed is a setting, not a mooring gesture — see
  `src/settings.rs` below.
- `src/settings.rs` — the **settings menu** (2026-08-20): an overlay for
  the knobs that configure the game rather than sail the boat. Settings
  do NOT live in the play HUD (owner call): the HUD's job is the boat,
  and something you set once and forget has no business holding
  permanent space on a phone screen. Follows the keel editor's overlay
  pattern — freezes the game while open (input AND the physics tick),
  which is what lets it reuse keys and take presses without fighting the
  HUD's touch claims. Opened by the gear button (bottom row, left of
  LINES) or **O**; closed by CLOSE, Escape, O again, or a tap on the
  dimmed scrim. Adding a setting is a row in `ROWS` plus an arm in
  both `value` and `value_mut` (`draw` reads through the former). Currently holds three, all of
  them the CREW rather than the world: LINE HANDLING (how fast a line
  goes ashore), HAUL FORCE (what a pair of hands can pull, in kilos) and
  LINE REACH (how far a line can be got ashore). The frame loop bundles
  them into `InputState::crew` (so they still ride the recorded input
  stream — see Mooring lines).
  **Gotcha (found live, 2026-08-20)**: the key press or button tap that
  OPENS an overlay is still "pressed" when that overlay's own input runs
  later in the same frame, so it closed itself instantly and the menu
  never appeared. `SettingsMenu::open` sets a `just_opened` flag and the
  first `update` returns without reading input at all. Any future
  self-toggling overlay needs the same guard — the keel editor dodges it
  only because E is handled in exactly one place.
  **Gotcha (CodeRabbit review, 2026-08-21)**: the mouse and touch paths
  share one `grab`, and `simulate_mouse_with_touch(false)` means the
  mouse button reads permanently UP on a phone — so the mouse branch's
  `else { grab = None }` fired every frame, ahead of the touch loop, and
  the slider took the value under the tap and then ignored the drag.
  The release is gated on `self.touch.is_none()`. The keel editor dodges
  this by keeping `mouse_on_weight` and `weight_touch` separate; a
  shared claim needs the gate.
- `src/keel_editor.rs` — in-app editor for `BoatDesign`: drag a fixed-grid
  bar chart to paint the underwater area distribution, drag a displacement
  slider (4–14 t range bracketing the reference boats, 100 kg steps;
  Up/Down keys for keyboard parity), four preset buttons named after the
  real boats in `boat.rs` — HR 38 [D] / O'Day 39 [F] / Elan 394 [G] /
  Alajuela 38 [L],
  each loading curve AND weight together (D and the arrows are game keys,
  but safe to reuse because the editor freezes all game input while open)
  — live-derived readout (since 2026-08-04 led by the LWL, read from
  `sim::waterline_extent` on the edited curve — paint the ends dry and
  the waterline shortens live; the wet span is also drawn bright on the
  canvas baseline; note the fixed 0.25 m grid widens resampled
  zero-crossings slightly, so a preset's readout LWL can sit ~0.3 m
  above its published figure — the offset exists ONLY in the editor's
  grid resample: the preset structs themselves are exact, pinned by
  `waterline_extent_reads_each_presets_real_lwl`, and the readout
  describes the curve as edited, which is what Apply hands the
  physics), Apply
  rebuilds the sim in place via
  `Sim::new_continuing` (keeping pose, velocity, and engine spool). The slider has its own mouse/touch claim
  (`mouse_on_weight`/`weight_touch`, same one-claim-per-control +
  recycled-id rules as the HUD dials) so a drag that starts on the track
  can't start painting bars when it sweeps across the curve canvas.
  Frontend-only — hands a plain `BoatDesign` value to
  sim-core, never reaches into physics directly. Also draws a
  non-paintable rudder marker (2026-08-03; since 2026-08-04 from the
  LOADED DESIGN's own `RudderDesign`, so it moves/resizes per preset and
  is carried through Apply — the same values the physics uses, not a
  separate guess) — stacked BELOW whatever the curve is at
  that station (not from the baseline) so it reads as an appendage
  hanging off the hull rather than overlapping the editable area; needed
  once the rudder stopped being part of the paintable profile (see the
  Keel profile bullet under Simulation model). Also draws a **CG marker**
  (2026-08-04, green, label at the canvas bottom so it can't collide
  with CLR's top label): the boat's centre of mass via
  `sim::hull_com_x()` — the `HULL_PTS` polygon centroid, i.e. the same
  COM Rapier derives from the uniform hull spread, unit-tested against
  Rapier's own `local_center_of_mass` so the marker can't drift from the
  physics. Deliberately separate from the CLR marker: CG follows only
  the hull outline (x ≈ −0.46 m, fixed) while CLR follows the painted
  keel curve (HR 38 ≈ −0.72 m, Alajuela ≈ −1.5 m), and their gap is the
  lever arm that turns sway force into yaw moment. When adjustable mass
  distribution lands (Roadmap), the marker must read the design instead
  of this constant.
- `src/wake.rs` — the boat's **churned-water trail** (2026-08-20).
  Cosmetic and frontend-only, like the ripples and the prop-wash
  streaks: it READS `Sim` (`boat_pose`/`boat_vel`/`engine`) and the
  active `BoatDesign` and feeds nothing back. A ring of `Eddy` patches
  (position, imparted water motion, swirl, radius, birth strength,
  aeration) shed from six sources — five stations spread over the
  design's waterline plus the prop — advected, spread and faded each
  frame. What makes it more than a particle effect is that the shed
  strength at a station is the SAME strip quantity `tick` integrates
  for cross-flow drag, `V(x) = v_lat + w·(x − com)`, squared: a hull
  with a sideways component is a separated bluff body and churns far
  harder than one running straight, which is exactly the behaviour
  asked for and what
  `slipping_through_a_turn_churns_far_harder_than_running_straight`
  pins (per metre sailed AND per patch shed). Straight running still
  sheds (boundary layer, `AXIAL_WEIGHT`, alternating quarters) and the
  prop race sheds its own aerated white water (`Eddy::foam`) distinct
  from the hull's smooth boil — the live slipstream streaks in
  `main.rs` are what LEAVES the blade, these are what it leaves lying
  on the water. **Everything is water-relative**: strengths come from
  `boat_vel − current_vel` and every eddy is advected by the current
  for its whole life, so a wake laid across a stream sets down-current
  while it fades and a boat drifting WITH the current stirs nothing
  (`shed_turbulence_is_carried_by_the_current`). Frozen with the sim
  while the keel editor is open (the `update` call lives inside the
  same `!editor.active` block as `sim.tick`); cleared by the R-reset
  (a surviving trail would streak across the marina to the spawn), NOT
  by the editor's Apply — that boat carries on from where it was.
  **Gotcha (measured 2026-08-20, three passes at the look)**: a trail
  of spreading translucent patches gets BRIGHTER with distance unless
  the dilution exponent beats the overlap. Patch count over a point
  goes as their radius (they line up along a track), so a `1/r`
  dilution composites flat and a `1/r²` one tapers — the first pass
  used neither and whited out into a solid cloud. Related: birth
  strength drives spawn RATE and patch SIZE but deliberately NOT
  opacity — coverage and texture are how turbulence actually reads on
  water, and folding strength in a third time crushed the faint end
  out of existence. Verified in headless Chromium (straight run, hard
  turn, and a 2.5 m/s beam current) rather than assumed; the trail
  costs ~25% of frame time under swiftshader's SOFTWARE rasteriser at
  the `EDDY_CAP` worst case, all of it fill rate from the translucent
  patches (skipping the swirl arcs on faint eddies measured as no
  change at all) — negligible on any real GPU, but that is the knob if
  it ever needs one.
- `index.html` — web wrapper: boot guard (standalone script ahead of the
  bundle that paints script errors on screen), loading overlay,
  `__GIT_REVISION__` placeholder (deploy-time sed → wasm `?v=` cache-buster),
  and the **About overlay** (2026-08-03, the Pegasus scr-about sized to this
  repo): a small ⓘ button — bottom-LEFT corner; the in-canvas help text
  indents 40 css px past it (`help_x` in main.rs — harmless dead space in
  native builds, which have no HTML layer) — that
  opens an HTML panel with the build revision **linked to its commit**, the
  build time (`__BUILD_TIME__`, a second deploy-time sed in `build-site`,
  ISO-8601 UTC re-rendered by `fmtDateTime` — the Pegasus timezone-derived
  region-locale formatter, ported verbatim with the memo key renamed to
  `harbour_sim_date_locale`; rendered on each overlay OPEN, not at boot,
  because the ~200 ms region scan is deferred off the boot path) and, on
  a preview
  deployment, a **link to the PR** (number parsed from the revision label's
  `-pr-<n>` suffix, `pr-<n>/` path as fallback), plus a static **link to
  the project's GitHub page** (owner request, PR #11 review). HTML, not
  in-canvas,
  because the rows are real links. The ⓘ and Close controls are native
  `<button>`s, the card carries `role="dialog"`/`aria-modal`/
  `aria-labelledby`, and focus moves to Close on open / back to ⓘ on close
  (CodeRabbit review, PR #11; the game's keys need CANVAS focus either way
  — miniquad wires onkeydown to the canvas — so this matches the page-load
  focus state). The button/overlay swallow
  `mousedown`/`touchstart`/`pointerdown` (stopPropagation, no
  preventDefault — the Pegasus menu rule) so a tap never doubles as a
  canvas press; local dev keeps the placeholders and shows
  "dev (local build)". The overlay does NOT pause the sim (there's no
  pause export yet — the boat just keeps drifting behind it, harmless).
- `mq_js_bundle.js` — **vendored** miniquad/quad-snd JS loader (same build as
  Pegasus). Pinned in-repo so deploys don't depend on a third-party host.
  **Gotcha**: it declares top-level globals (`const canvas`, `var gl`,
  `wasm_exports`, `function load`, …) that share the page's global scope —
  redeclaring any of them in `index.html`'s inline script is a SyntaxError
  that silently kills the whole inline script. Pick distinct names.
  **Gotcha (2026-08-03)**: `canvas.onmouseup` (and `onmousedown`/
  `onmousemove`) is wired to the canvas element only, not `window`. Click a
  draggable HUD control (e.g. a wind/current dial), drag the pointer outside
  the *browser window*, and release there: no `mouseup` DOM event fires
  anywhere, so miniquad's button-down state sticks `true` forever and
  `is_mouse_button_down` never goes false — the drag claim in `main.rs`
  stays "grabbed" even after the pointer returns. Fixed in `index.html` (not
  here, to keep this file identical to upstream) with **Pointer Capture**:
  `canvas.setPointerCapture()` on `pointerdown` (mouse pointers only — touch
  is untouched, see Touch controls below) makes the browser keep delivering
  `pointerup`/`pointermove` to `canvas` even while the pointer is outside the
  window, so the real release still reaches us; that forwards a synthetic
  `wasm_exports.mouse_up(x, y, button)` for all three buttons. Verified in
  headless Chromium via Playwright (`pointerup` fired with real off-viewport
  coordinates once captured) — **first tried `document`'s `mouseleave` as
  the release signal and it does not fire on window-exit in any browser
  tested**; a `mouseout`/`blur` fallback (the standard `relatedTarget ===
  null` trick) is kept for browsers without Pointer Capture, but Pointer
  Capture is the real fix — don't remove it and rely on the fallback alone.
- `icon.svg` — source for the PNG icons rendered at deploy time
  (`rsvg-convert` in build-site).
- `docs/reference-boats.md` — published specs (with sources) for the real
  boats the `BoatDesign` presets are named after, the derived numbers each
  preset produces, and exactly what the sim does / does not take from each
  boat (the shared-hull caveat).
- `rust-toolchain.toml` — **pins the Rust toolchain (1.94.1)**. The first
  preview deploy failed because the runner's newer preinstalled stable broke
  the wasm RELEASE link (`rust-lld: undefined symbol: console_log/now/...` —
  miniquad's JS imports stopped becoming implicit wasm imports). Beware:
  `cargo check --target wasm32-unknown-unknown` does NOT link, so CI's check
  job stays green while the deploy build fails. Upgrade the pin deliberately
  with a full wasm build + browser smoke test. (Pegasus has no pin and will
  likely hit the same wall on its next deploy.)

## Simulation model (sim-core/src/sim.rs)

Top-down 2D, world units are metres, y = north (up on screen), x = east.
No gravity — the projected-away vertical is replaced by hydrodynamic drag,
wind load, propulsion, and quay contact. Fixed timestep `PHYSICS_DT =
1/120 s`, advanced ONLY by `Sim::tick(&Env, &InputState)`; the frontend runs
an accumulator with render interpolation (`lerp` + shortest-path angle lerp)
like Pegasus.

- **Harbour** (2026-08-04, modeled on Hinsholmen marina, Långedrag,
  Gothenburg — owner-supplied aerial photos; mirrored in both axes
  2026-08-05 on owner request, so the channel's concave side faces UP
  and the dock row sits on the lower/outer shore): the whole marina is
  **GENERATED, not hand-placed** — a channel whose road (dock-carrying,
  SE/outer) shore is chord-marched head→sea from `ROAD_HEAD_ROOT` at
  `SHORE_BEARING_HEAD_DEG` (207°, SSW), bending
  `SHORE_BEND_PER_STATION_DEG` per 80 m jetty station toward WSW (jetty
  roots land exactly on the shore polyline's kinks by construction —
  station spacing = two 20 m berths + a manoeuvring lane of THREE BOAT
  LENGTHS (~37 m) between the opposing pole rows, owner spec
  2026-08-04). Ten pontoon **jetties** on that shore (index 9, the
  seaward-most/outermost, is `OUTER_JETTY_LEN` = 60 m vs the standard
  34), plus two `HILL_JETTY_STATIONS` jetties on the hill (NW, inner)
  shore TUCKED UP BY THE HEAD (stations 0.5/1.5) so the rest of the
  inner shore stays open water; `CHANNEL_W` = 125 m across (widened from
  110, owner request 2026-08-05: room on the inner side) — ~63 m of
  fairway against the hill jetties, ~91 m along the open inner shore.
  Every berth ("spot") is `BERTH_LEN` 20 m long × `POLE_SPACING`
  5 m wide (owner spec 2026-08-04: the big-boat trot of the second
  reference photo, ~50 ft class with long stern lines to the poles — NOT
  the 30 ft spots of the first zoom; the sim boat berths with room to
  spare). Rows of **mooring poles** flank every jetty (`POLE_ROW_OFFSET`
  = `JETTY_HALF_W` + `BERTH_LEN` off the centreline, ball colliders of
  `POLE_RADIUS`), and `moored_boats()` fills
  ~55% of berths with STATIC hull colliders via a berth-identity hash
  (deterministic — the same fleet every run; was ~80%, thinned 2026-08-04
  so free berths are easy to find) — one end at the jetty, the other tied
  between its pole pair, ~15% bow-out. The NE end is capped by a
  **ROUNDED BAY HEAD** (`head_arc()`: a half-ellipse `HEAD_BULGE` = 70 m
  beyond the shores' ends — big enough to turn in), and the SW end is
  **OPEN TO THE SEA** (owner request 2026-08-05): past
  `ENTRANCE_MARGIN` the two coasts diverge (`SEA_COAST` offsets) into a
  ~335 m-wide patch of open water whose far edge — the world has to end
  somewhere — is the boundary segment joining the coasts' ends, rendered
  as a skerry chain. The whole boundary is ONE polyline collider: road
  shore head→sea, skerry line, hill shore sea→head, head arc closing the
  loop — no wall crosses the entrance. `jetties()`, `pole_positions()`,
  `moored_boats()`, `road_shore()`, `hill_shore()`, `head_arc()`,
  `marina_shore_len()`, `world_bounds()`, `start_pose()` are pure fp
  functions of the constants — the SINGLE SOURCE OF TRUTH shared with
  the renderer, so what's drawn IS what collides. Collider insertion
  order is FIXED (boundary → jetties → poles → moored boats → boat;
  handle numbering must be deterministic). Fender feel via friction 0.5
  / restitution 0.1; poles are slipperier (0.3). **The boat spawns in
  the FAIRWAY** between the head-end jetties (`start_pose()`: 58 m off
  the road shore at station 1.6, bow pointing seaward down-channel).
  **Gotcha (2026-08-04, re-learned at the 2026-08-05 mirror)**: the
  spawn heading is orientation-dependent (~-121° now) — heading- and
  direction-sensitive tests must derive from the pose, not hardcode:
  they measure heading DELTAS, the axial-windage test derives the bow's
  compass bearing from the pose, the lee-shore test derives its onshore
  wind bearing as bow bearing + 90°, and the spin-coupling test projects
  drift onto the boat's own starboard axis. `a_mooring_pole_stops_the_boat`
  and `an_occupied_berth_is_blocked_by_the_moored_boat` build their
  approach runs from the geometry functions (`set_pose` +
  travel-distance bounds); when placing test boats near the marina,
  approach berths along the jetty axis from the fairway or down the
  ~37 m lanes between adjacent jetties' opposing pole rows.
- **Boat**: one dynamic body, convex-hull collider of `HULL_PTS` (bow = +x
  local, ~12 m × 3.8 m). Mass = the active `BoatDesign`'s
  `displacement_kg` (2026-08-04, replacing the old `HULL_DENSITY = 200`
  ≈ 7.5 t): set via `ColliderBuilder::mass`, so Rapier still derives the
  COM and angular inertia from the hull shape (uniform spread) — only the
  total is designer-set; adjustable COM/radius of gyration is agreed
  follow-up work (see Roadmap). `HULL_PTS` is shared
  with the renderer, so **visuals match collision exactly** (the Pegasus
  alignment rule). Currently the hull, windage coefficients,
  and rendering (coachroof, cockpit, sprayhood, mast + boom — see Frontend
  conventions below) are all sized for the one ship type in service: a
  small cruising sailboat. See **Ship types** under Roadmap for how a
  second type would be added.
- **Waterline vs LOA** (2026-08-04): `HULL_PTS` is the boat's DECK
  outline — collision shape, windage, rendering, mass spread (LOA ~12 m).
  The UNDERWATER body is the keel profile alone, and since 2026-08-04
  the preset curves carry their boats' real overhangs: zero draught over
  the dry ends, so the curve's nonzero support IS the waterline
  (`waterline_extent` in sim.rs — zero-crossings interpolated exactly;
  the Alajuela's deadwood ends in a vertical CLIFF at full draught, which
  the support convention handles without any per-design constant, and
  `waterline_extent_reads_each_presets_real_lwl` pins all four published
  LWLs). Everything hydrodynamic that needs a length reads the design's
  waterline length (Reynolds, Froude/hull speed, wave resistance), and
  `wetted_surface_area` integrates over the wet range only — before this,
  every design motored like an 11.9 m-waterline boat and carried phantom
  wetted area over its dry overhangs. Measured consequences (28 hp, full
  throttle, in-`tick` open-water measurement 2026-08-07 — **superseding
  the offline-integrated 6.5/6.7/6.7/6.0 kn quoted here before, which
  the real `tick` cannot reproduce**, see the open-water-benchmarks
  bullet below): HR 38 (LWL 9.50) 5.6 kn, O'Day 39 (10.21) 5.9, Elan
  I394 (10.01) 5.8, Alajuela 38 (9.93, 11.8 t) 5.5 — each 71–76% of its
  own hull speed, the heavy short-waterline full keeler honestly
  slowest. 3→1 kn coasting: 110 m (O'Day) to 129 m (Alajuela — heavy
  boats carry their way), matching the "real boats are above 1 kn past
  100 m" benchmark that motivated the ITTC rewrite. `hull_length()` (the
  LOA measure) survives only as the tests' boat-length yardstick.
- **Env** (`wind_from_deg`, `wind_speed`, `current_to_deg`, `current_speed`):
  compass convention 0° = north = +y, 90° = east = +x. Wind is named by
  where it blows FROM (mariners' convention), current by where it sets
  TOWARD. Passed to `tick` per call like an input stream — together with
  `InputState` it's the future recording format's complete input.
- **InputState** (`throttle`, `rudder`, both -1..=1, `InputState::NEUTRAL`):
  the helm/engine half of the input stream, passed to `tick` alongside
  `Env`. `rudder` is sign-conventioned as HELM: positive = the boat turns
  to starboard (the blade deflects the other way). Both fields are clamped
  defensively at the top of `tick` so a corrupt recording can't command
  super-physical inputs.
- **Mooring lines** (2026-08-20, `sim-core/src/line.rs` + `step_lines` in
  sim.rs): NOT a Rapier joint, for the same reason hull drag isn't Rapier
  damping — every force here is computed in `tick` from modeled geometry,
  and a rope has two properties a joint can't express: it is
  **unilateral** (pulls, never pushes; slack is the normal state of a
  tended line and gives exactly zero force) and **elastic**, which is the
  whole point (nylon stretching is what stops 8.5 t without snatching).
  The force is applied at the FAIRLEAD's own world point, which is what
  makes spring lines work with no special case — a line made fast forward
  plus ahead thrust pivots the boat about that fairlead and swings her
  alongside, straight out of the existing keel/rudder model
  (`a_spring_line_swings_the_boat_alongside` pins it, against a no-line
  control so prop walk can't be mistaken for the spring).
  - **Load-elongation curve.** `T(e) = MBL*(e/e_break)^p`, fitted through
    two published points for three-strand nylon: 2.5 % extension at 10 %
    of breaking load, ~20 % at 50 %. That gives p = ln5/ln8 = 0.774 and
    e_break = 0.49. The exponent is BELOW 1 — the curve SOFTENS as it
    loads, the opposite of the intuition that a rope stiffens, and it is
    what the published data says. **Independent corroboration**: the
    extrapolated break strain, ~49 %, lands inside the 40–55 % that
    three-strand nylon is separately published as breaking at, and
    nothing in the two-point fit put it there. Both constants are
    re-derived from the anchor points by
    `nylon_curve_matches_its_published_anchor_points` so they can't drift
    apart (same derive-don't-assert rule as `RUDDER_AR`). Consequences
    worth knowing: a working breeze load (~1.1 kN, this hull in 20 kn
    abeam) gives only ~6 cm on a 10 m line — a taut line, not a bungee —
    while the stretch nylon is famous for lives an order of magnitude up
    (metres at half breaking load). Known simplification: below the lower
    anchor the law extrapolates a stiffer toe than real rope's
    constructional stretch; the fix is a third published point, not a
    shape change.
  - **`LINE_MBL`** = 34.6 kN: 1/2" three-strand nylon at 6400 lbf
    (Cordage Institute / ASTM D-4268 test methods) scaled by d² to the
    modeled 14 mm. A line exceeding it PARTS. **`LINE_DAMPING_RATIO`**
    (0.20 of critical) is a calibration inside a sourced band rather than
    a measured constant — see the snub-restitution bullet below.
  - **Lines are sim STATE, orders are input** — exactly the engine-spool
    split. `InputState::line` carries at most one `LineCommand` per tick
    (`MakeFast`/`Tend`/`CastOff`), which is all a pair of hands can issue
    at 120 Hz; `tick` owns the lines. So the rope work costs a future
    recording nothing extra: the orders are already in the stream, and
    replaying it reproduces them (the recording/replay TOOLING itself is
    still roadmap work — see Scenarios and scoring).
    `same_input_sequence_is_bit_identical` now scripts line orders too
    and compares the line state as well as the pose. Ids
    are monotonic and NEVER recycled (a `Tend` in flight must not land on
    a rope that replaced the one it meant — the same lesson as the
    frontend's recycled touch ids). `new_continuing` carries the lines
    across, so opening the keel editor while lying to your ropes doesn't
    cast them off; R-reset builds a fresh `Sim` and they're gone with it.
  - **Getting a line ashore takes time, proportional to the distance**
    (`LineState::Passing`): `dist / pass_speed`, so a metre off the
    pontoon is a step and a turn on the cleat while a full-reach throw is
    a second or so. A line is made fast at the length it turns out to be
    WHEN IT LANDS, not when it was thrown, and if the boat has drifted
    out of reach by then the throw falls short and is lost. Everything
    after that is hauling and surging from that length — which is why a
    line goes visibly slack the moment you close on the cleat.
    **The reach is checked at exactly two moments, both in sim-core**:
    when the order is given (`apply_command`, against the distance right
    then) and when the line LANDS (`step_lines`, at the `Passing → Fast`
    transition). Never in between, and never once it is fast — after
    that the scope is fixed and the rope simply stretches. The landing
    check uses the reach the throw was made under, carried on
    `LineState::Passing`, not the live setting: winding the knob down
    while a line is in the air must not retroactively lose it.
  - **The crew's limits are SETTINGS, not constants of the world**
    (`CrewLimits { pass_speed, haul_kg, reach }` on `InputState`,
    `clamped()` at the top of `tick` exactly like throttle and rudder):
    how fast a line goes ashore (1–20 m/s), what a pair of hands can
    pull (1–150 kg, default 10) and how far a line can be thrown
    (1–25 m, **default 4** — owner call 2026-08-21: short enough that
    you have to bring her alongside, rather than mooring by rope from
    half a berth away). Player knobs for how much of the game is rope
    work, set in the settings menu (`src/settings.rs`) and carried in
    the input stream rather than on the `Sim`, so a recording would
    replay with the crew it was made with. One struct rather than three loose
    fields, so `apply_command` doesn't grow a parameter per knob.
  - **Tending is force-limited one way and not the other.** Hauling in
    derates linearly to zero as the line's own tension approaches what
    the crew can pull — `CrewLimits::haul_kg`, **10 kg by default**
    (owner call, 2026-08-21), expressed in kilos because that is how
    anyone talks about what they can hold on a rope. So you cannot winch
    8.5 t up to the pontoon against a breeze by hand — you rig a spring
    and use the engine, and the crew gathers the slack you make. Surging
    out isn't limited (a turn round a cleat, let it run) but stops at
    `LINE_SCOPE_MAX`, the end of the rope.
  - **Fairleads are read off `HULL_PTS`** (the shoulder vertex, the
    waist, midway along the aft run — the outline's own half-beam at each
    station), so a fairlead is always exactly on the hull the renderer
    draws and the collider uses. Shore anchors are the pontoon cleats
    (`CLEAT_SPACING`, half a berth width apart along both faces, so every
    berth has one at each end and one in the middle) and the mooring
    poles, both from sim-core's geometry functions.
  - **What lets go is a fitting on the BOAT** (2026-08-20, owner: "what I
    wanted simulated was a cleat tearing off a boat; shore cleats are
    more robust than the ones on boats"). Each fitting has a limit
    (`fitting_limit`) and a name for the HUD when it goes
    (`fitting_gave`). BoatUS Foundation's cleat testing
    had cleat ASSEMBLIES failing between 1,190 and 7,500 lbf (5.3–33 kN),
    and note what that tested: BOAT deck hardware bolted through a deck.
    `DECK_FITTING_MBL` = 14 kN sits mid-band. It is NOT evidence about
    pontoon cleats, which are commercial gear through-bolted to a float
    frame, so `PONTOON_CLEAT_MBL` = 25 kN sits above any boat's fitting
    and below the rope's 34.6 kN. **The ordering matters more than the
    values**: deck fitting < shore cleat < rope, so what tears out is the
    thing on the boat. A mooring POLE has no fitting at all — the line
    goes round it — so there too the boat's own cleat is the limit. A
    parted ROPE was therefore unreachable until per-fitting sums landed
    (the deck fitting's 14 kN was always the lowest of the three, so it
    always went first); it becomes possible only when opposed ropes on
    one fitting cancel enough of its load for the rope to reach 34.6 kN
    first. When a fitting is carried away by a SUM no single rope
    failed, so the HUD is handed the heaviest-loaded rope on it — a
    shared fitting still names one thing.
    Measured threshold (`measure_snap_threshold`): backing onto a 5 m
    scope with slack out, she holds at a 3.2 kn arrival and tears the
    fitting out at 3.8 — a genuinely bad arrival for 8.5 tonnes.
    **Getting the ordering backwards was a real bug** (the first version
    had the pontoon cleat weakest at 9 kN): the marina then destroyed
    itself at the top of the wind dial, losing 63 harbour cleats to a
    slammed wind reversal, because those breast lines sat at 87 % of a
    9 kN limit before anything happened. With the ordering right the same
    reversal peaks at 10.6 kN against 14 kN and costs nothing.
  - **What the weather can do to the marina** (2026-08-20, re-measured
    2026-08-21; `measure_resting_mooring_loads` and
    `measure_mooring_loads`). At `WIND_MAX` (25 m/s) the fleet's
    most-loaded ropes sit at a p90 of 7.8 kN — 56 % of the 14 kN deck
    fitting that holds them — and a slammed SE→NW reversal peaks at
    10.6 kN and costs nothing. The suddenness was never the issue: a
    step from calm to 50 knots in one tick peaks at 4.8 kN. What made
    the marina fragile was the FITTING ORDERING (see
    `DECK_FITTING_MBL`), not the weather model. Two things worth
    remembering from that investigation: a berth's breast lines carry
    more than its crossed pole lines (p90 7.8 vs 6.9 kN at `WIND_MAX`)
    because the pole lines meet a beam load at poor angles; and a wind
    DIRECTION swept round has a resonance band at 6–8 s per revolution,
    the mooring system's own period (12.4 kN at 6 s against 7.8 kN at
    45 s), far more punishing than any step — so "smooth the dial" would
    move a fast sweep toward the dangerous band, not away from it. Max
    wind AND max current together still reach 14 kN, which is correct
    (water is ~800× denser than air) and a marina nobody would have
    built; `CURRENT_MAX` is the knob there, not the physics.
  - **A torn-out fitting stays torn out** (2026-08-20). Without this the
    punishment for a bad arrival was "throw the same line at the same
    cleat again", which quietly undoes the consequence. `Sim.broken`
    records each `Fitting` as it goes — `Fitting::Shore(pos)` for a
    pontoon cleat, `Fitting::Deck(hull, fairlead)` for a deck fitting —
    and `MakeFast` refuses either end if its fitting is gone. Identified
    by POSITION rather than index because the marina's cleats are
    generated geometry with no identity of their own. A parted ROPE
    leaves both fittings intact; anything else takes one with it — and
    the fitting takes EVERY rope that was on it, not just the one that
    overloaded it (`line_on_broken_fitting`, shared with
    `new_continuing` so the live rule and the carry-across rule cannot
    drift; `a_fitting_that_tears_out_takes_every_rope_that_was_on_it`).
    A cleat torn off the pontoon cannot still be holding its
    neighbour's rope. This was already reachable through a shore cleat
    (two fairleads could always share one) and allowing ropes to share a
    fairlead made it reachable on deck fittings too. Only the rope that
    actually overloaded is reported in `line_failures` — the others did
    not fail, they lost what they were tied to, so the HUD still names
    one thing. Frontend consequence: `report_failures` sets a flag that
    stops `prune` reporting a rope taken with its fitting as a throw
    that "fell short", which would otherwise overwrite the accurate
    message in the same frame.
    `new_continuing` carries the damage across (a keel change does not
    repair the marina) — and PRUNES the fresh rigging to match, because
    `build` re-rigs the fleet knowing nothing about it and a boat would
    otherwise get its carried-away cleat back while the renderer still
    drew the holes (CodeRabbit review, 2026-08-21;
    `a_keel_change_does_not_re_rig_a_fitting_that_tore_out`). R-reset
    builds a fresh `Sim` and everything is whole again, which is what
    starting a run over should mean. The
    renderer draws a carried-away cleat as the holes it left and a torn
    fairlead as a crossed-out stub, and `reachable`/`nearest_fairlead`
    stop offering them.
  - **The damper must not out-pull the rope.** Damping force is bounded
    by the elastic tension (`LINE_DAMP_FORCE_CAP`), which is the shape of
    hysteretic damping — energy lost per cycle is a FRACTION of energy
    stored — and fixes a measured artefact: because the load-elongation
    curve is sub-linear its tangent stiffness is formally infinite at
    zero strain, so a damper sized from that stiffness spiked at exactly
    the moment of contact (1.6 kN of damping against 267 N of elastic
    tension), felt as a snatch the rope should not give.
  - **How springy the rope is, in one number.** `measure_snub_restitution`
    (ignored harness) brings the boat up on a paid-out line at 2.5 kn and
    reports how much of her way she gets back. The damping ratio is a
    calibration inside the published 20–50 % hysteresis band for nylon,
    not a measured constant: 0.05 returns 61 % of her kinetic energy,
    0.10 48 %, 0.20 32 %, 0.35 20 %. Shipped at 0.20 — the damped end,
    deliberately, because nothing here models the friction of a line
    surging round a cleat and through a fairlead, which in life eats a
    real share of a snatch.
  - **Gotcha (found in the first backing test)**: arriving on a short
    line with real sternway SNATCHES, and the boat rebounds forward off
    it and goes slack again before settling, so line tests must check the
    ENVELOPE over a run (peak tension, peak stretch) rather than sampling
    at one instant. A second lesson from the same place (2026-08-20): a
    snatch needs SPEED onto SLACK. Backing from rest on a taut line just
    loads up to the thrust and holds — two tests were written with that
    backwards and proved the opposite of what they claimed.
  - **Six fairleads, none on the centreline** (owner call, 2026-08-20):
    bow pair, waist pair, quarters. A stem-head or stern fairlead sits
    right between its own port and starboard pair, which makes the
    nearest-handle picker ambiguous exactly where you most want to be
    sure which side you are leading from — and a bow line off the stem is
    barely a different rope from one off the port bow.
  - **Not the same connection twice** (owner call, 2026-08-21,
    REPLACING the one-rope-per-fairlead rule of 2026-08-20). Doubling up
    is real seamanship and is now allowed: several ropes off one
    fairlead, several onto one cleat, on the boat's fittings and the
    harbour's alike. What is still refused is a second rope between the
    SAME fairlead and the SAME anchor — the fumbled gesture the old rule
    was really aimed at, and the one case that would sit exactly on top
    of the first where nothing could select it
    (`a_fairlead_may_carry_several_ropes_but_not_the_same_one_twice`).
    Anchor identity is by VALUE, and a shore anchor carries its position,
    so this leans on the same exact-float identity `Fitting::Shore` does
    — sound for the same reason: every cleat position comes from the one
    generator (`cleat_point`). `LINE_COUNT_MAX` no longer coincides with
    the handle count: six ropes is now a real cap rather than an
    arithmetic accident.
    **A fitting carries the SUM of what is on it** (2026-08-22,
    implemented in the same breath as allowing ropes to share one).
    Doubling onto ONE fitting must not make that fitting stronger, which
    is what checking each rope separately would have meant; spreading a
    load over TWO fittings genuinely does strengthen a mooring, and falls
    out of the same rule for free
    (`one_fitting_carries_the_sum_of_the_ropes_on_it` tears a deck
    fitting out with two ropes neither of which is near its own limit).
    The sum is a VECTOR sum, because these are forces: two ropes off one
    cleat pulling opposite ways largely cancel the pull-out load, so
    doubling up can make a mooring EASIER as well as harder (owner call —
    "it might as well make mooring easier sometimes"). Mildly optimistic
    about the horn-crushing mode, where opposed loads still stress a
    fitting.
    **`step_lines` is two phases because of it.** Phase one walks the
    lines: tension, forces onto the hulls, the one per-line limit left
    (the ROPE's own `LINE_MBL`), and a deposit of `(fitting, force)` into
    a scratch buffer. Phase two groups that buffer and breaks any fitting
    whose total exceeds its limit. Turning it round this way is what
    kills the QUADRATIC: asking each line "who else is on my fitting?"
    is a SEARCH — a line names its fittings by value and there is no
    fitting→lines index — and doing that per line is ~2n² comparisons a
    tick, measured at **313 µs against a ~111 µs tick**. Depositing
    instead of searching makes phase one linear, and phase two is then a
    grouping problem: a sort of a REUSED scratch buffer (`Sim.fit_loads`,
    cleared not reallocated) plus a walk of the runs, keyed by
    `fitting_key`. So the whole thing is **O(n log n), not linear** —
    the sort is the dominant term, and an earlier draft of this note
    wrongly called it linear.
    **A HashMap accumulator IS linear and was measured, not argued
    away** (2026-08-22, owner: "why create a tmp Vec and then sort it,
    when you could create a temp Map"): keyed by the same
    `fitting_key`, depositing with `entry().or_insert()`, and — to stay
    deterministic — reading it back by walking the LINES rather than
    iterating the map, since `HashMap` order would decide which failure
    the HUD names first and in what order `broken` grows. It builds for
    wasm and passes the whole suite. It is also **slower here**: 143 µs
    with a cheap FNV-style hasher and 160 µs with the default SipHash,
    against 121 µs for the sort (111 µs baseline). Not a hashing
    problem — a locality one: ~450 contiguous entries sort with almost
    no cache misses, while the map pays random-access probing on every
    one of ~1000 deposits and lookups a tick, and `HashMap::clear` walks
    its control bytes where `Vec::clear` is free. The asymptotically
    better structure loses at this n; revisit if the marina ever grows
    an order of magnitude. **Measured cost of the whole change:
    111 → 121 µs, +9%**, of which the grouping is ~13 µs; at n = 450
    entries log₂n ≈ 9, so phase one's real work dominates the clock even
    though the sort dominates the asymptotics. Truly linear IS available
    and was deliberately NOT taken: a persistent fitting table with
    indices on each line makes each deposit O(1) and phase two O(F), for
    O(n + F) overall. Rejected because `Fitting` is public
    API (`broken_fittings()`, the renderer's torn-cleat drawing, the
    `MakeFast` refusal), so indices would ripple through all of it and
    add an index-validity invariant to keep in step with `MakeFast`,
    `new_continuing` and the fleet rebuild — a new class of bug to save
    ~10 µs. A spatial index over the cleats is the wrong tool entirely:
    it answers "what is NEAR this point?", which is LINES-mode
    hit-testing, not "which ropes are on exactly THIS fitting?", and
    fitting identity is already exact by construction.
    **Consequence for sleep**: a sleeping fleet boat's ropes are now
    WORKED OUT every tick and skipped only for the force application,
    because a fitting she shares with someone else's rope has to carry
    her load too. Skipping them outright — as the first version did —
    would silently drop her contribution the moment a player rope landed
    on a cleat she was already using, which is reachable. The perf win
    survives because it was never the arithmetic, it was the WAKING.
    A side benefit: her `tension` is live now rather than stale from
    whenever she last woke, which is why the resting-load figures moved a
    few percent without any change in the dynamics (the storm peaks are
    bit-identical).

  - **A line can be made fast to another BOAT** (2026-08-20):
    `Anchor::Boat { hull, fairlead }` alongside `Anchor::Shore`, so
    rafting up — or taking a line to your neighbour while you get sorted
    out — is a rope like any other. It pulls at BOTH ends: `step_lines`
    applies the equal and opposite force at the far hull's own fairlead
    and wakes her, because she is being hauled on rather than lying to
    her own moorings. Pinned by a force balance against a no-rope control
    (`a_line_to_a_neighbour_hauls_on_the_neighbour_too`): her own
    moorings go from ~2 N lying quiet to a few hundred N with a rope on
    her quarter. Measuring her DISPLACEMENT instead is a trap — the
    marina shakes down onto its moorings over the first half-minute
    either way, and that motion is far bigger than the effect.
  - **The moored fleet lies to the same ropes** (2026-08-20, second
    commit): every berthed boat is a DYNAMIC body with four real `Line`s
    — two crossed to its pole pair, two breast lines to the pontoon face,
    the rig of the reference photos — instead of a static collider with
    renderer-only decoration. One `Line` type, one tension law, one
    `tick`: `Hull::Player` / `Hull::Moored(i)` on each line says which
    hull it pulls. So an occupied berth still stops you, but it GIVES
    when you lean on it and its moorings put it back
    (`an_occupied_berth_is_blocked_by_the_moored_boat` checks all three),
    and the whole marina leans a little when the wind gets up. Their
    lines are made fast at exactly the distance they span at rest, so the
    marina starts snug with no slack and no invented pre-tension: any
    movement in any direction lengthens at least one of a berth's four
    ropes. The breast lines belay to the two studs STRADDLING the berth
    centre — real stations from `cleat_point`, the one generator
    `cleat_positions` also goes through, so a rope ends where a stud is
    drawn (CodeRabbit review, 2026-08-21: they used to sit a free 1.3 m
    either side of centre, about half a cleat spacing, so all 150 of
    them finished on bare planking and a fitting torn out there was
    recorded at a point no stud ever occupied;
    `every_fleet_cleat_is_a_cleat_the_renderer_draws` pins it). Exact
    float equality is why that is ONE generator and not two agreeing
    formulas: a shore fitting is identified BY its position. What made
    it fit was PHASING the cleat grid half a spacing off the pole
    stations (`CLEAT_PHASE`), so each berth gets a pair 1.25 m either
    side of its centre instead of one stud dead centre and one out at
    the boundary where a pole already stands. That was not cosmetic
    book-keeping: snapping to the boundary studs (±2.5 m) instead splays
    the lines wider, meets a beam load at poorer angles, and turned a
    max-wind reversal from costing nothing into tearing out 22 fittings;
    putting both lines on one central stud drops the inboard yaw
    restraint entirely. The phased grid leaves every measured mooring
    load within noise of what the off-grid rig gave.
  - **Sleep is what makes ~75 extra hulls affordable**, and it took some
    care. Measured cost per `tick` (native release, full marina): 101 µs
    with the old static fleet, 113 µs now — but **375 µs** in the version
    where the fleet never slept, a 3.7× regression that would have shown
    up on a phone. The fix is that a berthed boat settling on its ropes
    is allowed to doze: the fleet's hull forces, its rope pulls, AND its
    per-tick `reset_forces`/`reset_torques` all pass `wake_up: false`.
    (2026-08-22 update, CodeRabbit review: this paragraph used to say
    `step_lines` skips a sleeping hull's ropes ENTIRELY, which stopped
    being true the moment fittings started carrying a sum — a fitting
    she shares with someone else's rope has to carry her load too, so
    her ropes are now WORKED OUT every tick and only the force
    APPLICATION is skipped; see the per-fitting-sum bullet below for the
    updated 121 µs figure.)
    **Gotcha**: `reset_forces(true)` was the one that kept the whole
    marina awake — easy to miss, since it reads as bookkeeping rather
    than as a force. The one thing sleep would otherwise break is that
    wind and current are knobs the PLAYER turns, so `tick` compares `env`
    against the previous tick's and wakes every moored body when it
    changes (`a_change_of_wind_wakes_the_sleeping_fleet` pins it);
    contact wakes a boat by itself, so nudging one still works.
  - Fleet line ids live in their own range (`FLEET_LINE_ID_BASE`), so the
    crew's ids stay 0, 1, 2… however many boats are berthed, and a
    `CastOff` can never name one of the marina's own. `new_continuing`
    transplants only the player's lines — `build` re-rigs the marina
    from scratch, and its boats sit centimetres from where they were.
  - **Known simplification**: lines don't collide with anything — they
    pass through poles, jetties and moored hulls, with no friction round
    a turn and no catenary weight. Top-down 2D; documented on the module.
- **Engine & propeller** (~28 hp auxiliary, fixed right-handed 3-blade
  prop): thrust, walk and wash act at the prop's station, which since
  2026-08-04 is DERIVED per design as `rudder.x + PROP_AHEAD_OF_RUDDER`
  (0.5 m ahead of the blade) instead of the old fixed `PROP_X = −5.6` —
  real geometry on every reference boat has the prop just forward of its
  rudder, and it's load-bearing: the prop-wash steering term assumes the
  blade stands in the race, which is only true if the prop leads it
  whatever design is active. Exposed as `sim::prop_station(rudder_x)`
  (2026-08-21, CodeRabbit review on PR #29) so the renderer's wake sheds
  the race where `tick` actually applies thrust — it had re-derived the
  station as the blade's own `x` and put the churn half a metre abaft
  the prop. Same single-source-of-truth rule as the harbour geometry and
  `HULL_PTS`. `Sim.engine` is the
  telegraph filtered by a first-order lag (`THROTTLE_TAU` 0.4 s) — sim
  STATE, not input, advanced only inside `tick` and reset for free by the
  fresh-`Sim`-per-run rule (`engine_spools_rather_than_steps`). Thrust =
  `T_max·n|n|·clamp(1 − adv·|adv|, -1, 2)` with `adv = surge·sign(n)/
  (|n|·U_PROP_RACE)` — bollard 4200 N ahead (~0.2 kN/kW rule), ×
  `ASTERN_RATIO` 0.6 astern, equilibria ≈ 3.2 m/s full ahead / 1.5 m/s
  half / 1.9 m/s astern (`full_throttle_equilibrium_speed_is_bracketed`,
  `astern_is_weaker_than_ahead`); the clamp bounds the windmilling brake
  and the crash-stop bite. **Prop walk** is a side force at the prop ∝
  |thrust|: `PROP_WALK_AHEAD` 0.06 (stern nudges starboard) vs
  `PROP_WALK_ASTERN` 0.13 (stern kicks port — "backs to port",
  `a_burst_astern_walks_the_stern_to_port`).
- **Rudder** (per-design since 2026-08-04: `RudderDesign { x, chord,
  depth, root_endplated }` on `BoatDesign`, derived once per `Sim` into
  `RudderFoil` — area, effective AR, post-stall ceiling; ±35° stays a
  shared constant, a property of typical steering gear rather than of a
  boat. Each preset carries its real boat's blade — the O'Day's
  replacement-listing dimensions are the anchor, the others derived from
  type + profile + the %-of-lateral-plane cross-check, all documented in
  boat.rs / reference-boats.md. Blade POSITIONS sit relative to each
  design's own waterline endings — since the profiles carry real
  overhangs (see the Waterline bullet below), a spade's trailing edge
  stands at the curve's own aft ending, and the Alajuela's outboard
  blade hangs entirely ABAFT its sternpost cliff. (Historical gotcha,
  2026-08-04: while `HULL_PTS` still doubled as the waterline, mapping
  the O'Day spade by LOA double-counted the overhang and collapsed the
  coast turn — position mappings are only meaningful against a
  consistent waterline, which is what motivated the waterline refactor.)
  A transom-hung
  blade's root breaks the surface with air above it → NO end-plate
  mirror, AR = depth/chord not 2×, which is why the Alajuela's barn door
  is mushier per m² than the spades. The editor draws the loaded design's
  blade and carries it through Apply, but doesn't edit it yet —
  follow-up work): a
  foil in the LOCAL water-relative flow at the stern — surge/sway PLUS the
  yaw sweep `w·x` at the blade's station, which is the rudder half of the
  keel coupling
  (the keel's moments set how fast yaw builds; built-up yaw feeds the
  rudder's angle of attack).
  **Blade dimensions (2026-08-03, second pass) — sized from a real boat,
  not picked to "look right"**: the original 0.4 m × 1.35 m (0.54 m²) was
  undersized by roughly half against two independent checks — the O'Day
  39's actual spade rudder (one of the reference boats already used for
  this hull's own dimensions: ~5 ft/1.52 m deep, chord tapering 28 in/
  0.71 m at the head to 20 in/0.51 m at the tip, average ≈0.61 m, area
  ≈0.93 m²) and the lateral-plane rule of thumb (rudder ≈10% of total
  underwater lateral plane, which against this hull's own
  `KeelDerived.area` solves to ≈0.95 m²). `RUDDER_AR` is now DERIVED as
  `2·(depth/chord)` instead of asserted as a bare `3.0`
  independently of the blade's own dimensions — the old value implied a
  geometric AR of ~1.5 before doubling, but the blade's own literal
  dimensions gave 3.375, an inconsistency that went unnoticed until sized
  against a real reference; deriving it structurally means the two numbers
  can't drift apart again. Net effect: lift slope rose from ≈3.77/rad to
  ≈4.48/rad (AR 3.0 → 4.98) on top of the ~72% bigger area — measured
  against the pure-rudder backing-turn benchmark from earlier testing (2.5
  kn sternway, engine neutral, full rudder), 90° of turn now arrives at
  ~13 m of travel, vs. never getting near 90° at all with the old blade —
  much closer to the ~90°/8m real-world mooring-class benchmark that
  motivated this whole rudder investigation.
  `rudder_lift_drag` (2026-08-03 rewrite, replacing the old `rudder_cl`):
  linear thin-airfoil slope 2π·AR/(AR+2) to ~17° (unchanged in FORM — this
  regime was never the problem) blended into a flat-plate normal-force
  law past ~25° (the Viterna–Corrigan technique used to extend
  wind-turbine blade sections past stall). The plate ceiling is
  `RUDDER_CD_MAX = flat_plate_cd(RUDDER_AR)` ≈ 1.20 (2026-08-04 second
  pass, maintainer-sourced against the standard drag tables): the first
  pass used Hoerner's 1.98, which is the 2D/infinite-plate LIMIT — a real
  AR-5 blade never reaches it because broadside flow escapes under the
  foot (Hoerner's finite-plate data: AR 1 → 1.18, 10 → 1.3, ∞ → 1.98;
  Viterna–Corrigan's own ceiling is `1.11 + 0.018·AR`, the function now
  in `keel.rs`). Deriving it from `RUDDER_AR` also means the lift slope
  and post-stall drag can't disagree about the blade's
  three-dimensionality — same derive-don't-assert move as `RUDDER_AR`
  itself. Resolved into lift/drag by the chord-to-flow
  angle, folded by ±π so a foil overtaken by the flow (backing) is still a
  foil (`backing_reverses_the_helm`). **The old post-stall curve
  (`0.9·sin 2α` lift, induced-drag-only `cd`) was backwards at the one
  angle that matters most**: both collapse toward ZERO at α=90°, exactly
  when a CENTERED rudder is swept broadside by the hull's own spin — so a
  boat spinning with the helm amidships got almost no rudder resistance
  from it, however hard it was actually spinning. The Hoerner law instead
  peaks at α=90° (zero lift, max drag — the barn-door case), so a centered
  blade correctly brakes a spin using the exact same live-angle
  calculation that lets a deflected blade drive a turn — no separate
  mechanism, no risk of the two disagreeing.
  The foil owns the rudder's ENTIRE physical footprint now: `keel.rs`'s
  presets no longer paint it as a fixed area strip (see that module's own
  notes) — the old split (profile owns "at rest", foil owns "deflected")
  meant a hard-over, actively-turning blade was STILL charged the full
  passive drag of a centered one, fighting its own turn. Verified by
  `rudder_lift_drag`'s own test asserting near-zero lift / near-max drag
  at 90°, and by `spinning_an_aft_biased_hull_shoves_it_to_starboard`'s
  "symmetric keel" control, which now drifts SOME on its own (the rudder's
  fixed aft position couples to spin regardless of keel symmetry) but
  reliably less than the aft-biased default — the keel's own
  `swept_moment` stacking on top of the shared rudder baseline, not a
  separate zero/nonzero split like before.
  **Composition (`rudder_force`, added alongside `rudder_lift_drag`)**: the
  boat physics (`tick`) computes the actual local inflow at the blade
  (surge/sway/yaw-sweep — it's the only side that knows the boat's motion)
  and hands it to `rudder_force(flow, delta)` as a plain vector in the same
  (fwd, side) frame; that function is a pure foil model — chord vs. flow
  angle in, lift+drag force in that SAME local frame out — with no idea
  it's attached to a boat at all. `tick` then rotates the returned force by
  `fwd`/`side` into world space to apply it at the blade's world position.
  Splitting it this way makes the foil directly unit-testable without a
  `Sim`: `rudder_aligned_with_flow_has_no_effect` (a chord parallel to the
  inflow, however that alignment arose, produces zero lift and only
  baseline drag) and
  `following_helm_stays_attached_but_opposing_helm_stalls_while_spinning`
  (spinning clockwise, helm INTO the turn re-attaches the flow even at full
  deflection because the sweep and the deflection rotate the chord-to-flow
  angle the same way; helm OPPOSING the spin stalls from just a few degrees
  because they fight each other) both call `rudder_force` directly instead
  of running a transient `Sim`.
  **Gotcha (2026-08-03, verify physics fixes against the benchmark that
  motivated them, don't assume)**: this fix does NOT, by itself, reproduce
  a tight prop-walk-free backing turn (real small-boat mooring-class
  benchmark: ~90° of turn within ~2 boat lengths at ~2.5 kn, rudder only,
  engine in neutral) — a from-rest transient check after the fix showed
  *less* turn over the same distance than before, because the corrected
  curve's peak lift (~0.76 at ~55°) sits above `RUDDER_MAX_DEG` (35°),
  while the old (wrong) `0.9·sin 2α` curve happened to peak near 45°,
  closer to the actual achievable hard-over angle. The spin-braking fix is
  real and correct; closing that remaining gap is a separate, still-open
  question (candidates: is 35° actually the right hard-over limit once
  the lift curve is honest, is `RUDDER_AREA` sized right, does neutral-gear
  prop drag/walk need modeling) — don't fold a fudge into this curve to
  chase that number, verify against a fresh transient run instead. Stall
  still shows up as a mushier INITIAL bite hard-over
  (`hard_over_stalls`) — at steady state the yaw feedback eases the
  effective angle back toward the slope, so hard-over still out-turns
  moderate helm, just draggier (that's real low-AR-rudder behaviour,
  don't "fix" it). **Prop wash**:
  thrust-deflection form `K_WASH·max(T,0)·sin δ` at the blade — steerage
  from a standing start ahead (THE harbour move: burst of power kicks the
  bow before the boat gathers way), nothing astern (the wash misses the
  blade), both from the single `max(T,0)`
  (`prop_wash_steers_at_rest_ahead_but_not_astern`). Chosen over a
  slipstream-velocity model because the deflected momentum flux IS the
  thrust — bounded by construction, no ad-hoc cap.
- **Axial (surge) water resistance is ITTC-1957 skin friction, not a bluff-body
  Cd** (2026-08-03 rewrite, replacing `CD_WATER_BOW`/`CD_WATER_STERN`/
  `WATER_AREA_FRONT`). The old model applied a frontal-area × flat-Cd bluff
  -body formula underwater — the same functional form correctly used for
  windage on the topsides, but wrong here: this hull's Froude number even
  at 3 kn is only ~0.14, well below the ~0.35–0.45 where wave-making
  resistance (a real bluff-body-like effect) matters, so real resistance at
  cruising speed is overwhelmingly skin friction over the WETTED SURFACE, a
  different mechanism with a much smaller coefficient (~0.003, not
  ~0.15–0.5) over a much larger area (wetted surface, tens of m², not
  frontal area, a few m²). Diagnosed from a real complaint: the boat lost
  way from 3 kn to 1 kn in ~17 m; real boats this size are still above 1 kn
  past 100 m.
  - `ittc57_cf(re) = 0.075/(log10(re)-2)²`, the standard formula (ITTC's
    own recommended procedure 7.5-02-02-02) — clamped to `re.max(ITTC_RE_FLOOR
    = 1e5)` before evaluating (2026-08-04 fix). The formula has a genuine
    mathematical POLE at `Re = 100` (`log10(Re) = 2` zeroes the
    denominator), not just the `Re = 0` case its shape suggests is the
    only edge case — real hulls never operate anywhere near there (this
    hull is Re ~1.5e7 at 3 kn; the line is meant for Re > ~10^6), so the
    1e5 floor is a generous margin below any speed this sim cares about
    while sitting safely away from the singularity, and the clamp only
    ever engages at speeds low enough that `surge·|surge|` at the call
    site already drives the actual FORCE toward zero regardless of what
    `Cf` does. **Found live, by the CD split above**: a boat resting
    almost exactly still against the quay (wind-pinned, near-zero but
    nonzero surge from floating-point noise) had its Reynolds number
    drift across exactly 100 on the way through zero and got launched
    sideways by a momentary near-infinite friction force — the CD split
    didn't cause this (the pole was always there), it just perturbed the
    settle trajectory enough to cross it within a 40 s test window where
    the previous trajectory hadn't. `the_quay_stops_an_onshore_wind` is
    what caught it.
  - `wetted_surface_area()` integrates a per-station semi-ellipse girth
    (`π/2·(half-beam+draught)`) along `HULL_PTS`' own beam curve and the
    keel profile's own draught curve (see the Keel profile bullet — profile
    values are real depth now, not tuned for feel), plus the rudder's own
    wetted area added separately (a movable appendage the profile excludes)
    — computed once per `Sim` from real modeled geometry, not an assumed
    whole-boat average.
  - `HULL_FORM_FACTOR = 1.2` (`(1+k)`, correcting the ITTC line's flat-plate
    calibration for a real 3D hull's viscous pressure resistance) is the
    ONE number in this whole model that isn't either read from the sim's
    own geometry or a fixed physical formula — a typical value (1.1–1.3)
    for a fine sailing hull, not fitted to hit a target.
  - No fore/aft asymmetry any more: friction depends on wetted area and
    speed, not which end leads, and this hull tapers to a point at both
    ends (`HULL_PTS`) rather than presenting a flat transom to separate
    flow off. No added low-speed linear term either — though NOT because
    Cf falls with speed (the first version of this claim had the
    mechanism backwards, caught in maintainer review): `Cf` actually
    RISES slowly as Re falls (~1/log²Re), and the FORCE converges to
    zero at rest because the u² factor collapses far faster than Cf's
    logarithmic growth (Cf is capped below `ITTC_RE_FLOOR` besides).
  - **Known simplification (maintainer review note; mechanism pinned
    down 2026-08-05, see the astern note at `C_WAVE_SCALE`)**: the wave
    term is direction-symmetric, so full-astern equilibrium is brisk
    (measured 5.3–5.8 kn per boat vs a real 2–4). The missing physics is
    the wave AMPLITUDE's dependence on the leading waterplane ending's
    fineness — backing, a wide transom leads and piles up a far bigger
    leading wave (a canoe-sterned double-ender barely pays this, part of
    why double-enders back sweetly). Not implemented: the factor can't
    be derived from modeled geometry (no transom exists in `HULL_PTS`)
    and no verifiable blunt-end residuary anchor was available. Harmless
    below ~3 kn (Fn ≲ 0.15, wave term negligible both ways), so no
    harbour manoeuvre sees it — only a straight-line astern sprint.
  - **Verified, not just derived**: `coasting_from_cruising_speed_covers_a_realistic_distance`
    runs the full 3 kn → 1 kn benchmark through the real `tick` in the
    open-water test arena (2026-08-07; it used to check a 20 s
    basin-safe slice — the ±40 m basin plus the hull's 6 m bow overhang
    stop a straight run at ~34 m — and lean on an OFFLINE re-integration
    of the formulas for the full distance, which was retired after it
    was caught drifting from the real `tick`, see the
    open-water-benchmarks bullet below). Per-boat distances: 110–129 m.
  - **What fixing this correctly EXPOSED**: full-throttle equilibrium
    initially came out to ~4.85 m/s (9.4 kn) — above this hull's classic
    displacement hull speed (~8.4 kn), not achievable on 28 hp in reality.
    The old bluff-body coefficient had been accidentally capping top speed
    at a plausible number while being wrong at low speed; fixing the low
    speed regime honestly uncovered that the sim had no wave-making
    resistance term at all to do that job for real. Fixed by adding one —
    see the next bullet — rather than papering over it by re-inflating the
    friction coefficient. Two lessons in one: a coefficient tuned to look
    right at one speed can be very wrong at another, and fixing one regime
    honestly can uncover what was quietly covering for a missing one.
  - Also surfaced the same "was calibrated to the old, too-strong drag"
    issue on the OTHER side of the same formula:
    `current_carries_the_boat_along` (a moored boat picked up by a current
    from a dead stop) is the mirror image of coasting to a stop — same
    weak-friction-at-low-relative-speed physics, so it's now also
    correctly slower, and its threshold/duration were updated to match
    real behaviour instead of the old too-fast pickup.
- **Wave-making resistance** (`wave_resistance_coefficient`, added right
  after the ITTC friction fix to close the gap it exposed — see above): the
  RIGHT tool is hull-form-specific (the Delft Systematic Yacht Hull Series,
  DSYHS — Gerritsma/Onnink/Versluis 1981, refined since by Keuning et al. —
  tank-tested residuary-resistance regressions against 22 yacht hull forms,
  what real yacht VPPs use), but **not implemented**: it needs a coefficient
  table that couldn't be verified from available sources (four fetch
  attempts 403'd or came up empty) and hull-form inputs this sim doesn't
  model yet (prismatic coefficient, LCB, midship coefficient, `BWL/Tc`) —
  reciting a plausible-looking but unverified table would have been exactly
  the kind of invented number this whole rewrite has been getting away
  from. **What IS implemented is the right FUNCTION CLASS, generically
  calibrated rather than hull-form-fitted**:
  `Cw(Fn) = C_WAVE_SCALE · exp(-C_WAVE_K / Fn²)` — the classical thin-ship
  (Michell/Havelock) asymptotic result for wave resistance at low-to-
  moderate Froude number: an essential singularity vanishing faster than
  any power of Fn as Fn→0, then rising steeply approaching the hull-speed
  hump. Theoretically correct SHAPE, independent of hull form (DSYHS would
  sharpen the AMPLITUDE for this specific hull, not change the shape). The
  two constants come from two widely-cited, GENERIC anchor points (not
  fitted to this boat's own target behaviour): `Rw/Δ ~ 0.001` (negligible)
  at `Fn=0.20`, `Rw/Δ ~ 0.12` (dominant) at `Fn=0.42` (~hull speed).
  **Consistency check, not a target**: solving those two constants and
  re-running the equilibrium landed full-throttle at 6.3 kn — close to the
  ~6.2 kn this sim's engine sizing always assumed (`T_BOLLARD_AHEAD`'s
  comment), without that number being an input anywhere in the derivation.
  **That check did not survive the waterline refactor** (caught
  2026-08-07 by the open-water in-`tick` measurement): the calibration
  was solved against the shared 11.9 m outline waterline, and on the
  presets' real 9.5–10.2 m LWLs the same amplitude bites ~1 kn harder —
  the shipped equilibria are 5.5–5.9 kn per boat (71–76% of hull speed,
  vs ~6.5–7 kn for real ~38 ft auxiliaries). An open calibration
  question, deliberately NOT retuned by feel: shrinking `C_WAVE_SCALE`
  until the number looks right is the invented-constant move this file
  bans; the honest fix is the DSYHS upgrade path below.
  Verified the low-speed coasting benchmark stays undisturbed too (Cw is
  negligible at 1–3 kn / Fn~0.05–0.15, so the 110–129 m per-boat
  distances in the Waterline bullet are set by friction alone) —
  `wave_resistance_is_negligible_at_low_speed_and_dominant_near_hull_speed`
  locks in both ends plus monotonicity. **Upgrade path, written down so it
  isn't lost**: replace `Cw(Fn)` with the real DSYHS regression once its
  coefficient table can be sourced and verified, deriving its hull-form
  inputs from `HULL_PTS`/`KeelProfile` the same way `wetted_surface_area`
  does now — sharpens the amplitude for this hull without changing the
  function class.
- **Sway/yaw hydrodynamics are ONE strip model with two halves** (the
  second half added 2026-08-04; before that the sim had only the first and
  turned "like a containership" going ahead). Each fore-aft station sees
  the local lateral relative flow `V(x) = v + w·χ` (χ measured from the
  COM) and the two halves are the separated and attached responses to it:
  - **Separated half — cross-flow drag** (quadratic + linear, on velocity
    RELATIVE TO THE WATER, via the real ρ·Cd·A formulas with the
    `drag_*` keel moments): a uniform current is just "the water moves",
    so the same term both damps the boat and carries it along. The linear
    terms ARE deliberate here (unlike surge, which dropped its linear
    term — see above): sway/yaw's quadratic coefficients are
    keel-profile-derived, and a profile far from the one a fixed linear
    floor was tuned against would fall out of proportion, so each uses a
    crossover speed/rate scaled by the profile's own quadratic
    coefficient instead. World-frame Rapier damping would fight a current
    instead of converging to it (holds the boat below water speed), so
    all drag lives in `tick`, relative to the water, and the body's
    Rapier damping is 0.
  - **Attached half — strip momentum exchange** (`AttachedFlow` /
    `attached_flow_coeffs` in sim.rs, coefficients integrated once per
    `Sim` from the profile's section added mass `m_a(x) = ρ·π/2·a(x)²`,
    the mirrored-plate value): the hull under way exchanges lateral
    momentum like a low-aspect-ratio wing, `dY/dx = u·d/dx[m_a·V]`,
    integrated from the bow back to the AFTMOST m_a-peak station and cut
    there (the Kutta condition — past the peak the ideal flow would hand
    the momentum back; really it separates at the keel's trailing edge
    and leaves in the wake; this cut is the ONE structural judgement,
    everything else derives). One integral yields the three classical
    results at once: Jones' slender-wing keel lift EXACTLY
    (`attached_flow_reproduces_jones_slender_wing_lift` pins it — this is
    what lets the boat carve a turn from ~3° of leeway instead of
    ploughing a 25° quadratic-drag skid), the destabilizing Munk moment
    (`attached_flow_moment_is_destabilizing_ahead` — bow-into-the-turn
    eagerness; overall directional stability comes from the rudder foil
    standing in the flow aft, the real mechanism), and the ideal
    yaw-rate coupling. Not fragile by construction: scales as u·V ∝
    U²·sinβ·cosβ, self-saturating at 45° drift and zero at rest / pure
    sway / pure yaw, exactly where the separated half is the right
    physics. **Ahead only** — the model assumes clean flow developing
    from the leading end; that's textbook for the fine bow entry but
    false making sternway (the aft "leading end" carries the deflected
    rudder, turning prop and aperture; astern derivatives are measured,
    not slender-body-derived, in the literature too). Found empirically
    first: an astern branch strangled the backing turn (18.6 m → 35.8 m
    for 90° at 2.5 kn against a real 8–16 m benchmark) via its
    u-proportional yaw damping — backing agility really does live in the
    rudder's wind-up instability plus cross-flow drag, which is what the
    sim already had right. Measured effect of the attached half
    (O'Day 39, full rudder at 2.5 kn, engine neutral): 90° ahead in
    26.1 m (~2.2 boat lengths, real fin-keeler benchmark ~2) vs. NEVER
    without it (75° after 34 m, still going); backing bit-identical
    (`a_forward_turn_carves_instead_of_ploughing` locks the ahead
    benchmark in-suite). Watch out when reproducing turn measurements:
    the berth spawn is 2.4 m off the quay and a starboard turn's first
    move is the stern swinging INTO it — the collision impulse reads
    exactly like a physics bug until you print the hull corners
    (`set_pose` exists test-side for open-water arenas because of this).
    **The benchmark protocol matters as much as the model** (2026-08-04,
    measured while investigating "not quite around the mast yet"; don't
    chase these gaps with model changes, they're helmsmanship): slamming
    the helm hard-over STALLS the blade (35° ≫ the ~17° stall onset —
    lift collapses to ~¼ of the attached peak until the yaw sweep
    catches up), so a 2 s progressive lead-in alone tightens 90° at
    2.5 kn from 24.8 m to 21.6 m, matching the real-world "lead the boat
    smoothly into the turn" instinct; adding a full-throttle burst over
    the deflected blade (prop wash) gives 17.5 m ≈ 1.5 boat lengths —
    the real tight-harbour-turn technique, and the sim now rewards it
    for the real reasons. Ruled OUT while investigating, with numbers:
    the rudder's position (its washout kinematics permit a ~9 m radius;
    the measured average is 16.6 m — not the binding constraint, and
    moving the blade to the O'Day's slightly-forward real post position
    would cut the entry moment 15% for a limit we never reach) and the
    yaw inertia (Rapier's uniform hull spread happens to land at
    gyradius 0.27·LOA, inside the real 0.25–0.30·LOA sailboat band).
    (The specific turn distances in this note are from the in-basin,
    shared-blade era they were measured in — the canonical per-boat
    numbers now live in docs/reference-boats.md's open-water table; the
    protocol lessons are what this note is for.)
- **Open-water benchmarks & pins** (2026-08-07, sim.rs tests): the
  shipped harbour bounds or obstructs any long benchmark run (in the
  pre-marina basin it was a hard ~30 m cap — the old handling table's
  "— ran out of basin" cells — and coasting used to be "verified" by
  re-integrating the tick() formulas OFFLINE; in today's marina a long
  run must dodge berths, poles and shores instead). `Sim::new_open_water`
  (`#[cfg(test)]`, same construction minus ALL harbour colliders — the
  shipped world and its fixed collider-insertion order are untouched)
  gives unbounded water, so every benchmark runs through the real
  `tick`. `measure_open_water_benchmarks` (`#[ignore]`d harness; run
  with `--ignored --nocapture`) regenerates the measured-performance
  table in docs/reference-boats.md; `open_water_benchmarks_stay_pinned`
  pins every cell (±2% speeds, ±5% distances, ±3° capped headings) so a
  physics change that shifts a boat's character fails CI until the table
  is updated in the same commit. **This machinery caught a real one on
  its first run**: the offline integration's top speeds (6.5–6.7 kn,
  "~85% of hull speed") are not reproducible by the shipped `tick` —
  the wave term alone exceeds available thrust there against the real
  waterlines; the offline copy had been calibrated on the old shared
  11.9 m outline. Shipped truth: 5.5–5.9 kn, 71–76% of hull speed
  (retraction + open calibration question recorded in
  docs/reference-boats.md). Top-speed protocol note: the benchmark holds
  course with a small P-D helm — hands-off, prop walk curls a full-power
  run into a slow circle that reads ~0.8 kn low.
- **Force application points create the characteristic behaviours**: lateral
  wind force acts slightly FORWARD of centre (`WIND_CENTER_OFFSET > 0`, bow
  windage → the bow falls off downwind). Tune behaviour there, not with
  fudge torques.
- **Axial windage is asymmetric fore/aft, unlike the water-drag terms**:
  `CD_AIR_BOW` (fine entry, sprayhood raked to deflect a headwind) is well
  below `CD_AIR_STERN` (wide stern, and a following wind finds the
  sprayhood's open concave side and scoops into it instead of being
  deflected). Selected in `tick` by the sign of the relative wind's axial
  component — a single symmetric coefficient here would silently assume
  the boat is shaped the same front and back, which it visibly isn't (see
  the Boat bullet above). `a_following_wind_pushes_harder_than_a_headwind_
  of_the_same_speed` pins the direction of this asymmetry.
- **Keel profile (`sim-core/src/keel.rs`)**: the lateral water force's lever
  arm and the yaw damping coefficient used to be two independently
  hand-tuned constants (`WATER_CLR_OFFSET`, `C_YAW_Q`) — but they're both
  moments of the *same* physical thing, the underwater lateral-area
  distribution along the hull, so tuning them separately could produce a
  combination no real keel shape would give (e.g. a fin keel's small lever
  arm paired with a full keel's yaw damping). `KeelProfile` (piecewise-linear
  area-per-length vs. hull position) is now the single source of truth:
  `KeelProfile::derive()` integrates it (trapezoidal rule) into
  `KeelDerived { area, clr_offset, cubic_moment, swept_moment }`, stored on
  `Sim` and used in `tick` in place of the old constants. The physical
  reasoning: a strip at distance `x` from the pivot sweeps sideways at
  `w·x` during yaw, and drag is quadratic in speed, so its torque
  contribution scales as `x³` — concentrating area near the pivot (fin
  keel) trades away yaw damping much faster than it trades away total
  area, which is *why* fin keels spin freely and full keels don't. The
  same sweep also yields a SIGNED `x·|x|` moment (`swept_moment`): when
  the area is biased fore/aft, the strips resisting a spin don't pull
  symmetrically, so rotation produces a net SIDE FORCE, not just the
  damping torque — spin an aft-biased hull clockwise and the stern
  out-drags the bow, shoving the boat to starboard; that's what puts the
  effective centre of rotation aft of the centre of mass (`clr_offset`
  and `swept_moment` are the two off-diagonal sway↔yaw couplings of the
  same damping matrix). `Sim::new()` uses the Hallberg-Rassy 38 preset
  (`BoatDesign::hallberg_rassy_38()` — see `boat.rs` and
  `docs/reference-boats.md`); `Sim::new_with_design(&BoatDesign)` takes
  any design (curve + displacement — used by the R-reset key and initial
  construction); `Sim::new_continuing` does the same but transplants
  pose, velocity, and engine spool from the predecessor (used by the
  keel editor's Apply);
  `Sim::new_with_keel(&profile)` is the custom-curve-default-weight
  convenience the keel-coupling tests use.
  **The presets were renamed after real boats and re-drawn against their
  published specs** (2026-08-04, replacing `default_sailboat()`/
  `fin_keel()`/`long_keel()`): the curves are now capped at each boat's
  real draft (area-per-length at a station = local draught in metres),
  the fin moved from dead-centre to ≈0.6 m aft of centre (real fin-keeler
  geometry — the old position read as too far forward), and each preset
  carries its boat's real displacement. Consequence of honest end-fading
  curves: the default's yaw damping dropped from ≈353k to ≈182k
  N·m/(rad/s)² (both in that comparison's flat-Cd, full-outline metric;
  today's Cd-weighted figure on the real waterline is 75k — see
  reference-boats.md) (the old hand-tuned curve painted 0.7–1.0 m of draught at
  the extreme hull ends, which the cubic weighting amplifies) — the live
  rudder foil provides the rest of the spin resistance, as it should.
  **The rudder is no longer part of any preset** (2026-08-03) — the old
  fin preset painted it as a fixed area strip at the stern, which
  double-counted it against the live rudder foil in `sim.rs` (see the
  Rudder bullet above); that strip is gone. Whether the aft-biased
  presets implicitly bake in some rudder footprint was resolved by the
  2026-08-04 re-draw from real hull profiles (skeg/heel painted
  explicitly, rudder excluded).
  **The profile's Cd is no longer one flat number** (2026-08-04): a
  fin/skeg keel is thin, flat-plate-like material broadside to the flow
  (`CD_KEEL_PLATE = 1.2`), while the hull's own canoe body is round
  (`CD_ROUND_HULL = 1.1`, the circular-cylinder cross-flow analogy).
  **Second pass, same day (maintainer-sourced against the standard drag
  tables — NASA shape-effects / Hoerner)**: the first pass used 1.98 for
  the plate material, which is the 2D/INFINITE-plate limit; real keels
  sit at finite mirrored aspect ratios (~1 for a chordy fin to ~8 for a
  full keel read as one slender plate — "mirrored" because the hull
  end-plates the root, same doubling as `RUDDER_AR`), which Hoerner's
  finite-plate data caps at ~1.13–1.25 (`flat_plate_cd(ar) = 1.11 +
  0.018·ar`, the Viterna–Corrigan ceiling, now the shared function in
  `keel.rs`; the rudder's `RUDDER_CD_MAX` evaluates it at the blade's own
  AR ≈ 5 → 1.20). Honest consequence: at real aspect ratios the two
  materials nearly converge (1.2 vs 1.1) and the split's numeric bite
  mostly collapses — what survives is the structure plus the one robust
  physical asymmetry, Re-dependence: sharp-edged plate material has no
  drag crisis (separation fixed at the edges), while a smooth round bilge
  crosses the cylinder drag-crisis band (Re ≈ 2–5·10⁵, Cd → ~0.3–0.7)
  right in this sim's sway-speed range — `CD_ROUND_HULL` keeps the
  subcritical value with that uncertainty documented on the constant as
  the upgrade path. (Also worth knowing: the "streamlined half-body
  Cd ≈ 0.09" from those same tables is AXIAL flow, not broadside — and
  the surge model already lands there for free: backing an effective
  frontal-area Cd out of the ITTC friction model at 3 kn gives ≈ 0.06.)
  `derive()`
  splits each station's depth at `HULL_BASELINE_DRAFT = 0.5` m (the
  judgement-call constant here, like `HULL_FORM_FACTOR` below — a keel
  bolts onto the BOTTOM of the hull, so a deep station's profile passes
  through this much rounded hull shell before it's keel material; not
  picked blind — it sits just below the "just canoe body, no fin/skeg"
  shoulder depth the presets already draw in `boat.rs`, 0.6 m for the
  Hallberg-Rassy 38 and 0.55 m for the O'Day 39, so those shoulders read
  as mostly hull rather than a too-thin baseline shaving off a chunk of
  them as flat-plate — an earlier 0.3 m pass undershot this on a first
  guess) and Cd-weights the two parts separately, producing `drag_area`/
  `drag_clr_offset`/`drag_cubic_moment`/`drag_swept_moment` alongside the
  existing pure-geometry fields (`area`/`clr_offset`/`cubic_moment`/
  `swept_moment`, kept unweighted for callers that want the real physical
  shape, e.g. boat-to-boat area comparisons in `boat.rs`'s tests). `tick`
  uses only the `drag_*` fields now — `CD_WATER_LAT` as a flat constant is
  gone from `sim.rs` entirely, folded into `keel.rs`'s per-station
  weighting instead.
- **Determinism rules (inherited verbatim from Pegasus)**: fresh `Sim` per
  run — never reuse one across runs (Rapier handle numbering / warm-start
  caches); all forces inside `tick` only; no wall clock, no `gen_range`, no
  macroquad in sim-core. `same_input_sequence_is_bit_identical` unit-tests
  the property that will make replays possible (scripting `Env` AND
  `InputState`, so the engine spool state is covered too).

## Frontend conventions (src/main.rs)
- **Mobile-first UI**: design and test every UI feature for touch/phone
  screens first. That doesn't exclude desktop — but the two UIs must stay
  **on par feature-wise**: anything reachable with a keyboard/mouse needs a
  touch equivalent and vice versa (the KEEL button existing because E has
  no touch equivalent is the canonical example). If a richer,
  more detailed desktop UI ever seems warranted, that divergence must be
  discussed and agreed by the maintainers first — don't let the two drift
  apart in ordinary feature work.
- **Units (verified against the vendored macroquad 0.4.15 source)**:
  `screen_width()/screen_height()` and `mouse_position()` are LOGICAL css px
  (physical / dpi); `touches()` returns RAW PHYSICAL px and every touch
  position is divided by `screen_dpi_scale()` before use (the Pegasus
  gotcha). HUD sizes are written directly in css px, clamped
  (`(min_dim * k).clamp(lo, hi)`) — no more `ui` multiplier.
- **Camera fills the screen and follows the boat**: `base_scale =
  max(sw/VIEW_MAX_W, sh/VIEW_MAX_H).min(sw/VIEW_MIN_W)` (defaults:
  never more than 150×85 m visible, never fewer than 30 m across),
  camera centred on the interpolated boat pose and clamped to the world
  rect. This is what makes portrait phones show a close-up instead of
  letterboxing the whole marina. `w2s` closure converts world → screen
  px (y inverted). **User zoom** (2026-08-05) multiplies `base_scale`:
  pinch on touch, scroll wheel or +/- keys on desktop (parity rule),
  clamped every frame so the visible width stays in
  `[ZOOM_IN_MIN_W = 24, ZOOM_OUT_MAX_W = 450]` m — re-clamped against
  the CURRENT window size (resize can't strand it) with 1.0 always
  admitted. Pinch = exactly two touches NOT claimed by any HUD control,
  tracked by sorted id pair (a control-claimed finger never pinches; a
  recycled or third finger ends the gesture instead of jumping the
  zoom). **Wheel-delta
  gotcha**: native reports ±1 per notch, web reports deltaY PIXELS
  (~±100 per notch) — deltas ≥40 are treated as pixels (/240), smaller
  ones as notches (×0.25), both bounded per event. Zoom is a camera
  preference: it survives R-reset and editor Apply. **Pan** (2026-08-05):
  ONE free finger dragging the water (or a mouse drag — `mouse_claim` 4)
  shifts a FOLLOW-OFFSET (`cam_offset`, world metres relative to the
  boat) — the camera keeps following the boat while panned, displaced
  by the offset (owner spec, second pass same day: a fixed-world-point
  anchor was tried first and replaced — it froze the view while the
  boat sailed off). Pan deltas convert screen→world via the PREVIOUS
  frame's scale (`last_scale` — input runs before the camera block).
  The offset is folded through a world-rect clamp of the target POINT
  each frame, so shoving against the edge racks up no invisible travel,
  and a zero offset can never turn nonzero on its own (the boat is
  always inside the world). Pinch/wheel zoom leaves the offset alone.
  A CENTER button (twin of the C key) appears left of the settings
  gear ONLY while the offset is >0.5 m; C, CENTER, R-reset and editor
  Apply all zero it
  (zoom persists throughout).
- **Touch controls**: the two HUD compass indicators are draggable **dials**
  (`Dial` struct) — drag direction from the dial centre = the flow's TOWARD
  direction (wind label still displays the mariners' FROM convention:
  from = to + 180°), drag distance = speed (rim = `WIND_MAX`/`CURRENT_MAX`,
  centre dead-zone = calm). The helm/engine are **sliders** (`Slider`
  struct) on the mid-left (throttle, vertical, up = ahead) and mid-right
  (rudder, horizontal, right = starboard helm) edges — the two-thumb zone;
  both HOLD where left (a real single-lever control / helm with friction —
  agreed in review, no spring-return) with a 10% centre detent and the
  dials' 1/20 quantisation, centred at `0.56·sh` to clear the dials+labels
  above and the buttons below down to ~360 px min-dim. A RESET button
  (bottom-right) twins the R key. Mouse drives the same controls via
  press/drag (`mouse_claim` discriminants: 0 wind, 1 current, 2 throttle,
  3 rudder). `simulate_mouse_with_touch(false)` at startup so touches
  don't double as mouse presses. **Touch claims are by
  id-not-seen-last-frame, NOT `TouchPhase::Started`** — touchstart
  collapses into the following touchmove whenever touch events outpace
  the frame loop (the hard-won Pegasus phase-collapse lesson; a `Started`
  phase on an already-claimed id means a recycled id = new finger, so the
  claim is dropped and re-evaluated). One `Option<u64>` claim per control
  is what makes simultaneous two-thumb throttle+rudder work.
- **LINES mode** (2026-08-20, `src/mooring.rs`): the mooring frontend, on
  the `T` key and the LINES button (the button exists for the same reason
  KEEL does — without it there's no way in on a touch-only device).
  Unlike the keel editor it does **NOT freeze the sim**: getting a line
  ashore is something you do with the boat still moving, so the handles
  have to work live.
  - **The gesture is deliberately forgiving**, because a fairlead is
    ~2 m from its neighbour and the camera is often zoomed out: a press
    near the hull takes the NEAREST handle or rope (see the
    nearest-wins bullet below), and one state
    machine accepts either a drag (fairlead → anchor) or two taps
    (fairlead, then anchor), so nobody has to be precise with a moving
    target. A drag that lands on nothing cancels; a TAP that lands on
    nothing leaves the fairlead armed for the second tap. Hit radii are
    `(scale*0.9).clamp(18, 42)` css px — floored so handles stay
    hittable when zoomed out.
  - **One claim covers every mooring gesture** (leading a line, holding
    HAUL/SLACK) — they're mutually exclusive by hand, so it's one
    `Option<Grab>` inside `MooringUi` plus one claim slot outside:
    `mouse_claim = Some(5)` and
    `mooring_touch: Option<(u64, Vec2)>`. The touch claim carries the
    finger's LAST SEEN POSITION because `touches()` drops a lifted id
    without reporting a final position, and where the rope was let go is
    exactly what decides whether it lands on a cleat.
  - **Input runs before the camera block**, so world↔screen hit-testing
    uses `last_cam`/`last_scale`/`last_boat` — last frame's view and
    INTERPOLATED pose, i.e. what the player was actually looking at when
    they pressed. (`last_scale` already existed for the pan code; the
    other two are its companions.)
  - **Held controls repeat, one-shots queue.** `next_command` drains a
    queued `MakeFast`/`CastOff` first, otherwise re-issues `Tend` for as
    long as HAUL or SLACK is held — that's how a continuous pull is
    expressed in a one-order-per-tick input stream. Sliding a finger off
    a tend button stops the pull, like a real press-and-hold control.
  - **Picking a rope vs picking a handle: NEAREST WINS, not a fixed
    priority** (bug found by the owner, 2026-08-20). Checking fairleads
    first made ropes unselectable near the boat, because every rope's
    inboard end sits exactly ON a fairlead — the handle swallowed the
    press every time. Two fixes, both needed: `press` now measures the
    distance to the nearest handle AND to the nearest rope and takes
    whichever is genuinely closer, and `dist_to_rope` hit-tests the rope
    as DRAWN, bight and all, instead of the straight chord — a slack line
    bulges off to one side, so tapping the rope you can see was missing
    it entirely. A rope gets the more generous radius of the two (it is a
    long thin target; a handle is a fat round one), which nearest-wins
    keeps honest. Ties go to the HANDLE (`df <= dr`), which is what
    makes doubling up reachable now that sim-core allows it: a press
    right ON an occupied fairlead arms it for a second rope, while a
    press further along the rope selects the rope.
  - **Refusals say why.** An order that silently does nothing is the
    worst kind, and the reach limit in particular is invisible until
    something mentions it. While a fairlead is armed its REACH is drawn
    as a ring on the water and the rubber band turns red the moment the
    pointer leaves it; a release outside it, a rope that would repeat one
    already there ("the stbd waist already has a line to there"), and no
    line left to spare each put a short note in the status line
    (`MooringUi::note`, ~2.6 s). A line that vanishes while still
    `Passing` reports itself as a throw that fell short. The frontend
    re-checks the same three rules sim-core enforces — not to enforce
    them, which is `tick`'s job, but so the refusal has a reason
    attached.
  - **The tend buttons follow the SELECTION, never the status line**: a
    transient note must not make live controls vanish from under a thumb.
  - **Drawing**: `draw_ropes` draws EVERY rope in the marina, the
    player's and the fleet's, through one path — they are the same thing,
    so they are drawn by the same code (the fleet's just quieter, and
    culled against the camera: a few hundred ropes exist and a berth or
    two is ever on screen). Ropes are drawn whether or not the mode is
    open (once a line is out it's part of the world), BEFORE the hull so the end tucks
    under the deck edge at its fairlead — the same ordering trick as the
    rudder blade. A slack line bights out to the side by the parabolic
    arc-length relation `h = sqrt(3*d*slack/8)`, so the bight grows the
    way real rope does as you close on the cleat rather than by an
    invented curve; in a top-down view the bight has to lie somewhere, so
    it's pushed to whichever side is away from the boat (rope draped over
    the deck reads as a mistake). Tension tints and thickens the rope; a
    line still going ashore snakes out with its end running ahead of it.
    Cleats are drawn as studs on the pontoon faces from
    `cleat_positions()`, so a stud you can see is a stud you can reach.
  - The crew's line-passing speed is a SETTING and lives in the settings
    menu (`src/settings.rs`), not in this panel — the mooring panel
    carries only the controls you use while handling a rope.
    `hud_button` is the shared plate/rim/label helper the whole bottom
    row uses.
- **Safe-area insets**: `index.html` resolves `env(safe-area-inset-*)` via a
  hidden probe element (+ folds the floating-toolbar height in via
  `visualViewport`) and pushes css px into the wasm export
  `set_safe_area(t,l,b,r)` (atomics, re-pushed on resize/orientation
  change). The HUD layout adds them to its margins; native builds stay 0.
  **Gotcha (iOS Safari, 2026-08-02)**: the canvas is sized `100dvh` (with a
  `100vh` fallback) because iOS defines `100vh` as the toolbar-COLLAPSED
  viewport and a non-scrolling page never collapses the toolbar — a 100vh
  canvas keeps its bottom strip permanently behind the address bar, hiding
  the KEEL/RESET buttons. The toolbar overlap fold-in must compare the
  canvas's `getBoundingClientRect().bottom` against
  `visualViewport.offsetTop + height`, NOT `window.innerHeight` — on iOS
  `innerHeight` shrinks with the visible area, so the difference reads 0.
- Cosmetic-only nondeterminism is allowed render-side (the water ripples
  and the prop-wash foam streaks use `get_time()`); nothing cosmetic may
  feed back into the sim. The wash streaks READ sim state (`sim.engine()`,
  so they fade with the spool lag) and follow the deflected blade ahead /
  boil forward along the quarters astern; the rudder blade itself is drawn
  BEFORE the hull fill (root under the counter), swinging by the same
  blade-angle formula sim-core uses. The churned-water trail
  (`src/wake.rs`, see Project structure) is the other render-side
  reader of sim state: it draws right after the ripples, so it sits on
  the water and under the land fills, jetties, moored boats, the ropes
  and the player's own hull.
- Controls: touch/mouse = drag the dials/sliders + the bottom row's
  RESET/KEEL/LINES buttons and settings gear (+ CENTER while panned,
  leftmost of the row), pinch = zoom, one-finger/mouse drag on the
  water = pan, scroll wheel / +/- keys = zoom and C = centre (desktop
  twins);
  keyboard = **the boat has the primary keys** (agreed 2026-08-03: driving
  is the main activity): W/S throttle up/down, A/D helm port/starboard
  (continuous `is_key_down`×dt like the env keys), Space = engine to
  neutral (edge-triggered). Wind keeps ←/→ dir + ↑/↓ speed; current sits
  on the IJKL "second arrows" cluster (J/L dir, I/K speed) — which is why
  the keel editor moved from K to **E** (K = current speed down now).
  **T** opens LINES mode (mooring lines — see above), **O** the settings
  menu. R
  reset (reset = `respawn(&design)`, a fresh `Sim::new_with_design`,
  never an in-place teleport; env is kept but **helm/engine reset to
  `InputState::NEUTRAL`** — a fresh boat doesn't inherit a live
  telegraph), E keel design editor (freezes physics — all input and the
  physics tick, not just rendering — while open; Apply builds a fresh
  `Sim` via `Sim::new_continuing` which keeps position, heading,
  velocity, engine spool, and helm/engine input — the user sees the
  hydrodynamic effect of a keel change in place; see `src/keel_editor.rs`).
  The KEEL button exists because E has no touch equivalent otherwise —
  without it there'd be no way to reach the editor on a touch-only device.
  Once open, the editor itself takes touch input too (`KeelEditor::update`'s
  own `touches()` handling, independent of the HUD's — mirrors the same
  fresh-touch-id pattern as the dials, since
  `simulate_mouse_with_touch(false)` means touches never synthesize a
  mouse press).

## Roadmap (agreed direction, not yet built)
- **Adjustable mass distribution** (agreed 2026-08-04, separate PR from
  the displacement work): make the centre of mass and the radius of
  gyration designer-adjustable alongside `displacement_kg` in
  `BoatDesign`. Today Rapier spreads the displacement uniformly over the
  hull shape (`ColliderBuilder::mass`), which fixes both; a real boat's
  ballast keel concentrates mass low and central (smaller gyradius than
  uniform) and its COM is not necessarily at the hull centroid. Rapier
  supports it via explicit `MassProperties` on the collider/body.
- **Ship types**: right now `Sim`/the renderer always build the one small
  cruising sailboat described under Simulation model — hull geometry
  (`HULL_PTS`), windage coefficients (`CD_AIR_BOW`/
  `CD_AIR_STERN`, `WIND_AREA_*`), and the deck rendering are all plain
  constants/
  functions, not behind any ship-type abstraction (`BoatDesign` varies
  only the keel curve and displacement on that shared hull — its presets
  are 38-foot sailboat configurations, not ship types). The agreed
  direction is
  to support a small number of other small-vessel types later (starting
  candidate: a plain workboat, which is what this sailboat itself replaced
  — see git history) by giving each its own set of these, picked at
  `Sim::new`/spawn time. Deliberately not built yet: a trait or enum for a
  single existing variant would be speculative generality (see this file's
  own rule against designing for hypothetical future requirements); add
  the abstraction once a second ship type actually needs to coexist with
  the first.
- **Ropes**: DONE 2026-08-20 (see Mooring lines under Simulation model and
  LINES mode under Frontend conventions), including the moored fleet —
  the whole marina lies to real ropes on dynamic hulls. What the rope
  work leaves open: line MATERIAL is fixed
  at 14 mm three-strand nylon (polyester would be a per-line choice, and
  the curve is already the only thing that would vary); and lines pass
  through poles, hulls and jetties without contact or friction (no turn
  round a pole, no catenary weight — the top-down 2D simplifications
  documented on the module).
- **Scenarios and scoring**: approach, spring off a lee quay, …;
  recordings/replays (the Pegasus hybrid format) — the input stream now
  carries line orders too, so a replay would reproduce the rope work.

## License
GPL-3.0-or-later (deliberate choice, 2026-08-02, formalising the field the
Cargo.tomls carried from the start). Canonical GPLv3 text in `LICENSE`;
both Cargo.toml `license` fields must stay `"GPL-3.0-or-later"`. Because
GPL makes later relicensing need every contributor's consent, contributors
sign the lightweight CLA in `CLA.md` (copyright stays theirs; the project
gets a broad license incl. relicensing rights) — agreement is a one-line
PR statement or a signature added to CLA.md, per its §6. The vendored
`mq_js_bundle.js` is MIT OR Apache-2.0 (GPL-compatible) and carries a
required attribution header — keep it when replacing the bundle.

## Git workflow
- Development branch: `claude/harbour-sim-feature-aokq29` (current).
- Same rules as Pegasus: curate branches before rebase-merging to `main`;
  the wasm binary is **not tracked** (gitignored) — deploy builds it from
  source; `git fetch origin main && git rebase origin/main` before PRs.
- **Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/)**:
  `<type>[optional scope]: <description>`, e.g. `fix(keel): clamp yaw damping
  to a minimum`. Common types here: `feat`, `fix`, `docs`, `refactor`,
  `test`, `chore`, `ci`, `perf`. Breaking changes get a `!` before the colon
  (`feat!: ...`) or a `BREAKING CHANGE:` footer. This applies to every
  commit, not just the final one on a branch — squash-merges take their
  message from the PR, but intermediate commits still get read individually
  during review and bisection.
- **PR titles follow the same convention** where the hosting platform
  allows it (GitHub does — the title becomes the squash-merge commit
  message), so a PR should be titled like a Conventional Commits subject
  line too, e.g. `feat(ropes): add bow line fairlead`.
