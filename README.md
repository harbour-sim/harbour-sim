# Harbour Sim

A harbour mooring simulator: a small sailboat under auxiliary engine in a
marina modeled on Hinsholmen (Långedrag, Gothenburg) — a gently curving
channel lined with a dozen pontoon jetties of pole berths, half of them
occupied, a rounded bay head at the top and an open run out to the sea at
the bottom — with wind and current to fight. Engine and rudder are fully
modeled — prop walk, prop wash over the rudder, rudder stall and all — so
real close-quarters technique works: back and fill, kick the bow round with
a burst of ahead power, feel the stern walk to port going astern. The plan
is to grow this into a game about mooring manoeuvres — placing ropes
(springs, breast lines, bow/stern lines) to make the boat move the way you
want under different conditions. No ropes yet.

**▶ Play it at <https://harbour-sim.github.io/harbour-sim/>**

Built on the same stack as [Pegasus](https://github.com/dannyrhubarb/pegasus):
Rust + macroquad + Rapier 2D, compiled to WebAssembly and served via GitHub
Pages.

## Controls

**Touch / mouse**: the vertical slider on the left edge is the engine
telegraph (up = ahead, down = astern, centre = neutral); the horizontal
slider on the right edge is the helm (right = starboard). Both hold where
you leave them, with a centre detent — drive with two thumbs. The
conditions panel in the top-left corner shows the wind and current; tap
it to open the **scenario modal**, where you set them — drag either
compass dial (drag direction is where the flow goes toward, distance is
speed, centre = calm), or tap one of the four condition presets (calm,
onshore, offshore, wind over tide), then Apply. The sim is paused while
the modal is open. Pinch anywhere on the water to zoom — from a
berth-threading close-up out to a ~450 m sweep of the marina (scroll
wheel does the same with a mouse) — and drag with one finger (or the
mouse) to pan — the camera keeps following the boat at the panned
offset, so you can watch your own approach from over the berth; a
CENTER button appears while panned to snap the view back onto the boat.
The RESET button returns the boat to its fairway start, engine to
neutral, helm amidships.

**Keyboard**:

| Keys | Effect |
|------|--------|
| W / S | Throttle up / down (ahead ↔ astern) |
| A / D | Helm to port / starboard |
| Space | Engine to neutral |
| V | Scenario modal (wind & current) — V again keeps the edits |
| + / - | Zoom in / out (also mouse wheel) |
| C | Centre the camera back on the boat |
| R | Reset the boat to its fairway start |
| E | Keel design editor |

Inside the scenario modal:

| Keys | Effect |
|------|--------|
| ← / → | Rotate wind direction |
| ↑ / ↓ | Wind speed |
| J / L | Rotate current direction |
| I / K | Current speed |
| 1 – 4 | Condition presets (calm, onshore, offshore, wind over tide) |
| Enter | Apply |
| Esc | Cancel |

## Build & run

```bash
cargo build               # native dev build (opens a window)
cargo run                 # play natively
cargo test --workspace    # unit tests (--workspace or sim-core's tests are skipped)
```

Web build (what actually deploys):

```bash
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/harbour-sim.wasm .
python3 -m http.server 8000   # then open http://localhost:8000/
```

## Deploy

Any push to `main` triggers the deploy workflow, which builds the wasm and
publishes to GitHub Pages. Every PR gets its own preview at `pr-<n>/`.
One-time repo setup: **Settings → Pages → Source = "GitHub Actions"**.

See `CLAUDE.md` for the architecture and pipeline details.

## License

Licensed under the GNU General Public License, version 3 or (at your
option) any later version — see [LICENSE](LICENSE) or
https://www.gnu.org/licenses/gpl-3.0.html.

The vendored `mq_js_bundle.js` is from the
[miniquad](https://github.com/not-fl3/miniquad) /
[quad-snd](https://github.com/not-fl3/quad-snd) projects, MIT OR
Apache-2.0 (GPL-compatible) — see the notice at the top of that file.

### Contribution

Contributions are welcome and require agreeing to the project's
Contributor License Agreement — see [CLA.md](CLA.md). In short: you keep
the copyright to your work and license it to the project broadly enough
that the maintainers can relicense later without tracking down every
past contributor. Agreeing is a one-line statement in your first pull
request.
