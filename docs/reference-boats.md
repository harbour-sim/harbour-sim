# Reference boats

The keel editor's five preset buttons are named after real boats. Each
preset (`sim-core/src/boat.rs`, `BoatDesign`) sets three things: the
underwater lateral-area curve (`KeelProfile`, carrying the boat's real
waterline extent), the rudder blade (`RudderDesign`), and the
displacement. This file records the published specifications those
presets are built from, and exactly how much of each real boat the sim
does — and does not — take.

## What a preset does and does not model

A `BoatDesign` varies **the keel curve, the rudder blade, and the
displacement** on the sim's single ~38 ft hull. Shared between all presets
(constants in `sim-core/src/sim.rs`):

- hull outline `HULL_PTS` (~12 m × 3.8 m) and, with it, the collider shape,
  the mass *distribution* (Rapier spreads the displacement uniformly over
  the hull shape — adjustable COM/radius of gyration is agreed follow-up
  work), and the deck rendering;
- windage areas/coefficients, the ~28 hp auxiliary and its prop (the
  prop's *position* follows the design's rudder — it sits a fixed
  clearance ahead of the blade, as on every real boat here, so the blade
  always stands in the wash).

Rudder blades are per-preset since 2026-08-04 (`RudderDesign`: position,
chord, depth, whether the root is end-plated by the hull, and — since
2026-09-11 — the LAYOUT: one blade on the centreline or a pair set out
either side of it). Published
blade dimensions essentially don't exist for production boats — the
O'Day's, from a replacement-rudder listing, is the anchor; the others are
derived from rudder type, each boat's own profile drawing/painted curve,
and the rudder-as-%-of-lateral-plane cross-check. A transom-hung blade's
root breaks the surface with air above it, so it gets NO end-plate mirror
— effective AR depth/chord instead of 2×, the honest reason a barn-door
rudder is mushier per square metre than a spade.

**Waterlines are real too** (2026-08-04): each preset's curve paints zero
draught over its boat's overhangs, so the curve's nonzero support IS the
boat's waterline at its published LWL (the Alajuela's deadwood ends in a
vertical sternpost cliff at full draught — a full keel's waterline does
not fade to zero aft). Reynolds/Froude numbers, hull speed, wave
resistance and wetted surface all read the design's own waterline length,
which is what produces the per-boat top speeds and per-boat carrying of
way in the measured-performance table below (each 71–76% of its own hull
speed on the shared 28 hp; 3→1 kn coasting from 110 m for the O'Day up
to 129 m for the heavy Alajuela — see that table for the retraction of
the earlier offline-integrated figures). Rudders sit relative to their
own waterline endings: spades
with the trailing edge at the aft ending, the Alajuela's outboard blade
hanging entirely abaft its sternpost. The overhang split bow/stern within
the shared 12 m outline is approximate (read from profile drawings); the
LWL lengths themselves are the published figures, unit-tested.

So "Alajuela 38" gives you the Alajuela's keel plan and weight on the
shared hull — its handling character, not a survey-grade model of the boat.

The keel curve's unit makes the naming honest: area-per-length (m²/m) at a
station **is** the local depth of underwater lateral plane in metres, so no
preset is allowed to paint deeper than its boat's published draft
(unit-tested: `no_preset_paints_deeper_than_its_boat_draws`).

## The boats

| | Hallberg-Rassy 38 | O'Day 39 | Elan Impression 394 | Beneteau Oceanis 38.1 | Alajuela 38 |
|---|---|---|---|---|---|---|
| Role in the sim | **default** — the middle configuration | modern fin-keel cruiser/racer | contemporary cruiser, most agile | **twin rudders** — the modern volume cruiser | traditional long-keel cruiser |
| Keel / rudder | fin keel, skeg-hung rudder | fin keel, spade rudder | cast-iron fin, deep single spade rudder | bulbed iron fin, **twin spade rudders** | full keel, transom-hung rudder |
| Designer, years | Olle Enderlein / Christoph Rassy, 1977–1986 (202 built) | Philippe Briand, from 1982 | Rob Humphreys, from 2012 | Finot-Conq, from 2013 | William Atkin's *Ingrid* lineage (Colin Archer ancestry), 1977–1985 |
| LOA | 11.58 m (38.0 ft) | ≈12.0 m (39 ft class) | 11.90 m (39.0 ft) | 11.50 m (37.7 ft; hull 11.13 m) | 11.58 m (38.0 ft hull, excl. bowsprit) |
| LWL | 9.50 m | 10.21 m | 10.01 m | **10.72 m** (longest here) | 9.93 m |
| Beam | 3.47 m (11.4 ft) | 3.83 m (12.58 ft) | 3.91 m (12.8 ft) | **3.99 m** (13.1 ft) | 3.51 m (11.5 ft) |
| Draft | ≈1.75 m (5′9″) | 1.93 m (6.33 ft, standard keel) | 1.80 m (standard; 1.50 m shoal) | **2.08 m** (deep; 1.64 m shoal, 1.26–2.40 m lifting) | 1.83 m (6.0 ft) |
| Displacement | 18,739 lb ≈ **8,500 kg** | 18,000 lb ≈ **8,165 kg** | **8,000 kg** (17,637 lb) | **6,850 kg** (15,100 lb, lightest here) | 26,000 lb ≈ **11,800 kg** |
| Ballast | ≈44% ratio, encapsulated iron | 6,600 lb (2,994 kg) | 2,479 kg cast iron | 1,790 kg cast iron (deep keel; 2,060 kg shoal) | 10,000 lb (4,536 kg) lead |
| Rudder blade (sim) | 0.55×1.35 m at x −4.6, AR 4.9 (skeg-plated) | 0.61×1.52 m at x −4.9, AR 5.0 (hull-plated) — real replacement-blade dims | 0.60×1.65 m at x −4.75, AR 5.5 (hull-plated) | **two** 0.45×1.25 m at x −4.95, **±1.20 m off the centreline**, AR 5.6 each, 1.12 m² total | 0.55×1.55 m at x −5.38 (abaft the sternpost), AR 2.8 (transom-hung, no plate) |
| Top speed, sim's 28 hp | 5.6 kn | 5.9 kn | 5.8 kn | **6.1 kn** | 5.5 kn |
| Preset derives to | area 8.4 m², CLR −0.60 m, yaw damping 75 kN·m/(rad/s)² | area 6.6 m², CLR −0.30 m, yaw damping 44 kN·m/(rad/s)² | area 5.4 m², CLR −0.29 m, yaw damping 33 kN·m/(rad/s)² | area 5.3 m², CLR −0.32 m, yaw damping 33 kN·m/(rad/s)² | area 13.2 m², CLR −1.22 m, yaw damping 212 kN·m/(rad/s)² |

The derived numbers tell the expected story: the fin keelers concentrate
little area near the pivot and spin freely — the Elan most of all, with
the smallest lateral plane and the least yaw damping of the four despite
drawing *less* water than the O'Day (its agility comes from the modern
flat underbody stripping area off the hull ends, not from a deeper fin);
the full keeler carries the most area spread along the whole hull and
resists spinning ~6× harder than the Elan, with a strongly aft CLR
(weathervanes); the HR 38 sits between them. Note the classic long-keel
package deal visible in the raw specs: the Alajuela is *shallower* than
the O'Day despite far more keel area (spread along the hull, not down),
and ~40% heavier at the same length.

### Notes per preset

- **Hallberg-Rassy 38** (default): longish fin just aft of the hull
  centre at the full 1.75 m draft, a marked skeg ahead of the rudder
  post, canoe body fading to nothing at the bow. Replaces the old
  hand-tuned `default_sailboat()` (area 11.9 m², CLR −0.87 m): total area
  and CLR land close, but yaw damping drops (353k → 182k, both in the
  flat-Cd, full-outline metric of that comparison's day — today's
  Cd-weighted figure on the real 9.5 m waterline is the table's 75k)
  because the old
  curve painted 0.7–1.0 m of depth at the extreme hull ends, which the
  cubic weighting amplifies — the honest curve fades toward the tips.
- **O'Day 39**: already this repo's reference boat for the hull
  dimensions and the rudder blade, so the fin preset now matches the same
  boat the rudder foil was sized from. The fin is centred ≈0.6 m aft of
  the hull centre — the old unnamed fin preset sat exactly on the centre,
  which read as too far forward against real fin-keeler profile drawings
  (the root chord belongs around/abaft the mast, ≈40% LOA from the bow).
- **Elan Impression 394**: ~1.4 m-chord iron fin at the full 1.80 m
  draft, centred slightly aft of the hull centre like the O'Day's, over a
  markedly shallower canoe body (~0.4 m amidships) running out to a
  shallow, wide stern with no skeg; its deep single spade rudder maps
  onto the sim's stern blade. Spec caveat: the direct spec pages 403'd at
  collection time, so the figures were triangulated from search excerpts
  of the sources below — validated by internal consistency: LWL 10.01 m +
  8,000 kg reproduce the boat's widely-quoted D/L ratio of 222 exactly.
  (With the per-preset rudders it now carries its own big high-AR spade
  and is measurably the most agile of the four — see the handling table
  below; before that, steering with the shared O'Day blade, its 90°
  distances landed within ~3% of the O'Day's.)
- **Beneteau Oceanis 38.1** (the twin-rudder preset, 2026-09-11): a
  narrow (≈1.3 m chord) bulbed iron fin at the full 2.08 m draft — the
  deepest keel of the five — over a canoe body shallower even than the
  Elan's, because a chined flat-bottomed hull carries almost no lateral
  plane outside its fin. Two spade blades 1.20 m either side of the
  centreline. **That offset is the whole preset**: it is what takes both
  blades out of the propeller race, and everything that follows from it
  is derived rather than declared (see *Twin rudders* below). Her raw
  specs also make her the outlier at both ends of the sim's performance
  table — longest waterline and lightest displacement, so fastest under
  the shared 28 hp; and lightest, so the only preset that does not carry
  its way 100 m from 3 kn to 1 kn. Both are her published figures doing
  the work, not a modelling choice.
- **Alajuela 38**: cutaway forefoot deepening steadily aft to the heel at
  the rudder post; its real transom-hung rudder maps directly onto the
  sim's fixed stern-post blade at `RUDDER_X`.

### Twin rudders

The sim models the blades, not the reputation. `RudderDesign` carries a
`RudderLayout` — how many blades and how far off the centreline — and
`tick` runs the same foil law once per blade, at each blade's own point
and in each blade's own inflow. There is no twin-rudder branch anywhere
in the physics and no "has prop wash" flag. Three behaviours fall out of
the geometry:

1. **No steerage from a burst of ahead power.** The propeller's race
   contracts to `R/√2` (actuator-disc theory) — about 0.15 m for the
   16-inch auxiliary wheel — and the blades sit 1.20 m out, eight times
   that. `wash_fraction` compares the two, and the deflected-momentum
   term that lets every other preset kick her bow round while stopped
   goes to zero. **Measured**: 1.5 s of full throttle and full helm from
   rest swings the Elan's bow 5.7° and the Oceanis's 0.04°, a factor of
   ~140. This is the complaint every twin-rudder owner has about
   marinas, and here it costs you the sim's most useful harbour move.
2. **The throttle makes her turning circle WIDER, not tighter.** She is
   the only preset whose full-throttle 90° turn (17.7 m) is longer than
   her rudder-only one (16.7 m): the burst buys her speed and no steering,
   and speed widens a circle. Nothing models this; it is the arithmetic
   of (1).
3. **But she steers beautifully with way on** — tightest of the five on
   the rudder-only row. 1.12 m² of blade at AR 5.6 is a lot of rudder
   (≈22% of her small lateral plane, against the ~10% single-spade rule
   of thumb), which is exactly why builders fit twins that generous:
   they have no wash to help them, and heeled, one blade does most of
   the work.

A fourth, smaller effect is free from the rigid-body kinematics: a blade
off the centreline sees the fore-aft part of the yaw sweep, `w·(−y)`, so
spinning the hull drives the outboard blade of the pair forward through
the water while the inboard one is dragged back. A centreline blade
never sees it, which is why generalising cost the other four presets
nothing — bit for bit, confirmed by the pinned benchmarks.

**Known simplifications.** The blades are modeled upright, parallel and
in line abreast. Real twin rudders are canted outboard (so the leeward
one is vertical when the boat heels) and often slightly toed in — but
this is a top-down 2D sim with no heel, which is precisely the condition
under which cant does nothing, and toe-in without heel would be an
invented constant rather than a derived one. Nor does either blade
ventilate or lift clear of the surface, for the same reason. The one
consequence worth naming: the sim gives you the twin-rudder *marina*
handling in full, and none of the twin-rudder *sailing* payoff (a
rudder that still grips at 25° of heel), because it models no heel to
pay off.

## Measured performance (open water, 2026-08-07)

Measured through the real `tick()` in the test-only wall-free arena
(`Sim::new_open_water` — the shipped harbour is a closed 80 × 36 m
basin, which caps any benchmark run at ~30 m of path; the "— never
develops before running out of basin" cells in the 2026-08-04 revision
of this table were reporting the walls, not the boats). Every cell is
regenerated by the `measure_open_water_benchmarks` harness
(`cargo test -p harbour-sim-core --release -- --ignored --nocapture
measure_open_water`) and pinned in CI by
`open_water_benchmarks_stay_pinned` (±2% speeds, ±5% distances, ±3°
capped headings): a physics change that moves a number fails the build
until this table is updated with it, deliberately in the same commit.

Protocols (scripted, because the numbers are only reproducible if the
helmsmanship is): **top speed** — full throttle from rest, course held
by a small P-D helm (hands-off, prop walk curls the run into a slow
circle ~0.8 kn cheaper), read at 90 s ≈ 25 surge time constants;
**90° turn** — 2.5 kn ahead, full starboard helm fed in linearly over
2 s, engine neutral or full ahead, distance = path length when the
heading has swung 90° (a boat that never gets there is reported at the
90 s cap, by which it has nearly stopped and the heading has plateaued —
an asymptote, not a cutoff); **coasting** — engine neutral from 3 kn,
path length to 1 kn.

| | Theory / anchor | HR 38 | O'Day 39 | Elan I394 | Oceanis 38.1 | Alajuela 38 |
|---|---|---|---|---|---|
| Hull speed (kn) | 1.34·√LWL(ft), per boat → | 7.5 | 7.8 | 7.7 | 7.9 | 7.7 |
| Top speed, 28 hp (kn) | real ~38 ft auxiliaries: ~6.5–7 kn | 5.6 (75%) | 5.9 (75%) | 5.8 (76%) | **6.1 (77%)** | 5.5 (71%) |
| 90°, rudder only (m) | fin keeler: ~2 boat lengths | 23.7 | 18.7 | 17.4 | **16.7** | plateaus at 44° (67 m) |
| 90°, full-throttle burst (m) | | 18.4 | 16.4 | **15.8** | 17.7 *(worse than without)* | 24.9 |
| Coasting 3→1 kn (m) | real boats: >100 m above 1 kn | 111 | 110 | 113 | **93** | 129 |

The characters survive the move to open water, with two corrections to
the 2026-08-04 in-basin table. The HR 38 **does** complete a rudder-only
turn given room — its old "—" cell was the basin, not the boat. The
Alajuela's "—" was real: its heading genuinely plateaus around 44° as
the boat coasts to a stop, the transom-hung un-end-plated barn door
(AR 2.8) fading with V² before it can beat ~6× the Elan's yaw damping —
you plan your turns in a full keeler, and you turn it on the propeller
(24.9 m with the burst). The Elan stays tightest on both rows (big
high-AR spade, least yaw damping) of the single-rudder boats, with the
Oceanis tighter still once she has way on. The coasting row is the one
place a boat misses its real-world anchor: at 6,850 kg the Oceanis
stops in 93 m rather than past 100, which is the O'Day's 110 m scaled
by the mass ratio to within a metre — light modern cruisers not
carrying their way is a real complaint about them, so the anchor is
per-boat in the pins rather than one flat 100 m. The heavy full keeler
honestly carries its way farthest. Also measured: slamming the helm hard-over instead of feeding
it in can leave the marginal blades stalled indefinitely in a coast
turn — lead the boat into the turn.

**Retraction (2026-08-07)**: this file previously quoted top speeds of
6.5/6.7/6.7/6.0 kn ("each ~85% of its own hull speed") and coasting of
119–138 m, produced by re-integrating the tick() formulas *offline*.
The top-speed figures cannot be reproduced through the shipped `tick`:
against each preset's real waterline, the wave-making term alone
exceeds the available thrust at those speeds (the offline copy appears
to have been calibrated against the old shared 11.9 m outline
waterline — exactly the second-copy drift that motivated retiring it).
The table above is what the shipped formulas actually produce, measured
end-to-end. Whether 71–76% of hull speed is *physically* right for
28 hp — real ~38-footers under comparable auxiliary power are usually
quoted at ~6.5–7 kn, which would need a smaller `C_WAVE_SCALE` against
these shorter real waterlines — is an open calibration question; per
the no-invented-numbers rule it needs a verifiable anchor (e.g. the
DSYHS regression noted in sim.rs), not a constant tuned until the row
looks right.

## Sources

Published figures collected 2026-08-04 (2026-09-11 for the Oceanis) from:

- Hallberg-Rassy 38: [sailboatdata.com](https://sailboatdata.com/sailboat/hallberg-rassy-38/),
  [Hallberg-Rassy — previous models](https://www.hallberg-rassy.com/yachts/previous-models/hallberg-rassy-38),
  [sailboat-cruising.com review](https://www.sailboat-cruising.com/Hallberg-Rassy-38-review.html),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/hallberg-rassey-38-mk-ii/)
- O'Day 39: [Wikipedia](https://en.wikipedia.org/wiki/O%27Day_39),
  [sailboatdata.com](https://sailboatdata.com/sailboat/oday-39/),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/oday-39/)
- Alajuela 38: [Wikipedia](https://en.wikipedia.org/wiki/Alajuela_38),
  [sailboatdata.com](https://sailboatdata.com/sailboat/alajuela-38/),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/alajuela-38/)
- Elan Impression 394 (collected 2026-08-04, via search excerpts — see
  the preset note above):
  [sailboatdata.com](https://sailboatdata.com/sailboat/impression-394-elan/),
  [sailboat.guide](https://sailboat.guide/elan/impression-394),
  [boats.com review](https://www.boats.com/reviews/elan-impression-394-a-new-edition/),
  [official Elan brochure (yachthub mirror)](https://imgs.yachthub.com/showroom/5/2/1/8/Imp394-brochure1.pdf),
  [YBW forum thread with the D/L figure](https://forums.ybw.com/threads/elan-impression-40-lwl.473246/)

- Beneteau Oceanis 38.1 (collected 2026-09-11, via search excerpts —
  sailboatdata.com and boat-specs.com were unreachable from the build
  environment, so the figures below are the ones on which the reachable
  sources agree):
  [Flagstaff Marine spec table](https://flagstaffmarine.com.au/beneteau-oceanis-38-1/),
  [Beneteau's own model page](https://www.beneteau.com/en-us/oceanis-2005-2014/oceanis-381),
  [boats.com review](https://uk.boats.com/reviews/beneteau-oceanis-38-1-review/),
  [The Sailboat Guide](https://www.bolsadenavegantes.net/en/sailboats/beneteau/beneteau-oceanis-38-1-sailboat-data-and-in-depth-review/),
  [boat-specs.com, deep-draft variant](https://www.boat-specs.com/sailing/sailboats/beneteau/oceanis-38-1-deep-draft)
  (listed for the record; not fetched)

On twin-rudder handling under power (the behaviour section above is
checked against these, not derived from them — the physics derives it
from the offset):
[SAIL, *Docking with Twin Rudders*](https://sailmagazine.com/cruising/boat-handling-docking-with-twin-rudders/),
[Attainable Adventure Cruising Q&A](https://www.morganscloud.com/2017/11/16/qa-coming-alongside-docking-with-twin-rudders/),
[NauticEd, *Dual Rudder: Maneuvering Under Power*](https://sailing-blog.nauticed.org/dual-rudder-maneuvering-under-power/),
[Modern Sailing, *Docking a Dual Rudder Sailboat*](https://www.modernsailing.com/article/docking-dual-rudder-sailboat),
[Hamble School of Yachting, *Life without prop wash*](https://www.hamble.co.uk/pdfs/twin-rud.pdf).

Displacement figures vary a little between sources (e.g. the Alajuela 38
is listed at 26,000 lb or 27,000 lb depending on source/mark); the presets
use the most commonly cited figure. Draft figures are for the standard
keel where a shoal option existed.
