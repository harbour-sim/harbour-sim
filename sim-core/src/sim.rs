//! The deterministic harbour simulation: a boat floating in a marina
//! channel full of jetties, pole berths and moored boats, pushed around by
//! wind and current.
//!
//! Top-down 2D view, world units are metres, y points "north" (up on
//! screen), x east. There is no gravity — the vertical axis of the real
//! world is projected away; everything that keeps the boat in place is
//! hydrodynamic drag, aerodynamic (wind) load, and contact with the quay.
//!
//! Everything physical is advanced ONLY by `Sim::tick(&Env, &InputState)`
//! at a fixed `PHYSICS_DT`. The environment (`Env`) and the helm/engine
//! inputs (`InputState`) are passed per tick like an input stream — same
//! input sequence + fresh `Sim` => bit-identical trajectory (unit-tested),
//! which is what will make recordings/replays possible later exactly like
//! Pegasus.

use crate::boat::{BoatDesign, RudderDesign};
use crate::keel::{flat_plate_cd, KeelDerived, KeelProfile};
use glam::Vec2;
use rapier2d::prelude::*;

/// Fixed physics timestep (120 Hz), same as Pegasus.
pub const PHYSICS_DT: f32 = 1.0 / 120.0;

// ---------------------------------------------------------------------------
// Harbour geometry (all metres, shared with the renderer)
// ---------------------------------------------------------------------------
//
// Modeled on Hinsholmen marina (Långedrag, Gothenburg), mirrored in both
// axes from the aerial reference photos (owner request 2026-08-05): the
// channel now runs from a ROUNDED BAY HEAD in the NE down toward the SW,
// curving so its concave (inner) side faces UP — and the seaward end is
// OPEN: the shores diverge into a patch of open sea, closed only by a
// distant skerry line, so the boat can leave the marina entirely. The
// road shore with the long row of pontoon jetties lies on the outer
// (lower/SE) side of the curve; the sparse hill shore with its two
// jetties tucked up by the head is the inner side, deliberately left
// with open water. Boats berth PERPENDICULAR to the jetties: one end at
// the jetty face, the other tied between a PAIR of wooden mooring poles
// standing one boat-length off. Everything is GENERATED from the
// constants below by pure float math — the same numbers every call — so
// the colliders the Sim builds and the scenery the renderer draws can
// never disagree (`jetties()`, `pole_positions()`, `moored_boats()`,
// `road_shore()`, `hill_shore()`, `head_arc()` are the shared sources
// of truth).

/// Shore tangent bearing at the HEAD end (compass degrees, pointing SSW
/// down-channel toward the sea) and the extra bend added per jetty
/// station — marching seaward the bearing swings toward WSW, which is
/// what makes the channel's concave side face up (the mirrored curve of
/// the photos).
const SHORE_BEARING_HEAD_DEG: f32 = 207.0;
const SHORE_BEND_PER_STATION_DEG: f32 = 4.4;
/// Root of the head-most road jetty (station 0), placed so the marina
/// roughly centres on the world origin.
const ROAD_HEAD_ROOT: Vec2 = Vec2::new(280.0, 290.0);
/// Jetty roots along the road shore sit this far apart: two 20 m berths
/// (pole rows at 21.25 m off each centreline) plus a manoeuvring lane of
/// THREE BOAT LENGTHS (~37 m) between the opposing pole rows (owner
/// requirement, 2026-08-04 — the original 38 m spacing left a 10.5 m
/// lane, too tight to work a 12 m boat into a berth). The bend per
/// station above is scaled to the wider station so the channel's curve
/// radius stays the photo's.
const JETTY_SPACING: f32 = 80.0;
const N_ROAD_JETTIES: usize = 10;
/// Standard road-shore jetty; the SEAWARD-most (station 9, nearest the
/// entrance — the outermost) is the long outer pontoon of the photos.
const ROAD_JETTY_LEN: f32 = 34.0;
const OUTER_JETTY_LEN: f32 = 60.0;
/// The hill shore's two jetties, tucked up by the head (rooted opposite
/// the gaps between road stations 0-1 and 1-2) so the rest of the inner
/// shore stays open water.
const HILL_JETTY_LEN: f32 = 28.0;
const HILL_JETTY_STATIONS: [f32; 2] = [0.5, 1.5];
/// Road shore → hill shore, straight across. Widened 110 → 125 (owner
/// request 2026-08-05: leave room on the inner side): ~63 m of fairway
/// against the hill jetties' tips, ~91 m along the open inner shore.
const CHANNEL_W: f32 = 125.0;
/// Shore past the head-most jetty before the rounded head begins, and
/// past the seaward (outer) jetty before the coast opens out to sea.
const HEAD_MARGIN: f32 = 50.0;
const ENTRANCE_MARGIN: f32 = 24.0;
/// The rounded bay head capping the channel's NE end: a half-ellipse
/// bulging this far beyond the shores' ends (its lateral radius is
/// CHANNEL_W / 2) — big enough to turn a boat in.
const HEAD_BULGE: f32 = 70.0;
/// Past the entrance the two coasts DIVERGE into open sea: each pulls
/// away from the channel axis by the lateral offset (second element) at
/// the given along-tangent distance (first element). The far end is
/// closed by a skerry line — the world has to end somewhere — by which
/// point the water is ~335 m across.
const SEA_COAST: [(f32, f32); 3] = [(60.0, 18.0), (150.0, 55.0), (260.0, 105.0)];

/// 2.5 m wide pontoons.
pub const JETTY_HALF_W: f32 = 1.25;
/// Berth ("spot") size, owner spec 2026-08-04: 20 m long × 5 m wide —
/// the big-boat trot of the reference photos (boats up to ~50 ft on
/// long stern lines out to the poles), NOT the 30 ft spots of the first
/// zoomed photo. The 11.9 m sim boat berths with room to spare.
pub const BERTH_LEN: f32 = 20.0;
/// Pole rows flank each jetty a full berth length off the face.
pub const POLE_ROW_OFFSET: f32 = JETTY_HALF_W + BERTH_LEN;
/// Along-jetty spacing between poles: one 5 m wide berth per gap.
pub const POLE_SPACING: f32 = 5.0;
/// First pole this far from the root (berths stop short of the shore),
/// none closer than the tip clearance to the end.
const POLE_ROOT_CLEARANCE: f32 = 4.5;
const POLE_TIP_CLEARANCE: f32 = 0.5;
/// A ~30 cm wooden pile.
pub const POLE_RADIUS: f32 = 0.15;

/// Point + seaward tangent of the ROAD shore at arc distance `s` from
/// the head-most jetty root (negative = further up into the head).
/// Chord-marched one jetty station at a time — a few degrees per 80 m
/// chord reads as an arc, and jetty roots land exactly on the polyline's
/// kinks, so the wall collider and the jetty roots agree by construction.
fn road_shore_at(s: f32) -> (Vec2, Vec2) {
    if s <= 0.0 {
        let t = Env::compass_vec(SHORE_BEARING_HEAD_DEG);
        return (ROAD_HEAD_ROOT + t * s, t);
    }
    let mut p = ROAD_HEAD_ROOT;
    let mut bearing = SHORE_BEARING_HEAD_DEG;
    let mut remaining = s;
    while remaining > JETTY_SPACING {
        p += Env::compass_vec(bearing) * JETTY_SPACING;
        bearing += SHORE_BEND_PER_STATION_DEG;
        remaining -= JETTY_SPACING;
    }
    let t = Env::compass_vec(bearing);
    (p + t * remaining, t)
}

/// The into-the-water direction for a road-shore tangent (rotate 90°
/// clockwise: marching seaward at SSW gives a WNW out — toward the hill
/// shore on the curve's inner side).
fn out_of(t: Vec2) -> Vec2 {
    Vec2::new(t.y, -t.x)
}

/// One pontoon jetty: `root` on its shore, `dir` (unit) pointing out
/// into the water, `len` metres long.
#[derive(Clone, Copy, Debug)]
pub struct Jetty {
    pub root: Vec2,
    pub dir: Vec2,
    pub len: f32,
}

impl Jetty {
    /// Unit vector across the jetty (`dir` rotated +90°); berth axes and
    /// pole rows lie along ±side.
    pub fn side(&self) -> Vec2 {
        Vec2::new(-self.dir.y, self.dir.x)
    }

    /// Along-jetty distances of a flanking row's poles.
    pub fn pole_stations(&self) -> Vec<f32> {
        let n = ((self.len - POLE_ROOT_CLEARANCE - POLE_TIP_CLEARANCE) / POLE_SPACING) as usize
            + 1;
        (0..n).map(|k| POLE_ROOT_CLEARANCE + k as f32 * POLE_SPACING).collect()
    }
}

/// All jetties, in the FIXED order colliders are inserted in: the road
/// jetties head→sea (index 9 = the long outer one by the entrance),
/// then the two hill ones up by the head.
pub fn jetties() -> Vec<Jetty> {
    let mut v = Vec::with_capacity(N_ROAD_JETTIES + HILL_JETTY_STATIONS.len());
    for i in 0..N_ROAD_JETTIES {
        let (root, t) = road_shore_at(i as f32 * JETTY_SPACING);
        let len = if i == N_ROAD_JETTIES - 1 { OUTER_JETTY_LEN } else { ROAD_JETTY_LEN };
        v.push(Jetty { root, dir: out_of(t), len });
    }
    for &station in &HILL_JETTY_STATIONS {
        let (p, t) = road_shore_at(station * JETTY_SPACING);
        let out = out_of(t);
        v.push(Jetty { root: p + out * CHANNEL_W, dir: -out, len: HILL_JETTY_LEN });
    }
    v
}

/// Arc stations of the shore polylines' MARINA part: head margin, every
/// jetty root, entrance margin. The sea-coast points come after these —
/// see `marina_shore_len`.
fn shore_stations() -> Vec<f32> {
    let last = (N_ROAD_JETTIES - 1) as f32 * JETTY_SPACING;
    let mut v = Vec::with_capacity(N_ROAD_JETTIES + 2);
    v.push(-HEAD_MARGIN);
    for i in 0..N_ROAD_JETTIES {
        v.push(i as f32 * JETTY_SPACING);
    }
    v.push(last + ENTRANCE_MARGIN);
    v
}

/// How many leading points of `road_shore()`/`hill_shore()` belong to
/// the marina channel proper (head margin + jetty roots + entrance
/// margin); the points after them are the diverging sea coast. The
/// renderer needs the split to keep the quay apron off the wild coast.
pub fn marina_shore_len() -> usize {
    N_ROAD_JETTIES + 2
}

/// The road (SE, dock-carrying) shore polyline, head → sea, continuing
/// past the entrance as the diverging sea coast — also the wall
/// collider's shape.
pub fn road_shore() -> Vec<Vec2> {
    let mut v: Vec<Vec2> = shore_stations().iter().map(|&s| road_shore_at(s).0).collect();
    let (end, t) = road_shore_at(*shore_stations().last().unwrap());
    let out = out_of(t);
    for (along, spread) in SEA_COAST {
        v.push(end + t * along - out * spread);
    }
    v
}

/// The hill (NW, inner) shore polyline, head → sea: the road shore
/// pushed straight across the channel, its sea coast diverging the
/// other way.
pub fn hill_shore() -> Vec<Vec2> {
    let mut v: Vec<Vec2> = shore_stations()
        .iter()
        .map(|&s| {
            let (p, t) = road_shore_at(s);
            p + out_of(t) * CHANNEL_W
        })
        .collect();
    let (end, t) = road_shore_at(*shore_stations().last().unwrap());
    let out = out_of(t);
    for (along, spread) in SEA_COAST {
        v.push(end + out * CHANNEL_W + t * along + out * spread);
    }
    v
}

/// The rounded bay head: a half-ellipse from `road_shore()[0]` to
/// `hill_shore()[0]`, bulging `HEAD_BULGE` beyond the chord between
/// them. Part of the wall collider AND the renderer's water/land fill.
pub fn head_arc() -> Vec<Vec2> {
    let (road0, t) = road_shore_at(-HEAD_MARGIN);
    let hill0 = road0 + out_of(t) * CHANNEL_W;
    let center = (road0 + hill0) * 0.5;
    let a = road0 - center; // lateral radius, CHANNEL_W / 2
    let b = -t * HEAD_BULGE; // bulge beyond the chord
    let n = 12;
    (0..=n)
        .map(|k| {
            let th = std::f32::consts::PI * k as f32 / n as f32;
            center + a * th.cos() + b * th.sin()
        })
        .collect()
}

/// Axis-aligned bounds of the whole world (both shorelines, the sea
/// coast and the bay head) — the renderer's camera clamp.
pub fn world_bounds() -> (Vec2, Vec2) {
    let mut lo = Vec2::splat(f32::INFINITY);
    let mut hi = Vec2::splat(f32::NEG_INFINITY);
    for p in road_shore().into_iter().chain(hill_shore()).chain(head_arc()) {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    (lo, hi)
}

/// Every mooring pole, in collider-insertion order (jetty order, -side
/// row then +side row, root → tip). Shared with the renderer so drawn
/// poles ARE the colliders.
pub fn pole_positions() -> Vec<Vec2> {
    let mut poles = Vec::new();
    for j in jetties() {
        for side_sign in [-1.0f32, 1.0] {
            let row = j.side() * (side_sign * POLE_ROW_OFFSET);
            for d in j.pole_stations() {
                poles.push(j.root + j.dir * d + row);
            }
        }
    }
    poles
}

/// A boat parked in a berth (a static collider in the Sim). Everything
/// the renderer needs to draw it and its mooring lines is precomputed
/// here, so it never has to go searching for poles or jetty faces.
#[derive(Clone, Copy, Debug)]
pub struct MooredBoat {
    pub pos: Vec2,
    pub heading: f32,
    /// Unit berth axis: from the jetty face toward the pole pair.
    pub out: Vec2,
    /// Where its inboard breast lines land on the jetty face.
    pub jetty_face: Vec2,
    /// The pole pair its outboard end is tied between.
    pub poles: [Vec2; 2],
    /// Most boats lie bow-to-jetty like the photos; a few moor bow-out.
    pub bow_to_jetty: bool,
}

/// Which berths are taken — and how each occupant lies — comes from a
/// tiny hash of the berth's identity: deterministic (the same fleet
/// every run, an unchanging part of the world), no RNG state anywhere.
///
/// This must be a full avalanche mix ("lowbias32"), not a bare Knuth
/// multiply: a single multiply maps CONSECUTIVE berth indices to low
/// bytes a fixed stride apart, so the taken/free pattern repeated almost
/// periodically along every row and the marina looked machine-filled
/// (owner report, 2026-08-04). The xorshift rounds break that neighbour
/// correlation.
fn berth_hash(seed: u32) -> u32 {
    let mut h = seed.wrapping_add(0x9E37_79B9);
    h ^= h >> 16;
    h = h.wrapping_mul(0x7feb_352d);
    h ^= h >> 15;
    h = h.wrapping_mul(0x846c_a68b);
    h ^= h >> 16;
    h
}

/// The moored fleet, in collider-insertion order (same jetty/side/berth
/// nesting as `pole_positions`). A bit over half the berths are taken
/// (down from ~80% — owner request 2026-08-04: a marina you can actually
/// find a spot in); the free gaps are the berthing targets.
pub fn moored_boats() -> Vec<MooredBoat> {
    let mut v = Vec::new();
    for (ji, j) in jetties().iter().enumerate() {
        let stations = j.pole_stations();
        for (si, side_sign) in [-1.0f32, 1.0].into_iter().enumerate() {
            let out = j.side() * side_sign;
            for k in 0..stations.len().saturating_sub(1) {
                let h = berth_hash((ji as u32) << 12 | (si as u32) << 8 | k as u32);
                if (h & 0xff) >= 140 {
                    continue; // a free berth (~45% of them)
                }
                let bow_to_jetty = ((h >> 8) & 0xff) >= 38; // ~15% bow-out
                // A small deterministic lie per boat, so the trot doesn't
                // look machine-stamped.
                let slide = (((h >> 16) & 0xff) as f32 / 255.0 - 0.5) * 0.3;
                let yaw = (((h >> 24) & 0xff) as f32 / 255.0 - 0.5) * 0.05;
                let mid = (stations[k] + stations[k + 1]) * 0.5 + slide;
                // The inboard tip rides a fender's 0.3 m off the face;
                // bow tip is 6.0 m from the centre, stern tip 5.9.
                let half_len = if bow_to_jetty { 6.0 } else { 5.9 };
                let pos = j.root + j.dir * mid + out * (JETTY_HALF_W + 0.3 + half_len);
                let bow_dir = if bow_to_jetty { -out } else { out };
                let row = out * POLE_ROW_OFFSET;
                v.push(MooredBoat {
                    pos,
                    heading: bow_dir.y.atan2(bow_dir.x) + yaw,
                    out,
                    jetty_face: j.root + j.dir * mid + out * JETTY_HALF_W,
                    poles: [
                        j.root + j.dir * stations[k] + row,
                        j.root + j.dir * stations[k + 1] + row,
                    ],
                    bow_to_jetty,
                });
            }
        }
    }
    v
}

/// Where a fresh boat spawns: lying in the fairway between the head-end
/// jetties, bow pointing seaward down the channel (the whole run to the
/// open water ahead of it). 58 m off the road shore keeps clear of both
/// the road jetty tips (34 m) and the hill jetties' reach (97 m), with
/// the dock row just inside a phone's close-up frame.
pub fn start_pose() -> (Vec2, f32) {
    let (p, t) = road_shore_at(1.6 * JETTY_SPACING);
    (p + out_of(t) * 58.0, t.y.atan2(t.x))
}

// ---------------------------------------------------------------------------
// Boat geometry & physical constants
// ---------------------------------------------------------------------------

/// Hull outline in boat-local metres, bow = +x, CCW. Convex — used both as
/// the Rapier collider and by the renderer, so visuals match collision
/// exactly (the Pegasus alignment rule).
pub const HULL_PTS: [(f32, f32); 8] = [
    (6.0, 0.0),    // bow tip
    (4.2, 1.5),
    (-3.6, 1.9),
    (-5.6, 1.5),
    (-5.9, 0.0),
    (-5.6, -1.5),
    (-3.6, -1.9),
    (4.2, -1.5),
];

// The boat's MASS is no longer a constant here: it's the displacement of
// the active `BoatDesign` (kg), passed to `new_with_design` and set on the
// collider via `ColliderBuilder::mass` — Rapier still derives the angular
// inertia and centre of mass from the hull shape (uniform distribution),
// only the total is designer-set. Making the mass DISTRIBUTION (COM,
// radius of gyration) adjustable too is agreed follow-up work. The boat
// remains the one modeled ship type — a small cruising sailboat under
// engine (sails furled, no sail force modeled; wind is purely an external
// load on the hull/rig): a second ship TYPE would bring its own hull
// geometry and windage constants, where a `BoatDesign` only varies the
// keel curve and displacement on the shared hull.

// Air / water densities (kg/m³) for the quadratic load formulas.
const RHO_AIR: f32 = 1.2;
const RHO_WATER: f32 = 1025.0;

// Projected areas (m²): windage lateral / frontal. The underwater LATERAL
// area isn't a flat constant any more — it, its lever arm, and the yaw
// damping coefficient are all derived together from a `KeelProfile` (see
// keel.rs), since they're moments of the same underlying area distribution
// along the hull. The underwater AXIAL resistance isn't a flat frontal-area
// constant either any more — see the ITTC-1957 block below.
const WIND_AREA_LAT: f32 = 18.0; // hull side + superstructure above water
const WIND_AREA_FRONT: f32 = 7.0;

// Drag coefficients. The underwater lateral (sway/yaw) Cd is no longer a
// single flat constant here — see `KeelProfile::derive`'s `drag_*` fields
// (`CD_ROUND_HULL`/`CD_KEEL_PLATE`/`HULL_BASELINE_DRAFT` in keel.rs), which
// weight it per station by round-hull vs flat-plate-keel material.
const CD_AIR_LAT: f32 = 1.0;
// Axial windage isn't symmetric fore/aft the way the water-drag terms are:
// the bow is a fine entry with a sprayhood shaped to deflect airflow when
// moving into it (low drag), while the stern is wide and presents the
// sprayhood's open, concave side to a following wind — which doesn't just
// fail to deflect, it scoops the airflow like a cupped sail. Selected by
// the sign of the relative wind's axial component in `tick`.
const CD_AIR_BOW: f32 = 0.45;
const CD_AIR_STERN: f32 = 0.95;

/// Yaw damping coefficient (N·m per (rad/s)²) for a keel's Cd-weighted
/// cubic moment (see `KeelDerived::drag_cubic_moment` — the round-hull/
/// flat-plate-keel Cd split is already folded in per station, so this is
/// just the density factor). Exposed so the keel editor's live readout
/// stays derived from the same `RHO_WATER` source of truth as `tick` uses,
/// instead of keeping its own copy of the formula that could silently go
/// stale if it changed here.
pub fn yaw_damping_coefficient(drag_cubic_moment: f32) -> f32 {
    0.5 * RHO_WATER * drag_cubic_moment
}

// ---------------------------------------------------------------------------
// Axial (surge) hull resistance: ITTC-1957 skin friction
// ---------------------------------------------------------------------------
//
// The old model applied a bluff-body drag formula (frontal area × a flat
// Cd) underwater — the same functional form correctly used for windage on
// the topsides above, but wrong here: this hull's Froude number even at 3
// kn is ~0.14, well below the ~0.35-0.45 where wave-making resistance (a
// real bluff-body-like effect) matters. Below that, real hull resistance is
// overwhelmingly skin friction over the WETTED SURFACE, not the frontal
// area, and the coefficient is a friction coefficient (~0.003), not a bluff
// body's (~0.15-0.5) — using the wrong mechanism made the boat decelerate
// roughly 6x too fast coasting from cruising speed (measured: 3 kn -> 1 kn
// in ~17 m; real boats this size are still above 1 kn past 100 m).
//
// Kinematic viscosity of seawater (m²/s, ~15°C) — the standard value paired
// with the ITTC-1957 line.
const NU_WATER: f32 = 1.19e-6;
/// Hull form factor `(1+k)`: the ITTC-1957 line is calibrated to a flat
/// plate, so a real 3D hull's viscous PRESSURE resistance (beyond pure
/// friction) needs this correction on top. ~1.1-1.3 is the typical range
/// for a fine displacement sailing hull in naval-architecture practice
/// (Holtrop-Mennen-style form-factor estimates land here for slender
/// hulls); this boat, a fairly slender ~39 ft cruiser, sits toward the lean
/// end. The one number in this whole model that isn't either read from the
/// sim's own geometry or a fixed physical formula — everything else below
/// derives from `HULL_PTS`/`KeelProfile` (real modeled geometry) and
/// `NU_WATER`/the ITTC formula (fixed physics).
const HULL_FORM_FACTOR: f32 = 1.2;

/// Below this Reynolds number the ITTC-1957 line is clamped flat instead of
/// evaluated directly (see `ittc57_cf`) — the formula has a genuine
/// mathematical POLE at `Re = 100` (`log10(Re) = 2` zeroes the
/// denominator), not just the `Re = 0` case its shape suggests is the only
/// edge case. Real hulls never operate anywhere near there — the line is
/// meant for ship/model-scale turbulent flow, conventionally Re > ~10^6 —
/// so `Re = 10^5` is already a generous margin below any speed this sim
/// cares about (3 kn on this hull is Re ~ 1.5e7) while sitting safely away
/// from the singularity. Found live: a boat resting almost exactly still
/// against the quay (wind-pinned, near-zero but nonzero surge from
/// floating-point noise) drifted its Reynolds number across exactly 100 on
/// its way through zero and got launched sideways by a momentary
/// near-infinite friction force.
const ITTC_RE_FLOOR: f32 = 1.0e5;

/// ITTC-1957 model-ship correlation line: the standard formula for a hull's
/// skin-friction coefficient from its Reynolds number, clamped below
/// `ITTC_RE_FLOOR` to avoid the real pole at `Re = 100` (see its doc
/// comment) — the clamp only ever engages at speeds low enough that the
/// resulting force is negligible anyway (it's still multiplied by
/// `surge * |surge|` at the call site, which is what actually drives it to
/// zero as the boat comes to rest).
fn ittc57_cf(re: f32) -> f32 {
    let re = re.max(ITTC_RE_FLOOR);
    0.075 / (re.log10() - 2.0).powi(2)
}

/// The boat's centre of mass along the hull (m, boat-local x). Rapier
/// spreads the design's displacement uniformly over the hull collider
/// (see `new_with_design`), so the COM is the `HULL_PTS` polygon's area
/// centroid — a property of the hull outline alone, NOT of the keel
/// profile, which is why it does not coincide with the keel's centre of
/// lateral resistance (and shouldn't: the CG↔CLR gap is what turns sway
/// force into yaw moment). Exposed `pub` so the keel editor can draw the
/// actual pivot next to the CLR marker; unit-tested against Rapier's own
/// derived `local_center_of_mass` so this shoelace formula can't silently
/// drift from what the physics really uses. When the Roadmap's
/// adjustable-mass-distribution work lands, this stops being a constant
/// of the hull and must come from the `BoatDesign` instead.
pub fn hull_com_x() -> f32 {
    let mut a2 = 0.0f32; // twice the signed area
    let mut cx6 = 0.0f32; // 6 × area × centroid_x
    let n = HULL_PTS.len();
    for i in 0..n {
        let (x0, y0) = HULL_PTS[i];
        let (x1, y1) = HULL_PTS[(i + 1) % n];
        let cross = x0 * y1 - x1 * y0;
        a2 += cross;
        cx6 += (x0 + x1) * cross;
    }
    cx6 / (3.0 * a2)
}

/// The waterline extent [aft, fwd] (m) of a keel profile: the range over
/// which the curve is nonzero — the profile IS the underwater body, so
/// where it is zero the boat is out of the water (overhangs). Since
/// 2026-08-04 the preset curves carry their boats' real overhangs (zero
/// tails at both ends; the full keeler's sternpost is a vertical CLIFF to
/// zero, so its waterline legitimately ends at full draught — the support
/// convention handles that without a separate per-design constant).
/// Zero-crossings of the piecewise-linear curve are interpolated exactly.
/// Falls back to the full hull extent for a degenerate all-zero profile
/// (keeps Reynolds/Froude finite while the editor paints from scratch).
/// `pub` so the keel editor's live readout can show the painted curve's
/// waterline length from the same code the physics uses.
pub fn waterline_extent(profile: &KeelProfile) -> (f32, f32) {
    let (hull_aft, hull_fwd) = HULL_PTS
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &(x, _)| (lo.min(x), hi.max(x)));
    let mut aft = f32::MAX;
    let mut fwd = f32::MIN;
    for w in profile.points.windows(2) {
        let (x0, a0) = (w[0].x, w[0].y);
        let (x1, a1) = (w[1].x, w[1].y);
        if a0 <= 0.0 && a1 <= 0.0 {
            continue;
        }
        // Segment carries area: its wet sub-range is bounded by any
        // zero-crossing inside it. The crossing formula is only evaluated
        // when exactly one endpoint is non-positive, so a0 != a1 there.
        let cross = || x0 + (x1 - x0) * (-a0) / (a1 - a0);
        let lo = if a0 <= 0.0 { cross() } else { x0 };
        let hi = if a1 <= 0.0 { cross() } else { x1 };
        aft = aft.min(lo);
        fwd = fwd.max(hi);
    }
    if aft >= fwd {
        (hull_aft, hull_fwd)
    } else {
        (aft.max(hull_aft), fwd.min(hull_fwd))
    }
}

/// Hull length (m), read from `HULL_PTS`' own extent — the boat's LOA
/// (deck/collision length). The HYDRODYNAMIC length is the per-design
/// waterline length (`waterline_extent`), which is what all the physics
/// uses since 2026-08-04; this LOA measure remains only as the tests'
/// "boat length" yardstick (turn distances quoted in boat lengths mean
/// lengths of the boat you can see, not of its waterline).
#[cfg(test)]
fn hull_length() -> f32 {
    let (lo, hi) = HULL_PTS
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &(x, _)| (lo.min(x), hi.max(x)));
    hi - lo
}

/// Local half-beam (m) at hull station `x`, interpolated from `HULL_PTS`'
/// own upper (y >= 0) half — bow tip to stern point, the first 5 of its 8
/// points (see the array's own layout: bow, CCW around the port side to
/// the stern point, then back up the starboard side). Reads the hull's
/// real beam curve instead of assuming a flat average.
fn hull_half_beam(x: f32) -> f32 {
    let upper = &HULL_PTS[..5];
    let (bow_x, bow_b) = upper[0];
    let (stern_x, stern_b) = upper[upper.len() - 1];
    if x >= bow_x {
        return bow_b;
    }
    if x <= stern_x {
        return stern_b;
    }
    for w in upper.windows(2) {
        let (x0, b0) = w[0];
        let (x1, b1) = w[1];
        if x <= x0 && x >= x1 {
            let t = (x0 - x) / (x0 - x1);
            return b0 + (b1 - b0) * t;
        }
    }
    0.0
}

/// Wetted surface area (m²) below the waterline, integrated from the
/// ACTUAL modeled geometry instead of an assumed whole-boat average:
/// `HULL_PTS`' beam at each station and the keel profile's draught at each
/// station (see `keel.rs` — profile values are real depth, not a curve
/// shaped for feel). Per-station girth uses a semi-ellipse approximation
/// (`π/2·(half-beam + draught)`), the standard quick-hydrostatics method
/// for a rounded hull section — the sim has no true 3D hull lines to
/// integrate exactly, this is the best a 2D top-down outline + a
/// depth-per-length profile can do. The rudder's own wetted area (both
/// faces) is added separately since it's a movable appendage the profile
/// deliberately excludes (see `keel.rs`'s module doc comment).
fn wetted_surface_area(profile: &KeelProfile, rudder: &RudderDesign) -> f32 {
    const SUBSTEPS: usize = 64;
    use std::f32::consts::FRAC_PI_2;
    // Integrate over the WATERLINE extent only (2026-08-04): outside it
    // the boat is overhang — hull in the air, not wetted. Before the
    // profiles carried real overhangs this ran over the full hull extent,
    // silently counting phantom wetted area over the dry ends.
    let (x0, x1) = waterline_extent(profile);
    let girth = |x: f32| FRAC_PI_2 * (hull_half_beam(x) + profile.sample(x));
    let dx = (x1 - x0) / SUBSTEPS as f32;
    let mut wsa = 0.0f32;
    for i in 0..SUBSTEPS {
        let xa = x0 + i as f32 * dx;
        let xb = xa + dx;
        wsa += 0.5 * (girth(xa) + girth(xb)) * dx;
    }
    wsa + 2.0 * rudder.area()
}

// ---------------------------------------------------------------------------
// Attached-flow (lifting) lateral hydrodynamics: strip momentum exchange
// ---------------------------------------------------------------------------
//
// The sway/yaw model used to be cross-flow drag ONLY — quadratic in the
// local sideways speed, zero at zero drift. That is the SEPARATED-flow half
// of the standard ship-maneuvering decomposition, and on its own it made
// the boat turn "like a containership" going ahead (measured: 12° of
// heading in the first 10 m of a full-rudder coast turn at 2.5 kn, where
// the same hull backing managed 33° — backing was right because the
// rudder's own wind-up instability carries it, and going ahead there was
// nothing to overpower the rudder washing out its own command).
//
// What was missing is the ATTACHED-flow half: a hull moving FORWARD
// through water exchanges lateral momentum with it like a low-aspect-ratio
// wing, not just like a dragging plate. Both halves are the same single
// strip model evaluated per station x:
//
//   each strip sees the local lateral relative flow  V(x) = v + w·χ
//   and carries section added mass                   m_a(x) = ρ·π/2·a(x)²
//
// (χ = x − x_com; a(x) = the keel profile's local draught; m_a is the
// classical 2D flat-plate added mass under the same free-surface mirror
// used everywhere else in this file). The separated half charges each
// strip quadratic drag on V — that's `drag_area`/`drag_cubic_moment`/
// `drag_swept_moment` in keel.rs. The attached half is the momentum a
// fluid plane gains as the hull slides through it lengthwise at u:
//
//   dY/dx = u · d/dx [ m_a(x) · V(x) ]
//
// integrated ONLY over the expansion side — from the leading end (bow
// when making way ahead, stern when making sternway) to the station where
// m_a peaks. Past the peak the ideal theory would have the fluid politely
// hand the momentum back; in reality it separates at the keel's sharp
// trailing edge and the momentum leaves in the wake (the Kutta condition —
// the ONE structural judgement in this model, everything else is derived).
// That single integral, evaluated with the boundary values, yields in one
// shot the three classical results this sim previously lacked:
//
//   - Jones' slender-wing lift, EXACTLY: pure drift gives net side force
//     Y = −u·v·m_a(peak) — the textbook low-AR keel lift, with the correct
//     slope, letting the boat generate centripetal force from a few
//     degrees of leeway ("carving") instead of a 25° quadratic-drag skid;
//   - the Munk moment: the same integral's moment is destabilizing
//     (∝ −u·v), the bow-into-the-turn eagerness every displacement hull
//     has under way;
//   - the yaw-rate coupling (the ideal part of the classic Yr/Nr
//     derivatives) from the w·χ part of V(x).
//
// Fragility guards, by construction rather than by cap: the whole term
// scales as u·V — that's U²·sinβ·cosβ across drift angle β, which
// SELF-SATURATES at β = 45° and vanishes at β = 90° and at rest, exactly
// the regimes where cross-flow drag (already modeled) is the correct
// physics. Directional stability ahead is provided by the rudder foil
// standing in the flow aft (its restoring moment outweighs the Munk
// destabilization at all modeled speeds) — which is the real mechanism on
// the real boat, not a tuned counterweight.

/// Precomputed attached-flow integrals for making way AHEAD, about the
/// boat's centre of mass (χ = x − `hull_com_x()`). See the block comment
/// above.
///
/// **Ahead only, deliberately** (2026-08-04, found empirically and then
/// explained): the model assumes clean potential flow DEVELOPING from the
/// leading end — textbook-valid for the fine bow entry (it's literally
/// the slender-wing derivation), but making sternway the "leading end"
/// carries the deflected rudder blade, the turning propeller and its
/// aperture, so the flow arriving at the aft body and fin is disturbed
/// from the first metre; astern maneuvering derivatives are measured, not
/// derived from slender-body theory, in the literature for the same
/// reason. Verified against behaviour, not just argued: with an astern
/// branch enabled, its u-proportional yaw damping strangled the backing
/// turn (90° in 18.6 m of travel at 2.5 kn without it — matching the
/// real-boat 8–16 m mooring benchmark — degrading to 57° after 35.8 m
/// with it). Backing therefore stays on the separated (cross-flow drag)
/// half alone, which is also what carries a real boat's backing agility:
/// the rudder's own wind-up instability at the leading end.
#[derive(Clone, Copy, Debug)]
struct AttachedFlow {
    /// χ of the Kutta cut: the AFTMOST m_a peak station — the trailing
    /// side of the peak as seen by the bow-first oncoming flow, so a
    /// flat-topped fin cuts at its trailing edge rather than mid-fin.
    chi_cut: f32,
    /// Section added mass at the cut (kg/m).
    ma_cut: f32,
    /// `∫ m_a dχ` over the attached region (kg).
    j0: f32,
    /// `∫ m_a·χ dχ` over the attached region (kg·m).
    j1: f32,
}

/// Integrate the attached-flow coefficients from the keel profile (ahead
/// travel: attached region = bow down to the aftmost m_a peak).
fn attached_flow_coeffs(profile: &KeelProfile, x_com: f32) -> AttachedFlow {
    use std::f32::consts::FRAC_PI_2;
    const SAMPLES: usize = 256;
    let (x_stern, x_bow) = HULL_PTS
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &(x, _)| (lo.min(x), hi.max(x)));
    let ma = |x: f32| {
        let a = profile.sample(x);
        RHO_WATER * FRAC_PI_2 * a * a
    };
    // Find the m_a peak station: the AFTMOST sample attaining the
    // maximum, so a flat-topped fin cuts at the trailing edge of the flat
    // rather than its middle.
    let step = (x_bow - x_stern) / SAMPLES as f32;
    let mut a_max = 0.0f32;
    for i in 0..=SAMPLES {
        a_max = a_max.max(profile.sample(x_stern + i as f32 * step));
    }
    let mut x_cut = x_bow;
    for i in 0..=SAMPLES {
        // Scan bow→stern, keeping the LAST (aftmost) peak sample.
        let x = x_bow - i as f32 * step;
        if profile.sample(x) >= a_max - 1e-4 {
            x_cut = x;
        }
    }
    // Attached region: bow (leading end) back to the cut.
    let (lo, hi) = (x_cut, x_bow);
    let n = SAMPLES.max(1);
    let dx = (hi - lo) / n as f32;
    let mut j0 = 0.0f32;
    let mut j1 = 0.0f32;
    if dx > 0.0 {
        for i in 0..n {
            let xa = lo + i as f32 * dx;
            let xb = xa + dx;
            let (ma_a, ma_b) = (ma(xa), ma(xb));
            j0 += 0.5 * (ma_a + ma_b) * dx;
            j1 += 0.5 * (ma_a * (xa - x_com) + ma_b * (xb - x_com)) * dx;
        }
    }
    AttachedFlow { chi_cut: x_cut - x_com, ma_cut: ma(x_cut), j0, j1 }
}

// ---------------------------------------------------------------------------
// Axial (surge) hull resistance: wave-making (approximate — see below)
// ---------------------------------------------------------------------------
//
// Fixing the ITTC friction model above correctly exposed that the sim had
// NO wave-making resistance term at all: full-throttle equilibrium came out
// to ~9.4 kn, above this hull's classic displacement hull speed (~8.4 kn,
// Fn=0.4) — not achievable on 28 hp in reality. The old bluff-body
// coefficient was accidentally capping top speed at a plausible number
// while ALSO being wrong at low speed; fixing one exposed the other.
//
// The RIGHT tool for this is hull-form-specific: the Delft Systematic Yacht
// Hull Series (DSYHS, Gerritsma/Onnink/Versluis 1981, refined since by
// Keuning et al.) — tank-tested residuary-resistance regressions against 22
// systematically varied yacht hull forms, the standard used by real yacht
// VPPs. NOT implemented here: it needs a coefficient table I could not
// verify from available sources (tried four; none surfaced the real
// numbers), and hull-form inputs this sim doesn't model yet (prismatic
// coefficient, LCB, midship coefficient, BWL/Tc) — reciting a plausible-
// looking but unverified table would be exactly the kind of invented
// number this whole rewrite has been trying to get away from.
//
// What IS implemented is the right FUNCTION CLASS, generically calibrated
// rather than hull-form-fitted:
//
//   Cw(Fn) = C_WAVE_SCALE · exp(-C_WAVE_K / Fn²)
//
// This is the classical thin-ship (Michell/Havelock) asymptotic result for
// wave resistance at low-to-moderate Froude number: an essential
// singularity that vanishes faster than any power of Fn as Fn -> 0, then
// rises steeply approaching the hull-speed hump — the theoretically correct
// SHAPE, independent of hull form (DSYHS would sharpen the AMPLITUDE for
// THIS hull, not change this underlying shape).
//
// The two constants are fit to two widely-cited, GENERIC (not hull-form-
// specific, not tuned to this boat's own target behaviour) anchor points:
//   - Rw/Δ ~ 0.001 (negligible) at Fn = 0.20 — well below where wave-making
//     is understood to matter for any displacement hull.
//   - Rw/Δ ~ 0.12 (dominant) at Fn = 0.42 (~hull speed) — the commonly
//     cited order-of-magnitude "hump" value for a moderate displacement
//     hull's residuary resistance near hull speed.
// Consistency check, not a target: solving these two constants and
// re-running the equilibrium lands full-throttle at 6.3 kn — close to the
// ~6.2 kn this sim's engine sizing (T_BOLLARD_AHEAD's comment) always
// assumed, without that number being an input anywhere in this derivation.
// The low-speed coasting distance (the benchmark the ITTC fix targeted) is
// unaffected: Cw is negligible at 1-3 kn (Fn ~0.05-0.15), as it should be.
//
// TODO(upgrade path): replace Cw(Fn) with the real DSYHS regression once
// its coefficient table can be sourced and verified, and derive its hull-
// form inputs from `HULL_PTS`/`KeelProfile` the same way
// `wetted_surface_area` derives from them now — would sharpen the
// AMPLITUDE for this specific hull without changing the function class.
//
// KNOWN SIMPLIFICATION — astern (2026-08-05): this amplitude is used for
// BOTH travel directions, which makes full-astern equilibrium too brisk
// (measured 5.3-5.8 kn per boat vs the 2-4 kn real boats manage). The
// exp(-k/Fn²) SHAPE is gravity physics and direction-independent, but the
// AMPLITUDE depends on the fineness of the LEADING waterplane ending:
// backing, a wide flat transom stern leads and piles up a far bigger
// leading wave than the fine bow does at the same Fn — so C_WAVE_SCALE
// should really be direction-dependent, and per stern type (a canoe-
// sterned double-ender like the Alajuela has near-symmetric endings and
// little astern penalty; a wide modern transom the most — part of why
// double-enders back sweetly). NOT implemented because the factor can't
// be derived from modeled geometry (`HULL_PTS` is pointed at both ends —
// the real boats' transom shape simply isn't in the model) and no
// verifiable blunt-leading-end residuary anchor was available to cite;
// inventing one is what this file doesn't do. Harmless in practice
// meanwhile: below ~3 kn (Fn ≲ 0.15) the wave term is negligible in
// EITHER direction, so every harbour manoeuvre and benchmark is
// untouched — only an unrealistic straight-line astern SPRINT shows it.
// The fix, when sourced: a stern-type flag on `BoatDesign` (same spirit
// as `root_endplated`) selecting an astern amplitude factor.
const C_WAVE_SCALE: f32 = 0.489;
const C_WAVE_K: f32 = 0.248;
/// Standard gravity (m/s²) — NOT the same thing as `Sim`'s Rapier `gravity`
/// (deliberately zero, this is a top-down sim with no vertical dynamics).
/// This is the real-world constant Froude number and hull-speed weight
/// (`mass · G_EARTH`) are defined against.
const G_EARTH: f32 = 9.81;

/// Wave-making resistance coefficient vs. Froude number — see the module
/// comment above for the derivation. `Fn <= 0` (dead stop) correctly gives
/// 0: no relative speed, no waves.
fn wave_resistance_coefficient(froude: f32) -> f32 {
    if froude <= 0.0 {
        return 0.0;
    }
    C_WAVE_SCALE * (-C_WAVE_K / (froude * froude)).exp()
}

// Sway and yaw keep a linear low-speed term (surge no longer has one — see
// below) because their quadratic terms are keel-profile-derived
// (`self.keel.drag_area`, `self.keel.drag_cubic_moment`), so a flat linear floor
// would silently fall out of proportion for any profile far from the one
// it was tuned against (an extreme fin keel would keep a full keel's
// low-speed damping; an extreme full keel would keep a fin keel's).
// Instead each is a crossover speed/rate scaled by the profile's own
// quadratic coefficient, so the crossover point stays put as the profile
// changes instead of the absolute force. Surge doesn't need this — not
// because Cf falls at low speed (it slowly RISES as Re drops, ~1/log²Re),
// but because the surge force's u² factor collapses far faster than that
// growth — see the comment on `tick`'s surge drag term.
const SWAY_LIN_CROSSOVER_SPEED: f32 = 0.22; // m/s
const YAW_LIN_CROSSOVER_RATE: f32 = 0.14; // rad/s

fn k_lin_sway(drag_area: f32) -> f32 {
    0.5 * RHO_WATER * drag_area * SWAY_LIN_CROSSOVER_SPEED
}

fn k_lin_yaw(c_yaw_q: f32) -> f32 {
    c_yaw_q * YAW_LIN_CROSSOVER_RATE
}

/// Where the lateral WIND force acts, forward of the centre (m). Slightly
/// forward — high bow / foredeck windage — so the bow blows off downwind,
/// the familiar behaviour of a boat lying still in a breeze.
const WIND_CENTER_OFFSET: f32 = 0.9;

// ---------------------------------------------------------------------------
// Engine & propeller
// ---------------------------------------------------------------------------

// A ~28 hp auxiliary diesel (≈21 kW shaft) with a fixed 3-blade prop, at the
// ~0.2 kN-per-kW bollard-pull rule of thumb. Equilibrium against the ITTC
// friction + wave-making surge drag above, at the default profile: full
// ahead ≈ 3.25 m/s (6.3 kn), half throttle ≈ 2.24 m/s (4.4 kn) — a boat
// that motors below hull speed (~8.4 kn, Fn=0.4), as auxiliaries do. (Before
// the wave-making term existed, this equilibrium came out to 9.4 kn — above
// hull speed, not physically achievable on 28 hp; see the wave-resistance
// block comment above for that story.)
const T_BOLLARD_AHEAD: f32 = 4200.0; // N
// A prop pitched for ahead delivers much less astern.
const ASTERN_RATIO: f32 = 0.6;
/// Advance speed (m/s) at which a full-throttle prop stops delivering
/// thrust ("races"). Thrust falls off quadratically in the advance ratio
/// u/(|n|·U_PROP_RACE), so backing off the throttle lowers both the bollard
/// thrust AND the speed the falloff bites at, like a real fixed prop.
const U_PROP_RACE: f32 = 6.0;
/// Clearance (m) between the propeller and the rudder stock: the prop sits
/// this far AHEAD of the blade, whatever design is active (2026-08-04,
/// replacing the fixed `PROP_X = -5.6` that was placed relative to the old
/// shared rudder). Real geometry on every one of the reference boats has
/// the prop just forward of its rudder — shaft prop ahead of a spade,
/// aperture prop in the deadwood ahead of a transom-hung blade — and it's
/// load-bearing for the physics: the prop-wash steering term assumes the
/// blade stands in the race, which is only true if the prop leads it. So
/// the prop's station is DERIVED per design as `rudder.x +
/// PROP_AHEAD_OF_RUDDER` (thrust, prop walk and the wash all act there)
/// instead of being a constant that silently ends up abaft a
/// forward-mounted spade.
const PROP_AHEAD_OF_RUDDER: f32 = 0.5;
/// First-order engine spool time constant (s): the delivered thrust chases
/// the telegraph, it doesn't step. Sim state (`Sim::engine`), advanced only
/// inside `tick` — deterministic.
const THROTTLE_TAU: f32 = 0.4;
// Prop walk: a rotating prop's blades bite asymmetrically (deeper blade in
// denser/slower water, plus the helical wash against the hull), producing a
// sideways force at the stern proportional to thrust. For the usual
// right-handed prop the stern walks to STARBOARD ahead (weakly — the rudder
// wash mostly straightens it) and to PORT astern (strongly — nothing
// straightens it), the classic "backs to port". Fractions of |thrust|.
const PROP_WALK_AHEAD: f32 = 0.06;
const PROP_WALK_ASTERN: f32 = 0.13;

// ---------------------------------------------------------------------------
// Rudder
// ---------------------------------------------------------------------------

// The rudder blade is no longer a set of shared constants here
// (2026-08-04): position and dimensions live on the active `BoatDesign`
// (`RudderDesign` in boat.rs — each preset carries its real boat's blade,
// the O'Day's replacement-rudder listing being the one with published
// dimensions and the others derived from type + profile + the
// %-of-lateral-plane cross-check, see boat.rs). What the physics needs is
// derived once per `Sim` in `RudderFoil::from` below. Hard-over angle and
// the stall band stay shared: they're properties of the foil physics and
// typical steering gear, not of a particular boat.

/// Hard-over blade angle (degrees each way).
const RUDDER_MAX_DEG: f32 = 35.0;

/// The foil quantities `tick` needs every step, derived once per `Sim`
/// from the design's `RudderDesign` — area, effective aspect ratio
/// (mirror-doubled only when the root is end-plated, see boat.rs), and
/// the finite-plate post-stall drag ceiling at that same AR
/// (`flat_plate_cd`, keel.rs) — one AR feeding both the lift slope and
/// the plate drag, so they can't disagree about the blade's
/// three-dimensionality.
#[derive(Clone, Copy, Debug)]
struct RudderFoil {
    x: f32,
    area: f32,
    ar: f32,
    cd_max: f32,
}

impl RudderFoil {
    fn from(r: &RudderDesign) -> RudderFoil {
        let ar = r.aspect_ratio();
        RudderFoil { x: r.x, area: r.area(), ar, cd_max: flat_plate_cd(ar) }
    }
}
/// The lift curve is linear (attached flow) up to STALL_ON (~17°) and
/// follows the Hoerner flat-plate law beyond STALL_OFF (~25°), linearly
/// blended between so neither force has a step at the break (a step would
/// limit-cycle a helm held right at stall).
const RUDDER_STALL_ON: f32 = 0.30; // rad
const RUDDER_STALL_OFF: f32 = 0.44; // rad
/// Fraction of ahead thrust the deflected prop wash converts to side
/// force at the rudder. Thrust-deflection form (F = K·T·sin δ) rather
/// than a slipstream-velocity model: the added momentum flux in the wash
/// IS the thrust, so this is bounded by construction where the velocity
/// form needs an ad-hoc cap.
const K_WASH: f32 = 0.85;

/// Lift and drag coefficients of the rudder foil vs angle of attack
/// between its chord and the LOCAL EFFECTIVE water direction (rad) — the
/// caller measures α against the actual flow (surge, sway, *and* the
/// yaw-sweep at the blade's station), not just the helm angle, so the
/// same law naturally covers both a deflected blade steering a turn and a
/// centered blade resisting one (see the call site in `tick`).
///
/// Below stall: textbook thin-airfoil theory — lift slope
/// `2π·AR/(AR+2)` (the standard finite-span correction to the ideal 2π)
/// with induced drag `cl²/(π·AR)`. Unchanged from before; this regime was
/// never the problem.
///
/// Above stall, this used to fall back to a lift-only curve
/// (`0.9·sin 2α`) with induced-drag-only `cd` — which collapses toward
/// ZERO at α=90°, exactly the case that matters most (a centered blade
/// swept broadside by the hull's own spin). That's backwards: a stalled
/// foil is approximately a flat plate, and a flat plate's force is
/// LARGEST at 90°, not smallest. The flat-plate law gives the force
/// normal to the CHORD (not the flow) as `foil.cd_max·sin(mag)`, then
/// resolves it into lift/drag by the chord-to-flow angle — at mag=90°
/// that's zero lift, maximum drag: the barn-door case that brakes a spin,
/// falling out of the same geometry as the steering force instead of
/// needing a separate mechanism.
///
/// A foil overtaken by the flow (|α| > 90°: making sternway, or
/// crash-stopping through its own wake) is still a foil with the other
/// edge leading, so fold by ±π first and serve all four quadrants from
/// one curve — this single fold is what makes steering reverse correctly
/// when backing, with zero special cases.
fn rudder_lift_drag(alpha: f32, foil: &RudderFoil) -> (f32, f32) {
    use std::f32::consts::{FRAC_PI_2, PI};
    const CD0: f32 = 0.01;
    let mut a = alpha;
    if a > FRAC_PI_2 {
        a -= PI;
    } else if a < -FRAC_PI_2 {
        a += PI;
    }
    let mag = a.abs();
    let lin_slope = 2.0 * PI * foil.ar / (foil.ar + 2.0);
    let cl_lin = lin_slope * mag;
    let cd_lin = CD0 + cl_lin * cl_lin / (PI * foil.ar);
    let s = mag.sin();
    let cn = foil.cd_max * s * s;
    let cl_plate = cn * mag.cos();
    let cd_plate = cn * s + CD0;
    let (cl, cd) = if mag <= RUDDER_STALL_ON {
        (cl_lin, cd_lin)
    } else if mag < RUDDER_STALL_OFF {
        let t = (mag - RUDDER_STALL_ON) / (RUDDER_STALL_OFF - RUDDER_STALL_ON);
        (cl_lin * (1.0 - t) + cl_plate * t, cd_lin * (1.0 - t) + cd_plate * t)
    } else {
        (cl_plate, cd_plate)
    };
    (cl.copysign(a), cd)
}

/// The rudder foil's force response to a given inflow, in the SAME local
/// (fwd, side) frame the inflow itself is expressed in — a pure function of
/// physics, knowing nothing about the boat's world position or heading.
/// Composition on purpose: `tick` computes the actual flow the blade sees
/// (surge, sway, and the yaw sweep at the blade's station — the boat
/// physics' job), hands it here as a plain vector alongside the blade
/// angle, and this function returns the resulting force in that same local
/// frame; `tick` then rotates that local force into world space by `fwd`/
/// `side` to apply it at the blade's world position. Neither side needs to
/// know how the other is implemented.
fn rudder_force(flow: Vec2, delta: f32, foil: &RudderFoil) -> Vec2 {
    if flow.length_squared() <= 1e-6 {
        return Vec2::ZERO;
    }
    let fhat = flow / flow.length();
    let chord = Vec2::new(-delta.cos(), delta.sin()); // stock → trailing edge
    let alpha = chord.perp_dot(fhat).atan2(chord.dot(fhat));
    let (cl, cd) = rudder_lift_drag(alpha, foil);
    let q = 0.5 * RHO_WATER * foil.area * flow.length_squared();
    Vec2::new(-fhat.y, fhat.x) * (q * cl) + fhat * (q * cd)
}

// ---------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------

/// Wind + current state for one tick. Directions use compass convention
/// (0° = north = +y, 90° = east = +x). Wind is named by where it blows FROM
/// (mariners' convention); current by where it sets TOWARD.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Env {
    pub wind_from_deg: f32,
    pub wind_speed: f32, // m/s
    pub current_to_deg: f32,
    pub current_speed: f32, // m/s
}

impl Env {
    pub const CALM: Env = Env {
        wind_from_deg: 270.0,
        wind_speed: 0.0,
        current_to_deg: 90.0,
        current_speed: 0.0,
    };

    /// Unit vector for a compass direction (0° = +y, 90° = +x).
    pub fn compass_vec(deg: f32) -> Vec2 {
        let r = deg.to_radians();
        Vec2::new(r.sin(), r.cos())
    }

    /// Air velocity vector (the direction the air MOVES, i.e. opposite the
    /// FROM direction).
    pub fn wind_vel(&self) -> Vec2 {
        -Self::compass_vec(self.wind_from_deg) * self.wind_speed
    }

    /// Water velocity vector.
    pub fn current_vel(&self) -> Vec2 {
        Self::compass_vec(self.current_to_deg) * self.current_speed
    }
}

/// Helm + engine inputs for one tick. Together with `Env` this is the
/// complete input stream of the future recording format: same sequence of
/// both + fresh `Sim` => bit-identical trajectory.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct InputState {
    /// Engine telegraph, -1 (full astern) ..= 1 (full ahead).
    pub throttle: f32,
    /// Helm, -1 ..= 1. POSITIVE = the boat turns to STARBOARD (helm "to
    /// starboard"); the rudder blade itself deflects the other way.
    pub rudder: f32,
}

impl InputState {
    pub const NEUTRAL: InputState = InputState { throttle: 0.0, rudder: 0.0 };
}

// ---------------------------------------------------------------------------
// Sim
// ---------------------------------------------------------------------------

pub struct Sim {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    integration_params: IntegrationParameters,
    gravity: Vector<f32>,
    boat: RigidBodyHandle,
    /// Underwater lateral-area moments (area, CLR lever arm, yaw damping
    /// integral), derived once from a `KeelProfile` at construction.
    keel: KeelDerived,
    /// Wetted surface area (m²), integrated once from `HULL_PTS` + the
    /// keel profile at construction — see `wetted_surface_area`. Feeds the
    /// ITTC-1957 axial friction term in `tick`.
    wetted_surface: f32,
    /// The design's WATERLINE length (m), from the keel profile's nonzero
    /// support at construction (`waterline_extent`) — the hydrodynamic
    /// length that Reynolds number, Froude number and hull speed are
    /// defined against. Per-design since 2026-08-04: an HR 38's 9.5 m
    /// waterline and an O'Day's 10.2 m now motor like different boats
    /// (hull speed scales with √LWL), where previously every design ran
    /// on the shared 11.9 m outline length.
    hull_length: f32,
    /// Attached-flow (lifting) lateral coefficients — integrated once from
    /// the keel profile at construction, see `attached_flow_coeffs` and the
    /// block comment above it. Ahead only (see ibid.).
    att_flow: AttachedFlow,
    /// The active design's rudder blade, in the derived form `tick` needs
    /// (position, area, effective AR, post-stall ceiling) — see
    /// `RudderFoil` and `RudderDesign` in boat.rs.
    rudder: RudderFoil,
    /// Spooled engine response, -1..=1: the throttle input filtered through
    /// `THROTTLE_TAU`. Sim state (not input) — advanced only inside `tick`,
    /// reset for free by the fresh-`Sim`-per-run rule.
    engine: f32,
    /// Ticks advanced since spawn.
    pub ticks: u64,
}

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

impl Sim {
    /// A boat with the default design: the Hallberg-Rassy 38 preset (a
    /// moderate fin keel with a skeg-hung rudder — see `boat.rs`).
    pub fn new() -> Sim {
        Self::new_with_design(&BoatDesign::hallberg_rassy_38())
    }

    /// A boat with the default displacement but a custom keel profile —
    /// convenience for tests that probe the keel coupling in isolation.
    pub fn new_with_keel(profile: &KeelProfile) -> Sim {
        Self::new_with_design(&BoatDesign {
            keel: profile.clone(),
            ..BoatDesign::hallberg_rassy_38()
        })
    }

    /// A boat built from a full `BoatDesign`: underwater lateral-area
    /// distribution (=> centre of lateral resistance, yaw damping) AND
    /// displacement. Used by the R-reset key (via `respawn`) and initial
    /// construction; the keel editor's Apply uses `new_continuing` instead.
    pub fn new_with_design(design: &BoatDesign) -> Sim {
        Self::build(design, true)
    }

    /// A fresh `Sim` with a new design, but CONTINUING at the old sim's
    /// pose, velocity, and engine spool — used by the keel editor's Apply
    /// so the user sees the hydrodynamic effect of a keel change in place,
    /// instead of being teleported back to the berth. Still a fresh `Sim`
    /// (determinism rule: never mutate coefficients in place), just one
    /// whose initial conditions are transplanted from the predecessor.
    pub fn new_continuing(&self, design: &BoatDesign) -> Sim {
        let (pos, heading) = self.boat_pose();
        let (vel, angvel) = self.boat_vel();
        let engine = self.engine;
        let mut sim = Self::build(design, true);
        {
            let rb = &mut sim.bodies[sim.boat];
            rb.set_translation(vector![pos.x, pos.y], true);
            rb.set_rotation(nalgebra::UnitComplex::new(heading), true);
            rb.set_linvel(vector![vel.x, vel.y], true);
            rb.set_angvel(angvel, true);
        }
        sim.engine = engine;
        sim
    }

    /// Test-only: the same boat in unbounded open water — no shores, no
    /// jetties, no poles, no moored fleet. The shipped marina bounds (or
    /// obstructs) long benchmark runs — enough to hide whether a
    /// slow-turning boat would EVER complete a turn, and awkward for the
    /// 100 m+ coasting benchmark (which used to be verified by
    /// re-integrating the tick() formulas OFFLINE — a second copy of the
    /// surge math, free to drift from the real one; retired now that this
    /// arena lets the benchmarks run through the actual `tick`). Kept
    /// `#[cfg(test)]` so the shipped world — and its fixed
    /// collider-insertion order (determinism rule) — is untouched.
    #[cfg(test)]
    fn new_open_water(design: &BoatDesign) -> Sim {
        Self::build(design, false)
    }

    fn build(design: &BoatDesign, with_harbour: bool) -> Sim {
        let keel = design.keel.derive();
        let wetted_surface = wetted_surface_area(&design.keel, &design.rudder);
        let (wl_aft, wl_fwd) = waterline_extent(&design.keel);
        let hull_length = wl_fwd - wl_aft;
        let att_flow = attached_flow_coeffs(&design.keel, hull_com_x());
        let rudder = RudderFoil::from(&design.rudder);
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        // Static harbour geometry first, in a FIXED order (collider handle
        // numbering must be identical across runs for determinism — same
        // rule as Pegasus): the basin boundary, then jetties, then mooring
        // poles, then the moored boats. The whole block is skipped for the
        // test-only open-water arena (`with_harbour` = false).
        //
        // The boundary is ONE closed polyline: road shore head→sea
        // (running out along its diverging sea coast), the skerry line
        // across the open water (the segment joining the two coasts' far
        // ends — the world's edge, rendered as a chain of rocky islets),
        // hill shore back sea→head, and the rounded head arc closing the
        // loop. No wall crosses the entrance itself: the marina is open
        // to the sea.
        let hull: Vec<Point<f32>> = HULL_PTS.iter().map(|&(x, y)| point![x, y]).collect();
        if with_harbour {
            let mut boundary: Vec<Point<f32>> =
                road_shore().iter().map(|p| point![p.x, p.y]).collect();
            let mut hill: Vec<Point<f32>> =
                hill_shore().iter().map(|p| point![p.x, p.y]).collect();
            hill.reverse();
            boundary.extend(hill);
            // Head arc runs road[0] → hill[0]; the loop needs hill[0] →
            // road[0], so append its interior reversed and close on road[0].
            let arc = head_arc();
            for p in arc[1..arc.len() - 1].iter().rev() {
                boundary.push(point![p.x, p.y]);
            }
            boundary.push(boundary[0]);
            colliders.insert(
                ColliderBuilder::polyline(boundary, None)
                    // Rubbing-strake-on-rock feel: grippy, nearly dead.
                    .friction(0.5)
                    .restitution(0.1)
                    .build(),
            );

            // Pontoon jetties: solid rotated slabs from root to tip.
            for j in jetties() {
                let mid = j.root + j.dir * (j.len * 0.5);
                colliders.insert(
                    ColliderBuilder::cuboid(j.len * 0.5, JETTY_HALF_W)
                        .translation(vector![mid.x, mid.y])
                        .rotation(j.dir.y.atan2(j.dir.x))
                        .friction(0.5)
                        .restitution(0.1)
                        .build(),
                );
            }

            // Mooring poles: thin wooden piles a hull slides along rather
            // than grips (they're what the stern lines will belay to,
            // later).
            for pole in pole_positions() {
                colliders.insert(
                    ColliderBuilder::ball(POLE_RADIUS)
                        .translation(vector![pole.x, pole.y])
                        .friction(0.3)
                        .restitution(0.1)
                        .build(),
                );
            }

            // Moored boats: the same hull shape as the player's, parked as
            // static colliders — an occupied berth is a real obstacle.
            for mb in moored_boats() {
                colliders.insert(
                    ColliderBuilder::convex_hull(&hull)
                        .expect("hull points form a convex polygon")
                        .translation(vector![mb.pos.x, mb.pos.y])
                        .rotation(mb.heading)
                        .friction(0.4)
                        .restitution(0.05)
                        .build(),
                );
            }
        }

        // The boat: one dynamic body with the convex hull collider.
        let (start, start_heading) = start_pose();
        let body = RigidBodyBuilder::dynamic()
            .translation(vector![start.x, start.y])
            .rotation(start_heading)
            .ccd_enabled(true)
            .build();
        let boat = bodies.insert(body);
        colliders.insert_with_parent(
            ColliderBuilder::convex_hull(&hull)
                .expect("hull points form a convex polygon")
                // Total mass = the design's displacement; Rapier derives
                // the angular inertia and COM from the shape as if that
                // mass were spread uniformly over it (see the mass note
                // with the physical constants above).
                .mass(design.displacement_kg)
                .friction(0.4)
                .restitution(0.05)
                .build(),
            boat,
            &mut bodies,
        );

        Sim {
            bodies,
            colliders,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            integration_params: IntegrationParameters {
                dt: PHYSICS_DT,
                ..IntegrationParameters::default()
            },
            gravity: vector![0.0, 0.0],
            boat,
            keel,
            wetted_surface,
            hull_length,
            att_flow,
            rudder,
            engine: 0.0,
            ticks: 0,
        }
    }

    /// The underwater lateral-area moments this boat is currently using
    /// (area, CLR lever arm, yaw damping integral) — exposed read-only so
    /// the frontend can show what a profile actually produced.
    pub fn keel(&self) -> KeelDerived {
        self.keel
    }

    /// Boat pose: (position, heading). Heading is the Rapier rotation angle;
    /// 0 = bow east (+x), positive CCW.
    pub fn boat_pose(&self) -> (Vec2, f32) {
        let rb = &self.bodies[self.boat];
        let t = rb.translation();
        (Vec2::new(t.x, t.y), rb.rotation().angle())
    }

    /// Boat velocity (m/s) and yaw rate (rad/s).
    pub fn boat_vel(&self) -> (Vec2, f32) {
        let rb = &self.bodies[self.boat];
        let v = rb.linvel();
        (Vec2::new(v.x, v.y), rb.angvel())
    }

    /// Spooled engine response, -1..=1 (the throttle after `THROTTLE_TAU`
    /// lag) — read-only, for HUD readouts and cosmetic prop wash.
    pub fn engine(&self) -> f32 {
        self.engine
    }

    /// Test-only initial condition: give the boat a spin. Setting an
    /// initial state before the first tick is not the same as mutating
    /// physics mid-run (which stays forbidden — determinism rule).
    #[cfg(test)]
    fn set_yaw_rate(&mut self, w: f32) {
        self.bodies[self.boat].set_angvel(w, true);
    }

    /// Test-only initial condition: place the boat somewhere other than
    /// the guest-quay spawn before the first tick. Same rule as
    /// `set_yaw_rate` — needed because the spawn sits 2.4 m off the quay
    /// wall, which a turning-circle test would clip with its stern swing
    /// (found the hard way: the impulse of the port quarter kissing the
    /// quay reads exactly like a physics bug until you print the hull
    /// corner positions).
    #[cfg(test)]
    fn set_pose(&mut self, x: f32, y: f32, heading: f32) {
        let rb = &mut self.bodies[self.boat];
        rb.set_translation(vector![x, y], true);
        rb.set_rotation(Rotation::new(heading), true);
    }

    /// Test-only initial condition: send the boat along its own heading at
    /// `u` m/s (negative = making sternway). Same rule as `set_yaw_rate`.
    #[cfg(test)]
    fn set_forward_speed(&mut self, u: f32) {
        let rb = &mut self.bodies[self.boat];
        let rot = *rb.rotation();
        rb.set_linvel(vector![rot.re * u, rot.im * u], true);
    }

    /// Advance one fixed step under the given environment and helm/engine
    /// inputs. All forces are recomputed here from the boat state + `env` +
    /// `input` — nothing outside `tick` may touch the physics (the Pegasus
    /// determinism rule).
    pub fn tick(&mut self, env: &Env, input: &InputState) {
        // Clamp defensively: a replayed recording (or a buggy frontend)
        // must not be able to command super-physical inputs.
        let throttle = input.throttle.clamp(-1.0, 1.0);

        // Engine spool: delivered response chases the telegraph with a
        // first-order lag. Advanced here, before the force math, so the
        // thrust below sees this tick's value deterministically.
        self.engine += (throttle - self.engine) * (PHYSICS_DT / THROTTLE_TAU);

        let rb = &mut self.bodies[self.boat];
        rb.reset_forces(true);
        rb.reset_torques(true);

        let rot = *rb.rotation();
        let fwd = Vec2::new(rot.re, rot.im); // bow direction (local +x)
        let side = Vec2::new(-rot.im, rot.re); // port direction (local +y)
        let pos = Vec2::new(rb.translation().x, rb.translation().y);
        let v = Vec2::new(rb.linvel().x, rb.linvel().y);
        let w = rb.angvel();

        // --- Hydrodynamic drag: the hull moving RELATIVE TO THE WATER.
        // A uniform current is just "the water moves"; the same formula
        // both damps the boat and carries it along. Quadratic in relative
        // speed, split into surge (easy) and sway (hard) components.
        let vr = v - env.current_vel();
        let surge = vr.dot(fwd);
        let sway = vr.dot(side);
        // Axial (surge) resistance: ITTC-1957 skin friction over the actual
        // wetted surface, not a bluff-body Cd over frontal area — see the
        // block comment above `NU_WATER`. No fore/aft asymmetry: friction
        // depends on wetted area and speed, not on which end leads (unlike
        // the windage below, this hull doesn't have a flat transom to
        // separate flow off — HULL_PTS tapers to a point at both ends).
        // No added low-speed linear term either — but for the right
        // reason (maintainer review caught the first version of this
        // comment stating the mechanism BACKWARDS): Cf actually RISES
        // slowly as Re falls (~1/log²Re — 0.003 at Re 10⁷, 0.008 at the
        // 10⁵ floor). The FORCE still converges cleanly to zero at rest
        // because the u² factor below collapses far faster than Cf's
        // logarithmic growth, and below ITTC_RE_FLOOR Cf is capped
        // anyway. It's the product that vanishes, not the coefficient.
        let re = surge.abs() * self.hull_length / NU_WATER;
        let cf = ittc57_cf(re) * HULL_FORM_FACTOR;
        let f_friction = 0.5 * RHO_WATER * cf * self.wetted_surface * surge * surge.abs();
        // Wave-making resistance — see the block comment above
        // `wave_resistance_coefficient` for the derivation (approximate,
        // generically calibrated, not hull-form-fitted; negligible here at
        // 1-3 kn, Fn~0.05-0.15, so this doesn't disturb the friction fix's
        // own benchmark).
        let froude = surge.abs() / (G_EARTH * self.hull_length).sqrt();
        let f_wave = rb.mass() * G_EARTH * wave_resistance_coefficient(froude) * surge.signum();
        let f_surge = -fwd * (f_friction + f_wave);
        let f_sway = -side
            * (0.5 * RHO_WATER * self.keel.drag_area * sway * sway.abs()
                + k_lin_sway(self.keel.drag_area) * sway);
        // Surge drag acts through the centre; the lateral force acts at the
        // keel profile's DRAG-WEIGHTED centre of lateral resistance (see
        // keel.rs's `drag_clr_offset` — pulled toward whichever end carries
        // more flat-plate keel material, not just where the raw area sits)
        // — aft-of-centre for a typical skeg/rudder boat => weathervaning.
        rb.add_force(vector![f_surge.x, f_surge.y], true);
        let clr = pos + fwd * self.keel.drag_clr_offset;
        rb.add_force_at_point(vector![f_sway.x, f_sway.y], point![clr.x, clr.y], true);

        // Yaw drag: the same lateral-area profile, but its Cd-weighted
        // cubic moment — the water resists the hull sweeping around its
        // own axis more than it resists straight sway, because points far
        // from the pivot move faster during rotation and drag is quadratic
        // in speed.
        let c_yaw_q = yaw_damping_coefficient(self.keel.drag_cubic_moment);
        rb.add_torque(-(c_yaw_q * w * w.abs() + k_lin_yaw(c_yaw_q) * w), true);

        // Rotation-induced SIDE FORCE (the torque above's inseparable twin):
        // the strips resisting the spin don't pull symmetrically when the
        // (Cd-weighted) area is biased fore/aft. A strip at position x
        // sweeps sideways at w·x, so its drag is ∝ cd(x)·a(x)·(w·x)|w·x|;
        // summed along the hull the net sway force is
        // -0.5·ρ·w|w|·∫cd(x)·a(x)·x|x|dx (the profile's signed
        // drag_swept_moment — no separate Cd factor, already folded in per
        // station, see keel.rs). For an aft-biased keel spun clockwise the
        // stern out-drags the bow and shoves the boat to starboard, which
        // is what puts the effective centre of rotation aft of the centre
        // of mass. Applied through the centre (the couple component is
        // already the torque above); sway↔yaw cross terms are neglected,
        // consistent with the sway/yaw drag split.
        let f_spin = -side * (0.5 * RHO_WATER * w * w.abs() * self.keel.drag_swept_moment);
        rb.add_force(vector![f_spin.x, f_spin.y], true);

        // Attached-flow (lifting) lateral force + moment — the OTHER half
        // of the strip model the three drag terms above belong to; see the
        // block comment at `AttachedFlow`. Making way AHEAD only — see the
        // validity-domain note on that struct for why sternway stays on
        // the separated (drag) half alone. The leading (bow) boundary
        // enters with m_a = 0 (undisturbed water ahead of the boat
        // carries no body-imposed momentum), so only the Kutta-cut
        // boundary survives in the boundary term. Scales with u·V:
        // identically zero at rest, in pure sway, and in pure yaw — those
        // regimes belong to the separated (drag) half above.
        if surge > 0.0 {
            let p = &self.att_flow;
            let v_cut = sway + w * p.chi_cut;
            let y_att = -surge * p.ma_cut * v_cut;
            let n_att =
                -surge * p.chi_cut * p.ma_cut * v_cut - surge * (sway * p.j0 + w * p.j1);
            let f_att = side * y_att;
            rb.add_force(vector![f_att.x, f_att.y], true);
            rb.add_torque(n_att, true);
        }

        // --- Wind load: air moving relative to the hull/superstructure.
        let ar = env.wind_vel() - v;
        let a_ax = ar.dot(fwd);
        let a_lat = ar.dot(side);
        // a_ax > 0: relative wind moves toward the bow, i.e. it's blowing
        // FROM astern (a following wind) => the stern meets it first.
        let cd_air_ax = if a_ax > 0.0 { CD_AIR_STERN } else { CD_AIR_BOW };
        let f_wax = fwd * (0.5 * RHO_AIR * cd_air_ax * WIND_AREA_FRONT * a_ax * a_ax.abs());
        let f_wlat = side * (0.5 * RHO_AIR * CD_AIR_LAT * WIND_AREA_LAT * a_lat * a_lat.abs());
        rb.add_force(vector![f_wax.x, f_wax.y], true);
        // Lateral windage centre sits forward => the bow falls off downwind.
        let wc = pos + fwd * WIND_CENTER_OFFSET;
        rb.add_force_at_point(vector![f_wlat.x, f_wlat.y], point![wc.x, wc.y], true);

        // --- Propulsion: thrust and prop walk at the prop, from the
        // spooled engine response `n` (not the raw telegraph).
        let n = self.engine;
        let thrust = if n.abs() < 0.02 {
            0.0 // idle/neutral band (also guards the division below)
        } else {
            let t_max = if n >= 0.0 { T_BOLLARD_AHEAD } else { T_BOLLARD_AHEAD * ASTERN_RATIO };
            // Advance ratio proxy: how fast the water already moves through
            // the disc, relative to what this throttle's rpm can grip.
            // Positive = advancing with the thrust (unloads the prop),
            // negative = moving against it (crash stop — loads it up, but
            // bounded: the clamp caps the windmilling brake at -1× and the
            // crash-stop bite at 2× bollard).
            let adv = surge * n.signum() / (n.abs() * U_PROP_RACE);
            t_max * n * n.abs() * (1.0 - adv * adv.abs()).clamp(-1.0, 2.0)
        };
        let prop = pos + fwd * (self.rudder.x + PROP_AHEAD_OF_RUDDER);
        let f_thrust = fwd * thrust;
        rb.add_force_at_point(vector![f_thrust.x, f_thrust.y], point![prop.x, prop.y], true);
        // Prop walk (right-handed prop): at heading 0, `side` = port (+y).
        // Ahead the stern nudges starboard (-side at the stern => bow falls
        // slightly to port); astern the stern kicks port (+side) — "backs
        // to port". Applied at the prop, so it is both a side force and the
        // stern-swinging torque, exactly like the real effect.
        let walk = if n >= 0.0 { -PROP_WALK_AHEAD } else { PROP_WALK_ASTERN } * thrust.abs();
        let f_walk = side * walk;
        rb.add_force_at_point(vector![f_walk.x, f_walk.y], point![prop.x, prop.y], true);

        // --- Rudder: a foil in the local flow at the stern. δ is the BLADE
        // angle (positive = trailing edge to port => the boat turns to
        // port), opposite the helm sign convention on `InputState::rudder`.
        let delta = -input.rudder.clamp(-1.0, 1.0) * RUDDER_MAX_DEG.to_radians();
        // The inflow the blade actually sees: the hull's water-relative
        // surge/sway PLUS the yaw sweep w·x at the rudder's station. That
        // yaw term is the rudder half of the keel coupling — the keel's
        // damping moments set how fast yaw builds, and the built-up yaw in
        // turn feeds the rudder's angle of attack (a boat with a spinning
        // stern has its rudder self-damp the spin, which is why a fin
        // keeler still tracks at all). This is now the ONLY place the
        // rudder's physical footprint acts — the keel profile no longer
        // paints it (see `keel.rs`), so there's nothing left to
        // double-count, and the blade's resistance to a spin is exactly
        // as stale or as fresh as its actual angle to the actual flow.
        let flow = Vec2::new(-surge, -(sway + w * self.rudder.x));
        let rud_pt = pos + fwd * self.rudder.x;
        // rudder_force is a PURE function of the inflow and the blade angle,
        // in the same local (fwd, side) frame `flow` is already expressed
        // in — it knows nothing about world position/orientation. `tick`
        // owns computing that inflow (surge/sway/yaw-sweep, above) and
        // converting the returned local force into world space to apply it
        // at the right point, below.
        let f_local = rudder_force(flow, delta, &self.rudder);
        let f_rudder = fwd * f_local.x + side * f_local.y;
        rb.add_force_at_point(vector![f_rudder.x, f_rudder.y], point![rud_pt.x, rud_pt.y], true);
        // Prop wash over the blade: motoring ahead the prop's slipstream
        // hits the deflected rudder, which turns it sideways — the reaction
        // is K_WASH·T·sin δ of side force at the stern, there the instant
        // the throttle opens, boat speed zero or not. THE harbour
        // manoeuvre: a burst of ahead power kicks the bow around before
        // the boat gathers way. Astern (thrust < 0) the wash goes forward
        // under the hull and misses the blade entirely — no steerage
        // astern until sternway builds real flow, only prop walk. Both
        // behaviours fall out of the single max(T, 0).
        let f_wash = side * (-K_WASH * thrust.max(0.0) * delta.sin());
        rb.add_force_at_point(vector![f_wash.x, f_wash.y], point![rud_pt.x, rud_pt.y], true);

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
        self.ticks += 1;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn run(sim: &mut Sim, env: &Env, secs: f32) {
        run_input(sim, env, &InputState::NEUTRAL, secs);
    }

    fn run_input(sim: &mut Sim, env: &Env, input: &InputState, secs: f32) {
        for _ in 0..(secs / PHYSICS_DT) as u32 {
            sim.tick(env, input);
        }
    }

    const FULL_AHEAD: InputState = InputState { throttle: 1.0, rudder: 0.0 };
    const FULL_ASTERN: InputState = InputState { throttle: -1.0, rudder: 0.0 };

    // -----------------------------------------------------------------
    // Open-water performance benchmarks
    // -----------------------------------------------------------------
    //
    // These helpers are the source of the measured-performance table in
    // docs/reference-boats.md: `measure_open_water_benchmarks` (ignored)
    // regenerates the numbers, `open_water_benchmarks_stay_pinned` fails
    // if a physics change moves any of them by more than its tolerance.
    // All of them run the REAL `tick` in the wall-free arena
    // (`new_open_water`) — the shipped basin caps any run at ~30 m of
    // path, which is why the docs' 2026-08-04 in-basin turn table had "—"
    // cells and why coasting used to be verified by re-integrating the
    // formulas offline instead of through the actual Sim.

    /// One knot in m/s.
    const KN: f32 = 0.5144;

    fn presets() -> [(&'static str, BoatDesign); 4] {
        [
            ("Hallberg-Rassy 38", BoatDesign::hallberg_rassy_38()),
            ("O'Day 39", BoatDesign::oday_39()),
            ("Elan Impression 394", BoatDesign::elan_impression_394()),
            ("Alajuela 38", BoatDesign::alajuela_38()),
        ]
    }

    /// Steady full-throttle speed (kn), from rest in calm open water,
    /// COURSE HELD by a small deterministic P-D helmsman (helm from
    /// heading error + yaw rate). Hands-off, prop walk curls the run into
    /// a perpetual gentle circle and the drift drag caps speed ~0.8 kn
    /// low — real published boat speeds are straight-line, steered
    /// figures, so the benchmark steers too (the helm corrections at
    /// equilibrium are a fraction of a degree; their drag is real but
    /// negligible, and a real helmsman pays it as well). 90 s is ~25
    /// surge time constants (τ = m·v_eq/2T ≈ 3.3 s) — fully converged.
    fn measure_top_speed_kn(design: &BoatDesign) -> f32 {
        let mut sim = Sim::new_open_water(design);
        for _ in 0..(90.0 / PHYSICS_DT) as u32 {
            let (_, h) = sim.boat_pose();
            let (_, w) = sim.boat_vel();
            // Positive rudder = turn to starboard = heading (CCW+) falls,
            // so positive error/rate need positive helm to cancel.
            let rudder = (2.0 * h + 4.0 * w).clamp(-1.0, 1.0);
            sim.tick(&Env::CALM, &InputState { throttle: 1.0, rudder });
        }
        sim.boat_vel().0.length() / KN
    }

    /// Path length (m) coasting from 3 kn down through 1 kn, engine
    /// neutral, calm open water — the benchmark that motivated the
    /// ITTC-1957 rewrite, measured through the real `tick`.
    fn measure_coasting_3_to_1_kn_m(design: &BoatDesign) -> f32 {
        let mut sim = Sim::new_open_water(design);
        sim.set_forward_speed(3.0 * KN);
        let mut dist = 0.0f32;
        for _ in 0..(400.0 / PHYSICS_DT) as u32 {
            sim.tick(&Env::CALM, &InputState::NEUTRAL);
            let v = sim.boat_vel().0.length();
            dist += v * PHYSICS_DT;
            if v < KN {
                return dist;
            }
        }
        panic!("still above 1 kn after 400 s / {dist:.0} m of coasting");
    }

    /// The docs' turn benchmark: 2.5 kn ahead in calm open water, full
    /// starboard helm fed in over 2 s (slamming it stalls the blade — see
    /// docs/reference-boats.md), engine held at `throttle`. Returns
    /// (heading swung in degrees, path length in m, completed): the state
    /// at the moment 90° is reached, or at the 90 s cap — by which a
    /// rudder-only boat has coasted to a near-stop and the heading has
    /// plateaued, so the capped angle is an asymptote, not a race cutoff.
    fn measure_turn_90(design: &BoatDesign, throttle: f32) -> (f32, f32, bool) {
        use std::f32::consts::{FRAC_PI_2, PI};
        let mut sim = Sim::new_open_water(design);
        sim.set_forward_speed(2.5 * KN);
        let mut dist = 0.0f32;
        let mut dpsi = 0.0f32;
        let (_, mut last_h) = sim.boat_pose();
        for t in 0..(90.0 / PHYSICS_DT) as u32 {
            let ramp = (t as f32 * PHYSICS_DT / 2.0).min(1.0);
            sim.tick(&Env::CALM, &InputState { throttle, rudder: ramp });
            dist += sim.boat_vel().0.length() * PHYSICS_DT;
            let (_, h) = sim.boat_pose();
            let mut dh = h - last_h;
            while dh > PI {
                dh -= 2.0 * PI;
            }
            while dh < -PI {
                dh += 2.0 * PI;
            }
            dpsi += dh;
            last_h = h;
            if dpsi.abs() >= FRAC_PI_2 {
                return (dpsi.abs().to_degrees(), dist, true);
            }
        }
        (dpsi.abs().to_degrees(), dist, false)
    }

    /// Measurement harness, not a check: regenerates the numbers behind
    /// docs/reference-boats.md's "Measured performance (open water)"
    /// table and the pins in `open_water_benchmarks_stay_pinned`. Run:
    /// `cargo test -p harbour-sim-core --release -- --ignored --nocapture measure_open_water`
    #[test]
    #[ignore = "measurement harness for the docs table, not a check"]
    fn measure_open_water_benchmarks() {
        for (name, design) in presets() {
            let (wl_aft, wl_fwd) = waterline_extent(&design.keel);
            let lwl = wl_fwd - wl_aft;
            let hull_speed_kn = 1.34 * (lwl * 3.2808).sqrt();
            let top = measure_top_speed_kn(&design);
            let coast = measure_coasting_3_to_1_kn_m(&design);
            let (deg_r, dist_r, done_r) = measure_turn_90(&design, 0.0);
            let (deg_b, dist_b, done_b) = measure_turn_90(&design, 1.0);
            println!("{name}:");
            println!("  LWL {lwl:.2} m, hull speed {hull_speed_kn:.1} kn (1.34·√LWL_ft)");
            println!("  top speed {top:.2} kn ({:.0}% of hull speed)", 100.0 * top / hull_speed_kn);
            println!("  coasting 3->1 kn: {coast:.1} m");
            println!("  90° rudder only: {deg_r:.1}° in {dist_r:.1} m (completed: {done_r})");
            println!("  90° with burst:  {deg_b:.1}° in {dist_b:.1} m (completed: {done_b})");
        }
    }

    #[test]
    fn open_water_benchmarks_stay_pinned() {
        // The measured-performance table in docs/reference-boats.md used
        // to be a snapshot nothing re-checked: a physics change could
        // quietly shift a boat's whole character and the docs would
        // silently go stale. (Found live while writing this pin: the
        // previously quoted top speeds — 6.5/6.7/6.7/6.0 kn, from the
        // retired offline integration — can NOT be reproduced through the
        // real `tick`, whose wave term alone exceeds available thrust at
        // those speeds against each preset's real waterline. The pins
        // below are what the shipped formulas actually produce.)
        //
        // Values from `measure_open_water_benchmarks`, 2026-08-07.
        // Tolerances — ±2% speeds, ±5% distances, ±3° capped headings —
        // are wide enough for toolchain/dependency float drift, tight
        // enough that a real behaviour change fails loudly. When a change
        // moves a number ON PURPOSE: re-run the harness, update these
        // pins AND the docs table in the same commit.
        struct Pin {
            name: &'static str,
            design: BoatDesign,
            top_speed_kn: f32,
            coast_m: f32,
            /// (deg, path m, completed) — at the 90° mark when completed,
            /// at the protocol's 90 s cap when not.
            rudder_only: (f32, f32, bool),
            with_burst: (f32, f32, bool),
        }
        let pins = [
            Pin {
                name: "Hallberg-Rassy 38",
                design: BoatDesign::hallberg_rassy_38(),
                top_speed_kn: 5.64,
                coast_m: 111.3,
                rudder_only: (90.0, 23.7, true),
                with_burst: (90.0, 18.4, true),
            },
            Pin {
                name: "O'Day 39",
                design: BoatDesign::oday_39(),
                top_speed_kn: 5.85,
                coast_m: 109.6,
                rudder_only: (90.0, 18.7, true),
                with_burst: (90.0, 16.4, true),
            },
            Pin {
                name: "Elan Impression 394",
                design: BoatDesign::elan_impression_394(),
                top_speed_kn: 5.83,
                coast_m: 112.6,
                rudder_only: (90.0, 17.4, true),
                with_burst: (90.0, 15.8, true),
            },
            Pin {
                name: "Alajuela 38",
                design: BoatDesign::alajuela_38(),
                top_speed_kn: 5.46,
                coast_m: 129.1,
                // The one genuine asymptote: heading plateaus well short
                // of 90° as the boat coasts to a stop — a property of the
                // boat now, not of the old basin's walls.
                rudder_only: (43.8, 67.3, false),
                with_burst: (90.0, 24.9, true),
            },
        ];
        for pin in pins {
            let name = pin.name;
            let top = measure_top_speed_kn(&pin.design);
            assert!(
                (top - pin.top_speed_kn).abs() <= 0.02 * pin.top_speed_kn,
                "{name}: top speed {top:.2} kn, pinned {:.2} kn ±2%",
                pin.top_speed_kn
            );
            let coast = measure_coasting_3_to_1_kn_m(&pin.design);
            assert!(
                (coast - pin.coast_m).abs() <= 0.05 * pin.coast_m,
                "{name}: coasting 3->1 kn {coast:.1} m, pinned {:.1} m ±5%",
                pin.coast_m
            );
            // Real-world anchor, not just a pin: boats this size coast
            // past 100 m before dropping below 1 kn.
            assert!(coast > 100.0, "{name}: coasting {coast:.0} m — real boats pass 100 m");
            for (label, throttle, expect) in [
                ("rudder only", 0.0, pin.rudder_only),
                ("with burst", 1.0, pin.with_burst),
            ] {
                let (deg, dist, done) = measure_turn_90(&pin.design, throttle);
                let (e_deg, e_dist, e_done) = expect;
                assert_eq!(
                    done, e_done,
                    "{name} {label}: completed={done} ({deg:.1}° in {dist:.1} m), pinned \
                     completed={e_done}"
                );
                assert!(
                    (deg - e_deg).abs() <= 3.0,
                    "{name} {label}: swung {deg:.1}°, pinned {e_deg:.1}° ±3°"
                );
                assert!(
                    (dist - e_dist).abs() <= 0.05 * e_dist,
                    "{name} {label}: {dist:.1} m of path, pinned {e_dist:.1} m ±5%"
                );
            }
        }
    }

    #[test]
    fn attached_flow_reproduces_jones_slender_wing_lift() {
        // A draught profile growing linearly from the bow tip to its peak
        // is, to the strip-momentum model, a slender delta wing on its
        // side. Jones' classical slender-wing result: the whole lift is
        // set by the added mass of the widest section, Y = −u·v·m_a(peak),
        // independent of how the area got there. The model must reproduce
        // that EXACTLY (it's the same integral), which pins both the
        // magnitude and the sign convention of the tick() term.
        let peak = 1.5f32;
        let profile = KeelProfile {
            points: vec![Vec2::new(-2.0, peak), Vec2::new(6.0, 0.0)],
        };
        let att = attached_flow_coeffs(&profile, hull_com_x());
        let ma_peak = RHO_WATER * std::f32::consts::FRAC_PI_2 * peak * peak;
        assert!(
            (att.ma_cut - ma_peak).abs() < ma_peak * 0.02,
            "cut must sit at the peak: m_a {} vs {}",
            att.ma_cut,
            ma_peak
        );
        // Pure drift (w = 0): Y = s·u·m_a·v with s = −1 ahead.
        let (u, v) = (1.5f32, 0.2f32);
        let y = -u * att.ma_cut * v;
        let jones = -u * v * ma_peak;
        assert!(
            (y - jones).abs() < jones.abs() * 0.02,
            "lift {} should equal Jones' slender-wing value {}",
            y,
            jones
        );
    }

    #[test]
    fn attached_flow_moment_is_destabilizing_ahead() {
        // The Munk-moment half of the same integral: making way ahead with
        // a drift angle, the attached-flow yaw moment must push the bow
        // FURTHER into the drift (destabilizing — what makes a hull under
        // way eager to turn; overall directional stability comes from the
        // rudder foil standing in the flow aft, not from this term).
        // Drifting to port (v > 0) while moving ahead, the destabilizing
        // sense is yaw to starboard: N < 0.
        let att = attached_flow_coeffs(&BoatDesign::oday_39().keel, hull_com_x());
        let (u, v, w) = (1.5f32, 0.2f32, 0.0f32);
        let v_cut = v + w * att.chi_cut;
        let n = -u * att.chi_cut * att.ma_cut * v_cut - u * (v * att.j0 + w * att.j1);
        assert!(n < 0.0, "ahead + port drift must yaw the bow to starboard, got N = {n}");
    }

    #[test]
    fn a_forward_turn_carves_instead_of_ploughing() {
        // The behaviour the attached-flow model buys, measured the same
        // way the real-world complaint was: full starboard rudder at
        // 2.5 kn ahead, engine to neutral, and the boat should get through
        // 90° of heading within a few boat lengths (the real fin-keeler
        // benchmark is ~2; drag-only it never got there — 75° after 34 m
        // and still going when it ran out of basin).
        let mut sim = Sim::new_with_design(&BoatDesign::oday_39());
        // The fairway spawn (bow NE up the channel) has ~50 m of clear
        // water toward the hill shore — a starboard turn's ~20 m radius
        // arc sweeps into it without reaching anything.
        sim.set_forward_speed(1.29);
        let turn = InputState { throttle: 0.0, rudder: 1.0 };
        let mut dist = 0.0f32;
        let mut dpsi = 0.0f32;
        let (_, mut last_h) = sim.boat_pose();
        let mut turned = false;
        for _ in 0..(60.0 / PHYSICS_DT) as u64 {
            sim.tick(&Env::CALM, &turn);
            let (v, _) = sim.boat_vel();
            dist += v.length() * PHYSICS_DT;
            let (_, h) = sim.boat_pose();
            let mut dh = h - last_h;
            while dh > std::f32::consts::PI {
                dh -= 2.0 * std::f32::consts::PI;
            }
            while dh < -std::f32::consts::PI {
                dh += 2.0 * std::f32::consts::PI;
            }
            dpsi += dh;
            last_h = h;
            if dpsi.abs() >= std::f32::consts::FRAC_PI_2 {
                turned = true;
                break;
            }
        }
        assert!(
            turned && dist < 3.0 * hull_length(),
            "expected 90° within 3 boat lengths, got {:.0}° after {:.1} m",
            dpsi.to_degrees().abs(),
            dist
        );
    }

    #[test]
    fn waterline_extent_reads_each_presets_real_lwl() {
        // The profile's nonzero support IS the waterline (overhangs paint
        // zero), including the full keeler's sternpost cliff — its
        // waterline ends at full draught, which the support convention
        // handles without a per-design constant. Values are the boats'
        // published LWLs (docs/reference-boats.md).
        for (design, lwl, name) in [
            (BoatDesign::hallberg_rassy_38(), 9.50, "Hallberg-Rassy 38"),
            (BoatDesign::oday_39(), 10.21, "O'Day 39"),
            (BoatDesign::elan_impression_394(), 10.01, "Elan Impression 394"),
            (BoatDesign::alajuela_38(), 9.93, "Alajuela 38"),
        ] {
            let (aft, fwd) = waterline_extent(&design.keel);
            let got = fwd - aft;
            assert!(
                (got - lwl).abs() < 0.08,
                "{name}: waterline length {got:.2} m vs published LWL {lwl} m"
            );
        }
        // And the cliff case explicitly: the Alajuela's aft ending sits at
        // full draught (the deadwood cuts off vertically), not at a fade
        // to zero.
        let alajuela = BoatDesign::alajuela_38();
        let (aft, _) = waterline_extent(&alajuela.keel);
        assert!(
            alajuela.keel.sample(aft + 0.05) > 1.5,
            "the full keeler's waterline must end at full draught, got {} m just inside",
            alajuela.keel.sample(aft + 0.05)
        );
    }

    #[test]
    fn hull_com_x_matches_rapiers_derived_centre_of_mass() {
        // `hull_com_x` re-derives the COM with its own shoelace formula so
        // the keel editor can draw it without holding a `Sim` — this pins
        // it to the value Rapier actually pivots the physics around, so
        // the marker can't silently drift from the real thing (same
        // visuals-match-physics rule as HULL_PTS itself).
        let sim = Sim::new();
        let com = sim.bodies[sim.boat].local_center_of_mass();
        assert!(
            (com.x - hull_com_x()).abs() < 1e-3,
            "shoelace centroid {} vs Rapier's local COM {}",
            hull_com_x(),
            com.x
        );
        assert!(com.y.abs() < 1e-3, "a symmetric hull's COM must be on the centreline, got {}", com.y);
    }

    #[test]
    fn calm_water_boat_stays_put() {
        let mut sim = Sim::new();
        let (start, h0) = sim.boat_pose();
        run(&mut sim, &Env::CALM, 20.0);
        let (pos, heading) = sim.boat_pose();
        assert!(
            (pos - start).length() < 0.05,
            "boat drifted {} m in dead calm",
            (pos - start).length()
        );
        assert!((heading - h0).abs() < 0.01);
    }

    #[test]
    fn coasting_from_cruising_speed_covers_a_realistic_distance() {
        // The benchmark that motivated the ITTC-1957 friction rewrite: a
        // real small cruising sailboat losing way from ~3 kn with no
        // engine/wind/current is still above ~1 kn past 100 m — the old
        // bluff-body drag model covered that whole speed drop in ~17 m (a
        // ~6x-too-fast stop). This used to check a 20 s basin-safe slice
        // (the harbour walls cap a straight run at ~34 m) and lean on an
        // OFFLINE re-integration of the tick() formulas for the full
        // distance; the open-water arena now runs the whole benchmark
        // through the actual `tick` — no second copy of the surge math to
        // drift. Per-preset distances are pinned in
        // `open_water_benchmarks_stay_pinned`; this keeps the benchmark
        // itself legible on the default boat.
        let design = BoatDesign::hallberg_rassy_38();
        let dist = measure_coasting_3_to_1_kn_m(&design);
        assert!(
            dist > 100.0,
            "3 kn -> 1 kn in only {dist:.0} m — real boats this size coast past 100 m"
        );
    }

    #[test]
    fn same_input_sequence_is_bit_identical() {
        // Fresh sim + same input stream (env AND helm/engine) => bit-exact
        // trajectory. This is the property future replays/verification will
        // rely on; the engine spool state must not break it.
        let script = |t: u64| {
            if t < 600 {
                (
                    Env { wind_from_deg: 200.0, wind_speed: 9.0, ..Env::CALM },
                    InputState { throttle: 1.0, rudder: 0.0 },
                )
            } else if t < 1200 {
                (
                    Env { wind_from_deg: 200.0, wind_speed: 9.0, ..Env::CALM },
                    InputState { throttle: 0.5, rudder: 0.7 },
                )
            } else {
                (
                    Env {
                        wind_from_deg: 45.0,
                        wind_speed: 4.0,
                        current_to_deg: 90.0,
                        current_speed: 0.8,
                    },
                    InputState { throttle: -0.8, rudder: -0.3 },
                )
            }
        };
        let mut a = Sim::new();
        let mut b = Sim::new();
        for t in 0..2400 {
            let (env, input) = script(t);
            a.tick(&env, &input);
            b.tick(&env, &input);
        }
        let (pa, ha) = a.boat_pose();
        let (pb, hb) = b.boat_pose();
        assert_eq!(pa.x.to_bits(), pb.x.to_bits());
        assert_eq!(pa.y.to_bits(), pb.y.to_bits());
        assert_eq!(ha.to_bits(), hb.to_bits());
    }

    #[test]
    fn wind_pushes_the_boat_downwind() {
        // Northerly wind blows the boat south across the open fairway.
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        let env = Env { wind_from_deg: 0.0, wind_speed: 10.0, ..Env::CALM };
        run(&mut sim, &env, 30.0);
        let pos = sim.boat_pose().0;
        assert!(
            pos.y < start.y - 3.0,
            "expected a clear southward drift, got dy = {}",
            pos.y - start.y
        );
    }

    #[test]
    fn current_carries_the_boat_along() {
        // An easterly-setting current carries the boat east through the
        // fairway — but slowly picking up way from a dead stop, not
        // snapping to current speed. Same physics, same direction of
        // surprise, as the coasting fix: the ITTC friction FORCE is
        // genuinely weak at low RELATIVE speed (the u² factor, not Cf,
        // which slowly rises as Re falls), and the default 8.5 t hull has
        // a lot of inertia for a gentle 0.8 m/s (1.6 kn) current to work
        // against. 60 s only gets it to ~36% of current speed and ~10 m
        // of drift — real, not a bug (this replaces a 30 s/5 m threshold
        // that was calibrated to the old, too-strong bluff-body drag).
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        let env = Env { current_to_deg: 90.0, current_speed: 0.8, ..Env::CALM };
        run(&mut sim, &env, 60.0);
        let pos = sim.boat_pose().0;
        assert!(
            pos.x > start.x + 8.0,
            "expected a clear eastward drift, got dx = {}",
            pos.x - start.x
        );
    }

    #[test]
    fn a_lee_shore_stops_the_wind_drift() {
        // Wind square onto the road shore blows the boat from the fairway
        // down onto the marina: it must fetch up on something solid (a
        // pole row, a jetty, a moored boat, the shore itself) and come to
        // rest there — not ghost through the geometry or bounce back out.
        // The spawn's bow lies along the shore tangent and the road shore
        // is on its -out side, so wind FROM the +out bearing (bow bearing
        // + 90°) is dead onshore — derived from the pose, not hardcoded,
        // so it survives reorientations of the marina.
        let mut sim = Sim::new();
        let (start, h0) = sim.boat_pose();
        let onshore_from = (90.0 - h0.to_degrees() + 90.0).rem_euclid(360.0);
        let env = Env { wind_from_deg: onshore_from, wind_speed: 12.0, ..Env::CALM };
        run(&mut sim, &env, 90.0);
        let (pos, _) = sim.boat_pose();
        let drift = (pos - start).length();
        assert!(drift > 5.0, "expected a real shoreward drift, got {drift} m");
        assert!(
            drift < 72.0,
            "boat ended {drift} m from the start — through the marina and the shore?"
        );
        let (v, _) = sim.boat_vel();
        assert!(
            v.length() < 0.5,
            "boat still moving {} m/s against the lee obstruction",
            v.length()
        );
    }

    #[test]
    fn spinning_an_aft_biased_hull_shoves_it_to_starboard() {
        // Rotational drag over a fore/aft-asymmetric area distribution is
        // not a pure torque: spin the default (aft-biased) hull clockwise
        // and the stern (big area, sweeping to port) out-drags the bow
        // (small area, sweeping to starboard) — net side force to
        // starboard, which is what puts the effective centre of rotation
        // aft of the centre of mass. Measured along the boat's OWN
        // starboard axis (the spawn heading is whatever the marina's
        // orientation makes it, so world axes prove nothing). Checked
        // over a fraction of a second so the heading (and with it the
        // force direction) hasn't swung far from its initial orientation.
        let mut sim = Sim::new();
        let h0 = sim.boat_pose().1;
        let stbd = Vec2::new(h0.sin(), -h0.cos()); // fwd rotated -90°
        sim.set_yaw_rate(-1.0);
        for _ in 0..12 {
            sim.tick(&Env::CALM, &InputState::NEUTRAL);
        }
        let (v, _) = sim.boat_vel();
        let drift = v.dot(stbd);
        assert!(
            drift > 0.02,
            "expected a clear starboard drift from the spin, got {drift} m/s"
        );

        // Control: a fore-aft symmetric profile has no such coupling — the
        // same spin produces no appreciable sideways drift.
        let symmetric = KeelProfile {
            points: vec![Vec2::new(-6.0, 1.0), Vec2::new(6.0, 1.0)],
        };
        let mut sym = Sim::new_with_keel(&symmetric);
        sym.set_yaw_rate(-1.0);
        for _ in 0..12 {
            sym.tick(&Env::CALM, &InputState::NEUTRAL);
        }
        let (vs, _) = sym.boat_vel();
        // A symmetric KEEL profile has no `swept_moment` coupling of its
        // own — but the rudder is a separate, always-aft foil now (see
        // `keel.rs`'s module doc comment), independent of whatever profile
        // is loaded, and it still sees a large angle of attack from the
        // spin and still drags the stern toward starboard by itself. So
        // the honest control isn't "zero drift" any more, it's "less
        // drift than the aft-biased hull" — the keel's own asymmetry
        // stacks on top of the same baseline rudder contribution both
        // sims share.
        let sym_drift = vs.dot(stbd);
        assert!(
            sym_drift > 0.0 && sym_drift < drift,
            "a symmetric keel should still drift to starboard, but less than the \
             aft-biased default (rudder-only coupling, no keel swept_moment on top): \
             got {sym_drift} vs default {drift} m/s"
        );
    }

    #[test]
    fn a_mooring_pole_stops_the_boat() {
        // Motor in from the fairway straight along a pole row, aimed at
        // its outermost pile: the bow must fetch up on it a few metres in
        // — well short of a free run of the same duration — instead of
        // ghosting through a thin ball collider.
        let mut sim = Sim::new();
        let j = jetties()[4];
        let d_tip = *j.pole_stations().last().unwrap();
        let pole = j.root + j.dir * d_tip - j.side() * POLE_ROW_OFFSET;
        let start = pole + j.dir * 12.0; // out in the fairway, aimed inward
        let heading = (-j.dir).y.atan2((-j.dir).x);
        sim.set_pose(start.x, start.y, heading);
        run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 6.0);
        let travel = (sim.boat_pose().0 - start).length();
        assert!(
            travel < 9.5,
            "expected to fetch up on the pole ~5.9 m in, travelled {travel} m"
        );
    }

    #[test]
    fn an_occupied_berth_is_blocked_by_the_moored_boat() {
        // Aim from open water straight down the first moored boat's berth
        // axis (it lies on the outer jetty's SW side, so the approach is
        // clear): its parked hull is a static collider — it neither
        // yields nor moves, and the incoming boat fetches up on its end.
        let mut sim = Sim::new();
        let mb = moored_boats()[0];
        let start = mb.pos + mb.out * 18.0;
        let heading = (-mb.out).y.atan2((-mb.out).x);
        sim.set_pose(start.x, start.y, heading);
        run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 6.0);
        let travel = (sim.boat_pose().0 - start).length();
        assert!(
            travel < 9.5,
            "expected to fetch up on the moored boat ~6.3 m in, travelled {travel} m"
        );
    }

    #[test]
    fn a_following_wind_pushes_harder_than_a_headwind_of_the_same_speed() {
        // The sprayhood deflects a headwind (fine bow) but presents its
        // open, concave side to a following wind (which scoops into it,
        // same idea as a wide stern) — axial windage is NOT symmetric
        // fore/aft the way the water-drag terms are. Wind dead on the bow
        // (from the bow's own compass bearing, derived from the spawn
        // heading) is a pure headwind; from dead astern a pure following
        // wind — both purely axial, no lateral component, so this
        // isolates CD_AIR_BOW vs CD_AIR_STERN.
        let mut headwind = Sim::new();
        let mut following = Sim::new();
        let bow_bearing = 90.0 - headwind.boat_pose().1.to_degrees();
        let headwind_env = Env { wind_from_deg: bow_bearing, wind_speed: 10.0, ..Env::CALM };
        let following_env =
            Env { wind_from_deg: bow_bearing + 180.0, wind_speed: 10.0, ..Env::CALM };
        run(&mut headwind, &headwind_env, 2.0);
        run(&mut following, &following_env, 2.0);
        let head_speed = headwind.boat_vel().0.length();
        let following_speed = following.boat_vel().0.length();
        assert!(
            following_speed > head_speed * 1.5,
            "expected a following wind to push noticeably harder than a headwind: \
             following {following_speed} m/s vs headwind {head_speed} m/s"
        );
    }

    #[test]
    fn full_throttle_equilibrium_speed_is_bracketed() {
        // The thrust curve intersects the ITTC friction + wave-making surge
        // drag somewhere around 3.25 m/s (6.3 kn) at the default profile —
        // back near this hull's intended ~6.2 kn ahead speed (see
        // T_BOLLARD_AHEAD's comment) now that wave-making resistance caps
        // it below hull speed again, without that number being tuned to hit
        // this test. Too far from equilibrium for a short run to settle,
        // so bracket instead: released below the equilibrium the boat must
        // still be gaining, released above it it must be losing. The spawn
        // lies in the open fairway with a long clear run up the channel,
        // so a straight blast from it stays in open water.
        let below = {
            let mut sim = Sim::new();
            sim.set_forward_speed(2.5);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            let (v, _) = sim.boat_vel();
            v.length()
        };
        assert!(below > 2.5, "expected to accelerate from 2.5 m/s at full ahead, got {below}");
        let above = {
            let mut sim = Sim::new();
            sim.set_forward_speed(3.8);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            let (v, _) = sim.boat_vel();
            v.length()
        };
        assert!(above < 3.8, "expected to slow from 3.8 m/s at full ahead, got {above}");
    }

    #[test]
    fn wave_resistance_is_negligible_at_low_speed_and_dominant_near_hull_speed() {
        // Low end: at the Froude numbers the ITTC friction fix targeted
        // (~1-3 kn for this hull, Fn~0.05-0.15), wave-making must stay
        // negligible so it doesn't disturb that fix's own benchmark.
        let low = wave_resistance_coefficient(0.15);
        assert!(low < 0.001, "wave resistance should be negligible at Fn=0.15, got Cw={low}");
        // High end: approaching hull speed (Fn~0.4), wave-making should
        // have become a substantial fraction of displacement weight — the
        // "wall" that caps a normally-powered displacement hull there.
        let high = wave_resistance_coefficient(0.42);
        assert!(high > 0.05, "wave resistance should be substantial near hull speed, got Cw={high}");
        // Monotonically increasing across that whole range — no spurious
        // hump or dip from the exponential's own shape.
        let samples: Vec<f32> =
            (10..=45).map(|i| wave_resistance_coefficient(i as f32 / 100.0)).collect();
        assert!(
            samples.windows(2).all(|w| w[1] >= w[0]),
            "Cw(Fn) should be monotonically increasing: {samples:?}"
        );
    }

    #[test]
    fn astern_is_weaker_than_ahead() {
        // A prop pitched for ahead delivers less astern (ASTERN_RATIO) —
        // axial drag itself is symmetric now (ITTC skin friction depends on
        // wetted area and speed, not which end leads), so this test is
        // purely about the thrust asymmetry. The fairway spawn has clear
        // water both up and down the channel, so 8 s of way in either
        // direction stays in the open.
        let mut ahead = Sim::new();
        run_input(&mut ahead, &Env::CALM, &FULL_AHEAD, 8.0);
        let ahead_speed = ahead.boat_vel().0.length();
        let mut astern = Sim::new();
        run_input(&mut astern, &Env::CALM, &FULL_ASTERN, 8.0);
        let astern_speed = astern.boat_vel().0.length();
        assert!(astern_speed > 0.5, "full astern barely moved the boat: {astern_speed} m/s");
        assert!(
            astern_speed < ahead_speed * 0.75,
            "expected astern to be clearly weaker: astern {astern_speed} vs ahead {ahead_speed}"
        );
    }

    #[test]
    fn a_burst_astern_walks_the_stern_to_port() {
        // Right-handed prop: going astern the walk force pushes the stern
        // to port => a clockwise yaw (the heading DECREASES from wherever
        // it started): the bow swings to starboard, the classic "backs to
        // port". Measured as a delta from the spawn heading.
        let mut astern = Sim::new();
        let h0 = astern.boat_pose().1;
        run_input(&mut astern, &Env::CALM, &FULL_ASTERN, 6.0);
        let h_astern = astern.boat_pose().1 - h0;
        assert!(
            h_astern < -0.02,
            "expected the bow to swing starboard (negative heading delta) going astern, \
             got {h_astern}"
        );

        // Control: ahead the walk reverses sign and the wash keeps it weak
        // — a smaller swing the other way.
        let mut ahead = Sim::new();
        run_input(&mut ahead, &Env::CALM, &FULL_AHEAD, 6.0);
        let h_ahead = ahead.boat_pose().1 - h0;
        assert!(
            h_ahead > 0.0,
            "expected a slight port swing (positive heading delta) going ahead, got {h_ahead}"
        );
        assert!(
            h_astern.abs() > h_ahead.abs(),
            "prop walk should bite harder astern: astern {h_astern} vs ahead {h_ahead}"
        );
    }

    #[test]
    fn rudder_turns_the_boat_when_making_way() {
        // Helm to starboard (positive input) with way on => clockwise turn
        // (negative heading delta); mirrored to port. Short runs from an
        // injected speed, engine off, so this isolates the foil from wash
        // and walk.
        let turn = |rudder: f32| {
            let mut sim = Sim::new();
            let h0 = sim.boat_pose().1;
            sim.set_forward_speed(2.5);
            let input = InputState { throttle: 0.0, rudder };
            run_input(&mut sim, &Env::CALM, &input, 1.5);
            sim.boat_pose().1 - h0
        };
        let stbd = turn(1.0);
        let port = turn(-1.0);
        assert!(stbd < -0.03, "helm to starboard should turn clockwise, got heading {stbd}");
        assert!(port > 0.03, "helm to port should turn anticlockwise, got heading {port}");
    }

    #[test]
    fn prop_wash_steers_at_rest_ahead_but_not_astern() {
        // From a dead stop, a burst of AHEAD power steers immediately: the
        // prop wash hits the deflected blade before the boat has any way
        // on. The same burst ASTERN gives (almost) no rudder authority —
        // the wash misses the blade; only slowly-building sternway flow
        // acts. Differential across both helm directions, so prop walk
        // (identical for either helm) cancels and only rudder authority
        // remains.
        let turn = |throttle: f32, rudder: f32| {
            let mut sim = Sim::new();
            run_input(&mut sim, &Env::CALM, &InputState { throttle, rudder }, 2.0);
            sim.boat_pose().1
        };
        let ahead_authority = turn(1.0, 1.0) - turn(1.0, -1.0);
        let astern_authority = turn(-1.0, 1.0) - turn(-1.0, -1.0);
        assert!(
            ahead_authority < -0.05,
            "starboard helm + ahead burst should swing the bow starboard, got diff {ahead_authority}"
        );
        assert!(
            ahead_authority.abs() > 3.0 * astern_authority.abs(),
            "rudder authority should be far greater ahead than astern: \
             ahead {ahead_authority} vs astern {astern_authority}"
        );
    }

    #[test]
    fn hard_over_stalls() {
        // Foil curves are per-design now — probe them on the anchor blade
        // (the O'Day's, the one with published dimensions).
        let foil = RudderFoil::from(&BoatDesign::oday_39().rudder);
        let rudder_lift_drag = |a: f32| rudder_lift_drag(a, &foil);
        // The lift curve: linear below stall, flat-plate above, odd, and
        // folded by π so a backing foil reads the same curve. CL at
        // hard-over (35° => 0.611 rad) sits BELOW the pre-stall peak —
        // more helm is not always more turn.
        assert!(rudder_lift_drag(0.28).0 > rudder_lift_drag(0.611).0);
        assert!(
            (rudder_lift_drag(-0.28).0 + rudder_lift_drag(0.28).0).abs() < 1e-6,
            "lift curve must be odd"
        );
        assert!(
            (rudder_lift_drag(0.28 - std::f32::consts::PI).0 - rudder_lift_drag(0.28).0).abs()
                < 1e-5,
            "folding by pi must land on the same curve (backing foil)"
        );
        // The flat-plate law this now falls back to past stall is LARGEST
        // at 90°, not smallest — the barn-door case (a centered rudder
        // swept broadside by the hull's own spin) must brake it, not go
        // silent the way a lift-only curve would.
        let (cl_90, cd_90) = rudder_lift_drag(std::f32::consts::FRAC_PI_2);
        assert!(cl_90.abs() < 1e-3, "a blade square to the flow produces no lift, got {cl_90}");
        // "Near-maximum" is relative to THIS blade's finite-AR ceiling
        // (RUDDER_CD_MAX ≈ 1.20), not the 2D-plate 1.98 — asserting a
        // hard-coded 1.5 here would quietly re-assert the infinite-plate
        // assumption the ceiling was derived to avoid.
        assert!(
            cd_90 > foil.cd_max * 0.95,
            "a blade square to the flow should be near its max drag {}, got {cd_90}",
            foil.cd_max
        );

        // And the behaviour it buys: the INITIAL helm bite. Slammed
        // hard-over the blade starts stalled and bites more weakly than a
        // moderate helm still on the linear slope. (Only the first instants
        // show it: once yaw builds, the swinging stern rotates the inflow,
        // eases the effective angle of attack back toward the slope, and
        // the deeper geometric angle wins again — a soft-stalling low-AR
        // rudder hard-over still out-turns moderate helm at steady state,
        // it just gets there mushily and with far more induced drag.)
        let initial_rate = |rudder: f32| {
            let mut sim = Sim::new();
            sim.set_forward_speed(3.0);
            let input = InputState { throttle: 0.0, rudder };
            for _ in 0..12 {
                sim.tick(&Env::CALM, &input);
            }
            sim.boat_vel().1.abs()
        };
        let moderate = initial_rate(0.45);
        let hard_over = initial_rate(1.0);
        assert!(
            moderate > hard_over,
            "a stalled hard-over should bite more weakly at first: \
             moderate {moderate} vs hard-over {hard_over} rad/s"
        );
    }

    #[test]
    fn rudder_aligned_with_flow_has_no_effect() {
        // rudder_force takes the ACTUAL inflow (which a spin can dominate
        // even with the helm centered — see the module doc comment on
        // rudder_lift_drag) and the blade angle; whenever the chord ends up
        // parallel to that inflow, regardless of why, the blade should
        // produce no lift and only the baseline parasitic drag. Flow along
        // +x (pure "surge"), chord angle 0 (helm centered) is the simplest
        // such case.
        let foil = RudderFoil::from(&BoatDesign::oday_39().rudder);
        let f = rudder_force(Vec2::new(2.5, 0.0), 0.0, &foil);
        assert!(f.y.abs() < 1e-3, "aligned blade should produce no side force, got {f:?}");
        // Drag pushes the blade WITH the relative flow (a passive object
        // gets carried along by the fluid moving past it), so a small
        // positive (flow-aligned) force remains — just the baseline
        // parasitic CD0, not zero.
        assert!(f.x > 0.0, "aligned blade should still drag along the flow, got {f:?}");

        // Sanity check that the zero above is really about ALIGNMENT, not
        // just "this function returns small numbers": a blade broadside to
        // a flow of the same magnitude (delta=0, but the flow itself is
        // purely lateral this time, e.g. a strong yaw sweep with no surge)
        // must produce a far bigger force, not another near-zero.
        let f_broadside = rudder_force(Vec2::new(0.0, 2.5), 0.0, &foil);
        assert!(
            f_broadside.length() > f.length() * 5.0,
            "a blade broadside to the flow should push much harder than one \
             aligned with it, got {f_broadside:?} vs {f:?}"
        );
    }

    #[test]
    fn following_helm_stays_attached_but_opposing_helm_stalls_while_spinning() {
        // A boat making way and ALREADY spinning clockwise (negative yaw
        // rate, per the sign convention used throughout this file): the
        // yaw sweep at the rudder biases the effective flow the same way a
        // starboard helm biases the chord, so committing FURTHER into the
        // turn (starboard, following the spin) rotates the effective angle
        // of attack back toward alignment even at full deflection, while
        // trying to check the spin (port, opposing it) pushes the angle
        // deeper into stall from the first few degrees of helm. Neither
        // side asserts the sign by hand-derivation — both are read off
        // rudder_lift_drag's own stall threshold, the same one `tick` uses.
        let surge = 2.0;
        let w = -0.3; // spinning clockwise
        let rudder_x = BoatDesign::oday_39().rudder.x;
        let flow = Vec2::new(-surge, -w * rudder_x);

        let alpha_mag = |rudder_cmd: f32| {
            let delta = -rudder_cmd * RUDDER_MAX_DEG.to_radians();
            let fhat = flow / flow.length();
            let chord = Vec2::new(-delta.cos(), delta.sin());
            chord.perp_dot(fhat).atan2(chord.dot(fhat)).abs()
        };
        let stall_on = 0.30_f32; // RUDDER_STALL_ON, kept in sync by the assertions below
        let port_10pct = alpha_mag(-0.1);
        let stbd_100pct = alpha_mag(1.0);
        assert!(
            port_10pct > stall_on,
            "a mere 10% of opposing (port) helm should already be stalled while \
             spinning this hard, got {port_10pct} rad"
        );
        assert!(
            stbd_100pct < stall_on,
            "full following (starboard) helm should re-attach the flow while \
             spinning this hard, got {stbd_100pct} rad"
        );
    }

    #[test]
    fn backing_reverses_the_helm() {
        // Making sternway the flow comes over the blade from astern, so
        // the same helm yaws the boat the other way (and the stern, which
        // now leads, seeks the helm side) — the fold in `rudder_lift_drag`
        // at
        // work. Same injected speed magnitude both ways, engine off.
        let heading_after = |u: f32| {
            let mut sim = Sim::new();
            let h0 = sim.boat_pose().1;
            sim.set_forward_speed(u);
            let input = InputState { throttle: 0.0, rudder: 1.0 };
            run_input(&mut sim, &Env::CALM, &input, 1.5);
            sim.boat_pose().1 - h0
        };
        let ahead = heading_after(1.5);
        let astern = heading_after(-1.5);
        assert!(ahead < -0.01, "starboard helm with headway: clockwise, got {ahead}");
        assert!(astern > 0.01, "starboard helm with sternway: anticlockwise, got {astern}");
    }

    #[test]
    fn a_heavier_boat_gathers_way_more_slowly() {
        // Same keel (same drag), same engine — only the displacement
        // differs, so the eventual equilibrium speed is identical but the
        // heavier boat takes longer to get there. Checked mid-transient:
        // from rest at full ahead, the light boat leads clearly. The two
        // displacements are the real spread the presets cover (O'Day 39
        // vs Alajuela 38).
        let speed_after = |displacement_kg: f32| {
            let design = BoatDesign { displacement_kg, ..BoatDesign::oday_39() };
            let mut sim = Sim::new_with_design(&design);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            sim.boat_vel().0.length()
        };
        let light = speed_after(8_165.0);
        let heavy = speed_after(11_800.0);
        assert!(
            light > heavy * 1.05,
            "expected the light boat to be clearly ahead mid-transient: \
             light {light} m/s vs heavy {heavy} m/s"
        );
    }

    #[test]
    fn engine_spools_rather_than_steps() {
        // The delivered engine response chases the telegraph with a
        // first-order lag (THROTTLE_TAU = 0.4 s): one time constant after
        // slamming to full ahead it sits near 1 - 1/e ≈ 0.63, neither still
        // at zero nor already at full.
        let mut sim = Sim::new();
        run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 0.4);
        let n = sim.engine();
        assert!(
            n > 0.55 && n < 0.72,
            "expected the engine near 1-1/e one time constant in, got {n}"
        );
    }
    #[test]
    fn ittc57_cf_is_finite_across_the_low_reynolds_pole() {
        // The raw ITTC-1957 line has a genuine mathematical pole at
        // Re = 100 (log10(Re) = 2 zeroes the denominator) — seen live as
        // a wind-pinned boat getting launched sideways when float noise
        // drifted its Reynolds number across it, and flagged in review.
        // The ITTC_RE_FLOOR clamp must keep Cf finite (and small) through
        // the whole sub-validity range, pole included.
        for re in [0.0, 99.0, 100.0, 101.0, ITTC_RE_FLOOR] {
            let cf = ittc57_cf(re);
            assert!(cf.is_finite(), "expected finite Cf at Re = {re}, got {cf}");
            assert!(cf > 0.0 && cf < 0.01, "clamped Cf should be small, got {cf} at Re = {re}");
        }
    }

    #[test]
    fn new_continuing_preserves_state_but_updates_coefficients() {
        // Set up a sim with non-default pose, velocity, yaw rate, and
        // engine spool, then continue into a different design — the
        // kinematic state must survive while the keel coefficients change.
        let design_a = BoatDesign::hallberg_rassy_38();
        let design_b = BoatDesign::oday_39();
        let mut sim = Sim::new_open_water(&design_a);
        sim.set_pose(5.0, -10.0, 0.5);
        sim.set_forward_speed(2.0);
        sim.set_yaw_rate(0.1);
        // Spool the engine partway by running a few ticks at full ahead.
        run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 0.2);
        // Snapshot the state AFTER those ticks (pose drifted a bit, engine
        // spooled partway — that's the state we want transplanted).
        let (pos, heading) = sim.boat_pose();
        let (vel, angvel) = sim.boat_vel();
        let engine = sim.engine();
        assert!(engine > 0.0, "engine should have spooled up");
        let keel_a = sim.keel();

        let sim2 = sim.new_continuing(&design_b);
        let (pos2, heading2) = sim2.boat_pose();
        let (vel2, angvel2) = sim2.boat_vel();

        // Kinematic state preserved exactly.
        assert_eq!(pos, pos2, "position must survive");
        assert_eq!(heading, heading2, "heading must survive");
        assert_eq!(vel, vel2, "velocity must survive");
        assert_eq!(angvel, angvel2, "yaw rate must survive");
        assert_eq!(engine, sim2.engine(), "engine spool must survive");

        // Keel coefficients must reflect the NEW design.
        let keel_b = sim2.keel();
        assert_ne!(keel_a, keel_b, "keel coefficients must change with the new design");
        let expected = design_b.keel.derive();
        assert_eq!(keel_b, expected, "keel must match the new design's derived values");
    }
}
