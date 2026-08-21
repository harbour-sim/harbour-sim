//! Mooring lines — the ropes that hold the boat to the harbour.
//!
//! A line is NOT a Rapier joint, deliberately, for the same reason the
//! hull's drag isn't Rapier damping (see `sim.rs`): every force in this
//! sim is computed in `tick` from modeled geometry, and a rope has two
//! properties a joint can't express anyway.
//!
//! 1. It is **unilateral** — it pulls, never pushes. Slack is the normal
//!    state of a tended line and produces exactly zero force.
//! 2. It is **elastic**, and that elasticity is the whole point: a nylon
//!    dock line stretching is what stops eight and a half tonnes without
//!    snatching. A rigid distance constraint would delete the one
//!    behaviour worth simulating.
//!
//! So a line is a spring that only pulls, made fast at whatever length
//! the crew got it ashore at, and shortened or surged from there by hand.
//! The force is applied at the fairlead's own world point, which is what
//! makes SPRING LINES work without any special case: a midship line plus
//! ahead thrust plus helm away yaws the boat alongside, straight out of
//! the existing rudder/keel model.

use crate::sim::{HULL_PTS, PHYSICS_DT, hull_half_beam};
use glam::Vec2;

// ---------------------------------------------------------------------------
// The rope itself
// ---------------------------------------------------------------------------

/// Modeled line: 14 mm three-strand nylon, the usual dock line size for a
/// 38-footer of this displacement. One line type for now — a polyester
/// line (far stiffer, much less stretch) would be a per-line material
/// choice later, and the whole curve below is what would vary.
pub const LINE_DIAMETER: f32 = 0.014;

/// Reference breaking load, and the diameter it was measured at: 1/2"
/// (12.7 mm) three-strand nylon at 6400 lbf = 28.5 kN, as quoted by
/// suppliers testing to Cordage Institute / ASTM D-4268. The same size is
/// also sold at 5750 lbf — the spread between manufacturers and grades is
/// the real uncertainty in this number, not the unit conversion.
const REF_MBL: f32 = 28_500.0;
const REF_DIAMETER: f32 = 0.0127;

/// Minimum breaking load (N): the reference figure scaled by d², since a
/// rope's strength goes with its cross-sectional area. ≈ 34.6 kN.
pub const LINE_MBL: f32 =
    REF_MBL * (LINE_DIAMETER * LINE_DIAMETER) / (REF_DIAMETER * REF_DIAMETER);

// Three-strand nylon's load–elongation curve. Nylon is not a linear
// spring and the difference matters at both ends: a linear `EA·ε` line
// either breaks implausibly early or holds implausibly stiff at the low
// loads a moored boat actually lives at.
//
// Two published points on the curve anchor it: **2.5 % extension at 10 %
// of breaking load, and ~20 % at 50 %**. Fitting a power law
//
//     T(ε) = MBL · (ε / ε_break)^p
//
// through them gives p = ln(0.50/0.10) / ln(0.20/0.025) = ln5/ln8 =
// 0.774, and ε_break = 0.025 / 0.10^(1/p) = 0.49.
//
// That exponent is BELOW 1 — the curve softens as it loads, rather than
// stiffening (each further unit of load buys proportionally more stretch).
// The independent check that this is really the shape of the published
// data, and not an artefact of fitting two points: the extrapolated break
// elongation, ~49 %, lands squarely inside the 40–55 % that three-strand
// nylon is separately published as breaking at. Nothing in the fit put it
// there. `nylon_curve_matches_its_published_anchor_points` pins both
// constants back against the two anchor points so they can't drift apart.
//
// Known simplification: BELOW the lower anchor point the power law
// extrapolates a stiffer toe than real rope, which has a soft
// constructional-stretch region as the lay beds in. Harmless here — the
// loads a moored boat actually lives at land a few centimetres either
// way on a dock line — and the fix, if it ever matters, is a third
// published point down there rather than a shape change.
//
// (The sub-linear exponent means tangent stiffness is formally infinite
// at ε = 0. Harmless: it grows only as ε^-0.226, so even at ε = 10⁻¹²
// the implied natural period is still ~45 physics steps, and the FORCE
// there is milli-newtons. No take-up fudge needed.)
#[cfg(test)]
const ELONG_LO: (f32, f32) = (0.025, 0.10);
#[cfg(test)]
const ELONG_HI: (f32, f32) = (0.20, 0.50);
/// Exponent of the load–elongation power law — see the block above.
const LINE_CURVE_P: f32 = 0.774;
/// Strain at break, extrapolated from the same two points.
pub const LINE_STRAIN_BREAK: f32 = 0.490;

/// Internal damping, as a fraction of critical for the line/boat pair.
///
/// A calibration inside a sourced band rather than a measured constant,
/// and worth being precise about which. Nylon rope's hysteresis — the
/// reason it warms up under cyclic load — is commonly put at a 20–50 %
/// energy loss per cycle, and the model's own snub benchmark spans that
/// band: at 0.05 the rope gives back 61 % of the boat's kinetic energy,
/// at 0.10 48 %, at 0.20 32 %, at 0.35 20 % (measured 2026-08-20, 2.5 kn
/// onto an 8 m scope).
///
/// 0.20 is the DAMPED end of that band, chosen deliberately (owner
/// report: "the ropes are too bouncy"). Two things justify sitting there
/// rather than at the material's own figure: the whole rope is one
/// Kelvin–Voigt element, and — the bigger omission — nothing here models
/// the friction of a line surging round a cleat and through a fairlead,
/// which in life eats a real share of a snatch. The remaining
/// springiness is nylon behaving like nylon; it is supposed to give.
const LINE_DAMPING_RATIO: f32 = 0.20;

/// Strain at `dist` for a line of unstretched length `scope`. Negative
/// (or zero) means slack.
fn strain(scope: f32, dist: f32) -> f32 {
    (dist - scope) / scope.max(1e-3)
}

/// Elastic tension (N) in a line of unstretched length `scope` stretched
/// to `dist`. Exactly zero when slack — a rope pulls, it never pushes.
pub fn line_tension(scope: f32, dist: f32) -> f32 {
    let e = strain(scope, dist);
    if e <= 0.0 {
        return 0.0;
    }
    LINE_MBL * (e / LINE_STRAIN_BREAK).powf(LINE_CURVE_P)
}

/// Tangent stiffness dT/d(length), N/m — what a damper has to be sized
/// against. `dT/dL = (dT/dε)/scope = p·T/(ε·scope)`.
pub fn line_stiffness(scope: f32, dist: f32) -> f32 {
    let e = strain(scope, dist);
    if e <= 0.0 {
        return 0.0;
    }
    LINE_CURVE_P * line_tension(scope, dist) / (e * scope.max(1e-3))
}

/// Cap on the damping force, as a multiple of the elastic tension the
/// line is carrying at that moment.
///
/// This is not a fudge, it is the shape of the physics: a rope's damping
/// is HYSTERETIC — the energy lost per cycle is a fraction of the energy
/// stored — so the damping force scales with the load and cannot run
/// away from it. Without the cap the model produced a real artefact
/// (measured 2026-08-20: 1.6 kN of damping against 267 N of elastic
/// tension at the instant a line came up). The cause is that the
/// load-elongation curve is sub-linear, so its TANGENT stiffness is
/// formally infinite at zero strain, and a damper sized from that
/// stiffness spikes at exactly the moment of first contact — which is
/// felt as a snatch the rope should not give.
const LINE_DAMP_FORCE_CAP: f32 = 1.0;

/// Total pull (N) along the line: elastic tension plus the material's
/// damping, given how fast it is being stretched (`stretch_rate` > 0 =
/// lengthening) and the mass it works against. The damping term is
/// bounded by the tension in both directions, so it can never turn a
/// rope into a strut, nor kick as the rope takes up.
pub fn line_pull(scope: f32, dist: f32, stretch_rate: f32, mass: f32) -> f32 {
    let t = line_tension(scope, dist);
    if t <= 0.0 {
        return 0.0;
    }
    let c = 2.0 * LINE_DAMPING_RATIO * (line_stiffness(scope, dist) * mass).sqrt();
    let damp = (c * stretch_rate).clamp(-t, t * LINE_DAMP_FORCE_CAP);
    t + damp
}

// ---------------------------------------------------------------------------
// Handling: where a line lands, how long it takes, how it is tended
// ---------------------------------------------------------------------------

// What the FITTINGS at each end of a line will take before they let go.
//
// A rope is rarely the weak link, and this is the mooring lesson people
// learn the expensive way: a cleat pulls out of a deck, taking a chunk of
// laminate with it, long before good nylon parts. BoatUS Foundation's
// cleat testing had real cleat ASSEMBLIES failing between 1,190 and
// 7,500 lbf (5.3–33 kN) — and note what was tested there: BOAT deck
// hardware, bolted through a deck. `DECK_FITTING_MBL` sits mid-band, for
// a backing-plated cleat on a 38-footer.
//
// SHORE hardware is the more robust end (owner call, 2026-08-20, and the
// honest reading of the source above: it is not evidence about pontoon
// cleats at all). A marina's cleat is commercial gear through-bolted to a
// float frame and rated for the berth, so it is set above any boat's own
// fitting and below the rope. The ordering is what matters more than the
// value: deck fitting < shore cleat < rope, so what tears out is the
// thing on the BOAT, which is what this models — and a mooring POLE has
// no fitting at all, the line goes round it.
//
// The consequence that matters for feel: a snatch load that would
// otherwise store enough energy to catapult the boat now tears the cleat
// off the deck instead, which is exactly what it does in life.
pub const DECK_FITTING_MBL: f32 = 14_000.0;
pub const PONTOON_CLEAT_MBL: f32 = 25_000.0;

/// What gave way when a line let go.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Gave {
    /// The rope itself parted.
    Rope,
    /// The fairlead or cleat on this boat's own deck.
    Fairlead,
    /// The cleat on the pontoon.
    Cleat,
    /// The fitting on the boat at the other end.
    Neighbour,
}

impl Gave {
    pub fn describe(self) -> &'static str {
        match self {
            Gave::Rope => "the line parted",
            Gave::Fairlead => "the deck fitting tore out",
            Gave::Cleat => "the pontoon cleat tore out",
            Gave::Neighbour => "the fitting on her deck tore out",
        }
    }
}

/// A fitting that has been torn out. Identified by WHERE it was rather
/// than by an index, because the marina's cleats are generated geometry
/// with no identity of their own — the position they came from is the
/// only stable name they have.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Fitting {
    /// A cleat ashore, at this point.
    Shore(Vec2),
    /// A deck fitting on a hull.
    Deck(Hull, Fairlead),
}

/// The weakest link in a line's load path, and what it is: the rope, the
/// fitting on this boat, or whatever the far end is made fast to. Every
/// line runs through this boat's own deck fitting, so that is always in
/// the chain.
pub fn weakest_link(anchor: Anchor) -> (f32, Gave) {
    let far = match anchor {
        // The line goes ROUND a pole; nothing to tear out of it.
        Anchor::Shore { kind: ShoreKind::Pole, .. } => (LINE_MBL, Gave::Rope),
        Anchor::Shore { kind: ShoreKind::Cleat, .. } => (PONTOON_CLEAT_MBL, Gave::Cleat),
        Anchor::Boat { .. } => (DECK_FITTING_MBL, Gave::Neighbour),
    };
    let mine = (DECK_FITTING_MBL, Gave::Fairlead);
    let rope = (LINE_MBL, Gave::Rope);
    // Ours first, so a tie with an identical fitting at the far end
    // blames this boat — we are the one loading it.
    [mine, far, rope].into_iter().min_by(|a, b| a.0.total_cmp(&b.0)).expect("three links")
}

/// How far a line can be got ashore, metres. A heaving line goes further
/// than this in real life; this is the game constraint that stops rope
/// from substituting for boat handling — you have to bring the boat
/// alongside before you can use it.
///
/// A DEFAULT, not a constant of the world: the live value is a player
/// setting (`CrewLimits::reach`). At 4 m she has to be close enough to
/// step ashore, which is the point.
pub const LINE_REACH: f32 = 4.0;

/// Range the reach setting is clamped to: from arm's length to the
/// length of the rope itself. The top end is DERIVED from
/// `LINE_SCOPE_MAX` rather than picked, because you cannot get a line
/// ashore further than the rope reaches: a line made fast beyond it
/// would be born longer than its own rope, and the first surge order
/// would then clamp the scope back and shorten it by metres in a single
/// tick — several kN of tension and a likely torn-out fitting, from a
/// PAY-OUT order (CodeRabbit review, 2026-08-21; the two constants were
/// 25 m and 20 m and could drift apart).
pub const LINE_REACH_MIN: f32 = 1.0;
pub const LINE_REACH_MAX: f32 = LINE_SCOPE_MAX;

/// Default rate at which a line goes ashore (m/s of connection
/// distance). Getting a line on takes TIME, proportional to how far it
/// has to travel: a metre off the pontoon it is a step and a turn on the
/// cleat; at full reach it is a heaving line thrown, gathered and made
/// fast — about three seconds at this rate. Long throws being slow is
/// what makes closing the distance first worth doing.
///
/// This is a DEFAULT, not a constant of the world: the live value is a
/// configuration setting the player can wind up or down
/// (`InputState::line_pass_speed`, adjustable from the mooring panel).
/// It rides the input stream rather than sitting on the `Sim` so that a
/// recording replays with the same crew speed it was made at.
pub const LINE_PASS_SPEED: f32 = 4.0;

/// Range the setting is clamped to. The slow end is a crew fumbling a
/// long throw; the fast end is near enough instant at any range, for
/// anyone who would rather practise the boat handling than the rope
/// work.
pub const LINE_PASS_SPEED_MIN: f32 = 1.0;
pub const LINE_PASS_SPEED_MAX: f32 = 20.0;

/// Free-running haul-in rate (m/s), hand over hand.
pub const LINE_HAUL_RATE: f32 = 0.6;

/// What the crew can pull, in KILOS — the weight they could hold hanging
/// on the rope, which is how anyone actually talks about this. Hauling
/// derates linearly to zero as the line's own tension approaches it, and
/// that is the real limit being modelled: you cannot winch 8.5 tonnes up
/// to the pontoon against a breeze by hand. You rig a spring and use the
/// engine, and the crew gathers the slack you make.
///
/// A DEFAULT, not a constant of the world: the live value is a player
/// setting (`CrewLimits::haul_kg`).
pub const LINE_HAUL_KG: f32 = 10.0;

/// Range the setting is clamped to: a child on the foredeck at one end,
/// two strong people tailing at the other.
pub const LINE_HAUL_KG_MIN: f32 = 1.0;
pub const LINE_HAUL_KG_MAX: f32 = 150.0;

/// The crew's own limits, as opposed to the rope's or the boat's. Player
/// SETTINGS, carried in the input stream rather than held on the `Sim`,
/// so a recording replays with the crew it was made with — the same
/// split as the helm and the engine telegraph.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrewLimits {
    /// How fast a line goes ashore, m/s of connection distance.
    pub pass_speed: f32,
    /// What one pair of hands can hold, kilos.
    pub haul_kg: f32,
    /// How far a line can be got ashore, metres.
    pub reach: f32,
}

impl CrewLimits {
    pub const DEFAULT: CrewLimits =
        CrewLimits { pass_speed: LINE_PASS_SPEED, haul_kg: LINE_HAUL_KG, reach: LINE_REACH };

    /// Every field brought inside its published range. Applied once at
    /// the top of `tick`, exactly like the throttle and helm clamps: a
    /// corrupt recording must not be able to command a super-physical
    /// crew.
    pub fn clamped(self) -> CrewLimits {
        CrewLimits {
            pass_speed: self.pass_speed.clamp(LINE_PASS_SPEED_MIN, LINE_PASS_SPEED_MAX),
            haul_kg: self.haul_kg.clamp(LINE_HAUL_KG_MIN, LINE_HAUL_KG_MAX),
            reach: self.reach.clamp(LINE_REACH_MIN, LINE_REACH_MAX),
        }
    }

    /// The haul limit as a FORCE (N), which is what the tension it is
    /// compared against is measured in.
    pub fn haul_force(self) -> f32 {
        self.haul_kg * crate::sim::G_EARTH
    }
}

impl Default for CrewLimits {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Surging rate (m/s) when paying out — a turn round a cleat, let it run.
/// Faster than hauling: gravity and load are helping.
pub const LINE_PAY_RATE: f32 = 1.2;

/// Shortest a line can be hauled to: the fairlead cannot be pulled into
/// the cleat.
pub const LINE_SCOPE_MIN: f32 = 0.3;

/// Longest a line can be surged to — the rope's own length. Past this
/// there is no more rope to pay out.
pub const LINE_SCOPE_MAX: f32 = 20.0;

/// How many lines the boat carries. Also bounds the sim's state against a
/// corrupt recording ordering an unbounded number of them.
pub const LINE_COUNT_MAX: usize = 6;

/// Where a line is made fast on the boat. Positions are read off
/// `HULL_PTS` (see `local`), so a fairlead is always exactly on the
/// outline the renderer draws and the collider uses.
///
/// Deliberately NO stem-head or stern fairlead (owner call, 2026-08-20):
/// on the centreline they sit right between their own port and starboard
/// pair, which makes the nearest-handle picker ambiguous exactly where
/// you most want to be sure which side you are leading from — and a bow
/// line off the stem is barely a different rope from one off the port
/// bow. Six handles, three a side.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Fairlead {
    PortBow,
    StbdBow,
    PortWaist,
    StbdWaist,
    PortQuarter,
    StbdQuarter,
}

impl Fairlead {
    pub const ALL: [Fairlead; 6] = [
        Fairlead::PortBow,
        Fairlead::StbdBow,
        Fairlead::PortWaist,
        Fairlead::StbdWaist,
        Fairlead::PortQuarter,
        Fairlead::StbdQuarter,
    ];

    /// Boat-local position (m, bow = +x, port = +y), derived from
    /// `HULL_PTS`: the bow pair sit on its shoulder vertex, the quarters
    /// midway along its aft run, and the waist pair amidships — all with
    /// the outline's own half-beam at that station, never a hand-placed
    /// offset.
    pub fn local(self) -> Vec2 {
        let shoulder_x = HULL_PTS[1].0;
        let quarter_x = (HULL_PTS[2].0 + HULL_PTS[3].0) * 0.5;
        let (x, side) = match self {
            Fairlead::PortBow => (shoulder_x, 1.0),
            Fairlead::StbdBow => (shoulder_x, -1.0),
            Fairlead::PortWaist => (0.0, 1.0),
            Fairlead::StbdWaist => (0.0, -1.0),
            Fairlead::PortQuarter => (quarter_x, 1.0),
            Fairlead::StbdQuarter => (quarter_x, -1.0),
        };
        Vec2::new(x, side * hull_half_beam(x))
    }

    /// Short label for the HUD.
    pub fn label(self) -> &'static str {
        match self {
            Fairlead::PortBow => "port bow",
            Fairlead::StbdBow => "stbd bow",
            Fairlead::PortWaist => "port waist",
            Fairlead::StbdWaist => "stbd waist",
            Fairlead::PortQuarter => "port quarter",
            Fairlead::StbdQuarter => "stbd quarter",
        }
    }
}

/// What kind of fixed point ashore a line is belayed to. Presentation
/// (and, later, scoring) only — physically both are the same thing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ShoreKind {
    /// A cleat on a pontoon face.
    Cleat,
    /// One of the marina's mooring poles.
    Pole,
}

/// Where a line's far end is made fast.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Anchor {
    /// A fixed point ashore: a pontoon cleat or a mooring pole.
    Shore { pos: Vec2, kind: ShoreKind },
    /// A fairlead on ANOTHER boat (2026-08-20): rafting up alongside, or
    /// taking a line to your neighbour while you get sorted out. Unlike
    /// a cleat this end MOVES, and the rope pulls on it just as hard as
    /// it pulls on you — `step_lines` applies the equal and opposite
    /// force at the far hull's own fairlead, so leaning on a neighbour's
    /// rope drags the neighbour.
    Boat { hull: Hull, fairlead: Fairlead },
}

/// A line's stage of life.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum LineState {
    /// Being passed ashore — the heaving line is in the air, or the crew
    /// is stepping off with the eye. Nothing holds yet. `total` is how
    /// long the pass takes end to end, so the renderer can show it
    /// running out; `reach` is the throw's OWN limit, carried from the
    /// moment it left the hand so that winding the reach setting while a
    /// line is in the air cannot retroactively lose it.
    Passing { elapsed: f32, total: f32, reach: f32 },
    /// Made fast, working.
    Fast,
}

/// Which hull a line is made fast to. The player's boat and every boat in
/// the moored fleet lie to the same kind of rope, computed by the same
/// code — the fleet's lines are not decoration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Hull {
    Player,
    /// Index into the moored fleet, in `moored_boats()` order.
    Moored(u16),
}

/// One mooring line.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Line {
    /// The hull this line is made fast to.
    pub hull: Hull,
    /// Stable identity for the frontend to select and tend by. Ids are
    /// monotonic and NEVER recycled: a cast-off line's id must not come
    /// back and quietly re-target a command aimed at the old one (the
    /// same lesson the touch-id handling in the frontend learned).
    pub id: u32,
    pub fairlead: Fairlead,
    pub anchor: Anchor,
    /// Unstretched length (m) — the length it was made fast at, plus
    /// whatever has since been hauled in or surged out. Meaningless while
    /// `Passing`.
    pub scope: f32,
    pub state: LineState,
    /// Pull (N) computed by the last `tick`, for the HUD and for the
    /// haul-rate derate. Zero while slack or passing.
    pub tension: f32,
}

impl Line {
    /// Fraction of the pass completed, 0..1 (1 once fast).
    pub fn pass_progress(&self) -> f32 {
        match self.state {
            LineState::Passing { elapsed, total, .. } => (elapsed / total.max(1e-3)).clamp(0.0, 1.0),
            LineState::Fast => 1.0,
        }
    }

    /// Test helper: a shore anchor's world point.
    #[cfg(test)]
    pub fn anchor_pos_for_test(&self) -> Vec2 {
        match self.anchor {
            Anchor::Shore { pos, .. } => pos,
            Anchor::Boat { .. } => unreachable!("shore anchors only in these tests"),
        }
    }

    pub fn is_fast(&self) -> bool {
        self.state == LineState::Fast
    }
}

/// The crew's orders, carried in `InputState` — one per tick, which is
/// all a pair of hands can issue at 120 Hz. Lines themselves are sim
/// STATE (like the engine spool), advanced only inside `tick`; the input
/// stream carries only the orders, so a recording replays them exactly.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum LineCommand {
    /// Pass a line from `fairlead` to `anchor`. Ignored if the anchor is
    /// out of reach or the boat has no line left to spare.
    MakeFast { fairlead: Fairlead, anchor: Anchor },
    /// Work a made-fast line: `rate` +1 hauls in at the full hand-over-
    /// hand rate, -1 surges out, 0 holds. Sent every tick the crew is
    /// pulling, so stopping is just the absence of the order.
    Tend { id: u32, rate: f32 },
    /// Let it go.
    CastOff { id: u32 },
}

/// The fitting an anchor depends on, if it has one — a pole has none.
pub fn anchor_fitting(anchor: Anchor) -> Option<Fitting> {
    match anchor {
        Anchor::Shore { kind: ShoreKind::Pole, .. } => None,
        Anchor::Shore { pos, kind: ShoreKind::Cleat } => Some(Fitting::Shore(pos)),
        Anchor::Boat { hull, fairlead } => Some(Fitting::Deck(hull, fairlead)),
    }
}

pub fn fitting_broken(broken: &[Fitting], f: Fitting) -> bool {
    broken.contains(&f)
}

/// Apply one order to the line set. `boat` maps a fairlead to its world
/// position, so this stays independent of how the caller stores the pose.
pub(crate) fn apply_command(
    lines: &mut Vec<Line>,
    next_id: &mut u32,
    cmd: LineCommand,
    crew: CrewLimits,
    broken: &[Fitting],
    fairlead_world: impl Fn(Fairlead) -> Vec2,
    anchor_world: impl Fn(Anchor) -> Vec2,
) {
    match cmd {
        LineCommand::MakeFast { fairlead, anchor } => {
            // The cap is on the PLAYER's ropes — the moored fleet's lines
            // share this set but are not the crew's to spend.
            if lines.iter().filter(|l| l.hull == Hull::Player).count() >= LINE_COUNT_MAX {
                return;
            }
            // One rope per fairlead. Doubling up is real seamanship, but
            // in a game a second line from a handle that already has one
            // is almost always a fumbled gesture rather than an intent
            // (owner call, 2026-08-20) — and it would sit exactly on top
            // of the first, unselectable.
            if lines.iter().any(|l| l.hull == Hull::Player && l.fairlead == fairlead) {
                return;
            }
            // A fitting that has been torn out stays torn out: you
            // cannot re-use the cleat you just pulled off the pontoon,
            // nor your own deck fitting once it has gone.
            if fitting_broken(broken, Fitting::Deck(Hull::Player, fairlead))
                || anchor_fitting(anchor).is_some_and(|f| fitting_broken(broken, f))
            {
                return;
            }
            // Not to your own boat. A rope from one of her fairleads to
            // another pulls at only one end (`anchor_of` has no second
            // hull to push back on), so hauling it in would drive her
            // along on her own bootstraps. The UI never offers it — the
            // reachable set is shore fittings and OTHER hulls — but a
            // command is input, and input is checked here for the same
            // reason `tick` clamps a corrupt recording's throttle.
            if matches!(anchor, Anchor::Boat { hull: Hull::Player, .. }) {
                return;
            }
            let dist = (anchor_world(anchor) - fairlead_world(fairlead)).length();
            if dist > crew.reach {
                return; // nobody can get a line that far
            }
            let id = *next_id;
            *next_id += 1;
            lines.push(Line {
                hull: Hull::Player,
                id,
                fairlead,
                anchor,
                scope: dist,
                state: LineState::Passing {
                    elapsed: 0.0,
                    total: (dist / crew.pass_speed).max(PHYSICS_DT),
                    reach: crew.reach,
                },
                tension: 0.0,
            });
        }
        LineCommand::Tend { id, rate } => {
            if let Some(l) =
                lines.iter_mut().find(|l| l.id == id && l.is_fast() && l.hull == Hull::Player)
            {
                let rate = rate.clamp(-1.0, 1.0);
                if rate > 0.0 {
                    // Hauling in: what you can gather is what the line
                    // isn't already holding.
                    let slip = (1.0 - l.tension / crew.haul_force()).clamp(0.0, 1.0);
                    l.scope = (l.scope - rate * LINE_HAUL_RATE * slip * PHYSICS_DT)
                        .max(LINE_SCOPE_MIN);
                } else if rate < 0.0 {
                    l.scope =
                        (l.scope - rate * LINE_PAY_RATE * PHYSICS_DT).min(LINE_SCOPE_MAX);
                }
            }
        }
        LineCommand::CastOff { id } => {
            lines.retain(|l| l.id != id || l.hull != Hull::Player);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Anchor resolver for these unit tests: everything here is belayed
    /// ashore, so the far end never moves.
    fn shore_at(a: Anchor) -> Vec2 {
        match a {
            Anchor::Shore { pos, .. } => pos,
            Anchor::Boat { .. } => unreachable!("no fleet in these unit tests"),
        }
    }

    /// The two constants of the load–elongation curve are DERIVED from
    /// the two published anchor points, not asserted independently of
    /// them — this recomputes the derivation and fails if either has
    /// drifted (the same derive-don't-assert discipline as `RUDDER_AR`).
    /// It also checks the extrapolated break strain against the
    /// separately published 40–55 % that three-strand nylon breaks at:
    /// nothing in the two-point fit put it in that range, so landing
    /// there is real corroboration that the power law is the right shape
    /// for this material.
    #[test]
    fn nylon_curve_matches_its_published_anchor_points() {
        let (e_lo, t_lo) = ELONG_LO;
        let (e_hi, t_hi) = ELONG_HI;
        let p = (t_hi / t_lo).ln() / (e_hi / e_lo).ln();
        let eps_break = e_lo / t_lo.powf(1.0 / p);
        assert!((p - LINE_CURVE_P).abs() < 0.002, "exponent drifted: {p}");
        assert!(
            (eps_break - LINE_STRAIN_BREAK).abs() < 0.005,
            "break strain drifted: {eps_break}"
        );
        assert!(
            (0.40..=0.55).contains(&LINE_STRAIN_BREAK),
            "break strain left the published band for 3-strand nylon"
        );
        // And the shipped curve really does pass through both points.
        for (e, frac) in [ELONG_LO, ELONG_HI] {
            let t = line_tension(1.0, 1.0 + e);
            assert!(
                (t / LINE_MBL - frac).abs() < 0.01,
                "curve misses its anchor point at {e}: {} vs {frac}",
                t / LINE_MBL
            );
        }
    }

    #[test]
    fn a_slack_line_pulls_nothing() {
        assert_eq!(line_tension(10.0, 9.0), 0.0);
        assert_eq!(line_tension(10.0, 10.0), 0.0);
        // Not even when the boat is charging away from the anchor: until
        // the slack is out, there is nothing to pull on.
        assert_eq!(line_pull(10.0, 9.0, 3.0, 8500.0), 0.0);
    }

    #[test]
    fn a_rope_pulls_but_never_pushes() {
        // Fully loaded line, boat rushing back toward the anchor faster
        // than any damper could resist: the pull bottoms out at zero
        // rather than turning into a strut that shoves the boat away.
        let (scope, dist) = (10.0, 11.0);
        assert!(line_tension(scope, dist) > 0.0);
        assert_eq!(line_pull(scope, dist, -50.0, 8500.0), 0.0);
    }

    #[test]
    fn tension_rises_with_stretch_and_hits_the_breaking_load_at_break_strain() {
        let t1 = line_tension(10.0, 10.2);
        let t2 = line_tension(10.0, 11.0);
        assert!(t2 > t1 && t1 > 0.0);
        let at_break = line_tension(10.0, 10.0 * (1.0 + LINE_STRAIN_BREAK));
        assert!((at_break - LINE_MBL).abs() < LINE_MBL * 0.01);
        // Scale check at the two loads that matter. A working breeze
        // load (~1.1 kN — this hull in 20 kn of beam wind, i.e. 3 % of
        // breaking load) should give a few centimetres on a 10 m line:
        // a taut line, not a bungee. The stretch nylon is famous for
        // belongs an order of magnitude higher up, and at half breaking
        // load the same line should give metres.
        let stretch_at = |load: f32| {
            (1..5000)
                .map(|i| 10.0 + i as f32 * 0.001)
                .find(|&d| line_tension(10.0, d) >= load)
                .map(|d| d - 10.0)
                .expect("5 m of stretch covers any load up to breaking")
        };
        let working = stretch_at(1100.0);
        assert!((0.02..0.30).contains(&working), "10 m line gives {working} m under 1.1 kN");
        assert!(stretch_at(LINE_MBL * 0.5) > 1.0, "half breaking load should give metres");
    }

    /// Fairleads are read off `HULL_PTS`, so every one of them lands
    /// exactly on the outline the renderer draws and the collider uses —
    /// no hand-placed deck fittings floating off the hull.
    #[test]
    fn every_fairlead_sits_on_the_hull_outline() {
        for f in Fairlead::ALL {
            let p = f.local();
            assert!(
                (p.y.abs() - hull_half_beam(p.x)).abs() < 1e-4,
                "{} is off the outline at {p:?}",
                f.label()
            );
        }
        // The bow pair sit on the outline's shoulder vertex, the
        // quarters midway along its aft run — both read off HULL_PTS.
        assert_eq!(Fairlead::PortBow.local().x, HULL_PTS[1].0);
        assert_eq!(
            Fairlead::PortQuarter.local().x,
            (HULL_PTS[2].0 + HULL_PTS[3].0) * 0.5
        );
        // No handle on the centreline: every one has a side.
        assert!(Fairlead::ALL.iter().all(|f| f.local().y.abs() > 0.1));
        assert!(Fairlead::PortWaist.local().y > 0.0);
        assert!(Fairlead::StbdWaist.local().y < 0.0);
    }

    fn fast_line(scope: f32, tension: f32) -> Vec<Line> {
        vec![Line {
            hull: Hull::Player,
            id: 1,
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Shore { pos: Vec2::ZERO, kind: ShoreKind::Cleat },
            scope,
            state: LineState::Fast,
            tension,
        }]
    }

    /// You can gather in a slack line hand over hand, but you cannot haul
    /// a loaded one: the rate derates to nothing as the line's own
    /// tension approaches what a person can hold. This is the constraint
    /// that makes springs and the engine the answer to a boat that won't
    /// come alongside, rather than pulling harder.
    #[test]
    fn hauling_stalls_as_the_line_takes_up_the_load() {
        let mut id = 9;
        let mut slack = fast_line(10.0, 0.0);
        crate::line::apply_command(
            &mut slack,
            &mut id,
            LineCommand::Tend { id: 1, rate: 1.0 },
            CrewLimits::DEFAULT,
            &[],
            |_| Vec2::new(10.0, 0.0),
            shore_at,
        );
        let gathered = 10.0 - slack[0].scope;
        assert!((gathered - LINE_HAUL_RATE * PHYSICS_DT).abs() < 1e-6);

        let mut loaded = fast_line(10.0, CrewLimits::DEFAULT.haul_force());
        crate::line::apply_command(
            &mut loaded,
            &mut id,
            LineCommand::Tend { id: 1, rate: 1.0 },
            CrewLimits::DEFAULT,
            &[],
            |_| Vec2::new(10.0, 0.0),
            shore_at,
        );
        assert_eq!(loaded[0].scope, 10.0, "a fully loaded line should not come in");
    }

    /// A line can never be born longer than its own rope. The reach
    /// setting's ceiling is derived from `LINE_SCOPE_MAX` for exactly
    /// this: at the top of the knob, a throw at maximum range still
    /// makes fast within the rope, so the first surge order cannot clamp
    /// the scope back and snatch metres out of it (CodeRabbit review,
    /// 2026-08-21).
    #[test]
    fn a_line_is_never_made_fast_longer_than_the_rope() {
        let mut lines = Vec::new();
        let mut id = 0;
        let crew = CrewLimits { reach: LINE_REACH_MAX, ..CrewLimits::DEFAULT };
        // A throw at the very limit of the longest reach the knob allows.
        let far = |_: Anchor| Vec2::new(crew.clamped().reach, 0.0);
        crate::line::apply_command(
            &mut lines,
            &mut id,
            LineCommand::MakeFast {
                fairlead: Fairlead::PortBow,
                anchor: Anchor::Shore { pos: Vec2::ZERO, kind: ShoreKind::Cleat },
            },
            crew,
            &[],
            |_| Vec2::ZERO,
            far,
        );
        let scope = lines[0].scope;
        assert!(
            scope <= LINE_SCOPE_MAX,
            "made fast at {scope} m on a {LINE_SCOPE_MAX} m rope"
        );
        // ...and surging out therefore cannot shorten it.
        let line_id = lines[0].id;
        lines[0].state = LineState::Fast;
        crate::line::apply_command(
            &mut lines,
            &mut id,
            LineCommand::Tend { id: line_id, rate: -1.0 },
            crew,
            &[],
            |_| Vec2::ZERO,
            far,
        );
        assert!(
            lines[0].scope >= scope,
            "a pay-out order shortened the line from {scope} m to {}",
            lines[0].scope
        );
    }

    /// How hard the crew can pull is a SETTING, not a constant of the
    /// world: the same tension that stops a weak crew dead is nothing to
    /// a strong one. Ten kilos is the default — a rope, not a winch.
    #[test]
    fn what_the_crew_can_pull_is_a_setting() {
        // A load a 10 kg crew cannot move at all, being their whole limit.
        let load = CrewLimits::DEFAULT.haul_force();
        let gathered = |haul_kg: f32| -> f32 {
            let mut lines = fast_line(10.0, load);
            let mut id = 9;
            crate::line::apply_command(
                &mut lines,
                &mut id,
                LineCommand::Tend { id: 1, rate: 1.0 },
                CrewLimits { haul_kg, ..CrewLimits::DEFAULT },
                &[],
                |_| Vec2::new(10.0, 0.0),
                shore_at,
            );
            10.0 - lines[0].scope
        };
        assert_eq!(gathered(LINE_HAUL_KG), 0.0, "their own limit stops the default crew dead");
        let strong = gathered(LINE_HAUL_KG * 4.0);
        assert!(strong > 0.0, "four times the crew should still be gathering, got {strong} m");
        assert!(
            strong < LINE_HAUL_RATE * PHYSICS_DT,
            "but still derated by the load, not hauling free"
        );
    }

    /// Surging out is not force-limited (a turn round a cleat, let it
    /// run) but it does stop at the end of the rope.
    #[test]
    fn paying_out_stops_at_the_end_of_the_rope() {
        let mut id = 9;
        let mut lines = fast_line(LINE_SCOPE_MAX - 0.001, LINE_MBL * 0.5);
        for _ in 0..10 {
            crate::line::apply_command(
                &mut lines,
                &mut id,
                LineCommand::Tend { id: 1, rate: -1.0 },
                CrewLimits::DEFAULT,
                &[],
                |_| Vec2::new(10.0, 0.0),
                shore_at,
            );
        }
        assert_eq!(lines[0].scope, LINE_SCOPE_MAX);
    }

    #[test]
    fn a_line_out_of_reach_is_never_passed() {
        let mut lines = Vec::new();
        let mut id = 0;
        let cmd = LineCommand::MakeFast {
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Shore { pos: Vec2::new(LINE_REACH + 1.0, 0.0),
                kind: ShoreKind::Pole,
            },
        };
        crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        assert!(lines.is_empty());
        assert_eq!(id, 0, "a refused order must not burn an id");
    }

    /// Ids are monotonic and never recycled: casting off line 0 and
    /// passing another must not hand the new one the old id, or a Tend
    /// order already in flight would silently land on the wrong rope.
    #[test]
    fn line_ids_are_never_recycled() {
        let mut lines = Vec::new();
        let mut id = 0;
        let cmd = LineCommand::MakeFast {
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Shore { pos: Vec2::new(3.0, 0.0), kind: ShoreKind::Cleat },
        };
        crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        let first = lines[0].id;
        crate::line::apply_command(
            &mut lines,
            &mut id,
            LineCommand::CastOff { id: first },
            CrewLimits::DEFAULT,
            &[],
            |_| Vec2::ZERO,
            shore_at,
        );
        crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        assert_ne!(lines[0].id, first);
    }

    /// A rope from one of her own fairleads to another pulls at only one
    /// end, so hauling it in would drive her along on her own bootstraps.
    /// The UI never offers such an anchor, but a command is input, and
    /// input is where a corrupt recording gets checked (CodeRabbit
    /// review, 2026-08-21).
    #[test]
    fn a_line_cannot_be_made_fast_to_your_own_boat() {
        let mut lines = Vec::new();
        let mut id = 0;
        let cmd = LineCommand::MakeFast {
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Boat { hull: Hull::Player, fairlead: Fairlead::StbdQuarter },
        };
        // Any anchor resolves 5 m off the bow here — near enough to
        // reach, so only the hull check can refuse it.
        let near = |_: Anchor| Vec2::new(3.0, 0.0);
        crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, near);
        assert!(lines.is_empty(), "she cannot haul on herself");
        // A rope to the boat in the next berth is still fine.
        let neighbour = LineCommand::MakeFast {
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Boat { hull: Hull::Moored(0), fairlead: Fairlead::StbdQuarter },
        };
        crate::line::apply_command(
            &mut lines,
            &mut id,
            neighbour,
            CrewLimits::DEFAULT,
            &[],
            |_| Vec2::ZERO,
            near,
        );
        assert_eq!(lines.len(), 1, "a neighbour is a legal place to make fast");
    }

    #[test]
    fn a_fairlead_carries_only_one_rope() {
        let mut lines = Vec::new();
        let mut id = 0;
        let cmd = LineCommand::MakeFast {
            fairlead: Fairlead::PortBow,
            anchor: Anchor::Shore { pos: Vec2::new(3.0, 0.0), kind: ShoreKind::Cleat },
        };
        for _ in 0..4 {
            crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        }
        assert_eq!(lines.len(), 1, "a fumbled second gesture must not double the line");
        // ...and a different handle is still free.
        let other = LineCommand::MakeFast {
            fairlead: Fairlead::StbdBow,
            anchor: Anchor::Shore { pos: Vec2::new(3.0, 0.0), kind: ShoreKind::Cleat },
        };
        crate::line::apply_command(&mut lines, &mut id, other, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn the_boat_carries_a_finite_number_of_lines() {
        let mut lines = Vec::new();
        let mut id = 0;
        // Every fairlead, then some: with one rope per handle the count
        // cap and the number of handles coincide, and both hold.
        for f in Fairlead::ALL.iter().chain(Fairlead::ALL.iter()) {
            let cmd = LineCommand::MakeFast {
                fairlead: *f,
                anchor: Anchor::Shore { pos: Vec2::new(3.0, 0.0), kind: ShoreKind::Cleat },
            };
            crate::line::apply_command(&mut lines, &mut id, cmd, CrewLimits::DEFAULT, &[], |_| Vec2::ZERO, shore_at);
        }
        assert_eq!(lines.len(), Fairlead::ALL.len().min(LINE_COUNT_MAX));
    }
}
