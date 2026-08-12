//! How far away each camera stands, and the bounds that keep a view
//! pulling back from ever reading as the camera sliding backwards:
//! `SpeedZoom` (the top-down view's visible width) and `FovZoom` (the 3D
//! rigs' speed-coupled field of view).
//!
//! The two are the same statement in different projections. The
//! perspective one is documented in full below, after `SpeedZoom`; this
//! header covers the orthographic case, which is the simpler of the two
//! and worth reading first.
//!
//! # Why a bound is needed at all (orthographic)
//!
//! The top-down camera is centred on the boat, so the boat is nailed to
//! the screen centre and the only motion cue is the SCENERY streaming
//! past. A world point `p` sits at screen offset `X = (p − c)·k` (`c` =
//! camera world position, `k` = px per metre). Widening the view shrinks
//! `k`, which pulls every feature TOWARD the centre — and for the
//! features ASTERN that direction is forwards. Widen fast enough and that
//! inward pull beats the backward streaming from the boat's own travel:
//! the shore behind the boat starts creeping forwards, which reads as the
//! camera sliding backwards, exactly while the boat is accelerating
//! hardest.
//!
//! # The bound (orthographic)
//!
//! With `W` the visible width in metres, `ρ = ½·√(1+(sh/sw)²)` (the
//! screen's half-diagonal measured in units of `W`) and `δ` the camera's
//! world travel over a frame, no visible static feature can move forwards
//! iff
//!
//! ```text
//!     ρ·|ΔW| ≤ δ
//! ```
//!
//! i.e. **the visible width may change by at most `1/ρ ≈ 1.7` metres per
//! metre the camera travels**, whatever the frame rate, speed or
//! acceleration — the bound contains no `W`, so it holds identically at
//! any user zoom level. `SpeedZoom::update` enforces half of that (see
//! `FLOW_MARGIN`), which additionally keeps the guarantee out to twice
//! the visible radius, so nothing can drift forwards INTO frame from
//! behind either.
//!
//! # Why a bound is needed at all (perspective)
//!
//! Same artifact, different projection. A perspective camera has a
//! vanishing point (strictly, a focus of
//! expansion: where its direction of travel projects to). Moving forward
//! pushes every static feature AWAY from it — that outward streaming IS
//! the sensation of motion. Widening the FOV pulls every feature TOWARD
//! it. Widen faster than the camera advances and the sum runs the wrong
//! way: the water, jetties and shores drift back toward the horizon while
//! the boat is accelerating hardest, which reads as the camera being
//! dragged backwards. Widening per se is not the problem — widening
//! FASTER THAN THE CAMERA TRAVELS is.
//!
//! # The bound (perspective)
//!
//! Write `f = cot(φ/2)` for the projection scale of a vertical FOV `φ`
//! (a point at camera-space `(X, Y, Z)` lands at screen `f·(X, Y)/Z`),
//! and let the camera advance `d` along its own view axis over a frame.
//! A static point's screen vector is then scaled by
//!
//! ```text
//!     λ = (f'/f) · Z/(Z − d)
//! ```
//!
//! — the SAME λ for every point at depth `Z` — so the flow keeps the sign
//! the camera's own motion gives it (outward advancing, inward backing),
//! for everything nearer than `Z`, exactly while
//!
//! ```text
//!     f' ≥ f·(1 − d/Z)      advancing; the inequality flips backing
//! ```
//!
//! In FOV-angle terms (`d(ln f)/dφ = −1/sin φ`) that is
//! `Δφ ≤ sin(φ)·d/Z`: **the FOV may open by at most `sin φ` radians per
//! `Z` metres the camera advances**, whatever the frame rate, speed or
//! acceleration. Full statement and proof: `docs/camera-speed-zoom.md`,
//! pinned by the tests at the bottom of this file.
//!
//! Two things are worth knowing before touching this:
//!
//! * **Only one direction ever binds.** Advancing, narrowing the FOV
//!   raises `f` and reinforces the outward flow — it cannot reverse
//!   anything, so only WIDENING is bounded. Making sternway it is the
//!   other way round.
//! * **No bound protects every depth.** `d/Z → 0` as `Z → ∞` while the
//!   zoom term stays finite: the far field HAS to drift inward when you
//!   widen — that is what widening means. So the guarantee comes with an
//!   explicit reach, `FLOW_REACH_M`.
//!
//! And note what the bound is NOT: it is not a smoothing time constant.
//! A lag filter still widens arbitrarily fast when the target jumps far
//! enough, which is why the hard cap, not the low-pass beside it, is what
//! carries the proof.

use macroquad::prelude::*;

/// Widest the speed zoom is allowed to pull back: `1.35 ×` the view the
/// user has selected (default 150 m → ~202 m at full speed). The span is
/// deliberately modest, because the bound below prices it in TRAVEL: at
/// the default view this span costs ~60 m of it in either direction, and
/// a wider span would leave the view pulled back well into a slow
/// approach (there is no way to buy it back faster without producing the
/// very artifact this module exists to rule out).
pub const SPEED_ZOOM_MAX: f32 = 1.35;
/// Below `MANOEUVRING_SPEED` the view is the close-up: manoeuvring wants
/// detail, not reach — the same threshold the FOV bound's idle carve-out
/// uses, and for the same reason.
pub use self::MANOEUVRING_SPEED as SPEED_ZOOM_LO;
/// At/above this SOG (m/s, ~5.4 kn — near this hull's measured top speed,
/// see docs/reference-boats.md) the view is fully pulled back.
pub const SPEED_ZOOM_HI: f32 = 2.8;

/// Travel (metres) over which the zoom e-folds toward its target. This is
/// the "feel" knob — momentum/slowness — and is deliberately measured in
/// DISTANCE, not seconds: the zoom is a function of how far the boat has
/// come, so it can't outrun the travel that pays for it. Short enough
/// that the bound governs the bulk of a transition (so the pull-back is
/// as prompt as it is allowed to be) and the low-pass only rounds off the
/// last of it, rather than arriving at the target on a corner.
const SETTLE_M: f32 = 25.0;

/// Fraction of the no-reversal bound the zoom is allowed to spend. At
/// `0.5` every visible feature keeps at least HALF the backward screen
/// flow it would have at a fixed zoom, and the no-forward-motion
/// guarantee extends to twice the visible radius (see the module docs).
const FLOW_MARGIN: f32 = 0.5;

/// While the boat is stopped there is no travel to spend, so the bound
/// freezes the zoom. This lets the view creep back IN (never out) at up
/// to this much multiplier per second when the boat is at rest — a static
/// camera has no streaming scenery to contradict, so there is no backward
/// illusion to protect against. Fades to zero at `SPEED_ZOOM_LO`, above
/// which the bound is the only thing in force.
const IDLE_RELAX: f32 = 0.12;

/// The speed-adaptive part of the camera scale: a multiplier on the
/// visible width, `1.0` (close-up) at manoeuvring speed up to
/// `SPEED_ZOOM_MAX` under way.
pub struct SpeedZoom {
    mult: f32,
    prev_cam: Option<Vec2>,
}

impl SpeedZoom {
    pub fn new() -> Self {
        Self { mult: 1.0, prev_cam: None }
    }

    /// The multiplier to divide the camera scale by this frame.
    pub fn mult(&self) -> f32 {
        self.mult
    }

    /// Back to the close-up, and forget where the camera was. Used on
    /// respawn: the camera teleports with the boat, and that jump is not
    /// travel the zoom is allowed to spend.
    pub fn reset(&mut self) {
        self.mult = 1.0;
        self.prev_cam = None;
    }

    /// Advance one frame and return the multiplier to render with.
    ///
    /// `cam` is THIS frame's camera world position (so the travel below is
    /// the real, world-clamped, pan-included camera displacement — the
    /// same `Δc` the proof is stated in, not an estimate); `screen` is the
    /// window size in px; `unit_width` is the visible width in metres at
    /// multiplier 1 (i.e. after the user's own zoom); `max_width` caps the
    /// total so the speed zoom can't push the view past the user zoom's
    /// own outer limit.
    pub fn update(
        &mut self,
        cam: Vec2,
        screen: Vec2,
        unit_width: f32,
        max_width: f32,
        sog: f32,
        dt: f32,
    ) -> f32 {
        let travel = match self.prev_cam {
            Some(prev) => (cam - prev).length(),
            None => 0.0,
        };
        self.prev_cam = Some(cam);

        let ramp = ((sog - SPEED_ZOOM_LO) / (SPEED_ZOOM_HI - SPEED_ZOOM_LO)).clamp(0.0, 1.0);
        let target = (1.0 + (SPEED_ZOOM_MAX - 1.0) * ramp).min((max_width / unit_width).max(1.0));
        let toward = target - self.mult;
        if toward == 0.0 {
            return self.mult;
        }

        // Feel: e-fold toward the target over SETTLE_M metres of travel.
        let smooth = toward.abs() * (1.0 - (-travel / SETTLE_M).exp());
        // Idle relaxation, narrowing only (see IDLE_RELAX).
        let idle = if toward < 0.0 {
            IDLE_RELAX * dt * ((SPEED_ZOOM_LO - sog) / SPEED_ZOOM_LO).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // The guarantee: ρ·|ΔW| ≤ FLOW_MARGIN·δ, with ΔW = unit_width·Δmult.
        let rho = 0.5 * (1.0 + (screen.y / screen.x).powi(2)).sqrt();
        let bound = FLOW_MARGIN * travel / (rho * unit_width) + idle;

        let step = smooth.max(idle).min(bound).min(toward.abs());
        self.mult += step * toward.signum();
        self.mult
    }
}

// Beyond the reach the residual inward drift of a point at screen radius
// `|s|` is `|s|·ε·(1 − FLOW_REACH_M/Z)`, with `ε = 1 − f'/f` the relative
// widening this frame: zero at the reach, rising smoothly to the full
// zoom rate at infinity.

/// Depth (metres) out to which the no-reversal guarantee holds. Chosen
/// as the near/mid field that carries the motion cue in this marina —
/// the channel is 125 m across, so this covers the water, the jetties,
/// the moored fleet and both shores; beyond it lie the far shore and the
/// horizon, whose own flow is already a fraction of a pixel per frame.
/// Raising it makes the FOV ramp cost proportionally more travel.
const FLOW_REACH_M: f32 = 150.0;

/// Degrees of extra FOV at `FOV_SPEED_REF`, and the cap on that ramp.
/// Speed-coupled FOV is the standard way to make a straight run READ as
/// motion — the edges of frame stretch as you gather way — and on open
/// water it is what carries the sense of speed.
const FOV_SPEED_GAIN_DEG: f32 = 5.0;
const FOV_SPEED_REF: f32 = 3.0;
const FOV_GAIN_MAX: f32 = 1.5;
/// Easing time constant (s) toward the speed target: it must read as
/// gathering way, not as tracking the throttle lever. Feel only — as in
/// the orthographic case, the bound above is what carries the proof.
const FOV_TAU: f32 = 1.2;
/// Below this speed over ground (m/s, ~1 kn) the boat is manoeuvring
/// rather than making passage: the top-down view stays at its close-up
/// (`SPEED_ZOOM_LO`), and the camera is close enough to stationary that
/// there is no streaming scenery for a zoom to contradict.
pub const MANOEUVRING_SPEED: f32 = 0.5;
/// Degrees per second the FOV may drift back toward its base while the
/// boat is stopped and the bound would otherwise freeze it — the one
/// carve-out, so a crash stop can't strand the view wide. Faded out
/// entirely by `MANOEUVRING_SPEED`, i.e. it never overlaps the regime
/// the artifact lives in.
const IDLE_FOV_RELAX: f32 = 3.0;
/// Sanity rails on the FOV, well outside anything the rigs ask for. They
/// stand in for "unbounded" on whichever side the flow bound leaves free.
const FOV_HARD_MIN: f32 = 5.0;
const FOV_HARD_MAX: f32 = 120.0;

fn cot_half(fov_deg: f32) -> f32 {
    1.0 / (fov_deg.to_radians() * 0.5).tan()
}

fn fov_from_cot(f: f32) -> f32 {
    2.0 * (1.0 / f).atan().to_degrees()
}

/// The speed-coupled vertical FOV of a perspective camera, rate-bounded
/// so the scenery it looks at can never flow the wrong way.
pub struct FovZoom {
    fov_deg: f32,
    /// The eased BASE FOV (the rig's own, chase vs cockpit). Tracked
    /// separately because a change here means a view-mode switch, which
    /// teleports the camera — see `update`.
    base_deg: f32,
    prev_eye: Option<Vec3>,
}

impl FovZoom {
    pub fn new(base_deg: f32) -> Self {
        Self { fov_deg: base_deg, base_deg, prev_eye: None }
    }

    /// The FOV last rendered with. Test-only: `update` hands the render
    /// path its value directly, so nothing else needs to read it back.
    #[cfg(test)]
    pub fn fov_deg(&self) -> f32 {
        self.fov_deg
    }

    /// Re-seat the rig (respawn): back to the base FOV, and forget where
    /// the eye was so the teleport buys no zoom.
    pub fn snap(&mut self, base_deg: f32) {
        self.fov_deg = base_deg;
        self.base_deg = base_deg;
        self.prev_eye = None;
    }

    /// Advance one frame and return the vertical FOV (degrees) to render
    /// with. `eye`/`fwd` are THIS frame's camera position and view
    /// direction — the realized ones, so the advance below is the real
    /// thing (trailing lag, band clamp and bob included) rather than the
    /// boat's velocity standing in for it. `base_deg` is the rig's own
    /// base FOV and `speed` the boat's speed over ground.
    pub fn update(&mut self, eye: Vec3, fwd: Vec3, base_deg: f32, speed: f32, dt: f32) -> f32 {
        let advance = match self.prev_eye {
            Some(prev) => (eye - prev).dot(fwd.normalize_or_zero()),
            None => 0.0,
        };
        self.prev_eye = Some(eye);
        let ease = 1.0 - (-dt / FOV_TAU).exp();

        // The BASE change is exempt from the bound: it moves only on a
        // view-mode switch, and that cuts the camera to a different place
        // entirely — there is no continuous optical flow across the cut
        // for the bound to protect, exactly as the user's own pinch is
        // exempt in the orthographic case.
        let base_step = (base_deg - self.base_deg) * ease;
        self.base_deg += base_step;
        let anchor = self.fov_deg + base_step;

        // The speed-driven part, which IS bounded.
        let want =
            self.base_deg + FOV_SPEED_GAIN_DEG * (speed / FOV_SPEED_REF).min(FOV_GAIN_MAX);
        let eased = anchor + (want - anchor) * ease;

        // The bound, `f' ≥ f·(1 − d/Z)` — one expression, but which SIDE
        // it binds depends on the direction of travel. Advancing, the
        // flow expands and only widening (which shrinks f, since
        // f = cot(φ/2)) can reverse it; making sternway the flow
        // contracts and only narrowing can. The other side is free: it
        // reinforces the flow rather than fighting it. At `d = 0` the two
        // meet and the FOV is frozen.
        let f = cot_half(anchor);
        let limit = fov_from_cot(f * (1.0 - advance / FLOW_REACH_M).max(0.5));
        let (mut narrowest, mut widest) =
            if advance >= 0.0 { (FOV_HARD_MIN, limit) } else { (limit, FOV_HARD_MAX) };
        // Idle relaxation, toward the base FOV only.
        let idle =
            IDLE_FOV_RELAX * dt * ((MANOEUVRING_SPEED - speed) / MANOEUVRING_SPEED).clamp(0.0, 1.0);
        if self.base_deg < anchor {
            narrowest = narrowest.min((anchor - idle).max(self.base_deg));
        } else {
            widest = widest.max((anchor + idle).min(self.base_deg));
        }

        self.fov_deg = eased.clamp(narrowest, widest);
        self.fov_deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SW: f32 = 1280.0;
    const SH: f32 = 720.0;
    const UNIT_W: f32 = 150.0;
    const MAX_W: f32 = 450.0;
    const DT: f32 = 1.0 / 60.0;

    fn rho() -> f32 {
        0.5 * (1.0 + (SH / SW).powi(2)).sqrt()
    }

    /// One frame of the real main-loop ordering: the camera moves, the
    /// zoom updates against THAT position, and the pair (camera, width) is
    /// what gets rendered. Returns the worst-case forward screen
    /// displacement, in px, over every static world point within `reach`
    /// visible radii of the camera — negative means everything on screen
    /// streamed backwards, as it must.
    fn step_and_worst_forward_px(
        z: &mut SpeedZoom,
        cam_prev: Vec2,
        cam: Vec2,
        sog: f32,
        reach: f32,
    ) -> f32 {
        let m_prev = z.mult();
        let m = z.update(cam, vec2(SW, SH), UNIT_W, MAX_W, sog, DT);
        let (w_prev, w) = (UNIT_W * m_prev, UNIT_W * m);
        let (k_prev, k) = (SW / w_prev, SW / w);
        let travel = (cam - cam_prev).length();
        // Half-diagonal of the previous frame's view, in metres.
        let r_prev = 0.5 * (SW * SW + SH * SH).sqrt() / k_prev;
        // Δx·û = ξ·Δk − δ·k for a static point ξ metres ahead of the
        // camera (negative = astern); linear in ξ, so the extremes bound
        // every point in between.
        let dk = k - k_prev;
        let ahead = reach * r_prev * dk - travel * k;
        let astern = -reach * r_prev * dk - travel * k;
        ahead.max(astern)
    }

    /// A straight-line run under a scripted speed profile, asserting every
    /// frame that nothing on screen (indeed nothing within `reach` visible
    /// radii) ever moves forwards.
    fn run_profile(speeds: impl Iterator<Item = f32>, reach: f32) -> SpeedZoom {
        let mut z = SpeedZoom::new();
        let mut cam = Vec2::ZERO;
        for sog in speeds {
            let prev = cam;
            cam += vec2(sog * DT, 0.0);
            let worst = step_and_worst_forward_px(&mut z, prev, cam, sog, reach);
            assert!(
                worst <= 1e-3,
                "a static feature moved {worst:.4} px FORWARDS at sog {sog:.2} m/s",
            );
        }
        z
    }

    /// The headline property: however violently the boat accelerates, the
    /// scenery never reverses. Swept over accelerations from a gentle
    /// spool-up to a physically impossible slam, since the whole point is
    /// that the bound doesn't depend on the acceleration.
    #[test]
    fn no_acceleration_can_make_the_scenery_run_forwards() {
        for accel in [0.05f32, 0.2, 0.5, 1.0, 5.0, 50.0] {
            let mut v = 0.0f32;
            run_profile(
                (0..900).map(|_| {
                    v = (v + accel * DT).min(3.0);
                    v
                }),
                1.0,
            );
        }
    }

    /// FLOW_MARGIN = 0.5 buys the same guarantee out to TWICE the visible
    /// radius, so no feature can drift forwards into frame from off-screen
    /// astern either.
    #[test]
    fn the_guarantee_extends_to_twice_the_visible_radius() {
        let mut v = 0.0f32;
        run_profile(
            (0..900).map(|_| {
                v = (v + 1.0 * DT).min(3.0);
                v
            }),
            2.0,
        );
    }

    /// Same property on the way down — a crash stop zooms back IN, which
    /// is the mirror image (features AHEAD would be the ones to run
    /// forwards). Held above SPEED_ZOOM_LO, where the bound is the only
    /// thing in force (below it the idle relaxation takes over, by design
    /// — see IDLE_RELAX).
    #[test]
    fn decelerating_never_pushes_the_scenery_forwards() {
        let mut v = 3.0f32;
        run_profile(
            (0..900).map(|_| {
                v = (v - 2.0 * DT).max(SPEED_ZOOM_LO);
                v
            }),
            2.0,
        );
    }

    /// The bound is a cap, not the behaviour: given enough travel the zoom
    /// still reaches its target, and comes back when the boat slows.
    #[test]
    fn the_zoom_still_reaches_its_target_and_returns() {
        let z = run_profile((0..3600).map(|_| 3.0), 1.0);
        assert!(
            z.mult() > SPEED_ZOOM_MAX - 0.02,
            "held at full speed the view should end up pulled back, got {}",
            z.mult()
        );
        // Ease down to a slow manoeuvring speed and hold it there. The
        // target at 1 m/s is a hair off the close-up; getting back to it
        // is bounded by travel, so this run is long by construction.
        let slow_target = 1.0
            + (SPEED_ZOOM_MAX - 1.0) * (1.0 - SPEED_ZOOM_LO) / (SPEED_ZOOM_HI - SPEED_ZOOM_LO);
        let mut z = z;
        let mut cam = vec2(3.0 * DT * 3600.0, 0.0);
        for _ in 0..(60 * 200) {
            let prev = cam;
            cam += vec2(1.0 * DT, 0.0);
            let worst = step_and_worst_forward_px(&mut z, prev, cam, 1.0, 1.0);
            assert!(worst <= 1e-3, "reverse flow while easing back in: {worst:.4} px");
        }
        assert!(
            (z.mult() - slow_target).abs() < 0.02,
            "back at manoeuvring speed the view should sit at {slow_target}, got {}",
            z.mult()
        );
    }

    /// A boat that isn't moving has no travel to spend, so the zoom cannot
    /// widen at all — this is the degenerate case of the bound, and the
    /// reason a stopped boat never sees the view creep outwards.
    #[test]
    fn a_stationary_camera_can_never_widen() {
        let mut z = SpeedZoom::new();
        for _ in 0..600 {
            // Full speed reported, but the camera pinned (world-clamped).
            z.update(Vec2::ZERO, vec2(SW, SH), UNIT_W, MAX_W, 3.0, DT);
        }
        assert_eq!(z.mult(), 1.0);
    }

    /// ...but a boat stopped WIDE does creep back to the close-up, so a
    /// crash stop doesn't strand the camera pulled back.
    #[test]
    fn a_stopped_boat_relaxes_back_to_the_close_up() {
        let mut z = run_profile((0..3600).map(|_| 3.0), 1.0);
        assert!(z.mult() > SPEED_ZOOM_MAX - 0.02);
        let cam = vec2(3.0 * DT * 3600.0, 0.0);
        for _ in 0..(60 * 20) {
            z.update(cam, vec2(SW, SH), UNIT_W, MAX_W, 0.0, DT);
        }
        assert!(z.mult() < 1.01, "expected the view back at the close-up, got {}", z.mult());
    }

    /// The per-frame cap is exactly the bound in the proof, expressed in
    /// its own terms: metres of visible width per metre of camera travel.
    #[test]
    fn the_width_change_never_exceeds_the_travel_bound() {
        let mut z = SpeedZoom::new();
        let mut cam = Vec2::ZERO;
        let mut v = 0.0f32;
        for _ in 0..1800 {
            v = (v + 1.5 * DT).min(2.8);
            let prev_w = UNIT_W * z.mult();
            let prev_cam = cam;
            cam += vec2(v * DT, 0.0);
            let w = UNIT_W * z.update(cam, vec2(SW, SH), UNIT_W, MAX_W, v, DT);
            let travel = (cam - prev_cam).length();
            assert!(
                rho() * (w - prev_w).abs() <= travel + 1e-4,
                "ρ·ΔW = {} exceeded the travel {travel}",
                rho() * (w - prev_w).abs()
            );
        }
    }

    /// The user's own zoom sets the scale the multiplier rides on, so the
    /// same speed lands at the same MULTIPLE of whatever they picked.
    #[test]
    fn the_speed_zoom_is_relative_to_the_user_zoom() {
        let mut close = SpeedZoom::new();
        let mut wide = SpeedZoom::new();
        let mut cam = Vec2::ZERO;
        for _ in 0..7200 {
            cam += vec2(3.0 * DT, 0.0);
            close.update(cam, vec2(SW, SH), 40.0, MAX_W, 3.0, DT);
            wide.update(cam, vec2(SW, SH), 150.0, MAX_W, 3.0, DT);
        }
        assert!((close.mult() - wide.mult()).abs() < 0.01);
    }

    /// ...but it never pushes past the user zoom's own outer limit.
    #[test]
    fn the_speed_zoom_respects_the_outer_zoom_limit() {
        let mut z = SpeedZoom::new();
        let mut cam = Vec2::ZERO;
        for _ in 0..7200 {
            cam += vec2(3.0 * DT, 0.0);
            z.update(cam, vec2(SW, SH), 400.0, MAX_W, 3.0, DT);
        }
        assert!(400.0 * z.mult() <= MAX_W + 1e-3);
    }

    /// Respawn teleports the camera; that jump is not travel, and must not
    /// buy the zoom a step it hasn't earned.
    #[test]
    fn a_respawn_teleport_does_not_pay_for_a_zoom_step() {
        let mut z = run_profile((0..3600).map(|_| 3.0), 1.0);
        z.reset();
        assert_eq!(z.mult(), 1.0);
        // First frame after the teleport: no travel is credited.
        let m = z.update(vec2(1000.0, 1000.0), vec2(SW, SH), UNIT_W, MAX_W, 3.0, DT);
        assert_eq!(m, 1.0);
    }

    // --- Perspective (FovZoom) ---------------------------------------

    const CHASE_BASE: f32 = 45.0;
    const COCKPIT_BASE: f32 = 58.0;

    /// λ = (f'/f)·Z/(Z−d): the factor a static point's screen vector is
    /// scaled by, at depth `z` under a camera advance of `d`. λ ≥ 1 is
    /// outward flow (correct while advancing); λ < 1 means the scenery
    /// drifted back toward the vanishing point.
    fn lambda(fov_prev: f32, fov: f32, d: f32, z: f32) -> f32 {
        (cot_half(fov) / cot_half(fov_prev)) * z / (z - d)
    }

    /// One frame of the real render3d ordering: the camera moves to `eye`,
    /// then the FOV is chosen against that frame's advance.
    fn fov_step(zoom: &mut FovZoom, eye: Vec3, base: f32, speed: f32) -> (f32, f32) {
        let prev = zoom.fov_deg();
        (prev, zoom.update(eye, Vec3::Z, base, speed, DT))
    }

    /// The headline property, perspective edition: however hard the boat
    /// accelerates, nothing within the guaranteed reach ever drifts back
    /// toward the vanishing point.
    #[test]
    fn no_acceleration_can_make_the_scenery_drift_toward_the_horizon() {
        for accel in [0.05f32, 0.3, 1.0, 5.0, 50.0] {
            for base in [CHASE_BASE, COCKPIT_BASE] {
                let mut z = FovZoom::new(base);
                let (mut eye, mut v) = (Vec3::ZERO, 0.0f32);
                for _ in 0..1200 {
                    v = (v + accel * DT).min(4.5);
                    let d = v * DT;
                    eye += vec3(0.0, 0.0, d);
                    let (prev, fov) = fov_step(&mut z, eye, base, v);
                    for depth in [5.0, 25.0, 60.0, FLOW_REACH_M] {
                        let l = lambda(prev, fov, d, depth);
                        assert!(
                            l >= 1.0 - 1e-6,
                            "λ={l:.6} at {depth} m (accel {accel}, base {base}): the \
                             scenery ran backwards",
                        );
                    }
                }
            }
        }
    }

    /// A camera that isn't advancing has bought no widening — the
    /// degenerate case of the bound, and what keeps a boat pinned against
    /// a jetty from having its view creep outwards.
    #[test]
    fn a_camera_that_is_not_advancing_cannot_widen() {
        let mut z = FovZoom::new(CHASE_BASE);
        for _ in 0..600 {
            // Full speed reported, but the eye held still.
            z.update(Vec3::ZERO, Vec3::Z, CHASE_BASE, 3.0, DT);
        }
        assert!(z.fov_deg() <= CHASE_BASE + 1e-4);
    }

    /// Making sternway flips which side binds: the flow contracts (that's
    /// what backing looks like), so NARROWING is what could reverse it.
    #[test]
    fn sternway_bounds_narrowing_instead_of_widening() {
        let mut z = FovZoom::new(CHASE_BASE);
        // Wind the FOV out first with a fast forward run...
        let mut eye = Vec3::ZERO;
        for _ in 0..2400 {
            eye += vec3(0.0, 0.0, 3.0 * DT);
            z.update(eye, Vec3::Z, CHASE_BASE, 3.0, DT);
        }
        assert!(z.fov_deg() > CHASE_BASE + 3.0);
        // ...then back away from the scene with the engine astern.
        for _ in 0..1200 {
            let d = -1.5 * DT;
            eye += vec3(0.0, 0.0, d);
            let (prev, fov) = fov_step(&mut z, eye, CHASE_BASE, 1.5);
            for depth in [5.0, 60.0, FLOW_REACH_M] {
                let l = lambda(prev, fov, d, depth);
                assert!(l <= 1.0 + 1e-6, "λ={l:.6} at {depth} m: the scenery expanded while backing");
            }
        }
    }

    /// The bound is a cap, not the behaviour: given travel, the FOV still
    /// reaches its speed target, and eases back when the boat slows.
    #[test]
    fn the_fov_reaches_its_speed_target_and_returns() {
        let mut z = FovZoom::new(CHASE_BASE);
        let mut eye = Vec3::ZERO;
        for _ in 0..1800 {
            eye += vec3(0.0, 0.0, 3.0 * DT);
            z.update(eye, Vec3::Z, CHASE_BASE, 3.0, DT);
        }
        assert!(
            (z.fov_deg() - (CHASE_BASE + FOV_SPEED_GAIN_DEG)).abs() < 0.1,
            "expected the full speed gain, got {}",
            z.fov_deg()
        );
        // Slowing down: narrowing is free while still advancing, so the
        // view comes back on the easing time constant alone.
        for _ in 0..600 {
            eye += vec3(0.0, 0.0, 0.6 * DT);
            z.update(eye, Vec3::Z, CHASE_BASE, 0.6, DT);
        }
        assert!(z.fov_deg() < CHASE_BASE + 1.5, "expected the view back near base, got {}", z.fov_deg());
    }

    /// Widening is paid for in travel, and this is the price: the full
    /// speed gain costs `ε·FLOW_REACH_M` metres of it, ~19 m here.
    #[test]
    fn the_full_speed_gain_costs_the_travel_the_bound_prices_it_at() {
        let mut z = FovZoom::new(CHASE_BASE);
        let mut eye = Vec3::ZERO;
        let mut travel = 0.0;
        while z.fov_deg() < CHASE_BASE + FOV_SPEED_GAIN_DEG - 0.05 {
            eye += vec3(0.0, 0.0, 3.0 * DT);
            travel += 3.0 * DT;
            z.update(eye, Vec3::Z, CHASE_BASE, 3.0, DT);
            assert!(travel < 200.0, "the FOV never got there");
        }
        let eps = 1.0 - cot_half(CHASE_BASE + FOV_SPEED_GAIN_DEG) / cot_half(CHASE_BASE);
        assert!(
            travel > eps * FLOW_REACH_M,
            "reached the target in {travel} m, under the bound's own price"
        );
        assert!(travel < 3.0 * eps * FLOW_REACH_M, "took far longer than the bound requires: {travel} m");
    }

    /// A stopped boat still settles back to its base FOV (see
    /// IDLE_FOV_RELAX) — a crash stop must not strand the view wide.
    #[test]
    fn a_stopped_boat_settles_back_to_the_base_fov() {
        let mut z = FovZoom::new(CHASE_BASE);
        let mut eye = Vec3::ZERO;
        for _ in 0..1800 {
            eye += vec3(0.0, 0.0, 3.0 * DT);
            z.update(eye, Vec3::Z, CHASE_BASE, 3.0, DT);
        }
        for _ in 0..(60 * 10) {
            z.update(eye, Vec3::Z, CHASE_BASE, 0.0, DT);
        }
        assert!((z.fov_deg() - CHASE_BASE).abs() < 0.05, "got {}", z.fov_deg());
    }

    /// A view-mode switch (chase ↔ cockpit) teleports the camera, so its
    /// base-FOV glide is exempt and completes without travel.
    #[test]
    fn a_view_mode_switch_glides_the_base_without_needing_travel() {
        let mut z = FovZoom::new(CHASE_BASE);
        for _ in 0..(60 * 10) {
            z.update(Vec3::ZERO, Vec3::Z, COCKPIT_BASE, 0.0, DT);
        }
        assert!((z.fov_deg() - COCKPIT_BASE).abs() < 0.05, "got {}", z.fov_deg());
    }

    /// Respawn re-seats the rig; the teleport buys no FOV either.
    #[test]
    fn a_respawn_teleport_does_not_pay_for_fov() {
        let mut z = FovZoom::new(CHASE_BASE);
        let mut eye = Vec3::ZERO;
        for _ in 0..1800 {
            eye += vec3(0.0, 0.0, 3.0 * DT);
            z.update(eye, Vec3::Z, CHASE_BASE, 3.0, DT);
        }
        z.snap(CHASE_BASE);
        assert_eq!(z.fov_deg(), CHASE_BASE);
        let fov = z.update(vec3(0.0, 0.0, 900.0), Vec3::Z, CHASE_BASE, 3.0, DT);
        assert!((fov - CHASE_BASE).abs() < 1e-3, "got {fov}");
    }
}


