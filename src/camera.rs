//! How far away the 3D cameras stand — the speed-coupled field of view of
//! the chase and cockpit rigs — and the bound that keeps a widening view
//! from ever reading as the camera sliding backwards.
//!
//! # Why a bound is needed at all
//!
//! A perspective camera has a vanishing point (strictly, a focus of
//! expansion: where its direction of travel projects to). Moving forward
//! pushes every static feature AWAY from it — that outward streaming IS
//! the sensation of motion. Widening the FOV pulls every feature TOWARD
//! it. Widen faster than the camera advances and the sum runs the wrong
//! way: the water, jetties and shores drift back toward the horizon while
//! the boat is accelerating hardest, which reads as the camera being
//! dragged backwards. Widening per se is not the problem — widening
//! FASTER THAN THE CAMERA TRAVELS is.
//!
//! # The bound
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
/// rather than making passage, and the camera is close enough to
/// stationary that there is no streaming scenery for a zoom to
/// contradict.
const MANOEUVRING_SPEED: f32 = 0.5;
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

    const DT: f32 = 1.0 / 60.0;
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


