//! Cosmetic wake: the churned water a boat leaves behind it.
//!
//! Render-only, like the ripples and the prop-wash streaks — it READS sim
//! state and feeds nothing back (the Pegasus rule: nothing outside
//! `Sim::tick` may touch physics). What it draws is the momentum the hull
//! has shed into the water, which is exactly the quantity `sim.rs` already
//! integrates for its cross-flow drag, so the picture and the forces come
//! from the same place:
//!
//! * **Sideways motion dominates.** A hull sliding sideways is a bluff
//!   body: the flow separates off the keel and hull ends and rolls up into
//!   a counter-rotating vortex pair. That is why a slip through a turn or
//!   a beam drift leaves a far wider, more violent trail than the same
//!   boat running straight. Each station's shed strength is taken from the
//!   local lateral flow `V(x) = v_lat + w·(x − com)` — the same strip
//!   quantity `tick` uses — squared, since shed momentum flux goes as V².
//! * **Going straight still stirs.** The turbulent boundary layer leaves
//!   along both quarters; weaker (`AXIAL_WEIGHT`) and shed alternately to
//!   either side rather than to one.
//! * **The prop race churns.** Momentum flux ∝ thrust ∝ `engine²`. The
//!   existing streaks in `main.rs` are the LIVE slipstream leaving the
//!   blade; these are what it leaves lying on the water behind.
//!
//! And because this is water, not ground: every eddy is advected by the
//! current for its whole life (plus the slug of water the hull dragged
//! along with it, decaying over `CARRY_TAU`), so a wake laid across a
//! stream visibly sets down-current while it fades. A boat drifting WITH
//! the current at current speed disturbs nothing at all — the strengths
//! are computed from water-relative motion throughout.

use harbour_sim_core::boat::BoatDesign;
use harbour_sim_core::sim::{Env, HULL_PTS, Sim, hull_com_x, prop_station, waterline_extent};
use macroquad::prelude::*;

/// Hull stations sampled for shed turbulence, spread over the design's
/// waterline. Five is enough for the band to read as continuous once the
/// eddies have grown; the prop is a sixth, separate source.
const STATIONS: usize = 5;
/// Eddies alive at once. The oldest is dropped when the budget is full,
/// so a hard-driven boat trades tail length for a denser fresh wake.
const EDDY_CAP: usize = 260;
/// How long an eddy stays on the water (s). At cruising speed that is a
/// trail some forty metres long behind the boat.
const EDDY_LIFE: f32 = 14.0;
/// Spawn rate per source at full strength (eddies/s).
const SPAWN_RATE: f32 = 7.0;
/// Fade-in (s) — stops eddies popping into existence alongside the hull.
const FADE_IN: f32 = 0.35;
/// Birth radius and turbulent spreading rate (m, m/s), both at full
/// strength — a station shedding hard throws a bigger patch that spreads
/// faster, so both scale with the eddy's own birth strength. This is what
/// makes a slipping hull lay a visibly WIDE swath while the same hull
/// running straight leaves a narrow ribbon.
const R0: f32 = 0.7;
const GROWTH: f32 = 0.20;
/// Floor on those two, as a fraction of full strength, so the faintest
/// churn still spreads instead of staying a pinprick forever.
const SPREAD_FLOOR: f32 = 0.4;
/// Fraction of the shedding station's own water-relative velocity that the
/// shed water carries away with it, and the time constant over which that
/// slug gives its momentum back to the surrounding water.
const CARRY: f32 = 0.35;
const CARRY_TAU: f32 = 2.0;
/// Swirl decay (s) — the visible rotation of an eddy winding down.
const SPIN_TAU: f32 = 3.5;
/// Reference speeds: lateral slip and axial speed (m/s) that each saturate
/// their half of an eddy's strength. `LAT_REF` is deliberately low — 0.8
/// m/s of slip (~1.6 kn) is already a boat being shoved bodily sideways,
/// and should churn like it; `AX_REF` is roughly this hull's top speed.
const LAT_REF: f32 = 0.8;
const AX_REF: f32 = 2.2;
/// Weight of the axial (boundary-layer) contribution relative to the
/// lateral (separated-flow) one. Skin friction sheds far less turbulence
/// than a stalled bluff body does — roughly fifty times less force, on
/// this hull at harbour speeds. This is nowhere near that ratio on
/// purpose: what you SEE is not the momentum, and a real boat under way
/// leaves an obvious slick astern. Sideways still wins comfortably.
const AXIAL_WEIGHT: f32 = 0.5;

/// One patch of churned water: a turbulent blob with a swirl in it.
struct Eddy {
    /// World position (m).
    p: Vec2,
    /// The water motion the hull imparted (m/s, world) — decays toward
    /// zero, leaving the eddy riding on the current alone.
    v: Vec2,
    /// Swirl rate (rad/s) and accumulated swirl angle (rad).
    spin: f32,
    phase: f32,
    /// Radius (m), grows with age, and the radius it was born at. The
    /// pair gives the dilution: a patch holds a fixed amount of stirred-up
    /// water, so spreading over more area makes it correspondingly
    /// fainter — that, not a clock, is most of why a wake fades.
    r: f32,
    r0: f32,
    age: f32,
    /// Birth strength, 0..1 — how hard the hull was working the water.
    strength: f32,
    /// How aerated the patch is, 0..1. The prop race is white water full
    /// of entrained air; flow separating off a keel is a smoother boil
    /// that mostly shows as a slick with a swirl in it. Drives the drawn
    /// colour and brightness — the two really do look different from a
    /// quay, and the trail reads better when it says which is which.
    foam: f32,
}

/// Aeration of the two shedding mechanisms — see `Eddy::foam`.
const FOAM_PROP: f32 = 1.0;
const FOAM_HULL: f32 = 0.25;

/// The whole trail. Frontend state: created once, cleared on respawn.
#[derive(Default)]
pub struct Wake {
    eddies: Vec<Eddy>,
    /// One fractional-spawn accumulator per source (hull stations, then
    /// the prop) so each sheds at its own rate independent of frame rate.
    spawn_acc: [f32; STATIONS + 1],
    /// Previous frame's pose, so a frame's worth of spawns can be spread
    /// along the path actually travelled instead of stacking at one point.
    last: Option<(Vec2, f32)>,
    seed: u32,
}

impl Wake {
    pub fn new() -> Wake {
        Wake::default()
    }

    /// Wipe the trail — for the R-reset, where the boat teleports back to
    /// its spawn and a surviving wake would draw a streak across the
    /// marina. NOT called for the keel editor's Apply: that boat carries
    /// on from where it was, and so does its wake.
    pub fn clear(&mut self) {
        self.eddies.clear();
        self.spawn_acc = [0.0; STATIONS + 1];
        self.last = None;
    }

    /// Advance and shed. Called once per rendered frame with the frame's
    /// own `dt` (cosmetic, so it does not need the physics timestep), and
    /// NOT called while the keel editor has the sim frozen.
    pub fn update(&mut self, dt: f32, sim: &Sim, design: &BoatDesign, env: &Env) {
        let current = env.current_vel();

        // --- Advect what is already on the water ------------------------
        let v_decay = (-dt / CARRY_TAU).exp();
        let spin_decay = (-dt / SPIN_TAU).exp();
        for e in &mut self.eddies {
            e.p += (current + e.v) * dt;
            e.v *= v_decay;
            e.phase += e.spin * dt;
            e.spin *= spin_decay;
            e.r += GROWTH * (SPREAD_FLOOR + (1.0 - SPREAD_FLOOR) * e.strength) * dt;
            e.age += dt;
        }
        self.eddies.retain(|e| e.age < EDDY_LIFE);

        // --- Shed new turbulence ----------------------------------------
        let (pos, heading) = sim.boat_pose();
        let (vel, yaw) = sim.boat_vel();
        let (prev_pos, prev_heading) = self.last.unwrap_or((pos, heading));
        self.last = Some((pos, heading));

        // Everything below is relative to the WATER: a boat carried along
        // by the current at current speed is not stirring anything.
        let rel = vel - current;
        let fwd = vec2(heading.cos(), heading.sin());
        let port = vec2(-fwd.y, fwd.x);
        let surge = rel.dot(fwd);
        let sway = rel.dot(port);

        let (aft, fore) = waterline_extent(&design.keel);
        let com = hull_com_x();
        let engine = sim.engine();

        for i in 0..STATIONS + 1 {
            let prop = i == STATIONS;
            // The race is shed at the PROP's own station, which sits
            // `PROP_AHEAD_OF_RUDDER` forward of the blade — read from
            // sim-core so the churn appears where `tick` actually
            // applies thrust, not half a metre abaft it.
            let x = if prop {
                prop_station(design.rudder.x)
            } else {
                aft + (fore - aft) * (i as f32 + 0.5) / STATIONS as f32
            };
            // Lateral velocity of this station through the water: sway
            // plus the yaw sweep about the centre of mass. This is the
            // same `V(x)` the cross-flow drag in `tick` integrates.
            let v_loc = sway + yaw * (x - com);

            let strength = if prop {
                // Thrust goes as engine·|engine|, and so does the momentum
                // the race dumps astern.
                (engine * engine).min(1.0)
            } else {
                let lat = (v_loc / LAT_REF).powi(2);
                let ax = (surge / AX_REF).powi(2) * AXIAL_WEIGHT;
                (lat + ax).min(1.0)
            };

            if strength < 0.02 {
                self.spawn_acc[i] = 0.0;
                continue;
            }
            self.spawn_acc[i] += strength * SPAWN_RATE * dt;
            // Cap the per-frame burst so a stalled frame cannot dump the
            // whole budget at one point on the track.
            let n = (self.spawn_acc[i] as u32).min(4);
            self.spawn_acc[i] -= n as f32;

            for _ in 0..n {
                self.seed = self.seed.wrapping_add(1);
                let (j0, j1, j2) = hash01(self.seed);
                let half_beam = half_beam_at(x);

                // Which side the water leaves on: the hull moving to port
                // sheds to starboard, and vice versa. With no lateral
                // motion at all (running straight, or the prop source)
                // the boundary layer leaves along both quarters, so
                // alternate instead of picking a meaningless sign.
                let side = if !prop && v_loc.abs() > 0.05 {
                    -v_loc.signum()
                } else if self.seed & 1 == 0 {
                    1.0
                } else {
                    -1.0
                };
                let local =
                    vec2(x + (j0 - 0.5) * 0.9, side * half_beam * (0.75 + 0.4 * j1));

                // Spread this frame's spawns along the path travelled, so
                // the trail stays a continuous band at low frame rates.
                let p = body_to_world(prev_pos, prev_heading, local)
                    .lerp(body_to_world(pos, heading, local), j2);

                // The shed water keeps some of the motion the hull gave
                // it: sideways for a slipping hull, straight astern for
                // the prop race.
                let v = if prop {
                    -fwd * (1.1 * engine)
                } else {
                    port * (v_loc * CARRY) + fwd * (surge * CARRY * 0.25)
                };

                // Counter-rotating pair: the end ahead of the pivot and
                // the end behind it roll their vortices opposite ways.
                let spin = if prop {
                    (j0 - 0.5) * 2.4
                } else {
                    2.2 * (v_loc / LAT_REF).clamp(-1.5, 1.5) * (x - com).signum()
                };

                let r0 =
                    R0 * (0.7 + 0.6 * j2) * (SPREAD_FLOOR + (1.0 - SPREAD_FLOOR) * strength);
                if self.eddies.len() >= EDDY_CAP {
                    self.eddies.remove(0);
                }
                self.eddies.push(Eddy {
                    p,
                    v,
                    spin,
                    phase: j1 * std::f32::consts::TAU,
                    r: r0,
                    r0,
                    age: 0.0,
                    strength,
                    foam: if prop { FOAM_PROP } else { FOAM_HULL },
                });
            }
        }
    }

    /// Draw the trail. Belongs on the water — after the ripples, before
    /// the shore fills, so eddies that drift ashore are covered like the
    /// stray ripples are.
    pub fn draw(&self, w2s: impl Fn(Vec2) -> Vec2, scale: f32, visible: impl Fn(Vec2, f32) -> bool) {
        // Smooth boil → a pale slick tinted with the water; aerated race
        // → near-white. `Eddy::foam` picks between them.
        let boil = Color::from_rgba(132, 186, 208, 255);
        let white = Color::from_rgba(226, 242, 248, 255);
        for e in &self.eddies {
            if !visible(e.p, e.r + 1.0) {
                continue;
            }
            let t = e.age / EDDY_LIFE;
            // Spreading dilutes the patch (see `Eddy::r0`); the age term
            // on top of it only makes sure nothing lingers at the moment
            // it is retired. Note what is NOT here: birth strength. How
            // hard the hull was working shows as how MANY patches there
            // are and how big each one is — coverage and texture, which
            // is how turbulence actually reads on water. Folding it into
            // the opacity as well would count it three times over.
            // The exponent is 2, not 1: as patches spread they also
            // OVERLAP more (their count along a track goes as their
            // radius), so a 1/r dilution would leave the far end of a
            // trail composited just as bright as the fresh churn — which
            // is what it looked like before this was raised.
            let dilution = (e.r0 / e.r).powi(2);
            let a = (e.age / FADE_IN).min(1.0) * (1.0 - t) * dilution;
            if a < 0.004 {
                continue;
            }
            let sp = w2s(e.p);
            let r_px = e.r * scale;
            let tint = Color::new(
                boil.r + (white.r - boil.r) * e.foam,
                boil.g + (white.g - boil.g) * e.foam,
                boil.b + (white.b - boil.b) * e.foam,
                1.0,
            );
            draw_poly(
                sp.x,
                sp.y,
                7,
                r_px,
                e.phase.to_degrees(),
                Color::new(tint.r, tint.g, tint.b, (0.20 + 0.16 * e.foam) * a),
            );
            // The swirl inside it only reads once the eddy is a few
            // pixels across and still bright enough to tell apart from
            // the patch it sits in; below either threshold the patch
            // alone carries it. (Measured: skipping these arcs does NOT
            // move the frame rate — the trail's cost is fill rate from
            // the translucent patches, not these few hundred lines. Kept
            // because it also removes detail nobody can see.)
            if r_px < 3.0 || a < 0.06 {
                continue;
            }
            let col = Color::new(tint.r, tint.g, tint.b, (0.34 + 0.22 * e.foam) * a);
            let w = (0.09 * scale).max(1.0);
            for k in 0..2 {
                let base = e.phase + k as f32 * std::f32::consts::PI;
                // A short spiral arm rather than a circle: an opening
                // curl reads as rotation, a closed ring reads as a bubble.
                let arm = |j: f32| -> Vec2 {
                    let ang = base + j * 2.3 * e.spin.signum();
                    let rad = r_px * (0.35 + 0.4 * j);
                    vec2(sp.x + rad * ang.cos(), sp.y + rad * ang.sin())
                };
                let mut prev = arm(0.0);
                for j in 1..=4 {
                    let next = arm(j as f32 / 4.0);
                    draw_line(prev.x, prev.y, next.x, next.y, w, col);
                    prev = next;
                }
            }
        }
    }
}

/// Boat-local metres → world.
fn body_to_world(pos: Vec2, heading: f32, local: Vec2) -> Vec2 {
    let (c, s) = (heading.cos(), heading.sin());
    pos + vec2(local.x * c - local.y * s, local.x * s + local.y * c)
}

/// Half beam (m) of `HULL_PTS` at a station, so eddies shed off the hull's
/// actual side rather than its centreline — linear interpolation over the
/// outline's +y chain, which runs bow → stern in `HULL_PTS` order.
fn half_beam_at(x: f32) -> f32 {
    let mut best = 0.0f32;
    for w in HULL_PTS.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if y0 < 0.0 || y1 < 0.0 {
            continue;
        }
        let (lo, hi) = if x0 <= x1 { (x0, x1) } else { (x1, x0) };
        if x < lo || x > hi || (x1 - x0).abs() < 1e-6 {
            continue;
        }
        let f = (x - x0) / (x1 - x0);
        best = best.max(y0 + (y1 - y0) * f);
    }
    best.max(0.35)
}

/// The deterministic scatter idiom used by the ripples and the scenery —
/// three 0..1 values from a counter, no RNG dependency.
fn hash01(i: u32) -> (f32, f32, f32) {
    let h = i.wrapping_mul(2654435761);
    (
        (h & 0xffff) as f32 / 65535.0,
        ((h >> 16) & 0x7fff) as f32 / 32767.0,
        ((h >> 8) & 0xff) as f32 / 255.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use harbour_sim_core::sim::{InputState, PHYSICS_DT};

    struct Trail {
        /// Churn shed by the HULL: the sum of the live eddies' birth
        /// strengths. The prop race is excluded throughout — it goes as
        /// `engine²` alone and would swamp any comparison at equal
        /// throttle.
        churn: f32,
        /// The same, per metre of track sailed — how hard the boat works
        /// the water for the distance it covers.
        per_metre: f32,
        /// Mean birth strength: how violent a typical patch is, as
        /// opposed to how many of them there are.
        mean_strength: f32,
        /// Whether any aerated prop-race churn was shed at all.
        prop: bool,
    }

    /// Run a boat on the given helm/engine, shedding wake at one eddy
    /// update per physics step. Six seconds, not three: an 8.5 t boat is
    /// still spooling up and gathering way before that, and correctly
    /// stirs almost nothing while it is.
    fn run(input: InputState) -> Trail {
        let design = BoatDesign::hallberg_rassy_38();
        let mut sim = Sim::new_with_design(&design);
        let mut wake = Wake::new();
        let mut sailed = 0.0;
        let mut last = sim.boat_pose().0;
        for _ in 0..(6.0 / PHYSICS_DT) as u32 {
            sim.tick(&Env::CALM, &input);
            wake.update(PHYSICS_DT, &sim, &design, &Env::CALM);
            let p = sim.boat_pose().0;
            sailed += (p - last).length();
            last = p;
        }
        let hull = || wake.eddies.iter().filter(|e| e.foam < FOAM_PROP);
        let churn: f32 = hull().map(|e| e.strength).sum();
        Trail {
            churn,
            per_metre: churn / sailed.max(0.1),
            mean_strength: churn / hull().count().max(1) as f32,
            prop: wake.eddies.iter().any(|e| e.foam == FOAM_PROP),
        }
    }

    /// The headline claim, and the reason this module exists: a hull with
    /// a sideways component to its motion is a separated bluff body, and
    /// churns far harder than the same hull running straight — both more
    /// churn for every metre it covers, and a more violent patch each
    /// time it sheds one.
    #[test]
    fn slipping_through_a_turn_churns_far_harder_than_running_straight() {
        let straight = run(InputState { throttle: 1.0, rudder: 0.0, ..InputState::NEUTRAL });
        let turning = run(InputState { throttle: 1.0, rudder: 1.0, ..InputState::NEUTRAL });
        assert!(
            turning.per_metre > straight.per_metre * 2.0,
            "turn should churn much harder per metre sailed: straight {:.2}, turning {:.2}",
            straight.per_metre,
            turning.per_metre
        );
        assert!(
            turning.mean_strength > straight.mean_strength * 1.5,
            "each patch the turn sheds should be more violent: straight {:.2}, turning {:.2}",
            straight.mean_strength,
            turning.mean_strength
        );
    }

    /// ...but running straight still leaves a trail. The old picture —
    /// prop streaks only, attached to the stern — left flat water behind
    /// the boat; the boundary layer sheds along both quarters too, and
    /// the race leaves its own aerated white water lying astern.
    #[test]
    fn running_straight_still_leaves_a_trail() {
        let t = run(InputState { throttle: 1.0, rudder: 0.0, ..InputState::NEUTRAL });
        assert!(t.churn > 1.0, "straight running should still shed some churn, got {:.2}", t.churn);
        assert!(t.prop, "the prop race should shed its own aerated churn");
    }

    /// This is water: shed turbulence is carried by the current for its
    /// whole life, it does not sit still on the ground.
    #[test]
    fn shed_turbulence_is_carried_by_the_current() {
        let design = BoatDesign::hallberg_rassy_38();
        let sim = Sim::new_with_design(&design);
        let env = Env { current_to_deg: 90.0, current_speed: 1.0, ..Env::CALM };
        let mut wake = Wake::new();
        wake.eddies.push(Eddy {
            p: Vec2::ZERO,
            v: Vec2::ZERO,
            spin: 0.0,
            phase: 0.0,
            r: R0,
            r0: R0,
            age: 0.0,
            strength: 1.0,
            foam: FOAM_HULL,
        });
        for _ in 0..120 {
            wake.update(PHYSICS_DT, &sim, &design, &env);
        }
        // 1 s of a 1 m/s current setting east (compass 90°) = 1 m east.
        let p = wake.eddies[0].p;
        assert!((p.x - 1.0).abs() < 0.02 && p.y.abs() < 0.02, "drifted to {p:?}, expected (1, 0)");
    }

    /// Eddies shed off the hull's side, so the half-beam lookup has to
    /// follow the real outline rather than a constant.
    #[test]
    fn half_beam_follows_the_hull_outline() {
        assert!(half_beam_at(-3.6) > half_beam_at(4.2), "widest amidships-aft, finest forward");
        assert!(half_beam_at(5.9) < 0.5, "the bow tapers to a point");
        for &(x, y) in HULL_PTS.iter() {
            if y > 0.0 {
                assert!((half_beam_at(x) - y).abs() < 1e-4, "outline vertex at x={x} not matched");
            }
        }
    }
}
