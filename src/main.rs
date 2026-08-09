//! Harbour Sim — macroquad frontend.
//!
//! Everything deterministic lives in `harbour-sim-core` (the `Sim`); this
//! file is input gathering, the fixed-timestep loop, and rendering. Nothing
//! here may mutate physics outside `Sim::tick` (the Pegasus rule).
//!
//! Units note (measured, and matching the Pegasus write-up): macroquad's
//! `screen_width()/screen_height()` and `mouse_position()` are LOGICAL css
//! px (physical / dpi), while `touches()` returns RAW PHYSICAL px — every
//! touch position must be divided by `screen_dpi_scale()` before it shares
//! space with the drawing/mouse coordinates. HUD sizes below are therefore
//! written directly in css px.

use harbour_sim_core::boat::BoatDesign;
use harbour_sim_core::sim::{Env, InputState, PHYSICS_DT, Sim};
use keel_editor::{EditorAction, EditorLayout, KeelEditor};
use macroquad::prelude::*;
use render2d::{Scenery, WorldFrame};
use render3d::Renderer3D;
use std::sync::atomic::{AtomicU32, Ordering};

mod keel_editor;
mod render2d;
mod render3d;

// DEFAULT zoom bounds for the fill-screen camera: never show more than
// VIEW_MAX_W × VIEW_MAX_H metres, never fewer than VIEW_MIN_W metres across.
// The camera fills the window (cropping the other axis) and follows the
// boat, clamped to the world rect — that's what makes a portrait phone show
// a sensible close-up.
const VIEW_MAX_W: f32 = 150.0;
const VIEW_MAX_H: f32 = 85.0;
const VIEW_MIN_W: f32 = 30.0;

// USER zoom on top of that (pinch on touch; scroll wheel or +/- keys on
// desktop — the two UIs stay on par, see CLAUDE.md): a multiplier on the
// default scale, clamped so the visible width stays between these. The
// wide end shows a ~450 m sweep of the ~800 m marina at once; the narrow
// end is a close-up for threading a berth.
const ZOOM_OUT_MAX_W: f32 = 450.0;
const ZOOM_IN_MIN_W: f32 = 24.0;

// Environment knob rates (per second of key held) and ranges. The touch
// dials share the same WIND_MAX / CURRENT_MAX: dial rim = max.
const DIR_RATE: f32 = 45.0; // degrees
const WIND_RATE: f32 = 3.0; // m/s
const CURRENT_RATE: f32 = 0.4; // m/s
const WIND_MAX: f32 = 25.0;
const CURRENT_MAX: f32 = 2.5;

// Helm/engine key rates (full-scale units per second of key held):
// hard-over in ~1.1 s, idle to full throttle in ~1.4 s.
const RUDDER_KEY_RATE: f32 = 0.9;
const THROTTLE_KEY_RATE: f32 = 0.7;

// --- Safe-area insets (css px), pushed from index.html on the web build.
// Native builds never call the export and stay at 0.
static SAFE_T: AtomicU32 = AtomicU32::new(0);
static SAFE_L: AtomicU32 = AtomicU32::new(0);
static SAFE_B: AtomicU32 = AtomicU32::new(0);
static SAFE_R: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
pub extern "C" fn set_safe_area(top: f32, left: f32, bottom: f32, right: f32) {
    SAFE_T.store(top.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_L.store(left.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_B.store(bottom.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_R.store(right.max(0.0).to_bits(), Ordering::Relaxed);
}

fn safe_area() -> (f32, f32, f32, f32) {
    (
        f32::from_bits(SAFE_T.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_L.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_B.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_R.load(Ordering::Relaxed)),
    )
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Harbour Sim".to_owned(),
        high_dpi: true,
        ..Default::default()
    }
}

/// Fresh `Sim` + reset render-interpolation state, shared by the R-reset
/// key and the keel editor's Apply — never mutate an existing `Sim` in
/// place (determinism rule), always spawn a new one.
fn respawn(design: &BoatDesign) -> (Sim, Vec2, f32, Vec2, f32) {
    let sim = Sim::new_with_design(design);
    let (pos, heading) = sim.boat_pose();
    (sim, pos, heading, pos, heading)
}

/// Which world view is on screen. Cycled by the V key / VIEW button; the
/// HUD is identical in every mode.
#[derive(Clone, Copy, PartialEq)]
enum ViewMode {
    /// The classic top-down view (with zoom/pan — see the camera block).
    TopDown,
    /// The 3D chase camera (see render3d.rs). No zoom/pan gestures here.
    Chase,
    /// Chase full-screen + a fixed top-down inset (top-centre) — the 3D
    /// immersion with the berthing-distance view kept in the corner of
    /// your eye. No zoom/pan either; the inset follows the boat.
    Both,
    /// First-person at the helm, rigid with the hull (see render3d.rs).
    /// The hardest view to moor from — and the most like the real thing.
    Cockpit,
}

impl ViewMode {
    fn next(self) -> ViewMode {
        match self {
            ViewMode::TopDown => ViewMode::Chase,
            ViewMode::Chase => ViewMode::Both,
            ViewMode::Both => ViewMode::Cockpit,
            ViewMode::Cockpit => ViewMode::TopDown,
        }
    }
}

/// Shortest-path angle interpolation (for the render lerp across a tick).
fn lerp_angle(a: f32, b: f32, t: f32) -> f32 {
    let mut d = (b - a) % std::f32::consts::TAU;
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    if d < -std::f32::consts::PI {
        d += std::f32::consts::TAU;
    }
    a + d * t
}

/// A draggable compass dial (screen-space geometry, css px).
#[derive(Clone, Copy)]
struct Dial {
    cx: f32,
    cy: f32,
    r: f32,
}

impl Dial {
    fn hit(&self, p: Vec2) -> bool {
        // Generous hit area — fat fingers land outside the drawn ring.
        (p - vec2(self.cx, self.cy)).length() <= self.r * 1.45
    }

    /// Drag position → (compass direction the flow points TOWARD, 0..1
    /// magnitude). Screen y grows downward, compass 0° = north = up.
    fn value(&self, p: Vec2) -> (f32, f32) {
        let v = p - vec2(self.cx, self.cy);
        let to_deg = v.x.atan2(-v.y).to_degrees().rem_euclid(360.0);
        let frac = if v.length() < self.r * 0.12 {
            0.0 // centre dead-zone: an easy way to set dead calm
        } else {
            (v.length() / self.r).clamp(0.0, 1.0)
        };
        (to_deg.round(), (frac * 20.0).round() / 20.0)
    }
}

/// A draggable linear slider (screen-space css px) holding a -1..=1 value:
/// the throttle (vertical, up = ahead) and the rudder (horizontal, right =
/// starboard helm). Like a real single-lever control or a helm with
/// friction it HOLDS where it's left — no spring-return — with a centre
/// detent so neutral/amidships is easy to hit by feel.
#[derive(Clone, Copy)]
struct Slider {
    rect: Rect,
    vertical: bool,
}

impl Slider {
    fn hit(&self, p: Vec2) -> bool {
        // Generous pad, same reason as Dial::hit.
        let pad = 12.0;
        p.x >= self.rect.x - pad
            && p.x <= self.rect.x + self.rect.w + pad
            && p.y >= self.rect.y - pad
            && p.y <= self.rect.y + self.rect.h + pad
    }

    fn value(&self, p: Vec2) -> f32 {
        let raw = if self.vertical {
            (self.rect.y + self.rect.h * 0.5 - p.y) / (self.rect.h * 0.5)
        } else {
            (p.x - self.rect.x - self.rect.w * 0.5) / (self.rect.w * 0.5)
        };
        let v = if raw.abs() < 0.10 { 0.0 } else { raw.clamp(-1.0, 1.0) };
        (v * 20.0).round() / 20.0 // same 1/20 quantisation as the dials
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Touches are handled natively below; without this a touch would also
    // synthesize a mouse press (= a phantom mouse dial-grab).
    simulate_mouse_with_touch(false);

    let mut design = BoatDesign::hallberg_rassy_38();
    let mut sim = Sim::new_with_design(&design);
    let mut editor = KeelEditor::new(&design);

    // --- Static scenery, computed once (see render2d::Scenery) — sim-core
    // is the single source of truth for all of it: what's drawn IS what
    // collides.
    let scenery = Scenery::build();
    let (bmin, bmax) = (scenery.bmin, scenery.bmax);
    let mut r3d = Renderer3D::new(&scenery);
    let mut view = ViewMode::TopDown;
    let mut env = Env {
        wind_from_deg: 315.0,
        wind_speed: 6.0,
        current_to_deg: 90.0,
        current_speed: 0.4,
    };
    // Helm + engine, the other half of the input stream. Unlike `env` this
    // resets to neutral with the boat (see the do_reset block).
    let mut input = InputState::NEUTRAL;

    let mut accum = 0.0f32;
    let (mut prev_pos, mut prev_heading) = sim.boat_pose();
    let (mut cur_pos, mut cur_heading) = (prev_pos, prev_heading);
    r3d.snap_to(cur_pos, cur_heading);

    // Touch/mouse claims for the two dials + two sliders. "Fresh touch"
    // detection is by id-not-seen-last-frame, NOT by TouchPhase::Started —
    // touchstart collapses into the following touchmove whenever events
    // outpace the frame loop (the hard-won Pegasus lesson in
    // docs/touch-input.md). Per-control claims are what make the two-thumb
    // grip work: one finger on the throttle, one on the rudder, at once.
    let mut prev_touch_ids: Vec<u64> = Vec::new();
    let mut wind_claim: Option<u64> = None;
    let mut current_claim: Option<u64> = None;
    let mut throttle_claim: Option<u64> = None;
    let mut rudder_claim: Option<u64> = None;
    let mut mouse_claim: Option<u8> = None; // 0 = wind, 1 = current, 2 = throttle, 3 = rudder
    // User camera zoom (see ZOOM_* above) and the live pinch, if any:
    // the two finger ids (sorted) + their separation last frame. Zoom is
    // a camera preference — it survives resets and respawns.
    let mut zoom = 1.0f32;
    let mut pinch: Option<(u64, u64, f32)> = None;
    // Camera pan: an OFFSET from the boat, in world metres (one-finger
    // drag on the water, or a mouse drag — mouse_claim 4). The camera
    // keeps FOLLOWING the boat while panned, displaced by this — watch
    // your own approach from over the berth, say — rather than freezing
    // on a fixed world point (owner request 2026-08-05; the fixed-anchor
    // version was tried first and replaced). Cleared by the CENTER
    // button / C key and by any respawn.
    let mut cam_offset = Vec2::ZERO;
    let mut pan_touch: Option<(u64, Vec2)> = None;
    let mut pan_mouse_prev = Vec2::ZERO;
    // Last frame's camera scale, for converting a pan's screen delta to
    // world metres at input time (the camera block runs later).
    let mut last_scale = 1.0f32;

    loop {
        let dt = get_frame_time().min(0.05);
        let sw = screen_width();
        let sh = screen_height();
        let dpi = screen_dpi_scale();
        let (sa_t, sa_l, sa_b, sa_r) = safe_area();
        let min_dim = sw.min(sh);

        // E = Editor. (Was K until the boat took WASD and the current took
        // IJKL — K is now current-speed-down.)
        if is_key_pressed(KeyCode::E) {
            if !editor.active {
                editor.load_design(&design);
            }
            editor.active = !editor.active;
        }

        // --- HUD layout (css px) -----------------------------------------
        // Computed every frame regardless of editor state: the dials/reset
        // button still render (frozen) behind the editor overlay.
        let margin = (min_dim * 0.02).clamp(8.0, 18.0);
        let dial_r = (min_dim * 0.11).clamp(34.0, 54.0);
        let fs = (min_dim * 0.035).clamp(12.0, 24.0);
        let wind_dial = Dial {
            cx: sa_l + margin + dial_r,
            cy: sa_t + margin + dial_r,
            r: dial_r,
        };
        let current_dial = Dial {
            cx: sw - sa_r - margin - dial_r,
            cy: sa_t + margin + dial_r,
            r: dial_r,
        };
        let reset_w = fs * 4.6;
        let reset_h = fs * 2.2;
        let reset_rect = Rect::new(
            sw - sa_r - margin - reset_w,
            sh - sa_b - margin - reset_h,
            reset_w,
            reset_h,
        );
        // Keel editor button, left of RESET — the touch/mouse twin of the K
        // key. Without this there's no way to reach the editor at all on a
        // touch-only device (no keyboard).
        let keel_w = fs * 4.6;
        let keel_h = fs * 2.2;
        let keel_rect = Rect::new(
            reset_rect.x - margin - keel_w,
            sh - sa_b - margin - keel_h,
            keel_w,
            keel_h,
        );
        // VIEW button, left of KEEL — the touch/mouse twin of the V key
        // (cycles the view mode; without it there'd be no way to reach the
        // 3D view on a touch-only device).
        let view_w = fs * 4.6;
        let view_rect = Rect::new(
            keel_rect.x - margin - view_w,
            sh - sa_b - margin - keel_h,
            view_w,
            keel_h,
        );
        // CENTER button, left of VIEW — the touch/mouse twin of the C key.
        // Only shown (and only hittable) while the camera is panned away
        // from the boat, so the button row stays uncluttered otherwise.
        let center_w = fs * 5.2;
        let center_rect = Rect::new(
            view_rect.x - margin - center_w,
            sh - sa_b - margin - keel_h,
            center_w,
            keel_h,
        );
        // Helm/engine sliders on the mid-left/mid-right edges — the
        // two-thumb zone on a phone, clear of the dials above (centre at
        // 0.56·sh keeps the throttle's top under the wind dial's label
        // down to ~360 px min-dim) and the buttons below.
        let sl_w = (min_dim * 0.085).clamp(26.0, 40.0);
        let sl_len = (min_dim * 0.42).clamp(130.0, 230.0);
        let throttle_slider = Slider {
            rect: Rect::new(sa_l + margin, sh * 0.56 - sl_len * 0.5, sl_w, sl_len),
            vertical: true,
        };
        let rudder_slider = Slider {
            rect: Rect::new(sw - sa_r - margin - sl_len, sh * 0.56 - sl_w * 0.5, sl_len, sl_w),
            vertical: false,
        };

        if !editor.active {
            let mut do_reset = is_key_pressed(KeyCode::R);
            let mut do_open_editor = false;
            // Zoom/pan (and the CENTER twin) belong to the top-down camera
            // only; in the chase view those gestures are ignored rather
            // than silently panning a camera you can't see. The COCKPIT
            // view reuses the same free-drag gesture as free-look (and
            // C/CENTER as "eyes forward").
            let top_down = view == ViewMode::TopDown;
            let in_cockpit = view == ViewMode::Cockpit;
            let mut do_center = (top_down || in_cockpit) && is_key_pressed(KeyCode::C);
            let mut do_toggle_view = is_key_pressed(KeyCode::V);
            // Drag-to-look sensitivity: a full-width swipe sweeps ~200° of
            // yaw whatever the screen size (phones get the same reach per
            // swipe as a desktop drag), pitch proportionally.
            let look_sens = 3.5 / sw;
            // Same "grab the world" convention as the top-down pan: drag
            // right → the world follows your finger → you look LEFT (CCW,
            // positive yaw); drag down → look up.
            let center_visible = (top_down && cam_offset.length() > 0.5)
                || (in_cockpit && r3d.look_active());

            // --- Touch input: dial drags + reset/keel taps -----------------
            let ts = touches();
            let cur_ids: Vec<u64> = ts.iter().map(|t| t.id).collect();
            for t in &ts {
                let p = t.position / dpi; // physical → logical (see header note)
                let fresh = !prev_touch_ids.contains(&t.id) || t.phase == TouchPhase::Started;
                if fresh {
                    // A recycled id is a NEW finger: drop any stale claim first.
                    if wind_claim == Some(t.id) {
                        wind_claim = None;
                    }
                    if current_claim == Some(t.id) {
                        current_claim = None;
                    }
                    if throttle_claim == Some(t.id) {
                        throttle_claim = None;
                    }
                    if rudder_claim == Some(t.id) {
                        rudder_claim = None;
                    }
                    if wind_dial.hit(p) && wind_claim.is_none() {
                        wind_claim = Some(t.id);
                    } else if current_dial.hit(p) && current_claim.is_none() {
                        current_claim = Some(t.id);
                    } else if throttle_slider.hit(p) && throttle_claim.is_none() {
                        throttle_claim = Some(t.id);
                    } else if rudder_slider.hit(p) && rudder_claim.is_none() {
                        rudder_claim = Some(t.id);
                    } else if reset_rect.contains(p) {
                        do_reset = true;
                    } else if keel_rect.contains(p) {
                        do_open_editor = true;
                    } else if view_rect.contains(p) {
                        do_toggle_view = true;
                    } else if center_visible && center_rect.contains(p) {
                        do_center = true;
                    }
                }
                if wind_claim == Some(t.id) {
                    let (to, frac) = wind_dial.value(p);
                    env.wind_from_deg = (to + 180.0).rem_euclid(360.0);
                    env.wind_speed = frac * WIND_MAX;
                } else if current_claim == Some(t.id) {
                    let (to, frac) = current_dial.value(p);
                    env.current_to_deg = to;
                    env.current_speed = frac * CURRENT_MAX;
                } else if throttle_claim == Some(t.id) {
                    input.throttle = throttle_slider.value(p);
                } else if rudder_claim == Some(t.id) {
                    input.rudder = rudder_slider.value(p);
                }
            }
            if wind_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                wind_claim = None;
            }
            if current_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                current_claim = None;
            }
            if throttle_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                throttle_claim = None;
            }
            if rudder_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                rudder_claim = None;
            }
            prev_touch_ids = cur_ids;

            // --- Pan and pinch on fingers that are NOT holding a HUD
            // control. One free finger drags the camera's follow-offset
            // off the boat (see `cam_offset`); exactly two free fingers
            // pinch-zoom, tracked by the sorted id pair so a third finger
            // landing on a slider (or one of the pair being recycled)
            // ends the gesture instead of jumping the zoom. Pinch does
            // NOT pan — zoom leaves the offset alone.
            let free: Vec<(u64, Vec2)> = ts
                .iter()
                .filter(|t| {
                    wind_claim != Some(t.id)
                        && current_claim != Some(t.id)
                        && throttle_claim != Some(t.id)
                        && rudder_claim != Some(t.id)
                })
                .map(|t| (t.id, t.position / dpi))
                .collect();
            match free[..] {
                [(id, p)] if top_down => {
                    if let Some((pid, prev)) = pan_touch
                        && pid == id
                    {
                        let d = p - prev;
                        cam_offset.x -= d.x / last_scale;
                        cam_offset.y += d.y / last_scale; // screen y is inverted
                    }
                    pan_touch = Some((id, p));
                    pinch = None;
                }
                [(id, p)] if in_cockpit => {
                    if let Some((pid, prev)) = pan_touch
                        && pid == id
                    {
                        let d = p - prev;
                        r3d.add_look(d.x * look_sens, d.y * look_sens);
                    }
                    pan_touch = Some((id, p));
                    pinch = None;
                }
                [(ida, pa), (idb, pb)] if top_down => {
                    let key = (ida.min(idb), ida.max(idb));
                    let d = (pa - pb).length();
                    if let Some((a, b, d0)) = pinch
                        && (a, b) == key
                        && d0 > 1.0
                    {
                        zoom *= d / d0;
                    }
                    pinch = Some((key.0, key.1, d));
                    pan_touch = None;
                }
                _ => {
                    pinch = None;
                    pan_touch = None;
                }
            }

            // --- Mouse input: same dials, same gesture ---------------------
            let mp: Vec2 = mouse_position().into();
            if is_mouse_button_pressed(MouseButton::Left) {
                if wind_dial.hit(mp) {
                    mouse_claim = Some(0);
                } else if current_dial.hit(mp) {
                    mouse_claim = Some(1);
                } else if throttle_slider.hit(mp) {
                    mouse_claim = Some(2);
                } else if rudder_slider.hit(mp) {
                    mouse_claim = Some(3);
                } else if reset_rect.contains(mp) {
                    do_reset = true;
                } else if keel_rect.contains(mp) {
                    do_open_editor = true;
                } else if view_rect.contains(mp) {
                    do_toggle_view = true;
                } else if center_visible && center_rect.contains(mp) {
                    do_center = true;
                } else if top_down || in_cockpit {
                    // Anywhere on the water: drag to pan — or, at the
                    // helm, drag to look around (claim 4 either way).
                    mouse_claim = Some(4);
                    pan_mouse_prev = mp;
                }
            }
            if is_mouse_button_down(MouseButton::Left) {
                match mouse_claim {
                    Some(0) => {
                        let (to, frac) = wind_dial.value(mp);
                        env.wind_from_deg = (to + 180.0).rem_euclid(360.0);
                        env.wind_speed = frac * WIND_MAX;
                    }
                    Some(1) => {
                        let (to, frac) = current_dial.value(mp);
                        env.current_to_deg = to;
                        env.current_speed = frac * CURRENT_MAX;
                    }
                    Some(2) => input.throttle = throttle_slider.value(mp),
                    Some(3) => input.rudder = rudder_slider.value(mp),
                    Some(4) if top_down => {
                        let d = mp - pan_mouse_prev;
                        cam_offset.x -= d.x / last_scale;
                        cam_offset.y += d.y / last_scale; // screen y is inverted
                        pan_mouse_prev = mp;
                    }
                    Some(4) if in_cockpit => {
                        let d = mp - pan_mouse_prev;
                        r3d.add_look(d.x * look_sens, d.y * look_sens);
                        pan_mouse_prev = mp;
                    }
                    _ => {}
                }
            } else {
                mouse_claim = None;
            }

            // --- Keyboard input ---------------------------------------------
            // The boat has the primary keys — driving it is the main
            // activity: W/S throttle, A/D helm, Space cuts the engine to
            // neutral. Wind keeps the arrows; the current sits on IJKL,
            // the "second arrows" cluster, with the same spatial meaning
            // (I/K speed up/down, J/L rotate direction).
            if is_key_down(KeyCode::W) {
                input.throttle = (input.throttle + THROTTLE_KEY_RATE * dt).min(1.0);
            }
            if is_key_down(KeyCode::S) {
                input.throttle = (input.throttle - THROTTLE_KEY_RATE * dt).max(-1.0);
            }
            if is_key_down(KeyCode::A) {
                input.rudder = (input.rudder - RUDDER_KEY_RATE * dt).max(-1.0);
            }
            if is_key_down(KeyCode::D) {
                input.rudder = (input.rudder + RUDDER_KEY_RATE * dt).min(1.0);
            }
            if is_key_pressed(KeyCode::Space) {
                input.throttle = 0.0;
            }
            if is_key_down(KeyCode::Left) {
                env.wind_from_deg -= DIR_RATE * dt;
            }
            if is_key_down(KeyCode::Right) {
                env.wind_from_deg += DIR_RATE * dt;
            }
            if is_key_down(KeyCode::Up) {
                env.wind_speed = (env.wind_speed + WIND_RATE * dt).min(WIND_MAX);
            }
            if is_key_down(KeyCode::Down) {
                env.wind_speed = (env.wind_speed - WIND_RATE * dt).max(0.0);
            }
            if is_key_down(KeyCode::J) {
                env.current_to_deg -= DIR_RATE * dt;
            }
            if is_key_down(KeyCode::L) {
                env.current_to_deg += DIR_RATE * dt;
            }
            if is_key_down(KeyCode::I) {
                env.current_speed = (env.current_speed + CURRENT_RATE * dt).min(CURRENT_MAX);
            }
            if is_key_down(KeyCode::K) {
                env.current_speed = (env.current_speed - CURRENT_RATE * dt).max(0.0);
            }
            env.wind_from_deg = env.wind_from_deg.rem_euclid(360.0);
            env.current_to_deg = env.current_to_deg.rem_euclid(360.0);

            // --- Zoom, desktop side: scroll wheel and +/- keys (the touch
            // twin is the pinch above). Wheel deltas differ wildly between
            // native (±1 per notch) and web (deltaY pixels, ~±100 per
            // notch), so small values are treated as notches and large
            // ones as pixels, both bounded per event.
            let (_, wheel_y) = mouse_wheel();
            if wheel_y != 0.0 && top_down {
                let step = if wheel_y.abs() >= 40.0 { wheel_y / 240.0 } else { wheel_y * 0.25 };
                zoom *= 2.0f32.powf(step.clamp(-0.6, 0.6));
            }
            if top_down && (is_key_down(KeyCode::Equal) || is_key_down(KeyCode::KpAdd)) {
                zoom *= 2.0f32.powf(dt);
            }
            if top_down && (is_key_down(KeyCode::Minus) || is_key_down(KeyCode::KpSubtract)) {
                zoom *= 2.0f32.powf(-dt);
            }

            if do_toggle_view {
                view = view.next();
            }
            if do_center {
                if in_cockpit {
                    r3d.reset_look(); // eyes forward
                } else {
                    cam_offset = Vec2::ZERO;
                }
            }
            if do_reset {
                // Fresh Sim per run — never reuse one (determinism rule).
                (sim, prev_pos, prev_heading, cur_pos, cur_heading) = respawn(&design);
                accum = 0.0;
                // Helm and engine come back neutral with the fresh boat;
                // the environment deliberately persists (same as always).
                input = InputState::NEUTRAL;
                // A fresh boat gets the camera back too (zoom persists),
                // and the chase camera snaps rather than swooping over.
                cam_offset = Vec2::ZERO;
                r3d.snap_to(cur_pos, cur_heading);
            }
            if do_open_editor {
                editor.load_design(&design);
                editor.active = true;
            }

            // --- Fixed-timestep physics with render interpolation. ---------
            accum += dt;
            while accum >= PHYSICS_DT {
                prev_pos = cur_pos;
                prev_heading = cur_heading;
                sim.tick(&env, &input);
                (cur_pos, cur_heading) = sim.boat_pose();
                accum -= PHYSICS_DT;
            }
        }
        // Physics is frozen while the keel editor is open — the displayed
        // pose just holds at whatever it last interpolated to.
        let alpha = accum / PHYSICS_DT;
        let pos = prev_pos.lerp(cur_pos, alpha);
        let heading = lerp_angle(prev_heading, cur_heading, alpha);

        // --- Camera: fill the screen, follow the boat, clamp to world ----
        // The user zoom multiplies the default fill-screen scale; it is
        // re-clamped every frame against the CURRENT window size so a
        // resize can't strand it outside its bounds, and the clamp always
        // admits 1.0 (the default) even on extreme aspect ratios.
        let base_scale = (sw / VIEW_MAX_W).max(sh / VIEW_MAX_H).min(sw / VIEW_MIN_W);
        let zoom_lo = (sw / ZOOM_OUT_MAX_W) / base_scale;
        let zoom_hi = (sw / ZOOM_IN_MIN_W) / base_scale;
        zoom = zoom.clamp(zoom_lo.min(1.0), zoom_hi.max(1.0));
        let scale = base_scale * zoom;
        let (wl, wr) = (bmin.x - 6.0, bmax.x + 6.0);
        let (wb, wt) = (bmin.y - 6.0, bmax.y + 6.0);
        let vis_hw = sw * 0.5 / scale;
        let vis_hh = sh * 0.5 / scale;
        // Keep the pan target inside the world, so shoving the offset
        // against the edge racks up no invisible travel to undo later —
        // clamped as a POINT (boat + offset), then folded back into the
        // offset. With a zero offset the boat is always inside the world,
        // so this can never conjure an offset out of nothing.
        cam_offset = vec2(
            (pos.x + cam_offset.x).clamp(wl, wr) - pos.x,
            (pos.y + cam_offset.y).clamp(wb, wt) - pos.y,
        );
        // Follow the boat, displaced by the pan offset.
        let follow = pos + cam_offset;
        let cam_x = if vis_hw * 2.0 >= wr - wl {
            (wl + wr) * 0.5
        } else {
            follow.x.clamp(wl + vis_hw, wr - vis_hw)
        };
        let cam_y = if vis_hh * 2.0 >= wt - wb {
            (wb + wt) * 0.5
        } else {
            follow.y.clamp(wb + vis_hh, wt - vis_hh)
        };
        last_scale = scale;

        // --- World (top-down render2d.rs / chase render3d.rs) -------------
        let frame = WorldFrame {
            pos,
            heading,
            // Blade angle for the rudder visual and the wash direction (same
            // formula as sim-core: positive = trailing edge to port).
            blade: -input.rudder * 35f32.to_radians(),
            engine: sim.engine(),
            time: get_time() as f32,
            env: &env,
            design: &design,
        };
        match view {
            ViewMode::TopDown => {
                clear_background(Color::from_rgba(12, 38, 54, 255));
                render2d::draw_world(sw, sh, scale, vec2(cam_x, cam_y), &scenery, &frame);
            }
            ViewMode::Chase | ViewMode::Both | ViewMode::Cockpit => {
                // Overcast-Nordic sky; the waterplane meets it at the
                // horizon.
                clear_background(Color::from_rgba(96, 118, 138, 255));
                r3d.draw(&scenery, &frame, dt, view == ViewMode::Cockpit);
                // Back to the screen-space camera for the HUD below.
                set_default_camera();

                if view == ViewMode::Both {
                    // Top-down inset, top-centre under the SOG line (the
                    // only reliably free HUD region: dials own the top
                    // corners, sliders the mid-edges, buttons the bottom).
                    // A Camera2D whose WORLD space is inset-local css px:
                    // the same draw_world code runs unchanged, and the GPU
                    // clips everything outside the viewport (macroquad
                    // viewports are PHYSICAL px, bottom-left origin).
                    let side = (min_dim * 0.30).clamp(110.0, 220.0);
                    let ix = sw * 0.5 - side * 0.5;
                    let iy = sa_t + margin + fs * 2.0;
                    let iscale =
                        (side / VIEW_MAX_W).max(side / VIEW_MAX_H).min(side / VIEW_MIN_W);
                    let vis_h = side * 0.5 / iscale;
                    let icam = vec2(
                        if vis_h * 2.0 >= wr - wl {
                            (wl + wr) * 0.5
                        } else {
                            pos.x.clamp(wl + vis_h, wr - vis_h)
                        },
                        if vis_h * 2.0 >= wt - wb {
                            (wb + wt) * 0.5
                        } else {
                            pos.y.clamp(wb + vis_h, wt - vis_h)
                        },
                    );
                    set_camera(&Camera2D {
                        target: vec2(side * 0.5, side * 0.5),
                        // +y zoom = css-px y-down here: macroquad's screen
                        // path already flips y once (`invert_y` in its
                        // Camera2D matrix), unlike `from_display_rect`.
                        zoom: vec2(2.0 / side, 2.0 / side),
                        offset: vec2(0.0, 0.0),
                        rotation: 0.0,
                        render_target: None,
                        viewport: Some((
                            (ix * dpi) as i32,
                            ((sh - iy - side) * dpi) as i32,
                            (side * dpi) as i32,
                            (side * dpi) as i32,
                        )),
                    });
                    render2d::draw_world(side, side, iscale, icam, &scenery, &frame);
                    set_default_camera();
                    // A thin frame so the inset reads as an instrument.
                    draw_rectangle_lines(
                        ix,
                        iy,
                        side,
                        side,
                        1.5,
                        Color::from_rgba(130, 160, 178, 255),
                    );
                }
            }
        }
        // --- HUD ---------------------------------------------------------
        let text = Color::from_rgba(205, 227, 240, 255);
        let dim = Color::from_rgba(130, 160, 178, 255);
        let wind_col = Color::from_rgba(120, 220, 255, 255);
        let cur_col = Color::from_rgba(90, 235, 170, 255);

        // A dial: bg disc, ring (bright while grabbed), N tick, arrow of the
        // flow's TOWARD direction with a knob at the magnitude, label below.
        let draw_dial = |d: &Dial, vel: Vec2, frac: f32, col: Color, grabbed: bool, label: &str| {
            draw_circle(d.cx, d.cy, d.r, Color::from_rgba(10, 20, 30, 150));
            let ring = if grabbed { col } else { dim };
            draw_circle_lines(d.cx, d.cy, d.r, if grabbed { 2.5 } else { 1.5 }, ring);
            draw_text("N", d.cx - fs * 0.22, d.cy - d.r + fs * 0.75, fs * 0.7, dim);
            if vel.length() > 1e-3 {
                let dir = vec2(vel.x, -vel.y).normalize(); // screen y down
                let tip = vec2(d.cx, d.cy) + dir * d.r * frac.max(0.18);
                let tail = vec2(d.cx, d.cy) - dir * d.r * 0.25;
                draw_line(tail.x, tail.y, tip.x, tip.y, 3.0, col);
                let n = vec2(-dir.y, dir.x);
                draw_triangle(
                    tip + dir * fs * 0.55,
                    tip - dir * fs * 0.1 + n * fs * 0.4,
                    tip - dir * fs * 0.1 - n * fs * 0.4,
                    col,
                );
            } else {
                draw_circle(d.cx, d.cy, 3.0, col);
            }
            let dims = measure_text(label, None, fs as u16, 1.0);
            draw_text(
                label,
                (d.cx - dims.width * 0.5).clamp(4.0, sw - dims.width - 4.0),
                d.cy + d.r + fs * 1.1,
                fs,
                col,
            );
        };

        draw_dial(
            &wind_dial,
            env.wind_vel(),
            env.wind_speed / WIND_MAX,
            wind_col,
            wind_claim.is_some() || mouse_claim == Some(0),
            &format!("WIND {:.1} m/s from {:03.0}", env.wind_speed, env.wind_from_deg),
        );
        draw_dial(
            &current_dial,
            env.current_vel(),
            env.current_speed / CURRENT_MAX,
            cur_col,
            current_claim.is_some() || mouse_claim == Some(1),
            &format!("CURR {:.1} m/s to {:03.0}", env.current_speed, env.current_to_deg),
        );

        let eng_col = Color::from_rgba(255, 185, 80, 255);
        let rud_col = Color::from_rgba(200, 160, 255, 255);

        // A slider: translucent track, centre-detent tick, filled bar from
        // the centre to the value, knob line, bright outline while grabbed.
        let draw_slider = |sl: &Slider, val: f32, col: Color, grabbed: bool| {
            let r = sl.rect;
            draw_rectangle(r.x, r.y, r.w, r.h, Color::from_rgba(10, 20, 30, 150));
            draw_rectangle_lines(
                r.x,
                r.y,
                r.w,
                r.h,
                if grabbed { 2.5 } else { 1.5 },
                if grabbed { col } else { dim },
            );
            let fill = Color::new(col.r, col.g, col.b, 0.35);
            if sl.vertical {
                let cy = r.y + r.h * 0.5;
                draw_line(r.x, cy, r.x + r.w, cy, 1.5, dim);
                let ky = cy - val * r.h * 0.5;
                if val.abs() > 1e-3 {
                    draw_rectangle(r.x + r.w * 0.2, ky.min(cy), r.w * 0.6, (cy - ky).abs(), fill);
                }
                draw_line(r.x, ky, r.x + r.w, ky, 3.0, col);
            } else {
                let cx = r.x + r.w * 0.5;
                draw_line(cx, r.y, cx, r.y + r.h, 1.5, dim);
                let kx = cx + val * r.w * 0.5;
                if val.abs() > 1e-3 {
                    draw_rectangle(kx.min(cx), r.y + r.h * 0.2, (kx - cx).abs(), r.h * 0.6, fill);
                }
                draw_line(kx, r.y, kx, r.y + r.h, 3.0, col);
            }
        };

        // Throttle: F(orward) / R(everse) end marks beside the track, and
        // an engine readout under it. The readout shows the TELEGRAPH
        // (what's commanded); the spooled response is what the boat does.
        let tr = throttle_slider.rect;
        draw_slider(
            &throttle_slider,
            input.throttle,
            eng_col,
            throttle_claim.is_some() || mouse_claim == Some(2),
        );
        draw_text("F", tr.x + tr.w + 4.0, tr.y + fs * 0.8, fs * 0.7, dim);
        draw_text("R", tr.x + tr.w + 4.0, tr.y + tr.h - fs * 0.15, fs * 0.7, dim);
        let eng_label = if input.throttle > 0.0 {
            format!("ENG {:.0}% AHD", input.throttle * 100.0)
        } else if input.throttle < 0.0 {
            format!("ENG {:.0}% AST", -input.throttle * 100.0)
        } else {
            "ENG NEUTRAL".to_owned()
        };
        draw_text(&eng_label, tr.x, tr.y + tr.h + fs * 1.1, fs * 0.8, eng_col);

        // Rudder: helm angle readout under the track's centre.
        let rr = rudder_slider.rect;
        draw_slider(
            &rudder_slider,
            input.rudder,
            rud_col,
            rudder_claim.is_some() || mouse_claim == Some(3),
        );
        let rud_label = if input.rudder > 0.0 {
            format!("RUD {:.0} STBD", input.rudder * 35.0)
        } else if input.rudder < 0.0 {
            format!("RUD {:.0} PORT", -input.rudder * 35.0)
        } else {
            "RUD AMIDSHIPS".to_owned()
        };
        let rd = measure_text(&rud_label, None, (fs * 0.8) as u16, 1.0);
        draw_text(
            &rud_label,
            (rr.x + (rr.w - rd.width) * 0.5).clamp(4.0, sw - rd.width - 4.0),
            rr.y + rr.h + fs * 1.1,
            fs * 0.8,
            rud_col,
        );

        // Boat speed-over-ground and speed-through-water, centred between
        // the dials. STW is SOG relative to the current, not the wind —
        // the reading a paddlewheel/pitot log would give.
        let (v, _) = sim.boat_vel();
        let stw = (v - env.current_vel()).length();
        let sog = format!("SOG {:.2} m/s   STW {:.2} m/s", v.length(), stw);
        let sd = measure_text(&sog, None, fs as u16, 1.0);
        draw_text(&sog, sw * 0.5 - sd.width * 0.5, sa_t + margin + fs, fs, text);

        // Reset button (bottom-right) — the touch/mouse twin of the R key.
        draw_rectangle(
            reset_rect.x,
            reset_rect.y,
            reset_rect.w,
            reset_rect.h,
            Color::from_rgba(10, 20, 30, 170),
        );
        draw_rectangle_lines(reset_rect.x, reset_rect.y, reset_rect.w, reset_rect.h, 2.0, dim);
        let rl = measure_text("RESET", None, fs as u16, 1.0);
        draw_text(
            "RESET",
            reset_rect.x + (reset_rect.w - rl.width) * 0.5,
            reset_rect.y + reset_rect.h * 0.5 + fs * 0.35,
            fs,
            text,
        );

        // Keel editor button (bottom-right, left of RESET) — the
        // touch/mouse twin of the K key.
        draw_rectangle(
            keel_rect.x,
            keel_rect.y,
            keel_rect.w,
            keel_rect.h,
            Color::from_rgba(10, 20, 30, 170),
        );
        draw_rectangle_lines(keel_rect.x, keel_rect.y, keel_rect.w, keel_rect.h, 2.0, dim);
        let kl = measure_text("KEEL", None, fs as u16, 1.0);
        draw_text(
            "KEEL",
            keel_rect.x + (keel_rect.w - kl.width) * 0.5,
            keel_rect.y + keel_rect.h * 0.5 + fs * 0.35,
            fs,
            text,
        );

        // View-mode button (left of KEEL) — the touch/mouse twin of the V
        // key, labelled with the view a press switches TO.
        draw_rectangle(
            view_rect.x,
            view_rect.y,
            view_rect.w,
            view_rect.h,
            Color::from_rgba(10, 20, 30, 170),
        );
        draw_rectangle_lines(view_rect.x, view_rect.y, view_rect.w, view_rect.h, 2.0, dim);
        let view_label = match view {
            ViewMode::TopDown => "3D",
            ViewMode::Chase => "3D+2D",
            ViewMode::Both => "HELM",
            ViewMode::Cockpit => "2D",
        };
        let vl = measure_text(view_label, None, fs as u16, 1.0);
        draw_text(
            view_label,
            view_rect.x + (view_rect.w - vl.width) * 0.5,
            view_rect.y + view_rect.h * 0.5 + fs * 0.35,
            fs,
            text,
        );

        // Centre button (left of VIEW) — the touch/mouse twin of the C key.
        // Only while it has something to undo: a panned-off camera in the
        // top-down view, or an off-axis gaze at the helm.
        let show_center = (view == ViewMode::TopDown && cam_offset.length() > 0.5)
            || (view == ViewMode::Cockpit && r3d.look_active());
        if show_center {
            draw_rectangle(
                center_rect.x,
                center_rect.y,
                center_rect.w,
                center_rect.h,
                Color::from_rgba(10, 20, 30, 170),
            );
            draw_rectangle_lines(
                center_rect.x,
                center_rect.y,
                center_rect.w,
                center_rect.h,
                2.0,
                dim,
            );
            let cl = measure_text("CENTER", None, fs as u16, 1.0);
            draw_text(
                "CENTER",
                center_rect.x + (center_rect.w - cl.width) * 0.5,
                center_rect.y + center_rect.h * 0.5 + fs * 0.35,
                fs,
                text,
            );
        }

        // Hints, bottom-left. Keyboard lines only where a keyboard is
        // likely (wide screens); ASCII only — the built-in font has no
        // arrow glyphs. Indented past the HTML About button (index.html),
        // which owns the bottom-left corner itself (30 px + gaps; the
        // indent is harmless dead space in native builds, which have no
        // HTML layer).
        let mut help: Vec<&str> = vec![
            "left slider = engine, right = rudder; dials set wind & current; pinch/drag = zoom/pan",
        ];
        if sw >= 700.0 {
            help.push("keys: W/S throttle, A/D rudder, Space stop engine, arrows wind");
            help.push("I/K+J/L = current, R = reset, E = keel editor, wheel/+- = zoom, C = centre, V = view");
        }
        let help_x = sa_l + margin + 40.0;
        // On narrow screens the hint line runs under the buttons (they
        // share the bottom edge) — lift the block above them then.
        let buttons_left = if show_center { center_rect.x } else { view_rect.x };
        let help_w = help
            .iter()
            .map(|l| measure_text(l, None, (fs * 0.8) as u16, 1.0).width)
            .fold(0.0, f32::max);
        let help_base = if help_x + help_w > buttons_left - margin {
            keel_rect.y - margin
        } else {
            sh - sa_b - margin
        };
        for (i, line) in help.iter().enumerate() {
            draw_text(
                line,
                help_x,
                help_base - (help.len() - 1 - i) as f32 * fs,
                fs * 0.8,
                dim,
            );
        }

        // --- Keel design editor overlay -----------------------------------
        if editor.active {
            // The editor predates the touch HUD's min_dim/fs/margin-based
            // css-px scaling; this is the same scale factor in that idiom,
            // dpi-free like the rest of the new HUD (macroquad's high_dpi
            // conf + logical measurement already handle that).
            let ui = (min_dim / 980.0).clamp(0.5, 1.0);
            let canvas = Rect::new(
                sw * 0.5 - 300.0 * ui,
                sh * 0.5 - 170.0 * ui,
                600.0 * ui,
                220.0 * ui,
            );
            let layout = EditorLayout::under(canvas, ui);
            match editor.update(canvas, layout) {
                EditorAction::Apply => {
                    design = editor.design();
                    (sim, prev_pos, prev_heading, cur_pos, cur_heading) = respawn(&design);
                    accum = 0.0;
                    // Respawn = camera back on the boat (zoom persists),
                    // chase camera snapped like the R reset.
                    cam_offset = Vec2::ZERO;
                    r3d.snap_to(cur_pos, cur_heading);
                    editor.active = false;
                }
                EditorAction::Cancel => {
                    editor.active = false;
                }
                EditorAction::None => {}
            }
            if editor.active {
                editor.draw(canvas, layout, ui);
            }
        }

        next_frame().await;
    }
}
