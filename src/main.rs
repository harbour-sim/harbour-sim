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
use harbour_sim_core::line::{Anchor, Fitting, Hull, ShoreKind};
use harbour_sim_core::sim::{
    Env, HULL_PTS, InputState, JETTY_HALF_W, PHYSICS_DT, POLE_RADIUS, Sim, cleat_positions,
    head_arc, hill_shore, jetties, marina_shore_len, pole_positions, road_shore, world_bounds,
};
use keel_editor::{EditorAction, EditorLayout, KeelEditor};
use macroquad::prelude::*;
use mooring::{Ctx as MooringCtx, MooringLayout, MooringUi, View};
use settings::{SettingsLayout, SettingsMenu};
use wake::Wake;
use std::sync::atomic::{AtomicU32, Ordering};

mod keel_editor;
mod mooring;
mod settings;
mod wake;

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

/// Fresh `Sim` at the default berth + reset render-interpolation state,
/// used by the R-reset key — never mutate an existing `Sim` in place
/// (determinism rule), always spawn a new one.
fn respawn(design: &BoatDesign) -> (Sim, Vec2, f32, Vec2, f32) {
    let sim = Sim::new_with_design(design);
    let (pos, heading) = sim.boat_pose();
    (sim, pos, heading, pos, heading)
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

/// Offset a polyline to the LEFT of its direction of travel by `d`
/// (per-vertex averaged segment normals — plenty for the shore's gentle
/// bends). The road shore runs SW→NE, so left = inland NW; the hill
/// shore's inland side is its right (pass a negative `d`).
fn offset_polyline(pts: &[Vec2], d: f32) -> Vec<Vec2> {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let din = if i > 0 { (pts[i] - pts[i - 1]).normalize_or_zero() } else { Vec2::ZERO };
            let dout =
                if i + 1 < n { (pts[i + 1] - pts[i]).normalize_or_zero() } else { Vec2::ZERO };
            let t = (din + dout).normalize_or_zero();
            pts[i] + vec2(-t.y, t.x) * d
        })
        .collect()
}

/// Fill the ribbon between two equal-length polylines (a quad strip).
fn draw_strip(a: &[Vec2], b: &[Vec2], w2s: impl Fn(Vec2) -> Vec2, col: Color) {
    for i in 0..a.len().min(b.len()).saturating_sub(1) {
        let (p0, p1) = (w2s(a[i]), w2s(a[i + 1]));
        let (q0, q1) = (w2s(b[i]), w2s(b[i + 1]));
        draw_triangle(p0, p1, q1, col);
        draw_triangle(p0, q1, q0, col);
    }
}

/// One HUD button: dark plate, thin rim, centred label. `on` draws it
/// engaged, for buttons that toggle a mode rather than firing an action.
fn hud_button(r: Rect, label: &str, fs: f32, on: bool) {
    let plate =
        if on { Color::from_rgba(24, 62, 84, 200) } else { Color::from_rgba(10, 20, 30, 170) };
    let rim =
        if on { Color::from_rgba(120, 200, 235, 220) } else { Color::from_rgba(130, 160, 178, 255) };
    draw_rectangle(r.x, r.y, r.w, r.h, plate);
    draw_rectangle_lines(r.x, r.y, r.w, r.h, 2.0, rim);
    let m = measure_text(label, None, fs as u16, 1.0);
    draw_text(
        label,
        r.x + (r.w - m.width) * 0.5,
        r.y + r.h * 0.5 + fs * 0.35,
        fs,
        Color::from_rgba(205, 227, 240, 255),
    );
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

    // --- Static scenery, computed once. sim-core is the single source of
    // truth for all of it: what's drawn IS what collides.
    let jetty_list = jetties();
    let poles = pole_positions();
    let cleats = cleat_positions();
    // Everything a rope can be belayed to, built once from sim-core's own
    // geometry like the rest of the scenery — what is drawn IS what a
    // line can reach.
    let anchors: Vec<Anchor> = cleats
        .iter()
        .map(|&pos| Anchor::Shore { pos, kind: ShoreKind::Cleat })
        .chain(poles.iter().map(|&pos| Anchor::Shore { pos, kind: ShoreKind::Pole }))
        .collect();
    let road = road_shore();
    let hill = hill_shore();
    let head = head_arc();
    let n_marina = marina_shore_len();
    let (bmin, bmax) = world_bounds();
    // Land fills reach far enough inland to cover the whole view whenever
    // the camera sits against the world clamp.
    let road_land = offset_polyline(&road, 260.0);
    let road_apron = offset_polyline(&road, 1.8);
    let hill_rock = offset_polyline(&hill, -2.0);
    let hill_land = offset_polyline(&hill, -260.0);
    // The rounded bay head's land rings: scaled copies of the arc about
    // its chord centre (a silt-green waterline band, then grass beyond).
    let head_center = (road[0] + hill[0]) * 0.5;
    let head_ring = |k: f32| -> Vec<Vec2> {
        head.iter().map(|&p| head_center + (p - head_center) * k).collect()
    };
    let head_silt = head_ring(1.18);
    let head_land = head_ring(5.0);
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
    // Cosmetic churned-water trail (src/wake.rs) — render-only state.
    let mut wake = Wake::new();
    let (mut prev_pos, mut prev_heading) = sim.boat_pose();
    let (mut cur_pos, mut cur_heading) = (prev_pos, prev_heading);

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
    // Mooring lines: the mode, its handles, and the finger holding one of
    // them. One claim covers every mooring gesture (leading a line,
    // hauling, the speed setting) — they are mutually exclusive by hand.
    let mut mooring = MooringUi::new();
    // Configuration lives in its own menu, not in the play HUD.
    let mut settings = SettingsMenu::new();
    let mut mooring_touch: Option<(u64, Vec2)> = None;
    // Last frame's camera centre, the companion to `last_scale`: input
    // runs before the camera block, so world↔screen hit-testing uses the
    // view the player was actually looking at when they pressed.
    let mut last_cam = Vec2::ZERO;
    // Last frame's INTERPOLATED pose, the companion to `last_cam`: what
    // the player saw when they reached for a fairlead.
    let mut last_boat = (prev_pos, prev_heading);
    // The moored fleet's live poses, refreshed once a frame into a reused
    // buffer — they lie to real ropes now, so where they are is sim state
    // rather than something the renderer can read off `moored_boats()`.
    let mut moored_poses: Vec<(Vec2, f32)> = sim.moored_poses().collect();
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
            // The HUD touch/mouse state (prev_touch_ids and all claims)
            // freezes while the editor is open because the input block is
            // skipped. Snapshot the current touches so the first frame
            // after the editor closes sees every held finger as "already
            // known" and doesn't false-grab dials/sliders. Claims are
            // cleared unconditionally — a finger that was dragging a
            // slider before the editor opened is not continuing that drag.
            prev_touch_ids = touches().iter().map(|t| t.id).collect();
            wind_claim = None;
            current_claim = None;
            throttle_claim = None;
            rudder_claim = None;
            mouse_claim = None;
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
        // LINES button, left of KEEL — the touch/mouse twin of the T key,
        // and the only way into mooring mode on a touch-only device (the
        // same reason KEEL exists).
        let lines_w = fs * 4.8;
        let lines_rect = Rect::new(
            keel_rect.x - margin - lines_w,
            sh - sa_b - margin - keel_h,
            lines_w,
            keel_h,
        );
        // Settings button, left of LINES — the touch/mouse twin of the O
        // key. Drawn as a gear rather than a word: the bottom row is
        // already four buttons wide on a phone, and configuration is the
        // one thing here that does not need a label to be found.
        let gear_rect = Rect::new(
            lines_rect.x - margin - keel_h,
            sh - sa_b - margin - keel_h,
            keel_h,
            keel_h,
        );
        // CENTER button, left of the settings gear — the touch/mouse twin
        // of the C key.
        // Only shown (and only hittable) while the camera is panned away
        // from the boat, so the button row stays uncluttered otherwise.
        let center_w = fs * 5.2;
        let center_rect = Rect::new(
            gear_rect.x - margin - center_w,
            sh - sa_b - margin - keel_h,
            center_w,
            keel_h,
        );
        // Mooring panel, stacked above the button row: the tend controls
        // for the selected line, nearest the thumb. Positions are fixed
        // whether or not a line is selected, so the buttons appear in
        // place instead of shuffling the row around under a finger.
        let tend_y = sh - sa_b - margin * 2.0 - keel_h * 2.0;
        let cast_w = fs * 5.8;
        let slack_w = fs * 5.0;
        let haul_w = fs * 4.4;
        let cast_rect = Rect::new(sw - sa_r - margin - cast_w, tend_y, cast_w, keel_h);
        let slack_rect =
            Rect::new(cast_rect.x - margin * 0.6 - slack_w, tend_y, slack_w, keel_h);
        let haul_rect = Rect::new(slack_rect.x - margin * 0.6 - haul_w, tend_y, haul_w, keel_h);
        // The mooring status line sits above the tend row.
        let mooring_status_y = tend_y - fs * 0.6;
        let mooring_layout = MooringLayout { haul: haul_rect, slack: slack_rect, cast: cast_rect };
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

        if !editor.active && !settings.active {
            let mut do_reset = is_key_pressed(KeyCode::R);
            let mut do_open_editor = false;
            let mut do_center = is_key_pressed(KeyCode::C);
            let mut do_toggle_lines = is_key_pressed(KeyCode::T);
            let mut do_open_settings = is_key_pressed(KeyCode::O);

            // Everything the mooring UI needs of the world this frame,
            // against LAST frame's camera and interpolated pose — input
            // runs ahead of the camera block, so this is the view the
            // player was actually looking at when they pressed.
            let mooring_ctx = MooringCtx {
                view: View { cam: last_cam, scale: last_scale, sw, sh },
                boat_pos: last_boat.0,
                boat_heading: last_boat.1,
                moored: &moored_poses,
                anchors: &anchors,
                lines: sim.lines(),
                broken: sim.broken_fittings(),
                reach: settings.reach,
                layout: mooring_layout,
            };

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
                    if mooring_touch.is_some_and(|(id, _)| id == t.id) {
                        mooring.clear_grabs();
                        mooring_touch = None;
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
                    } else if lines_rect.contains(p) {
                        do_toggle_lines = true;
                    } else if gear_rect.contains(p) {
                        do_open_settings = true;
                    } else if cam_offset.length() > 0.5 && center_rect.contains(p) {
                        do_center = true;
                    } else if mooring_touch.is_none() && mooring.press(p, &mooring_ctx) {
                        mooring_touch = Some((t.id, p));
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
                } else if mooring_touch.is_some_and(|(id, _)| id == t.id) {
                    mooring.hold(p, &mooring_ctx);
                    mooring_touch = Some((t.id, p));
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
            // A lifted finger releases its mooring gesture where it was
            // last seen — `touches()` drops the id without reporting a
            // final position, and where the rope was let go is exactly
            // what decides whether it lands on a cleat.
            if let Some((id, at)) = mooring_touch
                && !cur_ids.contains(&id)
            {
                mooring.release(at, &mooring_ctx);
                mooring_touch = None;
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
                        && mooring_touch.is_none_or(|(id, _)| id != t.id)
                })
                .map(|t| (t.id, t.position / dpi))
                .collect();
            match free[..] {
                [(id, p)] => {
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
                [(ida, pa), (idb, pb)] => {
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
                } else if lines_rect.contains(mp) {
                    do_toggle_lines = true;
                } else if gear_rect.contains(mp) {
                    do_open_settings = true;
                } else if cam_offset.length() > 0.5 && center_rect.contains(mp) {
                    do_center = true;
                } else if mooring.press(mp, &mooring_ctx) {
                    mouse_claim = Some(5);
                } else {
                    // Anywhere on the water: drag to pan (claim 4).
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
                    Some(4) => {
                        let d = mp - pan_mouse_prev;
                        cam_offset.x -= d.x / last_scale;
                        cam_offset.y += d.y / last_scale; // screen y is inverted
                        pan_mouse_prev = mp;
                    }
                    Some(5) => mooring.hold(mp, &mooring_ctx),
                    _ => {}
                }
            } else {
                if mouse_claim == Some(5) {
                    mooring.release(mp, &mooring_ctx);
                }
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
            if wheel_y != 0.0 {
                let step = if wheel_y.abs() >= 40.0 { wheel_y / 240.0 } else { wheel_y * 0.25 };
                zoom *= 2.0f32.powf(step.clamp(-0.6, 0.6));
            }
            if is_key_down(KeyCode::Equal) || is_key_down(KeyCode::KpAdd) {
                zoom *= 2.0f32.powf(dt);
            }
            if is_key_down(KeyCode::Minus) || is_key_down(KeyCode::KpSubtract) {
                zoom *= 2.0f32.powf(-dt);
            }

            if do_open_settings {
                settings.open();
                // Same claim reset as the keel editor: the fingers that
                // opened this are not driving dials or ropes.
                prev_touch_ids = touches().iter().map(|t| t.id).collect();
                wind_claim = None;
                current_claim = None;
                throttle_claim = None;
                rudder_claim = None;
                mouse_claim = None;
                mooring.clear_grabs();
                mooring_touch = None;
            }
            if do_toggle_lines {
                mooring.active = !mooring.active;
                // Leaving the mode drops anything in hand; entering it
                // starts clean too, so a stale armed fairlead from last
                // time can't complete on the first tap.
                mooring.clear_grabs();
                mooring_touch = None;
                if !mooring.active {
                    mooring.selected = None;
                }
            }
            if do_center {
                cam_offset = Vec2::ZERO;
            }
            if do_reset {
                // Fresh Sim per run — never reuse one (determinism rule).
                (sim, prev_pos, prev_heading, cur_pos, cur_heading) = respawn(&design);
                accum = 0.0;
                // Helm and engine come back neutral with the fresh boat;
                // the environment deliberately persists (same as always).
                input = InputState::NEUTRAL;
                // A fresh boat gets the camera back too (zoom persists).
                cam_offset = Vec2::ZERO;
                // ...and a clean slate of water: a surviving wake would
                // draw a streak from wherever the boat was to the spawn.
                wake.clear();
                // ...and no ropes: `respawn` builds a fresh `Sim`, so the
                // lines are gone with it. Drop every one of the UI's
                // references to them, not just the grab and selection —
                // a queued order or a stale `passing` id would otherwise
                // reach into a sim that never issued them.
                mooring.reset();
                mooring_touch = None;
            }
            if do_open_editor {
                editor.load_design(&design);
                editor.active = true;
                mooring.clear_grabs();
                mooring_touch = None;
                // Drop stale claims — these fingers are opening the
                // editor, not driving dials/sliders.
                wind_claim = None;
                current_claim = None;
                throttle_claim = None;
                rudder_claim = None;
                mouse_claim = None;
            }

            // --- Fixed-timestep physics with render interpolation. ---------
            accum += dt;
            while accum >= PHYSICS_DT {
                prev_pos = cur_pos;
                prev_heading = cur_heading;
                // One line order per tick, drained from the mooring UI:
                // queued one-shots first, then a held HAUL/SLACK repeated
                // for as long as it is held.
                input.line = mooring.next_command(sim.lines());
                input.crew = settings.crew();
                sim.tick(&env, &input);
                mooring.report_failures(sim.line_failures());
                (cur_pos, cur_heading) = sim.boat_pose();
                accum -= PHYSICS_DT;
            }
            input.line = None;
            mooring.prune(sim.lines());
            moored_poses.clear();
            moored_poses.extend(sim.moored_poses());

            // Cosmetic wake, advanced on the FRAME's dt (it is render
            // state, not physics) and only while the sim is running —
            // the editor freeze holds the water still too.
            wake.update(dt, &sim, &design, &env);
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
        last_cam = vec2(cam_x, cam_y);
        last_boat = (pos, heading);
        let w2s = |p: Vec2| -> Vec2 {
            vec2(sw * 0.5 + (p.x - cam_x) * scale, sh * 0.5 - (p.y - cam_y) * scale)
        };
        // Cheap visibility cull for the marina's many static props.
        let vis_r = vec2(vis_hw, vis_hh).length();
        let visible = |p: Vec2, r: f32| (p - vec2(cam_x, cam_y)).length() < vis_r + r;

        // --- Water -------------------------------------------------------
        clear_background(Color::from_rgba(12, 38, 54, 255));
        let water = Color::from_rgba(16, 48, 66, 255);
        // One strip covers the channel AND the widening sea past the
        // entrance (the shore polylines continue out along the sea coast);
        // a fan over the head arc fills the rounded bay head.
        draw_strip(&road, &hill, w2s, water);
        for i in 1..head.len() - 1 {
            draw_triangle(w2s(head[0]), w2s(head[i]), w2s(head[i + 1]), water);
        }

        // Cosmetic ripples: short streaks drifting with the current (and a
        // touch of wind), wrapped over the world box. Purely render-side —
        // the land fills drawn next cover the strays that land ashore.
        let t = get_time() as f32;
        let drift = env.current_vel() + env.wind_vel() * 0.02;
        let (bw, bh) = (bmax.x - bmin.x, bmax.y - bmin.y);
        for i in 0u32..220 {
            let h = i.wrapping_mul(2654435761);
            let fx = (h & 0xffff) as f32 / 65535.0;
            let fy = ((h >> 16) & 0xffff) as f32 / 65535.0;
            let x = (fx * bw + drift.x * t).rem_euclid(bw) + bmin.x;
            let y = (fy * bh + drift.y * t).rem_euclid(bh) + bmin.y;
            if !visible(vec2(x, y), 2.0) {
                continue;
            }
            let a = w2s(vec2(x, y));
            let b = w2s(vec2(x + 1.4, y));
            draw_line(a.x, a.y, b.x, b.y, 1.5, Color::from_rgba(120, 170, 190, 26));
        }

        // The boat's own churned-water trail, on the water with the
        // ripples and under the land fills that cover the strays.
        wake.draw(w2s, scale, visible);

        // --- Shore (Hinsholmen look: road side NW, wooded hill SE) --------
        // Deterministic scatter helper for trees: same hash idiom as the
        // ripples, but static (no time term) — pure scenery.
        let hash01 = |i: u32, salt: u32| -> (f32, f32, f32) {
            let h = i.wrapping_add(salt).wrapping_mul(2654435761);
            (
                (h & 0xffff) as f32 / 65535.0,
                ((h >> 16) & 0x7fff) as f32 / 32767.0,
                ((h >> 8) & 0xff) as f32 / 255.0,
            )
        };
        let grass = Color::from_rgba(58, 88, 52, 255);
        let tree_a = Color::from_rgba(40, 70, 40, 255);
        let tree_b = Color::from_rgba(48, 80, 44, 255);
        let rock = Color::from_rgba(98, 100, 96, 255);
        let forest = Color::from_rgba(36, 60, 40, 255);
        // Road (dock-carrying) shore: grass from the waterline inland,
        // with the concrete quay apron drawn over it along the MARINA
        // stretch only — past the entrance the coast runs wild.
        draw_strip(&road, &road_land, w2s, grass);
        draw_strip(
            &road[..n_marina],
            &road_apron[..n_marina],
            w2s,
            Color::from_rgba(126, 128, 126, 255),
        );
        for i in 0..road.len() - 1 {
            let (sa, sb) = (road[i], road[i + 1]);
            let seg = sb - sa;
            let n = vec2(-seg.y, seg.x).normalize_or_zero(); // inland
            for k in 0u32..6 {
                let (f, d, r) = hash01(i as u32 * 8 + k, 11);
                let p = sa + seg * f + n * (4.0 + d * 22.0);
                if !visible(p, 3.0) {
                    continue;
                }
                let sp = w2s(p);
                let col = if (i + k as usize).is_multiple_of(2) { tree_a } else { tree_b };
                draw_circle(sp.x, sp.y, (0.8 + r * 1.0) * scale, col);
            }
        }
        // Hill (SE) shore: a rocky waterline, forest behind.
        draw_strip(&hill_rock, &hill, w2s, rock);
        draw_strip(&hill_land, &hill_rock, w2s, forest);
        for i in 0..hill.len() - 1 {
            let (sa, sb) = (hill[i], hill[i + 1]);
            let seg = sb - sa;
            let n = vec2(seg.y, -seg.x).normalize_or_zero(); // inland (SE)
            for k in 0u32..8 {
                let (f, d, r) = hash01(i as u32 * 8 + k, 97);
                let p = sa + seg * f + n * (2.5 + d * 26.0);
                if !visible(p, 3.0) {
                    continue;
                }
                let sp = w2s(p);
                let col = if (i + k as usize).is_multiple_of(2) { tree_a } else { tree_b };
                draw_circle(sp.x, sp.y, (0.9 + r * 1.2) * scale, col);
            }
        }
        // The rounded bay head: a silted-green shallow band right at the
        // waterline, grass beyond (the land rings are scaled copies of
        // the same arc the wall collider uses).
        draw_strip(&head, &head_silt, w2s, Color::from_rgba(74, 96, 74, 255));
        draw_strip(&head_silt, &head_land, w2s, grass);
        // The skerry line closing the open sea (the boundary polyline's
        // segment between the two coasts' far ends): a chain of rocky
        // islets along the world's edge.
        {
            let (a, b) = (road[road.len() - 1], hill[hill.len() - 1]);
            for i in 0u32..14 {
                let (f, d, r) = hash01(i, 53);
                let p = a + (b - a) * ((i as f32 + 0.5 + (f - 0.5) * 0.6) / 14.0)
                    + (b - a).normalize_or_zero().perp() * ((d - 0.5) * 6.0);
                if !visible(p, 8.0) {
                    continue;
                }
                let sp = w2s(p);
                draw_circle(sp.x, sp.y, (2.0 + r * 3.5) * scale, rock);
            }
        }

        // --- Pontoon jetties ----------------------------------------------
        let deck = Color::from_rgba(168, 162, 148, 255);
        let deck_seam = Color::from_rgba(146, 140, 126, 255);
        let deck_edge = Color::from_rgba(110, 104, 92, 255);
        for j in &jetty_list {
            let mid = j.root + j.dir * (j.len * 0.5);
            if !visible(mid, j.len * 0.5 + 6.0) {
                continue;
            }
            let side = j.side();
            let (r0, r1) = (j.root + side * JETTY_HALF_W, j.root - side * JETTY_HALF_W);
            let (t0, t1) =
                (r0 + j.dir * j.len, r1 + j.dir * j.len);
            let (sr0, sr1, st0, st1) = (w2s(r0), w2s(r1), w2s(t0), w2s(t1));
            draw_triangle(sr0, sr1, st1, deck);
            draw_triangle(sr0, st1, st0, deck);
            // Transverse deck seams every couple of metres.
            let mut d = 2.0;
            while d < j.len {
                let l = w2s(r0 + j.dir * d);
                let r = w2s(r1 + j.dir * d);
                draw_line(l.x, l.y, r.x, r.y, 1.0, deck_seam);
                d += 2.0;
            }
            draw_line(sr0.x, sr0.y, st0.x, st0.y, 1.5, deck_edge);
            draw_line(sr1.x, sr1.y, st1.x, st1.y, 1.5, deck_edge);
            draw_line(st0.x, st0.y, st1.x, st1.y, 1.5, deck_edge);
        }
        // Cleats along the faces — real marina furniture, and what a bow
        // or breast line belays to. Positions come from sim-core, so a
        // stud you can see is a stud you can reach.
        let cleat_col = Color::from_rgba(84, 80, 72, 255);
        for c in &cleats {
            if !visible(*c, 1.0) {
                continue;
            }
            let sc = w2s(*c);
            let r = (0.22 * scale).max(1.2);
            // One that has been pulled off the pontoon leaves its holes
            // behind, and is not offered as an anchor any more.
            if sim.broken_fittings().contains(&Fitting::Shore(*c)) {
                draw_circle_lines(sc.x, sc.y, r * 1.6, 2.0, mooring::TORN_OUT);
            } else {
                draw_circle(sc.x, sc.y, r, cleat_col);
            }
        }

        // --- Moored boats -------------------------------------------------
        // Dynamic in sim-core since 2026-08-20, so they are drawn at the
        // pose the sim gives them rather than at the fixed berth
        // geometry — a boat you lean on moves, and comes back. Their
        // mooring lines are real `Line`s and were drawn above with
        // everyone else's, so there is no decorative rigging left here.
        let moored_line_col = Color::from_rgba(46, 48, 54, 255);
        let moored_fills = [
            Color::from_rgba(226, 222, 208, 255),
            Color::from_rgba(212, 216, 220, 255),
            Color::from_rgba(230, 224, 212, 255),
            Color::from_rgba(206, 200, 188, 255),
        ];
        for (bi, &(mpos, mheading)) in moored_poses.iter().enumerate() {
            if !visible(mpos, 16.0) {
                continue;
            }
            let (mc, ms) = (mheading.cos(), mheading.sin());
            let ml = |lx: f32, ly: f32| -> Vec2 {
                w2s(mpos + vec2(lx * mc - ly * ms, lx * ms + ly * mc))
            };

            // Hull: same outline as the player's, quieter deck detail.
            let fill = moored_fills[bi % moored_fills.len()];
            let m0 = ml(HULL_PTS[0].0, HULL_PTS[0].1);
            for i in 1..HULL_PTS.len() - 1 {
                let m1 = ml(HULL_PTS[i].0, HULL_PTS[i].1);
                let m2 = ml(HULL_PTS[i + 1].0, HULL_PTS[i + 1].1);
                draw_triangle(m0, m1, m2, fill);
            }
            for (i, &(ax, ay)) in HULL_PTS.iter().enumerate() {
                let a = ml(ax, ay);
                let (bx2, by2) = HULL_PTS[(i + 1) % HULL_PTS.len()];
                let b = ml(bx2, by2);
                draw_line(a.x, a.y, b.x, b.y, (0.12 * scale).max(1.0), moored_line_col);
            }
            if bi % 4 == 3 {
                // A motor cruiser among the sailboats: long cabin, no rig.
                let cab = [(-3.4, 1.2), (1.6, 1.2), (1.6, -1.2), (-3.4, -1.2)];
                let c0 = ml(cab[0].0, cab[0].1);
                draw_triangle(c0, ml(cab[1].0, cab[1].1), ml(cab[2].0, cab[2].1), Color::from_rgba(198, 202, 206, 255));
                draw_triangle(c0, ml(cab[2].0, cab[2].1), ml(cab[3].0, cab[3].1), Color::from_rgba(198, 202, 206, 255));
                for i in 0..4 {
                    let a = ml(cab[i].0, cab[i].1);
                    let b = ml(cab[(i + 1) % 4].0, cab[(i + 1) % 4].1);
                    draw_line(a.x, a.y, b.x, b.y, (0.08 * scale).max(1.0), moored_line_col);
                }
            } else {
                let ch = [(-2.6, 1.0), (0.3, 1.0), (0.3, -1.0), (-2.6, -1.0)];
                let c0 = ml(ch[0].0, ch[0].1);
                draw_triangle(c0, ml(ch[1].0, ch[1].1), ml(ch[2].0, ch[2].1), Color::from_rgba(203, 208, 212, 255));
                draw_triangle(c0, ml(ch[2].0, ch[2].1), ml(ch[3].0, ch[3].1), Color::from_rgba(203, 208, 212, 255));
                for i in 0..4 {
                    let a = ml(ch[i].0, ch[i].1);
                    let b = ml(ch[(i + 1) % 4].0, ch[(i + 1) % 4].1);
                    draw_line(a.x, a.y, b.x, b.y, (0.08 * scale).max(1.0), moored_line_col);
                }
                let mast = ml(0.6, 0.0);
                let boom = ml(-2.0, 0.0);
                draw_line(mast.x, mast.y, boom.x, boom.y, (0.08 * scale).max(1.0), moored_line_col);
                draw_circle(mast.x, mast.y, (0.18 * scale).max(1.5), moored_line_col);
            }
        }

        // --- Mooring poles (drawn over the lines belayed to them) ---------
        let pole_fill = Color::from_rgba(92, 64, 40, 255);
        let pole_rim = Color::from_rgba(50, 34, 20, 255);
        for p in &poles {
            if !visible(*p, 1.0) {
                continue;
            }
            let sp = w2s(*p);
            let r = (POLE_RADIUS * scale).max(2.0);
            draw_circle(sp.x, sp.y, r + 1.0, pole_rim);
            draw_circle(sp.x, sp.y, r, pole_fill);
        }

        // --- Mooring lines ------------------------------------------------
        // Drawn before the hull so a rope's end tucks under the deck edge
        // at its fairlead (same ordering trick as the rudder blade), and
        // always — once a line is out it is part of the world, not part
        // of the LINES-mode overlay.
        let mooring_ctx = MooringCtx {
            view: View { cam: vec2(cam_x, cam_y), scale, sw, sh },
            boat_pos: pos,
            boat_heading: heading,
            moored: &moored_poses,
            anchors: &anchors,
            lines: sim.lines(),
            broken: sim.broken_fittings(),
            reach: settings.reach,
            layout: mooring_layout,
        };
        mooring::draw_ropes(&mooring_ctx, mooring.selected, visible);

        // --- Boat --------------------------------------------------------
        let (c, s) = (heading.cos(), heading.sin());
        let bl = |lx: f32, ly: f32| -> Vec2 { w2s(pos + vec2(lx * c - ly * s, lx * s + ly * c)) };
        let hull_fill = Color::from_rgba(230, 226, 212, 255);
        let hull_line = Color::from_rgba(40, 42, 48, 255);

        // Blade angle for the rudder visual and the wash direction (same
        // formula as sim-core: positive = trailing edge to port).
        let blade = -input.rudder * 35f32.to_radians();

        // Cosmetic prop wash: foam streaks driven by the SPOOLED engine
        // (sim.engine(), so they fade in/out with the lag, not the lever).
        // Render-only — reads sim state, feeds nothing back (get_time is
        // allowed here like the ripples). Ahead the slipstream leaves the
        // stern along the deflected blade; astern it boils forward along
        // both quarters — the two directions you'd see from a quay.
        let engine = sim.engine();
        if engine.abs() > 0.05 {
            for i in 0u32..8 {
                let h = i.wrapping_mul(2654435761);
                let fy = ((h & 0xffff) as f32 / 65535.0) - 0.5;
                let phase = ((h >> 16) & 0xffff) as f32 / 65535.0;
                let ph = (t * 1.8 + phase).fract();
                let alpha = engine.abs().min(1.0) * (1.0 - ph) * 0.5;
                let foam = Color::new(0.75, 0.88, 0.92, alpha);
                let (a, b) = if engine > 0.0 {
                    let dir = vec2(-blade.cos(), blade.sin());
                    let start = vec2(-6.0, fy * 1.1);
                    let p = start + dir * (ph * (1.5 + 2.2 * engine));
                    (bl(p.x, p.y), bl(p.x + dir.x * 0.7, p.y + dir.y * 0.7))
                } else {
                    let side_y = if i % 2 == 0 { 1.0 } else { -1.0 };
                    let start = vec2(-5.2, side_y * (1.4 + 0.6 * fy.abs()));
                    let p = start + vec2(ph * (2.0 - 2.5 * engine), 0.0);
                    (bl(p.x, p.y), bl(p.x + 0.6, p.y))
                };
                draw_line(a.x, a.y, b.x, b.y, (0.14 * scale).max(1.0), foam);
            }
        }

        // Rudder blade: stock at the ACTIVE DESIGN's blade position (each
        // preset carries its real boat's rudder — see `RudderDesign` in
        // boat.rs; same values the physics uses), drawn BEFORE the hull
        // fill so the root reads as under the counter and only the swung
        // part shows past it. Stock at the blade's leading edge, the
        // drawn line is the chord.
        let stock_x = design.rudder.x + design.rudder.chord / 2.0;
        let te = vec2(
            stock_x - design.rudder.chord * blade.cos(),
            design.rudder.chord * blade.sin(),
        );
        let rp = bl(stock_x, 0.0);
        let tep = bl(te.x, te.y);
        draw_line(rp.x, rp.y, tep.x, tep.y, (0.16 * scale).max(1.5), hull_line);

        let p0 = bl(HULL_PTS[0].0, HULL_PTS[0].1);
        for i in 1..HULL_PTS.len() - 1 {
            let p1 = bl(HULL_PTS[i].0, HULL_PTS[i].1);
            let p2 = bl(HULL_PTS[i + 1].0, HULL_PTS[i + 1].1);
            draw_triangle(p0, p1, p2, hull_fill);
        }
        for (i, &(ax, ay)) in HULL_PTS.iter().enumerate() {
            let a = bl(ax, ay);
            let (bx2, by2) = HULL_PTS[(i + 1) % HULL_PTS.len()];
            let b = bl(bx2, by2);
            draw_line(a.x, a.y, b.x, b.y, (0.18 * scale).max(1.0), hull_line);
        }
        // Deck details for the current (and, for now, only) modeled ship
        // type — a small cruising sailboat: foredeck lines, coachroof,
        // cockpit, sprayhood, mast + boom (rendered even with the sail
        // furled/down — see Simulation model in CLAUDE.md; the rig is
        // cosmetic, not a physics input). A future second ship type would
        // get its own rendering branch alongside this one.
        let d1 = bl(3.2, 0.0);
        let d2a = bl(4.2, 1.2);
        let d2b = bl(4.2, -1.2);
        draw_line(d2a.x, d2a.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);
        draw_line(d2b.x, d2b.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);

        // Coachroof (cabin trunk): lower and narrower than full beam — side
        // decks stay clear either side for walking forward.
        let ch = [(-2.6, 1.0), (0.3, 1.0), (0.3, -1.0), (-2.6, -1.0)];
        let ch0 = bl(ch[0].0, ch[0].1);
        draw_triangle(ch0, bl(ch[1].0, ch[1].1), bl(ch[2].0, ch[2].1), Color::from_rgba(205, 210, 214, 255));
        draw_triangle(ch0, bl(ch[2].0, ch[2].1), bl(ch[3].0, ch[3].1), Color::from_rgba(205, 210, 214, 255));
        for i in 0..4 {
            let a = bl(ch[i].0, ch[i].1);
            let b = bl(ch[(i + 1) % 4].0, ch[(i + 1) % 4].1);
            draw_line(a.x, a.y, b.x, b.y, (0.1 * scale).max(1.0), hull_line);
        }

        // Cockpit: open well aft of the coachroof, outline only (nothing to
        // fill — it's a recess, not a structure).
        let cp = [(-2.6, 0.9), (-4.3, 0.9), (-4.3, -0.9), (-2.6, -0.9)];
        for i in 0..4 {
            let a = bl(cp[i].0, cp[i].1);
            let b = bl(cp[(i + 1) % 4].0, cp[(i + 1) % 4].1);
            draw_line(a.x, a.y, b.x, b.y, (0.1 * scale).max(1.0), hull_line);
        }

        // Sprayhood: a small hood over the companionway at the coachroof's
        // aft edge, raked to a point forward — this is the actual source
        // of the bow/stern windage asymmetry in sim-core: a headwind meets
        // this point and deflects, a following wind finds the open aft
        // mouth instead and scoops into it.
        let sh_front = bl(-2.2, 0.0);
        let sh_l = bl(-3.0, 0.8);
        let sh_r = bl(-3.0, -0.8);
        draw_triangle(sh_front, sh_l, sh_r, Color::from_rgba(70, 110, 130, 255));

        // Mast (stepped forward of the coachroof) + boom laid along the
        // centreline — both present even with the sail furled/down.
        let mast = bl(0.6, 0.0);
        let boom_end = bl(-2.0, 0.0);
        draw_line(mast.x, mast.y, boom_end.x, boom_end.y, (0.08 * scale).max(1.0), hull_line);
        draw_circle(mast.x, mast.y, (0.18 * scale).max(1.5), hull_line);

        // --- HUD ---------------------------------------------------------
        // Mooring handles sit over everything they attach to.
        mooring::draw_handles(&mooring, &mooring_ctx);

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
        hud_button(reset_rect, "RESET", fs, false);

        // Keel editor button (bottom-right, left of RESET) — the
        // touch/mouse twin of the E key.
        hud_button(keel_rect, "KEEL", fs, editor.active);

        // Mooring lines button (left of KEEL) — the touch/mouse twin of
        // the T key, and the only way into the mode without a keyboard.
        hud_button(lines_rect, "LINES", fs, mooring.active);

        // Settings gear (left of LINES) — the touch/mouse twin of the O
        // key. Drawn from primitives: the built-in font is ASCII only, so
        // there is no gear glyph to set.
        hud_button(gear_rect, "", fs, settings.active);
        {
            let c = vec2(gear_rect.x + gear_rect.w * 0.5, gear_rect.y + gear_rect.h * 0.5);
            let r = gear_rect.h * 0.26;
            for k in 0..6 {
                let a = k as f32 * std::f32::consts::TAU / 6.0;
                let (sn, cs) = a.sin_cos();
                draw_line(
                    c.x + cs * r * 0.9,
                    c.y + sn * r * 0.9,
                    c.x + cs * r * 1.5,
                    c.y + sn * r * 1.5,
                    2.5,
                    text,
                );
            }
            draw_circle_lines(c.x, c.y, r, 2.5, text);
            draw_circle(c.x, c.y, r * 0.34, text);
        }

        // Mooring panel: only while the mode is open. The tend controls
        // for the selected rope sit nearest the thumb, the line-handling
        // speed setting above them.
        if mooring.active {
            let selected = mooring.selected.and_then(|id| sim.lines().iter().find(|l| l.id == id));
            // The tend controls follow the SELECTION, never the status
            // line: a transient note must not make live buttons vanish
            // from under a thumb.
            if selected.is_some() {
                hud_button(haul_rect, "HAUL", fs, false);
                hud_button(slack_rect, "SLACK", fs, false);
                hud_button(cast_rect, "CAST OFF", fs, false);
            }
            // One status line, with a refused order's reason taking
            // priority for a couple of seconds — an order that does
            // nothing without saying why is the worst kind.
            if let Some(msg) = mooring.note() {
                draw_text(
                    msg,
                    haul_rect.x,
                    mooring_status_y,
                    fs * 0.85,
                    Color::from_rgba(255, 150, 120, 255),
                );
            } else if let Some(l) = selected {
                // Both ends from the render pose: mixing in the sim's
                // un-interpolated one makes the number disagree with the
                // rope that is drawn, and flickers it across the 2 cm
                // slack threshold between frames.
                let dist = (mooring_ctx.anchor_pos(l.anchor)
                    - mooring_ctx.fairlead_of(l.hull, l.fairlead))
                .length();
                let slack = l.scope - dist;
                let state = if !l.is_fast() {
                    "going ashore".to_string()
                } else if slack > 0.02 {
                    format!("{slack:.1} m slack")
                } else {
                    format!("{:.2} kN", l.tension / 1000.0)
                };
                let label = format!("{} line, {:.1} m - {state}", l.fairlead.label(), l.scope);
                draw_text(&label, haul_rect.x, mooring_status_y, fs * 0.85, text);
            } else {
                let hint = if sim.lines().iter().all(|l| l.hull != Hull::Player) {
                    "drag from a fairlead to a cleat or pole"
                } else {
                    "tap a rope to haul, slack or cast it off"
                };
                draw_text(hint, haul_rect.x, mooring_status_y, fs * 0.85, dim);
            }
        }

        // Centre-on-boat button (left of LINES) — only while the camera is
        // panned off the boat; the touch/mouse twin of the C key.
        if cam_offset.length() > 0.5 {
            hud_button(center_rect, "CENTER", fs, false);
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
            help.push(
                "I/K+J/L = current, R = reset, E = keel, T = lines, O = settings, C = centre",
            );
        }
        let help_x = sa_l + margin + 40.0;
        // On narrow screens the hint line runs under the buttons (they
        // share the bottom edge) — lift the block above them then.
        // The LEFTMOST button of the row, so the help text clears all of
        // them: LINES and the settings gear both sit left of KEEL.
        let buttons_left = if cam_offset.length() > 0.5 { center_rect.x } else { gear_rect.x };
        let help_w = help
            .iter()
            .map(|l| measure_text(l, None, (fs * 0.8) as u16, 1.0).width)
            .fold(0.0, f32::max);
        let help_base = if help_x + help_w > buttons_left - margin {
            keel_rect.y - margin
        } else {
            sh - sa_b - margin
        };
        // ...and in mooring mode the panel occupies that same strip above
        // the buttons, so the help block steps up clear of the panel AND
        // of the status line drawn just above it.
        let help_base =
            if mooring.active { help_base.min(mooring_status_y - fs * 1.1) } else { help_base };
        for (i, line) in help.iter().enumerate() {
            draw_text(
                line,
                help_x,
                help_base - (help.len() - 1 - i) as f32 * fs,
                fs * 0.8,
                dim,
            );
        }

        // --- Settings overlay ---------------------------------------------
        // Freezes the game while open (input AND the physics tick), the
        // same rule as the keel editor — which is what lets it reuse keys
        // and take presses without fighting the HUD's touch claims.
        if settings.active {
            let layout = SettingsLayout::centred(sw, sh, fs);
            if settings.update(&layout) {
                settings.active = false;
                prev_touch_ids = touches().iter().map(|t| t.id).collect();
                wind_claim = None;
                current_claim = None;
                throttle_claim = None;
                rudder_claim = None;
                mouse_claim = None;
            }
            if settings.active {
                settings.draw(&layout, sw, sh);
            }
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
                    sim = sim.new_continuing(&design);
                    let (pos, heading) = sim.boat_pose();
                    prev_pos = pos;
                    prev_heading = heading;
                    cur_pos = pos;
                    cur_heading = heading;
                    accum = 0.0;

                    editor.active = false;
                    // Same claim reset as the E-key path — see comment there.
                    prev_touch_ids = touches().iter().map(|t| t.id).collect();
                    wind_claim = None;
                    current_claim = None;
                    throttle_claim = None;
                    rudder_claim = None;
                    mouse_claim = None;
                }
                EditorAction::Cancel => {
                    editor.active = false;
                    prev_touch_ids = touches().iter().map(|t| t.id).collect();
                    wind_claim = None;
                    current_claim = None;
                    throttle_claim = None;
                    rudder_claim = None;
                    mouse_claim = None;
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
