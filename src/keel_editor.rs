//! In-app keel design editor: drag a bar-chart-style curve of underwater
//! lateral area per unit length along the hull, drag a displacement
//! slider, see the derived hydrodynamic constants (area, centre of lateral
//! resistance, yaw damping) update live, and apply the result to a fresh
//! `Sim`. The four preset buttons load real reference boats' designs
//! (curve AND weight together — see `boat.rs` / `docs/reference-boats.md`).
//!
//! Frontend-only (macroquad) — the editor manipulates a `BoatDesign` value
//! and hands it to `Sim::new_with_design`; nothing here reaches into
//! physics outside a full respawn, keeping the "fresh Sim per run"
//! determinism rule.

use harbour_sim_core::boat::{BoatDesign, RudderDesign};
use harbour_sim_core::keel::KeelProfile;
use harbour_sim_core::sim::{hull_com_x, waterline_extent, yaw_damping_coefficient};
use macroquad::prelude::*;

/// Fixed control-point x positions (hull-local metres, bow positive),
/// spanning the ~12 m hull at 0.25 m spacing. The editor only ever moves
/// the height at each of these columns — simpler and more robust than
/// free-form point dragging, and still reads as "drawing" the curve since
/// you can drag across columns to paint several at once. 0.25 m (not the
/// coarser 0.5 m this started at) matters for presets with small features
/// — e.g. a ~0.4 m rudder chord — that would otherwise resample onto a
/// single column and render as a spike instead of a blade shape.
const EDIT_XS: [f32; 49] = [
    -6.0, -5.75, -5.5, -5.25, -5.0, -4.75, -4.5, -4.25, -4.0, -3.75, -3.5, -3.25, -3.0, -2.75,
    -2.5, -2.25, -2.0, -1.75, -1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0,
    1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25,
    5.5, 5.75, 6.0,
];
const MAX_AREA_PER_LEN: f32 = 4.0;
const HULL_HALF_LEN: f32 = 6.0;

/// Displacement slider range (kg): brackets the real boats the presets
/// come from — modern fin 39-footers start near 4.8 t, traditional
/// full-keel 38s run past 12 t (see `docs/reference-boats.md`).
const DISPL_MIN_KG: f32 = 4_000.0;
const DISPL_MAX_KG: f32 = 14_000.0;
/// Slider quantisation (kg) — same idea as the HUD dials' 1/20 steps:
/// coarse enough to be reproducible by hand, fine enough to not matter.
const DISPL_STEP_KG: f32 = 100.0;

pub enum EditorAction {
    None,
    Apply,
    Cancel,
}

pub struct KeelEditor {
    pub active: bool,
    ys: [f32; EDIT_XS.len()],
    /// Displacement (kg) — edited by the slider / Up-Down keys, loaded and
    /// applied together with the curve (a preset is a whole boat design).
    displacement_kg: f32,
    /// The rudder blade of the loaded design — carried through Apply and
    /// drawn as the fixed marker, but not yet editable here (follow-up
    /// work): presets change it, painting doesn't.
    rudder: RudderDesign,
    /// Touch id + position seen last frame — lets button taps use the same
    /// fresh-touch detection as the HUD dials, and lets drag painting fill
    /// in the columns between two frames' samples instead of leaving gaps
    /// (see `update`/`drag_fill`).
    prev_touches: Vec<(u64, Vec2)>,
    /// Mouse position last frame while the button was held, for the same
    /// gap-filling reason on a mouse drag.
    mouse_drag_prev: Option<Vec2>,
    /// True while a held mouse button is claimed by the displacement
    /// slider — the same one-claim-per-control idea as the HUD's
    /// `mouse_claim`: a drag that starts on the slider must keep driving
    /// the slider even when it sweeps across the curve canvas, not start
    /// painting bars mid-drag.
    mouse_on_weight: bool,
    /// Touch id claimed by the displacement slider, same rule as above
    /// (and the same recycled-id `Started` re-check as the HUD dials).
    weight_touch: Option<u64>,
}

impl KeelEditor {
    pub fn new(initial: &BoatDesign) -> Self {
        let mut editor = KeelEditor {
            active: false,
            ys: [0.0; EDIT_XS.len()],
            displacement_kg: DISPL_MIN_KG,
            rudder: initial.rudder,
            prev_touches: Vec::new(),
            mouse_drag_prev: None,
            mouse_on_weight: false,
            weight_touch: None,
        };
        editor.load_design(initial);
        editor
    }

    /// Load a whole design: curve resampled onto the fixed grid AND the
    /// displacement. Used for the initial boat, reopening the editor on
    /// the current design, and the preset buttons.
    pub fn load_design(&mut self, design: &BoatDesign) {
        self.load(&design.keel);
        self.displacement_kg = design.displacement_kg.clamp(DISPL_MIN_KG, DISPL_MAX_KG);
        self.rudder = design.rudder;
    }

    /// Resample any profile onto this editor's fixed grid.
    fn load(&mut self, profile: &KeelProfile) {
        for (y, &x) in self.ys.iter_mut().zip(EDIT_XS.iter()) {
            *y = profile.sample(x).clamp(0.0, MAX_AREA_PER_LEN);
        }
    }

    /// The design as currently edited — what Apply hands to
    /// `Sim::new_with_design`.
    pub fn design(&self) -> BoatDesign {
        BoatDesign {
            keel: self.profile(),
            rudder: self.rudder,
            displacement_kg: self.displacement_kg,
        }
    }

    fn profile(&self) -> KeelProfile {
        KeelProfile {
            points: EDIT_XS.iter().zip(self.ys).map(|(&x, y)| vec2(x, y)).collect(),
        }
    }

    /// Map a screen x on the slider track to a quantised displacement.
    fn set_weight_from(&mut self, track: Rect, px: f32) {
        let t = ((px - track.x) / track.w).clamp(0.0, 1.0);
        let kg = DISPL_MIN_KG + t * (DISPL_MAX_KG - DISPL_MIN_KG);
        self.displacement_kg = (kg / DISPL_STEP_KG).round() * DISPL_STEP_KG;
    }

    /// Paint every column between `prev` and `cur` (screen px), linearly
    /// interpolating the value between them, if `cur` falls inside
    /// `canvas`. Shared by mouse-drag and touch-drag.
    ///
    /// A single sample per frame isn't enough: at the finer 0.25 m grid
    /// (as little as ~6 px/column on a portrait phone) a normal-speed
    /// stroke easily moves several columns between two frames' samples,
    /// which without filling leaves a comb of isolated painted columns
    /// instead of a continuous curve.
    fn drag_fill(&mut self, canvas: Rect, prev: Vec2, cur: Vec2) {
        if !canvas.contains(cur) {
            return;
        }
        // The baseline (0 area) is the TOP of the canvas — this is a keel
        // hanging below the hull, so more area reads as deeper (further
        // down), like a real keel-profile sketch, not taller.
        let project = |p: Vec2| {
            let t = ((p.x - canvas.x) / canvas.w).clamp(0.0, 1.0);
            let idx = (t * (EDIT_XS.len() - 1) as f32).round() as usize;
            let v =
                ((p.y - canvas.y) / canvas.h * MAX_AREA_PER_LEN).clamp(0.0, MAX_AREA_PER_LEN);
            (idx, v)
        };
        let (idx0, v0) = project(prev);
        let (idx1, v1) = project(cur);
        let (idx_a, v_a, idx_b, v_b) =
            if idx0 <= idx1 { (idx0, v0, idx1, v1) } else { (idx1, v1, idx0, v0) };
        for idx in idx_a..=idx_b {
            let frac =
                if idx_b > idx_a { (idx - idx_a) as f32 / (idx_b - idx_a) as f32 } else { 1.0 };
            self.ys[idx] = v_a + (v_b - v_a) * frac;
        }
    }

    /// A click/tap at `p` (screen px) on one of the six buttons, if any.
    /// Shared by mouse-click and touch-tap. The preset buttons load the
    /// whole design — curve AND displacement — because each is a real
    /// boat, not just a keel shape.
    fn button_at(&mut self, layout: EditorLayout, p: Vec2) -> EditorAction {
        if layout.hr38.contains(p) {
            self.load_design(&BoatDesign::hallberg_rassy_38());
        } else if layout.oday.contains(p) {
            self.load_design(&BoatDesign::oday_39());
        } else if layout.elan.contains(p) {
            self.load_design(&BoatDesign::elan_impression_394());
        } else if layout.oceanis.contains(p) {
            self.load_design(&BoatDesign::beneteau_oceanis_381());
        } else if layout.alajuela.contains(p) {
            self.load_design(&BoatDesign::alajuela_38());
        } else if layout.apply.contains(p) {
            return EditorAction::Apply;
        } else if layout.cancel.contains(p) {
            return EditorAction::Cancel;
        }
        EditorAction::None
    }

    /// Handle mouse/touch/keyboard input for one frame. `canvas` is the
    /// graph area in screen pixels; `layout` carries the button rects and
    /// the displacement slider track, also in screen pixels.
    pub fn update(&mut self, canvas: Rect, layout: EditorLayout) -> EditorAction {
        let (mx, my) = mouse_position();
        let cur = vec2(mx, my);
        if is_mouse_button_pressed(MouseButton::Left) && layout.weight.contains(cur) {
            self.mouse_on_weight = true;
        }
        if is_mouse_button_down(MouseButton::Left) {
            if self.mouse_on_weight {
                self.set_weight_from(layout.weight, cur.x);
            } else {
                self.drag_fill(canvas, self.mouse_drag_prev.unwrap_or(cur), cur);
                self.mouse_drag_prev = Some(cur);
            }
        } else {
            self.mouse_on_weight = false;
            self.mouse_drag_prev = None;
        }
        if is_mouse_button_pressed(MouseButton::Left) {
            let action = self.button_at(layout, cur);
            if !matches!(action, EditorAction::None) {
                return action;
            }
        }

        // --- Touch input: same gestures as the mouse. The app disables
        // touch→mouse synthesis (`simulate_mouse_with_touch(false)`, for
        // the HUD dials) so this is the only path that makes the editor
        // usable on an actual touchscreen.
        let dpi = screen_dpi_scale();
        let ts = touches();
        let mut next_prev_touches: Vec<(u64, Vec2)> = Vec::with_capacity(ts.len());
        let mut touch_action = EditorAction::None;
        for t in &ts {
            let p = t.position / dpi; // physical → logical px
            let prev_pos = self.prev_touches.iter().find(|(id, _)| *id == t.id).map(|&(_, p)| p);
            // Only a FRESH touch taps a button — a finger held on Apply
            // must not re-trigger it every frame it stays down. A fresh
            // touch also has no real "previous" drag position, so its fill
            // degenerates to painting just the one column under it.
            let fresh = prev_pos.is_none() || t.phase == TouchPhase::Started;
            // Slider claim: a fresh touch landing on the track owns the
            // slider until it lifts; a recycled id starting elsewhere
            // (`Started` on an already-claimed id = new finger) drops a
            // stale claim — the same rules as the HUD dials.
            if fresh {
                if layout.weight.contains(p) {
                    self.weight_touch = Some(t.id);
                } else if self.weight_touch == Some(t.id) {
                    self.weight_touch = None;
                }
            }
            if self.weight_touch == Some(t.id) {
                self.set_weight_from(layout.weight, p.x);
            } else {
                self.drag_fill(canvas, prev_pos.unwrap_or(p), p);
            }
            next_prev_touches.push((t.id, p));
            if fresh {
                let action = self.button_at(layout, p);
                if !matches!(action, EditorAction::None) {
                    touch_action = action;
                }
            }
        }
        self.prev_touches = next_prev_touches;
        if self.weight_touch.is_some_and(|id| !ts.iter().any(|t| t.id == id)) {
            self.weight_touch = None;
        }
        if !matches!(touch_action, EditorAction::None) {
            return touch_action;
        }

        // D/F/G/B/L load the preset boats. D is helm-to-starboard and the
        // arrows are env keys in the game, but the editor freezes all game
        // input while open, so reusing them here can't conflict.
        if is_key_pressed(KeyCode::D) {
            self.load_design(&BoatDesign::hallberg_rassy_38());
        }
        if is_key_pressed(KeyCode::F) {
            self.load_design(&BoatDesign::oday_39());
        }
        if is_key_pressed(KeyCode::G) {
            self.load_design(&BoatDesign::elan_impression_394());
        }
        if is_key_pressed(KeyCode::B) {
            self.load_design(&BoatDesign::beneteau_oceanis_381());
        }
        if is_key_pressed(KeyCode::L) {
            self.load_design(&BoatDesign::alajuela_38());
        }
        // Displacement on Up/Down (continuous, 2 t/s) for keyboard parity
        // with the slider.
        let dkg = 2_000.0 * get_frame_time();
        if is_key_down(KeyCode::Up) {
            self.displacement_kg = (self.displacement_kg + dkg).min(DISPL_MAX_KG);
        }
        if is_key_down(KeyCode::Down) {
            self.displacement_kg = (self.displacement_kg - dkg).max(DISPL_MIN_KG);
        }
        if is_key_pressed(KeyCode::Enter) {
            return EditorAction::Apply;
        }
        if is_key_pressed(KeyCode::Escape) {
            return EditorAction::Cancel;
        }
        EditorAction::None
    }

    pub fn draw(&self, canvas: Rect, layout: EditorLayout, ui: f32) {
        draw_rectangle(0.0, 0.0, screen_width(), screen_height(), Color::from_rgba(6, 10, 14, 210));

        let derived = self.profile().derive();
        let fs = 24.0 * ui;
        let text = Color::from_rgba(220, 235, 245, 255);
        let dim = Color::from_rgba(140, 165, 180, 255);

        draw_text("KEEL DESIGN", canvas.x, canvas.y - 46.0 * ui, fs * 1.2, text);
        draw_text(
            "drag to paint the underwater area-per-length curve (stern <-> bow)",
            canvas.x,
            canvas.y - 18.0 * ui,
            fs * 0.7,
            dim,
        );

        // Canvas background + axes. The top edge is the baseline (hull
        // bottom / waterline, 0 area) — the keel hangs BELOW it, so the
        // curve is drawn growing downward like a real profile sketch.
        draw_rectangle(canvas.x, canvas.y, canvas.w, canvas.h, Color::from_rgba(14, 22, 30, 255));
        draw_rectangle_lines(canvas.x, canvas.y, canvas.w, canvas.h, 1.5, dim);
        let x_to_px = |x: f32| canvas.x + (x + HULL_HALF_LEN) / (2.0 * HULL_HALF_LEN) * canvas.w;
        draw_line(
            canvas.x,
            canvas.y,
            canvas.x + canvas.w,
            canvas.y,
            2.0,
            Color::from_rgba(150, 170, 185, 220),
        );

        // X-axis ruler: gridlines + numeric ticks every 2 m, hull position
        // in metres from the pivot, with STERN/BOW called out at the ends.
        let grid_col = Color::from_rgba(50, 60, 70, 130);
        let mut tick = -HULL_HALF_LEN;
        while tick <= HULL_HALF_LEN + 0.01 {
            let tx = x_to_px(tick);
            draw_line(tx, canvas.y, tx, canvas.y + canvas.h, 1.0, grid_col);
            let label = if tick == 0.0 {
                "0".to_string()
            } else if tick <= -HULL_HALF_LEN {
                format!("{:+.0}m STERN", tick)
            } else if tick >= HULL_HALF_LEN {
                format!("{:+.0}m BOW", tick)
            } else {
                format!("{:+.0}m", tick)
            };
            let tw = if tick == 0.0 { 4.0 } else { 30.0 };
            draw_text(
                &label,
                (tx - tw * ui).max(canvas.x),
                canvas.y + canvas.h + 20.0 * ui,
                fs * 0.6,
                dim,
            );
            tick += 2.0;
        }

        // Y-axis ruler: gridlines + numeric ticks every 1 m^2/m of area
        // (baseline at the top = 0, deepest at the bottom = MAX).
        for i in 0..=(MAX_AREA_PER_LEN as i32) {
            let a = i as f32;
            let ty = canvas.y + (a / MAX_AREA_PER_LEN) * canvas.h;
            draw_line(canvas.x, ty, canvas.x + canvas.w, ty, 1.0, grid_col);
            let label_y = if i == 0 { ty + 12.0 * ui } else { ty - 4.0 * ui };
            draw_text(format!("{a:.0}"), canvas.x + 3.0 * ui, label_y, fs * 0.55, dim);
        }
        draw_text(
            "m^2/m",
            canvas.x + 3.0 * ui,
            canvas.y + canvas.h - 4.0 * ui,
            fs * 0.55,
            dim,
        );

        // The curve itself: bars hanging down from the baseline, plus a
        // connecting polyline along their bottom edge (the keel's outline).
        // Bar width adapts to the column spacing so a finer grid doesn't
        // overlap.
        let col_w = canvas.w / (EDIT_XS.len() - 1) as f32;
        let bar_hw = (col_w * 0.35).clamp(1.0, 3.0);
        let bar_col = Color::from_rgba(120, 200, 230, 255);
        let mut prev: Option<Vec2> = None;
        for (i, &y) in self.ys.iter().enumerate() {
            let cx = canvas.x + col_w * i as f32;
            let h = (y / MAX_AREA_PER_LEN) * canvas.h;
            draw_rectangle(cx - bar_hw, canvas.y, bar_hw * 2.0, h, bar_col);
            let p = vec2(cx, canvas.y + h);
            if let Some(prev) = prev {
                draw_line(prev.x, prev.y, p.x, p.y, 2.0, Color::from_rgba(190, 230, 245, 255));
            }
            prev = Some(p);
        }
        // The rudder: no longer part of the paintable curve (it's a
        // separate movable foil in `sim.rs` now, see its doc comment for
        // why folding it into this profile double-counted it), drawn here
        // as a non-paintable reference from the LOADED DESIGN's own blade
        // (`self.rudder` — the same `RudderDesign` the physics uses, not
        // a separate guess): each preset brings its real boat's blade, so
        // the marker moves and resizes with the preset. Drawn stacked
        // BELOW whatever the hull/keel curve is at that station, rather
        // than from the baseline, so it reads as an appendage hanging off
        // the hull rather than overlapping the editable area.
        let rudder_hull_depth = self.profile().sample(self.rudder.x);
        let rudder_x0 = x_to_px(self.rudder.x - self.rudder.chord / 2.0);
        let rudder_x1 = x_to_px(self.rudder.x + self.rudder.chord / 2.0);
        let rudder_top_v = rudder_hull_depth.min(MAX_AREA_PER_LEN);
        let rudder_bottom_v = (rudder_hull_depth + self.rudder.depth).min(MAX_AREA_PER_LEN);
        let rudder_top = canvas.y + (rudder_top_v / MAX_AREA_PER_LEN) * canvas.h;
        let rudder_bottom = canvas.y + (rudder_bottom_v / MAX_AREA_PER_LEN) * canvas.h;
        let rudder_col = Color::from_rgba(230, 120, 80, 200);
        draw_rectangle(
            rudder_x0,
            rudder_top,
            rudder_x1 - rudder_x0,
            rudder_bottom - rudder_top,
            Color::from_rgba(230, 120, 80, 90),
        );
        draw_rectangle_lines(
            rudder_x0,
            rudder_top,
            rudder_x1 - rudder_x0,
            rudder_bottom - rudder_top,
            1.5,
            rudder_col,
        );
        // The canvas is a side elevation, so a twin pair sits exactly
        // behind its sibling and there is nothing extra to draw — but the
        // label has to say so, or the boat with the most rudder of the
        // five reads as having the least. (The top-down view in main.rs
        // is where you actually see both blades.)
        let rudder_label = if self.rudder.layout.blades() > 1 {
            "twin rudders (preset-set, not paintable)"
        } else {
            "rudder (preset-set, not paintable)"
        };
        draw_text(
            rudder_label,
            rudder_x1 + 4.0 * ui,
            (rudder_top + rudder_bottom) * 0.5,
            fs * 0.55,
            rudder_col,
        );

        // Centre of lateral resistance marker — the DRAG-weighted centroid
        // (round-hull vs flat-plate-keel material, see keel.rs), the same
        // lever arm `tick()` actually applies the sway force at.
        let clr_t = ((derived.drag_clr_offset + 6.0) / 12.0).clamp(0.0, 1.0);
        let clr_x = canvas.x + canvas.w * clr_t;
        draw_line(
            clr_x,
            canvas.y,
            clr_x,
            canvas.y + canvas.h,
            2.0,
            Color::from_rgba(255, 170, 60, 255),
        );
        draw_text("CLR", clr_x + 4.0 * ui, canvas.y + 16.0 * ui, fs * 0.65, Color::from_rgba(255, 170, 60, 255));

        // Centre of gravity marker — the hull polygon's centroid
        // (`sim::hull_com_x`, the same COM Rapier derives from the uniform
        // hull spread, unit-tested against it). Deliberately a SEPARATE
        // marker from the CLR: the two do not coincide (the CLR follows
        // the painted keel curve, the CG follows only the hull outline),
        // and their gap is exactly the lever arm that turns sway drag
        // into yaw moment. Label at the canvas BOTTOM so it can't collide
        // with CLR's top label when the lines sit close together.
        let cg_col = Color::from_rgba(150, 230, 130, 255);
        let cg_t = ((hull_com_x() + 6.0) / 12.0).clamp(0.0, 1.0);
        let cg_x = canvas.x + canvas.w * cg_t;
        draw_line(cg_x, canvas.y, cg_x, canvas.y + canvas.h, 2.0, cg_col);
        draw_circle_lines(cg_x, canvas.y + canvas.h - 26.0 * ui, 5.0 * ui, 1.5, cg_col);
        draw_text("CG", cg_x + 4.0 * ui, canvas.y + canvas.h - 8.0 * ui, fs * 0.65, cg_col);

        // Live readout. Area stays the real geometric total (m², sanity-
        // checkable against a published keel area); CLR/yaw damping use
        // the drag-weighted (round-hull/flat-plate-keel) values, since
        // those are the actual lever arm and damping `tick()` applies.
        // LWL comes from the same `waterline_extent` the physics reads
        // (the curve's nonzero support — paint the ends dry and the
        // waterline shortens live, hull speed with it).
        let (wl_aft, wl_fwd) = waterline_extent(&self.profile());
        let c_yaw_q = yaw_damping_coefficient(derived.drag_cubic_moment);
        draw_text(
            format!(
                "LWL {:.1} m   area {:.1} m^2   CLR {:+.2} m   yaw damping {:.0} N*m/(rad/s)^2",
                wl_fwd - wl_aft,
                derived.area,
                derived.drag_clr_offset,
                c_yaw_q
            ),
            canvas.x,
            canvas.y + canvas.h + 46.0 * ui,
            fs * 0.8,
            text,
        );
        // And mark the waterline's extent on the baseline itself: the wet
        // span drawn solid over the (dimmer) full-hull baseline, so the
        // dry overhangs read as exactly that.
        let wl_col = Color::from_rgba(120, 200, 230, 255);
        draw_line(
            x_to_px(wl_aft),
            canvas.y,
            x_to_px(wl_fwd),
            canvas.y,
            3.0,
            wl_col,
        );

        // Displacement slider: track + filled portion + knob, value and
        // key hint to the right of the track.
        let track = layout.weight;
        let cy = track.y + track.h * 0.5;
        draw_line(track.x, cy, track.x + track.w, cy, 2.0, dim);
        let t = ((self.displacement_kg - DISPL_MIN_KG) / (DISPL_MAX_KG - DISPL_MIN_KG))
            .clamp(0.0, 1.0);
        let kx = track.x + track.w * t;
        draw_line(track.x, cy, kx, cy, 4.0, bar_col);
        draw_circle(kx, cy, 8.0 * ui, Color::from_rgba(190, 230, 245, 255));
        draw_text(
            format!("displacement {:.1} t [Up/Dn]", self.displacement_kg / 1000.0),
            track.x + track.w + 12.0 * ui,
            cy + fs * 0.25,
            fs * 0.65,
            text,
        );

        // Buttons. The four presets are real boats (design = curve +
        // weight, specs in docs/reference-boats.md); labels shrink to fit
        // their button so the boat names survive narrow viewports.
        for (rect, label) in [
            (layout.hr38, "HR 38 [D]"),
            (layout.oday, "O'Day 39 [F]"),
            (layout.elan, "Elan 394 [G]"),
            (layout.oceanis, "Oceanis 38.1 [B]"),
            (layout.alajuela, "Alajuela 38 [L]"),
            (layout.apply, "Apply [Enter]"),
            (layout.cancel, "Cancel [Esc]"),
        ] {
            draw_rectangle(rect.x, rect.y, rect.w, rect.h, Color::from_rgba(30, 40, 50, 255));
            draw_rectangle_lines(rect.x, rect.y, rect.w, rect.h, 1.5, dim);
            let mut size = fs * 0.75;
            let mut tw = measure_text(label, None, size as u16, 1.0).width;
            while tw > rect.w - 10.0 * ui && size > 9.0 {
                size -= 1.0;
                tw = measure_text(label, None, size as u16, 1.0).width;
            }
            draw_text(label, rect.x + (rect.w - tw) * 0.5, rect.y + rect.h * 0.65, size, text);
        }
    }
}

/// The editor's interactive regions below the curve canvas: the four
/// preset-boat buttons, Apply/Cancel, and the displacement slider track.
#[derive(Clone, Copy)]
pub struct EditorLayout {
    pub hr38: Rect,
    pub oday: Rect,
    pub elan: Rect,
    pub oceanis: Rect,
    pub alajuela: Rect,
    pub apply: Rect,
    pub cancel: Rect,
    /// Displacement slider TRACK (the interactive part — the value label
    /// drawn to its right is display-only).
    pub weight: Rect,
}

impl EditorLayout {
    /// Lay out the rows under `canvas`: readout text (drawn by `draw`,
    /// not a rect here), then the displacement slider, then the buttons,
    /// sized to fill exactly `canvas.w` — a fixed button width could
    /// overflow the canvas (and the screen) on narrow viewports, working
    /// against the whole point of having a touch-reachable editor.
    ///
    /// TWO rows since the fifth preset landed (2026-09-11): five presets
    /// plus Apply and Cancel across one row would be `canvas.w / 7` per
    /// button, well under a thumb on a phone, and the mobile-first rule
    /// in CLAUDE.md makes that the layout's problem rather than the
    /// preset's. Presets on top, the two actions below at half width
    /// each — which also stops a mis-aimed tap at Cancel from landing on
    /// a preset and silently rewriting the curve.
    pub fn under(canvas: Rect, ui: f32) -> EditorLayout {
        let gap = 12.0 * ui;
        let bw = (canvas.w - gap * 4.0) / 5.0;
        let bh = 40.0 * ui;
        let y = canvas.y + canvas.h + 104.0 * ui;
        let y2 = y + bh + gap;
        let aw = (canvas.w - gap) / 2.0;
        let x0 = canvas.x;
        EditorLayout {
            hr38: Rect::new(x0, y, bw, bh),
            oday: Rect::new(x0 + (bw + gap), y, bw, bh),
            elan: Rect::new(x0 + (bw + gap) * 2.0, y, bw, bh),
            oceanis: Rect::new(x0 + (bw + gap) * 3.0, y, bw, bh),
            alajuela: Rect::new(x0 + (bw + gap) * 4.0, y, bw, bh),
            apply: Rect::new(x0, y2, aw, bh),
            cancel: Rect::new(x0 + aw + gap, y2, aw, bh),
            // 55% of the width for the track leaves room for the value
            // label on the right; 26 px tall so it's a real touch target.
            weight: Rect::new(x0, canvas.y + canvas.h + 56.0 * ui, canvas.w * 0.55, 26.0 * ui),
        }
    }
}
