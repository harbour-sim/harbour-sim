//! The settings menu: an overlay for the knobs that configure the game
//! rather than sail the boat.
//!
//! Settings live here, NOT in the play HUD (owner call, 2026-08-20): the
//! HUD's job is the boat, and something you set once and forget has no
//! business taking permanent space on a phone screen. The menu follows
//! the keel editor's overlay pattern — it freezes the game while it is
//! open, which is what lets it reuse keys and take presses without
//! fighting the HUD's touch claims.
//!
//! Adding a setting is a row in `ROWS` plus an arm in BOTH `value` and
//! `value_mut` — `draw` reads through the former, so a row with only the
//! latter panics on the first frame the menu is open.

use harbour_sim_core::line::{
    CrewLimits, LINE_HAUL_KG, LINE_HAUL_KG_MAX, LINE_HAUL_KG_MIN, LINE_PASS_SPEED,
    LINE_PASS_SPEED_MAX, LINE_PASS_SPEED_MIN, LINE_REACH, LINE_REACH_MAX, LINE_REACH_MIN,
};
use macroquad::prelude::*;

/// One adjustable value: a labelled slider with a line of explanation,
/// because a setting nobody understands is a setting nobody touches.
struct Row {
    label: &'static str,
    note: &'static str,
    unit: &'static str,
    min: f32,
    max: f32,
    /// Quantisation, same idea as the HUD dials' 1/20 steps: coarse
    /// enough to be reproducible by hand, fine enough not to matter.
    step: f32,
}

const ROWS: [Row; 3] = [
    Row {
        label: "LINE HANDLING",
        note: "how fast the crew gets a line ashore",
        unit: "m/s",
        min: LINE_PASS_SPEED_MIN,
        max: LINE_PASS_SPEED_MAX,
        step: 0.5,
    },
    Row {
        label: "HAUL FORCE",
        note: "what the crew can pull, hand over hand",
        unit: "kg",
        min: LINE_HAUL_KG_MIN,
        max: LINE_HAUL_KG_MAX,
        step: 1.0,
    },
    Row {
        label: "LINE REACH",
        note: "how far a line can be got ashore",
        unit: "m",
        min: LINE_REACH_MIN,
        max: LINE_REACH_MAX,
        step: 0.5,
    },
];

/// A row without a value would panic on the first frame the menu is
/// drawn, so the count is checked here instead: add a row and this stops
/// compiling until `value` and `value_mut` grow an arm to match
/// (CodeRabbit review, 2026-08-21 — the module note above was the only
/// thing enforcing it).
const _: () = assert!(
    ROWS.len() == 3,
    "every ROWS entry needs an arm in both `value` and `value_mut`",
);

pub struct SettingsMenu {
    pub active: bool,
    /// What the crew can do with a rope: how fast they get one ashore,
    /// how hard they can haul, how far they can throw. Fed to
    /// `InputState::crew`, which is where these become part of the
    /// recorded input stream. See `line::CrewLimits`.
    pub line_pass_speed: f32,
    pub haul_kg: f32,
    pub reach: f32,
    /// Which row's slider a press owns, and the finger holding it — the
    /// same one-claim-per-control rule as the HUD.
    grab: Option<usize>,
    touch: Option<u64>,
    prev_touch_ids: Vec<u64>,
    /// Set by `open`, cleared by the first `update`. The key press (or
    /// the gear tap) that opens the menu is STILL "pressed" when the
    /// overlay's own input runs later in the same frame, and would close
    /// it again instantly — the menu would never appear.
    just_opened: bool,
}

/// Screen rects (css px) of the menu's panel and controls.
#[derive(Clone)]
pub struct SettingsLayout {
    pub panel: Rect,
    pub close: Rect,
    tracks: [Rect; ROWS.len()],
    fs: f32,
}

impl SettingsLayout {
    /// Centred on the screen, sized from the HUD's own font scale so it
    /// matches the rest of the UI on a phone and on a desktop alike.
    pub fn centred(sw: f32, sh: f32, fs: f32) -> SettingsLayout {
        let pad = fs * 1.2;
        let w = (sw * 0.86).min(fs * 26.0);
        // A row spans from its label (0.45 fs above the track) to its
        // note (0.95 fs below it), about 3.8 fs all told — the pitch has
        // to clear that or one row's note lands on the next row's label,
        // which is exactly what happened when the menu grew past one row.
        let row_h = fs * 4.4;
        let h = pad * 2.0 + fs * 2.2 + ROWS.len() as f32 * row_h + fs * 2.6;
        let panel = Rect::new((sw - w) * 0.5, (sh - h) * 0.5, w, h);
        let mut tracks = [Rect::new(0.0, 0.0, 0.0, 0.0); ROWS.len()];
        for (i, t) in tracks.iter_mut().enumerate() {
            *t = Rect::new(
                panel.x + pad,
                panel.y + pad + fs * 2.2 + i as f32 * row_h + fs * 1.3,
                panel.w - pad * 2.0,
                fs * 1.3,
            );
        }
        let close_w = fs * 5.0;
        let close = Rect::new(
            panel.x + panel.w - pad - close_w,
            panel.y + panel.h - pad - fs * 2.2,
            close_w,
            fs * 2.2,
        );
        SettingsLayout { panel, close, tracks, fs }
    }
}

impl SettingsMenu {
    pub fn new() -> SettingsMenu {
        SettingsMenu {
            active: false,
            line_pass_speed: LINE_PASS_SPEED,
            haul_kg: LINE_HAUL_KG,
            reach: LINE_REACH,
            grab: None,
            touch: None,
            prev_touch_ids: Vec::new(),
            just_opened: false,
        }
    }

    /// Open the menu, ignoring input for the rest of this frame.
    pub fn open(&mut self) {
        self.active = true;
        self.just_opened = true;
        self.grab = None;
        self.touch = None;
        self.prev_touch_ids = touches().iter().map(|t| t.id).collect();
    }

    fn value_mut(&mut self, row: usize) -> &mut f32 {
        match row {
            0 => &mut self.line_pass_speed,
            1 => &mut self.haul_kg,
            2 => &mut self.reach,
            _ => unreachable!("every row has a value"),
        }
    }

    fn value(&self, row: usize) -> f32 {
        [self.line_pass_speed, self.haul_kg, self.reach][row]
    }

    /// The settings as sim-core wants them, ready for `InputState::crew`.
    pub fn crew(&self) -> CrewLimits {
        CrewLimits { pass_speed: self.line_pass_speed, haul_kg: self.haul_kg, reach: self.reach }
    }

    fn set_from(&mut self, row: usize, x: f32, layout: &SettingsLayout) {
        let t = ((x - layout.tracks[row].x) / layout.tracks[row].w.max(1.0)).clamp(0.0, 1.0);
        let r = &ROWS[row];
        let raw = r.min + t * (r.max - r.min);
        // Clamped after quantising, not before: a row's bounds need not
        // be multiples of its step, and rounding can land half a step
        // outside the range the panel advertises.
        *self.value_mut(row) = ((raw / r.step).round() * r.step).clamp(r.min, r.max);
    }

    fn press(&mut self, p: Vec2, layout: &SettingsLayout) -> bool {
        if layout.close.contains(p) {
            return true; // close
        }
        for i in 0..ROWS.len() {
            // A generous pad, same reason as the HUD's controls: fat
            // fingers land outside the drawn track.
            let t = layout.tracks[i];
            let pad = layout.fs * 0.8;
            if p.x >= t.x - pad
                && p.x <= t.x + t.w + pad
                && p.y >= t.y - pad
                && p.y <= t.y + t.h + pad
            {
                self.grab = Some(i);
                self.set_from(i, p.x, layout);
                return false;
            }
        }
        // Tapping the darkened screen outside the panel dismisses it —
        // the usual way out of a modal, and the only one that needs no
        // aiming.
        !layout.panel.contains(p)
    }

    /// One frame of input. Returns true when the menu should close.
    pub fn update(&mut self, layout: &SettingsLayout) -> bool {
        if self.just_opened {
            self.just_opened = false;
            return false;
        }
        let mut close = is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::O);
        let mp: Vec2 = mouse_position().into();
        if is_mouse_button_pressed(MouseButton::Left) && self.press(mp, layout) {
            close = true;
        }
        if is_mouse_button_down(MouseButton::Left) {
            if let Some(i) = self.grab {
                self.set_from(i, mp.x, layout);
            }
        } else if self.touch.is_none() {
            // Only when no FINGER owns the grab: `grab` is shared with
            // the touch path below, and on a touchscreen the mouse
            // button is never down (`simulate_mouse_with_touch(false)`),
            // so clearing it unconditionally ended every touch drag on
            // its second frame — the slider took the value under the
            // tap and then ignored the drag.
            self.grab = None;
        }

        // Touch: same gestures. `simulate_mouse_with_touch(false)` means
        // this is the only path that works on a real touchscreen, and
        // fresh-touch detection is by id-not-seen-last-frame exactly as
        // in the HUD (a `Started` phase on a known id = a recycled id,
        // i.e. a new finger).
        let dpi = screen_dpi_scale();
        let ts = touches();
        let ids: Vec<u64> = ts.iter().map(|t| t.id).collect();
        for t in &ts {
            let p = t.position / dpi;
            let fresh = !self.prev_touch_ids.contains(&t.id) || t.phase == TouchPhase::Started;
            if fresh {
                // One claim at a time, as everywhere else in the HUD: a
                // second finger landing while the first still owns a
                // control must not steal it, nor dismiss the menu from
                // the scrim out from under it.
                if self.touch.is_some_and(|owner| owner != t.id) {
                    continue;
                }
                if self.touch == Some(t.id) {
                    self.grab = None;
                    self.touch = None;
                }
                if self.press(p, layout) {
                    close = true;
                } else if self.grab.is_some() {
                    self.touch = Some(t.id);
                }
            } else if self.touch == Some(t.id)
                && let Some(i) = self.grab
            {
                self.set_from(i, p.x, layout);
            }
        }
        if self.touch.is_some_and(|id| !ids.contains(&id)) {
            self.touch = None;
            self.grab = None;
        }
        self.prev_touch_ids = ids;
        close
    }

    pub fn draw(&self, layout: &SettingsLayout, sw: f32, sh: f32) {
        let fs = layout.fs;
        let text = Color::from_rgba(205, 227, 240, 255);
        let dim = Color::from_rgba(130, 160, 178, 255);
        // Scrim: dims the game behind without hiding it, so you can see
        // what the setting is going to affect.
        draw_rectangle(0.0, 0.0, sw, sh, Color::new(0.02, 0.06, 0.09, 0.72));
        let p = layout.panel;
        draw_rectangle(p.x, p.y, p.w, p.h, Color::from_rgba(12, 26, 36, 245));
        draw_rectangle_lines(p.x, p.y, p.w, p.h, 2.0, Color::from_rgba(120, 200, 235, 200));
        let pad = fs * 1.2;
        draw_text("SETTINGS", p.x + pad, p.y + pad + fs, fs * 1.15, text);

        for (i, row) in ROWS.iter().enumerate() {
            let t = layout.tracks[i];
            let v = self.value(i);
            draw_text(row.label, t.x, t.y - fs * 0.45, fs * 0.9, text);
            let val = format!("{:.1} {}", v, row.unit);
            let m = measure_text(&val, None, (fs * 0.9) as u16, 1.0);
            draw_text(&val, t.x + t.w - m.width, t.y - fs * 0.45, fs * 0.9, text);
            let frac = ((v - row.min) / (row.max - row.min)).clamp(0.0, 1.0);
            draw_rectangle(t.x, t.y, t.w, t.h, Color::from_rgba(8, 16, 24, 255));
            draw_rectangle(t.x, t.y, t.w * frac, t.h, Color::from_rgba(24, 62, 84, 255));
            draw_rectangle_lines(t.x, t.y, t.w, t.h, 2.0, dim);
            let knob = t.x + t.w * frac;
            draw_rectangle(knob - 2.0, t.y - 3.0, 4.0, t.h + 6.0, Color::from_rgba(150, 215, 245, 255));
            draw_text(row.note, t.x, t.y + t.h + fs * 0.95, fs * 0.78, dim);
        }

        let c = layout.close;
        draw_rectangle(c.x, c.y, c.w, c.h, Color::from_rgba(10, 20, 30, 200));
        draw_rectangle_lines(c.x, c.y, c.w, c.h, 2.0, dim);
        let m = measure_text("CLOSE", None, fs as u16, 1.0);
        draw_text("CLOSE", c.x + (c.w - m.width) * 0.5, c.y + c.h * 0.5 + fs * 0.35, fs, text);
    }
}
