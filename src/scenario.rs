//! Scenario setup: the environment half of a run — wind and current.
//!
//! These were two draggable dials pinned in the HUD's top corners. They
//! live in a modal now (opened with `V`, or by tapping the conditions
//! panel the dials left behind), because that is what wind and current
//! ARE: the scenario a mooring manoeuvre is set in, chosen deliberately
//! before the run rather than flown by hand during it. The HUD keeps a
//! read-only panel of the same two indicators, so the conditions stay
//! visible — and one tap away — while driving.
//!
//! Frontend-only, like `keel_editor`: everything here edits a plain `Env`
//! value and hands it back to `main`, which passes it to `Sim::tick`.
//! Nothing in this file touches physics.

use harbour_sim_core::sim::Env;
use macroquad::prelude::*;

/// Dial rim = these. Shared by the modal's dials and by the HUD panel's
/// read-only twins (the fraction each arrow is drawn at).
pub const WIND_MAX: f32 = 25.0;
pub const CURRENT_MAX: f32 = 2.5;

/// Keyboard rates inside the modal (per second of key held). The arrows
/// still steer the wind and IJKL the current — same keys, same rates as
/// when they were game keys; they just act on the modal's copy now, which
/// is also what freed I/K/J/L and the arrows from the driving keymap.
const DIR_RATE: f32 = 45.0; // degrees
const WIND_RATE: f32 = 3.0; // m/s
const CURRENT_RATE: f32 = 0.4; // m/s

// The HUD's palette (`Color::from_rgba` isn't const, hence the /255).
const WIND_COL: Color = Color::new(120.0 / 255.0, 220.0 / 255.0, 1.0, 1.0);
const CUR_COL: Color = Color::new(90.0 / 255.0, 235.0 / 255.0, 170.0 / 255.0, 1.0);
const TEXT_COL: Color = Color::new(205.0 / 255.0, 227.0 / 255.0, 240.0 / 255.0, 1.0);
const DIM_COL: Color = Color::new(130.0 / 255.0, 160.0 / 255.0, 178.0 / 255.0, 1.0);
const PANEL_BG: Color = Color::new(10.0 / 255.0, 20.0 / 255.0, 30.0 / 255.0, 170.0 / 255.0);

/// A draggable compass dial (screen-space geometry, css px). Moved here
/// from the HUD with the settings it drives.
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

/// A condition preset, written RELATIVE to the marina's down-channel
/// bearing (passed in from the shore geometry, see `Scenario::new`) so a
/// preset keeps meaning what its label says if the harbour is ever
/// re-laid or re-mirrored — the way the direction-sensitive sim-core
/// tests derive their bearings from the geometry instead of hardcoding
/// them. Zero = down-channel (toward the sea), +90° = the water side the
/// dock row faces, so an onshore wind (`wind_from_rel` +90) is the one
/// that pins the boat onto the docks.
struct Preset {
    label: &'static str,
    wind_from_rel: f32,
    wind_speed: f32,
    current_to_rel: f32,
    current_speed: f32,
}

const PRESETS: [Preset; 4] = [
    Preset {
        label: "Calm [1]",
        wind_from_rel: 0.0,
        wind_speed: 0.0,
        current_to_rel: 0.0,
        current_speed: 0.0,
    },
    // Blowing off the water onto the dock row: berthing to leeward, the
    // wind setting you down onto the boats you are trying to lie beside.
    Preset {
        label: "Onshore [2]",
        wind_from_rel: 90.0,
        wind_speed: 9.0,
        current_to_rel: 0.0,
        current_speed: 0.3,
    },
    // The mirror image: a lee shore for anyone in a berth, blowing the
    // bow off as you come alongside.
    Preset {
        label: "Offshore [3]",
        wind_from_rel: -90.0,
        wind_speed: 11.0,
        current_to_rel: 0.0,
        current_speed: 0.2,
    },
    // Wind over tide: the stream runs down-channel, the breeze straight
    // back up it — boat lies to whichever wins, which is the point.
    Preset {
        label: "Wind v tide [4]",
        wind_from_rel: 0.0,
        wind_speed: 10.0,
        current_to_rel: 0.0,
        current_speed: 1.2,
    },
];

impl Preset {
    fn env(&self, channel_deg: f32) -> Env {
        Env {
            wind_from_deg: (channel_deg + self.wind_from_rel).rem_euclid(360.0),
            wind_speed: self.wind_speed,
            current_to_deg: (channel_deg + self.current_to_rel).rem_euclid(360.0),
            current_speed: self.current_speed,
        }
    }
}

pub enum ScenarioAction {
    None,
    /// Keep the edited conditions (Apply button, Enter, or the V key that
    /// opened the modal in the first place).
    Apply,
    /// Drop them and go back to what the run was already sailing in.
    Cancel,
}

/// Screen-space geometry of the modal, recomputed every frame from the
/// window size — same idiom as `EditorLayout`, and the same reason: a
/// fixed pixel layout would overflow a phone screen.
#[derive(Clone, Copy)]
pub struct ScenarioLayout {
    card: Rect,
    ui: f32,
    wind: Dial,
    current: Dial,
    presets: [Rect; PRESETS.len()],
    apply: Rect,
    cancel: Rect,
}

impl ScenarioLayout {
    /// Centre the card in the window. `ui` is the keel editor's scale
    /// factor idiom (`(min_dim / 980).clamp(0.5, 1.0)`), passed in so the
    /// two overlays scale identically.
    ///
    /// The two button rows carry a MINIMUM height in css px on top of that
    /// scaling, and the card grows by whatever those minimums add: at the
    /// small end of `ui` a purely proportional row lands about 17 px tall,
    /// which is a fine-looking button and a miserable thumb target — and on
    /// a phone this modal is the ONLY way to set the conditions. Both rows
    /// hang off the card's bottom edge, so the growth eats the slack under
    /// the dials instead of overflowing the card; nothing moves at all on a
    /// desktop-sized window, where the proportional heights already win.
    pub fn centred(sw: f32, sh: f32, ui: f32) -> ScenarioLayout {
        let preset_h = (34.0 * ui).max(32.0);
        let apply_h = (40.0 * ui).max(40.0);
        let grown = (preset_h - 34.0 * ui) + (apply_h - 40.0 * ui);
        let (cw, ch) = (620.0 * ui, 380.0 * ui + grown);
        let card = Rect::new(sw * 0.5 - cw * 0.5, sh * 0.5 - ch * 0.5, cw, ch);
        let pad = 18.0 * ui;
        let content = card.w - pad * 2.0;
        let r = 62.0 * ui;
        let cy = card.y + 138.0 * ui;
        let gap = 10.0 * ui;
        let pw = (content - gap * (PRESETS.len() as f32 - 1.0)) / PRESETS.len() as f32;
        let aw = (content - gap) * 0.5;
        let ay = card.y + card.h - 34.0 * ui - apply_h;
        let py = ay - 14.0 * ui - preset_h;
        ScenarioLayout {
            card,
            ui,
            wind: Dial { cx: card.x + pad + content * 0.25, cy, r },
            current: Dial { cx: card.x + pad + content * 0.75, cy, r },
            presets: std::array::from_fn(|i| {
                Rect::new(card.x + pad + (pw + gap) * i as f32, py, pw, preset_h)
            }),
            apply: Rect::new(card.x + pad, ay, aw, apply_h),
            cancel: Rect::new(card.x + pad + aw + gap, ay, aw, apply_h),
        }
    }
}

pub struct Scenario {
    pub active: bool,
    /// The edited copy: seeded from the live `Env` when the modal opens,
    /// read back by `main` on Apply, dropped on Cancel. The live `Env`
    /// is never mutated from in here.
    env: Env,
    /// Down-channel compass bearing of the marina, straight off the shore
    /// geometry — what the presets above are written relative to.
    channel_deg: f32,
    /// Touch/mouse claims, one per dial, with the same rules as the HUD's
    /// (claim by id-not-seen-last-frame, a `Started` phase on a claimed
    /// id means a recycled id = new finger).
    wind_touch: Option<u64>,
    current_touch: Option<u64>,
    prev_touch_ids: Vec<u64>,
    mouse_claim: Option<u8>, // 0 = wind, 1 = current
}

impl Scenario {
    pub fn new(channel_deg: f32) -> Scenario {
        Scenario {
            active: false,
            env: Env::CALM,
            channel_deg,
            wind_touch: None,
            current_touch: None,
            prev_touch_ids: Vec::new(),
            mouse_claim: None,
        }
    }

    /// Open on a copy of the run's current conditions. Held fingers are
    /// snapshotted as "already known" so the tap that opened the modal
    /// can't immediately grab a dial under it (the same transition rule
    /// the HUD uses when an overlay opens).
    pub fn open(&mut self, env: &Env) {
        self.env = *env;
        self.active = true;
        self.prev_touch_ids = touches().iter().map(|t| t.id).collect();
        self.wind_touch = None;
        self.current_touch = None;
        self.mouse_claim = None;
    }

    /// The conditions as edited — what Apply hands back to `main`.
    pub fn env(&self) -> Env {
        self.env
    }

    fn set_wind(&mut self, (to_deg, frac): (f32, f32)) {
        // The dial is dragged toward where the flow GOES; wind is named
        // for where it comes from (mariners' convention).
        self.env.wind_from_deg = (to_deg + 180.0).rem_euclid(360.0);
        self.env.wind_speed = frac * WIND_MAX;
    }

    fn set_current(&mut self, (to_deg, frac): (f32, f32)) {
        self.env.current_to_deg = to_deg;
        self.env.current_speed = frac * CURRENT_MAX;
    }

    /// A click/tap at `p` on one of the buttons, if any. Shared by the
    /// mouse and touch paths.
    fn button_at(&mut self, layout: &ScenarioLayout, p: Vec2) -> ScenarioAction {
        for (rect, preset) in layout.presets.iter().zip(PRESETS.iter()) {
            if rect.contains(p) {
                self.env = preset.env(self.channel_deg);
                return ScenarioAction::None;
            }
        }
        if layout.apply.contains(p) {
            return ScenarioAction::Apply;
        }
        if layout.cancel.contains(p) {
            return ScenarioAction::Cancel;
        }
        ScenarioAction::None
    }

    /// One frame of mouse/touch/keyboard input for the modal.
    pub fn update(&mut self, layout: &ScenarioLayout) -> ScenarioAction {
        let dt = get_frame_time().min(0.05);

        // --- Mouse: press claims a dial, drag drives it.
        let mp: Vec2 = mouse_position().into();
        if is_mouse_button_pressed(MouseButton::Left) {
            if layout.wind.hit(mp) {
                self.mouse_claim = Some(0);
            } else if layout.current.hit(mp) {
                self.mouse_claim = Some(1);
            } else {
                let action = self.button_at(layout, mp);
                if !matches!(action, ScenarioAction::None) {
                    return action;
                }
            }
        }
        if is_mouse_button_down(MouseButton::Left) {
            match self.mouse_claim {
                Some(0) => self.set_wind(layout.wind.value(mp)),
                Some(1) => self.set_current(layout.current.value(mp)),
                _ => {}
            }
        } else {
            self.mouse_claim = None;
        }

        // --- Touch: same gestures. Touch→mouse synthesis is off app-wide
        // (`simulate_mouse_with_touch(false)`), so this is the only path
        // that makes the modal work on a touchscreen.
        let dpi = screen_dpi_scale();
        let ts = touches();
        let cur_ids: Vec<u64> = ts.iter().map(|t| t.id).collect();
        let mut touch_action = ScenarioAction::None;
        for t in &ts {
            let p = t.position / dpi; // physical → logical px
            let fresh = !self.prev_touch_ids.contains(&t.id) || t.phase == TouchPhase::Started;
            if fresh {
                // A recycled id is a NEW finger: drop any stale claim.
                if self.wind_touch == Some(t.id) {
                    self.wind_touch = None;
                }
                if self.current_touch == Some(t.id) {
                    self.current_touch = None;
                }
                if layout.wind.hit(p) && self.wind_touch.is_none() {
                    self.wind_touch = Some(t.id);
                } else if layout.current.hit(p) && self.current_touch.is_none() {
                    self.current_touch = Some(t.id);
                } else {
                    // Only a FRESH touch taps a button — a finger resting
                    // on Apply must not re-trigger it every frame.
                    let action = self.button_at(layout, p);
                    if !matches!(action, ScenarioAction::None) {
                        touch_action = action;
                    }
                }
            }
            if self.wind_touch == Some(t.id) {
                self.set_wind(layout.wind.value(p));
            } else if self.current_touch == Some(t.id) {
                self.set_current(layout.current.value(p));
            }
        }
        if self.wind_touch.is_some_and(|id| !cur_ids.contains(&id)) {
            self.wind_touch = None;
        }
        if self.current_touch.is_some_and(|id| !cur_ids.contains(&id)) {
            self.current_touch = None;
        }
        self.prev_touch_ids = cur_ids;
        if !matches!(touch_action, ScenarioAction::None) {
            return touch_action;
        }

        // --- Keyboard: the wind keeps the arrows, the current keeps the
        // IJKL "second arrows" cluster, 1-4 load the presets.
        if is_key_down(KeyCode::Left) {
            self.env.wind_from_deg -= DIR_RATE * dt;
        }
        if is_key_down(KeyCode::Right) {
            self.env.wind_from_deg += DIR_RATE * dt;
        }
        if is_key_down(KeyCode::Up) {
            self.env.wind_speed = (self.env.wind_speed + WIND_RATE * dt).min(WIND_MAX);
        }
        if is_key_down(KeyCode::Down) {
            self.env.wind_speed = (self.env.wind_speed - WIND_RATE * dt).max(0.0);
        }
        if is_key_down(KeyCode::J) {
            self.env.current_to_deg -= DIR_RATE * dt;
        }
        if is_key_down(KeyCode::L) {
            self.env.current_to_deg += DIR_RATE * dt;
        }
        if is_key_down(KeyCode::I) {
            self.env.current_speed = (self.env.current_speed + CURRENT_RATE * dt).min(CURRENT_MAX);
        }
        if is_key_down(KeyCode::K) {
            self.env.current_speed = (self.env.current_speed - CURRENT_RATE * dt).max(0.0);
        }
        self.env.wind_from_deg = self.env.wind_from_deg.rem_euclid(360.0);
        self.env.current_to_deg = self.env.current_to_deg.rem_euclid(360.0);
        for (key, preset) in
            [KeyCode::Key1, KeyCode::Key2, KeyCode::Key3, KeyCode::Key4].iter().zip(PRESETS.iter())
        {
            if is_key_pressed(*key) {
                self.env = preset.env(self.channel_deg);
            }
        }
        if is_key_pressed(KeyCode::Enter) {
            return ScenarioAction::Apply;
        }
        if is_key_pressed(KeyCode::Escape) {
            return ScenarioAction::Cancel;
        }
        ScenarioAction::None
    }

    pub fn draw(&self, layout: &ScenarioLayout) {
        let ui = layout.ui;
        let card = layout.card;
        let fs = 24.0 * ui;
        draw_rectangle(0.0, 0.0, screen_width(), screen_height(), Color::from_rgba(6, 10, 14, 210));
        draw_rectangle(card.x, card.y, card.w, card.h, Color::from_rgba(14, 22, 30, 255));
        draw_rectangle_lines(card.x, card.y, card.w, card.h, 1.5, DIM_COL);

        let pad = 18.0 * ui;
        draw_text("SCENARIO", card.x + pad, card.y + 34.0 * ui, fs * 1.2, TEXT_COL);
        draw_text(
            "the conditions this run is sailed in - drag a dial, or pick a preset",
            card.x + pad,
            card.y + 56.0 * ui,
            fs * 0.7,
            DIM_COL,
        );

        for (dial, vel, frac, col, label, keys, grabbed) in [
            (
                &layout.wind,
                self.env.wind_vel(),
                self.env.wind_speed / WIND_MAX,
                WIND_COL,
                wind_label(&self.env),
                "dir [<-/->]  speed [up/dn]",
                self.wind_touch.is_some() || self.mouse_claim == Some(0),
            ),
            (
                &layout.current,
                self.env.current_vel(),
                self.env.current_speed / CURRENT_MAX,
                CUR_COL,
                current_label(&self.env),
                "dir [J/L]  speed [I/K]",
                self.current_touch.is_some() || self.mouse_claim == Some(1),
            ),
        ] {
            draw_face(
                dial.cx,
                dial.cy,
                dial.r,
                vel,
                frac,
                col,
                if grabbed { col } else { DIM_COL },
                if grabbed { 2.5 } else { 1.5 },
                true,
                fs,
            );
            centred_text(&label, dial.cx, dial.cy + dial.r + 24.0 * ui, fs * 0.8, col);
            centred_text(keys, dial.cx, dial.cy + dial.r + 44.0 * ui, fs * 0.6, DIM_COL);
        }

        for (rect, preset) in layout.presets.iter().zip(PRESETS.iter()) {
            draw_button(*rect, preset.label, fs * 0.7, ui);
        }
        draw_button(layout.apply, "Apply [Enter]", fs * 0.8, ui);
        draw_button(layout.cancel, "Cancel [Esc]", fs * 0.8, ui);
    }
}

fn wind_label(env: &Env) -> String {
    format!("WIND {:.1} m/s from {:03.0}", env.wind_speed, env.wind_from_deg)
}

fn current_label(env: &Env) -> String {
    format!("CURR {:.1} m/s to {:03.0}", env.current_speed, env.current_to_deg)
}

/// Draw a compass face: background disc, ring, optional N tick, and an
/// arrow along `vel` (the direction the flow MOVES) reaching `frac` of the
/// way to the rim. Shared by the modal's big dials and the HUD panel's
/// small read-only twins, so the two can never drift apart.
#[allow(clippy::too_many_arguments)]
fn draw_face(
    cx: f32,
    cy: f32,
    r: f32,
    vel: Vec2,
    frac: f32,
    col: Color,
    ring: Color,
    ring_w: f32,
    north: bool,
    fs: f32,
) {
    draw_circle(cx, cy, r, Color::from_rgba(10, 20, 30, 150));
    draw_circle_lines(cx, cy, r, ring_w, ring);
    if north {
        draw_text("N", cx - fs * 0.22, cy - r + fs * 0.75, fs * 0.7, DIM_COL);
    }
    if vel.length() > 1e-3 {
        let dir = vec2(vel.x, -vel.y).normalize(); // screen y down
        let head = r * 0.28;
        let tip = vec2(cx, cy) + dir * r * frac.max(0.18);
        let tail = vec2(cx, cy) - dir * r * 0.25;
        draw_line(tail.x, tail.y, tip.x, tip.y, (r * 0.07).clamp(1.5, 3.5), col);
        let n = vec2(-dir.y, dir.x);
        draw_triangle(
            tip + dir * head,
            tip - dir * head * 0.2 + n * head * 0.72,
            tip - dir * head * 0.2 - n * head * 0.72,
            col,
        );
    } else {
        draw_circle(cx, cy, (r * 0.08).clamp(2.0, 3.0), col);
    }
}

fn centred_text(label: &str, cx: f32, y: f32, size: f32, col: Color) {
    let d = measure_text(label, None, size as u16, 1.0);
    draw_text(label, (cx - d.width * 0.5).clamp(4.0, screen_width() - d.width - 4.0), y, size, col);
}

/// A modal button: filled rect, outline, label shrunk to fit (the keel
/// editor's rule — labels must survive narrow viewports).
fn draw_button(rect: Rect, label: &str, size: f32, ui: f32) {
    draw_rectangle(rect.x, rect.y, rect.w, rect.h, Color::from_rgba(30, 40, 50, 255));
    draw_rectangle_lines(rect.x, rect.y, rect.w, rect.h, 1.5, DIM_COL);
    let mut size = size;
    let mut tw = measure_text(label, None, size as u16, 1.0).width;
    while tw > rect.w - 10.0 * ui && size > 8.0 {
        size -= 1.0;
        tw = measure_text(label, None, size as u16, 1.0).width;
    }
    draw_text(label, rect.x + (rect.w - tw) * 0.5, rect.y + rect.h * 0.65, size, TEXT_COL);
}

/// The in-game conditions panel (top-left): where the two HUD dials used
/// to be, now a read-only readout of the same two indicators. Its rect is
/// also the touch target that opens the modal — the touch twin of `V`,
/// and what keeps the settings one tap away on a phone.
///
/// Sized from the actual label strings so it never clips them.
pub fn hud_panel_rect(env: &Env, x: f32, y: f32, fs: f32) -> Rect {
    let r = fs * 0.85;
    let label_w = [wind_label(env), current_label(env)]
        .iter()
        .map(|l| measure_text(l, None, (fs * 0.75) as u16, 1.0).width)
        .fold(0.0, f32::max);
    Rect::new(
        x,
        y,
        fs * 0.5 + r * 2.0 + fs * 0.5 + label_w + fs * 0.5,
        fs * 1.5 + (r * 2.0 + fs * 0.35) * 2.0 + fs * 0.35,
    )
}

pub fn draw_hud_panel(env: &Env, rect: Rect, fs: f32) {
    draw_rectangle(rect.x, rect.y, rect.w, rect.h, PANEL_BG);
    draw_rectangle_lines(rect.x, rect.y, rect.w, rect.h, 1.5, DIM_COL);
    draw_text("SCENARIO", rect.x + fs * 0.5, rect.y + fs * 1.1, fs * 0.75, TEXT_COL);
    let r = fs * 0.85;
    let cx = rect.x + fs * 0.5 + r;
    let row_h = r * 2.0 + fs * 0.35;
    for (i, (vel, frac, col, label)) in [
        (env.wind_vel(), env.wind_speed / WIND_MAX, WIND_COL, wind_label(env)),
        (env.current_vel(), env.current_speed / CURRENT_MAX, CUR_COL, current_label(env)),
    ]
    .iter()
    .enumerate()
    {
        let cy = rect.y + fs * 1.5 + row_h * i as f32 + r;
        draw_face(cx, cy, r, *vel, *frac, *col, DIM_COL, 1.0, false, fs);
        draw_text(label, cx + r + fs * 0.5, cy + fs * 0.28, fs * 0.75, *col);
    }
}
