//! Frontend for the mooring lines: LINES mode, its handles and controls,
//! and the drawing of the ropes themselves.
//!
//! Mobile-first like the rest of the HUD (see CLAUDE.md): every gesture
//! here works with one finger, and the same press/drag/release path
//! serves the mouse. Unlike the keel editor this mode does NOT freeze the
//! sim — getting a line ashore is something you do with the boat still
//! moving, so the handles have to be usable live.
//!
//! The gesture is deliberately forgiving in two ways, because a fairlead
//! is only a couple of metres from its neighbour and the camera is often
//! zoomed out: a press anywhere near the hull arms the NEAREST fairlead,
//! and the same state machine accepts either a drag (fairlead → anchor)
//! or two taps (fairlead, then anchor), so nobody has to be precise with
//! a moving target.
//!
//! Nothing here touches physics: it turns gestures into `LineCommand`s
//! and hands them to `Sim::tick` through `InputState`, one per tick.

use harbour_sim_core::line::{
    Anchor, AnchorKind, Fairlead, Line, LineCommand, LineState, LINE_COUNT_MAX, LINE_MBL,
    LINE_PASS_SPEED_MAX, LINE_PASS_SPEED_MIN, LINE_REACH_MAX,
};
use macroquad::prelude::*;
use std::collections::VecDeque;

/// The camera as it stood LAST frame. Input runs before the camera block,
/// so screen↔world conversion at input time has to use the previous
/// frame's view — the same reason the pan code carries `last_scale`.
#[derive(Clone, Copy)]
pub struct View {
    pub cam: Vec2,
    pub scale: f32,
    pub sw: f32,
    pub sh: f32,
}

impl View {
    /// Screen back to world — what a release point means in metres.
    pub fn s2w(&self, p: Vec2) -> Vec2 {
        vec2(
            self.cam.x + (p.x - self.sw * 0.5) / self.scale,
            self.cam.y - (p.y - self.sh * 0.5) / self.scale,
        )
    }

    pub fn w2s(&self, p: Vec2) -> Vec2 {
        vec2(
            self.sw * 0.5 + (p.x - self.cam.x) * self.scale,
            self.sh * 0.5 - (p.y - self.cam.y) * self.scale,
        )
    }
}

/// Screen rects of the mooring controls, laid out by the HUD in main.rs
/// (which owns all HUD geometry) and handed back here for hit-testing and
/// drawing.
#[derive(Clone, Copy, Default)]
pub struct MooringLayout {
    pub haul: Rect,
    pub slack: Rect,
    pub cast: Rect,
    /// Track of the line-handling speed setting.
    pub speed: Rect,
}

/// What a press currently owns. One at a time — you cannot haul a line
/// and throw another with the same finger.
#[derive(Clone, Copy, PartialEq)]
enum Grab {
    /// Dragging from a fairlead: (the fairlead, the live pointer point).
    Pass(Fairlead, Vec2),
    /// Holding HAUL (+1) or SLACK (-1).
    Tend(f32),
    /// Dragging the line-handling speed setting.
    Speed,
}

pub struct MooringUi {
    /// LINES mode: handles visible, presses go to the ropes.
    pub active: bool,
    /// The line the tend controls act on.
    pub selected: Option<u32>,
    /// A fairlead picked up but not yet led anywhere — kept between taps
    /// so tap-fairlead-then-tap-anchor works as well as a single drag.
    armed: Option<Fairlead>,
    grab: Option<Grab>,
    /// Where the current grab started, to tell a tap from a drag.
    press_at: Vec2,
    queue: VecDeque<LineCommand>,
    /// Configuration: how fast the crew gets a line ashore (m/s of
    /// connection distance). Rides the input stream — see
    /// `InputState::line_pass_speed`.
    pub pass_speed: f32,
    /// A short message and the time it fades at: why an order was
    /// refused, or what just happened to a rope. An order that silently
    /// does nothing is the worst kind — the reach limit in particular is
    /// invisible until something tells you about it.
    note: Option<(String, f64)>,
    /// Ids of lines that were still going ashore last frame, so one that
    /// vanishes can be reported as a throw that fell short.
    passing: Vec<u32>,
}

/// How long a note stays up.
const NOTE_SECS: f64 = 2.6;

/// How far off a handle a press still counts, in css px — a fat-finger
/// pad, floored so the handles stay hittable when zoomed out.
fn grab_radius(scale: f32) -> f32 {
    (scale * 0.9).clamp(18.0, 42.0)
}

impl MooringUi {
    pub fn new(pass_speed: f32) -> MooringUi {
        MooringUi {
            active: false,
            selected: None,
            armed: None,
            grab: None,
            press_at: Vec2::ZERO,
            queue: VecDeque::new(),
            pass_speed,
            note: None,
            passing: Vec::new(),
        }
    }

    fn say(&mut self, text: &str) {
        self.note = Some((text.to_string(), get_time() + NOTE_SECS));
    }

    /// The message to show right now, if any.
    pub fn note(&self) -> Option<&str> {
        self.note.as_ref().filter(|(_, t)| get_time() < *t).map(|(s, _)| s.as_str())
    }

    /// The tend rate the held controls are commanding right now.
    fn tend_rate(&self) -> f32 {
        match self.grab {
            Some(Grab::Tend(r)) => r,
            _ => 0.0,
        }
    }

    /// The order to feed this tick's `InputState`. Queued one-shots
    /// (pass, cast off) go first; otherwise a held HAUL/SLACK repeats for
    /// as long as it is held, which is how a continuous pull is expressed
    /// in a one-order-per-tick input stream.
    pub fn next_command(&mut self, lines: &[Line]) -> Option<LineCommand> {
        if let Some(cmd) = self.queue.pop_front() {
            return Some(cmd);
        }
        let rate = self.tend_rate();
        let id = self.selected?;
        if rate == 0.0 || !lines.iter().any(|l| l.id == id && l.is_fast()) {
            return None;
        }
        Some(LineCommand::Tend { id, rate })
    }

    /// Forget stale selections once a line is gone (cast off, parted, or
    /// a throw that fell short) — ids are never recycled, so a stale one
    /// can only ever match nothing, but the tend buttons should stop
    /// offering to work a rope that isn't there.
    pub fn prune(&mut self, lines: &[Line]) {
        if self.selected.is_some_and(|id| !lines.iter().any(|l| l.id == id)) {
            self.selected = None;
        }
        // A line that was in the air and is now gone never landed: the
        // boat drifted out of reach while it was being passed.
        if self.passing.iter().any(|id| !lines.iter().any(|l| l.id == *id)) {
            self.say("the line fell short - she drifted out of reach");
        }
        self.passing.clear();
        self.passing
            .extend(lines.iter().filter(|l| !l.is_fast()).map(|l| l.id));
    }

    /// Leaving the mode drops whatever is in hand, mid-gesture.
    pub fn clear_grabs(&mut self) {
        self.grab = None;
        self.armed = None;
    }

    /// A full teardown, for when the `Sim` itself is being replaced
    /// wholesale (R-reset) rather than continued (the keel editor's
    /// Apply, which carries the player's lines across and so must NOT
    /// call this). `clear_grabs` alone leaves two things dangling: a
    /// queued one-shot order would drain into the FRESH sim on the next
    /// tick and rig a rope onto a boat that never asked for one, and a
    /// stale `passing` id would read as a throw that fell short the
    /// instant the new sim reports no such line (found live, 2026-08-22
    /// — CodeRabbit review).
    pub fn reset(&mut self) {
        self.clear_grabs();
        self.selected = None;
        self.queue.clear();
        self.passing.clear();
        self.note = None;
    }

    /// Try to take a fresh press. Returns true if it belongs to us — the
    /// caller then claims the finger/mouse so it can't also pan.
    pub fn press(&mut self, p: Vec2, cx: &Ctx) -> bool {
        if !self.active {
            return false;
        }
        self.press_at = p;
        // Controls first: they sit over the water and would otherwise
        // read as a pan.
        if let Some(selected) = self.selected {
            if cx.layout.haul.contains(p) {
                self.grab = Some(Grab::Tend(1.0));
                return true;
            }
            if cx.layout.slack.contains(p) {
                self.grab = Some(Grab::Tend(-1.0));
                return true;
            }
            if cx.layout.cast.contains(p) {
                self.queue.push_back(LineCommand::CastOff { id: selected });
                // Let go on purpose. Without this, a line cast off while
                // still going ashore vanishes and `prune` reports it as
                // a throw that fell short.
                self.passing.retain(|id| *id != selected);
                self.selected = None;
                self.grab = None;
                return true;
            }
        }
        if cx.layout.speed.contains(p) {
            self.grab = Some(Grab::Speed);
            self.set_speed(p, cx);
            return true;
        }
        let r = grab_radius(cx.view.scale);
        // An armed fairlead + a press on an anchor completes the pass —
        // the two-tap path, for anyone who would rather not drag.
        if let Some(f) = self.armed
            && let Some(a) = self.anchor_near(p, f, cx)
        {
            self.order_pass(f, a, cx);
            return true;
        }
        // Fairlead or rope, whichever is genuinely NEARER — not fairlead
        // first. Every rope's inboard end sits exactly on a fairlead, so
        // a fixed priority made the handle swallow every press near the
        // boat and a rope could not be selected at all (owner report,
        // 2026-08-20).
        let handle = nearest_fairlead(p, cx, r);
        // A rope is a long thin target and a fairlead is a fat round one,
        // so the rope gets the more generous radius — nearest-wins below
        // stops that stealing presses meant for a handle.
        let rope = nearest_line(p, cx, r * 1.4);
        match (handle, rope) {
            (Some((f, df)), Some((_, dr))) if df <= dr => {
                self.arm(f, p);
                true
            }
            (_, Some((id, _))) => {
                self.selected = Some(id);
                self.armed = None;
                self.grab = None;
                true
            }
            (Some((f, _)), None) => {
                self.arm(f, p);
                true
            }
            (None, None) => false,
        }
    }

    /// Pick up a fairlead and start the gesture. Whether the line can
    /// actually be made is `order_pass`'s business, on release.
    fn arm(&mut self, f: Fairlead, p: Vec2) {
        self.armed = Some(f);
        self.grab = Some(Grab::Pass(f, p));
    }

    /// Continue whatever the press grabbed.
    pub fn hold(&mut self, p: Vec2, cx: &Ctx) {
        match self.grab {
            Some(Grab::Pass(f, _)) => self.grab = Some(Grab::Pass(f, p)),
            Some(Grab::Tend(r)) => {
                // Sliding off the button stops the pull, like a real
                // press-and-hold control.
                let still_on = (r > 0.0 && cx.layout.haul.contains(p))
                    || (r < 0.0 && cx.layout.slack.contains(p));
                if !still_on {
                    self.grab = None;
                }
            }
            Some(Grab::Speed) => self.set_speed(p, cx),
            None => {}
        }
    }

    /// Finish the press.
    pub fn release(&mut self, p: Vec2, cx: &Ctx) {
        if let Some(Grab::Pass(f, _)) = self.grab {
            if let Some(a) = self.anchor_near(p, f, cx) {
                self.order_pass(f, a, cx);
            } else if let Some(far) = self.out_of_reach_near(p, f, cx) {
                // Dropped on a real cleat, just too far away to make.
                self.order_pass(f, far, cx);
            } else if (p - self.press_at).length() > 12.0 {
                // A real drag that landed on nothing: cancelled. A tap
                // leaves the fairlead armed for a second tap. If it ended
                // outside the reach ring, say so — "I tried to lead a
                // line further than anyone can throw one" is the most
                // likely thing that just happened, and silence there is
                // what makes the limit feel like a broken control.
                let out = (cx.view.s2w(p) - cx.fairlead_world(f)).length();
                if out > LINE_REACH_MAX {
                    self.say(&format!(
                        "too far to throw - {out:.0} m, reach is {LINE_REACH_MAX:.0} m"
                    ));
                }
                self.armed = None;
            }
        }
        self.grab = None;
    }

    /// Queue a pass, or say why not. The same three rules `tick` applies
    /// (reach, one rope per fairlead, lines in hand) are checked here
    /// FIRST — not to enforce them, which is sim-core's job, but so the
    /// refusal has a reason attached instead of nothing happening.
    fn order_pass(&mut self, fairlead: Fairlead, anchor: Anchor, cx: &Ctx) {
        let reach = (cx.anchor_pos(anchor) - cx.fairlead_world(fairlead)).length();
        let mine = cx.lines.iter();
        if reach > LINE_REACH_MAX {
            self.say(&format!("too far to throw - {reach:.0} m, reach is {LINE_REACH_MAX:.0} m"));
        } else if cx.lines.iter().any(|l| l.fairlead == fairlead) {
            self.say(&format!("the {} already has a line on it", fairlead.label()));
        } else if mine.count() >= LINE_COUNT_MAX {
            self.say("no line left to spare - cast one off first");
        } else {
            self.queue.push_back(LineCommand::MakeFast { fairlead, anchor });
        }
        self.armed = None;
        self.grab = None;
    }

    fn set_speed(&mut self, p: Vec2, cx: &Ctx) {
        let t = ((p.x - cx.layout.speed.x) / cx.layout.speed.w.max(1.0)).clamp(0.0, 1.0);
        let raw = LINE_PASS_SPEED_MIN + t * (LINE_PASS_SPEED_MAX - LINE_PASS_SPEED_MIN);
        self.pass_speed = (raw * 2.0).round() / 2.0; // 0.5 m/s steps
    }

    /// The anchor under `p`, if it is one this fairlead could reach.
    fn anchor_near(&self, p: Vec2, f: Fairlead, cx: &Ctx) -> Option<Anchor> {
        self.anchor_under(p, f, cx, true)
    }

    /// The anchor under `p` that is OUT of reach — so a drop on a cleat
    /// you cannot make says "too far" rather than just going nowhere.
    fn out_of_reach_near(&self, p: Vec2, f: Fairlead, cx: &Ctx) -> Option<Anchor> {
        self.anchor_under(p, f, cx, false)
    }

    fn anchor_under(&self, p: Vec2, f: Fairlead, cx: &Ctx, within: bool) -> Option<Anchor> {
        let r = grab_radius(cx.view.scale);
        let from = cx.fairlead_world(f);
        cx.reachable()
            .into_iter()
            .filter(|a| ((cx.anchor_pos(*a) - from).length() <= LINE_REACH_MAX) == within)
            .map(|a| (a, (cx.view.w2s(cx.anchor_pos(a)) - p).length()))
            .filter(|(_, d)| *d <= r)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(a, _)| a)
    }

    /// 0..1 position of the speed setting on its track.
    pub fn speed_frac(&self) -> f32 {
        (self.pass_speed - LINE_PASS_SPEED_MIN) / (LINE_PASS_SPEED_MAX - LINE_PASS_SPEED_MIN)
    }
}

/// Everything the mooring UI needs to know about the world this frame.
pub struct Ctx<'a> {
    pub view: View,
    /// INTERPOLATED boat pose (what is on screen), so handles and ropes
    /// sit on the hull the player can see rather than juddering to the
    /// last physics tick.
    pub boat_pos: Vec2,
    pub boat_heading: f32,
    pub anchors: &'a [Anchor],
    pub lines: &'a [Line],
    pub layout: MooringLayout,
}

impl Ctx<'_> {
    /// Where an anchor is in the world.
    pub fn anchor_pos(&self, a: Anchor) -> Vec2 {
        a.pos
    }

    /// World position of one of the boat's fairleads, at the pose the
    /// player can actually see.
    pub fn fairlead_world(&self, f: Fairlead) -> Vec2 {
        let l = f.local();
        let (c, s) = (self.boat_heading.cos(), self.boat_heading.sin());
        self.boat_pos + vec2(l.x * c - l.y * s, l.x * s + l.y * c)
    }

    /// Every anchor the crew could plausibly reach right now.
    pub fn reachable(&self) -> Vec<Anchor> {
        let near = LINE_REACH_MAX + 14.0;
        self.anchors
            .iter()
            .copied()
            .filter(|a| (a.pos - self.boat_pos).length() <= near)
            .collect()
    }
}

fn nearest_fairlead(p: Vec2, cx: &Ctx, r: f32) -> Option<(Fairlead, f32)> {
    Fairlead::ALL
        .iter()
        .map(|&f| (f, (cx.view.w2s(cx.fairlead_world(f)) - p).length()))
        .filter(|(_, d)| *d <= r)
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// Distance from `p` to the segment a-b, in the same space.
fn dist_to_segment(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 <= f32::EPSILON { 0.0 } else { ((p - a).dot(ab) / len2).clamp(0.0, 1.0) };
    (p - (a + ab * t)).length()
}

/// The bight the renderer gives a line still going ashore: a heaving
/// line snakes out rather than flying straight. Shared with the hit
/// test below, because measuring a different curve from the one drawn
/// is exactly the bug that comment records.
fn passing_slack(from: Vec2, to: Vec2, t: f32) -> f32 {
    (to - from).length() * t * 0.25
}

/// Screen-space distance from `p` to a rope as DRAWN — including its
/// slack bight. Hit-testing the straight chord instead was a real bug:
/// a slack line is drawn bulging off to one side, so tapping the rope
/// you can see missed it entirely (owner report, 2026-08-20).
fn dist_to_rope(p: Vec2, l: &Line, cx: &Ctx) -> f32 {
    let hull_pos = cx.boat_pos;
    let from = cx.fairlead_world(l.fairlead);
    let to = l.anchor.pos;
    let (a, b) = match l.state {
        LineState::Passing { .. } => (from, from + (to - from) * l.pass_progress()),
        LineState::Fast => (from, to),
    };
    let slack = match l.state {
        LineState::Fast => (l.scope - (to - from).length()).max(0.0),
        LineState::Passing { .. } => passing_slack(from, to, l.pass_progress()),
    };
    rope_points(a, b, slack, hull_pos)
        .windows(2)
        .map(|w| dist_to_segment(p, cx.view.w2s(w[0]), cx.view.w2s(w[1])))
        .fold(f32::MAX, f32::min)
}

fn nearest_line(p: Vec2, cx: &Ctx, r: f32) -> Option<(u32, f32)> {
    cx.lines
        .iter()
        .map(|l| (l.id, dist_to_rope(p, l, cx)))
        .filter(|(_, d)| *d <= r)
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

const ROPE: Color = Color::new(0.87, 0.85, 0.76, 0.95);
const ROPE_SLACK: Color = Color::new(0.72, 0.71, 0.65, 0.85);
const ROPE_LOADED: Color = Color::new(0.95, 0.55, 0.25, 1.0);
const HANDLE: Color = Color::new(0.95, 0.93, 0.85, 0.9);
const ARMED: Color = Color::new(1.0, 0.82, 0.30, 1.0);
const TOO_FAR: Color = Color::new(1.0, 0.42, 0.36, 1.0);
const REACH_RING: Color = Color::new(1.0, 0.82, 0.30, 0.22);

/// Points along a line from `a` to `b` carrying `slack` metres of extra
/// rope. A taut line is the straight chord; a slack one bights out to the
/// side, sagging by the parabolic arc-length relation `L - d =
/// 8h²/(3d)`, i.e. `h = sqrt(3·d·(L-d)/8)` — so the bight grows the way
/// real rope does as you close on the cleat, instead of by an invented
/// curve. In a top-down view the bight has to lie somewhere, so it is
/// pushed to whichever side is away from the boat: rope draped over the
/// deck would read as a mistake.
fn rope_points(a: Vec2, b: Vec2, slack: f32, away_from: Vec2) -> Vec<Vec2> {
    let chord = b - a;
    let d = chord.length();
    if slack <= 0.001 || d < 0.05 {
        return vec![a, b];
    }
    let h = (3.0 * d * slack / 8.0).sqrt().min(d.max(slack) * 0.6);
    let dir = chord / d;
    let mut n = vec2(-dir.y, dir.x);
    let mid = (a + b) * 0.5;
    if n.dot(mid - away_from) < 0.0 {
        n = -n;
    }
    // Quadratic Bezier: a control point 2h out gives a curve h deep.
    let ctrl = mid + n * (2.0 * h);
    (0..=12)
        .map(|i| {
            let t = i as f32 / 12.0;
            let u = 1.0 - t;
            a * (u * u) + ctrl * (2.0 * u * t) + b * (t * t)
        })
        .collect()
}

fn polyline(pts: &[Vec2], w2s: impl Fn(Vec2) -> Vec2, width: f32, col: Color) {
    for w in pts.windows(2) {
        let (a, b) = (w2s(w[0]), w2s(w[1]));
        draw_line(a.x, a.y, b.x, b.y, width, col);
    }
}

/// Every rope in the marina — the player's and the moored fleet's, drawn
/// by one path because they ARE the same thing. Drawn whether or not
/// LINES mode is open: once a line is out it is part of the world.
///
/// `visible` culls against the camera: the marina carries a few hundred
/// ropes and only a berth or two is ever on screen.
pub fn draw_ropes(cx: &Ctx, selected: Option<u32>) {
    let scale = cx.view.scale;
    let w2s = |p: Vec2| cx.view.w2s(p);
    for l in cx.lines {
        let from = cx.fairlead_world(l.fairlead);
        let to = l.anchor.pos;
        let width = (0.11 * scale).max(1.6);
        match l.state {
            LineState::Passing { .. } => {
                // Still going ashore: the rope snakes out toward the
                // cleat, with the heaving line's end running ahead of it.
                let t = l.pass_progress();
                let tip = from + (to - from) * t;
                let pts = rope_points(from, tip, (to - from).length() * t * 0.25, cx.boat_pos);
                polyline(&pts, w2s, width * 0.8, ROPE_SLACK);
                let s = w2s(tip);
                draw_circle(s.x, s.y, (0.25 * scale).max(2.5), ROPE);
            }
            LineState::Fast => {
                let dist = (to - from).length();
                let slack = (l.scope - dist).max(0.0);
                let load = (l.tension / LINE_MBL).clamp(0.0, 1.0).powf(0.4);
                let col = if slack > 0.02 {
                    ROPE_SLACK
                } else {
                    Color::new(
                        ROPE.r + (ROPE_LOADED.r - ROPE.r) * load,
                        ROPE.g + (ROPE_LOADED.g - ROPE.g) * load,
                        ROPE.b + (ROPE_LOADED.b - ROPE.b) * load,
                        1.0,
                    )
                };
                let pts = rope_points(from, to, slack, cx.boat_pos);
                if selected == Some(l.id) {
                    polyline(&pts, w2s, width + 4.0, Color::new(0.4, 0.85, 1.0, 0.35));
                }
                polyline(&pts, w2s, width * (1.0 + load * 0.6), col);
            }
        }
    }
}

/// The mode's handles: fairleads on the hull, and every anchor the boat
/// can currently reach. Only while LINES mode is open.
pub fn draw_handles(ui: &MooringUi, cx: &Ctx) {
    if !ui.active {
        return;
    }
    let scale = cx.view.scale;
    // Anchors in reach of ANY fairlead. Checked against the hull centre
    // with the boat's own half-length folded in, so the ring appears for
    // everything a fairlead could actually make.
    for a in cx.reachable() {
        if !Fairlead::ALL.iter().any(|&f| (a.pos - cx.fairlead_world(f)).length() <= LINE_REACH_MAX)
        {
            continue;
        }
        let s = cx.view.w2s(a.pos);
        let r = (0.9 * scale).max(7.0);
        let col = match a.kind {
            AnchorKind::Cleat => Color::new(0.95, 0.93, 0.85, 0.55),
            AnchorKind::Pole => Color::new(1.0, 0.85, 0.55, 0.55),
        };
        draw_circle_lines(s.x, s.y, r, 2.0, col);
    }
    for &f in &Fairlead::ALL {
        let s = cx.view.w2s(cx.fairlead_world(f));
        let armed = ui.armed == Some(f);
        let r = if armed { (0.42 * scale).max(6.0) } else { (0.28 * scale).max(3.5) };
        draw_circle(s.x, s.y, r, if armed { ARMED } else { HANDLE });
    }
    // The armed fairlead's REACH, drawn as a ring on the water: the
    // honest answer to "why won't this line go on". Everything inside it
    // can be made; the rubber band goes red the moment the pointer
    // leaves it, so a too-long line is visible before you let go rather
    // than after (owner request, 2026-08-20).
    if let Some(f) = ui.armed {
        let at = cx.fairlead_world(f);
        let c = cx.view.w2s(at);
        draw_circle_lines(c.x, c.y, LINE_REACH_MAX * scale, 1.5, REACH_RING);
        if let Some(Grab::Pass(_, p)) = ui.grab {
            let over = (p - c).length() > LINE_REACH_MAX * scale;
            draw_line(c.x, c.y, p.x, p.y, if over { 3.0 } else { 2.0 },
                      if over { TOO_FAR } else { ARMED });
        }
    }
}
