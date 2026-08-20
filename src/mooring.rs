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
    anchor_fitting, fitting_broken, Anchor, Fairlead, Fitting, Gave, Hull, Line, LineCommand,
    LineState, ShoreKind, LINE_COUNT_MAX, LINE_MBL, LINE_REACH_MAX,
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
}

/// What a press currently owns. One at a time — you cannot haul a line
/// and throw another with the same finger.
#[derive(Clone, Copy, PartialEq)]
enum Grab {
    /// Dragging from a fairlead: (the fairlead, the live pointer point).
    Pass(Fairlead, Vec2),
    /// Holding HAUL (+1) or SLACK (-1).
    Tend(f32),
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
    pub fn new() -> MooringUi {
        MooringUi {
            active: false,
            selected: None,
            armed: None,
            grab: None,
            press_at: Vec2::ZERO,
            queue: VecDeque::new(),
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

    /// Report anything that let go this tick, with the cause sim-core
    /// worked out — a rope that vanishes without explanation is exactly
    /// the kind of silence the notes exist to prevent. Called once per
    /// physics tick, and almost always with an empty slice.
    pub fn report_failures(&mut self, failures: &[(u32, Hull, Gave)]) {
        // The player's own ropes first — but a mooring parting on a
        // berthed boat is worth hearing about too, because she is now
        // lying to one rope fewer and it is very likely your fault.
        if let Some((_, hull, gave)) = failures
            .iter()
            .find(|(_, h, _)| *h == Hull::Player)
            .or_else(|| failures.first())
        {
            match hull {
                Hull::Player => self.say(gave.describe()),
                Hull::Moored(_) => self.say("a mooring gave way on the boat you leaned on"),
            }
        }
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
            .extend(lines.iter().filter(|l| l.hull == Hull::Player && !l.is_fast()).map(|l| l.id));
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
        let mine = cx.lines.iter().filter(|l| l.hull == Hull::Player);
        if reach > LINE_REACH_MAX {
            self.say(&format!("too far to throw - {reach:.0} m, reach is {LINE_REACH_MAX:.0} m"));
        } else if cx.lines.iter().any(|l| l.hull == Hull::Player && l.fairlead == fairlead) {
            self.say(&format!("the {} already has a line on it", fairlead.label()));
        } else if mine.count() >= LINE_COUNT_MAX {
            self.say("no line left to spare - cast one off first");
        } else {
            self.queue.push_back(LineCommand::MakeFast { fairlead, anchor });
        }
        self.armed = None;
        self.grab = None;
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

}

/// Everything the mooring UI needs to know about the world this frame.
pub struct Ctx<'a> {
    pub view: View,
    /// INTERPOLATED boat pose (what is on screen), so handles and ropes
    /// sit on the hull the player can see rather than juddering to the
    /// last physics tick.
    pub boat_pos: Vec2,
    pub boat_heading: f32,
    /// The moored fleet's live poses, in sim-core order — the fleet lies
    /// to real ropes, so its boats move, and their fairleads move with
    /// them.
    pub moored: &'a [(Vec2, f32)],
    pub anchors: &'a [Anchor],
    pub lines: &'a [Line],
    /// Fittings torn out this run — no longer offered, and drawn as the
    /// wreckage they are.
    pub broken: &'a [Fitting],
    pub layout: MooringLayout,
}

impl Ctx<'_> {
    /// Pose of whichever hull a line is made fast to.
    pub fn hull_pose(&self, hull: Hull) -> (Vec2, f32) {
        match hull {
            Hull::Player => (self.boat_pos, self.boat_heading),
            Hull::Moored(i) => {
                *self.moored.get(usize::from(i)).unwrap_or(&(self.boat_pos, self.boat_heading))
            }
        }
    }

    pub fn fairlead_of(&self, hull: Hull, f: Fairlead) -> Vec2 {
        let (pos, heading) = self.hull_pose(hull);
        let l = f.local();
        let (c, s) = (heading.cos(), heading.sin());
        pos + vec2(l.x * c - l.y * s, l.x * s + l.y * c)
    }

    /// Where an anchor is in the world. A cleat or a pole is a fixed
    /// point; a neighbour's fairlead moves with the neighbour.
    pub fn anchor_pos(&self, a: Anchor) -> Vec2 {
        match a {
            Anchor::Shore { pos, .. } => pos,
            Anchor::Boat { hull, fairlead } => self.fairlead_of(hull, fairlead),
        }
    }

    /// Every anchor the crew could plausibly reach right now: the fixed
    /// ones ashore, plus the fairleads of any boat lying close enough to
    /// take a line (rafting up, or a line to your neighbour while you
    /// get sorted out). The boat ones are generated per call rather than
    /// held in a list because they MOVE — and only for hulls near
    /// enough to matter, which is a handful at most.
    pub fn reachable(&self) -> Vec<Anchor> {
        let near = LINE_REACH_MAX + 14.0;
        let mut v: Vec<Anchor> = self
            .anchors
            .iter()
            .copied()
            .filter(|a| (self.anchor_pos(*a) - self.boat_pos).length() <= near)
            .filter(|a| !self.fitting_gone(*a))
            .collect();
        for (i, (p, _)) in self.moored.iter().enumerate() {
            if (*p - self.boat_pos).length() > near {
                continue;
            }
            let hull = Hull::Moored(i as u16);
            v.extend(
                Fairlead::ALL
                    .map(|fairlead| Anchor::Boat { hull, fairlead })
                    .into_iter()
                    .filter(|a| !self.fitting_gone(*a)),
            );
        }
        v
    }

    /// Has this anchor's fitting been torn out?
    pub fn fitting_gone(&self, a: Anchor) -> bool {
        anchor_fitting(a).is_some_and(|f| fitting_broken(self.broken, f))
    }

    /// Has one of OUR OWN deck fittings gone?
    pub fn fairlead_gone(&self, f: Fairlead) -> bool {
        fitting_broken(self.broken, Fitting::Deck(Hull::Player, f))
    }

    /// The player's own fairleads — what the handles and the reach test
    /// are about.
    pub fn fairlead_world(&self, f: Fairlead) -> Vec2 {
        self.fairlead_of(Hull::Player, f)
    }
}

fn nearest_fairlead(p: Vec2, cx: &Ctx, r: f32) -> Option<(Fairlead, f32)> {
    Fairlead::ALL
        .iter()
        .filter(|&&f| !cx.fairlead_gone(f))
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
    let (hull_pos, _) = cx.hull_pose(l.hull);
    let from = cx.fairlead_of(l.hull, l.fairlead);
    let to = cx.anchor_pos(l.anchor);
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
        // The marina's own moorings are not the crew's to work.
        .filter(|l| l.hull == Hull::Player)
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
/// The moored fleet's own ropes: the same model, drawn quieter.
const FLEET_ROPE: Color = Color::new(0.78, 0.77, 0.72, 0.78);
const HANDLE: Color = Color::new(0.95, 0.93, 0.85, 0.9);
const ARMED: Color = Color::new(1.0, 0.82, 0.30, 1.0);
const TOO_FAR: Color = Color::new(1.0, 0.42, 0.36, 1.0);
const REACH_RING: Color = Color::new(1.0, 0.82, 0.30, 0.22);
/// A fitting that has been carried away.
pub const TORN_OUT: Color = Color::new(0.85, 0.35, 0.30, 0.85);

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
pub fn draw_ropes(cx: &Ctx, selected: Option<u32>, visible: impl Fn(Vec2, f32) -> bool) {
    let scale = cx.view.scale;
    let w2s = |p: Vec2| cx.view.w2s(p);
    for l in cx.lines {
        let (hull_pos, _) = cx.hull_pose(l.hull);
        if !visible(hull_pos, 26.0) {
            continue;
        }
        let mine = l.hull == Hull::Player;
        let from = cx.fairlead_of(l.hull, l.fairlead);
        let to = cx.anchor_pos(l.anchor);
        // The fleet's ropes are the same physics but quieter on screen:
        // they are scenery until you disturb one.
        let width = if mine { (0.11 * scale).max(1.6) } else { (0.07 * scale).max(1.0) };
        match l.state {
            LineState::Passing { .. } => {
                // Still going ashore: the rope snakes out toward the
                // cleat, with the heaving line's end running ahead of it.
                let t = l.pass_progress();
                let tip = from + (to - from) * t;
                let pts = rope_points(from, tip, passing_slack(from, to, t), hull_pos);
                polyline(&pts, w2s, width * 0.8, ROPE_SLACK);
                let s = w2s(tip);
                draw_circle(s.x, s.y, (0.25 * scale).max(2.5), ROPE);
            }
            LineState::Fast => {
                let dist = (to - from).length();
                let slack = (l.scope - dist).max(0.0);
                let load = (l.tension / LINE_MBL).clamp(0.0, 1.0).powf(0.4);
                let base = if mine { ROPE } else { FLEET_ROPE };
                let col = if slack > 0.02 {
                    if mine { ROPE_SLACK } else { FLEET_ROPE }
                } else {
                    Color::new(
                        base.r + (ROPE_LOADED.r - base.r) * load,
                        base.g + (ROPE_LOADED.g - base.g) * load,
                        base.b + (ROPE_LOADED.b - base.b) * load,
                        base.a,
                    )
                };
                let pts = rope_points(from, to, slack, hull_pos);
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
        let at = cx.anchor_pos(a);
        if !Fairlead::ALL.iter().any(|&f| (at - cx.fairlead_world(f)).length() <= LINE_REACH_MAX) {
            continue;
        }
        let s = cx.view.w2s(at);
        let r = (0.9 * scale).max(7.0);
        match a {
            Anchor::Shore { kind: ShoreKind::Cleat, .. } => {
                draw_circle_lines(s.x, s.y, r, 2.0, Color::new(0.95, 0.93, 0.85, 0.55))
            }
            Anchor::Shore { kind: ShoreKind::Pole, .. } => {
                draw_circle_lines(s.x, s.y, r, 2.0, Color::new(1.0, 0.85, 0.55, 0.55))
            }
            // A neighbour's own fairlead: a legal place to make fast, and
            // marked differently because taking a line to her drags HER.
            Anchor::Boat { .. } => {
                draw_circle_lines(s.x, s.y, r * 0.8, 2.0, Color::new(0.62, 0.86, 1.0, 0.6))
            }
        }
    }
    for &f in &Fairlead::ALL {
        let s = cx.view.w2s(cx.fairlead_world(f));
        if cx.fairlead_gone(f) {
            // Torn out: the hole is still there, the fitting is not.
            let r = (0.3 * scale).max(4.0);
            draw_circle_lines(s.x, s.y, r, 2.0, TORN_OUT);
            draw_line(s.x - r, s.y - r, s.x + r, s.y + r, 2.0, TORN_OUT);
            continue;
        }
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
