//! 3D chase-cam renderer — a perspective view of the SAME world the
//! top-down renderer draws, built from the same `Scenery` (sim-core stays
//! the single source of truth; the waterline footprint of every hull is
//! exactly `HULL_PTS`, so visuals still match collision at the only plane
//! the physics knows about).
//!
//! Everything vertical in here is COSMETIC, frontend-only guesswork (see
//! `dims`): sim-core is strictly 2D and has no heights to offer. Coordinate
//! convention, stated once in `w3`: sim world (x=east, y=north) maps to a
//! right-handed y-up render space as 3D x = world x, 3D z = -world y, 3D
//! y = height above the waterplane.
//!
//! macroquad has no lighting; shading is baked per-face into vertex colors
//! against a fixed fake sun (`shade`). A single `draw_mesh` call is clamped
//! to 10 000 vertices / 5 000 indices by macroquad's draw-call buffers, so
//! static scenery is chunked by `MeshBook` well under those caps.

use crate::render2d::{Scenery, WorldFrame};
use harbour_sim_core::sim::{HULL_PTS, JETTY_HALF_W, POLE_RADIUS};
use macroquad::models::Vertex;
use macroquad::prelude::*;

/// Vertical dimensions (metres) invented for the 3D view — cosmetic
/// frontend constants, deliberately NOT in sim-core. Sized for the ~12 m
/// cruising-sailboat hull all boats currently share.
mod dims {
    /// Sheer line above the waterline: aft … forward (linear in between).
    pub const FREEBOARD_STERN: f32 = 1.0;
    pub const FREEBOARD_BOW: f32 = 1.4;
    /// Cabin trunk height above the local deck.
    pub const COACHROOF_H: f32 = 0.8;
    /// Mast top above deck; a slim square section, stepped at local (0.6, 0).
    pub const MAST_H: f32 = 14.0;
    pub const MAST_R: f32 = 0.09;
    /// Boom height above deck at the gooseneck.
    pub const BOOM_H: f32 = 1.7;
    /// Floating pontoon freeboard, and how far its box reaches below the
    /// water to hide the seam.
    pub const PONTOON_H: f32 = 0.4;
    pub const UNDERWATER: f32 = 0.3;
    /// Mooring pole top above water.
    pub const POLE_H: f32 = 1.2;
    /// Land heights: grassy road shore, rocky wooded hill, silted bay head.
    pub const ROAD_LAND_H: f32 = 1.2;
    pub const HILL_LAND_H: f32 = 2.5;
    pub const HEAD_LAND_H: f32 = 1.0;
    /// Translucent water decals (ripples, wash, rudder hint) float this far
    /// above the waterplane so they never z-fight it.
    pub const DECAL_LIFT: f32 = 0.02;
    /// How far past the world bounds the waterplane reaches (the horizon).
    pub const WATER_MARGIN: f32 = 600.0;
}

// Chase camera: behind and above the boat, looking a little ahead of it.
// The camera's PLANAR POSITION is the smoothed state (not the yaw): easing
// the position toward the ideal chase point means a small yaw wiggle
// barely moves the camera (the old yaw-lag rig swung the whole 22 m arm
// with every helm correction), a steady turn settles into a natural
// trailing angle, and accelerations make the camera hang back and catch
// up. On top of that, a subtle ambient bob/sway (wind-scaled, cosmetic
// render-clock motion like the ripples) keeps the view alive when the
// boat runs dead straight.
const CHASE_DIST: f32 = 22.0;
const CHASE_HEIGHT: f32 = 8.0;
const CHASE_LOOKAHEAD: f32 = 6.0;
const CHASE_AIM_UP: f32 = 1.5;
const CHASE_POS_TAU: f32 = 0.7; // s, position-easing time constant
/// The eased camera may trail/lead, but never outside this band of the
/// ideal distance (a crash-stop can't lose the boat off-screen).
const CHASE_DIST_BAND: (f32, f32) = (0.6, 1.5);
/// Look-ahead along the boat's actual track (render-side velocity
/// estimate), so the view leads a turn the way your eyes would.
const CHASE_VEL_LOOKAHEAD: f32 = 0.4; // s
const CHASE_VEL_MAX: f32 = 8.0; // m/s cap on the estimate (respawn jumps)
/// Base vertical FOV, widened by up to `CHASE_FOV_SPEED_GAIN` at 3 m/s
/// (about this hull's cruising speed). Speed-coupled FOV is the standard
/// trick for making a straight run READ as motion — the edges of frame
/// stretch as you gather way — and it's what carries the sense of speed
/// on open water, where there's little nearby geometry to sweep past.
const CHASE_FOV_DEG: f32 = 45.0;
const CHASE_FOV_SPEED_GAIN: f32 = 5.0; // degrees at 3 m/s
const CHASE_FOV_TAU: f32 = 1.2; // s — must lag well behind the throttle

/// Sim world (x=east, y=north) + height → render space (y-up, north = -z).
fn w3(p: Vec2, up: f32) -> Vec3 {
    vec3(p.x, up, -p.y)
}

/// Flat per-face shading against a fixed fake sun (macroquad has no
/// lighting; vertex color is the whole lighting model).
fn shade(base: Color, n: Vec3) -> Color {
    let sun = vec3(-0.5, 0.8, -0.3).normalize();
    let f = 0.55 + 0.45 * n.normalize_or_zero().dot(sun).max(0.0);
    Color::new(base.r * f, base.g * f, base.b * f, base.a)
}

/// Deck (sheer) height above water at boat-local x.
fn deck_h(lx: f32) -> f32 {
    let t = ((lx + 5.9) / 11.9).clamp(0.0, 1.0);
    dims::FREEBOARD_STERN + (dims::FREEBOARD_BOW - dims::FREEBOARD_STERN) * t
}

/// Accumulates world-space triangles into as many `Mesh`es as needed to
/// stay under macroquad's per-draw-call clamp (see module docs).
struct MeshBook {
    meshes: Vec<Mesh>,
    verts: Vec<Vertex>,
    idx: Vec<u16>,
}

const CHUNK_VERTS: usize = 8000;
const CHUNK_IDX: usize = 4500;

impl MeshBook {
    fn new() -> MeshBook {
        MeshBook { meshes: Vec::new(), verts: Vec::new(), idx: Vec::new() }
    }

    fn ensure(&mut self, v: usize, i: usize) {
        if self.verts.len() + v > CHUNK_VERTS || self.idx.len() + i > CHUNK_IDX {
            self.flush();
        }
    }

    fn flush(&mut self) {
        if !self.verts.is_empty() {
            self.meshes.push(Mesh {
                vertices: std::mem::take(&mut self.verts),
                indices: std::mem::take(&mut self.idx),
                texture: None,
            });
        }
    }

    /// A flat-shaded quad, corners in order (a b c d), normal `n`.
    fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, n: Vec3, col: Color) {
        self.ensure(4, 6);
        let base = self.verts.len() as u16;
        let sc = shade(col, n);
        for p in [a, b, c, d] {
            self.verts.push(Vertex::new2(p, vec2(0.0, 0.0), sc));
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// A flat-shaded triangle.
    fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, n: Vec3, col: Color) {
        self.ensure(3, 3);
        let base = self.verts.len() as u16;
        let sc = shade(col, n);
        for p in [a, b, c] {
            self.verts.push(Vertex::new2(p, vec2(0.0, 0.0), sc));
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }

    /// An UNSHADED quad (used for the waterplane, which must keep the exact
    /// 2D water colour).
    fn quad_flat(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, col: Color) {
        self.ensure(4, 6);
        let base = self.verts.len() as u16;
        for p in [a, b, c, d] {
            self.verts.push(Vertex::new2(p, vec2(0.0, 0.0), col));
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// Vertical wall along a polyline, from height `y0` up to `y1`, outward
    /// normal = the 2D `out(segment_dir)` mapped into 3D.
    fn wall(&mut self, pts: &[Vec2], y0: f32, y1: f32, out: impl Fn(Vec2) -> Vec2, col: Color) {
        for i in 0..pts.len().saturating_sub(1) {
            let t = (pts[i + 1] - pts[i]).normalize_or_zero();
            let n2 = out(t);
            self.quad(
                w3(pts[i], y0),
                w3(pts[i + 1], y0),
                w3(pts[i + 1], y1),
                w3(pts[i], y1),
                w3(n2, 0.0) - w3(Vec2::ZERO, 0.0),
                col,
            );
        }
    }

    /// Horizontal ribbon between two equal-length polylines at height `y`
    /// (the 3D twin of render2d's `draw_strip`).
    fn ribbon(&mut self, a: &[Vec2], b: &[Vec2], y: f32, col: Color) {
        for i in 0..a.len().min(b.len()).saturating_sub(1) {
            self.quad(w3(a[i], y), w3(a[i + 1], y), w3(b[i + 1], y), w3(b[i], y), Vec3::Y, col);
        }
    }

    /// An axis-free box (prism) over a 2D rectangle given by centre-line
    /// segment `a→b` and half-width `hw`, from `y0` to `y1`: four walls and
    /// a top cap, one base colour (the per-face sun shading separates the
    /// top from the sides on its own).
    fn box_along(&mut self, a: Vec2, b: Vec2, hw: f32, y0: f32, y1: f32, col: Color) {
        let d = (b - a).normalize_or_zero();
        let s = vec2(-d.y, d.x) * hw;
        let (c0, c1, c2, c3) = (a + s, b + s, b - s, a - s);
        let n_left = vec3(s.x, 0.0, -s.y).normalize_or_zero();
        self.quad(w3(c0, y0), w3(c1, y0), w3(c1, y1), w3(c0, y1), n_left, col);
        self.quad(w3(c3, y0), w3(c2, y0), w3(c2, y1), w3(c3, y1), -n_left, col);
        let n_end = vec3(d.x, 0.0, -d.y);
        self.quad(w3(c1, y0), w3(c2, y0), w3(c2, y1), w3(c1, y1), n_end, col);
        self.quad(w3(c0, y0), w3(c3, y0), w3(c3, y1), w3(c0, y1), -n_end, col);
        self.quad(w3(c0, y1), w3(c1, y1), w3(c2, y1), w3(c3, y1), Vec3::Y, col);
    }

    /// One boat hull as a solid: `HULL_PTS` extruded from the waterline up
    /// to the sheer, a deck cap, and a cabin — the same silhouette recipe
    /// for the player and the moored fleet.
    fn boat(&mut self, pos: Vec2, heading: f32, fill: Color, deck: Color, cabin: Color, motor: bool) {
        let (c, s) = (heading.cos(), heading.sin());
        let lw = |lx: f32, ly: f32| -> Vec2 { pos + vec2(lx * c - ly * s, lx * s + ly * c) };
        // Sides: one quad per hull edge, outward normal from the 2D edge
        // (HULL_PTS is CCW, so outward = edge direction rotated -90°).
        for i in 0..HULL_PTS.len() {
            let (ax, ay) = HULL_PTS[i];
            let (bx, by) = HULL_PTS[(i + 1) % HULL_PTS.len()];
            let n2l = vec2(by - ay, -(bx - ax)).normalize_or_zero();
            let n2 = vec2(n2l.x * c - n2l.y * s, n2l.x * s + n2l.y * c);
            self.quad(
                w3(lw(ax, ay), 0.0),
                w3(lw(bx, by), 0.0),
                w3(lw(bx, by), deck_h(bx)),
                w3(lw(ax, ay), deck_h(ax)),
                vec3(n2.x, 0.0, -n2.y),
                fill,
            );
        }
        // Deck cap: a fan over the sheer line.
        let p0 = HULL_PTS[0];
        for i in 1..HULL_PTS.len() - 1 {
            let p1 = HULL_PTS[i];
            let p2 = HULL_PTS[i + 1];
            self.tri(
                w3(lw(p0.0, p0.1), deck_h(p0.0)),
                w3(lw(p1.0, p1.1), deck_h(p1.0)),
                w3(lw(p2.0, p2.1), deck_h(p2.0)),
                Vec3::Y,
                deck,
            );
        }
        // Cabin: the coachroof footprint from the 2D renderer (or the motor
        // cruiser's longer cab), a box on deck.
        let (x0, x1, hw) = if motor { (-3.4, 1.6, 1.2) } else { (-2.6, 0.3, 1.0) };
        let base = deck_h((x0 + x1) * 0.5);
        self.box_along(
            lw((x0 + x1) * 0.5 - (x1 - x0) * 0.5, 0.0),
            lw((x0 + x1) * 0.5 + (x1 - x0) * 0.5, 0.0),
            hw,
            base - 0.05,
            base + dims::COACHROOF_H,
            cabin,
        );
    }
}

/// The 3D renderer: static world meshes built once from `Scenery`, plus a
/// small dynamic mesh rebuilt each frame for the player's boat, plus the
/// chase-camera state.
pub struct Renderer3D {
    statics: Vec<Mesh>,
    /// Eased camera position on the water plane (world metres). This is
    /// the smoothed state — see the CHASE_* notes above.
    cam_pos: Vec2,
    /// Previous frame's boat position + the render-side velocity estimate
    /// it feeds (used for the track look-ahead; the sim's own velocity
    /// isn't handed to the renderer, and this stays purely cosmetic).
    last_boat: Vec2,
    vel: Vec2,
    /// Eased vertical FOV in degrees (speed-coupled — see CHASE_FOV_*).
    fov_deg: f32,
}

impl Renderer3D {
    pub fn new(sc: &Scenery) -> Renderer3D {
        let mut book = MeshBook::new();
        let water = Color::from_rgba(16, 48, 66, 255);

        // Waterplane out to the horizon (unshaded — the 2D water colour).
        let (lo, hi) = (sc.bmin - Vec2::splat(dims::WATER_MARGIN), sc.bmax + Vec2::splat(dims::WATER_MARGIN));
        book.quad_flat(
            w3(vec2(lo.x, lo.y), 0.0),
            w3(vec2(hi.x, lo.y), 0.0),
            w3(vec2(hi.x, hi.y), 0.0),
            w3(vec2(lo.x, hi.y), 0.0),
            water,
        );

        // --- Shores: a wall at the waterline, land ribbons on top (the 3D
        // twins of the 2D land fills; trees/rocks/skerries are deferred).
        let grass = Color::from_rgba(58, 88, 52, 255);
        let rock = Color::from_rgba(98, 100, 96, 255);
        let forest = Color::from_rgba(36, 60, 40, 255);
        let apron = Color::from_rgba(126, 128, 126, 255);
        // Road shore: seaward is the RIGHT of its direction of travel.
        book.wall(&sc.road, 0.0, dims::ROAD_LAND_H, |t| vec2(t.y, -t.x), grass);
        book.ribbon(&sc.road, &sc.road_land, dims::ROAD_LAND_H, grass);
        book.ribbon(
            &sc.road[..sc.n_marina],
            &sc.road_apron[..sc.n_marina],
            dims::ROAD_LAND_H + 0.02,
            apron,
        );
        // Hill shore: seaward is its LEFT.
        book.wall(&sc.hill, 0.0, dims::HILL_LAND_H, |t| vec2(-t.y, t.x), rock);
        book.ribbon(&sc.hill_rock, &sc.hill, dims::HILL_LAND_H, rock);
        book.ribbon(&sc.hill_land, &sc.hill_rock, dims::HILL_LAND_H, forest);
        // Bay head: silt band at the waterline, grass beyond.
        book.wall(&sc.head, 0.0, dims::HEAD_LAND_H, |t| vec2(t.y, -t.x), Color::from_rgba(74, 96, 74, 255));
        book.ribbon(&sc.head, &sc.head_silt, dims::HEAD_LAND_H, Color::from_rgba(74, 96, 74, 255));
        book.ribbon(&sc.head_silt, &sc.head_land, dims::HEAD_LAND_H, grass);

        // --- Pontoon jetties: floating plank boxes.
        let deck = Color::from_rgba(168, 162, 148, 255);
        for j in &sc.jetties {
            book.box_along(
                j.root,
                j.root + j.dir * j.len,
                JETTY_HALF_W,
                -dims::UNDERWATER,
                dims::PONTOON_H,
                deck,
            );
        }

        // --- Mooring poles: slim square piles.
        let pole = Color::from_rgba(92, 64, 40, 255);
        for p in &sc.poles {
            book.box_along(
                *p - vec2(POLE_RADIUS, 0.0),
                *p + vec2(POLE_RADIUS, 0.0),
                POLE_RADIUS,
                -dims::UNDERWATER,
                dims::POLE_H,
                pole,
            );
        }

        // --- The moored fleet: same hulls, quieter colours, no rigs (a
        // forest of 100 masts would bury the skyline the player needs).
        let moored_fills = [
            Color::from_rgba(226, 222, 208, 255),
            Color::from_rgba(212, 216, 220, 255),
            Color::from_rgba(230, 224, 212, 255),
            Color::from_rgba(206, 200, 188, 255),
        ];
        let moored_deck = Color::from_rgba(196, 192, 180, 255);
        let moored_cabin = Color::from_rgba(203, 208, 212, 255);
        for (bi, mb) in sc.moored.iter().enumerate() {
            book.boat(
                mb.pos,
                mb.heading,
                moored_fills[bi % moored_fills.len()],
                moored_deck,
                moored_cabin,
                bi % 4 == 3,
            );
        }

        book.flush();
        Renderer3D {
            statics: book.meshes,
            cam_pos: Vec2::ZERO,
            last_boat: Vec2::ZERO,
            vel: Vec2::ZERO,
            fov_deg: CHASE_FOV_DEG,
        }
    }

    /// Snap the chase camera onto a pose (fresh spawn / reset — no swooping
    /// across the marina to the new boat).
    pub fn snap_to(&mut self, pos: Vec2, heading: f32) {
        self.cam_pos = pos - Vec2::from_angle(heading) * CHASE_DIST;
        self.last_boat = pos;
        self.vel = Vec2::ZERO;
        self.fov_deg = CHASE_FOV_DEG;
    }

    /// Draw the 3D scene for this frame. Sets its own `Camera3D`; the
    /// caller returns to the default camera afterwards for the HUD.
    pub fn draw(&mut self, sc: &Scenery, fr: &WorldFrame, dt: f32) {
        // Perspective FOV is vertical, so a portrait phone would otherwise
        // fill its narrow width with the boat: back the camera off (and
        // lift it a little) as the aspect ratio drops below ~1.15.
        let aspect = screen_width() / screen_height();
        let boost = (1.15 / aspect).clamp(1.0, 1.9);
        let dist = CHASE_DIST * boost;

        // Render-side velocity estimate, itself smoothed (the raw
        // frame-to-frame delta is noisy at high frame rates). Cosmetic
        // only — nothing here feeds back into the sim.
        if dt > 1e-4 {
            let raw = ((fr.pos - self.last_boat) / dt).clamp_length_max(CHASE_VEL_MAX);
            self.vel = self.vel.lerp(raw, 1.0 - (-dt / 0.3).exp());
        }
        self.last_boat = fr.pos;

        // Ease the camera POSITION toward the ideal chase point astern of
        // the boat. Because the state is a position rather than an angle,
        // a small helm correction barely disturbs it, while a sustained
        // turn lets it settle into a trailing quarter view.
        let ideal = fr.pos - Vec2::from_angle(fr.heading) * dist;
        self.cam_pos = self.cam_pos.lerp(ideal, 1.0 - (-dt / CHASE_POS_TAU).exp());
        // Keep the eased position inside a band of the ideal distance, so
        // a crash-stop or a hard acceleration can never lose the boat.
        let mut off = self.cam_pos - fr.pos;
        let len = off.length();
        if len < 1e-3 {
            off = -Vec2::from_angle(fr.heading) * dist;
        } else {
            off *= len.clamp(dist * CHASE_DIST_BAND.0, dist * CHASE_DIST_BAND.1) / len;
        }
        self.cam_pos = fr.pos + off;

        // Ambient motion: a slow bob/sway so the view is never dead still
        // going straight. Scaled by the wind (a calm marina gets a calm
        // camera) — render-clock cosmetics, like the ripples.
        let t = fr.time;
        let sea = (fr.env.wind_speed / 12.0).clamp(0.15, 1.0);
        let bob = (t * 0.9).sin() * 0.22 * sea + (t * 1.37 + 1.1).sin() * 0.10 * sea;
        let sway = (t * 0.7 + 0.4).sin() * 0.5 * sea;
        let sway_v = off.perp().normalize_or_zero() * sway;

        // Aim a little along the boat's actual track, not just its heading:
        // the view leads a turn the way your eyes would.
        let aim = fr.pos
            + Vec2::from_angle(fr.heading) * CHASE_LOOKAHEAD
            + self.vel * CHASE_VEL_LOOKAHEAD;

        // Speed-coupled FOV, eased slowly so it reads as gathering way
        // rather than tracking the throttle lever.
        let want_fov = CHASE_FOV_DEG + CHASE_FOV_SPEED_GAIN * (self.vel.length() / 3.0).min(1.5);
        self.fov_deg += (want_fov - self.fov_deg) * (1.0 - (-dt / CHASE_FOV_TAU).exp());

        set_camera(&Camera3D {
            position: w3(self.cam_pos + sway_v, CHASE_HEIGHT * boost.sqrt() + bob),
            target: w3(aim, CHASE_AIM_UP),
            up: Vec3::Y,
            fovy: self.fov_deg.to_radians(),
            aspect: None,
            projection: Projection::Perspective,
            render_target: None,
            viewport: None,
            z_near: 0.5,
            z_far: 4000.0,
        });

        for m in &self.statics {
            draw_mesh(m);
        }

        // Mooring lines of nearby berths (straight spans — no catenary yet):
        // outboard quarters to the pole tops, inboard quarters to the jetty.
        let moor_line = Color::from_rgba(200, 198, 190, 200);
        for mb in &sc.moored {
            if (mb.pos - fr.pos).length() > 160.0 {
                continue;
            }
            let (mc, ms) = (mb.heading.cos(), mb.heading.sin());
            let ml = |lx: f32, ly: f32| -> Vec2 {
                mb.pos + vec2(lx * mc - ly * ms, lx * ms + ly * mc)
            };
            let fwd = vec2(mc, ms);
            let port = vec2(-ms, mc);
            let (end_x, quarter_x) = if mb.bow_to_jetty { (-5.9, -5.6) } else { (6.0, 4.2) };
            let end_c = mb.pos + fwd * end_x;
            for p in mb.poles {
                let side_sign = if (p - end_c).dot(port) >= 0.0 { 1.0 } else { -1.0 };
                let q = ml(quarter_x, -1.5 * side_sign);
                draw_line_3d(w3(q, deck_h(quarter_x)), w3(p, dims::POLE_H), moor_line);
            }
            let jetty_quarter_x = if mb.bow_to_jetty { 4.2 } else { -5.6 };
            let across = vec2(-mb.out.y, mb.out.x);
            for side in [1.0f32, -1.0] {
                let qw = ml(jetty_quarter_x, 1.5 * side);
                let s = if (qw - mb.jetty_face).dot(across) >= 0.0 { 1.0 } else { -1.0 };
                let a = mb.jetty_face + across * (1.3 * s);
                draw_line_3d(w3(qw, deck_h(jetty_quarter_x)), w3(a, dims::PONTOON_H), moor_line);
            }
        }

        // Ripples: the SAME field the top-down view draws (shared geometry
        // — see render2d::for_each_ripple), laid on the waterplane just
        // above it.
        crate::render2d::for_each_ripple(sc, fr.env, fr.time, |a, b| {
            if (a - fr.pos).length() > 250.0 {
                return;
            }
            draw_line_3d(
                w3(a, dims::DECAL_LIFT),
                w3(b, dims::DECAL_LIFT),
                crate::render2d::RIPPLE_COLOR,
            );
        });

        // --- The player's boat: hull rebuilt through the live pose each
        // frame (draw_mesh has no transform), rig as slim solids/lines.
        let mut book = MeshBook::new();
        book.boat(
            fr.pos,
            fr.heading,
            Color::from_rgba(230, 226, 212, 255),
            Color::from_rgba(216, 212, 198, 255),
            Color::from_rgba(205, 210, 214, 255),
            false,
        );
        // Mast: a slim square prism from the deck to MAST_H.
        let (c, s) = (fr.heading.cos(), fr.heading.sin());
        let bl = |lx: f32, ly: f32| -> Vec2 { fr.pos + vec2(lx * c - ly * s, lx * s + ly * c) };
        let mast_base = bl(0.6, 0.0);
        let mast_dir = vec2(c, s);
        book.box_along(
            mast_base - mast_dir * dims::MAST_R,
            mast_base + mast_dir * dims::MAST_R,
            dims::MAST_R,
            deck_h(0.6),
            deck_h(0.6) + dims::MAST_H,
            Color::from_rgba(188, 190, 196, 255),
        );
        book.flush();
        for m in &book.meshes {
            draw_mesh(m);
        }

        // Boom along the centreline at gooseneck height.
        let rig = Color::from_rgba(120, 124, 132, 255);
        let boom_h = deck_h(0.6) + dims::BOOM_H;
        draw_line_3d(w3(bl(0.6, 0.0), boom_h), w3(bl(-2.0, 0.0), boom_h - 0.15), rig);

        // Rudder hint at the waterline (the blade itself is underwater):
        // the same chord the 2D renderer draws, as a surface decal.
        let (stock, te) = crate::render2d::rudder_chord(fr);
        draw_line_3d(
            w3(stock, dims::DECAL_LIFT),
            w3(te, dims::DECAL_LIFT),
            Color::from_rgba(40, 42, 48, 255),
        );

        // Prop wash: the same streaks the top-down view draws, on the
        // waterplane (shared geometry — see render2d::for_each_wash_streak).
        crate::render2d::for_each_wash_streak(fr, |a, b, foam| {
            draw_line_3d(w3(a, dims::DECAL_LIFT), w3(b, dims::DECAL_LIFT), foam);
        });
    }
}
