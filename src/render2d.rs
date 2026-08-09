//! Top-down 2D world renderer, extracted from `main.rs` so the same drawing
//! can serve both the full-screen view and (later) a viewport inset.
//!
//! Everything here draws in VIEWPORT-LOCAL css px, 0..w × 0..h: `draw_world`
//! builds its own world→viewport transform from the `(scale, cam)` pair the
//! caller computed, so at full-screen size the output is byte-identical to
//! the pre-extraction code. Nothing in this module touches input or physics.

use harbour_sim_core::boat::BoatDesign;
use harbour_sim_core::sim::{
    Env, HULL_PTS, JETTY_HALF_W, Jetty, MooredBoat, POLE_RADIUS, head_arc, hill_shore, jetties,
    marina_shore_len, moored_boats, pole_positions, road_shore, world_bounds,
};
use macroquad::prelude::*;

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

/// Static scenery, computed once at startup. sim-core is the single source
/// of truth for all of it: what's drawn IS what collides (plus the pure-
/// cosmetic land fills derived from the same shore polylines).
pub struct Scenery {
    pub jetties: Vec<Jetty>,
    pub poles: Vec<Vec2>,
    pub moored: Vec<MooredBoat>,
    pub road: Vec<Vec2>,
    pub hill: Vec<Vec2>,
    pub head: Vec<Vec2>,
    pub n_marina: usize,
    pub road_land: Vec<Vec2>,
    pub road_apron: Vec<Vec2>,
    pub hill_rock: Vec<Vec2>,
    pub hill_land: Vec<Vec2>,
    pub head_silt: Vec<Vec2>,
    pub head_land: Vec<Vec2>,
    pub bmin: Vec2,
    pub bmax: Vec2,
}

impl Scenery {
    pub fn build() -> Scenery {
        let road = road_shore();
        let hill = hill_shore();
        let head = head_arc();
        let (bmin, bmax) = world_bounds();
        // Land fills reach far enough inland to cover the whole view
        // whenever the camera sits against the world clamp.
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
        Scenery {
            jetties: jetties(),
            poles: pole_positions(),
            moored: moored_boats(),
            road,
            hill,
            head,
            n_marina: marina_shore_len(),
            road_land,
            road_apron,
            hill_rock,
            hill_land,
            head_silt,
            head_land,
            bmin,
            bmax,
        }
    }
}

/// Everything the world drawing reads about the CURRENT frame: the
/// interpolated boat pose, the rudder blade angle (same formula as
/// sim-core: positive = trailing edge to port), the SPOOLED engine, the
/// environment, and the render clock (`get_time()`, allowed for cosmetics).
pub struct WorldFrame<'a> {
    pub pos: Vec2,
    pub heading: f32,
    pub blade: f32,
    pub engine: f32,
    pub time: f32,
    pub env: &'a Env,
    pub design: &'a BoatDesign,
}

/// The drifting ripple field, as world-space segments — SHARED between the
/// top-down and 3D renderers so the two views show the same water (and a
/// future tweak can't update one and miss the other). Callback style so
/// each caller applies its own culling and projection.
pub fn for_each_ripple(sc: &Scenery, env: &Env, time: f32, mut f: impl FnMut(Vec2, Vec2)) {
    let drift = env.current_vel() + env.wind_vel() * 0.02;
    let (bw, bh) = (sc.bmax.x - sc.bmin.x, sc.bmax.y - sc.bmin.y);
    for i in 0u32..220 {
        let hsh = i.wrapping_mul(2654435761);
        let fx = (hsh & 0xffff) as f32 / 65535.0;
        let fy = ((hsh >> 16) & 0xffff) as f32 / 65535.0;
        let x = (fx * bw + drift.x * time).rem_euclid(bw) + sc.bmin.x;
        let y = (fy * bh + drift.y * time).rem_euclid(bh) + sc.bmin.y;
        f(vec2(x, y), vec2(x + 1.4, y));
    }
}

/// The colour every ripple streak is drawn in.
pub const RIPPLE_COLOR: Color = Color::new(120.0 / 255.0, 170.0 / 255.0, 190.0 / 255.0, 26.0 / 255.0);

/// The prop-wash foam streaks, as world-space segments + per-streak colour
/// — shared between the renderers like the ripples. Streaks are driven by
/// the SPOOLED engine (they fade with the lag, not the lever): ahead the
/// slipstream leaves the stern along the deflected blade; astern it boils
/// forward along both quarters.
pub fn for_each_wash_streak(fr: &WorldFrame, mut f: impl FnMut(Vec2, Vec2, Color)) {
    if fr.engine.abs() <= 0.05 {
        return;
    }
    let (c, s) = (fr.heading.cos(), fr.heading.sin());
    let bw = |p: Vec2| -> Vec2 { fr.pos + vec2(p.x * c - p.y * s, p.x * s + p.y * c) };
    let (engine, blade) = (fr.engine, fr.blade);
    for i in 0u32..8 {
        let hsh = i.wrapping_mul(2654435761);
        let fy = ((hsh & 0xffff) as f32 / 65535.0) - 0.5;
        let phase = ((hsh >> 16) & 0xffff) as f32 / 65535.0;
        let ph = (fr.time * 1.8 + phase).fract();
        let alpha = engine.abs().min(1.0) * (1.0 - ph) * 0.5;
        let foam = Color::new(0.75, 0.88, 0.92, alpha);
        let (a, b) = if engine > 0.0 {
            let dir = vec2(-blade.cos(), blade.sin());
            let start = vec2(-6.0, fy * 1.1);
            let p = start + dir * (ph * (1.5 + 2.2 * engine));
            (p, p + dir * 0.7)
        } else {
            let side_y = if i % 2 == 0 { 1.0 } else { -1.0 };
            let start = vec2(-5.2, side_y * (1.4 + 0.6 * fy.abs()));
            let p = start + vec2(ph * (2.0 - 2.5 * engine), 0.0);
            (p, p + vec2(0.6, 0.0))
        };
        f(bw(a), bw(b), foam);
    }
}

/// The rudder chord in world space (stock at the active design's blade
/// position, trailing edge swung by `fr.blade`) — shared so both views
/// draw the same blade the physics uses.
pub fn rudder_chord(fr: &WorldFrame) -> (Vec2, Vec2) {
    let (c, s) = (fr.heading.cos(), fr.heading.sin());
    let bw = |p: Vec2| -> Vec2 { fr.pos + vec2(p.x * c - p.y * s, p.x * s + p.y * c) };
    let stock_x = fr.design.rudder.x + fr.design.rudder.chord / 2.0;
    let te = vec2(
        stock_x - fr.design.rudder.chord * fr.blade.cos(),
        fr.design.rudder.chord * fr.blade.sin(),
    );
    (bw(vec2(stock_x, 0.0)), bw(te))
}

/// Draw the whole top-down world into a `w × h` css-px viewport with the
/// given px-per-metre `scale`, camera-centred on `cam` (world metres).
pub fn draw_world(w: f32, h: f32, scale: f32, cam: Vec2, sc: &Scenery, fr: &WorldFrame) {
    let w2s = |p: Vec2| -> Vec2 {
        vec2(w * 0.5 + (p.x - cam.x) * scale, h * 0.5 - (p.y - cam.y) * scale)
    };
    let vis_hw = w * 0.5 / scale;
    let vis_hh = h * 0.5 / scale;
    // Cheap visibility cull for the marina's many static props.
    let vis_r = vec2(vis_hw, vis_hh).length();
    let visible = |p: Vec2, r: f32| (p - cam).length() < vis_r + r;

    // --- Water -------------------------------------------------------
    // Deep-water backdrop over the whole viewport (the full-screen caller
    // clears to the same colour; in an inset this IS the backdrop).
    draw_rectangle(0.0, 0.0, w, h, Color::from_rgba(12, 38, 54, 255));
    let water = Color::from_rgba(16, 48, 66, 255);
    // One strip covers the channel AND the widening sea past the
    // entrance (the shore polylines continue out along the sea coast);
    // a fan over the head arc fills the rounded bay head.
    draw_strip(&sc.road, &sc.hill, w2s, water);
    for i in 1..sc.head.len() - 1 {
        draw_triangle(w2s(sc.head[0]), w2s(sc.head[i]), w2s(sc.head[i + 1]), water);
    }

    // Cosmetic ripples: short streaks drifting with the current (and a
    // touch of wind), wrapped over the world box. Purely render-side —
    // the land fills drawn next cover the strays that land ashore.
    // (Geometry shared with the 3D view — see `for_each_ripple`.)
    for_each_ripple(sc, fr.env, fr.time, |a, b| {
        if !visible(a, 2.0) {
            return;
        }
        let (a, b) = (w2s(a), w2s(b));
        draw_line(a.x, a.y, b.x, b.y, 1.5, RIPPLE_COLOR);
    });

    // --- Shore (Hinsholmen look: road side NW, wooded hill SE) --------
    // Deterministic scatter helper for trees: same hash idiom as the
    // ripples, but static (no time term) — pure scenery.
    let hash01 = |i: u32, salt: u32| -> (f32, f32, f32) {
        let hsh = i.wrapping_add(salt).wrapping_mul(2654435761);
        (
            (hsh & 0xffff) as f32 / 65535.0,
            ((hsh >> 16) & 0x7fff) as f32 / 32767.0,
            ((hsh >> 8) & 0xff) as f32 / 255.0,
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
    draw_strip(&sc.road, &sc.road_land, w2s, grass);
    draw_strip(
        &sc.road[..sc.n_marina],
        &sc.road_apron[..sc.n_marina],
        w2s,
        Color::from_rgba(126, 128, 126, 255),
    );
    for i in 0..sc.road.len() - 1 {
        let (sa, sb) = (sc.road[i], sc.road[i + 1]);
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
    draw_strip(&sc.hill_rock, &sc.hill, w2s, rock);
    draw_strip(&sc.hill_land, &sc.hill_rock, w2s, forest);
    for i in 0..sc.hill.len() - 1 {
        let (sa, sb) = (sc.hill[i], sc.hill[i + 1]);
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
    draw_strip(&sc.head, &sc.head_silt, w2s, Color::from_rgba(74, 96, 74, 255));
    draw_strip(&sc.head_silt, &sc.head_land, w2s, grass);
    // The skerry line closing the open sea (the boundary polyline's
    // segment between the two coasts' far ends): a chain of rocky
    // islets along the world's edge.
    {
        let (a, b) = (sc.road[sc.road.len() - 1], sc.hill[sc.hill.len() - 1]);
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
    for j in &sc.jetties {
        let mid = j.root + j.dir * (j.len * 0.5);
        if !visible(mid, j.len * 0.5 + 6.0) {
            continue;
        }
        let side = j.side();
        let (r0, r1) = (j.root + side * JETTY_HALF_W, j.root - side * JETTY_HALF_W);
        let (t0, t1) = (r0 + j.dir * j.len, r1 + j.dir * j.len);
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

    // --- Moored boats (static in sim-core) + their mooring lines ------
    let moor_line = Color::from_rgba(200, 198, 190, 200);
    let moored_line_col = Color::from_rgba(46, 48, 54, 255);
    let moored_fills = [
        Color::from_rgba(226, 222, 208, 255),
        Color::from_rgba(212, 216, 220, 255),
        Color::from_rgba(230, 224, 212, 255),
        Color::from_rgba(206, 200, 188, 255),
    ];
    for (bi, mb) in sc.moored.iter().enumerate() {
        if !visible(mb.pos, 16.0) {
            continue;
        }
        let (mc, ms) = (mb.heading.cos(), mb.heading.sin());
        let ml =
            |lx: f32, ly: f32| -> Vec2 { w2s(mb.pos + vec2(lx * mc - ly * ms, lx * ms + ly * mc)) };
        let fwd = vec2(mc, ms);
        let port = vec2(-ms, mc);

        // Crossed lines from the outboard end's quarters to its pole
        // pair — the classic Swedish pole berth from the photos.
        let (end_x, quarter_x) = if mb.bow_to_jetty { (-5.9, -5.6) } else { (6.0, 4.2) };
        let end_c = mb.pos + fwd * end_x;
        for p in mb.poles {
            let side_sign = if (p - end_c).dot(port) >= 0.0 { 1.0 } else { -1.0 };
            let q = ml(quarter_x, -1.5 * side_sign);
            let pw = w2s(p);
            draw_line(q.x, q.y, pw.x, pw.y, (0.07 * scale).max(1.0), moor_line);
        }
        // Short breast lines from the jetty end's quarters to the face.
        let jetty_quarter_x = if mb.bow_to_jetty { 4.2 } else { -5.6 };
        let across = vec2(-mb.out.y, mb.out.x);
        for side in [1.0f32, -1.0] {
            let qw = mb.pos
                + vec2(
                    jetty_quarter_x * mc - 1.5 * side * ms,
                    jetty_quarter_x * ms + 1.5 * side * mc,
                );
            let s = if (qw - mb.jetty_face).dot(across) >= 0.0 { 1.0 } else { -1.0 };
            let a = w2s(mb.jetty_face + across * (1.3 * s));
            let q = w2s(qw);
            draw_line(q.x, q.y, a.x, a.y, (0.07 * scale).max(1.0), moor_line);
        }

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
            draw_triangle(
                c0,
                ml(cab[1].0, cab[1].1),
                ml(cab[2].0, cab[2].1),
                Color::from_rgba(198, 202, 206, 255),
            );
            draw_triangle(
                c0,
                ml(cab[2].0, cab[2].1),
                ml(cab[3].0, cab[3].1),
                Color::from_rgba(198, 202, 206, 255),
            );
            for i in 0..4 {
                let a = ml(cab[i].0, cab[i].1);
                let b = ml(cab[(i + 1) % 4].0, cab[(i + 1) % 4].1);
                draw_line(a.x, a.y, b.x, b.y, (0.08 * scale).max(1.0), moored_line_col);
            }
        } else {
            let ch = [(-2.6, 1.0), (0.3, 1.0), (0.3, -1.0), (-2.6, -1.0)];
            let c0 = ml(ch[0].0, ch[0].1);
            draw_triangle(
                c0,
                ml(ch[1].0, ch[1].1),
                ml(ch[2].0, ch[2].1),
                Color::from_rgba(203, 208, 212, 255),
            );
            draw_triangle(
                c0,
                ml(ch[2].0, ch[2].1),
                ml(ch[3].0, ch[3].1),
                Color::from_rgba(203, 208, 212, 255),
            );
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
    for p in &sc.poles {
        if !visible(*p, 1.0) {
            continue;
        }
        let sp = w2s(*p);
        let r = (POLE_RADIUS * scale).max(2.0);
        draw_circle(sp.x, sp.y, r + 1.0, pole_rim);
        draw_circle(sp.x, sp.y, r, pole_fill);
    }

    // --- Boat --------------------------------------------------------
    let (c, s) = (fr.heading.cos(), fr.heading.sin());
    let bl =
        |lx: f32, ly: f32| -> Vec2 { w2s(fr.pos + vec2(lx * c - ly * s, lx * s + ly * c)) };
    let hull_fill = Color::from_rgba(230, 226, 212, 255);
    let hull_line = Color::from_rgba(40, 42, 48, 255);

    // Cosmetic prop wash (geometry shared with the 3D view — see
    // `for_each_wash_streak`).
    for_each_wash_streak(fr, |a, b, foam| {
        let (a, b) = (w2s(a), w2s(b));
        draw_line(a.x, a.y, b.x, b.y, (0.14 * scale).max(1.0), foam);
    });

    // Rudder blade: stock at the ACTIVE DESIGN's blade position (each
    // preset carries its real boat's rudder — see `RudderDesign` in
    // boat.rs; same values the physics uses), drawn BEFORE the hull
    // fill so the root reads as under the counter and only the swung
    // part shows past it. Stock at the blade's leading edge, the
    // drawn line is the chord (shared geometry: `rudder_chord`).
    let (stock, te) = rudder_chord(fr);
    let rp = w2s(stock);
    let tep = w2s(te);
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
    draw_triangle(
        ch0,
        bl(ch[1].0, ch[1].1),
        bl(ch[2].0, ch[2].1),
        Color::from_rgba(205, 210, 214, 255),
    );
    draw_triangle(
        ch0,
        bl(ch[2].0, ch[2].1),
        bl(ch[3].0, ch[3].1),
        Color::from_rgba(205, 210, 214, 255),
    );
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
}
