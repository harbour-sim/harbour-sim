//! Named boat designs: the parameter bundle the keel editor manipulates
//! and `Sim::new_with_design` consumes — the underwater lateral-area
//! profile, the rudder blade, and the displacement. The four presets are
//! named after real
//! boats (published specs and sources in `docs/reference-boats.md`), and
//! each preset's curve is drawn against its boat's actual draft: the
//! profile's value at a station is the local depth of underwater lateral
//! plane (m²/m = m), so no preset may paint deeper than its boat draws
//! (unit-tested below).
//!
//! This is NOT a ship-type abstraction (see CLAUDE.md's Roadmap): the hull
//! outline, windage and engine are still the sim's single ~38 ft
//! sailboat. A `BoatDesign` is the set of parameters that already vary
//! between the real 38-footers the presets are named after, layered onto
//! that one shared hull — which is also the honest limit of the naming:
//! a preset gives you the named boat's keel plan, rudder and weight, not
//! its whole hull.

use crate::keel::KeelProfile;
use glam::Vec2;

/// How many blades the boat steers with and where they sit athwartships
/// (2026-09-11, added with the Beneteau Oceanis 38.1 preset). The one
/// number that matters is the lateral OFFSET: a blade on the centreline
/// stands in the propeller race and a blade a metre outboard of it does
/// not, which is the whole reason a twin-ruddered boat has no steerage
/// from a burst of ahead power and has to be handled on her momentum
/// instead. sim.rs derives that from the offset against the race's own
/// radius rather than carrying a "has prop wash" flag, so the two can
/// never disagree.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RudderLayout {
    /// Number of blades: 1 for a single rudder, 2 for twins.
    blades: u8,
    /// Lateral offset of each blade from the centreline (m). Zero for a
    /// single rudder; the half-separation for twins.
    offset: f32,
}

impl RudderLayout {
    /// One blade on the centreline — the configuration every boat here
    /// had before 2026-09-11, and still the one that lets a burst of
    /// ahead power steer a stopped boat (the blade stands in the prop
    /// race; see `K_WASH` and the wash fractions in sim.rs).
    pub const SINGLE: RudderLayout = RudderLayout { blades: 1, offset: 0.0 };

    /// Two blades, each `offset` metres off the centreline — one to
    /// port, one to starboard. It is what takes both blades OUT of the
    /// propeller race, which is the whole handling difference and is
    /// derived, not declared (sim.rs measures each blade against the
    /// race's own radius).
    ///
    /// **Panics** unless `offset` is finite and strictly positive
    /// (CodeRabbit review, 2026-09-12). `BoatDesign` is public, so a
    /// layout built outside this file reaches `tick`'s force path
    /// unchecked: a NaN or infinite offset would put NaN into a blade's
    /// world position and silently poison the whole rigid body, and zero
    /// would stack both blades on the centreline — a "twin" rudder that
    /// is really one blade of double area sitting in the prop race,
    /// i.e. the exact opposite of what this type exists to express.
    /// One comparison catches all four cases: NaN, negatives and zero
    /// all fail `> 0.0`, and `< INFINITY` takes the last one. Plain
    /// comparisons rather than `is_finite`, so this stays a `const fn`
    /// and a bad literal fails to COMPILE in a const context.
    pub const fn twin(offset: f32) -> RudderLayout {
        assert!(
            offset > 0.0 && offset < f32::INFINITY,
            "twin rudder offset must be finite and strictly positive"
        );
        RudderLayout { blades: 2, offset }
    }

    /// How many blades this layout has.
    pub fn blades(&self) -> usize {
        self.blades as usize
    }

    /// Half the separation between the blades (m); 0 for a single rudder.
    pub fn offset(&self) -> f32 {
        self.offset
    }

    /// The blades' lateral offsets, in the boat's own (fwd, side) frame —
    /// `side` is port, so a twin's entries are ±offset. Fixed-size so the
    /// derived foil stays `Copy`: only the first `blades()` entries are
    /// meaningful.
    pub fn offsets(&self) -> [f32; 2] {
        match self.blades {
            1 => [0.0, 0.0],
            _ => [-self.offset, self.offset],
        }
    }
}

/// The rudder blade of a [`BoatDesign`] (2026-08-04, previously the
/// shared `RUDDER_*` constants in sim.rs sized from the O'Day 39 alone):
/// position and dimensions, from which sim.rs derives the foil's area,
/// aspect ratio and post-stall drag ceiling. Published blade dimensions
/// essentially don't exist for production boats (the O'Day's, our anchor,
/// came from a replacement-rudder listing), so the other presets' blades
/// are DERIVED — from each boat's rudder type, its own painted keel
/// profile, and the rudder-as-%-of-lateral-plane cross-check — with the
/// derivation documented on each preset. The editor displays but does not
/// yet edit these (follow-up work).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RudderDesign {
    /// Blade centre position along the hull (local x, m — negative aft).
    /// Both blades of a twin installation share it: they sit abreast of
    /// each other, not in line.
    pub x: f32,
    /// Fore-aft chord (m). Real blades taper; this is the mean chord.
    pub chord: f32,
    /// Depth (m) below the hull's local baseline draught.
    pub depth: f32,
    /// Whether the hull (or skeg) above the blade root acts as an end
    /// plate, mirroring the blade and doubling its effective aspect
    /// ratio — true for spade and skeg-hung rudders tucked under the
    /// hull, FALSE for a transom-hung blade whose root breaks the surface
    /// with only air above it (no plate, no mirror): that's a genuine
    /// physical difference of rudder TYPE, not a tuning knob, and it's
    /// why a barn-door outboard rudder has a mushier lift slope than a
    /// spade of the same area.
    pub root_endplated: bool,
    /// One blade on the centreline, or two set out either side of it —
    /// see [`RudderLayout`]. Everything that follows from twins (no prop
    /// wash over the blades, each blade seeing its own inflow as the
    /// hull spins, more total area at a shorter span) is DERIVED from
    /// this offset in sim.rs rather than declared here.
    pub layout: RudderLayout,
}

impl RudderDesign {
    /// Area of ONE blade (m²). See [`RudderDesign::total_area`] for what
    /// the boat actually drags around.
    pub fn area(&self) -> f32 {
        self.chord * self.depth
    }

    /// Total blade area of the installation (m²) — one blade's area times
    /// the blade count. This is the figure that belongs in a wetted-area
    /// sum or a rudder-as-%-of-lateral-plane cross-check; the per-blade
    /// `area` is what each foil's own lift and drag are computed on.
    pub fn total_area(&self) -> f32 {
        self.area() * self.layout.blades() as f32
    }

    /// Effective aspect ratio: geometric depth/chord, doubled when the
    /// root is end-plated (see `root_endplated`). Sets both the lift
    /// slope `2π·AR/(AR+2)` and, via `flat_plate_cd`, the post-stall
    /// drag ceiling — one number, so the two can't disagree about the
    /// blade's three-dimensionality.
    pub fn aspect_ratio(&self) -> f32 {
        let mirror = if self.root_endplated { 2.0 } else { 1.0 };
        mirror * self.depth / self.chord
    }
}

/// The tunable design parameters of the simulated boat.
#[derive(Clone, Debug, PartialEq)]
pub struct BoatDesign {
    /// Underwater lateral-area distribution along the hull (see
    /// [`KeelProfile`]). Value = local depth of lateral plane below the
    /// waterline (m²/m = m), capped at the boat's real draft. Does NOT
    /// include the rudder (see `rudder` — a movable foil, not fixed
    /// area).
    pub keel: KeelProfile,
    /// The rudder blade — see [`RudderDesign`].
    pub rudder: RudderDesign,
    /// Displacement (kg). Sets the rigid body's total mass; the mass
    /// DISTRIBUTION (centre of mass, radius of gyration) still comes from
    /// the hull shape — making those adjustable too is agreed follow-up
    /// work, not part of this struct yet.
    pub displacement_kg: f32,
}

impl BoatDesign {
    /// **Hallberg-Rassy 38** (Olle Enderlein / Christoph Rassy, Sweden,
    /// 1977–1986, 202 built) — the default design: the middle
    /// configuration between the other two presets, a moderate fin keel
    /// with the rudder hung on a substantial skeg.
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA 11.58 m, beam 3.47 m, draft ≈1.75 m, displacement 18,739 lb ≈
    /// 8,500 kg, ballast ratio ≈44% (encapsulated iron). The curve: a
    /// longish fin (root chord ≈2.4 m) just aft of the hull centre at the
    /// full 1.75 m draft, a marked skeg ahead of the rudder post, and a
    /// canoe body fading to nothing at the bow — net area ≈8.4 m², CLR
    /// ≈0.6 m aft of centre (close to the retired hand-tuned default:
    /// area 11.9 m², CLR −0.87 m), aft-biased enough to weathervane.
    pub fn hallberg_rassy_38() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-4.8, 0.0), // aft waterline ending (LWL 9.50 m of 11.58 LOA)
                    Vec2::new(-4.75, 0.3),
                    Vec2::new(-4.35, 1.1), // skeg ahead of the rudder post
                    Vec2::new(-3.9, 0.6),
                    Vec2::new(-2.2, 0.6),
                    Vec2::new(-1.8, 1.75), // fin, at the real 1.75 m draft
                    Vec2::new(0.6, 1.75),
                    Vec2::new(1.0, 0.6),
                    Vec2::new(3.6, 0.45),
                    Vec2::new(4.7, 0.0), // forward waterline ending
                    Vec2::new(6.0, 0.0),
                ],
            },
            // Skeg-hung blade immediately abaft this preset's own painted
            // skeg (the −4.35 bump above), trailing edge reaching the aft
            // waterline ending: moderate dimensions read from the boat's
            // profile drawing against its 1.75 m draft — blade ≈0.74 m²,
            // ≈7% of the total lateral plane, the low blade fraction a
            // skeg boat should have (the skeg itself is fixed area,
            // already painted in the curve). Root end-plated by hull +
            // skeg.
            rudder: RudderDesign { x: -4.6, chord: 0.55, depth: 1.35, root_endplated: true, layout: RudderLayout::SINGLE },
            displacement_kg: 8_500.0,
        }
    }

    /// **O'Day 39** (Philippe Briand, USA, from 1982) — the modern
    /// fin-keel cruiser/racer preset: deep fin, spade rudder, and already
    /// this repo's reference boat for the hull dimensions and the anchor
    /// for rudder sizing (its blade is the one with real published
    /// dimensions — see `rudder` below).
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA ≈12.0 m, LWL 10.21 m, beam 3.83 m, draft 1.93 m (standard
    /// keel), displacement 18,000 lb ≈ 8,165 kg, ballast 2,994 kg. The
    /// curve: a short fin at the full 1.93 m draft over a thin canoe-body
    /// baseline — net area ≈6.6 m², among the smallest of the presets, which is
    /// the point of a fin keel: concentrating the area near the pivot
    /// trades away yaw damping (cubic in distance) much faster than area.
    ///
    /// The fin sits CENTERED SLIGHTLY AFT of the hull centre (−1.4..+0.2,
    /// centroid ≈−0.6 m): the older unnamed fin preset sat exactly on the
    /// centre, which read as too far forward against real fin-keeler
    /// profile drawings — the root chord belongs around/abaft the mast
    /// (≈40% LOA from the bow), not symmetric about amidships.
    pub fn oday_39() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-5.2, 0.0), // aft waterline ending (LWL 10.21 m of ~12.0 LOA)
                    Vec2::new(-5.15, 0.1),
                    Vec2::new(-1.7, 0.55),
                    Vec2::new(-1.4, 1.93), // fin, at the real 1.93 m draft
                    Vec2::new(0.2, 1.93),
                    Vec2::new(0.5, 0.55),
                    Vec2::new(3.5, 0.35),
                    Vec2::new(4.7, 0.1),
                    Vec2::new(5.0, 0.0), // forward waterline ending
                    Vec2::new(6.0, 0.0),
                ],
            },
            // The anchor blade — the ONE with real published dimensions
            // (replacement-rudder listing): ~5 ft (1.52 m) deep, chord
            // tapering 28 in head to 20 in tip, mean ≈0.61 m — 0.93 m²,
            // ≈11.6% of the total lateral plane (the ~10% rule of thumb's
            // independent cross-check). Position: a spade stands just
            // inside the aft end of the waterline — which, since the
            // profiles carry real overhangs, is the curve's own aft
            // ending at −5.2: trailing edge there, centre at −4.9.
            rudder: RudderDesign { x: -4.9, chord: 0.61, depth: 1.52, root_endplated: true, layout: RudderLayout::SINGLE },
            displacement_kg: 8_165.0,
        }
    }

    /// **Elan Impression 394** (Rob Humphreys, Slovenia, from 2012) — the
    /// modern-cruiser preset: shallow flat-bottomed canoe body, cast-iron
    /// fin, deep single spade rudder (which maps onto the sim's stern
    /// blade), the most agile configuration of the four.
    ///
    /// Published specs (see `docs/reference-boats.md` for sources and the
    /// D/L cross-check that validates them): LOA 11.90 m, LWL 10.01 m,
    /// beam 3.91 m, draft 1.80 m (standard keel; 1.50 m shoal option),
    /// displacement 8,000 kg, ballast 2,479 kg cast iron. The curve: a
    /// ~1.4 m-chord iron fin at the full 1.80 m draft, centred slightly
    /// aft of the hull centre like the O'Day's (root around/abaft the
    /// mast), over a MARKEDLY shallower canoe body than the older boats —
    /// the modern flat underbody carries only ~0.4 m of lateral depth
    /// amidships and runs out to a shallow, wide stern with no skeg. Net
    /// area ≈5.4 m², the smallest of the presets, with the least yaw
    /// damping — despite drawing LESS water than the O'Day (1.80 m vs
    /// 1.93 m): the agility comes from stripping lateral plane off the
    /// hull ends, not from a deeper fin.
    pub fn elan_impression_394() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-5.05, 0.0), // aft waterline ending (LWL 10.01 m of 11.90 LOA)
                    Vec2::new(-5.0, 0.12), // shallow run under the wide stern
                    Vec2::new(-1.6, 0.38),
                    Vec2::new(-1.3, 1.8), // fin, at the real 1.80 m draft
                    Vec2::new(0.1, 1.8),
                    Vec2::new(0.4, 0.42),
                    Vec2::new(3.2, 0.32),
                    Vec2::new(4.6, 0.08),
                    Vec2::new(4.95, 0.0), // forward waterline ending
                    Vec2::new(6.0, 0.0),
                ],
            },
            // "Deep single spade rudder" (the phrase every source uses):
            // tip reaching near the 1.80 m keel tip from the shallow
            // (~0.35 m) hull exit → ≈1.65 m deep, mean chord ≈0.60 m —
            // 0.99 m², ≈15% of the boat's (small) lateral plane and the
            // highest aspect ratio of the four (AR ≈ 5.5): the modern
            // pattern of a big, deep, high-slope blade doing
            // proportionally more of the boat's steering and tracking.
            // Same spade position rule as the O'Day: trailing edge at the
            // curve's own aft waterline ending (−5.05), centre −4.75.
            rudder: RudderDesign { x: -4.75, chord: 0.60, depth: 1.65, root_endplated: true, layout: RudderLayout::SINGLE },
            displacement_kg: 8_000.0,
        }
    }

    /// **Beneteau Oceanis 38.1** (Finot-Conq, France, from 2013) — the
    /// TWIN-RUDDER preset (2026-09-11), and the reason twin rudders exist
    /// in this sim at all: the archetype of the modern volume-production
    /// cruiser, a beamy chined hull whose broad transom is what makes a
    /// single blade impractical in the first place (heel it and a
    /// centreline rudder lifts toward the surface, while the leeward one
    /// of a pair digs in).
    ///
    /// Published specs (see `docs/reference-boats.md` for sources): LOA
    /// 11.50 m, hull length 11.13 m, LWL 10.72 m — the longest waterline
    /// of the five presets on a plumb bow and a short reverse transom —
    /// beam 3.99 m, draft 2.08 m (deep keel; 1.64 m shoal, 1.26–2.40 m
    /// lifting), light displacement 6,850 kg with 1,790 kg of deep-keel
    /// ballast. Lightest of the presets by a clear margin, which is the
    /// honest modern figure and not a thumb on the scale.
    ///
    /// The curve: a narrow (≈1.3 m chord) bulbed iron fin at the full
    /// 2.08 m draft — the deepest keel here — over a canoe body shallower
    /// even than the Elan's, because a chined flat-bottomed hull carries
    /// almost no lateral plane outside its fin. Net area comes out the
    /// smallest of the five: the depth is all in a short fin, and a short
    /// fin buys area without buying yaw damping (cubic in distance from
    /// the pivot).
    pub fn beneteau_oceanis_381() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-5.35, 0.0), // aft waterline ending (LWL 10.72 m of 11.50 LOA)
                    Vec2::new(-5.30, 0.09), // very shallow run under the wide chined stern
                    Vec2::new(-1.55, 0.30),
                    Vec2::new(-1.25, 2.08), // bulbed iron fin, at the real 2.08 m draft
                    Vec2::new(0.05, 2.08),
                    Vec2::new(0.35, 0.34),
                    Vec2::new(3.3, 0.26),
                    Vec2::new(4.9, 0.06),
                    Vec2::new(5.37, 0.0), // forward waterline ending (plumb bow)
                    Vec2::new(6.0, 0.0),
                ],
            },
            // TWIN blades, 1.20 m either side of the centreline. That
            // offset is the load-bearing number and it is geometry, not a
            // handling knob: the hull's own half-beam at this station is
            // ≈1.63 m (`HULL_PTS`), so the blades sit inboard of the
            // topsides where a real pair does, and 1.20 m is an order of
            // magnitude outside the propeller race (radius ≈0.15 m — see
            // `prop_race_radius` in sim.rs), which is what removes the
            // wash steering.
            //
            // Dimensions: each blade 0.45 m mean chord × 1.25 m deep
            // hanging off a hull only ~0.3 m deep out there, so the tips
            // reach ≈1.55 m — inside the 2.08 m keel, as twin rudders
            // must be (they are the boat's grounding limit otherwise).
            // 1.13 m² of blade in total, ≈22% of this boat's small
            // lateral plane, far above the ~10% single-spade rule of
            // thumb — which is exactly right and not an error: twins are
            // deliberately generous in area because they have no prop
            // wash to help them and because, heeled, one of them is
            // doing most of the work. Roots end-plated by the flat hull
            // bottom above them (AR ≈5.6 per blade).
            rudder: RudderDesign {
                x: -4.95,
                chord: 0.45,
                depth: 1.25,
                root_endplated: true,
                layout: RudderLayout::twin(1.20),
            },
            displacement_kg: 6_850.0,
        }
    }

    /// **Alajuela 38** (William Atkin's Ingrid lineage, back to Colin
    /// Archer; Alajuela Yacht Corp., USA, 1977–1985) — the traditional
    /// long-keel preset: a heavy full-keel double-ender with a
    /// transom-hung rudder (carried as this preset's own `RudderDesign`
    /// at the hull's stern tip).
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA 11.58 m (hull, excl. bowsprit), beam 3.51 m, draft 1.83 m,
    /// displacement 26,000 lb ≈ 11,800 kg, ballast 4,536 kg lead. The
    /// curve: a cutaway forefoot deepening steadily aft, carrying nearly
    /// the full 1.83 m draft along the whole aft body to the heel at the
    /// rudder post — net area ≈13.2 m², double the O'Day's, spread far from
    /// the pivot. Shallower than the fin boats despite far more keel:
    /// long keels spread their area along the hull instead of down. The
    /// classic full-keel package deal is also ~40% more displacement at
    /// the same length — set here from the real 26,000 lb, not invented.
    pub fn alajuela_38() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-5.12, 0.0), // aft waterline ending (LWL 9.93 m of 11.58 LOA)
                    Vec2::new(-5.1, 1.75), // sternpost CLIFF: the deadwood keeps full
                    // draught to the very end and cuts off vertically — a full
                    // keel's waterline does NOT fade to zero aft, and the
                    // profile represents that as a (near-)vertical segment.
                    Vec2::new(-2.0, 1.83), // deepest point of the 1.83 m draft
                    Vec2::new(1.0, 1.4),
                    Vec2::new(3.0, 0.7), // cutaway forefoot
                    Vec2::new(4.4, 0.2),
                    Vec2::new(4.83, 0.0), // forward waterline ending
                    Vec2::new(6.0, 0.0),
                ],
            },
            // Transom-hung outboard blade hanging BEHIND the sternpost
            // cliff (the profile's −5.1 ending): leading edge on the
            // post, the blade entirely abaft the waterline ending, as an
            // outboard rudder really hangs. ≈0.85 m² but only ≈5% of
            // this boat's big lateral plane — a full keel does the
            // tracking itself and needs proportionally little rudder.
            // NOT root-end-plated: the blade breaks the surface with air
            // above the root, so there's no plate to mirror it —
            // effective AR ≈ 2.8 (vs ≈5 for the spades), the honest
            // reason a barn-door rudder feels mushier per square metre
            // than a spade.
            rudder: RudderDesign { x: -5.38, chord: 0.55, depth: 1.55, root_endplated: false, layout: RudderLayout::SINGLE },
            displacement_kg: 11_800.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_preset_paints_deeper_than_its_boat_draws() {
        // The profile's value IS the local depth of lateral plane, so a
        // preset claiming more than its boat's published draft anywhere
        // along the hull would be a physically impossible keel — the unit
        // consistency the boat naming is supposed to buy.
        for (design, draft, name) in [
            (BoatDesign::hallberg_rassy_38(), 1.75, "Hallberg-Rassy 38"),
            (BoatDesign::oday_39(), 1.93, "O'Day 39"),
            (BoatDesign::elan_impression_394(), 1.80, "Elan Impression 394"),
            (BoatDesign::beneteau_oceanis_381(), 2.08, "Beneteau Oceanis 38.1"),
            (BoatDesign::alajuela_38(), 1.83, "Alajuela 38"),
        ] {
            let deepest = design.keel.points.iter().map(|p| p.y).fold(0.0f32, f32::max);
            assert!(
                deepest <= draft + 1e-4,
                "{name} paints {deepest} m of local draught but the boat only draws {draft} m"
            );
        }
    }

    #[test]
    fn presets_rank_like_the_real_boats() {
        let hr = BoatDesign::hallberg_rassy_38();
        let oday = BoatDesign::oday_39();
        let elan = BoatDesign::elan_impression_394();
        let oceanis = BoatDesign::beneteau_oceanis_381();
        let alajuela = BoatDesign::alajuela_38();
        // Displacement: modern cruiser < fin cruiser/racer < fin+skeg
        // cruiser < full-keel heavy cruiser — straight from the published
        // numbers.
        assert!(oceanis.displacement_kg < elan.displacement_kg);
        assert!(elan.displacement_kg < oday.displacement_kg);
        assert!(oday.displacement_kg < hr.displacement_kg);
        assert!(hr.displacement_kg < alajuela.displacement_kg);
        // Lateral area ranks the same way: modern flat underbody < spade-
        // rudder fin boat < fin + skeg < full keel (the full keel carries
        // area along the whole hull, the fin concentrates it; the Elan
        // strips it off the hull ends entirely).
        let (a_elan, a_oday, a_hr, a_alajuela) = (
            elan.keel.derive().area,
            oday.keel.derive().area,
            hr.keel.derive().area,
            alajuela.keel.derive().area,
        );
        assert!(
            a_elan < a_oday && a_oday < a_hr && a_hr < a_alajuela,
            "areas should rank Elan < O'Day < HR < Alajuela, got {a_elan} / {a_oday} / {a_hr} / {a_alajuela}"
        );
        // The Oceanis belongs in the modern shallow-fin band WITH the
        // Elan — same design generation, same flat chined underbody, and
        // their areas land within a few percent of each other (5.3 vs
        // 5.4 m²). Deliberately NOT ranked against the Elan: a 2% gap
        // between two boats drawn from profile readings is not a claim
        // either set of published specs can support, and asserting it
        // would make an honest curve edit fail for no reason.
        let a_oceanis = oceanis.keel.derive().area;
        assert!(
            a_oceanis < a_oday,
            "the Oceanis's flat underbody should carry less lateral plane than the O'Day's, \
             got {a_oceanis} vs {a_oday}"
        );
        assert!(
            (a_oceanis - a_elan).abs() < 0.15 * a_elan,
            "Oceanis {a_oceanis} and Elan {a_elan} should sit in the same band"
        );
    }

    // Compile-time proof of the const-context claim in `twin`'s docs: a
    // GOOD literal is accepted in a const item. (The bad-literal half
    // cannot be written here — it is a compile error by design.)
    const _GOOD: RudderLayout = RudderLayout::twin(1.2);

    #[test]
    fn a_twin_layout_refuses_an_offset_that_is_not_a_real_separation() {
        // `BoatDesign` is public, so these reach `tick`'s force path if
        // the constructor lets them: NaN/inf poison the rigid body, and
        // zero is a "twin" rudder that is really one blade on the
        // centreline — standing in the prop race, the exact behaviour
        // this type exists to rule out.
        for bad in [0.0, -1.2, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(
                std::panic::catch_unwind(|| RudderLayout::twin(bad)).is_err(),
                "twin({bad}) should be refused"
            );
        }
        // And the real thing still builds.
        let ok = RudderLayout::twin(1.2);
        assert_eq!(ok.blades(), 2);
        assert_eq!(ok.offsets(), [-1.2, 1.2]);
    }

    #[test]
    fn only_the_oceanis_has_twin_rudders_and_they_sit_inside_her_topsides() {
        for (design, name) in [
            (BoatDesign::hallberg_rassy_38(), "Hallberg-Rassy 38"),
            (BoatDesign::oday_39(), "O'Day 39"),
            (BoatDesign::elan_impression_394(), "Elan Impression 394"),
            (BoatDesign::alajuela_38(), "Alajuela 38"),
        ] {
            assert_eq!(
                design.rudder.layout,
                RudderLayout::SINGLE,
                "{name} is a single-rudder boat"
            );
            assert_eq!(design.rudder.total_area(), design.rudder.area(), "{name}: one blade");
        }
        let oceanis = BoatDesign::beneteau_oceanis_381();
        assert_eq!(oceanis.rudder.layout.blades(), 2);
        assert!((oceanis.rudder.total_area() - 2.0 * oceanis.rudder.area()).abs() < 1e-6);
        // A real pair hangs under the hull, not out past the topsides —
        // and a blade outboard of the hull would be a fender-catching
        // liability no builder ships. The hull's own half-beam at the
        // blades' station (interpolated on `HULL_PTS`' aft run, −3.6 m at
        // 1.9 to −5.6 m at 1.5) is the check.
        let x = oceanis.rudder.x;
        let half_beam = 1.5 + (1.9 - 1.5) * ((x - (-5.6)) / (-3.6 - (-5.6)));
        assert!(
            oceanis.rudder.layout.offset() < half_beam,
            "blades {:.2} m out vs a half-beam of {half_beam:.2} m at x {x}",
            oceanis.rudder.layout.offset()
        );
        // And the tips must stay inside the keel: twin rudders that draw
        // more than the keel make the RUDDERS the grounding limit, which
        // is exactly what no one builds.
        let hull_depth = oceanis.keel.sample(x);
        let rudder_draft = hull_depth + oceanis.rudder.depth;
        let keel_draft = oceanis.keel.points.iter().map(|p| p.y).fold(0.0f32, f32::max);
        assert!(
            rudder_draft < keel_draft,
            "rudders draw {rudder_draft:.2} m, keel {keel_draft:.2} m — the keel must be deeper"
        );
    }

    #[test]
    fn the_fin_keeler_spins_more_freely_than_the_full_keeler() {
        let fin = BoatDesign::oday_39().keel.derive();
        let long = BoatDesign::alajuela_38().keel.derive();
        // The whole point of a fin keel: concentrating the BULK of its area
        // near the pivot trades away yaw damping much faster than it trades
        // away area, because the yaw term is cubic in distance from the
        // pivot. Compare cubic moment PER UNIT AREA — the presets don't
        // have similar total areas, so an absolute comparison could pass
        // for the wrong reason (the O'Day just has less area overall, not
        // less per unit of it).
        let fin_per_area = fin.cubic_moment / fin.area;
        let long_per_area = long.cubic_moment / long.area;
        assert!(
            fin_per_area < long_per_area * 0.5,
            "O'Day cubic_moment/area {} should be well below the Alajuela's {}",
            fin_per_area,
            long_per_area
        );
        assert!(fin.clr_offset.abs() < long.clr_offset.abs());
    }
}
