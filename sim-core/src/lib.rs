// The deterministic half of Harbour Sim, kept macroquad-free (same
// architecture as Pegasus): NOTHING in this crate may depend on the frame
// clock, rendering, or any other source of nondeterminism — the sim must be
// a pure function of its input stream so recordings/verification stay
// possible later. See the determinism rules in CLAUDE.md.

pub mod boat;
pub mod keel;
pub mod line;
pub mod sim;
