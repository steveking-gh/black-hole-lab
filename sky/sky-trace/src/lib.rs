//! sky-trace, the parts that are not the command line.
//!
//! `bhl` reads a Black Hole Lab save: the hole, the clock and the observers. `worldline` carries a
//! saved observer forward from the saved moment at equal steps of the observer's own proper time,
//! as the app would move it. Both are pure - no printing, no files but the one being read - and
//! the program in `main.rs` is the thin layer that turns their answers into text.
//!
//! `film` turns each event of the walk into a frame of the observer's sky - the triad, the traced
//! planes from `kerr-sky`, the manifest entry - and states the manifest of the whole film. It is
//! pure too: the program opens the save and writes the bundle, and the tests run all of it on
//! grids of a few pixels.

pub mod bhl;
pub mod film;
pub mod worldline;
