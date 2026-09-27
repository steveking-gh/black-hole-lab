//! sky-trace, the parts that are not the command line.
//!
//! `bhl` reads a Black Hole Lab save: the hole, the clock and the observers. `worldline` carries a
//! saved observer forward from the saved moment at equal steps of the observer's own proper time,
//! as the app would move it. Both are pure - no printing, no files but the one being read - and
//! the program in `main.rs` is the thin layer that turns their answers into text.
//!
//! The light tracing that turns each event into a frame of the observer's sky is not here yet: it
//! is wired in once the light-tracing library, `kerr-sky`, has been reviewed.

pub mod bhl;
pub mod worldline;
