//! sky-look: one command from a Black Hole Lab save to a 360-degree view of what one observer sees
//! at the saved moment. It is the program behind the app's Look Around button:
//!
//! ```text
//! sky-look <file.bhl> --observer bob|alice --open
//! ```
//!
//! It runs `sky-trace` on the one frame of the saved moment and `sky-render` on what that traced,
//! as a still held for a minute and as a 360-degree photograph, puts the two files in the views
//! directory, and with `--open` hands the video to the system's player. The contract with the app,
//! which `src/look_around.rs` at the repository root is built against:
//!
//! - progress on standard output, one complete short sentence a line, flushed after each;
//! - exit 0 on success, the last line of standard output the video's full path and nothing else;
//! - exit 2 when the moment is refused (the observer inside the inner horizon, dragged, at the
//!   ring, frozen onto the inner horizon, not in the save; a hole spinning the other way), with the
//!   tracer's own sentence on standard error;
//! - exit 1 on any other failure, with one sentence on standard error saying what to do;
//! - nothing on standard error on a run that succeeds.
//!
//! `args` reads the command line; `find` finds the pieces; `run` runs the two programs and turns
//! what they print into sentences; `names` names and places the finished files; `scratch` keeps
//! every intermediate in one directory of its own and clears what killed runs left; `save` reads
//! the three facts this program takes from the save; `open` hands the video to the player.

mod args;
mod find;
mod names;
mod open;
mod run;
mod save;
mod scratch;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_run;

fn main() {
    // Before anything is started, so that nothing started from here can hold the app's pipes open
    // after this program has exited (`open` says why that matters).
    open::keep_own_pipes_to_ourselves();
    let args: Vec<std::ffi::OsString> = std::env::args_os().skip(1).collect();
    let env = run::Environment::of_this_process();
    let code = run::cli(&args, &env, &mut std::io::stdout(), &mut std::io::stderr());
    std::process::exit(code);
}
