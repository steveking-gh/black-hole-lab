//! sky-look: one command from a Black Hole Lab save to a 360-degree view of what one observer sees
//! at the saved moment. It is the program behind the app's Look Around button:
//!
//! ```text
//! sky-look <file.bhl> --observer bob|alice --status <file> --move-save --open [--shell]
//! ```
//!
//! It runs `sky-trace` on the one frame of the saved moment and `sky-render` on what that traced,
//! as a 360-degree photograph with the observer's read-outs painted on a panel inside the hole's
//! dark region (or below the opening view when the dark region is too small to hold them) - the
//! observer's watch, the coordinate time (the app's distant clock), and the proper distance from
//! the outer horizon (or between the horizons the proper time since crossing it), in seconds and kilometres unless
//! `--units geometric` asks for M, and the speed and heading of travel past each local reference observer there is where the
//! observer is - and small green signs on the sky marking those directions of travel. Each view
//! gets a folder of its own in the views directory, holding the photograph, the save, the traced
//! bundle, `commands.txt` and `log.txt`, and nothing in it is ever deleted (`names`). With `--open`
//! it hands the photograph to a viewer: VLC when it is installed, which pans a 360-degree
//! photograph, before the system's default program for `.jpg` files, which may show it flat.
//!
//! The contract with the app, which `src/look_around.rs` at the repository root is built against:
//!
//! - the app writes a temporary save, creates an empty status file beside it, and starts this
//!   program on the save with `--status` naming that file and `--move-save`, at below-normal
//!   priority; with no console window, or with `--shell` in a console window of its own;
//! - the first thing this program writes to disk, once it has found every piece it needs, is the
//!   view's folder, and then the save, moved into it (renamed, or copied and deleted across
//!   volumes); a run stopped before that leaves the temporary save where it was, for the app to
//!   delete;
//! - every progress sentence goes on a line of its own to the status file and to `log.txt`, and
//!   to standard output unless `--shell` is given, flushed after each; a sentence begins with a capital letter or a digit and ends
//!   with a full stop, and while the tracer and the renderer work it carries their percentage, as
//!   in `Tracing the light that reaches Alice: 35%.`;
//! - the last line of the status file, and only the last, is the verdict, one of
//!
//!   ```text
//!   done <the photograph's full path>
//!   refused <one sentence saying why the moment is refused>
//!   failed <one sentence saying what went wrong and what to do>
//!   ```
//!
//!   a lowercase word, one space and the rest of the line: which no sentence of progress can be
//!   taken for. The status file is UTF-8, every line ended by a line feed and written in one piece;
//!   a reader that finds a line without its line feed has read it too soon, and waits for it. A
//!   command line this program refuses still gets `failed <sentence>` in the file it names. A
//!   status file with no verdict when this program has exited means it was stopped or crashed;
//! - the console, which is not the app's to read, is split two ways (`run::Say`). With no
//!   `--shell` - a person or a script at a terminal - standard output carries the sentences, and
//!   just before each program is started the command that starts it (a PowerShell line beginning
//!   with `&`), and on success, last, the photograph's full path and nothing else; standard error
//!   carries a failure's one sentence. With `--shell` - the rendering terminal - the console
//!   carries only each command, as above, and then what that program prints, standard output to
//!   standard output and standard error to standard error, byte for byte as it arrives: nothing
//!   of this program's own, since its sentences are on the app's card already. The tracer's and
//!   the renderer's lines are read for the percentages and the rest the same way in both;
//! - exit 0 on success; exit 2 when the moment is refused (the observer inside the inner horizon,
//!   dragged, at the ring, frozen onto the inner horizon, not in the save; a hole spinning the
//!   other way), with the tracer's own sentence on standard error; exit 1 on any other failure,
//!   with one sentence on standard error saying what to do (without `--shell`); nothing of this
//!   program's on standard error on a run that succeeds. The codes are for a person or a script at a terminal: the app reads the
//!   verdict, and takes an exit only as a sign that no verdict is coming;
//! - with `--shell`, once the verdict is written - whether the view was made, refused or failed -
//!   a command prompt starts in the view's folder, in this program's console window, and this
//!   program exits without waiting for it (`shell`).
//!
//! Two more command lines make no view, and are what the app uses to say what a view needs before
//! one is asked for, and to supply the piece it can:
//!
//! ```text
//! sky-look --check
//! sky-look --fetch-map --status <file>
//! ```
//!
//! - `--check` looks for every piece a view needs and prints three lines on standard output, for
//!   `tools`, `ffmpeg` and `map` in that order: the word, a space, and `ok`, or `missing
//!   <sentence>`, or for a default star map that `--fetch-map` would supply, `fetchable
//!   <sentence>`; and when ffmpeg is missing and the command that installs it on this system is
//!   known, a fourth line, `ffmpeg-install <command>`. It exits 0 when all three are `ok`, else 1;
//! - `--fetch-map` downloads the default star map with curl to where this program looks for it,
//!   and checks its size and SHA-256 before putting it there (`fetch`). Its status file is a
//!   view's: sentences of progress carrying the percentage downloaded, and last `done <the map's
//!   full path>` or `failed <sentence>`.
//!
//! `args` reads the command line; `find` finds the pieces; `run` runs the two programs and turns
//! what they print into sentences; `names` names the view and makes its folder; `save` reads the
//! three facts this program takes from the save; `open` chooses the viewer and hands the
//! photograph to it; `shell` takes the console and leaves the prompt in it.

mod args;
mod fetch;
mod find;
mod names;
mod open;
mod run;
mod save;
mod sha256;
mod shell;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_run;

fn main() {
    let args: Vec<std::ffi::OsString> = std::env::args_os().skip(1).collect();
    // Before anything is printed, so that every line reaches the console window the app opened
    // for this program rather than wherever the handles it was given point (`shell` says why).
    if args::wants_shell(&args) {
        shell::use_own_console();
    }
    // Before anything is started, so that nothing started from here holds this program's own
    // standard streams (`open` says why that matters).
    open::keep_own_pipes_to_ourselves();
    let env = run::Environment::of_this_process();
    let code = run::cli(&args, &env, &mut std::io::stdout(), &mut std::io::stderr());
    std::process::exit(code);
}
