//! Look Around: a 360-degree view of the sky from where one observer stands, made by a program
//! outside this one.
//!
//! The view is traced and rendered by `sky-look`, a program of the sky tools under `sky/`, which
//! is a cargo workspace of its own. Nothing of that pipeline is linked in here. What this module
//! does is the whole of the app's side of the arrangement: find the program, hand it a save of the
//! present moment, read how far it has got without ever waiting on it, stop it when the user
//! cancels, and clean up after it. The contract it is written against, which `sky/sky-look/src/
//! main.rs` states from the other side, is:
//!
//! ```text
//! sky-look <file.bhl> --observer bob|alice --status <file> --move-save --open [--shell]
//! ```
//!
//! * the app writes the save and an empty status file beside it in the temporary directory, and
//!   starts `sky-look` at below-normal priority: with no window, or, with `--shell` when Show
//!   Rendering Terminal is ticked, in a console window of its own;
//! * `sky-look` makes the view's folder in the views directory and moves the save into it before it
//!   does anything else it can be stopped part-way through; everything else of the view - the
//!   photograph, the traced bundle, `commands.txt`, `log.txt` - goes into that folder too, and
//!   nothing in it is ever deleted, by either program, whether or not the view was made;
//! * every progress sentence is appended to the status file on a line of its own, and while the
//!   tracer and the renderer work a sentence carries their percentage (`Tracing the light that
//!   reaches Alice: 35%.`); a sentence begins with a capital letter or a digit;
//! * the last line of the status file is the verdict: `done <the photograph's full path>` once the
//!   photograph is made and handed to a viewer because of `--open` (VLC when it is installed, else
//!   the system's default program for `.jpg` files), `refused <sentence>` when the moment is one no
//!   view can be made from (an observer inside r₋, on the ring, being dragged), or
//!   `failed <sentence>` for anything else;
//! * the file is UTF-8, every line ended by a line feed; a line found without its line feed has
//!   been read too soon, and is left until the rest of it arrives;
//! * with `--shell`, once the verdict is written, `sky-look` leaves a command prompt running in the
//!   view's folder in its console window and exits. The window stays until the user closes it.
//!
//! `sky-look` also exits 0, 1 or 2 for success, failure and refusal, and prints its progress on
//! standard output, for a person or a script at a terminal. The app reads neither: the exit is
//! taken only as a sign that no verdict is coming, which means `sky-look` was stopped or crashed.
//! A file rather than pipes because a pipe can only go one place: it had to be the app's, so the
//! rendering terminal could never have shown the progress, and a pipe outlives the child whenever
//! something the child started inherits it, which a verdict read from a file cannot be held up by.
//!
//! A separate process rather than a library call, for the rule the sky work is held to: it must
//! not touch what the app costs or how the app builds. A process costs the app nothing until the
//! button is pressed, and while a view is being made it costs one read of a small file a frame.
//! The tracer takes up to a minute on every core the machine has, and none of that is on this
//! thread; the below-normal priority, which everything `sky-look` starts inherits, keeps the app
//! and the rest of the desktop responsive while it does.
//!
//! # What the app does not know
//!
//! Whether a view *can* be made from a given moment is the tracer's judgement: it follows the
//! observer on in the full Kerr spacetime, which this app does not integrate. The app refuses only
//! what it can see for itself (`gui::controls::look_around_blocked`) and otherwise shows the
//! tracer's own sentence.
//!
//! # Cancelling, and closing the app while a view is being made
//!
//! Either way `sky-look` is stopped, and so is anything it started, and the app's two temporary
//! files are deleted: see `Job`'s `Drop`. The view's folder, if `sky-look` got as far as making
//! it, is left as it stood, with the save in it. Letting the child run on after the app closes was
//! the other choice, and it was turned down: a view finished after the app has gone opens a viewer
//! out of nowhere, up to a minute after the user closed the program they asked it from, with no
//! status line left to say what it is.

use std::ffi::{OsStr, OsString};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::physics::observer::Who;

/// The environment variable that names the program outright, ahead of every other place it is
/// looked for: the way to point a build of the app at a build of the sky tools that is not where
/// the search expects it.
pub const SKY_LOOK_ENV: &str = "BLACK_HOLE_LAB_SKY_LOOK";

/// The program's file name on this platform.
pub const SKY_LOOK_FILE: &str = if cfg!(windows) { "sky-look.exe" } else { "sky-look" };

/// Whether this platform can show the rendering terminal: a console window of its own for a
/// program is a Windows notion, and elsewhere there is no terminal for the app to open.
pub const TERMINAL_AVAILABLE: bool = cfg!(windows);

/// The extension of a press's status file, beside its save.
pub const STATUS_EXTENSION: &str = "status";

/// Where a release build of the sky tools puts its programs, relative to the repository root: the
/// sky workspace's own target directory, because the sky tools are built from `sky/` and not from
/// the root.
const SKY_RELEASE_DIR: &str = "sky/target/release";

/// Windows' `CREATE_NO_WINDOW` process creation flag. A console program started from a windowed
/// program gets a console window of its own unless it is told not to, and a black window flashing
/// up over the app on every press is not something the user asked for.
#[cfg(windows)]
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// Windows' `CREATE_NEW_CONSOLE`: the program gets a console window of its own, whatever console
/// the app has. This is the rendering terminal; on a Windows 11 whose default terminal is Windows
/// Terminal the window opens there.
#[cfg(windows)]
const CREATE_NEW_CONSOLE: u32 = 0x0000_0010;

/// Windows' `BELOW_NORMAL_PRIORITY_CLASS`. The tracer runs on every core, and at normal priority
/// it competes with the app's own frames and the rest of the desktop for all of them; one class
/// below normal, those win whenever they want a core and the tracer has the rest. A child started
/// without a class of its own inherits this one, so the tracer, the renderer and its ffmpeg all run
/// below normal too.
#[cfg(windows)]
const BELOW_NORMAL_PRIORITY_CLASS: u32 = 0x0000_4000;

/// Where `sky-look` is, or a sentence saying it could not be found and how to get it.
///
/// A pure function of its two inputs, so that the search can be tested against a directory layout
/// made for the purpose rather than against wherever the test binary happens to have been built.
/// `locate` reads the two out of the process. In order:
///
/// 1. `from_env`, the value of `SKY_LOOK_ENV`, when it is set and not empty. A variable naming a
///    file that is not there is reported as such rather than passed over: whoever set it meant that
///    program, and quietly running a different one would hide the mistake.
/// 2. `SKY_LOOK_FILE` in `exe_dir`, the directory the app's own executable is in, which is where a
///    packaged app would ship the program beside itself.
/// 3. `sky/target/release/SKY_LOOK_FILE` under `exe_dir` or the nearest directory above it that has
///    one, which is where a developer's release build of the sky tools lands: the app runs from
///    `target/release` or `target/probe/release` of the same repository.
pub fn find_sky_look(from_env: Option<&OsStr>, exe_dir: Option<&Path>) -> Result<PathBuf, String> {
    if let Some(named) = from_env.filter(|value| !value.is_empty()) {
        let named = PathBuf::from(named);
        return if named.is_file() {
            Ok(named)
        } else {
            Err(format!(
                "{SKY_LOOK_ENV} names {}, and there is no program there.",
                named.display()
            ))
        };
    }
    if let Some(dir) = exe_dir {
        let beside = dir.join(SKY_LOOK_FILE);
        if beside.is_file() {
            return Ok(beside);
        }
        if let Some(built) = dir
            .ancestors()
            .map(|above| above.join(SKY_RELEASE_DIR).join(SKY_LOOK_FILE))
            .find(|candidate| candidate.is_file())
        {
            return Ok(built);
        }
    }
    Err(format!(
        "{SKY_LOOK_FILE} was not found. Build the sky tools with cargo build --release in the sky \
         directory, or set {SKY_LOOK_ENV} to the program's full path."
    ))
}

/// `find_sky_look` for this process: the environment variable as it stands, and the directory of
/// the executable that is running. Called on a press and never on a frame.
pub fn locate() -> Result<PathBuf, String> {
    let exe = std::env::current_exe().ok();
    find_sky_look(std::env::var_os(SKY_LOOK_ENV).as_deref(), exe.as_deref().and_then(Path::parent))
}

/// A path in `dir` for one press's save, which no other press and no other running copy of the app
/// can be using: the process id tells two instances apart, and a counter tells two presses of one
/// instance apart. A file left under the same name by an instance that crashed and whose id has
/// since been reused is simply written over.
pub fn scratch_save_path(dir: &Path) -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    dir.join(format!(
        "black-hole-lab-look-{}-{}.{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed),
        crate::save::EXTENSION
    ))
}

/// The status file of the press whose save is `save`: the same name with `STATUS_EXTENSION`, so
/// that it is as unique as the save's own, and is named before `sky-look` has chosen the view's
/// folder.
pub fn status_path_for(save: &Path) -> PathBuf {
    save.with_extension(STATUS_EXTENSION)
}

/// The command line the app gives `sky-look`, after the program's name.
pub fn arguments(save: &Path, status: &Path, who: Who, terminal: bool) -> Vec<OsString> {
    let mut args: Vec<OsString> = vec![
        save.into(),
        "--observer".into(),
        who.name().to_lowercase().into(),
        "--status".into(),
        status.into(),
        "--move-save".into(),
        "--open".into(),
    ];
    if terminal {
        args.push("--shell".into());
    }
    args
}

/// The Windows process creation flags for `sky-look`: a console window of its own for the
/// rendering terminal, else none, and below-normal priority either way.
#[cfg(windows)]
pub fn creation_flags(terminal: bool) -> u32 {
    let window = if terminal { CREATE_NEW_CONSOLE } else { CREATE_NO_WINDOW };
    window | BELOW_NORMAL_PRIORITY_CLASS
}

/// Where a view has got to, as `Job::poll` reports it.
pub enum Progress {
    /// Still being made; the text is the line to show while it is.
    Running(String),
    /// Finished one way or the other: the line to leave on the card, and whether it is a failure.
    Finished { text: String, failed: bool },
}

/// The last line of a status file, parsed.
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    /// `done <path>`: the photograph's full path.
    Done(String),
    /// `refused <sentence>`.
    Refused(String),
    /// `failed <sentence>`.
    Failed(String),
}

impl Verdict {
    /// The verdict a line states, or None for a line of progress: see the module's contract.
    pub fn of(line: &str) -> Option<Verdict> {
        if let Some(path) = line.strip_prefix("done ") {
            Some(Verdict::Done(path.to_string()))
        } else if let Some(why) = line.strip_prefix("refused ") {
            Some(Verdict::Refused(why.to_string()))
        } else {
            line.strip_prefix("failed ").map(|why| Verdict::Failed(why.to_string()))
        }
    }

    /// What the card says of a view that ended so, for the observer named `name`.
    fn progress(&self, name: &str) -> Progress {
        let (text, failed) = match self {
            Verdict::Done(photo) => (format!("Made {name}'s view and opened it: {photo}"), false),
            Verdict::Refused(why) => (format!("No view from {name}'s place: {why}"), true),
            Verdict::Failed(why) => (format!("Could not make {name}'s view: {why}"), true),
        };
        Progress::Finished { text, failed }
    }
}

/// The reading end of a status file: each call to `lines` hands over the complete lines written
/// since the last, and never blocks.
///
/// The file is opened on the first read that finds it (the app creates it before `sky-look`
/// starts, so that is the first read) and kept open, and read from where the last read stopped. A
/// line without its line feed yet is kept back until the rest of it arrives, so a sentence is
/// never shown cut in half; `rest` gives it up once the writer has gone and nothing more is coming.
pub struct StatusTail {
    path: PathBuf,
    file: Option<std::fs::File>,
    /// Bytes read after the last line feed.
    partial: Vec<u8>,
}

impl StatusTail {
    pub fn new(path: PathBuf) -> Self {
        Self { path, file: None, partial: Vec::new() }
    }

    /// The complete lines written since the last call, their line endings (`\r` included) trimmed,
    /// and blank lines left out. Bytes that are not UTF-8 are decoded leniently: a line that is not
    /// is still a line of progress.
    pub fn lines(&mut self) -> Vec<String> {
        if self.file.is_none() {
            self.file = std::fs::File::open(&self.path).ok();
        }
        let Some(file) = self.file.as_mut() else { return Vec::new() };
        // The file's own cursor is where the last read stopped: nothing else reads this handle.
        // A read that fails part-way keeps what it did read, which the cursor has passed, and the
        // rest is read on the next frame.
        let mut fresh = Vec::new();
        let _ = file.read_to_end(&mut fresh);
        self.partial.extend_from_slice(&fresh);
        let Some(end) = self.partial.iter().rposition(|&b| b == b'\n') else {
            return Vec::new();
        };
        let complete: Vec<u8> = self.partial.drain(..=end).collect();
        split_lines(&complete)
    }

    /// Whatever is left without a line feed, as a line, once nothing more will be written.
    pub fn rest(&mut self) -> Option<String> {
        let rest = split_lines(&std::mem::take(&mut self.partial));
        rest.into_iter().next_back()
    }
}

/// `bytes` as lines: split at line feeds, decoded leniently, trimmed, blank lines left out.
fn split_lines(bytes: &[u8]) -> Vec<String> {
    bytes
        .split(|&b| b == b'\n')
        .map(|line| String::from_utf8_lossy(line).trim().to_string())
        .filter(|line| !line.is_empty())
        .collect()
}

/// A view being made: the running `sky-look`, what its status file has said so far, and the two
/// temporary files the app made for it.
///
/// Its `Drop` is the cleanup, whichever way the job ends - finished and polled, cancelled, or
/// dropped with the app mid-run - so that no path out of a view leaves the child running or the
/// app's files on disk.
pub struct Job {
    pub who: Who,
    child: Child,
    tail: StatusTail,
    /// The latest line of progress, which is the status while the view is being made.
    last_line: Option<String>,
    /// How the run ended, once the status file has said.
    verdict: Option<Verdict>,
    /// The temporary save the child was handed. `sky-look` moves it into the view's folder as its
    /// first act; the path is deleted when the job is dropped all the same, which removes nothing
    /// once it has been moved and removes the save when `sky-look` stopped before moving it.
    save: PathBuf,
    /// The status file, the app's own, deleted when the job is dropped.
    status: PathBuf,
    /// Whether the app has seen the child exit.
    exited: bool,
}

impl Job {
    /// Start `program` on `save` for `who`, with a console window of its own when `terminal` asks
    /// for the rendering terminal and the platform has one, and return at once.
    ///
    /// The job owns the save from here on, so a child that will not start deletes it on the way
    /// out, and so does everything after that. The status file is made here, empty, before the
    /// child starts, so that it is there to be read from the first frame and is the app's to
    /// delete.
    pub fn start(program: &Path, save: PathBuf, who: Who, terminal: bool) -> std::io::Result<Job> {
        let status = status_path_for(&save);
        let discard = |cause: std::io::Error| {
            let _ = std::fs::remove_file(&save);
            let _ = std::fs::remove_file(&status);
            cause
        };
        std::fs::File::create(&status).map_err(discard)?;
        let terminal = terminal && TERMINAL_AVAILABLE;
        let mut command = Command::new(program);
        // Standard output and error go nowhere the app reads: the status file is the app's channel.
        // In the rendering terminal `sky-look` opens its own console window's streams by name,
        // since whatever handles the app passed on would lead back to the app's console, if any,
        // and not to the new window.
        command
            .args(arguments(&save, &status, who, terminal))
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(creation_flags(terminal));
        }
        let child = command.spawn().map_err(discard)?;
        Ok(Job {
            who,
            child,
            tail: StatusTail::new(status.clone()),
            last_line: None,
            verdict: None,
            save,
            status,
            exited: false,
        })
    }

    /// Take in whatever the status file has said since the last call, and say where the view has
    /// got to. Never blocks: a frame calls this once.
    ///
    /// The exit is asked for before the file is read, so that a child which wrote its verdict and
    /// exited between the two is found with its verdict, and never taken for one that stopped
    /// without saying why.
    pub fn poll(&mut self) -> Progress {
        let exit = match self.child.try_wait() {
            Ok(exit) => exit,
            Err(cause) => {
                return Progress::Finished {
                    text: format!(
                        "Lost track of sky-look while it made {}'s view: {cause}",
                        self.who.name()
                    ),
                    failed: true,
                };
            }
        };
        for line in self.tail.lines() {
            self.take(line);
        }
        if exit.is_some() {
            self.exited = true;
            if let Some(line) = self.tail.rest() {
                self.take(line);
            }
        }
        if let Some(verdict) = &self.verdict {
            return verdict.progress(self.who.name());
        }
        match exit {
            None => self.running(),
            Some(status) => stopped_without_verdict(self.who, status),
        }
    }

    /// One line of the status file: the verdict, or the latest progress.
    fn take(&mut self, line: String) {
        if self.verdict.is_some() {
            return;
        }
        match Verdict::of(&line) {
            Some(verdict) => self.verdict = Some(verdict),
            None => self.last_line = Some(line),
        }
    }

    /// The status of a view still being made: the child's latest line, or before it has written
    /// one, that it has been started.
    fn running(&self) -> Progress {
        Progress::Running(
            self.last_line
                .clone()
                .unwrap_or_else(|| format!("Starting sky-look for {}'s view.", self.who.name())),
        )
    }
}

impl Drop for Job {
    fn drop(&mut self) {
        // A child that has not exited and has not given its verdict is stopped, with whatever it
        // started: see the module's own note on cancelling. One that has given its verdict is left
        // to exit by itself, since all it may still be doing is starting the command prompt that
        // the user asked to be left running.
        if !self.exited
            && self.verdict.is_none()
            && !matches!(self.child.try_wait(), Ok(Some(_)))
        {
            stop(&mut self.child);
        }
        let _ = std::fs::remove_file(&self.save);
        let _ = std::fs::remove_file(&self.status);
    }
}

/// What the card says of a `sky-look` that exited without a verdict: stopped from outside, or
/// crashed.
fn stopped_without_verdict(who: Who, status: ExitStatus) -> Progress {
    let name = who.name();
    let text = match status.code() {
        Some(code) => format!(
            "Could not make {name}'s view: sky-look stopped with exit code {code} without saying \
             why."
        ),
        None => format!("sky-look was stopped before it finished {name}'s view."),
    };
    Progress::Finished { text, failed: true }
}

/// Stop `child` and everything it started, and reap it.
///
/// On Windows `Child::kill` ends the one process it names, and `sky-look` may be running stages
/// of the pipeline as processes of their own, which would carry on without it; `taskkill /T` takes
/// the whole tree. The child has not been reaped, so its id cannot have been handed to another
/// process. Anywhere `taskkill` will not run, and on other platforms, the child alone is killed.
fn stop(child: &mut Child) {
    #[cfg(windows)]
    let stopped = {
        use std::os::windows::process::CommandExt;
        Command::new("taskkill")
            .args(["/T", "/F", "/PID", &child.id().to_string()])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .creation_flags(CREATE_NO_WINDOW)
            .status()
            .is_ok_and(|status| status.success())
    };
    #[cfg(not(windows))]
    let stopped = false;
    if !stopped {
        let _ = child.kill();
    }
    let _ = child.wait();
}

#[cfg(test)]
mod tests;
