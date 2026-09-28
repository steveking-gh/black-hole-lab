//! Look Around: a 360-degree view of the sky from where one observer stands, made by a program
//! outside this one.
//!
//! The view is traced and rendered by `sky-look`, a program of the sky tools under `sky/`, which
//! is a cargo workspace of its own. Nothing of that pipeline is linked in here. What this module
//! does is the whole of the app's side of the arrangement: find the program, hand it a save of the
//! present moment, read what it prints without ever waiting on it, and clean up after it. The
//! contract it is written against is:
//!
//! ```text
//! sky-look <file.bhl> --observer bob|alice --open
//! ```
//!
//! * progress on standard output, one complete short sentence a line, flushed after each;
//! * exit 0 on success, with the full path of the video as the last line of standard output, and
//!   the video already handed to the system's player because of `--open`;
//! * exit 2 when it refuses the moment - an observer inside r₋, on the ring, being dragged - with
//!   one sentence on standard error saying why;
//! * exit 1 on any other failure, with one sentence on standard error.
//!
//! A separate process rather than a library call, for the rule the sky work is held to: it must
//! not touch what the app costs or how the app builds. A process costs the app nothing until the
//! button is pressed, and while a view is being made it costs one non-blocking poll a frame. The
//! tracer can take a minute on every core the machine has, and none of that is on this thread.
//!
//! # What the app does not know
//!
//! Whether a view *can* be made from a given moment is the tracer's judgement: it follows the
//! observer on in the full Kerr spacetime, which this app does not integrate. The app refuses only
//! what it can see for itself (`gui::controls::look_around_blocked`) and otherwise shows the
//! tracer's own sentence.
//!
//! # When the app closes while a view is being made
//!
//! `sky-look` is stopped, and so is anything it started, and the temporary save is deleted: see
//! `Job`'s `Drop`. Letting it run on was the other choice, and it was turned down. A view finished
//! after the app has gone opens a video player out of nowhere, minutes after the user closed the
//! program they asked it from, with no status line left to say what it is; and the temporary save
//! cannot be deleted while a program that has not yet read it is still running, so letting the
//! child outlive the app would mean leaving a multi-megabyte file in the temporary directory every
//! time.

use std::ffi::OsStr;
use std::io::BufRead;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::{Duration, Instant};

use crate::physics::observer::Who;

/// The environment variable that names the program outright, ahead of every other place it is
/// looked for: the way to point a build of the app at a build of the sky tools that is not where
/// the search expects it.
pub const SKY_LOOK_ENV: &str = "BLACK_HOLE_LAB_SKY_LOOK";

/// The program's file name on this platform.
pub const SKY_LOOK_FILE: &str = if cfg!(windows) { "sky-look.exe" } else { "sky-look" };

/// Where a release build of the sky tools puts its programs, relative to the repository root: the
/// sky workspace's own target directory, because the sky tools are built from `sky/` and not from
/// the root.
const SKY_RELEASE_DIR: &str = "sky/target/release";

/// How long the app goes on reading after `sky-look` has exited, for the lines it printed just
/// before it did.
///
/// The exit and the last line race: the child flushes its last line and exits, and the thread
/// reading the pipe may not have handed that line over by the frame that sees the exit. So the
/// verdict waits for both pipes to close. It does not wait for ever, because a pipe outlives the
/// child when something the child started inherited it - a player launched by `--open` can hold
/// standard output open for as long as it plays - and a view that finished must not wait on that.
const PIPE_GRACE: Duration = Duration::from_secs(2);

/// Windows' `CREATE_NO_WINDOW` process creation flag. A console program started from a windowed
/// program gets a console window of its own unless it is told not to, and a black window flashing
/// up over the app on every press is not something the user asked for.
#[cfg(windows)]
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

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

/// One thing a reading thread has to report.
enum Said {
    /// A line of standard output: progress, or at the end the path of the video.
    Out(String),
    /// A line of standard error: the reason for a refusal or a failure.
    Err(String),
    /// One of the two pipes has closed.
    Closed,
}

/// Where a view has got to, as `Job::poll` reports it.
pub enum Progress {
    /// Still being made; the text is the line to show while it is.
    Running(String),
    /// Finished one way or the other: the line to leave on the card, and whether it is a failure.
    Finished { text: String, failed: bool },
}

/// A view being made: the running `sky-look`, what it has said so far, and the save it was given.
///
/// Its `Drop` is the cleanup, whichever way the job ends - finished and polled, replaced, or
/// dropped with the app mid-run - so that no path out of a view leaves the child running or the
/// save on disk.
pub struct Job {
    pub who: Who,
    child: Child,
    heard: Receiver<Said>,
    /// How many of the two pipes are still open.
    open_pipes: u8,
    /// The latest line of standard output, which is the status while the view is being made and
    /// the path of the video once it is made.
    last_line: Option<String>,
    /// Every line of standard error, in order.
    complaint: Vec<String>,
    /// The temporary save the child was handed, deleted when the job is dropped.
    save: PathBuf,
    /// The child's exit and when the app saw it, once it has.
    exited: Option<(ExitStatus, Instant)>,
}

impl Job {
    /// Start `program` on `save` for `who`, and return at once.
    ///
    /// The job owns the save from here on, so a child that will not start deletes it on the way
    /// out, and so does everything after that.
    pub fn start(program: &Path, save: PathBuf, who: Who) -> std::io::Result<Job> {
        let mut command = Command::new(program);
        command
            .arg(&save)
            .arg("--observer")
            .arg(who.name().to_lowercase())
            .arg("--open")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(CREATE_NO_WINDOW);
        }
        let mut child = match command.spawn() {
            Ok(child) => child,
            Err(cause) => {
                let _ = std::fs::remove_file(&save);
                return Err(cause);
            }
        };

        // One thread a pipe, each reading lines and handing them over. Two because a child that
        // fills the pipe the app is not reading blocks on it, and a single reader of one pipe can
        // then wait for ever on a child stuck writing to the other. The threads are not joined:
        // each ends when its pipe closes, or at the first line after the job has been dropped.
        let (tell, heard) = channel();
        if let Some(out) = child.stdout.take() {
            read_lines(out, tell.clone(), Said::Out);
        }
        if let Some(err) = child.stderr.take() {
            read_lines(err, tell, Said::Err);
        }
        Ok(Job {
            who,
            child,
            heard,
            open_pipes: 2,
            last_line: None,
            complaint: Vec::new(),
            save,
            exited: None,
        })
    }

    /// Take in whatever the child has said since the last call, and say where the view has got to.
    /// Never blocks: a frame calls this once.
    pub fn poll(&mut self) -> Progress {
        while let Ok(said) = self.heard.try_recv() {
            match said {
                Said::Out(line) => self.last_line = Some(line),
                Said::Err(line) => self.complaint.push(line),
                Said::Closed => self.open_pipes = self.open_pipes.saturating_sub(1),
            }
        }
        if self.exited.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.exited = Some((status, Instant::now())),
                Ok(None) => return self.running(),
                Err(cause) => {
                    return Progress::Finished {
                        text: format!(
                            "Lost track of sky-look while it made {}'s view: {cause}",
                            self.who.name()
                        ),
                        failed: true,
                    };
                }
            }
        }
        let Some((status, at)) = self.exited else { return self.running() };
        if self.open_pipes > 0 && at.elapsed() < PIPE_GRACE {
            return self.running();
        }
        verdict(self.who, status.code(), self.last_line.as_deref(), &self.complaint)
    }

    /// The status of a view still being made: the child's latest line, or before it has printed
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
        // A child that has not exited is stopped, with whatever it started: see the module's
        // own note on closing the app mid-view.
        if self.exited.is_none() && !matches!(self.child.try_wait(), Ok(Some(_))) {
            stop(&mut self.child);
        }
        let _ = std::fs::remove_file(&self.save);
    }
}

/// Read `pipe` a line at a time on a thread of its own, sending each line as `wrap` makes it and
/// then `Said::Closed`.
///
/// Lines are read as bytes and decoded leniently, because a line that is not UTF-8 is still a line
/// of progress and must not stop the reading; the line ending is trimmed, `\r` included, and blank
/// lines are skipped rather than shown as an empty status.
fn read_lines(
    pipe: impl std::io::Read + Send + 'static,
    tell: Sender<Said>,
    wrap: fn(String) -> Said,
) {
    std::thread::spawn(move || {
        let mut reader = std::io::BufReader::new(pipe);
        let mut bytes = Vec::new();
        loop {
            bytes.clear();
            match reader.read_until(b'\n', &mut bytes) {
                Ok(0) | Err(_) => break,
                Ok(_) => {
                    let line = String::from_utf8_lossy(&bytes).trim().to_string();
                    if !line.is_empty() && tell.send(wrap(line)).is_err() {
                        return;
                    }
                }
            }
        }
        let _ = tell.send(Said::Closed);
    });
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

/// What a finished view says on the card, from how `sky-look` exited and what it printed.
fn verdict(who: Who, code: Option<i32>, last_line: Option<&str>, complaint: &[String]) -> Progress {
    let name = who.name();
    let reason = complaint.join(" ");
    let (text, failed) = match code {
        Some(0) => match last_line {
            Some(video) => (format!("Made {name}'s view and opened it: {video}"), false),
            None => (
                format!("sky-look finished {name}'s view without saying where the video is."),
                true,
            ),
        },
        Some(2) if !reason.is_empty() => (format!("No view from {name}'s place: {reason}"), true),
        Some(code) if reason.is_empty() => (
            format!(
                "Could not make {name}'s view: sky-look stopped with exit code {code} and gave no \
                 reason."
            ),
            true,
        ),
        Some(_) => (format!("Could not make {name}'s view: {reason}"), true),
        None => (format!("sky-look was stopped before it finished {name}'s view."), true),
    };
    Progress::Finished { text, failed }
}

#[cfg(test)]
mod tests;
