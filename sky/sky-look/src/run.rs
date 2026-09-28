//! A run: find the pieces, trace, render, put the files in place, say what was made.
//!
//! The two programs run as child processes, found beside this one: `sky-render` has no library to
//! call, and a child can be stopped with everything it started, which is what the app does to this
//! program when it closes. Their standard output and error are read here, a thread a pipe, and
//! never passed through: what the user sees is the sentences this program writes from them.
//!
//! [`cli`] is the whole program with the process around it taken out - the arguments, the
//! environment and the two streams are handed in - so that the tests run it in-process against
//! stand-in tools and read exactly what it wrote where, and the exit code it chose.

use std::ffi::OsString;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::channel;
use std::time::{Instant, SystemTime};

use crate::args::{self, Options, Request};
use crate::find::{self, Names, Pieces, Search};
use crate::names;
use crate::save;
use crate::scratch::{self, Scratch};

/// Windows' `CREATE_NO_WINDOW`: a console program started without a console window. The app
/// starts this program so, and this program starts its own children so, because each console
/// program started from a windowed one would otherwise flash up a black window of its own.
#[cfg(windows)]
pub const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// The size of the video and the photograph: 8K equirectangular, the size of the owner's approved
/// films and `sky-render`'s own default, stated here so that the count of red pixels can be put as
/// a share of the picture.
pub const VIDEO_SIZE: (u64, u64) = (8192, 4096);

/// How many times a second `sky-render` repeats a still's picture (its `--hold-rate` default).
const HOLD_RATE: f64 = 0.5;

/// What tracing one ray costs, in seconds of one thread, measured on the owner's machine: 13 µs at
/// the saved moment of demos/near_fall.bhl, 23 µs later in the fall, where more of the light has
/// circled the hole. The estimate a user is given spans the two.
const RAY_COST: (f64, f64) = (13e-6, 23e-6);

/// What rendering a still costs on the owner's machine at 8192 x 4096: about 4.3 s to read the map,
/// draw the picture, finish the file and write the photograph, and 0.15 s for each time the
/// encoder is handed the held picture (30 times for the minute's hold). Encoding is 2.6 s of the
/// fixed part.
const RENDER_FIXED: f64 = 4.3;
const ENCODE_FIXED: f64 = 2.6;
const ENCODE_PER_REPEAT: f64 = 0.15;

/// Where this program is and what surrounds it: everything [`cli`] would otherwise read from the
/// process.
#[derive(Debug, Clone)]
pub struct Environment {
    pub exe_dir: Option<PathBuf>,
    /// `PATH`.
    pub path: Option<OsString>,
    /// `BLACK_HOLE_LAB_SKY_MAP`.
    pub sky_env: Option<OsString>,
    /// `BLACK_HOLE_LAB_VIEWS`.
    pub views_env: Option<OsString>,
    pub videos: Option<PathBuf>,
    pub home: Option<PathBuf>,
    /// The system's temporary directory, inside which [`scratch::ROOT`] is kept.
    pub temp: PathBuf,
    pub names: Names,
    /// The threads the tracer will trace on, for the estimate of how long it takes.
    pub threads: usize,
}

impl Environment {
    /// This process's own.
    pub fn of_this_process() -> Self {
        let exe = std::env::current_exe().ok();
        let home = std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" })
            .filter(|v| !v.is_empty())
            .map(PathBuf::from);
        Self {
            exe_dir: exe.as_deref().and_then(Path::parent).map(Path::to_path_buf),
            path: std::env::var_os("PATH"),
            sky_env: std::env::var_os(find::SKY_MAP_ENV),
            views_env: std::env::var_os(names::VIEWS_ENV),
            videos: names::videos_folder(home.as_deref()),
            home,
            temp: std::env::temp_dir(),
            names: Names::native(),
            threads: std::thread::available_parallelism().map_or(1, |n| n.get()),
        }
    }
}

/// Why a run made no view.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Failure {
    /// The moment is one no view can be made from; the sentence is the tracer's own. Exit code 2.
    Refused(String),
    /// Anything else. Exit code 1.
    Failed(String),
}

impl Failure {
    pub fn code(&self) -> i32 {
        match self {
            Self::Refused(_) => 2,
            Self::Failed(_) => 1,
        }
    }

    pub fn sentence(&self) -> &str {
        match self {
            Self::Refused(s) | Self::Failed(s) => s,
        }
    }
}

/// Writes progress: one line at a time, flushed after each, because the app shows each line as it
/// arrives. A standard output that has gone away (the app closed) is not a reason to stop: the app
/// kills this program when it wants it stopped.
pub struct Say<'a> {
    out: &'a mut dyn Write,
}

impl Say<'_> {
    pub fn line(&mut self, line: &str) {
        let _ = writeln!(self.out, "{line}");
        let _ = self.out.flush();
    }
}

/// The program: parses `args`, runs, writes progress and the video's path to `out` and a failure's
/// one sentence to `err`, and returns the exit code. Nothing reaches `err` on a run that succeeds.
pub fn cli(args: &[OsString], env: &Environment, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let options = match args::parse(args) {
        Ok(Request::Help) => {
            let _ = out.write_all(args::HELP.as_bytes());
            let _ = out.flush();
            return 0;
        }
        Ok(Request::Look(options)) => options,
        Err(sentence) => {
            let _ = writeln!(err, "{sentence}");
            return 1;
        }
    };
    let mut say = Say { out };
    match look(&options, env, &mut say) {
        Ok(video) => {
            // The contract's last line: the full path, and nothing else on it.
            say.line(&video.display().to_string());
            0
        }
        Err(failure) => {
            let _ = writeln!(err, "{}", one_line(failure.sentence()));
            let _ = err.flush();
            failure.code()
        }
    }
}

/// Makes the view, and returns the video's full path.
pub fn look(o: &Options, env: &Environment, say: &mut Say) -> Result<PathBuf, Failure> {
    let started = Instant::now();
    let made_at = SystemTime::now();
    let who = o.who.name();

    // Everything that can be missing is looked for before anything is done.
    let search = Search {
        exe_dir: env.exe_dir.clone(),
        tools: o.tools.clone(),
        ffmpeg: o.ffmpeg.clone(),
        path: env.path.clone(),
        sky: o.sky.clone(),
        sky_env: env.sky_env.clone(),
    };
    let pieces = find::find(&search, &env.names).map_err(Failure::Failed)?;
    if !o.save.is_file() {
        return Err(Failure::Failed(format!(
            "There is no save at {}; name a .bhl file that Black Hole Lab wrote.",
            o.save.display()
        )));
    }
    let out_dir = names::out_dir(
        o.out_dir.as_deref(),
        env.views_env.as_deref(),
        env.videos.as_deref(),
        env.home.as_deref(),
    )
    .map_err(Failure::Failed)?;
    // Made now rather than at the end, so that a directory that cannot be made is found before
    // half a minute of work and not after it.
    std::fs::create_dir_all(&out_dir).map_err(|e| {
        Failure::Failed(format!(
            "Could not create {} for the view ({e}); name another directory with --out-dir <dir> \
             or {}.",
            out_dir.display(),
            names::VIEWS_ENV
        ))
    })?;
    let out_dir = std::path::absolute(&out_dir).unwrap_or(out_dir);
    let scratch = Scratch::new(&env.temp).map_err(|e| {
        Failure::Failed(format!(
            "Could not make a working directory in {} ({e}); check that the disk has room.",
            env.temp.join(scratch::ROOT).display()
        ))
    })?;
    let facts = save::facts(&o.save, o.who);
    let comma = facts.comma;
    let stem = names::stem(
        made_at,
        facts.name.as_deref().unwrap_or(who),
        facts.tau,
        comma,
    );
    let bundle = scratch.dir().join("bundle");

    let result = make(
        o, env, say, &pieces, &scratch, &bundle, &out_dir, &stem, comma,
    );

    // The bundle is kept whether or not the view was made: after a failure it is what there is
    // to look at. It is moved out of the scratch directory, which is about to be deleted.
    if o.keep && bundle.is_dir() {
        let kept_stem = match &result {
            Ok(placed) => placed
                .video
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| stem.clone()),
            Err(_) => stem.clone(),
        };
        let kept = out_dir.join(format!("{kept_stem} bundle"));
        match scratch::move_dir(&bundle, &kept) {
            Ok(()) => say.line(&format!(
                "Kept the traced sky bundle in {}.",
                kept.display()
            )),
            Err(e) => say.line(&format!(
                "Could not keep the traced sky bundle ({e}); it is deleted with the other \
                 intermediate files."
            )),
        }
    }
    // The intermediates go before the last line is written, so that a caller who sees the path
    // sees a run that has finished with the disk.
    drop(scratch);
    let placed = result?;

    say.line(&format!(
        "Made {who}'s view in {} seconds: the video, and beside it a 360-degree photograph of the \
         same name ending .jpg.",
        number(started.elapsed().as_secs_f64(), 1, comma)
    ));
    if o.open {
        crate::open::open(&placed.video).map_err(|e| {
            Failure::Failed(format!(
                "Made {who}'s view at {}, but could not hand it to the system's video player \
                 ({e}); open the file yourself.",
                placed.video.display()
            ))
        })?;
        say.line("Handed the video to the system's video player.");
    }
    Ok(placed.video)
}

/// Trace, render, and put the two files in place; everything that happens inside the scratch
/// directory.
#[allow(clippy::too_many_arguments)]
fn make(
    o: &Options,
    env: &Environment,
    say: &mut Say,
    pieces: &Pieces,
    scratch: &Scratch,
    bundle: &Path,
    out_dir: &Path,
    stem: &str,
    comma: bool,
) -> Result<names::Placed, Failure> {
    let who = o.who.name();
    let (w, h) = o.grid;

    // Trace.
    let rays = f64::from(w) * f64::from(h);
    let threads = env.threads.max(1) as f64;
    say.line(&format!(
        "Tracing the light that reaches {who} from every direction, {w} x {h} rays; this takes \
         about {}.",
        span(rays * RAY_COST.0 / threads, rays * RAY_COST.1 / threads)
    ));
    let clock = Instant::now();
    // The bundle directory third, for the tests' stand-in scripts, as below.
    let trace_args: Vec<OsString> = vec![
        o.save.clone().into(),
        "--out".into(),
        bundle.into(),
        "--observer".into(),
        o.who.flag().into(),
        "--frames".into(),
        "1".into(),
        "--grid".into(),
        format!("{w}x{h}").into(),
    ];
    let traced = run_child(&pieces.trace, &trace_args, |_| {}).map_err(|e| {
        Failure::Failed(format!(
            "{} could not be started ({e}); build the sky tools again with cargo build --release \
             in the sky directory.",
            pieces.trace.display()
        ))
    })?;
    trace_verdict(&traced, &o.save, &env.temp)?;
    say.line(&format!(
        "Traced the light that reaches {who} in {} seconds.",
        number(clock.elapsed().as_secs_f64(), 1, comma)
    ));

    // Render.
    let video = scratch.dir().join("view.mkv");
    let photo = scratch.dir().join("view.jpg");
    let repeats = (o.hold * HOLD_RATE).ceil().max(1.0);
    say.line(&format!(
        "Rendering {who}'s view over the star map at {} x {}, as a video held for {} seconds and \
         as a photograph; this takes about {}.",
        VIDEO_SIZE.0,
        VIDEO_SIZE.1,
        number(o.hold, 0, comma),
        seconds(RENDER_FIXED + ENCODE_PER_REPEAT * repeats)
    ));
    let clock = Instant::now();
    // In this order so that the tests' stand-in scripts find the three paths they act on as their
    // second, fourth and sixth arguments: `cmd` numbers only nine.
    let mut render_args: Vec<OsString> = vec![
        "--out".into(),
        video.clone().into(),
        "--photo".into(),
        photo.clone().into(),
        "--bundle".into(),
        bundle.into(),
        "--sky".into(),
        pieces.sky.clone().into(),
        "--ffmpeg".into(),
        pieces.ffmpeg.clone().into(),
        "--size".into(),
        format!("{}x{}", VIDEO_SIZE.0, VIDEO_SIZE.1).into(),
        "--still".into(),
        "--hold".into(),
        o.hold.to_string().into(),
        "--readouts".into(),
        "overlay".into(),
    ];
    if let Some(stops) = o.exposure {
        render_args.extend(["--exposure".into(), stops.to_string().into()]);
    }
    if comma {
        render_args.push("--decimal-comma".into());
    }
    let mut tally = None;
    let rendered = run_child(&pieces.render, &render_args, |line| {
        if line.starts_with("map ") {
            say.line(&format!("Read the star map; drawing {who}'s view."));
        } else if line.starts_with("frame ") {
            say.line(&format!(
                "Drew {who}'s view; encoding the video, which takes about {}.",
                seconds(ENCODE_FIXED + ENCODE_PER_REPEAT * repeats)
            ));
        } else if let Some(counts) = parse_tally(line) {
            tally = Some(counts);
        }
    })
    .map_err(|e| {
        Failure::Failed(format!(
            "{} could not be started ({e}); build the sky tools again with cargo build --release \
             in the sky directory.",
            pieces.render.display()
        ))
    })?;
    if rendered.code != Some(0) {
        return Err(Failure::Failed(format!(
            "sky-render could not make {who}'s view: {}",
            full_stop(&complaint(&rendered, "sky-render: ", "sky-render"))
        )));
    }
    if !video.is_file() || !photo.is_file() {
        return Err(Failure::Failed(format!(
            "sky-render finished without writing {who}'s view; build the sky tools again with \
             cargo build --release in the sky directory."
        )));
    }
    say.line(&format!(
        "Rendered {who}'s view in {} seconds.",
        number(clock.elapsed().as_secs_f64(), 1, comma)
    ));

    let placed = names::place(&video, &photo, out_dir, stem).map_err(|e| {
        Failure::Failed(format!(
            "Could not put {who}'s view in {} ({e}); check that the disk has room and that the \
             directory can be written to.",
            out_dir.display()
        ))
    })?;
    if let Some(sentence) = tally.and_then(|t| red_sentence(t, comma)) {
        say.line(&sentence);
    }
    Ok(placed)
}

/// How a child process ended.
#[derive(Debug)]
struct Ended {
    /// The exit code; `None` when it was stopped without one.
    code: Option<i32>,
    /// Every line it wrote to standard error.
    complaint: Vec<String>,
}

/// Runs `program` with `args`, handing each line of its standard output to `heard` as it comes and
/// keeping its standard error, and waits for it.
///
/// A thread a pipe, as the app reads this program: a child that fills the pipe nobody is reading
/// stops, and one reader could wait for ever on one pipe while the child waits on the other. Lines
/// are read as bytes and decoded leniently; the line ending, `\r` included, is trimmed.
fn run_child(
    program: &Path,
    args: &[OsString],
    mut heard: impl FnMut(&str),
) -> std::io::Result<Ended> {
    let mut command = Command::new(program);
    command
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(CREATE_NO_WINDOW);
    }
    let mut child = command.spawn()?;
    enum Said {
        Out(String),
        Err(String),
        Closed,
    }
    /// A pipe of the child's, and how its lines are to be told apart from the other's.
    type Pipe = (Option<Box<dyn std::io::Read + Send>>, fn(String) -> Said);
    let (tell, listen) = channel();
    let pipes: [Pipe; 2] = [
        (child.stdout.take().map(|p| Box::new(p) as _), Said::Out),
        (child.stderr.take().map(|p| Box::new(p) as _), Said::Err),
    ];
    let mut open = 0;
    for (pipe, wrap) in pipes {
        let Some(pipe) = pipe else { continue };
        open += 1;
        let tell = tell.clone();
        std::thread::spawn(move || {
            let mut reader = std::io::BufReader::new(pipe);
            let mut bytes = Vec::new();
            while matches!(reader.read_until(b'\n', &mut bytes), Ok(n) if n > 0) {
                let line = String::from_utf8_lossy(&bytes).trim().to_string();
                bytes.clear();
                if !line.is_empty() {
                    let _ = tell.send(wrap(line));
                }
            }
            let _ = tell.send(Said::Closed);
        });
    }
    drop(tell);
    let mut complaint = Vec::new();
    while open > 0 {
        match listen.recv() {
            Ok(Said::Out(line)) => heard(&line),
            Ok(Said::Err(line)) => complaint.push(line),
            Ok(Said::Closed) => open -= 1,
            Err(_) => break,
        }
    }
    let status = child.wait()?;
    Ok(Ended {
        code: status.code(),
        complaint,
    })
}

/// What the tracer's exit means.
///
/// `sky-trace` exits with 2 for anything it will not act on, before it has written anything: a
/// moment no film can be made from, and also a command line or a save it cannot use. The command
/// line is this program's, checked before it was passed on, so what can come back is the moment
/// or the save. A save that cannot be read is not a moment refused, and the tracer says so by
/// starting its sentence with the save's path, as it prints every failure to read one; that one is
/// a failure, exit 1. Every other refusal is passed on unchanged but for the tracer's `sky-trace: `
/// prefix: it is physics this program did not compute, in the tracer's own words. Exit 1 from the
/// tracer is a bundle whose writing failed, which is a disk to see to.
fn trace_verdict(ended: &Ended, save: &Path, temp: &Path) -> Result<(), Failure> {
    let said = complaint(ended, "sky-trace: ", "sky-trace");
    match ended.code {
        Some(0) => Ok(()),
        Some(2) if said.starts_with(&format!("{}: ", save.display())) => Err(Failure::Failed(
            format!("sky-trace could not read the save: {}", full_stop(&said)),
        )),
        Some(2) if !ended.complaint.is_empty() => Err(Failure::Refused(said)),
        Some(1) => Err(Failure::Failed(format!(
            "sky-trace could not write the traced light ({said}); check that the disk holding {} \
             has room, and try again.",
            temp.display()
        ))),
        _ => Err(Failure::Failed(full_stop(&said))),
    }
}

/// A child's standard error as one line, without the program's own prefix; or, when it said
/// nothing, a sentence saying how it stopped.
fn complaint(ended: &Ended, prefix: &str, program: &str) -> String {
    let joined = ended
        .complaint
        .iter()
        .map(|line| line.strip_prefix(prefix).unwrap_or(line))
        .collect::<Vec<_>>()
        .join(" ");
    if !joined.is_empty() {
        return joined;
    }
    match ended.code {
        Some(code) => format!("{program} stopped with exit code {code} and gave no reason"),
        None => format!("{program} was stopped before it finished"),
    }
}

/// `text` ending in exactly one full stop, for a sentence this program composes around another's.
fn full_stop(text: &str) -> String {
    let text = text.trim_end();
    if text.ends_with(['.', '!', '?']) {
        text.to_string()
    } else {
        format!("{text}.")
    }
}

/// The sentence on standard error is one line, whatever went into it.
fn one_line(text: &str) -> String {
    text.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// The renderer's counts of what it drew, from its line
/// `pixels drawn over the 1 frame(s): unresolved 0 (0 %), under-sampled 55728 (0.166 %), dark ...`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tally {
    pub unresolved: u64,
    pub undersampled: u64,
}

pub fn parse_tally(line: &str) -> Option<Tally> {
    let counts = line.strip_prefix("pixels drawn over ")?;
    let after = |word: &str| -> Option<u64> {
        let from = counts.find(word)? + word.len();
        counts[from..]
            .split(|c: char| !c.is_ascii_digit())
            .next()?
            .parse()
            .ok()
    };
    Some(Tally {
        unresolved: after("unresolved ")?,
        undersampled: after("under-sampled ")?,
    })
}

/// The one sentence that explains the red in a view, or `None` when there is none.
///
/// Two kinds of pixel are red, and they are in different places. Under-sampled pixels lie along
/// the edge of the dark region, where light has circled the hole and the traced rays are too far
/// apart to say which part of the sky it came from. Unresolved pixels are where the tracer could
/// not follow the light back at all, which can be anywhere. The sentence says which it is.
pub fn red_sentence(t: Tally, comma: bool) -> Option<String> {
    let total = t.unresolved + t.undersampled;
    if total == 0 {
        return None;
    }
    let share = percent(total, VIDEO_SIZE.0 * VIDEO_SIZE.1, comma);
    let edge = "along the edge of the dark region, where the traced rays are too far apart to say \
                which part of the sky the light came from";
    let lost = "where the tracer could not follow the light back to where it came from";
    Some(match (t.undersampled > 0, t.unresolved > 0) {
        (true, false) => format!(
            "{total} pixels ({share} of the picture) could not be determined and are drawn in red \
             {edge}."
        ),
        (false, true) => format!(
            "{total} pixels ({share} of the picture) could not be determined and are drawn in red, \
             {lost}."
        ),
        _ => format!(
            "{total} pixels ({share} of the picture) could not be determined and are drawn in red: \
             {} {edge}, and {} {lost}.",
            t.undersampled, t.unresolved
        ),
    })
}

/// `n` of `of` as a percentage to three significant figures, as `sky-render` writes it.
fn percent(n: u64, of: u64, comma: bool) -> String {
    let p = 100.0 * n as f64 / of.max(1) as f64;
    let decimals = (2 - p.log10().floor() as i32).clamp(0, 6) as usize;
    format!("{} %", number(p, decimals, comma))
}

/// `x` to `decimals` places, with the decimal mark the app was set to.
fn number(x: f64, decimals: usize, comma: bool) -> String {
    let text = format!("{x:.decimals$}");
    if comma { text.replace('.', ",") } else { text }
}

/// An estimated duration, rounded to the whole second: "a second", "9 seconds".
fn seconds(s: f64) -> String {
    let s = s.round().max(1.0);
    if s == 1.0 {
        "a second".into()
    } else {
        format!("{s} seconds")
    }
}

/// An estimated range of durations: "7 to 12 seconds", or one figure when they round together.
fn span(low: f64, high: f64) -> String {
    let (low, high) = (low.round().max(1.0), high.round().max(1.0));
    if low == high {
        seconds(low)
    } else {
        format!("{low} to {high} seconds")
    }
}
