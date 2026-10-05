//! A run: find the pieces, make the view's folder and put the save in it, trace, render, say what
//! was made and where on it to look for the read-outs and the marks of the directions of travel.
//!
//! The two programs run as child processes, found beside this one: `sky-render` has no library to
//! call, and a child can be stopped with everything it started, which is what the app does to this
//! program when the user cancels or closes it. Their standard output and error are read here, a
//! thread a pipe, and never passed through: what the user sees is the sentences this program writes
//! from them, among them the percentage each program says it has done.
//!
//! Everything a run makes goes straight into the view's folder (`names`), and nothing is deleted:
//! the bundle is traced into the folder, the photograph rendered into it, and a run that fails
//! leaves the folder as it stood, to be looked into. So the commands written to `commands.txt`
//! name the files that are really there, and run again as they stand.
//!
//! [`cli`] is the whole program with the process around it taken out - the arguments, the
//! environment and the two streams are handed in - so that the tests run it in-process against
//! stand-in tools and read exactly what it wrote where, and the exit code it chose.

use std::ffi::{OsStr, OsString};
use std::fs::File;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::channel;
use std::time::{Instant, SystemTime};

use crate::args::{self, CheckOptions, Options, Request};
use crate::fetch;
use crate::find::{self, Names, Pieces, Search};
use crate::names;
use crate::open::{self, ViewerSearch};
use crate::save;
use crate::shell;

/// Windows' `CREATE_NO_WINDOW`: a console program started without a console window. This program
/// starts its own children so, because each console program started from a windowed one would
/// otherwise flash up a black window of its own, and the tracer's and renderer's output is read
/// here and never shown as it stands.
#[cfg(windows)]
pub const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// The size of the photograph: 8K equirectangular, the size of the owner's approved films and
/// `sky-render`'s own default, stated here so that the count of red pixels can be put as a share
/// of the picture.
pub const PHOTO_SIZE: (u64, u64) = (8192, 4096);

/// The least rise in a child's percentage that is passed on as a sentence. The tracer and the
/// renderer, run without `--verbose` as this program runs them, print their percentage every ten
/// points, and every one of those is passed on; the step is five and not ten so that a child run
/// by hand with `--verbose`, printing every point, is still passed on at no more than twenty lines,
/// about one every half second on the owner's machine.
const PERCENT_STEP: u32 = 5;

/// The file in the view's folder that holds the commands this program ran.
pub const COMMANDS_FILE: &str = "commands.txt";

/// The file in the view's folder that holds every progress sentence and the verdict.
pub const LOG_FILE: &str = "log.txt";

/// The folder in the view's folder that the tracer writes the sky bundle into.
pub const BUNDLE_DIR: &str = "bundle";

/// What `commands.txt` says before the commands.
const COMMANDS_HEADER: &str = "\
# The commands sky-look ran to make this view, in order, as PowerShell reads them: paste a line
# into PowerShell to run it again. sky-trace will not write over the bundle folder that is here
# (give it --resume, or another --out), and sky-render will not write over the photograph (give
# it --overwrite, or another --photo). sky-render runs ffmpeg itself to write the photograph.
";

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
    /// `BLACK_HOLE_LAB_VIEWER`.
    pub viewer_env: Option<OsString>,
    /// Where VLC would be, in the order to try (`open::vlc_places`).
    pub vlc: Vec<PathBuf>,
    /// Where ffmpeg is installed to, looked in after the PATH (`find::ffmpeg_places_here`).
    pub ffmpeg_places: Vec<PathBuf>,
    /// The text of `/etc/os-release`, where there is one (`find::os_release_here`).
    pub os_release: Option<String>,
    /// This user's data folder for Black Hole Lab (`find::user_data_dir_here`).
    pub data_dir: Option<PathBuf>,
    pub videos: Option<PathBuf>,
    pub home: Option<PathBuf>,
    pub names: Names,
    /// Starts the command prompt `--shell` leaves in the view's folder: `shell::start`, which a
    /// test replaces so that no prompt opens on the desktop of whoever runs the tests.
    pub prompt: fn(&Path) -> std::io::Result<()>,
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
            viewer_env: std::env::var_os(open::VIEWER_ENV),
            vlc: open::vlc_places_here(),
            ffmpeg_places: find::ffmpeg_places_here(),
            os_release: find::os_release_here(),
            data_dir: find::user_data_dir_here(),
            videos: names::videos_folder(home.as_deref()),
            home,
            names: Names::native(),
            prompt: shell::start,
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

    /// The word the status file's last line opens with for this failure.
    pub fn word(&self) -> &'static str {
        match self {
            Self::Refused(_) => "refused",
            Self::Failed(_) => "failed",
        }
    }
}

/// Where everything a run says goes: its sentences of progress, its verdict, the commands it runs
/// and what those programs print.
///
/// Every sentence and the verdict go to the status file when `--status` names one, and to
/// `log.txt` in the view's folder once there is a folder - with every line said before then
/// written there first, so the log is the whole run. A line goes to the status file in one write of
/// the whole line and its line feed, so that the app, which reads the file while it grows, finds
/// either the whole line or none of it in all but the rarest case, and waits for the line feed in
/// that one.
///
/// What reaches the console depends on who is watching it:
///
/// - At a terminal, with no `--shell`: the sentences, each command just before it runs, and at the
///   end the photograph's path on standard output, or the failure's sentence on standard error.
///   This program is the whole of what the person asked for, and speaks for the programs it runs.
/// - In the rendering terminal (`--shell`): each command, and then what that program prints,
///   standard output and standard error alike, byte for byte as it arrives, and nothing of this
///   program's own. The person who ticked Show Rendering Terminal asked to watch the tools at work;
///   the app's card already carries the sentences, and the same sentences again between the
///   programs' own lines would be the same story told twice in two voices. The terminal is the
///   place to see exactly what `sky-trace` and `sky-render` said, which the sentences leave out.
///
/// None of these is a reason to stop if it goes away: the app stops this program when it wants it
/// stopped.
pub struct Say<'a> {
    out: &'a mut dyn Write,
    err: &'a mut dyn Write,
    /// Whether this is the rendering terminal (`--shell`).
    terminal: bool,
    status: Option<File>,
    log: Option<File>,
    /// Every line written to the status file so far, for `log.txt` when the folder is made.
    said: Vec<String>,
    /// The view's folder, once it is made.
    folder: Option<PathBuf>,
}

impl<'a> Say<'a> {
    /// Progress for the console `out` and `err`, as `terminal` decides, and for the file at
    /// `status` too when there is one: opened to append, and made if it is not there.
    pub fn new(
        out: &'a mut dyn Write,
        err: &'a mut dyn Write,
        terminal: bool,
        status: Option<&Path>,
    ) -> std::io::Result<Self> {
        let status = match status {
            Some(path) => Some(
                std::fs::OpenOptions::new()
                    .append(true)
                    .create(true)
                    .open(path)?,
            ),
            None => None,
        };
        Ok(Self {
            out,
            err,
            terminal,
            status,
            log: None,
            said: Vec::new(),
            folder: None,
        })
    }

    /// A sentence of progress: the status file and the log, and standard output unless this is the
    /// rendering terminal.
    pub fn line(&mut self, line: &str) {
        if !self.terminal {
            write_line(self.out, line);
        }
        self.record(line);
    }

    /// A command about to be run, on standard output wherever the console is.
    pub fn command(&mut self, line: &str) {
        write_line(self.out, line);
    }

    /// The photograph's path, the last line of standard output of a run at a terminal; nothing in
    /// the rendering terminal.
    pub fn result(&mut self, path: &str) {
        if !self.terminal {
            write_line(self.out, path);
        }
    }

    /// A failure's sentence, on standard error at a terminal; nothing in the rendering terminal,
    /// where the program that failed has said why in its own words above.
    pub fn complain(&mut self, sentence: &str) {
        if !self.terminal {
            write_line(self.err, sentence);
        }
    }

    /// A line a child printed: passed through as it came, to this program's standard output or
    /// error as the child printed it, in the rendering terminal and nowhere else; and handed back,
    /// when it was standard output, as the text this program reads its percentages and its
    /// read-out, mark and tally lines from - which it does the same way wherever the console is.
    fn echo<'l>(&mut self, heard: Heard<'l>) -> Option<&'l str> {
        let (to, raw, line) = match heard {
            Heard::Out { raw, line } => (&mut *self.out, raw, Some(line)),
            Heard::Err { raw } => (&mut *self.err, raw, None),
        };
        if self.terminal {
            let _ = to.write_all(raw);
            let _ = to.flush();
        }
        line
    }

    /// The last line of the status file and the log: `word`, a space, and `text` on one line.
    pub fn verdict(&mut self, word: &str, text: &str) {
        self.record(&format!("{word} {}", one_line(text)));
    }

    /// A line for the status file and the log.
    fn record(&mut self, line: &str) {
        let whole = format!("{line}\n");
        for file in [&mut self.status, &mut self.log].into_iter().flatten() {
            let _ = file.write_all(whole.as_bytes());
        }
        self.said.push(line.to_string());
    }

    /// From now on the log is kept in `folder`, beginning with every line said so far.
    fn keep_in(&mut self, folder: &Path) -> std::io::Result<()> {
        let mut log = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(folder.join(LOG_FILE))?;
        for line in &self.said {
            log.write_all(format!("{line}\n").as_bytes())?;
        }
        self.log = Some(log);
        self.folder = Some(folder.to_path_buf());
        Ok(())
    }

    /// The view's folder, if the run got as far as making it.
    pub fn folder(&self) -> Option<&Path> {
        self.folder.as_deref()
    }
}

/// `line` and a line feed on `to`, flushed.
fn write_line(to: &mut dyn Write, line: &str) {
    let _ = writeln!(to, "{line}");
    let _ = to.flush();
}

/// The program: parses `args`, runs, writes progress and the photograph's path to `out` and a
/// failure's one sentence to `err` (or, in the rendering terminal, the commands and the programs'
/// own output: see [`Say`]), writes the verdict to the status file, starts the prompt that
/// `--shell` asks for, and returns the exit code. Nothing of this program's reaches `err` on a run
/// that succeeds.
pub fn cli(args: &[OsString], env: &Environment, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let options = match args::parse(args) {
        Ok(Request::Help) => {
            let _ = out.write_all(args::HELP.as_bytes());
            let _ = out.flush();
            return 0;
        }
        Ok(Request::Check(options)) => return check(&options, env, out),
        Ok(Request::FetchMap { status }) => return fetch_map(status.as_deref(), env, out, err),
        Ok(Request::Look(options)) => options,
        Err(sentence) => {
            // A command line that names a status file still gets its verdict there, so that the
            // app shows this sentence rather than a program that stopped without saying why.
            if let Some(path) = args::status_named(args)
                && let Ok(mut say) = Say::new(
                    &mut std::io::sink(),
                    &mut std::io::sink(),
                    false,
                    Some(&path),
                )
            {
                say.verdict("failed", &sentence);
            }
            let _ = writeln!(err, "{sentence}");
            return 1;
        }
    };
    let mut say = match Say::new(out, err, options.shell, options.status.as_deref()) {
        Ok(say) => say,
        Err(e) => {
            // Said on standard error even in the rendering terminal: with no status file, the
            // console is the only place left to say it.
            let _ = writeln!(
                err,
                "Could not open the status file {} ({e}); name a file in a directory that can be \
                 written to.",
                options.status.as_deref().unwrap_or(Path::new("")).display()
            );
            return 1;
        }
    };
    let code = match look(&options, env, &mut say) {
        Ok(photo) => {
            let path = photo.display().to_string();
            say.verdict("done", &path);
            // The contract's last line of standard output: the full path, and nothing else on it.
            say.result(&path);
            0
        }
        Err(failure) => {
            say.verdict(failure.word(), failure.sentence());
            say.complain(&one_line(failure.sentence()));
            failure.code()
        }
    };
    if options.shell {
        // After the verdict, so that the app has its answer before the prompt starts. With no
        // folder - the run stopped before one was made - the prompt starts where this program was
        // started. Nothing is said of it: the prompt, showing the folder, says it for itself, and
        // a prompt that will not start leaves nothing attached to keep the window open to read a
        // sentence in.
        let dir = say
            .folder()
            .map(Path::to_path_buf)
            .or_else(|| std::env::current_dir().ok())
            .unwrap_or_else(|| PathBuf::from("."));
        let _ = (env.prompt)(&dir);
    }
    code
}

/// `--check`: the report of [`find::check`] on `out`, a piece a line, and 0 when nothing is
/// missing, else 1. Nothing is written anywhere else and nothing is started.
fn check(o: &CheckOptions, env: &Environment, out: &mut dyn Write) -> i32 {
    let search = Search {
        exe_dir: env.exe_dir.clone(),
        tools: o.tools.clone(),
        ffmpeg: o.ffmpeg.clone(),
        path: env.path.clone(),
        sky: o.sky.clone(),
        sky_env: env.sky_env.clone(),
        ffmpeg_places: env.ffmpeg_places.clone(),
        os_release: env.os_release.clone(),
        data_dir: env.data_dir.clone(),
    };
    let report = find::check(&search, &env.names);
    for line in report.lines() {
        write_line(out, &one_line(&line));
    }
    if report.complete() { 0 } else { 1 }
}

/// `--fetch-map`: the download's sentences on `out` and in the status file, and the verdict there
/// as a view's is - `done <the map's full path>` or `failed <sentence>` - with the path last on
/// `out`, or the sentence on `err`. 0 for a map that is there at the end, else 1.
fn fetch_map(
    status: Option<&Path>,
    env: &Environment,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> i32 {
    let mut say = match Say::new(out, err, false, status) {
        Ok(say) => say,
        Err(e) => {
            let _ = writeln!(
                err,
                "Could not open the status file {} ({e}); name a file in a directory that can be \
                 written to.",
                status.unwrap_or(Path::new("")).display()
            );
            return 1;
        }
    };
    match fetch::fetch_default(env, &mut say) {
        Ok(map) => {
            let path = map.display().to_string();
            say.verdict("done", &path);
            say.result(&path);
            0
        }
        Err(sentence) => {
            say.verdict("failed", &sentence);
            say.complain(&one_line(&sentence));
            1
        }
    }
}

/// Makes the view, and returns the photograph's full path.
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
        ffmpeg_places: env.ffmpeg_places.clone(),
        os_release: env.os_release.clone(),
        data_dir: env.data_dir.clone(),
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
    let out_dir = std::path::absolute(&out_dir).unwrap_or(out_dir);
    let cannot_make = |e: std::io::Error| {
        Failure::Failed(format!(
            "Could not create a folder for the view in {} ({e}); name another directory with \
             --out-dir <dir> or {}.",
            out_dir.display(),
            names::VIEWS_ENV
        ))
    };
    std::fs::create_dir_all(&out_dir).map_err(cannot_make)?;
    let facts = save::facts(&o.save, o.who);
    let comma = facts.comma;
    let stem = names::stem(facts.name.as_deref().unwrap_or(who), facts.tau, comma);
    let folder = names::make_folder(&out_dir, &stem).map_err(cannot_make)?;
    // The folder's own name, which is `stem` unless another view already had it.
    let name = folder
        .file_name()
        .map_or_else(|| stem.clone(), |n| n.to_string_lossy().into_owned());
    say.keep_in(&folder).map_err(|e| {
        Failure::Failed(format!(
            "Could not write {LOG_FILE} in {} ({e}); check that the disk has room.",
            folder.display()
        ))
    })?;
    say.line(&format!(
        "Making {who}'s view in {}, at {}.",
        folder.display(),
        names::utc_stamp(made_at)
    ));

    // The save goes into the folder first of all, so that every command after this names the
    // copy that stays with the view, and the app's temporary save, moved, is no longer the app's
    // to delete.
    let save = folder.join(format!("{name}.bhl"));
    let placed = if o.move_save {
        names::move_file(&o.save, &save)
    } else {
        std::fs::copy(&o.save, &save).map(drop)
    };
    let result = placed
        .map_err(|e| {
            Failure::Failed(format!(
                "Could not put the save {} in {} ({e}); check that the disk has room.",
                o.save.display(),
                folder.display()
            ))
        })
        .and_then(|()| make(o, say, &pieces, &folder, &save, &name, comma));
    let photo = match result {
        Ok(photo) => photo,
        Err(failure) => {
            say.line(&format!(
                "The files of this run are kept in {}.",
                folder.display()
            ));
            return Err(failure);
        }
    };

    say.line(&format!(
        "Made {who}'s view in {} seconds: a 360-degree photograph.",
        number(started.elapsed().as_secs_f64(), 1, comma)
    ));
    if o.open {
        // Chosen only now, after the work: a viewer named wrongly costs the user the opening and
        // not the view, which is made and in its place, and the sentence says where.
        let search = ViewerSearch {
            flag: o.viewer.clone(),
            env: env.viewer_env.clone(),
            vlc: env.vlc.clone(),
        };
        let viewer = open::choose(&search).map_err(|why| {
            Failure::Failed(format!(
                "Made {who}'s view at {}, but {why}.",
                photo.display()
            ))
        })?;
        open::open(&viewer, &photo).map_err(|e| {
            Failure::Failed(format!(
                "Made {who}'s view at {}, but could not start {} to show it ({e}); open the file \
                 yourself.",
                photo.display(),
                open::viewer_name(&viewer)
            ))
        })?;
        say.line(&open::opened_sentence(&viewer));
    }
    Ok(photo)
}

/// Trace and render into `folder`, from the `save` already there, and return the photograph's
/// path: `name.jpg` in the folder.
fn make(
    o: &Options,
    say: &mut Say,
    pieces: &Pieces,
    folder: &Path,
    save: &Path,
    name: &str,
    comma: bool,
) -> Result<PathBuf, Failure> {
    let who = o.who.name();
    let (w, h) = o.grid;
    let bundle = folder.join(BUNDLE_DIR);
    let mut commands = Commands::new(folder);

    // Trace.
    say.line(&format!(
        "Tracing the light that reaches {who} from every direction, {w} x {h} rays."
    ));
    let clock = Instant::now();
    // The bundle directory third, for the tests' stand-in scripts, as below. `--units` decides
    // what the tracer writes into the bundle for the read-outs, which the renderer then paints as
    // they are.
    let trace_args: Vec<OsString> = vec![
        save.into(),
        "--out".into(),
        bundle.clone().into(),
        "--observer".into(),
        o.who.flag().into(),
        "--frames".into(),
        "1".into(),
        "--grid".into(),
        format!("{w}x{h}").into(),
        "--units".into(),
        o.units.flag().into(),
    ];
    commands.ran(say, &pieces.trace, &trace_args);
    let mut shown = 0;
    let traced = run_child(&pieces.trace, &trace_args, |heard| {
        let Some(line) = say.echo(heard) else { return };
        if let Some(sentence) = percent_sentence(line, &mut shown, || {
            format!("Tracing the light that reaches {who}")
        }) {
            say.line(&sentence);
        }
    })
    .map_err(|e| {
        Failure::Failed(format!(
            "{} could not be started ({e}); build the sky tools again with cargo build --release \
             in the sky directory.",
            pieces.trace.display()
        ))
    })?;
    trace_verdict(&traced, save, folder)?;
    say.line(&format!(
        "Traced the light that reaches {who} in {} seconds.",
        number(clock.elapsed().as_secs_f64(), 1, comma)
    ));

    // Render: the photograph alone. `--encoder none` makes no video, `--still` draws the one
    // traced frame, `--readouts panel` paints the read-outs into the picture (a photograph has no
    // subtitle track to carry them), and `--readout-at dark` puts that panel inside the hole's
    // dark region, where it hides none of the sky, or below the opening view when the dark region
    // is too small to hold it. The renderer draws the marks of the directions of travel on the
    // sky of a still by itself, and says where each one is.
    let photo = folder.join(format!("{name}.jpg"));
    say.line(&format!(
        "Rendering {who}'s view over the star map as a 360-degree photograph of {} x {} pixels.",
        PHOTO_SIZE.0, PHOTO_SIZE.1,
    ));
    let clock = Instant::now();
    // In this order so that the tests' stand-in scripts find the two paths they act on as their
    // second and fourth arguments: `cmd` numbers only nine.
    let mut render_args: Vec<OsString> = vec![
        "--photo".into(),
        photo.clone().into(),
        "--bundle".into(),
        bundle.into(),
        "--sky".into(),
        pieces.sky.clone().into(),
        "--ffmpeg".into(),
        pieces.ffmpeg.clone().into(),
        "--size".into(),
        format!("{}x{}", PHOTO_SIZE.0, PHOTO_SIZE.1).into(),
        "--encoder".into(),
        "none".into(),
        "--still".into(),
        "--readouts".into(),
        "panel".into(),
        "--readout-at".into(),
        "dark".into(),
    ];
    if let Some(stops) = o.exposure {
        render_args.extend(["--exposure".into(), stops.to_string().into()]);
    }
    if comma {
        render_args.push("--decimal-comma".into());
    }
    commands.ran(say, &pieces.render, &render_args);
    let mut tally = None;
    let mut shown = 0;
    let rendered = run_child(&pieces.render, &render_args, |heard| {
        let Some(line) = say.echo(heard) else { return };
        if let Some(sentence) =
            percent_sentence(line, &mut shown, || format!("Rendering {who}'s view"))
        {
            say.line(&sentence);
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
    if !photo.is_file() {
        return Err(Failure::Failed(format!(
            "sky-render finished without writing {who}'s view; build the sky tools again with \
             cargo build --release in the sky directory."
        )));
    }
    say.line(&format!(
        "Rendered {who}'s view in {} seconds.",
        number(clock.elapsed().as_secs_f64(), 1, comma)
    ));

    if let Some(sentence) = tally.and_then(|t| red_sentence(t, comma)) {
        say.line(&sentence);
    }
    Ok(photo)
}

/// The commands a run has started, written to `commands.txt` in the view's folder and printed on
/// standard output, each just before it is run, so that a run that fails part-way still has the
/// command that failed on record.
struct Commands {
    file: PathBuf,
    /// Whether the file has its header yet.
    begun: bool,
}

impl Commands {
    fn new(folder: &Path) -> Self {
        Self {
            file: folder.join(COMMANDS_FILE),
            begun: false,
        }
    }

    /// `program` is about to be started with `args`.
    fn ran(&mut self, say: &mut Say, program: &Path, args: &[OsString]) {
        let line = powershell_line(program, args);
        say.command(&line);
        let mut text = String::new();
        if !self.begun {
            text.push_str(COMMANDS_HEADER);
            self.begun = true;
        }
        text.push_str(&line);
        text.push('\n');
        // The record is a courtesy: a folder that takes the log takes this, and a failure to
        // write it is no reason to refuse the view.
        let _ = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(&self.file)
            .and_then(|mut file| file.write_all(text.as_bytes()));
    }
}

/// `program` and `args` as one line PowerShell runs as it stands: `& 'C:\...\sky-trace.exe' ...`.
///
/// The call operator `&` first, because PowerShell reads a line that opens with a quoted string as
/// the string and not as a program to run. An argument made only of letters, digits and `-`, `_`,
/// `.` and `+` goes as it is: PowerShell passes every such word to a program unchanged. Anything
/// else - every path, with its spaces, colons and backslashes - goes in single quotes, inside
/// which PowerShell reads nothing as syntax but the quote itself, which is doubled; PowerShell
/// takes the typographic single quotes for quotes too, so they are doubled as well.
pub fn powershell_line(program: &Path, args: &[OsString]) -> String {
    let mut line = format!("& {}", powershell_word(program.as_os_str()));
    for arg in args {
        line.push(' ');
        line.push_str(&powershell_word(arg));
    }
    line
}

fn powershell_word(word: &OsStr) -> String {
    let word = word.to_string_lossy();
    let plain = !word.is_empty()
        && word
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || "-_.+".contains(c));
    if plain {
        return word.into_owned();
    }
    let mut quoted = String::from("'");
    for c in word.chars() {
        if matches!(c, '\'' | '\u{2018}' | '\u{2019}' | '\u{201A}' | '\u{201B}') {
            quoted.push(c);
        }
        quoted.push(c);
    }
    quoted.push('\'');
    quoted
}

/// The sentence a child's `progress <p>%` line becomes - `what`, a colon and the percentage - or
/// `None` for any other line, or for a percentage less than [`PERCENT_STEP`] above the last one
/// `shown`. 100 % is never said: the sentence after it says the stage is done, and how long it
/// took.
pub fn percent_sentence(
    line: &str,
    shown: &mut u32,
    what: impl FnOnce() -> String,
) -> Option<String> {
    let p: u32 = line
        .strip_prefix("progress ")?
        .strip_suffix('%')?
        .trim()
        .parse()
        .ok()?;
    if p >= 100 || p < *shown + PERCENT_STEP {
        return None;
    }
    *shown = p;
    Some(format!("{}: {p}%.", what()))
}

/// How a child process ended.
#[derive(Debug)]
struct Ended {
    /// The exit code; `None` when it was stopped without one.
    code: Option<i32>,
    /// Every line it wrote to standard error.
    complaint: Vec<String>,
}

/// One line a child printed, as [`run_child`] hands it over.
enum Heard<'a> {
    /// A line of standard output: the bytes as they came, line ending and all, and the line as
    /// text, decoded leniently and trimmed, for this program to read.
    Out { raw: &'a [u8], line: &'a str },
    /// A line of standard error, as it came.
    Err { raw: &'a [u8] },
}

/// Runs `program` with `args`, handing each line it prints to `heard` as it comes - standard output
/// and standard error both, in the order they arrive - keeping its standard error for the verdict,
/// and waits for it.
///
/// A thread a pipe: a child that fills the pipe nobody is reading stops, and one reader could wait
/// for ever on one pipe while the child waits on the other. Every line is handed over as the bytes
/// that came, blank ones too, so that the rendering terminal can show exactly what the child
/// printed; the text this program reads is decoded leniently, with the line ending, `\r`
/// included, trimmed.
fn run_child(
    program: &Path,
    args: &[OsString],
    mut heard: impl FnMut(Heard),
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
        Out(Vec<u8>),
        Err(Vec<u8>),
        Closed,
    }
    /// A pipe of the child's, and how its lines are to be told apart from the other's.
    type Pipe = (Option<Box<dyn std::io::Read + Send>>, fn(Vec<u8>) -> Said);
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
                let _ = tell.send(wrap(std::mem::take(&mut bytes)));
            }
            let _ = tell.send(Said::Closed);
        });
    }
    drop(tell);
    let mut complaint = Vec::new();
    while open > 0 {
        match listen.recv() {
            Ok(Said::Out(raw)) => {
                let line = String::from_utf8_lossy(&raw).trim().to_string();
                heard(Heard::Out {
                    raw: &raw,
                    line: &line,
                });
            }
            Ok(Said::Err(raw)) => {
                heard(Heard::Err { raw: &raw });
                let line = String::from_utf8_lossy(&raw).trim().to_string();
                if !line.is_empty() {
                    complaint.push(line);
                }
            }
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
fn trace_verdict(ended: &Ended, save: &Path, folder: &Path) -> Result<(), Failure> {
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
            folder.display()
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
    let share = percent(total, PHOTO_SIZE.0 * PHOTO_SIZE.1, comma);
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
