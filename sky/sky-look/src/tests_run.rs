//! Whole runs, against stand-ins for `sky-trace` and `sky-render`.
//!
//! As the app's own tests of Look Around do (`src/look_around/tests.rs` at the repository root),
//! each stand-in is a script the test writes into a scratch directory of its own - a batch file on
//! Windows, a shell script elsewhere - which records the arguments it was given, does what the
//! real program would do to the disk (the tracer makes its bundle; the renderer checks that the
//! bundle is there and writes the photograph), and then does what the test tells it:
//! print a line, complain on standard error, exit with a chosen code. A script rather than a test
//! binary because `cmd.exe` and `sh` are on every machine these tests run on, and a binary would
//! have to be built for the purpose.
//!
//! The program runs in-process through `run::cli`, with the arguments, the environment and both
//! output streams handed in, so that a test reads exactly what it wrote where - standard output,
//! standard error, the status file and the view's folder - and the exit code it chose. The views
//! directory is the test's own, so that what a run leaves in it is all there is to look at.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use crate::find::Names;
use crate::run::{BUNDLE_DIR, COMMANDS_FILE, Environment, LOG_FILE, cli};
use crate::tests::scratch_dir;

/// One thing a stand-in does after its work on the disk.
enum Step<'a> {
    /// Print a line on standard output.
    Say(&'a str),
    /// Print a line on standard error.
    Complain(&'a str),
    /// Exit with this code.
    Exit(i32),
}

/// A line as a script prints it: `cmd` reads `%` as the start of a variable, so it is doubled.
/// The lines the tests print hold nothing else either shell reads as syntax.
fn quoted(line: &str) -> String {
    if cfg!(windows) {
        line.replace('%', "%%")
    } else {
        format!("'{line}'")
    }
}

/// Writes a stand-in named `name` into `dir`: it records its arguments in `<name>.args`, does
/// `work` (lines of the script, already in its shell's language), then `steps`.
fn stand_in(dir: &Path, name: &str, work: &[&str], steps: &[Step]) {
    let args = dir.join(format!("{name}.args"));
    let mut script = String::new();
    let end = if cfg!(windows) { "\r\n" } else { "\n" };
    if cfg!(windows) {
        script.push_str("@echo off\r\n");
        script.push_str(&format!(">\"{}\" echo %*\r\n", args.display()));
    } else {
        script.push_str("#!/bin/sh\n");
        script.push_str(&format!("echo \"$@\" > '{}'\n", args.display()));
    }
    for line in work {
        script.push_str(line);
        script.push_str(end);
    }
    for step in steps {
        let line = match step {
            Step::Say(line) => format!("echo {}", quoted(line)),
            Step::Complain(line) if cfg!(windows) => format!(">&2 echo {}", quoted(line)),
            Step::Complain(line) => format!("echo {} >&2", quoted(line)),
            Step::Exit(code) => format!("exit {code}"),
        };
        script.push_str(&line);
        script.push_str(end);
    }
    let file = dir.join(program(name));
    std::fs::write(&file, script).expect("the stand-in");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&file, std::fs::Permissions::from_mode(0o755))
            .expect("the stand-in is executable");
    }
}

/// A stand-in's file name.
fn program(name: &str) -> String {
    if cfg!(windows) {
        format!("{name}.cmd")
    } else {
        name.to_string()
    }
}

/// What the tracer stand-in does to the disk: its third argument is the bundle directory, which it
/// makes, with a manifest in it.
fn tracer_work() -> Vec<&'static str> {
    if cfg!(windows) {
        vec!["mkdir \"%~3\"", ">\"%~3\\manifest.json\" echo {}"]
    } else {
        vec!["mkdir -p \"$3\"", "echo '{}' > \"$3/manifest.json\""]
    }
}

/// What the renderer stand-in does to the disk: refuses if the bundle (its fourth argument) is
/// not there, and writes the photograph (its second).
fn renderer_work() -> Vec<&'static str> {
    if cfg!(windows) {
        vec![
            "if not exist \"%~4\\manifest.json\" exit 9",
            ">\"%~2\" echo a photograph",
        ]
    } else {
        vec![
            "[ -f \"$4/manifest.json\" ] || exit 9",
            "echo 'a photograph' > \"$2\"",
        ]
    }
}

/// What a real renderer says on a run that works, near enough: two marks of directions of travel
/// among the rest, one before the line on the read-outs and one after the counts, to show that
/// the order of the sentences is this program's and not the renderer's.
const RENDERER_SAYS: [&str; 11] = [
    "progress 15%",
    "progress 65%",
    "mark: a ring in green at 37.25 degrees right of the opening view and 0 degrees up: Direction of travel past the static observer",
    "read-outs: 6 line(s) on a panel inside the dark region, centred 180 degrees right of the opening view and -20 degrees up, each line 2 degrees high",
    "map C:\\maps\\starmap_2020_8k_gal.exr: 8192 x 4096 galactic, read in 0.6 s",
    "frame 0 (1/1), 1.00 frames/s, 0:00 left",
    "progress 100%",
    "wrote view.jpg (7.3 MB), a JPEG marked as a 360-degree photograph, in 0.4 s",
    "pixels drawn over the 1 frame(s): unresolved 0 (0 %), under-sampled 16888 (0.0503 %), dark 20422816 (60.9 %)",
    "mark: a diamond in green at -141.6 degrees right of the opening view and 8.4 degrees up: Direction of travel past the ZAMO",
    "A published video made from NASA's star maps must carry this credit:",
];

/// What this program says of the two marks in [`RENDERER_SAYS`], in order.
const MARKS_SAID: [&str; 2] = [
    "A green ring marks the direction of travel past the static observer, 37 degrees right of the \
     opening view.",
    "A green diamond marks the direction of travel past the ZAMO, 142 degrees left of the opening \
     view and 8 degrees up.",
];

/// The tracer's sentence for an observer inside the inner horizon, as it prints it.
const REFUSAL: &str = "Bob is at r = 0.3 M, at or inside the inner horizon r- = 0.43 M; an observer \
                       there sees light that came through the ring and from the other sheet of r-, \
                       which this program cannot yet trace";

/// A layout for one run: the tools, a map and an ffmpeg to be found, a save, a views directory,
/// and the status file a run is told to write when a test gives it `--status`.
struct Case {
    root: PathBuf,
    tools: PathBuf,
    save: PathBuf,
    out: PathBuf,
    status: PathBuf,
    map: PathBuf,
    ffmpeg: PathBuf,
}

impl Case {
    fn new(what: &str, save: &str) -> Self {
        let root = scratch_dir(what);
        let case = Case {
            tools: root.join("tools"),
            save: root.join("the run.bhl"),
            out: root.join("views"),
            status: root.join("the run.status"),
            map: root.join("map.exr"),
            ffmpeg: root.join("ffmpeg.exe"),
            root,
        };
        std::fs::create_dir_all(&case.tools).expect("a directory");
        std::fs::write(&case.save, save).expect("the save");
        std::fs::write(&case.map, b"").expect("a map");
        std::fs::write(&case.ffmpeg, b"").expect("an ffmpeg");
        case
    }

    fn tracer(&self, steps: &[Step]) -> &Self {
        stand_in(&self.tools, "sky-trace", &tracer_work(), steps);
        self
    }

    fn renderer(&self, steps: &[Step]) -> &Self {
        stand_in(&self.tools, "sky-render", &renderer_work(), steps);
        self
    }

    fn env(&self) -> Environment {
        Environment {
            exe_dir: Some(self.tools.clone()),
            path: None,
            sky_env: None,
            views_env: None,
            viewer_env: None,
            vlc: Vec::new(),
            videos: None,
            home: None,
            names: Names {
                trace: program("sky-trace"),
                render: program("sky-render"),
                ffmpeg: "ffmpeg.exe".into(),
            },
        }
    }

    /// Runs the program as the app would, with the map, ffmpeg and views directory named, and
    /// `extra` after; returns the exit code and what it wrote to each stream.
    fn run(&self, extra: &[&str]) -> (i32, String, String) {
        let mut args: Vec<OsString> = vec![
            self.save.clone().into(),
            "--observer".into(),
            "bob".into(),
            "--out-dir".into(),
            self.out.clone().into(),
            "--sky".into(),
            self.map.clone().into(),
            "--ffmpeg".into(),
            self.ffmpeg.clone().into(),
        ];
        args.extend(extra.iter().map(OsString::from));
        let (mut out, mut err) = (Vec::new(), Vec::new());
        let code = cli(&args, &self.env(), &mut out, &mut err);
        (
            code,
            String::from_utf8(out).expect("standard output is UTF-8"),
            String::from_utf8(err).expect("standard error is UTF-8"),
        )
    }

    /// What a stand-in was given, or `None` if it never ran.
    fn args_of(&self, name: &str) -> Option<String> {
        std::fs::read_to_string(self.tools.join(format!("{name}.args"))).ok()
    }

    /// Whatever is in the views directory.
    fn views(&self) -> Vec<PathBuf> {
        match std::fs::read_dir(&self.out) {
            Ok(listing) => listing.map(|e| e.expect("an entry").path()).collect(),
            Err(_) => Vec::new(),
        }
    }

    /// The one folder the run made in the views directory.
    fn folder(&self) -> PathBuf {
        let views = self.views();
        assert_eq!(views.len(), 1, "one folder for the one view: {views:?}");
        assert!(views[0].is_dir(), "{views:?}");
        views[0].clone()
    }

    /// The names in `dir`, sorted.
    fn listing(dir: &Path) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .expect("a directory")
            .map(|e| {
                e.expect("an entry")
                    .file_name()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect();
        names.sort();
        names
    }

    /// The status file's lines.
    fn status_lines(&self) -> Vec<String> {
        let text = std::fs::read_to_string(&self.status).expect("the status file");
        assert!(
            text.is_empty() || text.ends_with('\n'),
            "every line ends in a line feed: {text:?}"
        );
        text.lines().map(str::to_string).collect()
    }
}

impl Drop for Case {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}

const SAVE: &str = r#"{"format":"black-hole-lab-save","version":1,"sim":{"bob":{"name":"Bob","tau":1.5}},"controls":{"decimal_is_comma":false}}"#;

/// The save's bytes and modification time, to show it was not touched.
fn fingerprint(path: &Path) -> (Vec<u8>, SystemTime) {
    let bytes = std::fs::read(path).expect("the save");
    let modified = std::fs::metadata(path)
        .and_then(|m| m.modified())
        .expect("its time");
    (bytes, modified)
}

/// The progress lines of standard output: everything but the commands, which begin with `& `,
/// and, on a run that succeeds, the path on the last line.
fn progress_of(out: &str) -> Vec<&str> {
    out.lines().filter(|l| !l.starts_with("& ")).collect()
}

/// Every progress line is a complete sentence: it starts with a capital or a digit and ends with
/// a full stop, and none of it is a line a child printed.
fn assert_sentences(lines: &[&str]) {
    for line in lines {
        let first = line.chars().next().expect("no empty lines");
        assert!(
            first.is_uppercase() || first.is_ascii_digit(),
            "not a sentence: {line:?}"
        );
        assert!(line.ends_with('.'), "not a complete sentence: {line:?}");
        for child in RENDERER_SAYS {
            assert!(!line.contains(child), "a child's line leaked: {line:?}");
        }
    }
}

/// A tracer that says how far it has got, as the real one does, and succeeds.
fn tracing_steps() -> Vec<Step<'static>> {
    vec![
        Step::Say("progress 1%"),
        Step::Say("progress 50%"),
        Step::Say("progress 52%"),
        Step::Say("frame 1 of 1: 0.14 frames a second, about 0 s left, 0 unresolved rays so far"),
        Step::Say("progress 100%"),
        // A child's standard error on a run that works is not passed on.
        Step::Complain("a warning the tracer printed"),
        Step::Exit(0),
    ]
}

/// A renderer that says what the real one says, and succeeds.
fn rendering_steps() -> Vec<Step<'static>> {
    let mut steps: Vec<Step> = RENDERER_SAYS.iter().map(|l| Step::Say(l)).collect();
    steps.push(Step::Exit(0));
    steps
}

#[test]
fn test_a_view_that_is_made_is_a_folder_holding_everything_and_ends_with_the_photographs_path() {
    let case = Case::new("made", SAVE);
    case.tracer(&tracing_steps());
    case.renderer(&rendering_steps());
    let before = fingerprint(&case.save);

    let (code, out, err) = case.run(&[]);
    assert_eq!(code, 0, "out: {out}\nerr: {err}");
    assert_eq!(err, "", "nothing on standard error");
    let lines = progress_of(&out);
    let (last, progress) = lines.split_last().expect("some output");
    assert_sentences(progress);
    let folder = case.folder();
    assert_eq!(
        progress[0],
        format!("Making Bob's view in {}.", folder.display()),
        "{out}"
    );
    assert!(
        progress[1].starts_with("Tracing the light that reaches Bob from every direction"),
        "{out}"
    );
    // The children's percentages, as sentences, five points apart at least and never 100.
    for said in [
        "Tracing the light that reaches Bob: 50%.",
        "Rendering Bob's view: 15%.",
        "Rendering Bob's view: 65%.",
    ] {
        assert!(progress.contains(&said), "{said}: {out}");
    }
    for unsaid in ["52%", "100%", ": 1%."] {
        assert!(!out.contains(unsaid), "{unsaid}: {out}");
    }
    assert!(!out.contains("this takes about"), "no estimates: {out}");
    assert!(
        progress
            .iter()
            .any(|l| l.starts_with("16888 pixels (0.0503 % of the picture)")
                && l.contains("drawn in red along the edge of the dark region")),
        "the red is explained: {out}"
    );

    // Where the read-outs went, then where each mark is, then the red, in sentences of this
    // program's own and one after another.
    let at = |sentence: &str| {
        progress
            .iter()
            .position(|l| *l == sentence)
            .unwrap_or_else(|| panic!("{sentence:?} is not said: {out}"))
    };
    let readouts = at(
        "The read-outs are written inside the dark region of the hole, 180 degrees \
                       right of the opening view and 20 degrees down.",
    );
    let red = progress
        .iter()
        .position(|l| l.starts_with("16888 pixels"))
        .expect("the red is explained");
    assert_eq!(
        [at(MARKS_SAID[0]), at(MARKS_SAID[1]), red],
        [readouts + 1, readouts + 2, readouts + 3],
        "{out}"
    );
    assert!(
        progress.iter().any(
            |l| l.starts_with("Made Bob's view in ") && l.ends_with("a 360-degree photograph.")
        ),
        "{out}"
    );
    assert!(!out.contains("video"), "nothing is said of a video: {out}");

    // The last line is the path, whole, of the photograph, which is in the view's folder and
    // named as the folder is.
    let photo = PathBuf::from(last);
    assert!(photo.is_absolute() && photo.is_file(), "{last}");
    assert_eq!(photo.parent(), Some(folder.as_path()));
    assert!(folder.parent() == Some(case.out.as_path()));
    let name = folder
        .file_name()
        .expect("a name")
        .to_string_lossy()
        .into_owned();
    assert!(name.ends_with(" UTC Bob at tau 1.500 M"), "{name}");
    assert_eq!(
        std::fs::read_to_string(&photo)
            .expect("the photograph")
            .trim(),
        "a photograph",
        "the file the renderer wrote"
    );

    // Everything of the view, and nothing else, is in the folder: the photograph, a copy of the
    // save (the save itself untouched), the bundle, the commands and the log.
    assert_eq!(
        Case::listing(&folder),
        [
            format!("{name}.bhl"),
            format!("{name}.jpg"),
            BUNDLE_DIR.to_string(),
            COMMANDS_FILE.to_string(),
            LOG_FILE.to_string(),
        ]
    );
    assert!(folder.join(BUNDLE_DIR).join("manifest.json").is_file());
    assert_eq!(
        std::fs::read(folder.join(format!("{name}.bhl"))).expect("the copy"),
        before.0,
        "a copy of the save"
    );
    assert_eq!(fingerprint(&case.save), before, "the save is untouched");

    // The two commands, on standard output before each program ran and in commands.txt after a
    // header, as PowerShell lines naming the files in the folder.
    let commands: Vec<&str> = out.lines().filter(|l| l.starts_with("& ")).collect();
    assert_eq!(commands.len(), 2, "{out}");
    assert!(commands[0].contains("sky-trace") && commands[1].contains("sky-render"));
    let quoted = |path: &Path| format!("'{}'", path.display());
    assert!(
        commands[0].contains(&quoted(&folder.join(format!("{name}.bhl"))))
            && commands[0].contains(&quoted(&folder.join(BUNDLE_DIR))),
        "{}",
        commands[0]
    );
    assert!(commands[1].contains(&quoted(&photo)), "{}", commands[1]);
    let recorded = std::fs::read_to_string(folder.join(COMMANDS_FILE)).expect("the commands");
    let (header, lines): (Vec<&str>, Vec<&str>) =
        recorded.lines().partition(|l| l.starts_with("# "));
    assert!(
        !header.is_empty() && recorded.starts_with("# "),
        "{recorded}"
    );
    assert_eq!(lines, commands);

    // What the two programs were asked for.
    let traced = case.args_of("sky-trace").expect("the tracer ran");
    for word in [
        "--observer bob",
        "--frames 1",
        "--grid 4096x2048",
        "--units physical",
    ] {
        assert!(traced.contains(word), "{traced}");
    }
    let rendered = case.args_of("sky-render").expect("the renderer ran");
    assert!(rendered.starts_with("--photo "), "{rendered}");
    for word in [
        ".jpg",
        "--encoder none",
        "--still",
        "--readouts panel",
        "--readout-at dark",
        "--size 8192x4096",
    ] {
        assert!(rendered.contains(word), "{rendered}");
    }
    for word in ["--decimal-comma", "--hold", "--out ", "overlay"] {
        assert!(!rendered.contains(word), "{word}: {rendered}");
    }
}

#[test]
fn test_the_status_file_gets_every_sentence_and_last_the_verdict_and_the_log_is_the_same() {
    let case = Case::new("status", SAVE);
    case.tracer(&tracing_steps());
    case.renderer(&rendering_steps());
    let status = case.status.display().to_string();
    let (code, out, err) = case.run(&["--status", &status]);
    assert_eq!((code, err.as_str()), (0, ""), "{out}");

    let lines = case.status_lines();
    let (verdict, said) = lines.split_last().expect("a verdict");
    let printed = progress_of(&out);
    let (path, progress) = printed.split_last().expect("some output");
    assert_eq!(
        verdict,
        &format!("done {path}"),
        "the verdict names the photograph"
    );
    assert_eq!(
        said, progress,
        "the same sentences as standard output, in order"
    );
    // A verdict is never a sentence, and a sentence never a verdict.
    for line in said {
        assert!(
            !["done ", "refused ", "failed "]
                .iter()
                .any(|w| line.starts_with(w)),
            "{line}"
        );
    }
    let log = std::fs::read_to_string(case.folder().join(LOG_FILE)).expect("the log");
    assert_eq!(
        log,
        std::fs::read_to_string(&case.status).expect("the status file"),
        "the log is the status file, kept"
    );
}

#[test]
fn test_move_save_moves_the_save_into_the_views_folder() {
    let case = Case::new("move", SAVE);
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&[Step::Exit(0)]);
    let (code, out, err) = case.run(&["--move-save"]);
    assert_eq!((code, err.as_str()), (0, ""), "{out}");
    let folder = case.folder();
    let name = folder.file_name().expect("a name").to_string_lossy();
    assert!(!case.save.exists(), "the save is no longer where it was");
    assert_eq!(
        std::fs::read_to_string(folder.join(format!("{name}.bhl"))).expect("the save, moved"),
        SAVE
    );
    assert!(
        case.args_of("sky-trace")
            .expect("the tracer ran")
            .contains(&folder.display().to_string()),
        "the tracer read the save in the folder"
    );
}

#[test]
fn test_the_tracers_refusal_comes_through_as_code_2_with_its_sentence_unchanged() {
    let case = Case::new("refused", SAVE);
    let complaint = format!("sky-trace: {REFUSAL}");
    case.tracer(&[
        Step::Say("the dry run's summary"),
        Step::Complain(&complaint),
        Step::Exit(2),
    ]);
    case.renderer(&[Step::Exit(0)]);
    let before = fingerprint(&case.save);

    let status = case.status.display().to_string();
    let (code, out, err) = case.run(&["--status", &status]);
    assert_eq!(code, 2, "out: {out}\nerr: {err}");
    assert_eq!(
        err,
        format!("{REFUSAL}\n"),
        "the tracer's sentence, and nothing else"
    );
    assert_sentences(&progress_of(&out));
    assert!(
        !out.contains("the dry run's summary"),
        "the tracer's output is not passed on: {out}"
    );
    assert_eq!(case.args_of("sky-render"), None, "nothing is rendered");
    assert_eq!(
        case.status_lines().last(),
        Some(&format!("refused {REFUSAL}")),
        "the verdict"
    );
    // The folder stays, with what the run had made - the save, the command and the log, which
    // says that it is kept - and no photograph. (The stand-in tracer makes its bundle before it
    // refuses, which the real one does not, so the bundle is neither here nor there.)
    let folder = case.folder();
    let name = folder
        .file_name()
        .expect("a name")
        .to_string_lossy()
        .into_owned();
    let listing = Case::listing(&folder);
    for kept in [format!("{name}.bhl"), COMMANDS_FILE.into(), LOG_FILE.into()] {
        assert!(listing.contains(&kept), "{kept}: {listing:?}");
    }
    assert!(!listing.iter().any(|f| f.ends_with(".jpg")), "{listing:?}");
    let log = std::fs::read_to_string(folder.join(LOG_FILE)).expect("the log");
    assert!(
        log.contains(&format!(
            "The files of this run are kept in {}.",
            folder.display()
        )),
        "{log}"
    );
    assert_eq!(fingerprint(&case.save), before, "the save is untouched");
}

#[test]
fn test_a_save_the_tracer_cannot_read_is_a_failure_and_not_a_refusal_of_the_moment() {
    let case = Case::new("unreadable", SAVE);
    let work = if cfg!(windows) {
        ">&2 echo sky-trace: %~1: the file is not readable as a save: expected value at line 1\r\nexit 2"
    } else {
        "echo \"sky-trace: $1: the file is not readable as a save: expected value at line 1\" >&2\nexit 2"
    };
    stand_in(&case.tools, "sky-trace", &[work], &[]);
    case.renderer(&[Step::Exit(0)]);
    let (code, _, err) = case.run(&[]);
    assert_eq!(code, 1, "{err}");
    assert!(
        err.starts_with("sky-trace could not read the save: ")
            && err.contains("not readable as a save"),
        "{err}"
    );
    assert_eq!(err.lines().count(), 1, "{err}");
}

#[test]
fn test_a_failure_of_the_renderer_comes_through_as_code_1_and_leaves_the_folder_to_look_into() {
    let case = Case::new("render-fails", SAVE);
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&[
        Step::Say("read-outs: 3 line(s)"),
        Step::Complain(
            "sky-render: ffmpeg stopped while writing the photograph: There is not enough space \
             on the disk",
        ),
        Step::Exit(1),
    ]);
    let before = fingerprint(&case.save);
    let status = case.status.display().to_string();
    let (code, out, err) = case.run(&["--status", &status]);
    assert_eq!(code, 1, "out: {out}\nerr: {err}");
    let sentence = "sky-render could not make Bob's view: ffmpeg stopped while writing the \
                    photograph: There is not enough space on the disk.";
    assert_eq!(err, format!("{sentence}\n"));
    assert_sentences(&progress_of(&out));
    assert_eq!(
        case.status_lines().last(),
        Some(&format!("failed {sentence}"))
    );
    // Nothing of the run is deleted: the save, the bundle, the commands and the log, and whatever
    // the renderer left.
    let folder = case.folder();
    let name = folder
        .file_name()
        .expect("a name")
        .to_string_lossy()
        .into_owned();
    let listing = Case::listing(&folder);
    for kept in [
        format!("{name}.bhl"),
        BUNDLE_DIR.into(),
        COMMANDS_FILE.into(),
        LOG_FILE.into(),
    ] {
        assert!(listing.contains(&kept), "{kept}: {listing:?}");
    }
    assert!(folder.join(BUNDLE_DIR).join("manifest.json").is_file());
    assert_eq!(fingerprint(&case.save), before, "the save is untouched");
}

#[test]
fn test_a_missing_piece_stops_the_run_before_anything_is_started_or_written() {
    let case = Case::new("missing", SAVE);
    case.tracer(&[Step::Exit(0)]);
    // No renderer.
    let status = case.status.display().to_string();
    let (code, out, err) = case.run(&["--status", &status, "--move-save"]);
    assert_eq!(code, 1);
    assert_eq!(out, "", "nothing was started, so there is no progress");
    assert!(
        err.contains("sky-render") && err.contains("was not found") && err.ends_with(".\n"),
        "{err}"
    );
    assert_eq!(err.lines().count(), 1);
    assert_eq!(
        case.args_of("sky-trace"),
        None,
        "the tracer was not started"
    );
    assert!(!case.out.exists(), "no folder was made");
    assert!(
        case.save.is_file(),
        "the save is where it was, to be deleted by its owner"
    );
    assert_eq!(
        case.status_lines(),
        [format!("failed {}", err.trim_end())],
        "the verdict and nothing else"
    );
}

#[test]
fn test_a_command_line_refused_still_ends_the_status_file_with_the_reason() {
    let case = Case::new("refused-line", SAVE);
    let status = case.status.display().to_string();
    let (code, out, err) = case.run(&["--status", &status, "--grid", "100x100"]);
    assert_eq!(code, 1, "{err}");
    assert_eq!(out, "");
    assert_eq!(case.status_lines(), [format!("failed {}", err.trim_end())]);
}

#[test]
fn test_units_reach_the_tracer_and_units_it_does_not_take_are_refused_before_it_starts() {
    let case = Case::new("units", SAVE);
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&[Step::Exit(0)]);
    let (code, out, err) = case.run(&["--units", "geometric"]);
    assert_eq!((code, err.as_str()), (0, ""), "{out}");
    let traced = case.args_of("sky-trace").expect("the tracer ran");
    assert!(
        traced.contains("--units geometric") && !traced.contains("physical"),
        "{traced}"
    );
    assert!(
        !case
            .args_of("sky-render")
            .expect("the renderer ran")
            .contains("--units"),
        "the renderer paints what the tracer wrote"
    );

    let refused = Case::new("units-refused", SAVE);
    refused.tracer(&[Step::Exit(0)]);
    refused.renderer(&[Step::Exit(0)]);
    let (code, out, err) = refused.run(&["--units", "imperial"]);
    assert_eq!(code, 1, "a command line, not a moment: {err}");
    assert_eq!(out, "", "nothing was started, so there is no progress");
    assert!(
        err.starts_with("There are no units \"imperial\"; --units is physical")
            && err.ends_with(".\n"),
        "{err}"
    );
    assert_eq!(
        refused.args_of("sky-trace"),
        None,
        "the tracer was not started"
    );
}

#[test]
fn test_a_save_set_to_the_decimal_comma_gets_comma_read_outs_and_names() {
    let case = Case::new(
        "comma",
        &SAVE.replace("\"decimal_is_comma\":false", "\"decimal_is_comma\":true"),
    );
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&rendering_steps());
    let (code, out, err) = case.run(&[]);
    assert_eq!((code, err.as_str()), (0, ""), "{out}");
    assert!(
        case.args_of("sky-render")
            .expect("rendered")
            .contains("--decimal-comma")
    );
    assert!(
        out.lines()
            .last()
            .expect("a path")
            .ends_with("Bob at tau 1,500 M.jpg"),
        "{out}"
    );
    assert!(out.contains("(0,0503 % of the picture)"), "{out}");
    // The marks' angles are whole degrees, which no decimal mark touches.
    for said in MARKS_SAID {
        assert!(out.contains(said), "{said}: {out}");
    }
}

#[test]
fn test_a_viewer_named_wrongly_fails_the_opening_and_says_where_the_view_that_was_made_is() {
    let case = Case::new("no-viewer", SAVE);
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&[Step::Exit(0)]);
    let nowhere = case.root.join("nowhere").join(program("viewer"));
    let nowhere_text = nowhere.display().to_string();
    let (code, out, err) = case.run(&["--open", "--viewer", &nowhere_text]);
    assert_eq!(code, 1, "out: {out}\nerr: {err}");
    assert_eq!(err.lines().count(), 1, "{err}");
    let folder = case.folder();
    let name = folder.file_name().expect("a name").to_string_lossy();
    let photo = folder.join(format!("{name}.jpg"));
    assert!(photo.is_file(), "the photograph was made");
    assert!(
        err.starts_with(&format!(
            "Made Bob's view at {}, but --viewer names ",
            photo.display()
        )) && err.contains(&nowhere_text)
            && err.contains("no program there")
            && err.trim_end().ends_with('.'),
        "{err}"
    );
    assert_sentences(&progress_of(&out));
    assert!(!out.contains("Opened"), "nothing was opened: {out}");
}

#[test]
fn test_open_starts_the_named_viewer_on_the_photograph_and_names_the_program() {
    // The viewer is a stand-in that records what it was given. Named `vlc`, so that it is also
    // given the option that keeps VLC's picture up. It is not waited for, so the test waits for
    // what it records.
    let case = Case::new("viewer", SAVE);
    case.tracer(&[Step::Exit(0)]);
    case.renderer(&[Step::Exit(0)]);
    stand_in(&case.tools, "vlc", &[], &[Step::Exit(0)]);
    let viewer = case.tools.join(program("vlc"));
    let viewer_text = viewer.display().to_string();
    let (code, out, err) = case.run(&["--open", "--viewer", &viewer_text]);
    assert_eq!((code, err.as_str()), (0, ""), "{out}");
    let lines = progress_of(&out);
    let (last, progress) = lines.split_last().expect("some output");
    assert_sentences(progress);
    assert_eq!(
        progress.last().copied(),
        Some(
            format!(
                "Opened the photograph in {viewer_text}, the viewer --viewer names; drag with the \
                 mouse to look in every direction."
            )
            .as_str()
        ),
        "{out}"
    );
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    let given = loop {
        if let Some(given) = case.args_of("vlc").filter(|g| g.ends_with('\n')) {
            break given;
        }
        assert!(std::time::Instant::now() < deadline, "the viewer never ran");
        std::thread::sleep(std::time::Duration::from_millis(10));
    };
    // The option, then the photograph's path as one argument. The standard library quotes every
    // argument it hands a batch file, which is what the stand-in is on Windows; a program such as
    // VLC reads the quotes away.
    let expected = if cfg!(windows) {
        format!("\"--image-duration=-1\" \"{last}\"")
    } else {
        format!("--image-duration=-1 {last}")
    };
    assert_eq!(given.trim(), expected);
}
