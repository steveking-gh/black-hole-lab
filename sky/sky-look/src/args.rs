//! The command line: what it takes, and every way it can be wrong, each as its own sentence.
//!
//! Hand-rolled, as `sky-trace`'s is: a dozen flags do not need a parsing crate, and a crate would
//! put its own wording on the refusals where this program wants to say what to do instead.
//!
//! The app always calls `sky-look <file.bhl> --observer bob|alice --status <file> --move-save
//! --open`, with `--shell` after it when the user has ticked Show Rendering Terminal; everything
//! else is for a person at a terminal. A command line this program cannot use exits with code 1 and
//! not 2, because the app's contract keeps 2 for a moment no view can be made from.

use std::ffi::OsString;
use std::path::PathBuf;

/// What `--help` prints.
pub const HELP: &str = "\
sky-look: makes a 360-degree photograph of the whole sky as one observer of a Black Hole Lab save
sees it at the saved moment. It runs sky-trace and then sky-render, and is what the app's Look
Around button starts. The observer's read-outs are written on a panel inside the black hole's
dark region, where they hide none of the sky, or below the opening view when the dark region is
too small to hold them: the observer's watch, the coordinate time (the app's distant clock), the
proper distance from the outer event horizon (or, between the horizons, the proper time since the
observer crossed it), and the speed and heading of travel past each local reference observer there is where the observer is
(the static observer and the ZAMO, or the raindrop inside the outer horizon). Small hollow green signs on the sky, a ring, a
diamond or a triangle as the panel names them, mark those directions of travel.

USAGE
    sky-look <file.bhl> --observer bob|alice [options]
    sky-look --check [--tools <dir>] [--ffmpeg <path>] [--sky <map.exr>]
    sky-look --fetch-map [--status <file>]

    <file.bhl>              The save. It is copied into the view's folder and never changed,
                            unless --move-save asks for it to be moved there.
    --observer bob|alice    Whose view. Required.
    --open                  Open the photograph when it is made: in the program --viewer names,
                            else in the one the environment variable BLACK_HOLE_LAB_VIEWER
                            names, else in VLC if it is installed, else in the program the
                            system opens .jpg files with, which may show the photograph flat
                            where VLC lets you drag to look round.
    --viewer <program>      The full path of the program --open starts on the photograph.
    --out-dir <dir>         The views directory, in which each view gets a folder of its own
                            (see THE VIEW'S FOLDER). Default: the directory the environment
                            variable BLACK_HOLE_LAB_VIEWS names, else a directory named
                            \"Black Hole Lab views\" in the user's Videos folder, else in the
                            user's home directory.
    --grid <W>x<H>          The rays traced, in columns and rows; W must be twice H. Default
                            4096x2048. Fewer rays are quicker and widen the red rim round the
                            dark region, where the rays are too far apart to say where the light
                            came from.
    --units physical|geometric
                            The units of the read-outs, passed to sky-trace: physical, in
                            seconds and kilometres as Black Hole Lab's panel shows them, or
                            geometric, in M. Default physical.
    --exposure <stops>      The exposure, passed to sky-render. Default: sky-render's.
    --tools <dir>           The directory holding sky-trace and sky-render. Default: this
                            program's own directory.
    --ffmpeg <path>         The ffmpeg sky-render writes the photograph with. Default: ffmpeg
                            on the PATH.
    --sky <map.exr>         The star map. Default: the file the environment variable
                            BLACK_HOLE_LAB_SKY_MAP names, else maps/starmap_2020_8k_gal.exr
                            beside this program, else that file in the nearest sky directory
                            at or above this program's directory.
    --status <file>         Also append every progress sentence to this file, and at the end
                            one line saying how the run ended (see STATUS FILE). The app names
                            a file here and reads it while the view is being made.
    --move-save             Move the save into the view's folder rather than copy it. The app
                            gives this for the temporary save it writes for each view.
    --shell                 When the run ends, however it ends, start a command prompt in the
                            view's folder, in this program's console window, and leave the
                            prompt running there. The console window then shows only the
                            commands this program runs and what those programs print, even
                            where standard output is redirected; the sentences go to the status
                            file and log.txt alone. The app gives this when Show Rendering
                            Terminal is ticked, and starts this program in a console window of
                            its own.
    --check                 Make no view: look for everything a view needs - sky-trace and
                            sky-render, ffmpeg, the star map - and print what was found (see
                            CHECK). The app runs this when it starts, to say what is missing
                            before a view is asked for.
    --fetch-map             Make no view: download the default star map, 153 MB, one of NASA's
                            Deep Star Maps 2020 (https://svs.gsfc.nasa.gov/4851), with curl,
                            to where this program looks for it, and check its size and its
                            SHA-256 before putting it there. A map already there and whole is
                            left alone. The app's Download Star Map button runs this.
    --help, -h              Print this and do nothing else.

OUTPUT
    Progress, one sentence a line, on standard output, with the percentage traced and rendered as
    the two programs go; before each program is started, the command that starts it, as a
    PowerShell line beginning with &. The last line of a run that succeeds is the photograph's
    full path. A refusal of the moment (the observer inside the inner horizon, being dragged, at
    the ring, not in the save) exits with code 2 and one sentence on standard error; any other
    failure exits with code 1 and one sentence on standard error. With --shell, none of this
    program's own lines are printed: only each command, and then what sky-trace and sky-render
    print, on standard output and standard error as they printed it.

CHECK
    --check prints three lines on standard output, one for each of tools (sky-trace and
    sky-render), ffmpeg and map, in that order: the word, a space, and ok, or missing and one
    sentence saying what to do, or, for a default star map that --fetch-map would supply,
    fetchable and the sentence. When ffmpeg is missing and this program knows the command that
    installs ffmpeg on this system, a fourth line gives it: ffmpeg-install and the command. It
    exits with code 0 when all three are ok and with 1 otherwise.

FETCHING THE MAP
    --fetch-map prints its progress as sentences on standard output, with the percentage
    downloaded, and last the map's full path; a failure exits with code 1 and one sentence on
    standard error. With --status, the sentences go to that file too, and its last line is
    done <the map's full path> or failed <sentence>.

THE VIEW'S FOLDER
    Each view gets a folder of its own in the views directory, named by the observer and the
    reading of the observer's watch (proper time, in M, to three decimals) at the saved moment,
    such as Bob_12.345; a second view from the same reading gets Bob_12.345 (2), and so on. In
    the folder: the photograph, named as the folder is, with .jpg after the name, such as
    Bob_12.345.jpg; the save, with .bhl; the traced sky bundle, in bundle; commands.txt, the commands that
    were run, which PowerShell runs again when they are pasted into it; and log.txt, every line
    of progress, the first saying when the view was made (UTC), and the line saying how the run
    ended. Nothing in the folder is ever deleted,
    whether or not the view was made: a run that failed leaves its folder to be looked into. A
    bundle of the default 4096x2048 grid takes about 90 MB on disk.

STATUS FILE
    The file --status names gets the progress sentences, each on a line of its own, and then one
    last line saying how the run ended: done <the photograph's full path>, refused <sentence>,
    or failed <sentence>. A progress sentence always begins with a capital letter or a digit, so
    the last line is never taken for one. The lines are UTF-8, each ended by a line feed and each
    written whole; a reader that finds a line without its line feed has read it too soon.
";

/// The ray grid when `--grid` is not given. Measured 2026-09-27 on the owner's machine (16
/// threads), on demos/near_fall.bhl, Bob at the saved moment and at later moments of his fall:
///
/// | grid        | trace          | trace memory | render       | render memory | red rim at 8192 x 4096 |
/// | ----------- | -------------- | ------------ | ------------ | ------------- | ---------------------- |
/// | 1024 x 512  | 0.5 s          | 26 MB        | 3.8 to 4.0 s | 4.0 GB        | 4 to 13 px, median 12  |
/// | 2048 x 1024 | 1.8 to 2.8 s   | 131 MB       | 3.9 to 4.0 s | 4.0 GB        | 2 to 6 px, median 4    |
/// | 4096 x 2048 | 7.0 to 12 s    | 508 MB       | 4.1 to 4.3 s | 4.2 GB        | 2 to 3 px, median 2    |
///
/// The render columns are of the photograph alone (`--encoder none --still --photo`), measured
/// again on the evening of 2026-09-27 at the saved moment, three renders a grid, by the wall clock
/// and the peak working set; the render then also wrote a video held for a minute, which cost
/// 8.7 to 8.9 s. Most of the render's memory is the star map's rip-map and the blackbody model's
/// tables, which do not depend on the grid. The trace grows with the number of rays, and more with
/// the depth of the fall (the later moments cost 1.6 times the first). The finest grid is answered
/// in 11 to 16 s in all, inside the twenty seconds the owner allowed a press, and its rim is a
/// quarter of the coarsest grid's width.
pub const DEFAULT_GRID: (u32, u32) = (4096, 2048);

/// The most rays `sky-trace` will trace in one frame: 16384 x 8192.
const MAX_RAYS: u64 = 16384 * 8192;

/// The largest `--exposure` either way that `sky-render` accepts.
const MAX_EXPOSURE_STOPS: f64 = 100.0;

/// Which of a save's two observers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Who {
    Alice,
    Bob,
}

impl Who {
    /// The name the app gives the observer.
    pub fn name(self) -> &'static str {
        match self {
            Self::Alice => "Alice",
            Self::Bob => "Bob",
        }
    }

    /// The word `sky-trace --observer` takes, and the key of the observer in a save.
    pub fn flag(self) -> &'static str {
        match self {
            Self::Alice => "alice",
            Self::Bob => "bob",
        }
    }
}

/// The units the read-outs are written in: `sky-trace --units` takes the same two words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    /// Seconds and kilometres, on the app's ladder of units (µs to yr, km to ly), as the app's
    /// panel shows them.
    Physical,
    /// Multiples of M, the hole's mass in geometric units.
    Geometric,
}

impl Units {
    /// The word `sky-trace --units` takes.
    pub fn flag(self) -> &'static str {
        match self {
            Self::Physical => "physical",
            Self::Geometric => "geometric",
        }
    }
}

/// A checked command line.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub save: PathBuf,
    pub who: Who,
    /// `--units`, physical unless it says otherwise.
    pub units: Units,
    pub open: bool,
    /// `--status <file>`: where the progress sentences, and last the line saying how the run
    /// ended, are appended for the app to read.
    pub status: Option<PathBuf>,
    /// `--move-save`: move the save into the view's folder rather than copy it.
    pub move_save: bool,
    /// `--shell`: start a command prompt in the view's folder when the run ends.
    pub shell: bool,
    pub out_dir: Option<PathBuf>,
    /// Columns and rows of rays, W = 2 H.
    pub grid: (u32, u32),
    pub exposure: Option<f64>,
    pub tools: Option<PathBuf>,
    pub ffmpeg: Option<PathBuf>,
    pub sky: Option<PathBuf>,
    /// `--viewer <program>`: what `--open` starts on the photograph, ahead of every other choice.
    pub viewer: Option<PathBuf>,
}

/// `--check`, with the three places a person can name outright.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CheckOptions {
    pub tools: Option<PathBuf>,
    pub ffmpeg: Option<PathBuf>,
    pub sky: Option<PathBuf>,
}

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    Help,
    Look(Box<Options>),
    /// `--check`: say which pieces are there.
    Check(CheckOptions),
    /// `--fetch-map`: download the default star map, with `--status <file>` if one is named.
    FetchMap {
        status: Option<PathBuf>,
    },
}

/// The flags that take a value.
const VALUED: [&str; 10] = [
    "--observer",
    "--status",
    "--out-dir",
    "--grid",
    "--units",
    "--exposure",
    "--tools",
    "--ffmpeg",
    "--sky",
    "--viewer",
];

/// Reads the command line, without the program's own name.
pub fn parse(args: &[OsString]) -> Result<Request, String> {
    // The tracer and the renderer each take their arguments as Unicode and refuse anything else,
    // so a name that is not is refused here first, where the sentence can say what to do.
    let args: Vec<String> = args
        .iter()
        .map(|a| {
            a.clone().into_string().map_err(|a| {
                format!(
                    "The argument {} is not valid Unicode; give the file or directory another name.",
                    a.to_string_lossy()
                )
            })
        })
        .collect::<Result<_, _>>()?;
    if args.iter().any(|a| a == "--help" || a == "-h") {
        return Ok(Request::Help);
    }
    if args.iter().any(|a| a == "--check") {
        let given = alone("--check", &["--tools", "--ffmpeg", "--sky"], &args)?;
        let value = |flag: &str| given.iter().find(|(f, _)| *f == flag).map(|(_, v)| *v);
        return Ok(Request::Check(CheckOptions {
            tools: value("--tools").map(PathBuf::from),
            ffmpeg: value("--ffmpeg").map(PathBuf::from),
            sky: value("--sky").map(PathBuf::from),
        }));
    }
    if args.iter().any(|a| a == "--fetch-map") {
        let given = alone("--fetch-map", &["--status"], &args)?;
        return Ok(Request::FetchMap {
            status: given.first().map(|(_, v)| PathBuf::from(v)),
        });
    }
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut save: Option<&str> = None;
    let (mut open, mut move_save, mut shell) = (false, false, false);
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        match flag {
            "--open" => open = true,
            "--move-save" => move_save = true,
            "--shell" => shell = true,
            _ if VALUED.contains(&flag) => {
                let Some(value) = rest.next() else {
                    return Err(format!(
                        "{flag} needs a value after it; sky-look --help lists the options."
                    ));
                };
                if given.iter().any(|(f, _)| *f == flag) {
                    return Err(format!(
                        "{flag} is given twice; give it once, with the value you mean."
                    ));
                }
                given.push((flag, value.as_str()));
            }
            _ if flag.starts_with('-') && flag.len() > 1 => {
                return Err(format!(
                    "There is no option {flag}; sky-look --help lists the options."
                ));
            }
            _ => {
                if let Some(first) = save {
                    return Err(format!(
                        "Two saves are named, {first} and {flag}; a view is made from one save."
                    ));
                }
                save = Some(flag);
            }
        }
    }
    let value = |flag: &str| given.iter().find(|(f, _)| *f == flag).map(|(_, v)| *v);

    let save = save.ok_or("No save is named: sky-look <file.bhl> --observer bob|alice.")?;
    let who = match value("--observer") {
        Some("bob" | "Bob") => Who::Bob,
        Some("alice" | "Alice") => Who::Alice,
        Some(other) => {
            return Err(format!(
                "There is no observer {other:?}; a save has two, bob and alice."
            ));
        }
        None => return Err("--observer is required: bob or alice.".into()),
    };
    let grid = match value("--grid") {
        None => DEFAULT_GRID,
        Some(text) => parse_grid(text)?,
    };
    // Checked here rather than left to the tracer, whose refusal of its command line comes back
    // with the same exit code as its refusal of the moment.
    let units = match value("--units") {
        None | Some("physical" | "Physical") => Units::Physical,
        Some("geometric" | "Geometric") => Units::Geometric,
        Some(other) => {
            return Err(format!(
                "There are no units {other:?}; --units is physical, for seconds and kilometres, \
                 or geometric, for M."
            ));
        }
    };
    let exposure = match value("--exposure") {
        None => None,
        Some(text) => Some(
            text.trim()
                .parse::<f64>()
                .ok()
                .filter(|s| s.is_finite() && s.abs() <= MAX_EXPOSURE_STOPS)
                .ok_or_else(|| {
                    format!(
                        "--exposure is a number of stops, at most {MAX_EXPOSURE_STOPS} either way, \
                         not {text:?}."
                    )
                })?,
        ),
    };
    Ok(Request::Look(Box::new(Options {
        save: PathBuf::from(save),
        who,
        units,
        open,
        status: value("--status").map(PathBuf::from),
        move_save,
        shell,
        out_dir: value("--out-dir").map(PathBuf::from),
        grid,
        exposure,
        tools: value("--tools").map(PathBuf::from),
        ffmpeg: value("--ffmpeg").map(PathBuf::from),
        sky: value("--sky").map(PathBuf::from),
        viewer: value("--viewer").map(PathBuf::from),
    })))
}

/// The command line of `mode`, a flag that makes no view: the values of the flags in `valued`,
/// which are all it may be given with. Anything else is refused, because a `--check` or a
/// `--fetch-map` that quietly ignored a save or an observer would look as if it had used them.
fn alone<'a>(
    mode: &str,
    valued: &[&'a str],
    args: &'a [String],
) -> Result<Vec<(&'a str, &'a str)>, String> {
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        if arg == mode {
            continue;
        }
        let Some(flag) = valued.iter().find(|flag| *flag == arg) else {
            return Err(format!(
                "{mode} makes no view and takes only {}, not {arg}; sky-look --help lists the \
                 options.",
                if valued.is_empty() {
                    "no other option".to_string()
                } else {
                    valued.join(", ")
                }
            ));
        };
        let Some(value) = rest.next() else {
            return Err(format!(
                "{flag} needs a value after it; sky-look --help lists the options."
            ));
        };
        if given.iter().any(|(f, _)| f == flag) {
            return Err(format!(
                "{flag} is given twice; give it once, with the value you mean."
            ));
        }
        given.push((flag, value.as_str()));
    }
    Ok(given)
}

/// `<W>x<H>`, both at least 1, W twice H, and no more rays than the tracer takes: checked here so
/// that a grid the tracer would refuse is not passed on to come back as its refusal, which this
/// program would have to tell apart from a refusal of the moment.
fn parse_grid(text: &str) -> Result<(u32, u32), String> {
    let malformed = || {
        format!(
            "--grid is the columns and rows of rays as <W>x<H>, such as 4096x2048, not {text:?}."
        )
    };
    let (w, h) = text.split_once('x').ok_or_else(malformed)?;
    let w: u32 = w.parse().map_err(|_| malformed())?;
    let h: u32 = h.parse().map_err(|_| malformed())?;
    if h == 0 || u64::from(w) != 2 * u64::from(h) {
        return Err(format!(
            "--grid {text} is not twice as wide as it is high; the rays span 360 degrees across \
             and 180 down, so W must be 2H, such as 4096x2048."
        ));
    }
    if u64::from(w) * u64::from(h) > MAX_RAYS {
        return Err(format!(
            "--grid {text} has more rays than sky-trace traces, 16384x8192."
        ));
    }
    Ok((w, h))
}

/// The file `--status` names, read off a command line without the rest of it: for a command line
/// [`parse`] refuses, so that the refusal still reaches the app as the status file's last line.
pub fn status_named(args: &[OsString]) -> Option<PathBuf> {
    let at = args.iter().position(|a| a == "--status")?;
    args.get(at + 1).map(PathBuf::from)
}

/// Whether `--shell` is on a command line, read without the rest of it: `main` has to know before
/// anything is printed, to take this program's console window for its standard streams.
pub fn wants_shell(args: &[OsString]) -> bool {
    args.iter().any(|a| a == "--shell")
}
