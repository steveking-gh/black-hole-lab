//! The command line: what it takes, and every way it can be wrong, each as its own sentence.
//!
//! Hand-rolled, as `sky-trace`'s is: ten flags do not need a parsing crate, and a crate would put
//! its own wording on the refusals where this program wants to say what to do instead.
//!
//! The app always calls `sky-look <file.bhl> --observer bob|alice --open`; everything else is for
//! a person at a terminal. A command line this program cannot use exits with code 1 and not 2,
//! because the app's contract keeps 2 for a moment no view can be made from.

use std::ffi::OsString;
use std::path::PathBuf;

/// What `--help` prints.
pub const HELP: &str = "\
sky-look: makes a 360-degree photograph of the whole sky as one observer of a Black Hole Lab save
sees it at the saved moment. It runs sky-trace and then sky-render, and is what the app's Look
Around button starts. The observer's watch, radius and distant clock are written on a panel
inside the black hole's dark region, where they hide none of the sky, or below the opening view
when the dark region is too small to hold them.

USAGE
    sky-look <file.bhl> --observer bob|alice [options]

    <file.bhl>              The save. It is read and never changed.
    --observer bob|alice    Whose view. Required.
    --open                  Open the photograph when it is made: in the program --viewer names,
                            else in the one the environment variable BLACK_HOLE_LAB_VIEWER
                            names, else in VLC if it is installed, else in the program the
                            system opens .jpg files with, which may show the photograph flat
                            where VLC lets you drag to look round.
    --viewer <program>      The full path of the program --open starts on the photograph.
    --out-dir <dir>         Where the photograph goes. Default: the directory the environment
                            variable BLACK_HOLE_LAB_VIEWS names, else a directory named
                            \"Black Hole Lab views\" in the user's Videos folder, else in the
                            user's home directory.
    --grid <W>x<H>          The rays traced, in columns and rows; W must be twice H. Default
                            4096x2048. Fewer rays are quicker and widen the red rim round the
                            dark region, where the rays are too far apart to say where the light
                            came from.
    --exposure <stops>      The exposure, passed to sky-render. Default: sky-render's.
    --keep                  Keep the traced sky bundle, beside the photograph, and say where it
                            is.
    --tools <dir>           The directory holding sky-trace and sky-render. Default: this
                            program's own directory.
    --ffmpeg <path>         The ffmpeg sky-render writes the photograph with. Default: ffmpeg
                            on the PATH.
    --sky <map.exr>         The star map. Default: the file the environment variable
                            BLACK_HOLE_LAB_SKY_MAP names, else maps/starmap_2020_8k_gal.exr in
                            the nearest sky directory at or above this program's directory.
    --help, -h              Print this and do nothing else.

OUTPUT
    Progress, one sentence a line, on standard output; the last line of a run that succeeds is
    the photograph's full path. A refusal of the moment (the observer inside the inner horizon,
    being dragged, at the ring, not in the save) exits with code 2 and one sentence on standard
    error; any other failure exits with code 1 and one sentence on standard error.

    The photograph is named by the time it was made (UTC), the observer, and the reading of the
    observer's watch (proper time, in M) at the saved moment. Intermediate files are kept in a
    directory of this program's own under the system's temporary directory, and deleted at the
    end of every run; what a run that was stopped left there is deleted by the first run an hour
    or more later.
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

/// A checked command line.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub save: PathBuf,
    pub who: Who,
    pub open: bool,
    pub keep: bool,
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

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    Help,
    Look(Box<Options>),
}

/// The flags that take a value.
const VALUED: [&str; 8] = [
    "--observer",
    "--out-dir",
    "--grid",
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
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut save: Option<&str> = None;
    let (mut open, mut keep) = (false, false);
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        match flag {
            "--open" => open = true,
            "--keep" => keep = true,
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
        open,
        keep,
        out_dir: value("--out-dir").map(PathBuf::from),
        grid,
        exposure,
        tools: value("--tools").map(PathBuf::from),
        ffmpeg: value("--ffmpeg").map(PathBuf::from),
        sky: value("--sky").map(PathBuf::from),
        viewer: value("--viewer").map(PathBuf::from),
    })))
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
