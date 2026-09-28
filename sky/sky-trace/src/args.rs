//! The command line: what it takes, and every way it can be wrong, each as its own sentence.
//!
//! Hand-rolled, like `sky-testgen`'s: a dozen flags do not need a parsing crate, and a crate would
//! put its own wording on the refusals where this program wants to say what to do instead.
//!
//! Two command lines share the program. `--info` makes a dry run, read exactly as it was before
//! tracing existed, so that its tests and its users see no change; anything else is a film.

use std::path::PathBuf;

use sky_trace::units::Units;

/// How many frames a film is allowed when `--frames` is not given. A worldline that ends - at the
/// ring, on the inner horizon - ends well inside this at any sensible step; one that does not (a
/// static observer, an orbit) needs a length. The dry run gives it this one; a tracing run, which
/// would spend hours on it, refuses and asks for the length instead.
pub const DEFAULT_MAX_FRAMES: usize = 10_000;

/// The default frame step, in M of the observer's proper time.
pub const DEFAULT_STEP: f64 = 0.1;

/// What `--help` prints.
pub const HELP: &str = "\
sky-trace: follows an observer of a Black Hole Lab save forward from the saved moment, at equal
steps of the observer's own proper time, traces what that observer sees of the whole sky at each
step, and writes it as a sky bundle for sky-render to make into a 360-degree video.

USAGE
    sky-trace <file.bhl> --out <dir> [options]
    sky-trace --hover <r> --spin <a> (--frames <n> | --seconds <s>) --out <dir> [options]
    sky-trace --info <file.bhl> [--observer bob|alice] [--step <tau>] [--frames <n>]

FILMING A SAVE
    <file.bhl>              The save to film: a .bhl written by Black Hole Lab, compressed or not.
    --out <dir>             The bundle directory. Required. It is created if it is not there.
    --observer bob|alice    Whose film. Default bob.
    --grid <W>x<H>          The observer-sky grid, in columns and rows; W must be twice H.
                            Default 1024x512.
    --fps <n>               Frames per second of the video. Default 30. Each frame of the bundle
                            is one frame of the video.
    --rate <M>              The observer's proper time shown per second of video, in M. Default
                            0.25. Frame k is at the saved proper time plus k * rate / fps, exactly.
    --frames <n>            The film's length in frames, frame 0 being the saved moment.
    --seconds <s>           Or its length in seconds of video: round(s * fps) frames.
                            Default for both: to the end of the observer's worldline.
    --threads <n>           Threads that trace the rows of a frame. Default: all of them.
    --units physical|geometric
                            How the renderer shows times and radii. physical, the default,
                            declares each clock and the radius with a unit a person reads -
                            seconds up to years, metres up to light-years - chosen once for
                            the whole film; geometric shows them in M alone. The values stored
                            are in M either way.
    --resume                Continue a bundle a previous run left at --out, tracing only the
                            frames it lacks. The settings must be the ones it was started with.
                            With no bundle there yet, a new one is started. Without --resume, an
                            existing bundle at --out is refused rather than written over.
    --help, -h              Print this and do nothing else.

A HOVER TEST
    --hover <r>             Instead of a save: a static observer at chart radius r, in M, on the
                            equatorial plane at azimuth 0, held there for good. It sees a shadow
                            whose size and shift are known in closed form, which is what it is
                            for. r must be outside the static limit, 2 M.
    --spin <a>              The hole's spin, in M: at least 0 and less than 1. Required with
                            --hover. The hole's mass M is 1.
    --solar-masses <m>      The hole's mass in solar masses, which sets how long 1 M lasts.
                            Default 4.15e6, Sagittarius A*.
    The hover never ends, so --frames or --seconds is required with it.

A DRY RUN
    --info <file.bhl>       Read the save, walk the observer's worldline and print what a film
                            would cover. Nothing is traced and nothing is written.
    --step <tau>            The proper time between frames, in M. Default 0.1.
    --frames <n>            The most frames. Default: until the worldline ends, at most 10000.

THE CAMERA
    The camera is tied to the hole, not carried by gyroscopes: at every frame it faces the hole
    (heading zero, the centre of the video frame, is the direction in the observer's rest space in
    which r falls fastest), with the hole's spin axis up. The hole keeps its place in the frame and
    the sky moves round it. An observer moving across the radial direction sees the shadow
    displaced toward the motion by aberration, as it would be.

THE OBSERVER
    The observer is carried forward as the app would carry it: along its geodesic in free fall;
    at fixed radius in closed form if static or a ZAMO; held at its radius until its release if it
    is still waiting at the saved moment, and then released exactly as the app releases it. A
    dragged observer is refused, since a worldline positioned by the mouse is not a physical one;
    so is an observer at or inside the inner horizon r-, which this program cannot yet film, and a
    hole that spins the other way (a < 0), which this version does not yet film.

WHERE A FILM ENDS
    At the frame count, or earlier where the worldline ends: at the ring; where it freezes onto the
    inner horizon (the app stops following it there); or where it crosses the inner horizon, since
    the view from inside r- is not yet traced. The film ends at the last frame before, and says why.
    A film of a worldline that does not end (a static observer, an orbit) needs --frames or
    --seconds.

THE DIRECTION OF TRAVEL
    The bundle gives the observer's speed past each local reference observer that exists at the
    event - the static observer outside r = 2 M, the ZAMO outside r+, and the raindrop (the
    E = 1, L = 0 fall from rest at infinity) where neither exists - and the direction of that
    travel on the observer's sky, as a heading to the right of the hole and as a mark the renderer
    draws: a ring past the static observer, a diamond past the ZAMO, a triangle past the raindrop.
    Above 0.9999 c the Lorentz factor is given instead of the speed. An observer at rest relative
    to one of them has no direction of travel past it, and no mark.

UNITS
    Times and radii are stored in M, the hole's mass in geometric units. Seconds and kilometres
    come from the hole's mass in solar masses and GM_sun/c^3 = 4.925490947e-6 s, and a kilometre
    of radius is c = 299792.458 km/s times that, as in Black Hole Lab itself.
";

/// Which of the save's two observers to film.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Who {
    Alice,
    Bob,
}

impl Who {
    pub fn name(self) -> &'static str {
        match self {
            Self::Alice => "Alice",
            Self::Bob => "Bob",
        }
    }
}

/// A parsed command line.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Print [`HELP`] and stop.
    Help,
    /// Walk the observer and say what the film would cover.
    Info(Options),
    /// Walk the observer, trace every frame and write the bundle.
    Trace(Trace),
}

/// Everything a dry run needs, checked.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub save: PathBuf,
    pub who: Who,
    /// Proper time between frames, in M; positive and finite.
    pub step: f64,
    /// The most frames the film may have; at least 1.
    pub frames: usize,
    /// Whether `frames` was asked for, or is the default cap.
    pub frames_given: bool,
}

/// Whose film a tracing run makes.
#[derive(Debug, Clone, PartialEq)]
pub enum Subject {
    /// An observer of a save.
    Save { path: PathBuf, who: Who },
    /// The static observer of a hover test, at chart radius `r` around a hole of spin `spin` and
    /// mass 1, which weighs `solar_masses`.
    Hover {
        r: f64,
        spin: f64,
        solar_masses: f64,
    },
}

/// Everything a tracing run needs, checked as far as a command line can be. The hover's radius is
/// checked against the static limit where the metric is at hand, in `film::hover_observer`.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub subject: Subject,
    pub out: PathBuf,
    /// W and H, with W = 2 H.
    pub width: u32,
    pub height: u32,
    /// Frames per second of video; positive and finite.
    pub fps: f64,
    /// Proper time shown per second of video, in M; positive and finite.
    pub rate: f64,
    /// The film's length in frames, from --frames or --seconds; `None` for the worldline's end.
    pub frames: Option<usize>,
    /// Threads to trace on; `None` for all the machine has.
    pub threads: Option<usize>,
    pub resume: bool,
    /// How the read-outs are declared; physical unless `--units geometric`.
    pub units: Units,
}

/// The default video frame rate.
pub const DEFAULT_FPS: f64 = 30.0;

/// The default proper time per second of video, in M.
pub const DEFAULT_RATE: f64 = 0.25;

/// The default grid: 1024 x 512.
pub const DEFAULT_GRID: (u32, u32) = (1024, 512);

/// The most rays a frame may hold: 16384 x 8192, as `sky-testgen` allows. A frame is held in
/// memory whole, at 19 bytes a ray before it is encoded and about as much again while it is.
pub const MAX_RAYS: u64 = 16384 * 8192;

const VALUED: [&str; 4] = ["--info", "--observer", "--step", "--frames"];

/// The flags of a tracing run that take a value.
const TRACE_VALUED: [&str; 12] = [
    "--out",
    "--observer",
    "--grid",
    "--fps",
    "--rate",
    "--frames",
    "--seconds",
    "--threads",
    "--hover",
    "--spin",
    "--solar-masses",
    "--units",
];

/// Reads the command line, without the program's own name.
///
/// `--info` anywhere makes it a dry run, read exactly as before tracing was added; otherwise it is
/// a tracing run.
pub fn parse(args: &[String]) -> Result<Command, String> {
    if args.iter().any(|a| a == "--help" || a == "-h") {
        return Ok(Command::Help);
    }
    if args.is_empty() {
        return Err(
            "nothing to do: film a save with sky-trace <file.bhl> --out <dir>, or ask for a dry \
             run with sky-trace --info <file.bhl>; sky-trace --help lists the rest"
                .into(),
        );
    }
    if args.iter().any(|a| a == "--info") {
        parse_info(args)
    } else {
        parse_trace(args)
    }
}

/// The dry run's command line.
fn parse_info(args: &[String]) -> Result<Command, String> {
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        if !VALUED.contains(&flag) {
            if flag == "--resume" || TRACE_VALUED.contains(&flag) {
                return Err(format!(
                    "there is no option {flag} in a dry run, which traces nothing and writes \
                     nothing; leave out --info to film"
                ));
            }
            return Err(format!(
                "there is no option {flag:?}; sky-trace --help lists the options"
            ));
        }
        let Some(value) = rest.next() else {
            return Err(format!("{flag} needs a value after it"));
        };
        if given.iter().any(|(f, _)| *f == flag) {
            return Err(format!(
                "{flag} is given twice; give it once, with the value you mean"
            ));
        }
        given.push((flag, value.as_str()));
    }
    let value = |flag: &str| given.iter().find(|(f, _)| *f == flag).map(|(_, v)| *v);

    let save =
        value("--info").ok_or("--info needs the save to read: sky-trace --info <file.bhl>")?;
    let who = parse_who(value("--observer"))?;
    let step = match value("--step") {
        None => DEFAULT_STEP,
        Some(text) => match text.trim().parse::<f64>() {
            Ok(v) if v.is_finite() && v > 0.0 => v,
            _ => {
                return Err(format!(
                    "--step wants a proper time greater than zero, in M, and was given {text:?}"
                ));
            }
        },
    };
    let (frames, frames_given) = match value("--frames") {
        None => (DEFAULT_MAX_FRAMES, false),
        Some(text) => (parse_frames(text)?, true),
    };
    Ok(Command::Info(Options {
        save: PathBuf::from(save),
        who,
        step,
        frames,
        frames_given,
    }))
}

fn parse_who(text: Option<&str>) -> Result<Who, String> {
    match text {
        None | Some("bob") | Some("Bob") => Ok(Who::Bob),
        Some("alice") | Some("Alice") => Ok(Who::Alice),
        Some(other) => Err(format!(
            "there is no observer {other:?}; a save has two, bob and alice"
        )),
    }
}

fn parse_frames(text: &str) -> Result<usize, String> {
    match text.trim().parse::<usize>() {
        Ok(n) if n >= 1 => Ok(n),
        _ => Err(format!(
            "--frames wants a whole number of frames, at least 1, and was given {text:?}"
        )),
    }
}

/// A finite number greater than zero.
fn positive(flag: &str, text: &str) -> Result<f64, String> {
    match text.trim().parse::<f64>() {
        Ok(v) if v.is_finite() && v > 0.0 => Ok(v),
        _ => Err(format!(
            "{flag} wants a number greater than zero, and was given {text:?}"
        )),
    }
}

/// A tracing run's command line.
fn parse_trace(args: &[String]) -> Result<Command, String> {
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut save: Option<&str> = None;
    let mut resume = false;
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        if flag == "--resume" {
            if resume {
                return Err("--resume is given twice".into());
            }
            resume = true;
        } else if TRACE_VALUED.contains(&flag) {
            // The next argument is the value whatever it looks like, so that `--spin -0.5` is
            // read as a spin, and refused as one.
            let Some(value) = rest.next() else {
                return Err(format!("{flag} needs a value after it"));
            };
            if given.iter().any(|(f, _)| *f == flag) {
                return Err(format!(
                    "{flag} is given twice; give it once, with the value you mean"
                ));
            }
            given.push((flag, value.as_str()));
        } else if flag == "--step" {
            return Err(
                "--step belongs to the dry run; a film's frames are --rate / --fps apart in the \
                 observer's proper time"
                    .into(),
            );
        } else if flag.starts_with('-') {
            return Err(format!(
                "there is no option {flag:?}; sky-trace --help lists the options"
            ));
        } else if let Some(first) = save {
            return Err(format!(
                "two saves are named, {first:?} and {flag:?}; a film is of one observer of one \
                 save"
            ));
        } else {
            save = Some(flag);
        }
    }
    let value = |flag: &str| given.iter().find(|(f, _)| *f == flag).map(|(_, v)| *v);

    let subject = match (save, value("--hover")) {
        (Some(_), Some(_)) => {
            return Err(
                "both a save and --hover name an observer; film one of them at a time".into(),
            );
        }
        (None, None) => {
            return Err(
                "no observer is named: film a save, sky-trace <file.bhl> --out <dir>, or a static \
                 test observer, sky-trace --hover <r> --spin <a> --frames <n> --out <dir>"
                    .into(),
            );
        }
        (Some(path), None) => {
            if value("--spin").is_some() || value("--solar-masses").is_some() {
                return Err(
                    "--spin and --solar-masses describe the hole of a --hover test, and a save \
                     states its own hole; leave them out"
                        .into(),
                );
            }
            Subject::Save {
                path: PathBuf::from(path),
                who: parse_who(value("--observer"))?,
            }
        }
        (None, Some(r_text)) => {
            if value("--observer").is_some() {
                return Err(
                    "--observer chooses between the two observers of a save, and a --hover test \
                     has one; leave it out"
                        .into(),
                );
            }
            let r = match r_text.trim().parse::<f64>() {
                Ok(v) if v.is_finite() => v,
                _ => {
                    return Err(format!(
                        "--hover wants the static observer's radius in M, and {r_text:?} is not \
                         a number"
                    ));
                }
            };
            let spin_text = value("--spin").ok_or(
                "--hover needs --spin <a> as well: the hole's spin in M, at least 0 and less \
                 than 1",
            )?;
            let spin = match spin_text.trim().parse::<f64>() {
                Ok(a) if (0.0..1.0).contains(&a) => a,
                _ => {
                    return Err(format!(
                        "--spin wants the hole's spin in M, at least 0 and less than 1, and was \
                         given {spin_text:?}"
                    ));
                }
            };
            let solar_masses = match value("--solar-masses") {
                None => sky_trace::film::SGR_A_SOLAR_MASSES,
                Some(text) => positive("--solar-masses", text)?,
            };
            if value("--frames").is_none() && value("--seconds").is_none() {
                return Err(
                    "a --hover observer stays where it is for ever, so its film needs a length: \
                     give --frames <n> or --seconds <s>"
                        .into(),
                );
            }
            Subject::Hover {
                r,
                spin,
                solar_masses,
            }
        }
    };
    let out = value("--out").ok_or("--out is required: the directory to write the bundle into")?;
    let (width, height) = match value("--grid") {
        None => DEFAULT_GRID,
        Some(text) => parse_grid(text)?,
    };
    let fps = match value("--fps") {
        None => DEFAULT_FPS,
        Some(text) => positive("--fps", text)?,
    };
    let rate = match value("--rate") {
        None => DEFAULT_RATE,
        Some(text) => positive("--rate", text)?,
    };
    let frames = match (value("--frames"), value("--seconds")) {
        (Some(_), Some(_)) => {
            return Err(
                "--frames and --seconds both set the film's length; give one of them".into(),
            );
        }
        (Some(text), None) => Some(parse_frames(text)?),
        (None, Some(text)) => {
            let seconds = positive("--seconds", text)?;
            // Rounded, not truncated: 10 s at 30 fps is 300 frames even if the product comes out
            // a hair under 300.
            let count = (seconds * fps).round();
            if count < 1.0 {
                return Err(format!(
                    "a film of {seconds} s at {fps} frames per second has no frames; make \
                     --seconds or --fps larger"
                ));
            }
            if count > f64::from(u32::MAX) {
                return Err(format!(
                    "a film of {seconds} s at {fps} frames per second has {count} frames, more \
                     than a bundle can number"
                ));
            }
            Some(count as usize)
        }
        (None, None) => None,
    };
    if frames.is_some_and(|n| n > u32::MAX as usize) {
        return Err("--frames asks for more frames than a bundle can number".into());
    }
    let threads = match value("--threads") {
        None => None,
        Some(text) => match text.trim().parse::<usize>() {
            Ok(n) if n >= 1 => Some(n),
            _ => {
                return Err(format!(
                    "--threads wants a whole number of threads, at least 1, and was given \
                     {text:?}"
                ));
            }
        },
    };
    let units = match value("--units") {
        None | Some("physical") => Units::Physical,
        Some("geometric") => Units::Geometric,
        Some(other) => {
            return Err(format!(
                "--units wants physical (seconds and kilometres, the default) or geometric (M \
                 alone), and was given {other:?}"
            ));
        }
    };
    Ok(Command::Trace(Trace {
        subject,
        out: PathBuf::from(out),
        width,
        height,
        fps,
        rate,
        frames,
        threads,
        resume,
        units,
    }))
}

/// `<W>x<H>`, both whole numbers of at least 1, W twice H.
fn parse_grid(text: &str) -> Result<(u32, u32), String> {
    let malformed = || {
        format!(
            "--grid wants the columns and rows as <W>x<H>, such as 1024x512, and was given \
             {text:?}"
        )
    };
    let (w, h) = text.split_once('x').ok_or_else(malformed)?;
    let w: u32 = w.parse().map_err(|_| malformed())?;
    let h: u32 = h.parse().map_err(|_| malformed())?;
    if w == 0 || h == 0 {
        return Err(format!(
            "--grid {text} has no rays; a grid needs at least one column and one row"
        ));
    }
    if u64::from(w) != 2 * u64::from(h) {
        return Err(format!(
            "--grid {text} is not twice as wide as it is high; the grid spans 360 degrees across \
             and 180 down, so W must be 2H, such as 1024x512"
        ));
    }
    if u64::from(w) * u64::from(h) > MAX_RAYS {
        return Err(format!(
            "--grid {text} has {} rays, and this program traces at most {MAX_RAYS} (16384x8192) \
             to keep a frame within memory",
            u64::from(w) * u64::from(h)
        ));
    }
    Ok((w, h))
}
