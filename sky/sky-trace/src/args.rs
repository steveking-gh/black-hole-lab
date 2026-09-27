//! The command line: what it takes, and every way it can be wrong, each as its own sentence.
//!
//! Hand-rolled, like `sky-testgen`'s: four flags do not need a parsing crate, and a crate would put
//! its own wording on the refusals where this program wants to say what to do instead.

use std::path::PathBuf;

/// How many frames a film is allowed when `--frames` is not given. A worldline that ends - at the
/// ring, on the inner horizon - ends well inside this at any sensible step; one that does not (a
/// static observer, an orbit) needs a length, and this is the one it gets.
pub const DEFAULT_MAX_FRAMES: usize = 10_000;

/// The default frame step, in M of the observer's proper time.
pub const DEFAULT_STEP: f64 = 0.1;

/// What `--help` prints.
pub const HELP: &str = "\
sky-trace: follows an observer of a Black Hole Lab save forward from the saved moment, at equal
steps of the observer's own proper time, and will trace what that observer sees of the whole sky.

For now it does a dry run only: it reads the save, walks the observer's worldline, and prints
what a film would cover. Nothing is traced and nothing is written. The tracing is added once the
light-tracing library has been reviewed.

USAGE
    sky-trace --info <file.bhl> [--observer bob|alice] [--step <tau>] [--frames <n>]

OPTIONS
    --info <file.bhl>       The save to read: a .bhl written by Black Hole Lab, compressed or
                            not. Required.
    --observer bob|alice    Whose film. Default bob.
    --step <tau>            The proper time between frames, in M of the observer's own clock.
                            Default 0.1. Frame k is at the saved proper time plus k * step,
                            exactly.
    --frames <n>            The most frames the film has, frame 0 being the saved moment. Default:
                            until the observer's worldline ends, and at most 10000.
    --help, -h              Print this and do nothing else.

THE OBSERVER
    The observer is carried forward as the app would carry it: along its geodesic in free fall;
    at fixed radius in closed form if static or a ZAMO; held at its radius until its release if it
    is still waiting at the saved moment, and then released exactly as the app releases it. A
    dragged observer is refused, since a worldline positioned by the mouse is not a physical one;
    so is an observer at or inside the inner horizon r-, which this program cannot yet film.

WHERE A FILM ENDS
    At the frame count, or earlier where the worldline ends: at the ring; where it freezes onto the
    inner horizon (the app stops following it there); or where it crosses the inner horizon, since
    the view from inside r- is not yet traced. The film ends at the last frame before, and says why.

UNITS
    Times and radii are in M, the hole's mass in geometric units. Seconds are quoted from the
    hole's mass in solar masses and GM_sun/c^3 = 4.925490947e-6 s.
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

const VALUED: [&str; 4] = ["--info", "--observer", "--step", "--frames"];

/// Reads the command line, without the program's own name.
pub fn parse(args: &[String]) -> Result<Command, String> {
    if args.iter().any(|a| a == "--help" || a == "-h") {
        return Ok(Command::Help);
    }
    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        if !VALUED.contains(&flag) {
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

    let save = value("--info").ok_or(
        "this program does only a dry run so far, and needs the save to read: sky-trace \
         --info <file.bhl>, or sky-trace --help",
    )?;
    let who = match value("--observer") {
        None | Some("bob") | Some("Bob") => Who::Bob,
        Some("alice") | Some("Alice") => Who::Alice,
        Some(other) => {
            return Err(format!(
                "there is no observer {other:?}; a save has two, bob and alice"
            ));
        }
    };
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
        Some(text) => match text.trim().parse::<usize>() {
            Ok(n) if n >= 1 => (n, true),
            _ => {
                return Err(format!(
                    "--frames wants a whole number of frames, at least 1, and was given {text:?}"
                ));
            }
        },
    };
    Ok(Command::Info(Options {
        save: PathBuf::from(save),
        who,
        step,
        frames,
        frames_given,
    }))
}
