//! The command line: what it takes, and every way it can be wrong, each as its own sentence.
//!
//! Hand-rolled, like the app's: a dozen flags do not need a parsing crate, and a crate would put
//! its own wording on the refusals where this program wants to say what to do instead.

use std::path::PathBuf;

use sky_format::Grid;

use crate::cases::Case;

/// The most rays a frame may hold: 16384 x 8192. A frame is held in memory whole, at 19 bytes a
/// ray before it is encoded and about as much again while it is, so a grid much larger than this
/// asks for more memory than the machine has and the program would abort rather than refuse.
pub const MAX_RAYS: u64 = 16384 * 8192;

/// What `--help` prints.
pub const HELP: &str = "\
sky-testgen: writes a sky bundle for a flat-space test case, whose right answer is known by
inspection, so that a renderer can be checked before any black-hole physics exists.

USAGE
    sky-testgen --case still|turn --out <dir> [options]

CASES
    still   An observer at rest in flat space, its triad along the far-sky axes. Every ray comes
            from the far sky (fate 1) from the direction it is looked along (d = n), unshifted
            (g = 1) and unwound (winding 0). The sky never changes: a renderer shows the star map
            as it is, the galactic centre in the middle of the frame.

    turn    The same observer turning at a constant rate about a fixed axis. The axis is given in
            the far-sky frame, and the turn is right-handed about it. At watch time t the triad
            is the far-sky axes turned by the angle rate * t about the axis, so a direction with
            triad components n comes from the far-sky direction d = R(axis, rate * t) n.
            A positive rate about +Z turns the observer to the LEFT (counter-clockwise seen from
            above). The sky stays put, so the viewer sees it move to the RIGHT across the frame.

OPTIONS
    --case still|turn       The case to write. Required.
    --out <dir>             The bundle directory. Required. It is created if it is not there.
    --grid <W>x<H>          The observer-sky grid, in columns and rows. Default 1024x512.
    --seconds <s>           The length of the run, in the observer's proper time. Default 10.
    --fps <n>               Frames per second of proper time. Default 30. Frame k is at proper
                            time k / fps exactly, and the run has round(seconds * fps) frames.
    --rate <deg/s>          For turn: the rate of turn in degrees per second. Default 6, one
                            whole turn a minute. Negative turns the other way.
    --axis <x>,<y>,<z>      For turn: the axis in the far-sky frame. Default 0,0,1. It is
                            normalised; an axis of zero length is refused.
    --resume                Continue a bundle a previous run left at --out, writing only the
                            frames it lacks. The settings must be the ones it was started with.
                            With no bundle there yet, a new one is started. Without --resume, an
                            existing bundle at --out is refused rather than written over.
    --help, -h              Print this and do nothing else.

The bundle's time unit is the second. The far sky is oriented by the galactic preset: X toward
the galactic centre, Z toward the north galactic pole. The video plays one second of proper time
per second, at the same frame rate. Each frame carries the stopwatch read-out, the proper time
since frame 0.
";

/// A parsed command line.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Print [`HELP`] and stop.
    Help,
    /// Write a bundle.
    Run(Options),
}

/// Everything a run needs, checked.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub case: Case,
    pub out: PathBuf,
    pub grid: Grid,
    /// Frames per second of proper time; positive and finite.
    pub fps: f64,
    /// How many frames the run writes, numbered 0 to `frames - 1`; at least 1.
    pub frames: u32,
    pub resume: bool,
}

impl Options {
    /// The proper time of frame `index`, in seconds: `index / fps`, computed as that one division
    /// for every frame so that no rounding accumulates along a run.
    pub fn proper_time(&self, index: u32) -> f64 {
        f64::from(index) / self.fps
    }
}

/// The flags that take a value, and the one that does not.
const VALUED: [&str; 7] = [
    "--case",
    "--out",
    "--grid",
    "--seconds",
    "--fps",
    "--rate",
    "--axis",
];

/// Reads the command line, without the program's own name.
pub fn parse(args: &[String]) -> Result<Command, String> {
    // `--help` wins wherever it is and whatever else is wrong: someone who asks for help is
    // usually someone whose command line is wrong.
    if args.iter().any(|a| a == "--help" || a == "-h") {
        return Ok(Command::Help);
    }
    if args.is_empty() {
        return Err(
            "no case was asked for; try sky-testgen --case still --out <dir>, or \
                    sky-testgen --help"
                .into(),
        );
    }

    let mut given: Vec<(&str, &str)> = Vec::new();
    let mut resume = false;
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        let flag = arg.as_str();
        if flag == "--resume" {
            if resume {
                return Err("--resume is given twice".into());
            }
            resume = true;
        } else if VALUED.contains(&flag) {
            // The next argument is the value whatever it looks like, so that `--axis -1,0,0` and
            // `--rate -6` work.
            let Some(value) = rest.next() else {
                return Err(format!("{flag} needs a value after it"));
            };
            if given.iter().any(|(f, _)| *f == flag) {
                return Err(format!(
                    "{flag} is given twice; give it once, with the value you mean"
                ));
            }
            given.push((flag, value.as_str()));
        } else {
            return Err(format!(
                "there is no option {flag:?}; sky-testgen --help lists the options"
            ));
        }
    }
    let value = |flag: &str| given.iter().find(|(f, _)| *f == flag).map(|(_, v)| *v);

    let case_name = value("--case").ok_or("--case is required: --case still or --case turn")?;
    let out = value("--out").ok_or("--out is required: the directory to write the bundle into")?;
    let grid = match value("--grid") {
        Some(text) => parse_grid(text)?,
        None => Grid::new(1024, 512),
    };
    let seconds = match value("--seconds") {
        Some(text) => positive("--seconds", text)?,
        None => 10.0,
    };
    let fps = match value("--fps") {
        Some(text) => positive("--fps", text)?,
        None => 30.0,
    };

    let case = match case_name {
        "still" => {
            if value("--rate").is_some() || value("--axis").is_some() {
                return Err(
                    "--rate and --axis set how the turn case turns, and the still case \
                            does not turn; leave them out, or ask for --case turn"
                        .into(),
                );
            }
            Case::Still
        }
        "turn" => Case::Turn {
            axis: match value("--axis") {
                Some(text) => parse_axis(text)?,
                None => [0.0, 0.0, 1.0],
            },
            degrees_per_second: match value("--rate") {
                Some(text) => number("--rate", text)?,
                None => 6.0,
            },
        },
        other => {
            return Err(format!(
                "there is no case {other:?}; the cases are still and turn"
            ));
        }
    };

    // Rounded, not truncated: 10 s at 30 fps must be 300 frames even if the product comes out a
    // hair under 300, and the video then lasts frames / fps, as near `seconds` as whole frames
    // allow.
    let count = (seconds * fps).round();
    if count < 1.0 {
        return Err(format!(
            "a run of {seconds} s at {fps} frames per second has no frames; make --seconds or \
             --fps larger"
        ));
    }
    if count > f64::from(u32::MAX) {
        return Err(format!(
            "a run of {seconds} s at {fps} frames per second has {count} frames, more than a \
             bundle can number"
        ));
    }

    Ok(Command::Run(Options {
        case,
        out: PathBuf::from(out),
        grid,
        fps,
        frames: count as u32,
        resume,
    }))
}

/// `<W>x<H>`, both whole numbers of at least 1.
fn parse_grid(text: &str) -> Result<Grid, String> {
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
    if u64::from(w) * u64::from(h) > MAX_RAYS {
        return Err(format!(
            "--grid {text} has {} rays, and this program writes at most {MAX_RAYS} (16384x8192) \
             to keep a frame within memory",
            u64::from(w) * u64::from(h)
        ));
    }
    Ok(Grid::new(w, h))
}

/// Three numbers separated by commas, normalised to a unit vector.
fn parse_axis(text: &str) -> Result<[f64; 3], String> {
    let parts: Vec<&str> = text.split(',').collect();
    let [x, y, z] = parts.as_slice() else {
        return Err(format!(
            "--axis wants three numbers separated by commas, such as 0,0,1, and was given \
             {text:?}"
        ));
    };
    let v = [
        number("--axis", x)?,
        number("--axis", y)?,
        number("--axis", z)?,
    ];
    // Scaled by the largest component before the length is taken, so that an axis such as
    // 1e-200,0,0 - short, but a direction all the same - does not square to zero and get refused.
    let largest = v.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if largest == 0.0 {
        return Err(format!(
            "--axis {text} has no direction; an axis must have a length other than zero"
        ));
    }
    let scaled = v.map(|c| c / largest);
    let length = scaled.iter().map(|c| c * c).sum::<f64>().sqrt();
    Ok(scaled.map(|c| c / length))
}

/// A finite number.
fn number(flag: &str, text: &str) -> Result<f64, String> {
    match text.trim().parse::<f64>() {
        Ok(v) if v.is_finite() => Ok(v),
        _ => Err(format!(
            "{flag} wants a finite number, and {text:?} is not one"
        )),
    }
}

/// A finite number greater than zero.
fn positive(flag: &str, text: &str) -> Result<f64, String> {
    let v = number(flag, text)?;
    if v > 0.0 {
        Ok(v)
    } else {
        Err(format!(
            "{flag} must be greater than zero, and was given {text}"
        ))
    }
}
