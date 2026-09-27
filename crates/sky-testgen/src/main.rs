//! sky-testgen: writes sky bundles for flat-space test cases.
//!
//! A sky bundle records, for each moment of an observer's watch and each direction on the
//! observer's sky, where on the distant sky the light came from. Eventually a tracer writes them
//! for an observer near a black hole; before that physics exists, this program writes bundles
//! whose right answer is known by inspection, so that the renderer and everything downstream of
//! the format can be proved on their own. `sky-testgen --help` states the cases.
//!
//! The cases' mathematics is in `cases`, the command line in `args`, and what is here is the
//! file handling between them: one frame at a time, each written atomically and followed by a
//! rewrite of the manifest, so that a crash costs at most the frame in flight and `--resume`
//! picks up from there.

mod args;
mod cases;
#[cfg(test)]
mod tests;

use std::io::Write;
use std::time::{Duration, Instant};

use sky_format::{
    BundleWriter, FORMAT, FarSky, FrameEntry, Geometry, GridSpec, MANIFEST, Manifest, Num,
    Observer, Playback, Position, ReadoutDecl, STOPWATCH, Source, TimeUnit, VERSION, WriterInfo,
};

use args::{Command, Options};

/// The program's name, as the manifest's `writer.program` records it and messages begin.
const PROGRAM: &str = "sky-testgen";

/// How often, in wall time, a progress line is printed.
const PROGRESS_EVERY: Duration = Duration::from_secs(1);

/// Why a run stopped. The two kinds exit with different codes, so that a script can tell a
/// command line to fix from a disk that failed.
#[derive(Debug)]
pub enum Failure {
    /// The command line, or the directory it names, is not something this program will act on.
    /// Nothing has been written.
    Usage(String),
    /// Writing the bundle failed part-way. What was written is intact; `--resume` goes on.
    Write(String),
}

impl Failure {
    fn message(&self) -> &str {
        match self {
            Self::Usage(m) | Self::Write(m) => m,
        }
    }
}

/// What a finished run did.
#[derive(Debug, PartialEq)]
pub struct Summary {
    /// Frames this run wrote.
    pub written: u32,
    /// Frames a resumed run found complete and left alone.
    pub skipped: u32,
}

fn main() {
    // `args_os`, not `args`: `args` panics on an argument that is not Unicode, and bad input is
    // answered with a sentence here, not a panic.
    let args: Result<Vec<String>, String> = std::env::args_os()
        .skip(1)
        .map(|a| {
            a.into_string().map_err(|a| {
                format!(
                    "the argument {} is not valid Unicode; give the path another name",
                    a.to_string_lossy()
                )
            })
        })
        .collect();
    let result = args
        .map_err(Failure::Usage)
        .and_then(|args| run(&args, &mut std::io::stdout()));
    let code = match result {
        Ok(_) => 0,
        Err(failure) => {
            eprintln!("{PROGRAM}: {}.", failure.message());
            match failure {
                Failure::Usage(_) => 2,
                Failure::Write(_) => 1,
            }
        }
    };
    std::process::exit(code);
}

/// Runs the program on a command line (without the program's name), printing progress and the
/// help text to `out`. Returns what it wrote; `None` in the summary's place means help was asked
/// for.
pub fn run(args: &[String], out: &mut dyn Write) -> Result<Option<Summary>, Failure> {
    let options = match args::parse(args).map_err(Failure::Usage)? {
        Command::Help => {
            // A closed stdout is not worth a failure: the help was all there was to do.
            let _ = out.write_all(args::HELP.as_bytes());
            return Ok(None);
        }
        Command::Run(options) => options,
    };
    write_bundle(&options, out).map(Some)
}

/// The manifest this run means to write, with no frames yet.
pub fn manifest(options: &Options) -> Manifest {
    Manifest {
        format: FORMAT.into(),
        version: VERSION,
        writer: WriterInfo {
            program: PROGRAM.into(),
            version: env!("CARGO_PKG_VERSION").into(),
            git: None,
        },
        source: Source {
            kind: "flat-space test".into(),
            name: Some(options.case.name().into()),
            state_hash: None,
        },
        geometry: Geometry {
            kind: "flat".into(),
            mass: None,
            spin: None,
        },
        time_unit: TimeUnit {
            name: "s".into(),
            seconds: Num(1.0),
        },
        observer: Some(Observer {
            name: None,
            triad: Some(options.case.triad()),
        }),
        grid: GridSpec::new(options.grid.width, options.grid.height),
        far_sky: FarSky::galactic(),
        playback: Playback {
            frames_per_second: Num(options.fps),
            proper_time_per_video_second: Num(1.0),
        },
        readouts: vec![ReadoutDecl {
            id: STOPWATCH.into(),
            label: "Stopwatch".into(),
            unit: "s".into(),
            decimals: 2,
        }],
        labels: Vec::new(),
        frames_planned: Some(options.frames),
        frames: Vec::new(),
    }
}

/// The first setting in which a bundle found on disk differs from the one this run would start,
/// or `None` when it is the same run. The writer's version is not compared: a later build of this
/// program writes the same frames, and refusing its bundles would only make them unresumable.
fn differing_setting(found: &Manifest, wanted: &Manifest) -> Option<&'static str> {
    [
        ("case", found.source == wanted.source),
        ("geometry", found.geometry == wanted.geometry),
        ("time unit", found.time_unit == wanted.time_unit),
        ("grid", found.grid == wanted.grid),
        ("far-sky orientation", found.far_sky == wanted.far_sky),
        ("frame rate", found.playback == wanted.playback),
        ("set of read-outs", found.readouts == wanted.readouts),
        // The triad sentence carries --rate and --axis, and the planned frame count carries
        // --seconds: the manifest has no other place for either.
        ("rate or axis of turn", found.observer == wanted.observer),
        (
            "length (--seconds)",
            found.frames_planned == wanted.frames_planned,
        ),
    ]
    .into_iter()
    .find(|(_, same)| !same)
    .map(|(what, _)| what)
}

/// Opens the bundle the options name, new or resumed, and writes every frame it lacks.
fn write_bundle(options: &Options, out: &mut dyn Write) -> Result<Summary, Failure> {
    let dir = &options.out;
    let wanted = manifest(options);
    let exists = dir.join(MANIFEST).exists();
    let mut writer = if exists {
        if !options.resume {
            return Err(Failure::Usage(format!(
                "{} already holds a sky bundle; pass --resume to finish it, or choose another \
                 directory",
                dir.display()
            )));
        }
        let writer = BundleWriter::resume(dir).map_err(|e| Failure::Usage(e.to_string()))?;
        if let Some(what) = differing_setting(writer.manifest(), &wanted) {
            return Err(Failure::Usage(format!(
                "{} holds a bundle written with a different {}; resume it with the settings it \
                 was started with, or choose another directory",
                dir.display(),
                what
            )));
        }
        writer
    } else {
        BundleWriter::create(dir, wanted).map_err(|e| Failure::Usage(e.to_string()))?
    };

    let started = Instant::now();
    let mut last_report = started;
    let (mut written, mut skipped) = (0, 0);
    for index in 0..options.frames {
        if writer.is_complete(index) {
            skipped += 1;
            continue;
        }
        let t = options.proper_time(index);
        let frame = options.case.frame(options.grid, index, t);
        // The stopwatch is the proper time since frame 0, and frame 0 is at proper time 0, so the
        // two are the same number.
        let entry = FrameEntry {
            position: Some(Position {
                chart: "cartesian".into(),
                coords: [Num(t), Num(0.0), Num(0.0), Num(0.0)],
            }),
            ..FrameEntry::new(index, t).with_readout(STOPWATCH, t)
        };
        writer
            .write_frame_and_manifest(&frame, entry)
            .map_err(|e| Failure::Write(e.to_string()))?;
        written += 1;

        if last_report.elapsed() >= PROGRESS_EVERY {
            last_report = Instant::now();
            // Progress is a courtesy: a stdout that has gone away does not stop the run.
            let _ = writeln!(
                out,
                "frame {} of {} written (proper time {t:.2} s), {:.0}%",
                index + 1,
                options.frames,
                100.0 * f64::from(index + 1) / f64::from(options.frames)
            );
        }
    }

    let _ = writeln!(
        out,
        "wrote {written} frame{} to {} in {:.1} s{}",
        if written == 1 { "" } else { "s" },
        dir.display(),
        started.elapsed().as_secs_f64(),
        if skipped > 0 {
            format!("; {skipped} were already complete")
        } else {
            String::new()
        }
    );
    Ok(Summary { written, skipped })
}
