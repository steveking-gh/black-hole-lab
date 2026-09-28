//! A film: the file handling between the command line and `sky_trace::film`.
//!
//! In order: name the observer (a save's, or the static observer of a hover test); walk the whole
//! worldline, which is cheap, so that the frame count and the reason the film ends are known before
//! any tracing is spent; check every event's triad, so that an event the camera cannot be set up
//! at is refused now and not hours in; print the dry run's summary; then trace the frames one at a
//! time, each on all the threads, writing each atomically and rewriting the manifest after it, so
//! that a crash costs at most the frame in flight and `--resume` picks up from there.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use kerr_equatorial::KerrSchild;
use kerr_sky::Kerr;
use sky_format::{BundleWriter, MANIFEST, Manifest, Source};
use sky_trace::bhl::{self, Hole, Save, SavedObserver};
use sky_trace::film::{self, Setup};
use sky_trace::worldline::{End, Worldline};

use crate::args::{DEFAULT_MAX_FRAMES, Subject, Trace};
use crate::{Described, describe, observer_of};

/// How often, in wall time, a progress line is printed. A frame of the default grid takes seconds,
/// so in practice this is a line a frame.
const PROGRESS_EVERY: Duration = Duration::from_secs(1);

/// The star map the printed `sky-render` command names: the default map of `sky/maps`, in
/// galactic coordinates like the bundle's far sky.
const STAR_MAP: &str = "maps/starmap_2020_8k_gal.exr";

/// Why a run stopped. The two kinds exit with different codes, so that a script can tell a
/// command line to fix from a disk that failed.
#[derive(Debug)]
pub enum Failure {
    /// The command line, the save or the directory it names is not something this program will
    /// act on. Nothing has been written.
    Usage(String),
    /// Writing the bundle failed part-way. What was written is intact; `--resume` goes on.
    Write(String),
}

impl Failure {
    pub fn message(&self) -> &str {
        match self {
            Self::Usage(m) | Self::Write(m) => m,
        }
    }
}

/// What a finished film did.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    /// Frames the film has.
    pub frames: u32,
    /// Frames this run traced and wrote.
    pub written: u32,
    /// Frames a resumed run found complete and left alone.
    pub skipped: u32,
    /// Unresolved rays over the frames this run traced.
    pub unresolved: u64,
    /// Why the film ends where it does.
    pub end: End,
}

/// The observer to film, whichever way it was named.
struct Named {
    /// The first line of the summary.
    title: String,
    hover: bool,
    save: Save,
    observer: SavedObserver,
    metric: KerrSchild,
    source: Source,
}

fn name_the_observer(subject: &Subject) -> Result<Named, String> {
    match subject {
        Subject::Save { path, who } => {
            let save = bhl::read_file(path).map_err(|e| format!("{}: {e}", path.display()))?;
            let observer = observer_of(&save, *who)?.clone();
            let metric = save.hole.metric();
            // The bundle's time unit is M and its geometry has mass 1; the app keeps M at 1, and a
            // save that did not would need every time and radius rescaled on the way out.
            if metric.m != 1.0 {
                return Err(format!(
                    "the hole in this save has M = {} in the chart's units, and this program \
                     films only the app's M = 1",
                    metric.m
                ));
            }
            film::check_spin(metric.a)?;
            let name = path
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| path.display().to_string());
            Ok(Named {
                title: path.display().to_string(),
                hover: false,
                source: Source {
                    kind: "scenario".into(),
                    name: Some(name),
                    state_hash: (!save.state_hash.is_empty()).then(|| save.state_hash.clone()),
                },
                save,
                observer,
                metric,
            })
        }
        &Subject::Hover {
            r,
            spin,
            solar_masses,
        } => {
            // Built directly rather than by `KerrSchild::with_solar_mass`, whose clamp would turn
            // a spin above 0.9999 into 0.9999 without saying so.
            let metric = KerrSchild {
                m: 1.0,
                a: spin,
                m_solar: solar_masses,
            };
            let observer = film::hover_observer(&metric, r)?;
            let hole = Hole {
                m: 1.0,
                a: spin,
                m_solar: solar_masses,
            };
            let title =
                format!("a static observer at r = {r} M on the equatorial plane, a = {spin} M");
            Ok(Named {
                source: Source {
                    kind: "hover test".into(),
                    name: Some(title.clone()),
                    state_hash: None,
                },
                title,
                hover: true,
                save: Save {
                    written_by: None,
                    saved_at_utc: String::new(),
                    note: String::new(),
                    state_hash: String::new(),
                    hole,
                    clock: 0.0,
                    alice: None,
                    bob: Some(observer.clone()),
                },
                observer,
                metric,
            })
        }
    }
}

/// The first setting in which a bundle found on disk differs from the one this run would start,
/// or `None` when it is the same run. The writer's version is not compared: a later build of this
/// program writes the same frames, and refusing its bundles would only make them unresumable.
fn differing_setting(found: &Manifest, wanted: &Manifest) -> Option<&'static str> {
    // The hole before the source: a hover test's name states the spin as well, and "a different
    // hole" is the more useful thing to say about a changed --spin.
    [
        ("hole", found.geometry == wanted.geometry),
        ("mass in solar masses", found.time_unit == wanted.time_unit),
        ("save or hover test", found.source == wanted.source),
        ("grid (--grid)", found.grid == wanted.grid),
        ("far-sky orientation", found.far_sky == wanted.far_sky),
        ("--fps or --rate", found.playback == wanted.playback),
        // The triad sentence carries the first frame's proper time and radius along with the rate:
        // a different observer of the same save differs there, or in its name.
        ("observer (--observer)", found.observer == wanted.observer),
        (
            "length (--frames or --seconds)",
            found.frames_planned == wanted.frames_planned,
        ),
        // Last, because the declarations follow from everything above: the units of the clocks
        // and the ruler are chosen from the whole film, and which reference observers occur, and
        // whether any is passed faster than 0.9999 c, from its every frame. With the settings
        // above the same, the walk is the same to the bit and so are they, unless --units
        // differs, or the bundle was written by a build that declared other read-outs or marks -
        // one from before displays and marks, say - whose frames would not match this build's.
        (
            "set of read-outs or their units (--units)",
            found.readouts == wanted.readouts,
        ),
        ("set of marks", found.marks == wanted.marks),
    ]
    .into_iter()
    .find(|(_, same)| !same)
    .map(|(what, _)| what)
}

/// Makes the film the options describe, printing the summary and progress to `out`.
pub fn film(options: &Trace, out: &mut dyn Write) -> Result<Summary, Failure> {
    let usage = Failure::Usage;
    let named = name_the_observer(&options.subject).map_err(usage)?;
    let metric = named.metric;
    let who = named.observer.name.clone();

    let mut worldline =
        Worldline::from_saved(&metric, &named.observer).map_err(|e| usage(e.to_string()))?;
    let limit = options.frames.unwrap_or(DEFAULT_MAX_FRAMES);
    let walked = film::walk(&mut worldline, options.rate, options.fps, limit);
    if options.frames.is_none() && matches!(walked.end, End::Frames { .. }) {
        return Err(usage(format!(
            "{who}'s worldline does not end within {DEFAULT_MAX_FRAMES} frames at {} M of proper \
             time a frame, so the film needs a length: give --frames <n> or --seconds <s>",
            options.rate / options.fps
        )));
    }
    if walked.events.is_empty() {
        return Err(usage(format!(
            "there is not one frame to film: {}",
            walked.end
        )));
    }
    let frames = walked.events.len() as u32;

    // Every event's camera and every frame's travel past the reference observers, before any ray
    // is traced: cheap, and a refusal now costs nothing. The read-outs' units are chosen here too,
    // from the whole film.
    let kerr = Kerr::from_equatorial(&metric);
    let taus: Vec<f64> = (0..walked.events.len())
        .map(|k| film::frame_tau(worldline.tau0(), k, options.rate, options.fps))
        .collect();
    let plan = film::plan(
        &kerr,
        &walked.events,
        &taus,
        named.save.hole.seconds_per_unit(),
        options.units,
    )
    .map_err(usage)?;

    let threads = options.threads.unwrap_or_else(|| {
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
    });
    let mut summary = describe(&Described {
        title: named.title.clone(),
        hover: named.hover,
        step: options.rate / options.fps,
        frames_given: options.frames.is_some(),
        metric: &metric,
        save: &named.save,
        obs: &named.observer,
        worldline: &worldline,
        film: &walked,
    });
    summary.push_str(&format!(
        "  film            {frames} frames of {} x {} rays, {} fps, {} M of proper time a second \
         of video ({:.3} s of video)\n  tracing         on {threads} thread{} into {}\n",
        options.width,
        options.height,
        options.fps,
        options.rate,
        f64::from(frames) / options.fps,
        if threads == 1 { "" } else { "s" },
        options.out.display()
    ));
    // Progress is a courtesy: a stdout that has gone away does not stop the run.
    let _ = out.write_all(summary.as_bytes());

    let first = walked.events[0];
    let tau0 = film::frame_tau(worldline.tau0(), 0, options.rate, options.fps);
    let setup = Setup {
        source: named.source.clone(),
        observer: who.clone(),
        spin: metric.a,
        solar_masses: metric.m_solar,
        width: options.width,
        height: options.height,
        fps: options.fps,
        rate: options.rate,
        tau0,
        r0: first.r,
        frames,
        readouts: plan.readouts.clone(),
        marks: plan.marks.clone(),
    };
    let mut writer = open_bundle(&options.out, film::manifest(&setup), options.resume)?;

    let started = Instant::now();
    let mut last_report = started;
    let (mut written, mut skipped, mut unresolved) = (0u32, 0u32, 0u64);
    for (k, event) in walked.events.iter().enumerate() {
        let index = k as u32;
        if writer.is_complete(index) {
            skipped += 1;
            continue;
        }
        let tau = film::frame_tau(worldline.tau0(), k, options.rate, options.fps);
        let traced = film::trace(&kerr, event, index, options.width, options.height, threads)
            .map_err(|why| Failure::Write(format!("frame {k} could not be traced: {why}")))?;
        unresolved += traced.unresolved as u64;
        let entry = film::entry(index, tau, tau0, event, &plan);
        writer
            .write_frame_and_manifest(&traced.frame, entry)
            .map_err(|e| Failure::Write(e.to_string()))?;
        written += 1;

        let done = written + skipped;
        if last_report.elapsed() >= PROGRESS_EVERY || done == frames {
            last_report = Instant::now();
            let rate = f64::from(written) / started.elapsed().as_secs_f64();
            let left = f64::from(frames - done) / rate;
            let _ = writeln!(
                out,
                "frame {} of {frames}: {rate:.3} frames a second, about {} left, {unresolved} \
                 unresolved rays so far",
                k + 1,
                clock(left)
            );
        }
    }

    let elapsed = started.elapsed().as_secs_f64();
    let size = size_on_disk(&options.out);
    let bundle = std::path::absolute(&options.out).unwrap_or_else(|_| options.out.clone());
    let mut report = format!(
        "wrote {written} frame{} in {elapsed:.1} s ({:.3} frames a second){}\n",
        if written == 1 { "" } else { "s" },
        if elapsed > 0.0 {
            f64::from(written) / elapsed
        } else {
            0.0
        },
        if skipped > 0 {
            format!("; {skipped} were already complete")
        } else {
            String::new()
        }
    );
    report.push_str(&format!("the film ends: {}\n", walked.end));
    report.push_str(&format!(
        "unresolved rays: {unresolved} in the frames this run traced\n"
    ));
    report.push_str(&format!(
        "bundle: {} ({size} bytes, {:.1} MB on disk)\n",
        bundle.display(),
        size as f64 / 1e6
    ));
    report.push_str(&format!(
        "render it, from the sky directory:\n    cargo run --release -p sky-render -- --bundle \
         \"{}\" --sky {STAR_MAP} --out \"{}.mkv\"\n",
        bundle.display(),
        bundle.display()
    ));
    let _ = out.write_all(report.as_bytes());

    Ok(Summary {
        frames,
        written,
        skipped,
        unresolved,
        end: walked.end,
    })
}

/// A new bundle at `dir`, or the one there, resumed, if `resume` asks and its settings match.
fn open_bundle(dir: &Path, wanted: Manifest, resume: bool) -> Result<BundleWriter, Failure> {
    if !dir.join(MANIFEST).exists() {
        return BundleWriter::create(dir, wanted).map_err(|e| Failure::Usage(e.to_string()));
    }
    if !resume {
        return Err(Failure::Usage(format!(
            "{} already holds a sky bundle; pass --resume to finish it, or choose another \
             directory",
            dir.display()
        )));
    }
    let writer = BundleWriter::resume(dir).map_err(|e| Failure::Usage(e.to_string()))?;
    if let Some(what) = differing_setting(writer.manifest(), &wanted) {
        return Err(Failure::Usage(format!(
            "{} holds a bundle written with a different {what}; resume it with the settings it \
             was started with, or choose another directory",
            dir.display()
        )));
    }
    Ok(writer)
}

/// A duration in seconds as hours, minutes and seconds.
fn clock(seconds: f64) -> String {
    if !seconds.is_finite() {
        return "an unknown time".into();
    }
    let s = seconds.round() as u64;
    match (s / 3600, (s / 60) % 60, s % 60) {
        (0, 0, s) => format!("{s} s"),
        (0, m, s) => format!("{m} min {s:02} s"),
        (h, m, _) => format!("{h} h {m:02} min"),
    }
}

/// The bytes of every file under `dir`.
fn size_on_disk(dir: &Path) -> u64 {
    let Ok(listing) = std::fs::read_dir(dir) else {
        return 0;
    };
    listing
        .flatten()
        .map(|item| match item.metadata() {
            Ok(m) if m.is_dir() => size_on_disk(&item.path()),
            Ok(m) => m.len(),
            Err(_) => 0,
        })
        .sum()
}
