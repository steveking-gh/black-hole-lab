//! The everyday regression check: the same hot spots and the same frame loops as the full suite,
//! measured at a steady state that is *restored* rather than played to.
//!
//! The full suite spends nine minutes, and nearly all of it getting somewhere. A replay of
//! `isco-pair-64` is thirty M of simulated time, of which the first eighteen are the field filling
//! towards its pulse cap: cost per frame climbs the whole way and the only part worth timing is the
//! plateau at the end. The micro tier pays the same toll differently, playing 20 M before it times
//! its first kernel. That ramp is not waste - it is what shows cost *growing* over a run, which is
//! the one thing this tier cannot see - but it is the wrong thing to pay for twenty times a day.
//!
//! A save is a snapshot and restores to the bit (`crate::save`), so the plateau can be reached once,
//! written to a file, and loaded in seventy-five milliseconds every time after. What is left is ten
//! seconds of pure measurement: all seventeen hot spots against the same field the full micro tier
//! uses, then three windows of consecutive frames on one restored app.
//!
//! **Why three windows on one app, in sequence.** At the steady state the workload is stationary:
//! the pulse count is flat, the ray count is flat, emissions fall at the same cadence throughout.
//! Whatever the second window inherits from the first is therefore the same thing it inherits every
//! run, and the three windows are comparable run to run without needing three loads. They are played
//! in the order below, and the order is part of the measurement:
//!
//! 1. `quick/sim` - `SIM_FRAMES` frames of simulation only, no ui at all.
//! 2. `quick/frame` - `FRAME_FRAMES` whole ui passes through the real `SpacetimeApp::ui` in the
//!    Global chart view, plus tessellation.
//! 3. `quick/frame-volume` - the same again with the View selector on the 2D+1 volume, which is a
//!    different painter and the most expensive thing the app does.
//!
//! **What this tier cannot answer.** Everything about how cost changes over a run: the per-10-M
//! series it prints covers a few M and is flat by construction. Cost growth, the cap of 128, the
//! default-infall layout and the far-branch freeze are the full suite's, and an idea that might
//! change the shape of a run rather than the price of a frame has to be measured there.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::app::SpacetimeApp;
use crate::gui::controls::ReferenceFrame;
use crate::perf::replay::{self, FRAME_DT, ReplayResult};
use crate::perf::{headless_context, micro};

/// Where the save points live.
///
/// Under `target/`, which is gitignored and therefore machine-local, by the owner's decision for
/// now. Committing save points to the repository was agreed in principle and deferred: a save point
/// is 3.8 MB of gzipped JSON and it is a build artefact until somebody wants a shared one. Moving
/// them is changing this constant to a tracked directory and adding the file to it - the harness
/// builds a missing save point wherever this points, and nothing else should need to change.
const SAVEPOINT_DIR: &str = "target/perf/savepoints";

/// The one save point there is: the `isco-pair-64` scenario at `micro::FIXTURE_UNTIL`, which is
/// exactly the state the full micro tier ramps to.
const ISCO_PAIR_64: &str = "isco-pair-64";

/// Frames in the simulation-only window. Four M of simulated time, which at the steady state covers
/// about twenty-six of Alice's emissions and as many of Bob's, so the window holds the full emission
/// cadence several times over rather than catching one at an accident of phase.
const SIM_FRAMES: usize = 240;

/// Frames in each of the two ui windows. Half the sim window, because a whole ui pass costs several
/// times what a simulation step does and the two together are most of this tier's wall time. Two M
/// still covers a dozen emissions apiece.
const FRAME_FRAMES: usize = 120;

/// How long the processor is put to work before the first thing is measured. See `spin_up`.
///
/// Half a second, which is where the effect this is for stops. Measured: with no spin-up at all the
/// first six benchmarks on the list read ten to fifteen percent slow; with 400 ms they read the same
/// as the ones after them, and a two-second spin-up measured over twenty runs was no steadier than
/// the short one and cost a fifth of the tier's whole budget.
const SPIN_UP: Duration = Duration::from_millis(500);

/// Which save point a quick run measured, recorded in the report so that a comparison can say
/// whether both sides measured the same workload.
///
/// The state hash is the file's own `state_hash`, which is `Simulation::fingerprint` of the run
/// inside it: two runs that quote the same hash measured the same field down to the last ray. The
/// git hash is the build that wrote the file, which is what says *why* they differ when they do.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(crate) struct SavepointId {
    pub name: String,
    pub state_hash: String,
    pub written_by_git: String,
}

impl SavepointId {
    /// One line for the report header.
    pub fn line(&self) -> String {
        format!("{} state {} written by git {}", self.name, self.state_hash, self.written_by_git)
    }
}

/// A save point, loaded: the run it holds and what the file says it is.
pub(crate) struct Restored {
    pub id: SavepointId,
    pub app: SpacetimeApp,
}

fn path_of(name: &str) -> PathBuf {
    PathBuf::from(SAVEPOINT_DIR).join(format!("{name}.{}", crate::save::EXTENSION))
}

/// Play the `isco-pair-64` steady state and write it, replacing whatever was there.
///
/// This is `micro::Fixtures::ramped`'s own ramp, stopped at the same place and written out instead
/// of being taken apart, so the save point and the full micro tier's fixture are the same state
/// arrived at by the same route.
fn build(name: &str) -> Result<PathBuf, String> {
    let path = path_of(name);
    let mut app = replay::app_for_fixtures();
    replay::play_sim(&mut app, micro::FIXTURE_UNTIL);
    let note = format!(
        "performance save point: the {name} scenario played to t = {} M, the steady state the \
         --quick tier measures",
        micro::FIXTURE_UNTIL
    );
    app.save_to(&path, &note).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(path)
}

/// `--perf --make-savepoints`: build the save points and say what was written.
///
/// Plural, and there is one of them. The name is of the job rather than of the file, because the
/// job is "put the quick tier's workloads on disk" and a second workload - a heavier cap, a
/// different pair - would be another line here and nothing else anywhere.
pub(crate) fn make_savepoints() -> Result<(), String> {
    let name = ISCO_PAIR_64;
    println!("Building the {name} save point ...");
    let path = build(name)?;
    let bytes = std::fs::metadata(&path).map(|m| m.len() as usize).unwrap_or(0);
    println!("  wrote {} ({})", path.display(), crate::save::human_bytes(bytes));
    Ok(())
}

/// The save point a quick run measures, built first if it is not there.
///
/// It is always read back off disk, even on the run that has just written it. The file is what the
/// next run will measure, so measuring the file is the only way the two runs are of the same state -
/// and the read is what checks the state hash, so a corrupted save point is refused here rather
/// than quietly measured.
pub(crate) fn restore() -> Result<Restored, String> {
    let path = path_of(ISCO_PAIR_64);
    if !path.exists() {
        println!(
            "No save point at {}. Building it once - a few seconds - and carrying on.",
            path.display()
        );
        build(ISCO_PAIR_64)?;
    }
    load(&path)
}

fn load(path: &Path) -> Result<Restored, String> {
    // The document is read for its envelope before the app takes it, because a load consumes it and
    // hands back only the two fields the app's status line wants. The second read inside `load_from`
    // costs a few tens of milliseconds on a file this size and keeps the loading path the one the
    // Load button takes rather than a second copy of it here.
    let bytes = crate::save::read_file(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let document =
        crate::save::read_document(&bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    let id = SavepointId {
        name: path.file_stem().map_or_else(|| "?".to_string(), |s| s.to_string_lossy().into_owned()),
        state_hash: document.state_hash.clone(),
        written_by_git: document.written_by.git.clone(),
    };
    let mut app = SpacetimeApp::default();
    app.load_from(path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(Restored { id, app })
}

/// Put the processor to work before the first thing is measured, and throw the work away.
///
/// The full suite gets this for free. It spends four seconds playing its ramp before it times a
/// single kernel, by which time the core it is on has been busy long enough to have reached its
/// boost clock and filled its caches. A quick run restores its state in seventy-five milliseconds
/// and starts sampling immediately, and measured that way the benchmarks at the *top* of the list
/// came out ten to fifteen percent slow - the first three kernels and the three ray steps, never the
/// ones after them. That is not a property of the code, and it is the worst possible shape of error
/// for this tier: it is one-sided, so whichever of a baseline and a check happened to be the colder
/// run reads as the slower build.
///
/// `SPIN_UP` of `SignalField::advance` over the restored field is the ramp's effect at a hundredth
/// of its cost. It is the app's most characteristic work rather than a busy loop, so what the core
/// is brought up to speed on is the kind of arithmetic that is about to be measured, and it is done
/// on a clone that is dropped, so the run it is done over is not moved by a single step.
pub(crate) fn spin_up(app: &SpacetimeApp) {
    let metric = app.sim.metric;
    let mut field = app.sim.alice_signal.clone();
    let until = Instant::now() + SPIN_UP;
    while Instant::now() < until {
        field.advance(&metric, FRAME_DT);
    }
    std::hint::black_box(&field);
}

/// Measure the three windows, in order, on one restored app.
///
/// Every window runs whatever `--filter` says, and the filter decides only what is reported. The
/// state each window starts from is the state the one before it ended in, so skipping one would
/// change what the others measure and the run would no longer be comparable with a run that did not
/// skip it.
pub(crate) fn windows(app: &mut SpacetimeApp) -> Vec<ReplayResult> {
    let mut out = Vec::with_capacity(3);

    // Simulation only. The app is still paused from the load, which is what `sim_frame` wants: it
    // pushes the Wavefronts kept slider in and steps, exactly as the play loop does, with no ui
    // anywhere near it.
    out.push(replay::sim_window(app, "quick", "sim", SIM_FRAMES));

    // The two ui windows, on one context and one animation clock. The first is the flat Global
    // chart, the second the 2D+1 volume; nothing else about the app changes between them.
    let ctx = headless_context();
    let mut time = 0.0;
    for (scenario, view) in [
        ("frame", ReferenceFrame::DistantObserver),
        ("frame-volume", ReferenceFrame::GlobalVolume),
    ] {
        replay::play_state(app, view);
        out.push(replay::frame_window(app, &ctx, "quick", scenario, FRAME_FRAMES, &mut time));
    }
    out
}

/// The simulated time one quick run covers, in M: what the three windows add up to. Printed so the
/// report says how much of a run it looked at.
pub(crate) fn span_m() -> f64 {
    (SIM_FRAMES + 2 * FRAME_FRAMES) as f64 * FRAME_DT
}

/// Second measurements of the rows a comparison called a regression. See `perf::reconfirm` for why.
#[derive(Default)]
pub(crate) struct Retry {
    /// `(name, median)` in the units the compare table quotes: nanoseconds for a micro row,
    /// milliseconds for a replay row.
    pub second: Vec<(String, f64)>,
    /// Windows whose fingerprint came back different from the first run's. It must be empty: the
    /// save point is the same file and the sequence is the same sequence, so anything here is a
    /// determinism bug and not a measurement.
    pub fingerprint_drift: Vec<String>,
}

/// Measure the named rows once more, so that a regression has to happen twice to be believed.
///
/// The micro rows are the cheap half: the benchmark bodies are still alive - the drift probe keeps
/// them - and re-running one costs what it cost the first time, a fraction of a second. No spin-up
/// is needed or wanted, because by now the process has been working for the best part of ten
/// seconds and is as warm as it is going to get.
///
/// The replay rows are the expensive half and cannot be re-measured one at a time. A window is only
/// the window it is when the two before it have run on the same app from the same save point, so a
/// single regressed window means reloading the save point and playing the whole sequence again -
/// once, however many of the three regressed, at about six seconds. The rows that did not regress
/// are re-measured too and their second readings are simply not used, which is the price of the
/// sequence being what makes any of them comparable.
pub(crate) fn remeasure(
    names: &[String],
    benches: &mut [crate::perf::harness::Bench<'_>],
    budget_for: &dyn Fn(&str) -> crate::perf::harness::Budget,
    first: &[ReplayResult],
) -> Result<Retry, String> {
    let mut retry = Retry::default();
    for bench in benches.iter_mut().filter(|b| names.contains(&b.name)) {
        let stats = crate::perf::harness::measure(&mut bench.body, &budget_for(&bench.name));
        retry.second.push((bench.name.clone(), stats.median_ns));
    }
    if names.iter().any(|name| first.iter().any(|result| &result.name == name)) {
        let mut point = restore()?;
        for result in windows(&mut point.app) {
            if let Some(before) = first.iter().find(|r| r.name == result.name)
                && before.fingerprint != result.fingerprint
            {
                retry.fingerprint_drift.push(result.name.clone());
            }
            retry.second.push((result.name.clone(), result.median_ms));
        }
    }
    Ok(retry)
}
