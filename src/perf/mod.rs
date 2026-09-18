//! The performance harness: what it measures, what it does not, and how to use it to decide
//! whether an idea made the app faster.
//!
//! It lives inside the binary because the crate has no library target and is not about to grow one
//! for this. `main` looks at its arguments before it opens anything, and `--perf` anywhere on the
//! command line runs the harness headless and exits with its status code without ever creating a
//! window:
//!
//! ```text
//! cargo run --release --target-dir target/probe -- --perf
//! ```
//!
//! Build it in release. A debug build runs this code at a fraction of speed and the numbers are not
//! comparable with anything; the harness says so loudly and marks the JSON, but it cannot stop you.
//!
//! # The three tiers
//!
//! **`micro`** times one call of one thing against state built once and shared: the geodesic
//! acceleration kernel, `ray_rhs`, one Dormand-Prince substep, `NullRay::step` for three rays in
//! very different places, `SignalField::advance`, `detect_receptions`, `step_back` and one
//! emission over a prebuilt steady-state field, one observer step, `Observer::event_at`,
//! `build_past_cone`, and each of the three canvases painted once through a headless egui pass.
//! What it tells you is which call got faster. What it cannot tell you is whether that matters:
//! nothing here knows how many times a frame makes the call, and a kernel twice as fast that is
//! called twice as often is a wash.
//!
//! **`sim`** plays a scripted scenario through the real `SpacetimeApp` at a fixed 1/60 M per frame
//! and times the whole simulation pipeline - observers, rays, emission, reception detection - with
//! no ui at all.
//!
//! **`frame`** plays the same scenarios through the real `SpacetimeApp::ui` in a headless
//! `egui::Context` at 1280x820 points, then tessellates what it produced, and times the two
//! separately. This is *CPU frame cost*. It excludes the GPU upload of the meshes, the present and
//! vsync, so it is a lower bound on what a window costs and an upper bound on what the CPU can be
//! blamed for.
//!
//! None of the three measures start-up, font loading, window creation or anything about the
//! machine's display. All three are single-threaded, which is what the app is.
//!
//! # `--quick`: the everyday regression check
//!
//! The full suite takes about nine minutes, and nearly all of it is spent *reaching* a steady state
//! rather than measuring one. `--quick` skips the reaching. It loads the `isco-pair-64` steady state
//! from a save point in seventy-five milliseconds - `crate::save` restores a run to the bit - and
//! spends the ten seconds that are left measuring. See `crate::perf::quick`.
//!
//! ```text
//! cargo perf-baseline    # --perf --quick --save check    : this build is the reference
//! cargo perf-check       # --perf --quick --compare check : and is this one still as fast?
//! ```
//!
//! Those two aliases are in `.cargo/config.toml` and are the whole workflow. The save point builds
//! itself the first time it is wanted, which costs a few seconds once; `--perf --make-savepoints`
//! builds it on demand.
//!
//! **What a quick run keeps from the full suite.** All seventeen micro-benchmarks, against the same
//! shared field, at fifteen samples instead of thirty. Simulation cost per frame. Full CPU frame
//! cost with the ui/tessellate split, for *both* painters - the flat Global chart and the 2D+1
//! volume. The workload counters. The end-of-window fingerprint, which is exact given the save
//! point. A quick comparison can fail the process, on a 10% gate rather than the full suite's 5%.
//!
//! **A regression has to happen twice.** On this machine an unchanged build put one row of the
//! twenty - a different one each time - uniformly 13 to 19 percent slow about one run in four, with
//! that benchmark's own sample spread still reading under one percent: the whole sixty-millisecond
//! window ran slow, so there is no outlier for the spread to see and nothing for the noise band to
//! widen itself by. It is consistent with the core stepping down a clock bin or the thread being
//! moved between cores, which is what Windows' Balanced power plan does. What such a transient
//! cannot do is land on the same row twice running, so a quick comparison that finds regressions
//! measures exactly those rows again and keeps a regression only if the second reading is also
//! outside the band. This is not a wider band - the threshold is untouched - and it costs nothing on
//! a clean run: a fraction of a second if the rows are micro-benchmarks, and about six seconds if a
//! replay window has to be played again, which it must be as a sequence or it is not the same
//! window. A `--save` always writes the first measurement and never a mixture.
//!
//! **What only the full suite has.** Cost growth over a run: a quick window is a few M at a plateau
//! and its per-10-M series is flat by construction, while a scenario replay is thirty to a hundred M
//! from t = 0 and shows the climb. The cap of 128 wavefronts. The `default-infall` layout. The
//! far-branch freeze, with its u^t of order 1e10. And best-of-several runs, which is a stronger
//! noise figure than one window's chunk medians. An idea that could change the *shape* of a run
//! rather than the price of a frame is measured there, not here.
//!
//! # The command line
//!
//! ```text
//! --perf [micro|sim|frame]...   which tiers to run (default: all three)
//! --filter <substr>             only benchmarks and scenarios whose name contains substr
//! --quick                       the ten-second check from a save point; see above
//! --make-savepoints             build the save points --quick measures, then exit
//! --once                        one pass of each replay instead of the best of several
//! --list                        print the names and what each one measures, then exit
//! --save <name>                 write the results to target/perf/<name>.json
//! --compare <name>              compare against target/perf/<name>.json; exit 1 on a regression
//! --threshold <percent>         regression threshold, default 5, or 10 for --quick
//! --help                        this text
//! ```
//!
//! A `--compare` fails the process on a regression when both sides are quick runs of the same save
//! point, or when neither is quick. It never fails when one side is quick and the other is not:
//! those two runs measured different lengths of different parts of a run with different sample
//! counts, and the ratio between their medians is not a number about the code. The table is still
//! printed, because a large move is worth seeing however it was arrived at.
//!
//! Two quick runs of *different* save points are warned about for the same reason and then judged
//! anyway: the warning says the workloads differ, and what to do about it is rebuild the baseline.
//!
//! `--once` is for the question a refactor asks, which is not "how fast" but "is the physics still
//! bit-identical". A fingerprint is the same on every pass, so replaying each scenario three times
//! to steady the timings buys it nothing: `--perf sim --once --compare <name>` plays every scenario
//! to its full length once, about forty seconds instead of two minutes, and the fingerprints it
//! prints are exactly the ones a full run would. Its timings are a single pass, so it prints its
//! verdicts and never exits 1 on them.
//!
//! `--save` and `--compare` combine, which is what an A/B session actually does: measure, compare
//! with the baseline, and keep the result under a new name. `target/perf` is resolved against the
//! current working directory, so the harness is meant to be run from the repository root, which is
//! where `cargo run` puts you. `target/` is gitignored, and that is deliberate: a baseline is a
//! statement about one machine on one afternoon and is worthless on anybody else's, so baselines
//! are local by design and are never committed.
//!
//! # The A/B workflow
//!
//! ```text
//! git stash                 # or: git checkout <base>
//! cargo run --release --target-dir target/probe -- --perf --save base
//! git stash pop             # or: apply the idea
//! cargo run --release --target-dir target/probe -- --perf --compare base
//! ```
//!
//! That is the full suite, for an idea worth nine minutes. The quick pair above is the same shape at
//! a twentieth of the cost, and is what a working afternoon actually runs.
//!
//! `--target-dir target/probe` is not optional in practice: the app may be running from
//! `target/release`, which locks the executable, and a build into the same directory then fails.
//!
//! Advice, in the order it matters:
//!
//! * Close the app and anything else heavy. A browser rendering video in the background moves these
//!   numbers by more than most optimisations do, and so does the editor's own `rust-analyzer`: a
//!   `cargo check` it starts after an edit runs on four or five cores for half a minute, and a quick
//!   run taken inside that window reads ten to twenty percent slow on whichever benchmarks it
//!   overlapped. Wait for the editor to go quiet before believing a ten-second check.
//! * On Windows, measure on the High performance power plan rather than Balanced. Balanced parks
//!   cores and moves the thread between them, and a benchmark measured across one of those moves
//!   comes out uniformly 13 to 17 percent slow with its own sample spread still reading under one
//!   percent - which is to say the harness cannot tell from inside that it happened. It is the one
//!   remaining thing that makes a quick check cry wolf.
//! * Run on mains power. A laptop on battery throttles, and it throttles *more* the longer the
//!   suite runs, which looks exactly like a regression in whatever runs last.
//! * Read the drift line at the foot of the report. It re-runs the first micro-benchmark after
//!   everything else and prints how far it has moved: a few percent is normal, and anything larger
//!   means the machine changed under the measurement and the run should be thrown away.
//! * Compare the effect you are looking for against the noise band in the compare table. If the
//!   band is wider than the effect, the run cannot answer the question - take more samples by not
//!   using `--quick`, or make the change bigger before measuring it.
//!
//! # The fingerprint
//!
//! Every replay ends with a 64-bit FNV-1a hash over the bit patterns of the simulated state: the
//! clock, both worldlines, every ray of every pulse of both transmissions, and every arrival
//! recorded. `--compare` prints, per scenario, whether that hash matches the baseline's. A match
//! means the physics is bit-identical and the two sets of timings are like for like. A mismatch is
//! reported and not failed - changing the physics is allowed - but it means the timings are of two
//! different amounts of work and the ratio between them is not an optimisation result.
//!
//! This is the feature that lets "this change is purely a speed-up" be checked rather than
//! asserted.

pub(crate) mod harness;
pub(crate) mod micro;
pub(crate) mod quick;
pub(crate) mod replay;

use std::io::Write;
use std::path::PathBuf;

use harness::{Budget, Stats, human_ns};
use replay::{Mode, ReplayResult};

use crate::stamp::{epoch_seconds, git_dirty, git_short_hash, utc_iso};

/// Format version of the JSON a `--save` writes. `--compare` refuses a file from another version
/// rather than misreading it: the fields are not stable and a silent mismatch would compare the
/// wrong numbers.
const FORMAT: u32 = 1;

/// The rectangle the micro tier's paint benchmarks draw a canvas into.
///
/// The app works its canvas sizes out from the window every frame, so there is no constant to
/// copy; these are that arithmetic applied by hand to the 1280x820 window the frame tier uses. The
/// 300-point control panel and egui's panel margins leave the central column about 964 points, of
/// which `SpacetimeApp::ui` gives the foliation view 53% and the equatorial view the rest less a
/// 12-point separator; the top bar and the 68-point HUD leave about 640 points of height.
///
/// Getting this right matters more than it looks. A canvas painted at the full 1280 points of the
/// window - which is what a bare `run_ui` hands you if you ask for `available_width` - draws two
/// and a half times the picture, and the tier then reports a single canvas costing more than the
/// whole frame does in the frame tier, which is nonsense that would be believed. What a benchmark
/// needs is a *fixed* size; what it needs it to be is roughly the real one.
pub(crate) const CANVAS_HEIGHT: f32 = 640.0;

/// Width of the foliation column: the (t, r) chart and the 2D+1 volume.
pub(crate) const LEFT_CANVAS_WIDTH: f32 = 511.0;

/// Width of the equatorial column.
pub(crate) const RIGHT_CANVAS_WIDTH: f32 = 441.0;

/// The window the frame tier lays out, in points, at one physical pixel per point.
const SCREEN: (f32, f32) = (1280.0, 820.0);

/// One 60 Hz frame, in milliseconds: the budget a frame-mode median is quoted against.
const FRAME_BUDGET_MS: f64 = 1000.0 / 60.0;

/// A headless egui context with the app's real fonts in it.
///
/// The fonts matter to the measurement. egui's bundled face has no subscripts or Greek, and the
/// canvases are full of r₋, τ and Ω: laying out a string in a fallback-heavy font stack is not the
/// same work as laying it out in one face, and `FontDefinitions::empty()` - which the unit tests
/// use, because they only read the strings - would measure neither.
pub(crate) fn headless_context() -> egui::Context {
    let ctx = egui::Context::default();
    crate::install_fonts(&ctx);
    ctx.set_pixels_per_point(1.0);
    ctx
}

/// One frame's raw input: a fixed screen rectangle, no events, and a clock advanced by the caller.
///
/// The time has to advance or egui's animations - the fade of a hovered button, the open of a combo
/// box - never finish, and a frame that is repainting an unfinished animation is doing different
/// work from one that is not.
pub(crate) fn raw_input(time: f64) -> egui::RawInput {
    egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(SCREEN.0, SCREEN.1),
        )),
        time: Some(time),
        ..Default::default()
    }
}

/// One micro-benchmark's result as it goes into the JSON.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct MicroResult {
    pub name: String,
    pub stats: Stats,
}

/// A whole run, saved and compared as one document.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct Report {
    pub format: u32,
    /// When the run finished, UTC, so two baselines can be told apart by eye.
    pub timestamp_utc: String,
    pub epoch_seconds: u64,
    /// The short hash the run was built from, and whether the tree had uncommitted changes in it.
    /// Both are best-effort: no git, no problem, the fields say "unknown".
    pub git_hash: String,
    pub git_dirty: bool,
    /// `PROCESSOR_IDENTIFIER` where the environment has it, which on Windows is the CPU's own
    /// description. Comparing two reports from different CPUs is meaningless and this is what lets
    /// that be noticed.
    pub cpu: String,
    pub os: String,
    pub quick: bool,
    /// Which save point a `--quick` run measured, and therefore what workload its timings are of.
    ///
    /// None for a full run, and also for a quick run written before save points existed - which is
    /// what `serde(default)` is here for, and what tells the compare step that the two sides cannot
    /// be judged against each other.
    #[serde(default)]
    pub savepoint: Option<quick::SavepointId>,
    /// True if each replay was played once rather than best-of-several: the fingerprints are as
    /// good as any, the timings have no spread behind them. Absent from reports written before the
    /// flag existed, which were never single-pass.
    #[serde(default)]
    pub once: bool,
    /// True if the harness was built with debug assertions, in which case every number in the
    /// document is worthless for comparison and this is the flag that says so.
    pub debug_assertions: bool,
    /// How far the first micro-benchmark moved between the start of the run and the end of it, as a
    /// percentage. None when the micro tier did not run.
    pub drift_percent: Option<f64>,
    pub micro: Vec<MicroResult>,
    pub replay: Vec<ReplayResult>,
}

/// Which tiers a run was asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tier {
    Micro,
    Sim,
    Frame,
}

/// The command line, parsed.
struct Options {
    tiers: Vec<Tier>,
    filter: Option<String>,
    quick: bool,
    make_savepoints: bool,
    once: bool,
    list: bool,
    save: Option<String>,
    compare: Option<String>,
    /// The regression threshold as a fraction, so 5% is 0.05, or None for the default - which is
    /// not one number: see `default_threshold`. An explicit `--threshold` wins either way, which is
    /// why this is an option rather than a number filled in while parsing.
    threshold: Option<f64>,
    help: bool,
}

/// The regression threshold a run is judged on when nobody asked for one.
///
/// 5% for the full suite, which takes thirty samples of every benchmark and the best of three runs
/// of every replay and can support a claim that small. 10% for the quick tier, which takes half the
/// samples and one window: its spreads come out at two to four percent rather than under two, and a
/// gate set at 5% would fire on the machine rather than on the code. The band is the larger of this
/// and three times the two measured spreads in any case - see `noise_band` - so this is a floor and
/// a noisy quick run is judged more leniently still.
fn default_threshold(quick: bool) -> f64 {
    if quick { 0.10 } else { 0.05 }
}

impl Options {
    fn wants(&self, tier: Tier) -> bool {
        self.tiers.contains(&tier)
    }

    fn matches(&self, name: &str) -> bool {
        self.filter.as_ref().is_none_or(|f| name.contains(f.as_str()))
    }
}

/// Parse the arguments `main` was given. Everything before `--perf` is ignored, which is what lets
/// `cargo run -- --perf ...` work without the harness caring how it was invoked.
fn parse(args: &[String]) -> Result<Options, String> {
    let mut opts = Options {
        tiers: Vec::new(),
        filter: None,
        quick: false,
        make_savepoints: false,
        once: false,
        list: false,
        save: None,
        compare: None,
        threshold: None,
        help: false,
    };
    let mut i = 0;
    let next = |i: &mut usize, flag: &str| -> Result<String, String> {
        *i += 1;
        args.get(*i).cloned().ok_or_else(|| format!("{flag} needs a value"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--perf" => {
                // The tier names follow `--perf` directly and stop at the next flag.
                while let Some(name) = args.get(i + 1).filter(|a| !a.starts_with("--")) {
                    opts.tiers.push(match name.as_str() {
                        "micro" => Tier::Micro,
                        "sim" => Tier::Sim,
                        "frame" => Tier::Frame,
                        other => return Err(format!("unknown tier `{other}`")),
                    });
                    i += 1;
                }
            }
            "--filter" => opts.filter = Some(next(&mut i, "--filter")?),
            "--save" => opts.save = Some(next(&mut i, "--save")?),
            "--compare" => opts.compare = Some(next(&mut i, "--compare")?),
            "--threshold" => {
                let value = next(&mut i, "--threshold")?;
                let percent: f64 =
                    value.parse().map_err(|_| format!("--threshold wants a number, got `{value}`"))?;
                opts.threshold = Some(percent / 100.0);
            }
            "--quick" => opts.quick = true,
            "--make-savepoints" => opts.make_savepoints = true,
            "--once" => opts.once = true,
            "--list" => opts.list = true,
            "--help" | "-h" => opts.help = true,
            other => return Err(format!("unknown argument `{other}`")),
        }
        i += 1;
    }
    if opts.tiers.is_empty() {
        opts.tiers = vec![Tier::Micro, Tier::Sim, Tier::Frame];
    }
    Ok(opts)
}

/// The `--help` text, which is the module documentation above said once more for somebody who has
/// the binary and not the source.
fn print_help() {
    println!(
        "\
Black Hole Lab performance harness.

  cargo run --release --target-dir target/probe -- --perf [options]

  --perf [micro|sim|frame]...   which tiers to run (default: all three). Under --quick, naming
                                sim or frame selects which of the three windows are reported
                                rather than which are played: they run in sequence on one app
  --filter <substr>             only benchmarks and scenarios whose name contains substr
  --quick                       the ten-second regression check, measured from a save point
                                instead of played to from t = 0. See below
  --make-savepoints             build the save points --quick measures, then exit
  --once                        one pass of each replay instead of the best of several: all a
                                fingerprint check needs (--perf sim --once --compare <name>,
                                about 40 s). Never exits 1 on its timings
  --list                        print the names and what each one measures, then exit
  --save <name>                 write the results to target/perf/<name>.json
  --compare <name>              compare against target/perf/<name>.json; exit 1 on a regression
  --threshold <percent>         regression threshold, default 5, or 10 for --quick
  --help                        this text

Tiers:
  micro   one call of one hot spot, against state built once and shared.
  sim     a scripted run of the real app at a fixed 1/60 M per frame, simulation only, no ui.
  frame   the same run through the real ui in a headless egui context, plus tessellation.
          CPU frame cost: no GPU upload, no present, no vsync.

The everyday check, about ten seconds:

  cargo perf-baseline       # --perf --quick --save check    : this build is the reference
  cargo perf-check          # --perf --quick --compare check : and is this one still as fast?

  Both aliases are in .cargo/config.toml. The save point builds itself the first time it is
  wanted, which costs a few seconds once.

  --quick keeps: all 17 hot spots against the same shared field, at 15 samples instead of 30;
  simulation cost per frame; full CPU frame cost with the ui/tessellate split for both
  painters, the flat Global chart and the 2D+1 volume; the workload counters; and the
  fingerprint of the state each window ended in. It can and does exit 1.

  A regression has to happen twice. An unchanged build puts one row in twenty uniformly 13 to
  19 percent slow about one run in four - the whole 60 ms window runs slow, so the row's own
  spread stays under 1% and the noise band cannot see it - and a transient like that does not
  land on the same row twice. So a quick compare that finds regressions measures those rows
  again and keeps only the ones that reproduce. Nothing on a clean run; a fraction of a second
  for micro rows; about 6 s if a replay window must be played again, which it must be as a
  sequence. --save always writes the first measurement, never a mixture.

  Only the full suite has: cost growth over a run, the cap of 128 wavefronts, the
  default-infall layout, the far-branch freeze, and best-of-several runs. An idea that could
  change the shape of a run rather than the price of a frame belongs there.

The A/B workflow, for an idea worth nine minutes:

  git stash                 # or: git checkout <base>
  cargo run --release --target-dir target/probe -- --perf --save base
  git stash pop             # or: apply the idea
  cargo run --release --target-dir target/probe -- --perf --compare base

  Close the app and other heavy programs first, and run on mains power: a throttling laptop
  looks exactly like a regression in whatever the suite runs last. Read the drift line at the
  foot of the report - it re-runs the first micro-benchmark at the end and says how far it
  moved - and if the noise band in the compare table is wider than the effect you are looking
  for, the run cannot answer your question: drop --quick, or make the change bigger.

  Baselines live in target/perf, which is gitignored on purpose: a baseline is a statement
  about one machine and is worthless on any other. So are the save points, in
  target/perf/savepoints.

A comparison of a quick run with a full one is a warning and never a failure: they measured
different lengths of different parts of a run.

Every replay prints a 64-bit fingerprint of the state it ended in. --compare says whether it
matches the baseline's, which is how a claim to have changed only the speed gets checked."
    );
}

/// Run the harness. The return value is the process's exit status: 0 for a clean run, 1 for a
/// regression against a baseline, 2 for a command line or file that could not be used.
pub(crate) fn run(args: &[String]) -> i32 {
    let opts = match parse(args) {
        Ok(opts) => opts,
        Err(message) => {
            eprintln!("--perf: {message}");
            eprintln!("try: --perf --help");
            return 2;
        }
    };
    if opts.help {
        print_help();
        return 0;
    }

    if cfg!(debug_assertions) {
        eprintln!("======================================================================");
        eprintln!("  WARNING: this is a build with debug assertions on.");
        eprintln!("  Every number below is of checked, unoptimised arithmetic and is NOT");
        eprintln!("  comparable with a release build or with a saved baseline.");
        eprintln!("  Build with --release.");
        eprintln!("======================================================================");
    }

    if opts.make_savepoints {
        return match quick::make_savepoints() {
            Ok(()) => 0,
            Err(message) => {
                eprintln!("--make-savepoints: {message}");
                2
            }
        };
    }

    // A quick run measures one restored state, and both of its tiers measure the same one: the micro
    // fixtures are taken off this app and then the replay windows are played on it. Loading comes
    // first for that reason, and because a save point that will not load is a reason to stop rather
    // than to measure something else.
    let mut restored = if opts.quick && !opts.list {
        match quick::restore() {
            Ok(point) => {
                println!("Save point: {}", point.id.line());
                // Before anything is timed, and before the fixtures are even taken off it: the
                // first benchmark on the list must not be the one that pays for a cold core.
                quick::spin_up(&point.app);
                Some(point)
            }
            Err(message) => {
                eprintln!("--quick: {message}");
                return 2;
            }
        }
    } else {
        None
    };

    // The micro tier's benchmarks are built from the shared fixtures, so even `--list` has to build
    // them - by the short ramp, which takes well under a second - rather than keep a second copy of
    // the list that could drift out of step with the first.
    let needs_fixtures = opts.wants(Tier::Micro);
    let fixtures = needs_fixtures.then(|| match restored.as_ref() {
        Some(point) => micro::Fixtures::from_app(&point.app),
        None => {
            if !opts.list {
                println!("Building the shared field ...");
            }
            micro::Fixtures::ramped(if opts.list {
                micro::FIXTURE_UNTIL_SHORT
            } else {
                micro::FIXTURE_UNTIL
            })
        }
    });

    if opts.list {
        if let Some(fx) = fixtures.as_ref() {
            println!("micro:");
            for bench in micro::benches(fx) {
                println!("  {:<28} {}", bench.name, bench.what);
            }
        }
        for tier in [Tier::Sim, Tier::Frame] {
            if !opts.wants(tier) {
                continue;
            }
            let mode = if tier == Tier::Sim { Mode::Sim } else { Mode::Frame };
            println!("{}:", mode.label());
            for scenario in replay::scenarios() {
                if scenario.frame_only && mode == Mode::Sim {
                    continue;
                }
                println!("  {:<28} {}", format!("{}/{}", mode.label(), scenario.name), scenario.what);
            }
        }
        return 0;
    }

    // The budget one benchmark gets. A quick run pays for the paint benchmarks by the sample rather
    // than by the iteration - one pass is already milliseconds - so those three take fewer of them;
    // everything else is a kernel that gets its fifteen.
    let budget_for = |name: &str| match (opts.quick, name.starts_with("paint/")) {
        (false, _) => Budget::full(),
        (true, false) => Budget::quick(),
        (true, true) => Budget::quick_paint(),
    };
    let mut micro_results: Vec<MicroResult> = Vec::new();
    let mut drift_percent = None;

    // The benchmarks are kept alive past the tier that runs them, because the drift probe at the
    // foot of the report re-runs the first of them once everything else has had its turn on the
    // machine.
    let mut benches = fixtures.as_ref().map(micro::benches).unwrap_or_default();
    if fixtures.is_some() {
        println!("\n== micro =============================================================");
        println!("{:<28} {:>12} {:>12} {:>9} {:>10}", "benchmark", "median", "min", "spread", "iters");
        for bench in benches.iter_mut() {
            if !opts.matches(&bench.name) {
                continue;
            }
            let stats = harness::measure(&mut bench.body, &budget_for(&bench.name));
            println!(
                "{:<28} {:>12} {:>12} {:>8.1}% {:>10}",
                bench.name,
                human_ns(stats.median_ns),
                human_ns(stats.min_ns),
                100.0 * stats.spread,
                stats.iters
            );
            micro_results.push(MicroResult { name: bench.name.clone(), stats });
        }
    }

    let mut replay_results: Vec<ReplayResult> = Vec::new();
    let wants_windows = opts.wants(Tier::Sim) || opts.wants(Tier::Frame);
    if let Some(point) = restored.as_mut().filter(|_| wants_windows) {
        // The quick tier's windows are one fixed composition rather than a tier selection: the three
        // run one after another on the restored app, so each one's starting state is the one before
        // it finishing, and leaving one out would change what the others measure. Naming `sim` or
        // `frame`, or filtering, therefore chooses what is *reported* here rather than what is
        // played; naming neither - `--perf micro --quick` - skips the windows altogether.
        println!("\n== quick =============================================================");
        println!(
            "three windows in sequence on the save point, {:.1} M of it in all",
            quick::span_m()
        );
        for result in quick::windows(&mut point.app) {
            if !opts.matches(&result.name) {
                continue;
            }
            print_replay(&result);
            replay_results.push(result);
        }
    } else if restored.is_none() {
        for (tier, mode) in [(Tier::Sim, Mode::Sim), (Tier::Frame, Mode::Frame)] {
            if !opts.wants(tier) {
                continue;
            }
            let mut printed_header = false;
            for scenario in replay::scenarios() {
                if scenario.frame_only && mode == Mode::Sim {
                    continue;
                }
                let name = format!("{}/{}", mode.label(), scenario.name);
                if !opts.matches(&name) {
                    continue;
                }
                if !printed_header {
                    println!("\n== {} ===========================================================", mode.label());
                    printed_header = true;
                }
                let result = replay::replay(&scenario, mode, opts.once);
                print_replay(&result);
                replay_results.push(result);
            }
        }
    }

    // The drift probe: the first micro-benchmark measured again, now that the replays have run the
    // machine hot. If it has moved, nothing measured in between is trustworthy either.
    if let Some(first) = benches.first_mut()
        && let Some(before) = micro_results.iter().find(|r| r.name == first.name)
    {
        let again = harness::measure(&mut first.body, &budget_for(&first.name));
        drift_percent = Some(100.0 * (again.median_ns / before.stats.median_ns - 1.0));
    }

    if let Some(drift) = drift_percent {
        println!(
            "\ndrift: the first micro-benchmark re-run after everything else is {drift:+.1}% \
             against its first measurement."
        );
        if drift.abs() > 5.0 {
            println!("       That is more than noise. The machine changed under the run; discard it.");
        }
    }

    let report = Report {
        format: FORMAT,
        timestamp_utc: utc_iso(epoch_seconds()),
        epoch_seconds: epoch_seconds(),
        git_hash: git_short_hash(),
        git_dirty: git_dirty(),
        cpu: std::env::var("PROCESSOR_IDENTIFIER").unwrap_or_else(|_| "unknown".to_string()),
        os: std::env::consts::OS.to_string(),
        quick: opts.quick,
        savepoint: restored.map(|point| point.id),
        once: opts.once,
        debug_assertions: cfg!(debug_assertions),
        drift_percent,
        micro: micro_results,
        replay: replay_results,
    };

    // What a `--save` writes is the first measurement and only ever the first measurement. The retry
    // below re-measures a handful of rows to decide whether a regression was real, and none of what
    // it measures goes into this document or into any baseline: a saved report has to be one run of
    // one build, taken in one pass, or two of them cannot be compared. Saving happens here, before
    // the comparison, so there is no path on which the two could be mixed.
    if let Some(name) = opts.save.as_ref()
        && let Err(message) = save(&report, name)
    {
        eprintln!("--save: {message}");
        return 2;
    }

    if let Some(name) = opts.compare.as_ref() {
        let threshold = opts.threshold.unwrap_or_else(|| default_threshold(opts.quick));
        let base = match load(name) {
            Ok(base) => base,
            Err(message) => {
                eprintln!("--compare: {message}");
                return 2;
            }
        };
        let mut comparison = compare(&base, &report, threshold);
        let mut retried = Vec::new();
        // The retry is the quick tier's alone. The full suite already takes thirty samples of every
        // benchmark and the best of two or three passes of every replay, which is a stronger answer
        // to the same question and one it has already paid for.
        let regressed = comparison.regressed();
        if opts.quick && comparison.why_not_failable.is_none() && !regressed.is_empty() {
            println!(
                "\nre-measuring {}",
                if regressed.len() == 1 {
                    "the regressed row to see whether it reproduces ...".to_string()
                } else {
                    format!("the {} regressed rows to see which reproduce ...", regressed.len())
                }
            );
            match quick::remeasure(&regressed, &mut benches, &budget_for, &report.replay) {
                Ok(again) => {
                    for name in &again.fingerprint_drift {
                        println!(
                            "  !!! {name} ended on a different state this time. The save point and \
                             the sequence are the same, so this is not noise: something in the step \
                             is not deterministic and every comparison this harness has ever made \
                             is in doubt. Investigate before reading anything below."
                        );
                    }
                    retried = reconfirm(&mut comparison.rows, &again.second);
                }
                // A retry that cannot run is not a reason to invent a verdict either way, so the
                // first measurement's verdicts stand and the run says why.
                Err(message) => println!("  could not re-measure ({message}); keeping the first verdicts"),
            }
        }
        return conclude(&comparison, &retried);
    }
    0
}

/// Where a baseline lives, relative to the current working directory: `target/perf/<name>.json`.
fn baseline_path(name: &str) -> PathBuf {
    PathBuf::from("target").join("perf").join(format!("{name}.json"))
}

fn save(report: &Report, name: &str) -> Result<(), String> {
    let path = baseline_path(name);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    let json = serde_json::to_string_pretty(report).map_err(|e| e.to_string())?;
    let mut file = std::fs::File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    file.write_all(json.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))?;
    file.write_all(b"\n").map_err(|e| format!("{}: {e}", path.display()))?;
    println!("\nsaved to {}", path.display());
    Ok(())
}

fn load(name: &str) -> Result<Report, String> {
    let path = baseline_path(name);
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let report: Report =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if report.format != FORMAT {
        return Err(format!(
            "{} was written by format {} and this build reads format {FORMAT}; re-measure the \
             baseline",
            path.display(),
            report.format
        ));
    }
    Ok(report)
}

/// What a comparison of one name concluded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    Regression,
    Improvement,
    Same,
}

impl Verdict {
    fn label(self) -> &'static str {
        match self {
            Self::Regression => "REGRESSION",
            Self::Improvement => "IMPROVEMENT",
            Self::Same => "same within noise",
        }
    }
}

/// How far apart two medians have to be before the difference is called rather than shrugged at.
///
/// The threshold is a floor and not the whole rule. Two runs of the same code differ by their own
/// noise, and a benchmark whose samples are spread by 4% cannot support a claim about a 5%
/// difference; the band is therefore the larger of the asked-for threshold and three times the two
/// spreads added together. Added rather than combined in quadrature, deliberately: the cost of
/// being conservative is missing a small regression, and the cost of not being is crying wolf at
/// every run, which is how a check like this stops being read.
fn noise_band(base_spread: f64, new_spread: f64, threshold: f64) -> f64 {
    (3.0 * (base_spread + new_spread)).max(threshold)
}

/// The verdict on one ratio of medians against one band.
fn verdict(ratio: f64, band: f64) -> Verdict {
    if !ratio.is_finite() {
        return Verdict::Same;
    }
    if ratio > 1.0 + band {
        Verdict::Regression
    } else if ratio < 1.0 - band {
        Verdict::Improvement
    } else {
        Verdict::Same
    }
}

/// What a report is, as far as the pass-or-fail rule is concerned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RunKind {
    quick: bool,
    /// Whether the run recorded which save point it measured, which is what makes it a quick run of
    /// the tier this build has rather than of the smoke-test `--quick` that came before it.
    savepoint: bool,
    once: bool,
}

impl RunKind {
    fn of(report: &Report) -> Self {
        Self { quick: report.quick, savepoint: report.savepoint.is_some(), once: report.once }
    }
}

/// Why a comparison of these two runs may not fail the process, or None when it may.
///
/// Three runs can be compared and only two shapes of comparison can fail. Two full runs can: thirty
/// samples of every benchmark and the best of three passes of every replay. Two quick runs of a save
/// point can: they measured the same restored state, the same windows, the same number of frames, at
/// the same sample counts, and the 10% gate is set against the spreads that arrangement produces.
///
/// Everything else is printed and shrugged at. A quick run against a full one measured different
/// parts of a run at different lengths, and the ratio of their medians is not a number about the
/// code. A quick run with no save point recorded is from a build whose `--quick` was a smoke test
/// with five-M replays in it. And a single-pass run's replays have no spread behind them at all, so
/// their band is the bare threshold and one descheduled pass would cross it.
fn why_not_failable(base: RunKind, new: RunKind) -> Option<&'static str> {
    if base.quick != new.quick {
        return Some(
            "one side of this comparison is a --quick run and the other is not. They measured \
             different lengths of different parts of a run, so the ratios above are not a verdict \
             on the code. Compare like with like.",
        );
    }
    if base.quick && !(base.savepoint && new.savepoint) {
        return Some(
            "one side of this comparison is a --quick run that did not record a save point, so \
             what it measured cannot be established. Re-measure the baseline.",
        );
    }
    if base.once || new.once {
        return Some(
            "one side of this comparison is a --once run, whose replay timings are a single pass. \
             The fingerprints above are what it is for.",
        );
    }
    None
}

/// One row of the compare table: a name measured on both sides.
struct CompareRow {
    name: String,
    base: f64,
    new: f64,
    unit: &'static str,
    band: f64,
    verdict: Verdict,
}

/// A comparison, printed, with its verdicts still open to a retry.
struct Comparison {
    rows: Vec<CompareRow>,
    /// None when this comparison is allowed to fail the process. See `why_not_failable`.
    why_not_failable: Option<&'static str>,
}

impl Comparison {
    fn regressed(&self) -> Vec<String> {
        self.rows
            .iter()
            .filter(|r| r.verdict == Verdict::Regression)
            .map(|r| r.name.clone())
            .collect()
    }
}

/// Print the comparison table and the fingerprints, and hand back the verdicts.
fn compare(base: &Report, new: &Report, threshold: f64) -> Comparison {
    println!("\n== compare against the baseline =======================================");
    println!(
        "baseline: {} git {}{} on {}",
        base.timestamp_utc,
        base.git_hash,
        if base.git_dirty { "-dirty" } else { "" },
        base.cpu
    );
    println!(
        "this run: {} git {}{} on {}",
        new.timestamp_utc,
        new.git_hash,
        if new.git_dirty { "-dirty" } else { "" },
        new.cpu
    );
    if base.cpu != new.cpu || base.os != new.os {
        println!("WARNING: the two runs are from different machines. The comparison is meaningless.");
    }
    if base.quick != new.quick {
        println!("WARNING: one of these is a --quick run and the other is not. Different sample counts.");
    }
    match (base.savepoint.as_ref(), new.savepoint.as_ref()) {
        (Some(old), Some(now)) if old == now => println!("save point: {}", now.line()),
        (Some(old), Some(now)) => println!(
            "WARNING: different workload - timings are not like for like.\n  \
             baseline measured {}\n  this run measured {}",
            old.line(),
            now.line()
        ),
        _ => {}
    }
    if base.debug_assertions || new.debug_assertions {
        println!("WARNING: one of these was built with debug assertions on.");
    }

    let mut rows: Vec<CompareRow> = Vec::new();
    let mut added: Vec<String> = Vec::new();
    let mut removed: Vec<String> = Vec::new();

    for result in &new.micro {
        match base.micro.iter().find(|b| b.name == result.name) {
            Some(old) => {
                let band = noise_band(old.stats.spread, result.stats.spread, threshold);
                let ratio = result.stats.median_ns / old.stats.median_ns;
                rows.push(CompareRow {
                    name: result.name.clone(),
                    base: old.stats.median_ns,
                    new: result.stats.median_ns,
                    unit: "ns",
                    band,
                    verdict: verdict(ratio, band),
                });
            }
            None => added.push(result.name.clone()),
        }
    }
    for old in &base.micro {
        if !new.micro.iter().any(|r| r.name == old.name) {
            removed.push(old.name.clone());
        }
    }
    for result in &new.replay {
        match base.replay.iter().find(|b| b.name == result.name) {
            Some(old) => {
                let band = noise_band(old.run_spread, result.run_spread, threshold);
                let ratio = result.median_ms / old.median_ms;
                rows.push(CompareRow {
                    name: result.name.clone(),
                    base: old.median_ms,
                    new: result.median_ms,
                    unit: "ms",
                    band,
                    verdict: verdict(ratio, band),
                });
            }
            None => added.push(result.name.clone()),
        }
    }
    for old in &base.replay {
        if !new.replay.iter().any(|r| r.name == old.name) {
            removed.push(old.name.clone());
        }
    }

    println!(
        "\n{:<28} {:>12} {:>12} {:>9} {:>9}  verdict",
        "name", "baseline", "this run", "change", "band"
    );
    for row in &rows {
        let change = 100.0 * (row.new / row.base - 1.0);
        println!(
            "{:<28} {:>12} {:>12} {:>8.1}% {:>8.1}%  {}",
            row.name,
            format_measure(row.base, row.unit),
            format_measure(row.new, row.unit),
            change,
            100.0 * row.band,
            row.verdict.label()
        );
    }
    for name in &added {
        println!("{name:<28} {:>12} {:>12}  added", "-", "-");
    }
    // Absent from this run, which is not the same as removed from the code: a run of one tier or
    // under a `--filter` measures a subset on purpose.
    for name in &removed {
        println!("{name:<28} {:>12} {:>12}  not measured in this run", "-", "-");
    }

    println!("\nfingerprints:");
    for result in &new.replay {
        match base.replay.iter().find(|b| b.name == result.name) {
            Some(old) if old.fingerprint == result.fingerprint => {
                println!("  {:<28} {} physics bit-identical", result.name, result.fingerprint);
            }
            Some(old) => {
                println!(
                    "  {:<28} {} was {} - physics changed; timings are not like for like",
                    result.name, result.fingerprint, old.fingerprint
                );
            }
            None => println!("  {:<28} {} (no baseline)", result.name, result.fingerprint),
        }
    }

    Comparison { rows, why_not_failable: why_not_failable(RunKind::of(base), RunKind::of(new)) }
}

/// One regressed row measured a second time, and what that made of its verdict.
struct Retried {
    name: String,
    first: f64,
    second: f64,
    unit: &'static str,
    band: f64,
    reproduced: bool,
}

/// Re-judge the regressed rows against a second measurement of the same thing.
///
/// A regression counts only if it reproduces. On this machine an unchanged build put one row of the
/// twenty - a different one each time - uniformly 13 to 19 percent slow about one run in four, with
/// that benchmark's own sample spread still reading under one percent: the whole sixty-millisecond
/// window ran slow, so there is no outlier for the median absolute deviation to see and no way for
/// `noise_band` to widen itself. What such a transient cannot do is land on the same row twice
/// running, so the row is measured again and keeps its verdict only if the second measurement is
/// *also* outside the band against the same baseline.
///
/// This is not a wider band. The threshold and the band are exactly what they were; what has
/// changed is that a regression has to happen twice. The cost is nothing on a clean run, because
/// nothing is re-measured when nothing regressed.
///
/// Improvements are never passed in and are never touched: they fail nothing, so confirming them
/// would buy nothing. A row with no second measurement - one that could not be re-measured - keeps
/// the verdict it had.
fn reconfirm(rows: &mut [CompareRow], second: &[(String, f64)]) -> Vec<Retried> {
    let mut out = Vec::new();
    for row in rows.iter_mut().filter(|r| r.verdict == Verdict::Regression) {
        let Some((_, again)) = second.iter().find(|(name, _)| name == &row.name) else {
            continue;
        };
        let reproduced = verdict(again / row.base, row.band) == Verdict::Regression;
        out.push(Retried {
            name: row.name.clone(),
            first: row.new,
            second: *again,
            unit: row.unit,
            band: row.band,
            reproduced,
        });
        if !reproduced {
            row.verdict = Verdict::Same;
        }
    }
    out
}

/// The retry table, the summary line and the process's exit code.
fn conclude(comparison: &Comparison, retried: &[Retried]) -> i32 {
    if !retried.is_empty() {
        println!(
            "\n{:<28} {:>12} {:>12} {:>9}  verdict",
            "re-measured", "first", "again", "band"
        );
        for row in retried {
            println!(
                "{:<28} {:>12} {:>12} {:>8.1}%  {}",
                row.name,
                format_measure(row.first, row.unit),
                format_measure(row.second, row.unit),
                100.0 * row.band,
                if row.reproduced { "reproduced - REGRESSION" } else { "did not reproduce - noise" }
            );
        }
    }
    let regressions = comparison.rows.iter().filter(|r| r.verdict == Verdict::Regression).count();
    println!(
        "\n{} regressed, {} improved, {} the same within noise.",
        regressions,
        comparison.rows.iter().filter(|r| r.verdict == Verdict::Improvement).count(),
        comparison.rows.iter().filter(|r| r.verdict == Verdict::Same).count()
    );
    if let Some(why) = comparison.why_not_failable {
        println!("Not failing on any of that: {why}");
        return 0;
    }
    i32::from(regressions > 0)
}

fn format_measure(value: f64, unit: &str) -> String {
    if unit == "ns" { human_ns(value) } else { format!("{value:.3} ms") }
}

/// One replay scenario's block of the report.
fn print_replay(result: &ReplayResult) {
    // "one pass" rather than "best of 1 runs": a single window's spread is of its chunk medians and
    // not of anything repeated, and the line should not imply a best-of that was not taken.
    let passes = if result.runs == 1 {
        format!("one pass in {:.1} s (chunk spread", result.wall_s)
    } else {
        format!("best of {} runs in {:.1} s (run spread", result.runs, result.wall_s)
    };
    println!(
        "\n{}  {} frames over {:.1} M, {passes} {:.1}%)",
        result.name,
        result.frames,
        result.sim_time,
        100.0 * result.run_spread
    );
    println!(
        "  ms/frame  median {:.3}  p95 {:.3}  max {:.3}   ({:.1}% of the {:.2} ms budget)",
        result.median_ms, result.p95_ms, result.max_ms, result.budget_percent, FRAME_BUDGET_MS
    );
    if let Some(split) = result.split {
        println!(
            "  split     ui {:.3} ms  tessellate {:.3} ms   (the simulation step is inside ui; \
             subtract the sim tier's median for this scenario)",
            split.ui_ms, split.tessellate_ms
        );
    }
    let counters = &result.counters;
    println!(
        "  workload  pulses {}/{}  live rays {}/{}  heard by Bob {}  by Alice {}  \
         budget exhausted {}  dropped in flight {}",
        counters.pulses_alice,
        counters.pulses_bob,
        counters.live_rays_alice,
        counters.live_rays_bob,
        counters.heard_by_bob,
        counters.heard_by_alice,
        counters.budget_exhausted,
        counters.dropped_in_flight
    );
    print!("  per 10 M ");
    for bucket in &result.buckets {
        print!("{:.0}M:{:.2} ", bucket.from_m, bucket.mean_ms);
    }
    println!();
    println!("  fingerprint {}", result.fingerprint);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_verdict_widens_with_the_measured_noise() {
        // The threshold is a floor: a pair of quiet benchmarks is judged on it alone.
        let quiet = noise_band(0.005, 0.005, 0.05);
        assert!((quiet - 0.05).abs() < 1e-12, "3 x 1% is under the 5% floor, so the floor holds");
        assert_eq!(verdict(1.10, quiet), Verdict::Regression);
        assert_eq!(verdict(0.90, quiet), Verdict::Improvement);
        assert_eq!(verdict(1.03, quiet), Verdict::Same);

        // A noisy pair cannot support a claim the noise would explain. 3 x (4% + 4%) = 24%.
        let noisy = noise_band(0.04, 0.04, 0.05);
        assert!((noisy - 0.24).abs() < 1e-12);
        assert_eq!(verdict(1.10, noisy), Verdict::Same, "a 10% move inside a 24% band says nothing");
        assert_eq!(verdict(1.30, noisy), Verdict::Regression);

        // The two directions are the same rule, and a ratio that is not a number is not a verdict.
        assert_eq!(verdict(f64::NAN, quiet), Verdict::Same);
        assert_eq!(verdict(1.0, 0.0), Verdict::Same, "identical is never a regression");
    }

    #[test]
    fn test_only_like_for_like_runs_can_fail_the_process() {
        let full = RunKind { quick: false, savepoint: false, once: false };
        let quick = RunKind { quick: true, savepoint: true, once: false };
        // The two shapes that are a check: two full runs, and two quick runs of a save point.
        assert_eq!(why_not_failable(full, full), None);
        assert_eq!(why_not_failable(quick, quick), None);
        // Their thresholds differ, because their spreads do.
        assert_eq!(default_threshold(false), 0.05);
        assert_eq!(default_threshold(true), 0.10);

        // Mixing the two is a warning whichever way round it is: different lengths of different
        // parts of a run.
        assert!(why_not_failable(full, quick).is_some());
        assert!(why_not_failable(quick, full).is_some());

        // A quick baseline from before save points existed says nothing about what it measured, so
        // there is nothing to be like for like with.
        let smoke = RunKind { quick: true, savepoint: false, once: false };
        assert!(why_not_failable(smoke, quick).is_some());
        assert!(why_not_failable(quick, smoke).is_some());

        // And a single pass never fails on its timings, on either side and in either tier.
        assert!(why_not_failable(RunKind { once: true, ..full }, full).is_some());
        assert!(why_not_failable(quick, RunKind { once: true, ..quick }).is_some());
    }

    #[test]
    fn test_a_regression_has_to_happen_twice_to_count() {
        // Three rows measured against a baseline of 100, with a band of 10%: one that regressed and
        // meant it, one that regressed because the machine dipped for sixty milliseconds, and an
        // improvement, which is never re-measured because it fails nothing.
        let row = |name: &str, new: f64, verdict| CompareRow {
            name: name.to_string(),
            base: 100.0,
            new,
            unit: "ns",
            band: 0.10,
            verdict,
        };
        let mut rows = vec![
            row("kernel/real", 130.0, Verdict::Regression),
            row("kernel/transient", 116.0, Verdict::Regression),
            row("kernel/faster", 80.0, Verdict::Improvement),
        ];
        // The second measurement of the transient comes back where it started; the real one does
        // not. The improvement is offered a second reading it never asked for, to prove it is left
        // alone rather than merely absent.
        let second = vec![
            ("kernel/real".to_string(), 128.0),
            ("kernel/transient".to_string(), 101.0),
            ("kernel/faster".to_string(), 100.0),
        ];
        let retried = reconfirm(&mut rows, &second);

        assert_eq!(rows[0].verdict, Verdict::Regression, "130 then 128 is the code, not the machine");
        assert_eq!(rows[1].verdict, Verdict::Same, "116 then 101 did not reproduce");
        assert_eq!(rows[2].verdict, Verdict::Improvement, "an improvement is never re-judged");

        assert_eq!(retried.len(), 2, "only the two regressions were re-measured");
        assert!(retried[0].reproduced && retried[0].first == 130.0 && retried[0].second == 128.0);
        assert!(!retried[1].reproduced);

        // A row that could not be re-measured at all keeps the verdict it had: the retry decides
        // regressions, and a retry that did not happen decides nothing.
        let mut lonely = vec![row("kernel/real", 130.0, Verdict::Regression)];
        assert!(reconfirm(&mut lonely, &[]).is_empty());
        assert_eq!(lonely[0].verdict, Verdict::Regression);
    }

    #[test]
    fn test_the_command_line_says_what_it_means() {
        let parse_of = |line: &str| {
            parse(&line.split_whitespace().map(str::to_string).collect::<Vec<_>>())
        };
        let all = parse_of("--perf").unwrap();
        assert_eq!(all.tiers.len(), 3, "no tier named means all three");
        assert!(!all.quick && !all.once && all.filter.is_none());
        assert!(parse_of("--perf sim --once").unwrap().once);

        let one = parse_of("--perf micro sim --quick --filter ray --threshold 12").unwrap();
        assert_eq!(one.tiers, vec![Tier::Micro, Tier::Sim]);
        assert!(one.quick);
        assert!(one.matches("ray/step-frozen") && !one.matches("field/advance"));
        assert_eq!(one.threshold, Some(0.12), "an explicit threshold wins over either default");
        assert_eq!(all.threshold, None, "and an unstated one is not a number yet");
        assert!(parse_of("--perf --make-savepoints").unwrap().make_savepoints);

        let both = parse_of("--perf --save base --compare base").unwrap();
        assert_eq!(both.save.as_deref(), Some("base"));
        assert_eq!(both.compare.as_deref(), Some("base"));

        assert!(parse_of("--perf nonsense").is_err(), "an unknown tier is a mistake, not a filter");
        assert!(parse_of("--perf --filter").is_err(), "a flag with no value is a mistake");
        assert!(parse_of("--perf --wat").is_err());
    }
}
