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
//! # The command line
//!
//! ```text
//! --perf [micro|sim|frame]...   which tiers to run (default: all three)
//! --filter <substr>             only benchmarks and scenarios whose name contains substr
//! --quick                       fewer samples and much shorter replays: a smoke run, not a measurement
//! --list                        print the names and what each one measures, then exit
//! --save <name>                 write the results to target/perf/<name>.json
//! --compare <name>              compare against target/perf/<name>.json; exit 1 on a regression
//! --threshold <percent>         regression threshold, default 5
//! --help                        this text
//! ```
//!
//! A `--compare` involving a `--quick` run on either side prints its table and then exits 0
//! whatever the table says. Eight samples of a millisecond and one pass of each replay leave an
//! unchanged build moving by ten to fifteen percent between runs, so a quick comparison that could
//! fail would be a coin toss dressed as a check, and a check that is a coin toss gets switched off.
//! Use `--quick` to see that the harness runs and that the fingerprints still match; drop it to
//! decide anything.
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
//! `--target-dir target/probe` is not optional in practice: the app may be running from
//! `target/release`, which locks the executable, and a build into the same directory then fails.
//!
//! Advice, in the order it matters:
//!
//! * Close the app and anything else heavy. A browser rendering video in the background moves these
//!   numbers by more than most optimisations do.
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
pub(crate) mod replay;

use std::io::Write;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use harness::{Budget, Stats, human_ns};
use replay::{Mode, ReplayResult};

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
    list: bool,
    save: Option<String>,
    compare: Option<String>,
    /// The regression threshold as a fraction, so 5% is 0.05.
    threshold: f64,
    help: bool,
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
        list: false,
        save: None,
        compare: None,
        threshold: 0.05,
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
                opts.threshold = percent / 100.0;
            }
            "--quick" => opts.quick = true,
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

  --perf [micro|sim|frame]...   which tiers to run (default: all three)
  --filter <substr>             only benchmarks and scenarios whose name contains substr
  --quick                       fewer samples and much shorter replays: a smoke run, not a measurement
                                (a --compare with a quick run on either side never exits 1)
  --list                        print the names and what each one measures, then exit
  --save <name>                 write the results to target/perf/<name>.json
  --compare <name>              compare against target/perf/<name>.json; exit 1 on a regression
  --threshold <percent>         regression threshold, default 5
  --help                        this text

Tiers:
  micro   one call of one hot spot, against state built once and shared.
  sim     a scripted run of the real app at a fixed 1/60 M per frame, simulation only, no ui.
  frame   the same run through the real ui in a headless egui context, plus tessellation.
          CPU frame cost: no GPU upload, no present, no vsync.

The A/B workflow:

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
  about one machine and is worthless on any other.

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

    // The micro tier's benchmarks are built from the shared fixtures, so even `--list` has to build
    // them - at the quick size, which takes well under a second - rather than keep a second copy of
    // the list that could drift out of step with the first.
    let needs_fixtures = opts.wants(Tier::Micro);
    let fixtures = needs_fixtures.then(|| {
        if !opts.list {
            println!("Building the shared field ...");
        }
        micro::Fixtures::build(opts.quick || opts.list)
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

    let budget = if opts.quick { Budget::quick() } else { Budget::full() };
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
            let stats = harness::measure(&mut bench.body, &budget);
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
            let result = replay::replay(&scenario, mode, opts.quick);
            print_replay(&result);
            replay_results.push(result);
        }
    }

    // The drift probe: the first micro-benchmark measured again, now that the replays have run the
    // machine hot. If it has moved, nothing measured in between is trustworthy either.
    if let Some(first) = benches.first_mut()
        && let Some(before) = micro_results.iter().find(|r| r.name == first.name)
    {
        let again = harness::measure(&mut first.body, &budget);
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
        debug_assertions: cfg!(debug_assertions),
        drift_percent,
        micro: micro_results,
        replay: replay_results,
    };

    if let Some(name) = opts.save.as_ref()
        && let Err(message) = save(&report, name)
    {
        eprintln!("--save: {message}");
        return 2;
    }

    if let Some(name) = opts.compare.as_ref() {
        return match load(name) {
            Ok(base) => compare(&base, &report, opts.threshold),
            Err(message) => {
                eprintln!("--compare: {message}");
                2
            }
        };
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

/// One row of the compare table: a name measured on both sides.
struct CompareRow {
    name: String,
    base: f64,
    new: f64,
    unit: &'static str,
    band: f64,
    verdict: Verdict,
}

/// Print the comparison and return the process's exit code: 1 if anything regressed.
fn compare(base: &Report, new: &Report, threshold: f64) -> i32 {
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

    let regressions = rows.iter().filter(|r| r.verdict == Verdict::Regression).count();
    println!(
        "\n{} regressed, {} improved, {} the same within noise.",
        regressions,
        rows.iter().filter(|r| r.verdict == Verdict::Improvement).count(),
        rows.iter().filter(|r| r.verdict == Verdict::Same).count()
    );
    // A quick run cannot fail the process, however its table reads. Eight samples of a millisecond
    // and a single pass of each replay put the run-to-run variation of an unchanged build at ten to
    // fifteen percent on the cheaper benchmarks, so a quick comparison that exited 1 would be a
    // coin toss dressed as a check - and something with a coin toss in it ends up being ignored or
    // switched off. The verdicts are still printed, because a *large* move is worth seeing even
    // from a smoke run; what they are not is a pass or a fail.
    if base.quick || new.quick {
        println!(
            "Not failing on any of that: one side of this comparison is a --quick run, whose \
             samples are too few to tell a regression from the machine. Re-measure without \
             --quick before believing a verdict."
        );
        return 0;
    }
    i32::from(regressions > 0)
}

fn format_measure(value: f64, unit: &str) -> String {
    if unit == "ns" { human_ns(value) } else { format!("{value:.3} ms") }
}

/// One replay scenario's block of the report.
fn print_replay(result: &ReplayResult) {
    println!(
        "\n{}  {} frames over {:.1} M, best of {} runs in {:.1} s (run spread {:.1}%)",
        result.name,
        result.frames,
        result.sim_time,
        result.runs,
        result.wall_s,
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

fn epoch_seconds() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// The short hash of the commit this was built from, or "unknown" if git is not there or the
/// directory is not a checkout. A report from an unknown commit is still a useful report; a harness
/// that fell over because git was missing would not be.
fn git_short_hash() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .map(|out| String::from_utf8_lossy(&out.stdout).trim().to_string())
        .filter(|hash| !hash.is_empty())
        .unwrap_or_else(|| "unknown".to_string())
}

/// Whether the tree had uncommitted changes. A dirty baseline is a baseline of something that is
/// not in the history and cannot be got back to, which is worth saying on the report.
fn git_dirty() -> bool {
    std::process::Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .is_some_and(|out| !String::from_utf8_lossy(&out.stdout).trim().is_empty())
}

/// Seconds since the Unix epoch as an ISO-8601 UTC timestamp.
///
/// Written out rather than taken from a crate, because the crate list is deliberately five entries
/// long and a date on a report is not worth a sixth. The calendar arithmetic is Howard Hinnant's
/// `civil_from_days`, which is exact for every date the proleptic Gregorian calendar covers.
fn utc_iso(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rest = secs % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rest / 3600,
        (rest % 3600) / 60,
        rest % 60
    )
}

/// The civil date of a day number counted from 1970-01-01.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    // Shift the epoch to 0000-03-01, which puts the leap day at the end of the year and makes the
    // month arithmetic a single linear formula.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
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
    fn test_the_report_timestamp_is_the_date_it_claims() {
        assert_eq!(utc_iso(0), "1970-01-01T00:00:00Z");
        assert_eq!(utc_iso(86_399), "1970-01-01T23:59:59Z");
        assert_eq!(utc_iso(86_400), "1970-01-02T00:00:00Z");
        // A leap day in a century that *is* a leap year, and the first of March in one that is
        // not, which is where a hand-rolled calendar usually goes wrong.
        assert_eq!(civil_from_days(11_016), (2000, 2, 29));
        assert_eq!(civil_from_days(47_541), (2100, 3, 1));
        assert_eq!(utc_iso(1_774_224_000), "2026-03-23T00:00:00Z");
    }

    #[test]
    fn test_the_command_line_says_what_it_means() {
        let parse_of = |line: &str| {
            parse(&line.split_whitespace().map(str::to_string).collect::<Vec<_>>())
        };
        let all = parse_of("--perf").unwrap();
        assert_eq!(all.tiers.len(), 3, "no tier named means all three");
        assert!(!all.quick && all.filter.is_none());

        let one = parse_of("--perf micro sim --quick --filter ray --threshold 12").unwrap();
        assert_eq!(one.tiers, vec![Tier::Micro, Tier::Sim]);
        assert!(one.quick);
        assert!(one.matches("ray/step-frozen") && !one.matches("field/advance"));
        assert!((one.threshold - 0.12).abs() < 1e-12);

        let both = parse_of("--perf --save base --compare base").unwrap();
        assert_eq!(both.save.as_deref(), Some("base"));
        assert_eq!(both.compare.as_deref(), Some("base"));

        assert!(parse_of("--perf nonsense").is_err(), "an unknown tier is a mistake, not a filter");
        assert!(parse_of("--perf --filter").is_err(), "a flag with no value is a mistake");
        assert!(parse_of("--perf --wat").is_err());
    }
}
