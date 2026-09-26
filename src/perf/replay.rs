//! Tiers 2 and 3: whole runs of the real app, scripted and played at a fixed step.
//!
//! A micro-benchmark answers "how long does this call take". A replay answers the question a user
//! actually has, which is "how long does a frame take, and does it get worse as the run goes on".
//! It does that by driving the real `SpacetimeApp` - the same object the window drives, configured
//! through the same `AppControls` cards the panel writes - rather than a model of it, so nothing in
//! the pipeline can be left out of the measurement by accident.
//!
//! **The fixed step is what makes a replay a measurement.** The app plays at `play_speed` units of
//! M per wall-clock second, so a slow machine takes larger steps and fewer of them per M of
//! simulated time: left alone, a replay would measure a moving target, and a slower build would
//! flatter itself by taking fewer frames over the same run. Both modes here therefore step by exactly 1/60 M per frame, which fixes the
//! work per frame and makes two runs comparable to the bit. `sim` mode calls
//! `SpacetimeApp::step_forward` directly; `frame` mode sets `SpacetimeApp::fixed_frame_dt`, which
//! is the one hook the app carries for this, and lets the ordinary play loop take the same step.
//!
//! **The two modes and what each leaves out.** `sim` runs the simulation and nothing else: no ui,
//! no layout, no painting, no tessellation. `frame` runs the whole of `SpacetimeApp::ui` through a
//! headless `egui::Context` at 1280x820 points and then tessellates the shapes it produced, and
//! times the two separately. That is CPU frame cost: it excludes the GPU upload of the tessellated
//! meshes, the present, and vsync, none of which happen in this process at all. The simulation step
//! is inside the ui figure and cannot be separated out from inside it - the sim tier's median for
//! the same scenario is what to subtract.
//!
//! **Windows.** A scenario replay is a whole run: a freshly dropped app at t = 0, played to a stated
//! length. The two loops that time it - `sim_window` and `frame_window` - take an app in whatever
//! state it is already in, which is what `crate::perf::quick` uses to measure three windows in a row
//! on one app restored from a save point. The measuring is the same code; what differs is where the
//! run came from and how far it is played.
//!
//! **The fingerprint.** At the end of every replay a 64-bit FNV-1a hash is taken over the bit
//! patterns of the whole simulated state: the clock, both worldlines, every ray of every pulse of
//! both transmissions, and every arrival either side recorded. It is `Simulation::fingerprint`,
//! which is where the state is, rather than anything this module knows. An optimisation that claims to
//! change only the speed must leave this number alone, and `--compare` says for each scenario
//! whether it did. A mismatch is reported rather than failed, because deliberately changing the
//! physics is allowed - what is not allowed is comparing timings across it without noticing.

use std::time::Instant;

use eframe::App;

use crate::app::SpacetimeApp;
use crate::gui::controls::{ReferenceFrame, StepMode};
use crate::perf::harness::{chunked_spread, median_of, percentile_of, relative_spread};
use crate::perf::{headless_context, raw_input};
use crate::physics::observer::{ObserverMode, Release};
use crate::physics::wavefront::SignalField;

/// The step every replay takes, in M of coordinate time: one frame of a 60 Hz playback at the
/// app's default play speed of 1 M per second.
pub(crate) const FRAME_DT: f64 = 1.0 / 60.0;

/// Coordinate time one bucket of the time series covers, in M. A 100 M scenario comes out as ten
/// rows, which is enough to see cost climb as the field fills and few enough to read.
const BUCKET_M: f64 = 10.0;

/// How many times each scenario is replayed, per tier. The run with the lowest total is the one
/// reported, on the same argument the minimum is kept per sample: nothing makes a run faster than
/// the machine allows, so the fastest is the closest to the cost of the code. The spread across the
/// repeats is reported beside it and is what `--compare` uses as this tier's noise figure.
///
/// The frame tier gets two rather than three because it is the expensive one - a frame-mode replay
/// of `isco-pair-128` costs five times what the same run costs with no ui - and the whole suite has
/// to stay inside about ten minutes to be something anybody will run before and after an idea. Two
/// repeats still give a spread; one would leave `--compare` with nothing but its own threshold to
/// judge by, which is why this is two and not one.
const SIM_RUNS: usize = 3;
const FRAME_RUNS: usize = 2;

/// How many chunks a window's frames are cut into to arrive at its own noise figure.
///
/// Six: few enough that a chunk spans several emissions and its median is therefore of the same mix
/// of work every time, many enough that the median absolute deviation over the chunks is a figure
/// rather than a coin toss. `harness::chunked_spread` is why this is not the spread over raw frames.
/// A scenario replayed several times overwrites this with the spread across the repeats, which is a
/// stronger statement and the one the full suite has always reported; a single window - which is
/// what the quick tier measures - keeps it.
const WINDOW_CHUNKS: usize = 6;

/// One scripted run of the app.
pub(crate) struct Scenario {
    pub name: &'static str,
    /// What the scenario is for: which code paths it puts under the measurement.
    pub what: &'static str,
    /// M of coordinate time to play.
    pub duration: f64,
    /// The cards, the geometry and the sliders this scenario sets, applied to a freshly defaulted
    /// app. Everything it does not touch is the app exactly as it opens.
    pub setup: fn(&mut SpacetimeApp),
    /// Which picture the foliation column draws in `frame` mode. Ignored by `sim`, which paints
    /// nothing.
    pub view: ReferenceFrame,
    /// Scenarios that exist only to measure a painter have nothing to say in `sim` mode, where
    /// they would be a duplicate of another scenario to the last bit.
    pub frame_only: bool,
}

/// Every scenario, in report order.
///
/// The durations are shorter than the 120 M the earlier hand measurements used, and deliberately
/// so: at the measured 0.9 to 2.1 seconds of wall time per M of a frame-mode replay, 120 M apiece
/// would put the frame tier alone at over twenty minutes, and a suite nobody runs measures nothing.
/// What a duration has to do instead is outlast the scenario's approach to its steady state, which
/// is one retention window of the longer-lived transmission - `max_pulses` emissions at 0.1 M of
/// the emitter's proper time each, so 18 M of the clock for Alice on the prograde ISCO at a cap of
/// 64 and 36 M at 128 - after which the pulse count is flat and the per-10-M series any run prints
/// stops climbing. `isco-pair-64` at 30 M and `isco-pair-128` at 50 M each end with a bucket or
/// more of plateau; cut either shorter and the median is of a field still filling. The other two
/// are set by the physics: `default-infall` has to outlast Bob's fall to the ring, and
/// `far-branch-freeze` has to outlast his freeze at t = 85 M.
///
/// Both tiers play the same length, so a scenario's `sim` and `frame` replays end on the same state
/// and their fingerprints can be compared with each other as well as with a baseline.
pub(crate) fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "default-infall",
            what: "the app exactly as it opens - Alice on the prograde ISCO, Bob a raindrop from \
                   4.5 M, both transmitting - played well past Bob's arrival on the ring, which \
                   is about 4 M of his proper time and some 15 M of the clock's",
            duration: 40.0,
            setup: |_app| {},
            view: ReferenceFrame::DistantObserver,
            frame_only: false,
        },
        Scenario {
            name: "isco-pair-64",
            what: "a = 0.9, Alice free on the prograde ISCO (r = 2.32 M) and Bob on the retrograde \
                   one (r = 8.72 M), both transmitting, at the default cap of 64 wavefronts. The \
                   layout this project's earlier hand measurements used, played to the plateau \
                   rather than to their 120 M",
            duration: 30.0,
            setup: |app| isco_pair(app, 64),
            view: ReferenceFrame::DistantObserver,
            frame_only: false,
        },
        Scenario {
            name: "isco-pair-128",
            what: "the same pair at the top of the Wavefronts kept slider, which is twice the \
                   rays in flight and so the worst case the panel can ask for. Longer than its                    sibling because the window it has to fill is twice as long",
            duration: 50.0,
            setup: |app| isco_pair(app, 128),
            view: ReferenceFrame::DistantObserver,
            frame_only: false,
        },
        Scenario {
            name: "isco-pair-1024x128",
            what: "the same pair with both panel sliders at their top, 1024 wavefront points and \
                   128 wavefronts kept: seven times the rays of `isco-pair-128` in flight, which \
                   is the heaviest field the panel can ask for. Not in the quick tier; run it \
                   with `--filter 1024`",
            duration: 50.0,
            setup: |app| {
                isco_pair(app, 128);
                app.controls.rays_per_pulse = 1024;
            },
            view: ReferenceFrame::DistantObserver,
            frame_only: false,
        },
        Scenario {
            name: "isco-pair-1024x128-raycomets",
            what: "`isco-pair-1024x128` with the panel's \"Ray comets\" diagnostic at every 8th \
                   ray, so the (t, r) chart strokes some 13 000 ray comets a frame, one on every \
                   8th of about 105 000 live rays, on top of its column comets. There to measure the chart's comets built and tessellated on \
                   the mesh workers; the stride is view state and moves no fingerprint, so \
                   `sim` would repeat `isco-pair-1024x128` to the bit",
            duration: 50.0,
            setup: |app| {
                isco_pair(app, 128);
                app.controls.rays_per_pulse = 1024;
                app.controls.ray_comet_stride = 8;
            },
            view: ReferenceFrame::DistantObserver,
            frame_only: true,
        },
        Scenario {
            name: "far-branch-freeze",
            what: "Bob released at r = 9 M with E = 1, L = 2.2 at a = 0.9, who freezes onto the far \
                   branch of r- at about t = 85 M. Played past that, so the frozen-observer paths - \
                   u^t of order 1e10, a stalled geodesic carried along the horizon's generator - \
                   are inside the measurement",
            duration: 100.0,
            setup: far_branch_freeze,
            view: ReferenceFrame::DistantObserver,
            frame_only: false,
        },
        Scenario {
            name: "isco-pair-64-volume",
            what: "the ISCO pair again with the View selector on the 2D+1 volume, which replaces \
                   the flat chart with a separate and much heavier painter: depth-sorted meshes, \
                   the pipes, and an exact past cone rebuilt whenever the focus event moves",
            duration: 30.0,
            setup: |app| isco_pair(app, 64),
            view: ReferenceFrame::GlobalVolume,
            frame_only: true,
        },
    ]
}

/// Alice on the prograde ISCO, Bob on the retrograde one, both in free fall and both transmitting,
/// at a stated pulse cap.
///
/// Both radii are read off the metric rather than typed in, so the scenario follows the geometry if
/// `OPENING_SPIN` ever moves: at a = 0.9 they are 2.321 M and 8.717 M.
fn isco_pair(app: &mut SpacetimeApp, max_pulses: usize) {
    app.controls.max_pulses = max_pulses;
    for (card, release, prograde) in [
        (&mut app.controls.alice, Release::CircularPrograde, true),
        (&mut app.controls.bob, Release::CircularRetrograde, false),
    ] {
        card.enabled = true;
        card.transmit = true;
        card.mode = ObserverMode::FreeFall;
        card.release = release;
        card.drop_r = app.sim.metric.isco(prograde);
        card.delta_t_delay = 0.0;
        card.l_ang = 0.0;
    }
    // Half a radian apart, so the two are a pair rather than the same observer twice and the light
    // between them has an arc to cross.
    app.controls.alice.drop_phi = 0.0;
    app.controls.bob.drop_phi = 0.5;
    drop_observers(app);
}

/// Bob on the one worldline that freezes onto the far branch of r-, with Alice out of the run.
///
/// E = 1 and L = 2.2 is `Observer::frozen_bob`'s worldline, stated here the way a card states one:
/// released from infinity (which is what E = 1 means) at r = 9 M with that angular momentum.
///
/// Alice stays in the run but transmits nothing. She is there to be a *receiver*: with nobody to
/// hear him, `SignalField::detect_receptions` is never called on Bob's transmission at all, and
/// the scan of a field whose fronts are piling onto r- is exactly one of the paths this scenario
/// exists to measure. Her own transmission would only add cost that has nothing to do with the
/// freeze, so it is off.
fn far_branch_freeze(app: &mut SpacetimeApp) {
    app.controls.alice.enabled = true;
    app.controls.alice.transmit = false;
    let bob = &mut app.controls.bob;
    bob.enabled = true;
    bob.transmit = true;
    bob.mode = ObserverMode::FreeFall;
    bob.release = Release::FromInfinity;
    bob.l_ang = 2.2;
    bob.drop_r = 9.0;
    bob.drop_phi = 0.0;
    bob.delta_t_delay = 0.0;
    drop_observers(app);
}

/// Rebuild the run from the two cards, which is what the panel's Reset and Drop Observers buttons
/// do and what the app's own startup does. A scenario states its cards and calls this rather than
/// assembling observers behind the panel's back, so a replay is a run the user could have set up.
fn drop_observers(app: &mut SpacetimeApp) {
    app.controls.drop_observers(&mut app.sim);
    app.controls.view_reset_requested = false;
}

/// The app the micro tier's shared field is built from: the ISCO pair at the default cap.
pub(crate) fn app_for_fixtures() -> SpacetimeApp {
    let mut app = SpacetimeApp::default();
    isco_pair(&mut app, 64);
    app
}

/// One simulation frame, in the order `SpacetimeApp::ui` takes it: the Wavefronts kept slider is
/// pushed into both transmissions first, played or paused, and the step follows.
///
/// Doing the push here rather than leaving it to `step_forward` is what makes a `sim` replay and a
/// `frame` replay of the same scenario the same run. `step_forward` is the arrow key's path and
/// never sees the slider; the play loop does, once a frame, before it steps. A scenario that sets
/// the cap to 128 would otherwise be simulated at 64 in `sim` mode and at 128 in `frame` mode.
fn sim_frame(app: &mut SpacetimeApp) {
    app.sim.set_max_pulses(app.controls.max_pulses);
    app.step_forward(FRAME_DT);
}

/// Play a scripted app forward to a coordinate time, at the replay step. Used by the sim tier and
/// by the micro tier, which needs a field in the same steady state.
pub(crate) fn play_sim(app: &mut SpacetimeApp, until: f64) {
    let frames = (until / FRAME_DT).round() as usize;
    for _ in 0..frames {
        sim_frame(app);
    }
}

/// What a replay scenario measured.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct ReplayResult {
    /// The name `--filter` and `--compare` match on: mode and scenario, e.g. `sim/isco-pair-64`.
    pub name: String,
    pub mode: String,
    pub scenario: String,
    pub frames: usize,
    /// M of coordinate time played.
    pub sim_time: f64,
    /// Wall-clock seconds the reported run took, start to finish.
    pub wall_s: f64,
    pub median_ms: f64,
    pub p95_ms: f64,
    pub max_ms: f64,
    /// The median as a percentage of one 60 Hz frame, 16.67 ms.
    pub budget_percent: f64,
    pub runs: usize,
    /// This result's noise figure, which `--compare` widens its band by.
    ///
    /// For a scenario replayed several times it is the relative spread of the total wall time across
    /// the repeats. For a single window - `--once`, and every window of the quick tier - there are
    /// no repeats to spread, so it is the spread of the window's chunk medians instead: see
    /// `harness::chunked_spread`.
    pub run_spread: f64,
    /// Frame mode only: where the frame went.
    pub split: Option<FrameSplit>,
    /// Mean ms per frame in each `BUCKET_M` of simulated time, so cost climbing over a run shows.
    pub buckets: Vec<Bucket>,
    pub counters: Counters,
    /// The state hash at the end of the run, as hex.
    pub fingerprint: String,
}

/// Where a frame-mode frame went. The simulation step is inside `ui_ms` and cannot be separated
/// from in there; subtract the sim tier's median for the same scenario.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub(crate) struct FrameSplit {
    /// Median ms in `SpacetimeApp::ui` through a headless context: the play step, the panel, the
    /// HUD and both canvases' shape building.
    pub ui_ms: f64,
    /// Median ms turning those shapes into triangles.
    pub tessellate_ms: f64,
}

/// Mean cost over one stretch of simulated time.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub(crate) struct Bucket {
    /// Coordinate time this bucket starts at, in M.
    pub from_m: f64,
    pub frames: usize,
    pub mean_ms: f64,
}

/// What the run was carrying when it ended, so that a cost change can be told apart from a workload
/// change. A build that got 20% faster by dropping a fifth of the rays is not a faster build.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub(crate) struct Counters {
    /// Wavefronts held by each transmission at the end.
    pub pulses_alice: usize,
    pub pulses_bob: usize,
    /// Rays still being integrated at the end, which is what a frame's cost is mostly made of.
    pub live_rays_alice: usize,
    pub live_rays_bob: usize,
    /// Arrivals recorded: Alice's transmission is the one Bob hears, and the other way round.
    pub heard_by_bob: usize,
    pub heard_by_alice: usize,
    /// Rays retired by the integrator's substep budget. Anything but zero means the field is
    /// missing rays and the timings are of an incomplete run.
    pub budget_exhausted: usize,
    /// Wavefronts the cap evicted while they could still have been heard. See
    /// `SignalField::dropped_in_flight`: not a fault, but a statement about what the run measured.
    pub dropped_in_flight: usize,
}

fn counters(app: &SpacetimeApp) -> Counters {
    let live = |field: &SignalField| {
        field.pulses.iter().flat_map(|p| p.rays.iter()).filter(|ray| ray.alive()).count()
    };
    Counters {
        pulses_alice: app.sim.alice_signal.pulses.len(),
        pulses_bob: app.sim.bob_signal.pulses.len(),
        live_rays_alice: live(&app.sim.alice_signal),
        live_rays_bob: live(&app.sim.bob_signal),
        heard_by_bob: app.sim.alice_signal.received_count(),
        heard_by_alice: app.sim.bob_signal.received_count(),
        budget_exhausted: app.sim.alice_signal.budget_exhausted()
            + app.sim.bob_signal.budget_exhausted(),
        dropped_in_flight: app.sim.alice_signal.dropped_in_flight()
            + app.sim.bob_signal.dropped_in_flight(),
    }
}

/// One pass of the frame timings into the numbers the report prints.
fn summarise_run(
    scenario: &str,
    mode: &str,
    frame_ms: &[f64],
    ui_ms: &[f64],
    tess_ms: &[f64],
    wall_s: f64,
    app: &SpacetimeApp,
) -> ReplayResult {
    let mut sorted = frame_ms.to_vec();
    sorted.sort_by(f64::total_cmp);
    let median = median_of(&sorted);
    let buckets = bucket_series(frame_ms);
    let split = (!ui_ms.is_empty()).then(|| {
        let mut ui = ui_ms.to_vec();
        ui.sort_by(f64::total_cmp);
        let mut tess = tess_ms.to_vec();
        tess.sort_by(f64::total_cmp);
        FrameSplit { ui_ms: median_of(&ui), tessellate_ms: median_of(&tess) }
    });
    ReplayResult {
        name: format!("{mode}/{scenario}"),
        mode: mode.to_string(),
        scenario: scenario.to_string(),
        frames: frame_ms.len(),
        sim_time: frame_ms.len() as f64 * FRAME_DT,
        wall_s,
        median_ms: median,
        p95_ms: percentile_of(&sorted, 0.95),
        max_ms: sorted.last().copied().unwrap_or(f64::NAN),
        budget_percent: 100.0 * median / (1000.0 / 60.0),
        runs: 1,
        run_spread: chunked_spread(frame_ms, WINDOW_CHUNKS),
        split,
        buckets,
        counters: counters(app),
        fingerprint: format!("{:016x}", app.sim.fingerprint()),
    }
}

/// Mean ms per frame over each `BUCKET_M` of simulated time.
fn bucket_series(frame_ms: &[f64]) -> Vec<Bucket> {
    let per_bucket = (BUCKET_M / FRAME_DT).round() as usize;
    frame_ms
        .chunks(per_bucket)
        .enumerate()
        .map(|(i, chunk)| Bucket {
            from_m: i as f64 * BUCKET_M,
            frames: chunk.len(),
            mean_ms: chunk.iter().sum::<f64>() / chunk.len() as f64,
        })
        .collect()
}

/// Replay a scenario `SIM_RUNS` or `FRAME_RUNS` times and keep the fastest, reporting the spread
/// across the runs beside it.
///
/// `once` plays the full length a single time. The timings then have no spread behind them, but the
/// fingerprint is the one any number of passes would end on, which is all a check that a refactor
/// left the physics alone is asking for.
pub(crate) fn replay(scenario: &Scenario, mode: Mode, once: bool) -> ReplayResult {
    let runs = match (once, mode) {
        (true, _) => 1,
        (false, Mode::Sim) => SIM_RUNS,
        (false, Mode::Frame) => FRAME_RUNS,
    };
    let frames = (scenario.duration / FRAME_DT).round() as usize;
    let mut results: Vec<ReplayResult> = Vec::with_capacity(runs);
    for _ in 0..runs {
        results.push(match mode {
            Mode::Sim => run_sim(scenario, frames),
            Mode::Frame => run_frame(scenario, frames),
        });
    }
    let mut totals: Vec<f64> = results.iter().map(|r| r.wall_s).collect();
    totals.sort_by(f64::total_cmp);
    let mut best = results
        .into_iter()
        .min_by(|a, b| a.wall_s.total_cmp(&b.wall_s))
        .expect("at least one run");
    best.runs = runs;
    // The spread across the repeats is the better noise figure of the two, because it is of whole
    // runs rather than of stretches inside one, and it replaces the chunk spread `summarise_run`
    // left behind. One pass has no such spread - `relative_spread` of a single total is zero, which
    // would read as a machine with no noise on it at all - so a single pass keeps its chunk spread.
    if runs > 1 {
        best.run_spread = relative_spread(&totals, median_of(&totals));
    }
    best
}

/// Which of the two replay tiers is being run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Mode {
    Sim,
    Frame,
}

impl Mode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Sim => "sim",
            Self::Frame => "frame",
        }
    }
}

/// Tier 2: the simulation pipeline and nothing else.
fn run_sim(scenario: &Scenario, frames: usize) -> ReplayResult {
    let mut app = SpacetimeApp::default();
    (scenario.setup)(&mut app);
    sim_window(&mut app, "sim", scenario.name, frames)
}

/// Time `frames` simulation-only frames of an app that is already set up, wherever it stands.
///
/// The scenarios enter here with a freshly dropped run at t = 0; the quick tier enters with a run
/// loaded from a save point and already at its steady state. Nothing here knows the difference,
/// which is the point: the two tiers measure the same loop.
pub(crate) fn sim_window(
    app: &mut SpacetimeApp,
    mode: &str,
    scenario: &str,
    frames: usize,
) -> ReplayResult {
    let mut frame_ms = Vec::with_capacity(frames);
    let started = Instant::now();
    for _ in 0..frames {
        let at = Instant::now();
        sim_frame(app);
        frame_ms.push(at.elapsed().as_secs_f64() * 1e3);
    }
    let wall = started.elapsed().as_secs_f64();
    summarise_run(scenario, mode, &frame_ms, &[], &[], wall, app)
}

/// Tier 3: the real `SpacetimeApp::ui` through a headless context, then tessellation.
///
/// The app is put in the state a played window is in - playing, at speed 1, in Time step mode - and
/// handed a fixed frame dt, so the play loop's own step is exactly the 1/60 M `sim` mode takes.
/// `RawInput::time` advances by 1/60 of a second a frame as well, so egui's animations behave as
/// they do in the window rather than all completing on the first frame.
fn run_frame(scenario: &Scenario, frames: usize) -> ReplayResult {
    let mut app = SpacetimeApp::default();
    (scenario.setup)(&mut app);
    play_state(&mut app, scenario.view);
    let ctx = headless_context();
    let mut time = 0.0;
    frame_window(&mut app, &ctx, "frame", scenario.name, frames, &mut time)
}

/// Put an app in the state a played window is in, looking at a stated view.
pub(crate) fn play_state(app: &mut SpacetimeApp, view: ReferenceFrame) {
    app.controls.frame_of_ref = view;
    app.controls.is_playing = true;
    app.controls.play_speed = 1.0;
    app.controls.step_mode = StepMode::Time;
    app.fixed_frame_dt = Some(FRAME_DT);
}

/// Time `frames` whole ui passes of an app that is already playing, plus their tessellation.
///
/// `time` is egui's own clock and is carried in and out rather than started at zero, so that a
/// second window on the same context - the quick tier switches the View selector and measures
/// again - continues the animation clock instead of restarting it, which would put every fade and
/// every combo box back to its first frame half way through a measurement.
pub(crate) fn frame_window(
    app: &mut SpacetimeApp,
    ctx: &egui::Context,
    mode: &str,
    scenario: &str,
    frames: usize,
    time: &mut f64,
) -> ReplayResult {
    let mut frame_ms = Vec::with_capacity(frames);
    let mut ui_ms = Vec::with_capacity(frames);
    let mut tess_ms = Vec::with_capacity(frames);
    let started = Instant::now();
    for _ in 0..frames {
        *time += FRAME_DT;
        let at = Instant::now();
        let mut output = ctx.run_ui(raw_input(*time), |ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        let ui_elapsed = at.elapsed().as_secs_f64() * 1e3;
        let pixels_per_point = output.pixels_per_point;
        let shapes = std::mem::take(&mut output.shapes);
        // Nothing here uploads a texture, and epaint panics on a delta dropped unapplied.
        output.drop_without_applying_deltas();
        let tess_at = Instant::now();
        let primitives = ctx.tessellate(shapes, pixels_per_point);
        let tess_elapsed = tess_at.elapsed().as_secs_f64() * 1e3;
        drop(primitives);
        frame_ms.push(ui_elapsed + tess_elapsed);
        ui_ms.push(ui_elapsed);
        tess_ms.push(tess_elapsed);
    }
    let wall = started.elapsed().as_secs_f64();
    summarise_run(scenario, mode, &frame_ms, &ui_ms, &tess_ms, wall, app)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_a_sim_replay_fingerprints_the_same_twice() {
        // The whole point of the fingerprint is that it is reproducible: an optimisation's claim to
        // be bit-identical is worth nothing if the baseline it is compared against was not. Two
        // quick replays of the same scenario in the same process must agree to the bit, and if they
        // ever stop agreeing then something in the step depends on the wall clock or on an
        // iteration order that is not fixed, which would make every comparison meaningless.
        //
        // Five M of it rather than its full forty, played straight through `sim_window` on a fresh
        // app: the loop under test is the one both replay tiers and the quick tier run, and a test
        // has no business either taking half a minute or depending on a save point under `target/`.
        let scenario = scenarios().into_iter().find(|s| s.name == "default-infall").unwrap();
        let play = || {
            let mut app = SpacetimeApp::default();
            (scenario.setup)(&mut app);
            sim_window(&mut app, "sim", scenario.name, 300)
        };
        let first = play();
        let second = play();
        assert_eq!(
            first.fingerprint, second.fingerprint,
            "two runs of {} disagree about the state they ended in",
            scenario.name
        );
        assert_eq!(first.frames, second.frames, "and they took the same number of frames");
        assert!(first.counters.budget_exhausted == 0, "no ray may be retired by the substep budget");
    }
}
