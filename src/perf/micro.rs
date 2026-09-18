//! Tier 1: the hot spots, measured one at a time on state built once and shared.
//!
//! Every benchmark here is a *kernel* in the sense that it is one call of one thing, timed against
//! inputs that do not change between iterations. That is what makes the numbers comparable across
//! builds and what makes them narrow: a micro-benchmark says how long `SignalField::advance` takes
//! on this particular field, and says nothing at all about how many times a frame calls it or how
//! big the field gets over a run. Those are the replay tier's questions.
//!
//! **The shared state.** Ten of the benchmarks run against one prebuilt field: the `isco-pair`
//! scenario - Alice on the prograde ISCO, Bob on the retrograde one, both transmitting at 144 rays
//! and 64 pulses - played to `FIXTURE_UNTIL` of coordinate time at the same 1/60 M step the app
//! plays at. By then both transmissions are over the pulse cap and have been evicting for some
//! time, so the field is at the steady state a long run actually spends its time in rather than at
//! the empty state a fresh app starts from. It is built once per process, deterministically, and
//! shared: building it costs a few seconds and measuring against a different field each time would
//! make the benchmarks incomparable with each other as well as across builds.
//!
//! **The three rays.** `NullRay::step` costs what it costs because of where the ray is, so one
//! number for it would be meaningless. Three are measured: a ray climbing out at large r, where the
//! substep control lets the step run to the geometric cap; a ray a tenth of an M above the ring,
//! where the connection stiffens and one frame of 1/60 M is cut into many substeps; and a ray of
//! the frozen family, E - Omega_- L < 0, settled onto r- after 40 M, which has stopped moving and
//! stopped turning and is capped by neither geometric condition. The third is the interesting one:
//! it is what a long run accumulates, and a field that is mostly frozen rays is a field whose cost
//! per frame should have stopped growing.
//!
//! **What the paint benchmarks include.** Each one is a whole headless egui pass -
//! `Context::run_ui` with exactly one canvas rendered inside it - so the figure carries egui's own
//! per-pass work as well as the canvas's. `paint/empty-pass` is that overhead on its own and is
//! there to be subtracted. None of them includes tessellation, texture upload, present or vsync;
//! the frame tier separates tessellation out and the rest never happens in this process at all.

use std::time::Instant;

use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::spacetime_canvas::SpacetimeCanvas;
use crate::gui::spatial_canvas::{FrontStyle, SpatialCanvas};
use crate::gui::volume_canvas::{ConeRes, VolumeCanvas, build_past_cone};
use crate::perf::harness::Bench;
use crate::perf::replay::{self, FRAME_DT};
use crate::perf::{
    CANVAS_HEIGHT, LEFT_CANVAS_WIDTH, RIGHT_CANVAS_WIDTH, headless_context, raw_input,
};
use crate::physics::geodesic::geodesic_accel;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::wavefront::{NullRay, RayState, SignalField, ray_dopri5, ray_rhs};

/// How far the shared field is played before anything is measured against it, in M of coordinate
/// time.
///
/// Alice's clock runs at roughly two thirds of the coordinate clock on the prograde ISCO and she
/// emits every 0.1 M of her own proper time, so 20 M puts about 130 pulses through a cap of 64: the
/// field has been evicting its oldest pulse for the second half of the build and holds a full 64
/// pulses of 144 rays at each end. Playing longer changes the cost per frame very little and the
/// build time a great deal.
const FIXTURE_UNTIL: f64 = 20.0;

/// The same, for `--quick` and for the tests: long enough that the cap has bitten and the field is
/// not a special case, short enough to build in well under a second.
const FIXTURE_UNTIL_QUICK: f64 = 6.0;

/// The prebuilt state every benchmark in this tier is measured against, built once and shared.
pub(crate) struct Fixtures {
    pub metric: KerrSchild,
    /// The simulation clock the field and both observers stand at.
    pub clock: f64,
    /// Alice, on the prograde ISCO, transmitting. She is the emitter of `alice_field` and the
    /// receiver of `bob_field`.
    pub alice: Observer,
    /// Bob, on the retrograde ISCO, the other way round.
    pub bob: Observer,
    /// Alice's transmission at the steady state: 64 pulses of 144 rays.
    pub alice_field: SignalField,
    /// Bob's, the same.
    pub bob_field: SignalField,
    /// Alice's transmission carried one frame past `clock` and not yet listened to, with
    /// `bob_next` the receiver at that same later time. `Pulse::sweep` intersects the receiver's
    /// worldline with what the front swept *between two passes*, and returns at once when the
    /// receiver's clock has not moved since the last one - so a detection pass over `alice_field`
    /// with `bob` as he stands would time the marks being built and never the sweep. This pair is
    /// the state a played frame is in at the moment it listens.
    pub alice_field_unheard: SignalField,
    pub bob_next: Observer,
    /// A ray climbing out through r = 16 M: the cheap case, capped by `MAX_DR_PER_SUBSTEP`.
    pub escaping: NullRay,
    /// A ray about a tenth of an M above the ring: the stiff case.
    pub infalling: NullRay,
    /// A ray of the frozen family settled onto r- after 40 M: the case a long run accumulates.
    pub frozen: NullRay,
    /// A ray state and its slope at the stiff radius, for the two kernel benchmarks that take a
    /// state rather than a ray.
    pub stiff_state: RayState,
    pub stiff_slope: RayState,
}

impl Fixtures {
    /// Build the shared state. Deterministic: same metric, same scenario, same fixed step, so two
    /// processes on two builds measure against the same field to the last bit.
    pub fn build(quick: bool) -> Self {
        let mut app = replay::app_for_fixtures();
        let until = if quick { FIXTURE_UNTIL_QUICK } else { FIXTURE_UNTIL };
        replay::play_sim(&mut app, until);
        let metric = app.metric;
        let mut alice_field_unheard = app.signal.clone();
        alice_field_unheard.advance(&metric, FRAME_DT);
        let mut bob_next = app.bob.clone().expect("the ISCO pair has Bob in it");
        bob_next.step(&metric, app.current_time + FRAME_DT, FRAME_DT);
        let (escaping, infalling, frozen) = representative_rays(&metric);
        let stiff_state: RayState = [infalling.r, infalling.phi, infalling.dr_dt, infalling.dphi_dt];
        let stiff_slope = ray_rhs(&metric, &stiff_state);
        Self {
            clock: app.current_time,
            alice: app.alice.clone().expect("the ISCO pair has Alice in it"),
            bob: app.bob.clone().expect("the ISCO pair has Bob in it"),
            alice_field: app.signal.clone(),
            bob_field: app.bob_signal.clone(),
            alice_field_unheard,
            bob_next,
            metric,
            escaping,
            infalling,
            frozen,
            stiff_state,
            stiff_slope,
        }
    }
}

/// The three rays the `ray/step-*` benchmarks run, each one integrated to where it is wanted rather
/// than constructed there, so that every one of them is a state the physics actually produces.
fn representative_rays(metric: &KerrSchild) -> (NullRay, NullRay, NullRay) {
    // Climbing out. Let go outward and radial from a raindrop at 12 M and carried 4 M, which puts
    // it near 16 M and well short of `R_ESCAPE`.
    let far = Observer::raindrop_tetrad(metric, 12.0);
    let mut escaping =
        NullRay::from_local_direction(metric, 0.0, 12.0, 0.0, &far, 0.0, &far.e0);
    escaping.step(metric, 4.0);
    assert!(escaping.alive(), "the escaping ray must still be in the field at r = {}", escaping.r);

    // Falling onto the ring. Let go inward and radial from a raindrop between the horizons and
    // walked in by steps small enough that it cannot reach R_STOP inside one of them: it is wanted
    // stiff, not dead.
    let inner = Observer::raindrop_tetrad(metric, 1.0);
    let mut infalling = NullRay::from_local_direction(
        metric,
        0.0,
        1.0,
        0.0,
        &inner,
        std::f64::consts::PI,
        &inner.e0,
    );
    let mut walked = 0;
    while infalling.alive() && infalling.r > 0.15 && walked < 4000 {
        infalling.step(metric, 0.02);
        walked += 1;
    }
    assert!(
        infalling.alive() && infalling.r < 0.2,
        "the stiff ray must be alive just above the ring, not at r = {}",
        infalling.r
    );

    // Frozen. The first direction of a 144-ray cone at r = 1 M whose energy relative to the inner
    // horizon's generator is negative, carried 40 M, by which time it is within a rounding of r-
    // and has stopped moving.
    let mut frozen = (0..144)
        .map(|i| {
            let alpha = std::f64::consts::TAU * (i as f64) / 144.0;
            NullRay::from_local_direction(metric, 0.0, 1.0, 0.0, &inner, alpha, &inner.e0)
        })
        .find(|ray| ray.frozen(metric))
        .expect("a cone let go between the horizons has a frozen family");
    for _ in 0..40 {
        frozen.step(metric, 1.0);
    }
    assert!(frozen.alive(), "a frozen ray never reaches anything and never dies");

    (escaping, infalling, frozen)
}

/// One headless pass with a canvas column of a stated width in it.
///
/// The column is allocated first so that the canvas sees the width the app gives it rather than
/// the whole window: `render` asks for `ui.available_width()`, and in a bare root ui that is 1280.
fn paint_pass(ctx: &egui::Context, time: f64, width: f32, mut draw: impl FnMut(&mut egui::Ui)) {
    let output = ctx.run_ui(raw_input(time), |ui| {
        ui.allocate_ui_with_layout(
            egui::Vec2::new(width, CANVAS_HEIGHT),
            egui::Layout::top_down(egui::Align::Min),
            |ui| draw(ui),
        );
    });
    output.drop_without_applying_deltas();
}

/// Every benchmark of this tier, in the order the report prints them.
///
/// The first one is the drift probe: `--perf` re-runs it at the very end of the whole suite and
/// prints the difference, so it has to be the cheapest and steadiest thing on the list.
pub(crate) fn benches(fx: &Fixtures) -> Vec<Bench<'_>> {
    let mut out: Vec<Bench<'_>> = Vec::new();
    // A macro rather than a closure: a closure taking a `Body<'a>` and pushing it into a
    // `&mut Vec<Bench<'a>>` cannot be written, because a mutable reference is invariant in its
    // parameter and the borrow checker will not let the body's lifetime out of the closure again.
    macro_rules! add {
        ($name:expr, $what:expr, $body:expr $(,)?) => {
            out.push(Bench { name: $name.to_string(), what: $what, body: $body })
        };
    }

    // ---- the arithmetic at the bottom of everything -------------------------------------------
    add!(
        "kernel/geodesic-accel",
        "acc^mu = -Gamma^mu_ab v^a v^b at the stiff radius: the Christoffel contraction every ray \
         substep is built out of",
        Box::new(|n| {
            let r = fx.stiff_state[0];
            let v = [1.0, fx.stiff_state[2], fx.stiff_state[3]];
            let started = Instant::now();
            for _ in 0..n {
                std::hint::black_box(geodesic_accel(
                    std::hint::black_box(&fx.metric),
                    std::hint::black_box(r),
                    std::hint::black_box(&v),
                ));
            }
            started.elapsed()
        }),
    );
    add!(
        "kernel/ray-rhs",
        "dy/dt for y = (r, phi, v^r, v^phi): one `geodesic_accel` plus the non-affine correction",
        Box::new(|n| {
            let started = Instant::now();
            for _ in 0..n {
                std::hint::black_box(ray_rhs(
                    std::hint::black_box(&fx.metric),
                    std::hint::black_box(&fx.stiff_state),
                ));
            }
            started.elapsed()
        }),
    );
    add!(
        "kernel/ray-dopri5-substep",
        "one accepted Dormand-Prince 5(4) substep of h = 0.002 M: six `ray_rhs` evaluations and \
         the embedded error estimate",
        Box::new(|n| {
            let started = Instant::now();
            for _ in 0..n {
                let substep = ray_dopri5(
                    std::hint::black_box(&fx.metric),
                    std::hint::black_box(&fx.stiff_state),
                    std::hint::black_box(&fx.stiff_slope),
                    std::hint::black_box(0.002),
                );
                std::hint::black_box(&substep);
            }
            started.elapsed()
        }),
    );

    // ---- one ray, one frame -------------------------------------------------------------------
    //
    // The ray is copied inside the timed span so that every iteration integrates the same interval
    // from the same state. A `NullRay` is nine f64 and a pair of Options; the copy is a rounding
    // error against the substeps of even the cheapest of the three.
    for (name, what, proto) in [
        (
            "ray/step-escaping",
            "one played frame (dt = 1/60 M) of a ray climbing out through r = 16 M",
            fx.escaping,
        ),
        (
            "ray/step-infalling",
            "the same frame for a ray a tenth of an M above the ring, where the connection \
             stiffens and the substep control bites",
            fx.infalling,
        ),
        (
            "ray/step-frozen",
            "the same frame for a ray of the frozen family settled onto r-, which has stopped \
             moving and stopped turning",
            fx.frozen,
        ),
    ] {
        add!(
            name,
            what,
            Box::new(move |n| {
                let started = Instant::now();
                for _ in 0..n {
                    let mut ray = std::hint::black_box(proto);
                    std::hint::black_box(ray.step(std::hint::black_box(&fx.metric), FRAME_DT));
                }
                started.elapsed()
            }),
        );
    }

    // ---- the whole field ----------------------------------------------------------------------
    //
    // Each of these consumes the state it is given, so the state is cloned per sample outside the
    // timed span and the iterations of one sample run consecutively on that clone - which is what
    // consecutive frames of the app do anyway.
    add!(
        "field/advance",
        "`SignalField::advance` by one played frame over Alice's steady-state transmission: every \
         live ray of 64 pulses, plus each pulse's extent track and ring history",
        Box::new(|n| {
            let mut field = fx.alice_field.clone();
            let started = Instant::now();
            for _ in 0..n {
                field.advance(std::hint::black_box(&fx.metric), FRAME_DT);
            }
            started.elapsed()
        }),
    );
    add!(
        "field/detect-receptions",
        "`SignalField::detect_receptions` for one played frame: `Pulse::scan` marking every ray of          every front, and `Pulse::sweep` intersecting Bob's worldline with the patch each segment          swept since the last pass. A pass is only a pass once, so the field is cloned afresh for          every iteration and the clock runs over the pass alone",
        Box::new(|n| {
            let mut timed = std::time::Duration::ZERO;
            for _ in 0..n {
                let mut field = fx.alice_field_unheard.clone();
                let started = Instant::now();
                field.detect_receptions(std::hint::black_box(&fx.metric), &fx.bob_next);
                timed += started.elapsed();
                std::hint::black_box(&field);
            }
            timed
        }),
    );
    add!(
        "field/step-back",
        "`SignalField::step_back` by one played frame: the same rays integrated the other way, \
         plus the truncation of every track, history and reception record",
        Box::new(|n| {
            let mut field = fx.alice_field.clone();
            let started = Instant::now();
            for _ in 0..n {
                field.step_back(std::hint::black_box(&fx.metric), FRAME_DT);
            }
            started.elapsed()
        }),
    );
    add!(
        "field/emit-144-rays",
        "one emission at the default ray count into an empty field: the tetrad, 144 \
         `NullRay::from_local_direction` and the allocation of the pulse",
        Box::new(|n| {
            let started = Instant::now();
            for _ in 0..n {
                let mut field = SignalField::default();
                field.emit_if_due(std::hint::black_box(&fx.metric), &fx.alice);
                std::hint::black_box(&field);
            }
            started.elapsed()
        }),
    );

    // ---- the timelike side --------------------------------------------------------------------
    add!(
        "observer/step",
        "one played frame of one observer on the prograde ISCO, trail recording included",
        Box::new(|n| {
            let mut alice = fx.alice.clone();
            let mut t = fx.clock;
            let started = Instant::now();
            for _ in 0..n {
                t += FRAME_DT;
                alice.step(std::hint::black_box(&fx.metric), t, FRAME_DT);
            }
            started.elapsed()
        }),
    );
    add!(
        "observer/event-at",
        "`Observer::event_at` 1 M back down the worldline: a clone and a `rewind_to`, paid once \
         per recorded reception",
        Box::new(|n| {
            let target = fx.bob.t - 1.0;
            let started = Instant::now();
            for _ in 0..n {
                std::hint::black_box(
                    fx.bob.event_at(std::hint::black_box(&fx.metric), std::hint::black_box(target)),
                );
            }
            started.elapsed()
        }),
    );

    // ---- the exact past cone the volume view draws ---------------------------------------------
    add!(
        "volume/build-past-cone",
        "`build_past_cone` at full resolution over 5 M: 36 null generators integrated backwards, \
         rebuilt by the volume view whenever the focus event moves",
        Box::new(|n| {
            let t_min = fx.bob.t - 5.0;
            let started = Instant::now();
            for _ in 0..n {
                std::hint::black_box(build_past_cone(
                    std::hint::black_box(&fx.metric),
                    &fx.bob,
                    t_min,
                    ConeRes::FULL,
                ));
            }
            started.elapsed()
        }),
    );

    // ---- painting, one canvas at a time --------------------------------------------------------
    //
    // One headless egui pass per iteration, with exactly one canvas inside it. The context, the
    // canvas and the observers are built once per benchmark: a canvas carries its own camera and
    // its own caches, and the steady state of a played run is a warm one. `paint/empty-pass` is the
    // same pass with nothing in it.
    add!(
        "paint/empty-pass",
        "a headless egui pass with an empty ui: the per-pass overhead the three paint benchmarks \
         below all carry, and the figure to subtract from them",
        Box::new(|n| {
            let ctx = headless_context();
            let mut time = 0.0;
            let started = Instant::now();
            for _ in 0..n {
                time += FRAME_DT;
                paint_pass(&ctx, time, LEFT_CANVAS_WIDTH, |_ui| {});
            }
            started.elapsed()
        }),
    );
    add!(
        "paint/spacetime-canvas",
        "one headless pass painting the (t, r) foliation chart of the steady state, both \
         transmissions drawn",
        Box::new(|n| {
            let ctx = headless_context();
            let mut canvas = SpacetimeCanvas::default();
            let mut time = 0.0;
            let started = Instant::now();
            for _ in 0..n {
                time += FRAME_DT;
                paint_pass(&ctx, time, LEFT_CANVAS_WIDTH, |ui| {
                    canvas.render(
                        ui,
                        &fx.metric,
                        Some(&fx.bob),
                        Some(&fx.alice),
                        fx.clock,
                        CANVAS_HEIGHT,
                        true,
                        ReferenceFrame::DistantObserver,
                        1.0,
                        SignalViews { alice: &fx.alice_field, bob: &fx.bob_field },
                        true,
                    );
                });
            }
            started.elapsed()
        }),
    );
    add!(
        "paint/spatial-canvas",
        "one headless pass painting the equatorial (x, y) view of the same state: every front as a \
         closed polyline of 144 points",
        Box::new(|n| {
            let ctx = headless_context();
            let mut canvas = SpatialCanvas::default();
            let mut bob = Some(fx.bob.clone());
            let mut alice = Some(fx.alice.clone());
            let mut details = false;
            let mut time = 0.0;
            let started = Instant::now();
            for _ in 0..n {
                time += FRAME_DT;
                paint_pass(&ctx, time, RIGHT_CANVAS_WIDTH, |ui| {
                    canvas.render(
                        ui,
                        &fx.metric,
                        &mut bob,
                        &mut alice,
                        fx.clock,
                        SignalViews { alice: &fx.alice_field, bob: &fx.bob_field },
                        CANVAS_HEIGHT,
                        true,
                        ReferenceFrame::DistantObserver,
                        1.0,
                        FrontStyle { arcs: true, hide_wound: true },
                        &mut details,
                    );
                });
            }
            started.elapsed()
        }),
    );
    add!(
        "paint/volume-canvas",
        "one headless pass painting the 2D+1 volume of the same state. The focus event does not \
         move between iterations, so the past cone is built on the first of them and cached for \
         the rest; a played frame rebuilds it, which the frame tier measures",
        Box::new(|n| {
            let ctx = headless_context();
            let mut canvas = VolumeCanvas::default();
            let mut time = 0.0;
            let started = Instant::now();
            for _ in 0..n {
                time += FRAME_DT;
                paint_pass(&ctx, time, LEFT_CANVAS_WIDTH, |ui| {
                    canvas.render(
                        ui,
                        &fx.metric,
                        Some(&fx.bob),
                        Some(&fx.alice),
                        fx.clock,
                        CANVAS_HEIGHT,
                        true,
                        1.0,
                        SignalViews { alice: &fx.alice_field, bob: &fx.bob_field },
                        true,
                        FrontStyle { arcs: true, hide_wound: true },
                    );
                });
            }
            started.elapsed()
        }),
    );

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perf::harness::Budget;
    use std::time::Duration;

    #[test]
    fn test_every_micro_benchmark_runs() {
        // One iteration of every registered benchmark, which is the one thing about this tier that
        // can rot silently: a benchmark whose set-up has drifted out of step with the code it
        // measures panics, and nothing else in the suite would notice. It is not a measurement -
        // one sample of one iteration on a debug-assertions build is not a number anybody should
        // read - so nothing is asserted about the times, only that every benchmark produced one.
        let fx = Fixtures::build(true);
        let budget = Budget { samples: 1, min_sample: Duration::ZERO, warmup: Duration::ZERO };
        let mut benches = benches(&fx);
        assert!(benches.len() >= 15, "the tier is a list of hot spots, not a token one");
        for bench in benches.iter_mut() {
            let stats = crate::perf::harness::measure(&mut bench.body, &budget);
            assert!(
                stats.median_ns.is_finite() && stats.median_ns >= 0.0,
                "{} reported {} ns",
                bench.name,
                stats.median_ns
            );
        }
    }
}
