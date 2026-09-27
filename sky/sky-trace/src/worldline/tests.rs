//! The walker against the app and against closed forms.
//!
//! Three kinds of evidence, kept apart because they prove different things:
//!
//! * **The app's own trail.** A save holds the events the app's integrator produced, each with its
//!   proper time and 4-velocity. Walking from an early one to the proper times of later ones must
//!   land on them. Of the two saves in the repository only the golden file's Alice has a trail of a
//!   free fall (31 events, t = 0 to 3 M in frames of 0.1 M); the demo's observers are saved at the
//!   moment of their release, with one trail point each. Golden Bob's two trail points are a hold.
//! * **Closed forms.** Where the repository has no save of a mode - a static observer, a ZAMO, a
//!   circular orbit, a hold followed by a release, a release at rest - a hand-built state is walked
//!   and compared with the answer the geometry gives in closed form, written out here and not taken
//!   from the walker.
//! * **Convergence.** The walker's answer must not move when its own step control is tightened
//!   (`with_call_window`), and must agree with an independent integration of the same geodesic in
//!   proper time (the core's `GeodesicState::step`) as that is refined.

use std::f64::consts::{PI, TAU};
use std::path::PathBuf;

use kerr_equatorial::geodesic::proper_time_between;

use super::*;
use crate::bhl::{self, Release, SavedGeodesic, TrailPoint};

fn repo_file(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

fn golden() -> bhl::Save {
    bhl::read_file(&repo_file("src/save/golden/v1.json")).unwrap()
}

fn near_fall() -> bhl::Save {
    bhl::read_file(&repo_file("demos/near_fall.bhl")).unwrap()
}

/// |g(u, u) + 1|, relative to the size of the terms it is a difference of. Double precision alone
/// delivers g(u, u) to about 1e-16 |u|^2, which only matters on the approach to a freeze onto r-,
/// where u^t runs out towards 1e10.
fn unit_residual(metric: &KerrSchild, e: &Event) -> f64 {
    let scale = e.u.iter().fold(1.0f64, |m, v| m.max(v.abs()));
    (metric.norm(e.r, &e.u) + 1.0).abs() / (scale * scale)
}

/// Every event of a film is unit timelike and future-directed, and sits on its frame's proper time.
fn check_film(metric: &KerrSchild, film: &Film, tau0: f64, step: f64) {
    for (k, e) in film.events.iter().enumerate() {
        let residual = unit_residual(metric, e);
        assert!(
            residual < 1e-9,
            "frame {k}: |g(u,u) + 1| / |u|^2 = {residual:e} at r = {}",
            e.r
        );
        assert!(
            e.u[0] > 0.0,
            "frame {k}: u^t = {} is not future-directed",
            e.u[0]
        );
        let want = tau0 + k as f64 * step;
        assert!(
            (e.tau - want).abs() <= 1e-12 * (1.0 + want.abs()),
            "frame {k} is at tau = {} and should be at {want}",
            e.tau
        );
        assert!(
            (0.0..TAU).contains(&e.phi),
            "frame {k}: phi = {} is not folded",
            e.phi
        );
    }
}

/// The smaller difference of two azimuths across the seam at 2 pi.
fn dphi(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU);
    d.min(TAU - d)
}

/// The largest difference, component by component, between an event and a recorded trail point.
fn deviation(e: &Event, p: &TrailPoint) -> [f64; 4] {
    let du = (0..3).map(|i| (e.u[i] - p.u[i]).abs()).fold(0.0, f64::max);
    [(e.t - p.t).abs(), (e.r - p.r).abs(), dphi(e.phi, p.phi), du]
}

fn max4(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0].max(b[0]),
        a[1].max(b[1]),
        a[2].max(b[2]),
        a[3].max(b[3]),
    ]
}

/// The geodesic state at a recorded trail point: the app's own `restore`.
fn state_at(point: &TrailPoint, energy: f64, l_ang: f64) -> GeodesicState {
    GeodesicState {
        t: point.t,
        r: point.r,
        phi: point.phi,
        tau: point.tau,
        energy,
        l_ang,
        u: point.u,
        stalled: point.stalled,
    }
}

/// Walk from trail point `from` to the proper times of every later one, with the given call
/// window, and return the largest deviation in (t, r, phi, u).
fn walk_the_trail(window: f64, from: usize) -> [f64; 4] {
    let save = golden();
    let metric = save.hole.metric();
    let alice = save.alice.unwrap();
    let geo = alice.geodesic.unwrap();
    let mut walker =
        Worldline::free_fall(&metric, state_at(&alice.trail[from], geo.energy, geo.l_ang))
            .unwrap()
            .with_call_window(window);
    let mut worst = [0.0; 4];
    for p in &alice.trail[from + 1..] {
        let e = walker.event_at(p.tau).unwrap();
        assert!(unit_residual(&metric, &e) < 1e-12);
        worst = max4(worst, deviation(&e, p));
    }
    worst
}

/// The same geodesic integrated in its own proper time by the core's fixed-step RK4, `step`, with
/// `per_unit` substeps per M of proper time, to each target: an integration independent of the
/// coordinate-time one the walker and the app use, sharing only the geodesic equation.
fn proper_time_reference(
    metric: &KerrSchild,
    start: GeodesicState,
    targets: &[f64],
    per_unit: f64,
) -> Vec<GeodesicState> {
    let mut geo = start;
    let mut out = Vec::new();
    for &tau in targets {
        let span = tau - geo.tau;
        let n = (span * per_unit).ceil().max(1.0) as usize;
        let h = span / n as f64;
        for _ in 0..n {
            geo.step(metric, h);
        }
        out.push(geo);
    }
    out
}

// ---------------------------------------------------------------------------------------------
// Against the app's recorded trail
// ---------------------------------------------------------------------------------------------

/// The tolerance on the agreement with the app's trail, in M (t, r, phi) and in units of c (u).
///
/// Both integrators are the core's classical RK4 with the same caps; over this stretch of Alice's
/// fall (r = 5.5 to 3.9 M) the 0.008 r cap binds, 0.044 M down to 0.031 M of coordinate time a
/// substep. Two numbers matter, and they are measured below and printed:
///
/// * The accuracy of that integration - how far the curve moves when its substeps are cut
///   differently - is 1.6e-11 M: it is what tightening the call window moves the walk by, and how
///   far both the walk and the app's trail sit from the core's proper-time RK4 refined until it
///   stops moving (to 2.6e-13).
/// * The agreement with the trail is far better than that, about 1e-13 M, and not by luck: walking
///   *to the trail's own proper times*, each call of the walker ends where one of the app's frame
///   calls ended, so it restarts its substeps on the app's grid, and its last substep of each call,
///   shortened until tau lands, is the app's last substep of the frame to within the landing's
///   1e-13. A walk to other proper times - a film - is on a different grid, and agrees with the
///   app to the accuracy, 1.6e-11 M, not to the agreement.
///
/// The tolerance is set on the accuracy, with a margin of six.
const TRAIL_TOLERANCE: f64 = 1e-10;

#[test]
fn test_walking_from_the_golden_trail_lands_on_the_events_the_app_recorded() {
    let mut worst = [0.0; 4];
    for from in [0, 7, 15, 25] {
        let d = walk_the_trail(f64::INFINITY, from);
        println!(
            "from trail point {from:2}: max |dt| = {:.2e}, |dr| = {:.2e}, |dphi| = {:.2e}, |du| = {:.2e}",
            d[0], d[1], d[2], d[3]
        );
        worst = max4(worst, d);
    }
    for (name, value) in ["t", "r", "phi", "u"].iter().zip(worst) {
        assert!(
            value < TRAIL_TOLERANCE,
            "{name} is off the app's trail by {value:e}"
        );
    }
}

#[test]
fn test_tightening_the_call_window_does_not_move_the_walk_off_the_trail() {
    // The walker's own step control: at most `window` of coordinate time per call, which can only
    // shorten substeps. A window of 0.1 M is the app's own frame here, and puts the walk on the
    // app's substep grid exactly: it must land on the trail to the bit. 0.01 M and 0.002 M cut
    // every substep to a quarter and a twentieth of the one that binds; what that moves is the
    // RK4 truncation error, and it must stay inside the tolerance.
    let loose = walk_the_trail(f64::INFINITY, 0);
    assert_eq!(
        walk_the_trail(0.1, 0),
        [0.0; 4],
        "on the app's grid the walk is the app's"
    );
    for window in [0.01, 0.002] {
        let tight = walk_the_trail(window, 0);
        println!(
            "window {window}: max |dt| = {:.2e}, |dr| = {:.2e}, |dphi| = {:.2e}, |du| = {:.2e}",
            tight[0], tight[1], tight[2], tight[3]
        );
        assert!(tight.iter().all(|&d| d < TRAIL_TOLERANCE), "{tight:?}");
    }
    // And between the loose walk and the tightest: the walker against itself.
    let save = golden();
    let metric = save.hole.metric();
    let alice = save.alice.unwrap();
    let geo = alice.geodesic.unwrap();
    let start = state_at(&alice.trail[0], geo.energy, geo.l_ang);
    let mut a = Worldline::free_fall(&metric, start).unwrap();
    let mut b = Worldline::free_fall(&metric, start)
        .unwrap()
        .with_call_window(0.002);
    let mut moved = 0.0f64;
    for p in &alice.trail[1..] {
        let (ea, eb) = (a.event_at(p.tau).unwrap(), b.event_at(p.tau).unwrap());
        moved = moved
            .max((ea.t - eb.t).abs())
            .max((ea.r - eb.r).abs())
            .max(dphi(ea.phi, eb.phi));
    }
    println!("loose walk against a 0.002 M window: {moved:.2e} M; the trail itself is {loose:?}");
    assert!(moved < TRAIL_TOLERANCE);
}

#[test]
fn test_the_walk_agrees_with_an_independent_proper_time_integration() {
    // The core's proper-time RK4, at 100 and 400 substeps per M, from the same event. Its own
    // change between the two says how converged it is; the walker must sit within that of it.
    let save = golden();
    let metric = save.hole.metric();
    let alice = save.alice.unwrap();
    let geo = alice.geodesic.unwrap();
    let start = state_at(&alice.trail[0], geo.energy, geo.l_ang);
    let targets: Vec<f64> = alice.trail[1..].iter().map(|p| p.tau).collect();
    let coarse = proper_time_reference(&metric, start, &targets, 100.0);
    let fine = proper_time_reference(&metric, start, &targets, 400.0);
    let mut walker = Worldline::free_fall(&metric, start).unwrap();
    let (mut spread, mut off, mut app_off) = (0.0f64, 0.0f64, 0.0f64);
    for ((c, f), p) in coarse.iter().zip(&fine).zip(&alice.trail[1..]) {
        let e = walker.event_at(p.tau).unwrap();
        spread = spread.max((c.t - f.t).abs()).max((c.r - f.r).abs());
        off = off
            .max((e.t - f.t).abs())
            .max((e.r - f.r).abs())
            .max(dphi(e.phi, f.phi));
        app_off = app_off
            .max((p.t - f.t).abs())
            .max((p.r - f.r).abs())
            .max(dphi(p.phi, f.phi));
    }
    println!(
        "proper-time RK4: 100/M against 400/M differ by {spread:.2e}; the walker is {off:.2e} \
         from the 400/M answer and the app's trail {app_off:.2e}"
    );
    assert!(spread < 1e-11, "the reference is not converged: {spread:e}");
    assert!(
        off < TRAIL_TOLERANCE,
        "the walker is {off:e} off the converged reference"
    );
    assert!(
        app_off < TRAIL_TOLERANCE,
        "the app's trail is {app_off:e} off the converged reference"
    );
}

#[test]
fn test_the_hold_reproduces_the_clock_the_app_recorded_for_golden_bob() {
    // Bob waits at r = 4 M for a release at t = 1000 M from infinity, so he is held as a static
    // observer, u^t = 1 / sqrt(1 - 2M/r) = sqrt(2). The app ticked his watch frame by frame to
    // tau = 2.1213203435596433 at t = 3.0000000000000013; the hold, walked from his creation event
    // to that reading, must be at that t.
    let save = golden();
    let metric = save.hole.metric();
    let bob = save.bob.unwrap();
    let from_start = bhl::SavedObserver {
        t: bob.start.t,
        r: bob.start.r,
        phi: bob.start.phi,
        tau: bob.start.tau,
        ..bob.clone()
    };
    let mut walker = Worldline::from_saved(&metric, &from_start).unwrap();
    let e = walker.event_at(bob.tau).unwrap();
    assert_eq!(e.motion, Motion::Holding);
    assert!(
        (e.t - bob.t).abs() < 1e-14,
        "t = {} against the app's {}",
        e.t,
        bob.t
    );
    assert!((e.u[0] - 2f64.sqrt()).abs() < 1e-15);
    assert_eq!((e.r, e.phi), (4.0, 0.0));
}

// ---------------------------------------------------------------------------------------------
// The two repository saves, walked
// ---------------------------------------------------------------------------------------------

#[test]
fn test_golden_bob_is_held_until_his_release_and_then_falls_as_a_raindrop() {
    let save = golden();
    let metric = save.hole.metric();
    let bob = save.bob.unwrap();
    let mut walker = Worldline::from_saved(&metric, &bob).unwrap();
    let tau_release = walker.tau_release().unwrap();
    // The hold's clock, closed form: (1000 - t_saved) / sqrt(2) more than the saved reading.
    let want = bob.tau + (1000.0 - bob.t) / 2f64.sqrt();
    assert!(
        (tau_release - want).abs() < 1e-12,
        "{tau_release} against {want}"
    );

    let step = 0.5;
    let film = walker.walk(step, 100_000);
    check_film(&metric, &film, bob.tau, step);
    let first_fall = film
        .events
        .iter()
        .position(|e| e.motion == Motion::FreeFall)
        .unwrap();
    let last_hold = &film.events[first_fall - 1];
    assert!(last_hold.tau < tau_release && film.events[first_fall].tau >= tau_release);
    assert!(last_hold.t < 1000.0 && last_hold.r == 4.0);
    // After the release: the E = 1, L = 0 geodesic from r = 4 M at t = 1000 M. Its proper time
    // from the release to each frame is the quadrature of dtau = r^2 dr / sqrt(R) from 4 M to the
    // frame's radius, which needs no worldline.
    let mut worst = 0.0f64;
    for e in &film.events[first_fall..] {
        let law = proper_time_between(&metric, 1.0, 0.0, 4.0, e.r).unwrap();
        worst = worst.max((e.tau - tau_release - law).abs());
        assert!(e.t >= 1000.0);
    }
    println!(
        "golden Bob: held for {first_fall} frames, released at tau = {tau_release:.6} M, then \
         {} frames of fall; proper time off the quadrature by at most {worst:.2e} M; {}",
        film.events.len() - first_fall,
        film.end
    );
    assert!(worst < 1e-9);
    assert!(matches!(film.end, End::InnerHorizon { .. }), "{}", film.end);
}

#[test]
fn test_the_demo_observers_fall_from_rest_to_the_inner_horizon() {
    let save = near_fall();
    let metric = save.hole.metric();
    for obs in [save.alice.unwrap(), save.bob.unwrap()] {
        let mut walker = Worldline::from_saved(&metric, &obs).unwrap();
        let (energy, l_ang) = walker.constants().unwrap();
        let step = 0.01;
        let film = walker.walk(step, 100_000);
        check_film(&metric, &film, 0.0, step);
        assert!(matches!(film.end, End::InnerHorizon { .. }), "{}", film.end);
        // Proper time between any two frames away from the turning point at the start, against the
        // quadrature of the radial law for the observer's own (E, L).
        let events = &film.events;
        let mut worst = 0.0f64;
        for pair in events[10..].windows(2) {
            let law = proper_time_between(&metric, energy, l_ang, pair[0].r, pair[1].r).unwrap();
            worst = worst.max((pair[1].tau - pair[0].tau - law).abs());
        }
        // And against the core's independent proper-time RK4 at the last frame.
        let start = GeodesicState {
            t: obs.t,
            r: obs.r,
            phi: obs.phi,
            tau: obs.tau,
            energy,
            l_ang,
            u: obs.geodesic.unwrap().u,
            stalled: false,
        };
        let last = events.last().unwrap();
        let fine = proper_time_reference(&metric, start, &[last.tau], 20_000.0)[0];
        let finer = proper_time_reference(&metric, start, &[last.tau], 80_000.0)[0];
        println!(
            "{}: {} frames of {step} M to r = {:.6} M; |dtau| against the quadrature {worst:.2e}; \
             last frame off the proper-time RK4 by dt = {:.2e}, dr = {:.2e} (that RK4's own \
             refinement moves it {:.2e}); {}",
            obs.name,
            events.len(),
            last.r,
            (last.t - finer.t).abs(),
            (last.r - finer.r).abs(),
            (fine.t - finer.t).abs().max((fine.r - finer.r).abs()),
            film.end
        );
        assert!(worst < 1e-9);
        assert!((last.t - finer.t).abs() < 1e-8 && (last.r - finer.r).abs() < 1e-8);
    }
}

// ---------------------------------------------------------------------------------------------
// Closed forms, for the modes the repository's saves do not exercise
// ---------------------------------------------------------------------------------------------

/// A saved observer built by hand, as the app would have written it.
fn observer(
    mode: Mode,
    t: f64,
    r: f64,
    tau: f64,
    geo: GeodesicState,
    release_t: f64,
    is_active: bool,
) -> bhl::SavedObserver {
    let g = SavedGeodesic {
        t: geo.t,
        r: geo.r,
        phi: geo.phi,
        tau: geo.tau,
        energy: geo.energy,
        l_ang: geo.l_ang,
        u: geo.u,
        stalled: geo.stalled,
    };
    let point = TrailPoint {
        t,
        r,
        phi: geo.phi,
        tau,
        u: geo.u,
        stalled: false,
    };
    bhl::SavedObserver {
        name: "Bob".into(),
        mode,
        t,
        r,
        phi: geo.phi,
        tau,
        geodesic: Some(g),
        trail: vec![point],
        start: point,
        release_t,
        release: Release::FromInfinity,
        is_active,
    }
}

#[test]
fn test_a_static_observer_ticks_at_the_static_dilation() {
    // u^t = 1 / sqrt(1 - 2M/r) on the equator, whatever the spin; phi does not move.
    let metric = KerrSchild::new(1.0, 0.9);
    let r = 6.0;
    let geo = GeodesicState::new_infall(&metric, 10.0, r, 1.0, 0.0);
    let mut walker = Worldline::from_saved(
        &metric,
        &observer(Mode::Static, 10.0, r, 3.0, geo, 0.0, true),
    )
    .unwrap();
    let film = walker.walk(0.7, 50);
    check_film(&metric, &film, 3.0, 0.7);
    let ut = 1.0 / (1.0 - 2.0 / r).sqrt();
    for (k, e) in film.events.iter().enumerate() {
        assert_eq!(e.motion, Motion::Static);
        assert!((e.t - (10.0 + ut * 0.7 * k as f64)).abs() < 1e-12);
        assert_eq!((e.r, e.phi, e.u[1], e.u[2]), (r, 0.0, 0.0, 0.0));
    }
    assert_eq!(film.end, End::Frames { count: 50 });
}

#[test]
fn test_a_zamo_ticks_at_the_lapse_and_turns_with_the_dragging() {
    // Equatorial Kerr, M = 1: A = (r^2 + a^2)^2 - a^2 Delta, lapse^2 = r^2 Delta / A, omega =
    // 2 a r / A. The ZAMO's u^t is 1 / lapse and it turns at dphi/dt = omega. A fixed-r worldline
    // has the same (t, phi) block in Boyer-Lindquist and ingoing Kerr-Schild, so these are the
    // chart's.
    let (a, r) = (0.9, 3.0);
    let metric = KerrSchild::new(1.0, a);
    let delta = r * r - 2.0 * r + a * a;
    let big_a = (r * r + a * a).powi(2) - a * a * delta;
    let ut = 1.0 / (r * r * delta / big_a).sqrt();
    let omega = 2.0 * a * r / big_a;
    let geo = GeodesicState::new_infall(&metric, 0.0, r, 1.0, 0.0);
    let mut walker =
        Worldline::from_saved(&metric, &observer(Mode::Zamo, 0.0, r, 0.0, geo, 0.0, true)).unwrap();
    let film = walker.walk(0.25, 400);
    check_film(&metric, &film, 0.0, 0.25);
    for (k, e) in film.events.iter().enumerate() {
        let tau = 0.25 * k as f64;
        assert!((e.t - ut * tau).abs() < 1e-11 * (1.0 + e.t));
        assert!(dphi(e.phi, omega * ut * tau) < 1e-11 * (1.0 + e.t));
        assert!((e.u[0] - ut).abs() < 1e-13 && (e.u[2] - omega * ut).abs() < 1e-13);
    }
}

#[test]
fn test_a_circular_orbit_keeps_its_radius_and_its_period() {
    // Bardeen, Press and Teukolsky, M = 1: Omega = 1 / (r^{3/2} + a) prograde, and
    // u^t = (r^{3/2} + a) / (r^{3/4} sqrt(r^{3/2} - 3 r^{1/2} + 2a)). The orbit is started exactly
    // on it (u^r = 0) and walked through three periods of proper time.
    for (a, r) in [(0.9f64, 6.0f64), (0.0, 7.0), (0.9, 2.5)] {
        let metric = KerrSchild::new(1.0, a);
        let omega = 1.0 / (r.powf(1.5) + a);
        let ut =
            (r.powf(1.5) + a) / (r.powf(0.75) * (r.powf(1.5) - 3.0 * r.sqrt() + 2.0 * a).sqrt());
        let (energy, l_ang) = metric.circular_orbit(r, true).unwrap();
        let geo = GeodesicState {
            t: 0.0,
            r,
            phi: 0.0,
            tau: 0.0,
            energy,
            l_ang,
            u: [ut, 0.0, omega * ut],
            stalled: false,
        };
        let period_tau = TAU / (omega * ut);
        let frames = 3 * 64 + 1;
        let step = period_tau / 64.0;
        let film = Worldline::free_fall(&metric, geo)
            .unwrap()
            .walk(step, frames);
        check_film(&metric, &film, 0.0, step);
        let (mut dr, mut dt, mut dp) = (0.0f64, 0.0f64, 0.0f64);
        for (k, e) in film.events.iter().enumerate() {
            let tau = step * k as f64;
            dr = dr.max((e.r - r).abs());
            dt = dt.max((e.t - ut * tau).abs());
            dp = dp.max(dphi(e.phi, omega * ut * tau));
        }
        println!(
            "circular orbit a = {a}, r = {r}: three periods (t = {:.3} M); |dr| {dr:.2e}, |dt| \
             {dt:.2e}, |dphi| {dp:.2e}",
            3.0 * TAU / omega
        );
        assert!(dr < 1e-8 && dt < 1e-8 && dp < 1e-8, "a = {a}, r = {r}");
        // After three whole periods the observer is back at phi = 0.
        assert!(dphi(film.events.last().unwrap().phi, 0.0) < 1e-8);
    }
}

/// The cycloid of a fall from rest at r0 with L = 0 in Schwarzschild: r = (r0/2)(1 + cos eta),
/// tau = sqrt(r0^3 / 8)(eta + sin eta). Returns (r, u^r) at proper time `tau` after the release,
/// solving for eta by bisection (tau is increasing in eta on [0, pi]).
fn cycloid(r0: f64, tau: f64) -> (f64, f64) {
    let scale = (r0.powi(3) / 8.0).sqrt();
    let (mut lo, mut hi) = (0.0f64, PI);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if scale * (mid + mid.sin()) < tau {
            lo = mid
        } else {
            hi = mid
        }
    }
    let eta = 0.5 * (lo + hi);
    let r = 0.5 * r0 * (1.0 + eta.cos());
    let ur = -0.5 * r0 * eta.sin() / (scale * (1.0 + eta.cos()));
    (r, ur)
}

#[test]
fn test_a_hold_then_a_release_at_rest_joins_the_cycloid_without_a_jump() {
    // Schwarzschild, dropped from rest at r0 = 8 M after a wait of 5 M of coordinate time. The hold
    // is the at-rest worldline, which at L = 0 is the static observer, u^t = 1 / sqrt(1 - 2M/r0);
    // the fall is the cycloid; the release joins them with no jump in any component of u.
    let metric = KerrSchild::new(1.0, 0.0);
    let (r0, wait) = (8.0f64, 5.0);
    let energy = (1.0 - 2.0 / r0).sqrt();
    let seed = GeodesicState::new_infall(&metric, wait, r0, energy, 0.0);
    let obs = observer(Mode::FreeFall, 0.0, r0, 0.0, seed, wait, false);
    let mut walker = Worldline::from_saved(&metric, &obs).unwrap();
    let tau_release = walker.tau_release().unwrap();
    assert!((tau_release - wait * energy).abs() < 1e-14);

    let step = 0.25;
    let film = walker.walk(step, 1000);
    check_film(&metric, &film, 0.0, step);
    let (mut worst_r, mut worst_u) = (0.0f64, 0.0f64);
    for e in &film.events {
        if e.tau < tau_release {
            assert_eq!(e.motion, Motion::Holding);
            assert!((e.t - e.tau / energy).abs() < 1e-13 && e.r == r0);
            continue;
        }
        let (r, ur) = cycloid(r0, e.tau - tau_release);
        worst_r = worst_r.max((e.r - r).abs());
        worst_u = worst_u.max((e.u[1] - ur).abs());
    }
    // The ring. The integration stops at the first substep ending inside r = R_STOP, at radius
    // r_end; the cycloid's proper time at that radius, from (r0/2)(1 + cos eta) = r_end, is where
    // it should have stopped.
    let End::Ring { tau, r: r_end } = film.end else {
        panic!("{}", film.end)
    };
    assert!(
        r_end <= R_STOP && r_end > 0.5 * R_STOP,
        "stopped at r = {r_end}"
    );
    let eta = (2.0 * r_end / r0 - 1.0).acos();
    let tau_ring = tau_release + (r0.powi(3) / 8.0).sqrt() * (eta + eta.sin());
    println!(
        "release at rest: {} frames; r off the cycloid by at most {worst_r:.2e} M, u^r by \
         {worst_u:.2e}; stopped at r = {r_end:.6} M at tau = {tau:.12}, the cycloid's tau there \
         {tau_ring:.12}",
        film.events.len()
    );
    assert!(worst_r < 1e-8 && worst_u < 1e-7);
    assert!((tau - tau_ring).abs() < 1e-9);
    assert!(film.events.last().unwrap().tau < tau && tau < film.events.last().unwrap().tau + step);
    // No jump at the release: the hold's u and the fall's u at the release event are one vector.
    let mut probe = Worldline::from_saved(&metric, &obs).unwrap();
    let before = probe.event_at(tau_release - 1e-9).unwrap();
    let after = probe.event_at(tau_release).unwrap();
    for i in 0..3 {
        assert!(
            (before.u[i] - after.u[i]).abs() < 1e-7,
            "u[{i}] jumps: {:?} -> {:?}",
            before.u,
            after.u
        );
    }
    assert!((after.t - wait).abs() < 1e-12 && after.motion == Motion::FreeFall);
}

#[test]
fn test_a_hold_then_a_release_from_infinity_jumps_onto_the_raindrop() {
    // Schwarzschild, r0 = 6 M, released at t = 2 M as if fallen from infinity: the hold is the
    // static observer, and the fall is the raindrop, r^{3/2} = r0^{3/2} - (3/2) sqrt(2M) (tau -
    // tau_release), u^r = -sqrt(2M/r). The release jumps u^r from 0 to -sqrt(2M/r0), as in the app.
    let metric = KerrSchild::new(1.0, 0.0);
    let (r0, release_t) = (6.0, 2.0);
    let seed = GeodesicState::new_infall(&metric, release_t, r0, 1.0, 0.0);
    let obs = observer(Mode::FreeFall, 0.5, r0, 0.1, seed, release_t, false);
    let mut walker = Worldline::from_saved(&metric, &obs).unwrap();
    let static_ut = 1.0 / (1.0 - 2.0 / r0).sqrt();
    let tau_release = walker.tau_release().unwrap();
    assert!((tau_release - (0.1 + 1.5 / static_ut)).abs() < 1e-14);
    let step = 0.125;
    let film = walker.walk(step, 1000);
    check_film(&metric, &film, 0.1, step);
    let mut worst = 0.0f64;
    for e in film.events.iter().filter(|e| e.motion == Motion::FreeFall) {
        let r = (r0.powf(1.5) - 1.5 * 2f64.sqrt() * (e.tau - tau_release)).powf(2.0 / 3.0);
        worst = worst
            .max((e.r - r).abs())
            .max((e.u[1] + (2.0 / e.r).sqrt()).abs());
    }
    println!(
        "release from infinity: off the raindrop by at most {worst:.2e}; {}",
        film.end
    );
    assert!(worst < 1e-8);
    let first = film
        .events
        .iter()
        .find(|e| e.motion == Motion::FreeFall)
        .unwrap();
    assert!(first.t > release_t);
    assert!(matches!(film.end, End::Ring { .. }));
}

#[test]
fn test_a_static_selection_held_until_release_switches_at_the_release_event_exactly() {
    // Held on an at-rest worldline with angular momentum (so the hold turns, u^phi != 0) until
    // t = 4, then static. The app ticks the whole step that crosses the release at the static
    // rate; the exact switch is at t = release_t, and every later frame follows from there.
    let metric = KerrSchild::new(1.0, 0.9);
    let (r, release_t, l_ang) = (5.0, 4.0, 2.0);
    let floor = GeodesicState::energy_floor(&metric, r, l_ang);
    let seed = GeodesicState::new_infall(&metric, release_t, r, floor, l_ang);
    let hold_u = seed.u;
    assert!(hold_u[1].abs() < 1e-6 && hold_u[2] != 0.0);
    let obs = observer(Mode::Static, 1.0, r, 0.0, seed, release_t, false);
    let mut walker = Worldline::from_saved(&metric, &obs).unwrap();
    let tau_release = 3.0 / hold_u[0];
    assert!((walker.tau_release().unwrap() - tau_release).abs() < 1e-14);
    let static_ut = 1.0 / (1.0 - 2.0 / r).sqrt();
    let phi_release = hold_u[2] / hold_u[0] * 3.0;
    let film = walker.walk(0.3, 40);
    check_film(&metric, &film, 0.0, 0.3);
    for e in &film.events {
        if e.tau < tau_release {
            assert_eq!(e.motion, Motion::Holding);
            assert!((e.t - (1.0 + hold_u[0] * e.tau)).abs() < 1e-13);
            assert!(dphi(e.phi, hold_u[2] * e.tau) < 1e-13);
        } else {
            assert_eq!(e.motion, Motion::Static);
            assert!((e.t - (release_t + static_ut * (e.tau - tau_release))).abs() < 1e-12);
            assert!(
                dphi(e.phi, phi_release) < 1e-13,
                "a static observer does not turn"
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The ends
// ---------------------------------------------------------------------------------------------

#[test]
fn test_a_freeze_onto_the_inner_horizon_ends_the_film_before_the_stall() {
    // E = 1, L = 2.2 at a = 0.9 has E - Omega_- L < 0, so it never crosses r-: it asymptotes to it
    // in coordinate time and the core declares the stall at u^t = 1e10. Every frame is outside r-,
    // and none is stalled.
    let metric = KerrSchild::new(1.0, 0.9);
    let r_minus = metric.inner_horizon();
    let geo = GeodesicState::new_infall(&metric, 0.0, 9.0, 1.0, 2.2);
    let step = 0.05;
    let film = Worldline::free_fall(&metric, geo)
        .unwrap()
        .walk(step, 100_000);
    check_film(&metric, &film, 0.0, step);
    let End::Stalled { tau, r } = film.end else {
        panic!("{}", film.end)
    };
    let last = film.events.last().unwrap();
    println!(
        "freeze: {} frames, last at r - r- = {:.3e} M with u^t = {:.3e}; stalled at tau = {tau:.9}, \
         r - r- = {:.3e}",
        film.events.len(),
        last.r - r_minus,
        last.u[0],
        r - r_minus
    );
    assert!(film.events.iter().all(|e| e.r > r_minus));
    assert!(last.tau < tau && tau < last.tau + step);
    assert!((r - r_minus).abs() < 1e-8);
}

#[test]
fn test_a_static_selection_the_radius_does_not_admit_falls_until_it_does() {
    // Static selected at r = 1.9 M, inside the static limit, where no static observer exists: the
    // app follows the geodesic the observer carries, here an outgoing one, and switches back to
    // Static at the first frame outside r = 2M. The film ends there.
    let metric = KerrSchild::new(1.0, 0.9);
    let geo = GeodesicState::new_with_direction(&metric, 0.0, 1.9, 1.2, 0.0, true);
    assert!(geo.u[1] > 0.0);
    let mut walker = Worldline::from_saved(
        &metric,
        &observer(Mode::Static, 0.0, 1.9, 0.0, geo, 0.0, true),
    )
    .unwrap();
    assert_eq!(walker.motion_after_release(), Motion::FreeFall);
    let film = walker.walk(0.01, 10_000);
    check_film(&metric, &film, 0.0, 0.01);
    let End::ModeResumes { mode, r, .. } = film.end else {
        panic!("{}", film.end)
    };
    assert_eq!(mode, Mode::Static);
    assert!(r > 2.0 && film.events.iter().all(|e| e.r <= 2.0));
}

#[test]
fn test_observers_the_walker_cannot_film_are_refused_with_a_sentence() {
    let save = near_fall();
    let metric = save.hole.metric();
    let bob = save.bob.unwrap();

    let dragged = bhl::SavedObserver {
        mode: Mode::ManualDrag,
        ..bob.clone()
    };
    let why = Worldline::from_saved(&metric, &dragged).unwrap_err();
    assert!(why.0.contains("dragged by hand"), "{why}");

    let inside = bhl::SavedObserver {
        r: 0.5,
        ..bob.clone()
    };
    let why = Worldline::from_saved(&metric, &inside).unwrap_err();
    assert!(why.0.contains("inner horizon"), "{why}");

    let mut frozen = bob.clone();
    frozen.geodesic.as_mut().unwrap().stalled = true;
    let why = Worldline::from_saved(&metric, &frozen).unwrap_err();
    assert!(why.0.contains("frozen"), "{why}");

    // Waiting inside the static limit for a release from infinity: nothing holds that radius.
    let seed = GeodesicState::new_infall(&metric, 50.0, 1.8, 1.0, 0.0);
    let waiting = observer(Mode::FreeFall, 0.0, 1.8, 0.0, seed, 50.0, false);
    let why = Worldline::from_saved(&metric, &waiting).unwrap_err();
    assert!(why.0.contains("static limit"), "{why}");

    let broken = bhl::SavedObserver {
        tau: f64::NAN,
        ..bob
    };
    let why = Worldline::from_saved(&metric, &broken).unwrap_err();
    assert!(why.0.contains("not finite"), "{why}");
}
