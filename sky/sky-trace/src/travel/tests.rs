//! The direction of travel against closed forms, the app's own figures, and an independent
//! evaluation in double-double arithmetic.

use std::f64::consts::{FRAC_PI_2, PI};

use kerr_equatorial::{GeodesicState, KerrSchild};
use kerr_sky::Kerr;

use super::dd::{self, Dd};
use super::*;
use crate::film;
use crate::units::Units;
use crate::worldline::{Motion, Worldline};

/// 0.01 degree, the precision the owner asked for, in radians.
const HUNDREDTH_OF_A_DEGREE: f64 = 0.01 * PI / 180.0;

fn kerr(a: f64) -> Kerr {
    Kerr::new(1.0, a)
}

/// An event at (r, phi = 0.3) with 4-velocity u.
fn event(r: f64, u: [f64; 3]) -> Event {
    Event {
        tau: 0.0,
        t: 0.0,
        r,
        phi: 0.3,
        u,
        motion: Motion::FreeFall,
    }
}

/// The passings at an event, through the film's own triad.
fn passings(kerr: &Kerr, r: f64, u: [f64; 3]) -> Vec<Passing> {
    let e = event(r, u);
    let triad = film::triad(kerr, &e).unwrap();
    passing(kerr, &e, &triad)
}

fn past(passings: &[Passing], reference: Reference) -> Passing {
    *passings
        .iter()
        .find(|p| p.reference == reference)
        .unwrap_or_else(|| panic!("no passing of {reference:?} in {passings:?}"))
}

fn references(passings: &[Passing]) -> Vec<Reference> {
    passings.iter().map(|p| p.reference).collect()
}

/// Angle between two unit vectors, without the loss of acos near 0 and pi.
fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        .sqrt()
        .atan2(dot)
}

fn raindrop_u(kerr: &Kerr, r: f64) -> [f64; 3] {
    GeodesicState::new_infall(&kerr.equatorial(), 0.0, r, 1.0, 0.0).u
}

#[test]
fn test_delta_is_good_to_its_last_bits_next_to_either_horizon() {
    // Against Delta evaluated in double-double, at radii from 1e-3 down to a few ulps from r+ and
    // r-, where the direct formula keeps fewer and fewer digits.
    let mut worst = 0.0f64;
    let mut worst_direct = 0.0f64;
    for a in [0.3, 0.9, 0.998] {
        let metric = KerrSchild {
            m: 1.0,
            a,
            m_solar: 1.0,
        };
        for horizon in [metric.outer_horizon(), metric.inner_horizon()] {
            let mut radii = vec![horizon];
            for k in 1..=4 {
                radii.push(f64::from_bits(horizon.to_bits() + k));
                radii.push(f64::from_bits(horizon.to_bits() - k));
            }
            for offset in [1e-3, 1e-6, 1e-9, 1e-12, 1e-14] {
                radii.extend([horizon + offset, horizon - offset]);
            }
            for r in radii {
                let (rd, ad) = (Dd::new(r), Dd::new(a));
                let exact = (rd * rd - Dd::new(2.0) * rd + ad * ad).to_f64();
                if exact == 0.0 {
                    assert_eq!(delta(1.0, a, r), 0.0);
                    continue;
                }
                let miss = ((delta(1.0, a, r) - exact) / exact).abs();
                worst = worst.max(miss);
                worst_direct = worst_direct.max(((metric.delta(r) - exact) / exact).abs());
            }
        }
    }
    println!(
        "Delta next to the horizons: relative error {worst:.1e}, against {worst_direct:.1e} for \
         the direct formula"
    );
    assert!(worst <= 2.0 * f64::EPSILON, "{worst}");
    assert!(
        worst_direct > 1e-3,
        "the direct formula was expected to lose digits there"
    );
}

#[test]
fn test_the_hover_tests_observer_is_at_rest_past_the_static_observer_and_retrograde_past_the_zamo()
{
    // The hover test's static observer at 6 M around a = 0.9. Past the static observer: at rest,
    // speed 0, no heading, no mark. Past the ZAMO, which the dragging carries round prograde, it
    // travels retrograde: straight to the viewer's left, heading -90 degrees, at the ZAMO's
    // speed relative to the stars, v = omega g_phiphi / sqrt(Delta) = 2 M a / (r sqrt(Delta))
    // (omega g_phiphi = -g_tphi = 2 M a / r; alpha = sqrt(Delta / g_phiphi)).
    let (r, a) = (6.0, 0.9);
    let metric = KerrSchild {
        m: 1.0,
        a,
        m_solar: 1.0,
    };
    let observer = film::hover_observer(&metric, r).unwrap();
    let e = Worldline::from_saved(&metric, &observer)
        .unwrap()
        .event_at(0.0)
        .unwrap();
    let kerr = Kerr::from_equatorial(&metric);
    let p = passing(&kerr, &e, &film::triad(&kerr, &e).unwrap());
    assert_eq!(references(&p), [Reference::Static, Reference::Zamo]);
    let at_rest = past(&p, Reference::Static);
    assert_eq!(
        (
            at_rest.speed,
            at_rest.celerity,
            at_rest.gamma,
            at_rest.direction
        ),
        (0.0, 0.0, 1.0, None),
        "the static observer does not move past the static observer"
    );
    let zamo = past(&p, Reference::Zamo);
    let want = 2.0 * a / (r * (r * r - 2.0 * r + a * a).sqrt());
    let heading = zamo.heading().unwrap();
    println!(
        "static at 6 M, a = 0.9: past the ZAMO at {:.12} c (closed form {want:.12}), heading \
         {heading}",
        zamo.speed
    );
    assert!(
        (zamo.speed - want).abs() < 1e-14,
        "{} against {want}",
        zamo.speed
    );
    assert!((heading + 90.0).abs() < 1e-12, "{heading}");
    assert_eq!(zamo.direction.unwrap()[2], 0.0);

    // What a film of it writes: no static mark or heading, a speed of 0, and the ZAMO's.
    let plan = film::plan(&kerr, "Bob", &[e], &[0.0], 20.44, Units::Physical).unwrap();
    let entry = film::entry(0, 0.0, 0.0, &e, &plan);
    assert_eq!(entry.readouts["speed_static"].0, 0.0);
    assert!(!entry.readouts.contains_key("heading_static"));
    assert!(!entry.marks.contains_key("travel_static"));
    assert!(entry.marks.contains_key("travel_zamo"));
    let marks: Vec<&str> = plan.marks.iter().map(|m| m.id.as_str()).collect();
    assert_eq!(
        marks,
        ["travel_zamo"],
        "only the marks that occur are declared"
    );

    // Around a hole that does not spin the ZAMO is the static observer, and both are at rest.
    let metric = KerrSchild {
        m: 1.0,
        a: 0.0,
        m_solar: 1.0,
    };
    let kerr = Kerr::from_equatorial(&metric);
    let observer = film::hover_observer(&metric, r).unwrap();
    let e = Worldline::from_saved(&metric, &observer)
        .unwrap()
        .event_at(0.0)
        .unwrap();
    for p in passing(&kerr, &e, &film::triad(&kerr, &e).unwrap()) {
        assert_eq!((p.speed, p.direction), (0.0, None), "{p:?}");
    }
}

#[test]
fn test_a_prograde_orbit_travels_to_the_right_at_the_apps_speeds_at_the_isco() {
    // The prograde circular orbit at the ISCO of a = 0.9 (r = 2.3209 M, a whisker outside the
    // static limit): heading +90 degrees past both hovering observers, and the speeds the app
    // quotes, 0.898 c past the static observer and 0.625 c past the ZAMO. The closed forms:
    //
    //   past the static observer, v = Omega sqrt(Delta) / (-g_tt - g_tphi Omega)
    //                               = Omega sqrt(Delta) / (1 - 2M/r + 2 M a Omega / r);
    //   past the ZAMO, Bardeen, Press and Teukolsky (1972), eq. 3.10:
    //       v = (r^2 - 2 a sqrt(M r) + a^2) / (sqrt(Delta) (r^(3/2) + a sqrt(M))).
    let a = 0.9;
    let kerr = kerr(a);
    let eq = kerr.equatorial();
    let r = eq.isco(true);
    let omega = eq.orbital_angular_velocity(r, true).unwrap();
    let ut = eq.circular_orbit_dilation(r, true).unwrap();
    let delta = r * r - 2.0 * r + a * a;
    let v_static = omega * delta.sqrt() / (1.0 - 2.0 / r + 2.0 * a * omega / r);
    let v_zamo = (r * r - 2.0 * a * r.sqrt() + a * a) / (delta.sqrt() * (r.powf(1.5) + a));
    let p = passings(&kerr, r, [ut, 0.0, ut * omega]);
    assert_eq!(references(&p), [Reference::Static, Reference::Zamo]);
    let (s, z) = (past(&p, Reference::Static), past(&p, Reference::Zamo));
    println!(
        "prograde ISCO of a = 0.9 at r = {r:.9} M: {:.10} c past the static observer (closed form \
         {v_static:.10}), {:.10} c past the ZAMO ({v_zamo:.10}); headings {} and {}",
        s.speed,
        z.speed,
        s.heading().unwrap(),
        z.heading().unwrap()
    );
    // The app's figures, and the same to ten places.
    assert_eq!(format!("{:.3}", s.speed), "0.898");
    assert_eq!(format!("{:.3}", z.speed), "0.625");
    assert!((v_static - 0.897_786_789_1).abs() < 1e-10, "{v_static}");
    assert!((v_zamo - 0.624_548_695_5).abs() < 1e-10, "{v_zamo}");
    assert!((s.speed - v_static).abs() < 1e-12, "{}", s.speed);
    assert!((z.speed - v_zamo).abs() < 1e-12, "{}", z.speed);
    for q in [s, z] {
        let heading = q.heading().unwrap();
        assert!((heading - 90.0).abs() < 1e-9, "{q:?}: {heading}");
    }
}

#[test]
fn test_the_raindrop_falls_straight_past_the_zamo_and_aslant_past_the_static_observer() {
    // The raindrop has L = 0, and so no motion across the ZAMO's radial direction: past the ZAMO
    // it travels straight at the hole, heading 0. The static observer is not turning with the
    // dragging, so past it the raindrop also moves prograde: a heading to the right, which the
    // double-double evaluation gives independently, and which vanishes when the hole does not
    // spin.
    for (a, r, want) in [
        (0.9, 6.0, 4.895_117_502),
        (0.9, 2.5, 16.854_704_292),
        (0.0, 6.0, 0.0),
    ] {
        let kerr = kerr(a);
        let u = raindrop_u(&kerr, r);
        let p = passings(&kerr, r, u);
        let zamo = past(&p, Reference::Zamo).heading().unwrap();
        let fixed = past(&p, Reference::Static).heading().unwrap();
        let m = dd::Metric::new(1.0, a, r);
        let (n, _) = m.travel(&dd::exact(u), &m.static_observer());
        let independent = heading_of(n);
        println!(
            "raindrop at r = {r}, a = {a}: heading {zamo:.1e} past the ZAMO, {fixed:.10} past \
             the static observer (double-double: {independent:.10})"
        );
        assert!(zamo.abs() < 1e-9, "{zamo}");
        assert!(
            (fixed - independent).abs() < 1e-9,
            "{fixed} against {independent}"
        );
        assert!((fixed - want).abs() < 1e-6, "{fixed} against {want}");
    }
}

#[test]
fn test_only_the_reference_observers_that_exist_are_quoted() {
    // a = 0.9: the static limit at 2 M, r+ = 1.436 M, r- = 0.564 M. Bob's fall in the
    // demonstration save (E = 0.46, L = 0, from rest at 2.266 M) at 2.2 M, in the ergosphere at
    // 1.8 M, and between the horizons at 1 M.
    let a = 0.9;
    let kerr = kerr(a);
    let eq = kerr.equatorial();
    let bob = |r: f64| GeodesicState::new_infall(&eq, 0.0, r, 0.46, 0.0).u;
    for (r, want) in [
        (2.2, vec![Reference::Static, Reference::Zamo]),
        (1.8, vec![Reference::Zamo]),
        (1.0, vec![Reference::Raindrop]),
    ] {
        let p = passings(&kerr, r, bob(r));
        assert_eq!(references(&p), want, "at r = {r}");
        for q in &p {
            assert!(q.direction.is_some() && q.speed > 0.1, "at r = {r}: {q:?}");
        }
    }
    // The raindrop between the horizons is at rest past the only observer there is: no mark, a
    // speed of 0, and nothing but that speed in the frame's entry.
    let u = raindrop_u(&kerr, 1.0);
    let p = passings(&kerr, 1.0, u);
    assert_eq!(references(&p), [Reference::Raindrop]);
    assert_eq!((p[0].speed, p[0].direction), (0.0, None));
    let (readouts, marks) = values(&p, &written_headings(&[p.clone(), p.clone()])[0]);
    assert_eq!(readouts, [("speed_raindrop".to_string(), 0.0)]);
    assert!(marks.is_empty());
    let (declared, marks) = declarations(&[p]);
    assert_eq!(declared.len(), 1, "a speed and nothing else: {declared:?}");
    assert!(marks.is_empty());
    // And outside r+ the ZAMO and the static observer, where they exist, whoever the observer.
    assert!(exists(&eq, Reference::Zamo, eq.outer_horizon() + 1e-12));
    assert!(!exists(&eq, Reference::Zamo, eq.outer_horizon()));
    assert!(exists(&eq, Reference::Static, 2.0 + 1e-12));
    assert!(!exists(&eq, Reference::Static, 2.0));
}

#[test]
fn test_every_direction_agrees_with_a_double_double_evaluation_to_a_hundredth_of_a_degree() {
    // Observers built exactly in double-double and handed to the program rounded to f64, as a
    // save hands them: the raindrop boosted to Lorentz factors from 1 to 1e9 in six directions,
    // the static observer and the ZAMO themselves, and the ZAMO boosted to celerities from 1e-9
    // to 30, at radii from 30 M down to 1e-12 M outside r+, and between the horizons down to
    // 1e-9 of their gap above r-. Every direction the program gives is compared with the literal
    // construction in double-double (`dd::Metric::travel`), and with the literal construction in
    // f64 on `kerr_sky`'s own triad legs where the boost is small enough for f64 to do it, which
    // ties the handedness to the triad the picture is drawn with.
    let mut worst = (0.0f64, String::new());
    let mut worst_raindrop = (0.0f64, String::new());
    let mut worst_direct = 0.0f64;
    let mut given = 0;
    // The slowest motion given a direction, and the fastest left without one, each with the
    // size of the observer's u that sets how well f64 holds it.
    let mut slowest_given = (f64::INFINITY, String::new());
    let mut fastest_omitted = (0.0f64, String::new());
    let mut omitted = 0;
    for a in [0.0, 0.5, 0.9, 0.998] {
        let kerr = kerr(a);
        let eq = kerr.equatorial();
        let (rp, rm) = (eq.outer_horizon(), eq.inner_horizon());
        let mut radii = vec![30.0, 6.0, 2.5, 2.0 + 1e-6, 1.9];
        radii.extend([1e-3, 1e-6, 1e-9, 1e-12].map(|d| rp + d));
        if a > 0.0 {
            let gap = rp - rm;
            radii.extend([0.5 * (rp + rm), rm + 1e-3 * gap, rm + 1e-9 * gap]);
        } else {
            radii.extend([1.0, 0.5]);
        }
        for r in radii {
            let m = dd::Metric::new(1.0, a, r);
            let mut observers: Vec<(String, dd::Vector)> = Vec::new();
            for gamma in [1.0, 1.0001, 1.5, 10.0, 1e3, 1e6, 1e9] {
                for chi in [0.0, 0.7, 1.9, PI, 4.0, 5.5] {
                    observers.push((
                        format!("the raindrop boosted to gamma {gamma:e} at {chi}"),
                        m.boosted_raindrop(gamma, chi),
                    ));
                }
            }
            if exists(&eq, Reference::Static, r) {
                observers.push(("the static observer".into(), m.static_observer()));
            }
            if exists(&eq, Reference::Zamo, r) {
                observers.push(("the ZAMO".into(), m.zamo()));
                for celerity in [1e-9, 1e-6, 1e-3, 0.3, 30.0] {
                    for chi in [0.0, 1.2, FRAC_PI_2, 2.8, 4.4] {
                        observers.push((
                            format!("the ZAMO boosted to celerity {celerity:e} at {chi}"),
                            m.boosted_zamo(celerity, chi),
                        ));
                    }
                }
            }
            for (who, u_dd) in observers {
                let u = dd::rounded(&u_dd);
                let where_ = format!("{who}, at r = {r} M around a = {a}");
                let e = event(r, u);
                let triad = film::triad(&kerr, &e)
                    .unwrap_or_else(|why| panic!("{where_}: no triad: {why}"));
                for p in passing(&kerr, &e, &triad) {
                    let u_ref = match p.reference {
                        Reference::Static => m.static_observer(),
                        Reference::Zamo => m.zamo(),
                        Reference::Raindrop => m.raindrop(),
                    };
                    let (truth, celerity) = m.travel(&u_dd, &u_ref);
                    // How far rounding u to f64 alone moves the celerity: a few roundings of the
                    // largest term of L or of u's frame components, which grow with u^t and the
                    // metric's size.
                    let input = 8.0 * f64::EPSILON * u[0].abs().max(1.0) * (1.0 + r * r);
                    let label = format!("past {}: {where_}", p.reference.name());
                    let Some(n) = p.direction else {
                        omitted += 1;
                        // Left out only at rest - gamma - 1 below rounding - or where f64 cannot
                        // hold the direction to 1e-4 rad: the motion is within 1e4 roundings of u
                        // of rest.
                        assert!(
                            celerity <= 1.000_001 * rest() || celerity <= 1e4 * input,
                            "{label}: no direction is given at a celerity of {celerity:e}, which \
                             the observer's u holds to {input:e}"
                        );
                        if celerity > fastest_omitted.0 {
                            fastest_omitted = (celerity, format!("{label} (u^t = {:.1e})", u[0]));
                        }
                        continue;
                    };
                    given += 1;
                    if celerity < slowest_given.0 {
                        slowest_given = (celerity, format!("{label} (u^t = {:.1e})", u[0]));
                    }
                    let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    assert!(
                        (length - 1.0).abs() < 4.0 * f64::EPSILON && n[2] == 0.0,
                        "{where_}: n = {n:?} is not a unit vector in the plane"
                    );
                    let miss = angle(n, truth);
                    assert!(
                        miss < HUNDREDTH_OF_A_DEGREE,
                        "{label}: the direction is {n:?} and the double-double evaluation's is \
                         {truth:?}, {} degrees apart",
                        miss.to_degrees()
                    );
                    // The celerity to its own precision, less what rounding u to f64 moved it.
                    assert!(
                        (p.celerity - celerity).abs() <= 1e-9 * celerity + 10.0 * input,
                        "{label}: celerity {} against {celerity}",
                        p.celerity
                    );
                    if miss > worst.0 {
                        worst = (miss, label.clone());
                    }
                    if p.reference == Reference::Raindrop && miss > worst_raindrop.0 {
                        worst_raindrop = (miss, label.clone());
                    }
                    // The literal construction in f64 on kerr_sky's legs, where f64 can do it: a
                    // modest boost, and a motion well clear of rest, since -w = gamma u - u_F is
                    // a difference of vectors of size u^t.
                    if p.gamma < 100.0 && u[0] < 100.0 && celerity > 1e-3 {
                        let direct = direct(&kerr, &triad, &e, p.reference);
                        worst_direct = worst_direct.max(angle(n, direct));
                    }
                }
            }
        }
    }
    println!(
        "{given} directions: worst {:.2e} rad ({:.2e} degrees), {}; past the raindrop worst \
         {:.2e} rad, {}; against f64 on kerr_sky's triad legs {worst_direct:.1e} rad",
        worst.0,
        worst.0.to_degrees(),
        worst.1,
        worst_raindrop.0,
        worst_raindrop.1
    );
    println!(
        "the slowest motion given a direction: celerity {:.1e}, {}",
        slowest_given.0, slowest_given.1
    );
    println!(
        "{omitted} passings without a direction; the fastest of them: celerity {:.1e}, {}",
        fastest_omitted.0, fastest_omitted.1
    );
    assert!(worst_direct < 1e-9, "{worst_direct}");
    assert!(given > 3500, "{given}");
}

/// The direction of travel past `reference` by the literal construction in f64: -w = gamma u -
/// u_F in the chart, carried to Cartesian components and projected on the triad's own x and y.
fn direct(kerr: &Kerr, triad: &kerr_sky::Triad, e: &Event, reference: Reference) -> [f64; 3] {
    let eq = kerr.equatorial();
    let g = eq.metric_components(e.r);
    let u_ref = match reference {
        Reference::Static => [1.0 / (-g[0][0]).sqrt(), 0.0, 0.0],
        Reference::Zamo => {
            let omega = -g[0][2] / g[2][2];
            let norm = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
            [1.0 / norm.sqrt(), 0.0, omega / norm.sqrt()]
        }
        Reference::Raindrop => raindrop_u(kerr, e.r),
    };
    let gamma = -kerr_equatorial::tetrad::inner(&eq, e.r, &e.u, &u_ref);
    let minus_w: [f64; 3] = std::array::from_fn(|mu| gamma * e.u[mu] - u_ref[mu]);
    let w = kerr.chart_vector(e.r, e.phi, minus_w);
    let (x, y) = (
        kerr.dot(triad.position, w, triad.x),
        kerr.dot(triad.position, w, triad.y),
    );
    let length = x.hypot(y);
    [x / length, y / length, 0.0]
}

#[test]
fn test_above_0_9999_c_the_lorentz_factor_is_given_instead_of_the_speed() {
    // The raindrop boosted to just below and just above 0.9999 c, between the horizons of
    // a = 0.9, where the raindrop is the one reference observer.
    let a = 0.9;
    let kerr = kerr(a);
    let r = 1.0;
    let m = dd::Metric::new(1.0, a, r);
    let frame_at = |v: f64| {
        let gamma = 1.0 / (1.0 - v * v).sqrt();
        passings(&kerr, r, dd::rounded(&m.boosted_raindrop(gamma, 2.0)))
    };
    let slow = frame_at(0.999_89);
    let fast = frame_at(0.999_91);
    assert!(!slow[0].fast() && fast[0].fast());
    let (slow_values, _) = values(&slow, &written_headings(std::slice::from_ref(&slow))[0]);
    let (fast_values, _) = values(&fast, &written_headings(std::slice::from_ref(&fast))[0]);
    let ids = |v: &[(String, f64)]| v.iter().map(|(id, _)| id.clone()).collect::<Vec<_>>();
    assert_eq!(ids(&slow_values), ["speed_raindrop", "heading_raindrop"]);
    assert_eq!(ids(&fast_values), ["gamma_raindrop", "heading_raindrop"]);
    assert!((fast_values[0].1 - 1.0 / (1.0 - 0.999_91f64.powi(2)).sqrt()).abs() < 1e-6);
    // Declared only when some frame needs it, after the speed and before the heading.
    let declared = |frames: &[Vec<Passing>]| {
        declarations(frames)
            .0
            .into_iter()
            .map(|r| (r.id, r.label, r.unit, r.decimals))
            .collect::<Vec<_>>()
    };
    let speed = (
        "speed_raindrop".to_string(),
        "Speed past the raindrop".to_string(),
        "c".to_string(),
        4,
    );
    let gamma = (
        "gamma_raindrop".to_string(),
        "Lorentz factor past the raindrop".to_string(),
        String::new(),
        1,
    );
    let heading = (
        "heading_raindrop".to_string(),
        "Heading past the raindrop (triangle)".to_string(),
        RIGHT_OF_THE_HOLE.to_string(),
        1,
    );
    assert_eq!(
        declared(&[slow.clone(), slow.clone()]),
        [speed.clone(), heading.clone()]
    );
    assert_eq!(declared(&[slow, fast]), [speed, gamma, heading]);
}

#[test]
fn test_a_still_gives_its_heading_as_a_magnitude_and_a_film_gives_it_signed() {
    let moving = |heading_degrees: f64| {
        let (s, c) = heading_degrees.to_radians().sin_cos();
        Passing {
            reference: Reference::Zamo,
            gamma: 1.25,
            celerity: 0.75,
            speed: 0.6,
            direction: Some([c, -s, 0.0]),
            rounding: 1e-16,
        }
    };
    let left = moving(-35.2);
    let right = moving(35.2);
    assert!((left.heading().unwrap() + 35.2).abs() < 1e-12);

    // A still: the magnitude, with the side in the unit.
    for (p, unit) in [(left, LEFT_OF_THE_HOLE), (right, RIGHT_OF_THE_HOLE)] {
        let (declared, marks) = declarations(&[vec![p]]);
        let heading = declared.iter().find(|r| r.id == "heading_zamo").unwrap();
        assert_eq!(heading.unit, unit);
        assert_eq!(heading.label, "Heading past the ZAMO (diamond)");
        assert_eq!(marks.len(), 1);
        assert_eq!(marks[0].id, "travel_zamo");
        assert_eq!(marks[0].label, "Direction of travel past the ZAMO");
        assert_eq!(marks[0].shape, "diamond");
        let (values, _) = values(&[p], &written_headings(&[vec![p]])[0]);
        let value = values
            .iter()
            .find(|(id, _)| id == "heading_zamo")
            .unwrap()
            .1;
        assert!((value - 35.2).abs() < 1e-12, "{value}");
    }

    // A film: one signed unit, negative to the left.
    let (declared, _) = declarations(&[vec![left], vec![right]]);
    let heading = declared.iter().find(|r| r.id == "heading_zamo").unwrap();
    assert_eq!(heading.unit, RIGHT_OF_THE_HOLE);
    let written = written_headings(&[vec![left], vec![right]]);
    let (values, _) = values(&[left], &written[0]);
    assert!((values[1].1 + 35.2).abs() < 1e-12);

    // The seam behind: (-180, 180], and no -0.
    assert_eq!(heading_of([-1.0, 0.0, 0.0]), 180.0);
    assert_eq!(heading_of([-1.0, -0.0, 0.0]), 180.0);
    assert_eq!(heading_of([-1.0, 1e-300, 0.0]), 180.0);
    assert!(heading_of([1.0, 0.0, 0.0]).is_sign_positive());
    assert_eq!(heading_of([0.0, -1.0, 0.0]), 90.0);
    assert_eq!(heading_of([0.0, 1.0, 0.0]), -90.0);
}

#[test]
fn test_bob_released_at_rest_is_at_rest_past_the_zamo_at_the_release() {
    // Bob in the demonstration save is released at rest with L = 0 at 2.27 M: at that moment he
    // is the ZAMO, and his u^r, the root of R at its zero, is a rounding's square root, 1e-8.
    // gamma - 1 is below rounding, so there is no direction and the speed is 0. One frame later
    // he is falling at the hole: heading 0 past the ZAMO.
    let save = crate::bhl::read_file(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../demos/near_fall.bhl"),
    )
    .unwrap();
    let metric = save.hole.metric();
    let kerr = Kerr::from_equatorial(&metric);
    let mut bob = Worldline::from_saved(&metric, save.bob.as_ref().unwrap()).unwrap();
    let film = film::walk(&mut bob, 0.25, 30.0, 2);
    let at = |k: usize| {
        let e = film.events[k];
        passing(&kerr, &e, &film::triad(&kerr, &e).unwrap())
    };
    let released = past(&at(0), Reference::Zamo);
    assert_eq!((released.speed, released.direction), (0.0, None));
    let falling = past(&at(1), Reference::Zamo);
    assert!(falling.speed > 1e-4, "{falling:?}");
    assert!(falling.heading().unwrap().abs() < 1e-3, "{falling:?}");
    // Past the static observer he is moving from the start: the dragging carries him round.
    let fixed = past(&at(0), Reference::Static);
    assert!(fixed.speed > 0.5 && (fixed.heading().unwrap() - 90.0).abs() < 1e-3);
}

#[test]
fn test_a_still_whose_heading_shows_as_zero_is_not_to_either_side() {
    let at = |n: [f64; 3]| Passing {
        reference: Reference::Raindrop,
        gamma: 2.0,
        celerity: 3f64.sqrt(),
        speed: 3f64.sqrt() / 2.0,
        direction: Some(n),
        rounding: 1e-16,
    };
    for n in [[1.0, 1e-9, 0.0], [1.0, -1e-9, 0.0], [-1.0, 1e-9, 0.0]] {
        let (declared, _) = declarations(&[vec![at(n)]]);
        assert_eq!(declared.last().unwrap().unit, RIGHT_OF_THE_HOLE, "{n:?}");
    }
    // -0.06 degrees shows as -0.1: to the left.
    let n = [0.06f64.to_radians().cos(), 0.06f64.to_radians().sin(), 0.0];
    let (declared, _) = declarations(&[vec![at(n)]]);
    assert_eq!(declared.last().unwrap().unit, LEFT_OF_THE_HOLE);
}

/// A passing past the ZAMO at the given heading, with the given rounding bound on its direction
/// in radians; `None` for no direction.
fn heading(degrees: Option<f64>, rounding_rad: f64) -> Passing {
    Passing {
        reference: Reference::Zamo,
        gamma: 2.0,
        celerity: 3f64.sqrt(),
        speed: 3f64.sqrt() / 2.0,
        direction: degrees.map(|h| {
            let (s, c) = h.to_radians().sin_cos();
            [c, -s, 0.0]
        }),
        rounding: rounding_rad * 3f64.sqrt(),
    }
}

/// The written headings of a film of single passings.
fn written(frames: &[Option<f64>]) -> Vec<Option<f64>> {
    let frames: Vec<Vec<Passing>> = frames.iter().map(|h| vec![heading(*h, 1e-16)]).collect();
    written_headings(&frames)
        .into_iter()
        .map(|line| line[0])
        .collect()
}

fn near(a: &[Option<f64>], b: &[Option<f64>]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| match (x, y) {
            (Some(x), Some(y)) => (x - y).abs() < 1e-9,
            (None, None) => true,
            _ => false,
        })
}

#[test]
fn test_a_films_heading_is_continuous_across_180() {
    // Across the seam behind: 179.9 then (-179.9) is written 180.1, and on round again, so that a
    // renderer interpolating between frames turns through 180 and not back through 0.
    let got = written(&[
        Some(170.0),
        Some(179.9),
        Some(-179.9),
        Some(-170.0),
        Some(-10.0),
        Some(100.0),
        Some(-100.0),
    ]);
    let want = [170.0, 179.9, 180.1, 190.0, 350.0, 460.0, 620.0].map(Some);
    assert!(near(&got, &want), "{got:?}");
    // And back the other way, below -180.
    let got = written(&[Some(-179.0), Some(179.0), Some(170.0)]);
    assert!(
        near(&got, &[Some(-179.0), Some(-181.0), Some(-190.0)]),
        "{got:?}"
    );
}

#[test]
fn test_after_a_gap_the_heading_starts_again_in_the_interval() {
    // A frame with no direction past the ZAMO (at rest, or the ZAMO not there) breaks the run.
    let got = written(&[Some(170.0), Some(-175.0), None, Some(-175.0), Some(175.0)]);
    let want = [Some(170.0), Some(185.0), None, Some(-175.0), Some(-185.0)];
    assert!(near(&got, &want), "{got:?}");
    // A frame at which the reference observer does not exist at all breaks it too.
    let frames = vec![
        vec![heading(Some(179.0), 1e-16)],
        vec![heading(Some(-179.0), 1e-16)],
        vec![],
        vec![heading(Some(-179.0), 1e-16)],
    ];
    let got: Vec<Option<f64>> = written_headings(&frames)
        .into_iter()
        .map(|line| line.first().copied().flatten())
        .collect();
    assert!(
        near(&got, &[Some(179.0), Some(181.0), None, Some(-179.0)]),
        "{got:?}"
    );
}

#[test]
fn test_a_heading_that_is_zero_to_its_rounding_is_written_as_plus_zero() {
    // -4e-7 degrees is 7e-9 rad: below a rounding bound of 1e-8 rad it is zero, and written +0.0,
    // in a film and in a still; above the bound it is kept as it is.
    for single in [true, false] {
        let frames: Vec<Vec<Passing>> = if single {
            vec![vec![heading(Some(-4e-7), 1e-8)]]
        } else {
            vec![
                vec![heading(Some(-4e-7), 1e-8)],
                vec![heading(Some(-4e-7), 1e-8)],
            ]
        };
        for line in written_headings(&frames) {
            let h = line[0].unwrap();
            assert!(h == 0.0 && h.is_sign_positive(), "{h}");
        }
    }
    let kept = written_headings(&[
        vec![heading(Some(-4e-7), 1e-12)],
        vec![heading(Some(-4e-7), 1e-12)],
    ]);
    assert!((kept[1][0].unwrap() + 4e-7).abs() < 1e-12);
}

#[test]
fn test_bob_past_the_raindrop_between_the_horizons_keeps_one_sign_at_180() {
    // Bob's fall in the demonstration save, 60 frames 0.066 M apart: from frame 46 he is between
    // the horizons, where only the raindrop is quoted, and he travels straight away from the hole
    // past it, since he fell from rest at 2.27 M and the raindrop from rest at infinity. His
    // heading there is 180 but for rounding, which sat either side of the seam from frame to
    // frame; written, it must be 180 to rounding and of one sign throughout.
    let save = crate::bhl::read_file(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../demos/near_fall.bhl"),
    )
    .unwrap();
    let metric = save.hole.metric();
    let kerr = Kerr::from_equatorial(&metric);
    let mut bob = Worldline::from_saved(&metric, save.bob.as_ref().unwrap()).unwrap();
    let film = film::walk(&mut bob, 1.98, 30.0, 60);
    assert_eq!(film.events.len(), 60);
    let taus: Vec<f64> = (0..60)
        .map(|k| film::frame_tau(bob.tau0(), k, 1.98, 30.0))
        .collect();
    let plan = film::plan(&kerr, "Bob", &film.events, &taus, 20.44, Units::Physical).unwrap();
    let raw: Vec<f64> = plan.passing[46..]
        .iter()
        .map(|p| past(p, Reference::Raindrop).heading().unwrap())
        .collect();
    assert!(
        raw.iter().any(|h| *h < 0.0) && raw.iter().any(|h| *h > 0.0),
        "the unwritten headings were expected to straddle the seam: {raw:?}"
    );
    let written: Vec<f64> = plan.headings[46..].iter().map(|l| l[0].unwrap()).collect();
    println!("Bob past the raindrop, frames 46 to 59: {written:?}");
    // Before frame 46 there was no raindrop heading, so the run starts there, and straight behind
    // is written at +180, never at -180.
    for (k, h) in written.iter().enumerate() {
        assert!((h - 180.0).abs() < 1e-6, "frame {}: {h}", 46 + k);
    }
}

#[test]
fn test_a_run_that_starts_straight_behind_is_written_at_plus_180() {
    // One decimal prints -179.96 as "-180.0" and -179.94 as "-179.9"; the cut is where the text
    // changes, at -179.95. A run's first heading below it is written 360 up, and the run unwraps
    // from there; at or above it, it stays as it is.
    assert_eq!(format!("{:.1}", -179.96f64), "-180.0");
    assert_eq!(format!("{:.1}", -179.94f64), "-179.9");
    assert_eq!(format!("{:.1}", -179.95f64), "-179.9");
    assert!((behind_cut() + 179.95).abs() < 1e-12);
    let got = written(&[Some(-179.96), Some(-179.8), Some(179.9)]);
    assert!(
        near(&got, &[Some(180.04), Some(180.2), Some(179.9)]),
        "{got:?}"
    );
    let got = written(&[Some(-179.94), Some(-179.8)]);
    assert!(near(&got, &[Some(-179.94), Some(-179.8)]), "{got:?}");
    let got = written(&[Some(-179.95), Some(-179.8)]);
    assert!(near(&got, &[Some(-179.95), Some(-179.8)]), "{got:?}");
    // After a gap the next run is cut the same way.
    let got = written(&[Some(170.0), None, Some(-179.99)]);
    assert!(near(&got, &[Some(170.0), None, Some(180.01)]), "{got:?}");
    // A still is a magnitude as before: 179.96 to the side the unit says.
    let still = written_headings(&[vec![heading(Some(-179.96), 1e-16)]]);
    assert!((still[0][0].unwrap() - 179.96).abs() < 1e-9);
}
