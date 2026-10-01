//! The distance to r+ against quadrature and its closed forms, and the time since r+ against the
//! closed form of a radial fall and against a forward integration of the same worldline.

use kerr_equatorial::{GeodesicState, KerrSchild, geodesic::proper_time_between};

use super::*;
use crate::worldline::Motion;

/// The distance by Simpson's rule on the integrand r / sqrt(Delta), with r = r+ + s^2 to take out
/// the inverse square root at r+: dr = 2 s ds and Delta = s^2 (r - r-), so the integrand in s is
/// 2 r / sqrt(r - r-), smooth all the way down to s = 0.
fn distance_by_quadrature(metric: &KerrSchild, r: f64) -> f64 {
    let (r_plus, r_minus) = (metric.outer_horizon(), metric.inner_horizon());
    let top = (r - r_plus).sqrt();
    let f = |s: f64| {
        let r = r_plus + s * s;
        2.0 * r / (r - r_minus).sqrt()
    };
    const PANELS: usize = 20_000;
    let h = top / PANELS as f64;
    let mut sum = f(0.0) + f(top);
    for i in 1..PANELS {
        sum += if i % 2 == 1 { 4.0 } else { 2.0 } * f(h * i as f64);
    }
    sum * h / 3.0
}

#[test]
fn test_the_distance_from_r_plus_is_the_quadrature_of_r_over_the_root_of_delta() {
    for a in [0.0, 0.5, 0.9, 0.998] {
        let metric = KerrSchild::new(1.0, a);
        let r_plus = metric.outer_horizon();
        for r in [r_plus + 1e-6, r_plus + 0.01, 2.0, 2.320882864, 6.0, 40.0] {
            if r <= r_plus {
                continue;
            }
            let closed = proper_distance_from_outer_horizon(&metric, r).expect("outside r+");
            let quad = distance_by_quadrature(&metric, r);
            assert!(
                (closed - quad).abs() <= 1e-9 * (1.0 + quad),
                "a = {a}, r = {r}: {closed} against {quad}"
            );
        }
    }
}

#[test]
fn test_at_a_equal_zero_the_distance_is_schwarzschilds_and_it_vanishes_at_the_horizon() {
    let metric = KerrSchild::new(1.0, 0.0);
    for r in [2.0f64 + 1e-9, 2.5, 3.0, 6.0, 100.0] {
        let schwarzschild =
            (r * (r - 2.0)).sqrt() + 2.0 * ((r.sqrt() + (r - 2.0).sqrt()) / 2f64.sqrt()).ln();
        let d = proper_distance_from_outer_horizon(&metric, r).expect("outside r+");
        assert!((d - schwarzschild).abs() <= 1e-12 * (1.0 + d), "r = {r}");
    }
    // Toward r+ the distance goes to 0 as 2 r+ sqrt((r - r+) / (r+ - r-)), with no cancellation
    // left in it, and at r+ and inside there is no distance to give.
    for a in [0.0, 0.9] {
        let metric = KerrSchild::new(1.0, a);
        let (r_plus, r_minus) = (metric.outer_horizon(), metric.inner_horizon());
        let r = r_plus + 1e-12;
        // The gap as the two f64s have it, which is what the distance is of.
        let eps = r - r_plus;
        let d = proper_distance_from_outer_horizon(&metric, r).expect("outside r+");
        let leading = 2.0 * r_plus * (eps / (r_plus - r_minus)).sqrt();
        assert!(
            (d - leading).abs() <= 1e-6 * leading,
            "a = {a}: {d} against {leading}"
        );
        assert_eq!(proper_distance_from_outer_horizon(&metric, r_plus), None);
        assert_eq!(
            proper_distance_from_outer_horizon(&metric, 0.5 * r_plus),
            None
        );
    }
}

#[test]
fn test_an_extremal_holes_horizon_is_infinitely_far_from_everywhere_outside() {
    // `KerrSchild::new` keeps a below 0.9999 M, so the extremal hole is written out.
    let metric = KerrSchild {
        m: 1.0,
        m_solar: 10.0,
        a: 1.0,
    };
    assert_eq!(metric.outer_horizon(), 1.0);
    for r in [1.0 + 1e-9, 2.0, 10.0] {
        assert_eq!(
            proper_distance_from_outer_horizon(&metric, r),
            Some(f64::INFINITY)
        );
    }
    // And the nearest hole to extremal the app allows is a long, but finite, way down.
    let near = KerrSchild::new(1.0, 1.0);
    let d = proper_distance_from_outer_horizon(&near, 2.0).expect("outside r+");
    assert!(
        d.is_finite()
            && d > proper_distance_from_outer_horizon(&KerrSchild::new(1.0, 0.9), 2.0).unwrap()
    );
}

/// The event of a geodesic state, as the walker hands it over.
fn event_of(geo: &GeodesicState) -> Event {
    Event {
        tau: geo.tau,
        t: geo.t,
        r: geo.r,
        phi: geo.phi,
        u: geo.u,
        motion: Motion::FreeFall,
    }
}

/// `geo` carried forward in small windows of coordinate time until r falls below `r_stop`.
fn fall_to(metric: &KerrSchild, geo: &mut GeodesicState, r_stop: f64) {
    while geo.r > r_stop {
        geo.step_coord_time(metric, 1e-3);
        assert!(!geo.stalled, "the fall stalled at r = {}", geo.r);
    }
}

#[test]
fn test_the_time_since_r_plus_of_a_radial_fall_from_rest_is_the_cycloids() {
    // Schwarzschild, released from rest at r0: the proper time from r0 to r is
    // sqrt(r0^3 / 2M) (sqrt(x (1 - x)) + arccos(sqrt(x))) with x = r / r0, so from r+ = 2M to r it
    // is the difference of two of those.
    let metric = KerrSchild::new(1.0, 0.0);
    for r0 in [4.0, 6.0, 10.0] {
        let since_r0 = |r: f64| {
            let x = r / r0;
            (r0 * r0 * r0 / 2.0).sqrt() * ((x * (1.0 - x)).sqrt() + x.sqrt().acos())
        };
        let energy = (1.0 - 2.0 / r0).sqrt();
        let mut geo = GeodesicState::new_infall(&metric, 0.0, r0, energy, 0.0);
        for r_now in [1.9, 1.2, 0.5] {
            fall_to(&metric, &mut geo, r_now);
            let event = event_of(&geo);
            let crossing = crossing_tau(&metric, &event).expect("r+ is reached");
            let expected = since_r0(event.r) - since_r0(2.0);
            let got = event.tau - crossing;
            assert!(
                (got - expected).abs() <= 1e-8,
                "r0 = {r0}, r = {}: {got} against {expected}",
                event.r
            );
        }
    }
}

#[test]
fn test_the_time_since_r_plus_in_kerr_is_what_the_same_fall_took_going_forward() {
    // a = 0.9, a fall with angular momentum from outside: the forward integration is sampled
    // finely across r+, and the crossing read off by linear interpolation in r between the two
    // samples either side; the backward search from a later event inside must find the same
    // proper time, and so must the core's independent quadrature of r^2 / sqrt(R).
    let metric = KerrSchild::new(1.0, 0.9);
    let (r_plus, r_minus) = (metric.outer_horizon(), metric.inner_horizon());
    for (energy, l_ang) in [(1.0, 0.0), (0.95, 2.0), (1.1, -1.0)] {
        let mut geo = GeodesicState::new_infall(&metric, 0.0, 3.0, energy, l_ang);
        let mut before = geo;
        while geo.r > r_plus {
            before = geo;
            geo.step_coord_time(&metric, 1e-4);
        }
        let w = (before.r - r_plus) / (before.r - geo.r);
        let forward = before.tau + w * (geo.tau - before.tau);

        let r_now = 0.5 * (r_plus + r_minus) + 0.1;
        fall_to(&metric, &mut geo, r_now);
        let event = event_of(&geo);
        let backward = crossing_tau(&metric, &event).expect("r+ is reached");
        assert!(
            (backward - forward).abs() <= 1e-8,
            "E = {energy}, L = {l_ang}: {backward} against {forward}"
        );
        let quadrature =
            proper_time_between(&metric, energy, l_ang, r_plus, event.r).expect("no turning point");
        assert!(
            (event.tau - backward - quadrature).abs() <= 1e-6,
            "E = {energy}, L = {l_ang}: {} against the quadrature's {quadrature}",
            event.tau - backward
        );
    }
}

#[test]
fn test_an_event_on_or_outside_r_plus_has_no_time_since_crossing_it_or_zero() {
    let metric = KerrSchild::new(1.0, 0.9);
    let r_plus = metric.outer_horizon();
    let geo = GeodesicState::new_infall(&metric, 0.0, r_plus, 1.0, 0.0);
    let on = Event {
        tau: 7.0,
        ..event_of(&geo)
    };
    assert_eq!(crossing_tau(&metric, &on), Some(7.0));
    let outside = Event { r: 2.5, ..on };
    assert_eq!(crossing_tau(&metric, &outside), None);
}
