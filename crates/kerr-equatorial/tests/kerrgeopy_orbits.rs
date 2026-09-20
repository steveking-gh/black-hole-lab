//! The geodesic integrator against closed-form bound orbits it shares no code with.
//!
//! The unit tests beside `GeodesicState` hold the integrator to its own conserved quantities and to
//! the turning points of its own radial potential. Those say the curve is *a* geodesic of the
//! crate's metric between the right radii; they do not time it or measure its precession against
//! anything outside the crate.
//!
//! `scripts/kerrgeopy_bound_orbits.py` does. KerrGeoPy solves the bound orbit of each (a, p, e) in
//! Jacobi elliptic functions of Mino time, in Boyer-Lindquist coordinates, and the script writes
//! down what the orbit accumulates over one radial cycle: azimuth, coordinate time, proper time.
//! The two charts differ in t and phi by functions of r alone, which cancel between one periapsis
//! and the next, so this crate has to reproduce all three in its own chart.
//!
//! Both integrators land within 5e-10 M of KerrGeoPy in every quantity, over cycles of up to
//! 500 M; the tolerance below is that with room for another platform's rounding, and no more.

use kerr_equatorial::{GeodesicState, KerrSchild};
use std::f64::consts::{PI, TAU};

pub struct OracleOrbit {
    pub a: f64,
    pub energy: f64,
    pub l_ang: f64,
    pub r_min: f64,
    pub r_max: f64,
    pub d_phi: f64,
    pub d_t: f64,
    pub d_tau: f64,
}

include!("data/kerrgeopy_bound_orbits.rs");

/// A worldline together with the azimuth it has swept, which `GeodesicState` itself keeps only
/// modulo 2 pi.
#[derive(Clone, Copy)]
struct Tracked {
    geo: GeodesicState,
    swept: f64,
}

impl Tracked {
    fn advanced(self, step: &impl Fn(&mut GeodesicState, f64), h: f64) -> Self {
        let mut next = self;
        step(&mut next.geo, h);
        // No single step here turns the orbit through anything like half a revolution.
        let d = (next.geo.phi - self.geo.phi + PI).rem_euclid(TAU) - PI;
        next.swept += d;
        next
    }

    /// The state at the turning point inside the step of size h that starts here, given that u^r
    /// changes sign across it: bisection on the length of a single step from this state, so the
    /// turning point is located to the accuracy of one step of the integrator and not of a chord.
    fn at_turning_point(self, step: &impl Fn(&mut GeodesicState, f64), h: f64) -> Self {
        let before = self.geo.u[1];
        let (mut lo, mut hi) = (0.0, h);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.advanced(step, mid).geo.u[1] * before > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        self.advanced(step, 0.5 * (lo + hi))
    }
}

/// One radial cycle of an orbit released near apoapsis: (periapsis, the apoapsis after it, the
/// periapsis after that), each located as a turning point.
///
/// The cycle is timed between the two periapses and not from the release. KerrGeoPy's E is good
/// to sixteen digits, which leaves R(r_max) a rounding away from zero instead of on it, and u^r is
/// the square root of that: the release is a few microseconds of proper time to one side of the
/// true apoapsis, and a cycle measured from it would inherit the offset.
fn one_radial_cycle(
    metric: &KerrSchild,
    orbit: &OracleOrbit,
    step: impl Fn(&mut GeodesicState, f64),
    h: f64,
) -> [Tracked; 3] {
    let geo = GeodesicState::new_infall(metric, 0.0, orbit.r_max, orbit.energy, orbit.l_ang);
    // If R(r_max) comes out a rounding below zero the constructor lifts E to the floor. That must
    // be all it does.
    assert!((geo.energy - orbit.energy).abs() < 1e-12, "the release moved E to {}", geo.energy);
    let mut now = Tracked { geo, swept: 0.0 };

    let mut turns = Vec::new();
    for _ in 0..10_000_000 {
        let next = now.advanced(&step, h);
        assert!(!next.geo.stalled, "a bound orbit must never stall (r = {})", next.geo.r);
        // Periapses are where u^r comes up through zero, apoapses where it goes back down, and
        // they are wanted in turn, starting with a periapsis.
        let rising = turns.len() % 2 == 0;
        let turned = if rising {
            now.geo.u[1] < 0.0 && next.geo.u[1] >= 0.0
        } else {
            now.geo.u[1] > 0.0 && next.geo.u[1] <= 0.0
        };
        if turned {
            turns.push(now.at_turning_point(&step, h));
            if let [first, second, third] = turns[..] {
                return [first, second, third];
            }
        }
        now = next;
    }
    panic!("no radial cycle completed: r = {}", now.geo.r);
}

fn check(name: &str, orbit: &OracleOrbit, step: impl Fn(&mut GeodesicState, f64), h: f64, tol: f64) {
    let metric = KerrSchild::new(1.0, orbit.a);
    let [first, apoapsis, second] = one_radial_cycle(&metric, orbit, step, h);
    let errors = [
        ("r_min", first.geo.r, orbit.r_min),
        ("r_max", apoapsis.geo.r, orbit.r_max),
        ("r_min again", second.geo.r, orbit.r_min),
        ("d_phi", second.swept - first.swept, orbit.d_phi),
        ("d_t", second.geo.t - first.geo.t, orbit.d_t),
        ("d_tau", second.geo.tau - first.geo.tau, orbit.d_tau),
    ];
    println!(
        "{name}, a = {}, r in [{}, {}]: {}",
        orbit.a,
        orbit.r_min,
        orbit.r_max,
        errors.map(|(what, got, want)| format!("{what} off by {:.1e}", got - want)).join(", ")
    );
    for (what, got, want) in errors {
        assert!(
            (got - want).abs() < tol * want.abs(),
            "{name}: {what} = {got} against KerrGeoPy's {want} (a = {}, E = {}, L = {})",
            orbit.a,
            orbit.energy,
            orbit.l_ang
        );
    }
}

#[test]
fn test_the_proper_time_integrator_reproduces_kerrgeopy_over_a_radial_cycle() {
    for orbit in ORBITS {
        let metric = KerrSchild::new(1.0, orbit.a);
        check("proper time", orbit, |geo, h| geo.step(&metric, h), 0.01, 1e-9);
    }
}

#[test]
fn test_the_coordinate_time_integrator_reproduces_kerrgeopy_over_a_radial_cycle() {
    // The one the app runs every worldline on.
    for orbit in ORBITS {
        let metric = KerrSchild::new(1.0, orbit.a);
        check("coordinate time", orbit, |geo, h| geo.step_coord_time(&metric, h), 0.05, 1e-9);
    }
}
