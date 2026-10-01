//! How far the observer is from the outer event horizon: a distance outside it, a time inside it.
//!
//! # Outside r+: the proper distance
//!
//! The distance is the length of the radial curve orthogonal to both Killing vectors, from r+ to
//! the observer's r on the equatorial plane. Its tangent is the gradient of r, whose squared length
//! g^rr = nabla r . nabla r is a scalar - the same in Boyer-Lindquist and in the ingoing
//! Kerr-Schild chart - and equals Delta / r^2 on the equator, with Delta = r^2 - 2Mr + a^2. So
//!
//!     d(r) = integral from r+ to r of dr' / sqrt(g^rr) = integral from r+ to r of r' dr' / sqrt(Delta(r')).
//!
//! It is the ruler distance both hovering observers measure: the static observer and the ZAMO share
//! the radial leg e_r = nabla r / |nabla r|, so a rod laid along it from the horizon has this
//! length for either. It is not what a falling observer's own ruler reads, which depends on the
//! observer's motion; the read-out is the distance the hole's geometry gives the place, where the
//! chart's r is only the place's coordinate.
//!
//! The integral is elementary. With u = r - M and k = sqrt(M^2 - a^2), Delta = u^2 - k^2 and
//!
//!     d = sqrt(u^2 - k^2) + M ln((u + sqrt(u^2 - k^2)) / k),
//!
//! which is 0 at r = r+ (u = k). Near r+ the difference u^2 - k^2 cancels catastrophically, so it
//! is written as the product it is, (u - k)(u + k) = (r - r+)(r - r-), in which r - r+ is exact to
//! the rounding of r and r+. At a = M, where the two horizons meet, k = 0 and the horizon is an
//! infinite proper distance down its throat from every point outside: the distance is then
//! infinite, and is stored as the infinity it is (the format writes it as `"Infinity"` and the
//! renderer shows it as the read-out crate shows an infinity), never as a guessed large number.
//!
//! # Between the horizons: the proper time since the crossing
//!
//! Inside r+ there is no distance to the horizon to give - r is a time there, and Delta < 0 makes
//! the integral above imaginary - but there is a time: how long the observer's watch has run since
//! the observer's worldline crossed r+. The app records no such crossing, so it is found here. The
//! geodesic through the saved event is integrated backward in proper time, by the core's own RK4
//! (`GeodesicState::step`, which takes a negative step along the same worldline), until r rises to
//! r+; the step that crosses is then taken again from the same state, shortened by bisection until
//! r lands on r+ to within [`R_LANDING`]. Between the horizons r is a time coordinate and falls
//! along every future-directed timelike worldline, so into the past it rises monotonically and
//! reaches r+ at a finite proper time: the bisection always has one crossing to find.
//!
//! The worldline integrated is the geodesic through the saved event, which is the observer's true
//! history whenever the observer fell freely through r+ - every observer the app lets into the
//! interior does, since no hold, static or ZAMO worldline exists inside r+. For an observer the
//! user dragged to a point inside, it is the history of the geodesic that the tracer follows
//! from there anyway, which is the only history the film has.

use kerr_equatorial::{GeodesicState, KerrSchild};

use crate::worldline::Event;

/// How close to r+ the backward integration lands, in M: a hundred ulps of r at a few M, well
/// below the integration's own error, which is what bounds the answer.
pub const R_LANDING: f64 = 1e-13;

/// The proper-time step of the backward integration, in M. RK4's error per unit of proper time
/// goes as the fourth power of the step, about 1e-10 M here over the few M between the horizons,
/// far below the three decimals the read-out is written to;
/// `test_the_time_since_r_plus_of_a_radial_fall_from_rest_is_the_cycloids` measures it.
pub const BACK_STEP: f64 = 0.005;

/// How much proper time the backward integration spends before it gives up: no observer between
/// the horizons of a hole of spin below M is further than a few M of its own time from r+, so a
/// worldline that has not reached r+ by then is not one this can answer for.
const BACK_LIMIT: f64 = 1000.0;

/// The proper distance from the outer horizon to radius `r` on the equatorial plane, in M, along
/// the radial curve orthogonal to both Killing vectors (see the module's notes); `None` at or
/// inside r+, where there is none, and infinity for an extremal hole.
pub fn proper_distance_from_outer_horizon(metric: &KerrSchild, r: f64) -> Option<f64> {
    let (m, a) = (metric.m, metric.a);
    let r_plus = metric.outer_horizon();
    if r.is_nan() || r <= r_plus {
        return None;
    }
    let k = (m * m - a * a).max(0.0).sqrt();
    if k == 0.0 {
        return Some(f64::INFINITY);
    }
    let r_minus = metric.inner_horizon();
    let s = ((r - r_plus) * (r - r_minus)).sqrt();
    let u = r - m;
    Some(s + m * ((u + s) / k).ln())
}

/// The observer's proper time at the event where the geodesic through `event` crosses r+, going
/// into the past from `event`, which must lie between the horizons; `None` if the integration does
/// not reach r+ (see the module's notes). An event on r+ is its own crossing.
pub fn crossing_tau(metric: &KerrSchild, event: &Event) -> Option<f64> {
    let r_plus = metric.outer_horizon();
    if event.r > r_plus {
        return None;
    }
    // E = -u_t and L = u_phi, lowered with the chart's metric. The integrator carries them only as
    // diagnostics - it advances the 4-velocity itself - but a state is not complete without them.
    let g = metric.metric_components(event.r);
    let lower = |row: [f64; 3]| row[0] * event.u[0] + row[1] * event.u[1] + row[2] * event.u[2];
    let (energy, l_ang) = (-lower(g[0]), lower(g[2]));
    let mut geo = GeodesicState {
        t: event.t,
        r: event.r,
        phi: event.phi,
        tau: event.tau,
        energy,
        l_ang,
        u: event.u,
        stalled: false,
    };
    if geo.r >= r_plus - R_LANDING {
        return Some(geo.tau);
    }
    while event.tau - geo.tau < BACK_LIMIT {
        let before = geo;
        geo.step(metric, -BACK_STEP);
        if geo.stalled || !geo.r.is_finite() {
            return None;
        }
        if geo.r >= r_plus {
            return Some(land_on(metric, before, r_plus));
        }
    }
    None
}

/// The proper time at which the backward step from `before` (inside r+) reaches r+, found by
/// bisecting the step's length and taking each trial step afresh from `before`, so that the
/// landing is a point of the integrated curve and not an interpolation between two of its points.
fn land_on(metric: &KerrSchild, before: GeodesicState, r_plus: f64) -> f64 {
    let (mut inside, mut outside) = (0.0, BACK_STEP);
    let mut tau = before.tau - outside;
    for _ in 0..100 {
        let h = 0.5 * (inside + outside);
        let mut trial = before;
        trial.step(metric, -h);
        tau = trial.tau;
        if (trial.r - r_plus).abs() <= R_LANDING {
            break;
        }
        if trial.r < r_plus {
            inside = h;
        } else {
            outside = h;
        }
        if outside - inside <= f64::EPSILON * BACK_STEP {
            break;
        }
    }
    tau
}

#[cfg(test)]
mod tests;
