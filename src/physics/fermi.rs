//! Fermi placement: where a surface r = const actually stands in an observer's own space.
//!
//! An observer's rest space at one event is the 3-plane orthogonal to their 4-velocity, and the
//! honest way to put a coordinate on it is the Fermi normal one: fire a spacelike geodesic in each
//! direction of that plane, parametrised by its own proper length s, and the point reached at
//! length s in direction beta *is* the chart point (s cos beta, s sin beta). That is a ruler laid
//! out along the ground, and it is what "the horizon is 1.7 M away in that direction" means.
//!
//! The linearised chart the volume view draws everything else through is the first-order
//! approximation to this map, and it is exact only for displacements small against r. At the
//! blueshifts a late fall reaches - u^t of order 150 at a blueshift of 300 - the chart is good to
//! about r/u^t, some 4e-3 M, while the canvas spans ten times that, so the sections of the
//! surfaces it drew landed in the wrong place entirely. The shots below place them where they are.
//!
//! One geodesic equation serves both kinds of curve. `geodesic::geodesic_accel` computes
//! -Gamma^mu_{alpha beta} k^alpha k^beta from the connection alone, which knows nothing about the
//! norm of k, so the same right-hand side that carries a worldline carries a Fermi shot; what
//! separates them is the initial tangent, unit timelike there and unit spacelike here.
//!
//! # Which crossings of a horizon this chart contains
//!
//! A ruler aimed into the hole meets a horizon twice, going in and coming out, and the section
//! drawn from it wants both faces. The ingoing Kerr-Schild chart does not always have them, and
//! the condition is exact rather than a matter of resolution.
//!
//! Take the equatorial crossing at a horizon r_h (Delta = 0, where r_h^2 + a^2 = 2 M r_h). Every
//! geodesic carries E = -k_t and L = k_phi, and the spacelike radial equation is
//!
//!     r^4 (dr/ds)^2 = R_s(r) = P^2 + Delta [r^2 - (L - aE)^2],   P = E (r^2 + a^2) - a L,
//!
//! so R_s -> P^2 at the surface and dr/ds -> +/- |P| / r_h^2. The chart change from Boyer-Lindquist
//! is dt_KS = dt_BL + (2 M r / Delta) dr, which gives
//!
//!     r^2 k^t_KS = [ (r^2 + a^2) P + 2 M r (r^2 dr/ds) ] / Delta + a (L - aE)
//!               -> 2 M r_h ( P +/- |P| ) / Delta + a (L - aE),
//!
//! and the 1/Delta pole cancels for exactly one of the two signs: the one with
//!
//!     sign(dr/ds) = -sign(P) = -sign(E - Omega_H L),      Omega_H = a / (2 M r_h).
//!
//! The other crossing is at t_KS -> -+ infinity. This is the same statement, read for a spacelike
//! tangent, that `geodesic::GeodesicState::new_with_direction` makes for a timelike one: an
//! ingoing chart cannot follow an outgoing crossing. For a worldline E - Omega_H L > 0 and the
//! ingoing crossing is the one that is covered; for a ruler the sign is whatever the direction
//! beta makes it, and so it is one face or the other - never both.
//!
//! That sounds worse for the picture than it is, and the measurement says why. From r = 3 at
//! a = 0.90, sixteen of seventy-two directions reach r+ and four of them reach it twice. Of the
//! other twelve, ten die on the ring - on the equator R_s has a simple root at r = 0, so
//! dr/ds ~ r^{-3/2} there and the ruler arrives at finite length and ends - and two freeze on the
//! way *in* to r-, which is the Cauchy horizon's own chart limit. Every ruler that comes back out
//! at all reports its far face, at the surface, through the stall rule in `shoot`. Nothing is lost
//! to arithmetic: an error-controlled step inside the last 0.05 M of either horizon, halving until
//! two half steps agree with one whole one to 1e-9 in r, returns exactly the same sixteen and four
//! and costs thirteen times as much (20.9 ms a row against 1.56 ms).

use crate::physics::geodesic::{R_FLOOR, R_STOP, geodesic_accel, velocity_step_cap};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::tetrad::Tetrad;

/// The state a shot is integrated on: (t, r, phi, k^t, k^r, k^phi), the event and the tangent.
type ShotState = [f64; 6];

/// How many crossings of one radius a single ruler will report.
///
/// One is not enough, and the reason is the shape of the answer rather than an edge case. A
/// surface the observer stands *outside* of is a closed curve that does not enclose them, so a
/// ruler aimed at it goes in through the near face and out through the far one: report the first
/// crossing alone and the section comes back as the near arc, which is half a picture. Four is
/// enough for everything this chart draws - a ray can cross one radius twice going in and out
/// again, and a turning point inside can buy it two more - and it bounds the report.
pub const CROSSINGS_MAX: usize = 4;

/// What one ruler found: every crossing of every target, and where the ruler itself ended.
///
/// The end matters as much as the crossings do. The crossings cut the ray into intervals, each of
/// which lies wholly in one region of the hole, and the last of those intervals runs from the last
/// crossing to wherever the ray stopped - so without `reach` and `r_end` there would be no way to
/// say how far the outermost region reaches along that direction, or which region it is.
pub struct Shot {
    /// Per target, in the order the targets were given: every crossing as (s, event), in order of
    /// s, at most `CROSSINGS_MAX` of them.
    pub crossings: Vec<Vec<(f64, [f64; 3])>>,
    /// The proper length the ruler actually ran before it stopped, which is `s_max` unless
    /// something else stopped it first.
    pub reach: f64,
    /// The radius it stopped at. Below `R_STOP` the ray died on the ring; anything else and it ran
    /// out of ruler, escaped, or met a surface the chart cannot follow it across.
    pub r_end: f64,
}

/// Radius past which a shot is abandoned outright, whatever it is doing.
///
/// The escape test below is the one that stops most outward shots, at the first substep; this is
/// the backstop for a direction it cannot rule on, and it is deliberately far outside the 30 M or
/// so the app is ever looked at so that it cannot cut short a shot that still had somewhere to go.
const R_ESCAPE: f64 = 200.0;

/// Whether a shot at radius r with tangent k can be certain never to turn around and come back in.
///
/// Every geodesic of this chart carries E = -k_t and L = k_phi, and on the equator the radial
/// equation for a *spacelike* one of unit norm is
///
///     r^4 (dr/ds)^2 = R_s(r) = P^2 + Delta [r^2 - (L - aE)^2],    P = E (r^2 + a^2) - a L,
///
/// which is the timelike form with the sign of the r^2 term flipped by g(k, k) = +1 rather than
/// -1. Beyond r+ the factor Delta is positive, and beyond |L - aE| the bracket is positive too, so
/// past the larger of those two radii R_s is bounded below by P^2 and cannot vanish: an outgoing
/// shot there has no turning point ahead of it at any larger radius and is gone for good.
///
/// It is worth a test of its own per substep because it is what makes the whole row affordable. Of
/// the 72 directions fired from an observer outside the hole, some fifty point away from it, and
/// without this they would each run the full length of the ruler through empty space to find the
/// nothing that is out there; with it they stop on their first step.
fn escaping(metric: &KerrSchild, r: f64, k: &[f64; 3]) -> bool {
    if k[1] <= 0.0 {
        return false;
    }
    let g = metric.metric_components(r);
    let low = |mu: usize| -> f64 { (0..3).map(|nu| g[mu][nu] * k[nu]).sum::<f64>() };
    let (e, l) = (-low(0), low(2));
    r > metric.outer_horizon() && r > (l - metric.a * e).abs()
}

/// Hard ceiling on the substeps one shot may take. A direction the caps cannot get through - the
/// last hundredth of an M above the ring, say, or a frame boosted to u^t ~ 1e10, where the
/// connection stiffens by orders of magnitude - then costs one bounded shot rather than hanging
/// the frame, and reports whatever it had found by the time it ran out. Measured, the longest shot
/// a row from r = 3 fires at a ruler spanning the whole canvas takes about 1700 substeps, so this
/// leaves better than a factor of two before it binds on anything the picture needs.
const MAX_STEPS: usize = 4_000;

/// How far the tangent may grow in one substep before the shot is abandoned where it stands.
///
/// The step floor above deliberately overrides the stiffness cap in the last tenth of an M above
/// the ring and on an outward approach to r-, and where it does the integration can stop being an
/// integration: measured, one 2e-4 substep at r = 0.564 took |k| from 95 to 1e34 and put the
/// shot's radius at 7e12 M, which would have been recorded as a crossing of everything in between.
/// A tangent that has grown a hundredfold in a single substep is not a curve any more, so the step
/// is thrown away and the ruler ends there, reporting the crossings it had made while it was still
/// measuring something. This is the spacelike counterpart of `geodesic::U_T_STALL`, and it has the
/// same content: the ingoing chart has run out of the resolution to follow this curve.
const K_GROWTH_MAX: f64 = 1e6;

/// Ceiling on the tangent's own size, the spacelike twin of `geodesic::U_T_STALL` and there for
/// the same reason: past it the ingoing chart can no longer resolve where the curve is, so there
/// is nothing further to measure.
const K_STALL: f64 = 1e10;

/// How near a target a stalled shot has to be standing for its stall to count as an arrival, as a
/// fraction of the target's own radius. The stalls this is for land within 1e-10 of the surface;
/// 1e-6 is loose enough to be insensitive to where exactly `K_STALL` is put and far tighter than
/// anything a picture could tell apart.
const STALL_ARRIVAL: f64 = 1e-6;

/// Largest substep in proper length, in M. The same 0.05 the worldline integrator uses: outside a
/// few tenths of M nothing in the geometry changes fast enough for it to bind.
const S_STEP_MAX: f64 = 0.05;

/// Substep as a fraction of the current radius, the cap that takes over in the strong field.
const S_STEP_RADIUS_FRACTION: f64 = 0.01;

/// How much looser than the worldline integrator's the tangent-growth cap is for a shot.
///
/// `velocity_step_cap` is tuned to carry a worldline for a hundred M without drift; a ruler is
/// run once, for a few M at most, and read at its crossings. Measured at r = 4 over 5 M of s the
/// norm and the constants of the motion are unchanged to their 1e-7 by loosening it fourfold -
/// the cap does not bind there - and where it does bind, in the last hundredth of an M above the
/// ring and on an outward approach to a horizon, the ruler is placing the ring's section to a
/// couple of pixels or standing on a surface it has already reported. Two rulers of 132 got a
/// little further; nothing else moved. A row of 72 shots from r = 3 went from 5.8 ms to 2.0 ms.
const SHOT_STEP_CAP_LOOSENING: f64 = 4.0;

/// Fewest substeps a shot is cut into, whatever `s_max` is.
///
/// The other caps are ceilings on the step and say nothing when the whole ruler is shorter than
/// one of them: at the 2e11 px/M the framing reaches on the far branch of r- the canvas is three
/// nanometres of the observer's own chart across, and a shot of that length would otherwise be a
/// single RK4 step with the crossing read off its two ends. This is what keeps the ruler's
/// resolution proportional to its own length rather than absolute, so the picture is as good at
/// 1e11 px/M as it is at 48.
const MIN_STEPS: usize = 32;

/// Floor on the substep, as a fraction of `s_max`.
///
/// It is the velocity cap this overrides, and it overrides it only where that cap has collapsed
/// towards zero - the last hundredth of an M above the ring, where the connection goes like 1/r^3,
/// and an outward approach to a horizon, where the ingoing chart's own components run away.
/// Everywhere else one of the fixed caps is the smaller and the floor never binds: at a ruler of
/// 15 M it sits at 1.6e-6, a hundredth of the radius cap's own floor.
///
/// What it costs is the last of the accuracy in those two places, where the thing being placed is
/// the ring's section at a couple of pixels across, or a surface the shot is already standing on.
/// What it buys is that the fifth of the directions that plunge do not spend their whole step
/// budget grinding down the last hundredth of an M. Measured, 1e-7 is where the set of surfaces
/// the 72 directions of a row actually reach stops changing: a row from r = 3 finds the same 57
/// crossings it finds with no floor at all and costs a fifth less, and by 1e-5 it has begun to
/// lose the ring.
const S_STEP_FLOOR_FRACTION: f64 = 1e-7;

/// The direction cos(beta) e1 + sin(beta) e2 of an observer's own space, in coordinate components.
///
/// The two spacelike legs of an orthonormal tetrad span the observer's rest space (together with
/// the third, which the equatorial plane does not have), and they are orthonormal, so every such
/// combination is a unit spacelike vector: g(k, k) = cos^2 + sin^2 = +1 exactly, with no
/// normalisation to do. beta = 0 is the observer's own outward radial direction and beta = pi/2
/// their local +phi.
pub fn spatial_direction(tetrad: &Tetrad, beta: f64) -> [f64; 3] {
    let (s, c) = beta.sin_cos();
    [
        c * tetrad.e1[0] + s * tetrad.e2[0],
        c * tetrad.e1[1] + s * tetrad.e2[1],
        c * tetrad.e1[2] + s * tetrad.e2[2],
    ]
}

/// Right-hand side of the geodesic equation in proper length:
///     dx^mu/ds = k^mu,   dk^mu/ds = -Gamma^mu_{alpha beta} k^alpha k^beta.
fn rhs(metric: &KerrSchild, y: &ShotState) -> ShotState {
    let r = y[1].max(R_FLOOR);
    let k = [y[3], y[4], y[5]];
    let a = geodesic_accel(metric, r, &k);
    [k[0], k[1], k[2], a[0], a[1], a[2]]
}

/// Cubic Hermite interpolation across one substep: the value at fraction `u` of a coordinate whose
/// endpoint values are p0, p1 and whose endpoint derivatives, already multiplied by the step h,
/// are m0, m1.
///
/// The tangent *is* the derivative of the position, so both ends of every substep carry dx^mu/ds
/// exactly, and using them costs nothing and takes the crossing from second-order accurate to
/// fourth. That matters: with a chord alone the interpolation error, not the integration, is what
/// sets how well the ruler reads - at h = 0.05 M it misses the Schwarzschild proper distance to
/// r = 5 by 9e-6 M, where the integrated curve itself is good to a hundredth of that.
fn hermite(p0: f64, m0: f64, p1: f64, m1: f64, u: f64) -> f64 {
    let u2 = u * u;
    let u3 = u2 * u;
    (2.0 * u3 - 3.0 * u2 + 1.0) * p0
        + (u3 - 2.0 * u2 + u) * m0
        + (-2.0 * u3 + 3.0 * u2) * p1
        + (u3 - u2) * m1
}

/// Where along one substep the radius passes `r_h`, as a fraction in [0, 1] of the step.
///
/// The two ends bracket the crossing by construction - the caller has just seen r - r_h change
/// sign - so bisection on the Hermite cubic cannot fail and needs no derivative of its own. Fifty
/// halvings take the bracket below the ulp of the fraction, and the search runs at most once per
/// target per shot.
fn crossing_fraction(r0: f64, m0: f64, r1: f64, m1: f64, r_h: f64) -> f64 {
    let outward = r1 > r0;
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        let above = hermite(r0, m0, r1, m1, mid) > r_h;
        if above == outward { hi = mid } else { lo = mid }
    }
    0.5 * (lo + hi)
}

/// One classical RK4 step of `rhs` with step size h, reusing the slope `k1` at `y`.
fn rk4(metric: &KerrSchild, y: &ShotState, k1: &ShotState, h: f64) -> ShotState {
    let advance = |y: &ShotState, k: &ShotState, s: f64| -> ShotState {
        core::array::from_fn(|i| y[i] + s * k[i])
    };
    let k2 = rhs(metric, &advance(y, k1, 0.5 * h));
    let k3 = rhs(metric, &advance(y, &k2, 0.5 * h));
    let k4 = rhs(metric, &advance(y, &k3, h));
    core::array::from_fn(|i| y[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
}

/// One spacelike geodesic fired from `event` = (t, r, phi) with coordinate tangent `k` of unit
/// proper length (g(k, k) = +1), integrated in proper length s up to `s_max`, reporting every
/// crossing of every radius in `targets` as (s, event), in order of s.
///
/// The crossing is interpolated across the bracketing substep by the cubic Hermite through both
/// ends, position and tangent - see `hermite`, which is what takes the reading from second-order
/// accurate to fourth and is the difference between matching the Schwarzschild proper distance to
/// nine digits and to five.
///
/// *Every* crossing, and found by sign change rather than by comparison, because r is not monotone
/// along a spacelike geodesic and assuming it were would be wrong in both directions. A ruler
/// aimed at the hole from outside goes in through the near face of a surface and out through the
/// far one, and both faces are on the same sheet of the observer's own space: keeping only the
/// first would draw half of every section. Inside r+ the surfaces t = const of this chart are
/// themselves spacelike and run out through the horizon, so a shot fired from region II reaches r+
/// perfectly well going *outward*. And a ruler with a turning point in it crosses the same radius
/// twice in a row without ever having been on the other side of it.
///
/// The shot stops at `s_max`, at the ring (r < `R_STOP`), far outside (r > `R_ESCAPE`), the
/// moment it can prove it will never come back (see `escaping`), after `MAX_STEPS` substeps, or
/// when the tangent stops being something this chart can carry; whatever it had found by then is
/// what it reports, plus any target it is standing on when it gives up.
pub fn shoot(
    metric: &KerrSchild,
    event: [f64; 3],
    k: [f64; 3],
    s_max: f64,
    targets: &[f64],
) -> Shot {
    let mut found: Vec<Vec<(f64, [f64; 3])>> = vec![Vec::new(); targets.len()];
    // The negation is the point rather than a way of writing <=: an s_max that has come out NaN
    // has to take this branch too, and `s_max <= 0.0` would let it through.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(s_max > 0.0) || !event.iter().chain(k.iter()).all(|v| v.is_finite()) {
        return Shot { crossings: found, reach: 0.0, r_end: event[1] };
    }
    let mut y: ShotState = [event[0], event[1], event[2], k[0], k[1], k[2]];
    let mut s = 0.0f64;
    let floor = S_STEP_FLOOR_FRACTION * s_max;

    for _ in 0..MAX_STEPS {
        let remaining = s_max - s;
        if remaining <= floor {
            break;
        }
        let k1 = rhs(metric, &y);
        let h = remaining
            .min(S_STEP_MAX)
            .min((S_STEP_RADIUS_FRACTION * y[1]).max(1e-4))
            .min(
                SHOT_STEP_CAP_LOOSENING
                    * velocity_step_cap(&[k1[3], k1[4], k1[5]], &[y[3], y[4], y[5]]),
            )
            .min(s_max / (MIN_STEPS as f64))
            .max(floor)
            .min(remaining);
        let next = rk4(metric, &y, &k1, h);
        let scale = |q: &ShotState| q[3].abs().max(q[4].abs()).max(q[5].abs()).max(1.0);
        if !next.iter().all(|v| v.is_finite())
            || scale(&next) > K_STALL
            || scale(&next) > K_GROWTH_MAX * scale(&y)
        {
            // The chart has run out: a shot that leaves a horizon *outward* has its ingoing
            // Kerr-Schild components diverge on the surface, exactly as an outgoing worldline's
            // u^t does, and that is where every one of these stalls happens - measured, |k| climbs
            // through 1e9 while r sits within 1.6e-10 M of r+ or r-. The curve is at the surface;
            // it is the chart that cannot follow it across. So a target the shot is standing on
            // when it gives up is reported as reached, at the length it had run to, and one it is
            // nowhere near is not.
            for (slot, &r_h) in found.iter_mut().zip(targets.iter()) {
                let fresh = slot.last().is_none_or(|(prev, _)| s > *prev + 1e-12);
                if slot.len() < CROSSINGS_MAX
                    && fresh
                    && (y[1] - r_h).abs() < STALL_ARRIVAL * r_h.max(1.0)
                {
                    slot.push((s, [y[0], y[1], y[2]]));
                }
            }
            break;
        }
        // The crossings of this substep, taken before the stop tests: a step that has just put the
        // shot below `R_STOP` is the step that crossed the ring, and its crossing counts.
        for (slot, &r_h) in found.iter_mut().zip(targets.iter()) {
            if slot.len() >= CROSSINGS_MAX {
                continue;
            }
            let (a, b) = (y[1] - r_h, next[1] - r_h);
            if a == 0.0 || (a > 0.0) == (b > 0.0) {
                continue;
            }
            let f = crossing_fraction(y[1], h * y[4], next[1], h * next[4], r_h);
            slot.push((
                s + f * h,
                core::array::from_fn(|i| {
                    hermite(y[i], h * y[3 + i], next[i], h * next[3 + i], f)
                }),
            ));
        }
        y = next;
        s += h;
        if y[1] < R_STOP || y[1] > R_ESCAPE {
            break;
        }
        // Nothing left that this shot could still meet. Either every target is full, or every one
        // of them is behind it and it is on its way out for good - a target below an escaping ray
        // can never be crossed again, whether or not it has been crossed already. See `escaping`.
        if found.iter().all(|slot| slot.len() >= CROSSINGS_MAX) {
            break;
        }
        let behind = targets.iter().all(|r_h| *r_h < y[1]);
        if behind && escaping(metric, y[1], &[y[3], y[4], y[5]]) {
            break;
        }
    }
    Shot { crossings: found, reach: s, r_end: y[1] }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::geodesic::GeodesicState;

    /// (E, L) = (-k_t, k_phi) of a tangent at radius r.
    fn invariants_of(metric: &KerrSchild, r: f64, k: &[f64; 3]) -> (f64, f64) {
        let (_, k_t, l) = invariants(metric, r, k);
        (-k_t, l)
    }

    /// (g(k,k), k_t, k_phi) of a tangent at radius r: the norm and the two constants of the motion
    /// the Killing vectors d_t and d_phi give every geodesic, spacelike ones included.
    fn invariants(metric: &KerrSchild, r: f64, k: &[f64; 3]) -> (f64, f64, f64) {
        let g = metric.metric_components(r);
        let low = |mu: usize| -> f64 { (0..3).map(|nu| g[mu][nu] * k[nu]).sum() };
        let norm: f64 = (0..3).map(|mu| low(mu) * k[mu]).sum();
        (norm, low(0), low(2))
    }

    /// The 4-velocity of the ingoing (E, L) geodesic at radius r, which is what an observer of
    /// this app is riding whenever they have been released.
    fn falling(metric: &KerrSchild, r: f64, energy: f64, l_ang: f64) -> [f64; 3] {
        GeodesicState::new_infall(metric, 0.0, r, energy, l_ang).u
    }

    /// The static observer's 4-velocity u^mu = (1, 0, 0) / sqrt(-g_tt), which exists only outside
    /// the static limit.
    fn stationary(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g = metric.metric_components(r);
        [1.0 / (-g[0][0]).sqrt(), 0.0, 0.0]
    }

    #[test]
    fn test_a_fermi_shot_keeps_its_norm_and_its_constants() {
        // The one check that the shot is a geodesic of *this* spacetime and stays spacelike while
        // it is one. g(k, k) = +1 is what makes s a proper length - a ruler that stretched would
        // measure the wrong distance to the horizon - and k_t = -E, k_phi = L are conserved by the
        // Killing vectors d_t and d_phi, which the integrator is told nothing about: it integrates
        // the connection and they come out.
        //
        // Walked step by step rather than through `shoot`, because what is being checked is every
        // point of the curve and not where it ended up.
        let metric = KerrSchild::new(1.0, 0.9);
        let r0 = 4.0;
        let u = falling(&metric, r0, 1.0, 2.2);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);
        let s_max = 5.0;
        let mut worst = (0.0f64, 0.0f64, 0.0f64);
        for i in 0..8 {
            let beta = std::f64::consts::TAU * (i as f64) / 8.0;
            let k = spatial_direction(&tetrad, beta);
            let mut y: ShotState = [0.0, r0, 0.0, k[0], k[1], k[2]];
            let (n0, e0, l0) = invariants(&metric, r0, &k);
            assert!(
                (n0 - 1.0).abs() < 1e-12,
                "the seed direction is unit spacelike by orthonormality: g(k,k) = {n0} at \
                 beta = {beta}"
            );
            let mut s = 0.0f64;
            while s < s_max && y[1] > R_STOP && y[1] < R_ESCAPE {
                let k1 = rhs(&metric, &y);
                let h = (s_max - s)
                    .min(S_STEP_MAX)
                    .min((S_STEP_RADIUS_FRACTION * y[1]).max(1e-4))
                    .min(velocity_step_cap(&[k1[3], k1[4], k1[5]], &[y[3], y[4], y[5]]))
                    .min(s_max / (MIN_STEPS as f64));
                y = rk4(&metric, &y, &k1, h);
                s += h;
                let (n, e, l) = invariants(&metric, y[1], &[y[3], y[4], y[5]]);
                worst = (
                    worst.0.max((n - 1.0).abs()),
                    worst.1.max((e - e0).abs()),
                    worst.2.max((l - l0).abs()),
                );
            }
            // 1e-7 and not the 1e-8 a worldline is held to, and the difference is one decision
            // rather than a slackening: this is the RK4 truncation at h = min(0.05, 0.01 r), and
            // buying the next decade means halving the step and doubling the cost of every frame.
            // What 1e-7 in g(k, k) is worth on the canvas: it is a relative error in the ruler, so
            // 2.5e-7 M over the five M walked here, which at the 48 px/M the view opens on is a
            // hundred-thousandth of a pixel and at the 1e11 px/M a late fall reaches is nothing at
            // all, because there the whole ruler is 1e-8 M long and the step shrinks with it.
            assert!(
                worst.0 < 1e-7 && worst.1 < 1e-7 && worst.2 < 1e-7,
                "after {s} M of proper length at beta = {beta}: |g(k,k) - 1| = {}, dE = {}, \
                 dL = {}",
                worst.0,
                worst.1,
                worst.2
            );
        }
        println!(
            "8 shots of 5 M from r = 4 (a = 0.9): |g(k,k) - 1| <= {:.2e}, |dk_t| <= {:.2e}, \
             |dk_phi| <= {:.2e}",
            worst.0, worst.1, worst.2
        );
    }

    #[test]
    fn test_a_radial_shot_from_a_static_observer_measures_the_schwarzschild_proper_distance() {
        // The closed form the ruler has to reproduce. At a = 0 the static observer's rest space is
        // the slice t = const of Schwarzschild time, whose radial lines are geodesics of the
        // spacetime by symmetry, and whose proper length is the elementary
        //     s(r) = integral dr / sqrt(1 - 2M/r) = sqrt(r(r-2)) + 2 ln(sqrt r + sqrt(r-2)).
        // Nothing of that appears in the integrator: it fires beta = pi from the axial tetrad of
        // u = d_t / sqrt(-g_tt) in *ingoing Kerr-Schild* coordinates, where the same curve has
        // t running away to -infinity, and the distance comes back anyway.
        let metric = KerrSchild::new(1.0, 0.0);
        let r0 = 6.0;
        let u = stationary(&metric, r0);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);
        let inward = spatial_direction(&tetrad, std::f64::consts::PI);
        assert!(inward[1] < 0.0, "beta = pi is the observer's inward direction: {inward:?}");

        let closed = |r: f64| (r * (r - 2.0)).sqrt() + 2.0 * ((r.sqrt()) + (r - 2.0).sqrt()).ln();
        let targets = [5.0, 4.0, 3.0, 2.5, 2.1];
        let got = shoot(&metric, [0.0, r0, 0.0], inward, 20.0, &targets);
        for (&r_h, slot) in targets.iter().zip(got.crossings.iter()) {
            let (s, event) = *slot.first().unwrap_or_else(|| panic!("the shot passes r = {r_h}"));
            assert_eq!(slot.len(), 1, "and passes it once: a radial ruler has no turning point");
            let want = closed(r0) - closed(r_h);
            println!("r = {r_h}: ruler {s:.9} M against the closed form {want:.9} M");
            assert!(
                (s - want).abs() < 1e-6,
                "the proper distance from r = 6 to r = {r_h} is {want}, the shot measured {s}"
            );
            assert!((event[1] - r_h).abs() < 1e-9, "and it reports the crossing event's own r");
        }

        // The horizon itself is a case apart, and the reason is geometry rather than arithmetic.
        // beta = pi from a *static* observer has k_t = k_phi = 0, so the radial equation reduces
        // to (dr/ds)^2 = 1 - 2M/r, whose root at r = 2 is a *double* one: the curve is tangent to
        // the horizon there, which is the statement that the slice ends on the bifurcation sphere
        // rather than passing through it. The distance to it is still the closed form's, and it is
        // still reported - but a tangency is a square root, so the last digits of s cost half the
        // digits of r, and the tolerance says so. This is the one target of the five-and-one whose
        // reading is not exact to the last bit, and it is the one where "how far to the horizon"
        // has a different answer for every observer at the same place.
        let edge = shoot(&metric, [0.0, r0, 0.0], inward, 40.0, &[2.0]);
        let (s, _) = *edge.crossings[0]
            .first()
            .expect("the ruler reaches the edge of the static slice");
        let want = closed(r0) - closed(2.0);
        println!("r+ = 2 is the slice's own edge, ruler {s:.6} M against {want:.6} M");
        assert!(
            (s - want).abs() < 1e-4,
            "the static slice ends {want} M from r = 6; the ruler measured {s}"
        );
    }

    #[test]
    fn test_a_shot_past_the_horizon_from_inside_can_reach_r_plus() {
        // Region II, where r is a time: no *worldline* gets back out to r+, and the view's old
        // picture, which placed a surface by a linear map, could say nothing else. A ruler is
        // another matter. The slices t = const of this chart are spacelike in region II and run
        // straight out through the horizon, so an observer between the horizons has r+ at a finite
        // distance in some direction of their own space, and r- at a finite distance in another.
        // Both have to come back, which is what says the shot is not assuming r is monotone.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let r0 = 1.0;
        assert!(r0 > rm && r0 < rp, "r = {r0} has to be between r- = {rm} and r+ = {rp}");
        let u = falling(&metric, r0, 1.0, 2.2);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);

        let s_max = 2.0;
        let (mut out, mut inn) = (None, None);
        let mut twice: Option<(f64, f64, f64)> = None;
        for i in 0..72 {
            let beta = std::f64::consts::TAU * (i as f64) / 72.0;
            let hit = shoot(
                &metric,
                [0.0, r0, 0.0],
                spatial_direction(&tetrad, beta),
                s_max,
                &[rm, rp],
            );
            for (target, list) in [(rm, &hit.crossings[0]), (rp, &hit.crossings[1])] {
                let mut previous = 0.0;
                for (s, e) in list {
                    assert!((e[1] - target).abs() < 1e-6, "a crossing of r = {target} is at it");
                    assert!(*s > previous && *s <= s_max, "and they come in order of s: {list:?}");
                    previous = *s;
                }
            }
            if let Some((s, _)) = hit.crossings[1].first()
                && out.is_none_or(|(best, _): (f64, f64)| *s < best)
            {
                out = Some((*s, beta));
            }
            if let Some((s, _)) = hit.crossings[0].first()
                && inn.is_none_or(|(best, _): (f64, f64)| *s < best)
            {
                inn = Some((*s, beta));
            }
            // A direction that turns around inside comes back to r+ a second time, and that far
            // face is on the same sheet of Bob's own space as the near one.
            if hit.crossings[1].len() >= 2 {
                twice = Some((beta, hit.crossings[1][0].0, hit.crossings[1][1].0));
            }
        }
        let (s_out, b_out) = out.expect("some direction of the observer's own space reaches r+");
        let (s_in, b_in) = inn.expect("and some direction reaches r-");
        println!(
            "from r = 1.0 between the horizons: r+ = {rp:.4} is {s_out:.4} M away at \
             beta = {:.1} deg, r- = {rm:.4} is {s_in:.4} M away at beta = {:.1} deg",
            b_out.to_degrees(),
            b_in.to_degrees()
        );
        assert!(s_out > 0.0 && s_in > 0.0);
        let (beta, first, second) =
            twice.expect("some direction reaches r+, turns, and reaches it again");
        println!(
            "and at beta = {:.1} deg the same ruler meets r+ twice, at {first:.4} M and \
             {second:.4} M",
            beta.to_degrees()
        );
        assert!(second > first, "the second crossing is further along the ruler");
    }

    #[test]
    fn test_a_ruler_aimed_past_the_hole_crosses_a_surface_twice() {
        // The near face and the far face. A surface the observer stands outside of is a closed
        // curve that does not enclose them, so a ruler aimed into it goes in through one side and
        // out through the other, and both crossings are points of the section drawn in that
        // observer's own space. Keeping only the first would draw half of every one.
        //
        // Schwarzschild and a static observer, so the pair can be checked against a closed form
        // rather than against a picture. Every spatial direction of a static observer is
        // orthogonal to d_t, so k_t = 0 and the radial equation is
        //     (dr/ds)^2 = (1 - 2M/r)(1 - L^2/r^2),   L = k_phi = r0 sin beta,
        // whose turning point is at r = |L| once that is outside the horizon. At beta = 150
        // degrees from r = 6 that is r = 3 exactly: the ruler runs in past r = 4, turns at r = 3
        // and comes back out through r = 4, and the two crossings bracket the closest approach.
        //
        // The surface is r = 4 and not r+ on purpose, and the reason is geometry rather than
        // choice. With k_t = 0 the factor (1 - 2M/r) makes r = 2 a *double* root for every
        // direction at once: the static slice is the Einstein-Rosen bridge and r+ is its throat,
        // touched and never crossed. A static observer's ruler has no far face of the horizon to
        // find. `test_a_shot_past_the_horizon_from_inside_can_reach_r_plus` has the genuine double
        // crossing of r+, from an observer who is moving.
        let metric = KerrSchild::new(1.0, 0.0);
        let r0 = 6.0;
        let beta = 150.0f64.to_radians();
        let u = stationary(&metric, r0);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);
        let k = spatial_direction(&tetrad, beta);
        let l_ang = r0 * beta.sin();
        assert!(
            (l_ang - 3.0).abs() < 1e-9,
            "L = r0 sin beta has to be the closest approach: {l_ang}"
        );

        let hit = shoot(&metric, [0.0, r0, 0.0], k, 40.0, &[4.0]);
        assert_eq!(
            hit.crossings[0].len(),
            2,
            "r = 4 is met on the way in and on the way out: {:?}",
            hit.crossings[0]
        );
        let (s_in, e_in) = hit.crossings[0][0];
        let (s_out, e_out) = hit.crossings[0][1];
        println!(
            "at beta = 150 deg a ruler from r = 6 enters r = 4 at {s_in:.6} M and leaves it at \
             {s_out:.6} M, ending at r = {:.4} after {:.4} M",
            hit.r_end, hit.reach
        );
        assert!(s_out > s_in, "the far face is further along the ruler than the near one");
        assert!(
            (e_in[1] - 4.0).abs() < 1e-9 && (e_out[1] - 4.0).abs() < 1e-9,
            "and both crossings are on the surface"
        );

        // The two bracket the closest approach, and by the slice's own symmetry about it the
        // midpoint of the two lengths *is* the turning point: a ruler cut off there ends at r = 3.
        let turn = shoot(&metric, [0.0, r0, 0.0], k, 0.5 * (s_in + s_out), &[4.0]);
        println!("and the midpoint of the two, {:.6} M, is the turn at r = {:.6}",
            0.5 * (s_in + s_out), turn.r_end);
        assert!(
            (turn.r_end - l_ang).abs() < 1e-4,
            "halfway between the two crossings the ruler is at its closest approach r = {l_ang}, \
             not r = {}",
            turn.r_end
        );
        assert!(
            turn.r_end < 4.0,
            "which is inside the surface, so the pair really are its two faces"
        );
    }

    #[test]
    fn test_a_ruler_that_comes_back_reports_its_far_face_and_one_that_does_not_is_gone() {
        // Why a section drawn from these rulers can be an arc with one face and not two, and why
        // that is the geometry rather than the integration. See the module doc for the equation.
        //
        // The claim is a census. Of the directions from r = 3 that reach r+ at all, every one that
        // comes back out reports its far face - at the surface, where the chart stops - and every
        // one that does not has ended somewhere it cannot come back from: on the ring, which is a
        // simple root of the spacelike radial potential and so is genuinely reached, or frozen on
        // the way in to r-, which is the Cauchy horizon's own limit and is what the app draws a
        // frozen worldline against.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let r0 = 3.0;
        let u = falling(&metric, r0, 1.0, 2.2);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);
        let omega_h = metric.a / (2.0 * metric.m * rp);

        let (mut reach, mut twice, mut ring, mut froze_inner, mut froze_outer) = (0, 0, 0, 0, 0);
        for i in 0..72 {
            let beta = std::f64::consts::TAU * (i as f64) / 72.0;
            let k = spatial_direction(&tetrad, beta);
            let hit = shoot(&metric, [0.0, r0, 0.0], k, 15.625, &[R_STOP, rm, rp]);
            if hit.crossings[2].is_empty() {
                continue;
            }
            reach += 1;
            // Every direction that gets to r+ from outside has E - Omega_H L > 0, so it is the
            // *ingoing* crossing this chart covers and the outgoing one that it does not.
            let (e, l) = invariants_of(&metric, r0, &k);
            assert!(
                e - omega_h * l > 0.0,
                "a ruler reaching r+ from r = 3 has E - Omega_H L = {} > 0, so the chart has its \
                 near face and not its far one",
                e - omega_h * l
            );
            if hit.crossings[2].len() >= 2 {
                twice += 1;
                assert!(
                    (hit.r_end - rp).abs() < 1e-4,
                    "a ruler with a far face ends on r+, where the chart runs out: r = {}",
                    hit.r_end
                );
                continue;
            }
            if hit.r_end < R_STOP {
                ring += 1;
            } else if (hit.r_end - rm).abs() < 1e-4 {
                froze_inner += 1;
            } else if (hit.r_end - rp).abs() < 1e-4 {
                froze_outer += 1;
            } else {
                panic!(
                    "a ruler with one face of r+ ended at r = {} for no reason this test knows",
                    hit.r_end
                );
            }
        }
        println!(
            "from r = 3: {reach} of 72 directions reach r+, {twice} of them twice; of the rest \
             {ring} died on the ring, {froze_inner} froze on the way in to r-, {froze_outer} on r+"
        );
        assert!(reach > 0 && twice > 0, "some rulers reach r+, and some come back out");
        assert_eq!(
            reach - twice,
            ring + froze_inner + froze_outer,
            "every direction with only the near face has an end this test accounted for"
        );
        assert!(ring > 0, "and most of them are rulers the ring destroyed");
    }

    #[test]
    fn test_a_shot_reports_no_crossing_it_did_not_make() {
        // s_max is the canvas's own reach, so a surface further away than the picture is wide has
        // no point on this row at all - and the shot has to say so rather than extrapolate to it.
        let metric = KerrSchild::new(1.0, 0.9);
        let r0 = 6.0;
        let u = falling(&metric, r0, 1.0, 2.2);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);
        let inward = spatial_direction(&tetrad, std::f64::consts::PI);
        let rp = metric.outer_horizon();

        let reached = shoot(&metric, [0.0, r0, 0.0], inward, 20.0, &[rp]);
        let (distance, _) = *reached.crossings[0]
            .first()
            .expect("r+ is within 20 M of proper length of r = 6");
        assert!(distance > 1.0, "and it is a real distance: {distance} M");

        let short = shoot(&metric, [0.0, r0, 0.0], inward, 0.5 * distance, &[rp]);
        assert!(
            short.crossings[0].is_empty(),
            "a ruler half as long as the distance reports nothing, not {:?}",
            short.crossings[0]
        );
        assert!(
            (short.reach - 0.5 * distance).abs() < 1e-9,
            "and it ran its whole length: {} of {}",
            short.reach,
            0.5 * distance
        );
        // Outward, the same surface is behind the shot and stays behind it however long the ruler
        // - and the shot proves that to itself and stops rather than running the whole 50 M.
        let outward = spatial_direction(&tetrad, 0.0);
        let away = shoot(&metric, [0.0, r0, 0.0], outward, 50.0, &[rp]);
        assert!(
            away.crossings[0].is_empty(),
            "and r+ is not out that way at all: {:?}",
            away.crossings[0]
        );
        assert!(away.reach < 1.0, "an escaping ruler stops at once: {} M", away.reach);
        println!(
            "r+ is {distance:.4} M inward from r = 6; a {:.4} M ruler and a 50 M outward one both \
             report nothing",
            0.5 * distance
        );
    }
}



