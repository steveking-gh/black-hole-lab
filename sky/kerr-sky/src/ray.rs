//! One light ray, traced backward from the observer to where it came from.
//!
//! # Backward
//!
//! The light arriving at the observer has a future-directed 4-momentum p (see `observer`). The
//! super-Hamiltonian (1/2) g^{mu nu} p_mu p_nu is even in p, so if (x(lambda), p(lambda)) solves
//! Hamilton's equations then so does (x(-lambda), -p(-lambda)): the same curve run the other way.
//! The ray is therefore followed forward in its own affine parameter with the covector k = -p,
//! which is past-directed, and every step goes back in time. Nothing is integrated with a negative
//! step, and the step control never has to think about signs. What k carries is E = -p_t = k_t,
//! constant; the constants reported are those of p.
//!
//! # The integration parameter and the step control
//!
//! The parameter is the affine one, lambda, with p normalised to unit frequency in the observer's
//! frame. The alternatives were weighed. Mino time (d lambda = Sigma d sigma) separates the
//! equations in Boyer-Lindquist coordinates, but here there is nothing to separate, and far from
//! the hole dr/d sigma = sqrt(R) ~ E r^2 makes the sigma-steps shrink as 1/r^2 for the same
//! distance covered. Coordinate time t runs to minus infinity on the approach to the past horizon,
//! so a t-step there covers nothing. In lambda a ray far from the hole is a straight line
//! travelled at a constant rate: dx/d lambda -> p, bent by forces of size M / r^2.
//!
//! The step is Dormand-Prince 5(4) with its embedded error estimate, first-same-as-last, as in the
//! app's `ray_dopri5`. The error is measured relative to the ray's own scales: the position error
//! against the distance from the hole |x|, and the momentum error against |p|,
//!
//!     err = max(|delta x| / |x|, |delta p| / |p|) / TOLERANCE,
//!
//! and the step is accepted when err <= 1, the next one scaled by 0.9 err^{-1/5} within [0.2, 5].
//! That is what lets a ray cost little far from the hole. At radius r the path's curvature is of
//! order M / r^2 and its k-th derivative of order M / r^{k+1}, so the local error of a fifth-order
//! step h is ~ M h^6 / r^6 while the allowance is TOLERANCE r: the accepted h grows like
//! r (TOLERANCE r / M)^{1/6}, faster than r itself, and the steps from 10 M to the far radius are a
//! handful per decade (the growth factor of 5 per step and the reach cap below are what bind).
//! Near the photon orbits the curvature is of order 1 / M and the same criterion asks for steps of
//! a fraction of M: a ray that winds is followed round every turn at the accuracy it needs. The
//! conservation test measures what that buys: L_z, Q and the null condition held to a few parts in
//! 1e9 over rays that turn eight times, and to about 1e-11 on rays that do not wind.
//!
//! What the relative position error does not protect is the impact parameter of a ray on a long
//! *inbound* leg far from the hole: there an error of TOLERANCE |x| across the ray is a change of
//! b, which the ray then carries into the strong field. A ray traced backward from an observer
//! near the hole never has such a leg - far from the hole it is always outbound, where a position
//! error across it is only an angle of TOLERANCE seen from the hole - so this costs the tracer
//! nothing; it is why the weak-field test starts its rays at their periapsis rather than at 1e8 M.
//!
//! One cap is geometric rather than an error bound: no step may carry the ray further than
//! `STEP_REACH` of its current distance from the hole. An error estimate is a statement about a
//! step's own stages, and a step long enough to jump over the hole can have stages that never feel
//! it; the cap keeps every step inside the region its stages sample.
//!
//! # Fates
//!
//! **Far sky (fate 1).** The ray is outbound (dr/d lambda > 0) past `FAR_RADIUS` and its radial
//! potential R(r) (below) has no root beyond, so it can never turn back. The direction at infinity
//! is the ray's direction of travel dx/d lambda there, not its position, which differ by b / r.
//!
//! The direction of travel still differs from its limit by the bending still to come, and in this
//! chart that is far smaller than the M b / r^2 of a harmonic or Schwarzschild chart. The backward
//! ray's outbound leg is the time reverse of light falling in, and the chart is built on the
//! ingoing principal null congruence l: along such a ray k is nearly parallel to l, with
//! s = l^mu k_mu = -E (1 - cos psi) ~ -E b^2 / (2 r^2) for psi ~ b / r the angle between the ray
//! and the radial direction. Every force in Hamilton's equations carries s (they are
//! s^2 d_i H + 2 H s (d_i l_j) k_j), and the transverse one is 2 H s times the transverse part of
//! k over r, ~ (M / r)(E b^2 / r^2)(E b / r^2): integrated outward from r it leaves a bending of
//! order M b^3 / r^4. Measured against the same rays followed to 1e9 M at a thousandth of the
//! tolerance, the raw direction's error falls by a factor ~100 per half decade of far radius,
//! as that says, and is ~1e-12 rad at 1e4 M for b ~ 30 M; and against the exact Schwarzschild
//! integral it grows as b^3 at a fixed far radius, which is why the far radius is the larger of
//! `FAR_RADIUS` and `FAR_PER_IMPACT` b. The plan's 1 / r^2 extrapolation was
//! built and measured and is not used: with no 1 / r^2 term to remove it only added the step
//! noise of a direction recorded further in, and made the result twenty times worse.
//! `test_the_direction_at_infinity_is_right_to_well_under_sixteen_arcseconds` holds the default
//! to that reference.
//!
//! **Past horizon (fate 2).** Traced backward, a ray that did not come from the sky came out of the
//! past horizon: it approaches r = r+ from outside as t -> minus infinity, winding with the
//! horizon's generators, and in this chart never arrives. It is classified instead of integrated to
//! exhaustion, by two criteria.
//!
//! *The closed criterion*, used wherever the ray is inbound (dr/d lambda < 0) in region I (r > r+).
//! For light with constants E, L_z, Q the radial equation in Mino time is (dr/d sigma)^2 = R(r) with
//!
//!     R(r) = [E (r^2 + a^2) - a L_z]^2 - Delta(r) [Q + (L_z - a E)^2]
//!          = E^2 r^4 + [2 E (E a^2 - a L_z) - K] r^2 + 2 M K r + (E a^2 - a L_z)^2 - a^2 K,
//!
//! K = Q + (L_z - a E)^2 >= 0. An inbound ray at r_c turns round exactly where R has a zero. If R > 0
//! on the whole of (r+, r_c], dr/d sigma = -sqrt(R) never vanishes, r decreases monotonically, and
//! it cannot settle above r+ (that would need R = 0 there): the ray reaches r+ - which, for a ray
//! running backward in time from region I, is the past horizon. So
//!
//!     fate 2  <=>  R(r_c) > 0 and R(s) > 0 at every stationary point s of R in (r+, r_c),
//!
//! the stationary points being the real roots of the depressed cubic R'(r) = 4 E^2 r^3 +
//! 2 [2 E (E a^2 - a L_z) - K] r + 2 M K, found in closed form. This holds for any E, K and spin in
//! region I, where Delta > 0; if R does vanish below r_c the ray turns and the integration goes on.
//! It is applied at the observer (so a ray that looks into the shadow costs no integration at all)
//! and again wherever a traced ray turns from outbound to inbound.
//!
//! **Between the horizons: the other branch of r+.** Inside r+, Delta < 0 makes R > 0 for every
//! ray and r is a time: a past-directed ray is always outbound and reaches r = r+ in finite affine
//! parameter. But not every such ray crosses into region I. At r = r+ the Mino-time radial velocity
//! of the future-directed p is Sigma dr/d lambda = -P(r+), with
//!
//!     P(r+) = E (r+^2 + a^2) - a L_z = (r+^2 + a^2) (E - Omega_H L_z),   Omega_H = a / (r+^2 + a^2),
//!
//! and light crossing the future horizon inward from region I needs dr/d lambda <= 0 there, so
//! E - Omega_H L_z >= 0. A ray between the horizons with E - Omega_H L_z < 0 did not come from
//! region I at all. Traced backward it closes on r+ from inside while t -> minus infinity, winding
//! with the horizon's generators, exactly as a region-I ray closes on the past horizon from
//! outside: it is heading for the *other* branch of r+ - the future horizon's left half in the
//! Kruskal diagram, beyond which lie the other exterior of the eternal hole and its white hole.
//! (It is the r+ twin of `kerr_equatorial`'s E - Omega_- L criterion for the right branch of r-.) In a
//! hole formed by collapse there is no other exterior: that light left the collapsing star. Either
//! way it is not light from this universe's far sky, and this crate gives it fate 2, the dark
//! part of the sky, with its own `Decision` so that a later format can give it a code of its own.
//! The specification's fate 2 is named "dark" and defined to hold both cases (section 4.7).
//!
//! The closed form is the sign of E - Omega_H L_z, fixed at the observer; the fallback, when that
//! is switched off, is the stall itself: the ray inside r+, outbound, within `LEFT_BRANCH_GAP` of
//! r+, and with its coordinate radial speed down to |dr/dt| < 2 kappa_+ (r+ - r). On the approach
//! to either branch every ray tends to the horizon's outgoing principal direction, whose
//! dr/dt = Delta / (r^2 + a^2 + 2 M r) ~ kappa_+ (r - r+) with kappa_+ = (r+ - M) / (2 M r+), so a ray
//! closing on the other branch has |dr/dt| / (r+ - r) -> kappa_+, while one about to cross has a
//! finite dr/dt there and the ratio diverges. The factor 2 separates the two, and the tests compare
//! the fallback with the closed form.
//!
//! *The proximity criterion*, the fallback wherever the closed one is not used (it can be switched
//! off, and the tests do, to compare them): the ray is inbound in region I and closer to r+ than
//! `PAST_HORIZON_MARGIN` = (r_ph - r+) / 2, with r_ph the prograde equatorial circular photon orbit.
//! Close to r+ the ray's radial motion stalls - in this chart dr/dt ~ (r - r+) -> 0 as it hugs the
//! horizon ever more tightly while t -> minus infinity - so there is no end to integrate to and a
//! threshold is needed. This one is justified by where light can turn: an E > 0 ray that turns
//! round in region I does so at a zero of R, and no zero of R for such a ray lies inside r_ph - the
//! photon region is where the double zeros are, and its innermost point is the prograde equatorial
//! orbit. `test_no_ray_from_the_sky_turns_round_inside_the_prograde_photon_orbit` checks that
//! statement over a quarter of a million rays. A ray inbound below r+ + (r_ph - r+) / 2 therefore
//! never turns and reaches the horizon; an E <= 0 ray (possible only inside the ergosphere) cannot
//! reach the far sky at all, so the criterion cannot misclassify it either.
//!
//! **Unresolved (fate 0).** The step budget `MAX_STEPS` ran out, or a step could not be made finite.
//! Counted by the frame tracer and never relabelled.
//!
//! # Shift and winding
//!
//! g = (-p . u) / E with u the observer's 4-velocity and E the conserved energy, exact.
//!
//! The winding counts whole turns about the spin axis along the light's travel from the far sky to
//! the observer, in the chart's azimuth phi = atan2(y, x) - atan2(a, r) (see `metric`). It is
//! accumulated continuously, as the sum over accepted steps of the change of phi wrapped into
//! (-pi, pi]; that is exact as long as no single step turns the ray by pi or more about the axis
//! as seen from the axis, which the reach cap ensures for any step that does not pass within a
//! hair of the axis itself. From the last step the change to the limit, the azimuth of d, is added
//! (that is where the far-sky azimuth is defined), and the total is negated to run from the sky to
//! the observer: Delta phi = phi(observer) - phi(infinity), winding = trunc(Delta phi / 2 pi).

use crate::metric::{Constants, Kerr};
use crate::observer::Triad;

/// Local error allowed per step, relative to the ray's distance from the hole and to its momentum.
/// The measured consequences are in the tests: L_z, Q and the null condition held to 3e-9 of their
/// scale over rays that turn eight times round the hole; directions at infinity within 6e-11 rad
/// of the Boyer-Lindquist oracle and of the exact Schwarzschild integral on rays that do not wind,
/// six orders below the sixteen arcseconds (7.8e-5 rad) the film asks of them; tightening it a
/// hundredfold moves directions by 3e-10 rad. Near the shadow's edge the rays' own conditioning
/// multiplies the error by ~e^{2 pi} a turn, and a ray within ~1e-10 of the critical curve on the
/// sky side can be carried inside it and called captured: the shadow's edge is found to ~4e-11 rad.
/// The cost is ~90 steps a ray at 6 M, 310 ns a step.
pub const TOLERANCE: f64 = 1e-10;

/// The smallest radius beyond which an outbound ray that cannot turn back is on the far sky.
///
/// The direction of travel there is left with the bending still to come, of order M b^3 / r^4
/// (module documentation): 3e-12 rad for b = 30 M at 1e4 M. A ray of larger impact parameter b is
/// followed out to `FAR_PER_IMPACT` b instead, which bounds the residual by M / (1e8 b) whatever b
/// is. Both are measured, against rays followed to 1e8 M and against the exact Schwarzschild
/// integral out to b = 3000 M. Going further costs a few steps and buys nothing the film can see.
pub const FAR_RADIUS: f64 = 1e4;

/// The far radius in units of the ray's impact parameter at infinity, b = sqrt(L_z^2 + Q) / E,
/// where that is the larger (see `FAR_RADIUS`).
pub const FAR_PER_IMPACT: f64 = 100.0;

/// Largest step, as a fraction of the ray's distance from the hole.
pub const STEP_REACH: f64 = 0.5;

/// Accepted steps a ray may take before it is declared unresolved.
pub const MAX_STEPS: u32 = 20_000;

/// How close to r+, from inside, a ray must come before the stall test for the other branch of r+
/// is applied, in units of M. Small enough that the ray's approach is in its asymptotic regime
/// (the corrections to dr/dt ~ kappa_+ (r - r+) are of relative order (r+ - r) / M), large enough
/// to be reached in a few dozen steps.
pub const LEFT_BRANCH_GAP: f64 = 1e-4;

/// The fate of a ray: specification section 4.7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Fate {
    /// The step budget ran out, or the integration broke down.
    Unresolved = 0,
    /// The ray reached the far sky.
    FarSky = 1,
    /// The ray did not come from the far sky: the past horizon seen from outside the hole, or the
    /// other branch of r+ seen from between the horizons (module documentation). Drawn black.
    Dark = 2,
}

/// Which rule decided a fate. Diagnostic only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    /// The far-radius rule.
    Far,
    /// The closed criterion on R(r), at the observer or at a turn.
    Closed,
    /// The proximity criterion.
    Proximity,
    /// Between the horizons, E - Omega_H L_z < 0: bound for the other branch of r+.
    OtherBranchClosed,
    /// Between the horizons, stalled against r+ from inside: bound for the other branch of r+.
    OtherBranchStall,
    /// `MAX_STEPS` ran out.
    Budget,
    /// A step could not be made finite.
    Breakdown,
}

/// Switches and limits of a trace. `Default` is what the frame tracer uses.
#[derive(Debug, Clone, Copy)]
pub struct TraceOptions {
    /// Local error allowed per step (`TOLERANCE`).
    pub tolerance: f64,
    /// `FAR_RADIUS`, the least far radius; a ray goes out to `FAR_PER_IMPACT` times its impact
    /// parameter if that is further.
    pub far_radius: f64,
    /// Whether to use the closed past-horizon criterion (the proximity criterion is always on).
    pub closed_criterion: bool,
    /// `MAX_STEPS`.
    pub max_steps: u32,
}

impl Default for TraceOptions {
    fn default() -> Self {
        Self {
            tolerance: TOLERANCE,
            far_radius: FAR_RADIUS,
            closed_criterion: true,
            max_steps: MAX_STEPS,
        }
    }
}

/// Where a ray came from.
#[derive(Debug, Clone, Copy)]
pub struct Outcome {
    /// Fate code.
    pub fate: Fate,
    /// Direction at infinity d in the far-sky frame (the chart's x, y, z axes); NaN unless fate 1.
    pub direction: [f64; 3],
    /// Shift g; NaN unless fate 1 or where no observer's 4-velocity was given.
    pub shift: f64,
    /// Change of the chart azimuth along the light's travel from the far sky to the observer; NaN
    /// unless fate 1.
    pub delta_phi: f64,
    /// trunc(delta_phi / 2 pi); 0 unless fate 1.
    pub winding: i32,
    /// Accepted integration steps.
    pub steps: u32,
    /// Which rule decided.
    pub decided_by: Decision,
    /// E, L_z, Q of the arriving light's p at the observer.
    pub constants: Constants,
}

/// The radial potential R(r) = c4 r^4 + c2 r^2 + c1 r + c0 of a null geodesic (module
/// documentation), for the past-horizon and far-sky criteria.
#[derive(Debug, Clone, Copy)]
pub struct RadialPotential {
    c4: f64,
    c2: f64,
    c1: f64,
    c0: f64,
}

impl RadialPotential {
    /// R for the constants E, L_z, Q of a ray in this hole.
    pub fn new(kerr: &Kerr, c: &Constants) -> Self {
        let (e, l, a, m) = (c.energy, c.lz, kerr.a, kerr.m);
        let k = c.carter_q + (l - a * e) * (l - a * e);
        let x = e * a * a - a * l;
        Self {
            c4: e * e,
            c2: 2.0 * e * x - k,
            c1: 2.0 * m * k,
            c0: x * x - a * a * k,
        }
    }

    /// R(r).
    pub fn at(&self, r: f64) -> f64 {
        let r2 = r * r;
        (self.c4 * r2 + self.c2) * r2 + self.c1 * r + self.c0
    }

    /// The real stationary points of R: roots of R'(r) = 4 c4 r^3 + 2 c2 r + c1, a depressed cubic,
    /// in closed form (trigonometric for three real roots, Cardano's for one) and polished by two
    /// Newton steps. Returned in a fixed array with a count.
    pub fn stationary_points(&self) -> ([f64; 3], usize) {
        let mut out = [0.0; 3];
        if self.c4 == 0.0 {
            // E = 0: R is a quadratic in r, R' = 2 c2 r + c1.
            if self.c2 != 0.0 {
                out[0] = -self.c1 / (2.0 * self.c2);
                return (out, 1);
            }
            return (out, 0);
        }
        // r^3 + p r + q = 0.
        let p = self.c2 / (2.0 * self.c4);
        let q = self.c1 / (4.0 * self.c4);
        let disc = q * q / 4.0 + p * p * p / 27.0;
        let n = if disc > 0.0 {
            let s = disc.sqrt();
            out[0] = (-q / 2.0 + s).cbrt() + (-q / 2.0 - s).cbrt();
            1
        } else if p == 0.0 {
            out[0] = 0.0;
            1
        } else {
            let m = 2.0 * (-p / 3.0).sqrt();
            let arg = (3.0 * q / (p * m)).clamp(-1.0, 1.0);
            let theta = arg.acos() / 3.0;
            for (k, root) in out.iter_mut().enumerate() {
                *root = m * (theta - std::f64::consts::TAU * k as f64 / 3.0).cos();
            }
            3
        };
        for root in out.iter_mut().take(n) {
            for _ in 0..2 {
                let f = (root.powi(2) + p) * *root + q;
                let df = 3.0 * root.powi(2) + p;
                if df != 0.0 {
                    *root -= f / df;
                }
            }
        }
        (out, n)
    }

    /// Whether R > 0 on the whole of (lo, hi]: R(hi) > 0 and R > 0 at every stationary point
    /// inside. (R(lo) is not asked about: at lo = r+ it is P(r+)^2 >= 0, and a zero there is the
    /// horizon itself.)
    pub fn positive_between(&self, lo: f64, hi: f64) -> bool {
        if self.at(hi).partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return false;
        }
        let (roots, n) = self.stationary_points();
        roots[..n]
            .iter()
            .all(|&s| !(s > lo && s < hi) || self.at(s) > 0.0)
    }

    /// Whether R > 0 on the whole of [lo, infinity): no turning point beyond lo.
    pub fn positive_beyond(&self, lo: f64) -> bool {
        if self.c4 <= 0.0 || self.at(lo).partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return false;
        }
        let (roots, n) = self.stationary_points();
        roots[..n].iter().all(|&s| s <= lo || self.at(s) > 0.0)
    }
}

/// A ray being followed backward: position and the past-directed covector k = -p, with k_t = E.
#[derive(Debug, Clone, Copy)]
pub struct NullRay {
    /// (x, y, z, k_x, k_y, k_z).
    pub state: [f64; 6],
    /// k_t = -p_t = E.
    pub energy: f64,
    /// Affine parameter travelled, backward, in units where the observer measures unit frequency.
    pub lambda: f64,
    /// d state / d lambda at `state`: the next step's first stage.
    slope: [f64; 6],
    /// The step the controller proposes next.
    h: f64,
    /// Accepted steps.
    pub steps: u32,
    /// Length scale below which the position error is measured absolutely: M, or 1 in flat space.
    scale: f64,
}

impl NullRay {
    /// Start following backward the light that arrives at `position` with future-directed
    /// covariant momentum p.
    pub fn backward(kerr: &Kerr, position: [f64; 3], p: [f64; 4]) -> Self {
        let state = [position[0], position[1], position[2], -p[1], -p[2], -p[3]];
        let energy = -p[0];
        let slope = kerr.hamilton(-energy, &state);
        let reach = norm3(&position) / norm3(&[slope[0], slope[1], slope[2]]).max(1e-300);
        let scale = if kerr.m > 0.0 { kerr.m } else { 1.0 };
        Self {
            state,
            energy,
            lambda: 0.0,
            slope,
            h: 0.02 * reach.max(scale),
            steps: 0,
            scale,
        }
    }

    /// The position (x, y, z).
    pub fn position(&self) -> [f64; 3] {
        [self.state[0], self.state[1], self.state[2]]
    }

    /// The future-directed covariant momentum p = -k of the light at the ray's current point.
    pub fn covector(&self) -> [f64; 4] {
        [-self.energy, -self.state[3], -self.state[4], -self.state[5]]
    }

    /// The backward direction of travel dx / d lambda.
    pub fn velocity(&self) -> [f64; 3] {
        [self.slope[0], self.slope[1], self.slope[2]]
    }

    /// Take one accepted step of at most `h_max` in lambda, or fail if no finite step can be made
    /// even at a step of 1e-14 of the length scale.
    pub fn step(&mut self, kerr: &Kerr, tolerance: f64, h_max: f64) -> Result<(), StepFailed> {
        let pos = self.position();
        let reach = STEP_REACH * norm3(&pos).max(self.scale) / norm3(&self.velocity()).max(1e-300);
        let proposal = self.h;
        let mut h = proposal.min(reach).min(h_max);
        let limited = h < proposal;
        let mut rejected = false;
        loop {
            // The negation is the point: an h that has come out NaN must fail here too.
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !(h > 1e-14 * self.scale / norm3(&self.velocity()).max(1e-300)) {
                return Err(StepFailed);
            }
            let (next, slope, err) = dopri5(kerr, -self.energy, &self.state, &self.slope, h);
            let finite = next.iter().chain(slope.iter()).all(|v| v.is_finite());
            if !finite {
                h *= 0.25;
                rejected = true;
                continue;
            }
            let pos_scale = norm3(&pos)
                .max(norm3(&[next[0], next[1], next[2]]))
                .max(self.scale);
            let mom_scale = norm3(&[self.state[3], self.state[4], self.state[5]])
                .max(norm3(&[next[3], next[4], next[5]]))
                .max(1e-300);
            let e = (norm3(&[err[0], err[1], err[2]]) / pos_scale)
                .max(norm3(&[err[3], err[4], err[5]]) / mom_scale)
                / tolerance;
            if !e.is_finite() {
                // An interior stage went non-finite although the result did not.
                h *= 0.25;
                rejected = true;
                continue;
            }
            let factor = if e > 0.0 {
                (0.9 * e.powf(-0.2)).clamp(0.2, 5.0)
            } else {
                5.0
            };
            if e <= 1.0 {
                self.state = next;
                self.slope = slope;
                self.lambda += h;
                self.steps += 1;
                // A step cut short by `h_max` or by the reach says nothing against the step the
                // error allows, so the proposal is not lowered on its account.
                self.h = if limited && !rejected {
                    proposal.max(h * factor)
                } else {
                    h * factor
                };
                return Ok(());
            }
            rejected = true;
            h *= factor;
        }
    }

    /// Follow the ray until lambda has advanced by exactly `span` (tests use this to compare
    /// against other integrations at equal affine parameter).
    pub fn advance(&mut self, kerr: &Kerr, tolerance: f64, span: f64) -> Result<(), StepFailed> {
        let end = self.lambda + span;
        while self.lambda < end {
            let left = end - self.lambda;
            self.step(kerr, tolerance, left)?;
            if end - self.lambda <= 1e-15 * end.abs() {
                break;
            }
        }
        Ok(())
    }
}

/// No finite step could be made from a ray's state: the integration has broken down there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepFailed;

impl std::fmt::Display for StepFailed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "no finite integration step could be made from this state"
        )
    }
}

impl std::error::Error for StepFailed {}

fn norm3(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Wrap an angle into (-pi, pi].
fn wrap(angle: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let w = angle - tau * (angle / tau).round();
    if w <= -std::f64::consts::PI {
        w + tau
    } else {
        w
    }
}

/// One Dormand-Prince 5(4) step of Hamilton's equations with p_t = -`energy_arg`, reusing the slope
/// k1 at y: the fifth-order state, its slope (first-same-as-last) and the fifth-minus-fourth-order
/// difference.
fn dopri5(
    kerr: &Kerr,
    energy_arg: f64,
    y: &[f64; 6],
    k1: &[f64; 6],
    h: f64,
) -> ([f64; 6], [f64; 6], [f64; 6]) {
    const A: [[f64; 6]; 6] = [
        [1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0],
        [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0],
        [
            19372.0 / 6561.0,
            -25360.0 / 2187.0,
            64448.0 / 6561.0,
            -212.0 / 729.0,
            0.0,
            0.0,
        ],
        [
            9017.0 / 3168.0,
            -355.0 / 33.0,
            46732.0 / 5247.0,
            49.0 / 176.0,
            -5103.0 / 18656.0,
            0.0,
        ],
        [
            35.0 / 384.0,
            0.0,
            500.0 / 1113.0,
            125.0 / 192.0,
            -2187.0 / 6784.0,
            11.0 / 84.0,
        ],
    ];
    // b - b*: fifth-order weights less the embedded fourth-order ones.
    const E: [f64; 7] = [
        71.0 / 57600.0,
        0.0,
        -71.0 / 16695.0,
        71.0 / 1920.0,
        -17253.0 / 339200.0,
        22.0 / 525.0,
        -1.0 / 40.0,
    ];
    let mut k = [*k1; 7];
    let mut next = *y;
    for (stage, row) in A.iter().enumerate() {
        let mut arg = *y;
        for (i, c) in arg.iter_mut().enumerate() {
            let mut sum = 0.0;
            for (j, a) in row[..=stage].iter().enumerate() {
                sum += a * k[j][i];
            }
            *c += h * sum;
        }
        k[stage + 1] = kerr.hamilton(energy_arg, &arg);
        next = arg;
    }
    let mut err = [0.0; 6];
    for (i, e) in err.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (j, w) in E.iter().enumerate() {
            sum += w * k[j][i];
        }
        *e = h * sum;
    }
    (next, k[6], err)
}

/// Half the gap between r+ and the prograde photon orbit: see the module documentation.
pub fn past_horizon_margin(kerr: &Kerr) -> f64 {
    0.5 * (kerr.prograde_photon_orbit() - kerr.outer_horizon())
}

/// Trace backward the light arriving at `position` with future-directed covariant momentum p.
/// The shift is left NaN: it needs the observer's 4-velocity, which [`trace_direction`] supplies.
pub fn trace_covector(
    kerr: &Kerr,
    position: [f64; 3],
    p: [f64; 4],
    options: &TraceOptions,
) -> Outcome {
    let constants = kerr.constants(position, p);
    let potential = RadialPotential::new(kerr, &constants);
    let r_plus = kerr.outer_horizon();
    let has_horizon = kerr.m > 0.0;
    let margin = past_horizon_margin(kerr);
    let mut ray = NullRay::backward(kerr, position, p);

    let finish = |fate: Fate, decided_by: Decision, steps: u32| Outcome {
        fate,
        direction: [f64::NAN; 3],
        shift: f64::NAN,
        delta_phi: f64::NAN,
        winding: 0,
        steps,
        decided_by,
        constants,
    };

    let mut r = kerr.radius(position);
    let mut rate = kerr.radial_rate(&ray.state, &ray.velocity());
    // At the observer: an inbound ray in region I with no turning point above r+ is in the
    // shadow, and costs nothing more.
    if has_horizon
        && options.closed_criterion
        && r > r_plus
        && rate < 0.0
        && potential.positive_between(r_plus, r)
    {
        return finish(Fate::Dark, Decision::Closed, 0);
    }
    // Between the horizons: bound for the other branch of r+ exactly when E - Omega_H L_z < 0.
    let omega_h = kerr.a / (r_plus * r_plus + kerr.a * kerr.a);
    if has_horizon
        && options.closed_criterion
        && r < r_plus
        && constants.energy - omega_h * constants.lz < 0.0
    {
        return finish(Fate::Dark, Decision::OtherBranchClosed, 0);
    }
    let kappa_plus = if has_horizon {
        (r_plus - kerr.m) / (2.0 * kerr.m * r_plus)
    } else {
        0.0
    };

    let impact = (constants.lz * constants.lz + constants.carter_q)
        .max(0.0)
        .sqrt()
        / constants.energy.abs();
    let far_radius = if impact.is_finite() {
        options.far_radius.max(FAR_PER_IMPACT * impact)
    } else {
        options.far_radius
    };
    let mut phi = kerr.azimuth(position);
    let mut turned = 0.0f64;
    let mut inbound = rate < 0.0;
    while ray.steps < options.max_steps {
        if ray.step(kerr, options.tolerance, f64::INFINITY).is_err() {
            return finish(Fate::Unresolved, Decision::Breakdown, ray.steps);
        }
        let pos = ray.position();
        r = kerr.radius(pos);
        let next_phi = kerr.azimuth(pos);
        turned += wrap(next_phi - phi);
        phi = next_phi;
        rate = kerr.radial_rate(&ray.state, &ray.velocity());

        if has_horizon
            && options.closed_criterion
            && rate < 0.0
            && r > r_plus
            && !inbound
            && potential.positive_between(r_plus, r)
        {
            return finish(Fate::Dark, Decision::Closed, ray.steps);
        }
        if has_horizon && rate < 0.0 && r < r_plus + margin {
            return finish(Fate::Dark, Decision::Proximity, ray.steps);
        }
        if has_horizon && rate > 0.0 && r < r_plus && r_plus - r < LEFT_BRANCH_GAP {
            // dr/dt = (dr/d lambda) / (dt/d lambda), with dt/d lambda = -E + 2 H s along the
            // past-directed flow (s = l^mu k_mu = -E + l . k).
            let f = kerr.field(pos);
            let s =
                -ray.energy + f.l[0] * ray.state[3] + f.l[1] * ray.state[4] + f.l[2] * ray.state[5];
            let dt = -ray.energy + 2.0 * f.h * s;
            if (rate / dt).abs() < 2.0 * kappa_plus * (r_plus - r) {
                return finish(Fate::Dark, Decision::OtherBranchStall, ray.steps);
            }
        }
        inbound = rate < 0.0;

        if rate > 0.0 && r > far_radius && potential.positive_beyond(r) {
            let v = ray.velocity();
            let n = norm3(&v);
            let d = [v[0] / n, v[1] / n, v[2] / n];
            // The last stretch of azimuth, from here to the limit, which is the azimuth of d; then
            // the whole change run the way the light ran.
            turned += wrap(d[1].atan2(d[0]) - phi);
            let delta_phi = -turned;
            return Outcome {
                fate: Fate::FarSky,
                direction: d,
                shift: f64::NAN,
                delta_phi,
                winding: (delta_phi / std::f64::consts::TAU).trunc() as i32,
                steps: ray.steps,
                decided_by: Decision::Far,
                constants,
            };
        }
    }
    finish(Fate::Unresolved, Decision::Budget, ray.steps)
}

/// Trace backward the light the observer sees looking along n (a unit vector in the triad), and
/// fill in the shift g = (-p . u) / E.
///
/// `Triad::look` builds p with -p . u = 1 exactly (unit frequency in the observer's frame), so
/// g = 1 / E. Evaluating -p . u again by a dot product would be the one cancellation that
/// construction exists to avoid.
pub fn trace_direction(kerr: &Kerr, triad: &Triad, n: [f64; 3], options: &TraceOptions) -> Outcome {
    let p = kerr.lower(triad.position, triad.look(n));
    let mut out = trace_covector(kerr, triad.position, p, options);
    if out.fate == Fate::FarSky {
        out.shift = 1.0 / out.constants.energy;
    }
    out
}

#[cfg(test)]
// Tensor components are indexed by their indices, as the formulae write them.
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;

    #[test]
    fn test_no_ray_from_the_sky_turns_round_inside_the_prograde_photon_orbit() {
        // The statement the proximity criterion rests on. For E = 1 (any E > 0 scales to it) and
        // a quarter of a million (L_z, Q) with Q >= 0 - every ray through the equatorial plane has
        // Q = p_theta^2 >= 0 there - the largest zero of R above r+, where there is one, is
        // outside the prograde equatorial photon orbit. The zeros are found as sign changes of R
        // between its stationary points, which `RadialPotential` computes; the grid is fine near
        // Q = 0 and near the critical impact parameters, where the zeros come closest.
        for &a in &[0.0, 0.5, 0.9, 0.99, 0.998] {
            let kerr = Kerr::new(1.0, a);
            let (rp, rph) = (kerr.outer_horizon(), kerr.prograde_photon_orbit());
            let mut closest = f64::INFINITY;
            let mut seen = 0;
            for i in 0..500 {
                let lz = -12.0 + 24.0 * (i as f64 + 0.5) / 500.0;
                for j in 0..500 {
                    let q = 40.0 * ((j as f64 + 0.5) / 500.0).powi(3);
                    let c = Constants {
                        energy: 1.0,
                        lz,
                        carter_q: q,
                        hamiltonian: 0.0,
                    };
                    let pot = RadialPotential::new(&kerr, &c);
                    // Largest zero above r+: bisect on the last sign change, scanning down from
                    // far out through the stationary points.
                    let (roots, n) = pot.stationary_points();
                    let mut marks: Vec<f64> =
                        roots[..n].iter().copied().filter(|&s| s > rp).collect();
                    marks.push(rp);
                    marks.sort_by(|x, y| y.total_cmp(x));
                    let mut hi = 1e3;
                    for &lo in &marks {
                        if pot.at(lo) <= 0.0 && pot.at(hi) > 0.0 {
                            let (mut lo, mut hi) = (lo, hi);
                            for _ in 0..200 {
                                let mid = 0.5 * (lo + hi);
                                if pot.at(mid) > 0.0 {
                                    hi = mid
                                } else {
                                    lo = mid
                                }
                            }
                            closest = closest.min(hi);
                            seen += 1;
                            break;
                        }
                        hi = lo;
                    }
                }
            }
            println!(
                "a = {a}: {seen} turning rays, innermost turn at {closest:.6} M; r+ = {rp:.6}, \
                 prograde photon orbit {rph:.6}, margin used {:.6}",
                past_horizon_margin(&kerr)
            );
            assert!(
                closest >= rph * (1.0 - 1e-9),
                "a ray turns at {closest} inside r_ph = {rph}"
            );
        }
    }

    #[test]
    fn test_the_stationary_points_of_the_radial_potential_are_its_turning_points() {
        for &(e, lz, q) in &[
            (1.0, 2.0, 5.0),
            (1.0, -7.0, 0.0),
            (0.7, 3.0, 30.0),
            (1.0, 0.0, 27.0),
        ] {
            let kerr = Kerr::new(1.0, 0.8);
            let pot = RadialPotential::new(
                &kerr,
                &Constants {
                    energy: e,
                    lz,
                    carter_q: q,
                    hamiltonian: 0.0,
                },
            );
            let (roots, n) = pot.stationary_points();
            assert!(n >= 1);
            for &s in &roots[..n] {
                let d = (pot.at(s + 1e-6) - pot.at(s - 1e-6)) / 2e-6;
                let size = pot.at(s).abs() + 1.0;
                assert!(d.abs() < 1e-6 * size * 100.0, "R'({s}) = {d}");
            }
            // And R itself matches its unexpanded form.
            let (a, m) = (kerr.a, kerr.m);
            for &r in &[1.3, 2.7, 9.0] {
                let delta = r * r - 2.0 * m * r + a * a;
                let want =
                    (e * (r * r + a * a) - a * lz).powi(2) - delta * (q + (lz - a * e).powi(2));
                assert!((pot.at(r) - want).abs() < 1e-10 * (1.0 + want.abs()));
            }
        }
    }
}
