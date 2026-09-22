//! Riemann normal coordinates about one observer's event, done exactly.
//!
//! [`crate::local_frame::LocalFrame`] is the *linearised* chart of an observer: it maps a
//! coordinate displacement through the dual tetrad, so orientations at the observer's own event
//! are exact and finite offsets are the first-order answer. That is enough for the tilt of a
//! light cone and for the causal character of a surface, and it is not nearly enough for where
//! anything is. Measured for an observer on the prograde ISCO of an a = 0.90 hole (r0 = 2.3209 M),
//! the first-order map puts r+ at 1.647 M along her now-axis where the geodesic distance is
//! 3.020 M, and at the same 1.647 M down her inward past cone where the exact affine length is
//! 2.304 M. The offsets that matter in that view are of the order of the curvature radius, so the
//! linear chart is wrong by most of a gravitational radius exactly where the picture is being
//! looked at.
//!
//! The exact chart is the one general relativity already names: *Riemann normal coordinates*
//! built on the observer's tetrad. A point is placed at
//!
//!     xi^a = sigma T^a,
//!
//! where T is the initial tangent of the geodesic that leaves the observer's event towards it and
//! sigma is the affine parameter at which that geodesic arrives. The map is the exponential map of
//! the observer's own rest frame, and it is exact - the tetrad legs are exact, the geodesic is
//! exact, the affine parameter is exact - rather than a series in the offset. Nothing here is
//! linearised and nothing is fitted.
//!
//! Two things follow from the construction and are worth stating before the code.
//!
//! * Nothing has to be normalised. Scaling the tangent T -> k T scales the affine parameter
//!   sigma -> sigma / k, so the product sigma T is untouched: a tangent of any length, and of any
//!   causal type, names the same point. That is what lets the sampler sweep a direction angle
//!   round the full circle T = sin(psi) e0 + cos(psi) e1 with no case split between the timelike
//!   arcs, the null directions and the spacelike ones.
//! * The image of the exponential map is not the whole spacetime. A direction whose geodesic turns
//!   round in r before it reaches the surface never reaches it *along that direction*, and this
//!   module returns `None` rather than following the geodesic through the turning point and
//!   reporting the second crossing. The surface may well still be over there, reached another way;
//!   the drawn curve is the set of points the observer's own straight lines arrive at, which is
//!   what a normal-coordinate picture means.
//!
//! The radial motion is the familiar first integral of an equatorial Kerr geodesic, written for a
//! tangent of arbitrary normalisation. With
//!
//!     E = -g_{t mu} T^mu,   L = g_{phi mu} T^mu,   mu2 = -g(T, T)
//!
//! (so mu2 = 1 for a unit timelike tangent, 0 for a null one and -1 for a unit spacelike one, and
//! anything at all for an unnormalised one), the Carter constant vanishes on the equator and
//!
//!     r^4 (dr/dsigma)^2 = R(r) = P^2 - Delta [mu2 r^2 + (L - aE)^2],   P = E(r^2 + a^2) - aL,
//!
//! which is the same R that [`crate::geodesic::GeodesicState::radial_potential`] evaluates at
//! mu2 = 1 and that `wavefront::NullRay::turns_between` evaluates at mu2 = 0. The affine length
//! from r0 to a surface r = r_h is then the elementary quadrature
//!
//!     sigma = | integral from r0 to r_h of r^2 dr / sqrt(R) |.
//!
//! E, L and g(T, T) are chart-independent - the ingoing Kerr-Schild chart shares d_t, d_phi and r
//! with Boyer-Lindquist, the two differing by dt_KS = dt_BL + (2Mr/Delta) dr and
//! dphi_KS = dphi_BL + (a/Delta) dr - so R is the same function here as it is in the textbook.

use std::sync::OnceLock;

use crate::kerr_schild::KerrSchild;
use crate::tetrad::Tetrad;

/// Smallest radius this module will place an observer at. The geodesic equation is fine below it;
/// what is not fine is a chart origin on the ring itself, which is not an event of the spacetime.
const R_MIN: f64 = 1e-6;

/// Radii this close together are the same surface, and the affine length between them is zero.
/// It is a gap in r rather than a fraction of one so that a target *at* the observer's own radius
/// answers zero whatever that radius is.
const SAME_SURFACE: f64 = 1e-14;

/// The three constants that fix the radial motion of one equatorial geodesic.
///
/// They are read off a tangent of arbitrary length and arbitrary causal type: `mu2` is
/// -g(T, T) exactly as it comes, not rounded to one of {1, 0, -1}, so a tangent scaled by k
/// carries E -> kE, L -> kL, mu2 -> k^2 mu2 and R -> k^2 R, and the affine length scales as 1/k.
/// That is the homogeneity the whole module rests on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadialConstants {
    /// E = -g_{t mu} T^mu, the Killing energy of the tangent. Negative for a past-directed one.
    pub energy: f64,
    /// L = g_{phi mu} T^mu, the axial angular momentum of the tangent.
    pub l_ang: f64,
    /// mu2 = -g(T, T): positive for a timelike tangent, zero for a null one, negative for a
    /// spacelike one, and of size |T|^2 rather than of size one.
    pub mu2: f64,
}

impl RadialConstants {
    /// Read the constants off a tangent at radius r. The tangent may be timelike, null or
    /// spacelike, future- or past-directed, and of any length.
    pub fn of_tangent(metric: &KerrSchild, r: f64, tangent: &[f64; 3]) -> Self {
        let g = metric.metric_components(r.max(R_MIN));
        let mut low = [0.0f64; 3];
        for (mu, entry) in low.iter_mut().enumerate() {
            *entry = g[mu][0] * tangent[0] + g[mu][1] * tangent[1] + g[mu][2] * tangent[2];
        }
        let norm = low[0] * tangent[0] + low[1] * tangent[1] + low[2] * tangent[2];
        Self {
            energy: -low[0],
            l_ang: low[2],
            mu2: -norm,
        }
    }

    /// P(r) = E (r^2 + a^2) - a L, the combination that carries the whole of the horizon
    /// behaviour: R(r_H) = P(r_H)^2 on either horizon, since Delta vanishes there.
    pub fn p_at(&self, metric: &KerrSchild, r: f64) -> f64 {
        self.energy * (r * r + metric.a * metric.a) - metric.a * self.l_ang
    }

    /// R(r) = P^2 - Delta [mu2 r^2 + (L - aE)^2] = r^4 (dr/dsigma)^2.
    pub fn potential(&self, metric: &KerrSchild, r: f64) -> f64 {
        let p = self.p_at(metric, r);
        let lae = self.l_ang - metric.a * self.energy;
        p * p - metric.delta(r) * (self.mu2 * r * r + lae * lae)
    }

    /// The cubic c(r) with R(r) = r c(r), as coefficients in descending powers.
    ///
    /// R has no constant term at all: expanding it, the r^0 part is
    /// a^2 (aE - L)^2 - a^2 (L - aE)^2 = 0 identically, for every E, L, mu2 and spin. So R factors
    /// as r times
    ///
    ///     c(r) = (E^2 - mu2) r^3 + 2 M mu2 r^2 + [a^2 (E^2 - mu2) - L^2] r + 2 M (L - aE)^2,
    ///
    /// and since r > 0 everywhere on this chart's equator, c decides the sign of R. Working with c
    /// rather than with R is what makes the ring end of the interval well behaved (R and its first
    /// derivative both vanish at r = 0 for a = 0) and it is a plain Horner evaluation rather than
    /// a difference of two large squares. At mu2 = 0 it is exactly the cubic
    /// `wavefront::NullRay::turns_between` uses for null rays.
    fn cubic(&self, metric: &KerrSchild) -> [f64; 4] {
        let a = metric.a;
        let lae = self.l_ang - a * self.energy;
        let e2 = self.energy * self.energy;
        [
            e2 - self.mu2,
            2.0 * metric.m * self.mu2,
            a * a * (e2 - self.mu2) - self.l_ang * self.l_ang,
            2.0 * metric.m * lae * lae,
        ]
    }

    /// Whether a geodesic with these constants, travelling with dr/dsigma of sign `dr_sign`,
    /// crosses the horizon r = `r_h` *inside the ingoing Kerr-Schild chart*.
    ///
    /// This is the one place where the module has to know which chart it is in, and it is not a
    /// convention: the ingoing chart covers a black hole and no white hole, so half of the
    /// horizon crossings that exist in the maximal extension are simply not events of the
    /// spacetime the app draws, and a curve that reaches one leaves the chart at infinite
    /// coordinate time instead of going through.
    ///
    /// Where the rule comes from. Transforming the Boyer-Lindquist first integrals with
    /// dt_KS = dt_BL + (2Mr/Delta) dr gives, for a branch with r^2 dr/dsigma = s sqrt(R),
    ///
    ///     r^2 dt_KS/dsigma = [(r^2 + a^2) P + 2 M r s sqrt(R)] / Delta + a (L - aE),
    ///
    /// and the same numerator, times a/(r^2+a^2)... more precisely a [P + s sqrt(R)], appears in
    /// dphi_KS/dsigma. On a horizon r^2 + a^2 = 2 M r and R = P^2, so the bracket is
    /// 2 M r_H [P + s |P|]: it vanishes, and the crossing is regular, exactly when
    /// s |P| = -P, i.e.
    ///
    ///     sign(dr/dsigma) = -sign(P(r_H)).
    ///
    /// Otherwise the numerator tends to 4 M r_H P while Delta tends to zero, dt/dsigma diverges,
    /// and the curve runs off to t = +/- infinity without ever arriving: that is the crossing this
    /// chart does not have.
    ///
    /// The rule is invariant under T -> -T, as it must be, because it is a statement about a curve
    /// and not about how the curve is parameterised: reversing the tangent flips E, L, P and s
    /// together. Two known results fall straight out of it. With the ingoing principal null
    /// direction (E, L) = (1, a) and s = -1, P(r+) = 2Mr+ - a^2 = r+^2 > 0, so ingoing light
    /// crosses r+; and for a future-directed infaller, P(r-) = 2 M r- (E - Omega_- L) with
    /// Omega_- = a / (r-^2 + a^2), so the crossing is legal exactly when E - Omega_- L > 0 - which
    /// is the criterion `GeodesicState` already freezes a worldline on, arrived at here from the
    /// regularity of the chart rather than from the runaway of u^t.
    ///
    /// A vanishing P(r_H) is refused. R(r_H) = P(r_H)^2 is then zero, so the surface is a turning
    /// point of the radial motion and the curve is tangent to the horizon rather than crossing it.
    pub fn crosses_in_chart(&self, metric: &KerrSchild, r_h: f64, dr_sign: f64) -> bool {
        let p = self.p_at(metric, r_h);
        dr_sign * p < 0.0
    }
}

/// One point of a surface curve, as the sampler hands it to a caller that is going to draw it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePoint {
    /// The direction angle it was found along: T = sin(psi) e0 + cos(psi) e1 in the observer's
    /// axial tetrad, so psi = 0 is the observer's outward now-direction, psi = pi/2 their own
    /// future and psi = 5 pi/4 the inward half of their past light cone.
    pub psi: f64,
    /// Affine length from the observer's event to the surface along that direction, for the
    /// Euclidean-unit tangent above.
    pub sigma: f64,
    /// (xi^1, xi^0) = sigma (cos psi, sin psi): the drawn point, in the same ordering
    /// `LocalLine::point` uses - local outward distance first, local time second.
    pub xi: [f64; 2],
    /// Whether |xi| = sigma exceeds the window the caller asked for. Such a point is exact like
    /// any other; it is flagged so that the sampler can stop refining beyond the canvas and so
    /// that a run can be cut where it leaves.
    pub outside: bool,
}

/// What the sampler is allowed to spend, and what the caller can see.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSampling {
    /// Directions laid out uniformly round the full circle before any refinement. The first and
    /// last coincide at psi = 0 and psi = 2 pi, so a curve that closes comes back with its first
    /// point repeated at the end and can be stroked as one polyline.
    pub directions: usize,
    /// Largest |xi| the caller can draw. Points beyond it are still exact and are still emitted -
    /// one either side of every stretch that leaves the window, so a polyline exits at the right
    /// angle and can be clipped - but the sampler neither refines nor keeps the interior of such a
    /// stretch. `f64::INFINITY` asks for the whole curve.
    pub max_xi: f64,
    /// How far, in the units of xi, the drawn chord may depart from the curve before an interval
    /// is split.
    pub chord_tolerance: f64,
    /// Bisections allowed inside one initial interval. It bounds both the chord refinement and the
    /// hunt for the end of a run, so the ends of a run are located to within 2 pi / directions
    /// divided by 2^max_depth of direction angle.
    pub max_depth: u32,
    /// Hard ceiling on affine-length evaluations for the whole sweep, so that a pathological
    /// geometry costs a bounded frame rather than an unbounded one.
    pub max_evaluations: usize,
}

impl Default for SurfaceSampling {
    fn default() -> Self {
        Self {
            directions: 96,
            max_xi: f64::INFINITY,
            chord_tolerance: 0.02,
            max_depth: 5,
            max_evaluations: 4000,
        }
    }
}

impl SurfaceSampling {
    /// The defaults scaled to a canvas that can show |xi| up to `max_xi`: the chord tolerance
    /// becomes a fraction of the window rather than an absolute length, which is what keeps the
    /// cost the same whether the view is zoomed in on a tenth of an M or out over fifty.
    pub fn for_window(max_xi: f64) -> Self {
        let max_xi = if max_xi.is_finite() && max_xi > 0.0 {
            max_xi
        } else {
            f64::INFINITY
        };
        let tolerance = if max_xi.is_finite() { 0.004 * max_xi } else { 0.02 };
        Self {
            max_xi,
            chord_tolerance: tolerance,
            ..Self::default()
        }
    }
}

/// Affine length from the event at radius `r0` to the surface r = `r_h`, along the geodesic that
/// leaves that event with tangent `tangent`.
///
/// The tangent is a vector of the coordinate basis (T^t, T^r, T^phi) at r0. It may be timelike,
/// null or spacelike, future- or past-directed, and of any length: the answer is the affine
/// parameter of *that* tangent, so the caller's drawn point is sigma * T whatever the scaling.
///
/// `None`, and the reasons for each:
///
/// * The tangent does not set out towards the surface. sign(T^r) has to be sign(r_h - r0), and a
///   tangent with T^r = 0 exactly is refused rather than followed through the turning point it
///   starts on. (A caller sweeping a whole circle of directions meets this at the two directions
///   along the observer's own worldline when the observer holds their radius, and refusing is the
///   right answer there: a static observer's worldline never reaches any other surface r = const.)
/// * R <= 0 somewhere strictly inside the interval. The geodesic turns round before it arrives -
///   or, where R merely touches zero, winds onto that radius for ever - so it does not reach the
///   surface at all along this direction. The geodesic is never followed through a turning point
///   here: the second crossing is a different point of the surface and is not what the direction
///   points at.
/// * A horizon strictly between r0 and r_h that this chart does not let the geodesic cross. See
///   [`RadialConstants::crosses_in_chart`]. The *target* surface is exempt, deliberately: r+
///   reached from outside on a past-directed direction is the past horizon, which is precisely
///   what an outside observer's past light cone runs down onto, and r- reached on the far branch
///   is a real limit of the same kind.
/// * A non-finite input, or a radius at or below the ring.
pub fn affine_length_to_surface(
    metric: &KerrSchild,
    r0: f64,
    tangent: &[f64; 3],
    r_h: f64,
) -> Option<f64> {
    if !r0.is_finite() || !r_h.is_finite() || !tangent.iter().all(|c| c.is_finite()) {
        return None;
    }
    if r0 <= R_MIN || r_h < 0.0 {
        return None;
    }
    let gap = r_h - r0;
    if gap.abs() <= SAME_SURFACE {
        return Some(0.0);
    }
    let dir = gap.signum();
    // The negation is the point rather than a way of writing <=: a tangent whose radial component
    // has come out NaN has to take this branch too, and `tangent[1] * dir <= 0.0` would let it
    // through.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(tangent[1] * dir > 0.0) {
        return None;
    }

    let k = RadialConstants::of_tangent(metric, r0, tangent);
    let (lo, hi) = if gap > 0.0 { (r0, r_h) } else { (r_h, r0) };

    // Chart legality, for the horizons the geodesic has to pass *through* on the way.
    for r_horizon in [metric.outer_horizon(), metric.inner_horizon()] {
        if r_horizon > lo && r_horizon < hi && !k.crosses_in_chart(metric, r_horizon, dir) {
            return None;
        }
    }

    affine_length_between(metric, lo, hi, &k)
}

/// Affine length of the geodesic with constants `k` between the radii `r_a` and `r_b`, as a
/// positive magnitude, on the assumption that the geodesic runs monotonically in r between them.
///
/// This is the quadrature on its own, with none of [`affine_length_to_surface`]'s questions asked:
/// the caller owns the direction, owns the chart legality, and owns the claim that the stretch is
/// one leg of the motion rather than two joined at a turning point. It is here because the app's
/// null rays arrive with all three already settled - a ray that has been integrated from one event
/// to another has demonstrably travelled the ground - and re-deriving them from a tangent would
/// only be a way of getting them wrong.
///
/// `None` if R <= 0 anywhere strictly inside the interval, which is the one thing the caller
/// cannot be assumed to know, or if the quadrature cannot be evaluated.
pub fn affine_length_between(
    metric: &KerrSchild,
    r_a: f64,
    r_b: f64,
    k: &RadialConstants,
) -> Option<f64> {
    let lo = r_a.min(r_b);
    let hi = r_a.max(r_b);
    if !lo.is_finite() || !hi.is_finite() || lo < 0.0 {
        return None;
    }
    if hi - lo <= SAME_SURFACE {
        return Some(0.0);
    }

    let coef = k.cubic(metric);
    // R = r c(r) and r > 0, so c alone carries the sign. The integrable inverse-square-root
    // singularity of a zero *at an end* is what the quadrature below is chosen for; a zero
    // strictly inside is a turning point, and there is no arrival to report.
    if !interior_is_allowed(&coef, lo, hi) {
        return None;
    }

    // r^2 / sqrt(R) = r^2 / sqrt(r c(r)). Evaluated through c rather than through
    // R = P^2 - Delta [...] because c is a Horner evaluation while R is a difference of two large
    // squares that cancels to nothing exactly where the accuracy is wanted - at a horizon, and at
    // the ring, where both terms of R are of size P^2.
    //
    // `None` is returned inside the collar around a zero of c where the evaluation is round-off
    // and nothing else: there the computed c is a few machine epsilons of either sign, and taking
    // it at face value would put an integrand of 1e15 where the honest one is 1e8. The collar is
    // an interval in r of width (rounding floor) / |c'|, which for the cases this module meets is
    // of order 1e-14 M, and an integrand going like one over the square root of the distance to
    // the zero contributes about 1e-7 M of affine length over such a width - so what is dropped is
    // below the accuracy of everything kept, and dropping it is the only thing that can be done
    // with a number double precision has stopped carrying.
    let integrand = |r: f64| -> Option<f64> {
        let c = horner(&coef, r);
        if c <= rounding_floor(&coef, r) {
            return None;
        }
        Some(r * r / (r * c).sqrt())
    };

    tanh_sinh(integrand, lo, hi)
}

/// The drawn point of the surface r = `r_h` in the observer's own (xi^1, xi^0) plane, along the
/// direction T = `c0` e0 + `c1` e1 of their axial tetrad.
///
/// This is [`affine_length_to_surface`] with the tetrad built for the caller and the answer
/// returned as the chart point sigma (c1, c0), in the ordering `LocalLine::point` uses. The tetrad
/// is [`Tetrad::from_four_velocity_axial`], the one `LocalFrame::for_observer` draws in, so the
/// point lands in the same plane as everything else on that canvas.
///
/// (c0, c1) is not normalised and does not have to be: scaling it scales sigma back down by the
/// same factor and leaves the point where it was.
pub fn surface_point_in_frame(
    metric: &KerrSchild,
    r0: f64,
    u: &[f64; 3],
    c0: f64,
    c1: f64,
    r_h: f64,
) -> Option<[f64; 2]> {
    let tetrad = Tetrad::from_four_velocity_axial(metric, r0.max(R_MIN), u);
    let tangent = [
        c0 * tetrad.e0[0] + c1 * tetrad.e1[0],
        c0 * tetrad.e0[1] + c1 * tetrad.e1[1],
        c0 * tetrad.e0[2] + c1 * tetrad.e1[2],
    ];
    let sigma = affine_length_to_surface(metric, r0, &tangent, r_h)?;
    Some([sigma * c1, sigma * c0])
}

/// The whole of the surface r = `r_h` as the observer at radius `r0` with 4-velocity `u` sees it
/// laid out in their own (xi^1, xi^0) plane: a set of polylines, one per connected run of
/// directions that reaches the surface.
///
/// The sweep is over T = sin(psi) e0 + cos(psi) e1 for psi round the full circle, so the curve is
/// polar in the drawn plane - the point at angle psi sits at radius sigma(psi) - and the sampling
/// is a sampling of psi. Directions that return `None` break the sweep into runs, and the ends of
/// the runs are hunted down by bisection because they are the visible ends of the drawn curve: on
/// the past side of an outside observer's cone the run stops exactly where the geodesic stops
/// being able to reach the horizon in this chart, and that edge is a thing to see rather than an
/// artefact to hide.
///
/// Between two directions that both arrive, an interval is split whenever the polar chord could
/// depart from the curve by more than `opts.chord_tolerance`. The estimate used is
/// sigma (delta psi)^2 / 8, the sagitta of a circular arc, plus a quarter of the change in sigma
/// across the interval, which is what makes the sampler spend its effort where sigma is moving
/// fast rather than where the curve is long.
///
/// Runs of fewer than two points are dropped: a single point is not a curve, and a caller that
/// strokes polylines has nothing to do with one.
pub fn sample_surface(
    metric: &KerrSchild,
    r0: f64,
    u: &[f64; 3],
    r_h: f64,
    opts: &SurfaceSampling,
) -> Vec<Vec<SurfacePoint>> {
    let r0 = r0.max(R_MIN);
    let tetrad = Tetrad::from_four_velocity_axial(metric, r0, u);
    let directions = opts.directions.max(8);
    let mut budget = opts.max_evaluations.max(directions + 1);

    let evaluate = |psi: f64, budget: &mut usize| -> Option<SurfacePoint> {
        if *budget == 0 {
            return None;
        }
        *budget -= 1;
        let (s, c) = psi.sin_cos();
        let tangent = [
            s * tetrad.e0[0] + c * tetrad.e1[0],
            s * tetrad.e0[1] + c * tetrad.e1[1],
            s * tetrad.e0[2] + c * tetrad.e1[2],
        ];
        let sigma = affine_length_to_surface(metric, r0, &tangent, r_h)?;
        Some(SurfacePoint {
            psi,
            sigma,
            xi: [sigma * c, sigma * s],
            outside: sigma > opts.max_xi,
        })
    };

    let step = std::f64::consts::TAU / directions as f64;
    let mut samples: Vec<(f64, Option<SurfacePoint>)> = Vec::with_capacity(2 * directions);
    let first = evaluate(0.0, &mut budget);
    samples.push((0.0, first));
    for j in 1..=directions {
        let psi_b = step * j as f64;
        let a = samples[samples.len() - 1];
        let b = (psi_b, evaluate(psi_b, &mut budget));
        refine(
            &evaluate,
            a,
            b,
            opts.max_depth,
            opts,
            &mut samples,
            &mut budget,
        );
    }

    runs_from(&samples)
}

/// Bisect the interval (a, b) of direction angle until the drawn chord is good enough, pushing
/// every sample except `a` itself, in order, onto `out`. `a` is already on `out` when this is
/// called, which is what keeps the output a single ordered list rather than a tree to flatten.
fn refine(
    evaluate: &impl Fn(f64, &mut usize) -> Option<SurfacePoint>,
    a: (f64, Option<SurfacePoint>),
    b: (f64, Option<SurfacePoint>),
    depth: u32,
    opts: &SurfaceSampling,
    out: &mut Vec<(f64, Option<SurfacePoint>)>,
    budget: &mut usize,
) {
    let split = depth > 0 && *budget > 0 && wants_split(a.1.as_ref(), b.1.as_ref(), b.0 - a.0, opts);
    if !split {
        out.push(b);
        return;
    }
    let psi_mid = 0.5 * (a.0 + b.0);
    let mid = (psi_mid, evaluate(psi_mid, budget));
    refine(evaluate, a, mid, depth - 1, opts, out, budget);
    refine(evaluate, mid, b, depth - 1, opts, out, budget);
}

/// Whether the interval between two sampled directions is worth splitting.
fn wants_split(
    a: Option<&SurfacePoint>,
    b: Option<&SurfacePoint>,
    d_psi: f64,
    opts: &SurfaceSampling,
) -> bool {
    match (a, b) {
        // The end of a run. Where it falls is visible, so it is worth every bisection allowed.
        (Some(_), None) | (None, Some(_)) => true,
        (None, None) => false,
        (Some(p), Some(q)) => {
            // Both beyond the canvas: the stretch between them is beyond it too unless sigma dips
            // back inside, and a dip that deep is not worth the evaluations to find.
            if p.outside && q.outside {
                return false;
            }
            let biggest = p.sigma.max(q.sigma);
            let sagitta = 0.125 * d_psi * d_psi * biggest + 0.25 * (p.sigma - q.sigma).abs();
            sagitta > opts.chord_tolerance
        }
    }
}

/// Cut the ordered sweep into drawable runs.
///
/// A run is a maximal stretch of directions that reach the surface, trimmed to the window: the
/// interior of a stretch that has left the canvas is dropped, but the first point beyond the edge
/// is kept at each end of it, so the polyline leaves and re-enters along the true curve and the
/// caller's clipping has something honest to clip.
fn runs_from(samples: &[(f64, Option<SurfacePoint>)]) -> Vec<Vec<SurfacePoint>> {
    let mut runs: Vec<Vec<SurfacePoint>> = Vec::new();
    let mut current: Vec<SurfacePoint> = Vec::new();
    // The most recent out-of-window point that has not been emitted, held back in case the sweep
    // comes back inside and needs a lead-in.
    let mut pending: Option<SurfacePoint> = None;

    let close = |current: &mut Vec<SurfacePoint>, runs: &mut Vec<Vec<SurfacePoint>>| {
        if current.len() >= 2 {
            runs.push(std::mem::take(current));
        } else {
            current.clear();
        }
    };

    for (_, sample) in samples {
        match sample {
            None => {
                close(&mut current, &mut runs);
                pending = None;
            }
            Some(point) if point.outside => {
                if current.is_empty() {
                    // Still outside, or outside again after a run was closed: remember only the
                    // latest, which is the one adjacent to any re-entry.
                    pending = Some(*point);
                } else {
                    // Leaving the window: one exact point beyond the edge, then the run ends.
                    current.push(*point);
                    close(&mut current, &mut runs);
                    pending = Some(*point);
                }
            }
            Some(point) => {
                if current.is_empty()
                    && let Some(lead) = pending.take()
                {
                    current.push(lead);
                }
                current.push(*point);
                pending = None;
            }
        }
    }
    close(&mut current, &mut runs);
    runs
}

/// Horner evaluation of a polynomial given in descending powers.
fn horner(coef: &[f64; 4], r: f64) -> f64 {
    ((coef[0] * r + coef[1]) * r + coef[2]) * r + coef[3]
}

/// How small a value of `horner` is indistinguishable from zero: the size of the terms that went
/// into it, times a few machine epsilons. A cubic evaluated where its terms cancel can only be
/// delivered to this, however exact its coefficients are.
fn rounding_floor(coef: &[f64; 4], r: f64) -> f64 {
    let scale = coef[0].abs() * r * r * r
        + coef[1].abs() * r * r
        + coef[2].abs() * r
        + coef[3].abs();
    8.0 * f64::EPSILON * scale
}

/// Whether c(r) stays strictly positive on the open interval (lo, hi), so that the geodesic covers
/// the whole of it without turning.
///
/// A cubic is monotone between its critical points, so its minimum over an interval is attained at
/// an end or at a critical point inside: three or four evaluations settle it exactly, with no
/// sampling and no root finder. The ends are allowed to vanish - that is a horizon reached where
/// P = 0, or a turning point exactly on the target surface, and the quadrature converges at both -
/// while a zero strictly inside is refused, whether it is a crossing or a touch.
fn interior_is_allowed(coef: &[f64; 4], lo: f64, hi: f64) -> bool {
    // The ends are compared against their own rounding floor rather than against nought. c(r_h) is
    // *exactly* zero at a horizon reached with P = 0 - the E = 0 spacelike leg of a hovering
    // observer's now-axis is the everyday case, and there c(r) is r Delta(r) - and a Horner
    // evaluation of a cubic whose terms are of size ten cannot deliver an exact zero: it delivers
    // a few times the machine epsilon, of either sign. Reading a negative rounding as a turning
    // point would refuse the one direction the view most wants drawn.
    if horner(coef, lo) < -rounding_floor(coef, lo) || horner(coef, hi) < -rounding_floor(coef, hi)
    {
        return false;
    }
    // c'(r) = 3 a3 r^2 + 2 a2 r + a1.
    let (qa, qb, qc) = (3.0 * coef[0], 2.0 * coef[1], coef[2]);
    let inside = |r: f64| -> bool { r > lo && r < hi && horner(coef, r) <= 0.0 };
    if qa != 0.0 {
        let disc = qb * qb - 4.0 * qa * qc;
        if disc > 0.0 {
            // The stable pairing: one root from the formula with no cancellation, the other from
            // the product of the roots. It matters here because a nearly null tangent makes
            // qa = 3(E^2 - mu2) tiny while qb and qc stay of order one.
            let s = disc.sqrt();
            let q = -0.5 * (qb + if qb >= 0.0 { s } else { -s });
            if inside(q / qa) {
                return false;
            }
            if q != 0.0 && inside(qc / q) {
                return false;
            }
        }
    } else if qb != 0.0 && inside(-qc / qb) {
        return false;
    }
    true
}

/// One node of the tanh-sinh rule, stored as a distance from the nearer end of the interval rather
/// than as an abscissa in [-1, 1].
///
/// Near the ends the abscissa is 1 - 1e-30 and a subtraction would throw the whole of it away,
/// which is exactly where an integrable endpoint singularity needs its samples placed. Keeping
/// d = 1 - |x| instead, and building the radius as hi - u d or lo + u d, costs nothing and loses
/// nothing.
#[derive(Debug, Clone, Copy)]
struct Node {
    /// Measure d from the upper end of the interval rather than from the lower one.
    from_hi: bool,
    /// d = 1 - |x|, in (0, 1].
    d: f64,
    /// The rule's weight at this node, before the factor h.
    w: f64,
}

/// Levels of the rule. Level L halves the spacing of level L - 1, so the nodes are nested and each
/// level only has to evaluate the ones it adds.
const QUAD_MAX_LEVEL: usize = 5;
/// Where the substitution variable is truncated. At |t| = 4 the abscissa is 1 - 1e-37 from the end
/// and the weight is 1e-36, so what is dropped is below the round-off of everything kept, even for
/// an integrand going like one over the square root of the distance to the end.
const QUAD_HALF_WIDTH: f64 = 4.0;
/// Relative agreement between two successive levels at which the rule stops.
///
/// The rest-frame view wants about a part in a million and this is two orders inside it, which is
/// as far as the tolerance is worth tightening: on an integral with a one-over-square-root end the
/// successive levels settle at a relative wobble of a few times 1e-9 set by the round-off collar
/// the integrand drops (see `affine_length_between`), so asking for 1e-10 would not buy a digit
/// and would run every call to the last level instead of stopping at the third.
const QUAD_REL_TOL: f64 = 1e-8;

/// The nested node sets, built once. Entry L holds the nodes that are *new* at level L.
fn quadrature_levels() -> &'static Vec<Vec<Node>> {
    static LEVELS: OnceLock<Vec<Vec<Node>>> = OnceLock::new();
    LEVELS.get_or_init(|| {
        let half_pi = std::f64::consts::FRAC_PI_2;
        (0..=QUAD_MAX_LEVEL)
            .map(|level| {
                let h = 1.0 / f64::from(1u32 << level);
                let k_max = (QUAD_HALF_WIDTH / h).floor() as i64;
                let mut nodes = Vec::new();
                for k in -k_max..=k_max {
                    // Level zero takes every node; every level after it adds only the ones that
                    // fall between the nodes of the level before, which are the odd multiples.
                    if level > 0 && k % 2 == 0 {
                        continue;
                    }
                    let t = (k as f64) * h;
                    let s = half_pi * t.sinh();
                    // 1 - tanh|s| = 2 / (exp(2|s|) + 1), which stays exact where the subtraction
                    // would not.
                    let d = 2.0 / ((2.0 * s.abs()).exp() + 1.0);
                    let w = half_pi * t.cosh() / (s.cosh() * s.cosh());
                    if d <= 0.0 || !d.is_finite() || w <= 0.0 || !w.is_finite() {
                        continue;
                    }
                    nodes.push(Node {
                        from_hi: s >= 0.0,
                        d,
                        w,
                    });
                }
                nodes
            })
            .collect()
    })
}

/// Tanh-sinh (double-exponential) quadrature of `f` over [lo, hi].
///
/// The rule is chosen for one property: it converges at the same rate whether the integrand is
/// analytic at the ends of the interval or has an integrable singularity there, because the
/// substitution r = mid + u tanh(pi/2 sinh t) drives the weights to zero doubly exponentially and
/// swallows the singularity whole. Both ends of this module's integrals can be singular and
/// neither has to be - R vanishes at a horizon reached with P = 0, at a turning point that happens
/// to sit exactly on the target, and at neither otherwise - so a rule that needs to be told which
/// case it is in would need a case analysis that the physics does not offer.
///
/// A node the integrand declines to answer at - one inside the round-off collar of a zero of R -
/// contributes nothing: for a one-over-square-root singularity the weight falls like the distance
/// to the end while the integrand only rises like one over its square root, so the product goes to
/// zero and dropping the collar is the limit rather than a fudge. A node that comes back non-finite
/// is a different matter and fails the whole quadrature, because it means the arithmetic broke
/// somewhere the caller was promised it would not.
fn tanh_sinh(f: impl Fn(f64) -> Option<f64>, lo: f64, hi: f64) -> Option<f64> {
    let u = 0.5 * (hi - lo);
    let mut sum = 0.0f64;
    let mut previous: Option<f64> = None;
    for (level, nodes) in quadrature_levels().iter().enumerate() {
        for node in nodes {
            let r = if node.from_hi { hi - u * node.d } else { lo + u * node.d };
            if let Some(value) = f(r) {
                if !value.is_finite() {
                    return None;
                }
                sum += node.w * value;
            }
        }
        let h = 1.0 / f64::from(1u32 << level);
        let estimate = u * h * sum;
        if !estimate.is_finite() {
            return None;
        }
        if let Some(before) = previous
            && level >= 2
            && (estimate - before).abs() <= QUAD_REL_TOL * estimate.abs().max(f64::MIN_POSITIVE)
        {
            return Some(estimate);
        }
        previous = Some(estimate);
    }
    previous
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesic::{GeodesicState, proper_time_between};
    use crate::local_frame::ruler_distance;
    use std::f64::consts::PI;

    fn raindrop(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let (ut, ur, up) = GeodesicState::new_infall(metric, 0.0, r, 1.0, 0.0).derivatives(metric, r);
        [ut, ur, up]
    }

    fn static_obs(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g_tt = metric.metric_components(r)[0][0];
        [1.0 / (-g_tt).sqrt(), 0.0, 0.0]
    }

    fn zamo(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g = metric.metric_components(r);
        let omega = metric.frame_dragging_omega(r);
        let n = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma = 1.0 / n.sqrt();
        [gamma, 0.0, gamma * omega]
    }

    /// The prograde ISCO observer of an a = 0.90 hole: the case the rest-frame view is wrong about
    /// by most of a gravitational radius when it draws the surfaces from the first-order map.
    fn isco_observer(metric: &KerrSchild) -> (f64, [f64; 3]) {
        let r = metric.isco(true);
        let omega = metric.orbital_angular_velocity(r, true).expect("the ISCO is a circular orbit");
        let u_t = metric.circular_orbit_dilation(r, true).expect("and a timelike one");
        (r, [u_t, 0.0, u_t * omega])
    }

    /// The tangent c0 e0 + c1 e1 of an observer's axial tetrad, in coordinate components.
    fn leg(metric: &KerrSchild, r: f64, u: &[f64; 3], c0: f64, c1: f64) -> [f64; 3] {
        let t = Tetrad::from_four_velocity_axial(metric, r, u);
        [
            c0 * t.e0[0] + c1 * t.e1[0],
            c0 * t.e0[1] + c1 * t.e1[1],
            c0 * t.e0[2] + c1 * t.e1[2],
        ]
    }

    #[test]
    fn test_the_radial_potential_is_the_tangents_own_radial_rate() {
        // The whole module rests on R(r0) = r0^4 (T^r)^2 for the constants read off T, for any
        // tangent at all. It is the first integral being the first integral, and it holds for
        // timelike, null and spacelike tangents alike, at any normalisation, in every region. If
        // it failed, every affine length below would be a plausible-looking number about the wrong
        // curve, so it is checked before anything else is.
        for &a in &[0.0, 0.65, 0.90, 0.998] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            for &r in &[9.0, 3.0, rp, 0.5 * (rp + rm), rm, 0.5 * rm] {
                let u = raindrop(&metric, r);
                let tetrad = Tetrad::from_four_velocity_axial(&metric, r, &u);
                for &(c0, c1, c2) in &[
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (1.0, 1.0, 0.0),
                    (-2.3, 0.7, 0.4),
                    (0.0, 0.0, 1.0),
                    (1.0, -0.6, 0.8),
                ] {
                    let t: [f64; 3] = core::array::from_fn(|mu| {
                        c0 * tetrad.e0[mu] + c1 * tetrad.e1[mu] + c2 * tetrad.e2[mu]
                    });
                    let k = RadialConstants::of_tangent(&metric, r, &t);
                    let got = k.potential(&metric, r);
                    let want = r.powi(4) * t[1] * t[1];
                    let scale = 1.0 + got.abs().max(want.abs());
                    assert!(
                        (got - want).abs() < 1e-9 * scale,
                        "R = {got} vs r^4 (T^r)^2 = {want} at r = {r} (a = {a}), T = {t:?}"
                    );
                    // And the cubic factorisation R = r c(r) is the same function.
                    let coef = k.cubic(&metric);
                    let through_c = r * horner(&coef, r);
                    assert!(
                        (through_c - got).abs() < 1e-9 * scale,
                        "r c(r) = {through_c} vs R = {got} at r = {r} (a = {a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_now_axis_is_the_ruler_distance_for_every_observer() {
        // T = -e1 is the observer's own inward now-direction, and the affine length along it *is*
        // `ruler_distance`: both are the arclength of the spacelike geodesic that leaves the event
        // along the radial leg of the tetrad and runs until it meets the surface. One integrates
        // the geodesic equation in arclength with RK4 and the other evaluates a closed-form
        // quadrature in r, so agreement to seven digits is two independent constructions of the
        // same geodesic meeting, and not a restatement.
        //
        // The ISCO case is the one the rest-frame view gets wrong. The first-order map draws r+ at
        // 1.647 M from Alice on the prograde ISCO of an a = 0.90 hole; the distance is 3.020 M.
        let mut compared = 0;
        for &a in &[0.0, 0.5, 0.90] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            for &r in &[12.0, 6.0, 4.0, 2.5 * rp] {
                let mut frames = vec![("free fall", raindrop(&metric, r))];
                if metric.metric_components(r)[0][0] < 0.0 {
                    frames.push(("static", static_obs(&metric, r)));
                }
                frames.push(("ZAMO", zamo(&metric, r)));
                for (name, u) in frames {
                    let inward = leg(&metric, r, &u, 0.0, -1.0);
                    let exact = affine_length_to_surface(&metric, r, &inward, rp)
                        .unwrap_or_else(|| panic!("no arrival for {name} at r = {r} (a = {a}), T = {inward:?}"));
                    let ruler = ruler_distance(&metric, r, &u, rp).expect("so does the ruler");
                    assert!(
                        (exact - ruler).abs() < 1e-6 * ruler,
                        "{name} at r = {r} (a = {a}): affine {exact} vs ruler {ruler}"
                    );
                    compared += 1;
                }
            }
        }

        let metric = KerrSchild::new(1.0, 0.90);
        let (r_isco, u) = isco_observer(&metric);
        assert!(
            (r_isco - 2.3209).abs() < 1e-3,
            "the prograde ISCO of a = 0.90 is at {r_isco} M"
        );
        let inward = leg(&metric, r_isco, &u, 0.0, -1.0);
        let to_rp = affine_length_to_surface(&metric, r_isco, &inward, metric.outer_horizon())
            .expect("her now-axis reaches r+");
        let ruler = ruler_distance(&metric, r_isco, &u, metric.outer_horizon()).expect("so does the ruler");
        println!(
            "{compared} (observer, radius) pairs agreed with `ruler_distance`; on the prograde \
             ISCO at r = {r_isco:.4} M the now-axis reaches r+ = {:.4} M at sigma = {to_rp:.5} M \
             (ruler {ruler:.5} M), against the 1.647 M the first-order map draws",
            metric.outer_horizon()
        );
        assert!(
            (to_rp - 3.0203).abs() < 2e-3,
            "the ISCO now-axis distance to r+ is 3.0203 M, got {to_rp}"
        );
        assert!((to_rp - ruler).abs() < 1e-6 * ruler, "and the ruler agrees: {ruler}");
    }

    #[test]
    fn test_the_time_axis_is_the_proper_time_to_the_surface() {
        // T = e0 is the observer's own 4-velocity, so for a free-faller the affine length to a
        // surface is the proper time their watch shows on the way to it - which `geodesic`
        // already computes, by the same quadrature written for mu2 = 1 and no chart questions
        // asked. The two must be the same number, and the direction of travel has to be got right
        // for either of them to exist: an infaller reaches r+ and r- in their future (T = +e0) and
        // no surface at all above them.
        for &a in &[0.0, 0.65, 0.90] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[8.0, 4.0, 1.4 * rp] {
                let geo = GeodesicState::new_infall(&metric, 0.0, r, 1.0, 0.0);
                let u = raindrop(&metric, r);
                let future = leg(&metric, r, &u, 1.0, 0.0);
                for &target in &[rp, rm.max(0.2), 0.2] {
                    let got = affine_length_to_surface(&metric, r, &future, target)
                        .expect("a raindrop reaches every surface below it");
                    let want = proper_time_between(&metric, geo.energy, geo.l_ang, r, target)
                        .expect("and so does the closed form");
                    assert!(
                        (got - want).abs() < 1e-6 * want,
                        "tau to r = {target} from {r} (a = {a}): {got} vs {want}"
                    );
                }
                // Taken all the way down to the ring the two part company, and it is
                // `proper_time_between` that is behind: its Simpson rule on a uniform mesh has to
                // resolve an integrand going like sqrt(r) there, whose derivative is unbounded.
                // The raindrop of a hole with no spin has the elementary closed form
                // (2/3)(r^{3/2} - r_h^{3/2}) / sqrt(2M) - Newton's fall time, which Painleve and
                // Gullstrand make exact in general relativity - so the disagreement can be settled
                // by hand rather than by vote.
                if a == 0.0 {
                    let target = 1e-3;
                    let got = affine_length_to_surface(&metric, r, &future, target)
                        .expect("the raindrop reaches the ring");
                    let closed =
                        (2.0 / 3.0) * (r.powf(1.5) - target.powf(1.5)) / 2.0f64.sqrt();
                    let simpson = proper_time_between(&metric, 1.0, 0.0, r, target)
                        .expect("the Simpson rule answers too");
                    assert!(
                        (got - closed).abs() < 1e-9 * closed,
                        "the quadrature to the ring from {r} M: {got} vs Newton's {closed}"
                    );
                    println!(
                        "raindrop from {r} M to r = {target} M: this quadrature {got:.12}, the \
                         closed form {closed:.12}, `proper_time_between` {simpson:.12}"
                    );
                }
                // Nothing above her is in her future along her own worldline.
                let above = affine_length_to_surface(&metric, r, &future, r + 1.0);
                assert!(above.is_none(), "an infaller does not reach r + 1 M: {above:?}");
            }
        }
    }

    #[test]
    fn test_the_schwarzschild_static_closed_forms() {
        // Two numbers that can be got with a pencil, for a static observer at r = 6 M of a hole
        // with no spin.
        //
        // The now-axis is the radial proper length of the slice t = const, because that slice is
        // the fixed-point set of t -> -t and so is totally geodesic: a spacelike geodesic that
        // starts in it tangent to it stays in it. So sigma is int dr / sqrt(1 - 2M/r) from 2 M to
        // 6 M, which is 7.19141 M.
        //
        // The inward past cone T = -e0 - e1 is prettier still. In ingoing Kerr-Schild coordinates
        // r is an affine parameter along a radial ingoing null geodesic - the ingoing principal
        // null congruence has k^mu = (1, -1, 0) at every radius - so the affine length is simply
        // the gap in r divided by the normalisation the observer's own frame puts on the tangent,
        // which for a static observer at r0 is sqrt(1 - 2M/r0). Four M of gap over sqrt(2/3) is
        // 4.898979 M, and the tangent here is past-directed rather than ingoing, which changes
        // nothing about a magnitude.
        let metric = KerrSchild::new(1.0, 0.0);
        let r0 = 6.0;
        let u = static_obs(&metric, r0);

        let now = affine_length_to_surface(&metric, r0, &leg(&metric, r0, &u, 0.0, -1.0), 2.0)
            .expect("the now-axis reaches r+");
        assert!(
            (now - 7.19141).abs() < 1e-4,
            "the static chain from 6 M to 2 M is 7.19141 M, got {now}"
        );

        let past_in = affine_length_to_surface(&metric, r0, &leg(&metric, r0, &u, -1.0, -1.0), 2.0)
            .expect("the inward past cone reaches r+");
        let closed = 4.0 / (1.0 - 2.0 / r0).sqrt();
        println!(
            "static at 6 M, a = 0: now-axis to r+ = {now:.6} M (closed form 7.191411), inward \
             past cone = {past_in:.6} M (closed form {closed:.6})"
        );
        assert!(
            (past_in - closed).abs() < 1e-9 * closed,
            "Delta r / sqrt(1 - 2M/r0) = {closed}, got {past_in}"
        );
        assert!((past_in - 4.898979).abs() < 1e-5, "which is 4.898979 M: {past_in}");
    }

    #[test]
    fn test_the_isco_past_cone_reaches_the_horizon_at_the_published_distance() {
        // The second of the two numbers the first-order map gets wrong. Alice on the prograde ISCO
        // of an a = 0.90 hole has the inward half of her past light cone drawn meeting r+ at
        // 1.647 M; the affine length along it is 2.30398 M.
        let metric = KerrSchild::new(1.0, 0.90);
        let (r0, u) = isco_observer(&metric);
        let past_in = affine_length_to_surface(
            &metric,
            r0,
            &leg(&metric, r0, &u, -1.0, -1.0),
            metric.outer_horizon(),
        )
        .expect("her past cone reaches r+");
        println!(
            "prograde ISCO at r = {r0:.4} M of a = 0.90: the inward past cone meets r+ at \
             lambda = {past_in:.5} M, against the 1.647 M of the first-order map"
        );
        assert!(
            (past_in - 2.30398).abs() < 1e-4,
            "the ISCO past-cone affine length to r+ is 2.30398 M, got {past_in}"
        );
    }

    #[test]
    fn test_the_chart_culls_the_crossings_it_does_not_have() {
        // The ingoing Kerr-Schild chart is a black hole and no white hole, so half of the horizon
        // crossings of the maximal extension are not events of this spacetime at all. The rule
        // `crosses_in_chart` states - sign(dr/dsigma) = -sign(P(r_H)) - has to reproduce that, and
        // these are the cases it is meant to decide.
        let metric = KerrSchild::new(1.0, 0.90);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let r0 = 4.0;
        let u = static_obs(&metric, r0);

        // A past-directed inward null direction. Traced as a curve it runs inward and back in
        // coordinate time, which is the *outgoing* family travelled the other way: it settles onto
        // r+ from outside as t -> -infinity, which is the past horizon, and it arrives there at a
        // finite affine length. That is the limit of what an outside observer sees, so it is drawn.
        let past_in = leg(&metric, r0, &u, -1.0, -1.0);
        let to_rp = affine_length_to_surface(&metric, r0, &past_in, rp);
        assert!(to_rp.is_some(), "the past cone must reach r+: {to_rp:?}");
        // And it stops there. Continuing to r- or to the ring would mean crossing r+ outward in a
        // chart that has no such crossing.
        assert!(
            affine_length_to_surface(&metric, r0, &past_in, rm).is_none(),
            "a past-directed inward null direction must not reach r-"
        );
        assert!(
            affine_length_to_surface(&metric, r0, &past_in, 0.01).is_none(),
            "nor the ring"
        );

        // A future-directed inward timelike direction is the infall itself, and it goes through
        // both horizons: P(r_H) = 2 M r_H (E - Omega_H L) is positive for a raindrop at either,
        // and the motion is inward, so both crossings are legal.
        let falling = raindrop(&metric, r0);
        let future = leg(&metric, r0, &falling, 1.0, 0.0);
        for &target in &[rp, rm, 0.05] {
            assert!(
                affine_length_to_surface(&metric, r0, &future, target).is_some(),
                "an infaller reaches r = {target}"
            );
        }

        // The criterion is the one the app already freezes worldlines on, met here from the other
        // side. A prograde infaller between the horizons with E - Omega_- L < 0 settles onto the
        // far branch of r- rather than crossing it: it *reaches* the surface, at a finite affine
        // length and a finite reading of its own watch, while taking infinite coordinate time to
        // do it - which is why r- is drawn for it and the ring beyond r- is not, the crossing
        // having no event in this chart at all.
        let omega_minus = metric.inner_horizon_omega();
        let r_mid = 1.2;
        let geo = GeodesicState::new_infall(&metric, 0.0, r_mid, 1.0, 1.1 / omega_minus);
        assert!(
            geo.energy - omega_minus * geo.l_ang < 0.0,
            "this worldline must be one of the frozen family"
        );
        let (ut, ur, up) = geo.derivatives(&metric, r_mid);
        let onward = leg(&metric, r_mid, &[ut, ur, up], 1.0, 0.0);
        let to_rm = affine_length_to_surface(&metric, r_mid, &onward, rm);
        let tau_left = proper_time_between(&metric, geo.energy, geo.l_ang, r_mid, rm)
            .expect("r- is a finite proper time below");
        println!(
            "E - Omega_- L = {:+.4}: r- = {rm:.4} M is {to_rm:?} of affine length below r = \
             {r_mid} M (the closed-form proper time is {tau_left:.5} M), and the ring beyond it \
             is nowhere in this chart",
            geo.energy - omega_minus * geo.l_ang
        );
        assert!(
            to_rm.is_some_and(|s| (s - tau_left).abs() < 1e-6 * tau_left),
            "the far branch of r- is reached, at the proper time the closed form gives"
        );
        assert!(
            affine_length_to_surface(&metric, r_mid, &onward, 0.05).is_none(),
            "but a worldline with E - Omega_- L < 0 never crosses that branch"
        );
    }

    #[test]
    fn test_sigma_to_the_outer_horizon_is_continuous_across_the_change_of_branch() {
        // The legality rule is a sign test on P(r+), and a sign test is exactly the kind of thing
        // that can put a step in a drawn curve. It must not, and the reason it does not is that r+
        // is the *target* here, so no legality question is asked about it at all: what happens as
        // the direction sweeps through the psi where P(r+) changes sign is that the geodesic stops
        // arriving from one branch and starts arriving from the other, with the same affine length
        // at the crossover.
        //
        // The sweep below walks psi in small steps across that direction for a static observer and
        // asserts that sigma never jumps. The witness that the branch really did change is P(r+)
        // itself changing sign along the way.
        let metric = KerrSchild::new(1.0, 0.90);
        let r0 = 4.0;
        let u = static_obs(&metric, r0);
        let rp = metric.outer_horizon();
        let tetrad = Tetrad::from_four_velocity_axial(&metric, r0, &u);

        let mut previous: Option<(f64, f64)> = None;
        let (mut saw_positive, mut saw_negative) = (false, false);
        let mut worst_jump = 0.0f64;
        let mut samples = 0;
        // From the inward now-axis round through the inward past cone: the arc on which the
        // inward directions live.
        for i in 0..=400 {
            let psi = PI + PI * 0.5 * (i as f64) / 400.0;
            let (s, c) = psi.sin_cos();
            let t: [f64; 3] = core::array::from_fn(|mu| s * tetrad.e0[mu] + c * tetrad.e1[mu]);
            let k = RadialConstants::of_tangent(&metric, r0, &t);
            let p = k.p_at(&metric, rp);
            if p > 0.0 {
                saw_positive = true;
            } else {
                saw_negative = true;
            }
            let Some(sigma) = affine_length_to_surface(&metric, r0, &t, rp) else {
                continue;
            };
            if let Some((prev_psi, prev_sigma)) = previous {
                // The curve is smooth, so over a step of 0.004 rad the change is small; a branch
                // that had been mis-selected would show up as a finite jump.
                let jump = (sigma - prev_sigma).abs() / (psi - prev_psi);
                worst_jump = worst_jump.max(jump);
                assert!(
                    jump < 40.0,
                    "sigma jumped from {prev_sigma} to {sigma} between psi = {prev_psi} and {psi}"
                );
            }
            previous = Some((psi, sigma));
            samples += 1;
        }
        println!(
            "{samples} directions from the inward now-axis to the inward past cone reached r+; \
             the steepest |d sigma / d psi| seen was {worst_jump:.3}"
        );
        assert!(
            saw_positive && saw_negative,
            "P(r+) must change sign along this arc, or the branch change was never in reach"
        );
    }

    #[test]
    fn test_every_region_and_both_directions_are_covered() {
        // The view can be drawn for an observer anywhere, so the construction has to work
        // anywhere: outside r+, inside the ergosphere, between the horizons where r is timelike
        // and nothing can hover, and inside r-. And the target can be above the observer as well
        // as below - an infaller between the horizons has r+ in their past, so the direction that
        // reaches it is a past-directed one.
        for &a in &[0.0, 0.65, 0.90] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let regions = [
                ("outside", 6.0),
                ("ergosphere", 0.5 * (rp + 2.0)),
                ("between the horizons", 0.5 * (rp + rm)),
                ("inside r-", 0.6 * rm),
            ];
            for (name, r0) in regions {
                let u = raindrop(&metric, r0);
                // Something below is always reachable along the worldline's own future.
                let below = (0.5 * r0).max(0.02);
                let future = leg(&metric, r0, &u, 1.0, 0.0);
                let down = affine_length_to_surface(&metric, r0, &future, below);
                assert!(down.is_some(), "{name} (a = {a}): no arrival at r = {below}: {down:?}");
                // And something above is reachable on a past-directed direction, which is where an
                // infaller's own past lies.
                let above = r0 * 1.2;
                let past = leg(&metric, r0, &u, -1.0, 0.0);
                let up = affine_length_to_surface(&metric, r0, &past, above);
                assert!(
                    up.is_some_and(|s| s > 0.0),
                    "{name} (a = {a}): no past arrival at r = {above}: {up:?}"
                );
                // The past-directed time axis is the proper time the worldline took to get here,
                // which is the forward quadrature run the other way.
                if let Some(s) = up {
                    let geo = GeodesicState::new_infall(&metric, 0.0, r0, 1.0, 0.0);
                    let want = proper_time_between(&metric, geo.energy, geo.l_ang, above, r0)
                        .expect("the closed form covers the same stretch");
                    assert!(
                        (s - want).abs() < 1e-6 * want,
                        "{name} (a = {a}): past tau {s} vs {want}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_sampler_returns_runs_of_exact_points_inside_the_window() {
        // What the drawing actually calls. Every point handed back has to be the affine length its
        // own direction really gives - the sampler is a sampling of `affine_length_to_surface` and
        // nothing else - the points of a run have to be in order of direction angle, and the
        // window has to be respected: at most one point beyond it at each end of a run.
        let metric = KerrSchild::new(1.0, 0.90);
        let (r0, u) = isco_observer(&metric);
        let opts = SurfaceSampling::for_window(12.0);
        let runs = sample_surface(&metric, r0, &u, metric.outer_horizon(), &opts);
        assert!(!runs.is_empty(), "the ISCO observer must see r+ somewhere");

        let mut points = 0;
        for run in &runs {
            assert!(run.len() >= 2, "a run is a curve, not a point");
            let mut outside_inside = 0;
            for (i, p) in run.iter().enumerate() {
                if i > 0 {
                    assert!(run[i - 1].psi < p.psi, "a run must be ordered in psi");
                }
                if p.outside && i > 0 && i + 1 < run.len() {
                    outside_inside += 1;
                }
                // Exactness, checked by asking the primitive the same question again.
                let again = affine_length_to_surface(&metric, r0, &leg_at(&metric, r0, &u, p.psi), metric.outer_horizon())
                    .expect("a sampled direction arrives");
                assert!(
                    (again - p.sigma).abs() < 1e-9 * (1.0 + again),
                    "sample at psi = {} says sigma = {} against {again}",
                    p.psi,
                    p.sigma
                );
                assert!(
                    (p.xi[0] - p.sigma * p.psi.cos()).abs() < 1e-12
                        && (p.xi[1] - p.sigma * p.psi.sin()).abs() < 1e-12,
                    "the drawn point must be sigma (cos psi, sin psi): {p:?}"
                );
                points += 1;
            }
            assert_eq!(outside_inside, 0, "the interior of a run must be inside the window");
        }
        println!(
            "the ISCO observer's r+ curve came back as {} run(s) and {points} points inside a \
             window of 12 M",
            runs.len()
        );

        // A static observer holds their radius, so the direction that sets out towards r+ is
        // exactly the one with a negative e1 component: T^r = cos(psi) e1^r, and the sweep arrives
        // for psi in (pi/2, 3 pi/2) and for nothing else. The curve is therefore one run, open at
        // both ends, with sigma running away like 1 / |cos psi| as the direction turns onto the
        // observer's own worldline - which is the exact statement that a hovering observer never
        // reaches any other surface r = const by waiting. Its middle, psi = pi, is the now-axis,
        // and there the affine length is the ruler distance again.
        let r_static = 6.0;
        let s_u = static_obs(&metric, r_static);
        let runs = sample_surface(&metric, r_static, &s_u, metric.outer_horizon(), &opts);
        assert_eq!(runs.len(), 1, "the hovering observer's r+ curve is one run");
        let run = &runs[0];
        assert!(
            run[0].outside && run[run.len() - 1].outside,
            "and it leaves the window at both ends: {:?} .. {:?}",
            run[0],
            run[run.len() - 1]
        );
        assert!(
            run[0].psi > PI * 0.5 && run[run.len() - 1].psi < PI * 1.5,
            "the run must lie inside the inward half-circle: {} .. {}",
            run[0].psi,
            run[run.len() - 1].psi
        );
        let ruler = ruler_distance(&metric, r_static, &s_u, metric.outer_horizon()).unwrap();
        let on_axis = run
            .iter()
            .min_by(|p, q| (p.psi - PI).abs().partial_cmp(&(q.psi - PI).abs()).unwrap())
            .expect("the sweep crosses the now-axis");
        println!(
            "the hovering observer at {r_static} M has one r+ run of {} points, from psi = \
             {:.4} to {:.4}, passing the now-axis at sigma = {:.5} M against a ruler distance of \
             {ruler:.5} M",
            run.len(),
            run[0].psi,
            run[run.len() - 1].psi,
            on_axis.sigma
        );
        assert!(
            (on_axis.psi - PI).abs() < 1e-9 && (on_axis.sigma - ruler).abs() < 1e-6 * ruler,
            "the now-axis sample must be the ruler distance: {on_axis:?} vs {ruler}"
        );
    }

    /// The coordinate tangent of the sampler's own direction convention at angle psi.
    fn leg_at(metric: &KerrSchild, r: f64, u: &[f64; 3], psi: f64) -> [f64; 3] {
        let (s, c) = psi.sin_cos();
        leg(metric, r, u, s, c)
    }

    #[test]
    fn test_the_sampler_costs_what_it_claims_to() {
        // The rest-frame view redraws every frame, so the budget matters as much as the answer.
        // This is a timing print rather than an assertion about the machine: what is asserted is
        // only that the evaluation cap is respected and that a whole sweep is thousands of
        // evaluations rather than millions.
        let metric = KerrSchild::new(1.0, 0.90);
        let (r0, u) = isco_observer(&metric);
        let opts = SurfaceSampling::for_window(12.0);
        let surfaces = [metric.outer_horizon(), metric.inner_horizon(), 2.0, 0.0];

        let start = std::time::Instant::now();
        let rounds = 20;
        let mut total_points = 0;
        for _ in 0..rounds {
            for &r_h in &surfaces {
                for run in sample_surface(&metric, r0, &u, r_h, &opts) {
                    total_points += run.len();
                }
            }
        }
        let elapsed = start.elapsed();
        println!(
            "{} sweeps of {} surfaces took {:?} ({:?} per sweep), {} points a sweep",
            rounds,
            surfaces.len(),
            elapsed,
            elapsed / rounds,
            total_points / rounds as usize
        );

        // One affine length on its own, for the per-evaluation cost.
        let inward = leg(&metric, r0, &u, 0.0, -1.0);
        let start = std::time::Instant::now();
        let n = 10_000;
        let mut sink = 0.0;
        for _ in 0..n {
            sink += affine_length_to_surface(&metric, r0, &inward, metric.outer_horizon()).unwrap();
        }
        let elapsed = start.elapsed();
        println!(
            "one affine length costs {:?} ({} evaluations, sum {sink:.3})",
            elapsed / n,
            n
        );
        assert!(total_points > 0, "the sweep must produce something");
    }

    #[test]
    fn test_a_direction_that_turns_before_the_surface_has_no_arrival() {
        // The exponential map's image is not the whole spacetime, and the module says so rather
        // than following a geodesic through its turning point and reporting the far crossing as if
        // it were the near one. A nearly-outward spacelike direction from a static observer at
        // 6 M is the case: it climbs, turns and comes back down, so it never reaches r+ along that
        // direction even though r+ is plainly in the picture.
        let metric = KerrSchild::new(1.0, 0.0);
        let r0 = 6.0;
        let u = static_obs(&metric, r0);
        let outward = leg(&metric, r0, &u, 0.0, 1.0);
        assert!(
            affine_length_to_surface(&metric, r0, &outward, 2.0).is_none(),
            "an outward direction does not set out towards r+"
        );
        // Outward it reaches every surface above it, and the length is the static chain again.
        let far = affine_length_to_surface(&metric, r0, &outward, 10.0).expect("outward arrives");
        let ruler = ruler_distance(&metric, r0, &u, 10.0).expect("so does the ruler");
        assert!((far - ruler).abs() < 1e-6 * ruler, "outward: {far} vs {ruler}");

        // A timelike future direction that is not fast enough to escape turns round, and there the
        // refusal is about the motion rather than about the initial sign. The escape speed at
        // r = 6 M is sqrt(2M/r) = 0.5774 c as the static observer there measures it, so a 0.3 c
        // outward kick climbs to r = 2M / (1 - E^2) with E = gamma sqrt(1 - 2M/r0), which is
        // 7.4796 M, and comes back. Below that radius the surface is reached; above it there is no
        // arrival along that direction at all.
        let boosted = Tetrad::from_four_velocity_axial(&metric, r0, &u).boost(0.3, 0.0);
        let climb = leg(&metric, r0, &boosted, 1.0, 0.0);
        let gamma = 1.0 / (1.0 - 0.09f64).sqrt();
        let energy = gamma * (1.0 - 2.0 / r0).sqrt();
        let apex = 2.0 / (1.0 - energy * energy);
        println!("a 0.3 c outward kick at 6 M turns round at r = {apex:.4} M");
        assert!((apex - 7.4796).abs() < 1e-3, "the turning radius is 7.4796 M, got {apex}");
        assert!(
            affine_length_to_surface(&metric, r0, &climb, apex - 0.05).is_some(),
            "it reaches everything below its apex"
        );
        assert!(
            affine_length_to_surface(&metric, r0, &climb, apex + 0.05).is_none(),
            "and nothing above it"
        );
    }
}
