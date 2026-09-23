//! What one observer actually *sees* of the other: the event where their past light cone cuts the
//! other's worldline, placed in their own frame by the exact normal-coordinate rule.
//!
//! The rest-frame view used to draw the other observer at the linear image of the coordinate
//! offset taken at the same coordinate time - which is neither where they are seen nor where they
//! are, but a statement about a simultaneity convention pushed through a first-order map. Two
//! things are wrong with it and this module replaces both.
//!
//! *What* is drawn. Nobody sees anybody "now". What arrives at an event is light, so the event to
//! draw is the one on the other's worldline that the focus observer's past light cone passes
//! through: the direct image, the latest such event, ignoring the higher-order images a ray that
//! winds round the hole would also deliver.
//!
//! *Where* it is drawn. In Riemann normal coordinates about the focus observer's event, a point is
//! at xi^a = sigma T^a for the geodesic that reaches it, and for a null geodesic there is a
//! natural scale: normalise the future-directed tangent K so that -K . u_focus = 1, i.e. so that
//! the observer's own clock measures unit frequency on it. Then the affine parameter lambda at the
//! emission event is at once the normal-coordinate *time* ago and the normal-coordinate *distance*
//! away, which is exactly the statement that the emission event lies on the 45-degree past cone of
//! the three-dimensional normal-coordinate chart:
//!
//!     xi = lambda (-1, n^1, n^2),
//!
//! with n the unit spatial direction, in the focus observer's tetrad, that the light arrives
//! *from* - it points from the observer towards the source, which is the way a telescope points.
//!
//! *Which plane the rest-frame view draws.* That view has room for one spatial direction and it
//! gives that room to the focus observer's own radial leg e1, whatever the other observer does:
//! the plane is span(e0, e1) of `Tetrad::from_four_velocity_axial`, and the frame, the surfaces,
//! the grid, the cone and the ruler are the ones that observer has when nobody else is on the
//! canvas. The emission event is then painted into that plane, at the orthogonal projection
//! [`AsSeen::xi_projected`] of xi onto it,
//!
//!     (xi^1, xi^0) = (lambda n^1, -lambda),
//!
//! which lies inside the 45-degree past cone by exactly the part of the offset that points round
//! the hole, lambda |n^2|, and on the cone when the light arrives in the plane. The view says as
//! much: it marks the cone at the light-travel distance lambda, ties that mark to the dot, and
//! prints lambda |n^2| in the other observer's box. The plane used to follow the line of sight
//! instead, which put the dot on the cone by construction and made the whole picture - every
//! surface curve, every grid line - depend on a solve that goes ill-conditioned at a large boost.
//! See `spacetime_canvas::PICTURE_BOOST_LIMIT`.
//!
//! The same ray carries the exact frequency ratio
//!
//!     g = nu_seen / nu_emitted = (K . u_focus) / (K . u_other at emission),
//!
//! in which the ray's affine scale and its conserved energy both cancel, so g is finite and
//! positive at and inside both horizons like everything else in `wavefront`.
//!
//! The ray is the app's own null geodesic, integrated in the coordinate time of the ingoing
//! Kerr-Schild chart (`wavefront::NullRay`), because that parameterisation stays regular where the
//! affine one runs away: an outgoing ray settling onto the Cauchy horizon takes infinite
//! coordinate time and has no finite affine tangent to carry. The affine length is then recovered
//! from the radial quadrature of `kerr_equatorial::normal_coords`, which is a closed form in r and
//! needs no step history at all; the tests check it against the accumulated dt / K^t along the ray.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

use kerr_equatorial::normal_coords::{RadialConstants, affine_length_between};

use crate::physics::geodesic::{GeodesicState, R_STOP, geodesic_accel};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::local_frame::LocalFrame;
use crate::physics::observer::{Observer, ObserverMode, TrailPoint};
use crate::physics::tetrad::Tetrad;
use crate::physics::wavefront::{NullRay, RayStep};

/// Directions in the cold-start fan. Twenty-four is fifteen degrees, which is fine enough that the
/// nearest ray to the crossing is within a tenth of a radian of it - a seed, not an answer - and
/// coarse enough that a cold solve is a fan of two dozen rays rather than a hundred.
const SCAN_RAYS: usize = 24;

/// Longest first step, in coordinate time, of the cold-start sweep backwards. It is an upper
/// bound rather than the step itself: `Solver::first_step` shortens it for a pair that is close
/// together, because a crossing nearer than a couple of strides is never bracketed at all.
const SCAN_FIRST_STEP: f64 = 0.25;

/// Floor under that shortened first step.
///
/// Why the first step has to follow the pair. A dip is three consecutive samples of one generator
/// with the middle one the smallest, and the sample at age zero is the first of them, so a crossing
/// closer than about two strides cannot be bracketed however many sweeps follow. Measured on two
/// observers holding station at r0 = 10 M of an a = 0.90 hole, with the fixed 0.25 M stride: a
/// radial gap of 0.2 M solved correctly (age 0.200 M looking inward, where an ingoing ray of this
/// chart has dr/dt = -1 exactly, and 0.299 M looking outward), while gaps of 0.1 M and below - a
/// crossing inside the first stride - came back `NotConverged` in both directions. Further out the
/// same fault was quieter and worse: at r0 = 25 and 30 M a radial gap of 0.1 M or an azimuthal gap
/// of 0.03 rad returned `Ok` with the image whose light had gone once round the hole, lambda and
/// age of 73 to 84 M and g = 1.3, in place of a direct image a tenth of an M old.
///
/// So the stride is a third of the separation the pair actually has, floored here. The floor is
/// what keeps the sweep's reach: from 1e-3 M at `SCAN_GROWTH` = 1.2 the stride reaches the
/// `SCAN_MAX_STEP` = 4 M cap after ln(4000)/ln(1.2) = 46 sweeps, having covered about 24 M of age,
/// and the remaining 114 of the `SCAN_MAX_STEPS` = 160 sweeps carry 114 * 4 = 456 M more - so
/// `MAX_AGE` = 400 M is still inside the budget from the finest start allowed.
const SCAN_MIN_STEP: f64 = 1e-3;

/// Growth of that step from one sweep to the next. The crossing is usually found in the first few
/// M, and the growth is what lets the same bounded number of steps still reach a pair a hundred M
/// apart without paying for the fine grid all the way out.
const SCAN_GROWTH: f64 = 1.2;

/// Largest step the sweep will take, however far it has got.
///
/// Without a ceiling the geometric growth makes the grid absurd exactly where the answer usually
/// is: a pair fifty M of light travel apart would be sampled every eleven M, and a ray's approach
/// to the other worldline - which is a dip a couple of M wide - would be stepped straight over
/// instead of bracketed. Four M is fine enough to bracket every dip the app can produce and coarse
/// enough that `MAX_AGE` is still inside `SCAN_MAX_STEPS`.
const SCAN_MAX_STEP: f64 = 4.0;

/// Sweeps the cold start is allowed. At the growth and the ceiling above this reaches `MAX_AGE`
/// with sweeps to spare, so it binds only on a geometry where nothing ever dips.
const SCAN_MAX_STEPS: usize = 160;

/// Cold-start candidates the refinement will be given before the solve is abandoned.
///
/// The sweep hands back the dips it finds in the order it finds them, which is the order of
/// increasing age, and the refinement takes them in that order. That ordering is the whole of the
/// direct-image rule: the direct image is the *latest* emission event, so it is the crossing at
/// the smallest age, so the first candidate that converges is it. A candidate that does not
/// converge costs one refinement and one re-swept fan.
const MAX_COLD_SEEDS: usize = 4;

/// Halvings of a refused Newton step before the refinement gives up on the iteration.
const MAX_BACKTRACKS: u32 = 10;

/// Epochs the cold march aims to take between the start of the run and the present. It is an aim
/// rather than a count: a step that lands on a different image is halved and retried, and a run of
/// easy ones lengthens the step again, so a long quiet run costs about this many refinements and a
/// short awkward stretch costs a few more.
const MARCH_STEPS: usize = 24;

/// Refinements the cold march may spend before it gives up, halvings included.
const MARCH_BUDGET: usize = 160;

/// Smallest march step, as a fraction of the whole span. A step below this has been halved a dozen
/// times without finding an image that continues the one before it, which is not a step size
/// problem any more.
const MARCH_MIN_FRACTION: f64 = 1e-4;

/// How far past the widest radius either worldline has been seen at a trial ray is followed before
/// it is abandoned, as a multiple of that radius, plus a fixed margin in M.
///
/// The direct image's ray never climbs above both of its endpoints - the maximum of |x| along a
/// segment is at an end, and light bends towards the hole rather than away from it - so a
/// generator that has gone well past everywhere either observer has ever been is not the one being
/// looked for. Retiring it early is worth a great deal: `wavefront` caps a substep at 0.02 M of
/// radius, so a ray followed out to 128 M from a focus at 8 M costs six thousand substeps, and two
/// dozen of them are what made a cold solve take 25 ms.
///
/// This is the only ceiling there is. A fixed backstop of 128 M used to sit on top of it, and it
/// was not a bound on the cost but a wall in the physics: a pair holding station at r0 >= 140 M
/// starts *outside* it, so `shoot` killed every trial ray at birth and the solve answered
/// `RaysDied` for a configuration whose direct image is as ordinary as any other. The cost argument
/// never needed a constant. It needs the ceiling to sit a little way above wherever the two
/// observers are, which is exactly what a multiple of their own reach says, and the work a ray may
/// do is then set by how far apart the pair is rather than by a number written down here.
const REACH_MARGIN: f64 = 1.6;
const REACH_SLACK: f64 = 4.0;

/// How far back in coordinate time the search will look for the emission event at all. A pair
/// inside the drawn field is at most a few tens of M of light travel apart; past this the answer
/// would be off every canvas the app has.
const MAX_AGE: f64 = 400.0;

/// Newton iterations allowed on the two-point problem.
const MAX_NEWTON: u32 = 24;

/// Residual at which the two-point problem is solved: the Kerr-Schild Cartesian separation between
/// the ray's event and the other observer's event at the same coordinate time, in M.
const NEWTON_TOL: f64 = 1e-9;

/// How far the finite-difference column of the Jacobian moves the launched ray's coordinate
/// velocity (dr/dt, r dphi/dt): the step in arrival angle is chosen to produce this, see
/// `Solver::alpha_step`. The residual is smooth in the ray's direction and of order one, so this
/// trades four digits of the derivative for four digits of the difference; Newton converges
/// quadratically on the rest.
const ALPHA_STEP: f64 = 1e-6;

/// How far back in coordinate time an emission event may come, from one frame's answer to the
/// next, and still count as the same image, in M. A frozen image's emission time jitters back by
/// 1e-6 to 6e-6 M from one frame to the next by rounding alone (see `continues`), so the slack
/// stands well above that, while an image that is not the same is tens of M away in emission time
/// - or, at a caustic, the same to four decimals and caught by the second guard on lambda.
const EMISSION_SLACK: f64 = 1e-3;

/// A solve that ends within this fraction of the image's own distance lambda of the other
/// worldline, without reaching `NEWTON_TOL`, is still the picture: the residual is carried in the
/// answer for anyone who wants to know.
///
/// The acceptance is relative and only relative. It used to be an absolute 1e-5 M as well, and
/// that absolute hair took in a wrong answer: two raindrops released 1.5e-4 M apart with E = 1,
/// L = 2 at a = 0.90 are seen at lambda = 4e-5 M and closing, and once the focus observer's boost
/// passed u^t ~ 20 the Newton, seeded a few hundredths of a radian off, stalled at a separation
/// of 2.5e-6 M - an eighth of the distance to the image - at whatever angle it was seeded with,
/// and the absolute hair accepted it. Its lambda came out half the true image's, so its emission
/// was later, so the sweep for the youngest image adopted it, and the drawn plane swung by 0.4
/// radians in one frame and then stood still for as long as the run lasted, because a stalled
/// solve seeded from itself never moves. Against the image's distance that answer is a 12% miss
/// and is refused, and the refusal sends the solve back to the cold march, which finds the true
/// image at a residual of 5e-10 M.
///
/// The fraction is what the two measured stalls need. A far image (lambda = 0.626 M, Alice's
/// raindrop seen from Bob freezing onto r-) stalls at 1e-6 to 4e-6 M once u^t is past 1e6, and
/// two observers freezing onto r- together, 1e-3 M apart with u^t in the tens of thousands, are
/// seen at lambda = 0.003 M and stall at 1.0e-5 to 1.2e-5 M - a floor set by the interpolated
/// worldline and the ray. Both sit under one per cent, and one per cent of the distance is under
/// a screen point at any zoom that shows the image at all.
const NEAR_MISS_OF_LAMBDA: f64 = 1e-2;

/// The bounds on that step in the angle itself. The lower bound is where the angle's own floating
/// point resolution starts to matter; the upper is a secant over a twentieth of a radian, which
/// is still a fair derivative of a residual this smooth.
const ALPHA_STEP_MIN: f64 = 1e-9;
const ALPHA_STEP_MAX: f64 = 0.05;

/// Where on the other observer's worldline the emission event was read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorldlineSource {
    /// Interpolated between two recorded events of the trail.
    Trail,
    /// Before the release: the hold, continued backwards in closed form, which is how anything is
    /// seen at all at t = 0. Only for a release the hold can join without a jump - a release from
    /// rest, or a fixed-radius worldline - see [`WorldlineSource::Geodesic`] for the other case.
    Hold,
    /// Before the release: the free-fall worldline itself, continued backwards through the release
    /// event. A release that arrives already moving - from rest at infinity, or onto a circular
    /// orbit - has no hold that joins it: the app's hold for such a release is the static
    /// observer, and a static Bob one instant and a Bob falling at 0.6 c the next is a kick that
    /// no light should report. Measured before this existed, at the user's own starting file:
    /// two raindrops released together at 4.5 M and 0.00015 M apart, which must see each other
    /// at g = 1, were seen at g = 1.39 one way and 1.12 the other, the Doppler shift between the
    /// static hold and the fall - so Bob's beacon went into the ultraviolet and Alice's turned
    /// green. Along the geodesic continued backwards both were always falling together.
    Geodesic,
    /// A Static or ZAMO worldline, carried back analytically at its own constant 4-velocity.
    FixedRadius,
    /// A marker the user is dragging, which the app holds at fixed (r, phi) and does not integrate.
    Dragged,
}

/// One event of the other observer's worldline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeenEvent {
    /// Coordinate time of the emission event.
    pub t: f64,
    /// Radius of the emission event.
    pub r: f64,
    /// Azimuth of the emission event, unwrapped against the focus observer's own azimuth rather
    /// than folded into [0, 2 pi), so that a difference against the focus is a small number.
    pub phi: f64,
    /// The emitter's own watch at the emission event. Negative for an event before the run began,
    /// the watch being zeroed at the observer's creation.
    pub tau: f64,
    /// The emitter's 4-velocity there.
    pub u: [f64; 3],
}

/// The whole answer: where the other observer is seen, how far away and how long ago that is in
/// the focus observer's own frame, and what their light does to a frequency on the way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AsSeen {
    /// The event the light left.
    pub emission: SeenEvent,
    /// How long ago that was on the chart's clock: t_focus - t_emission. It is *not* what the
    /// focus observer's own clock says about the delay, which is `lambda`.
    pub age: f64,
    /// Affine length of the null geodesic from the emission event to the focus observer's event,
    /// with the tangent normalised so that -K . u_focus = 1. It is at once the normal-coordinate
    /// time ago and the normal-coordinate distance away.
    pub lambda: f64,
    /// The unit spatial arrival direction (n^1, n^2) in the focus observer's *axial* tetrad - the
    /// one `LocalFrame::for_observer` draws in - pointing from the observer towards the source.
    pub n: [f64; 2],
    /// The emission event in the focus observer's normal coordinates, (xi^0, xi^1, xi^2) =
    /// lambda (-1, n^1, n^2).
    pub xi: [f64; 3],
    /// nu_seen / nu_emitted, exactly, from the ray. Below one is a redshift.
    pub g: f64,
    /// The tetrad angle of the arrival direction: n = (cos alpha, sin alpha). Carried so that the
    /// next frame can be warm-started from it.
    pub alpha: f64,
    /// Where the emission event was read from on the other's worldline.
    pub source: WorldlineSource,
    /// Kerr-Schild Cartesian separation, in M, between the solved ray event and the other
    /// observer's event at the same coordinate time. A measure of the solve, not of the physics.
    pub residual: f64,
    /// Newton iterations the solve took.
    pub iterations: u32,
    /// Whether the cold-start fan had to be run, as against the caller's warm start being enough.
    pub cold: bool,
    /// How many extra turns round the hole the ray made on its way here, signed, prograde
    /// positive: the ray's own azimuthal excursion less the direct one, divided by a full turn.
    /// Zero for the direct image and non-zero for a ray that has wound round the photon sphere,
    /// which is what [`images`] enumerates.
    pub windings: i32,
}

impl AsSeen {
    /// The seed to hand back next frame, which is what turns a few-millisecond cold solve into a
    /// sub-millisecond warm one.
    pub fn seed(&self) -> AsSeenSeed {
        AsSeenSeed {
            alpha: self.alpha,
            age: self.age,
            emission_t: self.emission.t,
            lambda: self.lambda,
        }
    }

    /// The same answer, marked as having been reached without a seed. The cold march refines every
    /// epoch it walks through from the one before it, so only the first of them is a cold solve in
    /// its own right; `cold` is a statement about what the *caller* supplied, which is what a
    /// caller deciding whether to keep threading seeds wants to know.
    fn cold(mut self) -> Self {
        self.cold = true;
        self
    }

    /// +1 when the line of sight has an outward component, or none and a prograde one; -1
    /// otherwise: which half of the rest-frame view's horizontal axis the light comes from.
    ///
    /// The view marks the past cone at (sign * lambda, -lambda), the point the emission event
    /// would occupy if the light arrived in the drawn plane, and ties that mark to the projected
    /// dot at (lambda n^1, -lambda). This sign is what puts the mark on the same side of the
    /// canvas as the dot - the outward side whenever the light has any outward component at all,
    /// and the inward side otherwise - so the tie runs horizontally and its length is the
    /// shortening the projection costs. A line of sight with no outward component at all takes
    /// the prograde side by convention; both halves of the cone stand at the same 45 degrees and
    /// the dot sits at xi^1 = 0 there, so that choice moves nothing the eye can read.
    pub fn sight_sign(&self) -> f64 {
        if self.n[0] > 0.0 || (self.n[0] == 0.0 && self.n[1] >= 0.0) {
            1.0
        } else {
            -1.0
        }
    }

    /// The orthogonal projection of `xi` onto the plane the rest-frame view draws, in the same
    /// ordering `LocalLine::point` uses: distance along the plane's spacelike leg first, local
    /// time second.
    ///
    /// That plane is span(e0, e1) of the focus observer's axial tetrad - the observer's own radial
    /// plane, which is what the view draws whether or not anybody else stands on the canvas - so
    /// the projection keeps the component of the offset along e1 and drops the component along e2:
    ///
    ///     xi^1 = lambda n^1,      xi^0 = -lambda,      dropped: xi^2 = lambda n^2.
    ///
    /// n is a unit vector, so |xi^1| <= lambda and the point lies *inside* the 45-degree past
    /// cone, reaching the cone exactly when the light arrives in the plane. The shortfall is the
    /// whole of what the projection costs: lambda (1 - |n^1|) along the axis, against an offset of
    /// lambda |n^2| out of the plane. The view draws both rather than hiding either - a faint mark
    /// on the cone at the light-travel distance, a dashed tie from the dot to that mark, and the
    /// out-of-plane offset itself in the other observer's box.
    ///
    /// So a dot inside a drawn horizon curve is by itself no statement about the emission event,
    /// because the curve is that surface's slice through this plane and the event stands off the
    /// plane. The region tag in the box's title reads the emission event's own radius and is the
    /// exact answer to that question.
    pub fn xi_projected(&self) -> [f64; 2] {
        [self.lambda * self.n[0], -self.lambda]
    }

    /// The signed angle of the line of sight from the observer's own outward radial leg, in
    /// radians on (-pi, pi]: positive towards the observer's local +phi direction, which is
    /// prograde. It is the angle the drawn plane has been turned through, and the header prints it
    /// so that a reader knows which way the horizontal axis points.
    pub fn sight_angle(&self) -> f64 {
        wrap(self.n[1].atan2(self.n[0]))
    }
}

/// A starting point for the solver, taken from the previous frame's answer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AsSeenSeed {
    /// The arrival angle that solved it last time.
    pub alpha: f64,
    /// How far back in coordinate time the emission event was.
    pub age: f64,
    /// Coordinate time of the emission event the seed came from. The warm path checks the answer
    /// it gets against this, so that a Newton which has landed on a different image is thrown away
    /// rather than drawn. See `continues`.
    pub emission_t: f64,
    /// Affine length of the ray the seed came from, checked the same way and for the same reason.
    pub lambda: f64,
}

/// Why there is nothing to draw. Every one of these is a physical statement or an honest limit of
/// the recorded data, never a numerical shrug.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoImage {
    /// The focus observer has settled onto the far branch of the Cauchy horizon. Their u^t has run
    /// out to `geodesic::U_T_STALL`, so their frame is aberrated into a single point and there is
    /// no drawable picture of anything in it; the app declares the worldline over there too.
    FocusFrozen,
    /// The focus observer has reached the ring. There is no more of their worldline to receive on.
    FocusEnded,
    /// The other observer's worldline does not extend to the time the light would have had to
    /// leave: they reached the ring, or froze, before then.
    OtherEnded,
    /// The stretch of the other's worldline the answer needs has been evicted from their trail by
    /// its cap. The data is gone and nothing here will invent it.
    TrailEvicted,
    /// Every trial ray was swallowed by the ring or climbed out of the search ceiling before it
    /// could reach the other's worldline.
    RaysDied,
    /// The search found no crossing inside `MAX_AGE` of coordinate time.
    NoCrossing,
    /// A crossing was bracketed but the refinement did not reach `NEWTON_TOL`.
    NotConverged,
}

/// Where the other observer is seen from the focus observer's current event, by the direct image.
///
/// `seed` is the previous frame's answer, if the caller kept one. With it the solve is the
/// two-point Newton and nothing else, which is a couple of ray integrations and some tens of
/// microseconds; without it, or when the warm start fails to converge, the cold path of
/// `march_from_the_start` runs instead, which is tens of milliseconds. A caller redrawing every
/// frame should therefore keep the seed and hand it back; handing back a *stale* one costs nothing
/// but a failed Newton, because the fallback is the cold path either way.
pub fn as_seen(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    seed: Option<AsSeenSeed>,
) -> Result<AsSeen, NoImage> {
    if focus.is_frozen() {
        return Err(NoImage::FocusFrozen);
    }
    if focus.r <= R_STOP {
        return Err(NoImage::FocusEnded);
    }
    if let Some(seed) = seed
        && let Ok(found) =
            Solver::new(metric, focus, other).refine(seed.alpha, seed.age.max(1e-6), false)
        && continues(&found, seed.emission_t, seed.lambda, focus.t)
    {
        return Ok(found);
    }
    march_from_the_start(metric, focus, other)
}

/// How much later than the current image's emission a candidate's has to be before it counts as
/// a younger image rather than as the same one refined from a different dip, in M of coordinate
/// time. Two dips of one crossing refine to the same event to far better than this.
const YOUNGER_MARGIN: f64 = 1e-3;

/// How far past the current image's age the sweep is still asked for dips when looking for a
/// younger image. The dips come in order of increasing age, but a dip is only registered one
/// sweep after the separation turned up and its age is a parabola's vertex through three coarse
/// samples, so the dip that refines to a younger image can sit a sweep or two past that image's
/// true age. Two of the widest sweep steps is the margin that measured as enough: with one M,
/// the youngest image at t = 90 M of the ISCO run was found from the threaded chain's seed and
/// missed from the cold march's answer, and with this it is found from both.
const YOUNGER_SLACK: f64 = 2.0 * SCAN_MAX_STEP;

/// A younger image of the other observer than `than`, if the fan can find one: an image whose
/// emission event is *later* on the other's worldline, which is the same as saying whose light
/// took less of the chart's time to arrive.
///
/// Why this exists. `as_seen` follows one image continuously, and continuity is the wrong rule
/// for an observer who goes round the hole. Alice on the prograde ISCO of an a = 0.90 hole
/// orbits once every 28 M, and the null connection that continuity hands her from one frame to
/// the next winds once more round the photon sphere with every orbit she makes: nothing about it
/// jumps, its emission time rises steadily and its affine length moves by a per cent a step, and
/// it is the image of Bob as he was a hundred M ago. Meanwhile a new image is born - at about
/// t = 43 M in that run - carrying Bob as he was a moment ago, and no local guard can tell the
/// chain so, because the chain has done nothing wrong. Which image is "Bob as seen" is therefore
/// a rule and not a computation, and the rule the view draws by is **the youngest image**: the
/// one whose emission event is latest. It is the picture with the least delay, the one whose ray
/// has wound the least, and - lensing magnification aside, which the brightness model does not
/// carry - the brightest.
///
/// How. One pass of `Solver::sweep` hands back its dips in order of increasing age, so the
/// first few dips are where any younger image is, and each is refined and kept if it converges
/// on an emission later than `than`'s. The sweep is stopped at `than`'s age plus a little slack,
/// since nothing after that can be younger. It asks for twice `MAX_COLD_SEEDS` dips because a
/// near miss makes a dip too: from the prograde ISCO the sweep at t = 60 M and at t = 65 M had
/// the young image behind four near misses, and four dips found nothing where eight found it. Late in an infall the sweep is
/// unreliable - see `march_from_the_start` - but unreliable here only costs a check that finds
/// nothing, and the caller keeps the image it had.
///
/// The fan is not the only candidate offered. `Solver::geometric_seed` is refined alongside the
/// dips and judged by exactly the same two tests, and it is what decides the case the fan cannot
/// see: two observers a hundredth of an M apart have a direct image at an age of a hundredth of an
/// M, inside the sweep's first sample, and the only crossing the fan brackets is the one whose
/// light went round the hole, which arrives some eighty M old. Without this candidate that wound
/// image is the only thing on offer and a pair standing next to each other watch each other's
/// distant past; with it the direct image is found, is the younger of the two, and wins on the
/// emission-time test the same way any other candidate would.
///
/// It costs a fan sweep and a few refinements, some milliseconds, so it is not for every frame:
/// the view runs it once per M of the focus observer's coordinate time and after any cold solve.
pub fn younger_image(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    than: &AsSeen,
) -> Option<AsSeen> {
    if focus.is_frozen() || focus.r <= R_STOP {
        return None;
    }
    let solver = Solver::new(metric, focus, other);
    let mut best: Option<AsSeen> = None;
    // A sweep that refuses outright is not a reason to give up here: the geometric candidate needs
    // no fan, and it is the one that answers for a close pair.
    let dips = solver
        .sweep(2 * MAX_COLD_SEEDS, than.age + YOUNGER_SLACK)
        .map_or_else(|_| Vec::new(), |sweep| sweep.dips);
    for (alpha, age) in solver.geometric_seed().into_iter().chain(dips) {
        // Nothing older than the current image can be younger than it. The dips arrive in order of
        // increasing age so this ends the useful part of the list, and the geometric candidate,
        // which does not come from the sweep and so is not in that order, is skipped by the same
        // test rather than ending it.
        if age > than.age + YOUNGER_SLACK {
            continue;
        }
        let Ok(image) = solver.refine(alpha, age, true) else {
            continue;
        };
        if image.emission.t < focus.t
            && image.emission.t > than.emission.t + YOUNGER_MARGIN
            && best.is_none_or(|kept| image.emission.t > kept.emission.t)
        {
            best = Some(image);
        }
    }
    best
}

/// The youngest image: `as_seen`, and then `younger_image` on top of it. This is the rule the
/// view draws by; `as_seen` alone is the cheap per-frame step of it.
pub fn as_seen_youngest(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    seed: Option<AsSeenSeed>,
) -> Result<AsSeen, NoImage> {
    let seen = as_seen(metric, focus, other, seed)?;
    Ok(younger_image(metric, focus, other, &seen).unwrap_or(seen))
}

/// Whether an answer continues the one the seed came from, or has landed on a different image.
///
/// The two guards are the cold march's, applied to the warm path for the same reason: a Newton
/// started from last frame's answer is only an initial guess, and deep in the strong field there
/// is a whole family of images to land on. Without them a chain threaded frame by frame follows
/// whichever image it happens to fall onto and stays there - from the prograde ISCO of an a = 0.90
/// hole, watching the app's own raindrop dropped from 27 M, an unguarded chain reports an emission
/// 96.81 M old at t = 120 M where the direct image is 47.49 M old, and everything about that
/// answer is self-consistent except which picture it is of.
///
/// * **The emission time is non-decreasing in the focus time.** The focus observer's past light
///   cone only grows as that observer's clock advances, so the latest event of the other's
///   worldline inside it only moves forward. An emission that has gone backwards, or one that has
///   overtaken the focus observer's own clock, is a different image.
/// * **The affine length is continuous.** Near a caustic two images agree in emission time to four
///   decimals while their lambdas differ by a factor of seven, so the first guard cannot tell them
///   apart and this one can. Lambda on a single image is a slow curve - it grew by a sixth per
///   step over the whole of the measured Kerr march - so a factor of four is slack by an order of
///   magnitude and still catches the jump.
///
/// A refusal costs the cold path, which is what the caller would have paid with no seed at all.
///
/// The slack on the first guard is `EMISSION_SLACK`, which has to sit above the solve's own
/// resolution and not at it. It was 1e-6 M, and that is where a frozen image jitters: once the
/// focus observer is freezing onto the far branch of r- their picture stops changing - Alice's
/// emission event sat at t = 4.87435 M frame after frame - and the converged emission time
/// came back 1e-6 to 6e-6 M *earlier* than the seed's from one frame to the next, pure
/// rounding. The guard refused every one of those honest frames, the cold march could not do
/// better, and the view reported no image for a picture that had not moved.
fn continues(next: &AsSeen, emission_t: f64, lambda: f64, now: f64) -> bool {
    next.emission.t >= emission_t - EMISSION_SLACK
        && next.emission.t < now
        && next.lambda <= 4.0 * lambda + 1.0
        && next.lambda + 1.0 >= 0.25 * lambda
}

/// Every image of the other observer the cold fan can find from the focus observer's event, the
/// direct one first and at most `max_images` in all.
///
/// A source deep in the strong field is seen more than once. Besides the direct image there is a
/// sequence of ever-fainter ones whose rays wind round the photon sphere before arriving, each of
/// them a real null connection between the same two worldlines and each carrying its own delay,
/// shift and arrival direction. `as_seen_youngest` reports the youngest image alone, because
/// that is the picture the view draws; this enumerates the rest.
///
/// How. `Solver::cold_seed` already hands back the dips of the past-cone sweep in the order it
/// finds them, which is the order of increasing age, so asking it for the first, second, third
/// dip and refining each is the whole of the method. Two answers count as the same image when
/// their emission events and their affine lengths both agree, which is what keeps two dips on
/// neighbouring generators of one crossing from being reported twice.
///
/// It is not cheap - every extra image costs another sweep of the fan, a couple of milliseconds -
/// so nothing that runs per frame should call it. Nothing in the app does yet; it is here because
/// the higher-order images are worth showing and the fan already knows where they are.
// No caller outside the tests until the view draws the fainter images too, which is why the
// enumeration is written down now rather than rediscovered then.
#[allow(dead_code)]
pub fn images(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    max_images: usize,
) -> Vec<AsSeen> {
    let mut found: Vec<AsSeen> = Vec::new();
    if max_images == 0 {
        return found;
    }
    // The direct image is the marched one rather than the fan's first dip: the march is what makes
    // the direct image reliable late in a run, and a caller comparing this list against `as_seen`
    // should find the same answer at the head of it.
    if let Ok(direct) = as_seen(metric, focus, other, None) {
        found.push(direct);
    }
    let solver = Solver::new(metric, focus, other);
    let distinct = |found: &[AsSeen], candidate: &AsSeen| {
        found.iter().all(|seen| {
            (seen.emission.t - candidate.emission.t).abs() > 1e-3
                || (seen.lambda - candidate.lambda).abs() > 1e-3 * seen.lambda.max(1.0)
        })
    };
    for skip in 0..(2 * max_images) {
        if found.len() >= max_images {
            break;
        }
        let Ok((alpha, age)) = solver.cold_seed(skip) else {
            break;
        };
        if let Ok(image) = solver.refine(alpha, age, true)
            && distinct(&found, &image)
        {
            found.push(image);
        }
    }
    found.sort_by(|a, b| a.age.total_cmp(&b.age));
    found
}

/// The cold path: solve at the earliest event of the run and walk the answer forward to now.
///
/// A sweep of the past cone can only ever *guess* which of its dips is the direct image, and the
/// guess gets worse as the run goes on. Late in an infall the generator that carries the image
/// hugs r+ for tens of M of coordinate time while the rest of the cone sweeps out past where the
/// other observer used to be, so a dip in the sweep is as likely to be a near miss as a crossing:
/// measured on an a = 0.90 hole with the focus on the prograde ISCO and the other dropped from
/// 27 M, a sweep-seeded cold solve failed outright at about a fifth of the sampled epochs and, at
/// several of the rest, answered with an emission event *later* than the one it had given a
/// shorter run before - which is not a different accuracy but a different image.
///
/// What a warm solve has that a cold one does not is history: it is handed the answer from an
/// event a frame earlier and only ever has to move it a little, so it cannot come off the branch
/// it is on. So the cold path manufactures that history. It solves once at the earliest event both
/// worldlines can still be read at - where both observers are holding station, the configuration
/// is static and a sweep is reliable - and then marches the focus forward to the present in
/// bounded steps, each refined from the last. Every step is the same `refine` the warm path uses,
/// so the answer that arrives at the present is the answer a run threaded frame by frame would
/// have arrived at.
///
/// The march is guarded by a monotonicity that belongs to the direct image and to nothing else:
/// as the focus observer's clock advances, their past light cone can only grow, so the latest
/// event of the other's worldline inside it can only move forward. **The emission time is
/// non-decreasing in the focus time.** A step that produces an earlier emission has jumped to a
/// different image and is refused; the step is halved and tried again. That, and a budget, is what
/// bounds the march.
fn march_from_the_start(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
) -> Result<AsSeen, NoImage> {
    let start = earliest_readable(metric, focus).max(earliest_readable(metric, other));
    let now = focus.t;
    let span = (now - start).max(0.0);

    let mut answer = solve_at(metric, focus, other, start.min(now))?;
    if span <= 1e-9 {
        return Ok(answer.cold());
    }

    let mut t = start;
    let mut step = span / MARCH_STEPS as f64;
    let mut budget = MARCH_BUDGET;
    while t < now && budget > 0 {
        budget -= 1;
        let target = (t + step).min(now);
        let attempt = solve_at_from(metric, focus, other, target, answer.seed());
        // The two guards of `continues`, which is where they are written down: between them they
        // are what makes the march follow one image rather than wander through the family of them,
        // and they are the same two the warm path applies to a caller's seed.
        let good = attempt
            .as_ref()
            .is_ok_and(|next| continues(next, answer.emission.t, answer.lambda, target));
        if good {
            answer = attempt.expect("checked just above");
            t = target;
            // Lengthen again after a run of easy steps, so that a single awkward epoch does not
            // make the rest of the march pay for it.
            step = (step * 1.4).min(span / 4.0);
        } else {
            step *= 0.5;
            if step <= span * MARCH_MIN_FRACTION {
                return Err(NoImage::NotConverged);
            }
        }
    }
    if t < now {
        return Err(NoImage::NotConverged);
    }
    Ok(answer.cold())
}

/// The earliest coordinate time this observer's worldline can still be read at.
///
/// A hold reaches as far back as it is asked to, so an observer whose trail still holds its
/// release event can be read from the beginning of time; one whose trail has been eaten into by
/// its cap can only be read from its oldest surviving event. The fixed-radius and dragged modes
/// are analytic and have no floor at all. Whichever it is, it is where the cold march has to
/// start, because it is the earliest event at which there is anything to solve.
fn earliest_readable(metric: &KerrSchild, obs: &Observer) -> f64 {
    if obs.effective_mode(metric) != ObserverMode::FreeFall {
        return obs.start.t;
    }
    match obs.trail.front() {
        Some(oldest) if oldest.t > obs.release_t + 1e-9 => oldest.t,
        _ => obs.start.t,
    }
}

/// Solve cold at one event of the focus observer's own worldline, by sweeping the past cone.
///
/// This is the seeded sweep, kept for the one job it is reliable at: the first epoch of the march,
/// where both observers are still holding station and the picture is static. It takes the sweep's
/// candidates in the order it found them, which is the order of increasing age, and stops at the
/// first that refines - the direct image being the crossing at the smallest age, that ordering is
/// the direct-image rule. A candidate that does not refine is not a failure: it is one dip in a
/// separation curve that turned out not to be a crossing, and the next one is asked for instead.
fn solve_at(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    t: f64,
) -> Result<AsSeen, NoImage> {
    let solver = Solver::at(metric, focus_frame_at(metric, focus, t)?, other);
    // The geometric guess, before the fan. When the pair is close this *is* the direct image - the
    // flat-space offset is the whole answer to first order - and it is the one case the fan cannot
    // reach, because the crossing happens inside its first stride. When the pair is not close the
    // guess is only a seed like any other, and a seed that does not converge costs one refinement
    // and falls through to the sweep below.
    if let Some((alpha, age)) = solver.geometric_seed()
        && let Ok(found) = solver.refine(alpha, age, true)
    {
        return Ok(found);
    }
    let mut refusal = NoImage::NoCrossing;
    for skip in 0..MAX_COLD_SEEDS {
        match solver.cold_seed(skip) {
            Ok((alpha, age)) => match solver.refine(alpha, age, true) {
                Ok(found) => return Ok(found),
                Err(why) => refusal = why,
            },
            Err(why) => {
                if skip == 0 || why != NoImage::NoCrossing {
                    refusal = why;
                }
                break;
            }
        }
    }
    Err(refusal)
}

/// One step of the march: the same `refine` the warm path runs, at an earlier event of the focus
/// observer's worldline.
fn solve_at_from(
    metric: &KerrSchild,
    focus: &Observer,
    other: &Observer,
    t: f64,
    seed: AsSeenSeed,
) -> Result<AsSeen, NoImage> {
    Solver::at(metric, focus_frame_at(metric, focus, t)?, other).refine(
        seed.alpha,
        seed.age.max(1e-6),
        false,
    )
}

/// The focus observer's frame at an earlier event of their own worldline, read the same way the
/// other observer's is.
fn focus_frame_at(metric: &KerrSchild, focus: &Observer, t: f64) -> Result<FocusFrame, NoImage> {
    let line = OtherWorldline::new(metric, focus);
    let (point, _) = line.point_at(t.min(focus.t))?;
    if point.r <= R_STOP {
        return Err(NoImage::FocusEnded);
    }
    Ok(FocusFrame::new(metric, point.t, point.r, point.phi, point.u))
}

/// The focus observer's event and the frame everything is quoted in.
struct FocusFrame {
    t: f64,
    r: f64,
    phi: f64,
    u: [f64; 3],
    tetrad: Tetrad,
}

impl FocusFrame {
    fn new(metric: &KerrSchild, t: f64, r: f64, phi: f64, u: [f64; 3]) -> Self {
        let r = r.max(1e-4);
        Self {
            t,
            r,
            phi,
            u,
            tetrad: Tetrad::from_four_velocity_axial(metric, r, &u),
        }
    }
}

/// One observer's recorded worldline, as a function of coordinate time that the root finder can
/// call tens of times without paying for a clone of their trail.
///
/// It is named for the observer it was written for - the *other* one, whose worldline the emission
/// event is found on - and the cold march reads the focus observer's own worldline through it too,
/// because "where was this observer when the chart's clock read t" is the same question either way.
struct OtherWorldline<'a> {
    metric: &'a KerrSchild,
    obs: &'a Observer,
    mode: ObserverMode,
    /// The worldline before the release, integrated backwards from the release event on demand
    /// and kept for the rest of this solve; oldest event at the front, the release event at the
    /// back. Empty until the first pre-run event is asked for, and never filled for a worldline
    /// whose pre-run stretch is a hold. See `pre_run_at`.
    pre_run: RefCell<VecDeque<TrailPoint>>,
}

struct Solver<'a> {
    metric: &'a KerrSchild,
    focus: FocusFrame,
    other: OtherWorldline<'a>,
    /// Radius past which a trial ray has left every part of the chart either worldline occupies
    /// and is abandoned. See `REACH_MARGIN`.
    ceiling: f64,
}

/// One evaluation of the two-point problem: the ray at the trial age, the other's event there, and
/// the mismatch between them.
/// What one pass of the past-cone fan found: see `Solver::sweep`.
struct Sweep {
    /// (arrival angle, age) of each dip, in the order found, which is the order of increasing age.
    dips: Vec<(f64, f64)>,
    /// The closest approach of any generator, when the sweep ran out before finding as many dips
    /// as it wanted; `None` when it found them all.
    closest: Option<(f64, f64)>,
    /// Why the last empty pass of the sweep was empty, for a caller with nothing else to report.
    refusal: NoImage,
}

struct Probe {
    ray: NullRay,
    event: SeenEvent,
    source: WorldlineSource,
    /// (r_ray - r_other, wrapped phi_ray - phi_other).
    gap: [f64; 2],
}

impl Probe {
    /// The mismatch as a length: the Kerr-Schild Cartesian separation of the two events, which is
    /// the one measure that does not go to zero merely because the azimuth has been divided by a
    /// small radius.
    fn separation(&self, metric: &KerrSchild) -> f64 {
        let (xa, ya) = metric.cartesian_position(self.ray.r, self.ray.phi);
        let (xb, yb) = metric.cartesian_position(self.event.r, self.event.phi);
        (xa - xb).hypot(ya - yb)
    }
}

impl<'a> Solver<'a> {
    fn new(metric: &'a KerrSchild, focus: &Observer, other: &'a Observer) -> Self {
        let frame = FocusFrame::new(
            metric,
            focus.t,
            focus.r,
            focus.phi,
            focus.four_velocity(metric),
        );
        Self::at(metric, frame, other)
    }

    /// The solver for an arbitrary focus event, which is what the cold march builds at each epoch
    /// it steps through.
    fn at(metric: &'a KerrSchild, focus: FocusFrame, other: &'a Observer) -> Self {
        // The widest radius either worldline has been seen at, in O(1): an infaller's oldest
        // recorded event is its highest, a hold never leaves the radius it was created at, and a
        // fixed-radius worldline is where it is. A bound orbit released inward could in principle
        // have an apoapsis above all four, which is what the generous margin is for.
        let reach = focus
            .r
            .max(other.r)
            .max(other.start.r)
            .max(other.trail.front().map_or(0.0, |p| p.r))
            .max(other.trail.back().map_or(0.0, |p| p.r));
        Self {
            metric,
            focus,
            other: OtherWorldline::new(metric, other),
            ceiling: REACH_MARGIN * reach + REACH_SLACK,
        }
    }

    /// A past-directed trial ray, launched from the focus event along the arrival direction
    /// n = (cos alpha, sin alpha) and integrated back by `age` of coordinate time.
    ///
    /// The ray object is the *future*-directed geodesic: `Tetrad::null_direction(alpha + pi)` has
    /// tetrad components (1, -cos alpha, -sin alpha), so it travels towards the observer from the
    /// direction n and arrives moving in -n, which is what "n points at the source" means. It is
    /// also already normalised the way `lambda` wants, because e0 is the observer's own
    /// 4-velocity and -g(e0 + ..., u) = 1 by orthonormality.
    fn launch(&self, alpha: f64) -> NullRay {
        NullRay::from_local_direction(
            self.metric,
            self.focus.t,
            self.focus.r,
            self.focus.phi,
            &self.focus.tetrad,
            alpha + PI,
            &self.focus.u,
        )
    }

    fn shoot(&self, alpha: f64, age: f64) -> Option<NullRay> {
        let mut ray = self.launch(alpha);
        if age > 0.0 && ray.step_back(self.metric, age) == RayStep::BudgetExhausted {
            return None;
        }
        (ray.alive() && ray.r.is_finite() && ray.r < self.ceiling).then_some(ray)
    }

    fn probe(&self, alpha: f64, age: f64) -> Result<Probe, NoImage> {
        let ray = self.shoot(alpha, age).ok_or(NoImage::RaysDied)?;
        self.probe_with(ray, age)
    }

    fn probe_with(&self, ray: NullRay, age: f64) -> Result<Probe, NoImage> {
        let (event, source) = self.other.at(self.focus.t - age, self.focus.phi)?;
        let gap = [ray.r - event.r, wrap(ray.phi - event.phi)];
        Ok(Probe {
            ray,
            event,
            source,
            gap,
        })
    }

    /// The flat-space first guess at where the other observer is seen, before any fan is swept.
    ///
    /// Take the other observer's event at the focus observer's *own* coordinate time - not at the
    /// emission time, which is the unknown - and carry the coordinate displacement (0, Delta r,
    /// Delta phi) into the focus tetrad with the first-order chart of `LocalFrame`. That gives a
    /// purely spatial local offset (xi^1, xi^2), and the flat-space picture of it is the whole
    /// guess: light arrives from the direction the source lies in,
    ///
    ///     alpha = atan2(xi^2, xi^1),
    ///
    /// measured from e1 towards e2 exactly as `Solver::launch` and `Tetrad::null_direction` measure
    /// it, and it took the time the distance is,
    ///
    ///     age = hypot(xi^1, xi^2),
    ///
    /// which is the first-order proper distance between the two events. Both statements are exact
    /// in the limit of a close pair and wrong by the curvature and by whatever the other observer
    /// did while the light was in flight otherwise - which is all a Newton seed has to be.
    ///
    /// Why it is worth having at all, when there is a fan. The fan samples the past cone on a grid
    /// and reads a crossing off a dip in one generator's separation, so it cannot see a crossing
    /// that happens before its first stride, and a pair a hundredth of an M apart is exactly that
    /// case. The guess has no grid: it is better the closer the pair is, which is precisely where
    /// the sweep is worst, so the two cover each other.
    fn geometric_seed(&self) -> Option<(f64, f64)> {
        let (event, _) = self.other.at(self.focus.t, self.focus.phi).ok()?;
        let frame = LocalFrame::new(self.metric, self.focus.r, self.focus.tetrad);
        let xi = frame.to_local(&[0.0, event.r - self.focus.r, event.phi - self.focus.phi]);
        let alpha = xi[2].atan2(xi[1]);
        let age = xi[1].hypot(xi[2]);
        // A guess that is not a number is no guess; and a pair at the same event has no direction
        // to offer, so the age is clamped the way `refine` clamps its own.
        (alpha.is_finite() && age.is_finite()).then_some((alpha, age.max(1e-6)))
    }

    /// The Kerr-Schild Cartesian separation of the two observers at the focus observer's own
    /// coordinate time: the age-zero sample of the sweep, which every generator of the fan shares
    /// because at age zero they are all still sitting on the focus event.
    fn separation_at_zero_age(&self) -> Option<f64> {
        self.probe(0.0, 0.0)
            .ok()
            .map(|probe| probe.separation(self.metric))
            .filter(|separation| separation.is_finite())
    }

    /// The first stride of the sweep, taken from the pair rather than from a constant: a third of
    /// the separation they actually have, capped at `SCAN_FIRST_STEP` and floored at
    /// `SCAN_MIN_STEP`, which is where the reasoning and the measurements are written down.
    ///
    /// A third is what a dip needs. The crossing of a close pair sits at an age of about their
    /// separation, and three samples have to fall on it with the middle one smallest, so the grid
    /// has to be finer than the crossing is near - a third puts samples at 0, s/3, (1 + 1.2) s / 3
    /// and so on, which brackets it.
    fn first_step(&self) -> f64 {
        match self.separation_at_zero_age() {
            Some(separation) => SCAN_FIRST_STEP.min(separation / 3.0).max(SCAN_MIN_STEP),
            None => SCAN_FIRST_STEP,
        }
    }

    /// The cold start: sweep the whole past cone backwards and hand back the `skip`-th coordinate
    /// time at which one of its generators came closest to the other's worldline.
    ///
    /// The cone's ring grows out of the focus event, so a given generator's separation from the
    /// other observer falls while the worldline is still outside the cone, is smallest as that
    /// generator sweeps past it, and grows again afterwards. A dip is therefore a candidate
    /// crossing, and the dips come in order of increasing age, which is the order of decreasing
    /// emission time: the direct image is the first of them that turns out to be a real crossing.
    ///
    /// **The dip has to be looked for on each ray separately, and that is the whole of this
    /// function.** Watching the minimum *over* the fan instead - which is what this did - loses the
    /// dip whenever some other generator happens to be closer at those particular sweeps, and it
    /// loses it exactly where it is most needed. A hovering watcher whose image of an infaller has
    /// piled up on r+ is the case: the generator that carries the image hugs the horizon while the
    /// rest of the cone sweeps out past the other observer's earlier, higher position, so the
    /// minimum over the fan never turns round at all, the sweep falls through to its global best,
    /// and the seed comes back at an age of 142 M for a crossing at 23 M. Newton cannot be expected
    /// to walk that, and it did not: the cold solve failed from about g = 1e-1 downwards while a
    /// warm one threaded from earlier frames solved the same configuration every time. Keeping one
    /// running triple of samples per ray costs three numbers a ray and finds the dip on the ray
    /// that made it.
    ///
    /// The age handed back is the vertex of the parabola through the three samples that bracket the
    /// dip, not the middle sample: the sweep is coarse by design and the vertex is free.
    ///
    /// The step grows geometrically up to `SCAN_MAX_STEP`. Almost every pair the app can make is a
    /// few M of light travel apart and is found in the first few sweeps; the growth is what lets
    /// the same bounded sweep still reach a hundred M without paying for a fine grid the whole way,
    /// and the ceiling is what stops the grid from growing past the width of the dip it is looking
    /// for.
    fn cold_seed(&self, skip: usize) -> Result<(f64, f64), NoImage> {
        let sweep = self.sweep(skip + 1, MAX_AGE)?;
        if let Some(dip) = sweep.dips.get(skip).copied() {
            return Ok(dip);
        }
        match sweep.closest {
            Some(closest) if skip == 0 && sweep.dips.is_empty() => Ok(closest),
            _ => Err(sweep.refusal),
        }
    }

    /// The sweep itself: the first `want` dips no older than `until_age`, in the order found,
    /// which is the order of increasing age. One pass of the fan serves every dip it finds, so a
    /// caller wanting the first few - `younger_image` - pays one sweep and not one per dip.
    fn sweep(&self, want: usize, until_age: f64) -> Result<Sweep, NoImage> {
        let alpha_of = |i: usize| TAU * (i as f64) / (SCAN_RAYS as f64);
        let mut dips: Vec<(f64, f64)> = Vec::new();
        let mut rays: Vec<Option<NullRay>> = (0..SCAN_RAYS)
            .map(|i| Some(self.launch(alpha_of(i))))
            .collect();
        // The two previous (age, separation) samples of each ray, oldest first. A sweep the ray
        // could not be probed at clears the pair, so a dip is only ever read off three consecutive
        // samples of the same generator.
        let mut recent: Vec<[Option<(f64, f64)>; 2]> = vec![[None; 2]; SCAN_RAYS];

        let mut age = 0.0f64;
        let mut step = self.first_step();
        // The closest approach seen anywhere in the sweep, as the last resort when no generator
        // ever turns round.
        let mut closest: Option<(f64, f64, usize)> = None;
        // Why the last sweep that found nothing found nothing, so that a search which never gets
        // started can say which of the two reasons it was.
        let mut refusal: Option<NoImage> = None;

        for _ in 0..SCAN_MAX_STEPS {
            let mut alive = 0;
            let mut probed = false;
            for (i, slot) in rays.iter().enumerate() {
                let Some(ray) = slot else { continue };
                alive += 1;
                let probe = match self.probe_with(*ray, age) {
                    Ok(probe) => probe,
                    Err(why) => {
                        refusal = Some(why);
                        recent[i] = [None; 2];
                        continue;
                    }
                };
                probed = true;
                let separation = probe.separation(self.metric);
                if closest.is_none_or(|(best, _, _)| separation < best) {
                    closest = Some((separation, age, i));
                }
                if let [Some(older), Some(middle)] = recent[i]
                    && middle.1 < older.1
                    && middle.1 < separation
                {
                    dips.push((alpha_of(i), parabolic_vertex(older, middle, (age, separation))));
                    if dips.len() >= want {
                        return Ok(Sweep { dips, closest: None, refusal: NoImage::NoCrossing });
                    }
                }
                recent[i] = [recent[i][1], Some((age, separation))];
            }
            if alive == 0 {
                if dips.is_empty() {
                    return Err(NoImage::RaysDied);
                }
                break;
            }
            if probed {
                refusal = None;
            } else if refusal == Some(NoImage::TrailEvicted) {
                // The trail's cap has swallowed the stretch being asked about, and going further
                // back only asks about a stretch further inside the gap.
                break;
            }
            // A sweep that found nothing because the emission time is still after the end of the
            // other's worldline is *early* rather than hopeless: the age only grows from here, so
            // the search carries on until it reaches events the worldline has.

            if age >= MAX_AGE || age >= until_age {
                break;
            }
            let dt = step.min(MAX_AGE - age);
            for slot in rays.iter_mut() {
                let Some(ray) = slot else { continue };
                if ray.step_back(self.metric, dt) == RayStep::BudgetExhausted
                    || !ray.alive()
                    || ray.r >= self.ceiling
                    || !ray.r.is_finite()
                {
                    *slot = None;
                }
            }
            age += dt;
            step = (step * SCAN_GROWTH).min(SCAN_MAX_STEP);
        }

        // Fewer dips than were asked for. If the sweep still found a closest approach, hand that
        // over too - a refinement that fails from there costs one solve and says so - and
        // otherwise there is nothing more to refine. Only the first candidate may be answered
        // this way: the fallback is one point and not a list, so asking for a later one is asking
        // for something that was never there, which `cold_seed` enforces.
        let closest = closest
            .filter(|(_, found_age, _)| *found_age > 0.0)
            .map(|(_, found_age, found_ray)| (alpha_of(found_ray), found_age));
        let refusal = refusal.unwrap_or(NoImage::NoCrossing);
        if dips.is_empty() && closest.is_none() {
            return Err(refusal);
        }
        Ok(Sweep { dips, closest, refusal })
    }

    /// Newton on the two unknowns (arrival angle, age), from a seed.
    ///
    /// The residual is the coordinate mismatch (delta r, delta phi) between the ray's event and
    /// the other observer's event *at the same coordinate time*, which is the honest statement of
    /// "the cone passes through the worldline": both curves are parameterised by t already, so the
    /// crossing is two equations in two unknowns and nothing has to be reparameterised.
    ///
    /// One column of the Jacobian is exact and free. Increasing the age moves both events back
    /// along their own curves, so
    ///
    ///     d(delta r)/d(age) = -(dr/dt along the ray - u^r/u^t along the worldline),
    ///
    /// and likewise in phi, out of numbers both curves are already carrying. The other column
    /// needs a second ray shot from the observer's event, which is what a Newton iteration costs
    /// here: two backward integrations.
    ///
    /// The step is damped - half the current age, and half a radian of angle - because a Newton
    /// step taken on a badly conditioned Jacobian can otherwise ask for a negative age or throw
    /// the direction round the far side of the hole, and the cost of a wasted iteration is another
    /// pair of ray integrations.
    ///
    /// Damping alone is not enough from a cold seed, so the step is also *backtracked*: it is
    /// halved until it lands somewhere the separation is smaller than it was. Two things make that
    /// necessary rather than tidy. A full Newton step from a seed the sweep only bracketed to four
    /// M can walk clean past the end of the other observer's recorded worldline, where the probe
    /// has nothing to answer with - and aborting there used to throw away a solve that a shorter
    /// step would have got. And where the emission event is piled up against r+ the residual is a
    /// long shallow valley in the age, so an undamped step overshoots it and the iteration walks
    /// away instead of in. Insisting on descent makes the refinement monotone, which is what lets
    /// the cold path reach the same answers a warm one does.
    fn refine(&self, alpha0: f64, age0: f64, cold: bool) -> Result<AsSeen, NoImage> {
        let mut alpha = alpha0;
        let mut age = age0.clamp(1e-9, MAX_AGE);
        let mut probe = self.probe(alpha, age)?;
        let mut separation = probe.separation(self.metric);

        for iteration in 0..MAX_NEWTON {
            if separation <= NEWTON_TOL {
                return self.finish(probe, alpha, separation, iteration, cold);
            }

            // The age column, exact.
            let u = probe.event.u;
            let d_age = [
                -(probe.ray.dr_dt - u[1] / u[0]),
                -(probe.ray.dphi_dt - u[2] / u[0]),
            ];
            // The angle column, by a finite difference that costs one more ray. Whichever side of
            // the current angle can be evaluated will do: near the edge of the fan's live arc one
            // of the two neighbours is a ray that has already died at the ring.
            let Some(d_alpha) = self.angle_column(&probe, alpha, age) else {
                break;
            };

            let det = d_alpha[0] * d_age[1] - d_alpha[1] * d_age[0];
            if det == 0.0 || !det.is_finite() {
                break;
            }
            let (g0, g1) = (probe.gap[0], probe.gap[1]);
            let full_alpha = (-(g0 * d_age[1] - g1 * d_age[0]) / det).clamp(-0.5, 0.5);
            // The age may move by half of itself, or by the current separation if that is more:
            // light covers a separation in about that much coordinate time, so a seed at a
            // near-zero age can still step out to the crossing in one go. The floor used to be a
            // fixed 1e-3 M, which is five times the age of the direct image of an observer
            // 2e-4 M away - two observers freezing onto r- together - so every step overshot
            // it and the Newton ended 3e-4 M off, twice the distance to the image, on every frame.
            let cap_age = (0.5 * age).max(separation).max(1e-9);
            let full_age = (-(d_alpha[0] * g1 - d_alpha[1] * g0) / det).clamp(-cap_age, cap_age);

            let mut scale = 1.0;
            let mut moved = false;
            for _ in 0..MAX_BACKTRACKS {
                let trial_alpha = alpha + scale * full_alpha;
                let trial_age = (age + scale * full_age).clamp(1e-9, MAX_AGE);
                if let Ok(trial) = self.probe(trial_alpha, trial_age) {
                    let closer = trial.separation(self.metric);
                    if closer < separation {
                        alpha = trial_alpha;
                        age = trial_age;
                        probe = trial;
                        separation = closer;
                        moved = true;
                        break;
                    }
                }
                scale *= 0.5;
            }
            if !moved {
                break;
            }
        }

        // Out of iterations, or out of descent. A solve that got within a hair is still a picture -
        // the residual is reported so a caller can decide - and the hair is measured against the
        // image's own distance, never in absolute terms: see `NEAR_MISS_OF_LAMBDA`.
        let found = self.finish(probe, alpha, separation, MAX_NEWTON, cold)?;
        if separation <= NEAR_MISS_OF_LAMBDA * found.lambda {
            Ok(found)
        } else {
            Err(NoImage::NotConverged)
        }
    }

    /// d(gap)/d(alpha) at a probed point, by a one-sided finite difference taken on whichever side
    /// of the arrival angle can be evaluated. `None` when neither can, which means the refinement
    /// has walked onto an angle whose neighbouring rays both die before they reach the worldline.
    fn angle_column(&self, probe: &Probe, alpha: f64, age: f64) -> Option<[f64; 2]> {
        let h = self.alpha_step(alpha);
        if let Ok(ahead) = self.probe(alpha + h, age) {
            return Some([(ahead.gap[0] - probe.gap[0]) / h, (ahead.gap[1] - probe.gap[1]) / h]);
        }
        let behind = self.probe(alpha - h, age).ok()?;
        Some([(probe.gap[0] - behind.gap[0]) / h, (probe.gap[1] - behind.gap[1]) / h])
    }

    /// The finite-difference step in the arrival angle, sized so that the launched ray's
    /// coordinate velocity moves by about `ALPHA_STEP`.
    ///
    /// The angle is measured in the focus observer's own frame, and that frame can be boosted
    /// out of all proportion: an observer freezing onto the far branch of r- has u^t in the
    /// hundreds of thousands, and aberration then folds almost the whole of their sky into one
    /// direction, so a fixed step of 1e-6 in the angle moves the ray's actual direction by 1e-12
    /// on the folded side and the difference of two residuals over it is integration noise. The
    /// Newton then walks on a derivative that means nothing and stops descending a hair short -
    /// measured at u^t = 4.5e5 with Alice's image steady at g = 0.028, it gave up at separations
    /// of 1e-5 to 1e-6 on forty of sixty frames. Sizing the step by what it does to the ray
    /// keeps the column honest whatever the boost, and is the old step exactly for a frame that
    /// is not boosted at all.
    fn alpha_step(&self, alpha: f64) -> f64 {
        let here = self.launch(alpha);
        let there = self.launch(alpha + ALPHA_STEP);
        let r = self.focus.r;
        let slope = (there.dr_dt - here.dr_dt).hypot(r * (there.dphi_dt - here.dphi_dt)) / ALPHA_STEP;
        // The negation is the point rather than a way of writing <=: a slope that has come out
        // NaN has to take this branch too.
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        if !(slope > 0.0) || !slope.is_finite() {
            return ALPHA_STEP;
        }
        (ALPHA_STEP / slope).clamp(ALPHA_STEP_MIN, ALPHA_STEP_MAX)
    }

    /// Turn a converged probe into the answer: the affine length of the ray and the frequency
    /// ratio it carries.
    fn finish(
        &self,
        probe: Probe,
        alpha: f64,
        residual: f64,
        iterations: u32,
        cold: bool,
    ) -> Result<AsSeen, NoImage> {
        let lambda = self.affine_length(&probe, alpha).ok_or(NoImage::NotConverged)?;
        // nu(here) / nu(there) is the reciprocal of what `frequency_ratio` answers, because the
        // ray was *emitted* at the focus event by the solver and *received* at the emission event:
        // the two ends are the same two events either way round, and `f_emit` is the factor at the
        // focus.
        let ratio = probe.ray.frequency_ratio(self.metric, &probe.event.u);
        let g = if ratio.is_finite() && ratio != 0.0 {
            1.0 / ratio
        } else {
            return Err(NoImage::NotConverged);
        };

        // The solve drives wrap(ray.phi - event.phi) to nothing, so what is left between the two
        // is a whole number of turns: the ray's own azimuthal excursion against the direct one.
        let windings = ((probe.ray.phi - probe.event.phi) / TAU).round();
        let (sin_a, cos_a) = alpha.sin_cos();
        Ok(AsSeen {
            age: self.focus.t - probe.event.t,
            emission: probe.event,
            lambda,
            n: [cos_a, sin_a],
            xi: [-lambda, lambda * cos_a, lambda * sin_a],
            g,
            alpha,
            source: probe.source,
            residual,
            iterations,
            cold,
            windings: windings as i32,
        })
    }

    /// The affine length of the solved ray, with the tangent normalised so that -K . u_focus = 1.
    ///
    /// The ray is carried as a direction v = K / K^t and has thrown its scale away, but the scale
    /// is conserved and can be put back from the one end where it is known: E_K = -K_t at the
    /// focus event, where K is `null_direction(alpha + pi)` itself. With that in hand the affine
    /// length is the radial quadrature of `normal_coords` for a null geodesic (mu2 = 0) with this
    /// ray's own (E, L) - a closed form in r that needs no step history and is exact across both
    /// horizons.
    ///
    /// A ray that turned round in r between the two events is integrated in two legs. The turning
    /// radius is a root of the same cubic, found in closed form: the radial potential of a null
    /// geodesic has no quadratic term, so it is unimodal on r > 0 and its minimum sits at
    /// sqrt(-B / 3A), which brackets the root that matters.
    fn affine_length(&self, probe: &Probe, alpha: f64) -> Option<f64> {
        let k = self.focus.tetrad.null_direction(alpha + PI);
        let constants = RadialConstants::of_tangent(self.metric, self.focus.r, &k);
        let (r_a, r_b) = (probe.event.r, self.focus.r);
        if (r_a - r_b).abs() < 1e-13 && probe.ray.dr_dt * dr_dt_of(&k) > 0.0 {
            // The two events are at the same radius and the ray never turned: it went round in
            // azimuth at fixed r, which only happens on a circular photon orbit. The quadrature
            // has nothing to integrate, and the stepped length is the honest answer.
            return Some(self.affine_length_by_stepping(alpha, self.focus.t - probe.event.t));
        }
        // Whether the ray reversed in r between the two ends, as the sign of dr/dt at each. The
        // ray runs from the emission event to the focus, so `leaving` is its radial rate as it
        // left the other observer and `arriving` is the rate it has when it gets here: a ray that
        // left going outward and arrives going inward turned at a radius *above* both events, and
        // one that left going inward and arrives going outward turned below them.
        let leaving = probe.ray.dr_dt;
        let arriving = dr_dt_of(&k);
        if leaving * arriving >= 0.0 {
            return affine_length_between(self.metric, r_a, r_b, &constants);
        }
        let outward = leaving > 0.0;
        let turn = null_turning_radius(self.metric, &constants, outward, r_a.min(r_b), r_a.max(r_b))?;
        let first = affine_length_between(self.metric, r_b, turn, &constants)?;
        let second = affine_length_between(self.metric, turn, r_a, &constants)?;
        Some(first + second)
    }

    /// The same affine length, accumulated along the ray as the integral of dt / K^t rather than
    /// taken from the closed form in r.
    ///
    /// K^t = E_K / e_v with e_v = -g_{t mu} v^mu, which is the first of `NullRay::constants`, so
    /// d lambda = e_v dt / E_K. It is here as the independent check on `affine_length` - the two
    /// are different integrals of different variables along the same curve - and it is the one
    /// that still answers where the quadrature cannot, which is a ray with a turning point the
    /// root finder could not bracket.
    fn affine_length_by_stepping(&self, alpha: f64, age: f64) -> f64 {
        let k = self.focus.tetrad.null_direction(alpha + PI);
        let g = self.metric.metric_components(self.focus.r);
        let energy = -(g[0][0] * k[0] + g[0][1] * k[1] + g[0][2] * k[2]);
        let mut ray = self.launch(alpha);
        let steps = 4096;
        let dt = age / steps as f64;
        let e_v = |ray: &NullRay| ray.constants(self.metric).0;
        let mut previous = e_v(&ray);
        let mut total = 0.0;
        for _ in 0..steps {
            if ray.step_back(self.metric, dt) == RayStep::BudgetExhausted || !ray.alive() {
                break;
            }
            let now = e_v(&ray);
            total += 0.5 * (previous + now) * dt;
            previous = now;
        }
        total / energy
    }
}

/// dr/dt of a coordinate tangent.
fn dr_dt_of(k: &[f64; 3]) -> f64 {
    k[1] / k[0]
}

/// Where the parabola through three (age, separation) samples has its minimum, clamped to the
/// interval the three samples span.
///
/// The cold sweep brackets a dip with three samples that can be four M of coordinate time apart,
/// and the middle one is the best it can say without this. The vertex is a couple of multiplies
/// and typically lands the seed inside a fraction of an M, which is several Newton iterations
/// saved on every cold solve and the difference between converging and not on a shallow one. A
/// degenerate triple - three collinear samples, or a denominator that has cancelled to nothing -
/// falls back to the middle sample, which is the answer this replaced.
fn parabolic_vertex(older: (f64, f64), middle: (f64, f64), newer: (f64, f64)) -> f64 {
    let (x0, f0) = older;
    let (x1, f1) = middle;
    let (x2, f2) = newer;
    let (d0, d2) = (x1 - x0, x1 - x2);
    let (g0, g2) = (f1 - f2, f1 - f0);
    let denominator = d0 * g0 - d2 * g2;
    if denominator == 0.0 || !denominator.is_finite() {
        return x1;
    }
    let vertex = x1 - 0.5 * (d0 * d0 * g0 - d2 * d2 * g2) / denominator;
    if vertex.is_finite() {
        vertex.clamp(x0.min(x2), x0.max(x2))
    } else {
        x1
    }
}

/// The radius at which a null geodesic with these constants turns round, bracketed between the
/// stretch the ray covered and the minimum of its own radial potential.
///
/// The potential is R(r) = r c(r) with c(r) = E^2 r^3 + (a^2 E^2 - L^2) r + 2M (L - aE)^2: a cubic
/// with no quadratic term, so c' = 3 E^2 r^2 + (a^2 E^2 - L^2) vanishes at most once on r > 0 and
/// c is unimodal there. A ray that turns has c < 0 at that minimum, and the two roots either side
/// of it bound the two regions the motion can live in - so an outward turn is the *lower* root,
/// reached from below, and an inward turn the upper one. Both are bracketed by the minimum and the
/// end of the stretch the ray covered, and bisection on a monotone piece does the rest.
fn null_turning_radius(
    metric: &KerrSchild,
    k: &RadialConstants,
    outward: bool,
    lo: f64,
    hi: f64,
) -> Option<f64> {
    let a = metric.a;
    let big_a = k.energy * k.energy;
    let big_b = a * a * big_a - k.l_ang * k.l_ang;
    if big_a <= 0.0 || big_b >= 0.0 {
        return None;
    }
    let r_star = (-big_b / (3.0 * big_a)).sqrt();
    let c = |r: f64| k.potential(metric, r) / r;
    if c(r_star).is_nan() || c(r_star) >= 0.0 {
        return None;
    }
    // The root sits between the end of the covered stretch and the minimum: outward turns above
    // `hi`, inward turns below `lo`.
    let (mut near, mut far) = if outward { (hi, r_star) } else { (lo, r_star) };
    let c_near = c(near);
    if c_near.is_nan() || c_near < 0.0 || (outward && r_star <= hi) || (!outward && r_star >= lo) {
        return None;
    }
    for _ in 0..200 {
        let mid = 0.5 * (near + far);
        if c(mid) >= 0.0 {
            near = mid;
        } else {
            far = mid;
        }
    }
    Some(0.5 * (near + far))
}

/// An angle folded into (-pi, pi].
fn wrap(angle: f64) -> f64 {
    let turned = angle.rem_euclid(TAU);
    if turned > PI { turned - TAU } else { turned }
}

impl<'a> OtherWorldline<'a> {
    fn new(metric: &'a KerrSchild, obs: &'a Observer) -> Self {
        Self {
            metric,
            obs,
            mode: obs.effective_mode(metric),
            pre_run: RefCell::new(VecDeque::new()),
        }
    }

    /// The other observer's event at coordinate time `t`, with the azimuth unwrapped so that it is
    /// within half a turn of `phi_reference`.
    ///
    /// Four ways of answering, and which one applies is the same case split `Observer::advance`
    /// and `Observer::rewind_to` make, because the worldline this reads has to be the worldline
    /// those two draw:
    ///
    /// * Before the release, the hold, in closed form and extended as far back as asked. This is
    ///   the whole of the answer at t = 0, when no light from the other observer has arrived yet
    ///   and the emission event is necessarily before the run began.
    /// * A Static or ZAMO worldline is carried at a constant 4-velocity at fixed r, so it is
    ///   carried back by subtracting exactly what a forward step of the same interval would add -
    ///   which is what `rewind_to` does for those modes.
    /// * A dragged marker is not integrated at all: `advance` moves its t and nothing else, so
    ///   this holds it where the user put it and flags the answer `Dragged`.
    /// * Otherwise the recorded trail, interpolated. A trail that no longer reaches back far
    ///   enough says so rather than extrapolating.
    fn at(&self, t: f64, phi_reference: f64) -> Result<(SeenEvent, WorldlineSource), NoImage> {
        let (point, source) = self.point_at(t)?;
        let phi = phi_reference + wrap(point.phi - phi_reference);
        Ok((
            SeenEvent {
                t: point.t,
                r: point.r,
                phi,
                tau: point.tau,
                u: point.u,
            },
            source,
        ))
    }

    fn point_at(&self, t: f64) -> Result<(TrailPoint, WorldlineSource), NoImage> {
        if t < self.obs.release_t {
            if let Some(point) = self.pre_run_at(t) {
                return Ok((point, WorldlineSource::Geodesic));
            }
            return Ok((self.obs.hold_event_at(self.metric, t), WorldlineSource::Hold));
        }
        // Nothing is known about the other's worldline past the clock they have been stepped to,
        // and for a worldline that has ended there is nothing to know.
        if t > self.obs.t + 1e-9 {
            return Err(NoImage::OtherEnded);
        }
        match self.mode {
            ObserverMode::ManualDrag => Ok((
                TrailPoint {
                    t,
                    r: self.obs.r,
                    phi: self.obs.phi,
                    tau: self.obs.tau,
                    u: self.obs.four_velocity(self.metric),
                    stalled: false,
                },
                WorldlineSource::Dragged,
            )),
            ObserverMode::Static | ObserverMode::Zamo => {
                let u = self.obs.four_velocity(self.metric);
                let back = self.obs.t - t;
                let ut = u[0].max(1e-9);
                Ok((
                    TrailPoint {
                        t,
                        r: self.obs.r,
                        phi: self.obs.phi - (u[2] / ut) * back,
                        tau: self.obs.tau - back / ut,
                        u,
                        stalled: false,
                    },
                    WorldlineSource::FixedRadius,
                ))
            }
            ObserverMode::FreeFall => {
                let trail = &self.obs.trail;
                let oldest = trail.front().ok_or(NoImage::TrailEvicted)?;
                if t < oldest.t - 1e-9 {
                    return Err(NoImage::TrailEvicted);
                }
                let after = trail.partition_point(|p| p.t <= t);
                if after == 0 {
                    return Ok((*oldest, WorldlineSource::Trail));
                }
                let a = trail[after - 1];
                let Some(&b) = trail.get(after) else {
                    return Ok((a, WorldlineSource::Trail));
                };
                Ok((self.between(&a, &b, t), WorldlineSource::Trail))
            }
        }
    }

    /// The event at `t` on the free-fall worldline continued backwards through the release event,
    /// or `None` when the stretch before the release is a hold rather than this geodesic.
    ///
    /// It is this geodesic exactly when the observer was released the moment they were created
    /// and the release arrives already moving. A release from rest is joined by the hold without
    /// a jump, so the hold is the honest history there; a fixed-radius worldline is its own
    /// history; and an observer who held station for a while before release really did hold, in
    /// the run as drawn, so the hold is read for that stretch too. What is left is the case the
    /// user's starting files are built on - "from rest at infinity", released at once - and for
    /// that the only worldline through the release event with the release's own 4-velocity is
    /// the geodesic, so it is integrated backwards, in proper time, with the same RK4 the forward
    /// run uses, from the release event. The integration is done once per solve and lazily: as
    /// far back as the solve asks, one step further each time, and the events are kept so that
    /// the next question about an earlier time starts from where the last one stopped.
    ///
    /// The step is a twentieth of a M of proper time near the hole, and grows with the radius
    /// because the field does not: a raindrop asked about 400 M of coordinate time ago has
    /// climbed to tens of M and needs no finer sampling there than the forward run gives it.
    fn pre_run_at(&self, t: f64) -> Option<TrailPoint> {
        if self.mode != ObserverMode::FreeFall {
            return None;
        }
        let start = self.obs.start;
        if self.obs.release_t > start.t + 1e-12 {
            return None;
        }
        let moving = start.u[1].abs() > 1e-6 * (1.0 + start.u[0].abs());
        if !moving {
            return None;
        }
        let geodesic = self.obs.geodesic.as_ref()?;
        let mut pre_run = self.pre_run.borrow_mut();
        if pre_run.is_empty() {
            // Anchored on the release event, which is the observer's current event while the run
            // has not started. The two differ only in a file saved by a build whose marker drag
            // left `start` behind (see `Observer::release_from_drag`), and the picture is drawn
            // from the current event, so that is the one the worldline is continued from.
            let anchor = if self.obs.t <= start.t + 1e-12 {
                TrailPoint {
                    t: self.obs.t,
                    r: self.obs.r,
                    phi: self.obs.phi,
                    tau: self.obs.tau,
                    u: geodesic.u,
                    stalled: false,
                }
            } else {
                start
            };
            pre_run.push_back(anchor);
        }
        let mut state = {
            let oldest = pre_run.front().expect("seeded just above");
            GeodesicState {
                t: oldest.t,
                r: oldest.r,
                phi: oldest.phi,
                tau: oldest.tau,
                energy: geodesic.energy,
                l_ang: geodesic.l_ang,
                u: oldest.u,
                stalled: false,
            }
        };
        // A worldline traced into the past cannot stall on r- - that is in its future - but it
        // can come *out of* the ring for a run that began inside r-, and there is nothing to read
        // before that.
        while state.t > t {
            if state.r <= R_STOP || !state.r.is_finite() {
                return None;
            }
            let dtau = -0.05 * (1.0 + state.r / 10.0);
            state.step(self.metric, dtau);
            pre_run.push_front(TrailPoint {
                t: state.t,
                r: state.r,
                phi: state.phi,
                tau: state.tau,
                u: state.u,
                stalled: false,
            });
        }
        // The pair of recorded events that bracket t, oldest first, interpolated the way the trail
        // itself is.
        let after = pre_run.partition_point(|p| p.t <= t);
        let a = pre_run.get(after.checked_sub(1)?)?;
        let Some(b) = pre_run.get(after) else {
            return Some(*a);
        };
        Some(self.between(a, b, t))
    }

    /// The event at `t` between two recorded ones, by cubic Hermite interpolation on the stored
    /// 4-velocity.
    ///
    /// The trail records u as well as the position, so the worldline is known to first order at
    /// both ends of every gap and a cubic Hermite is the natural interpolant: it matches position
    /// and velocity at both ends and is third-order accurate in the gap, which at one recorded
    /// event per played frame is far below anything a root find can see. A linear interpolation
    /// would not do - the emission event feeds a two-point solve whose residual is driven to 1e-9,
    /// and a kink in the curve being solved against is a kink in the answer.
    ///
    /// Two things are handled rather than assumed. The azimuth is recorded folded into [0, 2 pi),
    /// so the end of the gap is first unwrapped against the start using the mean rate the two ends
    /// carry; averaging 6.28 and 0.01 would otherwise put the observer on the opposite side of the
    /// hole. And the interpolated 4-velocity is rescaled to be exactly unit timelike, which is a
    /// projection back onto the mass shell rather than a correction: the interpolant is exact at
    /// both ends, so the rescaling does nothing there and only removes the interpolation's own
    /// second-order drift in between.
    ///
    /// A worldline frozen on the far branch of r- is carried, not interpolated. `advance` holds
    /// its r and its watch and slides its azimuth along the horizon's own generators at Omega_-,
    /// and that is what is reproduced here; interpolating a u^t of 1e10 would be interpolating
    /// arithmetic rather than a worldline.
    fn between(&self, a: &TrailPoint, b: &TrailPoint, t: f64) -> TrailPoint {
        let span = b.t - a.t;
        if span <= 0.0 || !span.is_finite() {
            return *a;
        }
        if a.stalled {
            return TrailPoint {
                t,
                r: a.r,
                phi: a.phi + self.metric.inner_horizon_omega() * (t - a.t),
                tau: a.tau,
                u: a.u,
                stalled: true,
            };
        }
        let s = ((t - a.t) / span).clamp(0.0, 1.0);
        let (s2, s3) = (s * s, s * s * s);
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = s3 - 2.0 * s2 + s;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = s3 - s2;
        let hermite = |ya: f64, da: f64, yb: f64, db: f64| {
            h00 * ya + h10 * span * da + h01 * yb + h11 * span * db
        };

        let (ra, rb) = (a.u[1] / a.u[0], b.u[1] / b.u[0]);
        let (pa, pb) = (a.u[2] / a.u[0], b.u[2] / b.u[0]);
        // Unwrap b's azimuth onto the branch the mean rate predicts.
        let predicted = a.phi + 0.5 * (pa + pb) * span;
        let phi_b = b.phi + TAU * ((predicted - b.phi) / TAU).round();

        let r = hermite(a.r, ra, b.r, rb).max(R_STOP * 0.5);
        let phi = hermite(a.phi, pa, phi_b, pb);
        let tau = hermite(a.tau, 1.0 / a.u[0], b.tau, 1.0 / b.u[0]);

        // du^mu/dt = (-Gamma^mu_{alpha beta} u^alpha u^beta) / u^t along a geodesic.
        let da = geodesic_accel(self.metric, a.r, &a.u);
        let db = geodesic_accel(self.metric, b.r, &b.u);
        let mut u: [f64; 3] = core::array::from_fn(|mu| {
            hermite(a.u[mu], da[mu] / a.u[0], b.u[mu], db[mu] / b.u[0])
        });
        let norm = self.metric.norm(r, &u);
        if norm < 0.0 {
            let scale = 1.0 / (-norm).sqrt();
            for c in u.iter_mut() {
                *c *= scale;
            }
        }

        TrailPoint {
            t,
            r,
            phi,
            tau,
            u,
            stalled: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::observer::{Release, WorldlineParams};
    use crate::physics::tetrad::inner;
    use kerr_equatorial::normal_coords::affine_length_to_surface;

    /// An observer holding station at radius `r`, azimuth `phi`, with the clock at zero. The
    /// worldline data is the at-rest release there, so the hold, the 4-velocity and the backward
    /// extension are all the static observer and agree with one another by construction.
    fn hovering(metric: &KerrSchild, name: &str, r: f64, phi: f64) -> Observer {
        let mut obs = Observer::new_with_phi(
            metric,
            name,
            0.0,
            r,
            0.0,
            phi,
            WorldlineParams::released(metric, r, 0.0, Release::AtRest),
        );
        obs.mode = ObserverMode::Static;
        obs.is_active = true;
        obs
    }

    /// An observer still waiting to be let go: holding the radius they were created at, on the
    /// worldline their at-rest release will begin on. This is what the app has before t reaches
    /// `release_t`, and its 4-velocity is `Observer::hover_four_velocity`'s - the zero-angular-
    /// momentum hover rather than the static observer, since at rest in r is not at rest in phi
    /// where the frame is dragged.
    fn held(metric: &KerrSchild, name: &str, r: f64, phi: f64) -> Observer {
        Observer::new_with_phi(
            metric,
            name,
            0.0,
            r,
            1e9,
            phi,
            WorldlineParams::released(metric, r, 0.0, Release::AtRest),
        )
    }

    /// A raindrop released at radius `r`: E = 1, L = 0, already moving when it gets there.
    fn raindrop(metric: &KerrSchild, name: &str, r: f64, phi: f64) -> Observer {
        Observer::new_with_phi(
            metric,
            name,
            0.0,
            r,
            0.0,
            phi,
            WorldlineParams::new(1.0, 0.0, false),
        )
    }

    /// Step both observers together on the one simulation clock, exactly as `Simulation` does.
    fn play(metric: &KerrSchild, a: &mut Observer, b: &mut Observer, steps: usize, dt: f64) {
        let mut t = a.t.max(b.t);
        for _ in 0..steps {
            t += dt;
            a.step(metric, t, dt);
            b.step(metric, t, dt);
        }
    }

    /// The direction T = c0 e0 + c1 e1 of an observer's axial tetrad, in coordinate components.
    fn leg(metric: &KerrSchild, r: f64, u: &[f64; 3], c0: f64, c1: f64) -> [f64; 3] {
        let t = Tetrad::from_four_velocity_axial(metric, r, u);
        [
            c0 * t.e0[0] + c1 * t.e1[0],
            c0 * t.e0[1] + c1 * t.e1[1],
            c0 * t.e0[2] + c1 * t.e1[2],
        ]
    }

    #[test]
    fn test_two_hovering_observers_see_each_other_at_the_closed_form_delay_and_shift() {
        // Everything about this case can be got with a pencil, which is why it is the first test.
        // Two static observers of a hole with no spin, at the same azimuth, so the light between
        // them is radial.
        //
        // Looking *outward*, the light is ingoing, and in ingoing Kerr-Schild coordinates an
        // ingoing radial null ray obeys dt/dr = -1 exactly at every radius - that is the defining
        // property of the chart - so the delay is the coordinate gap itself. Its affine length is
        // elementary for the same reason: the tangent the observer's own frame puts on that ray is
        // -e0 + e1 = sqrt(1 - 2M/r0) (-1, 1, 0), so r is affine along it at that rate and
        // lambda = Delta r / sqrt(1 - 2M/r0).
        //
        // Looking *inward*, the light is outgoing and has to climb, obeying
        // dt/dr = (r + 2M)/(r - 2M) = 1 + 4M/(r - 2M), which integrates to
        // Delta r + 4M ln[(r_A - 2M)/(r_B - 2M)].
        //
        // The shift is the ratio of the two hovering clocks either way:
        // g = sqrt[(1 - 2M/r_emit)/(1 - 2M/r_see)].
        let metric = KerrSchild::new(1.0, 0.0);
        let focus = hovering(&metric, "Alice", 6.0, 0.0);

        // Outward: the other is above, the light falls in.
        let other = hovering(&metric, "Bob", 10.0, 0.0);
        let seen = as_seen(&metric, &focus, &other, None).expect("a hovering pair see each other");
        let expected_age = 10.0 - 6.0;
        let expected_g = ((1.0 - 2.0 / 10.0) / (1.0 - 2.0 / 6.0f64)).sqrt();
        let expected_lambda = 4.0 / (1.0 - 2.0 / 6.0f64).sqrt();
        println!(
            "static 6 M looking at static 10 M: age {:.9} M (closed form {expected_age}), \
             lambda {:.9} M (closed form {expected_lambda:.9}), g {:.9} (closed form \
             {expected_g:.9}), n = {:?}, residual {:.2e}",
            seen.age, seen.lambda, seen.g, seen.n, seen.residual
        );
        assert!((seen.age - expected_age).abs() < 1e-7, "delay {}", seen.age);
        assert!((seen.emission.r - 10.0).abs() < 1e-7, "seen radius {}", seen.emission.r);
        assert!((seen.g - expected_g).abs() < 1e-7, "g = {}", seen.g);
        assert!((seen.lambda - expected_lambda).abs() < 1e-6, "lambda = {}", seen.lambda);
        assert!(
            (seen.n[0] - 1.0).abs() < 1e-6 && seen.n[1].abs() < 1e-6,
            "the source is straight outward: n = {:?}",
            seen.n
        );
        // The drawn point reaches the 45-degree past cone, because this light arrives along the
        // radial leg the view draws and the projection onto that plane therefore drops nothing.
        // That lambda is at once the time and the distance is what normal coordinates make of a
        // null geodesic.
        let xi = seen.xi_projected();
        assert!(
            (xi[1] + seen.lambda).abs() < 1e-12 && (xi[0] - seen.lambda).abs() < 1e-6,
            "xi = {xi:?} against lambda = {}",
            seen.lambda
        );
        assert_eq!(seen.source, WorldlineSource::Hold, "the light left before the run began");
        // And lambda is the same number the normal-coordinate quadrature gives for the direction
        // the light came from, which is the other half of the construction meeting the first.
        let u = focus.four_velocity(&metric);
        let along = leg(&metric, focus.r, &u, -1.0, seen.n[0]);
        let quadrature = affine_length_to_surface(&metric, focus.r, &along, 10.0)
            .expect("the past cone reaches r = 10");
        assert!(
            (seen.lambda - quadrature).abs() < 1e-6,
            "lambda {} vs the surface quadrature {quadrature}",
            seen.lambda
        );

        // Inward: the other is below, the light has to climb out.
        let other = hovering(&metric, "Bob", 4.0, 0.0);
        let seen = as_seen(&metric, &focus, &other, None).expect("the pair still see each other");
        let expected_age = 2.0 + 4.0 * ((6.0 - 2.0) / (4.0 - 2.0f64)).ln();
        let expected_g = ((1.0 - 2.0 / 4.0) / (1.0 - 2.0 / 6.0f64)).sqrt();
        let along = leg(&metric, focus.r, &u, -1.0, -1.0);
        let quadrature =
            affine_length_to_surface(&metric, focus.r, &along, 4.0).expect("and reaches r = 4");
        println!(
            "static 6 M looking at static 4 M: age {:.9} M (closed form {expected_age:.9}), \
             lambda {:.9} M (quadrature {quadrature:.9}), g {:.9} (closed form {expected_g:.9})",
            seen.age, seen.lambda, seen.g
        );
        assert!((seen.age - expected_age).abs() < 1e-6, "delay {}", seen.age);
        assert!((seen.g - expected_g).abs() < 1e-7, "g = {} (a redshift)", seen.g);
        assert!(seen.g < 1.0, "light climbing out must be redshifted: {}", seen.g);
        assert!(
            (seen.n[0] + 1.0).abs() < 1e-6 && seen.n[1].abs() < 1e-6,
            "the source is straight inward: n = {:?}",
            seen.n
        );
        assert!(
            (seen.lambda - quadrature).abs() < 1e-6,
            "lambda {} vs the surface quadrature {quadrature}",
            seen.lambda
        );
    }

    #[test]
    fn test_a_hovering_watcher_never_sees_the_infaller_cross() {
        // The picture everybody knows, as numbers. A static observer at 8 M watches a raindrop
        // fall past. The seen radius falls towards r+ and never reaches it, the shift falls
        // towards zero, and the affine distance to what is seen tends to the distance to the
        // horizon *down the observer's own past light cone* - which is a number the surface
        // quadrature gives independently: Delta r / sqrt(1 - 2M/r0) = 6 / sqrt(3/4) = 6.928203 M.
        //
        // The last of those three is the one worth having. The image does not recede without
        // limit; it piles up on a definite place in the observer's own frame, and that place is
        // where the inward half of their past cone meets r+.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut focus = hovering(&metric, "Alice", 8.0, 0.0);
        let mut other = raindrop(&metric, "Bob", 6.0, 0.0);

        let horizon_cone = affine_length_to_surface(
            &metric,
            8.0,
            &leg(&metric, 8.0, &focus.four_velocity(&metric), -1.0, -1.0),
            2.0,
        )
        .expect("the inward past cone reaches r+");
        assert!(
            (horizon_cone - 6.0 / 0.75f64.sqrt()).abs() < 1e-9,
            "the closed form is 6 / sqrt(3/4): {horizon_cone}"
        );

        let mut seed = None;
        let mut previous: Option<AsSeen> = None;
        let mut samples = Vec::new();
        for _ in 0..120 {
            play(&metric, &mut focus, &mut other, 5, 0.1);
            let seen = as_seen(&metric, &focus, &other, seed).expect("the image never goes out");
            seed = Some(seen.seed());
            if let Some(before) = previous {
                assert!(
                    seen.emission.r < before.emission.r + 1e-9,
                    "the seen radius must fall: {} after {}",
                    seen.emission.r,
                    before.emission.r
                );
                assert!(
                    seen.g < before.g + 1e-9,
                    "the shift must fall: {} after {}",
                    seen.g,
                    before.g
                );
            }
            assert!(
                seen.emission.r > 2.0,
                "the crossing is never seen: {}",
                seen.emission.r
            );
            samples.push((focus.t, seen));
            previous = Some(seen);
        }

        let (t_last, last) = samples[samples.len() - 1];
        println!(
            "at t = {t_last:.1} M the hovering watcher sees the raindrop at r = {:.6} M (it \
             crossed r+ long ago), redshifted to g = {:.3e}, at lambda = {:.6} M against the \
             past-cone distance to r+ of {horizon_cone:.6} M",
            last.emission.r, last.g, last.lambda
        );
        assert!(last.emission.r < 2.001, "the image must have piled up on r+: {}", last.emission.r);
        assert!(last.g < 1e-3, "and faded away: g = {}", last.g);
        assert!(
            (last.lambda - horizon_cone).abs() < 5e-3,
            "lambda must approach the past-cone distance to r+: {} vs {horizon_cone}",
            last.lambda
        );
    }

    #[test]
    fn test_the_horizon_crossing_is_an_ordinary_moment_in_the_frame_view() {
        // Two raindrops falling radially at the same azimuth, the second one deeper. The moment
        // the focus observer crosses r+ they see the other's *own* crossing of r+ - and not
        // because anything has been arranged: r = r+ is a null surface whose generators are the
        // outgoing radial rays, which stand still in r, so light let go on the horizon stays on
        // the horizon until the next worldline crosses it.
        //
        // Nothing singular may happen to the picture there. The seen radius passes through r+
        // smoothly, the shift and the affine distance stay finite, and a little later the seen
        // event is *inside* r+, which is the whole point of drawing the rest frame in a
        // horizon-penetrating chart.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut focus = raindrop(&metric, "Alice", 6.0, 0.0);
        let mut other = raindrop(&metric, "Bob", 4.0, 0.0);

        let mut seed = None;
        let mut before: Option<(f64, AsSeen)> = None;
        let mut at_crossing: Option<AsSeen> = None;
        let mut after: Option<AsSeen> = None;
        for _ in 0..600 {
            play(&metric, &mut focus, &mut other, 1, 0.02);
            if focus.r <= R_STOP * 2.0 {
                break;
            }
            let Ok(seen) = as_seen(&metric, &focus, &other, seed) else {
                continue;
            };
            seed = Some(seen.seed());
            if focus.r > 2.0 {
                before = Some((focus.r, seen));
            } else {
                if at_crossing.is_none() {
                    at_crossing = Some(seen);
                }
                if focus.r < 1.7 && after.is_none() {
                    after = Some(seen);
                }
            }
        }

        let (r_before, _) = before.expect("the focus must have been outside r+ at some point");
        let crossing = at_crossing.expect("and must have crossed it");
        let later = after.expect("and carried on inside");
        println!(
            "the focus crosses r+ (last outside at r = {r_before:.4}): it sees the other at \
             r = {:.5} M, g = {:.5}, lambda = {:.5} M; by r = 1.7 M it sees r = {:.5} M, \
             g = {:.5}, lambda = {:.5} M",
            crossing.emission.r, crossing.g, crossing.lambda, later.emission.r, later.g, later.lambda
        );
        assert!(
            (crossing.emission.r - 2.0).abs() < 2e-2,
            "the seen event at the crossing is the other's own crossing: r = {}",
            crossing.emission.r
        );
        assert!(
            crossing.g.is_finite() && crossing.g > 0.0 && crossing.g < 10.0,
            "nothing happens to the shift there: g = {}",
            crossing.g
        );
        assert!(
            crossing.lambda.is_finite() && crossing.lambda > 0.0 && crossing.lambda < 50.0,
            "nor to the affine distance: lambda = {}",
            crossing.lambda
        );
        assert!(
            later.emission.r < 2.0,
            "and a little later the seen event is inside r+: r = {}",
            later.emission.r
        );
    }

    #[test]
    fn test_a_kerr_solve_is_null_separated_from_the_focus_event() {
        // The check that does not take the solver's word for anything. Alice is on the prograde
        // ISCO of an a = 0.90 hole and Bob is falling; the solver says the light left a particular
        // event on Bob's worldline in a particular direction. So take that direction at *Bob's*
        // end, integrate a fresh ray forward with the app's own integrator, and see whether it
        // lands on Alice.
        //
        // The frequency ratio is re-derived at the same time, from the raw inner products
        // -K . u at the two ends, with K carried between them by its own conserved energy rather
        // than through the `f_factor` machinery the solver used.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let omega = metric.orbital_angular_velocity(r_isco, true).unwrap();
        let u_t = metric.circular_orbit_dilation(r_isco, true).unwrap();
        let mut focus = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut other = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            5.0,
            0.0,
            1.1,
            WorldlineParams::new(1.0, 0.6, false),
        );
        let u_alice = focus.four_velocity(&metric);
        println!(
            "Alice on the prograde ISCO at r = {r_isco:.6}: u = {u_alice:?} against the closed              form ({u_t:.9}, 0, {:.9})",
            u_t * omega
        );
        assert!(
            u_alice[1].abs() < 1e-6
                && (u_alice[0] - u_t).abs() < 1e-6
                && (u_alice[2] - u_t * omega).abs() < 1e-6,
            "Alice must really be on the ISCO: {u_alice:?}"
        );
        play(&metric, &mut focus, &mut other, 120, 0.05);

        let seen = as_seen(&metric, &focus, &other, None).expect("Alice sees Bob");
        println!(
            "a = 0.90, Alice on the ISCO at r = {:.4}, phi = {:.4}: she sees Bob at t = {:.5}, \
             r = {:.5}, phi = {:.5}, lambda = {:.5} M, g = {:.5}, n = ({:.4}, {:.4}), residual \
             {:.2e} in {} iterations",
            focus.r,
            focus.phi,
            seen.emission.t,
            seen.emission.r,
            seen.emission.phi,
            seen.lambda,
            seen.g,
            seen.n[0],
            seen.n[1],
            seen.residual,
            seen.iterations
        );

        // Re-shoot the solved ray from the focus, to get the direction it had at Bob's end.
        let u_focus = focus.four_velocity(&metric);
        let tetrad = Tetrad::from_four_velocity_axial(&metric, focus.r, &u_focus);
        let mut back = NullRay::from_local_direction(
            &metric,
            focus.t,
            focus.r,
            focus.phi,
            &tetrad,
            seen.alpha + PI,
            &u_focus,
        );
        back.step_back(&metric, seen.age);
        assert!(
            (back.r - seen.emission.r).abs() < 1e-8,
            "the re-shot ray must reach the same emission event: {} vs {}",
            back.r,
            seen.emission.r
        );

        // Now forwards, from Bob's event, with a ray built at that event alone.
        let mut forward = NullRay {
            t: back.t,
            r: back.r,
            phi: back.phi,
            dr_dt: back.dr_dt,
            dphi_dt: back.dphi_dt,
            f_emit: 0.0,
            v_emit: back.direction(),
            death_t: None,
            death_end: None,
            slope: None,
        };
        forward.step(&metric, seen.age);
        let d_phi = wrap(forward.phi - focus.phi);
        println!(
            "re-integrated forward from the emission event it lands at r = {:.9} against Alice's \
             {:.9}, and within {:.2e} rad of her azimuth",
            forward.r, focus.r, d_phi
        );
        assert!(
            (forward.r - focus.r).abs() < 1e-6 && d_phi.abs() < 1e-6,
            "the emission event is null-separated from Alice's: landed at ({}, {}) against \
             ({}, {})",
            forward.r,
            forward.phi,
            focus.r,
            focus.phi
        );

        // And the shift, from the inner products alone. K at the focus is already normalised so
        // that -K . u_focus = 1, so g is simply 1 / (-K . u_other) once K has been carried to the
        // emission event by its own conserved energy E_K = -K_t.
        let k_focus = tetrad.null_direction(seen.alpha + PI);
        let g_focus = metric.metric_components(focus.r);
        let energy = -(g_focus[0][0] * k_focus[0]
            + g_focus[0][1] * k_focus[1]
            + g_focus[0][2] * k_focus[2]);
        let v_there = back.direction();
        let g_there = metric.metric_components(back.r);
        let e_v = -(g_there[0][0] * v_there[0]
            + g_there[0][1] * v_there[1]
            + g_there[0][2] * v_there[2]);
        let scale = energy / e_v;
        let k_there = [scale * v_there[0], scale * v_there[1], scale * v_there[2]];
        let nu_emitted = -inner(&metric, back.r, &k_there, &seen.emission.u);
        let nu_seen = -inner(&metric, focus.r, &k_focus, &u_focus);
        println!(
            "-K . u at the two ends: {nu_seen:.12} here and {nu_emitted:.12} there, so \
             g = {:.12} against the solver's {:.12}",
            nu_seen / nu_emitted,
            seen.g
        );
        assert!(
            (nu_seen - 1.0).abs() < 1e-12,
            "the tangent is normalised so the focus measures unit frequency: {nu_seen}"
        );
        assert!(
            (seen.g - nu_seen / nu_emitted).abs() < 1e-8 * seen.g,
            "g = {} against the inner products' {}",
            seen.g,
            nu_seen / nu_emitted
        );
    }

    #[test]
    fn test_the_affine_length_agrees_with_the_length_stepped_along_the_ray() {
        // Two ways of measuring the same parameter along the same geodesic. `affine_length` reads
        // it off a closed-form quadrature in r, which knows nothing about the integration; the
        // other accumulates d lambda = e_v dt / E_K along the ray the integrator actually traced.
        // They have no arithmetic in common beyond the metric, so agreeing is a statement about
        // the curve.
        for &(a, r_focus, r_other, l_other) in &[
            (0.0, 6.0, 9.0, 0.0),
            (0.90, 4.0, 7.5, 1.4),
            (0.65, 3.0, 5.0, -1.0),
        ] {
            let metric = KerrSchild::new(1.0, a);
            let mut focus = raindrop(&metric, "Alice", r_focus, 0.0);
            let mut other = Observer::new_with_phi(
                &metric,
                "Bob",
                0.0,
                r_other,
                0.0,
                0.8,
                WorldlineParams::new(1.0, l_other, false),
            );
            play(&metric, &mut focus, &mut other, 40, 0.05);
            let seen = as_seen(&metric, &focus, &other, None).expect("they see each other");
            let solver = Solver::new(&metric, &focus, &other);
            let stepped = solver.affine_length_by_stepping(seen.alpha, seen.age);
            println!(
                "a = {a}: quadrature lambda = {:.9} M, stepped lambda = {stepped:.9} M \
                 (relative {:.2e})",
                seen.lambda,
                (seen.lambda - stepped).abs() / seen.lambda
            );
            assert!(
                (seen.lambda - stepped).abs() < 1e-6 * seen.lambda,
                "lambda {} vs stepped {stepped} at a = {a}",
                seen.lambda
            );
        }
    }

    #[test]
    fn test_the_interpolated_worldline_is_the_one_the_app_draws() {
        // The solver reads the other observer's worldline hundreds of times per solve, so it
        // cannot afford `Observer::event_at`, which clones the observer and re-integrates. It
        // interpolates the recorded trail instead - and the interpolation has to be the same
        // curve, or the emission event is on a worldline nobody is drawing. This measures the one
        // against the other at times spread across a whole infall, including inside r+.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut focus = hovering(&metric, "Alice", 9.0, 0.0);
        let mut other = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            6.0,
            0.0,
            0.3,
            WorldlineParams::new(1.0, 1.8, false),
        );
        play(&metric, &mut focus, &mut other, 400, 0.05);
        let line = OtherWorldline::new(&metric, &other);

        let mut worst_r = 0.0f64;
        let mut worst_phi = 0.0f64;
        let mut worst_tau = 0.0f64;
        let mut checked = 0;
        for i in 1..=80 {
            let t = other.t * (i as f64) / 81.0;
            let exact = other.event_at(&metric, t);
            let (got, source) = line.at(t, exact.phi).expect("the trail still holds it");
            assert_eq!(source, WorldlineSource::Trail);
            worst_r = worst_r.max((got.r - exact.r).abs());
            worst_phi = worst_phi.max(wrap(got.phi - exact.phi).abs());
            worst_tau = worst_tau.max((got.tau - exact.tau).abs());
            checked += 1;
        }
        println!(
            "{checked} times across an infall from 6 M through r+ = {:.4}: the interpolated \
             worldline differs from the re-integrated one by at most {worst_r:.2e} in r, \
             {worst_phi:.2e} rad and {worst_tau:.2e} M of the emitter's own clock",
            metric.outer_horizon()
        );
        assert!(worst_r < 1e-7, "r differs by {worst_r}");
        assert!(worst_phi < 1e-7, "phi differs by {worst_phi}");
        assert!(worst_tau < 1e-7, "tau differs by {worst_tau}");
    }

    #[test]
    fn test_the_hold_is_extended_backwards_so_there_is_a_picture_at_t_zero() {
        // At t = 0 nothing has had time to arrive, so every image is of an event before the run.
        // A hovering observer genuinely was hovering then - a platform under thrust does not begin
        // to exist when a simulation clock is started - and the hold is an integral curve of a
        // Killing field, so the extension is a closed form rather than a guess. The user asked for
        // this behaviour explicitly, and it is what makes the very first frame of the rest-frame
        // view show anything at all.
        let metric = KerrSchild::new(1.0, 0.65);
        let focus = held(&metric, "Alice", 7.0, 0.0);
        let other = held(&metric, "Bob", 7.0, 0.9);
        assert_eq!(focus.t, 0.0, "the run has not started");

        let seen = as_seen(&metric, &focus, &other, None).expect("the pair see each other at t = 0");
        println!(
            "at t = 0, a = 0.65: Alice at 7 M sees Bob (0.9 rad away at the same radius) as he \
             was at t = {:.5} M, his watch reading {:.5} M, at lambda = {:.5} M with \
             g = {:.6} and n = ({:.4}, {:.4})",
            seen.emission.t, seen.emission.tau, seen.lambda, seen.g, seen.n[0], seen.n[1]
        );
        assert_eq!(seen.source, WorldlineSource::Hold);
        assert!(seen.emission.t < 0.0, "the light left before t = 0: {}", seen.emission.t);
        assert!(
            seen.emission.tau < 0.0,
            "and his watch had not started either: {}",
            seen.emission.tau
        );
        assert!(
            (seen.emission.r - 7.0).abs() < 1e-7,
            "he was holding the same radius: {}",
            seen.emission.r
        );
        // Both are holding the same radius on the same zero-angular-momentum worldline, which is
        // an orbit of the Killing field d_t + Omega(r) d_phi normalised by |xi| - the same field
        // and the same normalisation at both events, since the radius is the same. The Killing
        // energy -k . xi is conserved along the ray, so the two ends measure the same frequency
        // and the shift is exactly one, however far apart they are and however hard the frame is
        // being dragged past them.
        assert!((seen.g - 1.0).abs() < 1e-7, "g between two equal hovers is 1: {}", seen.g);
        // The source is round the ring rather than up or down it, so the arrival direction is
        // almost entirely azimuthal.
        assert!(
            seen.n[1].abs() > 0.9,
            "the light arrives from round the hole: n = {:?}",
            seen.n
        );

        // This is also the configuration that exercises the split quadrature. Two events at the
        // same radius cannot be joined by a monotone radial leg: the light dips below them both
        // and climbs back, so the affine length is taken in two pieces meeting at the exact
        // turning radius. The length stepped along the ray knows nothing about that split, so
        // agreeing with it is a check on the root of the cubic as well as on the quadrature.
        let solver = Solver::new(&metric, &focus, &other);
        let stepped = solver.affine_length_by_stepping(seen.alpha, seen.age);
        println!(
            "the ray turned in r on the way, so lambda was taken in two legs: {:.9} M against              {stepped:.9} M stepped along the ray",
            seen.lambda
        );
        assert!(
            (seen.lambda - stepped).abs() < 1e-6 * seen.lambda,
            "the split quadrature gives {} against the stepped {stepped}",
            seen.lambda
        );
    }

    #[test]
    fn test_a_warm_start_agrees_with_a_cold_one_and_is_much_cheaper() {
        // What the view will actually do: solve once from nothing, then hand last frame's answer
        // back every frame after. The two must be the same solve - a warm start is an initial
        // guess and nothing else - and the warm one has to fit inside a frame with room to spare.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut focus = raindrop(&metric, "Alice", 8.0, 0.0);
        let mut other = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            5.5,
            0.0,
            0.7,
            WorldlineParams::new(1.0, 1.5, false),
        );
        play(&metric, &mut focus, &mut other, 60, 0.05);

        let cold = as_seen(&metric, &focus, &other, None).expect("a cold solve");
        assert!(cold.cold, "the first solve has no seed to warm from");
        let warm = as_seen(&metric, &focus, &other, Some(cold.seed())).expect("a warm solve");
        assert!(!warm.cold, "the second must not need the fan");
        assert!(
            (cold.lambda - warm.lambda).abs() < 1e-7
                && (cold.g - warm.g).abs() < 1e-7
                && (cold.emission.t - warm.emission.t).abs() < 1e-7,
            "cold {cold:?} against warm {warm:?}"
        );

        let rounds = 50;
        let start = std::time::Instant::now();
        for _ in 0..rounds {
            std::hint::black_box(as_seen(&metric, &focus, &other, None)).ok();
        }
        let cold_time = start.elapsed() / rounds;
        let seed = Some(cold.seed());
        let start = std::time::Instant::now();
        for _ in 0..rounds {
            std::hint::black_box(as_seen(&metric, &focus, &other, seed)).ok();
        }
        let warm_time = start.elapsed() / rounds;
        println!(
            "a = 0.90, a falling pair {:.2} M apart: cold solve {cold_time:?} ({} iterations \
             after the fan), warm solve {warm_time:?} ({} iterations)",
            (focus.r - other.r).abs(),
            cold.iterations,
            warm.iterations
        );
        assert!(
            warm_time < cold_time,
            "the seed has to buy something: warm {warm_time:?} against cold {cold_time:?}"
        );
    }

    #[test]
    fn test_there_is_no_image_rather_than_a_guess_when_there_is_nothing_to_see() {
        // Every way out of the solver is a statement, and these are the ones the view has to be
        // ready for.
        let metric = KerrSchild::new(1.0, 0.90);

        // A focus observer settled on the far branch of r-. Their u^t has run out to 1e10, so
        // every direction in their frame is aberrated into one point and there is no drawable
        // picture of anything; the app already treats that worldline as over.
        let mut frozen = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            9.0,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.2, false),
        );
        let mut t = 0.0;
        while !frozen.is_frozen() && t < 200.0 {
            t += 0.25;
            frozen.step(&metric, t, 0.25);
        }
        assert!(frozen.is_frozen(), "the walk must reach the freeze");
        let other = hovering(&metric, "Bob", 9.0, 0.5);
        assert_eq!(
            as_seen(&metric, &frozen, &other, None),
            Err(NoImage::FocusFrozen)
        );

        // A focus observer who has reached the ring has no worldline left to receive on.
        let mut ended = raindrop(&metric, "Alice", 6.0, 0.0);
        let mut t = 0.0;
        while ended.r > R_STOP && t < 200.0 {
            t += 0.25;
            ended.step(&metric, t, 0.25);
        }
        assert!(ended.has_ended(), "the raindrop must reach the ring");
        assert_eq!(as_seen(&metric, &ended, &other, None), Err(NoImage::FocusEnded));

        // A trail whose cap has swallowed the stretch the answer needs. Nothing here will invent
        // it: the reversible window is the window the trail keeps, exactly as it is for a rewind.
        let focus = hovering(&metric, "Alice", 9.0, 0.0);
        let mut far = raindrop(&metric, "Bob", 9.0, 2.5);
        far.release_t = -1e9;
        far.trail.clear();
        far.trail.push_back(TrailPoint {
            t: far.t,
            r: far.r,
            phi: far.phi,
            tau: far.tau,
            u: far.four_velocity(&metric),
            stalled: false,
        });
        let answer = as_seen(&metric, &focus, &far, None);
        println!("with the needed stretch evicted from the trail the solver says {answer:?}");
        assert_eq!(answer, Err(NoImage::TrailEvicted));
    }

    #[test]
    fn test_the_focus_observer_sees_from_every_region() {
        // The rest-frame view can be drawn for an observer anywhere, so the solve has to work
        // anywhere: outside the static limit, inside the ergosphere, between the horizons where r
        // is timelike and nothing can hover, and inside r- where the cone has reopened outward.
        // Alice falls from 4 M to the ring of an a = 0.90 hole while Bob falls in behind her, and
        // every frame of her descent is solved and checked: the ray carries a finite positive
        // shift, the arrival direction is a unit vector, and the emission event is in her past.
        //
        // Light from above crosses r+ and then the near branch of r- quite happily in this chart -
        // that is what a horizon-penetrating chart is for - so there is nothing special about the
        // interior except that nobody can hold station in it.
        let metric = KerrSchild::new(1.0, 0.90);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let mut focus = raindrop(&metric, "Alice", 4.0, 0.0);
        let mut other = raindrop(&metric, "Bob", 9.0, 0.25);

        let names = ["outside 2M", "inside the ergosphere", "between the horizons", "inside r-"];
        let mut solved = [0usize; 4];
        let mut report: [Option<(f64, f64, f64)>; 4] = [None; 4];
        let mut seed = None;
        for _ in 0..4000 {
            play(&metric, &mut focus, &mut other, 1, 0.01);
            if focus.r <= R_STOP * 1.5 || focus.has_ended() {
                break;
            }
            let region = if focus.r > 2.0 {
                0
            } else if focus.r > rp {
                1
            } else if focus.r > rm {
                2
            } else {
                3
            };
            let Ok(seen) = as_seen(&metric, &focus, &other, seed) else {
                continue;
            };
            seed = Some(seen.seed());
            solved[region] += 1;
            report[region] = Some((focus.r, seen.lambda, seen.g));
            assert!(
                seen.g.is_finite() && seen.g > 0.0,
                "{}: g = {} at r = {}",
                names[region],
                seen.g,
                focus.r
            );
            assert!(
                seen.lambda.is_finite() && seen.lambda > 0.0,
                "{}: lambda = {} at r = {}",
                names[region],
                seen.lambda,
                focus.r
            );
            let unit = seen.n[0].hypot(seen.n[1]);
            assert!(
                (unit - 1.0).abs() < 1e-12,
                "{}: the arrival direction must be a unit vector, got {unit}",
                names[region]
            );
            assert!(
                seen.emission.t < focus.t,
                "{}: the light must have left before it arrived: {} against {}",
                names[region],
                seen.emission.t,
                focus.t
            );
            assert!(
                seen.residual < 1e-6,
                "{}: residual {} at r = {}",
                names[region],
                seen.residual,
                focus.r
            );
        }

        for (i, name) in names.iter().enumerate() {
            let (r, lambda, g) = report[i]
                .unwrap_or_else(|| panic!("Alice never saw Bob from {name}"));
            println!(
                "{name} (r+ = {rp:.4}, r- = {rm:.4}): {} frames solved, last at r = {r:.4} with \
                 lambda = {lambda:.4} M and g = {g:.4}",
                solved[i]
            );
        }
    }

    /// How far apart two solves of the same configuration may be before they are two answers
    /// rather than one. The emission time is the quantity everything else is exponentially
    /// sensitive to on this approach - g falls like exp(-t/4M) for a hovering watcher - so it is
    /// held to a micro-M and the rest is allowed to follow.
    fn assert_same_solve(what: &str, a: &AsSeen, b: &AsSeen) {
        assert!(
            (a.emission.t - b.emission.t).abs() < 1e-5,
            "{what}: emission t {} against {}",
            a.emission.t,
            b.emission.t
        );
        assert!(
            (a.emission.r - b.emission.r).abs() < 1e-7 * (1.0 + a.emission.r),
            "{what}: emission r {} against {}",
            a.emission.r,
            b.emission.r
        );
        assert!(
            (a.lambda - b.lambda).abs() < 1e-6 * a.lambda,
            "{what}: lambda {} against {}",
            a.lambda,
            b.lambda
        );
        assert!(
            (a.g - b.g).abs() < 1e-4 * a.g,
            "{what}: g {} against {}",
            a.g,
            b.g
        );
    }

    #[test]
    fn test_the_warm_path_throws_away_a_solve_that_has_left_the_seeds_own_image() {
        // What the guards on the warm path do, and - measured, at the end - what they do not.
        //
        // Alice on the prograde ISCO of an a = 0.90 hole sits barely above the prograde photon
        // orbit, so Bob - the app's own raindrop, dropped from 27 M - has many images: the direct
        // one, and a sequence whose rays wind round the photon sphere. A Newton started from last
        // frame's answer is an initial guess and nothing more, so it can land on any of them.
        // `continues` is the test that decides whether the answer it landed on is the one the
        // seed was about, and a refusal costs the cold march, which is what a caller with no seed
        // would have paid anyway.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let mut focus = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut other = raindrop(&metric, "Bob", 27.0, 0.0);

        // Thread a chain frame by frame at the pace the view plays at, keeping the answers at the
        // epochs the sibling test measures. Every one of these solves must succeed: a guard that
        // fired on an honest step would turn a warm frame into a cold one for nothing.
        let mut seed = None;
        let mut chain: Vec<(f64, AsSeen)> = Vec::new();
        let mut epochs = vec![120.0, 150.0, 180.0];
        let mut cold_frames = 0;
        // The pair as they stood at the first epoch, kept so that the guard can be put to work
        // on the event where the chain and the direct image disagree most.
        let mut deep: Option<(Observer, Observer)> = None;
        let start = std::time::Instant::now();
        for _ in 0..1800 {
            play(&metric, &mut focus, &mut other, 1, 0.1);
            let threaded = as_seen(&metric, &focus, &other, seed)
                .expect("the threaded solve must never drop out");
            if threaded.cold {
                cold_frames += 1;
            }
            seed = Some(threaded.seed());
            if epochs.first().is_some_and(|due| focus.t >= *due) {
                epochs.remove(0);
                if deep.is_none() {
                    deep = Some((focus.clone(), other.clone()));
                }
                chain.push((focus.t, threaded));
            }
        }
        let elapsed = start.elapsed();
        println!(
            "1800 guarded frames of the ISCO run took {elapsed:?}, {:?} a frame, of which \
             {cold_frames} fell through to the cold path",
            elapsed / 1800
        );
        assert_eq!(chain.len(), 3, "every epoch must have been reached");
        // The cost of the guards is the frames they refuse, and on an honest run that is a
        // handful: the first frame, which has no seed to be guarded, and the odd epoch where the
        // Newton itself does not converge and would have fallen through with or without them.
        // Counting rather than timing, so the assertion says the same thing on every machine.
        // (The 3 ms a frame the print reports is not the guards: the same loop with the test
        // taken out measures 3.16 ms against 3.19 ms. The cost is tracing a ray fifty to a
        // hundred M back through the strong field, which is what this configuration asks for
        // however the answer is seeded, and it is an order of magnitude above the 14 us a warm
        // solve costs for a pair a few M apart.)
        assert!(
            cold_frames <= 4,
            "the guards must not turn honest warm frames into cold ones: {cold_frames} of 1800"
        );

        // The guard itself, at the deepest epoch. The chain has drifted onto an image whose ray
        // winds round the hole, and a seed taken from the *direct* image at that same event
        // describes a different picture entirely. Hand the solver the wound image's starting
        // angle under the direct image's emission time and affine length - which is exactly the
        // shape of a Newton that has wandered off its own branch - and the answer has to be
        // thrown away rather than drawn.
        let (t_deep, wound) = chain[0];
        let (focus, other) = deep.expect("the chain reached the first epoch");
        let direct = as_seen(&metric, &focus, &other, None).expect("the cold march finds it");
        println!(
            "at t = {t_deep:.1} M the threaded chain is on an image of age {:.4} M (lambda \
             {:.4} M, emission t = {:.4}) while the direct image has age {:.4} M (lambda \
             {:.4} M, emission t = {:.4})",
            wound.age,
            wound.lambda,
            wound.emission.t,
            direct.age,
            direct.lambda,
            direct.emission.t
        );
        assert!(
            !continues(&wound, direct.emission.t, direct.lambda, t_deep),
            "an answer this far from the seed's own image must be refused"
        );
        let crossed = AsSeenSeed {
            alpha: wound.alpha,
            age: wound.age,
            emission_t: direct.emission.t,
            lambda: direct.lambda,
        };
        let recovered = as_seen(&metric, &focus, &other, Some(crossed))
            .expect("and the cold path picks the answer up again");
        assert_same_solve("recovered after a refused seed", &recovered, &direct);
        // Without the guard the same seed would have handed the wound image straight back.
        let unguarded = Solver::new(&metric, &focus, &other)
            .refine(crossed.alpha, crossed.age, false)
            .expect("the Newton converges on it perfectly well");
        assert!(
            (unguarded.age - wound.age).abs() < 1e-3,
            "the refused answer is the wound image: age {} against {}",
            unguarded.age,
            wound.age
        );
        println!(
            "the guard refused an age of {:.4} M and the cold path answered {:.4} M",
            unguarded.age, recovered.age
        );

        // And the limit of the two guards, recorded rather than papered over. Both of them are
        // *local*: they ask whether this answer continues the last one. The ISCO chain does not
        // fail either of them - its emission time rises steadily and its lambda moves by a per
        // cent a step - and it is still on the wrong image, because a new and more direct image
        // comes into existence at about t = 43 M and nothing local can tell the chain that. The
        // measured numbers: the chain and a cold solve agree exactly up to t = 40 M, at t = 45 M
        // the cold solve reports an emission at t = 12.17 where the chain is at t = -9.89, and by
        // t = 120 M the chain reports an age of 96.81 M against the direct image's 47.49 M. A
        // winding count does not separate the two either, because the direct image's own ray is
        // dragged several times round the hole once the source has frozen on r+. Deciding which
        // image is the direct one takes a sweep of the whole past cone, which is the cold path;
        // what the guards buy is that a chain which *jumps* is caught, not that a chain which
        // drifts is.
        assert!(
            wound.age > direct.age + 1.0,
            "this test is about a chain that has drifted; it has not: {} against {}",
            wound.age,
            direct.age
        );
    }

    #[test]
    fn test_the_youngest_image_replaces_a_chain_that_has_wound_round_the_hole() {
        // The rule the view draws by, at the configuration that needs it. The sibling test above
        // records that a chain threaded frame by frame from the prograde ISCO of an a = 0.90 hole
        // drifts, without ever jumping, onto an image whose ray has wound round the hole: by
        // t = 120 M it reports Bob as he was 96.81 M ago while a younger image carries him as he
        // was 47.49 M ago. Nothing local catches that. `younger_image` is the non-local check: a
        // sweep of the past cone for an image with a later emission event.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let mut focus = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut other = raindrop(&metric, "Bob", 27.0, 0.0);
        let mut seed = None;
        let mut chain = None;
        for _ in 0..1200 {
            play(&metric, &mut focus, &mut other, 1, 0.1);
            let threaded = as_seen(&metric, &focus, &other, seed).expect("the chain holds");
            seed = Some(threaded.seed());
            chain = Some(threaded);
        }
        let wound = chain.expect("1200 frames were threaded");

        let start = std::time::Instant::now();
        let younger = younger_image(&metric, &focus, &other, &wound)
            .expect("the sweep finds the image that was born at about t = 43 M");
        let elapsed = start.elapsed();
        println!(
            "at t = {:.1} M the chain is on an image {:.4} M old and the sweep found one {:.4} M \
             old in {elapsed:?}: emission t = {:.4} against {:.4}, lambda {:.4} M against \
             {:.4} M, g = {:.4} against {:.4}",
            focus.t,
            wound.age,
            younger.age,
            younger.emission.t,
            wound.emission.t,
            younger.lambda,
            wound.lambda,
            younger.g,
            wound.g
        );
        assert!(
            younger.emission.t > wound.emission.t + 10.0,
            "a younger image by a wide margin: {} against {}",
            younger.emission.t,
            wound.emission.t
        );
        assert!(younger.emission.t < focus.t, "and it is still in the past");
        assert!(
            younger.residual < 1e-6,
            "and it is a real crossing: residual {}",
            younger.residual
        );
        // It is at least as young as anything the cold march reaches, and there is nothing
        // younger still: the rule has a fixed point.
        let marched = as_seen(&metric, &focus, &other, None).expect("the march finds an image");
        assert!(
            younger.emission.t >= marched.emission.t - YOUNGER_MARGIN,
            "the youngest image is not older than the march's: {} against {}",
            younger.emission.t,
            marched.emission.t
        );
        assert!(
            younger_image(&metric, &focus, &other, &younger).is_none(),
            "nothing younger than the youngest"
        );
        // And the combined rule, warm-started from the wound chain's own seed, gives the same
        // answer the view will draw.
        let drawn = as_seen_youngest(&metric, &focus, &other, Some(wound.seed()))
            .expect("the combined rule answers");
        assert_same_solve("the youngest image from the chain's seed", &drawn, &younger);
    }




    #[test]
    fn test_a_release_that_arrives_moving_is_seen_along_its_own_geodesic_before_the_run() {
        // The user's own starting file: two raindrops released together at 4.5 M of an a = 0.90
        // hole, 0.00015 M apart. Two observers at the same place with the same velocity see each
        // other at g = 1, whatever the field. Read off the static hold the app keeps for a moving
        // release, they were seen at g = 1.39 one way and 1.12 the other - the Doppler shift
        // between a hovering emitter and a receiver already falling at 0.6 c - so Bob's beacon
        // came out in the ultraviolet as a dashed ring and Alice's came out green. Along the
        // geodesic continued backwards through the release event, they were always falling
        // together, and the shift is one.
        let metric = KerrSchild::new(1.0, 0.90);
        let alice = raindrop(&metric, "Alice", 4.5, 0.0);
        let bob = raindrop(&metric, "Bob", 4.5002, 0.0);
        for (focus, other) in [(&alice, &bob), (&bob, &alice)] {
            let seen = as_seen_youngest(&metric, focus, other, None)
                .expect("a pair this close see each other");
            println!(
                "{} sees {}: g = {:.6}, lambda = {:.6} M, emission {:?} at t = {:.6}, r = {:.6}",
                focus.name, other.name, seen.g, seen.lambda, seen.source, seen.emission.t,
                seen.emission.r
            );
            assert_eq!(seen.source, WorldlineSource::Geodesic);
            assert!(seen.emission.t < 0.0, "the light left before the run began");
            assert!(
                (seen.g - 1.0).abs() < 2e-3,
                "the same velocity at the same place is no shift at all: g = {}",
                seen.g
            );
            assert!(seen.age < 0.01, "and the delay is the gap: {}", seen.age);
        }

        // The pre-run worldline is the geodesic and not a look-alike: take the event it reports
        // 5 M of coordinate time before the release, integrate the forward run's own stepper
        // from there, and it has to arrive back on the release event.
        let line = OtherWorldline::new(&metric, &bob);
        let (before, source) = line.point_at(-5.0).expect("the pre-run worldline reaches back");
        assert_eq!(source, WorldlineSource::Geodesic);
        assert!(
            before.r > bob.start.r + 0.5,
            "a raindrop was higher up 5 M ago: r = {} against {}",
            before.r,
            bob.start.r
        );
        assert!(before.tau < 0.0, "and its watch had not yet reached zero: {}", before.tau);
        let geodesic = bob.geodesic.as_ref().expect("a free-fall observer carries one");
        let mut state = GeodesicState {
            t: before.t,
            r: before.r,
            phi: before.phi,
            tau: before.tau,
            energy: geodesic.energy,
            l_ang: geodesic.l_ang,
            u: before.u,
            stalled: false,
        };
        let mut steps = 0;
        while state.t < bob.start.t - 1e-12 && steps < 100_000 {
            let dt = (bob.start.t - state.t).min(1e-3);
            state.step_coord_time(&metric, dt);
            steps += 1;
        }
        println!(
            "integrated forward from t = -5 in {steps} steps: r = {:.9} against the release at \
             {:.9}, tau = {:.9} against {:.9}",
            state.r, bob.start.r, state.tau, bob.start.tau
        );
        assert!(
            (state.r - bob.start.r).abs() < 1e-5,
            "the backward integration has to be the forward one run in reverse: r = {} against {}",
            state.r,
            bob.start.r
        );
        assert!(
            (state.tau - bob.start.tau).abs() < 1e-5,
            "and on the same watch: tau = {} against {}",
            state.tau,
            bob.start.tau
        );

        // A hovering pair keep the hold: a platform under thrust genuinely was hovering.
        let hover_a = hovering(&metric, "Alice", 4.5, 0.0);
        let hover_b = hovering(&metric, "Bob", 4.5002, 0.0);
        let seen = as_seen(&metric, &hover_a, &hover_b, None).expect("hovering pair");
        assert_eq!(seen.source, WorldlineSource::Hold);
    }





    #[test]
    fn test_a_frozen_picture_is_not_refused_for_jittering_backwards_by_rounding() {
        // Bob on the one worldline that freezes onto the far branch of r- (E = 1, L = 2.2 at
        // a = 0.90), watching Alice's raindrop fall in ahead of him. Once his u^t passes 1e5 his
        // clock all but stops and the picture with it: Alice's emission event stood at
        // t = 4.87435 M for frame after frame while the solve's rounding moved it 1e-6 to 6e-6 M
        // *earlier* each time, and the continuity guard - whose slack was 1e-6 - refused every
        // one of those frames as a different image. Threaded at the app's own pace, the chain has
        // to hold to u^t = 1e6 without a single refusal; beyond about 8e6 the frame is boosted
        // past what the solver resolves and the view holds the picture instead (see
        // `spacetime_canvas::PICTURE_BOOST_LIMIT`).
        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.2, false),
        );
        let mut alice = raindrop(&metric, "Alice", 4.45, 0.0);
        let mut seed = None;
        let mut frames = 0;
        let mut failures = 0;
        let mut last: Option<AsSeen> = None;
        while bob.four_velocity(&metric)[0] < 1e6 {
            play(&metric, &mut bob, &mut alice, 1, 0.05);
            assert!(!bob.is_frozen(), "the run must reach u^t = 1e6 before the stall");
            frames += 1;
            match as_seen(&metric, &bob, &alice, seed) {
                Ok(seen) => {
                    seed = Some(seen.seed());
                    last = Some(seen);
                }
                Err(_) => failures += 1,
            }
        }
        let last = last.expect("Bob saw Alice");
        println!(
            "{frames} frames to u^t = {:.3e}, {failures} refused; the picture at the end: g = \
             {:.4}, lambda = {:.4} M, emission at t = {:.5} M, r = {:.5} M",
            bob.four_velocity(&metric)[0],
            last.g,
            last.lambda,
            last.emission.t,
            last.emission.r
        );
        assert_eq!(failures, 0, "a picture that is not changing must not be refused");
        assert!(last.g < 0.05, "and it is the deeply redshifted one: g = {}", last.g);
    }


    #[test]
    fn test_the_fan_enumerates_the_higher_order_images_as_well_as_the_direct_one() {
        // A source this deep in the field is seen more than once, and the cold fan already knows
        // where the extra images are: it finds the dips of the past-cone sweep one after another
        // and the refinement turns each into a crossing. `images` is that enumeration, and this
        // is the check that the answers are real and distinct - each one a genuine null
        // connection with its own delay, its own affine length and its own arrival direction.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let mut focus = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut other = raindrop(&metric, "Bob", 27.0, 0.0);
        play(&metric, &mut focus, &mut other, 600, 0.1);

        let start = std::time::Instant::now();
        let found = images(&metric, &focus, &other, 4);
        let elapsed = start.elapsed();
        for image in &found {
            println!(
                "image at age {:8.4} M: r = {:9.5} M, lambda = {:8.4} M, g = {:.4e}, \
                 n = ({:+.4}, {:+.4}), {} extra turn(s), residual {:.2e}",
                image.age,
                image.emission.r,
                image.lambda,
                image.g,
                image.n[0],
                image.n[1],
                image.windings,
                image.residual
            );
        }
        println!("{} image(s) found in {elapsed:?}", found.len());
        assert!(
            found.len() >= 2,
            "the ISCO observer sees this source more than once: {} image(s)",
            found.len()
        );
        // The list is ordered by age, most direct first, and the answer `as_seen` reports is one
        // of its entries. It is not always the first, and that is the same finding the warm-guard
        // test records: at this event the fan finds a crossing of age 12.69 M while the cold
        // march, walking the answer forward from t = 0, is on the one of age 64.58 M. Both are
        // real null connections between the two worldlines; which of them is the picture Alice
        // mostly sees is a question the march answers by continuity and the fan answers by
        // sweeping, and the two do not always agree this deep in the field.
        let drawn = as_seen(&metric, &focus, &other, None).expect("the marched answer");
        assert!(
            found.windows(2).all(|pair| pair[0].age <= pair[1].age),
            "the images come back most direct first"
        );
        assert!(
            found
                .iter()
                .any(|image| (image.age - drawn.age).abs() < 1e-6),
            "the marched answer must be one of the enumerated images: age {} against {:?}",
            drawn.age,
            found.iter().map(|i| i.age).collect::<Vec<_>>()
        );
        for image in &found {
            assert!(image.residual < 1e-6, "every image is a solve: {}", image.residual);
            assert!(image.age > 0.0 && image.g > 0.0, "and a real one: {image:?}");
        }
    }

    #[test]
    fn test_a_cold_solve_reaches_the_deep_image_that_a_warm_one_does() {
        // The failure this pins. A view opened late in a run, or a run loaded off a file, has no
        // seed to warm from, and the cold sweep used to hand the refinement a seed that was not
        // in the right county: for a hovering watcher whose image of an infaller has piled up on
        // r+, the cold solve refused from about g = 1e-1 downwards while a warm solve threaded
        // from earlier frames answered the same configuration every time. The cause was the sweep
        // watching the minimum separation *over* the fan rather than each generator's own: the
        // generator carrying the image hugs the horizon while the rest of the cone sweeps out past
        // the other observer's earlier, higher position, so the minimum over the fan never turned
        // round, the sweep fell through to its global best, and the seed came back at an age of
        // 142 M for a crossing at 23 M.
        //
        // So the test threads a warm solution down a whole infall and, at each rung of a ladder in
        // the redshift, solves the *same* configuration cold and demands the same answer. Five
        // rungs, from a tenth down to a hundred-thousandth, which on this approach is a seen radius
        // of r+ + 4e-6 M and an emission event nearly three quarters of the way to the horizon
        // crossing in coordinate time.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut focus = hovering(&metric, "Alice", 8.0, 0.0);
        let mut other = raindrop(&metric, "Bob", 6.0, 0.0);

        let mut rungs = vec![1e-1, 1e-2, 1e-3, 1e-4, 1e-5];
        let mut seed = None;
        let mut deepest = None;
        for _ in 0..1500 {
            play(&metric, &mut focus, &mut other, 1, 0.1);
            let warm = as_seen(&metric, &focus, &other, seed)
                .expect("the threaded solve must never drop out");
            seed = Some(warm.seed());
            if rungs.first().is_none_or(|target| warm.g > *target) {
                continue;
            }
            let target = rungs.remove(0);
            let start = std::time::Instant::now();
            let cold = as_seen(&metric, &focus, &other, None);
            let elapsed = start.elapsed();
            let cold = cold.unwrap_or_else(|why| {
                panic!(
                    "cold solve refused at t = {:.1} M, g = {:.3e} (rung {target:.0e}): {why:?}",
                    focus.t, warm.g
                )
            });
            println!(
                "rung {target:.0e} at t = {:6.1} M: warm sees r = {:.7} M, g = {:.4e}, \
                 lambda = {:.6} M, age = {:.4} M; the cold solve took {elapsed:?} and {} \
                 iterations to reach r = {:.7} M, g = {:.4e}, lambda = {:.6} M",
                focus.t,
                warm.emission.r,
                warm.g,
                warm.lambda,
                warm.age,
                cold.iterations,
                cold.emission.r,
                cold.g,
                cold.lambda
            );
            assert!(cold.cold, "the solve under test must really have started cold");
            assert_same_solve("hovering watcher", &warm, &cold);
            deepest = Some((focus.t, warm.g, elapsed));
            if rungs.is_empty() {
                break;
            }
        }
        let (t_deep, g_deep, cost) = deepest.expect("the ladder must have been climbed");
        assert!(
            rungs.is_empty(),
            "the ladder stopped at g = {g_deep:.3e} (t = {t_deep:.1} M) with {} rungs left",
            rungs.len()
        );
        println!(
            "the deepest rung, g = {g_deep:.3e} at t = {t_deep:.1} M, cost {cost:?} cold"
        );

        // And a Kerr case, with the focus on the prograde ISCO of an a = 0.90 hole and the other
        // on the app's own default worldline - a raindrop, E = 1, L = 0 - dropped from 27 M and
        // followed until its image has frozen on r+ and faded by seven decades. Nothing here is
        // radial: Alice goes round at Omega = 0.2254 per M while Bob is dragged the other way onto
        // the horizon's rotation, so the arrival angle is a real unknown rather than pi by
        // symmetry.
        //
        // The reference cannot be a chain threaded from the beginning of the run, and that is a
        // finding rather than a shortcut. An observer on the ISCO of a fast hole sits barely above
        // the prograde photon orbit, so the source has *many* images - the direct one, and a
        // sequence of ever-fainter ones whose rays wind round the photon sphere - and a warm chain
        // threaded frame by frame will happily follow one of them for as long as it exists. Run
        // from t = 0 in steps of 0.1 M this one does exactly that: by t = 45 M it is reporting an
        // emission 53 M in the past where the direct image is 33 M in the past, and it stays on
        // that branch until the branch ends near t = 167 M. Everything about it is
        // self-consistent - the residual is 1e-13, g and d(emission)/d(focus) agree to four
        // figures - and it is still not what Alice mostly sees. So what is checked here is what
        // can be checked: that the cold answer is a real null connection, by re-integrating the
        // ray forward from the emission event and landing on Alice; that it is at least as direct
        // as the threaded chain's, its age never being the larger of the two; and that a second
        // cold solve two M earlier, threaded forward to the same event, lands on the same answer.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let mut focus = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut other = raindrop(&metric, "Bob", 27.0, 0.0);

        let mut chain = None;
        let mut epochs = vec![120.0, 150.0, 180.0];
        let mut previous_emission = f64::NEG_INFINITY;
        let mut checked = 0;
        for _ in 0..2200 {
            play(&metric, &mut focus, &mut other, 1, 0.1);
            if let Ok(threaded) = as_seen(&metric, &focus, &other, chain) {
                chain = Some(threaded.seed());
            }
            if epochs.first().is_none_or(|due| focus.t < *due) {
                continue;
            }
            epochs.remove(0);

            let start = std::time::Instant::now();
            let cold = as_seen(&metric, &focus, &other, None);
            let elapsed = start.elapsed();
            let cold = cold.unwrap_or_else(|why| {
                panic!("Kerr cold solve refused at t = {:.1} M: {why:?}", focus.t)
            });
            let threaded = as_seen(&metric, &focus, &other, chain)
                .expect("the chain is on some image, whichever one it is");
            println!(
                "a = 0.90 ISCO at t = {:6.1} M: cold sees r = {:.7} M, g = {:.4e}, \
                 lambda = {:.5} M, age = {:.4} M, n = ({:.4}, {:.4}) in {elapsed:?}; the chain \
                 threaded from t = 0 is on an image of age {:.4} M",
                focus.t,
                cold.emission.r,
                cold.g,
                cold.lambda,
                cold.age,
                cold.n[0],
                cold.n[1],
                threaded.age
            );
            assert!(cold.cold, "the Kerr solve under test must really have started cold");
            assert!(
                cold.age <= threaded.age + 1e-6,
                "the cold answer must be at least as direct as the chain's: age {} against {}",
                cold.age,
                threaded.age
            );
            assert!(
                cold.emission.t >= previous_emission - 1e-6,
                "the emission event must not go backwards as the run goes on: {} after {}",
                cold.emission.t,
                previous_emission
            );
            previous_emission = cold.emission.t;

            // A real null connection: rebuild the ray at the emission event and run it forward.
            let u_focus = focus.four_velocity(&metric);
            let tetrad = Tetrad::from_four_velocity_axial(&metric, focus.r, &u_focus);
            let mut back = NullRay::from_local_direction(
                &metric,
                focus.t,
                focus.r,
                focus.phi,
                &tetrad,
                cold.alpha + PI,
                &u_focus,
            );
            back.step_back(&metric, cold.age);
            let mut forward = NullRay {
                t: back.t,
                r: back.r,
                phi: back.phi,
                dr_dt: back.dr_dt,
                dphi_dt: back.dphi_dt,
                f_emit: 0.0,
                v_emit: back.direction(),
                death_t: None,
                death_end: None,
                slope: None,
            };
            forward.step(&metric, cold.age);
            let off_phi = wrap(forward.phi - focus.phi);
            let miss = (forward.r - focus.r).abs().hypot(focus.r * off_phi);
            // What the round trip is allowed to miss by, and why it is an exponential.
            //
            // The solve's own residual is asserted separately and is 1e-12 M. This is a different
            // and much harsher measurement: a ray sent tens of M back along the outside of r+ and
            // then forward again. Neighbouring outgoing rays there separate at the horizon's own
            // surface gravity - r - r+ grows like exp(kappa_+ t) - so *any* integrator's error on
            // that path grows at the same rate, and no fixed tolerance can hold for an arbitrarily
            // deep image. The bound below carries that factor explicitly, and the three epochs
            // measure 3.8e-5, 1.8e-3 and 5.8e-1 M at ages of 47, 77 and 107 M: a growth of e^0.13
            // per M against kappa_+ = 0.152 for this hole, which is the geometry and not the
            // arithmetic. The check is therefore decisive at the first epoch and increasingly
            // nominal after it, which is exactly what it should be, and it is the residual that
            // pins the deep end.
            let kappa_plus = (metric.outer_horizon() - metric.inner_horizon())
                / (2.0 * (metric.outer_horizon().powi(2) + metric.a * metric.a));
            let allowed = 1e-6 * (kappa_plus * cold.age).exp();
            println!(
                "    the solved ray, re-integrated forward from the emission event, comes back \
                 to within {miss:.2e} M of Alice after {:.1} M of coordinate time (the horizon's \
                 own divergence allows {allowed:.2e} M), and the solve's own residual is \
                 {:.2e} M",
                cold.age, cold.residual
            );
            assert!(
                miss < allowed,
                "the cold emission event must be null-separated from Alice's: the ray lands at \
                 ({}, {}) against ({}, {}), a miss of {miss:.2e} M",
                forward.r,
                forward.phi,
                focus.r,
                focus.phi
            );
            assert!(
                cold.residual < 1e-6,
                "and the solve's own residual must be tight: {}",
                cold.residual
            );

            // And a second cold solve, taken two M of coordinate time earlier and threaded
            // forward to this same event, has to land on it: two different routes through the
            // continuation, one answer.
            let mut earlier = focus.clone();
            let mut earlier_other = other.clone();
            earlier.rewind_to(&metric, focus.t - 2.0);
            earlier_other.rewind_to(&metric, focus.t - 2.0);
            let mut local = as_seen(&metric, &earlier, &earlier_other, None)
                .expect("a cold solve two M earlier");
            for _ in 0..40 {
                play(&metric, &mut earlier, &mut earlier_other, 1, 0.05);
                local = as_seen(&metric, &earlier, &earlier_other, Some(local.seed()))
                    .expect("threading two M forward");
            }
            assert_same_solve("ISCO, threaded from two M back", &local, &cold);
            checked += 1;
        }
        assert_eq!(checked, 3, "every Kerr epoch must have been checked");
    }

    /// The Kerr-Schild Cartesian separation between two observers' present events - the same
    /// length `Probe::separation` drives to nothing, and the one the close-pair test measures its
    /// answers against, because for a close pair the light travel time is that distance.
    fn cartesian_gap(metric: &KerrSchild, a: &Observer, b: &Observer) -> f64 {
        let (xa, ya) = metric.cartesian_position(a.r, a.phi);
        let (xb, yb) = metric.cartesian_position(b.r, b.phi);
        (xa - xb).hypot(ya - yb)
    }

    #[test]
    fn test_two_observers_standing_together_see_each_other_directly_and_not_round_the_hole() {
        // The case a grid-based search cannot do on its own. A dip in the sweep needs three
        // consecutive samples of one generator with the middle one the smallest, and the first
        // sample is taken at age zero, so a crossing nearer than a couple of strides is invisible
        // to it. With the old fixed first stride of 0.25 M that lost every pair closer than about
        // a tenth of an M, and it lost them in two different ways, both measured on an a = 0.90
        // hole with both observers holding station:
        //
        // * at r0 = 10 M, radial gaps of 0.0 to 0.1 M gave `NotConverged` in both directions,
        //   while a gap of 0.2 M solved correctly (0.200 M of age looking inward, 0.299 M looking
        //   outward);
        // * at r0 = 25 and 30 M the same close pairs - radial gaps up to 0.1 M, azimuthal gaps of
        //   0.025 to 0.03 rad - returned `Ok` with the wrong picture: the image whose ray had gone
        //   once round the hole, lambda and age of 73 to 84 M and g = 1.3, where the direct image
        //   is a tenth of an M old and unshifted. Two people standing next to each other watching
        //   each other's distant past is not an accuracy problem.
        //
        // What fixes it is `Solver::geometric_seed`, which is exact in the limit the fan fails in:
        // the local offset (xi^1, xi^2) of the other observer in the focus tetrad gives the
        // arrival direction as atan2(xi^2, xi^1) and the age as its length. The adaptive first
        // stride of `Solver::first_step` then lets the fan bracket the same crossing as a
        // fallback rather than stepping over it.
        //
        // The bound the age is held to is the pair's own Cartesian separation. Light from a source
        // further out arrives ingoing, and an ingoing radial ray of this chart has dr/dt = -1
        // exactly, so the age is then the gap itself; light from a source further in arrives
        // outgoing, at dt/dr = (r + 2M) / (r - 2M), so the age is that factor times the gap - 1.50
        // at r0 = 10 M and 1.14 at r0 = 30 M. Twice the separation is therefore above every direct
        // answer and a factor of eight hundred below the wound one.
        let metric = KerrSchild::new(1.0, 0.90);
        let offsets: [(&str, f64, f64); 5] = [
            ("radial 0.01 M", 0.01, 0.0),
            ("radial 0.05 M", 0.05, 0.0),
            ("radial 0.10 M", 0.10, 0.0),
            ("azimuthal 0.001 rad", 0.0, 0.001),
            ("azimuthal 0.005 rad", 0.0, 0.005),
        ];
        let mut checked = 0usize;
        let mut worst_ratio = 0.0f64;
        for &r0 in &[10.0f64, 30.0] {
            for &(what, dr, dphi) in offsets.iter() {
                let alice = held(&metric, "Alice", r0, 0.0);
                let bob = held(&metric, "Bob", r0 + dr, dphi);
                let gap = cartesian_gap(&metric, &alice, &bob);
                for (who, focus, other) in [
                    ("Alice sees Bob", &alice, &bob),
                    ("Bob sees Alice", &bob, &alice),
                ] {
                    let seen = as_seen(&metric, focus, other, None).unwrap_or_else(|why| {
                        panic!("r0 = {r0} M, {what}, {who}: no image at all ({why:?})")
                    });
                    println!(
                        "r0 = {r0:>4} M, {what:<19}, {who}: gap {gap:.6} M, age {:.6} M \
                         (age/gap {:.4}), lambda {:.6} M, g = {:.6}, windings {}, \
                         residual {:.2e}, emission r = {:.9} M against {:.9} M",
                        seen.age,
                        seen.age / gap,
                        seen.lambda,
                        seen.g,
                        seen.windings,
                        seen.residual,
                        seen.emission.r,
                        other.r
                    );
                    assert_eq!(
                        seen.source,
                        WorldlineSource::Hold,
                        "{who} at r0 = {r0} M, {what}: the light left before the run began"
                    );
                    assert!(
                        (seen.emission.r - other.r).abs() < 1e-6,
                        "{who} at r0 = {r0} M, {what}: a held observer never leaves their radius, \
                         so the emission event is at {} M and not {}",
                        other.r,
                        seen.emission.r
                    );
                    assert!(
                        seen.age < 2.0 * gap,
                        "{who} at r0 = {r0} M, {what}: age {} M against a separation of {gap} M",
                        seen.age
                    );
                    assert!(
                        seen.age < 1.0,
                        "{who} at r0 = {r0} M, {what}: the wound image is eighty M old and this \
                         one is {} M",
                        seen.age
                    );
                    assert!(
                        seen.residual < 1e-6,
                        "{who} at r0 = {r0} M, {what}: residual {}",
                        seen.residual
                    );
                    // The rule the view draws by has to reach the same place: nothing is younger
                    // than the direct image, so the extra sweep must leave it alone.
                    let youngest = as_seen_youngest(&metric, focus, other, None)
                        .expect("the youngest-image rule answers wherever `as_seen` does");
                    assert_same_solve(&format!("r0 = {r0} M, {what}, {who}"), &seen, &youngest);
                    worst_ratio = worst_ratio.max(seen.age / gap);
                    checked += 1;
                }
            }
        }
        println!(
            "{checked} close pairs, all direct: the largest age was {worst_ratio:.4} times the \
             pair's own separation"
        );
        assert_eq!(checked, 20, "two radii, five offsets, both directions");
    }

    #[test]
    fn test_a_pair_held_far_from_the_hole_is_not_walled_off_by_a_fixed_search_ceiling() {
        // The search used to carry an absolute backstop at 128 M on top of the ceiling it reads off
        // the two worldlines, and a pair holding station at r0 >= 140 M starts outside it: `shoot`
        // killed every trial ray the moment it was launched and the solve answered `RaysDied` for a
        // configuration with nothing unusual about it at all. The ceiling is now the pair's own
        // reach alone, so the cost still scales with how far apart the two observers are and the
        // wall is gone.
        //
        // Far from the hole the answer is nearly flat and can be written down. The light Alice
        // receives from Bob, who is further out, is ingoing, and an ingoing radial ray has
        // dr/dt = -1 exactly in this chart, so her age is the gap itself: 1 M. The light Bob
        // receives from Alice is outgoing, at dt/dr = (r + 2M) / (r - 2M) = 302/298, so his age is
        // 1.0134 M.
        let metric = KerrSchild::new(1.0, 0.90);
        let alice = held(&metric, "Alice", 300.0, 0.0);
        let bob = held(&metric, "Bob", 301.0, 0.0);
        for (who, focus, other, want) in [
            ("Alice at 300 M sees Bob at 301 M (ingoing light)", &alice, &bob, 1.0),
            ("Bob at 301 M sees Alice at 300 M (outgoing light)", &bob, &alice, 302.0 / 298.0),
        ] {
            let seen = as_seen(&metric, focus, other, None)
                .unwrap_or_else(|why| panic!("{who}: no image ({why:?})"));
            println!(
                "{who}: age {:.6} M against the flat-space {want:.6} M, lambda {:.6} M, \
                 g = {:.9}, windings {}, residual {:.2e}, emission r = {:.6} M",
                seen.age, seen.lambda, seen.g, seen.windings, seen.residual, seen.emission.r
            );
            assert_eq!(seen.source, WorldlineSource::Hold);
            assert!(
                (seen.emission.r - other.r).abs() < 1e-6,
                "{who}: emission at {} M and not {} M",
                seen.emission.r,
                other.r
            );
            assert!(
                (seen.age - want).abs() < 1e-3,
                "{who}: age {} M against {want} M",
                seen.age
            );
            assert_eq!(seen.windings, 0, "{who}: the direct image winds round nothing");
            assert!(seen.residual < 1e-6, "{who}: residual {}", seen.residual);
            let youngest = as_seen_youngest(&metric, focus, other, None)
                .expect("the youngest-image rule answers here too");
            assert_same_solve(who, &seen, &youngest);
        }
    }
}
