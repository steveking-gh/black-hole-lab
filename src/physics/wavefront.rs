//! One observer's signal, broadcast into the whole of their light cone: exact null geodesics
//! carrying an exact frequency ratio.
//!
//! Nothing in this module knows which observer is transmitting and which is listening. A
//! `SignalField` is handed an *emitter* when a pulse is due and a *receiver* when arrivals are
//! looked for, so the app runs one field for Alice's transmission (received by Bob) and a second
//! for Bob's (received by Alice) out of the same code. Where a concrete example makes the physics
//! easier to state below, it is Alice transmitting and Bob, released later on the same infall,
//! listening.
//!
//! Everything in this module is integrated in the *coordinate time* t of the ingoing Kerr-Schild
//! chart rather than in an affine parameter. That choice is forced by the geometry, not by
//! convenience. Every future-directed null ray in this chart has k^t > 0, so the direction
//!
//!     v^mu = k^mu / k^t = (1, dr/dt, dphi/dt)
//!
//! is always well defined; but the affine vector k itself blows up on an outgoing ray that
//! approaches r- from Region II, because such a ray takes infinite coordinate time to get there.
//! Dividing the affine geodesic equation dk^mu/dlambda = -Gamma^mu_{alpha beta} k^alpha k^beta by
//! (k^t)^2 and using d/dlambda = k^t d/dt gives the non-affine form
//!
//!     d v^i/dt = -Gamma^i_{alpha beta} v^alpha v^beta + v^i Gamma^t_{alpha beta} v^alpha v^beta,
//!     i = r, phi     (the t component is identically zero, since v^t = 1 by construction)
//!
//! which stays perfectly regular there: the direction simply tends to the outgoing principal null
//! direction of r-, whose dr/dt tends to zero. That is the whole reason the stack of outgoing
//! light against the Cauchy horizon can be drawn at all.
//!
//! The scale that the direction throws away is recovered exactly, because it is conserved. With
//! E = -k_t fixed along the ray,
//!
//!     k^t = E / (-g_{t mu} v^mu),
//!     nu(observer u) = -k . u = k^t (-g_{mu nu} v^mu u^nu) = E f,
//!     f(r, v, u) = (-g_{mu nu} v^mu u^nu) / (-g_{t mu} v^mu),
//!
//! so the ratio of the frequency measured at reception to the frequency measured at emission is
//! f_receive / f_emit with E cancelling. Each ray therefore only has to remember the one number
//! f_emit, and no affine normalisation is ever needed. A ray also keeps the direction it left with,
//! v_emit, which is what lets `NullRay::gain_between` re-evaluate f at the emission event against
//! some observer other than the emitter - the shift between two raindrops along the ray, say, which
//! is what the equatorial view colours its fronts by.
//!
//! Which rays pile up on r- and which cross it is settled exactly, and it is not simply the
//! outward half of the cone. Lowering the null condition to the covariant components (k_t, k_r,
//! k_phi) = (-E, k_r, L) with the inverse metric of this chart gives a quadratic in k_r,
//!
//!     Delta k_r^2 + 2 (a L - 2 M r E) k_r + [L^2 - (r^2 + 2 M r) E^2] = 0,
//!
//! whose two roots are the two radial branches. As Delta -> 0 one root stays finite and the other
//! behaves as k_r ~ -2 (a L - 2 M r E) / Delta, and k^t = (1 + 2M/r) E + (2M/r) k_r runs away with
//! it. Delta -> 0 from below inside r+, so the runaway root is the future-directed one exactly when
//! a L - 2 M r- E > 0. Using 2 M r- = r-^2 + a^2 (which is Delta(r-) = 0 rearranged) that condition
//! is
//!
//!     E - Omega_- L < 0,     Omega_- = a / (r-^2 + a^2),
//!
//! with Omega_- the angular velocity of the inner horizon, so the discriminant is the sign of
//! -k . xi for xi = d_t + Omega_- d_phi, the null generator of r-. A ray whose energy relative to
//! that generator is positive crosses r- at finite coordinate time; a ray whose relative energy is
//! negative takes infinite coordinate time and freezes onto the surface. The outgoing principal
//! null direction sits on the second family, and so does a broad arc of every interior light cone
//! (the arc dragged forward in phi, not the arc pointing outward in r, since frame dragging beats
//! aberration inside r+). The stack a later infaller crosses is that family. An infalling worldline
//! sweeps through the whole of it in finite proper time, and that is what the receptions below
//! record for a receiver who is still above r- when the emitter's frozen arcs settle onto it.
//!
//! What becomes of the rest of a pulse - the part that crosses r- instead of freezing onto it - is
//! settled by an exact statement of the same kind, and it is worth stating because it is what the
//! deep interior of the equatorial view is showing. For an equatorial null geodesic of conserved
//! (E, L) the radial motion obeys (r^2 dr/dlambda)^2 = R(r) with
//!
//!     R(r) = [E (r^2 + a^2) - a L]^2 - Delta (L - a E)^2
//!          = r [ E^2 r^3 + (a^2 E^2 - L^2) r + 2 M (L - a E)^2 ],
//!
//! so a ray turns only where the cubic in the bracket vanishes. Near the ring that cubic tends to
//! 2 M (L - a E)^2, which is positive for every ray except the one with L = a E exactly, so the
//! potential is positive in a neighbourhood of r = 0 and the fate of a ray is decided by whether
//! the negative middle term (a^2 E^2 - L^2) r can overcome that constant before r gets small: a ray
//! with no zero above the ring falls monotonically into it, and one with a zero turns there and
//! climbs back out, arbitrarily close to the ring as L/E approaches the boundary between the two.
//! Measured on a whole cone let go at r = 0.45 inside r- at a = 0.90, sixty of the seventy-two rays
//! reach the ring and twelve turn; the twelve then climb to r- and freeze onto it from below, since
//! Region III is where the *outgoing* congruence accumulates on the Cauchy horizon from the inside.
//! `test_a_pulse_inside_r_minus_ends_on_the_ring_or_turns` checks every ray against the turning
//! radius of its own (E, L), and no live ray has ever been found below one.
//!
//! That is also why a front deep inside r- is drawn as a set of arcs rather than as a polygon of
//! chords. Rays a few degrees apart in emission angle end up most of a radian apart in azimuth
//! there - the ones near the ring wind at dphi/dt of about -5 per M against +0.8 for the ones
//! settling onto r- - and Region III is a thin annulus in the Kerr-Schild embedding, so a straight
//! screen chord between two neighbouring rays cuts across it and through the disk inside the ring.
//! The front never goes there; see `spatial_canvas::MAX_ARC_STEP`.
//!
//! Which of the two transmissions the app draws produces such a stack is not symmetric, and the
//! asymmetry is the whole content of running both. Where Bob trails Alice on the same infall his
//! pulses have to chase her inward, and what reaches her is the ingoing part of his cone, whose
//! shift is finite on the branch of r- she crosses; his frozen family settles onto r- behind her,
//! after she has already passed through, so she never meets it. She hears him with an ordinary
//! shift and then stops hearing him at all - and where he is the deeper of the two his light climbs
//! to her instead, which ends the same way, because what ends the transmission is the end of her
//! worldline and not the direction his light had to travel.
//!
//! What a (t, r) diagram can show of any of this is not a ray but a range. A pulse is a closed
//! curve in (r, phi); projected onto the radial axis it is the interval its front spans, and that
//! interval swept up in coordinate time is `Pulse::extent_track`, the wedge the spacetime canvas
//! draws. Its lower edge is bounded exactly by the ingoing edge of the emitter's own light cone,
//! and its upper edge freezes onto r- for every pulse emitted inside r+, which is why the wedges
//! stack against the Cauchy horizon there. While the pulse is being swallowed the lower edge is on
//! the ring itself and not on any one ray, for the reason `Pulse::radial_extent` gives: the rays
//! are a sampling of the front, and the front is a continuum that stands on the ring for the whole
//! of the interval its rays arrive over. A worldline inside a wedge is only in *range* of the
//! pulse; whether the pulse reaches it is a question about azimuth, which the projection has thrown
//! away and only the per-sheet crossing test of `Pulse::scan` answers.

use crate::physics::geodesic::{R_STOP, geodesic_accel};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::tetrad::Tetrad;

/// Directions per pulse the app starts on, and the default of `SignalField::rays_per_pulse`, which
/// is the count an emission actually reads: the whole of the emitter's local light cone sampled at
/// 360/n degrees, which at this n is two and a half.
///
/// The angles are alpha = 2 pi i / n for i in 0..n whatever n is, so alpha = 0, the emitter's own
/// outward radial leg, is ray zero of every pulse at every count, and an even count puts the
/// ingoing radial leg alpha = pi on a ray as well. Nothing downstream reads this constant: a pulse
/// is scanned, measured and drawn by its own `rays.len()`, so pulses of different counts can be in
/// flight together. The tests use it as the count a field emits at unless they say otherwise.
pub const RAYS_PER_PULSE: usize = 144;

/// Pulses kept at once. The oldest is dropped past this, which bounds both the drawing and the
/// integration cost of a long run.
///
/// A pulse whose every ray has died is not retired early. A dead ray keeps its state at the death
/// event and can be revived by `SignalField::step_back`, so dropping a spent pulse would put a hole
/// in the field that stepping backwards could never fill. Dead rays cost nothing to integrate and
/// nothing to draw, and this cap bounds the pile either way; a whole infall is some forty pulses,
/// well under it.
pub const MAX_PULSES: usize = 64;

/// The emitter's proper-time interval between pulses, in units of M. A whole infall from r = 4.5M
/// is about 4.2M of proper time, so this puts of order forty pulses on the wire before the emitter
/// reaches the ring.
///
/// The interval is not just a display rate: it decides whether the transmission reads as continuous
/// on r-. A pulse's frozen arc co-rotates with the inner horizon at Omega_- = a / (r-^2 + a^2) while
/// it waits there, so consecutive arcs are offset in azimuth by roughly Omega_- (dt between
/// emissions), which for this interval is about a quarter of a radian against an arc of order two
/// radians wide (at r = 1.2 and a = 0.65 the frozen arc runs from alpha = 35 to 150 degrees). The
/// arcs therefore overlap and the stack covers every azimuth, so an infaller meets several sheets
/// of it wherever they happen to cross r-. At the half-M interval this started with,
/// consecutive arcs were more than a radian apart and left gaps: whether an infaller met the stack
/// at all was then a matter of where they crossed, which is true of a single pulse but not of a
/// transmission.
pub const EMISSION_INTERVAL_TAU: f64 = 0.1;

/// Radius past which a ray has left the drawn field and is retired. It sits beyond the widest view
/// the app offers, and dr/dt of an escaping ray only grows with r, so nothing that passes it ever
/// comes back into the picture.
pub const R_ESCAPE: f64 = 16.0;

/// Largest |dr| allowed in one integration substep, in units of M.
const MAX_DR_PER_SUBSTEP: f64 = 0.02;

/// Fractional change of the direction (v^r, v^phi) allowed per substep. It is the cap that binds
/// wherever the connection stiffens, which for a ray means the last decade of radius before the
/// ring; 0.004 is what holds the scale-free constant of the motion L/E to better than one part in
/// 1e7 over a run of tens of M (see the tests). It relaxes again once a ray has frozen onto r-,
/// where the direction stops changing.
const DIRECTION_STEP_FRACTION: f64 = 0.004;

/// Local error allowed per substep, in units of M, on the two coordinates (r, phi) that a substep
/// carries the ray through.
///
/// This is the scheme's own estimate of its own truncation error rather than a proxy for it, which
/// is the difference between this integrator and the fixed subdivision it replaces: see
/// `NullRay::integrate`. The two caps above still bound a substep from above, so the tolerance is
/// what binds only where they are not enough - the last decade of radius above the ring, and the
/// approach to r-, which is exactly where a subdivision read off one end of a long interval went
/// wrong. At 1e-9 M per substep a whole infall of some 1e3 substeps carries an accumulated error
/// three orders of magnitude below the 1e-6 M the large-step tests hold the integration to.
const SUBSTEP_TOLERANCE: f64 = 1e-9;

/// Substep below which the error estimate stops meaning anything and the substep is taken as it
/// stands. The local error of a fifth-order scheme at 1e-9 M of coordinate time is of order 1e-45,
/// so whatever the estimate still reports there is floating-point round-off in the difference of
/// two nearly equal states, which shrinking the step only makes worse.
const MIN_SUBSTEP: f64 = 1e-9;

/// Safety budget of substeps per ray per call. Nothing is tuned against it and no accuracy claim
/// rests on it: at the radial cap above it covers some 2000 M of coordinate time in a single call,
/// two orders of magnitude more than the longest interval the app can ask for. It is there so that
/// a pathological state cannot stall a frame, and a ray that exhausts it is retired at its last
/// good event and counted in `SignalField::budget_exhausted` rather than being left standing at a
/// position it never integrated to.
const MAX_SUBSTEPS: usize = 100_000;

/// Smallest coordinate-time spacing between stored points of a pulse's radial-extent track.
///
/// The track is extended at every `SignalField::advance`, subject only to this minimum, rather than
/// on a cadence of its own. What the spacing has to resolve is not the smooth part of the wedge -
/// the two edges are slow curves - but the moments at which the front's inner edge arrives at and
/// leaves the ring, which is a step from a falling radius to R_STOP and back. Deaths at the ring
/// come in a burst: sixty of the seventy-two rays of a pulse let go at r = 0.45 reach the ring, and
/// they do it inside a couple of M. At the 0.2 M this started with, several of those events fell
/// between two samples and were joined by one long diagonal chord; at 0.02 M a played frame of
/// 1/50 M stores a point every frame and nothing is joined across an event that the rays resolved.
const TRACK_MIN_DT: f64 = 0.02;

/// Points past which a track is thinned rather than stopped: see `Pulse::extend_track`. At
/// `TRACK_MIN_DT` this is 80 M of coordinate time before the first thinning, which is more than a
/// whole infall, and the spacing doubles at each one after that.
const TRACK_MAX_POINTS: usize = 4000;

/// State vector of a ray in coordinate time: y = (r, phi, v^r, v^phi).
type RayState = [f64; 4];

/// The bilinear form -g_{mu nu} a^mu b^nu at radius r.
fn minus_inner(metric: &KerrSchild, r: f64, a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let g = metric.metric_components(r);
    let mut sum = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            sum += g[i][j] * a[i] * b[j];
        }
    }
    -sum
}

/// f(r, v, u) = (-g_{mu nu} v^mu u^nu) / (-g_{t mu} v^mu), the frequency an observer with
/// 4-velocity u measures on a ray of direction v, divided by that ray's conserved energy E = -k_t.
///
/// The denominator is -k_t / k^t = E / k^t, which is strictly positive for a future-directed ray,
/// so dividing by it is safe everywhere in the chart, horizons included. Because E is the same
/// number at every event of a ray, the quotient of this factor at two events on the same ray is
/// the exact frequency ratio between them: that is all `NullRay` has to carry.
pub(crate) fn f_factor(metric: &KerrSchild, r: f64, v: &[f64; 3], u: &[f64; 3]) -> f64 {
    let g = metric.metric_components(r);
    let e_over_kt = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
    minus_inner(metric, r, v, u) / e_over_kt
}

/// One exact null geodesic of the coded metric, carried as a direction rather than as an affine
/// tangent, plus the emission-frequency factor that turns it into a measurable shift.
#[derive(Debug, Clone, Copy)]
pub struct NullRay {
    /// The field clock this ray is carried on. For a live ray it is the coordinate time of the
    /// ray's current event; a dead ray goes on being carried on it without moving, so that
    /// `step_back` can ask each ray whether it was still alive at the time being stepped back to.
    pub t: f64,
    /// Radius of the ray's current event.
    pub r: f64,
    /// Azimuth of the ray's current event, unwrapped (never folded into [0, 2 pi)), so that a
    /// wavefront polyline stays continuous however many turns frame dragging puts into it.
    pub phi: f64,
    /// dr/dt of the ray at its current event.
    pub dr_dt: f64,
    /// dphi/dt of the ray at its current event.
    pub dphi_dt: f64,
    /// `f_factor` evaluated at the emission event against the emitter's 4-velocity. The frequency
    /// ratio anywhere later on the ray is the current factor divided by this one.
    pub f_emit: f64,
    /// The ray's direction v^mu = (1, dr/dt, dphi/dt) at the *emission* event, kept as it was let
    /// go rather than recomputed from the current state.
    ///
    /// `f_emit` answers one question about that event - what the emitter themselves measured - and
    /// it answers it in a form that has already thrown the direction away. Anything that wants the
    /// shift between some *other* observer at the emission event and some other observer here has
    /// to evaluate `f_factor` at the emission event again, against a different 4-velocity, and
    /// that needs the direction the ray had there. Keeping it costs three numbers per ray and
    /// makes `gain_between` exact rather than a reconstruction: the emission event is not on the
    /// integrated track any more once the ray has moved, and nothing else stores it.
    ///
    /// The radius of the emission event is not carried here because the pulse already owns it, as
    /// `Pulse::emitted_r`, and every ray of a pulse left the same event.
    pub v_emit: [f64; 3],
    /// Coordinate time at which the ray left the field (r > `R_ESCAPE`) or reached the ring
    /// (r < R_STOP), or None while it is still running.
    ///
    /// The rest of the state is left standing at that death event rather than discarded, which is
    /// what makes the death reversible: a `step_back` over an interval reaching past `death_t`
    /// clears this field and integrates the stored state backwards out of the boundary again.
    pub death_t: Option<f64>,
    /// Which boundary that death was at, None exactly when `death_t` is None. Both are set and
    /// cleared together, through `NullRay::die` and `NullRay::revive`, so the pair cannot drift.
    ///
    /// The two boundaries cannot be told apart from `death_t` alone, and the radius the ray was
    /// left standing at is no substitute: a ray retired by `RayEnd::Unintegrable` stands wherever
    /// it happened to be, so a radial test would have to guess. `Pulse::radial_extent` has to know
    /// which end of the front a death happened at, so the integrator records it.
    pub death_end: Option<RayEnd>,
}

/// Which boundary of the drawn field a dead ray reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RayEnd {
    /// R_STOP, the ring: the ray was swallowed. In the continuum it reaches r = 0; R_STOP is the
    /// radius at which this chart's equation is left alone and the ray retired, and it is what the
    /// drawn front's inner edge stands on.
    Ring,
    /// `R_ESCAPE`: the ray climbed out of the drawn field and is not coming back.
    Escape,
    /// Neither boundary. The state stopped being evaluable at the round-off floor, or the substep
    /// budget of `MAX_SUBSTEPS` ran out; either way the ray was retired where it last stood rather
    /// than at a boundary, and it says nothing about either edge of the front.
    Unintegrable,
}

/// What one call to `NullRay::step` or `NullRay::step_back` did to the ray.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RayStep {
    /// The interval was integrated. The ray may have died at a boundary inside it, which is a
    /// physical outcome and not a failure of the integration.
    Integrated,
    /// The substep budget of `MAX_SUBSTEPS` ran out before the interval did, so the ray has been
    /// retired at the last event it actually reached rather than left standing at a position
    /// nothing integrated to. `SignalField::budget_exhausted` counts these; a run that reports any
    /// of them is a run whose field is missing rays, which is why it is counted rather than logged.
    BudgetExhausted,
}

impl NullRay {
    /// The ray an observer with 4-velocity `u_emitter` sends at local angle alpha, alpha = 0 being
    /// their own outward radial direction and alpha = pi/2 their local +phi direction.
    ///
    /// `Tetrad::null_direction` returns k = e0 + cos(alpha) e1 + sin(alpha) e2, which is null by
    /// orthonormality; dividing by k^t drops to the direction v, which is what gets integrated.
    pub fn from_local_direction(
        metric: &KerrSchild,
        t: f64,
        r: f64,
        phi: f64,
        tetrad: &Tetrad,
        alpha: f64,
        u_emitter: &[f64; 3],
    ) -> Self {
        let k = tetrad.null_direction(alpha);
        let (dr_dt, dphi_dt) = tetrad.coordinate_velocity(&k);
        let v = [1.0, dr_dt, dphi_dt];
        Self {
            t,
            r,
            phi,
            dr_dt,
            dphi_dt,
            f_emit: f_factor(metric, r, &v, u_emitter),
            v_emit: v,
            death_t: None,
            death_end: None,
        }
    }

    /// Whether the ray is still running. A dead ray keeps its state at the death event, so this is
    /// a question about the ray's clock rather than about whether its state means anything.
    pub fn alive(&self) -> bool {
        self.death_t.is_none()
    }

    /// Whether this ray was swallowed by the ring, as against having escaped the drawn field or
    /// been retired by the integrator. False for a live ray.
    pub fn died_at_ring(&self) -> bool {
        self.death_end == Some(RayEnd::Ring)
    }

    /// Whether this ray left the drawn field at `R_ESCAPE`. False for a live ray.
    pub fn died_escaping(&self) -> bool {
        self.death_end == Some(RayEnd::Escape)
    }

    /// The ray's direction v^mu = (1, dr/dt, dphi/dt).
    pub fn direction(&self) -> [f64; 3] {
        [1.0, self.dr_dt, self.dphi_dt]
    }

    /// nu(observer) / nu(emission) for this ray: `f_factor` here over `f_emit` there. Both the
    /// affine scale of the ray and its conserved energy cancel out of the quotient, so this is the
    /// exact measured shift, finite and positive at and inside both horizons.
    pub fn frequency_ratio(&self, metric: &KerrSchild, u_observer: &[f64; 3]) -> f64 {
        if self.f_emit.abs() < 1e-300 {
            return 1.0;
        }
        f_factor(metric, self.r, &self.direction(), u_observer) / self.f_emit
    }

    /// nu(`u_now_observer` here) / nu(`u_emit_observer` at the emission event) for this ray: the
    /// shift between two observers of the caller's choosing, neither of them necessarily the
    /// emitter.
    ///
    /// `frequency_ratio` asks what this ray's frequency is *now* against what the emitter measured
    /// as it left, which is the question a reception asks. This asks the other one: hold the ray
    /// fixed and change the observer at *both* ends. The emitter's own frame then drops out of the
    /// answer entirely, and with `u_emit_observer` and `u_now_observer` taken from one congruence -
    /// the raindrops, say, the E = 1, L = 0 infallers from rest at infinity, which exist at every
    /// radius including inside both horizons - what comes back is the ordinary
    /// gravitational-plus-Doppler shift between two members of that congruence along the ray.
    ///
    /// Both ends are the same `f_factor` construction as everywhere else in this module, so the
    /// ray's conserved energy E and its affine scale cancel out of the quotient exactly, and the
    /// result is finite and positive at and inside both horizons. Two properties are worth naming
    /// because a display leans on them. At the emission event itself the two f_factors are the same
    /// number computed twice - same radius, same direction, same observer - so the gain is exactly
    /// 1 for every ray of a fresh pulse, whatever the emitter was doing. And for a ray of the
    /// frozen family, E - Omega_- L < 0, it grows like exp(kappa_- t) without bound, because the
    /// raindrops keep falling through a surface the ray never crosses.
    ///
    /// `r_emit` is the radius of the emission event, which the ray does not carry: it belongs to
    /// the pulse (`Pulse::emitted_r`), and every ray of a pulse shares it.
    pub fn gain_between(
        &self,
        metric: &KerrSchild,
        r_emit: f64,
        u_emit_observer: &[f64; 3],
        u_now_observer: &[f64; 3],
    ) -> f64 {
        let f_there = f_factor(metric, r_emit, &self.v_emit, u_emit_observer);
        if f_there.abs() < 1e-300 {
            return 1.0;
        }
        f_factor(metric, self.r, &self.direction(), u_now_observer) / f_there
    }

    /// The ray's energy relative to the null generator of the inner horizon, per unit k^t:
    ///
    ///     E - Omega_- L,     Omega_- = a / (r-^2 + a^2),
    ///
    /// which is -k . xi for xi = d_t + Omega_- d_phi, the Killing generator of r-, divided by the
    /// positive scale k^t. Its *sign* is what matters and the scale cannot change it. As the module
    /// header derives, a ray with positive relative energy crosses r- at finite coordinate time and
    /// a ray with negative relative energy takes infinite coordinate time, freezing onto the surface
    /// instead: this one number sorts every ray of a pulse into the crossing family or the frozen
    /// family, and it is conserved, so it may be evaluated wherever the ray happens to be.
    ///
    /// A hole with no spin has r- = 0 and a = 0, so xi degenerates to d_t and Omega_- is taken as
    /// zero: there is no inner horizon to freeze onto and every ray crosses.
    pub fn inner_horizon_energy(&self, metric: &KerrSchild) -> f64 {
        let omega_minus = metric.inner_horizon_omega();
        let g = metric.metric_components(self.r);
        let v = self.direction();
        let e = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
        let l = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
        e - omega_minus * l
    }

    /// Whether the ray belongs to the frozen family: `inner_horizon_energy` negative, so the ray
    /// approaches r- from outside as r - r- ~ exp(-kappa_- t), co-rotating at Omega_-, and never
    /// crosses this branch of the Cauchy horizon at any finite coordinate time.
    ///
    /// The sign is the exact criterion at every radius, not only inside r+, so no radial test
    /// guards it. It is conserved along the ray, but evaluating it costs a metric, so a caller
    /// drawing a whole field should ask once per ray per frame and keep the answer.
    pub fn frozen(&self, metric: &KerrSchild) -> bool {
        self.inner_horizon_energy(metric) < 0.0
    }

    /// The ray's two conserved Killing constants per unit k^t, read off the direction at whatever
    /// event the ray is standing on:
    ///
    ///     E / k^t = -g_{t mu} v^mu,     L / k^t = g_{phi mu} v^mu.
    ///
    /// E and L themselves are conserved and k^t is not, so the pair is only defined up to that one
    /// positive common factor. Everything asked of it here - L/E, the inner-horizon energy, the
    /// radial potential of `turns_between` - is homogeneous in (E, L), so the factor cancels out of
    /// the answer and the ray never needs an affine normalisation it cannot carry.
    pub fn constants(&self, metric: &KerrSchild) -> (f64, f64) {
        let g = metric.metric_components(self.r);
        let v = self.direction();
        let e = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
        let l = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
        (e, l)
    }

    /// L / E = g_{phi mu} v^mu / (-g_{t mu} v^mu), the one scale-free constant of the motion a
    /// direction can carry. Conserved exactly along the ray, which is what the tests check.
    #[allow(dead_code)] // the integration's conservation diagnostic; the tests are its caller
    pub fn l_over_e(&self, metric: &KerrSchild) -> f64 {
        let (e, l) = self.constants(metric);
        l / e
    }

    /// Whether the ray has a radial turning point strictly between `lo` and `hi`: whether, in
    /// other words, there is anything in that stretch of radius to stop it.
    ///
    /// This is exact and in closed form, with no sampling and no root finder. The radial potential
    /// of an equatorial null geodesic of conserved (E, L) is
    ///
    ///     R(r) = [E (r^2 + a^2) - a L]^2 - Delta (L - a E)^2 = r c(r),
    ///     c(r) = A r^3 + B r + C,   A = E^2,  B = a^2 E^2 - L^2,  C = 2 M (L - a E)^2,
    ///
    /// the motion needs R >= 0, and r > 0 everywhere in this chart's equator, so R and c have the
    /// same zeros there and c alone decides. c carries no quadratic term, so c'(r) = 3 A r^2 + B
    /// vanishes at most once on r > 0 - at r* = sqrt(-B / 3A), and only when B < 0. c is therefore
    /// unimodal on the positive axis: falling to a single minimum at r* and rising after it, or
    /// simply rising from c(0) = C >= 0 when B >= 0. A unimodal function is positive across the
    /// whole of [lo, hi] exactly when it is positive at both ends and at its minimum if that lies
    /// between them, so three evaluations settle it.
    ///
    /// A ray standing at r has R(r) >= 0 by construction, so a zero found below it is the radius it
    /// turns at on the way in and a zero above it the radius it turns at on the way out. Both A and
    /// C are squares, and the whole of c is homogeneous of degree two in (E, L), so the per-unit-
    /// k^t constants of `constants` give the same zeros as affinely normalised ones would.
    fn turns_between(&self, metric: &KerrSchild, lo: f64, hi: f64) -> bool {
        if lo >= hi {
            return false;
        }
        let (e, l) = self.constants(metric);
        let q = l - metric.a * e;
        let (a3, b1, c0) =
            (e * e, metric.a * metric.a * e * e - l * l, 2.0 * metric.m * q * q);
        let c = |r: f64| a3 * r * r * r + b1 * r + c0;
        let mut lowest = c(lo).min(c(hi));
        if a3 > 0.0 && b1 < 0.0 {
            let r_star = (-b1 / (3.0 * a3)).sqrt();
            if r_star > lo && r_star < hi {
                lowest = lowest.min(c(r_star));
            }
        }
        lowest <= 0.0
    }

    /// Whether this ray will reach the ring: its radial potential has no zero between the ring
    /// and the radius it now stands at, and it is either falling already or climbing towards a
    /// turning point it can actually get to, from which it comes back down.
    ///
    /// Exact. Both halves are statements about the ray as it stands - the sign of dr/dt, and the
    /// closed-form potential of the conserved (E, L) that `turns_between` evaluates - so the fate
    /// is settled without integrating anything, and it cannot change: (E, L) are constants, a ray
    /// with no turning point below it never reverses on the way in, and a ray with one above it
    /// always does on the way out. The second clause is not a corner case. Inside r- a pulse's
    /// rays with |L| just above a|E| are sent outward, climb a few hundredths of an M to the outer
    /// zero of their potential over more than a full M of coordinate time, and then fall to the
    /// ring; with the cone sampled at 2.5 degrees, the default count's spacing, one such ray sits next to the last ray that fell
    /// straight in, and reading it as "not ring-bound" while it climbed dropped the drawn front's
    /// inner edge off the ring in the middle of the swallowing. "The ring" here is R_STOP, the
    /// radius at which this chart's equation is left alone and the ray retired; see
    /// `NullRay::integrate`. A ray whose outer turning point lies beyond `R_ESCAPE` leaves the
    /// drawn field before it turns, so it is not counted.
    ///
    /// *Which* zeros count, and indeed whether any of them is the whole answer, is a question
    /// about the horizons rather than about the potential alone, and that is what the horizons are
    /// doing in an otherwise closed-form test. Four facts about this chart bound it:
    ///
    /// * nothing between r- and r+ ever moves outward. Delta < 0 there, so
    ///   R(r) = [E (r^2 + a^2) - a L]^2 - Delta (L - a E)^2 is strictly positive - the trapped
    ///   region holds no turning point at all - and every null ray in it has dr/dt < 0.
    /// * a ray inside r- that moves outward asymptotes to r- from below, r- - r ~ exp(-kappa_- t),
    ///   and never crosses it at finite coordinate time. A zero of its own potential above r- is
    ///   a turning point it never gets to.
    /// * nothing inside r+ ever reaches `R_ESCAPE`, which is the same statement at the other
    ///   horizon: the outward branch of the interior accumulates on r- or on r+ and crosses
    ///   neither.
    /// * anything standing outside r- has to cross r- to reach the ring, and only the crossing
    ///   family does that at finite coordinate time. The frozen family, E - Omega_- L < 0, closes
    ///   on r- as exp(-kappa_- t) and stops there for good, so no ray of it reaches the ring
    ///   however clear the potential below r- happens to be. `NullRay::frozen` is that sign, and
    ///   it is exact and conserved, so it costs one metric and cannot be wrong about a ray it is
    ///   asked of anywhere.
    ///
    /// So a climbing ray is turned back only by a zero on its own side of the horizons: one in
    /// (r, r-) if it is inside r-, and one in (r, `R_ESCAPE`) if it is outside r+, where a ray
    /// that turns and falls goes through both horizons and, having no zero below, reaches the
    /// ring. Between the horizons dr/dt < 0 always, so the second clause is unreachable there;
    /// the arm below is written so that it cannot be applied, and what decides the fate of a ray
    /// in the trapped region is the potential below it and the family it belongs to.
    ///
    /// The horizon-blind form of this test broke at late times, and visibly. A ray frozen on r-
    /// from above sits at r - r- ~ exp(-kappa_- t), and at the default a = 0.90 (kappa_- =
    /// 0.386/M, r- = 0.564) that offset drops below the spacing of f64 near r- at about t = 100 M.
    /// Every frozen ray then rounds onto r- and some land an ulp below it, where Delta has changed
    /// sign and the equation hands them a dr/dt of the other sign. Counting a zero out beyond r+ -
    /// which such a ray can never reach - called them ring-bound, and so did reading the dr/dt of
    /// one that had not yet crossed as "falling, with nothing below to stop it". Their dead
    /// neighbours supplied the other half of `Pulse::radial_extent`'s pin, and every wedge on the
    /// (t, r) diagram snapped back to full width in a band at t ~ 100 M. See
    /// `test_a_ray_frozen_onto_r_minus_has_no_fate` and `test_late_wedges_never_widen_again`.
    pub fn ring_bound(&self, metric: &KerrSchild) -> bool {
        if !self.alive() || self.turns_between(metric, R_STOP, self.r) {
            return false;
        }
        // Everything outside r- has to cross r- to reach the ring, and only the crossing family
        // does that at finite coordinate time: `frozen` is the exact, conserved sign that sorts
        // the two, and it is the same horizon statement as the three above rather than a new one.
        if self.r >= metric.inner_horizon() && self.frozen(metric) {
            return false;
        }
        if self.dr_dt < 0.0 {
            return true;
        }
        // Climbing, so it needs somewhere to turn - and the only turning points it can reach are
        // the ones on its own side of the horizons. The middle arm is the trapped region, where
        // this line is never reached in exact arithmetic and must not be acted on when round-off
        // near r- has put a frozen ray's dr/dt on the wrong side of zero.
        let (rm, rp) = (metric.inner_horizon(), metric.outer_horizon());
        if self.r < rm {
            self.turns_between(metric, self.r, rm)
        } else if self.r > rp {
            self.turns_between(metric, self.r, R_ESCAPE)
        } else {
            false
        }
    }

    /// The mirror of `ring_bound` at the outer boundary: the ray stands strictly outside r+, has
    /// no turning point between there and `R_ESCAPE`, and is either climbing already or falling
    /// towards a turning point still outside r+ from which it climbs back out, so it leaves the
    /// drawn field.
    ///
    /// Moving outward is not on its own enough, even well outside r+, which is why the same exact
    /// test is used at both ends. An equatorial null geodesic with enough angular momentum turns
    /// round at the outer root of its own potential - that root is what the photon orbits are made
    /// of - so a ray climbing at r = 2 M can perfectly well fall back without ever reaching 16 M.
    ///
    /// Standing outside r+ is not enough either, but standing at or inside it is disqualifying,
    /// which is the horizon entering the test for the reasons `ring_bound` sets out: nothing
    /// between r- and r+ ever moves outward, a ray inside r- that climbs asymptotes to r- from
    /// below without crossing it, and so nothing at or inside r+ ever reaches `R_ESCAPE`, whatever
    /// its potential does out there. For the same reason the falling arm looks for its turning
    /// point in (r+, r) rather than all the way down to the ring: a zero inside r+ is a zero the
    /// ray would have to come back out through, and it does not.
    ///
    /// This is the other half of the late-time failure `ring_bound` describes: a frozen ray that
    /// has rounded onto r- near t = 100 M is handed a dr/dt of the wrong sign there, and the
    /// horizon-blind test read the outer zeros of its potential as an escape from the deep
    /// interior, re-pinning the wedge's upper edge to `R_ESCAPE`.
    pub fn escape_bound(&self, metric: &KerrSchild) -> bool {
        let rp = metric.outer_horizon();
        if !self.alive() || self.r <= rp || self.turns_between(metric, self.r, R_ESCAPE) {
            return false;
        }
        self.dr_dt > 0.0 || self.turns_between(metric, rp, self.r)
    }

    /// Advance the ray by dt of coordinate time along the non-affine geodesic equation of the
    /// module header, adaptively substepped: see `NullRay::integrate`.
    ///
    /// An outgoing ray closing on r- from above simply freezes, its dr/dt decaying to zero like
    /// exp(-kappa_- t). That is the correct behaviour and not a stall: the ray genuinely never
    /// crosses this branch of the Cauchy horizon at finite t. It is also cheap, because a frozen
    /// ray stops turning and the substep control lets the step grow again.
    ///
    /// A dead ray is carried on the clock without moving. Its state stays at the death event and
    /// `death_t` goes on naming that event, so the interval between them is exactly the time a
    /// step backwards has to skip before it starts integrating.
    pub fn step(&mut self, metric: &KerrSchild, dt: f64) -> RayStep {
        if dt <= 0.0 {
            return RayStep::Integrated;
        }
        let end_t = self.t + dt;
        let outcome = if self.alive() {
            self.integrate(metric, dt)
        } else {
            RayStep::Integrated
        };
        // The clock runs on even if the ray died part way through the step: `death_t` records the
        // event it died at, `t` records what time it is.
        self.t = end_t;
        outcome
    }

    /// Carry the ray back by dt of coordinate time, dt > 0 meaning "go back by dt".
    ///
    /// `ray_rhs` is a first-order system in (r, phi, v^r, v^phi) with no explicit dependence on t,
    /// so it is time symmetric: the same substepped scheme run with a negative step integrates the
    /// same geodesic in the other direction rather than solving a different problem. The substep
    /// control is local - each substep is sized by the state it starts at and by the error the
    /// scheme estimates for it - so the way back is subdivided by what the ray is doing at each
    /// point of the path, exactly as the way out was, and the retrace is left with nothing but the
    /// scheme's own error. The tests measure what that comes to.
    ///
    /// A death is reversible too. A ray that reached the ring or left the field inside the interval
    /// being undone is put back on its feet at the death event, where its full state was kept, and
    /// integrated backwards from there to the target time; the remainder of the interval is time it
    /// spent already dead, over which it covered no distance. A ray that died before the target
    /// time stays dead, with its clock wound back like everything else.
    pub fn step_back(&mut self, metric: &KerrSchild, dt: f64) -> RayStep {
        if dt <= 0.0 {
            return RayStep::Integrated;
        }
        let target_t = self.t - dt;
        let outcome = match self.death_t {
            None => self.integrate(metric, -dt),
            Some(death_t) if death_t > target_t => {
                self.revive();
                self.t = death_t;
                self.integrate(metric, target_t - death_t)
            }
            Some(_) => RayStep::Integrated,
        };
        // The clock is the field's, not the ray's own: a ray left dead by the retrace is carried
        // back on it like any other dead ray, with its state standing at the event it died on.
        self.t = target_t;
        outcome
    }

    /// The one integrator behind `step` and `step_back`: an adaptively substepped Dormand-Prince
    /// 5(4) integration of `ray_rhs` over a signed interval, leaving (r, phi, v^r, v^phi, t) at the
    /// event that interval ends on.
    ///
    /// Every substep is sized from the state it starts at, never from the state the whole interval
    /// starts at, and by three conditions at once:
    ///
    /// * `MAX_DR_PER_SUBSTEP` of radius at the local dr/dt, and `DIRECTION_STEP_FRACTION` of the
    ///   direction's own size at the local rate of turn (`substep_cap`). These are geometric caps:
    ///   they keep a substep short enough that the connection the scheme samples is the connection
    ///   along the path, whatever the error estimate says.
    /// * the scheme's own embedded error estimate, `SUBSTEP_TOLERANCE` on (r, phi). The Dormand-
    ///   Prince tableau carries a fourth-order solution alongside its fifth-order one out of the
    ///   same six evaluations, so the difference between them is a free estimate of the local
    ///   truncation error; a substep whose estimate exceeds the tolerance is shrunk by the usual
    ///   fifth-order controller and retried, and the fifth-order state is what is kept. Step
    ///   doubling - one step of h against two of h/2 - would give the same control for eleven
    ///   evaluations per accepted substep instead of six, and at fourth order instead of fifth,
    ///   which is why the embedded pair is used here: the whole field is integrated every frame.
    ///
    /// That locality is the point. The subdivision this replaces read one substep count off the
    /// state at the start of the interval - which for a step backwards is the far end of the path
    /// being retraced - and applied it to the whole of it. A hand step of 0.1 M was then integrated
    /// at the resolution the ray needed where it happened to be standing rather than where it was
    /// going, and a Distance-mode step of a few M could leave a ray that had gone deep inside at
    /// r = 1.07 where it belonged at r = 2.46. Played frames were accurate only because they are a
    /// fiftieth of an M long. Nothing here is tuned to the caller's step size at all now.
    ///
    /// The two boundaries of the field are not the same kind of thing and are not handled the
    /// same way.
    ///
    /// `R_ESCAPE` is a drawing limit in ordinary geometry: a forward step whose substep lands
    /// beyond it retires the ray at the last event inside, and a backward step ignores it, because
    /// a ray running backwards is retracing ground it has already covered and the test would only
    /// re-kill it at the boundary it is climbing away from.
    ///
    /// R_STOP is the ring, where the equation itself gives out, so nothing is ever evaluated below
    /// it in either direction. A substep any of whose stages would fall below R_STOP is rejected
    /// and halved, exactly as one that fails the error test is, and a ray that cannot take even a
    /// `MIN_SUBSTEP` without reaching under the ring has arrived: it is retired where it stands,
    /// which is above R_STOP and integrated with nothing but the true equation. That is also what
    /// makes the death reversible - the state kept is one the geodesic actually passed through, so
    /// the backward integration retraces from it with nothing but the scheme's own error - and it
    /// is why `ray_rhs` needs no clamp. A ray whose radial turning point is inside a substep of
    /// the ring is retired there rather than turned; at R_STOP = 0.02 M that is the ring swallowing
    /// it, which `test_a_pulse_inside_r_minus_ends_on_the_ring_or_turns` measures against the exact
    /// turning radius of each ray's own (E, L).
    fn integrate(&mut self, metric: &KerrSchild, dt: f64) -> RayStep {
        let forward = dt > 0.0;
        let sign = if forward { 1.0 } else { -1.0 };
        let end_t = self.t + dt;
        let mut y: RayState = [self.r, self.phi, self.dr_dt, self.dphi_dt];
        let mut k = ray_rhs(metric, &y);
        let mut remaining = dt.abs();
        // The first proposal is the whole interval; the caps cut it down on the first substep, and
        // the controller carries a working size from one substep to the next after that.
        let mut proposal = remaining;
        let mut budget = MAX_SUBSTEPS;

        while remaining > 0.0 {
            // Derived from what is left rather than accumulated, so that the substeps cannot drift
            // the clock and the last of them lands on `end_t` exactly.
            let t_now = end_t - sign * remaining;
            let mut h = proposal.min(substep_cap(&y, &k)).min(remaining);
            let (next, slope) = loop {
                if budget == 0 {
                    self.die(&y, t_now, RayEnd::Unintegrable);
                    return RayStep::BudgetExhausted;
                }
                budget -= 1;
                let (candidate, slope, error) = match ray_dopri5(metric, &y, &k, sign * h) {
                    Ok(step) => step,
                    Err(fail) => {
                        // A stage of this substep could not be evaluated. Shrink it until one can
                        // be; when even the smallest substep cannot, the ray is retired at the last
                        // event it did reach, at the ring if that is what stopped it.
                        if h <= MIN_SUBSTEP {
                            let end = match fail {
                                StageFail::BelowRing => RayEnd::Ring,
                                StageFail::NotFinite => RayEnd::Unintegrable,
                            };
                            self.die(&y, t_now, end);
                            return RayStep::Integrated;
                        }
                        h *= 0.5;
                        continue;
                    }
                };
                let estimate = error[0].abs().max(error[1].abs());
                let sane = candidate.iter().all(|c| c.is_finite()) && estimate.is_finite();
                if sane && (estimate <= SUBSTEP_TOLERANCE || h <= MIN_SUBSTEP) {
                    proposal = h * substep_factor(estimate);
                    break (candidate, slope);
                }
                if !sane && h <= MIN_SUBSTEP {
                    // Nothing finite can be made of this state even at the round-off floor. The
                    // ray is retired where it last stood, as it would be at a boundary - but not
                    // *at* either boundary, so the front's edges are told nothing by it.
                    self.die(&y, t_now, RayEnd::Unintegrable);
                    return RayStep::Integrated;
                }
                h *= if sane { substep_factor(estimate) } else { 0.5 };
            };

            if forward && next[0] > R_ESCAPE {
                self.die(&y, t_now, RayEnd::Escape);
                return RayStep::Integrated;
            }
            y = next;
            k = slope;
            remaining -= h;
        }
        self.commit(&y, end_t);
        RayStep::Integrated
    }

    /// Retire the ray at an integrated state, recording both when it died and which boundary of
    /// the field it died at. The two are set here and cleared in `revive`, and nowhere else, so
    /// `death_end` is Some exactly when `death_t` is.
    fn die(&mut self, y: &RayState, t: f64, end: RayEnd) {
        self.commit(y, t);
        self.death_t = Some(t);
        self.death_end = Some(end);
    }

    /// Put a dead ray back on its feet, which is what a `step_back` reaching past its death does.
    fn revive(&mut self) {
        self.death_t = None;
        self.death_end = None;
    }

    /// Move the ray onto an integrated state at a given coordinate time.
    fn commit(&mut self, y: &RayState, t: f64) {
        self.t = t;
        self.r = y[0];
        self.phi = y[1];
        self.dr_dt = y[2];
        self.dphi_dt = y[3];
    }
}

/// dy/dt for y = (r, phi, v^r, v^phi) with v^t = 1.
///
/// `geodesic_accel` returns acc^mu = -Gamma^mu_{alpha beta} v^alpha v^beta, so the non-affine
/// equation d v^i/dt = -Gamma^i vv + v^i Gamma^t vv is simply acc[i] - v^i acc[t].
///
/// The radius is used as it stands: this is the true equation at every point it is asked about,
/// and it is never asked about a point below R_STOP. `ray_dopri5` refuses to build a stage there
/// and `NullRay::integrate` shrinks the substep until none of them is, so the assertion below is
/// the statement of an invariant rather than a guard. Clamping r instead, which is what this used
/// to do, fed a different equation into a substep that could still be accepted, and left the ray's
/// death standing on a state partly integrated with it.
fn ray_rhs(metric: &KerrSchild, y: &RayState) -> RayState {
    debug_assert!(y[0] >= R_STOP, "ray_rhs evaluated below the ring at r = {}", y[0]);
    let v = [1.0, y[2], y[3]];
    let acc = geodesic_accel(metric, y[0], &v);
    [y[2], y[3], acc[1] - y[2] * acc[0], acc[2] - y[3] * acc[0]]
}

/// One Dormand-Prince 5(4) step of `ray_rhs`, reusing the slope k1 already evaluated at y.
///
/// Returns the fifth-order state at y + h, the slope of `ray_rhs` there, and the difference between
/// the fifth-order state and the embedded fourth-order one, component by component: the local error
/// estimate the substep control of `NullRay::integrate` runs on.
///
/// Fails instead if any stage of the step could not be evaluated: `StageFail::BelowRing` if a stage
/// would have fallen below R_STOP, the ring, where the equation gives out, and
/// `StageFail::NotFinite` if a stage went non-finite, which is the round-off floor rather than a
/// boundary of the spacetime. The seventh stage is the step's own result, so this covers the state
/// the step would produce as well as the interior stages that produce it: a step that comes back is
/// a step that stayed in the geometry throughout. The caller shrinks and retries, and retires the
/// ray when even `MIN_SUBSTEP` will not fit - at the ring, or as `RayEnd::Unintegrable`, according
/// to which of the two stopped it.
///
/// The seventh stage is evaluated at the fifth-order state itself, so the slope returned is both
/// the last stage of this step's error estimate and the first stage of the next step (the
/// tableau's first-same-as-last property). One accepted substep therefore costs six evaluations
/// of `ray_rhs`, against four for the classical RK4 this replaces and eleven for the same error
/// control by step doubling.
fn ray_dopri5(
    metric: &KerrSchild,
    y: &RayState,
    k1: &RayState,
    h: f64,
) -> Result<(RayState, RayState, RayState), StageFail> {
    // Rows two to seven of the tableau's a_{ij}. The last row is the fifth-order solution's own
    // weights, which is what makes its stage argument the new state.
    const A: [[f64; 6]; 6] = [
        [1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0],
        [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0],
        [19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0, 0.0],
        [9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0, 0.0],
        [35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0],
    ];
    // b - b*: the fifth-order weights less the embedded fourth-order ones.
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
        // A stage that has gone non-finite is no more evaluable than one below the ring, and
        // both are the caller's cue to shrink and, in the end, to retire the ray; they are told
        // apart because only one of them is the ray arriving somewhere.
        if !arg[0].is_finite() {
            return Err(StageFail::NotFinite);
        }
        if arg[0] < R_STOP {
            return Err(StageFail::BelowRing);
        }
        k[stage + 1] = ray_rhs(metric, &arg);
        next = arg;
    }

    let mut error = [0.0; 4];
    for (i, e) in error.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (j, w) in E.iter().enumerate() {
            sum += w * k[j][i];
        }
        *e = h * sum;
    }
    Ok((next, k[6], error))
}

/// Why a Dormand-Prince substep could not be taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StageFail {
    /// A stage would have been evaluated below R_STOP: the ray is arriving at the ring.
    BelowRing,
    /// A stage went non-finite: the state has stopped meaning anything, which is not a place.
    NotFinite,
}

/// The largest substep the two geometric caps allow from this state, in coordinate time and always
/// positive: `MAX_DR_PER_SUBSTEP` of radius at the local dr/dt, and `DIRECTION_STEP_FRACTION` of
/// the direction's own size at the local rate of turn. A ray that has stopped moving in r and
/// stopped turning - the frozen family, standing on r- - is capped by neither, which is what keeps
/// the frozen part of a field cheap however long it stands there.
fn substep_cap(y: &RayState, k: &RayState) -> f64 {
    let radial = if y[2] != 0.0 {
        MAX_DR_PER_SUBSTEP / y[2].abs()
    } else {
        f64::INFINITY
    };
    let scale = 1.0f64.max(y[2].abs()).max(y[3].abs());
    let rate = k[2].abs().max(k[3].abs());
    let turning = if rate > 0.0 {
        DIRECTION_STEP_FRACTION * scale / rate
    } else {
        f64::INFINITY
    };
    radial.min(turning)
}

/// The size of the next substep as a factor of the one just taken: the standard fifth-order
/// controller (tolerance / error)^(1/5) with the usual safety factor, clamped so that no single
/// substep can grow or shrink by more than a fixed ratio. An error estimate of zero - a stretch of
/// ray the scheme integrates exactly, such as the ingoing principal null direction, whose dr/dt is
/// -1 at every radius - asks for the largest growth allowed and is then held by the caps.
fn substep_factor(error: f64) -> f64 {
    if error <= 0.0 {
        return 5.0;
    }
    (0.9 * (SUBSTEP_TOLERANCE / error).powf(0.2)).clamp(0.2, 5.0)
}

/// The blueshift of the light Alice sends at the moment she crosses r-, as the observer who
/// crosses the same surface Delta t of coordinate time later measures it: exp(kappa_- Delta t),
/// with kappa_- = `KerrSchild::inner_surface_gravity`.
///
/// Both observers are released the same way, Delta t apart, so their worldlines are one curve
/// shifted in t and they cross r- at t_A and t_A + Delta t. A ray that freezes onto r- has its
/// radial offset decay as r - r- ~ exp(-kappa_- t), and the factor `f_factor` returns for it grows
/// as the reciprocal of that offset, so between two crossings separated by Delta t of Killing time
/// the measured frequency is multiplied by exp(kappa_- Delta t). That is the Marolf and Ori (2012)
/// statement in its simplest form, and it is finite for every finite Delta t, which is why the
/// crossing drawn here is survivable at all; it diverges only in the limit of a hole that has
/// existed forever.
///
/// Read it as a scale, not as a ceiling on every reception. A pulse Alice sends *before* her own
/// crossing has to spend the interval between emission and her crossing freezing onto r- as well,
/// so the light of that pulse reaches the later observer multiplied by exp(kappa_- (t_receive -
/// t_emit)) instead, which exceeds exp(kappa_- Delta t) by exactly the lead time. The integration
/// test measures both statements.
pub fn limiting_blueshift(metric: &KerrSchild, delta_t: f64) -> f64 {
    (metric.inner_surface_gravity() * delta_t).exp()
}

/// One crossing of the receiver's worldline by one sheet of one of the emitter's wavefronts.
///
/// A pulse can reach the receiver more than once. Where the receiver trails the emitter down the
/// same infall, as Bob trails Alice, the pulse's crossing family sweeps past him first, while he is
/// still well above r-, and arrives redshifted; the same pulse's frozen family is still standing on
/// r- when he gets there, and he cuts through that stack in the last fraction of an M of his fall,
/// with the blueshift that the Cauchy horizon is famous for. Both are recorded. The other way round
/// there is no such second act: the light that reaches an emitter's *leader* is the ingoing part of
/// the cone, which crosses r- like she does, and the frozen part never catches her up at all.
#[allow(dead_code)] // the record is complete on purpose: the HUD reads the ratio, the (t, r)
// diagram the event, and the proper time is what makes a reception quotable on the receiver's clock
#[derive(Debug, Clone, Copy)]
pub struct Reception {
    /// Serial number of the pulse this arrival belongs to.
    pub pulse_index: usize,
    /// Coordinate time of the crossing event.
    ///
    /// Every number in this record is stamped with the *crossing*, not with the detection pass
    /// that noticed it. A sheet is found to have swept over the receiver when the sign of
    /// receiver.r - r_front differs between two consecutive passes, which puts the crossing
    /// somewhere inside that interval; the whole record is the linear interpolation of the two
    /// passes in the side value, so it is the crossing event to first order in the pass interval
    /// rather than the event of whichever pass happened to look. See `Pulse::scan`.
    ///
    /// That is what makes an arrival reproducible across a rewind. Stamped with the pass, an
    /// arrival that happened just before the time being rewound to but was noticed just after it
    /// was retracted by the rewind and then never seen again, because the re-established side was
    /// already the far one.
    pub t: f64,
    /// The receiver's proper time at the crossing.
    pub tau_receiver: f64,
    /// The receiver's radius at the crossing.
    pub r: f64,
    /// The receiver's azimuth at the crossing, taken from the same interpolation as `t` and `r`.
    /// The (t, r) diagram has no use for it; the equatorial view needs it to put the arrival where
    /// it happened, which for the frozen family is a point on the r- circle.
    pub phi: f64,
    /// nu(receiver) / nu(emitter at emission) for the ray that reached them, interpolated between
    /// the two passes exactly as the event is.
    pub ratio: f64,
    /// Whether the receiving ray belongs to the frozen family, E - Omega_- L < 0, which never
    /// crosses r- and piles onto it, rather than to the crossing family that passes straight
    /// through. Decided by `NullRay::inner_horizon_energy` on whichever of the two bracketing rays
    /// is nearer to the receiver's azimuth.
    pub frozen_family: bool,
    /// Which segment of the front's polyline carried the crossing. A record of the event, not a
    /// key: the segment straddling a fixed azimuth changes as the front deforms, which is what
    /// `loop_s` is for.
    segment: usize,
    /// Where along the ray loop the crossing happened, on the same continuous coordinate
    /// `SheetSide::loop_s` carries. `Pulse::retract_unseen` matches a standing arrival to a sheet
    /// by this, so that a rewind whose priming pass finds the sheet one segment along from where
    /// the crossing was recorded still recognises it as the same sheet.
    loop_s: f64,
    /// receiver.r - r_front for that sheet on the pass that found the crossing: which side of the
    /// sheet the crossing left the receiver on.
    side_after: f64,
    /// Coordinate time of the *pass* that found the crossing, as against `t`, the interpolated
    /// crossing itself. The two differ by up to one pass interval.
    ///
    /// None of these three is drawn or reported; they exist for one job, done in
    /// `SignalField::prime`. A rewind decides what to keep by the interpolated `t`, which is only
    /// first-order accurate, so a target landing in the O(dt^2) window between `t` and the true
    /// crossing keeps an arrival whose crossing has not happened yet at the rewound state. The
    /// three together say so exactly: the pass that noticed it is inside the interval being undone
    /// (`t_pass` after the target) and the primed side is still the near one (opposite to
    /// `side_after`) on the sheet it belongs to (`segment`).
    t_pass: f64,
}

/// The emission event of a pulse that was received, kept as a record in its own right.
///
/// It is a copy of the numbers that locate a `Pulse`'s emission event on the emitter's worldline,
/// plus the time of the arrival that made the pulse a delivery, and it exists because the emission
/// event has to outlive the pulse. A transmitter running for a whole infall sends more pulses than
/// the `MAX_PULSES` cap keeps - a hovering emitter alone sends about sixty before release at the
/// app's default delay - and the pulse that carried the last signal to arrive is one of the
/// oldest, so it is the first the cap throws away. The event it marks is a fact about the
/// spacetime and does not stop being true when the drawing of its wavefront is dropped.
#[derive(Debug, Clone, Copy)]
pub struct Delivery {
    /// Serial number of the pulse that was received.
    pub pulse_index: usize,
    /// Coordinate time of the emission event.
    pub emitted_t: f64,
    /// The emitter's proper time at emission.
    pub emitted_tau: f64,
    /// Radius of the emission event. There is no azimuth here: the HUD line this record is for
    /// names the pulse by when it was sent and how deep the emitter was, and nothing draws the
    /// event any more.
    pub emitted_r: f64,
    /// Coordinate time of the earliest arrival of this pulse: the event at which the delivery
    /// became a fact, and so the time a rewind has to reach past before it can retract it.
    pub received_t: f64,
}

/// One emission event of the emitter's, and the wavefront it launched.
#[allow(dead_code)] // the emission event is recorded in full: the drawing needs the rays and the
// extent track, and the tests need the event itself to check the kappa_- law pulse by pulse
#[derive(Debug, Clone)]
pub struct Pulse {
    /// Serial number of the pulse, counting from the first one this emitter sent.
    pub index: usize,
    /// Coordinate time of the emission event.
    pub emitted_t: f64,
    /// The emitter's proper time at emission.
    pub emitted_tau: f64,
    /// Radius of the emission event.
    pub emitted_r: f64,
    /// Azimuth of the emission event.
    pub emitted_phi: f64,
    /// The whole of the emitter's light cone at emission: n exact null geodesics ordered by their
    /// emission angle alpha, from alpha = 0, their own outward radial leg, round to
    /// alpha = 2 pi - 2 pi / n. The emitter broadcasts in every direction, so the polyline these
    /// rays form is closed: the last ray joins back to the first.
    ///
    /// n is `SignalField::rays_per_pulse` as it stood at this emission - `RAYS_PER_PULSE` unless
    /// the panel's "Wavefront points" slider has been moved - and it belongs to this pulse rather
    /// than to the field: everything that reads a wavefront reads `rays.len()`, so a field can
    /// carry pulses of several counts at once and each keeps its own for life.
    pub rays: Vec<NullRay>,
    /// The pulse's *radial extent*, as (t, r_min, r_max) over the rays that were still alive at
    /// that time: what the pulse is on the (t, r) diagram, where the fan of azimuths cannot be
    /// drawn. It is seeded with (t_emit, r_emit, r_emit), the emission event, extended at every
    /// `SignalField::advance` no closer together than `TRACK_MIN_DT` by `Pulse::extend_track`,
    /// truncated by `SignalField::step_back`, and it stops growing once every ray of the pulse is
    /// dead.
    ///
    /// Drawn as a wedge, its lower edge has an exact bound, and that bound is not the 45-degree
    /// ingoing principal ray. The steepest ingoing null direction of this chart is the inner edge
    /// of the light cone, `KerrSchild::null_wedge(r).dr_dt_in`, which is the ingoing
    /// *zero-angular-momentum* ray and not a principal one: the null condition's discriminant at
    /// dr/dt = -1 is exactly a^2, so -1 lies strictly inside the cone wherever the hole spins. The
    /// edge is -1.010 at r = 4.5M for a = 0.65 and -1.020 there for a = 0.90, and it steepens
    /// inward, reaching -1.28 at r = M and -2.27 at r = 0.2M for a = 0.90. The bound is therefore
    /// the solution of
    ///
    ///     dR/dt = null_wedge(R).dr_dt_in,     R(t_emit) = r_emit,
    ///
    /// which is the straight line r_emit - (t - t_emit) only for a = 0. Every r_min of the track
    /// sits at or above R(t): no ray of the pulse can be more ingoing than the cone at the radius
    /// it is passing through, and the comparison theorem carries that pointwise statement into a
    /// statement about the curves.
    ///
    /// Two things hold r_min off the envelope: the discretisation of the cone, which leaves the
    /// most ingoing ray up to half the 360/n spacing - 1.25 degrees at the default count - off the
    /// extremal direction, and, once the pulse has finished being swallowed, the loss of every ray that
    /// reached the ring, after which the minimum is taken over whatever is still alive and lifts
    /// away from the bound for good. Before that the gap is at most 8e-4 M, measured in
    /// `test_extent_track_lower_edge_hugs_the_steepest_ingoing_ray`, so the drawn lower edge is
    /// the ingoing edge of the emitter's own light cone to well within a pixel - and it runs up to
    /// 0.07 M *below* the 45-degree line over an infall at a = 0.65, which is why the 45-degree
    /// line is not what is claimed here.
    ///
    /// The upper edge has no closed form: it is the outermost live ray, which is the emitter's
    /// outward radial leg only while the pulse is outside r+. Inside r+ every ray falls, and the
    /// outermost of them tends to the outgoing principal null direction of r-, so the upper edge
    /// freezes onto the Cauchy horizon while the lower edge runs on to the ring.
    ///
    /// While the pulse is being swallowed, neither edge is a live ray at all: see
    /// `Pulse::radial_extent`, which pins them to the boundary the front is standing on.
    pub extent_track: Vec<(f64, f64, f64)>,
    /// Coordinate-time spacing the track is currently stored at: `TRACK_MIN_DT`, doubled once for
    /// each thinning `TRACK_MAX_POINTS` has forced. See `Pulse::extend_track`.
    track_dt: f64,
    /// One entry per sheet of the front that stood across the receiver's azimuth on the previous
    /// detection pass, carrying the side they were on and the event they were at. A sheet is
    /// identified from pass to pass by where it sits along the ray loop rather than by which
    /// segment happens to carry it, so folds and windings are tracked independently instead of
    /// collapsing into one number, and a sheet that stops straddling them drops out of the list.
    sheets: Vec<SheetSide>,
    /// Every crossing of the receiver's worldline by this pulse, in the order they met them.
    pub receptions: Vec<Reception>,
}

/// Where one sheet of a wavefront stood relative to the receiver at the previous detection pass,
/// and where and when the receiver was.
///
/// The receiver's event is kept alongside the side because a reception is stamped with the crossing
/// rather than with the pass that found it: the two bracketing passes are interpolated in the side
/// value, so the earlier of the two has to be on hand in full. See `Pulse::scan`.
#[derive(Debug, Clone, Copy)]
struct SheetSide {
    /// Index of the polyline segment (the ray pair) carrying this sheet at that pass. Kept for the
    /// record and for the tests; it is `loop_s` and not this that says which sheet this is.
    segment: usize,
    /// Where the sheet sits along the closed ray loop: s = i + w, the index of the segment carrying
    /// it plus the fraction of the way along that segment at which it crosses the receiver's
    /// azimuth. It runs over [0, n) and is cyclic, n being the number of rays, and it moves
    /// continuously as the front deforms, which the segment index does not: see `Pulse::scan`.
    loop_s: f64,
    /// receiver.r - r_front for this sheet, as both stood at that pass. Storing the side rather
    /// than r_front alone is what makes the receiver's own motion count: inside r+ the front has
    /// all but stopped and it is the receiver who does the crossing.
    side: f64,
    /// The receiver's coordinate time at that pass.
    t: f64,
    /// The receiver's radius at that pass.
    r: f64,
    /// The receiver's azimuth at that pass.
    phi: f64,
    /// The receiver's proper time at that pass.
    tau: f64,
    /// The shift this sheet carried at that pass, by the same interpolation along the segment that
    /// a crossing uses. It is evaluated on every pass, not only on a crossing, so that the shift of
    /// a crossing can be interpolated between the two passes exactly as the event is.
    ratio: f64,
}

impl Pulse {
    /// The radial interval the pulse's front spans right now, or None once no ray of it is alive.
    ///
    /// The front is a closed curve in (r, phi) and its projection onto the r axis is exactly this
    /// interval. Taking it as the minimum and maximum over the live rays is right while the front
    /// is in the open, and wrong at either boundary of the field, because a boundary is where the
    /// *sampling* of the front stops being the front.
    ///
    /// Inside r- most rays of a pulse are swallowed, one after another, in an order fixed by the
    /// conserved L/E of each (see the module header: from r = 0.45 at a = 0.90, sixty of the
    /// seventy-two reach the ring). Take the minimum over the survivors and the inner edge sits on
    /// the innermost sampled ray, which jumps outward to the next one every time one dies and then
    /// dives again - a sawtooth, and an artefact of sampling the cone at all (2.5 degrees at the
    /// default count). The
    /// continuum front has no such thing in it. The time at which a ray arrives at the ring depends
    /// continuously on its emission angle, and its descent is bounded away from a standstill there
    /// (the potential tends to 2 M (L - aE)^2 r, positive for every ray but the one with L = aE),
    /// so between an angle whose ray has already arrived and an angle whose ray is still on its way
    /// there is an angle whose ray arrives exactly now: for the whole interval over which the pulse
    /// is being swallowed, some part of the true front stands on the ring. That is the exact
    /// statement, and it is what is drawn:
    ///
    /// * r_min is R_STOP whenever two *neighbouring* rays of the cone have one dead at the ring and
    ///   the other still `NullRay::ring_bound` - the two ends of that intermediate-value argument.
    /// * r_max is `R_ESCAPE` under the same test at the other boundary, with
    ///   `NullRay::escape_bound` in place of `ring_bound`.
    ///
    /// Otherwise the plain extremum over the live rays stands. Neighbouring is the point. The two
    /// ends of the argument have to bracket a stretch of the continuum that is all going the same
    /// way, and adjacent emission angles are 360/n degrees apart - 2.5 at the default count -
    /// whereas taking them from anywhere
    /// in the cone would assume the whole arc between them shares one fate - true of every pulse
    /// measured here, since the fate is fixed by L/E and L/E runs monotonically round each half of
    /// the cone, but not a theorem. Where the ring-bound rays do form one arc the two readings
    /// agree exactly: an arc of consecutive indices holding both a dead ray and a live ring-bound
    /// one holds a neighbouring pair of them.
    ///
    /// Both halves of the test are exact: which boundary a dead ray died at is recorded by the
    /// integrator (`RayEnd`), and whether a live ray will reach one is settled in closed form from
    /// its own conserved (E, L) and from the horizons it would have to cross to get there, with
    /// nothing integrated and nothing assumed. The horizons are half of it and not a refinement:
    /// see `NullRay::ring_bound`, whose doc sets out what goes wrong at late times without them.
    ///
    /// There is a resolution limit, at the far end of the swallowing and only there. The last rays
    /// to be lost are the ones whose L/E sits just below the boundary between the two fates, and
    /// just *above* that boundary are rays that turn arbitrarily close to the ring and linger there
    /// before climbing away. The continuum front therefore leaves the ring smoothly, and seventy-
    /// two samples of the cone cannot resolve the last of it: once the final ring-bound sample
    /// dies, the drawn inner edge steps outward to the innermost survivor, sooner and more abruptly
    /// than the continuum would. Nothing here hides that step; it is the point at which the drawing
    /// runs out of rays, and `test_the_swallowed_front_sits_on_the_ring` measures where it falls.
    fn radial_extent(&self, metric: &KerrSchild) -> Option<(f64, f64)> {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let (mut swallowed, mut escaped) = (false, false);
        for ray in self.rays.iter() {
            if ray.alive() {
                lo = lo.min(ray.r);
                hi = hi.max(ray.r);
            } else {
                swallowed |= ray.died_at_ring();
                escaped |= ray.died_escaping();
            }
        }
        if lo > hi {
            return None;
        }
        // Is the front standing on a boundary: is there a neighbouring pair of rays with one of
        // them already there and the other still on its way? The potential test costs a metric, so
        // it is only ever asked of the neighbour of a ray that has died at the boundary in
        // question, and not at all until one has - which for most of a pulse's life is never.
        let n = self.rays.len();
        let straddles = |arrived: fn(&NullRay) -> bool, coming: fn(&NullRay, &KerrSchild) -> bool| {
            (0..n).any(|i| {
                let (a, b) = (&self.rays[i], &self.rays[(i + 1) % n]);
                (arrived(a) && coming(b, metric)) || (arrived(b) && coming(a, metric))
            })
        };
        if swallowed && straddles(NullRay::died_at_ring, NullRay::ring_bound) {
            lo = R_STOP;
        }
        if escaped && straddles(NullRay::died_escaping, NullRay::escape_bound) {
            hi = R_ESCAPE;
        }
        Some((lo, hi))
    }

    /// Record the pulse's radial extent at the field time `t`, unless the last stored point is less
    /// than `track_dt` behind it or the pulse has no ray left alive.
    ///
    /// The spacing is measured against the last stored point rather than accumulated per call, so
    /// a run of short frames stores one point per `track_dt` of coordinate time exactly as one long
    /// frame does, and the drawn wedge does not depend on the frame rate. Nothing is interpolated:
    /// a point is stored at whatever time the first call past the spacing lands on, because the
    /// extent is read off the rays and the rays only ever stand at times they have been stepped to.
    /// A caller stepping in intervals longer than `track_dt` - Distance mode at a supermassive
    /// hole - gets one point per call and no more, which is all the resolution the rays themselves
    /// were carried at.
    ///
    /// A long run degrades by thinning rather than by stopping. At `TRACK_MAX_POINTS` every other
    /// point is dropped and the spacing doubles, which halves the resolution of the whole track and
    /// buys the same span again. Both ends are kept: the seed, the emission event, is index zero,
    /// and the newest point is kept too, so the spacing coming out of a thinning is the new one
    /// everywhere rather than one and a half of it across the join. The alternative, which this
    /// replaces, was to stop recording, and the wedge then ended in mid-air at a time that
    /// depended on how long the app had been running.
    fn extend_track(&mut self, metric: &KerrSchild, t: f64) {
        let Some(&(last_t, ..)) = self.extent_track.last() else {
            return;
        };
        // The comparison carries a relative slack of a part in 1e9 because a caller whose own
        // step *is* the spacing - a played frame of 1/50 M against `TRACK_MIN_DT` - must not lose
        // every other point to the accumulated clock landing a fraction of an ulp short of it.
        // Nothing physical turns on it: it is a floating-point guard on a display cadence.
        if t < last_t + self.track_dt * (1.0 - 1e-9) {
            return;
        }
        let Some((r_min, r_max)) = self.radial_extent(metric) else {
            return;
        };
        if self.extent_track.len() >= TRACK_MAX_POINTS {
            let newest = self.extent_track.len() - 1;
            let mut index = 0;
            self.extent_track.retain(|_| {
                let keep = index % 2 == 0 || index == newest;
                index += 1;
                keep
            });
            self.track_dt *= 2.0;
        }
        self.extent_track.push((t, r_min, r_max));
    }

    /// Record every crossing of the receiver's worldline by this wavefront on this pass.
    ///
    /// The rays of a pulse are a closed polyline in the (r, phi) plane, ordered by their emission
    /// angle: the emitter broadcasts into their whole light cone, so the last ray joins back to the
    /// first and that closing segment is a sheet like any other. The receiver is located on the
    /// polyline through the *unwrapped* azimuth of each ray relative to theirs.
    ///
    /// Only the first ray is placed against the receiver, and only because it has to be: the
    /// receiver's own azimuth is an angle, kept in [0, 2 pi) by `Observer`, so folding
    /// rays[0].phi - receiver.phi into [-pi, pi] is a choice of which turn to call zero and nothing
    /// more. Every step after it is the *raw* difference of two integrated azimuths, and that is
    /// exact rather than a convention. Every ray of the pulse left the emission event at the
    /// emitter's own azimuth, and `NullRay::phi` is integrated continuously and never reduced mod
    /// 2 pi, so the difference between two neighbouring rays is a continuous function of time
    /// starting at zero: the integrated phi already is the unwrapped coordinate, and the raw
    /// difference already is the physical winding between the pair. Folding it into [-pi, pi]
    /// agrees with that only while the pair are less than half a turn apart, and a front that has
    /// wound - a ray hung on a circular photon orbit for tens of M while the neighbour it was
    /// emitted next to fell in, say - is exactly the case that violates it: past half a turn the
    /// fold flips the sign of the step and lays that piece of the front down on the wrong side of
    /// the hole. The raw steps also telescope, so the loop closes back on rel[0] exactly, which is
    /// the statement that a front is a closed curve.
    ///
    /// On that unwrapped axis the receiver is not one angle but the whole family 0, +/-2 pi,
    /// +/-4 pi, ..., because a front that has wound one turn further passes over them again. Each
    /// crossing of one of those angles by a polyline segment is one *sheet* of the front standing
    /// across their azimuth. A segment whose two rays have wound several turns apart crosses
    /// several of them and carries a sheet at each: those are real, separate pieces of front
    /// standing across the receiver's azimuth at their own radii, and each is tracked on its own.
    ///
    /// One consequence is worth stating on its own, because the fold got it wrong the other way. A
    /// loop all of whose rays are alive crosses the receiver's azimuth an *even* number of times:
    /// the raw steps telescope to zero round the loop, so such a front has no net winding, and
    /// every sheet is paired with the one the loop makes coming back - the near side and the far
    /// side of the same front, which is what a receiver at a fixed azimuth is actually swept by. A
    /// front that genuinely encircles the hole is not a counter-example: to enclose it some ray of
    /// the loop has to have gone into it, and that ray is dead, so both of its segments are skipped
    /// and what is left is an open arc of live rays that may span any number of turns. The fold
    /// used to invent a net winding for any loop whose rays happened to span a full turn, and drop
    /// the returning sheet.
    ///
    /// A sheet is identified from one pass to the next by *where it is*, not by which segment
    /// carries it. Its position is the loop coordinate
    ///
    ///     s = i + w,     i the segment index, w in [0, 1] the fraction along that segment,
    ///
    /// which runs over [0, n) cyclically and moves continuously as the front deforms, while the
    /// integer part of it - the old key - steps. A sheet on this pass is matched to the sheet of
    /// the previous pass at the smallest cyclic distance in s, provided that distance is under a
    /// window of n/4, and each previous sheet is claimed at most once, nearest pair first.
    ///
    /// Keying by the segment index instead lost arrivals, and not only in the deep interior. An
    /// off-centre circle expanding in the weak field sweeps its segment indices past a fixed
    /// azimuth as it grows - no winding is needed for that at all, just the front moving - so the
    /// straddling segment hands over to a neighbour every so often, and the pass on which it did so
    /// had no remembered side for the new key. A crossing inside that step was simply not seen:
    /// regular gaps in the reception ticks along both worldlines, and worse inside r+, where frame
    /// dragging shifts the segments faster.
    ///
    /// The window is generous on purpose. A quarter of the loop is far more than the handover of a
    /// segment or two that this is for, and it is still far less than the separation of genuinely
    /// distinct sheets, which are the two sides of a front (half a loop apart) or two folds of one
    /// inside r+ (a fold being a whole arc of rays). Where two sheets do approach each other they
    /// are approaching a tangency, at which they merge and vanish together: the front grazes the
    /// receiver's azimuth without sweeping over it, both sheets stop straddling, and no crossing is
    /// invented for either. Nearest-first matching keeps the pairing of two nearby sheets stable
    /// until then.
    ///
    /// A sheet gives r_front by linear interpolation along the segment, and the shift by the same
    /// linear interpolation of the two bracketing rays' own frequency ratios, each evaluated at its
    /// own event against the receiver's 4-velocity. Interpolating the finished ratios rather than
    /// the raw factors keeps the answer between two numbers that are both exact measurements, which
    /// matters once the ratios span orders of magnitude. The family the arrival belongs to is read
    /// off the nearer of the two bracketing rays with `NullRay::inner_horizon_energy`.
    ///
    /// A reception is a sign change of receiver.r - r_front for one sheet between two consecutive
    /// passes on which that same sheet stood across them. Tracking sheets separately rather than
    /// reducing the front to a single representative radius is what makes both arrivals of a pulse
    /// show up where both happen: the crossing family sweeping past a trailing receiver high above
    /// r-, and the frozen family waiting on r- for them to fall through it. A sheet whose rays have
    /// reached the ring is retired with them, since only segments with both ends still alive can be
    /// interpolated; that sheet simply stops being tracked, and no crossing is invented for it.
    ///
    /// The arrival is recorded at the *crossing*, not at the pass that found it. The side value is
    /// interpolated linearly between the two bracketing passes,
    ///
    ///     t_cross = t_prev + (t_now - t_prev) side_prev / (side_prev - side_now),
    ///
    /// and the receiver's (r, phi, tau) and the sheet's shift are interpolated at the same
    /// fraction, so the whole record is the crossing event to first order in the pass interval.
    /// Stamping the pass instead made an arrival depend on where the passes happened to fall, which
    /// a rewind changes: a crossing that had happened before the time being rewound to but was
    /// noticed after it was dropped by the rewind and then never re-detected, because the side
    /// re-established on the way forward was already the far one.
    ///
    /// `record` says whether the pass may record anything. A pass with it false establishes the
    /// sides and nothing else, which is what a field has to do after a rewind before it can call
    /// any later sign change a crossing: see `SignalField::prime`.
    fn scan(
        &mut self,
        metric: &KerrSchild,
        receiver: &Observer,
        u_receiver: &[f64; 3],
        record: bool,
    ) {
        if self.rays.len() < 2 {
            self.sheets.clear();
            return;
        }
        let n = self.rays.len();
        let two_pi = 2.0 * std::f64::consts::PI;
        // The one fold: which turn of the receiver's azimuth to call zero. Everything after it is
        // the raw difference of two continuously integrated azimuths, which is already unwrapped.
        let wrap = |d: f64| d - two_pi * (d / two_pi).round();
        let mut rel = Vec::with_capacity(n);
        rel.push(wrap(self.rays[0].phi - receiver.phi));
        for i in 1..n {
            rel.push(rel[i - 1] + (self.rays[i].phi - self.rays[i - 1].phi));
        }

        // The closing segment runs from the last ray back to the first, over the raw difference
        // like every other step. Those steps telescope, so this lands back on rel[0] to within
        // rounding: the loop of rays is closed, and stays closed however far it has wound.
        let closing = rel[n - 1] + (self.rays[0].phi - self.rays[n - 1].phi);
        let mut sheets: Vec<SheetSide> = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            if !self.rays[i].alive() || !self.rays[j].alive() {
                continue;
            }
            let (a, b) = (rel[i], if i + 1 < n { rel[i + 1] } else { closing });
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            let first = (lo / two_pi).ceil() as i64;
            let last = (hi / two_pi).floor() as i64;
            for turn in first..=last {
                let target = (turn as f64) * two_pi;
                let span = b - a;
                let w = if span.abs() < 1e-15 {
                    0.0
                } else {
                    ((target - a) / span).clamp(0.0, 1.0)
                };
                let r_front = self.rays[i].r + w * (self.rays[j].r - self.rays[i].r);
                // The shift this sheet carries right now, by the same interpolation along the
                // segment: the two bracketing rays' own frequency ratios, each evaluated at its own
                // event. It is wanted on every pass, not only on a crossing, because the crossing's
                // shift is interpolated between two passes like everything else about the event.
                let f0 = self.rays[i].frequency_ratio(metric, u_receiver);
                let f1 = self.rays[j].frequency_ratio(metric, u_receiver);
                sheets.push(SheetSide {
                    segment: i,
                    loop_s: (i as f64) + w,
                    side: receiver.r - r_front,
                    t: receiver.t,
                    r: receiver.r,
                    phi: receiver.phi,
                    tau: receiver.tau,
                    ratio: f0 + w * (f1 - f0),
                });
            }
        }

        // Which sheet of the previous pass each of these is, by continuity of the loop coordinate:
        // nearest pair first, each previous sheet claimed at most once, nothing paired across more
        // than a quarter of the loop. Sorting the pairs by distance is what makes the assignment
        // independent of the order the sheets happen to be found in.
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for (now, sheet) in sheets.iter().enumerate() {
            for (before, prev) in self.sheets.iter().enumerate() {
                let d = self.loop_distance(sheet.loop_s, prev.loop_s);
                if d < self.sheet_window() {
                    pairs.push((d, now, before));
                }
            }
        }
        pairs.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
        let mut was: Vec<Option<SheetSide>> = vec![None; sheets.len()];
        let mut claimed = vec![false; self.sheets.len()];
        for (_, now, before) in pairs {
            if was[now].is_none() && !claimed[before] {
                was[now] = Some(self.sheets[before]);
                claimed[before] = true;
            }
        }

        // A sign change on a sheet that was tracked at the previous pass is a crossing, and nothing
        // here second-guesses it. A sheet drops out of straddling the receiver and comes back as
        // the front winds, and while it is away the receiver can pass it by another part of the
        // loop, so no rule that compares this crossing with the last one recorded on the same sheet
        // is safe: it would suppress a real arrival every time a sheet handed the receiver over to
        // a neighbour and took them back. The one case where the same crossing can be offered twice
        // is a rewind, and it is settled there, at the rewound state, by `SignalField::prime`.
        if record {
            for (sheet, prev) in sheets.iter().zip(was.iter()) {
                let Some(prev) = prev else { continue };
                if prev.side * sheet.side >= 0.0 {
                    continue;
                }
                // Where in the interval between the two passes the side changed sign.
                let fraction = prev.side / (prev.side - sheet.side);
                let at = |before: f64, now: f64| before + fraction * (now - before);
                let crossing_ratio = at(prev.ratio, sheet.ratio);
                if !crossing_ratio.is_finite() || crossing_ratio <= 0.0 {
                    continue;
                }
                let i = sheet.segment;
                let nearer = if sheet.loop_s - (i as f64) < 0.5 { i } else { (i + 1) % n };
                self.receptions.push(Reception {
                    pulse_index: self.index,
                    t: at(prev.t, receiver.t),
                    tau_receiver: at(prev.tau, receiver.tau),
                    r: at(prev.r, receiver.r),
                    phi: at(prev.phi, receiver.phi),
                    ratio: crossing_ratio,
                    frozen_family: self.rays[nearer].frozen(metric),
                    segment: i,
                    loop_s: sheet.loop_s,
                    side_after: sheet.side,
                    t_pass: receiver.t,
                });
            }
        }
        self.sheets = sheets;
    }

    /// The distance between two loop coordinates of this pulse, the loop being closed: never more
    /// than half the number of rays.
    fn loop_distance(&self, a: f64, b: f64) -> f64 {
        let span = self.rays.len() as f64;
        let d = (a - b).rem_euclid(span);
        d.min(span - d)
    }

    /// How far apart along the loop two sightings of a sheet may be and still be the same sheet: a
    /// quarter of the loop, which is stated and argued for in `Pulse::scan`.
    fn sheet_window(&self) -> f64 {
        self.rays.len() as f64 / 4.0
    }

    /// Drop the arrivals that the state just primed says have not happened yet, and return them.
    ///
    /// Called only from `SignalField::prime`, immediately after a priming pass has filled `sheets`
    /// with the sides at the rewound state, and only for a rewind to `target_t`.
    ///
    /// A rewind keeps the arrivals whose interpolated crossing time is at or before its target.
    /// That stamp is first-order accurate in the pass interval, so for a target inside the O(dt^2)
    /// window between the stamp and the crossing it actually estimates, the record says an arrival
    /// has happened while the geometry at the rewound state says the receiver has not reached the
    /// sheet yet. Left alone, the next step forward finds that same change of side and records the
    /// arrival a second time.
    ///
    /// The disagreement is exactly detectable, and only in that window. For each sheet standing
    /// across the receiver now, the last arrival recorded on it is retracted when both of these
    /// hold:
    ///
    /// * the pass that noticed it is inside the interval being undone (`t_pass` after `target_t`),
    ///   so the rewind has taken the field back to before the observation itself; and
    /// * the primed side is opposite to the side that arrival left the receiver on, so the crossing
    ///   has not happened at the rewound state.
    ///
    /// An arrival noticed at or before the target is consistent with the primed state by
    /// construction and is left alone, whatever side it left the receiver on. A retracted arrival
    /// is not lost: the next step forward brackets the same crossing and records it again, from the
    /// primed pass instead of the one the rewind undid, which moves it by the O(dt^2) the stamp was
    /// uncertain by in the first place.
    fn retract_unseen(&mut self, target_t: f64) -> Vec<Reception> {
        let mut doomed: Vec<usize> = Vec::new();
        for sheet in self.sheets.iter() {
            // The sheet is matched to the arrival by the same continuity rule `Pulse::scan` tracks
            // it with, and for the same reason: the priming pass stands up to one step away from
            // the pass that recorded the arrival, which is long enough for the straddling segment
            // to have handed over. Matching the segment index exactly would let exactly the
            // handovers this is meant to survive defeat the retraction as well.
            let last = self
                .receptions
                .iter()
                .enumerate()
                .rfind(|(_, rec)| {
                    self.loop_distance(rec.loop_s, sheet.loop_s) < self.sheet_window()
                });
            if let Some((index, rec)) = last
                && rec.t_pass > target_t
                && rec.side_after * sheet.side < 0.0
            {
                doomed.push(index);
            }
        }
        if doomed.is_empty() {
            return Vec::new();
        }
        let mut retracted = Vec::new();
        let mut index = 0;
        self.receptions.retain(|rec| {
            let keep = !doomed.contains(&index);
            if !keep {
                retracted.push(*rec);
            }
            index += 1;
            keep
        });
        retracted
    }
}

/// Is this observer waiting for release on a worldline that exists?
///
/// Before release `Observer::step` holds the observer at fixed (r, phi) and advances their clock as
/// dtau = sqrt(-g_tt) dt, which is the static observer's proper time: the worldline they are on
/// while they wait is an integral curve of the time-translation Killing vector. That curve is
/// timelike only where g_tt < 0, i.e. outside the equatorial static limit r = 2M; inside it no
/// rocket can hold phi fixed and there is no such observer to be.
fn is_static_hover(metric: &KerrSchild, observer: &Observer) -> bool {
    !observer.is_active && metric.metric_components(observer.r)[0][0] < 0.0
}

/// The 4-velocity to quote an observer's signals in, at emission and at reception alike: the
/// 4-velocity of the worldline the observer is on, and nothing else.
///
/// It is `Observer::four_velocity`, and this function exists only to say why the signal code may
/// use it unqualified. A pulse has to be emitted isotropically in the frame of the worldline the
/// emitter is actually travelling on, and an arrival has to be measured in the frame of the one the
/// receiver is actually travelling on; anything less than that puts a real tetrad at an event no
/// worldline passes through. `Observer::four_velocity` now guarantees it in every case - the
/// released faller reports the geodesic being integrated, the hoverer of `is_static_hover` reports
/// the static frame its clock is keeping, and an impossible Static or ZAMO selection reports the
/// free fall it is being stepped along - so the local correction this used to apply for hovering
/// emitters is gone, and with it the last place where the drawn worldline and the quoted frame
/// could disagree.
fn signalling_four_velocity(metric: &KerrSchild, observer: &Observer) -> [f64; 3] {
    observer.four_velocity(metric)
}

/// Every pulse one emitter currently has in flight.
///
/// The field is agnostic about who is at each end of it: `emit_if_due` takes the emitter and
/// `detect_receptions` the receiver, so the app carries one of these for Alice's transmission and
/// a second for Bob's, both advanced on the same clock and both rewound by the same `step_back`.
#[derive(Debug, Clone)]
pub struct SignalField {
    /// Live pulses, oldest first.
    pub pulses: Vec<Pulse>,
    /// The field's own coordinate clock, advanced by `advance` and wound back by `step_back`. It
    /// carries the time of the field as a whole, so that a step backwards does not have to
    /// interrogate every pulse for it, and it is what an empty field falls back on.
    pub t: f64,
    /// Serial number the next pulse will carry.
    next_index: usize,
    /// The emitter's proper time at the last emission, or None before they have sent anything.
    last_emit_tau: Option<f64>,
    /// The emitter's proper-time interval between pulses.
    pub interval_tau: f64,
    /// How many rays the *next* pulse will carry: the sampling of the emitter's light cone, set
    /// from the panel's "Wavefront points" slider and starting at `RAYS_PER_PULSE`.
    ///
    /// `emit_if_due` is the only reader. A pulse already in flight keeps the count it went out
    /// with, because its rays are the null geodesics that were launched and there is nowhere to get
    /// more of them from without emitting a different pulse; moving the slider therefore changes
    /// the pulses sent from then on and leaves the standing ones exactly as they are, and a field
    /// can hold several counts at once. Nothing downstream needs to be told which: the closed
    /// polyline the equatorial view draws, the loop coordinate of `Pulse::scan` (taken mod n), the
    /// neighbouring pairs of `Pulse::radial_extent` and the width of `Pulse::sheet_window` all read
    /// the pulse's own `rays.len()`.
    ///
    /// Whatever n is, the emission angles are alpha = 2 pi i / n, so alpha = 0 - the emitter's own
    /// outward radial leg - is ray zero of every pulse and the spacing is 360/n degrees.
    pub rays_per_pulse: usize,
    /// The newest delivery this transmission has made, remembered separately from the pulses so
    /// that the `MAX_PULSES` cap cannot erase the causal boundary it marks. Maintained by
    /// `detect_receptions` and wound back by `step_back`; read through `last_delivered_pulse`.
    last_delivered: Option<Delivery>,
    /// Every arrival this transmission has made, in the order they were recorded, for the same
    /// reason: an arrival is an event that happened, and the cap dropping the wavefront that
    /// carried it does not unhappen it. Each one is also kept on its own pulse for as long as that
    /// pulse lives, which is what `Delivery` is read off; this is the copy that outlives it.
    heard: Vec<Reception>,
    /// Rays retired by the integrator's substep budget rather than by the geometry, over the whole
    /// life of this field. See `RayStep::BudgetExhausted`: it is a safety net that should never
    /// fire, so the count is kept rather than discarded and the tests assert it is zero.
    budget_exhausted: usize,
}

impl Default for SignalField {
    fn default() -> Self {
        Self {
            pulses: Vec::new(),
            t: 0.0,
            next_index: 0,
            last_emit_tau: None,
            interval_tau: EMISSION_INTERVAL_TAU,
            rays_per_pulse: RAYS_PER_PULSE,
            last_delivered: None,
            heard: Vec::new(),
            budget_exhausted: 0,
        }
    }
}

impl SignalField {
    /// Emit a pulse if the emitter's own clock says one is due: at the first call, and every
    /// `interval_tau` of their proper time after that. A stalled or retired worldline sends
    /// nothing, since a worldline that is no longer advancing has no proper time to space pulses
    /// by.
    ///
    /// An observer still waiting for release transmits too, and must. While they wait they are the
    /// static observer of `is_static_hover`, hovering at fixed (r, phi) with a perfectly good clock
    /// ticking at dtau = sqrt(-g_tt) dt and a perfectly good orthonormal frame to broadcast into;
    /// nothing in the geometry stops them, and cutting them off at their release would draw a
    /// transmission that starts for no physical reason. So the cadence runs on their proper time
    /// from the first call onward, through the release event and on down the infall, and the
    /// emission frame is `signalling_four_velocity`: the static frame while they hover, their own
    /// once they fall. The only observer with nothing to transmit from is one waiting at or inside
    /// the static limit, where the hovering worldline does not exist; that one waits in silence.
    pub fn emit_if_due(&mut self, metric: &KerrSchild, emitter: &Observer) {
        if (!emitter.is_active && !is_static_hover(metric, emitter)) || emitter.r <= R_STOP {
            return;
        }
        if let Some(geo) = emitter.geodesic
            && geo.stalled
        {
            return;
        }
        if let Some(last) = self.last_emit_tau
            && emitter.tau < last + self.interval_tau
        {
            return;
        }

        // Light is isotropic in the frame of the worldline the emitter is actually on: their own
        // once they fall, the static frame while they hover. That is what
        // `signalling_four_velocity` returns, for every mode and at every radius.
        let u = signalling_four_velocity(metric, emitter);
        let tetrad = Tetrad::from_four_velocity(metric, emitter.r, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        // The count is read here and nowhere else, so this pulse is fixed at whatever the slider
        // said when it was sent and the pulses already in flight are untouched.
        let n = self.rays_per_pulse;
        let rays = (0..n)
            .map(|i| {
                // alpha = 0 is the emitter's outward radial leg and the ray count divides the
                // turn exactly, so the last ray stops one step short of alpha = 2 pi and the front
                // closes. That holds for every n: the angles are 2 pi i / n, spaced 360/n degrees.
                let alpha = two_pi * (i as f64) / (n as f64);
                NullRay::from_local_direction(
                    metric, emitter.t, emitter.r, emitter.phi, &tetrad, alpha, &u,
                )
            })
            .collect();

        self.pulses.push(Pulse {
            index: self.next_index,
            emitted_t: emitter.t,
            emitted_tau: emitter.tau,
            emitted_r: emitter.r,
            emitted_phi: emitter.phi,
            rays,
            extent_track: vec![(emitter.t, emitter.r, emitter.r)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        });
        self.next_index += 1;
        self.last_emit_tau = Some(emitter.tau);
        while self.pulses.len() > MAX_PULSES {
            self.pulses.remove(0);
        }
    }

    /// Advance every live ray by dt of coordinate time, then record how far each pulse's radial
    /// extent now reaches.
    ///
    /// `NullRay::step` carries a dead ray forward on the clock without moving it, so every ray of
    /// the field reads the same t as the field itself and `step_back` can ask each of them the one
    /// question that matters: was this ray still alive dt ago?
    pub fn advance(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.t += dt;
        let t = self.t;
        let mut exhausted = 0;
        for pulse in self.pulses.iter_mut() {
            for ray in pulse.rays.iter_mut() {
                if ray.step(metric, dt) == RayStep::BudgetExhausted {
                    exhausted += 1;
                }
            }
            // After the rays and never before: the extent is read off where they now stand.
            pulse.extend_track(metric, t);
        }
        self.budget_exhausted += exhausted;
    }

    /// Carry the whole field back by dt of coordinate time, the way `Observer::rewind_to` carries a
    /// worldline back, so that stepping the simulation backwards undoes the transmission instead of
    /// deleting it.
    ///
    /// The target time is dt before the latest event in the field, and everything else follows from
    /// it. A pulse emitted after the target was never sent and goes. Every ray of a pulse that
    /// survives is integrated back to the target, reviving if it died inside the interval. A
    /// extent track is truncated to the points it had reached by then, never below its seed, the
    /// emission event itself. A reception recorded after the target is unrecorded.
    ///
    /// The per-sheet bookkeeping of `Pulse::scan` is dropped rather than rewound: rewinding a side
    /// is meaningless, and keeping the stale one would invent a sign change out of the rewind
    /// itself. Dropping it is not enough on its own, though. A field with no sides at all spends
    /// its next forward pass establishing them, so a crossing that happens inside that first step
    /// is never seen; the sides have to be re-established *at the rewound state* instead, which is
    /// `SignalField::prime`. `SignalPair::step_back` calls it for both fields, which is why it
    /// takes the observers and why they are rewound before the fields are.
    ///
    /// `last_emit_tau` falls back to the newest surviving pulse, so the emitter resumes on the same
    /// cadence as time runs forward again, and `next_index` is left alone: serial numbers are not
    /// reused, and a re-emitted pulse is a new pulse even where it lands on an old emission event.
    ///
    /// One thing does not come back. A pulse already evicted by the `MAX_PULSES` cap is gone from
    /// the field, and no amount of stepping backwards restores it; the reversible window is the
    /// window the cap keeps.
    pub fn step_back(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let latest = self
            .pulses
            .iter()
            .flat_map(|p| p.rays.iter())
            .map(|ray| ray.t)
            .fold(f64::NEG_INFINITY, f64::max);
        let target_t = if latest.is_finite() { latest } else { self.t } - dt;

        self.pulses.retain(|p| p.emitted_t <= target_t + 1e-12);
        let mut exhausted = 0;
        for pulse in self.pulses.iter_mut() {
            for ray in pulse.rays.iter_mut() {
                if ray.step_back(metric, dt) == RayStep::BudgetExhausted {
                    exhausted += 1;
                }
            }
            let keep = pulse
                .extent_track
                .iter()
                .take_while(|(t, _, _)| *t <= target_t + 1e-9)
                .count()
                .max(1);
            pulse.extent_track.truncate(keep);
            pulse.receptions.retain(|rec| rec.t <= target_t);
            pulse.sheets.clear();
        }
        self.budget_exhausted += exhausted;
        self.heard.retain(|rec| rec.t <= target_t);
        self.last_emit_tau = self.pulses.iter().map(|p| p.emitted_tau).reduce(f64::max);
        // A delivery is retracted only if the arrival that made it happened after the target time;
        // one that had already happened by then still stands, even where the cap has since thrown
        // its pulse away. Whichever of the standing record and the pulses in hand names the newer
        // pulse wins, so a rewind that unsays the newest delivery falls back on the one before it
        // as long as that pulse is still in the field.
        let standing = self.last_delivered.filter(|d| d.received_t <= target_t);
        let in_hand = newest_delivery(&self.pulses);
        self.last_delivered = match (standing, in_hand) {
            (Some(a), Some(b)) if b.pulse_index > a.pulse_index => Some(b),
            (Some(a), _) => Some(a),
            (None, b) => b,
        };
        self.t = target_t;
    }

    /// Record every crossing of the receiver's worldline by every live wavefront on this pass.
    ///
    /// Outside r+ a front overtakes a receiver from below as it climbs outward; inside r+
    /// everything falls and it is a trailing receiver who overtakes a front that has all but
    /// stopped against r-. A receiver *ahead* of the emitter is caught only by the ingoing part of
    /// each front, which runs at up to dr/dt = -1 and so outruns any timelike worldline. The
    /// per-sheet sign change of `Pulse::scan` catches all of these, and catches them for each
    /// sheet of a folded front separately, which is why a single pulse can be received more than
    /// once.
    pub fn detect_receptions(&mut self, metric: &KerrSchild, receiver: &Observer) {
        let u_receiver = signalling_four_velocity(metric, receiver);
        for pulse in self.pulses.iter_mut() {
            let before = pulse.receptions.len();
            pulse.scan(metric, receiver, &u_receiver, true);
            self.heard.extend_from_slice(&pulse.receptions[before..]);
        }
        // The record only ever moves forward here. Arrivals do not come in emission order, so a
        // pulse older than the one on record can be heard at any time without changing which pulse
        // was the last to get through; only a *newer* one does that. Going backwards is
        // `step_back`'s job, and it has the target time to do it with.
        if let Some(newest) = newest_delivery(&self.pulses)
            && self.last_delivered.is_none_or(|old| old.pulse_index < newest.pulse_index)
        {
            self.last_delivered = Some(newest);
        }
    }

    /// Establish which side of every sheet the receiver stands on, recording nothing.
    ///
    /// This is the other half of `step_back`. A rewind drops the per-sheet sides, because a side is
    /// a statement about two events and cannot be wound back; but a field left with no sides at all
    /// cannot see a crossing that happens inside the first step forward, since that step is spent
    /// finding out which side the receiver was on to begin with. Priming does that at the rewound
    /// state, where the answer is known, so the first step forward is a step like any other and the
    /// arrivals it makes are the arrivals the run made the first time through.
    ///
    /// It must be called with the receiver already rewound, which is why `SignalPair::step_back`
    /// takes the observers and why both call sites wind the worldlines back before the fields.
    ///
    /// Priming is also the one place that can tell whether an arrival the rewind kept has in fact
    /// happened at the rewound state, because it is the one place that has both the record and the
    /// sides in hand: see `Pulse::retract_unseen`, which is what `target_t` is for. `target_t` is
    /// the time the rewind landed on, which after `step_back` is the field's own clock.
    pub fn prime(&mut self, metric: &KerrSchild, receiver: &Observer, target_t: f64) {
        let u_receiver = signalling_four_velocity(metric, receiver);
        let mut retracted: Vec<Reception> = Vec::new();
        for pulse in self.pulses.iter_mut() {
            pulse.scan(metric, receiver, &u_receiver, false);
            retracted.extend(pulse.retract_unseen(target_t));
        }
        if retracted.is_empty() {
            return;
        }
        // The same arrival is held in three places, and all three have to let go of it: the pulse
        // (done above), the heard log that outlives the pulse, and the delivery record, which names
        // the arrival that made a pulse a delivery.
        self.heard.retain(|rec| {
            !retracted.iter().any(|gone| gone.pulse_index == rec.pulse_index && gone.t == rec.t)
        });
        let standing = self.last_delivered.filter(|delivery| {
            !retracted.iter().any(|gone| {
                gone.pulse_index == delivery.pulse_index && gone.t == delivery.received_t
            })
        });
        let in_hand = newest_delivery(&self.pulses);
        self.last_delivered = match (standing, in_hand) {
            (Some(a), Some(b)) if b.pulse_index > a.pulse_index => Some(b),
            (Some(a), _) => Some(a),
            (None, b) => b,
        };
    }

    /// Every crossing of the receiver's worldline this transmission has made, including those
    /// carried by pulses the `MAX_PULSES` cap has since dropped.
    pub fn receptions(&self) -> impl Iterator<Item = &Reception> {
        self.heard.iter()
    }

    /// The reception with the latest coordinate time. Arrivals do not come in emission order: the
    /// frozen family of an early pulse can reach the receiver long after the crossing family of a
    /// late one, so they have to be compared by their own event time.
    pub fn last_reception(&self) -> Option<&Reception> {
        self.receptions().max_by(|a, b| a.t.total_cmp(&b.t))
    }

    /// The largest shift the receiver has measured so far, over all arrivals.
    pub fn max_ratio(&self) -> Option<f64> {
        self.receptions().map(|r| r.ratio).reduce(f64::max)
    }

    /// How many arrivals the receiver has recorded in total, counting a pulse once per sheet of it
    /// that has swept over them rather than once per pulse.
    pub fn received_count(&self) -> usize {
        self.heard.len()
    }

    /// How many rays of this field the integrator's substep budget has retired, over the whole life
    /// of the field. Anything but zero means some ray was too stiff to integrate inside
    /// `MAX_SUBSTEPS` and was left dead at its last good event: a fact about the run that the tests
    /// assert away rather than a number the app expects to have to show.
    #[allow(dead_code)] // the integrator's safety net; the tests are its caller
    pub fn budget_exhausted(&self) -> usize {
        self.budget_exhausted
    }

    /// The emission event of the latest pulse of this transmission that has been received.
    ///
    /// Once the receiver's worldline has ended (`Observer::has_ended`) this is the last signal of
    /// the emitter's that ever arrived, and its emission event is the boundary, on the emitter's
    /// own worldline, of the causal past of the end of the receiver's: everything sent after it is
    /// sent into a region the receiver has already left, and never arrives. Nothing here predicts
    /// that boundary; it is read off the simulation, which is the only criterion this app trusts.
    ///
    /// It is reported and not drawn. The HUD names the pulse, the event it was sent from and how
    /// many later ones never arrive; neither canvas marks that event, because a lone ring on a
    /// worldline reads as a thing in the spacetime rather than as a fact about the run.
    ///
    /// It survives the `MAX_PULSES` cap, which matters: an emitter transmitting from t = 0 through
    /// a whole infall sends of order a hundred pulses against a cap of sixty-four, and the last
    /// pulse to be delivered is usually one of the first to have been sent. See `Delivery`.
    pub fn last_delivered_pulse(&self) -> Option<Delivery> {
        self.last_delivered
    }

    /// How many pulses this transmission sent after the given serial number: with
    /// `last_delivered_pulse`, the count of transmissions that were sent and never arrived.
    ///
    /// Counted from the newest pulse's serial number rather than by counting the pulses in hand, so
    /// that the answer is the number actually emitted even where the cap has dropped some of them,
    /// and so that a rewind, which un-sends the newest pulses but leaves `next_index` alone, takes
    /// the count back down with it.
    pub fn pulses_after(&self, index: usize) -> usize {
        self.pulses.last().map_or(0, |p| p.index.saturating_sub(index))
    }

    /// Drop every pulse and put the clock back to zero. This is the reset, not the rewind: it is
    /// for re-dropping the observers and for changing the geometry under them, where the standing
    /// wavefronts are null geodesics of a metric that no longer applies. Stepping the simulation
    /// backwards uses `step_back` instead, which keeps the transmission.
    pub fn clear(&mut self) {
        self.pulses.clear();
        self.t = 0.0;
        self.last_emit_tau = None;
        self.last_delivered = None;
        self.heard.clear();
        self.budget_exhausted = 0;
    }

    /// Drop everything this transmission has built but leave the field's own clock where it stands.
    ///
    /// This is what an emitter who is not transmitting leaves behind - the "Transmit Signal" box
    /// unticked, or the observer out of the simulation altogether. It differs from `clear` in the
    /// clock alone, and that is the whole reason it exists: `clear` is the reset that starts a run
    /// over, so it puts the field back to t = 0, while a silent field is still riding the same
    /// simulation clock as everything else and has to go on doing so, or the first thing sent when
    /// the box is ticked again would be dated from a clock that had never left the start.
    ///
    /// It is stated as a condition rather than as an edge: `SignalPair::advance` calls it on every
    /// step for every endpoint that is not sending, so a silent field is empty at every moment
    /// rather than only just after the click that silenced it.
    pub fn silence(&mut self) {
        let t = self.t;
        self.clear();
        self.t = t;
    }
}

/// The delivery of the newest pulse among these that has been received, or None if none has.
fn newest_delivery(pulses: &[Pulse]) -> Option<Delivery> {
    pulses
        .iter()
        .filter(|p| !p.receptions.is_empty())
        .max_by_key(|p| p.index)
        .map(|p| Delivery {
            pulse_index: p.index,
            emitted_t: p.emitted_t,
            emitted_tau: p.emitted_tau,
            emitted_r: p.emitted_r,
            received_t: p.receptions.iter().map(|r| r.t).fold(f64::INFINITY, f64::min),
        })
}

/// The two transmissions the app carries at once: Alice's, which Bob receives, and Bob's, which
/// Alice receives.
///
/// It exists so that there is exactly one description of how a step of the simulation moves both
/// fields. The play loop, the arrow keys and the panel's transport buttons all go through it, so
/// they cannot drift apart, and the order inside `advance` — carry the light, then emit, then
/// listen — is stated once instead of three times.
pub struct SignalPair<'a> {
    /// Alice's transmission: emitted by Alice, received by Bob.
    pub alice: &'a mut SignalField,
    /// Bob's transmission: emitted by Bob, received by Alice.
    pub bob: &'a mut SignalField,
}

/// One end of the two-way transmission: the observer, when they are in the simulation at all, and
/// whether they are broadcasting.
///
/// The two questions are separate and both belong here. An observer who is not in the simulation
/// neither sends nor receives; one who is there with "Transmit Signal" unticked receives normally
/// and sends nothing, and their field is dropped rather than frozen, because light nobody emitted
/// is not light standing still.
#[derive(Clone, Copy)]
pub struct Endpoint<'a> {
    pub observer: Option<&'a Observer>,
    pub transmitting: bool,
}

impl<'a> Endpoint<'a> {
    /// The emitter at this end, or None when there is nothing being sent from it.
    fn sender(&self) -> Option<&'a Observer> {
        self.observer.filter(|_| self.transmitting)
    }
}

impl SignalPair<'_> {
    /// Carry both transmissions forward by dt of the simulation clock.
    ///
    /// The order matters and is the same for each field. Advancing first and emitting second keeps
    /// a fresh pulse at its emitter's current event instead of one step behind it, and detecting
    /// last means a pulse emitted this frame already has a recorded side for its receiver before
    /// the next frame can move it.
    ///
    /// An endpoint that is not sending - the observer out of the simulation, or there with
    /// "Transmit Signal" unticked - has its field silenced first and then carried on empty, so it
    /// stays on the simulation clock while it holds nothing. The *other* field is not touched by
    /// that: the light an emitter has already sent is still in flight whether or not anyone is left
    /// to hear it, and it goes on being carried and drawn. What does stop is the record. A
    /// reception is a crossing of a receiver's worldline, so with no receiver there is nothing to
    /// detect and nothing is written down.
    pub fn advance(
        &mut self,
        metric: &KerrSchild,
        dt: f64,
        alice: Endpoint<'_>,
        bob: Endpoint<'_>,
    ) {
        if alice.sender().is_none() {
            self.alice.silence();
        }
        if bob.sender().is_none() {
            self.bob.silence();
        }
        self.alice.advance(metric, dt);
        self.bob.advance(metric, dt);
        if let Some(al) = alice.sender() {
            self.alice.emit_if_due(metric, al);
        }
        if let Some(al) = alice.observer {
            self.bob.detect_receptions(metric, al);
        }
        if let Some(b) = bob.sender() {
            self.bob.emit_if_due(metric, b);
        }
        if let Some(b) = bob.observer {
            self.alice.detect_receptions(metric, b);
        }
    }

    /// Carry both transmissions back by dt of coordinate time, undoing `advance` rather than
    /// deleting what it built, and leave each field able to see the next crossing it makes.
    ///
    /// The observers passed in must already be at the rewound state: a step backwards moves the
    /// worldlines first and the fields second, because the fields need the receivers where they now
    /// stand and the worldlines need nothing from the fields at all. Both call sites - the arrow
    /// key through `SpacetimeApp::step_backward` and the panel's Step Back button - go through this
    /// one call in that order.
    ///
    /// Priming is what makes a rewind reversible in the receptions as well as in the light.
    /// `SignalField::step_back` drops the per-sheet sides, and a field with no sides spends its
    /// first step forward re-establishing them, during which any crossing is missed. Priming
    /// re-establishes them here instead, at the rewound state, so the first step forward is an
    /// ordinary step and re-records exactly the arrival the run recorded the first time.
    pub fn step_back(
        &mut self,
        metric: &KerrSchild,
        dt: f64,
        alice: Option<&Observer>,
        bob: Option<&Observer>,
    ) {
        self.alice.step_back(metric, dt);
        self.bob.step_back(metric, dt);
        // Each field is primed against its own receiver: Alice's transmission is received by Bob,
        // and Bob's by Alice. A field's own clock is the time its rewind landed on, which is the
        // target the priming pass reconciles the reception record against. A field with no receiver
        // has no sides to re-establish and nothing to reconcile, so it is left as the rewind left
        // it.
        let (alice_target, bob_target) = (self.alice.t, self.bob.t);
        if let Some(b) = bob {
            self.alice.prime(metric, b, alice_target);
        }
        if let Some(al) = alice {
            self.bob.prime(metric, al, bob_target);
        }
    }

    /// Set how many rays the next pulse of *either* transmission will carry.
    ///
    /// The two fields are one control: a pulse of Alice's and a pulse of Bob's sampled at different
    /// densities would make the two pictures incomparable, and the panel offers one slider. This is
    /// the only way the app writes `SignalField::rays_per_pulse`, so the two cannot fall out of
    /// step, and it is called on the stepping paths, just before the step, rather than at the
    /// moment of the click: the count is a standing request about the next emission, like every
    /// other setting the transport reads as it goes.
    pub fn set_rays_per_pulse(&mut self, rays: usize) {
        self.alice.rays_per_pulse = rays;
        self.bob.rays_per_pulse = rays;
    }

    /// Drop both transmissions and put both clocks back to zero: the reset that re-dropping the
    /// observers or changing the geometry under them needs.
    pub fn clear(&mut self) {
        self.alice.clear();
        self.bob.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::geodesic::GeodesicState;
    use crate::physics::observer::{Observer, WorldlineParams};

    /// Raindrop (E = 1, L = 0) 4-velocity, the one frame that exists at every r > 0.
    fn raindrop(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let (ut, ur, up) = GeodesicState::new_infall(metric, 0.0, r, 1.0, 0.0).derivatives(metric, r);
        [ut, ur, up]
    }

    /// A ray with an explicit coordinate direction, for the closed-form comparisons.
    fn ray_with_slopes(metric: &KerrSchild, r: f64, dr_dt: f64, dphi_dt: f64) -> NullRay {
        let v = [1.0, dr_dt, dphi_dt];
        let u = raindrop(metric, r);
        NullRay {
            t: 0.0,
            r,
            phi: 0.0,
            dr_dt,
            dphi_dt,
            f_emit: f_factor(metric, r, &v, &u),
            v_emit: v,
            death_t: None,
            death_end: None,
        }
    }

    #[test]
    fn test_rays_stay_null_and_conserve_l_over_e() {
        // Six directions out of a raindrop frame at r = 3, integrated for 40M of coordinate time.
        // g(v, v) = 0 and L/E are both exact statements about a null geodesic, so any drift in
        // them is pure integration error. Measured worst over the six, at 0.01 M steps: 2.3e-13 in
        // g(v, v) and 4.2e-10 relative in L/E. The tolerances below were 1e-8 and 1e-7, set against
        // the fixed-subdivision RK4 this integrator replaces; they are tightened here to a couple
        // of orders of magnitude above what the adaptive Dormand-Prince scheme actually delivers,
        // and the measurement is printed so that the next change to the integrator has a number to
        // beat rather than a bound to fit under.
        let metric = KerrSchild::new(1.0, 0.65);
        let r0 = 3.0;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let mut worst_norm = 0.0f64;
        let mut worst_l = 0.0f64;
        for i in 0..6 {
            let alpha = 2.0 * std::f64::consts::PI * (i as f64) / 6.0;
            let mut ray = NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u);
            let l_over_e0 = ray.l_over_e(&metric);
            let mut steps = 0;
            while ray.alive() && steps < 4000 {
                ray.step(&metric, 0.01);
                steps += 1;
                if !ray.alive() {
                    break;
                }
                let n = metric.norm(ray.r, &ray.direction());
                worst_norm = worst_norm.max(n.abs());
                assert!(
                    n.abs() < 1e-11,
                    "g(v, v) = {n} at r={} on alpha={alpha}",
                    ray.r
                );
                // L/E is a quotient of two quantities that both vanish as a ray freezes onto r-
                // (k_phi/k^t and E/k^t alike go to zero there, since the direction tends to the
                // horizon generator), so the ratio loses significant digits at exactly the rate the
                // ray freezes. It is checked while the ray is still well clear of that limit.
                let e_over_kt = {
                    let g = metric.metric_components(ray.r);
                    let v = ray.direction();
                    -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2])
                };
                if e_over_kt.abs() > 1e-3 {
                    let l = ray.l_over_e(&metric);
                    worst_l = worst_l.max((l - l_over_e0).abs() / (1.0 + l_over_e0.abs()));
                    assert!(
                        (l - l_over_e0).abs() < 1e-8 * (1.0 + l_over_e0.abs()),
                        "L/E = {l} vs {l_over_e0} at r={} on alpha={alpha}",
                        ray.r
                    );
                }
            }
        }
        println!(
            "over six rays and 40 M of coordinate time: worst |g(v, v)| {worst_norm:.3e}, worst \
             relative drift in L/E {worst_l:.3e}"
        );
    }

    #[test]
    fn test_outgoing_principal_ray_freezes_onto_the_cauchy_horizon() {
        // Started on the outgoing PND inside r+, the ray must stay on it: the PND congruence is
        // geodesic, so the closed-form slope Delta / (r^2 + a^2 + 2Mr) is a solution of the
        // equation being integrated. It closes on r- from above and never crosses.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let s = metric.radial_null_slopes(1.2);
        let mut ray = ray_with_slopes(&metric, 1.2, s.dr_dt_outgoing, s.dphi_dt_outgoing);
        for _ in 0..1200 {
            ray.step(&metric, 0.01);
            let expected = metric.radial_null_slopes(ray.r).dr_dt_outgoing;
            assert!(
                (ray.dr_dt - expected).abs() < 1e-6,
                "dr/dt = {} vs PND {expected} at r={}",
                ray.dr_dt,
                ray.r
            );
            assert!(ray.r > rm, "the outgoing ray must not cross r- = {rm}: r = {}", ray.r);
        }
        assert!(
            ray.r - rm < 1e-3,
            "after 12M of t the ray should be pinned to r-: r - r- = {}",
            ray.r - rm
        );
    }

    #[test]
    fn test_ingoing_principal_ray_runs_at_minus_one_and_crosses_r_minus() {
        // The ingoing PND is the generator of this chart: dr/dt = -1 at every radius, exactly.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let mut ray = ray_with_slopes(&metric, 1.2, -1.0, 0.0);
        let mut crossed_at = None;
        for i in 0..200 {
            ray.step(&metric, 0.01);
            assert!(
                (ray.dr_dt + 1.0).abs() < 1e-9,
                "ingoing dr/dt = {} at r={}",
                ray.dr_dt,
                ray.r
            );
            if crossed_at.is_none() && ray.r < rm {
                crossed_at = Some(0.01 * (i + 1) as f64);
            }
        }
        let t_cross = crossed_at.expect("the ingoing ray must cross r- at finite t");
        assert!(
            (t_cross - (1.2 - rm)).abs() < 0.02,
            "crossing at t = {t_cross}, expected {}",
            1.2 - rm
        );
    }

    #[test]
    fn test_inner_horizon_generator_energy_decides_which_rays_cross() {
        // Sweep a whole interior light cone and check the exact criterion of the module header:
        // a ray crosses r- at finite coordinate time when its energy relative to the null generator
        // of r-, E - Omega_- L, is positive, and freezes onto r- when that is negative. Note what
        // this is *not*: the arc that freezes is centred on the direction dragged forward in phi,
        // not on the observer's own outward radial leg, which crosses like the rest.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let omega_minus = metric.a / (rm * rm + metric.a * metric.a);
        for &r0 in &[1.7, 1.2, 0.6] {
            let u = raindrop(&metric, r0);
            let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
            for i in 0..24 {
                let alpha = -std::f64::consts::PI
                    + 2.0 * std::f64::consts::PI * (i as f64) / 24.0;
                let mut ray =
                    NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u);
                // E and L per unit k^t; the positive scale k^t does not affect the sign.
                let g = metric.metric_components(r0);
                let v = ray.direction();
                let e = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
                let l = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
                let relative_energy = e - omega_minus * l;
                if relative_energy.abs() < 1e-3 {
                    continue; // the marginal ray, where the two radial branches merge
                }

                let mut crossed = false;
                for _ in 0..1200 {
                    ray.step(&metric, 0.05);
                    if ray.r < rm {
                        crossed = true;
                        break;
                    }
                    if !ray.alive() {
                        break;
                    }
                }
                assert_eq!(
                    crossed,
                    relative_energy > 0.0,
                    "E - Omega_- L = {relative_energy} at alpha={alpha}, r0={r0}: crossed={crossed}"
                );
            }
        }
    }

    #[test]
    fn test_the_inward_leg_crosses_and_the_dragged_leg_freezes() {
        // The two ends of the same statement, in the plainest form: the raindrop's own inward
        // radial ray crosses r- quickly, and the ray she sends along the outgoing principal null
        // direction is still above r- after 60M of coordinate time, with its dr/dt down to nothing.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let r0 = 1.2;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);

        let mut inward = NullRay::from_local_direction(
            &metric,
            0.0,
            r0,
            0.0,
            &tetrad,
            std::f64::consts::PI,
            &u,
        );
        let mut crossed = false;
        for _ in 0..600 {
            inward.step(&metric, 0.01);
            if inward.r < rm {
                crossed = true;
                break;
            }
        }
        assert!(crossed, "the inward ray must cross r-: r = {}", inward.r);

        let s = metric.radial_null_slopes(r0);
        let mut dragged = ray_with_slopes(&metric, r0, s.dr_dt_outgoing, s.dphi_dt_outgoing);
        for _ in 0..1200 {
            dragged.step(&metric, 0.05);
            // After tens of M the offset r - r- has decayed through the last representable bit,
            // so equality with r- is the floating-point end of "never crosses".
            assert!(
                dragged.r >= rm,
                "the dragged ray must not cross r- = {rm}: r = {}",
                dragged.r
            );
        }
        assert!(dragged.dr_dt.abs() < 1e-6, "it should have frozen: dr/dt = {}", dragged.dr_dt);
    }

    #[test]
    fn test_limiting_blueshift_is_the_inner_surface_gravity_exponential() {
        let metric = KerrSchild::new(1.0, 0.65);
        let kappa = metric.inner_surface_gravity();
        assert!((limiting_blueshift(&metric, 4.0) - (4.0 * kappa).exp()).abs() < 1e-9);
        assert!((limiting_blueshift(&metric, 4.0) - 562.0).abs() < 2.0);
        assert!((limiting_blueshift(&metric, 8.0) / 3.16e5 - 1.0).abs() < 0.02);
        assert!((limiting_blueshift(&metric, 0.0) - 1.0).abs() < 1e-12);
    }

    /// One transmission: Alice released from r = 4.5M at t = 0 at azimuth `alice_phi`, Bob held at
    /// the same radius until t = `delta_t` and then released at azimuth 0, both raindrops. Returns
    /// every arrival Bob recorded, ordered by its own event time, together with the emission time
    /// of each pulse.
    ///
    /// The step refines as Bob descends. Out in the open nothing needs resolving to better than a
    /// fiftieth of an M; the stack on r- is a few thousandths of an M thick and a sheet has to be
    /// seen on two consecutive passes for its crossing to register, so over the last fiftieth of an
    /// M the step becomes a fixed fraction of Bob's remaining offset above r-. He then approaches
    /// the Cauchy horizon geometrically and never jumps across it, so every sheet is met at the
    /// radius it actually stands at, outside r-, rather than being lumped onto the one radius below
    /// r- that a fixed step happens to land on. Ray accuracy itself does not depend on any of this,
    /// since `NullRay::step` substeps internally on its own caps.
    fn run_transmission(
        metric: &KerrSchild,
        alice_phi: f64,
        delta_t: f64,
    ) -> (Vec<Reception>, Vec<(usize, f64)>) {
        let rm = metric.inner_horizon();
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, alice_phi, params);
        let mut bob = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, delta_t, 0.0, params);
        let mut field = SignalField::default();

        let mut t = 0.0;
        while t < 60.0 {
            let dt = if bob.r > 1.0 {
                0.02
            } else if bob.r > rm + 0.02 {
                0.005
            } else {
                // A raindrop covers about 3 M of radius per M of coordinate time here, so a step of
                // a twentieth of the offset moves him about 15% of the way to r-: fine enough to
                // resolve the sheets and coarse enough to reach the cut-off in a few dozen steps.
                0.05 * (bob.r - rm)
            };
            t += dt;
            alice.step(metric, t, dt);
            bob.step(metric, t, dt);
            field.advance(metric, dt);
            field.emit_if_due(metric, &alice);
            field.detect_receptions(metric, &bob);
            if bob.r < rm + 1e-4 || bob.geodesic.map(|g| g.stalled).unwrap_or(false) {
                break;
            }
        }

        let emitted = field.pulses.iter().map(|p| (p.index, p.emitted_t)).collect();
        let mut heard: Vec<Reception> = field.receptions().copied().collect();
        heard.sort_by(|a, b| a.t.total_cmp(&b.t));
        (heard, emitted)
    }

    /// The sheets of the stack Bob crossed in the last 0.05M above r-, outermost first.
    fn stack_sheets(metric: &KerrSchild, heard: &[Reception]) -> Vec<Reception> {
        let rm = metric.inner_horizon();
        let mut sheets: Vec<Reception> = heard
            .iter()
            .copied()
            .filter(|r| r.frozen_family && r.r > rm && r.r < rm + 0.05)
            .collect();
        sheets.sort_by(|a, b| b.r.total_cmp(&a.r));
        sheets
    }

    /// What must hold for any transmission, at any azimuth and any release delay.
    fn assert_transmission(
        metric: &KerrSchild,
        label: &str,
        heard: &[Reception],
        emitted: &[(usize, f64)],
    ) {
        let kappa = metric.inner_surface_gravity();
        let emitted_t = |index: usize| -> f64 {
            emitted
                .iter()
                .find(|(i, _)| *i == index)
                .map(|(_, t)| *t)
                .expect("every reception belongs to a live pulse")
        };

        assert!(heard.len() >= 5, "{label}: Bob should hear the transmission: {heard:?}");
        for reception in heard.iter() {
            assert!(
                reception.ratio.is_finite() && reception.ratio > 0.0,
                "{label}: every measured shift is finite and positive: {reception:?}"
            );
            // The exponential ceiling: no ray can be blueshifted by more than the freezing factor
            // exp(kappa_- (t_receive - t_emit)) that r- can accumulate over the interval.
            let ceiling = (kappa * (reception.t - emitted_t(reception.pulse_index))).exp();
            assert!(
                reception.ratio < 1.05 * ceiling,
                "{label}: {reception:?} passes the kappa_- ceiling {ceiling}"
            );
        }
        // The sheets met on r- itself, which is where the blueshift lives.
        let sheets = stack_sheets(metric, heard);
        for sheet in sheets.iter() {
            assert!(
                sheet.ratio > 10.0,
                "{label}: a sheet met on r- is a large blueshift: {sheet:?}"
            );
        }
        // No ordering is asserted across sheets of different pulses. In the runs measured so far
        // the shift falls as Bob goes deeper, because the deepest sheets belong to the pulses Alice
        // sent closest to r-, which had the least distance to fall and so collapsed onto r- fastest
        // while also being the youngest; but that is a property of one emitter's descent, not a
        // theorem. The ordering that does follow from the geometry is the one inside a single arc,
        // where the deeper rays are the ones that froze earliest, and
        // `test_the_frozen_stack_blueshifts_inward` measures that one.
    }

    #[test]
    fn test_bob_receives_alices_pulses_with_growing_blueshift() {
        // The app's own geometry: Alice released from r = 4.5M at t = 0 at the azimuth the app puts
        // her on, Bob held at the same radius and released at the app's default delay. Both are
        // raindrops (E = 1, L = 0), so their worldlines are one curve shifted in coordinate time,
        // and Bob hears the transmission in two acts. The crossing family of each pulse sweeps over
        // him on the way down and arrives with an ordinary shift. Then, in the last twentieth of an
        // M above r-, he cuts through the frozen arcs that have been standing on the Cauchy horizon
        // waiting for him, several sheets of them one after another, each of them a blueshift of
        // one to two orders of magnitude.
        //
        // That second act only reads as a stack because the transmission is dense. A frozen arc
        // co-rotates at Omega_- = a / (r-^2 + a^2) while it waits, so consecutive arcs are carried
        // apart in azimuth at that rate; at the emission interval used here they are about a quarter
        // of a radian apart against an arc some half a radian wide, so they overlap and Bob meets
        // several of them in a row instead of one or none.
        let metric = KerrSchild::new(1.0, 0.65);
        let (heard, emitted) = run_transmission(&metric, 0.25, 8.0);
        assert!(
            heard.iter().any(|r| !r.frozen_family),
            "the crossing family must sweep past Bob as well: {heard:?}"
        );
        assert_transmission(&metric, "phi=0.25 Dt=8", &heard, &emitted);

        let sheets = stack_sheets(&metric, &heard);
        assert!(
            sheets.len() >= 3,
            "Bob must cross several sheets of the stack: {heard:?}"
        );
        // The loudest thing he hears in the whole transmission comes from the frozen family: either
        // one of these sheets or an arc he met a little further out, on its way onto r-.
        let loudest = heard
            .iter()
            .max_by(|a, b| a.ratio.total_cmp(&b.ratio))
            .expect("the run recorded arrivals");
        assert!(
            loudest.frozen_family && loudest.ratio > 10.0,
            "the largest shift of the run belongs to the frozen family: {loudest:?}"
        );
    }

    #[test]
    fn test_the_stack_is_met_across_azimuths_and_delays() {
        // Four azimuths a radian or more apart, two release delays, run in parallel because each
        // one is a full infall. Every configuration has to satisfy the invariants of
        // `assert_transmission`; how much of the stack Bob meets on r- is the part that depends on
        // where he crosses, and the sweep pins down how often that happens.
        //
        // Measured, with the emission interval of `EMISSION_INTERVAL_TAU` and Alice broadcasting
        // into her whole light cone: six of the eight give him three or more sheets on r-, in job
        // order (Dt = 4 at phi = 0.25, 1.4, 3.0, 5.0, then Dt = 8 at the same four azimuths) the
        // counts are 30, 18, 0, 10, 3, 0, 35, 35. Two crossings still meet no sheet at all on r-
        // and hear the frozen family further out instead, where it has not finished freezing and
        // the shift is only single figures. See the module header: one emitter's transmission
        // illuminates a band of the Cauchy horizon rather than all of it, and the band is a good
        // deal wider now that the prograde arc is emitted whole rather than cut off at alpha = 90
        // degrees, which is where the outward half of the cone used to end.
        let metric = KerrSchild::new(1.0, 0.65);
        let mut jobs = Vec::new();
        for &delta_t in &[4.0f64, 8.0] {
            for &alice_phi in &[0.25f64, 1.4, 3.0, 5.0] {
                jobs.push((alice_phi, delta_t));
            }
        }

        let outcomes: Vec<usize> = std::thread::scope(|scope| {
            let handles: Vec<_> = jobs
                .iter()
                .map(|&(alice_phi, delta_t)| {
                    let metric = &metric;
                    scope.spawn(move || {
                        let (heard, emitted) = run_transmission(metric, alice_phi, delta_t);
                        let label = format!("phi={alice_phi} Dt={delta_t}");
                        assert_transmission(metric, &label, &heard, &emitted);
                        stack_sheets(metric, &heard).len()
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        let with_a_stack = outcomes.iter().filter(|n| **n >= 3).count();
        assert!(
            with_a_stack >= 3,
            "several crossings must land on the stack: sheets per run = {outcomes:?}"
        );
    }

    #[test]
    fn test_the_frozen_stack_blueshifts_inward() {
        // The radial structure of the stack, without any azimuth luck in the way. A full pulse,
        // the whole light cone, is emitted in Region II and left to settle for 8M of coordinate
        // time, by which point its frozen family is strung out along r- with the rays that froze
        // earliest sitting deepest. At r = 1.2 the frozen arc runs from alpha = 35 to 150 degrees,
        // so a little under a third of the rays end up on the surface. Reading the
        // shift each of them carries for a raindrop crossing at that ray's own radius gives the
        // profile an infaller sees as they fall through: deeper is later light, and later light has
        // spent longer on the exponential, so the blueshift climbs monotonically inward.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let r0 = 1.2;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        let mut rays: Vec<NullRay> = (0..RAYS_PER_PULSE)
            .map(|i| {
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        for _ in 0..1600 {
            for ray in rays.iter_mut() {
                ray.step(&metric, 0.005);
            }
        }

        let mut profile: Vec<(f64, f64)> = rays
            .iter()
            .filter(|ray| {
                ray.alive() && ray.inner_horizon_energy(&metric) < 0.0 && ray.r < rm + 0.05
            })
            .map(|ray| (ray.r, ray.frequency_ratio(&metric, &raindrop(&metric, ray.r))))
            .collect();
        assert!(
            profile.len() >= 20,
            "the frozen family should be the whole prograde arc: {profile:?}"
        );
        profile.sort_by(|a, b| b.0.total_cmp(&a.0));
        // The ordering holds across the settled part of the arc. Its outer edge is the ray with
        // E - Omega_- L closest to zero from below, which freezes the slowest of all and after 8 M
        // is still on its way in, so it can sit a hair outside a neighbour that froze earlier and
        // carries less shift. With the cone sampled at 2.5 degrees that edge ray is caught
        // (at five degrees it fell in the gap), and it is the only inversion allowed: it must be
        // the shallowest pair, and everything deeper must be strictly bluer.
        let inversions: Vec<usize> = profile
            .windows(2)
            .enumerate()
            .filter(|(_, w)| w[1].1 <= w[0].1)
            .map(|(k, _)| k)
            .collect();
        assert!(
            inversions.is_empty() || inversions == [0],
            "deeper in the stack must be bluer, except at the arc's unsettled outer edge:              inversions at {inversions:?} of {profile:?}"
        );
        let (shallowest, deepest) = (profile[0], profile[profile.len() - 1]);
        assert!(
            deepest.1 > 100.0 * shallowest.1,
            "the deep end of the stack must be far bluer than the shallow end: {shallowest:?}              against {deepest:?}"
        );
        assert!(
            profile.last().unwrap().1 > 100.0 * profile[0].1,
            "and the profile must span orders of magnitude: {profile:?}"
        );
    }

    /// The exact radial potential of an equatorial null geodesic of conserved (E, L), divided by
    /// the overall r that every term of it carries:
    ///
    ///     R(r) = [E (r^2 + a^2) - a L]^2 - Delta (L - a E)^2
    ///          = r [ E^2 r^3 + (a^2 E^2 - L^2) r + 2 M (L - a E)^2 ],
    ///
    /// so this is the cubic in the bracket. The motion needs R >= 0, so a ray falling inward stops
    /// at the largest zero of this cubic below it and comes back out; a ray with no zero below it
    /// reaches r = 0. Nothing about the ray's history enters, and the whole thing is homogeneous of
    /// degree two in (E, L), so the per-unit-k^t values the integration carries give the same roots
    /// as the affinely normalised ones.
    fn radial_potential(metric: &KerrSchild, e: f64, l: f64, r: f64) -> f64 {
        let q = l - metric.a * e;
        e * e * r * r * r + (metric.a * metric.a * e * e - l * l) * r + 2.0 * metric.m * q * q
    }

    /// The exact turning radius of such a ray somewhere below `from`: the largest root of
    /// `radial_potential` in (0, from), or None if the potential stays positive all the way down,
    /// in which case nothing stops the ray reaching the ring.
    fn turning_radius(metric: &KerrSchild, e: f64, l: f64, from: f64) -> Option<f64> {
        let steps = 20_000;
        let mut hi = from;
        let mut hi_value = radial_potential(metric, e, l, hi);
        for i in 1..=steps {
            let lo = from * (1.0 - (i as f64) / (steps as f64));
            let lo_value = radial_potential(metric, e, l, lo);
            if hi_value * lo_value <= 0.0 {
                // Bisect the bracket to machine precision.
                let (mut a, mut b) = (lo, hi);
                for _ in 0..80 {
                    let mid = 0.5 * (a + b);
                    if radial_potential(metric, e, l, mid) * lo_value > 0.0 {
                        a = mid;
                    } else {
                        b = mid;
                    }
                }
                return Some(0.5 * (a + b));
            }
            hi = lo;
            hi_value = lo_value;
        }
        None
    }

    /// The whole of an emitter's light cone at (t, r, phi) = (0, r0, 0), let go from a raindrop
    /// frame: the same construction `SignalField::emit_if_due` makes, and the one the fate tests
    /// need at radii no observer of the app happens to be standing at.
    fn cone_at(metric: &KerrSchild, r0: f64) -> Vec<NullRay> {
        let u = raindrop(metric, r0);
        let tetrad = Tetrad::from_four_velocity(metric, r0, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        (0..RAYS_PER_PULSE)
            .map(|i| {
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(metric, 0.0, r0, 0.0, &tetrad, alpha, &u)
            })
            .collect()
    }

    /// dr/dt of the null direction at radius r with the given dphi/dt, on the outgoing branch if
    /// `outgoing` and on the ingoing one otherwise.
    ///
    /// Writing the direction as v = (1, dr/dt, w), the null condition g_{mu nu} v^mu v^nu = 0 is a
    /// quadratic in dr/dt with coefficients g_rr, 2 (g_tr + g_rphi w) and g_tt + 2 g_tphi w +
    /// g_phiphi w^2. Solving it is what lets a test put an exact null ray at a radius and a
    /// direction of its own choosing rather than at one an emitter's tetrad happens to offer.
    fn null_slope(metric: &KerrSchild, r: f64, dphi_dt: f64, outgoing: bool) -> f64 {
        let g = metric.metric_components(r);
        let (w, quad_a) = (dphi_dt, g[1][1]);
        let quad_b = 2.0 * (g[0][1] + g[1][2] * w);
        let quad_c = g[0][0] + 2.0 * g[0][2] * w + g[2][2] * w * w;
        let root = (quad_b * quad_b - 4.0 * quad_a * quad_c).sqrt();
        assert!(root.is_finite(), "no null direction at r = {r} with dphi/dt = {w}");
        let (lo, hi) = ((-quad_b - root) / (2.0 * quad_a), (-quad_b + root) / (2.0 * quad_a));
        if outgoing { lo.max(hi) } else { lo.min(hi) }
    }

    /// The horizon-blind fate tests this pair replaced, kept here as the thing the new ones are
    /// measured against: no zero of the potential between the ray and the boundary, and either
    /// already going that way or a zero somewhere on the other side to turn at - with no question
    /// asked about whether the ray can get to that zero through the horizons in between.
    fn horizon_blind_fates(metric: &KerrSchild, ray: &NullRay) -> (bool, bool) {
        let below = ray.turns_between(metric, R_STOP, ray.r);
        let above = ray.turns_between(metric, ray.r, R_ESCAPE);
        (
            ray.alive() && !below && (ray.dr_dt < 0.0 || above),
            ray.alive() && !above && (ray.dr_dt > 0.0 || below),
        )
    }

    #[test]
    fn test_a_ray_frozen_onto_r_minus_has_no_fate() {
        // A ray of the frozen family closes on r- as r - r- ~ exp(-kappa_- t) and never crosses
        // it, so at no finite coordinate time does it reach either boundary of the drawn field: it
        // is neither ring-bound nor escape-bound, however long the run goes on. What breaks that
        // in floating point is that the offset runs out of exponent long before the run runs out
        // of time. At a = 0.90, kappa_- = 0.386/M and r- = 0.5641, whose ulp is 1.1e-16, so
        // exp(-kappa_- t) passes under the spacing of f64 near r- at about t = 100 M. Every frozen
        // ray then rounds onto r-, and the integrator's own round-off carries some of them an ulp
        // or two past it, into Region III where Delta has changed sign and the equation hands them
        // a dr/dt of the other sign.
        //
        // A whole cone is let go at r = 0.9, which at this spin is between the horizons, and
        // carried to t = 120 M. What is asserted is that every ray that has settled onto r- -
        // within 1e-9 of it, on either side - has no fate at all; what is reported is how many of
        // them the horizon-blind test claimed one for.
        let metric = KerrSchild::new(1.0, 0.90);
        let (rm, rp) = (metric.inner_horizon(), metric.outer_horizon());
        let mut rays = cone_at(&metric, 0.9);
        let frozen_family: Vec<bool> = rays.iter().map(|ray| ray.frozen(&metric)).collect();
        for _ in 0..120 {
            for ray in rays.iter_mut() {
                ray.step(&metric, 1.0);
            }
        }

        let mut settled = 0;
        let mut from_above = 0;
        let mut worst_offset = 0.0f64;
        let (mut blind_ring, mut blind_escape) = (0, 0);
        for (i, ray) in rays.iter().enumerate() {
            if !ray.alive() || (ray.r - rm).abs() > 1e-9 {
                continue;
            }
            settled += 1;
            from_above += usize::from(frozen_family[i]);
            worst_offset = worst_offset.max((ray.r - rm).abs());
            let (blind_a, blind_b) = horizon_blind_fates(&metric, ray);
            blind_ring += usize::from(blind_a);
            blind_escape += usize::from(blind_b);
            assert!(
                !ray.ring_bound(&metric),
                "ray {i} has settled onto r- at r - r- = {:.3e} (dr/dt = {:.3e}) and cannot reach \
                 the ring at any finite coordinate time",
                ray.r - rm,
                ray.dr_dt
            );
            assert!(
                !ray.escape_bound(&metric),
                "ray {i} has settled onto r- at r - r- = {:.3e}, well inside r+ = {rp}, so it \
                 cannot reach R_ESCAPE",
                ray.r - rm
            );
        }
        assert!(settled > 0, "the run has to produce the case it is about");
        assert!(from_above > 0, "and some of them have to be the family that froze from above");

        // And the same statement made by hand rather than by round-off: the outgoing principal
        // null direction one ulp inside r-, which is the direction the frozen family tends to and
        // the generator of the surface it is standing on.
        let below = f64::from_bits(rm.to_bits() - 1);
        let pnd = metric.radial_null_slopes(below);
        let generator = ray_with_slopes(&metric, below, pnd.dr_dt_outgoing, pnd.dphi_dt_outgoing);
        assert!(
            !generator.ring_bound(&metric) && !generator.escape_bound(&metric),
            "the generator of r- one ulp below it reaches neither boundary: r - r- = {:.3e}, \
             dr/dt = {:.3e}",
            generator.r - rm,
            generator.dr_dt
        );
        println!(
            "a cone from r = 0.9 at a = 0.90, carried to t = 120 M: {settled} of {} rays have \
             settled onto r- = {rm:.6} (worst offset {worst_offset:.3e} M; {from_above} of them \
             the family that froze onto it from above), and none of them is ring-bound or \
             escape-bound. The horizon-blind test called {blind_ring} of them ring-bound and \
             {blind_escape} escape-bound",
            rays.len()
        );
    }

    #[test]
    fn test_an_outgoing_ray_inside_r_minus_climbs_to_r_minus_and_is_not_ring_bound() {
        // Nothing crosses r- outward in this chart: a ray inside it that moves outward asymptotes
        // to r- from below, ending on the other branch of the Cauchy horizon, which this chart
        // does not cover. A zero of its potential above r- is therefore a turning point it never
        // gets to, and reading one as "it will turn there and come back down to the ring" is wrong
        // twice over - the ray neither turns there nor comes back.
        //
        // The ray is built by hand rather than taken from a cone: dphi/dt is chosen and the null
        // condition solved for the outgoing dr/dt, so it is an exact null direction with an exact
        // potential. Both halves of the claim are then checked, the classification and the motion
        // it is a claim about.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let r0 = 0.3;
        let dphi_dt = 1.2;
        let dr_dt = null_slope(&metric, r0, dphi_dt, true);
        let mut ray = ray_with_slopes(&metric, r0, dr_dt, dphi_dt);
        assert!(dr_dt > 0.0, "the ray has to be the outgoing one: dr/dt = {dr_dt}");
        assert!(
            !ray.turns_between(&metric, R_STOP, rm),
            "and it has to be the case with nothing below r- to stop it"
        );
        assert!(!ray.ring_bound(&metric), "so it is not ring-bound");
        assert!(!ray.escape_bound(&metric), "and it is nowhere near escaping");

        // 80 M of coordinate time later it is still alive, still inside r-, and standing on it.
        let mut highest = ray.r;
        for _ in 0..800 {
            ray.step(&metric, 0.1);
            if ray.alive() {
                highest = highest.max(ray.r);
            }
        }
        assert!(ray.alive(), "it must not have reached the ring: died at {:?}", ray.death_t);
        assert!(
            ray.r < rm && rm - ray.r < 1e-6,
            "it must be standing on r- = {rm} from below, not at r = {}",
            ray.r
        );
        assert!(highest < rm, "and it must never have crossed r-: highest r = {highest}");
        println!(
            "an outgoing null ray at r = {r0} inside r- = {rm:.6} (dphi/dt = {dphi_dt}, \
             dr/dt = {dr_dt:.4}, L/E = {:.4}): after 80 M it is alive at r - r- = {:.3e}, having \
             climbed to r- from below and never crossed it",
            ray.l_over_e(&metric),
            ray.r - rm
        );
    }

    #[test]
    fn test_nothing_between_the_horizons_can_escape() {
        // Delta < 0 between the horizons, so R(r) = [E (r^2 + a^2) - a L]^2 - Delta (L - a E)^2 is
        // strictly positive there - the trapped region holds no turning point at all - and every
        // null ray in it has dr/dt < 0. Two consequences the fate tests have to carry: nothing in
        // there is escape-bound, whatever the potential does out beyond r+, and the climbing arm
        // of `ring_bound` is unreachable, so what is left of that test in the trapped region is
        // the potential below the ray and the family the ray belongs to.
        //
        // A ray of the crossing family goes through r- at finite coordinate time and then has
        // nothing but its own potential between it and the ring; a ray of the frozen family never
        // gets through r- at all, and so reaches neither boundary. A whole cone is checked at
        // emission, every ray of it against both statements.
        let metric = KerrSchild::new(1.0, 0.90);
        let (rm, rp) = (metric.inner_horizon(), metric.outer_horizon());
        let r0 = 0.9;
        assert!(r0 > rm && r0 < rp, "the cone has to be let go inside the trapped region");
        let rays = cone_at(&metric, r0);
        let (mut ring_bound, mut frozen_rays, mut with_zero_below) = (0, 0, 0);
        for (i, ray) in rays.iter().enumerate() {
            assert!(ray.dr_dt < 0.0, "ray {i} moves outward at r = {r0} between the horizons");
            assert!(!ray.escape_bound(&metric), "ray {i} cannot escape from between the horizons");
            let zero_below = ray.turns_between(&metric, R_STOP, ray.r);
            let frozen = ray.frozen(&metric);
            with_zero_below += usize::from(zero_below);
            frozen_rays += usize::from(frozen);
            assert_eq!(
                ray.ring_bound(&metric),
                !zero_below && !frozen,
                "ray {i} between the horizons: ring-bound is the potential below it (zero below: \
                 {zero_below}) and the family it belongs to (frozen: {frozen}), and nothing else"
            );
            ring_bound += usize::from(ray.ring_bound(&metric));
        }
        println!(
            "a cone let go at r = {r0}, between r- = {rm:.4} and r+ = {rp:.4}: all {} of its rays \
             fall and none is escape-bound; {frozen_rays} belong to the frozen family and \
             {with_zero_below} have a turning point above the ring, leaving {ring_bound} \
             ring-bound",
            rays.len()
        );
    }

    #[test]
    fn test_a_pulse_inside_r_minus_ends_on_the_ring_or_turns() {
        // What becomes of a pulse emitted inside the Cauchy horizon, measured against the exact
        // radial potential of each ray rather than against an expectation. A whole light cone is
        // let go at r = 0.45 at a = 0.90 (r- = 0.564, so this is Region III) and integrated for
        // 20 M of coordinate time.
        //
        // The potential is R(r) = r [E^2 r^3 + (a^2 E^2 - L^2) r + 2M (L - aE)^2], and near r = 0
        // the last two terms decide: R ~ 2M (L - aE)^2 r + (a^2 E^2 - L^2) r^2, positive for small
        // r whenever L != aE, so a ray with no zero of the cubic above the ring falls all the way
        // in. That is most of the cone. The rays that do turn are the ones whose L/E is far enough
        // above a for the cubic's negative middle term to win before the constant does, and they
        // turn arbitrarily close to the ring as L/E approaches that boundary.
        //
        // Three things are asserted, all of them exact statements about the geodesics: no live ray
        // ever goes below the turning radius of its own (E, L); every ray whose potential stays
        // positive down to R_STOP reaches the ring; and every ray that survives 20 M ends on r-,
        // from below, which is where the outgoing family of Region III accumulates.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let r0 = 0.45;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        let mut rays: Vec<NullRay> = (0..RAYS_PER_PULSE)
            .map(|i| {
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u)
            })
            .collect();

        // (E, L) per unit k^t at emission, and the turning radius they imply.
        let constants: Vec<(f64, f64)> = rays
            .iter()
            .map(|ray| {
                let g = metric.metric_components(ray.r);
                let v = ray.direction();
                let e = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
                let l = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
                (e, l)
            })
            .collect();
        let turning: Vec<Option<f64>> = constants
            .iter()
            .map(|&(e, l)| turning_radius(&metric, e, l, r0))
            .collect();

        let mut lowest: Vec<f64> = rays.iter().map(|ray| ray.r).collect();
        let mut l_over_e_drift = 0.0f64;
        let start_l_over_e: Vec<f64> = rays.iter().map(|ray| ray.l_over_e(&metric)).collect();
        let dt = 0.01;
        for _ in 0..2000 {
            for (i, ray) in rays.iter_mut().enumerate() {
                ray.step(&metric, dt);
                if ray.alive() {
                    lowest[i] = lowest[i].min(ray.r);
                }
            }
        }

        let mut died = 0;
        let mut survived = 0;
        let mut worst_below_turning = 0.0f64;
        let mut worst_off_r_minus = 0.0f64;
        for (i, ray) in rays.iter().enumerate() {
            let (e, l) = constants[i];
            let floor = turning[i].unwrap_or(0.0);
            worst_below_turning = worst_below_turning.max(floor - lowest[i]);
            assert!(
                lowest[i] > floor - 1e-3,
                "ray {i} reached r = {} below its turning radius {floor} (E = {e}, L = {l})",
                lowest[i]
            );
            if ray.alive() {
                survived += 1;
                worst_off_r_minus = worst_off_r_minus.max((ray.r - rm).abs());
                assert!(
                    ray.r < rm && rm - ray.r < 1e-3,
                    "ray {i} survived 20 M but is at r = {} rather than on r- = {rm} from below",
                    ray.r
                );
                // The scale-free constant of the motion, over the whole 20 M.
                let drift = (ray.l_over_e(&metric) - start_l_over_e[i]).abs()
                    / (1.0 + start_l_over_e[i].abs());
                l_over_e_drift = l_over_e_drift.max(drift);
            } else {
                died += 1;
                assert!(
                    turning[i].is_none_or(|turn| turn < R_STOP + 1e-3),
                    "ray {i} reached the ring although it should have turned at {:?}",
                    turning[i]
                );
            }
            // The converse, for the rays that were sent inward: with no turning point above the
            // ring there is nothing to stop them, so they must be the ones that died.
            let ingoing = rays[i].dr_dt < 0.0;
            if ingoing && turning[i].is_none_or(|turn| turn <= R_STOP) {
                assert!(
                    !ray.alive(),
                    "ray {i} was sent inward with no turning point and must reach the ring: \
                     r = {}, turning {:?}",
                    ray.r,
                    turning[i]
                );
            }
        }
        let turners = turning.iter().filter(|t| t.is_some()).count();
        println!(
            "a pulse from r = {r0} inside r- = {rm:.4}, after 20 M: {died} of {} rays have reached \
             the ring and {survived} are alive, all of them within {worst_off_r_minus:.3e} M of r- \
             from below; {turners} rays have a turning point above the ring, the worst excursion \
             below one is {worst_below_turning:.3e} M, and L/E of the survivors is held to \
             {l_over_e_drift:.3e} relative over the whole run",
            rays.len()
        );
        assert!(died > 0 && survived > 0, "the run should show both fates: {died} and {survived}");
    }

    #[test]
    fn test_the_swallowed_front_sits_on_the_ring() {
        // The lower edge of a wedge, over the interval in which the pulse is being eaten.
        //
        // A pulse let go inside r- loses most of its rays to the ring, one at a time and in an
        // order fixed by the conserved L/E of each. Drawn as the minimum over the surviving
        // samples, the inner edge jumped outward to the next ray at every death and then dived
        // again: a sawtooth, entirely an artefact of sampling the cone at 2.5 degrees, and made
        // worse by a track stored only every 0.2 M, which joined several deaths with one long
        // diagonal chord. The continuum front does nothing of the kind - between a ray that has
        // already reached the ring and a ray that is still on its way there is a ray arriving
        // exactly now - so for the whole of that interval the inner edge belongs at R_STOP, and
        // `Pulse::radial_extent` puts it there.
        //
        // What is measured here is that interval: r_min is exactly R_STOP at every stored point
        // from the first death at the ring to the last, it never moves outward inside it, and the
        // track is dense enough to say so. Outside the interval the plain minimum over the live
        // rays stands, and the step at the end of it - the drawing running out of ring-bound
        // samples while the continuum still has some - is reported rather than smoothed.
        let metric = KerrSchild::new(1.0, 0.90);
        let r0 = 0.45;
        let dt = 0.017;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        let rays: Vec<NullRay> = (0..RAYS_PER_PULSE)
            .map(|i| {
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        // The closed-form fate of each ray, read off the emission event before anything is
        // integrated: this is the prediction the run then has to bear out.
        let bound_at_emission =
            rays.iter().filter(|ray| ray.ring_bound(&metric)).count();
        let mut pulse = Pulse {
            index: 0,
            emitted_t: 0.0,
            emitted_tau: 0.0,
            emitted_r: r0,
            emitted_phi: 0.0,
            rays,
            extent_track: vec![(0.0, r0, r0)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        };

        let mut t = 0.0;
        while t < 20.0 {
            t += dt;
            for ray in pulse.rays.iter_mut() {
                ray.step(&metric, dt);
            }
            pulse.extend_track(&metric, t);
        }

        // Every ray the closed form called ring-bound at emission did reach the ring, and every
        // ray that reached the ring did so at a recorded time.
        let ring_deaths: Vec<f64> = pulse
            .rays
            .iter()
            .filter(|ray| ray.died_at_ring())
            .map(|ray| ray.death_t.expect("a dead ray has a death time"))
            .collect();
        assert!(
            ring_deaths.len() >= bound_at_emission,
            "{bound_at_emission} rays were ring-bound at emission but only {} reached the ring",
            ring_deaths.len()
        );
        assert!(ring_deaths.len() > 10, "the pulse should be eaten: {} deaths", ring_deaths.len());
        assert!(
            pulse.rays.iter().any(|ray| ray.alive()),
            "and some of it should survive, or there is no edge left to draw"
        );
        assert!(
            !pulse.rays.iter().any(|ray| ray.death_end == Some(RayEnd::Unintegrable)),
            "no ray of this pulse may be retired anywhere but at a boundary"
        );
        let first = ring_deaths.iter().copied().fold(f64::INFINITY, f64::min);
        let last = ring_deaths.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        // Inside the swallowing interval: exactly R_STOP, at every stored point, with no step
        // outward anywhere in it.
        let mut inside = 0;
        let mut previous: Option<f64> = None;
        for &(t, r_min, _) in pulse.extent_track.iter() {
            if t < first || t > last {
                continue;
            }
            inside += 1;
            assert_eq!(
                r_min, R_STOP,
                "at t = {t}, inside the swallowing interval [{first}, {last}], the inner edge is \
                 at {r_min} rather than on the ring"
            );
            if let Some(before) = previous {
                assert!(
                    r_min <= before + 1e-15,
                    "the inner edge stepped outward from {before} to {r_min} at t = {t}"
                );
            }
            previous = Some(r_min);
        }
        assert!(
            inside > 20,
            "the track must resolve the interval it is being asserted over: {inside} points in \
             {:.3} M",
            last - first
        );

        // Before the first death the inner edge is the innermost live ray, and it only falls.
        let mut worst_rise_before = 0.0f64;
        let mut before_points = 0;
        for pair in pulse.extent_track.windows(2) {
            let ((t0, lo0, _), (t1, lo1, _)) = (pair[0], pair[1]);
            if t1 > first {
                break;
            }
            before_points += 1;
            worst_rise_before = worst_rise_before.max(lo1 - lo0);
            assert!(
                lo1 <= lo0 + 1e-12,
                "before the first ring death the inner edge only falls: {lo0} at t = {t0} to \
                 {lo1} at t = {t1}"
            );
        }

        // After the last one it steps out to the innermost survivor, which is the resolution limit
        // of a seventy-two ray cone rather than anything the geometry does. It is measured, not
        // hidden.
        let after: Vec<(f64, f64, f64)> = pulse
            .extent_track
            .iter()
            .copied()
            .filter(|(t, _, _)| *t > last)
            .collect();
        let step_out = after.first().map(|&(_, lo, _)| lo - R_STOP).unwrap_or(0.0);
        let survivor_floor = pulse
            .rays
            .iter()
            .filter(|ray| ray.alive())
            .map(|ray| ray.r)
            .fold(f64::INFINITY, f64::min);
        println!(
            "a pulse from r = {r0} at a = 0.90, 20 M at dt = {dt}: {} of {} rays are ring-bound at \
             emission and {} reach the ring, the first at t = {first:.4} and the last at \
             t = {last:.4}, an interval of {:.4} M carrying {inside} of the track's {} points, \
             every one of them with the inner edge exactly on R_STOP = {R_STOP}. Over the \
             {before_points} points before it the edge falls monotonically (worst rise \
             {worst_rise_before:.1e} M); at the end of it the edge steps out by {step_out:.4} M to \
             the innermost survivor, now at r = {survivor_floor:.4}",
            bound_at_emission,
            pulse.rays.len(),
            ring_deaths.len(),
            last - first,
            pulse.extent_track.len()
        );
        assert!(step_out > 0.0, "the step out of the ring is the thing being reported: {step_out}");
    }

    #[test]
    fn test_late_wedges_never_widen_again() {
        // The whole of a transmission, carried far past the end of the emitter's own worldline:
        // Alice released from r = 4.5 M at a = 0.90, her field stepped at the app's own frame of
        // 1/50 M out to t = 110 M. She reaches the ring in about 6 M, so every one of her pulses
        // spends more than a hundred M with nothing left of it but the arcs standing on r-.
        //
        // A wedge narrows as it is eaten and then stops changing, because what is left of the
        // pulse is frozen. It must never widen again, and the horizon-blind fate tests made it do
        // exactly that. A frozen ray's offset from r- decays as exp(-kappa_- t) and passes under
        // the spacing of f64 near r- at about t = 100 M; the ray rounds onto the surface, some
        // land an ulp below it with the sign of dr/dt flipped by the change of sign of Delta, and
        // the horizon-blind test then read the zeros of their potential out beyond r+ - which such
        // a ray can never reach - as a fate. A dead neighbour supplied the other half of
        // `Pulse::radial_extent`'s pin and the wedge snapped back to the full width of the drawn
        // field, in a band of them across the diagram at t ~ 100 M.
        //
        // What is asserted is the shape of the thing: for every pulse, the wedge at the last
        // sample of its track is no wider than it was at t = 60 M, by which time every pulse of
        // this run has long finished being swallowed.
        let metric = KerrSchild::new(1.0, 0.90);
        let field = run_field_to(&metric, 110.0, 0.02);
        let width_at = |track: &[(f64, f64, f64)], when: f64| {
            track.iter().rev().find(|(t, ..)| *t <= when).map(|&(t, lo, hi)| (t, hi - lo))
        };

        let mut checked = 0;
        let mut worst_widening = f64::NEG_INFINITY;
        let mut widest_late = 0.0f64;
        let mut widest_late_pulse = 0;
        let mut pinned_late = 0;
        for pulse in field.pulses.iter() {
            let Some((t60, w60)) = width_at(&pulse.extent_track, 60.0) else {
                continue;
            };
            let &(t_last, lo, hi) = pulse.extent_track.last().expect("a track has its seed");
            assert!(
                t_last > 100.0,
                "pulse {} stopped recording at t = {t_last}, before the band this test is about",
                pulse.index
            );
            checked += 1;
            let late = hi - lo;
            if late > widest_late {
                widest_late = late;
                widest_late_pulse = pulse.index;
            }
            pinned_late += usize::from(lo <= R_STOP);
            worst_widening = worst_widening.max(late - w60);
            assert!(
                late <= w60 + 1e-12,
                "pulse {} widened from {w60} at t = {t60:.2} to {late} at t = {t_last:.2} \
                 (r_min = {lo}, r_max = {hi})",
                pulse.index
            );
        }
        assert!(checked > 20, "the run has to carry a whole transmission: {checked} pulses");
        println!(
            "Alice's field at a = 0.90, stepped at 1/50 M to t = 110 M: {checked} pulses, none of \
             them widening after t = 60 M (worst change {worst_widening:.3e} M). The widest late \
             wedge is pulse {widest_late_pulse}'s at {widest_late:.3e} M, and {pinned_late} of the \
             tracks end with their inner edge pinned to R_STOP = {R_STOP}"
        );
    }

    #[test]
    fn test_a_long_run_thins_its_extent_track_instead_of_stopping() {
        // The cap on a track is a resolution limit, not a time limit. A pulse carried far past
        // `TRACK_MAX_POINTS` at the minimum spacing has to go on recording where its front is,
        // at half the resolution and then a quarter, rather than stop drawing part way up the
        // diagram; and the emission event, which is the track's seed and the apex of the wedge,
        // has to survive every thinning.
        let metric = KerrSchild::new(1.0, 0.90);
        let r0 = 4.5;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let rays: Vec<NullRay> = (0..RAYS_PER_PULSE)
            .map(|i| {
                let alpha = 2.0 * std::f64::consts::PI * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        let mut pulse = Pulse {
            index: 0,
            emitted_t: 0.0,
            emitted_tau: 0.0,
            emitted_r: r0,
            emitted_phi: 0.0,
            rays,
            extent_track: vec![(0.0, r0, r0)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        };
        // Two thinnings' worth of coordinate time, with the rays left standing: what is being
        // measured is the bookkeeping of the track, so the extent is fed to it by hand rather than
        // by integrating 250 M of light.
        // The clock is derived from the step count rather than accumulated, so that what is
        // measured is the thinning rather than the drift of 12500 additions: a caller whose own
        // step does not divide the spacing stores its point on the first call past it, which puts
        // up to one of the caller's steps into a gap and has nothing to do with the cap.
        let dt = TRACK_MIN_DT;
        for i in 1..=12_500 {
            pulse.extend_track(&metric, (i as f64) * dt);
        }
        let (last_t, ..) = *pulse.extent_track.last().unwrap();
        assert!(
            last_t > 249.0,
            "the track must still be recording at the end of the run: last point at t = {last_t}"
        );
        assert!(pulse.extent_track.len() <= TRACK_MAX_POINTS);
        assert_eq!(pulse.extent_track[0].0, 0.0, "the emission event is never thinned away");
        assert!((pulse.track_dt / TRACK_MIN_DT - 4.0).abs() < 1e-9, "{}", pulse.track_dt);
        let mut worst_gap = 0.0f64;
        for pair in pulse.extent_track.windows(2) {
            worst_gap = worst_gap.max(pair[1].0 - pair[0].0);
        }
        println!(
            "250 M at {TRACK_MIN_DT} M: {} points spaced {:.3} M apart at most, the cap being \
             {TRACK_MAX_POINTS}",
            pulse.extent_track.len(),
            format!("{worst_gap:.3}")
        );
        assert!(worst_gap <= pulse.track_dt + 1e-9, "spacing {worst_gap} against {}", pulse.track_dt);
    }

    #[test]
    fn test_frozen_family_is_the_prograde_arc_and_narrows_inward() {
        // Which part of a pulse freezes, read straight off the emission event rather than off a
        // long integration: E - Omega_- L is conserved, so the family a ray belongs to is already
        // settled the instant Alice lets it go. Four raindrop emissions between r+ and the ring
        // give the shape of the answer. The frozen arc is always the *prograde* half of the cone,
        // the rays dragged forward in phi, never the outward radial leg at alpha = 0 and never the
        // retrograde half; and it shrinks as the emitter falls, because a pulse sent close to r-
        // has almost no room left in which frame dragging can beat aberration.
        //
        // Measured, out of the 144 rays of a pulse: 53 frozen at r = 1.7 (just inside r+ = 1.76,
        // alpha from 30 to 155 degrees), 42 at r = 1.0 (40 to 140), 23 at r = 0.5 (60 to 115) and 8
        // at r = 0.3 (75 to 90), against r- = 0.240. So the arc is a little over a third of the
        // ring high in Region II and a bare sliver by the time Alice is nearly on the Cauchy
        // horizon.
        let metric = KerrSchild::new(1.0, 0.65);
        let rm = metric.inner_horizon();
        let omega_minus = metric.a / (rm * rm + metric.a * metric.a);
        let params = WorldlineParams::default();
        let two_pi = 2.0 * std::f64::consts::PI;
        let mut counts: Vec<(f64, usize)> = Vec::new();
        for &r0 in &[1.7f64, 1.0, 0.5, 0.3] {
            let alice = Observer::new_with_phi(&metric, "Alice", 0.0, r0, 0.0, 0.0, params);
            let mut field = SignalField::default();
            field.emit_if_due(&metric, &alice);
            let pulse = field.pulses.first().expect("a released Alice emits at once");
            assert_eq!(pulse.rays.len(), RAYS_PER_PULSE);

            let mut frozen = 0usize;
            for (i, ray) in pulse.rays.iter().enumerate() {
                if !ray.frozen(&metric) {
                    continue;
                }
                frozen += 1;
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                assert!(
                    alpha > 0.0 && alpha < std::f64::consts::PI,
                    "r0={r0}: a frozen ray must be prograde, not alpha={alpha}"
                );
                // The same criterion in the one scale-free constant the ray carries. E and L per
                // unit k^t are not separately conserved along a ray; their quotient L/E is, so
                // this is the form of the statement that survives the fall. Frozen means
                // E - Omega_- L < 0, which divided by E reads L/E > 1/Omega_- where E is positive
                // and L/E < 1/Omega_- where E is negative. Both branches are populated: inside the
                // ergoregion the leading edge of the arc has negative energy per unit k^t, which is
                // why the plain statement "L/E > 0" is not the criterion (measured: at r = 1 the
                // frozen ray at alpha = 50 degrees has E = -0.045, L = +0.107, L/E = -2.39).
                let g = metric.metric_components(ray.r);
                let v = ray.direction();
                let e_over_kt = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
                let l_over_e = ray.l_over_e(&metric);
                if e_over_kt > 0.0 {
                    assert!(
                        l_over_e > 1.0 / omega_minus,
                        "r0={r0}: a frozen ray of positive energy co-rotates past the generator:                          L/E = {l_over_e} against 1/Omega_- = {} at alpha={alpha}",
                        1.0 / omega_minus
                    );
                } else {
                    assert!(
                        l_over_e < 1.0 / omega_minus,
                        "r0={r0}: a frozen ray of negative energy sits below the generator:                          L/E = {l_over_e} against 1/Omega_- = {} at alpha={alpha}",
                        1.0 / omega_minus
                    );
                }
            }
            counts.push((r0, frozen));
        }

        println!("frozen rays of {RAYS_PER_PULSE} per pulse: {counts:?}");
        for w in counts.windows(2) {
            assert!(
                w[1].1 < w[0].1,
                "the frozen arc must narrow inward: {:?} then {:?}",
                w[0],
                w[1]
            );
        }
        assert!(
            counts[0].1 >= 20,
            "just inside r+ the frozen arc is about a third of the ring: {counts:?}"
        );
        assert!(
            counts[counts.len() - 1].1 <= 8,
            "close to r- it is only a sliver: {counts:?}"
        );
    }

    /// The largest component-wise gap between two ray states, in the four numbers the integration
    /// actually carries.
    fn state_gap(a: &NullRay, b: &NullRay) -> f64 {
        [
            (a.r - b.r).abs(),
            (a.phi - b.phi).abs(),
            (a.dr_dt - b.dr_dt).abs(),
            (a.dphi_dt - b.dphi_dt).abs(),
        ]
        .into_iter()
        .fold(0.0f64, f64::max)
    }

    #[test]
    fn test_ray_step_back_retraces_the_forward_path() {
        // `ray_rhs` is a first-order autonomous system and it is time symmetric, so the scheme
        // with a negative step integrates the same null geodesic in the other direction rather
        // than some other curve. What is left over is the scheme's own asymmetry: each substep is
        // sized by the state it starts at and by the error estimated for it, and running the path
        // the other way starts each substep at the far end of the one the forward pass took, so
        // the two directions do not cut the interval in the same places.
        //
        // Measured over a whole light cone at r = 3 and again in Region II at r = 1, the eight
        // directions come back to between 1e-16 and 8.2e-8 of where they started. The large end of
        // that belongs to the rays that run into the stiff last decade of radius above the ring,
        // most of them reaching it and being revived on the way out: since the ring death is a
        // rejected substep rather than a clamped equation, such a ray stops within a hair of R_STOP
        // and its retrace starts from the stiffest point of its path. 1e-6 covers the worst of
        // them (8.2e-8, at alpha = 270 degrees from r = 3) with a decade to spare. Under the fixed
        // subdivision this replaces the same measurement was 8.4e-6, and the deaths were a whole
        // substep short of the ring.
        let metric = KerrSchild::new(1.0, 0.65);
        let two_pi = 2.0 * std::f64::consts::PI;
        for &(r0, span) in &[(3.0f64, 4.0f64), (1.0, 1.5)] {
            let u = raindrop(&metric, r0);
            let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
            let mut worst = 0.0f64;
            for i in 0..8 {
                let alpha = two_pi * (i as f64) / 8.0;
                let start = NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, alpha, &u);
                let mut ray = start;
                let n = (span / 0.02).round() as usize;
                for _ in 0..n {
                    ray.step(&metric, 0.02);
                }
                for _ in 0..n {
                    ray.step_back(&metric, 0.02);
                }
                assert!(ray.alive(), "the retrace must bring the ray back to life: {ray:?}");
                assert!(
                    (ray.t - start.t).abs() < 1e-12,
                    "and back to the time it started at: t = {}",
                    ray.t
                );
                let err = state_gap(&ray, &start);
                worst = worst.max(err);
                println!("r0={r0} alpha={alpha}: retrace error {err:e}");
                assert!(
                    err < 1e-6,
                    "r0={r0} alpha={alpha}: retrace error {err}, {ray:?} vs {start:?}"
                );
            }
            println!("r0 = {r0}, span = {span} M: worst retrace error {worst:e}");
        }
    }

    #[test]
    fn test_dead_rays_revive_on_step_back() {
        // Death is a state, not a deletion. A ray aimed straight at the ring from r = 0.5 reaches
        // R_STOP well inside 1 M and stops there with its full state kept; two M later it is still
        // exactly where it stopped, and stepping the two M back walks it away from the boundary and
        // all the way home along the geodesic it came in on. Measured error on the round trip:
        // 4.7e-10 in the worst of the four carried numbers, against 3.3e-7 under the fixed
        // subdivision this replaces.
        let metric = KerrSchild::new(1.0, 0.65);
        let r0 = 0.5;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
        let start =
            NullRay::from_local_direction(&metric, 0.0, r0, 0.0, &tetrad, std::f64::consts::PI, &u);

        let mut ray = start;
        for _ in 0..100 {
            ray.step(&metric, 0.02);
        }
        let death_t = ray.death_t.expect("the inward ray must reach the ring");
        assert!(death_t < 1.0, "and reach it inside 1 M: death_t = {death_t}");
        // The state kept is the last event the ray reached with the true equation: a substep is
        // rejected and halved if any of its stages would fall below R_STOP, so the ray stops just
        // above the ring rather than just inside it, and much closer to it than the old clamped
        // scheme's one-substep-short death. Measured: 1.4e-9 M above R_STOP.
        println!("the ray stopped {:.3e} M above R_STOP = {R_STOP}", ray.r - R_STOP);
        assert!(
            (R_STOP..R_STOP + MAX_DR_PER_SUBSTEP).contains(&ray.r),
            "with its state kept at the ring: r = {}",
            ray.r
        );
        assert!(
            ray.r - R_STOP < 1e-6,
            "and kept there, not a substep short of it: r - R_STOP = {}",
            ray.r - R_STOP
        );
        assert!(
            (ray.t - 2.0).abs() < 1e-12,
            "while its clock runs on with the field: t = {}",
            ray.t
        );

        for _ in 0..100 {
            ray.step_back(&metric, 0.02);
        }
        assert!(ray.alive(), "stepping back past the death event must revive the ray");
        let err = state_gap(&ray, &start);
        println!("revived ray: worst component error {err:e}");
        assert!(err < 1e-8, "revival error {err}: {ray:?} vs {start:?}");
        assert!(
            (ray.r - r0).abs() < 1e-8,
            "and back to the radius it started at: r = {} vs {r0}",
            ray.r
        );
    }

    /// The transmission of `run_transmission` on a plain fixed step, run to a given coordinate
    /// time. A fixed step is what makes two runs of different length share one history exactly, so
    /// a field run to 12 and wound back to 9 can be compared with a field run to 9.
    fn run_field_to(metric: &KerrSchild, until: f64, dt: f64) -> SignalField {
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut bob = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, 8.0, 0.0, params);
        let mut field = SignalField::default();
        let steps = (until / dt).round() as usize;
        for i in 0..steps {
            let t = ((i + 1) as f64) * dt;
            alice.step(metric, t, dt);
            bob.step(metric, t, dt);
            field.advance(metric, dt);
            field.emit_if_due(metric, &alice);
            field.detect_receptions(metric, &bob);
        }
        field
    }

    /// One RK4 step of the steepest ingoing null curve, dr/dt = `KerrSchild::null_wedge`'s inner
    /// edge, which is the exact lower envelope of every null ray's radius: no ray can be more
    /// ingoing than the cone allows at the radius it is passing through.
    fn ingoing_edge_advance(metric: &KerrSchild, r: f64, dt: f64) -> f64 {
        let slope = |r: f64| metric.null_wedge(r.max(R_STOP)).dr_dt_in;
        let n = 64;
        let h = dt / n as f64;
        let mut r = r;
        for _ in 0..n {
            let k1 = slope(r);
            let k2 = slope(r + 0.5 * h * k1);
            let k3 = slope(r + 0.5 * h * k2);
            let k4 = slope(r + h * k3);
            r += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
        r
    }

    #[test]
    fn test_extent_track_lower_edge_hugs_the_steepest_ingoing_ray() {
        // What the (t, r) diagram draws for a pulse is the interval [min r, max r] over its live
        // rays, and the lower edge of that interval is bounded exactly, by the steepest ingoing
        // null curve through the emission event: dr/dt = `null_wedge(r).dr_dt_in`, the inner edge
        // of the local cone. That edge is *not* the 45-degree ingoing principal ray whenever the
        // hole spins - the null discriminant at dr/dt = -1 is exactly a^2, so -1 sits strictly
        // inside the cone and the edge is steeper - and this test measures both gaps, to the exact
        // envelope and to the 45-degree line, so the difference is on the record rather than
        // assumed.
        //
        // The bound is not attained, for two reasons. The cone is sampled at 2.5 degrees, so the
        // most ingoing of the `RAYS_PER_PULSE` rays sits up to 1.25 degrees off the extremal
        // direction. And once that ray reaches the ring it leaves the extent, after which the
        // minimum is taken over rays that are still falling and the gap opens for good; the close
        // tracking is asserted while the innermost ray is still outside r+, where nothing has died
        // and the discretisation is the only error.
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let field = run_field_to(&metric, 8.0, 0.01);
        assert!(field.pulses.len() > 10, "expected a transmission in flight: {}", field.pulses.len());

        let mut points = 0;
        let mut outside = 0;
        let mut worst_gap = 0.0f64;
        let mut worst_outside = 0.0f64;
        let mut worst_45_outside = 0.0f64;
        let mut below_45 = 0;
        for pulse in field.pulses.iter() {
            let seed = pulse.extent_track[0];
            assert_eq!(
                (seed.0, seed.1, seed.2),
                (pulse.emitted_t, pulse.emitted_r, pulse.emitted_r),
                "pulse {} does not start at its own emission event",
                pulse.index
            );
            let mut envelope = pulse.emitted_r;
            let mut envelope_t = pulse.emitted_t;
            for &(t, r_min, r_max) in pulse.extent_track.iter() {
                assert!(r_min <= r_max, "pulse {}: extent {r_min} > {r_max} at t = {t}", pulse.index);
                if t > envelope_t {
                    envelope = ingoing_edge_advance(&metric, envelope, t - envelope_t);
                    envelope_t = t;
                }
                let gap = r_min - envelope;
                assert!(
                    gap > -1e-6,
                    "pulse {}: r_min = {r_min} is below the steepest ingoing null curve at \
                     {envelope}, t = {t}",
                    pulse.index
                );
                points += 1;
                worst_gap = worst_gap.max(gap);
                if r_min - (pulse.emitted_r - (t - pulse.emitted_t)) < 0.0 {
                    below_45 += 1;
                }
                if r_min > rp {
                    outside += 1;
                    worst_outside = worst_outside.max(gap);
                    worst_45_outside = worst_45_outside
                        .max((r_min - (pulse.emitted_r - (t - pulse.emitted_t))).abs());
                }
            }
        }
        println!(
            "extent tracks: {points} points, {outside} with r_min outside r+; worst gap to the \
             exact ingoing envelope {worst_gap:.3e} (outside r+ {worst_outside:.3e}); {below_45} \
             points lie below the 45-degree line, worst |offset| from it outside r+ \
             {worst_45_outside:.3e}"
        );
        assert!(outside > 100, "the measurement needs a decent sample: {outside} points");
        assert!(
            worst_outside < 2e-3,
            "outside r+ the innermost ray should hug the cone's ingoing edge: gap {worst_outside}"
        );
        assert!(
            worst_outside > 1e-5,
            "and it is a 2.5-degree sampling of the cone, not the edge itself: gap {worst_outside}"
        );
        // The 45-degree line is not a bound at a = 0.65: the ingoing edge of the cone is steeper
        // than -1 wherever the hole spins, so the drawn lower edge dips below it.
        assert!(
            below_45 > 0,
            "at a = 0.65 the steepest ingoing ray beats dr/dt = -1, so some point must be below it"
        );
    }

    #[test]
    fn test_a_reception_is_stamped_with_the_interpolated_crossing_event() {
        // An arrival is stamped with the *crossing*, not with the pass that noticed it. A sheet is
        // seen to have swept over the receiver when the sign of receiver.r - r_front differs
        // between two consecutive passes, which puts the crossing somewhere inside that interval;
        // every number of the record - t, r, phi, tau and the shift - is the linear interpolation
        // of the two passes at the fraction where the side value vanishes.
        //
        // Two things are asserted about that. The event lies inside the interval, on the receiver's
        // own segment of worldline between the two passes, which is what lets the equatorial view
        // put a tick on their trail rather than near it. And all five numbers are interpolated at
        // the *same* fraction, which is what makes the record one event rather than five: the
        // fraction recovered from t and the fraction recovered from r agree to round-off.
        let metric = KerrSchild::new(1.0, 0.9);
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 8.0, 0.0, params);
        let mut field = SignalField::default();

        let dt = 0.01;
        let mut t = 0.0;
        let mut checked = 0;
        let mut worst_offset = 0.0f64;
        let mut worst_fraction_gap = 0.0f64;
        while t < 16.0 {
            let before_event = (bob.t, bob.r, bob.phi, bob.tau);
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &alice);
            let before = field.received_count();
            field.detect_receptions(&metric, &bob);
            for reception in field.receptions().skip(before) {
                let (t0, r0, phi0, tau0) = before_event;
                assert!(
                    reception.t > t0 - 1e-12 && reception.t <= bob.t + 1e-12,
                    "an arrival at t = {} is outside the pass interval ({t0}, {}]",
                    reception.t,
                    bob.t
                );
                for (value, a, b, what) in [
                    (reception.r, r0, bob.r, "r"),
                    (reception.phi, phi0, bob.phi, "phi"),
                    (reception.tau_receiver, tau0, bob.tau, "tau"),
                ] {
                    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                    assert!(
                        value >= lo - 1e-12 && value <= hi + 1e-12,
                        "the arrival's {what} = {value} is off the receiver's worldline segment \
                         [{lo}, {hi}]"
                    );
                }
                // One event, five numbers: the same fraction of the interval in each of them.
                let fraction_t = (reception.t - t0) / (bob.t - t0);
                if (bob.r - r0).abs() > 1e-9 {
                    let fraction_r = (reception.r - r0) / (bob.r - r0);
                    worst_fraction_gap = worst_fraction_gap.max((fraction_t - fraction_r).abs());
                    assert!(
                        (fraction_t - fraction_r).abs() < 1e-9,
                        "t and r were interpolated at different fractions: {fraction_t} vs \
                         {fraction_r}"
                    );
                }
                worst_offset = worst_offset.max(bob.t - reception.t);
                checked += 1;
            }
        }
        assert!(checked > 5, "the run should record several arrivals: {checked}");
        println!(
            "{checked} arrivals stamped at the crossing: the furthest was {worst_offset:.3e} M of \
             coordinate time before the pass that found it (the pass interval is {dt}), and the \
             worst disagreement between the fraction read off t and the one read off r was \
             {worst_fraction_gap:.3e}"
        );
        assert!(
            worst_offset > 1e-6,
            "the stamp must be an interpolation, not the pass event: worst offset {worst_offset}"
        );

        // The frozen family is received where it waits, on r-, so those ticks land on the r-
        // circle of the equatorial view rather than being spread along the fall.
        let rm = metric.inner_horizon();
        let frozen: Vec<&Reception> = field.receptions().filter(|rec| rec.frozen_family).collect();
        let worst = frozen.iter().map(|rec| rec.r - rm).fold(0.0f64, f64::max);
        println!(
            "{checked} arrivals on the receiver's worldline, {} of them from the frozen \
             family, the furthest out {worst:.3e} M above r- = {rm:.4}",
            frozen.len()
        );
        assert!(!frozen.is_empty(), "he should have met the frozen family by then");

        // Where those ticks land is the other half of the drawing claim. A frozen-family ray can
        // be met anywhere on its way in - the family is defined by E - Omega_- L < 0, not by where
        // the ray currently stands - so a fixed step like the one above catches some of them high
        // up and then jumps Bob across the last thousandths of an M in one go. Resolving the stack
        // itself needs the refining step of `run_transmission`, and with it the arrivals bunch
        // into the last tenth of an M above r-, a few thousandths of an M apart: at the default
        // zoom of the equatorial view that is a handful of pixels, so they draw as a row of
        // overlapping triangles sitting on the r- circle.
        let (heard, _) = run_transmission(&metric, 0.25, 8.0);
        let stack: Vec<&Reception> = heard.iter().filter(|rec| rec.r - rm < 0.1).collect();
        let closest = stack.iter().map(|rec| rec.r - rm).fold(f64::INFINITY, f64::min);
        println!(
            "with the refining step: {} arrivals, {} of them in the last 0.1 M above r- (the \
             nearest {closest:.3e} M above it), of which {} are frozen-family",
            heard.len(),
            stack.len(),
            stack.iter().filter(|rec| rec.frozen_family).count()
        );
        assert!(
            stack.len() >= 5,
            "the last tenth of an M above r- should carry a run of arrivals: {} of {}",
            stack.len(),
            heard.len()
        );
        assert!(
            closest < 0.02,
            "and they should reach the horizon: the nearest is {closest} M above it"
        );
    }

    #[test]
    fn test_a_sheet_handed_over_and_taken_back_records_both_of_its_crossings() {
        // A sheet is a pair of rays, and the pair straddling the receiver's azimuth changes from
        // pass to pass as the front deforms: a sheet drops out of the tracking and comes back, and
        // while it is away the receiver can pass through the front by one of its neighbours. This
        // is that handoff, built by hand so that it is unambiguous.
        //
        // Six rays of one loop, at two radii. Rays 0, 1 and 2 are the near part of the front, at
        // the radius the receiver is about to be swept by; rays 3, 4 and 5 are the far part, parked
        // at r = 30. The loop is closed and every ray of it is alive, so its rays' azimuths - which
        // are integrated continuously and differenced raw - telescope to zero round the loop: the
        // front has no net winding, and it therefore crosses the receiver's azimuth an even number
        // of times. Here it crosses twice, once on the near part and once on the far part coming
        // back, and only the near one is ever close enough to the receiver to be an arrival. The
        // far sheet stands outside r = 6 throughout and never crosses them.
        //
        //   1. segment 1 carries the near sheet; the front sweeps outward past the receiver (one)
        //   2. the loop rotates by a quarter of a radian and segment 0 carries it instead;
        //      the front sweeps back inward past the receiver                              (two)
        //   3. the loop rotates back, segment 1 carries it again, and the front sweeps
        //      outward past the receiver a second time                                   (three)
        //
        // All three are real arrivals. The third is the one a rule keyed to the last crossing
        // recorded on the same sheet gets wrong: it leaves the receiver on the same side as
        // crossing one did, because crossing two - which put them back - was recorded on a
        // different segment. An earlier version of `Pulse::scan` suppressed exactly this, and was
        // measured doing so: with that rule in place this test records two arrivals, not three.
        let metric = KerrSchild::new(1.0, 0.90);
        let params = WorldlineParams::default();
        let r_receiver = 3.0;
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, r_receiver, 0.0, 0.0, params);
        let u_receiver = signalling_four_velocity(&metric, &bob);

        // The loop: six rays of a real emission at r = 3, whose azimuths and radii are then set by
        // hand.
        let u = raindrop(&metric, r_receiver);
        let tetrad = Tetrad::from_four_velocity(&metric, r_receiver, &u);
        let rays: Vec<NullRay> = (0..6)
            .map(|i| {
                let alpha = 2.0 * std::f64::consts::PI * (i as f64) / 6.0;
                NullRay::from_local_direction(&metric, 0.0, r_receiver, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        let mut pulse = Pulse {
            index: 0,
            emitted_t: 0.0,
            emitted_tau: 0.0,
            emitted_r: r_receiver,
            emitted_phi: 0.0,
            rays,
            extent_track: vec![(0.0, r_receiver, r_receiver)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        };

        // One detection pass with the loop put where the caller says: the near rays at `r_front`,
        // the far ones at r = 30, and the whole loop rotated by `turn`.
        let base = [-0.5f64, -0.1, 0.3, 1.5, 3.0, 1.5];
        let pass = |pulse: &mut Pulse, bob: &mut Observer, t: f64, r_front: f64, turn: f64| {
            for (k, ray) in pulse.rays.iter_mut().enumerate() {
                ray.t = t;
                ray.r = if k < 3 { r_front } else { 30.0 };
                ray.phi = base[k] + turn;
            }
            bob.t = t;
            bob.tau = 0.8 * t;
            pulse.scan(&metric, bob, &u_receiver, true);
            pulse.sheets.iter().map(|sheet| sheet.segment).collect::<Vec<_>>()
        };

        // 1. Segment 1 carries the near sheet: the front starts inside the receiver and sweeps out
        //    past them. Segment 5, the closing one, carries the far sheet the whole way through.
        assert_eq!(
            pass(&mut pulse, &mut bob, 0.1, 2.9, 0.0),
            vec![1, 5],
            "the near sheet on segment 1, the far one on the closing segment"
        );
        assert!(pulse.receptions.is_empty(), "the first pass only establishes the sides");
        assert_eq!(pass(&mut pulse, &mut bob, 0.2, 3.1, 0.0), vec![1, 5]);
        assert_eq!(pulse.receptions.len(), 1, "the front swept out past the receiver");

        // 2. The loop rotates: segment 0 takes over, and the receiver passes back through the
        //    front by that segment instead.
        assert_eq!(
            pass(&mut pulse, &mut bob, 0.3, 3.1, 0.25),
            vec![0, 5],
            "segment 0 should carry the near sheet now"
        );
        assert_eq!(pulse.receptions.len(), 1, "the handoff itself is not a crossing");
        assert_eq!(pass(&mut pulse, &mut bob, 0.4, 2.9, 0.25), vec![0, 5]);
        assert_eq!(pulse.receptions.len(), 2, "the front swept back in past the receiver");

        // 3. The loop rotates back and segment 1 sweeps out past the receiver a second time.
        assert_eq!(pass(&mut pulse, &mut bob, 0.5, 2.9, 0.0), vec![1, 5], "segment 1 again");
        assert_eq!(pulse.receptions.len(), 2, "coming back is not a crossing either");
        assert_eq!(pass(&mut pulse, &mut bob, 0.6, 3.1, 0.0), vec![1, 5]);
        assert_eq!(
            pulse.receptions.len(),
            3,
            "the second crossing of segment 1 is a real arrival and must be recorded: {:?}",
            pulse.receptions
        );

        let events: Vec<(usize, f64)> =
            pulse.receptions.iter().map(|rec| (rec.segment, rec.t)).collect();
        println!("handoff: three crossings, (segment, t) = {events:?}");
        assert_eq!(events[0].0, 1);
        assert_eq!(events[1].0, 0);
        assert_eq!(events[2].0, 1);
        // Each is stamped inside the pass interval that found it, and the two crossings of segment
        // 1 leave the receiver on the same side, which is the whole point.
        for (rec, (lo, hi)) in pulse.receptions.iter().zip([(0.1, 0.2), (0.3, 0.4), (0.5, 0.6)]) {
            assert!(rec.t > lo && rec.t < hi, "{rec:?} is outside ({lo}, {hi})");
            assert!(rec.ratio.is_finite() && rec.ratio > 0.0, "{rec:?}");
        }
        assert!(
            pulse.receptions[0].side_after * pulse.receptions[2].side_after > 0.0,
            "the two crossings of segment 1 leave the receiver on the same side"
        );
    }

    #[test]
    fn test_a_two_segment_handover_still_records_the_crossing() {
        // The handover that the segment key lost. A sheet is tracked from pass to pass by where it
        // sits along the ray loop, so the segment carrying it may move - and here it moves by two
        // indices in the very step in which the front sweeps over the receiver, which is the case
        // that used to be dropped: the new key had no remembered side, so nothing compared the two
        // sides and no arrival was recorded.
        //
        // Twelve rays. Six of them are the near part of the front, evenly spaced 0.2 rad apart at
        // the radius the receiver is about to be swept by; six are the far part, out at r = 30.
        // One step both rotates the whole loop by two of those spacings and carries the near part
        // from inside the receiver's radius to outside it. The far part carries the second sheet
        // that a closed loop of live rays must have - the front crosses the receiver's azimuth on
        // the way out and again on the way back - and it stands outside r = 4 throughout.
        let metric = KerrSchild::new(1.0, 0.90);
        let params = WorldlineParams::default();
        let r_receiver = 3.0;
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, r_receiver, 0.0, 0.0, params);
        let u_receiver = signalling_four_velocity(&metric, &bob);
        let u = raindrop(&metric, r_receiver);
        let tetrad = Tetrad::from_four_velocity(&metric, r_receiver, &u);
        let rays: Vec<NullRay> = (0..12)
            .map(|i| {
                let alpha = 2.0 * std::f64::consts::PI * (i as f64) / 12.0;
                NullRay::from_local_direction(&metric, 0.0, r_receiver, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        let mut pulse = Pulse {
            index: 0,
            emitted_t: 0.0,
            emitted_tau: 0.0,
            emitted_r: r_receiver,
            emitted_phi: 0.0,
            rays,
            extent_track: vec![(0.0, r_receiver, r_receiver)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        };

        let step = 0.2;
        // The near arc runs from -0.5 to +0.5 in steps of `step`, so before any rotation the
        // receiver's azimuth falls halfway along segment 2; the far arc goes out to 4.5 rad and
        // comes back, and the closing segment carries it back across the receiver's azimuth.
        let base = [-0.5f64, -0.3, -0.1, 0.1, 0.3, 0.5, 1.5, 3.0, 4.5, 4.5, 3.0, 1.5];
        let pass = |pulse: &mut Pulse, bob: &mut Observer, t: f64, r_front: f64, turn: f64| {
            for (k, ray) in pulse.rays.iter_mut().enumerate() {
                ray.t = t;
                ray.r = if k < 6 { r_front } else { 30.0 };
                ray.phi = base[k] + turn;
            }
            bob.t = t;
            bob.tau = 0.8 * t;
            pulse.scan(&metric, bob, &u_receiver, true);
            pulse.sheets.iter().map(|sheet| (sheet.segment, sheet.loop_s)).collect::<Vec<_>>()
        };

        let before = pass(&mut pulse, &mut bob, 0.1, 2.9, 0.0);
        assert_eq!(before.len(), 2, "the near sheet and the far one: {before:?}");
        assert_eq!(before[0].0, 2, "the near sheet is on segment 2: {before:?}");
        assert_eq!(before[1].0, 11, "and the far one on the closing segment: {before:?}");
        assert!(pulse.receptions.is_empty(), "the first pass only establishes the sides");

        let after = pass(&mut pulse, &mut bob, 0.2, 3.1, 2.0 * step);
        assert_eq!(after.len(), 2, "still two sheets: {after:?}");
        assert_eq!(after[0].0, 0, "carried now by the segment two round the loop: {after:?}");
        assert_eq!(after[1].0, 11, "the far sheet has not moved segment: {after:?}");
        assert_eq!(
            pulse.receptions.len(),
            1,
            "the near part of the front swept out past the receiver and that is an arrival, \
             whichever segment was carrying it; the far sheet never came near them: {:?}",
            pulse.receptions
        );
        assert_eq!(pulse.receptions[0].segment, 0, "recorded on the segment that now carries it");
        let moved = (after[0].1 - before[0].1).rem_euclid(12.0);
        println!(
            "a sheet handed from segment {} to segment {} in one pass - two indices, a loop \
             coordinate of {:.3} against {:.3} - still records its crossing at t = {:.4}",
            before[0].0,
            after[0].0,
            after[0].1,
            before[0].1,
            pulse.receptions[0].t
        );
        assert!(
            (moved.min(12.0 - moved) - 2.0).abs() < 1e-9,
            "the sheet must have moved two segments along the loop: {moved}"
        );
        assert!(pulse.receptions[0].t > 0.1 && pulse.receptions[0].t < 0.2);
    }

    #[test]
    fn test_a_wound_front_is_detected_on_the_raw_azimuth_difference() {
        // A front that has wound, and a receiver the fold cannot see it reach.
        //
        // Four rays whose integrated azimuths are 0.3, 2 pi + 0.6, 4 pi + 0.9 and 6 pi + 1.2: each
        // neighbour has lapped the one before it once round the hole, which is what a few tens of
        // M near a circular photon orbit does to a real pulse. Those raw differences are the
        // physical winding between the pairs, so the polyline through them spans three whole turns
        // and crosses the receiver's azimuth - the family 0, +/-2 pi, +/-4 pi, ... on the unwrapped
        // axis - six times, three on the way out and three on the way back.
        //
        // Folded into [-pi, pi] the same four rays read as 0.3, 0.6, 0.9, 1.2: a loop that sits
        // entirely between the receiver's azimuth and half a turn past it, crossing nothing. The
        // fold does not put this front on the wrong side of the hole so much as delete it - the
        // receiver is never in front of it at all, and no arrival is ever recorded.
        let metric = KerrSchild::new(1.0, 0.90);
        let params = WorldlineParams::default();
        let r_receiver = 6.0;
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, r_receiver, 0.0, 0.0, params);
        let u_receiver = signalling_four_velocity(&metric, &bob);
        let two_pi = 2.0 * std::f64::consts::PI;
        let u = raindrop(&metric, r_receiver);
        let tetrad = Tetrad::from_four_velocity(&metric, r_receiver, &u);
        let rays: Vec<NullRay> = (0..4)
            .map(|i| {
                let alpha = 0.5 * std::f64::consts::PI * (i as f64);
                NullRay::from_local_direction(&metric, 0.0, r_receiver, 0.0, &tetrad, alpha, &u)
            })
            .collect();
        let mut pulse = Pulse {
            index: 0,
            emitted_t: 0.0,
            emitted_tau: 0.0,
            emitted_r: r_receiver,
            emitted_phi: 0.0,
            rays,
            extent_track: vec![(0.0, r_receiver, r_receiver)],
            track_dt: TRACK_MIN_DT,
            sheets: Vec::new(),
            receptions: Vec::new(),
        };

        // Ray 0 is the inner end of the front and the other three are 4 M outside it, so of the six
        // sheets the innermost is the one the closing segment carries down to ray 0. That is the
        // one the receiver at r = 6 meets, as the whole front is moved out from r_front = 4.5 to
        // r_front = 5.0 between the two passes. The azimuths are the same on both passes, so the
        // fold sees the same thing on either.
        let base = [0.3, two_pi + 0.6, 2.0 * two_pi + 0.9, 3.0 * two_pi + 1.2];
        let pass = |pulse: &mut Pulse, bob: &mut Observer, t: f64, r_front: f64| {
            for (k, ray) in pulse.rays.iter_mut().enumerate() {
                ray.t = t;
                ray.r = if k == 0 { r_front } else { r_front + 4.0 };
                ray.phi = base[k];
            }
            bob.t = t;
            bob.tau = 0.8 * t;
            pulse.scan(&metric, bob, &u_receiver, true);
            pulse.sheets.iter().map(|sheet| (sheet.segment, sheet.side)).collect::<Vec<_>>()
        };

        // What the fold would have made of the same four rays, on both passes: the polyline built
        // the way `Pulse::scan` used to build it, and the multiples of 2 pi it straddles.
        let folded_sheets = |pulse: &Pulse, receiver_phi: f64| {
            let wrap = |d: f64| d - two_pi * (d / two_pi).round();
            let n = pulse.rays.len();
            let mut rel = vec![wrap(pulse.rays[0].phi - receiver_phi)];
            for i in 1..n {
                rel.push(rel[i - 1] + wrap(pulse.rays[i].phi - pulse.rays[i - 1].phi));
            }
            let closing = rel[n - 1] + wrap(pulse.rays[0].phi - pulse.rays[n - 1].phi);
            let mut count = 0;
            for i in 0..n {
                let (a, b) = (rel[i], if i + 1 < n { rel[i + 1] } else { closing });
                let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                count += ((hi / two_pi).floor() as i64 - (lo / two_pi).ceil() as i64 + 1).max(0);
            }
            (rel, count)
        };

        let first = pass(&mut pulse, &mut bob, 0.1, 4.5);
        let (folded_rel, folded_count) = folded_sheets(&pulse, bob.phi);
        assert_eq!(
            first.len(),
            6,
            "three turns out and three back: six sheets stand across the receiver's azimuth, \
             {first:?}"
        );
        assert!(pulse.receptions.is_empty(), "the first pass only establishes the sides");

        let second = pass(&mut pulse, &mut bob, 0.2, 5.0);
        assert_eq!(second.len(), 6, "the same six: {second:?}");
        assert_eq!(
            pulse.receptions.len(),
            1,
            "the innermost sheet swept out past the receiver and that is an arrival: {:?}",
            pulse.receptions
        );
        let rec = pulse.receptions[0];
        assert!(rec.t > 0.1 && rec.t < 0.2, "{rec:?}");
        assert!(rec.ratio.is_finite() && rec.ratio > 0.0, "{rec:?}");
        println!(
            "a front wound three turns: {} sheets on the raw polyline, one of them crossing the \
             receiver at t = {:.4}; the folded polyline is {:?} and straddles {folded_count} \
             multiples of 2 pi",
            first.len(),
            rec.t,
            folded_rel.iter().map(|x| (x * 1e3).round() / 1e3).collect::<Vec<_>>()
        );
        assert_eq!(
            folded_count, 0,
            "the folded polyline must find no sheet at all - it reads these four rays as a loop \
             lying between 0.3 and 1.2 rad, entirely on one side of the receiver's azimuth, so it \
             would have recorded no arrival on either pass: {folded_rel:?}"
        );
    }

    #[test]
    fn test_the_raw_azimuth_difference_of_a_neighbouring_pair_is_continuous() {
        // Why the raw difference and not the folded one: it is the physical winding between two
        // neighbouring rays, and the evidence for that is that it moves continuously.
        //
        // Both rays of a pair leave the emission event at the emitter's own azimuth, so their
        // difference starts at exactly zero, and both integrate phi continuously, so the difference
        // can only change at the rate the two rays' own dphi/dt allow. It therefore never jumps -
        // and in particular it passes through pi without anything happening to it, which is exactly
        // where folding into [-pi, pi] would send it discontinuously to -pi and put that piece of
        // the front on the other side of the hole.
        //
        // A whole light cone is let go at r = 2.0 at a = 0.90 and run for 60 M at the frame step.
        // The winding comes from the unstable circular photon orbits outside r+ (r = 1.56 prograde
        // and r = 3.89 retrograde at this spin): a ray on very nearly the critical impact parameter
        // hangs at one of them for tens of M while the neighbour it was emitted next to has escaped
        // or fallen in.
        let metric = KerrSchild::new(1.0, 0.90);
        let alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 2.0, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses.len(), 1);
        let dt = 0.017;
        let steps = 3530; // 60.01 M of coordinate time
        let n = field.pulses[0].rays.len();

        // Per ray: azimuth, dphi/dt and whether it is still running, at the previous step.
        let sample = |field: &SignalField| -> Vec<(f64, f64, bool)> {
            field.pulses[0].rays.iter().map(|ray| (ray.phi, ray.dphi_dt, ray.alive())).collect()
        };
        let mut prev = sample(&field);
        let mut worst_ratio = 0.0f64;
        let mut worst = (0.0f64, 0usize, 0.0f64, 0.0f64);
        let mut t = 0.0;
        for _ in 0..steps {
            field.advance(&metric, dt);
            t += dt;
            let now = sample(&field);
            for i in 0..n {
                let j = (i + 1) % n;
                // Only a pair that was running at both ends of the step: a ray that reached the
                // ring or left the field inside it stands still afterwards, and the drawing and
                // the detection both drop that segment.
                if !(prev[i].2 && prev[j].2 && now[i].2 && now[j].2) {
                    continue;
                }
                let jump = ((now[j].0 - now[i].0) - (prev[j].0 - prev[i].0)).abs();
                // The generous bound the brief asks for: six times the step times the largest
                // |dphi/dt| either ray had at either end of it. The exact bound is the integral of
                // |dphi/dt_j - dphi/dt_i| over the step, which is at most 2 dt times that maximum;
                // six leaves room for the rate itself moving inside the step.
                let rate = prev[i]
                    .1
                    .abs()
                    .max(prev[j].1.abs())
                    .max(now[i].1.abs())
                    .max(now[j].1.abs());
                let bound = 6.0 * dt * rate;
                assert!(
                    jump <= bound,
                    "the raw difference of rays {i} and {j} jumped by {jump} at t = {t}, more \
                     than the {bound} that 6 dt max|dphi/dt| = 6 x {dt} x {rate} allows: it is \
                     not a continuous function of time and the raw difference would not be the \
                     physical one"
                );
                if bound > 0.0 && jump / bound > worst_ratio {
                    worst_ratio = jump / bound;
                    worst = (t, i, jump, bound);
                }
            }
            prev = now;
        }

        // And over that run the winding does get past half a turn, which is where the fold and the
        // raw difference part company.
        let pulse = &field.pulses[0];
        let pi = std::f64::consts::PI;
        let mut wound = 0;
        let mut live_pairs = 0;
        let mut largest = 0.0f64;
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            live_pairs += 1;
            let raw = (pulse.rays[j].phi - pulse.rays[i].phi).abs();
            if raw > pi {
                wound += 1;
            }
            largest = largest.max(raw);
        }
        println!(
            "after {:.2} M a pulse of {n} rays let go at r = 2.0 at a = 0.90 has {live_pairs} live \
             neighbouring pairs, {wound} of them more than half a turn apart, the largest raw \
             |d phi| being {largest:.3} rad ({:.2} turns); over the whole run the worst step-to-step \
             jump in a pair's raw difference was {:.3} of the 6 dt max|dphi/dt| bound ({:.3e} \
             against {:.3e}, at t = {:.2} on segment {})",
            (steps as f64) * dt,
            largest / (2.0 * pi),
            worst_ratio,
            worst.2,
            worst.3,
            worst.0,
            worst.1
        );
        assert!(
            wound >= 1,
            "a ray hung on a circular photon orbit must wind past half a turn away from its \
             neighbour within 60 M; none of the {live_pairs} live pairs did"
        );
        assert!(largest > two_pi_turns(3.0), "and past three turns: {largest}");
    }

    /// Whole turns of azimuth, in radians: a named number for the winding assertions.
    fn two_pi_turns(turns: f64) -> f64 {
        turns * 2.0 * std::f64::consts::PI
    }

    /// The app's startup layout, run on a fixed grid: Alice released from r = 4.5M at t = 0 and Bob
    /// from r = 3.8M at the same moment, both raindrops at a = 0.90, each broadcasting and each
    /// listening to the other. Returns the two reception records and the two worldlines as (t, r)
    /// samples, so that a caller can ask the extent tracks where the receiver was.
    ///
    /// The emission cadence is put on the grid rather than on the emitter's proper clock.
    /// `emit_if_due` fires at the first pass at which the next emission is due, so two runs on
    /// different step sizes send their pulses from events up to a step apart - an O(dt) shift in
    /// every crossing that says nothing about the detection being compared. Here both runs emit
    /// from the same events, every `emit_every` steps, and what is left between them is the
    /// detection and the integration.
    #[allow(clippy::type_complexity)]
    fn run_startup_layout(
        metric: &KerrSchild,
        until: f64,
        dt: f64,
        emit_every: usize,
    ) -> (SignalField, SignalField, Vec<(f64, f64)>, Vec<(f64, f64)>) {
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut bob = Observer::new_with_phi(metric, "Bob", 0.0, 3.8, 0.0, 0.0, params);
        let mut from_alice = SignalField::default();
        let mut from_bob = SignalField::default();
        let mut alice_track = vec![(alice.t, alice.r)];
        let mut bob_track = vec![(bob.t, bob.r)];
        from_alice.emit_if_due(metric, &alice);
        from_bob.emit_if_due(metric, &bob);
        let steps = (until / dt).round() as usize;
        for i in 1..=steps {
            let t = (i as f64) * dt;
            alice.step(metric, t, dt);
            bob.step(metric, t, dt);
            from_alice.advance(metric, dt);
            from_bob.advance(metric, dt);
            if i % emit_every == 0 {
                // The cadence is the grid's, not the emitter's: see the note above.
                from_alice.last_emit_tau = None;
                from_alice.emit_if_due(metric, &alice);
                from_bob.last_emit_tau = None;
                from_bob.emit_if_due(metric, &bob);
            }
            from_alice.detect_receptions(metric, &bob);
            from_bob.detect_receptions(metric, &alice);
            alice_track.push((alice.t, alice.r));
            bob_track.push((bob.t, bob.r));
        }
        (from_alice, from_bob, alice_track, bob_track)
    }

    /// The receiver's radius at coordinate time `t`, interpolated between the samples of a
    /// worldline recorded by `run_startup_layout`.
    fn radius_at(track: &[(f64, f64)], t: f64) -> f64 {
        match track.iter().position(|&(sample, _)| sample >= t) {
            None => track.last().unwrap().1,
            Some(0) => track[0].1,
            Some(k) => {
                let ((t0, r0), (t1, r1)) = (track[k - 1], track[k]);
                let w = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
                r0 + w * (r1 - r0)
            }
        }
    }

    /// The first coordinate time at which a pulse's radial extent came to bracket the receiver,
    /// having not done so at the previous stored point, or None if it never did.
    ///
    /// This is the (t, r) diagram's own statement that the receiver is in *range* of the pulse:
    /// some ray of the front stands at their radius. It is not the same as an arrival, because the
    /// ray standing there may be at any azimuth, so it is used here only as an oracle - a pulse
    /// that came into range and stayed there long enough must have been heard - and never as a
    /// substitute for the per-sheet crossing test.
    fn came_into_range(pulse: &Pulse, track: &[(f64, f64)]) -> Option<f64> {
        let mut was_inside = false;
        for &(t, lo, hi) in pulse.extent_track.iter() {
            let r = radius_at(track, t);
            let inside = r >= lo && r <= hi;
            if inside && !was_inside {
                return Some(t);
            }
            was_inside = inside;
        }
        None
    }

    #[test]
    fn test_every_pulse_of_the_startup_layout_is_heard_once_at_either_step_size() {
        // The bug the user could see: regular gaps in the reception dots along both worldlines,
        // out in the weak field where nothing is winding at all. A pulse's front is an off-centre
        // closed curve that grows, so the segment of it standing across a fixed azimuth changes as
        // it does, and a sheet keyed by that segment index handed itself over to a neighbour every
        // so often. Whenever a handover fell in the same step as the crossing, the new key had no
        // remembered side and the arrival was never recorded.
        //
        // Two things are asserted here, on the layout the app opens in. First, determinism: the
        // same run at a quarter of the step must find the same arrivals, at the same events. A
        // detection that depends on how often the app happened to look is a detection that is
        // missing some. Second, completeness, with the extent track as the oracle: every pulse
        // whose radial extent came to bracket the receiver, early enough for the front to have
        // finished sweeping round to their azimuth, must have been heard exactly once. The extent
        // track is only a statement about range, not about arrival, which is why it is used this
        // way round - as a lower bound on what must have been heard - and not as a detector.
        let metric = KerrSchild::new(1.0, 0.90);
        let until = 4.0;
        // 0.2 M between emissions, a whole number of steps at either size.
        let (coarse_alice, coarse_bob, coarse_a_track, coarse_b_track) =
            run_startup_layout(&metric, until, 0.02, 10);
        let (fine_alice, fine_bob, ..) = run_startup_layout(&metric, until, 0.005, 40);

        let listed = |field: &SignalField| -> Vec<(usize, f64, f64)> {
            let mut out: Vec<(usize, f64, f64)> =
                field.receptions().map(|rec| (rec.pulse_index, rec.t, rec.ratio)).collect();
            out.sort_by(|a, b| a.1.total_cmp(&b.1));
            out
        };
        let mut worst_t = 0.0f64;
        let mut worst_ratio = 0.0f64;
        for (label, coarse, fine) in [
            ("Alice -> Bob", &coarse_alice, &fine_alice),
            ("Bob -> Alice", &coarse_bob, &fine_bob),
        ] {
            let (a, b) = (listed(coarse), listed(fine));
            assert_eq!(
                a.len(),
                b.len(),
                "{label}: {} arrivals at dt = 0.02 against {} at dt = 0.005:\n{a:?}\nvs\n{b:?}",
                a.len(),
                b.len()
            );
            assert!(a.len() >= 8, "{label}: the sample is thin: {a:?}");
            for (x, y) in a.iter().zip(b.iter()) {
                assert_eq!(x.0, y.0, "{label}: a different pulse: {x:?} vs {y:?}");
                worst_t = worst_t.max((x.1 - y.1).abs());
                worst_ratio = worst_ratio.max((x.2 - y.2).abs() / y.2);
            }
        }
        assert!(worst_t < 1e-3, "a crossing moved {worst_t} M with the step size");
        assert!(worst_ratio < 1e-3, "a shift moved {worst_ratio} with the step size");

        // The oracle. A pulse that comes into range at t has to be given time for the sheet at the
        // receiver's azimuth to reach them: the two are a quarter of a radian apart in azimuth and
        // the extent is the extremal ray over all of them. `LAG` is that allowance; the largest lag
        // actually measured is 0.359 M, and it is printed beside the allowance so that the
        // allowance can be seen to be one.
        const LAG: f64 = 0.75;
        let mut checked = 0;
        let mut worst_lag = 0.0f64;
        for (label, field, track) in [
            ("Alice -> Bob", &coarse_alice, &coarse_b_track),
            ("Bob -> Alice", &coarse_bob, &coarse_a_track),
        ] {
            for pulse in field.pulses.iter() {
                let Some(entered) = came_into_range(pulse, track) else {
                    continue;
                };
                let heard = pulse.receptions.len();
                if entered > until - LAG {
                    continue;
                }
                checked += 1;
                assert_eq!(
                    heard, 1,
                    "{label}: pulse {} came into range at t = {entered:.3} and was heard {heard} \
                     times, not once: {:?}",
                    pulse.index, pulse.receptions
                );
                worst_lag = worst_lag.max(pulse.receptions[0].t - entered);
            }
        }
        println!(
            "the startup layout to t = {until}: {} + {} arrivals, the same at dt = 0.02 and dt = \
             0.005 to {worst_t:.3e} M in the crossing time and {worst_ratio:.3e} relative in the \
             shift; {checked} pulses came into range early enough to have been heard and every one \
             of them was heard exactly once, the slowest {worst_lag:.3} M after coming into range \
             against an allowance of {LAG} M",
            coarse_alice.received_count(),
            coarse_bob.received_count()
        );
        assert!(checked >= 15, "the oracle should cover most of the transmission: {checked}");
        assert!(worst_lag < LAG, "the allowance must be one: {worst_lag}");
    }

    #[test]
    fn test_field_step_back_removes_later_pulses_and_receptions() {
        // The whole field is reversible, not just one ray. A transmission run to t = 12 and then
        // wound back 3 M has to become the transmission run to t = 9: the pulses Alice sent in
        // those three M were never sent, the crossings Bob recorded in them never happened, and
        // every ray of every surviving pulse is back where it stood at t = 9.
        let metric = KerrSchild::new(1.0, 0.65);
        let dt = 0.01;
        let mut wound = run_field_to(&metric, 12.0, dt);
        let pulses_at_12 = wound.pulses.len();
        let receptions_at_12 = wound.received_count();
        assert!(pulses_at_12 > 10, "the run should have a transmission in flight: {pulses_at_12}");
        assert!(receptions_at_12 > 0, "and Bob should have heard some of it");

        for _ in 0..300 {
            wound.step_back(&metric, dt);
        }
        assert!(
            (wound.t - 9.0).abs() < 1e-9,
            "the field's clock lands on the target: t = {}",
            wound.t
        );
        println!(
            "pulses {} -> {}, receptions {} -> {}",
            pulses_at_12,
            wound.pulses.len(),
            receptions_at_12,
            wound.received_count()
        );
        assert!(
            wound.received_count() < receptions_at_12,
            "the crossings Bob made in those three M must be unrecorded: {} of {receptions_at_12}",
            wound.received_count()
        );
        for pulse in wound.pulses.iter() {
            assert!(
                pulse.emitted_t <= 9.0 + 1e-9,
                "a pulse emitted at t = {} survived a rewind to t = 9",
                pulse.emitted_t
            );
            // The extent track is truncated with everything else, and never below its seed.
            let seed = pulse.extent_track[0];
            assert_eq!(
                (seed.0, seed.1, seed.2),
                (pulse.emitted_t, pulse.emitted_r, pulse.emitted_r),
                "pulse {} lost its emission event to the rewind",
                pulse.index
            );
            for &(t, r_min, r_max) in pulse.extent_track.iter() {
                assert!(
                    t <= 9.0 + 1e-9 || pulse.extent_track.len() == 1,
                    "pulse {}: an extent point at t = {t} survived a rewind to t = 9",
                    pulse.index
                );
                assert!(r_min <= r_max, "pulse {}: extent {r_min} > {r_max} at t = {t}", pulse.index);
            }
            for reception in pulse.receptions.iter() {
                assert!(
                    reception.t <= 9.0 + 1e-9,
                    "a crossing recorded at t = {} survived a rewind to t = 9",
                    reception.t
                );
            }
        }
        assert_eq!(
            wound.last_emit_tau,
            wound.pulses.iter().map(|p| p.emitted_tau).reduce(f64::max),
            "Alice resumes from the newest pulse she has left"
        );

        // The exact statement: what the rewind leaves is what running forward to t = 9 builds.
        let fresh = run_field_to(&metric, 9.0, dt);
        let mut compared = 0;
        let mut worst = 0.0f64;
        for pulse in wound.pulses.iter().filter(|p| p.emitted_t < 9.0 - 1e-9) {
            let Some(same) = fresh.pulses.iter().find(|q| q.index == pulse.index) else {
                panic!("pulse {} is missing from the forward run", pulse.index);
            };
            assert_eq!(pulse.rays.len(), same.rays.len());
            for (a, b) in pulse.rays.iter().zip(same.rays.iter()) {
                assert_eq!(
                    a.alive(),
                    b.alive(),
                    "pulse {}: a rewound ray and a freshly run one disagree about being alive",
                    pulse.index
                );
                if a.alive() {
                    let err = state_gap(a, b);
                    worst = worst.max(err);
                    assert!(err < 1e-4, "pulse {}: ray state gap {err}", pulse.index);
                }
                compared += 1;
            }
        }
        assert!(compared > 100, "the comparison should cover most of the field: {compared} rays");
        println!("rewound vs forward: {compared} rays, worst state gap {worst:e}");

        // Alice has reached the ring and stopped broadcasting well before t = 9, so the window
        // above un-sends no pulse; a window inside her broadcast does. Wound from t = 4 back to
        // t = 1, the field keeps exactly the pulses she had sent by then and drops the rest.
        let mut early = run_field_to(&metric, 4.0, dt);
        let sent_by_4 = early.pulses.len();
        for _ in 0..300 {
            early.step_back(&metric, dt);
        }
        assert!(
            early.pulses.len() < sent_by_4,
            "three M of emissions must be un-sent: {} of {sent_by_4} left",
            early.pulses.len()
        );
        assert!(!early.pulses.is_empty(), "and everything sent before t = 1 must stay");
        for pulse in early.pulses.iter() {
            assert!(
                pulse.emitted_t <= 1.0 + 1e-9,
                "a pulse emitted at t = {} survived a rewind to t = 1",
                pulse.emitted_t
            );
        }
        assert_eq!(
            early.last_emit_tau,
            early.pulses.iter().map(|p| p.emitted_tau).reduce(f64::max)
        );
    }

    #[test]
    fn test_emission_cadence_and_clear() {
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut field = SignalField::default();

        // An observer still waiting for release transmits: they are the static observer at their
        // hover radius, with a clock ticking at sqrt(-g_tt) and a frame to broadcast into, and
        // r = 4.5M is well outside the static limit r = 2M where that worldline stops existing.
        let mut waiting = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 3.0, 0.25, params);
        waiting.step(&metric, 0.5, 0.5);
        assert!(!waiting.is_active && waiting.tau > 0.0, "she is hovering, and her clock runs");
        let mut hover_field = SignalField::default();
        hover_field.emit_if_due(&metric, &waiting);
        assert_eq!(hover_field.pulses.len(), 1, "a hovering Alice transmits from the first call");
        let hovered = hover_field.pulses[0].clone();
        assert!(
            (hovered.emitted_r - 4.5).abs() < 1e-12 && (hovered.emitted_t - 0.5).abs() < 1e-12,
            "at the event she is hovering at: {:?}",
            (hovered.emitted_t, hovered.emitted_r)
        );
        // The frame is the static one: light is isotropic in the frame of the worldline she is on,
        // and while she waits that is the integral curve of d/dt her clock is keeping time by.
        let u_static = signalling_four_velocity(&metric, &waiting);
        let g_tt = metric.metric_components(4.5)[0][0];
        assert!(
            (u_static[0] - 1.0 / (-g_tt).sqrt()).abs() < 1e-12
                && u_static[1] == 0.0
                && u_static[2] == 0.0,
            "a hoverer signals in the static frame: {u_static:?}"
        );
        assert_eq!(
            u_static,
            waiting.four_velocity(&metric),
            "and that is the frame the observer reports too: there is no correction left here"
        );
        // Inside the static limit there is no such worldline, and a waiting observer is silent.
        let mut deep = Observer::new_with_phi(&metric, "Alice", 0.0, 1.5, 3.0, 0.0, params);
        deep.step(&metric, 0.5, 0.5);
        let mut deep_field = SignalField::default();
        deep_field.emit_if_due(&metric, &deep);
        assert!(!deep.is_active);
        assert_eq!(deep_field.pulses.len(), 0, "no static observer exists at r = 1.5M to transmit");

        let dt = 0.01;
        let mut t = 0.0;
        let mut taus = Vec::new();
        for _ in 0..300 {
            t += dt;
            alice.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &alice);
        }
        for p in field.pulses.iter() {
            taus.push(p.emitted_tau);
            assert_eq!(p.rays.len(), RAYS_PER_PULSE);
        }
        assert!(taus.len() >= 2, "at least two pulses in 3M of t: {taus:?}");
        for w in taus.windows(2) {
            assert!(
                w[1] - w[0] >= EMISSION_INTERVAL_TAU - 1e-9,
                "cadence is Alice's proper time: {taus:?}"
            );
            assert!(w[1] - w[0] < EMISSION_INTERVAL_TAU + 0.05, "and not much more: {taus:?}");
        }

        field.clear();
        assert!(field.pulses.is_empty());
        // After a clear the next step re-emits immediately, since there is no last emission left.
        alice.step(&metric, t + dt, dt);
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses.len(), 1);
    }

    #[test]
    fn test_the_ray_count_is_read_at_emission_and_a_pulse_keeps_its_own() {
        // The "Wavefront points" slider sets how finely the *next* pulse samples the emitter's
        // cone. A pulse already in flight cannot be resampled - its rays are the null geodesics
        // that were launched - so the count belongs to the pulse and not to the field, and a field
        // that has been turned up carries both densities at once.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let alice = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut field = SignalField::default();
        assert_eq!(field.rays_per_pulse, RAYS_PER_PULSE, "the default is the named constant");

        field.rays_per_pulse = 64;
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses[0].rays.len(), 64);

        // Turned up between emissions, with a step in between so that the cadence lets the second
        // pulse out at all.
        field.rays_per_pulse = 1024;
        let mut alice = alice;
        let mut t = 0.0;
        while field.pulses.len() < 2 && t < 1.0 {
            t += 0.05;
            alice.step(&metric, t, 0.05);
            field.advance(&metric, 0.05);
            field.emit_if_due(&metric, &alice);
        }
        assert_eq!(field.pulses.len(), 2, "the cadence let a second pulse out by t = {t}");
        assert_eq!(field.pulses[1].rays.len(), 1024, "the new pulse takes the new count");
        assert_eq!(field.pulses[0].rays.len(), 64, "and the old one keeps the count it went out at");
    }

    #[test]
    fn test_alpha_zero_is_the_outward_radial_leg_at_every_count() {
        // The angle convention, pinned. The emission angles are alpha = 2 pi i / n whatever n is,
        // so ray zero of a 64-point pulse and ray zero of a 1024-point one from the same event are
        // the *same* null geodesic - the emitter's own outward radial leg - and every ray of the
        // coarse pulse is a ray of the fine one, at sixteen times the index. Nothing about the
        // sampling density may leak into the physics of a ray.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let alice = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);

        let emit = |n: usize| {
            let mut field = SignalField { rays_per_pulse: n, ..Default::default() };
            field.emit_if_due(&metric, &alice);
            field.pulses.into_iter().next().expect("a released Alice emits at once")
        };
        let (coarse, fine) = (emit(64), emit(1024));
        assert_eq!((coarse.rays.len(), fine.rays.len()), (64, 1024));

        let same = |a: &NullRay, b: &NullRay, what: &str| {
            for (x, y, name) in [
                (a.r, b.r, "r"),
                (a.phi, b.phi, "phi"),
                (a.dr_dt, b.dr_dt, "dr/dt"),
                (a.dphi_dt, b.dphi_dt, "dphi/dt"),
                (a.f_emit, b.f_emit, "f_emit"),
            ] {
                assert!((x - y).abs() <= 1e-12, "{what}: {name} {x} against {y}");
            }
        };
        // alpha = 0 is radially outward in the emitter's frame: dphi/dt is the frame dragging of
        // the event and nothing else, and dr/dt is the outgoing edge of her own light cone.
        let leg = &coarse.rays[0];
        assert!(leg.dr_dt > 0.0, "alpha = 0 leaves outward: dr/dt = {}", leg.dr_dt);
        same(leg, &fine.rays[0], "the outward radial leg");
        // And the whole of the coarse cone is a sub-sampling of the fine one: 1024 = 16 * 64, and
        // 2 pi i / 64 is 2 pi (16 i) / 1024 exactly.
        for i in 0..coarse.rays.len() {
            same(&coarse.rays[i], &fine.rays[16 * i], &format!("ray {i} against ray {}", 16 * i));
        }
    }

    /// The two radial branches k_r of a null ray of conserved (E, L) at radius r, straight from the
    /// null condition of the module header,
    ///
    ///     Delta k_r^2 + 2 (a L - 2 M r E) k_r + [L^2 - (r^2 + 2 M r) E^2] = 0,
    ///
    /// which is g^{mu nu} k_mu k_nu = 0 written out with the inverse metric of this chart. Nothing
    /// about the ray's history enters: given the two conserved components (E, L) = (-k_t, k_phi),
    /// the radial one is fixed up to the choice of branch at every radius it visits.
    fn radial_covector_roots(metric: &KerrSchild, r: f64, e: f64, l: f64) -> (f64, f64) {
        let delta = metric.delta(r);
        let half_b = metric.a * l - 2.0 * metric.m * r * e;
        let c = l * l - (r * r + 2.0 * metric.m * r) * e * e;
        let disc = (half_b * half_b - delta * c).max(0.0).sqrt();
        ((-half_b + disc) / delta, (-half_b - disc) / delta)
    }

    /// nu = -k . u for a ray of conserved (E, L) whose radial branch at this event is k_r, measured
    /// by an observer of 4-velocity u: with k_mu = (-E, k_r, L) that is E u^t - k_r u^r - L u^phi.
    fn frequency_from_constants(e: f64, k_r: f64, l: f64, u: &[f64; 3]) -> f64 {
        e * u[0] - k_r * u[1] - l * u[2]
    }

    /// Bob's transmission, received by Alice: the mirror image of `run_transmission`, with the
    /// emitter and the receiver swapped. Alice is released from r = 4.5M at t = 0 at phi = 0.25 and
    /// Bob is held at the same radius until t = `delta_t`, both raindrops, exactly as the Drop
    /// Observers button builds them. Returns the two worldlines and the field, run to `until` on a
    /// fixed step.
    ///
    /// Bob transmits from t = 0, through the whole of his wait: while he hovers he is the static
    /// observer at r = 4.5M, whose proper time runs at sqrt(-g_tt) = 0.745 of coordinate time, so
    /// his pulses come every 0.134 M of t rather than every 0.1.
    ///
    /// No adaptive step is needed here, unlike `run_transmission`. Nothing of Bob's ever stands on
    /// r- waiting for Alice: she is ahead of him, so the only rays of his that reach her are the
    /// ones that outrun her, and they sweep over her out in the open where a fixed hundredth of an
    /// M resolves them easily.
    pub(super) fn run_return_transmission(
        metric: &KerrSchild,
        delta_t: f64,
        until: f64,
        dt: f64,
    ) -> (Observer, Observer, SignalField) {
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        let mut bob = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, delta_t, 0.0, params);
        let mut field = SignalField::default();
        let steps = (until / dt).round() as usize;
        for i in 0..steps {
            let t = ((i + 1) as f64) * dt;
            alice.step(metric, t, dt);
            bob.step(metric, t, dt);
            field.advance(metric, dt);
            field.emit_if_due(metric, &bob);
            field.detect_receptions(metric, &alice);
        }
        (alice, bob, field)
    }

    #[test]
    fn test_alice_receives_bobs_pulses_from_behind() {
        // The return path, with Bob released 1 M of coordinate time after Alice from the same
        // radius so that most of what he sends is sent while he is falling behind her. His pulses
        // have to chase her, and only the part of each front that outruns her ever arrives: in this
        // chart the ingoing principal null ray travels at dr/dt = -1, which no timelike worldline
        // can match, so the ingoing arc of every pulse gains on her while the outward arc, which
        // falls no faster than the raindrop congruence itself, never does. Her own frozen family is
        // not in the picture at all: the rays of his that pile onto r- settle there behind her,
        // after she has already crossed and gone on to the ring, so unlike Bob she never meets a
        // stack.
        //
        // Measured with the step below: 26 arrivals, ratios from 1.081 down to 0.533, every one of
        // them recorded before her worldline ends at t = 6.30. The mild blueshifts at the top of
        // that range belong to the pulses he sends once he is falling: a prograde ray of his front
        // caught by a receiver who has fallen deeper into the potential can come in above unity,
        // where the ingoing ray of the same front cannot. Deeper in it is all redshift, because
        // catching her from behind means catching an observer running away.
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let (alice, _bob, field) = run_return_transmission(&metric, 1.0, 6.5, 0.005);

        let mut heard: Vec<Reception> = field.receptions().copied().collect();
        heard.sort_by(|a, b| a.t.total_cmp(&b.t));
        assert!(heard.len() >= 10, "Alice should hear the transmission: {heard:?}");
        assert!(alice.has_ended(), "and the run should carry her to the end of her worldline");
        assert!(
            heard.iter().any(|rec| {
                field
                    .pulses
                    .iter()
                    .any(|p| p.index == rec.pulse_index && p.emitted_r < 4.4)
            }),
            "some of what she hears must have been sent after he let go: {heard:?}"
        );
        for reception in heard.iter() {
            assert!(
                reception.ratio.is_finite() && reception.ratio > 0.0,
                "every measured shift is finite and positive: {reception:?}"
            );
            // Inside r+ the only rays of his that can still reach her are the ones outrunning her,
            // and those are redshifted: see the exact calculation in the next test, which gives the
            // shift of the ingoing principal null ray between two raindrops as the quotient of
            // their two values of u^t + u^r - a u^phi, a number below one whenever the receiver is
            // the deeper of the two.
            if reception.r < rp {
                assert!(
                    reception.ratio < 1.0,
                    "light caught inside r+ from behind must be redshifted: {reception:?}"
                );
            }
        }
        let loudest = heard.iter().map(|r| r.ratio).fold(f64::MIN, f64::max);
        let quietest = heard.iter().map(|r| r.ratio).fold(f64::MAX, f64::min);
        println!(
            "Bob -> Alice at Dt = 1: {} arrivals, ratio {quietest:.4} to {loudest:.4}, Alice ends at t = {:.2}",
            heard.len(),
            alice.t
        );
        // Nothing of his piles up on r- in front of her, so there is no blueshift of the kind Bob
        // measures crossing her stack: the whole transmission stays inside a factor of two.
        assert!(loudest < 2.0, "no stack for her to cut through: loudest = {loudest}");
    }

    #[test]
    fn test_the_shift_alice_measures_on_bobs_light_is_the_exact_one() {
        // The shift, computed twice by two routes that share nothing but the metric.
        //
        // Route one is the app's: `NullRay::frequency_ratio`, the ratio of `f_factor` at the
        // reception event to the `f_emit` stored at emission, with the ray's direction carried
        // there by the integrator.
        //
        // Route two uses only the ray's conserved covariant components. Normalise the ray at
        // emission so that k^t = 1 there; then E = -k_t and L = k_phi are two numbers fixed for the
        // whole flight, and at any radius the third component k_r is a root of the null condition
        // of the module header, a quadratic in k_r with no reference to the ray's history at all.
        // The frequency an observer of 4-velocity u measures is then -k . u = E u^t - k_r u^r -
        // L u^phi, and the ratio between two events is the quotient of those. The integrated ray is
        // asked one question only, which of the two radial branches it is on, and the residual of
        // that identification is itself asserted below.
        //
        // Agreement between the two is therefore a measurement of how well the integration holds E
        // and L over the flight. Measured: better than one part in 1e6 over the ~1 M of coordinate
        // time the leading ray needs to catch her.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let mut alice = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params);
        // The app's own delay, so the emitter here is the *static* Bob of the first pulse he sends,
        // hovering at r = 4.5M and still 8 M of coordinate time from release. Both routes below
        // therefore have to use the static 4-velocity, which is what `signalling_four_velocity`
        // returns for him and what the emission tetrad was built on.
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 8.0, 0.0, params);
        let mut field = SignalField::default();
        let dt = 0.005;

        // The probe: the leading ray of Bob's first pulse, the one with the most negative dr/dt,
        // which is the ray of that front that gains on Alice fastest.
        let mut probe: Option<(NullRay, f64, f64, f64)> = None;
        let mut checked = false;
        let mut t = 0.0;
        for _ in 0..800 {
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &bob);
            field.detect_receptions(&metric, &alice);

            if probe.is_none()
                && let Some(pulse) = field.pulses.first()
            {
                let ray = *pulse
                    .rays
                    .iter()
                    .min_by(|a, b| a.dr_dt.total_cmp(&b.dr_dt))
                    .expect("a pulse has rays");
                let g = metric.metric_components(ray.r);
                let v = ray.direction();
                let e = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
                let k_r = g[1][0] * v[0] + g[1][1] * v[1] + g[1][2] * v[2];
                let l = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
                // The same numbers must satisfy the null condition at the emission radius, which
                // checks the quadratic itself before it is used anywhere.
                let (root_a, root_b) = radial_covector_roots(&metric, ray.r, e, l);
                let nearest = if (root_a - k_r).abs() < (root_b - k_r).abs() { root_a } else { root_b };
                assert!(
                    (nearest - k_r).abs() < 1e-9 * (1.0 + k_r.abs()),
                    "k_r = {k_r} is not a root {root_a} / {root_b} of the null condition"
                );
                let u_bob = signalling_four_velocity(&metric, &bob);
                assert!(!bob.is_active && u_bob[1] == 0.0, "the emitter is hovering: {u_bob:?}");
                let nu_bob = frequency_from_constants(e, k_r, l, &u_bob);
                assert!(nu_bob > 0.0, "the emitter measures a positive frequency: {nu_bob}");
                probe = Some((ray, e, l, nu_bob));
                continue;
            }

            if let Some((ray, e, l, nu_bob)) = probe.as_mut() {
                ray.step(&metric, dt);
                if checked || !ray.alive() || ray.r > alice.r {
                    continue;
                }
                // The reception event: the leading ray has just caught Alice up in radius. She is
                // an E = 1, L = 0 raindrop, so her 4-velocity at that radius *is* the raindrop
                // congruence there, which is how both routes get to evaluate it at exactly the
                // ray's radius rather than a fraction of a step away from it.
                let u_alice = raindrop(&metric, ray.r);
                let integrated = alice.four_velocity(&metric);
                for mu in 0..3 {
                    assert!(
                        (u_alice[mu] - integrated[mu]).abs() < 2e-2 * (1.0 + integrated[mu].abs()),
                        "the congruence value at r = {} is Alice's own u: {u_alice:?} vs {integrated:?}",
                        ray.r
                    );
                }

                let measured = ray.frequency_ratio(&metric, &u_alice);
                // Which branch: the root the ray has been on since emission. The gap between that
                // root and the ray's own k_r, in the emission normalisation, is the integration's
                // drift off the null cone and off the conserved (E, L).
                let g = metric.metric_components(ray.r);
                let v = ray.direction();
                let e_now = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
                let k_t_scale = *e / e_now;
                let own_k_r = (g[1][0] * v[0] + g[1][1] * v[1] + g[1][2] * v[2]) * k_t_scale;
                let (root_a, root_b) = radial_covector_roots(&metric, ray.r, *e, *l);
                let k_r = if (root_a - own_k_r).abs() < (root_b - own_k_r).abs() {
                    root_a
                } else {
                    root_b
                };
                let branch_gap = (k_r - own_k_r).abs() / (1.0 + own_k_r.abs());
                assert!(
                    branch_gap < 1e-4,
                    "the ray must still be on a branch of its own null cone: {branch_gap}"
                );

                let predicted = frequency_from_constants(*e, k_r, *l, &u_alice) / *nu_bob;
                println!(
                    "reception at t = {:.3}, r = {:.4}: measured {measured:.8} vs predicted \
                     {predicted:.8} (branch gap {branch_gap:e})",
                    ray.t, ray.r
                );
                assert!(
                    (measured - predicted).abs() < 1e-3 * predicted.abs(),
                    "measured {measured} vs the (E, L) prediction {predicted}"
                );
                assert!(predicted > 0.0 && predicted.is_finite());
                checked = true;
            }
        }
        assert!(checked, "the leading ray of Bob's first pulse must catch Alice");

        // The same statement in the one case that has a closed form, and the one the Theory Guide
        // quotes: the ingoing principal null ray. It runs at dr/dt = -1 and dphi/dt = 0 everywhere
        // in this chart, and the frequency any observer measures on it is nu/nu_inf = u^t + u^r -
        // a u^phi (`KerrSchild::ingoing_frequency_ratio`), so between two raindrops the shift is
        // just the quotient of their two values - no integration, no f_factor. It is a redshift
        // whenever the receiver is the deeper of the two, which is Alice's whole situation here;
        // for a = 0 the emitter at infinity and the receiver at the horizon give exactly 1/2, and
        // that is the same formula with r_emit -> infinity.
        let r_emit = 4.0;
        let u_emit = raindrop(&metric, r_emit);
        let mut pnd = NullRay {
            t: 0.0,
            r: r_emit,
            phi: 0.0,
            dr_dt: -1.0,
            dphi_dt: 0.0,
            f_emit: f_factor(&metric, r_emit, &[1.0, -1.0, 0.0], &u_emit),
            v_emit: [1.0, -1.0, 0.0],
            death_t: None,
            death_end: None,
        };
        for _ in 0..600 {
            pnd.step(&metric, 0.005);
            if !pnd.alive() {
                break;
            }
            let u_here = raindrop(&metric, pnd.r);
            let measured = pnd.frequency_ratio(&metric, &u_here);
            let closed_form = metric.ingoing_frequency_ratio(pnd.r, &u_here)
                / metric.ingoing_frequency_ratio(r_emit, &u_emit);
            assert!(
                (measured - closed_form).abs() < 1e-6 * closed_form,
                "ingoing PND shift {measured} vs (u^t + u^r - a u^phi) quotient {closed_form} at r = {}",
                pnd.r
            );
            assert!(
                measured < 1.0,
                "a raindrop below the emitter must see the ingoing ray redshifted: {measured} at r = {}",
                pnd.r
            );
        }
    }

    #[test]
    fn test_bobs_late_pulses_never_reach_alice() {
        // The point of running the transmission both ways, at the app's own default layout: Alice
        // released from r = 4.5M at t = 0, Bob hovering at the same radius until t = 8. He
        // transmits throughout the wait, as the static observer he is while he waits, and his
        // pulses chase her down; her worldline ends on the ring at t = 6.30, and once it has, no
        // later pulse of his has anywhere to arrive. The last one that did marks the event on his
        // own worldline - a point on the vertical hover segment, hours of his proper time before he
        // even lets go - beyond which nothing he sends can ever be heard: the boundary of the
        // causal past of the end of her worldline. Nothing predicts that event here; it is read off
        // the run, which is the only criterion this app trusts.
        //
        // Measured: 15 arrivals, ratios 0.362 to 0.751, and the last delivered pulse is #14, sent
        // at t = 1.97 from r = 4.5 exactly (he has not moved) at his proper time 1.468, first heard
        // by Alice at t = 5.51 when she was down at r = 0.99. The 83 pulses he sends after it - 45
        // more from the hover and 38 on the way down - are never heard by anybody.
        let metric = KerrSchild::new(1.0, 0.65);
        let (alice, bob, field) = run_return_transmission(&metric, 8.0, 15.0, 0.01);
        assert!(alice.has_ended(), "the run must carry Alice to the end of her worldline");
        assert!(bob.is_active, "and carry Bob past his own release at t = 8");

        let heard = field.received_count();
        assert!(heard >= 5, "his hover transmission must reach her: {heard} arrivals");
        let ratios: Vec<f64> = field.receptions().map(|r| r.ratio).collect();
        assert!(
            ratios.iter().all(|r| r.is_finite() && *r > 0.0 && *r < 1.0),
            "a static emitter above a falling receiver is a redshift throughout: {ratios:?}"
        );

        let last = field
            .last_delivered_pulse()
            .expect("some of Bob's transmission reached her");
        let later = field.pulses_after(last.pulse_index);
        println!(
            "default layout: {heard} arrivals, ratio {:.4} to {:.4}; last delivered #{} sent at \
             t = {:.3}, r = {:.6}, Bob's tau = {:.3}, first heard at t = {:.3}; {later} later \
             pulses never arrive",
            ratios.iter().copied().fold(f64::MAX, f64::min),
            ratios.iter().copied().fold(f64::MIN, f64::max),
            last.pulse_index,
            last.emitted_t,
            last.emitted_r,
            last.emitted_tau,
            last.received_t
        );
        assert!(
            (1.5..3.0).contains(&last.emitted_t),
            "the cut-off is about 2 M into his wait: t = {}",
            last.emitted_t
        );
        assert!(
            (last.emitted_r - 4.5).abs() < 1e-9,
            "and he had not moved when he sent it: r = {}",
            last.emitted_r
        );
        assert!(last.emitted_t < 8.0, "so the boundary event is on his hover segment");
        assert!(later >= 40, "he goes on transmitting long past it: {later}");

        // Everything sent after that event is sent to nobody, hover pulses and infall pulses alike.
        for pulse in field.pulses.iter() {
            if pulse.index > last.pulse_index {
                assert!(
                    pulse.receptions.is_empty(),
                    "pulse {} was sent after the last delivery and cannot have arrived: {:?}",
                    pulse.index,
                    pulse.receptions
                );
                assert!(
                    pulse.emitted_t > last.emitted_t,
                    "serial order is emission order: {} at t = {}",
                    pulse.index,
                    pulse.emitted_t
                );
            }
        }
        let after_release = field.pulses.iter().filter(|p| p.emitted_t >= 8.0).count();
        assert!(after_release > 0, "he transmits after his release too");
        assert!(
            field
                .pulses
                .iter()
                .filter(|p| p.emitted_t >= 8.0)
                .all(|p| p.receptions.is_empty()),
            "and none of those {after_release} pulses is ever heard"
        );
        // Nothing arrives after her worldline ends, either.
        let end_t = alice.trail.iter().map(|p| p.t).fold(0.0f64, f64::max);
        for reception in field.receptions() {
            assert!(
                reception.t <= end_t + 1e-9,
                "an arrival at t = {} is past the end of her worldline",
                reception.t
            );
        }

        // The record outlives the wavefront. By t = 15 Bob has sent about a hundred pulses against
        // a cap of `MAX_PULSES`, so the pulse that carried the last delivery is long gone from the
        // field; the delivery is not, which is the whole reason `Delivery` is kept separately.
        assert!(
            field.pulses.iter().all(|p| p.index != last.pulse_index),
            "the delivering pulse should have been evicted by now"
        );
        assert!(
            field.pulses.first().map(|p| p.index).unwrap_or(0) > last.pulse_index,
            "every pulse still in hand is newer than it"
        );
    }

    /// The worst gap between two fields, ray by ray, over the rays both of them have alive: first
    /// in position (r, phi), which is what "the ray is in the same place" means, then in the
    /// direction (v^r, v^phi) it carries. Also returns how many rays were compared and how many
    /// disagreed about being alive at all.
    ///
    /// A ray that one field has retired and the other has not is counted rather than compared. The
    /// state of a dead ray stands at the last event before it left the field, which is one substep
    /// short of the boundary, and two runs that cut the interval differently take that last substep
    /// from different places; the death event is therefore a per-run quantity of order
    /// `MAX_DR_PER_SUBSTEP`, and comparing a dead ray with a live one measures nothing.
    fn field_gap(a: &SignalField, b: &SignalField) -> (f64, f64, usize, usize) {
        let mut position = 0.0f64;
        let mut direction = 0.0f64;
        let mut compared = 0usize;
        let mut disagreed = 0usize;
        assert_eq!(a.pulses.len(), b.pulses.len(), "the two fields hold different pulses");
        for (pa, pb) in a.pulses.iter().zip(b.pulses.iter()) {
            assert_eq!(pa.index, pb.index);
            for (ra, rb) in pa.rays.iter().zip(pb.rays.iter()) {
                if ra.alive() != rb.alive() {
                    disagreed += 1;
                    continue;
                }
                if !ra.alive() {
                    continue;
                }
                position = position.max((ra.r - rb.r).abs().max((ra.phi - rb.phi).abs()));
                direction =
                    direction.max((ra.dr_dt - rb.dr_dt).abs().max((ra.dphi_dt - rb.dphi_dt).abs()));
                compared += 1;
            }
        }
        (position, direction, compared, disagreed)
    }

    #[test]
    fn test_a_large_step_is_the_same_geodesics_as_the_frame_steps_it_replaces() {
        // The bug this integrator was written for. A played frame is about a fiftieth of an M, and
        // the fixed subdivision it replaces - one substep count read off the state at the start of
        // the interval and applied to all of it - was accurate at that size and nowhere else. A
        // hand step of 0.1 M was already 9e-3 M out on the rays near the ring; one 2 M backstep,
        // which is an ordinary Distance-mode step at a supermassive hole, put rays that had gone
        // deep inside at r = 1.07 where they belonged at r = 2.46, an error of 2 M.
        //
        // The claim now is that the caller's step size is not a physical parameter: one step of dt
        // and the same interval in frame-sized pieces must be the same geodesics, because either
        // way the substeps are chosen from the states along the path. Measured as the worst gap
        // over every ray of a whole transmission (Alice broadcasting from r = 4.5 at a = 0.90, run
        // to t = 5 in frames of 0.017), one step against pieces of 0.017:
        //
        //     dt = 0.1    forward 2.8e-12   backward 3.6e-12
        //     dt = 2.0    forward 3.4e-10   backward 6.0e-8
        //     dt = 20.0   forward 3.2e-9    backward 1.4e-5
        //
        // against 9.4e-4, 1.5 and worse under the fixed subdivision.
        //
        // The last of those is not a step-size effect and no scheme can do better: a 20 M *retrace*
        // is an ill-conditioned problem. A ray that has frozen onto r- sits at r - r- ~
        // exp(-kappa_- t), so running it backwards blows that offset back up by exp(kappa_- dt),
        // which at a = 0.90 is 2.3e3 over 20 M; the same amplification acts on any difference
        // between two ways of cutting the path. The measurement that says so is in the test: each
        // of the two rewinds is also compared with the state it should return to, and both miss by
        // the same 3.6e-5, which is the size of their disagreement. So the assertion for a
        // rewind is 1e-6 or the round trip's own residual, whichever is larger, and the round trip
        // is printed either way.
        let metric = KerrSchild::new(1.0, 0.90);
        let frame = 0.017;
        let base = run_field_to(&metric, 5.0, frame);
        let rays: usize = base.pulses.iter().map(|p| p.rays.len()).sum();
        assert!(base.pulses.len() > 20, "the test needs a full field: {}", base.pulses.len());

        // The 20 M case is run on the oldest four pulses rather than on the whole field: it is 80 M
        // of ray integration per job, and the claim is about a ray, so the sample is cut instead of
        // the interval.
        let mut trimmed = base.clone();
        trimmed.pulses.truncate(4);

        let jobs: Vec<(f64, SignalField)> =
            vec![(0.1, base.clone()), (2.0, base.clone()), (20.0, trimmed)];
        let outcomes: Vec<String> = std::thread::scope(|scope| {
            let handles: Vec<_> = jobs
                .iter()
                .map(|(dt, start)| {
                    let metric = &metric;
                    scope.spawn(move || {
                        let dt = *dt;
                        let pieces = (dt / frame).ceil() as usize;
                        let h = dt / pieces as f64;

                        let mut one = start.clone();
                        one.advance(metric, dt);
                        let mut many = start.clone();
                        for _ in 0..pieces {
                            many.advance(metric, h);
                        }
                        let (position, direction, compared, disagreed) = field_gap(&one, &many);
                        assert!(compared > 100, "dt = {dt}: only {compared} rays compared");
                        assert!(
                            position < 1e-6,
                            "dt = {dt} forward: {position} M between one step and {pieces} of {h}"
                        );
                        let forward = format!(
                            "dt = {dt} forward: position {position:.3e}, direction \
                             {direction:.3e}, {compared} rays, {disagreed} disagreed about dying"
                        );

                        // The rewind is measured from the far end of that same interval, so that a
                        // step backwards of dt has dt of light to undo and lands back on `start`.
                        let mut one = many.clone();
                        one.step_back(metric, dt);
                        let mut back = many.clone();
                        for _ in 0..pieces {
                            back.step_back(metric, h);
                        }
                        let (position, direction, compared, disagreed) = field_gap(&one, &back);
                        // What each rewind cost against the state it should have returned to: the
                        // retrace's own conditioning, which is what the two of them share.
                        let (trip_one, ..) = field_gap(&one, start);
                        let (trip_many, ..) = field_gap(&back, start);
                        assert!(compared > 100, "dt = {dt}: only {compared} rays compared back");
                        assert!(
                            position < 1e-6f64.max(3.0 * trip_many),
                            "dt = {dt} backward: {position} M between one step and {pieces} of \
                             {h}, against a round-trip residual of {trip_many} M"
                        );
                        assert_eq!(
                            (one.budget_exhausted(), back.budget_exhausted()),
                            (0, 0),
                            "no ray may be retired by the substep budget"
                        );
                        format!(
                            "{forward}\ndt = {dt} backward: position {position:.3e}, direction \
                             {direction:.3e}, {compared} rays, {disagreed} disagreed about dying; \
                             round trip to the state it started from {trip_one:.3e} in one step \
                             and {trip_many:.3e} in {pieces}"
                        )
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });
        println!("{rays} rays in the field\n{}", outcomes.join("\n"));
    }

    #[test]
    fn test_the_field_round_trip_lands_where_it_started() {
        // Forward N, back M, forward M: the field has to end up where it stood after the first N,
        // because the M steps in between undid each other. What is left is the integration's own
        // asymmetry - the way back cuts the path at different places from the way out - and it is
        // measured rather than assumed. Measured: 2.2e-10 M over 1619 live rays, at 0.1 M steps
        // over a field of 34 pulses, against 1.2e-6 under the fixed subdivision this replaces.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut field = run_field_to(&metric, 5.0, 0.017);
        for _ in 0..10 {
            field.advance(&metric, 0.1);
        }
        let mark = field.clone();
        for _ in 0..5 {
            field.step_back(&metric, 0.1);
        }
        assert!(
            (field.t - (mark.t - 0.5)).abs() < 1e-12,
            "the clock came back by half an M: {} vs {}",
            field.t,
            mark.t
        );
        for _ in 0..5 {
            field.advance(&metric, 0.1);
        }
        let (position, direction, compared, disagreed) = field_gap(&field, &mark);
        println!(
            "round trip (10 forward, 5 back, 5 forward, all at 0.1 M): position {position:.3e}, \
             direction {direction:.3e} over {compared} live rays, {disagreed} disagreed about dying"
        );
        assert_eq!(disagreed, 0, "the round trip must not change which rays are alive");
        assert!(position < 1e-8, "round-trip residual {position} M");
        assert_eq!(field.budget_exhausted(), 0, "no ray may be retired by the substep budget");
    }

    #[test]
    fn test_one_pulse_crossings_are_the_same_at_a_quarter_of_the_step() {
        // A crossing time is a measurement of the geometry, so it must not depend on how often the
        // app happened to look. It is found by interpolating the side value between two detection
        // passes, which is first-order accurate in the pass interval, so halving the step should
        // move a crossing by O(dt^2) and nothing else.
        //
        // What must not be in the way of that measurement is the emission cadence. `emit_if_due`
        // fires on the first pass at which the emitter's proper time is due, so two runs on
        // different grids send their pulses from events up to a step apart, and a front launched
        // 0.02 M later arrives 0.02 M later - an O(dt) difference that has nothing to do with the
        // detection. So a single pulse is sent here, at t = 0, from the same event in both runs.
        //
        // Measured over the crossing family outside r+ (the sheets that sweep past a receiver in
        // the open, where the front and the receiver are both moving at an ordinary rate): nine
        // crossings from nine emissions, the worst moving 1.2e-4 M between a step of 0.02 and a
        // step of 0.005, which is 0.30 dt^2, and the worst shift moving 9.1e-6 relative.
        //
        // The last two geometries in the list are here because they used to be impossible. From
        // r = 4.0 to a receiver at r = 3.3, and from r = 4.5 to one at r = 4.0, the coarse run
        // recorded no crossing at all where the fine run recorded one - not an integration error
        // and not the interpolation, but the sheet key of `Pulse::scan`, which was the index of the
        // polyline segment. Where the segment straddling the receiver changed between two passes,
        // the sign change was split across two keys and neither pass saw it. A sheet is now
        // identified by its position along the loop instead, so a handover is not a new sheet, and
        // both geometries record their crossing at either step size.
        let metric = KerrSchild::new(1.0, 0.90);
        let rp = metric.outer_horizon();

        let run = |dt: f64, r_alice: f64, r_bob: f64| -> Vec<Reception> {
            let params = WorldlineParams::default();
            let mut alice =
                Observer::new_with_phi(&metric, "Alice", 0.0, r_alice, 0.0, 0.25, params);
            let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, r_bob, 0.0, 0.0, params);
            // One pulse and no more: the cadence is put out of reach so that both runs compare the
            // same wavefront, launched from the same event.
            let mut field = SignalField { interval_tau: f64::INFINITY, ..Default::default() };
            field.emit_if_due(&metric, &alice);
            assert_eq!(field.pulses.len(), 1);
            let mut t = 0.0;
            while t < 4.0 {
                t += dt;
                alice.step(&metric, t, dt);
                bob.step(&metric, t, dt);
                field.advance(&metric, dt);
                field.detect_receptions(&metric, &bob);
            }
            let mut heard: Vec<Reception> = field
                .receptions()
                .filter(|rec| !rec.frozen_family && rec.r > rp)
                .copied()
                .collect();
            heard.sort_by(|a, b| a.t.total_cmp(&b.t));
            heard
        };

        // One pulse sweeps over one receiver about once, so the sample is four emissions at
        // different radii rather than one long run: each is its own single-pulse transmission with
        // its own receiver below it, and the crossings of all four are compared.
        let mut crossings = 0;
        let mut worst_t = 0.0f64;
        let mut worst_ratio = 0.0f64;
        let emissions = [
            (4.5f64, 3.8f64),
            (5.0, 4.2),
            (5.5, 4.6),
            (6.0, 5.0),
            (3.5, 2.9),
            (5.0, 3.9),
            (6.5, 5.6),
            (4.0, 3.3),
            (4.5, 4.0),
        ];
        for &(r_alice, r_bob) in emissions.iter() {
            let coarse = run(0.02, r_alice, r_bob);
            let fine = run(0.005, r_alice, r_bob);
            assert!(
                !coarse.is_empty(),
                "the pulse from r = {r_alice} must sweep over a receiver at r = {r_bob}"
            );
            assert_eq!(
                coarse.len(),
                fine.len(),
                "the two step sizes must find the same sheets from r = {r_alice}: {coarse:?} vs \
                 {fine:?}"
            );
            for (a, b) in coarse.iter().zip(fine.iter()) {
                worst_t = worst_t.max((a.t - b.t).abs());
                worst_ratio = worst_ratio.max((a.ratio - b.ratio).abs() / b.ratio);
                crossings += 1;
            }
        }
        assert!(crossings >= 4, "the sweep should give several crossings: {crossings}");
        println!(
            "{crossings} single-pulse crossings outside r+: the worst moves {worst_t:.3e} M \
             between dt = 0.02 and dt = 0.005, which is {:.2} dt^2, and its shift moves \
             {worst_ratio:.3e} relative",
            worst_t / (0.02 * 0.02)
        );
        assert!(worst_t < 1e-3, "a crossing time moved {worst_t} M with the step size");
        assert!(worst_ratio < 1e-3, "a crossing shift moved {worst_ratio} with the step size");
    }

    /// The gain of a ray between two raindrops, the quantity the equatorial view colours its
    /// fronts by: the frequency the drop at the ray's current event measures, over the frequency
    /// the drop passing the emitter measured as the ray left. The drawing code assembles it from
    /// `NullRay::gain_between` and `Pulse::emitted_r`, and so does this.
    fn ray_gain(metric: &KerrSchild, pulse: &Pulse, ray: &NullRay) -> f64 {
        let u_emit = raindrop(metric, pulse.emitted_r);
        let u_now = raindrop(metric, ray.r);
        ray.gain_between(metric, pulse.emitted_r, &u_emit, &u_now)
    }

    #[test]
    fn test_a_fresh_front_has_gain_one_on_every_ray_whatever_the_emitter_is_doing() {
        // The property the wavefront colouring is built on. Held against the emitter's own frame,
        // the rays of one pulse are spread across the whole shift ramp the instant they leave -
        // that is aberration, and it is a true statement about the emission. Held between two
        // members of the raindrop congruence, one at the emission event and one at the ray's
        // current event, they are all exactly 1 at emission, because at that moment the two ends
        // are the same f_factor computed twice: same radius, same direction, same observer. So a
        // fresh front is one uniform colour however the emitter was moving, and everything the
        // colour later shows is gain the light picked up on its way.
        //
        // "Exactly" is meant: the two evaluations are bit-for-bit identical expressions, so the
        // tolerance below is round-off in the division and nothing else.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();

        // Three emitters at the same radius, on three quite different worldlines: one hovering
        // (never released, so `is_static_hover` and the static frame), one falling as a raindrop
        // from rest at infinity, and one on a prograde geodesic with real angular momentum, whose
        // aberration is the strongest of the three.
        let emitters: [(&str, Observer); 3] = [
            (
                "a static hoverer at r = 4.5",
                Observer::new_with_phi(&metric, "Hover", 0.0, 4.5, 100.0, 0.0, params),
            ),
            (
                "a raindrop released at r = 4.5",
                Observer::new_with_phi(&metric, "Rain", 0.0, 4.5, 0.0, 0.0, params),
            ),
            (
                "a prograde faller at r = 4.5, E = 1.3, L = 2.0",
                Observer::new_with_phi(
                    &metric,
                    "Prograde",
                    0.0,
                    4.5,
                    0.0,
                    0.0,
                    WorldlineParams::new(1.3, 2.0, false),
                ),
            ),
        ];

        for (what, emitter) in emitters.iter() {
            let mut field = SignalField::default();
            field.emit_if_due(&metric, emitter);
            let pulse = field.pulses.first().unwrap_or_else(|| panic!("{what} must emit"));
            assert!(pulse.rays.len() >= 64, "{what}: {} rays", pulse.rays.len());

            // The emitter-relative ratio, for contrast: this is the quantity the receptions and the
            // HUD use, and it is exactly what a front must not be coloured by, because it is
            // already spread out before the light has gone anywhere.
            let u_here = raindrop(&metric, pulse.emitted_r);
            let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
            let mut worst_gain = 0.0f64;
            for ray in pulse.rays.iter() {
                let gain = ray_gain(&metric, pulse, ray);
                assert!(gain.is_finite() && gain > 0.0, "{what}: gain {gain}");
                worst_gain = worst_gain.max((gain - 1.0).abs());
                let ratio = ray.frequency_ratio(&metric, &u_here);
                lo = lo.min(ratio);
                hi = hi.max(ratio);
            }
            println!(
                "{what}: every one of the {} rays leaves at gain 1 to {worst_gain:.3e}, while the \
                 ratio against the emitter's own frame already runs from {lo:.4} to {hi:.4}",
                pulse.rays.len()
            );
            assert!(
                worst_gain < 1e-12,
                "{what}: a ray left at gain {} away from 1; a fresh front must be one colour",
                worst_gain
            );
        }
    }

    #[test]
    fn test_the_frozen_family_climbs_the_gain_ramp_without_bound() {
        // The other end of the ramp. A pulse let go inside r+ splits into the family that crosses
        // r- at finite coordinate time and the family with E - Omega_- L < 0, which cannot: those
        // rays settle onto r- as r - r- ~ exp(-kappa_- t) while the raindrops go on falling
        // through the surface, so the frequency a drop at the ray's event measures runs away like
        // exp(kappa_- t). At a = 0.90, kappa_- = (r- - r+) / (2 (r-^2 + a^2)) = -0.386 per M, so
        // 30 M of it is e^11.6 ~ 1e5 - which is why the drawn ramp runs to a gain of 1e5 where the
        // reception ramp stops at 1e3. They are different quantities with different ranges.
        let metric = KerrSchild::new(1.0, 0.90);
        let rp = metric.outer_horizon();
        let alice = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            1.0,
            0.0,
            0.0,
            WorldlineParams::default(),
        );
        assert!(alice.r < rp, "the pulse must be let go inside r+ = {rp}");
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses.len(), 1);
        let dt = 0.02;
        let steps = 1500; // 30 M of coordinate time
        for _ in 0..steps {
            field.advance(&metric, dt);
        }

        let pulse = &field.pulses[0];
        let mut frozen = 0;
        let mut smallest_frozen = f64::INFINITY;
        let mut largest = 0.0f64;
        let mut live = 0;
        for ray in pulse.rays.iter() {
            if !ray.alive() {
                continue;
            }
            live += 1;
            let gain = ray_gain(&metric, pulse, ray);
            assert!(
                gain.is_finite() && gain > 0.0,
                "a live ray at r = {} has gain {gain}, which is not a measurement",
                ray.r
            );
            largest = largest.max(gain);
            if ray.frozen(&metric) {
                frozen += 1;
                smallest_frozen = smallest_frozen.min(gain);
            }
        }
        println!(
            "{} M after a pulse of {} rays was let go at r = 1.0 at a = 0.90, {live} are still \
             running and {frozen} of them are frozen (E - Omega_- L < 0); the dimmest frozen ray \
             has gained {smallest_frozen:.4e} and the brightest ray of the whole front \
             {largest:.4e}",
            (steps as f64) * dt,
            pulse.rays.len()
        );
        assert!(frozen > 0, "a pulse let go inside r+ must have a frozen family");
        assert!(
            smallest_frozen > 100.0,
            "every frozen ray must have climbed well up the ramp in 30 M: the dimmest is at \
             {smallest_frozen}"
        );
    }
}

#[cfg(test)]
mod late_survivors_on_r_minus {
    //! Where the survivors of a pulse sent inside r+ sit at late times, and on which side of r-.
    use super::*;
    use crate::physics::observer::{Observer, WorldlineParams};

    /// Both families of survivors hug r-, from opposite sides, and both close on it at the rate
    /// kappa_-.
    ///
    /// The frozen family, E - Omega_- L < 0, never reaches r-: it approaches from *above* as
    /// r - r- ~ exp(-kappa_- t), which is the stack a later infaller cuts through. The crossing
    /// family, E - Omega_- L > 0, crosses r- inward at finite t. Those of its rays whose radial
    /// potential has a root above the ring turn there, in Region III, and climb back out - and an
    /// outgoing ray cannot cross r- outward in this chart any more than one can cross r+ outward,
    /// so they approach r- from *below*, as r- - r ~ exp(-kappa_- t), with dr/dt > 0 all the way.
    /// Seen from the equatorial view at high zoom that is a second set of arcs riding r- from the
    /// inside, thinner than the frozen ones and lagging them by their own turning time, and they
    /// are physical: the outgoing light of Region III accumulating on the future boundary of that
    /// region, which this chart pins to t = infinity at r = r- just as it does the outgoing light of
    /// Region II.
    ///
    /// Measured at a = 0.90 for a pulse let go at r = 1.0 (kappa_- = 0.386 / M): at t = 20 the 16
    /// rays inside stand at most 1.3e-4 M below r- and the 23 outside at most 9.8e-4 M above; by
    /// t = 60 those have shrunk to 2.5e-11 and 1.9e-10, both a factor exp(15.45) in 40 M against
    /// the exp(15.44) that kappa_- predicts; by t = 150 every survivor is on r- to 1e-12 in f64,
    /// which is the precision floor and not physics.
    #[test]
    fn test_survivors_close_on_r_minus_from_both_sides_at_the_rate_kappa_minus() {
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let kappa = metric.inner_surface_gravity();
        let mut alice =
            Observer::new_with_phi(&metric, "A", 0.0, 1.0, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &alice);
        let dt = 0.02;
        let mut t = 0.0;
        // The largest offset from r- on each side, over the live rays, at the two sample times.
        let mut run_to = |target: f64, field: &mut SignalField| -> (f64, f64, usize, usize) {
            while t < target - 1e-9 {
                alice.step(&metric, t, dt);
                field.advance(&metric, dt);
                t += dt;
            }
            let (mut above, mut below) = (0.0f64, 0.0f64);
            let (mut n_above, mut n_below) = (0, 0);
            for ray in field.pulses[0].rays.iter().filter(|r| r.alive()) {
                let d = ray.r - rm;
                if d > 0.0 {
                    n_above += 1;
                    above = above.max(d);
                    assert!(ray.frozen(&metric), "every survivor above r- is of the frozen family");
                } else {
                    n_below += 1;
                    below = below.max(-d);
                    assert!(!ray.frozen(&metric), "no survivor below r- is of the frozen family");
                    assert!(ray.dr_dt > 0.0, "a survivor below r- is climbing back toward it");
                }
            }
            (above, below, n_above, n_below)
        };
        let (a20, b20, na, nb) = run_to(20.0, &mut field);
        let (a60, b60, na60, nb60) = run_to(60.0, &mut field);
        println!(
            "t = 20: {na} survivors above r- within {a20:.2e}, {nb} below within {b20:.2e}; t = 60: {na60} above within {a60:.2e}, {nb60} below within {b60:.2e}; predicted collapse over 40 M: exp(-{:.2})",
            40.0 * kappa
        );
        assert!(na >= 10 && nb >= 10, "both populations are well represented: {na} / {nb}");
        assert_eq!((na, nb), (na60, nb60), "and nobody changes side or dies between 20 and 60 M");
        for (early, late, side) in [(a20, a60, "above"), (b20, b60, "below")] {
            let measured = (early / late).ln();
            assert!(
                (measured - 40.0 * kappa).abs() < 0.05 * 40.0 * kappa,
                "{side}: closed on r- by exp({measured:.2}) over 40 M against kappa_- t = {:.2}",
                40.0 * kappa
            );
        }
    }
}
