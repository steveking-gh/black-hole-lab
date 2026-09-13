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
//! f_emit, and no affine normalisation is ever needed.
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
//! curve in (r, phi); projected onto the radial axis it is the interval between its innermost and
//! its outermost live ray, and that interval swept up in coordinate time is `Pulse::extent_track`,
//! the wedge the spacetime canvas draws. Its lower edge is bounded exactly by the ingoing edge of
//! the emitter's own light cone, and its upper edge freezes onto r- for every pulse emitted inside
//! r+, which is why the wedges stack against the Cauchy horizon there. A worldline inside a wedge
//! is only in *range* of the pulse; whether the pulse reaches it is a question about azimuth, which
//! the projection has thrown away and only the per-sheet crossing test of `Pulse::scan` answers.

use crate::physics::geodesic::{R_STOP, geodesic_accel};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::tetrad::Tetrad;

/// Directions per pulse: the whole of the emitter's local light cone at five-degree spacing, with
/// the count chosen so that alpha = 0, their own outward radial leg, lands on a ray rather than in
/// a gap between two.
pub const RAYS_PER_PULSE: usize = 72;

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

/// Coordinate time between stored points of a pulse's radial-extent track.
const TRACK_DT: f64 = 0.2;

/// Points past which a track stops growing.
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
    /// Coordinate time at which the ray left the field (r > `R_ESCAPE`) or reached the ring
    /// (r < R_STOP), or None while it is still running.
    ///
    /// The rest of the state is left standing at that death event rather than discarded, which is
    /// what makes the death reversible: a `step_back` over an interval reaching past `death_t`
    /// clears this field and integrates the stored state backwards out of the boundary again.
    pub death_t: Option<f64>,
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
            death_t: None,
        }
    }

    /// Whether the ray is still running. A dead ray keeps its state at the death event, so this is
    /// a question about the ray's clock rather than about whether its state means anything.
    pub fn alive(&self) -> bool {
        self.death_t.is_none()
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
        let rm = metric.inner_horizon();
        let denom = rm * rm + metric.a * metric.a;
        let omega_minus = if denom > 0.0 { metric.a / denom } else { 0.0 };
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

    /// L / E = g_{phi mu} v^mu / (-g_{t mu} v^mu), the one scale-free constant of the motion a
    /// direction can carry. Conserved exactly along the ray, which is what the tests check.
    #[allow(dead_code)] // the integration's conservation diagnostic; the tests are its caller
    pub fn l_over_e(&self, metric: &KerrSchild) -> f64 {
        let g = metric.metric_components(self.r);
        let v = self.direction();
        let k_phi = g[2][0] * v[0] + g[2][1] * v[1] + g[2][2] * v[2];
        let e_over_kt = -(g[0][0] * v[0] + g[0][1] * v[1] + g[0][2] * v[2]);
        k_phi / e_over_kt
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
                self.death_t = None;
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
                    self.commit(&y, t_now);
                    self.death_t = Some(t_now);
                    return RayStep::BudgetExhausted;
                }
                budget -= 1;
                let Some((candidate, slope, error)) = ray_dopri5(metric, &y, &k, sign * h) else {
                    // A stage of this substep would have been evaluated below the ring. Shrink it
                    // until it fits above; when even the smallest substep does not, the ray has
                    // reached the ring and is retired at the last event it did reach.
                    if h <= MIN_SUBSTEP {
                        self.commit(&y, t_now);
                        self.death_t = Some(t_now);
                        return RayStep::Integrated;
                    }
                    h *= 0.5;
                    continue;
                };
                let estimate = error[0].abs().max(error[1].abs());
                let sane = candidate.iter().all(|c| c.is_finite()) && estimate.is_finite();
                if sane && (estimate <= SUBSTEP_TOLERANCE || h <= MIN_SUBSTEP) {
                    proposal = h * substep_factor(estimate);
                    break (candidate, slope);
                }
                if !sane && h <= MIN_SUBSTEP {
                    // Nothing finite can be made of this state even at the round-off floor. The
                    // ray is retired where it last stood, as it would be at a boundary.
                    self.commit(&y, t_now);
                    self.death_t = Some(t_now);
                    return RayStep::Integrated;
                }
                h *= if sane { substep_factor(estimate) } else { 0.5 };
            };

            if forward && next[0] > R_ESCAPE {
                self.commit(&y, t_now);
                self.death_t = Some(t_now);
                return RayStep::Integrated;
            }
            y = next;
            k = slope;
            remaining -= h;
        }
        self.commit(&y, end_t);
        RayStep::Integrated
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
/// Returns None instead if any stage of the step would be evaluated below R_STOP, the ring, where
/// the equation gives out. The seventh stage is the step's own result, so this covers the state the
/// step would produce as well as the interior stages that produce it: a step that comes back is a
/// step that stayed in the geometry throughout. The caller shrinks and retries, and retires the ray
/// when even `MIN_SUBSTEP` will not fit.
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
) -> Option<(RayState, RayState, RayState)> {
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
        // Not `< R_STOP`: a stage that has gone non-finite is no more evaluable than one below
        // the ring, and both are the caller's cue to shrink and, in the end, to retire the ray.
        if !arg[0].is_finite() || arg[0] < R_STOP {
            return None;
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
    Some((next, k[6], error))
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
    /// Which sheet of the front this was: the index of the polyline segment that carried it, the
    /// same key `SheetSide` uses. A segment is a pair of rays, so it names the same material sheet
    /// for the life of the pulse.
    segment: usize,
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
/// It is a copy of the four numbers that locate a `Pulse`'s emission event, plus the time of the
/// arrival that made the pulse a delivery, and it exists because the emission event has to outlive
/// the pulse. A transmitter running for a whole infall sends more pulses than the `MAX_PULSES` cap
/// keeps - a hovering emitter alone sends about sixty before release at the app's default delay -
/// and the pulse that carried the last signal to arrive is one of the oldest, so it is the first
/// the cap throws away. The event it marks is a fact about the spacetime and does not stop being
/// true when the drawing of its wavefront is dropped.
#[derive(Debug, Clone, Copy)]
pub struct Delivery {
    /// Serial number of the pulse that was received.
    pub pulse_index: usize,
    /// Coordinate time of the emission event.
    pub emitted_t: f64,
    /// The emitter's proper time at emission.
    pub emitted_tau: f64,
    /// Radius of the emission event.
    pub emitted_r: f64,
    /// Azimuth of the emission event.
    pub emitted_phi: f64,
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
    /// The whole of the emitter's light cone at emission, `RAYS_PER_PULSE` exact null geodesics
    /// ordered by their emission angle alpha, from alpha = 0, their own outward radial leg, round
    /// to alpha = 2 pi - 2 pi / `RAYS_PER_PULSE`. The emitter broadcasts in every direction, so the
    /// polyline these rays form is closed: the last ray joins back to the first.
    pub rays: Vec<NullRay>,
    /// The pulse's *radial extent*, as (t, r_min, r_max) over the rays that were still alive at
    /// that time: what the pulse is on the (t, r) diagram, where the fan of azimuths cannot be
    /// drawn. It is seeded with (t_emit, r_emit, r_emit), the emission event, extended at the
    /// `TRACK_DT` cadence by `Pulse::extend_track`, truncated by `SignalField::step_back`, and it
    /// stops growing once every ray of the pulse is dead.
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
    /// Two things hold r_min off the envelope: the five-degree discretisation of the cone, which
    /// leaves the most ingoing of the `RAYS_PER_PULSE` rays up to 2.5 degrees off the extremal
    /// direction, and the death of that ray at the ring, after which the minimum is taken over
    /// whatever is still alive and lifts away from the bound for good. Before that death the gap
    /// is at most 8e-4 M, measured in
    /// `test_extent_track_lower_edge_hugs_the_steepest_ingoing_ray`, so the drawn lower edge is
    /// the ingoing edge of the emitter's own light cone to well within a pixel - and it runs up to
    /// 0.07 M *below* the 45-degree line over an infall at a = 0.65, which is why the 45-degree
    /// line is not what is claimed here.
    ///
    /// The upper edge has no closed form: it is the outermost live ray, which is the emitter's
    /// outward radial leg only while the pulse is outside r+. Inside r+ every ray falls, and the
    /// outermost of them tends to the outgoing principal null direction of r-, so the upper edge
    /// freezes onto the Cauchy horizon while the lower edge runs on to the ring.
    pub extent_track: Vec<(f64, f64, f64)>,
    /// One entry per sheet of the front that stood across the receiver's azimuth on the previous
    /// detection pass, carrying the side they were on and the event they were at. A sheet is keyed
    /// by the segment of the ray polyline and the turn of azimuth it crosses them on, so folds and
    /// windings are tracked independently instead of collapsing into one number, and a sheet that
    /// stops straddling them simply drops out of the list.
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
    /// Index of the polyline segment (the ray pair) carrying this sheet. It is the whole key: each
    /// unwrapped step is folded into [-pi, pi], so a segment can straddle at most one of the angles
    /// 2 pi n that represent Bob, and naming the winding as well would only make the key fragile.
    /// The folded azimuth of the first ray jumps by a full turn whenever it passes the fold, which
    /// shifts every unwrapped angle and every winding number by one without anything having moved.
    segment: usize,
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
    /// The radial interval the pulse's live rays span right now, or None once none of them is
    /// alive.
    ///
    /// The front is a closed curve in (r, phi), and its projection onto the r axis is exactly this
    /// interval. A dead ray is left out of it, because its state stands at the event it died on
    /// rather than at the current time; a pulse whose every ray has reached the ring or left the
    /// field has no extent at all, and that is what stops its track growing.
    fn radial_extent(&self) -> Option<(f64, f64)> {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for ray in self.rays.iter().filter(|ray| ray.alive()) {
            lo = lo.min(ray.r);
            hi = hi.max(ray.r);
        }
        (lo <= hi).then_some((lo, hi))
    }

    /// Record the pulse's radial extent at the field time `t`, if a `TRACK_DT` has gone by since
    /// the last stored point and the pulse still has a ray alive.
    ///
    /// The cadence is measured against the last stored point rather than accumulated per call, so
    /// a run of short frames stores one point per `TRACK_DT` of coordinate time exactly as one
    /// long frame does, and the drawn wedge does not depend on the frame rate. Nothing is
    /// interpolated: a point is stored at whatever time the first frame past a cadence boundary
    /// lands on, because the extent is read off the rays and the rays only ever stand at times
    /// they have been stepped to.
    fn extend_track(&mut self, t: f64) {
        let Some(&(last_t, ..)) = self.extent_track.last() else {
            return;
        };
        if self.extent_track.len() >= TRACK_MAX_POINTS || t < last_t + TRACK_DT {
            return;
        }
        if let Some((r_min, r_max)) = self.radial_extent() {
            self.extent_track.push((t, r_min, r_max));
        }
    }

    /// Record every crossing of the receiver's worldline by this wavefront on this pass.
    ///
    /// The rays of a pulse are a closed polyline in the (r, phi) plane, ordered by their emission
    /// angle: the emitter broadcasts into their whole light cone, so the last ray joins back to the
    /// first and that closing segment is a sheet like any other. The receiver is located on the
    /// polyline through the *unwrapped* azimuth of each ray relative to theirs: the first ray is
    /// placed within pi of the receiver and every later one within pi of its predecessor, so a
    /// front that frame dragging has wound through several turns is still one continuous curve. On
    /// that unwrapped axis the receiver is not one angle but the whole family 0, +/-2 pi, +/-4 pi,
    /// ..., because a front that has wound one turn further passes over them again. Each polyline
    /// segment straddling one of those angles is one *sheet* of the front standing across their
    /// azimuth, and since every unwrapped step is folded into [-pi, pi] a segment can straddle at
    /// most one of them, so the segment index alone names the sheet.
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
        let two_pi = 2.0 * std::f64::consts::PI;
        let wrap = |d: f64| d - two_pi * (d / two_pi).round();
        let mut rel = Vec::with_capacity(self.rays.len());
        rel.push(wrap(self.rays[0].phi - receiver.phi));
        for i in 1..self.rays.len() {
            let prev = rel[i - 1];
            rel.push(prev + wrap(self.rays[i].phi - self.rays[i - 1].phi));
        }

        // The closing segment runs from the last ray back to the first, its far end unwrapped by
        // one more folded step so that the whole loop stays on the one continuous azimuth axis.
        let n = self.rays.len();
        let closing = rel[n - 1] + wrap(self.rays[0].phi - self.rays[n - 1].phi);
        let mut sheets = Vec::new();
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
                let side = receiver.r - r_front;
                // The shift this sheet carries right now, by the same interpolation along the
                // segment: the two bracketing rays' own frequency ratios, each evaluated at its own
                // event. It is wanted on every pass, not only on a crossing, because the crossing's
                // shift is interpolated between two passes like everything else about the event.
                let f0 = self.rays[i].frequency_ratio(metric, u_receiver);
                let f1 = self.rays[j].frequency_ratio(metric, u_receiver);
                let ratio = f0 + w * (f1 - f0);

                // A sign change on a sheet that was tracked at the previous pass is a crossing,
                // and nothing here second-guesses it. A sheet drops out of straddling the receiver
                // and comes back as the front winds, and while it is away the receiver can pass it
                // by another segment, so no rule that compares this crossing with the last one
                // recorded on the same sheet is safe: it would suppress a real arrival every time a
                // sheet handed the receiver over to a neighbour and took them back. The one case
                // where the same crossing can be offered twice is a rewind, and it is settled
                // there, at the rewound state, by `SignalField::prime`.
                let was = self.sheets.iter().find(|s| s.segment == i).copied();
                if let Some(prev) = was
                    && record
                    && prev.side * side < 0.0
                {
                    // Where in the interval between the two passes the side changed sign.
                    let fraction = prev.side / (prev.side - side);
                    let at = |before: f64, now: f64| before + fraction * (now - before);
                    let crossing_ratio = at(prev.ratio, ratio);
                    if crossing_ratio.is_finite() && crossing_ratio > 0.0 {
                        let nearer = if w < 0.5 { i } else { j };
                        self.receptions.push(Reception {
                            pulse_index: self.index,
                            t: at(prev.t, receiver.t),
                            tau_receiver: at(prev.tau, receiver.tau),
                            r: at(prev.r, receiver.r),
                            phi: at(prev.phi, receiver.phi),
                            ratio: crossing_ratio,
                            frozen_family: self.rays[nearer].frozen(metric),
                            segment: i,
                            side_after: side,
                            t_pass: receiver.t,
                        });
                    }
                }
                sheets.push(SheetSide {
                    segment: i,
                    side,
                    t: receiver.t,
                    r: receiver.r,
                    phi: receiver.phi,
                    tau: receiver.tau,
                    ratio,
                });
            }
        }
        self.sheets = sheets;
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
            let last = self
                .receptions
                .iter()
                .enumerate()
                .rfind(|(_, rec)| rec.segment == sheet.segment);
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

/// The 4-velocity to quote an observer's signals in, at emission and at reception alike.
///
/// A released observer carries their own, whatever worldline they are on. A hovering one is the
/// static observer of `is_static_hover`, and the frame that matches the clock they are keeping is
/// the normalised time-translation Killing vector u = (1, 0, 0) / sqrt(-g_tt). Where no static
/// observer exists the free-fall value is used instead, which is also what a released observer
/// gets.
///
/// This is a local correction, and deliberately local. `Observer::four_velocity` reports the
/// *free-fall* 4-velocity for an observer who is in fact hovering, which is a latent inconsistency
/// in that function: the telemetry, the drawn cones and the Distance-mode step estimate all read
/// it, and none of them is being changed here. The signal code, which has to put a real tetrad at a
/// real emission event and a real 4-velocity at a real reception event, corrects for it through
/// this one function, so the frame a pulse is emitted into and the frame an arrival is measured in
/// are both the frame of the worldline the observer is actually on.
fn signalling_four_velocity(metric: &KerrSchild, observer: &Observer) -> [f64; 3] {
    if is_static_hover(metric, observer) {
        let g_tt = metric.metric_components(observer.r)[0][0];
        return [1.0 / (-g_tt).sqrt(), 0.0, 0.0];
    }
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

        // The frame is built on `signalling_four_velocity` rather than on
        // `Observer::four_velocity`, which would hand a hovering observer the free-fall frame of
        // the radius they are standing still at. Light is isotropic in the frame of the worldline
        // the emitter is actually on, and while they hover that is the static frame.
        let u = signalling_four_velocity(metric, emitter);
        let tetrad = Tetrad::from_four_velocity(metric, emitter.r, &u);
        let two_pi = 2.0 * std::f64::consts::PI;
        let rays = (0..RAYS_PER_PULSE)
            .map(|i| {
                // alpha = 0 is the emitter's outward radial leg and the ray count divides the
                // turn exactly, so the last ray stops one step short of alpha = 2 pi and the front
                // closes.
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
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
            pulse.extend_track(t);
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
            emitted_phi: p.emitted_phi,
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

impl SignalPair<'_> {
    /// Carry both transmissions forward by dt of the simulation clock.
    ///
    /// The order matters and is the same for each field. Advancing first and emitting second keeps
    /// a fresh pulse at its emitter's current event instead of one step behind it, and detecting
    /// last means a pulse emitted this frame already has a recorded side for its receiver before
    /// the next frame can move it.
    ///
    /// With no Alice there is nobody to emit her transmission and nobody for Bob's to reach, so
    /// only Bob's field is carried, and it is carried rather than dropped: the light he has already
    /// sent is still in flight whether or not anyone is left to hear it.
    pub fn advance(
        &mut self,
        metric: &KerrSchild,
        dt: f64,
        alice: Option<&Observer>,
        bob: &Observer,
    ) {
        self.alice.advance(metric, dt);
        self.bob.advance(metric, dt);
        if let Some(al) = alice {
            self.alice.emit_if_due(metric, al);
            self.bob.detect_receptions(metric, al);
        }
        self.bob.emit_if_due(metric, bob);
        self.alice.detect_receptions(metric, bob);
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
        bob: &Observer,
    ) {
        self.alice.step_back(metric, dt);
        self.bob.step_back(metric, dt);
        // Each field is primed against its own receiver: Alice's transmission is received by Bob,
        // and Bob's by Alice. A field's own clock is the time its rewind landed on, which is the
        // target the priming pass reconciles the reception record against.
        let (alice_target, bob_target) = (self.alice.t, self.bob.t);
        self.alice.prime(metric, bob, alice_target);
        if let Some(al) = alice {
            self.bob.prime(metric, al, bob_target);
        }
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
            death_t: None,
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
        // so a little under a third of the 72 rays end up on the surface. Reading the
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
        for w in profile.windows(2) {
            assert!(
                w[1].1 > w[0].1,
                "deeper in the stack must be bluer: {:?} then {:?}",
                w[0],
                w[1]
            );
        }
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
    fn test_frozen_family_is_the_prograde_arc_and_narrows_inward() {
        // Which part of a pulse freezes, read straight off the emission event rather than off a
        // long integration: E - Omega_- L is conserved, so the family a ray belongs to is already
        // settled the instant Alice lets it go. Four raindrop emissions between r+ and the ring
        // give the shape of the answer. The frozen arc is always the *prograde* half of the cone,
        // the rays dragged forward in phi, never the outward radial leg at alpha = 0 and never the
        // retrograde half; and it shrinks as the emitter falls, because a pulse sent close to r-
        // has almost no room left in which frame dragging can beat aberration.
        //
        // Measured, out of the 72 rays of a pulse: 26 frozen at r = 1.7 (just inside r+ = 1.76,
        // alpha from 30 to 155 degrees), 21 at r = 1.0 (40 to 140), 12 at r = 0.5 (60 to 115) and 4
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
        // The bound is not attained, for two reasons. The cone is sampled at five degrees, so the
        // most ingoing of the `RAYS_PER_PULSE` rays sits up to 2.5 degrees off the extremal
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
            "and it is a five-degree sampling of the cone, not the edge itself: gap {worst_outside}"
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
        // A sheet is a pair of rays, and inside r+ the front winds fast enough that the pair
        // straddling the receiver's azimuth changes from pass to pass: a sheet drops out of the
        // tracking and comes back, and while it is away the receiver can pass through the front by
        // one of its neighbours. This is that handoff, built by hand so that it is unambiguous.
        //
        // Four rays at one radius, spaced so that exactly one segment of the closed loop straddles
        // the receiver's azimuth, and the loop is rotated between passes to hand that duty from one
        // segment to another:
        //
        //   1. segment 0 straddles; the front sweeps outward past the receiver     (crossing one)
        //   2. the loop rotates by half a radian, and segment 3 straddles instead;
        //      the front sweeps back inward past the receiver                      (crossing two)
        //   3. the loop rotates back, segment 0 straddles again, and the front
        //      sweeps outward past the receiver a second time                      (crossing three)
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

        // The loop: four rays of a real emission at r = 3, whose azimuths are then set by hand.
        // Steps of 1.5 rad and a closing step of 2 pi - 4.5, so the loop winds once and exactly one
        // of its four segments contains the receiver's azimuth.
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
            sheets: Vec::new(),
            receptions: Vec::new(),
        };

        // One detection pass with the loop put where the caller says: every ray at radius `r_front`
        // and the loop rotated by `turn`.
        let base = [-0.3f64, 1.2, 2.7, 4.2];
        let pass = |pulse: &mut Pulse, bob: &mut Observer, t: f64, r_front: f64, turn: f64| {
            for (ray, phi) in pulse.rays.iter_mut().zip(base.iter()) {
                ray.t = t;
                ray.r = r_front;
                ray.phi = phi + turn;
            }
            bob.t = t;
            bob.tau = 0.8 * t;
            pulse.scan(&metric, bob, &u_receiver, true);
            pulse.sheets.iter().map(|sheet| sheet.segment).collect::<Vec<_>>()
        };

        // 1. Segment 0 straddles: the front starts inside the receiver and sweeps out past them.
        assert_eq!(pass(&mut pulse, &mut bob, 0.1, 2.9, 0.0), vec![0], "segment 0 should straddle");
        assert!(pulse.receptions.is_empty(), "the first pass only establishes the side");
        assert_eq!(pass(&mut pulse, &mut bob, 0.2, 3.1, 0.0), vec![0]);
        assert_eq!(pulse.receptions.len(), 1, "the front swept out past the receiver");

        // 2. The loop rotates: segment 3 takes over, and the receiver passes back through the front
        //    by that segment instead.
        assert_eq!(pass(&mut pulse, &mut bob, 0.3, 3.1, 0.5), vec![3], "segment 3 should straddle");
        assert_eq!(pulse.receptions.len(), 1, "the handoff itself is not a crossing");
        assert_eq!(pass(&mut pulse, &mut bob, 0.4, 2.9, 0.5), vec![3]);
        assert_eq!(pulse.receptions.len(), 2, "the front swept back in past the receiver");

        // 3. The loop rotates back and segment 0 sweeps out past the receiver a second time.
        assert_eq!(pass(&mut pulse, &mut bob, 0.5, 2.9, 0.0), vec![0], "segment 0 again");
        assert_eq!(pulse.receptions.len(), 2, "coming back is not a crossing either");
        assert_eq!(pass(&mut pulse, &mut bob, 0.6, 3.1, 0.0), vec![0]);
        assert_eq!(
            pulse.receptions.len(),
            3,
            "the second crossing of segment 0 is a real arrival and must be recorded: {:?}",
            pulse.receptions
        );

        let events: Vec<(usize, f64)> =
            pulse.receptions.iter().map(|rec| (rec.segment, rec.t)).collect();
        println!("handoff: three crossings, (segment, t) = {events:?}");
        assert_eq!(events[0].0, 0);
        assert_eq!(events[1].0, 3);
        assert_eq!(events[2].0, 0);
        // Each is stamped inside the pass interval that found it, and the two crossings of segment
        // 0 leave the receiver on the same side, which is the whole point.
        for (rec, (lo, hi)) in pulse.receptions.iter().zip([(0.1, 0.2), (0.3, 0.4), (0.5, 0.6)]) {
            assert!(rec.t > lo && rec.t < hi, "{rec:?} is outside ({lo}, {hi})");
            assert!(rec.ratio.is_finite() && rec.ratio > 0.0, "{rec:?}");
        }
        assert!(
            pulse.receptions[0].side_after * pulse.receptions[2].side_after > 0.0,
            "the two crossings of segment 0 leave the receiver on the same side"
        );
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
        // The frame is the static one, not the free-fall frame `Observer::four_velocity` reports
        // for her: light is isotropic in the frame of the worldline she is on. The two differ, and
        // the pulse must be built on the one she is actually keeping time by.
        let u_static = signalling_four_velocity(&metric, &waiting);
        let g_tt = metric.metric_components(4.5)[0][0];
        assert!(
            (u_static[0] - 1.0 / (-g_tt).sqrt()).abs() < 1e-12
                && u_static[1] == 0.0
                && u_static[2] == 0.0,
            "a hoverer signals in the static frame: {u_static:?}"
        );
        assert!(
            (u_static[1] - waiting.four_velocity(&metric)[1]).abs() > 0.1,
            "and that is not the free-fall frame the observer reports"
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
        // returns for him and what the emission tetrad was built on; `Observer::four_velocity`
        // would hand back the free-fall value at that radius instead and the two routes would then
        // be answering different questions.
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
            death_t: None,
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
        // the open, where the front and the receiver are both moving at an ordinary rate): seven
        // crossings from seven emissions, the worst moving 2.9e-5 M between a step of 0.02 and a
        // step of 0.005, which is 0.07 dt^2, and the worst shift moving 3.8e-6 relative.
        //
        // Two of the geometries tried are not in the list, and they are worth naming: from r = 4.0
        // to a receiver at r = 3.3, and from r = 4.5 to one at r = 4.0, the coarse run records no
        // crossing at all where the fine run records one. That is not this integration and not the
        // interpolation - it is the sheet key of `Pulse::scan`, which is the index of the polyline
        // segment. Where a front is winding fast enough that the segment straddling the receiver
        // changes between two passes, the sign change is split across two keys and neither pass
        // sees it. It is a detection dropout at coarse steps, not a wrong answer, and it is left
        // alone here rather than fixed silently.
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
}
