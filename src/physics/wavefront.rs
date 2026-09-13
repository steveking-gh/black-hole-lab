//! Alice's signal, broadcast into the whole of her light cone: exact null geodesics carrying an
//! exact frequency ratio.
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
//! aberration inside r+). The stack Bob crosses is that family. An infalling worldline sweeps
//! through the whole of it in finite proper time, and that is what Bob's receptions below record.

use crate::physics::geodesic::{R_STOP, geodesic_accel};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::tetrad::Tetrad;

/// Directions per pulse: the whole of Alice's local light cone at five-degree spacing, with the
/// count chosen so that alpha = 0, her own outward radial leg, lands on a ray rather than in a gap
/// between two.
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

/// Alice's proper-time interval between pulses, in units of M. Her whole infall from r = 4.5M is
/// about 4.2M of her proper time, so this puts of order forty pulses on the wire before she reaches
/// the ring.
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

/// Hard cap on substeps per ray per call, so a large dt cannot stall a frame.
const MAX_SUBSTEPS: usize = 200;

/// Coordinate time between stored points of a principal-null track.
const TRACK_DT: f64 = 0.2;

/// Points past which a principal-null track stops growing.
const TRACK_MAX_POINTS: usize = 4000;

/// A track stops being extended once it is this close to r-, where it has effectively joined the
/// Cauchy horizon and every further point would land on the same pixel.
const TRACK_R_MINUS_EPS: f64 = 1e-5;

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

    /// Advance the ray by dt of coordinate time with RK4 on the non-affine geodesic equation of
    /// the module header.
    ///
    /// The step is subdivided so that no substep moves the ray more than `MAX_DR_PER_SUBSTEP` in
    /// radius nor turns its direction by more than `DIRECTION_STEP_FRACTION` of the direction's own
    /// size, capped at `MAX_SUBSTEPS`. The second condition is the one that binds close to the ring
    /// and wherever the connection stiffens; outside a few tenths of M neither binds at the frame
    /// steps this app uses.
    ///
    /// An outgoing ray closing on r- from above simply freezes, its dr/dt decaying to zero like
    /// exp(-kappa_- t). That is the correct behaviour and not a stall: the ray genuinely never
    /// crosses this branch of the Cauchy horizon at finite t.
    ///
    /// A dead ray is carried on the clock without moving. Its state stays at the death event and
    /// `death_t` goes on naming that event, so the interval between them is exactly the time a
    /// step backwards has to skip before it starts integrating.
    pub fn step(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let end_t = self.t + dt;
        if self.alive() {
            self.integrate(metric, dt);
        }
        // The clock runs on even if the ray died part way through the step: `death_t` records the
        // event it died at, `t` records what time it is.
        self.t = end_t;
    }

    /// Carry the ray back by dt of coordinate time, dt > 0 meaning "go back by dt".
    ///
    /// `ray_rhs` is a first-order system in (r, phi, v^r, v^phi) with no explicit dependence on t,
    /// so it is time symmetric: the same substepped RK4 run with a negative step integrates the
    /// same geodesic in the other direction rather than solving a different problem, and retraces
    /// the forward path to the accuracy of the scheme. The retrace is not bit-exact, because the
    /// substep count is read off the state a step starts from and that is the far end of the
    /// interval on the way back; the tests measure what is left over.
    ///
    /// A death is reversible too. A ray that reached the ring or left the field inside the interval
    /// being undone is put back on its feet at the death event, where its full state was kept, and
    /// integrated backwards from there to the target time; the remainder of the interval is time it
    /// spent already dead, over which it covered no distance. A ray that died before the target
    /// time stays dead, with its clock wound back like everything else.
    pub fn step_back(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let target_t = self.t - dt;
        match self.death_t {
            None => {
                self.integrate(metric, -dt);
                if self.alive() {
                    self.t = target_t;
                }
            }
            Some(death_t) if death_t > target_t => {
                self.death_t = None;
                self.t = death_t;
                self.integrate(metric, target_t - death_t);
                if self.alive() {
                    self.t = target_t;
                }
            }
            Some(_) => self.t = target_t,
        }
    }

    /// The one integrator behind `step` and `step_back`: substepped RK4 of `ray_rhs` over a signed
    /// interval, leaving (r, phi, v^r, v^phi, t) at the event that interval ends on.
    ///
    /// The substep caps are read off |dt|, so a backward step is subdivided exactly as finely as a
    /// forward step of the same length. Only a forward step retires a ray at R_STOP or `R_ESCAPE`;
    /// a ray running backwards is retracing ground it has already covered inside the field, and
    /// applying the test there would only re-kill a ray at the boundary it is climbing away from.
    fn integrate(&mut self, metric: &KerrSchild, dt: f64) {
        let forward = dt > 0.0;
        let t0 = self.t;
        let mut y: RayState = [self.r, self.phi, self.dr_dt, self.dphi_dt];
        let k1 = ray_rhs(metric, &y);
        let n = substep_count(&y, &k1, dt.abs());
        let h = dt / n as f64;

        let mut k = k1;
        for i in 0..n {
            if i > 0 {
                k = ray_rhs(metric, &y);
            }
            let next = ray_rk4(metric, &y, &k, h);
            let leaves = forward && (next[0] < R_STOP || next[0] > R_ESCAPE);
            if leaves || !next.iter().all(|c| c.is_finite()) {
                // The ray is left standing at the last event *inside* the field rather than at the
                // first one outside it. That is what makes the death reversible: `ray_rhs` clamps
                // r to R_STOP, so its value below the ring is not the equation the ray came in on,
                // and a state kept out there could not be integrated back through. Kept one
                // substep short, the state is one the forward pass actually visited, and the
                // backward RK4 retraces from it with nothing but the scheme's own error.
                self.commit(&y, t0 + (i as f64) * h);
                self.death_t = Some(self.t);
                return;
            }
            y = next;
        }
        self.commit(&y, t0 + dt);
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
fn ray_rhs(metric: &KerrSchild, y: &RayState) -> RayState {
    let r = y[0].max(R_STOP);
    let v = [1.0, y[2], y[3]];
    let acc = geodesic_accel(metric, r, &v);
    [y[2], y[3], acc[1] - y[2] * acc[0], acc[2] - y[3] * acc[0]]
}

/// One classical RK4 step of `ray_rhs`, reusing the slope k1 already evaluated at y.
fn ray_rk4(metric: &KerrSchild, y: &RayState, k1: &RayState, h: f64) -> RayState {
    let advance = |y: &RayState, k: &RayState, s: f64| -> RayState {
        let mut out = *y;
        for i in 0..out.len() {
            out[i] += s * k[i];
        }
        out
    };
    let k2 = ray_rhs(metric, &advance(y, k1, 0.5 * h));
    let k3 = ray_rhs(metric, &advance(y, &k2, 0.5 * h));
    let k4 = ray_rhs(metric, &advance(y, &k3, h));

    let mut out = *y;
    for i in 0..out.len() {
        out[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    out
}

/// Substeps needed to hold both the radial and the directional caps over a step of dt.
fn substep_count(y: &RayState, k1: &RayState, dt: f64) -> usize {
    let radial = (y[2].abs() * dt / MAX_DR_PER_SUBSTEP).ceil();
    let scale = 1.0f64.max(y[2].abs()).max(y[3].abs());
    let rate = k1[2].abs().max(k1[3].abs());
    let turning = if rate > 0.0 {
        (rate * dt / (DIRECTION_STEP_FRACTION * scale)).ceil()
    } else {
        1.0
    };
    let n = radial.max(turning).max(1.0).min(MAX_SUBSTEPS as f64);
    (n as usize).max(1)
}

/// One RK4 step of the closed-form outgoing principal null direction
/// dr/dt = Delta / (r^2 + a^2 + 2Mr), substepped on the same radial cap as a full ray.
///
/// This is a one-dimensional problem because `KerrSchild::radial_null_slopes` gives dr/dt as an
/// explicit function of r alone: the outgoing PND congruence is a solution of the geodesic equation
/// in closed form, so no direction has to be carried. That makes it the cheap representative of a
/// pulse in the (t, r) diagram, and the exact shape of the outgoing light stack of Feature B.
fn pnd_advance(metric: &KerrSchild, r: f64, dt: f64) -> f64 {
    let slope = |r: f64| metric.radial_null_slopes(r.max(R_STOP)).dr_dt_outgoing;
    let n = ((slope(r).abs() * dt / MAX_DR_PER_SUBSTEP).ceil().max(1.0))
        .min(MAX_SUBSTEPS as f64) as usize;
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

/// The master outgoing principal null ray of the interior: the (t, r) curve that peels off r+ and
/// falls onto r-.
///
/// Because the geometry is stationary, every outgoing PND inside r+ is this one curve translated in
/// t, so one integration serves the whole congruence. It starts just inside r+ (at r+(1 - 1e-3),
/// which is where the exponential departure from the horizon at rate kappa_+ has already begun) and
/// runs until it is within 1e-4 of r- or 400M of coordinate time have passed.
///
/// The curve never reaches r-: r - r- decays like exp(-kappa_- t). That accumulation is the point.
/// The Cauchy horizon in this chart is the surface on which the outgoing light of the whole
/// interior piles up, and an infalling worldline crosses the entire pile in finite proper time.
pub fn outgoing_ray_track(metric: &KerrSchild) -> Vec<(f64, f64)> {
    let rp = metric.outer_horizon();
    let rm = metric.inner_horizon();
    let mut r = rp * (1.0 - 1e-3);
    let mut t = 0.0;
    let mut out = vec![(t, r)];
    while out.len() < TRACK_MAX_POINTS && t < 400.0 && r - rm > 1e-4 && r > R_STOP {
        r = pnd_advance(metric, r, TRACK_DT);
        t += TRACK_DT;
        out.push((t, r));
    }
    out
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

/// One crossing of Bob's worldline by one sheet of one of Alice's wavefronts.
///
/// A pulse reaches Bob more than once. Its crossing family sweeps past him first, while he is still
/// well above r-, and arrives redshifted; the same pulse's frozen family is still standing on r-
/// when he gets there, and he cuts through that stack in the last fraction of an M of his fall,
/// with the blueshift that the Cauchy horizon is famous for. Both are recorded.
#[allow(dead_code)] // the record is complete on purpose: the HUD reads the ratio, the (t, r)
// diagram the event, and the proper time is what makes a reception quotable on Bob's own clock
#[derive(Debug, Clone, Copy)]
pub struct Reception {
    /// Serial number of the pulse this arrival belongs to.
    pub pulse_index: usize,
    /// Coordinate time of the reception event.
    pub t: f64,
    /// Bob's proper time at reception.
    pub tau_bob: f64,
    /// Bob's radius at reception.
    pub r: f64,
    /// nu(Bob) / nu(Alice at emission) for the ray that reached him.
    pub ratio: f64,
    /// Whether the receiving ray belongs to the frozen family, E - Omega_- L < 0, which never
    /// crosses r- and piles onto it, rather than to the crossing family that passes straight
    /// through. Decided by `NullRay::inner_horizon_energy` on whichever of the two bracketing rays
    /// is nearer to Bob's azimuth.
    pub frozen_family: bool,
}

/// One emission event of Alice's, and the wavefront it launched.
#[allow(dead_code)] // the emission event is recorded in full: the drawing needs the rays and the
// track, and the tests need the event itself to check the kappa_- law pulse by pulse
#[derive(Debug, Clone)]
pub struct Pulse {
    /// Serial number of the pulse, counting from the first one Alice sent.
    pub index: usize,
    /// Coordinate time of the emission event.
    pub emitted_t: f64,
    /// Alice's proper time at emission.
    pub emitted_tau: f64,
    /// Radius of the emission event.
    pub emitted_r: f64,
    /// Azimuth of the emission event.
    pub emitted_phi: f64,
    /// The whole of Alice's light cone at emission, `RAYS_PER_PULSE` exact null geodesics ordered
    /// by their emission angle alpha, from alpha = 0, her own outward radial leg, round to
    /// alpha = 2 pi - 2 pi / `RAYS_PER_PULSE`. She broadcasts in every direction, so the polyline
    /// these rays form is closed: the last ray joins back to the first.
    pub rays: Vec<NullRay>,
    /// The outgoing principal null ray from the emission event, as (t, r) pairs: the pulse's
    /// representative in the (t, r) diagram, where a fan of azimuths cannot be drawn.
    pub pnd_track: Vec<(f64, f64)>,
    /// One entry per sheet of the front that stood across Bob's azimuth on the previous detection
    /// pass, carrying the side he was on. A sheet is keyed by the segment of the ray polyline and
    /// the turn of azimuth it crosses him on, so folds and windings are tracked independently
    /// instead of collapsing into one number, and a sheet that stops straddling him simply drops
    /// out of the list.
    sheets: Vec<SheetSide>,
    /// Every crossing of Bob's worldline by this pulse, in the order he met them.
    pub receptions: Vec<Reception>,
}

/// Which side of one sheet of a wavefront Bob was on at the previous detection pass.
#[derive(Debug, Clone, Copy)]
struct SheetSide {
    /// Index of the polyline segment (the ray pair) carrying this sheet. It is the whole key: each
    /// unwrapped step is folded into [-pi, pi], so a segment can straddle at most one of the angles
    /// 2 pi n that represent Bob, and naming the winding as well would only make the key fragile.
    /// The folded azimuth of the first ray jumps by a full turn whenever it passes the fold, which
    /// shifts every unwrapped angle and every winding number by one without anything having moved.
    segment: usize,
    /// bob.r - r_front for this sheet, as both stood at that pass. Storing the side rather than
    /// r_front alone is what makes Bob's own motion count: inside r+ the front has all but stopped
    /// and it is Bob who does the crossing.
    side: f64,
}

impl Pulse {
    /// Extend the pulse's principal-null track by dt, unless it has already joined r- or grown to
    /// its cap.
    fn extend_track(&mut self, metric: &KerrSchild, dt: f64) {
        let rm = metric.inner_horizon();
        let Some(&(t, r)) = self.pnd_track.last() else {
            return;
        };
        if self.pnd_track.len() >= TRACK_MAX_POINTS
            || r > R_ESCAPE
            || r <= R_STOP
            || (r - rm).abs() < TRACK_R_MINUS_EPS
        {
            return;
        }
        self.pnd_track.push((t + dt, pnd_advance(metric, r, dt)));
    }

    /// Record every crossing of Bob's worldline by this wavefront on this pass.
    ///
    /// The rays of a pulse are a closed polyline in the (r, phi) plane, ordered by their emission
    /// angle: Alice broadcasts into her whole light cone, so the last ray joins back to the first
    /// and that closing segment is a sheet like any other. Bob is located on the polyline through
    /// the *unwrapped* azimuth of each ray relative to his: the first ray is placed within pi of Bob
    /// and every later one within pi of its predecessor, so a front that frame dragging has wound
    /// through several turns is still one continuous curve. On that unwrapped axis Bob is not one
    /// angle but the whole family 0, +/-2 pi, +/-4 pi, ..., because a front that has wound one turn
    /// further passes over him again. Each polyline segment straddling one of those angles is one
    /// *sheet* of the front standing across his azimuth, and since every unwrapped step is folded
    /// into [-pi, pi] a segment can straddle at most one of them, so the segment index alone names
    /// the sheet.
    ///
    /// A sheet gives r_front by linear interpolation along the segment, and the shift by the same
    /// linear interpolation of the two bracketing rays' own frequency ratios, each evaluated at its
    /// own event against Bob's 4-velocity. Interpolating the finished ratios rather than the raw
    /// factors keeps the answer between two numbers that are both exact measurements, which matters
    /// once the ratios span orders of magnitude. The family the arrival belongs to is read off the
    /// nearer of the two bracketing rays with `NullRay::inner_horizon_energy`.
    ///
    /// A reception is a sign change of bob.r - r_front for one sheet between two consecutive passes
    /// on which that same sheet stood across him. Tracking sheets separately rather than reducing
    /// the front to a single representative radius is what makes both arrivals of a pulse show up:
    /// the crossing family sweeping past Bob high above r-, and the frozen family waiting on r- for
    /// him to fall through it. A sheet whose rays have reached the ring is retired with them, since
    /// only segments with both ends still alive can be interpolated; that sheet simply stops being
    /// tracked, and no crossing is invented for it.
    fn detect(&mut self, metric: &KerrSchild, bob: &Observer, u_bob: &[f64; 3]) {
        if self.rays.len() < 2 {
            self.sheets.clear();
            return;
        }
        let two_pi = 2.0 * std::f64::consts::PI;
        let wrap = |d: f64| d - two_pi * (d / two_pi).round();
        let mut rel = Vec::with_capacity(self.rays.len());
        rel.push(wrap(self.rays[0].phi - bob.phi));
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
                let side = bob.r - r_front;

                let was = self.sheets.iter().find(|s| s.segment == i).map(|s| s.side);
                if let Some(prev) = was
                    && prev * side < 0.0
                {
                    let f0 = self.rays[i].frequency_ratio(metric, u_bob);
                    let f1 = self.rays[j].frequency_ratio(metric, u_bob);
                    let ratio = f0 + w * (f1 - f0);
                    if ratio.is_finite() && ratio > 0.0 {
                        let nearer = if w < 0.5 { i } else { j };
                        self.receptions.push(Reception {
                            pulse_index: self.index,
                            t: bob.t,
                            tau_bob: bob.tau,
                            r: bob.r,
                            ratio,
                            frozen_family: self.rays[nearer].frozen(metric),
                        });
                    }
                }
                sheets.push(SheetSide { segment: i, side });
            }
        }
        self.sheets = sheets;
    }
}

/// The 4-velocity the receiving observer measures with.
///
/// A released observer carries their own, whatever worldline they are on. Before release the
/// observer is held at a fixed radius, and `Observer::step` already advances their clock as
/// dtau = sqrt(-g_tt) dt while they wait, which is the static observer's proper time; the frame
/// that matches that clock is the normalised time-translation Killing vector u = (1, 0, 0) /
/// sqrt(-g_tt), so that is the frame their pre-release measurements are quoted in. No static
/// observer exists at or inside the static limit, where g_tt >= 0, and there the free-fall value
/// is used instead.
fn measuring_four_velocity(metric: &KerrSchild, observer: &Observer) -> [f64; 3] {
    if !observer.is_active {
        let g_tt = metric.metric_components(observer.r)[0][0];
        if g_tt < 0.0 {
            return [1.0 / (-g_tt).sqrt(), 0.0, 0.0];
        }
    }
    observer.four_velocity(metric)
}

/// Every pulse Alice currently has in flight.
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
    /// Alice's proper time at the last emission, or None before she has sent anything.
    last_emit_tau: Option<f64>,
    /// Alice's proper-time interval between pulses.
    pub interval_tau: f64,
}

impl Default for SignalField {
    fn default() -> Self {
        Self {
            pulses: Vec::new(),
            t: 0.0,
            next_index: 0,
            last_emit_tau: None,
            interval_tau: EMISSION_INTERVAL_TAU,
        }
    }
}

impl SignalField {
    /// Emit a pulse if Alice's own clock says one is due: at her release, and every
    /// `interval_tau` of her proper time after that. A stalled or retired worldline sends nothing,
    /// since a worldline that is no longer advancing has no proper time to space pulses by.
    pub fn emit_if_due(&mut self, metric: &KerrSchild, alice: &Observer) {
        if !alice.is_active || alice.r <= R_STOP {
            return;
        }
        if let Some(geo) = alice.geodesic
            && geo.stalled
        {
            return;
        }
        if let Some(last) = self.last_emit_tau
            && alice.tau < last + self.interval_tau
        {
            return;
        }

        let tetrad = alice.tetrad(metric);
        let u = alice.four_velocity(metric);
        let two_pi = 2.0 * std::f64::consts::PI;
        let rays = (0..RAYS_PER_PULSE)
            .map(|i| {
                // alpha = 0 is her outward radial leg and the ray count divides the turn exactly,
                // so the last ray stops one step short of alpha = 2 pi and the front closes.
                let alpha = two_pi * (i as f64) / (RAYS_PER_PULSE as f64);
                NullRay::from_local_direction(
                    metric, alice.t, alice.r, alice.phi, &tetrad, alpha, &u,
                )
            })
            .collect();

        self.pulses.push(Pulse {
            index: self.next_index,
            emitted_t: alice.t,
            emitted_tau: alice.tau,
            emitted_r: alice.r,
            emitted_phi: alice.phi,
            rays,
            pnd_track: vec![(alice.t, alice.r)],
            sheets: Vec::new(),
            receptions: Vec::new(),
        });
        self.next_index += 1;
        self.last_emit_tau = Some(alice.tau);
        while self.pulses.len() > MAX_PULSES {
            self.pulses.remove(0);
        }
    }

    /// Advance every live ray and every principal-null track by dt of coordinate time.
    ///
    /// `NullRay::step` carries a dead ray forward on the clock without moving it, so every ray of
    /// the field reads the same t as the field itself and `step_back` can ask each of them the one
    /// question that matters: was this ray still alive dt ago?
    pub fn advance(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.t += dt;
        for pulse in self.pulses.iter_mut() {
            for ray in pulse.rays.iter_mut() {
                ray.step(metric, dt);
            }
            pulse.extend_track(metric, dt);
        }
    }

    /// Carry the whole field back by dt of coordinate time, the way `Observer::step_back` carries a
    /// worldline back, so that stepping the simulation backwards undoes the transmission instead of
    /// deleting it.
    ///
    /// The target time is dt before the latest event in the field, and everything else follows from
    /// it. A pulse emitted after the target was never sent and goes. Every ray of a pulse that
    /// survives is integrated back to the target, reviving if it died inside the interval. A
    /// principal-null track is truncated to the points it had reached by then, never below its own
    /// emission point. A reception recorded after the target is unrecorded.
    ///
    /// The per-sheet bookkeeping of `Pulse::detect` is dropped rather than rewound, so the next
    /// forward pass re-establishes which side of each sheet Bob stands on before it can call
    /// anything a crossing. Rewinding a side is meaningless, and keeping the stale one would invent
    /// a sign change out of the rewind itself.
    ///
    /// `last_emit_tau` falls back to the newest surviving pulse, so Alice resumes on the same
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
        for pulse in self.pulses.iter_mut() {
            for ray in pulse.rays.iter_mut() {
                ray.step_back(metric, dt);
            }
            let keep = pulse
                .pnd_track
                .iter()
                .take_while(|(t, _)| *t <= target_t + 1e-9)
                .count()
                .max(1);
            pulse.pnd_track.truncate(keep);
            pulse.receptions.retain(|rec| rec.t <= target_t);
            pulse.sheets.clear();
        }
        self.last_emit_tau = self.pulses.iter().map(|p| p.emitted_tau).reduce(f64::max);
        self.t = target_t;
    }

    /// Record every crossing of Bob's worldline by every live wavefront on this pass.
    ///
    /// Outside r+ a front overtakes Bob from below as it climbs outward; inside r+ everything falls
    /// and it is Bob who overtakes a front that has all but stopped against r-. The per-sheet sign
    /// change of `Pulse::detect` catches both, and catches them for each sheet of a folded front
    /// separately, which is why a single pulse can be received more than once.
    pub fn detect_receptions(&mut self, metric: &KerrSchild, bob: &Observer) {
        let u_bob = measuring_four_velocity(metric, bob);
        for pulse in self.pulses.iter_mut() {
            pulse.detect(metric, bob, &u_bob);
        }
    }

    /// Every recorded crossing of Bob's worldline, from every pulse still in the field.
    pub fn receptions(&self) -> impl Iterator<Item = &Reception> {
        self.pulses.iter().flat_map(|p| p.receptions.iter())
    }

    /// The reception with the latest coordinate time. Arrivals do not come in emission order: the
    /// frozen family of an early pulse can reach Bob long after the crossing family of a late one,
    /// so they have to be compared by their own event time.
    pub fn last_reception(&self) -> Option<&Reception> {
        self.receptions().max_by(|a, b| a.t.total_cmp(&b.t))
    }

    /// The largest shift Bob has measured so far, over all arrivals.
    pub fn max_ratio(&self) -> Option<f64> {
        self.receptions().map(|r| r.ratio).reduce(f64::max)
    }

    /// How many arrivals Bob has recorded in total, counting a pulse once per sheet of it that has
    /// swept over him rather than once per pulse.
    pub fn received_count(&self) -> usize {
        self.pulses.iter().map(|p| p.receptions.len()).sum()
    }

    /// Drop every pulse and put the clock back to zero. This is the reset, not the rewind: it is
    /// for re-dropping the observers and for changing the geometry under them, where the standing
    /// wavefronts are null geodesics of a metric that no longer applies. Stepping the simulation
    /// backwards uses `step_back` instead, which keeps the transmission.
    pub fn clear(&mut self) {
        self.pulses.clear();
        self.t = 0.0;
        self.last_emit_tau = None;
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
        // them is pure integration error.
        let metric = KerrSchild::new(1.0, 0.65);
        let r0 = 3.0;
        let u = raindrop(&metric, r0);
        let tetrad = Tetrad::from_four_velocity(&metric, r0, &u);
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
                assert!(
                    n.abs() < 1e-8,
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
                    assert!(
                        (l - l_over_e0).abs() < 1e-7 * (1.0 + l_over_e0.abs()),
                        "L/E = {l} vs {l_over_e0} at r={} on alpha={alpha}",
                        ray.r
                    );
                }
            }
        }
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
    fn test_outgoing_ray_track_peels_off_r_plus_and_lands_on_r_minus() {
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let track = outgoing_ray_track(&metric);
        assert!(track.len() > 10, "track is too short: {}", track.len());
        assert!((track[0].1 - rp * (1.0 - 1e-3)).abs() < 1e-12);
        // Monotonically inward, and it stops just above r- rather than crossing it.
        for w in track.windows(2) {
            assert!(w[1].1 < w[0].1, "the interior PND must fall: {:?} -> {:?}", w[0], w[1]);
            assert!(w[1].1 > rm, "it must not cross r-: {:?}", w[1]);
        }
        let last = *track.last().unwrap();
        assert!(last.1 - rm < 1e-4, "it should end on r-: r - r- = {}", last.1 - rm);
        assert!(last.0 < 400.0, "and get there in finite t: t = {}", last.0);
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
        // `ray_rhs` is a first-order autonomous system and it is time symmetric, so RK4 with a
        // negative step integrates the same null geodesic in the other direction rather than some
        // other curve. What is left over is the scheme's own asymmetry: the substep count is read
        // off the state a step begins at, and on the way back that is the far end of the interval,
        // so the two directions do not subdivide identically.
        //
        // Measured over a whole light cone at r = 3 and again in Region II at r = 1, the eight
        // directions split into two groups. The ones that stay out in the open come back to
        // between 5e-16 and 2e-9 of where they started, which is round-off. The ones that run into
        // the stiff last decade of radius above the ring, some of them reaching it and being
        // revived on the way out, come back to between 7e-8 and 8.4e-6: a substep of a given size
        // is simply not the same instrument in both directions where the connection is that steep.
        // 1e-5 is the tolerance that covers the worst of those and 1e-6 is not, the worst measured
        // being 8.4e-6 at alpha = 270 degrees from r = 3.
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
                    err < 1e-5,
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
        // 3.3e-7 in the worst of the four carried numbers.
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
        // The state kept is the last event inside the field, one substep short of the boundary,
        // which is why the ray is left just outside R_STOP rather than just inside it.
        assert!(
            (R_STOP..R_STOP + MAX_DR_PER_SUBSTEP).contains(&ray.r),
            "with its state kept at the ring: r = {}",
            ray.r
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
        assert!(err < 1e-5, "revival error {err}: {ray:?} vs {start:?}");
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

        // Nothing is emitted before release, one pulse at release, and the next only after
        // `interval_tau` of Alice's proper time.
        let mut waiting = Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 3.0, 0.25, params);
        waiting.step(&metric, 0.5, 0.5);
        field.emit_if_due(&metric, &waiting);
        assert_eq!(field.pulses.len(), 0, "an unreleased Alice sends nothing");

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
}
