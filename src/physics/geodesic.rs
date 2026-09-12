use crate::physics::kerr_schild::KerrSchild;

/// Geodesic state and numerical integrator for observers (Alice, Bob, etc.).
///
/// The worldline is advanced by integrating the geodesic equation itself,
///     du^mu/dlambda = -Gamma^mu_{alpha beta} u^alpha u^beta,
/// on the state y = (t, r, phi, tau, u^t, u^r, u^phi). The 4-velocity is therefore a *state
/// variable*, not a function of r: no branch of a square root is ever selected, so a worldline
/// with a turning point (dr/dtau = 0) simply turns around, and E = -u_t, L = u_phi and
/// g(u, u) = -1 come out as conserved diagnostics rather than being imposed at every step.
#[derive(Debug, Clone, Copy)]
pub struct GeodesicState {
    /// Coordinate time t (ingoing Kerr-Schild)
    pub t: f64,
    /// Radial coordinate r
    pub r: f64,
    /// Azimuthal angle phi (ingoing Kerr-Schild azimuth, regular across both horizons)
    pub phi: f64,
    /// Proper time tau accumulated by the observer
    pub tau: f64,
    /// Energy parameter E = -u_t (normalized to rest mass, E = 1 for drop from rest at infinity)
    pub energy: f64,
    /// Angular momentum parameter L = u_phi (zero for radial infall)
    pub l_ang: f64,
    /// Contravariant 4-velocity u^mu = (u^t, u^r, u^phi) at the current event.
    pub u: [f64; 3],
    /// Set when the integrator has abandoned the worldline: either u^t ran away past
    /// `U_T_STALL` (an outgoing worldline reaching r-, which this chart cannot follow: see
    /// `new_with_direction`), or a step produced a non-finite value. A stalled state stops
    /// advancing instead of emitting NaNs into the rest of the app.
    pub stalled: bool,
}

/// Radius at which integration stops (ring singularity at r = 0 on the equator).
pub const R_STOP: f64 = 0.02;
const R_FLOOR: f64 = 0.01;

/// u^t past which the worldline is declared frozen. An outgoing worldline approaching r- from
/// below has t -> infinity at finite proper time, so u^t = dt/dtau diverges there: the ingoing
/// Kerr-Schild chart simply does not cover the outward crossing.
const U_T_STALL: f64 = 1e6;

/// Fractional change of the 4-velocity allowed per coordinate-time substep. Together with the
/// fixed and radius-proportional caps this is what holds E, L and g(u, u) to better than 1e-8
/// over a whole infall (see the tests).
const U_STEP_FRACTION: f64 = 0.004;

/// Integration state vector y = (t, r, phi, tau, u^t, u^r, u^phi).
type StateVec = [f64; 7];

/// The geodesic acceleration -Gamma^mu_{alpha beta} u^alpha u^beta at radius r.
pub(crate) fn geodesic_accel(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> [f64; 3] {
    let gamma = metric.christoffel(r);
    let mut acc = [0.0f64; 3];
    for (mu, a) in acc.iter_mut().enumerate() {
        let mut sum = 0.0;
        for alpha in 0..3 {
            for beta in 0..3 {
                sum += gamma[mu][alpha][beta] * u[alpha] * u[beta];
            }
        }
        *a = -sum;
    }
    acc
}

/// Right-hand side with proper time as the independent variable:
///     dt/dtau = u^t,  dr/dtau = u^r,  dphi/dtau = u^phi,  dtau/dtau = 1,
///     du^mu/dtau = -Gamma^mu_{alpha beta} u^alpha u^beta.
fn rhs_proper_time(metric: &KerrSchild, y: &StateVec) -> StateVec {
    let r = y[1].max(R_FLOOR);
    let u = [y[4], y[5], y[6]];
    let a = geodesic_accel(metric, r, &u);
    [u[0], u[1], u[2], 1.0, a[0], a[1], a[2]]
}

/// Right-hand side with coordinate time as the independent variable. Dividing the proper-time
/// system by u^t = dt/dtau gives
///     dr/dt = u^r/u^t,  dphi/dt = u^phi/u^t,  dtau/dt = 1/u^t,
///     du^mu/dt = -Gamma^mu_{alpha beta} u^alpha u^beta / u^t.
/// u^t > 0 for every future-directed timelike worldline in this chart, so the division is safe;
/// the floor below only guards a state that has already gone bad.
fn rhs_coord_time(metric: &KerrSchild, y: &StateVec) -> StateVec {
    let r = y[1].max(R_FLOOR);
    let u = [y[4], y[5], y[6]];
    let inv = 1.0 / y[4].max(1e-9);
    let a = geodesic_accel(metric, r, &u);
    [1.0, u[1] * inv, u[2] * inv, inv, a[0] * inv, a[1] * inv, a[2] * inv]
}

/// Largest substep for which the 4-velocity changes by less than `U_STEP_FRACTION` of its own
/// size, given the slope k1 already evaluated at the current state.
///
/// A cap tied to r alone is not enough close to the ring: on the equator u^mu ~ 1/r^2 and
/// Gamma^mu_{alpha beta} ~ 1/r^3, so the geodesic equation stiffens by orders of magnitude over
/// the last decade of radius, and it stiffens again wherever u^t runs away (an approach to r-
/// that this chart cannot follow). Outside a few tenths of M this cap never binds.
fn velocity_step_cap(k1: &StateVec, u: &[f64; 3]) -> f64 {
    let scale = u.iter().fold(1.0f64, |m, v| m.max(v.abs()));
    let rate = k1[4].abs().max(k1[5].abs()).max(k1[6].abs());
    if rate > 0.0 {
        U_STEP_FRACTION * scale / rate
    } else {
        f64::INFINITY
    }
}

/// One classical RK4 step of `rhs` with step size h, reusing the slope `k1` at `y`.
fn rk4(
    metric: &KerrSchild,
    y: &StateVec,
    k1: &StateVec,
    h: f64,
    rhs: fn(&KerrSchild, &StateVec) -> StateVec,
) -> StateVec {
    let advance = |y: &StateVec, k: &StateVec, s: f64| -> StateVec {
        let mut out = *y;
        for i in 0..out.len() {
            out[i] += s * k[i];
        }
        out
    };
    let k2 = rhs(metric, &advance(y, k1, 0.5 * h));
    let k3 = rhs(metric, &advance(y, &k2, 0.5 * h));
    let k4 = rhs(metric, &advance(y, &k3, h));

    let mut out = *y;
    for i in 0..out.len() {
        out[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    out
}

impl GeodesicState {
    /// Ingoing geodesic through (start_t, start_r) with conserved (E, L).
    pub fn new_infall(
        metric: &KerrSchild,
        start_t: f64,
        start_r: f64,
        energy: f64,
        l_ang: f64,
    ) -> Self {
        Self::new_with_direction(metric, start_t, start_r, energy, l_ang, false)
    }

    /// Geodesic through (start_t, start_r) with conserved (E, L), started on the ingoing
    /// (`outgoing = false`) or outgoing (`outgoing = true`) root of r^4 (dr/dtau)^2 = R(r).
    ///
    /// If R(start_r) < 0 the point lies in a region forbidden to that (E, L) - no timelike
    /// geodesic with those constants passes through it - so E is raised to `energy_floor`, the
    /// smallest energy for which R(start_r) >= 0. The worldline then starts exactly at a turning
    /// point, which the second-order integrator handles without any special case.
    ///
    /// The outgoing root is built from the direct Boyer-Lindquist forms transformed to ingoing
    /// Kerr-Schild (dt_KS = dt_BL + (2Mr/Delta) dr, dphi_KS = dphi_BL + (a/Delta) dr):
    ///     r^2 u^t   = [(r^2 + a^2) P + 2 M r sqrt(R)] / Delta + a (L - aE)
    ///     r^2 u^phi = (L - aE) + a (P + sqrt(R)) / Delta
    ///     u^r       = +sqrt(R) / r^2
    /// Unlike the ingoing root, whose 1/Delta poles cancel identically (see `derivatives`), these
    /// genuinely diverge at Delta = 0, and that is physics rather than a coordinate artefact: an
    /// outgoing worldline cannot cross a horizon outward in an ingoing chart. Within
    /// |Delta| < 1e-6 (or if R is still negative) the outgoing start is refused and the ingoing
    /// root is used instead.
    pub fn new_with_direction(
        metric: &KerrSchild,
        start_t: f64,
        start_r: f64,
        energy: f64,
        l_ang: f64,
        outgoing: bool,
    ) -> Self {
        let r = start_r.max(R_FLOOR);
        let energy = if Self::radial_potential(metric, r, energy, l_ang) < 0.0 {
            Self::energy_floor(metric, r, l_ang)
        } else {
            energy
        };
        let mut state = Self {
            t: start_t,
            r,
            phi: 0.0,
            tau: 0.0,
            energy,
            l_ang,
            u: [1.0, 0.0, 0.0],
            stalled: false,
        };
        state.u = state.branch_four_velocity_at(metric, r, outgoing);
        state
    }

    /// R(r) = P^2 - Delta [r^2 + (L - aE)^2] = r^4 (dr/dtau)^2, with P = E (r^2 + a^2) - a L.
    /// Negative R marks a region no timelike geodesic with these constants can reach.
    pub(crate) fn radial_potential(metric: &KerrSchild, r: f64, energy: f64, l_ang: f64) -> f64 {
        let r = r.max(R_FLOOR);
        let a = metric.a;
        let r2 = r * r;
        let a2 = a * a;
        let lae = l_ang - a * energy;
        let p = energy * (r2 + a2) - a * l_ang;
        p * p - metric.delta(r) * (r2 + lae * lae)
    }

    /// Smallest energy E for which R(r) >= 0, i.e. the effective potential V(r, L). R is the
    /// upward parabola in E
    ///     R(E) = [(r^2 + a^2)^2 - a^2 Delta] E^2 - 4 M a L r E + a^2 L^2 - Delta (r^2 + L^2),
    /// so the floor is its larger root; for a = 0 that is the familiar
    /// sqrt((1 - 2M/r)(1 + L^2/r^2)). Where the parabola has no real root - everywhere inside a
    /// horizon, since Delta < 0 makes R > 0 for every E - there is nothing to clamp and 0 is
    /// returned.
    pub fn energy_floor(metric: &KerrSchild, r: f64, l_ang: f64) -> f64 {
        let r = r.max(R_FLOOR);
        let a = metric.a;
        let m = metric.m;
        let r2 = r * r;
        let a2 = a * a;
        let delta = metric.delta(r);

        let quad_a = (r2 + a2) * (r2 + a2) - a2 * delta;
        let quad_b = -4.0 * m * a * l_ang * r;
        let quad_c = a2 * l_ang * l_ang - delta * (r2 + l_ang * l_ang);

        let disc = quad_b * quad_b - 4.0 * quad_a * quad_c;
        if disc <= 0.0 || quad_a <= 0.0 {
            return 0.0;
        }
        ((-quad_b + disc.sqrt()) / (2.0 * quad_a)).max(0.0)
    }

    /// The 4-velocity of the ingoing or outgoing root of this worldline's (E, L) at radius r.
    /// See `new_with_direction` for the outgoing expressions and why they are refused at
    /// Delta = 0. Being a function of r alone, this is also what the observer code differentiates
    /// to check that the integrated worldline is weightless.
    pub(crate) fn branch_four_velocity_at(
        &self,
        metric: &KerrSchild,
        r: f64,
        outgoing: bool,
    ) -> [f64; 3] {
        let ingoing = || {
            let (ut, ur, up) = self.derivatives(metric, r);
            [ut, ur, up]
        };
        if !outgoing {
            return ingoing();
        }

        let r = r.max(R_FLOOR);
        let m = metric.m;
        let a = metric.a;
        let r2 = r * r;
        let a2 = a * a;
        let delta = metric.delta(r);
        let big_r = Self::radial_potential(metric, r, self.energy, self.l_ang);
        if delta.abs() < 1e-6 || big_r < 0.0 {
            return ingoing();
        }

        let lae = self.l_ang - a * self.energy;
        let p = self.energy * (r2 + a2) - a * self.l_ang;
        let sqrt_r = big_r.sqrt();

        let ut = (((r2 + a2) * p + 2.0 * m * r * sqrt_r) / delta + a * lae) / r2;
        let ur = sqrt_r / r2;
        let up = (lae + a * (p + sqrt_r) / delta) / r2;
        [ut, ur, up]
    }

    /// Re-derive the 4-velocity from the conserved (E, L) at radius r, on whichever root matches
    /// the current sign of u^r. Used when a worldline is rewound to a recorded event, where only
    /// (t, r, phi) were stored.
    pub fn reseed_at(&mut self, metric: &KerrSchild, r: f64) {
        let outgoing = self.u[1] > 0.0;
        self.r = r.max(R_FLOOR);
        self.u = self.branch_four_velocity_at(metric, self.r, outgoing);
    }

    fn normalize_phi(&mut self) {
        let two_pi = 2.0 * std::f64::consts::PI;
        self.phi = self.phi.rem_euclid(two_pi);
    }

    fn as_state(&self) -> StateVec {
        [self.t, self.r, self.phi, self.tau, self.u[0], self.u[1], self.u[2]]
    }

    /// Commit an integrated state vector. Returns false (and freezes the worldline) when the
    /// step produced something the chart cannot represent: a non-finite component, or a runaway
    /// u^t at an outward horizon crossing.
    fn commit(&mut self, y: &StateVec) -> bool {
        if y.iter().any(|v| !v.is_finite()) {
            self.stalled = true;
            return false;
        }
        self.t = y[0];
        self.r = y[1].max(R_FLOOR);
        self.phi = y[2];
        self.tau = y[3];
        self.u = [y[4], y[5], y[6]];
        if self.u[0] > U_T_STALL {
            self.stalled = true;
            return false;
        }
        true
    }

    /// Step the geodesic forward by proper time step dtau using 4th-order Runge-Kutta (RK4) on
    /// the second-order system. Negative dtau integrates backward along the same worldline.
    pub fn step(&mut self, metric: &KerrSchild, dtau: f64) {
        if self.stalled || (self.r <= R_STOP && dtau > 0.0) {
            return;
        }
        let y = self.as_state();
        let k1 = rhs_proper_time(metric, &y);
        let y = rk4(metric, &y, &k1, dtau, rhs_proper_time);
        self.commit(&y);
        self.normalize_phi();
    }

    /// Advance the geodesic by coordinate time dt using RK4 with t as the independent variable.
    ///
    /// Each substep is capped at 0.05 M, at a small fraction of r, and at the step on which the
    /// 4-velocity itself changes by `U_STEP_FRACTION` (`velocity_step_cap`); the last two are
    /// what hold the conserved quantities to 1e-8 through the strong field, where a cap on
    /// coordinate time alone is far too coarse. A whole infall from r = 6 to the ring costs
    /// under a millisecond, so the accuracy is free at UI rates.
    pub fn step_coord_time(&mut self, metric: &KerrSchild, dt: f64) {
        if self.stalled || self.r <= R_STOP || dt <= 0.0 {
            return;
        }
        let mut remaining = dt;
        let mut guard = 0;
        while remaining > 1e-12 && guard < 10_000 {
            guard += 1;
            let y = self.as_state();
            let k1 = rhs_coord_time(metric, &y);
            let h = remaining.min(
                0.05f64
                    .min((0.008 * self.r).max(1e-4))
                    .min(velocity_step_cap(&k1, &self.u))
                    .max(1e-7),
            );

            let y = rk4(metric, &y, &k1, h, rhs_coord_time);
            if !self.commit(&y) {
                break;
            }
            remaining -= h;

            if self.r <= R_STOP {
                break;
            }
        }
        self.normalize_phi();
    }

    /// Exact geodesic 4-velocity components (dt/dtau, dr/dtau, dphi/dtau) in ingoing
    /// Kerr-Schild coordinates (t, r, phi) on the equatorial plane, for an *ingoing* timelike
    /// geodesic with conserved energy E and axial angular momentum L.
    ///
    /// This closed form seeds `u` when a worldline is created and is the independent check the
    /// integrated worldline is measured against; the worldline itself is advanced by `step` /
    /// `step_coord_time`, which take no root and can therefore turn around.
    ///
    /// Starting from the Boyer-Lindquist first integrals (Sigma = r^2 on the equator)
    ///     P      = E (r^2 + a^2) - a L
    ///     R      = P^2 - Delta [ r^2 + (L - aE)^2 ]           (= r^4 (dr/dtau)^2)
    ///     r^2 t' = (r^2 + a^2) P / Delta + a (L - aE)
    ///     r^2 f' = a P / Delta + (L - aE)
    /// and the chart change  dt_KS = dt_BL + (2Mr/Delta) dr,  dphi_KS = dphi_BL + (a/Delta) dr,
    /// the 1/Delta poles cancel exactly for ingoing motion (dr/dtau = -sqrt(R)/r^2).
    /// Rationalising gives the manifestly regular forms used below:
    ///     r^2 t'_KS = [P^2 (r^2 + a^2 + 2Mr) + 4M^2 r^2 (r^2 + (L-aE)^2)] / [(r^2+a^2) P + 2Mr sqrt(R)] + a (L - aE)
    ///     r^2 f'_KS = (L - aE) + a (r^2 + (L-aE)^2) / (P + sqrt(R))
    /// These are finite and smooth through r+ and r- (Delta never appears).
    pub(crate) fn derivatives(&self, metric: &KerrSchild, r: f64) -> (f64, f64, f64) {
        let r = r.max(R_FLOOR);
        let m = metric.m;
        let a = metric.a;
        let e = self.energy;
        let l = self.l_ang;
        let r2 = r * r;
        let a2 = a * a;
        let delta = metric.delta(r);

        let lae = l - a * e;
        let p = e * (r2 + a2) - a * l;
        let q = r2 + lae * lae;
        let big_r = (p * p - delta * q).max(0.0);
        let sqrt_r = big_r.sqrt();

        // Ingoing branch: dr/dtau <= 0
        let dr_dtau = -sqrt_r / r2;

        let denom_t = (r2 + a2) * p + 2.0 * m * r * sqrt_r;
        let dt_dtau = if denom_t.abs() > 1e-12 {
            ((p * p * (r2 + a2 + 2.0 * m * r) + 4.0 * m * m * r2 * q) / denom_t + a * lae) / r2
        } else {
            // Degenerate case (P <= 0 at a horizon): fall back to the direct expression.
            (((r2 + a2) * p - 2.0 * m * r * sqrt_r) / delta + a * lae) / r2
        };

        let denom_p = p + sqrt_r;
        let dphi_dtau = if denom_p.abs() > 1e-12 {
            (lae + a * q / denom_p) / r2
        } else {
            ((p - sqrt_r) * a / delta + lae) / r2
        };

        (dt_dtau, dr_dtau, dphi_dtau)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// (g(u,u), E = -u_t, L = u_phi) of a 4-velocity at radius r.
    fn invariants(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> (f64, f64, f64) {
        let g = metric.metric_components(r);
        let mut u_low = [0.0f64; 3];
        for (mu, low) in u_low.iter_mut().enumerate() {
            *low = (0..3).map(|nu| g[mu][nu] * u[nu]).sum();
        }
        let norm: f64 = (0..3).map(|mu| u_low[mu] * u[mu]).sum();
        (norm, -u_low[0], u_low[2])
    }

    /// Reference integrator kept deliberately independent of the production one: a dense RK4 of
    /// the *first-order* ingoing system dr/dt = u^r/u^t with (u^t, u^r) taken from the closed
    /// form `derivatives` at each radius. This is what the module did before the second-order
    /// rewrite, so it pins the new integrator to the old behaviour on ingoing worldlines.
    fn reference_r_of_t(
        metric: &KerrSchild,
        geo: &GeodesicState,
        start_r: f64,
        t_end: f64,
        steps: usize,
    ) -> f64 {
        let h = t_end / steps as f64;
        let rate = |r: f64| {
            let (dt_dtau, dr_dtau, _) = geo.derivatives(metric, r);
            dr_dtau / dt_dtau
        };
        let mut r = start_r;
        for _ in 0..steps {
            let k1 = rate(r);
            let k2 = rate(r + 0.5 * h * k1);
            let k3 = rate(r + 0.5 * h * k2);
            let k4 = rate(r + h * k3);
            r += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
        r
    }

    /// Every root of R(r) = 0 in [lo, hi], found by scanning for sign changes and bisecting.
    fn radial_roots(metric: &KerrSchild, energy: f64, l_ang: f64, lo: f64, hi: f64) -> Vec<f64> {
        let n = 20_000;
        let step = (hi - lo) / n as f64;
        let big_r = |r: f64| GeodesicState::radial_potential(metric, r, energy, l_ang);
        let mut roots = Vec::new();
        let mut a = lo;
        let mut fa = big_r(a);
        for i in 1..=n {
            let b = lo + step * i as f64;
            let fb = big_r(b);
            if fa == 0.0 {
                roots.push(a);
            } else if fa * fb < 0.0 {
                let (mut x0, mut x1) = (a, b);
                for _ in 0..200 {
                    let mid = 0.5 * (x0 + x1);
                    if big_r(x0) * big_r(mid) <= 0.0 {
                        x1 = mid;
                    } else {
                        x0 = mid;
                    }
                }
                roots.push(0.5 * (x0 + x1));
            }
            a = b;
            fa = fb;
        }
        roots
    }

    fn norm(metric: &KerrSchild, geo: &GeodesicState, r: f64) -> f64 {
        let (ut, ur, up) = geo.derivatives(metric, r);
        let g = metric.metric_components(r);
        let u = [ut, ur, up];
        let mut s = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                s += g[i][j] * u[i] * u[j];
            }
        }
        s
    }

    #[test]
    fn test_four_velocity_normalisation_across_horizons() {
        // u.u must equal -1 everywhere, including at and inside both horizons.
        for &(a, e, l) in &[(0.0, 1.0, 0.0), (0.65, 1.0, 0.0), (0.9, 1.2, 0.5), (0.998, 1.0, -1.0)] {
            let metric = KerrSchild::new(1.0, a);
            let geo = GeodesicState::new_infall(&metric, 0.0, 6.0, e, l);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[8.0, 3.0, 2.0, rp, 0.5 * (rp + rm), rm.max(0.05), 0.5 * rm.max(0.05), 0.05] {
                let n = norm(&metric, &geo, r);
                let (ut, _, _) = geo.derivatives(&metric, r);
                assert!(ut > 0.0, "dt/dtau must be future-directed at r={r} (a={a}): {ut}");
                assert!((n + 1.0).abs() < 1e-8, "u.u = {n} at r={r} (a={a}, E={e}, L={l})");
            }
        }
    }

    #[test]
    fn test_schwarzschild_radial_closed_form() {
        // Schwarzschild, E=1, L=0: dt_KS/dtau = (1 + x + x^2)/(1 + x), x = sqrt(2M/r)
        let metric = KerrSchild::new(1.0, 0.0);
        let geo = GeodesicState::new_infall(&metric, 0.0, 6.0, 1.0, 0.0);
        for &r in &[10.0, 4.0, 2.0, 1.0, 0.3] {
            let x = (2.0_f64 / r).sqrt();
            let expected = (1.0 + x + x * x) / (1.0 + x);
            let (ut, ur, _) = geo.derivatives(&metric, r);
            assert!((ut - expected).abs() < 1e-10, "r={r}: {ut} vs {expected}");
            assert!((ur + x).abs() < 1e-10);
        }
        // The state's own 4-velocity is seeded from the same closed form.
        let x0 = (2.0_f64 / 6.0).sqrt();
        assert!((geo.u[0] - (1.0 + x0 + x0 * x0) / (1.0 + x0)).abs() < 1e-12);
        assert!((geo.u[1] + x0).abs() < 1e-12);
    }

    #[test]
    fn test_geodesic_infall() {
        let metric = KerrSchild::new(1.0, 0.6);
        let mut geo = GeodesicState::new_infall(&metric, 0.0, 4.0, 1.0, 0.0);

        let initial_r = geo.r;
        for _ in 0..50 {
            geo.step(&metric, 0.05);
        }
        assert!(geo.r < initial_r);
        assert!(geo.tau > 0.0);
        assert!(geo.t > 0.0);
        assert!(!geo.stalled);
    }

    #[test]
    fn test_coord_time_and_proper_time_steppers_agree() {
        let metric = KerrSchild::new(1.0, 0.65);
        let mut a = GeodesicState::new_infall(&metric, 0.0, 4.5, 1.0, 0.0);
        let mut b = a;
        // Integrate A in coordinate time up to t = 3, then B in proper time up to the same tau.
        a.step_coord_time(&metric, 3.0);
        let n = 3000;
        let dtau = a.tau / n as f64;
        for _ in 0..n {
            b.step(&metric, dtau);
        }
        assert!((a.t - b.t).abs() < 1e-4, "t: {} vs {}", a.t, b.t);
        assert!((a.r - b.r).abs() < 1e-4, "r: {} vs {}", a.r, b.r);
        assert!((a.phi - b.phi).abs() < 1e-4, "phi: {} vs {}", a.phi, b.phi);
    }

    #[test]
    fn test_conserved_quantities_along_a_full_infall() {
        // The integrator imposes nothing: g(u,u) = -1, E = -u_t and L = u_phi are conserved by
        // the geodesic equation alone, from r = 6 all the way down.
        //
        // Three of these four worldlines end on the ring. The fourth, (a, E, L) = (0.9, 1.1, 2.5),
        // has E - Omega_- L < 0 with Omega_- = a / (2 M r-): a particle whose angular momentum is
        // that far above its energy cannot cross the inner horizon at all. Its P = E(r^2+a^2) - aL
        // turns negative inside r+, dt/dtau runs away as r -> r-, and the worldline freezes there
        // in coordinate time - the same chart limit as an outgoing crossing, met from the other
        // side. The integrator must reproduce that instead of pushing through.
        for &(a, e, l) in &[(0.0, 1.0, 0.0), (0.65, 1.0, 0.0), (0.9, 1.1, 2.5), (0.998, 1.0, -1.0)] {
            let metric = KerrSchild::new(1.0, a);
            let mut geo = GeodesicState::new_infall(&metric, 0.0, 6.0, e, l);
            let (n0, e0, l0) = invariants(&metric, geo.r, &geo.u);
            assert!((n0 + 1.0).abs() < 1e-12, "seed u.u = {n0} (a={a})");
            assert!((e0 - e).abs() < 1e-12 && (l0 - l).abs() < 1e-12, "seed (E, L) = ({e0}, {l0})");

            let mut guard = 0;
            while geo.r > R_STOP && !geo.stalled && guard < 10_000 {
                guard += 1;
                geo.step_coord_time(&metric, 0.25);
                if geo.r <= R_STOP {
                    break;
                }
                let (n, ee, ll) = invariants(&metric, geo.r, &geo.u);
                assert!((ee - e).abs() < 1e-8, "E = {ee} vs {e} at r={} (a={a})", geo.r);
                assert!((ll - l).abs() < 1e-8, "L = {ll} vs {l} at r={} (a={a})", geo.r);
                // g(u,u) is a difference of terms of size |u|^2, so double precision alone can
                // only deliver 1e-16 |u|^2; that floor matters solely on the freeze, where u^t
                // climbs through six decades.
                let scale = geo.u.iter().fold(1.0f64, |m, v| m.max(v.abs()));
                let tol = 1e-8 + 1e-14 * scale * scale;
                assert!(
                    (n + 1.0).abs() < tol,
                    "u.u = {n} at r={} u^t={} (a={a}, E={e}, L={l})",
                    geo.r,
                    geo.u[0]
                );
            }

            let rm = metric.inner_horizon();
            let omega_minus = if rm > 1e-9 { a / (2.0 * metric.m * rm) } else { 0.0 };
            if e - omega_minus * l > 0.0 {
                assert!(geo.r <= R_STOP, "the infall must reach the ring: r = {} (a={a})", geo.r);
                assert!(!geo.stalled, "a plain infall must not stall (a={a})");
            } else {
                assert!(geo.stalled, "the worldline must freeze at r- (a={a}, E={e}, L={l})");
                assert!(
                    (geo.r - rm).abs() < 1e-3,
                    "it must freeze *at* r- = {rm}, got r = {} (a={a})",
                    geo.r
                );
            }
        }
    }

    #[test]
    fn test_integrated_radius_matches_the_first_order_ingoing_stepper() {
        // The second-order integrator must reproduce, to 1e-6, the r(t) the old first-order
        // ingoing stepper produced from the closed-form `derivatives`.
        for &(a, e, l) in &[(0.0, 1.0, 0.0), (0.65, 1.0, 0.0), (0.9, 1.1, 2.5)] {
            let metric = KerrSchild::new(1.0, a);
            let start_r = 6.0;
            let mut geo = GeodesicState::new_infall(&metric, 0.0, start_r, e, l);
            for &t in &[1.0, 2.0, 3.0] {
                geo.step_coord_time(&metric, 1.0);
                let expected = reference_r_of_t(&metric, &geo, start_r, t, 60_000);
                assert!(
                    (geo.r - expected).abs() < 1e-6,
                    "r({t}) = {} vs reference {expected} (a={a}, E={e}, L={l})",
                    geo.r
                );
                assert!((geo.t - t).abs() < 1e-12, "coordinate time must advance exactly");
            }
        }
    }

    #[test]
    fn test_energy_floor_clamps_a_forbidden_start() {
        // Schwarzschild: the floor is the effective potential V = sqrt((1-2M/r)(1+L^2/r^2)).
        let schw = KerrSchild::new(1.0, 0.0);
        for &(r, l) in &[(4.5, 4.0), (10.0, 3.0), (6.0, 0.0)] {
            let floor = GeodesicState::energy_floor(&schw, r, l);
            let expected = ((1.0 - 2.0 / r) * (1.0 + l * l / (r * r))).sqrt();
            assert!((floor - expected).abs() < 1e-12, "V({r}, {l}) = {floor} vs {expected}");
            assert!(GeodesicState::radial_potential(&schw, r, floor, l).abs() < 1e-8);
        }

        // A start below the floor is raised, and lands exactly on a turning point.
        let metric = KerrSchild::new(1.0, 0.9);
        let floor = GeodesicState::energy_floor(&metric, 4.5, 3.5);
        assert!(floor > 0.90, "r = 4.5M with L = 3.5M is forbidden below E = {floor}");
        let geo = GeodesicState::new_infall(&metric, 0.0, 4.5, 0.90, 3.5);
        assert!((geo.energy - floor).abs() < 1e-12, "E = {} vs floor {floor}", geo.energy);
        assert!(GeodesicState::radial_potential(&metric, 4.5, geo.energy, 3.5).abs() < 1e-8);
        assert!((metric.norm(4.5, &geo.u) + 1.0).abs() < 1e-9, "clamped seed must be unit timelike");
        assert!(geo.u[1].abs() < 1e-6, "a clamped start sits at a turning point: {:?}", geo.u);

        // An allowed start is left alone...
        let free = GeodesicState::new_infall(&metric, 0.0, 4.5, 1.2, 3.5);
        assert_eq!(free.energy, 1.2);
        // ...and inside a horizon R > 0 for every E, so there is no floor at all.
        let mid = 0.5 * (metric.outer_horizon() + metric.inner_horizon());
        assert_eq!(GeodesicState::energy_floor(&metric, mid, 2.0), 0.0);
    }

    /// Integrate a bound orbit and report (smallest r seen, largest r seen, sign changes of u^r).
    fn survey_bound_orbit(
        metric: &KerrSchild,
        geo: &mut GeodesicState,
        t_end: f64,
        r_min: f64,
        r_max: f64,
    ) -> (f64, f64, usize) {
        let mut lo = geo.r;
        let mut hi = geo.r;
        let mut changes = 0usize;
        let mut prev = geo.u[1];
        let mut t = 0.0;
        while t < t_end {
            geo.step_coord_time(metric, 0.5);
            t += 0.5;
            assert!(!geo.stalled, "a bound orbit must never stall (r = {})", geo.r);
            assert!(
                geo.r >= r_min - 1e-4 && geo.r <= r_max + 1e-4,
                "r = {} escaped [{r_min}, {r_max}] at t = {t}",
                geo.r
            );
            lo = lo.min(geo.r);
            hi = hi.max(geo.r);
            if prev * geo.u[1] < 0.0 {
                changes += 1;
            }
            if geo.u[1] != 0.0 {
                prev = geo.u[1];
            }
        }
        (lo, hi, changes)
    }

    #[test]
    fn test_schwarzschild_bound_orbit_turns_at_both_ends() {
        // E = 0.97, L = 4.0 is bound: R(r) > 0 only between the two outer roots, and the
        // second-order integrator oscillates between them instead of plunging.
        let metric = KerrSchild::new(1.0, 0.0);
        let (e, l) = (0.97, 4.0);
        let roots = radial_roots(&metric, e, l, 0.05, 60.0);
        assert!(roots.len() >= 2, "expected a bound band, got {roots:?}");
        let r_max = roots[roots.len() - 1];
        let r_min = roots[roots.len() - 2];
        assert!(r_min > 6.0 && r_max > r_min + 1.0, "bound band [{r_min}, {r_max}]");

        let mut geo = GeodesicState::new_infall(&metric, 0.0, r_max, e, l);
        assert!((geo.energy - e).abs() < 1e-9, "apastron start must not move E: {}", geo.energy);
        // The radial period of this orbit is close to 500 M, so 400 M would only reach
        // perihelion; 600 M covers a full turn and back.
        let (lo, hi, changes) = survey_bound_orbit(&metric, &mut geo, 600.0, r_min, r_max);
        assert!((lo - r_min).abs() < 1e-3, "perihelion reached {lo}, want {r_min}");
        assert!((hi - r_max).abs() < 1e-3, "apastron reached {hi}, want {r_max}");
        assert!(changes >= 2, "u^r must change sign at least twice, got {changes}");
    }

    #[test]
    fn test_kerr_bound_orbit_turns_at_both_ends() {
        let metric = KerrSchild::new(1.0, 0.9);
        let (e, l) = (0.95, 2.8);
        let roots = radial_roots(&metric, e, l, 0.05, 60.0);
        if roots.len() < 2 {
            // No bound band for this (a, E, L): nothing to check.
            return;
        }
        let r_max = roots[roots.len() - 1];
        let r_min = roots[roots.len() - 2];
        if r_max - r_min < 0.5 || r_min < metric.outer_horizon() {
            return;
        }

        let mut geo = GeodesicState::new_infall(&metric, 0.0, r_max, e, l);
        let (lo, hi, changes) = survey_bound_orbit(&metric, &mut geo, 400.0, r_min, r_max);
        assert!((lo - r_min).abs() < 1e-3, "perihelion reached {lo}, want {r_min}");
        assert!((hi - r_max).abs() < 1e-3, "apastron reached {hi}, want {r_max}");
        assert!(changes >= 2, "u^r must change sign at least twice, got {changes}");
    }

    #[test]
    fn test_outgoing_start_in_region_iii_freezes_at_the_cauchy_horizon() {
        // The ingoing chart cannot follow an outgoing crossing of r-: the worldline freezes
        // there as t -> infinity. The integrator must show exactly that - r increasing towards
        // r-, u^t running away - without panicking or emitting NaNs.
        let metric = KerrSchild::new(1.0, 0.9);
        let rm = metric.inner_horizon();
        let start_r = 0.5 * rm;

        let mut geo = GeodesicState::new_with_direction(&metric, 0.0, start_r, 1.0, 0.0, true);
        assert!(geo.u[1] > 0.0, "must start outgoing: {:?}", geo.u);
        assert!((metric.norm(start_r, &geo.u) + 1.0).abs() < 1e-10, "outgoing seed must be unit");

        let mut prev_r = geo.r;
        let mut t = 0.0;
        while t < 200.0 && !geo.stalled {
            geo.step_coord_time(&metric, 0.5);
            t += 0.5;
            assert!(geo.r.is_finite() && geo.u.iter().all(|v| v.is_finite()), "NaN at t = {t}");
            assert!(geo.r >= prev_r - 1e-12, "r must increase: {} -> {}", prev_r, geo.r);
            assert!(geo.r < rm + 1e-9, "the outward crossing of r- is not in this chart: {}", geo.r);
            prev_r = geo.r;
        }
        assert!(geo.stalled, "u^t must run away at r-, got u^t = {}", geo.u[0]);
        assert!(geo.u[0] > 1e5, "u^t = {}", geo.u[0]);
        assert!(geo.r > start_r && (rm - geo.r) < 1e-3, "frozen at r = {} (r- = {rm})", geo.r);

        // On a horizon the outgoing forms are singular, so the outgoing start is refused.
        let refused = GeodesicState::new_with_direction(&metric, 0.0, rm, 1.0, 0.0, true);
        assert!(refused.u[1] < 0.0, "an outgoing start at r- must fall back to ingoing: {:?}", refused.u);
    }
}
