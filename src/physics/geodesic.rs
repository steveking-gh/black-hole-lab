use crate::physics::kerr_schild::KerrSchild;

/// Infalling geodesic state and numerical integrator for observers (Alice, Bob, etc.)
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
    /// Energy parameter E (normalized to rest mass, E = 1 for drop from rest at infinity)
    pub energy: f64,
    /// Angular momentum parameter L (zero for radial infall)
    pub l_ang: f64,
}

/// Radius at which integration stops (ring singularity at r = 0 on the equator).
pub const R_STOP: f64 = 0.02;
const R_FLOOR: f64 = 0.01;

impl GeodesicState {
    pub fn new_infall(start_t: f64, start_r: f64, energy: f64, l_ang: f64) -> Self {
        Self {
            t: start_t,
            r: start_r,
            phi: 0.0,
            tau: 0.0,
            energy,
            l_ang,
        }
    }

    fn normalize_phi(&mut self) {
        let two_pi = 2.0 * std::f64::consts::PI;
        self.phi = self.phi.rem_euclid(two_pi);
    }

    /// Step the geodesic forward by proper time step dtau using 4th-order Runge-Kutta (RK4).
    /// Negative dtau integrates backward along the same worldline.
    pub fn step(&mut self, metric: &KerrSchild, dtau: f64) {
        if self.r <= R_STOP && dtau > 0.0 {
            return;
        }

        let (k1_t, k1_r, k1_p) = self.derivatives(metric, self.r);
        let r_mid1 = (self.r + 0.5 * dtau * k1_r).max(R_FLOOR);
        let (k2_t, k2_r, k2_p) = self.derivatives(metric, r_mid1);
        let r_mid2 = (self.r + 0.5 * dtau * k2_r).max(R_FLOOR);
        let (k3_t, k3_r, k3_p) = self.derivatives(metric, r_mid2);
        let r_end = (self.r + dtau * k3_r).max(R_FLOOR);
        let (k4_t, k4_r, k4_p) = self.derivatives(metric, r_end);

        self.r = (self.r + (dtau / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r)).max(R_FLOOR);
        self.t += (dtau / 6.0) * (k1_t + 2.0 * k2_t + 2.0 * k3_t + k4_t);
        self.phi += (dtau / 6.0) * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
        self.tau += dtau;
        self.normalize_phi();
    }

    /// Coordinate-time rates (dr/dt, dphi/dt, dtau/dt) at radius r.
    fn coord_rates(&self, metric: &KerrSchild, r: f64) -> (f64, f64, f64) {
        let (dt_dtau, dr_dtau, dphi_dtau) = self.derivatives(metric, r);
        let inv = 1.0 / dt_dtau.max(1e-9);
        (dr_dtau * inv, dphi_dtau * inv, inv)
    }

    /// Advance geodesic by coordinate time dt using RK4 on the state (r, phi, tau) with t as the
    /// independent variable. Sub-steps are capped so the integration stays accurate near r = 0.
    pub fn step_coord_time(&mut self, metric: &KerrSchild, dt: f64) {
        if self.r <= R_STOP || dt <= 0.0 {
            return;
        }
        let mut remaining = dt;
        let mut guard = 0;
        while remaining > 1e-12 && guard < 10_000 {
            guard += 1;
            let h = remaining.min(0.05).min((0.25 * self.r).max(1e-3));

            let (k1_r, k1_p, k1_tau) = self.coord_rates(metric, self.r);
            let r2 = (self.r + 0.5 * h * k1_r).max(R_FLOOR);
            let (k2_r, k2_p, k2_tau) = self.coord_rates(metric, r2);
            let r3 = (self.r + 0.5 * h * k2_r).max(R_FLOOR);
            let (k3_r, k3_p, k3_tau) = self.coord_rates(metric, r3);
            let r4 = (self.r + h * k3_r).max(R_FLOOR);
            let (k4_r, k4_p, k4_tau) = self.coord_rates(metric, r4);

            self.r = (self.r + (h / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r)).max(R_FLOOR);
            self.phi += (h / 6.0) * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
            self.tau += (h / 6.0) * (k1_tau + 2.0 * k2_tau + 2.0 * k3_tau + k4_tau);
            self.t += h;
            remaining -= h;

            if self.r <= R_STOP {
                break;
            }
        }
        self.normalize_phi();
    }

    /// Exact geodesic 4-velocity components (dt/dtau, dr/dtau, dphi/dtau) in ingoing
    /// Kerr-Schild coordinates (t, r, phi) on the equatorial plane, for an ingoing timelike
    /// geodesic with conserved energy E and axial angular momentum L.
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
            let geo = GeodesicState::new_infall(0.0, 6.0, e, l);
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
        let geo = GeodesicState::new_infall(0.0, 6.0, 1.0, 0.0);
        for &r in &[10.0, 4.0, 2.0, 1.0, 0.3] {
            let x = (2.0_f64 / r).sqrt();
            let expected = (1.0 + x + x * x) / (1.0 + x);
            let (ut, ur, _) = geo.derivatives(&metric, r);
            assert!((ut - expected).abs() < 1e-10, "r={r}: {ut} vs {expected}");
            assert!((ur + x).abs() < 1e-10);
        }
    }

    #[test]
    fn test_geodesic_infall() {
        let metric = KerrSchild::new(1.0, 0.6);
        let mut geo = GeodesicState::new_infall(0.0, 4.0, 1.0, 0.0);

        let initial_r = geo.r;
        for _ in 0..50 {
            geo.step(&metric, 0.05);
        }
        assert!(geo.r < initial_r);
        assert!(geo.tau > 0.0);
        assert!(geo.t > 0.0);
    }

    #[test]
    fn test_coord_time_and_proper_time_steppers_agree() {
        let metric = KerrSchild::new(1.0, 0.65);
        let mut a = GeodesicState::new_infall(0.0, 4.5, 1.0, 0.0);
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
}
