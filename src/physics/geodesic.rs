use crate::physics::kerr_schild::KerrSchild;

/// Infalling geodesic state and numerical integrator for observers (Alice, Bob, etc.)
#[derive(Debug, Clone, Copy)]
pub struct GeodesicState {
    /// Coordinate time t
    pub t: f64,
    /// Radial coordinate r
    pub r: f64,
    /// Azimuthal angle phi
    pub phi: f64,
    /// Proper time tau accumulated by the observer
    pub tau: f64,
    /// Energy parameter E (normalized to rest mass, E = 1 for drop from rest at infinity)
    pub energy: f64,
    /// Angular momentum parameter L (zero for radial infall)
    pub l_ang: f64,
}

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

    /// Step the geodesic forward by proper time step dtau using 4th-order Runge-Kutta (RK4)
    /// or adaptive Euler step across Kerr-Schild metric.
    pub fn step(&mut self, metric: &KerrSchild, dtau: f64) {
        if self.r <= 0.02 {
            // Reached near physical singularity ring
            return;
        }

        // Compute derivatives at current state
        let (dt_dtau, dr_dtau, dphi_dtau) = self.derivatives(metric, self.r);

        // RK4 integration for precision and stability
        let k1_r = dr_dtau;
        let k1_t = dt_dtau;
        let k1_p = dphi_dtau;

        let r_mid1 = (self.r + 0.5 * dtau * k1_r).max(0.01);
        let (k2_t, k2_r, k2_p) = self.derivatives(metric, r_mid1);

        let r_mid2 = (self.r + 0.5 * dtau * k2_r).max(0.01);
        let (k3_t, k3_r, k3_p) = self.derivatives(metric, r_mid2);

        let r_end = (self.r + dtau * k3_r).max(0.01);
        let (k4_t, k4_r, k4_p) = self.derivatives(metric, r_end);

        self.r = (self.r + (dtau / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r)).max(0.01);
        self.t += (dtau / 6.0) * (k1_t + 2.0 * k2_t + 2.0 * k3_t + k4_t);
        self.phi += (dtau / 6.0) * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
        self.tau += dtau;

        // Keep phi normalized in [0, 2*pi)
        while self.phi >= 2.0 * std::f64::consts::PI {
            self.phi -= 2.0 * std::f64::consts::PI;
        }
        while self.phi < 0.0 {
            self.phi += 2.0 * std::f64::consts::PI;
        }
    }

    /// Advance geodesic by coordinate time dt (integrating coordinate velocities dr/dt and dphi/dt).
    pub fn step_coord_time(&mut self, metric: &KerrSchild, dt: f64) {
        if self.r <= 0.02 {
            return;
        }
        let dt_step = 0.05;
        let mut t_acc = 0.0;
        while t_acc < dt {
            let step = (dt - t_acc).min(dt_step);
            let (dt_dtau, dr_dtau, dphi_dtau) = self.derivatives(metric, self.r);
            let dt_dtau = dt_dtau.max(0.01);
            let dr_dt = dr_dtau / dt_dtau;
            let dphi_dt = dphi_dtau / dt_dtau;
            let dtau = step / dt_dtau;

            self.r = (self.r + dr_dt * step).max(0.01);
            self.phi += dphi_dt * step;
            self.t += step;
            self.tau += dtau;
            t_acc += step;
            if self.r <= 0.02 {
                break;
            }
        }

        while self.phi >= 2.0 * std::f64::consts::PI {
            self.phi -= 2.0 * std::f64::consts::PI;
        }
        while self.phi < 0.0 {
            self.phi += 2.0 * std::f64::consts::PI;
        }
    }

    /// Geodesic derivatives (dt/dtau, dr/dtau, dphi/dtau) on the equatorial plane
    /// for infall with conserved energy E and angular momentum L.
    pub(crate) fn derivatives(&self, metric: &KerrSchild, r: f64) -> (f64, f64, f64) {
        let r = r.max(0.01);
        let m = metric.m;
        let a = metric.a;
        let a2 = a * a;
        let r2 = r * r;

        // Effective potential in equatorial Kerr:
        // (dr/dtau)^2 = E^2 - 1 + 2M/r - (L^2 - a^2(E^2 - 1))/r^2 + 2M(L - aE)^2 / r^3
        let term1 = self.energy * self.energy - 1.0;
        let term2 = 2.0 * m / r;
        let term3 = -(self.l_ang * self.l_ang - a2 * term1) / r2;
        let diff = self.l_ang - a * self.energy;
        let term4 = (2.0 * m * diff * diff) / (r2 * r);

        let v_sq = (term1 + term2 + term3 + term4).max(0.0001);
        // Infalling direction: dr/dtau is negative
        let dr_dtau = -v_sq.sqrt();

        // In Kerr-Schild coordinates, the coordinate dt/dtau remains positive and finite across horizons:
        let h = metric.h_scalar(r);
        let sqrt_2h = (2.0 * h).max(0.0).sqrt();
        let dt_dtau = if self.energy > 0.0 {
            let u_t_radial = self.energy + (2.0 * h * self.energy) / (1.0 + sqrt_2h).max(0.1);
            let rot_corr = if a2 > 1e-6 {
                (a * self.l_ang * h) / (r2 + a2)
            } else {
                0.0
            };
            (u_t_radial + rot_corr).max(0.1)
        } else {
            1.0
        };

        // Frame dragging angular velocity contributes to dphi/dtau:
        let omega = metric.frame_dragging_omega(r);
        let dphi_dtau = omega * dt_dtau + self.l_ang / (r2 + a2).max(1e-4);

        (dt_dtau, dr_dtau, dphi_dtau)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geodesic_infall() {
        let metric = KerrSchild::new(1.0, 0.6);
        let mut geo = GeodesicState::new_infall(0.0, 4.0, 1.0, 0.0);

        let initial_r = geo.r;
        for _ in 0..50 {
            geo.step(&metric, 0.05);
        }

        // Observer must have moved inward
        assert!(geo.r < initial_r);
        // Proper time tau and coordinate time t must have advanced positively
        assert!(geo.tau > 0.0);
        assert!(geo.t > 0.0);
    }
}
