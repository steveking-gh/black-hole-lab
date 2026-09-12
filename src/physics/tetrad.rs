use crate::physics::kerr_schild::KerrSchild;

/// Local orthonormal tetrad frame e_{(a)}^mu at radius r on the equatorial plane.
/// This connects the observer's local Minkowski frame (where light travels isotropically at c=1)
/// to the global Kerr-Schild coordinate frame.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct Tetrad {
    /// Zero-Angular-Momentum / coordinate frame tetrad vectors [e_{(0)}, e_{(1)}, e_{(2)}]
    /// Each is a 3-vector [v^t, v^r, v^phi]
    pub e0: [f64; 3], // Timelike basis vector (future-directed)
    pub e1: [f64; 3], // Radial spacelike basis vector
    pub e2: [f64; 3], // Azimuthal spacelike basis vector
}

impl Tetrad {
    /// Construct the local orthonormal tetrad for an observer at radius r in Kerr-Schild coordinates.
    pub fn new(metric: &KerrSchild, r: f64) -> Self {
        let r = r.max(1e-4);
        let slopes = metric.radial_null_slopes(r);
        let omega = slopes.dphi_dt_drag;

        // In Kerr-Schild, the stationary / dragging observer has 4-velocity:
        // u^mu ~ (1, 0, omega)
        // Normalization: g_{mu nu} u^mu u^nu = g_{tt} + 2 omega g_{t phi} + omega^2 g_{phi phi}
        let g = metric.metric_components(r);
        let norm_sq = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma_stat = if norm_sq > 1e-7 {
            1.0 / norm_sq.sqrt()
        } else {
            // Inside ergosphere/horizon where static observer cannot exist, use infalling baseline
            1.0
        };

        let e0 = [gamma_stat, 0.0, gamma_stat * omega];

        // Radial spacelike basis vector:
        // Orthogonal to e0, pointing along +r
        let g_rr = g[1][1].max(1.0);
        let e1_scale = 1.0 / g_rr.sqrt();
        let e1 = [0.0, e1_scale, 0.0];

        // Azimuthal spacelike basis vector:
        let g_pp = g[2][2].max(1e-4);
        let e2_scale = 1.0 / g_pp.sqrt();
        let e2 = [0.0, 0.0, e2_scale];

        Self { e0, e1, e2 }
    }

    /// Boost a local null direction vector (1, cos_alpha, sin_alpha) by observer local velocity
    /// beta = (beta_r, beta_phi) where |beta| < 1, and return the coordinate velocities (dr/dt, dphi/dt).
    pub fn local_to_coordinate_velocity(
        &self,
        metric: &KerrSchild,
        r: f64,
        beta_r: f64,
        beta_phi: f64,
        alpha: f64,
    ) -> (f64, f64) {
        // Clamp beta to physical subluminal speeds |beta| < 0.99
        let beta_sq = (beta_r * beta_r + beta_phi * beta_phi).min(0.98);
        let gamma = 1.0 / (1.0 - beta_sq).sqrt();

        // Local photon 3-velocity in observer's rest frame: direction alpha
        let n_r = alpha.cos();
        let n_phi = alpha.sin();

        // Standard Lorentz aberration / boost:
        // v'_local = [ (n + beta*(gamma/(gamma+1)*(n.beta) + 1)) ] / [ gamma * (1 + beta.n) ]
        let n_dot_beta = n_r * beta_r + n_phi * beta_phi;
        let denom = 1.0 + n_dot_beta;
        let boost_factor = if beta_sq > 1e-9 {
            (gamma - 1.0) / beta_sq
        } else {
            0.5
        };

        let v_loc_r = (n_r + gamma * beta_r + boost_factor * n_dot_beta * beta_r) / (gamma * denom.max(1e-6));
        let v_loc_phi = (n_phi + gamma * beta_phi + boost_factor * n_dot_beta * beta_phi) / (gamma * denom.max(1e-6));

        // Now map from local tetrad frame to coordinate velocities
        // dr/dt and dphi/dt are bounded by the metric's null cone
        let slopes = metric.radial_null_slopes(r);
        let v_ingoing = slopes.dr_dt_ingoing;
        let v_outgoing = slopes.dr_dt_outgoing;

        // Interpolate dr/dt based on local boosted radial direction
        let w = 0.5 * (1.0 + v_loc_r.clamp(-1.0, 1.0));
        let dr_dt = (1.0 - w) * v_ingoing + w * v_outgoing;

        // Azimuthal coordinate velocity incorporates frame dragging + local boosted transverse speed
        let omega = slopes.dphi_dt_drag;
        let dphi_dt = omega + (v_loc_phi / r.max(1e-3)) * 0.7;

        (dr_dt, dphi_dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tetrad_radial_boost() {
        let metric = KerrSchild::new(1.0, 0.5);
        let r = 3.0;
        let tetrad = Tetrad::new(&metric, r);

        // At rest
        let (dr_rest_out, _) = tetrad.local_to_coordinate_velocity(&metric, r, 0.0, 0.0, 0.0);
        let (dr_rest_in, _) = tetrad.local_to_coordinate_velocity(&metric, r, 0.0, 0.0, std::f64::consts::PI);

        // Inward boost beta_r < 0 should shift light cone inward
        let (dr_boost_out, _) = tetrad.local_to_coordinate_velocity(&metric, r, -0.8, 0.0, 0.0);
        assert!(dr_boost_out < dr_rest_out);

        // Ingoing ray remains inward
        assert!(dr_rest_in < 0.0);
    }
}
