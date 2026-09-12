use serde::{Deserialize, Serialize};

/// Kerr-Schild metric in the equatorial plane (theta = pi/2).
/// In Kerr-Schild coordinates (t, r, phi), the metric remains smooth, finite,
/// and nonsingular across both the outer event horizon (r+) and inner Cauchy horizon (r-).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KerrSchild {
    /// Black hole mass M in geometric units (typically normalized to 1.0 for foliation).
    pub m: f64,
    /// Physical mass in Solar Masses (M_sun), e.g. 4.15e6 for Sgr A*, 6.6e10 for TON 618.
    pub m_solar: f64,
    /// Spin parameter a in [0, M). (Angular momentum J = M * a).
    pub a: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct NullSlopes {
    /// Coordinate velocity dr/dt of the ingoing null ray.
    pub dr_dt_ingoing: f64,
    /// Coordinate velocity dr/dt of the outgoing null ray.
    pub dr_dt_outgoing: f64,
    /// Frame-dragging angular coordinate velocity dphi/dt.
    pub dphi_dt_drag: f64,
}

#[allow(dead_code)]
impl KerrSchild {
    pub fn new(m: f64, a: f64) -> Self {
        Self::with_solar_mass(m, a, 10.0)
    }

    pub fn with_solar_mass(m: f64, a: f64, m_solar: f64) -> Self {
        let m = m.max(0.01);
        let max_a = m * 0.9999;
        let a = a.clamp(-max_a, max_a);
        Self {
            m,
            m_solar: m_solar.max(0.1),
            a,
        }
    }

    /// Dimensionless spin parameter a* = a / M in [-0.9999, 0.9999]
    pub fn a_star(&self) -> f64 {
        self.a / self.m
    }

    /// Physical gravitational radius r_g = G * M / c^2 in kilometers.
    /// G * M_sun / c^2 = 1.477 km
    pub fn r_grav_km(&self) -> f64 {
        self.m_solar * 1.477
    }

    /// Physical gravitational time unit t_g = G * M / c^3 in seconds.
    /// G * M_sun / c^3 = 1.477 km / 299792.458 km/s = 4.927e-6 s (microseconds).
    pub fn t_grav_seconds(&self) -> f64 {
        self.m_solar * 4.927038e-6
    }

    /// Format a radial coordinate r (in units of M) into physical distance units (km, AU, or light-years).
    pub fn format_physical_distance(&self, r: f64) -> String {
        let km = (r / self.m) * self.r_grav_km();
        let au = km / 1.496e8;
        let ly = km / 9.461e12;

        if ly >= 0.01 {
            format!("{:.2} ly ({:.0} AU)", ly, au)
        } else if au >= 0.05 {
            format!("{:.1} AU ({:.2e} km)", au, km)
        } else if km >= 1e6 {
            format!("{:.2} M km", km / 1e6)
        } else {
            format!("{:.1} km", km)
        }
    }

    /// Format a coordinate time interval t (in units of M/c) into human-readable physical time units.
    pub fn format_physical_time(&self, t_in_m: f64) -> String {
        let secs = (t_in_m / self.m).abs() * self.t_grav_seconds();
        let sign = if t_in_m < 0.0 { "-" } else { "" };

        if secs >= 86400.0 * 365.25 {
            format!("{}{:.2} yr", sign, secs / (86400.0 * 365.25))
        } else if secs >= 86400.0 {
            format!("{}{:.2} days", sign, secs / 86400.0)
        } else if secs >= 3600.0 {
            format!("{}{:.2} hrs", sign, secs / 3600.0)
        } else if secs >= 60.0 {
            format!("{}{:.2} min", sign, secs / 60.0)
        } else if secs >= 1.0 {
            format!("{}{:.2} s", sign, secs)
        } else if secs >= 1e-3 {
            format!("{}{:.2} ms", sign, secs * 1e3)
        } else {
            format!("{}{:.2} µs", sign, secs * 1e6)
        }
    }

    /// Convert coordinate radius r (in units of M) to kilometers.
    pub fn r_to_km(&self, r: f64) -> f64 {
        (r / self.m) * self.r_grav_km()
    }

    /// Convert kilometers to coordinate radius r (in units of M).
    pub fn km_to_r(&self, km: f64) -> f64 {
        (km / self.r_grav_km()) * self.m
    }

    /// Convert coordinate time t (in units of M/c) to seconds.
    pub fn t_to_seconds(&self, t: f64) -> f64 {
        (t / self.m) * self.t_grav_seconds()
    }

    /// Convert seconds to coordinate time t (in units of M/c).
    pub fn seconds_to_t(&self, secs: f64) -> f64 {
        (secs / self.t_grav_seconds()) * self.m
    }

    /// Format a distance value in kilometers nicely.
    pub fn format_km(&self, km: f64) -> String {
        if km >= 1e9 {
            format!("{:.2e} km", km)
        } else if km >= 1e6 {
            format!("{:.2}M km", km / 1e6)
        } else if km >= 1e3 {
            format!("{:.1}k km", km / 1e3)
        } else if km >= 10.0 {
            format!("{:.1} km", km)
        } else {
            format!("{:.2} km", km)
        }
    }

    /// Format km for grid lines ensuring at least the least significant digit changes between consecutive steps.
    pub fn format_grid_km(&self, km: f64, km_step: f64) -> String {
        let km = if km.abs() < 1e-12 { 0.0 } else { km };
        let km_step = km_step.max(1e-9);

        let (unit, suffix) = if km >= 1e9 && km_step >= 1e6 {
            (1e9, "B km")
        } else if km >= 1e6 && km_step >= 1e3 {
            (1e6, "M km")
        } else if km >= 1e3 && km_step >= 1.0 {
            (1e3, "k km")
        } else {
            (1.0, " km")
        };

        let step_in_unit = km_step / unit;
        let decimals = if step_in_unit >= 0.999 {
            0
        } else {
            ((-step_in_unit.log10()).ceil().max(1.0)) as usize
        };

        format!("{:.prec$}{}", km / unit, suffix, prec = decimals)
    }

    /// Format r in units of M for grid lines ensuring at least the least significant digit changes between consecutive steps.
    pub fn format_grid_m(&self, r: f64, r_step: f64) -> String {
        let r = if r.abs() < 1e-12 { 0.0 } else { r };
        let r_step = r_step.max(1e-9);

        let decimals = if r_step >= 0.999 {
            0
        } else {
            ((-r_step.log10()).ceil().max(1.0)) as usize
        };

        format!("{:.prec$}M", r, prec = decimals)
    }

    /// Format a radius either in km or M based on use_km flag.
    pub fn format_r(&self, r: f64, use_km: bool) -> String {
        if use_km {
            self.format_km(self.r_to_km(r))
        } else {
            format!("{:.2}M", r)
        }
    }

    /// Differential tidal acceleration stretching force across height_m in units of Earth g's (9.81 m/s^2).
    /// a_tidal = (2 * G * M / r^3) * height_m
    pub fn tidal_acceleration_g(&self, r: f64, height_m: f64) -> f64 {
        let r_m = (r / self.m).max(0.01) * self.r_grav_km() * 1000.0;
        let g_const = 6.67430e-11;
        let m_kg = self.m_solar * 1.98847e30;
        let a_tidal_si = (2.0 * g_const * m_kg / (r_m * r_m * r_m)) * height_m;
        (a_tidal_si / 9.80665).max(0.0)
    }

    /// Tidal gravity gradient in Earth gravities per meter (g/m = 9.80665 m/s^2 per meter).
    /// dg/dr = (2 * G * M / r^3) / 9.80665
    pub fn tidal_gradient_g_per_m(&self, r: f64) -> f64 {
        self.tidal_acceleration_g(r, 1.0)
    }

    /// Outer event horizon radius r+ = M + sqrt(M^2 - a^2)
    pub fn outer_horizon(&self) -> f64 {
        let disc = (self.m * self.m - self.a * self.a).max(0.0);
        self.m + disc.sqrt()
    }

    /// Inner Cauchy horizon radius r- = M - sqrt(M^2 - a^2)
    pub fn inner_horizon(&self) -> f64 {
        let disc = (self.m * self.m - self.a * self.a).max(0.0);
        self.m - disc.sqrt()
    }

    /// Static limit / ergosphere radius on the equatorial plane (theta = pi/2): r_E = 2M.
    pub fn ergosphere_equatorial(&self) -> f64 {
        2.0 * self.m
    }

    /// Delta(r) = r^2 - 2Mr + a^2 = (r - r+)(r - r-)
    pub fn delta(&self, r: f64) -> f64 {
        r * r - 2.0 * self.m * r + self.a * self.a
    }

    /// Surface gravity of the outer event horizon: kappa+ > 0
    pub fn surface_gravity_outer(&self) -> f64 {
        let rp = self.outer_horizon();
        let disc = (self.m * self.m - self.a * self.a).max(0.0);
        disc.sqrt() / (2.0 * self.m * rp)
    }

    /// Surface gravity of the inner Cauchy horizon: kappa- < 0
    /// The negative sign reflects the exponential blueshift pile-up at r-.
    pub fn surface_gravity_inner(&self) -> f64 {
        let rm = self.inner_horizon();
        let disc = (self.m * self.m - self.a * self.a).max(0.0);
        -disc.sqrt() / (2.0 * self.m * rm.max(1e-6))
    }

    /// Exterior time compression factor dt_exterior / dtau_proper relative to distant universe.
    /// In Region I (r > r+), dt/dtau ~ 1 / sqrt(1 - 2M/r).
    /// In Region II (r- < r < r+), incoming signals pile up exponentially:
    /// C(r) = dt_ext / dtau ~ ((r+ - r-) / (r - r-))^{2 * |kappa-| * M}
    pub fn exterior_time_compression(&self, r: f64) -> f64 {
        let rp = self.outer_horizon();
        let rm = self.inner_horizon();

        if r >= rp {
            // Gravitational time dilation in exterior region
            let g_tt = -(1.0 - 2.0 * self.h_scalar(r));
            if g_tt < 0.0 {
                1.0 / (-g_tt).sqrt().max(0.05)
            } else {
                1.0
            }
        } else if r > rm {
            // Exponential blueshift pile-up in Region II approaching Cauchy horizon
            let dist_to_rm = (r - rm).max(0.0001);
            let kappa_m = self.surface_gravity_inner().abs();
            let exponent = (kappa_m * 4.0 * self.m).clamp(0.8, 3.5);
            let base = ((rp - rm) / dist_to_rm).max(1.0);
            (1.0 + base.powf(exponent)).max(1.0)
        } else {
            // Region III: inside Cauchy horizon
            1.0
        }
    }

    /// Equatorial frame-dragging angular velocity omega(r) = -g_{t phi} / g_{phi phi}
    pub fn frame_dragging_omega(&self, r: f64) -> f64 {
        let r = r.max(1e-5);
        let r2 = r * r;
        let a2 = self.a * self.a;
        // Sigma on equatorial plane: (r^2 + a^2)^2 - a^2 * Delta
        let delta = self.delta(r);
        let sigma = (r2 + a2) * (r2 + a2) - a2 * delta;
        (2.0 * self.m * self.a * r) / sigma.max(1e-9)
    }

    /// Kerr-Schild scalar function H(r) = M / r on equatorial plane.
    #[inline]
    pub fn h_scalar(&self, r: f64) -> f64 {
        self.m / r.max(1e-6)
    }

    /// Metric tensor components g_{mu nu} in equatorial Kerr-Schild coordinates (t, r, phi).
    /// Ordered as [0: t, 1: r, 2: phi].
    pub fn metric_components(&self, r: f64) -> [[f64; 3]; 3] {
        let h = self.h_scalar(r);
        let a = self.a;
        let r2 = r * r;
        let a2 = a * a;

        let g_tt = -(1.0 - 2.0 * h);
        let g_tr = 2.0 * h;
        let g_rr = 1.0 + 2.0 * h;
        let g_tphi = -2.0 * h * a;
        let g_rphi = -a * (1.0 + 2.0 * h);
        let g_phiphi = r2 + a2 + 2.0 * h * a2;

        [
            [g_tt, g_tr, g_tphi],
            [g_tr, g_rr, g_rphi],
            [g_tphi, g_rphi, g_phiphi],
        ]
    }

    /// Inverse metric component g^{rr} = Delta / rho^2.
    /// In equatorial plane rho^2 = r^2, so g^{rr} = Delta / r^2.
    /// When Delta > 0 (r > r+ or r < r-), g^{rr} > 0: r is spacelike (maneuverable).
    /// When Delta < 0 (r- < r < r+), g^{rr} < 0: r is timelike (trapped inward!).
    pub fn g_upper_rr(&self, r: f64) -> f64 {
        let r = r.max(1e-6);
        self.delta(r) / (r * r)
    }

    /// Compute exact coordinate slopes dr/dt for ingoing and outgoing null geodesics
    /// with zero angular momentum (or along the principal null directions).
    pub fn radial_null_slopes(&self, r: f64) -> NullSlopes {
        let r = r.max(1e-5);
        let a2 = self.a * self.a;
        let delta = self.delta(r);

        // In ingoing Kerr-Schild coordinates, ingoing null rays propagate at dr/dt = -1 everywhere.
        let dr_dt_ingoing = -1.0;

        // Outgoing null ray slope dr/dt:
        // Derived from g_{mu nu} v^mu v^nu = 0 with dphi/dt matching the frame-dragging stream.
        // For Kerr-Schild, outgoing coordinate velocity is:
        // dr/dt = Delta / (r^2 + a^2 + 2 * M * r)
        let denominator = r * r + a2 + 2.0 * self.m * r;
        let dr_dt_outgoing = delta / denominator.max(1e-9);

        let dphi_dt_drag = self.frame_dragging_omega(r);

        NullSlopes {
            dr_dt_ingoing,
            dr_dt_outgoing,
            dphi_dt_drag,
        }
    }

    /// Compute an angular fan of null vectors around Bob at radius r in his local frame,
    /// mapped to coordinate velocities (dr/dt, dphi/dt).
    /// `angles`: array of emission angles alpha in [0, 2*pi] in the local rest frame.
    /// Returns vector of (dr/dt, dphi/dt).
    pub fn null_cone_fan(&self, r: f64, num_rays: usize) -> Vec<(f64, f64)> {
        let mut fan = Vec::with_capacity(num_rays);
        let slopes = self.radial_null_slopes(r);
        let omega = slopes.dphi_dt_drag;

        for i in 0..num_rays {
            let alpha = 2.0 * std::f64::consts::PI * (i as f64) / (num_rays as f64);
            // Local direction vector (cos(alpha), sin(alpha))
            // cos(alpha) = +1 corresponds to local outgoing radial direction
            // cos(alpha) = -1 corresponds to local ingoing radial direction
            let cos_a = alpha.cos();
            let sin_a = alpha.sin();

            // Interpolate radial coordinate velocity between ingoing (-1) and outgoing (slopes.dr_dt_outgoing)
            // Weight: 0 for ingoing, 1 for outgoing
            let radial_weight = 0.5 * (1.0 + cos_a);
            let dr_dt = (1.0 - radial_weight) * slopes.dr_dt_ingoing + radial_weight * slopes.dr_dt_outgoing;

            // Azimuthal component includes frame-dragging omega plus local transverse velocity
            let dphi_dt = omega + (sin_a / r.max(1e-3)) * 0.5;

            fan.push((dr_dt, dphi_dt));
        }
        fan
    }

    /// Kretschmann curvature scalar K(r) on equatorial plane.
    /// Illustrates why the curvature remains finite at r+ and r-,
    /// only diverging at the physical ring singularity (r -> 0).
    pub fn kretschmann_scalar(&self, r: f64) -> f64 {
        let r = r.max(1e-4);
        let r2 = r * r;
        let a2 = self.a * self.a;
        let denom = (r2 + a2).powi(6);
        let num = 48.0 * self.m * self.m * (r2 - a2) * (r2.powi(2) - 14.0 * a2 * r2 + a2.powi(2));
        num / denom.max(1e-12)
    }

    /// Calculate the blue-shift amplification factor for radiation emitted at exterior coordinate time t_ext
    /// received near the Cauchy horizon r-:
    /// Factor ~ exp(|kappa-| * t_ext)
    pub fn cauchy_blueshift_factor(&self, t_ext: f64) -> f64 {
        let kappa_abs = self.surface_gravity_inner().abs();
        (kappa_abs * t_ext.min(50.0)).exp()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_schwarzschild_limit() {
        let m = 1.0;
        let ks = KerrSchild::new(m, 0.0);

        assert!((ks.outer_horizon() - 2.0).abs() < 1e-10);
        assert!((ks.inner_horizon() - 0.0).abs() < 1e-10);
        assert!((ks.ergosphere_equatorial() - 2.0).abs() < 1e-10);

        // Outside horizon: outgoing > 0
        let slopes_out = ks.radial_null_slopes(4.0);
        assert_eq!(slopes_out.dr_dt_ingoing, -1.0);
        assert!(slopes_out.dr_dt_outgoing > 0.0);

        // At horizon: outgoing == 0
        let slopes_hor = ks.radial_null_slopes(2.0);
        assert!((slopes_hor.dr_dt_outgoing).abs() < 1e-10);

        // Inside horizon: outgoing < 0 (tipped!)
        let slopes_in = ks.radial_null_slopes(1.0);
        assert!(slopes_in.dr_dt_outgoing < 0.0);
    }

    #[test]
    fn test_kerr_horizons_and_tipping() {
        let m = 1.0;
        let a = 0.6;
        let ks = KerrSchild::new(m, a);

        let rp = ks.outer_horizon(); // 1.0 + sqrt(1.0 - 0.36) = 1.0 + 0.8 = 1.8
        let rm = ks.inner_horizon(); // 1.0 - 0.8 = 0.2

        assert!((rp - 1.8).abs() < 1e-10);
        assert!((rm - 0.2).abs() < 1e-10);

        // Region I (r > 1.8): outgoing slope > 0
        let s_i = ks.radial_null_slopes(3.0);
        assert!(s_i.dr_dt_outgoing > 0.0);

        // Horizon r+: outgoing slope == 0
        let s_rp = ks.radial_null_slopes(rp);
        assert!(s_rp.dr_dt_outgoing.abs() < 1e-10);

        // Region II (0.2 < r < 1.8): outgoing slope < 0 (TIPPED PAST VERTICAL!)
        let s_ii = ks.radial_null_slopes(1.0);
        assert!(s_ii.dr_dt_outgoing < 0.0);

        // Horizon r-: outgoing slope == 0 (Cauchy horizon)
        let s_rm = ks.radial_null_slopes(rm);
        assert!(s_rm.dr_dt_outgoing.abs() < 1e-10);

        // Region III (0 < r < 0.2): outgoing slope > 0 (UN-TIPPED!)
        let s_iii = ks.radial_null_slopes(0.1);
        assert!(s_iii.dr_dt_outgoing > 0.0);
    }

    #[test]
    fn test_frame_dragging_monotonicity() {
        let ks = KerrSchild::new(1.0, 0.8);
        let omega_far = ks.frame_dragging_omega(10.0);
        let omega_near = ks.frame_dragging_omega(2.0);
        let omega_inner = ks.frame_dragging_omega(0.5);

        assert!(omega_far < omega_near);
        assert!(omega_near < omega_inner);
    }

    #[test]
    fn test_physical_units_conversion() {
        // Solar mass: 1M_sun => Rg ~ 1.477 km, tg ~ 4.93 µs
        let ks_sun = KerrSchild::with_solar_mass(1.0, 0.0, 1.0);
        let dist = ks_sun.format_physical_distance(1.0);
        assert!(dist.contains("1.5 km") || dist.contains("1.48 km") || dist.contains("km"));

        let time = ks_sun.format_physical_time(1.0);
        assert!(time.contains("µs"));

        // Sagittarius A*: 4.15e6 M_sun => tg ~ 20.4 s
        let ks_sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        let sgr_time = ks_sgr.format_physical_time(1.0);
        assert!(sgr_time.contains("s"));
    }

    #[test]
    fn test_tidal_acceleration_and_observer_telemetry() {
        // Stellar mass black hole (10 M_sun): tidal forces at r = 2M are lethal (> 10^5 g)
        let ks_stellar = KerrSchild::with_solar_mass(1.0, 0.7, 10.0);
        let tidal_stellar = ks_stellar.tidal_acceleration_g(2.0, 1.8);
        assert!(tidal_stellar > 1e4, "Stellar BH tidal force at r=2M should be extreme: {}", tidal_stellar);

        // Supermassive black hole (Ton 618: 6.6e10 M_sun): tidal forces at r = 2M are negligible (< 10^-5 g)
        let ks_ton = KerrSchild::with_solar_mass(1.0, 0.99, 6.6e10);
        let tidal_ton = ks_ton.tidal_acceleration_g(2.0, 1.8);
        assert!(tidal_ton < 1e-4, "Supermassive BH tidal force at horizon should be tiny: {}", tidal_ton);

        // Test tidal gradient (g/m)
        let grad_stellar = ks_stellar.tidal_gradient_g_per_m(2.0);
        assert!(grad_stellar > 5e3, "Stellar BH tidal gradient should be > 5000 g/m: {}", grad_stellar);
        let grad_ton = ks_ton.tidal_gradient_g_per_m(2.0);
        assert!(grad_ton < 1e-4, "Ton 618 tidal gradient should be < 1e-4 g/m: {}", grad_ton);
    }

    #[test]
    fn test_grid_labels_consecutive_digits_change() {
        let ks_sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        // Test Sgr A* with 1000 km step around 6.13M km
        let step_km = 1000.0;
        let base_km = 6_130_000.0;
        let l1 = ks_sgr.format_grid_km(base_km, step_km);
        let l2 = ks_sgr.format_grid_km(base_km + step_km, step_km);
        assert_ne!(l1, l2, "Labels must differ for consecutive km grid lines: {} vs {}", l1, l2);

        // Test with 100 km step
        let l3 = ks_sgr.format_grid_km(base_km, 100.0);
        let l4 = ks_sgr.format_grid_km(base_km + 100.0, 100.0);
        assert_ne!(l3, l4, "Labels must differ for 100 km grid lines: {} vs {}", l3, l4);

        // Test M units with fine step near Cauchy horizon
        let r_step = 0.0001;
        let r_base = 0.4358;
        let m1 = ks_sgr.format_grid_m(r_base, r_step);
        let m2 = ks_sgr.format_grid_m(r_base + r_step, r_step);
        assert_ne!(m1, m2, "Labels must differ for consecutive M grid lines: {} vs {}", m1, m2);

        // Test extreme zoom 0.00002 M step
        let r_step_micro = 0.00002;
        let u1 = ks_sgr.format_grid_m(r_base, r_step_micro);
        let u2 = ks_sgr.format_grid_m(r_base + r_step_micro, r_step_micro);
        assert_ne!(u1, u2, "Labels must differ for microscopic M grid lines: {} vs {}", u1, u2);
    }
}
