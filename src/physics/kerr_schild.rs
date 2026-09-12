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

/// Coordinate slopes of the two *principal null directions* (PNDs) of the Kerr geometry at a
/// radius r on the equatorial plane. These are the repeated null eigendirections of the Weyl
/// tensor (the algebraically special rays that make Kerr type D), not the extreme rays of the
/// light cone: see `NullWedge` for the latter.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct NullSlopes {
    /// dr/dt of the ingoing PND: exactly -1 in ingoing Kerr-Schild coordinates, at every r.
    pub dr_dt_ingoing: f64,
    /// dr/dt of the outgoing PND: Delta / (r^2 + a^2 + 2Mr).
    pub dr_dt_outgoing: f64,
    /// dphi/dt of the ingoing PND: exactly 0 (the ingoing PND is the generator of the chart).
    pub dphi_dt_ingoing: f64,
    /// dphi/dt of the outgoing PND: 2a / (r^2 + a^2 + 2Mr).
    pub dphi_dt_outgoing: f64,
}

/// The projection of the full null cone at radius r onto the (t, r) plane of the spacetime
/// diagram: the open interval of radial coordinate velocities dr/dt that a light ray can have.
/// It is a property of the event alone, independent of which observer draws it.
#[derive(Debug, Clone, Copy)]
pub struct NullWedge {
    /// Most negative dr/dt attainable by a null ray (the inner edge of the wedge).
    pub dr_dt_in: f64,
    /// Most positive dr/dt attainable by a null ray (the outer edge of the wedge).
    pub dr_dt_out: f64,
    /// dphi/dt carried by the ray that realises `dr_dt_in`.
    pub dphi_dt_in: f64,
    /// dphi/dt carried by the ray that realises `dr_dt_out`.
    pub dphi_dt_out: f64,
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

    /// Radial derivative d g_{mu nu} / dr of the equatorial Kerr-Schild metric, analytic.
    /// The metric depends on r alone, so this is the only non-zero derivative and it is all
    /// that is needed to build the Christoffel symbols. With h = M/r and h' = -M/r^2:
    ///   d g_tt/dr     =  2 h'
    ///   d g_tr/dr     =  2 h'
    ///   d g_rr/dr     =  2 h'
    ///   d g_tphi/dr   = -2 a h'
    ///   d g_rphi/dr   = -2 a h'
    ///   d g_phiphi/dr =  2 r + 2 a^2 h'
    pub fn metric_derivative_r(&self, r: f64) -> [[f64; 3]; 3] {
        let r = r.max(1e-6);
        let a = self.a;
        let a2 = a * a;
        // h = M / r  =>  dh/dr = -M / r^2
        let dh = -self.m / (r * r);

        let d_tt = 2.0 * dh;
        let d_tr = 2.0 * dh;
        let d_rr = 2.0 * dh;
        let d_tphi = -2.0 * a * dh;
        let d_rphi = -2.0 * a * dh;
        let d_phiphi = 2.0 * r + 2.0 * a2 * dh;

        [
            [d_tt, d_tr, d_tphi],
            [d_tr, d_rr, d_rphi],
            [d_tphi, d_rphi, d_phiphi],
        ]
    }

    /// Contravariant metric g^{mu nu} in equatorial Kerr-Schild coordinates (t, r, phi).
    /// The 3-metric is regular (det g = -r^2) everywhere off the ring singularity, so the
    /// inverse exists at and inside both horizons; g^{rr} = Delta / r^2 changes sign there.
    pub fn inverse_metric(&self, r: f64) -> [[f64; 3]; 3] {
        let g = self.metric_components(r);
        let m = nalgebra::Matrix3::new(
            g[0][0], g[0][1], g[0][2],
            g[1][0], g[1][1], g[1][2],
            g[2][0], g[2][1], g[2][2],
        );
        let inv = m.try_inverse().unwrap_or_else(nalgebra::Matrix3::zeros);
        [
            [inv[(0, 0)], inv[(0, 1)], inv[(0, 2)]],
            [inv[(1, 0)], inv[(1, 1)], inv[(1, 2)]],
            [inv[(2, 0)], inv[(2, 1)], inv[(2, 2)]],
        ]
    }

    /// Christoffel symbols of the second kind, indexed [mu][alpha][beta] = Gamma^mu_{alpha beta}:
    ///     Gamma^mu_{alpha beta} = 1/2 g^{mu nu} (d_alpha g_{nu beta} + d_beta g_{nu alpha} - d_nu g_{alpha beta})
    /// Only d_r is non-zero because the equatorial Kerr-Schild metric is stationary and axisymmetric.
    /// The equatorial plane is totally geodesic, so these 3-metric symbols coincide with the
    /// projections of the full 4D ones for motion that stays in the plane.
    pub fn christoffel(&self, r: f64) -> [[[f64; 3]; 3]; 3] {
        let ginv = self.inverse_metric(r);
        // dg[alpha][nu][beta] = d_alpha g_{nu beta}; only alpha = 1 (the r direction) survives.
        let mut dg = [[[0.0f64; 3]; 3]; 3];
        dg[1] = self.metric_derivative_r(r);

        let mut gamma = [[[0.0f64; 3]; 3]; 3];
        for mu in 0..3 {
            for alpha in 0..3 {
                for beta in 0..3 {
                    let mut sum = 0.0;
                    for nu in 0..3 {
                        sum += ginv[mu][nu]
                            * (dg[alpha][nu][beta] + dg[beta][nu][alpha] - dg[nu][alpha][beta]);
                    }
                    gamma[mu][alpha][beta] = 0.5 * sum;
                }
            }
        }
        gamma
    }

    /// Squared norm g_{mu nu} u^mu u^nu of a contravariant vector u = (u^t, u^r, u^phi) at radius r.
    /// Negative for timelike, zero for null, positive for spacelike vectors.
    pub fn norm(&self, r: f64, u: &[f64; 3]) -> f64 {
        let g = self.metric_components(r);
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                sum += g[i][j] * u[i] * u[j];
            }
        }
        sum
    }

    /// The two *principal null directions* of the equatorial Kerr geometry at radius r, as
    /// coordinate slopes in ingoing Kerr-Schild coordinates. As tangent vectors (dt, dr, dphi):
    ///
    ///     ingoing   ~ (1, -1, 0)
    ///     outgoing  ~ (r^2 + a^2 + 2Mr, Delta, 2a)
    ///
    /// The ingoing PND is the generator of the ingoing Kerr-Schild congruence, which is why the
    /// chart is regular across both horizons and why ingoing light always moves at dr/dt = -1.
    /// The outgoing PND has dr/dt = Delta / (r^2 + a^2 + 2Mr), so it changes sign exactly at r+
    /// and again at r-: outgoing light is dragged inward throughout Region II and turns around
    /// again inside the Cauchy horizon.
    ///
    /// These are *not* the edges of the light cone as seen in the (t, r) diagram; the extreme
    /// radial slopes are carried by the zero-angular-momentum rays of `null_wedge`.
    pub fn radial_null_slopes(&self, r: f64) -> NullSlopes {
        let r = r.max(1e-5);
        let a2 = self.a * self.a;
        let delta = self.delta(r);

        // Common denominator k^t of the outgoing PND, strictly positive for r > 0.
        let kt_out = (r * r + a2 + 2.0 * self.m * r).max(1e-9);

        NullSlopes {
            dr_dt_ingoing: -1.0,
            dr_dt_outgoing: delta / kt_out,
            dphi_dt_ingoing: 0.0,
            dphi_dt_outgoing: 2.0 * self.a / kt_out,
        }
    }

    /// Extreme radial coordinate velocities dr/dt of light at radius r: the shadow the null cone
    /// casts on the (t, r) plane of the spacetime diagram.
    ///
    /// Writing a null tangent as (1, v, w) with v = dr/dt and w = dphi/dt, the null condition is
    ///     g_tt + 2 g_tr v + g_rr v^2 + 2 (g_tphi + g_rphi v) w + g_phiphi w^2 = 0.
    /// A real w exists for a given v exactly when the discriminant of that quadratic in w is
    /// non-negative:
    ///     D(v) = (g_tphi + g_rphi v)^2 - g_phiphi (g_tt + 2 g_tr v + g_rr v^2) = A v^2 + B v + C
    /// with
    ///     A = g_rphi^2 - g_phiphi g_rr = -(1 + 2M/r) r^2   < 0
    ///     B = 2 g_tphi g_rphi - 2 g_phiphi g_tr = -4 M r
    ///     C = g_tphi^2 - g_phiphi g_tt = Delta             (exactly)
    /// so D is a downward parabola and the admissible v form the closed interval between its two
    /// roots. Extremising v over the cone by Lagrange multipliers puts the boundary rays at
    /// D(v) = 0, where the double root in w is w = -(g_tphi + g_rphi v) / g_phiphi, i.e. exactly
    /// k_phi = 0: the wedge edges are the *zero-angular-momentum* rays, not the PNDs.
    ///
    /// Consequences encoded in the tests: D(-1) = a^2 >= 0, so dr_dt_in <= -1 always (with
    /// equality iff a = 0); D(0) = Delta, so dr_dt_out = 0 exactly on either horizon, is negative
    /// throughout Region II (nothing can even hold station) and positive outside it. For a = 0 the
    /// roots are -1 and (r - 2M) / (r + 2M).
    pub fn null_wedge(&self, r: f64) -> NullWedge {
        let r = r.max(1e-5);
        let g = self.metric_components(r);
        let (g_tt, g_tr, g_tphi) = (g[0][0], g[0][1], g[0][2]);
        let (g_rr, g_rphi, g_phiphi) = (g[1][1], g[1][2], g[2][2]);

        let quad_a = g_rphi * g_rphi - g_phiphi * g_rr;
        let quad_b = 2.0 * g_tphi * g_rphi - 2.0 * g_phiphi * g_tr;
        let quad_c = g_tphi * g_tphi - g_phiphi * g_tt;

        // quad_a = -(1 + 2M/r) r^2 is strictly negative, so the parabola always opens downward and
        // the discriminant is non-negative (D(-1) = a^2 >= 0 puts a real point above the axis).
        let root = (quad_b * quad_b - 4.0 * quad_a * quad_c).max(0.0).sqrt();
        let v_a = (-quad_b + root) / (2.0 * quad_a);
        let v_b = (-quad_b - root) / (2.0 * quad_a);
        let (dr_dt_in, dr_dt_out) = if v_a <= v_b { (v_a, v_b) } else { (v_b, v_a) };

        // Along an edge the w-quadratic has a double root: w = -(g_tphi + g_rphi v) / g_phiphi.
        let w_of = |v: f64| -(g_tphi + g_rphi * v) / g_phiphi;

        NullWedge {
            dr_dt_in,
            dr_dt_out,
            dphi_dt_in: w_of(dr_dt_in),
            dphi_dt_out: w_of(dr_dt_out),
        }
    }

    /// Kretschmann curvature scalar K = R_{abcd} R^{abcd} on the equatorial plane.
    /// For Kerr, K = 48 M^2 (r^2 - a^2 cos^2 th)[(r^2 + a^2 cos^2 th)^2 - 16 r^2 a^2 cos^2 th] / (r^2 + a^2 cos^2 th)^6,
    /// which at th = pi/2 reduces to K = 48 M^2 / r^6: finite at r+ and r-, divergent only at the ring r = 0.
    pub fn kretschmann_scalar(&self, r: f64) -> f64 {
        let r = r.max(1e-4);
        48.0 * self.m * self.m / r.powi(6)
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

    /// Radii spanning every region: exterior, r+, between the horizons, r-, and inside r-.
    fn probe_radii(ks: &KerrSchild) -> Vec<f64> {
        let rp = ks.outer_horizon();
        let rm = ks.inner_horizon().max(0.05);
        vec![12.0, 6.0, 3.0, rp, 0.5 * (rp + rm), rm, 0.5 * rm, 0.1]
    }

    #[test]
    fn test_inverse_metric_is_a_true_inverse() {
        for &a in &[0.0, 0.5, 0.65, 0.95, 0.9999] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let g = ks.metric_components(r);
                let gi = ks.inverse_metric(r);
                for i in 0..3 {
                    for j in 0..3 {
                        let mut s = 0.0;
                        for k in 0..3 {
                            s += g[i][k] * gi[k][j];
                        }
                        let expected = if i == j { 1.0 } else { 0.0 };
                        assert!(
                            (s - expected).abs() < 1e-9,
                            "g * g^-1 [{i}][{j}] = {s} at r={r} (a={a})"
                        );
                    }
                }
                // g^{rr} must reproduce the closed form Delta / r^2, including where it is negative.
                assert!(
                    (gi[1][1] - ks.g_upper_rr(r)).abs() < 1e-9 * (1.0 + gi[1][1].abs()),
                    "g^rr = {} vs Delta/r^2 = {} at r={r} (a={a})",
                    gi[1][1],
                    ks.g_upper_rr(r)
                );
            }
        }
    }

    #[test]
    fn test_metric_derivative_matches_finite_difference() {
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in &[8.0, 4.0, 2.0, 1.0, 0.3] {
                let h = 1e-6 * r;
                let gp = ks.metric_components(r + h);
                let gm = ks.metric_components(r - h);
                let dg = ks.metric_derivative_r(r);
                for i in 0..3 {
                    for j in 0..3 {
                        let fd = (gp[i][j] - gm[i][j]) / (2.0 * h);
                        assert!(
                            (dg[i][j] - fd).abs() < 1e-6 * (1.0 + fd.abs()),
                            "dg[{i}][{j}] = {} vs FD {fd} at r={r} (a={a})",
                            dg[i][j]
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_christoffel_symmetric_in_lower_indices() {
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let gam = ks.christoffel(r);
                for mu in 0..3 {
                    for al in 0..3 {
                        for be in 0..3 {
                            let d = (gam[mu][al][be] - gam[mu][be][al]).abs();
                            assert!(
                                d < 1e-12,
                                "Gamma^{mu}_{{{al}{be}}} asymmetric by {d} at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_metric_compatibility_of_christoffels() {
        // Nabla_alpha g_{mu nu} = 0: d_alpha g_{mu nu} - Gamma^l_{alpha mu} g_{l nu} - Gamma^l_{alpha nu} g_{mu l} = 0.
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let g = ks.metric_components(r);
                let gam = ks.christoffel(r);
                let mut dg = [[[0.0f64; 3]; 3]; 3];
                dg[1] = ks.metric_derivative_r(r);
                for al in 0..3 {
                    for mu in 0..3 {
                        for nu in 0..3 {
                            let mut s = dg[al][mu][nu];
                            let mut scale = dg[al][mu][nu].abs();
                            for l in 0..3 {
                                let term = gam[l][al][mu] * g[l][nu] + gam[l][al][nu] * g[mu][l];
                                s -= term;
                                scale += term.abs();
                            }
                            assert!(
                                s.abs() < 1e-10 * (1.0 + scale),
                                "Nabla_{al} g_{{{mu}{nu}}} = {s} at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_principal_null_directions_are_null() {
        // Both PNDs must satisfy g(k, k) = 0 against the coded metric, at every radius.
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let s = ks.radial_null_slopes(r);
                for k in [
                    [1.0, s.dr_dt_ingoing, s.dphi_dt_ingoing],
                    [1.0, s.dr_dt_outgoing, s.dphi_dt_outgoing],
                ] {
                    let n = ks.norm(r, &k);
                    assert!(n.abs() < 1e-10, "PND norm = {n} for {k:?} at r={r} (a={a})");
                }
                assert_eq!(s.dr_dt_ingoing, -1.0);
                assert_eq!(s.dphi_dt_ingoing, 0.0);
                // The outgoing PND is dragged in the direction of the spin.
                assert!(s.dphi_dt_outgoing * a >= 0.0);
            }
        }
    }

    #[test]
    fn test_null_wedge_discriminant_constant_is_delta() {
        // C = g_tphi^2 - g_phiphi g_tt = Delta exactly, which is why the outer edge of the wedge
        // vanishes precisely on the horizons.
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let g = ks.metric_components(r);
                let c = g[0][2] * g[0][2] - g[2][2] * g[0][0];
                assert!(
                    (c - ks.delta(r)).abs() < 1e-10 * (1.0 + ks.delta(r).abs()),
                    "C = {c} vs Delta = {} at r={r} (a={a})",
                    ks.delta(r)
                );
            }
        }
    }

    #[test]
    fn test_null_wedge_schwarzschild_closed_form() {
        // With a = 0 the roots of D are exactly -1 and (r - 2M) / (r + 2M).
        let ks = KerrSchild::new(1.0, 0.0);
        for &r in &[12.0, 6.0, 2.0, 1.0, 0.3, 0.05] {
            let w = ks.null_wedge(r);
            assert!((w.dr_dt_in + 1.0).abs() < 1e-12, "in = {} at r={r}", w.dr_dt_in);
            let expected = (r - 2.0) / (r + 2.0);
            assert!(
                (w.dr_dt_out - expected).abs() < 1e-12,
                "out = {} vs {expected} at r={r}",
                w.dr_dt_out
            );
            // Without spin there is no frame dragging, so both edges are purely radial.
            assert!(w.dphi_dt_in.abs() < 1e-15 && w.dphi_dt_out.abs() < 1e-15);
        }
    }

    #[test]
    fn test_null_wedge_edges_are_null_and_carry_zero_angular_momentum() {
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            let g_of = |r: f64| ks.metric_components(r);
            for &r in probe_radii(&ks).iter() {
                let w = ks.null_wedge(r);
                let g = g_of(r);
                for k in [
                    [1.0, w.dr_dt_in, w.dphi_dt_in],
                    [1.0, w.dr_dt_out, w.dphi_dt_out],
                ] {
                    let scale = 1.0 + g[2][2].abs() * (1.0 + k[2] * k[2]);
                    let n = ks.norm(r, &k);
                    assert!(
                        n.abs() < 1e-9 * scale,
                        "edge norm = {n} for {k:?} at r={r} (a={a})"
                    );
                    // k_phi = g_{phi mu} k^mu = 0 is what makes these the extremal rays.
                    let k_phi = g[2][0] * k[0] + g[2][1] * k[1] + g[2][2] * k[2];
                    assert!(
                        k_phi.abs() < 1e-9 * scale,
                        "k_phi = {k_phi} for {k:?} at r={r} (a={a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_null_wedge_sign_structure_by_region() {
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            let rp = ks.outer_horizon();
            let rm = ks.inner_horizon().max(1e-4);

            // v_min <= -1 everywhere, because D(-1) = a^2 >= 0.
            for &r in probe_radii(&ks).iter() {
                let w = ks.null_wedge(r);
                assert!(w.dr_dt_in <= -1.0 + 1e-12, "in = {} at r={r} (a={a})", w.dr_dt_in);
                assert!(w.dr_dt_in < w.dr_dt_out, "wedge must be non-degenerate at r={r}");
            }

            // v_max = 0 exactly on the horizons (Delta = 0)...
            let horizons: Vec<f64> = if ks.inner_horizon() > 1e-3 { vec![rp, rm] } else { vec![rp] };
            for &r in horizons.iter() {
                let w = ks.null_wedge(r);
                assert!(w.dr_dt_out.abs() < 1e-9, "out = {} at horizon r={r} (a={a})", w.dr_dt_out);
            }
            // ...negative strictly inside the trapped band...
            if rp - rm > 1e-3 {
                for f in [0.25, 0.5, 0.75] {
                    let r = rm + f * (rp - rm);
                    assert!(
                        ks.null_wedge(r).dr_dt_out < 0.0,
                        "Region II must be trapped at r={r} (a={a})"
                    );
                }
            }
            // ...and positive outside it, in both Region I and Region III.
            for &r in &[rp + 0.5, rp + 4.0, 12.0] {
                assert!(ks.null_wedge(r).dr_dt_out > 0.0, "Region I at r={r} (a={a})");
            }
            if rm > 0.02 {
                for &r in &[0.25 * rm, 0.75 * rm] {
                    assert!(ks.null_wedge(r).dr_dt_out > 0.0, "Region III at r={r} (a={a})");
                }
            }
        }
    }

    #[test]
    fn test_null_wedge_matches_principal_null_directions_without_spin() {
        // With a = 0 the geometry is spherically symmetric: the radial PNDs carry no angular
        // momentum, so they *are* the edges of the wedge. With spin they part company.
        let ks0 = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 4.0, 2.0, 0.8, 0.1] {
            let s = ks0.radial_null_slopes(r);
            let w = ks0.null_wedge(r);
            assert!((s.dr_dt_ingoing - w.dr_dt_in).abs() < 1e-12);
            assert!((s.dr_dt_outgoing - w.dr_dt_out).abs() < 1e-12);
        }
        let ks = KerrSchild::new(1.0, 0.9);
        let s = ks.radial_null_slopes(4.0);
        let w = ks.null_wedge(4.0);
        assert!(
            (s.dr_dt_ingoing - w.dr_dt_in).abs() > 1e-4,
            "with spin the ingoing PND is strictly inside the wedge"
        );
        assert!(s.dr_dt_outgoing < w.dr_dt_out, "the wedge must bound the outgoing PND");
    }

    #[test]
    fn test_norm_of_ingoing_null_ray() {
        // In ingoing Kerr-Schild coordinates the ingoing principal null ray has dr/dt = -1, dphi/dt = 0.
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in probe_radii(&ks).iter() {
                let n = ks.norm(r, &[1.0, -1.0, 0.0]);
                assert!(n.abs() < 1e-10, "ingoing null norm = {n} at r={r} (a={a})");
            }
        }
    }
}
