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
#[allow(dead_code)] // constructed by `radial_null_slopes`, which the tests exercise
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

// Several members below are exercised only by the test suite (the unit conversions, the
// closed forms the GUI cross-checks against); they are part of the geometry's public surface.
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

    /// A rate quoted per unit of the chart's time, in inverse seconds.
    ///
    /// The inverse of the conversion `format_physical_time` makes: one M of coordinate time is
    /// t_g/m seconds, so a rate per M is that many times m/t_g per second. Angular velocities are
    /// the ones this is for - the observer's own dphi/dt and the local frame-dragging rate - which
    /// come out of the metric per M and have to be readable by somebody who does not think in M.
    pub fn rate_per_second(&self, per_m: f64) -> f64 {
        per_m * self.m.max(1e-12) / self.t_grav_seconds()
    }

    /// Convert coordinate radius r (in units of M) to kilometers.
    pub fn r_to_km(&self, r: f64) -> f64 {
        (r / self.m) * self.r_grav_km()
    }

    /// Convert kilometers to coordinate radius r (in units of M).
    pub fn km_to_r(&self, km: f64) -> f64 {
        (km / self.r_grav_km()) * self.m
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

    /// Format a radius either in km or M based on use_physical_units flag.
    pub fn format_r(&self, r: f64, use_physical_units: bool) -> String {
        if use_physical_units {
            self.format_km(self.r_to_km(r))
        } else {
            format!("{:.2}M", r)
        }
    }

    /// Differential tidal acceleration stretching force across height_m in units of Earth g's
    /// (9.81 m/s^2), from the Newtonian radial expression
    ///
    ///     a_tidal = (2 G M / r^3) * height_m
    ///
    /// evaluated at the observer's radius. Two things it is not. It carries no dependence on the
    /// spin - it is the field of a hole of this mass, not the equatorial tidal tensor of Kerr - and
    /// none on the observer's motion. And it is the *background* field only: the perturbation that
    /// makes the Cauchy horizon singular, whose tidal force diverges on the approach to r- while
    /// this expression stays finite, is not in it. See `TELEMETRY_HOVER_TIP` for what that costs
    /// the reading near r-.
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

    /// Surface gravity of the inner (Cauchy) horizon,
    ///     kappa_- = (r+ - r-) / (2 (r-^2 + a^2)).
    ///
    /// Derivation: the horizon generators of r = r- are the null Killing direction
    /// xi = d_t + Omega_- d_phi with Omega_- = a / (r-^2 + a^2), and for a Killing horizon of
    /// Kerr the surface gravity is kappa = |Delta'(r_H)| / (2 (r_H^2 + a^2)). With
    /// Delta = r^2 - 2Mr + a^2 the derivative is Delta'(r) = 2(r - M), so at r- it has modulus
    /// 2(M - r-) = 2 sqrt(M^2 - a^2) = r+ - r-, which gives the form above. The same construction
    /// at r+ gives the familiar kappa_+ = (r+ - r-) / (2 (r+^2 + a^2)); the two agree only in the
    /// extremal limit, where both vanish.
    ///
    /// kappa_- is the exponential rate at which an outgoing principal null ray inside r+ closes on
    /// r- in this chart: r - r- decays like exp(-kappa_- t). Two infallers released the same way
    /// but Delta t of coordinate time apart therefore cross that stack of outgoing rays with a
    /// relative blueshift approaching exp(kappa_- Delta t), which is what
    /// `wavefront::limiting_blueshift` returns.
    ///
    /// A non-spinning hole has r- = 0 and a = 0, so the denominator vanishes and kappa_- is
    /// infinite: there is no inner horizon to have a finite surface gravity, and the value
    /// diverges as a -> 0 at fixed M. Infinity is returned in that degenerate case rather than a
    /// clamped stand-in, so callers must decide what to draw.
    pub fn inner_surface_gravity(&self) -> f64 {
        let rm = self.inner_horizon();
        let denom = 2.0 * (rm * rm + self.a * self.a);
        if denom <= 0.0 {
            return f64::INFINITY;
        }
        (self.outer_horizon() - rm) / denom
    }

    /// Angular velocity Omega_- = a / (r-^2 + a^2) of the null generator of the inner horizon.
    ///
    /// The surface r = r- is a Killing horizon, and the one combination of the two Killing fields
    /// that goes null on it is chi = d_t + Omega_- d_phi: chi is the tangent to the generators, so
    /// Omega_- is the rate at which the Cauchy horizon itself turns. Delta(r-) = 0 reads
    /// r-^2 + a^2 = 2 M r-, so this is equally a / (2 M r-), the inner twin of the familiar
    /// Omega_+ = a / (2 M r+); the same chi is what `inner_surface_gravity` takes kappa_- from.
    ///
    /// It is therefore the rate at which everything that asymptotes to the far branch of r- ends
    /// up co-rotating. A ray with E - Omega_- L < 0 never crosses that branch and winds onto it at
    /// Omega_- (`wavefront::NullRay::frozen`), and so does a timelike worldline with the same sign:
    /// neither can cross a surface whose own generators it is settling onto.
    ///
    /// A hole with no spin has r- = 0 and a = 0, so the denominator vanishes. Zero is returned:
    /// chi degenerates to d_t, and there is no inner horizon to turn.
    pub fn inner_horizon_omega(&self) -> f64 {
        let rm = self.inner_horizon();
        let denom = rm * rm + self.a * self.a;
        if denom > 0.0 { self.a / denom } else { 0.0 }
    }

    /// Static limit / ergosphere radius on the equatorial plane (theta = pi/2): r_E = 2M.
    pub fn ergosphere_equatorial(&self) -> f64 {
        2.0 * self.m
    }

    /// Delta(r) = r^2 - 2Mr + a^2 = (r - r+)(r - r-)
    pub fn delta(&self, r: f64) -> f64 {
        r * r - 2.0 * self.m * r + self.a * self.a
    }

    /// Covariant components k_mu of the ingoing principal null ray, normalised to unit conserved
    /// energy at infinity E_gamma = -k_t = 1.
    ///
    /// The ray's tangent is k^mu = (1, -1, 0) at every radius (that is the defining property of the
    /// ingoing Kerr-Schild chart). Lowering with `metric_components` and using h = M/r:
    ///
    ///     k_t   = g_tt - g_tr     = -(1 - 2h) - 2h        = -1
    ///     k_r   = g_tr - g_rr     = 2h - (1 + 2h)         = -1
    ///     k_phi = g_tphi - g_rphi = -2 h a + a (1 + 2h)   =  a
    ///
    /// So k_mu = (-1, -1, a), independent of r and of M: the ray's energy at infinity, its radial
    /// covector component and its angular momentum are all constants of the motion, as they must be
    /// for a null geodesic of a stationary, axisymmetric spacetime.
    pub fn ingoing_null_covector(&self) -> [f64; 3] {
        [-1.0, -1.0, self.a]
    }

    /// Frequency of an ingoing principal null ray as measured by an observer with 4-velocity u,
    /// divided by the frequency the same ray has at infinity:
    ///
    ///     nu_obs / nu_inf = -k_mu u^mu = u^t + u^r - a u^phi
    ///
    /// Values above 1 are a blueshift, below 1 a redshift. This is the exact, frame-independent
    /// answer everywhere in the chart, including at r+ and at r-, where it stays finite and
    /// positive. Closed forms worth remembering (M = 1):
    ///
    ///   * raindrop (E = 1, L = 0) in Schwarzschild: 1 / (1 + sqrt(2M/r)), so exactly 1/2 at the
    ///     horizon: an ingoing observer *red*shifts the ingoing ray, because the Doppler term from
    ///     running away from it beats the gravitational blueshift;
    ///   * static observer: 1 / sqrt(1 - 2M/r), a blueshift diverging only at the static limit;
    ///   * ZAMO: gamma (1 - a omega) with omega = -g_tphi/g_phiphi.
    pub fn ingoing_frequency_ratio(&self, r: f64, u: &[f64; 3]) -> f64 {
        // k_mu = g_{mu nu} k^nu with k^nu = (1, -1, 0). The result is the constant covector of
        // `ingoing_null_covector`, but lowering it here keeps this in step with the coded metric.
        let g = self.metric_components(r);
        let k = [
            g[0][0] - g[0][1],
            g[1][0] - g[1][1],
            g[2][0] - g[2][1],
        ];
        -(k[0] * u[0] + k[1] * u[1] + k[2] * u[2])
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

    /// The conserved (E, L) of the equatorial circular geodesic at radius r, prograde (the sense
    /// of the spin) or retrograde, or None where no circular orbit exists.
    ///
    /// Bardeen, Press and Teukolsky (1972), with x = r/M and a in units of M:
    ///
    ///     E = (x^{3/2} - 2 x^{1/2} ± a) / (x^{3/4} sqrt(x^{3/2} - 3 x^{1/2} ± 2a))
    ///     L = ± M (x^2 ∓ 2a x^{1/2} + a^2) / (x^{3/4} sqrt(x^{3/2} - 3 x^{1/2} ± 2a))
    ///
    /// upper signs prograde. The square root's argument vanishes on the circular photon orbit,
    /// inside which no timelike circular orbit exists at any energy; between there and the ISCO
    /// the orbit exists but is unstable to the smallest push. These are the only thrust-free
    /// orbits there are, and both numbers are fixed by the radius once the sense is chosen. The
    /// coordinate time is Boyer-Lindquist's here, but a circular orbit has dr = 0 and the two
    /// charts differ by a function of r alone, so E, L and the angular velocity are the same in
    /// the ingoing chart the app integrates in.
    pub fn circular_orbit(&self, r: f64, prograde: bool) -> Option<(f64, f64)> {
        let m = self.m.max(1e-12);
        let x = r / m;
        if x.is_nan() || x <= 0.0 {
            return None;
        }
        let a = self.a / m;
        let sign = if prograde { 1.0 } else { -1.0 };
        let (x12, x32, x34) = (x.sqrt(), x.powf(1.5), x.powf(0.75));
        let under = x32 - 3.0 * x12 + sign * 2.0 * a;
        if under.is_nan() || under <= 0.0 {
            return None;
        }
        let root = x34 * under.sqrt();
        let energy = (x32 - 2.0 * x12 + sign * a) / root;
        let l_ang = sign * m * (x * x - sign * 2.0 * a * x12 + a * a) / root;
        (energy.is_finite() && l_ang.is_finite()).then_some((energy, l_ang))
    }

    /// Radius of the equatorial circular photon orbit, prograde or retrograde:
    /// r = 2M {1 + cos[(2/3) arccos(∓a/M)]}. Inside it no circular timelike orbit exists.
    pub fn photon_orbit(&self, prograde: bool) -> f64 {
        let a = (self.a / self.m.max(1e-12)).clamp(-1.0, 1.0);
        let arg = if prograde { -a } else { a };
        2.0 * self.m * (1.0 + (2.0 / 3.0 * arg.acos()).cos())
    }

    /// Radius of the innermost stable circular orbit, prograde or retrograde (Bardeen, Press and
    /// Teukolsky 1972): with Z1 = 1 + (1 - a^2)^{1/3} [(1 + a)^{1/3} + (1 - a)^{1/3}] and
    /// Z2 = sqrt(3 a^2 + Z1^2), r = M [3 + Z2 ∓ sqrt((3 - Z1)(3 + Z1 + 2 Z2))]. Six M for no spin,
    /// one M prograde and nine M retrograde at a = M.
    pub fn isco(&self, prograde: bool) -> f64 {
        let a = (self.a / self.m.max(1e-12)).clamp(-1.0, 1.0);
        let z1 = 1.0 + (1.0 - a * a).cbrt() * ((1.0 + a).cbrt() + (1.0 - a).cbrt());
        let z2 = (3.0 * a * a + z1 * z1).sqrt();
        let inner = ((3.0 - z1) * (3.0 + z1 + 2.0 * z2)).max(0.0).sqrt();
        let x = if prograde { 3.0 + z2 - inner } else { 3.0 + z2 + inner };
        x * self.m
    }

    /// Coordinate angular velocity dphi/dt of the circular orbit at r, prograde or retrograde:
    /// Ω = ± M^{1/2} / (r^{3/2} ± a M^{1/2}). None where the orbit does not exist.
    pub fn orbital_angular_velocity(&self, r: f64, prograde: bool) -> Option<f64> {
        self.circular_orbit(r, prograde)?;
        let sqrt_m = self.m.max(1e-12).sqrt();
        let sign = if prograde { 1.0 } else { -1.0 };
        Some(sign * sqrt_m / (r.powf(1.5) + sign * self.a * sqrt_m))
    }

    /// dt/dtau on the circular orbit at r: the dilation between the distant clock and the
    /// orbiting observer's own, from the norm of (1, 0, Ω) in the (t, phi) block of the metric,
    /// which is the same block in both charts since dr = 0.
    pub fn circular_orbit_dilation(&self, r: f64, prograde: bool) -> Option<f64> {
        let omega = self.orbital_angular_velocity(r, prograde)?;
        let g = self.metric_components(r);
        let norm = -(g[0][0] + 2.0 * g[0][2] * omega + g[2][2] * omega * omega);
        (norm > 0.0).then(|| 1.0 / norm.sqrt())
    }

    /// Cartesian radius rho of the chart radius r in the equatorial plane.
    ///
    /// The equatorial plane is embedded in Kerr-Schild Cartesian coordinates as
    /// x + i y = (r + i a) e^{i phi}, so a surface of constant r is *not* a circle of radius r
    /// but the circle rho = sqrt(r^2 + a^2). In particular the ring singularity r = 0 is the
    /// circle rho = a, and every constant-r surface (r+, r-, the static limit 2M) is drawn at
    /// its own rho.
    pub fn cartesian_radius(&self, r: f64) -> f64 {
        (r * r + self.a * self.a).sqrt()
    }

    /// Kerr-Schild Cartesian position of the equatorial chart point (r, phi):
    ///     x + i y = (r + i a) e^{i phi}
    ///     x = r cos phi - a sin phi,   y = r sin phi + a cos phi
    /// Its modulus is `cartesian_radius(r)` and its polar angle is psi = phi + atan2(a, r).
    pub fn cartesian_position(&self, r: f64, phi: f64) -> (f64, f64) {
        let (s, c) = phi.sin_cos();
        (r * c - self.a * s, r * s + self.a * c)
    }

    /// The equatorial chart point (r, phi) drawn at the Cartesian position (x, y): the inverse of
    /// `cartesian_position`, with the radius held at or above `r_floor`.
    ///
    /// x + iy = (r + ia)e^{i phi} has modulus sqrt(r^2 + a^2) and argument phi + atan2(a, r), so
    ///     r   = sqrt(rho^2 - a^2),   rho = hypot(x, y)
    ///     phi = atan2(y, x) - atan2(a, r)
    /// and the inversion is exact wherever it exists. It does not exist everywhere: rho < |a| is a
    /// disc of the drawn plane no equatorial point maps into - the ring is the circle rho = |a|,
    /// and the whole r > 0 equator lies outside it - so a point in there is read as the smallest
    /// radius allowed instead, at the azimuth it was pointing at. The floor is the caller's because
    /// it is a drawing decision: the ring is the end of every worldline that reaches it, and a drop
    /// onto r = 0 is a drop onto nothing there is a frame at.
    pub fn chart_point(&self, x: f64, y: f64, r_floor: f64) -> (f64, f64) {
        let a2 = self.a * self.a;
        let rho2 = x * x + y * y;
        let r = (rho2 - a2).max(0.0).sqrt().max(r_floor);
        (r, y.atan2(x) - self.a.atan2(r))
    }

    /// Jacobian of `cartesian_position`, i.e. the Cartesian velocity of a coordinate velocity
    /// (dr/dt, dphi/dt) at the chart point (r, phi):
    ///     d(x + i y)/dt = (dr/dt - a dphi/dt + i r dphi/dt) e^{i phi}
    /// so the screen direction is the CHART angle phi rotation of (dr/dt - a dphi/dt, r dphi/dt),
    /// not the polar angle psi rotation of (dr/dt, r dphi/dt).
    ///
    /// The ingoing principal null ray (dr/dt = -1, dphi/dt = 0) therefore maps to the direction
    /// -e^{i phi} at *every* radius: ingoing Kerr-Schild rays are straight lines in these
    /// Cartesian coordinates, and they run tangent to the ring singularity.
    pub fn cartesian_velocity(&self, r: f64, phi: f64, dr_dt: f64, dphi_dt: f64) -> (f64, f64) {
        let (s, c) = phi.sin_cos();
        let radial = dr_dt - self.a * dphi_dt;
        let tangential = r * dphi_dt;
        (radial * c - tangential * s, radial * s + tangential * c)
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

    /// Contravariant metric g^{mu nu} in equatorial Kerr-Schild coordinates (t, r, phi), in closed
    /// form. The 3-metric is regular (det g = -r^2) everywhere off the ring singularity, so the
    /// inverse exists at and inside both horizons; g^{rr} = Delta / r^2 changes sign there.
    ///
    /// The Kerr-Schild form is what makes the inverse elementary. Writing the metric as a flat
    /// background plus a null rank-one piece,
    ///
    ///     g_{mu nu} = eta_{mu nu} + 2 H l_mu l_nu,   H = M / r,   l_mu = (1, 1, -a),
    ///
    /// with eta the equatorial flat metric in these coordinates,
    ///
    ///     eta_{mu nu} = [[-1, 0, 0], [0, 1, -a], [0, -a, r^2 + a^2]],
    ///
    /// reproduces `metric_components` term by term. The covector l is null for eta (and hence for
    /// g), so raising it with eta gives l^mu = eta^{mu nu} l_nu = (-1, 1, 0) and the inverse is
    /// exactly the same rank-one correction with the opposite sign:
    ///
    ///     g^{mu nu} = eta^{mu nu} - 2 H l^mu l^nu.
    ///
    /// The (r, phi) block of eta has determinant r^2, so eta^{tt} = -1, eta^{rr} = (r^2 + a^2)/r^2,
    /// eta^{r phi} = a / r^2 and eta^{phi phi} = 1 / r^2, which leaves
    ///
    ///     g^{tt} = -(1 + 2M/r),   g^{tr} = 2M/r,        g^{t phi}   = 0,
    ///     g^{rr} = Delta / r^2,   g^{r phi} = a / r^2,  g^{phi phi} = 1 / r^2.
    ///
    /// The g^{rr} entry is `g_upper_rr` and carries the causal character of the surfaces r = const;
    /// g^{t phi} vanishing is the statement that the ingoing Kerr-Schild time function is dragged
    /// along with l rather than twisted against d_phi.
    pub fn inverse_metric(&self, r: f64) -> [[f64; 3]; 3] {
        let r = r.max(1e-6);
        let h = self.h_scalar(r);
        let a = self.a;
        let inv_r2 = 1.0 / (r * r);

        [
            [-(1.0 + 2.0 * h), 2.0 * h, 0.0],
            [2.0 * h, self.delta(r) * inv_r2, a * inv_r2],
            [0.0, a * inv_r2, inv_r2],
        ]
    }

    /// Christoffel symbols of the second kind, indexed [mu][alpha][beta] = Gamma^mu_{alpha beta}:
    ///     Gamma^mu_{alpha beta} = 1/2 g^{mu nu} (d_alpha g_{nu beta} + d_beta g_{nu alpha} - d_nu g_{alpha beta})
    /// Only d_r is non-zero because the equatorial Kerr-Schild metric is stationary and axisymmetric.
    /// The equatorial plane is totally geodesic, so these 3-metric symbols coincide with the
    /// projections of the full 4D ones for motion that stays in the plane.
    pub fn christoffel(&self, r: f64) -> [[[f64; 3]; 3]; 3] {
        let ginv = self.inverse_metric(r);
        // d_alpha g_{nu beta} vanishes unless alpha = 1, the r direction, so the three derivative
        // terms collapse to
        //     2 Gamma^mu_{alpha beta} = [alpha = r] c^mu_beta + [beta = r] c^mu_alpha
        //                               - g^{mu r} d_r g_{alpha beta},
        // with c^mu_beta = g^{mu nu} d_r g_{nu beta} contracted once and reused. Writing it this
        // way is the same algebra as the general formula, evaluated only where it is non-zero.
        let dg = self.metric_derivative_r(r);
        let mut c = [[0.0f64; 3]; 3];
        for (mu, row) in c.iter_mut().enumerate() {
            for (beta, entry) in row.iter_mut().enumerate() {
                *entry = ginv[mu][0] * dg[0][beta] + ginv[mu][1] * dg[1][beta] + ginv[mu][2] * dg[2][beta];
            }
        }

        let mut gamma = [[[0.0f64; 3]; 3]; 3];
        for (mu, block) in gamma.iter_mut().enumerate() {
            for (alpha, row) in block.iter_mut().enumerate() {
                for (beta, entry) in row.iter_mut().enumerate() {
                    let mut sum = -ginv[mu][1] * dg[alpha][beta];
                    if alpha == 1 {
                        sum += c[mu][beta];
                    }
                    if beta == 1 {
                        sum += c[mu][alpha];
                    }
                    *entry = 0.5 * sum;
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
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_the_chart_point_of_a_drawn_position_inverts_the_embedding() {
        // What a drag on the equatorial view has to do with the pixel under the pointer. The view
        // draws x + iy = (r + ia)e^{i phi}, so a screen position is a Cartesian point of that
        // plane and `chart_point` is the way back to the (r, phi) it came from. Exactly the way
        // back, wherever the way back exists: this round-trips every point of a grid over the
        // drawn plane and asks for the same chart point out.
        let metric = KerrSchild::new(1.0, 0.90);
        let floor = 0.04;
        let mut worst_r = 0.0f64;
        let mut worst_phi = 0.0f64;
        for k in 0..40 {
            for j in 0..40 {
                let r = 0.05 + 12.0 * f64::from(k) / 39.0;
                let phi = -std::f64::consts::PI + std::f64::consts::TAU * f64::from(j) / 39.0;
                let (x, y) = metric.cartesian_position(r, phi);
                let (back_r, back_phi) = metric.chart_point(x, y, floor);
                worst_r = worst_r.max((back_r - r).abs());
                let turn = std::f64::consts::TAU;
                let apart = (back_phi - phi).rem_euclid(turn);
                worst_phi = worst_phi.max(apart.min(turn - apart));
            }
        }
        println!(
            "1600 points of the drawn plane round-tripped: r to within {worst_r:.2e}, phi to \
             within {worst_phi:.2e} rad"
        );
        assert!(worst_r < 1e-9 && worst_phi < 1e-9, "the inversion is exact: {worst_r}, {worst_phi}");

        // The hole in the middle of the map. The ring is the circle rho = |a|, and the whole r > 0
        // equator lies outside it, so a pointer inside that disc is asking for a point that does
        // not exist: it gets the floor, at the azimuth it was pointing at, rather than a NaN.
        let inside = metric.a * 0.5;
        let (r, phi) = metric.chart_point(inside, 0.0, floor);
        assert!((r - floor).abs() < 1e-12, "inside the ring the radius floors: {r}");
        assert!(phi.is_finite(), "and the azimuth is still an angle: {phi}");
        let (r, _) = metric.chart_point(0.0, 0.0, floor);
        assert!((r - floor).abs() < 1e-12, "dead centre included: {r}");
        // And just outside it the radius comes back up off the floor continuously.
        let (r, _) = metric.chart_point(metric.a * 1.001, 0.0, floor);
        println!("rho = 1.001a gives r = {r:.4} against a floor of {floor}");
        assert!(r >= floor, "never below the floor: {r}");
    }

    use super::*;

    #[test]
    fn test_cartesian_embedding_radius_and_angle() {
        // |x + i y| = sqrt(r^2 + a^2) and arg(x + i y) = phi + atan2(a, r), for every (r, phi).
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in &[12.0, 3.0, 1.0, 0.3, 0.0] {
                for &phi in &[0.0, 0.7, 2.5, -1.3, 5.9] {
                    let (x, y) = ks.cartesian_position(r, phi);
                    let rho = (x * x + y * y).sqrt();
                    assert!(
                        (rho - ks.cartesian_radius(r)).abs() < 1e-12,
                        "rho = {rho} vs {} at r={r} (a={a})",
                        ks.cartesian_radius(r)
                    );
                    if rho > 1e-12 {
                        let psi = y.atan2(x);
                        let expected = phi + a.atan2(r);
                        let d = (psi - expected).sin().abs();
                        assert!(d < 1e-12, "psi = {psi} vs {expected} at r={r} phi={phi} (a={a})");
                    }
                }
            }
            // The ring singularity r = 0 sits at Cartesian radius exactly a.
            assert!((ks.cartesian_radius(0.0) - a).abs() < 1e-15);
        }
    }

    #[test]
    fn test_cartesian_velocity_matches_finite_difference() {
        for &a in &[0.0, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &r in &[9.0, 4.0, 1.0, 0.2] {
                for &phi in &[0.0, 1.1, -2.2] {
                    for &(dr_dt, dphi_dt) in &[(-1.0, 0.0), (0.5, 0.3), (0.0, -0.8), (-0.7, 0.2)] {
                        let h = 1e-6;
                        let (xp, yp) = ks.cartesian_position(r + h * dr_dt, phi + h * dphi_dt);
                        let (xm, ym) = ks.cartesian_position(r - h * dr_dt, phi - h * dphi_dt);
                        let fd = ((xp - xm) / (2.0 * h), (yp - ym) / (2.0 * h));
                        let got = ks.cartesian_velocity(r, phi, dr_dt, dphi_dt);
                        assert!(
                            (got.0 - fd.0).abs() < 1e-8 && (got.1 - fd.1).abs() < 1e-8,
                            "v = {got:?} vs FD {fd:?} at r={r} phi={phi} (a={a})"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_ingoing_null_ray_is_a_straight_line_tangent_to_the_ring() {
        // (dr/dt, dphi/dt) = (-1, 0) maps to -e^{i phi} at every radius: the ingoing principal
        // null rays are straight lines in Kerr-Schild Cartesian coordinates.
        for &a in &[0.0, 0.5, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            for &phi in &[0.0f64, 0.9, -2.0, 4.4] {
                let expected = (-phi.cos(), -phi.sin());
                for &r in &[20.0, 6.0, 2.0, 0.5, 0.01, 0.0] {
                    let v = ks.cartesian_velocity(r, phi, -1.0, 0.0);
                    assert!(
                        (v.0 - expected.0).abs() < 1e-14 && (v.1 - expected.1).abs() < 1e-14,
                        "ingoing ray direction {v:?} vs {expected:?} at r={r} (a={a})"
                    );
                }
                // At r = 0 the ray is tangent to the ring: its direction is perpendicular to the
                // position vector, which there has modulus a.
                let p = ks.cartesian_position(0.0, phi);
                let v = ks.cartesian_velocity(0.0, phi, -1.0, 0.0);
                assert!(
                    (p.0 * v.0 + p.1 * v.1).abs() < 1e-14,
                    "ray must be tangent to the ring: p={p:?} v={v:?} (a={a})"
                );
            }
        }
    }

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
    fn test_inner_surface_gravity_matches_the_horizon_derivative_of_delta() {
        // kappa_- = |Delta'(r-)| / (2 (r-^2 + a^2)) is the definition; (r+ - r-) / (2 (r-^2 + a^2))
        // is the closed form the code uses. Delta' is taken by a central difference here, so the
        // check does not just restate the same algebra.
        for &a in &[0.2, 0.5, 0.65, 0.9, 0.99] {
            let ks = KerrSchild::new(1.0, a);
            let rm = ks.inner_horizon();
            let h = 1e-6;
            let d_prime = (ks.delta(rm + h) - ks.delta(rm - h)) / (2.0 * h);
            let expected = d_prime.abs() / (2.0 * (rm * rm + a * a));
            let got = ks.inner_surface_gravity();
            assert!(
                (got - expected).abs() < 1e-6 * (1.0 + expected),
                "kappa_- = {got} vs |Delta'|/(2(r-^2+a^2)) = {expected} (a={a})"
            );
        }
        // The reference value quoted throughout the app.
        let ks = KerrSchild::new(1.0, 0.65);
        assert!(
            (ks.inner_surface_gravity() - 1.583).abs() < 1e-3,
            "kappa_-(a=0.65) = {}",
            ks.inner_surface_gravity()
        );
        // Without spin there is no inner horizon at all, and the formula diverges.
        assert!(!KerrSchild::new(1.0, 0.0).inner_surface_gravity().is_finite());
    }

    #[test]
    fn test_inner_horizon_omega_is_the_generators_angular_velocity() {
        // Omega_- is defined by what it does, not by the expression that computes it: chi =
        // d_t + Omega_- d_phi has to be the *null* Killing direction on r = r-, which is checked
        // here by taking g(chi, chi) with the coded metric at the coded radius. The closed form
        // a / (2 M r-) is the second statement, and it is a different expression: it leans on
        // Delta(r-) = 0, i.e. r-^2 + a^2 = 2 M r-, so agreeing with it is a check that the two
        // horizon radii and this rate are all solving the same Delta.
        for &a in &[0.0, 0.65, 0.90, 0.998] {
            let ks = KerrSchild::new(1.0, a);
            let rm = ks.inner_horizon();
            let omega = ks.inner_horizon_omega();
            if a == 0.0 {
                // No inner horizon: chi degenerates to d_t and there is nothing to co-rotate with.
                assert_eq!(omega, 0.0, "a hole with no spin has no generator to turn");
                continue;
            }
            let closed = a / (2.0 * ks.m * rm);
            assert!(
                (omega - closed).abs() < 1e-12,
                "Omega_- = {omega} vs a/(2 M r-) = {closed} (a={a})"
            );
            let chi = [1.0, 0.0, omega];
            let null = ks.norm(rm, &chi);
            assert!(
                null.abs() < 1e-10,
                "g(chi, chi) = {null} must vanish on r- = {rm} (a={a})"
            );
            // And it is the only such rate: the null condition is a quadratic in Omega with a
            // double root here, so moving off it by a little makes chi spacelike either way.
            for &d in &[-1e-3, 1e-3] {
                let off = ks.norm(rm, &[1.0, 0.0, omega + d]);
                assert!(off > 0.0, "chi at Omega_- {d:+} is spacelike, got {off} (a={a})");
            }
        }
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
    fn test_ingoing_null_covector_is_the_lowered_ingoing_ray() {
        // k_mu = g_{mu nu} (1, -1, 0)^nu must equal the constant (-1, -1, a) at every radius.
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let ks = KerrSchild::new(1.0, a);
            let expected = ks.ingoing_null_covector();
            assert_eq!(expected, [-1.0, -1.0, a]);
            for &r in probe_radii(&ks).iter() {
                let g = ks.metric_components(r);
                let k_up = [1.0, -1.0, 0.0];
                for mu in 0..3 {
                    let mut lowered = 0.0;
                    for nu in 0..3 {
                        lowered += g[mu][nu] * k_up[nu];
                    }
                    assert!(
                        (lowered - expected[mu]).abs() < 1e-10,
                        "k_{mu} = {lowered} vs {} at r={r} (a={a})",
                        expected[mu]
                    );
                }
                // -k_mu k^mu must vanish: the ray is null, so its "frequency" for itself is zero.
                assert!(ks.ingoing_frequency_ratio(r, &k_up).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_circular_orbits_of_a_schwarzschild_hole_match_their_closed_forms() {
        // With no spin the two senses are the same orbit run the other way:
        // E = (1 - 2M/r) / sqrt(1 - 3M/r), L = ± sqrt(M r) / sqrt(1 - 3M/r), the photon orbit
        // at 3M and the ISCO at 6M.
        let ks = KerrSchild::new(1.0, 0.0);
        for &r in &[3.5f64, 4.0, 6.0, 10.0, 50.0] {
            let want_e = (1.0 - 2.0 / r) / (1.0 - 3.0 / r).sqrt();
            let want_l = r.sqrt() / (1.0 - 3.0 / r).sqrt();
            let (e, l) = ks.circular_orbit(r, true).expect("a circular orbit outside 3M");
            assert!((e - want_e).abs() < 1e-12, "E({r}) = {e} vs {want_e}");
            assert!((l - want_l).abs() < 1e-12, "L({r}) = {l} vs {want_l}");
            let (e_r, l_r) = ks.circular_orbit(r, false).expect("and retrograde");
            assert!((e_r - want_e).abs() < 1e-12 && (l_r + want_l).abs() < 1e-12);
            let omega = ks.orbital_angular_velocity(r, true).unwrap();
            assert!((omega - r.powf(-1.5)).abs() < 1e-12, "Kepler's law: {omega}");
        }
        assert!(ks.circular_orbit(2.9, true).is_none(), "nothing inside the photon orbit");
        assert!((ks.photon_orbit(true) - 3.0).abs() < 1e-12);
        assert!((ks.isco(true) - 6.0).abs() < 1e-12 && (ks.isco(false) - 6.0).abs() < 1e-12);
    }

    #[test]
    fn test_kerr_circular_orbits_have_the_textbook_radii_and_sit_on_the_apps_own_potential() {
        // The extreme hole's famous numbers, and a consistency check on the two functions the
        // app already has: a circular orbit is a turning point of R(r), so its E is exactly the
        // energy floor at that L, and a geodesic started there with the app's own integrator
        // stays at that radius.
        use crate::physics::geodesic::GeodesicState;
        // The constructor stops the spin a hair short of extremal (a = 0.9999 M), which moves the
        // prograde radii off their extremal values of exactly M by a few percent and leaves the
        // retrograde ones, which are insensitive there, at 9 M and 4 M.
        let extreme = KerrSchild::new(1.0, 1.0);
        println!(
            "a = {}: ISCO {:.4} / {:.4} M, photon orbit {:.4} / {:.4} M (prograde / retrograde)",
            extreme.a, extreme.isco(true), extreme.isco(false), extreme.photon_orbit(true), extreme.photon_orbit(false)
        );
        assert!(extreme.isco(true) > 1.0 && extreme.isco(true) < 1.1, "prograde ISCO near M");
        assert!((extreme.isco(false) - 9.0).abs() < 0.02, "retrograde ISCO at 9M");
        assert!(extreme.photon_orbit(true) > 1.0 && extreme.photon_orbit(true) < 1.05);
        assert!((extreme.photon_orbit(false) - 4.0).abs() < 0.01);

        let ks = KerrSchild::new(1.0, 0.9);
        let (isco_p, isco_r) = (ks.isco(true), ks.isco(false));
        println!("a = 0.9: prograde ISCO {isco_p:.4} M, retrograde {isco_r:.4} M");
        assert!(isco_p > 2.3 && isco_p < 2.35 && isco_r > 8.6 && isco_r < 8.8);
        for (r, prograde) in [(6.0, true), (3.0, true), (12.0, false)] {
            let (e, l) = ks.circular_orbit(r, prograde).expect("stable orbits at these radii");
            let floor = GeodesicState::energy_floor(&ks, r, l);
            assert!((e - floor).abs() < 1e-9, "E = {e} is the floor V(r, L) = {floor} at r = {r}");
            let mut geo = GeodesicState::new_infall(&ks, 0.0, r, e, l);
            let period = std::f64::consts::TAU / ks.orbital_angular_velocity(r, prograde).unwrap().abs();
            let mut t = 0.0;
            let mut worst = 0.0f64;
            while t < 2.0 * period {
                geo.step_coord_time(&ks, 0.05);
                t += 0.05;
                worst = worst.max((geo.r - r).abs());
            }
            println!("r = {r} {}: two periods of {period:.2} M, radius wandered {worst:.2e} M", if prograde { "prograde" } else { "retrograde" });
            assert!(worst < 1e-3, "a circular orbit stays circular: {worst}");
            let dilation = ks.circular_orbit_dilation(r, prograde).unwrap();
            assert!(dilation > 1.0 && dilation.is_finite(), "dt/dtau = {dilation}");
        }
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

