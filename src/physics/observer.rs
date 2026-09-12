use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::tetrad::Tetrad;

/// How an observer's worldline is generated. Every mode pins down a contravariant 4-velocity
/// u^mu at the observer's current event, and all telemetry (coordinate velocity, proper velocity,
/// proper acceleration) is derived from that single object rather than from per-mode formulas.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObserverMode {
    /// Timelike geodesic: ingoing free fall with conserved energy E and angular momentum L.
    /// Exists everywhere in the equatorial plane, including at and inside both horizons, and has
    /// identically zero proper acceleration (that is what "geodesic" means).
    FreeFall,
    /// Worldline positioned by the user's mouse. Its 4-velocity is the boost of the local
    /// *raindrop* frame (the E = 1, L = 0 ingoing geodesic, which exists at every r > 0) by the
    /// local velocity (beta_r, beta_phi). beta = 0 reproduces free fall exactly; any other beta is
    /// a rocket, and carries the proper acceleration that keeping it up requires.
    ManualDrag,
    /// Static observer: fixed r *and* fixed phi, u^mu = (1, 0, 0) / sqrt(-g_tt).
    /// The Killing vector d/dt is timelike only outside the equatorial static limit, so this
    /// observer exists only where g_tt = -(1 - 2M/r) < 0, i.e. r > 2M. Inside the ergosphere
    /// frame dragging makes "holding phi fixed" a spacelike motion, so no rocket can do it.
    Static,
    /// Zero-angular-momentum observer (ZAMO): fixed r, but swept around by frame dragging at
    /// omega(r) = -g_tphi / g_phiphi, so that u_phi = 0. A fixed-r worldline is timelike only where
    /// the r direction is spacelike, i.e. outside the outer horizon r+ (between r+ and r- the
    /// coordinate r is timelike and nothing can hover). Unlike the static observer, the ZAMO
    /// survives the whole ergosphere 2M > r > r+.
    Zamo,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct Observer {
    pub name: String,
    pub mode: ObserverMode,
    /// Coordinate time t
    pub t: f64,
    /// Radius r
    pub r: f64,
    /// Azimuth phi in radians
    pub phi: f64,
    /// Accumulated proper time tau
    pub tau: f64,
    /// Local rest frame radial boost beta_r in (-0.99, 0.99)
    pub beta_r: f64,
    /// Local rest frame azimuthal boost beta_phi in (-0.99, 0.99)
    pub beta_phi: f64,
    /// Geodesic state for automated infall simulation
    pub geodesic: Option<GeodesicState>,
    /// History of (t, r) positions for drawing worldline trail
    pub trail: Vec<[f64; 2]>,
    /// Release coordinate time t_release (e.g. 0 for Alice, delta_t for Bob)
    pub release_t: f64,
    /// Is the observer active/released yet?
    pub is_active: bool,
}

impl Observer {
    pub fn new(name: &str, start_t: f64, start_r: f64, release_t: f64) -> Self {
        Self::new_with_phi(name, start_t, start_r, release_t, 0.0)
    }

    pub fn new_with_phi(name: &str, start_t: f64, start_r: f64, release_t: f64, start_phi: f64) -> Self {
        let release_t = release_t.max(start_t);
        let mut geodesic = GeodesicState::new_infall(release_t, start_r, 1.0, 0.0);
        geodesic.phi = start_phi;
        let mut obs = Self {
            name: name.to_string(),
            mode: ObserverMode::FreeFall,
            t: start_t,
            r: start_r,
            phi: start_phi,
            tau: 0.0,
            beta_r: 0.0,
            beta_phi: 0.0,
            geodesic: Some(geodesic),
            trail: Vec::new(),
            release_t,
            is_active: start_t >= release_t,
        };
        obs.trail.push([start_t, start_r]);
        obs
    }

    /// Reset observer back to initial conditions
    pub fn reset(&mut self, start_t: f64, start_r: f64) {
        self.reset_with_phi(start_t, start_r, 0.0);
    }

    /// Reset observer with initial radius, coordinate time, and azimuth phi
    pub fn reset_with_phi(&mut self, start_t: f64, start_r: f64, start_phi: f64) {
        self.t = start_t;
        self.r = start_r;
        self.phi = start_phi;
        self.tau = 0.0;
        self.beta_r = 0.0;
        self.beta_phi = 0.0;
        self.release_t = self.release_t.max(start_t);
        let mut geo = GeodesicState::new_infall(self.release_t, start_r, 1.0, 0.0);
        geo.phi = start_phi;
        self.geodesic = Some(geo);
        self.trail.clear();
        self.trail.push([start_t, start_r]);
        self.is_active = self.t >= self.release_t;
    }

    /// Set position directly from user mouse dragging
    pub fn set_drag_position(&mut self, t: f64, r: f64) {
        self.mode = ObserverMode::ManualDrag;
        self.t = t;
        self.r = r.max(0.01);
        self.is_active = true;
        if self.trail.len() > 500 {
            self.trail.remove(0);
        }
        self.trail.push([t, self.r]);
    }

    /// While waiting for release the observer hovers at fixed (r, phi): coordinate time follows the
    /// simulation clock and proper time ticks at the static-observer rate sqrt(-g_tt) dt.
    /// The worldline is therefore a vertical segment that turns into the infall curve at t = release_t.
    fn hover(&mut self, metric: &KerrSchild, current_sim_time: f64, dt: f64) {
        self.is_active = false;
        self.t = current_sim_time;
        let g_tt = metric.metric_components(self.r)[0][0];
        if g_tt < 0.0 {
            self.tau += (-g_tt).sqrt() * dt.max(0.0);
        }
        if let Some(ref mut geo) = self.geodesic {
            geo.t = self.release_t;
            geo.tau = self.tau;
        }
        // Keep the trail as [start point, current hover point]
        self.trail.truncate(1);
        self.trail.push([self.t, self.r]);
    }

    /// Kerr-Schild Cartesian azimuth psi of the observer, x + i y = (r + i a) e^{i phi}.
    /// Use this (not the raw chart angle phi) when plotting a top-down (x, y) view.
    pub fn azimuth(&self, metric: &KerrSchild) -> f64 {
        self.phi + metric.a.atan2(self.r.max(1e-9))
    }

    /// Can the currently selected mode exist at the observer's radius?
    /// A static observer needs a timelike d/dt (g_tt < 0, i.e. r > 2M on the equator); a ZAMO
    /// needs a timelike fixed-r worldline, which only exists outside the outer horizon r+.
    /// Free fall and manual drag are always admissible.
    pub fn mode_admissible(&self, metric: &KerrSchild) -> bool {
        Self::mode_admissible_at(self.mode, metric, self.r)
    }

    fn mode_admissible_at(mode: ObserverMode, metric: &KerrSchild, r: f64) -> bool {
        match mode {
            ObserverMode::Static => metric.metric_components(r)[0][0] < 0.0,
            ObserverMode::Zamo => r > metric.outer_horizon(),
            ObserverMode::FreeFall | ObserverMode::ManualDrag => true,
        }
    }

    /// Contravariant 4-velocity u^mu = (u^t, u^r, u^phi) at the observer's current event,
    /// normalised so that g_{mu nu} u^mu u^nu = -1.
    ///
    /// If the selected mode cannot exist at the current radius (see `mode_admissible`) the
    /// free-fall 4-velocity is returned instead, so the display never shows a spacelike "observer".
    pub fn four_velocity(&self, metric: &KerrSchild) -> [f64; 3] {
        self.four_velocity_at(metric, self.r)
    }

    /// `four_velocity` for the same family of worldlines evaluated at an arbitrary radius.
    /// Because the metric depends on r alone, every mode's components are functions of r only,
    /// which is what makes the 4-acceleration computable by differentiating in r.
    fn four_velocity_at(&self, metric: &KerrSchild, r: f64) -> [f64; 3] {
        let r = r.max(1e-4);
        match self.mode {
            ObserverMode::Static if Self::mode_admissible_at(ObserverMode::Static, metric, r) => {
                // u^mu = (1, 0, 0) / sqrt(-g_tt): the normalised time-translation Killing vector.
                let g_tt = metric.metric_components(r)[0][0];
                [1.0 / (-g_tt).sqrt(), 0.0, 0.0]
            }
            ObserverMode::Zamo if Self::mode_admissible_at(ObserverMode::Zamo, metric, r) => {
                // u^mu = gamma (1, 0, omega) with omega = -g_tphi/g_phiphi (so u_phi = 0) and
                // gamma = 1 / sqrt(-(g_tt + 2 omega g_tphi + omega^2 g_phiphi)).
                let g = metric.metric_components(r);
                let omega = metric.frame_dragging_omega(r);
                let norm_sq = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
                let gamma = 1.0 / norm_sq.max(1e-14).sqrt();
                [gamma, 0.0, gamma * omega]
            }
            ObserverMode::ManualDrag => {
                // The dragged observer is defined as a *boost of the local raindrop frame*:
                // u = gamma (e0 + beta_r e1 + beta_phi e2) built on the orthonormal tetrad of the
                // E = 1, L = 0 ingoing geodesic at this radius. (beta_r, beta_phi) is therefore
                // the observer's velocity, as a fraction of c, relative to an observer dropped
                // from rest at infinity and passing through the same event.
                //
                // The raindrop frame is the reference because it exists at every r > 0, including
                // between the horizons where no static or ZAMO frame exists; beta = 0 reproduces
                // free fall exactly, and any beta != 0 is a rocket with real proper acceleration.
                Self::raindrop_tetrad(metric, r).boost(self.beta_r, self.beta_phi)
            }
            // An inadmissible Static / Zamo selection lands here and rides free fall instead.
            _ => self.free_fall_four_velocity(metric, r),
        }
    }

    /// Orthonormal tetrad of the raindrop (E = 1, L = 0 ingoing geodesic) observer at radius r.
    fn raindrop_tetrad(metric: &KerrSchild, r: f64) -> Tetrad {
        let raindrop = GeodesicState::new_infall(0.0, r, 1.0, 0.0);
        let (dt_dtau, dr_dtau, dphi_dtau) = raindrop.derivatives(metric, r);
        Tetrad::from_four_velocity(metric, r, &[dt_dtau, dr_dtau, dphi_dtau])
    }

    /// The observer's own orthonormal frame: the Gram-Schmidt tetrad built on this observer's
    /// 4-velocity. Light leaves the observer isotropically in this frame, so
    /// `tetrad.null_direction(alpha)` sweeps the local null cone as alpha runs over [0, 2 pi).
    pub fn tetrad(&self, metric: &KerrSchild) -> Tetrad {
        Tetrad::from_four_velocity(metric, self.r, &self.four_velocity(metric))
    }

    /// Exact ingoing-geodesic 4-velocity at radius r for this observer's conserved (E, L).
    fn free_fall_four_velocity(&self, metric: &KerrSchild, r: f64) -> [f64; 3] {
        let geo = self
            .geodesic
            .unwrap_or_else(|| GeodesicState::new_infall(self.t, r, 1.0, 0.0));
        let (dt_dtau, dr_dtau, dphi_dtau) = geo.derivatives(metric, r);
        [dt_dtau, dr_dtau, dphi_dtau]
    }

    /// Proper 4-acceleration a^mu = du^mu/dtau + Gamma^mu_{alpha beta} u^alpha u^beta.
    ///
    /// Static and ZAMO observers sit at fixed r and their components depend on r alone, so
    /// du^mu/dtau = 0 and only the connection term survives: their acceleration is exactly the
    /// thrust needed to resist gravity. For free fall (and, for now, manual drag) u^r != 0, and
    /// since u^mu = u^mu(r) along the worldline, du^mu/dtau = (du^mu/dr) u^r. The r derivative is
    /// taken by a central difference with step h ~ 1e-5 max(r, 0.1), using the 5-point stencil
    /// (-u(r+2h) + 8u(r+h) - 8u(r-h) + u(r-2h)) / 12h; its O(h^4) truncation error keeps the
    /// residual below 1e-7 even at r ~ r-/2, where u varies on the scale of r itself. The two
    /// terms then cancel to numerical noise, which is the statement that the coded geodesic
    /// really is a geodesic of the coded metric.
    pub fn four_acceleration(&self, metric: &KerrSchild) -> [f64; 3] {
        let u = self.four_velocity(metric);
        let gamma = metric.christoffel(self.r);

        let mut accel = [0.0f64; 3];
        for mu in 0..3 {
            let mut sum = 0.0;
            for alpha in 0..3 {
                for beta in 0..3 {
                    sum += gamma[mu][alpha][beta] * u[alpha] * u[beta];
                }
            }
            accel[mu] = sum;
        }

        // du^mu/dtau = (du^mu/dr) u^r; identically zero for the fixed-r modes.
        if u[1] != 0.0 {
            let h = 1e-5 * self.r.max(0.1);
            let u_p1 = self.four_velocity_at(metric, self.r + h);
            let u_p2 = self.four_velocity_at(metric, self.r + 2.0 * h);
            let u_m1 = self.four_velocity_at(metric, self.r - h);
            let u_m2 = self.four_velocity_at(metric, self.r - 2.0 * h);
            for mu in 0..3 {
                let du_dr =
                    (-u_p2[mu] + 8.0 * u_p1[mu] - 8.0 * u_m1[mu] + u_m2[mu]) / (12.0 * h);
                accel[mu] += du_dr * u[1];
            }
        }

        accel
    }

    /// Magnitude sqrt(g_{mu nu} a^mu a^nu) of the proper acceleration, in geometric units (1/M).
    /// The 4-acceleration of a timelike worldline is spacelike, so the radicand is non-negative
    /// up to round-off; it is clamped at zero.
    pub fn proper_acceleration_geom(&self, metric: &KerrSchild) -> f64 {
        let accel = self.four_acceleration(metric);
        metric.norm(self.r, &accel).max(0.0).sqrt()
    }

    /// Is this worldline weightless, i.e. a geodesic?
    /// The comparison is against a curvature-relative floor rather than an absolute one: the
    /// residual left by the finite-difference du^mu/dtau grows with the local curvature scale
    /// sqrt(K) = sqrt(48) M / r^3, and so does every honest acceleration near the singularity.
    pub fn is_free_falling(&self, metric: &KerrSchild) -> bool {
        let curvature_scale = metric.kretschmann_scalar(self.r).sqrt().max(1e-12);
        self.proper_acceleration_geom(metric) < 1e-6 * curvature_scale
    }

    /// Advance simulation by coordinate time delta dt
    pub fn step(&mut self, metric: &KerrSchild, current_sim_time: f64, dt: f64) {
        if current_sim_time < self.release_t {
            self.hover(metric, current_sim_time, dt);
            return;
        }
        self.is_active = true;

        match self.mode {
            ObserverMode::FreeFall => {
                if let Some(ref mut geo) = self.geodesic {
                    if geo.r > 0.02 {
                        geo.step_coord_time(metric, dt);
                        self.t = geo.t;
                        self.r = geo.r;
                        self.phi = geo.phi;
                        self.tau = geo.tau;

                        if self.trail.len() > 800 {
                            self.trail.remove(0);
                        }
                        self.trail.push([self.t, self.r]);
                    }
                }
            }
            ObserverMode::ManualDrag => {
                // Keep manual position, just advance t slightly if playing
                self.t += dt;
            }
            ObserverMode::Static | ObserverMode::Zamo => {
                // Fixed r: advance along u^mu. dphi/dt = u^phi/u^t (zero for Static, the
                // frame-dragging rate omega for the ZAMO) and dtau/dt = 1/u^t.
                let u = self.four_velocity(metric);
                let ut = u[0].max(1e-9);
                self.t += dt;
                self.phi += (u[2] / ut) * dt;
                self.tau += dt / ut;
            }
        }
    }

    /// Step observer backward in time. Pops from recorded worldline trail if available,
    /// or integrates backward with negative step.
    pub fn step_back(&mut self, metric: &KerrSchild, dt: f64) {
        if self.trail.len() > 1 {
            self.trail.pop();
            if let Some(&[prev_t, prev_r]) = self.trail.last() {
                self.t = prev_t;
                self.r = prev_r;
                if let Some(ref mut geo) = self.geodesic {
                    // Rewind the geodesic clock to the recorded event; tau is re-derived from dtau/dt.
                    let (dt_dtau, _, _) = geo.derivatives(metric, prev_r);
                    geo.tau = (geo.tau - (geo.t - prev_t).abs() / dt_dtau.max(1e-6)).max(0.0);
                    geo.t = prev_t;
                    geo.r = prev_r;
                    self.tau = geo.tau;
                }
                return;
            }
        }
        // Fallback backward step
        match self.mode {
            ObserverMode::FreeFall => {
                if let Some(ref mut geo) = self.geodesic {
                    let dtau = -(dt.abs() * 0.5).min(0.05);
                    geo.step(metric, dtau);
                    self.t = geo.t;
                    self.r = geo.r;
                    self.phi = geo.phi;
                    self.tau = geo.tau;
                }
            }
            ObserverMode::ManualDrag | ObserverMode::Static | ObserverMode::Zamo => {
                self.t = (self.t - dt.abs()).max(0.0);
            }
        }
    }

    /// Generate polygon coordinates for the light cone at the observer's event on the (t, r)
    /// diagram. `time_height`: height in coordinate time units to extend the cone upward (future)
    /// and downward (past).
    ///
    /// The wedge drawn here is `KerrSchild::null_wedge`, the projection of the full null cone onto
    /// the (t, r) plane. That projection belongs to the *event*, not to the observer: boosting the
    /// observer re-labels which local angle alpha emits the extreme ray, but the extreme values of
    /// dr/dt are unchanged. So the cone drawn in the diagram deliberately does not depend on the
    /// observer's mode or on (beta_r, beta_phi).
    pub fn compute_lightcone_polygon(
        &self,
        metric: &KerrSchild,
        time_height: f64,
    ) -> LightConePolygon {
        let r0 = self.r;
        let t0 = self.t;

        let wedge = metric.null_wedge(r0);
        let (dr_dt_in, dphi_dt_in) = (wedge.dr_dt_in, wedge.dphi_dt_in);
        let (dr_dt_out, dphi_dt_out) = (wedge.dr_dt_out, wedge.dphi_dt_out);

        // Future cone endpoints at t = t0 + time_height.
        // If a ray reaches r = 0 before time_height, terminate the ray at the singularity
        // preserving exact dr/dt slope rather than clamping r with fixed t.
        let (t_fut_in, r_fut_in) = if dr_dt_in < -1e-7 && (r0 + dr_dt_in * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_in.abs();
            (t0 + dt_sing, 0.0)
        } else {
            (t0 + time_height, (r0 + dr_dt_in * time_height).max(0.0))
        };

        let (t_fut_out, r_fut_out) = if dr_dt_out < -1e-7 && (r0 + dr_dt_out * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_out.abs();
            (t0 + dt_sing, 0.0)
        } else {
            (t0 + time_height, (r0 + dr_dt_out * time_height).max(0.0))
        };

        // Past cone endpoints at t = t0 - time_height
        let (t_pst_in, r_pst_in) = if dr_dt_in > 1e-7 && (r0 - dr_dt_in * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_in;
            (t0 - dt_sing, 0.0)
        } else {
            (t0 - time_height, (r0 - dr_dt_in * time_height).max(0.0))
        };

        let (t_pst_out, r_pst_out) = if dr_dt_out > 1e-7 && (r0 - dr_dt_out * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_out;
            (t0 - dt_sing, 0.0)
        } else {
            (t0 - time_height, (r0 - dr_dt_out * time_height).max(0.0))
        };

        LightConePolygon {
            apex: [t0, r0],
            future_in: [t_fut_in, r_fut_in],
            future_out: [t_fut_out, r_fut_out],
            past_in: [t_pst_in, r_pst_in],
            past_out: [t_pst_out, r_pst_out],
            dr_dt_in,
            dr_dt_out,
            dphi_dt_out,
            dphi_dt_in,
        }
    }

    /// Instantaneous radial coordinate velocity dr/dt = u^r / u^t as a fraction of c, in the
    /// global Kerr-Schild foliation. Always inside the local light cone (dr/dt > -1), because
    /// the ingoing principal null ray travels at exactly dr/dt = -1 in this chart.
    pub fn velocity_c(&self, metric: &KerrSchild) -> f64 {
        let u = self.four_velocity(metric);
        u[1] / u[0].max(1e-9)
    }

    /// Instantaneous proper radial velocity dr/dtau = u^r. Unbounded: it exceeds 1.0c inside the
    /// horizon, where r is a time coordinate and no local frame can hold r still.
    pub fn proper_velocity_c(&self, metric: &KerrSchild) -> f64 {
        self.four_velocity(metric)[1]
    }

    /// Physical radial velocity in km/s
    pub fn velocity_km_s(&self, metric: &KerrSchild) -> f64 {
        self.velocity_c(metric) * 299792.458
    }

    /// Proper acceleration felt by the observer in Earth g's (weightlessness = 0.0), for every
    /// mode. This is |a^mu| in geometric units (1/M) converted with a_SI = a_geom c^2 / r_g,
    /// r_g = GM/c^2 in metres, then divided by 9.80665 m/s^2.
    pub fn proper_acceleration_g(&self, metric: &KerrSchild) -> f64 {
        let accel_geom = self.proper_acceleration_geom(metric);
        let c = 299792458.0;
        let rg_m = metric.r_grav_km() * 1000.0;
        (accel_geom * c * c / rg_m) / 9.80665
    }

    /// Radial tidal stretching force across a 2-meter body in Earth g's
    #[allow(dead_code)]
    pub fn tidal_force_g(&self, metric: &KerrSchild) -> f64 {
        metric.tidal_acceleration_g(self.r, 2.0)
    }

    /// Radial tidal gradient in Earth gravities per meter (g/m)
    pub fn tidal_gradient_g_per_m(&self, metric: &KerrSchild) -> f64 {
        metric.tidal_gradient_g_per_m(self.r)
    }

    /// Frequency this observer measures for an ingoing principal null ray, as a multiple of the
    /// frequency the same ray has at infinity: nu_obs / nu_inf = -k_mu u^mu = u^t + u^r - a u^phi.
    ///
    /// This is the honest replacement for the old exterior-time heuristic. It is exact for every
    /// mode, and finite and positive everywhere in this chart, r+ and r- included: an infalling
    /// observer sees the exterior universe *red*shifted (1/2 at the Schwarzschild horizon for a
    /// raindrop), not squeezed into a flash.
    pub fn ingoing_frequency_ratio(&self, metric: &KerrSchild) -> f64 {
        metric.ingoing_frequency_ratio(self.r, &self.four_velocity(metric))
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct LightConePolygon {
    pub apex: [f64; 2],
    pub future_in: [f64; 2],
    pub future_out: [f64; 2],
    pub past_in: [f64; 2],
    pub past_out: [f64; 2],
    pub dr_dt_in: f64,
    pub dr_dt_out: f64,
    pub dphi_dt_out: f64,
    pub dphi_dt_in: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observer_manual_drag_and_cone() {
        // The drawn cone is the zero-angular-momentum wedge of the event, so it must reproduce
        // `null_wedge` exactly and tip according to the sign of Delta, in every region.
        let metric = KerrSchild::new(1.0, 0.7);
        let mut bob = Observer::new("Bob", 0.0, 3.0, 0.0);

        let check = |cone: &LightConePolygon, r: f64| {
            let w = metric.null_wedge(r);
            assert!((cone.dr_dt_in - w.dr_dt_in).abs() < 1e-12);
            assert!((cone.dr_dt_out - w.dr_dt_out).abs() < 1e-12);
            assert!((cone.dphi_dt_in - w.dphi_dt_in).abs() < 1e-12);
            assert!((cone.dphi_dt_out - w.dphi_dt_out).abs() < 1e-12);
            assert!(cone.dr_dt_in <= -1.0 + 1e-12, "inner edge = {}", cone.dr_dt_in);
        };

        // Region I: outgoing edge is positive.
        let cone_out = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_out, 3.0);
        assert!(cone_out.dr_dt_out > 0.0);

        // Region II (between r- = 0.286 and r+ = 1.714): trapped, outgoing edge dr/dt < 0.
        bob.set_drag_position(5.0, 1.0);
        let cone_in = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_in, 1.0);
        assert!(cone_in.dr_dt_out < 0.0);

        // Region III (r < 0.286): un-tipped again, outgoing edge dr/dt > 0.
        bob.set_drag_position(8.0, 0.15);
        let cone_core = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_core, 0.15);
        assert!(cone_core.dr_dt_out > 0.0);

        // The wedge belongs to the event: thrusting must not change it.
        bob.set_drag_position(8.0, 3.0);
        let rest = bob.compute_lightcone_polygon(&metric, 1.0);
        bob.beta_r = 0.8;
        bob.beta_phi = -0.4;
        let boosted = bob.compute_lightcone_polygon(&metric, 1.0);
        assert_eq!(rest.dr_dt_in, boosted.dr_dt_in);
        assert_eq!(rest.dr_dt_out, boosted.dr_dt_out);
    }

    #[test]
    fn test_observer_tetrads_are_orthonormal_everywhere() {
        // Every mode the UI can select must hand `Tetrad::from_four_velocity` a unit timelike
        // vector, and the resulting frame must be exactly orthonormal in all three regions.
        use crate::physics::tetrad::inner;
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let radii = [8.0, 3.0, rp, 0.5 * (rp + rm), rm, (0.5 * rm).max(0.05)];
            for &r in radii.iter() {
                for &mode in &[
                    ObserverMode::FreeFall,
                    ObserverMode::ManualDrag,
                    ObserverMode::Static,
                    ObserverMode::Zamo,
                ] {
                    let mut obs = observer_at(mode, r);
                    if mode == ObserverMode::ManualDrag {
                        obs.beta_r = 0.5;
                        obs.beta_phi = -0.3;
                    }
                    let u = obs.four_velocity(&metric);
                    let uu = metric.norm(r, &u);
                    assert!((uu + 1.0).abs() < 1e-9, "u.u = {uu} for {mode:?} at r={r} (a={a})");

                    let t = obs.tetrad(&metric);
                    let legs = [t.e0, t.e1, t.e2];
                    for i in 0..3 {
                        for j in 0..3 {
                            let expected = match (i == j, i) {
                                (true, 0) => -1.0,
                                (true, _) => 1.0,
                                (false, _) => 0.0,
                            };
                            let got = inner(&metric, r, &legs[i], &legs[j]);
                            assert!(
                                (got - expected).abs() < 1e-9,
                                "g(e{i}, e{j}) = {got} (want {expected}) for {mode:?} at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_manual_drag_zero_boost_is_free_fall() {
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            for &r in &[9.0, 3.0, rp, 0.5 * (rp + rm), (0.5 * rm).max(0.05)] {
                let drag = observer_at(ObserverMode::ManualDrag, r);
                let free = observer_at(ObserverMode::FreeFall, r);
                let ud = drag.four_velocity(&metric);
                let uf = free.four_velocity(&metric);
                for mu in 0..3 {
                    assert!(
                        (ud[mu] - uf[mu]).abs() < 1e-10 * (1.0 + uf[mu].abs()),
                        "beta = 0 must reproduce free fall: u^{mu} {ud:?} vs {uf:?} at r={r} (a={a})"
                    );
                }
                // ...and a weightless one at that.
                assert!(drag.is_free_falling(&metric), "unboosted drag at r={r} (a={a})");
            }
        }
    }

    #[test]
    fn test_manual_drag_boost_stays_inside_the_null_wedge() {
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        for &r in &[8.0, 3.0, rp, 0.5 * (rp + rm), 0.5 * rm] {
            let wedge = metric.null_wedge(r);
            for &(b_r, b_phi) in &[
                (0.0, 0.0),
                (0.5, 0.0),
                (-0.5, 0.0),
                (0.9, 0.0),
                (-0.9, 0.0),
                (0.5, -0.3),
                (-0.4, 0.7),
            ] {
                let mut obs = observer_at(ObserverMode::ManualDrag, r);
                obs.beta_r = b_r;
                obs.beta_phi = b_phi;
                let u = obs.four_velocity(&metric);
                let n = metric.norm(r, &u);
                assert!((n + 1.0).abs() < 1e-9, "u.u = {n} for beta=({b_r},{b_phi}) at r={r}");
                assert!(u[0] > 0.0, "must move forward in t: {u:?}");

                let v = obs.velocity_c(&metric);
                assert!(
                    v > wedge.dr_dt_in && v < wedge.dr_dt_out,
                    "dr/dt = {v} escapes the wedge ({}, {}) for beta=({b_r},{b_phi}) at r={r}",
                    wedge.dr_dt_in,
                    wedge.dr_dt_out
                );
            }
        }
    }

    #[test]
    fn test_manual_drag_boost_costs_proper_acceleration() {
        // Because `four_acceleration` differentiates `four_velocity_at` in r, a boosted drag
        // observer automatically picks up the thrust its worldline family requires.
        let metric = KerrSchild::with_solar_mass(1.0, 0.65, 10.0);
        let r = 3.0;

        let mut boosted = observer_at(ObserverMode::ManualDrag, r);
        boosted.beta_r = 0.5;
        let a_boost = boosted.proper_acceleration_geom(&metric);
        assert!(a_boost.is_finite() && a_boost > 0.0, "|a| = {a_boost} for beta_r = 0.5");
        assert!(!boosted.is_free_falling(&metric));

        let rest = observer_at(ObserverMode::ManualDrag, r);
        let a_rest = rest.proper_acceleration_geom(&metric);
        assert!(a_rest < 1e-6, "|a| = {a_rest} for beta = 0 must vanish");
        assert!(rest.is_free_falling(&metric));
    }

    #[test]
    fn test_ingoing_frequency_ratio_schwarzschild_raindrop() {
        // Raindrop (E = 1, L = 0) in Schwarzschild: u^t = 1 + 2M/r ... but the invariant answer is
        // nu_obs / nu_inf = 1 / (1 + sqrt(2M/r)), exactly 1/2 at the horizon. The infaller
        // *red*shifts the ingoing ray: running away from it beats the gravitational blueshift.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 4.0, 2.0, 1.0, 0.3] {
            let obs = observer_at(ObserverMode::FreeFall, r);
            let got = obs.ingoing_frequency_ratio(&metric);
            let expected = 1.0 / (1.0 + (2.0 * metric.m / r).sqrt());
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs {expected} at r={r}"
            );
            assert!(got < 1.0, "an ingoing raindrop must see a redshift, got {got} at r={r}");
        }
        // The horizon value is exactly 1/2, and the ratio -> 0 at the singularity.
        let at_horizon = observer_at(ObserverMode::FreeFall, 2.0).ingoing_frequency_ratio(&metric);
        assert!((at_horizon - 0.5).abs() < 1e-12, "at r+ = 2M the ratio is 1/2: {at_horizon}");
        let deep = observer_at(ObserverMode::FreeFall, 0.01).ingoing_frequency_ratio(&metric);
        assert!(deep > 0.0 && deep < 0.1, "ratio -> 0 as r -> 0, got {deep}");
    }

    #[test]
    fn test_ingoing_frequency_ratio_static_observer() {
        // Static observer: u^mu = (1, 0, 0)/sqrt(-g_tt) and a u^phi = 0, so the ratio is
        // 1 / sqrt(1 - 2M/r): a blueshift that diverges only at the static limit r = 2M.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 4.0, 2.5] {
            let obs = observer_at(ObserverMode::Static, r);
            assert!(obs.mode_admissible(&metric));
            let got = obs.ingoing_frequency_ratio(&metric);
            let expected = 1.0 / (1.0 - 2.0 * metric.m / r).sqrt();
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs {expected} at r={r}"
            );
            assert!(got > 1.0, "a hovering observer must see a blueshift, got {got} at r={r}");
        }
    }

    #[test]
    fn test_ingoing_frequency_ratio_zamo() {
        // ZAMO: u^mu = gamma (1, 0, omega), so the ratio is gamma (1 - a omega).
        let metric = KerrSchild::new(1.0, 0.9);
        for &r in &[3.0, 5.0] {
            let obs = observer_at(ObserverMode::Zamo, r);
            assert!(obs.mode_admissible(&metric));
            let u = obs.four_velocity(&metric);
            let omega = metric.frame_dragging_omega(r);
            let gamma = u[0];
            let expected = gamma * (1.0 - metric.a * omega);
            let got = obs.ingoing_frequency_ratio(&metric);
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs gamma (1 - a omega) = {expected} at r={r}"
            );
            assert!(got > 0.0 && got.is_finite());
        }
    }

    #[test]
    fn test_ingoing_frequency_ratio_kerr_raindrop_is_finite_across_both_horizons() {
        // There is no divergence on the branch of r- that an infalling observer actually crosses:
        // the ratio stays finite and positive at r+, between the horizons, at r-, and below it.
        for &a in &[0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[rp, 0.5 * (rp + rm), rm, 0.5 * rm] {
                let ratio = observer_at(ObserverMode::FreeFall, r).ingoing_frequency_ratio(&metric);
                assert!(
                    ratio.is_finite() && ratio > 0.0,
                    "nu ratio = {ratio} at r={r} (a={a}) must be finite and positive"
                );
                assert!(ratio < 1.0, "an infaller still sees a redshift: {ratio} at r={r} (a={a})");
            }
            // Closed form away from the horizons, where Delta != 0. With E = 1, L = 0 the only
            // non-trivial covariant component is u_r = (2Mr - sqrt(2Mr(r^2 + a^2))) / Delta, and
            // nu_obs/nu_inf = -k.u = E + u_r (because k^mu = (1, -1, 0) gives k^mu u_mu = u_t - u_r
            // and u_t = -E). Its numerator has the opposite sign to Delta at every radius, which is
            // why an infaller always measures a redshift, inside the horizons included.
            let closed_form = |r: f64| {
                let m = metric.m;
                1.0 + (2.0 * m * r - (2.0 * m * r * (r * r + a * a)).sqrt()) / metric.delta(r)
            };
            for &r in &[9.0, 3.0, 0.5 * (rp + rm), 0.5 * rm, 0.05] {
                let got = observer_at(ObserverMode::FreeFall, r).ingoing_frequency_ratio(&metric);
                assert!(
                    (got - closed_form(r)).abs() < 1e-10,
                    "nu ratio = {got} vs {} at r={r} (a={a})",
                    closed_form(r)
                );
            }
            // Deep inside, the a = 0 raindrop's ratio runs to 0, but with spin Delta -> a^2 and the
            // shift runs back up to 1: the ring's repulsion, not a blueshift catastrophe.
            let deep = observer_at(ObserverMode::FreeFall, 0.02).ingoing_frequency_ratio(&metric);
            assert!(deep > 0.5 && deep < 1.0, "spun-up core ratio -> 1, got {deep} (a={a})");
        }
    }

    #[test]
    fn test_observer_velocity_and_acceleration_telemetry() {
        let metric = KerrSchild::with_solar_mass(1.0, 0.7, 10.0);
        let obs = Observer::new("Bob", 0.0, 3.0, 0.0);

        let v_c = obs.velocity_c(&metric);
        assert!(v_c.abs() < 1.0, "Velocity fraction of c magnitude must be < 1: {}", v_c);
        assert!(v_c < 0.0, "Inward infalling observer must have negative radial velocity: {}", v_c);

        let v_kms = obs.velocity_km_s(&metric);
        assert!(v_kms.abs() <= 300_000.0, "Velocity magnitude in km/s must be <= c: {}", v_kms);
        assert!(v_kms < 0.0, "Inward velocity in km/s must be negative: {}", v_kms);

        // A geodesic observer is weightless: the proper acceleration vanishes to the accuracy of
        // the central difference used for du^mu/dtau (see `four_acceleration`).
        let a_geom = obs.proper_acceleration_geom(&metric);
        assert!(
            a_geom < 1e-6,
            "Free-falling geodesic observer must have zero proper acceleration, got {a_geom}/M"
        );

        let tidal = obs.tidal_force_g(&metric);
        assert!(tidal > 0.0, "Tidal force must be positive");

        let grad = obs.tidal_gradient_g_per_m(&metric);
        assert!(grad > 0.0, "Tidal gradient must be positive");
        assert!((grad * 2.0 - tidal).abs() < 1e-6, "2-meter tidal force should be 2x the 1-meter gradient");

        // Verify inside horizon dynamics: coordinate velocity dr/dt stays within light cone, proper velocity dr/dtau exceeds -1.0c
        let obs_inside = Observer::new("Bob", 0.0, 1.0, 0.0);
        let v_c_inside = obs_inside.velocity_c(&metric);
        let u_inside = obs_inside.proper_velocity_c(&metric);
        assert!(v_c_inside < 0.0 && v_c_inside > -1.0, "Coordinate velocity dr/dt stays causal: {}", v_c_inside);
        assert!(u_inside < -1.0, "Proper velocity dr/dtau exceeds -1.0c inside horizon: {}", u_inside);
    }

    #[test]
    fn test_delayed_release_worldline_starts_at_release_event() {
        // Bob released at t = 4 must sit at t = 4 (not t = 0) when he starts falling, so his
        // worldline is NOT a copy of Alice's shifted down the diagram.
        let metric = KerrSchild::new(1.0, 0.65);
        let mut alice = Observer::new("Alice", 0.0, 4.5, 0.0);
        let mut bob = Observer::new("Bob", 0.0, 4.5, 4.0);
        let dt = 0.05;
        let mut t = 0.0;
        while t < 3.95 {
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
        }
        assert!(!bob.is_active);
        assert!((bob.t - t).abs() < 1e-9, "hovering Bob must track the simulation clock");
        assert!((bob.r - 4.5).abs() < 1e-12);
        assert!(bob.tau > 0.0, "a hovering observer's clock still runs");
        while t < 6.0 {
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
        }
        assert!(bob.is_active);
        assert!((bob.t - t).abs() < 1e-9, "Bob's coordinate time must equal the simulation clock: {} vs {}", bob.t, t);
        assert!(bob.r > alice.r + 0.5, "Bob released later must trail Alice: bob.r={} alice.r={}", bob.r, alice.r);
        // First trail point is the initial hover event, then the release event follows.
        assert_eq!(bob.trail[0], [0.0, 4.5]);
        assert!((bob.trail[1][0] - 4.0).abs() < 0.06, "trail must show the release event near t=4: {:?}", bob.trail[1]);
    }

    #[test]
    fn test_observer_step_back() {
        let metric = KerrSchild::new(1.0, 0.6);
        let mut obs = Observer::new("Bob", 0.0, 4.0, 0.0);
        let initial_r = obs.r;

        // Step forward 5 times
        for i in 1..=5 {
            obs.step(&metric, (i as f64) * 0.1, 0.1);
        }
        assert!(obs.r < initial_r, "Observer should fall inward");
        let forward_r = obs.r;

        // Step back
        obs.step_back(&metric, 0.1);
        assert!(obs.r > forward_r, "Stepping backward must restore previous outward radius");
    }

    /// Build an observer parked at radius r in the given mode (E = 1, L = 0 free-fall data).
    fn observer_at(mode: ObserverMode, r: f64) -> Observer {
        let mut obs = Observer::new("Probe", 0.0, r, 0.0);
        obs.mode = mode;
        obs
    }

    #[test]
    fn test_static_observer_acceleration_schwarzschild() {
        // Schwarzschild static observer: |a| = M / (r^2 sqrt(1 - 2M/r)).
        let metric = KerrSchild::new(1.0, 0.0);
        let r = 4.0;
        let obs = observer_at(ObserverMode::Static, r);
        assert!(obs.mode_admissible(&metric));

        let u = obs.four_velocity(&metric);
        assert!(u[1] == 0.0 && u[2] == 0.0, "static observer must not move: {u:?}");
        assert!((metric.norm(r, &u) + 1.0).abs() < 1e-12, "u.u = {}", metric.norm(r, &u));

        let expected = metric.m / (r * r * (1.0 - 2.0 * metric.m / r).sqrt());
        let got = obs.proper_acceleration_geom(&metric);
        assert!((got - expected).abs() < 1e-8, "|a| = {got} vs {expected}");
    }

    #[test]
    fn test_static_observer_acceleration_kerr() {
        // Equatorial Kerr static observer: |a| = sqrt(Delta) M / (r^2 (r - 2M)).
        let metric = KerrSchild::new(1.0, 0.65);
        let r = 5.0;
        let obs = observer_at(ObserverMode::Static, r);
        assert!(obs.mode_admissible(&metric));
        assert!((metric.norm(r, &obs.four_velocity(&metric)) + 1.0).abs() < 1e-12);

        let expected = metric.delta(r).sqrt() * metric.m / (r * r * (r - 2.0 * metric.m));
        let got = obs.proper_acceleration_geom(&metric);
        assert!((got - expected).abs() < 1e-8, "|a| = {got} vs {expected}");
    }

    #[test]
    fn test_zamo_reduces_to_static_without_spin() {
        // With a = 0 there is no frame dragging, so the ZAMO is the static observer.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 6.0, 4.0, 2.5, 2.05] {
            let stat = observer_at(ObserverMode::Static, r);
            let zamo = observer_at(ObserverMode::Zamo, r);
            assert!(stat.mode_admissible(&metric) && zamo.mode_admissible(&metric));
            let us = stat.four_velocity(&metric);
            let uz = zamo.four_velocity(&metric);
            for mu in 0..3 {
                assert!((us[mu] - uz[mu]).abs() < 1e-12, "u^{mu} differs at r={r}: {us:?} {uz:?}");
            }
            let a_s = stat.proper_acceleration_geom(&metric);
            let a_z = zamo.proper_acceleration_geom(&metric);
            assert!((a_s - a_z).abs() < 1e-12, "|a| differs at r={r}: {a_s} vs {a_z}");
        }
    }

    #[test]
    fn test_zamo_acceleration_is_the_lapse_gradient() {
        // For an equatorial ZAMO the proper acceleration is the gradient of the lapse:
        //     |a| = sqrt(g^rr) d(ln alpha)/dr,   alpha^2 = Delta r^2 / ((r^2 + a^2)^2 - a^2 Delta).
        let metric = KerrSchild::new(1.0, 0.9);
        let ln_alpha = |r: f64| {
            let a2 = metric.a * metric.a;
            let d = metric.delta(r);
            let alpha_sq = d * r * r / ((r * r + a2) * (r * r + a2) - a2 * d);
            0.5 * alpha_sq.ln()
        };

        for &r in &[3.0, 5.0, 8.0] {
            let zamo = observer_at(ObserverMode::Zamo, r);
            assert!(zamo.mode_admissible(&metric));

            let u = zamo.four_velocity(&metric);
            let n = metric.norm(r, &u);
            assert!((n + 1.0).abs() < 1e-10, "ZAMO u.u = {n} at r={r}");
            assert!(u[1] == 0.0 && u[2] > 0.0, "ZAMO must co-rotate at fixed r: {u:?}");

            let got = zamo.proper_acceleration_geom(&metric);
            assert!(got.is_finite() && got > 0.0, "|a| = {got} at r={r}");

            let h = 1e-5 * r;
            let dln = (ln_alpha(r + h) - ln_alpha(r - h)) / (2.0 * h);
            let expected = metric.g_upper_rr(r).sqrt() * dln;
            assert!((got - expected).abs() < 1e-6, "|a| = {got} vs lapse gradient {expected} at r={r}");
        }
    }

    #[test]
    fn test_free_fall_worldline_is_a_geodesic_of_the_coded_metric() {
        // a^mu = du^mu/dtau + Gamma^mu_{ab} u^a u^b must vanish for the exact infall solution,
        // everywhere including at r+ and deep inside the Cauchy horizon.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[6.0, 3.0, rp, 1.0, (0.5 * rm).max(0.05)] {
                let obs = observer_at(ObserverMode::FreeFall, r);
                let u = obs.four_velocity(&metric);
                assert!((metric.norm(r, &u) + 1.0).abs() < 1e-8, "u.u at r={r} (a={a})");

                let acc = obs.four_acceleration(&metric);
                let biggest = acc.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                assert!(biggest < 1e-6, "a^mu = {acc:?} at r={r} (a={a})");
                assert!(
                    metric.norm(r, &acc).abs() < 1e-6,
                    "|a|^2 = {} at r={r} (a={a})",
                    metric.norm(r, &acc)
                );
                assert!(obs.proper_acceleration_geom(&metric) < 1e-6);
            }
        }
    }

    #[test]
    fn test_inadmissible_modes_fall_back_to_free_fall() {
        let metric = KerrSchild::new(1.0, 0.65);

        // r = 1.5 is inside the equatorial static limit 2M: no static observer exists.
        let stat = observer_at(ObserverMode::Static, 1.5);
        assert!(!stat.mode_admissible(&metric));
        let free = observer_at(ObserverMode::FreeFall, 1.5);
        assert_eq!(stat.four_velocity(&metric), free.four_velocity(&metric));

        // Inside r+ nothing can hover, so the ZAMO is inadmissible too.
        let inside = 0.5 * (metric.outer_horizon() + metric.inner_horizon());
        let zamo = observer_at(ObserverMode::Zamo, inside);
        assert!(!zamo.mode_admissible(&metric));
        let free_in = observer_at(ObserverMode::FreeFall, inside);
        assert_eq!(zamo.four_velocity(&metric), free_in.four_velocity(&metric));

        // ... but a ZAMO in the ergosphere (r+ < r < 2M), where no static observer exists, is fine.
        let ergo = 0.5 * (metric.outer_horizon() + 2.0 * metric.m);
        let zamo_ergo = observer_at(ObserverMode::Zamo, ergo);
        assert!(zamo_ergo.mode_admissible(&metric));
        assert!(!observer_at(ObserverMode::Static, ergo).mode_admissible(&metric));
    }

    #[test]
    fn test_static_and_zamo_stepping_uses_the_four_velocity() {
        let metric = KerrSchild::new(1.0, 0.8);
        let dt = 0.1;

        let mut stat = observer_at(ObserverMode::Static, 5.0);
        let u_s = stat.four_velocity(&metric);
        stat.step(&metric, dt, dt);
        assert!((stat.r - 5.0).abs() < 1e-15, "static observer must not move radially");
        assert!(stat.phi.abs() < 1e-15, "static observer must not rotate");
        assert!((stat.tau - dt / u_s[0]).abs() < 1e-12);

        let mut zamo = observer_at(ObserverMode::Zamo, 5.0);
        let u_z = zamo.four_velocity(&metric);
        zamo.step(&metric, dt, dt);
        assert!((zamo.r - 5.0).abs() < 1e-15);
        let omega = metric.frame_dragging_omega(5.0);
        assert!((zamo.phi - omega * dt).abs() < 1e-12, "ZAMO must drift at omega");
        assert!((zamo.tau - dt / u_z[0]).abs() < 1e-12);
    }

    #[test]
    fn test_static_observer_acceleration_diverges_at_the_static_limit() {
        // Hovering costs more and more thrust as r -> 2M, and becomes impossible below it.
        let metric = KerrSchild::new(1.0, 0.5);
        let far = observer_at(ObserverMode::Static, 20.0).proper_acceleration_geom(&metric);
        let near = observer_at(ObserverMode::Static, 2.01).proper_acceleration_geom(&metric);
        assert!(near > far * 100.0, "near = {near}, far = {far}");
        // ZAMO acceleration stays finite through the ergosphere and diverges only at r+.
        let ergo = observer_at(ObserverMode::Zamo, 2.0).proper_acceleration_geom(&metric);
        assert!(ergo.is_finite() && ergo > 0.0, "ZAMO |a| at the static limit = {ergo}");
    }

    #[test]
    fn test_is_free_falling_separates_geodesics_from_hovering() {
        // The telemetry label "(Free Fall)" is driven by this predicate, so it must not be fooled
        // by the numerical residual of a geodesic, nor call a hovering rocket weightless.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::with_solar_mass(1.0, a, 10.0);
            let rp = metric.outer_horizon();
            for &r in &[20.0, 6.0, 3.0, rp, 1.0, 0.2] {
                assert!(
                    observer_at(ObserverMode::FreeFall, r).is_free_falling(&metric),
                    "geodesic at r={r} (a={a}) must read as weightless"
                );
            }
            for &r in &[20.0, 6.0, 2.5] {
                let stat = observer_at(ObserverMode::Static, r);
                assert!(!stat.is_free_falling(&metric), "hovering at r={r} (a={a}) is not free fall");
                assert!(stat.proper_acceleration_g(&metric) > 1.0);
            }
        }
    }
}
