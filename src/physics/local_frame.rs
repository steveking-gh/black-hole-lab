use crate::physics::kerr_schild::KerrSchild;
use crate::physics::tetrad::Tetrad;

/// A straight line in the drawn (xi^1, xi^0) plane of the observer's local frame: the point is
/// (xi^1, xi^0) and the direction is a Euclidean-unit vector in the same ordering, so
/// `dir[1] / dir[0]` is the diagram slope d xi^0 / d xi^1 and |slope| = 1 is a 45 degree ray.
#[derive(Debug, Clone, Copy)]
pub struct LocalLine {
    /// A point on the line, as (xi^1, xi^0) = (local outward distance, local time).
    pub point: [f64; 2],
    /// Unit direction along the line, as (d xi^1, d xi^0).
    pub dir: [f64; 2],
}

/// Causal character of a surface as read off the line it draws in the local frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceCharacter {
    /// Steeper than 45 degrees: the surface is timelike, so a rocket can stay off it.
    Timelike,
    /// Exactly 45 degrees: the surface is null; the observer is crossing it now.
    Null,
    /// Flatter than 45 degrees: the surface is spacelike, a moment of the observer's history.
    Spacelike,
}

#[allow(dead_code)]
impl LocalLine {
    /// Diagram slope d xi^0 / d xi^1, infinite for a line of constant xi^1.
    pub fn slope(&self) -> f64 {
        if self.dir[0] == 0.0 {
            f64::INFINITY
        } else {
            self.dir[1] / self.dir[0]
        }
    }

    /// Causal character of the line, i.e. of the surface it depicts: the Minkowski norm of the
    /// (Euclidean-unit) direction, -d0^2 + d1^2, is negative for a timelike line. `tol` is the
    /// half-width of the null band in that norm; it lies in [0, 1].
    pub fn character(&self, tol: f64) -> SurfaceCharacter {
        let norm = -self.dir[1] * self.dir[1] + self.dir[0] * self.dir[0];
        if norm.abs() <= tol {
            SurfaceCharacter::Null
        } else if norm < 0.0 {
            SurfaceCharacter::Timelike
        } else {
            SurfaceCharacter::Spacelike
        }
    }

    /// Local time xi^0 at which the line crosses the observer's own worldline xi^1 = 0.
    /// `None` for a line of constant xi^1 that misses the worldline entirely.
    pub fn xi0_at_axis(&self) -> Option<f64> {
        if self.dir[0] == 0.0 {
            return None;
        }
        Some(self.point[1] - self.point[0] * self.dir[1] / self.dir[0])
    }
}

/// The local inertial chart of an observer, to first order in the displacement from their event.
///
/// An orthonormal tetrad {e_a} defines local Minkowski coordinates through its dual basis: for a
/// coordinate displacement Delta x^mu away from the observer's event,
///
///     xi^a = e^a_mu Delta x^mu,      e^a_mu = eta^{ab} g_{mu nu} e_b^nu,   eta = diag(-1, 1, 1)
///
/// with xi^0 the local time, xi^1 the local outward radial distance and xi^2 the local azimuthal
/// distance. The map is linear, so it takes every coordinate plane to a plane and every light cone
/// to the 45 degree cone; that is what makes the picture drawable.
///
/// It is a *first-order* chart. The tetrad is exact at the observer's own event, so orientations
/// there - the tilt of the light cone, the causal character of a surface r = const, the direction
/// of another worldline - are exact. Finite offsets (where the other observer is drawn, how far
/// away a horizon is) are the linearised answer, accurate while the offset is small compared with
/// the curvature radius. The header banner of the view says so.
#[derive(Debug, Clone, Copy)]
pub struct LocalFrame {
    tetrad: Tetrad,
    /// dual[a][mu] = e^a_mu, the covariant legs.
    dual: [[f64; 3]; 3],
    /// Radius of the observer's event, the origin of the chart.
    r: f64,
}

#[allow(dead_code)]
impl LocalFrame {
    /// Build the chart of an arbitrary orthonormal tetrad carried at radius r.
    pub fn new(metric: &KerrSchild, r: f64, tetrad: Tetrad) -> Self {
        let r = r.max(1e-4);
        let g = metric.metric_components(r);
        let legs = [tetrad.e0, tetrad.e1, tetrad.e2];
        // eta^{ab} is diagonal: the timelike row picks up a minus sign, the spacelike ones do not.
        let eta = [-1.0, 1.0, 1.0];
        let mut dual = [[0.0f64; 3]; 3];
        for a in 0..3 {
            for mu in 0..3 {
                let mut s = 0.0;
                for nu in 0..3 {
                    s += g[mu][nu] * legs[a][nu];
                }
                dual[a][mu] = eta[a] * s;
            }
        }
        Self { tetrad, dual, r }
    }

    /// The chart an observer with 4-velocity u uses for the (time, radius) diagram: the tetrad is
    /// `Tetrad::from_four_velocity_axial`, whose azimuthal leg is tangent to the surfaces
    /// r = const, so the drawn plane xi^2 = 0 is transverse to them and `surface_r_const` returns
    /// their exact causal character.
    pub fn for_observer(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> Self {
        let tetrad = Tetrad::from_four_velocity_axial(metric, r, u);
        Self::new(metric, r, tetrad)
    }

    pub fn tetrad(&self) -> &Tetrad {
        &self.tetrad
    }

    /// e^a_mu, the covariant legs of the tetrad.
    pub fn dual(&self) -> &[[f64; 3]; 3] {
        &self.dual
    }

    pub fn r(&self) -> f64 {
        self.r
    }

    /// Local coordinates xi^a = e^a_mu Delta x^mu of a coordinate displacement (dt, dr, dphi).
    pub fn to_local(&self, dx: &[f64; 3]) -> [f64; 3] {
        let mut xi = [0.0f64; 3];
        for a in 0..3 {
            for mu in 0..3 {
                xi[a] += self.dual[a][mu] * dx[mu];
            }
        }
        xi
    }

    /// The same linear map applied to a vector (a 4-velocity, say) rather than to a displacement:
    /// v^a = e^a_mu v^mu.
    pub fn vector_to_local(&self, v: &[f64; 3]) -> [f64; 3] {
        self.to_local(v)
    }

    /// The line drawn by the surface r = r_h in the (xi^1, xi^0) plane.
    ///
    /// A displacement stays on the surface iff its r-component is Delta r = r_h - r_obs. Writing
    /// Delta x^mu = xi^a e_a^mu, that condition reads
    ///
    ///     n_a xi^a = Delta r,      n_a = e_a^mu (dr)_mu = e_a^r,
    ///
    /// so the surface maps to an affine plane whose normal is the covector dr in local components.
    /// The view draws the slice xi^2 = 0 of that plane, the line n_0 xi^0 + n_1 xi^1 = Delta r,
    /// anchored here at its point closest to the origin. Its direction (n_0, -n_1) annihilates
    /// (n_0, n_1), and its slope is d xi^0 / d xi^1 = -n_1 / n_0.
    ///
    /// With the `for_observer` gauge n_2 = 0, so -n_0^2 + n_1^2 = eta^{ab} n_a n_b = g^rr: the
    /// drawn line is steeper than 45 degrees exactly where the surface is timelike (g^rr > 0), at
    /// 45 degrees on either horizon (Delta = 0), and flatter where it is spacelike. Nothing about
    /// the tilt is put in by hand.
    pub fn surface_r_const(&self, r_h: f64) -> LocalLine {
        let n0 = self.tetrad.e0[1];
        let n1 = self.tetrad.e1[1];
        let d_r = r_h - self.r;

        let norm_sq = n0 * n0 + n1 * n1;
        if norm_sq < 1e-300 {
            // dr has no component in the drawn plane at all: nothing sensible to draw, so return
            // the observer's own worldline. Unreachable for a timelike u (n_0 = n_1 = 0 forces
            // g^rr = -n_2^2 <= 0 with dr tangent to the frame's azimuthal leg).
            return LocalLine {
                point: [0.0, 0.0],
                dir: [0.0, 1.0],
            };
        }

        let point = [d_r * n1 / norm_sq, d_r * n0 / norm_sq];
        let inv = norm_sq.sqrt();
        let mut dir = [n0 / inv, -n1 / inv];
        // Canonical orientation: rightward, or upward for a line of constant xi^1.
        if dir[0] < 0.0 || (dir[0] == 0.0 && dir[1] < 0.0) {
            dir = [-dir[0], -dir[1]];
        }
        LocalLine { point, dir }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::geodesic::GeodesicState;
    use std::f64::consts::PI;

    fn raindrop(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let (ut, ur, up) = GeodesicState::new_infall(0.0, r, 1.0, 0.0).derivatives(metric, r);
        [ut, ur, up]
    }

    fn zamo(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g = metric.metric_components(r);
        let omega = metric.frame_dragging_omega(r);
        let n = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma = 1.0 / n.sqrt();
        [gamma, 0.0, gamma * omega]
    }

    fn static_obs(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g_tt = metric.metric_components(r)[0][0];
        [1.0 / (-g_tt).sqrt(), 0.0, 0.0]
    }

    /// Radii spanning every region: exterior, r+, between the horizons, r-, inside r-.
    fn probe_radii(metric: &KerrSchild) -> Vec<f64> {
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon().max(0.05);
        vec![8.0, 3.0, rp, 0.5 * (rp + rm), rm, (0.5 * rm).max(0.05)]
    }

    #[test]
    fn test_dual_tetrad_is_the_inverse_of_the_tetrad() {
        // e^a_mu e_b^mu = delta^a_b, for every admissible frame in every region.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            for &r in probe_radii(&metric).iter() {
                let mut velocities = vec![("free fall", raindrop(&metric, r))];
                if r > rp {
                    velocities.push(("ZAMO", zamo(&metric, r)));
                }
                if metric.metric_components(r)[0][0] < 0.0 {
                    velocities.push(("static", static_obs(&metric, r)));
                }
                for (name, u) in velocities {
                    for frame in [
                        LocalFrame::new(&metric, r, Tetrad::from_four_velocity(&metric, r, &u)),
                        LocalFrame::for_observer(&metric, r, &u),
                    ] {
                        let legs = [frame.tetrad.e0, frame.tetrad.e1, frame.tetrad.e2];
                        for a_idx in 0..3 {
                            for b in 0..3 {
                                let mut s = 0.0;
                                for mu in 0..3 {
                                    s += frame.dual[a_idx][mu] * legs[b][mu];
                                }
                                let expected = if a_idx == b { 1.0 } else { 0.0 };
                                assert!(
                                    (s - expected).abs() < 1e-10,
                                    "e^{a_idx}_mu e_{b}^mu = {s} (want {expected}) for {name} at r={r} (a={a})"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_observer_is_at_rest_and_light_is_isotropic_in_the_local_chart() {
        // u maps to (1, 0, 0) and the null cone maps to (1, cos alpha, sin alpha): the observer is
        // at rest at the origin and light moves at c = 1 in every direction.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in probe_radii(&metric).iter() {
                let u = raindrop(&metric, r);
                let frame = LocalFrame::for_observer(&metric, r, &u);
                let v = frame.vector_to_local(&u);
                assert!(
                    (v[0] - 1.0).abs() < 1e-9 && v[1].abs() < 1e-9 && v[2].abs() < 1e-9,
                    "u maps to {v:?} at r={r} (a={a})"
                );
                for i in 0..16 {
                    let alpha = 2.0 * PI * (i as f64) / 16.0;
                    let k = frame.tetrad().null_direction(alpha);
                    let xi = frame.vector_to_local(&k);
                    let (s, c) = alpha.sin_cos();
                    assert!(
                        (xi[0] - 1.0).abs() < 1e-9 && (xi[1] - c).abs() < 1e-9 && (xi[2] - s).abs() < 1e-9,
                        "k(alpha={alpha}) maps to {xi:?} at r={r} (a={a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_surface_r_const_passes_through_the_mapped_radial_offset() {
        // Every displacement with Delta r = r_h - r_obs lands on the plane n_a xi^a = Delta r, and
        // the drawn line is that plane's slice at xi^2 = 0.
        let metric = KerrSchild::new(1.0, 0.65);
        for &r in probe_radii(&metric).iter() {
            let frame = LocalFrame::for_observer(&metric, r, &raindrop(&metric, r));
            let n = [
                frame.tetrad().e0[1],
                frame.tetrad().e1[1],
                frame.tetrad().e2[1],
            ];
            for &r_h in &[0.0, metric.inner_horizon(), metric.outer_horizon(), 2.0] {
                let line = frame.surface_r_const(r_h);
                let d_r = r_h - r;
                let on_line = n[0] * line.point[1] + n[1] * line.point[0];
                assert!(
                    (on_line - d_r).abs() < 1e-9 * (1.0 + d_r.abs()),
                    "anchor off the plane: {on_line} vs {d_r} at r={r}, r_h={r_h}"
                );
                let along = n[0] * line.dir[1] + n[1] * line.dir[0];
                assert!(along.abs() < 1e-9, "direction leaves the plane: {along}");
                assert!((line.dir[0] * line.dir[0] + line.dir[1] * line.dir[1] - 1.0).abs() < 1e-12);
                // The whole surface really does map to that plane: sample displacements with the
                // right Delta r and arbitrary Delta t, Delta phi.
                for &(dt, dphi) in &[(0.0, 0.0), (0.3, -0.2), (-0.7, 0.4)] {
                    let xi = frame.to_local(&[dt, d_r, dphi]);
                    let s = n[0] * xi[0] + n[1] * xi[1] + n[2] * xi[2];
                    assert!(
                        (s - d_r).abs() < 1e-9 * (1.0 + d_r.abs()),
                        "n . xi = {s} vs Delta r = {d_r} at r={r}, r_h={r_h}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_surface_tilts_follow_the_regions() {
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();

        // Region I: r = const is timelike, steeper than 45 degrees.
        let far = LocalFrame::for_observer(&metric, 4.0, &raindrop(&metric, 4.0));
        let line = far.surface_r_const(rp);
        assert!(line.slope().abs() > 1.0, "|slope| = {} at r=4", line.slope().abs());
        assert_eq!(line.character(1e-9), SurfaceCharacter::Timelike);
        // ... and it lies inward of the observer, at negative xi^1.
        assert!(line.point[0] < 0.0, "r+ must be drawn inward: {:?}", line.point);

        // On the horizon itself: the surface is null, exactly 45 degrees, through the origin.
        let on_rp = LocalFrame::for_observer(&metric, rp, &raindrop(&metric, rp));
        let line = on_rp.surface_r_const(rp);
        assert!(
            (line.slope().abs() - 1.0).abs() < 1e-8,
            "|slope| = {} at r = r+",
            line.slope().abs()
        );
        assert_eq!(line.character(1e-9), SurfaceCharacter::Null);
        assert!(line.point[0].abs() < 1e-12 && line.point[1].abs() < 1e-12);

        // Region II: both horizons are spacelike, flatter than 45 degrees; r- is in the future and
        // r+ in the past, which is precisely what "trapped" means.
        let mid = 0.5 * (rp + rm);
        let inside = LocalFrame::for_observer(&metric, mid, &raindrop(&metric, mid));
        let to_rm = inside.surface_r_const(rm);
        let to_rp = inside.surface_r_const(rp);
        assert!(to_rm.slope().abs() < 1.0, "|slope| = {} for r- at r={mid}", to_rm.slope().abs());
        assert_eq!(to_rm.character(1e-9), SurfaceCharacter::Spacelike);
        assert!(to_rm.xi0_at_axis().unwrap() > 0.0, "r- must lie in the future");
        assert!(to_rp.slope().abs() < 1.0);
        assert!(to_rp.xi0_at_axis().unwrap() < 0.0, "r+ must lie in the past");

        // Region III: r = const is timelike again.
        let core = 0.5 * rm;
        let deep = LocalFrame::for_observer(&metric, core, &raindrop(&metric, core));
        let line = deep.surface_r_const(rm);
        assert!(line.slope().abs() > 1.0, "|slope| = {} at r={core}", line.slope().abs());
        assert_eq!(line.character(1e-9), SurfaceCharacter::Timelike);
    }

    #[test]
    fn test_slope_classification_matches_g_upper_rr() {
        // |slope| > 1 iff g^rr > 0, |slope| = 1 iff Delta = 0 - for every observer, in every
        // region, at every spin. This is the whole content of the drawing rule.
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let radii = [
                12.0,
                4.0,
                rp,
                0.75 * rp + 0.25 * rm,
                0.5 * (rp + rm),
                rm,
                0.5 * rm,
                (0.2 * rm).max(0.02),
            ];
            for &r in radii.iter() {
                let mut velocities = vec![raindrop(&metric, r)];
                if r > rp {
                    velocities.push(zamo(&metric, r));
                }
                for u in velocities {
                    let frame = LocalFrame::for_observer(&metric, r, &u);
                    // The line's tilt is a property of the observer's event, not of r_h.
                    for &r_h in &[0.0, rm, rp, 2.0] {
                        let slope = frame.surface_r_const(r_h).slope().abs();
                        let grr = metric.g_upper_rr(r);
                        if metric.delta(r).abs() < 1e-12 {
                            assert!(
                                (slope - 1.0).abs() < 1e-8,
                                "Delta = 0 needs 45 degrees: |slope| = {slope} at r={r} (a={a})"
                            );
                        } else if grr > 0.0 {
                            assert!(
                                slope > 1.0,
                                "g^rr = {grr} > 0 needs |slope| = {slope} > 1 at r={r} (a={a})"
                            );
                        } else {
                            assert!(
                                slope < 1.0,
                                "g^rr = {grr} < 0 needs |slope| = {slope} < 1 at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_static_observer_sees_vertical_constant_r_surfaces() {
        // A hovering observer's own r = const surface is his worldline: the line is vertical, and
        // every other one is vertical too, offset by the local proper distance.
        let metric = KerrSchild::new(1.0, 0.65);
        let r = 5.0;
        let u = static_obs(&metric, r);
        let frame = LocalFrame::for_observer(&metric, r, &u);
        let own = frame.surface_r_const(r);
        assert!(own.slope().is_infinite());
        assert_eq!(own.character(1e-9), SurfaceCharacter::Timelike);
        assert!(own.point[0].abs() < 1e-12 && own.xi0_at_axis().is_none());

        // Proper distance to the horizon: the anchor's xi^1 is Delta r / e1^r, and e1^r = sqrt(g^rr)
        // for an observer with u^r = 0.
        let horizon = frame.surface_r_const(metric.outer_horizon());
        let expected = (metric.outer_horizon() - r) / metric.g_upper_rr(r).sqrt();
        assert!(
            (horizon.point[0] - expected).abs() < 1e-9 * (1.0 + expected.abs()),
            "xi^1 = {} vs {expected}",
            horizon.point[0]
        );
    }

    #[test]
    fn test_other_observer_maps_into_the_chart() {
        // A second observer a little further out, at the same coordinate time: he is drawn ahead in
        // xi^1 and his worldline direction is timelike and future-directed in the chart.
        let metric = KerrSchild::new(1.0, 0.65);
        let (r_focus, r_other) = (4.0, 4.4);
        let frame = LocalFrame::for_observer(&metric, r_focus, &raindrop(&metric, r_focus));
        let xi = frame.to_local(&[0.0, r_other - r_focus, 0.1]);
        assert!(xi[1] > 0.0, "an observer further out is drawn outward: {xi:?}");

        let v = frame.vector_to_local(&raindrop(&metric, r_other));
        assert!(v[0] > 0.0, "the other worldline must run into the future: {v:?}");
        let speed = (v[1] * v[1] + v[2] * v[2]).sqrt() / v[0];
        assert!(speed < 1.0, "relative speed {speed} must be sub-luminal");
        // v^a is very nearly a unit timelike vector of the local Minkowski metric: exactly unit
        // for a vector at the observer's own event, and unit to first order in the offset for one
        // carried from another event, which is the accuracy this chart claims.
        let n = -v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        assert!((n + 1.0).abs() < 0.05, "eta(v, v) = {n}");
        let here = frame.vector_to_local(&raindrop(&metric, r_focus));
        let n_here = -here[0] * here[0] + here[1] * here[1] + here[2] * here[2];
        assert!((n_here + 1.0).abs() < 1e-9, "eta(v, v) = {n_here} at the observer's own event");
    }
}
