use crate::physics::kerr_schild::KerrSchild;

/// Bilinear form g_{mu nu} a^mu b^nu at radius r, for two contravariant vectors written in the
/// equatorial Kerr-Schild chart (t, r, phi). `KerrSchild::norm` is the diagonal case a = b.
pub fn inner(metric: &KerrSchild, r: f64, a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let g = metric.metric_components(r);
    let mut sum = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            sum += g[i][j] * a[i] * b[j];
        }
    }
    sum
}

/// Determinant of a 3 x 3 matrix given by rows.
fn det3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Local orthonormal tetrad e_{(a)}^mu carried by an observer with 4-velocity u at radius r on the
/// equatorial plane: the frame in which that observer is at rest and light moves isotropically at
/// c = 1. The three legs satisfy g(e_a, e_b) = diag(-1, +1, +1) exactly, so the cross terms
/// g_tr, g_tphi and g_rphi of the ingoing Kerr-Schild chart are fully accounted for.
#[derive(Debug, Clone, Copy)]
pub struct Tetrad {
    /// Timelike leg, equal to the observer's 4-velocity u^mu (future-directed, g(e0, e0) = -1).
    pub e0: [f64; 3],
    /// Spacelike radial leg: the observer's local outward direction, g(e1, d_r) > 0.
    pub e1: [f64; 3],
    /// Spacelike azimuthal leg: the observer's local +phi direction.
    pub e2: [f64; 3],
}

impl Tetrad {
    /// Gram-Schmidt orthonormalisation of the coordinate frame against a timelike 4-velocity u:
    ///
    ///     e0 = u                                              (normalised, g(u, u) = -1)
    ///     v1 = d_r   + g(d_r, e0) e0,                     e1 = v1 / sqrt(g(v1, v1))
    ///     v2 = d_phi + g(d_phi, e0) e0 - g(d_phi, e1) e1, e2 = v2 / sqrt(g(v2, v2))
    ///
    /// The plus sign in front of g(x, e0) e0 is the Lorentzian projector: the subtracted piece is
    /// g(x, e0) / g(e0, e0) times e0, and g(e0, e0) = -1 flips the sign. The orthogonal complement
    /// of a timelike vector is a positive-definite subspace, so g(v1, v1) > 0 and g(v2, v2) > 0 at
    /// *every* radius r > 0, including at and inside both horizons, where the surfaces r = const
    /// turn spacelike (g^rr < 0). No horizon-dependent special case is needed. (The coordinate
    /// direction d_r itself stays spacelike in this chart, g_rr = 1 + 2M/r; it is the covector dr
    /// whose character changes.) v1 can never vanish, since that would need d_r parallel to u.
    pub fn from_four_velocity(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> Self {
        let r = r.max(1e-4);
        debug_assert!(
            (inner(metric, r, u, u) + 1.0).abs() < 1e-6,
            "tetrad needs a unit timelike 4-velocity; got u.u = {} at r = {r}",
            inner(metric, r, u, u)
        );

        let e0 = *u;
        let d_r = [0.0, 1.0, 0.0];
        let d_phi = [0.0, 0.0, 1.0];

        let u_r = inner(metric, r, &d_r, &e0);
        let mut v1 = [0.0f64; 3];
        for mu in 0..3 {
            v1[mu] = d_r[mu] + u_r * e0[mu];
        }
        let n1 = inner(metric, r, &v1, &v1).max(1e-300).sqrt();
        let e1 = [v1[0] / n1, v1[1] / n1, v1[2] / n1];

        let u_phi = inner(metric, r, &d_phi, &e0);
        let p_phi = inner(metric, r, &d_phi, &e1);
        let mut v2 = [0.0f64; 3];
        for mu in 0..3 {
            v2[mu] = d_phi[mu] + u_phi * e0[mu] - p_phi * e1[mu];
        }
        let n2 = inner(metric, r, &v2, &v2).max(1e-300).sqrt();
        let e2 = [v2[0] / n2, v2[1] / n2, v2[2] / n2];

        Self { e0, e1, e2 }
    }

    /// Orthonormal tetrad adapted to a (time, radius) diagram: the same observer, but with the
    /// azimuthal leg chosen *tangent to the surfaces r = const*, i.e. e2^r = 0.
    ///
    /// The surfaces r = const are spanned by d_t and d_phi, so the vectors tangent to them with
    /// zero radial component form the 2-plane span(d_t, d_phi). Requiring in addition
    /// orthogonality to u picks the single direction
    ///
    ///     w^mu = (u_phi, 0, -u_t),      g(w, u) = u_phi u_t - u_t u_phi = 0,
    ///
    /// which is spacelike because it lies in the (positive-definite) rest space of u. Then
    /// e2 = w / sqrt(g(w, w)) and e1 is what is left over: the one unit direction orthogonal to
    /// both e0 and e2, with its sign fixed by the handedness of the frame (see the body).
    ///
    /// Why this gauge: the drawn plane of the diagram is span(e0, e1), the slice xi^2 = 0. The
    /// covector dr has local components n_a = e_a^r, and this gauge makes n_2 = 0, so
    ///
    ///     -n_0^2 + n_1^2 = eta^{ab} n_a n_b = g^{rr} = Delta / r^2
    ///
    /// exactly. The trace of a surface r = const in the drawn plane therefore carries the exact
    /// causal character of the surface itself: steeper than 45 degrees where g^rr > 0, at 45
    /// degrees on either horizon, flatter where g^rr < 0. With the `from_four_velocity` gauge the
    /// leftover n_2 = e2^r (which equals a / r for a raindrop) would tilt the drawn line away from
    /// 45 degrees at the horizons, because there the null generator of the surface leaves the
    /// slice.
    ///
    /// The degenerate case u_t = u_phi = 0 (possible only between the horizons, where the
    /// covector dr is timelike and u can be proportional to it) makes w vanish; the construction
    /// then falls back to the rest-space projection of d_phi, which is the continuous limit,
    /// since u_phi -> 0 there.
    pub fn from_four_velocity_axial(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> Self {
        let r = r.max(1e-4);
        debug_assert!(
            (inner(metric, r, u, u) + 1.0).abs() < 1e-6,
            "tetrad needs a unit timelike 4-velocity; got u.u = {} at r = {r}",
            inner(metric, r, u, u)
        );

        let e0 = *u;
        let g = metric.metric_components(r);
        // u_mu = g_{mu nu} u^nu.
        let mut u_low = [0.0f64; 3];
        for mu in 0..3 {
            for nu in 0..3 {
                u_low[mu] += g[mu][nu] * u[nu];
            }
        }

        let mut w = [u_low[2], 0.0, -u_low[0]];
        let mut w2 = inner(metric, r, &w, &w);
        let scale = 1.0 + u_low[0] * u_low[0] + u_low[2] * u_low[2];
        if !(w2 > 1e-12 * scale) {
            // u_t and u_phi both vanish: any direction in span(d_t, d_phi) is orthogonal to u, and
            // the rest-space projection of d_phi is the one that matches the limit.
            let d_phi = [0.0, 0.0, 1.0];
            let u_phi = inner(metric, r, &d_phi, &e0);
            w = [u_phi * e0[0], u_phi * e0[1], 1.0 + u_phi * e0[2]];
            w2 = inner(metric, r, &w, &w);
        }
        let n2 = w2.max(1e-300).sqrt();
        let e2 = [w[0] / n2, w[1] / n2, w[2] / n2];

        // e1: the one unit direction orthogonal to both e0 and e2, built as the Hodge dual of
        // e0 ^ e2 rather than as the rest-space projection of d_r. The projection does the same
        // job wherever it is non-zero, and its natural sign rule g(e1, d_r) > 0 is what "outward"
        // means far away. But d_r can fall into the plane span(e0, e2) along a perfectly good
        // worldline - it happens between the horizons, near r = 1.09, for a prograde infall with
        // E = 1, L = 2 at a = 0.90 - and there the projection passes through zero, so normalising
        // it reverses e1 end for end from one step to the next and every surface drawn in the
        // observer's frame mirrors left to right. The dual never vanishes. With
        //
        //     omega_alpha = sqrt|g| eps_{alpha beta gamma} e0^beta e2^gamma,
        //     e1^mu       = +/- g^{mu alpha} omega_alpha,
        //
        // omega annihilates e0 and e2 by antisymmetry, and the cofactor identity
        //     g^{mu alpha} eps_{mu beta gamma} eps_{alpha delta epsilon}
        //         = (g_{beta delta} g_{gamma epsilon} - g_{beta epsilon} g_{gamma delta}) / det g
        // gives g(e1, e1) = -[g(e0, e0) g(e2, e2) - g(e0, e2)^2] = 1 with no normalisation at all.
        // The sign is the handedness of the frame: (e0, e1, e2) is given the orientation of
        // (d_t, d_r, d_phi). That agrees with g(e1, d_r) > 0 for a static observer at large r, and
        // it is continuous along every worldline because the determinant of an orthonormal frame
        // in a fixed coordinate basis never vanishes. So e1 points outward wherever outward has a
        // meaning, and keeps pointing the same way through the places where it does not.
        let ginv = metric.inverse_metric(r);
        let det_g = det3(&g);
        debug_assert!(
            det_g < 0.0,
            "a Lorentzian 3-metric has det g < 0; got {det_g} at r = {r}"
        );
        let sqrt_abs_g = (-det_g).sqrt();
        let omega = [
            sqrt_abs_g * (e0[1] * e2[2] - e0[2] * e2[1]),
            sqrt_abs_g * (e0[2] * e2[0] - e0[0] * e2[2]),
            sqrt_abs_g * (e0[0] * e2[1] - e0[1] * e2[0]),
        ];
        let mut e1 = [0.0f64; 3];
        for mu in 0..3 {
            for alpha in 0..3 {
                e1[mu] += ginv[mu][alpha] * omega[alpha];
            }
        }
        if det3(&[e0, e1, e2]) < 0.0 {
            e1 = [-e1[0], -e1[1], -e1[2]];
        }

        Self { e0, e1, e2 }
    }

    /// Future-directed null vector k^mu = e0 + cos(alpha) e1 + sin(alpha) e2 emitted by this
    /// observer at local angle alpha: alpha = 0 is the local outward radial direction, alpha = pi
    /// the local inward one, alpha = pi/2 the local +phi direction. It is null by construction,
    /// because the frame is orthonormal: g(k, k) = -1 + cos^2 + sin^2 = 0.
    pub fn null_direction(&self, alpha: f64) -> [f64; 3] {
        let (s, c) = alpha.sin_cos();
        [
            self.e0[0] + c * self.e1[0] + s * self.e2[0],
            self.e0[1] + c * self.e1[1] + s * self.e2[1],
            self.e0[2] + c * self.e1[2] + s * self.e2[2],
        ]
    }

    /// Coordinate slopes (dr/dt, dphi/dt) = (k^r / k^t, k^phi / k^t) of a vector in this frame.
    /// For every `null_direction` and every r > 0 the component k^t is positive: the ingoing
    /// Kerr-Schild time function increases along all future-directed null rays.
    pub fn coordinate_velocity(&self, k: &[f64; 3]) -> (f64, f64) {
        (k[1] / k[0], k[2] / k[0])
    }

    /// 4-velocity of an observer moving with local velocity beta = (beta_r, beta_phi) relative to
    /// this frame: u' = gamma (e0 + beta_r e1 + beta_phi e2), gamma = 1 / sqrt(1 - |beta|^2).
    /// The speed |beta| is clamped to 0.99 so the result stays timelike and finite.
    pub fn boost(&self, beta_r: f64, beta_phi: f64) -> [f64; 3] {
        let speed = (beta_r * beta_r + beta_phi * beta_phi).sqrt();
        let (b_r, b_phi) = if speed > 0.99 {
            (beta_r * 0.99 / speed, beta_phi * 0.99 / speed)
        } else {
            (beta_r, beta_phi)
        };
        let gamma = 1.0 / (1.0 - b_r * b_r - b_phi * b_phi).max(1e-12).sqrt();
        [
            gamma * (self.e0[0] + b_r * self.e1[0] + b_phi * self.e2[0]),
            gamma * (self.e0[1] + b_r * self.e1[1] + b_phi * self.e2[1]),
            gamma * (self.e0[2] + b_r * self.e1[2] + b_phi * self.e2[2]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::geodesic::GeodesicState;

    /// Raindrop (E = 1, L = 0) 4-velocity. It exists at every r > 0, so it is the one reference
    /// frame available in all three regions.
    fn raindrop(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let (ut, ur, up) = GeodesicState::new_infall(metric, 0.0, r, 1.0, 0.0).derivatives(metric, r);
        [ut, ur, up]
    }

    /// ZAMO 4-velocity; timelike only outside the outer horizon.
    fn zamo(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g = metric.metric_components(r);
        let omega = metric.frame_dragging_omega(r);
        let n = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma = 1.0 / n.sqrt();
        [gamma, 0.0, gamma * omega]
    }

    /// Radii spanning every region: exterior, r+, between the horizons, r-, inside r-.
    fn probe_radii(metric: &KerrSchild) -> Vec<f64> {
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon().max(0.05);
        vec![8.0, 3.0, rp, 0.5 * (rp + rm), rm, (0.5 * rm).max(0.05)]
    }

    #[test]
    fn test_tetrad_is_orthonormal_in_every_region() {
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in probe_radii(&metric).iter() {
                let t = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
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
                            "g(e{i}, e{j}) = {got} (want {expected}) at r={r} (a={a})"
                        );
                    }
                }
                // e1 really points outward: g(e1, d_r) > 0.
                assert!(inner(&metric, r, &t.e1, &[0.0, 1.0, 0.0]) > 0.0);
            }
        }
    }

    #[test]
    fn test_null_directions_are_null_and_future_directed() {
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in probe_radii(&metric).iter() {
                let t = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
                for i in 0..32 {
                    let alpha = 2.0 * std::f64::consts::PI * (i as f64) / 32.0;
                    let k = t.null_direction(alpha);
                    let n = metric.norm(r, &k);
                    let scale = 1.0 + k.iter().fold(0.0f64, |m, v| m.max(v.abs())).powi(2);
                    assert!(
                        n.abs() < 1e-9 * scale,
                        "g(k, k) = {n} at alpha={alpha}, r={r} (a={a})"
                    );
                    assert!(
                        k[0] > 0.0,
                        "k^t = {} must be positive at alpha={alpha}, r={r} (a={a})",
                        k[0]
                    );
                }
            }
        }
    }

    #[test]
    fn test_wedge_is_the_frame_independent_extreme_of_dr_dt() {
        // The projection of the null cone onto the (t, r) plane is a property of the event, not of
        // the observer: sweeping alpha in a free-fall frame and in a ZAMO frame at the same r must
        // give the same extreme coordinate slopes, and those are `KerrSchild::null_wedge`.
        let samples = 20_000;
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            for &r in probe_radii(&metric).iter() {
                let mut frames = vec![Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r))];
                if r > rp {
                    frames.push(Tetrad::from_four_velocity(&metric, r, &zamo(&metric, r)));
                }
                let wedge = metric.null_wedge(r);
                for t in frames {
                    let mut lo = f64::INFINITY;
                    let mut hi = f64::NEG_INFINITY;
                    for i in 0..samples {
                        let alpha = 2.0 * std::f64::consts::PI * (i as f64) / (samples as f64);
                        let (dr_dt, _) = t.coordinate_velocity(&t.null_direction(alpha));
                        lo = lo.min(dr_dt);
                        hi = hi.max(dr_dt);
                    }
                    assert!(
                        (lo - wedge.dr_dt_in).abs() < 1e-6,
                        "min dr/dt = {lo} vs wedge {} at r={r} (a={a})",
                        wedge.dr_dt_in
                    );
                    assert!(
                        (hi - wedge.dr_dt_out).abs() < 1e-6,
                        "max dr/dt = {hi} vs wedge {} at r={r} (a={a})",
                        wedge.dr_dt_out
                    );
                }
            }
        }
    }

    #[test]
    fn test_boost_is_a_unit_timelike_vector() {
        let metric = KerrSchild::new(1.0, 0.65);
        for &r in probe_radii(&metric).iter() {
            let t = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
            for &(b_r, b_phi) in &[(0.0, 0.0), (0.5, -0.3), (-0.7, 0.2), (0.9, 0.9)] {
                let u = t.boost(b_r, b_phi);
                let n = metric.norm(r, &u);
                assert!((n + 1.0).abs() < 1e-9, "u.u = {n} for beta=({b_r},{b_phi}) at r={r}");
                assert!(u[0] > 0.0, "a boosted observer must still move forward in t: {u:?}");
            }
            // Zero boost is the identity.
            let u0 = t.boost(0.0, 0.0);
            for mu in 0..3 {
                assert!((u0[mu] - t.e0[mu]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_axial_tetrad_is_orthonormal_and_tangent_to_constant_r() {
        // The diagram gauge must still be an exact orthonormal frame with e0 = u, and its
        // azimuthal leg must have no radial component, so that -n_0^2 + n_1^2 = g^rr exactly.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in probe_radii(&metric).iter() {
                let mut frames = vec![Tetrad::from_four_velocity_axial(&metric, r, &raindrop(&metric, r))];
                if r > metric.outer_horizon() {
                    frames.push(Tetrad::from_four_velocity_axial(&metric, r, &zamo(&metric, r)));
                }
                for t in frames {
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
                                "g(e{i}, e{j}) = {got} (want {expected}) at r={r} (a={a})"
                            );
                        }
                    }
                    assert!(t.e2[1].abs() < 1e-12, "e2^r = {} must vanish at r={r} (a={a})", t.e2[1]);
                    assert!(inner(&metric, r, &t.e1, &[0.0, 1.0, 0.0]) > 0.0, "e1 must point outward");
                    // n_a = e_a^r is the covector dr in this frame; its Minkowski norm is g^rr.
                    let n = [t.e0[1], t.e1[1], t.e2[1]];
                    let norm = -n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
                    let grr = metric.g_upper_rr(r);
                    assert!(
                        (norm - grr).abs() < 1e-8 * (1.0 + grr.abs()),
                        "|dr|^2 = {norm} vs g^rr = {grr} at r={r} (a={a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_axial_e1_does_not_reverse_where_d_r_falls_into_the_plane_of_e0_and_e2() {
        // The worldline that showed the flip: E = 1, L = 2 at a = 0.90, released from r = 4.5.
        // E - Omega_- L < 0, so it heads for the far branch of r_- and never crosses the drawn one;
        // on the way, between the horizons near r = 1.09, the coordinate direction d_r passes
        // through span(e0, e2). With e1 taken as the normalised rest-space projection of d_r, its
        // sign reference vanished there and e1 reversed between two consecutive steps, mirroring
        // every surface in the observer's frame view. Along the whole walk from release to just
        // above r_- the frame must stay orthonormal and consecutive e1's must nearly coincide.
        // The walk is paced by the observer's own clock (capped in coordinate time, see below),
        // because near r_- u^t grows without bound and a fixed coordinate step there spans a
        // large turn of the frame; a reversal is a jump to overlap -1 at any step size, so the
        // check discriminates.
        // The walk also has to actually go through the degenerate configuration for the check to
        // mean anything, and the witness for that is the old sign reference g(e1, d_r) changing
        // sign along the way - under the old rule g(e1, d_r) = |v1| was positive by construction,
        // so a sign change proves the old e1 and the new one differ by a sign somewhere, i.e. that
        // the old e1 reversed on this worldline.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let (r_plus, r_minus) = (metric.outer_horizon(), metric.inner_horizon());
        let mut bob = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.0, false),
        );
        // Components of a vector in the raindrop tetrad at r. Two of Bob's frames at neighbouring
        // events are compared through these rather than by dotting coordinate components taken
        // at two different radii: for a frame boosted by u^t ~ 30 that mismatch is an error of
        // order (delta g)(u^t)^2 and swamps the signal, while the raindrop frame is smooth in r
        // and the Minkowski product of local components is cosh(delta rapidity) cos(delta angle),
        // near 1 for a small step and near -1 for a reversal.
        let local = |r: f64, v: &[f64; 3]| -> [f64; 3] {
            let rain = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
            [
                -inner(&metric, r, &rain.e0, v),
                inner(&metric, r, &rain.e1, v),
                inner(&metric, r, &rain.e2, v),
            ]
        };
        // Each step is the smaller of one tick of Bob's own clock and 0.05 M of coordinate time:
        // his clock, because in coordinate time the frame turns slowly early on and fast near r_-
        // where u^t runs away; the cap, because near r_- one tick of his clock is many M.
        let d_tau = 0.02;
        let mut t = 0.0;
        // One step first, so every frame below is the free-faller's own and not the static
        // observer's that `four_velocity` reports before release.
        t += d_tau;
        bob.step(&metric, t, d_tau);
        let d_r = [0.0, 1.0, 0.0];
        let mut prev: Option<[f64; 3]> = None;
        let mut proj_min = f64::INFINITY;
        let (mut proj_pos, mut proj_neg) = (false, false);
        let mut r_min = f64::INFINITY;
        let mut overlap_min = f64::INFINITY;
        let mut steps = 0;
        while bob.r > r_minus + 1e-3 && t < 60.0 && steps < 10_000 {
            let r = bob.r;
            let u = bob.four_velocity(&metric);
            let f = Tetrad::from_four_velocity_axial(&metric, r, &u);
            let legs = [f.e0, f.e1, f.e2];
            // Rounding in the products of components of size u^t, which runs into the thousands
            // by the end of the walk.
            let tol = 1e-11 * (1.0 + u[0] * u[0]);
            for i in 0..3 {
                for j in 0..3 {
                    let expected = if i != j { 0.0 } else if i == 0 { -1.0 } else { 1.0 };
                    let got = inner(&metric, r, &legs[i], &legs[j]);
                    assert!(
                        (got - expected).abs() < tol,
                        "g(e{i}, e{j}) = {got} (want {expected}) at t={t}, r={r}"
                    );
                }
            }
            assert!(det3(&[f.e0, f.e1, f.e2]) > 0.0, "right-handed frame at t={t}, r={r}");
            let proj = inner(&metric, r, &f.e1, &d_r);
            proj_min = proj_min.min(proj.abs());
            if proj > 0.0 {
                proj_pos = true;
            } else {
                proj_neg = true;
            }
            let xi = local(r, &f.e1);
            if let Some(p) = prev {
                let overlap = -p[0] * xi[0] + p[1] * xi[1] + p[2] * xi[2];
                assert!(
                    overlap > 0.9,
                    "e1 reversed between steps at t={t}, r={r}: overlap {overlap}"
                );
                overlap_min = overlap_min.min(overlap);
            }
            prev = Some(xi);
            r_min = r_min.min(r);
            // u^t = dt / d tau, so one tick of the observer's clock is this much coordinate time.
            let dt = (d_tau * u[0]).min(0.05);
            t += dt;
            bob.step(&metric, t, dt);
            steps += 1;
        }
        println!(
            "walked to r = {r_min:.4} (r+ = {r_plus:.4}, r- = {r_minus:.4}) by t = {t:.2} in \
             {steps} steps; smallest |g(e1, d_r)| seen = {proj_min:.3e}; smallest step overlap \
             of e1 in the raindrop frame = {overlap_min:.4}"
        );
        assert!(r_min < r_plus, "the walk crossed r+ (reached r = {r_min})");
        assert!(
            proj_pos && proj_neg,
            "g(e1, d_r) must change sign along this worldline, or the flip was never in reach"
        );
    }
}
