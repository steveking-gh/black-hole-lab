use crate::kerr_schild::KerrSchild;

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

/// The n generators of the null cone at the equatorial event (r, phi), as Cartesian velocities
/// (dx/dt, dy/dt) of the Kerr-Schild embedding x + i y = (r + i a) e^{i phi}.
///
/// Each sample is one future-directed null direction of `Tetrad::null_direction`, turned into the
/// coordinate slopes (dr/dt, dphi/dt) that light actually leaves the event on and then pushed
/// through `KerrSchild::cartesian_velocity`. Appending dz/dt = 1 to each makes it a generator of
/// the 3-dimensional cone in (x, y, t), so the returned polygon is the section that cone cuts out
/// of the slice t = 1: in flat space, far from the hole, it is the unit circle.
///
/// The *set* is a property of the event alone. Any observer at that event sees the same cone -
/// the null directions are the light through the event, not a choice of frame - so a boosted
/// tetrad returns the same closed curve; what the tetrad decides is only where along the rim the
/// n samples fall, since aberration bunches them towards the boost. That is why the caller picks
/// the frame: at u^t ~ 1e10, on the approach to the far branch of r-, the observer's own frame
/// crowds every sample into one point of the rim and leaves the rest of the curve undrawn.
// The volume view now routes its generators through `Chart::direction`, which is this same
// composition applied one null vector at a time, so nothing outside the tests calls this; it is
// kept because it is the named construction the cone drawing is written against.
#[allow(dead_code)]
pub fn light_cone_generators(
    metric: &KerrSchild,
    r: f64,
    phi: f64,
    tetrad: &Tetrad,
    n: usize,
) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let alpha = std::f64::consts::TAU * (i as f64) / (n as f64);
            let (dr_dt, dphi_dt) = tetrad.coordinate_velocity(&tetrad.null_direction(alpha));
            metric.cartesian_velocity(r, phi, dr_dt, dphi_dt)
        })
        .collect()
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
    ///
    /// Both steps are taken in the closed forms worked out below rather than by evaluating the
    /// formulae above term by term. They are the same vectors to a rounding for an observer whose
    /// u^mu is of order 1, and they are the difference between a frame and a NaN for one whose
    /// u^t has run away: a worldline settling onto the far branch of r- is carried out to
    /// `geodesic::U_T_STALL` = 1e10, and written literally this construction loses every digit of
    /// g(v1, v1) by u^t ~ 1e8 (v1's components are of size u_r u^t while g(v1, v1) is only u_r^2,
    /// a cancellation of sixteen orders) and every digit of the *direction* of v2 at the same
    /// place (its two large terms are each of size u_phi u^t and cancel down to order 1). The
    /// closed forms have no such cancellation in them, so the frame stays good wherever the
    /// worldline does.
    pub fn from_four_velocity(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> Self {
        let r = r.max(1e-4);
        // g(u, u) is a sum of terms of size |u|^2, so double precision alone can only deliver it
        // to ~1e-16 |u|^2 however exact the 4-velocity is. The check has to carry that factor or
        // it fires on a perfectly good worldline whose u^t has run away, which is what happens on
        // the approach to the far branch of r-: `geodesic::U_T_STALL` follows u^t out to 1e10
        // there, where (u^t)^2 is 1e20 and the rounding floor of this very expression is 1e4. The
        // 1e-6 is what it is checking everywhere else, where |u| is of order 1.
        debug_assert!(
            (inner(metric, r, u, u) + 1.0).abs()
                < 1e-6 + 1e-14 * u.iter().fold(1.0f64, |m, v| m.max(v.abs())).powi(2),
            "tetrad needs a unit timelike 4-velocity; got u.u = {} at r = {r}",
            inner(metric, r, u, u)
        );

        let e0 = *u;
        let g = metric.metric_components(r);
        let d_r = [0.0, 1.0, 0.0];
        let d_phi = [0.0, 0.0, 1.0];
        // The two covariant components of u that the whole construction is built from.
        let u_r = inner(metric, r, &d_r, &e0);
        let u_phi = inner(metric, r, &d_phi, &e0);

        let mut v1 = [0.0f64; 3];
        for mu in 0..3 {
            v1[mu] = d_r[mu] + u_r * e0[mu];
        }
        // g(v1, v1) = g_rr + 2 u_r g(d_r, e0) + u_r^2 g(e0, e0) = g_rr + u_r^2, since
        // g(d_r, e0) = u_r and g(e0, e0) = -1. A sum of two positive terms: exact to a rounding,
        // positive at any boost, and free of the cancellation that evaluating g on v1 itself runs
        // into once |v1| ~ u_r u^t dwarfs the answer.
        let n1_sq = g[1][1] + u_r * u_r;
        let n1 = n1_sq.max(1e-300).sqrt();
        let e1 = [v1[0] / n1, v1[1] / n1, v1[2] / n1];

        // Substituting e1 = (d_r + u_r e0) / n1 and g(d_phi, e1) = (g_rphi + u_r u_phi) / n1 into
        // v2 = d_phi + u_phi e0 - g(d_phi, e1) e1 and collecting the coordinate directions,
        //     v2 = d_phi - [(g_rphi + u_r u_phi) / n1^2] d_r
        //               + [(u_phi g_rr - u_r g_rphi) / n1^2] e0.
        // The coefficient of e0 falls off like 1/u^t while e0 grows like u^t, which is why the
        // literal form has two terms of size u_phi u^t cancelling down to a v2 of order 1: here
        // that cancellation has been done once, algebraically, and what is left has none.
        let a_r = -(g[1][2] + u_r * u_phi) / n1_sq;
        let a_0 = (u_phi * g[1][1] - u_r * g[1][2]) / n1_sq;
        let mut v2 = [0.0f64; 3];
        for mu in 0..3 {
            v2[mu] = d_phi[mu] + a_r * d_r[mu] + a_0 * e0[mu];
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
        // g(u, u) is a sum of terms of size |u|^2, so double precision alone can only deliver it
        // to ~1e-16 |u|^2 however exact the 4-velocity is. The check has to carry that factor or
        // it fires on a perfectly good worldline whose u^t has run away, which is what happens on
        // the approach to the far branch of r-: `geodesic::U_T_STALL` follows u^t out to 1e10
        // there, where (u^t)^2 is 1e20 and the rounding floor of this very expression is 1e4. The
        // 1e-6 is what it is checking everywhere else, where |u| is of order 1.
        debug_assert!(
            (inner(metric, r, u, u) + 1.0).abs()
                < 1e-6 + 1e-14 * u.iter().fold(1.0f64, |m, v| m.max(v.abs())).powi(2),
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
        // The negation is the point rather than a way of writing <=: a w2 that has come out NaN
        // has to take this branch too, and `w2 <= 1e-12 * scale` would let it through.
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
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
        //
        // The minus sign below is that handedness, and it is a constant rather than something to
        // be decided event by event. Contracting the dual with e1 gives
        //     omega_alpha e1^alpha = g^{alpha beta} omega_alpha omega_beta = g(e1, e1) = 1,
        // and the same contraction written out through eps is -sqrt|g| det[e0, e1, e2], so
        //     det[e0, e1, e2] = -1 / sqrt|g|
        // for *every* timelike u at every radius: the dual as ordered here is always left-handed
        // and always has to be reversed. That used to be settled by computing det[e0, e1, e2] and
        // flipping when it came out negative, which is the same answer wherever the determinant
        // can be computed - but it is a cancellation of products of components of size |u|^2 down
        // to a number of order 1, so its rounding floor is ~1e-16 |u|^2, and past u^t ~ 1e8 the
        // sign it returns is noise. With `geodesic::U_T_STALL` carrying a worldline onto the far
        // branch of r- out to u^t = 1e10, that noise reversed e1 end for end between one step and
        // the next and mirrored every surface in the observer's frame view - precisely the failure
        // the dual was introduced to remove, arriving by another route. Taking the sign from the
        // identity instead costs nothing and cannot be reversed by rounding.
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
                e1[mu] -= ginv[mu][alpha] * omega[alpha];
            }
        }
        // The identity, checked wherever double precision can still see it. The determinant's own
        // rounding floor is ~1e-16 |u|^2, so for a frame boosted past u^t ~ 1e8 there is nothing
        // left to check and the assertion stands aside rather than firing on noise.
        debug_assert!(
            {
                let scale = e0.iter().chain(e1.iter()).fold(1.0f64, |m, v| m.max(v.abs()));
                let noise = 16.0 * f64::EPSILON * scale * scale * sqrt_abs_g;
                noise > 0.5 || (det3(&[e0, e1, e2]) * sqrt_abs_g - 1.0).abs() < 0.5
            },
            "the dual must be right-handed after the sign: det = {} at r = {r}, u = {u:?}",
            det3(&[e0, e1, e2]) * sqrt_abs_g
        );

        Self { e0, e1, e2 }
    }

    /// The same observer's frame with the two spatial legs turned in their own plane, so that e1
    /// points along the spatial direction of `s`.
    ///
    /// The observer, the time leg and the rest space are all untouched: this is a rotation of an
    /// orthonormal pair by the angle of s in that pair, so g(e_a, e_b) = diag(-1, 1, 1) survives
    /// exactly rather than to a normalisation. What changes is which spatial direction the drawn
    /// plane span(e0, e1) contains - and that is the whole of what the rest-frame view needs when
    /// the plane it draws is the one containing the line of sight rather than the radial one.
    ///
    /// `s` is read for its spatial part alone. The components taken are c1 = g(s, e1) and
    /// c2 = g(s, e2), so any component of s along e0 drops out and any length of s drops out with
    /// the normalisation; a caller handing over an exactly unit vector orthogonal to u gets back
    /// the frame whose e1 *is* that vector. A vector with no spatial part at all leaves the frame
    /// as it was, which is the only answer a direction with no direction in it admits.
    pub fn turned_towards(&self, metric: &KerrSchild, r: f64, s: &[f64; 3]) -> Self {
        let c1 = inner(metric, r, s, &self.e1);
        let c2 = inner(metric, r, s, &self.e2);
        let length = c1.hypot(c2);
        // The negation is the point rather than a way of writing <=: a length that has come out
        // NaN has to take this branch too, and `length <= 0.0` would let it through.
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        if !(length > 0.0) || !length.is_finite() {
            return *self;
        }
        let (cos, sin) = (c1 / length, c2 / length);
        Self {
            e0: self.e0,
            e1: core::array::from_fn(|mu| cos * self.e1[mu] + sin * self.e2[mu]),
            e2: core::array::from_fn(|mu| -sin * self.e1[mu] + cos * self.e2[mu]),
        }
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
    use crate::geodesic::GeodesicState;

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
        use crate::geodesic::R_STOP;
        let metric = KerrSchild::new(1.0, 0.90);
        let (r_plus, r_minus) = (metric.outer_horizon(), metric.inner_horizon());
        let mut bob = GeodesicState::new_with_direction(&metric, 0.0, 4.5, 1.0, 2.0, false);
        // The app's free-falling observer steps its geodesic under exactly this gate: settled on
        // the far branch of r- the state is held, and so is one that has reached the ring.
        let step = |bob: &mut GeodesicState, dt: f64| {
            if !bob.stalled && bob.r > R_STOP {
                bob.step_coord_time(&metric, dt);
            }
        };
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
        // One step first, so every frame below is one the integrator produced and not the
        // closed-form 4-velocity the state was seeded with.
        t += d_tau;
        step(&mut bob, d_tau);
        let d_r = [0.0, 1.0, 0.0];
        let mut prev: Option<[f64; 3]> = None;
        let mut proj_min = f64::INFINITY;
        let (mut proj_pos, mut proj_neg) = (false, false);
        let mut r_min = f64::INFINITY;
        let mut overlap_min = f64::INFINITY;
        let mut steps = 0;
        while bob.r > r_minus + 1e-3 && t < 60.0 && steps < 10_000 {
            let r = bob.r;
            let u = bob.u;
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
            step(&mut bob, dt);
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

    #[test]
    fn test_both_tetrads_survive_the_whole_approach_to_the_far_branch_of_r_minus() {
        // The observer whose u^t runs away. E = 1, L = 2.2 at a = 0.90 has E - Omega_- L < 0, so
        // he never crosses r-: he asymptotes to it with u^t growing like exp(kappa_- t) until the
        // integrator sets him down at `geodesic::U_T_STALL` = 1e10. Every frame he is drawn in -
        // the telemetry, the rest-frame view, the direction his light goes out in - is one of
        // these two tetrads, so both have to survive the whole approach, and this is the test that
        // walks them there. Run in a debug build it is also what exercises the `debug_assert!`s
        // inside both constructors at a u^t they would once have fired on.
        //
        // Two things are checked, and neither of them used to hold. g(e_a, e_b) must stay eta to
        // the only accuracy double precision can offer: the legs have components of size |u|, so
        // the products have a rounding floor of ~1e-16 |u|^2, and the bound carries that factor
        // rather than pretending to an absolute one. And e1 must not reverse - a reversal mirrors
        // every surface drawn in the observer's frame from one step to the next - for which its
        // own t-component is witness enough here, being of size u^t and nowhere near zero.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut geo = GeodesicState::new_infall(&metric, 0.0, 4.5, 1.0, 2.2);
        let mut previous_sign = [0.0f64; 2];
        let mut steps = 0;
        while !geo.stalled && geo.t < 200.0 {
            geo.step_coord_time(&metric, 0.1);
            steps += 1;
            let u = geo.u;
            let tol = 1e-11 * (1.0 + u[0] * u[0]);
            let frames = [
                ("Gram-Schmidt", Tetrad::from_four_velocity(&metric, geo.r, &u)),
                ("axial", Tetrad::from_four_velocity_axial(&metric, geo.r, &u)),
            ];
            for (which, (name, frame)) in frames.iter().enumerate() {
                let legs = [frame.e0, frame.e1, frame.e2];
                for i in 0..3 {
                    for j in 0..3 {
                        let expected = if i != j { 0.0 } else if i == 0 { -1.0 } else { 1.0 };
                        let got = inner(&metric, geo.r, &legs[i], &legs[j]);
                        assert!(
                            (got - expected).abs() < tol,
                            "{name}: g(e{i}, e{j}) = {got} (want {expected}) at u^t = {}, r = {}",
                            u[0],
                            geo.r
                        );
                    }
                }
                let sign = frame.e1[0].signum();
                if frame.e1[0].abs() > 1.0 {
                    assert!(
                        previous_sign[which] == 0.0 || sign == previous_sign[which],
                        "{name}: e1 reversed at u^t = {}, r = {}: e1 = {:?}",
                        u[0],
                        geo.r,
                        frame.e1
                    );
                    previous_sign[which] = sign;
                }
            }
        }
        println!(
            "both tetrads built at every one of {steps} steps, down to r - r- = {:.3e} with u^t = {:.3e}, at t = {:.2}",
            geo.r - metric.inner_horizon(),
            geo.u[0],
            geo.t
        );
        assert!(geo.stalled, "the walk must reach the stall, got u^t = {}", geo.u[0]);
        assert!(geo.u[0] > 1e9, "and it must be the runaway that stopped it: {}", geo.u[0]);
    }

    /// The chart slopes (dr/dt, dphi/dt) a Cartesian generator came from: the exact inverse of
    /// `KerrSchild::cartesian_velocity`, which rotates by the chart angle phi and mixes the spin
    /// into the radial part, (dx + i dy)/dt = (dr/dt - a dphi/dt + i r dphi/dt) e^{i phi}. Going
    /// back this way means the tests below read the numbers `light_cone_generators` actually
    /// returned rather than recomputing the slopes it was built from.
    fn chart_slopes(metric: &KerrSchild, r: f64, phi: f64, v: (f64, f64)) -> (f64, f64) {
        let (s, c) = phi.sin_cos();
        let radial = v.0 * c + v.1 * s;
        let tangential = -v.0 * s + v.1 * c;
        let dphi_dt = tangential / r;
        (radial + metric.a * dphi_dt, dphi_dt)
    }

    /// Distance from a point to a closed polyline, measured to the nearest point of the nearest
    /// segment rather than to the nearest vertex: two samplings of the same curve put their
    /// vertices in different places, so a vertex-to-vertex distance would be measuring the
    /// sampling and not the curve.
    fn distance_to_polyline(p: (f64, f64), poly: &[(f64, f64)]) -> f64 {
        let mut best = f64::INFINITY;
        for i in 0..poly.len() {
            let q = poly[i];
            let s = poly[(i + 1) % poly.len()];
            let (dx, dy) = (s.0 - q.0, s.1 - q.1);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((p.0 - q.0) * dx + (p.1 - q.1) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (cx, cy) = (q.0 + t * dx, q.1 + t * dy);
            best = best.min(((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt());
        }
        best
    }

    #[test]
    fn test_the_light_cone_generators_are_null_at_every_radius() {
        // Every point of the drawn cone must be light. The generators come back as Cartesian
        // velocities of the embedding, so the check inverts the embedding and asks the metric:
        // the tangent (1, dr/dt, dphi/dt) has to be null at the radius it was taken at, in all
        // three regions and for a hole with and without spin. The tolerance carries |v|^2 because
        // g(v, v) is a sum of terms of that size and inside r- the azimuthal slope is large.
        for &a in &[0.9, 0.0] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let phi = 0.7;
            for &r in &[9.0, 3.0, rp, 0.5 * (rp + rm), rm, 0.3] {
                let frame = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
                let n = 64;
                let generators = light_cone_generators(&metric, r, phi, &frame, n);
                assert_eq!(generators.len(), n, "one generator per sample");
                for (i, &g) in generators.iter().enumerate() {
                    let (dr_dt, dphi_dt) = chart_slopes(&metric, r, phi, g);
                    let v = [1.0, dr_dt, dphi_dt];
                    let scale = 1.0 + v.iter().fold(0.0f64, |m, c| m.max(c.abs())).powi(2);
                    let null = metric.norm(r, &v);
                    assert!(
                        null.abs() < 1e-9 * scale,
                        "g(v, v) = {null} for generator {i} = {g:?} at r={r} (a={a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_cone_section_is_the_same_in_every_frame() {
        // The null cone is the light through an event, so the curve it cuts out of dt = 1 is a
        // property of the event and of nothing else. Two observers passing through the same event
        // at 0.67 c relative to one another must therefore hand back the same closed curve; what
        // the boost changes is only where along the rim the samples land, which is aberration and
        // is why the comparison is curve-to-curve rather than sample-to-sample.
        //
        // The second radius is inside r-, where the wedge has reopened outward, so the claim is
        // being made in a region as well as outside one.
        let metric = KerrSchild::new(1.0, 0.90);
        let phi = 0.4;
        let n = 720;
        for &r in &[3.0, 0.8] {
            let rain = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
            let boosted = Tetrad::from_four_velocity(&metric, r, &rain.boost(0.6, -0.3));
            let from_rain = light_cone_generators(&metric, r, phi, &rain, n);
            let from_boost = light_cone_generators(&metric, r, phi, &boosted, n);
            let mut worst = 0.0f64;
            for &p in from_rain.iter() {
                worst = worst.max(distance_to_polyline(p, &from_boost));
            }
            for &p in from_boost.iter() {
                worst = worst.max(distance_to_polyline(p, &from_rain));
            }
            assert!(
                worst < 1e-3,
                "the two frames' cone sections part by {worst} at r={r}"
            );

            // And the curve is the one the (t, r) diagram draws its wedge from: the extreme
            // radial slopes over the samples are `KerrSchild::null_wedge`, which is derived
            // independently, by extremising dr/dt over the null condition.
            let wedge = metric.null_wedge(r);
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for &p in from_rain.iter() {
                let (dr_dt, _) = chart_slopes(&metric, r, phi, p);
                lo = lo.min(dr_dt);
                hi = hi.max(dr_dt);
            }
            println!(
                "r = {r}: the two frames' sections agree to {worst:.2e}; dr/dt runs over \
                 [{lo:.6}, {hi:.6}] against the wedge [{:.6}, {:.6}]",
                wedge.dr_dt_in, wedge.dr_dt_out
            );
            assert!(
                (lo - wedge.dr_dt_in).abs() < 1e-4,
                "min dr/dt = {lo} vs wedge {} at r={r}",
                wedge.dr_dt_in
            );
            assert!(
                (hi - wedge.dr_dt_out).abs() < 1e-4,
                "max dr/dt = {hi} vs wedge {} at r={r}",
                wedge.dr_dt_out
            );
        }
    }

    #[test]
    fn test_the_outermost_generator_stands_still_in_r_on_either_horizon() {
        // What makes a horizon a horizon, read off the drawn cone. On r+ and again on r- the
        // outward edge of the cone has dr/dt = 0 exactly - `null_wedge` puts the wedge edge at
        // D(0) = Delta, which vanishes on both - so not one generator gets out and the best of
        // them holds station. The cone is tipped, but only just: a sample either side of the
        // horizon would show it closing inward or reopening outward.
        let metric = KerrSchild::new(1.0, 0.90);
        let phi = 1.1;
        let n = 3600;
        for (name, r) in [("r+", metric.outer_horizon()), ("r-", metric.inner_horizon())] {
            let frame = Tetrad::from_four_velocity(&metric, r, &raindrop(&metric, r));
            let mut hi = f64::NEG_INFINITY;
            for &p in light_cone_generators(&metric, r, phi, &frame, n).iter() {
                let (dr_dt, _) = chart_slopes(&metric, r, phi, p);
                assert!(
                    dr_dt <= 1e-6,
                    "a generator escapes {name}: dr/dt = {dr_dt} at r={r}"
                );
                hi = hi.max(dr_dt);
            }
            println!("on {name} = {r:.6} the outermost of {n} generators has dr/dt = {hi:.3e}");
            assert!(
                hi > -1e-6,
                "and one of them must reach dr/dt = 0 on {name}: the best is {hi} at r={r}"
            );
        }
    }

    #[test]
    fn test_far_from_the_hole_the_cone_section_is_the_unit_circle() {
        // The chart is asymptotically Minkowski, so a long way out the cone has to be the flat
        // one: every generator moves at c through the drawn plane and the rim is centred on the
        // emitter. Both halves are checked, but on different frames, and that is the aberration
        // the doc comment warns about. The circle itself is the event's, so the raindrop sees the
        // same rim; but the raindrop at r = 1e4 is falling at beta = sqrt(2M/r) = 1.4e-2, and
        // uniform sampling in its frame crowds the rim forward by enough to shift the mean of the
        // samples by beta/2 - a hundredfold over the tolerance here, and a statement about where
        // the samples fall rather than about where the curve is. The ZAMO out there is the
        // coordinate frame to one part in 1e4, so its samples are the evenly spaced ones.
        let metric = KerrSchild::new(1.0, 0.90);
        let (r, phi) = (1e4, 2.0);
        let n = 256;
        for (name, u) in [("raindrop", raindrop(&metric, r)), ("ZAMO", zamo(&metric, r))] {
            let frame = Tetrad::from_four_velocity(&metric, r, &u);
            let generators = light_cone_generators(&metric, r, phi, &frame, n);
            let mut worst = 0.0f64;
            for &(dx, dy) in generators.iter() {
                worst = worst.max((dx.hypot(dy) - 1.0).abs());
            }
            assert!(
                worst < 1e-3,
                "{name}: a generator moves at {} c at r={r}",
                worst + 1.0
            );
            if name == "ZAMO" {
                let mean = generators.iter().fold((0.0, 0.0), |acc, g| {
                    (acc.0 + g.0 / n as f64, acc.1 + g.1 / n as f64)
                });
                let off = mean.0.hypot(mean.1);
                println!(
                    "at r = {r:.0} M the {n} generators sit on the unit circle to {worst:.2e}, \
                     centred to {off:.2e}"
                );
                assert!(off < 1e-3, "the rim is off centre by {off} at r={r}");
            }
        }
    }
}
