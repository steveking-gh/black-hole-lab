//! The Kerr metric in Cartesian ingoing Kerr-Schild coordinates (t, x, y, z), and Hamilton's
//! equations for light in it.
//!
//! # The chart
//!
//! The metric is flat space plus a null rank-one piece,
//!
//!     g_{mu nu} = eta_{mu nu} + 2 H l_mu l_nu,      eta = diag(-1, 1, 1, 1),
//!
//!     H     = M r^3 / (r^4 + a^2 z^2),
//!     l_mu  = (1, (r x + a y) / (r^2 + a^2), (r y - a x) / (r^2 + a^2), z / r),
//!
//! where r is not a coordinate but the function of position fixed by
//!
//!     r^4 - (x^2 + y^2 + z^2 - a^2) r^2 - a^2 z^2 = 0,      r > 0,
//!
//! whose positive root is r^2 = [w + sqrt(w^2 + 4 a^2 z^2)] / 2 with w = x^2 + y^2 + z^2 - a^2.
//!
//! # Why these signs: the pull-back to the app's chart
//!
//! The signs of a above were not copied from a textbook, where both conventions are printed. They
//! are the ones for which the metric agrees with `kerr_equatorial`, and the agreement is derived
//! here and tested below (`test_the_metric_pulled_back_to_the_equatorial_chart_is_kerr_equatorials`,
//! the first test written for this crate).
//!
//! The spheroidal coordinates behind the Cartesian ones are the embedding
//!
//!     x + i y = (r + i a) sin(theta) e^{i phi},      z = r cos(theta),
//!
//! which on theta = pi/2 is `KerrSchild::cartesian_position`, x + iy = (r + ia) e^{i phi}, exactly.
//! Then x^2 + y^2 = (r^2 + a^2) sin^2(theta) and z^2 = r^2 cos^2(theta), which eliminate theta to
//! give the quartic for r above. Pulling back the spatial part of l,
//!
//!     (r x + a y) dx + (r y - a x) dy = Re[(r + i a) conj(w) dw],   w = x + i y,
//!
//! because (r x + a y) + i (r y - a x) = (r - i a) w. With conj(w) = (r - i a) sin(theta) e^{-i phi}
//! and dw = [sin(theta) dr + (r + i a) cos(theta) d theta + i (r + i a) sin(theta) d phi] e^{i phi},
//! the real part divided by r^2 + a^2 is sin^2(theta) dr + r sin(theta) cos(theta) d theta
//! - a sin^2(theta) d phi; and (z / r) dz = cos^2(theta) dr - r sin(theta) cos(theta) d theta. So
//!
//!     l_mu dx^mu = dt + dr - a sin^2(theta) d phi,
//!
//! the ingoing principal null covector of Kerr in ingoing Kerr coordinates (t, r, theta, phi), and
//! H = M r^3 / (r^4 + a^2 z^2) = M r / (r^2 + a^2 cos^2 theta) = M r / Sigma. The same pull-back
//! of eta gives dr^2 - 2 a sin^2(theta) dr d phi + Sigma d theta^2 + (r^2 + a^2) sin^2(theta) d phi^2.
//! That is the form which `crates/kerr-equatorial/scripts/kerr_sympy_oracle.py` derives from the
//! Boyer-Lindquist line element and proves Ricci-flat, and on theta = pi/2 it is term by term the
//! `eta + 2 H l l` of `KerrSchild::inverse_metric`'s comment, with l = (1, 1, -a), H = M / r: the
//! metric `KerrSchild::metric_components` returns. The t of this chart is therefore the app's t,
//! the phi of the embedding is the app's phi, and the tests hold both statements to rounding, on
//! the plane against the app and off it against the closed form above.
//!
//! Two properties make the form worth having. l is null for eta (|l_spatial|^2 = sin^2 + cos^2 =
//! 1), so the inverse is the same rank-one correction with the opposite sign,
//!
//!     g^{mu nu} = eta^{mu nu} - 2 H l^mu l^nu,      l^mu = eta^{mu nu} l_nu = (-1, l_x, l_y, l_z),
//!
//! and det g = det eta = -1 everywhere. And the chart has no coordinate axis: the spin axis
//! x = y = 0 is an ordinary line (there r = |z| and l = (1, 0, 0, sign z)), which is why rays that
//! leave the equatorial plane and cross the axis can be followed here and not in spherical
//! coordinates. The only singularity is the ring x^2 + y^2 = a^2, z = 0, with the disc it bounds
//! as a branch cut of r; no ray this crate follows goes inside r-, where both lie.
//!
//! # The azimuth off the plane
//!
//! The winding (specification section 4.8) counts turns in the chart's own azimuth. Off the plane
//! the embedding gives e^{i phi} = (x + i y) / [(r + i a) sin(theta)], and sin(theta) > 0 off the
//! axis, so
//!
//!     phi = atan2(y, x) - atan2(a, r),
//!
//! with r the spheroidal radius at (x, y, z). On the plane it is `KerrSchild::chart_point`'s
//! formula; it is undefined only on the axis itself, which a ray crosses on a set of measure zero.
//! At large r it tends to atan2(y, x): the far-sky azimuth of the specification's section 4.4.
//!
//! # Hamilton's equations
//!
//! Light is followed as the flow of the super-Hamiltonian
//!
//!     Ham(x, p) = (1/2) g^{mu nu} p_mu p_nu = (1/2)(-p_t^2 + |p|^2) - H s^2,   s = l^mu p_mu,
//!
//! with s = -p_t + l_x p_x + l_y p_y + l_z p_z. The metric does not depend on t, so p_t is a
//! constant and the state is the six numbers (x, y, z, p_x, p_y, p_z). Differentiating,
//!
//!     dx^i / d lambda =  d Ham / d p_i = p_i - 2 H s l_i,
//!     dt   / d lambda =  d Ham / d p_t = -p_t + 2 H s,
//!     dp_i / d lambda = -d Ham / d x^i = s^2 d_i H + 2 H s (d_i l_j) p_j,
//!
//! all in closed form through the gradient of r. Implicit differentiation of the quartic gives
//!
//!     d_x r = x r^3 / (r^4 + a^2 z^2),   d_y r = y r^3 / (r^4 + a^2 z^2),
//!     d_z r = z r (r^2 + a^2) / (r^4 + a^2 z^2),
//!
//! and from H = M r^3 / (r^4 + a^2 z^2),
//!
//!     dH/dr|_z = M r^2 (3 a^2 z^2 - r^4) / (r^4 + a^2 z^2)^2,
//!     dH/dz|_r = -2 M a^2 r^3 z / (r^4 + a^2 z^2)^2.
//!
//! For the l terms write l_x p_x + l_y p_y = N / S with N = r (x p_x + y p_y) + a (y p_x - x p_y)
//! and S = r^2 + a^2; then d_i (N / S) = (d_i N - 2 r (N / S) d_i r) / S with
//! d_i N = (x p_x + y p_y) d_i r + r p_i [i in {x, y}] + a (p_x [i = y] - p_y [i = x]), and
//! d_i (z p_z / r) = p_z ([i = z] - (z / r) d_i r) / r. No finite difference enters the flow; a test
//! checks it against one.

use kerr_equatorial::KerrSchild;

/// A Kerr hole of mass M and spin a in geometric units (G = c = 1), described in Cartesian
/// ingoing Kerr-Schild coordinates. See the module documentation for the metric.
///
/// Unlike `KerrSchild::new` this does not clamp: M = 0, a = 0 is flat space, which is what the
/// flat-limit tests need, and a spin is taken as given as long as it is sub-extremal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Kerr {
    /// Mass M.
    pub m: f64,
    /// Spin parameter a = J / M, of either sign; the hole turns in the +phi sense for a > 0.
    pub a: f64,
}

/// The scalar and covector of the Kerr-Schild form at one point, with the spheroidal radius r they
/// are functions of.
#[derive(Debug, Clone, Copy)]
pub struct Field {
    /// Spheroidal radius r.
    pub r: f64,
    /// H = M r^3 / (r^4 + a^2 z^2).
    pub h: f64,
    /// The spatial components (l_x, l_y, l_z) of the null covector; l_t = 1.
    pub l: [f64; 3],
}

/// E, L_z and Carter's Q of a null covector at a point, and the super-Hamiltonian, which is zero
/// for light.
#[derive(Debug, Clone, Copy)]
pub struct Constants {
    /// E = -p_t, the energy at infinity; positive for a future-directed ray that reaches infinity.
    pub energy: f64,
    /// L_z = p_phi = x p_y - y p_x, the angular momentum about the spin axis.
    pub lz: f64,
    /// Carter's constant Q, in the convention in which Q = 0 for a ray in the equatorial plane.
    pub carter_q: f64,
    /// (1/2) g^{mu nu} p_mu p_nu.
    pub hamiltonian: f64,
}

impl Kerr {
    /// A hole of mass m and spin a. Panics unless |a| < m, or m = a = 0 (flat space): an extremal
    /// or over-spun hole has no inner horizon to put the scope of this crate against.
    pub fn new(m: f64, a: f64) -> Self {
        assert!(
            m.is_finite() && a.is_finite() && m >= 0.0 && (a.abs() < m || (m == 0.0 && a == 0.0)),
            "Kerr needs 0 <= |a| < M (or flat space, M = a = 0); got M = {m}, a = {a}"
        );
        Self { m, a }
    }

    /// The same hole as the app's equatorial geometry.
    pub fn from_equatorial(metric: &KerrSchild) -> Self {
        Self::new(metric.m, metric.a)
    }

    /// The app's equatorial geometry for this hole, built without `KerrSchild::new`'s clamps so
    /// that M = 0 stays flat space. `m_solar` plays no part in the geometry.
    pub fn equatorial(&self) -> KerrSchild {
        KerrSchild {
            m: self.m,
            m_solar: 1.0,
            a: self.a,
        }
    }

    /// Outer horizon r+ = M + sqrt(M^2 - a^2).
    pub fn outer_horizon(&self) -> f64 {
        self.m + (self.m * self.m - self.a * self.a).max(0.0).sqrt()
    }

    /// Inner horizon r- = M - sqrt(M^2 - a^2).
    pub fn inner_horizon(&self) -> f64 {
        self.m - (self.m * self.m - self.a * self.a).max(0.0).sqrt()
    }

    /// Delta(r) = r^2 - 2 M r + a^2.
    pub fn delta(&self, r: f64) -> f64 {
        r * r - 2.0 * self.m * r + self.a * self.a
    }

    /// Radius of the prograde circular photon orbit on the equator,
    /// 2M {1 + cos[(2/3) arccos(-|a|/M)]}: the smallest radius at which any ray that comes in from
    /// the far sky turns round (see `ray::PAST_HORIZON_MARGIN`). 3M without spin.
    pub fn prograde_photon_orbit(&self) -> f64 {
        if self.m == 0.0 {
            return 0.0;
        }
        let spin = (self.a.abs() / self.m).min(1.0);
        2.0 * self.m * (1.0 + (2.0 / 3.0 * (-spin).acos()).cos())
    }

    /// The spheroidal radius r at a Cartesian position: the positive root of
    /// r^4 - (x^2 + y^2 + z^2 - a^2) r^2 - a^2 z^2 = 0.
    ///
    /// r^2 = [w + sqrt(w^2 + 4 a^2 z^2)] / 2 with w = x^2 + y^2 + z^2 - a^2. Where w < 0 - inside
    /// the ring's sphere, which no ray here reaches - the sum cancels, and the equivalent
    /// 2 a^2 z^2 / [sqrt(w^2 + 4 a^2 z^2) - w] is used instead.
    #[inline]
    pub fn radius(&self, p: [f64; 3]) -> f64 {
        let a2 = self.a * self.a;
        let w = p[0] * p[0] + p[1] * p[1] + p[2] * p[2] - a2;
        let root = (w * w + 4.0 * a2 * p[2] * p[2]).sqrt();
        let r2 = if w >= 0.0 {
            0.5 * (w + root)
        } else {
            2.0 * a2 * p[2] * p[2] / (root - w)
        };
        r2.sqrt()
    }

    /// H, l and r at a Cartesian position.
    #[inline]
    pub fn field(&self, p: [f64; 3]) -> Field {
        let r = self.radius(p);
        let a = self.a;
        let s = r * r + a * a;
        let q4 = r * r * r * r + a * a * p[2] * p[2];
        Field {
            r,
            h: self.m * r * r * r / q4,
            l: [
                (r * p[0] + a * p[1]) / s,
                (r * p[1] - a * p[0]) / s,
                p[2] / r,
            ],
        }
    }

    /// g_{mu nu} at a Cartesian position, index order (t, x, y, z).
    pub fn metric(&self, p: [f64; 3]) -> [[f64; 4]; 4] {
        let f = self.field(p);
        let l = [1.0, f.l[0], f.l[1], f.l[2]];
        let mut g = [[0.0; 4]; 4];
        for (mu, row) in g.iter_mut().enumerate() {
            for (nu, entry) in row.iter_mut().enumerate() {
                let eta = match (mu, nu) {
                    (0, 0) => -1.0,
                    (i, j) if i == j => 1.0,
                    _ => 0.0,
                };
                *entry = eta + 2.0 * f.h * l[mu] * l[nu];
            }
        }
        g
    }

    /// g^{mu nu} = eta^{mu nu} - 2 H l^mu l^nu with l^mu = (-1, l_x, l_y, l_z).
    pub fn inverse_metric(&self, p: [f64; 3]) -> [[f64; 4]; 4] {
        let f = self.field(p);
        let l = [-1.0, f.l[0], f.l[1], f.l[2]];
        let mut g = [[0.0; 4]; 4];
        for (mu, row) in g.iter_mut().enumerate() {
            for (nu, entry) in row.iter_mut().enumerate() {
                let eta = match (mu, nu) {
                    (0, 0) => -1.0,
                    (i, j) if i == j => 1.0,
                    _ => 0.0,
                };
                *entry = eta - 2.0 * f.h * l[mu] * l[nu];
            }
        }
        g
    }

    /// v_mu = g_{mu nu} v^nu at p.
    pub fn lower(&self, p: [f64; 3], v: [f64; 4]) -> [f64; 4] {
        let f = self.field(p);
        // g_{mu nu} v^nu = eta_{mu nu} v^nu + 2 H l_mu (l_nu v^nu).
        let lv = v[0] + f.l[0] * v[1] + f.l[1] * v[2] + f.l[2] * v[3];
        let c = 2.0 * f.h * lv;
        [
            -v[0] + c,
            v[1] + c * f.l[0],
            v[2] + c * f.l[1],
            v[3] + c * f.l[2],
        ]
    }

    /// g_{mu nu} u^mu v^nu at p.
    pub fn dot(&self, p: [f64; 3], u: [f64; 4], v: [f64; 4]) -> f64 {
        let lowered = self.lower(p, u);
        lowered.iter().zip(v.iter()).map(|(a, b)| a * b).sum()
    }

    /// The chart's azimuth at a Cartesian position, phi = atan2(y, x) - atan2(a, r): the phi of
    /// x + iy = (r + ia) sin(theta) e^{i phi}. See the module documentation.
    #[inline]
    pub fn azimuth(&self, p: [f64; 3]) -> f64 {
        p[1].atan2(p[0]) - self.a.atan2(self.radius(p))
    }

    /// The Cartesian position of the spheroidal point (r, theta, phi):
    /// x + iy = (r + ia) sin(theta) e^{i phi}, z = r cos(theta).
    pub fn embed(&self, r: f64, theta: f64, phi: f64) -> [f64; 3] {
        let (st, ct) = theta.sin_cos();
        let (sp, cp) = phi.sin_cos();
        [
            st * (r * cp - self.a * sp),
            st * (r * sp + self.a * cp),
            r * ct,
        ]
    }

    /// The Cartesian position of the equatorial chart point (r, phi), by `kerr_equatorial`'s own
    /// embedding, so that an observer is placed exactly where the app draws them.
    pub fn embed_equatorial(&self, r: f64, phi: f64) -> [f64; 3] {
        let (x, y) = self.equatorial().cartesian_position(r, phi);
        [x, y, 0.0]
    }

    /// A vector with equatorial chart components (v^t, v^r, v^phi) at (r, phi), in Cartesian
    /// components (v^t, v^x, v^y, v^z). The (x, y) part is the Jacobian of `cartesian_position`,
    /// which `KerrSchild::cartesian_velocity` is (it is linear in its last two arguments); v^z = 0
    /// because the vector lies in the plane.
    pub fn chart_vector(&self, r: f64, phi: f64, v: [f64; 3]) -> [f64; 4] {
        let (vx, vy) = self.equatorial().cartesian_velocity(r, phi, v[1], v[2]);
        [v[0], vx, vy, 0.0]
    }

    /// E, L_z, Q and the super-Hamiltonian of a covector p_mu = (p_t, p_x, p_y, p_z) at a point.
    ///
    /// Carter's constant in Boyer-Lindquist form is
    ///
    ///     Q = p_theta^2 + cos^2(theta) [L_z^2 / sin^2(theta) - a^2 E^2]        (light, mu = 0),
    ///
    /// and it holds unchanged in ingoing Kerr coordinates: they differ from Boyer-Lindquist only
    /// by t and phi shifted by functions of r, which leaves p_theta, p_t and p_phi as they are.
    /// The Cartesian components are reached through the embedding: at fixed (t, r, phi),
    /// d(x + iy)/d theta = (x + iy) cot(theta) and dz/d theta = -r sin(theta), so
    ///
    ///     p_theta = cot(theta) (x p_x + y p_y) - r sin(theta) p_z,     p_phi = x p_y - y p_x.
    ///
    /// Squaring, the cot^2 terms join the L_z^2 / sin^2 term through
    /// (x p_x + y p_y)^2 + (x p_y - y p_x)^2 = (x^2 + y^2)(p_x^2 + p_y^2) and
    /// x^2 + y^2 = (r^2 + a^2) sin^2(theta), and every division by sin(theta) cancels:
    ///
    ///     Q = (z^2 / r^2)(r^2 + a^2)(p_x^2 + p_y^2) - 2 z (x p_x + y p_y) p_z
    ///         + (r^2 - z^2) p_z^2 - a^2 E^2 z^2 / r^2,
    ///
    /// regular on the axis, where rays do pass. With a = 0 it is |x cross p|^2 - L_z^2, the
    /// squared angular momentum about the axes perpendicular to z, as it must be.
    pub fn constants(&self, p: [f64; 3], k: [f64; 4]) -> Constants {
        let r = self.radius(p);
        let (x, y, z) = (p[0], p[1], p[2]);
        let (px, py, pz) = (k[1], k[2], k[3]);
        let energy = -k[0];
        let a2 = self.a * self.a;
        let cos2 = if r > 0.0 { z * z / (r * r) } else { 0.0 };
        let carter_q = cos2 * (r * r + a2) * (px * px + py * py) - 2.0 * z * (x * px + y * py) * pz
            + (r * r - z * z) * pz * pz
            - a2 * energy * energy * cos2;
        let ginv = self.inverse_metric(p);
        let mut ham = 0.0;
        for mu in 0..4 {
            for nu in 0..4 {
                ham += 0.5 * ginv[mu][nu] * k[mu] * k[nu];
            }
        }
        Constants {
            energy,
            lz: x * py - y * px,
            carter_q,
            hamiltonian: ham,
        }
    }

    /// The right-hand side of Hamilton's equations for the state y = (x, y, z, p_x, p_y, p_z) with
    /// p_t = -`energy` held constant: (dx^i / d lambda, dp_i / d lambda). See the module
    /// documentation for the derivation.
    #[inline]
    pub fn hamilton(&self, energy: f64, y: &[f64; 6]) -> [f64; 6] {
        let (x, yy, z) = (y[0], y[1], y[2]);
        let (px, py, pz) = (y[3], y[4], y[5]);
        let (m, a) = (self.m, self.a);
        let a2 = a * a;
        let r = self.radius([x, yy, z]);
        let r2 = r * r;
        let s_den = r2 + a2;
        let q4 = r2 * r2 + a2 * z * z;
        let inv_q4 = 1.0 / q4;
        let h = m * r2 * r * inv_q4;
        let inv_s = 1.0 / s_den;
        let inv_r = 1.0 / r;
        let l = [
            (r * x + a * yy) * inv_s,
            (r * yy - a * x) * inv_s,
            z * inv_r,
        ];
        // s = l^mu p_mu with l^t = -1 and p_t = -E.
        let s = energy + l[0] * px + l[1] * py + l[2] * pz;

        // Gradient of r.
        let c = r2 * r * inv_q4;
        let dr = [x * c, yy * c, z * c * s_den / r2];
        // Gradient of H: dH/dr at fixed z times grad r, plus dH/dz at fixed r on the z component.
        let h_r = m * r2 * (3.0 * a2 * z * z - r2 * r2) * inv_q4 * inv_q4;
        let h_z = -2.0 * m * a2 * r2 * r * z * inv_q4 * inv_q4;
        let dh = [h_r * dr[0], h_r * dr[1], h_r * dr[2] + h_z];

        // d_i (l_j p_j) at fixed p, in the N / S and z p_z / r pieces of the module comment.
        let xp = x * px + yy * py;
        let n_over_s = (r * xp + a * (yy * px - x * py)) * inv_s;
        let dn = [
            xp * dr[0] + r * px - a * py,
            xp * dr[1] + r * py + a * px,
            xp * dr[2],
        ];
        let lz_term = z * inv_r;
        let dlp = [
            (dn[0] - 2.0 * r * n_over_s * dr[0]) * inv_s - pz * lz_term * dr[0] * inv_r,
            (dn[1] - 2.0 * r * n_over_s * dr[1]) * inv_s - pz * lz_term * dr[1] * inv_r,
            (dn[2] - 2.0 * r * n_over_s * dr[2]) * inv_s + pz * (1.0 - lz_term * dr[2]) * inv_r,
        ];

        let two_hs = 2.0 * h * s;
        let s2 = s * s;
        [
            px - two_hs * l[0],
            py - two_hs * l[1],
            pz - two_hs * l[2],
            s2 * dh[0] + two_hs * dlp[0],
            s2 * dh[1] + two_hs * dlp[1],
            s2 * dh[2] + two_hs * dlp[2],
        ]
    }

    /// dr / d lambda along the flow at state y: grad r . dx/d lambda.
    #[inline]
    pub fn radial_rate(&self, y: &[f64; 6], velocity: &[f64; 3]) -> f64 {
        let (x, yy, z) = (y[0], y[1], y[2]);
        let a2 = self.a * self.a;
        let r = self.radius([x, yy, z]);
        let r2 = r * r;
        let q4 = r2 * r2 + a2 * z * z;
        let c = r2 * r / q4;
        c * (x * velocity[0] + yy * velocity[1] + z * velocity[2] * (r2 + a2) / r2)
    }
}

#[cfg(test)]
// Tensor components are indexed by their indices, as the formulae write them.
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;

    /// Spins including none and one near extremal.
    const SPINS: [f64; 5] = [0.0, 0.3, 0.65, 0.9, 0.998];

    /// Radii outside r+, between the horizons and inside r- (where there is one).
    fn probe_radii(kerr: &Kerr) -> Vec<f64> {
        let (rp, rm) = (kerr.outer_horizon(), kerr.inner_horizon());
        let mut radii = vec![40.0, 6.0, 3.0, 1.01 * rp, 0.5 * (rp + rm)];
        if rm > 0.02 {
            radii.extend([0.99 * rm, 0.5 * rm]);
        } else {
            radii.push(0.3);
        }
        radii
    }

    #[test]
    fn test_the_metric_pulled_back_to_the_equatorial_chart_is_kerr_equatorials() {
        // The test that fixed this crate's conventions. On z = 0 the chart point (r, phi) sits at
        // (x, y) = `KerrSchild::cartesian_position(r, phi)`, and the Jacobian of that map is
        //     d(x, y)/dr = (cos phi, sin phi),   d(x, y)/d phi = (-y, x),
        // so the (t, r, phi) components of the Cartesian metric are J^T g J, which must equal
        // `KerrSchild::metric_components(r)`. The z row plays no part: on the plane l_z = z/r = 0,
        // so g_{z mu} = delta_{z mu} and the plane is orthogonal to d/dz. Checked outside r+,
        // between the horizons and inside r-, at several spins and azimuths, to rounding.
        let mut worst = 0.0f64;
        for &a in &SPINS {
            let kerr = Kerr::new(1.0, a);
            let eq = kerr.equatorial();
            for &r in &probe_radii(&kerr) {
                for &phi in &[0.0, 0.7, 2.9, -1.9, 5.5] {
                    let p = kerr.embed_equatorial(r, phi);
                    let g = kerr.metric(p);
                    let (s, c) = phi.sin_cos();
                    let jac = [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, c, s, 0.0],
                        [0.0, -p[1], p[0], 0.0],
                    ];
                    let want = eq.metric_components(r);
                    let scale = want.iter().flatten().fold(1.0f64, |m, v| m.max(v.abs()));
                    for i in 0..3 {
                        for j in 0..3 {
                            let mut got = 0.0;
                            for mu in 0..4 {
                                for nu in 0..4 {
                                    got += jac[i][mu] * g[mu][nu] * jac[j][nu];
                                }
                            }
                            let err = (got - want[i][j]).abs() / scale;
                            worst = worst.max(err);
                            assert!(
                                err < 1e-14,
                                "g[{i}][{j}]: pulled back {got}, kerr-equatorial {} at r = {r}, \
                                 phi = {phi}, a = {a}",
                                want[i][j]
                            );
                        }
                    }
                    for mu in 0..4 {
                        let want = if mu == 3 { 1.0 } else { 0.0 };
                        assert!(
                            (g[3][mu] - want).abs() < 1e-15,
                            "g_z{mu} = {} on the plane",
                            g[3][mu]
                        );
                    }
                }
            }
        }
        println!(
            "equatorial pull-back agrees with kerr-equatorial to {worst:.1e} of the largest entry"
        );
    }

    #[test]
    fn test_the_metric_pulled_back_off_the_plane_is_ingoing_kerr() {
        // Off the plane there is no app chart to compare with, so the comparison is with the
        // closed form in ingoing Kerr coordinates (t, r, theta, phi) that
        // crates/kerr-equatorial/scripts/kerr_sympy_oracle.py derives from Boyer-Lindquist and
        // proves Ricci-flat: eta + 2 H l l with H = M r / Sigma, l = (1, 1, 0, -a sin^2 theta),
        // and eta's (r, phi) block (1, -a sin^2, (r^2 + a^2) sin^2), eta_{theta theta} = Sigma.
        let mut worst = 0.0f64;
        for &a in &SPINS {
            let kerr = Kerr::new(1.0, a);
            for &r in &probe_radii(&kerr) {
                for &theta in &[0.2, 0.9, 1.4, 2.3, 3.0] {
                    let phi = 1.3;
                    let p = kerr.embed(r, theta, phi);
                    assert!(
                        (kerr.radius(p) - r).abs() < 1e-12 * r.max(1.0),
                        "the quartic returns r: {} vs {r}",
                        kerr.radius(p)
                    );
                    let g = kerr.metric(p);
                    let (st, ct) = theta.sin_cos();
                    let (sp, cp) = phi.sin_cos();
                    // Columns d/dt, d/dr, d/dtheta, d/dphi in Cartesian components.
                    let jac = [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, st * cp, st * sp, ct],
                        [0.0, ct * (r * cp - a * sp), ct * (r * sp + a * cp), -r * st],
                        [0.0, -p[1], p[0], 0.0],
                    ];
                    let sigma = r * r + a * a * ct * ct;
                    let h = r / sigma;
                    let l = [1.0, 1.0, 0.0, -a * st * st];
                    let mut eta = [[0.0; 4]; 4];
                    eta[0][0] = -1.0;
                    eta[1][1] = 1.0;
                    eta[1][3] = -a * st * st;
                    eta[3][1] = eta[1][3];
                    eta[2][2] = sigma;
                    eta[3][3] = (r * r + a * a) * st * st;
                    for i in 0..4 {
                        for j in 0..4 {
                            let want = eta[i][j] + 2.0 * h * l[i] * l[j];
                            let mut got = 0.0;
                            for mu in 0..4 {
                                for nu in 0..4 {
                                    got += jac[i][mu] * g[mu][nu] * jac[j][nu];
                                }
                            }
                            let scale = (r * r + a * a).max(1.0);
                            let err = (got - want).abs() / scale;
                            worst = worst.max(err);
                            assert!(
                                err < 1e-13,
                                "g[{i}][{j}] = {got} vs {want} at r = {r}, theta = {theta}, a = {a}"
                            );
                        }
                    }
                }
            }
        }
        println!("off-plane pull-back agrees with ingoing Kerr to {worst:.1e}");
    }

    #[test]
    fn test_the_inverse_metric_is_an_inverse_and_the_determinant_is_minus_one() {
        for &a in &SPINS {
            let kerr = Kerr::new(1.0, a);
            for &r in &probe_radii(&kerr) {
                for &theta in &[0.3, std::f64::consts::FRAC_PI_2, 2.6] {
                    let p = kerr.embed(r, theta, 0.4);
                    let (g, gi) = (kerr.metric(p), kerr.inverse_metric(p));
                    for i in 0..4 {
                        for j in 0..4 {
                            let s: f64 = (0..4).map(|k| g[i][k] * gi[k][j]).sum();
                            let want = if i == j { 1.0 } else { 0.0 };
                            assert!(
                                (s - want).abs() < 1e-12,
                                "g g^-1 [{i}][{j}] = {s} at r = {r}"
                            );
                        }
                    }
                    // The determinant is a sum of products of four entries, each as large as 2H,
                    // which grows like M / r near the ring: its rounding floor is ~1e-16 (1 + 2H)^4.
                    let det = det4(&g);
                    let floor = 1e-15 * (1.0 + 2.0 * kerr.field(p).h).powi(4);
                    assert!(
                        (det + 1.0).abs() < floor,
                        "det g = {det} at r = {r}, a = {a}"
                    );
                }
            }
        }
    }

    fn det4(m: &[[f64; 4]; 4]) -> f64 {
        let minor = |skip: usize| {
            let rows: Vec<[f64; 3]> = (1..4)
                .map(|i| {
                    let mut row = [0.0; 3];
                    let mut k = 0;
                    for j in 0..4 {
                        if j != skip {
                            row[k] = m[i][j];
                            k += 1;
                        }
                    }
                    row
                })
                .collect();
            rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
                - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
                + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
        };
        (0..4)
            .map(|j| if j % 2 == 0 { 1.0 } else { -1.0 } * m[0][j] * minor(j))
            .sum()
    }

    #[test]
    fn test_hamiltons_equations_match_finite_differences_of_the_hamiltonian() {
        // The flow is written out in closed form; this checks it against central differences of
        // Ham(x, p) = (1/2) g^{mu nu}(x) p_mu p_nu computed from `inverse_metric`, which shares
        // none of that algebra. Off the plane, near the axis, and between the horizons.
        for &a in &[0.0, 0.6, 0.95] {
            let kerr = Kerr::new(1.0, a);
            for &(r, theta, phi) in &[
                (7.0, 0.8, 0.3),
                (2.5, 1.9, -2.0),
                (1.2, 1.2, 1.0),
                (4.0, 0.02, 2.2),
            ] {
                let pos = kerr.embed(r, theta, phi);
                let k = [0.37, -0.81, 0.29];
                let energy = 0.9;
                let ham = |pos: [f64; 3], k: [f64; 3]| {
                    let cov = [-energy, k[0], k[1], k[2]];
                    let gi = kerr.inverse_metric(pos);
                    let mut s = 0.0;
                    for mu in 0..4 {
                        for nu in 0..4 {
                            s += 0.5 * gi[mu][nu] * cov[mu] * cov[nu];
                        }
                    }
                    s
                };
                let y = [pos[0], pos[1], pos[2], k[0], k[1], k[2]];
                let f = kerr.hamilton(energy, &y);
                for i in 0..3 {
                    let step = 1e-6;
                    let (mut kp, mut km) = (k, k);
                    kp[i] += step;
                    km[i] -= step;
                    let dx = (ham(pos, kp) - ham(pos, km)) / (2.0 * step);
                    let (mut pp, mut pm) = (pos, pos);
                    pp[i] += step;
                    pm[i] -= step;
                    let dp = -(ham(pp, k) - ham(pm, k)) / (2.0 * step);
                    assert!(
                        (f[i] - dx).abs() < 1e-8,
                        "dx^{i}: {} vs {dx} at r = {r} (a = {a})",
                        f[i]
                    );
                    assert!(
                        (f[3 + i] - dp).abs() < 1e-7,
                        "dp_{i}: {} vs {dp} at r = {r}, theta = {theta} (a = {a})",
                        f[3 + i]
                    );
                }
                let vel = [f[0], f[1], f[2]];
                let rate = kerr.radial_rate(&y, &vel);
                let step = 1e-6;
                let ahead = kerr.radius([
                    pos[0] + step * vel[0],
                    pos[1] + step * vel[1],
                    pos[2] + step * vel[2],
                ]);
                let behind = kerr.radius([
                    pos[0] - step * vel[0],
                    pos[1] - step * vel[1],
                    pos[2] - step * vel[2],
                ]);
                let fd = (ahead - behind) / (2.0 * step);
                assert!((rate - fd).abs() < 1e-8, "dr/d lambda {rate} vs {fd}");
            }
        }
    }

    #[test]
    fn test_carters_constant_matches_its_boyer_lindquist_form() {
        // The Cartesian Q against Q = p_theta^2 + cos^2 [L^2 / sin^2 - a^2 E^2] with p_theta and
        // p_phi obtained by pulling p back to (t, r, theta, phi) with the embedding's Jacobian.
        for &a in &[0.0, 0.5, 0.9] {
            let kerr = Kerr::new(1.0, a);
            for &(r, theta, phi) in &[(6.0, 0.7, 0.2), (2.0, 2.2, 1.7), (15.0, 1.5, -0.4)] {
                let p = kerr.embed(r, theta, phi);
                let k = [-1.1, 0.4, -0.3, 0.8];
                let (st, ct) = theta.sin_cos();
                let (sp, cp) = phi.sin_cos();
                let d_theta = [ct * (r * cp - a * sp), ct * (r * sp + a * cp), -r * st];
                let p_theta = d_theta[0] * k[1] + d_theta[1] * k[2] + d_theta[2] * k[3];
                let p_phi = -p[1] * k[1] + p[0] * k[2];
                let e = -k[0];
                let want =
                    p_theta * p_theta + ct * ct * (p_phi * p_phi / (st * st) - a * a * e * e);
                let c = kerr.constants(p, k);
                assert!(
                    (c.carter_q - want).abs() < 1e-12 * (1.0 + want.abs()),
                    "Q {} vs {want}",
                    c.carter_q
                );
                assert!((c.lz - p_phi).abs() < 1e-14);
                assert_eq!(c.energy, e);
            }
        }
    }

    #[test]
    fn test_the_azimuth_off_the_plane_inverts_the_embedding() {
        for &a in &[0.0, 0.7, 0.99] {
            let kerr = Kerr::new(1.0, a);
            for &r in &[0.8, 2.0, 9.0, 1e4] {
                for &theta in &[0.1, 1.0, std::f64::consts::FRAC_PI_2, 2.8] {
                    for &phi in &[0.0, 1.0, -2.5, 3.0] {
                        let got = kerr.azimuth(kerr.embed(r, theta, phi));
                        let d = (got - phi).rem_euclid(std::f64::consts::TAU);
                        let d = d.min(std::f64::consts::TAU - d);
                        assert!(d < 1e-12, "phi {got} vs {phi} at r = {r}, theta = {theta}");
                    }
                }
            }
        }
    }
}
