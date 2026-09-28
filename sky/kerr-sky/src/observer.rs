//! The observer: an event on the equatorial plane with a 4-velocity, exactly as `kerr_equatorial`
//! and the app's save files carry them, and the orthonormal triad that directions on the
//! observer's sky are written in.
//!
//! # The triad
//!
//! The specification (section 4.2) asks for three unit vectors x, y, z in the observer's rest space,
//! orthonormal and right-handed, x cross y = z. They are built here as follows.
//!
//! **z, along the spin axis.** On the plane z = 0 the covector l of the metric has l_z = z / r = 0,
//! so g_{z mu} = eta_{z mu} + 2 H l_z l_mu = delta_{z mu}: the coordinate direction d/dz is a unit
//! vector (g_zz = 1) orthogonal to d/dt, d/dx and d/dy. The observer's 4-velocity lies in the plane
//! (u^z = 0), so it is a combination of those three and g(d/dz, u) = 0 too. d/dz is therefore
//! already a unit vector of the observer's rest space, for every observer on the plane, at every
//! radius - no projection, no normalisation - and it points along the hole's spin axis, in the
//! sense of its angular momentum when a > 0. `test_the_z_leg_is_unit_and_orthogonal_to_the_plane`
//! checks it with the metric.
//!
//! **x and y, in the plane.** `kerr_equatorial::Tetrad::from_four_velocity` gives the observer's
//! own frame (e0, e1, e2) = (u, radial, azimuthal) in chart components: e1 is the rest-space
//! projection of d/dr, normalised, so it points outward wherever outward means anything (g(e1,
//! d_r) > 0), and e2 is what remains of d/dphi, pointing prograde (the +phi sense). Its construction
//! is regular at every r > 0, including between and inside the horizons, and at any boost (see its
//! documentation). The two legs are mapped into Cartesian components with the Jacobian of
//! `cartesian_position` (`Kerr::chart_vector`); a map of tangent vectors preserves the inner
//! products, so they stay orthonormal and orthogonal to u and to d/dz.
//!
//! **Handedness.** Far from the hole, for an observer at rest at phi = 0, e1 is d/dx and e2 is
//! d/dy, so (e1, e2, d/dz) is right-handed, and since the frame is orthonormal its orientation
//! cannot change as the observer moves or boosts. The test measures it everywhere it is used: the
//! 4 x 4 determinant of (u, x, y, z) in Cartesian components is +1, which, because det g = -1 in
//! this chart, is the statement that the frame is orthonormal, future-directed and right-handed.
//!
//! **Heading.** The in-plane legs are then turned about z by a heading angle psi,
//!
//!     x = cos(psi) e1 + sin(psi) e2,     y = -sin(psi) e1 + cos(psi) e2,
//!
//! (`Tetrad::turned`, exact at any boost). psi = 0 faces along the rest-space part of the chart's
//! d/dr, psi = pi against it, psi = pi/2 along the prograde leg. That is "outward" and "toward the
//! hole" only loosely: d/dr is taken at fixed Kerr-Schild t and phi, so its rest-space part depends
//! on the chart's slicing, and for a static observer at 6 M around a = 0.9 M it is turned 10.4
//! degrees from the gradient of r, which does not. [`Triad::towards`] turns x onto the rest-space
//! direction of any vector the caller names instead - minus the gradient of r, which is how
//! `sky-trace` faces the hole; the observer's motion relative to some frame; or a gyroscope's axis
//! carried from the last frame. Which of these a film uses is the caller's decision.
//!
//! # What a direction on the sky means
//!
//! A direction n = (n_x, n_y, n_z) in the triad is where the observer LOOKS (specification 4.3).
//! The light seen there travels along -n, so its 4-momentum, normalised to unit frequency in the
//! observer's frame, is
//!
//!     p^mu = u^mu - n_x x^mu - n_y y^mu - n_z z^mu,       -p . u = 1,
//!
//! future-directed and null because the triad is orthonormal. To trace it backward the ray is
//! followed along -p, the past-directed tangent: see `ray`.
//!
//! # Building p without cancellation
//!
//! Written as it stands, u - n^i e_i subtracts vectors whose components are of the size of the
//! observer's Lorentz factor gamma relative to any well-conditioned frame, and in the directions
//! where the light is blueshifted most relative to that frame the answer is of size 1 / gamma: the
//! sum loses (u^t)^2 1e-16 of its relative precision. For most observers that is nothing. For one
//! freezing onto the right branch of r- - E - Omega_- L < 0, followed by the app to u^t = 1e10 - it
//! is everything, and it is worst exactly where the far sky crowds, in the direction of motion.
//! (`tests/precision.rs` measures it: see the report there.)
//!
//! So [`Triad::look`] builds p in the frame of the raindrop (E = 1, L = 0) at the same event, which
//! exists at every radius, falls through both horizons, and has a u^t of order one near r-. The
//! observer's 4-velocity in that frame is u = gamma (e0 + beta dir . e), read off by three dot
//! products of u with legs of order one, each the size of its answer. 1 - beta is taken from the
//! normalisation, 1 - beta = 1 / (gamma (gamma + gamma beta)), never as a difference. The observer's
//! frame is then the pure boost of the raindrop's by beta along dir, followed by a turn about z
//! that puts x where the triad's x is: the rest-space direction of d/dr in the observer's frame,
//! whose component along the boost gamma (S_par - beta S0) is computed as
//! gamma ((S_par - S0) + (1 - beta) S0), so that only numbers of order one are subtracted. Light
//! seen along n then has, in the raindrop's frame (Einstein 1905, section 7), frequency
//! gamma (1 - beta n_par) and direction of travel ((beta - n_par) / (1 - beta n_par),
//! -n_across / (gamma (1 - beta n_par))), with 1 - n_par = |n_across|^2 / (1 + n_par) wherever
//! n_par >= 0: every quantity a sum of numbers of one sign, or a difference that is small only
//! where its own size does not matter. p = nu_R (e0 + m^i e_i) with the raindrop's legs is then
//! exact to rounding at any boost. [`Triad::look_direct`] keeps the direct form for comparison, and
//! [`Triad::sky_direction`] inverts `look` by the same formulae.
//!
//! The triad's legs x, y and u are still given, from `Tetrad`, for anyone who wants them; they are
//! orthonormal to their own rounding. It is only their difference that cannot be taken.

use kerr_equatorial::tetrad::inner;
use kerr_equatorial::{GeodesicState, Tetrad};

use crate::metric::Kerr;

/// An observer on the equatorial plane: chart radius, chart azimuth and the 4-velocity's chart
/// components (u^t, u^r, u^phi), the numbers `kerr_equatorial::GeodesicState` and the app's save
/// files hold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observer {
    /// Chart radius r.
    pub r: f64,
    /// Chart azimuth phi.
    pub phi: f64,
    /// (u^t, u^r, u^phi).
    pub u: [f64; 3],
}

/// Why an observer cannot be traced for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScopeError {
    /// The observer is at or inside the inner horizon.
    InsideInnerHorizon {
        /// The observer's radius.
        r: f64,
        /// The inner horizon's.
        r_minus: f64,
    },
    /// The 4-velocity is not a unit future-directed timelike vector at that event.
    NotAFourVelocity {
        /// g(u, u), which should be -1.
        norm: f64,
        /// u^t, which should be positive.
        u_t: f64,
    },
    /// A number was not finite.
    NotFinite,
}

impl std::fmt::Display for ScopeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InsideInnerHorizon { r, r_minus } => write!(
                f,
                "the observer is at r = {r} M, at or inside the inner horizon r- = {r_minus} M: the \
                 view from inside the inner horizon includes light that came through the ring from \
                 negative r and from the inner horizon's other sheet, which needs fates this \
                 version does not have"
            ),
            Self::NotAFourVelocity { norm, u_t } => write!(
                f,
                "the observer's 4-velocity is not a unit future-directed timelike vector: g(u, u) = \
                 {norm}, u^t = {u_t}"
            ),
            Self::NotFinite => write!(f, "the observer's event or 4-velocity is not finite"),
        }
    }
}

impl std::error::Error for ScopeError {}

impl Observer {
    /// An observer, checked: finite, at r > r-, and with a unit future-directed timelike u.
    ///
    /// The unit check carries the rounding floor of g(u, u), which is ~1e-16 |u|^2: an observer
    /// whose u^t has run away (the app follows worldlines to u^t = 1e10 on the approach to the
    /// right branch of r-) is not refused for rounding.
    pub fn new(kerr: &Kerr, r: f64, phi: f64, u: [f64; 3]) -> Result<Self, ScopeError> {
        if !(r.is_finite() && phi.is_finite() && u.iter().all(|c| c.is_finite())) {
            return Err(ScopeError::NotFinite);
        }
        let r_minus = kerr.inner_horizon();
        if r <= r_minus || r <= 0.0 {
            return Err(ScopeError::InsideInnerHorizon { r, r_minus });
        }
        let norm = kerr.equatorial().norm(r, &u);
        let size = u.iter().fold(1.0f64, |m, c| m.max(c.abs()));
        if (norm + 1.0).abs() > 1e-8 + 1e-14 * size * size || u[0] <= 0.0 {
            return Err(ScopeError::NotAFourVelocity { norm, u_t: u[0] });
        }
        Ok(Self { r, phi, u })
    }

    /// The observer at rest with respect to the stars, u = (1 / sqrt(-g_tt), 0, 0): it exists
    /// outside the static limit, r > 2M on the plane, and nowhere else.
    pub fn stationary(kerr: &Kerr, r: f64, phi: f64) -> Result<Self, ScopeError> {
        let g_tt = kerr.equatorial().metric_components(r)[0][0];
        Self::new(kerr, r, phi, [1.0 / (-g_tt).sqrt(), 0.0, 0.0])
    }

    /// The Cartesian position of the event.
    pub fn position(&self, kerr: &Kerr) -> [f64; 3] {
        kerr.embed_equatorial(self.r, self.phi)
    }
}

/// An observer's orthonormal triad and 4-velocity in Cartesian components (t, x, y, z) at the
/// observer's event, and the same frame as a boost of a well-conditioned reference frame, which is
/// what light is built in. See the module documentation for both.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triad {
    /// The event, (x, y, z).
    pub position: [f64; 3],
    /// The 4-velocity u^mu.
    pub u: [f64; 4],
    /// The x leg: heading zero, the centre of the frame.
    pub x: [f64; 4],
    /// The y leg, z cross x: the observer's left.
    pub y: [f64; 4],
    /// The z leg: the pole of the sky.
    pub z: [f64; 4],
    reference: Reference,
}

/// The observer's frame as the pure boost of the raindrop's frame at the same event, followed by a
/// turn about z. See the module documentation.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Reference {
    /// The raindrop's e0, e1, e2 in Cartesian components (the z leg is d/dz for both).
    legs: [[f64; 4]; 3],
    /// Lorentz factor of the observer relative to the raindrop.
    gamma: f64,
    /// Speed relative to the raindrop.
    beta: f64,
    /// Celerity gamma beta relative to the raindrop: the length of the observer's velocity in
    /// the raindrop's (e1, e2).
    celerity: f64,
    /// A bound on the rounding error of `celerity` (see [`Triad::celerity`]).
    celerity_rounding: f64,
    /// 1 - beta, as 1 / (gamma (gamma + gamma beta)): never by subtraction.
    one_minus_beta: f64,
    /// Unit direction of the observer's velocity in the raindrop's (e1, e2).
    dir: [f64; 2],
    /// (cos, sin) of the angle from the boosted raindrop's e1 to the triad's x, about z.
    turn: [f64; 2],
}

impl Reference {
    /// Frame components (V0, V1, V2) of a chart vector v in the raindrop's frame: -g(v, e0),
    /// g(v, e1), g(v, e2). Well conditioned: the raindrop's legs are of order one.
    fn components(kerr: &Kerr, r: f64, tetrad: &Tetrad, v: &[f64; 3]) -> [f64; 3] {
        let eq = kerr.equatorial();
        [
            -inner(&eq, r, v, &tetrad.e0),
            inner(&eq, r, v, &tetrad.e1),
            inner(&eq, r, v, &tetrad.e2),
        ]
    }

    /// The reference frame for an observer, and the turn that puts the triad's x on the rest-space
    /// direction of the chart vector `s`, further turned by `heading`.
    fn new(kerr: &Kerr, observer: &Observer, s: &[f64; 3], heading: f64) -> Self {
        let eq = kerr.equatorial();
        let r = observer.r;
        let rain_u = GeodesicState::new_infall(&eq, 0.0, r, 1.0, 0.0).u;
        let rain = Tetrad::from_four_velocity(&eq, r, &rain_u);
        // The observer's velocity in the raindrop's frame: u = gamma (e0 + beta dir . e). The
        // three components are dot products of u with legs of order one, each of the size of the
        // answer, so they are exact to rounding however large u is.
        let c = Self::components(kerr, r, &rain, &observer.u);
        let gamma = c[0];
        let speed = c[1].hypot(c[2]);
        // Each of c1 and c2 is a sum of nine products g_{mu nu} u^mu e^nu, formed from numbers
        // that each carry a rounding of their own: the metric's components, u, and legs built
        // from the raindrop's closed form. Its error is a few units of rounding of the sum of the
        // products' sizes, whatever the size of the answer; 8 of them bounds it with room (the
        // test below measures the raindrop against itself, where the answer is the error).
        let g = eq.metric_components(r);
        let size = |e: &[f64; 3]| {
            let mut sum = 0.0;
            for (i, row) in g.iter().enumerate() {
                for (j, g_ij) in row.iter().enumerate() {
                    sum += (g_ij * observer.u[i] * e[j]).abs();
                }
            }
            sum
        };
        let celerity_rounding = 8.0 * f64::EPSILON * (size(&rain.e1) + size(&rain.e2));
        let (dir, beta, one_minus_beta) = if speed > 0.0 {
            // gamma^2 - (gamma beta)^2 = 1 for a unit u, so 1 - beta = 1 / (gamma (gamma + gamma beta)).
            (
                [c[1] / speed, c[2] / speed],
                speed / gamma,
                1.0 / (gamma * (gamma + speed)),
            )
        } else {
            ([1.0, 0.0], 0.0, 1.0)
        };
        // The rest-space direction of s in the observer's frame, by the inverse boost of s's
        // components in the raindrop's: along dir, gamma (S_par - beta S0), written as
        // gamma ((S_par - S0) + (1 - beta) S0) so that no two numbers of size gamma meet; across
        // dir, unchanged.
        let sc = Self::components(kerr, r, &rain, s);
        let par = sc[1] * dir[0] + sc[2] * dir[1];
        let perp = -sc[1] * dir[1] + sc[2] * dir[0];
        let boosted_par = gamma * ((par - sc[0]) + one_minus_beta * sc[0]);
        let v = [
            boosted_par * dir[0] - perp * dir[1],
            boosted_par * dir[1] + perp * dir[0],
        ];
        let len = v[0].hypot(v[1]);
        let (c0, s0) = if len > 0.0 {
            (v[0] / len, v[1] / len)
        } else {
            (1.0, 0.0)
        };
        let (sh, ch) = heading.sin_cos();
        let turn = [c0 * ch - s0 * sh, s0 * ch + c0 * sh];
        let phi = observer.phi;
        Self {
            legs: [
                kerr.chart_vector(r, phi, rain.e0),
                kerr.chart_vector(r, phi, rain.e1),
                kerr.chart_vector(r, phi, rain.e2),
            ],
            gamma,
            beta,
            celerity: speed,
            celerity_rounding,
            one_minus_beta,
            dir,
            turn,
        }
    }

    /// The triad direction n in the orthonormal basis (dir, dir_perp, z) of the boosted raindrop's
    /// rest space, dir_perp = (-dir_1, dir_0): (n_par, n_perp, n_z).
    ///
    /// The basis matters. Light seen just off the direction of motion has its across-the-boost
    /// part multiplied by 1 / (gamma (1 - beta n_par)) on the way into the raindrop's frame - 2e8 at
    /// u^t = 1e8 - so that part must have no component along dir at all. Computed as
    /// n - n_par dir it keeps a residual of 1e-16 along dir, which that factor made into an error of
    /// 4e-8 in the light's direction (and its null condition) before this was written so. In the
    /// basis (dir, dir_perp) the two are orthogonal bit for bit: dir . dir_perp = -d0 d1 + d1 d0 = 0
    /// exactly in floating point.
    fn split(&self, n: [f64; 3]) -> [f64; 3] {
        let [c, s] = self.turn;
        let m = [c * n[0] - s * n[1], s * n[0] + c * n[1]];
        let [d0, d1] = self.dir;
        [m[0] * d0 + m[1] * d1, -m[0] * d1 + m[1] * d0, n[2]]
    }

    /// A vector with components (along, perp, z) in (dir, dir_perp, z), in the reference's
    /// (e1, e2, z).
    fn unsplit(&self, v: [f64; 3]) -> [f64; 3] {
        let [d0, d1] = self.dir;
        [v[0] * d0 - v[1] * d1, v[0] * d1 + v[1] * d0, v[2]]
    }
}

impl Triad {
    /// The triad with z along the spin axis and x turned from the observer's radial leg (the
    /// rest-space part of the chart's d/dr) by `heading` toward the prograde leg. Heading 0 is
    /// outward and pi inward only as far as that leg is radial: see the module's note on headings,
    /// and use [`Triad::towards`] with minus the gradient of r to face the hole.
    pub fn new(kerr: &Kerr, observer: &Observer, heading: f64) -> Self {
        let (s, c) = heading.sin_cos();
        let tetrad = Self::tetrad(kerr, observer).turned(c, s);
        let reference = Reference::new(kerr, observer, &[0.0, 1.0, 0.0], heading);
        Self::from_tetrad(kerr, observer, &tetrad, reference)
    }

    /// The triad with z along the spin axis and x along the rest-space part of the vector with
    /// chart components `s` = (s^t, s^r, s^phi) at the observer's event: the observer's motion
    /// relative to some other frame, say, or a carried gyroscope. A vector with no rest-space part
    /// leaves x outward.
    pub fn towards(kerr: &Kerr, observer: &Observer, s: [f64; 3]) -> Self {
        let eq = kerr.equatorial();
        let tetrad = Self::tetrad(kerr, observer).turned_towards(&eq, observer.r, &s);
        let reference = Reference::new(kerr, observer, &s, 0.0);
        Self::from_tetrad(kerr, observer, &tetrad, reference)
    }

    fn tetrad(kerr: &Kerr, observer: &Observer) -> Tetrad {
        Tetrad::from_four_velocity(&kerr.equatorial(), observer.r, &observer.u)
    }

    fn from_tetrad(
        kerr: &Kerr,
        observer: &Observer,
        tetrad: &Tetrad,
        reference: Reference,
    ) -> Self {
        let (r, phi) = (observer.r, observer.phi);
        Self {
            position: observer.position(kerr),
            u: kerr.chart_vector(r, phi, tetrad.e0),
            x: kerr.chart_vector(r, phi, tetrad.e1),
            y: kerr.chart_vector(r, phi, tetrad.e2),
            z: [0.0, 0.0, 0.0, 1.0],
            reference,
        }
    }

    /// The future-directed 4-momentum p^mu of the light seen when looking along n (a unit vector
    /// in the triad), normalised to unit frequency in the observer's frame, so that -p . u = 1.
    ///
    /// Built in the raindrop's frame and carried to the observer's by the closed forms of
    /// aberration and the Doppler shift (module documentation): exact to rounding at any boost.
    pub fn look(&self, n: [f64; 3]) -> [f64; 4] {
        let f = &self.reference;
        let [par, perp, nz] = f.split(n);
        let across2 = perp * perp + nz * nz;
        // 1 - beta n_par and beta - n_par, with 1 - n_par = |n_across|^2 / (1 + n_par) where n_par
        // is near 1, so that neither is a difference of two numbers near 1.
        let (one_minus_bn, beta_minus_n) = if par >= 0.0 {
            let one_minus_n = across2 / (1.0 + par);
            (
                f.one_minus_beta + f.beta * one_minus_n,
                one_minus_n - f.one_minus_beta,
            )
        } else {
            (1.0 - f.beta * par, f.beta - par)
        };
        let nu = f.gamma * one_minus_bn;
        // Direction of travel in the raindrop's frame: along the boost (beta - n_par) / (1 - beta
        // n_par), across it -n_across / (gamma (1 - beta n_par)).
        let k = -1.0 / (f.gamma * one_minus_bn);
        let m = f.unsplit([beta_minus_n / one_minus_bn, k * perp, k * nz]);
        let [e0, e1, e2] = f.legs;
        core::array::from_fn(|mu| {
            nu * (e0[mu] + m[0] * e1[mu] + m[1] * e2[mu] + if mu == 3 { m[2] } else { 0.0 })
        })
    }

    /// The same momentum by the direct construction u - n^i e_i from the triad's legs. It is exact
    /// for an observer of modest u^t and loses (u^t)^2 1e-16 of its relative precision in the
    /// directions where the light is blueshifted most relative to the raindrop; kept as the
    /// comparison the tests and the precision measurement make.
    pub fn look_direct(&self, n: [f64; 3]) -> [f64; 4] {
        core::array::from_fn(|mu| {
            self.u[mu] - n[0] * self.x[mu] - n[1] * self.y[mu] - n[2] * self.z[mu]
        })
    }

    /// Where in the triad the observer sees a null vector p (contravariant, Cartesian), and at
    /// what frequency -p . u: the inverse of [`Triad::look`], by the same closed forms, so that it
    /// can say what direction a momentum built some other way really points in, at any boost.
    pub fn sky_direction(&self, kerr: &Kerr, p: [f64; 4]) -> ([f64; 3], f64) {
        let f = &self.reference;
        let pos = self.position;
        let [e0, e1, e2] = f.legs;
        let z = [0.0, 0.0, 0.0, 1.0];
        let frame = [
            -kerr.dot(pos, p, e0),
            kerr.dot(pos, p, e1),
            kerr.dot(pos, p, e2),
            kerr.dot(pos, p, z),
        ];
        // Direction of travel in the raindrop's frame, in (dir, dir_perp, z).
        let len = (frame[1] * frame[1] + frame[2] * frame[2] + frame[3] * frame[3]).sqrt();
        let [d0, d1] = f.dir;
        let (m1, m2, mz) = (frame[1] / len, frame[2] / len, frame[3] / len);
        let par = m1 * d0 + m2 * d1;
        let perp = -m1 * d1 + m2 * d0;
        let across2 = perp * perp + mz * mz;
        // The observer runs at beta along dir: frequency gamma nu_R (1 - beta m_par), and direction
        // (m_par - beta) / (1 - beta m_par) along, m_across / (gamma (1 - beta m_par)) across.
        let (one_minus_bm, m_minus_beta) = if par >= 0.0 {
            let one_minus_m = across2 / (1.0 + par);
            (
                f.one_minus_beta + f.beta * one_minus_m,
                f.one_minus_beta - one_minus_m,
            )
        } else {
            (1.0 - f.beta * par, par - f.beta)
        };
        let nu = f.gamma * len * one_minus_bm;
        let k = 1.0 / (f.gamma * one_minus_bm);
        // Travel direction in (dir, dir_perp, z) of the boosted raindrop, then in its (e1', e2', z),
        // then in the triad; the observer looks the other way.
        let t = f.unsplit([m_minus_beta / one_minus_bm, k * perp, k * mz]);
        let [c, s] = f.turn;
        ([-(c * t[0] + s * t[1]), -(-s * t[0] + c * t[1]), -t[2]], nu)
    }

    /// The observer's Lorentz factor relative to the raindrop at the same event, and 1 - beta.
    pub fn boost(&self) -> (f64, f64) {
        (self.reference.gamma, self.reference.one_minus_beta)
    }

    /// The observer's celerity gamma beta relative to the raindrop at the same event, and a bound
    /// on its rounding error.
    ///
    /// The celerity is the length of the observer's velocity components (c1, c2) in the
    /// raindrop's frame, each a dot product of u with a leg of order one, so its error does not
    /// grow with the boost relative to its size. But it is an absolute error: an observer who is
    /// the raindrop, or all but, gets a celerity of a few roundings pointing anywhere. Where the
    /// celerity is not larger than the bound, the observer is at rest relative to the raindrop as
    /// far as f64 can tell, and [`Triad::forward`] names no direction; where it is, `forward` is
    /// good to about bound / celerity radians.
    pub fn celerity(&self) -> (f64, f64) {
        (self.reference.celerity, self.reference.celerity_rounding)
    }

    /// The direction of the observer's motion relative to the raindrop, in the triad: where the
    /// sky crowds when the boost is large.
    ///
    /// It is also the direction in which the observer sees itself travel past the raindrop. The
    /// observer's frame is the raindrop's boosted by beta along dir, and in the boosted frame the
    /// raindrop moves at beta along -dir, the boosted legs' own components; so the observer moves
    /// past it along +dir of the boosted legs, which the turn carries into the triad. There is no
    /// cancellation in it at any boost: dir is two components of order one over their length,
    /// and the turn is exact. It means nothing where [`Triad::celerity`] says the observer is at
    /// rest relative to the raindrop.
    pub fn forward(&self) -> [f64; 3] {
        let f = &self.reference;
        let [c, s] = f.turn;
        [
            c * f.dir[0] + s * f.dir[1],
            -s * f.dir[0] + c * f.dir[1],
            0.0,
        ]
    }

    /// Whether the sky of this triad is symmetric under n_z -> -n_z: the observer is on the
    /// plane, the z leg is exactly d/dz and nothing else has a z component. The metric is
    /// symmetric under z -> -z (H depends on z^2, and l_mu dx^mu is even), so the ray seen at
    /// (n_x, n_y, -n_z) is the mirror image of the one seen at (n_x, n_y, n_z).
    pub fn is_reflection_symmetric(&self) -> bool {
        self.position[2] == 0.0
            && self.z == [0.0, 0.0, 0.0, 1.0]
            && self.u[3] == 0.0
            && self.x[3] == 0.0
            && self.y[3] == 0.0
            && self.reference.legs.iter().all(|leg| leg[3] == 0.0)
    }
}

#[cfg(test)]
// Tensor components are indexed by their indices, as the formulae write them.
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;

    /// Named 4-velocities at radius r, each where it exists.
    fn observers(kerr: &Kerr, r: f64) -> Vec<(&'static str, [f64; 3])> {
        let eq = kerr.equatorial();
        let g = eq.metric_components(r);
        let mut out = Vec::new();
        if r > 2.0 * kerr.m {
            out.push(("static", [1.0 / (-g[0][0]).sqrt(), 0.0, 0.0]));
        }
        if r > kerr.outer_horizon() {
            let omega = eq.frame_dragging_omega(r);
            let n = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
            out.push(("ZAMO", [1.0 / n.sqrt(), 0.0, omega / n.sqrt()]));
        }
        for prograde in [true, false] {
            if let (Some(omega), Some(ut)) = (
                eq.orbital_angular_velocity(r, prograde),
                eq.circular_orbit_dilation(r, prograde),
            ) {
                out.push((
                    if prograde {
                        "prograde orbit"
                    } else {
                        "retrograde orbit"
                    },
                    [ut, 0.0, ut * omega],
                ));
            }
        }
        out.push((
            "raindrop",
            GeodesicState::new_infall(&eq, 0.0, r, 1.0, 0.0).u,
        ));
        out.push((
            "infall E = 1.2, L = 3",
            GeodesicState::new_infall(&eq, 0.0, r, 1.2, 3.0).u,
        ));
        out
    }

    fn det4(m: [[f64; 4]; 4]) -> f64 {
        let mut a = m;
        let mut det = 1.0;
        for col in 0..4 {
            let pivot = (col..4)
                .max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))
                .unwrap();
            if pivot != col {
                a.swap(pivot, col);
                det = -det;
            }
            det *= a[col][col];
            for row in col + 1..4 {
                let f = a[row][col] / a[col][col];
                for k in col..4 {
                    a[row][k] -= f * a[col][k];
                }
            }
        }
        det
    }

    #[test]
    fn test_the_z_leg_is_unit_and_orthogonal_to_the_plane() {
        // g(d/dz, d/dz) = 1 and g(d/dz, v) = 0 for d/dt, d/dx, d/dy and for u, on z = 0 at every
        // radius the observer can have, including between the horizons.
        for &a in &[0.0, 0.65, 0.95] {
            let kerr = Kerr::new(1.0, a);
            for &r in &[
                20.0,
                3.0,
                1.2 * kerr.outer_horizon(),
                0.5 * (kerr.outer_horizon() + kerr.inner_horizon()),
            ] {
                let p = kerr.embed_equatorial(r, 0.8);
                let dz = [0.0, 0.0, 0.0, 1.0];
                assert!((kerr.dot(p, dz, dz) - 1.0).abs() < 1e-15);
                for v in [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                ] {
                    assert!(kerr.dot(p, dz, v).abs() < 1e-15, "g(d_z, {v:?}) at r = {r}");
                }
                for (name, u) in observers(&kerr, r) {
                    let obs = Observer::new(&kerr, r, 0.8, u).unwrap();
                    let t = Triad::new(&kerr, &obs, 0.0);
                    assert!(
                        kerr.dot(p, t.z, t.u).abs() < 1e-15,
                        "{name}: g(z, u) at r = {r}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_triad_is_orthonormal_and_right_handed_for_every_kind_of_observer() {
        // Static, zero-angular-momentum, circular-orbit and infalling observers, outside r+ and
        // between the horizons, at several headings: g(e_a, e_b) = diag(-1, 1, 1, 1) with
        // e = (u, x, y, z), and det[u, x, y, z] = +1 (orthonormal, future-directed, right-handed,
        // since det g = -1). Also x cross y = z in the frame's own terms: the specification's
        // "y points to the observer's left" is this.
        let mut count = 0;
        for &a in &[0.0, 0.65, 0.9] {
            let kerr = Kerr::new(1.0, a);
            let (rp, rm) = (kerr.outer_horizon(), kerr.inner_horizon());
            for &r in &[12.0, 6.0, 3.0, 1.1 * rp, 0.5 * (rp + rm)] {
                for (name, u) in observers(&kerr, r) {
                    for &heading in &[0.0, 1.0, std::f64::consts::PI, -2.2] {
                        let obs = Observer::new(&kerr, r, 1.7, u).unwrap();
                        let t = Triad::new(&kerr, &obs, heading);
                        let legs = [t.u, t.x, t.y, t.z];
                        let size = t.u.iter().fold(1.0f64, |m, c| m.max(c.abs()));
                        for i in 0..4 {
                            for j in 0..4 {
                                let want = if i != j {
                                    0.0
                                } else if i == 0 {
                                    -1.0
                                } else {
                                    1.0
                                };
                                let got = kerr.dot(t.position, legs[i], legs[j]);
                                assert!(
                                    (got - want).abs() < 1e-12 * size * size,
                                    "{name} at r = {r}, a = {a}: g(e{i}, e{j}) = {got}"
                                );
                            }
                        }
                        let det = det4(legs);
                        assert!(
                            (det - 1.0).abs() < 1e-10 * size,
                            "{name} at r = {r}: det = {det}"
                        );
                        count += 1;
                    }
                }
            }
        }
        println!("{count} triads orthonormal and right-handed");
    }

    #[test]
    fn test_heading_zero_faces_outward_and_pi_faces_the_hole() {
        // For a static observer the x leg at heading 0 is the outward coordinate direction d/dr
        // normalised, which in Cartesian components is (cos phi, sin phi) (not the position's
        // own direction, which differs by atan2(a, r)); heading pi reverses it, and heading pi/2
        // is the prograde leg.
        let kerr = Kerr::new(1.0, 0.9);
        let obs = Observer::stationary(&kerr, 6.0, 0.6).unwrap();
        let out = Triad::new(&kerr, &obs, 0.0);
        let hole = Triad::new(&kerr, &obs, std::f64::consts::PI);
        let spatial = |v: [f64; 4]| {
            let n = (v[1] * v[1] + v[2] * v[2] + v[3] * v[3]).sqrt();
            [v[1] / n, v[2] / n, v[3] / n]
        };
        let (s, c) = 0.6f64.sin_cos();
        let o = spatial(out.x);
        assert!(
            (o[0] - c).abs() < 1e-12 && (o[1] - s).abs() < 1e-12 && o[2] == 0.0,
            "{o:?}"
        );
        for mu in 0..4 {
            assert!((hole.x[mu] + out.x[mu]).abs() < 1e-12);
            assert!((hole.y[mu] + out.y[mu]).abs() < 1e-12);
        }
        let pro = Triad::new(&kerr, &obs, std::f64::consts::FRAC_PI_2);
        for mu in 0..4 {
            assert!((pro.x[mu] - out.y[mu]).abs() < 1e-12);
        }
        // `towards` with the chart vector d/dr is heading 0.
        let t = Triad::towards(&kerr, &obs, [0.0, 1.0, 0.0]);
        for mu in 0..4 {
            assert!((t.x[mu] - out.x[mu]).abs() < 1e-12);
        }
    }

    #[test]
    fn test_an_observer_at_or_inside_the_inner_horizon_is_refused_with_a_sentence() {
        let kerr = Kerr::new(1.0, 0.9);
        let rm = kerr.inner_horizon();
        let eq = kerr.equatorial();
        for r in [rm, 0.5 * rm] {
            let u = GeodesicState::new_infall(&eq, 0.0, r, 1.0, 0.0).u;
            let err = Observer::new(&kerr, r, 0.0, u).unwrap_err();
            assert!(matches!(err, ScopeError::InsideInnerHorizon { .. }));
            let sentence = err.to_string();
            assert!(
                sentence.contains("inside the inner horizon") && sentence.contains("fates"),
                "{sentence}"
            );
        }
        let r = 1.01 * rm;
        let u = GeodesicState::new_infall(&eq, 0.0, r, 1.0, 0.0).u;
        assert!(
            Observer::new(&kerr, r, 0.0, u).is_ok(),
            "just above r- is in scope"
        );
        assert!(matches!(
            Observer::new(&kerr, 6.0, 0.0, [1.0, 0.0, 0.0]),
            Err(ScopeError::NotAFourVelocity { .. })
        ));
    }

    #[test]
    fn test_the_conditioned_and_direct_momenta_agree_where_the_direct_one_is_exact() {
        // For observers whose u^t is of order one or ten the direct u - n^i e_i has nothing to
        // lose, so the boosted-raindrop construction must reproduce it to rounding, in every
        // direction, for every kind of observer, at several headings and with `towards`. The
        // comparison is component by component against the size of p, and the inverse
        // `sky_direction` must hand back the n and the unit frequency that went in.
        let mut worst = 0.0f64;
        let mut worst_back = 0.0f64;
        for &a in &[0.0, 0.65, 0.9] {
            let kerr = Kerr::new(1.0, a);
            let (rp, rm) = (kerr.outer_horizon(), kerr.inner_horizon());
            for &r in &[12.0, 3.0, 1.1 * rp, 0.5 * (rp + rm)] {
                for (name, u) in observers(&kerr, r) {
                    let obs = Observer::new(&kerr, r, 0.9, u).unwrap();
                    let triads = [
                        Triad::new(&kerr, &obs, 0.0),
                        Triad::new(&kerr, &obs, 2.5),
                        Triad::towards(&kerr, &obs, [0.3, -0.2, 0.05]),
                    ];
                    for t in triads {
                        for n in [
                            [1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0],
                            [0.36, 0.48, 0.8],
                            [-0.6, 0.0, -0.8],
                        ] {
                            let (b, d) = (t.look(n), t.look_direct(n));
                            let size = d.iter().fold(0.0f64, |m, c| m.max(c.abs()));
                            for mu in 0..4 {
                                worst = worst.max((b[mu] - d[mu]).abs() / size);
                            }
                            let (back, nu) = t.sky_direction(&kerr, b);
                            let miss = (0..3).map(|i| (back[i] - n[i]).abs()).fold(0.0, f64::max);
                            worst_back = worst_back.max(miss).max((nu - 1.0).abs());
                            assert!(
                                worst < 1e-12 && worst_back < 1e-12,
                                "{name} at r = {r}, a = {a}: {worst}, {worst_back}"
                            );
                        }
                    }
                }
            }
        }
        println!(
            "conditioned against direct: {worst:.1e}; sky_direction round trip {worst_back:.1e}"
        );
    }

    #[test]
    fn test_the_raindrop_is_at_rest_relative_to_itself_to_within_the_rounding_bound() {
        // The raindrop's own 4-velocity, handed in as an observer's, must come out with a
        // celerity no larger than the bound `celerity` states, at every radius and spin, through
        // both horizons and down to just outside r-: the answer there is exactly zero, so what is
        // measured is the rounding itself. And an observer who does move relative to the raindrop,
        // by the other observers of this module, must come out far above it.
        let mut worst = 0.0f64;
        let mut moving = f64::INFINITY;
        for &a in &[0.0, 0.5, 0.9, 0.998] {
            let kerr = Kerr::new(1.0, a);
            let eq = kerr.equatorial();
            let (rp, rm) = (kerr.outer_horizon(), kerr.inner_horizon());
            let mut radii = vec![40.0, 6.0, 2.0, 1.0 + 1e-9, rp + 1e-9, rp, rp - 1e-9];
            radii.extend([0.5 * (rp + rm), 1.001 * rm, rm + 1e-9, 0.3]);
            for r in radii {
                // Outside r- and off the ring: at a = 0, r- is 0 and "just outside r-" is the
                // singularity itself.
                if r <= rm || r < 0.05 {
                    continue;
                }
                let rain = GeodesicState::new_infall(&eq, 0.0, r, 1.0, 0.0).u;
                let obs = Observer::new(&kerr, r, 0.4, rain).unwrap();
                let t = Triad::new(&kerr, &obs, 0.0);
                let (celerity, rounding) = t.celerity();
                assert!(
                    celerity <= rounding,
                    "the raindrop at r = {r}, a = {a} moves past itself at {celerity:e}, above \
                     the bound {rounding:e}"
                );
                worst = worst.max(celerity / rounding);
                for (name, u) in observers(&kerr, r) {
                    // The helper's closed forms are not all good 4-velocities at every radius of
                    // this list (an orbit a whisker outside its photon orbit is not); those are
                    // not what this test is about.
                    let Ok(obs) = Observer::new(&kerr, r, 0.4, u) else {
                        continue;
                    };
                    if name == "raindrop" {
                        continue;
                    }
                    let t = Triad::new(&kerr, &obs, 0.0);
                    let (celerity, rounding) = t.celerity();
                    moving = moving.min(celerity / rounding);
                }
            }
        }
        println!(
            "the raindrop's celerity past itself reaches {worst:.3} of the bound; the other \
             observers' is at least {moving:.1e} times it"
        );
        assert!(moving > 1e6, "{moving}");
    }

    #[test]
    fn test_looking_along_n_gives_a_null_future_directed_momentum_of_unit_frequency() {
        let kerr = Kerr::new(1.0, 0.9);
        let eq = kerr.equatorial();
        let obs = Observer::new(
            &kerr,
            1.0,
            0.3,
            GeodesicState::new_infall(&eq, 0.0, 1.0, 1.0, 2.0).u,
        )
        .unwrap();
        let t = Triad::new(&kerr, &obs, 0.4);
        for n in [[1.0, 0.0, 0.0], [0.0, 0.6, 0.8], [-0.36, 0.48, -0.8]] {
            let p = t.look(n);
            assert!(kerr.dot(t.position, p, p).abs() < 1e-12);
            assert!((kerr.dot(t.position, p, t.u) + 1.0).abs() < 1e-12);
            // The light travels along -n in the observer's frame.
            for (i, leg) in [t.x, t.y, t.z].into_iter().enumerate() {
                assert!((kerr.dot(t.position, p, leg) + n[i]).abs() < 1e-12);
            }
            assert!(p[0] > 0.0, "future-directed: p^t = {}", p[0]);
        }
    }
}
