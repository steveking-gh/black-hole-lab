//! Double-double arithmetic, and the direction of travel evaluated in it by a route of its own:
//! the independent evaluation the tests hold the program's directions to.
//!
//! A double-double is an unevaluated sum hi + lo of two f64 with |lo| <= ulp(hi) / 2, good to about
//! 32 significant digits. The algorithms are the standard error-free ones (Dekker 1971; Knuth's
//! two-sum; Hida, Li and Bailey's QD library): each product and sum is split exactly into its
//! rounded value and its rounding error by `f64::mul_add` and Knuth's two-sum, and the errors are
//! carried in `lo`.
//!
//! The route is the literal one, which is what f64 cannot do at a large boost: the reference
//! observer's u_F and the observer's u, gamma = -g(u, u_F), w = u_F - gamma u, the triad's x as
//! minus the rest-space part of nabla r normalised, its y as the unit vector of the rest space
//! orthogonal to x with det[u, x, y] > 0 in the chart's (t, r, phi) (which, with z, is
//! right-handed: the embedding x + iy = (r + ia) e^{i phi} has Jacobian determinant r > 0), and
//! n = -w projected on x and y. It is carried out in an orthonormal frame of its own, the
//! raindrop's built by Gram-Schmidt in double-double from d_r and d_phi. None of the closed forms
//! of `travel` is used, nor anything of `kerr_sky`'s triad or `kerr_equatorial`'s tetrad. A
//! cancellation of gamma^2 = 1e18 leaves 1e-14 of 32 digits.

use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dd {
    pub hi: f64,
    pub lo: f64,
}

fn two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    let v = s - a;
    Dd {
        hi: s,
        lo: (a - (s - v)) + (b - v),
    }
}

fn quick_two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    Dd {
        hi: s,
        lo: b - (s - a),
    }
}

fn two_prod(a: f64, b: f64) -> Dd {
    let p = a * b;
    Dd {
        hi: p,
        lo: a.mul_add(b, -p),
    }
}

impl Dd {
    pub const ZERO: Self = Self { hi: 0.0, lo: 0.0 };
    pub const ONE: Self = Self { hi: 1.0, lo: 0.0 };

    pub fn new(x: f64) -> Self {
        Self { hi: x, lo: 0.0 }
    }

    pub fn to_f64(self) -> f64 {
        self.hi + self.lo
    }

    pub fn sqrt(self) -> Self {
        if self.hi <= 0.0 {
            assert!(self.hi == 0.0, "the square root of {self:?}");
            return Self::ZERO;
        }
        // Two Newton steps from the f64 root: each doubles the digits.
        let mut y = Self::new(self.hi.sqrt());
        for _ in 0..2 {
            y = y + (self - y * y) / (y * Self::new(2.0));
        }
        y
    }
}

impl From<f64> for Dd {
    fn from(x: f64) -> Self {
        Self::new(x)
    }
}

impl Add for Dd {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        let s = two_sum(self.hi, o.hi);
        let t = two_sum(self.lo, o.lo);
        let s = quick_two_sum(s.hi, s.lo + t.hi);
        quick_two_sum(s.hi, s.lo + t.lo)
    }
}

impl Neg for Dd {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            hi: -self.hi,
            lo: -self.lo,
        }
    }
}

impl Sub for Dd {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        self + (-o)
    }
}

impl Mul for Dd {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let p = two_prod(self.hi, o.hi);
        quick_two_sum(p.hi, p.lo + (self.hi * o.lo + self.lo * o.hi))
    }
}

impl Div for Dd {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let q1 = self.hi / o.hi;
        let r = self - o * Self::new(q1);
        let q2 = r.hi / o.hi;
        let r = r - o * Self::new(q2);
        let q3 = r.hi / o.hi;
        quick_two_sum(q1, q2) + Self::new(q3)
    }
}

pub type Vector = [Dd; 3];

/// The equatorial ingoing Kerr-Schild metric at r, in (t, r, phi), and its inverse, with h = M/r:
/// the components of `KerrSchild::metric_components` and `inverse_metric`, which define the chart.
pub struct Metric {
    pub m: Dd,
    pub a: Dd,
    pub r: Dd,
    pub g: [[Dd; 3]; 3],
    pub inverse: [[Dd; 3]; 3],
}

impl Metric {
    pub fn new(m: f64, a: f64, r: f64) -> Self {
        let (m, a, r) = (Dd::new(m), Dd::new(a), Dd::new(r));
        let one = Dd::ONE;
        let two = Dd::new(2.0);
        let h = m / r;
        let delta = r * r - two * m * r + a * a;
        let g_tt = -(one - two * h);
        let g_tr = two * h;
        let g_rr = one + two * h;
        let g_tphi = -(two * h * a);
        let g_rphi = -(a * (one + two * h));
        let g_phiphi = r * r + a * a + two * h * a * a;
        let inv_r2 = one / (r * r);
        Self {
            m,
            a,
            r,
            g: [
                [g_tt, g_tr, g_tphi],
                [g_tr, g_rr, g_rphi],
                [g_tphi, g_rphi, g_phiphi],
            ],
            inverse: [
                [-(one + two * h), two * h, Dd::ZERO],
                [two * h, delta * inv_r2, a * inv_r2],
                [Dd::ZERO, a * inv_r2, inv_r2],
            ],
        }
    }

    // Tensor components are indexed by their indices, as the formulae write them.
    #[allow(clippy::needless_range_loop)]
    pub fn dot(&self, x: &Vector, y: &Vector) -> Dd {
        let mut sum = Dd::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                sum = sum + self.g[i][j] * x[i] * y[j];
            }
        }
        sum
    }

    fn raise(&self, covector: &Vector) -> Vector {
        std::array::from_fn(|i| {
            (0..3).fold(Dd::ZERO, |sum, j| sum + self.inverse[i][j] * covector[j])
        })
    }

    fn unit(&self, v: &Vector) -> Vector {
        let length = self.dot(v, v).sqrt();
        v.map(|c| c / length)
    }

    /// The raindrop's 4-velocity: covariant u_t = -1, u_phi = 0, u^r = -s with
    /// s = sqrt(2M (r^2 + a^2) / r^3), and u_r = -(2h) / (2h + s), the regular form of
    /// (u^r - g^rt u_t) / g^rr (4h^2 - s^2 = -2M Delta / r^3 cancels the Delta).
    pub fn raindrop(&self) -> Vector {
        let (m, a, r) = (self.m, self.a, self.r);
        let two = Dd::new(2.0);
        let h = m / r;
        let s = (two * m * (r * r + a * a) / (r * r * r)).sqrt();
        let u_r = -(two * h) / (two * h + s);
        [(Dd::ONE + two * h) + two * h * u_r, -s, a / (r * r) * u_r]
    }

    /// The static observer, d_t / sqrt(-g_tt).
    pub fn static_observer(&self) -> Vector {
        [Dd::ONE / (-self.g[0][0]).sqrt(), Dd::ZERO, Dd::ZERO]
    }

    /// The ZAMO, (d_t + omega d_phi) / alpha with omega = -g_tphi / g_phiphi.
    pub fn zamo(&self) -> Vector {
        let omega = -self.g[0][2] / self.g[2][2];
        self.unit_timelike(&[Dd::ONE, Dd::ZERO, omega])
    }

    /// A future-directed timelike vector normalised to g(u, u) = -1.
    pub fn unit_timelike(&self, v: &Vector) -> Vector {
        let length = (-self.dot(v, v)).sqrt();
        v.map(|c| c / length)
    }

    /// The raindrop's frame (e0, e1, e2) by Gram-Schmidt from d_r and d_phi.
    pub fn raindrop_frame(&self) -> [Vector; 3] {
        let e0 = self.raindrop();
        let project = |v: Vector, legs: &[(Vector, Dd)]| {
            let mut out = v;
            for (leg, sign) in legs {
                let c = self.dot(&v, leg) * *sign;
                for mu in 0..3 {
                    out[mu] = out[mu] - c * leg[mu];
                }
            }
            out
        };
        let minus = -Dd::ONE;
        let e1 = self.unit(&project([Dd::ZERO, Dd::ONE, Dd::ZERO], &[(e0, minus)]));
        let e2 = self.unit(&project(
            [Dd::ZERO, Dd::ZERO, Dd::ONE],
            &[(e0, minus), (e1, Dd::ONE)],
        ));
        [e0, e1, e2]
    }

    /// The observer moving at Lorentz factor gamma relative to the raindrop, along the direction
    /// at angle chi from the raindrop's e1 toward its e2: exactly unit.
    pub fn boosted_raindrop(&self, gamma: f64, chi: f64) -> Vector {
        let [e0, e1, e2] = self.raindrop_frame();
        let gamma = Dd::new(gamma);
        let celerity = (gamma * gamma - Dd::ONE).sqrt();
        let (s, c) = chi.sin_cos();
        let length = (Dd::new(c) * Dd::new(c) + Dd::new(s) * Dd::new(s)).sqrt();
        let (c, s) = (Dd::new(c) / length, Dd::new(s) / length);
        std::array::from_fn(|mu| gamma * e0[mu] + celerity * (c * e1[mu] + s * e2[mu]))
    }

    /// The observer moving at celerity `celerity` relative to the ZAMO, at angle chi from the
    /// outward radial toward the prograde direction: exactly unit.
    pub fn boosted_zamo(&self, celerity: f64, chi: f64) -> Vector {
        let e0 = self.zamo();
        let radial = self.unit(&self.raise(&[Dd::ZERO, Dd::ONE, Dd::ZERO]));
        let azimuthal = self.unit(&[Dd::ZERO, Dd::ZERO, Dd::ONE]);
        let celerity = Dd::new(celerity);
        let gamma = (Dd::ONE + celerity * celerity).sqrt();
        let (s, c) = chi.sin_cos();
        let length = (Dd::new(c) * Dd::new(c) + Dd::new(s) * Dd::new(s)).sqrt();
        let (c, s) = (Dd::new(c) / length, Dd::new(s) / length);
        std::array::from_fn(|mu| gamma * e0[mu] + celerity * (c * radial[mu] + s * azimuthal[mu]))
    }

    /// The direction of travel past the reference observer `u_ref`, in the film's triad, and the
    /// celerity: the literal construction of the module documentation, carried out in the
    /// raindrop's orthonormal frame (`raindrop_frame`).
    ///
    /// Every vector is first read into that frame by dot products with its legs, which are of
    /// order one, so that nothing is lost there; the rest is Minkowski algebra, eta =
    /// diag(-1, 1, 1). Done in the chart instead, every product of two boosted vectors carries the
    /// chart's own factors as well, and at gamma = 1e9 even 32 digits did not hold the triad's y.
    ///
    /// n_x = g(-w, x) / |w| is a sum whose terms exceed it by gamma^2 at most, which leaves it good
    /// to 1e-14. n_y is taken as sqrt((1 - n_x)(1 + n_x)) with the sign of g(-w, y): the sign alone
    /// of a product that loses more, so that where it could be wrong, n_y is too small for it to
    /// matter.
    pub fn travel(&self, u: &Vector, u_ref: &Vector) -> ([f64; 3], f64) {
        let legs = self.raindrop_frame();
        let in_frame = |v: &Vector| -> Vector {
            [
                -self.dot(v, &legs[0]),
                self.dot(v, &legs[1]),
                self.dot(v, &legs[2]),
            ]
        };
        let dot = |a: &Vector, b: &Vector| -(a[0] * b[0]) + a[1] * b[1] + a[2] * b[2];
        let (uu, f) = (in_frame(u), in_frame(u_ref));
        // nabla r = g^{r mu}, whose products with the legs are their r components: g(nabla r, e)
        // = e^r.
        let grad = [-legs[0][1], legs[1][1], legs[2][1]];

        let gamma = -dot(&uu, &f);
        // -w = gamma u - u_F.
        let minus_w: Vector = std::array::from_fn(|a| gamma * uu[a] - f[a]);
        // At rest, -w is zero to the rounding of u, and its square may come out a hair below it.
        let square = dot(&minus_w, &minus_w);
        if square.hi <= 0.0 {
            return ([f64::NAN; 3], 0.0);
        }
        let celerity = square.sqrt();
        // x = -(nabla r + g(nabla r, u) u) / |...|.
        let along = dot(&grad, &uu);
        let p: Vector = std::array::from_fn(|a| grad[a] + along * uu[a]);
        let length = dot(&p, &p).sqrt();
        let x = p.map(|c| -c / length);
        // y: the covector epsilon_{abc} u^b x^c raised with eta, which is orthogonal to u and x
        // and makes det[u, x, y] = g(y, y) > 0. The raindrop's frame is positively oriented in
        // (t, r, phi): Gram-Schmidt from (u, d_r, d_phi) is triangular, det[e0, e1, e2] =
        // det[u, d_r, d_phi] / (n1 n2) = u^t / (n1 n2) > 0.
        let lowered = [
            uu[1] * x[2] - uu[2] * x[1],
            uu[2] * x[0] - uu[0] * x[2],
            uu[0] * x[1] - uu[1] * x[0],
        ];
        let y = [-lowered[0], lowered[1], lowered[2]];
        let n_x = dot(&minus_w, &x) / celerity;
        let across = ((Dd::ONE - n_x) * (Dd::ONE + n_x)).to_f64().max(0.0).sqrt();
        let n_y = if dot(&minus_w, &y).hi < 0.0 {
            -across
        } else {
            across
        };
        ([n_x.to_f64(), n_y, 0.0], celerity.to_f64())
    }
}

/// A vector of double-doubles rounded to f64, as the program is handed it.
pub fn rounded(v: &Vector) -> [f64; 3] {
    v.map(Dd::to_f64)
}

/// A vector of f64 as double-doubles.
pub fn exact(v: [f64; 3]) -> Vector {
    v.map(Dd::new)
}
