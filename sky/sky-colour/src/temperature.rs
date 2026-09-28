//! The temperature a colour implies.
//!
//! # The rule
//!
//! The implied temperature of a colour is the temperature of the point of the Planckian locus
//! nearest to it in the CIE 1960 (u, v) diagram, and its Duv is the signed distance to that
//! point, positive above the locus (toward green), negative below (toward magenta). That is the
//! CIE's definition of the correlated colour temperature (CIE 015:2018, section 9.5), chosen for
//! three reasons: it is the convention, so a number this crate prints can be checked against any
//! other program; it is exact for a blackbody (a colour on the locus is its own nearest point);
//! and the (u, v) diagram was made so that equal distances look roughly equally different, which
//! is what "nearest" should mean. The locus used is this crate's own, from the 1 nm CIE table and
//! the exact c2, not a published approximation, and the nearest point is found to the precision
//! of the arithmetic, not by the 31 isotherms of Robertson's 1968 method.
//!
//! Checked: the BT.709 white (0.3127, 0.3290) implies 6504.3 K with Duv +0.00321; the CIE's own
//! D65 spectrum implies 6502.7 K, which is what colour-science's Ohno (2013) method gives for it
//! (6502.68 K, Duv 0.003205). The nominal "6504 K" of D65 is 6500 x 1.4388 / 1.4380 and not a
//! computed value (see `planck`).
//!
//! # The search
//!
//! The locus is parameterised by tau = 1 / T from tau = 0 (T infinite, the Rayleigh-Jeans limit)
//! to 1 / 300 K, at the table's nodes plus 16 even steps of tau between 0 and 10^-6 K^-1. Along
//! it, s(tau) = (p - L(tau)) . L'(tau) = -(1/2) d|p - L|^2 / d tau is positive while the distance
//! is falling and negative once it rises. A binary search over the nodes finds the first node with
//! s < 0; the root of s between it and the one before is then found by Newton's method on the
//! cubic Hermite curve through the two nodes (see `Locus::nearest`). If s < 0 already
//! at tau = 0, the nearest point is the end at T infinite (a colour bluer than any blackbody); if
//! s > 0 at the last node, it is the end at 300 K.
//!
//! The binary search finds *a* local minimum of the distance, always (a change from + to -), and
//! the global one whenever s changes sign only once. When does it change more than once? The
//! locus bends one way only (its curvature has one sign, tested), toward the magentas below it.
//! Its radius of curvature in (u, v) is least, 0.1001, at 5235 K, and is larger everywhere else
//! (0.105 at 6800 K, 0.113 at 3700 K, 0.34 at 2000 K; nearly straight below 1000 K). The normals
//! to a curve cross first at its centres of curvature, so a colour closer to the locus than 0.1001
//! has one nearest point, and so does every colour above the locus, where the normals spread
//! apart. The centres of curvature from about 3500 K to 15000 K lie inside the BT.709 gamut, among
//! the magentas and purples: there, and beyond, a colour has two nearest points, one on each
//! side of 5235 K, and the nearest point jumps from one to the other across the line where they
//! are equally near. Tested on a grid of the whole gamut
//! (`test_the_nearest_point_is_unique_wherever_it_is_used`): every colour with more than one
//! change of sign is below the locus and at least 0.1001 from it on every branch.
//!
//! # Far from the locus
//!
//! The CIE states that a correlated colour temperature should not be used for a colour farther
//! than Duv 0.05 from the locus. There the idea "the colour of a blackbody" does not apply and any
//! temperature is a continuation, not an inference. The rule here keeps the output continuous
//! across the jump just described:
//!
//! - above the locus (Duv > 0: yellow-greens, greens, cyans) the nearest point is used at any
//!   distance, since it is unique there however far;
//! - below it, the nearest point is used down to Duv = -0.05 and is blended, in tau, with a
//!   continuation over Duv from -0.05 to -0.09 by a smoothstep, beyond which the continuation alone
//!   is used. -0.09 leaves a margin of 0.01 inside the least radius of curvature, so the nearest
//!   point is unique wherever it has any weight. The continuation is the temperature whose locus
//!   point has the same u as the colour (vertical isotherms in (u, v); u rises monotonically with
//!   tau along the whole locus, tested), clamped to the ends. It is continuous everywhere and
//!   meaningless as physics, which is why [`Implied::within_cct_domain`] reports whether a pixel is
//!   inside the CIE's limit.
//!
//! The awkward cases, each as the rule treats it:
//!
//! - *exactly grey* (R = G = B): nothing special. BT.709 white is 6504.3 K with Duv +0.0032, a
//!   little above the locus; its temperature is a CCT like any other, and the tint (the 0.0032)
//!   is carried through the shift unchanged, see `shift`. About a quarter of the map's faint Gaia
//!   stars are grey because they had no colour to draw (sky/maps/README.md); for them 6504 K is a
//!   default of the map, not a measurement, and nothing at pixel level can tell them from a star
//!   measured as white;
//! - *brown dust lanes*: brown is dark orange, whose chromaticity is near the locus at 2000 to
//!   3000 K; darkness does not enter (the implied temperature depends on chromaticity only);
//! - *black*: no chromaticity, no temperature: [`crate::Model::implied_temperature`] returns
//!   `None`, and the shift of black is black;
//! - *the primaries*: red (1, 0, 0) implies 986 K at Duv -0.0058, the only corner near the locus.
//!   Green implies 6064 K at Duv +0.099. Blue is beyond the locus's hot end, 0.158 from it, and
//!   magenta (1, 0, 1) 0.132 below it: both are far outside the CIE's limit.

use crate::colorimetry::uv;
use crate::table::{T_TABLE_MAX, Table};

/// The lowest temperature the search considers, K. The locus below it is within 5e-4 in (x, y)
/// of its limit, the colour of 830 nm, and no colour of the BT.709 gamut is nearer to it than to
/// the locus at 986 K.
pub const T_LOCUS_MIN: f64 = 300.0;

/// The CIE's limit of |Duv| beyond which a correlated colour temperature should not be used
/// (CIE 015:2018, section 9.5).
pub const CCT_DOMAIN_DUV: f64 = 0.05;

/// Below the locus, the Duv at which the continuation has fully replaced the nearest point.
pub const CONTINUATION_DUV: f64 = 0.09;

const TAIL_STEPS: usize = 16;

/// The temperature a colour implies, and how far the colour is from being a blackbody's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Implied {
    /// Kelvin; `f64::INFINITY` for a colour at or beyond the hot end of the locus.
    pub kelvin: f64,
    /// The signed distance from the locus in CIE 1960 (u, v), positive above it (toward green).
    pub duv: f64,
}

impl Implied {
    /// Whether the colour is within the CIE's limit |Duv| <= 0.05, where a correlated colour
    /// temperature means something. Outside it the temperature, and so the shifted colour, is a
    /// continuation and not an inference from the model.
    pub fn within_cct_domain(&self) -> bool {
        self.duv.abs() <= CCT_DOMAIN_DUV
    }
}

#[derive(Debug, Clone, Copy)]
struct Node {
    tau: f64,
    u: f64,
    v: f64,
    du: f64,
    dv: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct Locus {
    nodes: Vec<Node>,
}

/// (u, v) of tristimulus values x and its derivative, given the derivative dx of x.
#[inline]
fn uv_and_derivative(x: [f64; 3], dx: [f64; 3]) -> (f64, f64, f64, f64) {
    let d = x[0] + 15.0 * x[1] + 3.0 * x[2];
    let dd = dx[0] + 15.0 * dx[1] + 3.0 * dx[2];
    let u = 4.0 * x[0] / d;
    let v = 6.0 * x[1] / d;
    (u, v, (4.0 * dx[0] - u * dd) / d, (6.0 * dx[1] - v * dd) / d)
}

/// The locus point and its derivative with respect to tau = 1 / T.
#[inline]
fn locus_at(table: &Table, tau: f64) -> (f64, f64, f64, f64) {
    if tau <= 1.0 / T_TABLE_MAX {
        let (x, dx) = table.scaled_series(tau);
        return uv_and_derivative(x, dx);
    }
    // (u, v) does not change when X, Y, Z are multiplied by a common factor, so they are scaled
    // by 1 / Y to keep them near 1; d ln X / d tau = -slope / tau, and the derivative is taken
    // with the factor held fixed, which leaves du / d tau unchanged.
    let (f, slope) = table.ln_xyz_and_slope(1.0 / tau);
    let x: [f64; 3] = std::array::from_fn(|c| (f[c] - f[1]).exp());
    let dx: [f64; 3] = std::array::from_fn(|c| -x[c] * slope[c] / tau);
    uv_and_derivative(x, dx)
}

impl Locus {
    pub(crate) fn new(table: &Table) -> Self {
        let tau_hi = 1.0 / T_TABLE_MAX;
        let mut taus: Vec<f64> = (0..=TAIL_STEPS)
            .map(|j| tau_hi * j as f64 / TAIL_STEPS as f64)
            .collect();
        // Then the table's own spacing in ln T, from 10^6 K down to 300 K.
        let step = 1.0 / 128.0;
        let mut w = T_TABLE_MAX.ln() - step;
        while w > T_LOCUS_MIN.ln() {
            taus.push((-w).exp());
            w -= step;
        }
        taus.push(1.0 / T_LOCUS_MIN);
        let nodes = taus
            .into_iter()
            .map(|tau| {
                let (u, v, du, dv) = locus_at(table, tau);
                Node { tau, u, v, du, dv }
            })
            .collect();
        Self { nodes }
    }

    /// The implied temperature of a colour with chromaticity (u, v).
    pub(crate) fn implied(&self, p: (f64, f64)) -> Implied {
        let (tau_near, lu, lv, ldu, ldv) = self.nearest(p);
        let (ex, ey) = (p.0 - lu, p.1 - lv);
        let dist = ex.hypot(ey);
        // The normal (-dv, du) points up: along the locus u rises with tau, so du > 0.
        let duv = if -ldv * ex + ldu * ey >= 0.0 {
            dist
        } else {
            -dist
        };
        let tau = if duv < -CCT_DOMAIN_DUV {
            let s = ((-duv - CCT_DOMAIN_DUV) / (CONTINUATION_DUV - CCT_DOMAIN_DUV)).min(1.0);
            let w = s * s * (3.0 - 2.0 * s);
            (1.0 - w) * tau_near + w * self.continuation(p.0)
        } else {
            tau_near
        };
        Implied {
            kelvin: 1.0 / tau,
            duv,
        }
    }

    /// tau of the nearest point, with the locus point and derivative there.
    fn nearest(&self, p: (f64, f64)) -> (f64, f64, f64, f64, f64) {
        let s_of = |n: &Node| (p.0 - n.u) * n.du + (p.1 - n.v) * n.dv;
        let first = &self.nodes[0];
        if s_of(first) <= 0.0 {
            return (first.tau, first.u, first.v, first.du, first.dv);
        }
        let last = &self.nodes[self.nodes.len() - 1];
        if s_of(last) > 0.0 {
            return (last.tau, last.u, last.v, last.du, last.dv);
        }
        // Invariant: s(lo) > 0 >= s(hi).
        let (mut lo, mut hi) = (0, self.nodes.len() - 1);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if s_of(&self.nodes[mid]) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (n0, n1) = (&self.nodes[lo], &self.nodes[hi]);
        let (s0, s1) = (s_of(n0), s_of(n1));
        if s1 == 0.0 {
            return (n1.tau, n1.u, n1.v, n1.du, n1.dv);
        }
        // Between two nodes the locus is the cubic Hermite curve through their exact points and
        // exact derivatives (in tau), which is within 1e-10 of the locus in (u, v) (tested; 8e-11
        // at worst); on it s(t) = (p - L(t)) . L'(t), t in [0, 1], is a polynomial, and Newton's
        // method, with
        // s'(t) = -|L'|^2 + (p - L) . L'', starts from Robertson's interpolation t = s0 / (s0 - s1)
        // and falls back to bisection if a step leaves the bracket. No table is read.
        let dt = n1.tau - n0.tau;
        let (p0, p1) = ((n0.u, n0.v), (n1.u, n1.v));
        let (m0, m1) = ((n0.du * dt, n0.dv * dt), (n1.du * dt, n1.dv * dt));
        let curve = |t: f64| {
            let (t2, t3) = (t * t, t * t * t);
            let h = [
                2.0 * t3 - 3.0 * t2 + 1.0,
                t3 - 2.0 * t2 + t,
                -2.0 * t3 + 3.0 * t2,
                t3 - t2,
            ];
            let d = [
                6.0 * t2 - 6.0 * t,
                3.0 * t2 - 4.0 * t + 1.0,
                -6.0 * t2 + 6.0 * t,
                3.0 * t2 - 2.0 * t,
            ];
            let e = [
                12.0 * t - 6.0,
                6.0 * t - 4.0,
                -12.0 * t + 6.0,
                6.0 * t - 2.0,
            ];
            let mix = |w: [f64; 4]| {
                (
                    w[0] * p0.0 + w[1] * m0.0 + w[2] * p1.0 + w[3] * m1.0,
                    w[0] * p0.1 + w[1] * m0.1 + w[2] * p1.1 + w[3] * m1.1,
                )
            };
            (mix(h), mix(d), mix(e))
        };
        let (mut lo_t, mut hi_t) = (0.0f64, 1.0f64);
        let mut t = s0 / (s0 - s1);
        let mut at = curve(t);
        for _ in 0..8 {
            let (l, d, e) = at;
            let (ex, ey) = (p.0 - l.0, p.1 - l.1);
            let s = ex * d.0 + ey * d.1;
            if s > 0.0 {
                lo_t = t;
            } else {
                hi_t = t;
            }
            let ds = -(d.0 * d.0 + d.1 * d.1) + ex * e.0 + ey * e.1;
            let step = s / ds;
            // Newton converges quadratically: after a step below 1e-9 of the interval (1e-11 of
            // ln T) the error left is ~1e-18, far below the rounding noise of s itself, which
            // moves t by ~1e-12; iterating further would only walk in that noise. A converged t
            // sits on one end of the bracket, so the tiny last step may leave the bracket by
            // rounding: it is clamped, never sent to bisection.
            if step.abs() < 1e-9 {
                t = (t - step).clamp(lo_t, hi_t);
                at = curve(t);
                break;
            }
            let next = t - step;
            t = if next > lo_t && next < hi_t {
                next
            } else {
                0.5 * (lo_t + hi_t)
            };
            at = curve(t);
        }
        let (l, d, _) = at;
        (n0.tau + t * dt, l.0, l.1, d.0 / dt, d.1 / dt)
    }

    /// tau of the locus point with the same u, clamped to the ends: continuous in u.
    fn continuation(&self, u: f64) -> f64 {
        let n = &self.nodes;
        if u <= n[0].u {
            return n[0].tau;
        }
        if u >= n[n.len() - 1].u {
            return n[n.len() - 1].tau;
        }
        let (mut lo, mut hi) = (0, n.len() - 1);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if n[mid].u <= u {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let f = (u - n[lo].u) / (n[hi].u - n[lo].u);
        n[lo].tau + f * (n[hi].tau - n[lo].tau)
    }

    #[cfg(test)]
    pub(crate) fn node_points(&self) -> Vec<(f64, f64, f64, f64, f64)> {
        self.nodes
            .iter()
            .map(|n| (n.tau, n.u, n.v, n.du, n.dv))
            .collect()
    }
}

/// The chromaticity (u, v) of tristimulus values of any positive scale, computed after scaling by
/// the largest so that very dark pixels (down to f64's least positive value) keep their colour.
#[inline]
pub(crate) fn chromaticity(xyz: [f64; 3]) -> (f64, f64) {
    let m = xyz[0].max(xyz[1]).max(xyz[2]);
    uv([xyz[0] / m, xyz[1] / m, xyz[2] / m])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::colorimetry::{BT709_WHITE, rgb_to_xyz};
    use crate::model;
    use crate::planck::blackbody_xyz;

    #[test]
    fn test_a_blackbody_colour_implies_its_own_temperature_and_duv_zero() {
        // From the direct sum, not the table, so that the two are compared. The tolerance is
        // what the table's 2e-9 in X, Y, Z allows: an error e in (u, v) moves T by
        // e / |dL / d ln T|, and |dL / d ln T| falls from ~0.07 at 3000 K to ~0.002 at 10^6 K.
        let m = model();
        for &t in &[
            300.0, 500.0, 985.0, 1500.0, 2856.0, 5778.0, 6504.0, 1e4, 4e4, 2e5, 1e6, 3e7, 1e10,
        ] {
            let i = m.implied_temperature_xyz(blackbody_xyz(t)).unwrap();
            let tol = if t > 1e5 { 1e-6 } else { 1e-8 };
            assert!((i.kelvin / t - 1.0).abs() < tol, "{t}: {}", i.kelvin);
            assert!(i.duv.abs() < 1e-10, "{t}: {}", i.duv);
        }
        // The hot end: the Rayleigh-Jeans colour implies infinity.
        let rj = m.table.rayleigh_jeans();
        let i = m.implied_temperature_xyz(rj).unwrap();
        assert!(i.kelvin.is_infinite() || i.kelvin > 1e12, "{}", i.kelvin);
    }

    #[test]
    fn test_bt709_white_implies_6504_k_with_duv_plus_0_0032() {
        // Independent value: colour-science 0.4.7, colour.uv_to_CCT(..., method="Ohno 2013") on
        // (0.3127, 0.3290): 6504.31 K, Duv 0.003207 (its c2 is 1.4388e-2, which moves T by
        // 1.6e-5 of itself, 0.1 K).
        let (x, y) = BT709_WHITE;
        let i = model()
            .implied_temperature_xyz([x / y, 1.0, (1.0 - x - y) / y])
            .unwrap();
        assert!((i.kelvin - 6504.3).abs() < 0.5, "{}", i.kelvin);
        assert!((i.duv - 0.00321).abs() < 1e-5, "{}", i.duv);
        // The CIE's D65 spectrum summed at 1 nm: (0.312727, 0.329023); Ohno 2013 gives 6502.68 K.
        let (x, y) = (0.312_727, 0.329_023);
        let i = model()
            .implied_temperature_xyz([x / y, 1.0, (1.0 - x - y) / y])
            .unwrap();
        assert!((i.kelvin - 6502.7).abs() < 0.5, "{}", i.kelvin);
    }

    #[test]
    fn test_duv_is_positive_above_the_locus_and_negative_below() {
        let m = model();
        let green = m.implied_temperature([0.0, 1.0, 0.0]).unwrap();
        let magenta = m.implied_temperature([1.0, 0.0, 1.0]).unwrap();
        assert!(
            green.duv > 0.09 && magenta.duv < -0.1,
            "{green:?} {magenta:?}"
        );
        // Red, (u, v) = (0.4507, 0.3486), is just below the locus, nearest it at 986 K.
        let red = m.implied_temperature([1.0, 0.0, 0.0]).unwrap();
        assert!(
            (red.kelvin - 986.4).abs() < 0.5 && (red.duv + 0.00578).abs() < 1e-4,
            "{red:?}"
        );
        // A mixture of two blackbodies lies on a chord of the locus, which bends toward the
        // magentas, so it is below the locus.
        let mix: [f64; 3] =
            std::array::from_fn(|c| m.blackbody_xyz(3000.0)[c] + m.blackbody_xyz(9000.0)[c]);
        assert!(m.implied_temperature_xyz(mix).unwrap().duv < 0.0);
    }

    #[test]
    fn test_the_hermite_curve_between_nodes_is_within_1e_10_of_the_locus() {
        // The search refines on the cubic through two nodes' exact points and derivatives. At
        // the midpoint of every interval, compare it with the locus evaluated from the table.
        let m = model();
        let pts = m.locus.node_points();
        let mut worst = 0.0f64;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let dt = b.0 - a.0;
            // Hermite at t = 1/2: (P0 + P1) / 2 + dt (D0 - D1) / 8.
            let hu = 0.5 * (a.1 + b.1) + dt * (a.3 - b.3) / 8.0;
            let hv = 0.5 * (a.2 + b.2) + dt * (a.4 - b.4) / 8.0;
            let (u, v, _, _) = locus_at(&m.table, 0.5 * (a.0 + b.0));
            worst = worst.max((hu - u).hypot(hv - v));
        }
        assert!(worst < 1e-10, "{worst:e}");
    }

    #[test]
    fn test_u_rises_monotonically_along_the_locus_so_the_continuation_is_well_defined() {
        let pts = model().locus.node_points();
        for w in pts.windows(2) {
            assert!(
                w[1].1 > w[0].1,
                "u falls between tau {} and {}",
                w[0].0,
                w[1].0
            );
            assert!(w[0].3 > 0.0, "du/dtau <= 0 at tau {}", w[0].0);
        }
    }

    /// The gamut's surface (max channel 1), which carries every chromaticity of the gamut.
    fn gamut_surface(n: usize) -> Vec<[f64; 3]> {
        let mut out = Vec::new();
        for face in 0..3 {
            for i in 0..=n {
                for j in 0..=n {
                    let mut c = [0.0; 3];
                    c[face] = 1.0;
                    c[(face + 1) % 3] = i as f64 / n as f64;
                    c[(face + 2) % 3] = j as f64 / n as f64;
                    out.push(c);
                }
            }
        }
        out
    }

    #[test]
    fn test_the_nearest_point_is_unique_wherever_it_is_used() {
        // For every chromaticity of the gamut: count the changes of sign of s over the locus's
        // nodes. Where there is more than one (two local minima of the distance), every local
        // minimum must lie below the locus and farther than CONTINUATION_DUV, so that the rule
        // has already handed over entirely to the continuation and no jump reaches the output.
        // The least such distance tends, as the grid is refined, to the least radius of
        // curvature, 0.1001 (measured 0.1032 at n = 120, 0.1017 at 400, 0.1003 at 1500); at the
        // n = 200 used here it is 0.1024.
        let m = model();
        let pts = m.locus.node_points();
        let mut least = f64::INFINITY;
        for rgb in gamut_surface(200) {
            let p = chromaticity(rgb_to_xyz(rgb));
            let s: Vec<f64> = pts
                .iter()
                .map(|n| (p.0 - n.1) * n.3 + (p.1 - n.2) * n.4)
                .collect();
            let changes = s
                .windows(2)
                .filter(|w| (w[0] > 0.0) != (w[1] > 0.0))
                .count();
            if changes > 1 {
                let i = m.implied_temperature(rgb).unwrap();
                assert!(i.duv < -CONTINUATION_DUV, "{rgb:?}: {i:?}");
                for k in 1..s.len() {
                    if s[k - 1] > 0.0 && s[k] <= 0.0 {
                        let d = (p.0 - pts[k].1).hypot(p.1 - pts[k].2);
                        least = least.min(d);
                    }
                }
            }
        }
        assert!(least > 0.1, "a second minimum at {least}");
        // And the hand-over is complete a margin of 0.01 inside that.
        const { assert!(CONTINUATION_DUV <= 0.1 - 0.009) };
    }

    #[test]
    fn test_the_locus_bends_one_way_and_its_least_radius_of_curvature_is_0_1001_at_5235_k() {
        // Curvature from the nodes' exact tangents: kappa = (u' v'' - v' u'') / |L'|^3 with the
        // second derivatives by central differences in tau. Its sign must not change (one bend),
        // and 1 / |kappa| is least, 0.1001, near 5235 K: the bound on how far from the locus the
        // nearest point is unique.
        let pts = model().locus.node_points();
        let mut least = (f64::INFINITY, 0.0);
        let mut sign = 0.0;
        for w in pts.windows(3) {
            let (a, b, c) = (w[0], w[1], w[2]);
            let (du2, dv2) = ((c.3 - a.3) / (c.0 - a.0), (c.4 - a.4) / (c.0 - a.0));
            let k = (b.3 * dv2 - b.4 * du2) / b.3.hypot(b.4).powi(3);
            if 1.0 / b.0 < 5e5 && 1.0 / b.0 > 400.0 {
                if sign == 0.0 {
                    sign = k.signum();
                }
                assert_eq!(
                    k.signum(),
                    sign,
                    "the locus bends the other way at {} K",
                    1.0 / b.0
                );
            }
            if 1.0 / k.abs() < least.0 {
                least = (1.0 / k.abs(), 1.0 / b.0);
            }
        }
        assert!(
            (least.0 - 0.1001).abs() < 2e-4 && (least.1 - 5235.0).abs() < 100.0,
            "{least:?}"
        );
    }

    /// The largest change of tau = 1 / T between neighbours of an (n + 1)^2 grid on each face of
    /// the gamut's surface.
    fn largest_step(n: usize) -> f64 {
        let m = model();
        let mut largest = 0.0f64;
        for face in 0..3 {
            let grid: Vec<Vec<f64>> = (0..=n)
                .map(|i| {
                    (0..=n)
                        .map(|j| {
                            let mut c = [0.0; 3];
                            c[face] = 1.0;
                            c[(face + 1) % 3] = i as f64 / n as f64;
                            c[(face + 2) % 3] = j as f64 / n as f64;
                            1.0 / m.implied_temperature(c).unwrap().kelvin
                        })
                        .collect()
                })
                .collect();
            for i in 0..=n {
                for j in 0..=n {
                    if i > 0 {
                        largest = largest.max((grid[i][j] - grid[i - 1][j]).abs());
                    }
                    if j > 0 {
                        largest = largest.max((grid[i][j] - grid[i][j - 1]).abs());
                    }
                }
            }
        }
        largest
    }

    #[test]
    fn test_the_implied_temperature_is_continuous_over_the_gamut() {
        // A continuous function's largest step between grid neighbours halves when the grid is
        // made twice as fine; a jump anywhere keeps it at the jump's size (between the locus's
        // two ends, ~5e-4 K^-1). Measured: 4.19e-5 at n = 100 and 2.15e-5 at n = 200.
        let coarse = largest_step(100);
        let fine = largest_step(200);
        assert!(fine < 0.6 * coarse, "{coarse:e} -> {fine:e}");
        assert!(coarse < 6e-5, "{coarse:e}");
    }
}
