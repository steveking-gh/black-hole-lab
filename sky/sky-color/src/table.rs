//! The blackbody's tristimulus values as a fast function of temperature.
//!
//! The renderer asks for them twice per pixel (at the implied T and at g T), 33 million pixels a
//! frame, and the direct sum costs 471 exponentials. So they are tabulated once.
//!
//! # The table
//!
//! What is tabulated is f_c(w) = ln X_c(e^w), the logarithm of each tristimulus value against the
//! logarithm of the temperature, together with its exact slope f_c'(w) = d ln X_c / d ln T (from
//! [`crate::planck::ln_blackbody_xyz`]), at 128 nodes per e-fold of T from 10 K to 10^6 K
//! (1475 nodes, 71 KB). Between nodes it is read by cubic Hermite interpolation, which uses both
//! and is exact for cubics.
//!
//! Logarithms, because X spans e^-1700 to 10^+2 over the range and its relative accuracy is what
//! matters; ln T, because that is the variable in which the curves are smoothest: in the Wien tail
//! f(w) ~ -c2 / (lambda e^w), whose derivatives in w are all of the size of f itself, c2 / lambda T
//! (~ 1700 at 10 K, ~ 3 at 6000 K), and in the Rayleigh-Jeans tail f(w) -> w + const. The error of
//! cubic Hermite interpolation on a step h is at most h^4 max|f''''| / 384; with h = 1/128 and
//! |f''''| ~ c2 / lambda T that is 4e-9 at 300 K and 2e-8 at 10 K, and below 1e-10 above 3000 K.
//! `test_the_table_matches_the_direct_sum_from_10_k_to_a_billion_kelvin` measures it against the
//! direct sum at every midpoint between nodes (where the error is largest) and holds it to 3e-8 of
//! the value from 10 K to 300 K (measured 2.1e-8, at 10 K) and 3e-9 above (measured 2.1e-9).
//!
//! Above 10^6 K the values come from the high-temperature series
//! ([`crate::planck::high_temperature_coefficients`]), exact there to 1e-13, and T may be
//! infinite in a ratio (see [`crate::Model::xyz_ratio`]). Below 10 K they come from the direct sum
//! in logarithms: slow, and never asked for by the colour shift (see `shift`).

use crate::planck::{high_temperature_coefficients, ln_blackbody_xyz};

/// The lowest temperature in the table, K.
pub const T_TABLE_MIN: f64 = 10.0;
/// The highest temperature in the table, K; above it the high-temperature series is used.
pub const T_TABLE_MAX: f64 = 1e6;
const NODES: usize = 1475;

#[derive(Debug, Clone)]
pub(crate) struct Table {
    w0: f64,
    h: f64,
    inv_h: f64,
    ln_xyz: Vec<[f64; 3]>,
    slope: Vec<[f64; 3]>,
    series: [[f64; 4]; 3],
}

impl Table {
    pub(crate) fn new() -> Self {
        let w0 = T_TABLE_MIN.ln();
        let h = (T_TABLE_MAX.ln() - w0) / (NODES - 1) as f64;
        let mut ln_xyz = Vec::with_capacity(NODES);
        let mut slope = Vec::with_capacity(NODES);
        for k in 0..NODES {
            // The last node exactly at T_TABLE_MAX, so that the table and the series meet there.
            let t = if k == NODES - 1 {
                T_TABLE_MAX
            } else {
                (w0 + h * k as f64).exp()
            };
            let (f, s) = ln_blackbody_xyz(t);
            ln_xyz.push(f);
            slope.push(s);
        }
        Self {
            w0,
            h,
            inv_h: 1.0 / h,
            ln_xyz,
            slope,
            series: high_temperature_coefficients(),
        }
    }

    /// ln X, ln Y, ln Z (W m^-2 sr^-1) of a blackbody at finite `t` > 0, and their slopes
    /// d ln X / d ln T.
    #[inline]
    pub(crate) fn ln_xyz_and_slope(&self, t: f64) -> ([f64; 3], [f64; 3]) {
        if t > T_TABLE_MAX {
            return self.series_ln_and_slope(t);
        }
        if t < T_TABLE_MIN {
            return ln_blackbody_xyz(t);
        }
        let x = (t.ln() - self.w0) * self.inv_h;
        let k = (x as usize).min(NODES - 2);
        let s = x - k as f64;
        let (s2, s3) = (s * s, s * s * s);
        let (h00, h10, h01, h11) = (
            2.0 * s3 - 3.0 * s2 + 1.0,
            s3 - 2.0 * s2 + s,
            -2.0 * s3 + 3.0 * s2,
            s3 - s2,
        );
        let (d00, d10, d01, d11) = (
            6.0 * s2 - 6.0 * s,
            3.0 * s2 - 4.0 * s + 1.0,
            -6.0 * s2 + 6.0 * s,
            3.0 * s2 - 2.0 * s,
        );
        let (f0, f1, m0, m1) = (
            &self.ln_xyz[k],
            &self.ln_xyz[k + 1],
            &self.slope[k],
            &self.slope[k + 1],
        );
        let mut f = [0.0; 3];
        let mut d = [0.0; 3];
        for c in 0..3 {
            f[c] = h00 * f0[c] + h10 * self.h * m0[c] + h01 * f1[c] + h11 * self.h * m1[c];
            d[c] = (d00 * f0[c] + d10 * self.h * m0[c] + d01 * f1[c] + d11 * self.h * m1[c])
                * self.inv_h;
        }
        (f, d)
    }

    /// ln X, ln Y, ln Z of a blackbody at finite `t` > 0.
    #[inline]
    pub(crate) fn ln_xyz(&self, t: f64) -> [f64; 3] {
        self.ln_xyz_and_slope(t).0
    }

    fn series_ln_and_slope(&self, t: f64) -> ([f64; 3], [f64; 3]) {
        let a = &self.series;
        let mut f = [0.0; 3];
        let mut d = [0.0; 3];
        for c in 0..3 {
            let x = a[c][0] * t - a[c][1] + a[c][2] / t - a[c][3] / (t * t * t);
            f[c] = x.ln();
            d[c] = (a[c][0] * t - a[c][2] / t + 3.0 * a[c][3] / (t * t * t)) / x;
        }
        (f, d)
    }

    /// The blackbody's X, Y, Z multiplied by tau = 1 / T, and the derivative of that product with
    /// respect to tau, for tau in [0, 1 / T_TABLE_MAX]: from the series,
    ///
    ///     tau X = a1 - a0 tau + a_1 tau^2 - a_3 tau^4,
    ///
    /// which is finite at tau = 0 (T infinite), where it is the Rayleigh-Jeans limit a1.
    pub(crate) fn scaled_series(&self, tau: f64) -> ([f64; 3], [f64; 3]) {
        let a = &self.series;
        let t2 = tau * tau;
        let mut v = [0.0; 3];
        let mut d = [0.0; 3];
        for c in 0..3 {
            v[c] = a[c][0] - a[c][1] * tau + a[c][2] * t2 - a[c][3] * t2 * t2;
            d[c] = -a[c][1] + 2.0 * a[c][2] * tau - 4.0 * a[c][3] * t2 * tau;
        }
        (v, d)
    }

    #[cfg(test)]
    /// The Rayleigh-Jeans coefficients a1 (X = a1 T at T -> infinity), W m^-2 sr^-1 K^-1.
    pub(crate) fn rayleigh_jeans(&self) -> [f64; 3] {
        [self.series[0][0], self.series[1][0], self.series[2][0]]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planck::blackbody_xyz;

    #[test]
    fn test_the_table_matches_the_direct_sum_from_10_k_to_a_billion_kelvin() {
        // At every midpoint between nodes, where Hermite interpolation is least accurate, and at
        // the nodes. The bounds (3e-8 below 300 K, 3e-9 above, in ln X, which is the relative
        // error of X) are those stated in the module comment, measured and not merely estimated.
        let table = Table::new();
        let mut worst_low = 0.0f64;
        let mut worst_high = 0.0f64;
        for k in 0..2 * (NODES - 1) {
            let t = (table.w0 + table.h * k as f64 * 0.5).exp();
            let (direct, _) = ln_blackbody_xyz(t);
            let fast = table.ln_xyz(t);
            for c in 0..3 {
                let e = (fast[c] - direct[c]).abs();
                if t < 300.0 {
                    worst_low = worst_low.max(e);
                } else {
                    worst_high = worst_high.max(e);
                }
            }
        }
        assert!(worst_low < 3e-8, "below 300 K: {worst_low:e}");
        assert!(worst_high < 3e-9, "above 300 K: {worst_high:e}");
        // Above the table, the series against the direct sum.
        for &t in &[1.0001e6, 3e6, 1e7, 1e9] {
            let direct = blackbody_xyz(t);
            let fast = table.ln_xyz(t);
            for c in 0..3 {
                assert!((fast[c] - direct[c].ln()).abs() < 1e-11, "T {t}");
            }
        }
    }

    #[test]
    fn test_the_series_and_the_table_meet_at_a_million_kelvin() {
        let table = Table::new();
        let below = table.ln_xyz_and_slope(T_TABLE_MAX);
        let above = table.series_ln_and_slope(T_TABLE_MAX);
        for c in 0..3 {
            assert!((below.0[c] - above.0[c]).abs() < 1e-12);
            assert!((below.1[c] - above.1[c]).abs() < 1e-9);
        }
    }

    #[test]
    fn test_the_tables_slope_is_the_derivative_of_its_values() {
        // The slope is read from the same Hermite cubic; a mismatch would bend the locus's
        // tangent and so the nearest-point search.
        let table = Table::new();
        for &t in &[12.0, 300.0, 1234.5, 5778.0, 3e4, 9e5] {
            let e = 1e-5;
            let up = table.ln_xyz(t * f64::exp(e));
            let down = table.ln_xyz(t * f64::exp(-e));
            let (_, d) = table.ln_xyz_and_slope(t);
            for c in 0..3 {
                let numeric = (up[c] - down[c]) / (2.0 * e);
                assert!(
                    (numeric - d[c]).abs() < 1e-5 * d[c].abs().max(1.0),
                    "T {t} c {c}"
                );
            }
        }
    }
}
