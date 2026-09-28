//! Planck's law, why a shifted blackbody is a blackbody, and the eye's integral of a blackbody.
//!
//! # Constants
//!
//! Since the 2019 revision of the SI, h, c and k are exact, and so is the second radiation constant
//!
//!     c2 = h c / k = 1.438 776 877... x 10^-2 m K,
//!
//! which is the one used here ([`C2`]). The spectrum of a blackbody depends on T only through
//! c2 / T, so the *same* spectrum carries different temperatures under different values of c2, and
//! a published chromaticity is only reproduced with the c2 it was computed with:
//!
//! - CIE illuminant A is defined (ISO/CIE 11664-2:2022, equation 1; the CIE's 1 nm data set,
//!   DOI 10.25039/CIE.DS.8jsxjrsn) as the relative spectrum of a Planckian radiator with
//!   c2 = 1.435 x 10^-2 m K and T = 2848 K. The temperature "2856 K" quoted for it is the same
//!   spectrum restated with the 1968 value c2 = 1.4388 x 10^-2 (2848 x 1.4388 / 1.435 = 2855.54,
//!   rounded). With the exact c2 the same spectrum is T = 2848 x 1.438777 / 1.435 = 2855.50 K, and
//!   a blackbody at 2856 K with the exact c2 is 4e-5 away from illuminant A in x. Its chromaticity
//!   (x, y) = (0.44757, 0.40745) is reproduced by [`C2_ILLUMINANT_A`] at 2848 K to 1e-5 (tested).
//! - The Planckian locus and the correlated colour temperatures in the literature were mostly
//!   computed with c2 = 1.4388 x 10^-2 (CIE 015:2018 uses it). It differs from the exact value by
//!   1.6e-5 of itself, which moves a temperature by as much: 0.1 K at 6504 K. The nominal "6504 K"
//!   of D65 is 6500 x 1.4388 / 1.4380, D65's historical 6500 K moved from c2 = 1.4380e-2 to
//!   1.4388e-2; it is a label, not a computed correlated colour temperature (see `temperature`).
//!
//! # Planck's law in the two variables
//!
//! Per unit frequency and per unit wavelength (radiance, W m^-2 sr^-1 Hz^-1 and W m^-2 sr^-1 m^-1):
//!
//!     B_nu(T)     = (2 h nu^3 / c^2)  / (exp(h nu / k T) - 1),
//!     B_lambda(T) = (2 h c^2 / lambda^5) / (exp(c2 / lambda T) - 1),
//!
//! related by B_lambda = B_nu c / lambda^2.
//!
//! # A shifted blackbody is a blackbody
//!
//! Along a ray I_nu / nu^3 is invariant (Liouville's theorem for photons; the specification,
//! section 4.6). Light emitted at nu / g and received at nu therefore has
//!
//!     I_nu,observed(nu) = g^3 I_nu,emitted(nu / g).
//!
//! For a blackbody at T,
//!
//!     g^3 B_{nu/g}(T) = g^3 (2 h nu^3 / (g^3 c^2)) / (exp(h nu / (k g T)) - 1)
//!                     = (2 h nu^3 / c^2) / (exp(h nu / (k (g T))) - 1) = B_nu(g T):
//!
//! the g^3 cancels the cube of the frequency's shift exactly, and the shift of the exponent's
//! argument is a change of temperature. The light of a blackbody at T, seen with shift g, is the
//! light of a blackbody at g T, with no other factor
//! (`test_a_shifted_blackbody_is_the_blackbody_at_g_times_the_temperature`).
//!
//! What that covers: the specific intensity, the surface brightness of the light, which is what a
//! pixel of a resolved patch of sky shows. Lensing changes the solid angle an image covers and not
//! its surface brightness (again because I_nu / nu^3 is conserved), so an extended source needs
//! nothing more. An unresolved point source is different: its flux is its surface brightness times
//! its solid angle, and lensing multiplies that solid angle by the magnification. That factor is
//! the renderer's business (it is colourless) and is not in this crate.
//!
//! Integrated over frequency, B(T) = integral B_nu dnu = sigma T^4 / pi (Stefan-Boltzmann), so the
//! bolometric radiance is multiplied by (g T)^4 / T^4 = g^4: the renderer's old factor, recovered
//! as the special case of the eye being equally sensitive to every frequency. The eye is not, and
//! [`crate::Model::visible_factor`] is what it sees instead.
//!
//! # The eye's integral
//!
//! The tristimulus values of a spectral radiance L_lambda are
//!
//!     X = integral L_lambda(lambda) x-bar(lambda) d lambda,     and Y, Z with y-bar, z-bar,
//!
//! over the table's range, 360 to 830 nm, outside which the CIE sets the functions to zero. The
//! integral is taken as the CIE takes it (CIE 015:2018, section 7.2: summation at the table's
//! 1 nm interval), here as the trapezoid rule on the 1 nm rows, which differs from the plain sum
//! only by half of the two end rows (1e-6 of the total). With L_lambda in W m^-2 sr^-1 m^-1 and
//! d lambda in metres, X, Y and Z are in W m^-2 sr^-1, and [`K_M`] Y is the luminance in cd m^-2.
//!
//! The rule's error against the continuous integral of the Planck function times the linearly
//! interpolated table (the interpolation the CIE's metadata specifies) was measured with 64
//! sub-steps per nanometre (tested at 16): for T >= 1000 K it is below 2e-4 of X, Y and Z and
//! below 3e-6 in chromaticity; from 150 K to 1000 K below 5e-4 of X and Y. It grows at very low
//! temperature, where the Planck function changes by a large factor between rows (at 100 K by
//! e^{2.1} from 829 to 830 nm) and the sum is dominated by its last row: 1e-3 at 100 K, 3 % at
//! 30 K. Nothing there is visible: the visible radiance of a 300 K body is 5e-31 of a 5778 K
//! body's.
//!
//! The table's end at 830 nm is also what fixes the colour of a cool body. Under the Wien tail the
//! longest wavelengths dominate more and more as T falls, so the chromaticity tends to that of the
//! last row, (x, y) = (0.73469, 0.26531), reached to 1e-6 at 100 K: the colour of 830 nm by the
//! CIE's convention of zero beyond it, not a measured colour of anything.

use crate::cie1931::{CMF, ROWS, wavelength_nm};

/// Planck's constant, J s (exact, SI 2019).
pub const PLANCK_H: f64 = 6.626_070_15e-34;
/// The speed of light, m s^-1 (exact).
pub const LIGHT_C: f64 = 299_792_458.0;
/// Boltzmann's constant, J K^-1 (exact, SI 2019).
pub const BOLTZMANN_K: f64 = 1.380_649e-23;
/// The second radiation constant c2 = h c / k, m K, from the exact values: 1.438776877e-2.
pub const C2: f64 = PLANCK_H * LIGHT_C / BOLTZMANN_K;
/// The first radiation constant for radiance, c1L = 2 h c^2, W m^2 sr^-1.
pub const C1L: f64 = 2.0 * PLANCK_H * LIGHT_C * LIGHT_C;
/// The c2 with which CIE illuminant A is defined (ISO/CIE 11664-2:2022, equation 1), m K.
pub const C2_ILLUMINANT_A: f64 = 1.435e-2;
/// The temperature with which CIE illuminant A is defined, under [`C2_ILLUMINANT_A`], K.
pub const T_ILLUMINANT_A: f64 = 2848.0;
/// The maximum luminous efficacy of photopic vision, lm W^-1 (CIE 018:2019): luminance in cd m^-2
/// is `K_M * Y` with Y from this crate.
pub const K_M: f64 = 683.002;
/// The Stefan-Boltzmann constant, 2 pi^5 k^4 / (15 c^2 h^3), W m^-2 K^-4.
pub const SIGMA: f64 = {
    let pi = std::f64::consts::PI;
    let k = BOLTZMANN_K;
    2.0 * pi * pi * pi * pi * pi * k * k * k * k
        / (15.0 * LIGHT_C * LIGHT_C * PLANCK_H * PLANCK_H * PLANCK_H)
};

/// B_nu(T), W m^-2 sr^-1 Hz^-1.
pub fn planck_per_frequency(nu: f64, t: f64) -> f64 {
    2.0 * PLANCK_H * nu * nu * nu
        / (LIGHT_C * LIGHT_C)
        / (PLANCK_H * nu / (BOLTZMANN_K * t)).exp_m1()
}

/// B_lambda(T), W m^-2 sr^-1 m^-1, with the exact c2.
pub fn planck_per_wavelength(lambda_m: f64, t: f64) -> f64 {
    planck_per_wavelength_c2(lambda_m, t, C2)
}

/// B_lambda(T) with a given c2 (and c1L unchanged), for reproducing definitions made with an older
/// value.
pub fn planck_per_wavelength_c2(lambda_m: f64, t: f64, c2: f64) -> f64 {
    C1L / lambda_m.powi(5) / (c2 / (lambda_m * t)).exp_m1()
}

/// The trapezoid weight of row `k` in nanometres, times 1e-9 to make d lambda metres.
fn row_weight(k: usize) -> f64 {
    let w = if k == 0 || k == ROWS - 1 { 0.5 } else { 1.0 };
    w * 1e-9
}

/// The tristimulus values of any spectral radiance `spectrum(lambda_m)` (W m^-2 sr^-1 m^-1), by
/// the trapezoid rule on the table's rows. W m^-2 sr^-1.
pub fn tristimulus_of(spectrum: impl Fn(f64) -> f64) -> [f64; 3] {
    let mut xyz = [0.0; 3];
    for (k, row) in CMF.iter().enumerate() {
        let s = spectrum(wavelength_nm(k) * 1e-9) * row_weight(k);
        for c in 0..3 {
            xyz[c] += s * row[c];
        }
    }
    xyz
}

/// The tristimulus values of a blackbody at `t` kelvin, in W m^-2 sr^-1, by direct summation:
/// the reference against which the fast table ([`crate::Model::blackbody_xyz`]) is tested. For
/// `t` below about 24 K the values underflow to zero; [`ln_blackbody_xyz`] does not.
pub fn blackbody_xyz(t: f64) -> [f64; 3] {
    tristimulus_of(|l| planck_per_wavelength(l, t))
}

/// As [`blackbody_xyz`], with a given c2.
pub fn blackbody_xyz_c2(t: f64, c2: f64) -> [f64; 3] {
    tristimulus_of(|l| planck_per_wavelength_c2(l, t, c2))
}

/// ln B_lambda(T) for x = c2 / (lambda T), without overflow or underflow at any x > 0:
/// ln B = ln c1L - 5 ln lambda - ln(e^x - 1), and ln(e^x - 1) = x + ln(1 - e^-x) for large x.
fn ln_planck(lambda_m: f64, x: f64) -> f64 {
    let ln_denominator = if x > 30.0 {
        x + (-(-x).exp()).ln_1p()
    } else {
        x.exp_m1().ln()
    };
    C1L.ln() - 5.0 * lambda_m.ln() - ln_denominator
}

/// The natural logarithms of a blackbody's X, Y, Z (W m^-2 sr^-1) and their slopes
/// d ln X / d ln T, for any T > 0, by direct summation in logarithms.
///
/// The slope follows from d ln B_lambda / d ln T = x e^x / (e^x - 1) = x / (1 - e^-x), weighted by
/// each row's share of the sum. The sums are formed as log-sum-exp, relative to their largest
/// term, so that a 10 K body (whose X is e^-1700 W m^-2 sr^-1, below f64's range) has finite
/// logarithms and exact slopes. The fast table is built from this.
pub fn ln_blackbody_xyz(t: f64) -> ([f64; 3], [f64; 3]) {
    let mut terms: Vec<(f64, f64, usize)> = Vec::with_capacity(3 * ROWS);
    let mut ln_max = [f64::NEG_INFINITY; 3];
    for (k, row) in CMF.iter().enumerate() {
        let lambda = wavelength_nm(k) * 1e-9;
        let x = C2 / (lambda * t);
        let ln_b = ln_planck(lambda, x) + row_weight(k).ln();
        let slope = x / -(-x).exp_m1();
        for c in 0..3 {
            if row[c] > 0.0 {
                let ln_term = ln_b + row[c].ln();
                ln_max[c] = ln_max[c].max(ln_term);
                terms.push((ln_term, slope, c));
            }
        }
    }
    let mut sum = [0.0; 3];
    let mut weighted = [0.0; 3];
    for &(ln_term, slope, c) in &terms {
        let e = (ln_term - ln_max[c]).exp();
        sum[c] += e;
        weighted[c] += e * slope;
    }
    (
        std::array::from_fn(|c| ln_max[c] + sum[c].ln()),
        std::array::from_fn(|c| weighted[c] / sum[c]),
    )
}

/// The coefficients of a blackbody's X, Y, Z at high temperature,
///
///     X(T) = a1 T - a0 + a_1 / T - a_3 / T^3 + O(T^-5),
///
/// from 1 / (e^x - 1) = 1/x - 1/2 + x/12 - x^3/720 + ... with x = c2 / (lambda T), row by row:
///
///     B_lambda = (c1L / lambda^5)
///                [lambda T / c2 - 1/2 + c2 / (12 lambda T) - c2^3 / (720 lambda^3 T^3)].
///
/// The first term is the Rayleigh-Jeans law, B_lambda -> 2 c k T / lambda^4: the radiance in any
/// fixed band grows as T, and the chromaticity tends to that of the integrals of lambda^-4 against
/// the functions. The next term omitted is x^5 / 30240 against 1/x, relative size x^6 / 30240: at
/// 1e6 K and 360 nm x = 0.04, so the series is exact there to 1e-13.
pub fn high_temperature_coefficients() -> [[f64; 4]; 3] {
    let mut a = [[0.0; 4]; 3];
    for (k, row) in CMF.iter().enumerate() {
        let lambda = wavelength_nm(k) * 1e-9;
        let base = C1L / lambda.powi(5) * row_weight(k);
        let terms = [
            lambda / C2,
            0.5,
            C2 / (12.0 * lambda),
            C2.powi(3) / (720.0 * lambda.powi(3)),
        ];
        for c in 0..3 {
            for (j, term) in terms.iter().enumerate() {
                a[c][j] += base * term * row[c];
            }
        }
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::colorimetry::xy;

    #[test]
    fn test_c2_from_the_exact_si_constants_is_1_438776877e_minus_2() {
        // CODATA 2018 lists c2 = 1.438 776 877... e-2 m K, exact.
        assert!((C2 / 1.438_776_877e-2 - 1.0).abs() < 1e-9, "{C2}");
    }

    #[test]
    fn test_a_shifted_blackbody_is_the_blackbody_at_g_times_the_temperature() {
        // g^3 B_{nu/g}(T) = B_nu(g T), pointwise, for shifts either way and frequencies from the
        // far infrared to the ultraviolet. Both sides are the same arithmetic reordered, so they
        // agree to a few units of rounding.
        for &t in &[2.7255, 300.0, 3000.0, 5778.0, 40_000.0] {
            for &g in &[0.01, 0.5, 1.0, 2.0, 389.0, 1e4] {
                for &nu in &[1e9, 1e11, 1e13, 3e14, 5.45e14, 1e15, 3e15] {
                    let lhs = g * g * g * planck_per_frequency(nu / g, t);
                    let rhs = planck_per_frequency(nu, g * t);
                    if rhs == 0.0 {
                        assert_eq!(lhs, 0.0);
                        continue;
                    }
                    assert!(
                        (lhs / rhs - 1.0).abs() < 1e-13,
                        "T {t} g {g} nu {nu}: {lhs} {rhs}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_planck_per_wavelength_is_per_frequency_times_c_over_lambda_squared() {
        for &t in &[1000.0, 5778.0] {
            for &l in &[300e-9, 555e-9, 2e-6] {
                let a = planck_per_wavelength(l, t);
                let b = planck_per_frequency(LIGHT_C / l, t) * LIGHT_C / (l * l);
                assert!((a / b - 1.0).abs() < 1e-13);
            }
        }
    }

    /// The integral of B_nu over frequency by Simpson's rule in u = h nu / k T on [0, 60] (the
    /// tail beyond is e^-60 of the whole), 60000 steps.
    fn bolometric(t: f64) -> f64 {
        let n = 60_000;
        let du = 60.0 / n as f64;
        let scale = BOLTZMANN_K * t / PLANCK_H;
        let f = |u: f64| {
            if u == 0.0 {
                0.0
            } else {
                planck_per_frequency(u * scale, t) * scale
            }
        };
        let mut s = f(0.0) + f(60.0);
        for i in 1..n {
            s += f(i as f64 * du) * if i % 2 == 1 { 4.0 } else { 2.0 };
        }
        s * du / 3.0
    }

    #[test]
    fn test_the_bolometric_radiance_is_sigma_t4_over_pi_and_scales_as_g_to_the_fourth() {
        // integral B_nu dnu = (2 k^4 T^4 / h^3 c^2) integral u^3 / (e^u - 1) du, and that integral
        // is pi^4 / 15 = 6.4939: sigma T^4 / pi. Simpson with 60000 steps on a function this
        // smooth is exact to ~1e-15; the tolerance is rounding.
        for &t in &[2.7255, 5778.0, 1e5] {
            let b = bolometric(t);
            assert!(
                (b / (SIGMA * t.powi(4) / std::f64::consts::PI) - 1.0).abs() < 1e-12,
                "{t}"
            );
        }
        // The shifted blackbody's bolometric radiance, g^4 times the unshifted one's.
        for &g in &[0.5, 2.0, 389.0] {
            let r = bolometric(g * 5778.0) / bolometric(5778.0);
            assert!((r / g.powi(4) - 1.0).abs() < 1e-12, "g {g}: {r}");
        }
        // And sigma itself against CODATA 2018's 5.670 374 419... e-8 W m^-2 K^-4.
        assert!((SIGMA / 5.670_374_419e-8 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_the_one_nanometre_rule_is_within_its_stated_error_of_the_continuous_integral() {
        // Reference: the table interpolated linearly between rows (the CIE's stated
        // interpolation), times the Planck function evaluated exactly, by Simpson's rule on 16
        // sub-steps per nanometre (Simpson's own error at 16 is below 1e-8 here). The stated
        // bounds: 2e-4 of each tristimulus value and 3e-6 in chromaticity for T >= 1000 K,
        // 5e-4 of X and Y from 150 K to 1000 K.
        let reference = |t: f64| {
            let n = 16;
            let mut xyz = [0.0; 3];
            for k in 0..ROWS - 1 {
                for i in 0..=2 * n {
                    let f = i as f64 / (2 * n) as f64;
                    let w = if i == 0 || i == 2 * n {
                        1.0
                    } else if i % 2 == 1 {
                        4.0
                    } else {
                        2.0
                    };
                    let l = (wavelength_nm(k) + f) * 1e-9;
                    let b = planck_per_wavelength(l, t) * w / (6.0 * n as f64) * 1e-9;
                    for c in 0..3 {
                        xyz[c] += b * (CMF[k][c] * (1.0 - f) + CMF[k + 1][c] * f);
                    }
                }
            }
            xyz
        };
        for &t in &[150.0, 300.0, 700.0, 1000.0, 2000.0, 5778.0, 1e4, 1e5, 1e7] {
            let a = blackbody_xyz(t);
            let b = reference(t);
            let bound = if t >= 1000.0 { 2e-4 } else { 5e-4 };
            for c in 0..2 {
                assert!(
                    (a[c] / b[c] - 1.0).abs() < bound,
                    "T {t} channel {c}: {}",
                    a[c] / b[c] - 1.0
                );
            }
            if t >= 1000.0 {
                assert!(
                    (a[2] / b[2] - 1.0).abs() < bound,
                    "T {t} Z: {}",
                    a[2] / b[2] - 1.0
                );
                let (p, q) = (xy(a), xy(b));
                assert!((p.0 - q.0).hypot(p.1 - q.1) < 3e-6, "T {t}: {p:?} {q:?}");
            }
        }
    }

    #[test]
    fn test_the_logarithmic_sum_agrees_with_the_direct_sum_and_its_slope_with_a_difference() {
        for &t in &[100.0, 1000.0, 5778.0, 1e6] {
            let (ln_xyz, slope) = ln_blackbody_xyz(t);
            let xyz = blackbody_xyz(t);
            let h: f64 = 1e-4;
            let (up, _) = ln_blackbody_xyz(t * h.exp());
            let (down, _) = ln_blackbody_xyz(t * (-h).exp());
            for c in 0..3 {
                assert!((ln_xyz[c] - xyz[c].ln()).abs() < 1e-12, "T {t} c {c}");
                // Central difference error h^2 f''' / 6 ~ 1e-8 f'''; f''' ~ slope here.
                let numeric = (up[c] - down[c]) / (2.0 * h);
                assert!(
                    (numeric - slope[c]).abs() < 1e-6 * slope[c].max(1.0),
                    "T {t} c {c}"
                );
            }
        }
    }

    #[test]
    fn test_illuminant_a_is_the_blackbody_at_2848_k_under_the_old_c2() {
        // CIE 015:2018 and ISO/CIE 11664-2 give illuminant A the chromaticity (0.44757, 0.40745)
        // (computed at 5 nm; the CIE's own 1 nm spectrum of A summed against this table gives
        // (0.447574, 0.407439), the 1e-5 difference being the step). The blackbody at 2848 K
        // with c2 = 1.435e-2 is that spectrum to the 6 digits the CIE rounds A to.
        let (x, y) = xy(blackbody_xyz_c2(T_ILLUMINANT_A, C2_ILLUMINANT_A));
        assert!(
            (x - 0.44757).abs() < 1.5e-5 && (y - 0.40745).abs() < 1.5e-5,
            "{x}, {y}"
        );
        // The same spectrum under the exact c2 is T = 2848 x c2 / 1.435e-2 = 2855.496 K ...
        let t = T_ILLUMINANT_A * C2 / C2_ILLUMINANT_A;
        assert!((t - 2855.496).abs() < 1e-3);
        let (x2, y2) = xy(blackbody_xyz(t));
        assert!((x2 - x).abs() < 1e-12 && (y2 - y).abs() < 1e-12);
        // ... and a blackbody at the rounded 2856 K under the exact c2 is 4e-5 away in x: the
        // published "2856 K" is the right temperature only to its four digits.
        let (x3, _) = xy(blackbody_xyz(2856.0));
        assert!((x3 - x - (-3.84e-5)).abs() < 1e-6, "{}", x3 - x);
    }

    #[test]
    fn test_the_chromaticities_at_5778_k_and_6504_k_agree_with_an_independent_implementation() {
        // colour-science 0.4.7 (colour.sd_blackbody on 360-830 nm at 1 nm, with its c2 =
        // 1.4388e-2, and colour.sd_to_XYZ, method "Integration", against its copy of the same CIE
        // table), run 2026-09-28: 5778 K -> (0.32643218, 0.33572479), 6504 K -> (0.31346516,
        // 0.32356915). Its temperatures are restated for the exact c2, T x C2 / 1.4388e-2. This is
        // a check of the arithmetic against another program, not against a published standard: the
        // CIE publishes no table of the locus. What differs is the sum's end weights, ~1e-7.
        for &(t, x, y) in &[
            (5778.0, 0.326_432_18, 0.335_724_79),
            (6504.0, 0.313_465_16, 0.323_569_15),
        ] {
            let (p, q) = xy(blackbody_xyz(t * C2 / 1.4388e-2));
            assert!((p - x).abs() < 5e-7 && (q - y).abs() < 5e-7, "{t}: {p} {q}");
        }
        // Under the exact c2 at the nominal temperatures, for the record:
        let (p, q) = xy(blackbody_xyz(5778.0));
        assert!(
            (p - 0.326_430).abs() < 1e-6 && (q - 0.335_723).abs() < 1e-6,
            "{p} {q}"
        );
    }

    #[test]
    fn test_the_colour_of_a_cold_body_is_the_colour_of_the_tables_last_row() {
        // The Wien tail: at low T the sum is dominated by 830 nm, whose chromaticity is
        // (0.734690, 0.265310).
        let last = CMF[ROWS - 1];
        let s = last[0] + last[1] + last[2];
        let (x, y) = xy(blackbody_xyz(100.0));
        assert!(
            (x - last[0] / s).abs() < 1e-6 && (y - last[1] / s).abs() < 1e-6,
            "{x} {y}"
        );
        // And ln Y falls as -c2 / (830 nm T): the slope of ln Y against 1/T tends to -c2 / 830 nm.
        // The row at 829 nm is smaller than the last by exp(-c2 (1/829 - 1/830) / nm / T) =
        // e^(-20.9 K / T): a third of it at 20 K, which bends the slope by 1e-3, and 1.5 % at
        // 5 K, which leaves 2e-5.
        let (a, _) = ln_blackbody_xyz(5.0);
        let (b, _) = ln_blackbody_xyz(6.0);
        let slope = (a[1] - b[1]) / (1.0 / 5.0 - 1.0 / 6.0);
        assert!(
            (slope / (-C2 / 830e-9) - 1.0).abs() < 1e-4,
            "{}",
            slope / (-C2 / 830e-9)
        );
    }

    #[test]
    fn test_at_high_temperature_the_radiance_grows_as_t_and_the_colour_is_the_lambda_minus_4_point()
    {
        // Rayleigh-Jeans: B_lambda -> 2 c k T / lambda^4, so X, Y, Z -> T times the integrals of
        // lambda^-4 against the functions, whose chromaticity is the limit of the locus. From the
        // 1 nm table, 360 to 830 nm: (0.239877, 0.234038). The value often quoted, (0.2399,
        // 0.2342), is that of the table cut to 380-780 nm, (0.239915, 0.234173): lambda^-4
        // weights the short end, and 360-380 nm moves y by 1.4e-4.
        let rj = |lo: f64, hi: f64| {
            let mut s = [0.0; 3];
            for (k, row) in CMF.iter().enumerate() {
                let l = wavelength_nm(k);
                if l >= lo && l <= hi {
                    // The trapezoid rule's half weights at the ends, as `tristimulus_of` has.
                    let w = if l == lo || l == hi { 0.5 } else { 1.0 };
                    for c in 0..3 {
                        s[c] += w * row[c] * l.powi(-4);
                    }
                }
            }
            xy(s)
        };
        let (x, y) = rj(360.0, 830.0);
        assert!(
            (x - 0.239_877).abs() < 1e-6 && (y - 0.234_038).abs() < 1e-6,
            "{x} {y}"
        );
        let (x2, y2) = rj(380.0, 780.0);
        assert!(
            (x2 - 0.2399).abs() < 5e-5 && (y2 - 0.2342).abs() < 5e-5,
            "{x2} {y2}"
        );
        // The blackbody reaches it: at 1e9 K the first correction is x / 2 ~ 1e-5 relative, and
        // it cancels in chromaticity to 1e-6.
        let (p, q) = xy(blackbody_xyz(1e9));
        assert!((p - x).abs() < 1e-6 && (q - y).abs() < 1e-6, "{p} {q}");
        // And the radiance in a fixed band grows as T. With Y = a1 T - a0 + ..., Y(2T) / Y(T) =
        // 2 + a0 / (a1 T) + ..., and a0 / a1 ~ c2 / (2 lambda) ~ 1.3e4 K: 2.000132 at 1e8 K.
        let r = blackbody_xyz(2e8)[1] / blackbody_xyz(1e8)[1];
        assert!(r > 2.0 && r - 2.0 < 1.5e-4, "{r}");
        let r = blackbody_xyz(2e10)[1] / blackbody_xyz(1e10)[1];
        assert!(r > 2.0 && r - 2.0 < 1.5e-6, "{r}");
    }

    #[test]
    fn test_the_high_temperature_series_is_the_direct_sum_above_a_million_kelvin() {
        // Terms omitted are of relative size x^6 / 30240 <= 2e-13 at 1e6 K.
        let a = high_temperature_coefficients();
        for &t in &[1e6, 1e7, 1e9, 1e12] {
            let direct = blackbody_xyz(t);
            for c in 0..3 {
                let series = a[c][0] * t - a[c][1] + a[c][2] / t - a[c][3] / t.powi(3);
                assert!((series / direct[c] - 1.0).abs() < 1e-11, "T {t} c {c}");
            }
        }
    }
}
