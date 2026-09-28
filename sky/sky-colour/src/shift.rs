//! From a pixel of the map and a shift g to the colour the observer sees.
//!
//! # The rule
//!
//! For a pixel with tristimulus values P = (X, Y, Z) whose chromaticity implies the temperature T
//! (see `temperature`), the shifted pixel is
//!
//!     S_c = P_c Q_c(T, g),      Q_c(T, g) = B_c(g T) / B_c(T),      c = X, Y, Z,
//!
//! where B_c(T) is the tristimulus value c of a blackbody at T, and the shifted pixel S is returned
//! as linear BT.709 RGB. In words: the pixel is taken to be the blackbody at T times a tint (P_c /
//! B_c(T), whatever is left over), and the tint is kept while the blackbody is replaced by the one
//! at g T, which is what the shift does to a blackbody (see `planck`).
//!
//! What it satisfies, and why:
//!
//! - *Exact for a blackbody pixel*, of any brightness: if P = k B(T) then T is implied exactly
//!   (Duv = 0) and S = k B(g T), the shifted light itself. Brightness and colour both come out of
//!   the one ratio: the visible factor is Q_Y, and there is no separate g^4.
//! - *It never divides by a non-positive number.* B_c(T) is an integral of a positive spectrum
//!   against a non-negative matching function that is not zero everywhere, so it is positive for
//!   every T > 0 (the logarithms are tabulated even where the value itself underflows, see
//!   `table`). This is why the ratio is taken in XYZ and not in RGB: a blackbody's BT.709
//!   coordinates are not all positive (blue is negative below 1905 K, green below 980 K, because
//!   the locus leaves the gamut there), and a ratio of RGB channels would divide by zero at 1905 K
//!   and change sign below it. Taken in XYZ, the ratio is positive, finite and smooth for all T and
//!   g > 0.
//! - *Continuous* in the pixel's colour (T is, see `temperature`; Q is smooth in T; P enters
//!   linearly) and in g (Q is smooth in g); linear in the pixel's brightness (T depends on
//!   chromaticity only), so it goes to black continuously as the pixel does, with no special case
//!   at black other than the exact 0 for exactly 0.
//! - *The identity at g = 1*: Q = 1 exactly there, but a round trip RGB -> XYZ -> RGB is not
//!   exact to the bit, so [`Model::shift`] returns its input unchanged when g == 1.0, and
//!   [`needs_shift`] lets the renderer skip the model altogether.
//!
//! # What else was considered
//!
//! - *The ratio in RGB*, the first candidate: exact for blackbodies too, but divides by a
//!   channel that is zero at 1905 K (blue) and 980 K (green), as above. Rejected.
//! - *The ratio in a cone space* (Hunt-Pointer-Estevez LMS), the space in which a von Kries
//!   scaling is usually made: it is also positive (its three functions are non-negative on
//!   360-830 nm; checked with the HPE matrix on this table, outside the crate). On the
//!   two-temperature mixture of the crate's documentation it did no better than XYZ: its colour
//!   errors were within 0.002 u'v' of XYZ's for g from 0.5 to 100 (0.006 at g = 0.25), better
//!   at some g and worse at others, against errors of 0.01 to 0.1, and its brightness errors
//!   the same to 0.1 %. XYZ's positivity needs no check. The choice of space matters only for
//!   the tint, which is exactly the part the model does not know. The RGB ratio, where it is
//!   defined, was no better either.
//! - *Dropping the tint*: S = Y(P) B(gT) / Y(B(T)), the blackbody at g T at the pixel's
//!   luminance. Simpler, but it is not the identity at g = 1 for any pixel off the locus (every
//!   grey star would turn slightly pink at g = 1.0001). Rejected.
//! - *A richer spectral family*: a blackbody times a power law lambda^-beta has two parameters,
//!   (T, beta), which two chromaticity coordinates fix, and it shifts exactly into the same family
//!   ((T, beta) -> (g T, beta), times g^-beta). It would model the dust lanes' reddening, which is
//!   close to a power law, better than a tint does. Not done: the owner chose the blackbody model,
//!   it needs a two-dimensional table, and it cannot reach saturated colours. Recorded here as the
//!   natural next step if the diffuse glow's colour matters.
//!
//! # Units and the table's ends
//!
//! Q is formed as exp(ln B(g T) - ln B(T)), from the table. When g T is below 10 K (the table's
//! floor) the shift returns black: the implied T is at least 300 K, so the ratio is then below
//! exp(-c2 / 830 nm x (1/10 K - 1/300 K)) = e^-1675, which is zero in f64. That is the exact
//! answer to the arithmetic's precision, not a clip. When T is infinite (a colour at or beyond the
//! locus's hot end) Q = g, the Rayleigh-Jeans limit, in which the visible radiance is
//! proportional to T.

use crate::colorimetry::{luminance, rgb_to_xyz, xyz_to_rgb};
use crate::table::{T_TABLE_MIN, Table};
use crate::temperature::{Implied, Locus, T_LOCUS_MIN, chromaticity};

/// Whether a shift g changes anything: false exactly at g == 1.0, where [`Model::shift`] returns
/// its input unchanged, so a renderer can skip the model and keep a film of flat space at rest
/// identical to the bit.
#[inline]
pub fn needs_shift(g: f64) -> bool {
    g != 1.0
}

/// The blackbody tables and the locus, built once (about 30 ms in a release build).
#[derive(Debug, Clone)]
pub struct Model {
    pub(crate) table: Table,
    pub(crate) locus: Locus,
}

impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}

impl Model {
    pub fn new() -> Self {
        let table = Table::new();
        let locus = Locus::new(&table);
        Self { table, locus }
    }

    /// The tristimulus values of a blackbody at `kelvin` > 0, W m^-2 sr^-1 (so that 683.002 Y is
    /// the luminance in cd m^-2), from the table: within 3e-9 of the direct sum above 300 K. They
    /// underflow to 0 below about 24 K.
    pub fn blackbody_xyz(&self, kelvin: f64) -> [f64; 3] {
        self.table.ln_xyz(kelvin).map(f64::exp)
    }

    /// The linear BT.709 RGB of a blackbody at `kelvin`, in the same absolute units (its luminance
    /// is Y in W m^-2 sr^-1). Channels are negative where the blackbody is outside the gamut.
    pub fn blackbody_rgb(&self, kelvin: f64) -> [f64; 3] {
        xyz_to_rgb(self.blackbody_xyz(kelvin))
    }

    /// Q_c(T, g) = B_c(g T) / B_c(T) for c = X, Y, Z: what a shift g does to each tristimulus value
    /// of a blackbody at `kelvin` (which may be infinite, where Q = g). Positive and finite for
    /// every T > 0 and g > 0; exactly 1 at g == 1.0. For T and g T both above 10 K it comes from
    /// the table; otherwise from the direct sum (slow).
    pub fn xyz_ratio(&self, kelvin: f64, g: f64) -> [f64; 3] {
        if g == 1.0 {
            return [1.0; 3];
        }
        if kelvin.is_infinite() {
            return [g; 3];
        }
        let a = self.table.ln_xyz(kelvin);
        let b = self.table.ln_xyz(g * kelvin);
        std::array::from_fn(|c| (b[c] - a[c]).exp())
    }

    /// The visible-band factor: the luminance of a blackbody at `kelvin` seen with shift g, over
    /// its luminance unshifted, Y(g T) / Y(T). The eye's counterpart of the bolometric g^4.
    pub fn visible_factor(&self, kelvin: f64, g: f64) -> f64 {
        self.xyz_ratio(kelvin, g)[1]
    }

    /// The temperature the colour of a linear BT.709 pixel implies, with its Duv. `None` for
    /// black, and for a pixel that is not the colour of any light (a tristimulus value below zero)
    /// or not finite.
    pub fn implied_temperature(&self, rgb: [f64; 3]) -> Option<Implied> {
        self.implied_temperature_xyz(rgb_to_xyz(rgb))
    }

    /// As [`Model::implied_temperature`], from tristimulus values.
    pub fn implied_temperature_xyz(&self, xyz: [f64; 3]) -> Option<Implied> {
        let xyz = light(xyz)?;
        if xyz == [0.0; 3] {
            return None;
        }
        Some(self.locus.implied(chromaticity(xyz)))
    }

    /// The linear BT.709 RGB the observer sees where the map shows `rgb` and the shift is `g`,
    /// under the blackbody model (see the crate's documentation). Not clipped and not brought
    /// into gamut: channels can be negative (outside BT.709) or far above 1.
    ///
    /// - g == 1.0: `rgb` itself, to the bit.
    /// - black: black, for any g. g == 0: black, the limit of an infinite redshift.
    /// - NaN in all three channels, "not known", for a g that is negative, infinite or NaN, and
    ///   for a pixel that is not light (not finite, or a tristimulus value below zero by more
    ///   than rounding). A pixel of the map, every channel in [0, 1], is always light.
    pub fn shift(&self, rgb: [f64; 3], g: f64) -> [f64; 3] {
        if g == 1.0 {
            return rgb;
        }
        let xyz = match light(rgb_to_xyz(rgb)) {
            Some(xyz) if g.is_finite() && g >= 0.0 => xyz,
            _ => return [f64::NAN; 3],
        };
        if xyz == [0.0; 3] || g == 0.0 {
            return [0.0; 3];
        }
        let implied = self.locus.implied(chromaticity(xyz));
        self.shift_xyz_at(xyz, implied.kelvin, g)
    }

    /// [`Model::shift`] for the renderer's f32 texels. The input is returned unchanged at g == 1.0.
    pub fn shift_f32(&self, rgb: [f32; 3], g: f64) -> [f32; 3] {
        if g == 1.0 {
            return rgb;
        }
        self.shift(rgb.map(f64::from), g).map(|c| c as f32)
    }

    /// The shift of a pixel whose implied temperature is already known (for a renderer that caches
    /// it per texel: it depends on the texel's chromaticity only). `rgb` must be light, as for
    /// [`Model::shift`]; `kelvin` at least 300 K or infinite, as [`Model::implied_temperature`]
    /// returns.
    pub fn shift_at(&self, rgb: [f64; 3], kelvin: f64, g: f64) -> [f64; 3] {
        if g == 1.0 {
            return rgb;
        }
        match light(rgb_to_xyz(rgb)) {
            Some(xyz) if g.is_finite() && g >= 0.0 => {
                if xyz == [0.0; 3] || g == 0.0 {
                    [0.0; 3]
                } else {
                    self.shift_xyz_at(xyz, kelvin, g)
                }
            }
            _ => [f64::NAN; 3],
        }
    }

    fn shift_xyz_at(&self, xyz: [f64; 3], kelvin: f64, g: f64) -> [f64; 3] {
        debug_assert!(kelvin >= T_LOCUS_MIN * (1.0 - 1e-12));
        if g * kelvin < T_TABLE_MIN {
            // Below e^-1675 of the input: zero in f64 (module comment).
            return [0.0; 3];
        }
        let q = self.xyz_ratio(kelvin, g);
        xyz_to_rgb([xyz[0] * q[0], xyz[1] * q[1], xyz[2] * q[2]])
    }

    /// The linear BT.709 RGB, in map units, of a uniform blackbody background of temperature
    /// `kelvin` (the cosmic microwave background: 2.7255 K) seen with shift g, where `scale`
    /// converts absolute radiance (W m^-2 sr^-1, this crate's unit) to the map's units.
    ///
    /// A shifted blackbody is exactly the blackbody at g T, so this is `scale` times the
    /// blackbody's RGB at g T: no g^3 or g^4 appears. `scale` is not known yet: the maps carry no
    /// photometric calibration (sky/maps/README.md, "Absolute scale"). To find it one needs the
    /// absolute surface brightness of some part of the map, for example the V-band magnitude of a
    /// bright star that is not clipped, set against the sum of its pixels times the pixel's solid
    /// angle, or a published surface brightness of the Milky Way's diffuse light at a stated
    /// place. Until then the renderer cannot add this term honestly, and should not.
    pub fn shifted_background_rgb(&self, kelvin: f64, g: f64, scale: f64) -> [f64; 3] {
        self.blackbody_rgb(g * kelvin).map(|c| c * scale)
    }
}

/// The tristimulus values if they are those of some light: finite, and none below zero by more
/// than rounding (a real spectrum has X, Y, Z >= 0, since the matching functions are). Values
/// below zero by rounding alone are set to 0.
#[inline]
fn light(xyz: [f64; 3]) -> Option<[f64; 3]> {
    if !xyz.iter().all(|c| c.is_finite()) {
        return None;
    }
    let size = xyz[0].abs() + xyz[1].abs() + xyz[2].abs();
    if xyz.iter().any(|&c| c < -1e-12 * size) {
        return None;
    }
    Some(xyz.map(|c| c.max(0.0)))
}

/// Brings a linear BT.709 colour into the display's range, [0, 1] in every channel, for showing,
/// keeping its luminance where the display can and then its hue.
///
/// Clipping is a limit of the display, not an unknown: a colour outside [0, 1]^3 is known, the
/// display cannot show it, and this function chooses what to give up. It is not part of the
/// model and is not applied by [`Model::shift`]. Apply it after the exposure, to values in which
/// 1 is the display's white.
///
/// The rule: with Y the colour's luminance,
///
/// - Y >= 1: white (1, 1, 1), the brightest the display has; nothing of the colour survives;
/// - Y <= 0: black (no shifted colour of this crate has Y <= 0, since its Y is a positive
///   multiple of the input's);
/// - otherwise the colour is moved toward the grey of the same luminance, (Y, Y, Y), just far
///   enough that every channel is in [0, 1]: out = Y + s (in - Y) with the largest s in [0, 1]
///   that allows it. Inside the range s = 1 and the colour is returned unchanged.
///
/// Moving toward grey at constant Y keeps the luminance exactly, and keeps the chromaticity on
/// the straight line from the white point through the colour: the dominant wavelength, the CIE's
/// measure of hue, is unchanged. Perceived hue is not quite constant along such lines (the
/// Abney effect: a desaturated blue looks slightly purple), which is the price of an exact and
/// simple rule. Luminance first because brightness is what the shift changes most and what the
/// viewer is told about; a clip channel by channel, as the renderer did, keeps neither: a
/// saturated blue clipped that way loses most of its luminance, and a bright orange turns yellow.
///
/// Continuous in the colour, including at Y -> 1 (s -> 0, the colour -> white). A NaN in gives
/// NaN out, so that a pixel that is not known stays marked.
pub fn into_display_gamut(rgb: [f64; 3]) -> [f64; 3] {
    if rgb.iter().any(|c| c.is_nan()) {
        return [f64::NAN; 3];
    }
    let y = luminance(rgb);
    if y >= 1.0 {
        return [1.0; 3];
    }
    if y <= 0.0 {
        return [0.0; 3];
    }
    let mut s = 1.0f64;
    for c in rgb {
        if c < 0.0 {
            s = s.min(y / (y - c));
        } else if c > 1.0 {
            s = s.min((1.0 - y) / (c - y));
        }
    }
    if s >= 1.0 {
        return rgb;
    }
    // Rounding can leave a channel a hair outside; the clamp only ever moves it by ~1e-16.
    rgb.map(|c| (y + s * (c - y)).clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::colorimetry::{delta_u_prime_v_prime, xy};
    use crate::model;

    fn close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
        let size = a.iter().chain(b.iter()).fold(0.0f64, |m, c| m.max(c.abs()));
        a.iter()
            .zip(b.iter())
            .all(|(p, q)| (p - q).abs() <= tol * size)
    }

    /// The BT.709 RGB of a blackbody, scaled so that its largest channel is 1: a pixel as the map
    /// would draw a star of that temperature.
    fn star(t: f64) -> [f64; 3] {
        let rgb = model().blackbody_rgb(t);
        let m = rgb[0].max(rgb[1]).max(rgb[2]);
        rgb.map(|c| c / m)
    }

    #[test]
    fn test_the_shift_is_the_identity_to_the_bit_at_g_equal_one() {
        let m = model();
        let pixels = [
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [0.25, 0.5, 0.75],
            [1.0, 0.0, 0.0],
            [3e-8, 1e-300, 0.0],
            [0.123_456_789_012_345_67, 0.987_654_321, 0.5],
        ];
        for p in pixels {
            let out = m.shift(p, 1.0);
            assert!(
                out.iter()
                    .zip(p.iter())
                    .all(|(a, b)| a.to_bits() == b.to_bits()),
                "{p:?}"
            );
            let p32 = p.map(|c| c as f32);
            let out32 = m.shift_f32(p32, 1.0);
            assert!(
                out32
                    .iter()
                    .zip(p32.iter())
                    .all(|(a, b)| a.to_bits() == b.to_bits())
            );
        }
        assert!(!needs_shift(1.0) && needs_shift(1.0 + f64::EPSILON) && needs_shift(0.5));
        assert_eq!(m.xyz_ratio(5778.0, 1.0), [1.0; 3]);
    }

    #[test]
    fn test_a_blackbody_pixel_shifts_to_the_blackbody_at_g_times_t() {
        // Exact in the model; the tolerance is the table's and the temperature search's.
        let m = model();
        for &t in &[2000.0, 3000.0, 5778.0, 10_000.0, 30_000.0] {
            for &g in &[0.125, 0.5, 0.9, 1.1, 2.0, 10.0, 389.0] {
                let pixel = m.blackbody_rgb(t).map(|c| c * 1e-3);
                let want = m.blackbody_rgb(g * t).map(|c| c * 1e-3);
                let got = m.shift(pixel, g);
                assert!(close(got, want, 1e-7), "T {t} g {g}: {got:?} {want:?}");
            }
        }
    }

    #[test]
    fn test_g_then_one_over_g_returns_a_blackbody_pixel() {
        // The first shift lands on the locus at g T, whose colour implies g T, and 1 / g returns
        // to T. Tolerance 1e-6 of the largest channel: at g T near 4e5 K the locus has nearly
        // stopped moving (d(u, v) / d ln T ~ 0.005), so the table's 3e-9 becomes 1e-7 in the
        // temperature found there, and returning to 1000 K multiplies that by d ln Q / d ln T
        // ~ c2 / (lambda T) ~ 26: measured 1.5e-7 at T = 1000 K, g = 389, and ~1e-9 elsewhere.
        // It holds while g T is at least ~500 K:
        // below that the locus has all but reached its end, the colour of 830 nm (it moves by
        // 0.013 in x from 500 K to 0 K and 4e-4 from 300 K), so the colour no longer tells the
        // temperature and the round trip cannot return; the light is then below 1e-19 of a
        // 5778 K body's.
        let m = model();
        for &t in &[1000.0, 2500.0, 5778.0, 10_000.0, 40_000.0] {
            for &g in &[0.2, 0.5, 2.0, 10.0, 100.0, 389.0] {
                if g * t < 500.0 {
                    continue;
                }
                let p = star(t);
                let back = m.shift(m.shift(p, g), 1.0 / g);
                assert!(close(back, p, 1e-6), "T {t} g {g}: {back:?} {p:?}");
            }
        }
    }

    #[test]
    fn test_the_shift_is_continuous_in_g_through_one() {
        // Q is smooth in g with Q(1) = 1, so shift(p, 1 +- e) - p = O(e). The check is that the
        // change is of the size of e times a factor of order c2 / lambda T (the slope of ln Y
        // in ln T), not a jump: for e = 1e-9 below 1e-7 of the pixel.
        let m = model();
        for p in [
            [1.0, 1.0, 1.0],
            [1.0, 0.6, 0.3],
            [0.3, 0.5, 1.0],
            [0.9, 0.05, 0.8],
        ] {
            for e in [1e-9, -1e-9] {
                let out = m.shift(p, 1.0 + e);
                assert!(close(out, p, 1e-7), "{p:?} {e}: {out:?}");
            }
        }
    }

    #[test]
    fn test_the_shift_is_continuous_across_grey_the_gamut_edge_and_the_blend_band() {
        // Along lines through the awkward places the output changes by an amount proportional to
        // the step: halving the step halves the largest change between neighbours.
        let m = model();
        let lines: [([f64; 3], [f64; 3]); 4] = [
            ([0.9, 1.0, 1.1], [1.1, 1.0, 0.9]),   // through grey, (1, 1, 1)
            ([1.0, 0.5, 0.02], [1.0, 0.5, -0.0]), // to the gamut's edge, blue -> 0
            ([1.0, 0.2, 0.5], [1.0, 0.0, 0.9]),   // below the locus, through the blend band
            ([0.0, 0.3, 1.0], [0.3, 0.0, 1.0]),   // around the blue corner, beyond the hot end
        ];
        for g in [0.5, 2.0, 30.0] {
            for (a, b) in lines {
                let step = |n: usize| {
                    let mut worst = 0.0f64;
                    let mut prev: Option<[f64; 3]> = None;
                    for i in 0..=n {
                        let f = i as f64 / n as f64;
                        let p: [f64; 3] = std::array::from_fn(|c| a[c] + f * (b[c] - a[c]));
                        let out = m.shift(p, g);
                        if let Some(q) = prev {
                            for c in 0..3 {
                                worst = worst.max((out[c] - q[c]).abs());
                            }
                        }
                        prev = Some(out);
                    }
                    worst
                };
                let (coarse, fine) = (step(200), step(400));
                assert!(
                    fine < 0.6 * coarse,
                    "g {g} {a:?}->{b:?}: {coarse:e} {fine:e}"
                );
            }
        }
    }

    #[test]
    fn test_a_dark_pixel_scales_with_its_brightness_down_to_black() {
        // T depends on chromaticity only, so shift(k p) = k shift(p): the output goes to black
        // continuously however dark the pixel, including in f64's subnormal range.
        let m = model();
        let p = [0.8, 0.5, 0.3];
        let base = m.shift(p, 3.0);
        for k in [1e-3, 1e-100, 1e-300, 1e-310] {
            let out = m.shift(p.map(|c| c * k), 3.0);
            let tol = if k < 1e-300 { 1e-6 } else { 1e-12 };
            assert!(close(out, base.map(|c| c * k), tol), "{k}: {out:?}");
        }
        assert_eq!(m.shift([0.0; 3], 3.0), [0.0; 3]);
        assert_eq!(m.shift([0.0; 3], 1e-9), [0.0; 3]);
        assert_eq!(m.shift(p, 0.0), [0.0; 3]);
    }

    #[test]
    fn test_what_is_not_light_or_not_a_shift_comes_back_as_nan() {
        let m = model();
        for (p, g) in [
            ([1.0, 1.0, 1.0], -1.0),
            ([1.0, 1.0, 1.0], f64::INFINITY),
            ([1.0, 1.0, 1.0], f64::NAN),
            ([f64::NAN, 1.0, 1.0], 2.0),
            ([-1.0, -1.0, -1.0], 2.0),
        ] {
            assert!(m.shift(p, g).iter().all(|c| c.is_nan()), "{p:?} {g}");
        }
        // A shifted colour with a negative channel is still light, and can be shifted again.
        let low = m.blackbody_rgb(1200.0);
        assert!(low[2] < 0.0);
        assert!(m.shift(low, 2.0).iter().all(|c| c.is_finite()));
    }

    #[test]
    fn test_the_ratio_is_positive_and_finite_for_every_temperature_and_shift() {
        let m = model();
        let mut t = super::T_LOCUS_MIN;
        while t < 1e12 {
            for g in [1e-3, 0.1, 0.5, 2.0, 389.0, 1e6] {
                if g * t < T_TABLE_MIN {
                    continue;
                }
                let q = m.xyz_ratio(t, g);
                assert!(
                    q.iter().all(|c| c.is_finite() && *c >= 0.0),
                    "T {t} g {g}: {q:?}"
                );
            }
            t *= 1.37;
        }
        assert_eq!(m.xyz_ratio(f64::INFINITY, 7.0), [7.0; 3]);
        // Near infinity the ratio tends to g continuously: at 1e12 K it is g to 1e-7.
        let q = m.xyz_ratio(1e12, 7.0);
        assert!(q.iter().all(|c| (c / 7.0 - 1.0).abs() < 1e-7), "{q:?}");
    }

    #[test]
    fn test_below_ten_kelvin_after_the_shift_the_true_ratio_underflows_to_zero() {
        // The shift returns black when g T < 10 K; the direct sum confirms that the ratio there
        // is zero in f64 for the lowest implied temperature, 300 K.
        let m = model();
        let q = m.xyz_ratio(super::T_LOCUS_MIN, 9.99 / super::T_LOCUS_MIN);
        assert!(q.iter().all(|c| *c == 0.0), "{q:?}");
        assert_eq!(m.shift([1.0, 0.1, 0.0], 1e-3), [0.0; 3]);
    }

    #[test]
    fn test_the_visible_factors_are_the_documented_ones() {
        // The table in the crate's documentation, Y(g T) / Y(T), to the four digits printed.
        let m = model();
        let rows: [(f64, [f64; 7]); 3] = [
            (
                3000.0,
                [
                    1.487e-23, 3.980e-11, 2.561e-4, 72.52, 4023.0, 6.172e4, 2.481e5,
                ],
            ),
            (
                5778.0,
                [2.026e-13, 2.243e-6, 1.180e-2, 10.54, 159.9, 1981.0, 7839.0],
            ),
            (
                10_000.0,
                [2.333e-8, 4.241e-4, 6.924e-2, 4.717, 42.81, 483.6, 1900.0],
            ),
        ];
        let gs = [0.125, 0.25, 0.5, 2.0, 10.0, 100.0, 389.0];
        for (t, want) in rows {
            for (g, w) in gs.iter().zip(want.iter()) {
                let f = m.visible_factor(t, *g);
                assert!(
                    (f / w - 1.0).abs() < 1e-3,
                    "T {t} g {g}: {f:e} against {w:e}"
                );
            }
        }
        // The Sun at g = 389: of the order of 10^4 in the visible against 2.3e10 bolometric.
        let f = m.visible_factor(5778.0, 389.0);
        assert!(f > 5e3 && f < 2e4);
    }

    #[test]
    fn test_cool_bodies_fall_below_what_a_display_shows_beside_a_star_as_documented() {
        // Y(T) / Y(5778 K), the numbers the crate's documentation gives, to 1 %; and a 5778 K
        // blackbody's luminance, 683.002 Y = 1.852e9 cd m^-2 (the solar disc's mean, with limb
        // darkening and lines, is about 1.6e9).
        let m = model();
        let sun = m.blackbody_xyz(5778.0)[1];
        assert!((crate::planck::K_M * sun / 1.852e9 - 1.0).abs() < 1e-3);
        for (t, want) in [
            (2000.0, 2.504e-4),
            (1400.0, 1.316e-6),
            (1000.0, 1.451e-9),
            (800.0, 4.367e-12),
            (500.0, 2.535e-19),
        ] {
            let r = m.blackbody_xyz(t)[1] / sun;
            assert!((r / want - 1.0).abs() < 1e-2, "{t}: {r:e}");
        }
    }

    #[test]
    fn test_the_microwave_background_at_g_389_is_a_dull_red_1060_k_blackbody() {
        // 2.7255 K (Fixsen 2009) x 389 = 1060.2 K: (x, y) = (0.6443, 0.3518), 10.2 cd m^-2.
        let m = model();
        let xyz = m.blackbody_xyz(2.7255 * 389.0);
        let (x, y) = xy(xyz);
        assert!(
            (x - 0.6443).abs() < 1e-4 && (y - 0.3518).abs() < 1e-4,
            "{x} {y}"
        );
        let l = crate::planck::K_M * xyz[1];
        assert!((l - 10.2).abs() < 0.1, "{l}");
        // With a unit scale the hook returns the blackbody's own RGB at g T.
        let rgb = m.shifted_background_rgb(2.7255, 389.0, 1.0);
        let direct = m.blackbody_rgb(2.7255 * 389.0);
        assert!(close(rgb, direct, 1e-15));
        // And unshifted it is black to f64: 2.7255 K has no visible light (e^-6000).
        assert_eq!(m.shifted_background_rgb(2.7255, 1.0, 1.0), [0.0; 3]);
    }

    #[test]
    fn test_into_display_gamut_keeps_colours_inside_and_luminance_below_white() {
        let inside = [0.2, 0.5, 0.9];
        assert_eq!(into_display_gamut(inside), inside);
        for c in [
            [1.4, 0.3, -0.2],
            [0.05, 0.1, 1.8],
            [-0.3, 0.9, 0.2],
            [2.0, 0.1, 0.0],
        ] {
            let y = luminance(c);
            let out = into_display_gamut(c);
            assert!(
                out.iter().all(|v| (0.0..=1.0).contains(v)),
                "{c:?} -> {out:?}"
            );
            if y < 1.0 {
                assert!((luminance(out) - y).abs() < 1e-15, "{c:?}");
                // The same dominant wavelength: the chromaticity is on the line from white.
                let (a, b) = xy(rgb_to_xyz(c));
                let (p, q) = xy(rgb_to_xyz(out));
                let (wx, wy) = crate::colorimetry::BT709_WHITE;
                let cross = (a - wx) * (q - wy) - (b - wy) * (p - wx);
                assert!(cross.abs() < 1e-12, "{c:?}");
            }
        }
        assert_eq!(into_display_gamut([3.0, 2.0, 1.5]), [1.0; 3]);
        assert!(into_display_gamut([f64::NAN, 0.0, 0.0])[0].is_nan());
    }

    #[test]
    fn test_into_display_gamut_is_continuous_as_luminance_reaches_white() {
        let c = [1.6, 0.9, 0.2];
        let y = luminance(c);
        let below = into_display_gamut(c.map(|v| v / y * (1.0 - 1e-9)));
        assert!(below.iter().all(|v| (v - 1.0).abs() < 1e-6), "{below:?}");
    }

    #[test]
    fn test_the_two_temperature_mixture_errs_as_documented() {
        // Equal luminance from 3500 K and 10000 K, which the model treats as one blackbody at the
        // mixture's implied temperature. The true shifted light is the sum of the two shifted
        // blackbodies. Pinned to the numbers in the crate's documentation.
        let m = model();
        let a = m.blackbody_xyz(3500.0);
        let b = m.blackbody_xyz(10_000.0);
        let mix: [f64; 3] = std::array::from_fn(|c| a[c] / a[1] + b[c] / b[1]);
        let i = m.implied_temperature_xyz(mix).unwrap();
        assert!(
            (i.kelvin - 5443.4).abs() < 0.5 && (i.duv + 0.00536).abs() < 1e-5,
            "{i:?}"
        );
        let pixel = crate::colorimetry::xyz_to_rgb(mix);
        for (g, want_duv, want_y) in [
            (0.5, 0.0676, -0.740),
            (0.8, 0.0272, -0.084),
            (1.25, 0.0187, -0.140),
            (2.0, 0.0269, -0.473),
            (10.0, 0.0118, -0.746),
        ] {
            let model_xyz = rgb_to_xyz(m.shift(pixel, g));
            let ga = m.blackbody_xyz(3500.0 * g);
            let gb = m.blackbody_xyz(10_000.0 * g);
            let truth: [f64; 3] = std::array::from_fn(|c| ga[c] / a[1] + gb[c] / b[1]);
            let d = delta_u_prime_v_prime(model_xyz, truth);
            let dy = model_xyz[1] / truth[1] - 1.0;
            assert!((d - want_duv).abs() < 5e-4, "g {g}: du'v' {d}");
            assert!((dy - want_y).abs() < 5e-3, "g {g}: dY {dy}");
        }
    }
}
