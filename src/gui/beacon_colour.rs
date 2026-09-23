//! What a monochromatic beacon looks like after the geometry has shifted its light.
//!
//! Each observer is taken to carry a beacon of one wavelength, and the wavelength is not invented:
//! it is the *dominant wavelength* of that observer's own marker colour, so the beacon Bob is drawn
//! with and the beacon Alice sees are the same beacon. The rest-frame view then paints the other
//! observer's dot in the colour that beacon arrives in, which is the only honest way to colour a
//! dot that stands for something seen rather than for something known.
//!
//! The chain, end to end, with nothing fitted and nothing tuned by eye:
//!
//! 1. **Rest wavelength.** The marker colour is converted to CIE XYZ, its chromaticity is taken
//!    against the D65 white point, and the ray from D65 through that chromaticity is followed out
//!    to the spectral locus. Where the ray lands is the dominant wavelength. See
//!    [`dominant_wavelength_nm`], and [`BOB_BEACON_NM`] and [`ALICE_BEACON_NM`] for the two the app
//!    uses.
//! 2. **Shift.** lambda_seen = lambda_rest / g, with g = nu_seen / nu_emitted the exact ratio the
//!    arriving ray carries (`physics::as_seen::AsSeen::g`). A redshift is g < 1 and lengthens the
//!    wavelength.
//! 3. **Chromaticity.** The colour matching functions are evaluated at lambda_seen, giving the XYZ
//!    of that monochromatic light, and XYZ goes to linear sRGB through the standard D65 matrix. A
//!    spectral chromaticity is outside the sRGB gamut almost everywhere, so it is brought in by
//!    *desaturating towards the white point* - mixing in D65 until the chromaticity crosses the
//!    gamut boundary - rather than by clamping the three channels separately, which would drag the
//!    hue as well as the saturation and would turn a deep red into orange.
//! 4. **Brightness.** The in-gamut chromaticity is taken at its brightest, then multiplied *in
//!    linear light* by
//!
//!        B = g^4 * V(lambda_seen) / V(lambda_rest),
//!
//!    where V is the photopic luminous efficiency, which for these purposes is the ybar matching
//!    function itself. g^4 is the bolometric flux factor of a point source: one power of g for the
//!    photon energy, one for the arrival rate, and two for the solid angle the source subtends
//!    under aberration. V(lambda_seen)/V(lambda_rest) is the eye's own response falling away at
//!    both ends of the visible band, which is what makes a beacon fade long before its wavelength
//!    has left the band altogether.
//! 5. **Cut-off.** Below [`VISIBLE_FLOOR`] of brightness, or outside 380-780 nm, nothing is drawn
//!    in colour at all: the view switches to a dashed neutral ring, which says "the light is still
//!    arriving from here and the eye can no longer see it" rather than quietly painting a black dot
//!    on a black canvas.
//!
//! The matching functions are the multi-lobe analytic fit of Wyman, Sloan & Shirley, "Simple
//! Analytic Approximations to the CIE XYZ Color Matching Functions", Journal of Computer Graphics
//! Techniques 2(2):1-11 (2013), which reproduces the tabulated CIE 1931 2-degree observer to a few
//! parts in a thousand across the whole band - far inside anything a coloured dot on a screen can
//! show - and needs no interpolation table.

use crate::gui::numbers;
use egui::Color32;

/// Shortest wavelength the eye responds to at all, in nm. Below it the beacon is ultraviolet.
pub const VISIBLE_MIN_NM: f64 = 380.0;

/// Longest wavelength the eye responds to at all, in nm. Above it the beacon is infrared.
pub const VISIBLE_MAX_NM: f64 = 780.0;

/// Smallest linear-light brightness the view will paint as a colour rather than as a ring.
///
/// This is a *display* threshold and not a physical one. The canvas background is very nearly
/// black, a monitor's own black level and the room it stands in swallow the bottom of the range,
/// and a dot at two per cent of full brightness is about the last one that can be told from the
/// background at all. Below it the honest statement is that the eye sees nothing, which is what the
/// dashed ring says.
pub const VISIBLE_FLOOR: f64 = 0.02;

/// The D65 white point's chromaticity, the white sRGB is defined against.
const D65_X: f64 = 0.312_7;
const D65_Y: f64 = 0.329_0;

/// Bob's beacon, in nm: the dominant wavelength of `Theme::BOB_COLOR`, the mint (0, 255, 200).
///
/// Derivation, which `test_the_beacon_constants_are_the_marker_colours_own_dominant_wavelengths`
/// runs again rather than trusting: (0, 255, 200) decodes from sRGB to the linear triple
/// (0, 1, 0.5776), which is XYZ = (0.4618, 0.7568, 0.6681), chromaticity (0.2448, 0.4011). The ray
/// from D65 (0.3127, 0.3290) through that chromaticity meets the spectral locus at 504.81 nm - a
/// blue-green, which is what a mint marker is once its whiteness is taken out.
pub const BOB_BEACON_NM: f64 = 504.81;

/// Alice's beacon, in nm: the dominant wavelength of `Theme::ALICE_COLOR`, the amber (255, 160, 40).
///
/// The same derivation: (255, 160, 40) is the linear triple (1, 0.3516, 0.0212), XYZ =
/// (0.5420, 0.4656, 0.0814), chromaticity (0.4977, 0.4276), and the ray from D65 through it meets
/// the locus at 585.47 nm, in the yellow-orange.
pub const ALICE_BEACON_NM: f64 = 585.47;

/// The stretch of the spectral locus over which the analytic fit is used as it stands, in nm.
///
/// Outside it the locus is clamped to the nearer end, for a reason that is about the eye rather
/// than about the fit. Below about 430 nm and above about 650 nm the *real* locus stops moving:
/// the long- and middle-wavelength cones keep a fixed response ratio out in the red tail, so the
/// locus converges on (0.7347, 0.2653) and stays there, and the same happens at the violet end near
/// (0.1741, 0.0050). The published CIE table moves by under a hundredth in x across either tail.
///
/// The Wyman-Sloan-Shirley fit reproduces the locus to three decimals across (430, 650) and does
/// not reproduce that convergence, because its lobes are Gaussians whose tails fall off at rates
/// the fit never had to match: past 660 nm its xbar decays faster than its ybar and the
/// chromaticity turns back towards orange, which would paint a deeply redshifted beacon the wrong
/// colour on the way out. Holding the locus at its last trustworthy point is the stationary
/// behaviour the real curves have, so the clamp puts the physics back rather than papering over it.
const LOCUS_MIN_NM: f64 = 430.0;
const LOCUS_MAX_NM: f64 = 650.0;

/// What the view is to paint for a beacon after the shift.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Beacon {
    /// The eye sees this colour: a filled dot, already sRGB-encoded and ready to paint.
    Seen(Color32),
    /// The eye sees nothing - the light has left the visible band, or what is left of it is below
    /// what a screen can show against this background - so the view draws the dashed ring instead.
    /// The position is still exact; it is the colour that has run out.
    Ring,
}

/// The one lobe of the Wyman-Sloan-Shirley fit: a Gaussian with a different width either side of
/// its peak, which is what lets three or four of them reproduce the awkward shoulders of the CIE
/// curves.
fn lobe(x: f64, mu: f64, sigma_low: f64, sigma_high: f64) -> f64 {
    let sigma = if x < mu { sigma_low } else { sigma_high };
    let t = (x - mu) / sigma;
    (-0.5 * t * t).exp()
}

/// The CIE 1931 2-degree colour matching functions (xbar, ybar, zbar) at `nm`, by the multi-lobe
/// fit of Wyman, Sloan & Shirley (2013), their equations for the multi-lobe case.
pub fn matching_functions(nm: f64) -> [f64; 3] {
    let x = 1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7)
        - 0.065 * lobe(nm, 501.1, 20.4, 26.2);
    let y = 0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1);
    let z = 1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8);
    [x, y, z]
}

/// The photopic luminous efficiency V(lambda), which is the ybar matching function by definition:
/// the CIE fixed V and ybar to be the same curve when the 1931 observer was standardised.
pub fn luminous_efficiency(nm: f64) -> f64 {
    matching_functions(nm)[1]
}

/// Linear sRGB from CIE XYZ, by the standard D65 matrix. The result may have negative components:
/// that is the statement that the colour is outside the sRGB gamut, and it is the caller's business
/// rather than something to hide here.
fn xyz_to_linear_srgb(xyz: [f64; 3]) -> [f64; 3] {
    let [x, y, z] = xyz;
    [
        3.240_454_2 * x - 1.537_138_5 * y - 0.498_531_4 * z,
        -0.969_266_0 * x + 1.876_010_8 * y + 0.041_556_0 * z,
        0.055_643_4 * x - 0.204_025_9 * y + 1.057_225_2 * z,
    ]
}

/// CIE XYZ from linear sRGB, the inverse of the matrix above.
///
/// Nothing in the drawing path runs this way round: the view only ever goes from a wavelength to a
/// colour. It is here, with `decode_srgb` and `dominant_wavelength_nm`, because it is one step of
/// the derivation that produced `BOB_BEACON_NM` and `ALICE_BEACON_NM`, and that derivation belongs
/// in the module beside the constants it fixed rather than in a notebook nobody can run again.
#[allow(dead_code)]
fn linear_srgb_to_xyz(rgb: [f64; 3]) -> [f64; 3] {
    let [r, g, b] = rgb;
    [
        0.412_456_4 * r + 0.357_576_1 * g + 0.180_437_5 * b,
        0.212_672_9 * r + 0.715_152_2 * g + 0.072_175_0 * b,
        0.019_333_9 * r + 0.119_192_0 * g + 0.950_304_1 * b,
    ]
}

/// The sRGB transfer function, encoding linear light into the values a `Color32` holds.
fn encode_srgb(linear: f64) -> u8 {
    let v = linear.clamp(0.0, 1.0);
    let encoded = if v <= 0.003_130_8 {
        12.92 * v
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    };
    (encoded * 255.0).round().clamp(0.0, 255.0) as u8
}

/// The sRGB transfer function run backwards, from a stored channel to linear light. Part of the
/// derivation of the two beacon constants; see `linear_srgb_to_xyz`.
#[allow(dead_code)]
fn decode_srgb(channel: u8) -> f64 {
    let v = f64::from(channel) / 255.0;
    if v <= 0.040_45 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

/// Chromaticity (x, y) of an XYZ triple, or `None` where the triple is black and has none.
fn chromaticity(xyz: [f64; 3]) -> Option<[f64; 2]> {
    let sum = xyz[0] + xyz[1] + xyz[2];
    (sum > 1e-12).then(|| [xyz[0] / sum, xyz[1] / sum])
}

/// The chromaticity of the spectral locus at `nm`, with the tails held at the ends of the range the
/// fit is trustworthy over. See [`LOCUS_MIN_NM`] for why the holding is the physical behaviour.
fn locus_chromaticity(nm: f64) -> [f64; 2] {
    let nm = nm.clamp(LOCUS_MIN_NM, LOCUS_MAX_NM);
    chromaticity(matching_functions(nm)).unwrap_or([D65_X, D65_Y])
}

/// Whether a chromaticity taken at unit luminance lies inside the sRGB gamut.
fn in_gamut(chroma: [f64; 2]) -> bool {
    xyz_to_linear_srgb(chroma_at_unit_luminance(chroma))
        .iter()
        .all(|c| *c >= -1e-9)
}

/// The XYZ triple of a chromaticity taken at Y = 1, which is the form the gamut test and the
/// brightest-colour construction both want.
fn chroma_at_unit_luminance(chroma: [f64; 2]) -> [f64; 3] {
    let [x, y] = chroma;
    let y = y.max(1e-9);
    [x / y, 1.0, (1.0 - x - y) / y]
}

/// The dominant wavelength of an sRGB colour, in nm, against the D65 white point.
///
/// The construction is the textbook one: take the colour's chromaticity, draw the ray from the
/// white point through it, and report where that ray crosses the spectral locus. The locus is
/// walked here as a function of wavelength and the crossing is found by the angle the locus point
/// makes about the white point passing the angle of the colour - a bracket and a bisection, with no
/// table and no assumption that the locus is convex where it matters.
///
/// The locus is the clamped one of [`locus_chromaticity`], so the search runs over the stretch on
/// which a wavelength and a hue really are in one-to-one correspondence.
///
/// `None` for a colour with no chromaticity at all (black), for one whose hue is a purple - a
/// chromaticity on the line of purples, between the two ends of the locus, which no single
/// wavelength produces - and for one out in either stationary tail, where a hue names a whole
/// stretch of wavelengths rather than one. Neither marker colour is any of those, and a beacon has
/// to be monochromatic to have a wavelength that a shift can act on at all.
#[allow(dead_code)] // the derivation of the beacon constants; see `linear_srgb_to_xyz`
pub fn dominant_wavelength_nm(colour: Color32) -> Option<f64> {
    let linear = [
        decode_srgb(colour.r()),
        decode_srgb(colour.g()),
        decode_srgb(colour.b()),
    ];
    let target = chromaticity(linear_srgb_to_xyz(linear))?;
    let white = [D65_X, D65_Y];
    let dx = target[0] - white[0];
    let dy = target[1] - white[1];
    if dx.hypot(dy) < 1e-6 {
        // The colour *is* the white point; no ray, so no dominant wavelength.
        return None;
    }
    let wanted = dy.atan2(dx);

    // The signed angle from the colour's own direction to the direction of the locus at `nm`,
    // folded into (-pi, pi] so that a sign change across a step is a crossing of the ray and
    // nothing else.
    let offset = |nm: f64| -> f64 {
        let locus = locus_chromaticity(nm);
        let angle = (locus[1] - white[1]).atan2(locus[0] - white[0]) - wanted;
        let turned = angle.rem_euclid(std::f64::consts::TAU);
        if turned > std::f64::consts::PI {
            turned - std::f64::consts::TAU
        } else {
            turned
        }
    };

    // One nanometre is far finer than the locus turns, so a sign change between neighbours is a
    // single crossing and the bisection that follows is unambiguous.
    let steps = ((LOCUS_MAX_NM - LOCUS_MIN_NM) as usize).max(1);
    let mut previous = (LOCUS_MIN_NM, offset(LOCUS_MIN_NM));
    for i in 1..=steps {
        let nm = LOCUS_MIN_NM + i as f64;
        let here = (nm, offset(nm));
        if previous.1 == 0.0 {
            return Some(previous.0);
        }
        if previous.1.signum() != here.1.signum() && previous.1.abs() < 1.0 && here.1.abs() < 1.0 {
            let (mut lo, mut f_lo) = previous;
            let (mut hi, _) = here;
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                let f_mid = offset(mid);
                if f_mid.signum() == f_lo.signum() {
                    lo = mid;
                    f_lo = f_mid;
                } else {
                    hi = mid;
                }
            }
            return Some(0.5 * (lo + hi));
        }
        previous = here;
    }
    None
}

/// The brightest in-gamut sRGB colour of the chromaticity `chroma`, as *linear* light.
///
/// A spectral chromaticity lies outside the sRGB gamut nearly everywhere, so something has to give.
/// What gives here is the saturation: the chromaticity is slid along the straight line towards D65
/// until it crosses the gamut boundary, which is the same operation as adding white light to the
/// beam and is the one that leaves the hue alone. Clamping the three linear channels independently,
/// the usual shortcut, moves the chromaticity sideways as well, and turns a 640 nm red into an
/// orange on the way.
fn brightest_in_gamut(chroma: [f64; 2]) -> [f64; 3] {
    let white = [D65_X, D65_Y];
    let mixed = |t: f64| [white[0] + t * (chroma[0] - white[0]), white[1] + t * (chroma[1] - white[1])];
    // The white point is in the gamut and the spectral point is not, so the boundary is bracketed
    // by construction; thirty halvings put it to a part in 1e9 of the way along.
    let t = if in_gamut(chroma) {
        1.0
    } else {
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..30 {
            let mid = 0.5 * (lo + hi);
            if in_gamut(mixed(mid)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    let rgb = xyz_to_linear_srgb(chroma_at_unit_luminance(mixed(t)));
    // At the boundary one channel is zero and the others are positive; scaling so that the largest
    // is one is the brightest that colour gets on this display. The tiny negatives a boundary
    // bisection can leave behind are rounding and are floored.
    let peak = rgb.iter().cloned().fold(0.0f64, f64::max).max(1e-12);
    [
        (rgb[0] / peak).max(0.0),
        (rgb[1] / peak).max(0.0),
        (rgb[2] / peak).max(0.0),
    ]
}

/// What the eye makes of a beacon of rest wavelength `rest_nm` arriving with frequency ratio `g`.
///
/// `g` is nu_seen / nu_emitted: below one is a redshift and lengthens the wavelength, above one is
/// a blueshift and shortens it. The whole of the model is in the module's own comment; this is the
/// four lines that carry it out.
pub fn seen_beacon(rest_nm: f64, g: f64) -> Beacon {
    if !(g.is_finite() && g > 0.0) || !rest_nm.is_finite() || rest_nm <= 0.0 {
        return Beacon::Ring;
    }
    let seen_nm = rest_nm / g;
    if !(VISIBLE_MIN_NM..=VISIBLE_MAX_NM).contains(&seen_nm) {
        return Beacon::Ring;
    }
    let rest_v = luminous_efficiency(rest_nm).max(1e-12);
    // g^4 is the bolometric flux factor of a point source - one power for the energy of each
    // photon, one for the rate they arrive at, two for the solid angle aberration puts the source
    // into - and the ratio of V is the eye's own response at the two wavelengths. The clamp at one
    // is a *display* limit and not a physical one: a blueshifted beacon really does get brighter
    // without bound, and a screen has nothing above full white to say so with.
    let brightness = (g.powi(4) * luminous_efficiency(seen_nm) / rest_v).min(1.0);
    if brightness < VISIBLE_FLOOR {
        return Beacon::Ring;
    }
    let linear = brightest_in_gamut(locus_chromaticity(seen_nm));
    Beacon::Seen(Color32::from_rgb(
        encode_srgb(linear[0] * brightness),
        encode_srgb(linear[1] * brightness),
        encode_srgb(linear[2] * brightness),
    ))
}

/// The brightness factor on its own, for the info box to print: B = g^4 V(seen)/V(rest), clamped at
/// one for the reason [`seen_beacon`] gives.
pub fn brightness_factor(rest_nm: f64, g: f64) -> f64 {
    if !(g.is_finite() && g > 0.0) {
        return 0.0;
    }
    let seen_nm = rest_nm / g;
    // Outside the band the eye's response is zero, and zero is what is reported. The analytic fit
    // has Gaussian tails that never quite reach it - at 900 nm it still answers 1e-15 - and a box
    // printing 7.362e-16 would be quoting the fit's arithmetic rather than the eye's response.
    if !(VISIBLE_MIN_NM..=VISIBLE_MAX_NM).contains(&seen_nm) {
        return 0.0;
    }
    let rest_v = luminous_efficiency(rest_nm).max(1e-12);
    (g.powi(4) * luminous_efficiency(seen_nm) / rest_v).min(1.0)
}

/// The plain name of the band a wavelength falls in, for the beacon line of the info box. The
/// boundaries are the conventional ones and the two outside the visible band are what the dashed
/// ring stands for.
pub fn band_name(nm: f64) -> &'static str {
    if !nm.is_finite() {
        "—"
    } else if nm < VISIBLE_MIN_NM {
        "ultraviolet"
    } else if nm < 450.0 {
        "violet"
    } else if nm < 495.0 {
        "blue"
    } else if nm < 570.0 {
        "green"
    } else if nm < 590.0 {
        "yellow"
    } else if nm < 620.0 {
        "orange"
    } else if nm <= VISIBLE_MAX_NM {
        "red"
    } else {
        "infrared"
    }
}

/// A wavelength printed in whatever unit reads naturally, from the picometres a hard blueshift
/// reaches to the metres a deep redshift does.
pub fn wavelength_label(nm: f64) -> String {
    if !nm.is_finite() || nm <= 0.0 {
        return "—".to_string();
    }
    let metres = nm * 1e-9;
    let (value, unit) = if metres < 1e-12 {
        (metres * 1e15, "fm")
    } else if metres < 1e-9 {
        (metres * 1e12, "pm")
    } else if metres < 1e-6 {
        (metres * 1e9, "nm")
    } else if metres < 1e-3 {
        (metres * 1e6, "µm")
    } else if metres < 1.0 {
        (metres * 1e3, "mm")
    } else if metres < 1e3 {
        (metres, "m")
    } else {
        (metres * 1e-3, "km")
    };
    let decimals = if value >= 100.0 { 0 } else if value >= 10.0 { 1 } else { 2 };
    format!("{} {unit}", numbers::fixed(value, decimals))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gui::theme::Theme;

    /// The linear-light luminance of a painted colour, for the brightness assertions.
    fn luminance(c: Color32) -> f64 {
        linear_srgb_to_xyz([decode_srgb(c.r()), decode_srgb(c.g()), decode_srgb(c.b())])[1]
    }

    fn seen(rest: f64, g: f64) -> Color32 {
        match seen_beacon(rest, g) {
            Beacon::Seen(c) => c,
            Beacon::Ring => panic!("{rest} nm at g = {g} should still be visible"),
        }
    }

    #[test]
    fn test_the_matching_functions_reproduce_the_published_landmarks() {
        // Three things the CIE 1931 2-degree observer is known by, checked before anything is built
        // on the fit. ybar peaks at 555 nm, which is the definition of the photopic peak; the
        // matching functions are non-negative everywhere; and the equal-energy white they integrate
        // to is the chromaticity (1/3, 1/3), which is what says the three curves are scaled
        // correctly relative to one another.
        let peak = (380..=780)
            .map(|nm| (nm as f64, luminous_efficiency(nm as f64)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .expect("the band is not empty");
        println!("the fitted ybar peaks at {} nm, V = {:.4}", peak.0, peak.1);
        assert!((peak.0 - 555.0).abs() <= 2.0, "the photopic peak is 555 nm, got {}", peak.0);
        for nm in 380..=780 {
            let cmf = matching_functions(nm as f64);
            assert!(cmf.iter().all(|c| *c >= -1e-9), "negative matching function at {nm} nm: {cmf:?}");
        }
        let mut sum = [0.0f64; 3];
        for i in 0..=4000 {
            let nm = VISIBLE_MIN_NM + (VISIBLE_MAX_NM - VISIBLE_MIN_NM) * (i as f64) / 4000.0;
            let cmf = matching_functions(nm);
            for (s, c) in sum.iter_mut().zip(cmf) {
                *s += c;
            }
        }
        let white = chromaticity(sum).expect("the integral is not black");
        println!("equal-energy white from the fit: ({:.4}, {:.4})", white[0], white[1]);
        assert!(
            (white[0] - 1.0 / 3.0).abs() < 0.01 && (white[1] - 1.0 / 3.0).abs() < 0.01,
            "equal-energy white must be (1/3, 1/3), got {white:?}"
        );
    }

    #[test]
    fn test_the_beacon_constants_are_the_marker_colours_own_dominant_wavelengths() {
        // The constants are not chosen: they are read off the two marker colours, and this is the
        // reading, run again. If somebody retunes a marker colour the constant has to move with it,
        // and this test is what says so.
        let bob = dominant_wavelength_nm(Theme::BOB_COLOR).expect("the mint marker has a hue");
        let alice = dominant_wavelength_nm(Theme::ALICE_COLOR).expect("and so does the amber one");
        println!(
            "Theme::BOB_COLOR {:?} -> {bob:.2} nm ({}); Theme::ALICE_COLOR {:?} -> {alice:.2} nm ({})",
            Theme::BOB_COLOR,
            band_name(bob),
            Theme::ALICE_COLOR,
            band_name(alice)
        );
        assert!(
            (bob - BOB_BEACON_NM).abs() < 0.1,
            "BOB_BEACON_NM says {BOB_BEACON_NM} nm, the colour says {bob}"
        );
        assert!(
            (alice - ALICE_BEACON_NM).abs() < 0.1,
            "ALICE_BEACON_NM says {ALICE_BEACON_NM} nm, the colour says {alice}"
        );
        // A pure spectral colour is its own dominant wavelength, which is the construction's own
        // fixed point and the one case where the answer can be known without the geometry.
        for nm in [480.0f64, 520.0, 600.0, 640.0] {
            let colour = seen(nm, 1.0);
            let back = dominant_wavelength_nm(colour).expect("a spectral colour has a hue");
            assert!(
                (back - nm).abs() < 6.0,
                "{nm} nm painted and read back as {back} nm ({colour:?})"
            );
        }
    }

    #[test]
    fn test_a_beacon_at_rest_is_the_colour_of_its_own_wavelength() {
        // The two ends of the visible band and the middle of it, unshifted: 500 nm has to come out
        // green, 650 nm red, and 430 nm violet-blue. "Green" here means the green channel leads,
        // which is the only claim a test can make about a hue without repeating the arithmetic.
        let green = seen(500.0, 1.0);
        println!("500 nm at rest: {green:?}");
        assert!(
            green.g() > green.r() && green.g() > green.b(),
            "500 nm must be greenish, got {green:?}"
        );
        let red = seen(650.0, 1.0);
        println!("650 nm at rest: {red:?}");
        assert!(red.r() > 2 * u8::max(red.g(), red.b()), "650 nm must be red, got {red:?}");
        let violet = seen(430.0, 1.0);
        println!("430 nm at rest: {violet:?}");
        assert!(violet.b() > violet.g(), "430 nm must be blue-violet, got {violet:?}");
    }

    #[test]
    fn test_the_far_red_is_red_and_very_dim() {
        // Bob's beacon redshifted to 650 nm and then to 705 nm. Past 700 nm the eye's response has
        // fallen by two orders from its peak, so the dot is still red and is nearly out. Both
        // halves matter: a model that kept the brightness would paint a bright red dot where the
        // physics says a barely visible one.
        let at = |nm: f64| BOB_BEACON_NM / nm;
        let bright = luminance(seen(BOB_BEACON_NM, at(620.0)));
        let faint = seen(BOB_BEACON_NM, at(660.0));
        let faint_l = luminance(faint);
        println!(
            "Bob's beacon seen at 620 nm has luminance {bright:.4}; at 660 nm it has {faint_l:.5} \
             and paints as {faint:?} (brightness factor {:.4} against {:.4})",
            brightness_factor(BOB_BEACON_NM, at(660.0)),
            brightness_factor(BOB_BEACON_NM, at(620.0))
        );
        assert!(faint.r() > 3 * u8::max(faint.g(), faint.b()), "660 nm is red: {faint:?}");
        assert!(faint_l < 0.2 * bright, "and far dimmer than at 620 nm: {faint_l} vs {bright}");
        // Past 700 nm the eye's response has fallen by two further orders and the ring takes over,
        // because what is left is below the display floor rather than because the light has stopped
        // arriving.
        println!(
            "at 705 nm the brightness factor is {:.2e}, under the {VISIBLE_FLOOR} floor",
            brightness_factor(BOB_BEACON_NM, at(705.0))
        );
        assert_eq!(
            seen_beacon(BOB_BEACON_NM, at(705.0)),
            Beacon::Ring,
            "a beacon seen at 705 nm is past what a screen can show"
        );
    }

    #[test]
    fn test_the_hue_drifts_the_one_way_as_the_shift_deepens() {
        // Bob's beacon reddening as g falls from 1: the seen wavelength must rise monotonically,
        // and the painted colour must walk the bands in order and never back. This is the thing the
        // view is actually showing, so it is checked on the sequence rather than on any one frame.
        let mut previous_nm = 0.0;
        let mut bands: Vec<&'static str> = Vec::new();
        let mut last_hue = 0.0f64;
        let mut g = 1.0f64;
        while g > 0.05 {
            let nm = BOB_BEACON_NM / g;
            assert!(nm > previous_nm, "the seen wavelength must rise as g falls");
            previous_nm = nm;
            if let Beacon::Seen(colour) = seen_beacon(BOB_BEACON_NM, g) {
                let band = band_name(nm);
                if bands.last() != Some(&band) {
                    bands.push(band);
                }
                // The hue of the *painted* dot, read back as a dominant wavelength: desaturating
                // towards the white point leaves the chromaticity on the ray it was on, so the
                // painted colour names the wavelength it was made from and that number has to
                // climb. Checked while the dot is still bright enough that the eight bits a
                // channel is stored in resolve its hue - down at the display floor a channel holds
                // single digits and the angle it carries is quantisation.
                if brightness_factor(BOB_BEACON_NM, g) >= 0.15 {
                    let hue = dominant_wavelength_nm(colour).expect("a painted beacon has a hue");
                    assert!(
                        hue > last_hue - 2.0,
                        "the hue turned back at g = {g} (seen at {nm:.1} nm): read {hue:.1} nm \
                         after {last_hue:.1} nm"
                    );
                    last_hue = hue;
                }
            }
            g *= 0.98;
        }
        println!("Bob's beacon walks {bands:?} as g falls from 1");
        assert_eq!(
            bands,
            vec!["green", "yellow", "orange", "red"],
            "the bands must be walked in order and each one only once"
        );
    }

    #[test]
    fn test_the_ring_takes_over_at_both_ends_of_the_band() {
        // The two ways a beacon goes out. Redshifted far enough its wavelength leaves the band
        // altogether; blueshifted far enough it leaves the other end. In between, the brightness
        // cut-off catches it first on the red side - which is the honest order, because the eye
        // stops responding before the light stops being light.
        let deep_red = seen_beacon(BOB_BEACON_NM, BOB_BEACON_NM / 2000.0);
        assert_eq!(deep_red, Beacon::Ring, "a beacon seen at 2 µm is infrared");
        let hard_blue = seen_beacon(ALICE_BEACON_NM, 4.0);
        assert_eq!(hard_blue, Beacon::Ring, "a fourfold blueshift puts 586 nm in the ultraviolet");

        // The first g at which the red side gives out, and what it is at: the seen wavelength is
        // still inside the band, so it is the brightness floor that ended it.
        let mut g = 1.0f64;
        while matches!(seen_beacon(BOB_BEACON_NM, g), Beacon::Seen(_)) && g > 1e-6 {
            g *= 0.995;
        }
        let out_at = BOB_BEACON_NM / g;
        println!(
            "Bob's {BOB_BEACON_NM} nm beacon goes out at g = {g:.4}, seen at {} ({}), brightness {:.4}",
            wavelength_label(out_at),
            band_name(out_at),
            brightness_factor(BOB_BEACON_NM, g)
        );
        assert!(g < 1.0 && g > 0.1, "the beacon must survive a modest redshift: out at g = {g}");
        assert!(
            out_at < VISIBLE_MAX_NM,
            "on the red side the brightness floor must bite first, not the band edge: {out_at} nm"
        );
    }

    #[test]
    fn test_a_blueshift_walks_the_other_way_and_then_runs_out() {
        // Alice's amber seen from a deep infall: 586 nm shifted up through green and blue to violet,
        // and out. The brightness clamp is what stops the dot getting brighter than white on the
        // way, and it is a display limit rather than a physical one - the flux really does grow.
        let mut bands: Vec<&'static str> = Vec::new();
        let mut g = 1.0f64;
        while g < 2.0 {
            let nm = ALICE_BEACON_NM / g;
            if let Beacon::Seen(_) = seen_beacon(ALICE_BEACON_NM, g) {
                let band = band_name(nm);
                if bands.last() != Some(&band) {
                    bands.push(band);
                }
            }
            g *= 1.01;
        }
        println!("Alice's beacon walks {bands:?} as g rises from 1");
        assert_eq!(bands, vec!["yellow", "green", "blue", "violet"]);
        assert_eq!(
            seen_beacon(ALICE_BEACON_NM, 1.7),
            Beacon::Ring,
            "past the violet end there is nothing left to paint"
        );
        assert!(
            (brightness_factor(ALICE_BEACON_NM, 1.1) - 1.0).abs() < 1e-12,
            "the brightness is clamped at one where the flux would otherwise run past white: \
             g^4 V(533 nm)/V(585 nm) = {}",
            1.1f64.powi(4) * luminous_efficiency(ALICE_BEACON_NM / 1.1)
                / luminous_efficiency(ALICE_BEACON_NM)
        );
    }

    #[test]
    fn test_a_gamut_clip_desaturates_rather_than_bending_the_hue() {
        // The one piece of the chain that could quietly lie. A spectral red at 640 nm is well
        // outside sRGB; brought in by desaturation its chromaticity stays on the line from D65
        // through the spectral point, so reading its dominant wavelength back gives 640 nm again.
        // Clamping the three channels instead moves the chromaticity off that line, and the read
        // back would come out tens of nanometres away. The stretch swept is the one on which a hue
        // names a wavelength at all: outside it the locus is stationary (see `LOCUS_MIN_NM`), so
        // reading a wavelength back from a hue there is not a question with one answer.
        for nm in [440.0f64, 480.0, 520.0, 590.0, 640.0] {
            let painted = seen(nm, 1.0);
            let back = dominant_wavelength_nm(painted).expect("the painted colour has a hue");
            println!("{nm} nm -> {painted:?} -> read back as {back:.1} nm");
            assert!(
                (back - nm).abs() < 8.0,
                "the hue moved from {nm} nm to {back} nm on the way through the gamut"
            );
        }
    }

    #[test]
    fn test_a_wavelength_is_printed_in_a_unit_worth_reading() {
        assert_eq!(wavelength_label(496.0), "496 nm");
        assert_eq!(wavelength_label(1900.0), "1.90 µm");
        assert_eq!(wavelength_label(1.0e9), "1.00 m");
        assert_eq!(wavelength_label(0.4), "400 pm");
        assert_eq!(wavelength_label(f64::INFINITY), "—");
    }
}
