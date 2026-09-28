//! From the map's linear light to the numbers written into a video frame.
//!
//! Three steps, in this order: the light as the shift makes it, times the exposure (linear
//! light), clip to [0, 1], and apply the sRGB transfer function. The frame then holds 16-bit
//! sRGB-encoded values, which ffmpeg turns into 10-bit BT.709 Y'CbCr for the encoder.
//!
//! # The shift
//!
//! How the shift colours light is chosen by `--colour` (`crate::colour`). Under `blackbody`, the
//! default, the colour and the visible brightness both follow the shift, by the model of the
//! crate `sky-colour`, formed in `crate::sky` and brought into the display's gamut below by
//! `crate::colour::lift_negatives`. Under `map`, the old rule, and under either at g = 1, it is
//! [`shade`] here: light arriving with shift g carries g^4 times the bolometric intensity of the
//! map at the point it came from (the specification, section 4.6: I_nu / nu^3 is invariant along
//! a ray, and integrating over frequency gives the fourth power), applied to every channel
//! alike, with the map's hue.
//!
//! # Exposure
//!
//! The maps carry no absolute scale, and their values scale with pixel area (sky/maps/README.md,
//! "Colour and values"): doubling the map's width quarters the values, as if each texel held the
//! light falling in it. The default gain is 2.5 stops for a map 8192 wide and two stops more for
//! every doubling of the width:
//!
//!     stops = 2.5 + 2 log2(W / 8192)
//!
//! which is 0.5 stops for a 4k map and 4.5 for a 16k one.
//!
//! The 2.5 is measured, not chosen. The owner approved the look of a hand-made clip of the 8k
//! `starmap`, and that clip was made with ffmpeg's `exposure` filter at 1.5 applied *after* the
//! sRGB curve, to the encoded values. A factor on encoded values is not an exposure: it raises
//! the contrast, leaving the faint sky about where 1.5 stops of true gain puts it and lifting the
//! bright parts to where 3 stops does. No gain on linear light reproduces that curve, and a gain
//! on linear light is what this renderer must use, because the shift acts on linear light too
//! and the two have to compose. Of the gains tried (1.5 to 4 stops in half-stop steps,
//! 2026-09-27, against the approved picture at 1600 x 800) 2.5 stops came nearest, with an rms
//! difference of 5 codes in 255: the 90th percentile and the mean agree, and the faint background
//! is some 5 codes brighter than in the approved clip.
//!
//! The README measures the 4k map's
//! solid-angle-weighted mean at 3.5 times the 8k map's rather than 4, so a 4k map under this rule
//! comes out about 0.2 stops brighter than the 8k one; `--exposure` is there for the eye to
//! correct that. The rule knows nothing of the `milkyway` product, whose background is 1.65 to 2
//! times brighter than `starmap`'s and so wants roughly one stop less.
//!
//! # Where a colour here could be produced without being known
//!
//! - *g^4 beyond f32's range* ([`shade`]). It was formed in f64 and cast to f32, and for g above
//!   about 10^9.6 the cast is infinite; infinity times a texel of zero is NaN, which the encoder
//!   then wrote as black. Now: a texel of zero is black whatever g is, a positive texel under an
//!   enormous g is clipped white, and a shift so small that g^4 underflows is black. All three are
//!   right, not merely plausible: the light is exactly zero, beyond any display, or under half a
//!   code. `--exposure` is limited to 100 stops either way so that the gain is itself a positive
//!   finite f32 and cannot supply the other half of an infinity times zero.
//! - *Clipping at 1* ([`Encoder::encode`]). A display limit, not an unknown: a clipped pixel is
//!   known to be at least that bright. Channels clip separately, so a very bright coloured pixel
//!   drifts toward white, as it does in any photograph. This holds under `--colour blackbody`
//!   too, where the library's own rule above white is not used, so that g = 1 is the old picture
//!   to the bit (`crate::colour`, "Into the display's gamut").
//! - *A negative channel* (`crate::colour::lift_negatives`). A shifted blackbody's colour can
//!   lie outside BT.709; it is moved toward the grey of its own luminance until no channel is
//!   negative. A display limit too: the luminance and the dominant wavelength are kept, and only
//!   the saturation the display cannot show is given up.
//! - *NaN encoded as 0* ([`Encoder::encode`]). Unreachable from the renderer: map texels are
//!   finite (`crate::load`), the rip-map's weights are finite and positive, and shifts are finite
//!   and not negative (`crate::field`), so the light is finite or, by overflow, infinite. Under
//!   the blackbody model the library answers NaN for a texel that is not the colour of any light
//!   (a map with negative values; NASA's have none), and `crate::render` draws and counts that
//!   pixel as unresolved before the encoder sees it
//!   (`test_no_nan_reaches_the_encoder_for_black_saturated_or_far_off_texels_and_every_awkward_shift`).
//!   The rule stays as a guard for direct callers.
//! - *The colour of shifted light*. Under `--colour blackbody`, the default, the hue and the
//!   visible brightness both follow g, by a stated model: each texel a blackbody at the
//!   temperature its colour implies. It is an approximation, not an unknown: the library documents
//!   its errors (for single stars good from g = 0.9 to 2, for the diffuse glow from 0.8 to 1.25,
//!   and beyond a shift of a few the light an eye would see is mostly not starlight), and marking
//!   it would mark most of a moving observer's sky. So it is not marked; the run prints what the
//!   model assumes and gets wrong, and what share of the film's sky lies outside those ranges, and
//!   `--show-model-range` draws where. The filter's order of averaging and shifting adds an error
//!   of its own, measured and bounded in `crate::colour`, "Filtering and shifting". Under
//!   `--colour map` only the brightness follows g, as g^4, and the hue is the map's: known to be
//!   wrong wherever g is far from 1, and kept for comparison.

/// The default exposure, in stops, for a map `width` texels wide.
pub fn default_exposure_stops(width: usize) -> f64 {
    2.5 + 2.0 * (width as f64 / 8192.0).log2()
}

/// The linear light a pixel shows under `--colour map`, and under either rule at g = 1: the map's
/// value `rgb` where the ray came from, seen with shift `g` as g^4, under the exposure `gain`.
/// Not clipped.
pub fn shade(rgb: [f32; 3], g: f64, gain: f32) -> [f32; 3] {
    let scale = (g * g * g * g) as f32 * gain;
    // The common case, in the arithmetic the renderer has always used. A scale that is not a
    // positive finite f32 comes from a shift beyond f32's range: g^4 is infinite in f32 for g
    // above about 10^9.6 (near an inner horizon that is physical) and zero below about 10^-11.2.
    // An infinite scale times a texel of exactly zero is NaN, which would be drawn as black by
    // accident and not by right; so those cases are formed in f64, with a zero texel zero
    // whatever the shift (no light, blueshifted, is still no light).
    if !(scale.is_finite() && scale > 0.0) {
        let factor = g * g * g * g * f64::from(gain);
        return rgb.map(|c| {
            if c == 0.0 {
                0.0
            } else {
                // Overflow gives an infinity, which the encoder clips to white: the light is
                // then beyond any display, as it should be. In f64 the factor is zero only for g
                // below 10^-81, which leaves nothing a display could show.
                (f64::from(c) * factor) as f32
            }
        });
    }
    // The g^4 above is the bolometric factor, the same for every channel: the old rule. The full
    // effect of a shift also moves the spectrum, I_nu,observed(nu) = g^3 I_nu,map(nu / g), so a
    // star of temperature T appears as one of temperature g T; that is `--colour blackbody`,
    // `crate::colour::light`, where the brightening comes out of the eye's integral of the
    // shifted spectrum rather than being applied separately.
    rgb.map(|c| c * scale)
}

/// The sRGB transfer function (IEC 61966-2-1), from linear light in [0, 1] to the encoded value.
pub fn srgb_encode(linear: f64) -> f64 {
    if linear <= 0.003_130_8 {
        12.92 * linear
    } else {
        1.055 * linear.powf(1.0 / 2.4) - 0.055
    }
}

/// Clipping and the sRGB transfer function, from linear light to a 16-bit code, by table.
///
/// The frame has 33.5 million pixels of three channels, and `powf` for each would be a large
/// share of the time a frame takes. The table has 65,537 entries evenly spaced over [0, 1] and is
/// read with linear interpolation; the curve's second derivative is largest just above the
/// linear segment, at 0.0031308, and even there the interpolation is out by 1e-7, a hundredth of
/// one step of the 16-bit output.
#[derive(Debug, Clone)]
pub struct Encoder {
    table: Vec<f32>,
}

impl Default for Encoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Encoder {
    const STEPS: usize = 65_536;

    pub fn new() -> Self {
        let table = (0..=Self::STEPS)
            .map(|k| (srgb_encode(k as f64 / Self::STEPS as f64) * 65_535.0) as f32)
            .collect();
        Self { table }
    }

    /// The 16-bit code for linear light `x`: clipped to [0, 1], encoded, rounded. A NaN is 0.
    pub fn encode(&self, x: f32) -> u16 {
        // `clamp` passes a NaN through; `max` then `min` turn it into 0.
        #[allow(clippy::manual_clamp)]
        let x = x.max(0.0).min(1.0) * Self::STEPS as f32;
        let k = (x as usize).min(Self::STEPS - 1);
        let f = x - k as f32;
        let y = self.table[k] + f * (self.table[k + 1] - self.table[k]);
        (y + 0.5) as u16
    }
}

/// A colour given as six hex digits (`RRGGBB`, with or without a leading `#`), sRGB-encoded, as
/// 16-bit codes. The marker colours (of unresolved rays and of under-sampled pixels) are given
/// this way and written as they are: they are markers, not light, so no exposure touches them.
pub fn parse_hex_colour(text: &str) -> Option<[u16; 3]> {
    let hex = text.strip_prefix('#').unwrap_or(text);
    if hex.len() != 6 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let channel = |k: usize| u8::from_str_radix(&hex[2 * k..2 * k + 2], 16).ok();
    Some([channel(0)?, channel(1)?, channel(2)?].map(|c| u16::from(c) * 257))
}
