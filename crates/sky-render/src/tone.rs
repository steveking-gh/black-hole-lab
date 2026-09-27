//! From the map's linear light to the numbers written into a video frame.
//!
//! Three steps, in this order: scale by the shift and the exposure (linear light), clip to [0, 1],
//! and apply the sRGB transfer function. The frame then holds 16-bit sRGB-encoded values, which
//! ffmpeg turns into 10-bit BT.709 Y'CbCr for the encoder.
//!
//! # The shift
//!
//! Light arriving with shift g carries g^4 times the bolometric intensity of the map at the point
//! it came from (the specification, section 4.6: I_nu / nu^3 is invariant along a ray, and
//! integrating over frequency gives the fourth power). That factor is applied here to every
//! channel alike. A blueshifted sky is also bluer, and a redshifted one redder, which is not done
//! yet: see [`shade`].
//!
//! # Exposure
//!
//! The maps carry no absolute scale, and their values scale with pixel area (assets/sky/README.md,
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
//! on linear light is what this renderer must use, because the shift's g^4 is a factor on linear
//! light too and the two have to compose. Of the gains tried (1.5 to 4 stops in half-stop steps,
//! 2026-09-27, against the approved picture at 1600 x 800) 2.5 stops came nearest, with an rms
//! difference of 5 codes in 255: the 90th percentile and the mean agree, and the faint background
//! is some 5 codes brighter than in the approved clip.
//!
//! The README measures the 4k map's
//! solid-angle-weighted mean at 3.5 times the 8k map's rather than 4, so a 4k map under this rule
//! comes out about 0.2 stops brighter than the 8k one; `--exposure` is there for the eye to
//! correct that. The rule knows nothing of the `milkyway` product, whose background is 1.65 to 2
//! times brighter than `starmap`'s and so wants roughly one stop less.

/// The default exposure, in stops, for a map `width` texels wide.
pub fn default_exposure_stops(width: usize) -> f64 {
    2.5 + 2.0 * (width as f64 / 8192.0).log2()
}

/// The linear light a pixel shows: the map's value `rgb` where the ray came from, seen with shift
/// `g`, under the exposure `gain`. Not clipped.
pub fn shade(rgb: [f32; 3], g: f64, gain: f32) -> [f32; 3] {
    let scale = (g * g * g * g) as f32 * gain;
    // ---- COLOUR SHIFT GOES HERE --------------------------------------------------------------
    // The g^4 above is the bolometric factor, the same for every channel. The full effect of a
    // shift also moves the spectrum: I_nu,observed(nu) = g^3 I_nu,map(nu / g), so a star of
    // temperature T appears as one of temperature g T. Doing that means treating each texel as a
    // blackbody (or some other spectrum) fitted to its RGB, scaling the temperature by g, and
    // integrating the new spectrum against the three primaries - with the g^4 then coming out of
    // that integral rather than being applied separately. Until then the colour is the map's.
    // -------------------------------------------------------------------------------------------
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
/// 16-bit codes. The flat colour of unresolved rays is given this way and written as it is: it is
/// a marker, not light, so no exposure touches it.
pub fn parse_hex_colour(text: &str) -> Option<[u16; 3]> {
    let hex = text.strip_prefix('#').unwrap_or(text);
    if hex.len() != 6 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let channel = |k: usize| u8::from_str_radix(&hex[2 * k..2 * k + 2], 16).ok();
    Some([channel(0)?, channel(1)?, channel(2)?].map(|c| u16::from(c) * 257))
}
