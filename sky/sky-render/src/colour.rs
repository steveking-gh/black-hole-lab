//! The colour of shifted light: which rule, how a pixel's light is formed under it, and what the
//! run tells the viewer about where the rule can be trusted.
//!
//! # The two rules
//!
//! `--colour blackbody` (the default) is the model of the crate `sky-colour`: each texel of the
//! map is taken to be a blackbody at the temperature its colour implies, times a tint, and light
//! seen with shift g is the blackbody at g T with the same tint, as a standard human observer sees
//! it. Hue and visible brightness both follow the shift, and there is no separate g^4: the
//! brightening is the eye's, inside the model. `--colour map` is the renderer's old rule, the
//! map's colour times the bolometric g^4, kept to the bit for comparison ([`crate::tone::shade`]).
//!
//! # g = 1
//!
//! Under either rule a pixel whose g is exactly 1 goes through [`crate::tone::shade`] as it always
//! did, so a flat-space film at rest is the same to the bit. The library provides for this
//! (`sky_colour::needs_shift`); the renderer skips the model there rather than calling it, because
//! its own path at g = 1 must also skip the display step below, which the old renderer did not
//! have.
//!
//! # Into the display's gamut
//!
//! A shifted colour can have a negative channel: a hot or a cool blackbody's colour lies outside
//! BT.709 (blue is negative below 1905 K, and the Rayleigh-Jeans colour of a large blueshift is
//! outside it too). A negative channel is light the display cannot make, and the encoder would
//! clip it to 0, which adds to the colour the complement of what is missing and changes its
//! luminance. [`lift_negatives`] instead moves the colour toward the grey of the same luminance,
//! just far enough that no channel is negative, with the library's `into_display_gamut`: the
//! luminance is kept exactly and so is the dominant wavelength.
//!
//! It is applied after the exposure because the library's function is defined on values in
//! which 1 is the display's white, and before the transfer curve because luminance, and the move
//! toward grey, are linear-light notions. (The lift toward grey is the same at any scale, so for
//! the negative channels alone the order against the exposure does not change the result; the
//! function is applied where its definition says.)
//!
//! Above white it is not used: a channel above 1 is clipped by the encoder, as it always was. The
//! library's own rule there (luminance at or above 1 made white, and a colour above 1 in one
//! channel desaturated at constant luminance) differs from a clip channel by channel for every
//! pixel brighter than the display, and at the default exposure the cores of the map's bright
//! stars are such pixels at g = 1. Using it would change a flat-space film at rest, which must
//! not change; using it only where g differs from 1 would make those pixels jump as g passes
//! through 1. Clipping above white is a display limit either way, not a statement about the
//! light. So [`lift_negatives`] calls the library's function on the colour scaled so that no
//! channel exceeds 1, where only its negative-channel branch can act, and scales the result back.
//!
//! # Filtering and shifting
//!
//! The model is not linear in colour: the shift of a mean of two colours is not the mean of their
//! shifts. The rip-map filter (`crate::sky`) gives a pixel a weighted mean of texels, so there are
//! two orders. The true one shifts every level-0 texel of the pixel's footprint and averages the
//! shifted light. The cheap one averages first (the rip-map's pre-averaged levels) and shifts
//! the mean, at the temperature the mean's colour implies. What is done here is between the two
//! ([`crate::sky::SkyMap::sample_shifted`]): each texel the filter reads (up to four levels, four
//! taps each) is shifted at its own temperature and the shifted texels are blended with the
//! filter's weights. That is the true order for the bilinear and between-level blends, and exact
//! wherever the filter reads level 0; what remains of the cheap order is inside a coarse level's
//! texels, each the mean of the level-0 texels it covers and shifted as one colour.
//!
//! Measured 2026-09-27 on the 8k galactic `starmap` and the frames the renderer draws at
//! 8192 x 4096 (`test_measure_the_filter_order_on_the_real_map`, every 61st sky pixel of video
//! frames 0, 237 and 473 of `bob_near_fall` and frame 0 of `hover_r6_a09`, 1.4 million pixels).
//! The truth is the map shifted texel by texel and built into a rip-map of its own: reading a
//! rip-map is linear in its texels, so that is exactly the true order with the filter's weights.
//! The footprints that occur, by the larger span in level-0 texels: at most 1, 10 %; 1 to 2, 40 %;
//! 2 to 4, 26 %; 4 to 8, 13 %; 8 to 16, 5 %; 16 to 64, 5 %; above 64, 1 %. The error in
//! brightness, as the luminance-weighted mean |dY| / Y, is always a dimming (the mean error is
//! the bias, to within 0.1 %, except at g = 0.9, where both are below 0.1 %):
//!
//! | g | order | span <= 1 | 1-2 | 2-4 | 4-8 | 8-16 | 16-64 | > 64 |
//! | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
//! | 0.5 | blend, shift | 5.1 % | 5.5 % | 8.7 % | 12.7 % | 17.8 % | 26 % | 33 % |
//! | 0.5 | per tap | 0 | 1.7 % | 4.7 % | 8.3 % | 12.7 % | 21 % | 30 % |
//! | 0.8 | blend, shift | 0.25 % | 0.28 % | 0.40 % | 0.55 % | 0.72 % | 0.99 % | 1.3 % |
//! | 0.8 | per tap | 0 | 0.09 % | 0.23 % | 0.38 % | 0.53 % | 0.79 % | 1.1 % |
//! | 1.25 | blend, shift | 0.56 % | 0.61 % | 0.96 % | 1.3 % | 1.7 % | 2.0 % | 2.4 % |
//! | 1.25 | per tap | 0 | 0.20 % | 0.56 % | 0.93 % | 1.3 % | 1.6 % | 2.1 % |
//! | 2 | blend, shift | 2.8 % | 3.0 % | 4.6 % | 6.1 % | 7.8 % | 8.8 % | 10.3 % |
//! | 2 | per tap | 0 | 1.0 % | 2.8 % | 4.4 % | 6.1 % | 7.4 % | 9.1 % |
//! | 10 | blend, shift | 8.0 % | 8.2 % | 13.3 % | 16.7 % | 23 % | 23 % | 24 % |
//! | 10 | per tap | 0 | 3.0 % | 8.1 % | 12.7 % | 19.0 % | 19.6 % | 22 % |
//! | 389 | blend, shift | 9.2 % | 9.5 % | 15.9 % | 20 % | 28 % | 27 % | 27 % |
//! | 389 | per tap | 0 | 3.5 % | 9.7 % | 15.5 % | 23 % | 23 % | 25 % |
//!
//! The colour error, as the 95th percentile of du'v' over visible pixels (at least 1e-3 of white
//! at the default exposure), per tap: at g = 0.8 from 0.0013 (span 1 to 2) to 0.0099 (above 64);
//! at 1.25 from 0.0009 to 0.0054; at 2 from 0.0017 to 0.0091; at 0.5 from 0.011 to 0.072; at 10
//! and above below 0.012. Blending first is worse at every span and g measured, and wrong even at
//! level 0, where the bilinear taps of neighbouring texels of different colour are mixed (at
//! g = 0.5 its 95th percentile there is 36 % in brightness and 0.031 in du'v').
//!
//! Against the model's own error where it is good (the library's "Where the model is good": for
//! single stars from g = 0.9 to 2, brightness within about 20 % and colour within about 0.015;
//! for the diffuse glow from 0.8 to 1.25, brightness some 10 % low and colour off by about 0.02),
//! the order used here is below it at every footprint: at most 2.1 % and 0.0099 from 0.8 to
//! 1.25, at most 9.1 % and 0.0091 at g = 2, and below 3 % for the three quarters of the sky whose
//! footprint is at most 4 texels. `test_measure_the_filter_order_on_the_real_map` asserts these
//! bounds (below 10 % in brightness and 0.01 in colour for g from 0.8 to 2, below 3 % for spans up
//! to 4, exact at level 0, never worse than blending first). Blending first would also have been
//! within the model's error there, narrowly (10.3 % at g = 2 against the glow's own -47 % there,
//! and a 95th percentile of 20 % against the stars' 20 %), and would cost somewhat less: the
//! library's shift of the blend, 122 ns a pixel with its search for the blend's temperature,
//! against 12 texels a sky pixel read here on average (measured, 11.9) at about 15 ns each, their
//! temperatures found once when the map is loaded (see "Speed").
//!
//! What remains is the same error as the library's item 2 (a mixture shifted as one blackbody is
//! too dim), at the scale of one coarse texel instead of one level-0 texel, and of the same sign:
//! it adds to that error and never cancels it. It could be made smaller still by storing, for each
//! coarse texel, its light split by temperature (two or more bins of ln T, each shifted at its own
//! temperature), at 16 bytes a bin for the three quarters of the rip-map above level 0, about
//! 1.6 GB a bin for the 8k map; or by reading a finer level than the footprint asks for and
//! averaging several reads, which halves the span of what is pre-averaged at four times the cost.
//! Neither is done: within the model's good range the error is already below the model's own,
//! and beyond it the model is itself the larger error by far.
//!
//! # Speed
//!
//! Measured 2026-09-27 on the owner's machine (Ryzen 7 7800X3D, 16 threads), release build,
//! 20 video frames (228 to 247) of `bob_near_fall` at 8192 x 4096 with `--encoder none`,
//! rendering time a frame, and the rip-map's memory:
//!
//! | build | ms a frame | map in memory |
//! | --- | ---: | ---: |
//! | as committed before this rule | 404 to 461 | 1610 MB |
//! | `--colour map` | 413 to 534 | 1610 MB |
//! | `--colour blackbody`, the library's `xyz_ratio` per texel read | 1899 to 1958 | 1610 + 537 MB |
//! | the same, with [`LnBlackbody`] and ln T stored per texel | 1314 | 1610 + 537 MB |
//! | the same, with ln T and the tint stored per texel (what is built) | 1017 | 1610 + 2147 MB |
//!
//! (The first runs of each build varied by 15 % from the later ones; the ranges are over two or
//! three runs.) A sky pixel reads 12 texels on average (11.9, measured), so the model adds about
//! 180 ns a pixel of one thread's time: per texel one lookup of [`LnBlackbody`] (6 ns) and three
//! exponentials (3 ns each), where the library's `xyz_ratio` takes 50 ns
//! (`test_bench_the_per_tap_costs`). The temperatures and tints of every texel of every level are
//! found once, when the map is loaded: 1.7 s on 16 threads for the 8k map's 134 million texels.
//! Storing the tint as well as ln T costs 1.6 GB more for 0.3 s a frame; storing ln T alone would
//! cost 537 MB and 1.3 s a frame.
//!
//! # Where the model holds
//!
//! The library states in prose, not as constants, the ranges of g in which the model is good
//! (its crate documentation, "Where the model is good"). They are defined here, citing it:
//! [`STAR_RANGE`], [`GLOW_RANGE`] and [`NOT_STARLIGHT`]. Pixels outside them are approximate, not
//! unknown, and are not marked; the run counts them (`crate::tally::ShiftCounts`) and prints the
//! fractions, and `--show-model-range` draws them in false colour ([`Class`]).
//!
//! # The microwave background
//!
//! The library can add the cosmic microwave background as a uniform blackbody background, but
//! only with the map's scale in absolute units, which is not known (sky/maps/README.md, "Absolute
//! scale"). It is left out, at the marked place in [`light`], and the run says so when the film's
//! largest g is past [`background_visible_g`].

use sky_colour::colorimetry::xy;
use sky_colour::planck::{K_M, ln_blackbody_xyz};
use sky_colour::table::T_TABLE_MIN;
use sky_colour::{into_display_gamut, needs_shift};

use crate::sky::{Footprint, SkyMap};
use crate::tone::shade;

/// How shifted light is coloured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColourRule {
    /// The map's colour times g^4: the renderer's old rule.
    Map,
    /// The blackbody model of the crate `sky-colour`.
    Blackbody,
}

impl ColourRule {
    pub fn name(self) -> &'static str {
        match self {
            Self::Map => "map",
            Self::Blackbody => "blackbody",
        }
    }
}

/// The range of g in which the model is good for a single star: brightness within about 20 % and
/// colour within about 0.015 u'v' for both the Sun and Vega. From the crate `sky-colour`'s
/// documentation, "Where the model is good" ("For individual stars, g from 0.9 to 2"), which
/// states it only in prose. Both ends included.
pub const STAR_RANGE: (f64, f64) = (0.9, 2.0);

/// The range of g in which the model is good for the diffuse glow of the Milky Way, and even
/// there its brightness is some 10 % low and its colour visibly off. From the same place ("For
/// the diffuse glow, only g from about 0.8 to 1.25"). Both ends included.
pub const GLOW_RANGE: (f64, f64) = (0.8, 1.25);

/// Beyond a shift of this factor either way (g above it, or below its inverse) what an eye would
/// see is mostly not starlight: under a blueshift light emitted beyond 1.9 to 3.9 micrometres,
/// where dust and the cosmic backgrounds take over from the stars, and under a redshift light
/// emitted at 76 to 156 nm, far in the ultraviolet and partly beyond the Lyman limit, which the
/// interstellar gas absorbs (the library's item 4: "Starlight dominates the real sky out to about
/// 3 to 5 micrometres (g up to about 5)").
pub const NOT_STARLIGHT: f64 = 5.0;

/// The temperature of the cosmic microwave background, K (Fixsen 2009, the value the library's
/// documentation and tests use).
pub const BACKGROUND_KELVIN: f64 = 2.7255;

/// The least luminance counted as seen by an eye, cd m^-2: the lower limit of mesopic vision,
/// 0.005 cd m^-2 (CIE 191:2010), below which only the rods see and see no colour. The library's
/// luminance is the photopic one (y-bar), so this is where its numbers begin to describe what an
/// eye sees; a dark-adapted eye detects a large field down to about 1e-6 cd m^-2, a shift of
/// about 0.8 times the one this gives (the background's visible light climbs a Wien tail steeply).
pub const VISIBLE_LUMINANCE: f64 = 0.005;

/// The classes of g that `--show-model-range` draws, in order of g.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    /// g < 1 / [`NOT_STARLIGHT`].
    FarRed,
    /// 1 / [`NOT_STARLIGHT`] <= g < 0.8.
    Red,
    /// 0.8 <= g < 0.9: within [`GLOW_RANGE`] only.
    GlowOnly,
    /// 0.9 <= g <= 1.25: within both ranges.
    Both,
    /// 1.25 < g <= 2: within [`STAR_RANGE`] only.
    StarOnly,
    /// 2 < g <= [`NOT_STARLIGHT`].
    Blue,
    /// g > [`NOT_STARLIGHT`].
    FarBlue,
}

impl Class {
    pub const ALL: [Class; 7] = [
        Class::FarRed,
        Class::Red,
        Class::GlowOnly,
        Class::Both,
        Class::StarOnly,
        Class::Blue,
        Class::FarBlue,
    ];

    /// The class of a shift. The ranges are those of the constants above; as they are written
    /// the two good ranges overlap from 0.9 to 1.25, and these seven classes tile g >= 0.
    pub fn of(g: f64) -> Class {
        debug_assert!(STAR_RANGE.0 > GLOW_RANGE.0 && STAR_RANGE.1 > GLOW_RANGE.1);
        if g < 1.0 / NOT_STARLIGHT {
            Class::FarRed
        } else if g < GLOW_RANGE.0 {
            Class::Red
        } else if g < STAR_RANGE.0 {
            Class::GlowOnly
        } else if g <= GLOW_RANGE.1 {
            Class::Both
        } else if g <= STAR_RANGE.1 {
            Class::StarOnly
        } else if g <= NOT_STARLIGHT {
            Class::Blue
        } else {
            Class::FarBlue
        }
    }

    pub fn index(self) -> usize {
        self as usize
    }

    /// The false colour, sRGB-encoded: greens where the model is good, oranges and browns under
    /// redshift beyond, blues under blueshift beyond. None is red (what is not known), black (the
    /// dark region) or white.
    pub fn hex(self) -> &'static str {
        match self {
            Class::FarRed => "8C510A",
            Class::Red => "F4A460",
            Class::GlowOnly => "D9EF8B",
            Class::Both => "1A9850",
            Class::StarOnly => "91CF60",
            Class::Blue => "74ADD1",
            Class::FarBlue => "313695",
        }
    }

    pub fn code(self) -> [u16; 3] {
        crate::tone::parse_hex_colour(self.hex()).expect("the class colours are six hex digits")
    }

    /// What the class means, for the run's legend.
    pub fn meaning(self) -> String {
        let n = NOT_STARLIGHT;
        let (s0, s1) = STAR_RANGE;
        let (l0, l1) = GLOW_RANGE;
        match self {
            Class::FarRed => format!(
                "g < 1/{n}: redshift beyond {n}; the eye would see light emitted in the far \
                 ultraviolet, mostly not starlight"
            ),
            Class::Red => {
                format!(
                    "1/{n} <= g < {l0}: redshift outside both ranges; stars' ultraviolet is not a blackbody's"
                )
            }
            Class::GlowOnly => format!(
                "{l0} <= g < {s0}: good for the diffuse glow only (an A star's Balmer jump enters \
                 the visible band)"
            ),
            Class::Both => format!("{s0} <= g <= {l1}: good for single stars and the diffuse glow"),
            Class::StarOnly => format!("{l1} < g <= {s1}: good for single stars only"),
            Class::Blue => {
                format!("{s1} < g <= {n}: blueshift outside both ranges, still mostly starlight")
            }
            Class::FarBlue => format!(
                "g > {n}: blueshift beyond {n}; the eye would see light emitted in the infrared, \
                 mostly not starlight"
            ),
        }
    }
}

/// ln X, ln Y, ln Z of a blackbody against w = ln T, for the shift of every texel the filter
/// reads.
///
/// The library's `Model::xyz_ratio` is the same quantity, exp(ln B(g T) - ln B(T)), but each of
/// its two lookups takes the logarithm of T and forms the slope as well as the value, 50 ns in
/// all, measured (`test_bench_the_per_tap_costs`); a frame of the film reads about ten texels a
/// pixel, 33.5 million pixels. This table is read at w directly (the texel's ln T is stored, and
/// ln g is found once a pixel) and returns values only. It is built as the library builds its own
/// (`sky_colour::table`): the exact logarithms and slopes of the direct sum,
/// `sky_colour::planck::ln_blackbody_xyz`, at 128 nodes per e-fold of T, read by cubic Hermite
/// interpolation, whose error at that step is below 3e-8 of the value at 10 K and 3e-9 above
/// 300 K (the library's measurement of the same scheme on the same function). It runs from
/// [`T_TABLE_MIN`] (10 K), below which the library's own shift is black, to 10^10 K, beyond the
/// library's 10^6 K, since in logarithms the direct sum is exact there too; a texel shifted past
/// its top is sent to the library. `test_the_renderers_blackbody_table_is_the_librarys_ratio`
/// holds the two to 1e-7.
pub struct LnBlackbody {
    w0: f64,
    h: f64,
    inv_h: f64,
    /// Per node: ln X, ln Y, ln Z, then their slopes d ln X / d ln T.
    nodes: Vec<[f64; 6]>,
}

/// The top of [`LnBlackbody`], K.
pub const LN_TABLE_T_MAX: f64 = 1e10;

impl LnBlackbody {
    fn new() -> Self {
        let w0 = T_TABLE_MIN.ln();
        let h = 1.0 / 128.0;
        let count = ((LN_TABLE_T_MAX.ln() - w0) / h).ceil() as usize + 1;
        let nodes = (0..count)
            .map(|k| {
                let (f, s) = ln_blackbody_xyz((w0 + h * k as f64).exp());
                [f[0], f[1], f[2], s[0], s[1], s[2]]
            })
            .collect();
        Self {
            w0,
            h,
            inv_h: 1.0 / h,
            nodes,
        }
    }

    /// The least and the largest w the table answers for.
    pub fn domain(&self) -> (f64, f64) {
        (self.w0, self.w0 + self.h * (self.nodes.len() - 1) as f64)
    }

    /// ln X, ln Y, ln Z at w = ln T, for w within [`LnBlackbody::domain`].
    #[inline]
    pub fn at(&self, w: f64) -> [f64; 3] {
        let x = (w - self.w0) * self.inv_h;
        let k = (x as usize).min(self.nodes.len() - 2);
        let s = x - k as f64;
        let (s2, s3) = (s * s, s * s * s);
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = (s3 - 2.0 * s2 + s) * self.h;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = (s3 - s2) * self.h;
        let (a, b) = (&self.nodes[k], &self.nodes[k + 1]);
        std::array::from_fn(|c| h00 * a[c] + h10 * a[c + 3] + h01 * b[c] + h11 * b[c + 3])
    }
}

/// The table, built on first use (about 40 ms) and shared.
pub fn ln_blackbody() -> &'static LnBlackbody {
    static TABLE: std::sync::OnceLock<LnBlackbody> = std::sync::OnceLock::new();
    TABLE.get_or_init(LnBlackbody::new)
}

/// The linear light an output pixel shows, after the exposure `gain`: the map over footprint `f`
/// seen with shift `g`, under the map's colour rule. Not clipped above; under `blackbody` brought
/// into the display's gamut below ([`lift_negatives`]). NaN when it is not known.
pub fn light(sky: &SkyMap, f: Footprint, g: f64, gain: f32) -> [f32; 3] {
    match sky.colour() {
        ColourRule::Map => shade(sky.sample(f), g, gain),
        // Exactly the old arithmetic at g = 1, so that a film at rest is the same to the bit.
        ColourRule::Blackbody if !needs_shift(g) => shade(sky.sample(f), g, gain),
        ColourRule::Blackbody => {
            let shifted = sky.sample_shifted(f, g);
            // ---- MICROWAVE BACKGROUND GOES HERE ---------------------------------------------
            // The sky behind the map's light: `sky_colour::model().shifted_background_rgb(
            // BACKGROUND_KELVIN, g, scale)` added to `shifted`, in map units, where `scale` turns
            // absolute radiance (W m^-2 sr^-1) into the map's units. That scale is not known (the
            // maps carry no photometric calibration), so the term is left out and the run says so
            // when it would be seen (`background_note`). It belongs before the exposure, which is
            // a gain on all of the light.
            // ----------------------------------------------------------------------------------
            let exposed = shifted.map(|c| c * f64::from(gain));
            lift_negatives(exposed).map(|c| c as f32)
        }
    }
}

/// A linear colour with its negative channels brought to zero by moving it toward the grey of
/// the same luminance, with the library's `into_display_gamut`; unchanged when no channel is
/// negative, and NaN for NaN. Channels above 1 are left for the encoder to clip (see the module
/// comment for why).
pub fn lift_negatives(rgb: [f64; 3]) -> [f64; 3] {
    // Not below zero: nothing to do. This also passes +infinity (an overflow, white) through.
    if rgb.iter().all(|&c| c >= 0.0) {
        return rgb;
    }
    if rgb.iter().any(|c| !c.is_finite()) {
        return [f64::NAN; 3];
    }
    // Scaled so that no channel exceeds 1: the library's function then has no channel above 1
    // to bring down, and a luminance below 1 (luminance is a mean of the channels with positive
    // weights, so at most the largest), so it only lifts the negative channels, keeping the
    // luminance. Its move toward grey is the same at any scale, and the result is scaled back.
    let k = rgb.iter().fold(1.0f64, |m, &c| m.max(c));
    into_display_gamut(rgb.map(|c| c / k)).map(|c| c * k)
}

/// The shift at which the microwave background, a blackbody at [`BACKGROUND_KELVIN`], reaches
/// [`VISIBLE_LUMINANCE`]: g = 288.7, a 787 K blackbody
/// (`test_the_microwave_background_is_reported_past_the_shift_at_which_an_eye_would_see_it`).
pub fn background_visible_g() -> f64 {
    let luminance = |g: f64| K_M * sky_colour::model().blackbody_xyz(BACKGROUND_KELVIN * g)[1];
    // The luminance rises monotonically with g; bisect in ln g between 1 and 10^4.
    let (mut lo, mut hi) = (0.0f64, 4.0 * std::f64::consts::LN_10);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if luminance(mid.exp()) < VISIBLE_LUMINANCE {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (0.5 * (lo + hi)).exp()
}

/// What the run says about the microwave background, given the film's largest shift: nothing
/// when the background would not be seen.
pub fn background_note(largest_g: f64) -> Option<String> {
    let threshold = background_visible_g();
    // A film with no sky pixel has -infinity here, and NaN never arises (`crate::tally`).
    if largest_g.is_nan() || largest_g <= threshold {
        return None;
    }
    let kelvin = BACKGROUND_KELVIN * largest_g;
    let xyz = sky_colour::model().blackbody_xyz(kelvin);
    let (x, y) = xy(xyz);
    let luminance = K_M * xyz[1];
    Some(format!(
        "The largest shift in this film, g = {largest_g:.1}, is past g = {threshold:.0}, at which \
         the cosmic microwave background ({BACKGROUND_KELVIN} K) seen with that shift reaches \
         {VISIBLE_LUMINANCE} cd/m^2, the lower limit of colour vision (CIE 191:2010). The film \
         omits it: adding it needs the star map in absolute units, which are not known. Where g \
         = {largest_g:.1} it would be a glow filling that part of the sky, the colour of a \
         {kelvin:.0} K blackbody, (x, y) = ({x:.3}, {y:.3}) ({}), of luminance {} cd/m^2, and it \
         grows steeply with g. At such shifts the real sky's visible light would be dominated by \
         the infrared glow of dust, which the map does not hold either, so the film is not right \
         there in what dominates the picture.",
        colour_word(kelvin),
        significant(luminance)
    ))
}

/// A plain word for the colour of a blackbody at `kelvin`, for a sentence.
fn colour_word(kelvin: f64) -> &'static str {
    match kelvin {
        k if k < 1300.0 => "a dull deep red",
        k if k < 2200.0 => "orange-red",
        k if k < 3500.0 => "orange",
        k if k < 5000.0 => "yellowish white",
        k if k < 8000.0 => "white",
        _ => "bluish white",
    }
}

/// Three significant figures, in plain or exponent form as the size needs.
fn significant(v: f64) -> String {
    if v != 0.0 && (v.abs() >= 1e5 || v.abs() < 1e-2) {
        format!("{v:.2e}")
    } else {
        let decimals = (2 - v.abs().log10().floor() as i32).clamp(0, 4) as usize;
        format!("{v:.decimals$}")
    }
}

/// What the run says about the colour rule, for its end.
pub fn explain(rule: ColourRule) -> Vec<String> {
    match rule {
        ColourRule::Map => vec![
            "colour: map (--colour map). Each pixel is the star map's own colour times g^4, the \
             total energy over all wavelengths, applied to every channel alike. The hue does not \
             follow the shift, and the brightness is not what an eye sees: g^4 counts ultraviolet \
             and infrared an eye does not see. This is the renderer's old rule, kept for \
             comparison."
                .into(),
        ],
        ColourRule::Blackbody => vec![
            "colour: blackbody (--colour blackbody). What is assumed: the light of each texel of \
             the star map is a blackbody's, at the temperature its colour implies (the CIE's \
             correlated colour temperature), times whatever tint is left over. A shift g turns a \
             blackbody at temperature T into exactly a blackbody at g T, so each shifted texel is \
             drawn as the blackbody at g T with the same tint, as a standard human eye (CIE 1931) \
             sees it: blueshifted light is bluer and brighter and redshifted light redder and \
             dimmer, by what the eye sees in its own band, not by the total energy."
                .into(),
            "What that gets wrong. A star is not a blackbody: it has absorption lines, the Balmer \
             jump and less ultraviolet, so under a redshift the model draws it too bright (2.4 \
             times for the Sun and 4.4 times for Vega at g = 0.5). The diffuse glow of the Milky \
             Way is the light of stars of many temperatures, reddened by dust, and the model makes \
             such a mixture too dim under any shift: by about half at g = 2 and three quarters by \
             g = 10. Under a shift beyond about 3 to 5 either way, the light an eye would see was \
             emitted in the infrared or the ultraviolet, where the real sky is dust, gas and the \
             cosmic backgrounds, none of which the map holds: there the picture shows a sky of \
             blackbody stars, not the real sky. The map's brightest stars are clipped, which gives \
             their cores the wrong temperature, and the map's colours are plausible rather than \
             measured. The model is good for single stars from g = 0.9 to 2 (brightness within \
             about 20 %, colour within about 0.015 u'v') and for the diffuse glow only from 0.8 \
             to 1.25. (From the crate sky-colour, \"The model, for the viewer\".)"
                .into(),
        ],
    }
}

/// The false colours of `--show-model-range`, one line a class.
pub fn legend() -> Vec<String> {
    let mut out = vec![
        "--show-model-range: the picture shows, for every sky pixel, the class of its shift g, \
         not its light; the dark region, unresolved and under-sampled pixels keep their colours"
            .to_string(),
    ];
    for class in Class::ALL {
        out.push(format!("  #{}  {}", class.hex(), class.meaning()));
    }
    out
}
