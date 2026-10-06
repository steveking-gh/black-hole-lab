//! What colour and brightness shifted starlight has to an eye.
//!
//! Light reaching an observer near a black hole is shifted in frequency by g = nu_observed /
//! nu_emitted (the specification, section 4.6). This crate says what that does to a pixel of the
//! star map as a human eye, or a camera made to match one, would see it: the linear BT.709 RGB
//! of the shifted light, its brightness against the unshifted light, and how far the answer can
//! be trusted.
//!
//! # The model, for the viewer
//!
//! This section is written to be quoted, whole or in part, where a film made with the model is
//! shown.
//!
//! **What is assumed.** The light of each pixel of the star map is taken to be the light of a
//! blackbody, at the temperature its colour implies (the CIE's correlated colour temperature),
//! times whatever tint is left over. A frequency shift turns the light of a blackbody at
//! temperature T into exactly the light of a blackbody at g T, with no other factor, so the
//! shifted pixel is the blackbody at g T with the same tint, as a standard human observer (CIE
//! 1931) sees it.
//!
//! **What that gets right.** The colour and the visible brightness of a star under a shift, to
//! the accuracy with which the star is a blackbody. Blueshifted light is bluer and redshifted
//! light redder, and the brightness is what the eye sees in its own band, not the total energy:
//! for a star like the Sun seen with g = 389 the energy arriving is 2.3 x 10^10 times greater,
//! but most of it is ultraviolet and X-rays, and the eye sees it 7800 times brighter. As g grows
//! every star tends to the same pale blue, (x, y) = (0.2399, 0.2340), the colour of the
//! Rayleigh-Jeans tail, and brightens in proportion to g; as g falls every star reddens and
//! fades exponentially.
//!
//! **What it gets wrong**, with the size of each error where it can be estimated:
//!
//! 1. *A star is not a blackbody.* It has absorption lines, the Balmer jump (a drop by a factor
//!    of 2 to 3 at 365 nm in A-type stars), molecular bands in cool stars, and far less
//!    ultraviolet than a blackbody of its colour. Under a redshift the visible band shows light
//!    that was emitted at g times its wavelength, so at g = 0.5 the eye sees what the star
//!    emitted at about 190 to 390 nm. Measured against real spectra (the HST CALSPEC spectra of the
//!    Sun, of Vega and of the solar twin 18 Sco, fetched 2026-09-28 and integrated outside this
//!    crate with its own method):
//!
//!    | g | Sun: brightness, colour | Vega: brightness, colour |
//!    | ---: | --- | --- |
//!    | 0.5 | model 2.4 times too bright, du'v' 0.069 | 4.4 times too bright, 0.014 |
//!    | 0.67 | +34 %, 0.023 | +94 %, 0.087 |
//!    | 0.8 | +2 %, 0.020 | +4 %, 0.074 |
//!    | 1.25 | +3 %, 0.003 | +1 %, 0.003 |
//!    | 2 | 0 %, 0.006 | -17 %, 0.011 |
//!    | 3 | -15 % (at 3.2), 0.005 | -20 %, 0.003 |
//!    | 10 to 100 | (no data) | -23 to -25 %, 0.001 |
//!
//!    du'v' is the distance between the model's colour and the true one in the CIE 1976 u'v'
//!    diagram; about 0.004 is a just-noticeable difference, 0.02 is plainly visible side by side.
//!    Vega's colour implies 14900 K where its effective temperature is 9600 K, and the star map's
//!    Vega implies 13900 K: the map, like this model, works with the colour temperature, which is
//!    what makes a hot star's redshift so wrong here.
//! 2. *The diffuse glow of the Milky Way is many stars.* It is the sum of stars of every
//!    temperature, reddened by dust, and a sum of blackbodies is not a blackbody: its spectrum is
//!    broader, so it brightens more under a blueshift (its cool members gain most) and fades less
//!    under a redshift (its hot members lose least). For equal visible luminance from 3500 K and
//!    10000 K stars (colour temperature 5443 K, Duv -0.0054, which the model reads as one
//!    blackbody), against the true sum of the two shifted blackbodies
//!    (`test_the_two_temperature_mixture_errs_as_documented`):
//!
//!    | g | model's brightness against the truth | colour error du'v' |
//!    | ---: | ---: | ---: |
//!    | 0.25 | 0.005 (-99.5 %) | 0.117 |
//!    | 0.5 | 0.26 (-74 %) | 0.068 |
//!    | 0.8 | 0.92 (-8 %) | 0.027 |
//!    | 1.25 | 0.86 (-14 %) | 0.019 |
//!    | 2 | 0.53 (-47 %) | 0.027 |
//!    | 3 | 0.38 (-62 %) | 0.022 |
//!    | 10 | 0.25 (-75 %) | 0.012 |
//!    | 100 | 0.22 (-78 %) | 0.009 |
//!
//!    The model always makes such a mixture too dim, in both directions: by a factor of 2 at
//!    g = 2 and of 4 by g = 10. This is the largest error of the model within the range where it
//!    is otherwise good, and it applies to most of the sky's light that is not a separate star.
//!    The mixture is a wide one; a real population's spread of temperatures is of the same
//!    order, but the size for the real Milky Way has not been computed.
//! 3. *Dust reddening is not a change of temperature.* A reddened star has a hot star's spectrum
//!    times a smooth extinction, not a cool star's spectrum. For extinction taken as a power law,
//!    tau proportional to lambda^-1.3, the model does well: a 10000 K star behind 3 magnitudes of
//!    visual extinction reads as 4078 K, and the model makes it too bright by 1 % at g = 0.8 and
//!    1.25, 6 % at 2, 16 % at 10 and 28 % at 0.5 (colour within 0.003 u'v' for g from 0.8 to 10).
//!    Real extinction has a bump at 217.5 nm and a different slope in the infrared, which these
//!    numbers do not include.
//! 4. *Under a large blueshift the visible light was emitted in the infrared, and under a large
//!    redshift in the ultraviolet, where the real sky is not a blackbody version of the visible
//!    sky.* At g the eye sees what was emitted at about 380 g to 780 g nm. Starlight dominates the
//!    real sky out to about 3 to 5 micrometres (g up to about 5). Beyond that the interstellar
//!    medium's own emission takes over: aromatic-molecule bands near 3 to 12 micrometres, warm
//!    dust, then from about 50 micrometres (g near 100) cold dust at about 18 to 20 K and the
//!    cosmic infrared background, and from about 1 mm (g near 2000) the cosmic microwave
//!    background. None of this is in the star map. The size can be estimated roughly (these figures
//!    are from memory of published far-infrared surveys, IRAS and COBE-DIRBE, not fetched or
//!    checked here): at high galactic latitude the sky at 100 to 240 micrometres has a surface
//!    brightness of about 1 MJy/sr, which seen with g = 389 is a luminance of order 10^4 cd/m^2,
//!    the colour of a 7000 K body (dust at 18 K times 389), and brighter still along the galactic
//!    plane, where the map's dark dust lanes are. The blueshifted starlight of the map at the same
//!    g is of order 0.1 to 1 cd/m^2, and the microwave background 10 cd/m^2. So at g of a few
//!    hundred the true sky would be a bright glow of dust in which the map's dust lanes are the
//!    brightest features, and the model's picture is wrong in what dominates it, not in a
//!    percentage. Under redshift, at g = 0.5 the visible band shows 190 to 390 nm, where stars
//!    depart from blackbodies most (item 1), and below g = 0.25 the Lyman limit at 91 nm, beyond
//!    which the interstellar gas absorbs almost everything.
//! 5. *The map is clipped at 1.* Its brightest stars (magnitude 1 and brighter) are flat-topped at
//!    1 channel by channel, so their cores have lost their true brightness and part of their
//!    colour: Betelgeuse's core is (1, 0.94, 0.69) where its unclipped wings are (1, 0.65, 0.36),
//!    which the model reads as a 5340 K star instead of a 3950 K one. Under a shift those
//!    cores take the wrong temperature's brightness factor as well as the wrong colour.
//! 6. *The map's colours are "plausible, not colorimetric"* (sky/maps/README.md). NASA gave the
//!    bright stars a blackbody's colour for their B-V index, and those colours are on the Planckian
//!    locus as this crate computes it: the unclipped wings of 15 bright stars, read from the 8k
//!    galactic map as linear BT.709, lie within Duv 0.0042 of the locus (read as sRGB-encoded
//!    instead, the hot ones fall 0.01 to 0.05 below it, so the values are linear). The Gaia stars'
//!    colours were balanced by eye, and about a quarter of the faint stars carry no colour and are
//!    drawn white: to the model they are 6504 K stars, which is a default of the map, not a
//!    measurement.
//!
//! **Where the model is good.** For individual stars, g from 0.9 to 2: brightness within about 20 %
//! and colour within about 0.015 u'v' for both the Sun and Vega (items 1 and 3); a Sun-like star
//! stays good down to g = 0.8, but at 0.8 an A star's Balmer jump (365 nm) enters the visible band
//! and Vega's colour is off by 0.07. For the diffuse glow, only g from about 0.8 to 1.25, and even
//! there its brightness is some 10 % low and its colour visibly off (item 2). Beyond g of about 3
//! to 5 the real sky's visible light at that g is increasingly not starlight at all (item 4); below
//! g of about 0.8 stars' ultraviolet departs from any blackbody (item 1). Outside those ranges the
//! film shows what a sky of blackbody stars would look like, which is still far closer to the truth
//! than the old rule (the map's colour times g^4, which at g = 389 overstates the visible
//! brightness of a Sun-like star three million times).
//!
//! **What would do better.** (a) The bright stars as point sources from a catalogue with
//! measured effective temperatures, or better spectral types, each drawn with a model
//! atmosphere's spectrum (for example the CALSPEC or Kurucz grids) instead of a blackbody; that
//! removes items 1, 5 and 6 for the stars that matter most. (b) For the diffuse light, a
//! population model (a sum of spectra with a known spread of temperatures) or the power-law
//! tinted family described in `shift`. (c) All-sky maps in the infrared and the microwave
//! (COBE-DIRBE, IRAS/IRIS, Planck, WISE) as further layers, each shifted with its own spectrum;
//! without them no film with g beyond a few can be right.
//!
//! # The visible-band factor against g^4
//!
//! The luminance of a blackbody seen with shift g, over its unshifted luminance, Y(gT) / Y(T)
//! ([`Model::visible_factor`]), against the bolometric g^4 the renderer used
//! (`test_the_visible_factors_are_the_documented_ones`):
//!
//! | g | 3000 K | 5778 K | 10000 K | g^4 |
//! | ---: | ---: | ---: | ---: | ---: |
//! | 1/8 | 1.49e-23 | 2.03e-13 | 2.33e-8 | 2.44e-4 |
//! | 1/4 | 3.98e-11 | 2.24e-6 | 4.24e-4 | 3.91e-3 |
//! | 1/2 | 2.56e-4 | 1.18e-2 | 6.92e-2 | 6.25e-2 |
//! | 2 | 72.5 | 10.5 | 4.72 | 16 |
//! | 10 | 4020 | 160 | 42.8 | 1.00e4 |
//! | 100 | 6.17e4 | 1980 | 484 | 1.00e8 |
//! | 389 | 2.48e5 | 7840 | 1900 | 2.29e10 |
//!
//! Under redshift the eye's factor falls far faster than g^4 (the visible band slides down the
//! Wien tail), under blueshift far slower (the energy leaves the visible band for the
//! ultraviolet): at large g it grows only as g, the Rayleigh-Jeans law. A cool star brightens most
//! under blueshift, a hot one least.
//!
//! How dark is dark: the visible radiance of a blackbody against a 5778 K one's is 2.5e-4 at
//! 2000 K, 1.3e-6 at 1400 K, 1.5e-9 at 1000 K, 4.4e-12 at 800 K and 2.5e-19 at 500 K. In a
//! 10-bit sRGB-encoded video the first code above black is 7.6e-5 of white in linear light, so
//! beside a Sun-like star shown at white, a surface cooler than about 1820 K is black, and one at
//! 1000 K would need 16 more stops of exposure, which would put the star 5 x 10^4 times above
//! white.
//!
//! # The microwave background
//!
//! [`Model::shifted_background_rgb`] gives the light of a uniform blackbody background seen with
//! shift g: the cosmic microwave background (2.7255 K) at g = 389 is a 1060 K blackbody, dull
//! orange-red, (x, y) = (0.644, 0.352), of luminance 10.2 cd/m^2, filling the sky. It grows steeply
//! with g, the visible band climbing its Wien tail: 2e-8 cd/m^2 at g = 200, 0.015 at 300, 1500 at
//! 500, 1.3e7 at 1000. Adding it to a picture needs the map in absolute units, which the map does
//! not give (sky/maps/README.md, "Absolute scale"): the function takes the scale from absolute
//! radiance to map units as its argument, and until that scale is measured it cannot be used
//! honestly. Item 4 above says why it is not the whole story either.
//!
//! # For the renderer
//!
//! - [`model`] builds the tables once (about 30 ms in a release build) and shares them.
//! - [`needs_shift`]`(g)` is false only at g == 1.0, where [`Model::shift`] returns its input
//!   unchanged: a film of flat space at rest comes out identical to the bit, whether the renderer
//!   skips the model or not.
//! - [`Model::shift`] / [`Model::shift_f32`]: the map's linear RGB and g in, the observer's linear
//!   RGB out, before exposure, not clipped, possibly with negative channels (outside BT.709) and
//!   possibly far above 1. It replaces the g^4 entirely: the brightness change is inside it.
//! - NaN out means "not known": a g that is negative, infinite or NaN, or a pixel that is not
//!   light. A pixel of the map is always light, and black stays black.
//! - [`Model::implied_temperature`] gives each pixel's temperature and Duv;
//!   [`Implied::within_cct_domain`] is false beyond the CIE's limit |Duv| <= 0.05, where the
//!   temperature, and so the shifted colour, is a continuation rather than an inference. The
//!   primaries blue and magenta are beyond it; the map's stars are not.
//! - [`into_display_gamut`], after exposure, brings a colour into [0, 1]^3 keeping luminance, then
//!   hue. It is a display limit, not a statement about the light, and is kept apart from the model.
//! - Cost, measured in a release build on the owner's machine (one thread): 122 ns a pixel, of
//!   which 75 ns finds the temperature and 52 ns forms the ratio (`tests/report.rs`). About 4 s of
//!   one core for a 33-million-pixel frame. The temperature depends on the pixel's chromaticity
//!   only, so a renderer that samples texels without filtering can compute it once per texel of
//!   the map and call [`Model::shift_at`].
//!
//! # Modules
//!
//! - [`cie1931`]: the CIE's colour-matching functions, with their provenance and checks.
//! - [`planck`]: constants, Planck's law, the shift of a blackbody, the eye's integral.
//! - [`colorimetry`]: XYZ and BT.709 RGB, chromaticity diagrams.
//! - [`table`]: the fast blackbody table.
//! - [`temperature`]: the temperature a colour implies.
//! - [`shift`]: the shift itself, the display gamut, the background hook.
//!
//! `tests/report.rs` prints every number quoted here; `tests/pictures.rs` draws the locus and
//! the charts of shifted stars. Both are `#[ignore]`d.

pub mod cie1931;
pub mod colorimetry;
pub mod planck;
pub mod shift;
pub mod table;
pub mod temperature;

pub use shift::{Model, into_display_gamut, needs_shift};
pub use temperature::{CCT_DOMAIN_DUV, Implied};

/// The model, built on first use and shared by every caller.
pub fn model() -> &'static Model {
    static MODEL: std::sync::OnceLock<Model> = std::sync::OnceLock::new();
    MODEL.get_or_init(Model::new)
}
