//! The star map: where a direction lands on it, and what it shows there, filtered to the size of
//! the pixel asking.
//!
//! # Where a direction lands
//!
//! A map is equirectangular in its own frame, celestial (ICRS) or galactic, with longitude
//! increasing to the left (sky/maps/README.md). A unit direction (x, y, z) in the map's frame
//! lands at the frame coordinates
//!
//!     u = W (0.5 - atan2(y, x) / 2 pi)
//!     v = H (0.5 - atan2(z, sqrt(x^2 + y^2)) / pi)
//!
//! with pixel centres at half-integers: the same formulae as the observer-sky grid
//! (`sky_format::Grid::frame_coordinates`), so the bilinear reader of `crate::bilinear` serves both.
//!
//! A bundle's direction `d` is in its far-sky frame. The manifest's `axes_in_icrs` takes it to
//! ICRS, and a galactic map needs one more rotation, by the Hipparcos matrix whose rows are the
//! galactic axes in ICRS. [`orientation`] composes the two into one matrix, applied once per ray.
//! With the default galactic preset and a galactic map that matrix is the identity to rounding;
//! nothing here assumes it.
//!
//! # Filtering: a rip-map
//!
//! An output pixel covers a patch of sky, and the patch can cover one texel of the map or
//! thousands: near the map's poles a patch a pixel high is hundreds of texels wide, and a lensing
//! map will squeeze whole regions of sky into a few pixels. Reading one texel (or four, bilinearly)
//! there aliases: stars flicker in and out as the view moves. Averaging a fixed neighbourhood blurs
//! where the patch is small. The filter has to fit the patch.
//!
//! The usual answer is a mip pyramid: the map halved again and again, and each pixel read from
//! the level whose texels are the size of its patch, interpolating between the two nearest levels
//! (trilinear filtering). A plain pyramid halves both axes together, so it fits a patch that
//! covers as many texels across as down. Here the patches are long and thin, and in a known
//! direction: at map latitude b a texel spans 1/cos(b) times less sky across than down, so a patch
//! round on the sky is 1/cos(b) texels wide for every texel high. A plain pyramid must choose
//! between the width (blurring the map's rows together near its poles) and the height (aliasing
//! along them).
//!
//! So this is a rip-map: a pyramid that halves the two axes independently. Level (lx, ly) is the
//! map halved lx times across and ly times down, and a pixel reads the level whose texels match
//! its patch's width and height separately, interpolating in both level indices and bilinearly in
//! each level (up to four levels, sixteen texels). It costs four times the map's memory, where a
//! plain pyramid costs a third more: 1.6 GB for an 8k map, 6.4 GB for a 16k one. Anisotropic
//! probing (several trilinear reads along the patch's long axis, as graphics cards do) would cost
//! less memory but caps the elongation it can follow, and at the map's pole row the elongation is
//! W / pi, about 2600 at 8k.
//!
//! The patch size comes from the directions of neighbouring output pixels, not from assumptions
//! about the view: where a pixel and its right-hand and lower neighbours land on the map says how
//! many texels across and down one output pixel spans. For the default view of the default
//! map that is exactly one texel each way at every latitude, because output and map share their
//! grid, so the level is (0, 0) and the map comes through unblurred.
//!
//! # Building the levels
//!
//! Each level is made from the one before by averaging pairs of texels, exactly tiling the level
//! below: the interval of cell k of a level n cells long covers [k n / m, (k + 1) n / m) of the
//! level below it, m = n / 2 cells long, weighted by overlap, so an odd count is handled without
//! dropping or doubling a texel. Because the cells tile the row, a cell never straddles the seam,
//! and because they tile the column, a cell never straddles a pole: the seam and the poles need no
//! special case when building. Down the map, the texels of two rows cover different amounts of sky,
//! more toward the equator; they are weighted by the solid angle of their overlap,
//! sin(b_top) - sin(b_bottom), so that a level's value near a pole is the mean of the sky it
//! covers and not over-weighted toward the pole. Reading a level uses the pole and seam rules of
//! `crate::bilinear`.
//!
//! # Shifting under the blackbody model
//!
//! Under `--colour blackbody` the map is built with each texel of every level also in the form
//! the model needs ([`SkyMap::with_colour`]: the temperature its colour implies and its tint,
//! found once when the map is loaded), and [`SkyMap::sample_shifted`] reads the same texels with
//! the same weights as [`SkyMap::sample`], shifting each before blending. Why in that order, and
//! what error the pre-averaged levels leave, is measured in `crate::colour`, "Filtering and
//! shifting".
//!
//! # Fallbacks in the footprint, and what each gives
//!
//! - *A missing neighbour* (not sky) gives a step of zero along that axis, so the patch is
//!   measured by the other neighbour alone, or taken as one texel with neither. That is a sharper
//!   read than the truth wherever the pixel's patch is larger than a texel: plausible, not known.
//!   `crate::render` supplies the other-side neighbour where it can (left for right; above for
//!   below, when judged) and, when judged, marks a pixel with no sky neighbour along an axis as
//!   under-sampled rather than read it one texel wide. Unjudged, the old rule stands.
//! - *Spans that are not finite* fall back to the whole map. Unreachable: a far-sky direction is
//!   a finite unit vector (`crate::field`), so `place` gives finite coordinates and their
//!   differences are finite. The guard stays so that a defect elsewhere cannot index out of range.
//! - *Spans larger than the map* are clamped to it. Right: a patch wider than the map at its
//!   latitude covers all of it, and the coarsest level is the mean of what it covers.

use std::f64::consts::PI;

use sky_colour::colorimetry::{rgb_to_xyz, xyz_to_rgb};

use crate::bilinear::taps;
use crate::colour::{ColourRule, ln_blackbody};
use crate::parallel::for_each_band;

/// A 3 x 3 matrix, row-major.
pub type Mat3 = [[f64; 3]; 3];

pub fn apply(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|r| m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2])
}

/// The coordinate system a star map is drawn in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapFrame {
    /// ICRS: right ascension and declination.
    Celestial,
    /// Galactic longitude and latitude, from ICRS by the Hipparcos matrix.
    Galactic,
}

impl MapFrame {
    /// The frame a NASA map is drawn in, from its file name: the galactic ones end in `_gal`.
    pub fn from_file_name(path: &std::path::Path) -> Self {
        let stem = path
            .file_stem()
            .map(|s| s.to_string_lossy().to_ascii_lowercase())
            .unwrap_or_default();
        if stem.ends_with("_gal") || stem.contains("_gal_") {
            Self::Galactic
        } else {
            Self::Celestial
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Celestial => "celestial",
            Self::Galactic => "galactic",
        }
    }

    /// The rotation that takes an ICRS vector into this frame.
    fn icrs_to_map(self) -> Mat3 {
        match self {
            Self::Celestial => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            // The rows of the Hipparcos matrix are the galactic axes written in ICRS, which is
            // exactly what the galactic far-sky preset lists; one copy of the numbers serves both.
            Self::Galactic => {
                let axes = sky_format::FarSky::galactic().axes_in_icrs;
                [axes.x, axes.y, axes.z].map(|row| row.map(|c| c.0))
            }
        }
    }
}

/// The rotation from a bundle's far-sky frame into a map's frame: first to ICRS by the far-sky
/// axes (they are the columns of that matrix), then into the map's frame.
pub fn orientation(far_sky: &sky_format::FarSky, map: MapFrame) -> Mat3 {
    let a = &far_sky.axes_in_icrs;
    let to_icrs: Mat3 = std::array::from_fn(|r| [a.x[r].0, a.y[r].0, a.z[r].0]);
    let to_map = map.icrs_to_map();
    std::array::from_fn(|r| {
        std::array::from_fn(|c| (0..3).map(|k| to_map[r][k] * to_icrs[k][c]).sum())
    })
}

/// One level of the rip-map: an equirectangular image of linear RGB.
#[derive(Debug, Clone)]
struct Level {
    width: usize,
    height: usize,
    /// This level's size over level 0's, across and down, for scaling a place on level 0.
    scale: (f64, f64),
    texels: Vec<[f32; 3]>,
    /// For `--colour blackbody`, each texel as the model sees it (see [`shift_forms`]); empty
    /// under the map's own colour.
    shift: Vec<ShiftForm>,
}

/// A texel as the blackbody model sees it: `[ln T, a_X, a_Y, a_Z]`, T the temperature (K) its
/// colour implies and a_c = ln P_c - ln B_c(T) the logarithm of its tint, P its tristimulus
/// values and B those of the blackbody at T. Seen with shift g the texel is then
/// exp(a_c + ln B_c(g T)): the model's P_c B_c(g T) / B_c(T), with no RGB to convert and one
/// lookup of the blackbody, at g T, instead of two.
///
/// ln T is +infinity for a colour at or beyond the hot end of the locus (a_c is then ln P_c),
/// -infinity for a black texel, and NaN for one that is not the colour of any light.
type ShiftForm = [f32; 4];

impl Level {
    /// The bilinear value at level-0 frame coordinates (u, v).
    fn bilinear(&self, u: f64, v: f64) -> [f32; 3] {
        let u = u * self.scale.0;
        let v = v * self.scale.1;
        let t = taps(self.width, self.height, u, v);
        let mut out = [0.0f32; 3];
        for k in 0..4 {
            let w = t.weight[k] as f32;
            if w == 0.0 {
                continue;
            }
            let texel = self.texels[t.index[k]];
            for c in 0..3 {
                out[c] += w * texel[c];
            }
        }
        out
    }
}

/// A star map and its rip-map levels.
#[derive(Debug, Clone)]
pub struct SkyMap {
    width: usize,
    height: usize,
    /// How many levels there are across (index lx) and down (index ly).
    across: usize,
    down: usize,
    /// Level (lx, ly) is at `ly * across + lx`.
    levels: Vec<Level>,
    /// How shifted light is coloured over this map; `Blackbody` exactly when every level carries
    /// its texels' temperatures.
    colour: ColourRule,
}

/// Where a direction lands on a map, and how many level-0 texels one output pixel spans there
/// across and down.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Footprint {
    pub u: f64,
    pub v: f64,
    pub across: f64,
    pub down: f64,
}

impl SkyMap {
    /// Builds the rip-map of a `width` x `height` map given row by row, top row first, each row
    /// left to right, as linear RGB, for the map's own colour times g^4 (`--colour map`).
    /// `threads` share the building. (The renderer itself calls [`SkyMap::with_colour`].)
    #[cfg(test)]
    pub fn new(width: usize, height: usize, texels: Vec<[f32; 3]>, threads: usize) -> Self {
        Self::with_colour(width, height, texels, threads, ColourRule::Map)
    }

    /// As [`SkyMap::new`], for the colour rule `colour`. Under `Blackbody` the temperature every
    /// texel of every level implies is found here, once, so that a frame does not search for it
    /// per pixel (see [`SkyMap::sample_shifted`]).
    pub fn with_colour(
        width: usize,
        height: usize,
        texels: Vec<[f32; 3]>,
        threads: usize,
        colour: ColourRule,
    ) -> Self {
        assert_eq!(
            texels.len(),
            width * height,
            "a map needs one texel per pixel"
        );
        let sizes = |mut n: usize| {
            let mut out = vec![n];
            while n > 1 {
                n /= 2;
                out.push(n);
            }
            out
        };
        let widths = sizes(width);
        let heights = sizes(height);
        let (across, down) = (widths.len(), heights.len());
        let mut levels: Vec<Level> = Vec::with_capacity(across * down);
        let mut full = Some(texels);
        for (ly, &h) in heights.iter().enumerate() {
            // Each row of levels starts from the full-width level above it, halved down; the
            // two halvings commute, since each is a fixed weighting along its own axis.
            let first = match full.take() {
                Some(texels) => Level {
                    width,
                    height,
                    scale: (1.0, 1.0),
                    texels,
                    shift: Vec::new(),
                },
                None => halve_down(&levels[(ly - 1) * across], h, threads),
            };
            levels.push(first);
            for &w in &widths[1..] {
                let next = halve_across(levels.last().expect("a level"), w, threads);
                levels.push(next);
            }
        }
        if colour == ColourRule::Blackbody {
            // Built here, with the map, rather than by the first pixel of the first frame.
            ln_blackbody();
            for level in &mut levels {
                level.shift = shift_forms(&level.texels, threads);
            }
        }
        Self {
            width,
            height,
            across,
            down,
            levels,
            colour,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    /// The colour rule this map was built for.
    pub fn colour(&self) -> ColourRule {
        self.colour
    }

    /// The bytes the levels hold: their texels, and their temperatures when there are any.
    pub fn bytes(&self) -> (usize, usize) {
        self.levels.iter().fold((0, 0), |(t, k), level| {
            (
                t + std::mem::size_of_val(level.texels.as_slice()),
                k + std::mem::size_of_val(level.shift.as_slice()),
            )
        })
    }

    pub fn height(&self) -> usize {
        self.height
    }

    #[cfg(test)]
    /// The level-0 texels, row by row.
    pub fn texels(&self) -> &[[f32; 3]] {
        &self.levels[0].texels
    }

    /// Where the unit direction `d`, in the map's frame, lands on the map: level-0 frame
    /// coordinates (u, v).
    pub fn place(&self, d: [f64; 3]) -> (f64, f64) {
        let [x, y, z] = d;
        let u = self.width as f64 * (0.5 - y.atan2(x) / (2.0 * PI));
        // atan2 against the horizontal length rather than asin(z): accurate near the poles, and
        // indifferent to the rounding of |d|.
        let v = self.height as f64 * (0.5 - z.atan2((x * x + y * y).sqrt()) / PI);
        (u, v)
    }

    /// The footprint of an output pixel that lands at `at`, and whose right-hand and lower
    /// neighbours land at `across` and `down`. Either neighbour may be missing (a shadow beside
    /// the pixel); with neither, the patch is taken to be one texel.
    ///
    /// The steps to the neighbours are differences of map coordinates, u taken the short way
    /// round across the seam. Differences and not derivatives: a derivative of the azimuth
    /// taken from the direction vectors is the sine of the step where the step is its angle, and
    /// that 0.2 % at a 64-pixel frame's spacing is enough to mix a sliver of the wrong level in.
    pub fn footprint(
        &self,
        at: (f64, f64),
        across: Option<(f64, f64)>,
        down: Option<(f64, f64)>,
    ) -> Footprint {
        let (w, h) = (self.width as f64, self.height as f64);
        let step = |to: Option<(f64, f64)>| match to {
            Some((u, v)) => {
                // Both u lie in [0, W), so one turn added or taken away finds the short way.
                let du = u - at.0;
                let du = if du > w / 2.0 {
                    du - w
                } else if du < -w / 2.0 {
                    du + w
                } else {
                    du
                };
                (du, v - at.1)
            }
            None => (0.0, 0.0),
        };
        let (across_u, across_v) = step(across);
        let (down_u, down_v) = step(down);
        // The patch's extent along each map axis: the two steps' components on that axis, taken
        // together. A pixel whose steps are one texel along each axis spans one texel each way;
        // one turned 45 degrees on the map spans one texel each way too, rather than the 1.4 a
        // bounding box would say. Near the map's poles the u steps grow without bound, and the
        // clamp takes the coarsest level across.
        // Plain square roots, not `hypot`: the steps are at most a map's width, far from overflow.
        let mut span_u = (across_u * across_u + down_u * down_u).sqrt();
        let mut span_v = (across_v * across_v + down_v * down_v).sqrt();
        if !(span_u.is_finite() && span_v.is_finite()) {
            (span_u, span_v) = (w, h);
        }
        Footprint {
            u: at.0,
            v: at.1,
            across: span_u.min(w),
            down: span_v.min(h),
        }
    }

    /// The map's linear RGB over a footprint: trilinear in the rip-map, in both level indices.
    pub fn sample(&self, f: Footprint) -> [f32; 3] {
        let (lx, fx) = level_of(f.across, self.across);
        let (ly, fy) = level_of(f.down, self.down);
        let mut out = [0.0f32; 3];
        for (a, wa) in [(lx, 1.0 - fx), (lx + 1, fx)] {
            if wa == 0.0 {
                continue;
            }
            for (b, wb) in [(ly, 1.0 - fy), (ly + 1, fy)] {
                if wb == 0.0 {
                    continue;
                }
                let texel = self.levels[b * self.across + a].bilinear(f.u, f.v);
                let weight = (wa * wb) as f32;
                for c in 0..3 {
                    out[c] += weight * texel[c];
                }
            }
        }
        out
    }

    #[cfg(test)]
    /// How many texels, with a weight above zero, [`SkyMap::sample`] reads over a footprint.
    pub fn texels_read(&self, f: Footprint) -> usize {
        let (lx, fx) = level_of(f.across, self.across);
        let (ly, fy) = level_of(f.down, self.down);
        let mut n = 0;
        for (a, wa) in [(lx, 1.0 - fx), (lx + 1, fx)] {
            for (b, wb) in [(ly, 1.0 - fy), (ly + 1, fy)] {
                if wa == 0.0 || wb == 0.0 {
                    continue;
                }
                let level = &self.levels[b * self.across + a];
                let t = taps(
                    level.width,
                    level.height,
                    f.u * level.scale.0,
                    f.v * level.scale.1,
                );
                n += t.weight.iter().filter(|&&w| w > 0.0).count();
            }
        }
        n
    }

    /// The light over a footprint seen with shift `g` under the blackbody model, in the map's
    /// linear units (no exposure), not clipped and possibly with negative channels: each of the
    /// texels [`SkyMap::sample`] reads is shifted at its own temperature, and the shifted texels
    /// are blended with `sample`'s weights. Only for a map built for `ColourRule::Blackbody`.
    ///
    /// Why texel by texel and not the blend: the model is not linear in colour (the shift of a
    /// mean of two colours is not the mean of their shifts; the crate `sky-colour`'s item 2), and
    /// each texel of the map is the smallest patch whose colour the map states. Shifting each
    /// texel the filter reads and then blending is the true order for every texel it reads; what
    /// remains of the other order is inside a coarse level's texels, each the mean of the level-0
    /// texels it covers and shifted at the temperature of that mean (the measurement and the bound
    /// are in `crate::colour`, "Filtering and shifting"). It is also what makes the temperatures
    /// found once per texel usable: the temperature of a blend of texels would have to be searched
    /// for per pixel.
    ///
    /// A texel that is not light makes the pixel NaN, not known. A shift so large that the light
    /// overflows f64 (g beyond about 10^270) makes it infinite, which the encoder shows as white:
    /// it is then more than 10^270 times the texel's light, beyond any display at any exposure.
    pub fn sample_shifted(&self, f: Footprint, g: f64) -> [f64; 3] {
        debug_assert_eq!(self.colour, ColourRule::Blackbody);
        // Not a shift (`crate::field` lets none of these through; a direct caller might).
        if !(g.is_finite() && g >= 0.0) {
            return [f64::NAN; 3];
        }
        let table = ln_blackbody();
        let (w_min, w_max) = table.domain();
        let ln_g = g.ln();
        let (lx, fx) = level_of(f.across, self.across);
        let (ly, fy) = level_of(f.down, self.down);
        // The shifted texels are summed in XYZ and turned into RGB once: the shift is formed in
        // XYZ and the conversion is linear, so this is the blend of the shifted RGB, reordered.
        let mut xyz = [0.0f64; 3];
        for (a, wa) in [(lx, 1.0 - fx), (lx + 1, fx)] {
            if wa == 0.0 {
                continue;
            }
            for (b, wb) in [(ly, 1.0 - fy), (ly + 1, fy)] {
                if wb == 0.0 {
                    continue;
                }
                let level = &self.levels[b * self.across + a];
                let t = taps(
                    level.width,
                    level.height,
                    f.u * level.scale.0,
                    f.v * level.scale.1,
                );
                for k in 0..4 {
                    let w = wa * wb * t.weight[k];
                    if w == 0.0 {
                        continue;
                    }
                    let [ln_t, tint @ ..] = level.shift[t.index[k]];
                    if ln_t == f32::NEG_INFINITY {
                        // Black: no light, shifted or not.
                        continue;
                    }
                    if ln_t.is_nan() {
                        return [f64::NAN; 3];
                    }
                    let ln_t = f64::from(ln_t);
                    let ln_gt = ln_t + ln_g;
                    if ln_gt < w_min {
                        // g T below 10 K: below e^-1675 of the texel, zero in f64, the library's
                        // own rule (`sky_colour::shift`, "Units and the table's ends"). It also
                        // takes g = 0, whose logarithm is -infinity.
                        continue;
                    }
                    let a = tint.map(f64::from);
                    let shifted: [f64; 3] = if ln_t == f64::INFINITY {
                        // A colour at or beyond the locus's hot end, whose a_c is ln P_c: the
                        // Rayleigh-Jeans limit, Q = g, as the library has it.
                        std::array::from_fn(|c| g * a[c].exp())
                    } else if ln_gt <= w_max {
                        let b = table.at(ln_gt);
                        std::array::from_fn(|c| (a[c] + b[c]).exp())
                    } else {
                        // Past the table's top (g T above 10^10 K): the library, slower.
                        let b = sky_colour::model().blackbody_xyz(ln_gt.exp());
                        std::array::from_fn(|c| a[c].exp() * b[c])
                    };
                    for c in 0..3 {
                        xyz[c] += w * shifted[c];
                    }
                }
            }
        }
        if xyz.iter().all(|c| c.is_finite()) {
            xyz_to_rgb(xyz)
        } else {
            // Every texel read is light, so its X, Y, Z are not below zero, Q is positive and g
            // finite: a sum that is not finite has overflowed (see above).
            [f64::INFINITY; 3]
        }
    }
}

/// Each texel as the blackbody model sees it ([`ShiftForm`]), on `threads` threads: the one
/// search for the temperature a texel's colour implies (the library's, 75 ns), made once when the
/// map is loaded instead of once per pixel per frame.
///
/// Its precision, in f32. ln T is below 24 (T up to 10^10 K), where f32's spacing is at most
/// 1.9e-6, so the stored T is within 1e-6 of the implied one (2.4e-7 below 2981 K, where
/// ln T < 8). The tint a_c is formed against the blackbody at that stored T, so at g near 1 the
/// texel comes back as itself whatever the rounding, and the stored T enters only through the
/// difference of the slopes d ln B / d ln T at g T and at T, each at most about c2 / (lambda T)
/// for the longest wavelength that counts: about 60 at the least implied temperature, 300 K, and
/// below 5 from 3000 K up. The shifted texel is then within 1.5e-5 of the model's at 300 K and
/// within 5e-6 from 3000 K up. a_c itself lies within about -25 to 60 (ln B_c(T) runs from about
/// -55 at 300 K to +25 at 10^6 K, and the map's values are at most 1), where f32 rounds to 2e-6 of
/// the texel's light at most. Against the output's step, 1.5e-5 of full scale, these are nothing.
fn shift_forms(texels: &[[f32; 3]], threads: usize) -> Vec<ShiftForm> {
    let model = sky_colour::model();
    let table = ln_blackbody();
    let (_, w_max) = table.domain();
    let mut out = vec![[0.0f32; 4]; texels.len()];
    for_each_band(&mut out, 4096, 16, threads, |rows, band| {
        let first = rows.start * 4096;
        for (k, out) in band.iter_mut().enumerate() {
            let texel = texels[first + k].map(f64::from);
            // A texel that is light has X, Y, Z >= 0 up to rounding; the library clamps that
            // rounding to 0 and so does this (its logarithm is then -infinity, and it adds 0).
            let ln_p = rgb_to_xyz(texel).map(|c| c.max(0.0).ln());
            *out = match model.implied_temperature(texel) {
                Some(implied) if implied.kelvin.is_infinite() => [
                    f32::INFINITY,
                    ln_p[0] as f32,
                    ln_p[1] as f32,
                    ln_p[2] as f32,
                ],
                Some(implied) => {
                    let ln_t = implied.kelvin.ln() as f32;
                    let w = f64::from(ln_t);
                    let b = if w <= w_max {
                        table.at(w)
                    } else {
                        model.blackbody_xyz(w.exp()).map(f64::ln)
                    };
                    [
                        ln_t,
                        (ln_p[0] - b[0]) as f32,
                        (ln_p[1] - b[1]) as f32,
                        (ln_p[2] - b[2]) as f32,
                    ]
                }
                None if texel.iter().all(|&c| c == 0.0) => [f32::NEG_INFINITY; 4],
                // Not the colour of any light (a tristimulus value below zero) or not finite.
                None => [f32::NAN; 4],
            };
        }
    });
    out
}

/// The level whose texels are `span` level-0 texels long, as a whole level and the fraction of
/// the way to the next. Within a thousandth of a level the fraction is snapped away, so that a
/// span of exactly one texel, computed with rounding, reads one level and not two.
fn level_of(span: f64, levels: usize) -> (usize, f64) {
    // The common case, a pixel no larger than a texel, without a logarithm.
    if span <= 1.0 + 1e-3 {
        return (0, 0.0);
    }
    let top = (levels - 1) as f64;
    let lod = span.max(1.0).log2().min(top);
    let whole = lod.floor();
    let fraction = lod - whole;
    let (whole, fraction) = if fraction < 1e-3 {
        (whole, 0.0)
    } else if fraction > 1.0 - 1e-3 {
        (whole + 1.0, 0.0)
    } else {
        (whole, fraction)
    };
    if whole >= top {
        (levels - 1, 0.0)
    } else {
        (whole as usize, fraction)
    }
}

/// For each of `m` cells tiling [0, n): the cells of the level below it overlaps, and by how much
/// of `measure`, which gives a cell's share of the interval [a, b] of the level below.
fn tiling(n: usize, m: usize, measure: impl Fn(f64, f64) -> f64) -> Vec<Vec<(usize, f32)>> {
    let ratio = n as f64 / m as f64;
    (0..m)
        .map(|k| {
            let (start, end) = (k as f64 * ratio, (k + 1) as f64 * ratio);
            let mut parts: Vec<(usize, f64)> = (start.floor() as usize
                ..(end.ceil() as usize).min(n))
                .map(|i| {
                    let a = start.max(i as f64);
                    let b = end.min(i as f64 + 1.0);
                    (i, measure(a, b))
                })
                .filter(|&(_, w)| w > 0.0)
                .collect();
            let total: f64 = parts.iter().map(|p| p.1).sum();
            for p in &mut parts {
                p.1 /= total;
            }
            parts.into_iter().map(|(i, w)| (i, w as f32)).collect()
        })
        .collect()
}

/// The level with `m` columns made from `src`, each row averaged over equal lengths.
fn halve_across(src: &Level, m: usize, threads: usize) -> Level {
    let parts = tiling(src.width, m, |a, b| b - a);
    let mut texels = vec![[0.0f32; 3]; m * src.height];
    for_each_band(&mut texels, m, 16, threads, |rows, band| {
        for (r, out_row) in rows.zip(band.chunks_mut(m)) {
            let in_row = &src.texels[r * src.width..(r + 1) * src.width];
            for (out, cell) in out_row.iter_mut().zip(&parts) {
                *out = weighted(cell.iter().map(|&(i, w)| (in_row[i], w)));
            }
        }
    });
    Level {
        width: m,
        height: src.height,
        scale: (m as f64 / src.width as f64 * src.scale.0, src.scale.1),
        texels,
        shift: Vec::new(),
    }
}

/// The level with `m` rows made from `src`, each column averaged over equal solid angles.
fn halve_down(src: &Level, m: usize, threads: usize) -> Level {
    let n = src.height as f64;
    // The sine of the latitude at v, in rows of `src`: the solid angle between two latitudes is
    // proportional to the difference of their sines.
    let sin_latitude = |v: f64| ((0.5 - v / n) * PI).sin();
    let parts = tiling(src.height, m, |a, b| sin_latitude(a) - sin_latitude(b));
    let width = src.width;
    let mut texels = vec![[0.0f32; 3]; width * m];
    for_each_band(&mut texels, width, 16, threads, |rows, band| {
        for (r, out_row) in rows.zip(band.chunks_mut(width)) {
            for (i, out) in out_row.iter_mut().enumerate() {
                *out = weighted(
                    parts[r]
                        .iter()
                        .map(|&(j, w)| (src.texels[j * width + i], w)),
                );
            }
        }
    });
    Level {
        width,
        height: m,
        scale: (src.scale.0, m as f64 / src.height as f64 * src.scale.1),
        texels,
        shift: Vec::new(),
    }
}

fn weighted(items: impl Iterator<Item = ([f32; 3], f32)>) -> [f32; 3] {
    let mut out = [0.0f32; 3];
    for (texel, w) in items {
        for c in 0..3 {
            out[c] += w * texel[c];
        }
    }
    out
}
