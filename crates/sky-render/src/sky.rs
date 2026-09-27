//! The star map: where a direction lands on it, and what it shows there, filtered to the size of
//! the pixel asking.
//!
//! # Where a direction lands
//!
//! A map is equirectangular in its own frame, celestial (ICRS) or galactic, with longitude
//! increasing to the left (assets/sky/README.md). A unit direction (x, y, z) in the map's frame
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

use std::f64::consts::PI;

use crate::bilinear::taps;
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
}

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
    /// left to right, as linear RGB. `threads` share the building.
    pub fn new(width: usize, height: usize, texels: Vec<[f32; 3]>, threads: usize) -> Self {
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
                },
                None => halve_down(&levels[(ly - 1) * across], h, threads),
            };
            levels.push(first);
            for &w in &widths[1..] {
                let next = halve_across(levels.last().expect("a level"), w, threads);
                levels.push(next);
            }
        }
        Self {
            width,
            height,
            across,
            down,
            levels,
        }
    }

    pub fn width(&self) -> usize {
        self.width
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
