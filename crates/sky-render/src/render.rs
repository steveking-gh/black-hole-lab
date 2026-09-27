//! One video frame: for every output pixel, the ray it looks along, the sky that ray reaches, and
//! the colour that makes.
//!
//! The work goes a row at a time. For a row, every pixel's ray is found first (interpolated from
//! the bundle's rays, blended between bundle frames, rotated into the star map's frame); then each
//! pixel reads the map over its footprint, which needs the rays of its right-hand neighbour in the
//! same row and of the pixel below it. So a band of rows keeps two rows of rays, the one being
//! drawn and the one below, and each row's rays are found once (the row after a band's last is
//! found twice, once by each band). The bottom row has no row below and measures its footprint
//! against the row above; the last column's right-hand neighbour is column 0, across the seam.

use std::ops::Range;

use crate::field::{Ray, RayField, blend};
use crate::parallel::for_each_band;
use crate::sky::{Mat3, SkyMap, apply};
use crate::tone::{Encoder, shade};

/// The bundle frames a video frame is made of, prepared.
#[derive(Clone, Copy)]
pub enum Fields<'a> {
    One(&'a RayField),
    /// `w` of the way from the first to the second.
    Two(&'a RayField, &'a RayField, f64),
}

/// Everything a frame is drawn from, apart from its size.
pub struct Scene<'a> {
    pub fields: Fields<'a>,
    /// From the bundle's far-sky frame to the map's frame.
    pub rotation: Mat3,
    pub sky: &'a SkyMap,
}

/// How a frame's light becomes its numbers.
#[derive(Debug, Clone)]
pub struct Look {
    /// The exposure, as a factor on linear light.
    pub gain: f32,
    /// The flat colour of unresolved pixels, as 16-bit sRGB-encoded codes.
    pub unresolved: [u16; 3],
    pub encoder: Encoder,
}

/// What one output pixel shows, before tone mapping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shade {
    /// Linear light, scaled by g^4 and the exposure, not clipped.
    Light([f32; 3]),
    Shadow,
    Unresolved,
}

/// The size of the video frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Size {
    pub width: usize,
    pub height: usize,
}

/// A pixel's ray, once it is known where on the map it lands.
#[derive(Debug, Clone, Copy)]
enum Placed {
    /// Level-0 frame coordinates on the map, and the shift.
    Sky {
        at: (f64, f64),
        g: f64,
    },
    Shadow,
    Unresolved,
}

impl Scene<'_> {
    /// Where output pixel (i, j) lands on the map.
    fn place(&self, size: Size, i: usize, j: usize) -> Placed {
        let at = |field: &RayField| {
            // The output pixel's centre, in the bundle grid's own frame coordinates: the two grids
            // cover the same sphere, so the scale is the ratio of their sizes.
            let u = (i as f64 + 0.5) * field.width as f64 / size.width as f64;
            let v = (j as f64 + 0.5) * field.height as f64 / size.height as f64;
            field.sample(u, v)
        };
        let ray = match self.fields {
            Fields::One(a) => at(a),
            Fields::Two(a, b, w) => blend(at(a), at(b), w),
        };
        match ray {
            Ray::Sky { d, g } => Placed::Sky {
                at: self.sky.place(apply(&self.rotation, d)),
                g,
            },
            Ray::Shadow => Placed::Shadow,
            Ray::Unresolved => Placed::Unresolved,
        }
    }

    fn row(&self, size: Size, j: usize, out: &mut Vec<Placed>) {
        out.clear();
        out.extend((0..size.width).map(|i| self.place(size, i, j)));
    }

    /// Shades rows `rows` of the frame, handing each pixel's shade to `emit(i, j, shade)`.
    pub fn shade_rows(
        &self,
        size: Size,
        gain: f32,
        rows: Range<usize>,
        mut emit: impl FnMut(usize, usize, Shade),
    ) {
        if rows.is_empty() {
            return;
        }
        let w = size.width;
        let mut here = Vec::with_capacity(w);
        let mut below = Vec::with_capacity(w);
        self.row(size, rows.start, &mut here);
        for j in rows {
            // The neighbouring row: the one below, or for the bottom row the one above.
            let other = if j + 1 < size.height {
                j + 1
            } else {
                j.saturating_sub(1)
            };
            self.row(size, other, &mut below);
            let sky_at = |p: Placed| match p {
                Placed::Sky { at, .. } => Some(at),
                _ => None,
            };
            for i in 0..w {
                let shade = match here[i] {
                    Placed::Sky { at, g } => {
                        // The right-hand neighbour, or the left-hand one if that is not sky.
                        let across =
                            sky_at(here[(i + 1) % w]).or_else(|| sky_at(here[(i + w - 1) % w]));
                        let down = if other == j { None } else { sky_at(below[i]) };
                        let footprint = self.sky.footprint(at, across, down);
                        Shade::Light(shade(self.sky.sample(footprint), g, gain))
                    }
                    Placed::Shadow => Shade::Shadow,
                    Placed::Unresolved => Shade::Unresolved,
                };
                emit(i, j, shade);
            }
            std::mem::swap(&mut here, &mut below);
        }
    }
}

/// Renders a frame into `out`, three 16-bit codes per pixel (R, G, B), rows top to bottom, on
/// `threads` threads.
pub fn render(scene: &Scene, size: Size, look: &Look, threads: usize, out: &mut [u16]) {
    assert_eq!(out.len(), size.width * size.height * 3);
    let row_len = size.width * 3;
    for_each_band(out, row_len, BAND_ROWS, threads, |rows, band| {
        let first = rows.start;
        scene.shade_rows(size, look.gain, rows, |i, j, s| {
            let rgb = match s {
                Shade::Light(light) => light.map(|c| look.encoder.encode(c)),
                Shade::Shadow => [0; 3],
                Shade::Unresolved => look.unresolved,
            };
            let at = (j - first) * row_len + 3 * i;
            band[at..at + 3].copy_from_slice(&rgb);
        });
    });
}

#[cfg(test)]
/// The frame's shades before tone mapping, for tests that look at linear light.
pub fn shades(scene: &Scene, size: Size, gain: f32, threads: usize) -> Vec<Shade> {
    let mut out = vec![Shade::Shadow; size.width * size.height];
    for_each_band(&mut out, size.width, BAND_ROWS, threads, |rows, band| {
        let first = rows.start;
        scene.shade_rows(size, gain, rows, |i, j, s| {
            band[(j - first) * size.width + i] = s
        });
    });
    out
}

/// Rows per band of work. Sixteen rows of an 8192-wide frame is about 14 ms of one thread's work:
/// 256 bands balance 16 threads well, and the extra row of rays each band finds (for the footprint
/// of its last row) adds a sixteenth to the cost of finding rays, 3 % of the frame's.
const BAND_ROWS: usize = 16;
