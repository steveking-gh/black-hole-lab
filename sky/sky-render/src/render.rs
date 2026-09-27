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
//!
//! # A neighbour that is not sky
//!
//! The footprint needs a neighbour along each axis whose direction is known. When the right-hand
//! neighbour is not sky the left-hand one is used. When the pixel below is not sky, a judged
//! frame (`--undersampled mark`) uses the pixel above, and when neither neighbour along an axis is
//! sky it marks the pixel as under-sampled: its patch of sky cannot be measured, and reading the
//! map one texel wide there (what a missing step of zero amounts to) would show detail the pixel
//! may not resolve, a sharper picture than the truth. Every such pixel sits beside the dark
//! region, an unresolved ray or a marked pixel. An unjudged frame keeps the renderer's old rule, a
//! step of zero for a missing neighbour, so that `--undersampled interpolate` draws what it always
//! drew.
//!
//! # Counting
//!
//! [`render`] returns how many pixels of the frame it drew as unresolved, under-sampled and dark
//! ([`Tally`]), before any read-out panel is painted over them. Light that came out as NaN, which
//! the arithmetic of `crate::tone` no longer produces, would be drawn and counted as unresolved
//! rather than written as black.

use std::ops::Range;
use std::sync::Mutex;

use crate::field::{Ray, RayField, blend, blend_judged};
use crate::parallel::for_each_band;
use crate::sky::{Mat3, SkyMap, apply};
use crate::tally::Tally;
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
    /// The flat colour of under-sampled pixels, likewise.
    pub undersampled: [u16; 3],
    pub encoder: Encoder,
}

/// What one output pixel shows, before tone mapping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shade {
    /// Linear light, scaled by g^4 and the exposure, not clipped.
    Light([f32; 3]),
    Shadow,
    Unresolved,
    Undersampled,
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
    Undersampled,
}

impl Scene<'_> {
    /// Whether the frame marks what its rays do not resolve.
    fn judged(&self) -> bool {
        match self.fields {
            Fields::One(a) => a.is_judged(),
            Fields::Two(a, b, _) => a.is_judged() || b.is_judged(),
        }
    }

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
            Fields::Two(a, b, w) if self.judged() => blend_judged(at(a), at(b), w),
            Fields::Two(a, b, w) => blend(at(a), at(b), w),
        };
        match ray {
            Ray::Sky { d, g } => Placed::Sky {
                at: self.sky.place(apply(&self.rotation, d)),
                g,
            },
            Ray::Shadow => Placed::Shadow,
            Ray::Unresolved => Placed::Unresolved,
            Ray::Undersampled => Placed::Undersampled,
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
        let judged = self.judged();
        let mut here = Vec::with_capacity(w);
        let mut below = Vec::with_capacity(w);
        // The row above the one being drawn, once known: the row drawn last, or for the first row
        // of a band found only when a judged frame needs it.
        let mut above: Option<Vec<Placed>> = None;
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
                        let mut down = if other == j { None } else { sky_at(below[i]) };
                        // Judged, the pixel above stands in for the pixel below; for the bottom
                        // row `below` is already the row above.
                        if judged && down.is_none() && j > 0 && other == j + 1 {
                            let above = above.get_or_insert_with(|| {
                                let mut row = Vec::with_capacity(w);
                                self.row(size, j - 1, &mut row);
                                row
                            });
                            down = sky_at(above[i]);
                        }
                        let unmeasured =
                            across.is_none() && w > 1 || down.is_none() && size.height > 1;
                        if judged && unmeasured {
                            Shade::Undersampled
                        } else {
                            let footprint = self.sky.footprint(at, across, down);
                            Shade::Light(shade(self.sky.sample(footprint), g, gain))
                        }
                    }
                    Placed::Shadow => Shade::Shadow,
                    Placed::Unresolved => Shade::Unresolved,
                    Placed::Undersampled => Shade::Undersampled,
                };
                emit(i, j, shade);
            }
            // Row j becomes the row above, row j + 1 the row drawn, and the old row above's
            // storage is reused for the next row below.
            let spare = above.take().unwrap_or_default();
            above = Some(std::mem::replace(
                &mut here,
                std::mem::replace(&mut below, spare),
            ));
        }
    }
}

/// Renders a frame into `out`, three 16-bit codes per pixel (R, G, B), rows top to bottom, on
/// `threads` threads, and counts what it drew.
pub fn render(scene: &Scene, size: Size, look: &Look, threads: usize, out: &mut [u16]) -> Tally {
    assert_eq!(out.len(), size.width * size.height * 3);
    let row_len = size.width * 3;
    let total = Mutex::new(Tally::default());
    for_each_band(out, row_len, BAND_ROWS, threads, |rows, band| {
        let first = rows.start;
        let mut tally = Tally::default();
        scene.shade_rows(size, look.gain, rows, |i, j, s| {
            tally.pixels += 1;
            let rgb = match s {
                Shade::Light(light) if !light.iter().any(|c| c.is_nan()) => {
                    light.map(|c| look.encoder.encode(c))
                }
                Shade::Light(_) | Shade::Unresolved => {
                    tally.unresolved += 1;
                    look.unresolved
                }
                Shade::Shadow => {
                    tally.dark += 1;
                    [0; 3]
                }
                Shade::Undersampled => {
                    tally.undersampled += 1;
                    look.undersampled
                }
            };
            let at = (j - first) * row_len + 3 * i;
            band[at..at + 3].copy_from_slice(&rgb);
        });
        total
            .lock()
            .expect("no band panics holding the tally")
            .add(&tally);
    });
    total
        .into_inner()
        .expect("no band panics holding the tally")
}

/// Fills `out` with the unresolved colour: a video frame for which the bundle has no rays.
pub fn fill_unresolved(size: Size, look: &Look, out: &mut [u16]) -> Tally {
    for pixel in out.as_chunks_mut::<3>().0 {
        *pixel = look.unresolved;
    }
    let pixels = (size.width * size.height) as u64;
    Tally {
        pixels,
        unresolved: pixels,
        ..Tally::default()
    }
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
