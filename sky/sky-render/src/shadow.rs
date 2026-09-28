//! Where a still's read-out panel goes when `--readout-at dark` asks for it: inside the black
//! hole's dark region, where it hides no sky.
//!
//! A panel is an annotation, not light, and wherever it sits it covers something. Over the dark
//! region it covers only black: the pixels the renderer draws as the shadow, which carry no
//! picture of anything. So the claim this module makes is exact and checkable: every output pixel
//! the panel covers, and every one within [`MARGIN_DEGREES`] of its outline, is one the renderer
//! draws as dark. The margin keeps the rim of the dark region in view, and with it the band of
//! red just outside the rim where the program does not know what the sky shows
//! (`crate::field`, "The edge of the dark region"): the panel must not tidy that away.
//!
//! # Which pixels are dark
//!
//! Not a guess from the fate plane: the renderer's own rule, pixel by pixel, at the output's size
//! ([`Fields::is_dark`]). A pixel is dark when the bundle frame it takes its fate from (the
//! nearer frame, for a still between two) has a dark ray nearest the pixel's centre. That is the
//! first thing `RayField::sample` decides, before it judges or interpolates anything, and it is
//! decided here by the same code. So "dark" below means black in the finished picture, before the
//! panel is painted.
//!
//! # The middle of the dark region, C0
//!
//! The panel is kept as near as it can be to where the dark region is deepest: the point
//! farthest, as an angle on the sphere, from every pixel that is not dark, the centre of the
//! largest cap the dark region holds. Call it C0. Its radius, a point's *clearance*, is measured
//! to the centres of the pixels that are not dark but touch a dark one (the *rim*). That is enough:
//! from a dark point, the nearest pixel that is not dark has a neighbour nearer the point, which is
//! dark, or it would not be the nearest; so it touches the dark region. The rim is a curve, a few
//! thousand pixels at 8K, and it is kept in buckets about [`BUCKET_DEGREES`] across, each with its
//! own centre and radius, so that a point's clearance reads only the buckets that could hold its
//! nearest rim pixel: a bucket whose nearest possible point is already farther than the nearest
//! found is not opened.
//!
//! C0 is found on a lattice [`LATTICE_DEGREES`] apart in heading and elevation, measured by true
//! angles on the sphere, never by distances in the frame, and the neighbourhood of the best is
//! searched again [`REFINE_DEGREES`] apart. The lattice runs round the whole circle, so a dark
//! region behind the observer, split by the frame's seam, is found whole. Elevations stop at the
//! limit `crate::panel` sets for any panel, [`POLE_CLEARANCE_DEGREES`] from a pole. Lattice points
//! are ranked by clearance (to a nanoradian, so that a symmetric region's mirror images tie), then
//! by nearness to the frame's equator, then to heading 0, then right before left and up before
//! down, so that the choice is the same on every run. C0 is always found in the whole dark
//! region, before the marks take anything from it (below, "Marks"): it is where a viewer looks
//! to see the dark region, and so where the panel is looked for.
//!
//! # Near the middle
//!
//! The candidate centres are C0 itself and every point of the lattice whose pixel is dark, taken
//! in order of their angle from C0, nearest first. Between centres equally near C0 (to a
//! nanoradian) the LOWER comes first, so that a panel with room both above and below the signs at
//! the middle goes below them, as a caption goes under the figure it names; this comes before
//! nearness to the frame's equator, which would otherwise put the panel above signs that sit
//! below the equator. Then the one nearer heading 0, then right before left. For each line height, the requested one first, the panel goes at the first
//! candidate at which it passes the exact test below. The lattice's first fit is at most a
//! lattice step from the nearest centre that could take the panel, so the points within a lattice
//! diagonal nearer C0 than it are tried again, in the same order, [`REFINE_DEGREES`] apart, and
//! the panel goes at the first of those that passes, if one does. Only when no candidate takes a
//! height is the next smaller height tried.
//!
//! Where the panel fits at C0 it goes there, as it always did. On 2026-09-27 it did in every still
//! tried of the real bundles without marks (`hover_r6_a0` and `hover_r6_a09` frame 0,
//! `bob_near_fall` frames 0, 237 and 473, at 8192 x 4096), whose placements and pictures are the
//! same to the bit as before this rule. A panel
//! that does not fit at C0 at the requested height now goes to the nearest centre that takes it
//! at that height, where before it went to the deepest of a few centres three degrees apart.
//!
//! # The size, and the exact test
//!
//! A cap of clearance rho holds a panel whose corners are within rho less the margin of its
//! centre, but a panel is a rectangle, wider than it is tall, and a rectangle often fits where its
//! circumscribed cap does not. So the cap only ranks centres and rules out hopeless ones (a centre
//! whose clearance is less than the panel's half-height and the margin cannot take it). The
//! decision is the exact one: the panel's outline, grown by the margin, is laid on the frame as
//! `crate::panel` lays a panel, and every pixel it covers must be dark.
//!
//! The margin is an angle on the sphere and the panel lives on its tangent plane, where the
//! gnomonic projection stretches angles by up to sec^2 theta at theta from the centre. So the
//! outline is grown on the plane by the margin times sec^2 of the panel's corner angle plus the
//! margin: every point within the margin of the panel, as an angle, is then inside the grown
//! outline, and the margin kept is at least the one promised, a little more toward the corners.
//!
//! The line heights tried are the requested one, then heights about [`SHRINK`] times smaller each
//! time, down to [`FLOOR_DEGREES`]. If nothing fits, the panel goes where `--readouts panel` puts
//! it by default, at the full size, and the run says why.
//!
//! The clearance only rules centres out quickly (a centre whose clearance is less than the
//! panel's half-height and the margin cannot take it); every centre that is not ruled out is
//! decided by the exact test, on the run's threads a batch at a time, the first in order that
//! passes winning. At 8192 x 4096 the whole search takes a few tenths of a second.
//!
//! # Marks
//!
//! A mark's sign (`crate::marks`) can lie inside the dark region: a radially falling observer's
//! direction of travel is straight at the hole. The panel must not hide it, so before the panel is
//! placed every pixel within a mark's sign, or within [`MARK_CLEARANCE_DEGREES`] of it, is taken
//! as not dark ([`DarkMap::keep_clear`]). A sign lies within `--mark-size` of its direction, so
//! those are the pixels within the size and the clearance of the direction. Nothing else changes:
//! the rim, the candidates and the exact test all see those pixels as they see any pixel that is
//! not dark, so the panel and its margin keep off them, and the sign and the panel do not touch;
//! only C0 is found in the whole region, so that the panel lands just beside the signs, which in a
//! falling observer's view sit at the middle of the dark region, and not in whatever part of it
//! is deepest once they are cut out (which can be forty degrees away, out of a 16:9 view). The
//! claim above stays exact as it was stated: a pixel taken as not dark is only ever one that was
//! dark, so every pixel the panel and its margin cover is still one the renderer draws as dark.
//! When no place is left the panel goes below the opening view as before, and the run says the
//! region was too small to hold it clear of its rim and of the marks.

use std::cmp::Ordering;
use std::f64::consts::PI;

use crate::field::angle;
use crate::panel::{POLE_CLEARANCE_DEGREES, Panel, Placement};
use crate::parallel::for_each_band;
use crate::render::{Fields, Size};

/// How far outside the panel's outline the pixels must still be dark, in degrees.
pub const MARGIN_DEGREES: f64 = 1.0;

/// How far beyond a mark's sign the pixels are taken as not dark, in degrees.
pub const MARK_CLEARANCE_DEGREES: f64 = 1.0;

/// The smallest line height the panel is shrunk to, in degrees. A player showing 90 degrees of
/// the sphere on a screen 1080 pixels high gives a degree 12 pixels: a line of text any smaller
/// is no longer comfortably read.
pub const FLOOR_DEGREES: f64 = 1.0;

/// Each smaller size tried is this much of the one before.
const SHRINK: f64 = 0.9;

/// The spacing of the candidate centres, in degrees of heading and of elevation.
const LATTICE_DEGREES: f64 = 1.0;

/// The spacing of the second search about the best candidate, in degrees, and how many steps it
/// goes either way (to one lattice step).
const REFINE_DEGREES: f64 = 0.25;
const REFINE_STEPS: i32 = 4;

/// The rough size of a bucket of rim pixels, in degrees.
const BUCKET_DEGREES: f64 = 2.0;

/// Rows per band of work, as in `crate::render`.
const BAND_ROWS: usize = 16;

/// Which output pixels of a frame the renderer draws as the dark region.
pub struct DarkMap {
    size: Size,
    dark: Vec<bool>,
    count: usize,
    /// How many dark pixels [`DarkMap::keep_clear`] has taken as not dark.
    cleared: usize,
    /// The dark region as it was before [`DarkMap::keep_clear`] took anything from it, where it
    /// took something: where its deepest point is, is where the panel is kept near.
    whole: Option<Vec<bool>>,
}

impl DarkMap {
    /// The dark pixels of a frame of `size` drawn from `fields`, found on `threads` threads; None
    /// for a frame before the bundle's first complete frame, which has no rays and so no dark
    /// region (it is drawn unresolved throughout).
    pub fn of(fields: Option<Fields>, size: Size, threads: usize) -> Self {
        let mut dark = vec![false; size.width * size.height];
        if let Some(fields) = fields {
            for_each_band(&mut dark, size.width, BAND_ROWS, threads, |rows, band| {
                for (j, row) in rows.zip(band.chunks_mut(size.width)) {
                    for (i, d) in row.iter_mut().enumerate() {
                        *d = fields.is_dark(size, i, j);
                    }
                }
            });
        }
        Self::from_mask(size, dark)
    }

    /// A map from a mask of `size.width * size.height` pixels, rows from the top.
    pub fn from_mask(size: Size, dark: Vec<bool>) -> Self {
        assert_eq!(dark.len(), size.width * size.height);
        let count = dark.iter().filter(|&&d| d).count();
        Self {
            size,
            dark,
            count,
            cleared: 0,
            whole: None,
        }
    }

    /// Takes as not dark every pixel within `reach` radians of any of `directions` (the module's
    /// comment, "Marks"), and returns how many dark pixels that was.
    pub fn keep_clear(&mut self, directions: &[[f64; 3]], reach: f64) -> usize {
        let before = self.count;
        let whole = self.whole.take().unwrap_or_else(|| self.dark.clone());
        for &n in directions {
            let centre = n.map(|c| c / crate::field::norm(n));
            let (size, dark) = (self.size, &mut self.dark);
            crate::marks::for_each_in_cap(size, centre, reach, |i, j, m| {
                if angle(m, centre) <= reach {
                    dark[j * size.width + i] = false;
                }
            });
        }
        self.count = self.dark.iter().filter(|&&d| d).count();
        self.cleared += before - self.count;
        if self.cleared > 0 {
            self.whole = Some(whole);
        }
        before - self.count
    }

    pub fn is_dark(&self, i: usize, j: usize) -> bool {
        self.dark[j * self.size.width + i]
    }

    /// How many pixels are dark.
    pub fn count(&self) -> usize {
        self.count
    }

    /// How many dark pixels [`DarkMap::keep_clear`] has taken for the marks.
    pub fn cleared(&self) -> usize {
        self.cleared
    }

    /// The unit direction through the centre of pixel (i, j), by the specification's formulae
    /// (section 4.3; `crate::panel` gives them).
    fn direction(&self, i: usize, j: usize) -> [f64; 3] {
        let (w, h) = (self.size.width as f64, self.size.height as f64);
        let lambda = ((i as f64 + 0.5) / w - 0.5) * 2.0 * PI;
        let beta = (0.5 - (j as f64 + 0.5) / h) * PI;
        towards(lambda, beta)
    }

    /// The pixel holding the direction of heading `h` and elevation `e`, in degrees.
    fn pixel_at(&self, h: f64, e: f64) -> (usize, usize) {
        let (w, hh) = (self.size.width, self.size.height);
        let i = ((h / 360.0 + 0.5) * w as f64).floor() as i64;
        let j = ((0.5 - e / 180.0) * hh as f64).floor() as i64;
        (
            i.rem_euclid(w as i64) as usize,
            j.clamp(0, hh as i64 - 1) as usize,
        )
    }
}

/// The unit direction at longitude (heading) `lambda` and latitude (elevation) `beta`, radians.
fn towards(lambda: f64, beta: f64) -> [f64; 3] {
    let (sl, cl) = lambda.sin_cos();
    let (sb, cb) = beta.sin_cos();
    [cb * cl, -cb * sl, sb]
}

/// The direction of heading `h` and elevation `e`, in degrees.
fn towards_degrees(h: f64, e: f64) -> [f64; 3] {
    towards(h.to_radians(), e.to_radians())
}

/// Rim pixels near one another, and the cap that holds them all.
struct Bucket {
    centre: [f64; 3],
    radius: f64,
    points: Vec<[f64; 3]>,
}

/// The rim: every pixel that is not dark and touches a dark pixel along a row or a column
/// (across the seam, and over a pole as `crate::bilinear` reads over it), in buckets.
fn rim(map: &DarkMap, threads: usize) -> Vec<Bucket> {
    let (w, h) = (map.size.width, map.size.height);
    // Per row, the columns of its rim pixels; rows are shared among threads one per item.
    let mut rows: Vec<Vec<u32>> = vec![Vec::new(); h];
    for_each_band(&mut rows, 1, BAND_ROWS, threads, |range, band| {
        for (j, out) in range.zip(band.iter_mut()) {
            for i in 0..w {
                if map.is_dark(i, j) {
                    continue;
                }
                let across = (i + w / 2) % w;
                let touches = map.is_dark((i + 1) % w, j)
                    || map.is_dark((i + w - 1) % w, j)
                    || if j > 0 {
                        map.is_dark(i, j - 1)
                    } else {
                        map.is_dark(across, 0)
                    }
                    || if j + 1 < h {
                        map.is_dark(i, j + 1)
                    } else {
                        map.is_dark(across, h - 1)
                    };
                if touches {
                    out.push(i as u32);
                }
            }
        }
    });
    // Buckets of `side` x `side` pixels: square in angle at the equator, and narrower toward
    // the poles, which only makes them smaller.
    let side = ((BUCKET_DEGREES / (180.0 / h as f64)).round() as usize).max(1);
    let columns = w.div_ceil(side);
    let mut grouped: Vec<Vec<[f64; 3]>> = vec![Vec::new(); h.div_ceil(side) * columns];
    for (j, row) in rows.iter().enumerate() {
        for &i in row {
            let i = i as usize;
            grouped[(j / side) * columns + i / side].push(map.direction(i, j));
        }
    }
    grouped
        .into_iter()
        .filter(|points| !points.is_empty())
        .map(|points| {
            let mut sum = [0.0; 3];
            for p in &points {
                for (s, c) in sum.iter_mut().zip(p) {
                    *s += c;
                }
            }
            // The points of a bucket lie within a few degrees of one another, so their sum is
            // never near zero.
            let length = crate::field::norm(sum);
            let centre = sum.map(|c| c / length);
            let radius = points.iter().map(|&p| angle(centre, p)).fold(0.0, f64::max);
            Bucket {
                centre,
                // A hair more, so that rounding cannot close a bucket that holds the nearest.
                radius: radius + 1e-12,
                points,
            }
        })
        .collect()
}

/// The angle from `c` to the nearest rim pixel: pi when there is no rim (every pixel is dark).
fn clearance(rim: &[Bucket], c: [f64; 3]) -> f64 {
    let mut order: Vec<(f64, usize)> = rim
        .iter()
        .enumerate()
        .map(|(k, b)| (angle(c, b.centre) - b.radius, k))
        .collect();
    order.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut best = PI;
    for (nearest_possible, k) in order {
        if nearest_possible >= best {
            break;
        }
        for &p in &rim[k].points {
            best = best.min(angle(c, p));
        }
    }
    best
}

/// A candidate centre and its clearance, in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Candidate {
    heading: f64,
    elevation: f64,
    clearance: f64,
}

impl Candidate {
    fn direction(&self) -> [f64; 3] {
        towards_degrees(self.heading, self.elevation)
    }
}

/// The ranking of the module's comment: `Less` when `a` is the better centre.
fn rank(a: &Candidate, b: &Candidate) -> Ordering {
    let depth = |c: &Candidate| (c.clearance * 1e9).round() as i64;
    depth(b)
        .cmp(&depth(a))
        .then(a.elevation.abs().total_cmp(&b.elevation.abs()))
        .then(a.heading.abs().total_cmp(&b.heading.abs()))
        .then(b.heading.total_cmp(&a.heading))
        .then(b.elevation.total_cmp(&a.elevation))
}

/// The candidates among `points` (heading, elevation in degrees) whose own pixel is dark, with
/// their clearances, found on `threads` threads, in the order of `points`.
fn survey(map: &DarkMap, rim: &[Bucket], points: &[(f64, f64)], threads: usize) -> Vec<Candidate> {
    let chunk = points.len().div_ceil(threads.max(1)).max(1);
    std::thread::scope(|scope| {
        let workers: Vec<_> = points
            .chunks(chunk)
            .map(|part| {
                scope.spawn(move || {
                    part.iter()
                        .filter(|&&(h, e)| {
                            let (i, j) = map.pixel_at(h, e);
                            map.is_dark(i, j)
                        })
                        .map(|&(heading, elevation)| Candidate {
                            heading,
                            elevation,
                            clearance: clearance(rim, towards_degrees(heading, elevation)),
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        workers
            .into_iter()
            .flat_map(|w| w.join().expect("no survey thread panics"))
            .collect()
    })
}

/// A heading taken into (-180, 180].
fn wrap_degrees(h: f64) -> f64 {
    let a = h.rem_euclid(360.0);
    if a > 180.0 { a - 360.0 } else { a }
}

/// Every point of the lattice of candidate centres (the module's comment, "The centre"), as
/// (heading, elevation) in degrees.
fn lattice() -> Vec<(f64, f64)> {
    let limit = 90.0 - POLE_CLEARANCE_DEGREES;
    let rows = (limit / LATTICE_DEGREES).floor() as i32;
    let across = (180.0 / LATTICE_DEGREES).round() as i32;
    (-rows..=rows)
        .flat_map(|b| {
            (-across + 1..=across)
                .map(move |a| (a as f64 * LATTICE_DEGREES, b as f64 * LATTICE_DEGREES))
        })
        .collect()
}

/// The deepest point of the dark region `map` describes, whose rim is `rim`: the best of the
/// lattice by [`rank`], searched again [`REFINE_DEGREES`] apart about it. None when no point of
/// the lattice is dark.
fn deepest(map: &DarkMap, rim: &[Bucket], threads: usize) -> Option<Candidate> {
    let limit = 90.0 - POLE_CLEARANCE_DEGREES;
    let mut found = survey(map, rim, &lattice(), threads);
    found.sort_by(rank);
    let first = *found.first()?;
    let near: Vec<(f64, f64)> = (-REFINE_STEPS..=REFINE_STEPS)
        .flat_map(|b| {
            (-REFINE_STEPS..=REFINE_STEPS).map(move |a| {
                (
                    wrap_degrees(first.heading + a as f64 * REFINE_DEGREES),
                    first.elevation + b as f64 * REFINE_DEGREES,
                )
            })
        })
        .filter(|&(_, e)| e.abs() <= limit)
        .collect();
    let mut refined = survey(map, rim, &near, threads);
    refined.sort_by(rank);
    Some(refined.first().copied().unwrap_or(first))
}

/// The order in which centres are tried (the module's comment, "Near the middle"): `Less` when
/// `a` is nearer `from`, as an angle to a nanoradian; between centres equally near, the lower
/// one, then the one nearer heading 0, then right before left.
fn nearer(from: [f64; 3], a: &Candidate, b: &Candidate) -> Ordering {
    let off = |c: &Candidate| (angle(from, c.direction()) * 1e9).round() as i64;
    off(a)
        .cmp(&off(b))
        .then(a.elevation.total_cmp(&b.elevation))
        .then(a.heading.abs().total_cmp(&b.heading.abs()))
        .then(b.heading.total_cmp(&a.heading))
}

/// The first of `candidates`, in their order, at which a panel of outline `o` passes the exact
/// test ([`fits`]); those whose clearance is less than the panel's half-height and the margin are
/// passed over without it. The tests run on `threads` threads a batch at a time, and the answer is
/// the first in order that passes, whichever thread finished first.
fn first_fit(
    map: &DarkMap,
    candidates: &[Candidate],
    o: Outline,
    threads: usize,
) -> Option<Candidate> {
    let pixel = 2.0 * PI / map.size.width as f64;
    let hopeful: Vec<(Candidate, Panel)> = candidates
        .iter()
        .map(|c| {
            let at = Placement {
                heading: c.heading,
                elevation: c.elevation,
            };
            (*c, Panel::new(at, pixel, o.width, o.height, o.radius))
        })
        // A centre nearer the rim than the panel's half-height and the margin cannot take it.
        .filter(|(c, panel)| {
            c.clearance + 1e-9 >= panel.inner_radius() + MARGIN_DEGREES.to_radians()
        })
        .collect();
    for batch in hopeful.chunks(threads.max(1)) {
        let passed: Vec<bool> = std::thread::scope(|scope| {
            let tests: Vec<_> = batch
                .iter()
                .map(|(_, panel)| scope.spawn(move || fits(map, panel)))
                .collect();
            tests
                .into_iter()
                .map(|t| t.join().expect("no test thread panics"))
                .collect()
        });
        if let Some(k) = passed.iter().position(|&p| p) {
            return Some(batch[k].0);
        }
    }
    None
}

/// Whether `panel` and every pixel within the margin of its outline are dark (the module's
/// comment, "The size, and the exact test"). A panel that reaches a pole never fits.
pub fn fits(map: &DarkMap, panel: &Panel) -> bool {
    if panel.reaches_pole() {
        return false;
    }
    let delta = MARGIN_DEGREES.to_radians();
    let reach = panel.angular_radius() + delta;
    if reach >= 0.5 * PI {
        return false;
    }
    let grown = panel.grown(delta / reach.cos().powi(2) / panel.pixel());
    grown
        .cover(map.size)
        .iter()
        .all(|t| map.dark[t.pixel as usize])
}

/// A panel's size in panel pixels, at the frame's panel pixel of 2 pi / W, and the radius of its
/// corners: what `crate::layout` makes of the read-outs at one line height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outline {
    pub width: usize,
    pub height: usize,
    pub radius: f64,
}

/// Where the panel goes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Placing {
    /// Inside the dark region, centred `at`, each line `line_degrees` high.
    Inside { at: Placement, line_degrees: f64 },
    /// At `Placement::DEFAULT` at the requested size, because of this.
    Outside(Why),
}

/// Why the panel is not inside the dark region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Why {
    /// No pixel of the picture is dark.
    NoDark,
    /// Every dark pixel lies within [`POLE_CLEARANCE_DEGREES`] of a pole.
    NearPole,
    /// The dark region cannot hold the panel with its margin even at this line height, the
    /// smallest tried.
    TooSmall { line_degrees: f64 },
    /// As `TooSmall`, where some of the dark region was taken for the marks' signs and their
    /// clearance (the module's comment, "Marks").
    TooSmallBesideMarks { line_degrees: f64 },
}

/// The line heights tried, largest first: `requested`, then [`SHRINK`] times the one before
/// (rounded to a hundredth of a degree) while that is above [`FLOOR_DEGREES`], then the floor.
/// A request at or below the floor is tried alone.
pub fn line_heights(requested: f64) -> Vec<f64> {
    let mut out = vec![requested];
    if requested <= FLOOR_DEGREES {
        return out;
    }
    let mut s = requested;
    loop {
        s = (s * SHRINK * 100.0).round() / 100.0;
        if s <= FLOOR_DEGREES {
            break;
        }
        out.push(s);
    }
    out.push(FLOOR_DEGREES);
    out
}

/// Where a panel whose outline at a line height of s degrees is `outline(s)` goes in the frame
/// `map` describes, asked for at `requested` degrees a line; on `threads` threads.
#[cfg(test)]
pub fn place(
    map: &DarkMap,
    requested: f64,
    threads: usize,
    outline: impl Fn(f64) -> Outline,
) -> Placing {
    place_near(map, requested, threads, outline).0
}

/// [`place`], and the middle of the dark region the panel is kept near, C0 (the module's comment,
/// "Near the middle"), when there is one.
pub fn place_near(
    map: &DarkMap,
    requested: f64,
    threads: usize,
    outline: impl Fn(f64) -> Outline,
) -> (Placing, Option<Placement>) {
    let heights = line_heights(requested);
    let smallest = *heights.last().expect("at least the requested height");
    // Where the marks took some of the dark region, it is the marks that leave no room.
    let outside = |why: Why| {
        if map.cleared > 0 {
            Placing::Outside(Why::TooSmallBesideMarks {
                line_degrees: smallest,
            })
        } else {
            Placing::Outside(why)
        }
    };
    if map.count() == 0 {
        return (outside(Why::NoDark), None);
    }
    let rim_now = rim(map, threads);
    // C0: the deepest point of the whole dark region, the marks' signs not cut out of it.
    let c0 = match &map.whole {
        Some(whole) => {
            let whole = DarkMap::from_mask(map.size, whole.clone());
            deepest(&whole, &rim(&whole, threads), threads)
        }
        None => deepest(map, &rim_now, threads),
    };
    let Some(c0) = c0 else {
        return (outside(Why::NearPole), None);
    };
    let middle = Placement {
        heading: c0.heading,
        elevation: c0.elevation,
    };
    let from = c0.direction();
    // Every centre that is dark with the marks cut out, C0 itself first among equals, nearest
    // C0 first.
    let mut points = lattice();
    points.insert(0, (c0.heading, c0.elevation));
    let mut found = survey(map, &rim_now, &points, threads);
    if found.is_empty() {
        return (outside(Why::NearPole), Some(middle));
    }
    found.sort_by(|a, b| nearer(from, a, b));
    for &s in &heights {
        let o = outline(s);
        let Some(best) = first_fit(map, &found, o, threads) else {
            continue;
        };
        // The lattice's first fit; any centre nearer C0 that could take the panel lies within a
        // lattice diagonal of it, and those are tried again REFINE_DEGREES apart.
        let off = angle(from, best.direction());
        let chosen = if off > 0.0 {
            let mut near = survey(map, &rim_now, &nearer_points(c0, off), threads);
            near.sort_by(|a, b| nearer(from, a, b));
            first_fit(map, &near, o, threads).unwrap_or(best)
        } else {
            best
        };
        let at = Placement {
            heading: chosen.heading,
            elevation: chosen.elevation,
        };
        return (
            Placing::Inside {
                at,
                line_degrees: s,
            },
            Some(middle),
        );
    }
    (
        outside(Why::TooSmall {
            line_degrees: smallest,
        }),
        Some(middle),
    )
}

/// The points REFINE_DEGREES apart about `c0`, in heading and elevation, whose angle from it is
/// less than `off` radians and not less than `off` less a lattice diagonal: where a centre nearer
/// C0 than the lattice's first fit could still lie.
fn nearer_points(c0: Candidate, off: f64) -> Vec<(f64, f64)> {
    let limit = 90.0 - POLE_CLEARANCE_DEGREES;
    let from = c0.direction();
    let inner = off - 2f64.sqrt() * LATTICE_DEGREES.to_radians();
    let reach = off.to_degrees();
    let down = (reach / REFINE_DEGREES).ceil() as i32;
    // Across, a degree of heading is cos e of a degree of angle; the steps are capped at a turn.
    let widest = (c0.elevation.abs() + reach).min(limit).to_radians().cos();
    let across = ((reach / (REFINE_DEGREES * widest)).ceil() as i32).min(720);
    (-down..=down)
        .flat_map(|b| {
            (-across..=across).map(move |a| {
                (
                    wrap_degrees(c0.heading + a as f64 * REFINE_DEGREES),
                    c0.elevation + b as f64 * REFINE_DEGREES,
                )
            })
        })
        .filter(|&(h, e)| {
            let here = angle(from, towards_degrees(h, e));
            e.abs() <= limit && here < off && here >= inner
        })
        .collect()
}

/// A number of degrees as the run writes it: to a hundredth at most, no trailing zeros, no -0.
pub fn degrees(x: f64) -> String {
    let text = format!("{:.2}", x + 0.0);
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" {
        "0".into()
    } else {
        text.into()
    }
}

/// What the run adds when the marks moved the panel: how far it is from the middle of the dark
/// region, C0, and where C0 is.
pub fn beside_marks_line(at: Placement, middle: Placement) -> String {
    let off = angle(
        towards_degrees(at.heading, at.elevation),
        towards_degrees(middle.heading, middle.elevation),
    );
    format!(
        "the panel keeps clear of the marks, {} from the middle of the dark region, which is {} \
         degrees right of the opening view and {} degrees up",
        angle_text(off.to_degrees()),
        degrees(middle.heading),
        degrees(middle.elevation)
    )
}

/// `1 degree`, `0.8 degrees`.
fn angle_text(x: f64) -> String {
    let number = degrees(x);
    if number == "1" {
        "1 degree".into()
    } else {
        format!("{number} degrees")
    }
}

/// What the run says of a panel of `lines` lines placed as `placing` says.
pub fn sentence(placing: &Placing, lines: usize) -> String {
    match placing {
        Placing::Inside { at, line_degrees } => format!(
            "read-outs: {lines} line(s) on a panel inside the dark region, centred {} degrees \
             right of the opening view and {} degrees up, each line {} degrees high",
            degrees(at.heading),
            degrees(at.elevation),
            degrees(*line_degrees)
        ),
        Placing::Outside(why) => {
            let reason = match why {
                Why::NoDark => "the picture has no dark region".to_string(),
                Why::NearPole => format!(
                    "the dark region lies within {} degrees of a pole, where no panel is drawn",
                    degrees(POLE_CLEARANCE_DEGREES)
                ),
                Why::TooSmall { line_degrees } => format!(
                    "the dark region is too small to hold them at {} a line and keep {} clear \
                     of its rim",
                    angle_text(*line_degrees),
                    angle_text(MARGIN_DEGREES)
                ),
                Why::TooSmallBesideMarks { line_degrees } => format!(
                    "the dark region is too small to hold them at {} a line and keep {} clear \
                     of its rim and of the marks",
                    angle_text(*line_degrees),
                    angle_text(MARGIN_DEGREES)
                ),
            };
            format!(
                "read-outs: {lines} line(s) on a panel below the opening view, because {reason}"
            )
        }
    }
}
