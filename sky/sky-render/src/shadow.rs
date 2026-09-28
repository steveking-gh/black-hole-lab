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
//! # The centre
//!
//! The panel is centred where the dark region is deepest: at the point farthest, as an angle on
//! the sphere, from every pixel that is not dark, the centre of the largest cap the dark region
//! holds. Its radius, the point's *clearance*, is measured to the centres of the pixels that are
//! not dark but touch a dark one (the *rim*). That is enough: from a dark point, the nearest pixel
//! that is not dark has a neighbour nearer the point, which is dark, or it would not be the
//! nearest; so it touches the dark region. The rim is a curve, a few thousand pixels at 8K, and
//! it is kept in buckets about [`BUCKET_DEGREES`] across, each with its own centre and radius, so
//! that a point's clearance reads only the buckets that could hold its nearest rim pixel: a bucket
//! whose nearest possible point is already farther than the nearest found is not opened.
//!
//! Candidate centres are a lattice [`LATTICE_DEGREES`] apart in heading and elevation, measured by
//! true angles on the sphere, never by distances in the frame, and the neighbourhood of the best
//! is searched again [`REFINE_DEGREES`] apart. The lattice runs round the whole circle, so a dark
//! region behind the observer, split by the frame's seam, is found whole. Elevations stop at the
//! limit `crate::panel` sets for any panel, [`POLE_CLEARANCE_DEGREES`] from a pole. Candidates are
//! ranked by clearance (to a nanoradian, so that a symmetric region's mirror images tie), then by
//! nearness to the frame's equator, then to heading 0, then right before left and up before down,
//! so that the choice is the same on every run.
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
//! The requested line height is tried first, at the best few centres [`CENTRES_APART_DEGREES`]
//! apart; then heights about [`SHRINK`] times smaller each time, down to [`FLOOR_DEGREES`]. If
//! nothing fits, the panel goes where `--readouts panel` puts it by default, at the full size,
//! and the run says why.

use std::cmp::Ordering;
use std::f64::consts::PI;

use crate::field::angle;
use crate::panel::{POLE_CLEARANCE_DEGREES, Panel, Placement};
use crate::parallel::for_each_band;
use crate::render::{Fields, Size};

/// How far outside the panel's outline the pixels must still be dark, in degrees.
pub const MARGIN_DEGREES: f64 = 1.0;

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

/// How many centres each size is tried at, and how far apart they must be, in degrees.
const CENTRES: usize = 6;
const CENTRES_APART_DEGREES: f64 = 3.0;

/// The rough size of a bucket of rim pixels, in degrees.
const BUCKET_DEGREES: f64 = 2.0;

/// Rows per band of work, as in `crate::render`.
const BAND_ROWS: usize = 16;

/// Which output pixels of a frame the renderer draws as the dark region.
pub struct DarkMap {
    size: Size,
    dark: Vec<bool>,
    count: usize,
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
        Self { size, dark, count }
    }

    pub fn is_dark(&self, i: usize, j: usize) -> bool {
        self.dark[j * self.size.width + i]
    }

    /// How many pixels are dark.
    pub fn count(&self) -> usize {
        self.count
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

/// The best centres, best first: up to [`CENTRES`] of them, each [`CENTRES_APART_DEGREES`] from
/// those before it, the first refined. Empty when no candidate is dark.
fn centres(map: &DarkMap, rim: &[Bucket], threads: usize) -> Vec<Candidate> {
    let limit = 90.0 - POLE_CLEARANCE_DEGREES;
    let rows = (limit / LATTICE_DEGREES).floor() as i32;
    let across = (180.0 / LATTICE_DEGREES).round() as i32;
    let lattice: Vec<(f64, f64)> = (-rows..=rows)
        .flat_map(|b| {
            (-across + 1..=across)
                .map(move |a| (a as f64 * LATTICE_DEGREES, b as f64 * LATTICE_DEGREES))
        })
        .collect();
    let mut found = survey(map, rim, &lattice, threads);
    found.sort_by(rank);
    let Some(&first) = found.first() else {
        return Vec::new();
    };
    // The best, searched again more finely about it.
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
    let mut chosen = vec![refined.first().copied().unwrap_or(first)];
    for c in found {
        if chosen.len() == CENTRES {
            break;
        }
        let apart = chosen
            .iter()
            .all(|d| angle(c.direction(), d.direction()) >= CENTRES_APART_DEGREES.to_radians());
        if apart {
            chosen.push(c);
        }
    }
    chosen
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
pub fn place(
    map: &DarkMap,
    requested: f64,
    threads: usize,
    outline: impl Fn(f64) -> Outline,
) -> Placing {
    if map.count() == 0 {
        return Placing::Outside(Why::NoDark);
    }
    let rim = rim(map, threads);
    let centres = centres(map, &rim, threads);
    if centres.is_empty() {
        return Placing::Outside(Why::NearPole);
    }
    let pixel = 2.0 * PI / map.size.width as f64;
    let heights = line_heights(requested);
    for &s in &heights {
        let o = outline(s);
        for c in &centres {
            let at = Placement {
                heading: c.heading,
                elevation: c.elevation,
            };
            let panel = Panel::new(at, pixel, o.width, o.height, o.radius);
            // A centre nearer the rim than the panel's half-height and the margin cannot take it.
            if c.clearance + 1e-9 < panel.inner_radius() + MARGIN_DEGREES.to_radians() {
                continue;
            }
            if fits(map, &panel) {
                return Placing::Inside {
                    at,
                    line_degrees: s,
                };
            }
        }
    }
    Placing::Outside(Why::TooSmall {
        line_degrees: *heights.last().expect("at least the requested height"),
    })
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
            };
            format!(
                "read-outs: {lines} line(s) on a panel below the opening view, because {reason}"
            )
        }
    }
}
