//! The rays of one bundle frame, and what they say about any direction on the observer's sky.
//!
//! A bundle records one ray per cell of its grid, and the video usually has more pixels than the
//! bundle has rays. So each output pixel is answered by interpolating the rays around it: the
//! direction at infinity `d` component by component and then renormalised (the specification
//! stores `d` as a vector for exactly this, section 4.5), and the shift `g` alongside it.
//!
//! **Fate decides first.** Interpolation is only meaningful between rays that reached the far sky.
//! A pixel therefore takes the fate of its nearest ray. If that ray reached the far sky, the pixel
//! is interpolated from those of its four neighbours that did too, with their weights
//! renormalised; a shadow ray beside it contributes nothing, rather than its NaNs poisoning the
//! sum or a weight of zero dragging the direction toward the origin. If the nearest ray is from
//! the shadow the pixel is shadow, and if it is unresolved (or carries a reserved code, which the
//! specification says to treat as unresolved) the pixel is unresolved. So the edge of a shadow
//! falls halfway between rays, where the nearest ray changes, and not smeared across a cell.
//!
//! **Between bundle frames** the same rule holds in time: a pixel takes its fate from the nearer
//! frame, and when both frames saw the far sky there, the two directions are blended linearly and
//! renormalised, and the shifts linearly. Blending the direction rather than the picture is what
//! makes a turning sky move between frames instead of cross-fading. Renormalised linear blending
//! runs slightly slower at the middle than a true rotation would (for frames 10 degrees apart the
//! midpoint is right and the quarter points are 0.005 degrees behind); a bundle whose frames are
//! far apart in angle is a bundle that wants more frames.
//!
//! # Where interpolation does not know the answer
//!
//! Interpolating `d` is right where the map from the observer's sky to the far sky is smooth on
//! the scale of the grid. Beside a black hole's dark region it is not: light there passed close to
//! the photon orbits, and each ray nearer the edge went round the hole more times than the last,
//! so neighbouring rays reach parts of the sky that nothing in between them connects. The
//! renormalised mean of such rays is a direction none of the light came from, and drawing the sky
//! there is an invention. A field that has been [judged](RayField::judge) (the default,
//! `--undersampled mark`) answers [`Ray::Undersampled`] for such a pixel, and the renderer draws it
//! in a marker colour of its own. An unjudged field (`--undersampled interpolate`, and every field
//! made by `from_frame` alone) interpolates as the renderer always did, bit for bit.
//!
//! **What is being estimated.** Along one axis of the grid, with rays a step h apart, linear
//! interpolation between two rays is out by at most |d''| h^2 / 8, at the middle of the cell. The
//! rays one cell further out give |d''| h^2 without any extra ray: with p- and p+ the neighbours of
//! ray p, the renormalised midpoint of d(p-) and d(p+) differs from d(p) by an angle
//! e = |d''| h^2 / 2 (its component across the sphere: renormalising removes the rest, so a map
//! that is locally a rotation, or a smooth magnification, has e near zero however steep it is).
//! So the interpolation error there is e / 4, as an angle on the far sky. It is a consistency
//! check with the map's own local affine fit, from rays the bundle already holds.
//!
//! **What it is compared with.** Not with the spread of the rays: a strong, smooth magnification
//! (aberration at high speed, or lensing well outside the photon orbits) puts neighbouring rays
//! tens of degrees apart, and the footprint filter of `crate::sky` handles that correctly by
//! averaging the map over the patch the pixel covers. What a pixel shows is already an average over
//! its footprint, so an error of position smaller than the footprint changes little: the pixel
//! still shows mostly the sky it should. The footprint along the axis is the step from d(p) to a
//! neighbour over the s output pixels a step of the grid spans; of the two steps, the smaller,
//! because beside a jump the larger step measures the jump and not the map (and would excuse the
//! jump by the very size of it). And no footprint is finer than one texel of the star map, below
//! which the map has nothing to show: a position error under a texel moves a star by less than its
//! own drawn width. So a ray is *suspect* along an axis when
//!
//!     e / 4 > max(min(angle(d(p-), d(p)), angle(d(p), d(p+))) / s, texel)
//!
//! that is, when the interpolation beside it may be wrong by more than the pixel's own footprint.
//! The threshold is 1 in these units: the band it marks is where the error exceeds one output
//! pixel of sky. A ray whose neighbour along the axis is not far sky cannot be checked along it and
//! is suspect too, and so is one whose neighbours point at opposite parts of the sky (their
//! midpoint has no direction).
//!
//! **From rays to pixels.** A pixel interpolates within one cell, between two rays along each
//! axis. The second difference at a ray measures the map over the two cells either side of it, so
//! curvature inside a cell shows at both of its corners along the axis, and curvature in the next
//! cell shows at only one. A pixel is marked when both corners of one of its edges are suspect
//! along that edge's axis: then the fault is inside its own cell. A jump of `d` between two rays
//! therefore marks exactly the one cell between them, and not the cells beside it, whose far corner
//! vouches for them. A corner that could not be checked defers to the other.
//!
//! **Windings.** The winding (the specification, 4.8) counts whole turns about the far-sky Z axis.
//! A step of one between neighbouring rays is not evidence by itself: it happens wherever the
//! azimuth swept by the light passes a whole number of turns, and `d` is continuous there (a ray
//! that swept 2 pi - x and one that swept 2 pi + x arrive from neighbouring directions). But a
//! step of two or more between rays of one cell means that the light of one went round at least
//! one whole turn more than the other's: between them the rays sweep every azimuth of the far sky,
//! and no four rays can say what lies in the cell, even when their `d` happen to agree. Such a
//! cell is marked. (The one case where that rule marks a smooth map is a ray passing over the far
//! sky's pole, where the azimuth jumps by a half-turn either way; it takes three passes near the
//! pole to make a step of two, which near a hole happens only far inside the band the directions
//! already mark.)
//!
//! **The edge of the dark region.** A pixel whose nearest ray reached the far sky but some of
//! whose four rays did not is marked too. It is interpolated from fewer than four rays, which is
//! extrapolation across part of its cell, and in Kerr the edge of the dark region is exactly
//! where the map has no bound: the rays just outside it approach the photon orbits and wind
//! without limit. Nothing there is known from the rays at hand. The cost is half a cell on the far
//! side of every edge of a dark or unresolved region; the dark side of the edge is drawn dark, as
//! before, since the nearest ray decides fate.
//!
//! **The cost at the margin.** The rule marks where the estimated error exceeds one pixel of sky,
//! so a pixel just outside the band may be wrong by up to about one pixel's footprint. Near a
//! photon-orbit edge the deflection grows as -ln x with the distance x from the edge; there e / 4
//! over the footprint is about s h / (8 x) for rays h apart and s pixels a ray, so at 8 pixels a
//! ray the curvature rule marks the cells within about one step of the edge (the half cell of the
//! edge rule comes on top), and at 16 pixels a ray within two. Halving the threshold would double
//! that band. The estimate is the one a smooth map's second difference gives; a true jump of `d`
//! between two rays is marked when it exceeds eight footprints (or eight texels), and a smaller one
//! escapes, drawn smeared across its cell with an error of up to half the jump. Structure that
//! falls entirely between two rays and leaves no trace on their neighbours cannot be detected from
//! the rays at all; the winding rule catches the largest such, a whole turn.
//! The shift `g` needs no such rule: for light from the far sky it is fixed by the ray's direction
//! at the observer alone, (-p . u) / E, whatever the ray did on the way, so it is smooth wherever
//! the grid is.
//!
//! # Fallbacks here, and what each gives
//!
//! - *The four directions cancel* (two rays of a one-column grid across the pole, or a writer's
//!   antipodal pair): interpolating shows the nearest ray's own direction, a plausible colour the
//!   rays do not support. Judged, the pixel is marked. It cannot arise in a judged field whose
//!   taps passed the checks above, since opposite neighbours make a ray suspect; the rule stays
//!   for the degenerate grids that cannot be checked.
//! - *A far-sky ray whose direction or shift is not a number*, or a shift below zero: a defect in
//!   the bundle, drawn as unresolved, since nothing about that direction is known.
//! - *Between frames, the blend has no length* (the sky turned by half a turn at this pixel between
//!   frames): interpolating shows the nearer frame's direction; judged, the pixel is marked.
//! - *Between frames, the frames disagree about fate*: the nearer frame decides the fate, as the
//!   nearest ray does in space, which places the moment of the change within half a frame. When
//!   the nearer frame saw sky and the other did not (or could not resolve it), the direction is
//!   one frame's, held rather than interpolated: judged, the pixel is marked, as a pixel with
//!   fewer than four sky rays is in space. When the nearer frame saw the dark region or an
//!   unresolved ray, the pixel keeps that meaning.

use sky_format::{Frame, fate};

use crate::bilinear::{Taps, taps};
use crate::parallel::for_each_band;

/// What one direction on the observer's sky shows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ray {
    /// Light from the far sky: the direction at infinity (a unit vector) and the shift.
    Sky { d: [f64; 3], g: f64 },
    /// The black hole's shadow.
    Shadow,
    /// The tracer gave up, or wrote a code this build does not know, or wrote a far-sky ray with
    /// no usable direction. Drawn in a flat artificial colour so that it cannot pass for sky.
    Unresolved,
    /// Light from the far sky, but the rays around this direction are too far apart to say from
    /// where (see the module comment). Only a judged field says this.
    Undersampled,
}

/// How finely a video samples a field, which decides what the field's rays resolve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Judge {
    /// Output pixels per ray, across and down.
    pub pixels_per_ray: (f64, f64),
    /// The star map's finest texel, as an angle in radians: 2 pi over its width.
    pub texel: f64,
}

/// A ray that is suspect along the grid's rows (u) and along its columns (v).
const SUSPECT_U: u8 = 1;
const SUSPECT_V: u8 = 2;

/// One bundle frame, rearranged for sampling: the fate plane as it is, and the direction and
/// shift of each ray packed together, so the four taps of a sample are four reads rather than
/// sixteen scattered over four planes.
#[derive(Debug, Clone)]
pub struct RayField {
    pub width: usize,
    pub height: usize,
    fate: Vec<u8>,
    dg: Vec<[f32; 4]>,
    winding: Vec<i16>,
    /// `SUSPECT_U | SUSPECT_V` bits per ray; empty until the field is judged.
    suspect: Vec<u8>,
}

impl RayField {
    pub fn from_frame(frame: &Frame) -> Self {
        let [x, y, z] = &frame.direction;
        let dg = (0..frame.fate.len())
            .map(|k| [x[k], y[k], z[k], frame.shift[k]])
            .collect();
        Self {
            width: frame.width as usize,
            height: frame.height as usize,
            fate: frame.fate.clone(),
            dg,
            winding: frame.winding.clone(),
            suspect: Vec::new(),
        }
    }

    /// Whether this field marks what its rays do not resolve.
    pub fn is_judged(&self) -> bool {
        !self.suspect.is_empty()
    }

    /// Checks every ray against its neighbours for a video sampled as `judge` says (see the module
    /// comment), on `threads` threads. From then on `sample` answers `Ray::Undersampled` where
    /// the rays do not resolve the sky.
    pub fn judge(&mut self, judge: Judge, threads: usize) {
        let (w, h) = (self.width, self.height);
        // Each ray's unit direction, or None when it is not a far-sky ray with a direction.
        let unit: Vec<Option<[f64; 3]>> = (0..w * h)
            .map(|k| {
                if self.fate[k] != fate::FAR_SKY {
                    return None;
                }
                let [x, y, z, _] = self.dg[k].map(f64::from);
                let n = norm([x, y, z]);
                (n.is_finite() && n > 0.0).then(|| [x / n, y / n, z / n])
            })
            .collect();
        // Suspect along one axis: the error the interpolation beside ray `k` may make is larger
        // than the footprint of a pixel there, or cannot be measured.
        let axis = |a: usize, k: usize, b: usize, pixels_per_ray: f64| -> bool {
            let (Some(da), Some(dk), Some(db)) = (unit[a], unit[k], unit[b]) else {
                return true;
            };
            let sum = [da[0] + db[0], da[1] + db[1], da[2] + db[2]];
            let length = norm(sum);
            // Neighbours within a thousandth of a radian of opposite: their midpoint has no
            // direction worth the name. (Unit vectors: the length is a number.)
            if length <= 1e-3 {
                return true;
            }
            let mid = sum.map(|c| c / length);
            let error = angle(mid, dk) / 4.0;
            let footprint = angle(da, dk).min(angle(dk, db)) / pixels_per_ray;
            error > footprint.max(judge.texel)
        };
        let mut suspect = vec![0u8; w * h];
        for_each_band(&mut suspect, w, 16, threads, |rows, band| {
            for (j, row) in rows.zip(band.chunks_mut(w)) {
                for (i, out) in row.iter_mut().enumerate() {
                    let k = j * w + i;
                    if unit[k].is_none() {
                        // Such a ray is never interpolated from: a far-sky ray without a direction
                        // makes every pixel that reads it unresolved.
                        continue;
                    }
                    let mut bits = 0;
                    // Along the row, across the seam. A grid under three columns wide has the
                    // same ray on both sides, and no second difference to take.
                    if w >= 3 {
                        let (left, right) = (j * w + (i + w - 1) % w, j * w + (i + 1) % w);
                        if axis(left, k, right, judge.pixels_per_ray.0) {
                            bits |= SUSPECT_U;
                        }
                    }
                    // Down the column, and over the poles as `crate::bilinear` reads them: the
                    // row beyond the first is the first, half a turn round.
                    let up = if j > 0 {
                        (j - 1) * w + i
                    } else {
                        (i + w / 2) % w
                    };
                    let down = if j + 1 < h {
                        (j + 1) * w + i
                    } else {
                        (h - 1) * w + (i + w / 2) % w
                    };
                    if up != down
                        && up != k
                        && down != k
                        && axis(up, k, down, judge.pixels_per_ray.1)
                    {
                        bits |= SUSPECT_V;
                    }
                    *out = bits;
                }
            }
        });
        // A field with no rays at all has nothing to judge; one entry keeps `is_judged` true.
        if suspect.is_empty() {
            suspect.push(0);
        }
        self.suspect = suspect;
    }

    /// Whether the rays of a far-sky pixel's cell, `t`, say what it shows (see the module
    /// comment). Only for a judged field.
    fn resolved(&self, t: &Taps) -> bool {
        let (mut lo, mut hi) = (i32::MAX, i32::MIN);
        for k in 0..4 {
            if t.weight[k] == 0.0 {
                continue;
            }
            let r = t.index[k];
            if self.fate[r] != fate::FAR_SKY {
                return false;
            }
            let turns = i32::from(self.winding[r]);
            lo = lo.min(turns);
            hi = hi.max(turns);
        }
        if hi - lo >= 2 {
            return false;
        }
        // An edge of the cell, between taps a and b along `bit`'s axis, is faulty when the pixel
        // interpolates along it and both its ends are suspect.
        let faulty = |a: usize, b: usize, bit: u8| {
            t.weight[a] > 0.0
                && t.weight[b] > 0.0
                && self.suspect[t.index[a]] & bit != 0
                && self.suspect[t.index[b]] & bit != 0
        };
        // The taps are the upper row's left and right, then the lower row's.
        !(faulty(0, 1, SUSPECT_U)
            || faulty(2, 3, SUSPECT_U)
            || faulty(0, 2, SUSPECT_V)
            || faulty(1, 3, SUSPECT_V))
    }

    /// What the rays say at frame coordinates (u, v) of this field's own grid.
    pub fn sample(&self, u: f64, v: f64) -> Ray {
        let t = taps(self.width, self.height, u, v);
        let nearest = t.index[t.nearest()];
        match self.fate[nearest] {
            fate::FAR_SKY => {}
            fate::DARK => return Ray::Shadow,
            _ => return Ray::Unresolved,
        }
        let judged = self.is_judged();
        if judged && !self.resolved(&t) {
            return Ray::Undersampled;
        }
        let mut d = [0.0f64; 3];
        let mut g = 0.0f64;
        let mut total = 0.0f64;
        for k in 0..4 {
            let w = t.weight[k];
            if w == 0.0 || self.fate[t.index[k]] != fate::FAR_SKY {
                continue;
            }
            let [x, y, z, s] = self.dg[t.index[k]].map(f64::from);
            d[0] += w * x;
            d[1] += w * y;
            d[2] += w * z;
            g += w * s;
            total += w;
        }
        let length = norm(d);
        // Rays on opposite sides of a direction can cancel (two rays of a one-column grid across
        // the pole, or a writer's antipodal pair). Interpolating, the nearest ray's own direction
        // is shown; judged, the pixel is marked, since the rays say nothing about what lies
        // between them. A far-sky ray whose direction or shift is not a number, or whose shift is
        // negative, is a defect in the bundle and is drawn as one.
        let d = if length > 1e-9 * total {
            d.map(|c| c / length)
        } else if judged {
            return Ray::Undersampled;
        } else {
            let [x, y, z, _] = self.dg[nearest].map(f64::from);
            let n = norm([x, y, z]);
            [x / n, y / n, z / n]
        };
        let g = g / total;
        if d.iter().all(|c| c.is_finite()) && g.is_finite() && g >= 0.0 {
            Ray::Sky { d, g }
        } else {
            Ray::Unresolved
        }
    }
}

/// The ray between two bundle frames, `w` of the way from `a` to `b`, as the renderer has always
/// blended: the nearer frame decides fate, and where the blend has no direction the nearer
/// frame's is held.
pub fn blend(a: Ray, b: Ray, w: f64) -> Ray {
    let nearer = if w <= 0.5 { a } else { b };
    match (a, b) {
        (Ray::Sky { d: da, g: ga }, Ray::Sky { d: db, g: gb }) => {
            mix(da, ga, db, gb, w).unwrap_or(nearer)
        }
        _ => nearer,
    }
}

/// The ray between two frames of judged fields: as [`blend`], except that a direction that is
/// not interpolated between two known directions is marked rather than held (see the module
/// comment).
pub fn blend_judged(a: Ray, b: Ray, w: f64) -> Ray {
    let nearer = if w <= 0.5 { a } else { b };
    match (a, b) {
        (Ray::Sky { d: da, g: ga }, Ray::Sky { d: db, g: gb }) => {
            mix(da, ga, db, gb, w).unwrap_or(Ray::Undersampled)
        }
        _ => match nearer {
            Ray::Sky { .. } => Ray::Undersampled,
            other => other,
        },
    }
}

/// The linear blend of two far-sky rays, renormalised; None when it has no direction.
fn mix(da: [f64; 3], ga: f64, db: [f64; 3], gb: f64, w: f64) -> Option<Ray> {
    let d: [f64; 3] = std::array::from_fn(|c| (1.0 - w) * da[c] + w * db[c]);
    let length = norm(d);
    (length > 1e-9).then(|| Ray::Sky {
        d: d.map(|c| c / length),
        g: (1.0 - w) * ga + w * gb,
    })
}

pub fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// The angle between two vectors, accurate at every size: atan2 of the cross and dot products.
pub fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    norm(cross).atan2(dot)
}
