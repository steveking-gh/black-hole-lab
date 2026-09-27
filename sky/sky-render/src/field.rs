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

use sky_format::{Frame, fate};

use crate::bilinear::taps;

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
}

/// One bundle frame, rearranged for sampling: the fate plane as it is, and the direction and
/// shift of each ray packed together, so the four taps of a sample are four reads rather than
/// sixteen scattered over four planes.
#[derive(Debug, Clone)]
pub struct RayField {
    pub width: usize,
    pub height: usize,
    fate: Vec<u8>,
    dg: Vec<[f32; 4]>,
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
        }
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
        // the pole, or a writer's antipodal pair); the nearest ray's own direction is then the
        // honest answer. A far-sky ray whose direction or shift is not a number is a defect in
        // the bundle and is drawn as one.
        let d = if length > 1e-9 * total {
            d.map(|c| c / length)
        } else {
            let [x, y, z, _] = self.dg[nearest].map(f64::from);
            let n = norm([x, y, z]);
            [x / n, y / n, z / n]
        };
        let g = g / total;
        if d.iter().all(|c| c.is_finite()) && g.is_finite() {
            Ray::Sky { d, g }
        } else {
            Ray::Unresolved
        }
    }
}

/// The ray between two bundle frames, `w` of the way from `a` to `b`.
pub fn blend(a: Ray, b: Ray, w: f64) -> Ray {
    let nearer = if w <= 0.5 { a } else { b };
    match (a, b) {
        (Ray::Sky { d: da, g: ga }, Ray::Sky { d: db, g: gb }) => {
            let d: [f64; 3] = std::array::from_fn(|c| (1.0 - w) * da[c] + w * db[c]);
            let length = norm(d);
            if length > 1e-9 {
                Ray::Sky {
                    d: d.map(|c| c / length),
                    g: (1.0 - w) * ga + w * gb,
                }
            } else {
                nearer
            }
        }
        _ => nearer,
    }
}

pub fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
