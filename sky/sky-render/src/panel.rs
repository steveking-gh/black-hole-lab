//! Where a read-out panel sits on the sphere, and which output pixels show it.
//!
//! A 360-degree video has no screen to write on: whatever is drawn is drawn on the sphere of
//! directions, and the viewer sees it only when looking that way. Text painted straight into the
//! equirectangular frame would come out stretched across and bent into an arc in the viewer. So a
//! panel is a flat rectangle tangent to the sphere at a chosen direction, the text is laid out on
//! that rectangle, and each output pixel whose direction passes through the rectangle takes its
//! colour from the point it passes through: the gnomonic (central) projection about the panel's
//! centre. A player shows the sphere through a pinhole at its centre, and the gnomonic projection
//! is exactly what such a pinhole sees of a flat plate, so the plate reads flat and its straight
//! lines stay straight however the viewer turns.
//!
//! # The mapping
//!
//! From the specification's section 4.3, output pixel (i, j) of a W x H frame looks along
//!
//! ```text
//! lambda = ((i + 0.5) / W - 0.5) 2 pi,   beta = (0.5 - (j + 0.5) / H) pi,   phi = -lambda
//! n      = (cos beta cos phi, cos beta sin phi, sin beta)
//! ```
//!
//! A panel is placed at heading `h` (degrees to the viewer's RIGHT of the frame's centre) and
//! elevation `e` (up from the equator). Heading to the right is longitude `lambda`, since moving
//! right in the frame raises `lambda`, so the panel's centre is the `n` of `lambda = h`,
//! `beta = e`:
//!
//! ```text
//! c = ( cos e cos h, -cos e sin h, sin e )
//! ```
//!
//! On the tangent plane at `c`, two unit vectors span the panel:
//!
//! ```text
//! r = dc/dlambda / |dc/dlambda| = ( -sin h, -cos h, 0 )             the panel's rightward edge
//! u = r x c                     = ( -cos h sin e, sin h sin e, cos e )  the panel's upward edge
//! ```
//!
//! `r` has no z component, so the panel's horizontal edges are level: parallel, at its centre, to
//! the frame's equator. `u` has a positive z component, so the panel's top is toward the frame's
//! top. A direction `n` in front of the panel (n . c > 0) meets the plane at unit distance at
//!
//! ```text
//! x = (n . r) / (n . c),   y = (n . u) / (n . c)
//! ```
//!
//! lengths on the plane in which 1 is the distance from the eye; the panel's pixels are `pixel`
//! of these long, and panel coordinates run from the top-left corner, X to the right and Y down:
//! `X = W_p / 2 + x / pixel`, `Y = H_p / 2 - y / pixel`.
//!
//! **Why the text is not mirrored.** Azimuth `phi` increases to the observer's LEFT (section 4.2),
//! and a mapping built on `phi` would write text right to left. This one is built on `r`, the
//! direction in which `lambda` increases: in the frame, moving right turns the viewer toward `-y`,
//! the observer's right, and `r` at heading 0 is exactly `-y`. A step to the right in the frame is
//! a step to larger X on the panel, and a player, which shows the sphere from inside with the
//! frame's right to the viewer's right, shows the text left to right. Moving down the frame
//! lowers `beta`, and with it `n . u`, so it is a step to larger Y: the text is upright.
//!
//! # The seam and the poles
//!
//! Nothing above cares where the frame's seam is: the pixels a panel covers are found on the
//! sphere and only then given column numbers, modulo W. A panel behind the observer (heading 180)
//! comes out whole, half at each edge of the frame. The poles are another matter: a panel near
//! one would be drawn across it, which the gnomonic projection handles but a viewer does not want,
//! and the command line refuses an elevation within [`POLE_CLEARANCE_DEGREES`] of either pole.

use std::f64::consts::PI;

use crate::render::Size;

/// How near a pole a panel's centre may be, in degrees.
pub const POLE_CLEARANCE_DEGREES: f64 = 20.0;

/// Where a panel's centre is, in degrees: `heading` to the viewer's right of the frame's centre,
/// `elevation` up from the frame's equator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    pub heading: f64,
    pub elevation: f64,
}

impl Placement {
    /// Below the opening view and clear of the equator, where a black hole and its neighbourhood
    /// will be.
    pub const DEFAULT: Self = Self {
        heading: 0.0,
        elevation: -30.0,
    };

    /// `<heading>,<elevation>` in degrees, as `--readout-at` takes it.
    pub fn parse(text: &str) -> Result<Self, String> {
        let malformed = || {
            format!(
                "--readout-at is <heading>,<elevation> in degrees with a point for any decimals, \
                 such as 90,-30 or 12.5,-20, or for a still the word dark, not {text:?}"
            )
        };
        let mut parts = text.split(',');
        let (Some(heading), Some(elevation), None) = (parts.next(), parts.next(), parts.next())
        else {
            return Err(malformed());
        };
        let number = |s: &str| s.trim().parse::<f64>().ok();
        let (Some(heading), Some(elevation)) = (number(heading), number(elevation)) else {
            return Err(malformed());
        };
        // Rust reads "inf" and "NaN" as numbers; neither is a direction.
        if !(heading.is_finite() && elevation.is_finite()) {
            return Err(format!(
                "--readout-at {text:?} is not a direction: the heading and the elevation must be \
                 finite numbers of degrees"
            ));
        }
        if elevation.abs() > 90.0 {
            return Err(format!(
                "--readout-at {text:?} has an elevation past a pole: elevations run from -90 \
                 (straight down) to 90 (straight up)"
            ));
        }
        let limit = 90.0 - POLE_CLEARANCE_DEGREES;
        if elevation.abs() > limit {
            return Err(format!(
                "--readout-at {text:?} puts the panel within {POLE_CLEARANCE_DEGREES} degrees of \
                 a pole, where it would be drawn across the pole; choose an elevation from \
                 -{limit} to {limit}"
            ));
        }
        Ok(Self { heading, elevation })
    }
}

/// One output pixel a panel covers: where on the panel it reads, and how much of it the panel
/// covers at the panel's rounded edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tap {
    /// The pixel's position in the frame, `j * W + i`.
    pub pixel: u32,
    /// Panel coordinates, in panel pixels from the top-left corner.
    pub x: f32,
    pub y: f32,
    /// In (0, 1]: 1 inside the panel, falling to 0 over the last panel pixel at its edge.
    pub edge: f32,
}

/// A panel on the sphere: its place, its size in panel pixels, and the size of a panel pixel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Panel {
    /// The heading (as a longitude, radians) and elevation (radians) of the centre.
    lambda: f64,
    elevation: f64,
    /// The unit vectors c, r and u of the module's comment.
    centre: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    /// A panel pixel's length on the tangent plane at unit distance.
    pixel: f64,
    width: f64,
    height: f64,
    /// The radius of the panel's corners, in panel pixels.
    radius: f64,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

impl Panel {
    /// A panel `width` x `height` panel pixels, each `pixel` long on the tangent plane, with
    /// corners rounded to `radius` panel pixels.
    pub fn new(at: Placement, pixel: f64, width: usize, height: usize, radius: f64) -> Self {
        let lambda = wrap(at.heading.to_radians());
        let elevation = at.elevation.to_radians();
        let (sh, ch) = lambda.sin_cos();
        let (se, ce) = elevation.sin_cos();
        let (width, height) = (width as f64, height as f64);
        Self {
            lambda,
            elevation,
            centre: [ce * ch, -ce * sh, se],
            right: [-sh, -ch, 0.0],
            up: [-ch * se, sh * se, ce],
            pixel,
            width,
            height,
            radius: radius.clamp(0.0, 0.5 * width.min(height)),
        }
    }

    /// The panel's outline pushed out by `by` panel pixels all round, at the same centre: the
    /// points of the tangent plane within `by` of the panel. A rounded rectangle is a rectangle
    /// swept by a disc of its corner radius, so sweeping it by a disc of radius `by` as well gives
    /// the rectangle `by` larger on each side with corners of radius `radius + by`, exactly.
    pub fn grown(&self, by: f64) -> Self {
        Self {
            width: self.width + 2.0 * by,
            height: self.height + 2.0 * by,
            radius: self.radius + by,
            ..*self
        }
    }

    #[cfg(test)]
    pub fn width(&self) -> f64 {
        self.width
    }

    #[cfg(test)]
    pub fn height(&self) -> f64 {
        self.height
    }

    /// Where direction `n` (any length but zero) meets the panel's plane, in panel coordinates;
    /// None for a direction at or behind the plane's horizon.
    pub fn locate(&self, n: [f64; 3]) -> Option<(f64, f64)> {
        let depth = dot(n, self.centre);
        // A direction 89.9 degrees off the centre meets the plane 600 widths away; anything
        // nearer the horizon than that is nowhere near a panel a few degrees across.
        if depth <= 1e-3 * (dot(n, n)).sqrt() {
            return None;
        }
        let x = dot(n, self.right) / depth;
        let y = dot(n, self.up) / depth;
        Some((
            0.5 * self.width + x / self.pixel,
            0.5 * self.height - y / self.pixel,
        ))
    }

    #[cfg(test)]
    /// The unit direction through panel coordinates (X, Y): the inverse of [`Panel::locate`].
    pub fn direction(&self, x: f64, y: f64) -> [f64; 3] {
        let a = (x - 0.5 * self.width) * self.pixel;
        let b = (0.5 * self.height - y) * self.pixel;
        let v: [f64; 3] =
            std::array::from_fn(|k| self.centre[k] + a * self.right[k] + b * self.up[k]);
        let length = dot(v, v).sqrt();
        v.map(|c| c / length)
    }

    /// How much of a pixel whose centre reads panel coordinates (X, Y) the panel covers: 1 more
    /// than a panel pixel inside its rounded outline, 0 outside, and in between a ramp one panel
    /// pixel wide, just inside the outline, which antialiases the edge without reaching past it.
    pub fn coverage(&self, x: f64, y: f64) -> f64 {
        // The signed distance to a rounded rectangle, negative inside.
        let r = self.radius;
        let qx = (x - 0.5 * self.width).abs() - (0.5 * self.width - r);
        let qy = (y - 0.5 * self.height).abs() - (0.5 * self.height - r);
        let outside = qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r;
        (-outside).clamp(0.0, 1.0)
    }

    /// The angle from the centre to the panel's furthest corner: the panel lies inside the cap of
    /// that radius about its centre.
    pub fn angular_radius(&self) -> f64 {
        ((0.5 * self.width).hypot(0.5 * self.height) * self.pixel).atan()
    }

    /// The angle from the centre to the nearest point of the panel's outline, the middle of its
    /// longer edges: the panel holds the cap of this radius about its centre, and no larger one.
    pub fn inner_radius(&self) -> f64 {
        (0.5 * self.width.min(self.height) * self.pixel).atan()
    }

    /// A panel pixel's length on the tangent plane.
    pub fn pixel(&self) -> f64 {
        self.pixel
    }

    /// Whether the panel reaches a pole. The panel's point furthest from the equator is the
    /// middle of its top edge (or its bottom edge, below the equator): along an edge the height
    /// above the equator is c_z + y u_z over |c + x r + y u|, and r has no z, so moving along the
    /// edge away from its middle only lengthens the denominator. That point is at elevation
    /// e + atan(half height).
    pub fn reaches_pole(&self) -> bool {
        self.elevation.abs() + (0.5 * self.height * self.pixel).atan() >= 0.5 * PI
    }

    /// Every output pixel of a frame of `size` that the panel covers, found once for a run: the
    /// panel's place never changes, only its text.
    ///
    /// The candidates are the pixels inside the cap that holds the panel. Row by row, the cap's
    /// half-width in longitude at latitude b follows from the spherical law of cosines,
    /// cos rho = sin b sin e + cos b cos e cos(dlambda); two columns are added either side for
    /// the rounding of pixel centres, and each candidate is then tested exactly against the
    /// panel's outline. Columns are taken modulo W, which is what draws a panel across the seam.
    pub fn cover(&self, size: Size) -> Vec<Tap> {
        let (w, h) = (size.width, size.height);
        let (wf, hf) = (w as f64, h as f64);
        let rho = self.angular_radius();
        let step = PI / hf;
        let top = (self.elevation + rho + step).min(0.5 * PI);
        let bottom = (self.elevation - rho - step).max(-0.5 * PI);
        let first = ((0.5 - top / PI) * hf).floor().max(0.0) as usize;
        let last = (((0.5 - bottom / PI) * hf).ceil() as usize).min(h);
        let (se, ce) = self.elevation.sin_cos();
        let mut taps = Vec::new();
        for j in first..last {
            let beta = (0.5 - (j as f64 + 0.5) / hf) * PI;
            let (sb, cb) = beta.sin_cos();
            let cos_dl = (rho.cos() - sb * se) / (cb * ce);
            if cos_dl > 1.0 {
                continue;
            }
            let half = if cos_dl <= -1.0 {
                PI
            } else {
                cos_dl.acos() + 2.0 * (2.0 * PI / wf)
            };
            let from = ((self.lambda - half) / (2.0 * PI) + 0.5) * wf;
            let to = ((self.lambda + half) / (2.0 * PI) + 0.5) * wf;
            let columns: Box<dyn Iterator<Item = usize>> = if to - from + 2.0 >= wf {
                Box::new(0..w)
            } else {
                Box::new(
                    (from.floor() as i64..=to.ceil() as i64)
                        .map(move |k| k.rem_euclid(w as i64) as usize),
                )
            };
            for i in columns {
                let phi = -((i as f64 + 0.5) / wf - 0.5) * 2.0 * PI;
                let n = [cb * phi.cos(), cb * phi.sin(), sb];
                let Some((x, y)) = self.locate(n) else {
                    continue;
                };
                let edge = self.coverage(x, y);
                if edge > 0.0 {
                    taps.push(Tap {
                        pixel: (j * w + i) as u32,
                        x: x as f32,
                        y: y as f32,
                        edge: edge as f32,
                    });
                }
            }
        }
        taps
    }
}

/// An angle taken into (-pi, pi].
fn wrap(angle: f64) -> f64 {
    let a = angle.rem_euclid(2.0 * PI);
    if a > PI { a - 2.0 * PI } else { a }
}
