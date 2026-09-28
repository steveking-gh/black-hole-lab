//! Marks: small hollow signs at directions the bundle names (the specification, section 12), such
//! as the direction in which the observer is travelling past a reference observer.
//!
//! The renderer draws a mark without knowing what it means. The manifest declares each mark once
//! (an id, a label, a shape), and each frame's entry may give, for some of them, the unit vector
//! `n` in the observer's triad along which the observer looks to see it, exactly as a pixel's `n`
//! is. A frame that omits a mark has no sign for it: the direction does not exist there.
//!
//! # An annotation, not light
//!
//! A sign is drawn over the finished picture, on its sRGB-encoded 16-bit codes, after the tone
//! curve, as the read-out panels are (`crate::overlay`, "Look and blending"): neither the exposure
//! nor a shift touches it, and its colour is the code written here. Marks are drawn before the
//! panels, so that a panel placed by hand over a mark stays readable; two marks that overlap are
//! both drawn, in the order the manifest declares them, the later over the earlier.
//!
//! It hides as little sky as it can. It is small (`--mark-size`, 1.5 degrees from the centre to
//! the outer edge by default) and hollow: a stroke round the direction, and the sky at the
//! direction itself, the sign's centre, left in view.
//!
//! # The colour
//!
//! By default a clear green, 33FF66. Under the renderer's blackbody model (`--colour blackbody`,
//! the default; `crate::colour`) every pixel of sky is the colour of a blackbody at some
//! temperature: the light of the sky runs from red through orange and white to blue along the
//! Planckian locus, and that locus never passes through green. A green sign therefore cannot be
//! taken for light. (Under `--colour map` the map's own colours are shown, and a map may hold a
//! greenish nebula; under `--show-model-range` the sky is drawn in false colours, some of them
//! greens. The outline below still sets the sign apart there, but the argument from the colour
//! alone is the blackbody model's.)
//!
//! The specification forbids the colour the renderer uses for what it does not know: a sign drawn
//! in it would read as a pixel the program cannot determine, or hide one. [`check_colour`] refuses
//! a mark colour equal to the unresolved colour, or to the under-sampled colour when under-sampled
//! pixels are marked.
//!
//! # The outline
//!
//! The stroke has a thin dark outline on both sides, so that the sign reads over black sky (the
//! green stroke against black) and over sky clipped to white (the dark lines against white) alike.
//! Both are drawn within the sign's size: nothing of a sign lies farther than `--mark-size` from
//! its direction.
//!
//! Measured on 8192 x 4096 photographs seen through a flat view 90 degrees wide on 1920 pixels (the
//! `v360` filter of ffmpeg, about 21 screen pixels a degree, and about 12 on a screen showing 90
//! degrees on 1080 rows): at the default size the stroke is [`STROKE_PER_SIZE`] of the size, 0.2
//! degree, about 4.6 pixels of the 8K frame; each outline is [`OUTLINE_PER_SIZE`] of it, 0.08
//! degree, about 1.8 pixels of the frame and one pixel of a screen showing 12 pixels a degree;
//! the outline is opaque black. Tried against 0.06 degree at 85 % opacity on the same pictures:
//! over clipped white that outline read as a faint grey line and the green, nearly as bright as
//! the white, barely stood off it; over dark sky the two looked the same, the outline invisible
//! against black either way, as it should be.
//!
//! # The geometry
//!
//! A sign is a flat figure on the plane tangent to the sphere at its direction, mapped to the
//! sphere by the gnomonic (central) projection, as a panel is (`crate::panel`, "The mapping"): a
//! player shows the sphere through a pinhole at its centre, which is exactly what sees a flat plate
//! as flat, so a ring looks round and a triangle straight-sided wherever the viewer turns. With the
//! direction `c = n / |n|` at longitude `lambda` (heading, to the right) and latitude `beta`,
//!
//! ```text
//! lambda = -atan2(n_y, n_x)
//! r      = ( -sin lambda, -cos lambda, 0 )        the sign's rightward direction
//! u      = r x c                                  the sign's upward direction, toward +z
//! x      = (m . r) / (m . c),   y = (m . u) / (m . c)
//! ```
//!
//! for a direction `m` in front of the plane: the plane's coordinates, in which 1 is the distance
//! from the eye. `r` is level and `u` points up the sky, so a triangle's point is down, toward the
//! nadir, however the sign is turned to. The panel's formulae, but built from `n` itself and not
//! from angles in degrees, and with `r` from `atan2`, which is defined at the poles too: at a pole
//! `lambda` is 0 and the sign is drawn upright as seen when facing heading 0. Nothing here divides
//! by the cosine of the latitude, so a sign at a pole, or across the seam behind the observer, is
//! drawn whole: the pixels are found on the sphere and only then given column numbers, modulo the
//! frame's width.
//!
//! On the plane each shape is its outer outline, a figure whose farthest point from the centre is
//! `R = tan(size)`, so that its farthest point on the sphere is `size` from the direction:
//!
//! - `ring`: the circle of radius `R`;
//! - `diamond`: the square stood on a corner, corners at `(±R, 0)` and `(0, ±R)`;
//! - `triangle`: the equilateral triangle with a corner at `(0, -R)`, point down, and the others at
//!   `(±R cos 30°, R sin 30°)`;
//! - any other shape name: a ring, as the specification says.
//!
//! A point's signed distance `d` to the outline, negative inside, is `|p| - R` for the ring, and
//! for the polygons the largest of the signed distances to the lines of their edges, which inside
//! a convex polygon is exactly the distance to its outline (outside it would underestimate, but
//! nothing outside is drawn). Inside, the sign is four bands of `d`, from the outline in: the outer
//! dark outline, the stroke, the inner dark outline, and the hollow. Each edge between bands is
//! antialiased by the fraction of a pixel on either side of it, a pixel being `2 pi / W` of the
//! plane (a row's height in angle). The outermost edge sits half a pixel inside the outline, so
//! that its ramp ends at the outline and not beyond.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use sky_format::{MarkDecl, Num};

use crate::render::Size;
use crate::timeline::Pick;

/// `--mark-size` by default, in degrees.
pub const DEFAULT_SIZE_DEGREES: f64 = 1.5;

/// `--mark-colour` by default, 33FF66, as 16-bit sRGB-encoded codes.
pub const DEFAULT_COLOUR: [u16; 3] = [0x33 * 257, 0xFF * 257, 0x66 * 257];

/// The width of the stroke, as a fraction of the size: 0.2 degree at the default 1.5.
pub const STROKE_PER_SIZE: f64 = 0.2 / 1.5;

/// The width of each dark outline, as a fraction of the size: 0.08 degree at the default 1.5.
pub const OUTLINE_PER_SIZE: f64 = 0.08 / 1.5;

/// How much of what is under the outline it hides: black at this opacity, all of it (the module's
/// comment, "The outline").
pub const OUTLINE_OPACITY: f32 = 1.0;

/// A mark's sign.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shape {
    Ring,
    Diamond,
    Triangle,
}

impl Shape {
    /// The sign a manifest's `shape` names; a ring for a name this renderer does not know.
    pub fn of(name: &str) -> Self {
        match name {
            "diamond" => Self::Diamond,
            "triangle" => Self::Triangle,
            _ => Self::Ring,
        }
    }

    /// The word the run writes for the sign drawn.
    pub fn name(self) -> &'static str {
        match self {
            Self::Ring => "ring",
            Self::Diamond => "diamond",
            Self::Triangle => "triangle",
        }
    }
}

/// The marks of a run: each declared mark, and its direction at each bundle frame being played.
#[derive(Debug, Clone, PartialEq)]
pub struct Marks {
    /// Each declared mark's label and sign, in the manifest's order.
    pub declared: Vec<(String, Shape)>,
    /// `directions[p][m]`: mark `m` at the `p`th bundle frame played, as `Pick` counts them;
    /// None where the frame omits it.
    directions: Vec<Vec<Option<[f64; 3]>>>,
}

impl Marks {
    /// `frames` holds the `marks` object of each bundle frame played, in order.
    pub fn new(decls: &[MarkDecl], frames: &[&BTreeMap<String, [Num; 3]>]) -> Self {
        Self {
            declared: decls
                .iter()
                .map(|d| (d.label.clone(), Shape::of(&d.shape)))
                .collect(),
            directions: frames
                .iter()
                .map(|marks| {
                    decls
                        .iter()
                        .map(|d| marks.get(&d.id).map(|n| n.map(|c| c.0)))
                        .collect()
                })
                .collect(),
        }
    }

    /// Each mark's direction at a video frame made of `pick`: None for a mark not drawn there.
    pub fn at(&self, pick: Pick) -> Vec<Option<[f64; 3]>> {
        (0..self.declared.len())
            .map(|m| match pick {
                Pick::One(a) => self.directions[a][m],
                Pick::Two(a, b, w) => turn(self.directions[a][m], self.directions[b][m], w),
            })
            .collect()
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let length = dot(v, v).sqrt();
    v.map(|c| c / length)
}

/// The direction `w` of the way from `a` to `b`, turned along the great circle between them, at
/// an even rate of angle (the spherical interpolation of the two unit vectors):
///
/// ```text
/// theta = angle(a, b),   n = (sin((1 - w) theta) a + sin(w theta) b) / sin theta
/// ```
///
/// Where only one of the two is given, the nearer one's, as it is, or nothing: exactly half way
/// counts as nearer the second, as for a read-out (`crate::values::interpolate`). Two directions
/// opposite each other have no one great circle between them, and are treated as if only the
/// nearer were given; that is a turn of 180 degrees between two bundle frames, which a writer
/// sampling a motion does not make.
pub fn turn(a: Option<[f64; 3]>, b: Option<[f64; 3]>, w: f64) -> Option<[f64; 3]> {
    match (a, b) {
        (Some(a), Some(b)) => {
            let (a, b) = (unit(a), unit(b));
            let theta = crate::field::angle(a, b);
            if theta < 1e-9 {
                return Some(unit(std::array::from_fn(|k| a[k] + w * (b[k] - a[k]))));
            }
            if theta > PI - 1e-6 {
                return Some(if w < 0.5 { a } else { b });
            }
            let (p, q) = (((1.0 - w) * theta).sin(), (w * theta).sin());
            let s = theta.sin();
            Some(unit(std::array::from_fn(|k| (p * a[k] + q * b[k]) / s)))
        }
        _ if w < 0.5 => a,
        _ => b,
    }
}

/// How the signs are drawn: `--mark-size` and `--mark-colour`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pen {
    pub size_degrees: f64,
    pub colour: [u16; 3],
}

/// One output pixel a sign touches, and how much of it each of the sign's two inks covers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ink {
    /// The pixel's position in the frame, `j * W + i`.
    pub pixel: u32,
    /// The dark outlines and everything between them, in [0, 1].
    pub dark: f32,
    /// The stroke, in [0, 1]; never more than `dark`.
    pub colour: f32,
}

/// A sign on the sphere: its shape, its tangent plane, and its bands, all on the plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sign {
    shape: Shape,
    /// The unit vectors c, r and u of the module's comment.
    centre: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    /// The angle from the centre to the outer outline, radians, and `R`, its tangent.
    size: f64,
    outer: f64,
    stroke: f64,
    outline: f64,
}

impl Sign {
    /// The sign `shape` at direction `n` (any length but zero), `size_degrees` from its centre to
    /// its outer outline.
    pub fn new(shape: Shape, n: [f64; 3], size_degrees: f64) -> Self {
        let centre = unit(n);
        let lambda = -centre[1].atan2(centre[0]);
        let right = [-lambda.sin(), -lambda.cos(), 0.0];
        let up = cross(right, centre);
        let size = size_degrees.to_radians();
        Self {
            shape,
            centre,
            right,
            up,
            size,
            outer: size.tan(),
            stroke: STROKE_PER_SIZE * size,
            outline: OUTLINE_PER_SIZE * size,
        }
    }

    /// The sign's direction, of unit length.
    #[cfg(test)]
    pub fn centre(&self) -> [f64; 3] {
        self.centre
    }

    /// Where direction `m` (any length but zero) meets the sign's plane, as (x, y) with x to the
    /// right and y up; None for a direction at or behind the plane's horizon.
    pub fn locate(&self, m: [f64; 3]) -> Option<(f64, f64)> {
        let depth = dot(m, self.centre);
        if depth <= 1e-3 * dot(m, m).sqrt() {
            return None;
        }
        Some((dot(m, self.right) / depth, dot(m, self.up) / depth))
    }

    /// The signed distance from (x, y) on the plane to the sign's outer outline, negative inside
    /// (the module's comment, "The geometry").
    pub fn distance(&self, x: f64, y: f64) -> f64 {
        let r = self.outer;
        match self.shape {
            Shape::Ring => x.hypot(y) - r,
            // The four edges' outward normals are (±1, ±1) / sqrt 2, at R / sqrt 2 from the centre.
            Shape::Diamond => (x.abs() + y.abs() - r) * std::f64::consts::FRAC_1_SQRT_2,
            // Outward normals up, and 30 degrees below the horizontal either way, each edge at the
            // inradius R / 2.
            Shape::Triangle => {
                let (s, c) = (0.5, 0.75f64.sqrt());
                let top = y;
                let sides = c * x.abs() - s * y;
                top.max(sides) - 0.5 * r
            }
        }
    }

    /// How much of a pixel whose centre reads (x, y) each ink covers, a pixel being `pixel` of the
    /// plane.
    pub fn inks(&self, x: f64, y: f64, pixel: f64) -> (f32, f32) {
        let d = self.distance(x, y);
        // The coverage of the part of the pixel where the distance is below `edge`: the box
        // filter of a straight edge a pixel wide.
        let below = |edge: f64| ((edge - d) / pixel + 0.5).clamp(0.0, 1.0);
        let e0 = -0.5 * pixel;
        let e1 = e0 - self.outline;
        let e2 = e1 - self.stroke;
        let e3 = e2 - self.outline;
        (
            (below(e0) - below(e3)) as f32,
            (below(e1) - below(e2)) as f32,
        )
    }

    /// Every output pixel of a frame of `size` the sign touches, and how much.
    pub fn cover(&self, size: Size) -> Vec<Ink> {
        let pixel = 2.0 * PI / size.width as f64;
        let mut out = Vec::new();
        for_each_in_cap(size, self.centre, self.size, |i, j, m| {
            let Some((x, y)) = self.locate(m) else {
                return;
            };
            let (dark, colour) = self.inks(x, y, pixel);
            if dark > 0.0 {
                out.push(Ink {
                    pixel: (j * size.width + i) as u32,
                    dark,
                    colour,
                });
            }
        });
        out
    }
}

/// Calls `f(i, j, n)` for every pixel (i, j) of a frame of `size`, with its direction `n`, that
/// may lie within `radius` radians of the unit vector `centre`, and for some that do not (the
/// caller tests each). Row by row, the longitudes of the cap follow from the spherical law of
/// cosines, as in `crate::panel`'s `Panel::cover`; where that would divide by the cosine of a
/// latitude near zero (a cap at or near a pole) a row is taken whole, and a row the cap wraps
/// round a pole is taken whole too. Columns are taken modulo W, which is what draws a sign across
/// the seam.
pub fn for_each_in_cap(
    size: Size,
    centre: [f64; 3],
    radius: f64,
    mut f: impl FnMut(usize, usize, [f64; 3]),
) {
    let (w, h) = (size.width, size.height);
    let (wf, hf) = (w as f64, h as f64);
    let step = PI / hf;
    let elevation = centre[2].atan2(centre[0].hypot(centre[1]));
    let lambda = -centre[1].atan2(centre[0]);
    let reach = radius + step;
    let top = (elevation + reach).min(0.5 * PI);
    let bottom = (elevation - reach).max(-0.5 * PI);
    let first = ((0.5 - top / PI) * hf).floor().max(0.0) as usize;
    let last = (((0.5 - bottom / PI) * hf).ceil() as usize).min(h);
    let (se, ce) = elevation.sin_cos();
    for j in first..last {
        let beta = (0.5 - (j as f64 + 0.5) / hf) * PI;
        let (sb, cb) = beta.sin_cos();
        let across = cb * ce;
        let all = if across < 1e-9 {
            true
        } else {
            let cos_dl = (reach.cos() - sb * se) / across;
            if cos_dl >= 1.0 {
                continue;
            }
            cos_dl <= -1.0
        };
        let columns: Box<dyn Iterator<Item = usize>> = if all {
            Box::new(0..w)
        } else {
            let cos_dl = (reach.cos() - sb * se) / across;
            let half = cos_dl.acos() + 2.0 * (2.0 * PI / wf);
            let from = ((lambda - half) / (2.0 * PI) + 0.5) * wf;
            let to = ((lambda + half) / (2.0 * PI) + 0.5) * wf;
            if to - from + 2.0 >= wf {
                Box::new(0..w)
            } else {
                Box::new(
                    (from.floor() as i64..=to.ceil() as i64)
                        .map(move |k| k.rem_euclid(w as i64) as usize),
                )
            }
        };
        for i in columns {
            let phi = -((i as f64 + 0.5) / wf - 0.5) * 2.0 * PI;
            f(i, j, [cb * phi.cos(), cb * phi.sin(), sb]);
        }
    }
}

/// The signs of the marks given at one video frame, in the order declared: `directions` holds one
/// entry per declared mark, None for a mark not drawn there.
pub fn signs(marks: &Marks, directions: &[Option<[f64; 3]>], pen: Pen) -> Vec<Sign> {
    marks
        .declared
        .iter()
        .zip(directions)
        .filter_map(|((_, shape), n)| n.map(|n| Sign::new(*shape, n, pen.size_degrees)))
        .collect()
}

/// Draws `signs`, in order, over a finished frame of `size` of 16-bit sRGB-encoded codes: at each
/// pixel a sign touches, the dark ink first (black at [`OUTLINE_OPACITY`]), then the colour over
/// it. Pixels no sign touches are not touched.
pub fn paint(frame: &mut [u16], size: Size, signs: &[Sign], colour: [u16; 3]) {
    for sign in signs {
        composite(frame, &sign.cover(size), colour);
    }
}

/// Composites one sign's inks over a frame.
pub fn composite(frame: &mut [u16], inks: &[Ink], colour: [u16; 3]) {
    for ink in inks {
        let keep = 1.0 - OUTLINE_OPACITY * ink.dark;
        let at = 3 * ink.pixel as usize;
        for (c, code) in frame[at..at + 3].iter_mut().enumerate() {
            let under = f32::from(*code) * keep;
            *code = (under + ink.colour * (f32::from(colour[c]) - under) + 0.5) as u16;
        }
    }
}

/// The heading (to the right of the opening view, in (-180, 180]) and elevation, in degrees, of
/// direction `n`, by the specification's inverse formulae (section 4.3).
pub fn heading_elevation(n: [f64; 3]) -> (f64, f64) {
    let lambda = (-n[1].atan2(n[0])).to_degrees();
    let heading = if lambda <= -180.0 {
        lambda + 360.0
    } else {
        lambda
    };
    (heading, n[2].atan2(n[0].hypot(n[1])).to_degrees())
}

/// A plain word for the mark colour in force: "green" for the default, and otherwise its hex code.
pub fn colour_name(colour: [u16; 3]) -> String {
    if colour == DEFAULT_COLOUR {
        return "green".into();
    }
    let [r, g, b] = colour.map(|c| c / 257);
    format!("#{r:02X}{g:02X}{b:02X}")
}

/// What a still's run says of its marks: one line for each mark drawn, in the order declared.
pub fn still_lines(marks: &Marks, directions: &[Option<[f64; 3]>], pen: Pen) -> Vec<String> {
    marks
        .declared
        .iter()
        .zip(directions)
        .filter_map(|((label, shape), n)| {
            let (h, e) = heading_elevation((*n)?);
            Some(format!(
                "mark: a {} in {} at {} degrees right of the opening view and {} degrees up: {label}",
                shape.name(),
                colour_name(pen.colour),
                crate::shadow::degrees(h),
                crate::shadow::degrees(e)
            ))
        })
        .collect()
}

/// What a film's run says of its marks: one line, or none when there are none to draw.
pub fn film_line(marks: &Marks, pen: Pen) -> Option<String> {
    (!marks.declared.is_empty()).then(|| {
        format!(
            "marks: {} declared, drawn in {}",
            marks.declared.len(),
            colour_name(pen.colour)
        )
    })
}

/// Refuses a mark colour that is the colour of what the program does not know: the unresolved
/// colour, or the under-sampled colour when under-sampled pixels are marked (`undersampled` is
/// then Some). The specification, section 12: a sign must not be drawn in that colour.
pub fn check_colour(
    colour: [u16; 3],
    unresolved: [u16; 3],
    undersampled: Option<[u16; 3]>,
) -> Result<(), String> {
    let [r, g, b] = colour.map(|c| c / 257);
    let hex = format!("{r:02X}{g:02X}{b:02X}");
    let clash = if colour == unresolved {
        Some("--unresolved-colour, the colour of rays the tracer did not resolve")
    } else if undersampled == Some(colour) {
        Some("--undersampled-colour, the colour of pixels the rays are too far apart to determine")
    } else {
        None
    };
    match clash {
        Some(what) => Err(format!(
            "--mark-colour {hex} is also {what}, and a mark must never be drawn in the colour of \
             what the program does not know; choose another --mark-colour, or --marks off"
        )),
        None => Ok(()),
    }
}
