use std::collections::VecDeque;
use std::ops::Range;

use crate::gui::axis;
use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::polyline::{SCREEN_SPACING, thin_to_pixels};
use crate::gui::spacetime_canvas::{CHART_BANNER, COARSE_ZOOM_STEPS, BoxId, Canvas, TelemetryBoxes};
use crate::gui::spatial_canvas::{
    CENTRED_RING_GAP, FrontStyle, Who, draw_reception_tick, draw_signal_field, draw_spatial_trail,
    frame_focus,
};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, TrailPoint};
use crate::physics::wavefront::{NullRay, RaySample, SignalField};
use egui::{Color32, Pos2, Stroke, Vec2};

/// How close to 0 or to +/-1 a basis component has to be before it is taken to *be* 0 or +/-1.
///
/// Rotating by an angle that is a multiple of pi/2 gives components that miss their exact value by
/// about one part in 10^7 in f32, and a floor placed at x * 0.99999994 is a floor drawn a pixel
/// away from the one the equatorial view draws. Snapping costs three comparisons per component and
/// buys the Top preset an exact agreement with `SpatialCanvas`.
const SNAP: f32 = 1e-6;

/// Orthographic orbit camera over the (x, y, t) volume. World coordinates: x, y in M from the
/// Kerr-Schild embedding, z = (t - t_now) * t_scale, computed by the caller in f64.
///
/// Orthographic rather than perspective because the picture is a diagram: a light cone at the far
/// edge of the volume has to be measurable against one at the near edge, and under a perspective
/// divide the two have different opening angles on screen for no physical reason. It also keeps
/// `scale` meaning exactly what `SpatialCanvas::zoom` means - pixels per M, everywhere in the
/// frame - so the two canvases can be zoomed together.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Camera {
    /// Rotation of the floor about the vertical axis, radians.
    pub yaw: f32,
    /// Elevation of the eye above the floor: pi/2 is straight down, 0 is edge-on.
    pub pitch: f32,
    /// Pixels per M, the same meaning as `SpatialCanvas::zoom`.
    pub scale: f32,
    /// Screen-pixel offset, the same meaning as `SpatialCanvas::pan_offset`. `project` does not
    /// read it: the caller folds it into the `centre` it passes, exactly as the equatorial view
    /// folds its pan into the centre its closures are built on.
    pub pan: Vec2,
    /// M of vertical world distance per M of coordinate time.
    ///
    /// At 1.0 a light ray far from the hole, which covers one M of space per M of t, rises at 45
    /// degrees, and the eye can read "is this steeper than light" straight off the picture. That
    /// is the whole point of drawing time as a length, so it is the default.
    pub t_scale: f64,
}

/// The camera positions the view offers as buttons.
///
/// `Top` is the equatorial view seen from directly above, which is what the other canvas draws;
/// `Side` is almost edge-on, where the floor collapses to a line and the picture is a (space, time)
/// diagram; `EdgeOn` is nearer still, the eye all but in the floor looking along +y; `ThreeQuarter`
/// is the one that shows a cone as a cone.
///
/// `EdgeOn` earns its place by turning the volume back into a (t, x) diagram: the eye almost in
/// the floor, x across the screen and coordinate time up it, so a pipe closes to a narrow band and
/// a worldline is a curve against the two horizons it crosses. Not pitch 0 exactly, because there
/// every surface the line of sight lies in projects to zero area and vanishes from the picture
/// altogether. A few degrees leaves each one a skinny band, which is what the eye needs to find
/// it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Preset {
    Top,
    Side,
    EdgeOn,
    ThreeQuarter,
}

/// The pitch of the `EdgeOn` preset, radians: about four degrees above the floor. Seen in the
/// app, exactly edge-on left the horizons invisible; this leaves each a skinny visible band.
pub const EDGE_ON_PITCH: f32 = 0.07;

impl Preset {
    /// The (yaw, pitch) this preset puts the eye at. `Side` is not exactly edge-on: at pitch 0 the
    /// floor is a single line and every worldline crossing the hole lands on top of every other, so
    /// it is tilted just far enough that near and far are distinguishable. `EdgeOn` is four
    /// degrees off pitch 0 on purpose, for the reason the type's doc gives.
    fn angles(self) -> (f32, f32) {
        match self {
            Self::Top => (0.0, std::f32::consts::FRAC_PI_2),
            Self::Side => (0.0, 0.17),
            Self::EdgeOn => (0.0, EDGE_ON_PITCH),
            Self::ThreeQuarter => (0.52, 0.61),
        }
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0)
    }
}

impl Camera {
    pub fn preset(p: Preset, scale: f32, pan: Vec2, t_scale: f64) -> Self {
        let (yaw, pitch) = p.angles();
        Self { yaw, pitch, scale, pan, t_scale }
    }

    /// Unit view direction d (from the eye into the scene) and the screen basis (right, up), in
    /// world coordinates.
    ///
    /// The three are orthonormal with `right x up = -d`: the frame carried by the eye is
    /// (right, up, towards the eye), the ordinary right-handed screen frame, so d itself points
    /// away from the eye and `p . d` grows with distance from it. At every pitch above the floor
    /// d has a negative z component, which is what makes later times nearer: the volume is drawn
    /// looking down and back along t, with the present at the top of the stack.
    fn basis(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let right = [cy, sy, 0.0];
        let up = [-sy * sp, cy * sp, cp];
        let d = [-sy * cp, cy * cp, -sp];
        (snap3(right), snap3(up), snap3(d))
    }

    /// Where the eye is looking, in world coordinates: the `d` of `basis`.
    ///
    /// It is the one leg of the frame the scene itself needs. `rim_weight` asks how edge-on a
    /// vertical wall is, which is a question about the wall's normal against the line of sight and
    /// about nothing on the screen, so the caller needs `d` and neither of the other two.
    pub fn view_direction(&self) -> [f32; 3] {
        self.basis().2
    }

    /// Screen position and depth (larger = farther from the eye) of a world point.
    ///
    /// `centre` is the screen point world (0, 0, 0) lands on, pan already included.
    pub fn project(&self, centre: Pos2, p: [f64; 3]) -> (Pos2, f32) {
        let (right, up, d) = self.basis();
        // Down to f32 once, before any of the three dot products, so that the Top preset's
        // x * 1.0 + y * 0.0 + z * 0.0 reproduces `x as f32` bit for bit rather than rounding a
        // different f64 sum.
        let v = [p[0] as f32, p[1] as f32, p[2] as f32];
        let x = centre.x + dot(v, right) * self.scale;
        let y = centre.y - dot(v, up) * self.scale;
        (Pos2::new(x, y), dot(v, d))
    }
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn snap3(v: [f32; 3]) -> [f32; 3] {
    [snap(v[0]), snap(v[1]), snap(v[2])]
}

fn snap(c: f32) -> f32 {
    if c.abs() < SNAP {
        0.0
    } else if c >= 1.0 - SNAP {
        1.0
    } else if c <= -(1.0 - SNAP) {
        -1.0
    } else {
        c
    }
}

/// Which side of the floor plane z = 0 a primitive lies on. The eye is above the floor for every
/// pitch > 0, so everything below it paints before the floor and everything above after it; the
/// depth sort orders primitives within a layer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Layer {
    Below,
    Above,
}

/// One thing to paint, already projected to the screen.
///
/// The variants are the shapes this canvas needs and no more. Everything translucent is a `Mesh`,
/// because a pipe wall or a cone flank is shaded per vertex and a `Shape` with one fill colour
/// cannot say that; `Line` is there so that an opaque worldline, a rim or the edge of a cone does
/// not pay for a two-triangle mesh. The opaque markers are not among them: they sit on the floor,
/// which is the sorting plane itself, and are painted after both layers rather than sorted into
/// one.
pub enum Prim {
    /// Per-vertex colours, premultiplied.
    Mesh(egui::Mesh),
    Line { points: Vec<Pos2>, stroke: Stroke, closed: bool },
}

impl Prim {
    fn into_shape(self) -> egui::Shape {
        match self {
            Self::Mesh(mesh) => egui::Shape::mesh(mesh),
            Self::Line { points, stroke, closed } => {
                if closed {
                    egui::Shape::closed_line(points, stroke)
                } else {
                    egui::Shape::line(points, stroke)
                }
            }
        }
    }
}

struct Placed {
    depth: f32,
    prim: Prim,
}

struct Label {
    at: Pos2,
    align: egui::Align2,
    text: String,
    colour: Color32,
}

/// Everything the scene wants painted, held until the whole scene is known.
///
/// A painter's algorithm needs the far end of the scene before it can draw the near end, and the
/// scene is not built in depth order - it is built observer by observer, surface by surface. So
/// the primitives are collected here and sorted once. Labels are kept out of the sort entirely:
/// text is annotation, not geometry, and a caption that disappears behind a horizon it names is a
/// caption that failed.
#[derive(Default)]
pub struct PrimBuffer {
    below: Vec<Placed>,
    above: Vec<Placed>,
    labels: Vec<Label>,
}

impl PrimBuffer {
    pub fn push(&mut self, layer: Layer, depth: f32, prim: Prim) {
        self.slot(layer).push(Placed { depth, prim });
    }

    pub fn label(
        &mut self,
        at: Pos2,
        align: egui::Align2,
        text: impl Into<String>,
        colour: Color32,
    ) {
        self.labels.push(Label { at, align, text: text.into(), colour });
    }

    /// Painter's algorithm: stable sort by depth descending, farthest first, then hand each
    /// primitive to the painter. Drains the layer, so the two layers can be painted either side of
    /// the floor without the second one repainting the first.
    ///
    /// The comparison is `f32::total_cmp`, which is a total order over every bit pattern including
    /// the NaNs, so a depth that came out NaN from a degenerate primitive cannot panic the sort.
    /// It lands at one end of the layer rather than in the middle of it - a positive NaN at the far
    /// end, painted first and buried, a negative NaN at the near end - which is the right outcome
    /// either way: nothing is drawn out of order around it.
    pub fn paint(&mut self, layer: Layer, painter: &egui::Painter) {
        let mut placed = std::mem::take(self.slot(layer));
        placed.sort_by(|a, b| b.depth.total_cmp(&a.depth));
        for p in placed {
            painter.add(p.prim.into_shape());
        }
    }

    /// Paint every label on top of everything else. Consumes the buffer: labels are last, so
    /// there is nothing left to do with it afterwards.
    pub fn paint_labels(self, painter: &egui::Painter, font: egui::FontId) {
        for l in self.labels {
            painter.text(l.at, l.align, l.text, font.clone(), l.colour);
        }
    }

    fn slot(&mut self, layer: Layer) -> &mut Vec<Placed> {
        match layer {
            Layer::Below => &mut self.below,
            Layer::Above => &mut self.above,
        }
    }
}

/// One strip of a pipe: a planar quad in world space as two triangles, one premultiplied colour.
/// Returns the mesh and its depth at the centroid.
///
/// The two triangles share the diagonal 0-2, so the corners have to be given in order around the
/// quad; given them in that order a strip that projects to zero area - which is every vertical
/// strip in the Top view - still produces a mesh egui accepts and tessellates away to nothing.
pub fn quad_mesh(
    cam: &Camera,
    centre: Pos2,
    corners: [[f64; 3]; 4],
    colour: Color32,
) -> (egui::Mesh, f32) {
    let mut mesh = egui::Mesh::default();
    for c in corners {
        mesh.colored_vertex(cam.project(centre, c).0, colour);
    }
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    (mesh, centroid_depth(cam, centre, corners.iter().copied()))
}

/// A cone half: apex plus n rim points as a triangle fan, closing back to rim[0].
///
/// This is the only primitive whose shape carries physics: the rim is where the null geodesics
/// leaving one event have got to after one step of t, so the fan is the cone's surface and not a
/// decoration drawn at a fixed opening angle. Near the horizon the rim is no longer centred on the
/// apex and the fan tips over with it, which is the whole thing the volume view exists to show.
pub fn cone_mesh(
    cam: &Camera,
    centre: Pos2,
    apex: [f64; 3],
    rim: &[[f64; 3]],
    fill: Color32,
) -> (egui::Mesh, f32) {
    let mut mesh = egui::Mesh::default();
    mesh.colored_vertex(cam.project(centre, apex).0, fill);
    for p in rim {
        mesh.colored_vertex(cam.project(centre, *p).0, fill);
    }
    let n = rim.len() as u32;
    for i in 0..n {
        mesh.add_triangle(0, 1 + i, 1 + (i + 1) % n);
    }
    let depth = centroid_depth(cam, centre, std::iter::once(apex).chain(rim.iter().copied()));
    (mesh, depth)
}

/// How far from edge-on a cone's wall may be, as the cosine between its outward normal and the
/// direction to the eye, before the lip along it is drawn at full strength. Below this the lip
/// fades linearly to nothing, so that as the camera orbits, or the cone tips over on a fall, the
/// lip dims out ahead of the wall turning to hide it rather than vanishing in one frame. On an
/// upright 45 degree cone seen from 35 degrees above the floor it is the last thirty degrees or
/// so of azimuth before the far lip goes behind the near wall.
const LIP_FADE_COS: f64 = 0.3;

/// How many levels the lip's strength is quantised to when it is cut into runs: consecutive
/// segments at the same level are one polyline, so a lip that is wholly lit is one loop and a
/// fading one is a handful of arcs rather than thirty-six separate strokes.
const LIP_LEVELS: f64 = 8.0;

/// The Euclidean cross product of two world vectors, which is what the cone's own geometry - the
/// facing of a wall triangle, the base plane, the sightline test - is read off.
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// How visible each point of a cone's rim is, in [0, 1], to an eye looking along `view`.
///
/// Two things can stand between a lip and the eye. The first is the cone's own wall: a cone over a
/// convex rim is a convex solid, so a rim point is hidden exactly when every face meeting it faces
/// away from the eye - the wall triangle either side of it and the open base - and the strength is
/// the cosine between the best of those and the direction to the eye, ramped over `LIP_FADE_COS`.
/// Seen from above, the future half is a cup looked into and its whole lip shows; the past half is
/// a cone looked down onto and its far lip is behind the near wall, and which part is missing is
/// what tells the eye which way the cone faces. The second is the *other* half of the same cone,
/// `other` being its rim about the same apex: from high enough above, the far lip of the past half
/// lies behind the wall of the future half, and a lip point whose line of sight to the eye passes
/// through that solid is hidden. Both are decided per rim point, and the segment between two
/// points takes the mean, so either boundary is a fade across a segment or two and not a step.
fn rim_visibility(apex: [f64; 3], rim: &[[f64; 3]], other: &[[f64; 3]], view: [f32; 3]) -> Vec<f64> {
    let n = rim.len();
    if n < 3 {
        return vec![1.0; n];
    }
    let eye = [-f64::from(view[0]), -f64::from(view[1]), -f64::from(view[2])];
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let unit = |v: [f64; 3]| {
        let len = dot(v, v).sqrt();
        if len > 0.0 { v.map(|c| c / len) } else { v }
    };
    let eye = unit(eye);
    // The mean of a solid's points, against which "outward" is decided; and a base's normal from
    // the rim's own fan about its centre, turned to point away from the apex.
    let solid_centre = |rim: &[[f64; 3]]| {
        let mut c = apex;
        for p in rim {
            for k in 0..3 {
                c[k] += p[k];
            }
        }
        c.map(|v| v / (rim.len() as f64 + 1.0))
    };
    let base_of = |rim: &[[f64; 3]]| -> ([f64; 3], [f64; 3]) {
        let m = rim.len();
        let mut centre = [0.0; 3];
        for p in rim {
            for k in 0..3 {
                centre[k] += p[k] / (m as f64);
            }
        }
        let mut normal = [0.0; 3];
        for i in 0..m {
            let c = cross3(sub(rim[i], centre), sub(rim[(i + 1) % m], centre));
            for k in 0..3 {
                normal[k] += c[k];
            }
        }
        if dot(normal, sub(centre, apex)) < 0.0 {
            normal = normal.map(|c| -c);
        }
        (centre, unit(normal))
    };
    let own_centre = solid_centre(rim);
    let (_, own_base) = base_of(rim);
    let base_facing = dot(own_base, eye);
    // The facing of each wall triangle (i, i + 1): the cosine between its outward normal and the
    // eye.
    let facing: Vec<f64> = (0..n)
        .map(|i| {
            let (a, b) = (rim[i], rim[(i + 1) % n]);
            let mut normal = unit(cross3(sub(a, apex), sub(b, apex)));
            let face_centre = [
                (apex[0] + a[0] + b[0]) / 3.0,
                (apex[1] + a[1] + b[1]) / 3.0,
                (apex[2] + a[2] + b[2]) / 3.0,
            ];
            if dot(normal, sub(face_centre, own_centre)) < 0.0 {
                normal = normal.map(|c| -c);
            }
            dot(normal, eye)
        })
        .collect();
    // Whether the line of sight from a point to the eye passes through the other half's solid:
    // any of its wall triangles, or its base. Möller-Trumbore, forward along the ray only.
    let hits_triangle = |from: [f64; 3], t0: [f64; 3], t1: [f64; 3], t2: [f64; 3]| -> bool {
        let e1 = sub(t1, t0);
        let e2 = sub(t2, t0);
        let p = cross3(eye, e2);
        let det = dot(e1, p);
        if det.abs() < 1e-300 {
            return false;
        }
        let inv = 1.0 / det;
        let s = sub(from, t0);
        let u = dot(s, p) * inv;
        if !(0.0..=1.0).contains(&u) {
            return false;
        }
        let q = cross3(s, e1);
        let v = dot(eye, q) * inv;
        if v < 0.0 || u + v > 1.0 {
            return false;
        }
        dot(e2, q) * inv > 1e-9
    };
    let occluded = |from: [f64; 3]| -> bool {
        let m = other.len();
        if m < 3 {
            return false;
        }
        let (centre, _) = base_of(other);
        (0..m).any(|i| {
            let (a, b) = (other[i], other[(i + 1) % m]);
            hits_triangle(from, apex, a, b) || hits_triangle(from, centre, a, b)
        })
    };
    (0..n)
        .map(|i| {
            // The point meets the wall triangle before it and the one after it, and the base.
            let best = facing[i].max(facing[(i + n - 1) % n]).max(base_facing);
            let own = (best / LIP_FADE_COS).clamp(0.0, 1.0);
            if own <= 0.0 || occluded(rim[i]) { 0.0 } else { own }
        })
        .collect()
}

/// The lip cut into polylines of one strength each: the world points of each run, its strength
/// in (0, 1], and whether it is the whole closed rim. A segment's strength is the mean of its two
/// ends', quantised to `LIP_LEVELS`; segments at zero are not drawn.
fn lip_runs(rim: &[[f64; 3]], visibility: &[f64]) -> Vec<(Vec<[f64; 3]>, f32, bool)> {
    let n = rim.len();
    if n < 2 {
        return Vec::new();
    }
    let level = |i: usize| -> u8 {
        let v = 0.5 * (visibility[i] + visibility[(i + 1) % n]);
        ((v * LIP_LEVELS).ceil().clamp(0.0, LIP_LEVELS)) as u8
    };
    let levels: Vec<u8> = (0..n).map(level).collect();
    let strength = |l: u8| f32::from(l) / LIP_LEVELS as f32;
    if levels.iter().all(|l| *l == levels[0]) {
        return if levels[0] == 0 {
            Vec::new()
        } else {
            vec![(rim.to_vec(), strength(levels[0]), true)]
        };
    }
    // Runs of consecutive segments at one level, started just after a change so that a run
    // wrapping round the seam is one run and not two.
    let start = (0..n).find(|&i| levels[i] != levels[(i + 1) % n]).unwrap_or(0);
    let mut runs: Vec<(Vec<[f64; 3]>, f32, bool)> = Vec::new();
    let mut run: Vec<[f64; 3]> = Vec::new();
    let mut run_level = 0u8;
    for step in 1..=n {
        let i = (start + step) % n;
        if !run.is_empty() && levels[i] != run_level {
            if run_level > 0 {
                runs.push((std::mem::take(&mut run), strength(run_level), false));
            } else {
                run.clear();
            }
        }
        if run.is_empty() {
            run.push(rim[i]);
            run_level = levels[i];
        }
        run.push(rim[(i + 1) % n]);
    }
    if !run.is_empty() && run_level > 0 {
        runs.push((run, strength(run_level), false));
    }
    runs
}


/// The depth of the mean of a set of world points. Depth is linear in the world position, so this
/// is also the mean of the points' depths; it is the one number a whole primitive has to be sorted
/// by, and taking it at the centroid is what makes the sort stable under a reversal of the corner
/// order.
fn centroid_depth(
    cam: &Camera,
    centre: Pos2,
    points: impl Iterator<Item = [f64; 3]>,
) -> f32 {
    let mut sum = [0.0f64; 3];
    let mut n = 0.0f64;
    for p in points {
        sum[0] += p[0];
        sum[1] += p[1];
        sum[2] += p[2];
        n += 1.0;
    }
    if n == 0.0 {
        return 0.0;
    }
    cam.project(centre, [sum[0] / n, sum[1] / n, sum[2] / n]).1
}

/// Fresnel weight of a vertical strip whose outward horizontal normal is `n_xy` against the view
/// direction: (1 - |n . d|)^2, so 1 at the silhouette rim and 0 where the surface faces the eye.
///
/// Real glass reflects most at grazing incidence, and the eye reads that gradient as "this is a
/// closed surface I am seeing through" rather than "this is a flat cyan shape". Without it a
/// horizon pipe is a solid cylinder that hides every worldline inside it; with it the pipe is
/// bright only where its wall is edge-on and nearly clear where it faces the eye, so the infall
/// stays visible through the front wall while the tube still reads as a tube. The square sharpens
/// the rim: a linear falloff leaves the front wall too bright to see through.
pub fn rim_weight(n_xy: (f32, f32), view: [f32; 3]) -> f32 {
    face_weight([n_xy.0, n_xy.1, 0.0], view)
}

/// The same weight for a surface whose normal is not horizontal: (1 - |n_hat . d|)^2 from the
/// Euclidean-unit normal in world axes.
///
/// A pipe wall is vertical and its normal has no z component, which is why `rim_weight` takes two
/// numbers and hands the three-component question on to this. It is the one shading rule in the
/// view, so everything drawn as glass reads as the same kind of glass.
pub fn face_weight(n: [f32; 3], view: [f32; 3]) -> f32 {
    let len = dot(n, n).sqrt();
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // a NaN component has to take this branch too
    if !(len > 0.0) {
        return 0.0;
    }
    let unit = [n[0] / len, n[1] / len, n[2] / len];
    let facing = dot(unit, view).abs().min(1.0);
    (1.0 - facing) * (1.0 - facing)
}

/// The alpha rule for glass: `base_alpha` scaled by 0.15 + 0.85 * weight.
///
/// The floor of 0.15 is there so that a surface facing the eye dead on does not vanish entirely -
/// a horizon that disappears where you are looking straight at it reads as a hole in the picture
/// rather than as glass. The scaling is `gamma_multiply`, which takes all four premultiplied
/// channels down together; scaling the alpha alone would leave the colour brighter than its own
/// alpha, which is not a representable premultiplied colour and paints as a glowing fringe.
pub fn glass(colour: Color32, base_alpha: u8, weight: f32) -> Color32 {
    Color32::from_rgba_unmultiplied(colour.r(), colour.g(), colour.b(), base_alpha)
        .gamma_multiply(0.15 + 0.85 * weight.clamp(0.0, 1.0))
}

/// Where an event (t, r, phi) is drawn in the volume: the one map the whole scene is routed
/// through.
///
/// It is the global Kerr-Schild chart and only that - the equatorial embedding laid out as a floor
/// with coordinate time as height - because that is what this view is a picture of. The chart is a
/// foliation rather than a frame of reference, so the picture it gives is the same for everybody,
/// which is what makes it the right place to read a horizon off: a pipe is where r = const stands,
/// and a cone's lean is the geometry's rather than a choice of observer's.
///
/// A rest-frame version of this view existed and has been taken out. Nothing was wrong with the
/// arithmetic; what was wrong was the claim. A first-order inertial chart is exact only at the
/// observer's own event, so everything at a finite offset - the pipes, the other worldline, the
/// pulse surfaces - was the linearised answer dressed as geometry, and the whole apparatus that
/// kept that honest (a validity radius, a fade, a tangent-plane fallback, a separate zoom ceiling)
/// was machinery for drawing a picture the flat (t, r) diagram already draws properly.
#[derive(Clone, Copy)]
struct Chart {
    /// The simulation clock the floor stands at, so that height is t - t_now.
    t_now: f64,
}

impl Chart {
    /// The world point an event is drawn at.
    fn world(&self, metric: &KerrSchild, t: f64, r: f64, phi: f64, t_scale: f64) -> [f64; 3] {
        let (x, y) = metric.cartesian_position(r, phi);
        [x, y, (t - self.t_now) * t_scale]
    }

    /// One point of the wall of the pipe r = const: the point of that surface at the azimuth
    /// `dphi` whose height in the drawn volume is `z`.
    ///
    /// The section is the circle of the embedding, whose reference azimuth is 0, so the offset is
    /// the azimuth itself and the rows of a pipe are sections at constant height. `None` when the
    /// point cannot be placed at all.
    fn pipe_point(
        &self,
        metric: &KerrSchild,
        r: f64,
        dphi: f64,
        z: f64,
    ) -> Option<[f64; 3]> {
        let (x, y) = metric.cartesian_position(r, dphi);
        let p = [x, y, z];
        finite3(p).then_some(p)
    }

    /// Where a coordinate vector k^mu carried at the event (r, phi) points, as a world direction
    /// scaled so that its vertical component is exactly `t_scale`: one unit of coordinate time per
    /// unit of the parameter, which is what makes `apex + span * direction` a rim at `span` of that
    /// time.
    ///
    /// This is `KerrSchild::cartesian_velocity` of the coordinate slopes, which is the composition
    /// `light_cone_generators` performs. `None` when any component came out non-finite: a primitive
    /// with a non-finite corner is dropped rather than drawn.
    fn direction(
        &self,
        metric: &KerrSchild,
        r: f64,
        phi: f64,
        k: &[f64; 3],
        t_scale: f64,
    ) -> Option<[f64; 3]> {
        let (vx, vy) = metric.cartesian_velocity(r, phi, k[1] / k[0], k[2] / k[0]);
        let d = [vx, vy, t_scale];
        finite3(d).then_some(d)
    }
}

/// Whether a world point can be projected at all. A ray that reached the ring, or an observer whose
/// own (E, L) has no real root where they stand, can hand the scene a coordinate that is not a
/// number; `Camera::project` rounds to f32, so the scene drops such a primitive rather than pushing
/// an infinity into a mesh.
fn finite3(p: [f64; 3]) -> bool {
    p[0].is_finite() && p[1].is_finite() && p[2].is_finite()
}

/// The spacing of the global chart's time rungs for a window `span` M tall: about seven rungs a
/// window on a round step of M, `axis::round_step`'s answer rather than a second copy of the rule.
/// This view's window is fixed at 14 M and the rungs therefore stand 2 M apart, but the rule
/// follows the window, so the day that window is opened to the wheel the rungs follow it too.
///
/// A round step of M rather than the clock ladder the (t, r) diagram's own axis now picks its step
/// from, because these rungs are labelled with absolute readings of the chart's clock and that
/// diagram's are labelled with offsets from now: a ladder rung is round on the reading a *pair* of
/// rungs differ by, which is what an offset counts, and a rung here has to be round on the reading
/// printed against it.
fn time_grid_step(span: f64) -> f64 {
    axis::round_step(span / 7.0)
}

/// The zoom ceiling, in pixels per M.
///
/// It is the equatorial view's, because the two views zoom the same plane at the same rate and a
/// picture of the whole hole has no use for more.
const SCALE_MAX_GLOBAL: f32 = 500_000.0;

/// How far outside the canvas, in pixels, a projected point may fall before it is not drawn.
///
/// At `SCALE_MAX_GLOBAL` the far side of a wide time window is millions of pixels from the middle,
/// and a stroke that long has a squared length past f32 and tessellates to NaN. A point a million
/// pixels off the canvas is off the canvas at every zoom this view will ever be looked at in.
const STAGE_PX: f32 = 1e6;

/// Whether a projected point is finite and not absurdly far off the canvas. See `STAGE_PX`.
fn on_stage(p: Pos2, rect: egui::Rect) -> bool {
    p.is_finite()
        && (p.x - rect.center().x).abs() < STAGE_PX
        && (p.y - rect.center().y).abs() < STAGE_PX
}

/// How many segments a pipe wall, a floor ring and a tick ring are each cut into. Seventy-two is
/// five degrees a segment: at every zoom this view offers, a chord of five degrees departs from the
/// circle it stands for by well under a pixel, and the pipe's shading needs one strip per segment
/// rather than one vertex, so the count is also the mesh budget of every surface in the view.
const RING_SEGMENTS: usize = 72;

/// How many null generators the cone at an observer's event is sampled on. Ten degrees apart, which
/// is enough to show a cone tipping over without making the fan the most expensive thing on screen;
/// the (t, r) diagram draws the same cone from two generators, because a radial chart has only two.
const CONE_SAMPLES: usize = 36;

/// Longest run of trail points in one worldline polyline.
///
/// The worldline is cut into runs because its colour fades with age and a single `Shape::line` has
/// one stroke: the fade has to be carried by the number of pieces. Thirty-two points a piece puts
/// a seam every few M of a trail that holds hundreds of points, which is fine enough that the fade
/// reads as a gradient and coarse enough that a long infall is not a thousand shapes.
const WORLDLINE_RUN: usize = 32;

/// How many null generators the *exact* past cone is integrated on. The same 36 as the local cone,
/// so that the curved surface and the straight fan at its apex are two drawings of one set of null
/// directions rather than two different samplings of it.
const PAST_CONE_RAYS: usize = 36;

/// M of coordinate time between consecutive samples of a past-cone generator.
///
/// It is the spacing of the drawn polyline, not the integrator's step: `NullRay::step_back`
/// substeps each of these intervals by what the ray is actually doing inside it, so this number
/// buys resolution in the picture and not accuracy in the geodesic. At 0.05 M a 10 M window is
/// 200 samples a generator, which draws a surface whose seams are well under a pixel at the zoom
/// the view opens on.
const PAST_CONE_DT: f64 = 0.05;

/// The same two numbers while the focus event is *moving*.
///
/// This is what keeps play smooth, and it buys it with resolution rather than with staleness. A
/// cone rebuilt every 150 ms while the event moves is a cone that jumps: the surface belongs to
/// where the observer was up to a tenth of a second ago, and on a played infall the eye sees the
/// picture lurch rather than flow. So the cone is rebuilt on every frame that moves it and the
/// build is made cheap enough to afford - 24 generators at 0.1 M a sample is a third of the work
/// of the full one, which is a fraction of a millisecond over the view's own window (see
/// `test_the_past_cone_build_is_affordable_every_frame`) - and the frame that stops the event
/// rebuilds once at `PAST_CONE_RAYS` and `PAST_CONE_DT`, where the surface is looked at closely.
const PAST_CONE_RAYS_MOVING: usize = 24;
const PAST_CONE_DT_MOVING: f64 = 0.1;

/// How finely one past cone was integrated, and whether that was the full resolution.
///
/// It travels with the built cone because it is what says whether the cone still has work owing to
/// it: a moving build is a draft, and the first frame that does not move the event replaces it with
/// the full one. Nothing in the drawing reads it - the surface is built from `rays.len()` and from
/// each ray's own samples, so a coarse cone draws by exactly the same code as a fine one.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) struct ConeRes {
    /// M of coordinate time between consecutive samples of a generator.
    dt: f64,
    /// How many null generators the cone is sampled on.
    rays: usize,
    /// Whether this is the full-resolution build, which is not to be rebuilt again.
    full: bool,
}

impl ConeRes {
    const MOVING: Self =
        Self { dt: PAST_CONE_DT_MOVING, rays: PAST_CONE_RAYS_MOVING, full: false };
    pub(crate) const FULL: Self = Self { dt: PAST_CONE_DT, rays: PAST_CONE_RAYS, full: true };
}

/// How many sample rows of the past cone's surface go into one mesh.
///
/// One mesh per quad would put ~7000 primitives through the depth sort for one cone; one mesh for
/// the whole surface would be a single primitive sorted at one depth and would interleave wrongly
/// with the pipes it passes through. Eight rows is the compromise: a few hundred meshes, each
/// short enough that its own centroid is a fair place to sort it.
const PAST_CONE_CHUNK: usize = 8;

/// Opacity of one quad of a tagged pulse's light-cone surface.
///
/// Low, and it has to be: a pulse's surface folds back on itself wherever the front does, the
/// frozen family piles a dozen rows of it into the width of the r- pipe, and two transmissions are
/// drawn at once. At the fronts' own `Theme::SHIFT_ALPHA` the overlap paints solid and buries the
/// worldlines the surface is there to be read against. At 40 a single sheet is a tint and the
/// places where the sheet stacks on itself are exactly the places that read as bright, which is
/// the right thing for it to say: that is where the light is piling up.
const FRONT_SURFACE_ALPHA: u8 = 40;

/// How many rows of one ray pair's strip of pulse surface go into one mesh.
///
/// `PAST_CONE_CHUNK`'s reason, in the same words: one mesh per quad is thousands of primitives
/// through the depth sort, one mesh per strip is a single primitive sorted at one depth that
/// interleaves wrongly with everything it passes through, and eight rows is short enough that its
/// own centroid is a fair place to sort it.
const PULSE_SURFACE_CHUNK: usize = 8;

/// The camera positions the view offers as buttons, and one more that undoes an exploration:
/// Reset puts the zoom, the pan and the time scale back where `Camera::default` has them and
/// leaves the eye where the user has moved it, which is the one part of the view they chose on
/// purpose.
const PRESET_BUTTONS: [(&str, Option<Preset>); 5] = [
    ("Top", Some(Preset::Top)),
    ("Side", Some(Preset::Side)),
    ("Edge", Some(Preset::EdgeOn)),
    ("3/4", Some(Preset::ThreeQuarter)),
    ("Reset", None),
];

/// Where the View Presets box stands, and how its buttons are laid out inside it.
///
/// The buttons sit in a named, outlined box, because five bare words in the corner of a picture
/// read as part of the picture: the frame is what says they are controls, and the heading is what
/// says which. The arithmetic is gathered here rather than spelled out where the box is painted,
/// because it is layout and the painting is drawing.
#[derive(Clone, Copy)]
struct PresetsBox {
    rect: egui::Rect,
    button: Vec2,
    gap: f32,
    pad: f32,
    heading: f32,
}

fn presets_box(rect: egui::Rect, font_scale: f32) -> PresetsBox {
    let button = Vec2::new(40.0 * font_scale, 16.0 * font_scale);
    let gap = 4.0 * font_scale;
    let n = PRESET_BUTTONS.len();
    let strip = button.x * n as f32 + gap * (n - 1) as f32;
    let pad = 6.0 * font_scale;
    let heading = Theme::MIN_FONT_PT * font_scale + 2.0;
    let size = Vec2::new(strip + 2.0 * pad, pad + heading + gap + button.y + pad);
    PresetsBox {
        rect: egui::Rect::from_min_size(rect.right_top() + Vec2::new(-8.0 - size.x, 6.0), size),
        button,
        gap,
        pad,
        heading,
    }
}

/// What a built past cone belongs to. If any of it changes the cone is a cone of a different event
/// in a different spacetime and has to be integrated again; if none of it changes the cached rays
/// are still exactly right, however the camera has been dragged.
#[derive(Clone, PartialEq, Debug)]
struct PastConeKey {
    name: String,
    t: f64,
    r: f64,
    phi: f64,
    m: f64,
    a: f64,
    t_min: f64,
}

/// The exact past light cone of one event: the null geodesics through it, run backwards.
pub(crate) struct PastCone {
    key: PastConeKey,
    /// Per generator, the (t, r, phi) samples of that geodesic: the event itself first, then
    /// earlier and earlier. Kept in coordinates rather than projected, so that a camera move
    /// redraws the same integration.
    rays: Vec<Vec<[f64; 3]>>,
    /// How finely it was integrated, and whether that was the full resolution.
    res: ConeRes,
}

/// Integrate the past light cone of the observer's current event down to `t_min`.
///
/// The past cone of an event *is* the set of null geodesics through it, run the other way:
/// `ray_rhs` is autonomous in t, so `NullRay::step_back` integrates the same curve backwards
/// rather than solving a different problem, and what comes back is the exact locus of everything
/// whose signals reach this event - not a cone drawn at some opening angle. That is the picture
/// the app's central claim needs: as the focus observer closes on the far branch of r-, this
/// surface's footprint at another observer's radius climbs without bound, so every ingoing photon
/// from any later time still gets to them.
///
/// The generators are taken in the *raindrop* frame at the event rather than in the observer's
/// own, for the reason `light_cone_generators` gives: the set of null directions is a property of
/// the event and does not depend on the frame, but where along the rim the 36 samples fall does,
/// and at u^t ~ 1e10 on the approach to r- the observer's own frame aberrates all of them into one
/// point and leaves the rest of the cone undrawn. The raindrop congruence exists at every radius,
/// both horizons included, so the rim is evenly sampled everywhere the view can be looked at.
///
/// A generator that dies is one that came out of the ring: run backwards it reached R_STOP, where
/// the chart's equation is left alone, so its history stops there and its share of the surface
/// simply ends. Its dead state is not recorded, because a dead `NullRay` is carried on the field
/// clock without moving and its (t, r, phi) would claim the ray sat at the ring for the rest of the
/// window. A generator whose past hugs a horizon needs no special case at all: its dr/dt decays to
/// zero and the steps get cheap.
pub(crate) fn build_past_cone(
    metric: &KerrSchild,
    obs: &Observer,
    t_min: f64,
    res: ConeRes,
) -> PastCone {
    let tetrad = Observer::raindrop_tetrad(metric, obs.r);
    let u = tetrad.e0;
    let mut rays = Vec::with_capacity(res.rays);
    for i in 0..res.rays {
        let alpha = std::f64::consts::TAU * (i as f64) / (res.rays as f64);
        let mut ray =
            NullRay::from_local_direction(metric, obs.t, obs.r, obs.phi, &tetrad, alpha, &u);
        let mut samples = vec![[ray.t, ray.r, ray.phi]];
        while ray.alive() && ray.t > t_min {
            ray.step_back(metric, res.dt);
            if !ray.alive() {
                break;
            }
            samples.push([ray.t, ray.r, ray.phi]);
        }
        rays.push(samples);
    }
    PastCone {
        key: PastConeKey {
            name: obs.name.clone(),
            t: obs.t,
            r: obs.r,
            phi: obs.phi,
            m: metric.m,
            a: metric.a,
            t_min,
        },
        rays,
        res,
    }
}

/// The 2D+1 volume: the equatorial plane laid out as a floor and ingoing Kerr-Schild time t drawn
/// as height, with the present at the floor and the past below it.
///
/// It draws the same plane the equatorial canvas draws - same embedding, same fronts, same trails,
/// on the floor - and adds the one axis that view cannot have. What the extra axis buys is the
/// light cone: on the floor a cone is a circle whose centre has drifted off the emitter, and in the
/// volume it is a cone, so "which way can this observer go" is a shape rather than an inference.
/// Surfaces of constant r become vertical pipes, which is what makes r+ and r- read as *places* - a
/// wall a worldline goes through and cannot come back out of - and a worldline frozen on the far
/// branch of r- reads as what it is: a helix wound onto the r- pipe, one turn per 2 pi / Omega_- of
/// t, riding a generator of the surface it can never cross.
pub struct VolumeCanvas {
    pub camera: Camera,
    /// How much coordinate time the volume holds, in M. The window runs from t_now - 0.7 W to
    /// t_now + 0.3 W, the same split the (t, r) diagram uses: most of what there is to look at has
    /// already happened, and the strip of future above the floor is there so that the future half
    /// of a cone has somewhere to point.
    pub time_window: f64,
    /// Slide of that window away from the present, in M. Zero puts the floor at t_now, which is
    /// what "the floor is now" means; step 4 zeroes it on a view reset alongside the (t, r)
    /// diagram's own offset.
    pub time_offset: f64,
    /// The observer the view is being kept centred on, set from the canvas's right-click menu, on
    /// the same terms as the equatorial view's: a view setting, and the more particular of the two
    /// ways of saying where to look.
    pub(crate) centred_on: Option<Who>,
    /// Draw a cone at every whole M the trail crosses, not only at the observer's present event.
    /// Off by default: a dozen translucent fans stacked down a worldline is the picture of how the
    /// cones tip over, and it is also, on a first look at the view, a mess.
    pub(crate) show_ghost_cones: bool,
    /// Draw the exact past light cone of the focus observer's current event: the null geodesics
    /// through it integrated backwards to the bottom of the window, as a surface. On by default,
    /// because it is the one thing this view can show that no other picture in the app can.
    pub(crate) show_past_cone: bool,
    /// That surface, held between frames. Integrating it is the only work in this view that a
    /// camera drag must not repeat, so it is cached against the event it belongs to; see
    /// `PastConeKey` and `ConeRes`.
    past_cone: Option<PastCone>,
    /// Draw the light cone of every tagged pulse as a surface: the ring history the physics keeps
    /// on every `HISTORY_PULSE_STRIDE`-th pulse, swept up in t and coloured by gain. On by default,
    /// because it is the one place in the app where the split between the frozen family and the
    /// crossing family is a shape rather than an inference: the sheet tears in two on r-.
    pub(crate) show_pulse_surfaces: bool,
    /// Where the user has dragged each observer's info box on this canvas.
    pub telemetry: TelemetryBoxes,
    /// The screen offset of the followed observer's floor point as the last frame projected it.
    ///
    /// `look_at` has to answer "what pan puts this floor point in the middle", and the middle is
    /// `rect.center() + pan - offset` where the offset is the followed observer's own projection -
    /// a number that needs the metric and both observers, neither of which a menu item hands it. The equatorial view recomputes it, because there the projection is `zoom` and
    /// a multiply; here it is the whole camera, so the frame that has just done the work leaves it
    /// behind. It is only ever read by `look_at`, and the menu that calls `look_at` is registered
    /// inside the frame that has just written it.
    pub(crate) focus_offset: Vec2,
}

impl Default for VolumeCanvas {
    fn default() -> Self {
        Self {
            camera: Camera::default(),
            time_window: 14.0,
            time_offset: 0.0,
            centred_on: None,
            show_ghost_cones: false,
            show_past_cone: true,
            past_cone: None,
            show_pulse_surfaces: true,
            telemetry: TelemetryBoxes::pinning(),
            focus_offset: Vec2::ZERO,
        }
    }
}

impl VolumeCanvas {
    /// Pan the view so that the point `target` of the equatorial plane - Cartesian, in M, as
    /// `Observer::cartesian_position` gives it - sits in the middle of the canvas, on the floor.
    ///
    /// One pan, not a standing request, exactly as on the equatorial view: whoever is there is free
    /// to move out of the middle again, which is what separates the menu's Goto items from its Keep
    /// Centered ones. A Goto while somebody is being followed pans away from them without letting
    /// go, since the pan is measured from whatever the view is tracking.
    pub fn look_at(&mut self, target: (f64, f64)) {
        let at = self.camera.project(Pos2::ZERO, [target.0, target.1, 0.0]).0;
        self.camera.pan = self.focus_offset - (at - Pos2::ZERO);
    }

    /// Where the view's anchor would project to under a given standing request, in screen pixels:
    /// the offset `render` subtracts to place the canvas, as `focus_offset` records it for the
    /// request that frame was drawn with. Pan-independent, the projection being taken about the
    /// origin.
    fn tracking(
        &self,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        centred: Option<Who>,
    ) -> Vec2 {
        frame_focus(centred, ReferenceFrame::GlobalVolume, bob, alice).map_or(Vec2::ZERO, |obs| {
            let (x, y) = obs.cartesian_position(metric);
            self.camera.project(Pos2::ZERO, [x, y, 0.0]).0 - Pos2::ZERO
        })
    }

    /// Keep `who` in the middle of the view, or stop doing so: this canvas's half of the rule
    /// `SpatialCanvas::re_anchor` states in full, which is where the reasoning lives. Taking hold
    /// brings the new anchor to the middle; letting go keeps the same floor point there so that the
    /// picture does not jump.
    pub(crate) fn hold_observer(
        &mut self,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        who: Who,
        hold: bool,
    ) {
        let want = hold.then_some(who);
        // The frame now drawing has already recorded the offset it was placed with, which is the
        // anchor being let go of; the one being taken up has to be projected.
        let before = self.focus_offset;
        let after = self.tracking(metric, bob, alice, want);
        self.centred_on = want;
        self.camera.pan = if hold { Vec2::ZERO } else { self.camera.pan + after - before };
    }

    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        current_time: f64,
        canvas_height: f32,
        use_physical_units: bool,
        font_scale: f32,
        signals: SignalViews<'_>,
        show_distant_clock_grid: bool,
        style: FrontStyle,
    ) {
        let desired_size = egui::Vec2::new(ui.available_width(), canvas_height);
        let (response, painter) = ui.allocate_painter(desired_size, egui::Sense::click_and_drag());
        let rect = response.rect;
        painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
        if rect.width() < 30.0 || rect.height() < 30.0 {
            return;
        }

        // 1. The camera, moved before anything is projected, so that this frame draws the view the
        // pointer has just asked for rather than the previous one. The equatorial view can afford
        // to defer its pan by a frame because a pan is a translation the eye does not track; an
        // orbit is not, and a view that lags the drag by a frame feels like it is being dragged
        // through treacle.
        //
        // A plain drag pans, as it does on the equatorial view, and shift holds the drag to the
        // orbit: the two canvases are looked at one after the other and the commoner gesture has
        // to mean the same thing on both, which is "move the picture" rather than "move the eye".
        let mut moved = false;
        if response.dragged() {
            let delta = response.drag_delta();
            if delta != Vec2::ZERO {
                if ui.input(|i| i.modifiers.shift) {
                    self.camera.yaw += delta.x * 0.01;
                    self.camera.pitch = (self.camera.pitch - delta.y * 0.01)
                        .clamp(0.05, std::f32::consts::FRAC_PI_2);
                } else {
                    self.camera.pan += delta;
                }
                moved = true;
            }
        }
        if response.hovered() {
            let (zoom_delta, scroll, shift) =
                ui.input(|i| (i.zoom_delta(), i.smooth_scroll_delta, i.modifiers.shift));
            // A shift-held wheel is handed over as horizontal scroll on some platforms and as
            // vertical on others, so whichever axis moved is the notch.
            let wheel = if scroll.y.abs() > 0.1 { scroll.y } else { scroll.x };
            if shift && wheel.abs() > 0.1 {
                // Shift-wheel stretches time against space. It is the one control here with no
                // counterpart on the other canvases: t_scale is the exchange rate between an M of
                // time and an M of length, and at 1 a far-away light ray rises at 45 degrees, so
                // moving it off 1 is moving the picture off the one setting where a slope can be
                // read as a speed.
                let mult = if wheel > 0.0 { 1.05 } else { 1.0 / 1.05 };
                self.camera.t_scale = (self.camera.t_scale * mult).clamp(0.1, 10.0);
                moved = true;
            } else {
                // The same fine step and the same clamp as the equatorial view's wheel, and the
                // same coarse gesture as the (t, r) diagram's: ctrl (or command) held turns the
                // wheel into egui's `zoom_delta`, read here for its direction only, so that one
                // notch of the coarse zoom is exactly `COARSE_ZOOM_STEPS` of the fine one however
                // fast the platform reports the wheel. The zoom is taken about the cursor as the
                // equatorial view takes it, so that the two canvases zoom at one rate and `scale`
                // goes on meaning px/M in both; the cursor is held on the point under it only up to
                // the tilt, the anchor being the nominal centre `rect.center() + pan`, which is
                // where world (0, 0, 0) lands with nobody being followed.
                let (inward, steps) = if (zoom_delta - 1.0).abs() > 1e-4 {
                    (zoom_delta > 1.0, COARSE_ZOOM_STEPS)
                } else if scroll.y.abs() > 0.1 {
                    (scroll.y > 0.0, 1)
                } else {
                    (false, 0)
                };
                if steps > 0 {
                    let step = 0.01875f32;
                    let zoom_mult =
                        if inward { (1.0 + step).powi(steps) } else { (1.0 - step).powi(steps) };
                    let old_scale = self.camera.scale;
                    let new_scale = (old_scale * zoom_mult).clamp(8.0, SCALE_MAX_GLOBAL);
                    if let Some(mpos) = response.hover_pos() {
                        let nominal = rect.center() + self.camera.pan;
                        self.camera.pan += (mpos - nominal) * (1.0 - new_scale / old_scale);
                    }
                    self.camera.scale = new_scale;
                    moved = true;
                }
            }
        }
        if moved {
            ui.ctx().request_repaint();
        }

        // 2. Where the view is anchored, and therefore where world (0, 0, 0) lands. In locals
        // rather than read through `self`, so that the closures below hold no borrow of the canvas:
        // the right-click menu takes `&mut self` while they are still alive.
        let camera = self.camera;
        let t_scale = camera.t_scale;
        // Whom the view is following, which here is only ever the canvas's own "Keep Centered"
        // request: this chart is drawn for nobody, so it names no focus observer of its own.
        // `frame_focus` is the rule the equatorial view follows as well, asked in this chart.
        let focus = frame_focus(self.centred_on, ReferenceFrame::GlobalVolume, bob, alice);
        let chart = Chart { t_now: current_time };
        let offset = focus.map_or(Vec2::ZERO, |obs| {
            let (x, y) = obs.cartesian_position(metric);
            camera.project(Pos2::ZERO, [x, y, 0.0]).0 - Pos2::ZERO
        });
        self.focus_offset = offset;
        let centre = rect.center() + camera.pan - offset;

        // 3. The projection, as the two closures the rest of the frame is written in.
        let project = |p: [f64; 3]| camera.project(centre, p);
        // The floor map is exactly the `Fn((f64, f64)) -> Pos2` the equatorial view's drawing
        // helpers take, which is what lets the fronts, the trails and the arrival ticks be the same
        // code here as there rather than a second implementation that could disagree with it.
        let floor = |(x, y): (f64, f64)| camera.project(centre, [x, y, 0.0]).0;
        // Height is a coordinate-time difference in M, kept in f64 the whole way into `project`,
        // which takes the one rounding to f32 it needs. Near r- the interesting times differ from
        // t_now by parts in 1e7 of t_now itself, and taking the difference in f32 would quantise
        // the whole late fall onto one height.
        let z_of = |t: f64| (t - current_time) * t_scale;
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;
        // The foot of every pipe: the bottom of the time window, as a height in the drawn volume.
        let z_bottom = z_of(t_min);
        let view_d = camera.view_direction();

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();
        // Where every pipe, every floor ring and every rung is cut: `RING_SEGMENTS` uniform
        // segments of the circle, from -pi round to +pi, the two ends being the same point of it.
        //
        // A pipe is parametrised by the chart azimuth rather than by the polar angle of the drawn
        // plane. The two differ only by the constant atan2(a, r) of the embedding, which rotates
        // where the vertices fall on one and the same circle. The floor ring and the rungs are
        // sections of this very partition, so a wall and the ring at its foot cannot disagree about
        // where the surface is.
        let base_step = std::f64::consts::TAU / (RING_SEGMENTS as f64);
        let cuts: Vec<f64> =
            (0..=RING_SEGMENTS).map(|i| -std::f64::consts::PI + base_step * (i as f64)).collect();

        // One section of the surface r = const at height z, as the projected loop through every
        // cut. `None` when a point of it cannot be placed at all, which leaves the surface undrawn
        // at that height rather than half drawn.
        let ring = |r: f64, z: f64| -> Option<Vec<Pos2>> {
            // The last cut is the first one again, so it is not a point of its own.
            cuts[..cuts.len() - 1]
                .iter()
                .map(|&dphi| Some(project(chart.pipe_point(metric, r, dphi, z)?).0))
                .collect()
        };

        let mut buf = PrimBuffer::default();

        // 4. The surfaces of constant r: glass pipes, and a ring on the floor where each crosses
        // it.
        //
        // A pipe is the honest picture of what r = const is: not a circle a worldline happens to
        // cross, but a wall standing in time, so that "Bob went through r+" is a worldline entering
        // a tube and never leaving it. The wall is only drawn below the floor, over the past the
        // simulation has actually integrated; above it there are rings alone, because the future of
        // a horizon is not something this run has computed and a solid wall up there would claim it
        // had.
        let tick_r = if metric.cartesian_radius(rm) > 0.0 { rm } else { rp };
        let mut floor_rings: Vec<(Vec<Pos2>, Stroke)> = Vec::new();
        let surfaces = [
            (0.0, Theme::SINGULARITY_LINE, 70u8, 2.0f32),
            (rm, Theme::HORIZON_CAUCHY, 60, 2.0),
            (rp, Theme::HORIZON_OUTER, 60, 2.5),
            (re, Theme::ERGOSPHERE_LINE, 35, 1.5),
        ];
        for (r, colour, base_alpha, width) in surfaces {
            // A surface of constant r is the circle of Cartesian radius sqrt(r^2 + a^2); r = 0 is
            // the ring, at rho = |a|, and a hole with no spin has no ring and so no pipe there.
            if metric.cartesian_radius(r) <= 0.0 {
                continue;
            }
            for w in cuts.windows(2) {
                let (p0, p1) = (w[0], w[1]);
                // The Fresnel weight wants the wall's outward horizontal normal, which is the
                // radial unit vector of the *drawn* plane at the strip's midpoint - the polar
                // direction of x + iy = (r + ia)e^{i phi}, not of e^{i phi}. `face_weight`
                // normalises, so the position itself is the normal.
                let (nx, ny) = metric.cartesian_position(r, 0.5 * (p0 + p1));
                let weight = rim_weight((nx as f32, ny as f32), view_d);
                let corners: Option<Vec<[f64; 3]>> =
                    [(p0, z_bottom), (p0, 0.0), (p1, 0.0), (p1, z_bottom)]
                        .into_iter()
                        .map(|(dphi, z)| chart.pipe_point(metric, r, dphi, z))
                        .collect();
                let Some(c) = corners else {
                    continue;
                };
                let (mesh, depth) = quad_mesh(
                    &camera,
                    centre,
                    [c[0], c[1], c[2], c[3]],
                    glass(colour, base_alpha, weight),
                );
                buf.push(Layer::Below, depth, Prim::Mesh(mesh));
            }
            if let Some(points) = ring(r, 0.0) {
                floor_rings.push((points, Stroke::new(width, colour)));
            }

            // The distant observer's clock, as rungs on one pipe, labelled with the coordinate
            // time itself - the reading on the chart's clock, not an offset from now - so that as
            // the clock runs the rungs slide down the pipe into the past. Rungs at offsets from now
            // would stand still with the same labels for ever, which in this picture is a clock
            // that appears to have stopped: the (t, r) diagram anchors its grid to now and reads
            // the running clock off the head of its time axis instead, and this view has no such
            // axis to put one on - the pipe and its rungs are the clock. Nothing races here either
            // way, because this window is fixed at 14 M and a rung crosses the floor every 2 M of
            // play. On r-, because that is the pipe a frozen
            // worldline winds up, one turn of helix per 2 pi / Omega_- of t, and the rungs are
            // what that pitch is read against; nothing runs away at r+ in this chart, a faller
            // crosses it at a finite t. The same ladder on all four pipes would be three ladders
            // saying nothing and one saying that. A hole with no spin has no r- pipe, and the
            // rungs go on r+ instead.
            if show_distant_clock_grid && r == tick_r {
                let t_step = time_grid_step(t_max - t_min);
                let first = (t_min / t_step).floor() as i64;
                let last = (t_max / t_step).ceil() as i64;
                for i in first..=last {
                    let t_val = (i as f64) * t_step;
                    if t_val < t_min || t_val > t_max {
                        continue;
                    }
                    let z = z_of(t_val);
                    let layer = if z >= 0.0 { Layer::Above } else { Layer::Below };
                    let depth = project([0.0, 0.0, z]).1;
                    if let Some(points) = ring(r, z) {
                        buf.push(
                            layer,
                            depth,
                            Prim::Line {
                                points,
                                stroke: Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
                                closed: true,
                            },
                        );
                    }
                    // The same labels the (t, r) diagram's axis carries, from the same step, so a
                    // rung on a pipe here and a grid line there read alike and carry the digits
                    // their shared step needs.
                    let text = if use_physical_units {
                        format!("t = {}", axis::time_label_physical(metric, t_val, t_step))
                    } else {
                        format!("t = {}", axis::time_label_m(t_val, t_step))
                    };
                    buf.label(
                        project([metric.cartesian_radius(r), 0.0, z]).0,
                        egui::Align2::LEFT_CENTER,
                        text,
                        Theme::TEXT_MUTED,
                    );
                }
            }
        }

        // 5. The worldlines themselves: the trail with its time given back, running up out of the
        // past to the marker standing on the floor.
        let present: Vec<(&Observer, Who)> = [(bob, Who::Bob), (alice, Who::Alice)]
            .into_iter()
            .filter_map(|(obs, who)| obs.map(|obs| (obs, who)))
            .collect();
        let colour_of = |who: Who| match who {
            Who::Alice => Theme::ALICE_COLOR,
            Who::Bob => Theme::BOB_COLOR,
        };
        let span = (current_time - t_min).max(1e-9);
        for (obs, who) in present.iter().copied() {
            // Heavier than the shadow on the floor: the worldline is seen through the glass of
            // whichever pipes stand between it and the eye, and at the floor's weight it was lost
            // behind them.
            let width = match who {
                Who::Alice => 1.8,
                Who::Bob => 2.2,
            };
            // Thinned to the screen once the world position is known, so the run count below
            // follows the drawn length of the worldline and not the depth of the buffer. The
            // thinning is in screen space but keeps the world point and the time, which the depth
            // sort and the fade still need. See `thin_to_pixels`.
            // The stretch of trail inside the view's time window. A trail is sorted in t, so
            // both ends are `partition_point` questions and the window is a `range` rather than a
            // filter that has to look at every event held to draw the few thousand on screen.
            let lo = obs.trail.partition_point(|p| p.t < t_min);
            let hi = obs.trail.partition_point(|p| p.t <= current_time);
            let points: Vec<(f64, [f64; 3])> = thin_to_pixels(
                obs.trail
                    .range(lo..hi)
                    .map(|p| (p.t, chart.world(metric, p.t, p.r, p.phi, t_scale)))
                    // A trail point that maps to an infinity is dropped rather than drawn: the
                    // rest of the worldline is still the worldline.
                    .filter(|(_, p)| finite3(*p)),
                |(_, at)| project(*at).0,
                SCREEN_SPACING,
            );
            let mut start = 0;
            while start + 1 < points.len() {
                let end = (start + WORLDLINE_RUN).min(points.len());
                let run = &points[start..end];
                // Older is fainter, so the eye reads the worldline's direction off it without an
                // arrowhead: the bright end is the end the observer is at now.
                let t_mid = 0.5 * (run[0].0 + run[run.len() - 1].0);
                let fade = (0.45 + 0.55 * ((t_mid - t_min) / span)).clamp(0.0, 1.0) as f32;
                let depth = centroid_depth(&camera, centre, run.iter().map(|(_, p)| *p));
                let points: Vec<Pos2> = run.iter().map(|(_, p)| project(*p).0).collect();
                // A run the zoom has thrown a million pixels off the canvas is not drawn: its
                // squared length is past f32 and it would tessellate to nothing good.
                if points.iter().all(|p| on_stage(*p, rect)) {
                    buf.push(
                        Layer::Below,
                        depth,
                        Prim::Line {
                            points,
                            stroke: Stroke::new(width, colour_of(who).gamma_multiply(fade)),
                            closed: false,
                        },
                    );
                }
                // One point of overlap, so the runs join instead of leaving a gap at every seam.
                start = end - 1;
            }
        }

        // 6. The cone at each observer's present event, which is the reason this view exists.
        //
        // Its rim is where the null geodesics leaving the event have got to after `cone_span` of
        // coordinate time, sampled from the exact generators rather than drawn at a fixed opening
        // angle, so inside r+ the whole cone leans over until every one of its generators points
        // inward and the picture says outright why there is no way back. The generators are taken
        // in the raindrop frame at the event, which exists at every radius including both horizons,
        // rather than in the observer's own: at u^t ~ 1e10 on the approach to r- an observer's own
        // frame crowds all 36 samples into one point of the rim and leaves the rest undrawn.
        //
        // The span is a fraction of the time window, capped so that the straight generators stay
        // a local statement - and capped again by the zoom, so that the rim is on the canvas. At
        // the top of the zoom range a rim 1.7 M up the observer's own time is a million pixels off
        // the screen, and what is left on it is the inside of the fill, a uniform tint no eye can
        // tell from the background. A quarter of the canvas height keeps the cone a cone at every
        // zoom.
        let on_canvas = f64::from(rect.height()) * 0.25 / (f64::from(camera.scale) * t_scale);
        let cone_span = (self.time_window * 0.12).min(on_canvas).clamp(1e-12, 1.8);
        let push_cone = |buf: &mut PrimBuffer,
                             t_at: f64,
                             r: f64,
                             phi: f64,
                             fills: (Color32, Color32),
                             ghost: bool| {
            let tetrad = Observer::raindrop_tetrad(metric, r);
            let apex = chart.world(metric, t_at, r, phi, t_scale);
            if !finite3(apex) || !on_stage(project(apex).0, rect) {
                return;
            }
            // The rim is the 36 null directions at the event, each carried a fixed span of the
            // chart's own time by the map the rest of the scene is drawn with - which is
            // `light_cone_generators`' own composition, so the cone leans over as the geometry
            // says rather than as a drawing rule says.
            //
            // Where the 36 samples fall on the rim depends on the frame they are spread evenly
            // in: seen from a frame boosted by gamma against it they bunch towards the boost, and
            // at the gamma ~ 1e10 of a worldline frozen on r- an observer's own frame would crowd
            // all 36 onto one point of the rim and leave the rest of the cone undrawn. The
            // raindrop congruence exists at every radius, both horizons included, so its sky is
            // where the rim is sampled evenly everywhere this view can be looked at.
            let mut future: Vec<[f64; 3]> = Vec::with_capacity(CONE_SAMPLES);
            for i in 0..CONE_SAMPLES {
                let alpha = std::f64::consts::TAU * (i as f64) / (CONE_SAMPLES as f64);
                let Some(d) =
                    chart.direction(metric, r, phi, &tetrad.null_direction(alpha), t_scale)
                else {
                    return;
                };
                let p = [
                    apex[0] + cone_span * d[0],
                    apex[1] + cone_span * d[1],
                    apex[2] + cone_span * d[2],
                ];
                if !finite3(p) {
                    return;
                }
                future.push(p);
            }
            let rise = cone_span * t_scale;
            let z0 = apex[2];
            // The past half is the future half reflected through the apex, which is what the past
            // cone of an event is: the same null directions run backwards.
            let past: Vec<[f64; 3]> = future
                .iter()
                .map(|p| [2.0 * apex[0] - p[0], 2.0 * apex[1] - p[1], 2.0 * z0 - p[2]])
                .collect();
            let n = future.len() as f64;
            for (rim, other, fill, mean_z, fixed) in [
                (&future, &past, fills.0, (z0 + n * (z0 + rise)) / (n + 1.0), Layer::Above),
                (&past, &future, fills.1, (z0 + n * (z0 - rise)) / (n + 1.0), Layer::Below),
            ] {
                // A cone at the present event straddles the floor and its two halves are the two
                // sides of it; a ghost further down the trail may be wholly below, so it is placed
                // by where it actually is.
                let layer = if ghost {
                    if mean_z >= 0.0 { Layer::Above } else { Layer::Below }
                } else {
                    fixed
                };
                // The sort stands: a cone inside a pipe really is seen through the pipe's wall.
                let (mesh, depth) = cone_mesh(&camera, centre, apex, rim, fill);
                buf.push(layer, depth, Prim::Mesh(mesh));
                // The rim, as a bright lip in this half's own colour, drawn only where the eye can
                // see it - past the cone's own wall, and past the other half's - and fading out
                // ahead of either boundary rather than stopping at it. See `rim_visibility`.
                let lip = Theme::cone_rim_colour(fill);
                let visibility = rim_visibility(apex, rim, other, view_d);
                for (points, strength, closed) in lip_runs(rim, &visibility) {
                    buf.push(
                        layer,
                        depth,
                        Prim::Line {
                            points: points.iter().map(|p| project(*p).0).collect(),
                            stroke: Stroke::new(1.8, lip.gamma_multiply(strength)),
                            closed,
                        },
                    );
                }
            }
        };
        for (obs, _) in present.iter().copied() {
            let (future_fill, past_fill, _) =
                Theme::cone_colours_at(Who::of(obs), Theme::VOLUME_CONE_FILL_ALPHA);
            push_cone(&mut buf, current_time, obs.r, obs.phi, (future_fill, past_fill), false);
            if self.show_ghost_cones {
                let k_lo = t_min.ceil() as i64;
                let k_hi = current_time.ceil() as i64 - 1;
                // The stretch of trail inside the window, found once rather than filtered for at
                // every whole M. The trail is recorded in order, so it is sorted in t and both ends
                // of the window are `partition_point` questions.
                let lo = obs.trail.partition_point(|p| p.t < t_min);
                let hi = obs.trail.partition_point(|p| p.t < current_time);
                for k in k_lo..=k_hi {
                    let t = k as f64;
                    // The nearest recorded event to that whole M. The trail is what the run
                    // actually integrated, so a ghost stands on a computed event rather than on an
                    // interpolation between two of them; a whole M the trail does not reach within
                    // one M gets no cone rather than one dragged over to it.
                    //
                    let nearest = nearest_recorded(&obs.trail, lo..hi, t);
                    let Some(p) = nearest.filter(|p| (p.t - t).abs() <= 1.0) else {
                        continue;
                    };
                    push_cone(
                        &mut buf,
                        p.t,
                        p.r,
                        p.phi,
                        (future_fill.gamma_multiply(0.4), past_fill.gamma_multiply(0.4)),
                        true,
                    );
                }
            }
        }

        // 7. The exact past light cone of the focus event, as a surface.
        //
        // Section 6 draws a *local* cone at every observer: the null directions at the event, run
        // straight for a fraction of the window, which says which way light can go and nothing
        // about where it has been. This says where it has been. Every generator is integrated
        // backwards to the bottom of the window, so the surface is the true locus of the events
        // whose light reaches the focus observer now - the boundary of everything they can
        // currently see. Near the far branch of r- it is the whole argument in one picture: the
        // surface stops climbing away from r- and instead sweeps up the pipe, so it crosses another
        // observer's worldline at later and later t without bound.
        //
        // The focus is whoever the view is anchored on, falling back to Bob and then Alice. The
        // right-click menu keeps the switch, because the near part of the surface can wash over the
        // geometry around it.
        let cone_focus = self.show_past_cone.then(|| focus.or(bob).or(alice)).flatten();
        if let Some(obs) = cone_focus {
            let key = PastConeKey {
                name: obs.name.clone(),
                t: obs.t,
                r: obs.r,
                phi: obs.phi,
                m: metric.m,
                a: metric.a,
                t_min,
            };
            // The cone follows the event on every frame, and pays for that in resolution rather
            // than in lag. A cone of where the observer was a tenth of a second ago is a cone of
            // the wrong event, and on a played infall the eye reads that as the surface jumping
            // rather than moving; a cone of this event on 24 generators is the right surface,
            // drawn a little coarsely. So a key that has changed builds at `ConeRes::MOVING` and
            // asks for one more frame, and that frame - if the event has stopped, and the key is
            // therefore the same - replaces the draft with the full build and marks it, so a
            // paused view integrates once and a camera drag over it integrates not at all.
            match self.past_cone.as_ref() {
                Some(cone) if cone.key == key && cone.res.full => {}
                Some(cone) if cone.key == key => {
                    self.past_cone = Some(build_past_cone(metric, obs, t_min, ConeRes::FULL));
                }
                _ => {
                    self.past_cone = Some(build_past_cone(metric, obs, t_min, ConeRes::MOVING));
                    ui.ctx().request_repaint();
                }
            }
            if let Some(cone) = &self.past_cone {
                // Half the density of the local cone's fill: this is a surface hundreds of rows
                // deep rather than a single fan, and at the fan's alpha the overlap where it folds
                // back on itself paints solid.
                let (_, past_fill, edge) =
                    Theme::cone_colours_at(Who::of(obs), Theme::VOLUME_CONE_FILL_ALPHA / 2);
                let n = cone.rays.len();
                // Where sample k of generator i is drawn: the event's place in the volume, as
                // everything else is.
                let place = |ray: usize, k: usize| -> [f64; 3] {
                    let s = cone.rays[ray][k];
                    chart.world(metric, s[0], s[1], s[2], t_scale)
                };
                for (i, a) in cone.rays.iter().enumerate() {
                    // Consecutive generators, closing the last back onto the first: the strip
                    // between them is the piece of the cone's surface they bound.
                    let b = &cone.rays[(i + 1) % n];
                    // Only rows that exist on both. Where one of the two died at the ring the
                    // surface simply ends there rather than being stretched to meet its neighbour.
                    let rows = a.len().min(b.len());
                    let mut k0 = 0;
                    while k0 + 1 < rows {
                        let k1 = (k0 + PAST_CONE_CHUNK).min(rows - 1);
                        let mut mesh = egui::Mesh::default();
                        let mut corners: Vec<[f64; 3]> = Vec::with_capacity(4 * (k1 - k0));
                        for k in k0..k1 {
                            let j = (i + 1) % n;
                            let quad = [place(i, k), place(j, k), place(j, k + 1), place(i, k + 1)];
                            // A corner that came out non-finite ends the surface there, exactly as
                            // a generator that died at the ring does.
                            if !quad.iter().all(|c| finite3(*c)) {
                                continue;
                            }
                            let base = mesh.vertices.len() as u32;
                            for c in quad {
                                mesh.colored_vertex(project(c).0, past_fill);
                                corners.push(c);
                            }
                            // The same diagonal split as `quad_mesh`, with the corners given in
                            // order around the quad.
                            mesh.add_triangle(base, base + 1, base + 2);
                            mesh.add_triangle(base, base + 2, base + 3);
                        }
                        if !mesh.is_empty() {
                            let depth = centroid_depth(&camera, centre, corners.into_iter());
                            buf.push(Layer::Below, depth, Prim::Mesh(mesh));
                        }
                        // One row of overlap, so the chunks meet instead of leaving a gap.
                        k0 = k1;
                    }
                }
                // Every fourth generator drawn as a line, so the eye can follow one photon's
                // history across a surface that is otherwise a wash. Nine of them: enough to read
                // the twist frame dragging puts into the cone, few enough not to fill it in.
                for i in (0..n).step_by(4) {
                    let points: Vec<[f64; 3]> = (0..cone.rays[i].len())
                        .map(|k| place(i, k))
                        .filter(|p| finite3(*p))
                        .collect();
                    let mut start = 0;
                    while start + 1 < points.len() {
                        let end = (start + WORLDLINE_RUN).min(points.len());
                        let run = &points[start..end];
                        let depth = centroid_depth(&camera, centre, run.iter().copied());
                        buf.push(
                            Layer::Below,
                            depth,
                            Prim::Line {
                                points: run.iter().map(|p| project(*p).0).collect(),
                                stroke: Stroke::new(0.8, edge),
                                closed: false,
                            },
                        );
                        start = end - 1;
                    }
                }
            }
        }

        // 8. The light cone of every tagged pulse, as a surface.
        //
        // Section 7 draws one observer's past cone; this draws the *future* cones of the emission
        // events, and it draws them from the geodesics the simulation actually integrated rather
        // than from a second integration of its own. Every eighth pulse keeps a `RingHistory` - the
        // sampled front at every `HISTORY_MIN_DT` of coordinate time - and the rows of one history
        // stacked in t are that pulse's light cone. What the picture then shows is the split the
        // wavefront module derives: the arc of each sheet with E - Omega_- L < 0 wraps onto the r-
        // pipe and stops there, running up the gain ramp into blue and violet as it waits, while
        // the rest of the sheet falls straight through and on to the ring. The tear between the two
        // is drawn, not asserted.
        //
        // Colour is the gain, on the same ramp and from the same numbers the equatorial view
        // colours its fronts with, so a sheet's colour here and the front's colour on the floor are
        // one measurement drawn twice. The emitter is told apart by where the surface starts, not
        // by its colour: a gain ramp that meant Alice on one sheet and Bob on another would mean
        // nothing on either.
        //
        // The cost is bounded by the physics module's own caps and by the window: at most the
        // pulse cap over `HISTORY_PULSE_STRIDE` tagged pulses per field - eight at the default cap
        // of `MAX_PULSES` and sixteen at the top of the Wavefronts kept slider - with 24 kept rays
        // each and `HISTORY_MAX_ROWS` = 256 rows, so 8 x 24 x 255 x 2 = 98 k triangles per field at
        // the default and 196 k at the widest, under 400 k for both - the worst case the caps
        // allow, against a typical window holding a hundred rows of two or three tagged pulses.
        //
        if self.show_pulse_surfaces {
            let to_world = |t: f64, s: &RaySample| {
                chart.world(metric, t, f64::from(s.r), f64::from(s.phi), t_scale)
            };
            for field in [signals.bob, signals.alice] {
                for pulse in field.pulses.iter() {
                    let Some(history) = pulse.history() else {
                        continue;
                    };
                    // Rows are stored oldest first, so they are sorted in t and the window is a
                    // suffix: everything older than the bottom of the volume is below the floor's
                    // floor and is not drawn. Where that suffix starts is a `partition_point`, and
                    // a start at the end of the rows means the whole history is below the window.
                    let first = history.rows.partition_point(|row| row.t < t_min);
                    if first == history.rows.len() {
                        continue;
                    }
                    let rows = &history.rows[first..];
                    let n = rows[0].samples.len();
                    if rows.len() < 2 || n < 2 {
                        continue;
                    }
                    for i in 0..n {
                        // Consecutive kept rays, the last closing back onto the first: the front is
                        // a closed curve, so the strip between them is a piece of its surface like
                        // any other.
                        let j = (i + 1) % n;
                        let mut k0 = 0;
                        while k0 + 1 < rows.len() {
                            let k1 = (k0 + PULSE_SURFACE_CHUNK).min(rows.len() - 1);
                            let mut mesh = egui::Mesh::default();
                            let mut corners: Vec<[f64; 3]> = Vec::with_capacity(4 * (k1 - k0));
                            for k in k0..k1 {
                                let (lo, hi) = (&rows[k], &rows[k + 1]);
                                let quad = [
                                    (lo.t, &lo.samples[i]),
                                    (lo.t, &lo.samples[j]),
                                    (hi.t, &hi.samples[j]),
                                    (hi.t, &hi.samples[i]),
                                ];
                                // A NaN radius is a ray that was already dead at that row. The
                                // surface ends where the ray did rather than being stretched over
                                // the event it died at, exactly as the past cone's strips end where
                                // a generator ran out.
                                if quad.iter().any(|(_, s)| s.r.is_nan()) {
                                    continue;
                                }
                                let corner: Vec<[f64; 3]> =
                                    quad.iter().map(|(t, s)| to_world(*t, s)).collect();
                                // Off the end of f32: the cell is not drawn, for the reason the
                                // past cone gives.
                                if !corner.iter().all(|c| finite3(*c)) {
                                    continue;
                                }
                                let base = mesh.vertices.len() as u32;
                                for ((_, s), c) in quad.into_iter().zip(corner) {
                                    mesh.colored_vertex(
                                        project(c).0,
                                        Theme::front_colour(f64::from(s.gain), FRONT_SURFACE_ALPHA),
                                    );
                                    corners.push(c);
                                }
                                // The same diagonal split as `quad_mesh`, with the corners given in
                                // order around the quad.
                                mesh.add_triangle(base, base + 1, base + 2);
                                mesh.add_triangle(base, base + 2, base + 3);
                            }
                            if !mesh.is_empty() {
                                let depth = centroid_depth(&camera, centre, corners.into_iter());
                                // Every row is at t <= now, so the whole surface is below the
                                // floor.
                                buf.push(Layer::Below, depth, Prim::Mesh(mesh));
                            }
                            // One row of overlap, so the chunks meet instead of leaving a gap.
                            k0 = k1;
                        }
                    }
                }
            }
        }

        // The scene is complete, so the painter's algorithm can run: everything below the floor
        // farthest first, then the floor, then everything above it. Nothing is painted before this
        // point, because the scene is built surface by surface and observer by observer rather
        // than in depth order - which is the whole reason `PrimBuffer` holds it.
        buf.paint(Layer::Below, &painter);

        // 9. The floor: the equatorial view's own picture of the present, laid flat in the volume.
        // It is painted straight onto the painter between the two layers rather than through the
        // buffer, because it *is* the sorting plane - the one surface whose place in the order is
        // known without a depth - and because the fronts and the trails come from the equatorial
        // view's helpers, which paint rather than return shapes.
        //
        // The region fills are the sections of the same four tubes at the floor's own height, built
        // from the same `pipe_point`. Each region is the band between two sections rather than a
        // disc laid over the discs outside it, for the reason `spatial_canvas::annulus_mesh` gives:
        // a region's pixel colour is then its own fill over the canvas background, the colour the
        // (t, r) diagram paints the same region in. The band is cut at the same partition the
        // tubes' rings are stroked through, so its edges pass through those very points.
        //
        // The ring is the tube r = 0, at rho = |a|; a hole with no spin has no ring, so the
        // innermost fill is floored at two pixels of drawn radius - which is the chart radius
        // sqrt((2 px)^2 - a^2) where that is real, and r = 0 itself where |a| already exceeds it.
        let px = 2.0 / f64::from(camera.scale);
        let ring_r = (px * px - metric.a * metric.a).max(0.0).sqrt();
        let section = |r: f64, dphi: f64| -> Option<Pos2> {
            Some(project(chart.pipe_point(metric, r, dphi, 0.0)?).0)
        };
        // A fill is a region, so it is the whole band or nothing: an arc closed across its own ends
        // is a lie about which side of the surface one is on.
        let band = |r_in: f64, r_out: f64, fill: Color32| -> Option<egui::Mesh> {
            let mut mesh = egui::Mesh::default();
            for w in cuts.windows(2) {
                let quad = [
                    section(r_in, w[0])?,
                    section(r_out, w[0])?,
                    section(r_out, w[1])?,
                    section(r_in, w[1])?,
                ];
                let base = mesh.vertices.len() as u32;
                for p in quad {
                    mesh.colored_vertex(p, fill);
                }
                mesh.add_triangle(base, base + 1, base + 2);
                mesh.add_triangle(base, base + 2, base + 3);
            }
            (!mesh.is_empty()).then_some(mesh)
        };
        for (r_in, r_out, fill) in [
            (rp, re, Theme::ERGOSPHERE_FILL),
            (rm, rp, Theme::REGION_II_FILL),
            (ring_r, rm, Theme::REGION_III_FILL),
        ] {
            if let Some(mesh) = band(r_in, r_out, fill) {
                painter.add(egui::Shape::mesh(mesh));
            }
        }
        if let Some(points) = ring(ring_r, 0.0) {
            painter.add(egui::Shape::convex_polygon(
                points,
                Theme::SINGULARITY_FILL,
                Stroke::NONE,
            ));
        }
        for (points, stroke) in floor_rings {
            painter.add(egui::Shape::closed_line(points, stroke));
        }
        // The two transmissions, Bob's first and at half the stroke width, exactly as the
        // equatorial view lays them down, so that where the two overlap it is the heavier field
        // that stays legible.
        draw_signal_field(
            &painter,
            metric,
            signals.bob,
            Theme::BOB_COLOR,
            Theme::SECONDARY_FRONT_WIDTH,
            style,
            &floor,
        );
        draw_signal_field(&painter, metric, signals.alice, Theme::ALICE_COLOR, 1.0, style, &floor);
        // Each worldline's shadow on the floor: the same trail the worldline above it is drawn
        // from, with the time thrown away. It is what ties the two pictures together - the curve on
        // the floor is what the equatorial view draws, and the curve above it is that curve given
        // its time back.
        if let Some(al) = alice {
            draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &floor);
        }
        if let Some(b) = bob {
            draw_spatial_trail(&painter, metric, b, Theme::BOB_COLOR, 1.5, &floor);
        }

        buf.paint(Layer::Above, &painter);

        // 10. Every arrival, on the receiver's worldline at the height of the crossing, in the
        // sender's colour: the same pairing the equatorial view draws, lifted off the floor onto
        // the event it happened at. Inside r+ these bunch onto the r- pipe, and in the volume the
        // bunch is legible as a stack up the wall rather than as a knot of triangles on a circle.
        let ticks = |field: &SignalField, sender: Color32| {
            for reception in field.receptions() {
                if reception.t < t_min || reception.t > current_time {
                    continue;
                }
                let world =
                    chart.world(metric, reception.t, reception.r, reception.phi, t_scale);
                if !finite3(world) {
                    continue;
                }
                let at = project(world).0;
                if rect.contains(at) {
                    draw_reception_tick(&painter, at, sender);
                }
            }
        };
        // Each tick sits on the receiver's worldline, so it is drawn only while that receiver is in
        // the simulation: Bob's transmission is received by Alice, and hers by him.
        if alice.is_some() {
            ticks(signals.bob, Theme::BOB_COLOR);
        }
        if bob.is_some() {
            ticks(signals.alice, Theme::ALICE_COLOR);
        }

        // 11. The observers, on the floor, because the floor is now.
        let mut markers: Vec<(Who, Pos2)> = Vec::new();
        for (obs, who) in present.iter().copied() {
            let world = chart.world(metric, current_time, obs.r, obs.phi, t_scale);
            if !finite3(world) {
                continue;
            }
            let at = project(world).0;
            if !on_stage(at, rect) {
                continue;
            }
            painter.circle_filled(at, who.marker_radius(), colour_of(who));
            markers.push((who, at));
            // A worldline frozen on the far branch of r- has not stopped: it is riding the
            // horizon's own null generator, so in the volume it is a helix wound onto the r- pipe
            // while the marker creeps round the ring at Omega_-. The observer's own info box says
            // so, in bold, while it is true; see `spacetime_canvas::telemetry_lines`.
        }
        // The observer the view is holding on to wears a ring, as on the equatorial view, so that a
        // picture which is no longer moving under a falling observer says which one it is holding.
        if let Some(centred) = self.centred_on
            && let Some((_, at)) = markers.iter().copied().find(|(who, _)| *who == centred)
        {
            painter.circle_stroke(
                at,
                centred.marker_radius() + CENTRED_RING_GAP,
                Stroke::new(1.0, colour_of(centred)),
            );
        }

        // 12. The canvas's right-click menu: where to look, what to go on looking at, and where the
        // eye stands. Registered after the markers are drawn and before the telemetry boxes, which
        // take their drags last.
        response.context_menu(|ui| {
            for (label, target) in [
                ("Goto Bob", bob.map(|o| o.cartesian_position(metric))),
                ("Goto Alice", alice.map(|o| o.cartesian_position(metric))),
                // The hole is at the origin of the embedding x + iy = (r + ia)e^{i phi}, which is
                // the centre of the ring rather than a point of the spacetime.
                ("Goto Black Hole", Some((0.0, 0.0))),
            ] {
                if ui.add_enabled(target.is_some(), egui::Button::new(label)).clicked() {
                    if let Some(at) = target {
                        self.look_at(at);
                    }
                    ui.close();
                }
            }
            ui.separator();
            for who in [Who::Bob, Who::Alice] {
                let in_run = match who {
                    Who::Alice => alice.is_some(),
                    Who::Bob => bob.is_some(),
                };
                let mut centred = self.centred_on == Some(who);
                let label = format!("Keep {} Centered", who.name());
                if ui.add_enabled(in_run, egui::Checkbox::new(&mut centred, label)).changed() {
                    self.hold_observer(metric, bob, alice, who, centred);
                    ui.close();
                }
            }
            ui.separator();
            if ui.checkbox(&mut self.show_ghost_cones, "Ghost cones along the trail").changed() {
                ui.close();
            }
            // Both surfaces are drawn through the same map as everything else. They stay switches
            // because the near part of either can wash over the geometry around it.
            if ui
                .checkbox(&mut self.show_past_cone, "Exact past cone of the focus event")
                .changed()
            {
                ui.close();
            }
            if ui
                .checkbox(
                    &mut self.show_pulse_surfaces,
                    "Light-cone surfaces of every 8th pulse",
                )
                .changed()
            {
                ui.close();
            }
            ui.separator();
            for (label, preset) in [
                ("Top", Preset::Top),
                ("Side", Preset::Side),
                ("Edge-on", Preset::EdgeOn),
                ("3/4", Preset::ThreeQuarter),
            ] {
                if ui.button(label).clicked() {
                    self.camera =
                        Camera::preset(preset, self.camera.scale, self.camera.pan, self.camera.t_scale);
                    ui.close();
                }
            }
        });

        // The same camera positions as buttons, because a right-click menu is not discoverable by
        // looking at a picture.
        let legend_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let PresetsBox { rect: box_rect, button: button_size, gap, pad, heading: heading_h } =
            presets_box(rect, font_scale);
        painter.rect_filled(box_rect, 4.0, Theme::PANEL_BG.gamma_multiply(0.85));
        painter.rect_stroke(
            box_rect,
            4.0,
            Stroke::new(1.0, Theme::CHIP_OUTLINE),
            egui::StrokeKind::Inside,
        );
        painter.text(
            box_rect.left_top() + Vec2::new(pad, pad),
            egui::Align2::LEFT_TOP,
            "View Presets",
            legend_font.clone(),
            Theme::TEXT_MUTED,
        );
        let row_top = box_rect.top() + pad + heading_h + gap;
        for (i, (label, preset)) in PRESET_BUTTONS.into_iter().enumerate() {
            let min = Pos2::new(box_rect.left() + pad + (button_size.x + gap) * i as f32, row_top);
            let button = egui::Button::new(
                egui::RichText::new(label).font(legend_font.clone()).color(Theme::TEXT_BRIGHT),
            )
            .frame(false);
            if ui.put(egui::Rect::from_min_size(min, button_size), button).clicked() {
                let default = Camera::default();
                self.camera = match preset {
                    Some(p) => Camera::preset(
                        p,
                        self.camera.scale,
                        self.camera.pan,
                        self.camera.t_scale,
                    ),
                    None => Camera {
                        scale: default.scale,
                        pan: default.pan,
                        t_scale: default.t_scale,
                        ..self.camera
                    },
                };
            }
        }

        // 13. The legend: what the picture is, where the eye is standing, and what the vertical
        // axis means, since a viewer arriving at a 3D diagram has no way to know any of the three.
        painter.text(
            rect.left_top() + Vec2::new(10.0, 6.0),
            egui::Align2::LEFT_TOP,
            "2D+1 Volume (x, y, t)",
            legend_font.clone(),
            Theme::TEXT_BRIGHT,
        );
        // The same line the (t, r) chart carries, for the same reason: see `CHART_BANNER`.
        painter.text(
            Pos2::new(rect.center().x, rect.top() + 6.0),
            egui::Align2::CENTER_TOP,
            CHART_BANNER,
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Color32::WHITE,
        );
        painter.text(
            rect.left_top() + Vec2::new(10.0, 8.0 + Theme::MIN_FONT_PT * font_scale),
            egui::Align2::LEFT_TOP,
            format!(
                "yaw {:.0}°  pitch {:.0}°  {:.0} px/M  t×{:.2}\n\
                 window {:.1} … {:.1} M  (floor = now)\n\
                 drag: pan  shift-drag: orbit  wheel: zoom  ctrl-wheel: coarse zoom  \
                 shift-wheel: time scale  \
                 right-click: menu\n\
                 below the floor: the past · above: the future · pipes: r = const · cones: exact \
                 null generators{}{}",
                camera.yaw.to_degrees(),
                camera.pitch.to_degrees(),
                camera.scale,
                t_scale,
                t_min,
                t_max,
                // Terse, on the end of the line that says what the other shapes are.
                if self.show_past_cone {
                    "\npast cone: the event's null geodesics run backwards"
                } else {
                    ""
                },
                if self.show_pulse_surfaces {
                    "\npulse surfaces: every 8th pulse's light cone, coloured by gain"
                } else {
                    ""
                },
            ),
            legend_font.clone(),
            Theme::TEXT_MUTED,
        );

        buf.paint_labels(&painter, legend_font);

        // 14. Draggable info boxes, registered last so they take the drag instead of the canvas.
        for (obs, who) in present {
            let Some((_, at)) = markers.iter().copied().find(|(w, _)| *w == who) else {
                continue;
            };
            self.telemetry.show(
                ui,
                &painter,
                Canvas::Volume,
                BoxId::Observer(who),
                rect,
                at,
                who.name(),
                colour_of(who),
                obs,
                metric,
                use_physical_units,
                font_scale,
            );
        }
    }
}

/// The recorded event nearest `t` among the trail entries in `range`, or None when that stretch of
/// trail is empty.
///
/// A trail is recorded in order, so it is sorted in t, and on a sorted trail the entry nearest `t`
/// is always one of the two that bracket it. So this is the standard library's `partition_point`
/// for the first entry at or after `t`, and a look either side of that - O(log n) against the O(n)
/// of scanning the trail, and the same answer.
///
/// `range` is the stretch of trail inside the view's time window, which the caller finds once by
/// the same means. Clamping the bracketing index into it before looking either side is what makes
/// the answer the nearest entry *of that stretch* for any `t` at all, rather than only for a `t`
/// inside the window: a `t` past either end of the window returns that end of it, which is what a
/// scan of the stretch would return too. An empty stretch has no nearest entry.
fn nearest_recorded(trail: &VecDeque<TrailPoint>, range: Range<usize>, t: f64) -> Option<&TrailPoint> {
    if range.is_empty() {
        return None;
    }
    let at = trail.partition_point(|point| point.t < t).clamp(range.start, range.end);
    [at.checked_sub(1), Some(at)]
        .into_iter()
        .flatten()
        .filter(|index| range.contains(index))
        .map(|index| &trail[index])
        .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// The nearest recorded event, found by scanning: what `nearest_recorded` has to agree with.
    fn nearest_by_scan(
        trail: &VecDeque<TrailPoint>,
        t_min: f64,
        t_max: f64,
        t: f64,
    ) -> Option<&TrailPoint> {
        trail
            .iter()
            .filter(|p| p.t >= t_min && p.t < t_max)
            .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))
    }

    #[test]
    fn test_the_nearest_recorded_event_is_the_one_a_scan_would_find() {
        // The ghost cones stand on recorded events, one per whole M of the window, and which event
        // that is used to be answered by filtering the whole trail and taking a minimum - per whole
        // M, so O(trail) times O(window). The trail is sorted in t, so it is a binary search and a
        // look either side. This is that refactor pinned: the same answer for every whole M of
        // every window tried, including the windows whose ends fall between recorded events and the
        // ones that exclude the trail altogether.
        let metric = KerrSchild::new(1.0, 0.90);
        let params = crate::physics::observer::WorldlineParams::default();
        let mut obs = Observer::new_with_phi(&metric, "Bob", 0.0, 20.0, 0.0, 0.0, params);
        // An uneven cadence on purpose: a real run's step varies with the play mode and with
        // Distance mode's own refinement, so the trail is not a uniform grid in t.
        let mut dt = 0.37;
        while obs.t < 26.0 {
            obs.step(&metric, obs.t + dt, dt);
            dt = if dt > 0.05 { dt * 0.93 } else { 0.41 };
        }
        let span = obs.trail.back().expect("a trail").t - obs.trail.front().expect("a trail").t;
        println!(
            "{} recorded events spanning {:.2} M, from t = {:.3} to {:.3}",
            obs.trail.len(),
            span,
            obs.trail.front().expect("a trail").t,
            obs.trail.back().expect("a trail").t
        );
        assert!(obs.trail.len() > 50, "the walk should record plenty: {}", obs.trail.len());

        let mut checked = 0;
        for &(t_min, t_max) in &[
            (0.0, 26.0),
            (0.0, 1.0),
            (3.5, 9.5),
            (4.25, 4.75),
            (12.0, 26.0),
            (25.999, 26.0),
            (26.5, 30.0),
            (-5.0, 0.5),
        ] {
            let lo = obs.trail.partition_point(|p| p.t < t_min);
            let hi = obs.trail.partition_point(|p| p.t < t_max);
            // Every whole M the renderer would ask about, plus the half-M offsets in between, which
            // land a target exactly between two recorded events far more often.
            let mut targets: Vec<f64> = Vec::new();
            let mut t = t_min.floor() - 1.0;
            while t <= t_max.ceil() + 1.0 {
                targets.push(t);
                targets.push(t + 0.5);
                t += 1.0;
            }
            for t in targets {
                let fast = nearest_recorded(&obs.trail, lo..hi, t);
                let slow = nearest_by_scan(&obs.trail, t_min, t_max, t);
                assert_eq!(
                    fast.map(|p| (p.t, p.r, p.phi)),
                    slow.map(|p| (p.t, p.r, p.phi)),
                    "window [{t_min}, {t_max}) at t = {t}: binary search {:?} against scan {:?}",
                    fast.map(|p| p.t),
                    slow.map(|p| p.t)
                );
                checked += 1;
            }
        }
        println!("{checked} (window, whole M) pairs agree with the scan");
        assert!(checked > 100, "the sweep should cover plenty of cases: {checked}");
    }

    /// One shape the painter emitted, reduced to what a test about the scene has to ask of it.
    /// Kept as owned data because the `FullOutput` the shapes live in has to be dropped before the
    /// assertions run.
    #[derive(Clone, Debug)]
    enum Painted {
        /// A translucent surface: a pipe strip has four vertices, a cone half its apex plus its 36
        /// rim points. The colour is Some only when every vertex carries the same one, which is
        /// what the pipes and the cones do; a pulse surface is shaded per vertex by the gain, so
        /// it is None there and `colours` is what a test about it has to read.
        Mesh {
            vertices: usize,
            colour: Option<Color32>,
            colours: Vec<Color32>,
            first: Pos2,
            points: Vec<Pos2>,
        },
        /// A filled polygon or a stroked polyline. The fill is `Color32::TRANSPARENT` on a
        /// polyline and the stroke is None on a fill. `closed` tells a rim - a cone's, a ring's -
        /// from an open run of a worldline.
        Path { fill: Color32, stroke: Option<Color32>, points: Vec<Pos2>, closed: bool },
        Circle { fill: Color32, radius: f32, centre: Pos2 },
        /// A run of text, with the screen rectangle the laid-out galley occupies and the colour it
        /// was painted in. The rectangle is where the alignment put it - `Align2::RIGHT_CENTER` at
        /// x puts its right edge at x - which is what a test about *where* a label was hung has to
        /// read, and the colour is what tells a rung of the distant clock from the legend.
        Text { text: String, rect: egui::Rect, colour: Color32 },
        Other,
    }

    /// Every shape of one frame, in paint order, flattened out of the `Shape::Vec` groups egui
    /// wraps a layer's shapes in.
    fn painted(output: &egui::FullOutput) -> Vec<Painted> {
        fn walk(shape: &egui::Shape, out: &mut Vec<Painted>) {
            match shape {
                egui::Shape::Vec(inner) => {
                    for s in inner {
                        walk(s, out);
                    }
                }
                egui::Shape::Mesh(mesh) => {
                    let first = mesh.vertices.first().map_or(Pos2::ZERO, |v| v.pos);
                    let colour = mesh.vertices.first().map(|v| v.color).filter(|c| {
                        mesh.vertices.iter().all(|v| v.color == *c)
                    });
                    let colours = mesh.vertices.iter().map(|v| v.color).collect();
                    let points = mesh.vertices.iter().map(|v| v.pos).collect();
                    out.push(Painted::Mesh {
                        vertices: mesh.vertices.len(),
                        colour,
                        colours,
                        first,
                        points,
                    });
                }
                egui::Shape::Path(path) => {
                    let stroke = match path.stroke.color {
                        egui::epaint::ColorMode::Solid(c) if path.stroke.width > 0.0 => Some(c),
                        _ => None,
                    };
                    out.push(Painted::Path {
                        fill: path.fill,
                        stroke,
                        points: path.points.clone(),
                        closed: path.closed,
                    });
                }
                egui::Shape::Circle(c) => {
                    out.push(Painted::Circle { fill: c.fill, radius: c.radius, centre: c.center })
                }
                egui::Shape::Text(t) => out.push(Painted::Text {
                    text: t.galley.text().to_string(),
                    rect: egui::Rect::from_min_size(t.pos, t.galley.rect.size()),
                    colour: t.fallback_color,
                }),
                _ => out.push(Painted::Other),
            }
        }
        let mut out = Vec::new();
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut out);
        }
        out
    }

    /// The canvas both views are measured in: 800 x 700 of screen, 600 px of canvas, no fonts.
    fn input() -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            ..Default::default()
        }
    }

    /// One real frame of the volume view, at one camera preset, with Bob alone on it.
    fn volume_frame(
        metric: &KerrSchild,
        bob: Option<&Observer>,
        preset: Preset,
        show_distant_clock_grid: bool,
    ) -> Vec<Painted> {
        volume_frame_with(metric, bob, preset, show_distant_clock_grid, false)
    }

    /// The same, with the cones along the trail turned on as the right-click menu turns them on.
    ///
    /// The exact past cone is off in both, and turned on only by the tests that are about it: it
    /// is the one thing in this view whose cost is an integration rather than a projection, and
    /// the tests that sweep every region at both presets draw dozens of frames apiece.
    fn volume_frame_with(
        metric: &KerrSchild,
        bob: Option<&Observer>,
        preset: Preset,
        show_distant_clock_grid: bool,
        ghosts: bool,
    ) -> Vec<Painted> {
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(preset, 48.0, Vec2::ZERO, 1.0),
            show_ghost_cones: ghosts,
            show_past_cone: false,
            ..Default::default()
        };
        volume_frame_on(&mut canvas, metric, bob, show_distant_clock_grid)
    }

    /// One frame drawn into a canvas the caller owns, so that a test can ask what the frame left
    /// behind on it - the past cone's cache being the one piece of scene state this canvas keeps
    /// from one frame to the next.
    fn volume_frame_on(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        show_distant_clock_grid: bool,
    ) -> Vec<Painted> {
        let signal = SignalField::default();
        volume_frame_signals(canvas, metric, bob, show_distant_clock_grid, &signal, &signal)
    }

    /// The same, over two transmissions the caller owns, for the tests that are about what the
    /// signal fields put in the volume rather than about the geometry around them.
    fn volume_frame_signals(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        show_distant_clock_grid: bool,
        alice_field: &SignalField,
        bob_field: &SignalField,
    ) -> Vec<Painted> {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        volume_frame_raw(
            canvas,
            &ctx,
            metric,
            bob,
            show_distant_clock_grid,
            alice_field,
            bob_field,
            input(),
        )
        .0
    }

    /// The frame every other helper is built on: one render into a context the caller owns, driven
    /// by a raw input the caller has filled in.
    ///
    /// The context is a parameter because a drag is not one frame - the button goes down on one and
    /// the pointer moves on the next - and egui carries that state on the context. The rect comes
    /// back with the shapes because a test that wants to predict where something landed has to be
    /// measuring the same rectangle the frame allocated.
    #[allow(clippy::too_many_arguments)]
    fn volume_frame_raw(
        canvas: &mut VolumeCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        show_distant_clock_grid: bool,
        alice_field: &SignalField,
        bob_field: &SignalField,
        raw: egui::RawInput,
    ) -> (Vec<Painted>, egui::Rect) {
        // Every worldline stands at the simulation clock, so the floor is the observer's own now.
        let clock = bob.map_or(0.0, |obs| obs.t);
        let mut rect = egui::Rect::ZERO;
        let output = ctx.clone().run_ui(raw, |ui| {
            // The same size `render` is about to allocate: the width it is given and the canvas
            // height it is passed.
            rect = egui::Rect::from_min_size(Pos2::ZERO, Vec2::new(ui.available_width(), 600.0));
            canvas.render(
                ui,
                metric,
                bob,
                None,
                clock,
                600.0,
                false,
                1.0,
                SignalViews { alice: alice_field, bob: bob_field },
                show_distant_clock_grid,
                FrontStyle { arcs: true, hide_wound: true },
            );
        });
        let shapes = painted(&output);
        output.drop_without_applying_deltas();
        (shapes, rect)
    }

    /// One frame at a clock the caller sets, rather than at the observer's own t: the rungs of the
    /// distant clock are read against the simulation clock, and what they do as it runs is the
    /// subject of a test.
    fn volume_frame_at_clock(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: &Observer,
        clock: f64,
        grid: bool,
    ) -> Vec<Painted> {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let signal = SignalField::default();
        let output = ctx.clone().run_ui(input(), |ui| {
            canvas.render(
                ui,
                metric,
                Some(bob),
                None,
                clock,
                600.0,
                false,
                1.0,
                SignalViews { alice: &signal, bob: &signal },
                grid,
                FrontStyle { arcs: true, hide_wound: true },
            );
        });
        let shapes = painted(&output);
        output.drop_without_applying_deltas();
        shapes
    }

    /// One frame with the exact past cone switched as the right-click menu switches it.
    fn volume_frame_past_cone(
        metric: &KerrSchild,
        bob: &Observer,
        show_past_cone: bool,
    ) -> Vec<Painted> {
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone,
            ..Default::default()
        };
        volume_frame_on(&mut canvas, metric, Some(bob), false)
    }

    /// The same frame from the equatorial view, which the Top preset has to reproduce.
    fn spatial_frame(metric: &KerrSchild, bob: Observer) -> Vec<Painted> {
        use crate::gui::spatial_canvas::SpatialCanvas;
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let signal = SignalField::default();
        let mut details = true;
        let clock = bob.t;
        let mut bob = Some(bob);
        let mut alice: Option<Observer> = None;
        let output = ctx.clone().run_ui(input(), |ui| {
            canvas.render(
                ui,
                metric,
                &mut bob,
                &mut alice,
                clock,
                SignalViews { alice: &signal, bob: &signal },
                600.0,
                false,
                ReferenceFrame::DistantObserver,
                1.0,
                FrontStyle { arcs: true, hide_wound: true },
                &mut details,
            );
        });
        let shapes = painted(&output);
        output.drop_without_applying_deltas();
        shapes
    }

    /// Where Bob's own marker was painted: the filled mint disc at his marker radius.
    fn marker_of(shapes: &[Painted]) -> Pos2 {
        shapes
            .iter()
            .find_map(|s| match s {
                Painted::Circle { fill, radius, centre }
                    if *fill == Theme::BOB_COLOR
                        && (*radius - Who::Bob.marker_radius()).abs() < 1e-6 =>
                {
                    Some(*centre)
                }
                _ => None,
            })
            .expect("Bob's marker is painted")
    }

    /// Everything the frame wrote, as one string.
    fn text_of(shapes: &[Painted]) -> String {
        let mut out = String::new();
        for s in shapes {
            if let Painted::Text { text: t, .. } = s {
                out.push_str(t);
                out.push('\n');
            }
        }
        out
    }

    /// Bob on an ordinary released worldline at radius r, at the azimuth the focus tests need.
    fn bob_at(metric: &KerrSchild, r: f64) -> Observer {
        Observer::new_with_phi(
            metric,
            "Bob",
            0.0,
            r,
            0.0,
            0.7,
            crate::physics::observer::WorldlineParams::new(1.0, 2.2, false),
        )
    }

    #[test]
    fn test_the_top_preset_of_the_volume_view_is_the_equatorial_view() {
        // The volume view earns its floor by being the equatorial view when it is looked at from
        // straight above: same embedding, same pixels per M, same place on the canvas. If the two
        // disagree by so much as a marker's width then one of them is drawing a different plane,
        // and a user switching between them would see the hole jump.
        let metric = KerrSchild::new(1.0, 0.9);
        let flat = spatial_frame(&metric, bob_at(&metric, 4.0));
        let volume = volume_frame(&metric, Some(&bob_at(&metric, 4.0)), Preset::Top, false);
        let (a, b) = (marker_of(&flat), marker_of(&volume));
        println!("Bob's marker: equatorial at {a:?}, volume Top preset at {b:?}");
        assert!(
            a.distance(b) < 0.5,
            "seen from straight above the volume view is the equatorial view, but it put Bob at \
             {b:?} where the equatorial view puts him at {a:?}"
        );
    }

    #[test]
    fn test_the_volume_view_renders_in_every_region_including_the_frozen_worldline() {
        // Region I, the ergosphere, the horizon itself, region II, the Cauchy horizon and region
        // III, plus the worldline that never leaves r-. Every one of them puts the observer, his
        // cone and his pipes somewhere different in the volume, and the one thing they all have to
        // do is come out as a frame rather than as a panic: `raindrop_tetrad` and
        // `light_cone_generators` are asked for a frame at radii where the coordinate r = const
        // surfaces are spacelike and where u^t has run away to 1e10.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let mut cases: Vec<(String, Observer)> = Vec::new();
        for r in [9.0, 3.0, rp, 0.5 * (rp + rm), rm, 0.3] {
            cases.push((format!("r = {r:.3}"), bob_at(&metric, r)));
        }
        cases.push(("frozen on r-".to_string(), Observer::frozen_bob(&metric)));

        for (name, bob) in cases {
            for grid in [false, true] {
                for preset in [Preset::Top, Preset::ThreeQuarter] {
                    let shapes = volume_frame(&metric, Some(&bob), preset, grid);
                    let text = text_of(&shapes);
                    assert!(
                        text.contains("2D+1 Volume"),
                        "{name} at {preset:?} (grid {grid}) drew no volume view"
                    );
                    assert!(
                        text.contains("View Presets"),
                        "{name} at {preset:?}: the preset box is named"
                    );
                }
            }
            // And with a cone at every whole M the trail crossed, which walks the recorded trail
            // and builds a fan from the generators at each of those events: the one path that
            // asks `light_cone_generators` for a frame at a radius the observer has already left.
            let ghosted = text_of(&volume_frame_with(
                &metric,
                Some(&bob),
                Preset::ThreeQuarter,
                false,
                true,
            ));
            assert!(ghosted.contains("2D+1 Volume"), "{name} drew no volume view with ghost cones");
            println!("{name}: drawn at both presets, grid and ghost cones on and off");
        }
    }

    #[test]
    fn test_the_pipes_are_glass_below_the_floor_and_rings_above_it() {
        // The whole point of the layer split: a pipe wall is the past of a surface of constant r,
        // so it belongs under the floor where the past is, and the floor - which is the present,
        // and is the equatorial view's own picture - paints over it. Above the floor a surface of
        // constant r is left as a ring, because nothing up there has been integrated.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let shapes =
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, true);

        let floor = shapes
            .iter()
            .position(|s| matches!(s, Painted::Path { fill, .. } if *fill == Theme::SINGULARITY_FILL))
            .expect("the ring's fill is the innermost of the floor's discs");
        // A pipe strip is the four-cornered mesh; a cone half is the 37-vertex fan, and its future
        // half is deliberately above the floor, so the claim is about the strips.
        let strips: Vec<usize> = shapes
            .iter()
            .enumerate()
            .filter(|(_, s)| matches!(s, Painted::Mesh { vertices: 4, .. }))
            .map(|(i, _)| i)
            .collect();
        assert!(!strips.is_empty(), "the pipes are drawn as strips of glass");
        assert!(
            strips.iter().all(|i| *i < floor),
            "every pipe wall belongs under the floor, but one was painted at shape {:?} against a \
             floor at {floor}",
            strips.iter().rfind(|i| **i > floor)
        );

        let tick = shapes.iter().skip(floor).position(
            |s| matches!(s, Painted::Path { stroke: Some(c), .. } if *c == Theme::GRID_LINE),
        );
        println!(
            "{} pipe strips under the floor at shape {floor}, and a tick ring over it at {:?}",
            strips.len(),
            tick.map(|i| i + floor)
        );
        assert!(
            tick.is_some(),
            "a whole M of the distant observer's clock in the future is a ring over the floor"
        );

        // And the count is the same at every zoom the view offers: four pipes of `RING_SEGMENTS`
        // strips, with no refinement anywhere. The partition is a property of the chart rather
        // than of the camera, so the mesh budget of the whole picture is known in advance.
        for scale in [48.0f32, 1e3, 1e4, 1e5, SCALE_MAX_GLOBAL] {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, scale, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: false,
                ..Default::default()
            };
            let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), false);
            let quads =
                shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count();
            assert_eq!(
                quads,
                4 * RING_SEGMENTS,
                "the global chart draws four pipes of {RING_SEGMENTS} strips at every zoom, but at \
                 {scale} px/M it drew {quads}"
            );
        }
    }

    #[test]
    fn test_the_light_cone_at_the_observers_event_has_a_future_and_a_past_half() {
        // The cone is two fans sharing an apex, and the apex is the event the observer is standing
        // at - so it is the marker, to the pixel. A cone drawn anywhere else is a cone belonging to
        // some other event, and the one thing the volume view is for is reading this observer's
        // future off this observer's position.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let shapes = volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, false);
        let at = marker_of(&shapes);
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at(Some(Who::Bob), Theme::VOLUME_CONE_FILL_ALPHA);

        for (half, fill) in [("future", future_fill), ("past", past_fill)] {
            let found = shapes
                .iter()
                .find_map(|s| match s {
                    Painted::Mesh { vertices, colour: Some(c), first, .. } if *c == fill => {
                        Some((*vertices, *first))
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("the {half} half of Bob's cone is painted"));
            assert_eq!(
                found.0,
                CONE_SAMPLES + 1,
                "the {half} half is the apex plus its {CONE_SAMPLES} null generators"
            );
            assert!(
                found.1.distance(at) < 0.5,
                "the {half} half's apex is Bob's own event, but it is at {:?} where his marker is \
                 at {at:?}",
                found.1
            );
            println!("{half} half: {} vertices, apex at {:?}", found.0, found.1);
        }
    }

    #[test]
    fn test_a_cone_rim_is_lit_in_its_own_halfs_colour_only_where_the_wall_does_not_hide_it() {
        // The lip of each half is stroked in a brighter shade of that half's own wall colour, and
        // only where the eye can see it. From above, the future half is a cup the eye looks into
        // and its whole lip is one closed loop at full strength; the past half is a cone the eye
        // looks down onto, and the far part of its lip lies behind the near wall, so its lip is
        // open arcs - which is what tells the viewer which way the cone faces - and it fades out
        // ahead of the boundary rather than stopping at it. From higher up, over the 45 degree
        // wall, the past lip is clear of its own wall but its far arc is behind the wall of the
        // future half, and that hides it too.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 9.0);
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at(Some(Who::Bob), Theme::VOLUME_CONE_FILL_ALPHA);
        // Every stroke in a lip's hue: (closed, points, strength).
        let lip_of = |shapes: &[Painted], fill: Color32| -> Vec<(bool, usize, f32)> {
            let want = Theme::cone_rim_colour(fill).to_srgba_unmultiplied();
            shapes
                .iter()
                .filter_map(|s| match s {
                    Painted::Path { stroke: Some(c), closed, points, .. } => {
                        let got = c.to_srgba_unmultiplied();
                        let same_hue = (0..3).all(|k| (i32::from(got[k]) - i32::from(want[k])).abs() <= 2);
                        (same_hue && c.a() > 0).then(|| (*closed, points.len(), f32::from(c.a()) / 255.0))
                    }
                    _ => None,
                })
                .collect()
        };
        let drawn = |lip: &[(bool, usize, f32)]| lip.iter().map(|(_, n, _)| n - 1).sum::<usize>();
        // Brighter than the wall on every channel, and opaque.
        for fill in [future_fill, past_fill] {
            let lip = Theme::cone_rim_colour(fill);
            let [r, g, b, _] = fill.to_srgba_unmultiplied();
            assert!(
                lip.r() >= r && lip.g() >= g && lip.b() >= b && lip.a() == 255,
                "the lip {lip:?} is a brighter, opaque shade of the wall {fill:?}"
            );
        }

        let three_quarter = volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, false);
        let future = lip_of(&three_quarter, future_fill);
        let past = lip_of(&three_quarter, past_fill);
        println!("three-quarter: future lip {future:?}, past lip {past:?}");
        assert_eq!(
            future,
            vec![(true, CONE_SAMPLES, 1.0)],
            "looked into from above, the future half shows its whole lip as one closed loop at \
             full strength"
        );
        assert!(!past.is_empty(), "the past half's lip is drawn");
        assert!(past.iter().all(|(closed, _, _)| !closed), "and it is open: {past:?}");
        let segments = drawn(&past);
        assert!(
            segments > CONE_SAMPLES / 2 && segments < CONE_SAMPLES,
            "at a pitch of 35 degrees under a 45 degree wall about a quarter of the lip is \
             hidden, not {} of {CONE_SAMPLES} segments",
            CONE_SAMPLES - segments
        );
        assert!(
            past.iter().any(|(_, _, s)| *s >= 1.0) && past.iter().any(|(_, _, s)| *s < 1.0),
            "the lip is at full strength facing the eye and fades towards the wall: {past:?}"
        );

        // From 60 degrees up the eye is over the past half's own wall and its whole lip would show,
        // but the far arc is behind the future half's wall, which hides it.
        let mut canvas = VolumeCanvas {
            camera: Camera { yaw: 0.52, pitch: 1.05, scale: 48.0, pan: Vec2::ZERO, t_scale: 1.0 },
            show_past_cone: false,
            ..Default::default()
        };
        let high =
            volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let past = lip_of(&high, past_fill);
        println!("from 60 degrees up: past lip {past:?}");
        assert!(!past.is_empty() && past.iter().all(|(closed, _, _)| !closed), "{past:?}");
        let segments = drawn(&past);
        assert!(
            segments < CONE_SAMPLES,
            "the far arc of the past lip is behind the future half's wall: {segments} segments"
        );
        assert_eq!(
            lip_of(&high, future_fill),
            vec![(true, CONE_SAMPLES, 1.0)],
            "and nothing stands between the eye and the future half's lip"
        );
    }


    #[test]
    fn test_the_past_cone_rays_retrace_forward_onto_the_focus_event() {
        // The claim the surface makes is that every point of it is joined to the focus event by a
        // null geodesic. Nothing about the drawing checks that - it would look exactly the same if
        // the backwards integration had quietly wandered onto some other curve - so the test walks
        // each generator back the way `build_past_cone` walked it and then forwards again by the
        // same steps, and asks to be put back on the event it started from. 1e-6 is the tolerance
        // `test_ray_step_back_retraces_the_forward_path` measures the scheme's own asymmetry
        // against in `wavefront`; the two directions do not cut the interval in the same places,
        // and that difference is all that is allowed to be left over.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let t_min = bob.t - 5.0;

        let started = Instant::now();
        let cone = build_past_cone(&metric, &bob, t_min, ConeRes::FULL);
        println!(
            "build_past_cone over {:.1} M at {PAST_CONE_RAYS} rays and dt = {PAST_CONE_DT}: {:?}",
            bob.t - t_min,
            started.elapsed()
        );
        let long = build_past_cone(&metric, &bob, bob.t - 10.0, ConeRes::FULL);
        println!(
            "and over 10 M: {:?} ({} samples on the longest generator)",
            started.elapsed(),
            long.rays.iter().map(Vec::len).max().unwrap_or(0)
        );
        assert_eq!(cone.rays.len(), PAST_CONE_RAYS, "one generator per sampled local direction");

        let tetrad = Observer::raindrop_tetrad(&metric, bob.r);
        let u = tetrad.e0;
        let mut worst = 0.0f64;
        for (i, samples) in cone.rays.iter().enumerate() {
            assert!(
                samples.len() >= 2,
                "generator {i} has no past at all: {} samples over a 5 M window",
                samples.len()
            );
            assert_eq!(
                [samples[0][1], samples[0][2]],
                [bob.r, bob.phi],
                "generator {i} starts at the focus event itself"
            );
            for w in samples.windows(2) {
                assert!(
                    w[1][0] < w[0][0],
                    "generator {i} runs into the past, so its samples are strictly earlier and \
                     earlier - but t went from {} to {}",
                    w[0][0],
                    w[1][0]
                );
            }

            let alpha = std::f64::consts::TAU * (i as f64) / (PAST_CONE_RAYS as f64);
            let mut ray = NullRay::from_local_direction(
                &metric, bob.t, bob.r, bob.phi, &tetrad, alpha, &u,
            );
            let n = samples.len() - 1;
            for _ in 0..n {
                ray.step_back(&metric, PAST_CONE_DT);
            }
            for _ in 0..n {
                ray.step(&metric, PAST_CONE_DT);
            }
            assert!(ray.alive(), "the retrace brings generator {i} back to life: {ray:?}");
            let err = (ray.r - bob.r).abs().max((ray.phi - bob.phi).abs());
            worst = worst.max(err);
            assert!(
                err < 1e-6,
                "generator {i} was run {n} steps back and {n} forward and came home to \
                 (r = {}, phi = {}) instead of the event (r = {}, phi = {})",
                ray.r,
                ray.phi,
                bob.r,
                bob.phi
            );
        }
        println!("worst round trip over the whole cone: {worst:e}");
    }

    #[test]
    fn test_no_past_ray_of_an_event_between_the_horizons_comes_from_below_it() {
        // Inside r+ every future-directed null ray moves inward - that is what makes the region
        // trapped - so run backwards every one of them moves outward, and nothing in the past of an
        // event there sits at a smaller radius than the event. It is the sharpest statement the
        // surface makes about region II, and it is a statement about the integration rather than
        // about the drawing, so it is asked of the samples themselves.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let bob = bob_at(&metric, 0.5 * (rp + rm));
        let cone = build_past_cone(&metric, &bob, bob.t - 5.0, ConeRes::FULL);

        let mut highest = bob.r;
        for (i, samples) in cone.rays.iter().enumerate() {
            for s in samples {
                highest = highest.max(s[1]);
                assert!(
                    s[1] >= bob.r - 1e-9,
                    "generator {i} of an event at r = {} inside r+ = {rp} reached back to \
                     r = {} at t = {}, which would be a ray that climbed out of the trapped \
                     region",
                    bob.r,
                    s[1],
                    s[0]
                );
            }
        }
        println!(
            "event at r = {:.4} between r- = {rm:.4} and r+ = {rp:.4}: its whole past cone lies \
             outward of it, out to r = {highest:.3}",
            bob.r
        );
    }

    #[test]
    fn test_the_frozen_observers_past_cone_climbs_out_of_region_ii() {
        // The frozen worldline's event sits a hair above the far branch of r-, and the surface of
        // everything it can see is not a small cone around it: run backwards the generators leave
        // r- exponentially fast and cross r+ inside the window. That is the other half of the
        // freeze - he is cut off from his own future, not from his past - and it is the reason the
        // exact cone is drawn at all rather than a local fan.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let (rp, rm) = (metric.outer_horizon(), metric.inner_horizon());
        let cone = build_past_cone(&metric, &frozen, frozen.t - 10.0, ConeRes::FULL);
        let highest = cone
            .rays
            .iter()
            .flatten()
            .map(|s| s[1])
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "frozen at r = {:.9} (r- = {rm:.6}), t = {:.2}: his past cone reaches out to \
             r = {highest:.3} against r+ = {rp:.4}",
            frozen.r, frozen.t
        );
        assert!(
            highest > rp,
            "the past of an event on r- is not confined to region II - its generators climb out \
             through r+ = {rp} - but the cone got no further than r = {highest}"
        );
    }

    #[test]
    fn test_the_past_cone_is_rebuilt_only_when_the_event_moves() {
        // The surface is the only thing in this view that costs an integration rather than a
        // projection, so it is cached against the event it belongs to: once it has settled at full
        // resolution, a camera drag, a resize or a paused frame must redraw the same rays rather
        // than integrate them again. When the event does move the cache has to let go, or the
        // picture would be the past cone of where the observer used to be.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 4.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            ..Default::default()
        };

        // The first frame has no cone to compare against, so it builds the moving draft; the
        // second finds the key unchanged and settles it at full resolution.
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let draft = canvas.past_cone.as_ref().expect("the first frame builds the cone").res;
        assert_eq!(draft, ConeRes::MOVING, "and builds it as the moving draft");
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let settled = canvas.past_cone.as_ref().expect("and the second settles it");
        assert_eq!(settled.res, ConeRes::FULL, "at the full resolution, the event having stopped");
        // Whether a third frame integrated anything is a question about identity, not about
        // contents: two builds of the same event agree ray for ray. The address of the outer
        // vector answers it - a rebuild allocates its own while this one is still alive - and no
        // wall clock has to be consulted to ask.
        let settled = settled.rays.as_ptr();
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let cone = canvas.past_cone.as_ref().expect("and the third keeps it");
        assert!(
            std::ptr::eq(cone.rays.as_ptr(), settled),
            "nothing about the event changed, so the third frame must have drawn the very rays              the second integrated"
        );

        // Move him: the cone follows on that frame, at the moving resolution and with no throttle
        // between the step and the picture.
        bob.step(&metric, bob.t + 0.5, 0.5);
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let cone = canvas.past_cone.as_ref().expect("and the fourth rebuilds it");
        println!(
            "Bob stepped to t = {:.3}, r = {:.4}; the cone's key followed to t = {:.3}, r = {:.4}              at {} generators",
            bob.t,
            bob.r,
            cone.key.t,
            cone.key.r,
            cone.rays.len()
        );
        assert_eq!(
            cone.res,
            ConeRes::MOVING,
            "the event moved, so the cone belongs to the new one and was integrated again as a              draft"
        );
        assert_eq!(cone.key.t, bob.t, "and it is the cone of the event Bob is standing on now");
        assert_eq!(cone.key.r, bob.r);
    }

    #[test]
    fn test_the_past_cone_follows_the_event_every_frame() {
        // The cone used to be rebuilt at most once per 150 ms while the event moved, which meant
        // that during play the surface on the screen was the past cone of where Bob was up to a
        // tenth of a second ago: it lagged, and then caught up in a jump. It now follows him on
        // every frame that moves him, and buys that with resolution rather than with staleness -
        // the moving build is the draft, and the first frame that does not move him replaces it
        // with the full one.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            ..Default::default()
        };
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);

        // One step of the simulation, taken exactly as `Observer::frozen_bob` takes its own.
        bob.step(&metric, bob.t + 0.25, 0.25);
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let cone = canvas.past_cone.as_ref().expect("the moved event has a cone of its own");
        println!(
            "Bob stepped to t = {:.3}; the cone's key is t = {:.3} on {} generators at dt = {}",
            bob.t,
            cone.key.t,
            cone.rays.len(),
            cone.res.dt
        );
        assert_eq!(
            cone.key.t, bob.t,
            "the frame that moved the event drew the cone of that event, with nothing held back              by a throttle"
        );
        assert_eq!(cone.res, ConeRes::MOVING, "at the moving resolution, which is what pays for it");
        assert_eq!(cone.rays.len(), PAST_CONE_RAYS_MOVING, "and it really is that many generators");

        // Nothing moves: the next frame settles the draft at the full resolution and marks it, and
        // the resolution is the only thing that changes - it is still the same event's cone.
        volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let cone = canvas.past_cone.as_ref().expect("and the event that stopped keeps it");
        assert_eq!(cone.res, ConeRes::FULL, "the event stopped, so the cone is rebuilt in full");
        assert_eq!(cone.rays.len(), PAST_CONE_RAYS);
        assert_eq!(cone.key.t, bob.t, "of the same event");
    }

    #[test]
    fn test_the_past_cone_build_is_affordable_every_frame() {
        // What the throttle used to buy, and what now has to be bought by the resolution instead:
        // a build that fits inside a frame. The window is the view's own 14 M, the event is an
        // ordinary one well outside r+, and the number that matters is the moving one, because
        // that is the build a played infall does sixty times a second.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let t_min = bob.t - VolumeCanvas::default().time_window;

        // The best of three, as a timing test has to be: the suite runs its tests in parallel, and
        // a build that takes 3 ms on its own can lose a scheduling slice to a neighbour and read
        // three times that. What is being bounded is the cost of the build, not the luck of the
        // draw.
        let mut moving = build_past_cone(&metric, &bob, t_min, ConeRes::MOVING);
        let mut moving_took = Duration::MAX;
        for _ in 0..3 {
            let started = Instant::now();
            moving = build_past_cone(&metric, &bob, t_min, ConeRes::MOVING);
            moving_took = moving_took.min(started.elapsed());
        }
        let started = Instant::now();
        let full = build_past_cone(&metric, &bob, t_min, ConeRes::FULL);
        let full_took = started.elapsed();
        println!(
            "a 14 M past cone at r = 3, a = 0.9: moving ({} rays, dt = {}) {moving_took:?},              {} samples on its longest generator; full ({} rays, dt = {}) {full_took:?}, {}              samples",
            ConeRes::MOVING.rays,
            ConeRes::MOVING.dt,
            moving.rays.iter().map(Vec::len).max().unwrap_or(0),
            ConeRes::FULL.rays,
            ConeRes::FULL.dt,
            full.rays.iter().map(Vec::len).max().unwrap_or(0),
        );
        assert!(
            moving_took < Duration::from_millis(5),
            "the moving build has to fit inside a frame with the rest of the scene, but it took              {moving_took:?}"
        );
    }

    #[test]
    fn test_the_focus_observers_past_cone_is_drawn_as_a_surface_below_the_floor() {
        // The whole surface is in the past, so every piece of it belongs under the floor, which is
        // the present: a chunk painted over the floor would be a claim that some of what Bob can
        // already see has not happened yet. And it is drawn only when it is asked for, since it is
        // the most expensive thing in the scene.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let fill = Theme::cone_colours_at(Some(Who::Bob), Theme::VOLUME_CONE_FILL_ALPHA / 2).1;

        let shapes = volume_frame_past_cone(&metric, &bob, true);
        let floor = shapes
            .iter()
            .position(
                |s| matches!(s, Painted::Path { fill, .. } if *fill == Theme::SINGULARITY_FILL),
            )
            .expect("the ring's fill is the innermost of the floor's discs");
        let surface: Vec<usize> = shapes
            .iter()
            .enumerate()
            .filter(|(_, s)| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == fill))
            .map(|(i, _)| i)
            .collect();
        println!(
            "{} chunks of past-cone surface, the last at shape {:?}, against a floor at {floor}",
            surface.len(),
            surface.last()
        );
        assert!(
            !surface.is_empty(),
            "the focus observer's past cone is painted as a surface in his own past fill {fill:?}"
        );
        assert!(
            surface.iter().all(|i| *i < floor),
            "every piece of the past cone is under the floor, but one was painted at shape {:?}",
            surface.iter().rfind(|i| **i > floor)
        );

        let off = volume_frame_past_cone(&metric, &bob, false);
        assert!(
            !off.iter().any(|s| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == fill)),
            "and with the past cone switched off nothing is drawn in that fill"
        );
    }

    #[test]
    fn test_a_frozen_observer_is_labelled_frozen_in_the_volume() {
        // In the volume a frozen worldline is a helix wound onto the r- pipe at a steady pitch,
        // which is a picture that could equally be read as an observer in a tidy orbit. He is not
        // orbiting - his radius and his own clock have stopped, and he is being carried along the
        // horizon's own null generator at Omega_- - so the marker says so, and only when it is true.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let text =
            text_of(&volume_frame(&metric, Some(&frozen), Preset::ThreeQuarter, false));
        println!("frozen at r = {:.9}, t = {:.3}", frozen.r, frozen.t);
        assert!(
            text.contains("Frozen"),
            "a worldline riding the r- generator says so in the volume: {text}"
        );

        let falling = bob_at(&metric, 3.0);
        let text =
            text_of(&volume_frame(&metric, Some(&falling), Preset::ThreeQuarter, false));
        assert!(
            !text.contains("Frozen"),
            "and a Bob who is still falling is not labelled frozen: {text}"
        );
    }

    #[test]
    fn test_the_volume_follows_the_observer_it_is_asked_to_keep_centred() {
        // The canvas draws one chart for everybody, so the only thing that moves the anchor off
        // the hole is the right-click menu's standing "Keep Centered" request - exactly as on the
        // equatorial view. Asked for, the pan is measured from that observer's floor point and
        // their marker is the middle of the canvas whatever the camera is doing; with nobody
        // centred the anchor is the hole, and an observer at r = 4 is nowhere near the middle.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 4.0);
        let global = volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, false);
        // With nobody followed and no pan, world (0, 0, 0) lands on the middle of the canvas, and
        // the ring's fill is the 72-gon drawn about it: its own centre is that middle. So the test
        // finds the middle in the picture rather than being told the layout.
        let hole = global
            .iter()
            .find_map(|s| match s {
                Painted::Path { fill, points, .. } if *fill == Theme::SINGULARITY_FILL => {
                    let n = points.len() as f32;
                    Some(Pos2::new(
                        points.iter().map(|p| p.x).sum::<f32>() / n,
                        points.iter().map(|p| p.y).sum::<f32>() / n,
                    ))
                }
                _ => None,
            })
            .expect("the ring's fill is drawn about the middle of the canvas");
        let free = marker_of(&global);
        assert!(
            free.distance(hole) > 1.0,
            "in the global foliation the view is anchored on the hole, so Bob at r = 4 is not in \
             the middle - but his marker is at {free:?} against a middle of {hole:?}"
        );

        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            centred_on: Some(Who::Bob),
            show_past_cone: false,
            ..Default::default()
        };
        let followed = volume_frame_on(&mut canvas, &metric, Some(&bob), false);
        let held = marker_of(&followed);
        println!("Bob's marker: {free:?} unfollowed, {held:?} followed, middle {hole:?}");
        assert!(
            held.distance(hole) < 0.5,
            "kept centred, the view is anchored on him, so his marker is the middle of the \
             canvas - but it is at {held:?} against a middle of {hole:?}"
        );
    }

    #[test]
    fn test_the_global_clock_rungs_read_coordinate_time_and_slide_into_the_past() {
        // Seen in the app: the rungs on the r- pipe were labelled as offsets from now, so they
        // stood still on the screen with the same labels for ever while the clock ran - a time
        // axis that had stopped. They read the chart's own time now, at the (t, r) diagram's
        // spacing, and as the clock advances a given reading slides down into the past.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 4.0);
        let labels = |clock: f64| {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: false,
                ..Default::default()
            };
            let shapes = volume_frame_at_clock(
                &mut canvas,
                &metric,
                &bob,
                clock,
                true,
            );
            shapes
                .iter()
                .filter_map(|s| match s {
                    Painted::Text { text, rect, colour }
                        if *colour == Theme::TEXT_MUTED && text.starts_with("t = ") =>
                    {
                        Some((text.clone(), rect.center().y))
                    }
                    _ => None,
                })
                .collect::<Vec<_>>()
        };
        let before = labels(20.0);
        let after = labels(20.7);
        println!("rungs at t = 20: {before:?}\nrungs at t = 20.7: {after:?}");
        assert!(before.len() >= 4, "the window is ruled by several rungs: {before:?}");
        let step = time_grid_step(14.0);
        assert!(
            before.iter().all(|(t, _)| t == "t = +0M" || t.ends_with('M')),
            "rungs read the chart's time in M: {before:?}"
        );
        // Every reading is a whole multiple of the step, and consecutive readings are one step
        // apart, as on the (t, r) diagram's axis.
        let mut values: Vec<i64> = before
            .iter()
            .map(|(t, _)| t.trim_start_matches("t = ").trim_end_matches('M').parse::<i64>().unwrap())
            .collect();
        values.sort_unstable();
        assert!(
            values.windows(2).all(|w| (w[1] - w[0]) as f64 == step),
            "consecutive rungs are {step} M apart, got {values:?}"
        );
        // And a rung that is on both frames has moved down the screen by the clock's advance.
        let moved = before.iter().find_map(|(t, y0)| {
            after.iter().find(|(u, _)| u == t).map(|(_, y1)| (t.clone(), *y0, *y1))
        });
        let (t, y0, y1) = moved.expect("a rung survives 0.7 M of the clock");
        println!("{t} moved from y = {y0} to y = {y1}");
        assert!(
            y1 > y0 + 10.0,
            "as the clock runs a fixed reading slides into the past, down the screen: {t} went \
             from {y0} to {y1}"
        );
    }

    const CENTRE: Pos2 = Pos2::new(400.0, 300.0);

    fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    /// Every shape the painter emitted this frame, flattened out of the `Shape::Vec` groups egui
    /// wraps a layer's shapes in, in paint order.
    fn flatten(output: &egui::FullOutput) -> Vec<&egui::Shape> {
        fn walk<'a>(shape: &'a egui::Shape, out: &mut Vec<&'a egui::Shape>) {
            match shape {
                egui::Shape::Vec(inner) => {
                    for s in inner {
                        walk(s, out);
                    }
                }
                other => out.push(other),
            }
        }
        let mut out = Vec::new();
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut out);
        }
        out
    }

    #[test]
    fn test_the_top_preset_projects_the_floor_exactly_as_the_equatorial_view() {
        let cam = Camera::preset(Preset::Top, 48.0, Vec2::ZERO, 1.0);
        for i in -10..=10 {
            for j in -10..=10 {
                let x = i as f64 * 0.5;
                let y = j as f64 * 0.5;
                let expected = CENTRE + Vec2::new(x as f32 * 48.0, -(y as f32) * 48.0);
                for z in [0.0, 3.0, -2.0] {
                    let (at, _) = cam.project(CENTRE, [x, y, z]);
                    assert_eq!(
                        at.x, expected.x,
                        "top view put x = {x} at screen x {} where the equatorial view puts it at \
                         {} (z = {z})",
                        at.x, expected.x
                    );
                    assert_eq!(
                        at.y, expected.y,
                        "top view put y = {y} at screen y {} where the equatorial view puts it at \
                         {} (z = {z})",
                        at.y, expected.y
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_edge_on_preset_lays_the_floor_flat_and_stands_time_up_the_screen() {
        // The reason the preset exists: an eye almost in the floor, looking along +y, so that x
        // runs across the screen and coordinate time runs up it and the whole volume reads as the
        // (t, x) diagram the floor is a plan of. Not pitch 0 exactly: seen in the app, that made
        // every surface tangent to the line of sight vanish - zero-area cells, a trace that is a
        // point - so the preset stands a few degrees off, and the floor keeps a band of depth.
        let cam = Camera::preset(Preset::EdgeOn, 48.0, Vec2::ZERO, 1.0);
        let (right, up, d) = cam.basis();
        let (sp, cp) = EDGE_ON_PITCH.sin_cos();
        assert!(sp > 0.0 && EDGE_ON_PITCH < 0.15, "a few degrees off the floor, not in it");
        for (name, got, want) in [("d", d, [0.0, cp, -sp]), ("up", up, [0.0, sp, cp])] {
            for k in 0..3 {
                assert!(
                    (got[k] - want[k]).abs() < 1e-6,
                    "{name} should be {want:?} at the edge-on pitch, got {got:?}"
                );
            }
        }
        assert_eq!(right, [1.0, 0.0, 0.0], "and x across the screen");

        // One M of time is a whole M up the screen, and one M of depth into the picture - along
        // the line of sight, world +y - is only sin(pitch) of one.
        let origin = cam.project(CENTRE, [0.0, 0.0, 0.0]).0;
        let up_one = cam.project(CENTRE, [0.0, 0.0, 1.0]).0;
        let into_one = cam.project(CENTRE, [0.0, 1.0, 0.0]).0;
        println!(
            "at the edge-on pitch 1 M of t rises {:.1} px and 1 M of depth rises {:.1} px",
            origin.y - up_one.y,
            origin.y - into_one.y
        );
        assert!(
            (origin.y - up_one.y - 48.0 * cp).abs() < 0.5,
            "a whole M of time is a whole M up the screen: {up_one:?} against {origin:?}"
        );
        let depth = (origin.y - into_one.y).abs();
        assert!(depth > 1.0, "the floor keeps a band of depth, not {depth} px");
        assert!(
            depth < 0.15 * 48.0,
            "and a narrow one: 1 M of depth rose {depth} px at 48 px/M"
        );
    }

    #[test]
    fn test_the_cone_stays_on_the_canvas_at_a_deep_zoom() {
        // Seen in the app: zoomed in hard on a worldline gliding along r-, a cone whose rim was a
        // fixed 1.7 M of coordinate time away was a million pixels off the canvas - the whole
        // picture was the inside of its fill, and the cone could not be seen at all. The span is
        // capped by the zoom, so both halves' rims sit inside the canvas at whatever scale the
        // wheel has been run to.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let mut canvas = VolumeCanvas {
            camera: Camera {
                scale: 20_000.0,
                ..Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0)
            },
            // Held on him, or at that zoom his own event is twenty thousand pixels off the canvas
            // and the question the test is asking does not arise.
            centred_on: Some(Who::Bob),
            show_past_cone: false,
            show_pulse_surfaces: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&frozen), false);
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at(Some(Who::Bob), Theme::VOLUME_CONE_FILL_ALPHA);
        let canvas_rect = egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0));
        for (half, fill) in [("future", future_fill), ("past", past_fill)] {
            let points = shapes
                .iter()
                .find_map(|s| match s {
                    Painted::Mesh { colour: Some(c), points, vertices, .. }
                        if *c == fill && *vertices == CONE_SAMPLES + 1 =>
                    {
                        Some(points.clone())
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("the {half} half of Bob's cone is painted"));
            let outside = points.iter().filter(|p| !canvas_rect.contains(**p)).count();
            assert_eq!(
                outside, 0,
                "the {half} half's rim should be on the canvas at {} px/M, but {outside} of its \
                 vertices are off it",
                canvas.camera.scale
            );
            let radius = points[1..].iter().map(|p| p.distance(points[0])).fold(0.0f32, f32::max);
            assert!(
                radius > 20.0,
                "and big enough to see: the {half} rim reaches only {radius} px from the apex"
            );
            // And a rim, not a sliver. The generators are sampled in the raindrop congruence,
            // which exists at this radius and is not boosted away from the chart, so the 36
            // samples spread round the whole circle rather than aberrating onto one point of it.
            // The rim's own extent is much smaller than its distance from the apex, and that is
            // the picture rather than a fault: on r- the cone has leant over so far that the
            // whole of it points one way.
            let rim = egui::Rect::from_points(&points[1..]);
            println!(
                "{half} rim: radius {radius:.0} px, extent {:.0} x {:.0} px",
                rim.width(),
                rim.height()
            );
            assert!(
                rim.width() > 20.0 && rim.height() > 20.0,
                "the {half} rim should spread round the cone, but its extent is {:.0} x {:.0} px",
                rim.width(),
                rim.height()
            );
        }
    }

    #[test]
    fn test_the_side_view_puts_later_times_up_the_screen() {
        // Edge-on exactly, as the `EdgeOn` preset is: the point is that up the screen is +t there.
        let edge = Camera { yaw: 0.0, pitch: 0.0, scale: 48.0, pan: Vec2::ZERO, t_scale: 1.0 };
        let (at, _) = edge.project(CENTRE, [0.0, 0.0, 1.0]);
        assert!(
            (at.y - (CENTRE.y - 48.0)).abs() < 1e-4,
            "edge-on, one M of z should rise one scale up the screen, to y {}, but landed at {}",
            CENTRE.y - 48.0,
            at.y
        );

        let far = edge.project(CENTRE, [0.0, 1.0, 0.0]).1;
        let near = edge.project(CENTRE, [0.0, -1.0, 0.0]).1;
        assert!(
            far > near,
            "looking along +y, the point at y = +1 should be the farther of the two, but its \
             depth {far} is not above {near}"
        );

        for p in [Preset::Top, Preset::Side, Preset::ThreeQuarter] {
            let cam = Camera::preset(p, 48.0, Vec2::ZERO, 1.0);
            let later = cam.project(CENTRE, [0.0, 0.0, 1.0]).1;
            let earlier = cam.project(CENTRE, [0.0, 0.0, -1.0]).1;
            assert!(
                later < earlier,
                "{p:?}: later time should be nearer the eye, but depth {later} at z = +1 is not \
                 below depth {earlier} at z = -1"
            );
        }
    }

    #[test]
    fn test_the_screen_basis_is_orthonormal_and_right_handed() {
        for yaw in [0.0f32, 0.52, 2.0] {
            for pitch in [0.17f32, 0.61, std::f32::consts::FRAC_PI_2] {
                let cam = Camera { yaw, pitch, scale: 48.0, pan: Vec2::ZERO, t_scale: 1.0 };
                let (right, up, d) = cam.basis();
                for (name, v) in [("right", right), ("up", up), ("d", d)] {
                    let len = dot(v, v).sqrt();
                    assert!(
                        (len - 1.0).abs() < 1e-6,
                        "{name} should be a unit vector at yaw {yaw} pitch {pitch}, but its \
                         length is {len}"
                    );
                }
                for (name, a, b) in
                    [("right.up", right, up), ("right.d", right, d), ("up.d", up, d)]
                {
                    assert!(
                        dot(a, b).abs() < 1e-6,
                        "{name} should vanish at yaw {yaw} pitch {pitch}, but it is {}",
                        dot(a, b)
                    );
                }
                // right x up points at the eye, so it is -d: depth grows away from the eye.
                let c = cross(right, up);
                for k in 0..3 {
                    assert!(
                        (c[k] + d[k]).abs() < 1e-6,
                        "right x up should be -d (the frame (right, up, towards the eye) is \
                         right-handed) at yaw {yaw} pitch {pitch}, but component {k} is {} \
                         against d's {}",
                        c[k],
                        d[k]
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_primitive_buffer_paints_the_farthest_first_and_the_labels_last() {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                Pos2::ZERO,
                egui::Vec2::new(800.0, 600.0),
            )),
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            let painter = ui.painter();
            let mut buf = PrimBuffer::default();
            for (depth, x) in [(1.0, 10.0), (5.0, 50.0), (3.0, 30.0)] {
                buf.push(
                    Layer::Above,
                    depth,
                    Prim::Line {
                        points: vec![Pos2::new(x, 100.0), Pos2::new(x, 140.0)],
                        stroke: Stroke::new(1.0, Color32::RED),
                        closed: false,
                    },
                );
            }
            buf.label(Pos2::new(70.0, 100.0), egui::Align2::LEFT_TOP, "r+", Color32::WHITE);
            buf.paint(Layer::Above, painter);
            buf.paint_labels(painter, egui::FontId::proportional(12.0));
        });

        let shapes = flatten(&output);
        let mut strokes = Vec::new();
        let mut last_stroke = None;
        let mut first_text = None;
        for (i, shape) in shapes.iter().enumerate() {
            match shape {
                egui::Shape::Path(path) => {
                    strokes.push(path.points[0].x);
                    last_stroke = Some(i);
                }
                egui::Shape::Text(_) => {
                    first_text.get_or_insert(i);
                }
                _ => {}
            }
        }
        output.drop_without_applying_deltas();

        assert_eq!(
            strokes,
            vec![50.0, 30.0, 10.0],
            "the strokes should be painted farthest first - depths 5, 3, 1, so screen x 50, 30, 10 - \
             but came out as {strokes:?}"
        );
        let last_stroke = last_stroke.expect("the three strokes were painted");
        let first_text = first_text.expect("the label was painted");
        assert!(
            first_text > last_stroke,
            "every label should follow every primitive, but the first text is shape {first_text} \
             and the last stroke is shape {last_stroke}"
        );
    }

    #[test]
    fn test_a_translucent_strip_is_built_with_premultiplied_vertex_colours() {
        let cam = Camera::default();
        let colour = glass(Theme::HORIZON_OUTER, 60, 0.5);
        let (mesh, _) = quad_mesh(
            &cam,
            CENTRE,
            [
                [2.0, 0.0, -1.0],
                [2.0, 0.0, 1.0],
                [0.0, 2.0, 1.0],
                [0.0, 2.0, -1.0],
            ],
            colour,
        );
        assert!(mesh.is_valid(), "a quad strip must be a mesh egui will accept");
        assert_eq!(mesh.vertices.len(), 4, "a quad has four corners");
        assert_eq!(mesh.indices.len(), 6, "a quad is two triangles sharing a diagonal");
        for (i, v) in mesh.vertices.iter().enumerate() {
            assert_eq!(
                v.color, colour,
                "corner {i} should carry the glass colour {colour:?}, but carries {:?}",
                v.color
            );
            let (r, g, b, a) = (v.color.r(), v.color.g(), v.color.b(), v.color.a());
            assert!(
                r <= a && g <= a && b <= a,
                "corner {i} is not a premultiplied colour: ({r}, {g}, {b}) is not all within \
                 alpha {a}"
            );
        }

        let apex = [0.0, 0.0, 0.0];
        let rim: Vec<[f64; 3]> = (0..36)
            .map(|k| {
                let th = k as f64 * std::f64::consts::TAU / 36.0;
                [th.cos(), th.sin(), 1.0]
            })
            .collect();
        let (cone, _) = cone_mesh(&cam, CENTRE, apex, &rim, colour);
        assert!(cone.is_valid(), "a cone fan must be a mesh egui will accept");
        assert_eq!(cone.vertices.len(), 37, "a cone is its apex plus its 36 rim points");
        assert_eq!(cone.indices.len(), 108, "36 rim points close into 36 triangles");
        assert_eq!(
            cone.vertices[0].pos,
            cam.project(CENTRE, apex).0,
            "vertex 0 is the apex, the event the cone is drawn at"
        );
    }

    #[test]
    fn test_the_rim_weight_is_one_on_the_silhouette_and_zero_facing_the_eye() {
        let (_, _, top) = Camera::preset(Preset::Top, 48.0, Vec2::ZERO, 1.0).basis();
        assert_eq!(top, [0.0, 0.0, -1.0], "the top view looks straight down the time axis");
        for k in 0..8 {
            let th = k as f32 * std::f32::consts::TAU / 8.0;
            let w = rim_weight((th.cos(), th.sin()), top);
            assert!(
                (w - 1.0).abs() < 1e-6,
                "seen from straight above, every vertical wall is edge-on, so the weight at \
                 angle {th} should be 1, not {w}"
            );
        }

        let edge = Camera { yaw: 0.0, pitch: 0.0, scale: 48.0, pan: Vec2::ZERO, t_scale: 1.0 };
        let (_, _, d) = edge.basis();
        let facing = rim_weight((0.0, -1.0), d);
        assert!(
            facing.abs() < 1e-6,
            "the wall whose normal points back at the eye should be clear, weight 0, not {facing}"
        );
        let silhouette = rim_weight((1.0, 0.0), d);
        assert!(
            (silhouette - 1.0).abs() < 1e-6,
            "the wall on the silhouette should be at full weight 1, not {silhouette}"
        );
    }

    /// A transmission with a few of the emitter's pulses in flight, the first of them tagged, and
    /// the emitter themselves standing where the run left them.
    ///
    /// Short on purpose: twelve steps of 0.1 M is eleven pulses, which is more than one
    /// `HISTORY_PULSE_STRIDE`, and every one of their rays is an exact null geodesic that has to be
    /// integrated. The drawing does not care how deep the histories are.
    fn emitting_field(metric: &KerrSchild) -> (SignalField, Observer) {
        let mut emitter = Observer::new_with_phi(
            metric,
            "Bob",
            0.0,
            4.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::new(1.0, 2.2, false),
        );
        let mut field = SignalField::default();
        let dt = 0.1;
        for i in 0..12 {
            let t = ((i + 1) as f64) * dt;
            emitter.step(metric, t, dt);
            field.advance(metric, dt);
            field.emit_if_due(metric, &emitter);
        }
        (field, emitter)
    }

    /// Every colour the wavefront ramp can produce at one opacity, swept finely enough in
    /// log10(gain) that no rounded stop between two of the ramp's knots is missed.
    fn front_ramp(alpha: u8) -> std::collections::HashSet<[u8; 4]> {
        let steps = 60_000;
        (0..=steps)
            .map(|i| {
                let log = Theme::FRONT_LOG_MIN
                    + (Theme::FRONT_LOG_MAX - Theme::FRONT_LOG_MIN) * (i as f64) / (steps as f64);
                Theme::front_colour(10.0_f64.powf(log), alpha).to_array()
            })
            .collect()
    }

    /// The pointer events one step of a drag is made of: the modifiers that are held - egui
    /// carries those on an event of their own rather than on the raw input - then a move, and
    /// optionally the button going down or coming up.
    fn pointer(pos: Pos2, pressed: Option<bool>, modifiers: egui::Modifiers) -> Vec<egui::Event> {
        let mut events =
            vec![egui::Event::ModifiersChanged(modifiers), egui::Event::PointerMoved(pos)];
        if let Some(pressed) = pressed {
            events.push(egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers,
            });
        }
        events
    }

    #[test]
    fn test_ctrl_wheel_is_the_coarse_zoom_and_shift_wheel_the_time_scale() {
        // The wheel means the same on every canvas: a notch is one fine step of zoom, and with
        // ctrl held it is twenty of them, as the (t, r) diagram has it. The one control this view
        // adds, the exchange rate between an M of time and an M of length, lives on shift-wheel so
        // that it cannot be mistaken for a zoom that did nothing.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 4.0);
        let signal = SignalField::default();

        let wheel = |modifiers: egui::Modifiers| {
            let ctx = egui::Context::default();
            ctx.set_fonts(egui::FontDefinitions::empty());
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: false,
                ..Default::default()
            };
            let before = canvas.camera;
            let at = Pos2::new(400.0, 320.0);
            // A warm-up frame puts the canvas on record under the pointer; the wheel turns on the
            // next one.
            for events in [
                pointer(at, None, modifiers),
                vec![
                    egui::Event::ModifiersChanged(modifiers),
                    egui::Event::PointerMoved(at),
                    egui::Event::MouseWheel {
                        unit: egui::MouseWheelUnit::Line,
                        delta: Vec2::new(0.0, 1.0),
                        phase: egui::TouchPhase::Move,
                        modifiers,
                    },
                ],
            ] {
                let raw = egui::RawInput { events, ..input() };
                volume_frame_raw(
                    &mut canvas,
                    &ctx,
                    &metric,
                    Some(&bob),
                    false,
                    &signal,
                    &signal,
                    raw,
                );
            }
            (before, canvas.camera)
        };

        let fine = 1.0 + 0.01875f32;
        let (before, plain) = wheel(egui::Modifiers::NONE);
        let (_, coarse) = wheel(egui::Modifiers::COMMAND);
        let (_, stretched) = wheel(egui::Modifiers::SHIFT);
        println!(
            "scale {}: plain wheel -> {}, ctrl-wheel -> {}; shift-wheel t_scale {} -> {}",
            before.scale, plain.scale, coarse.scale, before.t_scale, stretched.t_scale
        );
        assert!(
            (plain.scale / before.scale - fine).abs() < 1e-4,
            "one notch is one fine step: {} -> {}",
            before.scale,
            plain.scale
        );
        assert!(
            (coarse.scale / before.scale - fine.powi(COARSE_ZOOM_STEPS)).abs() < 1e-3,
            "ctrl-wheel is {COARSE_ZOOM_STEPS} fine steps at once: {} -> {}",
            before.scale,
            coarse.scale
        );
        assert_eq!(plain.t_scale, before.t_scale, "a zoom leaves the time scale alone");
        assert_eq!(coarse.t_scale, before.t_scale);
        assert_eq!(stretched.scale, before.scale, "and shift-wheel is not a zoom");
        assert!(
            stretched.t_scale > before.t_scale,
            "shift-wheel stretches time: {} -> {}",
            before.t_scale,
            stretched.t_scale
        );
    }

    #[test]
    fn test_a_plain_drag_pans_and_a_shift_drag_orbits() {
        // The two canvases are looked at one after the other, and a gesture that means "move the
        // picture" on one and "move the eye" on the other is a gesture the hand has to think
        // about. The equatorial view pans on a plain drag, so this does too, and the orbit - which
        // is the thing only this view has - is the one that takes a modifier.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 4.0);
        let signal = SignalField::default();

        let drag = |modifiers: egui::Modifiers| {
            let ctx = egui::Context::default();
            ctx.set_fonts(egui::FontDefinitions::empty());
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: false,
                ..Default::default()
            };
            let before = canvas.camera;
            let press = Pos2::new(400.0, 320.0);
            for (pos, pressed) in [
                // egui resolves a press against the widgets the *previous* frame registered, so
                // the first frame is the one that puts the canvas on the record.
                (press, None),
                (press, Some(true)),                          // the button goes down
                (press + Vec2::new(30.0, 0.0), None),         // past egui's drag threshold
                (press + Vec2::new(70.0, 24.0), None),        // the move itself
                (press + Vec2::new(70.0, 24.0), Some(false)), // and the release
            ] {
                let raw =
                    egui::RawInput { events: pointer(pos, pressed, modifiers), ..input() };
                volume_frame_raw(
                    &mut canvas,
                    &ctx,
                    &metric,
                    Some(&bob),
                    false,
                    &signal,
                    &signal,
                    raw,
                );
            }
            (before, canvas.camera)
        };

        let (before, after) = drag(egui::Modifiers::NONE);
        println!(
            "plain drag: pan {:?} -> {:?}, yaw {} -> {}, pitch {} -> {}",
            before.pan, after.pan, before.yaw, after.yaw, before.pitch, after.pitch
        );
        assert!(
            (after.pan - before.pan).length() > 1.0,
            "a plain drag pans, but the pan went from {:?} to {:?}",
            before.pan,
            after.pan
        );
        assert_eq!(after.yaw, before.yaw, "and leaves the eye where it was");
        assert_eq!(after.pitch, before.pitch);

        let (before, after) = drag(egui::Modifiers::SHIFT);
        println!(
            "shift-drag: pan {:?} -> {:?}, yaw {} -> {}, pitch {} -> {}",
            before.pan, after.pan, before.yaw, after.yaw, before.pitch, after.pitch
        );
        assert_eq!(after.pan, before.pan, "a shift-drag leaves the picture where it was");
        assert!(
            after.yaw != before.yaw && after.pitch != before.pitch,
            "and moves the eye, but yaw went {} -> {} and pitch {} -> {}",
            before.yaw,
            after.yaw,
            before.pitch,
            after.pitch
        );
    }

    #[test]
    fn test_a_tagged_pulses_light_cone_is_drawn_as_a_gain_coloured_surface() {
        // The one picture in the app where the two families of a pulse can be seen parting company:
        // the sampled front of every eighth pulse, swept up in t, is that pulse's light cone, and
        // the sheet is coloured by the gain each ray carries rather than by whose transmission it
        // is. So the claim is about two things at once - that a surface is drawn at all, and that
        // its colours are the wavefront ramp's own, varying across the sheet. A mesh of one flat
        // colour would be a pipe or a cone; a mesh of colours off the ramp would be a decoration.
        let metric = KerrSchild::new(1.0, 0.65);
        let (field, emitter) = emitting_field(&metric);
        assert!(
            field.pulses.iter().any(|p| p.history().is_some()),
            "the run must leave a tagged pulse in flight to draw"
        );
        let idle = SignalField::default();
        let ramp = front_ramp(FRONT_SURFACE_ALPHA);
        let alpha = Theme::front_colour(1.0, FRONT_SURFACE_ALPHA).a();

        let frame = |on: bool| {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: on,
                ..Default::default()
            };
            volume_frame_signals(
                &mut canvas,
                &metric,
                Some(&emitter),
                false,
                &idle,
                &field,
            )
        };

        let on = frame(true);
        let surfaces: Vec<&Vec<Color32>> = on
            .iter()
            .filter_map(|s| match s {
                Painted::Mesh { colours, .. }
                    if colours.iter().all(|c| c.a() == alpha)
                        && colours.iter().any(|c| *c != colours[0]) =>
                {
                    Some(colours)
                }
                _ => None,
            })
            .collect();
        println!(
            "{} pulse-surface chunks out of {} shapes, the first carrying {} vertices",
            surfaces.len(),
            on.len(),
            surfaces.first().map_or(0, |c| c.len())
        );
        assert!(
            !surfaces.is_empty(),
            "a tagged pulse's light cone is painted as a surface shaded per vertex"
        );
        for colours in surfaces.iter() {
            for c in colours.iter() {
                assert!(
                    ramp.contains(&c.to_array()),
                    "every vertex of a pulse surface is a wavefront gain colour at alpha \
                     {FRONT_SURFACE_ALPHA}, but one is {c:?}"
                );
            }
        }

        let off = frame(false);
        assert!(
            !off.iter().any(|s| matches!(
                s,
                Painted::Mesh { colours, .. }
                    if colours.iter().all(|c| c.a() == alpha)
                        && colours.iter().any(|c| *c != colours[0])
            )),
            "and with the surfaces switched off nothing is drawn in the gain ramp"
        );
    }
}
