use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::spacetime_canvas::{
    COARSE_ZOOM_STEPS, SpacetimeCanvas, TelemetryBoxes, distant_clock_grid_step,
    distant_clock_offset_label,
};
use crate::gui::spatial_canvas::{
    CENTRED_RING_GAP, FrontStyle, RING_DROP_FLOOR, Who, draw_reception_tick, draw_signal_field,
    draw_spatial_trail, frame_focus,
};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::local_frame::{EmbeddedFrame, LocalFrame};
use crate::physics::observer::Observer;
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
/// `EdgeOn` earns its place in a rest frame. There the drawn axes are (xi^1, xi^2, xi^0), and the
/// axial gauge of `Tetrad::from_four_velocity_axial` puts e2^r = 0, so the xi^2 direction is
/// tangent to every surface r = const at the observer's own event: a line of sight along xi^2 lies
/// all but *in* the wall of every pipe where it passes them, and each closes to a narrow band whose
/// slope is its causal character. The picture is the flat rest-frame diagram's (xi^1, xi^0) plane,
/// with the light cone at 45 degrees either side of the axis. Not exactly in, because at pitch 0
/// each wall is seen exactly edge-on - every cell of it projects to zero area - and the horizons
/// vanish from the picture altogether. A few degrees of pitch leaves each one a skinny band, which
/// is what the eye needs to find it.
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

    /// The same orthographic projection, kept in f64: screen (x, y) in pixels.
    ///
    /// `project` rounds the world point to f32 before the dot products, deliberately, so that the
    /// Top preset reproduces the equatorial view bit for bit. That is the right thing for a point
    /// about to become a vertex and the wrong thing for a *measurement*: the adaptive pipe
    /// partition asks how far a strip is from the middle of the canvas and how far its chord has
    /// sagged, and at the zooms a rest frame reaches those distances run to 1e20 px, which f32
    /// cannot order and 1e39 of which it cannot hold at all. So the partition measures here and
    /// only the corners it settles on go through `project`.
    ///
    /// The basis is the same snapped f32 basis widened, rather than recomputed from the angles, so
    /// that the two answers agree about direction exactly.
    pub fn project_f64(&self, centre: Pos2, p: [f64; 3]) -> [f64; 2] {
        let (right, up, _) = self.basis();
        let dot64 = |b: [f32; 3]| {
            p[0] * f64::from(b[0]) + p[1] * f64::from(b[1]) + p[2] * f64::from(b[2])
        };
        let scale = f64::from(self.scale);
        [
            f64::from(centre.x) + dot64(right) * scale,
            f64::from(centre.y) - dot64(up) * scale,
        ]
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
/// numbers; a slice of the distant clock leans in every direction there is, so the rest-frame
/// picture needs the three-component question. It is the one shading rule in the view, and having
/// the two callers share it is what keeps a plane and a pipe reading as the same kind of glass.
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
/// through, chosen by the frame-of-reference selector.
///
/// `Global` is the Kerr-Schild chart the view has always drawn - the equatorial embedding laid out
/// as a floor with coordinate time as height - and the picture it gives is the same for everybody,
/// which is what makes it the right place to read a horizon off. `Frame` is the focus observer's
/// own first-order inertial chart applied *through the embedding*: the tetrad of `LocalFrame`
/// carried over to (Delta t, Delta x, Delta y) by `EmbeddedFrame`, with the drawn axes
/// (xi^1, xi^2, xi^0 t_scale).
///
/// Why through the embedding. The tetrad's own chart xi^a = e^a_mu Delta x^mu applied to
/// (Delta t, Delta r, Delta phi) is linear in *those* coordinates, so r = const is a flat sheet and
/// a circle is a straight line in phi: every pipe becomes a plane and every helix a line, and the
/// five things this view exists to show - pipes, helices, glass, relative positions, cone tilts -
/// are all lost at once. The same tetrad applied to the embedding agrees with it to first order,
/// keeps both properties a rest frame is asked for exactly - the focus observer's worldline is the
/// vertical axis and their cone is the 45 degree cone - and takes a cylinder to a (sheared,
/// elliptic) cylinder and a helix to a helix. See `EmbeddedFrame`.
///
/// Everything the two charts disagree about then follows from the map being linear and the tetrad
/// being orthonormal. In `Frame` the focus observer is at rest at the origin, their cone is exactly
/// 45 degrees, a pipe is sheared over by their boost so that it is tangent to that cone where they
/// cross it, and the distant clock's slices are a fan of lines through their axis that lean into
/// the past cone as u^t runs away. In `Global` nothing is boosted and the cone leans over instead.
/// Neither is a
/// drawing rule: both come out of the same geometry read in two charts.
///
/// It is a first-order chart. The orientations at the focus observer's own event - the tilt of
/// every cone, the causal character of every surface - are exact; finite offsets are the linearised
/// answer. The legend says so.
// One local of the frame, never a collection, so the size difference between a chart that carries a
// tetrad and one that carries a clock reading costs nothing worth an indirection for.
#[allow(clippy::large_enum_variant)]
enum Chart {
    Global {
        /// The simulation clock the floor stands at, so that height is t - t_now.
        t_now: f64,
    },
    Frame {
        /// The chart itself: the tetrad carried through the Kerr-Schild embedding.
        embed: EmbeddedFrame,
        /// The same tetrad's own two-dimensional chart, which the distant clock's label traces and
        /// the ticks of the focus observer's own axis are read off (`surface_t_const`).
        frame: LocalFrame,
    },
}

impl Chart {
    /// The world point an event is drawn at.
    fn world(&self, metric: &KerrSchild, t: f64, r: f64, phi: f64, t_scale: f64) -> [f64; 3] {
        match self {
            Self::Global { t_now } => {
                let (x, y) = metric.cartesian_position(r, phi);
                [x, y, (t - t_now) * t_scale]
            }
            Self::Frame { embed, .. } => {
                let xi = embed.event(metric, t, r, phi);
                [xi[1], xi[2], xi[0] * t_scale]
            }
        }
    }

    /// One point of the wall of the pipe r = const: the point of that surface at the azimuthal
    /// *offset* `dphi` whose height in the drawn volume is `z`.
    ///
    /// This is what makes a pipe a pipe in both charts. In `Global` the height is coordinate time
    /// and the section is the circle of the embedding, whose reference azimuth is 0, so the offset
    /// is the azimuth; in `Frame` the height is the focus observer's own chart time, the reference
    /// azimuth is theirs, and the section is the set of events of that tube which their clock
    /// calls simultaneous. The Global picture is the special case E = identity, and the rows of a
    /// pipe are sections at constant chart height either way.
    ///
    /// An *offset* rather than an azimuth, and never `phi0 + dphi` anywhere along the way: at the
    /// boosts of the stall the arc of a horizon that lands on the canvas spans about 1e-18 rad,
    /// and adding that to an azimuth of order one loses it entirely in f64. See
    /// `EmbeddedFrame::event_at_height_offset`.
    ///
    /// `None` when the point cannot be placed - a collapsed time axis, or a non-finite corner.
    fn pipe_point(
        &self,
        metric: &KerrSchild,
        r: f64,
        dphi: f64,
        z: f64,
        t_scale: f64,
    ) -> Option<[f64; 3]> {
        let p = match self {
            Self::Global { .. } => {
                let (x, y) = metric.cartesian_position(r, dphi);
                [x, y, z]
            }
            Self::Frame { embed, .. } => {
                let xi = embed.event_at_height_offset(metric, r, dphi, z / t_scale)?;
                // The height is the one that was asked for, not xi^0 * t_scale rounded back.
                [xi[1], xi[2], z]
            }
        };
        finite3(p).then_some(p)
    }

    /// Where a coordinate vector k^mu carried at the event (r, phi) points, as a world direction
    /// scaled so that its vertical component is exactly `t_scale`: one unit of the chart's own time
    /// per unit of the parameter, which is what makes `apex + span * direction` a rim at `span` of
    /// that time.
    ///
    /// In `Global` this is `KerrSchild::cartesian_velocity` of the coordinate slopes, which is the
    /// composition `light_cone_generators` performs. In `Frame` it is the same composition followed
    /// by the linear map `world` uses - so a null k stays null, and the rim of the cone it
    /// generates at the focus event is the unit circle at 45 degrees.
    ///
    /// `None` when any component came out non-finite: at u^t ~ 1e10 a mapped vector can overflow
    /// what an f32 projection can carry, and a primitive with a non-finite corner is dropped rather
    /// than drawn.
    fn direction(
        &self,
        metric: &KerrSchild,
        r: f64,
        phi: f64,
        k: &[f64; 3],
        t_scale: f64,
    ) -> Option<[f64; 3]> {
        let d = match self {
            Self::Global { .. } => {
                let (vx, vy) = metric.cartesian_velocity(r, phi, k[1] / k[0], k[2] / k[0]);
                [vx, vy, t_scale]
            }
            Self::Frame { embed, .. } => {
                let v = embed.vector(metric, r, phi, k);
                [v[1] / v[0], v[2] / v[0], t_scale]
            }
        };
        finite3(d).then_some(d)
    }

    fn is_frame(&self) -> bool {
        matches!(self, Self::Frame { .. })
    }
}

/// Whether a world point can be projected at all. A frozen observer's frame carries u^t ~ 1e10, and
/// a displacement mapped through it can be enormous or not a number; `Camera::project` rounds to
/// f32, so the scene drops such a primitive rather than pushing an infinity into a mesh.
fn finite3(p: [f64; 3]) -> bool {
    p[0].is_finite() && p[1].is_finite() && p[2].is_finite()
}

/// One plane of the rest-frame chart, ready to be drawn as a square patch.
///
/// A slice t = const of the chart's Killing time maps to the affine plane n_a xi^a = d, with the
/// normal read off the tetrad legs: n_a = e_a^t, which is `EmbeddedFrame::time_normal`. That is the
/// whole content of the drawing: the numbers come from the geometry and the patch is only how much
/// of an infinite plane fits on the screen.
///
/// The surfaces r = const are *not* drawn this way. In the embedding chart they are cylinders, and
/// a cylinder is drawn as the pipe it is; a plane is what the tetrad's own (t, r, phi) chart made
/// of them, and that is the picture this step replaced.
struct LocalPlane {
    /// Euclidean-unit normal in chart axes (xi^0, xi^1, xi^2). Euclidean, because this is the
    /// geometry of the *picture* - which way the patch faces the eye, where its nearest point is -
    /// and not of the spacetime; the Minkowski norm of the same normal is what says whether the
    /// plane is timelike, and that is read elsewhere.
    unit: [f64; 3],
    /// The point of the plane nearest the chart's origin - nearest the focus observer's own event.
    /// The patch is centred there and a label hung off it.
    nearest: [f64; 3],
    /// Two orthonormal legs spanning the plane, in the same axes.
    legs: [[f64; 3]; 2],
}

/// The plane n_a xi^a = `d` of the local chart, or `None` when the normal has no Euclidean length
/// and there is no plane to speak of.
fn local_plane(n: [f64; 3], d: f64) -> Option<LocalPlane> {
    let len_sq = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // a NaN normal has to take this branch too
    if !(len_sq > 0.0) || !len_sq.is_finite() || !d.is_finite() {
        return None;
    }
    let len = len_sq.sqrt();
    let unit = [n[0] / len, n[1] / len, n[2] / len];
    let nearest = [n[0] * d / len_sq, n[1] * d / len_sq, n[2] * d / len_sq];
    // Any axis that is not nearly parallel to the normal gives a first leg; crossing twice gives
    // the second. The choice of seed rotates the patch about its own normal and nothing else.
    let seed = if unit[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let first = cross3(seed, unit);
    let first_len = (first[0] * first[0] + first[1] * first[1] + first[2] * first[2]).sqrt();
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(first_len > 0.0) {
        return None;
    }
    let u = [first[0] / first_len, first[1] / first_len, first[2] / first_len];
    let v = cross3(unit, u);
    Some(LocalPlane { unit, nearest, legs: [u, v] })
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// How many quads a plane's patch is cut into along each side.
///
/// A plane cannot be one mesh: it runs from the bottom of the window to the top, so a single
/// primitive sorted at one depth would interleave wrongly with every worldline and cone it passes
/// through, and its centroid would place the whole surface in one layer when half of it is in the
/// observer's past and half in their future. Eight a side is 64 pieces, each short enough that its
/// own centroid is a fair place to sort and to layer it.
const PLANE_CELLS: usize = 8;

/// How far past the corner of the canvas a plane's patch reaches, as a multiple of the
/// rect's half-diagonal.
///
/// A patch is a square, and a square drawn in a plane the camera is free to spin has no orientation
/// the canvas can count on: sized to fit, its own corners come into the picture as chevrons and the
/// surface reads as a lozenge rather than as a plane that goes on. Half the diagonal is the radius
/// of the disc the canvas is inscribed in, so a patch of that half-width covers the canvas at every
/// orientation, and half again puts the corners comfortably outside it. The patch is centred on the
/// plane's nearest point to the origin, which is near the middle of the canvas because the focus
/// event *is* the origin, so the overscan is measured from there.
const PATCH_OVERSCAN: f64 = 1.5;

/// Half-width of a plane's patch, in M of the local chart, for a canvas of `rect` at `scale`
/// pixels per M. See `PATCH_OVERSCAN`.
fn patch_half_width(rect: egui::Rect, scale: f32) -> f64 {
    let half_diagonal = f64::from((rect.width() * 0.5).hypot(rect.height() * 0.5));
    PATCH_OVERSCAN * half_diagonal / f64::from(scale).max(1e-6)
}

/// At most this many slices of the distant clock, either side of the observer's own now.
///
/// The bound that actually decides the count is geometric - a slice whose nearest point is further
/// from the origin than the canvas reaches has nothing on screen - and this is the backstop for the
/// case where the step has collapsed relative to the window.
const CLOCK_SLICE_MAX_K: i64 = 100;

/// How close two neighbouring slices of the distant clock may be drawn, in pixels, before their
/// labels are dropped.
///
/// The lines themselves stay: a fan of them leaning into the past cone *is* the picture on the
/// approach to the far branch of r-. What cannot survive it is the text, which at that spacing is a
/// solid block of overlapping glyphs, so the labels go and the geometry stays.
const CLOCK_LABEL_MIN_PX: f32 = 14.0;

/// How far in from the left-hand edge of the canvas the distant clock's labels are hung, in
/// pixels: the same margin, the same distance in, as the flat rest-frame diagram's.
const CLOCK_LABEL_EDGE_PX: f32 = 6.0;

/// The spacing of the global chart's time rungs for a window `span` M tall: the (t, r) diagram's
/// own rule, about seven rungs a window on a 0.5, 1, 2, 5, 10, 25 ladder, so that the two pictures
/// of the foliation are ruled the same way.
fn time_grid_step(span: f64) -> f64 {
    let raw = span / 7.0;
    if raw > 20.0 {
        25.0
    } else if raw > 10.0 {
        10.0
    } else if raw > 4.0 {
        5.0
    } else if raw > 1.5 {
        2.0
    } else if raw > 0.7 {
        1.0
    } else {
        0.5
    }
}

/// The zoom ceilings, in pixels per M, for the two charts.
///
/// In the global chart the ceiling is the equatorial view's, because the two views zoom the same
/// plane at the same rate and a picture of the whole hole has no use for more. A rest frame is
/// another matter: on the approach to the far branch of r- the surface ahead of the observer
/// closes to 1e-10 M of the local chart and the flat diagram follows it down to 1e-12 M, and a
/// volume capped at the equatorial ceiling stops following at about 1e-3 M - after which its
/// framing, its planes and the rung of the observer's own clock all stand still while the outside
/// universe's labels go on climbing, which reads as the ticks having stopped. Seen in the app at
/// about 2 ms a tick. At 1e12 the r- plane is still 260 px from the observer when the integration
/// stalls, so the volume follows the whole fall.
///
/// Anything mapped through a rest frame from far away lands at absurd screen coordinates at that
/// zoom - the other observer, their trail, their pulses - and is kept off the tessellator by
/// `on_stage`.
const SCALE_MAX_GLOBAL: f32 = 500_000.0;
const SCALE_MAX_LOCAL: f32 = 1e12;

/// How far outside the canvas, in pixels, a projected point may fall before it is not drawn.
///
/// A rest frame zoomed to `SCALE_MAX_LOCAL` puts the other observer's worldline at 1e20 px or
/// more, which is a finite number and a meaningless one: a stroke that long has a squared length
/// past f32 and tessellates to NaN. A point a million pixels off the canvas is off the canvas at
/// every zoom this view will ever be looked at in.
const STAGE_PX: f32 = 1e6;

/// Whether a projected point is finite and not absurdly far off the canvas. See `STAGE_PX`.
fn on_stage(p: Pos2, rect: egui::Rect) -> bool {
    p.is_finite()
        && (p.x - rect.center().x).abs() < STAGE_PX
        && (p.y - rect.center().y).abs() < STAGE_PX
}

/// The same point, pulled onto the stage if it is off it, for a drawing helper that takes a
/// projector and cannot skip a point: what it draws there is wrong by at most the width of the
/// stage, which is a million pixels off the canvas either way.
fn stage_clamp(p: Pos2, rect: egui::Rect) -> Pos2 {
    if on_stage(p, rect) {
        return p;
    }
    let c = rect.center();
    let clamp = |v: f32, o: f32| if v.is_finite() { (v - o).clamp(-STAGE_PX, STAGE_PX) + o } else { o };
    Pos2::new(clamp(p.x, c.x), clamp(p.y, c.y))
}

/// Half the length of one tick of the focus observer's own clock on their axis, in pixels, and the
/// backstop on how many of them are drawn either side of their own event. The count that actually
/// decides it is the canvas height over the tick pitch; this is only the guard for a pitch that has
/// collapsed against the window.
const AXIS_CLOCK_TICK_PX: f32 = 4.0;
const AXIS_CLOCK_MAX_K: i64 = 512;

/// How many segments a pipe wall, a floor ring and a tick ring are each cut into *before* the
/// adaptive refinement. Seventy-two is five degrees a segment: at the zoom the view opens on, a
/// chord of five degrees departs from the circle it stands for by well under a pixel, and the
/// pipe's shading needs one strip per segment rather than one vertex, so the count is also the mesh
/// budget of every surface in the global chart, where nothing is ever refined.
const RING_SEGMENTS: usize = 72;

/// How far a chord of a pipe may sag away from the wall it stands for, in pixels, before the
/// segment is bisected. Half a pixel is the width of the seam it would otherwise leave.
const PIPE_CHORD_PX: f64 = 0.5;

/// How many times a segment of a pipe may be bisected.
///
/// Each level halves the arc, so sixty-four levels reach 0.087 * 2^-64 rad, which is below what an
/// offset of order 1e-10 can even resolve in f64. The number that actually stops the recursion is
/// the chord test; this is the backstop, and it is also the bound on the cost - a pipe is at most
/// `RING_SEGMENTS` + 2 * `PIPE_MAX_DEPTH` strips, because only the segments the canvas actually
/// looks at are refined and a bisection that keeps only one of its two halves adds one strip a
/// level.
const PIPE_MAX_DEPTH: u32 = 64;

/// Distance in pixels from a point to a segment, in f64 - the question the adaptive pipe partition
/// asks of each of a strip's four edges. Both a chord that passes the canvas between two far-away
/// vertices and a generator that runs through it have to be found, which is why it is the edges
/// that are measured and not the corners.
fn point_to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len_sq = dx * dx + dy * dy;
    let t = if len_sq > 0.0 && len_sq.is_finite() {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p[0] - (a[0] + t * dx)).hypot(p[1] - (a[1] + t * dy))
}

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
struct ConeRes {
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
    const FULL: Self = Self { dt: PAST_CONE_DT, rays: PAST_CONE_RAYS, full: true };
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
/// says which. The arithmetic lives here rather than at the one place that paints it, because two
/// places want it - section 12 draws the box, and section 4 has to know where it stands so that a
/// label of the distant clock's is not hung underneath it.
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
struct PastCone {
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
fn build_past_cone(metric: &KerrSchild, obs: &Observer, t_min: f64, res: ConeRes) -> PastCone {
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
    centred_on: Option<Who>,
    /// Draw a cone at every whole M the trail crosses, not only at the observer's present event.
    /// Off by default: a dozen translucent fans stacked down a worldline is the picture of how the
    /// cones tip over, and it is also, on a first look at the view, a mess.
    show_ghost_cones: bool,
    /// Draw the exact past light cone of the focus observer's current event: the null geodesics
    /// through it integrated backwards to the bottom of the window, as a surface. On by default,
    /// because it is the one thing this view can show that no other picture in the app can.
    show_past_cone: bool,
    /// That surface, held between frames. Integrating it is the only work in this view that a
    /// camera drag must not repeat, so it is cached against the event it belongs to; see
    /// `PastConeKey` and `ConeRes`.
    past_cone: Option<PastCone>,
    /// Draw the light cone of every tagged pulse as a surface: the ring history the physics keeps
    /// on every `HISTORY_PULSE_STRIDE`-th pulse, swept up in t and coloured by gain. On by default,
    /// because it is the one place in the app where the split between the frozen family and the
    /// crossing family is a shape rather than an inference: the sheet tears in two on r-.
    show_pulse_surfaces: bool,
    /// Keep the next surface the focus observer meets framed, re-deriving `camera.scale` every
    /// frame from the same rule the flat rest-frame diagram uses. See `KEEP_SURFACE_FRAMED_TIP`
    /// and `SpacetimeCanvas::framed_window`.
    ///
    /// It only does anything in a rest frame. The global foliation's zoom is a window on x and y
    /// that the user pans, with no observer at its origin and no surface ahead of anybody in
    /// particular to frame, so in `Chart::Global` this flag is carried and ignored.
    pub keep_surface_framed: bool,
    /// Where the user has dragged each observer's info box on this canvas.
    pub telemetry: TelemetryBoxes,
    /// The screen offset of the focus observer's floor point as the last frame projected it.
    ///
    /// `look_at` has to answer "what pan puts this floor point in the middle", and the middle is
    /// `rect.center() + pan - offset` where the offset is the focus observer's own projection - a
    /// number that needs the metric, both observers and the frame selector, none of which a menu
    /// item hands it. The equatorial view recomputes it, because there the projection is `zoom` and
    /// a multiply; here it is the whole camera, so the frame that has just done the work leaves it
    /// behind. It is only ever read by `look_at`, and the menu that calls `look_at` is registered
    /// inside the frame that has just written it.
    focus_offset: Vec2,
    /// How many surfaces of constant r were drawn as their tangent plane rather than as a pipe on
    /// the last frame. Written by `render`, read only by the tests that are about which of the two
    /// pictures was drawn; see section 4's fallback rule.
    #[cfg(test)]
    fallback_planes: usize,
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
            keep_surface_framed: true,
            telemetry: TelemetryBoxes::pinning(),
            focus_offset: Vec2::ZERO,
            #[cfg(test)]
            fallback_planes: 0,
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

    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        current_time: f64,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
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

        // Which observer's rest frame the view is being asked for, read before anything is drawn
        // because the automatic framing below needs it. The selector names an observer; if that
        // observer is not in the run there is no frame to build, and the scene falls back to the
        // global foliation exactly as `frame_focus` falls back to following nobody.
        let frame_obs = match frame_of_ref {
            ReferenceFrame::Bob => bob,
            ReferenceFrame::Alice => alice,
            ReferenceFrame::DistantObserver => None,
        };
        // The zoom ceiling belongs to the chart: see `SCALE_MAX_LOCAL`.
        let scale_max = if frame_obs.is_some() { SCALE_MAX_LOCAL } else { SCALE_MAX_GLOBAL };

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
                    let new_scale = (old_scale * zoom_mult).clamp(8.0, scale_max);
                    if let Some(mpos) = response.hover_pos() {
                        let nominal = rect.center() + self.camera.pan;
                        self.camera.pan += (mpos - nominal) * (1.0 - new_scale / old_scale);
                    }
                    self.camera.scale = new_scale;
                    // The user has taken the wheel, so the automatic framing stands down until
                    // they ask for it back - the same bargain the flat rest-frame diagram's own
                    // wheel makes, and for the same reason.
                    if frame_of_ref != ReferenceFrame::DistantObserver {
                        self.keep_surface_framed = false;
                    }
                    moved = true;
                }
            }
        }
        if moved {
            ui.ctx().request_repaint();
        }

        // Keep the next surface the focus observer meets on the canvas, if the user has not taken
        // the wheel. It is the flat rest-frame diagram's own rule, carried across intact: the
        // surface r = const the observer is about to reach crosses their time axis at a xi^0 that
        // is exactly the proper time they have left, and `framed_window` turns that into the window
        // in M the picture has to hold. Here the window is spent on the height of the canvas rather
        // than on its width, because in the volume the observer's own clock runs up the screen.
        //
        // A frame that has no answer - nobody selected, an observer with no radial motion, or one
        // stalled on the far branch of r- whose u^r has gone to nothing - leaves the scale where it
        // was rather than moving it to an infinity.
        if self.keep_surface_framed
            && let Some(obs) = frame_obs
            && let Some(window_m) = SpacetimeCanvas::framed_window(metric, obs, rect)
        {
            let scale = (f64::from(rect.height()) * 0.4 / window_m) as f32;
            if scale.is_finite() {
                self.camera.scale = scale.clamp(8.0, scale_max);
            }
        }

        // 2. Where the view is anchored, and therefore where world (0, 0, 0) lands. In locals
        // rather than read through `self`, so that the closures below hold no borrow of the canvas:
        // the right-click menu takes `&mut self` while they are still alive.
        let camera = self.camera;
        let t_scale = camera.t_scale;
        let focus = frame_focus(self.centred_on, frame_of_ref, bob, alice);
        // Which chart the scene is drawn in, from the observer read off the selector above.
        let chart = frame_obs
            .and_then(|obs| {
                // A rest frame is built on a 4-velocity, and an observer placed where their own
                // (E, L) has no real radial root has none: u comes back as NaN and there is no
                // frame at that event to draw. The view says so by drawing the global chart rather
                // than by filling the canvas with nothing.
                let u = obs.four_velocity(metric);
                if !finite3(u) {
                    return None;
                }
                let frame = LocalFrame::for_observer(metric, obs.r, &u);
                let embed = EmbeddedFrame::new(metric, &frame, obs.t, obs.r, obs.phi)?;
                Some(Chart::Frame { embed, frame })
            })
            .unwrap_or(Chart::Global { t_now: current_time });
        // The radius the chart is built at, which is the offset every surface r = const is a plane
        // at in the rest frame's fallback below.
        let focus_r = chart.is_frame().then(|| frame_obs.map(|obs| obs.r)).flatten();
        // In a rest frame the focus observer *is* the origin of the chart, so there is nothing to
        // follow: the offset that keeps them in the middle is zero, and the view is anchored on
        // them by construction rather than by a pan.
        let offset = if chart.is_frame() {
            Vec2::ZERO
        } else {
            focus.map_or(Vec2::ZERO, |obs| {
                let (x, y) = obs.cartesian_position(metric);
                camera.project(Pos2::ZERO, [x, y, 0.0]).0 - Pos2::ZERO
            })
        };
        self.focus_offset = offset;
        let centre = rect.center() + camera.pan - offset;

        // 3. The projection, as the three closures the rest of the frame is written in.
        let project = |p: [f64; 3]| camera.project(centre, p);
        // The floor map is exactly the `Fn((f64, f64)) -> Pos2` the equatorial view's drawing
        // helpers take, which is what lets the fronts, the trails and the arrival ticks be the same
        // code here as there rather than a second implementation that could disagree with it.
        //
        // In a rest frame the floor is no longer a slice of constant t, so a Cartesian point of the
        // equatorial plane cannot simply be dropped onto z = 0: it is inverted back to the chart
        // point (r, phi) it stands for and then placed, as the event (t_now, r, phi), by the same
        // map everything else goes through. That lands the fronts on the tilted slice t = t_now
        // exactly, which is where they actually are.
        let floor = |(x, y): (f64, f64)| match &chart {
            Chart::Global { .. } => camera.project(centre, [x, y, 0.0]).0,
            Chart::Frame { .. } => {
                let (r, phi) = metric.chart_point(x, y, RING_DROP_FLOOR);
                // A front many M from the focus event lands absurdly far off the canvas at the
                // zooms a rest frame reaches; the helper cannot skip a point, so it is clamped.
                stage_clamp(
                    camera.project(centre, chart.world(metric, current_time, r, phi, t_scale)).0,
                    rect,
                )
            }
        };
        // Height is a coordinate-time difference in M, kept in f64 the whole way into `project`,
        // which takes the one rounding to f32 it needs. Near r- the interesting times differ from
        // t_now by parts in 1e7 of t_now itself, and taking the difference in f32 would quantise
        // the whole late fall onto one height.
        let z_of = |t: f64| (t - current_time) * t_scale;
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;
        // The foot of every pipe: the bottom of the window, as a height in the drawn volume. It is
        // read here rather than in section 4 because the adaptive pipe partition below measures a
        // whole strip, from this height up to the floor, and not a point of a section.
        //
        // In a rest frame it is also *clamped to the canvas*. The height of the volume is the
        // chart's own time, and in a rest frame that is the focus observer's proper time: at the
        // framing zoom of a stalled worldline the canvas holds about 3e-9 M of it, while the time
        // window holds 9.8 M. A wall drawn over the whole window is then 2e12 px tall, so every one
        // of its strips has a corner off the stage and is dropped however finely it is cut - the
        // surfaces vanish from the picture on exactly the approach the view exists to show. The
        // canvas's own reach is the same `patch_half_width` the distant clock's planes are sized
        // by, so the wall and the planes are cut off at the same place. In the global chart the
        // height is coordinate time and the window is the window: nothing is clamped.
        let canvas_reach_m = patch_half_width(rect, camera.scale);
        let z_bottom = if frame_obs.is_some() {
            z_of(t_min).max(-canvas_reach_m)
        } else {
            z_of(t_min)
        };
        // The top of the volume, clamped the same way for the same reason: the focus observer's
        // own axis runs from the bottom of the window to the top, and at the zooms a rest frame
        // reaches an unclamped axis is a stroke a trillion pixels long.
        let z_top = if frame_obs.is_some() {
            z_of(t_max).min(canvas_reach_m)
        } else {
            z_of(t_max)
        };
        let view_d = camera.view_direction();

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();
        // Both charts parametrise a pipe by an azimuthal *offset* from their own reference azimuth
        // - the focus event's in `Frame`, zero in `Global` - rather than by the polar angle of the
        // drawn plane. In `Global` the offset is the chart azimuth and differs from the polar angle
        // only by the constant atan2(a, r), which rotates where the vertices fall on one and the
        // same circle. In `Frame` it is the only parametrisation f64 can carry; see
        // `Chart::pipe_point`.
        let base_step = std::f64::consts::TAU / (RING_SEGMENTS as f64);
        let seg_offset = |i: usize| -std::f64::consts::PI + base_step * (i as f64);
        // Whether a corner of a pipe is one this chart is allowed to paint. `pipe_point` has
        // already refused a non-finite one; what is left is the rest frame's own hazard, where at
        // `SCALE_MAX_LOCAL` a perfectly finite chart coordinate lands 1e20 px off the canvas and a
        // primitive built on it tessellates to nothing good. The global chart's coordinates are
        // pixels of a picture of the hole, and the bottom of a pipe there is a few million of them
        // below the floor at the highest zoom it offers - far off the canvas, and the honest place
        // for it - so nothing is dropped in that chart.
        let placeable = |p: [f64; 3]| !chart.is_frame() || on_stage(project(p).0, rect);

        // The measuring rod the adaptive partition below works with: the same projection, in f64,
        // of one point of the wall.
        let wall_at = |r: f64, dphi: f64, z: f64| -> Option<[f64; 2]> {
            let p = chart.pipe_point(metric, r, dphi, z, t_scale)?;
            let at = camera.project_f64(centre, p);
            (at[0].is_finite() && at[1].is_finite()).then_some(at)
        };
        let canvas_centre = [f64::from(rect.center().x), f64::from(rect.center().y)];
        let canvas_reach = canvas_reach_m * f64::from(camera.scale);

        // Where a pipe is cut into strips: the boundaries of its segments, as azimuthal offsets
        // running from -pi to +pi, the two ends being the same point of the circle.
        //
        // Seventy-two uniform segments are the right answer in the global chart at every zoom it
        // offers, and hopeless in a rest frame at the boosts of a late fall: there the chart
        // magnifies an azimuthal offset by u^t, so the arc of r- that lands on the canvas spans
        // about 1e-18 rad, and a uniform sampling of the whole circle puts every one of its
        // vertices 1e20 px away. The surfaces then vanish from the picture on exactly the approach
        // the view exists to show. So each of the 72 segments is bisected, recursively, while all
        // three of these hold:
        //
        //  (i)   the strip's *arc* could reach the canvas - the least screen distance from the
        //        middle of the canvas to one of the quad's four edges (two chords, two generators),
        //        less the sag of the chord, is inside the disc the canvas is inscribed in with
        //        `PATCH_OVERSCAN` to spare. The sag has to be subtracted: a chord can miss by a
        //        mile what the arc it stands for passes straight through, and at the stall it does
        //        exactly that - the wall runs within sixty pixels of the observer while the chord
        //        of the five-degree segment containing that point is 1e18 px away on the far side
        //        of the bulge. Measuring the chord alone rejects every segment at depth zero and
        //        nothing is ever refined. A segment that fails this is kept whole: it is somewhere
        //        else in the picture, and refining it would buy nothing.
        //  (ii)  it is not yet drawable - a corner is still off stage, or the chord has sagged more
        //        than half a pixel away from the curve it stands for.
        //  (iii) the recursion is under `PIPE_MAX_DEPTH`.
        //
        // The sag falls by four per level while the distance to the near point holds, so a segment
        // an angle theta from that point stops being refined once theta exceeds about one eighth of
        // its own width squared: at every level it is one or two segments that go on, and the cost
        // is the depth rather than two to the depth. At the stall that walks down to the 1e-18 rad
        // arc in about fifty levels and the wall comes back as a handful of narrow strips that are
        // on stage and on the canvas. At the intermediate zooms where a pipe subtends a few hundred
        // thousand pixels, (ii) alone fires and takes out the faceting 72 chords would show. In the
        // global chart (ii) fails at depth zero at every zoom - 72 chords of a circle that is never
        // more than a megapixel across are already sub-pixel - so the partition is the uniform one
        // and the picture is unchanged.
        let pipe_partition = |r: f64| -> Vec<f64> {
            let mut out: Vec<f64> = Vec::with_capacity(RING_SEGMENTS + 1);
            // The global chart is never refined. Its coordinates are pixels of a picture of the
            // hole, every one of them within a megapixel of the middle at the highest zoom it
            // offers, so nothing there is ever off stage and the uniform 72 are what that picture
            // has always been drawn from. (The chord test would fire between about 1e3 and 1e5
            // px/M, where a 72-gon of a circle a few thousand pixels across facets by a pixel or
            // so; taking that out would be an improvement, and it would also be a change to the
            // one picture in this view that is the same for everybody, so it is not made here.)
            if !chart.is_frame() {
                out.extend((0..=RING_SEGMENTS).map(seg_offset));
                return out;
            }
            // The quad of one candidate segment, as the four f64 screen points in order around it:
            // the two ends at the bottom of the wall and the two at the floor.
            let quad = |a: f64, b: f64| -> Option<[[f64; 2]; 4]> {
                Some([
                    wall_at(r, a, z_bottom)?,
                    wall_at(r, a, 0.0)?,
                    wall_at(r, b, 0.0)?,
                    wall_at(r, b, z_bottom)?,
                ])
            };
            // How far the chord of a segment sags from the wall it stands for, in pixels, taken at
            // the two heights the strip spans. The map is affine in the height, so if the chord
            // stands for the curve at both ends it stands for it all the way up the wall. `None`
            // when the midpoint cannot be placed at all.
            let sag = |a: f64, b: f64, q: &[[f64; 2]; 4]| -> Option<f64> {
                let mid = 0.5 * (a + b);
                let mut worst = 0.0f64;
                for (z, lo, hi) in [(z_bottom, q[0], q[3]), (0.0, q[1], q[2])] {
                    let m = wall_at(r, mid, z)?;
                    worst = worst
                        .max((m[0] - 0.5 * (lo[0] + hi[0])).hypot(m[1] - 0.5 * (lo[1] + hi[1])));
                }
                worst.is_finite().then_some(worst)
            };
            // Whether the arc could reach the canvas: the nearest of the quad's four edges, less
            // the sag, which is how far the arc may lie off its own chord.
            let reaches = |q: &[[f64; 2]; 4], sag: f64| {
                let nearest = (0..4)
                    .map(|e| point_to_segment(canvas_centre, q[e], q[(e + 1) % 4]))
                    .fold(f64::INFINITY, f64::min);
                nearest - sag < canvas_reach
            };
            let on_stage_quad = |q: &[[f64; 2]; 4]| {
                !chart.is_frame()
                    || q.iter().all(|p| {
                        p[0].abs() <= f64::from(f32::MAX)
                            && p[1].abs() <= f64::from(f32::MAX)
                            && on_stage(Pos2::new(p[0] as f32, p[1] as f32), rect)
                    })
            };
            for i in 0..RING_SEGMENTS {
                let mut stack = vec![(seg_offset(i), seg_offset(i + 1), 0u32)];
                // Popped last-in-first-out with the right half pushed first, so the leaves come
                // out in increasing order of offset and the partition is sorted by construction.
                while let Some((a, b, depth)) = stack.pop() {
                    let split = depth < PIPE_MAX_DEPTH
                        && match quad(a, b).and_then(|q| sag(a, b, &q).map(|s| (q, s))) {
                            Some((q, s)) => {
                                let drawable = on_stage_quad(&q) && s <= PIPE_CHORD_PX;
                                reaches(&q, s) && !drawable
                            }
                            // A corner this chart cannot place at all is not something bisection
                            // can mend: keep the segment whole and let `placeable` drop it.
                            None => false,
                        };
                    if split {
                        let m = 0.5 * (a + b);
                        stack.push((m, b, depth + 1));
                        stack.push((a, m, depth + 1));
                    } else {
                        out.push(a);
                    }
                }
            }
            out.push(std::f64::consts::PI);
            out
        };

        // One section of the surface r = const at chart height z, projected, cut at the same
        // offsets the wall is: the runs of consecutive points this chart can place, and whether
        // the whole loop is among them.
        //
        // Runs rather than all-or-nothing, because a rest frame near the stall can place the arc of
        // a horizon that crosses the canvas and nothing else of it, and that arc is the picture.
        // Only a section every one of whose points is placeable closes.
        let ring_runs = |r: f64, z: f64, cuts: &[f64]| -> (Vec<Vec<Pos2>>, bool) {
            // The last cut is the first one again, so it is not a point of its own.
            let placed: Vec<Option<Pos2>> = cuts[..cuts.len() - 1]
                .iter()
                .map(|&dphi| {
                    let p = chart.pipe_point(metric, r, dphi, z, t_scale)?;
                    placeable(p).then(|| project(p).0)
                })
                .collect();
            let whole = placed.iter().all(Option::is_some);
            if whole {
                return (vec![placed.into_iter().flatten().collect()], true);
            }
            let mut runs: Vec<Vec<Pos2>> = Vec::new();
            let mut run: Vec<Pos2> = Vec::new();
            for slot in placed {
                match slot {
                    Some(at) => run.push(at),
                    None => {
                        if run.len() > 1 {
                            runs.push(std::mem::take(&mut run));
                        } else {
                            run.clear();
                        }
                    }
                }
            }
            if run.len() > 1 {
                runs.push(run);
            }
            (runs, false)
        };

        // A world point from a chart point, and the drawer for one plane of the distant clock: both
        // live here, where the camera and the view direction are.
        let xi_to_world = |xi: [f64; 3]| [xi[1], xi[2], xi[0] * t_scale];
        let push_plane = |buf: &mut PrimBuffer,
                          plane: &LocalPlane,
                          half_width: f64,
                          cells: usize,
                          colour: Color32,
                          alphas: (u8, u8)|
         -> Option<[Pos2; 2]> {
            // The Fresnel weight a pipe's wall is shaded by, asked of a plane: the surface
            // n_0 xi^0 + n_1 xi^1 + n_2 xi^2 = d is n_1 X + n_2 Y + (n_0 / t_scale) Z = d once the
            // world axes are (xi^1, xi^2, xi^0 t_scale), so that is the normal the eye sees it by.
            let weight = face_weight(
                [
                    plane.unit[1] as f32,
                    plane.unit[2] as f32,
                    (plane.unit[0] / t_scale) as f32,
                ],
                view_d,
            );
            let at = |s: f64, t: f64| -> [f64; 3] {
                core::array::from_fn(|i| {
                    plane.nearest[i] + half_width * (s * plane.legs[0][i] + t * plane.legs[1][i])
                })
            };
            let n = cells as f64;
            for i in 0..cells {
                for j in 0..cells {
                    let s0 = -1.0 + 2.0 * (i as f64) / n;
                    let s1 = -1.0 + 2.0 * ((i + 1) as f64) / n;
                    let t0 = -1.0 + 2.0 * (j as f64) / n;
                    let t1 = -1.0 + 2.0 * ((j + 1) as f64) / n;
                    let xi = [at(s0, t0), at(s1, t0), at(s1, t1), at(s0, t1)];
                    let corners = xi.map(xi_to_world);
                    // The same rule the pipes are held to: a cell the chart places off the stage is
                    // dropped rather than handed to the tessellator. A patch is centred on its own
                    // plane's nearest point to the origin, and a surface that is genuinely far away
                    // in this frame - r+ and the ring, seen from a worldline stalled on r- - has
                    // that point tens of billions of pixels off the canvas. Nothing is lost by
                    // leaving it out: it was never going to be seen.
                    if !corners
                        .iter()
                        .all(|c| finite3(*c) && on_stage(camera.project(centre, *c).0, rect))
                    {
                        continue;
                    }
                    // Each piece is placed by its own centroid: the half of a plane that lies in
                    // the observer's future goes over the floor and at half the opacity, so the
                    // past stays the louder of the two exactly as it does for the pipes.
                    let mid = 0.25 * (xi[0][0] + xi[1][0] + xi[2][0] + xi[3][0]);
                    let (layer, alpha) = if mid > 0.0 {
                        (Layer::Above, alphas.1)
                    } else {
                        (Layer::Below, alphas.0)
                    };
                    let (mesh, depth) =
                        quad_mesh(&camera, centre, corners, glass(colour, alpha, weight));
                    buf.push(layer, depth, Prim::Mesh(mesh));
                }
            }
            // Where the plane cuts the floor xi^0 = 0, as the segment joining the two edges of the
            // patch it crosses. xi^0 is affine over the patch, so the crossing on an edge is one
            // linear interpolation and nothing has to be sampled.
            let square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
            let mut hits: Vec<[f64; 3]> = Vec::new();
            for e in 0..4 {
                let (s0, t0) = square[e];
                let (s1, t1) = square[(e + 1) % 4];
                let (a, b) = (at(s0, t0)[0], at(s1, t1)[0]);
                if !a.is_finite() || !b.is_finite() || (a > 0.0) == (b > 0.0) {
                    continue;
                }
                let f = a / (a - b);
                hits.push(at(s0 + f * (s1 - s0), t0 + f * (t1 - t0)));
            }
            if hits.len() < 2 {
                return None;
            }
            let ends = [xi_to_world(hits[0]), xi_to_world(hits[hits.len() - 1])];
            if !ends.iter().all(|c| finite3(*c)) {
                return None;
            }
            Some(ends.map(|p| camera.project(centre, p).0))
        };

        let mut buf = PrimBuffer::default();
        // Where the View Presets box will stand, settled now because the distant clock's labels
        // run up the right-hand edge of the canvas and the box is what they have to dodge.
        let presets = presets_box(rect, font_scale);

        // 4. The surfaces of constant r: glass pipes in *both* charts, and in both a ring on the
        // floor where each crosses it.
        //
        // A pipe is the honest picture of what r = const is: not a circle a worldline happens to
        // cross, but a wall standing in time, so that "Bob went through r+" is a worldline entering
        // a tube and never leaving it. The wall is only drawn below the floor, over the past the
        // simulation has actually integrated; above it there are rings alone, because the future of
        // a horizon is not something this run has computed and a solid wall up there would claim it
        // had.
        //
        // In a rest frame it is the same tube, drawn through the same `pipe_point`: the chart is
        // linear in the embedding, so a cylinder maps to a sheared elliptic cylinder rather than
        // to a plane, and what the observer's boost does to it is read off its lean. On the horizon
        // the lean is exactly the light cone's - the tube is tangent to the cone along the
        // generator d/dt + Omega_H d/dphi, which is
        // `local_frame::tests::test_the_horizon_pipe_is_tangent_to_the_cone_at_the_crossing` - so
        // the 45 degrees the old tangent-plane picture drew is still there, as a tangency rather
        // than as a tilt.
        let tick_r = if metric.cartesian_radius(rm) > 0.0 { rm } else { rp };
        let mut floor_rings: Vec<(Vec<Pos2>, Stroke, bool)> = Vec::new();
        // The ticks of the focus observer's own clock up their own axis, built in section 4b where
        // the rung they are spaced by is worked out and painted in section 9b, after the layer
        // above the floor: they are an annotation on the axis rather than geometry in the volume,
        // and a wash of clock plane laid over one is a tick nobody can read.
        let mut axis_clock_ticks: Vec<egui::Shape> = Vec::new();
        let surfaces = [
            (0.0, Theme::SINGULARITY_LINE, 70u8, 2.0f32),
            (rm, Theme::HORIZON_CAUCHY, 60, 2.0),
            (rp, Theme::HORIZON_OUTER, 60, 2.5),
            (re, Theme::ERGOSPHERE_LINE, 35, 1.5),
        ];
        if let Chart::Frame { embed, frame } = &chart {
            let tetrad = *frame.tetrad();
            // How wide a patch has to be to cover the canvas at every orientation the camera can
            // be spun to. It is a question about the canvas and the zoom and about nothing in the
            // geometry, which is why it is not `window`: a patch sized to the time window showed
            // its own corners the moment the eye was moved off the axes.
            let half = patch_half_width(rect, camera.scale);
            // Where the legend stands, so that a label of the distant clock's does not pile into
            // it. The legend is painted last and would win the pixels either way; what is wanted
            // is for the slice labels not to be under it in the first place.
            let legend_rect = egui::Rect::from_min_size(
                rect.left_top(),
                Vec2::new(rect.width() * 0.6, 7.0 * Theme::MIN_FONT_PT * font_scale),
            );

            // The distant observer's clock, as a fan of lines through the observer's axis rather
            // than as rungs on one pipe.
            //
            // A slice t = const of the chart's Killing time is the set of events that far-away
            // clock labels with one reading, and in this frame it is the plane e_a^t xi^a = dt. It
            // is drawn as the line that plane cuts in the slice xi^2 = 0 through the observer's
            // own worldline - the very line the flat rest-frame diagram draws for it, lifted into
            // the volume - and not as a surface. It used to be a surface: a translucent patch the
            // size of the canvas per slice, and with the rung at a constant pixel pitch a hundred
            // of them stood between the eye and everything else, which tinted the whole picture
            // the grid's colour and said nothing a line does not say. What the slice carries is
            // one-dimensional: where it crosses the observer's neighbourhood and how steeply it
            // leans. The step between neighbours is picked by the same rule the flat diagram uses -
            // a round unit of the observer's *own* clock, carried across by u^t - so the fan keeps
            // a constant pixel pitch while the outside clock runs away. On the approach to the far
            // branch of r- u^t grows like exp(kappa_- t), and what that does to this picture is the
            // whole point of drawing it: the lines lean over into the observer's past cone without
            // limit, so infinitely many of the distant clock's moments are crossed in a finite
            // amount of their own time. Every one of them is flatter than 45 degrees, in every
            // region, because dt is timelike everywhere in this chart.
            //
            // The rung is worked out whether or not the slices are asked for, because section 4b
            // spaces the observer's *own* clock by the same rung divided by u^t, and that clock is
            // theirs rather than the distant one's: it is ticked either way.
            let n_t = embed.time_normal();
            let n_len = (n_t[0] * n_t[0] + n_t[1] * n_t[1] + n_t[2] * n_t[2]).sqrt();
            let seconds_per_m = metric.t_grav_seconds() / metric.m.max(1e-12);
            let u_t = tetrad.e0[0];
            let step = distant_clock_grid_step(u_t, camera.scale, seconds_per_m, font_scale).step_m;
            // The line one slice is drawn as: the plane's intersection with the drawn slice
            // xi^2 = 0 - the same line `LocalFrame::surface_t_const` gives the flat diagram, in
            // (xi^1, xi^0), lifted to world by the chart's own axes - as a point on it and a
            // direction along it. Its label hangs where it crosses the left-hand margin, exactly
            // as the flat rest-frame diagram hangs its labels, so that in the Edge-on preset the
            // margin reads exactly as the flat diagram's does at every boost, and the labels stack
            // up one edge in the order the slices cross it while the picture is left alone.
            let slice_line = |dt: f64| -> Option<([f64; 3], [f64; 3])> {
                if !dt.is_finite() || !finite3(n_t) {
                    return None;
                }
                let line = frame.surface_t_const(dt);
                let at = xi_to_world([line.point[1], line.point[0], 0.0]);
                let along = [line.dir[0], 0.0, line.dir[1] * t_scale];
                (finite3(at) && finite3(along)).then_some((at, along))
            };
            let label_x = rect.left() + CLOCK_LABEL_EDGE_PX;
            let edge_y = |dt: f64| -> Option<f32> {
                let (at, along) = slice_line(dt)?;
                let a = camera.project(centre, at).0;
                let b = camera
                    .project(centre, [at[0] + along[0], at[1] + along[1], at[2] + along[2]])
                    .0;
                let d = b - a;
                // A centreline the camera has turned edge-on projects to a point, and one drawn
                // vertically never reaches the margin at all. Neither has a crossing to label.
                if !a.is_finite() || !d.is_finite() || d.x.abs() < 1e-6 {
                    return None;
                }
                let y = a.y + (label_x - a.x) * d.y / d.x;
                y.is_finite().then_some(y)
            };
            // The pixel-spacing rule, asked where the labels actually land: how far apart in y two
            // neighbouring slices cross that margin. Closer than a glyph's height they are thinned
            // by a stride rather than dropped wholesale, as the flat diagram thins its own - the
            // lines crowding into the past cone is the picture, and a reading every n-th line is
            // still a reading.
            let label_every = match (edge_y(0.0), edge_y(step)) {
                (Some(y0), Some(y1)) => {
                    let spacing = (y1 - y0).abs();
                    (spacing.is_finite() && spacing > 0.0).then(|| {
                        ((CLOCK_LABEL_MIN_PX * font_scale / spacing).ceil() as i64).max(1)
                    })
                }
                _ => None,
            };
            if show_distant_clock_grid
                && step.is_finite()
                && step > 0.0
                && n_len.is_finite()
                && n_len > 0.0
            {
                // A slice whose plane's nearest point is further from the origin than the canvas
                // reaches has nothing on screen: |k step| / |n| > W. That is the bound, and the
                // count cap behind it is only a backstop.
                let reach = half * n_len / step;
                let k_max = if reach.is_finite() {
                    (reach.floor() as i64).clamp(0, CLOCK_SLICE_MAX_K)
                } else {
                    CLOCK_SLICE_MAX_K
                };
                for k in -k_max..=k_max {
                    let dt = (k as f64) * step;
                    let Some((at, along)) = slice_line(dt) else {
                        continue;
                    };
                    // The line runs the canvas's reach either side of its anchor, and is cut at
                    // the floor so that the half in the observer's past is sorted under it with
                    // every other past and the half in their future over it. A point of it the
                    // chart cannot place on the stage ends it there.
                    let norm = (along[0] * along[0] + along[2] * along[2]).sqrt();
                    if norm.is_nan() || norm <= 0.0 {
                        continue;
                    }
                    let span = half / norm;
                    let point_at = |s: f64| -> [f64; 3] {
                        [at[0] + s * along[0], at[1] + s * along[1], at[2] + s * along[2]]
                    };
                    let crossing = (along[2].abs() > 1e-300)
                        .then(|| -at[2] / along[2])
                        .filter(|s| s.abs() < span);
                    let pieces: Vec<(f64, f64)> = match crossing {
                        Some(s0) => vec![(-span, s0), (s0, span)],
                        None => vec![(-span, span)],
                    };
                    for (s_a, s_b) in pieces {
                        let (a, b) = (point_at(s_a), point_at(s_b));
                        if !placeable(a) || !placeable(b) {
                            continue;
                        }
                        let mid_z = 0.5 * (a[2] + b[2]);
                        let layer = if mid_z >= 0.0 { Layer::Above } else { Layer::Below };
                        buf.push(
                            layer,
                            centroid_depth(&camera, centre, [a, b].into_iter()),
                            Prim::Line {
                                points: vec![project(a).0, project(b).0],
                                stroke: Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
                                closed: false,
                            },
                        );
                    }
                    let Some(every) = label_every else {
                        continue;
                    };
                    if k % every != 0 {
                        continue;
                    }
                    let Some(y) = edge_y(dt) else {
                        continue;
                    };
                    // A slice that leaves the canvas above or below the margin has no crossing to
                    // label, and one whose crossing is under the legend or under the View Presets
                    // box has a caption nobody can read. Neither is drawn.
                    let at = Pos2::new(label_x, y);
                    if !rect.contains(at)
                        || legend_rect.contains(at)
                        || presets.rect.contains(at)
                    {
                        continue;
                    }
                    buf.label(
                        at,
                        egui::Align2::LEFT_CENTER,
                        distant_clock_offset_label(dt * seconds_per_m),
                        Theme::TEXT_MUTED,
                    );
                }
            }

            // 4b. The focus observer's own clock, ticked up their own axis.
            //
            // The slice t = t_obs + k * step crosses the axis xi^1 = xi^2 = 0 at
            // xi^0 = k * step / u^t exactly - set xi^1 = 0 in `LocalFrame::surface_t_const`'s line
            // and everything else cancels - so the planes above cut this axis at exact multiples
            // of the round step on the observer's own watch that the rung was chosen to be, and
            // the axis can be ticked with the same numbers the planes are spaced by. It is the
            // flat rest-frame diagram's section 2b, in the volume: the same rung, the same ±4 px
            // ticks, the same labels in the observer's own colour.
            //
            // Drawn whether or not the distant grid is, because this is the observer's own clock
            // rather than the distant one's - and in this view it is the only reading of it there
            // is. At the zoom the automatic framing settles on near r- these are femtoseconds, and
            // the r- plane's crossing of this same axis is read straight off them.
            let clock_proper_step =
                if u_t.is_finite() && u_t > 0.0 { step / u_t } else { f64::INFINITY };
            if clock_proper_step.is_finite() && clock_proper_step > 0.0 {
                let colour = match frame_of_ref {
                    ReferenceFrame::Alice => Theme::ALICE_COLOR,
                    _ => Theme::BOB_COLOR,
                };
                let tick_at = |k: i64| {
                    camera
                        .project(centre, [0.0, 0.0, (k as f64) * clock_proper_step * t_scale])
                        .0
                };
                let origin = tick_at(0);
                // World (0, 0, z) carries no component along the screen's right, at any yaw or
                // pitch, so the axis is vertical on the screen and the pitch of its ticks is a
                // separation in y alone.
                let spacing = (tick_at(1).y - origin.y).abs();
                if spacing.is_finite() && spacing >= 1.0 {
                    let k_max =
                        ((rect.height() / spacing).ceil() as i64).clamp(0, AXIS_CLOCK_MAX_K);
                    let label_every =
                        ((CLOCK_LABEL_MIN_PX * font_scale / spacing).ceil() as i64).max(1);
                    for k in -k_max..=k_max {
                        // k = 0 is the observer's own event, which is already a marker with their
                        // name beside it.
                        if k == 0 {
                            continue;
                        }
                        let at = tick_at(k);
                        if !at.is_finite()
                            || !rect.contains(at)
                            || legend_rect.contains(at)
                            || presets.rect.contains(at)
                        {
                            continue;
                        }
                        axis_clock_ticks.push(egui::Shape::line(
                            vec![
                                Pos2::new(at.x - AXIS_CLOCK_TICK_PX, at.y),
                                Pos2::new(at.x + AXIS_CLOCK_TICK_PX, at.y),
                            ],
                            Stroke::new(1.2, colour),
                        ));
                        if k % label_every != 0 {
                            continue;
                        }
                        buf.label(
                            at + Vec2::new(AXIS_CLOCK_TICK_PX + 3.0, 0.0),
                            egui::Align2::LEFT_CENTER,
                            distant_clock_offset_label(
                                (k as f64) * clock_proper_step * seconds_per_m,
                            ),
                            colour,
                        );
                    }
                }
            }
        }
        #[cfg(test)]
        let mut fallback_planes = 0usize;
        for (r, colour, base_alpha, width) in surfaces {
            // A surface of constant r is the circle of Cartesian radius sqrt(r^2 + a^2); r = 0 is
            // the ring, at rho = |a|, and a hole with no spin has no ring and so no pipe there.
            if metric.cartesian_radius(r) <= 0.0 {
                continue;
            }
            let mut strips = 0usize;
            // Where this pipe is cut: the uniform 72 in the global chart, refined towards the
            // canvas in a rest frame. The floor ring and the rungs are then sections of the very
            // same partition, so a wall and the ring at its foot cannot disagree about where the
            // surface is.
            let cuts = pipe_partition(r);
            for w in cuts.windows(2) {
                let (p0, p1) = (w[0], w[1]);
                // The Fresnel weight wants the wall's outward horizontal normal, which is the
                // radial unit vector of the *drawn* plane at the strip's midpoint - the polar
                // direction of x + iy = (r + ia)e^{i phi}, not of e^{i phi}. `face_weight`
                // normalises, so the position itself is the normal. It is asked at the offset,
                // which in the global chart is the azimuth and in a rest frame is the same
                // direction round the circle: the eye reads it off the drawn plane either way.
                let (nx, ny) = metric.cartesian_position(r, 0.5 * (p0 + p1));
                let weight = rim_weight((nx as f32, ny as f32), view_d);
                // Every corner through `pipe_point`, and a strip with one that cannot be placed is
                // dropped rather than clamped: at the 1e12 px/M a rest frame reaches, the far side
                // of a pipe is 1e20 px away and a primitive built on it tessellates to nothing
                // good. See `placeable`.
                let corners: Option<Vec<[f64; 3]>> =
                    [(p0, z_bottom), (p0, 0.0), (p1, 0.0), (p1, z_bottom)]
                        .into_iter()
                        .map(|(dphi, z)| {
                            let p = chart.pipe_point(metric, r, dphi, z, t_scale)?;
                            placeable(p).then_some(p)
                        })
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
                strips += 1;
            }
            // When a wall is too close to show its curve, it is drawn as its tangent plane.
            //
            // A pipe is cut into sections at constant chart height, and finding one means solving
            // for the coordinate time at which the tube crosses that height. In a rest frame at a
            // large boost that solve is a cancellation the arithmetic cannot carry out: on the
            // approach to r- it asks for a time offset of about 7 M and then for two terms of size
            // 1e10 to cancel down to the 1e-10 M the surface actually stands at, which is twenty
            // digits where f64 has sixteen. Every section then comes back metres away, every strip
            // is off the stage, and the surface disappears from the picture on exactly the approach
            // the view exists to show. Refining the partition cannot mend it; the measurement is in
            // `test_the_rest_frame_zoom_follows_the_fall_to_the_stall`.
            //
            // The geometry is not in doubt - only that one section. The linearised chart puts the
            // same surface at n_a xi^a = r_h - r0 with n_a = e_a^r, in closed form, with nothing
            // subtracted; and wherever a pipe's sections cannot be placed the wall's own curvature
            // over the canvas is far below a pixel, so the plane and the tube are the same picture.
            // So: a surface that yielded no wall at all is drawn as its tangent plane, and one that
            // yielded a wall is a pipe. The rule is per surface, because the two horizons and the
            // ring are at very different distances and a frame can want one of each.
            let mut fell_back = false;
            if strips == 0
                && let Chart::Frame { frame, .. } = &chart
                && let Some(r0) = focus_r
            {
                // n_a = e_a^r, the r-components of the tetrad legs, in the chart's own
                // (xi^0, xi^1, xi^2) order - the same normal `LocalFrame::surface_r_const` draws
                // the flat diagram's line from.
                let t = frame.tetrad();
                if let Some(plane) = local_plane([t.e0[1], t.e1[1], t.e2[1]], r - r0) {
                    let trace = push_plane(
                        &mut buf,
                        &plane,
                        canvas_reach_m,
                        PLANE_CELLS,
                        colour,
                        (base_alpha, base_alpha / 2),
                    );
                    if let Some(ends) = trace {
                        floor_rings.push((ends.to_vec(), Stroke::new(width, colour), false));
                    }
                    fell_back = true;
                }
            }
            #[cfg(test)]
            {
                if fell_back {
                    fallback_planes += 1;
                }
            }
            if !fell_back {
                let (runs, closed) = ring_runs(r, 0.0, &cuts);
                for points in runs {
                    floor_rings.push((points, Stroke::new(width, colour), closed));
                }
            }

            // The distant observer's clock, as rungs on one pipe, labelled with the coordinate
            // time itself - the reading on the chart's clock, not an offset from now - at the step
            // the (t, r) diagram spaces its own time axis by, so that as the clock runs the rungs
            // slide down into the past exactly as that diagram's grid lines do. Rungs at offsets
            // from now would stand still on the screen with the same labels for ever, which is a
            // clock that appears to have stopped. On r-, because that is the pipe a frozen
            // worldline winds up, one turn of helix per 2 pi / Omega_- of t, and the rungs are
            // what that pitch is read against; nothing runs away at r+ in this chart, a faller
            // crosses it at a finite t. The same ladder on all four pipes would be three ladders
            // saying nothing and one saying that. A hole with no spin has no r- pipe, and the
            // rungs go on r+ instead.
            //
            // In the global chart only: there a rung is a slice t = const of the chart's own time,
            // which is what the ladder is a reading of. In a rest frame the surfaces t = const are
            // not sections of a pipe at all - they lean against it - and the distant clock is drawn
            // as the fan of lines section 4 builds instead.
            if show_distant_clock_grid && r == tick_r && !chart.is_frame() {
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
                    let (runs, closed) = ring_runs(r, z, &cuts);
                    for points in runs {
                        buf.push(
                            layer,
                            depth,
                            Prim::Line {
                                points,
                                stroke: Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
                                closed,
                            },
                        );
                    }
                    let text = if use_km {
                        format!("t = {}", metric.format_physical_time(t_val))
                    } else if t_step >= 1.0 {
                        format!("t = {:+}M", t_val as i64)
                    } else {
                        format!("t = {t_val:+.1}M")
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
        // Whose worldline is the chart's own vertical axis, if anybody's. In a rest frame the focus
        // observer is at rest at the origin *by construction* - that is what the chart is - so their
        // worldline is the line xi^1 = xi^2 = 0 and nothing else.
        //
        // It cannot be drawn by pushing their trail through the map. A trail point one M back is a
        // finite coordinate offset, the chart answers for it to first order, and the chord between
        // two such answers is a straight line that leans: the picture then shows the observer
        // drifting sideways through their own rest frame, which is the one thing a rest frame says
        // cannot happen. So the axis is drawn as the axis.
        let axis_who = chart
            .is_frame()
            .then_some(match frame_of_ref {
                ReferenceFrame::Bob => Some(Who::Bob),
                ReferenceFrame::Alice => Some(Who::Alice),
                ReferenceFrame::DistantObserver => None,
            })
            .flatten();
        for (obs, who) in present.iter().copied() {
            // Heavier than the shadow on the floor: the worldline is seen through the glass of
            // whichever pipes stand between it and the eye, and at the floor's weight it was lost
            // behind them.
            let width = match who {
                Who::Alice => 1.8,
                Who::Bob => 2.2,
            };
            if Some(who) == axis_who {
                let origin = [0.0, 0.0, 0.0];
                let foot = [0.0, 0.0, z_bottom];
                let head = [0.0, 0.0, z_top];
                // The past half at the worldline's own weight, under the floor with every other
                // past; the future half over it and faint, because the run has not integrated it
                // and the axis up there is a statement about the chart rather than about anything
                // that has happened.
                buf.push(
                    Layer::Below,
                    centroid_depth(&camera, centre, [foot, origin].into_iter()),
                    Prim::Line {
                        points: vec![project(foot).0, project(origin).0],
                        stroke: Stroke::new(width, colour_of(who)),
                        closed: false,
                    },
                );
                buf.push(
                    Layer::Above,
                    centroid_depth(&camera, centre, [origin, head].into_iter()),
                    Prim::Line {
                        points: vec![project(origin).0, project(head).0],
                        stroke: Stroke::new(width, colour_of(who).gamma_multiply(0.35)),
                        closed: false,
                    },
                );
                continue;
            }
            let points: Vec<(f64, [f64; 3])> = obs
                .trail
                .iter()
                .filter(|p| p.t >= t_min && p.t <= current_time)
                .map(|p| (p.t, chart.world(metric, p.t, p.r, p.phi, t_scale)))
                // A trail point that maps to an infinity is a point of the linearised chart that
                // has run off the far end of f32, and it is dropped rather than drawn: the rest of
                // the worldline is still the worldline. So is one a rest frame has thrown off the
                // stage (see `placeable`): a stroke a million pixels long tessellates to nothing.
                .filter(|(_, p)| finite3(*p) && placeable(*p))
                .collect();
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
                // A run the rest frame's zoom has thrown a million pixels off the canvas is not
                // drawn: its squared length is past f32 and it would tessellate to nothing good.
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
        // the 500 000 px/M the automatic framing reaches on the approach to r-, a rim 1.7 M up the
        // observer's own time is a million pixels off the screen, and what is left on it is the
        // inside of the fill, a uniform tint no eye can tell from the background. A quarter of the
        // canvas height keeps the cone a cone at every zoom.
        let on_canvas = f64::from(rect.height()) * 0.25 / (f64::from(camera.scale) * t_scale);
        let cone_span = (self.time_window * 0.12).min(on_canvas).clamp(1e-12, 1.8);
        let push_cone = |buf: &mut PrimBuffer,
                             t_at: f64,
                             r: f64,
                             phi: f64,
                             fills: (Color32, Color32),
                             ghost: bool,
                             own_chart: bool| {
            let tetrad = Observer::raindrop_tetrad(metric, r);
            // The focus observer's own event is the origin of their chart by definition, and is
            // not computed: the clock and their own t agree to rounding, and at u^t ~ 1e10 and
            // 1e11 px/M a rounding of 1e-12 M is billions of pixels. Seen in the app as the cone
            // vanishing at the stall.
            let apex =
                if own_chart { [0.0; 3] } else { chart.world(metric, t_at, r, phi, t_scale) };
            if !finite3(apex) || !on_stage(project(apex).0, rect) {
                return;
            }
            // The rim is the 36 null directions at the event, each carried a fixed span of the
            // chart's own time by the map the rest of the scene is drawn with. In the global chart
            // that is `light_cone_generators`' own composition and the cone leans over as the
            // geometry says; in a rest frame the same null vectors stay null under a linear map,
            // so the rim comes out as the unit circle and the cone is at exactly 45 degrees.
            //
            // Where the 36 samples fall on the rim depends on the frame they are spread evenly
            // in: seen from a frame boosted by gamma against it they bunch towards the boost, and
            // at the gamma ~ 1e10 of a worldline frozen on r- the raindrop's samples bunch onto a
            // single point of the rim, so the fan has length and no width. Sampling in the
            // observer's own tetrad instead does not survive the arithmetic either: its legs are
            // all of size gamma and nearly parallel, and building a null vector from them and
            // mapping it back cancels the digits away. So the focus observer's own cone in their
            // own chart is not computed at all. There the rim is (cos, sin) at height 1 by
            // construction - the chart is the orthonormal frame the cone is 45 degrees in - which
            // is the one fact the flat rest-frame diagram draws its cone from too.
            let mut future: Vec<[f64; 3]> = Vec::with_capacity(CONE_SAMPLES);
            for i in 0..CONE_SAMPLES {
                let alpha = std::f64::consts::TAU * (i as f64) / (CONE_SAMPLES as f64);
                let d = if own_chart {
                    [alpha.cos(), alpha.sin(), t_scale]
                } else {
                    let Some(d) =
                        chart.direction(metric, r, phi, &tetrad.null_direction(alpha), t_scale)
                    else {
                        return;
                    };
                    d
                };
                let p = [
                    apex[0] + cone_span * d[0],
                    apex[1] + cone_span * d[1],
                    apex[2] + cone_span * d[2],
                ];
                // A rim point a rest frame has thrown off the stage is a cone nobody can see, and
                // a fan built on it is a wash across the canvas rather than a cone: see
                // `placeable`. The focus cone's own rim is on stage by construction.
                if !finite3(p) || !placeable(p) {
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
                let (mesh, depth) = cone_mesh(&camera, centre, apex, rim, fill);
                // In a rest frame the observer's own cone is what the picture is of, and every
                // slice of the distant clock passes through its apex, so a depth sort puts dozens
                // of their cells over it - seen in the app on the approach to r-, where fifty
                // layers of glass left no cone at all. It is painted last in its layer there, over
                // the planes, as the subject of a picture is painted over its context. In the
                // global chart the sort stands: a cone inside a pipe really is seen through the
                // pipe's wall.
                let depth = if chart.is_frame() && !ghost { f32::NEG_INFINITY } else { depth };
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
        for (obs, who) in present.iter().copied() {
            let (future_fill, past_fill, _) =
                Theme::cone_colours_at(&obs.name, Theme::VOLUME_CONE_FILL_ALPHA);
            // The focus observer, drawn in their own chart, is the one cone that is exact by
            // construction rather than by computation.
            let own_chart = chart.is_frame() && Some(who) == axis_who;
            push_cone(
                &mut buf,
                current_time,
                obs.r,
                obs.phi,
                (future_fill, past_fill),
                false,
                own_chart,
            );
            if self.show_ghost_cones {
                let k_lo = t_min.ceil() as i64;
                let k_hi = current_time.ceil() as i64 - 1;
                for k in k_lo..=k_hi {
                    let t = k as f64;
                    // The nearest recorded event to that whole M. The trail is what the run
                    // actually integrated, so a ghost stands on a computed event rather than on an
                    // interpolation between two of them; a whole M the trail does not reach within
                    // one M gets no cone rather than one dragged over to it.
                    let nearest = obs
                        .trail
                        .iter()
                        .filter(|p| p.t >= t_min && p.t < current_time)
                        .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()));
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
                        false,
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
        // The focus is whoever the view is anchored on, falling back to Bob and then Alice. It is a
        // fact about an event rather than about a choice of frame, so it is drawn in both charts,
        // cell by cell through `chart.world`: a quad with a corner the linearised chart has thrown
        // off the end of f32 ends the surface there, exactly as a generator that died at the ring
        // does, so what survives near the focus event is the part of the surface the chart can
        // actually answer for. The right-click menu keeps the switch, because on the late approach
        // to r- the near part of the surface can still wash over the geometry a rest frame is being
        // looked at for.
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
                    Theme::cone_colours_at(&obs.name, Theme::VOLUME_CONE_FILL_ALPHA / 2);
                let to_world = |s: [f64; 3]| chart.world(metric, s[0], s[1], s[2], t_scale);
                let n = cone.rays.len();
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
                            let quad = [
                                to_world(a[k]),
                                to_world(b[k]),
                                to_world(b[k + 1]),
                                to_world(a[k + 1]),
                            ];
                            // A corner the linearised chart has thrown off the end of f32 ends the
                            // surface there, exactly as a generator that died at the ring does -
                            // and so does one a rest frame has thrown off the stage. A cell with a
                            // corner at 1e20 px is not a piece of surface, it is a flat wash over
                            // the whole canvas, which is what the rest frame showed on the approach
                            // to r- before these were held to the pipes' rule. See `placeable`.
                            if !quad.iter().all(|c| finite3(*c) && placeable(*c)) {
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
                    let points: Vec<[f64; 3]> = cone.rays[i]
                        .iter()
                        .map(|s| to_world(*s))
                        .filter(|p| finite3(*p) && placeable(*p))
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
        // The cost is bounded by the physics module's own caps and by the window: at most
        // `MAX_PULSES / HISTORY_PULSE_STRIDE` = 8 tagged pulses per field, 24 kept rays each and
        // `HISTORY_MAX_ROWS` = 256 rows, so 8 x 24 x 255 x 2 = 98 k triangles per field and under
        // 200 k for both - the worst case the caps allow, against a typical window holding a
        // hundred rows of two or three tagged pulses.
        //
        // In both charts, for the reason section 7 gives, and with the same per-cell drop: a quad
        // the rest frame cannot place is left out and the sheet ends there.
        if self.show_pulse_surfaces {
            let to_world = |t: f64, s: &RaySample| {
                chart.world(metric, t, f64::from(s.r), f64::from(s.phi), t_scale)
            };
            for field in [signals.bob, signals.alice] {
                for pulse in field.pulses.iter() {
                    let Some(history) = pulse.history() else {
                        continue;
                    };
                    // Rows are stored oldest first, so the window is a suffix: everything older
                    // than the bottom of the volume is below the floor's floor and is not drawn.
                    let Some(first) = history.rows.iter().position(|row| row.t >= t_min) else {
                        continue;
                    };
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
                                // Off the end of f32, or off the stage in a rest frame: either way
                                // the cell is not drawn, for the reason the past cone gives.
                                if !corner.iter().all(|c| finite3(*c) && placeable(*c)) {
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
        // The region fills are the sections of the same four tubes at the floor's own height, so
        // they are the same shapes in both charts and are built from the same `pipe_point`. In a
        // rest frame the floor xi^0 = 0 is the observer's own local space rather than a slice
        // t = const, and the section of a cylinder by it is an ellipse: the map from the section to
        // the (xi^1, xi^2) floor is affine, so the sections stay convex and stay nested, and "which
        // region am I standing in" is still read off them. A section the chart cannot place in
        // full is left out rather than half drawn.
        //
        // Each region is the band between two sections rather than a disc laid over the discs
        // outside it, for the reason `spatial_canvas::annulus_mesh` gives: a region's pixel colour
        // is then its own fill over the canvas background, the colour the (t, r) diagram paints the
        // same region in. The band is cut at the union of its two tubes' partitions, so its edges
        // pass through the very points those tubes' rings are stroked through.
        //
        // The ring is the tube r = 0, at rho = |a|; a hole with no spin has no ring, so the
        // innermost fill is floored at two pixels of drawn radius - which is the chart radius
        // sqrt((2 px)^2 - a^2) where that is real, and r = 0 itself where |a| already exceeds it.
        let px = 2.0 / f64::from(camera.scale);
        let ring_r = (px * px - metric.a * metric.a).max(0.0).sqrt();
        let section = |r: f64, dphi: f64| -> Option<Pos2> {
            let p = chart.pipe_point(metric, r, dphi, 0.0, t_scale)?;
            placeable(p).then(|| project(p).0)
        };
        // A fill is a region, so it is the whole band or nothing: a run of it is an arc, and an
        // arc closed across its own ends is a lie about which side of the surface one is on. Near
        // the stall the sections run off the stage and the floor carries only the rings.
        let band = |r_in: f64, r_out: f64, fill: Color32| -> Option<egui::Mesh> {
            let mut cuts = pipe_partition(r_in);
            cuts.extend(pipe_partition(r_out));
            cuts.sort_by(f64::total_cmp);
            cuts.dedup();
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
        {
            let (runs, closed) = ring_runs(ring_r, 0.0, &pipe_partition(ring_r));
            if closed && let Some(points) = runs.into_iter().next() {
                painter.add(egui::Shape::convex_polygon(
                    points,
                    Theme::SINGULARITY_FILL,
                    Stroke::NONE,
                ));
            }
        }
        for (points, stroke, closed) in floor_rings {
            painter.add(if closed {
                egui::Shape::closed_line(points, stroke)
            } else {
                egui::Shape::line(points, stroke)
            });
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
        // Only in the global chart: the shadow is the worldline with its time thrown away, and
        // that is a picture of the slice t = t_now. The rest frame's floor is not that slice, so a
        // shadow cast onto it would be a curve of no events at all.
        if !chart.is_frame() {
            if let Some(al) = alice {
                draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &floor);
            }
            if let Some(b) = bob {
                draw_spatial_trail(&painter, metric, b, Theme::BOB_COLOR, 1.5, &floor);
            }
        }

        buf.paint(Layer::Above, &painter);

        // 9b. The ticks of the focus observer's own clock, laid on their axis over both layers.
        // Built in section 4b; painted here, because a tick sorted into the volume with the
        // planes is a tick with an alpha-30 wash of clock plane over it, and this is an
        // annotation on the axis rather than a thing standing in the spacetime. Its labels went
        // through the buffer and land later still, with every other label.
        for tick in axis_clock_ticks {
            painter.add(tick);
        }

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
            // In a rest frame the focus observer's own marker is the origin of the chart, which is
            // the one place in the picture that needs no calculation at all - and must not get
            // one: the clock and their own t agree only to rounding, and at u^t ~ 1e10 and the
            // zooms a rest frame reaches, a rounding of 1e-12 M maps to billions of pixels. Seen
            // in the app as the marker, the cone and the telemetry box vanishing at the stall.
            let world = if Some(who) == axis_who {
                [0.0; 3]
            } else {
                chart.world(metric, current_time, obs.r, obs.phi, t_scale)
            };
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
                // Drawn in somebody's rest frame the view is already anchored on them - they are
                // the origin - so there is nothing for a standing request to do.
                let can_centre = in_run && !chart.is_frame();
                if ui.add_enabled(can_centre, egui::Checkbox::new(&mut centred, label)).changed() {
                    self.centred_on = centred.then_some(who);
                    ui.close();
                }
            }
            ui.separator();
            if ui.checkbox(&mut self.show_ghost_cones, "Ghost cones along the trail").changed() {
                ui.close();
            }
            // Both surfaces are drawn in both charts now, through the same map as everything else,
            // so both switches are the user's in both. They stay switches because near the stall
            // the near part of either surface can wash over the geometry a rest frame is being
            // looked at for.
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
        // looking at a picture. Where the box stands was settled before anything was drawn, in
        // `presets`, because the distant clock's labels have to keep out from under it.
        let legend_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let PresetsBox { rect: box_rect, button: button_size, gap, pad, heading: heading_h } =
            presets;
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
        // Whose chart this is, if it is anybody's. A viewer arriving at a 3D diagram has no way to
        // tell the global foliation from a rest frame by looking at it, and the two say different
        // things about every shape in the picture. It is read off the chart actually drawn, not
        // off the selector: when the selected observer has no frame to build the picture fell back
        // to the global foliation, and the title has to say what is on the canvas.
        let chart_name = chart.is_frame().then(|| frame_obs.map(|obs| obs.name.as_str())).flatten();
        painter.text(
            rect.left_top() + Vec2::new(10.0, 6.0),
            egui::Align2::LEFT_TOP,
            match chart_name {
                Some(name) => format!("2D+1 Volume, {name}'s rest frame (ξ¹, ξ², ξ⁰)"),
                None => "2D+1 Volume (x, y, t)".to_string(),
            },
            legend_font.clone(),
            Theme::TEXT_BRIGHT,
        );
        painter.text(
            rect.left_top() + Vec2::new(10.0, 8.0 + Theme::MIN_FONT_PT * font_scale),
            egui::Align2::LEFT_TOP,
            format!(
                "yaw {:.0}°  pitch {:.0}°  {:.0} px/M  t×{:.2}\n\
                 window {:.1} … {:.1} M  (floor = now){}\n\
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
                // The one claim the rest-frame picture has to make about itself: the tetrad is
                // exact at the observer's own event, so every orientation there - the 45 degree
                // cone, the tilt of each surface - is exact, and everything at a finite offset is
                // the linearised answer, taken through the embedding.
                match chart_name {
                    Some(name) => format!(
                        "\nfirst-order local inertial chart, exact at {name}'s event, extended \
                         linearly through the Kerr-Schild embedding: pipes are sheared, not \
                         curved, by the boost; a wall too close to show its curve is drawn as its \
                         tangent plane"
                    ),
                    None => String::new(),
                },
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

        #[cfg(test)]
        {
            self.fallback_planes = fallback_planes;
        }

        // 14. Draggable info boxes, registered last so they take the drag instead of the canvas.
        for (obs, who) in present {
            let Some((_, at)) = markers.iter().copied().find(|(w, _)| *w == who) else {
                continue;
            };
            self.telemetry.show(
                ui,
                &painter,
                "volume",
                rect,
                at,
                who.name(),
                colour_of(who),
                obs,
                metric,
                use_km,
                font_scale,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

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
        /// read, and the colour is what tells the distant clock's slice labels from the focus
        /// observer's own clock ticked up his axis.
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
        frame_of_ref: ReferenceFrame,
        show_distant_clock_grid: bool,
    ) -> Vec<Painted> {
        volume_frame_with(metric, bob, preset, frame_of_ref, show_distant_clock_grid, false)
    }

    /// The same, with the cones along the trail turned on as the right-click menu turns them on.
    ///
    /// The exact past cone is off in both, and turned on only by the tests that are about it: it
    /// is the one thing in this view whose cost is an integration rather than a projection, and
    /// the tests that sweep every region and both presets draw dozens of frames apiece.
    fn volume_frame_with(
        metric: &KerrSchild,
        bob: Option<&Observer>,
        preset: Preset,
        frame_of_ref: ReferenceFrame,
        show_distant_clock_grid: bool,
        ghosts: bool,
    ) -> Vec<Painted> {
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(preset, 48.0, Vec2::ZERO, 1.0),
            show_ghost_cones: ghosts,
            show_past_cone: false,
            ..Default::default()
        };
        volume_frame_on(&mut canvas, metric, bob, frame_of_ref, show_distant_clock_grid)
    }

    /// One frame drawn into a canvas the caller owns, so that a test can ask what the frame left
    /// behind on it - the past cone's cache being the one piece of scene state this canvas keeps
    /// from one frame to the next.
    fn volume_frame_on(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        frame_of_ref: ReferenceFrame,
        show_distant_clock_grid: bool,
    ) -> Vec<Painted> {
        let signal = SignalField::default();
        volume_frame_signals(
            canvas,
            metric,
            bob,
            frame_of_ref,
            show_distant_clock_grid,
            &signal,
            &signal,
        )
    }

    /// The same, over two transmissions the caller owns, for the tests that are about what the
    /// signal fields put in the volume rather than about the geometry around them.
    #[allow(clippy::too_many_arguments)]
    fn volume_frame_signals(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        frame_of_ref: ReferenceFrame,
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
            frame_of_ref,
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
    /// back with the shapes because the framing rules in this view are measured against the canvas
    /// the frame actually allocated, and a test that wants to predict the zoom has to be measuring
    /// the same rectangle.
    #[allow(clippy::too_many_arguments)]
    fn volume_frame_raw(
        canvas: &mut VolumeCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        frame_of_ref: ReferenceFrame,
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
                frame_of_ref,
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

    /// One frame at a clock the caller sets, rather than at the observer's own t: the app's clock
    /// and a frozen observer's t agree only to rounding, and what a rounding does at the zoom a
    /// rest frame reaches is the subject of a test.
    fn volume_frame_at_clock(
        canvas: &mut VolumeCanvas,
        metric: &KerrSchild,
        bob: &Observer,
        frame_of_ref: ReferenceFrame,
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
                frame_of_ref,
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
        volume_frame_on(
            &mut canvas,
            metric,
            Some(bob),
            ReferenceFrame::DistantObserver,
            false,
        )
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
                false,
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

    /// Every four-cornered mesh painted in one surface's glass, whether it came from a pipe's
    /// strips or from a tangent plane's cells.
    ///
    /// `glass` premultiplies and then scales all four channels together, so a cell keeps its
    /// surface's hue whatever its Fresnel weight and whichever alpha it was given. The ratio of the
    /// channels to the largest of them is that hue, and the five colours the view draws surfaces in
    /// are far enough apart in it that a sixteenth is a comfortable margin.
    fn glass_cells(shapes: &[Painted], surface: Color32) -> Vec<&Vec<Pos2>> {
        let hue = |c: Color32| {
            let m = f32::from(c.r().max(c.g()).max(c.b())).max(1.0);
            [f32::from(c.r()) / m, f32::from(c.g()) / m, f32::from(c.b()) / m]
        };
        let want = hue(surface);
        shapes
            .iter()
            .filter_map(|s| match s {
                Painted::Mesh { vertices: 4, colour: Some(c), points, .. }
                    if (0..3).all(|k| (hue(*c)[k] - want[k]).abs() < 0.0625) =>
                {
                    Some(points)
                }
                _ => None,
            })
            .collect()
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
        let volume = volume_frame(
            &metric,
            Some(&bob_at(&metric, 4.0)),
            Preset::Top,
            ReferenceFrame::DistantObserver,
            false,
        );
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
                    // Alice is not in any of these runs, so her selector has no observer to
                    // build a frame on and the view has to fall back to the global chart rather
                    // than draw nothing.
                    for frame in [
                        ReferenceFrame::DistantObserver,
                        ReferenceFrame::Bob,
                        ReferenceFrame::Alice,
                    ] {
                        let shapes = volume_frame(&metric, Some(&bob), preset, frame, grid);
                        let text = text_of(&shapes);
                        assert!(
                            text.contains("2D+1 Volume"),
                            "{name} at {preset:?} in {frame:?} (grid {grid}) drew no volume view"
                        );
                        assert!(
                            text.contains("View Presets"),
                            "{name} at {preset:?} in {frame:?}: the preset box is named"
                        );
                    }
                }
            }
            // And with a cone at every whole M the trail crossed, which walks the recorded trail
            // and builds a fan from the generators at each of those events: the one path that
            // asks `light_cone_generators` for a frame at a radius the observer has already left.
            let ghosted = text_of(&volume_frame_with(
                &metric,
                Some(&bob),
                Preset::ThreeQuarter,
                ReferenceFrame::DistantObserver,
                false,
                true,
            ));
            assert!(ghosted.contains("2D+1 Volume"), "{name} drew no volume view with ghost cones");
            println!("{name}: drawn at both presets, both frames, grid and ghost cones on and off");
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
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, ReferenceFrame::DistantObserver, true);

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

        // And the count is exactly the uniform partition's, at every zoom the global chart offers.
        // The adaptive partition that cuts a pipe finer where the canvas is looking is a rest
        // frame's business: the global picture is the one that is the same for everybody, and it
        // is drawn from the same 72 segments a surface it has always been drawn from. The chord
        // test would fire here between about 1e3 and 1e5 px/M - see `pipe_partition` - so this is
        // the assertion that keeps it out.
        for scale in [48.0f32, 1e3, 1e4, 1e5, SCALE_MAX_GLOBAL] {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, scale, Vec2::ZERO, 1.0),
                show_past_cone: false,
                show_pulse_surfaces: false,
                ..Default::default()
            };
            let shapes = volume_frame_on(
                &mut canvas,
                &metric,
                Some(&bob),
                ReferenceFrame::DistantObserver,
                false,
            );
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
        let shapes =
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, ReferenceFrame::DistantObserver, false);
        let at = marker_of(&shapes);
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA);

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
            Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA);
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

        let three_quarter = volume_frame(
            &metric,
            Some(&bob),
            Preset::ThreeQuarter,
            ReferenceFrame::DistantObserver,
            false,
        );
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
            volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
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
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let draft = canvas.past_cone.as_ref().expect("the first frame builds the cone").res;
        assert_eq!(draft, ConeRes::MOVING, "and builds it as the moving draft");
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let settled = canvas.past_cone.as_ref().expect("and the second settles it");
        assert_eq!(settled.res, ConeRes::FULL, "at the full resolution, the event having stopped");
        // Whether a third frame integrated anything is a question about identity, not about
        // contents: two builds of the same event agree ray for ray. The address of the outer
        // vector answers it - a rebuild allocates its own while this one is still alive - and no
        // wall clock has to be consulted to ask.
        let settled = settled.rays.as_ptr();
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let cone = canvas.past_cone.as_ref().expect("and the third keeps it");
        assert!(
            std::ptr::eq(cone.rays.as_ptr(), settled),
            "nothing about the event changed, so the third frame must have drawn the very rays              the second integrated"
        );

        // Move him: the cone follows on that frame, at the moving resolution and with no throttle
        // between the step and the picture.
        bob.step(&metric, bob.t + 0.5, 0.5);
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
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
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);

        // One step of the simulation, taken exactly as `Observer::frozen_bob` takes its own.
        bob.step(&metric, bob.t + 0.25, 0.25);
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
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
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
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

        let started = Instant::now();
        let moving = build_past_cone(&metric, &bob, t_min, ConeRes::MOVING);
        let moving_took = started.elapsed();
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
        let fill = Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA / 2).1;

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
        let text = text_of(&volume_frame(
            &metric,
            Some(&frozen),
            Preset::ThreeQuarter,
            ReferenceFrame::DistantObserver,
            false,
        ));
        println!("frozen at r = {:.9}, t = {:.3}", frozen.r, frozen.t);
        assert!(
            text.contains("Frozen"),
            "a worldline riding the r- generator says so in the volume: {text}"
        );

        let falling = bob_at(&metric, 3.0);
        let text = text_of(&volume_frame(
            &metric,
            Some(&falling),
            Preset::ThreeQuarter,
            ReferenceFrame::DistantObserver,
            false,
        ));
        assert!(
            !text.contains("Frozen"),
            "and a Bob who is still falling is not labelled frozen: {text}"
        );
    }

    #[test]
    fn test_the_volume_follows_the_focus_observer() {
        // Drawn in an observer's rest frame the view is anchored on them, exactly as the equatorial
        // view is: the pan is measured from their floor point, so their marker is the middle of the
        // canvas whatever the camera is doing. In the global foliation with nobody centred the
        // anchor is the hole instead, and an observer at r = 4 is nowhere near the middle.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 4.0);
        let global = volume_frame(
            &metric,
            Some(&bob),
            Preset::ThreeQuarter,
            ReferenceFrame::DistantObserver,
            false,
        );
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

        let followed =
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, ReferenceFrame::Bob, false);
        let held = marker_of(&followed);
        println!("Bob's marker: {free:?} unfollowed, {held:?} followed, middle {hole:?}");
        assert!(
            held.distance(hole) < 0.5,
            "in Bob's own rest frame the view is anchored on him, so his marker is the middle of \
             the canvas - but it is at {held:?} against a middle of {hole:?}"
        );
    }

    /// The chart the volume draws in this observer's rest frame.
    fn rest_chart(metric: &KerrSchild, obs: &Observer) -> (LocalFrame, Chart) {
        let frame = LocalFrame::for_observer(metric, obs.r, &obs.four_velocity(metric));
        let embed = EmbeddedFrame::new(metric, &frame, obs.t, obs.r, obs.phi)
            .expect("a timelike observer at r > 0 has an embedded frame");
        (frame, Chart::Frame { embed, frame })
    }

    #[test]
    fn test_in_the_rest_frame_volume_the_focus_cone_is_the_45_degree_circle() {
        // The one thing a rest frame is for. The chart is the dual tetrad, which is linear and
        // orthonormal, so a null vector stays null under it: whatever the null direction and
        // whoever's frame it was written down in, the world direction it maps to rises exactly as
        // fast as it spreads. `hypot(d_1, d_2) = d_0` is the statement "the rim of the cone is the
        // unit circle", i.e. 45 degrees, and nothing in the drawing puts that in by hand - it is
        // what an orthonormal frame *is*.
        //
        // Both of the frame's own generators and the raindrop congruence's are checked, because
        // the drawer takes the raindrop's: the set of null directions belongs to the event and not
        // to a frame, so the rim has to come out at 45 degrees either way.
        let metric = KerrSchild::new(1.0, 0.9);
        for (name, bob) in [
            ("r = 3", bob_at(&metric, 3.0)),
            ("frozen on r-", Observer::frozen_bob(&metric)),
        ] {
            let (frame, chart) = rest_chart(&metric, &bob);
            let raindrop = Observer::raindrop_tetrad(&metric, bob.r);
            let mut worst = [0.0f64; 2];
            for i in 0..CONE_SAMPLES {
                let alpha = std::f64::consts::TAU * (i as f64) / (CONE_SAMPLES as f64);
                for (j, k) in
                    [frame.tetrad().null_direction(alpha), raindrop.null_direction(alpha)]
                        .into_iter()
                        .enumerate()
                {
                    let d = chart
                        .direction(&metric, bob.r, bob.phi, &k, 1.0)
                        .expect("a null direction maps to a finite world direction");
                    worst[j] = worst[j].max((d[0].hypot(d[1]) - d[2]).abs());
                }
            }
            let u_t = frame.tetrad().e0[0];
            println!(
                "{name} (u^t = {u_t:.4e}): the rim is the unit circle to {:e} from his own \
                 generators and to {:e} from the raindrop's",
                worst[0],
                worst[1]
            );
            // The raindrop's are the ones the drawer takes, at every radius and at every u^t, and
            // they are exact to the last few bits either way.
            assert!(
                worst[1] < 1e-9,
                "{name}: a generator taken in the raindrop frame - which is the frame the cone is \
                 actually sampled in - missed 45 degrees by {:e}",
                worst[1]
            );
            // The observer's own generators are the same cone and come out to the same 1e-9 while
            // the frame is conditioned like one. They cannot at the frozen worldline, and the
            // reason is not this chart: `Tetrad::null_direction` adds e0 to e1, whose components
            // are both of order u^t = 1e10, to make a vector whose components are of order 1, so
            // the cancellation has eaten the digits before `Chart::direction` is ever called. It
            // is the same fact `light_cone_generators` records when it says an observer's own
            // frame is the wrong one to sample a rim from - and it is exactly why the drawer takes
            // the raindrop frame, which exists at every radius and is conditioned like one.
            if u_t < 1e3 {
                assert!(
                    worst[0] < 1e-9,
                    "{name}: a generator taken in his own frame missed 45 degrees by {:e}",
                    worst[0]
                );
            } else {
                println!(
                    "  (his own generators are {:e} off at u^t = {u_t:.4e}: that is the \
                     cancellation inside `null_direction` at a boost of 1e10, not the chart, and \
                     it is why the drawer samples the raindrop frame instead)",
                    worst[0]
                );
                assert!(worst[0].is_finite(), "{name}: his own generators still map somewhere");
            }
        }
    }

    #[test]
    fn test_in_the_rest_frame_volume_the_focus_worldline_is_the_vertical_axis() {
        // The observer is at rest at the origin of their own chart, so their worldline is the
        // vertical through it. Only two statements of that are exact - the chart is first order,
        // and a trail point one M back is placed by a linearisation, not by a claim that the
        // worldline is straight - so those two are what is asserted: their current event *is* the
        // origin, and their 4-velocity points straight up it.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 4.0);
        for k in 1..=5 {
            let t = (k as f64) * 0.2;
            bob.step(&metric, t, 0.2);
        }
        assert!(bob.trail.len() > 1, "the walk has to leave a trail to draw");

        let t_scale = 1.5;
        let u = bob.four_velocity(&metric);
        let (_, chart) = rest_chart(&metric, &bob);
        let here = chart.world(&metric, bob.t, bob.r, bob.phi, t_scale);
        assert!(
            here.iter().all(|c| c.abs() < 1e-12),
            "his own event is the origin of his own chart, but it was drawn at {here:?}"
        );
        let d = chart
            .direction(&metric, bob.r, bob.phi, &u, t_scale)
            .expect("a timelike 4-velocity maps to a finite direction");
        println!(
            "Bob at t = {:.2}, r = {:.4}: his event maps to {here:?} and his 4-velocity to {d:?}",
            bob.t, bob.r
        );
        assert!(
            d[0].abs() < 1e-9 && d[1].abs() < 1e-9 && (d[2] - t_scale).abs() < 1e-9,
            "his own worldline is the vertical axis, so his 4-velocity maps to [0, 0, {t_scale}] \
             - but it maps to {d:?}"
        );
    }

    #[test]
    fn test_in_a_rest_frame_the_horizons_are_pipes_through_pipe_point() {
        // What replaced the tangent planes. A surface r = const is a cylinder, and in a rest frame
        // it is still a cylinder - sheared over by the observer's boost, but a tube - so it is
        // drawn by the same strip loop through `Chart::pipe_point` as the global chart's, with the
        // same layer rule: glass below the floor, where the past is, and a ring on the floor.
        //
        // The same claim `test_the_pipes_are_glass_below_the_floor_and_rings_above_it` makes about
        // the global picture, asked of the rest frame, plus the one rule that is particular to it:
        // a strip whose corner the chart cannot place is dropped rather than clamped, so every
        // corner of every strip that *is* painted is on the stage.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);

        let floor = shapes
            .iter()
            .position(
                |s| matches!(s, Painted::Path { fill, .. } if *fill == Theme::SINGULARITY_FILL),
            )
            .expect("the ring's fill is the innermost of the floor's discs, in either chart");
        // The glass of the walls, counted as the global test counts them: a strip is the
        // four-cornered mesh, and with the distant clock off there is nothing else in the rest
        // frame that is one. Four surfaces at `RING_SEGMENTS` segments each, every one of them
        // under the floor.
        let strips: Vec<usize> = shapes
            .iter()
            .enumerate()
            .filter(|(_, s)| matches!(s, Painted::Mesh { vertices: 4, .. }))
            .map(|(i, _)| i)
            .collect();
        println!("{} pipe strips against a floor at {floor}", strips.len());
        assert_eq!(
            strips.len(),
            4 * RING_SEGMENTS,
            "the four surfaces r = const are pipes in a rest frame too, at {RING_SEGMENTS} \
             segments apiece"
        );
        assert!(
            strips.iter().all(|i| *i < floor),
            "every pipe wall belongs under the floor, but one was painted at shape {:?}",
            strips.iter().rfind(|i| **i > floor)
        );
        // And each leaves a closed ring where it crosses the floor, in its own full colour,
        // painted with the floor rather than under it - where the old picture left the open trace
        // of a tangent plane.
        for (name, colour) in [("r-", Theme::HORIZON_CAUCHY), ("r+", Theme::HORIZON_OUTER)] {
            let ring = shapes.iter().skip(floor).any(
                |s| matches!(s, Painted::Path { stroke: Some(c), closed: true, .. } if *c == colour),
            );
            assert!(ring, "{name} leaves a closed ring on the floor of the rest frame");
        }

        // Nothing is clamped onto the stage: a corner the linearised chart cannot place ends the
        // strip. `STAGE_PX` either side of the canvas centre is the whole of what may be painted.
        let canvas_centre = Pos2::new(400.0, 300.0);
        for s in shapes.iter() {
            if let Painted::Mesh { points, .. } = s {
                for p in points {
                    assert!(
                        p.is_finite()
                            && (p.x - canvas_centre.x).abs() <= STAGE_PX
                            && (p.y - canvas_centre.y).abs() <= STAGE_PX,
                        "a mesh corner was painted at {p:?}, which is off the stage"
                    );
                }
            }
        }
    }

    #[test]
    fn test_the_pipe_partition_refines_only_where_the_canvas_looks() {
        // A pipe in a rest frame is cut adaptively: each of the 72 base segments is bisected while
        // its arc could still reach the canvas and is not yet drawable. The two things that have to
        // hold of it are that it does nothing when nothing needs doing, and that it cannot run
        // away - a bisection that refined everywhere would be 2^64 strips.
        //
        // At an ordinary rest-frame zoom every strip is already on stage and sub-pixel, so the
        // partition is the uniform one and the count is exactly four pipes of 72.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);
        let quads =
            shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count();
        assert_eq!(
            quads,
            4 * RING_SEGMENTS,
            "nothing needs refining at 48 px/M in Bob's own frame, so the partition is the uniform \
             one - but {quads} strips were painted"
        );

        // And at the framing zoom of a stalled worldline - where the refinement runs hardest - the
        // work is bounded by the depth rather than by two to the depth, and nothing reaches the
        // tessellator off stage. Only the segments whose arc could still touch the canvas are
        // bisected, and at every level that is one or two of them, so a pipe cannot cost more than
        // its base segments plus two strips a level.
        let frozen = Observer::frozen_bob(&metric);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: true,
            show_past_cone: false,
            ..Default::default()
        };
        let started = Instant::now();
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&frozen), ReferenceFrame::Bob, true);
        let took = started.elapsed();
        let strips: Vec<&Vec<Pos2>> = shapes
            .iter()
            .filter_map(|s| match s {
                Painted::Mesh { vertices: 4, points, .. } => Some(points),
                _ => None,
            })
            .collect();
        println!(
            "frozen Bob framed at {:e} px/M: {} four-cornered meshes, frame built in {took:?}",
            canvas.camera.scale,
            strips.len()
        );
        assert!(
            strips.len() <= 4 * (RING_SEGMENTS + 2 * PIPE_MAX_DEPTH as usize + PLANE_CELLS * PLANE_CELLS),
            "the refinement has to be bounded by its depth, but {} quads were painted",
            strips.len()
        );
        assert!(
            took < Duration::from_millis(20),
            "and it has to fit inside a frame, but the frame took {took:?}"
        );
        let canvas_centre = Pos2::new(400.0, 300.0);
        for points in &strips {
            for p in points.iter() {
                assert!(
                    p.is_finite()
                        && (p.x - canvas_centre.x).abs() <= STAGE_PX
                        && (p.y - canvas_centre.y).abs() <= STAGE_PX,
                    "a corner was painted at {p:?}, off the stage"
                );
            }
        }
    }

    #[test]
    fn test_in_a_rest_frame_the_floor_carries_the_region_fills() {
        // The floor of a rest frame is the observer's own local space xi^0 = 0 rather than a slice
        // t = const, and the section of each tube by it is an ellipse - but the map from the
        // section to the (xi^1, xi^2) floor is affine, so the four regions are still four nested
        // convex polygons and "which region am I standing in" is still read straight off them.
        // They used not to be drawn here at all, because in the tangent-plane picture the floor
        // carried nothing but the traces.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);
        // The shoelace area of a closed loop of points.
        let area_of = |points: &[Pos2]| -> f64 {
            (0.5 * points
                .iter()
                .zip(points.iter().cycle().skip(1))
                .map(|(a, b)| f64::from(a.x) * f64::from(b.y) - f64::from(b.x) * f64::from(a.y))
                .sum::<f64>())
            .abs()
        };
        // Each region is one band between two sections, as a mesh of one quad per cut in the
        // region's own fill - not a disc over the discs outside it, so that its colour on the
        // floor is the (t, r) diagram's colour for the same region. The quads go in as four
        // vertices each, inner-outer-outer-inner, so the two edges can be read back off them.
        // Painted from the outside in, and each band's outer edge is the next one's inner edge.
        let mut previous_inner: Option<f64> = None;
        for (name, fill) in [
            ("the ergosphere", Theme::ERGOSPHERE_FILL),
            ("region II", Theme::REGION_II_FILL),
            ("region III", Theme::REGION_III_FILL),
        ] {
            let points = shapes
                .iter()
                .find_map(|s| match s {
                    Painted::Mesh { colour: Some(c), points, .. } if *c == fill => {
                        Some(points.clone())
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("{name} is a band on the rest frame's floor"));
            assert_eq!(
                points.len(),
                4 * RING_SEGMENTS,
                "{name} is one quad per cut of the uniform partition at this zoom"
            );
            let inner: Vec<Pos2> = points.iter().copied().step_by(4).collect();
            let outer: Vec<Pos2> = points.iter().copied().skip(1).step_by(4).collect();
            let (a_in, a_out) = (area_of(&inner), area_of(&outer));
            println!("{name}: a band from {a_in:.0} px^2 out to {a_out:.0} px^2");
            assert!(a_in > 0.0 && a_out > a_in, "{name} is a band with an inside and a width");
            if let Some(prev) = previous_inner {
                assert!(
                    (a_out - prev).abs() < 1e-3 * prev,
                    "{name}'s outer edge is the edge of the region outside it: {a_out} against \
                     {prev}"
                );
            }
            previous_inner = Some(a_in);
        }
        // The ring is a disc, the innermost of the floor's fills, inside region III.
        let ring = shapes
            .iter()
            .find_map(|s| match s {
                Painted::Path { fill, points, .. } if *fill == Theme::SINGULARITY_FILL => {
                    Some(points.clone())
                }
                _ => None,
            })
            .expect("the ring is filled on the rest frame's floor");
        assert_eq!(ring.len(), RING_SEGMENTS, "the ring is the section of its own tube");
        let a_ring = area_of(&ring);
        assert!(
            a_ring > 0.0 && a_ring <= previous_inner.unwrap() * (1.0 + 1e-3),
            "the ring lies inside region III: {a_ring} against {previous_inner:?}"
        );
    }

    #[test]
    fn test_in_a_rest_frame_the_other_worldline_is_a_helix_not_a_line() {
        // The picture the embedding chart buys, as a number. A circular orbit is a helix wound on
        // a cylinder, and in the tetrad's own (Delta t, Delta r, Delta phi) chart it is a straight
        // line - r is constant and phi is proportional to t, so the whole orbit maps into one
        // ray. Applied through the embedding, the same tetrad takes the same orbit to a helix, and
        // its footprint on the floor winds once per turn.
        //
        // Closed form: the orbit is r = const, phi = Omega t with Omega the circular geodesic's
        // own, and both charts are asked about the same list of events.
        let metric = KerrSchild::new(1.0, 0.9);
        let (r_bob, r_alice) = (8.0f64, 6.0f64);
        // Bob hovering at r = 8: his own worldline is the axis, and the frame is his.
        let g_tt = metric.metric_components(r_bob)[0][0];
        let u = [1.0 / (-g_tt).sqrt(), 0.0, 0.0];
        let frame = LocalFrame::for_observer(&metric, r_bob, &u);
        let embed = EmbeddedFrame::new(&metric, &frame, 0.0, r_bob, 0.0)
            .expect("a hovering observer at r = 8 has an embedded frame");
        let chart = Chart::Frame { embed, frame };

        // Alice on the prograde circular geodesic at r = 6: Omega = 1 / (r^{3/2} + a).
        let omega = 1.0 / (r_alice.powf(1.5) + metric.a);
        let turns = 3.0;
        let steps = 240;
        let period = std::f64::consts::TAU / omega;
        let events: Vec<(f64, f64, f64)> = (0..=steps)
            .map(|k| {
                let t = turns * period * (k as f64) / (steps as f64);
                (t, r_alice, omega * t)
            })
            .collect();

        // The total winding of a footprint about its own centroid: 2 pi a turn for a loop, and
        // about pi for a straight segment, which sweeps its centroid once and stops.
        let winding = |points: &[(f64, f64)]| -> f64 {
            let n = points.len() as f64;
            let cx = points.iter().map(|p| p.0).sum::<f64>() / n;
            let cy = points.iter().map(|p| p.1).sum::<f64>() / n;
            points
                .windows(2)
                .map(|w| {
                    let a = (w[0].1 - cy).atan2(w[0].0 - cx);
                    let b = (w[1].1 - cy).atan2(w[1].0 - cx);
                    let d = b - a;
                    (d + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                        - std::f64::consts::PI
                })
                .sum::<f64>()
                .abs()
        };

        let embedded: Vec<(f64, f64)> = events
            .iter()
            .map(|&(t, r, phi)| {
                let w = chart.world(&metric, t, r, phi, 1.0);
                (w[0], w[1])
            })
            .collect();
        // The same events through the tetrad's own chart, which is what the view used to draw.
        let flat: Vec<(f64, f64)> = events
            .iter()
            .map(|&(t, r, phi)| {
                let xi = frame.to_local(&[t, r - r_bob, crate::physics::local_frame::wrap_pi(phi)]);
                (xi[1], xi[2])
            })
            .collect();

        let (helix, line) = (winding(&embedded), winding(&flat));
        println!(
            "{turns} turns of a circular orbit at r = {r_alice} seen from a hovering Bob at \
             r = {r_bob}: the embedded footprint winds {:.2} turns, the tetrad's own \
             (Delta t, Delta r, Delta phi) chart {:.2}",
            helix / std::f64::consts::TAU,
            line / std::f64::consts::TAU
        );
        assert!(
            helix > 2.0 * std::f64::consts::TAU,
            "three turns of the orbit have to wind at least twice on the floor, but the footprint \
             turned by only {helix} rad"
        );
        assert!(
            line < 1.1 * std::f64::consts::PI,
            "and in the tetrad's own chart the same orbit is a straight line, which cannot wind - \
             but it turned by {line} rad"
        );
    }

    #[test]
    fn test_the_rest_frame_volume_names_its_first_order_chart() {
        // The chart is exact at the observer's event and linearised away from it, and a viewer has
        // no way to tell that by looking at the picture. So the legend says it - and says it only
        // when it is true, because in the global foliation nothing is linearised.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        for (frame, local) in [
            (ReferenceFrame::DistantObserver, false),
            (ReferenceFrame::Bob, true),
            // Alice is not in this run, so her selector falls back to the global chart.
            (ReferenceFrame::Alice, false),
        ] {
            let text =
                text_of(&volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, frame, false));
            assert!(text.contains("2D+1 Volume"), "{frame:?} drew no volume view");
            assert_eq!(
                text.contains("first-order"),
                local,
                "{frame:?}: the first-order banner should be there only in a rest frame, and the \
                 legend reads:\n{text}"
            );
            if local {
                assert!(
                    text.contains("Bob's rest frame"),
                    "and it says whose frame it is: {text}"
                );
                // And which chart it is: the tetrad carried through the embedding, which is why
                // the surfaces are pipes here as well as in the global foliation.
                assert!(
                    text.contains("embedding"),
                    "and that the chart is the first-order one extended through the Kerr-Schild \
                     embedding: {text}"
                );
            }
        }
    }

    #[test]
    fn test_in_the_rest_frame_the_distant_clocks_slices_are_drawn_as_lines() {
        // In the global chart the distant clock is a ladder of rings up one pipe. In a rest frame
        // each slice e_a^t xi^a = k * step is a plane, and it is drawn as the line that plane cuts
        // in the slice xi^2 = 0 through the observer's own worldline - the flat diagram's own line
        // for it, lifted into the volume. Not as a surface: a canvas-sized translucent patch per
        // slice, a hundred deep, tinted the whole picture and said nothing a line does not. So the
        // claim is two counts - turning the grid on adds grid-coloured strokes and adds no
        // four-cornered mesh at all - together with the label on the slice through the observer's
        // own now.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let frame = |grid| {
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, ReferenceFrame::Bob, grid)
        };
        let quads = |shapes: &[Painted]| {
            shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count()
        };
        let grid_lines = |shapes: &[Painted]| {
            shapes
                .iter()
                .filter(|s| {
                    matches!(s, Painted::Path { stroke: Some(c), closed: false, points, .. }
                        if *c == Theme::GRID_LINE && points.len() == 2)
                })
                .count()
        };

        let off = frame(false);
        let on = frame(true);
        let (n_off, n_on) = (quads(&off), quads(&on));
        let (l_off, l_on) = (grid_lines(&off), grid_lines(&on));
        println!(
            "Bob's rest frame: {n_off} quads and {l_off} grid strokes with the distant clock off, \
             {n_on} quads and {l_on} grid strokes with it on"
        );
        assert_eq!(
            n_off,
            4 * RING_SEGMENTS,
            "with the clock off the only quads are the four pipes' strips"
        );
        assert_eq!(n_on, n_off, "the distant clock adds no area to the picture");
        assert_eq!(l_off, 0, "and with it off there is no grid stroke to be seen");
        assert!(
            l_on >= 3,
            "turning the distant clock on draws its slices as lines, but only {l_on} appeared"
        );
        // The slice through the observer's own event reads "now", on a line of its own; the
        // legend's "(floor = now)" is a different line and is there either way.
        let labelled = |shapes: &[Painted]| text_of(shapes).lines().any(|l| l == "now");
        assert!(
            labelled(&on),
            "the slice through the observer's own event is labelled: {}",
            text_of(&on)
        );
        assert!(
            !labelled(&off),
            "and with the clock off there are no slice labels: {}",
            text_of(&off)
        );
    }

    #[test]
    fn test_in_a_rest_frame_the_clock_labels_sit_at_the_left_edge() {
        // Seen in the app: a dozen slice labels strewn diagonally across the middle of the
        // picture, each one hung wherever its own plane's nearest point happened to project to,
        // over the geometry the planes were drawn to be read against. They belong in a margin, as
        // the flat rest-frame diagram's do - and in the Edge-on preset, which is that diagram, the
        // two pictures are the same picture and the labels stack up the edge in the order the
        // slices cross it. The same margin as the flat diagram's, the left; a label that would
        // land under the legend is simply not drawn.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let signal = SignalField::default();
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::EdgeOn, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            ..Default::default()
        };
        let (shapes, rect) = volume_frame_raw(
            &mut canvas,
            &ctx,
            &metric,
            Some(&frozen),
            ReferenceFrame::Bob,
            true,
            &signal,
            &signal,
            input(),
        );

        // Every reading of the distant clock, and nothing else: a slice label is muted text whose
        // first character is the sign of the offset it names. The slice through Bob's own event
        // says "now", his own clock's ticks up the axis are in his own colour, and the legend, the
        // presets and his telemetry all start with a letter.
        let labels: Vec<(&str, egui::Rect)> = shapes
            .iter()
            .filter_map(|s| match s {
                Painted::Text { text, rect, colour }
                    if *colour == Theme::TEXT_MUTED
                        && (text.starts_with('+') || text.starts_with('-')) =>
                {
                    Some((text.as_str(), *rect))
                }
                _ => None,
            })
            .collect();
        let want = rect.left() + CLOCK_LABEL_EDGE_PX;
        println!(
            "{} slice labels against a left edge at {want}: {:?}",
            labels.len(),
            labels.iter().map(|(t, r)| (*t, r.left(), r.center().y)).collect::<Vec<_>>()
        );
        assert!(
            !labels.is_empty(),
            "the slices of the distant clock are labelled in a rest frame: {}",
            text_of(&shapes)
        );
        for (text, at) in &labels {
            assert!(
                (at.left() - want).abs() <= 8.0,
                "every slice label is left-aligned on the margin at x = {want}, but \"{text}\" \
                 starts at {}",
                at.left()
            );
        }
    }

    #[test]
    fn test_the_rest_frame_zoom_follows_the_fall_to_the_stall() {
        // Seen in the app: on the approach to r- the automatic framing ran into the equatorial
        // view's zoom ceiling of 500 000 px/M, and from then on the planes, the framing and the
        // rung of Bob's own clock all stood still at about 2 ms a tick while the outside
        // universe's labels went on climbing - which read as his ticks having stopped. A rest
        // frame has its own ceiling now, high enough that the surface ahead of a frozen Bob is
        // still hundreds of pixels from him, and his clock is ticked in the units that leaves.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: true,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&frozen), ReferenceFrame::Bob, true);
        let ticks: Vec<&str> = shapes
            .iter()
            .filter_map(|s| match s {
                Painted::Text { text, colour, .. }
                    if *colour == Theme::BOB_COLOR && text.starts_with('+') =>
                {
                    Some(text.as_str())
                }
                _ => None,
            })
            .collect();
        println!("frozen Bob framed at {} px/M; his clock reads {ticks:?}", canvas.camera.scale);
        assert!(
            canvas.camera.scale > SCALE_MAX_GLOBAL,
            "a rest frame zooms past the equatorial ceiling to follow the fall: {} px/M",
            canvas.camera.scale
        );
        assert!(
            ticks.iter().any(|t| t.ends_with("ps") || t.ends_with("fs")),
            "and Bob's own clock is ticked in the picoseconds that leaves, not {ticks:?}"
        );
        // The r- surface is on the canvas at that zoom, and it is drawn as its tangent plane
        // rather than as a pipe. A pipe is cut into sections at constant chart height, and at this
        // boost that section cannot be computed: it asks for a coordinate time offset of about 7 M
        // and then for two terms of size 1e10 to cancel down to the 1e-10 M the surface stands at,
        // which is twenty digits where f64 has sixteen. Measured, the sections come back no nearer
        // than 3.1 M at every azimuth, against the 2.6e-10 M the plane gives in closed form with
        // nothing subtracted. So the wall falls back to the plane - and nothing is lost by it,
        // because a wall whose sections cannot be placed is one whose curvature over the whole
        // canvas is far below a pixel. The two are the same picture here; only the section
        // arithmetic is impossible, not the geometry.
        let trace = shapes.iter().find_map(|s| match s {
            Painted::Path { stroke: Some(c), closed: false, points, .. }
                if *c == Theme::HORIZON_CAUCHY =>
            {
                Some(points.clone())
            }
            _ => None,
        });
        assert!(trace.is_some(), "the r- surface leaves a trace on the floor at that zoom");
        let cells = glass_cells(&shapes, Theme::HORIZON_CAUCHY);
        println!(
            "{} cells of r- glass at {} px/M, its floor trace at {:?}",
            cells.len(),
            canvas.camera.scale,
            trace.as_ref().map(|p| p[0])
        );
        assert!(!cells.is_empty(), "and the surface itself is painted, not just its trace");

        // Whichever picture a thing is drawn in, nothing of it reaches the tessellator off the
        // stage: a corner the chart cannot place is dropped rather than clamped. That holds for
        // every mesh and every stroke - a pipe's strips, a patch's cells, a slice of the distant
        // clock, the cone fans, the past cone's and the pulse surfaces' cells, the worldlines. A
        // cell with one corner at 1e20 px is not a piece of surface but a flat wash over the whole
        // canvas, and that wash is what the rest frame showed on the approach to r- before the
        // extended surfaces were held to the pipes' rule.
        let canvas_centre = Pos2::new(400.0, 300.0);
        for s in shapes.iter() {
            let points = match s {
                Painted::Mesh { points, .. } | Painted::Path { points, .. } => points,
                _ => continue,
            };
            for p in points {
                assert!(
                    p.is_finite()
                        && (p.x - canvas_centre.x).abs() <= STAGE_PX
                        && (p.y - canvas_centre.y).abs() <= STAGE_PX,
                    "at {} px/M a point was painted at {p:?}, off the stage",
                    canvas.camera.scale
                );
            }
        }
    }

    #[test]
    fn test_a_wall_too_close_to_curve_falls_back_to_its_tangent_plane() {
        // The rule, both ways round. A surface of constant r is a pipe wherever its sections at
        // constant chart height can be placed, and its own tangent plane wherever they cannot -
        // which is wherever the linearised chart's region of validity has shrunk below the canvas,
        // and there the wall's curve is far under a pixel anyway. The switch is per surface,
        // because the two horizons and the ring sit at very different distances and one frame can
        // want one of each.
        let metric = KerrSchild::new(1.0, 0.9);

        // Gliding on r-, framed: every surface falls back, and each is the tessellated patch of
        // `PLANE_CELLS` a side rather than a run of strips. The count is what tells the two apart -
        // a refined wall would be anything from 73 to 72 + 2 * PIPE_MAX_DEPTH strips, and never
        // exactly 64 - and the canvas's own record of how many surfaces fell back confirms it.
        let frozen = Observer::frozen_bob(&metric);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: true,
            show_past_cone: false,
            ..Default::default()
        };
        let started = Instant::now();
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&frozen), ReferenceFrame::Bob, true);
        let took = started.elapsed();
        println!(
            "frozen Bob at {:e} px/M: {} surfaces fell back to their tangent plane, frame in {took:?}",
            canvas.camera.scale, canvas.fallback_planes
        );
        assert_eq!(
            canvas.fallback_planes,
            4,
            "at the stall no surface's sections can be placed, so all four are planes"
        );
        // r- is the surface that is on the canvas - it is the one the framing is framing - and it
        // is one whole patch of `PLANE_CELLS` a side. The other three are genuinely far away in
        // this frame, tens of billions of pixels off, and their cells are dropped by the same stage
        // rule the pipes are held to rather than handed to the tessellator.
        assert_eq!(
            glass_cells(&shapes, Theme::HORIZON_CAUCHY).len(),
            PLANE_CELLS * PLANE_CELLS,
            "r- is drawn as one patch of {PLANE_CELLS} cells a side"
        );
        for (name, colour) in [
            ("r+", Theme::HORIZON_OUTER),
            ("the ring", Theme::SINGULARITY_LINE),
            ("the ergosphere", Theme::ERGOSPHERE_LINE),
        ] {
            let cells = glass_cells(&shapes, colour);
            assert!(
                cells.is_empty(),
                "{name} is off the canvas from a worldline stalled on r-, so none of its patch is \
                 painted - but {} cells were",
                cells.len()
            );
        }

        // And at an ordinary rest-frame zoom nothing falls back: every surface is a pipe, cut into
        // its `RING_SEGMENTS` strips, and no patch is drawn at all.
        let bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: false,
            show_past_cone: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);
        println!(
            "Bob at r = 3, 48 px/M: {} surfaces fell back",
            canvas.fallback_planes
        );
        assert_eq!(canvas.fallback_planes, 0, "at 48 px/M every surface is a pipe");
        let cells = glass_cells(&shapes, Theme::HORIZON_CAUCHY);
        assert_eq!(
            cells.len(),
            RING_SEGMENTS,
            "r- is {RING_SEGMENTS} strips of wall there, not a patch of {} plane cells",
            PLANE_CELLS * PLANE_CELLS
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
                ReferenceFrame::DistantObserver,
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

    #[test]
    fn test_the_focus_event_is_the_origin_whatever_the_clock_rounding() {
        // Seen in the app at the stall: the marker, the cone and the telemetry box all gone,
        // while the axis and its ticks stayed. The app's clock and the frozen observer's own t
        // differ by a rounding, about 1e-12 M; mapped through his frame that is multiplied by
        // u^t ~ 1e10, and at the 1e11 px/M the framing has reached, by that again - billions of
        // pixels, so his own event was off the stage and everything hung on it was skipped. His
        // event is the origin by definition and is placed there without arithmetic.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let clock = frozen.t * (1.0 + 4.0 * f64::EPSILON);
        assert!(clock != frozen.t, "the clock is a rounding away from Bob's own t");
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: true,
            ..Default::default()
        };
        let shapes =
            volume_frame_at_clock(&mut canvas, &metric, &frozen, ReferenceFrame::Bob, clock, false);
        println!("framed at {} px/M with the clock {clock} against t = {}", canvas.camera.scale, frozen.t);
        let at = marker_of(&shapes);
        let canvas_rect = egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0));
        assert!(canvas_rect.contains(at), "Bob's marker is on the canvas, at {at:?}");
        let (future_fill, _, _) = Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA);
        let cone = shapes.iter().find_map(|s| match s {
            Painted::Mesh { colour: Some(c), first, vertices, .. }
                if *c == future_fill && *vertices == CONE_SAMPLES + 1 =>
            {
                Some(*first)
            }
            _ => None,
        });
        let apex = cone.expect("his cone is painted");
        assert!(apex.distance(at) < 0.5, "and its apex is his marker: {apex:?} against {at:?}");
        assert!(text_of(&shapes).contains("Bob ["), "and his telemetry box is there");
    }

    #[test]
    fn test_in_a_rest_frame_the_focus_clock_is_ticked_up_the_axis() {
        // The volume's rest frame had no reading of the focus observer's *own* clock anywhere on
        // it: the planes are the distant clock's, and the axis they cut was a bare line. The flat
        // diagram ticks that axis at the same rung divided by u^t - which is exactly where the
        // planes cross it - and this is the same ticking in the volume, drawn whether or not the
        // planes themselves are asked for, because it is his clock and not the far-away one's.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        for grid in [false, true] {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                keep_surface_framed: false,
                ..Default::default()
            };
            let shapes =
                volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, grid);
            let axis_x = marker_of(&shapes).x;
            let ticks: Vec<&Vec<Pos2>> = shapes
                .iter()
                .filter_map(|s| match s {
                    Painted::Path { stroke: Some(c), points, .. }
                        if *c == Theme::BOB_COLOR
                            && points.len() == 2
                            && (points[0].y - points[1].y).abs() < 1e-6
                            && ((points[1].x - points[0].x).abs()
                                - 2.0 * AXIS_CLOCK_TICK_PX)
                                .abs()
                                < 1e-6 =>
                    {
                        Some(points)
                    }
                    _ => None,
                })
                .collect();
            let labels: Vec<(&str, egui::Rect)> = shapes
                .iter()
                .filter_map(|s| match s {
                    Painted::Text { text, rect, colour }
                        if *colour == Theme::BOB_COLOR
                            && (text.starts_with('+') || text.starts_with('-')) =>
                    {
                        Some((text.as_str(), *rect))
                    }
                    _ => None,
                })
                .collect();
            println!(
                "distant clock {}: {} ticks on the axis at x = {axis_x}, {} of them labelled: {:?}",
                if grid { "on" } else { "off" },
                ticks.len(),
                labels.len(),
                labels.iter().map(|(t, _)| *t).collect::<Vec<_>>()
            );
            assert!(
                ticks.len() >= 4,
                "his own clock is ticked up his own axis, but only {} short mint strokes were                  painted",
                ticks.len()
            );
            for points in &ticks {
                let mid = 0.5 * (points[0].x + points[1].x);
                assert!(
                    (mid - axis_x).abs() < 0.5,
                    "each tick is centred on the axis at x = {axis_x}, but one runs {:?}",
                    points
                );
            }
            assert!(
                labels.iter().any(|(_, at)| at.left() >= axis_x),
                "and the readings are written beside it, to the right: {labels:?}"
            );
        }
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
    fn test_the_edge_on_preset_lays_every_tangent_plane_flat() {
        // The reason the preset exists. In a rest frame the xi^2 direction is tangent to every
        // surface r = const at the observer's own event, so an eye looking along xi^2 - which is
        // world +y - sees the wall of each pipe as it passes them as a band no wider than the pitch
        // makes it, and the picture is the flat rest-frame diagram's own (xi^1, xi^0) plane. The
        // patch below stands for one such wall. Not pitch 0 exactly: seen in the app, that made
        // every such surface vanish - zero-area cells, a trace that is a point - so the preset
        // stands a few degrees off, and the test asks for the band to be narrow and for it to be
        // there at all.
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
        assert_eq!(right, [1.0, 0.0, 0.0], "and xi^1 across the screen");

        // A plane n . xi = d with n_2 = 0, tilted at some slope in the (xi^1, xi^0) plane, as
        // every r = const plane of the axial gauge is.
        let plane = local_plane([0.8, 0.6, 0.0], 0.7).expect("a plane with a normal");
        let at = |s: f64, t: f64| -> [f64; 3] {
            core::array::from_fn(|i| plane.nearest[i] + 3.0 * (s * plane.legs[0][i] + t * plane.legs[1][i]))
        };
        let xi_to_world = |xi: [f64; 3]| [xi[1], xi[2], xi[0]];
        let samples: Vec<Pos2> = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
            .into_iter()
            .map(|(s, t)| cam.project(CENTRE, xi_to_world(at(s, t))).0)
            .collect();
        // The line the plane would collapse to at pitch 0: through its xi^2 = 0 trace, which the
        // pitch does not move sideways.
        let flat = Camera { pitch: 0.0, ..cam };
        let (a, b) = (
            flat.project(CENTRE, xi_to_world(at(0.0, 0.0))).0,
            flat.project(CENTRE, xi_to_world(at(1.0, 0.0))).0,
        );
        let dir = if a.distance(b) > 1e-3 { b - a } else { flat.project(CENTRE, xi_to_world(at(0.0, 1.0))).0 - a };
        // Off that line by at most the band's half-width: the patch reaches 3 units along xi^2
        // either side, and the pitch lifts each unit by sin(pitch) * scale pixels.
        let band = 3.0 * 48.0 * sp + 1e-3;
        let mut widest = 0.0f32;
        for p in &samples {
            let off = ((p.x - a.x) * dir.y - (p.y - a.y) * dir.x).abs() / dir.length();
            assert!(
                off <= band,
                "near edge-on, a tangent plane is a band no wider than {band} px, but {p:?} is \
                 {off} px off the line through {a:?} along {dir:?}"
            );
            widest = widest.max(off);
        }
        println!("the plane's band is {widest:.1} px half-wide at the edge-on pitch");
        assert!(widest > 1.0, "and wide enough to be seen, not {widest} px");
    }

    #[test]
    fn test_the_cone_stays_on_the_canvas_at_the_framing_zoom() {
        // Seen in the app: with Bob gliding on r- in his own frame the automatic framing runs the
        // zoom up to its ceiling, and a cone whose rim is a fixed 1.7 M of his time away was a
        // million pixels off the canvas - the whole picture was the inside of its fill, and the
        // cone could not be seen at all. The span is now capped by the zoom, so both halves' rims
        // sit inside the canvas at whatever scale the framing has chosen.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            keep_surface_framed: true,
            show_past_cone: false,
            show_pulse_surfaces: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&frozen), ReferenceFrame::Bob, false);
        println!("framing zoom for the frozen Bob: {} px/M", canvas.camera.scale);
        assert!(canvas.camera.scale > 1000.0, "the framing has zoomed in hard on the frozen Bob");
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA);
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
            // And a rim, not a sliver. Seen in the app: sampled in the raindrop frame and mapped
            // into Bob's, all 36 rim points aberrated onto one point at his gamma ~ 1e10, so the
            // fan had length and no width. Sampled in his own tetrad they spread round the whole
            // circle, and the rim's own extent has to say so in both directions of the screen.
            let rim = egui::Rect::from_points(&points[1..]);
            println!("{half} rim: radius {radius:.0} px, extent {:.0} x {:.0} px", rim.width(), rim.height());
            assert!(
                rim.width() > 0.8 * radius && rim.height() > 0.3 * radius,
                "the {half} rim should spread round the cone, but its extent is {:.0} x {:.0} px \
                 against a radius of {radius:.0} px",
                rim.width(),
                rim.height()
            );
        }
    }

    #[test]
    fn test_in_a_rest_frame_the_focus_cone_is_painted_over_the_planes() {
        // Seen in the app with Bob gliding on r- in his own frame: the cone was painted, on the
        // canvas, at a readable size, and invisible - every surface and every slice of the distant
        // clock passes through its apex, and the depth sort laid some fifty of their cells over
        // it. In a rest frame the cone is therefore painted last in its layer: the future half is
        // the last mesh of the whole frame, and the past half comes after every mesh that lies
        // under the floor with it.
        //
        // Both ends of the range the rest frame is looked at over. Bob gliding on r- is the case
        // the rule was written for, and there the picture is the clock's planes; Bob at r = 3 is
        // the ordinary case, where the pipes and the pulse surfaces are under the floor with the
        // past half and it is their ordering that has to hold.
        let metric = KerrSchild::new(1.0, 0.9);
        let frozen = Observer::frozen_bob(&metric);
        let falling = bob_at(&metric, 3.0);
        let (future_fill, past_fill, _) =
            Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA);
        for (name, bob, framed) in
            [("gliding on r-", &frozen, true), ("falling at r = 3", &falling, false)]
        {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                keep_surface_framed: framed,
                ..Default::default()
            };
            let shapes = volume_frame_on(&mut canvas, &metric, Some(bob), ReferenceFrame::Bob, true);
            let index_of = |fill: Color32| {
                shapes.iter().position(|s| matches!(s, Painted::Mesh { colour: Some(c), vertices, .. } if *c == fill && *vertices == CONE_SAMPLES + 1))
            };
            let future = index_of(future_fill).expect("the future half is painted");
            let past = index_of(past_fill).expect("the past half is painted");
            let last_mesh = shapes.iter().rposition(|s| matches!(s, Painted::Mesh { .. })).unwrap();
            let meshes = shapes.iter().filter(|s| matches!(s, Painted::Mesh { .. })).count();
            println!(
                "{name}: {meshes} meshes; past half at {past}, future half at {future}, last mesh \
                 at {last_mesh}"
            );
            assert_eq!(
                future, last_mesh,
                "{name}: the future half of the focus cone is painted over every plane"
            );
            // Under the floor the past half is last too: nothing but the floor's own strokes and
            // the layer above separate it from the future half's neighbours, so no mesh painted
            // between the two halves may lie under the floor - and every mesh under the floor
            // precedes it. The floor is found by its first fill, which is painted between the two
            // layers in either chart: the outermost band where the sections can be placed, the
            // ring's disc where only it can.
            let region_fills =
                [Theme::ERGOSPHERE_FILL, Theme::REGION_II_FILL, Theme::REGION_III_FILL];
            let Some(floor) = shapes.iter().position(|s| match s {
                Painted::Path { fill, .. } => *fill == Theme::SINGULARITY_FILL,
                Painted::Mesh { colour: Some(c), .. } => region_fills.contains(c),
                _ => false,
            }) else {
                // At the stall the sections of the tubes are off the stage and the floor carries
                // nothing at all, so there is no marker to measure the past half against; the
                // claim about the future half above is the one that matters there anyway.
                println!("  (nothing on the floor at {} px/M)", canvas.camera.scale);
                continue;
            };
            assert!(past < floor, "{name}: the past half is painted under the floor");
            let between =
                shapes[past + 1..floor].iter().filter(|s| matches!(s, Painted::Mesh { .. })).count();
            assert_eq!(
                between, 0,
                "{name}: and after every other mesh under it, but {between} follow it"
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

    /// Whether a stroke is one of Bob's own.
    ///
    /// `BOB_COLOR` is opaque, so every fade of it is a `gamma_multiply` of the whole premultiplied
    /// colour: the red channel stays at zero and the green-to-blue ratio is held at 200/255. That
    /// identifies it whatever the alpha, and it tells his mint apart from the only other red-free
    /// stroke in the picture, the outer horizon's cyan at 255/230.
    fn is_bob_stroke(c: Color32) -> bool {
        c.r() == 0 && c.g() > 0 && (f32::from(c.b()) / f32::from(c.g()) - 200.0 / 255.0).abs() < 0.05
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
                keep_surface_framed: false,
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
                    ReferenceFrame::DistantObserver,
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
                keep_surface_framed: false,
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
                    ReferenceFrame::DistantObserver,
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
    fn test_in_a_rest_frame_the_focus_worldline_is_the_axis() {
        // The focus observer is at rest at the origin of their own chart: that is what the chart
        // is. Their worldline is therefore the vertical through it, and drawing it instead by
        // pushing their recorded trail through the map draws something else - the chord between
        // two first-order answers about a curved worldline, which leans, and which reads as the
        // observer drifting sideways through the frame they themselves define.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 6.0);
        for k in 1..=8 {
            bob.step(&metric, (k as f64) * 0.25, 0.25);
        }
        assert!(bob.trail.len() > 2, "the walk has to leave a trail to draw");

        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);
        let at = marker_of(&shapes);

        let vertical = |points: &[Pos2]| points.iter().all(|p| (p.x - points[0].x).abs() < 0.5);
        let mint: Vec<(&Vec<Pos2>, bool)> = shapes
            .iter()
            .filter_map(|s| match s {
                Painted::Path { stroke: Some(c), points, closed, .. } if is_bob_stroke(*c) => {
                    Some((points, *closed))
                }
                _ => None,
            })
            .collect();
        // The ticks of his own clock are mint strokes too, and horizontal on purpose: they are
        // the rungs section 4b lays across the axis, not pieces of it. Two points at one height is
        // what tells them from a run of worldline.
        let is_tick = |p: &Vec<Pos2>| p.len() == 2 && (p[0].y - p[1].y).abs() < 1e-6;
        let open: Vec<&Vec<Pos2>> = mint
            .iter()
            .filter(|(p, closed)| !closed && !is_tick(p))
            .map(|(p, _)| *p)
            .collect();
        println!(
            "{} mint strokes in Bob's own frame, {} of them open pieces of his worldline and {}              ticks of his own clock; his marker is at {at:?}",
            mint.len(),
            open.len(),
            mint.iter().filter(|(p, _)| is_tick(p)).count()
        );
        assert!(
            !open.is_empty(),
            "his own worldline is drawn in his own frame: {:?}",
            mint.iter().map(|(p, c)| (p.len(), c)).collect::<Vec<_>>()
        );
        assert!(
            open.iter().any(|p| vertical(p) && (p[0].x - at.x).abs() < 0.5),
            "it is the vertical through his marker at x = {}, but the mint strokes start at {:?}",
            at.x,
            open.iter().map(|p| p[0]).collect::<Vec<_>>()
        );
        // And nothing of his leans. A closed mint polyline would be a rim of some kind; there are
        // none, because his cone is drawn in its own light blue, so every open mint stroke that is
        // not a tick of his clock is a piece of the axis.
        for p in open.iter() {
            assert!(
                vertical(p),
                "every stroke of his worldline is vertical in his own rest frame, but one runs \
                 from {:?} to {:?}",
                p.first(),
                p.last()
            );
        }
    }

    #[test]
    fn test_in_a_rest_frame_the_past_cone_and_pulse_surfaces_are_drawn_through_the_chart() {
        // Both surfaces used to be suppressed in a rest frame, because the tetrad's own
        // (Delta t, Delta r, Delta phi) chart turned every locus of events several M across into a
        // sheet with no shape in it. Through the embedding they are the same surfaces the global
        // foliation draws, sheared: the past cone is a cone, and a pulse's sheet is a sheet. So
        // both are drawn in both charts, from the same `chart.world` and the same per-cell drop -
        // and the past cone is built in both, since it is the drawing rather than the integration
        // that the chart changes.
        let metric = KerrSchild::new(1.0, 0.65);
        let (field, emitter) = emitting_field(&metric);
        assert!(
            field.pulses.iter().any(|p| p.history().is_some()),
            "the run must leave a tagged pulse in flight for there to be a surface to draw"
        );
        let idle = SignalField::default();
        let ramp = front_ramp(FRONT_SURFACE_ALPHA);
        let ramp_alpha = Theme::front_colour(1.0, FRONT_SURFACE_ALPHA).a();
        let past_fill = Theme::cone_colours_at("Bob", Theme::VOLUME_CONE_FILL_ALPHA / 2).1;
        let in_ramp = |colours: &[Color32]| {
            !colours.is_empty()
                && colours.iter().all(|c| c.a() == ramp_alpha && ramp.contains(&c.to_array()))
        };

        let frame = |frame_of_ref| {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: true,
                show_pulse_surfaces: true,
                keep_surface_framed: false,
                ..Default::default()
            };
            assert!(canvas.past_cone.is_none(), "the cache starts empty");
            let shapes = volume_frame_signals(
                &mut canvas,
                &metric,
                Some(&emitter),
                frame_of_ref,
                false,
                &idle,
                &field,
            );
            (shapes, canvas.past_cone.is_some())
        };

        for frame_of_ref in [ReferenceFrame::Bob, ReferenceFrame::DistantObserver] {
            let (shapes, built) = frame(frame_of_ref);
            let cone = shapes
                .iter()
                .filter(|s| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == past_fill))
                .count();
            let sheets = shapes
                .iter()
                .filter(|s| matches!(s, Painted::Mesh { colours, .. } if in_ramp(colours)))
                .count();
            println!(
                "{frame_of_ref:?}: {} shapes, cone built: {built}, {cone} chunks of past-cone \
                 surface and {sheets} of pulse surface",
                shapes.len()
            );
            assert!(built, "{frame_of_ref:?}: the past cone is integrated in either chart");
            assert!(
                cone > 0,
                "{frame_of_ref:?}: the past cone is painted in its own half-alpha fill \
                 {past_fill:?}"
            );
            assert!(
                sheets > 0,
                "{frame_of_ref:?}: the tagged pulse's light cone is painted in the gain ramp"
            );
        }

        // And both are still the user's to switch off, since near the stall either can wash over
        // the geometry a rest frame is being looked at for.
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            show_pulse_surfaces: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let off = volume_frame_signals(
            &mut canvas,
            &metric,
            Some(&emitter),
            ReferenceFrame::Bob,
            false,
            &idle,
            &field,
        );
        assert!(
            !canvas.past_cone.is_some(),
            "with the switch off the cone is not integrated at all, which is the expensive half"
        );
        assert!(
            !off.iter()
                .any(|s| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == past_fill)),
            "and neither surface is painted"
        );
        assert!(
            !off.iter().any(|s| matches!(s, Painted::Mesh { colours, .. } if in_ramp(colours))),
            "and neither surface is painted"
        );
    }

    #[test]
    fn test_in_a_rest_frame_the_patches_cover_the_canvas() {
        // A patch is a square drawn in a plane the camera is free to spin, so there is no
        // orientation its own corners can be kept out of the picture by - except by making it
        // bigger than the canvas at every one of them. Half the diagonal is the radius of the disc
        // the canvas is inscribed in; the overscan puts the corners outside that.
        let rect = egui::Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));
        let half = patch_half_width(rect, 48.0);
        let expected = PATCH_OVERSCAN * f64::from(400.0f32.hypot(300.0)) / 48.0;
        println!("800 x 600 at 48 px/M: half-width {half} M against {expected} M");
        assert!(
            (half - expected).abs() < 1e-12,
            "the patch half-width is the overscan times the rect's half-diagonal in M: {half} \
             against {expected}"
        );
        assert!(
            half > 14.0,
            "which is wider than the 14 M time window the patch used to be sized to: {half}"
        );

        // What the half-width is spent on, now that the surfaces r = const are pipes and the
        // distant clock's slices are lines: the reach of each of those lines either side of its
        // anchor, and the patch of a wall that has fallen back to its tangent plane. At an ordinary
        // zoom no wall falls back, so turning the clock on adds strokes and no mesh: the four pipes'
        // strips are the only quads either way.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let quads = |grid: bool| {
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                keep_surface_framed: false,
                ..Default::default()
            };
            let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, grid);
            shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count()
        };
        let (bare, clocked) = (quads(false), quads(true));
        println!("{bare} four-cornered meshes with the clock off, {clocked} with it on");
        assert_eq!(
            bare,
            4 * RING_SEGMENTS,
            "with the clock off the only quads are the four pipes' {RING_SEGMENTS} strips apiece"
        );
        assert_eq!(clocked, bare, "and the clock adds none: its slices are lines");
    }

    #[test]
    fn test_auto_zoom_frames_the_next_surface_in_a_rest_frame() {
        // The flat rest-frame diagram's own framing rule, brought across: the surface the observer
        // is about to meet crosses their own time axis at the proper time they have left, and
        // `framed_window` turns that into the window in M the picture has to hold. Here it is
        // spent on the height of the canvas, because in the volume the observer's own clock runs
        // up the screen.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 3.0);
        for k in 1..=8 {
            bob.step(&metric, (k as f64) * 0.2, 0.2);
        }
        let signal = SignalField::default();

        let frame = |framed: bool| {
            let ctx = egui::Context::default();
            ctx.set_fonts(egui::FontDefinitions::empty());
            let mut canvas = VolumeCanvas {
                camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
                show_past_cone: false,
                keep_surface_framed: framed,
                ..Default::default()
            };
            let (_, rect) = volume_frame_raw(
                &mut canvas,
                &ctx,
                &metric,
                Some(&bob),
                ReferenceFrame::Bob,
                false,
                &signal,
                &signal,
                input(),
            );
            (canvas.camera.scale, rect)
        };

        let (scale, rect) = frame(true);
        let window = SpacetimeCanvas::framed_window(&metric, &bob, rect)
            .expect("a falling observer has a surface ahead of them to frame");
        let expected = (f64::from(rect.height()) * 0.4 / window) as f32;
        println!(
            "Bob at r = {:.4}: the window ahead is {window:.4} M, so the zoom is {scale} px/M \
             against {expected}",
            bob.r
        );
        assert!(
            (scale - expected).abs() < 1e-3,
            "the zoom puts that window across four tenths of the canvas height: {scale} against \
             {expected}"
        );

        let (parked, _) = frame(false);
        assert_eq!(parked, 48.0, "and with the framing off the zoom is the user's own");
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
                ReferenceFrame::DistantObserver,
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
