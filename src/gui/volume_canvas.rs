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
use crate::physics::local_frame::LocalFrame;
use crate::physics::observer::Observer;
use crate::physics::wavefront::{NullRay, RaySample, SignalField};
use egui::{Color32, Pos2, Stroke, Vec2};
use std::time::{Duration, Instant};

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
/// diagram; `EdgeOn` is exactly that, the eye in the floor looking along +y; `ThreeQuarter` is the
/// one that shows a cone as a cone.
///
/// `EdgeOn` earns its place in a rest frame. There the drawn axes are (xi^1, xi^2, xi^0) and every
/// surface r = const is a plane containing the xi^2 direction (the axial gauge of
/// `Tetrad::from_four_velocity_axial` puts e2^r = 0), so a line of sight along xi^2 lies *in* every
/// one of those planes and each collapses to a line whose slope is its causal character: the
/// picture is the flat rest-frame diagram's (xi^1, xi^0) plane, with the light cone at exactly 45
/// degrees either side of the axis. Any other pitch tilts the planes open into bands.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Preset {
    Top,
    Side,
    EdgeOn,
    ThreeQuarter,
}

impl Preset {
    /// The (yaw, pitch) this preset puts the eye at. `Side` is not exactly edge-on: at pitch 0 the
    /// floor is a single line and every worldline crossing the hole lands on top of every other, so
    /// it is tilted just far enough that near and far are distinguishable. `EdgeOn` is pitch 0 on
    /// purpose, for the reason the type's doc gives.
    fn angles(self) -> (f32, f32) {
        match self {
            Self::Top => (0.0, std::f32::consts::FRAC_PI_2),
            Self::Side => (0.0, 0.17),
            Self::EdgeOn => (0.0, 0.0),
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
/// numbers; a tangent plane of the local chart leans in every direction there is, so the rest-frame
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
/// which is what makes it the right place to read a horizon off. `Local` is the focus observer's
/// own first-order inertial chart, the same one `gui::spacetime_canvas::render_observer_frame`
/// draws in two dimensions, lifted into three: the dual tetrad xi^a = e^a_mu Delta x^mu of
/// `LocalFrame`, with the drawn axes (xi^1, xi^2, xi^0 t_scale).
///
/// Everything the two charts disagree about follows from the map being linear and the tetrad being
/// orthonormal. In `Local` the focus observer's worldline is the vertical axis, their light cone is
/// the exact 45 degree circular cone, a surface r = const is a *plane* whose tilt is its causal
/// character, and the distant clock's slices are a stack of planes that crowd into the past cone as
/// u^t runs away. In `Global` a surface r = const is a pipe and a cone leans over instead. Neither
/// is a drawing rule: both come out of the same geometry read in two charts.
///
/// It is a first-order chart. The orientations at the focus observer's own event - the tilt of
/// every cone, the causal character of every surface - are exact; finite offsets are the linearised
/// answer. The legend says so.
enum Chart {
    Global {
        /// The simulation clock the floor stands at, so that height is t - t_now.
        t_now: f64,
    },
    Local {
        frame: LocalFrame,
        t0: f64,
        r0: f64,
        phi0: f64,
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
            Self::Local { frame, t0, r0, phi0 } => {
                // The azimuth is a difference on a circle: an observer three turns round the hole
                // from the focus is not three turns' worth of local distance away, they are next
                // door, and the shortest offset is the one the chart is linearised about.
                let xi = frame.to_local(&[t - t0, r - r0, wrap_pi(phi - phi0)]);
                [xi[1], xi[2], xi[0] * t_scale]
            }
        }
    }

    /// Where a coordinate vector k^mu carried at the event (r, phi) points, as a world direction
    /// scaled so that its vertical component is exactly `t_scale`: one unit of the chart's own time
    /// per unit of the parameter, which is what makes `apex + span * direction` a rim at `span` of
    /// that time.
    ///
    /// In `Global` this is `KerrSchild::cartesian_velocity` of the coordinate slopes, which is the
    /// composition `light_cone_generators` performs. In `Local` it is the same linear map `world`
    /// uses, applied to a vector rather than to a displacement - so a null k stays null, and the rim
    /// of the cone it generates is the unit circle at 45 degrees.
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
            Self::Local { frame, .. } => {
                let v = frame.vector_to_local(k);
                [v[1] / v[0], v[2] / v[0], t_scale]
            }
        };
        finite3(d).then_some(d)
    }

    fn is_local(&self) -> bool {
        matches!(self, Self::Local { .. })
    }
}

/// The shortest signed azimuthal offset, in (-pi, pi].
fn wrap_pi(d_phi: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (d_phi + PI).rem_euclid(TAU) - PI
}

/// Whether a world point can be projected at all. A frozen observer's frame carries u^t ~ 1e10, and
/// a displacement mapped through it can be enormous or not a number; `Camera::project` rounds to
/// f32, so the scene drops such a primitive rather than pushing an infinity into a mesh.
fn finite3(p: [f64; 3]) -> bool {
    p[0].is_finite() && p[1].is_finite() && p[2].is_finite()
}

/// One tangent plane of the local chart, ready to be drawn as a square patch.
///
/// A surface of the spacetime maps to the affine plane n_a xi^a = d under the dual tetrad, with the
/// normal read off the tetrad legs - n_a = e_a^r for a surface r = const, n_a = e_a^t for a slice of
/// the distant clock. That is the whole content of the drawing: the numbers come from the geometry
/// and the patch is only how much of an infinite plane fits on the screen.
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

/// How many quads a tangent plane's patch is cut into along each side.
///
/// A plane cannot be one mesh: it runs from the bottom of the window to the top, so a single
/// primitive sorted at one depth would interleave wrongly with every worldline and cone it passes
/// through, and its centroid would place the whole surface in one layer when half of it is in the
/// observer's past and half in their future. Eight a side is 64 pieces, each short enough that its
/// own centroid is a fair place to sort and to layer it, and cheap enough that four surfaces cost
/// less than one pipe's 72 strips did.
const PLANE_CELLS: usize = 8;

/// How far past the corner of the canvas a tangent plane's patch reaches, as a multiple of the
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

/// Half-width of a tangent plane's patch, in M of the local chart, for a canvas of `rect` at
/// `scale` pixels per M. See `PATCH_OVERSCAN`.
fn patch_half_width(rect: egui::Rect, scale: f32) -> f64 {
    let half_diagonal = f64::from((rect.width() * 0.5).hypot(rect.height() * 0.5));
    PATCH_OVERSCAN * half_diagonal / f64::from(scale).max(1e-6)
}

/// The same for one slice of the distant clock. Coarser, because there are many of them and each is
/// a faint wash rather than a surface to be read against.
const CLOCK_PLANE_CELLS: usize = 4;

/// Opacity of one slice of the distant clock drawn as a plane.
const CLOCK_PLANE_ALPHA: u8 = 30;

/// At most this many slices of the distant clock, either side of the observer's own now.
///
/// The bound that actually decides the count is geometric - a plane whose nearest point is further
/// from the origin than the patch is wide has nothing on screen - and this is the backstop for the
/// case where the step has collapsed relative to the window.
const CLOCK_PLANE_MAX_K: i64 = 100;

/// How close two neighbouring slices of the distant clock may be drawn, in pixels, before their
/// labels are dropped.
///
/// The planes themselves stay: a wall of them piling into the past cone *is* the picture on the
/// approach to the far branch of r-. What cannot survive it is the text, which at that spacing is a
/// solid block of overlapping glyphs, so the labels go and the geometry stays.
const CLOCK_LABEL_MIN_PX: f32 = 14.0;

/// How many segments a pipe wall, a floor ring and a tick ring are each cut into. Seventy-two is
/// five degrees a segment: at the zoom the view opens on, a chord of five degrees departs from the
/// circle it stands for by well under a pixel, and the pipe's shading needs one strip per segment
/// rather than one vertex, so the count is also the mesh budget of every surface in the scene.
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

/// While the focus event keeps moving, at most one rebuild of the past cone per this many
/// milliseconds.
///
/// This is what keeps play smooth. A build is a few ms of integration - 36 rays times a couple of
/// hundred steps - which is nothing once and is the whole frame budget sixty times a second. So a
/// cone whose key has gone stale but which was built less than this ago is drawn again as it
/// stands, and the frame asks for a repaint after the throttle expires so the fresh one arrives
/// without the user having to touch anything. Paused, the key is stable and it builds exactly once.
const PAST_CONE_REBUILD_MS: u128 = 150;

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

/// Why the two extended surfaces are greyed out in a rest frame.
const GLOBAL_SURFACES_ONLY_TIP: &str =
    "Drawn in the global foliation only. Both surfaces are loci of events several M away from the \
     focus event, and a rest frame places those through a chart that is exact at that event and \
     linearised everywhere else - at the boosts of a late fall the linearisation of an offset that \
     size paints as a wash over the whole canvas rather than as a surface.";

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
    /// When this cone was integrated, for the rebuild throttle.
    built: Instant,
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
fn build_past_cone(metric: &KerrSchild, obs: &Observer, t_min: f64) -> PastCone {
    let tetrad = Observer::raindrop_tetrad(metric, obs.r);
    let u = tetrad.e0;
    let mut rays = Vec::with_capacity(PAST_CONE_RAYS);
    for i in 0..PAST_CONE_RAYS {
        let alpha = std::f64::consts::TAU * (i as f64) / (PAST_CONE_RAYS as f64);
        let mut ray =
            NullRay::from_local_direction(metric, obs.t, obs.r, obs.phi, &tetrad, alpha, &u);
        let mut samples = vec![[ray.t, ray.r, ray.phi]];
        while ray.alive() && ray.t > t_min {
            ray.step_back(metric, PAST_CONE_DT);
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
        built: Instant::now(),
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
    /// `PastConeKey` and `PAST_CONE_REBUILD_MS`.
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
                    let new_scale = (old_scale * zoom_mult).clamp(8.0, 500_000.0);
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
                self.camera.scale = scale.clamp(8.0, 500_000.0);
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
                finite3(u).then(|| Chart::Local {
                    frame: LocalFrame::for_observer(metric, obs.r, &u),
                    t0: obs.t,
                    r0: obs.r,
                    phi0: obs.phi,
                })
            })
            .unwrap_or(Chart::Global { t_now: current_time });
        // In a rest frame the focus observer *is* the origin of the chart, so there is nothing to
        // follow: the offset that keeps them in the middle is zero, and the view is anchored on
        // them by construction rather than by a pan.
        let offset = if chart.is_local() {
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
            Chart::Local { .. } => {
                let (r, phi) = metric.chart_point(x, y, RING_DROP_FLOOR);
                camera.project(centre, chart.world(metric, current_time, r, phi, t_scale)).0
            }
        };
        // Height is a coordinate-time difference in M, kept in f64 the whole way into `project`,
        // which takes the one rounding to f32 it needs. Near r- the interesting times differ from
        // t_now by parts in 1e7 of t_now itself, and taking the difference in f32 would quantise
        // the whole late fall onto one height.
        let z_of = |t: f64| (t - current_time) * t_scale;
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;
        let view_d = camera.view_direction();

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();
        let seg_angle = |i: usize| std::f64::consts::TAU * (i as f64) / (RING_SEGMENTS as f64);
        let ring_points = |rho: f64, z: f64| -> Vec<Pos2> {
            (0..RING_SEGMENTS)
                .map(|i| {
                    let th = seg_angle(i);
                    project([rho * th.cos(), rho * th.sin(), z]).0
                })
                .collect()
        };

        // A world point from a chart point, and the drawer for one tangent plane: both live here,
        // where the camera and the view direction are, and both are used by the surfaces of
        // constant r and by the distant clock's slices alike.
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
                    if !corners.iter().all(|c| finite3(*c)) {
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

        // 4. The surfaces of constant r: glass pipes in the global chart, tangent planes in a rest
        // frame, and in both a mark on the floor where they cross it.
        //
        // A pipe is the honest picture of what r = const is in the *global* chart: not a circle a
        // worldline happens to cross, but a wall standing in time, so that "Bob went through r+" is
        // a worldline entering a tube and never leaving it. The wall is only drawn below the floor,
        // over the past the simulation has actually integrated; above it there are rings alone,
        // because the future of a horizon is not something this run has computed and a solid wall
        // up there would claim it had.
        //
        // In a rest frame the same surface is a plane, because the chart is linear: the condition
        // for a displacement to stay on it is n_a xi^a = r_h - r_obs with n_a = e_a^r, the
        // r-components of the tetrad legs (see `LocalFrame::surface_r_const`, which is this plane's
        // two-dimensional slice). Its tilt is then its causal character and not a drawing rule:
        // steeper than 45 degrees where g^rr > 0 and the surface can be hovered at, exactly 45 on
        // either horizon, flatter where it is spacelike and lies wholly in a future or a past. The
        // trace it leaves on the floor is drawn in the full ring colour, so the eye still finds the
        // surfaces without having to read the glass.
        let z_bottom = z_of(t_min);
        let tick_r = if metric.cartesian_radius(rm) > 0.0 { rm } else { rp };
        let mut floor_rings: Vec<(Vec<Pos2>, Stroke, bool)> = Vec::new();
        let surfaces = [
            (0.0, Theme::SINGULARITY_LINE, 70u8, 2.0f32),
            (rm, Theme::HORIZON_CAUCHY, 60, 2.0),
            (rp, Theme::HORIZON_OUTER, 60, 2.5),
            (re, Theme::ERGOSPHERE_LINE, 35, 1.5),
        ];
        if let Chart::Local { frame, r0, .. } = &chart {
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
            // n_a = e_a^r, in the chart's own (xi^0, xi^1, xi^2) order.
            let n = [tetrad.e0[1], tetrad.e1[1], tetrad.e2[1]];
            for (r_h, colour, base_alpha, width) in surfaces {
                let Some(plane) = local_plane(n, r_h - r0) else {
                    continue;
                };
                let trace = push_plane(
                    &mut buf,
                    &plane,
                    half,
                    PLANE_CELLS,
                    colour,
                    (base_alpha, base_alpha / 2),
                );
                if let Some(ends) = trace {
                    floor_rings.push((ends.to_vec(), Stroke::new(width, colour), false));
                }
            }

            // The distant observer's clock, as a stack of planes rather than as rungs on one pipe.
            //
            // A slice t = const of the chart's Killing time is the set of events that far-away
            // clock labels with one reading, and in this frame it is the plane e_a^t xi^a = dt. The
            // step between neighbours is picked by the same rule the flat rest-frame diagram uses -
            // a round unit of the observer's *own* clock, carried across by u^t - so the grid keeps
            // a constant pixel pitch while the outside clock runs away. On the approach to the far
            // branch of r- u^t grows like exp(kappa_- t), and what that does to this picture is the
            // whole point of drawing it: the planes crowd into the observer's past cone without
            // limit, so infinitely many of the distant clock's moments are crossed in a finite
            // amount of their own time. Every one of these planes is flatter than 45 degrees, in
            // every region, because dt is timelike everywhere in this chart.
            if show_distant_clock_grid {
                let n_t = [tetrad.e0[0], tetrad.e1[0], tetrad.e2[0]];
                let n_len = (n_t[0] * n_t[0] + n_t[1] * n_t[1] + n_t[2] * n_t[2]).sqrt();
                let seconds_per_m = metric.t_grav_seconds() / metric.m.max(1e-12);
                let step =
                    distant_clock_grid_step(tetrad.e0[0], camera.scale, seconds_per_m, font_scale)
                        .step_m;
                if step.is_finite() && step > 0.0 && n_len.is_finite() && n_len > 0.0 {
                    // A plane whose nearest point is further from the origin than the patch is wide
                    // has nothing on screen: |k step| / |n| > W. That is the bound, and the count
                    // cap behind it is only a backstop.
                    let reach = half * n_len / step;
                    let k_max = if reach.is_finite() {
                        (reach.floor() as i64).clamp(0, CLOCK_PLANE_MAX_K)
                    } else {
                        CLOCK_PLANE_MAX_K
                    };
                    // How far apart two neighbouring slices land on the screen, measured between
                    // the nearest points of k = 0 and k = 1. Below a glyph's width the labels are a
                    // solid block, so they are dropped and the planes go on without them.
                    let label = local_plane(n_t, step).is_some_and(|one| {
                        let a = camera.project(centre, xi_to_world([0.0, 0.0, 0.0])).0;
                        let b = camera.project(centre, xi_to_world(one.nearest)).0;
                        a.distance(b) >= CLOCK_LABEL_MIN_PX * font_scale
                    });
                    for k in -k_max..=k_max {
                        let dt = (k as f64) * step;
                        let Some(plane) = local_plane(n_t, dt) else {
                            continue;
                        };
                        push_plane(
                            &mut buf,
                            &plane,
                            half,
                            CLOCK_PLANE_CELLS,
                            Theme::GRID_LINE,
                            (CLOCK_PLANE_ALPHA, CLOCK_PLANE_ALPHA),
                        );
                        if !label {
                            continue;
                        }
                        let at = xi_to_world(plane.nearest);
                        if !finite3(at) {
                            continue;
                        }
                        // A patch that covers the canvas hangs its label wherever its nearest
                        // point happens to project to, and that is often off the edge or under
                        // the legend in the top-left corner. Neither is a caption anybody can
                        // read, so neither is drawn.
                        let at = camera.project(centre, at).0;
                        if !rect.contains(at) || legend_rect.contains(at) {
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
            }
        }
        let pipes: &[(f64, Color32, u8, f32)] = if chart.is_local() { &[] } else { &surfaces };
        for (r, colour, base_alpha, width) in pipes.iter().copied() {
            // A surface of constant r is the circle of Cartesian radius sqrt(r^2 + a^2); r = 0 is
            // the ring, at rho = |a|, and a hole with no spin has no ring and so no pipe there.
            let rho = metric.cartesian_radius(r);
            if rho <= 0.0 {
                continue;
            }
            for i in 0..RING_SEGMENTS {
                let (th0, th1) = (seg_angle(i), seg_angle(i + 1));
                let mid = 0.5 * (th0 + th1);
                let weight = rim_weight((mid.cos() as f32, mid.sin() as f32), view_d);
                let (c0, s0) = (rho * th0.cos(), rho * th0.sin());
                let (c1, s1) = (rho * th1.cos(), rho * th1.sin());
                let (mesh, depth) = quad_mesh(
                    &camera,
                    centre,
                    [[c0, s0, z_bottom], [c0, s0, 0.0], [c1, s1, 0.0], [c1, s1, z_bottom]],
                    glass(colour, base_alpha, weight),
                );
                buf.push(Layer::Below, depth, Prim::Mesh(mesh));
            }
            floor_rings.push((ring_points(rho, 0.0), Stroke::new(width, colour), true));

            // The distant observer's clock, as rungs on one pipe: one ring per whole M of t. On
            // r-, because that is the pipe a frozen worldline winds up, one turn of helix per
            // 2 pi / Omega_- of t, and the rungs are what that pitch is read against; nothing runs
            // away at r+ in this chart, a faller crosses it at a finite t. The same ladder on all
            // four pipes would be three ladders saying nothing and one saying that. A hole with no
            // spin has no r- pipe, and the rungs go on r+ instead.
            if show_distant_clock_grid && r == tick_r {
                let k_lo = (t_min - current_time).ceil() as i64;
                let k_hi = (t_max - current_time).floor() as i64;
                for k in k_lo..=k_hi {
                    let z = z_of(current_time + k as f64);
                    let layer = if z >= 0.0 { Layer::Above } else { Layer::Below };
                    let depth = project([0.0, 0.0, z]).1;
                    buf.push(
                        layer,
                        depth,
                        Prim::Line {
                            points: ring_points(rho, z),
                            stroke: Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
                            closed: true,
                        },
                    );
                    let text = if use_km {
                        let sign = if k < 0 { "-" } else { "+" };
                        format!("t = {sign}{}", metric.format_physical_time(k.abs() as f64))
                    } else {
                        format!("t = {k:+} M")
                    };
                    buf.label(
                        project([rho, 0.0, z]).0,
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
            .is_local()
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
                let head = [0.0, 0.0, z_of(t_max)];
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
                // the worldline is still the worldline.
                .filter(|(_, p)| finite3(*p))
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
                buf.push(
                    Layer::Below,
                    depth,
                    Prim::Line {
                        points: run.iter().map(|(_, p)| project(*p).0).collect(),
                        stroke: Stroke::new(width, colour_of(who).gamma_multiply(fade)),
                        closed: false,
                    },
                );
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
                             fills: (Color32, Color32, Color32),
                             ghost: bool| {
            let tetrad = Observer::raindrop_tetrad(metric, r);
            let apex = chart.world(metric, t_at, r, phi, t_scale);
            if !finite3(apex) {
                return;
            }
            // The rim is the 36 null directions at the event, each carried a fixed span of the
            // chart's own time by the map the rest of the scene is drawn with. In the global chart
            // that is `light_cone_generators`' own composition and the cone leans over as the
            // geometry says; in a rest frame the same null vectors stay null under a linear map,
            // so the rim comes out as the unit circle and the cone is at exactly 45 degrees.
            let mut future: Vec<[f64; 3]> = Vec::with_capacity(CONE_SAMPLES);
            for i in 0..CONE_SAMPLES {
                let alpha = std::f64::consts::TAU * (i as f64) / (CONE_SAMPLES as f64);
                let Some(d) = chart.direction(metric, r, phi, &tetrad.null_direction(alpha), t_scale)
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
            for (rim, fill, mean_z, fixed) in [
                (&future, fills.0, (z0 + n * (z0 + rise)) / (n + 1.0), Layer::Above),
                (&past, fills.1, (z0 + n * (z0 - rise)) / (n + 1.0), Layer::Below),
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
                buf.push(layer, depth, Prim::Mesh(mesh));
                buf.push(
                    layer,
                    depth,
                    Prim::Line {
                        points: rim.iter().map(|p| project(*p).0).collect(),
                        stroke: Stroke::new(1.0, fills.2),
                        closed: true,
                    },
                );
            }
        };
        for (obs, _) in present.iter().copied() {
            let (future_fill, past_fill, edge) =
                Theme::cone_colours_at(&obs.name, Theme::VOLUME_CONE_FILL_ALPHA);
            push_cone(&mut buf, current_time, obs.r, obs.phi, (future_fill, past_fill, edge), false);
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
                        (
                            future_fill.gamma_multiply(0.4),
                            past_fill.gamma_multiply(0.4),
                            edge.gamma_multiply(0.4),
                        ),
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
        // The focus is whoever the view is anchored on, falling back to Bob and then Alice, so the
        // cone is drawn in the global foliation too - it is a fact about an event, not about a
        // choice of frame, and the view drawn from nobody's rest frame is the one where that is
        // easiest to say.
        //
        // It is drawn *only* there. The surface is a locus of events many M away from the focus
        // event, and a rest frame places those through a linear map that is exact at that event and
        // linearised everywhere else; at the u^t ~ 1e5 of a late fall the linearisation of a 10 M
        // offset is a number with no picture in it, and the surface arrives as a canvas-wide wash
        // that hides the geometry the rest frame is being looked at for. So in `Chart::Local` the
        // cone is not drawn - and not built either, since building it is the expensive half.
        // `past_cone` is left exactly as it stands, so switching back to the global foliation
        // redraws the cached surface rather than integrating it again.
        let cone_focus = (self.show_past_cone && !chart.is_local())
            .then(|| focus.or(bob).or(alice))
            .flatten();
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
            if self.past_cone.as_ref().map(|c| &c.key) != Some(&key) {
                // Stale - but a rebuild is a few ms, and doing one on every frame of a played
                // infall would cost more than everything else this view draws put together. So a
                // cone built inside the throttle is drawn again as it stands and the fresh one is
                // asked for by a timed repaint, which arrives whether or not the user touches
                // anything. Paused, the key stops changing and the first build is the only one.
                match self.past_cone.as_ref().map(|c| c.built.elapsed()) {
                    Some(since) if since.as_millis() < PAST_CONE_REBUILD_MS => {
                        let throttle = Duration::from_millis(PAST_CONE_REBUILD_MS as u64);
                        ui.ctx().request_repaint_after(throttle.saturating_sub(since));
                    }
                    _ => self.past_cone = Some(build_past_cone(metric, obs, t_min)),
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
                            // surface there, exactly as a generator that died at the ring does.
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
                    let points: Vec<[f64; 3]> = cone.rays[i]
                        .iter()
                        .map(|s| to_world(*s))
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
        // The cost is bounded by the physics module's own caps and by the window: at most
        // `MAX_PULSES / HISTORY_PULSE_STRIDE` = 8 tagged pulses per field, 24 kept rays each and
        // `HISTORY_MAX_ROWS` = 256 rows, so 8 x 24 x 255 x 2 = 98 k triangles per field and under
        // 200 k for both - the worst case the caps allow, against a typical window holding a
        // hundred rows of two or three tagged pulses.
        //
        // In the global foliation only, for the reason section 7 gives: every row of a history is
        // a front several M across, and a rest frame's linear map is trusted at the focus event
        // rather than out there.
        if self.show_pulse_surfaces && !chart.is_local() {
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
        // The region fills are a picture of one slice t = const, and in a rest frame the floor
        // xi^0 = 0 is not one: it is the observer's own local space, tilted against every slice of
        // the chart's time there is. Painting the equatorial regions on it would be drawing one
        // slicing on another, so in a rest frame the floor carries only what belongs to it - the
        // traces of the tangent planes, and the fronts, which are placed event by event.
        if !chart.is_local() {
            let ring_rho = metric.cartesian_radius(0.0).max(2.0 / f64::from(camera.scale));
            for (rho, fill) in [
                (metric.cartesian_radius(re), Theme::ERGOSPHERE_FILL),
                (metric.cartesian_radius(rp), Theme::REGION_II_FILL),
                (metric.cartesian_radius(rm), Theme::REGION_III_FILL),
                (ring_rho, Theme::SINGULARITY_FILL),
            ] {
                painter.add(egui::Shape::convex_polygon(ring_points(rho, 0.0), fill, Stroke::NONE));
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
        if !chart.is_local() {
            if let Some(al) = alice {
                draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &floor);
            }
            if let Some(b) = bob {
                draw_spatial_trail(&painter, metric, b, Theme::BOB_COLOR, 1.5, &floor);
            }
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
            // In a rest frame the focus observer's own marker is the origin of the chart, which is
            // the one place in the picture that needs no calculation at all.
            let world = chart.world(metric, current_time, obs.r, obs.phi, t_scale);
            if !finite3(world) {
                continue;
            }
            let at = project(world).0;
            painter.circle_filled(at, who.marker_radius(), colour_of(who));
            markers.push((who, at));
            // A worldline frozen on the far branch of r- has not stopped: it is riding the
            // horizon's own null generator, so in the volume it is a helix wound onto the r- pipe
            // while the marker creeps round the ring at Omega_-. The label goes into the
            // buffer rather than onto the painter so that it paints after every pipe, and no glass
            // wall standing between the eye and the marker can swallow it.
            if obs.is_frozen() {
                buf.label(
                    at + Vec2::new(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    "Frozen: gliding on the r₋ generator at Ω₋",
                    Theme::TEXT_MUTED,
                );
            }
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
                let can_centre = in_run && !chart.is_local();
                if ui.add_enabled(can_centre, egui::Checkbox::new(&mut centred, label)).changed() {
                    self.centred_on = centred.then_some(who);
                    ui.close();
                }
            }
            ui.separator();
            if ui.checkbox(&mut self.show_ghost_cones, "Ghost cones along the trail").changed() {
                ui.close();
            }
            // Both surfaces are built from coordinate offsets of many M, which a rest frame's
            // first-order chart cannot place, so both are drawn in the global foliation alone and
            // the menu says why rather than offering a switch that does nothing.
            let global_only = !chart.is_local();
            if ui
                .add_enabled(
                    global_only,
                    egui::Checkbox::new(
                        &mut self.show_past_cone,
                        "Exact past cone of the focus event",
                    ),
                )
                .on_disabled_hover_text(GLOBAL_SURFACES_ONLY_TIP)
                .changed()
            {
                ui.close();
            }
            if ui
                .add_enabled(
                    global_only,
                    egui::Checkbox::new(
                        &mut self.show_pulse_surfaces,
                        "Light-cone surfaces of every 8th pulse",
                    ),
                )
                .on_disabled_hover_text(GLOBAL_SURFACES_ONLY_TIP)
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
        // looking at a picture, and one more that undoes an exploration: Reset puts the zoom, the
        // pan and the time scale back where `Camera::default` has them and leaves the eye where the
        // user has moved it, which is the one part of the view they chose on purpose.
        let legend_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let buttons = [
            ("Top", Some(Preset::Top)),
            ("Side", Some(Preset::Side)),
            ("Edge", Some(Preset::EdgeOn)),
            ("3/4", Some(Preset::ThreeQuarter)),
            ("Reset", None),
        ];
        let button_size = Vec2::new(40.0 * font_scale, 16.0 * font_scale);
        let gap = 4.0 * font_scale;
        let strip = button_size.x * buttons.len() as f32 + gap * (buttons.len() - 1) as f32;
        // The buttons sit in a named, outlined box, because five bare words in the corner of a
        // picture read as part of the picture: the frame is what says they are controls, and the
        // heading is what says which.
        let pad = 6.0 * font_scale;
        let heading_h = Theme::MIN_FONT_PT * font_scale + 2.0;
        let box_size = Vec2::new(strip + 2.0 * pad, pad + heading_h + gap + button_size.y + pad);
        let box_rect = egui::Rect::from_min_size(
            rect.right_top() + Vec2::new(-8.0 - box_size.x, 6.0),
            box_size,
        );
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
        for (i, (label, preset)) in buttons.into_iter().enumerate() {
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
        let chart_name = chart.is_local().then(|| frame_obs.map(|obs| obs.name.as_str())).flatten();
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
                 below the floor: the past · above: the future · {}: r = const · cones: exact \
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
                // the linearised answer.
                match chart_name {
                    Some(name) => format!(
                        "\nfirst-order local inertial chart, exact at {name}'s event; surfaces \
                         are their tangent planes there"
                    ),
                    None => String::new(),
                },
                if chart_name.is_some() { "planes" } else { "pipes" },
                // Terse, on the end of the line that says what the other shapes are, and only when
                // the surface is actually on screen to be named - which in a rest frame neither of
                // them is.
                if self.show_past_cone && chart_name.is_none() {
                    "\npast cone: the event's null geodesics run backwards"
                } else {
                    ""
                },
                if self.show_pulse_surfaces && chart_name.is_none() {
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
        Text(String),
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
                egui::Shape::Text(t) => out.push(Painted::Text(t.galley.text().to_string())),
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
            if let Painted::Text(t) = s {
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
        let cone = build_past_cone(&metric, &bob, t_min);
        println!(
            "build_past_cone over {:.1} M at {PAST_CONE_RAYS} rays and dt = {PAST_CONE_DT}: {:?}",
            bob.t - t_min,
            started.elapsed()
        );
        let long = build_past_cone(&metric, &bob, bob.t - 10.0);
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
        let cone = build_past_cone(&metric, &bob, bob.t - 5.0);

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
        let cone = build_past_cone(&metric, &frozen, frozen.t - 10.0);
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
        // projection, so it is cached against the event it belongs to: a camera drag, a resize or
        // a paused frame must redraw the same rays rather than integrate them again. When the
        // event does move the cache has to let go, or the picture would be the past cone of where
        // the observer used to be.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut bob = bob_at(&metric, 4.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            ..Default::default()
        };

        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let first = canvas.past_cone.as_ref().expect("the first frame builds the cone").built;
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let second = canvas.past_cone.as_ref().expect("and the second keeps it").built;
        assert_eq!(
            second, first,
            "nothing about the event changed between the two frames, so the second must have drawn \
             the cone the first integrated"
        );

        // Move him, and wind the build clock back past the throttle rather than sleeping through
        // it: the throttle is a wall-clock rule and a test has no business waiting on one.
        bob.step(&metric, bob.t + 0.5, 0.5);
        canvas.past_cone.as_mut().unwrap().built -= Duration::from_secs(1);
        volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::DistantObserver, false);
        let cone = canvas.past_cone.as_ref().expect("and the third rebuilds it");
        println!(
            "Bob stepped to t = {:.3}, r = {:.4}; the cone's key followed to t = {:.3}, r = {:.4}",
            bob.t, bob.r, cone.key.t, cone.key.r
        );
        assert!(
            cone.built > first,
            "the event moved, so the cone belongs to the new one and must have been integrated \
             again"
        );
        assert_eq!(cone.key.t, bob.t, "and it is the cone of the event Bob is standing on now");
        assert_eq!(cone.key.r, bob.r);
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

    /// The r-components of the tetrad legs: the normal n_a = e_a^r of every surface r = const in
    /// this observer's chart, in the (xi^0, xi^1, xi^2) order the plane drawer takes it.
    fn radial_normal(frame: &LocalFrame) -> [f64; 3] {
        let tetrad = frame.tetrad();
        [tetrad.e0[1], tetrad.e1[1], tetrad.e2[1]]
    }

    /// The chart the volume draws in this observer's rest frame.
    fn rest_chart(metric: &KerrSchild, obs: &Observer) -> (LocalFrame, Chart) {
        let frame = LocalFrame::for_observer(metric, obs.r, &obs.four_velocity(metric));
        (frame, Chart::Local { frame, t0: obs.t, r0: obs.r, phi0: obs.phi })
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
    fn test_the_horizon_planes_pass_through_the_mapped_radial_offset() {
        // What makes a surface r = const a plane here: a displacement stays on it exactly when its
        // r-component is r_h - r_obs, and under the dual tetrad that condition is the affine
        // equation n_a xi^a = r_h - r_obs with n_a = e_a^r. So the point the drawer centres its
        // patch on, the legs it spans it with and the mapped radial offset itself all have to
        // satisfy the same equation - otherwise the patch is a plane somewhere else.
        let metric = KerrSchild::new(1.0, 0.9);
        for &r in &[8.0, 3.0, metric.outer_horizon()] {
            let bob = bob_at(&metric, r);
            let (frame, _) = rest_chart(&metric, &bob);
            let n = radial_normal(&frame);
            for r_h in [
                metric.inner_horizon(),
                metric.outer_horizon(),
                metric.ergosphere_equatorial(),
            ] {
                let d = r_h - bob.r;
                let plane = local_plane(n, d).expect("a radial normal has a length");
                let dot = |p: [f64; 3]| n[0] * p[0] + n[1] * p[1] + n[2] * p[2];
                let tol = 1e-9 * (1.0 + d.abs());
                assert!(
                    (dot(plane.nearest) - d).abs() < tol,
                    "the patch is centred off its own plane at r = {r}, r_h = {r_h}: \
                     n . xi = {} against {d}",
                    dot(plane.nearest)
                );
                let xi = frame.to_local(&[0.0, d, 0.0]);
                assert!(
                    (dot(xi) - d).abs() < tol,
                    "the mapped radial offset is off the plane at r = {r}, r_h = {r_h}: \
                     n . xi = {} against {d}",
                    dot(xi)
                );
                for (i, leg) in plane.legs.iter().enumerate() {
                    assert!(
                        dot(*leg).abs() < 1e-9,
                        "leg {i} of the patch leaves the plane: n . leg = {}",
                        dot(*leg)
                    );
                }
            }
        }
    }

    #[test]
    fn test_on_a_horizon_the_plane_is_null_in_the_rest_frame() {
        // The tilt of the plane is its causal character and nothing else. The Minkowski norm of its
        // normal is eta^{ab} n_a n_b = g^rr, so on either horizon, where Delta = 0, the normal is a
        // null covector and the plane stands at exactly 45 degrees - the surface the observer is
        // crossing is the light cone's own wall. Outside, g^rr > 0, the normal is spacelike and the
        // plane is steeper than 45 degrees: a surface that can still be hovered at.
        let metric = KerrSchild::new(1.0, 0.9);
        let rp = metric.outer_horizon();
        let minkowski = |n: [f64; 3]| -n[0] * n[0] + n[1] * n[1] + n[2] * n[2];

        let on = bob_at(&metric, rp);
        let (frame, _) = rest_chart(&metric, &on);
        let n = radial_normal(&frame);
        let norm = minkowski(n);
        println!(
            "at r = r+ = {rp:.6}: n = {n:?}, eta(n, n) = {norm:e} against g^rr = {:e}",
            metric.g_upper_rr(rp)
        );
        assert!(
            norm.abs() < 1e-9,
            "on r+ the surface r = r+ is null, so its normal is a null covector - but \
             eta(n, n) = {norm}"
        );

        let out = bob_at(&metric, 3.0);
        let (frame, _) = rest_chart(&metric, &out);
        let n = radial_normal(&frame);
        let norm = minkowski(n);
        println!("at r = 3: n = {n:?}, eta(n, n) = {norm:e} against g^rr = {:e}", metric.g_upper_rr(3.0));
        assert!(
            norm > 1e-6,
            "outside r+ the surface r = r+ is timelike, so its normal is spacelike and the plane \
             is steeper than 45 degrees - but eta(n, n) = {norm}"
        );
        assert!(
            (norm - metric.g_upper_rr(3.0)).abs() < 1e-9,
            "and that norm is g^rr exactly: {norm} against {}",
            metric.g_upper_rr(3.0)
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
            }
        }
    }

    #[test]
    fn test_in_the_rest_frame_the_distant_clocks_slices_are_drawn_as_planes() {
        // In the global chart the distant clock is a ladder of rings up one pipe. In a rest frame
        // it is what it actually is: a stack of planes, e_a^t xi^a = k * step, each of them the set
        // of events the far-away clock gives one reading. They are drawn as glass quads like every
        // other surface here, so the claim is a count - turning the grid on adds at least one whole
        // slice's worth of four-cornered meshes - together with the label on the slice through the
        // observer's own now.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let frame = |grid| {
            volume_frame(&metric, Some(&bob), Preset::ThreeQuarter, ReferenceFrame::Bob, grid)
        };
        let quads = |shapes: &[Painted]| {
            shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count()
        };

        let off = frame(false);
        let on = frame(true);
        let (n_off, n_on) = (quads(&off), quads(&on));
        println!(
            "four-cornered meshes in Bob's rest frame: {n_off} with the distant clock off, \
             {n_on} with it on ({} per slice)",
            CLOCK_PLANE_CELLS * CLOCK_PLANE_CELLS
        );
        assert!(
            n_off >= PLANE_CELLS * PLANE_CELLS,
            "the surfaces r = const are drawn as tessellated planes even with the clock off, but \
             only {n_off} quads were painted"
        );
        assert!(
            n_on >= n_off + CLOCK_PLANE_CELLS * CLOCK_PLANE_CELLS,
            "turning the distant clock on has to add at least one slice of {} quads, but the \
             count went from {n_off} to {n_on}",
            CLOCK_PLANE_CELLS * CLOCK_PLANE_CELLS
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
        // The reason the preset exists. In a rest frame every surface r = const is a plane
        // containing the xi^2 direction, so an eye looking exactly along xi^2 - which is world +y -
        // sees each of them as a line, and the picture is the flat rest-frame diagram's own
        // (xi^1, xi^0) plane. That takes pitch exactly 0, which the snapped basis has to deliver
        // as an exact +y line of sight, and it takes every point of such a plane to land on one
        // screen line whatever its xi^2.
        let cam = Camera::preset(Preset::EdgeOn, 48.0, Vec2::ZERO, 1.0);
        let (right, up, d) = cam.basis();
        assert_eq!(d, [0.0, 1.0, 0.0], "edge-on looks exactly along +y, the xi^2 axis");
        assert_eq!(up, [0.0, 0.0, 1.0], "with the observer's time straight up the screen");
        assert_eq!(right, [1.0, 0.0, 0.0], "and xi^1 across it");

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
        let (a, b) = (samples[0], samples[1]);
        let dir = if a.distance(b) > 1e-3 { b - a } else { samples[2] - a };
        for p in &samples {
            let off = ((p.x - a.x) * dir.y - (p.y - a.y) * dir.x).abs() / dir.length();
            assert!(
                off < 1e-3,
                "edge-on, every point of a tangent plane lies on one screen line, but {p:?} is \
                 {off} px off the line through {a:?} along {dir:?}"
            );
        }
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
        let open: Vec<&Vec<Pos2>> =
            mint.iter().filter(|(_, closed)| !closed).map(|(p, _)| *p).collect();
        println!(
            "{} mint strokes in Bob's own frame, {} of them open; his marker is at {at:?}",
            mint.len(),
            open.len()
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
        // none, because his cone is drawn in its own light blue, so every mint stroke here is a
        // piece of the axis.
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
    fn test_in_a_rest_frame_no_pulse_surface_or_past_cone_is_drawn() {
        // Both surfaces are loci of events several M from the focus event, and both are placed by
        // a chart that answers exactly at that event and to first order anywhere else. At the
        // boosts of a late fall that answer is a number with no picture in it, and the surface
        // arrives as a wash across the canvas. So neither is drawn in a rest frame - and the past
        // cone is not built either, which is the expensive half, nor is the cache it would have
        // gone into disturbed.
        let metric = KerrSchild::new(1.0, 0.65);
        let (field, emitter) = emitting_field(&metric);
        assert!(
            field.pulses.iter().any(|p| p.history().is_some()),
            "the run must leave a tagged pulse in flight for there to be a surface to suppress"
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

        let (shapes, built) = frame(ReferenceFrame::Bob);
        println!("{} shapes in the emitter's own rest frame, cone built: {built}", shapes.len());
        assert!(!built, "a rest frame does not integrate the past cone, so the cache is untouched");
        assert!(
            !shapes
                .iter()
                .any(|s| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == past_fill)),
            "and nothing is painted in the past cone's half-alpha fill {past_fill:?}"
        );
        assert!(
            !shapes
                .iter()
                .any(|s| matches!(s, Painted::Mesh { colours, .. } if in_ramp(colours))),
            "and nothing is painted in the wavefront gain ramp"
        );

        // The same run in the global foliation draws both, so what suppresses them is the chart
        // rather than a switch that has quietly turned itself off.
        let (global, built) = frame(ReferenceFrame::DistantObserver);
        assert!(built, "the global foliation does build the past cone");
        assert!(
            global
                .iter()
                .any(|s| matches!(s, Painted::Mesh { colour: Some(c), .. } if *c == past_fill)),
            "and paints it"
        );
        assert!(
            global.iter().any(|s| matches!(s, Painted::Mesh { colours, .. } if in_ramp(colours))),
            "and paints the pulse surfaces too"
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

        // The cells are unchanged - the pieces got bigger, not more numerous - so a rest frame
        // still draws exactly one tessellated plane per surface of constant r.
        let metric = KerrSchild::new(1.0, 0.9);
        let bob = bob_at(&metric, 3.0);
        let mut canvas = VolumeCanvas {
            camera: Camera::preset(Preset::ThreeQuarter, 48.0, Vec2::ZERO, 1.0),
            show_past_cone: false,
            keep_surface_framed: false,
            ..Default::default()
        };
        let shapes = volume_frame_on(&mut canvas, &metric, Some(&bob), ReferenceFrame::Bob, false);
        let quads = shapes.iter().filter(|s| matches!(s, Painted::Mesh { vertices: 4, .. })).count();
        println!("{quads} four-cornered meshes for the four surfaces r = const");
        assert_eq!(
            quads,
            4 * PLANE_CELLS * PLANE_CELLS,
            "four surfaces at {PLANE_CELLS} cells a side"
        );
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
