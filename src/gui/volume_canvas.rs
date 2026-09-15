use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::spacetime_canvas::TelemetryBoxes;
use crate::gui::spatial_canvas::{
    CENTRED_RING_GAP, FrontStyle, Who, draw_reception_tick, draw_signal_field, draw_spatial_trail,
    frame_focus,
};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::tetrad::light_cone_generators;
use crate::physics::wavefront::{NullRay, SignalField};
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

/// The three camera positions the view offers as buttons.
///
/// `Top` is the equatorial view seen from directly above, which is what the other canvas draws;
/// `Side` is almost edge-on, where the floor collapses to a line and the picture is a (space, time)
/// diagram; `ThreeQuarter` is the one that shows a cone as a cone.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Preset {
    Top,
    Side,
    ThreeQuarter,
}

impl Preset {
    /// The (yaw, pitch) this preset puts the eye at. `Side` is not exactly edge-on: at pitch 0 the
    /// floor is a single line and every worldline crossing the hole lands on top of every other, so
    /// it is tilted just far enough that near and far are distinguishable.
    fn angles(self) -> (f32, f32) {
        match self {
            Self::Top => (0.0, std::f32::consts::FRAC_PI_2),
            Self::Side => (0.0, 0.17),
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
    let len = (n_xy.0 * n_xy.0 + n_xy.1 * n_xy.1).sqrt();
    if len <= 0.0 {
        return 0.0;
    }
    let n = [n_xy.0 / len, n_xy.1 / len, 0.0];
    let facing = dot(n, view).abs().min(1.0);
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

        // 1. The camera, moved before anything is projected, so that this frame draws the view the
        // pointer has just asked for rather than the previous one. The equatorial view can afford
        // to defer its pan by a frame because a pan is a translation the eye does not track; an
        // orbit is not, and a view that lags the drag by a frame feels like it is being dragged
        // through treacle.
        let mut moved = false;
        if response.dragged() {
            let delta = response.drag_delta();
            if delta != Vec2::ZERO {
                if ui.input(|i| i.modifiers.shift) {
                    self.camera.pan += delta;
                } else {
                    self.camera.yaw += delta.x * 0.01;
                    self.camera.pitch = (self.camera.pitch - delta.y * 0.01)
                        .clamp(0.05, std::f32::consts::FRAC_PI_2);
                }
                moved = true;
            }
        }
        if response.hovered() {
            // The same step and the same clamp as the equatorial view's wheel, taken about the
            // cursor in the same way, so that the two canvases zoom at one rate and `scale` goes on
            // meaning px/M in both. The cursor is held on the point under it only up to the tilt:
            // the anchor is the nominal centre `rect.center() + pan`, which is where world
            // (0, 0, 0) lands with nobody being followed.
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let step = 0.01875;
                let zoom_mult = if scroll > 0.0 { 1.0 + step } else { 1.0 - step };
                let old_scale = self.camera.scale;
                let new_scale = (old_scale * zoom_mult).clamp(8.0, 500_000.0);
                if let Some(mpos) = response.hover_pos() {
                    let nominal = rect.center() + self.camera.pan;
                    self.camera.pan += (mpos - nominal) * (1.0 - new_scale / old_scale);
                }
                self.camera.scale = new_scale;
                moved = true;
            }
            // Ctrl-wheel stretches time against space. It is the one control here with no
            // counterpart on the other canvases: t_scale is the exchange rate between an M of time
            // and an M of length, and at 1 a far-away light ray rises at 45 degrees, so moving it
            // off 1 is moving the picture off the one setting where a slope can be read as a speed.
            let zoom_delta = ui.input(|i| i.zoom_delta());
            if (zoom_delta - 1.0).abs() > 1e-4 {
                self.camera.t_scale =
                    (self.camera.t_scale * f64::from(zoom_delta)).clamp(0.1, 10.0);
                moved = true;
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
        let focus = frame_focus(self.centred_on, frame_of_ref, bob, alice);
        let offset = focus.map_or(Vec2::ZERO, |obs| {
            let (x, y) = obs.cartesian_position(metric);
            camera.project(Pos2::ZERO, [x, y, 0.0]).0 - Pos2::ZERO
        });
        self.focus_offset = offset;
        let centre = rect.center() + camera.pan - offset;

        // 3. The projection, as the three closures the rest of the frame is written in.
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

        let mut buf = PrimBuffer::default();

        // 4. The surfaces of constant r, as glass pipes below the floor and as rings on it.
        //
        // A pipe is the honest picture of what r = const is in this chart: not a circle a worldline
        // happens to cross, but a wall standing in time, so that "Bob went through r+" is a
        // worldline entering a tube and never leaving it. The wall is only drawn below the floor,
        // over the past the simulation has actually integrated; above it there are rings alone,
        // because the future of a horizon is not something this run has computed and a solid wall
        // up there would claim it had.
        let z_bottom = z_of(t_min);
        let tick_r = if metric.cartesian_radius(rm) > 0.0 { rm } else { rp };
        let mut floor_rings: Vec<(Vec<Pos2>, Stroke)> = Vec::new();
        for (r, colour, base_alpha, width) in [
            (0.0, Theme::SINGULARITY_LINE, 70u8, 2.0f32),
            (rm, Theme::HORIZON_CAUCHY, 60, 2.0),
            (rp, Theme::HORIZON_OUTER, 60, 2.5),
            (re, Theme::ERGOSPHERE_LINE, 35, 1.5),
        ] {
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
            floor_rings.push((ring_points(rho, 0.0), Stroke::new(width, colour)));

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
        for (obs, who) in present.iter().copied() {
            // Heavier than the shadow on the floor: the worldline is seen through the glass of
            // whichever pipes stand between it and the eye, and at the floor's weight it was lost
            // behind them.
            let width = match who {
                Who::Alice => 1.8,
                Who::Bob => 2.2,
            };
            let points: Vec<(f64, [f64; 3])> = obs
                .trail
                .iter()
                .filter(|p| p.t >= t_min && p.t <= current_time)
                .map(|p| {
                    let (x, y) = metric.cartesian_position(p.r, p.phi);
                    (p.t, [x, y, z_of(p.t)])
                })
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
        let cone_span = (self.time_window * 0.12).clamp(1e-4, 1.8);
        let push_cone = |buf: &mut PrimBuffer,
                             r: f64,
                             phi: f64,
                             z0: f64,
                             fills: (Color32, Color32, Color32),
                             ghost: bool| {
            let tetrad = Observer::raindrop_tetrad(metric, r);
            let gens = light_cone_generators(metric, r, phi, &tetrad, CONE_SAMPLES);
            if gens.is_empty() {
                return;
            }
            let (x, y) = metric.cartesian_position(r, phi);
            let apex = [x, y, z0];
            let rise = cone_span * t_scale;
            let future: Vec<[f64; 3]> = gens
                .iter()
                .map(|(gx, gy)| [x + cone_span * gx, y + cone_span * gy, z0 + rise])
                .collect();
            // The past half is the future half reflected through the apex, which is what the past
            // cone of an event is: the same null directions run backwards.
            let past: Vec<[f64; 3]> =
                future.iter().map(|p| [2.0 * x - p[0], 2.0 * y - p[1], 2.0 * z0 - p[2]]).collect();
            let n = gens.len() as f64;
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
            push_cone(&mut buf, obs.r, obs.phi, 0.0, (future_fill, past_fill, edge), false);
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
                        p.r,
                        p.phi,
                        z_of(p.t),
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
                let to_world = |s: [f64; 3]| {
                    let (x, y) = metric.cartesian_position(s[1], s[2]);
                    [x, y, z_of(s[0])]
                };
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
                        let depth = centroid_depth(&camera, centre, corners.into_iter());
                        buf.push(Layer::Below, depth, Prim::Mesh(mesh));
                        // One row of overlap, so the chunks meet instead of leaving a gap.
                        k0 = k1;
                    }
                }
                // Every fourth generator drawn as a line, so the eye can follow one photon's
                // history across a surface that is otherwise a wash. Nine of them: enough to read
                // the twist frame dragging puts into the cone, few enough not to fill it in.
                for i in (0..n).step_by(4) {
                    let points: Vec<[f64; 3]> = cone.rays[i].iter().map(|s| to_world(*s)).collect();
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

        // The scene is complete, so the painter's algorithm can run: everything below the floor
        // farthest first, then the floor, then everything above it. Nothing is painted before this
        // point, because the scene is built surface by surface and observer by observer rather
        // than in depth order - which is the whole reason `PrimBuffer` holds it.
        buf.paint(Layer::Below, &painter);

        // 8. The floor: the equatorial view's own picture of the present, laid flat in the volume.
        // It is painted straight onto the painter between the two layers rather than through the
        // buffer, because it *is* the sorting plane - the one surface whose place in the order is
        // known without a depth - and because the fronts and the trails come from the equatorial
        // view's helpers, which paint rather than return shapes.
        let ring_rho = metric.cartesian_radius(0.0).max(2.0 / f64::from(camera.scale));
        for (rho, fill) in [
            (metric.cartesian_radius(re), Theme::ERGOSPHERE_FILL),
            (metric.cartesian_radius(rp), Theme::REGION_II_FILL),
            (metric.cartesian_radius(rm), Theme::REGION_III_FILL),
            (ring_rho, Theme::SINGULARITY_FILL),
        ] {
            painter.add(egui::Shape::convex_polygon(ring_points(rho, 0.0), fill, Stroke::NONE));
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

        // 9. Every arrival, on the receiver's worldline at the height of the crossing, in the
        // sender's colour: the same pairing the equatorial view draws, lifted off the floor onto
        // the event it happened at. Inside r+ these bunch onto the r- pipe, and in the volume the
        // bunch is legible as a stack up the wall rather than as a knot of triangles on a circle.
        let ticks = |field: &SignalField, sender: Color32| {
            for reception in field.receptions() {
                if reception.t < t_min || reception.t > current_time {
                    continue;
                }
                let (x, y) = metric.cartesian_position(reception.r, reception.phi);
                let at = project([x, y, z_of(reception.t)]).0;
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

        // 10. The observers, on the floor, because the floor is now.
        let mut markers: Vec<(Who, Pos2)> = Vec::new();
        for (obs, who) in present.iter().copied() {
            let (x, y) = obs.cartesian_position(metric);
            let at = project([x, y, 0.0]).0;
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

        // 11. The canvas's right-click menu: where to look, what to go on looking at, and where the
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
                    self.centred_on = centred.then_some(who);
                    ui.close();
                }
            }
            ui.separator();
            if ui.checkbox(&mut self.show_ghost_cones, "Ghost cones along the trail").changed() {
                ui.close();
            }
            if ui
                .checkbox(&mut self.show_past_cone, "Exact past cone of the focus event")
                .changed()
            {
                ui.close();
            }
            ui.separator();
            for (label, preset) in
                [("Top", Preset::Top), ("Side", Preset::Side), ("3/4", Preset::ThreeQuarter)]
            {
                if ui.button(label).clicked() {
                    self.camera =
                        Camera::preset(preset, self.camera.scale, self.camera.pan, self.camera.t_scale);
                    ui.close();
                }
            }
        });

        // The same three camera positions as buttons, because a right-click menu is not
        // discoverable by looking at a picture, and a fourth that undoes an exploration: Reset puts
        // the zoom, the pan and the time scale back where `Camera::default` has them and leaves the
        // eye where the user has moved it, which is the one part of the view they chose on purpose.
        let legend_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let button_size = Vec2::new(40.0 * font_scale, 16.0 * font_scale);
        let gap = 4.0 * font_scale;
        let strip = button_size.x * 4.0 + gap * 3.0;
        for (i, (label, preset)) in [
            ("Top", Some(Preset::Top)),
            ("Side", Some(Preset::Side)),
            ("3/4", Some(Preset::ThreeQuarter)),
            ("Reset", None),
        ]
        .into_iter()
        .enumerate()
        {
            let min =
                rect.right_top() + Vec2::new(-8.0 - strip + (button_size.x + gap) * i as f32, 6.0);
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

        // 12. The legend: what the picture is, where the eye is standing, and what the vertical
        // axis means, since a viewer arriving at a 3D diagram has no way to know any of the three.
        painter.text(
            rect.left_top() + Vec2::new(10.0, 6.0),
            egui::Align2::LEFT_TOP,
            "2D+1 Volume (x, y, t)",
            legend_font.clone(),
            Theme::TEXT_BRIGHT,
        );
        painter.text(
            rect.left_top() + Vec2::new(10.0, 8.0 + Theme::MIN_FONT_PT * font_scale),
            egui::Align2::LEFT_TOP,
            format!(
                "yaw {:.0}°  pitch {:.0}°  {:.0} px/M  t×{:.2}\n\
                 window {:.1} … {:.1} M  (floor = now)\n\
                 drag: orbit  shift-drag: pan  wheel: zoom  ctrl-wheel: time scale  \
                 right-click: menu\n\
                 below the floor: the past · above: the future · pipes: r = const · cones: exact \
                 null generators{}",
                camera.yaw.to_degrees(),
                camera.pitch.to_degrees(),
                camera.scale,
                t_scale,
                t_min,
                t_max,
                // Terse, on the end of the line that says what the other shapes are, and only when
                // the surface is actually on screen to be named.
                if self.show_past_cone {
                    "\npast cone: the event's null geodesics run backwards"
                } else {
                    ""
                },
            ),
            legend_font.clone(),
            Theme::TEXT_MUTED,
        );

        buf.paint_labels(&painter, legend_font);

        // 13. Draggable info boxes, registered last so they take the drag instead of the canvas.
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
        /// what both of this canvas's meshes do.
        Mesh { vertices: usize, colour: Option<Color32>, first: Pos2 },
        /// A filled polygon or a stroked polyline. The fill is `Color32::TRANSPARENT` on a
        /// polyline and the stroke is None on a fill.
        Path { fill: Color32, stroke: Option<Color32>, points: Vec<Pos2> },
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
                    out.push(Painted::Mesh { vertices: mesh.vertices.len(), colour, first });
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
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let signal = SignalField::default();
        // Every worldline stands at the simulation clock, so the floor is the observer's own now.
        let clock = bob.map_or(0.0, |obs| obs.t);
        let output = ctx.clone().run_ui(input(), |ui| {
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
                SignalViews { alice: &signal, bob: &signal },
                show_distant_clock_grid,
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
                    for frame in [ReferenceFrame::DistantObserver, ReferenceFrame::Bob] {
                        let shapes = volume_frame(&metric, Some(&bob), preset, frame, grid);
                        let text = text_of(&shapes);
                        assert!(
                            text.contains("2D+1 Volume"),
                            "{name} at {preset:?} in {frame:?} (grid {grid}) drew no volume view"
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
                    Painted::Mesh { vertices, colour: Some(c), first } if *c == fill => {
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
    fn test_the_side_view_puts_later_times_up_the_screen() {
        // Edge-on exactly, which no preset is: the point is that up the screen is +t there.
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
}
