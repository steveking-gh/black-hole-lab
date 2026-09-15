#![allow(dead_code)] // Step 3 removes this when VolumeCanvas::render draws with the pieces below.

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
/// cannot say that; the flat variants exist so that opaque markers and worldlines do not each pay
/// for a two-triangle mesh.
pub enum Prim {
    /// Per-vertex colours, premultiplied.
    Mesh(egui::Mesh),
    Polygon { points: Vec<Pos2>, fill: Color32 },
    Line { points: Vec<Pos2>, stroke: Stroke, closed: bool },
    Disc { centre: Pos2, radius: f32, fill: Color32 },
}

impl Prim {
    fn into_shape(self) -> egui::Shape {
        match self {
            Self::Mesh(mesh) => egui::Shape::mesh(mesh),
            Self::Polygon { points, fill } => {
                egui::Shape::convex_polygon(points, fill, Stroke::NONE)
            }
            Self::Line { points, stroke, closed } => {
                if closed {
                    egui::Shape::closed_line(points, stroke)
                } else {
                    egui::Shape::line(points, stroke)
                }
            }
            Self::Disc { centre, radius, fill } => egui::Shape::circle_filled(centre, radius, fill),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gui::theme::Theme;

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
                    Prim::Disc { centre: Pos2::new(x, 100.0), radius: 4.0, fill: Color32::RED },
                );
            }
            buf.label(Pos2::new(70.0, 100.0), egui::Align2::LEFT_TOP, "r+", Color32::WHITE);
            buf.paint(Layer::Above, painter);
            buf.paint_labels(painter, egui::FontId::proportional(12.0));
        });

        let shapes = flatten(&output);
        let mut circles = Vec::new();
        let mut last_circle = None;
        let mut first_text = None;
        for (i, shape) in shapes.iter().enumerate() {
            match shape {
                egui::Shape::Circle(c) => {
                    circles.push(c.center.x);
                    last_circle = Some(i);
                }
                egui::Shape::Text(_) => {
                    first_text.get_or_insert(i);
                }
                _ => {}
            }
        }
        output.drop_without_applying_deltas();

        assert_eq!(
            circles,
            vec![50.0, 30.0, 10.0],
            "the discs should be painted farthest first - depths 5, 3, 1, so screen x 50, 30, 10 - \
             but came out as {circles:?}"
        );
        let last_circle = last_circle.expect("the three discs were painted");
        let first_text = first_text.expect("the label was painted");
        assert!(
            first_text > last_circle,
            "every label should follow every primitive, but the first text is shape {first_text} \
             and the last circle is shape {last_circle}"
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
