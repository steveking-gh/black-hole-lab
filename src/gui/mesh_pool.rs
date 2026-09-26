//! Meshes built off the main thread, for the painters too heavy to build on it.
//!
//! A painter on an egui canvas does two jobs every frame, and at the top of the panel's sliders
//! both are big. It builds its shapes - a polyline per band of front, a fade under each - and egui
//! then tessellates every one of them into triangles at the end of the frame. Both run on the one
//! thread the whole ui runs on. At 1024 points by 128 wavefronts the equatorial view's fronts are
//! about 50 ms of building and most of 31 ms of tessellation in a frame of 104 ms, with every other
//! core of the machine standing idle.
//!
//! egui draws a finished `Shape::Mesh` exactly as it draws anything else, and a `Shape` is a plain
//! value with no tie to the context that will paint it. So a worker can do both jobs for a piece of
//! a picture without touching egui at all: build that piece's shapes, and tessellate them with an
//! `epaint::Tessellator` of its own, set up exactly as egui's end-of-frame one will be. What comes
//! back is meshes, which egui's own tessellation then only has to append. The workers never touch
//! a `Painter` - adding a shape takes the context's write lock - so the calling thread hands the
//! meshes over once they are all in.
//!
//! The picture has to come out the same as if the calling thread had built it alone, and it does:
//! the same vertices, the same colours, the same triangles in the same order. That holds for three
//! reasons, each of which the code here keeps.
//!
//! - A tessellation is a function of the shape, the pixel density, the tessellation options and the
//!   clip rect, and of nothing else: a shape's triangles are indexed from the end of whatever mesh
//!   they are appended to, so tessellating a shape into a mesh of its own and appending that mesh
//!   later lands exactly the vertices and indices a direct tessellation would have.
//! - Translucent triangles do not commute, so order is the picture. The jobs return their meshes
//!   in job order, whichever thread finished first, and the caller adds them in that order.
//! - egui throws away a shape whose bounds miss the clip rect before it tessellates it, and does it
//!   again for a whole mesh, by the bounds of all its vertices. `MeshBuilder::tessellate` culls
//!   every shape exactly as egui would, and hands over a mesh only where egui's second cull will
//!   keep it - otherwise it hands over the shapes themselves, and egui does what it always did with
//!   them.
//!
//! A shape whose triangles depend on the font atlas is not tessellated here at all. A small filled
//! circle is drawn from a disc prerasterized into that atlas, and its texture coordinates are
//! normalised by the atlas's size at the moment egui tessellates it - at the end of the frame,
//! after any glyph first laid out later in the same frame has grown the atlas. A worker cannot know
//! that size, and the atlas's discs are not reachable through egui's public interface in any case.
//! So circles, text, and every other shape this module has no reason to take on pass through
//! untouched, in their place in the order, and egui tessellates them as before.
//!
//! The threads themselves are `crate::pool`'s, which the simulation's ray integration shares: see
//! there for why they are scoped and spawned per call, how the jobs are shared out, and how a
//! persistent pool would slot in. `build_meshes` is that fork-join with the painters' worker count,
//! and the painters know nothing else about threads.
//!
//! Two painters are built on this: the equatorial view's fronts (`draw_signal_fields_parallel`)
//! and the (t, r) chart's comets (`draw_comets_parallel`), whose "Ray comets" pass at All is a line
//! a live ray, about 105 000 of them at 1024 points by 128 wavefronts. The comets are the simpler of the two, because nothing carries
//! from one pulse's comets to the next, and the one with a colour callback: a column comet's fade
//! is computed per vertex by whichever tessellator lays it down, which is why a worker's colours
//! are egui's. Each has its own equality test.
//!
//! Not yet built on this, and the same pattern of shapes into a mesh per chunk of a field: the 2D+1
//! volume's pulse surfaces and floor fronts. The volume's floor is not an affine projection, so its
//! chunks want their own equality test before they are trusted.

use egui::epaint::{Mesh, TessellationOptions, Tessellator, TextureId};
use egui::{Rect, Shape};

/// How many threads `build_meshes` will start besides the one calling it: all that
/// `crate::pool::spare_threads` offers, which is one fewer than the machine has and never more
/// than `crate::pool::MAX_WORKERS`. The painters size their runs by it.
pub(crate) fn mesh_workers() -> usize {
    crate::pool::spare_threads()
}

/// Everything a worker needs to tessellate a shape exactly as egui will at the end of this frame.
///
/// `Context::tessellate` builds its `Tessellator` from the pixel density, the context's
/// tessellation options and the font atlas. The first two are all the shapes tessellated here
/// read; the atlas's size is carried because the tessellator's constructor asks for it, and its
/// prerasterized discs are left out because nothing that would read them is tessellated here. See
/// the module doc.
#[derive(Clone, Copy)]
pub(crate) struct TessellationSetup {
    pixels_per_point: f32,
    options: TessellationOptions,
    font_tex_size: [usize; 2],
}

impl TessellationSetup {
    /// The setup of the frame being drawn, read off the context on the calling thread.
    pub(crate) fn capture(ctx: &egui::Context) -> Self {
        Self {
            pixels_per_point: ctx.pixels_per_point(),
            options: ctx.tessellation_options(|options| *options),
            font_tex_size: ctx.fonts(|fonts| fonts.font_image_size()),
        }
    }

    /// A builder that tessellates into `clip`, which has to be the clip rect of the painter the
    /// meshes will be added to: it is the rect egui culls them against.
    pub(crate) fn builder(&self, clip: Rect) -> MeshBuilder {
        let mut tessellator =
            Tessellator::new(self.pixels_per_point, self.options, self.font_tex_size, Vec::new());
        tessellator.set_clip_rect(clip);
        MeshBuilder { tessellator, clip, culls: self.options.coarse_tessellation_culling }
    }
}

/// One worker's tessellator, and the clip rect it culls against.
pub(crate) struct MeshBuilder {
    tessellator: Tessellator,
    clip: Rect,
    culls: bool,
}

impl MeshBuilder {
    /// Whether egui keeps a `Shape::Mesh` whose vertices span `bounds`, or drops it whole before
    /// appending it: the test `Tessellator::tessellate_shape` makes of every mesh it is given.
    pub(crate) fn keeps(&self, bounds: Rect) -> bool {
        !self.culls || self.clip.intersects(bounds)
    }

    /// `shapes`, in order, as egui will paint them: every run of consecutive shapes this builder
    /// takes on turned into one mesh, and everything else passed through as it came.
    ///
    /// A polyline, a line segment and an untextured mesh are taken on. Each is culled and
    /// tessellated here exactly as egui would, so the mesh of a run carries the very vertices egui
    /// would have appended for those shapes. The run is then handed over as that mesh only where
    /// egui's own cull of the mesh will keep it. A polyline passes egui's cull when its points come
    /// within half a stroke width of the clip rect, and with feathering off a butt-ended line
    /// tessellates to no vertex beyond its end points, so a run of such lines can carry triangles
    /// egui would have kept in a mesh whose bounds lie wholly outside the rect. Such a run goes
    /// back as its shapes instead, and egui tessellates them itself. (With feathering on, as egui
    /// has it by default, the feather carries every line's vertices past that half stroke width,
    /// and no run of lines has yet been seen to need it.) A run culled to nothing hands over
    /// nothing, which is also what egui would have made of it.
    ///
    /// `Shape::Vec` is flattened and `Shape::Noop` dropped: neither puts a vertex on the screen.
    ///
    /// A run's mesh is sized before it is filled, for the polylines standing at its head: see
    /// `stroke_reserve`.
    pub(crate) fn tessellate(&mut self, shapes: impl IntoIterator<Item = Shape>) -> Vec<Shape> {
        let shapes: Vec<Shape> = shapes.into_iter().collect();
        let ahead = stroke_reserve(&shapes);
        let mut run = Run { mesh: Mesh::default(), shapes: Vec::new(), out: Vec::new() };
        for (shape, (vertices, indices)) in shapes.into_iter().zip(ahead) {
            if run.mesh.is_empty() {
                run.mesh.vertices.reserve(vertices);
                run.mesh.indices.reserve(indices);
            }
            self.take(shape, &mut run);
        }
        self.close(&mut run);
        run.out
    }

    fn take(&mut self, shape: Shape, run: &mut Run) {
        match shape {
            Shape::Noop => {}
            Shape::Vec(inner) => {
                for shape in inner {
                    self.take(shape, run);
                }
            }
            Shape::Path(ref path) => {
                self.tessellator.tessellate_path(path, &mut run.mesh);
                run.shapes.push(shape);
            }
            Shape::LineSegment { .. } => {
                self.tessellator.tessellate_shape(shape.clone(), &mut run.mesh);
                run.shapes.push(shape);
            }
            Shape::Mesh(ref mesh) if mesh.texture_id == TextureId::default() => {
                self.tessellator.tessellate_shape(shape.clone(), &mut run.mesh);
                run.shapes.push(shape);
            }
            other => {
                self.close(run);
                run.out.push(other);
            }
        }
    }

    /// End the run in progress: its mesh into the output if egui will keep it, its shapes if egui
    /// would cull the mesh, and nothing if nothing of it survived the cull.
    fn close(&self, run: &mut Run) {
        if !run.mesh.is_empty() {
            if self.keeps(run.mesh.calc_bounds()) {
                run.out.push(Shape::mesh(std::mem::take(&mut run.mesh)));
            } else {
                run.mesh.clear();
                run.out.append(&mut run.shapes);
            }
        }
        run.shapes.clear();
    }
}

/// For each of `shapes`, the vertices and indices at most that the unbroken stretch of polylines
/// starting there can tessellate to: nothing where a shape is not a polyline.
///
/// epaint strokes an open polyline wider than its feathering as four vertices a point and six
/// triangles a segment, plus two at each end, which is 4 vertices and 18 indices a point with room
/// to spare; a thinner one takes fewer. A run's mesh sized to that in one allocation is written
/// once, where a mesh left to grow is copied on every doubling, and at the (t, r) chart's "Ray
/// comets" pass - some 3.4 million vertices a frame at every 8th of 1024 rays - the copies and the
/// fresh pages they touch were the larger part of the workers' tessellating (80 ms of thread time
/// a frame against 57 sized). A polyline egui culls draws nothing and its share of the reservation
/// is left untouched; the mesh itself is the same either way, only its capacity differs.
fn stroke_reserve(shapes: &[Shape]) -> Vec<(usize, usize)> {
    let mut ahead = vec![(0, 0); shapes.len()];
    let mut stretch = (0, 0);
    for (k, shape) in shapes.iter().enumerate().rev() {
        stretch = match shape {
            Shape::Path(path) => {
                let points = path.points.len();
                (stretch.0 + 4 * points, stretch.1 + 18 * points)
            }
            _ => (0, 0),
        };
        ahead[k] = stretch;
    }
    ahead
}

/// The run of shapes `MeshBuilder::tessellate` is gathering into one mesh, the shapes it was made
/// from in case egui would cull the mesh, and everything handed over so far.
struct Run {
    mesh: Mesh,
    shapes: Vec<Shape>,
    out: Vec<Shape>,
}

/// Run every job and return what each returned, in the order the jobs were given, on
/// `mesh_workers` threads besides the caller: `crate::pool::run_jobs`, which says how the jobs are
/// shared out and why the order is kept.
pub(crate) fn build_meshes<J, R>(jobs: Vec<J>) -> Vec<R>
where
    J: FnOnce() -> R + Send,
    R: Send,
{
    crate::pool::run_jobs(jobs, mesh_workers())
}

#[cfg(test)]
pub(crate) mod frames {
    //! The frame egui's own end-of-frame tessellation makes of a painter's work, as bits, and the
    //! first difference between two such frames: what the painters built on `build_meshes` are
    //! proved against, the serial frame to the parallel one.

    /// One vertex exactly as the GPU is given it: position, texture coordinate and colour, the
    /// floats as their bits so that equality is equality to the bit.
    type RawVertex = [u32; 5];

    /// One primitive of a tessellated frame: its clip rect as bits, its texture, its vertices and
    /// its indices.
    pub(crate) type RawPrimitive = ([u32; 4], egui::TextureId, Vec<RawVertex>, Vec<u32>);

    /// Whatever `paint` puts on a 400 px canvas, and the frame egui's own end-of-frame
    /// tessellation makes of it.
    pub(crate) fn tessellated_frame(
        mut paint: impl FnMut(&egui::Ui, &egui::Painter),
    ) -> Vec<RawPrimitive> {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            paint(ui, &painter);
        });
        let shapes = std::mem::take(&mut output.shapes);
        let primitives = ctx.tessellate(shapes, output.pixels_per_point);
        output.drop_without_applying_deltas();
        primitives
            .into_iter()
            .map(|clipped| {
                let r = clipped.clip_rect;
                let clip = [r.min.x, r.min.y, r.max.x, r.max.y].map(f32::to_bits);
                match clipped.primitive {
                    egui::epaint::Primitive::Mesh(mesh) => {
                        let vertices = mesh
                            .vertices
                            .iter()
                            .map(|v| {
                                [
                                    v.pos.x.to_bits(),
                                    v.pos.y.to_bits(),
                                    v.uv.x.to_bits(),
                                    v.uv.y.to_bits(),
                                    u32::from_le_bytes(v.color.to_array()),
                                ]
                            })
                            .collect();
                        (clip, mesh.texture_id, vertices, mesh.indices)
                    }
                    egui::epaint::Primitive::Callback(_) => {
                        panic!("the painters these frames compare paint no callbacks")
                    }
                }
            })
            .collect()
    }

    /// The first difference between two tessellated frames, or None where they are the same to
    /// the bit.
    pub(crate) fn first_difference(
        serial: &[RawPrimitive],
        parallel: &[RawPrimitive],
    ) -> Option<String> {
        if serial.len() != parallel.len() {
            return Some(format!("{} primitives against {}", serial.len(), parallel.len()));
        }
        for (p, (a, b)) in serial.iter().zip(parallel).enumerate() {
            if a.0 != b.0 || a.1 != b.1 {
                return Some(format!("primitive {p}: clip rect or texture differs"));
            }
            if a.2.len() != b.2.len() || a.3.len() != b.3.len() {
                return Some(format!(
                    "primitive {p}: {} vertices and {} indices against {} and {}",
                    a.2.len(),
                    a.3.len(),
                    b.2.len(),
                    b.3.len()
                ));
            }
            if let Some(v) = (0..a.2.len()).find(|&v| a.2[v] != b.2[v]) {
                let (x, y) = (a.2[v], b.2[v]);
                return Some(format!("primitive {p}, vertex {v}: {x:?} against {y:?}"));
            }
            if let Some(i) = (0..a.3.len()).find(|&i| a.3[i] != b.3[i]) {
                return Some(format!("primitive {p}, index {i}: {} against {}", a.3[i], b.3[i]));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_a_run_egui_would_drop_as_a_mesh_goes_back_as_its_shapes() {
        // The one case in which a mesh of lines is not the same thing to egui as the lines. A line
        // ending a third of a point short of the clip rect, at 1.2 points wide, passes egui's cull
        // of the line - its points widened by half the stroke reach the rect - and tessellates to
        // triangles; but with feathering off a butt-ended line puts no vertex past its own end, so
        // a mesh of those triangles has bounds wholly outside the rect and egui would drop the mesh
        // whole. The builder has to hand back the line itself, which egui then tessellates as it
        // always did. A line well inside the rect is handed over as a mesh, and a dot between the
        // two is left for egui in its place, because egui draws a small dot from its font atlas.
        let clip = Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(100.0, 100.0));
        let options = TessellationOptions { feathering: false, ..Default::default() };
        let setup = TessellationSetup { pixels_per_point: 1.0, options, font_tex_size: [1024, 32] };
        let stroke = egui::Stroke::new(1.2, egui::Color32::WHITE);
        let short = Shape::line(vec![egui::pos2(-20.0, 50.0), egui::pos2(-0.3, 50.0)], stroke);
        let inside = Shape::line(vec![egui::pos2(20.0, 50.0), egui::pos2(80.0, 50.0)], stroke);
        let dot = Shape::circle_filled(egui::pos2(50.0, 20.0), 2.0, egui::Color32::WHITE);

        // The premise, from egui's own tessellator: the short line is kept and drawn, and the mesh
        // of it would be dropped.
        let mut alone = Mesh::default();
        let mut tessellator = Tessellator::new(1.0, setup.options, setup.font_tex_size, Vec::new());
        tessellator.set_clip_rect(clip);
        tessellator.tessellate_shape(short.clone(), &mut alone);
        assert!(!alone.is_empty(), "egui draws the short line");
        let bounds = alone.calc_bounds();
        assert!(!clip.intersects(bounds), "and would drop a mesh of it whole: {bounds:?}");

        let out = setup.builder(clip).tessellate([short, dot, inside]);
        assert!(
            matches!(out.as_slice(), [Shape::Path(_), Shape::Circle(_), Shape::Mesh(_)]),
            "the short line as itself, the dot left for egui, the other line as a mesh: {out:?}"
        );
    }
}
