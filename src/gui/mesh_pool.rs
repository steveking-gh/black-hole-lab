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
//! The threads are scoped and spawned per frame. They borrow what they draw - a field of light
//! several megabytes deep - straight from the caller, with no copy, no `'static` and no `unsafe`,
//! and they are joined before the borrow ends. Spawning a handful costs about a third of a
//! millisecond on Windows, against tens of milliseconds of work shared out.
//!
//! How a persistent pool would slot in: `build_meshes` is the only place that knows threads exist,
//! and its signature - a list of jobs in, their results out in the same order - is what a pool
//! would also offer, so the painters would not change. The jobs would. A pool's threads outlive the
//! frame, so a job could no longer borrow the field; it would need its own snapshot of the fronts,
//! about 16 bytes a ray, taken as each simulation thread finishes its own advance. That snapshot is
//! also what would let the drawing of one transmission start while the other is still being
//! stepped, which scoped threads joined after the whole step cannot do.
//!
//! Not yet built on this, and each the same pattern of shapes into a mesh per chunk of a field:
//! the (t, r) chart's ray comets and role comets - the ray-comet pass at All with 1024 points is
//! 130 000 fading lines - and the 2D+1 volume's pulse surfaces and floor fronts. The volume's floor
//! is not an affine projection, so its chunks want their own equality test before they are
//! trusted.

use std::sync::Mutex;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicUsize, Ordering};

use egui::epaint::{Mesh, TessellationOptions, Tessellator, TextureId};
use egui::{Rect, Shape};

/// Most threads `build_meshes` starts besides the one calling it.
///
/// Past about eight the jobs of one frame are too small for another thread to pay for its spawn,
/// and the physics has two threads of its own that the next frame will want back.
pub(crate) const MAX_MESH_WORKERS: usize = 8;

/// How many threads `build_meshes` will start besides the one calling it: one fewer than the
/// machine has, so the caller's share is not taken from it, and never more than
/// `MAX_MESH_WORKERS`. Zero on a machine with one core, where every job runs on the caller.
pub(crate) fn mesh_workers() -> usize {
    static WORKERS: OnceLock<usize> = OnceLock::new();
    *WORKERS.get_or_init(|| {
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        cores.saturating_sub(1).min(MAX_MESH_WORKERS)
    })
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
    pub(crate) fn tessellate(&mut self, shapes: impl IntoIterator<Item = Shape>) -> Vec<Shape> {
        let mut run = Run { mesh: Mesh::default(), shapes: Vec::new(), out: Vec::new() };
        for shape in shapes {
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

/// The run of shapes `MeshBuilder::tessellate` is gathering into one mesh, the shapes it was made
/// from in case egui would cull the mesh, and everything handed over so far.
struct Run {
    mesh: Mesh,
    shapes: Vec<Shape>,
    out: Vec<Shape>,
}

/// Run every job and return what each returned, in the order the jobs were given.
///
/// Up to `mesh_workers` scoped threads are started and the calling thread works alongside them,
/// every thread taking the next job not yet taken until none are left, so a heavy job holds up one
/// thread and not a share of the rest. A panic in a job is carried back to the caller. With one
/// job, or no thread to spare, the jobs simply run on the caller in order.
pub(crate) fn build_meshes<J, R>(jobs: Vec<J>) -> Vec<R>
where
    J: FnOnce() -> R + Send,
    R: Send,
{
    let workers = mesh_workers().min(jobs.len().saturating_sub(1));
    if workers == 0 {
        return jobs.into_iter().map(|job| job()).collect();
    }
    let count = jobs.len();
    let queue: Vec<Mutex<Option<J>>> = jobs.into_iter().map(|job| Mutex::new(Some(job))).collect();
    let next = AtomicUsize::new(0);
    let work = || {
        let mut done = Vec::new();
        loop {
            let k = next.fetch_add(1, Ordering::Relaxed);
            let Some(slot) = queue.get(k) else { break };
            let job = slot
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .take()
                .expect("each job is taken by exactly one thread");
            done.push((k, job()));
        }
        done
    };
    let mut done = std::thread::scope(|s| {
        let threads: Vec<_> = (0..workers).map(|_| s.spawn(work)).collect();
        let mut done = work();
        for thread in threads {
            done.extend(thread.join().unwrap_or_else(|panic| std::panic::resume_unwind(panic)));
        }
        done
    });
    done.sort_unstable_by_key(|(k, _)| *k);
    debug_assert_eq!(done.len(), count, "every job ran once");
    done.into_iter().map(|(_, result)| result).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_results_come_back_in_job_order_whatever_thread_ran_them() {
        // The jobs finish in a scrambled order - the early ones sleep longest - and the results
        // must come back in the order the jobs were given all the same, since that order is the
        // order the meshes are painted in.
        let jobs: Vec<_> = (0..24u64)
            .map(|k| {
                move || {
                    std::thread::sleep(std::time::Duration::from_micros((24 - k) * 50));
                    k * k
                }
            })
            .collect();
        let got = build_meshes(jobs);
        assert_eq!(got, (0..24u64).map(|k| k * k).collect::<Vec<_>>());
    }

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
