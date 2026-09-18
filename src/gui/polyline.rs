//! Thinning a drawn polyline down to the resolution of the screen it is drawn on.
//!
//! A worldline trail holds one recorded event per stepped frame, so its length says how long the
//! run has been going and not how much of it can be seen. Handing all of it to `Shape::line` ties
//! the cost of drawing to the depth of the history buffer: at 10 000 recorded events one trail is
//! 10 000 vertices tessellated into 20 000 triangles every frame, for a curve with a few thousand
//! distinguishable positions on an 800-point canvas. Most of those vertices land on top of one
//! another, costing a triangle each and buying nothing - and the tessellator, asked for the joint
//! between two segments a hundredth of a point apart, has to take its normal from a direction that
//! is mostly rounding error.
//!
//! So let the screen decide. `thin_to_pixels` keeps a vertex once it has moved clear of the last
//! vertex kept, which bounds how far the drawn curve can depart from the recorded one by that same
//! distance: every dropped vertex lies inside a disc of that radius about a vertex that was kept.
//! At half a point the difference is smaller than a pixel on any display the app runs on, and the
//! vertex count stops following the buffer's depth and starts following the length of the curve on
//! the canvas, which is the thing the eye is actually being shown.
//!
//! That is the whole of the decoupling, and it is what lets the buffer's depth be chosen for how
//! much history is worth keeping rather than for what a polyline costs to draw.

use egui::Pos2;

/// How far a vertex must clear the last one drawn before it is worth drawing, in screen points.
///
/// Half a point is under one physical pixel on every display the app runs on, and a quarter of one
/// where the desktop is scaled, so thinning at this distance cannot be seen. It bounds the error
/// and not just the spacing: see `thin_to_pixels`.
pub(crate) const SCREEN_SPACING: f32 = 0.5;

/// `items` with every entry dropped that maps to within `spacing` of the last entry kept, keeping
/// the first and the last whatever falls out in between.
///
/// `at` says where an entry lands on the screen, which is why this is generic in the entry rather
/// than taking positions: the volume view carries a world position and a time with each vertex and
/// needs them back for the depth sort and the fade, while the flat views carry the screen position
/// itself and hand over the identity.
///
/// Keeping the last entry matters as much as the thinning does. A trail ends on the observer's
/// present event with the marker drawn standing on it, so a polyline that stopped at the last
/// vertex to clear the spacing would leave the worldline short of its own observer, by up to
/// `spacing`, every frame in which the observer had not yet moved that far.
pub(crate) fn thin_to_pixels<T>(
    items: impl IntoIterator<Item = T>,
    at: impl Fn(&T) -> Pos2,
    spacing: f32,
) -> Vec<T> {
    let min_sq = spacing * spacing;
    let mut kept: Vec<T> = Vec::new();
    // The newest entry that did not clear the spacing, held back rather than dropped outright: it
    // is drawn only if nothing further out comes along to replace it, which is what makes the last
    // entry of the input the last entry kept.
    let mut pending: Option<T> = None;
    let mut anchor: Option<Pos2> = None;
    for item in items {
        let here = at(&item);
        if anchor.is_none_or(|last| (here - last).length_sq() >= min_sq) {
            anchor = Some(here);
            pending = None;
            kept.push(item);
        } else {
            pending = Some(item);
        }
    }
    kept.extend(pending);
    kept
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::kerr_schild::KerrSchild;
    use crate::physics::observer::{Observer, WorldlineParams};

    /// The furthest any input vertex sits from the nearest vertex that survived: the error the
    /// thinning actually commits, which is what the spacing promises to bound.
    fn worst_departure(input: &[Pos2], kept: &[Pos2]) -> f32 {
        input
            .iter()
            .map(|p| kept.iter().map(|k| (*p - *k).length()).fold(f32::INFINITY, f32::min))
            .fold(0.0, f32::max)
    }

    #[test]
    fn test_thinning_keeps_the_ends_and_stays_inside_the_spacing() {
        // The two promises the drawing leans on, on a curve dense enough that most of it goes: a
        // thousand vertices around a circle a hundred points across, where consecutive vertices
        // are a third of a point apart.
        let input: Vec<Pos2> = (0..1000)
            .map(|k| {
                let a = k as f32 * std::f32::consts::TAU / 1000.0;
                Pos2::new(400.0 + 50.0 * a.cos(), 300.0 + 50.0 * a.sin())
            })
            .collect();
        let kept = thin_to_pixels(input.iter().copied(), |p| *p, SCREEN_SPACING);
        println!("{} vertices thinned to {}", input.len(), kept.len());
        assert!(kept.len() < input.len(), "a curve this dense should thin");
        assert_eq!(kept.first(), input.first(), "the first vertex is always drawn");
        assert_eq!(kept.last(), input.last(), "the last vertex is always drawn");
        let worst = worst_departure(&input, &kept);
        assert!(
            worst < SCREEN_SPACING,
            "thinning moved the curve by {worst} points, over the {SCREEN_SPACING} promised"
        );
    }

    #[test]
    fn test_a_curve_already_coarser_than_the_screen_is_left_alone() {
        // Nothing is thinned out of a polyline whose vertices are already far apart, and the
        // degenerate lengths come back as themselves rather than as a panic.
        let sparse: Vec<Pos2> = (0..20).map(|k| Pos2::new(k as f32 * 10.0, 0.0)).collect();
        let kept = thin_to_pixels(sparse.iter().copied(), |p| *p, SCREEN_SPACING);
        assert_eq!(kept, sparse, "a coarse curve is drawn as it stands");
        assert!(thin_to_pixels(Vec::<Pos2>::new(), |p| *p, SCREEN_SPACING).is_empty());
        let one = vec![Pos2::new(1.0, 2.0)];
        assert_eq!(thin_to_pixels(one.iter().copied(), |p| *p, SCREEN_SPACING), one);
    }

    #[test]
    fn test_a_deep_trail_draws_as_a_bounded_number_of_vertices() {
        // The point of the exercise, on a real worldline rather than on a circle: the vertex count
        // of a drawn trail has to follow the curve on the canvas and not the depth of the buffer
        // behind it. Ten thousand recorded events is the buffer at its full depth, and 0.0167 M an
        // event is the step a played frame takes at 60 fps and the default speed of 1 M/s, so this
        // is what the (t, r) diagram is handed after three minutes of playing. The drop is high
        // enough that the fall lasts the whole run: from 60 M the ring is 219 M away.
        let metric = KerrSchild::new(1.0, 0.90);
        let params = WorldlineParams::default();
        let mut obs = Observer::new_with_phi(&metric, "Bob", 0.0, 60.0, 0.0, 0.0, params);
        // Collected here rather than read off `obs.trail` on purpose: the cap is what this is
        // about, so the test states the depth itself instead of inheriting whatever it is set to.
        let mut input: Vec<Pos2> = Vec::new();
        // Four points per M on an 800-point canvas, roughly the zoom the diagram opens on.
        let to_screen = |r: f64, t: f64| Pos2::new(60.0 + 4.0 * r as f32, 580.0 - 4.0 * t as f32);
        for _ in 0..10_000 {
            obs.step(&metric, obs.t + 0.0167, 0.0167);
            input.push(to_screen(obs.r, obs.t));
        }
        let kept = thin_to_pixels(input.iter().copied(), |p| *p, SCREEN_SPACING);
        let worst = worst_departure(&input, &kept);
        println!(
            "{} recorded events, {:.1} M of fall to r = {:.2}, drawn as {} vertices; worst \
             departure {worst:.4} points",
            input.len(),
            obs.t,
            obs.r,
            kept.len()
        );
        assert!(obs.r > 1.0, "the fall should outlast the run: r = {}", obs.r);
        assert!(
            kept.len() * 2 < input.len(),
            "{} vertices out of {} is not a decoupling",
            kept.len(),
            input.len()
        );
        assert!(worst < SCREEN_SPACING, "the real trail departed by {worst} points");
    }
}
