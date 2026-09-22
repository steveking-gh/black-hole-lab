//! The distance scale every canvas measures itself with: a ruler along the bottom left edge,
//! from nought, ticked at a round step of the units in force and drawn at the view's own scale.
//!
//! One function, because the three canvases that carry it - the equatorial plane, the global
//! (t, r) foliation and an observer's rest frame - are drawing the same thing and a reader
//! comparing two of them is entitled to one style and one rule for the round step. What differs
//! between them is only what a point of screen measures: Cartesian distance in the Kerr-Schild
//! embedding on the equatorial view, the chart radius r on the foliation, and proper distance
//! along the drawn spacelike leg in a rest frame. Each caller passes its own points per M and,
//! where the quantity is not the plain radius the other two draw, says so in the caption.
//!
//! The ruler is deliberately not an info box. It is a scale, like the axis ticks: it never moves,
//! never shuts and carries no reading of its own, so it has nothing the box framework is for.

use crate::gui::axis;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use egui::{Pos2, Rect, Stroke};

/// Draw the ruler into `rect`, at `px_per_m` points of screen per M of the horizontal quantity.
///
/// The scale is recomputed from `px_per_m` on every frame, so a view that zooms itself - the rest
/// frame's automatic framing, the equatorial wheel - always carries a round step of the scale it
/// is drawn at rather than of the one it had when the user last touched it.
///
/// Pixels per km has no floor under it, because it has no natural one: a heavy hole at the widest
/// zoom is 1e-10 px/km, and a floor above that spaces the ticks for one scale and places them by
/// another. The step itself does have a floor, below which the labels would be all zeroes; past
/// that the whole ruler is dropped rather than drawn off the edge of the canvas.
pub(crate) fn draw_distance_ruler(
    painter: &egui::Painter,
    rect: Rect,
    px_per_m: f64,
    use_physical_units: bool,
    metric: &KerrSchild,
    font_scale: f32,
    caption: Option<&str>,
) {
    if !px_per_m.is_finite() || px_per_m <= 0.0 || rect.width() < 60.0 {
        return;
    }
    let extent_px = f64::from(rect.width().max(rect.height()));
    let (px_per_unit, step) = if use_physical_units {
        // One M of the horizontal quantity is this many km, by the same conversion every other
        // distance on screen is quoted through.
        let px_per_km = px_per_m / metric.r_to_km(1.0);
        (px_per_km, axis::round_step((extent_px * 0.7 / px_per_km / 5.0).max(1e-4)))
    } else {
        (px_per_m, axis::round_step((extent_px * 0.7 / px_per_m / 8.0).max(1e-6)))
    };
    let step_px = (step * px_per_unit) as f32;
    // A step that cannot be got on the canvas is one the floor above has lifted off the scale the
    // view is actually drawn at, and a ruler whose first tick is off the right edge measures
    // nothing. Say nothing instead.
    if !step_px.is_finite() || step_px <= 0.0 || step_px > rect.width() * 0.6 {
        return;
    }
    let font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
    let stroke = Stroke::new(1.0, Theme::TEXT_BRIGHT);
    let left = rect.left() + 24.0;
    let y = rect.bottom() - 10.0 - Theme::MIN_FONT_PT * font_scale;
    // As many steps as fit in the left half of the canvas, and one at the least.
    let steps = ((rect.width() * 0.5 / step_px).floor() as usize).clamp(1, 4);
    let right = left + step_px * steps as f32;
    painter.line_segment([Pos2::new(left, y), Pos2::new(right, y)], stroke);
    for k in 0..=steps {
        let x = left + step_px * k as f32;
        painter.line_segment([Pos2::new(x, y - 4.0), Pos2::new(x, y)], stroke);
        let value = step * k as f64;
        let label = if use_physical_units {
            metric.format_grid_km(value, step)
        } else {
            metric.format_grid_m(value, step)
        };
        painter.text(Pos2::new(x, y + 3.0), egui::Align2::CENTER_TOP, label, font.clone(), Theme::TEXT_BRIGHT);
    }
    // The caption, where the canvas measures something other than the radius the other views
    // measure, sits above the left end of the bar: the ticks reach 4 points up, the labels hang
    // below, and the bottom right of every one of these canvases belongs to the info boxes.
    if let Some(caption) = caption {
        painter.text(
            Pos2::new(left, y - 6.0),
            egui::Align2::LEFT_BOTTOM,
            caption,
            font,
            Theme::TEXT_MUTED,
        );
    }
}
