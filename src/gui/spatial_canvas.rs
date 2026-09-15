use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::river::RiverField;
use crate::gui::spacetime_canvas::TelemetryBoxes;
use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode};
use crate::physics::wavefront::SignalField;
use egui::{Color32, Pos2, Stroke, Vec2};

/// How the wavefronts of a transmission are drawn on this canvas: the two view settings of the
/// Simulation Control panel that decide what is put on screen *between* the calculated rays.
///
/// Both are drawing choices in the strict sense. `Pulse::scan` interpolates in (r, phi) along the
/// same segments whatever this struct says, so nothing here can move an arrival, change a measured
/// shift or touch the integration; what they change is how much of the picture is inference and how
/// much is the raw output of the integrator. They travel together in one value because they answer
/// the same question, and because two bare bools threaded through two call layers is how the wrong
/// one eventually gets passed.
#[derive(Clone, Copy)]
pub struct FrontStyle {
    /// Draw each segment of a front as the curve linear in (r, phi) between its two rays
    /// (`segment_arc`), or draw nothing between them and leave the front as its calculated points.
    /// See `FRONT_POINT_RADIUS`.
    pub arcs: bool,
    /// Drop the segments whose two rays have wound more than `MAX_RESOLVED_WINDING` apart, which
    /// the sampling cannot resolve and which the interpolation draws along a curve no ray took.
    pub hide_wound: bool,
}

/// One of the two observers, named for the things the equatorial view does per observer rather
/// than per frame of reference.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Who {
    Alice,
    Bob,
}

impl Who {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Alice => "Alice",
            Self::Bob => "Bob",
        }
    }

    /// The drawn radius of this observer's marker on the equatorial view, which also sets how
    /// close the pointer has to come to open their menu.
    pub(crate) fn marker_radius(self) -> f32 {
        match self {
            Self::Alice => 5.0,
            Self::Bob => 7.0,
        }
    }
}

/// How close to the ring a drag may put an observer. The ring is the curvature singularity and
/// the end of every worldline that reaches it, so a drop onto r = 0 is a drop onto nothing there is
/// a frame at.
const RING_DROP_FLOOR: f64 = 0.04;

/// A marker drag in progress: whose it is, and the Motion they were on when it started.
///
/// The Motion is kept because `Observer::set_drag_position` puts whoever is being moved into
/// `ObserverMode::ManualDrag` for as long as the pointer holds them - that is what a hand on the
/// marker means - and dropping them has to give back the worldline they were on. A free-faller is
/// released again at the event they were dropped at, on their own `Release`; a ZAMO goes back to
/// holding the new radius. Nothing selects that held state from the panel: it is the mechanism of
/// the drag and nothing else.
#[derive(Clone, Copy, PartialEq, Debug)]
struct MarkerDrag {
    who: Who,
    mode_before: ObserverMode,
    /// Marker centre minus pointer at the moment the pointer took hold, in screen pixels, added
    /// back on every frame of the drag. Without it the observer is placed *at* the pointer, so
    /// picking a marker up anywhere but dead centre teleports it under the cursor by as much as
    /// three marker radii before the drag has moved at all.
    grab: Vec2,
}

pub struct SpatialCanvas {
    pub zoom: f32,
    pub pan_offset: Vec2,
    /// The observer the view is being kept centred on, set from the canvas's right-click menu.
    ///
    /// It is a view setting, held here with the pan and the zoom rather than on the panel: it says
    /// where the canvas is looking and nothing about the physics, and like the zoom it survives a
    /// Reset. It is independent of the Frame of Reference selector, which centres the view on
    /// whoever's rest frame is being drawn; this one can keep Bob in the middle of a view drawn in
    /// the global foliation, which is the case the selector cannot express. Where the two disagree
    /// this one wins, being the more particular request, and it falls back to the selector's
    /// choice while the observer it names is not in the simulation.
    centred_on: Option<Who>,
    /// The marker drag in progress on this canvas, if any. Cleared when the pointer is released,
    /// when the observer being dragged leaves the simulation, and by `SpatialCanvas::end_drag` when
    /// the run is rebuilt under it.
    dragging: Option<MarkerDrag>,
    /// Where the user has dragged each info box on this canvas, per observer.
    pub telemetry: TelemetryBoxes,
    /// The animated raindrop flow. It is advanced from the app's own simulation clock, not from
    /// the frame rate, so it freezes when paused and steps with the arrow keys.
    pub river: RiverField,
}

impl Default for SpatialCanvas {
    fn default() -> Self {
        Self {
            zoom: 48.0, // pixels per M
            pan_offset: Vec2::ZERO,
            centred_on: None,
            dragging: None,
            telemetry: TelemetryBoxes::pinning(),
            river: RiverField::default(),
        }
    }
}

impl SpatialCanvas {
    /// Forget any marker drag in progress. Called when the run is rebuilt under the canvas - the
    /// observer being held is replaced by a fresh one from their card - so that the next pointer
    /// press starts a drag of the new worldline rather than continuing one of a worldline that no
    /// longer exists. Both markers are pickable again the moment the reset lands.
    pub fn end_drag(&mut self) {
        self.dragging = None;
    }

    /// Start, continue and finish a drag of either observer's marker.
    ///
    /// One set of rules for both of them. A press within three marker radii picks up the nearest
    /// marker under it; while the pointer holds it the observer stands at the chart point under the
    /// pointer, put there by `Observer::set_drag_position`, which is what a hand on the marker
    /// means: the worldline is being placed rather than integrated. Releasing hands them back the
    /// Motion they were on through `Observer::release_from_drag`, which releases them again at the
    /// event they were dropped at, so a drag asks "what if they were here" without answering the
    /// separate question of how they move.
    ///
    /// This is the view to ask it in. The plane it draws is where the two observers actually stand
    /// relative to one another, so a drag here sets both coordinates an equatorial observer has -
    /// the radius and the azimuth - and the pair can be put on opposite sides of the hole, which no
    /// control could say before. The (t, r) diagram, where this used to live, squeezes the whole
    /// plane onto one axis: a drag there could only ever slide somebody along the radius, and the
    /// other axis was a time, which is not a thing an observer can be moved along at all.
    ///
    /// The azimuth is free in a way the radius is not: Kerr is axisymmetric, so moving an observer
    /// in phi changes none of their constants - E and L and the whole radial problem are untouched
    /// - and only the *difference* in azimuth between the two observers means anything.
    ///
    /// A drag whose observer has left the simulation - their card unticked while the pointer is
    /// down - is dropped rather than carried, because there is no worldline left to place.
    #[allow(clippy::too_many_arguments)] // the same count the drawing paths carry
    fn drag_markers(
        &mut self,
        metric: &KerrSchild,
        response: &egui::Response,
        alice: Option<&mut Observer>,
        bob: Option<&mut Observer>,
        current_time: f64,
        to_screen: impl Fn(&Observer) -> Pos2,
        to_chart: impl Fn(Pos2) -> (f64, f64),
    ) {
        let mut markers: Vec<(Who, &mut Observer)> = Vec::new();
        if let Some(al) = alice {
            markers.push((Who::Alice, al));
        }
        if let Some(b) = bob {
            markers.push((Who::Bob, b));
        }

        if response.drag_started()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            let mut nearest: Option<(MarkerDrag, f32)> = None;
            for (who, obs) in markers.iter() {
                let reach = who.marker_radius() * 3.0;
                let at = to_screen(obs);
                let distance = at.distance(pointer);
                if distance < reach && nearest.is_none_or(|(_, best)| distance < best) {
                    let drag = MarkerDrag { who: *who, mode_before: obs.mode, grab: at - pointer };
                    nearest = Some((drag, distance));
                }
            }
            self.dragging = nearest.map(|(drag, _)| drag);
        }

        let Some(drag) = self.dragging else {
            return;
        };
        match markers.iter_mut().find(|(who, _)| *who == drag.who) {
            Some((_, obs)) => {
                if response.dragged()
                    && let Some(pointer) = response.interact_pointer_pos()
                {
                    let (r, phi) = to_chart(pointer + drag.grab);
                    obs.set_drag_position(current_time, r);
                    obs.phi = phi;
                }
                if response.drag_stopped() {
                    obs.release_from_drag(metric, drag.mode_before);
                    self.dragging = None;
                }
            }
            None => self.dragging = None,
        }
    }

    /// The observer the view is anchored to this frame, or None when it is anchored to the hole.
    ///
    /// Following an observer who is not in the simulation is following nobody, so the view stays on
    /// the hole rather than on a remembered position. A standing request to keep one of them
    /// centred is answered first, and the Frame of Reference selector's own tracking is what is
    /// left when there is no such request or the observer it names has gone.
    fn followed<'a>(
        &self,
        bob: &'a Option<Observer>,
        alice: &'a Option<Observer>,
        frame_of_ref: ReferenceFrame,
    ) -> Option<&'a Observer> {
        // A drag of the observer the view is holding on to suspends the hold: otherwise the pan
        // compensates for every pixel the pointer moves them, the marker sits pinned to the middle
        // of the canvas, and the drag looks like it is doing nothing at all. It resumes the moment
        // the pointer lets go, with the view re-centring on wherever they were put. That clause is
        // this canvas's alone - it is about the marker drag, which only this canvas offers - so it
        // is applied here and the rest of the rule is `frame_focus`, shared with the volume view.
        let held = self.dragging.map(|drag| drag.who);
        frame_focus(
            self.centred_on.filter(|who| held != Some(*who)),
            frame_of_ref,
            bob.as_ref(),
            alice.as_ref(),
        )
    }

    /// Pan the view so that the point `target` of the equatorial plane - Cartesian, in M, as
    /// `Observer::cartesian_position` gives it - sits in the middle of the canvas.
    ///
    /// One pan, not a standing request: whoever is there is free to move out of the middle again,
    /// which is what separates the menu's Goto items from its Keep Centered ones. The view's centre
    /// is `rect.center() + pan_offset - tracking`, so putting `target` in the middle means a pan of
    /// `tracking - target`, and when the view is already tracking that observer it is zero.
    pub fn look_at(
        &mut self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
        target: (f64, f64),
    ) {
        let zoom = self.zoom;
        let to_offset = |(x, y): (f64, f64)| Vec2::new(x as f32 * zoom, -(y as f32) * zoom);
        let tracking = self
            .followed(bob, alice, frame_of_ref)
            .map_or(Vec2::ZERO, |obs| to_offset(obs.cartesian_position(metric)));
        self.pan_offset = tracking - to_offset(target);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &mut Option<Observer>,
        alice: &mut Option<Observer>,
        current_time: f64,
        show_river: bool,
        signals: SignalViews<'_>,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
        style: FrontStyle,
        show_details: &mut bool,
    ) {
        let desired_size = egui::Vec2::new(ui.available_width(), canvas_height);
        let (response, painter) =
            ui.allocate_painter(desired_size, egui::Sense::click_and_drag());
        let rect = response.rect;

        if rect.width() < 30.0 || rect.height() < 30.0 {
            return;
        }

        // Mouse wheel zoom (refined to 1/8 step size for smooth control, cursor-centered)
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let step = 0.01875;
                let zoom_mult = if scroll > 0.0 { 1.0 + step } else { 1.0 - step };
                let old_zoom = self.zoom;
                let new_zoom = (old_zoom * zoom_mult).clamp(8.0, 500_000.0);
                if let Some(mpos) = response.hover_pos() {
                    let center_nom = rect.center() + self.pan_offset;
                    let cursor_vec = mpos - center_nom;
                    let ratio = new_zoom / old_zoom;
                    self.pan_offset += cursor_vec * (1.0 - ratio);
                }
                self.zoom = new_zoom;
            }
        }

        // Center of the canvas with frame of reference tracking
        // Every point of the equatorial plane is placed by the Kerr-Schild embedding
        // x + i y = (r + i a) e^{i phi}, never by (r cos psi, r sin psi).
        // Screen y grows downward; Cartesian y grows upward (matching the +Y tick labels), so flip it.
        // In a local rather than read through `self`, so that the closures built on it do not
        // hold a borrow of the canvas for as long as they live: the marker menu below takes
        // `&mut self` while they are still in scope.
        let zoom = self.zoom;
        let to_offset = |(x, y): (f64, f64)| Vec2::new(x as f32 * zoom, -(y as f32) * zoom);
        // Following an observer who is not in the simulation is following nobody, so the view
        // stays on the hole rather than jumping to a remembered position. A standing request to
        // keep one of them centred is answered first, and the Frame of Reference selector's own
        // tracking is what is left when there is no such request or the observer it names has
        // gone.
        let followed = self.followed(bob, alice, frame_of_ref);
        let frame_tracking_offset =
            followed.map_or(Vec2::ZERO, |obs| to_offset(obs.cartesian_position(metric)));
        let center = rect.center() + self.pan_offset - frame_tracking_offset;

        // Background
        painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();

        // A surface of constant r is the circle of Cartesian radius rho = sqrt(r^2 + a^2);
        // the ring singularity r = 0 is the circle rho = |a|.
        let rho_p = metric.cartesian_radius(rp);
        let rho_m = metric.cartesian_radius(rm);
        let rho_e = metric.cartesian_radius(re);
        let rho_ring = metric.a.abs();

        let r_to_px = |r: f64| -> f32 { (r as f32) * zoom };
        let to_screen =
            |(x, y): (f64, f64)| center + Vec2::new(x as f32 * zoom, -(y as f32) * zoom);
        // Direction vectors (velocities, tangents) need the same y flip as positions.

        // 0. Spatial Coordinate Axes (X and Y)
        let axis_stroke = Stroke::new(1.0, Color32::from_rgba_premultiplied(55, 65, 88, 120));
        if center.y >= rect.top() && center.y <= rect.bottom() {
            painter.line_segment([Pos2::new(rect.left(), center.y), Pos2::new(rect.right(), center.y)], axis_stroke);
        }
        if center.x >= rect.left() && center.x <= rect.right() {
            painter.line_segment([Pos2::new(center.x, rect.top()), Pos2::new(center.x, rect.bottom())], axis_stroke);
        }

        // Ticks and labels along X and Y axes
        if use_km {
            let px_per_km = zoom / (metric.r_grav_km() as f32);
            let max_span_km = ((rect.width().max(rect.height()) * 0.7) / px_per_km.max(1e-6)) as f64;
            let target_step = (max_span_km / 5.0).max(1e-4);
            let power = 10.0_f64.powf(target_step.log10().floor());
            let mantissa = target_step / power;
            let km_step = if mantissa < 1.5 { 1.0 * power } else if mantissa < 3.5 { 2.0 * power } else if mantissa < 7.5 { 5.0 * power } else { 10.0 * power };

            let mut km = km_step;
            while km <= max_span_km {
                let offset_px = (km as f32) * px_per_km;
                let km_str = metric.format_grid_km(km, km_step);
                // +X tick
                let px_x = center.x + offset_px;
                if px_x <= rect.right() - 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_x, center.y - 3.0), Pos2::new(px_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("+{}", km_str), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // -X tick
                let px_neg_x = center.x - offset_px;
                if px_neg_x >= rect.left() + 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_neg_x, center.y - 3.0), Pos2::new(px_neg_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_neg_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("-{}", km_str), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // +Y tick
                let py_pos = center.y - offset_px;
                if py_pos >= rect.top() + 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_pos), Pos2::new(center.x + 3.0, py_pos)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_pos), egui::Align2::LEFT_CENTER, format!("+{}", km_str), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // -Y tick
                let py_neg = center.y + offset_px;
                if py_neg <= rect.bottom() - 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_neg), Pos2::new(center.x + 3.0, py_neg)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_neg), egui::Align2::LEFT_CENTER, format!("-{}", km_str), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                km += km_step;
            }
        } else {
            let visible_m = (rect.width().max(rect.height()) as f64 / zoom as f64) * 0.7;
            let target_step = (visible_m / 8.0).max(1e-6);
            let power = 10.0_f64.powf(target_step.log10().floor());
            let mantissa = target_step / power;
            let r_step = if mantissa < 1.5 { 1.0 * power } else if mantissa < 3.5 { 2.0 * power } else if mantissa < 7.5 { 5.0 * power } else { 10.0 * power };

            let max_ticks = 15;
            let mut s = r_step;
            for _ in 0..max_ticks {
                let offset_px = (s as f32) * zoom;
                if offset_px > rect.width().max(rect.height()) * 0.8 {
                    break;
                }
                let label = metric.format_grid_m(s, r_step);

                // +X tick
                let px_x = center.x + offset_px;
                if px_x <= rect.right() - 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_x, center.y - 3.0), Pos2::new(px_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("+{}", label), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // -X tick
                let px_neg_x = center.x - offset_px;
                if px_neg_x >= rect.left() + 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_neg_x, center.y - 3.0), Pos2::new(px_neg_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_neg_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("-{}", label), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // +Y tick
                let py_pos = center.y - offset_px;
                if py_pos >= rect.top() + 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_pos), Pos2::new(center.x + 3.0, py_pos)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_pos), egui::Align2::LEFT_CENTER, format!("+{}", label), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                // -Y tick
                let py_neg = center.y + offset_px;
                if py_neg <= rect.bottom() - 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_neg), Pos2::new(center.x + 3.0, py_neg)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_neg), egui::Align2::LEFT_CENTER, format!("-{}", label), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::TEXT_MUTED);
                }
                s += r_step;
            }
        }

        // Axis Titles with Physical Conversion
        let phys_m_str = metric.format_physical_distance(1.0);
        let x_title = if use_km {
            "► Spatial x  [Kilometers (km)]".to_string()
        } else {
            format!("► Spatial x  [Units of M = GM/c² : 1M = {}]", phys_m_str)
        };
        painter.text(
            Pos2::new(rect.right() - 8.0, rect.bottom() - 4.0),
            egui::Align2::RIGHT_BOTTOM,
            x_title,
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Theme::TEXT_BRIGHT,
        );
        let y_title = if use_km {
            "▲ Spatial y  [Kilometers (km)]".to_string()
        } else {
            format!("▲ Spatial y  [Units of M = GM/c² : 1M = {}]", phys_m_str)
        };
        painter.text(
            Pos2::new(rect.right() - 8.0, rect.top() + 8.0),
            egui::Align2::RIGHT_TOP,
            y_title,
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Theme::TEXT_MUTED,
        );

        // 1. Concentric Zone Fills, every boundary at its Cartesian radius sqrt(r^2 + a^2).
        // Ergosphere: between r+ and the static limit 2M.
        painter.circle_filled(center, r_to_px(rho_e), Theme::ERGOSPHERE_FILL);

        // Region II: between r- and r+.
        painter.circle_filled(center, r_to_px(rho_p), Theme::REGION_II_FILL);

        // Region III: between the ring and r-.
        painter.circle_filled(center, r_to_px(rho_m), Theme::REGION_III_FILL);

        // The disc rho < a is the hole of the ring: it is not part of this sheet of the equatorial
        // plane at r > 0 at all, so it gets its own fill rather than a region colour.
        let ring_px = r_to_px(rho_ring);
        painter.circle_filled(center, ring_px.max(2.0), Theme::SINGULARITY_FILL);

        // 2. Concentric Boundary Rings
        // Ergosphere boundary
        painter.circle_stroke(center, r_to_px(rho_e), Stroke::new(1.5, Theme::ERGOSPHERE_LINE));

        // Outer Horizon r+
        painter.circle_stroke(center, r_to_px(rho_p), Stroke::new(2.5, Theme::HORIZON_OUTER));

        // Inner Cauchy Horizon r-
        painter.circle_stroke(center, r_to_px(rho_m), Stroke::new(2.0, Theme::HORIZON_CAUCHY));

        // Ring singularity r = 0: the circle of Cartesian radius exactly a, and inside it the
        // arrow that says which way the hole turns.
        draw_ring_spin_arrow(&painter, center, ring_px, metric.a);
        painter.circle_stroke(center, ring_px.max(2.0), Stroke::new(2.0, Theme::SINGULARITY_LINE));
        if ring_px >= 6.0 {
            painter.text(
                center + Vec2::new(0.0, ring_px + 4.0),
                egui::Align2::CENTER_TOP,
                "ring singularity r = 0 (ρ = a)",
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                Theme::SINGULARITY_LINE,
            );
        }

        // 3. River of Space: the E = 1, L = 0 raindrop congruence, drawn under the arrows, the
        // trails and the observer markers so it never competes with them for legibility.
        if show_river {
            self.river.draw(&painter, metric, &to_screen, zoom);
        }

        // 4. The two transmissions, drawn over the flow but under the worldlines and the markers,
        // so the fronts read as something moving through the field rather than as part of the
        // observers' own trajectories. Bob's goes down first and Alice's over it, so where the two
        // overlap it is the heavier, primary field that stays legible.
        draw_signal_field(
            &painter,
            metric,
            signals.bob,
            Theme::BOB_COLOR,
            Theme::SECONDARY_FRONT_WIDTH,
            style,
            &to_screen,
        );
        draw_signal_field(
            &painter,
            metric,
            signals.alice,
            Theme::ALICE_COLOR,
            1.0,
            style,
            &to_screen,
        );

        // 5. Both worldline trails, drawn together and before anything that sits on them: the
        // reception ticks below and the observers' own markers.
        if let Some(al) = alice {
            draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &to_screen);
        }
        if let Some(b) = bob {
            draw_spatial_trail(&painter, metric, b, Theme::BOB_COLOR, 1.5, &to_screen);
        }

        // 6b. Every arrival, marked on the *receiver's* trail in the *sender's* colour: amber
        // triangles where Bob received one of Alice's pulses, mint ones where Alice received one
        // of Bob's. Each is drawn at the receiver's own event, (r, phi) at the detection pass,
        // pushed through the same Kerr-Schild embedding as everything else on this canvas, so a
        // tick sits exactly on the trail it belongs to.
        //
        // The pairing is worth stating: the emission dots on a trail say what that observer sent,
        // the triangles on it say what they heard, and the colour of a triangle names who they
        // heard it from. Inside r+ the ticks bunch onto the r- circle, and that is the physics:
        // the frozen family of every pulse waits there, at fixed radius and co-rotating, until the
        // receiver falls through the stack, so a whole run of arrivals happens at one radius in
        // the last fraction of an M of the fall. The (t, r) diagram keeps colouring its reception
        // dots by the measured shift, which is a different question and stays where it can be read.
        let draw_reception_ticks = |field: &SignalField, sender: Color32| {
            for reception in field.receptions() {
                let at = to_screen(metric.cartesian_position(reception.r, reception.phi));
                if rect.contains(at) {
                    draw_reception_tick(&painter, at, sender);
                }
            }
        };
        // Each tick sits on the receiver's trail, so it is drawn only while that receiver is in
        // the simulation: Bob's transmission is received by Alice, and hers by him.
        if alice.is_some() {
            draw_reception_ticks(signals.bob, Theme::BOB_COLOR);
        }
        if bob.is_some() {
            draw_reception_ticks(signals.alice, Theme::ALICE_COLOR);
        }

        // 6. The observers themselves. Alice's info box is registered at the end of the frame,
        // after every other interaction on this canvas, so a drag on it does not pan. Bob's local
        // null cone is not drawn as a fan of stubs any more: he broadcasts the same pulses Alice
        // does, and a whole light cone integrated as exact null geodesics says everything the
        // twenty-four stubs said and keeps saying it as the light travels.

        // A worldline frozen on the far branch of r- has not stopped: it is riding the horizon's
        // own null generator, so its marker goes on creeping round the r- circle at Omega_- while
        // the radius and the observer's own clock stand still. Drawn, that is a dot moving
        // steadily along a circle, which is exactly what an ordinary orbit looks like from here.
        // The label is what tells the two apart, and it says which motion is left.
        let mark_if_frozen = |obs: &Observer, at: Pos2| {
            if obs.is_frozen() {
                painter.text(
                    Pos2::new(at.x + 6.0, at.y - 6.0),
                    egui::Align2::LEFT_BOTTOM,
                    "Frozen: gliding on the r₋ generator at Ω₋",
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                    Theme::TEXT_MUTED,
                );
            }
        };
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice
            && al.is_active
        {
            let al_pos = to_screen(al.cartesian_position(metric));
            painter.circle_filled(al_pos, Who::Alice.marker_radius(), Theme::ALICE_COLOR);
            mark_if_frozen(al, al_pos);
            alice_box = Some(al_pos);
        }
        let bob_box = bob.as_ref().map(|b| {
            // A plain dot, as Alice's is. He used to wear a white ring as well, which made the
            // ring below - the one that means something - read as a second decoration on a marker
            // that already had one, and gave two observers drawn from the same code two different
            // liveries for no reason. The colour and the radius tell them apart.
            let bob_pos = to_screen(b.cartesian_position(metric));
            painter.circle_filled(bob_pos, Who::Bob.marker_radius(), Theme::BOB_COLOR);
            mark_if_frozen(b, bob_pos);
            bob_pos
        });

        // The observer the view is holding on to wears a ring, and now it is the only ring either
        // marker can have, so that a picture which is no longer moving under a falling observer
        // says which one it is following.
        if let Some(centred) = self.centred_on
            && let Some(at) = match centred {
                Who::Alice => alice_box,
                Who::Bob => bob_box,
            }
        {
            let colour =
                if centred == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
            painter.circle_stroke(
                at,
                centred.marker_radius() + CENTRED_RING_GAP,
                Stroke::new(1.0, colour),
            );
        }

        // 6a. The drag, registered after the markers are drawn - their screen positions are what
        // a press is tested against - and before the canvas's own pan below, so that a press on a
        // marker moves the observer instead of the view.
        //
        // The pointer is mapped back through the embedding: a screen point is a Cartesian (x, y) of
        // the drawn plane, and `KerrSchild::chart_point` inverts x + iy = (r + ia)e^{i phi} to the
        // (r, phi) it came from. Inside the ring circle, rho < a, there is no equatorial point at
        // all, so that whole disc of the picture maps to the floor rather than to nothing.
        self.drag_markers(
            metric,
            &response,
            alice.as_mut(),
            bob.as_mut(),
            current_time,
            |obs| to_screen(obs.cartesian_position(metric)),
            |at| {
                let x = f64::from(at.x - center.x) / f64::from(zoom);
                let y = -f64::from(at.y - center.y) / f64::from(zoom);
                metric.chart_point(x, y, RING_DROP_FLOOR)
            },
        );

        // Pan with the mouse, unless a marker has the drag. It takes effect on the next frame,
        // since this frame's projection was fixed above.
        if response.dragged() && self.dragging.is_none() {
            self.pan_offset += response.drag_delta();
        }

        // 6b. The canvas's right-click menu: where the view should look, and what it should go
        // on looking at. It is offered anywhere on the canvas rather than on the markers, because
        // "take me to Bob" is most useful exactly when Bob is not on the screen to be pointed at,
        // and it is offered whether or not the run is moving, since none of it touches the physics.
        // It is registered on the canvas's own response, after the markers are drawn and before the
        // telemetry boxes, which take their drags last.
        //
        // Going to somebody and keeping them centred are different requests and both are here: the
        // first moves the view once and lets them walk out of it again, the second re-answers
        // itself every frame. A Goto while somebody is being followed pans away from them without
        // letting go, since the pan is measured from whatever the view is tracking.
        response.context_menu(|ui| {
            for (label, target) in [
                ("Goto Bob", bob.as_ref().map(|o| o.cartesian_position(metric))),
                ("Goto Alice", alice.as_ref().map(|o| o.cartesian_position(metric))),
                // The hole is at the origin of the embedding x + iy = (r + ia)e^{i phi}, which is
                // the centre of the ring rather than a point of the spacetime.
                ("Goto Black Hole", Some((0.0, 0.0))),
            ] {
                if ui.add_enabled(target.is_some(), egui::Button::new(label)).clicked() {
                    if let Some(at) = target {
                        self.look_at(metric, bob, alice, frame_of_ref, at);
                    }
                    ui.close();
                }
            }
            ui.separator();
            for who in [Who::Bob, Who::Alice] {
                let present = match who {
                    Who::Alice => alice.is_some(),
                    Who::Bob => bob.is_some(),
                };
                let mut centred = self.centred_on == Some(who);
                let label = format!("Keep {} Centered", who.name());
                if ui.add_enabled(present, egui::Checkbox::new(&mut centred, label)).changed() {
                    self.centred_on = centred.then_some(who);
                    ui.close();
                }
            }
        });

        // 7. Title and Legend Overlay
        // Horizon angular velocity Ω_H = a / (2 M r₊) is a rate per unit coordinate time, so in
        // geometric units it is a number per M; only dividing by t_g = GM/c³ makes it rad/s.
        let omega_h = metric.a / (2.0 * metric.m * rp);
        // The cut, stated on the canvas as well as in the panel's tip: a gap in a drawn front has
        // to say for itself that it is deliberate.
        let wound_line = if style.hide_wound {
            "Segments wound past a full turn are not drawn: the front there straddles a photon\n\
             orbit and two rays cannot resolve it\n"
        } else {
            ""
        };
        // What the view is holding on to, and how to ask it to hold on to somebody: a picture
        // that has stopped moving under a falling observer should say why, and the menu that did
        // it is not discoverable by looking at the canvas.
        let centred_line = match self.centred_on {
            Some(who) => {
                format!("Keeping {} centered (right-click anywhere to change)\n", who.name())
            }
            None => "Right-click anywhere: go to Bob, Alice or the hole, or keep one centered\n"
                .to_string(),
        };
        let legend_text = if !*show_details {
            // Collapsed: the view's name and the one number that changes under the mouse.
            match self.centred_on {
                Some(who) => format!(
                    "Equatorial View (θ = π/2)   🔍 {:.0} px/M   centered on {}",
                    self.zoom,
                    who.name()
                ),
                None => format!("Equatorial View (θ = π/2)   🔍 {:.0} px/M", self.zoom),
            }
        } else if use_km {
            format!(
                "Equatorial View (θ = π/2, x + iy = (r + ia) e^{{iϕ}})\n\
                 Cartesian radius ρ = √(r²+a²); ring singularity at ρ = a\n\
                 Units: Kilometers (km) & Seconds (s)\n\
                 Scale: 1M = {}\n\
                 Mass: {:.2e} M☉\n\
                 Outer Horizon r₊: {} ({:.2}M, ρ = {:.2}M)\n\
                 Cauchy Horizon r₋: {} ({:.2}M, ρ = {:.2}M)\n\
                 Spin a/M: {:.3}\n\
                 Drag: Ω_H = {:.3}/M = {:.3e} rad/s\n\
                 River: colour √(1−α²) vs ZAMO (1 at r₊); length √(2M/r) (1 at 2M)\n\
                 Front colour: ν an infaller here measures ÷ ν the infaller passing the emitter\n\
                 measured as it left: red ×1 (every front is born red), orange ×3, yellow ×10,\n\
                 green ×30, blue ×1000, violet ×100000, grey below ×1. One lightness throughout,\n\
                 so the colour carries the shift and nothing else\n\
                 Beaded arcs on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 {}\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour (amber = Alice → Bob, mint = Bob → Alice)\n\
                 {}\
                 🔍 Zoom: {:.0} px/M (scroll to zoom, drag the background to pan,\n\
                 drag either observer's marker to put them anywhere in the plane)",
                metric.format_physical_distance(1.0),
                metric.m_solar,
                metric.format_km(metric.r_to_km(rp)),
                rp,
                rho_p,
                metric.format_km(metric.r_to_km(rm)),
                rm,
                rho_m,
                metric.a_star(),
                omega_h,
                omega_h / metric.t_grav_seconds(),
                wound_line,
                centred_line,
                self.zoom,
            )
        } else {
            format!(
                "Equatorial View (θ = π/2, x + iy = (r + ia) e^{{iϕ}})\n\
                 Cartesian radius ρ = √(r²+a²); ring singularity at ρ = a\n\
                 Physical Scale: 1M = GM/c² = {}\n\
                 Time Scale:     1M/c = GM/c³ = {}\n\
                 Mass: {:.2e} M☉\n\
                 Outer Horizon r₊: {:.2}M ({}), ρ = {:.2}M\n\
                 Cauchy Horizon r₋: {:.2}M ({}), ρ = {:.2}M\n\
                 Spin a/M: {:.3}\n\
                 Drag: Ω_H = {:.3}/M\n\
                 River: colour √(1−α²) vs ZAMO (1 at r₊); length √(2M/r) (1 at 2M)\n\
                 Front colour: ν an infaller here measures ÷ ν the infaller passing the emitter\n\
                 measured as it left: red ×1 (every front is born red), orange ×3, yellow ×10,\n\
                 green ×30, blue ×1000, violet ×100000, grey below ×1. One lightness throughout,\n\
                 so the colour carries the shift and nothing else\n\
                 Beaded arcs on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 {}\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour (amber = Alice → Bob, mint = Bob → Alice)\n\
                 {}\
                 🔍 Zoom: {:.0} px/M (scroll to zoom, drag the background to pan,\n\
                 drag either observer's marker to put them anywhere in the plane)",
                metric.format_physical_distance(1.0),
                metric.format_physical_time(1.0),
                metric.m_solar,
                rp,
                metric.format_physical_distance(rp),
                rho_p,
                rm,
                metric.format_physical_distance(rm),
                rho_m,
                metric.a_star(),
                omega_h,
                wound_line,
                centred_line,
                self.zoom,
            )
        };

        // The Details button sits just above the block it shows and hides, set in the block's own
        // font so it reads as the block's first line. It is a widget placed over the canvas, so
        // it takes the click instead of the canvas drag.
        let legend_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let button_label = if *show_details { "▾ Details" } else { "▸ Details" };
        let button_size = Vec2::new(80.0 * font_scale, 16.0 * font_scale);
        let button_rect = egui::Rect::from_min_size(rect.left_top() + Vec2::new(8.0, 6.0), button_size);
        let button = egui::Button::new(
            egui::RichText::new(button_label).font(legend_font.clone()).color(Theme::TEXT_BRIGHT),
        )
        .frame(false);
        if ui
            .put(button_rect, button)
            .on_hover_text(
                "Show or hide the block of details below: horizon radii, scale, spin, and the \
                 colour keys.",
            )
            .clicked()
        {
            *show_details = !*show_details;
        }
        painter.text(
            Pos2::new(rect.left() + 10.0, button_rect.bottom() + 2.0),
            egui::Align2::LEFT_TOP,
            legend_text,
            legend_font,
            Theme::TEXT_BRIGHT,
        );

        // 8. Draggable info boxes, registered last so they take the drag instead of the canvas.
        if let (Some(al), Some(al_pos)) = (alice.as_ref(), alice_box) {
            self.telemetry.show(
                ui, &painter, "spatial", rect, al_pos, "Alice", Theme::ALICE_COLOR, al, metric, use_km,
                font_scale,
            );
        }
        if let (Some(b), Some(bob_pos)) = (bob.as_ref(), bob_box) {
            self.telemetry.show(
                ui, &painter, "spatial", rect, bob_pos, "Bob", Theme::BOB_COLOR, b, metric, use_km,
                font_scale,
            );
        }
    }
}

/// Which observer a canvas is anchored to, given a standing request to keep one centred and the
/// Frame of Reference selector: the rule both the equatorial view and the volume view follow.
///
/// Following an observer who is not in the simulation is following nobody, so the caller is handed
/// None and the view stays on the hole rather than on a remembered position. A standing request is
/// answered first, and the selector's own tracking is what is left when there is no such request or
/// the observer it names has gone. Where the two disagree the standing request wins, being the more
/// particular of the two: it can keep Bob in the middle of a view drawn in the global foliation,
/// which the selector cannot say.
pub(crate) fn frame_focus<'a>(
    centred: Option<Who>,
    frame: ReferenceFrame,
    bob: Option<&'a Observer>,
    alice: Option<&'a Observer>,
) -> Option<&'a Observer> {
    let observer = |who: Who| match who {
        Who::Alice => alice,
        Who::Bob => bob,
    };
    centred.and_then(observer).or(match frame {
        ReferenceFrame::Bob => bob,
        ReferenceFrame::Alice => alice,
        ReferenceFrame::DistantObserver => None,
    })
}

/// Three quarters of a turn of arrow inside the ring, pointing the way the hole rotates.
///
/// The disc rho < a is not part of this sheet of the equatorial plane at all - it is the hole of
/// the ring, which a worldline reaching r = 0 off the ring passes through into the r < 0 sheet -
/// so there is nothing to draw in there that would compete with it, and the one fact worth putting
/// in that space is the sense of the spin. Everything else on this canvas shows the rotation only
/// through what it does to something else: the ergosphere's bulge, the winding of a front, the
/// prograde arc that freezes on r_-. The arrow says it outright.
///
/// The direction is read off the sign of the spin parameter and from nothing else. Prograde is
/// increasing phi, and the embedding x + iy = (r + ia)e^{i phi} maps that to a counter-clockwise
/// turn in the drawn plane, so the arc is swept with phi and the y flip of `to_screen` is applied
/// here in the same way, by negating the sine. A hole with a = 0 has no ring and no sense of
/// rotation to draw, and one drawn too small to hold an arrowhead gets nothing rather than a blob.
fn draw_ring_spin_arrow(painter: &egui::Painter, center: Pos2, ring_px: f32, spin: f64) {
    const SWEEP: f64 = 1.5 * std::f64::consts::PI;
    const START: f64 = -0.75 * std::f64::consts::PI;
    const STEPS: usize = 96;
    if ring_px < RING_ARROW_MIN_PX || spin == 0.0 {
        return;
    }
    let radius = 0.60 * ring_px;
    let width = (0.10 * ring_px).clamp(1.5, 4.0);
    let sense = spin.signum();
    let at = |theta: f64| -> Pos2 {
        center + Vec2::new((radius as f64 * theta.cos()) as f32, -(radius as f64 * theta.sin()) as f32)
    };
    let points: Vec<Pos2> = (0..=STEPS)
        .map(|i| at(START + sense * SWEEP * (i as f64) / (STEPS as f64)))
        .collect();
    let end_theta = START + sense * SWEEP;
    let tip_end = *points.last().expect("the arc has STEPS + 1 points");
    painter.add(egui::Shape::line(points, Stroke::new(width, Theme::SINGULARITY_SPIN)));

    // The arrowhead, on the tangent at the end of the sweep: d/dtheta of the drawn point is
    // (-sin theta, -cos theta) once the y flip is in, and the sense of travel multiplies it.
    let tangent = Vec2::new(-(end_theta.sin()) as f32, -(end_theta.cos()) as f32) * sense as f32;
    let normal = Vec2::new(-tangent.y, tangent.x);
    let head = (0.34 * ring_px).clamp(4.0, 14.0);
    painter.add(egui::Shape::convex_polygon(
        vec![
            tip_end + tangent * head,
            tip_end - tangent * head * 0.35 + normal * head * 0.45,
            tip_end - tangent * head * 0.35 - normal * head * 0.45,
        ],
        Theme::SINGULARITY_SPIN,
        Stroke::NONE,
    ));
}

/// Draw every live wavefront of one transmission in the equatorial embedding.
///
/// The same code draws Alice's field and Bob's, because it is the same physics either way. What
/// tells them apart on screen is `emission_colour`, the colour of the dot marking each emission
/// event on its emitter's trail, and `width_scale`, which thins the secondary field's strokes (see
/// `Theme::SECONDARY_FRONT_WIDTH`). The gain colouring of the fronts themselves is not available
/// as an identifying mark: it is a measurement, and it has to mean the same thing in both fields.
///
/// The emitter broadcasts into their whole light cone, so each pulse is a *closed* polyline through
/// the Kerr-Schild positions of its surviving rays, ordered by emission angle and with the last ray
/// joined back to the first. Each segment is coloured by the *gain* its light has picked up since it
/// was let go:
///
///     gain = nu(a raindrop at the ray's current event) / nu(the raindrop passing the emitter as it left),
///
/// the two `f_factor` evaluations of `NullRay::gain_between`, with the ray's conserved energy and
/// affine scale cancelling out of the quotient. Both observers are drops of the E = 1, L = 0
/// congruence - free fall from rest at infinity - which is the one family of observers that exists
/// at every radius, inside both horizons included, so the colour means the same thing across the
/// whole picture rather than being quoted against a frame that stops existing at r+. It is a real
/// measured shift, the ordinary gravitational-plus-Doppler one between two members of that
/// congruence along the ray, and nothing about the emitter's own motion enters it.
///
/// That last property is why the front is coloured by this and not by the emitter-relative ratio the
/// receptions and the HUD quote. Held against the emitter, the rays of one pulse are already spread
/// across the whole ramp at the instant they leave - the prograde half aberrated blue, the retrograde
/// half red - so a fresh front is born split in two, which says something true about the emission but
/// nothing at all about where the light has since been. Held between raindrops, every ray of a pulse
/// starts at gain exactly 1, because at the emission event the two f_factors are the same number
/// computed twice: a new front comes out one uniform red and then earns its way up the ramp as
/// it falls, and what the colour then shows is what the light has gained on its way here. The gain
/// is carried *along* each segment rather than averaged over it: log10(gain) is interpolated
/// linearly between the two rays in the same loop coordinate the position is interpolated in, and
/// the polyline is cut into bands of at most `FRONT_BAND_DECADES` each, every band drawn at the
/// colour of its own midpoint (see `banded_segment`). Where the front is being torn apart - one ray
/// freezing onto r- while its neighbour crosses - a single segment spans the whole ramp, and one
/// mean colour said the far end had gained a hundred thousandfold when it had gained nothing.
///
/// Segments with a dead endpoint are skipped: a ray that has reached the ring is gone, and the front
/// genuinely ends there rather than jumping across the gap. Segments wound past
/// `MAX_RESOLVED_WINDING` are skipped too, when `FrontStyle::hide_wound` is set, because two samples
/// that far apart no longer bound a resolved piece of front; each of their two rays is then drawn as
/// its own dot instead, so the cut reads as a gap with marked ends rather than as a silent hole.
///
/// Every pulse also gets a dot at its own emission event, in the emitter's own colour. Without it
/// the nested loops inside r+ read as circles drawn around the hole, which is the wrong picture:
/// each loop is one pulse and encloses its emitter, because light is isotropic in the emitter's own
/// frame, and the flow then carries the whole loop inward. The dot sits on the emitter's trail at
/// the radius the pulse left them at, and the loop's outer edge never gets further from the hole
/// than that dot, which is the statement the drawing exists to make.
///
/// The frozen family is drawn twice over, heavily, because otherwise it cannot be seen at all.
/// Every ray whose `NullRay::inner_horizon_energy` is negative approaches r- as r - r- ~
/// exp(-kappa_- t) while co-rotating at Omega_-, so within a few M of coordinate time the whole arc
/// has collapsed to a fraction of a pixel of the magenta r- circle, where a hairline stroke leaves
/// it indistinguishable from the circle underneath. So: a segment with *both* ends frozen is drawn
/// at width 2 and at `Theme::FRONT_FROZEN_ALPHA`, and every frozen ray also gets a small filled dot,
/// so an arc squeezed below a pixel of width still reads as a beaded arc riding the Cauchy horizon.
/// All of it is drawn after every ordinary segment of every pulse, so no later front paints over it.
/// What is *not* special about it any more is its colour: it takes the same `Theme::front_colour` of
/// the same gain as every other segment, and the dots take their own ray's, so the exponential climb
/// of the frozen stack up the ramp - it is what runs the ramp out to a gain of 1e5 - is on screen
/// instead of being flattened into one marker colour. The extra weight is legibility and says
/// nothing about the physics. A segment with one frozen end and one crossing end is drawn in the
/// ordinary pass: that pair is the tear in the loop, where the front is being pulled apart into its
/// two families, and it belongs to neither.
///
/// The heavy pass is applied only inside r+. The sign of E - Omega_- L is a property of the ray
/// from birth, and outside r+ it is carried by most of the prograde half of every ring an emitter
/// sends - light that is nowhere near r- and may never get there, since a ray let go at 4.5 M with
/// that sign can escape just as well as fall. Drawn heavily out there it split each fresh ring into
/// a bright half and a faint one for no reason the picture could show, since the weight exists to
/// keep an arc visible once it has collapsed onto r-, and nothing outside r+ has. Inside r+ every
/// ray of the family is on its way to r- and reaches a pixel of it within a few M, which is where
/// the weight is needed and the only place it is now applied.
/// Largest azimuthal span of one drawn piece of a wavefront segment, in radians.
///
/// A segment of the front is the piece of null surface between two neighbouring rays, and what it
/// looks like in the equatorial plane is decided by the two rays' (r, phi), not by the straight
/// line between their screen positions. The two differ by nothing worth drawing while the rays are
/// close together in azimuth, and by the whole picture where they are not - which is the deep
/// interior. Inside r- the annulus that Region III occupies is thin (at a = 0.90 the embedding puts
/// r- at rho = 1.06 and the ring at rho = 0.90), and the rays there wind at wildly different rates:
/// dphi/dt reaches about -5 per M for a ray near the ring against +0.8 for one settling onto r-.
/// Neighbouring rays are then most of a radian apart, and the chord between them cuts straight
/// across the annulus and through the disk inside the ring, which drew as spikes into the
/// singularity that no ray ever took.
///
/// So each segment is drawn as the curve linear in (r, phi) between its two ends, over the *raw*
/// azimuth difference `to.1 - from.1`, cut into pieces no wider than this before each is embedded.
/// That is the same interpolation along the same segment that `Pulse::scan` uses to decide where
/// the front crosses a receiver, so what is drawn and what is detected are one thing. Two frozen
/// rays sitting on r- are now joined by an arc of the r- circle rather than by a chord dipping
/// inside it.
const MAX_ARC_STEP: f64 = 0.05;

/// Most pieces one segment of a front may be cut into, however far apart in azimuth its two ends
/// have wound.
///
/// Nothing in the physics bounds that separation. A ray let go on very nearly the critical impact
/// parameter hangs on one of the unstable circular photon orbits outside r+ (at a = 0.90 the
/// equatorial ones are at r = 1.56 prograde and r = 3.89 retrograde) for as long as it takes to
/// fall off them, going round and round while the neighbour it was emitted next to escapes or
/// falls in; the difference between the two grows for as long as that lasts, and all of it is
/// front. `test_a_wound_front_of_a_real_pulse_is_drawn_over_its_raw_azimuth_difference` measures
/// 3.7 turns in one segment of one ordinary pulse within 60 M, and the number has no ceiling in
/// it. The cap is therefore not a physical claim and not a cost measurement; it is the guarantee
/// that no state of the field can turn one segment of one pulse into an unbounded amount of work
/// in a frame that has to be drawn now. It sits far above anything a run has been seen to reach -
/// 2000 pieces at `MAX_ARC_STEP` is 100 radians, sixteen turns of the hole - and a segment that
/// did hit it would still be drawn over the whole of its span, in pieces coarser than
/// `MAX_ARC_STEP`, rather than truncated.
const MAX_ARC_PIECES: usize = 2000;

/// The azimuthal separation past which a neighbouring pair of rays no longer bounds a *resolved*
/// piece of front: one whole turn of the hole.
///
/// A segment joins two rays let go 2.5 degrees apart at the default sampling, and everything drawn
/// between them is the interpolation linear in (r, phi) of `segment_arc`. That is a faithful
/// picture of the front while the pair stays together, and it stops being one the moment the pair
/// straddles a critical impact parameter. At a = 0.90 the prograde equatorial photon orbit sits at
/// r_ph = 1.56, just outside r+ = 1.44: a ray let go marginally inside the critical angle spirals
/// in and freezes on r-, its neighbour marginally outside it hangs on r_ph for tens of M and then
/// escapes, and the real front between them is *pinned on that orbit* - a spiral in from the far
/// ray to r_ph, a pile-up of turns at r_ph that no sampling of the light cone can resolve, and a
/// spiral from r_ph down to r-. Two rays cannot carry that shape. What the interpolation draws
/// instead is an Archimedean spiral with its winding spread evenly over every radius between the
/// two ends, r+ included, and because the outer ray runs away at nearly c while the winding grows
/// only at Omega_-, those turns drift steadily outward across the outer horizon as the run goes on.
/// No ray does that, and nothing else in the picture does either: it is the one place where the arc
/// between two neighbours is not a statement about the front but an artefact of joining two samples
/// that no longer belong to one another.
///
/// A full turn is where the claim becomes indefensible rather than where the error becomes large.
/// Past 2 pi the two rays have gone round the hole relative to one another at least once, so the
/// segment covers every azimuth and the sampling has no information at all about what the front
/// does inside it: the drawn curve is then chosen entirely by the interpolation. Below it the arc
/// still misplaces the winding, but it is one arc between two neighbours on the same sheet, and it
/// is the same curve `Pulse::scan` tests. Only the two segments straddling the prograde and the
/// retrograde critical angle get there - about two per pulse - so dropping them removes those and
/// nothing else. Reception is not one of the things it removes: `Pulse::scan` interpolates along
/// every segment whether or not it is drawn, so an arrival happens at the same event either way.
const MAX_RESOLVED_WINDING: f64 = std::f64::consts::TAU;

/// The drawn polyline of one segment of a front: the curve linear in (r, phi) from one ray to the
/// next, embedded point by point. See `MAX_ARC_STEP`.
///
/// The azimuth difference is taken raw and is never folded into [-pi, pi], and that is exact rather
/// than a choice. Every ray of a pulse leaves the emission event at the emitter's own azimuth and
/// carries phi as a continuously integrated coordinate that is never reduced mod 2 pi (see
/// `NullRay::phi`), so the difference between two neighbouring rays is a continuous function of
/// time starting at zero: the integrated phi already *is* the unwrapped coordinate, and the raw
/// difference already is the physical winding between the pair. Folding agrees with it only while
/// |d_phi| < pi, which is exactly what a front that has wound around the hole violates: past half
/// a turn the fold flips the sign, and the arc that ran the long way round is redrawn the short
/// way, through the near side of the picture - spokes popping into existence across the drawing as
/// the fronts wind. Unfolded, a segment several turns long is drawn as several turns of arc, which
/// is what that piece of the front is. Frame dragging inside r+ pulls neighbouring rays apart by
/// most of a radian, which is what `MAX_ARC_STEP` is for; what takes a pair past half a turn is a
/// ray hung on a circular photon orbit outside r+, which is what `MAX_ARC_PIECES` is for.
fn segment_arc<F: Fn((f64, f64)) -> Pos2>(
    metric: &KerrSchild,
    from: (f64, f64),
    to: (f64, f64),
    to_screen: &F,
) -> Vec<Pos2> {
    let d_phi = to.1 - from.1;
    let pieces = (d_phi.abs() / MAX_ARC_STEP).ceil().max(1.0).min(MAX_ARC_PIECES as f64) as usize;
    (0..=pieces)
        .map(|k| {
            let s = (k as f64) / (pieces as f64);
            let r = from.0 + s * (to.0 - from.0);
            let phi = from.1 + s * d_phi;
            to_screen(metric.cartesian_position(r, phi))
        })
        .collect()
}

/// The radius, times the field's stroke scale, of the dot each calculated point of a front is
/// drawn as: every live ray when the arcs are switched off, and the frozen family's beads always.
///
/// The Arcs between wavefront points checkbox is a drawing choice and only a drawing choice. On,
/// each segment of a front between two neighbouring rays is drawn as the curve of `segment_arc`,
/// the same interpolation in (r, phi) that `Pulse::scan` uses to test the front against a
/// receiver, so the drawn front is the curve the detector is testing. Off, nothing is drawn between
/// the rays at all: the front is shown as the calculated points themselves, one dot per live ray,
/// which is the raw output of the integrator with no interpolation of any kind laid over it. That
/// is worth being able to see, because everything an arc adds is inference - a segment between two
/// rays most of a radian apart in the deep interior is drawn along a curve no ray was integrated
/// on - and the dots are the part that is not. Nothing about a reception moves when the box is
/// unticked; `Pulse::scan` interpolates along the same segments either way.
///
/// The same dot marks each end of a segment dropped for winding past `MAX_RESOLVED_WINDING`, in
/// that ray's own gain colour and at the ordinary `Theme::SHIFT_ALPHA`: those two calculated points
/// are still calculated points, and it is only the inference between them that has been withdrawn.
/// A ray already drawn as a dot - every live ray in points-only mode, and the frozen family's
/// heavier beads - is not drawn twice for it.
const FRONT_POINT_RADIUS: f32 = 1.6;

/// Below this drawn radius the ring is a dot on the screen and the spin arrow is not drawn: it
/// would be a smear over the ring's own stroke rather than an arrow. A hole of a = 0.90 at the
/// default zoom is well above it; zooming out far enough takes the arrow away and leaves the ring.
const RING_ARROW_MIN_PX: f32 = 14.0;

/// How far outside their own marker the ring around a centred observer is drawn.
pub(crate) const CENTRED_RING_GAP: f32 = 5.0;

/// How much of the gain ramp one flat-coloured band of a segment may cover, in decades of gain.
///
/// A segment is drawn in the colour of the gain its light carries, and its two rays need not carry
/// anything like the same gain. Where the front is being torn apart - one ray settling onto r- and
/// climbing like exp(kappa_- t) up the ramp while its neighbour crosses and is gone - a single
/// segment runs from gain 1 to gain 1e5: five decades, the whole ramp from red to violet.
/// Painted in one colour that segment is violet along its entire length, which says the light at
/// the far end has gained a hundred thousandfold when it has gained nothing at all, and the ramp
/// then reads as a jump at a ray rather than as the climb along the front that it is.
///
/// So the gain is carried along the segment: log10(gain) is interpolated linearly in the same loop
/// coordinate s that `segment_arc` interpolates the position in, and the polyline is cut into bands
/// of at most this many decades, each drawn at the colour of its own midpoint. A quarter of a
/// decade is under half the narrowest leg of `Theme::FRONT_STOPS` - the half-decade from red
/// to orange - so no band can straddle a stop of the ramp unnoticed, and the five-decade case costs
/// twenty polylines where it used to cost one. The interpolation is in log10 because that is the
/// coordinate the ramp itself is keyed to, so a band is a fixed slice of the drawn ramp rather than
/// a fixed slice of a quantity spanning five orders of magnitude.
const FRONT_BAND_DECADES: f64 = 0.25;

/// Most bands one segment may be cut into: the same kind of guarantee `MAX_ARC_PIECES` makes about
/// the pieces.
///
/// The clamp `Theme::front_colour` puts on log10(gain) already holds any segment to the six decades
/// of the ramp, so twenty-four bands is the most the physics can ask for and this cap is not
/// reached in any state the field can be in. It is here so that no later widening of the ramp, and
/// no unphysical gain that finds its way past the clamps, can turn one segment of one pulse into
/// unbounded work in a frame that has to be drawn now.
const MAX_FRONT_BANDS: usize = 64;

/// log10 of a gain, clamped to the two ends of the wavefront ramp exactly as `Theme::front_colour`
/// clamps it, so that interpolating between two of these and colouring the result agrees with
/// colouring the two ends directly. A non-positive or non-finite gain, which no real measurement
/// produces, sits at the dark end.
fn front_log(gain: f64) -> f64 {
    if gain.is_finite() && gain > 0.0 {
        gain.log10().clamp(Theme::FRONT_LOG_MIN, Theme::FRONT_LOG_MAX)
    } else {
        Theme::FRONT_LOG_MIN
    }
}

/// Cut one drawn segment into bands of nearly constant gain: the polyline of `segment_arc` split at
/// shared boundary points, each piece paired with the gain to colour it by. See
/// `FRONT_BAND_DECADES`.
///
/// `gain_from` belongs to the ray at s = 0 and `gain_to` to the ray at s = 1, the same loop
/// coordinate `segment_arc` walks the position along, so band b covers the s-range from b/bands to
/// (b + 1)/bands of that same curve and is coloured at the gain interpolated to its midpoint. The
/// interpolation is linear in log10(gain), clamped as the ramp clamps it.
///
/// Consecutive bands share their boundary point rather than abutting, so the drawn front has no
/// gaps in it: the concatenation of the bands, each shared point counted once, is the original
/// polyline in order. A segment whose two ends carry the same gain is one band over the whole arc,
/// which is the common case and costs nothing over drawing it directly. Asking for more bands than
/// the arc has pieces would need boundary points that are not on the polyline, so the count falls
/// back to the piece count instead: the colour of a two-point segment is then quantised more
/// coarsely than `FRONT_BAND_DECADES` asks for, which is all two points can carry anyway.
fn banded_segment(arc: Vec<Pos2>, gain_from: f64, gain_to: f64) -> Vec<(Vec<Pos2>, f64)> {
    let (log_from, log_to) = (front_log(gain_from), front_log(gain_to));
    let gain_at = |s: f64| 10.0_f64.powf(log_from + s * (log_to - log_from));
    let pieces = arc.len().saturating_sub(1);
    let wanted = ((log_to - log_from).abs() / FRONT_BAND_DECADES).ceil().max(1.0) as usize;
    let bands = wanted.clamp(1, MAX_FRONT_BANDS).min(pieces.max(1));
    if bands <= 1 || pieces == 0 {
        return vec![(arc, gain_at(0.5))];
    }
    (0..bands)
        .map(|b| {
            // Integer bounds, so band b ends exactly where band b + 1 begins and the last ends on
            // the final point of the arc: the bands tile the segment with no gap and no overlap
            // beyond the single point each consecutive pair shares.
            let start = b * pieces / bands;
            let end = (b + 1) * pieces / bands;
            let s_mid = 0.5 * ((start + end) as f64) / (pieces as f64);
            (arc[start..=end].to_vec(), gain_at(s_mid))
        })
        .collect()
}

pub(crate) fn draw_signal_field<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    signal: &SignalField,
    emission_colour: Color32,
    width_scale: f32,
    style: FrontStyle,
    to_screen: &F,
) {
    // `derivatives` reads only (E, L) off the state and takes the radius as an argument, so one
    // instance of the raindrop congruence serves every ray of every pulse, as it does in the river.
    let raindrop = GeodesicState::new_infall(metric, 0.0, 12.0, 1.0, 0.0);
    // The emitter's colour, faint: present enough to read as a mark on their trail, quiet enough
    // not to compete with the wavefront it anchors.
    let dot = Color32::from_rgba_unmultiplied(
        emission_colour.r(),
        emission_colour.g(),
        emission_colour.b(),
        150,
    );
    // The frozen family of every pulse, held back and drawn last so that it lies on top of both the
    // r- circle and the ordinary fronts it is buried in. Each carries its own colour, which is the
    // same gain colouring every other segment gets: only the weight and the opacity are special.
    let mut frozen_segments: Vec<(Vec<Pos2>, Color32)> = Vec::new();
    let mut frozen_dots: Vec<(Pos2, Color32)> = Vec::new();
    for pulse in signal.pulses.iter() {
        // A spent pulse is kept in the field so that stepping backwards can bring it back, but it
        // has no front left to draw and no dot to anchor.
        if !pulse.rays.iter().any(|ray| ray.alive()) {
            continue;
        }
        let n = pulse.rays.len();
        if n < 2 {
            continue;
        }
        // The drop that was passing the emitter as this pulse left, which is the denominator of
        // every ray's gain: one per pulse, because every ray of a pulse left the same event.
        let u_emit = {
            let (ut, ur, up) = raindrop.derivatives(metric, pulse.emitted_r);
            [ut, ur, up]
        };
        let gains: Vec<f64> = pulse
            .rays
            .iter()
            .map(|ray| {
                if !ray.alive() {
                    return 1.0;
                }
                let (ut, ur, up) = raindrop.derivatives(metric, ray.r);
                ray.gain_between(metric, pulse.emitted_r, &u_emit, &[ut, ur, up])
            })
            .collect();
        // One classification and one projection per ray per frame, both of which the segment loop
        // would otherwise repeat for each of the two segments a ray belongs to.
        // "Frozen" for the drawing is the frozen family *inside r+*, the only place the heavy pass
        // is needed: see the doc above `MAX_ARC_STEP`.
        let r_plus = metric.outer_horizon();
        let frozen: Vec<bool> = pulse
            .rays
            .iter()
            .map(|ray| ray.alive() && ray.r < r_plus && ray.frozen(metric))
            .collect();
        // The ray positions themselves, which the frozen dots sit on; the segments between them
        // are drawn as arcs in (r, phi) between them, or not at all when the arcs are off.
        let points: Vec<Pos2> = pulse
            .rays
            .iter()
            .map(|ray| to_screen(metric.cartesian_position(ray.r, ray.phi)))
            .collect();

        // The two ends of every segment dropped for winding: each is a live calculated point whose
        // segment has been withdrawn, and each is drawn as its own dot below so that the cut reads
        // as a gap with marked ends rather than as a silent hole in the front.
        let mut cut_end = vec![false; n];
        // n segments rather than n - 1: the closing one runs from the last ray back to the first.
        // With the arcs off there are no segments at all, only the points below.
        for i in (0..n).filter(|_| style.arcs) {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            // The raw, unfolded winding between the pair, which is the span `segment_arc` would
            // draw this segment over. Past a whole turn the two samples no longer bound a resolved
            // piece of front and the curve between them is the interpolation's own invention, so
            // nothing is drawn there and the two ends are marked instead. See
            // `MAX_RESOLVED_WINDING`.
            if style.hide_wound
                && (pulse.rays[j].phi - pulse.rays[i].phi).abs() > MAX_RESOLVED_WINDING
            {
                cut_end[i] = true;
                cut_end[j] = true;
                continue;
            }
            let arc = segment_arc(
                metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                to_screen,
            );
            // The gain is carried along the segment rather than averaged over it: one polyline per
            // band of at most `FRONT_BAND_DECADES`, and a single band over the whole arc in the
            // common case where the two rays carry the same gain.
            let frozen_pair = frozen[i] && frozen[j];
            let alpha = if frozen_pair { Theme::FRONT_FROZEN_ALPHA } else { Theme::SHIFT_ALPHA };
            for (band, gain) in banded_segment(arc, gains[i], gains[j]) {
                if frozen_pair {
                    frozen_segments.push((band, Theme::front_colour(gain, alpha)));
                } else {
                    painter.add(egui::Shape::line(
                        band,
                        Stroke::new(1.2 * width_scale, Theme::front_colour(gain, alpha)),
                    ));
                }
            }
        }
        for (i, point) in points.iter().enumerate() {
            if !pulse.rays[i].alive() {
                continue;
            }
            let alpha = if frozen[i] { Theme::FRONT_FROZEN_ALPHA } else { Theme::SHIFT_ALPHA };
            if frozen[i] {
                // A frozen ray already carries a bead, which is this same dot drawn heavier, so it
                // is not drawn a second time for being the end of a cut segment.
                frozen_dots.push((*point, Theme::front_colour(gains[i], alpha)));
            } else if !style.arcs || cut_end[i] {
                // The calculated point itself: with the arcs on it is implied by the two segments
                // meeting there, with them off it is all there is of this ray, and at the end of a
                // segment dropped for winding it is what is left once the inference is withdrawn.
                painter.circle_filled(
                    *point,
                    FRONT_POINT_RADIUS * width_scale,
                    Theme::front_colour(gains[i], alpha),
                );
            }
        }
        // The anchor: where on Alice's trail this loop was let go of.
        let emitted = to_screen(metric.cartesian_position(pulse.emitted_r, pulse.emitted_phi));
        painter.circle_filled(emitted, 2.0, dot);
    }

    for (segment, colour) in frozen_segments {
        painter.add(egui::Shape::line(segment, Stroke::new(2.0 * width_scale, colour)));
    }
    for (point, colour) in frozen_dots {
        painter.circle_filled(point, FRONT_POINT_RADIUS * width_scale, colour);
    }
}

/// One arrival, as a mark on the receiver's trail: a small filled triangle, apex up, in the
/// colour of whoever sent the pulse.
///
/// The shape is what separates it from the emission dots already on that trail, and the colour is
/// what says which transmission it belongs to; a hairline white outline keeps it legible where it
/// lands on top of a shift-coloured front, which near r- is most of the time. The orientation is
/// fixed rather than aligned with anything: an arrival has a direction on the sky, but not one
/// this projection could draw honestly.
const RECEPTION_TICK_RADIUS: f32 = 4.0;

pub(crate) fn draw_reception_tick(painter: &egui::Painter, at: Pos2, colour: Color32) {
    let (h, w) = (RECEPTION_TICK_RADIUS, RECEPTION_TICK_RADIUS * 0.866);
    painter.add(egui::Shape::convex_polygon(
        vec![
            Pos2::new(at.x, at.y - h),
            Pos2::new(at.x + w, at.y + h * 0.5),
            Pos2::new(at.x - w, at.y + h * 0.5),
        ],
        colour,
        Stroke::new(0.5, Color32::WHITE),
    ));
}

/// Faint spatial trajectory of an observer: the recorded (t, r, phi) trail pushed through the
/// Kerr-Schild embedding x + i y = (r + i a) e^{i phi}. With E = 1, L = 0 the curve spirals in and,
/// for a spinning hole, terminates on the ring rho = a rather than at the origin.
pub(crate) fn draw_spatial_trail<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    obs: &Observer,
    color: Color32,
    width: f32,
    to_screen: &F,
) {
    if obs.trail.len() < 2 {
        return;
    }
    let points: Vec<Pos2> = obs
        .trail
        .iter()
        .map(|point| to_screen(metric.cartesian_position(point.r, point.phi)))
        .collect();
    let faint = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 120);
    painter.add(egui::Shape::line(points, Stroke::new(width, faint)));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The embedded radius of a chart point: |x + i y| = |(r + i a) e^{i phi}| = sqrt(r^2 + a^2),
    /// so a curve of constant r is a circle in the drawing and this is its radius.
    fn embedded_radius(point: Pos2) -> f64 {
        ((point.x as f64).powi(2) + (point.y as f64).powi(2)).sqrt()
    }

    /// The arc of the spin arrow as `draw_ring_spin_arrow` puts it on a painter: the polyline in
    /// `Theme::SINGULARITY_SPIN`, in the order it is drawn, taken relative to the ring's centre.
    fn spin_arrow_arc(spin: f64, ring_px: f32) -> Vec<(f64, f64)> {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let centre = Pos2::new(200.0, 200.0);
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            draw_ring_spin_arrow(&painter, centre, ring_px, spin);
        });
        let mut arc = Vec::new();
        fn walk(shape: &egui::Shape, centre: Pos2, arc: &mut Vec<(f64, f64)>) {
            match shape {
                egui::Shape::Path(path) => {
                    let solid = matches!(
                        path.stroke.color,
                        egui::epaint::ColorMode::Solid(c) if c == Theme::SINGULARITY_SPIN
                    );
                    if solid {
                        arc.extend(path.points.iter().map(|p| {
                            ((p.x - centre.x) as f64, (p.y - centre.y) as f64)
                        }));
                    }
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, centre, arc);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, centre, &mut arc);
        }
        output.drop_without_applying_deltas();
        arc
    }

    #[test]
    fn test_the_ring_carries_an_arrow_three_quarters_round_in_the_sense_of_the_spin() {
        // What the arrow inside the ring has to say, measured off the painter rather than off the
        // helper's arithmetic. It has to be *inside* the ring, since the disc rho < a is the space
        // it is drawn in and an arc spilling over the ring's own circle would read as something
        // crossing it. It has to go three quarters of the way round, because that is what says
        // "turning" rather than "a mark at an angle". And it has to turn the way the hole turns:
        // prograde is increasing phi, which the embedding x + iy = (r + ia)e^{i phi} draws
        // counter-clockwise, so on a screen whose y runs downward consecutive points must cross
        // *negatively*, and a hole spun the other way must reverse every one of those crossings.
        // A ring too small to hold an arrowhead gets no arrow at all.
        let ring_px = 60.0_f32;
        for &(spin, name, want_sign) in
            &[(0.90_f64, "prograde", -1.0_f64), (-0.90, "retrograde", 1.0)]
        {
            let arc = spin_arrow_arc(spin, ring_px);
            assert!(arc.len() > 32, "{name}: the arc is a polyline, got {} points", arc.len());
            let mut swept = 0.0;
            for pair in arc.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                let cross = a.0 * b.1 - a.1 * b.0;
                let dot = a.0 * b.0 + a.1 * b.1;
                assert!(
                    cross * want_sign > 0.0,
                    "{name}: the arc must turn one way only, got a crossing of {cross}"
                );
                swept += cross.atan2(dot).abs();
            }
            let turn = swept / std::f64::consts::TAU;
            assert!(
                (turn - 0.75).abs() < 1e-6,
                "{name}: the arrow sweeps {turn} of a turn, and must sweep three quarters"
            );
            let radii: Vec<f64> = arc.iter().map(|(x, y)| (x * x + y * y).sqrt()).collect();
            let widest = radii.iter().fold(0.0_f64, |m, r| m.max(*r));
            assert!(
                widest < ring_px as f64,
                "{name}: the arc reaches {widest} px, outside the ring at {ring_px}"
            );
            println!("{name} spin: {} points, {turn:.3} of a turn at up to {widest:.1} px inside a ring of {ring_px} px", arc.len());
        }
        assert!(
            spin_arrow_arc(0.90, RING_ARROW_MIN_PX - 0.1).is_empty(),
            "a ring too small for an arrowhead is left alone"
        );
        assert!(spin_arrow_arc(0.0, ring_px).is_empty(), "and a hole with no spin has no arrow");
    }

    /// One real frame of the equatorial view, returning every filled circle it painted - colour,
    /// radius and centre - and the id of the `Ui` it was drawn in, which is what the marker menus
    /// are keyed off.
    fn spatial_frame(
        canvas: &mut SpatialCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        alice: &mut Option<Observer>,
        bob: &mut Option<Observer>,
        events: Vec<egui::Event>,
    ) -> (Vec<(Color32, f32, Pos2)>, egui::Id) {
        use crate::physics::wavefront::SignalField;
        let signal = SignalField::default();
        let mut details = true;
        let mut ui_id = egui::Id::NULL;
        // The observers' own t, because in the app every worldline stands at the simulation clock,
        // and a drag places them at it.
        let clock = alice.as_ref().or(bob.as_ref()).map_or(0.0, |obs| obs.t);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            events,
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            ui_id = ui.id();
            canvas.render(
                ui,
                metric,
                bob,
                alice,
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
        let mut circles = Vec::new();
        fn walk(shape: &egui::Shape, out: &mut Vec<(Color32, f32, Pos2)>) {
            match shape {
                egui::Shape::Circle(c) => out.push((c.fill, c.radius, c.center)),
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, out);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut circles);
        }
        output.drop_without_applying_deltas();
        (circles, ui_id)
    }

    /// Where the hole was painted this frame: the ring's fill is drawn once, at the centre of the
    /// geometry. With no pan and nobody being tracked it is also the middle of the canvas, which is
    /// how a test knows where the middle is without knowing the layout.
    fn hole_of(circles: &[(Color32, f32, Pos2)]) -> Pos2 {
        circles
            .iter()
            .find(|(fill, _, _)| *fill == Theme::SINGULARITY_FILL)
            .map(|(_, _, at)| *at)
            .expect("the ring's fill marks the centre of the hole")
    }

    /// Where the observer's own marker was painted this frame.
    fn marker_of(circles: &[(Color32, f32, Pos2)], who: Who) -> Pos2 {
        let colour = if who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        circles
            .iter()
            .find(|(fill, r, _)| *fill == colour && (*r - who.marker_radius()).abs() < 1e-6)
            .map(|(_, _, at)| *at)
            .unwrap_or_else(|| panic!("{}'s marker is painted", who.name()))
    }

    /// The text of every galley the equatorial view painted for one frame with this observer on
    /// it. Fonts are left empty, as they are everywhere else a frame is run in a test: a galley
    /// carries its string whether or not there are glyphs to draw it with.
    fn spatial_frame_text(metric: &KerrSchild, bob: &mut Option<Observer>) -> String {
        use crate::physics::wavefront::SignalField;
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let signal = SignalField::default();
        let mut details = true;
        let mut alice: Option<Observer> = None;
        let clock = bob.as_ref().map_or(0.0, |obs| obs.t);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            canvas.render(
                ui,
                metric,
                bob,
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
        fn collect(shape: &egui::Shape, out: &mut String) {
            match shape {
                egui::Shape::Text(text) => {
                    out.push_str(text.galley.text());
                    out.push('\n');
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        collect(shape, out);
                    }
                }
                _ => {}
            }
        }
        let mut text = String::new();
        for clipped in output.shapes.iter() {
            collect(&clipped.shape, &mut text);
        }
        output.drop_without_applying_deltas();
        text
    }

    #[test]
    fn test_a_frozen_observer_is_labelled_frozen_on_the_equatorial_view() {
        // On this view a worldline frozen on the far branch of r- is a dot creeping round a
        // circle at a steady rate, which is exactly what an ordinary orbit looks like. It is not
        // one - the radius and the observer's own clock have stopped, and the motion left is the
        // horizon's null generator carrying him - so the marker has to say so, and only when it
        // is true: a Bob still falling gets no such label.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut frozen = Some(Observer::frozen_bob(&metric));
        let text = spatial_frame_text(&metric, &mut frozen);
        assert!(
            text.contains("Frozen"),
            "the frozen marker is unlabelled; the view painted:\n{text}"
        );

        let mut falling = Some(Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            9.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::new(1.0, 2.2, false),
        ));
        falling.as_mut().unwrap().step(&metric, 0.25, 0.25);
        assert!(!falling.as_ref().unwrap().is_frozen(), "he has only just been let go of");
        let text = spatial_frame_text(&metric, &mut falling);
        assert!(
            !text.contains("Frozen"),
            "a falling observer is labelled frozen; the view painted:\n{text}"
        );
    }

    #[test]
    fn test_keeping_an_observer_centred_pans_the_view_under_them_instead_of_moving_them() {
        // What the toggle on the marker's right-click menu has to do: with it off, an observer
        // falling inward crosses the canvas while the hole stays put, which is the ordinary view;
        // with it on, the observer holds still in the middle and the hole - and with it every
        // horizon, the ring and everything else drawn in the geometry - slides past instead. The
        // two are the same picture from a different place, which is the whole content of "keep
        // this one centred", and it is independent of the Frame of Reference selector: this is
        // drawn in the global foliation throughout.
        let metric = KerrSchild::new(1.0, 0.65);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::default(),
        ));
        let fall = |bob: &mut Option<Observer>| {
            let b = bob.as_mut().expect("Bob is in this run");
            for _ in 0..10 {
                b.step(&metric, b.t + 0.1, 0.1);
            }
        };
        // Off: Bob moves, the hole does not.
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let bob_moved = (marker_of(&after, Who::Bob) - marker_of(&before, Who::Bob)).length();
        let hole_moved = (hole_of(&after) - hole_of(&before)).length();
        assert!(bob_moved > 5.0, "he crosses the canvas as he falls: {bob_moved} px");
        assert!(hole_moved < 1e-3, "and the hole stays where it is: {hole_moved} px");

        // On: Bob does not move, the hole does, and by exactly what his own motion would have been.
        canvas.centred_on = Some(Who::Bob);
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (moved, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let bob_held = (marker_of(&moved, Who::Bob) - marker_of(&before, Who::Bob)).length();
        let hole_slid = hole_of(&moved) - hole_of(&before);
        println!(
            "uncentred: Bob moved {bob_moved:.1} px and the hole {hole_moved:.3} px; centred: Bob \
             moved {bob_held:.3} px and the hole {:.1} px",
            hole_slid.length()
        );
        assert!(bob_held < 1e-3, "centred, he holds still: {bob_held} px");
        assert!(hole_slid.length() > 5.0, "and the geometry slides past him: {hole_slid:?}");
        // He is held where the ring around his marker says he is: the same point of the canvas as
        // the frame before, whatever the observer did in between.
        let his_marker = marker_of(&moved, Who::Bob);
        assert!(
            moved.iter().any(|(_, r, at)| {
                (*r - (Who::Bob.marker_radius() + CENTRED_RING_GAP)).abs() < 1e-6
                    && (*at - his_marker).length() < 1e-6
            }),
            "and the ring that says he is the one being followed is drawn on him: {moved:?}"
        );

        // Switched off again, he goes back to crossing the canvas.
        canvas.centred_on = None;
        let (parked, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (released, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (marker_of(&released, Who::Bob) - marker_of(&parked, Who::Bob)).length() > 5.0,
            "with the toggle off he is on the move again"
        );
        assert!(
            (hole_of(&released) - hole_of(&parked)).length() < 1e-3,
            "and the hole is back to standing still"
        );
    }

    /// The pointer events one step of a drag is made of: a move, optionally with the button going
    /// down or coming up.
    fn pointer(pos: Pos2, pressed: Option<bool>) -> Vec<egui::Event> {
        let mut events = vec![egui::Event::PointerMoved(pos)];
        if let Some(pressed) = pressed {
            events.push(egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: Default::default(),
            });
        }
        events
    }

    #[test]
    fn test_either_marker_can_be_dragged_anywhere_in_the_plane() {
        // The drag lives here now, and this is why: the equatorial view draws the plane the two
        // observers actually stand in, so a drag on it sets both coordinates an equatorial observer
        // has. On the (t, r) diagram, where this used to be, the whole plane is squeezed onto one
        // axis and a drag could only ever slide somebody along the radius - the pair could never be
        // put on opposite sides of the hole, which is the arrangement that decides how long light
        // takes to cross between them and how much of the other's frozen light a crosser meets.
        //
        // Bob is dragged to the point diametrically opposite Alice. Both are then at the same
        // drawn radius, so the embedding turns both by the same atan2(a, r) and the azimuths differ
        // by exactly half a turn.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice =
            Some(Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, params));
        let mode_before = bob.as_ref().expect("Bob is in this run").mode;

        // Frame 1: nothing but the painting, to find out where the markers are.
        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&painted);
        let her = marker_of(&painted, Who::Alice);
        let his = marker_of(&painted, Who::Bob);
        assert!((his - centre).length() > 50.0, "his marker is well clear of the hole");

        // The antipode of Alice's marker, which is where Bob is going. egui calls the drag
        // started on the frame that crosses its own threshold, and the grab offset is taken from
        // the pointer *there*, so the marker moves by what the pointer has moved since that frame:
        // to land him on the target, the pointer finishes that much past it.
        let target = centre - (her - centre);
        let nudge = his + Vec2::new(12.0, 0.0);
        let finish = target + (nudge - his);
        for (pos, pressed) in [
            (his, Some(true)),      // press on him
            (nudge, None),          // past egui's drag threshold: the pick happens here
            (finish, None),         // the move itself
            (finish, Some(false)),  // and the release
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }

        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = marker_of(&after, Who::Bob);
        let al = alice.as_ref().expect("Alice is in this run");
        let b = bob.as_ref().expect("Bob is in this run");
        let turn = std::f64::consts::TAU;
        let apart = (b.phi - al.phi).rem_euclid(turn);
        println!(
            "Bob dragged to Alice's antipode: r = {:.4} against her {:.4}, and their azimuths are \
             {:.4} rad apart",
            b.r,
            al.r,
            apart.min(turn - apart)
        );
        assert!((landed - target).length() < 2.0, "the marker went where the pointer did: {landed:?}");
        assert!((b.r - al.r).abs() < 0.05, "at the same radius she is at: {} vs {}", b.r, al.r);
        assert!(
            (apart.min(turn - apart) - std::f64::consts::PI).abs() < 0.02,
            "half a turn apart: {apart} rad"
        );
        assert_eq!(b.mode, mode_before, "and dropped back onto the Motion he was on");
        assert!(canvas.dragging.is_none(), "with the canvas no longer holding him");
        // Alice, who was not touched, has not moved at all.
        assert!((al.r - 4.5).abs() < 1e-12 && (al.phi - 0.25).abs() < 1e-12, "Alice stayed put");
    }

    #[test]
    fn test_a_drag_keeps_the_grab_offset_and_stops_at_the_ring() {
        // Two things the drag has to get right about the pointer. It is picked up wherever it is
        // pressed - press a marker off centre and it must not jump under the cursor - and it cannot
        // be dropped onto the ring: the disc rho < a in the middle of the picture is not a region
        // of the equatorial plane at all, and the ring itself is the curvature singularity, so the
        // radius floors at `RING_DROP_FLOOR` rather than going to nothing.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));

        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&painted);
        let his = marker_of(&painted, Who::Bob);
        // Pressed a little off centre, which is where the grab offset earns its place.
        // The travel is measured from the frame egui calls the drag started on, since that is
        // the pointer the grab offset is taken against.
        let press = his + Vec2::new(5.0, 3.0);
        let nudge = press + Vec2::new(12.0, 0.0);
        let travel = Vec2::new(-60.0, 40.0);
        for (pos, pressed) in [
            (press, Some(true)),
            (nudge, None),
            (nudge + travel, None),
            (nudge + travel, Some(false)),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = marker_of(&after, Who::Bob);
        println!(
            "pressed {:.0} px off his centre and moved {:?}: the marker moved to {landed:?}, \
             wanted {:?}",
            (press - his).length(),
            travel,
            his + travel
        );
        assert!(
            (landed - (his + travel)).length() < 2.0,
            "the marker moves by what the pointer moved, not to where it is: {landed:?}"
        );

        // And now into the middle of the hole, which is not a place.
        let nudge = landed + Vec2::new(12.0, 0.0);
        let into_the_hole = centre + (nudge - landed);
        for (pos, pressed) in [
            (landed, Some(true)),
            (nudge, None),
            (into_the_hole, None),
            (into_the_hole, Some(false)),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let r = bob.as_ref().expect("Bob is in this run").r;
        println!("dragged onto the middle of the ring: r = {r} against a floor of {RING_DROP_FLOOR}");
        assert!((r - RING_DROP_FLOOR).abs() < 1e-9, "he stops at the ring floor: {r}");
    }

    #[test]
    fn test_centring_lets_go_of_an_observer_while_they_are_being_dragged() {
        // Keep Centered pans the view to hold somebody in the middle, which is exactly the wrong
        // thing to do to the observer under the pointer: the pan would cancel every pixel of the
        // drag, the marker would sit pinned to the centre, and the drag would look like it was
        // doing nothing while in fact moving them. So a drag of the centred observer suspends the
        // centring, and it resumes on release with the view re-centring on wherever they were put.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));
        canvas.centred_on = Some(Who::Bob);

        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let at = marker_of(&painted, Who::Bob);
        let travel = Vec2::new(-70.0, 35.0);
        for (pos, pressed) in [
            (at, Some(true)),
            (at + Vec2::new(12.0, 0.0), None),
            (at + travel, None),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let (held, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let moved = marker_of(&held, Who::Bob);
        println!("centred on Bob and dragged {travel:?}: his marker went from {at:?} to {moved:?}");
        assert!(
            (moved - at).length() > 20.0,
            "the drag moves him across the canvas rather than being cancelled by the pan: {moved:?}"
        );

        // Released, the hold comes back and puts him in the middle again.
        spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(at + travel, Some(false)));
        let (released, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let recentred = marker_of(&released, Who::Bob);
        assert_eq!(canvas.centred_on, Some(Who::Bob), "the request was never cancelled");
        assert!(
            (recentred - hole_of(&painted)).length() < 2.0 || (recentred - at).length() < 2.0,
            "and he is back in the middle of the view: {recentred:?}"
        );
    }

    #[test]
    fn test_going_to_an_observer_centres_them_once_and_lets_them_move_again() {
        // The menu's two kinds of item, told apart. Goto is a pan: it puts somebody in the middle
        // of the canvas at the moment it is asked, and from then on they are free to fall out of
        // it again, which is what makes it the right answer to "where did Bob get to" and the
        // wrong one to "follow Bob". Keep Centered is the standing request, and the test above is
        // about that one.
        let metric = KerrSchild::new(1.0, 0.65);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice =
            Some(Observer::new_with_phi(&metric, "Alice", 0.0, 9.0, 0.0, 2.2, params));
        let mut bob =
            Some(Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, params));
        // Nothing is panned and nothing is being tracked yet, so the hole is in the middle of the
        // canvas and that is the point everything below is measured against.
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&before);
        let off_centre = (marker_of(&before, Who::Bob) - centre).length();
        canvas.look_at(
            &metric,
            &bob,
            &alice,
            ReferenceFrame::DistantObserver,
            bob.as_ref().expect("Bob is in this run").cartesian_position(&metric),
        );
        let (gone_to, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = (marker_of(&gone_to, Who::Bob) - centre).length();
        println!(
            "Bob started {off_centre:.0} px off centre and Goto put him {landed:.3} px from it"
        );
        assert!(off_centre > 20.0, "he is not already in the middle: {off_centre} px");
        assert!(landed < 1e-3, "and Goto puts him there exactly: {landed} px");

        // Then he falls, and nothing holds him: that is the difference from Keep Centered.
        let b = bob.as_mut().expect("Bob is in this run");
        for _ in 0..10 {
            b.step(&metric, b.t + 0.1, 0.1);
        }
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let drifted = (marker_of(&after, Who::Bob) - centre).length();
        assert!(drifted > 5.0, "a Goto does not follow him: {drifted} px");

        // And the hole is a target like any other: it sits at the origin of the embedding.
        canvas.look_at(&metric, &bob, &alice, ReferenceFrame::DistantObserver, (0.0, 0.0));
        let (at_hole, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (hole_of(&at_hole) - centre).length() < 1e-3,
            "Goto Black Hole centres the hole: {:?}",
            hole_of(&at_hole)
        );
    }

    #[test]
    fn test_the_front_ramp_starts_red_and_ends_violet() {
        // The three statements the wavefront colouring makes to the eye. A front is born at gain 1
        // and must come out at the red stop exactly, because every ray of a fresh pulse is at that
        // gain and the whole point is that the loop is one colour. The frozen family runs to a gain
        // of 1e5 within a run, and that end of the ramp must be the violet stop rather than
        // saturating early or wrapping. And below 1 - a ray can lose frequency between two
        // raindrops, one climbing outward away from the congruence's fall - it must leave the
        // spectrum rather than run further along it.
        //
        // What is *not* asserted here any more is that a loss looks darker. It cannot: every stop
        // of this ramp is struck at one lightness, so that the colour carries the shift and the eye
        // has no brightness to misread. `theme::tests` measures that; this measures the landmarks.
        let red = Theme::front_colour(1.0, 255);
        assert_eq!(
            (red.r(), red.g(), red.b()),
            (
                Theme::FRONT_RED_RGB[0],
                Theme::FRONT_RED_RGB[1],
                Theme::FRONT_RED_RGB[2]
            ),
            "a front at gain 1 must be exactly the red stop"
        );
        let violet = Theme::front_colour(1e5, 255);
        assert_eq!(
            (violet.r(), violet.g(), violet.b()),
            (
                Theme::FRONT_VIOLET_RGB[0],
                Theme::FRONT_VIOLET_RGB[1],
                Theme::FRONT_VIOLET_RGB[2]
            ),
            "a gain of 1e5 must be exactly the violet stop"
        );
        // And past the top of the ramp it stays there rather than running off it: the brightest
        // rays of a real front reach 1e6 within thirty M.
        assert_eq!(Theme::front_colour(1e9, 255), violet, "the ramp is clamped at the top");

        let grey = Theme::front_colour(0.1, 255);
        assert_eq!(
            (grey.r(), grey.g(), grey.b()),
            (
                Theme::FRONT_GREY_RGB[0],
                Theme::FRONT_GREY_RGB[1],
                Theme::FRONT_GREY_RGB[2]
            ),
            "a tenfold loss must be exactly the grey stop"
        );
        println!(
            "the front ramp: gain 1 -> {:?}, 10 -> {:?}, 30 -> {:?}, 1e3 -> {:?}, 1e5 -> {:?}, \
             and 0.1 -> {:?}",
            (red.r(), red.g(), red.b()),
            {
                let c = Theme::front_colour(10.0, 255);
                (c.r(), c.g(), c.b())
            },
            {
                let c = Theme::front_colour(10.0_f64.powf(1.5), 255);
                (c.r(), c.g(), c.b())
            },
            {
                let c = Theme::front_colour(1e3, 255);
                (c.r(), c.g(), c.b())
            },
            (violet.r(), violet.g(), violet.b()),
            (grey.r(), grey.g(), grey.b()),
        );
    }

    /// Count what `draw_signal_field` puts into a painter for one field: the polylines (segments
    /// of front) and the filled circles (calculated points, frozen beads and emission dots).
    fn count_front_shapes(
        metric: &KerrSchild,
        field: &SignalField,
        style: FrontStyle,
    ) -> (usize, usize) {
        fn tally(shape: &egui::Shape, lines: &mut usize, circles: &mut usize) {
            match shape {
                egui::Shape::Path(_) | egui::Shape::LineSegment { .. } => *lines += 1,
                egui::Shape::Circle(_) => *circles += 1,
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        tally(shape, lines, circles);
                    }
                }
                _ => {}
            }
        }
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            let to_screen =
                |(x, y): (f64, f64)| Pos2::new(200.0 + 60.0 * x as f32, 200.0 - 60.0 * y as f32);
            draw_signal_field(&painter, metric, field, Theme::ALICE_COLOR, 1.0, style, &to_screen);
        });
        let (mut lines, mut circles) = (0, 0);
        for clipped in output.shapes.iter() {
            tally(&clipped.shape, &mut lines, &mut circles);
        }
        output.drop_without_applying_deltas();
        (lines, circles)
    }

    #[test]
    fn test_with_the_arcs_turned_off_a_front_is_its_calculated_points_and_nothing_between_them() {
        // The checkbox, at the one place it acts: `draw_signal_field` adds either one polyline per
        // live segment (arcs on) or one filled circle per live ray (arcs off), and never both. A
        // real transmission is drawn both ways into an egui painter and the shapes counted, so what
        // is measured is the drawing the user sees rather than a helper's return value. With the
        // arcs off nothing at all is drawn between neighbouring rays, however far apart they have
        // wound: that is the point of the setting, the raw integrated points with no interpolation
        // laid over them. The emission dot, one per pulse, is there either way.
        use crate::physics::observer::{Observer, WorldlineParams};
        use crate::physics::wavefront::SignalField;
        let metric = KerrSchild::new(1.0, 0.90);
        // An emitter let go inside r+ (1.436 at this spin), so the fronts it sends carry a frozen
        // family too and the beads are part of what is counted.
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 1.2, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let mut t = 0.0;
        for _ in 0..40 {
            field.emit_if_due(&metric, &alice);
            alice.step(&metric, t, 0.02);
            field.advance(&metric, 0.02);
            t += 0.02;
        }
        let live_pulses: Vec<_> =
            field.pulses.iter().filter(|p| p.rays.iter().any(|r| r.alive())).collect();
        let live: usize =
            live_pulses.iter().map(|p| p.rays.iter().filter(|r| r.alive()).count()).sum();
        assert!(live > 100, "the field has a front to draw: {live} live rays");

        // The winding cut is held off in both counts, so what is measured here is the one
        // checkbox this test is about and not the two of them together.
        let (lines_on, circles_on) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_off, circles_off) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: false, hide_wound: false });
        println!(
            "arcs on: {lines_on} polylines and {circles_on} circles; arcs off: {lines_off} \
             polylines and {circles_off} circles, over {live} live rays in {} pulses",
            live_pulses.len()
        );
        assert!(lines_on > 0, "with the arcs on the front is drawn as polylines");
        assert_eq!(lines_off, 0, "with the arcs off nothing is drawn between the rays");
        assert_eq!(
            circles_off,
            live + live_pulses.len(),
            "with the arcs off every live ray is one dot, plus one emission dot per pulse"
        );
        assert!(circles_off > circles_on, "and that is more dots than the frozen beads alone");
    }

    #[test]
    fn test_outside_r_plus_a_fresh_ring_is_drawn_at_one_weight() {
        // A ring let go at r = 4.5 has a prograde half with E - Omega_- L < 0 - the sign is fixed
        // at birth - but nothing about it has collapsed onto r-, so nothing about it needs the
        // heavy frozen pass, and drawing it heavily split every ring into a bright half and a
        // faint one. Every polyline of such a ring must come out at the same stroke width and the
        // same opacity, and no bead at all.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let emitter =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &emitter);
        field.advance(&metric, 0.5);
        let rays = &field.pulses[0].rays;
        assert!(rays.iter().all(|ray| ray.alive() && ray.r > metric.outer_horizon()));
        assert!(
            rays.iter().any(|ray| ray.frozen(&metric)),
            "the ring does carry rays of the frozen family, which is what the test is about"
        );

        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            let to_screen =
                |(x, y): (f64, f64)| Pos2::new(200.0 + 30.0 * x as f32, 200.0 - 30.0 * y as f32);
            draw_signal_field(
                &painter,
                &metric,
                &field,
                Theme::BOB_COLOR,
                Theme::SECONDARY_FRONT_WIDTH,
                FrontStyle { arcs: true, hide_wound: true },
                &to_screen,
            );
        });
        let mut strokes: Vec<(u32, u8)> = Vec::new();
        let mut circles = 0;
        fn walk(shape: &egui::Shape, strokes: &mut Vec<(u32, u8)>, circles: &mut usize) {
            match shape {
                egui::Shape::Path(path) => {
                    let alpha = match path.stroke.color {
                        egui::epaint::ColorMode::Solid(colour) => colour.a(),
                        egui::epaint::ColorMode::UV(_) => 0,
                    };
                    strokes.push((path.stroke.width.to_bits(), alpha));
                }
                egui::Shape::Circle(_) => *circles += 1,
                egui::Shape::Vec(inner) => inner.iter().for_each(|s| walk(s, strokes, circles)),
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut strokes, &mut circles);
        }
        output.drop_without_applying_deltas();
        assert_eq!(strokes.len(), rays.len(), "one polyline per segment of the ring");
        let first = strokes[0];
        assert!(
            strokes.iter().all(|s| *s == first),
            "every segment at one width and one opacity: {:?}",
            strokes.iter().collect::<std::collections::HashSet<_>>()
        );
        assert_eq!(first.1, Theme::SHIFT_ALPHA, "the ordinary opacity, not the frozen pass");
        assert_eq!(circles, 1, "the emission dot and no beads");
    }

    #[test]
    fn test_a_segment_is_drawn_in_bands_of_the_gain_along_it() {
        // A segment from a ray at gain 1 to a ray at gain 1e5: the whole ramp in one segment, which
        // is what the pair being torn apart carries - one ray settling onto r- and climbing like
        // exp(kappa_- t), its neighbour crossing at the gain it was born with. Painted in one
        // colour that segment said its deep-red end had gained a hundred thousandfold. Instead the
        // polyline is cut into bands of at most `FRONT_BAND_DECADES` of gain, each coloured at the
        // gain interpolated to its own midpoint in the same loop coordinate `segment_arc` walks the
        // position along, so the ramp runs *along* the segment from one ray to the other.
        let arc: Vec<Pos2> = (0..=100).map(|k| Pos2::new(k as f32, 0.0)).collect();
        let bands = banded_segment(arc.clone(), 1.0, 1e5);
        let n = bands.len();
        assert_eq!(n, 20, "five decades in quarter-decade bands: {n} bands");
        assert!(n <= MAX_FRONT_BANDS);

        // Every band's gain is log10 interpolated to the midpoint of its own s-range, and every
        // band covers exactly the pieces of the arc in that range.
        for (b, (points, gain)) in bands.iter().enumerate() {
            let (start, end) = (b * 100 / n, (b + 1) * 100 / n);
            let s_mid = 0.5 * ((start + end) as f64) / 100.0;
            let expect = 10.0_f64.powf(5.0 * s_mid);
            assert!((gain / expect - 1.0).abs() < 1e-12, "band {b}: {gain} against {expect}");
            assert_eq!(points.len(), end - start + 1, "band {b} covers its own pieces");
        }

        // The two ends of the segment come out at the two ends of the ramp, and each band's colour
        // is `Theme::front_colour` of the gain at that band's midpoint - nothing else.
        let first = Theme::front_colour(bands[0].1, Theme::SHIFT_ALPHA);
        let last = Theme::front_colour(bands[n - 1].1, Theme::SHIFT_ALPHA);
        assert_eq!(
            first,
            Theme::front_colour(10.0_f64.powf(5.0 * 0.025), Theme::SHIFT_ALPHA),
            "the first band is coloured at the gain of its own midpoint, s = 0.025"
        );
        assert!(
            bands[0].1 < 10.0_f64.powf(Theme::FRONT_LOG_ORANGE),
            "the first band is still on the deep-red leg of the ramp: gain {}",
            bands[0].1
        );
        assert!(first.r() > first.b(), "and reads red: {first:?}");
        assert!(
            bands[n - 1].1 > 10.0_f64.powf(Theme::FRONT_LOG_BLUE),
            "the last band is past the blue stop: gain {}",
            bands[n - 1].1
        );
        assert!(last.b() > last.r(), "and reads violet: {last:?}");

        // No gaps: consecutive bands share their boundary point, and the bands laid end to end are
        // the original arc in order with nothing left out and nothing left uncoloured.
        for pair in bands.windows(2) {
            assert_eq!(
                pair[0].0.last(),
                pair[1].0.first(),
                "consecutive bands must share their boundary point"
            );
        }
        let mut union: Vec<Pos2> = vec![bands[0].0[0]];
        for (points, _) in bands.iter() {
            union.extend_from_slice(&points[1..]);
        }
        assert_eq!(union, arc, "the bands are the arc, in order");
        println!(
            "a segment from gain 1 to gain 1e5 over 100 pieces is drawn as {n} bands, the first at \
             gain {:.4} ({:?}) and the last at {:.4e} ({:?})",
            bands[0].1,
            (first.r(), first.g(), first.b()),
            bands[n - 1].1,
            (last.r(), last.g(), last.b()),
        );

        // And the common case costs nothing: both ends at the same gain is one band over the whole
        // arc, drawn exactly as it was before there were bands at all.
        let flat = banded_segment(arc.clone(), 7.0, 7.0);
        assert_eq!(flat.len(), 1, "equal gains at the two ends is a single band");
        assert_eq!(flat[0].0, arc, "and it is the whole arc");
        assert!((flat[0].1 - 7.0).abs() < 1e-12, "at that gain: {}", flat[0].1);
    }

    #[test]
    fn test_a_segment_wound_past_a_full_turn_is_cut_and_its_two_ends_marked() {
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);

        // First, one pulse with the winding put in by hand, so that exactly one pair of the loop is
        // past a whole turn and every other pair is well inside one. A closed front cannot have a
        // single wound pair and nothing else - the azimuth differences around the loop sum to zero
        // - so the three turns of the one segment are paid back over the other eleven, a sixth of a
        // turn each, which is what a front in the deep interior looks like anyway.
        let emitter =
            Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.rays_per_pulse = 12;
        field.emit_if_due(&metric, &emitter);
        assert_eq!(field.pulses.len(), 1, "one pulse, let go at r = 4.5");
        let n = field.pulses[0].rays.len();
        assert_eq!(n, 12);
        assert!(field.pulses[0].rays.iter().all(|ray| ray.alive()), "a fresh pulse is all alive");
        // The wound pair is put between two rays of the crossing family, so that the two dots the
        // cut leaves are ordinary dots rather than beads the frozen pass would have drawn anyway.
        let frozen: Vec<bool> =
            field.pulses[0].rays.iter().map(|ray| ray.frozen(&metric)).collect();
        let i0 = (0..n - 1)
            .find(|&i| !frozen[i] && !frozen[i + 1])
            .expect("a pulse let go at r = 4.5 has neighbouring rays outside the frozen family");
        let tau = std::f64::consts::TAU;
        let mut phi = 0.0;
        for k in 0..n {
            field.pulses[0].rays[k].phi = phi;
            phi += if k == i0 { 3.0 * tau } else { -3.0 * tau / ((n - 1) as f64) };
        }
        let wound: Vec<usize> = (0..n)
            .filter(|&i| {
                let j = (i + 1) % n;
                (field.pulses[0].rays[j].phi - field.pulses[0].rays[i].phi).abs()
                    > MAX_RESOLVED_WINDING
            })
            .collect();
        assert_eq!(wound, vec![i0], "exactly one pair is past a full turn, by construction");

        let (lines_kept, circles_kept) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_cut, circles_cut) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: true });
        println!(
            "a hand-built pulse of {n} rays with one pair three turns apart: {lines_kept} \
             polylines and {circles_kept} circles drawn whole, {lines_cut} and {circles_cut} with \
             the wound segment cut"
        );
        assert_eq!(lines_kept, n, "every live segment of a fresh pulse is one band, so one polyline");
        assert_eq!(lines_cut, lines_kept - 1, "the wound segment, and only it, is not drawn");
        assert_eq!(
            circles_cut,
            circles_kept + 2,
            "and its two rays are marked as dots, so the cut reads as a gap with ends"
        );

        // Now the same cut on a front nobody built by hand. A whole light cone is let go at r = 4.5
        // at a = 0.90 and integrated for 40 M. What winds a pair there is the pair straddling a
        // photon-orbit critical angle: the ray inside it hangs on the orbit and then settles onto
        // r-, where it co-rotates at Omega_- = 0.9 per M for ever, while its neighbour outside it
        // is long gone outward, so the raw difference between them grows without bound. Those are
        // exactly the segments the interpolation cannot speak for, and they are the ones dropped.
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &emitter);
        let dt = 0.02;
        for _ in 0..2000 {
            field.advance(&metric, dt);
        }
        let pulse = &field.pulses[0];
        let n = pulse.rays.len();

        // The gains the drawing colours by, computed here exactly as `draw_signal_field` computes
        // them, so that the bands counted are the bands drawn.
        let raindrop = GeodesicState::new_infall(&metric, 0.0, 12.0, 1.0, 0.0);
        let u_emit = {
            let (ut, ur, up) = raindrop.derivatives(&metric, pulse.emitted_r);
            [ut, ur, up]
        };
        let gain_of = |i: usize| {
            let ray = &pulse.rays[i];
            let (ut, ur, up) = raindrop.derivatives(&metric, ray.r);
            ray.gain_between(&metric, pulse.emitted_r, &u_emit, &[ut, ur, up])
        };
        let to_screen =
            |(x, y): (f64, f64)| Pos2::new(200.0 + 60.0 * x as f32, 200.0 - 60.0 * y as f32);

        let (mut wound, mut bands_lost, mut largest, mut live_pairs) = (0usize, 0usize, 0.0f64, 0);
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            live_pairs += 1;
            let d_phi = pulse.rays[j].phi - pulse.rays[i].phi;
            if d_phi.abs() <= MAX_RESOLVED_WINDING {
                continue;
            }
            wound += 1;
            largest = largest.max(d_phi.abs());
            let arc = segment_arc(
                &metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                &to_screen,
            );
            // Each band of a drawn segment is one polyline, so this is what the cut removes.
            bands_lost += banded_segment(arc, gain_of(i), gain_of(j)).len();
        }
        assert!(
            wound > 0,
            "within 40 M of a pulse let go at r = 4.5 at a = 0.90, some live pair must have wound \
             past a full turn; none of the {live_pairs} live pairs did"
        );

        let (lines_kept, _) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_cut, _) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: true });
        println!(
            "after 40 M a pulse of {n} rays let go at r = 4.5 has {live_pairs} live neighbouring \
             pairs, {wound} of them past a full turn (the largest {:.2} turns); the drawing goes \
             from {lines_kept} polylines to {lines_cut}, the {bands_lost} bands those segments \
             were drawn in",
            largest / tau
        );
        assert_eq!(
            lines_kept - lines_cut,
            bands_lost,
            "the cut must remove exactly the wound segments and nothing else"
        );
    }

    #[test]
    fn test_a_segment_between_two_frozen_rays_is_drawn_along_r_minus() {
        // Two rays of the same front frozen on the Cauchy horizon, most of a radian apart in
        // azimuth: the front between them lies on r-, and that is what has to be drawn. The chord
        // between their two screen positions does not - it cuts the chord of the circle, which at
        // this separation is well inside r- and, deeper in, inside the ring itself.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let to_screen = |(x, y): (f64, f64)| Pos2::new(x as f32, y as f32);
        let circle = (rm * rm + metric.a * metric.a).sqrt();

        let arc = segment_arc(&metric, (rm, 0.3), (rm, 1.5), &to_screen);
        // 1.2 radians in pieces of at most `MAX_ARC_STEP`, so 24 or 25 of them.
        assert!(arc.len() >= 25 && arc.len() <= 26, "{} points", arc.len());
        for pair in arc.windows(2) {
            let step = (pair[1] - pair[0]).length() as f64;
            assert!(step <= MAX_ARC_STEP * circle * 1.01, "a piece spans {step}");
        }
        let worst = arc
            .iter()
            .map(|p| (embedded_radius(*p) - circle).abs())
            .fold(0.0f64, f64::max);

        // The chord for comparison: its midpoint is the sagitta of the arc inside the circle.
        let chord_mid = Pos2::new(
            0.5 * (arc[0].x + arc[arc.len() - 1].x),
            0.5 * (arc[0].y + arc[arc.len() - 1].y),
        );
        let sagitta = circle - embedded_radius(chord_mid);
        println!(
            "an arc of {} pieces over 1.2 rad of r- stays within {worst:.3e} M of the r- circle \
             (rho = {circle:.4}); the chord it replaces dips {sagitta:.4} M inside it",
            arc.len() - 1
        );
        // 1e-6 rather than round-off: the drawn points are f32 screen coordinates.
        assert!(worst < 1e-6, "the drawn arc must lie on the r- circle: {worst}");
        assert!(sagitta > 0.15, "and the chord it replaces must not: {sagitta}");

        // A segment whose ends are close in azimuth is one straight piece, as it always was: the
        // subdivision costs nothing where it buys nothing.
        let short = segment_arc(&metric, (rm, 0.0), (rm, 0.04), &to_screen);
        assert_eq!(short.len(), 2, "a short segment is still a single line");

        // And a segment deep inside r-, where the two rays are far apart in azimuth and a chord
        // would cut through the disk inside the ring: every drawn point stays outside the ring.
        let ring = metric.a;
        let deep = segment_arc(&metric, (0.05, 0.0), (0.5, 2.5), &to_screen);
        let closest = deep
            .iter()
            .map(|p| embedded_radius(*p))
            .fold(f64::INFINITY, f64::min);
        let chord_closest = {
            let mid = Pos2::new(
                0.5 * (deep[0].x + deep[deep.len() - 1].x),
                0.5 * (deep[0].y + deep[deep.len() - 1].y),
            );
            embedded_radius(mid)
        };
        println!(
            "a segment from (r = 0.05, phi = 0) to (r = 0.5, phi = 2.5) is drawn in {} pieces and \
             never comes closer to the centre than rho = {closest:.4}, against the ring at \
             rho = {ring:.4}; the midpoint of the chord it replaces sits at rho = \
             {chord_closest:.4}, inside the ring",
            deep.len() - 1
        );
        assert!(closest > ring, "the drawn front must stay outside the ring: {closest}");
        assert!(chord_closest < ring, "whereas the chord does not: {chord_closest}");
    }

    /// The azimuth of a chart point: x + i y = (r + i a) e^{i phi}, so the argument is phi plus
    /// atan(a/r), and at fixed r a difference of arguments is a difference of phi.
    fn chart_azimuth((x, y): (f64, f64)) -> f64 {
        y.atan2(x)
    }

    /// The total azimuth a polyline of chart points sweeps, signed, counting the turns rather than
    /// folding them away: each piece is at most `MAX_ARC_STEP`, so the piece's own difference is
    /// unambiguous, and the sum of the pieces telescopes to the whole swept angle.
    fn swept_azimuth(points: &[(f64, f64)]) -> f64 {
        let two_pi = 2.0 * std::f64::consts::PI;
        points
            .windows(2)
            .map(|pair| {
                let d = chart_azimuth(pair[1]) - chart_azimuth(pair[0]);
                d - two_pi * (d / two_pi).round()
            })
            .sum()
    }

    #[test]
    fn test_a_segment_that_has_wound_is_drawn_the_long_way_round() {
        // Two neighbouring rays of one front, at the same radius, whose integrated azimuths differ
        // by 2 pi + 0.3: one of them has lapped the other round the hole. Both left the emission
        // event at the emitter's azimuth and both carry phi as a continuously integrated
        // coordinate, so that difference is not a representative of an angle - it is the winding
        // between the pair, and the piece of front between them is one whole turn plus 0.3 rad.
        // All of it has to be drawn.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let two_pi = 2.0 * std::f64::consts::PI;
        let span = two_pi + 0.3;

        // The chart points `segment_arc` embeds, in full f64, recorded on the way through the
        // projection: what is measured here is the curve the function generates, not the f32
        // screen coordinates it is finally rounded to.
        let chart = std::cell::RefCell::new(Vec::new());
        let to_screen = |(x, y): (f64, f64)| {
            chart.borrow_mut().push((x, y));
            Pos2::new(x as f32, y as f32)
        };

        let arc = segment_arc(&metric, (rm, 0.3), (rm, 0.3 + span), &to_screen);
        let points = chart.into_inner();
        assert_eq!(points.len(), arc.len(), "one chart point embedded per drawn point");

        // 2 pi + 0.3 = 6.583 rad in pieces of at most `MAX_ARC_STEP` = 0.05, so 132 of them.
        let pieces = (span / MAX_ARC_STEP).ceil() as usize;
        assert_eq!(pieces, 132);
        assert_eq!(
            arc.len(),
            pieces + 1,
            "a segment of {span:.4} rad is {pieces} pieces of at most {MAX_ARC_STEP} rad"
        );

        let swept = swept_azimuth(&points);
        println!(
            "a segment whose two rays differ by 2 pi + 0.3 = {span:.6} rad is drawn as {} pieces \
             sweeping {swept:.15} rad: one whole turn of the hole plus 0.3",
            arc.len() - 1
        );
        assert!(
            (swept - span).abs() < 1e-9,
            "the drawn arc must sweep the raw difference {span}, not {swept}. Folding it into \
             [-pi, pi] gives 2 pi + 0.3 - 2 pi = 0.3 - the same azimuth reached the other way \
             round the hole, which is -(2 pi - 0.3) of travel - and would have drawn 7 pieces of \
             a short arc across the near side of the picture instead of the {} pieces of the turn \
             the front actually made",
            arc.len() - 1
        );

        // Every piece is still on the r- circle and still no wider than `MAX_ARC_STEP`: winding
        // changes how far round the arc goes, not what it is drawn along.
        let circle = (rm * rm + metric.a * metric.a).sqrt();
        for pair in points.windows(2) {
            let d = chart_azimuth(pair[1]) - chart_azimuth(pair[0]);
            let d = d - two_pi * (d / two_pi).round();
            assert!(d.abs() <= MAX_ARC_STEP * (1.0 + 1e-12), "a piece spans {d} rad");
        }
        for point in points.iter() {
            let rho = (point.0 * point.0 + point.1 * point.1).sqrt();
            assert!((rho - circle).abs() < 1e-12, "the arc must stay on r-: {rho} vs {circle}");
        }
    }

    #[test]
    fn test_a_wound_front_of_a_real_pulse_is_drawn_over_its_raw_azimuth_difference() {
        // The same statement about a front nobody built by hand.
        //
        // A whole light cone is let go at r = 2.0 at a = 0.90 and integrated for 60 M of
        // coordinate time at the frame step. What winds a front there is the unstable circular
        // photon orbits outside r+ (at a = 0.90 the equatorial ones sit at r = 1.56 prograde and
        // r = 3.89 retrograde): a ray let go on very nearly the critical impact parameter hangs at
        // one of them for tens of M, going round and round, while the neighbour it was emitted
        // next to has long since escaped outward or spiralled in. Their azimuths are then several
        // whole turns apart, and the segment of front between them is that whole spiral. It is one
        // continuous piece of the null surface, however many times it goes round.
        //
        // (Emission from inside r+ does not do this: the rays that wind fastest there are the ones
        // being carried onto the ring, and they reach it and die within a couple of M. Over 20 M
        // of a pulse let go at r = 1.0, no pair of *live* neighbours ever gets past 1.1 rad.)
        let metric = KerrSchild::new(1.0, 0.90);
        let alice = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            2.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::default(),
        );
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses.len(), 1, "one pulse, let go at r = 2.0");
        let dt = 0.017;
        let steps = 3530; // 60.01 M of coordinate time
        for _ in 0..steps {
            field.advance(&metric, dt);
        }

        let pulse = &field.pulses[0];
        let n = pulse.rays.len();
        let pi = std::f64::consts::PI;
        let mut wound: Vec<(usize, f64)> = Vec::new();
        let mut live_pairs = 0;
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            live_pairs += 1;
            let raw = pulse.rays[j].phi - pulse.rays[i].phi;
            if raw.abs() > pi {
                wound.push((i, raw));
            }
        }
        let largest = wound.iter().map(|&(_, d)| d.abs()).fold(0.0f64, f64::max);
        println!(
            "after {:.2} M a pulse of {n} rays let go at r = 2.0 at a = 0.90 has {live_pairs} live \
             neighbouring pairs, {} of them more than half a turn apart; the largest raw |d phi| \
             is {largest:.3} rad ({:.2} turns)",
            (steps as f64) * dt,
            wound.len(),
            largest / (2.0 * pi)
        );
        assert!(
            !wound.is_empty(),
            "a ray hung on a circular photon orbit must wind past half a turn away from its \
             neighbour within 60 M; none of the {live_pairs} live pairs did"
        );

        // Each of them is drawn over exactly that raw difference. The fold would have replaced it
        // with the same angle mod 2 pi, which is a short arc across the near side of the picture
        // and not the spiral the front is.
        for &(i, raw) in wound.iter() {
            let j = (i + 1) % n;
            let chart = std::cell::RefCell::new(Vec::new());
            let to_screen = |(x, y): (f64, f64)| {
                chart.borrow_mut().push((x, y));
                Pos2::new(x as f32, y as f32)
            };
            let arc = segment_arc(
                &metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                &to_screen,
            );
            let points = chart.into_inner();
            assert_eq!(points.len(), arc.len());
            // The two ends are at different radii, and arg = phi + atan(a/r), so the swept
            // argument is the swept phi plus the change in that offset across the segment.
            let offset = (metric.a / pulse.rays[j].r).atan() - (metric.a / pulse.rays[i].r).atan();
            let swept = swept_azimuth(&points) - offset;
            let folded = raw - 2.0 * pi * (raw / (2.0 * pi)).round();
            assert!(
                (swept - raw).abs() < 1e-9,
                "segment {i} must be drawn over its raw difference {raw}, not {swept}; the fold \
                 would have drawn {folded} instead"
            );
            assert!(
                (folded - raw).abs() > 1.0,
                "segment {i}: the raw {raw} and the folded {folded} must be the two different \
                 pictures this test is about"
            );
            // And the piece count is the span, not the fold: pieces of at most `MAX_ARC_STEP`.
            let pieces = (raw.abs() / MAX_ARC_STEP).ceil().min(MAX_ARC_PIECES as f64) as usize;
            assert_eq!(arc.len(), pieces + 1, "segment {i} of {raw} rad in {} pieces", arc.len() - 1);
        }
    }
}
