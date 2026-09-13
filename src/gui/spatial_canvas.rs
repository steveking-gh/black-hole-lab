use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::river::RiverField;
use crate::gui::spacetime_canvas::TelemetryBoxes;
use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::wavefront::SignalField;
use egui::{Color32, Pos2, Stroke, Vec2};

pub struct SpatialCanvas {
    pub zoom: f32,
    pub pan_offset: Vec2,
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
            telemetry: TelemetryBoxes::default(),
            river: RiverField::default(),
        }
    }
}

impl SpatialCanvas {
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        show_river: bool,
        show_streamlines: bool,
        signals: SignalViews<'_>,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
        draw_front_arcs: bool,
    ) {
        let desired_size = egui::Vec2::new(ui.available_width(), canvas_height);
        let (response, painter) = ui.allocate_painter(desired_size, egui::Sense::drag());
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

        // Pan with mouse drag
        if response.dragged() {
            self.pan_offset += response.drag_delta();
        }

        // Center of the canvas with frame of reference tracking
        // Every point of the equatorial plane is placed by the Kerr-Schild embedding
        // x + i y = (r + i a) e^{i phi}, never by (r cos psi, r sin psi).
        // Screen y grows downward; Cartesian y grows upward (matching the +Y tick labels), so flip it.
        let to_offset = |(x, y): (f64, f64)| Vec2::new(x as f32 * self.zoom, -(y as f32) * self.zoom);
        // Following an observer who is not in the simulation is following nobody, so the view
        // stays on the hole rather than jumping to a remembered position.
        let followed = match frame_of_ref {
            ReferenceFrame::Bob => bob.as_ref(),
            ReferenceFrame::Alice => alice.as_ref(),
            ReferenceFrame::DistantObserver => None,
        };
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

        let r_to_px = |r: f64| -> f32 { (r as f32) * self.zoom };
        let to_screen =
            |(x, y): (f64, f64)| center + Vec2::new(x as f32 * self.zoom, -(y as f32) * self.zoom);
        // Direction vectors (velocities, tangents) need the same y flip as positions.
        let to_screen_dir = |(vx, vy): (f64, f64)| Vec2::new(vx as f32, -(vy as f32));

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
            let px_per_km = self.zoom / (metric.r_grav_km() as f32);
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
                    painter.text(Pos2::new(px_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("+{}", km_str), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // -X tick
                let px_neg_x = center.x - offset_px;
                if px_neg_x >= rect.left() + 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_neg_x, center.y - 3.0), Pos2::new(px_neg_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_neg_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("-{}", km_str), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // +Y tick
                let py_pos = center.y - offset_px;
                if py_pos >= rect.top() + 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_pos), Pos2::new(center.x + 3.0, py_pos)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_pos), egui::Align2::LEFT_CENTER, format!("+{}", km_str), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // -Y tick
                let py_neg = center.y + offset_px;
                if py_neg <= rect.bottom() - 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_neg), Pos2::new(center.x + 3.0, py_neg)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_neg), egui::Align2::LEFT_CENTER, format!("-{}", km_str), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                km += km_step;
            }
        } else {
            let visible_m = (rect.width().max(rect.height()) as f64 / self.zoom as f64) * 0.7;
            let target_step = (visible_m / 8.0).max(1e-6);
            let power = 10.0_f64.powf(target_step.log10().floor());
            let mantissa = target_step / power;
            let r_step = if mantissa < 1.5 { 1.0 * power } else if mantissa < 3.5 { 2.0 * power } else if mantissa < 7.5 { 5.0 * power } else { 10.0 * power };

            let max_ticks = 15;
            let mut s = r_step;
            for _ in 0..max_ticks {
                let offset_px = (s as f32) * self.zoom;
                if offset_px > rect.width().max(rect.height()) * 0.8 {
                    break;
                }
                let label = metric.format_grid_m(s, r_step);

                // +X tick
                let px_x = center.x + offset_px;
                if px_x <= rect.right() - 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_x, center.y - 3.0), Pos2::new(px_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("+{}", label), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // -X tick
                let px_neg_x = center.x - offset_px;
                if px_neg_x >= rect.left() + 5.0 && center.y >= rect.top() && center.y <= rect.bottom() {
                    painter.line_segment([Pos2::new(px_neg_x, center.y - 3.0), Pos2::new(px_neg_x, center.y + 3.0)], axis_stroke);
                    painter.text(Pos2::new(px_neg_x, center.y + 5.0), egui::Align2::CENTER_TOP, format!("-{}", label), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // +Y tick
                let py_pos = center.y - offset_px;
                if py_pos >= rect.top() + 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_pos), Pos2::new(center.x + 3.0, py_pos)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_pos), egui::Align2::LEFT_CENTER, format!("+{}", label), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
                }
                // -Y tick
                let py_neg = center.y + offset_px;
                if py_neg <= rect.bottom() - 5.0 && center.x >= rect.left() && center.x <= rect.right() {
                    painter.line_segment([Pos2::new(center.x - 3.0, py_neg), Pos2::new(center.x + 3.0, py_neg)], axis_stroke);
                    painter.text(Pos2::new(center.x + 5.0, py_neg), egui::Align2::LEFT_CENTER, format!("-{}", label), egui::FontId::monospace(9.0 * font_scale), Theme::TEXT_MUTED);
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
            egui::FontId::proportional(10.0 * font_scale),
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
            egui::FontId::proportional(10.0 * font_scale),
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

        // Ring singularity r = 0: the circle of Cartesian radius exactly a.
        painter.circle_stroke(center, ring_px.max(2.0), Stroke::new(2.0, Theme::SINGULARITY_LINE));
        if ring_px >= 6.0 {
            painter.text(
                center + Vec2::new(0.0, ring_px + 4.0),
                egui::Align2::CENTER_TOP,
                "ring singularity r = 0 (ρ = a)",
                egui::FontId::monospace(9.0 * font_scale),
                Theme::SINGULARITY_LINE,
            );
        }

        // 3. River of Space: the E = 1, L = 0 raindrop congruence, drawn under the arrows, the
        // trails and the observer markers so it never competes with them for legibility.
        if show_river {
            self.river.draw(&painter, metric, &to_screen, self.zoom);
        }

        // 4. Frame Dragging Swirl Vector Field
        if show_streamlines && metric.a.abs() > 0.01 {
            let radii = [0.4 * rm, rm, 0.5 * (rm + rp), rp, 0.5 * (rp + re), re, 3.0 * metric.m, 4.5 * metric.m];
            for &r in &radii {
                if r <= 0.05 {
                    continue;
                }
                let omega = metric.frame_dragging_omega(r);
                let num_arrows = 8;
                for i in 0..num_arrows {
                    // Step around the *chart* angle phi; the embedding then places the arrow on the
                    // circle of Cartesian radius sqrt(r^2 + a^2) by itself.
                    let phi = (i as f64) * 2.0 * std::f64::consts::PI / (num_arrows as f64);
                    let p_start = to_screen(metric.cartesian_position(r, phi));

                    // Tangential arrow length scaled by omega. The direction is the Cartesian image
                    // of the frame-dragging coordinate velocity (dr/dt, dphi/dt) = (0, omega),
                    // which carries the right handedness for either sign of the spin.
                    let arrow_len = (omega.abs() * 40.0).clamp(3.0, 22.0) as f32;
                    let drag = metric.cartesian_velocity(r, phi, 0.0, omega);
                    let tangent = to_screen_dir(drag).normalized() * arrow_len;
                    let p_end = p_start + tangent;

                    let color = if r < rp {
                        Theme::HORIZON_CAUCHY
                    } else if r < re {
                        Theme::ERGOSPHERE_LINE
                    } else {
                        Color32::from_rgba_premultiplied(0, 180, 255, 120)
                    };

                    painter.line_segment([p_start, p_end], Stroke::new(1.2, color));

                    // Arrow tip
                    let tip_side1 = p_end - tangent * 0.25 + Vec2::new(tangent.y, -tangent.x) * 0.18;
                    let tip_side2 = p_end - tangent * 0.25 - Vec2::new(tangent.y, -tangent.x) * 0.18;
                    painter.line_segment([p_end, tip_side1], Stroke::new(1.0, color));
                    painter.line_segment([p_end, tip_side2], Stroke::new(1.0, color));
                }
            }
        }

        // 5. The two transmissions, drawn over the flow but under the worldlines and the markers,
        // so the fronts read as something moving through the field rather than as part of the
        // observers' own trajectories. Bob's goes down first and Alice's over it, so where the two
        // overlap it is the heavier, primary field that stays legible.
        draw_signal_field(
            &painter,
            metric,
            signals.bob,
            Theme::BOB_COLOR,
            Theme::SECONDARY_FRONT_WIDTH,
            draw_front_arcs,
            &to_screen,
        );
        draw_signal_field(
            &painter,
            metric,
            signals.alice,
            Theme::ALICE_COLOR,
            1.0,
            draw_front_arcs,
            &to_screen,
        );

        // 6. Both worldline trails, drawn together and before anything that sits on them: the
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

        // 7. The observers themselves. Alice's info box is registered at the end of the frame,
        // after every other interaction on this canvas, so a drag on it does not pan. Bob's local
        // null cone is not drawn as a fan of stubs any more: he broadcasts the same pulses Alice
        // does, and a whole light cone integrated as exact null geodesics says everything the
        // twenty-four stubs said and keeps saying it as the light travels.
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice
            && al.is_active
        {
            let al_pos = to_screen(al.cartesian_position(metric));
            painter.circle_filled(al_pos, 5.0, Theme::ALICE_COLOR);
            alice_box = Some(al_pos);
        }
        let bob_box = bob.as_ref().map(|b| {
            let bob_pos = to_screen(b.cartesian_position(metric));
            // Bob circle marker
            painter.circle_filled(bob_pos, 7.0, Theme::BOB_COLOR);
            painter.circle_stroke(bob_pos, 9.0, Stroke::new(1.5, Color32::WHITE));
            bob_pos
        });

        // 8. Title and Legend Overlay
        // Horizon angular velocity Ω_H = a / (2 M r₊) is a rate per unit coordinate time, so in
        // geometric units it is a number per M; only dividing by t_g = GM/c³ makes it rad/s.
        let omega_h = metric.a / (2.0 * metric.m * rp);
        let legend_text = if use_km {
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
                 measured as it left: deep red ×1 (every front is born red), yellow ×10,\n\
                 white ×30, blue ×1000, violet ×100000, maroon below ×1\n\
                 Beaded arcs on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour (amber = Alice → Bob, mint = Bob → Alice)\n\
                 🔍 Zoom: {:.0} px/M (Scroll to zoom, drag to pan)",
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
                 measured as it left: deep red ×1 (every front is born red), yellow ×10,\n\
                 white ×30, blue ×1000, violet ×100000, maroon below ×1\n\
                 Beaded arcs on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour (amber = Alice → Bob, mint = Bob → Alice)\n\
                 🔍 Zoom: {:.0} px/M (Scroll to zoom, drag to pan)",
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
                self.zoom,
            )
        };

        painter.text(
            rect.left_top() + Vec2::new(10.0, 10.0),
            egui::Align2::LEFT_TOP,
            legend_text,
            egui::FontId::monospace(11.0 * font_scale),
            Theme::TEXT_BRIGHT,
        );

        // 9. Draggable info boxes, registered last so they take the drag instead of the canvas.
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
/// computed twice: a new front comes out one uniform deep red and then earns its way up the ramp as
/// it falls, and what the colour then shows is what the light has gained on its way here. A segment
/// takes the mean of its two endpoints' gains, so the ramp is continuous along the front.
///
/// Segments with a dead endpoint are skipped: a ray that has reached the ring is gone, and the front
/// genuinely ends there rather than jumping across the gap.
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

/// What one segment of a front is drawn as, under the user's choice of `draw_arcs`: either the
/// curve of `segment_arc`, or the straight chord between the two rays' own screen positions.
///
/// The choice is a drawing choice and only a drawing choice. `Pulse::scan` interpolates linearly in
/// (r, phi) between the same two rays either way, so which of these two polylines is on screen
/// changes nothing about where a reception happens or what shift it is measured at; what it changes
/// is whether the drawn front is the same curve the reception test is testing. With arcs on it is,
/// and that is the default. With them off the drawing is faster and the front reads as the raw
/// polygon on its rays, which is worth being able to see - but in the deep interior it is wrong in
/// a way worth naming: inside r- neighbouring rays wind at wildly different rates and end up most
/// of a radian apart, and the chord between two such rays cuts straight across the annulus and
/// through the disk inside the ring, drawing spikes into the singularity that no ray ever took.
fn segment_polyline<F: Fn((f64, f64)) -> Pos2>(
    metric: &KerrSchild,
    from: (f64, f64),
    to: (f64, f64),
    ends: (Pos2, Pos2),
    draw_arcs: bool,
    to_screen: &F,
) -> Vec<Pos2> {
    if draw_arcs {
        segment_arc(metric, from, to, to_screen)
    } else {
        vec![ends.0, ends.1]
    }
}

fn draw_signal_field<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    signal: &SignalField,
    emission_colour: Color32,
    width_scale: f32,
    draw_front_arcs: bool,
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
        let frozen: Vec<bool> =
            pulse.rays.iter().map(|ray| ray.alive() && ray.frozen(metric)).collect();
        // The ray positions themselves, which the frozen dots sit on; the segments between them
        // are drawn as arcs in (r, phi) rather than as chords between these points.
        let points: Vec<Pos2> = pulse
            .rays
            .iter()
            .map(|ray| to_screen(metric.cartesian_position(ray.r, ray.phi)))
            .collect();

        // n segments rather than n - 1: the closing one runs from the last ray back to the first.
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            let arc = segment_polyline(
                metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                (points[i], points[j]),
                draw_front_arcs,
                to_screen,
            );
            let gain = 0.5 * (gains[i] + gains[j]);
            if frozen[i] && frozen[j] {
                frozen_segments.push((arc, Theme::front_colour(gain, Theme::FRONT_FROZEN_ALPHA)));
                continue;
            }
            painter.add(egui::Shape::line(
                arc,
                Stroke::new(1.2 * width_scale, Theme::front_colour(gain, Theme::SHIFT_ALPHA)),
            ));
        }
        for (i, point) in points.iter().enumerate() {
            if frozen[i] {
                frozen_dots
                    .push((*point, Theme::front_colour(gains[i], Theme::FRONT_FROZEN_ALPHA)));
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
        painter.circle_filled(point, 1.6 * width_scale, colour);
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

fn draw_reception_tick(painter: &egui::Painter, at: Pos2, colour: Color32) {
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
fn draw_spatial_trail<F: Fn((f64, f64)) -> Pos2>(
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

    #[test]
    fn test_the_front_ramp_starts_deep_red_and_ends_violet() {
        // The three statements the wavefront colouring makes to the eye. A front is born at gain 1
        // and must come out at the deep-red stop exactly, because every ray of a fresh pulse is at
        // that gain and the whole point is that the loop is one colour. The frozen family runs to
        // a gain of 1e5 within a run, and that end of the ramp must be the violet stop rather than
        // saturating early or wrapping. And in between, the direction of travel has to read as
        // "more blueshift" all the way up, which for this palette means the blue channel never
        // goes back down between the yellow of a tenfold gain and the violet at the top.
        let deep_red = Theme::front_colour(1.0, 255);
        assert_eq!(
            (deep_red.r(), deep_red.g(), deep_red.b()),
            (
                Theme::FRONT_RED_RGB[0],
                Theme::FRONT_RED_RGB[1],
                Theme::FRONT_RED_RGB[2]
            ),
            "a front at gain 1 must be exactly the deep-red stop"
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

        // Below 1 it darkens towards maroon rather than brightening: a ray can lose frequency
        // between two raindrops, and that must not look like a gain.
        let maroon = Theme::front_colour(0.1, 255);
        assert_eq!(
            (maroon.r(), maroon.g(), maroon.b()),
            (
                Theme::FRONT_MAROON_RGB[0],
                Theme::FRONT_MAROON_RGB[1],
                Theme::FRONT_MAROON_RGB[2]
            ),
            "a tenfold loss must be exactly the maroon stop"
        );
        let dimmer = Theme::front_colour(0.5, 255);
        assert!(
            (dimmer.r() as u32 + dimmer.g() as u32 + dimmer.b() as u32)
                < (deep_red.r() as u32 + deep_red.g() as u32 + deep_red.b() as u32),
            "losing frequency must darken the deep red, not brighten it: {dimmer:?}"
        );

        // Monotone in blue from a tenfold gain to the top of the ramp, sampled finely enough to
        // catch a dip inside any one leg of the ramp as well as at the joins.
        let mut previous = 0u8;
        let mut worst: Option<(f64, u8, u8)> = None;
        for step in 0..=400 {
            let log = Theme::FRONT_LOG_YELLOW
                + (Theme::FRONT_LOG_MAX - Theme::FRONT_LOG_YELLOW) * (step as f64) / 400.0;
            let blue = Theme::front_colour(10.0_f64.powf(log), 255).b();
            if blue < previous && worst.is_none() {
                worst = Some((log, previous, blue));
            }
            previous = blue;
        }
        assert!(
            worst.is_none(),
            "the blue channel must never fall between gains of 10 and 1e5: {worst:?}"
        );
        println!(
            "the front ramp: gain 1 -> {:?}, 10 -> {:?}, 30 -> {:?}, 1e3 -> {:?}, 1e5 -> {:?}, \
             and 0.1 -> {:?}",
            (deep_red.r(), deep_red.g(), deep_red.b()),
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
            (maroon.r(), maroon.g(), maroon.b()),
        );
    }

    #[test]
    fn test_with_the_arcs_turned_off_a_segment_is_the_chord_between_its_two_rays() {
        // The checkbox, at the one place it acts. With the arcs on, a segment between two rays a
        // radian apart on r- is the curve linear in (r, phi) between them, which is dozens of
        // points lying on the r- circle. With them off it is the two ray positions and nothing in
        // between - the straight chord, which for this pair dips well inside r-. Both are drawings
        // of the same segment of the same front; `Pulse::scan` interpolates in (r, phi) either
        // way, so nothing about a reception moves when the box is unticked.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let to_screen = |(x, y): (f64, f64)| Pos2::new(x as f32, y as f32);
        let (from, to) = ((rm, 0.3), (rm, 1.5));
        let ends = (
            to_screen(metric.cartesian_position(from.0, from.1)),
            to_screen(metric.cartesian_position(to.0, to.1)),
        );

        let chord = segment_polyline(&metric, from, to, ends, false, &to_screen);
        assert_eq!(chord, vec![ends.0, ends.1], "with the arcs off a segment is its two ends");

        let arc = segment_polyline(&metric, from, to, ends, true, &to_screen);
        assert_eq!(
            arc,
            segment_arc(&metric, from, to, &to_screen),
            "with the arcs on a segment is exactly what `segment_arc` draws"
        );
        assert!(arc.len() > 20, "and that is the subdivided curve: {} points", arc.len());
        // The two agree at the ends and nowhere else: same front, two drawings of it.
        assert_eq!((arc[0], arc[arc.len() - 1]), (chord[0], chord[1]));
        let circle = (rm * rm + metric.a * metric.a).sqrt();
        let chord_mid = Pos2::new(
            0.5 * (chord[0].x + chord[1].x),
            0.5 * (chord[0].y + chord[1].y),
        );
        println!(
            "over 1.2 rad of r- the arc is {} points on the rho = {circle:.4} circle; the chord is \
             2 points whose midpoint sits at rho = {:.4}, inside it",
            arc.len(),
            embedded_radius(chord_mid)
        );
        assert!(embedded_radius(chord_mid) < circle - 0.15);
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
