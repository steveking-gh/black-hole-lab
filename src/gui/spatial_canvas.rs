use crate::gui::controls::ReferenceFrame;
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
        bob: &Observer,
        alice: &Option<Observer>,
        show_river: bool,
        show_streamlines: bool,
        show_signal: bool,
        signal: &SignalField,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
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
        let frame_tracking_offset = match frame_of_ref {
            ReferenceFrame::Bob => to_offset(bob.cartesian_position(metric)),
            ReferenceFrame::Alice => alice
                .as_ref()
                .map_or(Vec2::ZERO, |al| to_offset(al.cartesian_position(metric))),
            ReferenceFrame::DistantObserver => Vec2::ZERO,
        };
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

        // 5. Alice's outward signal pulses, drawn over the flow but under the worldlines and the
        // markers, so the fronts read as something moving through the field rather than as part of
        // the observers' own trajectories.
        if show_signal {
            draw_signal_field(&painter, metric, signal, &to_screen);
        }

        // 6. Draw Alice's Spatial Position and Trail. Her info box is registered at the end of the
        // frame, after every other interaction on this canvas, so a drag on it does not pan.
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice {
            draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &to_screen);
            if al.is_active {
                let al_pos = to_screen(al.cartesian_position(metric));

                painter.circle_filled(al_pos, 5.0, Theme::ALICE_COLOR);
                alice_box = Some(al_pos);
            }
        }

        // 7. Draw Bob's Spatial Position & Local Null Fan
        draw_spatial_trail(&painter, metric, bob, Theme::BOB_COLOR, 1.5, &to_screen);
        let bob_pos = to_screen(bob.cartesian_position(metric));

        // Project Bob's null emission fan (only while outside singularity)
        if bob.r > 0.02 && bob.is_active {
            // Bob's local null cone: 24 rays emitted isotropically in his own orthonormal frame
            // (alpha = 0 outward, alpha = pi/2 along +phi), mapped to coordinate slopes.
            let tetrad = bob.tetrad(metric);
            let ray_len = 28.0;
            for i in 0..24 {
                let alpha = 2.0 * std::f64::consts::PI * (i as f64) / 24.0;
                let (dr_dt, dphi_dt) = tetrad.coordinate_velocity(&tetrad.null_direction(alpha));
                // The screen direction is the Jacobian of the embedding applied to the coordinate
                // velocity, so the ingoing ray (dr/dt = -1, dphi/dt = 0) comes out as the straight
                // line -e^{i phi} tangent to the ring, as it must in this chart.
                let (vx, vy) = metric.cartesian_velocity(bob.r, bob.phi, dr_dt, dphi_dt);
                let ray_vector = to_screen_dir((vx, vy)).normalized() * ray_len;
                let ray_end = bob_pos + ray_vector;

                let ray_color = if dr_dt < 0.0 {
                    Color32::from_rgba_premultiplied(255, 80, 100, 140) // Trapped inward ray
                } else {
                    Color32::from_rgba_premultiplied(100, 240, 255, 160) // Outward ray
                };

                painter.line_segment([bob_pos, ray_end], Stroke::new(1.0, ray_color));
            }
        }

        // Bob circle marker
        painter.circle_filled(bob_pos, 7.0, Theme::BOB_COLOR);
        painter.circle_stroke(bob_pos, 9.0, Stroke::new(1.5, Color32::WHITE));

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
        self.telemetry.show(
            ui, &painter, "spatial", rect, bob_pos, "Bob", Theme::BOB_COLOR, bob, metric, use_km, font_scale,
        );
    }
}

/// Draw every live wavefront of Alice's signal in the equatorial embedding.
///
/// Each pulse is a polyline through the Kerr-Schild positions of its surviving rays, ordered by
/// emission angle, and each segment is coloured by the frequency a *local raindrop* would measure on
/// it against Alice's emission. The raindrop is the reference because it is the one frame that
/// exists at every radius, inside both horizons included, so the colour means the same thing across
/// the whole picture: it is the shift a body falling freely from rest at infinity would see, not a
/// shift quoted against a frame that stops existing at r+. A segment takes the mean of its two
/// endpoints' ratios, so the ramp is continuous along the front.
///
/// Segments with a dead endpoint are skipped: a ray that has reached the ring is gone, and the front
/// genuinely ends there rather than jumping across the gap.
fn draw_signal_field<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    signal: &SignalField,
    to_screen: &F,
) {
    // `derivatives` reads only (E, L) off the state and takes the radius as an argument, so one
    // instance of the raindrop congruence serves every ray of every pulse, as it does in the river.
    let raindrop = GeodesicState::new_infall(metric, 0.0, 12.0, 1.0, 0.0);
    for pulse in signal.pulses.iter() {
        let ratios: Vec<f64> = pulse
            .rays
            .iter()
            .map(|ray| {
                if !ray.alive {
                    return 1.0;
                }
                let (ut, ur, up) = raindrop.derivatives(metric, ray.r);
                ray.frequency_ratio(metric, &[ut, ur, up])
            })
            .collect();
        for i in 0..pulse.rays.len().saturating_sub(1) {
            let (a, b) = (&pulse.rays[i], &pulse.rays[i + 1]);
            if !a.alive || !b.alive {
                continue;
            }
            let p0 = to_screen(metric.cartesian_position(a.r, a.phi));
            let p1 = to_screen(metric.cartesian_position(b.r, b.phi));
            let colour = Theme::shift_colour(0.5 * (ratios[i] + ratios[i + 1]), Theme::SHIFT_ALPHA);
            painter.line_segment([p0, p1], Stroke::new(1.2, colour));
        }
    }
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
        .map(|&[_t, r, phi]| to_screen(metric.cartesian_position(r, phi)))
        .collect();
    let faint = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 120);
    painter.add(egui::Shape::line(points, Stroke::new(width, faint)));
}
