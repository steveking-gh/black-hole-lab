use crate::gui::controls::ReferenceFrame;
use crate::gui::spacetime_canvas::draw_hovering_telemetry;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use egui::{Color32, Pos2, Stroke, Vec2};

pub struct SpatialCanvas {
    pub zoom: f32,
    pub pan_offset: Vec2,
}

impl Default for SpatialCanvas {
    fn default() -> Self {
        Self {
            zoom: 48.0, // pixels per M
            pan_offset: Vec2::ZERO,
        }
    }
}

impl SpatialCanvas {
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Observer,
        alice: &Option<Observer>,
        show_streamlines: bool,
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
        let frame_tracking_offset = match frame_of_ref {
            ReferenceFrame::Bob => {
                let psi = bob.azimuth(metric);
                Vec2::new((bob.r * psi.cos()) as f32 * self.zoom, (bob.r * psi.sin()) as f32 * self.zoom)
            }
            ReferenceFrame::Alice => {
                if let Some(al) = alice {
                    let psi = al.azimuth(metric);
                    Vec2::new((al.r * psi.cos()) as f32 * self.zoom, (al.r * psi.sin()) as f32 * self.zoom)
                } else {
                    Vec2::ZERO
                }
            }
            ReferenceFrame::DistantObserver => Vec2::ZERO,
        };
        let center = rect.center() + self.pan_offset - frame_tracking_offset;

        // Background
        painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();

        let r_to_px = |r: f64| -> f32 { (r as f32) * self.zoom };

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

        // 1. Concentric Zone Fills
        // Ergosphere disc
        painter.circle_filled(center, r_to_px(re), Theme::ERGOSPHERE_FILL);

        // Region II disc (between rm and rp)
        painter.circle_filled(center, r_to_px(rp), Theme::REGION_II_FILL);

        // Region III disc (inside rm)
        painter.circle_filled(center, r_to_px(rm), Theme::REGION_III_FILL);

        // Singularity core (r -> 0)
        let ring_px = (metric.a.abs() as f32) * self.zoom * 0.4;
        painter.circle_filled(center, ring_px.max(3.0), Theme::SINGULARITY_FILL);

        // 2. Concentric Boundary Rings
        // Ergosphere boundary
        painter.circle_stroke(center, r_to_px(re), Stroke::new(1.5, Theme::ERGOSPHERE_LINE));

        // Outer Horizon r+
        painter.circle_stroke(center, r_to_px(rp), Stroke::new(2.5, Theme::HORIZON_OUTER));

        // Inner Cauchy Horizon r-
        painter.circle_stroke(center, r_to_px(rm), Stroke::new(2.0, Theme::HORIZON_CAUCHY));

        // Ring Singularity
        painter.circle_stroke(center, ring_px.max(2.0), Stroke::new(2.0, Theme::SINGULARITY_LINE));

        // 3. Frame Dragging Swirl Vector Field
        if show_streamlines && metric.a.abs() > 0.01 {
            let radii = [0.4 * rm, rm, 0.5 * (rm + rp), rp, 0.5 * (rp + re), re, 3.0 * metric.m, 4.5 * metric.m];
            for &r in &radii {
                if r <= 0.05 {
                    continue;
                }
                let omega = metric.frame_dragging_omega(r);
                let px_radius = r_to_px(r);
                let num_arrows = 8;
                for i in 0..num_arrows {
                    let angle = (i as f32) * 2.0 * std::f32::consts::PI / (num_arrows as f32);
                    let p_start = center + Vec2::new(angle.cos() * px_radius, angle.sin() * px_radius);

                    // Tangential arrow length scaled by omega
                    let arrow_len = (omega * 40.0).clamp(3.0, 22.0) as f32;
                    let sign = if metric.a > 0.0 { 1.0 } else { -1.0 };
                    let tangent = Vec2::new(-angle.sin() * sign, angle.cos() * sign) * arrow_len;
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

        // 4. Draw Alice's Spatial Position and Trail
        if let Some(al) = alice {
            if al.is_active {
                let al_psi = al.azimuth(metric);
                let al_x = (al.r * al_psi.cos()) as f32 * self.zoom;
                let al_y = (al.r * al_psi.sin()) as f32 * self.zoom;
                let al_pos = center + Vec2::new(al_x, al_y);

                painter.circle_filled(al_pos, 5.0, Theme::ALICE_COLOR);
                draw_hovering_telemetry(&painter, rect, al_pos, "Alice", Theme::ALICE_COLOR, al, metric, use_km, font_scale);
            }
        }

        // 5. Draw Bob's Spatial Position & Local Null Fan
        let bob_psi = bob.azimuth(metric);
        let bob_x = (bob.r * bob_psi.cos()) as f32 * self.zoom;
        let bob_y = (bob.r * bob_psi.sin()) as f32 * self.zoom;
        let bob_pos = center + Vec2::new(bob_x, bob_y);

        // Project Bob's null emission fan (only while outside singularity)
        if bob.r > 0.02 && bob.is_active {
            let fan = metric.null_cone_fan(bob.r, 24);
            let ray_len = 28.0;
            for (dr_dt, dphi_dt) in fan {
                let radial_dir = Vec2::new(bob_psi.cos() as f32, bob_psi.sin() as f32);
                let azim_dir = Vec2::new(-bob_psi.sin() as f32, bob_psi.cos() as f32);

                let ray_vector = (radial_dir * (dr_dt as f32) + azim_dir * (dphi_dt as f32 * bob.r as f32)).normalized() * ray_len;
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
        draw_hovering_telemetry(&painter, rect, bob_pos, "Bob", Theme::BOB_COLOR, bob, metric, use_km, font_scale);

        // 6. Title and Legend Overlay
        let legend_text = if use_km {
            format!(
                "Equatorial View (θ = π/2, x = r cos ϕ, y = r sin ϕ)\n\
                 Units: Kilometers (km) & Seconds (s)\n\
                 Scale: 1M = {}\n\
                 Mass: {:.2e} M☉\n\
                 Outer Horizon r₊: {} ({:.2}M)\n\
                 Cauchy Horizon r₋: {} ({:.2}M)\n\
                 Spin a/M: {:.3}\n\
                 Drag: Ω_H = {:.3} rad/s\n\
                 🔍 Zoom: {:.0} px/M (Scroll to zoom, drag to pan)",
                metric.format_physical_distance(1.0),
                metric.m_solar,
                metric.format_km(metric.r_to_km(rp)),
                rp,
                metric.format_km(metric.r_to_km(rm)),
                rm,
                metric.a_star(),
                metric.a / (2.0 * metric.m * rp),
                self.zoom,
            )
        } else {
            format!(
                "Equatorial View (θ = π/2, x = r cos ϕ, y = r sin ϕ)\n\
                 Physical Scale: 1M = GM/c² = {}\n\
                 Time Scale:     1M/c = GM/c³ = {}\n\
                 Mass: {:.2e} M☉\n\
                 Outer Horizon r₊: {:.2}M ({})\n\
                 Cauchy Horizon r₋: {:.2}M ({})\n\
                 Spin a/M: {:.3}\n\
                 Drag: Ω_H = {:.3} rad/s\n\
                 🔍 Zoom: {:.0} px/M (Scroll to zoom, drag to pan)",
                metric.format_physical_distance(1.0),
                metric.format_physical_time(1.0),
                metric.m_solar,
                rp,
                metric.format_physical_distance(rp),
                rm,
                metric.format_physical_distance(rm),
                metric.a_star(),
                metric.a / (2.0 * metric.m * rp),
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
    }
}
