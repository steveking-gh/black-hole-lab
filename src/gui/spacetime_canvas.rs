use crate::gui::controls::ReferenceFrame;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use egui::{epaint::PathShape, Color32, Pos2, Rect, Stroke, Vec2};

pub fn draw_hovering_telemetry(
    painter: &egui::Painter,
    canvas_rect: Rect,
    pos: Pos2,
    name: &str,
    color: Color32,
    obs: &Observer,
    metric: &KerrSchild,
    use_km: bool,
    font_scale: f32,
) {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let v_c = obs.velocity_c(metric);
    let v_kms = obs.velocity_km_s(metric);
    let u_prop = obs.proper_velocity_c(metric);
    let a_prop = obs.proper_acceleration_g(metric);
    // Decide "free fall" from the geometric magnitude, not from a_prop: the g-conversion
    // multiplies by ~c^2/r_g (about 6e11 for a 10 M_sun hole), which would turn the numerical
    // noise floor of a genuine geodesic into a few spurious g.
    let is_geodesic = obs.is_free_falling(metric);
    let a_tidal_grad = obs.tidal_gradient_g_per_m(metric);
    let time_comp = obs.exterior_time_compression(metric);

    let v_str = if use_km {
        format!("dr/dt = {:+.0} km/s ({:+.2}c) | dr/dτ = {:+.2}c", v_kms, v_c, u_prop)
    } else {
        format!("dr/dt = {:+.2}c | dr/dτ = {:+.2}c", v_c, u_prop)
    };

    let a_str = if is_geodesic || a_prop < 0.05 {
        "a_prop = 0.00g (Free Fall)".to_string()
    } else {
        format!("a_thrust = {:.1}g", a_prop)
    };

    let tidal_str = if a_tidal_grad >= 1e6 {
        format!("Tidal = {:.2e} g/m", a_tidal_grad)
    } else if a_tidal_grad >= 100.0 {
        format!("Tidal = {:.0} g/m", a_tidal_grad)
    } else if a_tidal_grad >= 0.01 {
        format!("Tidal = {:.2} g/m", a_tidal_grad)
    } else {
        format!("Tidal = {:.2e} g/m", a_tidal_grad)
    };

    let comp_str = if time_comp >= 1e6 {
        format!("Ext Comp dt/dτ = {:.2e}x", time_comp)
    } else if time_comp >= 100.0 {
        format!("Ext Comp dt/dτ = {:.0}x", time_comp)
    } else if time_comp > 1.05 {
        format!("Ext Comp dt/dτ = {:.1}x", time_comp)
    } else {
        "Ext Comp dt/dτ = 1.0x (Normal)".to_string()
    };

    let rm = metric.inner_horizon();
    let rp = metric.outer_horizon();
    let re = metric.ergosphere_equatorial();

    let region_tag = if obs.r > re {
        "Reg I"
    } else if obs.r > rp {
        "Ergo"
    } else if obs.r > rm {
        "Reg II (Trapped)"
    } else {
        "Reg III (Core)"
    };

    let title_line = format!("{} [{}] • {}", name, region_tag, v_str);

    let font_title = egui::FontId::monospace(10.0 * font_scale);
    let font_body = egui::FontId::monospace(9.0 * font_scale);

    // Measure actual rendered text widths so box dynamically encloses all text
    let w_title = painter.layout_no_wrap(title_line.clone(), font_title.clone(), color).size().x;
    let w_a = painter.layout_no_wrap(a_str.clone(), font_body.clone(), color).size().x;
    let w_tidal = painter.layout_no_wrap(tidal_str.clone(), font_body.clone(), color).size().x;
    let w_comp = painter.layout_no_wrap(comp_str.clone(), font_body.clone(), color).size().x;

    let max_text_w = w_title.max(w_a).max(w_tidal).max(w_comp);
    let pad_x = 10.0 * font_scale;
    let pad_y = 6.0 * font_scale;
    let line_spacing = 13.0 * font_scale;

    let badge_w = (max_text_w + pad_x * 2.0).max(180.0 * font_scale);
    let badge_h = (pad_y * 2.0 + line_spacing * 3.8).max(56.0 * font_scale);

    let bx = (pos.x + 12.0).min(canvas_rect.right() - badge_w - 6.0).max(canvas_rect.left() + 6.0);
    let by = (pos.y - badge_h - 6.0).max(canvas_rect.top() + 6.0).min(canvas_rect.bottom() - badge_h - 6.0);

    let badge_rect = Rect::from_min_size(
        Pos2::new(bx, by),
        egui::Vec2::new(badge_w, badge_h),
    );

    painter.rect_filled(badge_rect, 4.0 * font_scale, Color32::from_black_alpha(230));
    painter.rect_stroke(badge_rect, 4.0 * font_scale, Stroke::new(1.2, color), egui::StrokeKind::Inside);

    painter.text(
        Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y),
        egui::Align2::LEFT_TOP,
        title_line,
        font_title,
        color,
    );
    painter.text(
        Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing),
        egui::Align2::LEFT_TOP,
        a_str,
        font_body.clone(),
        Color32::from_rgb(180, 240, 180),
    );
    painter.text(
        Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing * 2.0),
        egui::Align2::LEFT_TOP,
        tidal_str,
        font_body.clone(),
        Color32::from_rgb(255, 200, 100),
    );
    painter.text(
        Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing * 3.0),
        egui::Align2::LEFT_TOP,
        comp_str,
        font_body,
        if time_comp > 10.0 { Theme::HORIZON_CAUCHY } else { Color32::from_rgb(160, 210, 255) },
    );
}

pub struct SpacetimeCanvas {
    pub max_r: f64,
    pub r_offset: f64,
    pub time_window: f64,
    pub time_offset: f64,
    pub is_dragging_bob: bool,
}

impl Default for SpacetimeCanvas {
    fn default() -> Self {
        Self {
            max_r: 5.5,
            r_offset: 0.0,
            time_window: 14.0,
            time_offset: 0.0,
            is_dragging_bob: false,
        }
    }
}

impl SpacetimeCanvas {
    pub fn reset_zoom(&mut self) {
        self.max_r = 5.5;
        self.r_offset = 0.0;
        self.time_window = 14.0;
        self.time_offset = 0.0;
    }

    pub fn focus_horizon(&mut self, rm: f64) {
        self.max_r = 0.05;
        self.r_offset = (rm - 0.025).max(0.0);
    }

    pub fn focus_bob(&mut self, bob_r: f64) {
        self.max_r = 0.05;
        self.r_offset = (bob_r - 0.025).max(0.0);
    }

    /// Render the (t, r) spacetime foliation canvas with integrated, perfectly aligned 1D Cauchy Compression Track
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &mut Observer,
        alice: &Option<Observer>,
        current_time: f64,
        show_wavefronts: bool,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
    ) {
        let total_size = egui::Vec2::new(ui.available_width(), canvas_height);
        let track_height = (60.0 * font_scale.sqrt()).max(50.0);
        let main_canvas_height = (total_size.y - track_height - 6.0).max(150.0);

        // =========================================================================
        // 1. MAIN SPACETIME CANVAS (t, r) / OBSERVER REST FRAME
        // =========================================================================
        let (response, painter) = ui.allocate_painter(egui::Vec2::new(total_size.x, main_canvas_height), egui::Sense::drag());
        let rect = response.rect;

        if rect.width() < 30.0 || rect.height() < 30.0 {
            return;
        }

        // Mouse wheel zoom on canvas (smooth 1/8th step size)
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let step = 0.025;
                let factor = if scroll > 0.0 { 1.0 - step } else { 1.0 + step };
                let new_max_r = (self.max_r * factor).clamp(0.0001, 50.0);
                if let Some(mpos) = response.hover_pos() {
                    let mouse_frac = ((mpos.x - rect.left()) / rect.width().max(1.0)).clamp(0.0, 1.0) as f64;
                    let mouse_r = self.r_offset + mouse_frac * self.max_r;
                    self.r_offset = (mouse_r - mouse_frac * new_max_r).max(0.0);
                }
                self.max_r = new_max_r;
                self.time_window = (self.time_window * factor).clamp(0.005, 500.0);
            }
        }

        match frame_of_ref {
            ReferenceFrame::Bob => {
                painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
                self.render_observer_frame(&painter, rect, metric, bob, alice.as_ref(), use_km, font_scale);
            }
            ReferenceFrame::Alice => {
                painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
                if let Some(al) = alice {
                    self.render_observer_frame(&painter, rect, metric, al, Some(bob), use_km, font_scale);
                } else {
                    self.render_observer_frame(&painter, rect, metric, bob, None, use_km, font_scale);
                }
            }
            ReferenceFrame::DistantObserver => {
                self.render_distant_observer(
                    &painter,
                    &response,
                    rect,
                    metric,
                    bob,
                    alice,
                    current_time,
                    show_wavefronts,
                    use_km,
                    font_scale,
                );
            }
        }

        // =========================================================================
        // 2. INTEGRATED 1D CAUCHY COMPRESSION TRACK (PIXEL-PERFECT HORIZONTAL ALIGNMENT)
        // =========================================================================
        ui.add_space(3.0);
        let (t_resp, t_painter) = ui.allocate_painter(egui::Vec2::new(total_size.x, track_height), egui::Sense::hover());
        let t_rect = t_resp.rect;

        // Background
        t_painter.rect_filled(t_rect, 4.0, Theme::CANVAS_BG);

        // Re-use EXACT same to_screen_x formula:
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();

        let track_to_x = |r: f64| -> f32 {
            let frac = ((r - self.r_offset) / self.max_r) as f32;
            t_rect.left() + frac * t_rect.width()
        };

        let clamp_x = |x: f32| x.clamp(t_rect.left(), t_rect.right());
        let x0 = clamp_x(track_to_x(0.0));
        let xm = clamp_x(track_to_x(rm));
        let xp = clamp_x(track_to_x(rp));
        let xe = clamp_x(track_to_x(re));

        // Region fills matching top canvas
        if xm > x0 {
            t_painter.rect_filled(Rect::from_min_max(Pos2::new(x0, t_rect.top()), Pos2::new(xm, t_rect.bottom())), 0.0, Theme::REGION_III_FILL);
        }
        if xp > xm {
            t_painter.rect_filled(Rect::from_min_max(Pos2::new(xm, t_rect.top()), Pos2::new(xp, t_rect.bottom())), 0.0, Theme::REGION_II_FILL);
        }
        if xe > xp {
            t_painter.rect_filled(Rect::from_min_max(Pos2::new(xp, t_rect.top()), Pos2::new(xe, t_rect.bottom())), 0.0, Theme::ERGOSPHERE_FILL);
        }

        // Vertical lines dropping straight down from top canvas
        let line_x_sing = track_to_x(0.0);
        let line_x_rm = track_to_x(rm);
        let line_x_rp = track_to_x(rp);
        let line_x_re = track_to_x(re);

        if line_x_sing >= t_rect.left() && line_x_sing <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_sing + 1.0, t_rect.top()), Pos2::new(line_x_sing + 1.0, t_rect.bottom())], Stroke::new(2.5, Theme::SINGULARITY_LINE));
            if use_km {
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0 km", egui::FontId::monospace(9.0 * font_scale), Theme::SINGULARITY_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0", egui::FontId::monospace(9.0 * font_scale), Theme::SINGULARITY_LINE);
            }
        }
        if line_x_rm >= t_rect.left() && line_x_rm <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rm, t_rect.top()), Pos2::new(line_x_rm, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_CAUCHY));
            if use_km {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₋={}", metric.format_km(metric.r_to_km(rm))), egui::FontId::monospace(9.0 * font_scale), Theme::HORIZON_CAUCHY);
            } else {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Cauchy r₋", egui::FontId::monospace(10.0 * font_scale), Theme::HORIZON_CAUCHY);
            }
        }
        if line_x_rp >= t_rect.left() && line_x_rp <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rp, t_rect.top()), Pos2::new(line_x_rp, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_OUTER));
            if use_km {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₊={}", metric.format_km(metric.r_to_km(rp))), egui::FontId::monospace(9.0 * font_scale), Theme::HORIZON_OUTER);
            } else {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Outer r₊", egui::FontId::monospace(10.0 * font_scale), Theme::HORIZON_OUTER);
            }
        }
        if line_x_re >= t_rect.left() && line_x_re <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_re, t_rect.top()), Pos2::new(line_x_re, t_rect.bottom())], Stroke::new(1.5, Theme::ERGOSPHERE_LINE));
            if use_km {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r_E={}", metric.format_km(metric.r_to_km(re))), egui::FontId::monospace(9.0 * font_scale), Theme::ERGOSPHERE_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "r_E", egui::FontId::monospace(9.0 * font_scale), Theme::ERGOSPHERE_LINE);
            }
        }

        // Track header badge
        let track_badge = if use_km {
            "1D CAUCHY COMPRESSION TRACK  |  Radial Axis r [Kilometers (km)]".to_string()
        } else {
            format!(
                "1D CAUCHY COMPRESSION TRACK  |  Radial Axis r  [1M = GM/c² = {}]",
                metric.format_physical_distance(1.0)
            )
        };
        t_painter.text(
            Pos2::new(t_rect.left() + 6.0, t_rect.top() + 4.0),
            egui::Align2::LEFT_TOP,
            track_badge,
            egui::FontId::proportional(9.0 * font_scale),
            Theme::TEXT_MUTED,
        );

        let center_y = t_rect.center().y + 2.0;

        // Draw Alice on Track
        if let Some(al) = alice {
            if al.is_active {
                let al_x = track_to_x(al.r);
                t_painter.circle_filled(Pos2::new(al_x, center_y), 6.0, Theme::ALICE_COLOR);
                t_painter.text(Pos2::new(al_x, center_y - 10.0), egui::Align2::CENTER_BOTTOM, "Alice", egui::FontId::proportional(10.0 * font_scale), Theme::ALICE_COLOR);
            }
        }

        // Draw Bob on Track
        if bob.is_active {
            let bob_x = track_to_x(bob.r);
            t_painter.circle_filled(Pos2::new(bob_x, center_y), 6.5, Theme::BOB_COLOR);
            t_painter.circle_stroke(Pos2::new(bob_x, center_y), 8.5, Stroke::new(1.0, Color32::WHITE));
            t_painter.text(Pos2::new(bob_x, center_y + 9.0), egui::Align2::CENTER_TOP, "Bob", egui::FontId::proportional(10.0 * font_scale), Theme::BOB_COLOR);
        }

        // Draw Coalescence indicator if Alice and Bob are both present
        if let Some(al) = alice {
            if al.is_active && bob.is_active {
                let diff = (bob.r - al.r).abs();
                let al_x = track_to_x(al.r);
                let bob_x = track_to_x(bob.r);

                // Distance bracket / line
                t_painter.line_segment([Pos2::new(al_x, center_y), Pos2::new(bob_x, center_y)], Stroke::new(2.0, Color32::WHITE));

                if diff < 0.15 && al.r <= rm + 0.15 {
                    // Coalescence beacon
                    let mid_x = (al_x + bob_x) * 0.5;
                    t_painter.text(
                        Pos2::new(mid_x, t_rect.top() + 4.0),
                        egui::Align2::CENTER_TOP,
                        "💥 COALESCENCE AT r₋ (ARRIVE AT SAME TIME)",
                        egui::FontId::proportional(11.0 * font_scale),
                        Theme::WARNING_RED,
                    );
                } else {
                    let mid_x = (al_x + bob_x) * 0.5;
                    let diff_text = if use_km {
                        format!("Δr = {}", metric.format_km(metric.r_to_km(diff)))
                    } else {
                        format!("Δr = {:.2}M", diff)
                    };
                    t_painter.text(
                        Pos2::new(mid_x, t_rect.top() + 4.0),
                        egui::Align2::CENTER_TOP,
                        diff_text,
                        egui::FontId::monospace(9.0 * font_scale),
                        Theme::TEXT_BRIGHT,
                    );
                }
            }
        }
    }

    fn render_distant_observer(
        &mut self,
        painter: &egui::Painter,
        response: &egui::Response,
        rect: Rect,
        metric: &KerrSchild,
        bob: &mut Observer,
        alice: &Option<Observer>,
        current_time: f64,
        show_wavefronts: bool,
        use_km: bool,
        font_scale: f32,
    ) {
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;

        // Mouse drag background panning for time and radial offset
        if response.dragged() && !self.is_dragging_bob {
            let delta = response.drag_delta();
            let dt = (delta.y as f64 / rect.height() as f64) * (t_max - t_min);
            self.time_offset += dt;
            let dr = (delta.x as f64 / rect.width() as f64) * self.max_r;
            self.r_offset = (self.r_offset - dr).max(0.0);
        }

        let to_screen_x = |r: f64| -> f32 {
            let frac = ((r - self.r_offset) / self.max_r) as f32;
            rect.left() + frac * rect.width()
        };

        let to_screen_y = |t: f64| -> f32 {
            let frac = ((t - t_min) / (t_max - t_min)) as f32;
            rect.bottom() - frac * rect.height()
        };

        let to_coord_r = |screen_x: f32| -> f64 {
            let frac = ((screen_x - rect.left()) / rect.width()) as f64;
            (self.r_offset + frac * self.max_r).max(0.0)
        };

        let to_coord_t = |screen_y: f32| -> f64 {
            let frac = ((rect.bottom() - screen_y) / rect.height()) as f64;
            t_min + frac * (t_max - t_min)
        };

        // Paint background
        painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);

        // Horizons and Boundaries
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();

        let clamp_x = |x: f32| x.clamp(rect.left(), rect.right());
        let x_singularity = clamp_x(to_screen_x(0.0));
        let x_rm = clamp_x(to_screen_x(rm));
        let x_rp = clamp_x(to_screen_x(rp));
        let x_re = clamp_x(to_screen_x(re));

        // Shaded Spacetime Regions (safely clamped)
        // Region III: [0, rm]
        if x_rm > x_singularity {
            let rect_r3 = Rect::from_min_max(Pos2::new(x_singularity, rect.top()), Pos2::new(x_rm, rect.bottom()));
            painter.rect_filled(rect_r3, 0.0, Theme::REGION_III_FILL);
        }

        // Region II: [rm, rp]
        if x_rp > x_rm {
            let rect_r2 = Rect::from_min_max(Pos2::new(x_rm, rect.top()), Pos2::new(x_rp, rect.bottom()));
            painter.rect_filled(rect_r2, 0.0, Theme::REGION_II_FILL);
        }

        // Ergosphere zone: [rp, re]
        if x_re > x_rp {
            let rect_ergo = Rect::from_min_max(Pos2::new(x_rp, rect.top()), Pos2::new(x_re, rect.bottom()));
            painter.rect_filled(rect_ergo, 0.0, Theme::ERGOSPHERE_FILL);
        }

        // Region I: [re, r_offset + max_r]
        if rect.right() > x_re {
            let rect_r1 = Rect::from_min_max(Pos2::new(x_re, rect.top()), Pos2::new(rect.right(), rect.bottom()));
            painter.rect_filled(rect_r1, 0.0, Theme::REGION_I_FILL);
        }

        // 1. Horizontal Time Grid & Vertical Time Axis Labels
        let raw_t_step = (t_max - t_min) / 7.0;
        let t_step = if raw_t_step > 20.0 {
            25.0
        } else if raw_t_step > 10.0 {
            10.0
        } else if raw_t_step > 4.0 {
            5.0
        } else if raw_t_step > 1.5 {
            2.0
        } else if raw_t_step > 0.7 {
            1.0
        } else {
            0.5
        };

        let first_t = (t_min / t_step).floor() as i32;
        let last_t = (t_max / t_step).ceil() as i32;
        for i in first_t..=last_t {
            let t_val = (i as f64) * t_step;
            let y = to_screen_y(t_val);
            if y >= rect.top() && y <= rect.bottom() {
                painter.line_segment(
                    [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                    Stroke::new(0.8, Color32::from_rgba_premultiplied(45, 52, 72, 90)),
                );
                // Time tick label on the left margin
                let t_label = if use_km {
                    format!("t = {}", metric.format_physical_time(t_val))
                } else {
                    format!("t = {:+}M", t_val as i32)
                };
                painter.text(
                    Pos2::new(rect.left() + 4.0, y - 2.0),
                    egui::Align2::LEFT_BOTTOM,
                    t_label,
                    egui::FontId::monospace(9.0 * font_scale),
                    Color32::from_rgba_premultiplied(140, 165, 195, 180),
                );
            }
        }

        // 2. Vertical Radial Grid & Tick Labels
        if use_km {
            let min_km = self.r_offset * metric.r_grav_km();
            let max_km = (self.r_offset + self.max_r) * metric.r_grav_km();
            let target_step = (self.max_r * metric.r_grav_km() / 7.0).max(1e-6);
            let power = 10.0_f64.powf(target_step.log10().floor());
            let mantissa = target_step / power;
            let km_step = if mantissa < 1.5 { 1.0 * power } else if mantissa < 3.5 { 2.0 * power } else if mantissa < 7.5 { 5.0 * power } else { 10.0 * power };
            let first_km = (min_km / km_step).floor() * km_step;
            let mut km = first_km;
            while km <= max_km {
                if km >= min_km {
                    let r_val = metric.km_to_r(km);
                    let x = to_screen_x(r_val);
                    if x >= rect.left() && x <= rect.right() {
                        painter.line_segment(
                            [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                            Stroke::new(1.0, Theme::GRID_LINE),
                        );
                        painter.text(
                            Pos2::new(x + 3.0, rect.bottom() - 18.0),
                            egui::Align2::LEFT_BOTTOM,
                            metric.format_grid_km(km, km_step),
                            egui::FontId::monospace(10.0 * font_scale),
                            Theme::TEXT_MUTED,
                        );
                    }
                }
                km += km_step;
            }
        } else {
            let min_r = self.r_offset;
            let max_r = self.r_offset + self.max_r;
            let target_step = (self.max_r / 7.0).max(1e-6);
            let power = 10.0_f64.powf(target_step.log10().floor());
            let mantissa = target_step / power;
            let r_step = if mantissa < 1.5 { 1.0 * power } else if mantissa < 3.5 { 2.0 * power } else if mantissa < 7.5 { 5.0 * power } else { 10.0 * power };
            let first_r = (min_r / r_step).floor() * r_step;
            let mut r_val = first_r;
            while r_val <= max_r {
                if r_val >= min_r {
                    let x = to_screen_x(r_val);
                    if x >= rect.left() && x <= rect.right() {
                        painter.line_segment(
                            [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                            Stroke::new(1.0, Theme::GRID_LINE),
                        );
                        let label = metric.format_grid_m(r_val, r_step);
                        painter.text(
                            Pos2::new(x + 3.0, rect.bottom() - 18.0),
                            egui::Align2::LEFT_BOTTOM,
                            label,
                            egui::FontId::monospace(10.0 * font_scale),
                            Theme::TEXT_MUTED,
                        );
                    }
                }
                r_val += r_step;
            }
        }

        // Prominent Axis Titles with Physical Conversion
        let r_phys_unit = metric.format_physical_distance(1.0);
        let t_phys_unit = metric.format_physical_time(1.0);

        // Horizontal Axis Title (Bottom Right)
        let r_axis_title = if use_km {
            format!("► Radial Distance r  [Kilometers (km) | 1M = {}]", r_phys_unit)
        } else {
            format!("► Radial Distance r  [Units of M = GM/c² : 1M = {}]", r_phys_unit)
        };
        painter.text(
            Pos2::new(rect.right() - 8.0, rect.bottom() - 4.0),
            egui::Align2::RIGHT_BOTTOM,
            r_axis_title,
            egui::FontId::proportional(11.0 * font_scale),
            Theme::TEXT_BRIGHT,
        );

        // Vertical Axis Title (Top Left)
        let t_axis_title = if use_km {
            format!("▲ Coordinate Time t  [Physical Time | 1M = {}]", t_phys_unit)
        } else {
            format!("▲ Coordinate Time t  [Units of M/c = GM/c³ : 1M = {}]", t_phys_unit)
        };
        painter.text(
            Pos2::new(rect.left() + 8.0, rect.top() + 24.0),
            egui::Align2::LEFT_TOP,
            t_axis_title,
            egui::FontId::proportional(11.0 * font_scale),
            Color32::from_rgb(135, 185, 255),
        );

        // Boundary lines
        // Singularity (r = 0)
        let x_sing_actual = to_screen_x(0.0);
        if x_sing_actual >= rect.left() && x_sing_actual <= rect.right() {
            painter.line_segment(
                [Pos2::new(x_sing_actual + 1.0, rect.top()), Pos2::new(x_sing_actual + 1.0, rect.bottom())],
                Stroke::new(3.0, Theme::SINGULARITY_LINE),
            );
        }

        // Inner Cauchy Horizon r-
        let x_rm_actual = to_screen_x(rm);
        if x_rm_actual >= rect.left() && x_rm_actual <= rect.right() {
            painter.line_segment([Pos2::new(x_rm_actual, rect.top()), Pos2::new(x_rm_actual, rect.bottom())], Stroke::new(2.5, Theme::HORIZON_CAUCHY));
            let rm_label = if use_km {
                format!("Cauchy Horizon r₋ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rm)), rm)
            } else {
                format!("Cauchy Horizon r₋ = {:.2}M ({})", rm, metric.format_physical_distance(rm))
            };
            painter.text(
                Pos2::new(x_rm_actual + 4.0, rect.top() + 42.0),
                egui::Align2::LEFT_TOP,
                rm_label,
                egui::FontId::proportional(11.0 * font_scale),
                Theme::HORIZON_CAUCHY,
            );
        }

        // Outer Event Horizon r+
        let x_rp_actual = to_screen_x(rp);
        if x_rp_actual >= rect.left() && x_rp_actual <= rect.right() {
            painter.line_segment([Pos2::new(x_rp_actual, rect.top()), Pos2::new(x_rp_actual, rect.bottom())], Stroke::new(2.5, Theme::HORIZON_OUTER));
            let rp_label = if use_km {
                format!("Event Horizon r₊ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rp)), rp)
            } else {
                format!("Event Horizon r₊ = {:.2}M ({})", rp, metric.format_physical_distance(rp))
            };
            painter.text(
                Pos2::new(x_rp_actual + 4.0, rect.top() + 58.0),
                egui::Align2::LEFT_TOP,
                rp_label,
                egui::FontId::proportional(11.0 * font_scale),
                Theme::HORIZON_OUTER,
            );
        }

        // Ergosphere boundary line
        let x_re_actual = to_screen_x(re);
        if x_re_actual >= rect.left() && x_re_actual <= rect.right() {
            painter.line_segment([Pos2::new(x_re_actual, rect.top()), Pos2::new(x_re_actual, rect.bottom())], Stroke::new(1.5, Theme::ERGOSPHERE_LINE));
            let re_label = if use_km {
                format!("Ergosphere r_E = {} ({:.2}M)", metric.format_km(metric.r_to_km(re)), re)
            } else {
                format!("Ergosphere r_E = {:.2}M ({})", re, metric.format_physical_distance(re))
            };
            painter.text(
                Pos2::new(x_re_actual + 4.0, rect.top() + 74.0),
                egui::Align2::LEFT_TOP,
                re_label,
                egui::FontId::proportional(11.0 * font_scale),
                Theme::ERGOSPHERE_LINE,
            );
        }

        // Ingoing Wavefront Pulses
        if show_wavefronts {
            let wave_spacing = 1.5;
            let first_wave = ((t_min - self.max_r) / wave_spacing).floor() as i32;
            let last_wave = ((t_max + self.max_r) / wave_spacing).ceil() as i32;

            for i in first_wave..=last_wave {
                let wave_t0 = (i as f64) * wave_spacing;
                let p1 = Pos2::new(to_screen_x(self.max_r), to_screen_y(wave_t0));
                let p2 = Pos2::new(to_screen_x(0.0), to_screen_y(wave_t0 + self.max_r));

                if (p1.y >= rect.top() && p1.y <= rect.bottom()) || (p2.y >= rect.top() && p2.y <= rect.bottom()) {
                    painter.line_segment([p1, p2], Stroke::new(1.0, Theme::WAVEFRONT_PULSE));
                }
            }
        }

        // Alice Worldline & Marker
        if let Some(al) = alice {
            if al.trail.len() >= 2 {
                let points: Vec<Pos2> = al
                    .trail
                    .iter()
                    .map(|&[t, r]| Pos2::new(to_screen_x(r), to_screen_y(t)))
                    .collect();
                painter.add(PathShape::line(points, Stroke::new(2.0, Theme::ALICE_COLOR)));
            }

            if al.is_active {
                let alice_pos = Pos2::new(to_screen_x(al.r), to_screen_y(al.t));
                if rect.contains(alice_pos) {
                    painter.circle_filled(alice_pos, 5.5, Theme::ALICE_COLOR);
                    painter.circle_stroke(alice_pos, 7.5, Stroke::new(1.0, Color32::WHITE));
                    draw_hovering_telemetry(painter, rect, alice_pos, "Alice", Theme::ALICE_COLOR, al, metric, use_km, font_scale);
                }
            }
        }

        // Bob Worldline & Dragging
        if bob.trail.len() >= 2 {
            let points: Vec<Pos2> = bob
                .trail
                .iter()
                .map(|&[t, r]| Pos2::new(to_screen_x(r), to_screen_y(t)))
                .collect();
            painter.add(PathShape::line(points, Stroke::new(2.5, Theme::BOB_COLOR)));
        }

        let bob_pos = Pos2::new(to_screen_x(bob.r), to_screen_y(bob.t));
        let bob_radius = 8.0;

        if response.drag_started() {
            if let Some(mouse_pos) = response.interact_pointer_pos() {
                if mouse_pos.distance(bob_pos) < bob_radius * 3.0 {
                    self.is_dragging_bob = true;
                }
            }
        }

        if self.is_dragging_bob && response.dragged() {
            if let Some(mouse_pos) = response.interact_pointer_pos() {
                let new_r = to_coord_r(mouse_pos.x).clamp(0.04, self.max_r);
                let new_t = to_coord_t(mouse_pos.y);
                bob.set_drag_position(new_t, new_r);
            }
        }

        if response.drag_stopped() {
            self.is_dragging_bob = false;
        }

        // Bob's Exact Light Cone
        let apex = Pos2::new(to_screen_x(bob.r), to_screen_y(bob.t));

        if bob.r > 0.02 && bob.is_active {
            // Bob's Exact Light Cone: span scales proportionally with zoom window
            let time_cone_span = (self.time_window * 0.12).min(self.max_r * 1.5).clamp(1e-4, 1.8);
            let cone = bob.compute_lightcone_polygon(metric, time_cone_span);

            let cone_apex = Pos2::new(to_screen_x(cone.apex[1]), to_screen_y(cone.apex[0]));
            let p_fut_in = Pos2::new(to_screen_x(cone.future_in[1]), to_screen_y(cone.future_in[0]));
            let p_fut_out = Pos2::new(to_screen_x(cone.future_out[1]), to_screen_y(cone.future_out[0]));
            let p_past_in = Pos2::new(to_screen_x(cone.past_in[1]), to_screen_y(cone.past_in[0]));
            let p_past_out = Pos2::new(to_screen_x(cone.past_out[1]), to_screen_y(cone.past_out[0]));

            // Future cone fill & borders
            painter.add(PathShape::convex_polygon(
                vec![cone_apex, p_fut_in, p_fut_out],
                Theme::LIGHTCONE_FUTURE_FILL,
                egui::epaint::PathStroke::NONE,
            ));
            // Past cone fill & borders
            painter.add(PathShape::convex_polygon(
                vec![cone_apex, p_past_in, p_past_out],
                Theme::LIGHTCONE_PAST_FILL,
                egui::epaint::PathStroke::NONE,
            ));

            // Collinear rays:
            // Ingoing ray: p_past_in (past-right) -> apex -> p_fut_in (future-left) with slope dr/dt = -1
            painter.line_segment([cone_apex, p_fut_in], Stroke::new(1.8, Theme::LIGHTCONE_BORDER_INGOING));
            painter.line_segment([p_past_in, cone_apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_INGOING));
            // Outgoing ray: p_past_out -> apex -> p_fut_out
            painter.line_segment([cone_apex, p_fut_out], Stroke::new(2.2, Theme::LIGHTCONE_BORDER_OUTGOING));
            painter.line_segment([p_past_out, cone_apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_OUTGOING));
        } else if bob.is_active {
            // Singularity collision marker
            painter.circle_filled(apex, 10.0, Theme::SINGULARITY_FILL);
            painter.circle_stroke(apex, 12.0, Stroke::new(1.5, Theme::SINGULARITY_LINE));
        }

        // Bob avatar circle
        painter.circle_filled(
            apex,
            bob_radius,
            if self.is_dragging_bob { Color32::WHITE } else { Theme::BOB_COLOR },
        );
        painter.circle_stroke(apex, bob_radius + 2.0, Stroke::new(1.5, Color32::WHITE));
        draw_hovering_telemetry(painter, rect, apex, "Bob", Theme::BOB_COLOR, bob, metric, use_km, font_scale);

        // Zoom hint overlay in top-left
        painter.text(
            Pos2::new(rect.left() + 8.0, rect.top() + 8.0),
            egui::Align2::LEFT_TOP,
            format!("🔍 Zoom: {:.1}x (Scroll wheel to zoom, drag background to pan time)", 5.5 / self.max_r),
            egui::FontId::proportional(10.0 * font_scale),
            Theme::TEXT_MUTED,
        );

        // Light cone slope telemetry box
        let slope_msg = if bob.r > 0.02 {
            let cone = bob.compute_lightcone_polygon(metric, 1.8);
            format!(
                "Null Slopes at Bob:\nOutgoing dr/dt = {:+.3}\nIngoing dr/dt = {:+.3}\nDrag dϕ/dt = {:+.3}",
                cone.dr_dt_out, cone.dr_dt_in, cone.dphi_dt_out
            )
        } else {
            "Singularity r = 0 Reached\nLight cone terminated\nCurvature Riem² → ∞".to_string()
        };
        painter.text(
            Pos2::new(rect.right() - 10.0, rect.top() + 10.0),
            egui::Align2::RIGHT_TOP,
            slope_msg,
            egui::FontId::monospace(11.0 * font_scale),
            Theme::TEXT_BRIGHT,
        );
    }

    fn render_observer_frame(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        metric: &KerrSchild,
        focus_obs: &Observer,
        other_obs: Option<&Observer>,
        use_km: bool,
        font_scale: f32,
    ) {
        let center = rect.center();
        let scale = (rect.width() / (self.max_r as f32).max(1e-5)) * 0.45;

        // 1. Grid lines in observer rest frame (xi, tau)
        let grid_stroke = Stroke::new(0.8, Color32::from_rgba_premultiplied(45, 52, 72, 90));
        painter.line_segment([Pos2::new(rect.left(), center.y), Pos2::new(rect.right(), center.y)], grid_stroke);
        painter.line_segment([Pos2::new(center.x, rect.top()), Pos2::new(center.x, rect.bottom())], grid_stroke);

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let r_obs = focus_obs.r;

        // 2. Horizon & Singularity Tilting Dynamics in Observer's Co-moving Frame
        // -------------------------------------------------------------------------
        // In general relativity:
        // - Region I (r > r₊): g^rr > 0 (spacelike). All surfaces of constant r (r₊, r₋, r=0) are TIMELIKE -> strictly VERTICAL.
        // - Crossing r₊: g^rr -> 0. r flips to timelike. Horizons tilt into horizontal boundaries!
        // - Region II (r₋ < r < r₊): g^rr < 0 (timelike). r₊ is a past boundary (horizontal below); r₋ is a future boundary (horizontal above).
        // - Crossing r₋: g^rr -> 0. r flips back to spacelike! r₋ un-tilts back to VERTICAL.
        // - Region III (r < r₋): g^rr > 0 (spacelike). r₋ is vertical behind; Ring Singularity r=0 is vertical ahead (timelike singularity of Kerr metric).

        // --- OUTER EVENT HORIZON r₊ ---
        let (theta_p, p_anchor_rp, rp_line1, rp_line2) = if r_obs > rp + 0.15 {
            // Region I: strictly vertical line ahead in space
            let delta_rp = (rp - r_obs) as f32;
            let anchor = Pos2::new(center.x + delta_rp * scale, center.y);
            (0.0_f32, anchor, "Event Horizon r₊", "[r₊ Spacelike Ahead]")
        } else if r_obs < rp - 0.15 {
            // Region II / III: horizontal boundary in the past (below center)
            let tau_past = (rp - r_obs).abs() as f32;
            let anchor = Pos2::new(center.x, center.y + tau_past * scale);
            (std::f32::consts::FRAC_PI_2, anchor, "Event Horizon r₊", "[Timelike in PAST]")
        } else {
            // Crossing r₊: tilts smoothly from 0 to pi/2
            let frac = ((rp + 0.15 - r_obs) / 0.30) as f32;
            let theta = frac * std::f32::consts::FRAC_PI_2;
            let delta_rp = (rp - r_obs) as f32;
            let tau_past = (rp - r_obs).abs() as f32;
            let anchor = Pos2::new(
                center.x + delta_rp * scale * theta.cos(),
                center.y + tau_past * scale * theta.sin(),
            );
            (theta, anchor, "Event Horizon r₊", "[Tilting across r₊]")
        };

        // Draw r₊ horizon line
        let dir_rp = Vec2::new(theta_p.sin(), -theta_p.cos());
        painter.line_segment(
            [p_anchor_rp - dir_rp * 1200.0, p_anchor_rp + dir_rp * 1200.0],
            Stroke::new(2.5, Theme::HORIZON_OUTER),
        );

        // Place r₊ note at display edges in 2 compact lines (no overlap with central cone)
        if theta_p > std::f32::consts::FRAC_PI_4 {
            // Horizontal line across display: place note at right margin
            let y_clamped = p_anchor_rp.y.clamp(rect.top() + 20.0, rect.bottom() - 24.0);
            painter.text(
                Pos2::new(rect.right() - 8.0, y_clamped - 2.0),
                egui::Align2::RIGHT_BOTTOM,
                format!("{}\n{}", rp_line1, rp_line2),
                egui::FontId::proportional(10.0 * font_scale),
                Theme::HORIZON_OUTER,
            );
        } else {
            // Vertical line down display: place note at top margin
            let x_clamped = p_anchor_rp.x.clamp(rect.left() + 8.0, rect.right() - 85.0);
            painter.text(
                Pos2::new(x_clamped + 4.0, rect.top() + 26.0),
                egui::Align2::LEFT_TOP,
                format!("{}\n{}", rp_line1, rp_line2),
                egui::FontId::proportional(10.0 * font_scale),
                Theme::HORIZON_OUTER,
            );
        }

        // --- INNER CAUCHY HORIZON r₋ ---
        let (theta_m, p_anchor_rm, rm_line1, rm_line2) = if r_obs > rp + 0.15 {
            // Region I: strictly vertical line ahead in space (NOT horizontal!)
            let delta_rm = (rm - r_obs) as f32;
            let anchor = Pos2::new(center.x + delta_rm * scale, center.y);
            (0.0_f32, anchor, "Cauchy Horizon r₋", "[r₋ Spacelike Ahead]")
        } else if r_obs >= rp - 0.15 {
            // Crossing r₊ into Region II: rotates from vertical (0) to horizontal (pi/2 in future)
            let frac = ((rp + 0.15 - r_obs) / 0.30) as f32;
            let theta = frac * std::f32::consts::FRAC_PI_2;
            let delta_rm = (rm - r_obs) as f32;
            let tau_fut = (r_obs - rm).max(0.0) as f32;
            let anchor = Pos2::new(
                center.x + delta_rm * scale * theta.cos(),
                center.y - tau_fut * scale * theta.sin(),
            );
            (theta, anchor, "Cauchy Horizon r₋", "[Tilting to Future]")
        } else if r_obs > rm + 0.15 {
            // Region II: strictly horizontal inescapable future boundary (above center)
            let tau_fut = (r_obs - rm) as f32;
            let anchor = Pos2::new(center.x, center.y - tau_fut * scale);
            (std::f32::consts::FRAC_PI_2, anchor, "Cauchy Horizon r₋", "[Inescapable FUTURE]")
        } else if r_obs >= rm - 0.15 {
            // Crossing r₋ into Region III: un-tilts from horizontal (pi/2) back to vertical (0)
            let frac = ((r_obs - (rm - 0.15)) / 0.30) as f32;
            let theta = frac * std::f32::consts::FRAC_PI_2;
            let delta_rm = (rm - r_obs) as f32;
            let tau_fut = (r_obs - rm).max(0.0) as f32;
            let anchor = Pos2::new(
                center.x + delta_rm * scale * theta.cos(),
                center.y - tau_fut * scale * theta.sin(),
            );
            (theta, anchor, "Cauchy Horizon r₋", "[Un-tilting across r₋]")
        } else {
            // Region III: vertical line behind in space
            let delta_rm = (rm - r_obs) as f32;
            let anchor = Pos2::new(center.x + delta_rm * scale, center.y);
            (0.0_f32, anchor, "Cauchy Horizon r₋", "[r₋ Spacelike Behind]")
        };

        // Draw r₋ horizon line
        let dir_rm = Vec2::new(theta_m.sin(), -theta_m.cos());
        painter.line_segment(
            [p_anchor_rm - dir_rm * 1200.0, p_anchor_rm + dir_rm * 1200.0],
            Stroke::new(2.5, Theme::HORIZON_CAUCHY),
        );

        // Place r₋ note at display edges in 2 compact lines
        if theta_m > std::f32::consts::FRAC_PI_4 {
            let y_clamped = p_anchor_rm.y.clamp(rect.top() + 20.0, rect.bottom() - 24.0);
            painter.text(
                Pos2::new(rect.right() - 8.0, y_clamped - 2.0),
                egui::Align2::RIGHT_BOTTOM,
                format!("{}\n{}", rm_line1, rm_line2),
                egui::FontId::proportional(10.0 * font_scale),
                Theme::HORIZON_CAUCHY,
            );
        } else {
            let x_clamped = p_anchor_rm.x.clamp(rect.left() + 8.0, rect.right() - 85.0);
            painter.text(
                Pos2::new(x_clamped + 4.0, rect.top() + 48.0),
                egui::Align2::LEFT_TOP,
                format!("{}\n{}", rm_line1, rm_line2),
                egui::FontId::proportional(10.0 * font_scale),
                Theme::HORIZON_CAUCHY,
            );
        }

        // --- RING SINGULARITY r = 0 ---
        // In Region I (r > rp) and Region III (r < rm), r=0 is ahead in space as a vertical line.
        if r_obs < rm || r_obs > rp {
            let x_sing = center.x - (r_obs as f32) * scale;
            if x_sing >= rect.left() - 50.0 && x_sing <= rect.right() + 50.0 {
                painter.line_segment(
                    [Pos2::new(x_sing, rect.top()), Pos2::new(x_sing, rect.bottom())],
                    Stroke::new(3.0, Theme::SINGULARITY_LINE),
                );
                let x_clamped = x_sing.clamp(rect.left() + 8.0, rect.right() - 110.0);
                painter.text(
                    Pos2::new(x_clamped + 4.0, rect.top() + 70.0),
                    egui::Align2::LEFT_TOP,
                    "Ring Singularity r = 0\n[Timelike Core Boundary]",
                    egui::FontId::proportional(10.0 * font_scale),
                    Theme::SINGULARITY_LINE,
                );
            }
        }

        // 3. Primary Observer's 45-Degree Minkowski Light Cone (Permanently locked at 45°)
        let cone_len = (rect.height() * 0.35).min(rect.width() * 0.35);
        let apex = center;

        if focus_obs.r > 0.02 && focus_obs.is_active {
            let p_fut_out = apex + Vec2::new(cone_len, -cone_len); // +45 deg (future outgoing)
            let p_fut_in = apex + Vec2::new(-cone_len, -cone_len); // -45 deg (future ingoing)
            let p_past_out = apex + Vec2::new(cone_len, cone_len);
            let p_past_in = apex + Vec2::new(-cone_len, cone_len);

            painter.add(PathShape::convex_polygon(
                vec![apex, p_fut_in, p_fut_out],
                Theme::LIGHTCONE_FUTURE_FILL,
                egui::epaint::PathStroke::NONE,
            ));
            painter.add(PathShape::convex_polygon(
                vec![apex, p_past_out, p_past_in],
                Theme::LIGHTCONE_PAST_FILL,
                egui::epaint::PathStroke::NONE,
            ));

            // 45° boundary lines
            painter.line_segment([apex, p_fut_in], Stroke::new(2.2, Theme::LIGHTCONE_BORDER_INGOING));
            painter.line_segment([p_past_out, apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_INGOING));
            painter.line_segment([apex, p_fut_out], Stroke::new(2.2, Theme::LIGHTCONE_BORDER_OUTGOING));
            painter.line_segment([p_past_in, apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_OUTGOING));

            // 45 degree angle indicators
            painter.text(
                p_fut_out + Vec2::new(4.0, -2.0),
                egui::Align2::LEFT_BOTTOM,
                "+45° Outgoing",
                egui::FontId::monospace(9.0 * font_scale),
                Theme::LIGHTCONE_BORDER_OUTGOING,
            );
            painter.text(
                p_fut_in + Vec2::new(-4.0, -2.0),
                egui::Align2::RIGHT_BOTTOM,
                "-45° Ingoing",
                egui::FontId::monospace(9.0 * font_scale),
                Theme::LIGHTCONE_BORDER_INGOING,
            );
        } else if focus_obs.is_active {
            // Singularity collision: light cone terminates
            painter.circle_filled(center, 12.0, Theme::SINGULARITY_FILL);
            painter.circle_stroke(center, 15.0, Stroke::new(2.0, Theme::SINGULARITY_LINE));
            painter.text(
                Pos2::new(center.x + 18.0, center.y),
                egui::Align2::LEFT_CENTER,
                "💥 SINGULARITY IMPACT\nLight cone terminated at r = 0",
                egui::FontId::proportional(11.0 * font_scale),
                Theme::WARNING_RED,
            );
        }

        // Primary Observer Avatar
        let obs_color = if focus_obs.name == "Alice" { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        painter.circle_filled(apex, 7.5, obs_color);
        painter.circle_stroke(apex, 9.5, Stroke::new(1.5, Color32::WHITE));

        // 4. Secondary Observer (Relative position, convergence & boosted light cone)
        if let Some(other) = other_obs {
            if other.is_active {
                // Coordinate transformation into focus observer's local rest frame:
                // Radial difference between observers
                let dr = (other.r - focus_obs.r) as f32;

                // In the observer's local rest frame, simultaneity is defined along their local spacelike tetrad.
                // In Region II (r- < r < r+), lines of constant r are spacelike (horizontal at distance tau_fut = r_obs - r-).
                // Alice's position along the horizon normal is (other.r - r-) vs Bob's (focus_obs.r - r-).
                let other_pos = if r_obs > rp + 0.15 {
                    // Region I: vertical coordinate r is spacelike horizontal, t is timelike vertical.
                    // Scale dt by local time dilation so Alice doesn't jump off screen.
                    let dt = ((other.t - focus_obs.t).clamp(-3.0, 3.0)) as f32;
                    center + Vec2::new(dr * scale, -dt * scale)
                } else if r_obs >= rm {
                    // Region II / Horizon Crossing:
                    // Radial coordinate r is timelike (points toward Cauchy horizon).
                    // As both observers approach r-, dr = other.r - focus_obs.r -> 0!
                    // On Bob's rest-frame display, the Cauchy horizon is at distance (r_obs - rm) in the future (above Bob).
                    // Alice's position relative to Bob along the approach to r- is given by her radial offset:
                    let dy_radial = ((focus_obs.r - other.r) as f32) * scale;
                    // Lateral drift from azimuthal/time separation:
                    let dx_lateral = (dr * 0.5) * scale;
                    center + Vec2::new(dx_lateral, -dy_radial)
                } else {
                    // Region III: inside Cauchy core, r is spacelike again
                    center + Vec2::new(dr * scale, 0.0)
                };

                if rect.contains(other_pos) {
                    let other_color = if other.name == "Alice" { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };

                    // Boosted light cone for secondary observer
                    let v_rel = ((other.velocity_c(metric) - focus_obs.velocity_c(metric))
                        / (1.0 - other.velocity_c(metric) * focus_obs.velocity_c(metric))).clamp(-0.95, 0.95);
                    let slope_in = ((-1.0 - v_rel) / (1.0 + v_rel)) as f32;

                    if other.r > 0.02 {
                        let other_cone_len = (cone_len * 0.45).max(18.0);

                        let o_fut_out = other_pos + Vec2::new(other_cone_len, -other_cone_len);
                        let o_fut_in = other_pos + Vec2::new(slope_in * other_cone_len, -other_cone_len);

                        painter.add(PathShape::convex_polygon(
                            vec![other_pos, o_fut_in, o_fut_out],
                            Color32::from_rgba_premultiplied(other_color.r(), other_color.g(), other_color.b(), 1),
                            egui::epaint::PathStroke::NONE,
                        ));
                        painter.line_segment([other_pos, o_fut_in], Stroke::new(1.2, other_color));
                        painter.line_segment([other_pos, o_fut_out], Stroke::new(1.2, other_color));
                    } else {
                        // Secondary observer singularity impact
                        painter.circle_filled(other_pos, 8.0, Theme::SINGULARITY_FILL);
                        painter.circle_stroke(other_pos, 10.0, Stroke::new(1.5, Theme::SINGULARITY_LINE));
                    }

                    // Other avatar
                    painter.circle_filled(other_pos, 6.0, other_color);
                    painter.circle_stroke(other_pos, 7.5, Stroke::new(1.0, Color32::WHITE));

                    // Relative separation bracket
                    painter.line_segment([center, other_pos], Stroke::new(1.2, Color32::from_white_alpha(120)));

                    // Telemetry card for secondary observer
                    draw_hovering_telemetry(painter, rect, other_pos, &other.name, other_color, other, metric, use_km, font_scale);
                }
            }
        }

        // Telemetry card for primary observer
        draw_hovering_telemetry(painter, rect, apex, &focus_obs.name, obs_color, focus_obs, metric, use_km, font_scale);

        // Header Title Banner
        painter.text(
            Pos2::new(rect.left() + 8.0, rect.top() + 8.0),
            egui::Align2::LEFT_TOP,
            format!(
                "🔭 {}'S REST FRAME  |  Local Minkowski Space (c ≡ 1, 45° Light Cones)\n\
                 Horizons Tilt Dynamically: Vertical in Spacelike Regions (I & III) ↔ Horizontal in Timelike Region (II)",
                focus_obs.name.to_uppercase()
            ),
            egui::FontId::proportional(11.0 * font_scale),
            Color32::from_rgb(150, 220, 255),
        );
    }
}
