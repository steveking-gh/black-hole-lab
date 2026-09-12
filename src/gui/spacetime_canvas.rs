use crate::gui::controls::ReferenceFrame;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::local_frame::{LocalFrame, SurfaceCharacter};
use crate::physics::observer::{Observer, ObserverMode};
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
    // Exact shift of ingoing principal null light: nu_obs/nu_inf = -k.u = u^t + u^r - a u^phi.
    let nu_ratio = obs.ingoing_frequency_ratio(metric);

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

    // The conserved constants of the worldline actually being integrated, for free-fallers.
    let constants_str = match obs.geodesic {
        Some(geo) if obs.mode == ObserverMode::FreeFall => {
            Some(format!("E = {:.3}  L = {:.3} M", geo.energy, geo.l_ang))
        }
        _ => None,
    };

    let shift_tag = if nu_ratio > 1.0 { "blueshift" } else { "redshift" };
    let nu_str = if nu_ratio < 0.01 {
        format!("ν_in/ν_∞ = {:.2e} ({})", nu_ratio, shift_tag)
    } else {
        format!("ν_in/ν_∞ = {:.2} ({})", nu_ratio, shift_tag)
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
    let w_nu = painter.layout_no_wrap(nu_str.clone(), font_body.clone(), color).size().x;

    let w_constants = constants_str
        .as_ref()
        .map(|s| painter.layout_no_wrap(s.clone(), font_body.clone(), color).size().x)
        .unwrap_or(0.0);

    let max_text_w = w_title.max(w_a).max(w_tidal).max(w_nu).max(w_constants);
    let pad_x = 10.0 * font_scale;
    let pad_y = 6.0 * font_scale;
    let line_spacing = 13.0 * font_scale;

    let body_lines = if constants_str.is_some() { 4.0 } else { 3.0 };
    let badge_w = (max_text_w + pad_x * 2.0).max(180.0 * font_scale);
    let badge_h = (pad_y * 2.0 + line_spacing * (body_lines + 0.8)).max(56.0 * font_scale);

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
        nu_str,
        font_body.clone(),
        if nu_ratio > 1.0 { Theme::BLUESHIFT_BLUE } else { Theme::TEXT_MUTED },
    );
    if let Some(constants_str) = constants_str {
        painter.text(
            Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing * 4.0),
            egui::Align2::LEFT_TOP,
            constants_str,
            font_body,
            Theme::TEXT_MUTED,
        );
    }
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

    /// Render the (t, r) spacetime foliation canvas with an integrated, perfectly aligned 1D radial track
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
        // 2. INTEGRATED 1D RADIAL TRACK (PIXEL-PERFECT HORIZONTAL ALIGNMENT)
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
            "1D RADIAL TRACK  |  Radial Axis r [Kilometers (km)]".to_string()
        } else {
            format!(
                "1D RADIAL TRACK  |  Radial Axis r  [1M = GM/c² = {}]",
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

        // Radial separation between Alice and Bob, if both are present
        if let Some(al) = alice {
            if al.is_active && bob.is_active {
                let diff = (bob.r - al.r).abs();
                let al_x = track_to_x(al.r);
                let bob_x = track_to_x(bob.r);

                // Distance bracket / line
                t_painter.line_segment([Pos2::new(al_x, center_y), Pos2::new(bob_x, center_y)], Stroke::new(2.0, Color32::WHITE));

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
                    .map(|&[t, r, _phi]| Pos2::new(to_screen_x(r), to_screen_y(t)))
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
                .map(|&[t, r, _phi]| Pos2::new(to_screen_x(r), to_screen_y(t)))
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
                "Null Wedge at Bob (ZAMO rays):\nOutgoing dr/dt (L=0 ray) = {:+.3}\nIngoing dr/dt (L=0 ray) = {:+.3}\nOutgoing dϕ/dt (L=0 ray) = {:+.3}",
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

    /// Render the focus observer's rest frame: the *first-order local inertial chart* defined by
    /// their orthonormal tetrad.
    ///
    /// Everything in the picture is placed by one linear map, the dual tetrad
    /// xi^a = e^a_mu Delta x^mu (see `LocalFrame`): the surfaces r = const (the ring singularity,
    /// both horizons and the static limit), the other observer's event, and the direction of their
    /// worldline. Because the map is linear and the frame is orthonormal, light cones are at
    /// exactly 45 degrees everywhere in the picture - the focus observer's and the other
    /// observer's alike - and the tilt of every surface comes out of the geometry rather than out
    /// of a drawing rule: a surface r = const is steeper than 45 degrees where g^rr > 0, at 45
    /// degrees on a horizon, and flatter than 45 degrees between them.
    ///
    /// The chart is exact at the focus observer's own event, where all of those orientations live,
    /// and linearised for finite offsets (how far away a horizon is drawn, where the other
    /// observer sits). The banner says so.
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
        let to_screen = |xi1: f64, xi0: f64| -> Pos2 {
            Pos2::new(
                center.x + (xi1 as f32) * scale,
                center.y - (xi0 as f32) * scale,
            )
        };

        // 1. The chart's own axes: the observer's worldline xi^1 = 0 and their local space xi^0 = 0.
        let grid_stroke = Stroke::new(0.8, Color32::from_rgba_premultiplied(45, 52, 72, 90));
        painter.line_segment(
            [Pos2::new(rect.left(), center.y), Pos2::new(rect.right(), center.y)],
            grid_stroke,
        );
        painter.line_segment(
            [Pos2::new(center.x, rect.top()), Pos2::new(center.x, rect.bottom())],
            grid_stroke,
        );

        let frame = LocalFrame::for_observer(metric, focus_obs.r, &focus_obs.four_velocity(metric));

        // 2. Surfaces r = const, every one of them placed by the dual tetrad.
        let ergo_faint = Color32::from_rgba_unmultiplied(
            Theme::ERGOSPHERE_LINE.r(),
            Theme::ERGOSPHERE_LINE.g(),
            Theme::ERGOSPHERE_LINE.b(),
            80,
        );
        let surfaces: [(f64, &str, Color32, f32); 4] = [
            (metric.ergosphere_equatorial(), "Static limit 2M", ergo_faint, 1.4),
            (metric.outer_horizon(), "Event Horizon r₊", Theme::HORIZON_OUTER, 2.5),
            (metric.inner_horizon(), "Cauchy Horizon r₋", Theme::HORIZON_CAUCHY, 2.5),
            (0.0, "Ring Singularity r = 0", Theme::SINGULARITY_LINE, 3.0),
        ];

        for (idx, &(r_h, name, color, width)) in surfaces.iter().enumerate() {
            let line = frame.surface_r_const(r_h);
            let anchor = to_screen(line.point[0], line.point[1]);
            let dir = Vec2::new(line.dir[0] as f32, -(line.dir[1] as f32));
            let Some((end_a, end_b)) = clip_line_to_rect(anchor, dir, rect) else {
                continue;
            };
            painter.line_segment([end_a, end_b], Stroke::new(width, color));

            // The causal character is read straight off the drawn slope: |d xi^0 / d xi^1| > 1 is
            // a timelike surface, = 1 a null one, < 1 a spacelike one. The 2e-3 tolerance is a
            // display band on that comparison, not a physical fudge.
            let note = match line.character(2e-3) {
                SurfaceCharacter::Null => "null surface (crossing now)",
                SurfaceCharacter::Timelike => "timelike surface (can be avoided)",
                SurfaceCharacter::Spacelike => {
                    if line.xi0_at_axis().unwrap_or(0.0) >= 0.0 {
                        "spacelike surface (in your future)"
                    } else {
                        "spacelike surface (in your past)"
                    }
                }
            };

            // Quantify the tilt so a 41.8-degree surface is not mistaken for a 45-degree one.
            // A timelike surface is the worldsheet of observers hovering at that r; in this frame it
            // moves at 1/|slope| (a boosted vertical line has slope 1/v). A spacelike surface is
            // a simultaneity slice of the E = 0 observers there; the focus observer's speed relative
            // to them is |slope|, and the surface crosses this worldline at tau = xi^0 on the axis.
            let slope_abs = line.slope().abs();
            let tilt_deg = if slope_abs.is_finite() { slope_abs.atan().to_degrees() } else { 90.0 };
            let detail = match line.character(2e-3) {
                SurfaceCharacter::Timelike => {
                    let xi1 = if line.dir[1].abs() > 1e-12 {
                        line.point[0] - line.dir[0] * line.point[1] / line.dir[1]
                    } else {
                        line.point[0]
                    };
                    let speed = if slope_abs.is_finite() { 1.0 / slope_abs.max(1e-9) } else { 0.0 };
                    format!("tilt {:.1}° • moving at {:.2}c • ξ¹ ≈ {:+.2} M (1st order)", tilt_deg, speed, xi1)
                }
                SurfaceCharacter::Null => format!("tilt {:.1}°", tilt_deg),
                SurfaceCharacter::Spacelike => {
                    let xi0 = line.xi0_at_axis().unwrap_or(0.0);
                    format!("tilt {:.1}° • closing at {:.2}c • on your worldline at τ ≈ {:+.2} M (1st order)", tilt_deg, slope_abs, xi0)
                }
            };
            let label = format!("{}\n{}\n{}", name, note, detail);
            let (label_pos, align) = if line.slope().abs() >= 1.0 {
                // Steep line: hang the label off it, stacked down the top margin.
                let y = (rect.top() + 26.0 + 36.0 * font_scale * (idx as f32)).min(rect.bottom() - 40.0);
                let x = segment_x_at_y(end_a, end_b, y)
                    .clamp(rect.left() + 6.0, rect.right() - 150.0 * font_scale);
                (Pos2::new(x + 5.0, y), egui::Align2::LEFT_TOP)
            } else {
                // Flat line: park the label on it at the right margin.
                let y = segment_y_at_x(end_a, end_b, rect.right() - 10.0)
                    .clamp(rect.top() + 24.0, rect.bottom() - 6.0);
                (Pos2::new(rect.right() - 8.0, y - 3.0), egui::Align2::RIGHT_BOTTOM)
            };
            painter.text(
                label_pos,
                align,
                label,
                egui::FontId::proportional(10.0 * font_scale),
                color,
            );
        }

        // Faint extensions of the observer's own null lines across the whole chart, so that the
        // intercept of a 45-degree ray with a nearly parallel spacelike surface is visible.
        let null_faint = Color32::from_rgba_unmultiplied(200, 220, 255, 45);
        for d in [Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0)] {
            if let Some((a, b)) = clip_line_to_rect(center, d, rect) {
                painter.line_segment([a, b], Stroke::new(0.8, null_faint));
            }
        }

        // 3. The focus observer's own light cone: 45 degrees through the origin, by construction.
        let cone_len = (rect.height() * 0.35).min(rect.width() * 0.35);
        let apex = center;

        if focus_obs.r > 0.02 && focus_obs.is_active {
            let p_fut_out = apex + Vec2::new(cone_len, -cone_len);
            let p_fut_in = apex + Vec2::new(-cone_len, -cone_len);
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

            painter.line_segment([apex, p_fut_in], Stroke::new(2.2, Theme::LIGHTCONE_BORDER_INGOING));
            painter.line_segment([p_past_out, apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_INGOING));
            painter.line_segment([apex, p_fut_out], Stroke::new(2.2, Theme::LIGHTCONE_BORDER_OUTGOING));
            painter.line_segment([p_past_in, apex], Stroke::new(1.2, Theme::LIGHTCONE_BORDER_OUTGOING));

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

        let obs_color = if focus_obs.name == "Alice" { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        painter.circle_filled(apex, 7.5, obs_color);
        painter.circle_stroke(apex, 9.5, Stroke::new(1.5, Color32::WHITE));

        // 4. The other observer: their event, their worldline direction and their light cone, all
        // from the same linear map. The azimuthal component xi^2 is dropped from the picture and
        // printed instead, so the projection is on the record.
        if let Some(other) = other_obs {
            if other.is_active {
                let two_pi = std::f64::consts::TAU;
                let mut d_phi = (other.phi - focus_obs.phi).rem_euclid(two_pi);
                if d_phi > std::f64::consts::PI {
                    d_phi -= two_pi;
                }
                let xi = frame.to_local(&[other.t - focus_obs.t, other.r - focus_obs.r, d_phi]);
                let other_pos = to_screen(xi[1], xi[0]);

                if rect.contains(other_pos) {
                    let other_color = if other.name == "Alice" { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
                    // v^a = e^a_mu u_other^mu is their 4-velocity in this frame: the drawn tangent
                    // is (v^1, v^0) normalised, and |v^1 / v^0| is their radial speed relative to
                    // the focus observer.
                    let v = frame.vector_to_local(&other.four_velocity(metric));
                    let len = ((v[0] * v[0] + v[1] * v[1]) as f32).sqrt().max(1e-9);
                    let tangent = Vec2::new((v[1] as f32) / len, -(v[0] as f32) / len);
                    let wl_len = (cone_len * 0.5).max(16.0);
                    painter.line_segment(
                        [other_pos - tangent * wl_len * 0.4, other_pos + tangent * wl_len],
                        Stroke::new(1.6, other_color),
                    );

                    if other.r > 0.02 {
                        // Light cones are frame-invariant: theirs is at 45 degrees too.
                        let other_cone_len = (cone_len * 0.45).max(18.0);
                        let o_fut_out = other_pos + Vec2::new(other_cone_len, -other_cone_len);
                        let o_fut_in = other_pos + Vec2::new(-other_cone_len, -other_cone_len);

                        painter.add(PathShape::convex_polygon(
                            vec![other_pos, o_fut_in, o_fut_out],
                            Color32::from_rgba_premultiplied(other_color.r(), other_color.g(), other_color.b(), 1),
                            egui::epaint::PathStroke::NONE,
                        ));
                        painter.line_segment([other_pos, o_fut_in], Stroke::new(1.2, other_color));
                        painter.line_segment([other_pos, o_fut_out], Stroke::new(1.2, other_color));
                    } else {
                        painter.circle_filled(other_pos, 8.0, Theme::SINGULARITY_FILL);
                        painter.circle_stroke(other_pos, 10.0, Stroke::new(1.5, Theme::SINGULARITY_LINE));
                    }

                    painter.circle_filled(other_pos, 6.0, other_color);
                    painter.circle_stroke(other_pos, 7.5, Stroke::new(1.0, Color32::WHITE));
                    painter.line_segment([center, other_pos], Stroke::new(1.2, Color32::from_white_alpha(120)));

                    let v_rel = (v[1] / v[0].abs().max(1e-12)).abs();
                    painter.text(
                        other_pos + Vec2::new(9.0, 9.0),
                        egui::Align2::LEFT_TOP,
                        format!(
                            "azimuthal offset ξ² = {}\nradial speed in this frame = {:.3}c",
                            metric.format_r(xi[2], use_km),
                            v_rel.min(9.999)
                        ),
                        egui::FontId::monospace(9.0 * font_scale),
                        Theme::TEXT_MUTED,
                    );

                    draw_hovering_telemetry(painter, rect, other_pos, &other.name, other_color, other, metric, use_km, font_scale);
                }
            }
        }

        draw_hovering_telemetry(painter, rect, apex, &focus_obs.name, obs_color, focus_obs, metric, use_km, font_scale);

        painter.text(
            Pos2::new(rect.left() + 8.0, rect.top() + 8.0),
            egui::Align2::LEFT_TOP,
            format!(
                "🔭 {}'S REST FRAME  |  first-order local inertial frame (c ≡ 1, 45° light cones); \
                 surfaces r = const placed by the dual tetrad",
                focus_obs.name.to_uppercase()
            ),
            egui::FontId::proportional(11.0 * font_scale),
            Color32::from_rgb(150, 220, 255),
        );
    }
}

/// Clip the infinite line p + t d to `rect` (Liang-Barsky), returning its visible segment.
fn clip_line_to_rect(p: Pos2, d: Vec2, rect: Rect) -> Option<(Pos2, Pos2)> {
    if d.x.abs() < 1e-12 && d.y.abs() < 1e-12 {
        return None;
    }
    let mut t_min = f32::NEG_INFINITY;
    let mut t_max = f32::INFINITY;
    // Each edge contributes one constraint num * t <= den.
    for &(num, den) in &[
        (-d.x, p.x - rect.left()),
        (d.x, rect.right() - p.x),
        (-d.y, p.y - rect.top()),
        (d.y, rect.bottom() - p.y),
    ] {
        if num.abs() < 1e-12 {
            if den < 0.0 {
                return None;
            }
        } else {
            let t = den / num;
            if num < 0.0 {
                t_min = t_min.max(t);
            } else {
                t_max = t_max.min(t);
            }
        }
    }
    if !(t_min <= t_max) || !t_min.is_finite() || !t_max.is_finite() {
        return None;
    }
    Some((p + d * t_min, p + d * t_max))
}

/// x on the segment a-b at screen height y, clamped to the segment.
fn segment_x_at_y(a: Pos2, b: Pos2, y: f32) -> f32 {
    if (b.y - a.y).abs() < 1e-3 {
        return 0.5 * (a.x + b.x);
    }
    let t = ((y - a.y) / (b.y - a.y)).clamp(0.0, 1.0);
    a.x + t * (b.x - a.x)
}

/// y on the segment a-b at screen abscissa x, clamped to the segment.
fn segment_y_at_x(a: Pos2, b: Pos2, x: f32) -> f32 {
    if (b.x - a.x).abs() < 1e-3 {
        return 0.5 * (a.y + b.y);
    }
    let t = ((x - a.x) / (b.x - a.x)).clamp(0.0, 1.0);
    a.y + t * (b.y - a.y)
}
