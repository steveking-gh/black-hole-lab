use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::local_frame::{LocalFrame, SurfaceCharacter};
use crate::physics::observer::{Observer, ObserverMode};
use crate::physics::wavefront::{SignalField, outgoing_ray_track};
use egui::{epaint::PathShape, Color32, Pos2, Rect, Stroke, Vec2};
use std::collections::HashMap;

/// Plain-language gloss on every number in a telemetry box, shown on hover.
/// Hover tip for the observer info boxes. Written as a plain multi-line literal (lines start at
/// column 0 so no indentation leaks into the text).
pub const TELEMETRY_HOVER_TIP: &str =
"Drag: move the box anywhere on the canvas. Double-click: snap the box back to the observer. Each canvas remembers box positions per observer.

dr/dt — map speed: how fast the dot crosses the (t, r) chart per tick of the chart's shared clock. Far from the hole, this equals what a distant observer would measure. Near the hole, the chart uses a clock that lets infall and light cross the horizon without freezing, so the number only means something relative to the light wedge.

dr/dτ — wristwatch speed: kilometres of radius per second on the observer's own watch. Can exceed c without breaking relativity, because the watch runs slow and the radius undercounts stretched space near the hole. Inside the horizon, radius becomes a countdown, and dr/dτ is how fast it runs.

a_prop — proper acceleration in Earth g, the accelerometer reading. Zero means free fall.

Tidal — gravitational acceleration difference across one metre, in g per metre. Curvature sets the value (48M²/r⁶ on the equator); tidal stretch, not infall speed, tears a body apart.

ν_in/ν_∞ — frequency of ingoing light measured by the observer, divided by the frequency at infinity. Below 1 means redshift; a raindrop measures 1/2 at the Schwarzschild horizon.

E, L — conserved energy and angular momentum per unit mass along the geodesic. E = 1, L = 0 means a drop from rest at infinity.

Region tag — location relative to the horizons: outside r₊, between r₊ and r₋, or inside r₋, plus the ergosphere.";

/// The master outgoing null ray of the interior together with the (M, a) it belongs to: the curve
/// is recomputed only when the hole changes underneath it.
type CachedOutgoingRay = (f64, f64, Vec<(f64, f64)>);

/// Coordinate-time spacing between consecutive members of the drawn outgoing null congruence
/// inside r+, in units of M. The geometry is stationary, so the congruence is one curve repeated at
/// this interval; the spacing is a drawing choice and nothing else depends on it.
///
/// It has to be small against the time a ray spends visibly off r-, which is 1/kappa_-: 0.63M at
/// a = 0.65, and 2.59M at the app's default a = 0.90, whose inner horizon is slacker
/// (kappa_- = 0.386/M). The fast case is the binding one - there a ray is inside one pixel of the
/// r- line within a couple of M of arriving - and at the 4M spacing this started with, at most one
/// ray was ever mid-swing and the bunching that is the whole point of the picture could not be
/// seen. At 1M several rays are in flight at once at either spin, and the exponential crowding onto
/// r- is drawn rather than asserted.
const OUTGOING_RAY_SPACING: f64 = 1.0;

/// Cap on how many members of that congruence are drawn in one frame, so that a very wide time
/// window cannot turn a faint background hatch into thousands of polylines. At the spacing above
/// this covers 400M of coordinate time, more than any window the canvas offers.
const MAX_DRAWN_OUTGOING_RAYS: i64 = 400;

/// The drag offsets of the hovering telemetry boxes on one canvas, keyed by canvas tag and
/// observer name so that the same observer can have a different box position in each diagram.
#[derive(Default)]
pub struct TelemetryBoxes {
    offsets: HashMap<String, Vec2>,
}

impl TelemetryBoxes {
    /// Lay out, register, drag and paint one observer's info box.
    ///
    /// The box is anchored next to `pos` exactly as before, then displaced by the offset the user
    /// has dragged it to and clamped back inside `canvas_rect`. The `ui.interact` must be called
    /// after the canvas has allocated its own painter response: within a layer egui hands an
    /// overlapping drag to the widget registered last, so registering here is what stops a drag on
    /// the box from panning the background or grabbing Bob's marker.
    #[allow(clippy::too_many_arguments)]
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        canvas_tag: &str,
        canvas_rect: Rect,
        pos: Pos2,
        name: &str,
        color: Color32,
        obs: &Observer,
        metric: &KerrSchild,
        use_km: bool,
        font_scale: f32,
    ) -> egui::Response {
        let key = format!("{canvas_tag}:{name}");
        let offset = self.offsets.get(&key).copied().unwrap_or(Vec2::ZERO);
        let lines = telemetry_lines(name, color, obs, metric, use_km);
        let size = telemetry_box_size(painter, &lines, font_scale);

        let anchored = default_badge_pos(canvas_rect, pos, size);
        let badge_rect = clamp_into(Rect::from_min_size(anchored + offset, size), canvas_rect);

        let id = ui.id().with(("telemetry", canvas_tag, name));
        // click_and_drag rather than drag alone: egui only reports a double-click on a widget that
        // senses clicks, and the double-click is what resets the offset.
        let response = ui.interact(badge_rect, id, egui::Sense::click_and_drag());

        let badge_rect = if response.double_clicked() {
            self.offsets.remove(&key);
            clamp_into(Rect::from_min_size(anchored, size), canvas_rect)
        } else {
            let moved = clamp_into(badge_rect.translate(response.drag_delta()), canvas_rect);
            // Store where the box actually ended up, so a drag against the canvas edge does not
            // build up an offset that snaps back later.
            self.offsets.insert(key, moved.min - anchored);
            moved
        };

        paint_telemetry_box(painter, badge_rect, color, &lines, font_scale);
        response.on_hover_text(TELEMETRY_HOVER_TIP)
    }
}

/// One printed line of a telemetry box: the text, its colour and whether it is the title.
struct TelemetryLine {
    text: String,
    color: Color32,
    is_title: bool,
}

/// The box position the anchor asks for, before the user's drag offset is added.
fn default_badge_pos(canvas_rect: Rect, pos: Pos2, size: Vec2) -> Pos2 {
    let bx = (pos.x + 12.0).min(canvas_rect.right() - size.x - 6.0).max(canvas_rect.left() + 6.0);
    let by = (pos.y - size.y - 6.0)
        .max(canvas_rect.top() + 6.0)
        .min(canvas_rect.bottom() - size.y - 6.0);
    Pos2::new(bx, by)
}

/// Keep `rect` inside `bounds` with a 6 px margin, sliding rather than shrinking it.
fn clamp_into(rect: Rect, bounds: Rect) -> Rect {
    let max_x = (bounds.right() - rect.width() - 6.0).max(bounds.left() + 6.0);
    let max_y = (bounds.bottom() - rect.height() - 6.0).max(bounds.top() + 6.0);
    Rect::from_min_size(
        Pos2::new(
            rect.left().clamp(bounds.left() + 6.0, max_x),
            rect.top().clamp(bounds.top() + 6.0, max_y),
        ),
        rect.size(),
    )
}

fn telemetry_fonts(font_scale: f32) -> (egui::FontId, egui::FontId) {
    (
        egui::FontId::monospace(10.0 * font_scale),
        egui::FontId::monospace(9.0 * font_scale),
    )
}

/// Width fitted to the longest line, height to the actual number of lines.
fn telemetry_box_size(painter: &egui::Painter, lines: &[TelemetryLine], font_scale: f32) -> Vec2 {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let (font_title, font_body) = telemetry_fonts(font_scale);
    let max_text_w = lines
        .iter()
        .map(|line| {
            let font = if line.is_title { font_title.clone() } else { font_body.clone() };
            painter.layout_no_wrap(line.text.clone(), font, line.color).size().x
        })
        .fold(0.0_f32, f32::max);

    let pad_x = 10.0 * font_scale;
    let pad_y = 6.0 * font_scale;
    let line_spacing = 13.0 * font_scale;
    Vec2::new(
        (max_text_w + pad_x * 2.0).max(180.0 * font_scale),
        (pad_y * 2.0 + line_spacing * (lines.len() as f32 - 0.2)).max(56.0 * font_scale),
    )
}

fn paint_telemetry_box(
    painter: &egui::Painter,
    badge_rect: Rect,
    color: Color32,
    lines: &[TelemetryLine],
    font_scale: f32,
) {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let (font_title, font_body) = telemetry_fonts(font_scale);
    let pad_x = 10.0 * font_scale;
    let pad_y = 6.0 * font_scale;
    let line_spacing = 13.0 * font_scale;

    painter.rect_filled(badge_rect, 4.0 * font_scale, Color32::from_black_alpha(230));
    painter.rect_stroke(badge_rect, 4.0 * font_scale, Stroke::new(1.2, color), egui::StrokeKind::Inside);

    for (i, line) in lines.iter().enumerate() {
        let font = if line.is_title { font_title.clone() } else { font_body.clone() };
        painter.text(
            Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing * i as f32),
            egui::Align2::LEFT_TOP,
            line.text.clone(),
            font,
            line.color,
        );
    }
}

/// The box's contents, one metric per line.
fn telemetry_lines(
    name: &str,
    color: Color32,
    obs: &Observer,
    metric: &KerrSchild,
    use_km: bool,
) -> Vec<TelemetryLine> {
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

    // One metric per line: the coordinate velocity and the proper velocity are different
    // statements about the infall and no longer share a row.
    let v_coord_str = if use_km {
        format!("dr/dt   = {:+.0} km/s ({:+.2}c)", v_kms, v_c)
    } else {
        format!("dr/dt   = {:+.2}c", v_c)
    };
    let v_proper_str = format!("dr/dτ   = {:+.2}c", u_prop);

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

    let mut lines = vec![
        TelemetryLine { text: format!("{} [{}]", name, region_tag), color, is_title: true },
        TelemetryLine { text: v_coord_str, color: Theme::TEXT_BRIGHT, is_title: false },
        TelemetryLine { text: v_proper_str, color: Theme::TEXT_BRIGHT, is_title: false },
        TelemetryLine { text: a_str, color: Color32::from_rgb(180, 240, 180), is_title: false },
        TelemetryLine { text: tidal_str, color: Color32::from_rgb(255, 200, 100), is_title: false },
        TelemetryLine {
            text: nu_str,
            color: if nu_ratio > 1.0 { Theme::BLUESHIFT_BLUE } else { Theme::TEXT_MUTED },
            is_title: false,
        },
    ];
    if let Some(constants_str) = constants_str {
        lines.push(TelemetryLine { text: constants_str, color: Theme::TEXT_MUTED, is_title: false });
    }
    lines
}

pub struct SpacetimeCanvas {
    pub max_r: f64,
    pub r_offset: f64,
    pub time_window: f64,
    pub time_offset: f64,
    pub is_dragging_bob: bool,
    /// Bob's mode when a marker drag began, restored when the drag ends so that a free-falling
    /// Bob resumes free fall from the dropped event rather than staying in manual mode.
    bob_mode_before_drag: Option<ObserverMode>,
    /// Where the user has dragged each info box on this canvas, per diagram and per observer.
    pub telemetry: TelemetryBoxes,
    /// The master outgoing principal null ray of the interior, cached against the (M, a) it was
    /// integrated for. The geometry is stationary, so every other member of that congruence is this
    /// one curve translated in t, and the integration is repeated only when the hole changes.
    outgoing_rays: Option<CachedOutgoingRay>,
}

impl Default for SpacetimeCanvas {
    fn default() -> Self {
        Self {
            max_r: 5.5,
            r_offset: 0.0,
            time_window: 14.0,
            time_offset: 0.0,
            is_dragging_bob: false,
            bob_mode_before_drag: None,
            telemetry: TelemetryBoxes::default(),
            outgoing_rays: None,
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

    /// Recompute the cached master outgoing null ray when the hole has changed.
    ///
    /// The curve depends on nothing but (M, a): it is the solution of dr/dt = Delta / (r^2 + a^2 +
    /// 2Mr) started just inside r+, and the geometry is stationary, so translating it in t sweeps
    /// out the whole outgoing congruence of the interior. See `wavefront::outgoing_ray_track`.
    fn ensure_outgoing_rays(&mut self, metric: &KerrSchild) {
        let stale = match &self.outgoing_rays {
            Some((m, a, _)) => *m != metric.m || *a != metric.a,
            None => true,
        };
        if stale {
            self.outgoing_rays = Some((metric.m, metric.a, outgoing_ray_track(metric)));
        }
    }

    pub fn focus_bob(&mut self, bob_r: f64) {
        self.max_r = 0.05;
        self.r_offset = (bob_r - 0.025).max(0.0);
    }

    /// Render the (t, r) spacetime foliation canvas with an integrated, perfectly aligned 1D radial track
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &mut Observer,
        alice: &Option<Observer>,
        current_time: f64,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
        signals: SignalViews<'_>,
        show_outgoing_rays: bool,
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
                self.render_observer_frame(ui, &painter, rect, metric, bob, alice.as_ref(), use_km, font_scale);
            }
            ReferenceFrame::Alice => {
                painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
                if let Some(al) = alice {
                    self.render_observer_frame(ui, &painter, rect, metric, al, Some(bob), use_km, font_scale);
                } else {
                    self.render_observer_frame(ui, &painter, rect, metric, bob, None, use_km, font_scale);
                }
            }
            ReferenceFrame::DistantObserver => {
                self.render_distant_observer(
                    ui,
                    &painter,
                    &response,
                    rect,
                    metric,
                    bob,
                    alice,
                    current_time,
                    use_km,
                    font_scale,
                    signals,
                    show_outgoing_rays,
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

    #[allow(clippy::too_many_arguments)]
    fn render_distant_observer(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        response: &egui::Response,
        rect: Rect,
        metric: &KerrSchild,
        bob: &mut Observer,
        alice: &Option<Observer>,
        current_time: f64,
        use_km: bool,
        font_scale: f32,
        signals: SignalViews<'_>,
        show_outgoing_rays: bool,
    ) {
        // Done before the screen-mapping closures below take their borrow of self.
        if show_outgoing_rays {
            self.ensure_outgoing_rays(metric);
        }

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

        // Both observers' cones span the same slice of the zoom window.
        let cone_span = (self.time_window * 0.12).min(self.max_r * 1.5).clamp(1e-4, 1.8);

        // One exact light cone in the (t, r) chart, drawn in the colours that belong to `obs`
        // rather than to the diagram, so a cone is identifiable in any frame.
        let draw_cone = |obs: &Observer| {
            let (future_fill, past_fill, edge) = Theme::cone_colours(&obs.name);
            let cone = obs.compute_lightcone_polygon(metric, cone_span);
            let cone_apex = Pos2::new(to_screen_x(cone.apex[1]), to_screen_y(cone.apex[0]));
            let p_fut_in = Pos2::new(to_screen_x(cone.future_in[1]), to_screen_y(cone.future_in[0]));
            let p_fut_out = Pos2::new(to_screen_x(cone.future_out[1]), to_screen_y(cone.future_out[0]));
            let p_past_in = Pos2::new(to_screen_x(cone.past_in[1]), to_screen_y(cone.past_in[0]));
            let p_past_out = Pos2::new(to_screen_x(cone.past_out[1]), to_screen_y(cone.past_out[0]));

            painter.add(PathShape::convex_polygon(
                vec![cone_apex, p_fut_in, p_fut_out],
                future_fill,
                egui::epaint::PathStroke::NONE,
            ));
            painter.add(PathShape::convex_polygon(
                vec![cone_apex, p_past_in, p_past_out],
                past_fill,
                egui::epaint::PathStroke::NONE,
            ));

            // Collinear rays:
            // Ingoing ray: p_past_in (past-right) -> apex -> p_fut_in (future-left), slope dr/dt = -1
            painter.line_segment([cone_apex, p_fut_in], Stroke::new(1.8, edge));
            painter.line_segment([p_past_in, cone_apex], Stroke::new(1.2, edge));
            // Outgoing ray: p_past_out -> apex -> p_fut_out
            painter.line_segment([cone_apex, p_fut_out], Stroke::new(2.2, edge));
            painter.line_segment([p_past_out, cone_apex], Stroke::new(1.2, edge));
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
                    Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
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
                            Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
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
                            Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
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

        // Outgoing light trapped inside r+: the congruence of outgoing principal null rays.
        //
        // Each of these lines peels off r+, falls inward because Region II is trapped, and then
        // asymptotes to r- from above without ever crossing it. They are one master curve
        // translated in t, `wavefront::outgoing_ray_track`, which starts at r+(1 - 1e-3) rather
        // than on r+ itself: the vertical run along the horizon is implicit, and drawing it would
        // only lay a second line on top of the r+ line. The offset r - r- decays like
        // exp(-kappa_- t), so the lines crowd exponentially onto r- and the last stretch of every
        // one of them is inside a pixel of it; they carry their own colour for that reason, so the
        // pile-up stays legible against the r- line itself. The Cauchy horizon in this chart is
        // therefore the accumulation surface of the interior's outgoing null congruence, and an
        // infalling worldline, which crosses r- at finite proper time, cuts through the whole stack
        // on its way. That is the geometry behind Bob's reception of Alice's entire transmission in
        // one moment. A hole with r- at the origin (no spin, or so little that r- is numerically
        // indistinguishable from the ring) has no such surface, and nothing is drawn.
        if show_outgoing_rays
            && rm >= 1e-3
            && let Some((_, _, track)) = self.outgoing_rays.as_ref()
            && let Some(&(track_end, _)) = track.last()
        {
            let k_lo = ((t_min - track_end) / OUTGOING_RAY_SPACING).ceil() as i64;
            let k_hi = (t_max / OUTGOING_RAY_SPACING).floor() as i64;
            for k in k_lo..=k_hi.min(k_lo + MAX_DRAWN_OUTGOING_RAYS) {
                let t0 = (k as f64) * OUTGOING_RAY_SPACING;
                let points: Vec<Pos2> = track
                    .iter()
                    .filter(|(t, _)| t + t0 >= t_min && t + t0 <= t_max)
                    .map(|&(t, r)| Pos2::new(to_screen_x(r), to_screen_y(t + t0)))
                    .collect();
                if points.len() >= 2 {
                    painter.add(PathShape::line(points, Stroke::new(1.0, Theme::OUTGOING_RAY)));
                }
            }
        }

        // The two transmissions. A wavefront is a closed curve in (r, phi) and this diagram has
        // no azimuth to draw it on, so what is drawn is the one thing the projection does define:
        // the pulse's radial extent, [min r, max r] over its live rays, swept up in t. That is the
        // wedge of `Pulse::extent_track`. Its lower edge is the ingoing edge of the emitter's own
        // light cone, carried from the emission event - the 45-degree line dr/dt = -1 only for a
        // hole with no spin, and slightly steeper than that for one that spins (-1.010 at r = 4.5M
        // for a = 0.65, -2.27 at r = 0.2M for a = 0.90) - and its upper edge is the outermost ray,
        // which outside r+ climbs and inside r+ falls and freezes onto r-. A worldline inside a
        // wedge is *in range* of that pulse - some ray of it stands at that radius - which is not
        // the same as receiving it, because the diagram cannot show azimuth and the receiver may be
        // at another one. The reception dots below are the actual arrivals.
        //
        // Each field is drawn in its emitter's colour: the interior in `Theme::WEDGE_FILL_ALPHA`,
        // faint enough that the forty-odd wedges of a whole infall stack up without flattening into
        // a block, and the two edges as thin lines at `Theme::WEDGE_EDGE_ALPHA`. Inside r+ every
        // upper edge freezes on r-, so those edges pile onto the Cauchy horizon exactly as the pink
        // congruence does, and that pile is what a later infaller cuts through.
        //
        // The fill is laid down as a strip of quads between consecutive track points rather than as
        // one polygon: the wedge is not convex in general - the upper edge bends back onto r- while
        // the lower edge runs on to the ring - and a quad spanning two adjacent times, with its two
        // horizontal sides, always is.
        let draw_wedges = |field: &SignalField, colour: Color32| {
            let shade = |alpha: u8| {
                Color32::from_rgba_unmultiplied(colour.r(), colour.g(), colour.b(), alpha)
            };
            let fill = shade(Theme::WEDGE_FILL_ALPHA);
            let edge_stroke = Stroke::new(1.0, shade(Theme::WEDGE_EDGE_ALPHA));
            for pulse in field.pulses.iter() {
                let visible: Vec<(f64, f64, f64)> = pulse
                    .extent_track
                    .iter()
                    .copied()
                    .filter(|(t, _, _)| *t >= t_min && *t <= t_max)
                    .collect();
                if visible.len() < 2 {
                    continue;
                }
                for pair in visible.windows(2) {
                    let (t0, lo0, hi0) = pair[0];
                    let (t1, lo1, hi1) = pair[1];
                    let (y0, y1) = (to_screen_y(t0), to_screen_y(t1));
                    painter.add(PathShape::convex_polygon(
                        vec![
                            Pos2::new(to_screen_x(lo0), y0),
                            Pos2::new(to_screen_x(hi0), y0),
                            Pos2::new(to_screen_x(hi1), y1),
                            Pos2::new(to_screen_x(lo1), y1),
                        ],
                        fill,
                        egui::epaint::PathStroke::NONE,
                    ));
                }
                for edge in [
                    visible.iter().map(|&(t, lo, _)| Pos2::new(to_screen_x(lo), to_screen_y(t))).collect::<Vec<_>>(),
                    visible.iter().map(|&(t, _, hi)| Pos2::new(to_screen_x(hi), to_screen_y(t))).collect::<Vec<_>>(),
                ] {
                    painter.add(PathShape::line(edge, edge_stroke));
                }
            }
        };
        if signals.show_bob {
            draw_wedges(signals.bob, Theme::BOB_COLOR);
        }
        if signals.show_alice {
            draw_wedges(signals.alice, Theme::ALICE_COLOR);
        }

        // Alice Worldline & Marker. The info boxes are registered last, below, so that a drag on
        // a box beats the canvas's own pan/drag response instead of panning time or moving Bob.
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice {
            if al.trail.len() >= 2 {
                let points: Vec<Pos2> = al
                    .trail
                    .iter()
                    .map(|point| Pos2::new(to_screen_x(point.r), to_screen_y(point.t)))
                    .collect();
                painter.add(PathShape::line(points, Stroke::new(2.0, Theme::ALICE_COLOR)));
            }

            if al.is_active {
                let alice_pos = Pos2::new(to_screen_x(al.r), to_screen_y(al.t));
                if al.r > 0.02 {
                    draw_cone(al);
                }
                if rect.contains(alice_pos) {
                    painter.circle_filled(alice_pos, 5.5, Theme::ALICE_COLOR);
                    painter.circle_stroke(alice_pos, 7.5, Stroke::new(1.0, Color32::WHITE));
                    alice_box = Some(alice_pos);
                }
            }
        }

        // Bob Worldline & Dragging
        if bob.trail.len() >= 2 {
            let points: Vec<Pos2> = bob
                .trail
                .iter()
                .map(|point| Pos2::new(to_screen_x(point.r), to_screen_y(point.t)))
                .collect();
            painter.add(PathShape::line(points, Stroke::new(2.5, Theme::BOB_COLOR)));
        }

        // Every arrival either observer has recorded, marked on the receiver's worldline at the
        // event of reception and coloured by the shift they measured: red where the signal arrives
        // redshifted, violet where crossing the stack on r- has multiplied its frequency a
        // thousandfold. One of Alice's pulses appears more than once on Bob's worldline, since its
        // crossing sheet sweeps past him well above r- and its frozen sheet waits on r- for him to
        // fall through it. Bob's pulses reach Alice once each and then stop reaching her at all.
        let draw_receptions = |field: &SignalField| {
            for reception in field.receptions() {
                if reception.t < t_min || reception.t > t_max {
                    continue;
                }
                let at = Pos2::new(to_screen_x(reception.r), to_screen_y(reception.t));
                if rect.contains(at) {
                    painter.circle_filled(at, 3.0, Theme::shift_colour(reception.ratio, 255));
                }
            }
        };
        if signals.show_alice {
            draw_receptions(signals.alice);
        }
        if signals.show_bob {
            draw_receptions(signals.bob);
            // The emission event of the last pulse of Bob's that ever reached Alice, ringed on his
            // worldline once hers has ended. Beyond that event his light cone no longer contains
            // any of her worldline, so nothing he sends arrives; the simulation is what decides
            // that, and the ring appears only once it has. He transmits through his wait as well as
            // through his fall, so the ring can land on the vertical hover segment of his
            // worldline, and at the app's default delay it does: everything below the ring on that
            // segment reached her, everything above it, release and infall included, did not.
            if alice.as_ref().is_some_and(|al| al.has_ended())
                && let Some(pulse) = signals.bob.last_delivered_pulse()
            {
                let at = Pos2::new(to_screen_x(pulse.emitted_r), to_screen_y(pulse.emitted_t));
                if rect.contains(at) {
                    painter.circle_stroke(at, 5.0, Stroke::new(2.0, Theme::BOB_COLOR));
                    painter.circle_stroke(at, 7.0, Stroke::new(1.0, Color32::WHITE));
                }
            }
        }

        let bob_pos = Pos2::new(to_screen_x(bob.r), to_screen_y(bob.t));
        let bob_radius = 8.0;

        if response.drag_started() {
            if let Some(mouse_pos) = response.interact_pointer_pos() {
                if mouse_pos.distance(bob_pos) < bob_radius * 3.0 {
                    self.is_dragging_bob = true;
                    self.bob_mode_before_drag = Some(bob.mode);
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
            if self.is_dragging_bob {
                // Resume the worldline from the dropped event in whatever mode Bob had before the
                // drag; an explicit Manual selection stays manual.
                let mode = self.bob_mode_before_drag.take().unwrap_or(ObserverMode::FreeFall);
                bob.release_from_drag(metric, mode);
            }
            self.is_dragging_bob = false;
        }

        // Bob's Exact Light Cone
        let apex = Pos2::new(to_screen_x(bob.r), to_screen_y(bob.t));

        if bob.r > 0.02 && bob.is_active {
            draw_cone(bob);
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

        // Draggable info boxes, registered after every other interaction on this canvas.
        if let (Some(al), Some(alice_pos)) = (alice.as_ref(), alice_box) {
            self.telemetry.show(
                ui, painter, "spacetime", rect, alice_pos, "Alice", Theme::ALICE_COLOR, al, metric, use_km,
                font_scale,
            );
        }
        self.telemetry.show(
            ui, painter, "spacetime", rect, apex, "Bob", Theme::BOB_COLOR, bob, metric, use_km, font_scale,
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
    #[allow(clippy::too_many_arguments)]
    fn render_observer_frame(
        &mut self,
        ui: &mut egui::Ui,
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
        let grid_stroke = Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE);
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
        let (focus_future_fill, focus_past_fill, focus_edge) = Theme::cone_colours(&focus_obs.name);

        if focus_obs.r > 0.02 && focus_obs.is_active {
            let p_fut_out = apex + Vec2::new(cone_len, -cone_len);
            let p_fut_in = apex + Vec2::new(-cone_len, -cone_len);
            let p_past_out = apex + Vec2::new(cone_len, cone_len);
            let p_past_in = apex + Vec2::new(-cone_len, cone_len);

            painter.add(PathShape::convex_polygon(
                vec![apex, p_fut_in, p_fut_out],
                focus_future_fill,
                egui::epaint::PathStroke::NONE,
            ));
            painter.add(PathShape::convex_polygon(
                vec![apex, p_past_out, p_past_in],
                focus_past_fill,
                egui::epaint::PathStroke::NONE,
            ));

            painter.line_segment([apex, p_fut_in], Stroke::new(2.2, focus_edge));
            painter.line_segment([p_past_out, apex], Stroke::new(1.2, focus_edge));
            painter.line_segment([apex, p_fut_out], Stroke::new(2.2, focus_edge));
            painter.line_segment([p_past_in, apex], Stroke::new(1.2, focus_edge));

            painter.text(
                p_fut_out + Vec2::new(4.0, -2.0),
                egui::Align2::LEFT_BOTTOM,
                "+45° Outgoing",
                egui::FontId::monospace(9.0 * font_scale),
                focus_edge,
            );
            painter.text(
                p_fut_in + Vec2::new(-4.0, -2.0),
                egui::Align2::RIGHT_BOTTOM,
                "-45° Ingoing",
                egui::FontId::monospace(9.0 * font_scale),
                focus_edge,
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
        let mut other_box: Option<(&Observer, Pos2, Color32)> = None;
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

                        let (other_future_fill, _, other_edge) = Theme::cone_colours(&other.name);
                        painter.add(PathShape::convex_polygon(
                            vec![other_pos, o_fut_in, o_fut_out],
                            other_future_fill,
                            egui::epaint::PathStroke::NONE,
                        ));
                        painter.line_segment([other_pos, o_fut_in], Stroke::new(1.2, other_edge));
                        painter.line_segment([other_pos, o_fut_out], Stroke::new(1.2, other_edge));
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

                    other_box = Some((other, other_pos, other_color));
                }
            }
        }

        // The info boxes go last so that dragging one wins over the canvas's own drag response.
        if let Some((other, other_pos, other_color)) = other_box {
            self.telemetry.show(
                ui, painter, "restframe", rect, other_pos, &other.name, other_color, other, metric, use_km,
                font_scale,
            );
        }
        self.telemetry.show(
            ui, painter, "restframe", rect, apex, &focus_obs.name, obs_color, focus_obs, metric, use_km,
            font_scale,
        );

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

#[cfg(test)]
mod tests {
    use super::*;

    /// One egui pass over a canvas-sized drag surface with a telemetry-sized box registered on
    /// top of it, in the same order the canvases use. Returns (canvas dragged, box dragged).
    fn drag_precedence_pass(
        ctx: &egui::Context,
        screen: Rect,
        box_rect: &mut Option<Rect>,
        events: Vec<egui::Event>,
    ) -> (bool, bool) {
        let mut dragged = (false, false);
        let input = egui::RawInput {
            screen_rect: Some(screen),
            events,
            ..Default::default()
        };
        let mut output = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                // The canvas allocates its own drag response first, exactly as `render` does.
                let (canvas, _painter) =
                    ui.allocate_painter(Vec2::new(360.0, 260.0), egui::Sense::drag());
                let rect = *box_rect.get_or_insert_with(|| {
                    Rect::from_min_size(canvas.rect.min + Vec2::new(40.0, 40.0), Vec2::new(120.0, 60.0))
                });
                // ... and the info box is registered afterwards, so it is on top.
                let badge = ui.interact(rect, ui.id().with("telemetry"), egui::Sense::click_and_drag());
                dragged = (canvas.dragged(), badge.dragged());
            });
        });
        // No renderer here to upload the font atlas to, so drop the texture delta on purpose
        // (epaint panics in debug builds if it is dropped unhandled).
        output.textures_delta.clear();
        dragged
    }

    fn press(pos: Pos2) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::default(),
        }
    }

    /// A drag that starts on an info box must go to the box, and a drag that starts on the bare
    /// canvas must still go to the canvas (which is what pans time and moves Bob). egui resolves
    /// the overlap by registration order within the layer, last one wins.
    #[test]
    fn info_box_takes_the_drag_and_the_background_keeps_it() {
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));

        for on_the_box in [true, false] {
            let ctx = egui::Context::default();
            let mut box_rect = None;
            // First pass only registers the widgets, which is what the hit test reads.
            drag_precedence_pass(&ctx, screen, &mut box_rect, vec![]);
            let rect = box_rect.expect("the box is laid out on the first pass");
            let start = if on_the_box {
                rect.center()
            } else {
                Pos2::new(rect.right() + 60.0, rect.bottom() + 60.0)
            };

            drag_precedence_pass(&ctx, screen, &mut box_rect, vec![egui::Event::PointerMoved(start)]);
            drag_precedence_pass(&ctx, screen, &mut box_rect, vec![press(start)]);
            let (canvas_dragged, box_dragged) = drag_precedence_pass(
                &ctx,
                screen,
                &mut box_rect,
                vec![egui::Event::PointerMoved(start + Vec2::new(45.0, 35.0))],
            );

            assert_eq!(
                (canvas_dragged, box_dragged),
                (!on_the_box, on_the_box),
                "drag started {} the box",
                if on_the_box { "on" } else { "off" }
            );
        }
    }

    /// The box grows a line at a time and stays wide enough for its longest line.
    #[test]
    fn telemetry_box_is_sized_from_the_line_count() {
        let line = |text: &str| TelemetryLine {
            text: text.to_string(),
            color: Color32::WHITE,
            is_title: false,
        };

        let six = vec![line("a"), line("b"), line("c"), line("d"), line("e"), line("f")];
        let seven = {
            let mut v = six.iter().map(|l| line(&l.text)).collect::<Vec<_>>();
            v.push(line("E = 1.000  L = 0.000 M"));
            v
        };
        // Fonts only exist inside a running context, so measure inside one.
        egui::__run_test_ui(|ui| {
            let painter = ui.painter();
            let h6 = telemetry_box_size(painter, &six, 1.0).y;
            let h7 = telemetry_box_size(painter, &seven, 1.0).y;
            assert!((h7 - h6 - 13.0).abs() < 1e-3, "one extra line is one extra line height");

            // The width never drops below the minimum badge width, whatever the line count.
            assert!(telemetry_box_size(painter, &six, 1.0).x >= 180.0);
        });
    }

    /// A box dragged past the edge is slid back inside the canvas rather than clipped.
    #[test]
    fn dragged_box_is_clamped_into_the_canvas() {
        let bounds = Rect::from_min_size(Pos2::new(10.0, 20.0), Vec2::new(400.0, 300.0));
        let size = Vec2::new(180.0, 90.0);
        let far_out = Rect::from_min_size(Pos2::new(9000.0, -9000.0), size);
        let clamped = clamp_into(far_out, bounds);
        assert_eq!(clamped.size(), size);
        assert!(bounds.contains_rect(clamped), "{clamped:?} outside {bounds:?}");
    }
}
