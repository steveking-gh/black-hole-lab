use crate::gui::controls::{impossible_mode_note, ReferenceFrame, SignalViews};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::local_frame::{LocalFrame, SurfaceCharacter};
use crate::physics::observer::{Observer, ObserverMode};
use crate::physics::wavefront::SignalField;
use egui::{epaint::PathShape, Color32, Pos2, Rect, Stroke, Vec2};
use std::collections::HashMap;

/// Plain-language gloss on every number in a telemetry box, shown on hover.
/// Hover tip for the observer info boxes. Written as a plain multi-line literal (lines start at
/// column 0 so no indentation leaks into the text).
pub const TELEMETRY_HOVER_TIP: &str =
"Drag: move the box anywhere on the canvas. On the equatorial view it then stays where you put it while the observer moves on; on the (t, r) diagram it keeps its offset from the observer. Double-click: snap the box back to the observer. Each canvas remembers box positions per observer.

dr/dt — map speed: how fast the dot crosses the (t, r) chart per tick of the chart's shared clock. Far from the hole, this equals what a distant observer would measure. Near the hole, the chart uses a clock that lets infall and light cross the horizon without freezing, so the number only means something relative to the light wedge.

dr/dτ — wristwatch speed: kilometres of radius per second on the observer's own watch. Can exceed c without breaking relativity, because the watch runs slow and the radius undercounts stretched space near the hole. Inside the horizon, radius becomes a countdown, and dr/dτ is how fast it runs.

a_prop — proper acceleration in Earth g, the accelerometer reading. Zero means free fall.

Tidal — gravitational acceleration difference across one metre, in g per metre. Curvature sets the value (48M²/r⁶ on the equator); tidal stretch, not infall speed, tears a body apart.

ν_in/ν_∞ — frequency of ingoing light measured by the observer, divided by the frequency at infinity. Below 1 means redshift; a raindrop measures 1/2 at the Schwarzschild horizon.

E, L — conserved energy and angular momentum per unit mass along the geodesic. E = 1, L = 0 means a drop from rest at infinity.

Region tag — location relative to the horizons: outside r₊, between r₊ and r₋, or inside r₋, plus the ergosphere.";

/// Seconds in a Julian year, the unit the top of the distant clock grid's ladder is counted in.
const SECONDS_PER_YEAR: f64 = 86400.0 * 365.25;

/// Smallest on-screen gap, in points at `font_scale` = 1, that `distant_clock_grid_step` will
/// leave between two neighbouring lines of the distant clock grid. Below this the lines stop being
/// readable as separate slices and start being a smear, so the ladder is climbed instead.
pub const MIN_GRID_PX: f32 = 28.0;

/// Smallest vertical gap, in points at `font_scale` = 1, between two *labelled* lines of that grid.
/// A 9-point monospace row is about 12 points tall; this leaves a little air around it. It only
/// ever bites at the very top of the ladder, where no rung is coarse enough to reach `MIN_GRID_PX`
/// and the lines are squeezed anyway.
const CLOCK_LABEL_MIN_PX: f32 = 16.0;

/// The rungs of the ladder below a year, as (seconds in the unit, the multiples of it that are
/// used, the unit's name). Within each unit the 1-2-5 pattern is carried on up through the decades
/// until the next unit takes over, so the ladder has no holes in it: no two neighbouring rungs are
/// more than a factor of 2.5 apart, and the step actually chosen is therefore never more than that
/// much coarser than the smallest readable one. (1, 2, 5 once per unit would leave gaps of 200
/// between 5 µs and 1 ms and of 73 between 5 days and a year, and in those gaps a grid asked for a
/// line every 36 µs would get one every 1 ms - two hundred times too coarse, which on a canvas a
/// few hundred points tall is one line and no grid at all.)
///
/// Microseconds and milliseconds are at the bottom because for a 10 solar-mass hole one M of
/// coordinate time is 49 microseconds, and outside the hole u^t is of order 1, so the grid an
/// exterior observer wants is measured in tens of microseconds.
const CLOCK_UNITS: [(f64, &[f64], &str); 6] = [
    (1e-6, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0], "µs"),
    (1e-3, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0], "ms"),
    (1.0, &[1.0, 2.0, 5.0, 10.0, 20.0, 30.0], "s"),
    (60.0, &[1.0, 2.0, 5.0, 10.0, 20.0, 30.0], "min"),
    (3600.0, &[1.0, 2.0, 5.0, 10.0, 12.0], "hr"),
    (86400.0, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0], "day"),
];

/// How many decades of years the ladder carries on for above one year. 1e30 years at a hundred
/// points per M is past any u^t a f64 worldline integrator can reach.
const CLOCK_YEAR_DECADES: i32 = 30;

/// The whole ladder, in ascending order, as (seconds, the step named in its own unit). Years run on
/// in the same 1-2-5 pattern decade after decade, which is where the powers of ten come from:
/// 1 yr, 2 yr, 5 yr, 10 yr, ..., 1e6 yr, 2e6 yr, and so on.
fn distant_clock_ladder() -> Vec<(f64, String)> {
    let name = |multiple: f64, unit: &str| {
        if multiple < 1e4 {
            format!("{multiple:.0} {unit}")
        } else {
            format!("{multiple:.0e} {unit}")
        }
    };
    let mut rungs = Vec::new();
    for (unit_seconds, multiples, unit) in CLOCK_UNITS {
        for &multiple in multiples {
            rungs.push((multiple * unit_seconds, name(multiple, unit)));
        }
    }
    for decade in 0..=CLOCK_YEAR_DECADES {
        for mantissa in [1.0, 2.0, 5.0] {
            let years = mantissa * 10.0_f64.powi(decade);
            rungs.push((years * SECONDS_PER_YEAR, name(years, "yr")));
        }
    }
    rungs
}

/// One rung of the distant clock grid: how much of the chart's Killing time t separates two
/// neighbouring lines, and the name of the unit that step is a round number of.
#[derive(Debug, Clone, PartialEq)]
pub struct GridStep {
    /// The step in units of M of coordinate time t, which is what `LocalFrame::surface_t_const`
    /// takes. Lines are drawn at t = t_obs + k * step_m for integer k.
    pub step_m: f64,
    /// The step named in its own unit, e.g. "5 min", "1 ms", "1e6 yr".
    pub label: String,
}

/// The coarsest-but-one rung of the ladder 1, 2, 5 x {us, ms, s, min, hr, day, yr}, then decades of
/// years, that still leaves at least `MIN_GRID_PX` * `font_scale` points between neighbouring lines
/// of the distant clock grid - that is, the smallest step that is readable.
///
/// The lines are the surfaces t = const of the chart's Killing time, and
/// `LocalFrame::surface_t_const` puts consecutive ones, `step_m` apart in t, exactly
/// `step_m / u^t` of the observer's proper time apart on their worldline. The drawn plane carries
/// `px_per_m` points per M of xi, so the on-screen gap is `step_m / u_t * px_per_m` and the rung
/// has to satisfy `step_m >= MIN_GRID_PX * font_scale * u_t / px_per_m`.
///
/// The choice is made from u^t and the pixel scale alone: nothing about where the observer is, or
/// what the hole is doing, enters except through u^t. That is deliberate. As an observer falls
/// toward the far branch of r- their u^t grows like exp(kappa_- t), so the distant clock runs away
/// on their screen; the grid answers by climbing the ladder - milliseconds, seconds, minutes,
/// years, then decades of years - and stays readable the whole way down instead of collapsing into
/// a solid block. `seconds_per_m` converts the hole's own time unit M into seconds (for a 10 solar
/// mass hole, 4.9e-5 s), so the same ladder serves a stellar-mass hole and a quasar.
pub fn distant_clock_grid_step(
    u_t: f64,
    px_per_m: f32,
    seconds_per_m: f64,
    font_scale: f32,
) -> GridStep {
    // u^t is positive along every future-directed timelike worldline in the ingoing chart; a
    // non-finite one can only come from a broken caller, and is answered with the top of the
    // ladder rather than a panic.
    let u_t = if u_t.is_finite() { u_t.max(1e-12) } else { f64::INFINITY };
    let px_per_m = if px_per_m.is_finite() {
        (px_per_m as f64).max(1e-9)
    } else {
        1e-9
    };
    let seconds_per_m = if seconds_per_m.is_finite() && seconds_per_m > 0.0 {
        seconds_per_m
    } else {
        1.0
    };
    let min_px = (MIN_GRID_PX * font_scale.max(0.05)) as f64;
    // The smallest readable step, first in M of coordinate time and then in seconds of the
    // distant clock, which is the ladder's own currency.
    let needed_seconds = min_px * u_t / px_per_m * seconds_per_m;

    let ladder = distant_clock_ladder();
    let (seconds, label) = ladder
        .iter()
        .find(|(seconds, _)| *seconds >= needed_seconds)
        // Off the top of the ladder: the coarsest rung there is, with the lines closer together
        // than `MIN_GRID_PX`. The labelling guard in the view thins the labels out in that case.
        .unwrap_or_else(|| ladder.last().expect("the ladder is never empty"));
    GridStep {
        step_m: seconds / seconds_per_m,
        label: label.clone(),
    }
}

/// A compact, signed reading of the distant clock `seconds` away from the observer's now: the label
/// on one line of the grid. "+5 min", "-2 h", "+1e6 yr", and "now" for the slice through the
/// observer's own event.
fn distant_clock_offset_label(seconds: f64) -> String {
    if seconds == 0.0 || !seconds.is_finite() {
        return "now".to_string();
    }
    let sign = if seconds < 0.0 { "-" } else { "+" };
    let s = seconds.abs();
    let (value, unit) = if s < 1e-3 {
        (s * 1e6, "µs")
    } else if s < 1.0 {
        (s * 1e3, "ms")
    } else if s < 60.0 {
        (s, "s")
    } else if s < 3600.0 {
        (s / 60.0, "min")
    } else if s < 86400.0 {
        (s / 3600.0, "h")
    } else if s < SECONDS_PER_YEAR {
        (s / 86400.0, "d")
    } else {
        (s / SECONDS_PER_YEAR, "yr")
    };
    let number = if value >= 1e4 {
        format!("{value:.0e}")
    } else if (value - value.round()).abs() < 1e-6 * value.max(1.0) {
        format!("{:.0}", value.round())
    } else {
        format!("{value:.1}")
    };
    format!("{sign}{number} {unit}")
}

/// Where a telemetry box is kept between frames once the user has moved it.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Placement {
    /// Displaced from the anchor next to the observer by this much: the box follows the observer.
    Offset(Vec2),
    /// At this position relative to the canvas's top-left corner, whatever the observer does.
    Pinned(Vec2),
}

/// Where the box goes this frame, before clamping.
fn resolve_placement(placement: Option<Placement>, anchored: Pos2, canvas_min: Pos2) -> Pos2 {
    match placement {
        None => anchored,
        Some(Placement::Offset(v)) => anchored + v,
        Some(Placement::Pinned(p)) => canvas_min + p,
    }
}

/// What to remember after a frame in which the box ended up at `moved_min`.
///
/// A following box records its offset every frame, so a drag against the canvas edge does not
/// build up an offset that snaps back later. A pinning box is left alone until the user actually
/// drags it, since a box that has never been touched should keep following its observer; from
/// the first drag on it records where it stands relative to the canvas every frame, which is what
/// keeps a box that a resize has pushed inward from jumping back out again.
fn remembered_placement(
    pin_on_drag: bool,
    dragged: bool,
    previous: Option<Placement>,
    moved_min: Pos2,
    anchored: Pos2,
    canvas_min: Pos2,
) -> Option<Placement> {
    if !pin_on_drag {
        Some(Placement::Offset(moved_min - anchored))
    } else if dragged || previous.is_some() {
        Some(Placement::Pinned(moved_min - canvas_min))
    } else {
        None
    }
}

/// The remembered positions of the hovering telemetry boxes on one canvas, keyed by canvas tag
/// and observer name so that the same observer can have a different box position in each diagram.
#[derive(Default)]
pub struct TelemetryBoxes {
    placements: HashMap<String, Placement>,
    /// Whether a drag pins the box to the canvas where it was dropped (the equatorial view), or
    /// keeps it following the observer at the dragged offset (the (t, r) diagram, the default).
    pin_on_drag: bool,
}

impl TelemetryBoxes {
    /// Boxes that stay where the user drops them, however the observer moves afterwards.
    pub fn pinning() -> Self {
        Self { placements: HashMap::new(), pin_on_drag: true }
    }

    /// Lay out, register, drag and paint one observer's info box.
    ///
    /// The box is anchored next to `pos` exactly as before until the user moves it; after that it
    /// is either displaced from that anchor by the dragged offset or pinned to the canvas where it
    /// was dropped (see `Placement`), and in both cases clamped back inside `canvas_rect`. A
    /// double-click forgets the move. The `ui.interact` must be called after the canvas has
    /// allocated its own painter response: within a layer egui hands an overlapping drag to the
    /// widget registered last, so registering here is what stops a drag on the box from panning
    /// the background or grabbing Bob's marker.
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
        let previous = self.placements.get(&key).copied();
        let lines = telemetry_lines(name, color, obs, metric, use_km);
        let size = telemetry_box_size(painter, &lines, font_scale);

        let anchored = default_badge_pos(canvas_rect, pos, size);
        let placed = resolve_placement(previous, anchored, canvas_rect.min);
        let badge_rect = clamp_into(Rect::from_min_size(placed, size), canvas_rect);

        let id = ui.id().with(("telemetry", canvas_tag, name));
        // click_and_drag rather than drag alone: egui only reports a double-click on a widget that
        // senses clicks, and the double-click is what resets the offset.
        let response = ui.interact(badge_rect, id, egui::Sense::click_and_drag());

        let badge_rect = if response.double_clicked() {
            self.placements.remove(&key);
            clamp_into(Rect::from_min_size(anchored, size), canvas_rect)
        } else {
            let moved = clamp_into(badge_rect.translate(response.drag_delta()), canvas_rect);
            match remembered_placement(
                self.pin_on_drag,
                response.dragged(),
                previous,
                moved.min,
                anchored,
                canvas_rect.min,
            ) {
                Some(placement) => {
                    self.placements.insert(key, placement);
                }
                None => {
                    self.placements.remove(&key);
                }
            }
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

    // A Static or ZAMO selection at a radius where that worldline does not exist is not quietly
    // shown as free fall: it *is* free fall - `Observer::effective_mode` steps the observer along
    // it - and the line says which selection was refused and why, in the observer card's own words.
    // Every other line of the box is already the free-faller's, because they are all read off the
    // one 4-velocity the worldline is being drawn from.
    let a_str = if let Some(note) = impossible_mode_note(obs, metric) {
        note.to_string()
    } else if is_geodesic || a_prop < 0.05 {
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
    // Scientific notation at both ends of the scale. The ratio is proportional to u^t near the
    // far branch of r- - exactly u^t r-^2/(r-^2 + a^2) in the limit - and `geodesic::U_T_STALL`
    // follows the worldline out to u^t = 1e10, so a plain decimal would run to ten digits in a
    // box laid out for four.
    let nu_str = if !(0.01..1e4).contains(&nu_ratio) {
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

/// How close to the ring a drag may put an observer. The ring is the curvature singularity and the
/// end of every worldline that reaches it, so a drop onto r = 0 is a drop onto nothing there is a
/// frame at; this is the same floor the old Bob-only drag used.
const RING_DROP_FLOOR: f64 = 0.04;

/// Which observer's marker a drag on the (t, r) diagram has hold of. Both of them can be picked
/// up: a drag is the question "what if they were *here* instead", and it is as fair to ask of
/// Alice as of Bob.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Marker {
    Alice,
    Bob,
}

impl Marker {
    /// The drawn radius of this observer's marker, which also sets how close the pointer has to
    /// come to pick it up.
    fn radius(self) -> f32 {
        match self {
            Self::Alice => 5.5,
            Self::Bob => 8.0,
        }
    }
}

/// A marker drag in progress: whose it is, and the Motion they were on when it started.
///
/// The Motion is kept because `Observer::set_drag_position` puts whoever is being moved into
/// Drag / Manual for as long as the pointer holds them - that is what a hand on the marker means -
/// and dropping them has to give back the worldline they were on. A free-faller resumes free fall
/// from the event they were dropped at; a ZAMO goes back to holding the new radius; an observer
/// who was in Drag / Manual by choice stays there.
#[derive(Clone, Copy, PartialEq, Debug)]
struct MarkerDrag {
    who: Marker,
    mode_before: ObserverMode,
    /// Marker centre minus pointer at the moment the pointer took hold, in screen pixels, added
    /// back on every frame of the drag. Without it the observer is placed *at* the pointer, so
    /// picking a marker up anywhere but dead centre teleports it under the cursor by as much as
    /// three marker radii before the drag has moved at all.
    grab: Vec2,
}

pub struct SpacetimeCanvas {
    pub max_r: f64,
    pub r_offset: f64,
    pub time_window: f64,
    pub time_offset: f64,
    /// The marker drag in progress on this canvas, if any. Cleared when the pointer is released,
    /// when the observer being dragged leaves the simulation, and by `SpacetimeCanvas::end_drag`
    /// when the run is rebuilt under it.
    dragging: Option<MarkerDrag>,
    /// Where the user has dragged each info box on this canvas, per diagram and per observer.
    pub telemetry: TelemetryBoxes,
}

impl Default for SpacetimeCanvas {
    fn default() -> Self {
        Self {
            max_r: 5.5,
            r_offset: 0.0,
            time_window: 14.0,
            time_offset: 0.0,
            dragging: None,
            telemetry: TelemetryBoxes::default(),
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
    /// marker under it; while the pointer holds it the observer stands at the pointer's event, put
    /// there by `Observer::set_drag_position`, which is what a hand on the marker means: the
    /// worldline is being placed rather than integrated. Releasing hands them back the Motion they
    /// were on through `Observer::release_from_drag`, which restarts the geodesic from the event
    /// they were dropped at, so a drag asks "what if they were here" without answering the
    /// separate question of how they move.
    ///
    /// The dropped radius is clamped into the radial window on screen and held off the ring, so a
    /// drag cannot put an observer somewhere the user cannot see or onto the curvature singularity
    /// itself. The window is `r_offset .. r_offset + max_r` and not `0 .. max_r`: `max_r` is the
    /// *width* of the drawn range rather than its top, and clamping to it threw every drag taken
    /// while the view was panned or zoomed - which is what the Focus r- and Focus Bob buttons do -
    /// onto a radius far below the one under the pointer.
    ///
    /// A drag whose observer has left the simulation - their card unticked while the pointer is
    /// down - is dropped rather than carried, because there is no worldline left to place.
    #[allow(clippy::too_many_arguments)]
    fn drag_markers(
        &mut self,
        metric: &KerrSchild,
        response: &egui::Response,
        alice: Option<&mut Observer>,
        bob: Option<&mut Observer>,
        window: std::ops::RangeInclusive<f64>,
        to_screen: impl Fn(&Observer) -> Pos2,
        to_event: impl Fn(Pos2) -> (f64, f64),
    ) {
        let mut markers: Vec<(Marker, &mut Observer)> = Vec::new();
        if let Some(al) = alice {
            markers.push((Marker::Alice, al));
        }
        if let Some(b) = bob {
            markers.push((Marker::Bob, b));
        }

        if response.drag_started()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            let mut nearest: Option<(MarkerDrag, f32)> = None;
            for (who, obs) in markers.iter() {
                let reach = who.radius() * 3.0;
                let at = to_screen(obs);
                let distance = at.distance(pointer);
                if distance < reach && nearest.is_none_or(|(_, best)| distance < best) {
                    let drag =
                        MarkerDrag { who: *who, mode_before: obs.mode, grab: at - pointer };
                    nearest = Some((drag, distance));
                }
            }
            self.dragging = nearest.map(|(drag, _)| drag);
        }

        let Some(drag) = self.dragging else {
            return;
        };
        let floor = RING_DROP_FLOOR.max(*window.start());
        match markers.iter_mut().find(|(who, _)| *who == drag.who) {
            Some((_, obs)) => {
                if response.dragged()
                    && let Some(pointer) = response.interact_pointer_pos()
                {
                    let (t, r) = to_event(pointer + drag.grab);
                    obs.set_drag_position(t, r.clamp(floor, window.end().max(floor)));
                }
                if response.drag_stopped() {
                    obs.release_from_drag(metric, drag.mode_before);
                    self.dragging = None;
                }
            }
            None => self.dragging = None,
        }
    }

    /// Render the (t, r) spacetime foliation canvas with an integrated, perfectly aligned 1D radial track
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: Option<&mut Observer>,
        alice: Option<&mut Observer>,
        current_time: f64,
        canvas_height: f32,
        use_km: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
        signals: SignalViews<'_>,
        show_distant_clock_grid: bool,
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

        // The rest-frame views need one observer to be the frame and take the other, if there is
        // one, as a guest. Either may be missing, so the frame the user asked for falls back to
        // whoever is left, and with nobody left there is no rest frame to draw at all.
        let (mut bob, mut alice) = (bob, alice);
        match frame_of_ref {
            ReferenceFrame::Bob | ReferenceFrame::Alice => {
                painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
                let (asked, other) = match frame_of_ref {
                    ReferenceFrame::Alice => (alice.as_deref(), bob.as_deref()),
                    _ => (bob.as_deref(), alice.as_deref()),
                };
                match (asked, other) {
                    (Some(focus), other) => self.render_observer_frame(
                        ui, &painter, rect, metric, focus, other, use_km, font_scale,
                        show_distant_clock_grid,
                    ),
                    (None, Some(focus)) => self.render_observer_frame(
                        ui, &painter, rect, metric, focus, None, use_km, font_scale,
                        show_distant_clock_grid,
                    ),
                    (None, None) => {
                        painter.text(
                            rect.center(),
                            egui::Align2::CENTER_CENTER,
                            "No observer in the simulation
Tick Enable Observer on Alice's or Bob's card",
                            egui::FontId::proportional(13.0 * font_scale),
                            Theme::TEXT_MUTED,
                        );
                    }
                }
            }
            ReferenceFrame::DistantObserver => {
                self.render_distant_observer(
                    ui,
                    &painter,
                    &response,
                    rect,
                    metric,
                    bob.as_deref_mut(),
                    alice.as_deref_mut(),
                    current_time,
                    use_km,
                    font_scale,
                    signals,
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
        if let Some(al) = alice.as_deref()
            && al.is_active
        {
            let al_x = track_to_x(al.r);
            t_painter.circle_filled(Pos2::new(al_x, center_y), 6.0, Theme::ALICE_COLOR);
            t_painter.text(Pos2::new(al_x, center_y - 10.0), egui::Align2::CENTER_BOTTOM, "Alice", egui::FontId::proportional(10.0 * font_scale), Theme::ALICE_COLOR);
        }

        // Draw Bob on Track
        if let Some(bob) = bob.as_deref().filter(|b| b.is_active) {
            let bob_x = track_to_x(bob.r);
            t_painter.circle_filled(Pos2::new(bob_x, center_y), 6.5, Theme::BOB_COLOR);
            t_painter.circle_stroke(Pos2::new(bob_x, center_y), 8.5, Stroke::new(1.0, Color32::WHITE));
            t_painter.text(Pos2::new(bob_x, center_y + 9.0), egui::Align2::CENTER_TOP, "Bob", egui::FontId::proportional(10.0 * font_scale), Theme::BOB_COLOR);
        }

        // Radial separation between Alice and Bob, if both are present
        if let (Some(al), Some(bob)) = (alice.as_deref(), bob.as_deref())
            && al.is_active
            && bob.is_active
        {
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

    #[allow(clippy::too_many_arguments)]
    fn render_distant_observer(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        response: &egui::Response,
        rect: Rect,
        metric: &KerrSchild,
        bob: Option<&mut Observer>,
        alice: Option<&mut Observer>,
        current_time: f64,
        use_km: bool,
        font_scale: f32,
        signals: SignalViews<'_>,
    ) {
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;

        // The radial projection, in locals rather than read through `self`, so that the closures
        // built on it do not hold a borrow of the canvas for as long as they live: the marker drag
        // below needs `&mut self` while they are still in scope. They are taken before the
        // background pan, so every projection in this frame is the one the user is looking at as
        // they press: a pan applied to the closures mid-frame would move the picture out from
        // under the pointer that is acting on it, which is exactly what the time axis already
        // avoids by fixing t_min and t_max above.
        let (r_offset, max_r) = (self.r_offset, self.max_r);

        let to_screen_x = |r: f64| -> f32 {
            let frac = ((r - r_offset) / max_r) as f32;
            rect.left() + frac * rect.width()
        };

        let to_screen_y = |t: f64| -> f32 {
            let frac = ((t - t_min) / (t_max - t_min)) as f32;
            rect.bottom() - frac * rect.height()
        };

        let to_coord_r = |screen_x: f32| -> f64 {
            let frac = ((screen_x - rect.left()) / rect.width()) as f64;
            (r_offset + frac * max_r).max(0.0)
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

        // The two transmissions. A wavefront is a closed curve in (r, phi) and this diagram has
        // no azimuth to draw it on, so what is drawn is the one thing the projection does define:
        // the pulse's radial extent, [min r, max r] over its front, swept up in t. That is the
        // wedge of `Pulse::extent_track`. Its lower edge is the ingoing edge of the emitter's own
        // light cone, carried from the emission event - the 45-degree line dr/dt = -1 only for a
        // hole with no spin, and slightly steeper than that for one that spins (-1.010 at r = 4.5M
        // for a = 0.65, -2.27 at r = 0.2M for a = 0.90) - and its upper edge is the outermost ray,
        // which outside r+ climbs and inside r+ falls and freezes onto r-. While the pulse is being
        // swallowed the lower edge stands on the ring: the rays are a sampling of a continuous
        // front, and `Pulse::radial_extent` is what says when the front itself is down there rather
        // than only the innermost sample of it. A worldline inside a wedge is *in range* of that
        // pulse - some part of the front stands at that radius - which is not the same as receiving
        // it, because the diagram cannot show azimuth and the receiver may be at another one. The
        // reception dots below are the actual arrivals.
        //
        // Each field is drawn in its emitter's colour: the interior in `Theme::WEDGE_FILL_ALPHA`,
        // faint enough that the forty-odd wedges of a whole infall stack up without flattening into
        // a block, and the two edges as thin lines at `Theme::WEDGE_EDGE_ALPHA`. Inside r+ every
        // upper edge freezes on r-, so those edges pile onto the Cauchy horizon, which in this
        // chart is where the outgoing light of the whole interior accumulates, and that pile is
        // what a later infaller cuts through.
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
        draw_wedges(signals.bob, Theme::BOB_COLOR);
        draw_wedges(signals.alice, Theme::ALICE_COLOR);

        // Alice Worldline & Marker. The info boxes are registered last, below, so that a drag on
        // a box beats the canvas's own pan/drag response instead of panning time or moving Bob.
        let (mut bob, mut alice) = (bob, alice);
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice.as_deref() {
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
                    let held = self.dragging.is_some_and(|d| d.who == Marker::Alice);
                    let colour = if held { Color32::WHITE } else { Theme::ALICE_COLOR };
                    painter.circle_filled(alice_pos, Marker::Alice.radius(), colour);
                    painter.circle_stroke(alice_pos, 7.5, Stroke::new(1.0, Color32::WHITE));
                    alice_box = Some(alice_pos);
                }
            }
        }

        // Bob Worldline & Dragging
        if let Some(bob) = bob.as_deref().filter(|b| b.trail.len() >= 2) {
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
        // An arrival is a mark on the *receiver's* worldline, so each transmission's marks are
        // drawn only while the observer who heard it is in the simulation: with the receiver gone
        // there is no worldline under the dots for them to sit on.
        if bob.is_some() {
            draw_receptions(signals.alice);
        }
        if alice.is_some() {
            draw_receptions(signals.bob);
        }


        // Either marker can be picked up and put somewhere else. It is registered before the
        // telemetry boxes, which go last of all, so that a drag on a box moves the box rather than
        // the observer underneath it, and *before* the background pan below, which stands down
        // while a marker is held: the pan reads `self.dragging`, and a pick made after it would
        // leave the first frame of every marker drag panning the diagram as well.
        self.drag_markers(
            metric,
            response,
            alice.as_deref_mut(),
            bob.as_deref_mut(),
            r_offset..=r_offset + max_r,
            |obs| Pos2::new(to_screen_x(obs.r), to_screen_y(obs.t)),
            |at| (to_coord_t(at.y), to_coord_r(at.x)),
        );

        // Mouse drag background panning for time and radial offset. It takes effect on the next
        // frame, since this frame's projections were fixed above.
        if response.dragged() && self.dragging.is_none() {
            let delta = response.drag_delta();
            let dt = (delta.y as f64 / rect.height() as f64) * (t_max - t_min);
            self.time_offset += dt;
            let dr = (delta.x as f64 / rect.width() as f64) * max_r;
            self.r_offset = (self.r_offset - dr).max(0.0);
        }

        // Everything left on this canvas is Bob's: his worldline, his light cone, his marker and
        // the two boxes read off him. With no Bob in the simulation there is none of it.
        let Some(bob) = bob else {
            if let (Some(al), Some(alice_pos)) = (alice.as_deref(), alice_box) {
                self.telemetry.show(
                    ui, painter, "spacetime", rect, alice_pos, "Alice", Theme::ALICE_COLOR, al,
                    metric, use_km, font_scale,
                );
            }
            return;
        };

        let bob_radius = Marker::Bob.radius();

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
        let held = self.dragging.is_some_and(|d| d.who == Marker::Bob);
        painter.circle_filled(
            apex,
            bob_radius,
            if held { Color32::WHITE } else { Theme::BOB_COLOR },
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
        if let (Some(al), Some(alice_pos)) = (alice.as_deref(), alice_box) {
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
        show_distant_clock_grid: bool,
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

        // 2. The distant clock's own slices: the surfaces t = const of the chart's Killing time,
        // which is proper time on a clock at rest at infinity. `surface_t_const` places each of
        // them from the covector dt in local components, so a line labelled "+5 min" is the set of
        // events the distant clock reads five minutes after it read the observer's now, and it
        // meets this worldline at exactly 5 min / u^t of the observer's own proper time. The step
        // is picked from u^t and the pixel scale alone, so the grid stays readable while the
        // outside clock runs away.
        //
        // `scale` is the pixel scale of the drawn plane: points per M of xi. The physical second
        // per M of coordinate time is t_g / M, the same conversion `format_physical_time` uses.
        let u_t = frame.tetrad().e0[0];
        let seconds_per_m = metric.t_grav_seconds() / metric.m.max(1e-12);
        let clock_grid = distant_clock_grid_step(u_t, scale, seconds_per_m, font_scale);
        // Proper time on the observer's own clock between two neighbouring lines, in M. Exact.
        let clock_proper_step = if u_t.is_finite() && u_t > 0.0 {
            clock_grid.step_m / u_t
        } else {
            f64::INFINITY
        };
        if show_distant_clock_grid && clock_proper_step.is_finite() && clock_proper_step > 0.0 {
            let spacing_px = (clock_proper_step as f32) * scale;
            // How far in xi^0 the grid has to reach to cover the rectangle: half its height, plus
            // half its width, because a slice is tilted by up to (but never quite) 45 degrees.
            let reach = ((rect.height() + rect.width()) * 0.5 / scale.max(1e-6)) as f64;
            let k_max = ((reach / clock_proper_step).ceil() as i64).clamp(0, 4096);
            // Label every n-th line, where n is the fewest that keeps two labels from touching.
            // With the step chosen at `MIN_GRID_PX` or coarser this is 1; it only rises at the top
            // of the ladder, where no rung is coarse enough and the lines are squeezed together.
            let label_every = if spacing_px > 0.0 {
                (((CLOCK_LABEL_MIN_PX * font_scale) / spacing_px).ceil() as i64).max(1)
            } else {
                1
            };
            let label_colour = Color32::from_rgba_premultiplied(140, 165, 195, 180);
            for k in -k_max..=k_max {
                let dt = (k as f64) * clock_grid.step_m;
                let line = frame.surface_t_const(dt);
                let anchor = to_screen(line.point[0], line.point[1]);
                let dir = Vec2::new(line.dir[0] as f32, -(line.dir[1] as f32));
                let Some((end_a, end_b)) = clip_line_to_rect(anchor, dir, rect) else {
                    continue;
                };
                painter.line_segment(
                    [end_a, end_b],
                    Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE),
                );
                if k % label_every != 0 {
                    continue;
                }
                let x = rect.left() + 6.0;
                let y = segment_y_at_x(end_a, end_b, x);
                painter.text(
                    Pos2::new(x, y - 2.0),
                    egui::Align2::LEFT_BOTTOM,
                    distant_clock_offset_label(dt * seconds_per_m),
                    egui::FontId::monospace(9.0 * font_scale),
                    label_colour,
                );
            }
        }

        // 3. Surfaces r = const, every one of them placed by the dual tetrad.
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
        if let Some(other) = other_obs
            && other.is_active
    {
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

        let mut banner = format!(
            "🔭 {}'S REST FRAME  |  first-order local inertial frame (c ≡ 1, 45° light cones); \
             surfaces r = const placed by the dual tetrad",
            focus_obs.name.to_uppercase()
        );
        if show_distant_clock_grid && clock_proper_step.is_finite() && clock_proper_step > 0.0 {
            // The grid's two numbers side by side: what one line is worth on the distant clock, and
            // what the gap between two of them is worth on this observer's own clock. The second is
            // step_m / u^t, the exact spacing `surface_t_const` puts on the worldline - a
            // simultaneity convention, not what the observer sees; see the checkbox's tip.
            banner.push_str(&format!(
                "\nDistant clock grid: 1 line per {} of distant time; \
                 {} on {}'s clock between lines",
                clock_grid.label,
                metric.format_physical_time(clock_proper_step),
                focus_obs.name
            ));
        }
        painter.text(
            Pos2::new(rect.left() + 8.0, rect.top() + 8.0),
            egui::Align2::LEFT_TOP,
            banner,
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
    if t_min > t_max || !t_min.is_finite() || !t_max.is_finite() {
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

#[cfg(test)]
mod telemetry_placement_tests {
    use super::*;

    #[test]
    fn test_a_dragged_box_on_a_pinning_canvas_stays_put_while_its_observer_moves() {
        // The equatorial view: before any drag the box follows its observer; after one it stands
        // at a fixed place on the canvas, and the anchor moving with the observer no longer moves
        // it. A double-click clears the placement, which is the `None` branch of `resolve`.
        let canvas_min = Pos2::new(100.0, 50.0);
        let anchored = Pos2::new(300.0, 200.0);
        assert_eq!(resolve_placement(None, anchored, canvas_min), anchored);
        assert_eq!(
            remembered_placement(true, false, None, anchored, anchored, canvas_min),
            None,
            "an untouched box keeps following"
        );
        let dropped = Pos2::new(140.0, 90.0);
        let pinned = remembered_placement(true, true, None, dropped, anchored, canvas_min);
        assert_eq!(pinned, Some(Placement::Pinned(Vec2::new(40.0, 40.0))));
        let later_anchor = Pos2::new(420.0, 310.0);
        assert_eq!(
            resolve_placement(pinned, later_anchor, canvas_min),
            dropped,
            "the observer moved on and the box did not"
        );
        // Not dragged this frame, but already pinned: still pinned, at where it now stands.
        let slid = Pos2::new(150.0, 90.0);
        assert_eq!(
            remembered_placement(true, false, pinned, slid, later_anchor, canvas_min),
            Some(Placement::Pinned(Vec2::new(50.0, 40.0)))
        );
    }

    #[test]
    fn test_a_dragged_box_on_a_following_canvas_keeps_its_offset_from_its_observer() {
        // The (t, r) diagram, unchanged: the drag is remembered as an offset from the anchor and
        // the box goes on tracking the observer at that offset.
        let canvas_min = Pos2::new(0.0, 0.0);
        let anchored = Pos2::new(300.0, 200.0);
        let dropped = Pos2::new(330.0, 180.0);
        let placement = remembered_placement(false, true, None, dropped, anchored, canvas_min);
        assert_eq!(placement, Some(Placement::Offset(Vec2::new(30.0, -20.0))));
        let later_anchor = Pos2::new(300.0, 150.0);
        assert_eq!(
            resolve_placement(placement, later_anchor, canvas_min),
            Pos2::new(330.0, 130.0)
        );
    }
}

/// The (t, r) canvas as the user meets it: the distant clock's grid in a rest-frame view, and the
/// marker drags on the foliation view. Both are driven through real frames of
/// `SpacetimeCanvas::render` with real pointer input, so what is measured is what is drawn and what
/// a hand on the mouse does, rather than the arithmetic underneath either.
#[cfg(test)]
mod canvas_tests {
    use super::*;

    /// One M of coordinate time in seconds for the app's default hole, ten solar masses:
    /// t_g = GM/c^3 = 4.9e-5 s. At that scale a raindrop outside the hole, whose u^t is of order 1,
    /// wants a grid measured in tens of microseconds, which is why the ladder reaches below a
    /// second - and why it carries the 1-2-5 pattern up through the decades inside each unit.
    const TEN_SOLAR_SECONDS_PER_M: f64 = 4.9e-5;

    #[test]
    fn test_the_distant_clock_grid_step_climbs_the_ladder_as_the_outside_clock_runs_away() {
        // The step is chosen from u^t and the pixel scale and from nothing else: the on-screen gap
        // between two lines is step_m / u^t * px_per_m, so the rung has to clear MIN_GRID_PX. What
        // makes that a *grid* rather than a single line is that the ladder has no holes in it - the
        // chosen rung is never much coarser than the one actually needed - so the four things
        // checked here are: the ladder's own step ratios, monotonicity, readability, and that a
        // real exterior view comes out with a run of lines in it rather than one.

        // 1. No hole wider than a factor of 2.5 anywhere in the ladder, ends included. 1, 2, 5 once
        //    per unit would leave 5 µs -> 1 ms (200x) and 5 day -> 1 yr (73x); carrying the 1-2-5
        //    pattern up through the decades inside each unit closes both.
        let ladder = distant_clock_ladder();
        let mut worst_ratio = 0.0f64;
        let mut worst_pair = (String::new(), String::new());
        for pair in ladder.windows(2) {
            let (lo, hi) = (&pair[0], &pair[1]);
            assert!(hi.0 > lo.0, "the ladder must ascend: {} then {}", lo.1, hi.1);
            let ratio = hi.0 / lo.0;
            if ratio > worst_ratio {
                worst_ratio = ratio;
                worst_pair = (lo.1.clone(), hi.1.clone());
            }
        }
        // The tolerance is f64 rounding on the ratios themselves (5 / 2 comes out as
        // 2.5000000000000004 through the unit scale factors), not slack in the rule.
        assert!(
            worst_ratio <= 2.5 * (1.0 + 1e-9),
            "widest hole in the ladder is {} -> {} ({worst_ratio}x)",
            worst_pair.0,
            worst_pair.1
        );

        let px_per_m = 100.0f32;
        let secs = TEN_SOLAR_SECONDS_PER_M;
        let mut previous = 0.0f64;
        let mut samples = 0usize;
        let mut worst_gap = f64::INFINITY;
        let mut widest_gap = 0.0f64;
        for i in 0..=200 {
            let u_t = 10.0f64.powf(i as f64 * 0.1);
            let step = distant_clock_grid_step(u_t, px_per_m, secs, 1.0);
            // 2. Monotone non-decreasing: a faster outside clock never buys a finer grid.
            assert!(
                step.step_m >= previous,
                "step fell from {previous} to {} at u^t = {u_t:e}",
                step.step_m
            );
            previous = step.step_m;
            // 3. Readable: never closer together on screen than MIN_GRID_PX, and - because the
            //    ladder has no holes - never more than 2.5 times that far apart either.
            let gap = step.step_m / u_t * (px_per_m as f64);
            assert!(
                gap >= MIN_GRID_PX as f64,
                "lines {gap} px apart at u^t = {u_t:e} (step {})",
                step.label
            );
            assert!(
                gap <= 2.5 * MIN_GRID_PX as f64 * (1.0 + 1e-9),
                "lines {gap} px apart at u^t = {u_t:e} (step {}): the ladder has a hole",
                step.label
            );
            worst_gap = worst_gap.min(gap);
            widest_gap = widest_gap.max(gap);
            samples += 1;
        }

        // 4. The view the app actually opens on: Bob falling free outside a ten solar-mass hole has
        //    u^t of order 1.5, and the frame view runs at about 57 points per M at the default zoom.
        //    One M is 49 µs there, so a readable grid is in tens of microseconds - and a 500 point
        //    tall canvas has to hold a run of those lines, not one.
        let exterior = distant_clock_grid_step(1.5, 57.0, TEN_SOLAR_SECONDS_PER_M, 1.0);
        let spacing_px = exterior.step_m / 1.5 * 57.0;
        let fit = (500.0 / spacing_px).floor() as i32;
        assert!(
            fit >= 6,
            "only {fit} lines of {} fit a 500 px view ({spacing_px:.1} px apart)",
            exterior.label
        );

        // Where the ladder stands for a ten solar-mass hole at a hundred points per M, decade by
        // decade of u^t: microseconds outside the hole, up through milliseconds, seconds, minutes,
        // hours and days, and into the years and then the powers of ten of years as the observer
        // falls toward the far branch of r- and the outside clock runs away.
        let at = |u_t: f64| distant_clock_grid_step(u_t, px_per_m, secs, 1.0).label;
        let walk: Vec<String> = (0..=20).map(|e| at(10.0f64.powi(e))).collect();
        println!(
            "distant clock ladder: {} rungs, widest step ratio {worst_ratio:.3} ({} -> {}); \
             {samples} samples of u^t in [1, 1e20], on-screen gap between {worst_gap:.2} and \
             {widest_gap:.2} px (MIN_GRID_PX = {MIN_GRID_PX}); u^t = 1e0..1e20 -> {walk:?}; \
             at u^t = 1.5, 57 px/M the step is {} = {:.4} M, {spacing_px:.1} px apart, {fit} lines \
             in a 500 px view",
            ladder.len(),
            worst_pair.0,
            worst_pair.1,
            exterior.label,
            exterior.step_m
        );
        assert_eq!(at(1.0), "20 µs");
        assert!(walk.iter().any(|l| l.ends_with(" ms")));
        assert!(walk.iter().any(|l| l.ends_with(" s")));
        assert!(walk.iter().any(|l| l.ends_with(" min")));
        assert!(walk.iter().any(|l| l.ends_with(" hr")));
        assert!(walk.iter().any(|l| l.ends_with(" day")));
        assert!(walk.last().unwrap().ends_with(" yr"));

        // Microseconds are reachable at the bottom: a supermassive hole, where one M is hours, puts
        // even a coarse grid far below a second of the chart's own time unit.
        let tiny = distant_clock_grid_step(1.0, 1e6, 1e-3, 1.0);
        assert!(
            tiny.label.ends_with(" \u{b5}s"),
            "the bottom of the ladder is microseconds, got {}",
            tiny.label
        );
        // A bigger font asks for more room and so for a coarser grid, never a finer one.
        let small = distant_clock_grid_step(1e6, px_per_m, secs, 1.0);
        let large = distant_clock_grid_step(1e6, px_per_m, secs, 2.0);
        assert!(large.step_m >= small.step_m, "{} vs {}", large.label, small.label);
    }

    /// Every line segment an egui painter was handed, counted through nested `Shape::Vec`s.
    fn count_line_segments(shape: &egui::Shape) -> usize {
        match shape {
            egui::Shape::LineSegment { .. } => 1,
            egui::Shape::Vec(inner) => inner.iter().map(count_line_segments).sum(),
            _ => 0,
        }
    }

    /// The simulation clock every `marker_frame` renders at.
    const MARKER_FRAME_CLOCK: f64 = 0.0;

    /// One real frame of the (t, r) diagram with both observers on it, driven by whatever pointer
    /// events are handed in, returning the screen position of each marker as it was painted.
    ///
    /// The positions are read off the painter rather than recomputed from the canvas's projection,
    /// so a test that then presses on one of them is pressing where the user sees it.
    fn marker_frame(
        canvas: &mut SpacetimeCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        alice: &mut Observer,
        bob: &mut Observer,
        events: Vec<egui::Event>,
    ) -> Vec<(Color32, f32, Pos2)> {
        use crate::physics::wavefront::SignalField;
        let signal = SignalField::default();
        // A clock of its own, not read off either observer: the time window is pinned to the
        // simulation clock, and a window that slid with the observer being dragged would hide
        // every vertical move by following it.
        let clock = MARKER_FRAME_CLOCK;
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
            events,
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            canvas.render(
                ui,
                metric,
                Some(bob),
                Some(alice),
                clock,
                600.0,
                false,
                ReferenceFrame::DistantObserver,
                1.0,
                SignalViews { alice: &signal, bob: &signal },
                false,
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
        circles
    }

    /// Where the marker of the given colour and drawn radius was painted on the main canvas.
    fn marker_at(circles: &[(Color32, f32, Pos2)], colour: Color32, radius: f32) -> Option<Pos2> {
        circles
            .iter()
            .find(|(fill, r, _)| *fill == colour && (*r - radius).abs() < 1e-6)
            .map(|(_, _, at)| *at)
    }

    #[test]
    fn test_a_drag_puts_an_observer_under_the_pointer_however_the_view_is_zoomed() {
        // Two ways the drag used to throw an observer somewhere the pointer never went.
        //
        // The radial window on screen is r_offset ..= r_offset + max_r, and `max_r` is its *width*
        // rather than its top. The drop was clamped into 0.04 ..= max_r, which is the same range
        // only while the view has never been panned; zoom in on r- with the Focus button - a
        // window 0.05 M wide sitting at r- itself - and every drag, however small, was clamped to
        // r = 0.05, throwing the observer from the Cauchy horizon to within a hair of the ring.
        //
        // And the observer was placed *at* the pointer rather than keeping the offset the pointer
        // took hold at, so a marker picked up anywhere but dead centre jumped under the cursor
        // before the drag had moved at all - as much as three marker radii, which is most of the
        // width of that zoomed-in window.
        //
        // Both are measured here against the drawing: the projection is recovered from where the
        // marker is actually painted at two known radii, so what is checked is that moving the
        // pointer by n pixels moves the observer by exactly the n pixels' worth of radius the
        // diagram is drawn at.
        let metric = KerrSchild::new(1.0, 0.65);
        let r_minus = metric.inner_horizon();
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());

        // The Focus r- window: 0.05 M wide, centred on the Cauchy horizon, and nowhere near 0.
        let mut canvas = SpacetimeCanvas {
            max_r: 0.05,
            r_offset: (r_minus - 0.025).max(0.0),
            ..Default::default()
        };
        let mut alice = Observer::new(&metric, "Alice", 0.0, 4.5, 0.0);
        let mut bob = Observer::new(&metric, "Bob", 0.0, r_minus, 0.0);
        let start_r = bob.r;

        // The projection, read off the painter at two radii a known distance apart.
        let bob_x = |canvas: &mut SpacetimeCanvas, alice: &mut Observer, bob: &mut Observer| {
            let painted = marker_frame(canvas, &ctx, &metric, alice, bob, vec![]);
            marker_at(&painted, Theme::BOB_COLOR, Marker::Bob.radius())
                .expect("Bob's marker is always painted")
        };
        let at = bob_x(&mut canvas, &mut alice, &mut bob);
        bob.r = start_r + 0.01;
        let moved = bob_x(&mut canvas, &mut alice, &mut bob);
        bob.r = start_r;
        let px_per_m = ((moved.x - at.x) / 0.01) as f64;
        assert!(px_per_m > 100.0, "the window is zoomed in: {px_per_m} px per M");

        // Pressed a little off centre, which is where the grab offset earns its place.
        let press = at + Vec2::new(6.0, 4.0);
        let events = |pos: Pos2, pressed: Option<bool>| {
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
        };
        marker_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, events(press, Some(true)));
        // Past egui's drag threshold: the pick happens here, and nothing has moved yet.
        let nudge = press + Vec2::new(9.0, 0.0);
        marker_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, events(nudge, None));
        assert_eq!(canvas.dragging.map(|d| d.who), Some(Marker::Bob), "his marker was picked up");

        let travel = Vec2::new(40.0, -25.0);
        let drop_at = press + travel;
        marker_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, events(drop_at, None));
        marker_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, events(drop_at, Some(false)));

        let window = canvas.r_offset..=canvas.r_offset + canvas.max_r;
        let expected = start_r + travel.x as f64 / px_per_m;
        println!(
            "Focus r- window {:.4}..={:.4} M at {px_per_m:.0} px/M: dragged {} px and Bob went \
             {:.5} -> {:.5} M, wanted {expected:.5}",
            window.start(),
            window.end(),
            travel.x,
            start_r,
            bob.r
        );
        assert!(
            window.contains(&bob.r),
            "he must land inside the window he was dragged in, not at {}",
            bob.r
        );
        assert!(
            (bob.r - expected).abs() < 2.0 / px_per_m,
            "he must land under the pointer: {} against {expected}",
            bob.r
        );
        // The same statement made the other way round: the marker ends up under the same point of
        // the cursor it was picked up by, which is what the grab offset is for.
        let ended = bob_x(&mut canvas, &mut alice, &mut bob);
        assert!(
            (ended - (at + travel)).length() < 2.0,
            "the marker follows the pointer: {ended:?} against {:?}",
            at + travel
        );
    }

    #[test]
    fn test_either_observers_marker_can_be_dragged_and_is_dropped_back_onto_their_own_motion() {
        // A drag is the question "what if they were *here* instead", and it is as fair to ask of
        // Alice as of Bob: until now only his marker could be picked up, and hers was painted on a
        // canvas that had been handed her worldline immutably. Both are driven here through the
        // real thing - a pointer pressed on the marker where it was painted, moved, and released,
        // three frames of `SpacetimeCanvas::render` with egui deciding what counts as a drag - and
        // each has to end up at the event it was dropped at, on the Motion it was on before the
        // hand went on it. Alice's is ZAMO, which is exactly the case the restore matters for: she
        // has to go back to holding her *new* radius rather than being left in Drag / Manual or
        // silently put into free fall.
        let metric = KerrSchild::new(1.0, 0.65);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());

        for (who, colour, mode) in [
            ("Alice", Theme::ALICE_COLOR, ObserverMode::Zamo),
            ("Bob", Theme::BOB_COLOR, ObserverMode::FreeFall),
        ] {
            let mut canvas = SpacetimeCanvas::default();
            let mut alice = Observer::new_with_phi(
                &metric,
                "Alice",
                0.0,
                4.5,
                0.0,
                0.25,
                crate::physics::observer::WorldlineParams::default(),
            );
            alice.mode = ObserverMode::Zamo;
            let mut bob = Observer::new_with_phi(
                &metric,
                "Bob",
                0.0,
                4.5,
                0.0,
                0.0,
                crate::physics::observer::WorldlineParams::default(),
            );
            for _ in 0..10 {
                alice.step(&metric, alice.t + 0.1, 0.1);
                bob.step(&metric, bob.t + 0.1, 0.1);
            }
            let radius = if who == "Alice" { Marker::Alice.radius() } else { Marker::Bob.radius() };

            // Frame 1: nothing but the painting, to find out where the marker is.
            let painted = marker_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
            let at = marker_at(&painted, colour, radius)
                .unwrap_or_else(|| panic!("{who}'s marker is painted: {painted:?}"));
            let before = match who {
                "Alice" => (alice.t, alice.r),
                _ => (bob.t, bob.r),
            };

            // Frame 2: the press, on the marker.
            marker_frame(
                &mut canvas, &ctx, &metric, &mut alice, &mut bob,
                vec![
                    egui::Event::PointerMoved(at),
                    egui::Event::PointerButton {
                        pos: at,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: Default::default(),
                    },
                ],
            );
            // Frame 3: past egui's drag threshold, but still on the marker, which is where the
            // pick happens.
            let nudge = at + Vec2::new(10.0, 0.0);
            marker_frame(
                &mut canvas, &ctx, &metric, &mut alice, &mut bob,
                vec![egui::Event::PointerMoved(nudge)],
            );
            assert_eq!(
                canvas.dragging.map(|d| d.who),
                Some(if who == "Alice" { Marker::Alice } else { Marker::Bob }),
                "{who}'s marker must be the one picked up, got {:?}",
                canvas.dragging
            );

            // Frame 4: the move itself, well away from where it started.
            let dropped_at = at + Vec2::new(-90.0, -40.0);
            marker_frame(
                &mut canvas, &ctx, &metric, &mut alice, &mut bob,
                vec![egui::Event::PointerMoved(dropped_at)],
            );
            // Frame 5: the release.
            marker_frame(
                &mut canvas, &ctx, &metric, &mut alice, &mut bob,
                vec![egui::Event::PointerButton {
                    pos: dropped_at,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: Default::default(),
                }],
            );

            let moved = match who {
                "Alice" => (alice.t, alice.r, alice.mode),
                _ => (bob.t, bob.r, bob.mode),
            };
            println!(
                "{who} dragged from (t = {:.3}, r = {:.3}) to (t = {:.3}, r = {:.3}), Motion {:?}",
                before.0, before.1, moved.0, moved.1, moved.2
            );
            assert!(
                moved.1 < before.1 - 0.05,
                "{who} must end up at a smaller radius: {} vs {}",
                moved.1,
                before.1
            );
            assert!(moved.0 > before.0, "{who} must end up later in t: {} vs {}", moved.0, before.0);
            assert_eq!(moved.2, mode, "{who} is dropped back onto the Motion they were on");
            assert!(canvas.dragging.is_none(), "and the canvas is not still holding them");

            // A drag interrupted by a rebuild of the run is dropped rather than carried into it.
            canvas.dragging =
                Some(MarkerDrag { who: Marker::Bob, mode_before: mode, grab: Vec2::ZERO });
            canvas.end_drag();
            assert!(canvas.dragging.is_none(), "a reset leaves both markers pickable again");
        }
    }

    /// One real frame of `SpacetimeCanvas::render` in Bob's rest frame, with the distant clock grid
    /// on or off, returning how many line segments were painted and the text of every galley.
    fn frame_view_pass(show_distant_clock_grid: bool) -> (usize, String) {
        use crate::physics::observer::{Observer, WorldlineParams};
        use crate::physics::wavefront::SignalField;

        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        bob.step(&metric, 0.1, 0.1);
        let mut alice: Option<Observer> = None;
        let signal = SignalField::default();
        // Zoomed in on the frame, so that the chosen rung of the ladder sits close to its own
        // minimum gap and a whole run of lines lands inside the rectangle rather than one.
        let mut canvas = SpacetimeCanvas { max_r: 0.6, ..Default::default() };

        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
            ..Default::default()
        };
        let output = ctx.run_ui(input, |ui| {
            canvas.render(
                ui,
                &metric,
                Some(&mut bob),
                alice.as_mut(),
                0.1,
                600.0,
                false,
                ReferenceFrame::Bob,
                1.0,
                SignalViews { alice: &signal, bob: &signal },
                show_distant_clock_grid,
            );
        });
        let mut lines = 0;
        let mut text = String::new();
        for clipped in output.shapes.iter() {
            lines += count_line_segments(&clipped.shape);
            fn collect(shape: &egui::Shape, out: &mut String) {
                match shape {
                    egui::Shape::Text(t) => {
                        out.push_str(t.galley.text());
                        out.push('\n');
                    }
                    egui::Shape::Vec(inner) => inner.iter().for_each(|s| collect(s, out)),
                    _ => {}
                }
            }
            collect(&clipped.shape, &mut text);
        }
        output.drop_without_applying_deltas();
        (lines, text)
    }

    #[test]
    fn test_the_frame_view_draws_the_distant_clock_grid_only_when_it_is_asked_to() {
        // The checkbox at the one place it acts. Two real passes of the canvas over the same
        // observer differ by the grid: the lines themselves, and the legend line that says what one
        // of them is worth on each of the two clocks. Nothing else in the view moves, because the
        // grid is read off `LocalFrame::surface_t_const` and touches no physics.
        let (off_lines, off_text) = frame_view_pass(false);
        let (on_lines, on_text) = frame_view_pass(true);
        println!(
            "Bob's rest frame at r = 4.5: {off_lines} line segments with the distant clock grid \
             off, {on_lines} with it on ({} grid lines drawn)",
            on_lines - off_lines
        );
        assert!(
            on_lines >= off_lines + 5,
            "the grid must add a run of lines: {on_lines} vs {off_lines}"
        );
        assert!(
            !off_text.contains("Distant clock grid"),
            "the legend line belongs to the grid and goes with it"
        );
        assert!(
            on_text.contains("Distant clock grid: 1 line per "),
            "the legend must name the unit and the proper interval: {on_text}"
        );
        assert!(
            on_text.contains("on Bob's clock between lines"),
            "the legend must say whose clock the interval is on: {on_text}"
        );
        assert!(on_text.contains("now"), "the slice through the observer's own event is labelled");
    }

    #[test]
    fn test_a_distant_clock_label_names_the_offset_in_its_own_unit_with_a_sign() {
        // The label on one line: which way it is from the observer's now, and how far in the
        // largest unit that leaves a number worth reading.
        assert_eq!(distant_clock_offset_label(0.0), "now");
        assert_eq!(distant_clock_offset_label(5e-5), "+50 \u{b5}s");
        assert_eq!(distant_clock_offset_label(-2e-3), "-2 ms");
        assert_eq!(distant_clock_offset_label(3.0), "+3 s");
        assert_eq!(distant_clock_offset_label(300.0), "+5 min");
        assert_eq!(distant_clock_offset_label(-7200.0), "-2 h");
        assert_eq!(distant_clock_offset_label(86400.0 * 3.0), "+3 d");
        assert_eq!(distant_clock_offset_label(SECONDS_PER_YEAR * 1e6), "+1e6 yr");
        // A step that lands between two units keeps one decimal rather than lying about being round.
        assert_eq!(distant_clock_offset_label(65.0 * 60.0), "+1.1 h");
    }
}
