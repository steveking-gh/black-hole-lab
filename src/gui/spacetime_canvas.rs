use crate::gui::controls::{impossible_mode_note, ReferenceFrame, SignalViews};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::geodesic::proper_time_between;
use crate::physics::local_frame::{ruler_distance, LocalFrame, SurfaceCharacter};
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

Tidal (bg) — the *background* tidal field: the gravitational acceleration difference across one metre, in Earth g per metre, from 2GM/r³ at the observer’s radius. It is the Newtonian radial stretch of a hole of this mass, so it carries no dependence on the spin and none on the observer’s motion; it is quoted because tidal stretch, not infall speed, is what tears a body apart, and because it is the part of the answer that is exact in vacuum.

What it leaves out is the part that matters at the Cauchy horizon. A real hole is perturbed — by the radiative tail of its own collapse, if by nothing else — and that perturbation is blueshifted without bound on the approach to r−, where the tidal force an infaller measures grows like (uᵗ)² divided by (ln uᵗ)⁷ while this number stays finite and nearly constant. So near r− this read-out says the ride is gentle, and for exact Kerr it is; for any hole that has ever been disturbed it is not. The singularity there is a weak one in Tipler’s sense — the integrated deformation stays bounded, because the diverging tidal force oscillates with a frequency that runs away too — but the instantaneous force does diverge, and nothing in this app models it. (Mallary, Khanna & Burko, Phys. Rev. D 98, 104024 (2018).)

ν_in/ν_∞ — frequency of ingoing light measured by the observer, divided by the frequency at infinity. Below 1 means redshift; a raindrop measures 1/2 at the Schwarzschild horizon.

E, L — conserved energy and angular momentum per unit mass along the geodesic. E = 1, L = 0 means a drop from rest at infinity.

Region tag — location relative to the horizons: outside r₊, between r₊ and r₋, or inside r₋, plus the ergosphere.";

/// What a rest-frame view of the (t, r) column is. It used to be painted across the head of that
/// canvas, above the picture; it is a statement about the chart rather than about anything moving
/// in it, so it is now read on hover from the selector that chooses the chart.
pub const REST_FRAME_TIP: &str =
"The focus observer's first-order local inertial frame, built from their orthonormal tetrad: c ≡ 1, so light cones are at 45° and every worldline through the event is steeper than that. The surfaces r = const - the horizons, the static limit, the ring singularity - are placed by the dual tetrad: exact at the observer's own event, linearised for offsets from it.";

/// Seconds in a Julian year, the unit the top of the distant clock grid's ladder is counted in.
const SECONDS_PER_YEAR: f64 = 86400.0 * 365.25;

/// Smallest on-screen gap, in points at `font_scale` = 1, that `distant_clock_grid_step` will
/// leave between two neighbouring lines of the distant clock grid. Below this the lines stop being
/// readable as separate slices and start being a smear, so the ladder is climbed instead. Under
/// the rule in `distant_clock_grid_step` the gap the user actually sees is this, give or take the
/// rounding up to a round step, for the whole fall: it is set by the zoom and nothing else.
pub const MIN_GRID_PX: f32 = 28.0;

/// Tightest window the rest-frame view will open to, in M of the local chart's xi. The gap an
/// observer freezing onto the far branch of r- has left when `geodesic::U_T_STALL` stops the
/// worldline is about 1e-10 M, and r - r- is resolved in f64 down to an ulp of r, about 1e-16 M,
/// so a floor of 1e-12 M leaves the whole of the approach the integrator can be trusted for on the
/// zoomable side of it while staying four decades clear of the arithmetic's own floor.
const FRAME_MAX_R_MIN: f64 = 1e-12;

/// The rest-frame view's default window, and the widest that the automatic framing will open it to.
/// The framing only ever tightens, so an observer with nothing close ahead of them is drawn at the
/// scale the view has always opened on.
const FRAME_MAX_R_DEFAULT: f64 = 5.5;

/// Where up the canvas the automatic framing puts the surface it is framing, as a fraction of the
/// half-height. Three tenths leaves the light cone below it room to be a cone.
const FRAME_SURFACE_FRACTION: f32 = 0.30;

/// How much of the way to the wanted window one frame of the automatic zoom travels, as an exponent
/// on the ratio of the two. Log-lerping rather than snapping keeps the picture from strobing: on
/// the approach to r- the gap closes by a factor of e every 1/kappa_- of coordinate time, which at
/// a = 0.9 is 2.6 M, and a zoom that jumped to each frame's answer would jitter at exactly that
/// rate.
const FRAME_ZOOM_LERP: f64 = 0.15;

/// How many fine zoom steps one event of the coarse zoom - ctrl, or command, with the wheel - is
/// worth. egui smooths a notch of the wheel over a few frames, so twenty per frame works out at
/// about a decade of zoom per notch of the hand, and the range between the view the app opens on
/// and an observer's last femtoseconds is eleven decades: at the fine step alone that is a thousand
/// scroll events.
pub(crate) const COARSE_ZOOM_STEPS: i32 = 20;

/// Hover gloss on the automatic framing.
pub const KEEP_SURFACE_FRAMED_TIP: &str =
"Re-derive the rest-frame view's zoom every frame so that the next surface the observer meets - the outer horizon, the Cauchy horizon, or the ring - stays on the canvas.

A surface r = const crosses the observer's own time axis at xi^0 = (r - r_h) / |dr/dtau| exactly: the proper time they have left before they reach it. The framing follows that one number, so it needs no special case for the last moments - and on the approach to the far branch of r-, where the gap closes like exp(-kappa_- t) while the observer's remaining proper time shrinks with it, it is the only way to see the two of them at once.

It only ever tightens the view. Any turn of the wheel switches it off.";

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
/// Microseconds and milliseconds are in the middle of it because for a 10 solar-mass hole one M
/// of coordinate time is 49 microseconds, and outside the hole u^t is of order 1, so the grid an
/// exterior observer wants is measured in tens of microseconds.
///
/// It runs on down to femtoseconds for the approach to r-. An observer asymptoting to the far
/// branch of the Cauchy horizon is a finite and *shrinking* proper time from it - the r- line
/// crosses their own time axis at exactly Delta r / u^r (see `LocalFrame::surface_r_const`) - and
/// at the point where `geodesic::U_T_STALL` stops the worldline that is about two femtoseconds for
/// a ten solar-mass hole. A ladder that stopped at the microsecond could not put a single line
/// between them and the horizon there, which is the one place in the app where the number is the
/// whole story.
const CLOCK_UNITS: [(f64, &[f64], &str); 9] = [
    (1e-15, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0], "fs"),
    (1e-12, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0], "ps"),
    (1e-9, &[1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0], "ns"),
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
    /// takes. Lines are drawn at t = t_obs + k * step_m for integer k. This is the *distant*
    /// clock's reading between two lines, and it is not a round number: it is the round step on
    /// the observer's own clock carried across by u^t, so it runs away as they fall.
    pub step_m: f64,
    /// The observer's *own* proper time between two neighbouring lines, named in its own unit -
    /// "50 µs", "5 min", "1e6 yr". This is the round one: `step_m / u^t` in seconds.
    pub label: String,
}

/// How far apart to put the lines of the distant clock grid, chosen as a round step on the
/// *observer's own* clock: the smallest rung of the ladder 1, 2, 5 x {us, ms, s, min, hr, day, yr},
/// then decades of years, that leaves at least `MIN_GRID_PX` * `font_scale` points between
/// neighbouring lines.
///
/// The lines are the surfaces t = const of the chart's Killing time, and
/// `LocalFrame::surface_t_const` puts consecutive ones, `step_m` apart in t, exactly
/// `step_m / u^t` of the observer's proper time apart on their worldline. The drawn plane carries
/// `px_per_m` points per M of xi, so that proper-time separation *is* the on-screen gap: the rung
/// has to satisfy `step_m / u_t >= MIN_GRID_PX * font_scale / px_per_m`, and the answer is
/// returned as `step_m = rung * u_t`.
///
/// u^t therefore drops out of the choice. The rung depends on the zoom, the font scale and the
/// hole's mass and on nothing else, so it is fixed for a whole fall and changes only when the user
/// turns the wheel: the grid is a ruler on the observer's watch, at a constant pixel pitch, and it
/// never needs a mid-fall change of unit. What runs away instead is what each line is worth on the
/// distant clock - as an observer falls toward the far branch of r- their u^t grows like
/// exp(kappa_- t), and `step_m` grows with it, continuously and without bound, which is what the
/// line labels and the head of the canvas report.
///
/// (Both numbers cannot be round at once: their ratio is u^t, which slides. Quantising the far
/// side instead - the earlier rule - made the labels round at the cost of a proper-time step that
/// never sat still and a pixel pitch that sawtoothed between `MIN_GRID_PX` and 2.5 times it.)
///
/// `seconds_per_m` converts the hole's own time unit M into seconds (for a 10 solar mass hole,
/// 4.9e-5 s), so the same ladder serves a stellar-mass hole and a quasar.
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
    // The smallest readable step on the observer's own clock: min_px / px_per_m M of their proper
    // time, converted to seconds, which is the ladder's currency.
    let needed_seconds = min_px / px_per_m * seconds_per_m;

    let ladder = distant_clock_ladder();
    let (seconds, label) = ladder
        .iter()
        .find(|(seconds, _)| *seconds >= needed_seconds)
        // Off the top of the ladder: the coarsest rung there is, with the lines closer together
        // than `MIN_GRID_PX`. The labelling guard in the view thins the labels out in that case.
        .unwrap_or_else(|| ladder.last().expect("the ladder is never empty"));
    // Back to coordinate time, which is what `surface_t_const` takes: a proper-time step of
    // `seconds` is u^t times as much of t.
    GridStep {
        step_m: seconds / seconds_per_m * u_t,
        label: label.clone(),
    }
}

/// A compact, signed reading of the distant clock `seconds` away from the observer's now: the label
/// on one line of the grid. "+5 min", "-2 h", "+1e6 yr", "+20 fs", and "now" for the slice through
/// the observer's own event.
pub(crate) fn distant_clock_offset_label(seconds: f64) -> String {
    if seconds == 0.0 || !seconds.is_finite() {
        return "now".to_string();
    }
    let sign = if seconds < 0.0 { "-" } else { "+" };
    format!("{sign}{}", duration_label(seconds))
}

/// A duration in seconds, printed in whatever unit reads naturally and with no sign: the same
/// ladder `distant_clock_offset_label` ticks the grid with, from femtoseconds to years, so a
/// horizon's countdown and the clock lines beside it are quoted in the same units.
fn duration_label(seconds: f64) -> String {
    if !seconds.is_finite() {
        return "—".to_string();
    }
    let s = seconds.abs();
    let (value, unit) = if s < 1e-12 {
        (s * 1e15, "fs")
    } else if s < 1e-9 {
        (s * 1e12, "ps")
    } else if s < 1e-6 {
        (s * 1e9, "ns")
    } else if s < 1e-3 {
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
    format!("{number} {unit}")
}

/// A proper distance in units of M, printed in whatever unit reads naturally. The ruler distance
/// to a horizon spans microns on a stellar-mass hole and light-seconds on a supermassive one, and
/// `KerrSchild::format_physical_distance` bottoms out at a tenth of a kilometre, so the small end
/// of the ladder is carried here and everything from a kilometre up is handed back to it.
fn ruler_distance_label(metric: &KerrSchild, r_in_m: f64) -> String {
    let km = metric.r_to_km(r_in_m.abs()).abs();
    if !km.is_finite() {
        return "—".to_string();
    }
    if km >= 1.0 {
        return metric.format_physical_distance(r_in_m.abs());
    }
    let metres = km * 1e3;
    if metres >= 1.0 {
        format!("{metres:.2} m")
    } else if metres >= 1e-3 {
        format!("{:.2} mm", metres * 1e3)
    } else if metres >= 1e-6 {
        format!("{:.2} µm", metres * 1e6)
    } else if metres >= 1e-9 {
        format!("{:.2} nm", metres * 1e9)
    } else {
        format!("{metres:.2e} m")
    }
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
        let lines = telemetry_lines(name, color, obs, metric, use_km);
        let size = telemetry_box_size(painter, &lines, font_scale);
        let anchored = default_badge_pos(canvas_rect, pos, size);
        self.show_lines(
            ui,
            painter,
            canvas_tag,
            name,
            canvas_rect,
            anchored,
            &lines,
            color,
            font_scale,
            TELEMETRY_HOVER_TIP,
        )
    }

    /// Whether this box has been dragged somewhere and left there.
    pub fn is_placed(&self, canvas_tag: &str, name: &str) -> bool {
        self.placements.contains_key(&format!("{canvas_tag}:{name}"))
    }

    /// Draw one box of arbitrary content at `anchored`, with the drag, the double-click reset and
    /// the remembered placement every box on these canvases gets. `show` is this with an
    /// observer's telemetry in it; the surface and horizon boxes are this with theirs.
    #[allow(clippy::too_many_arguments)]
    fn show_lines(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        canvas_tag: &str,
        name: &str,
        canvas_rect: Rect,
        anchored: Pos2,
        lines: &[TelemetryLine],
        color: Color32,
        font_scale: f32,
        tip: &'static str,
    ) -> egui::Response {
        let key = format!("{canvas_tag}:{name}");
        let previous = self.placements.get(&key).copied();
        let size = telemetry_box_size(painter, lines, font_scale);
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

        paint_telemetry_box(painter, badge_rect, color, lines, font_scale);
        response.on_hover_text(tip)
    }
}

/// An info box worked out while its subject was being drawn and painted only once everything else
/// on the canvas is down. Deferring them is what makes the opaque fill mean anything: a box painted
/// in the middle of the pass gets a worldline, a light cone or a grid line drawn straight across it.
struct PendingBox {
    /// Also the key its remembered position is filed under, so it has to be unique per canvas.
    name: String,
    anchor: Pos2,
    lines: Vec<TelemetryLine>,
    color: Color32,
    tip: &'static str,
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

/// Corner radius of every floating info box, in points before the font scale. Large enough that
/// the rounding reads as a deliberate shape rather than as an anti-aliased square corner.
const BOX_CORNER_RADIUS: f32 = 8.0;

/// The two sizes a telemetry box prints at. They are now the same size: 10 pt is the floor for
/// text anywhere in this app, and the title is told apart by colour and position rather than by
/// being the only legible row.
fn telemetry_fonts(font_scale: f32) -> (egui::FontId, egui::FontId) {
    (
        egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
        egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
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

    painter.rect_filled(badge_rect, BOX_CORNER_RADIUS * font_scale, Color32::from_black_alpha(230));
    painter.rect_stroke(
        badge_rect,
        BOX_CORNER_RADIUS * font_scale,
        Stroke::new(1.2, color),
        egui::StrokeKind::Inside,
    );

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

/// One surface drawn across the rest-frame view: its radius, the title of its box, the colour and
/// width of its line, the colour its box is outlined in, and whether that box is a horizon's - the
/// two horizons report a place-or-moment reading, the static limit and the ring report the causal
/// character of the drawn line.
type DrawnSurface<'a> = (f64, &'a str, Color32, f32, Color32, bool);

/// Hover tip for the two horizon boxes, which name the methodology their third line uses.
/// Hover tip for the static limit's and the ring's boxes, whose two lines read the drawn line
/// rather than integrating anything.
pub const SURFACE_BOX_TIP: &str =
"What the surface is, read straight off the slope of its line in this frame. Steeper than 45 degrees is timelike - the world-tube of observers holding that radius, something a rocket can stay off. Exactly 45 degrees is null. Flatter than 45 degrees is spacelike: not a place at all but a moment of your history, which arrives whatever you do. Nothing about the tilt is put in by hand; it comes from the sign of g^rr at your own radius through the dual tetrad, so the reading is exact at the dot.

tilt — the angle of the line, quoted so that a 41.8 degree surface is not mistaken for a 45 degree one.

moving at / closing at — for a timelike surface, the speed its world-tube crosses this frame at; for a spacelike one, your speed relative to the observers whose simultaneity slice it is. Both are read off the same slope.

ξ¹ and τ — where the line cuts your axes: how far out the surface is at your own moment, and when it meets your worldline. Both are marked (1st order) because this chart is linear. The tetrad is exact at the dot, so orientations there are exact, but a finite offset is a straight-line extrapolation through curved geometry and reads low - measured against the exact integral it is short by about 7% for an infaller at 4.5M, by 38% for a hoverer at 4M, and by 63% deep inside r₋. The two horizons do not use it: their boxes integrate the geodesic and the worldline instead, and carry no such tag.";

/// The Cauchy box's border. Red rather than the magenta of the r₋ line itself: the line is the
/// geometry, the box is the warning.
const HORIZON_BOX_RED: Color32 = Color32::from_rgb(255, 60, 60);


pub const HORIZON_BOX_TIP: &str =
"Whether a horizon is a place or a moment is not a matter of taste: it is whether r is spacelike or timelike between you and it.

Outside r₊ and inside r₋ the metric function Δ = r² − 2Mr + a² is positive, r is an ordinary radial direction, and a surface r = const is a timelike world-tube — something that persists, that you can hold station beside, and that has a distance. Between the horizons Δ < 0, r is timelike, and the surface is a moment of your history instead: it arrives, it cannot be hovered beside, and asking how far away it is has no answer. The read-out flips at each horizon because the geometry does.

Ruler Distance — the arclength of the spacelike geodesic that leaves your event along your own radial axis and runs until it meets the surface: the radial coordinate of Fermi normal coordinates built on your tetrad. That is the length contraction done exactly. It is not the static observers' chain of rulers divided by your Lorentz factor, which rescales somebody else's ruler and answers a different question, and unlike that chain it still exists inside the ergosphere, where nothing can hold station to lay rulers out. It does assume a simultaneity — your own — because \"how far away is it right now\" cannot be asked without one.

Time — the proper time on your own watch between here and the crossing, ∫ r² dr / √R with R = r⁴(dr/dτ)², integrated along the worldline your E and L put you on. R is a square and never changes sign, which is why a horizon has a time even where it has no distance, while the distance integral carries a √Δ that goes imaginary throughout Region II.

\"beyond r₊\" — the path from here to r₋ would have to cross Region II, so no spacelike curve in your rest space reaches it and the integral has nothing to return. It is not on your worldline yet either, and whether it ever will be depends on what you do next.";

/// The three lines of a horizon's box: what the surface is called, whether it is a place or a
/// moment from where this observer stands, and the one number that reading admits.
///
/// The place-or-moment test is region adjacency, not the local causal character the drawn line
/// carries. `LocalLine::character` reads the tilt of r = const at the *observer's* radius and so
/// returns the same answer for both horizons; what decides whether a spacelike path from the
/// observer reaches a particular surface is whether Delta stays positive over the whole interval
/// between them, i.e. whether that interval avoids Region II. Outside r+ that holds for r+ and
/// fails for r-; inside r- it holds for r- and fails for r+; in Region II it fails for both, which
/// is exactly the set of cases where the surface is on the worldline instead.
fn horizon_box_lines(
    metric: &KerrSchild,
    obs: &Observer,
    obs_name: &str,
    title: &str,
    r_h: f64,
    color: Color32,
) -> Vec<TelemetryLine> {
    let rp = metric.outer_horizon();
    let rm = metric.inner_horizon();
    let r = obs.r;

    // Delta > 0 over the whole interval: the one condition under which a spacelike path from the
    // observer's rest space reaches the surface at all.
    let is_place = (r > rp && r_h >= rp) || (r < rm && r_h <= rm);

    let (kind, detail) = if (r - r_h).abs() < 1e-12 {
        ("On it".to_string(), "crossing now".to_string())
    } else if is_place {
        match ruler_distance(metric, r, &obs.four_velocity(metric), r_h) {
            Some(d) => (
                "Place".to_string(),
                format!("{} from {}", ruler_distance_label(metric, d), obs_name),
            ),
            None => (
                "Place".to_string(),
                "no spacelike path reaches it from here".to_string(),
            ),
        }
    } else if r > rp {
        // r- seen from outside r+. The path to it crosses Region II, where the distance integral
        // goes imaginary, and it is not on this worldline yet either - a static observer never
        // reaches it at all - so neither number exists.
        (
            "Place".to_string(),
            format!("beyond r₊ — no distance from {obs_name}"),
        )
    } else {
        // Inside r+, so the surface is on this worldline and has a time. Everything inside r+ is
        // falling, so the surface above the observer is the one already crossed.
        let past = r_h > r;
        let frozen = obs.is_frozen();
        let tau = obs
            .geodesic
            .and_then(|g| proper_time_between(metric, g.energy, g.l_ang, r, r_h));
        let when = match tau {
            Some(t) => {
                let secs = (t / metric.m.max(1e-12)) * metric.t_grav_seconds();
                let tail = if past { "past" } else { "future" };
                let note = if frozen && !past { " (frozen)" } else { "" };
                format!("{} in {}'s {}{}", duration_label(secs), obs_name, tail, note)
            }
            None => {
                let tail = if past { "past" } else { "future" };
                format!("in {}'s {}", obs_name, tail)
            }
        };
        ("Moment, not a place".to_string(), when)
    };

    vec![
        TelemetryLine { text: title.to_string(), color, is_title: true },
        TelemetryLine { text: kind, color: Theme::TEXT_BRIGHT, is_title: false },
        TelemetryLine { text: detail, color: Theme::TEXT_BRIGHT, is_title: false },
    ]
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

    // "bg" for background: this is the vacuum field of the hole alone, and the perturbation it
    // leaves out is the whole of the Cauchy horizon singularity. The hover tip says so at length;
    // the label is there so that nobody reads a mild number near r- as a promise. See
    // `TELEMETRY_HOVER_TIP`.
    let tidal_str = if a_tidal_grad >= 1e6 {
        format!("Tidal (bg) = {:.2e} g/m", a_tidal_grad)
    } else if a_tidal_grad >= 100.0 {
        format!("Tidal (bg) = {:.0} g/m", a_tidal_grad)
    } else if a_tidal_grad >= 0.01 {
        format!("Tidal (bg) = {:.2} g/m", a_tidal_grad)
    } else {
        format!("Tidal (bg) = {:.2e} g/m", a_tidal_grad)
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
        "Region I"
    } else if obs.r > rp {
        "Ergo"
    } else if obs.r > rm {
        "Region II (Trapped)"
    } else {
        "Region III (Core)"
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

/// The drawn radius of each observer's marker on the (t, r) diagram. They are not pickable here -
/// a marker is dragged on the equatorial view, which draws the plane they actually stand in - so
/// unlike `Who::marker_radius` these are drawing sizes and nothing else.
const ALICE_MARKER_RADIUS: f32 = 5.5;
const BOB_MARKER_RADIUS: f32 = 8.0;

pub struct SpacetimeCanvas {
    pub max_r: f64,
    pub r_offset: f64,
    pub time_window: f64,
    pub time_offset: f64,
    /// The rest-frame view's window, in M of the local chart's xi across the full width. It is its
    /// own number rather than `max_r` because the two views work at scales that have nothing to do
    /// with each other: the foliation view looks at whole M of r, while a rest frame closing on r-
    /// is interesting at 1e-10 M of xi. Sharing one would mean every trip into an observer's last
    /// femtoseconds left the foliation view at a window it can draw nothing in.
    pub frame_max_r: f64,
    /// Keep the next surface the focus observer meets framed on the rest-frame view, re-deriving
    /// the zoom every frame. See `KEEP_SURFACE_FRAMED_TIP`.
    pub keep_surface_framed: bool,
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
            frame_max_r: FRAME_MAX_R_DEFAULT,
            keep_surface_framed: true,
            telemetry: TelemetryBoxes::default(),
        }
    }
}

impl SpacetimeCanvas {
    /// Render the (t, r) spacetime foliation canvas with an integrated, perfectly aligned 1D radial track
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
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

        // Mouse wheel zoom on canvas, with ctrl (or command) held for the coarse step. That
        // gesture never arrives as a scroll: egui reads the wheel's modifiers against its own zoom
        // modifier and turns a match into `zoom_delta` instead, the same number a trackpad pinch
        // produces. It is read here for its direction only, so one notch of the coarse zoom is
        // worth exactly `COARSE_ZOOM_STEPS` of the fine one however fast the platform reports the
        // wheel.
        //
        // The two views zoom their own windows: the foliation view's is a window on r, and panning
        // it keeps the radius under the pointer where it is, while a rest frame's is a window on
        // the local chart and is centred on the observer by construction.
        if response.hovered() {
            let (zoom_delta, scroll) =
                ui.input(|i| (i.zoom_delta(), i.smooth_scroll_delta.y));
            let (inward, steps) = if (zoom_delta - 1.0).abs() > 1e-4 {
                (zoom_delta > 1.0, COARSE_ZOOM_STEPS)
            } else if scroll.abs() > 0.1 {
                (scroll > 0.0, 1)
            } else {
                (false, 0)
            };
            if steps > 0 {
                let factor =
                    if inward { 0.975f64.powi(steps) } else { 1.025f64.powi(steps) };
                match frame_of_ref {
                    ReferenceFrame::Bob | ReferenceFrame::Alice => {
                        self.frame_max_r =
                            (self.frame_max_r * factor).clamp(FRAME_MAX_R_MIN, 50.0);
                        // The user has taken the wheel, so the automatic framing stands down until
                        // they ask for it back - the same bargain the equatorial view's "Keep
                        // centered" makes with a drag.
                        self.keep_surface_framed = false;
                    }
                    ReferenceFrame::DistantObserver => {
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
            }
        }

        // The rest-frame views need one observer to be the frame and take the other, if there is
        // one, as a guest. Either may be missing, so the frame the user asked for falls back to
        // whoever is left, and with nobody left there is no rest frame to draw at all.
        let (bob, alice) = (bob, alice);
        match frame_of_ref {
            ReferenceFrame::Bob | ReferenceFrame::Alice => {
                painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);
                let (asked, other) = match frame_of_ref {
                    ReferenceFrame::Alice => (alice, bob),
                    _ => (bob, alice),
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
                    bob,
                    alice,
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
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0 km", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::SINGULARITY_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::SINGULARITY_LINE);
            }
        }
        if line_x_rm >= t_rect.left() && line_x_rm <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rm, t_rect.top()), Pos2::new(line_x_rm, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_CAUCHY));
            if use_km {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₋={}", metric.format_km(metric.r_to_km(rm))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_CAUCHY);
            } else {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Cauchy r₋", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_CAUCHY);
            }
        }
        if line_x_rp >= t_rect.left() && line_x_rp <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rp, t_rect.top()), Pos2::new(line_x_rp, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_OUTER));
            if use_km {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₊={}", metric.format_km(metric.r_to_km(rp))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_OUTER);
            } else {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Outer r₊", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_OUTER);
            }
        }
        if line_x_re >= t_rect.left() && line_x_re <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_re, t_rect.top()), Pos2::new(line_x_re, t_rect.bottom())], Stroke::new(1.5, Theme::ERGOSPHERE_LINE));
            if use_km {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r_E={}", metric.format_km(metric.r_to_km(re))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::ERGOSPHERE_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "r_E", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::ERGOSPHERE_LINE);
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
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Theme::TEXT_MUTED,
        );

        let center_y = t_rect.center().y + 2.0;

        // Draw Alice on Track
        if let Some(al) = alice
            && al.is_active
        {
            let al_x = track_to_x(al.r);
            t_painter.circle_filled(Pos2::new(al_x, center_y), 6.0, Theme::ALICE_COLOR);
            t_painter.text(Pos2::new(al_x, center_y - 10.0), egui::Align2::CENTER_BOTTOM, "Alice", egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale), Theme::ALICE_COLOR);
        }

        // Draw Bob on Track
        if let Some(bob) = bob.filter(|b| b.is_active) {
            let bob_x = track_to_x(bob.r);
            t_painter.circle_filled(Pos2::new(bob_x, center_y), 6.5, Theme::BOB_COLOR);
            t_painter.circle_stroke(Pos2::new(bob_x, center_y), 8.5, Stroke::new(1.0, Color32::WHITE));
            t_painter.text(Pos2::new(bob_x, center_y + 9.0), egui::Align2::CENTER_TOP, "Bob", egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale), Theme::BOB_COLOR);
        }

        // Radial separation between Alice and Bob, if both are present
        if let (Some(al), Some(bob)) = (alice, bob)
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
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
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
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        current_time: f64,
        use_km: bool,
        font_scale: f32,
        signals: SignalViews<'_>,
    ) {
        let t_min = current_time + self.time_offset - self.time_window * 0.7;
        let t_max = current_time + self.time_offset + self.time_window * 0.3;

        // The radial projection, in locals rather than read through `self`, so that the closures
        // built on it do not hold a borrow of the canvas for as long as they live. They are taken
        // before the background pan, so every projection in this frame is the one the user is
        // looking at as they press: a pan applied to the closures mid-frame would move the picture
        // out from
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
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
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
                            egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
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
                            egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
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
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
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
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Color32::from_rgb(135, 185, 255),
        );

        // Boundary lines
        //
        // The two horizons carry the same info box the rest-frame view gives them. What a horizon
        // *is* - a place with a distance, or a moment on somebody's clock - is a statement about an
        // observer, not about the chart, so the box is drawn against one and every line of it names
        // whoever it is quoting. Bob if he is on the canvas, otherwise Alice; with neither there is
        // nothing to read and the plain label stands.
        let horizon_reader = bob.or(alice);
        let mut pending_boxes: Vec<PendingBox> = Vec::new();

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
            if let Some(obs) = horizon_reader {
                let box_lines =
                    horizon_box_lines(metric, obs, &obs.name, &rm_label, rm, HORIZON_BOX_RED);
                pending_boxes.push(PendingBox {
                    // A stable key, not the title: the title carries the radius and so changes with
                    // the Mass and Spin sliders, which would lose a box the user had dragged.
                    name: "Cauchy Horizon".to_string(),
                    anchor: Pos2::new(x_rm_actual + 4.0, rect.top() + 42.0),
                    lines: box_lines,
                    color: HORIZON_BOX_RED,
                    tip: HORIZON_BOX_TIP,
                });
            } else {
                painter.text(
                    Pos2::new(x_rm_actual + 4.0, rect.top() + 42.0),
                    egui::Align2::LEFT_TOP,
                    rm_label,
                    egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
                    Theme::HORIZON_CAUCHY,
                );
            }
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
            if let Some(obs) = horizon_reader {
                let box_lines =
                    horizon_box_lines(metric, obs, &obs.name, &rp_label, rp, Theme::HORIZON_OUTER);
                pending_boxes.push(PendingBox {
                    // A stable key, not the title: the title carries the radius and so changes with
                    // the Mass and Spin sliders, which would lose a box the user had dragged.
                    name: "Outer Horizon".to_string(),
                    anchor: Pos2::new(x_rp_actual + 4.0, rect.top() + 42.0),
                    lines: box_lines,
                    color: Theme::HORIZON_OUTER,
                    tip: HORIZON_BOX_TIP,
                });
            } else {
                painter.text(
                    Pos2::new(x_rp_actual + 4.0, rect.top() + 58.0),
                    egui::Align2::LEFT_TOP,
                    rp_label,
                    egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
                    Theme::HORIZON_OUTER,
                );
            }
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
                egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
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
        // a box beats the canvas's own pan response instead of panning the diagram.
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
                    painter.circle_filled(alice_pos, ALICE_MARKER_RADIUS, Theme::ALICE_COLOR);
                    painter.circle_stroke(alice_pos, 7.5, Stroke::new(1.0, Color32::WHITE));
                    alice_box = Some(alice_pos);
                }
            }
        }

        // Bob Worldline & Dragging
        if let Some(bob) = bob.filter(|b| b.trail.len() >= 2) {
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


        // Mouse drag background panning for time and radial offset. It takes effect on the next
        // frame, since this frame's projections were fixed above.
        if response.dragged() {
            let delta = response.drag_delta();
            let dt = (delta.y as f64 / rect.height() as f64) * (t_max - t_min);
            self.time_offset += dt;
            let dr = (delta.x as f64 / rect.width() as f64) * max_r;
            self.r_offset = (self.r_offset - dr).max(0.0);
        }

        // Everything left on this canvas is Bob's: his worldline, his light cone, his marker and
        // the two boxes read off him. With no Bob in the simulation there is none of it.
        let Some(bob) = bob else {
            if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
                self.telemetry.show(
                    ui, painter, "spacetime", rect, alice_pos, "Alice", Theme::ALICE_COLOR, al,
                    metric, use_km, font_scale,
                );
            }
            return;
        };

        let bob_radius = BOB_MARKER_RADIUS;

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
        painter.circle_filled(apex, bob_radius, Theme::BOB_COLOR);
        painter.circle_stroke(apex, bob_radius + 2.0, Stroke::new(1.5, Color32::WHITE));

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
            egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
            Theme::TEXT_BRIGHT,
        );

        // Every info box on this canvas is painted here, after everything else is down, so that
        // the opaque fill of a box actually blocks out what is behind it.
        self.flush_pending_boxes(ui, painter, "spacetime", rect, &pending_boxes, font_scale);

        if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
            self.telemetry.show(
                ui, painter, "spacetime", rect, alice_pos, "Alice", Theme::ALICE_COLOR, al, metric, use_km,
                font_scale,
            );
        }
        self.telemetry.show(
            ui, painter, "spacetime", rect, apex, "Bob", Theme::BOB_COLOR, bob, metric, use_km, font_scale,
        );
    }

    /// Paint every deferred info box, in order, once the rest of the canvas is down.
    ///
    /// A box still sitting where it was put is nudged down clear of the ones already placed, so a
    /// stack of surfaces whose lines all leave by the same corner stays readable. One the user has
    /// dragged is left exactly where they dragged it, overlap or not: they can see the overlap and
    /// they chose it.
    fn flush_pending_boxes(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        canvas_tag: &str,
        rect: Rect,
        pending: &[PendingBox],
        font_scale: f32,
    ) {
        let mut occupied: Vec<Rect> = Vec::new();
        for b in pending {
            let size = telemetry_box_size(painter, &b.lines, font_scale);
            let anchor = if self.telemetry.is_placed(canvas_tag, &b.name) {
                b.anchor
            } else {
                let mut r = clamp_into(Rect::from_min_size(b.anchor, size), rect);
                for _ in 0..8 {
                    if !occupied.iter().any(|p| p.intersects(r)) {
                        break;
                    }
                    r = clamp_into(r.translate(Vec2::new(0.0, size.y + 6.0)), rect);
                }
                r.min
            };
            let response = self.telemetry.show_lines(
                ui, painter, canvas_tag, &b.name, rect, anchor, &b.lines, b.color, font_scale, b.tip,
            );
            occupied.push(response.rect);
        }
    }

    /// The rest-frame window that puts the next surface the observer meets `FRAME_SURFACE_FRACTION`
    /// of the way up the canvas, or `None` when there is nothing ahead of them to frame.
    ///
    /// A surface r = r_h crosses the observer's own time axis at xi^0 = (r_h - r) / u^r - set
    /// xi^1 = 0 in the line `LocalFrame::surface_r_const` returns and everything else cancels - and
    /// that is exactly the proper time they have left before they reach it. The smallest positive
    /// one of those is the surface they are about to meet: the ring for a raindrop, r+ for anyone
    /// still outside it, r- for a worldline with E - Omega_- L < 0 freezing onto the far branch.
    ///
    /// Nothing in the rule knows about horizons or about how far down the fall is: the same
    /// expression asks for a window of M early on and of femtometres of r on the approach to r-,
    /// where the observer's remaining proper time is femtoseconds and the gap is nine decades
    /// below anything the view has ever had to draw.
    pub(crate) fn framed_window(metric: &KerrSchild, obs: &Observer, rect: Rect) -> Option<f64> {
        let u_r = obs.four_velocity(metric)[1];
        if !u_r.is_finite() || u_r == 0.0 || rect.height() <= 1.0 || rect.width() <= 1.0 {
            return None;
        }
        let ahead = [metric.outer_horizon(), metric.inner_horizon(), 0.0]
            .into_iter()
            .map(|r_h| (r_h - obs.r) / u_r)
            .filter(|xi0| xi0.is_finite() && *xi0 > 0.0)
            .fold(f64::INFINITY, f64::min);
        if !ahead.is_finite() {
            return None;
        }
        // xi^0 = ahead is to land FRAME_SURFACE_FRACTION of the half-height above the centre, and
        // the view carries rect.width() / window * 0.45 points per M of xi.
        let px_per_m = (FRAME_SURFACE_FRACTION * rect.height() * 0.5) as f64 / ahead;
        let window = (rect.width() * 0.45) as f64 / px_per_m;
        Some(window.clamp(FRAME_MAX_R_MIN, FRAME_MAX_R_DEFAULT))
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
        // Keep the next surface the observer meets on the canvas, if the user has not taken the
        // wheel. The window follows from one number and the same rule serves the whole fall, so
        // there is no threshold anywhere in this and no special case for the last moments.
        if self.keep_surface_framed
            && let Some(target) = Self::framed_window(metric, focus_obs, rect)
        {
            let current = self.frame_max_r.max(FRAME_MAX_R_MIN);
            self.frame_max_r = current * (target / current).powf(FRAME_ZOOM_LERP);
        }

        let center = rect.center();
        let scale = (rect.width() / (self.frame_max_r as f32).max(1e-12)) * 0.45;
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

        // The head of the canvas: what one grid line is worth on each of the two clocks, and the
        // ratio of those. It is laid out here, where the numbers are, and painted at the very end
        // so that nothing covers it; `head_bottom` is where everything else may start.
        //
        // The first reading is the round step on the observer's own clock, fixed by the zoom and
        // held for the whole fall (see `distant_clock_grid_step`); the second is that step times
        // u^t, and is the one that runs away. Their ratio *is* u^t = dt/dtau, exactly - and once
        // the two are in different units, 10 ms against 251 ms or 1 fs against 10 us, no eye
        // compares them, so the number the picture actually turns on is the one to print. It is set
        // in the same size as the pair it stands for, and its two sides carry their colours, so the
        // eye pairs "1" with the observer's reading and the ratio with the distant one.
        let obs_color =
            if focus_obs.name == "Alice" { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        let (head_lines, head_bottom) = if show_distant_clock_grid
            && clock_proper_step.is_finite()
            && clock_proper_step > 0.0
        {
            let font = egui::FontId::proportional(16.0 * font_scale);
            let distant = distant_clock_offset_label(clock_grid.step_m * seconds_per_m);
            let ratio = if !u_t.is_finite() {
                "∞".to_string()
            } else if u_t < 1e4 {
                format!("{u_t:.1}")
            } else {
                format!("{u_t:.2e}")
            };
            let font2 = font.clone();
            let lines = vec![
                vec![
                    painter.layout_no_wrap(
                        format!("{} on {}'s clock", clock_grid.label, focus_obs.name),
                        font.clone(),
                        obs_color,
                    ),
                    painter.layout_no_wrap("  =  ".to_string(), font.clone(), Theme::TEXT_MUTED),
                    painter.layout_no_wrap(
                        format!("{} on the distant clock", distant.trim_start_matches('+')),
                        font.clone(),
                        Theme::TEXT_BRIGHT,
                    ),
                ],
                vec![
                    painter.layout_no_wrap("1".to_string(), font2.clone(), obs_color),
                    painter.layout_no_wrap(" : ".to_string(), font2.clone(), Theme::TEXT_MUTED),
                    painter.layout_no_wrap(ratio, font2.clone(), Theme::TEXT_BRIGHT),
                    painter.layout_no_wrap(
                        "   (dt/dτ)".to_string(),
                        font2,
                        Theme::TEXT_MUTED,
                    ),
                ],
            ];
            let height: f32 = lines
                .iter()
                .map(|line| line.iter().map(|g| g.rect.height()).fold(0.0, f32::max))
                .sum();
            (lines, rect.top() + 4.0 + height)
        } else {
            (Vec::new(), rect.top() + 18.0)
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
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                    label_colour,
                );
            }
        }

        // 2b. The observer's own clock, ticked up his own worldline.
        //
        // The surface t = t_obs + k * step_m crosses xi^1 = 0 at xi^0 = k * step_m / u^t - set
        // xi^1 = 0 in the line `LocalFrame::surface_t_const` returns and everything else cancels -
        // so the grid's own lines cut this axis at exact multiples of the round step the rung was
        // chosen to be, and the axis can be ticked with the same numbers the grid is spaced by.
        // They are the reading the grid's labels give, taken on the other clock: at the scale the
        // automatic framing settles on near r- they are in femtoseconds, and the r- line's crossing
        // of this same axis - at Delta r / u^r, the proper time left before it - is then read
        // straight off them.
        //
        // Drawn whether or not the distant grid is, because this is the observer's own clock rather
        // than the distant one's, and each label names its own unit.
        if clock_proper_step.is_finite() && clock_proper_step > 0.0 {
            let spacing_px = (clock_proper_step as f32) * scale;
            if spacing_px >= 1.0 {
                let k_max = ((rect.height() * 0.5 / spacing_px).ceil() as i64).clamp(0, 512);
                let label_every = ((CLOCK_LABEL_MIN_PX * font_scale / spacing_px).ceil() as i64).max(1);
                for k in -k_max..=k_max {
                    // k = 0 is the observer's own event, which is already a dot with their name
                    // beside it.
                    if k == 0 {
                        continue;
                    }
                    let y = center.y - (k as f32) * spacing_px;
                    // Stop two ticks short of the top edge. The head banner sits there, and a
                    // label that runs right up to it lands on the banner's own text.
                    if y < rect.top() + 2.0 + 2.0 * spacing_px || y > rect.bottom() - 2.0 {
                        continue;
                    }
                    painter.line_segment(
                        [Pos2::new(center.x - 4.0, y), Pos2::new(center.x + 4.0, y)],
                        Stroke::new(1.2, obs_color),
                    );
                    if k % label_every != 0 {
                        continue;
                    }
                    painter.text(
                        Pos2::new(center.x + 7.0, y),
                        egui::Align2::LEFT_CENTER,
                        distant_clock_offset_label((k as f64) * clock_proper_step * seconds_per_m),
                        egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                        obs_color,
                    );
                }
            }
        }

        // 3. Surfaces r = const, every one of them placed by the dual tetrad.
        let ergo_faint = Color32::from_rgba_unmultiplied(
            Theme::ERGOSPHERE_LINE.r(),
            Theme::ERGOSPHERE_LINE.g(),
            Theme::ERGOSPHERE_LINE.b(),
            80,
        );
        // The last field is the info box the surface gets, as (title, border colour). Only the two
        // horizons have one: they are the surfaces whose reading changes kind when the observer
        // crosses them. The static limit and the ring keep the plain three-line label, which says
        // the same things about a surface whose character never flips.
        let surfaces: [DrawnSurface; 4] = [
            (
                metric.ergosphere_equatorial(),
                "Static Limit 2M",
                ergo_faint,
                1.4,
                Theme::ERGOSPHERE_LINE,
                false,
            ),
            (
                metric.outer_horizon(),
                "Outer Horizon r₊",
                Theme::HORIZON_OUTER,
                2.5,
                Theme::HORIZON_OUTER,
                true,
            ),
            (
                metric.inner_horizon(),
                "Cauchy Horizon r₋",
                Theme::HORIZON_CAUCHY,
                2.5,
                HORIZON_BOX_RED,
                true,
            ),
            (
                0.0,
                "Ring Singularity r = 0",
                Theme::SINGULARITY_LINE,
                3.0,
                Theme::SINGULARITY_LINE,
                false,
            ),
        ];

        // Worked out here, where the line geometry is; painted at the end of the pass.
        let mut pending_boxes: Vec<PendingBox> = Vec::new();

        for (idx, &(r_h, title, color, width, border, is_horizon)) in surfaces.iter().enumerate() {
            let line = frame.surface_r_const(r_h);
            let anchor = to_screen(line.point[0], line.point[1]);
            let dir = Vec2::new(line.dir[0] as f32, -(line.dir[1] as f32));
            let Some((end_a, end_b)) = clip_line_to_rect(anchor, dir, rect) else {
                continue;
            };
            painter.line_segment([end_a, end_b], Stroke::new(width, color));

            let (label_pos, align) = if line.slope().abs() >= 1.0 {
                // Steep line: hang the label off it, stacked down the top margin.
                let y = (head_bottom + 8.0 + 36.0 * font_scale * (idx as f32)).min(rect.bottom() - 40.0);
                let x = segment_x_at_y(end_a, end_b, y)
                    .clamp(rect.left() + 6.0, rect.right() - 150.0 * font_scale);
                (Pos2::new(x + 5.0, y), egui::Align2::LEFT_TOP)
            } else {
                // Flat line: park the label on it at the right margin.
                let y = segment_y_at_x(end_a, end_b, rect.right() - 10.0)
                    .clamp(rect.top() + 24.0, rect.bottom() - 6.0);
                (Pos2::new(rect.right() - 8.0, y - 3.0), egui::Align2::RIGHT_BOTTOM)
            };

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

            // Every surface answers in a box now. A horizon reports what it is to this observer -
            // a place with a distance, or a moment on his own clock - and the other two report the
            // causal character of their line as drawn, which is the same question asked of a
            // surface whose answer never changes.
            let box_lines = if is_horizon {
                horizon_box_lines(metric, focus_obs, &focus_obs.name, title, r_h, border)
            } else {
                vec![
                    TelemetryLine { text: title.to_string(), color: border, is_title: true },
                    TelemetryLine { text: note.to_string(), color: Theme::TEXT_BRIGHT, is_title: false },
                    TelemetryLine { text: detail, color: Theme::TEXT_BRIGHT, is_title: false },
                ]
            };
            // `label_pos` is a corner of the text the box stands in for; make it the box's own.
            let size = telemetry_box_size(painter, &box_lines, font_scale);
            let min = if align == egui::Align2::RIGHT_BOTTOM {
                Pos2::new(label_pos.x - size.x, label_pos.y - size.y)
            } else {
                label_pos
            };
            pending_boxes.push(PendingBox {
                name: title.to_string(),
                anchor: min,
                lines: box_lines,
                color: border,
                tip: if is_horizon { HORIZON_BOX_TIP } else { SURFACE_BOX_TIP },
            });
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
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                focus_edge,
            );
            painter.text(
                p_fut_in + Vec2::new(-4.0, -2.0),
                egui::Align2::RIGHT_BOTTOM,
                "-45° Ingoing",
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                focus_edge,
            );
        } else if focus_obs.is_active {
            painter.circle_filled(center, 12.0, Theme::SINGULARITY_FILL);
            painter.circle_stroke(center, 15.0, Stroke::new(2.0, Theme::SINGULARITY_LINE));
            painter.text(
                Pos2::new(center.x + 18.0, center.y),
                egui::Align2::LEFT_CENTER,
                "💥 SINGULARITY IMPACT\nLight cone terminated at r = 0",
                egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
                Theme::WARNING_RED,
            );
        }

        // No outline. A white ring round the dot said nothing the dot did not already say, and the
        // dot sits on the origin of the axes here, where an extra stroke is only clutter.
        painter.circle_filled(apex, 7.5, obs_color);

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
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                    Theme::TEXT_MUTED,
                );

                other_box = Some((other, other_pos, other_color));
            }
        }


        // The comparison is centred, which keeps it clear of the grid's own line labels down the
        // left edge. Every row under it is left-aligned to where that line starts rather than
        // centred on its own width: centred it would sit on the observer's clock ticks, which run
        // up the middle of the canvas beside his axis, and pushed out to the margin it would sit on
        // the distant grid's labels, which run down the left edge.
        let mut y = rect.top() + 4.0;
        let mut left = rect.left() + 8.0;
        for (row, line) in head_lines.into_iter().enumerate() {
            let total: f32 = line.iter().map(|g| g.rect.width()).sum();
            let height = line.iter().map(|g| g.rect.height()).fold(0.0, f32::max);
            if row == 0 {
                left = (rect.center().x - total * 0.5).max(rect.left() + 4.0);
            }
            let mut x = left;
            for galley in line {
                let width = galley.rect.width();
                painter.galley(Pos2::new(x, y), galley, Color32::WHITE);
                x += width;
            }
            y += height;
        }

        // Every info box on this canvas goes here, after the head banner and everything else, so
        // that the opaque fill of a box blocks out what is behind it and dragging one wins over
        // the canvas's own drag response. The surfaces first, the observers over them.
        self.flush_pending_boxes(ui, painter, "restframe", rect, &pending_boxes, font_scale);
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
    fn test_the_distant_clock_grid_step_is_a_round_step_on_the_observers_own_clock() {
        // The rung is chosen on the falling observer's own clock, not on the distant one: the gap
        // between two lines on screen is their proper-time separation times the pixel scale, so the
        // rule is `step_m / u^t >= MIN_GRID_PX * font_scale / px_per_m` and u^t cancels out of it.
        // Four things follow, and are checked here: the ladder has no holes, the rung does not
        // depend on u^t at all, the drawn gap is readable, and a coarser view asks for a coarser
        // rung. What does run away is `step_m` itself - what one line is worth on the distant clock
        // - and that is checked to grow exactly in proportion to u^t.

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
        // 2. Twenty decades of u^t - the whole runaway, from a raindrop outside the hole to the far
        //    branch of r- - and the observer's own step between two lines never changes. This is the
        //    property the rule exists for: the grid is a fixed ruler on their watch, so it never
        //    needs a change of unit mid-fall and the lines never breathe.
        let at_rest = distant_clock_grid_step(1.0, px_per_m, secs, 1.0);
        let mut worst_gap = f64::INFINITY;
        let mut widest_gap = 0.0f64;
        for i in 0..=200 {
            let u_t = 10.0f64.powf(i as f64 * 0.1);
            let step = distant_clock_grid_step(u_t, px_per_m, secs, 1.0);
            assert_eq!(
                step.label, at_rest.label,
                "the rung moved to {} at u^t = {u_t:e}: it must not depend on u^t",
                step.label
            );
            // 3. What each line is worth on the distant clock is that same proper step carried
            //    across by u^t, so it grows exactly in proportion. That is the runaway, and it is
            //    now carried by the labels instead of by the spacing.
            let proper_step = step.step_m / u_t;
            assert!(
                (proper_step - at_rest.step_m).abs() <= 1e-9 * at_rest.step_m,
                "step_m must be the proper step times u^t: {proper_step} vs {} at u^t = {u_t:e}",
                at_rest.step_m
            );
            // 4. Readable: never closer together on screen than MIN_GRID_PX, and - because the
            //    ladder has no holes - never more than 2.5 times that far apart either.
            let gap = proper_step * (px_per_m as f64);
            assert!(gap >= MIN_GRID_PX as f64, "lines {gap} px apart (step {})", step.label);
            assert!(
                gap <= 2.5 * MIN_GRID_PX as f64 * (1.0 + 1e-9),
                "lines {gap} px apart (step {}): the ladder has a hole",
                step.label
            );
            worst_gap = worst_gap.min(gap);
            widest_gap = widest_gap.max(gap);
        }

        // 5. The zoom is the only thing that moves the rung, and it moves it the right way: winding
        //    the view out - fewer points per M - never buys a finer step on the observer's clock.
        let mut previous = 0.0f64;
        let mut walk: Vec<String> = Vec::new();
        for e in (-12..=12).rev() {
            let zoom = 10.0f32.powi(e);
            let step = distant_clock_grid_step(1.0, zoom, secs, 1.0);
            assert!(
                step.step_m >= previous * (1.0 - 1e-9),
                "winding out from {previous} M gave a finer step {} at {zoom} px/M",
                step.step_m
            );
            previous = step.step_m;
            walk.push(step.label);
        }
        // For one hole, the pixel scale alone walks the ladder from microseconds at the deepest
        // zoom up to the days at the shallowest. The span swept here is wider than the wheel's own
        // range, which is the point: the rung is a function of the scale, and no unit in that stretch
        // of the ladder is out of reach for want of the right u^t.
        for unit in [" fs", " ps", " ns", " \u{b5}s", " ms", " s", " min", " hr", " day"] {
            assert!(
                walk.iter().any(|l| l.ends_with(unit)),
                "no rung in{unit} across the swept range of pixel scales: {walk:?}"
            );
        }
        // The top of the ladder belongs to the supermassive end instead, where one M of the chart's
        // own time is an hour and more: a 1e9 solar-mass hole (4900 s per M) wound right out wants a
        // line every few million years of the faller's own time.
        let supermassive = distant_clock_grid_step(1.0, 1e-9, 4900.0, 1.0);
        assert!(
            supermassive.label.ends_with(" yr"),
            "the top of the ladder is years, got {}",
            supermassive.label
        );

        // 6. The view the app actually opens on: the frame view runs at about 57 points per M at
        //    the default zoom and one M is 49 µs for a ten solar-mass hole, so Bob gets a line every
        //    50 µs of his own time - and a 500 point tall canvas has to hold a run of them, not one.
        let exterior = distant_clock_grid_step(1.5, 57.0, TEN_SOLAR_SECONDS_PER_M, 1.0);
        let spacing_px = exterior.step_m / 1.5 * 57.0;
        let fit = (500.0 / spacing_px).floor() as i32;
        assert_eq!(exterior.label, "50 \u{b5}s");
        assert!(
            fit >= 6,
            "only {fit} lines of {} fit a 500 px view ({spacing_px:.1} px apart)",
            exterior.label
        );
        println!(
            "distant clock grid: {} rungs, widest step ratio {worst_ratio:.3} ({} -> {}); u^t over \
             1e0..1e20 leaves the step at {} throughout, on-screen gap between {worst_gap:.2} and \
             {widest_gap:.2} px (MIN_GRID_PX = {MIN_GRID_PX}); zoom 1e12..1e-12 px/M -> {walk:?}; at \
             the default view the step is {} = {:.4} M of Bob's own time, {spacing_px:.1} px apart, \
             {fit} lines in a 500 px view",
            ladder.len(),
            worst_pair.0,
            worst_pair.1,
            at_rest.label,
            exterior.label,
            exterior.step_m / 1.5
        );

        // The bottom of the ladder is what the approach to r- needs. When `geodesic::U_T_STALL`
        // stops a worldline freezing onto the far branch it has about 1e-10 M of r left, which for
        // a ten solar-mass hole is a couple of femtoseconds of the faller's own time; the automatic
        // framing puts that on the canvas at around 1.8e12 points per M, and the rung wanted there
        // is under one femtosecond. A ladder stopping at the microsecond would answer with a single
        // line ten decades off the canvas.
        let last_moments = distant_clock_grid_step(1e10, 1.8e12, TEN_SOLAR_SECONDS_PER_M, 1.0);
        assert!(
            last_moments.label.ends_with(" fs"),
            "the bottom of the ladder is femtoseconds, got {}",
            last_moments.label
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

    /// One real frame of `SpacetimeCanvas::render` in Bob's rest frame, with the distant clock grid
    /// on or off, returning how many line segments were painted and the text of every galley.
    fn frame_view_pass(show_distant_clock_grid: bool) -> (usize, String) {
        use crate::physics::observer::{Observer, WorldlineParams};
        use crate::physics::wavefront::SignalField;

        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        bob.step(&metric, 0.1, 0.1);
        let alice: Option<Observer> = None;
        let signal = SignalField::default();
        // Zoomed in on the frame, so that the chosen rung of the ladder sits close to its own
        // minimum gap and a whole run of lines lands inside the rectangle rather than one. The
        // automatic framing is off: this test is about the grid, and the framing would set the
        // zoom from Bob's distance to the next surface instead.
        let mut canvas = SpacetimeCanvas {
            frame_max_r: 0.6,
            keep_surface_framed: false,
            ..Default::default()
        };

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
                Some(&bob),
                alice.as_ref(),
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
        // observer differ by the grid: the lines themselves, and the reading at the head of the
        // canvas that says what one of them is worth on each of the two clocks. Nothing else in the
        // view moves, because the grid is read off `LocalFrame::surface_t_const` and touches no
        // physics.
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
            !off_text.contains("on the distant clock"),
            "the reading at the head of the canvas belongs to the grid and goes with it"
        );
        // The two numbers are laid out as separate galleys so that each can carry its own colour,
        // so they are looked for one at a time rather than as one sentence.
        assert!(
            on_text.contains("on Bob's clock"),
            "the head of the canvas must say what one line is worth on Bob's own clock: {on_text}"
        );
        assert!(
            on_text.contains("on the distant clock"),
            "and what the same line is worth on a clock at rest at infinity: {on_text}"
        );
        assert!(on_text.contains("now"), "the slice through the observer's own event is labelled");
        // Bob's own clock is ticked up his own worldline whether or not the distant grid is shown:
        // it is his clock, not the distant one's, and each tick names its own unit. Nothing else in
        // the view writes a signed number at the head of a line.
        assert!(
            off_text
                .lines()
                .any(|l| l.starts_with(['+', '-']) && l[1..].starts_with(|c: char| c.is_ascii_digit())),
            "the observer's own clock axis goes on being ticked with the grid off: {off_text}"
        );
    }

    #[test]
    fn test_the_distant_clocks_slices_cut_the_observers_own_axis_at_his_round_step() {
        // What lets the observer's own time axis be ticked with the grid's own spacing. The surface
        // t = t_obs + k * step_m meets xi^1 = 0 at xi^0 = k * step_m / u^t exactly - the same
        // proper-time step `distant_clock_grid_step` chose the rung to be - so the ticks up the
        // axis and the lines across the canvas are one set of events, read on the two clocks.
        //
        // Checked on a worldline deep in the runaway, u^t = 1e5, where the step in t and the step
        // in proper time are five decades apart and any confusion between them would be loud.
        use crate::physics::local_frame::LocalFrame;
        use crate::physics::observer::{Observer, WorldlineParams};

        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob = Observer::new_with_phi(
            &metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::new(1.0, 2.2, false),
        );
        let mut guard = 0;
        while bob.four_velocity(&metric)[0] < 1e5 && !bob.has_ended() && guard < 20_000 {
            guard += 1;
            bob.step(&metric, 0.0, 0.05);
        }
        let u = bob.four_velocity(&metric);
        assert!(u[0] >= 1e5, "the sample wants a runaway u^t, got {}", u[0]);

        let frame = LocalFrame::for_observer(&metric, bob.r, &u);
        let step = distant_clock_grid_step(u[0], 3.0e5, TEN_SOLAR_SECONDS_PER_M, 1.0);
        let proper_step = step.step_m / u[0];
        for k in 1..=5 {
            let xi0 = frame
                .surface_t_const((k as f64) * step.step_m)
                .xi0_at_axis()
                .expect("a t = const slice is never parallel to the observer's own axis");
            let wanted = (k as f64) * proper_step;
            assert!(
                (xi0 - wanted).abs() <= 1e-9 * wanted.abs(),
                "slice {k} cuts the axis at {xi0} M, wanted {wanted} M"
            );
        }
        println!(
            "at u^t = {:.3e} the grid puts {} of Bob's own time between lines, which is {:.3e} M of \
             the chart's t, or {:.3e} s on the distant clock of a ten solar-mass hole",
            u[0],
            step.label,
            step.step_m,
            step.step_m * TEN_SOLAR_SECONDS_PER_M
        );
    }

    #[test]
    fn test_the_automatic_framing_follows_the_proper_time_to_the_next_surface() {
        use crate::physics::observer::{Observer, WorldlineParams};

        let metric = KerrSchild::new(1.0, 0.90);
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(600.0, 500.0));
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();

        // What the rule is: with the window it returns, the framed surface lands
        // FRAME_SURFACE_FRACTION of the way up the half-height. The surface r = r_h crosses the
        // observer's own time axis at xi^0 = (r_h - r) / u^r, so that is the number checked here -
        // the same expression `LocalFrame::surface_r_const` puts on the canvas.
        let lands_at = |obs: &Observer, r_h: f64, window: f64| -> f32 {
            let scale = (rect.width() / window as f32) * 0.45;
            let xi0 = (r_h - obs.r) / obs.four_velocity(&metric)[1];
            xi0 as f32 * scale
        };
        let wanted_px = FRAME_SURFACE_FRACTION * rect.height() * 0.5;

        // 1. Just above the outer horizon, the next surface is r+ and not the two below it: a
        //    raindrop at r = 1.5 M is 0.047 M of proper time from r+ and 0.7 M from r-.
        let mut near = Observer::new_with_phi(
            &metric, "Bob", 0.0, 1.5, 0.0, 0.0, WorldlineParams::default(),
        );
        near.step(&metric, 0.0, 1e-6);
        let window = SpacetimeCanvas::framed_window(&metric, &near, rect)
            .expect("a falling observer always has a surface ahead of them");
        assert!(
            (lands_at(&near, rp, window) - wanted_px).abs() < 0.5,
            "r+ landed at {} px, wanted {wanted_px}",
            lands_at(&near, rp, window)
        );
        assert!(
            lands_at(&near, rm, window) > wanted_px,
            "r- is further ahead than r+ and must be framed looser, not tighter"
        );

        // 2. Far out, where nothing is close, the framing stands at the view's default: it only
        //    ever tightens, so the picture the app opens on is the one it has always opened on.
        let mut far = Observer::new_with_phi(
            &metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default(),
        );
        far.step(&metric, 0.0, 1e-6);
        assert_eq!(
            SpacetimeCanvas::framed_window(&metric, &far, rect),
            Some(FRAME_MAX_R_DEFAULT)
        );

        // 3. The case it exists for. E = 1, L = 2.2 at a = 0.90 has E - Omega_- L < 0, so this
        //    worldline never crosses r-: it freezes onto the far branch, and `geodesic::U_T_STALL`
        //    stops it with the gap at about 1e-10 M. The framing is asked for no special case and
        //    given none - the same expression that wanted 5.5 M at r = 4.5 wants a window nine
        //    decades below that here - and at the scale it asks for, the clock ladder's own rule
        //    picks a rung in femtoseconds.
        let mut falling = Observer::new_with_phi(
            &metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::new(1.0, 2.2, false),
        );
        let mut guard = 0;
        while !falling.has_ended() && guard < 20_000 {
            guard += 1;
            falling.step(&metric, 0.0, 0.05);
        }
        assert!(falling.has_ended(), "the worldline must reach the freeze; r = {}", falling.r);
        let gap = falling.r - rm;
        let u = falling.four_velocity(&metric);
        let tau_left = gap / u[1].abs();
        let window = SpacetimeCanvas::framed_window(&metric, &falling, rect)
            .expect("the frozen worldline still has r- ahead of it");
        let scale = (rect.width() / window as f32) * 0.45;
        let rung = distant_clock_grid_step(u[0], scale, TEN_SOLAR_SECONDS_PER_M, 1.0);
        println!(
            "frozen at r - r- = {gap:.3e} M with u^t = {:.3e}, dr/dtau = {:.3}: {tau_left:.3e} M of \
             Bob's own time left ({:.2e} s for a ten solar-mass hole); the framing asks for a \
             {window:.3e} M window, {scale:.3e} px/M, and the ladder answers {}",
            u[0],
            u[1],
            tau_left * TEN_SOLAR_SECONDS_PER_M,
            rung.label
        );
        assert!(
            (lands_at(&falling, rm, window) - wanted_px).abs() < 0.5,
            "r- landed at {} px, wanted {wanted_px}",
            lands_at(&falling, rm, window)
        );
        assert!(
            window < 1e-8,
            "the last moments need a window nine decades below the default, got {window:e}"
        );
        assert!(
            rung.label.ends_with(" fs"),
            "at the framed scale the grid is a femtosecond ruler, got {}",
            rung.label
        );
    }

    #[test]
    fn test_a_distant_clock_label_names_the_offset_in_its_own_unit_with_a_sign() {
        // The label on one line: which way it is from the observer's now, and how far in the
        // largest unit that leaves a number worth reading.
        assert_eq!(distant_clock_offset_label(0.0), "now");
        assert_eq!(distant_clock_offset_label(2e-15), "+2 fs");
        assert_eq!(distant_clock_offset_label(-5e-13), "-500 fs");
        assert_eq!(distant_clock_offset_label(2e-11), "+20 ps");
        assert_eq!(distant_clock_offset_label(1e-7), "+100 ns");
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
