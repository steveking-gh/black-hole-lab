use crate::gui::axis::{self, SECONDS_PER_YEAR};
use crate::gui::controls::{impossible_mode_note, ReferenceFrame, SignalViews};
use crate::gui::polyline::{SCREEN_SPACING, thin_to_pixels};
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::geodesic::proper_time_between;
use crate::physics::local_frame::{ruler_distance, LocalFrame, SurfaceCharacter};
use crate::physics::observer::{LocalRestFrame, LocalSpeed, Observer, ObserverMode, Who};
use crate::physics::wavefront::{NullRay, Reception, SignalField};
use egui::{epaint::PathShape, Color32, Pos2, Rect, Stroke, Vec2};
use std::collections::HashMap;

/// Plain-language gloss on every number in a telemetry box, shown on hover.
/// Hover tip for the observer info boxes. Written as a plain multi-line literal (lines start at
/// column 0 so no indentation leaks into the text).
pub const TELEMETRY_HOVER_TIP: &str =
"Drag: move the box anywhere on the canvas. On the equatorial view the box then holds that screen position while the observer moves on; on the (t, r) diagram the box keeps a fixed offset from the observer. Double-click: snap the box back to the observer. Each canvas remembers box positions per observer.

dr/dt — map speed, in c: how fast the marker crosses the (t, r) chart per tick of the chart's shared clock. Far from the hole this rate equals what a distant observer measures. Near the hole the chart runs on a clock that lets infall and light cross the horizon without freezing, so the number means something only against the drawn light wedge.

dr/dτ — how fast the radial coordinate changes per unit of the observer's own proper time: kilometres of radius per second of that observer's watch, or M of r per M of τ with the units box ticked. Not a speed, and deliberately not labelled in c. Two owners split the two halves of this ratio — the chart owns r, the observer owns τ — so the value passes 1 on any deep infall and reaches −2.68 on a worldline asymptoting to r₋ while nothing moves faster than light. The watch runs slow, and r undercounts stretched space. Inside r₊ the radial coordinate is not even spacelike, which makes the quantity a countdown rate rather than a velocity. For speeds that really are speeds, read the v_ rows.

Ω — dϕ/dt: how fast the observer goes round the hole per unit of the chart's shared clock, signed, prograde positive. Radians per second, or inverse M with the units box ticked. The bracketed figure alongside is the local frame-dragging rate ω = −g_tϕ/g_ϕϕ, defined as the Ω of the zero-angular-momentum observer at that radius: the rate at which the hole turns space itself there. Far from the hole the two rates have nothing to do with each other and ω falls off as 2Ma/r³. Inside the static limit r = 2M every timelike worldline must share the hole's sign of Ω, however hard that worldline thrusts — the ergosphere is that fact, not the horizon. On r₋ the dragging reaches Ω₋ = a/(r₋² + a²), and a frozen worldline rides round at exactly Ω₋, the rate the Frozen line names.

One warning about reading Ω as which way something goes round. The app charts in ingoing Kerr-Schild coordinates, whose ϕ shears with radius — the metric's rϕ cross term g_rϕ = −a(1 + 2M/r) does not vanish — so for any worldline with dr ≠ 0 the chart's dϕ/dt departs from the angular velocity a distant observer would infer, by exactly (a/Δ)(dr/dt). A raindrop dropped from rest at infinity carries precisely zero angular momentum, and the hole drags that raindrop prograde, yet at r = 4.5M with a = 0.90 this line reads Ω = −0.0138/M: the Boyer-Lindquist dϕ/dt is +0.0265/M, and the chart's own twist, −0.0403/M at that infall speed, is the steeper of the two. Nothing orbits backwards. The invariant statement is the L printed lower down the box — L = 0 means zero angular momentum whatever dϕ/dt says. For an observer holding r the shear contributes nothing, so Ω and ω are both chart-independent there and the comparison between the two is exact. A worldline frozen on r₋ is the clean case, where Ω and ω agree to every printed digit, because that worldline rides the horizon's own generator.

v_stat, v_ZAMO, v_rain — how fast the observer moves past a local observer standing at the same event, as that local observer measures the motion. Each row prints three numbers, and the three do not share one ruler or one clock. Take Alice on the prograde ISCO at a = 0.90, whose row reads v_stat = 0.898c (γ 2.27, γv 2.04c):

• v — the measurer's ruler and the measurer's clock. Alice crosses 0.898 light-seconds of the hovering observer's ruler per second of the hovering observer's clock. One frame owns both halves, so v stays below c everywhere.

• γv, the celerity — the measurer's ruler and the *moving* observer's clock. Alice crosses 2.04 light-seconds of the hovering observer's ruler per second of her own watch. Two clocks split the ratio, so γv carries no c bound and outruns nothing: γv is momentum per unit mass, p/m, and dr/dτ above is the radial part of the same idea.

• γ — how much slower the moving observer's clock runs than the measurer's. One second on Alice's watch costs 2.27 seconds on the hovering observer's.

No row reports Alice's speed on her own ruler against her own watch. By her own ruler and her own watch Alice sits still — a rest frame means exactly that — and the hovering observer sweeps past her at the same 0.898c her row prints.

No row reports a distant observer's measurement either. A distant observer cannot measure a local speed at all, for want of a shared frame, and the chart rates above are not speeds. What distant observers actually receive is light, which the ν_in/ν_∞ row below reports. Alice's clock against the *distant* clock is a third quantity again, uᵗ = dt/dτ = 2.70 at that ISCO, drawn by the rest-frame view's distant-clock grid. Beware of assembling uᵗ out of γ: multiplying γ = 2.27 by the hovering observer's own dilation of 2.689 gives 6.1, which is wrong, because that hovering observer is itself moving relative to the ZAMO and the two boosts do not simply multiply. The decomposition that does work runs through the ZAMO and the lapse, γ_ZAMO/α = 1.280/0.4749 = 2.696.

Every v_ row comes from the one invariant the pair of worldlines shares, γ = −g(u, u_frame), and never from a difference of coordinate rates. That is what holds each row below c at every radius and in every region, horizons included, however far the chart rates above have run away.

Which rows appear depends on who exists at that event to do the measuring, because a velocity with no second worldline named is not a quantity. Outside the static limit r = 2M two hovering observers exist and the box quotes both: the static observer, who holds ϕ fixed and so stays at rest with respect to the distant stars, and the ZAMO, who holds the same radius but goes along with the dragging.

The two rows disagree sharply, and the gap between the two *is* the frame dragging. Alice, on that ISCO at r = 2.32M, passes the ZAMO at 0.625c. The static observer holds ϕ fixed against space the hole sweeps round at ω = 0.1125/M, which costs real thrust, and therefore passes that same ZAMO at 0.622c retrograde. Add the two speeds relativistically: (0.625 + 0.622)/(1 + 0.625 × 0.622) = 0.898c, exactly what v_stat prints. So v_stat carries Alice's orbital motion plus the dragging the static observer resists, and v_ZAMO carries the orbital motion with the dragging divided out. Neither row is the more correct one.

Inside the static limit no rocket holds ϕ fixed, so the static row drops out and the ZAMO becomes the only hovering observer left. Inside r₊ nothing holds a radius at all, and the box measures against the raindrop, the observer dropped from rest at infinity, which is the one frame that survives at every r > 0. An infaller close to that congruence therefore reads a near-zero speed in there, which is the honest statement that nothing remains to be moving relative to. Where a row approaches c the printed v stops at >0.9999c rather than rounding to a flat 1.000c, and γ carries the magnitude.

Two closed forms pin these rows, and the tests check both: with no spin an orbiter on the ISCO at r = 6M passes the static observer at exactly 0.5c with γ = 2/√3, and a raindrop passes a static observer at r at exactly √(2M/r), the Newtonian escape speed.

a_prop — defined as the accelerometer reading, in Earth g. Zero means free fall.

Tidal (bg) — defined as the *background* tidal field: the gravitational acceleration difference across one metre, in Earth g per metre, from 2GM/r³ at the observer’s radius. That expression gives the Newtonian radial stretch of a hole of this mass, so the figure carries no dependence on the spin and none on the observer’s motion. The box reports tidal stretch because tidal stretch, not infall speed, tears a body apart, and because vacuum makes this part of the answer exact.

What the figure leaves out is the part that matters at the Cauchy horizon. Every real hole carries a perturbation — the radiative tail of that hole's own collapse, if nothing else — and the approach to r− blueshifts the perturbation without bound, until the tidal force an infaller measures grows like (uᵗ)² divided by (ln uᵗ)⁷ while this figure stays finite and nearly constant. So near r− this read-out promises a gentle ride. Exact Kerr keeps the promise; any hole that has ever suffered a disturbance breaks the promise. The singularity there stays weak in Tipler’s sense — the integrated deformation remains bounded, because the diverging tidal force oscillates at a frequency that runs away too — but the instantaneous force does diverge, and this app models none of that. (Mallary, Khanna & Burko, Phys. Rev. D 98, 104024 (2018).)

ν_in/ν_∞ — defined as the frequency of ingoing light that the observer measures, divided by the frequency of the same light at infinity. Below 1 means redshift; a raindrop measures 1/2 at the Schwarzschild horizon.

E, L — defined as the conserved energy and angular momentum per unit mass along the geodesic. E = 1 with L = 0 marks a drop from rest at infinity. Both stay in M whichever way the units box is set, because the panel's L slider sets L in M and because L is a specific angular momentum rather than a distance.

Region tag — where the observer stands relative to the surfaces: outside r₊, between r₊ and r₋, or inside r₋, plus the ergosphere.";

/// What a rest-frame view of the (t, r) column is. It used to be painted across the head of that
/// canvas, above the picture; it is a statement about the chart rather than about anything moving
/// in it, so it is now read on hover from the selector that chooses the chart.
/// The one line both charts of the global foliation carry at the top centre, in white: the
/// distinction that decides what a viewer may read off the picture.
pub const CHART_BANNER: &str = "This is a chart, not a frame of reference.";

pub const REST_FRAME_TIP: &str =
"The focus observer's first-order local inertial frame, built from their orthonormal tetrad: c ≡ 1, so light cones stand at 45° and every worldline through the event stands steeper. The dual tetrad places the surfaces r = const - the horizons, the static limit, the ring singularity - exactly at the observer's own event, and linearised for offsets away from that event.";

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
"Re-derive the rest-frame view's zoom every frame, so that the next surface the observer meets - the outer horizon, the Cauchy horizon, or the ring - stays on the canvas.

A surface r = const crosses the observer's own time axis at xi^0 = (r - r_h) / |dr/dtau| exactly: the proper time the observer has left before reaching that surface. The framing follows that one number, which is why the framing needs no special case for the last moments. On the approach to the far branch of r-, where the gap closes like exp(-kappa_- t) while the observer's remaining proper time shrinks alongside, following that number is the only way to watch both at once.

The zoom only ever tightens the view. Any turn of the wheel switches this setting off.";

/// Smallest vertical gap, in points at `font_scale` = 1, between two *labelled* lines of that grid.
/// A 9-point monospace row is about 12 points tall; this leaves a little air around it. It only
/// ever bites at the very top of the ladder, where no rung is coarse enough to reach `MIN_GRID_PX`
/// and the lines are squeezed anyway.
const CLOCK_LABEL_MIN_PX: f32 = 16.0;

/// The ladder this grid climbs, in ascending order, as (seconds, the step named in its own unit):
/// `axis::clock_ladder` with each rung written out as the text a legend can print - "50 µs",
/// "30 min", "1e6 yr". The rungs themselves live beside the axis rules, because the foliation
/// chart's time grid is ruled by the same table and two copies of it would drift apart.
fn distant_clock_ladder() -> Vec<(f64, String)> {
    axis::clock_ladder()
        .into_iter()
        .map(|rung| {
            let name = if rung.multiple < 1e4 {
                format!("{:.0} {}", rung.multiple, rung.unit)
            } else {
                format!("{:.0e} {}", rung.multiple, rung.unit)
            };
            (rung.seconds, name)
        })
        .collect()
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
pub(crate) enum Placement {
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

/// Which diagram a telemetry box stands on. Half of a box's identity: the same observer keeps a
/// separate dragged position on each picture, so a box is a canvas and a subject together.
///
/// `key` is how that identity is spelled outside the program - in a save file, which is what will
/// hold a user's dragged box positions between runs. Those slugs are therefore fixed once a save
/// format version has shipped: renaming a variant here, or rewording the View selector, must not
/// orphan placements somebody has already saved. Nothing on screen reads them, so the titles the
/// boxes are drawn with stay free to change.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Canvas {
    /// The (t, r) global foliation chart.
    Spacetime,
    /// An observer's own rest frame, the 1D+1 local inertial chart.
    RestFrame,
    /// The equatorial view.
    Spatial,
    /// The 2D+1 volume.
    Volume,
}

impl Canvas {
    /// Every canvas there is, for iterating the set. The reading half of the identity - this and
    /// `from_key` - is how `crate::save` comes back in: a file names a canvas by its slug, and a
    /// load turns each slug it recognises back into one of these.
    pub const ALL: [Self; 4] = [Self::Spacetime, Self::RestFrame, Self::Spatial, Self::Volume];

    /// This canvas's slug. See the type's own comment before touching one of these strings.
    pub fn key(self) -> &'static str {
        match self {
            Self::Spacetime => "spacetime",
            Self::RestFrame => "restframe",
            Self::Spatial => "spatial",
            Self::Volume => "volume",
        }
    }

    /// The canvas a slug names, or None for one this version has never written.
    pub fn from_key(key: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|c| c.key() == key)
    }
}

/// Which box on that canvas: the subject it reads, never the words it prints. The other half of a
/// box's identity, and the half that outlives every rewording of a title - the Cauchy box titles
/// itself with the current r₋ in kilometres, and is this same box whatever that says.
///
/// Its `key` carries the same promise, and the same warning, as `Canvas::key`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BoxId {
    /// One observer's telemetry, read off their own worldline.
    Observer(Who),
    /// What the named observer's transmission measures as a wave at the observer whose frame is
    /// drawn: the sender is the subject, so the box is theirs.
    Signal(Who),
    /// The outer horizon r₊, as a place or a moment for whoever the box quotes.
    OuterHorizon,
    /// The Cauchy horizon r₋, likewise.
    CauchyHorizon,
    /// The static limit, which reports the causal character of its own drawn line.
    Ergosphere,
    /// The ring singularity r = 0, likewise.
    RingSingularity,
}

impl BoxId {
    /// Every box there is, for iterating the set, read by `crate::save` exactly as `Canvas::ALL`
    /// is.
    pub const ALL: [Self; 8] = [
        Self::Observer(Who::Alice),
        Self::Observer(Who::Bob),
        Self::Signal(Who::Alice),
        Self::Signal(Who::Bob),
        Self::OuterHorizon,
        Self::CauchyHorizon,
        Self::Ergosphere,
        Self::RingSingularity,
    ];

    /// This box's slug, the payload of the two per-observer variants spelled into it. See the
    /// type's own comment before touching one of these strings.
    pub fn key(self) -> &'static str {
        match self {
            Self::Observer(Who::Alice) => "alice",
            Self::Observer(Who::Bob) => "bob",
            Self::Signal(Who::Alice) => "alice-signal",
            Self::Signal(Who::Bob) => "bob-signal",
            Self::OuterHorizon => "outer-horizon",
            Self::CauchyHorizon => "cauchy-horizon",
            Self::Ergosphere => "ergosphere",
            Self::RingSingularity => "ring-singularity",
        }
    }

    /// The box a slug names, or None for one this version has never written.
    pub fn from_key(key: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|b| b.key() == key)
    }
}

/// The remembered positions of the hovering telemetry boxes on one canvas, keyed by the canvas and
/// the box's subject so that the same observer can have a different box position in each diagram.
#[derive(Default)]
pub struct TelemetryBoxes {
    /// Visible to the crate for `crate::save`, which writes these placements to a file under the
    /// stable slugs of `Canvas::key` and `BoxId::key` and puts them back on a load.
    pub(crate) placements: HashMap<(Canvas, BoxId), Placement>,
    /// Whether a drag pins the box to the canvas where it was dropped (the equatorial view), or
    /// keeps it following the observer at the dragged offset (the (t, r) diagram, the default).
    pub(crate) pin_on_drag: bool,
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
        canvas: Canvas,
        id: BoxId,
        canvas_rect: Rect,
        pos: Pos2,
        title: &str,
        color: Color32,
        obs: &Observer,
        metric: &KerrSchild,
        use_physical_units: bool,
        font_scale: f32,
    ) -> egui::Response {
        let lines = telemetry_lines(title, color, obs, metric, use_physical_units);
        let size = telemetry_box_size(painter, &lines, font_scale);
        let anchored = default_badge_pos(canvas_rect, pos, size);
        self.show_lines(
            ui,
            painter,
            canvas,
            id,
            canvas_rect,
            anchored,
            &lines,
            color,
            font_scale,
            TELEMETRY_HOVER_TIP,
        )
    }

    /// Whether this box has been dragged somewhere and left there.
    pub fn is_placed(&self, canvas: Canvas, id: BoxId) -> bool {
        self.placements.contains_key(&(canvas, id))
    }

    /// Draw one box of arbitrary content at `anchored`, with the drag, the double-click reset and
    /// the remembered placement every box on these canvases gets. `show` is this with an
    /// observer's telemetry in it; the surface and horizon boxes are this with theirs.
    #[allow(clippy::too_many_arguments)]
    fn show_lines(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        canvas: Canvas,
        id: BoxId,
        canvas_rect: Rect,
        anchored: Pos2,
        lines: &[TelemetryLine],
        color: Color32,
        font_scale: f32,
        tip: &'static str,
    ) -> egui::Response {
        let key = (canvas, id);
        let previous = self.placements.get(&key).copied();
        let size = telemetry_box_size(painter, lines, font_scale);
        let placed = resolve_placement(previous, anchored, canvas_rect.min);
        let badge_rect = clamp_into(Rect::from_min_size(placed, size), canvas_rect);

        let widget_id = ui.id().with(("telemetry", canvas.key(), id.key()));
        // click_and_drag rather than drag alone: egui only reports a double-click on a widget that
        // senses clicks, and the double-click is what resets the offset.
        let response = ui.interact(badge_rect, widget_id, egui::Sense::click_and_drag());

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
    /// Also what its remembered position is filed under, so it has to be unique per canvas. The
    /// title the box prints is the first of its `lines` and is no part of this.
    id: BoxId,
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
    /// Set in the bold face, for a line that states a condition rather than reporting a number.
    bold: bool,
}

/// The name the bold face is registered under in `main::install_fonts`. A line asked to be bold
/// is set in it when it is registered and in the body face when it is not - a test context has
/// no fonts at all, and a family egui has never heard of is a panic, not a fallback.
pub const BOLD_FAMILY: &str = "bold";

/// The bold face at `size`, or the body face where no bold one is registered.
fn bold_font(painter: &egui::Painter, size: f32) -> egui::FontId {
    let family = egui::FontFamily::Name(BOLD_FAMILY.into());
    if painter.ctx().fonts(|f| f.families().contains(&family)) {
        egui::FontId::new(size, family)
    } else {
        egui::FontId::monospace(size)
    }
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

/// The shadow every telemetry box casts, which is what lifts it off the chart underneath.
///
/// A box is drawn over the picture it is reporting on, and it is drawn nearly black on a nearly
/// black canvas, so without something under its edge it reads as a hole in the chart rather than as
/// a card lying on it. The shadow is the cheapest way to say which.
///
/// The geometry is one light, high and to the upper left, so every box on every canvas is lit the
/// same way: `epaint`'s `margin()` works out to nothing above or to the left of the box, and ten
/// points below and to the right, from `spread + blur/2 -/+ offset`. Equal on the two lit sides is
/// what puts the light at 45 degrees, and `blur/2 == offset` is what keeps it off the other two -
/// a shadow creeping out of the top left would read as a second light rather than as depth. See
/// `test_the_box_shadow_falls_down_and_to_the_right_only`, which asserts exactly that.
///
/// It does not scale with the font, unlike the box and its type. A shadow is a statement about how
/// far the card is off the page, and the card does not rise as the type grows.
const BOX_SHADOW: egui::epaint::Shadow = egui::epaint::Shadow {
    offset: [5, 5],
    blur: 10,
    spread: 0,
    // Dark enough to read against the canvas and the region fills both, and no darker: the boxes
    // sit over the drawn worldlines, and a shadow that hid one would be reporting on the chart by
    // obscuring it.
    color: Color32::from_black_alpha(130),
};

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
            let font = if line.bold {
                bold_font(painter, Theme::MIN_FONT_PT * font_scale)
            } else if line.is_title {
                font_title.clone()
            } else {
                font_body.clone()
            };
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

    // Under the box and before it, so it falls on the chart and not on the card. Boxes can
    // overlap, and when they do the upper one's shadow lands on the lower one, which is what a
    // shadow does.
    painter.add(BOX_SHADOW.as_shape(badge_rect, BOX_CORNER_RADIUS * font_scale));
    painter.rect_filled(badge_rect, BOX_CORNER_RADIUS * font_scale, Color32::from_black_alpha(230));
    painter.rect_stroke(
        badge_rect,
        BOX_CORNER_RADIUS * font_scale,
        Stroke::new(1.2, color),
        egui::StrokeKind::Inside,
    );

    for (i, line) in lines.iter().enumerate() {
        let font = if line.bold {
            bold_font(painter, Theme::MIN_FONT_PT * font_scale)
        } else if line.is_title {
            font_title.clone()
        } else {
            font_body.clone()
        };
        painter.text(
            Pos2::new(badge_rect.left() + pad_x, badge_rect.top() + pad_y + line_spacing * i as f32),
            egui::Align2::LEFT_TOP,
            line.text.clone(),
            font,
            line.color,
        );
    }
}

/// One surface drawn across the rest-frame view: its radius, the identity of its box, the title
/// that box prints, the colour and width of its line, the colour its box is outlined in, and
/// whether that box is a horizon's - the two horizons report a place-or-moment reading, the static
/// limit and the ring report the causal character of the drawn line.
type DrawnSurface<'a> = (f64, BoxId, &'a str, Color32, f32, Color32, bool);

/// Hover tip for the two horizon boxes, which name the methodology their third line uses.
/// Hover tip for the static limit's and the ring's boxes, whose two lines read the drawn line
/// rather than integrating anything.
/// The hover text of the signal box in an observer's frame.
pub const SIGNAL_BOX_TIP: &str = "The other observer's transmission, read as a wave at this worldline. Their pulses are the crests, drawn as null strokes through the arrivals on the worldline. The receive frequency is one over the proper time between the last two arrivals of consecutive pulses, on this observer's own clock; the transmit frequency is one over the proper time between those two emissions, on the sender's clock; and the blueshift is the ratio of the two - a ratio of two measured intervals, not a formula. On the approach to r- the arrivals crowd together without limit, and the blueshift runs away with them.

An \"Incomplete\" line means the Wavefronts kept cap has evicted pulses that could still have arrived, so this box is reading a trimmed run: arrivals are missing, and a receive frequency measured across the gap they left is wrong rather than merely coarse. The count is how many went. Raise Wavefronts kept to stop losing them - the evicted ones do not come back, so a run that matters wants the cap raised before it starts. The count is exact for a receiver who stays outside r+ and a floor for one who crosses, since a crosser also meets the frozen arcs standing on r-, which this test treats as already past arriving.";

pub const SURFACE_BOX_TIP: &str =
"What the surface is, read straight off the slope of the surface's line in this frame. Steeper than 45 degrees means timelike - the world-tube of observers holding that radius, something a rocket can stay off. Exactly 45 degrees means null. Flatter than 45 degrees means spacelike: not a place at all but a moment of your history, which arrives whatever you do. Nothing about the tilt goes in by hand; the tilt follows from the sign of g^rr at your own radius through the dual tetrad, so the reading stays exact at the dot.

Moving at / closing at - for a timelike surface, the speed at which that world-tube crosses this frame; for a spacelike surface, your own speed relative to the observers whose simultaneity slice the surface is. The same slope gives both.";

/// The Cauchy box's border. Red rather than the magenta of the r₋ line itself: the line is the
/// geometry, the box is the warning.
const HORIZON_BOX_RED: Color32 = Color32::from_rgb(255, 60, 60);


pub const HORIZON_BOX_TIP: &str =
"Whether a horizon is a place or a moment is not a matter of taste: the answer is whether r runs spacelike or timelike between you and that horizon.

Outside r₊ and inside r₋ the metric function Δ = r² − 2Mr + a² stays positive, r is an ordinary radial direction, and a surface r = const is a timelike world-tube — something that persists, that you can hold station beside, and that has a distance. Between the horizons Δ < 0, r turns timelike, and that same surface becomes a moment of your history instead: the surface arrives, no rocket hovers beside the surface, and asking how far away the surface lies has no answer. The read-out flips at each horizon because the geometry flips there.

Ruler Distance — defined as the arclength of the spacelike geodesic that leaves your event along your own radial axis and runs until meeting the surface: the radial coordinate of Fermi normal coordinates built on your tetrad. That construction performs the length contraction exactly. Do not read the figure as the static observers' chain of rulers divided by your Lorentz factor, which rescales somebody else's ruler and answers a different question; unlike that chain, this geodesic survives inside the ergosphere, where nothing can hold station to lay rulers out. The figure does assume a simultaneity — your own — because the question “how far away is that surface right now” carries no meaning without one.

Time — the proper time on your own watch between here and the crossing, ∫ r² dr / √R with R = r⁴(dr/dτ)², integrated along the worldline your E and L put you on. R is a square and never changes sign, which is why a horizon has a time even where that horizon has no distance, while the distance integral carries a √Δ that goes imaginary throughout Region II.

“beyond r₊” — the path from here to r₋ would have to cross Region II, so no spacelike curve in your rest space reaches r₋ and the integral has nothing to return. r₋ does not lie on your worldline yet either, and whether r₋ ever will depends on what you do next.";

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
        TelemetryLine { text: title.to_string(), color, is_title: true, bold: false },
        TelemetryLine { text: kind, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
        TelemetryLine { text: detail, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
    ]
}



/// A dphi/dt or a dragging rate, in inverse M, signed so that prograde reads positive.
///
/// Scientific notation below a thousandth of a radian per M: far from the hole Omega falls off as
/// r^-3/2 and a fixed number of decimals would print a column of zeroes, while near r- the rates
/// that matter are of order one.
fn rate_per_m(omega: f64) -> String {
    if omega != 0.0 && omega.abs() < 1e-3 {
        format!("{:+.2e}/M", omega)
    } else {
        format!("{:+.4}/M", omega)
    }
}

/// The same rate in radians per second, for the mode that does not speak M. The span is wide -
/// a supermassive hole's ISCO turns in hours and a stellar-mass one's in milliseconds - so this
/// stays in scientific notation rather than trying to pick a scale per decade.
fn rate_per_second(per_s: f64) -> String {
    format!("{:+.3e} rad/s", per_s)
}

/// One measured speed: v, the Lorentz factor between the two worldlines, and the celerity, with
/// the frame that measured it named in the label.
///
/// v is held below 1 by construction, so the formatting has to stay honest where it gets close:
/// a worldline frozen on r- is measured against the raindrop at a gamma of order 1e10, where
/// three decimals of v would print a flat "1.000c" and say that something reached the speed of
/// light. Past four nines it prints the bound instead and leaves gamma to carry the magnitude.
fn speed_row(s: &LocalSpeed) -> String {
    let v = if s.v >= 0.9999 { ">0.9999c".to_string() } else { format!("{:.3}c", s.v) };
    let big = |x: f64| if x >= 1e3 { format!("{:.2e}", x) } else { format!("{:.2}", x) };
    // Six characters and a pad to eight, so the = lands in the same column as the rates above.
    // The names are abbreviated to hold that column; the hover tip spells all three out, and
    // which of them can appear at all is `Observer::local_speeds`.
    let label = match s.frame {
        LocalRestFrame::Static => "v_stat",
        LocalRestFrame::Zamo => "v_ZAMO",
        LocalRestFrame::Raindrop => "v_rain",
    };
    format!("{label:<8}= {v} (γ {}, γv {}c)", big(s.gamma), big(s.celerity()))
}

/// The box's contents, one metric per line.
fn telemetry_lines(
    name: &str,
    color: Color32,
    obs: &Observer,
    metric: &KerrSchild,
    use_physical_units: bool,
) -> Vec<TelemetryLine> {
    let v_c = obs.velocity_c(metric);
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
    // In c in both modes. It is the one unit on the box that needs no introduction, and the
    // km/s this used to lead with in physical mode was a six-digit number nobody reads: -161898
    // km/s says less than -0.54c does, and says it in more space. `Observer::velocity_km_s` is
    // still the conversion the tests bound against c.
    let v_coord_str = format!("dr/dt   = {:+.2}c", v_c);
    // No "c" on this one. dr/dtau is a coordinate rate against a proper time, not a speed
    // anybody measures: it runs past 1 on any deep infall and reads -2.68 on a worldline
    // asymptoting to r-, where nothing is moving faster than light and the radial coordinate is
    // not even spacelike. Printed with a c it read as an impossibility. The numbers that *are*
    // speeds, and are below c in every region and every frame, are the v_ rows below it.
    // dr/dtau is a coordinate rate against a proper time, so its physical form is kilometres
    // of radius per second of the observer's *own* clock - which is exactly why it can pass c,
    // and why the two clocks are named in the label rather than left to be inferred.
    let v_proper_str = if use_physical_units {
        format!("dr/dτ   = {:+.3e} km/s (own clock)", u_prop * 299_792.458)
    } else {
        format!("dr/dτ   = {:+.2} M/τ", u_prop)
    };

    // The chart's other rate. Omega = dphi/dt against the local dragging rate omega: outside the
    // static limit they are independent, and inside it every timelike worldline is forced to
    // share the hole's sign of Omega however hard it thrusts, which is the one number that makes
    // the ergosphere a place rather than a label.
    let omega_obs = obs.angular_velocity(metric);
    let omega_drag = metric.frame_dragging_omega(obs.r);
    let rate = |per_m: f64| {
        if use_physical_units {
            rate_per_second(metric.rate_per_second(per_m))
        } else {
            rate_per_m(per_m)
        }
    };
    let omega_str = format!("Ω       = {} (drag {})", rate(omega_obs), rate(omega_drag));

    // What a local observer actually measures for this observer's motion past them, one row per
    // observer who exists at this event to do the measuring. See `Observer::local_speeds`: both
    // hovering frames outside the static limit, the ZAMO alone through the ergosphere, and the
    // raindrop between the horizons where nothing can hover at all.
    let speed_strs: Vec<String> = obs.local_speeds(metric).iter().map(speed_row).collect();

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
        TelemetryLine { text: format!("{} [{}]", name, region_tag), color, is_title: true, bold: false },
        TelemetryLine { text: v_coord_str, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
        TelemetryLine { text: v_proper_str, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
        TelemetryLine { text: omega_str, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
        TelemetryLine { text: a_str, color: Color32::from_rgb(180, 240, 180), is_title: false, bold: false },
        TelemetryLine { text: tidal_str, color: Color32::from_rgb(255, 200, 100), is_title: false, bold: false },
        TelemetryLine {
            text: nu_str,
            color: if nu_ratio > 1.0 { Theme::BLUESHIFT_BLUE } else { Theme::TEXT_MUTED },
            is_title: false,
            bold: false,
        },
    ];
    // Inserted after the chart's own rates and before the accelerometer, so the box reads
    // outward from what the chart says to what somebody standing there measures.
    let insert_at = 4;
    for (i, text) in speed_strs.into_iter().enumerate() {
        lines.insert(
            insert_at + i,
            TelemetryLine { text, color: Theme::SPEED_MEASURED, is_title: false, bold: false },
        );
    }
    if let Some(constants_str) = constants_str {
        lines.push(TelemetryLine { text: constants_str, color: Theme::TEXT_MUTED, is_title: false, bold: false });
    }
    // A worldline frozen on the far branch of r- has not stopped: it is riding the horizon's own
    // null generator, so on the equatorial view its marker creeps round the r- circle at Omega_-
    // and in the volume it is a helix on the r- pipe, while the radius and the observer's own
    // clock stand still. Drawn, that is exactly what an ordinary orbit looks like, and this is the
    // line that tells the two apart. It is in the box rather than floating at the marker so that
    // it goes where the box goes and nothing in the picture is painted over it, and it is bold and
    // white because it is the one line here that is a state rather than a reading.
    if obs.is_frozen() {
        lines.push(TelemetryLine {
            text: "Frozen: gliding on the r₋ generator at Ω₋".to_string(),
            color: Color32::WHITE,
            is_title: false,
            bold: true,
        });
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
    /// Which surfaces' boxes are currently hung off the top margin as "steep" lines rather than
    /// parked at the right margin as "flat" ones. This is hysteresis for the session in hand and
    /// nothing a save file would ever hold; it carries a `BoxId` because the loop that draws the
    /// surfaces has one to hand, not because the key travels anywhere. A line near 45 degrees - a
    /// horizon on the approach, which is null - would otherwise flip between the two rules every
    /// frame as the drawn slope crosses 1, and its box would jump between two corners of the
    /// canvas. See `steep_with_hysteresis`.
    steep_boxes: std::collections::HashSet<BoxId>,
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
            steep_boxes: std::collections::HashSet::new(),
        }
    }
}

/// Whether a surface's box is hung off the top margin (steep line) or parked at the right margin
/// (flat line), given where it was last frame and the line's drawn |slope| now.
///
/// The rule is |slope| >= 1, with a band of hysteresis either side of it: a box that was steep
/// stays steep down to 0.8, one that was flat stays flat up to 1.25. A null surface is exactly
/// |slope| = 1, and an observer on the approach to a horizon draws one for frame after frame,
/// wobbling about 1 by less than the band; without the hysteresis the box jumped between the two
/// margins as the slope crossed 1 each way. Seen in the app on the Cauchy box.
pub(crate) fn steep_with_hysteresis(was_steep: bool, slope_abs: f64) -> bool {
    if was_steep { slope_abs >= 0.8 } else { slope_abs >= 1.25 }
}

/// Width of a wavefront comet on the global chart, in screen points.
///
/// A shade heavier than the 1.0 the old full-length wedge edges were drawn at. Those edges ran the
/// height of the canvas and forty of them overlapped; a comet is `Theme::COMET_TAIL_PX` long,
/// stands alone, and spends most of its length below half brightness, so it wants the extra tenth
/// of a point. Past about 1.5 the head stops reading as a point on a curve and starts reading as a
/// blob, which is the one thing a mark whose job is to say *where* the front is must not do.
const COMET_WIDTH: f32 = 1.2;

/// One polyline lit `head_colour` at `head` and fading linearly to nothing `fade_px` further along
/// it, as a single anti-aliased shape.
///
/// The gradient is a `PathStroke::new_uv` colour callback rather than a run of short segments at
/// stepped alpha or a quad strip built by hand. epaint tessellates a stroke into the two feathered
/// edges either side of the centre line and calls the callback once per vertex of them, then
/// interpolates the colour across the triangles between, so one shape buys a continuous fade with
/// exactly the anti-aliasing an ordinary stroke gets. A strip of stepped segments would cost a
/// shape per step, and the global chart draws up to 128 pulses in each of two fields with two
/// edges apiece; a `Mesh` written by hand would have no feathering at all, which at these widths
/// on a sloping line is the whole of what makes the line look like a line.
///
/// The callback is handed a vertex position on one of those edges, half a stroke width off the
/// centre line, and takes the straight-line distance from `head` rather than the arc length back
/// to it. On the rest-frame chart a crest is one straight segment and the two agree exactly; on the
/// global chart the tail is a piece of a null track `Theme::COMET_TAIL_PX` long, whose departure
/// from its own chord is set by how fast dr/dt turns over that piece and comes to well under a
/// point anywhere outside the last moments on the ring.
fn fading_line(
    points: Vec<Pos2>,
    head: Pos2,
    fade_px: f32,
    width: f32,
    head_colour: Color32,
) -> egui::Shape {
    let [r, g, b, a] = head_colour.to_srgba_unmultiplied();
    // Unmultiplied in, unmultiplied out: `from_rgba_unmultiplied` does the premultiplication, so
    // scaling the alpha alone here dims the line without shifting its hue toward black.
    let span = fade_px.max(1.0);
    let stroke = egui::epaint::PathStroke::new_uv(width, move |_bbox, at| {
        let lit = 1.0 - (at.distance(head) / span).clamp(0.0, 1.0);
        Color32::from_rgba_unmultiplied(r, g, b, (a as f32 * lit).round() as u8)
    });
    egui::Shape::Path(PathShape::line(points, stroke))
}

/// The head of a track as a screen polyline, cut off `max_px` back along the drawn curve.
///
/// `head_first` starts at where the front stands now and walks the track backwards from its newest
/// point. The cut is made on arc length on the screen rather than on coordinate time or on a count
/// of track points, because on this chart those are nothing like proportional to each other: an
/// edge frozen on r- packs tens of M of track into one pixel column while the outer edge of a
/// pulse far from the hole crosses the canvas over the same interval, and only a cut on the screen
/// gives every comet the same drawn length. The far point is interpolated onto the `max_px` mark,
/// so the tail ends where `Theme::COMET_TAIL_PX` says and not at whatever track point lies beyond
/// it.
///
/// Points that do not clear `SCREEN_SPACING` of the last one kept are dropped on the way, for the
/// reason `thin_to_pixels` gives, and here for a second one: a track recorded every 0.02 M of
/// coordinate time is mostly sub-pixel steps at a wide zoom, and without the thinning a single
/// comet could put a thousand vertices a hundredth of a point apart into the tessellator. The
/// dropping bounds the polyline at `max_px / SCREEN_SPACING` vertices however deep the track is.
fn comet_tail(head_first: impl Iterator<Item = Pos2>, max_px: f32) -> Vec<Pos2> {
    let mut tail: Vec<Pos2> = Vec::new();
    let mut anchor = Pos2::ZERO;
    let mut run = 0.0;
    for point in head_first {
        if tail.is_empty() {
            anchor = point;
            tail.push(point);
            continue;
        }
        let step = anchor.distance(point);
        if step < SCREEN_SPACING {
            continue;
        }
        if run + step >= max_px {
            tail.push(anchor + (point - anchor) * ((max_px - run) / step));
            break;
        }
        run += step;
        anchor = point;
        tail.push(point);
    }
    tail
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
        use_physical_units: bool,
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
                    // Both charts of the global foliation pan and zoom the same window on r. The
                    // volume is drawn on its own canvas, so it never reaches this code; the arm
                    // names it so that the match stays exhaustive rather than swallowing it.
                    ReferenceFrame::DistantObserver | ReferenceFrame::GlobalVolume => {
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
                    ReferenceFrame::Bob
                    | ReferenceFrame::DistantObserver
                    | ReferenceFrame::GlobalVolume => (bob, alice),
                };
                match (asked, other) {
                    (Some(focus), other) => self.render_observer_frame(
                        ui, &painter, rect, metric, focus, other, use_physical_units, font_scale,
                        show_distant_clock_grid, signals,
                    ),
                    (None, Some(focus)) => self.render_observer_frame(
                        ui, &painter, rect, metric, focus, None, use_physical_units, font_scale,
                        show_distant_clock_grid, signals,
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
            // The volume draws the same foliation on its own canvas and the app switches between
            // them, so this diagram is never asked for it; named rather than left to a wildcard so
            // that the choice is exhaustive.
            ReferenceFrame::DistantObserver | ReferenceFrame::GlobalVolume => {
                self.render_distant_observer(
                    ui,
                    &painter,
                    &response,
                    rect,
                    metric,
                    bob,
                    alice,
                    current_time,
                    use_physical_units,
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
            if use_physical_units {
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0 km", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::SINGULARITY_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_sing + 2.0, t_rect.bottom() - 3.0), egui::Align2::LEFT_BOTTOM, "r=0", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::SINGULARITY_LINE);
            }
        }
        if line_x_rm >= t_rect.left() && line_x_rm <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rm, t_rect.top()), Pos2::new(line_x_rm, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_CAUCHY));
            if use_physical_units {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₋={}", metric.format_km(metric.r_to_km(rm))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_CAUCHY);
            } else {
                t_painter.text(Pos2::new(line_x_rm, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Cauchy r₋", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_CAUCHY);
            }
        }
        if line_x_rp >= t_rect.left() && line_x_rp <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_rp, t_rect.top()), Pos2::new(line_x_rp, t_rect.bottom())], Stroke::new(2.5, Theme::HORIZON_OUTER));
            if use_physical_units {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r₊={}", metric.format_km(metric.r_to_km(rp))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_OUTER);
            } else {
                t_painter.text(Pos2::new(line_x_rp, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "Outer r₊", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::HORIZON_OUTER);
            }
        }
        if line_x_re >= t_rect.left() && line_x_re <= t_rect.right() {
            t_painter.line_segment([Pos2::new(line_x_re, t_rect.top()), Pos2::new(line_x_re, t_rect.bottom())], Stroke::new(1.5, Theme::ERGOSPHERE_LINE));
            if use_physical_units {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, format!("r_E={}", metric.format_km(metric.r_to_km(re))), egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::ERGOSPHERE_LINE);
            } else {
                t_painter.text(Pos2::new(line_x_re, t_rect.bottom() - 3.0), egui::Align2::CENTER_BOTTOM, "r_E", egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale), Theme::ERGOSPHERE_LINE);
            }
        }

        // Track header badge
        let track_badge = if use_physical_units {
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
            let diff_text = if use_physical_units {
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
        use_physical_units: bool,
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
            let (future_fill, past_fill, edge) = Theme::cone_colours(Who::of(obs));
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

        // Region I, [re, r_offset + max_r], is not filled: the exterior is the canvas background,
        // as it is in the volume, and the three fills above mark what is not the exterior.

        // 1. Horizontal Time Grid & Vertical Time Axis Labels
        //
        // The step follows the zoom, because this axis is zoomed across five decades - the wheel
        // clamps `time_window` to 0.005 ..= 500 M - and a grid with a floor under its step has no
        // lines at all on the tight side of that floor. The lines are anchored to the present rather
        // than to the origin of t, so that a playing run moves the picture under a grid that stands
        // still instead of dragging the grid across the canvas at (play rate)/(time window), and so
        // that each label is a short offset - "now", "+0.02M", "-80 s" - rather than an absolute
        // reading deep enough to tell one fine line from the next. The absolute clock is printed
        // once, under the now line. See `axis::TimeGrid`.
        let t_grid = axis::TimeGrid::for_window(
            current_time,
            t_min,
            t_max,
            if use_physical_units { axis::TimeUnits::Physical(metric) } else { axis::TimeUnits::M },
        );
        // The one absolute reading on this axis: the chart's clock, printed under the now line at
        // the left, where the offsets above and below are counted from. Every other label on the
        // axis is short because this one carries the long number, and it is legible however fast
        // the run plays because the now line does not move.
        //
        // The user can pan the now line off the canvas, and the clock should not go with it: the
        // reading is then pinned to the edge the present lies beyond, with an arrow saying which
        // way, and the grid labels that would sit under it give way.
        let t_now = if use_physical_units {
            format!("t = {}", metric.format_physical_time(current_time))
        } else {
            format!("t = {current_time:+.2}M")
        };
        let label_font = egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale);
        let label_height = Theme::MIN_FONT_PT * font_scale * 1.4;
        let now_y = to_screen_y(current_time);
        let now_pinned_to = if now_y < rect.top() {
            Some((rect.top() + 4.0, egui::Align2::LEFT_TOP, format!("▲ now: {t_now}")))
        } else if now_y > rect.bottom() {
            Some((rect.bottom() - 4.0, egui::Align2::LEFT_BOTTOM, format!("▼ now: {t_now}")))
        } else {
            None
        };
        // The axis text is collected here and painted later, once the light is down: the grid
        // lines belong under the wavefronts, but a transmission at its steady state fills the past
        // half of this chart edge to edge, and labels painted with the lines were washed out by it
        // - the clock reading under the now line first of all. See where `time_labels` is drained.
        let mut time_labels: Vec<(Pos2, egui::Align2, String, Color32)> = Vec::new();
        // The band of canvas a pinned reading occupies, which a grid label must keep out of.
        let pinned_band = now_pinned_to.as_ref().map(|(y, align, _)| {
            if *align == egui::Align2::LEFT_TOP {
                (*y - 2.0)..=(*y + 2.0 * label_height)
            } else {
                (*y - 2.0 * label_height)..=(*y + 2.0)
            }
        });
        for k in t_grid.ticks() {
            let y = to_screen_y(t_grid.time_of(k));
            if y >= rect.top() && y <= rect.bottom() {
                // The present gets the heavier stroke and the brighter label, so that "now" names a
                // line the eye can already pick out of its neighbours.
                let now_line = k == 0;
                painter.line_segment(
                    [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                    if now_line {
                        Stroke::new(1.2, Theme::TEXT_MUTED)
                    } else {
                        Stroke::new(Theme::GRID_LINE_WIDTH, Theme::GRID_LINE)
                    },
                );
                if pinned_band.as_ref().is_some_and(|band| band.contains(&y)) {
                    continue;
                }
                time_labels.push((
                    Pos2::new(rect.left() + 4.0, y - 2.0),
                    egui::Align2::LEFT_BOTTOM,
                    t_grid.label(k),
                    if now_line {
                        Theme::TEXT_BRIGHT
                    } else {
                        Color32::from_rgba_premultiplied(140, 165, 195, 180)
                    },
                ));
                if now_line {
                    time_labels.push((
                        Pos2::new(rect.left() + 4.0, y + 2.0),
                        egui::Align2::LEFT_TOP,
                        t_now.clone(),
                        Theme::TEXT_BRIGHT,
                    ));
                }
            }
        }
        if let Some((y, align, text)) = now_pinned_to {
            time_labels.push((Pos2::new(rect.left() + 4.0, y), align, text, Theme::TEXT_BRIGHT));
        }

        // 2. Vertical Radial Grid & Tick Labels
        if use_physical_units {
            let min_km = self.r_offset * metric.r_grav_km();
            let max_km = (self.r_offset + self.max_r) * metric.r_grav_km();
            let km_step = axis::round_step((self.max_r * metric.r_grav_km() / 7.0).max(1e-6));
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
            let r_step = axis::round_step((self.max_r / 7.0).max(1e-6));
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

        // Horizontal Axis Title (Bottom Right)
        let r_axis_title = if use_physical_units {
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

        // What this picture is, said once where the eye lands first. A chart places every event
        // where the coordinates put it and claims nothing about distance or simultaneity for any
        // observer; the two rest-frame views make the opposite claim, and the reader has to know
        // which of the two they are looking at.
        painter.text(
            Pos2::new(rect.center().x, rect.top() + 6.0),
            egui::Align2::CENTER_TOP,
            CHART_BANNER,
            egui::FontId::proportional(Theme::MIN_FONT_PT * font_scale),
            Color32::WHITE,
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
            let rm_label = if use_physical_units {
                format!("Cauchy Horizon r₋ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rm)), rm)
            } else {
                format!("Cauchy Horizon r₋ = {:.2}M ({})", rm, metric.format_physical_distance(rm))
            };
            if let Some(obs) = horizon_reader {
                let box_lines =
                    horizon_box_lines(metric, obs, &obs.name, &rm_label, rm, HORIZON_BOX_RED);
                pending_boxes.push(PendingBox {
                    // The subject, not the title: the title carries the radius and so changes with
                    // the Mass and Spin sliders, which would lose a box the user had dragged.
                    id: BoxId::CauchyHorizon,
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
            let rp_label = if use_physical_units {
                format!("Event Horizon r₊ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rp)), rp)
            } else {
                format!("Event Horizon r₊ = {:.2}M ({})", rp, metric.format_physical_distance(rp))
            };
            if let Some(obs) = horizon_reader {
                let box_lines =
                    horizon_box_lines(metric, obs, &obs.name, &rp_label, rp, Theme::HORIZON_OUTER);
                pending_boxes.push(PendingBox {
                    // The subject, not the title: the title carries the radius and so changes with
                    // the Mass and Spin sliders, which would lose a box the user had dragged.
                    id: BoxId::OuterHorizon,
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
            let re_label = if use_physical_units {
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
        // the pulse's radial extent, [min r, max r] over its front, which `Pulse::extent_track`
        // carries swept up in t. Each of the two edges of that extent is drawn as a *comet*: full
        // brightness at the head, where that edge stands at the chart's present, fading to nothing
        // `Theme::COMET_TAIL_PX` back down the edge's own track. The lower edge is the ingoing edge
        // of the emitter's own light cone, carried from the emission event - the 45-degree line
        // dr/dt = -1 only for a hole with no spin, and slightly steeper than that for one that
        // spins (-1.010 at r = 4.5M for a = 0.65, -2.27 at r = 0.2M for a = 0.90) - and the upper
        // edge is the outermost ray, which outside r+ climbs and inside r+ falls and freezes onto
        // r-. So the row of comets along the now line is where every front of both transmissions
        // stands at this moment, and each tail says which way that edge of it is going.
        //
        // A worldline between the two edges of one pulse is *in range* of that pulse - some part
        // of the front stands at that radius - which is not the same as receiving it, because the
        // diagram cannot show azimuth and the receiver may be at another one. That range used to
        // be drawn, as a faint fill between the edges. It is not drawn now: a statement that weak,
        // laid down once per pulse, covered the whole past half of the canvas within a few M of a
        // run starting, and the reception dots below - the actual arrivals - had to be read
        // through it. The comets leave that half of the chart to the worldlines, the light cones
        // and those dots.
        //
        // Three things decide whether a pulse has a comet at all, and each of them is physics.
        // A pulse with no live ray has no front: `Pulse::extend_track` declines to record an
        // extent `Pulse::radial_extent` will not give it, so the track of a spent pulse stops at
        // the death of its last ray, and drawing a head there would leave a front standing at a
        // radius nothing occupies. Spent pulses are skipped outright - their tracks are kept
        // because `SignalField::step_back` can bring the rays back, and a step back that does
        // brings the comet back with them. While a pulse is being swallowed its lower edge stands
        // on the ring rather than on any one ray, for the reason `Pulse::radial_extent` gives, and
        // its comet is then a vertical tick on the ring, which is the front arriving there; the
        // tick lasts until the last ring-bound ray dies and takes the whole pulse's comet with it.
        // And every pulse emitted inside r+ has its upper edge frozen on r-, so that comet is a
        // vertical tick riding the Cauchy horizon for as long as the pulse lives. The front really
        // is still there, and the column of ticks is the pile of outgoing interior light that a
        // later infaller cuts through.
        //
        // The head is the pulse's extent at the field's own clock, `Pulse::radial_extent` read off
        // the live rays, and the recorded track supplies only the tail behind it. Nothing is
        // extrapolated or interpolated by that: the rays stand at exactly `SignalField::t`, which
        // is the time this chart draws its now line at, so the head is a radius the front has been
        // computed to be at and not a guess past the end of what is known. The track cannot serve
        // as the head, because `Pulse::extend_track` records on a cadence of its own -
        // `TRACK_MIN_DT`, 0.02 M, doubled once per thinning - so its newest point lags the clock by
        // up to that whole spacing. Any step shorter than the cadence, and any zoom that makes
        // 0.02 M more than a point of screen, shows that lag directly: the head stands still for
        // several steps while the now line moves on, and then jumps forward when the next point is
        // recorded. Which edge of the extent a head belongs to is the same question for the tail,
        // so one pass over the rays serves both edges of a pulse.
        //
        // The head is prepended only while the field's clock is at or after the newest track point.
        // It always is in the app - `SignalField::step_back` truncates the track to t <= its target
        // and leaves the clock on that target - but a clock behind the track would put the head
        // below the point the tail starts from and draw a line that doubles back on itself, so that
        // case falls back to the track alone rather than drawing something untrue.
        let draw_comets = |field: &SignalField, colour: Color32| {
            let head_colour = Color32::from_rgba_unmultiplied(
                colour.r(),
                colour.g(),
                colour.b(),
                Theme::COMET_HEAD_ALPHA,
            );
            // A tail reaches at most `COMET_TAIL_PX` from its head, so a head this far outside the
            // canvas cannot put anything on it, and the whole pulse costs one rectangle test.
            let reach = rect.expand(Theme::COMET_TAIL_PX);
            for pulse in field.pulses.iter() {
                // This is the "has a live front" test as well as the reading: `radial_extent` is
                // None exactly when no ray of the pulse is alive, since it returns None on an empty
                // min/max, so a spent pulse is skipped here and nothing else has to ask.
                let Some((lo_now, hi_now)) = pulse.radial_extent(metric) else {
                    continue;
                };
                let Some(&newest) = pulse.extent_track.last() else {
                    continue;
                };
                let head_at = if field.t >= newest.0 { (field.t, lo_now, hi_now) } else { newest };
                for hi_edge in [false, true] {
                    let at = |&(t, lo, hi): &(f64, f64, f64)| {
                        Pos2::new(to_screen_x(if hi_edge { hi } else { lo }), to_screen_y(t))
                    };
                    let head = at(&head_at);
                    if !reach.contains(head) {
                        continue;
                    }
                    // The head leads the recorded track walked backwards. When the newest point of
                    // the track is the head - the usual case while a run plays in steps at or over
                    // the recording cadence - `comet_tail` drops it as a point no further than
                    // `SCREEN_SPACING` from the one already kept, so the duplicate costs a vertex
                    // that is never emitted rather than a zero-length first segment.
                    let track = pulse.extent_track.iter().rev().map(at);
                    let tail =
                        comet_tail(std::iter::once(head).chain(track), Theme::COMET_TAIL_PX);
                    if tail.len() < 2 {
                        continue;
                    }
                    painter.add(fading_line(
                        tail,
                        head,
                        Theme::COMET_TAIL_PX,
                        COMET_WIDTH,
                        head_colour,
                    ));
                }
            }
        };
        draw_comets(signals.bob, Theme::BOB_COLOR);
        draw_comets(signals.alice, Theme::ALICE_COLOR);

        // Alice Worldline & Marker. The info boxes are registered last, below, so that a drag on
        // a box beats the canvas's own pan response instead of panning the diagram.
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice {
            if al.trail.len() >= 2 {
                // Thinned to the screen, so that what this costs to draw follows the length of the
                // curve on the canvas rather than the depth of the buffer behind it: a worldline
                // recorded once a frame is mostly vertices a fraction of a pixel apart. See
                // `thin_to_pixels`.
                let points = thin_to_pixels(
                    al.trail.iter().map(|p| Pos2::new(to_screen_x(p.r), to_screen_y(p.t))),
                    |at| *at,
                    SCREEN_SPACING,
                );
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
            // Thinned to the screen, exactly as Alice's is just above.
            let points = thin_to_pixels(
                bob.trail.iter().map(|p| Pos2::new(to_screen_x(p.r), to_screen_y(p.t))),
                |at| *at,
                SCREEN_SPACING,
            );
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

        // The time axis's text, held back from where its lines were ruled so that it lands on top
        // of the light rather than under it. Each label gets a dark plate a little larger than its
        // own text first: the fills behind it run from the canvas's near-black to a wavefront's
        // full orange, and no single text colour reads against both.
        for (pos, align, text, colour) in time_labels {
            let galley = painter.layout_no_wrap(text, label_font.clone(), colour);
            let text_rect = align.anchor_size(pos, galley.size());
            painter.rect_filled(text_rect.expand2(Vec2::new(3.0, 1.0)), 2.0, Theme::LABEL_PLATE);
            painter.galley(text_rect.min, galley, colour);
        }

        // Everything left on this canvas is Bob's: his worldline, his light cone, his marker and
        // the two boxes read off him. With no Bob in the simulation there is none of it.
        let Some(bob) = bob else {
            if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
                self.telemetry.show(
                    ui, painter, Canvas::Spacetime, BoxId::Observer(Who::Alice), rect, alice_pos,
                    "Alice", Theme::ALICE_COLOR, al, metric, use_physical_units, font_scale,
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

        // Every info box on this canvas is painted here, after everything else is down, so that
        // the opaque fill of a box actually blocks out what is behind it.
        self.flush_pending_boxes(ui, painter, Canvas::Spacetime, rect, &pending_boxes, font_scale);

        if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
            self.telemetry.show(
                ui, painter, Canvas::Spacetime, BoxId::Observer(Who::Alice), rect, alice_pos, "Alice",
                Theme::ALICE_COLOR, al, metric, use_physical_units, font_scale,
            );
        }
        self.telemetry.show(
            ui, painter, Canvas::Spacetime, BoxId::Observer(Who::Bob), rect, apex, "Bob", Theme::BOB_COLOR, bob,
            metric, use_physical_units, font_scale,
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
        canvas: Canvas,
        rect: Rect,
        pending: &[PendingBox],
        font_scale: f32,
    ) {
        let mut occupied: Vec<Rect> = Vec::new();
        for b in pending {
            let size = telemetry_box_size(painter, &b.lines, font_scale);
            let anchor = if self.telemetry.is_placed(canvas, b.id) {
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
                ui, painter, canvas, b.id, rect, anchor, &b.lines, b.color, font_scale, b.tip,
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
        use_physical_units: bool,
        font_scale: f32,
        show_distant_clock_grid: bool,
        signals: SignalViews<'_>,
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
        let focus_who = Who::of(focus_obs).unwrap_or(Who::Bob);
        let obs_color =
            if focus_who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
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
                BoxId::Ergosphere,
                "Ergosphere",
                ergo_faint,
                1.4,
                Theme::ERGOSPHERE_LINE,
                false,
            ),
            (
                metric.outer_horizon(),
                BoxId::OuterHorizon,
                "Outer Horizon r₊",
                Theme::HORIZON_OUTER,
                2.5,
                Theme::HORIZON_OUTER,
                true,
            ),
            (
                metric.inner_horizon(),
                BoxId::CauchyHorizon,
                "Cauchy Horizon r₋",
                Theme::HORIZON_CAUCHY,
                2.5,
                HORIZON_BOX_RED,
                true,
            ),
            (
                0.0,
                BoxId::RingSingularity,
                "Ring Singularity (r=0)",
                Theme::SINGULARITY_LINE,
                3.0,
                Theme::SINGULARITY_LINE,
                false,
            ),
        ];

        // Worked out here, where the line geometry is; painted at the end of the pass.
        let mut pending_boxes: Vec<PendingBox> = Vec::new();

        for (idx, &(r_h, id, title, color, width, border, is_horizon)) in surfaces.iter().enumerate() {
            let line = frame.surface_r_const(r_h);
            let anchor = to_screen(line.point[0], line.point[1]);
            let dir = Vec2::new(line.dir[0] as f32, -(line.dir[1] as f32));
            let Some((end_a, end_b)) = clip_line_to_rect(anchor, dir, rect) else {
                continue;
            };
            painter.line_segment([end_a, end_b], Stroke::new(width, color));

            let steep = steep_with_hysteresis(self.steep_boxes.contains(&id), line.slope().abs());
            if steep {
                self.steep_boxes.insert(id);
            } else {
                self.steep_boxes.remove(&id);
            }
            let (label_pos, align) = if steep {
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
                SurfaceCharacter::Null => "Null Surface (crossing now)",
                SurfaceCharacter::Timelike => "Timelike Surface (avoidable)",
                SurfaceCharacter::Spacelike => {
                    if line.xi0_at_axis().unwrap_or(0.0) >= 0.0 {
                        "Spacelike Surface (in your future)"
                    } else {
                        "Spacelike Surface (in your past)"
                    }
                }
            };

            // The one number the box keeps: a timelike surface is the world-tube of observers
            // hovering at that r, and in this frame it moves at 1/|slope| (a boosted vertical
            // line has slope 1/v); a spacelike surface is a simultaneity slice of the observers
            // there, closing at |slope|; a null one moves at c.
            let slope_abs = line.slope().abs();
            let detail = match line.character(2e-3) {
                SurfaceCharacter::Timelike => {
                    let speed = if slope_abs.is_finite() { 1.0 / slope_abs.max(1e-9) } else { 0.0 };
                    format!("Moving at {speed:.2}c")
                }
                SurfaceCharacter::Null => "Moving at 1.00c".to_string(),
                SurfaceCharacter::Spacelike => format!("Closing at {slope_abs:.2}c"),
            };

            // Every surface answers in a box now. A horizon reports what it is to this observer -
            // a place with a distance, or a moment on his own clock - and the other two report the
            // causal character of their line as drawn, which is the same question asked of a
            // surface whose answer never changes.
            let box_lines = if is_horizon {
                horizon_box_lines(metric, focus_obs, &focus_obs.name, title, r_h, border)
            } else {
                vec![
                    TelemetryLine { text: title.to_string(), color: border, is_title: true, bold: false },
                    TelemetryLine { text: note.to_string(), color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
                    TelemetryLine { text: detail, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
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
                id,
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
        // 3b. The other observer's transmission as a wave: every pulse is a crest, and a crest is
        // a null surface, so through this plane it is a line at 45 degrees when it arrives
        // radially and steeper when it arrives obliquely, since only the radial part of its motion
        // lies in the drawn plane. A received crest passes through its arrival on the observer's
        // own worldline, which is exact; one still in flight is placed by its nearest ray's
        // current event, to first order. The spacing of the arrivals up the axis *is* the
        // received period on the observer's own clock, and against the emitter's spacing it is
        // the frequency ratio without a formula. On the approach to r- the rungs crowd together
        // without limit: the infinite blueshift, drawn as crests. See `wave_crests`.
        let (sender_field, sender, crest_colour) = if focus_who == Who::Alice {
            (signals.bob, Who::Bob, Theme::BOB_COLOR)
        } else {
            (signals.alice, Who::Alice, Theme::ALICE_COLOR)
        };
        let sender_name = sender.name();
        let crests = wave_crests(&frame, focus_obs, sender_field, self.frame_max_r);
        // A crest is drawn as a stroke through its anchor a third of the canvas long, not across
        // the whole plane: near the worldline the placement is exact and far from it the chart is
        // only first order, and a stroke that ends says so. The older arrivals keep their dot on
        // the worldline and lose their stroke, so the rungs stay countable as they crowd.
        let half_len = 0.3 * rect.height().min(rect.width());
        let received_total = crests.crests.iter().filter(|c| c.received).count();
        let mut received_seen = 0usize;
        for crest in &crests.crests {
            let anchor = to_screen(crest.xi1, crest.xi0);
            let dir = Vec2::new(crest.dir[0] as f32, -(crest.dir[1] as f32));
            let as_line = if crest.received {
                received_seen += 1;
                received_total - received_seen < CRESTS_RECEIVED_AS_LINES
            } else {
                true
            };
            if as_line {
                let (alpha, width) = if crest.latest {
                    (0.9, 1.6)
                } else if crest.received {
                    (0.5, 1.0)
                } else {
                    (0.3, 1.0)
                };
                // Graded along its length rather than flat: the crest's own colour at the future
                // end of the stroke, falling to nothing at the past end, so a still picture says
                // which way the wave is sweeping. `crest.dir` is the future-directed null vector
                // (1, dr/dt, dphi/dt) pushed through the tetrad and normalised in the drawn plane,
                // so its xi^0 component is positive and the future end is anchor + dir * half_len;
                // the sign is read rather than assumed, because `wave_crests` hands back a fixed
                // fallback direction where the push degenerates and a chart is a worse place than
                // this to discover that the fallback ever changed.
                let (past, future) = if crest.dir[1] >= 0.0 {
                    (anchor - dir * half_len, anchor + dir * half_len)
                } else {
                    (anchor + dir * half_len, anchor - dir * half_len)
                };
                // The fade spans the whole stroke, so the alpha reaches zero exactly at the past
                // end and the stroke keeps the two end points it has always had.
                painter.add(fading_line(
                    vec![past, future],
                    future,
                    2.0 * half_len,
                    width,
                    crest_colour.gamma_multiply(alpha),
                ));
            }
            if crest.received && rect.contains(anchor) {
                painter.circle_filled(anchor, 2.5, crest_colour);
            }
        }
        // The readout: the two frequencies in hertz, on the two clocks that measure them, and
        // their ratio. The periods are proper times in M; a second of a clock is M of it times
        // t_g / M, the same conversion the distant clock's labels use. Where no consecutive pair
        // of the same family has arrived yet there is no period, and the ray's own factor is the
        // one number there is.
        if let Some(ray) = crests.last_ray_ratio {
            let seconds_per_m = metric.t_grav_seconds() / metric.m.max(1e-12);
            let hz = |period_m: f64| format_frequency(1.0 / (period_m * seconds_per_m));
            let mut lines = vec![TelemetryLine {
                text: format!("{sender_name}'s signal at {}", focus_obs.name),
                color: crest_colour,
                is_title: true,
                bold: false,
            }];
            let line = |text: String| TelemetryLine {
                text,
                color: Theme::TEXT_BRIGHT,
                is_title: false,
                bold: false,
            };
            match (crests.received_period, crests.emitted_period, crests.period_ratio()) {
                (Some(rx), Some(tx), Some(ratio)) => {
                    lines.push(line(format!("{} Receive Frequency: {}", focus_obs.name, hz(rx))));
                    lines.push(line(format!("{sender_name} Transmit Frequency: {}", hz(tx))));
                    lines.push(line(format!("Blueshift: {ratio:.3}")));
                }
                _ => lines.push(line(format!("Blueshift: {ray:.3} (last ray)"))),
            }
            // What the wavefront cap has cost this reading, on the runs where it has cost it
            // anything. Every number above is measured off the pulses the field still holds, so a
            // pulse evicted while it could still have arrived is an arrival that never happened
            // and a receive frequency measured across a gap that was not there. The reading is then
            // a lower bound and says so, which beats quietly reporting a trimmed range: at the
            // default cap a transmitting orbit loses five arrivals in six. See
            // `SignalField::dropped_in_flight`.
            let dropped = sender_field.dropped_in_flight();
            if dropped > 0 {
                lines.push(TelemetryLine {
                    text: format!("Incomplete: {dropped} fronts dropped in flight"),
                    color: Theme::WARNING_RED,
                    is_title: false,
                    bold: true,
                });
            }
            // Anchored at the bottom right of the canvas until it is dragged somewhere else.
            let size = telemetry_box_size(painter, &lines, font_scale);
            pending_boxes.push(PendingBox {
                id: BoxId::Signal(sender),
                anchor: Pos2::new(rect.right() - 10.0 - size.x, rect.bottom() - 10.0 - size.y),
                lines,
                color: crest_colour,
                tip: SIGNAL_BOX_TIP,
            });
        }

        let cone_len = (rect.height() * 0.35).min(rect.width() * 0.35);
        let apex = center;
        let (focus_future_fill, focus_past_fill, focus_edge) = Theme::cone_colours(Some(focus_who));

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
                "SINGULARITY IMPACT\nLight cone terminated at r = 0",
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
        let mut other_box: Option<(&Observer, Who, Pos2, Color32)> = None;
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
                let other_who = Who::of(other).unwrap_or(Who::Bob);
                let other_color = if other_who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
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

                    let (other_future_fill, _, other_edge) = Theme::cone_colours(Some(other_who));
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
                        metric.format_r(xi[2], use_physical_units),
                        v_rel.min(9.999)
                    ),
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                    Theme::TEXT_MUTED,
                );

                other_box = Some((other, other_who, other_pos, other_color));
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
        self.flush_pending_boxes(ui, painter, Canvas::RestFrame, rect, &pending_boxes, font_scale);
        if let Some((other, other_who, other_pos, other_color)) = other_box {
            self.telemetry.show(
                ui, painter, Canvas::RestFrame, BoxId::Observer(other_who), rect, other_pos, &other.name, other_color,
                other, metric, use_physical_units, font_scale,
            );
        }
        self.telemetry.show(
            ui, painter, Canvas::RestFrame, BoxId::Observer(focus_who), rect, apex, &focus_obs.name, obs_color,
            focus_obs, metric, use_physical_units, font_scale,
        );
    }
}

/// Clip the infinite line p + t d to `rect` (Liang-Barsky), returning its visible segment.
/// How many crests still on their way are drawn, nearest first, and how many received ones are
/// drawn as strokes rather than as dots on the worldline alone: the wave at the observer, not
/// the whole history of the transmission.
const CRESTS_IN_FLIGHT: usize = 4;
const CRESTS_RECEIVED_AS_LINES: usize = 12;

/// One crest of a transmission, placed in the focus observer's chart: a point of the drawn
/// (xi^1, xi^0) plane it passes through and its Euclidean-unit direction there.
pub(crate) struct Crest {
    pub xi1: f64,
    pub xi0: f64,
    /// (d xi^1, d xi^0), the trace of the crest's null direction in the drawn plane.
    pub dir: [f64; 2],
    /// Whether the crest has already reached the observer, in which case it passes through the
    /// arrival on their own worldline and the placement is exact.
    pub received: bool,
    /// The most recent arrival.
    pub latest: bool,
}

/// The other observer's transmission read as a wave at the focus observer's worldline.
pub(crate) struct WaveCrests {
    pub crests: Vec<Crest>,
    /// Proper time on the focus observer's clock between the last two arrivals, in M.
    pub received_period: Option<f64>,
    /// Proper time on the emitter's clock between the emissions of those same two pulses, in M.
    pub emitted_period: Option<f64>,
    /// nu(receiver) / nu(emitter) carried by the ray of the most recent arrival.
    pub last_ray_ratio: Option<f64>,
}

impl WaveCrests {
    /// f_rx / f_tx from the two periods: the ratio of proper intervals between the same pair of
    /// pulses at emission and at reception, which is exact, and independent of the per-ray
    /// factor it should agree with.
    pub fn period_ratio(&self) -> Option<f64> {
        match (self.emitted_period, self.received_period) {
            (Some(tx), Some(rx)) if rx > 0.0 && tx.is_finite() => Some(tx / rx),
            _ => None,
        }
    }
}

/// A transmission's pulses as wave crests in the focus observer's local chart.
///
/// A train of pulses emitted at a fixed interval of the emitter's clock is a wave with the
/// pulses as crests, and the observer's received frequency is the ratio of two proper intervals:
/// the emitter's between two emissions and the observer's between the corresponding arrivals.
/// Both are recorded - every reception carries the observer's proper time at the crossing, every
/// pulse its emitter's at emission - so the ratio is measured rather than computed, and the
/// per-ray frequency factor the integrator carries is the monochromatic answer it agrees with in
/// the limit of closely spaced pulses.
///
/// A received crest is placed through its arrival on the observer's worldline, xi^1 = 0 at
/// xi^0 = tau_arrival - tau_now, which is exact. Its direction is that of the pulse's ray
/// nearest the observer in azimuth, pushed into the chart: a radial arrival is a 45 degree
/// line, an oblique one steeper, since only the radial part of its motion is in the drawn plane.
/// A crest still in flight is placed by that ray's current event through the linearised chart,
/// and only within twice `reach` M of the observer, the window the picture is framed to, which
/// is the region the chart can speak for.
pub(crate) fn wave_crests(
    frame: &LocalFrame,
    focus: &Observer,
    field: &SignalField,
    reach: f64,
) -> WaveCrests {
    use std::f64::consts::{PI, TAU};
    let wrap = |d: f64| (d + PI).rem_euclid(TAU) - PI;
    // The trace in the drawn plane of a crest moving with coordinate slopes (dr/dt, dphi/dt):
    // the null vector (1, dr/dt, dphi/dt) in the chart's components, projected onto (xi^1, xi^0).
    let direction = |dr_dt: f64, dphi_dt: f64| -> [f64; 2] {
        let radial_ingoing = [-std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2];
        let v = frame.vector_to_local(&[1.0, dr_dt, dphi_dt]);
        let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
        if !len.is_finite() || len <= 0.0 {
            return radial_ingoing;
        }
        [v[1] / len, v[0] / len]
    };
    let pulse_of = |index: usize| field.pulses.iter().find(|p| p.index == index);
    // Where a ray's current event lands in the chart.
    let place = |ray: &NullRay| -> [f64; 3] {
        frame.to_local(&[ray.t - focus.t, ray.r - focus.r, wrap(ray.phi - focus.phi)])
    };

    let mut received: Vec<&Reception> = field.receptions().collect();
    received.sort_by(|a, b| a.tau_receiver.total_cmp(&b.tau_receiver));
    let latest_index = received.last().map(|r| r.pulse_index);

    let mut crests = Vec::new();
    for rec in &received {
        crests.push(Crest {
            xi1: 0.0,
            xi0: rec.tau_receiver - focus.tau,
            dir: direction(rec.dr_dt, rec.dphi_dt),
            received: true,
            latest: latest_index == Some(rec.pulse_index),
        });
    }
    let mut in_flight: Vec<(f64, Crest)> = Vec::new();
    for pulse in &field.pulses {
        if received.iter().any(|r| r.pulse_index == pulse.index) {
            continue;
        }
        // The point of the front nearest the observer in their own chart is where the crest
        // will sweep over them, to first order: the ray whose current event lies closest.
        let Some((ray, xi)) = pulse
            .rays
            .iter()
            .filter(|ray| ray.alive())
            .map(|ray| (ray, place(ray)))
            .filter(|(_, xi)| xi.iter().all(|c| c.is_finite()))
            .min_by(|(_, a), (_, b)| a[1].hypot(a[0]).total_cmp(&b[1].hypot(b[0])))
        else {
            continue;
        };
        let distance = xi[1].hypot(xi[0]);
        if distance >= 2.0 * reach {
            continue;
        }
        in_flight.push((
            distance,
            Crest {
                xi1: xi[1],
                xi0: xi[0],
                dir: direction(ray.dr_dt, ray.dphi_dt),
                received: false,
                latest: false,
            },
        ));
    }
    // The few crests about to arrive, nearest first: the picture is of the wave at the
    // observer's worldline, and forty fronts in flight drawn across the whole plane are a web.
    in_flight.sort_by(|a, b| a.0.total_cmp(&b.0));
    crests.extend(in_flight.into_iter().take(CRESTS_IN_FLIGHT).map(|(_, c)| c));

    // The periods come from the last arrival and the arrival of the pulse sent just before it,
    // on the same sheet family. Not simply the last two arrivals: a front is a loop, and inside
    // r+ the same pulse can sweep the observer twice, once with its crossing family and once with
    // its frozen one, and a pair mixing two pulses' families or skipping a pulse measures nothing
    // a wave has. Where no such pair exists there is no period to report, only the ray's factor.
    let pair = received.last().and_then(|b| {
        received
            .iter()
            .rev()
            .find(|a| a.pulse_index + 1 == b.pulse_index && a.frozen_family == b.frozen_family)
            .map(|a| (a, b))
    });
    let (received_period, emitted_period) = match pair {
        Some((a, b)) => (
            Some(b.tau_receiver - a.tau_receiver),
            match (pulse_of(a.pulse_index), pulse_of(b.pulse_index)) {
                (Some(pa), Some(pb)) => Some(pb.emitted_tau - pa.emitted_tau),
                _ => None,
            },
        ),
        None => (None, None),
    };
    WaveCrests {
        crests,
        received_period,
        emitted_period,
        last_ray_ratio: received.last().map(|r| r.ratio),
    }
}

/// A frequency in hertz with the usual SI prefix, at four significant digits: 489.0 mHz,
/// 12.35 Hz, 1.095 kHz, 136.2 MHz. Below a microhertz or above a terahertz it falls back to
/// scientific notation, and anything that is not a finite positive number is "n/a".
pub(crate) fn format_frequency(hz: f64) -> String {
    if !hz.is_finite() || hz <= 0.0 {
        return "n/a".to_string();
    }
    // Rounded to four significant digits first, so that a value which rounds up to the next
    // power of a thousand takes the next prefix rather than printing as 1000.0.
    let magnitude = hz.log10().floor() - 3.0;
    let quantum = 10f64.powf(magnitude);
    let hz = (hz / quantum).round() * quantum;
    const PREFIXES: [(&str, f64); 7] = [
        ("µHz", 1e-6),
        ("mHz", 1e-3),
        ("Hz", 1.0),
        ("kHz", 1e3),
        ("MHz", 1e6),
        ("GHz", 1e9),
        ("THz", 1e12),
    ];
    // The largest prefix the value is at least one of, so the mantissa lies in [1, 1000).
    let Some(&(unit, scale)) = PREFIXES.iter().rev().find(|(_, scale)| hz >= *scale) else {
        return format!("{hz:.3e} Hz");
    };
    let mantissa = hz / scale;
    if mantissa >= 1000.0 {
        return format!("{hz:.3e} Hz");
    }
    // Four significant digits: three decimals for a mantissa below ten, two below a hundred,
    // one below a thousand.
    let decimals = if mantissa < 10.0 { 3 } else if mantissa < 100.0 { 2 } else { 1 };
    format!("{mantissa:.decimals$} {unit}")
}

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

    #[test]
    fn test_the_box_shadow_falls_down_and_to_the_right_only() {
        // One light for every box on every canvas, high and to the upper left. `epaint` works the
        // reach out as `spread + blur/2 -/+ offset` per side, so this is the shadow's own geometry
        // rather than a guess at it, and it is asserted because the two halves are easy to set
        // independently and wrong: a blur wider than twice the offset creeps out of the top left
        // and reads as a second light, and unequal offsets put the light off the diagonal.
        let reach = BOX_SHADOW.margin();
        assert_eq!(reach.left, 0.0, "nothing to the left of the box");
        assert_eq!(reach.top, 0.0, "nor above it");
        assert_eq!(reach.right, 10.0, "and gone by ten points to the right");
        assert_eq!(reach.bottom, 10.0, "and ten points below");
        assert_eq!(reach.right, reach.bottom, "which puts the light at 45 degrees");
        // Transparent, so what is under a box is dimmed rather than erased.
        assert!(BOX_SHADOW.color.a() > 0 && BOX_SHADOW.color.a() < 255);
    }

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
            bold: false,
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

    #[test]
    fn test_every_canvas_and_box_has_its_own_slug_and_comes_back_from_it() {
        // The slugs are the identity a save file will carry, so two of them being equal is a
        // collision that silently merges two boxes' positions, and one that does not come back
        // through `from_key` is a placement nothing can reload. Both are checked over the whole
        // set rather than over the variants somebody remembered to list.
        let mut seen: Vec<&'static str> = Vec::new();
        for canvas in Canvas::ALL {
            let key = canvas.key();
            assert!(!seen.contains(&key), "{key} names two canvases");
            seen.push(key);
            assert_eq!(Canvas::from_key(key), Some(canvas), "{key} does not come back");
        }
        let mut seen: Vec<&'static str> = Vec::new();
        for id in BoxId::ALL {
            let key = id.key();
            assert!(!seen.contains(&key), "{key} names two boxes");
            seen.push(key);
            assert_eq!(BoxId::from_key(key), Some(id), "{key} does not come back");
        }
        assert_eq!(Canvas::from_key("restframe "), None, "a slug is matched whole");
        assert_eq!(BoxId::from_key("Alice"), None, "and in its own lowercase spelling");
    }

    #[test]
    fn test_a_placement_belongs_to_one_box_on_one_canvas() {
        // What the typed key buys: the same observer's box is a different box on every diagram,
        // and the other subjects on the diagram it was dragged on are untouched. Dragging is the
        // ui's business and is driven elsewhere; the placement itself is written here, since what
        // is under test is the filing and not the drag.
        let mut boxes = TelemetryBoxes::pinning();
        boxes
            .placements
            .insert((Canvas::Spacetime, BoxId::Observer(Who::Alice)), Placement::Pinned(Vec2::new(12.0, 8.0)));
        assert!(boxes.is_placed(Canvas::Spacetime, BoxId::Observer(Who::Alice)));
        assert!(
            !boxes.is_placed(Canvas::RestFrame, BoxId::Observer(Who::Alice)),
            "her box on another diagram is another box"
        );
        assert!(
            !boxes.is_placed(Canvas::Spacetime, BoxId::Observer(Who::Bob)),
            "and so is his on the same one"
        );
        assert!(!boxes.is_placed(Canvas::Spacetime, BoxId::Signal(Who::Alice)));
        assert!(!boxes.is_placed(Canvas::Spacetime, BoxId::CauchyHorizon));
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

    #[test]
    fn test_wave_crests_read_the_received_frequency_off_consecutive_arrivals() {
        // Two static observers, Alice above Bob, and no spin: the frequency Bob receives from
        // Alice is the gravitational shift sqrt((1 - 2M/r_A) / (1 - 2M/r_B)), a closed form. The
        // crests give it two ways - the ratio of proper intervals between the same pair of pulses
        // at emission and at arrival, and the per-ray factor on the last arrival - and both have
        // to be it. Every received crest passes through the arrival on Bob's own worldline, in
        // his past, heading inward; and the crests still on their way are placed too.
        use crate::physics::observer::{ObserverMode, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.0);
        let (r_alice, r_bob) = (6.0, 4.5);
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, r_alice, 0.0, 0.0, WorldlineParams::default());
        alice.mode = ObserverMode::Static;
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, r_bob, 0.0, 0.0, WorldlineParams::default());
        bob.mode = ObserverMode::Static;
        let mut field = SignalField::default();
        let dt = 0.1;
        for i in 0..60 {
            let t = ((i + 1) as f64) * dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &alice);
            field.detect_receptions(&metric, &bob);
        }
        assert!(field.received_count() >= 3, "Bob heard {} pulses", field.received_count());

        let frame = LocalFrame::for_observer(&metric, bob.r, &bob.four_velocity(&metric));
        let crests = wave_crests(&frame, &bob, &field, 10.0);
        let expected = ((1.0 - 2.0 / r_alice) / (1.0 - 2.0 / r_bob)).sqrt();
        let ratio = crests.period_ratio().expect("two arrivals give a period ratio");
        let ray = crests.last_ray_ratio.expect("the last arrival carries its ray's factor");
        println!(
            "received every {:?} M of Bob's, sent every {:?} M of Alice's: f_rx/f_tx = {ratio:.4} \
             from the periods, {ray:.4} on the last ray, {expected:.4} in closed form",
            crests.received_period, crests.emitted_period
        );
        assert!((ratio / expected - 1.0).abs() < 0.03, "the period ratio is the gravitational shift");
        assert!((ray / expected - 1.0).abs() < 0.03, "and so is the ray's own factor");

        let received: Vec<&Crest> = crests.crests.iter().filter(|c| c.received).collect();
        assert!(received.len() >= 3, "{} received crests drawn", received.len());
        assert_eq!(received.iter().filter(|c| c.latest).count(), 1, "one crest is the latest");
        for c in &received {
            assert_eq!(c.xi1, 0.0, "a received crest passes through Bob's own worldline");
            assert!(c.xi0 <= 0.0, "in his past: xi0 = {}", c.xi0);
            let len = c.dir[0].hypot(c.dir[1]);
            assert!((len - 1.0).abs() < 1e-9, "unit direction, not {len}");
            assert!(c.dir[1] > 0.0 && c.dir[0] < 0.0, "future-directed and ingoing: {:?}", c.dir);
        }
        assert!(
            crests.crests.iter().any(|c| !c.received),
            "and the pulses still on their way to him are drawn too"
        );
    }

    #[test]
    fn test_a_surface_box_changes_margin_only_when_the_slope_is_well_past_one() {
        // Seen in the app: on the approach to r- the Cauchy line is null, |slope| = 1 give or
        // take a little each frame, and its box jumped between the top margin and the right one
        // as the slope crossed 1. With the band it changes side only when the line has clearly
        // become the other kind.
        for slope in [0.81, 0.95, 1.0, 1.05, 1.24] {
            assert!(steep_with_hysteresis(true, slope), "a steep box stays steep at {slope}");
            assert!(!steep_with_hysteresis(false, slope), "a flat box stays flat at {slope}");
        }
        assert!(!steep_with_hysteresis(true, 0.79), "and lets go below the band");
        assert!(steep_with_hysteresis(false, 1.26), "and takes hold above it");
        assert!(steep_with_hysteresis(true, f64::INFINITY) && steep_with_hysteresis(false, f64::INFINITY));
    }

    #[test]
    fn test_a_frequency_is_printed_with_its_si_prefix_at_four_significant_digits() {
        for (hz, want) in [
            (0.489, "489.0 mHz"),
            (12.345, "12.35 Hz"),
            (1095.4, "1.095 kHz"),
            (136_150.846, "136.2 kHz"),
            (2.5e6, "2.500 MHz"),
            (7.77e10, "77.70 GHz"),
            (3.0e12, "3.000 THz"),
            (4.2e-6, "4.200 µHz"),
            (999.96, "1.000 kHz"),
        ] {
            assert_eq!(format_frequency(hz), want, "{hz} Hz");
        }
        assert_eq!(format_frequency(0.0), "n/a");
        assert_eq!(format_frequency(f64::NAN), "n/a");
        assert_eq!(format_frequency(1e-9), "1.000e-9 Hz");
    }

    #[test]
    fn test_the_frame_view_reports_the_received_frequency_of_the_others_signal() {
        // The readout belongs to the picture, not to a test of the model alone: with Alice
        // transmitting and Bob receiving, Bob's frame prints the ratio, and Alice's frame, where
        // the roles are the other way round and Bob is silent, prints nothing.
        use crate::physics::observer::{ObserverMode, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.0);
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 6.0, 0.0, 0.0, WorldlineParams::default());
        alice.mode = ObserverMode::Static;
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        bob.mode = ObserverMode::Static;
        let mut field = SignalField::default();
        let dt = 0.1;
        for i in 0..60 {
            let t = ((i + 1) as f64) * dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &alice);
            field.detect_receptions(&metric, &bob);
        }
        let idle = SignalField::default();
        let text_of = |frame: ReferenceFrame| {
            let mut canvas = SpacetimeCanvas { keep_surface_framed: false, ..Default::default() };
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
                    Some(&alice),
                    6.0,
                    600.0,
                    false,
                    frame,
                    1.0,
                    SignalViews { alice: &field, bob: &idle },
                    false,
                );
            });
            let mut text = String::new();
            for clipped in output.shapes.iter() {
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
            text
        };
        let bobs = text_of(ReferenceFrame::Bob);
        println!("{bobs}");
        assert!(bobs.contains("Alice's signal at Bob"), "the box is titled with the signal");
        // Both frequencies in hertz on the clocks that measure them, and their ratio.
        let hz = |label: &str| -> f64 {
            let line = bobs.lines().find(|l| l.starts_with(label)).unwrap_or_else(|| panic!("{label} in {bobs}"));
            let mut parts = line.trim_start_matches(label).split_whitespace();
            let number: f64 = parts.next().and_then(|n| n.parse().ok()).unwrap_or_else(|| panic!("a number on {line:?}"));
            let scale = match parts.next() {
                Some("µHz") => 1e-6,
                Some("mHz") => 1e-3,
                Some("Hz") => 1.0,
                Some("kHz") => 1e3,
                Some("MHz") => 1e6,
                Some("GHz") => 1e9,
                Some("THz") => 1e12,
                other => panic!("a frequency unit on {line:?}, not {other:?}"),
            };
            number * scale
        };
        let (f_rx, f_tx) = (hz("Bob Receive Frequency:"), hz("Alice Transmit Frequency:"));
        let expected = ((1.0 - 2.0 / 6.0) / (1.0 - 2.0 / 4.5_f64)).sqrt();
        assert!((f_rx / f_tx / expected - 1.0).abs() < 0.03, "{f_rx} Hz over {f_tx} Hz is the shift");
        assert!(bobs.contains("Blueshift: 1.095"), "and it is printed as the blueshift: {bobs}");
        let alices = text_of(ReferenceFrame::Alice);
        assert!(!alices.contains("Receive Frequency"), "Bob sends nothing, so Alice's frame is silent");
    }

    #[test]
    fn test_the_box_quotes_a_measured_speed_only_against_a_frame_that_exists() {
        use crate::physics::observer::{Release, WorldlineParams};

        // A circular orbit is the case the radial rates cannot describe at all: dr/dt and dr/dtau
        // are both exactly zero on it, and without the rows below them the box would say an ISCO
        // orbiter at a quarter of the speed of light was standing still.
        let metric = KerrSchild::new(1.0, 0.90);
        let r = metric.isco(true);
        let (energy, l_ang) = metric.circular_orbit(r, true).expect("the prograde ISCO");
        let orbiter = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r,
            0.0,
            0.0,
            WorldlineParams { energy, l_ang, outgoing: false, release: Release::CircularPrograde },
        );
        let text = |obs: &Observer| {
            telemetry_lines("Alice", Theme::ALICE_COLOR, obs, &metric, false)
                .iter()
                .map(|l| l.text.clone())
                .collect::<Vec<_>>()
                .join("\n")
        };
        let box_text = text(&orbiter);
        assert!(box_text.contains("0.00c"), "not moving in r: {box_text}");
        assert!(box_text.contains("v_stat  = 0.898c"), "the static observer's reading: {box_text}");
        assert!(box_text.contains("v_ZAMO  = 0.625c"), "and the ZAMO's: {box_text}");
        // Every label field is eight wide, so the = of every line sits in one column.
        // The rates and the speeds are one block of the box and share a column for their =. The
        // lines below them - a_prop, Tidal, ν - are each their own width and always were; this is
        // about the group that reads as a column. Counted in characters, not bytes, because the
        // labels carry τ and Ω.
        let block: Vec<&str> = box_text
            .lines()
            .filter(|l| l.starts_with("dr/") || l.starts_with('Ω') || l.starts_with("v_"))
            .collect();
        assert_eq!(block.len(), 5, "two rates, an Ω and two speeds: {block:?}");
        for line in block {
            assert_eq!(
                line.chars().position(|c| c == '='),
                Some(8),
                "the = of {line:?} is out of column"
            );
        }
        assert!(box_text.contains("Ω       = +0.2254/M (drag +0.1125/M)"), "{box_text}");

        // dr/dtau is the one rate in the box that is not a speed, and it no longer claims to be.
        assert!(box_text.contains("0.00 M/τ"), "{box_text}");
        assert!(
            !box_text.lines().any(|l| l.starts_with("dr/dτ") && l.contains('c')),
            "dr/dτ must not be labelled in c: {box_text}"
        );

        // Inside the static limit the static observer is gone and the box stops quoting one.
        let ergo_r = 0.5 * (metric.outer_horizon() + metric.ergosphere_equatorial());
        let in_ergo = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            ergo_r,
            0.0,
            0.0,
            WorldlineParams::default(),
        );
        let box_text = text(&in_ergo);
        assert!(!box_text.contains("v_stat "), "no static observer in the ergosphere: {box_text}");
        assert!(box_text.contains("v_ZAMO"), "but the ZAMO survives it: {box_text}");

        // Between the horizons nothing hovers, and the raindrop is what is left.
        let inside = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            0.5 * (metric.inner_horizon() + metric.outer_horizon()),
            0.0,
            0.0,
            WorldlineParams::default(),
        );
        let box_text = text(&inside);
        assert!(box_text.contains("v_rain"), "the frame that exists everywhere: {box_text}");
        assert!(!box_text.contains("v_ZAMO"), "and no hovering frame: {box_text}");

        // The default is physical units, and in that mode the box may not put an M in front of a
        // reader who has not asked for one. The E and L of a geodesic are the documented
        // exception: they are the constants the sliders set, and that slider is labelled in M too.
        let phys = telemetry_lines("Alice", Theme::ALICE_COLOR, &orbiter, &metric, true)
            .iter()
            .map(|l| l.text.clone())
            .collect::<Vec<_>>()
            .join("
");
        assert!(phys.contains("rad/s"), "Ω reads as a rate per second: {phys}");
        assert!(phys.contains("km/s (own clock)"), "and dr/dτ in km per second: {phys}");
        assert!(phys.contains("dr/dt   = -0.00c"), "speeds stay in c in both modes: {phys}");
        for line in phys.lines().filter(|l| !l.starts_with("E =")) {
            assert!(
                !line.contains(" M") && !line.contains("/M") && !line.ends_with('M'),
                "an M reached the default readout: {line:?}"
            );
        }

        // However far the chart rates run away, every measured speed in the box is below c: a
        // worldline frozen on r- is measured against the raindrop at a huge gamma, and the row
        // prints the bound rather than rounding up to exactly c.
        let frozen = Observer::frozen_bob(&metric);
        let box_text = text(&frozen);
        assert!(
            box_text.contains("v_rain  = >0.9999c"),
            "frozen on r-, the bound rather than a flat 1.000c: {box_text}"
        );
        for line in box_text.lines().filter(|l| l.starts_with("v_")) {
            let token = line
                .split('=')
                .nth(1)
                .and_then(|t| t.split_whitespace().next())
                .unwrap_or_else(|| panic!("a speed on {line:?}"));
            let v: f64 = token
                .trim_start_matches('>')
                .trim_end_matches('c')
                .parse()
                .unwrap_or_else(|_| panic!("a number on {line:?}"));
            assert!(v <= 0.9999, "{line} claims c or better");
        }
    }

    #[test]
    fn test_a_frozen_observers_box_carries_the_glide_in_bold_white() {
        // The one line of an info box that is a state rather than a reading: while a worldline is
        // frozen on the far branch of r- the box says so, in bold and in white, and while it is
        // falling the box says nothing of the kind. It lives in the box rather than floating at
        // the marker, so it goes where the box goes and nothing in either picture paints over it.
        let metric = KerrSchild::new(1.0, 0.90);
        let frozen = Observer::frozen_bob(&metric);
        let lines = telemetry_lines("Bob", Theme::BOB_COLOR, &frozen, &metric, false);
        let glide = lines.last().expect("the box has lines");
        assert!(glide.text.starts_with("Frozen"), "the last line is the glide: {}", glide.text);
        assert!(glide.bold && glide.color == Color32::WHITE, "and it is bold and white");
        assert_eq!(lines.iter().filter(|l| l.bold).count(), 1, "and it is the only bold line");

        let mut falling = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            9.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::new(1.0, 2.2, false),
        );
        falling.step(&metric, 0.25, 0.25);
        let lines = telemetry_lines("Bob", Theme::BOB_COLOR, &falling, &metric, false);
        assert!(
            lines.iter().all(|l| !l.bold && !l.text.starts_with("Frozen")),
            "a falling observer's box has no such line: {:?}",
            lines.iter().map(|l| l.text.as_str()).collect::<Vec<_>>()
        );
    }

    /// Every shape a painter was handed, flattened out of the nested `Shape::Vec`s.
    fn flatten(shape: &egui::Shape, out: &mut Vec<egui::Shape>) {
        match shape {
            egui::Shape::Vec(inner) => inner.iter().for_each(|s| flatten(s, out)),
            other => out.push(other.clone()),
        }
    }

    /// The main canvas rectangle of a painted frame, read off the one shape that is always there:
    /// the background, which every chart lays down over the whole canvas before anything else. The
    /// radial track under the diagram carries the same fill, so the taller of the two is the one
    /// wanted. Read rather than recomputed, because the layout that decides it - the track's
    /// height, the gap, the panel's margins - is not what any of these tests is about.
    fn canvas_rect(shapes: &[egui::Shape]) -> Rect {
        shapes
            .iter()
            .filter_map(|s| match s {
                egui::Shape::Rect(r) if r.fill == Theme::CANVAS_BG => Some(r.rect),
                _ => None,
            })
            .max_by(|a, b| a.height().total_cmp(&b.height()))
            .expect("every frame paints its canvas background")
    }

    /// Every stroke of a painted frame whose colour is graded along its length. On either of the
    /// two charts this file draws, a `ColorMode::UV` callback means a comet of the global chart or
    /// a crest of a rest-frame chart and nothing else.
    fn faded_strokes(shapes: &[egui::Shape]) -> Vec<PathShape> {
        shapes
            .iter()
            .filter_map(|s| match s {
                egui::Shape::Path(p)
                    if matches!(p.stroke.color, egui::epaint::ColorMode::UV(_)) =>
                {
                    Some(p.clone())
                }
                _ => None,
            })
            .collect()
    }

    /// The alpha such a stroke paints at one point of its own polyline. The callback is the only
    /// place the gradient exists - the tessellator calls it per vertex, and nothing is stored on
    /// the shape - so the fade is measured by asking it, at the ends the eye reads.
    fn alpha_at(stroke: &egui::epaint::PathStroke, at: Pos2) -> u8 {
        match &stroke.color {
            egui::epaint::ColorMode::UV(shade) => shade(Rect::NOTHING, at).a(),
            solid => panic!("expected a graded stroke, got {solid:?}"),
        }
    }

    /// One real frame of the global foliation chart over a transmission of Bob's, flattened.
    ///
    /// The canvas is left at its defaults, so the projection the assertions rebuild - r across
    /// (0, `max_r`), coordinate time up a `time_window`-wide window with the present three tenths
    /// from the top - is the one the app opens on.
    fn distant_view_pass(
        metric: &KerrSchild,
        bob: &Observer,
        field: &SignalField,
        current_time: f64,
    ) -> Vec<egui::Shape> {
        let mut canvas = SpacetimeCanvas::default();
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
            ..Default::default()
        };
        let silent = SignalField::default();
        let output = ctx.run_ui(input, |ui| {
            canvas.render(
                ui,
                metric,
                Some(bob),
                None,
                current_time,
                600.0,
                false,
                ReferenceFrame::DistantObserver,
                1.0,
                SignalViews { alice: &silent, bob: field },
                false,
            );
        });
        let mut shapes = Vec::new();
        for clipped in output.shapes.iter() {
            flatten(&clipped.shape, &mut shapes);
        }
        output.drop_without_applying_deltas();
        shapes
    }

    /// Bob dropped from r = 4.5M at a = 0.90 and transmitting, carried 3 M of coordinate time down
    /// the fall with his field beside him, and the clock they both stand on.
    ///
    /// The step is 0.1 M, five times `TRACK_MIN_DT`, so every `SignalField::advance` records a
    /// track point and the newest point of every live pulse stands at the field's own clock. That
    /// is what lets the assertions place a comet head on the now line exactly rather than within
    /// a recording cadence of it.
    fn falling_transmission() -> (KerrSchild, Observer, SignalField, f64) {
        use crate::physics::observer::WorldlineParams;
        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let dt = 0.1;
        for i in 0..30 {
            let t = ((i + 1) as f64) * dt;
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &bob);
        }
        // The field's own clock, not the loop's: the app paints the chart at the time the field
        // stands on, and thirty additions of 0.1 M land an ulp away from thirty times it.
        let now = field.t;
        (metric, bob, field, now)
    }

    #[test]
    fn test_the_global_chart_marks_each_live_front_with_a_comet_at_the_now_line() {
        // What replaced the wedge. Every pulse that still has a front puts a comet on each of its
        // two edges, at the radius that edge stands at now; nothing is filled between them any
        // more; and each comet is bright at its head and fades away back down the track the edge
        // has already covered, over no more than `Theme::COMET_TAIL_PX` of screen.
        let (metric, bob, field, now) = falling_transmission();
        let shapes = distant_view_pass(&metric, &bob, &field, now);
        let rect = canvas_rect(&shapes);
        let canvas = SpacetimeCanvas::default();
        let (t_min, t_max) = (now - canvas.time_window * 0.7, now + canvas.time_window * 0.3);
        let to_x = |r: f64| rect.left() + (r / canvas.max_r) as f32 * rect.width();
        let to_y = |t: f64| rect.bottom() - ((t - t_min) / (t_max - t_min)) as f32 * rect.height();

        // The fill is gone, in both of the ways it could be looked for: it was laid down as a
        // strip of filled quads, and it was laid down in the emitter's own colour. The filled
        // polygons a frame of this chart still paints are the light cones, which are triangles in
        // the cone hues.
        for shape in &shapes {
            let egui::Shape::Path(path) = shape else { continue };
            let [r, g, b, a] = path.fill.to_srgba_unmultiplied();
            assert!(a == 0 || path.points.len() != 4, "a filled quad is a piece of a wedge fill");
            let bob_rgb = [Theme::BOB_COLOR.r(), Theme::BOB_COLOR.g(), Theme::BOB_COLOR.b()];
            assert!(a == 0 || [r, g, b] != bob_rgb, "a fill in the transmission's own colour");
        }

        // Where the heads have to be: the two edges of every pulse that still has a live ray, at
        // the newest point of its track, which at this step is the field's own clock. The 0.1 M
        // step is five times the recording cadence, so the track and the clock agree here and this
        // test cannot tell which of the two a head is read from;
        // `test_a_comet_head_stands_at_the_live_front_between_two_recorded_track_points` is the one
        // that separates them.
        let live: Vec<(f64, f64, f64)> = field
            .pulses
            .iter()
            .filter(|p| p.rays.iter().any(NullRay::alive))
            .filter_map(|p| p.extent_track.last().copied())
            .collect();
        assert!(live.len() >= 10, "only {} of Bob's pulses still have a front", live.len());
        let mut heads: Vec<Pos2> = Vec::new();
        for &(t, lo, hi) in &live {
            assert_eq!(t, now, "a 0.1 M step records a track point at every advance");
            heads.push(Pos2::new(to_x(lo), to_y(t)));
            heads.push(Pos2::new(to_x(hi), to_y(t)));
        }

        let comets = faded_strokes(&shapes);
        let now_y = to_y(now);
        for comet in &comets {
            let head = comet.points[0];
            assert!((head.y - now_y).abs() < 1e-3, "a head at y = {} off the now line", head.y);
            assert!(
                heads.iter().any(|h| (*h - head).length() < 1e-3),
                "a comet at {head:?}, where no edge of a live front stands"
            );
        }
        for head in heads.iter().filter(|h| rect.contains(**h)) {
            assert!(
                comets.iter().any(|c| (c.points[0] - *head).length() < 1e-3),
                "no comet at {head:?}, where an edge of a live front does stand"
            );
        }

        // The fade itself. The cut is made on arc length along the polyline while the gradient
        // reads straight-line distance from the head, so a curved comet ends a shade above zero;
        // the gap is what the approximation in `fading_line` costs, and it is measured here rather
        // than assumed away.
        let mut full_length = 0;
        let mut worst_end_alpha = 0;
        for comet in &comets {
            let head = comet.points[0];
            let far = *comet.points.last().expect("a comet has two ends");
            let drawn: f32 = comet.points.windows(2).map(|p| p[0].distance(p[1])).sum();
            assert_eq!(alpha_at(&comet.stroke, head), Theme::COMET_HEAD_ALPHA, "full at the head");
            assert!(drawn <= Theme::COMET_TAIL_PX + 1e-3, "a comet {drawn} points long");
            if drawn >= Theme::COMET_TAIL_PX - 1e-3 {
                full_length += 1;
                worst_end_alpha = worst_end_alpha.max(alpha_at(&comet.stroke, far));
            }
        }
        println!(
            "{} comets, {full_length} of them a full {} points long; the dimmest end of those \
             reaches alpha {worst_end_alpha} against the head's {}",
            comets.len(),
            Theme::COMET_TAIL_PX,
            Theme::COMET_HEAD_ALPHA
        );
        assert!(full_length > 0, "some of these tracks are longer than one tail");
        assert!(worst_end_alpha <= 5, "a full tail must reach the background: {worst_end_alpha}");
    }

    #[test]
    fn test_a_comet_head_stands_at_the_live_front_between_two_recorded_track_points() {
        // The head is where the front stands now, not the last place it was written down.
        // `Pulse::extend_track` records at most once per `track_dt` - `TRACK_MIN_DT`, 0.02 M, until
        // the first thinning doubles it - so a caller stepping in shorter intervals than that
        // leaves the newest track point behind the field's clock for several steps together. The
        // app does that routinely: a press at the finest grain is a hundredth of a played frame,
        // far under the cadence, and even a played frame at the default one M per real second is a
        // sixtieth of an M on a 60 Hz window, already under it. A head read off the track is then
        // pinned to that stale point while the now line moves on, and jumps forward when the next
        // point lands: the lag the user sees.
        // Three steps of 0.004 M carry the field 0.012 M past the last recorded point, which is
        // under the 0.02 M cadence, so the clock here stands between two track points by
        // construction.
        let (metric, mut bob, mut field, coarse_now) = falling_transmission();
        let dt = 0.004;
        for i in 0..3 {
            let t = coarse_now + ((i + 1) as f64) * dt;
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &bob);
        }
        let now = field.t;

        // What is drawn, and what the track alone would have drawn: the live extent at the field's
        // clock against the newest recorded point, for every pulse that still has a front.
        let shapes = distant_view_pass(&metric, &bob, &field, now);
        let rect = canvas_rect(&shapes);
        let canvas = SpacetimeCanvas::default();
        let (t_min, t_max) = (now - canvas.time_window * 0.7, now + canvas.time_window * 0.3);
        let to_x = |r: f64| rect.left() + (r / canvas.max_r) as f32 * rect.width();
        let to_y = |t: f64| rect.bottom() - ((t - t_min) / (t_max - t_min)) as f32 * rect.height();
        let now_y = to_y(now);

        let mut heads: Vec<Pos2> = Vec::new();
        let mut worst_lag_m: f64 = 0.0;
        let mut worst_gap_px: f32 = 0.0;
        for pulse in field.pulses.iter() {
            let Some((lo, hi)) = pulse.radial_extent(&metric) else { continue };
            let &(t_rec, lo_rec, hi_rec) = pulse.extent_track.last().expect("a track has its seed");
            worst_lag_m = worst_lag_m.max(now - t_rec);
            for (live_r, recorded_r) in [(lo, lo_rec), (hi, hi_rec)] {
                let live = Pos2::new(to_x(live_r), now_y);
                let recorded = Pos2::new(to_x(recorded_r), to_y(t_rec));
                worst_gap_px = worst_gap_px.max(live.distance(recorded));
                heads.push(live);
            }
        }
        assert!(heads.len() >= 20, "only {} edges of Bob's still have a front", heads.len());
        assert!(
            worst_lag_m > dt,
            "a clock between two track points is the whole point of this test, and the newest \
             point is only {worst_lag_m} M behind"
        );

        let comets = faded_strokes(&shapes);
        println!(
            "{} comets at a clock {worst_lag_m:.4} M past the last recorded point: the stale head \
             sits up to {worst_gap_px:.2} points from the live one at the 14 M window the app \
             opens on, and further in proportion as the user zooms in",
            comets.len()
        );
        assert!(comets.len() >= 10, "only {} comets drawn", comets.len());
        for comet in &comets {
            let head = comet.points[0];
            assert!(
                (head.y - now_y).abs() < 1e-3,
                "a head at y = {} against the now line at {now_y}, {} points of lag",
                head.y,
                now_y - head.y
            );
            assert!(
                heads.iter().any(|h| (*h - head).length() < 1e-3),
                "a comet at {head:?}, where no edge of the live front stands"
            );
        }
        for head in heads.iter().filter(|h| rect.contains(**h)) {
            assert!(
                comets.iter().any(|c| (c.points[0] - *head).length() < 1e-3),
                "no comet at {head:?}, where an edge of the live front does stand"
            );
        }
    }

    #[test]
    fn test_a_pulse_with_no_live_ray_draws_no_comet() {
        // A spent pulse has no front, only a track, and the track is kept because
        // `SignalField::step_back` can bring the rays back. Nothing of it is drawn meanwhile: the
        // same field with every ray killed paints the same chart with no comets on it at all.
        let (metric, bob, mut field, now) = falling_transmission();
        let before = faded_strokes(&distant_view_pass(&metric, &bob, &field, now)).len();
        assert!(before > 0, "the live field draws comets to begin with");
        for pulse in field.pulses.iter_mut() {
            for ray in pulse.rays.iter_mut() {
                ray.death_t = Some(now);
                ray.death_end = Some(crate::physics::wavefront::RayEnd::Ring);
            }
        }
        let after = faded_strokes(&distant_view_pass(&metric, &bob, &field, now)).len();
        println!("{before} comets with the fronts live, {after} with every ray dead");
        assert_eq!(after, 0, "a spent pulse has no front to mark");
    }

    #[test]
    fn test_a_crest_in_a_rest_frame_chart_is_brightest_at_its_future_end() {
        // The same grading, on the other chart. A crest keeps the stroke it has always had - the
        // anchor at its centre and a half length of 0.3 of the canvas either side, so the drawn
        // length is 0.6 of it exactly - and gains a gradient along that stroke: the crest's own
        // colour at the future end, up the screen, falling to nothing at the past end. So a still
        // picture says which way the wave is sweeping over the observer.
        use crate::physics::observer::{ObserverMode, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.0);
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 6.0, 0.0, 0.0, WorldlineParams::default());
        alice.mode = ObserverMode::Static;
        let mut bob =
            Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        bob.mode = ObserverMode::Static;
        let mut field = SignalField::default();
        let dt = 0.1;
        for i in 0..60 {
            let t = ((i + 1) as f64) * dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
            field.advance(&metric, dt);
            field.emit_if_due(&metric, &alice);
            field.detect_receptions(&metric, &bob);
        }

        let mut canvas = SpacetimeCanvas { keep_surface_framed: false, ..Default::default() };
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
            ..Default::default()
        };
        let silent = SignalField::default();
        let output = ctx.run_ui(input, |ui| {
            canvas.render(
                ui,
                &metric,
                Some(&bob),
                Some(&alice),
                6.0,
                600.0,
                false,
                ReferenceFrame::Bob,
                1.0,
                SignalViews { alice: &field, bob: &silent },
                false,
            );
        });
        let mut shapes = Vec::new();
        for clipped in output.shapes.iter() {
            flatten(&clipped.shape, &mut shapes);
        }
        output.drop_without_applying_deltas();

        let rect = canvas_rect(&shapes);
        let crests = faded_strokes(&shapes);
        assert!(crests.len() >= 3, "{} of Alice's crests drawn at Bob", crests.len());
        let full_len = 0.6 * rect.height().min(rect.width());
        for crest in &crests {
            assert_eq!(crest.points.len(), 2, "a crest is one straight null stroke");
            let (a, b) = (crest.points[0], crest.points[1]);
            let drawn = a.distance(b);
            assert!((drawn - full_len).abs() < 1e-3, "{drawn} points against {full_len}");
            // Up the screen is the future, so the smaller y is the end that must be lit.
            let (future, past) = if a.y < b.y { (a, b) } else { (b, a) };
            let (bright, dark) = (alpha_at(&crest.stroke, future), alpha_at(&crest.stroke, past));
            assert!(bright > 0, "a crest lit at its future end");
            assert_eq!(dark, 0, "and run out at its past end, {bright} against {dark}");
        }
    }
}
