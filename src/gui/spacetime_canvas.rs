use crate::gui::axis::{self, SECONDS_PER_YEAR};
use crate::gui::beacon_colour::{self, Beacon};
use crate::gui::controls::{impossible_mode_note, ReferenceFrame, SignalViews};
use crate::gui::polyline::{SCREEN_SPACING, thin_to_pixels};
use crate::gui::ruler;
use crate::gui::theme::Theme;
use crate::physics::as_seen::{
    as_seen, as_seen_youngest, younger_image, AsSeen, AsSeenSeed, NoImage, WorldlineSource,
};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::geodesic::proper_time_between;
use crate::physics::local_frame::{ruler_distance, LocalFrame, SurfaceCharacter};
use crate::physics::observer::{LocalRestFrame, LocalSpeed, Observer, ObserverMode, Who};
use crate::physics::wavefront::{NullRay, Reception, SignalField};
use egui::{epaint::PathShape, Color32, Pos2, Rect, Stroke, Vec2};
use crate::physics::normal_coords::{
    HorizonBranch, SurfaceSampling, affine_length_to_surface, sample_surface_in_plane,
};
use crate::physics::tetrad::Tetrad;
use std::collections::{HashMap, HashSet, VecDeque};

/// Plain-language gloss on every number in a telemetry box, shown on hover.
/// Hover tip for the observer info boxes. Written as a plain multi-line literal (lines start at
/// column 0 so no indentation leaks into the text).
pub const TELEMETRY_HOVER_TIP: &str =
"Drag: move the box anywhere on the canvas. On the equatorial view the box then holds that screen position while the observer moves on; on the (t, r) diagram the box keeps a fixed offset from the observer. Double-click: snap the box back to the observer. Click the triangle at the left of the title line: shut the box down to that title line, or open the box again. A shut box keeps its top-left corner and goes on reading the observer, so the title line stays live. Each canvas remembers box positions, and shut boxes, per observer.

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
"The focus observer's own local chart, built from that observer's orthonormal tetrad: c ≡ 1, so light cones stand at 45° and every worldline through the event stands steeper.

The horizontal axis is the line of sight, and right is outward. A rest frame has three spatial directions and this canvas has room for one, so the view draws the plane spanned by the focus observer's time axis and the direction the other observer's light arrives from, turned so that its outward component points right. The other observer's dot then lies on the right-hand edge of the past cone when they are outward of the focus observer and on the left-hand edge when they are inward, and the header states the angle between the line of sight and the nearer radial direction, prograde or retrograde. With no other observer on the canvas, or with no image of one, the horizontal axis falls back to the outward radial direction and the header says so.

Why the line of sight rather than the radial axis. Light from an observer on a fast orbit arrives strongly aberrated, so most of the offset to the emission event points round the hole rather than outward. The radial plane dropped that part: the dot sat inside the 45° past cone instead of on it, and the shortening could carry the dot past a drawn horizon curve while the emission event lay outside that horizon. In the plane of the line of sight nothing is dropped.

Two constructions share this canvas, and the canvas does not pretend otherwise.

Exact, in Riemann normal coordinates. The view draws each surface r = const — the horizons, the static limit, the ring singularity — as the curve that surface really is: the point of a surface at chart angle ψ sits at the affine length of the geodesic that leaves the focus observer's event in the direction ψ, inside the drawn plane, and arrives on that surface. Where the curve meets the now-axis is the ruler distance the horizon box prints, and the two agree because the two are the same integral. The other observer's dot carries the same construction: the dot marks the event on the other worldline that the focus observer's past light cone passes through, and stands at the affine length of the arriving light ray, in the direction the ray arrives from — which in this plane is the horizontal axis itself, so the dot lands exactly on the 45° past cone. The dot and the surface curves then measure along the very same directions, so the dot stands beyond a curve exactly when the light really did come from beyond that surface along that ray.

A horizon is two surfaces, and the curve says which. The heavy stroke is the branch the observer's own future turns on — the future horizon for r₊, the far branch for r₋, the one a worldline freezes on — and the light stroke of the same colour is the other branch, the past horizon of r₊ that nothing crosses, or the branch of r₋ an infaller has already come through. The two meet at a corner, and that corner is the bifurcation, where the arriving geodesic runs tangent to the horizon rather than through it. Each horizon's box names the branches on the canvas.

First order, from the tetrad at the focus observer's own event. The distant clock's grid lines — the surfaces t = const — come through the linear map as straight lines, and so do the signal crests. Each of those lines is exact where it crosses the focus observer's worldline, which is the one place the view reads it, and linearised away from there.";

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
/// How much of the focus observer's coordinate time passes between sweeps for a younger image
/// of the other observer, in M. An orbit of the prograde ISCO of an a = 0.90 hole is 28 M, so a
/// hand-over between images is late by a few per cent of an orbit at worst.
const YOUNGEST_CHECK_INTERVAL: f64 = 1.0;

/// How far the focus observer's own clock has to move, in M of proper time, before the picture
/// of the other observer is solved again rather than held from the last solve.
///
/// What an observer sees is a function of their own event. Two events on one worldline this
/// close together see pictures that differ by that much and no more, which is far under a screen
/// point at any zoom; so between them the last solve *is* the picture. It matters for one
/// observer only: one freezing onto the far branch of r-, whose u^t climbs through 1e6 and on to
/// 1e10 while their clock all but stops - a few 1e-8 M a frame - and whose frame is by then
/// boosted past what the solver can resolve. Measured with Bob on the E = 1, L = 2.2 worldline
/// watching Alice's raindrop: the picture was steady (g = 0.028, lambda = 0.626 M) and the solve
/// still came back "no image" on a third of the frames from u^t ~ 8e6 to the stall, and where it
/// did come back its g wandered by 15%. Every one of those frames sits inside 1e-6 M of Bob's
/// clock. Holding the picture is the exact statement that nothing has changed, and it also
/// keeps the drawn plane from snapping to the radial one and back as the solve comes and goes.
const HELD_PICTURE_TAU: f64 = 1e-3;

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

A surface r = const crosses the observer's own time axis at the affine length of the observer's own worldline from here to the crossing, which for a worldline is the proper time the observer has left before reaching that surface. The framing follows that one number - the same number the drawn curve crosses the axis at, so the framing and the picture agree - and that is why the framing needs no special case for the last moments. On the approach to the far branch of r-, where the gap closes like exp(-kappa_- t) while the observer's remaining proper time shrinks alongside, following that number is the only way to watch both at once.

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
/// Either kind of box is left alone until the user actually drags it: a placement is what
/// `is_placed` reads, and a box nobody has touched has to go on counting as untouched, or
/// `TelemetryBoxes::flush` stops nudging it clear of its neighbours after its first frame. From the
/// first drag on, a following box records its offset every frame, so a drag against the canvas
/// edge does not build up an offset that snaps back later, and a pinning box records where it
/// stands relative to the canvas every frame, which is what keeps a box that a resize has pushed
/// inward from jumping back out again.
fn remembered_placement(
    pin_on_drag: bool,
    dragged: bool,
    previous: Option<Placement>,
    moved_min: Pos2,
    anchored: Pos2,
    canvas_min: Pos2,
) -> Option<Placement> {
    if !dragged && previous.is_none() {
        None
    } else if pin_on_drag {
        Some(Placement::Pinned(moved_min - canvas_min))
    } else {
        Some(Placement::Offset(moved_min - anchored))
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
/// box's identity, and the half that outlives every rewording of a title or of a line under it -
/// the Cauchy box quotes the current r₋ in kilometres, and is this same box whatever that says.
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
    /// The canvas's own legend: what the picture is, its scale and its keys. The equatorial view
    /// and the 2D+1 volume each have one.
    Legend,
}

impl BoxId {
    /// Every box there is, for iterating the set, read by `crate::save` exactly as `Canvas::ALL`
    /// is.
    pub const ALL: [Self; 9] = [
        Self::Observer(Who::Alice),
        Self::Observer(Who::Bob),
        Self::Signal(Who::Alice),
        Self::Signal(Who::Bob),
        Self::OuterHorizon,
        Self::CauchyHorizon,
        Self::Ergosphere,
        Self::RingSingularity,
        Self::Legend,
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
            Self::Legend => "legend",
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
    /// The boxes the user has shut down to their title lines, under the same key as a placement
    /// and saved beside one. A set of its own rather than a flag on the placement: a box can be
    /// shut without ever having been dragged, and `is_placed` means dragged.
    pub(crate) collapsed: HashSet<(Canvas, BoxId)>,
}

impl TelemetryBoxes {
    /// Boxes that stay where the user drops them, however the observer moves afterwards.
    pub fn pinning() -> Self {
        Self { pin_on_drag: true, ..Self::default() }
    }

    /// The same boxes with every box on `canvases` shut down to its title line, which is how the
    /// app's own canvases start: a fresh picture shows the names of what can be read, and the
    /// triangle on a title line opens that one box. The canvases are named rather than taken
    /// from `Canvas::ALL` because the set is saved as it stands, and a canvas should not write
    /// boxes of a picture it never draws into its file.
    pub fn starting_shut(mut self, canvases: &[Canvas]) -> Self {
        for &canvas in canvases {
            self.collapsed.extend(BoxId::ALL.into_iter().map(|id| (canvas, id)));
        }
        self
    }

    /// Paint every deferred info box, in order, once the rest of the canvas is down.
    ///
    /// A box still sitting where it was put is nudged down clear of the ones already placed, so a
    /// stack of surfaces whose lines all leave by the same corner stays readable, and so does an
    /// observer's box whose marker has carried it over a surface's or over the other observer's.
    /// One the user has
    /// dragged is left exactly where they dragged it, overlap or not: they can see the overlap and
    /// they chose it.
    pub(crate) fn flush(
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
            // The size as drawn, so that a row of shut boxes packs as a row of title lines and not
            // as the open boxes they are not showing.
            let shut = self.collapsed.contains(&(canvas, b.id));
            let size = drawn_box_size(painter, &b.lines, font_scale, shut);
            let was_placed = self.is_placed(canvas, b.id);
            let anchor = if was_placed {
                b.anchor
            } else {
                // Slide past whatever is in the way, by no more than clears it: down first, and up
                // if the bottom of the canvas stops the box before it is clear. A box that cannot
                // be cleared either way stays where it was asked for.
                let start = clamp_into(Rect::from_min_size(b.anchor, size), rect);
                let slide = |down: bool| -> Option<Rect> {
                    let mut r = start;
                    for _ in 0..16 {
                        let Some(p) = occupied.iter().find(|p| p.intersects(r)) else {
                            return Some(r);
                        };
                        let y = if down { p.bottom() + 6.0 } else { p.top() - 6.0 - size.y };
                        let next = clamp_into(Rect::from_min_size(Pos2::new(r.left(), y), size), rect);
                        if next == r {
                            return None;
                        }
                        r = next;
                    }
                    None
                };
                slide(true).or_else(|| slide(false)).unwrap_or(start).min
            };
            let response = self.show_lines(
                ui, painter, canvas, b.id, rect, anchor, &b.lines, b.color, font_scale, b.tip,
            );
            // A drag that began this frame measured its offset from the nudged anchor, and every
            // later frame resolves it from the plain one. Rebase it once, here, or the box jumps
            // back by the nudge on the second frame of the drag.
            if !was_placed
                && let Some(Placement::Offset(v)) = self.placements.get_mut(&(canvas, b.id))
            {
                *v += anchor - b.anchor;
            }
            occupied.push(response.rect);
        }
    }

    /// Whether this box stands shut down to its title line.
    pub fn is_shut(&self, canvas: Canvas, id: BoxId) -> bool {
        self.collapsed.contains(&(canvas, id))
    }

    /// Shut this box or open it, as the triangle on its title line does.
    pub fn set_shut(&mut self, canvas: Canvas, id: BoxId, shut: bool) {
        if shut {
            self.collapsed.insert((canvas, id));
        } else {
            self.collapsed.remove(&(canvas, id));
        }
    }

    /// Whether this box has been dragged somewhere and left there.
    pub fn is_placed(&self, canvas: Canvas, id: BoxId) -> bool {
        self.placements.contains_key(&(canvas, id))
    }

    /// Draw one box of arbitrary content at `anchored`, with the drag, the double-click reset, the
    /// disclosure triangle and the remembered placement every box on these canvases gets. An
    /// observer's telemetry, a surface's or a horizon's reading, a signal and a canvas's legend are
    /// all this with their own lines in it. Every info box on every canvas comes through here, which is why the triangle
    /// and its hit test are written here once.
    ///
    /// `anchored` is always measured on the whole box, open or shut, and the caller measures it:
    /// a box anchored beside its marker is placed above or to the left of that marker by its own
    /// height and width, so anchoring the short box would land a shut box somewhere the open one
    /// never stood. Anchoring both on the open size leaves a shut box exactly where the open box's
    /// title line was, and `clamp_into` then slides whichever box is actually drawn back inside
    /// the canvas.
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
        let mut collapsed = self.collapsed.contains(&key);
        let size = drawn_box_size(painter, lines, font_scale, collapsed);
        let placed = resolve_placement(previous, anchored, canvas_rect.min);
        let badge_rect = clamp_into(Rect::from_min_size(placed, size), canvas_rect);

        let widget_id = ui.id().with(("telemetry", canvas.key(), id.key()));
        // click_and_drag rather than drag alone: egui only reports a double-click on a widget that
        // senses clicks, and the double-click is what resets the offset.
        let response = ui.interact(badge_rect, widget_id, egui::Sense::click_and_drag());
        // The triangle registers after the box for the reason the box registers after the canvas:
        // within a layer egui hands an overlapping press to the widget registered last. So a press
        // on the triangle reaches the triangle rather than starting a drag of the box or counting
        // towards the box's double-click, and a press anywhere else on the box still reaches the
        // box. `Sense::click` against the box's `click_and_drag` obeys the same order.
        let toggle = ui.interact(
            disclosure_hit_rect(badge_rect, font_scale),
            widget_id.with("disclosure"),
            egui::Sense::click(),
        );
        let toggled = toggle.clicked();
        let over_triangle = toggle.contains_pointer();

        let badge_rect = if toggled {
            // The corner the user is looking at holds still: the box is redrawn from the same
            // top-left, shorter or taller, rather than re-anchored at its new size. Nothing is
            // written to the placement this frame, so a click on the triangle can neither move a
            // box nor forget where a dragged one was put.
            collapsed = !collapsed;
            if collapsed {
                self.collapsed.insert(key);
            } else {
                self.collapsed.remove(&key);
            }
            let size = drawn_box_size(painter, lines, font_scale, collapsed);
            clamp_into(Rect::from_min_size(badge_rect.min, size), canvas_rect)
        } else if response.double_clicked() {
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

        // A shut box paints its title line and nothing else, and no body line is measured or laid
        // out on the way there - `drawn_box_size` asked the title alone for the size.
        let drawn = if collapsed { &lines[..lines.len().min(1)] } else { lines };
        paint_telemetry_box(painter, badge_rect, color, drawn, font_scale, collapsed);
        let _ = toggle.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(if collapsed {
            "Expand this box"
        } else {
            "Collapse this box to its title line"
        });
        // The box's own tip belongs to the rest of the box: two tooltips over one triangle would
        // ask the reader which of the two answers the triangle.
        if over_triangle {
            response
        } else {
            response.on_hover_text(tip)
        }
    }
}

/// An info box worked out while its subject was being drawn and painted only once everything else
/// on the canvas is down. Deferring them is what makes the opaque fill mean anything: a box painted
/// in the middle of the pass gets a worldline, a light cone or a grid line drawn straight across it.
pub(crate) struct PendingBox {
    /// Also what its remembered position is filed under, so it has to be unique per canvas. The
    /// title the box prints is the first of its `lines` and is no part of this.
    id: BoxId,
    anchor: Pos2,
    lines: Vec<TelemetryLine>,
    color: Color32,
    tip: &'static str,
}

impl PendingBox {
    /// A canvas's legend as a deferred box: a title, and under it the block of text the canvas
    /// used to paint for itself, one box line per line of `body`. It stands at `anchor` until
    /// dragged, and goes first in the queue so that the other boxes are slid clear of it.
    pub(crate) fn legend(anchor: Pos2, title: &str, body: &str, tip: &'static str) -> Self {
        let line = |text: &str, is_title: bool| TelemetryLine {
            text: text.to_string(),
            color: Theme::TEXT_BRIGHT,
            is_title,
            bold: false,
        };
        let mut lines = vec![line(title, true)];
        lines.extend(body.lines().map(|text| line(text, false)));
        Self { id: BoxId::Legend, anchor, lines, color: Theme::CHIP_OUTLINE, tip }
    }

    /// An observer's telemetry as a deferred box, anchored beside their marker at `pos`. Every
    /// canvas sends the observers' boxes through the same queue as its other boxes, so that `TelemetryBoxes::flush` keeps
    /// every untouched box on the canvas clear of every other. `extra` is whatever the canvas has
    /// to say about this observer that is true only on that canvas, printed under the telemetry.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn observer(
        painter: &egui::Painter,
        canvas_rect: Rect,
        who: Who,
        pos: Pos2,
        title: &str,
        color: Color32,
        obs: &Observer,
        metric: &KerrSchild,
        use_physical_units: bool,
        font_scale: f32,
        extra: Vec<TelemetryLine>,
    ) -> Self {
        let mut lines = telemetry_lines(title, color, obs, metric, use_physical_units);
        lines.extend(extra);
        let size = telemetry_box_size(painter, &lines, font_scale);
        Self {
            id: BoxId::Observer(who),
            anchor: default_badge_pos(canvas_rect, pos, size),
            lines,
            color,
            tip: TELEMETRY_HOVER_TIP,
        }
    }

    /// The same box under a different hover gloss. The rest-frame view's box for the *other*
    /// observer carries the ordinary telemetry lines and a great deal that the ordinary tip says
    /// nothing about, so that box answers with `AS_SEEN_BOX_TIP` instead.
    pub(crate) fn with_tip(mut self, tip: &'static str) -> Self {
        self.tip = tip;
        self
    }
}

/// One printed line of a telemetry box: the text, its colour and whether it is the title.
pub(crate) struct TelemetryLine {
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

/// The one layout every info box is measured and painted by: the padding inside the frame, the
/// step from one printed line to the next, and the column the disclosure triangle stands in at the
/// left of the title line. Measuring and painting have to agree to the point, or the triangle
/// lands somewhere other than the room reserved for it and the title text starts over the
/// triangle, so both ask here rather than each spelling the numbers out.
struct BoxMetrics {
    pad_x: f32,
    pad_y: f32,
    line_spacing: f32,
    /// How far right the title line's text sits, which is how much wider the title line makes the
    /// box. Every box pays for the column in both states: a box that changed width as it shut
    /// would move its own right-hand edge for no reason the user asked for.
    column: f32,
}

fn box_metrics(font_scale: f32) -> BoxMetrics {
    let font_scale = font_scale.clamp(0.7, 2.0);
    BoxMetrics {
        pad_x: 10.0 * font_scale,
        pad_y: 6.0 * font_scale,
        line_spacing: 13.0 * font_scale,
        column: 18.0 * font_scale,
    }
}

/// The smallest comfortable click target, in points. The drawn triangle is smaller than this at
/// every font scale: a mark that reads well at ten points is still a mark the mouse has to find,
/// so the click area is the title row's left gutter grown to this square rather than the outline
/// of the art.
const DISCLOSURE_HIT: f32 = 16.0;

/// Where the disclosure triangle is drawn: a small square centred in the title row's left gutter.
fn disclosure_art_rect(badge_rect: Rect, font_scale: f32) -> Rect {
    let m = box_metrics(font_scale);
    let scale = font_scale.clamp(0.7, 2.0);
    Rect::from_center_size(
        Pos2::new(
            badge_rect.left() + m.pad_x + m.column * 0.5,
            badge_rect.top() + m.pad_y + m.line_spacing * 0.5,
        ),
        Vec2::new(9.0 * scale, 7.0 * scale),
    )
}

/// What a click on the triangle has to land in: the title row's gutter, out to where the title's
/// own text begins and down the row, grown to `DISCLOSURE_HIT` where the gutter is smaller than
/// that and then held inside the box. It stops at the text so that a drag started on the title
/// still moves the box, and it is held inside the box so that it never takes a press meant for
/// the canvas underneath.
fn disclosure_hit_rect(badge_rect: Rect, font_scale: f32) -> Rect {
    let m = box_metrics(font_scale);
    let text_left = badge_rect.left() + m.pad_x + m.column;
    let width = DISCLOSURE_HIT.max(text_left - badge_rect.left());
    let height = DISCLOSURE_HIT.max(m.pad_y + m.line_spacing);
    Rect::from_min_max(
        Pos2::new(text_left - width, badge_rect.top()),
        Pos2::new(text_left, badge_rect.top() + height),
    )
    .intersect(badge_rect)
}

/// The size the box is drawn at this frame: the whole card, or the title line alone.
fn drawn_box_size(
    painter: &egui::Painter,
    lines: &[TelemetryLine],
    font_scale: f32,
    collapsed: bool,
) -> Vec2 {
    match (collapsed, lines.first()) {
        (true, Some(title)) => collapsed_box_size(painter, title, font_scale),
        _ => telemetry_box_size(painter, lines, font_scale),
    }
}

/// Width fitted to the longest line, height to the actual number of lines.
fn telemetry_box_size(painter: &egui::Painter, lines: &[TelemetryLine], font_scale: f32) -> Vec2 {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let m = box_metrics(font_scale);
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
            let text_w = painter.layout_no_wrap(line.text.clone(), font, line.color).size().x;
            // The title line carries the triangle's column, so the box is wide enough for both
            // whether the title is the longest line or not.
            if line.is_title { text_w + m.column } else { text_w }
        })
        .fold(0.0_f32, f32::max);

    Vec2::new(
        (max_text_w + m.pad_x * 2.0).max(180.0 * font_scale),
        (m.pad_y * 2.0 + m.line_spacing * (lines.len() as f32 - 0.2)).max(56.0 * font_scale),
    )
}

/// The size of a box shut down to its title line: that line, the triangle's column and the same
/// padding, and neither of the floors the open box carries. Those floors keep a column of readings
/// in one shape; a shut box is a label with a handle on it, and padding a label out to the width
/// of the readings it is hiding would say the readings are still there.
fn collapsed_box_size(painter: &egui::Painter, title: &TelemetryLine, font_scale: f32) -> Vec2 {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let m = box_metrics(font_scale);
    let (font_title, _) = telemetry_fonts(font_scale);
    let font = if title.bold {
        bold_font(painter, Theme::MIN_FONT_PT * font_scale)
    } else {
        font_title
    };
    let text_w = painter.layout_no_wrap(title.text.clone(), font, title.color).size().x;
    Vec2::new(
        text_w + m.column + m.pad_x * 2.0,
        m.pad_y * 2.0 + m.line_spacing * 0.8,
    )
}

/// Paint the card, the disclosure triangle and the lines handed in - which is the title line alone
/// when the box stands shut.
fn paint_telemetry_box(
    painter: &egui::Painter,
    badge_rect: Rect,
    color: Color32,
    lines: &[TelemetryLine],
    font_scale: f32,
    collapsed: bool,
) {
    let font_scale = font_scale.clamp(0.7, 2.0);
    let m = box_metrics(font_scale);
    let (font_title, font_body) = telemetry_fonts(font_scale);

    // Under the box and before it, so it falls on the chart and not on the card. Boxes can
    // overlap, and when they do the upper one's shadow lands on the lower one, which is what a
    // shadow does. A shut box keeps the frame, the fill, the corner and the shadow of the open
    // one: what changes is how much of the box there is, not what kind of thing the box is.
    painter.add(BOX_SHADOW.as_shape(badge_rect, BOX_CORNER_RADIUS * font_scale));
    painter.rect_filled(badge_rect, BOX_CORNER_RADIUS * font_scale, Color32::from_black_alpha(230));
    painter.rect_stroke(
        badge_rect,
        BOX_CORNER_RADIUS * font_scale,
        Stroke::new(1.2, color),
        egui::StrokeKind::Inside,
    );

    // The disclosure triangle, drawn rather than set in a glyph: the bundled fonts carry no arrow,
    // and a test context carries no font at all. Down while the box is open and right while the
    // box is shut, which is the triangle every file list turns.
    let art = disclosure_art_rect(badge_rect, font_scale);
    let points = if collapsed {
        vec![art.left_top(), art.left_bottom(), Pos2::new(art.right(), art.center().y)]
    } else {
        vec![art.left_top(), art.right_top(), Pos2::new(art.center().x, art.bottom())]
    };
    let triangle_colour = lines.first().map_or(color, |line| line.color);
    painter.add(PathShape::convex_polygon(
        points,
        triangle_colour,
        egui::epaint::PathStroke::NONE,
    ));

    for (i, line) in lines.iter().enumerate() {
        let font = if line.bold {
            bold_font(painter, Theme::MIN_FONT_PT * font_scale)
        } else if line.is_title {
            font_title.clone()
        } else {
            font_body.clone()
        };
        let indent = if line.is_title { m.column } else { 0.0 };
        painter.text(
            Pos2::new(
                badge_rect.left() + m.pad_x + indent,
                badge_rect.top() + m.pad_y + m.line_spacing * i as f32,
            ),
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
"What the surface is, read straight off the slope its trace has where that trace passes your own event. Steeper than 45 degrees means timelike - the world-tube of observers holding that radius, something a rocket can stay off. Exactly 45 degrees means null. Flatter than 45 degrees means spacelike: not a place at all but a moment of your history, which arrives whatever you do. Nothing about the tilt goes in by hand; the tilt follows from the metric at your own radius through the dual tetrad, so the reading stays exact at the dot.

The reading belongs to the trace and not always to the surface. This canvas draws one plane through your event - the plane of your time axis and your line of sight to the other observer - and what you see of a surface is where that plane cuts it. The cut of a plane by a plane is still a line, so there is always a slope to read, and where your line of sight runs along the outward radial direction the line is the surface's own and the two readings agree. Where the line of sight has swung round, the plane cuts the surface at an angle, and such a cut can only flatten a trace, never steepen it: the trace of a null surface is a spacelike line, flatter than 45°, or exactly null when the plane holds the surface's own null generator, and a trace steeper than 45° belongs only to a timelike surface, such as r = const outside r₊. No sub-light path ever slips past a horizon drawn here. The box reports the trace, honestly, and the region tag in your own title reports the surface.

The box takes that reading at your own event and nowhere else. The drawn curve gives the exact position of the surface in your chart, and a curve bends: far from you the same surface tilts differently, and that far tilt says something about a distant event rather than about the event you are standing on.

Moving at / closing at - for a timelike trace, the speed at which that world-tube crosses this frame; for a spacelike one, your own speed relative to the observers whose simultaneity slice the surface is. The same slope gives both.";

/// Hover gloss on the other observer's box in a rest-frame view, which reports the event that
/// observer is *seen* at rather than the event that observer is at.
pub const AS_SEEN_BOX_TIP: &str =
"As seen - defined as the event on the other observer's worldline that lies on this observer's past light cone. Nobody sees anybody now; what arrives at an event is light, and this box reads the event that light left. The direct image only: a ray that has wound round the hole and come back would deliver a second, older picture, and this box ignores that second picture.

Where the dot stands. The view follows the arriving ray back to the emission event and puts the dot where the exact normal-coordinate rule puts that event: at the affine length of that ray, in the direction the light arrives from. Normalise the ray so that this observer's own clock measures unit frequency on the ray, and the affine length λ becomes at once how far away the emission event lies and how long ago the emission event happened - which is exactly what puts the dot on the 45° past cone. The thin line from the dot to this observer's event is that ray: in normal coordinates a null geodesic runs as a straight 45° line, so the line is the light itself rather than an annotation over the top of the picture.

The picture drops nothing. This canvas draws the plane spanned by the focus observer's time axis and this very line of sight, so the whole of the offset to the emission event lies in the picture: the dot stands at (±λ, −λ), on the 45° past cone, on the right-hand side when the other observer is outward of you and on the left-hand side when they are inward. The surface curves sweep the same plane, so the dot and each curve are affine lengths along the same directions, and the dot stands beyond a curve exactly when the light really did come from beyond that surface.

line of sight

The angle between the arriving light and the focus observer's own outward radial direction, prograde positive. It is the direction the horizontal axis of this canvas points in, and on a fast orbit aberration swings it a long way round: an observer on the prograde ISCO of an a = 0.9 hole looks back over one shoulder to see a companion that a chart would place straight below.

light left - λ, as a time ago on this observer's own clock and as a distance away on this observer's own ruler. One number, because in normal coordinates a null geodesic makes those the same number.

Every ordinary telemetry line above reads the other observer at the emission event and not now: the radius, the rates, the speeds and the region tag in the title are all what was true where the light left.

watch - the other observer's own clock at the emission event. Before the run began means the light left before the release, on the backward extension of the hold, and that extension is what makes anybody visible at all at t = 0.

1 + z and g - g is ν_seen/ν_emitted, which the arriving ray carries exactly rather than a formula supplying it, and 1 + z is one over g. Below 1 in g means a redshift.

beacon - each observer carries a beacon whose rest wavelength is the dominant wavelength of that observer's own marker colour. The view paints the dot the colour that beacon arrives in: the seen wavelength is the rest wavelength divided by g, the CIE 1931 colour matching functions give that wavelength its chromaticity, and the dot's brightness is the bolometric factor g⁴ times the eye's own response at the seen wavelength against the eye's response at the rest wavelength. Once the seen light leaves the visible band, or once what is left of the light falls below what a screen can show against this background, the dot becomes a dashed grey ring at the same exact position: the light still arrives, and the eye no longer has anything to see.

brightness and watch rate - the flux factor g⁴, and the rate this observer sees the other observer's clock run at, which is g. Both numbers belong to what this observer receives rather than to anything the other observer does.

The two cases worth watching. An observer who stays outside r₊ never sees the other cross it: the seen radius falls towards r₊ and the dot piles up where the inward half of the past cone meets the drawn r₊ curve, dimming without limit. An observer who falls in sees the crossing happen exactly as that observer crosses r₊, because r₊ is a null surface whose generators stand still in r, so light let go on the horizon waits there until the next worldline arrives.";

/// The Cauchy box's border. Red rather than the magenta of the r₋ line itself: the line is the
/// geometry, the box is the warning.
const HORIZON_BOX_RED: Color32 = Color32::from_rgb(255, 60, 60);


pub const HORIZON_BOX_TIP: &str =
"Whether a horizon is a place or a moment is not a matter of taste: the answer is whether r runs spacelike or timelike between you and that horizon.

Outside r₊ and inside r₋ the metric function Δ = r² − 2Mr + a² stays positive, r is an ordinary radial direction, and a surface r = const is a timelike world-tube — something that persists, that you can hold station beside, and that has a distance. Between the horizons Δ < 0, r turns timelike, and that same surface becomes a moment of your history instead: the surface arrives, no rocket hovers beside the surface, and asking how far away the surface lies has no answer. The read-out flips at each horizon because the geometry flips there.

Ruler Distance — defined as the arclength of the spacelike geodesic that leaves your event along your own radial axis and runs until meeting the surface: the radial coordinate of Fermi normal coordinates built on your tetrad. That construction performs the length contraction exactly. Do not read the figure as the static observers' chain of rulers divided by your Lorentz factor, which rescales somebody else's ruler and answers a different question; unlike that chain, this geodesic survives inside the ergosphere, where nothing can hold station to lay rulers out. The figure does assume a simultaneity — your own — because the question “how far away is that surface right now” carries no meaning without one.

Time — the proper time on your own watch between here and the crossing, ∫ r² dr / √R with R = r⁴(dr/dτ)², integrated along the worldline your E and L put you on. R is a square and never changes sign, which is why a horizon has a time even where that horizon has no distance, while the distance integral carries a √Δ that goes imaginary throughout Region II.

“beyond r₊” — the path from here to r₋ would have to cross Region II, so no spacelike curve in your rest space reaches r₋ and the integral has nothing to return. r₋ does not lie on your worldline yet either, and whether r₋ ever will depends on what you do next.

heavy / light — a horizon is two surfaces, and on the rest-frame view this line says which of them the canvas holds. The heavy stroke is the branch your own future turns on: the future horizon of r₊, and the far branch of r₋ that a worldline with E − Ω₋L < 0 settles onto for ever. The light stroke of the same colour is the other branch: the past horizon of r₊, which no worldline crosses and which your past light cone merely runs down onto, and the branch of r₋ an infaller has already come through. Where both appear the curve has a corner between them. That corner is the bifurcation — defined as the one direction whose geodesic arrives tangent to the horizon rather than through it — and it is geometry rather than a kink in the drawing: at that direction the quantity P(r_H) = E(r_H² + a²) − aL of the arriving geodesic passes through zero, which is exactly the test that decides which crossings this chart has.";

/// Hover tip for the static limit's box on the (t, r) chart, which quotes a radius and nothing
/// that depends on an observer.
pub const ERGOSPHERE_CHART_TIP: &str =
"The static limit r_E — defined as the radius where the Killing vector ∂_t turns null, g_tt = 0. In the equatorial plane that radius is 2M whatever the spin.

Inside r_E frame dragging carries every timelike worldline round with the hole, so no rocket holds a fixed azimuth against the distant stars. A rocket inside r_E can still hold a fixed radius and can still climb back out, so r_E is a place on this chart and not a horizon.";

/// A surface's box where no observer is being quoted: its name, and its radius on the line under
/// the name, so that a shut box is the name alone.
fn surface_radius_lines(title: &str, radius: &str, color: Color32) -> Vec<TelemetryLine> {
    vec![
        TelemetryLine { text: title.to_string(), color, is_title: true, bold: false },
        TelemetryLine { text: radius.to_string(), color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
    ]
}

/// The lines of a horizon's box: what the surface is called, its radius where the caller quotes
/// one (the (t, r) chart does, the rest-frame view does not), whether it is a place or a moment
/// from where this observer stands, and the one number that reading admits.
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
    radius: Option<&str>,
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

    let body = |text: String| TelemetryLine { text, color: Theme::TEXT_BRIGHT, is_title: false, bold: false };
    let mut lines = vec![TelemetryLine { text: title.to_string(), color, is_title: true, bold: false }];
    // The radius goes under the title rather than in it, so that a shut box is its name alone.
    lines.extend(radius.map(|r| body(r.to_string())));
    lines.push(body(kind));
    lines.push(body(detail));
    lines
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

/// Which region of the geometry a radius lies in, as the title line of an observer's box names it.
///
/// Its own function because two titles carry it: the observer's box, which names the region the
/// observer stands in now, and the *as seen* box, which names the region the other observer stood
/// in when the light left - a different event, and often a different region.
fn region_tag(metric: &KerrSchild, r: f64) -> &'static str {
    if r > metric.ergosphere_equatorial() {
        "Region I"
    } else if r > metric.outer_horizon() {
        "Ergo"
    } else if r > metric.inner_horizon() {
        "Region II (Trapped)"
    } else {
        "Region III (Core)"
    }
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

    let mut lines = vec![
        TelemetryLine {
            text: format!("{} [{}]", name, region_tag(metric, obs.r)),
            color,
            is_title: true,
            bold: false,
        },
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
    /// The last frame's answer for where each observer is seen from each other observer's event,
    /// kept so that `as_seen` can be warm-started. A warm solve is a Newton from a seed that is one
    /// frame stale and costs about fourteen microseconds; a cold one has to fan two dozen rays
    /// across the whole past cone first and costs nearly two milliseconds. It is view state and
    /// nothing a save file would hold: a stale seed costs one failed Newton and the cold path picks
    /// the answer up again.
    seen_seeds: HashMap<(Who, Who), AsSeenSeed>,
    /// The focus observer's coordinate time at which each pair's image was last checked against
    /// the sweep for a younger one (`as_seen::younger_image`). A warm solve follows one image
    /// continuously, and for an observer who goes round the hole the image continuity follows
    /// winds up while a younger one is born beside it; the sweep is what notices, and it costs a
    /// few milliseconds, so it runs once per `YOUNGEST_CHECK_INTERVAL` of the focus observer's
    /// time rather than once a frame. View state, like the seeds.
    seen_checked: HashMap<(Who, Who), f64>,
    /// The last solved picture of each pair and the focus observer's proper time it was solved
    /// at, held and redrawn until that clock has moved `HELD_PICTURE_TAU`. View state.
    seen_held: HashMap<(Who, Who), (AsSeen, f64)>,
    /// The drawn surface curves, and what they were sampled for. Sampling all four surfaces costs
    /// about 270 microseconds, which is worth paying once per change of the observer's event and
    /// not once per frame: with the run paused, every frame after the first reuses this.
    surfaces: SurfaceCurves,
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
            telemetry: TelemetryBoxes::default()
                .starting_shut(&[Canvas::Spacetime, Canvas::RestFrame]),
            seen_seeds: HashMap::new(),
            seen_checked: HashMap::new(),
            seen_held: HashMap::new(),
            surfaces: SurfaceCurves::default(),
        }
    }
}

/// Everything a sampled set of surface curves depends on. Two keys that compare equal name the same
/// curves, so the sampler is skipped; anything else and the curves are taken again.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SurfaceKey {
    /// The hole, which the observer cards can change under the view.
    m: f64,
    a: f64,
    /// The chart's origin: the observer's radius and 4-velocity. Both move on every played frame,
    /// which is why the cache pays for itself when the run is paused rather than when it is
    /// running.
    r0: f64,
    u: [f64; 3],
    /// The spacelike leg of the drawn plane, in coordinate components: the line of sight to the
    /// other observer, or the radial leg when there is no image to look along. The curves are a
    /// slice of each surface by that plane, so a change of plane is a change of curve.
    s: [f64; 3],
    /// The window the sampler was asked for, quantised by `sampling_window` so that the automatic
    /// zoom's own per-frame creep does not invalidate the curves on its own.
    window: f64,
}

/// One drawable run of a surface curve: the branch of the target horizon it reached, and its
/// points in the observer's drawn plane.
struct DrawnRun {
    /// `None` for a surface that is not a horizon, and for a run that is nothing but the
    /// bifurcation point the two branches share.
    branch: Option<HorizonBranch>,
    points: Vec<[f64; 2]>,
}

/// The four surfaces as polylines in the observer's (xi^1, xi^0) plane, with the key they were
/// sampled under.
#[derive(Default)]
struct SurfaceCurves {
    key: Option<SurfaceKey>,
    /// One entry per surface of the view's `surfaces` table, each a list of open runs of exact
    /// points. A run is never closed into a loop: see `normal_coords::sample_surface`, where a run
    /// ends at the direction past which the observer's own straight lines stop reaching the
    /// surface at all, and that edge is a thing to see rather than a gap to bridge.
    runs: Vec<Vec<DrawnRun>>,
}

/// The window the sampler is asked for, in M of xi: the half-diagonal of the canvas with a margin,
/// rounded up onto a ladder of eighth-octaves.
///
/// The rounding is what makes the cache worth having while the automatic framing is settling. That
/// framing log-lerps the zoom towards its target every frame (`FRAME_ZOOM_LERP`), so the raw
/// half-diagonal changes by a fraction of a per cent per frame for as long as the zoom is moving,
/// and a key that carried the raw number would miss every time. Rounding *up* onto a coarse ladder
/// means the sampled window always covers the canvas, and the key only changes when the zoom has
/// moved by nine per cent.
fn sampling_window(rect: Rect, px_per_m: f64) -> f64 {
    let half_diagonal = 0.5 * (rect.width().hypot(rect.height()) as f64) / px_per_m.max(1e-300);
    // A tenth of margin so that a curve leaving by a corner still has its exit point sampled.
    let wanted = half_diagonal * 1.1;
    if !wanted.is_finite() || wanted <= 0.0 {
        return FRAME_MAX_R_DEFAULT;
    }
    2f64.powf((wanted.log2() * 8.0).ceil() / 8.0)
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
    /// Render the 1D+1 spacetime canvas: the (t, r) foliation chart, or an observer's rest frame.
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
        let main_canvas_height = total_size.y.max(150.0);

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
        // nothing to read and the box is the name and the radius alone. The static limit's box is
        // that in every case: nothing about it changes kind with where an observer stands.
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
            let rm_radius = if use_physical_units {
                format!("r₋ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rm)), rm)
            } else {
                format!("r₋ = {:.2}M ({})", rm, metric.format_physical_distance(rm))
            };
            pending_boxes.push(PendingBox {
                id: BoxId::CauchyHorizon,
                anchor: Pos2::new(x_rm_actual + 4.0, rect.top() + 42.0),
                lines: match horizon_reader {
                    Some(obs) => horizon_box_lines(
                        metric,
                        obs,
                        &obs.name,
                        "Cauchy Horizon",
                        Some(&rm_radius),
                        rm,
                        HORIZON_BOX_RED,
                    ),
                    None => surface_radius_lines("Cauchy Horizon", &rm_radius, HORIZON_BOX_RED),
                },
                color: HORIZON_BOX_RED,
                tip: HORIZON_BOX_TIP,
            });
        }

        // Outer Event Horizon r+
        let x_rp_actual = to_screen_x(rp);
        if x_rp_actual >= rect.left() && x_rp_actual <= rect.right() {
            painter.line_segment([Pos2::new(x_rp_actual, rect.top()), Pos2::new(x_rp_actual, rect.bottom())], Stroke::new(2.5, Theme::HORIZON_OUTER));
            let rp_radius = if use_physical_units {
                format!("r₊ = {} ({:.2}M)", metric.format_km(metric.r_to_km(rp)), rp)
            } else {
                format!("r₊ = {:.2}M ({})", rp, metric.format_physical_distance(rp))
            };
            pending_boxes.push(PendingBox {
                id: BoxId::OuterHorizon,
                anchor: Pos2::new(x_rp_actual + 4.0, rect.top() + 42.0),
                lines: match horizon_reader {
                    Some(obs) => horizon_box_lines(
                        metric,
                        obs,
                        &obs.name,
                        "Event Horizon",
                        Some(&rp_radius),
                        rp,
                        Theme::HORIZON_OUTER,
                    ),
                    None => surface_radius_lines("Event Horizon", &rp_radius, Theme::HORIZON_OUTER),
                },
                color: Theme::HORIZON_OUTER,
                tip: HORIZON_BOX_TIP,
            });
        }

        // Ergosphere boundary line
        let x_re_actual = to_screen_x(re);
        if x_re_actual >= rect.left() && x_re_actual <= rect.right() {
            painter.line_segment([Pos2::new(x_re_actual, rect.top()), Pos2::new(x_re_actual, rect.bottom())], Stroke::new(1.5, Theme::ERGOSPHERE_LINE));
            let re_radius = if use_physical_units {
                format!("r_E = {} ({:.2}M)", metric.format_km(metric.r_to_km(re)), re)
            } else {
                format!("r_E = {:.2}M ({})", re, metric.format_physical_distance(re))
            };
            pending_boxes.push(PendingBox {
                id: BoxId::Ergosphere,
                anchor: Pos2::new(x_re_actual + 4.0, rect.top() + 42.0),
                lines: surface_radius_lines("Ergosphere", &re_radius, Theme::ERGOSPHERE_LINE),
                color: Theme::ERGOSPHERE_LINE,
                tip: ERGOSPHERE_CHART_TIP,
            });
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
        // the two boxes read off him. With no Bob in the simulation there is none of it, and the
        // surfaces' boxes are painted here instead.
        let Some(bob) = bob else {
            if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
                pending_boxes.push(PendingBox::observer(
                    painter, rect, Who::Alice, alice_pos, "Alice", Theme::ALICE_COLOR, al, metric,
                    use_physical_units, font_scale, Vec::new(),
                ));
            }
            self.telemetry.flush(ui, painter, Canvas::Spacetime, rect, &pending_boxes, font_scale);
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
        // the opaque fill of a box actually blocks out what is behind it. The surfaces first, then
        // Alice, then Bob, which is also the order in which a box gives way: a later one is nudged
        // clear of the earlier ones.
        if let (Some(al), Some(alice_pos)) = (alice, alice_box) {
            pending_boxes.push(PendingBox::observer(
                painter, rect, Who::Alice, alice_pos, "Alice", Theme::ALICE_COLOR, al, metric,
                use_physical_units, font_scale, Vec::new(),
            ));
        }
        pending_boxes.push(PendingBox::observer(
            painter, rect, Who::Bob, apex, "Bob", Theme::BOB_COLOR, bob, metric, use_physical_units,
            font_scale, Vec::new(),
        ));
        self.telemetry.flush(ui, painter, Canvas::Spacetime, rect, &pending_boxes, font_scale);
    }

    /// The rest-frame window that puts the next surface the observer meets `FRAME_SURFACE_FRACTION`
    /// of the way up the canvas, or `None` when there is nothing ahead of them to frame.
    ///
    /// A surface r = r_h crosses the observer's own time axis at the affine length of the geodesic
    /// that leaves the observer's event along their own 4-velocity and arrives on that surface -
    /// which for a timelike tangent is the proper time the observer has left before reaching it,
    /// and is the same number the drawn curve crosses the axis at. The smallest of those is the
    /// surface they are about to meet: the ring for a raindrop, r+ for anyone still outside it, r-
    /// for a worldline with E - Omega_- L < 0 freezing onto the far branch.
    ///
    /// `affine_length_to_surface` answers `None` for a surface the observer's own worldline does
    /// not set out towards, which is the whole of the test for "ahead": a static observer reaches
    /// no surface r = const by waiting and is framed at the default, and a surface behind an
    /// infaller is refused on the sign of their u^r before the quadrature is ever run.
    ///
    /// This used to be the first-order (r_h - r) / u^r, which is the same quantity linearised and
    /// is wrong by most of a gravitational radius exactly where the framing matters: from the
    /// prograde ISCO of an a = 0.90 hole the linear map puts r+ 1.647 M down the time axis where
    /// the proper time to the crossing is 3.020 M. The framing and the drawn curve then disagreed
    /// about where the surface was, on the one canvas whose job is to say where the surface is.
    ///
    /// Nothing in the rule knows about horizons or about how far down the fall is: the same
    /// expression asks for a window of M early on and of femtometres of r on the approach to r-,
    /// where the observer's remaining proper time is femtoseconds and the gap is nine decades
    /// below anything the view has ever had to draw.
    pub(crate) fn framed_window(metric: &KerrSchild, obs: &Observer, rect: Rect) -> Option<f64> {
        if rect.height() <= 1.0 || rect.width() <= 1.0 {
            return None;
        }
        let u = obs.four_velocity(metric);
        let ahead = [metric.outer_horizon(), metric.inner_horizon(), 0.0]
            .into_iter()
            .filter_map(|r_h| affine_length_to_surface(metric, obs.r, &u, r_h))
            .filter(|tau| tau.is_finite() && *tau > 0.0)
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

    /// Render the focus observer's rest frame: the local chart of their orthonormal tetrad, with
    /// the surfaces and the other observer placed *exactly* and the distant clock's grid placed to
    /// first order.
    ///
    /// Two constructions share the canvas and the view keeps them apart rather than blurring them.
    ///
    /// The exact half is Riemann normal coordinates about the focus observer's event
    /// (`kerr_equatorial::normal_coords`): a point sits at xi^a = sigma T^a, where T is the initial
    /// tangent of the geodesic that leaves the event towards it and sigma is the affine parameter
    /// at which that geodesic arrives. The four surfaces r = const are drawn by sweeping T round
    /// the circle and taking the affine length to each surface, so a surface is a curve and not a
    /// line, and the curve crosses the now-axis at the ruler distance the horizon box prints. The
    /// other observer is placed by the same rule applied to the light that actually arrives: see
    /// `physics::as_seen`.
    ///
    /// The first-order half is the linear map of `LocalFrame`: the distant clock's slices
    /// t = const and the signal crests, each of them exact where it crosses the focus observer's
    /// own worldline - which is where each one is read - and linearised away from it.
    ///
    /// What both halves share is the tetrad at the observer's own event, which is exact. Light
    /// cones therefore stand at exactly 45 degrees, and the causal character of a surface at the
    /// observer's own event comes out of the geometry rather than out of a drawing rule: a surface
    /// r = const is steeper than 45 degrees where g^rr > 0, at 45 degrees on a horizon, and
    /// flatter than 45 degrees between them. `REST_FRAME_TIP` says all of this to the user.
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

        // 1b. Which plane this canvas draws.
        //
        // A rest-frame view has three spatial directions to show and room for one, so it draws the
        // plane spanned by the focus observer's time axis e0 and one spacelike leg s. The leg used
        // to be the radial one, e1, and that choice put the other observer in the wrong place: the
        // light arrives from a direction n = (n^1, n^2) that is not radial - from the prograde
        // ISCO of a fast hole the orbital speed is about half of c, aberration swings the image
        // well round, and |n^2| is large - so projecting the emission event onto the radial plane
        // shortened its distance from the origin by however much of the offset lay along e2. The
        // dot then sat inside the 45-degree past cone rather than on it, and could be drawn beyond
        // the r+ curve while the emission event was plainly in Region I.
        //
        // So the leg is the line of sight itself: s = +/- (n^1 e1 + n^2 e2), the unit spacelike
        // direction the light comes from, with the sign fixed by `AsSeen::plane_leg` so that
        // right is outward in every view. Three things follow, and none of them is a choice.
        //
        // * The emission event is at xi = lambda (-1, n^1, n^2), so its coordinates in this plane
        //   are (xi^1, xi^0) = (lambda n . s, -lambda) = (+/- lambda, -lambda): on the past cone,
        //   on the source's side, exactly - the right-hand edge for a source outward of the
        //   observer and the left-hand edge for one inward.
        // * The surfaces are swept in the same plane, T = sin(psi) e0 + cos(psi) s, so the curve's
        //   own point in the direction of the source lies on the *same ray from the origin* as the
        //   dot. Both are affine lengths along the same geodesic direction, one to the surface and
        //   one to the emission event, so the dot is drawn past the curve exactly when the light
        //   really did come from beyond that surface along that ray, and never otherwise.
        // * With no other observer, or with no image of one, s falls back to e1 and the view is
        //   the radial plane it has always been.
        //
        // The plane turns smoothly while an image lasts, because n does; it switches without any
        // animation when an image appears or goes out, because there is nothing in between to
        // animate. The head banner says which plane is on the screen either way.
        let focus_who = Who::of(focus_obs).unwrap_or(Who::Bob);
        let obs_color =
            if focus_who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        let u_focus = focus_obs.four_velocity(metric);
        let axial = Tetrad::from_four_velocity_axial(metric, focus_obs.r, &u_focus);
        let mut other_seen: Option<(Who, &Observer, Result<AsSeen, NoImage>)> = None;
        let mut seen_held_for: Option<f64> = None;
        if let Some(other) = other_obs.filter(|o| o.is_active) {
            let other_who = Who::of(other).unwrap_or(Who::Bob);
            let key = (focus_who, other_who);
            let seed = self.seen_seeds.get(&key).copied();
            // The image the view draws is the *youngest* one (see `as_seen::younger_image`).
            // The warm solve keeps one image continuous frame to frame, and the sweep for a
            // younger one born beside it costs some milliseconds, so it runs once per interval
            // of the focus observer's own time - and after any cold solve, since a cold march
            // lands on whichever image it lands on.
            // Held from the last solve while the focus observer's own clock has not moved: see
            // `HELD_PICTURE_TAU`. `held` is how far it has moved, for the box.
            let held = self
                .seen_held
                .get(&key)
                .map(|(_, tau)| focus_obs.tau - tau)
                .filter(|moved| moved.abs() < HELD_PICTURE_TAU);
            let answer = match held {
                Some(_) => Ok(self.seen_held[&key].0),
                None => {
                    let due = self.seen_checked.get(&key).is_none_or(|last| {
                        (focus_obs.t - last).abs() >= YOUNGEST_CHECK_INTERVAL
                    });
                    let mut answer = if due {
                        as_seen_youngest(metric, focus_obs, other, seed)
                    } else {
                        as_seen(metric, focus_obs, other, seed)
                    };
                    if due {
                        self.seen_checked.insert(key, focus_obs.t);
                    } else if let Ok(seen) = &answer
                        && seen.cold
                    {
                        self.seen_checked.insert(key, focus_obs.t);
                        if let Some(younger) = younger_image(metric, focus_obs, other, seen) {
                            answer = Ok(younger);
                        }
                    }
                    if let Ok(seen) = &answer {
                        self.seen_seeds.insert(key, seen.seed());
                        self.seen_held.insert(key, (*seen, focus_obs.tau));
                    }
                    answer
                }
            };
            other_seen = Some((other_who, other, answer));
            seen_held_for = held;
        }
        let sight = other_seen
            .as_ref()
            .and_then(|(_, _, answer)| answer.as_ref().ok())
            .map(|seen| seen.plane_leg());
        let s_leg: [f64; 3] = match sight {
            Some(leg) => {
                core::array::from_fn(|mu| leg[0] * axial.e1[mu] + leg[1] * axial.e2[mu])
            }
            None => axial.e1,
        };
        let frame = LocalFrame::for_observer_plane(metric, focus_obs.r, &u_focus, &s_leg);

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
        let font = egui::FontId::proportional(16.0 * font_scale);
        // The first row names the horizontal axis. Right is always outward, or the nearest
        // direction to outward in the drawn plane, but the plane itself is the plane of the line
        // of sight (section 1b), so the row says where that line of sight points: a reader shown
        // "43° retrograde of inward" knows the other observer lies inward and round behind, and
        // that the horizontal axis is the outward-leaning half of that direction.
        let mut head_lines = vec![match &other_seen {
            Some((other_who, other, Ok(seen))) => {
                let other_color =
                    if *other_who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
                vec![
                    painter.layout_no_wrap(
                        "→ outward · line of sight to ".to_string(),
                        font.clone(),
                        Theme::TEXT_MUTED,
                    ),
                    painter.layout_no_wrap(other.name.clone(), font.clone(), other_color),
                    painter.layout_no_wrap(
                        sight_angle_label(seen.sight_angle()),
                        font.clone(),
                        Theme::TEXT_BRIGHT,
                    ),
                ]
            }
            _ => vec![painter.layout_no_wrap(
                "→ outward, along the radial axis".to_string(),
                font.clone(),
                Theme::TEXT_MUTED,
            )],
        }];
        if show_distant_clock_grid && clock_proper_step.is_finite() && clock_proper_step > 0.0 {
            let distant = distant_clock_offset_label(clock_grid.step_m * seconds_per_m);
            let ratio = if !u_t.is_finite() {
                "∞".to_string()
            } else if u_t < 1e4 {
                format!("{u_t:.1}")
            } else {
                format!("{u_t:.2e}")
            };
            let font2 = font.clone();
            head_lines.push(vec![
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
            ]);
            head_lines.push(vec![
                painter.layout_no_wrap("1".to_string(), font2.clone(), obs_color),
                painter.layout_no_wrap(" : ".to_string(), font2.clone(), Theme::TEXT_MUTED),
                painter.layout_no_wrap(ratio, font2.clone(), Theme::TEXT_BRIGHT),
                painter.layout_no_wrap("   (dt/dτ)".to_string(), font2, Theme::TEXT_MUTED),
            ]);
        }
        let head_height: f32 = head_lines
            .iter()
            .map(|line| line.iter().map(|g| g.rect.height()).fold(0.0, f32::max))
            .sum();
        let head_bottom = rect.top() + 4.0 + head_height;
        // How far into the canvas the distant clock's labels reach down the left edge, kept as
        // they are drawn so that the distance scale at the foot can start clear of the whole
        // column of them rather than of a guess at one. With the grid off there is no column.
        let mut labels_right = rect.left();
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
                let drawn = painter.text(
                    Pos2::new(x, y - 2.0),
                    egui::Align2::LEFT_BOTTOM,
                    distant_clock_offset_label(dt * seconds_per_m),
                    egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                    label_colour,
                );
                labels_right = labels_right.max(drawn.right() + 3.0);
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

        // 3. Surfaces r = const, every one of them the exact curve it is in this chart.
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

        // Worked out here, where the geometry is; painted at the end of the pass.
        let mut pending_boxes: Vec<PendingBox> = Vec::new();

        // The exact curves, taken once per change of the observer's event and of the zoom rather
        // than once per frame. The sampler's window is the half-diagonal of the canvas, because a
        // polar curve about the centre of the canvas can leave by a corner, and the margin and the
        // quantisation are `sampling_window`'s.
        let window = sampling_window(rect, scale as f64);
        let key = SurfaceKey {
            m: metric.m,
            a: metric.a,
            r0: focus_obs.r,
            u: u_focus,
            s: s_leg,
            window,
        };
        if self.surfaces.key != Some(key) {
            let opts = SurfaceSampling::for_window(window);
            self.surfaces.runs = surfaces
                .iter()
                .map(|&(r_h, ..)| {
                    sample_surface_in_plane(metric, focus_obs.r, &u_focus, &s_leg, r_h, &opts)
                        .into_iter()
                        .map(|run| DrawnRun {
                            // A run never mixes branches, so the first point that names one names
                            // the run's. The bifurcation point is on two runs and names neither.
                            branch: run.iter().find_map(|p| p.branch),
                            points: run.into_iter().map(|p| p.xi).collect(),
                        })
                        .collect()
                })
                .collect();
            self.surfaces.key = Some(key);
        }

        // Everything drawn from a curve is clipped to the canvas rather than trimmed by hand. The
        // clipping matters here and not only for tidiness: a run of the sampler ends with one exact
        // point beyond the window, and near the direction where sigma runs away that point can be
        // decades outside it, so the polyline has to be cut rather than drawn.
        let clipped = painter.with_clip_rect(rect);

        for (idx, &(r_h, id, title, color, width, border, is_horizon)) in surfaces.iter().enumerate() {
            // The curve in screen points, run by run, clipped to the canvas. Carried in f64 all the
            // way to the clip because the far end of a run can be many decades outside the window
            // and an f32 would have overflowed on the way.
            // A horizon is two surfaces and the sweep meets both, so the drawing says which is
            // which. The branch drawn in the full stroke is the one the observer's own future
            // turns on - the future horizon for r+, the far branch for r-, the Cauchy horizon
            // proper that a worldline with E - Omega_- L < 0 freezes on - and the other branch is
            // drawn in a light stroke of the same colour: the past horizon of r+, which nothing
            // ever crosses, and the branch of r- an infaller has already come through. Same
            // colour because the two are the same surface r = const and a reader hunting for r-
            // must find all of it; different weight because they are different halves of it. The
            // curves meet at the bifurcation point, which both runs carry.
            let heavy = branch_in_full_stroke(metric, r_h, is_horizon);
            let mut visible: Vec<Vec<Pos2>> = Vec::new();
            let mut drawn_branches: Vec<Option<HorizonBranch>> = Vec::new();
            for run in self.surfaces.runs.get(idx).into_iter().flatten() {
                let screen: Vec<[f64; 2]> = run
                    .points
                    .iter()
                    .map(|p| {
                        [
                            center.x as f64 + p[0] * scale as f64,
                            center.y as f64 - p[1] * scale as f64,
                        ]
                    })
                    .collect();
                let pieces = clip_polyline_to_rect(&screen, rect);
                if pieces.is_empty() {
                    continue;
                }
                let stroke = if heavy.is_none() || run.branch == heavy {
                    Stroke::new(width, color)
                } else {
                    Stroke::new((width * 0.4).max(0.8), color)
                };
                for piece in &pieces {
                    clipped.add(egui::Shape::line(piece.clone(), stroke));
                }
                if !drawn_branches.contains(&run.branch) {
                    drawn_branches.push(run.branch);
                }
                visible.extend(pieces);
            }

            // The line the *box* reads is still `surface_r_const`, and deliberately so. That line
            // is the exact tangent of this surface at the observer's own event: its slope is the
            // causal character and the closing speed there, which is what the box quotes. Reading
            // either off the far curve would be reading a statement about a distant event.
            let line = frame.surface_r_const(r_h);

            // The causal character is read straight off that slope: |d xi^0 / d xi^1| > 1 is
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
            //
            // A surface with no visible run gets no box. The box stands beside its own curve, and
            // one with nothing to stand beside would be a reading about something the user cannot
            // see and cannot scroll to.
            let Some(label_pos) = curve_anchor(&visible, center.y, rect.center()) else {
                continue;
            };
            let mut box_lines = if is_horizon {
                horizon_box_lines(metric, focus_obs, &focus_obs.name, title, None, r_h, border)
            } else {
                vec![
                    TelemetryLine { text: title.to_string(), color: border, is_title: true, bold: false },
                    TelemetryLine { text: note.to_string(), color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
                    TelemetryLine { text: detail, color: Theme::TEXT_BRIGHT, is_title: false, bold: false },
                ]
            };
            // Which halves of this horizon the picture holds, so that a reader meeting a curve
            // with a corner in it knows the corner is the bifurcation of two branches and knows
            // which stroke is which.
            if let Some(text) = branches_drawn_line(metric, r_h, heavy, &drawn_branches) {
                box_lines.push(TelemetryLine {
                    text,
                    color: Theme::TEXT_MUTED,
                    is_title: false,
                    bold: false,
                });
            }
            let size = telemetry_box_size(painter, &box_lines, font_scale);
            // Clear of the head banner, which is painted last and would otherwise have a surface's
            // box sitting on its text: a curve that leaves by the top of the canvas anchors its box
            // at the top of the canvas, which is exactly where the banner is.
            let anchor = default_badge_pos(rect, label_pos, size);
            let anchor = Pos2::new(
                anchor.x,
                anchor.y.max(head_bottom + 6.0).min(rect.bottom() - size.y - 6.0),
            );
            pending_boxes.push(PendingBox {
                id,
                anchor,
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
        let mut focus_extra: Vec<TelemetryLine> = Vec::new();

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
                "+45° Outward",
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                focus_edge,
            );
            painter.text(
                p_fut_in + Vec2::new(-4.0, -2.0),
                egui::Align2::RIGHT_BOTTOM,
                "-45° Inward",
                egui::FontId::monospace(Theme::MIN_FONT_PT * font_scale),
                focus_edge,
            );
        } else if focus_obs.is_active {
            painter.circle_filled(center, 12.0, Theme::SINGULARITY_FILL);
            painter.circle_stroke(center, 15.0, Stroke::new(2.0, Theme::SINGULARITY_LINE));
            // Said in the observer's own box, as a condition of theirs, not beside the marker.
            focus_extra.push(TelemetryLine {
                text: "Singularity impact: light cone terminated at r = 0".to_string(),
                color: Theme::WARNING_RED,
                is_title: false,
                bold: true,
            });
        }

        // No outline. A white ring round the dot said nothing the dot did not already say, and the
        // dot sits on the origin of the axes here, where an extra stroke is only clutter.
        painter.circle_filled(apex, 7.5, obs_color);

        // 4. The other observer, as seen. The solve happened in section 1b, because its answer is
        // what chose the plane this canvas draws; what is left here is the drawing.
        //
        // Nobody sees anybody now. What arrives at the focus observer's event is light, so the
        // event to draw is the one on the other worldline that the focus observer's past light cone
        // passes through, and the place to draw it is where the exact normal-coordinate rule puts
        // it: at the affine length of the arriving ray, along the direction the ray arrives from.
        // In this plane that direction *is* the horizontal axis, so the dot lands at
        // (xi^1, xi^0) = (+/- lambda, -lambda), on the 45-degree past cone and on the source's
        // side: right for a source outward of the observer, left for one inward.
        //
        // Only a dot. The other observer's light cone, the stroke along their worldline and the
        // singularity glyph have all gone, and each for the same reason: every one of them was a
        // statement about the other observer *now*, drawn at a position that is a statement about
        // the other observer *then*. A cone at the seen event would be the cone of an event whose
        // future the focus observer has not received.
        let mut other_box: Option<PendingBox> = None;
        if let Some((other_who, other, answer)) = other_seen {
            let other_color =
                if other_who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
            match answer {
                Ok(seen) => {
                    let xi = seen.xi_plane();
                    let other_pos = to_screen(xi[0], xi[1]);

                    // Each observer carries a beacon at the dominant wavelength of their own marker
                    // colour, and the dot is painted the colour that beacon arrives in. See
                    // `gui::beacon_colour` for the whole chain.
                    let rest_nm = beacon_nm(other_who);
                    let beacon = beacon_colour::seen_beacon(rest_nm, seen.g);

                    // The straight line from the dot to the focus observer's own event *is* the
                    // light. In Riemann normal coordinates a null geodesic through the origin is a
                    // straight 45-degree line, and the emission event was placed on that geodesic
                    // by its own affine length, so the segment between the two is the ray drawn to
                    // scale rather than a pointer added to explain the picture.
                    let ray_colour = match beacon {
                        Beacon::Seen(colour) => colour.gamma_multiply(0.45),
                        Beacon::Ring => Color32::from_white_alpha(60),
                    };
                    for piece in clip_polyline_to_rect(
                        &[
                            [center.x as f64, center.y as f64],
                            [other_pos.x as f64, other_pos.y as f64],
                        ],
                        rect,
                    ) {
                        clipped.add(egui::Shape::line(piece, Stroke::new(1.2, ray_colour)));
                    }

                    match beacon {
                        // No white outline round the dot: the colour is the whole of what the dot
                        // has to say, and a white ring would take a bite out of it.
                        Beacon::Seen(colour) => {
                            clipped.circle_filled(other_pos, 6.0, colour);
                        }
                        Beacon::Ring => {
                            for dash in egui::Shape::dashed_line(
                                &ring_polyline(other_pos, 6.5),
                                Stroke::new(1.2, Theme::TEXT_MUTED),
                                3.0,
                                3.0,
                            ) {
                                clipped.add(dash);
                            }
                        }
                    }

                    // The box reads the emission event, so it is built on a snapshot of the other
                    // observer standing there rather than on the observer standing here.
                    let snapshot = seen_snapshot(other, &seen);
                    let mut extra = as_seen_lines(metric, &seen, rest_nm, other);
                    if seen_held_for.is_some() {
                        extra.push(TelemetryLine {
                            text: format!(
                                "picture held: your clock has moved under {HELD_PICTURE_TAU} M \
                                 since this image was solved, so it cannot have changed"
                            ),
                            color: Theme::TEXT_MUTED,
                            is_title: false,
                            bold: false,
                        });
                    }
                    other_box = Some(
                        PendingBox::observer(
                            painter,
                            rect,
                            other_who,
                            other_pos,
                            &format!("{} as seen", other.name),
                            other_color,
                            &snapshot,
                            metric,
                            use_physical_units,
                            font_scale,
                            extra,
                        )
                        .with_tip(AS_SEEN_BOX_TIP),
                    );
                }
                Err(why) => {
                    // The seed is *kept* on a failure rather than dropped. A failed solve costs one
                    // Newton before `as_seen` falls back to the cold fan anyway, and the last seed
                    // that worked is much the best guess there is for the next frame: the cold fan
                    // is the path that gives up on a deeply redshifted image, so throwing the seed
                    // away would turn one bad frame into a permanent one.
                    //
                    // Still a box, because "there is no image, and here is why" is a reading and
                    // the user has nowhere else to read it. It stands where the focus observer's
                    // own box would stand, since there is no dot to stand beside.
                    let lines = vec![
                        TelemetryLine {
                            text: format!("{}: no image", other.name),
                            color: other_color,
                            is_title: true,
                            bold: false,
                        },
                        TelemetryLine {
                            text: no_image_reason(why, &focus_obs.name, &other.name),
                            color: Theme::TEXT_BRIGHT,
                            is_title: false,
                            bold: false,
                        },
                    ];
                    let size = telemetry_box_size(painter, &lines, font_scale);
                    other_box = Some(PendingBox {
                        id: BoxId::Observer(other_who),
                        anchor: default_badge_pos(rect, apex, size),
                        lines,
                        color: other_color,
                        tip: AS_SEEN_BOX_TIP,
                    });
                }
            }
        }

        // The banner is centred as a block on its widest row, which keeps it clear of the grid's
        // own line labels down the left edge. Every row shares that block's left edge rather than
        // being centred on its own width: centred, a short row would sit on the observer's clock
        // ticks, which run up the middle of the canvas beside the time axis, and pushed out to the
        // margin it would sit on the distant grid's labels, which run down the left edge.
        let widest = head_lines
            .iter()
            .map(|line| line.iter().map(|g| g.rect.width()).sum::<f32>())
            .fold(0.0, f32::max);
        let mut y = rect.top() + 4.0;
        let left = (rect.center().x - widest * 0.5).max(rect.left() + 4.0);
        for line in head_lines.into_iter() {
            let height = line.iter().map(|g| g.rect.height()).fold(0.0, f32::max);
            let mut x = left;
            for galley in line {
                let width = galley.rect.width();
                painter.galley(Pos2::new(x, y), galley, Color32::WHITE);
                x += width;
            }
            y += height;
        }

        // The distance scale, in the same style and the same code as the equatorial view's and the
        // foliation's. What it measures here is not a radius: the horizontal axis is the local
        // normal coordinate xi^1 along the drawn spacelike leg, whose affine length is the proper
        // distance the focus observer's own ruler measures along that geodesic, so the caption
        // says so. The scale is read off the view every frame, which is what the automatic framing
        // needs: the window shrinks by decades on a deep approach and the round step follows it.
        //
        // What it is given is the bottom left corner the chart's own labels leave it: the distant
        // clock's numbers run down the left edge and the observer's own clock is ticked up the
        // middle, so the rect stops at the left of one and at the observer's own worldline on the
        // right, and the bar is laid out inside that.
        ruler::draw_distance_ruler(
            painter,
            Rect::from_min_max(
                Pos2::new(labels_right, rect.top()),
                Pos2::new(center.x, rect.bottom()),
            ),
            f64::from(scale),
            use_physical_units,
            metric,
            font_scale,
            Some("proper distance"),
        );

        // Every info box on this canvas goes here, after the head banner and everything else, so
        // that the opaque fill of a box blocks out what is behind it and dragging one wins over
        // the canvas's own drag response. The surfaces first, the observers over them, which is
        // also the order in which a box gives way: a later one is nudged clear of the earlier ones.
        if let Some(other_box) = other_box {
            pending_boxes.push(other_box);
        }
        pending_boxes.push(PendingBox::observer(
            painter, rect, focus_who, apex, &focus_obs.name, obs_color, focus_obs, metric,
            use_physical_units, font_scale, focus_extra,
        ));
        self.telemetry.flush(ui, painter, Canvas::RestFrame, rect, &pending_boxes, font_scale);
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

/// The visible pieces of a screen-space polyline, clipped segment by segment to `rect`.
///
/// Carried in f64 rather than in `Pos2`. The caller is handing over an exact curve whose sampler
/// keeps one point beyond the window at each end of a run, and near a direction where the affine
/// length runs away that point can be many decades outside the canvas: converted to f32 first it
/// would be an infinity, and every clip against it would come back as nothing.
///
/// Liang-Barsky on each segment, with the pieces stitched back together while consecutive segments
/// leave the rectangle nowhere, so a curve that crosses the canvas once comes back as one polyline
/// and not as a hundred two-point ones.
fn clip_polyline_to_rect(points: &[[f64; 2]], rect: Rect) -> Vec<Vec<Pos2>> {
    let (left, right) = (rect.left() as f64, rect.right() as f64);
    let (top, bottom) = (rect.top() as f64, rect.bottom() as f64);
    let mut pieces: Vec<Vec<Pos2>> = Vec::new();
    let mut current: Vec<Pos2> = Vec::new();
    let finite = |p: &[f64; 2]| p[0].is_finite() && p[1].is_finite();
    /// End the piece being built, keeping it only if it is a curve rather than a single point.
    macro_rules! flush {
        () => {
            if current.len() >= 2 {
                pieces.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        };
    }

    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if !finite(&a) || !finite(&b) {
            flush!();
            continue;
        }
        let d = [b[0] - a[0], b[1] - a[1]];
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let mut inside = true;
        for &(num, den) in &[
            (-d[0], a[0] - left),
            (d[0], right - a[0]),
            (-d[1], a[1] - top),
            (d[1], bottom - a[1]),
        ] {
            if num == 0.0 {
                if den < 0.0 {
                    inside = false;
                    break;
                }
            } else {
                let t = den / num;
                if num < 0.0 {
                    t0 = t0.max(t);
                } else {
                    t1 = t1.min(t);
                }
            }
        }
        if !inside || t0 > t1 {
            flush!();
            continue;
        }
        let at = |t: f64| Pos2::new((a[0] + t * d[0]) as f32, (a[1] + t * d[1]) as f32);
        // A piece carries on only when this segment starts exactly where the last one was cut off,
        // which is t0 = 0: any clipped entry is a fresh arrival into the rectangle.
        if t0 > 0.0 || current.is_empty() {
            flush!();
            current.push(at(t0));
        }
        current.push(at(t1));
        if t1 < 1.0 {
            flush!();
        }
    }
    flush!();
    pieces
}

/// Which branch of a horizon the view draws in the full stroke, and `None` for a surface that is
/// not a horizon and has only the one branch to draw.
///
/// r+ is drawn heavy on the branch a worldline crosses, the future horizon, because that is the
/// surface an observer's own future turns on; its other branch is the past horizon, which nothing
/// ever crosses and which the picture only ever shows because an outside observer's past light
/// cone runs down onto it. r- is drawn heavy on the branch worldlines freeze on - the far branch,
/// the Cauchy horizon proper, the one this app is about - and light on the branch an infaller has
/// already come through.
fn branch_in_full_stroke(
    metric: &KerrSchild,
    r_h: f64,
    is_horizon: bool,
) -> Option<HorizonBranch> {
    if !is_horizon {
        return None;
    }
    Some(if r_h >= metric.outer_horizon() {
        HorizonBranch::Crossing
    } else {
        HorizonBranch::Asymptotic
    })
}

/// What this horizon is called on each of its branches, in the words the box prints.
fn branch_name(outer: bool, branch: HorizonBranch) -> &'static str {
    match (outer, branch) {
        (true, HorizonBranch::Crossing) => "the future horizon",
        (true, HorizonBranch::Asymptotic) => "the past horizon",
        (false, HorizonBranch::Crossing) => "the branch an infaller crosses",
        (false, HorizonBranch::Asymptotic) => "the far branch",
    }
}

/// The box's line naming which branches of a horizon the canvas is showing, and in which stroke.
///
/// `None` for a surface with no branches to tell apart, and for a horizon of which only the
/// heavy branch is on the canvas, where a line about strokes would be a line about nothing.
fn branches_drawn_line(
    metric: &KerrSchild,
    r_h: f64,
    heavy: Option<HorizonBranch>,
    drawn: &[Option<HorizonBranch>],
) -> Option<String> {
    let heavy = heavy?;
    let outer = r_h >= metric.outer_horizon();
    let light = if heavy == HorizonBranch::Crossing {
        HorizonBranch::Asymptotic
    } else {
        HorizonBranch::Crossing
    };
    match (
        drawn.contains(&Some(heavy)),
        drawn.contains(&Some(light)),
    ) {
        (true, true) => Some(format!(
            "heavy: {} · light: {}",
            branch_name(outer, heavy),
            branch_name(outer, light)
        )),
        (false, true) => Some(format!("light: {}", branch_name(outer, light))),
        _ => None,
    }
}

/// Where a surface's info box stands: the point of the drawn curve the box is a reading about.
///
/// The now-axis crossing first. xi^0 = 0 is the observer's own rest space, so where the curve cuts
/// it is where the surface is *at this moment* for this observer - and it is the point the box's
/// own ruler distance names, so the box and the number in it stand at the same place. Failing that,
/// the visible point nearest the middle of the canvas, which is the one a reader is most likely to
/// be looking at.
///
/// `None` when the curve has no visible point at all.
fn curve_anchor(pieces: &[Vec<Pos2>], axis_y: f32, centre: Pos2) -> Option<Pos2> {
    let mut best_axis: Option<Pos2> = None;
    let mut best_near: Option<(f32, Pos2)> = None;
    for piece in pieces {
        for pair in piece.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if (a.y - axis_y).signum() != (b.y - axis_y).signum() || a.y == axis_y {
                let x = segment_x_at_y(a, b, axis_y);
                let here = Pos2::new(x, axis_y);
                // The outermost crossing, so that a curve cutting the axis on both sides of the
                // observer puts its box clear of the observer's own marker rather than on it.
                if best_axis.is_none_or(|p| (p.x - centre.x).abs() < (x - centre.x).abs()) {
                    best_axis = Some(here);
                }
            }
        }
        for p in piece {
            let d = p.distance_sq(centre);
            if best_near.is_none_or(|(best, _)| d < best) {
                best_near = Some((d, *p));
            }
        }
    }
    best_axis.or(best_near.map(|(_, p)| p))
}

/// A circle of radius `radius` about `centre` as a closed polyline, which is what
/// `Shape::dashed_line` takes: epaint dashes a path and has no dashed-circle of its own.
fn ring_polyline(centre: Pos2, radius: f32) -> Vec<Pos2> {
    // Thirty-two sides at six or seven points of radius is under half a point of sagitta, which is
    // the same resolution `polyline::SCREEN_SPACING` calls indistinguishable.
    const SIDES: usize = 32;
    (0..=SIDES)
        .map(|i| {
            let a = std::f32::consts::TAU * (i as f32) / (SIDES as f32);
            Pos2::new(centre.x + radius * a.cos(), centre.y + radius * a.sin())
        })
        .collect()
}

/// The rest wavelength of an observer's beacon: the dominant wavelength of that observer's own
/// marker colour. See `gui::beacon_colour`.
fn beacon_nm(who: Who) -> f64 {
    match who {
        Who::Alice => beacon_colour::ALICE_BEACON_NM,
        Who::Bob => beacon_colour::BOB_BEACON_NM,
    }
}

/// The other observer as they stood at the emission event, so that every ordinary telemetry line
/// of their box reads the event the light left rather than the event they are at now.
///
/// Only the fields that describe an event are moved: the position, the clock, and the 4-velocity,
/// which is carried by pushing the emission 4-velocity into the geodesic state so that
/// `Observer::four_velocity` reports it. The mode, the boosts and the release are the worldline's
/// own and do not belong to one event of it.
///
/// Three things are deliberate. The trail is left empty, because nothing a telemetry box reads
/// touches the trail and cloning a ten-thousand-event deque every frame would cost more than the
/// whole solve. `release_t` is pulled back to the emission time where the light left before the
/// release, so that `four_velocity` recognises the geodesic as standing on the snapshot's own event
/// instead of falling back to the closed form at that radius. And `stalled` is set from the watch:
/// a worldline frozen on the far branch of r- holds its proper time at the value it froze with, so
/// an emission event carrying that same reading is one from the frozen stretch and an earlier one
/// is not.
fn seen_snapshot(other: &Observer, seen: &AsSeen) -> Observer {
    let event = seen.emission;
    let frozen_now = other.is_frozen();
    let geodesic = other.geodesic.map(|mut geo| {
        geo.t = event.t;
        geo.r = event.r;
        geo.phi = event.phi;
        geo.tau = event.tau;
        geo.u = event.u;
        geo.stalled =
            frozen_now && (event.tau - other.tau).abs() <= 1e-9 * (1.0 + other.tau.abs());
        geo
    });
    Observer {
        name: other.name.clone(),
        mode: other.mode,
        t: event.t,
        r: event.r,
        phi: event.phi,
        tau: event.tau,
        beta_r: other.beta_r,
        beta_phi: other.beta_phi,
        geodesic,
        trail: VecDeque::new(),
        start: other.start,
        release_t: other.release_t.min(event.t),
        release: other.release,
        is_active: true,
    }
}

/// The lines the *as seen* box carries under the ordinary telemetry: what the light did on the way
/// here, and what the eye makes of what arrived.
fn as_seen_lines(
    metric: &KerrSchild,
    seen: &AsSeen,
    rest_nm: f64,
    other: &Observer,
) -> Vec<TelemetryLine> {
    let seconds_per_m = metric.t_grav_seconds() / metric.m.max(1e-12);
    let body = |text: String| TelemetryLine {
        text,
        color: Theme::TEXT_BRIGHT,
        is_title: false,
        bold: false,
    };
    let mut lines = Vec::new();

    // lambda is at once how long ago the light left, on this observer's own clock, and how far away
    // the emission event is, on this observer's own ruler: in normal coordinates a null geodesic
    // makes those one number, which is exactly why the dot lands on the 45-degree past cone.
    lines.push(body(format!(
        "light left = {} ago ({} away)",
        duration_label(seen.lambda * seconds_per_m),
        ruler_distance_label(metric, seen.lambda)
    )));

    // The emitter's own watch at the emission event. A negative reading is not an error: the watch
    // is zeroed at the observer's creation, and the backward extension of the hold is what makes
    // anybody visible at all at t = 0.
    lines.push(body(if seen.emission.tau < 0.0 {
        format!("{}'s watch = before the run began", other.name)
    } else {
        format!(
            "{}'s watch = {}",
            other.name,
            duration_label(seen.emission.tau * seconds_per_m)
        )
    }));

    // g is taken from the ray itself rather than from a formula, so it stays finite and positive
    // at and inside both horizons. 1 + z is its reciprocal, which is the number a spectroscopist
    // quotes.
    let one_plus_z = if seen.g > 0.0 { 1.0 / seen.g } else { f64::INFINITY };
    lines.push(body(format!("shift = 1+z = {}", loose_number(one_plus_z))));
    lines.push(body(format!("g = {} (ν_seen/ν_emitted)", loose_number(seen.g))));

    let seen_nm = rest_nm / seen.g;
    lines.push(body(format!(
        "beacon {} seen at {} ({})",
        beacon_colour::wavelength_label(rest_nm),
        beacon_colour::wavelength_label(seen_nm),
        beacon_colour::band_name(seen_nm)
    )));
    // The flux factor first, because it is the physics: g^4 is what a point source's bolometric
    // flux is multiplied by. The second number is what the *eye* is left with, the flux factor
    // times the eye's own response at the seen wavelength against the response at the rest one,
    // and it is what decides whether the dot is painted at all.
    lines.push(body(format!(
        "brightness = ×g⁴ = {} (eye ×{})",
        loose_number(seen.g.powi(4)),
        loose_number(beacon_colour::brightness_factor(rest_nm, seen.g))
    )));
    lines.push(body(format!(
        "watch rate as seen = ×g = {}",
        loose_number(seen.g)
    )));

    // Which way the canvas is looking. The view draws the plane of the focus observer's time axis
    // and this line of sight, so the horizontal axis points at the source and the angle says where
    // that is against the one direction a reader already has a name for. Nothing is dropped from
    // the offset any more - the whole of it lies in the drawn plane - so there is no third
    // component left to print.
    lines.push(body(format!(
        "line of sight ={}",
        sight_angle_label(seen.sight_angle())
    )));

    // Where the solver read the emission event from, on the two occasions when that is a statement
    // about the physics rather than about the bookkeeping.
    match seen.source {
        WorldlineSource::Hold | WorldlineSource::FixedRadius
            if seen.emission.t < other.release_t =>
        {
            lines.push(body(format!(
                "light left before the run began: {} was holding the worldline the release \
                 begins on",
                other.name
            )));
        }
        WorldlineSource::Geodesic if seen.emission.t < other.release_t => {
            lines.push(body(format!(
                "light left before the run began: {} was already falling, on the worldline the \
                 run continues",
                other.name
            )));
        }
        WorldlineSource::Dragged => {
            lines.push(body(format!(
                "you are dragging {}, so the run holds that marker at this (r, ϕ) rather than \
                 integrating a worldline through it",
                other.name
            )));
        }
        _ => {}
    }
    lines
}

/// The line of sight's angle from the observer's own outward radial direction, in the words the
/// header and the as-seen box both print: " 137° prograde of outward".
///
/// Prograde is the observer's local +ϕ direction, the way the hole turns, so the two words name
/// the side rather than leaving a signed number to be read as one. Straight out and straight in
/// get their own wording, because "0° prograde of outward" is a sentence about nothing. The
/// leading space belongs to the string so that a caller can concatenate it onto a name.
fn sight_angle_label(radians: f64) -> String {
    let degrees = radians.to_degrees();
    if degrees.abs() < 0.5 {
        return " straight outward".to_string();
    }
    if degrees.abs() > 179.5 {
        return " straight inward".to_string();
    }
    // From whichever radial direction is nearer, so that the label never quotes an angle over
    // 90 degrees and the word says at once which cone edge the dot is on: "of outward" and it is
    // on the right-hand edge, "of inward" and it is on the left.
    let sense = if degrees >= 0.0 { "prograde" } else { "retrograde" };
    if degrees.abs() <= 90.0 {
        format!(" {:.0}° {sense} of outward", degrees.abs())
    } else {
        format!(" {:.0}° {sense} of inward", 180.0 - degrees.abs())
    }
}

/// A ratio printed at three decimals where three decimals say something, and in scientific notation
/// at both ends of the scale. g runs from about 1e-10 on a hovering observer's last sight of an
/// infaller to tens on the approach to r-, and a fixed number of decimals would print a column of
/// zeroes at one end and a wall of digits at the other.
fn loose_number(x: f64) -> String {
    if !x.is_finite() {
        return "∞".to_string();
    }
    if x != 0.0 && !(1e-3..1e4).contains(&x.abs()) {
        format!("{x:.3e}")
    } else {
        format!("{x:.3}")
    }
}

/// Why there is no image of the other observer, in the words the box prints. Every variant is a
/// physical statement or an honest limit of the recorded data, and each one says which.
fn no_image_reason(why: NoImage, focus_name: &str, other_name: &str) -> String {
    match why {
        NoImage::FocusFrozen => format!(
            "{focus_name} has frozen on the far branch of r₋: aberration has closed every \
             direction onto one point, and no picture of anything survives that"
        ),
        NoImage::FocusEnded => {
            format!("{focus_name} has reached the ring, and has no more worldline to receive on")
        }
        NoImage::OtherEnded => format!(
            "the light would have had to leave {other_name} after {other_name}'s worldline ended"
        ),
        NoImage::TrailEvicted => format!(
            "the trail's cap has dropped the stretch of {other_name}'s worldline the light left \
             from; raise the cap before a run rather than after"
        ),
        NoImage::RaysDied => format!(
            "the ring swallowed every trial ray, or every trial ray climbed out of the search, \
             before any of them reached {other_name}"
        ),
        NoImage::NoCrossing => format!(
            "{focus_name}'s past light cone does not reach {other_name} inside the time the search \
             looks back over"
        ),
        NoImage::NotConverged => {
            "the search bracketed the crossing and the solve did not settle on it".to_string()
        }
    }
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

/// Every shape a painter was handed, flattened out of the nested `Shape::Vec`s. Shared by both of
/// this file's test modules, which is why it sits out here rather than in either of them.
#[cfg(test)]
fn flatten(shape: &egui::Shape, out: &mut Vec<egui::Shape>) {
    match shape {
        egui::Shape::Vec(inner) => inner.iter().for_each(|s| flatten(s, out)),
        other => out.push(other.clone()),
    }
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

    fn release(pos: Pos2) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::default(),
        }
    }

    /// A box with a live title and two readings under it, which is the shape of every info box on
    /// every canvas: the title is the first line and the body is the rest.
    fn sample_lines() -> Vec<TelemetryLine> {
        let reading = |text: &str| TelemetryLine {
            text: text.to_string(),
            color: Theme::TEXT_BRIGHT,
            is_title: false,
            bold: false,
        };
        vec![
            TelemetryLine {
                text: "Bob [Region II]".to_string(),
                color: Theme::BOB_COLOR,
                is_title: true,
                bold: false,
            },
            reading("dr/dt -0.412 c"),
            reading("tau 12.5 M"),
        ]
    }

    /// One headless frame of a single info box, registered in the order every canvas registers
    /// one: the background allocates its own drag response first and the box goes on top. The box
    /// is anchored well inside the canvas, so nothing measured here is about clamping. Returns
    /// every shape the frame painted.
    fn telemetry_frame(
        ctx: &egui::Context,
        boxes: &mut TelemetryBoxes,
        lines: &[TelemetryLine],
        events: Vec<egui::Event>,
    ) -> Vec<egui::Shape> {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(420.0, 340.0))),
            events,
            ..Default::default()
        };
        let output = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                let (canvas, painter) =
                    ui.allocate_painter(Vec2::new(360.0, 260.0), egui::Sense::drag());
                let anchored = canvas.rect.min + Vec2::new(60.0, 70.0);
                let _ = boxes.show_lines(
                    ui,
                    &painter,
                    Canvas::Spacetime,
                    BoxId::Observer(Who::Bob),
                    canvas.rect,
                    anchored,
                    lines,
                    Theme::BOB_COLOR,
                    1.0,
                    TELEMETRY_HOVER_TIP,
                );
            });
        });
        let mut shapes = Vec::new();
        for clipped in output.shapes.iter() {
            flatten(&clipped.shape, &mut shapes);
        }
        output.drop_without_applying_deltas();
        shapes
    }

    /// The card a frame painted, read off the one rectangle carrying the box's own fill.
    fn painted_card(shapes: &[egui::Shape]) -> Rect {
        shapes
            .iter()
            .find_map(|s| match s {
                egui::Shape::Rect(r) if r.fill == Color32::from_black_alpha(230) => Some(r.rect),
                _ => None,
            })
            .expect("the frame painted a box")
    }

    /// Every string a frame printed.
    fn painted_text(shapes: &[egui::Shape]) -> Vec<String> {
        shapes
            .iter()
            .filter_map(|s| match s {
                egui::Shape::Text(t) => Some(t.galley.text().to_string()),
                _ => None,
            })
            .collect()
    }

    /// The triangle shuts the box down to its title line and opens it again, and the corner the
    /// user is looking at does not move under either click. A box that has never been dragged is
    /// the case that could move: the anchor a box is placed at depends on the box's size, so the
    /// anchor is measured on the open box in both states and only the drawn size is clamped.
    #[test]
    fn test_a_click_on_the_triangle_shuts_a_box_to_its_title_line_and_another_click_opens_it() {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut boxes = TelemetryBoxes::default();
        let lines = sample_lines();
        let key = (Canvas::Spacetime, BoxId::Observer(Who::Bob));

        // The first frame only registers the widgets, which is what the next frame's hit test
        // reads.
        let shapes = telemetry_frame(&ctx, &mut boxes, &lines, vec![]);
        let open = painted_card(&shapes);
        assert!(
            painted_text(&shapes).iter().any(|t| t == "dr/dt -0.412 c"),
            "the open box prints its readings"
        );

        let at = disclosure_hit_rect(open, 1.0).center();
        telemetry_frame(&ctx, &mut boxes, &lines, vec![egui::Event::PointerMoved(at)]);
        telemetry_frame(&ctx, &mut boxes, &lines, vec![press(at)]);
        let shapes = telemetry_frame(&ctx, &mut boxes, &lines, vec![release(at)]);

        assert!(boxes.collapsed.contains(&key), "the click shut the box");
        let shut = painted_card(&shapes);
        assert_eq!(shut.min, open.min, "and the box's top-left corner stayed where it was");
        let m = box_metrics(1.0);
        assert!(
            (shut.height() - (m.pad_y * 2.0 + m.line_spacing * 0.8)).abs() < 1e-3,
            "a shut box is one title row tall, not {}",
            shut.height()
        );
        assert!(shut.width() < open.width(), "and no wider than the title needs");
        let printed = painted_text(&shapes);
        assert!(printed.iter().any(|t| t == "Bob [Region II]"), "the title line is still drawn");
        assert!(
            !printed.iter().any(|t| t.starts_with("dr/dt") || t.starts_with("tau")),
            "and no reading under it is: {printed:?}"
        );

        telemetry_frame(&ctx, &mut boxes, &lines, vec![press(at)]);
        let shapes = telemetry_frame(&ctx, &mut boxes, &lines, vec![release(at)]);
        assert!(!boxes.collapsed.contains(&key), "a second click opens the box again");
        assert_eq!(painted_card(&shapes), open, "exactly as it was");
        assert!(painted_text(&shapes).iter().any(|t| t == "tau 12.5 M"), "readings and all");
    }

    /// The triangle takes the click and nothing else does: within the layer it is registered after
    /// the box, so the press reaches the triangle rather than starting a drag or counting towards
    /// the box's double-click reset. A press on the title text itself still drags the box.
    #[test]
    fn test_the_triangle_takes_the_click_and_the_title_line_keeps_the_drag() {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        // The pinning canvases are the strict case: there a drag is what creates the placement at
        // all, so a placement appearing or vanishing is visible in the map.
        let mut boxes = TelemetryBoxes::pinning();
        let lines = sample_lines();
        let key = (Canvas::Spacetime, BoxId::Observer(Who::Bob));

        let anchored = painted_card(&telemetry_frame(&ctx, &mut boxes, &lines, vec![]));
        assert!(boxes.placements.is_empty(), "an untouched box has no placement");

        // A press on the title line, well clear of the triangle's column, and then two frames of
        // motion: egui starts the drag on the frame that crosses its own threshold.
        let m = box_metrics(1.0);
        let on_title = Pos2::new(anchored.left() + m.pad_x + m.column + 30.0, anchored.top() + 8.0);
        telemetry_frame(&ctx, &mut boxes, &lines, vec![egui::Event::PointerMoved(on_title)]);
        telemetry_frame(&ctx, &mut boxes, &lines, vec![press(on_title)]);
        for step in [Vec2::new(30.0, 20.0), Vec2::new(50.0, 40.0)] {
            let to = on_title + step;
            telemetry_frame(&ctx, &mut boxes, &lines, vec![egui::Event::PointerMoved(to)]);
        }
        telemetry_frame(&ctx, &mut boxes, &lines, vec![release(on_title + Vec2::new(50.0, 40.0))]);
        let dragged = painted_card(&telemetry_frame(&ctx, &mut boxes, &lines, vec![]));
        assert!(
            (dragged.min - anchored.min).length() > 30.0,
            "a drag from the title text still moves the box: {:?} from {:?}",
            dragged.min,
            anchored.min
        );
        let placement = boxes.placements.get(&key).copied().expect("and the move is remembered");

        // Now the triangle, on the box where the drag left it.
        let at = disclosure_hit_rect(dragged, 1.0).center();
        telemetry_frame(&ctx, &mut boxes, &lines, vec![egui::Event::PointerMoved(at)]);
        telemetry_frame(&ctx, &mut boxes, &lines, vec![press(at)]);
        let shapes = telemetry_frame(&ctx, &mut boxes, &lines, vec![release(at)]);
        assert!(boxes.collapsed.contains(&key), "the click shut the box");
        assert_eq!(
            boxes.placements.get(&key).copied(),
            Some(placement),
            "and left the placement alone - it neither moved the box nor reset the drag"
        );
        assert_eq!(
            painted_card(&shapes).min,
            dragged.min,
            "a dragged box keeps its corner as it shuts, too"
        );
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
        // FRAME_SURFACE_FRACTION of the way up the half-height. Where the surface r = r_h crosses
        // the observer's own time axis is the affine length of the observer's own worldline from
        // here to the crossing, which is what the drawn curve crosses that axis at, so that is the
        // number checked here.
        //
        // This test used to check the first-order (r_h - r) / u^r, because that is what the
        // framing used to follow and what the straight lines of the old picture were drawn from.
        // Both have been replaced by the exact quadrature, so the subject of the assertion has
        // moved with them: the two numbers differ by most of a gravitational radius close in, and
        // asking for the old one now would be asking the framing to disagree with the picture.
        let lands_at = |obs: &Observer, r_h: f64, window: f64| -> f32 {
            let scale = (rect.width() / window as f32) * 0.45;
            let xi0 = affine_length_to_surface(
                &metric,
                obs.r,
                &obs.four_velocity(&metric),
                r_h,
            )
            .expect("the surface is ahead of this observer");
            xi0 as f32 * scale
        };
        let wanted_px = FRAME_SURFACE_FRACTION * rect.height() * 0.5;

        // 1. Just above the outer horizon, the next surface is r+ and not the two below it: a
        //    raindrop at r = 1.5 M is a few hundredths of an M of proper time from r+ and most of
        //    an M from r-.
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
        let tau_left = affine_length_to_surface(&metric, falling.r, &u, rm)
            .expect("r- is a finite proper time below the freeze");
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
            // The boxes start shut, and the frequencies are what an open one prints.
            canvas.telemetry.collapsed.clear();
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

    /// The main canvas rectangle of a painted frame, read off the one shape that is always there:
    /// the background, which every chart lays down over the whole canvas before anything else.
    /// The tallest such fill is the chart's. Read rather than recomputed, because the layout that
    /// decides it - the panel's margins and the header - is not what any of these tests is about.
    pub(super) fn canvas_rect(shapes: &[egui::Shape]) -> Rect {
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
            // One filled path on this chart carries Bob's colour and is no part of a wedge: the
            // disclosure triangle on the title line of Bob's own info box. It is three points
            // inside a ten-point square, where a wedge fill was a strip of quads laid across the
            // whole chart, so the size tells the two apart without naming the box.
            let bounds = Rect::from_points(&path.points);
            let triangle = path.points.len() == 3 && bounds.width().max(bounds.height()) <= 10.0;
            assert!(
                a == 0 || triangle || [r, g, b] != bob_rgb,
                "a fill in the transmission's own colour"
            );
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

/// The rest-frame view, driven through real frames of `SpacetimeCanvas::render`.
///
/// Everything here is measured off the shapes a painter was actually handed, and converted back
/// into the chart's own (xi^1, xi^0) with the same two numbers the view uses: the canvas rectangle
/// is read off the background it paints, and the pixel scale is `rect.width() / frame_max_r * 0.45`
/// with `frame_max_r` taken from the canvas after the pass, which is the value that pass drew with.
#[cfg(test)]
mod rest_frame_tests {
    use super::*;
    use crate::physics::observer::{Release, WorldlineParams};

    /// One painted frame of an observer's rest frame, with everything the assertions read.
    struct Pass {
        shapes: Vec<egui::Shape>,
        text: String,
        rect: Rect,
        /// The window the pass was drawn at, so that a screen position can be turned back into the
        /// chart position it came from.
        frame_max_r: f64,
    }

    impl Pass {
        fn scale(&self) -> f64 {
            (self.rect.width() as f64 / self.frame_max_r.max(1e-300)) * 0.45
        }

        /// A painted point turned back into the chart coordinates (xi^1, xi^0) it was drawn from.
        fn to_chart(&self, p: Pos2) -> [f64; 2] {
            let c = self.rect.center();
            [
                (p.x - c.x) as f64 / self.scale(),
                (c.y - p.y) as f64 / self.scale(),
            ]
        }

        /// The as-seen dot: the one filled circle of the marker's size *on the chart*. Every other
        /// circle the chart paints is a different size - the focus marker is 7.5, a received crest
        /// 2.5, the singularity glyph 12 and 15 - so the radius is the identity there. The
        /// rectangle is still asked for, so that the dot read is one the canvas actually shows.
        fn seen_dot(&self) -> Option<Pos2> {
            self.shapes.iter().find_map(|s| match s {
                egui::Shape::Circle(c)
                    if (c.radius - 6.0).abs() < 1e-6 && self.rect.contains(c.center) =>
                {
                    Some(c.center)
                }
                _ => None,
            })
        }

        /// The dashes of the ring the view draws in place of a dot when the eye has nothing left
        /// to see: short neutral-grey segments, which is what `Shape::dashed_line` makes of the
        /// ring's polyline and nothing else on this canvas paints.
        fn ring_dashes(&self) -> Vec<[Pos2; 2]> {
            self.shapes
                .iter()
                .filter_map(|s| match s {
                    egui::Shape::LineSegment { points, stroke }
                        if stroke.color == Theme::TEXT_MUTED
                            && points[0].distance(points[1]) <= 4.0
                            && self.rect.contains(points[0]) =>
                    {
                        Some(*points)
                    }
                    _ => None,
                })
                .collect()
        }

        /// The centre of that ring, read off the bounding box of its dash ends.
        ///
        /// `None` unless the whole ring is on the canvas. A ring the canvas edge has cut through
        /// has a bounding box narrower than the ring is, and a centre taken from it would be a
        /// number about the clipping rather than about where the light came from.
        fn ring_centre(&self) -> Option<Pos2> {
            let dashes = self.ring_dashes();
            if dashes.is_empty() {
                return None;
            }
            let mut bounds = Rect::NOTHING;
            for end in dashes.iter().flatten() {
                bounds.extend_with(*end);
            }
            let whole = |side: f32| (side - 13.0).abs() < 1.5;
            (whole(bounds.width()) && whole(bounds.height())).then(|| bounds.center())
        }

        /// Every polyline stroked in `colour`, as chart points. The surface curves are the only
        /// paths this canvas strokes in a surface's own colour.
        fn curve(&self, colour: Color32) -> Vec<Vec<[f64; 2]>> {
            self.shapes
                .iter()
                .filter_map(|s| match s {
                    egui::Shape::Path(p) if path_stroke_colour(p) == Some(colour) => {
                        Some(p.points.iter().map(|q| self.to_chart(*q)).collect::<Vec<_>>())
                    }
                    _ => None,
                })
                .collect()
        }

        /// How many wedges of `who`'s light cone were painted. A cone is two filled triangles in
        /// that observer's own cone colours, and nothing else on this canvas is filled in them -
        /// an info box's disclosure triangle is a filled triangle too, which is why the fill is
        /// what this matches on rather than the shape.
        fn cone_wedges(&self, who: Who) -> usize {
            let (future, past, _) = Theme::cone_colours(Some(who));
            self.shapes
                .iter()
                .filter(|s| match s {
                    egui::Shape::Path(p) => {
                        p.closed && p.points.len() == 3 && (p.fill == future || p.fill == past)
                    }
                    _ => false,
                })
                .count()
        }
    }

    /// The solid colour a stroked path carries, or `None` for a graded one (a crest or a comet).
    fn path_stroke_colour(p: &PathShape) -> Option<Color32> {
        match &p.stroke.color {
            egui::epaint::ColorMode::Solid(c) => Some(*c),
            egui::epaint::ColorMode::UV(_) => None,
        }
    }

    /// Render one frame of the named rest frame and collect what it painted.
    ///
    /// The boxes are opened first: `SpacetimeCanvas::default` starts every box on this canvas shut
    /// down to its title line, which is right for the app and would leave these assertions reading
    /// four titles and no numbers.
    fn pass(
        metric: &KerrSchild,
        alice: Option<&Observer>,
        bob: Option<&Observer>,
        frame: ReferenceFrame,
        window: f64,
    ) -> Pass {
        let mut canvas = fresh(window);
        pass_on(&mut canvas, metric, alice, bob, frame)
    }

    /// A canvas opened on a window of `window` M of xi with the automatic framing off.
    ///
    /// The framing is off because these tests are about what is drawn rather than about the zoom,
    /// and the window is named because the default 5.5 M is chosen for the app's own pair: a
    /// hovering watcher a long way from an infaller sees that infaller several M down the past
    /// cone, which is off the bottom of a 5.5 M canvas, and a test that drew it there would be
    /// asserting things about the clipping.
    fn fresh(window: f64) -> SpacetimeCanvas {
        SpacetimeCanvas { frame_max_r: window, keep_surface_framed: false, ..Default::default() }
    }

    /// The same frame drawn on a canvas that is kept between calls, which is what the app has: the
    /// view holds the last frame's as-seen answer and warm-starts the next solve from it. A test
    /// that made a fresh canvas every frame would be asking for a cold solve every frame, which is
    /// neither what the app does nor what the solver is tuned for.
    fn pass_on(
        canvas: &mut SpacetimeCanvas,
        metric: &KerrSchild,
        alice: Option<&Observer>,
        bob: Option<&Observer>,
        frame: ReferenceFrame,
    ) -> Pass {
        canvas.telemetry.collapsed.clear();
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
            ..Default::default()
        };
        let silent = SignalField::default();
        let now = alice.map_or(0.0, |a| a.t).max(bob.map_or(0.0, |b| b.t));
        let output = ctx.run_ui(input, |ui| {
            canvas.render(
                ui,
                metric,
                bob,
                alice,
                now,
                600.0,
                false,
                frame,
                1.0,
                SignalViews { alice: &silent, bob: &silent },
                false,
            );
        });
        let mut shapes = Vec::new();
        let mut text = String::new();
        for clipped in output.shapes.iter() {
            flatten(&clipped.shape, &mut shapes);
        }
        for shape in &shapes {
            if let egui::Shape::Text(t) = shape {
                text.push_str(t.galley.text());
                text.push('\n');
            }
        }
        output.drop_without_applying_deltas();
        let rect = super::canvas_tests::canvas_rect(&shapes);
        Pass { shapes, text, rect, frame_max_r: canvas.frame_max_r }
    }


    /// An observer hovering at `r`, at rest and staying there.
    fn hovering(metric: &KerrSchild, name: &str, r: f64, phi: f64) -> Observer {
        let mut obs = Observer::new_with_phi(
            metric,
            name,
            0.0,
            r,
            0.0,
            phi,
            WorldlineParams::released(metric, r, 0.0, Release::AtRest),
        );
        obs.mode = ObserverMode::Static;
        obs.is_active = true;
        obs
    }

    /// A raindrop: E = 1, L = 0, released at once.
    fn raindrop(metric: &KerrSchild, name: &str, r: f64, phi: f64) -> Observer {
        Observer::new_with_phi(metric, name, 0.0, r, 0.0, phi, WorldlineParams::new(1.0, 0.0, false))
    }

    /// Step both observers on the one simulation clock, exactly as `Simulation` does.
    fn play(metric: &KerrSchild, a: &mut Observer, b: &mut Observer, steps: usize, dt: f64) {
        let mut t = a.t.max(b.t);
        for _ in 0..steps {
            t += dt;
            a.step(metric, t, dt);
            b.step(metric, t, dt);
        }
    }

    /// Where a polar curve about the origin crosses the ray at chart angle `psi`, as a radius.
    ///
    /// The surface curves are polar by construction - `normal_coords::sample_surface` sweeps the
    /// direction angle and puts each point at that direction's own affine length - so one radius
    /// per angle is the whole of a curve's shape, and this is the number a "which side of the
    /// surface is the dot on" question turns on. `None` when the curve has no painted point at that
    /// angle, which is what happens when the crossing lies off the canvas.
    fn curve_radius_at(runs: &[Vec<[f64; 2]>], psi: f64) -> Option<f64> {
        let (want_s, want_c) = psi.sin_cos();
        let mut best: Option<f64> = None;
        for run in runs {
            for pair in run.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                // The ray crosses the chord where the two ends lie on opposite sides of the line
                // through the origin along (cos psi, sin psi).
                let cross = |p: [f64; 2]| p[0] * want_s - p[1] * want_c;
                let (ca, cb) = (cross(a), cross(b));
                if ca == cb {
                    continue;
                }
                if ca != 0.0 && cb != 0.0 && (ca > 0.0) == (cb > 0.0) {
                    continue;
                }
                let t = ca / (ca - cb);
                let x = a[0] + t * (b[0] - a[0]);
                let y = a[1] + t * (b[1] - a[1]);
                // The same half-line, not the opposite one.
                if x * want_c + y * want_s <= 0.0 {
                    continue;
                }
                let radius = x.hypot(y);
                if best.is_none_or(|r| radius < r) {
                    best = Some(radius);
                }
            }
        }
        best
    }

    #[test]
    fn test_the_isco_view_puts_the_seen_dot_on_the_cone_edge_and_inside_the_horizon_curve() {
        // The defect this change was for, at the configuration that showed it. Alice orbits on
        // the prograde ISCO of an a = 0.90 hole, where the orbital speed against the hovering
        // observers is about 0.9 c and against the ZAMO about 0.6 c, so the light reaching her
        // from Bob - the app's own raindrop, dropped from 27 M - arrives strongly aberrated and
        // the arrival direction has a large azimuthal component. Projected onto the radial plane
        // the dot lost that component: it sat inside the 45-degree past cone, and once Bob's
        // image had piled up against r+ the shortening carried the dot *through* the drawn r+
        // curve while the emission event was still plainly in Region I.
        //
        // With the drawn plane turned onto the line of sight there is nothing left to lose. The
        // dot is at (lambda, -lambda) by construction, and the r+ curve's point in that same
        // direction is the affine length along the same geodesic direction, so the two are
        // comparable and the dot can only pass the curve when the light really did come from
        // beyond r+.
        //
        // Most of the time there is no such point to compare against, and that is physics too:
        // the ray along the line of sight, followed into the past beyond the emission event, is a
        // ray that came from a source outside r+, so it either turned at a periapsis outside r+
        // or it came from the past horizon further back than the window shows. Measured over this
        // run the r+ curve never stood in the dot's direction at all - it lay in the future and
        // inward sectors, at 10 M and more - so the comparison is made wherever it can be and its
        // count is reported rather than required.
        //
        // The image drawn is the youngest one (`as_seen_youngest`), which is what the canvas
        // draws, and the window is wide enough to hold it: from this orbit the youngest image's
        // affine length swings up past 30 M before the direct image is born.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let mut alice = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let mut bob = raindrop(&metric, "Bob", 27.0, 0.0);
        let mut canvas = fresh(40.0);

        let mut checked = 0;
        let mut against_curve = 0;
        let mut widest_angle = 0.0f64;
        let mut deepest = f64::INFINITY;
        let mut outward_frames = 0;
        let mut inward_frames = 0;
        for step in 1..=1200 {
            play(&metric, &mut alice, &mut bob, 1, 0.1);
            let p = pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Alice);
            // Every tenth frame, which is a frame every M of coordinate time: the assertions are
            // about the placement rather than about the run, and a render a frame for 120 M of a
            // Kerr solve is a minute of test time for no more evidence.
            if step % 10 != 0 {
                continue;
            }
            let seen =
                as_seen_youngest(&metric, &alice, &bob, None).expect("Alice sees Bob throughout");
            // The exact statement first, before any of it has been through a screen coordinate:
            // in this plane the emission event is on the past cone edge, on the source's side,
            // which is the right-hand edge while Bob is outward of Alice and the left-hand edge
            // once the line of sight to him has swung inward.
            let xi = seen.xi_plane();
            let side = seen.sight_sign();
            assert!(
                (xi[0] - side * seen.lambda).abs() < 1e-12 * seen.lambda.max(1.0)
                    && (xi[1] + seen.lambda).abs() < 1e-12 * seen.lambda.max(1.0),
                "the drawn point must be (+/- lambda, -lambda): {xi:?} against {}",
                seen.lambda
            );
            assert_eq!(
                side,
                if seen.n[0] > 0.0 { 1.0 } else { -1.0 },
                "the side is the sign of the outward component of the line of sight: n = {:?}",
                seen.n
            );
            if side > 0.0 {
                outward_frames += 1;
            } else {
                inward_frames += 1;
            }
            widest_angle = widest_angle.max(seen.sight_angle().abs().to_degrees());

            let Some(drawn) = p.seen_dot().or_else(|| p.ring_centre()) else {
                continue;
            };
            let painted = p.to_chart(drawn);
            checked += 1;
            // Half a screen point in chart units, which is the resolution the dot was painted at.
            let tol = 0.5 / p.scale();
            assert!(
                (painted[1] + painted[0].abs()).abs() < tol,
                "the painted dot must sit on the 45-degree past cone: {painted:?}"
            );
            // The side the canvas painted the dot on has to agree with the direction the canvas
            // printed for the line of sight. Read off the painting rather than off `seen`,
            // because the canvas re-checks for the youngest image once per M and the fresh solve
            // above can be a frame ahead of it, on a different image with the other sign.
            let painted_inward = painted[0] < 0.0;
            let printed_inward = p.text.contains("of inward") || p.text.contains("straight inward");
            let printed_outward =
                p.text.contains("of outward") || p.text.contains("straight outward");
            assert!(
                printed_inward != printed_outward,
                "the header names one radial direction and only one: {}",
                p.text
            );
            assert_eq!(
                painted_inward, printed_inward,
                "the dot is on the left exactly when the line of sight is named from inward: \
                 painted at {painted:?}, text {}",
                p.text
            );

            // Against the r+ curve along the dot's own direction.
            let psi = painted[1].atan2(painted[0]);
            if let Some(surface) = curve_radius_at(&p.curve(Theme::HORIZON_OUTER), psi) {
                against_curve += 1;
                let radius = painted[0].hypot(painted[1]);
                let margin = surface - radius;
                deepest = deepest.min(margin);
                assert!(
                    seen.emission.r <= metric.outer_horizon() || margin > -tol,
                    "the dot is past the r+ curve at {radius} M against {surface} M while the \
                     emission event stands at r = {} M, outside r+ = {}",
                    seen.emission.r,
                    metric.outer_horizon()
                );
            }
            assert!(
                seen.emission.r > metric.outer_horizon(),
                "this run never sees Bob cross: seen at r = {}",
                seen.emission.r
            );
            // Region I, which the box tags "Ergo" once the emission event is inside r_E.
            assert!(
                p.text.contains("Bob as seen [Region I]") || p.text.contains("Bob as seen [Ergo]"),
                "and the box says which region the emission event is in: {}",
                p.text
            );
        }
        println!(
            "{checked} frames of the prograde ISCO watching a raindrop from 27 M: the dot stayed \
             on the past cone edge every time, the line of sight swung as far as {widest_angle:.1} \
             degrees from outward, and in {against_curve} of the frames the r+ curve stood along \
             the dot's own direction - the dot's closest approach to it was {deepest:.4} M"
        );
        assert!(checked >= 80, "only {checked} frames had anything drawn");
        assert!(
            widest_angle > 60.0,
            "the point of the case is a line of sight well off radial: {widest_angle} degrees"
        );
        println!("{outward_frames} frames put Bob on the right-hand edge and {inward_frames} on the left");
        assert!(
            outward_frames > 0 && inward_frames > 0,
            "this run swings the line of sight through both halves: {outward_frames} outward, \
             {inward_frames} inward"
        );

        // And the consistency the signal crests have to share. A crest is placed by pushing the
        // arriving ray's own null direction through this same frame and dropping whatever lies
        // along the third leg - see `wave_crests` - and the ray that carries Bob's newest pulse
        // to Alice is the ray that carries Bob's image, because there is only one null connection
        // between those two events. So that direction has no third component left to drop: in
        // the frame's own (time, along the leg, third) order it comes through as exactly
        // (1, -/+ 1, 0), a future-directed 45-degree stroke through the arrival that runs away
        // from the dot along the leg, on whichever side the dot is.
        let seen = as_seen_youngest(&metric, &alice, &bob, None).expect("Alice still sees Bob");
        let u = alice.four_velocity(&metric);
        let axial = Tetrad::from_four_velocity_axial(&metric, alice.r, &u);
        let leg = seen.plane_leg();
        let s_leg: [f64; 3] =
            core::array::from_fn(|mu| leg[0] * axial.e1[mu] + leg[1] * axial.e2[mu]);
        let frame = LocalFrame::for_observer_plane(&metric, alice.r, &u, &s_leg);
        let arriving = axial.null_direction(seen.alpha + std::f64::consts::PI);
        let local = frame.vector_to_local(&arriving);
        let side = seen.sight_sign();
        println!(
            "the arriving ray's direction in the drawn frame is {local:?}, against (1, {}, 0)",
            -side
        );
        assert!(
            (local[0] - 1.0).abs() < 1e-9
                && (local[1] + side).abs() < 1e-9
                && local[2].abs() < 1e-9,
            "the light arrives in the drawn plane, along the line of sight: {local:?}"
        );
    }

    #[test]
    fn test_the_picture_is_held_while_the_observers_clock_stands_still() {
        // The user's report: Bob freezing onto the far branch of r- with Alice's image deep in
        // redshift, and on one frame in three the box read "Alice: no image" and the drawn plane
        // snapped to the radial one, so r- flipped from a shallow line across his future to a
        // 45-degree line and back. Past u^t ~ 1e7 the solver cannot resolve his frame, but his
        // clock is moving under 1e-8 M a frame, so the picture at his event is the picture it was:
        // the view holds it and says so, and the plane stays put.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.2, false),
        );
        let mut alice = raindrop(&metric, "Alice", 4.45, 0.0);
        let mut canvas = fresh(16.0);
        let mut no_image_frames = 0;
        let mut held_frames = 0;
        let mut frames = 0;
        let mut angle: Option<String> = None;
        let mut angle_changes = 0;
        while bob.four_velocity(&metric)[0] < 1e8 {
            play(&metric, &mut bob, &mut alice, 1, 0.05);
            assert!(!bob.is_frozen(), "the run must reach u^t = 1e8 before the stall");
            let p = pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Bob);
            frames += 1;
            if p.text.contains("no image") {
                no_image_frames += 1;
            }
            if p.text.contains("picture held") {
                held_frames += 1;
            }
            // The header's direction, as the plane's witness: it may not flip to the radial
            // fallback and back once the picture is deep.
            if bob.four_velocity(&metric)[0] > 1e5 {
                let now = p
                    .text
                    .lines()
                    .find(|line| line.contains("of outward") || line.contains("of inward"))
                    .map(str::to_string);
                if angle.is_some() && now != angle {
                    angle_changes += 1;
                }
                angle = now;
            }
        }
        println!(
            "{frames} frames to u^t = {:.2e}: {no_image_frames} with no image,              {held_frames} held, the header's direction changed {angle_changes} times past              u^t = 1e5",
            bob.four_velocity(&metric)[0]
        );
        assert_eq!(no_image_frames, 0, "the picture never goes out");
        assert!(held_frames > 0, "and it is held once his clock has stopped");
        assert_eq!(angle_changes, 0, "and the plane does not snap");
    }

    #[test]
    fn test_the_other_observer_is_drawn_on_the_past_cone_and_never_past_the_horizon() {
        // The two statements the placement has to make, over a long run rather than at one moment.
        //
        // On or inside the past light cone, because the dot is at xi = lambda (-1, n^1, n^2) and
        // the view drops n^2: |xi^1| <= lambda, with equality only for light that arrives with no
        // azimuthal component at all. The first-order placement of the same-coordinate-time offset
        // this replaced had no such bound and could put the other observer outside the cone, which
        // no arriving light can do.
        //
        // And never on the far side of the drawn r+ curve, which is the other half of the picture
        // being one picture: a hovering observer never sees an infaller cross, so the seen event
        // has to pile up *outside* r+ and stay there, on the curve this same canvas draws.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut alice = hovering(&metric, "Alice", 8.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 6.0, 0.0);

        let mut checked = 0;
        let mut against_curve = 0;
        let mut closest = f64::INFINITY;
        // One canvas for the whole run, as the app has one: the view keeps the last answer and
        // warm-starts the next solve from it.
        let mut canvas = fresh(16.0);
        for _ in 0..40 {
            play(&metric, &mut alice, &mut bob, 10, 0.05);
            let p = pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Alice);
            // In ring mode there is no dot, and the ring stands at the same exact place.
            let Some(drawn) = p.seen_dot().or_else(|| p.ring_centre()) else {
                continue;
            };
            let xi = p.to_chart(drawn);
            checked += 1;
            // A tolerance of half a screen point in chart units, which is the resolution the dot
            // was painted at: `Pos2` is f32 and a ring's centre is an average of dash ends.
            let tol = 0.5 / p.scale();
            assert!(
                xi[1] <= -xi[0].abs() + tol,
                "the seen event must sit on or inside the past cone: xi = {xi:?}"
            );
            let psi = xi[1].atan2(xi[0]);
            if let Some(surface) = curve_radius_at(&p.curve(Theme::HORIZON_OUTER), psi) {
                against_curve += 1;
                let radius = xi[0].hypot(xi[1]);
                assert!(
                    radius <= surface + tol,
                    "the seen event is past the drawn r+ curve: {radius} against {surface} at \
                     psi = {psi}"
                );
                closest = closest.min(surface - radius);
            }
        }
        println!(
            "{checked} frames of a hovering watcher: the seen event stayed on the past cone every \
             time, and in {against_curve} of them the drawn r+ curve was in reach of its own \
             direction - the closest the image came to that curve was {closest:.4} M"
        );
        assert!(checked >= 30, "only {checked} frames had anything drawn");
        assert!(against_curve >= 10, "only {against_curve} frames could be checked against r+");
        assert!(closest >= 0.0, "the image crossed the curve by {}", -closest);
    }

    #[test]
    fn test_only_the_focus_observer_gets_a_light_cone() {
        // The other observer's cone has gone, and for a reason rather than for tidiness: a cone
        // drawn at the seen event would be the future of an event whose future this observer has
        // not received. What is left is the focus observer's own two wedges.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut alice = hovering(&metric, "Alice", 8.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 6.0, 0.4);
        play(&metric, &mut alice, &mut bob, 40, 0.05);

        let both = pass(&metric, Some(&alice), Some(&bob), ReferenceFrame::Alice, 16.0);
        assert!(
            both.seen_dot().is_some() || both.ring_centre().is_some(),
            "Bob has to be drawn at all for this to mean anything"
        );
        assert_eq!(
            both.cone_wedges(Who::Alice),
            2,
            "one cone, two wedges: the focus observer's own future and past"
        );
        assert_eq!(
            both.cone_wedges(Who::Bob),
            0,
            "and not one wedge of a cone for the observer who is only being seen"
        );
        let alone = pass(&metric, Some(&alice), None, ReferenceFrame::Alice, 16.0);
        assert_eq!(
            alone.cone_wedges(Who::Alice),
            2,
            "and the focus observer's count does not depend on the other observer at all"
        );
    }

    #[test]
    fn test_the_other_observers_box_reports_the_emission_event() {
        // What the box says, and what it no longer says. The title stands alone because the box
        // starts shut, so the title has to carry both the name and the region the *seen* event is
        // in; the lines under it are the ordinary telemetry read at that event, followed by what
        // the light did on the way here. The three lines this replaced - "In Alice's frame:", an
        // azimuthal offset under it and a first-order radial speed - are gone.
        //
        // The azimuthal offset xi^2 has gone too, and for a better reason than tidiness: the view
        // now draws the plane that contains the line of sight, so the whole of the offset lies in
        // the picture and the third component is zero by construction. What stands in its place
        // is the direction that plane points in.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut alice = hovering(&metric, "Alice", 8.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 6.0, 0.0);
        play(&metric, &mut alice, &mut bob, 60, 0.05);
        let p = pass(&metric, Some(&alice), Some(&bob), ReferenceFrame::Alice, 16.0);
        println!("Alice's view of Bob:\n{}", p.text);

        assert!(p.text.contains("Bob as seen ["), "the title names the seen event: {}", p.text);
        for wanted in [
            "light left =",
            "Bob's watch =",
            "shift = 1+z =",
            "beacon ",
            "brightness = ×g⁴",
            "watch rate as seen = ×g",
            "line of sight =",
        ] {
            assert!(p.text.contains(wanted), "no {wanted:?} line in the box: {}", p.text);
        }
        assert!(p.text.contains("dr/dt"), "the box still carries the telemetry: {}", p.text);
        // Alice hovers straight above Bob at the same azimuth of a hole with no spin, so the light
        // comes to her straight in and the box says so rather than quoting an angle of nothing.
        assert!(
            p.text.contains("line of sight = straight inward"),
            "the radial pair must read as radial: {}",
            p.text
        );
        for gone in ["In Alice's frame:", "radial speed", "azimuthal offset"] {
            assert!(!p.text.contains(gone), "{gone:?} is still being printed: {}", p.text);
        }

        // The telemetry really is read at the emission event and not at Bob's present one. Bob has
        // fallen a long way in three M of coordinate time, and what Alice sees is where he was.
        let seen = as_seen(&metric, &alice, &bob, None).expect("Alice sees Bob");
        println!(
            "Bob stands at r = {:.4} M and is seen at r = {:.4} M, {:.4} M of affine length away, \
             at g = {:.4}",
            bob.r, seen.emission.r, seen.lambda, seen.g
        );
        assert!(
            seen.emission.r > bob.r + 0.2,
            "the test wants a visible gap between the two events: {} against {}",
            seen.emission.r,
            bob.r
        );
        let snapshot = seen_snapshot(&bob, &seen);
        let u = snapshot.four_velocity(&metric);
        for (a, b) in u.iter().zip(seen.emission.u.iter()) {
            assert!(
                (a - b).abs() <= 1e-9 * (1.0 + b.abs()),
                "the snapshot must carry the emission 4-velocity: {u:?} against {:?}",
                seen.emission.u
            );
        }
        let at_emission = telemetry_lines("Bob", Theme::BOB_COLOR, &snapshot, &metric, false);
        let at_now = telemetry_lines("Bob", Theme::BOB_COLOR, &bob, &metric, false);
        assert_ne!(
            at_emission[1].text, at_now[1].text,
            "the box would be reading the present event, not the emission event"
        );
        assert!(
            p.text.contains(&at_emission[1].text),
            "the emission event's own dr/dt must be the line in the box: {:?} in {}",
            at_emission[1].text,
            p.text
        );
    }

    #[test]
    fn test_a_beacon_too_far_shifted_to_see_is_drawn_as_a_dashed_ring() {
        // The hovering watcher's endgame. The image of the infaller piles up on r+ with g running
        // to nothing, so the beacon leaves the visible band and the view stops painting a colour
        // it has not got: no filled dot, a dashed ring at the same exact position, and a box that
        // says in words where the light has gone.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut alice = hovering(&metric, "Alice", 8.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 6.0, 0.0);
        // Played frame by frame with the answer carried forward, exactly as the app plays it. The
        // cold fan gives up on an image this far down the redshift - it has to bracket a crossing
        // that has piled up within a millionth of an M of r+ - and the warm Newton does not, which
        // is why `AsSeen::seed` exists and why the view keeps one.
        let mut seed = None;
        let mut seen = as_seen(&metric, &alice, &bob, seed).expect("the pair start out visible");
        for _ in 0..120 {
            play(&metric, &mut alice, &mut bob, 5, 0.05);
            seen = as_seen(&metric, &alice, &bob, seed).expect("the image never goes out");
            seed = Some(seen.seed());
        }
        println!(
            "after 30 M Alice sees Bob at r = {:.5} M with g = {:.3e}, which puts his {} nm beacon \
             at {}",
            seen.emission.r,
            seen.g,
            beacon_colour::BOB_BEACON_NM,
            beacon_colour::wavelength_label(beacon_colour::BOB_BEACON_NM / seen.g)
        );
        assert!(seen.g < 0.05, "the test wants a deep redshift, got g = {}", seen.g);

        // The canvas is handed the same run, frame by frame, so that its own seed is the one the
        // run built rather than a cold start on the last frame alone.
        let mut canvas = fresh(16.0);
        let mut alice_back = hovering(&metric, "Alice", 8.0, 0.0);
        let mut bob_back = raindrop(&metric, "Bob", 6.0, 0.0);
        let mut p = pass_on(
            &mut canvas,
            &metric,
            Some(&alice_back),
            Some(&bob_back),
            ReferenceFrame::Alice,
        );
        for _ in 0..120 {
            play(&metric, &mut alice_back, &mut bob_back, 5, 0.05);
            p = pass_on(
                &mut canvas,
                &metric,
                Some(&alice_back),
                Some(&bob_back),
                ReferenceFrame::Alice,
            );
        }
        assert!(p.seen_dot().is_none(), "nothing may be painted in a colour the eye has not got");
        let dashes = p.ring_dashes();
        println!("the ring came back as {} dashes", dashes.len());
        assert!(dashes.len() >= 6, "a dashed ring wants dashes, got {}", dashes.len());
        // Every dash lies on one circle of the marker's size, which is what makes the ring a ring
        // and not a scatter.
        let centre = p.ring_centre().expect("the dashes have a centre");
        for end in dashes.iter().flatten() {
            let radius = end.distance(centre);
            assert!(
                (radius - 6.5).abs() < 1.0,
                "a dash end stands {radius} from the ring's centre, not 6.5"
            );
        }
        assert!(p.text.contains("infrared"), "the box has to say where the light went: {}", p.text);
        // The ring stands where the exact solve put it, not at some fallback.
        let xi = p.to_chart(centre);
        let want = seen.xi_plane();
        assert!(
            (xi[0] - want[0]).abs() < 2.0 / p.scale() && (xi[1] - want[1]).abs() < 2.0 / p.scale(),
            "the ring must stand on the seen event: {xi:?} against {want:?}"
        );
    }

    #[test]
    fn test_no_image_still_answers_in_a_box_and_draws_no_dot() {
        // The focus observer has reached the ring, so there is no more of that worldline for light
        // to arrive on. That is a physical statement and the box makes it; what the view must not
        // do is quietly draw the other observer somewhere anyway.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut alice = raindrop(&metric, "Alice", 6.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 9.0, 0.5);
        let mut guard = 0;
        while alice.r > crate::physics::geodesic::R_STOP && guard < 20_000 {
            guard += 1;
            play(&metric, &mut alice, &mut bob, 1, 0.02);
        }
        assert!(alice.r <= crate::physics::geodesic::R_STOP, "Alice must reach the ring");
        assert_eq!(
            as_seen(&metric, &alice, &bob, None),
            Err(NoImage::FocusEnded),
            "the solver has to agree about why"
        );

        let p = pass(&metric, Some(&alice), Some(&bob), ReferenceFrame::Alice, 16.0);
        println!("Alice's view once she has reached the ring:\n{}", p.text);
        assert!(p.text.contains("Bob: no image"), "the box is still queued: {}", p.text);
        assert!(
            p.text.contains("no more worldline to receive on"),
            "and says why in words: {}",
            p.text
        );
        assert!(p.seen_dot().is_none(), "no dot for an image that does not exist");
        assert!(p.ring_dashes().is_empty(), "and no ring either: a ring means light still arrives");
    }

    #[test]
    fn test_the_surface_curves_are_sampled_once_per_event_and_not_once_per_frame() {
        // The exact curves cost about a quarter of a millisecond for all four surfaces, which is
        // worth paying when the observer has moved and is not worth paying sixty times a second
        // while the run is paused. The cache key is the observer's event, the hole and the window,
        // so a frame that changes none of them reuses the curves - and a frame that changes any of
        // them must not.
        //
        // The perf suite's `paint/spacetime-canvas` row paints the (t, r) foliation chart and never
        // this view, so the numbers printed here are the only measurement of the rest frame's own
        // pass that the tree carries.
        // A falling focus, because a hovering one never moves: a static observer's event, and so
        // the key, is the same on every frame of the run, and a cache that looked clever on that
        // observer would say nothing about the case that costs.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut alice = raindrop(&metric, "Alice", 8.0, 0.0);
        let mut bob = raindrop(&metric, "Bob", 6.0, 0.3);
        play(&metric, &mut alice, &mut bob, 20, 0.05);

        let mut canvas = fresh(16.0);
        let start = std::time::Instant::now();
        pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Alice);
        let cold = start.elapsed();
        let first = canvas.surfaces.key.expect("the first pass samples the curves");
        let points: usize = canvas.surfaces.runs.iter().flatten().map(|r| r.points.len()).sum();

        let start = std::time::Instant::now();
        let repeats = 20;
        for _ in 0..repeats {
            pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Alice);
        }
        let paused = start.elapsed() / repeats;
        assert_eq!(
            canvas.surfaces.key,
            Some(first),
            "a frame over the same event must not re-sample the curves"
        );

        // And the moment the observer moves, the curves are taken again: they are a statement about
        // that observer's event and about nothing else. This is also the frame that costs, so it is
        // the one worth timing - the app spends it once per played frame and never more.
        let start = std::time::Instant::now();
        for _ in 0..repeats {
            play(&metric, &mut alice, &mut bob, 1, 0.05);
            pass_on(&mut canvas, &metric, Some(&alice), Some(&bob), ReferenceFrame::Alice);
        }
        let playing = start.elapsed() / repeats;
        assert_ne!(
            canvas.surfaces.key,
            Some(first),
            "the curves must follow the observer's event"
        );
        println!(
            "a rest-frame pass over {points} sampled points: {cold:?} on the first frame, with \
             both the curves and the as-seen solve cold; {paused:?} a frame with the run paused, \
             where neither is re-done; {playing:?} a frame while the run plays, which re-samples \
             the curves and warm-starts the solve"
        );
    }

    #[test]
    fn test_the_horizon_curve_meets_the_now_axis_where_the_horizon_box_says_it_does() {
        // The inconsistency this whole change was for. The box has always printed the ruler
        // distance to r+ - the arclength of the spacelike geodesic down the observer's own radial
        // axis - while the picture drew r+ at the first-order offset, and from the prograde ISCO of
        // an a = 0.90 hole those are 3.020 M and 1.647 M. One canvas, two answers.
        //
        // Now the drawn curve *is* that geodesic's endpoint swept over every direction, so where
        // the curve crosses the now-axis is the number in the box by construction. Checked on the
        // ISCO observer, where the gap used to be widest.
        let metric = KerrSchild::new(1.0, 0.90);
        let r_isco = metric.isco(true);
        let alice = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            r_isco,
            0.0,
            0.0,
            WorldlineParams::released(&metric, r_isco, 0.0, Release::CircularPrograde),
        );
        let u = alice.four_velocity(&metric);
        let ruler = ruler_distance(&metric, alice.r, &u, metric.outer_horizon())
            .expect("r+ is a place for an observer on the ISCO");
        let first_order = (alice.r - metric.outer_horizon())
            / (-metric.metric_components(alice.r)[1][1]).abs().sqrt().max(1e-12);

        // The window the app itself opens on, because this is the number the app itself draws.
        let p = pass(&metric, Some(&alice), None, ReferenceFrame::Alice, FRAME_MAX_R_DEFAULT);
        // psi = pi is the observer's own inward now-direction: xi^0 = 0, xi^1 < 0.
        let inward = curve_radius_at(&p.curve(Theme::HORIZON_OUTER), std::f64::consts::PI)
            .expect("the r+ curve crosses the inward now-axis on this canvas");
        println!(
            "from the prograde ISCO at r = {r_isco:.4} M the drawn r+ curve cuts the now-axis at \
             {inward:.4} M and the box prints {} ({ruler:.4} M); the first-order map drew about \
             {first_order:.3} M",
            ruler_distance_label(&metric, ruler)
        );
        // Half a screen point of tolerance: the curve is a polyline of exact points and the
        // crossing is interpolated across one of its chords.
        assert!(
            (inward - ruler).abs() < (1.0 / p.scale()).max(1e-3 * ruler),
            "the drawn curve says {inward} M and the box says {ruler} M"
        );
        assert!(
            (ruler - 3.0203).abs() < 2e-3,
            "and the number they agree on is the published one: {ruler}"
        );
        assert!(
            p.text.contains(&ruler_distance_label(&metric, ruler)),
            "the box must print that distance: {}",
            p.text
        );
    }

    #[test]
    fn test_a_rest_frame_carries_a_distance_scale_and_no_radial_track_under_it() {
        // The 1D radial track that used to sit under every spacetime chart is gone from the app
        // (the foliation chart's own labelled r axis and the equatorial view say everything it
        // said), and the height it took is the chart's. What a reader of this chart needs instead
        // is the scale of the axis it does draw, which is the observer's own xi^1 - proper
        // distance along the drawn spacelike geodesic - so the ruler says as much and is ticked
        // at the view's live scale. The foliation chart gets no ruler: its r axis is already one.
        let metric = KerrSchild::new(1.0, 0.90);
        let alice = hovering(&metric, "Alice", 8.0, 0.0);
        let bob = hovering(&metric, "Bob", 12.0, 0.4);
        let p = pass(&metric, Some(&alice), Some(&bob), ReferenceFrame::Alice, 16.0);

        assert!(
            !p.text.contains("RADIAL TRACK"),
            "no track under the chart: {}",
            p.text
        );
        assert!(
            p.text.lines().any(|line| line == "proper distance"),
            "the scale has to say what it measures: {}",
            p.text
        );
        // The ruler is ticked at a round step of the scale this very pass was drawn at, from
        // nought, in the units in force - which for these tests is M.
        let extent = f64::from(p.rect.width().max(p.rect.height()));
        let step = axis::round_step((extent * 0.7 / p.scale() / 8.0).max(1e-6));
        for k in 0..=1 {
            let label = metric.format_grid_m(step * k as f64, step);
            assert!(
                p.text.lines().any(|line| line == label),
                "the ruler must carry the tick {label}: {}",
                p.text
            );
        }
    }
}
