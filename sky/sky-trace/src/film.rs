//! From an observer's event to a frame of the sky bundle: the pure part of tracing a film.
//!
//! Everything here is a function of its arguments. It reads no file and writes none; the program
//! in `main.rs` opens the save, calls these, and hands the results to `sky_format::BundleWriter`.
//! That split is what lets the tests run the whole of it on a grid of a few pixels.
//!
//! # The frames
//!
//! Frame k is at the observer's proper time tau_k = tau_0 + k * rate / fps, computed afresh from k
//! in exactly that order of operations for every frame, so that no rounding accumulates along a
//! film and a test can reproduce the number to the bit. `rate` is the proper time shown per second
//! of video, in M, and `fps` the video's frame rate, so one bundle frame is one video frame. The
//! whole worldline is walked first ([`walk`]): the walker is cheap and the tracer is not, and the
//! frame count and the reason the film ends are known before any tracing is spent.
//!
//! The manifest records tau_k as each frame's `proper_time`. For the fixed-radius motions that is
//! the event's own proper time exactly; for free fall the walker lands on it to within 1e-13 M
//! above (it re-integrates the last substep until it does), which is the event traced.
//!
//! # The camera (the owner's decisions, 2026-09-27)
//!
//! **Tied to the hole, facing it.** The triad is rebuilt at every frame from the hole's directions
//! at the observer's event, and is not carried by gyroscopes; so the hole keeps its place in the
//! frame and the sky moves round it. Heading zero - the triad's x leg and the centre of the video
//! frame - is the direction toward the hole in the observer's rest space, and z is the spin axis.
//!
//! "Toward the hole" is taken as minus the gradient of the radius r, projected into the observer's
//! rest space: the direction in which, of all the directions the observer can point, r falls
//! fastest. It is the one choice that does not depend on the chart. r is a scalar, so its gradient
//! nabla r = g^{mu r} d_mu is the same vector in Boyer-Lindquist and Kerr-Schild coordinates; the
//! 1-form dr annihilates d_t and d_phi, so nabla r is orthogonal to both Killing directions, and
//! for any observer at rest in r (static, ZAMO, circular orbit) it already lies in the rest space
//! and is the radial leg of the Boyer-Lindquist frame. Its rest-space part has the squared length
//!
//!     g(nabla r, nabla r) + g(nabla r, u)^2 = g^{rr} + (u^r)^2,
//!
//! which vanishes only where u is parallel to nabla r - possible only between the horizons, for an
//! observer with E = L = 0 - and there "toward the hole" has no direction; [`toward_hole`] refuses
//! it with a sentence.
//!
//! The alternative the library offers, `kerr_sky::Triad::new(.., pi)`, faces along minus the
//! rest-space part of the chart's coordinate vector d/dr. That vector is taken at fixed ingoing
//! Kerr-Schild t and phi, and d/dr|_KS = d/dr|_BL - (2Mr / Delta) d/dt - (a / Delta) d/dphi (from
//! dt_KS = dt_BL + (2Mr / Delta) dr and dphi_KS = dphi_BL + (a / Delta) dr), so its rest-space part
//! is turned away from the hole by the chart's slicing: 10.4 degrees for a static observer at
//! r = 6 M around a = 0.9 (there g(d_r, w) = -a for w = (u_phi, 0, -u_t), the static observer's
//! azimuthal direction, against g(d_r, nabla r) = 1), and 6.4 degrees for a circular orbit at
//! r = 8 M around a = 0 (from the d/dt term alone, which a moving observer sees a part of); for a
//! static observer around a = 0 the two agree. A test
//! (`test_the_chart_radial_leg_would_turn_the_camera_away_from_the_hole`) measures all three. A
//! camera yawed by the choice of time slicing is not facing the hole, so it is not used.
//!
//! What "tied to the hole" does not do is put the shadow at the centre of the frame for every
//! observer. An observer moving across the radial direction sees the sky aberrated toward the
//! motion, the shadow with it: an observer on a circular orbit at 8 M around a = 0 sees the shadow
//! span 11.6 degrees left to 52.1 degrees right of heading zero, its middle 20.3 degrees toward the
//! direction of the orbit. It stays there frame after frame, which is what the tie promises, and
//! `test_on_a_circular_orbit_the_shadow_holds_still_and_the_sky_turns` holds the offset to the
//! closed form of aberration. A radial fall has no such offset; Bob's, in the demonstration save,
//! is radial but for the frame dragging.
//!
//! # The read-outs and the marks
//!
//! Every film declares three clock read-outs - the stopwatch, the observer's watch, the coordinate
//! time - stored in M, each with a display in seconds and their multiples unless
//! `--units geometric` asks for none (`units`). The chart's radius is not among them: it is a
//! coordinate, not a length anyone measures, and the frame's `position` carries it for whoever
//! wants it. After the clocks comes the frame's place relative to the outer horizon (`horizon`):
//! on a frame outside r+ the proper distance from it, a ruler reading in kilometres and their
//! multiples; on a frame between the horizons the proper time since the observer's worldline
//! crossed it, a clock reading displayed as the watch is. A frame carries the one for its side,
//! and a film that falls through r+ declares both. Then, for each local
//! reference observer that the
//! film passes - the static observer, the ZAMO, the raindrop - the speed past it (or the Lorentz
//! factor above 0.9999 c), the heading of travel past it, and a mark on the sky where the observer
//! sees itself going (`travel`). All of it is decided by [`plan`] from the walk of the whole
//! worldline before any ray is traced, because a declaration is one for the whole film: its unit
//! is chosen with every frame in view, and it is declared only if some frame needs it.
//!
//! # The spin's sense
//!
//! The far-sky frame's Z is along the hole's angular momentum (specification 4.4) and `kerr_sky`
//! traces in the chart's +z, which agree for a >= 0. A hole with a < 0 is refused by
//! [`check_spin`]; its comment says what lifting the refusal takes.

use std::f64::consts::FRAC_PI_2;

use kerr_equatorial::KerrSchild;
use kerr_sky::{Kerr, Observer, TraceOptions, Triad, trace_frame_reporting};
use sky_format::{
    Display, FORMAT, FarSky, Frame, FrameEntry, Geometry, GridSpec, Manifest, MarkDecl, Num,
    Playback, Position, ReadoutDecl, STOPWATCH, Source, TimeUnit, VERSION, WriterInfo, fate,
};

use crate::bhl::{self, Mode, Release, SavedObserver, TrailPoint};
use crate::horizon;
use crate::travel::{self, Passing};
use crate::units::{self, Units};
use crate::worldline::{End, Event, Film, Worldline};

/// The program's name, as the manifest's `writer.program` records it.
pub const PROGRAM: &str = "sky-trace";

/// The id of the watch read-out: the observer's proper time as the app counts it.
pub const WATCH: &str = "watch";

/// The id of the coordinate-time read-out: the chart's time t, which the app's distant clock shows.
pub const COORDINATE_TIME: &str = "coordinate_time";

/// The id of the read-out of the proper distance from the outer horizon, on frames outside it.
pub const HORIZON_DISTANCE: &str = "horizon_distance";

/// The id of the read-out of the proper time since crossing the outer horizon, on frames between
/// the horizons.
pub const HORIZON_TIME: &str = "horizon_time";

/// The mass of Sagittarius A*, in solar masses: the default of `--solar-masses` for a hover test,
/// and the mass of the repository's demonstration save.
pub const SGR_A_SOLAR_MASSES: f64 = 4.15e6;

/// The proper time of frame k: tau_0 + k * rate / fps, in that order of operations.
pub fn frame_tau(tau0: f64, k: usize, rate: f64, fps: f64) -> f64 {
    tau0 + k as f64 * rate / fps
}

/// Frames 0 to `limit - 1` of the worldline, frame k at [`frame_tau`], or fewer where the
/// worldline ends first.
///
/// The same walk as `Worldline::walk`, with the frame's proper time formed as rate and fps give
/// it rather than from a precomputed step: `k * (rate / fps)` and `k * rate / fps` round
/// differently, and the manifest states the second.
pub fn walk(worldline: &mut Worldline, rate: f64, fps: f64, limit: usize) -> Film {
    let tau0 = worldline.tau0();
    let mut events = Vec::with_capacity(limit.min(1 << 16));
    for k in 0..limit {
        match worldline.event_at(frame_tau(tau0, k, rate, fps)) {
            Ok(event) => events.push(event),
            Err(end) => return Film { events, end },
        }
    }
    Film {
        events,
        end: End::Frames { count: limit },
    }
}

/// Refuses a hole whose spin is not along the chart's +z.
///
/// Lifting the refusal needs no new physics, only a mirror. The proper rotation by pi about the
/// chart's x axis, (x, y, z) -> (x, -y, -z), takes the Kerr-Schild form of spin a to that of spin
/// -a (with y -> -y, z -> -z and a -> -a, l = (1, (r x + a y) / (r^2 + a^2), (r y - a x) /
/// (r^2 + a^2), z / r) goes over into itself with l_y and l_z negated, and r is unchanged), and
/// the embedding x + iy = (r + ia) e^{i phi} to its complex conjugate, (r - ia) e^{-i phi}. So a
/// hole of spin a < 0 with the observer at (r, phi) and u = (u^t, u^r, u^phi) is, in the rotated
/// chart, the hole of spin |a| with the observer at (r, -phi) and u = (u^t, u^r, -u^phi). Walk the
/// observer in the save's own chart as now, map each event so, trace it, and d, g and the winding
/// come out in the rotated chart's axes - X at azimuth zero, Z along the angular momentum, winding
/// positive in the prograde sense - which are exactly the far-sky frame of section 4.4. The triad
/// needs nothing: its z is the rotated chart's +z, "up" along the angular momentum, and x still
/// faces the hole. What must change is the manifest's `position`, to be written in the rotated
/// chart (phi negated) or said to be in the save's, and the triad sentence, which should say which.
/// Until that is done and tested, a < 0 is refused rather than filmed with the far sky mirrored.
pub fn check_spin(a: f64) -> Result<(), String> {
    if a < 0.0 {
        return Err(format!(
            "the hole in this save spins the other way (a = {a} M, its angular momentum along -z), \
             and this version films only a hole whose spin is along +z"
        ));
    }
    Ok(())
}

/// The static observer of a hover test: at chart radius `r` on the equatorial plane at azimuth 0,
/// at t = 0 and tau = 0, held there for good. Refused where no static observer exists: at or
/// inside the static limit, where g_tt >= 0 and d/dt is not timelike, so that no observer can keep
/// r and phi fixed.
pub fn hover_observer(metric: &KerrSchild, r: f64) -> Result<SavedObserver, String> {
    let g_tt = metric.metric_components(r)[0][0];
    if !(r.is_finite() && r > 0.0 && g_tt < 0.0) {
        return Err(format!(
            "no static observer exists at r = {r} M: on the equatorial plane nothing can hold a \
             fixed r and phi at or inside the static limit r = {} M, so give --hover a radius \
             greater than that",
            2.0 * metric.m
        ));
    }
    let here = TrailPoint {
        t: 0.0,
        r,
        phi: 0.0,
        tau: 0.0,
        u: [1.0 / (-g_tt).sqrt(), 0.0, 0.0],
        stalled: false,
    };
    Ok(SavedObserver {
        name: "The static observer".into(),
        mode: Mode::Static,
        t: 0.0,
        r,
        phi: 0.0,
        tau: 0.0,
        geodesic: None,
        trail: vec![here],
        start: here,
        release_t: 0.0,
        release: Release::AtRest,
        is_active: true,
    })
}

/// The observer at an event of the film, checked by `kerr_sky`: finite, outside r-, and moving
/// on a unit future-directed timelike 4-velocity.
pub fn observer(kerr: &Kerr, event: &Event) -> Result<Observer, String> {
    Observer::new(kerr, event.r, event.phi, event.u)
        .map_err(|e| format!("at tau = {} M: {e}", event.tau))
}

/// Minus the gradient of r, in chart components (t, r, phi): -g^{mu r}. Its rest-space part is the
/// direction toward the hole (module documentation). Refused where it has none.
pub fn toward_hole(metric: &KerrSchild, event: &Event) -> Result<[f64; 3], String> {
    let g_inv = metric.inverse_metric(event.r);
    // |P nabla r|^2 = g^{rr} + (u^r)^2, a sum that is a difference only between the horizons,
    // where g^{rr} < 0; compared against the size of its two terms.
    let dr_dtau = event.u[1];
    let rest = g_inv[1][1] + dr_dtau * dr_dtau;
    let scale = g_inv[1][1].abs() + dr_dtau * dr_dtau;
    // The negation is the point rather than a way of writing <=: a NaN has to be refused too.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(rest > 1e-12 * scale) {
        return Err(format!(
            "at tau = {} M, r = {} M, the observer moves along the gradient of r itself, so that \
             in the observer's rest space no direction points toward the hole more than another \
             and the camera has no heading",
            event.tau, event.r
        ));
    }
    Ok([-g_inv[0][1], -g_inv[1][1], -g_inv[2][1]])
}

/// The triad at an event: z along the spin axis, x toward the hole (module documentation).
pub fn triad(kerr: &Kerr, event: &Event) -> Result<Triad, String> {
    let obs = observer(kerr, event)?;
    let s = toward_hole(&kerr.equatorial(), event)?;
    Ok(Triad::towards(kerr, &obs, s))
}

/// One traced frame, ready for the bundle, and what it cost.
#[derive(Debug, Clone)]
pub struct Traced {
    pub frame: Frame,
    /// Rays of fate 0 in the frame.
    pub unresolved: usize,
    /// Rays integrated or decided (a little over half the grid, by the mirror symmetry).
    pub rays_traced: usize,
    /// Accepted integration steps over those rays.
    pub steps: u64,
}

/// Traces frame `index` of a W x H grid at an event on `threads` threads, telling `rows_done` after
/// each row how many of the frame's rows are traced and how many there are
/// (`kerr_sky::trace_frame_reporting`).
///
/// `kerr_sky` hands back the planes already in the format's types - f32 directions and shifts,
/// the winding as i16 clamped to [-32767, 32767] - with NaN and 0 wherever the fate is not 1.
/// That rule is the specification's (4.7, 4.8) and binds the writer, so it is checked here on
/// every frame before the frame can reach a file; a frame that breaks it is a bug in this program
/// or in the tracer, never data, and stops the run with a panic that says so.
pub fn trace(
    kerr: &Kerr,
    event: &Event,
    index: u32,
    width: u32,
    height: u32,
    threads: usize,
    rows_done: &(dyn Fn(usize, usize) + Sync),
) -> Result<Traced, String> {
    let triad = triad(kerr, event)?;
    let sky = trace_frame_reporting(
        kerr,
        &triad,
        width as usize,
        height as usize,
        &TraceOptions::default(),
        threads,
        true,
        rows_done,
    );
    let frame = Frame {
        width,
        height,
        index,
        fate: sky.fate,
        direction: sky.direction,
        shift: sky.shift,
        winding: sky.winding,
        points: None,
    };
    if let Err(why) = check_rays(&frame) {
        panic!(
            "sky-trace has a bug: frame {index} (tau = {} M, r = {} M) breaks the \
             specification's rule for rays: {why}",
            event.tau, event.r
        );
    }
    Ok(Traced {
        frame,
        unresolved: sky.unresolved,
        rays_traced: sky.rays_traced,
        steps: sky.steps,
    })
}

/// The specification's rule for a writer: every ray not of fate 1 has NaN in all three components
/// of d and in g, and 0 in the winding; and no ray has a fate that version 1 does not define.
pub fn check_rays(frame: &Frame) -> Result<(), String> {
    frame.check().map_err(|e| e.to_string())?;
    for (k, &f) in frame.fate.iter().enumerate() {
        let (i, j) = (k % frame.width as usize, k / frame.width as usize);
        if f > fate::DARK {
            return Err(format!(
                "pixel ({i}, {j}) has fate {f}, which version 1 reserves"
            ));
        }
        if f == fate::FAR_SKY {
            continue;
        }
        let d = [0, 1, 2].map(|c| frame.direction[c][k]);
        if !(d.iter().all(|v| v.is_nan()) && frame.shift[k].is_nan() && frame.winding[k] == 0) {
            return Err(format!(
                "pixel ({i}, {j}) has fate {f} and carries d = {d:?}, g = {}, winding {}, where \
                 NaN, NaN and 0 are required",
                frame.shift[k], frame.winding[k]
            ));
        }
    }
    Ok(())
}

/// How each of the three clock read-outs is displayed: stopwatch, watch and coordinate time, in
/// the order drawn. `None` shows the value in M, as stored.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Displays {
    pub stopwatch: Option<Display>,
    pub watch: Option<Display>,
    pub coordinate_time: Option<Display>,
}

/// The displays of a film's read-outs (specification 6.1, and the module `units`).
///
/// Each read-out's unit is chosen once, from the largest magnitude it takes over the film's
/// frames: the stopwatch's |tau_k - tau_0|, the watch's |tau_k| and the coordinate time's |t|,
/// with `taus[k]` the proper time of frame k ([`frame_tau`]) and `events[k]` its event.
/// `Units::Geometric` gives no displays at all.
pub fn displays(units: Units, seconds_per_m: f64, taus: &[f64], events: &[Event]) -> Displays {
    if units == Units::Geometric {
        return Displays::default();
    }
    let largest =
        |values: &mut dyn Iterator<Item = f64>| values.fold(0.0f64, |m, v| m.max(v.abs()));
    let tau0 = taus.first().copied().unwrap_or(0.0);
    Displays {
        stopwatch: Some(units::time_display(
            largest(&mut taus.iter().map(|&tau| tau - tau0)),
            seconds_per_m,
        )),
        watch: Some(units::time_display(
            largest(&mut taus.iter().copied()),
            seconds_per_m,
        )),
        coordinate_time: Some(units::time_display(
            largest(&mut events.iter().map(|e| e.t)),
            seconds_per_m,
        )),
    }
}

/// The three clock read-outs every film of this program declares, in the order drawn, each with
/// its display; `who` is the observer's name, which labels the watch ("Bob's Watch").
///
/// Two of them read the observer's own clock, and they are not the same thing. The stopwatch is
/// the format's: proper time since the film's first frame, zero where the film starts. The watch
/// is the app's: the proper time the app shows for this observer, counted from wherever the app
/// started counting, so that a number read off the film can be found again in the app. They differ
/// by a constant along a film. A view of one moment has no elapsed time to show, which is why a
/// renderer leaves the stopwatch out of a still; the watch is what says which moment it is.
///
/// The coordinate time is the chart's t at the frame's event, the same number the app's distant
/// clock shows, and not the time since the first frame. The chart is ingoing Kerr-Schild, whose t
/// is a Killing time: along any static worldline a difference of t is the proper time a clock at
/// rest at infinity records, which is why the app calls it the distant clock. It is not
/// Boyer-Lindquist t, which differs from it by a function of r alone and agrees only far from the
/// hole.
pub fn readouts(displays: &Displays, who: &str) -> Vec<ReadoutDecl> {
    let declare = |id: &str, label: &str, decimals: u32, display: &Option<Display>| ReadoutDecl {
        id: id.into(),
        label: label.into(),
        unit: "M".into(),
        decimals,
        display: display.clone(),
    };
    vec![
        declare(STOPWATCH, "Stopwatch", 3, &displays.stopwatch),
        declare(WATCH, &format!("{who}'s Watch"), 3, &displays.watch),
        declare(
            COORDINATE_TIME,
            "Coordinate time",
            2,
            &displays.coordinate_time,
        ),
    ]
}

/// What the walk decides before any ray is traced: every read-out and mark the manifest declares,
/// and each frame's travel past the reference observers that exist at its event.
///
/// The declarations are written into the manifest when the bundle is created, from the walk of the
/// whole worldline, so a resumed run - which walks the same worldline to the bit - makes the same
/// ones, and `trace.rs` refuses to resume a bundle whose declarations differ.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub readouts: Vec<ReadoutDecl>,
    pub marks: Vec<MarkDecl>,
    /// Frame k's passings, outermost reference observer first (`travel::passing`).
    pub passing: Vec<Vec<Passing>>,
    /// The heading each of frame k's passings is written with (`travel::written_headings`):
    /// continuous along the film.
    pub headings: Vec<Vec<Option<f64>>>,
    /// Frame k's place relative to the outer horizon ([`Horizon`]).
    pub horizon: Vec<Horizon>,
}

/// Where a frame's event stands relative to the outer horizon r+, as its read-out gives it: a
/// distance outside, a time inside (the module `horizon`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Horizon {
    /// Outside r+: the proper distance from r+, in M; infinite for an extremal hole.
    Distance(f64),
    /// At or inside r+: the proper time since the observer's worldline crossed r+, in M; NaN where
    /// the crossing could not be found, which the renderer shows as a value it does not know
    /// rather than a guess.
    Time(f64),
}

impl Horizon {
    /// Frame `event`'s, around `metric`. The crossing of r+ is the same event for every frame of a
    /// worldline's fall, so `crossing` keeps the first one found and the frames after it are not
    /// integrated back again.
    fn of(metric: &KerrSchild, event: &Event, crossing: &mut Option<Option<f64>>) -> Horizon {
        match horizon::proper_distance_from_outer_horizon(metric, event.r) {
            Some(d) => Horizon::Distance(d),
            None => {
                let tau = *crossing.get_or_insert_with(|| horizon::crossing_tau(metric, event));
                Horizon::Time(tau.map_or(f64::NAN, |tau| event.tau - tau))
            }
        }
    }

    /// The read-out's id and value.
    fn readout(self) -> (&'static str, f64) {
        match self {
            Horizon::Distance(d) => (HORIZON_DISTANCE, d),
            Horizon::Time(t) => (HORIZON_TIME, t),
        }
    }
}

/// The declarations of the horizon read-outs a film's frames carry, each only if some frame
/// carries it: the distance on frames outside r+, the time on frames inside, so a film that falls
/// through r+ declares both and shows each on its own side. The distance is a ruler reading and is
/// displayed as the radius is, in kilometres and their multiples, to three decimals of M; the time
/// is a clock reading and is displayed as the watch is. Each unit is chosen from the largest value
/// the read-out takes over the film, leaving out an infinite distance, which no unit shows better
/// than another.
fn horizon_readouts(horizon: &[Horizon], units: Units, seconds_per_m: f64) -> Vec<ReadoutDecl> {
    let largest = |pick: fn(&Horizon) -> Option<f64>| -> Option<f64> {
        let mut values = horizon.iter().filter_map(pick).peekable();
        values.peek()?;
        Some(
            values
                .filter(|v| v.is_finite())
                .fold(0.0f64, |m, v| m.max(v.abs())),
        )
    };
    let physical = units == Units::Physical;
    let mut out = Vec::new();
    if let Some(d) = largest(|h| match h {
        Horizon::Distance(d) => Some(*d),
        Horizon::Time(_) => None,
    }) {
        out.push(ReadoutDecl {
            id: HORIZON_DISTANCE.into(),
            label: "Proper distance from the outer event horizon".into(),
            unit: "M".into(),
            decimals: 3,
            display: physical.then(|| units::length_display(d, seconds_per_m)),
        });
    }
    if let Some(t) = largest(|h| match h {
        Horizon::Time(t) => Some(*t),
        Horizon::Distance(_) => None,
    }) {
        out.push(ReadoutDecl {
            id: HORIZON_TIME.into(),
            label: "Proper time from outer event horizon".into(),
            unit: "M".into(),
            decimals: 3,
            display: physical.then(|| units::time_display(t, seconds_per_m)),
        });
    }
    out
}

impl Plan {
    /// Whether the film is a still: its headings are then magnitudes (`travel::declarations`).
    pub fn single(&self) -> bool {
        self.passing.len() == 1
    }
}

/// The plan of a film whose frame k is at proper time `taus[k]` and event `events[k]`, for a hole
/// whose M lasts `seconds_per_m` seconds.
///
/// It sets up the camera at every event, which is cheap, so that an event it cannot be set up at
/// is refused now, with the frame's number, and not hours into the tracing.
pub fn plan(
    kerr: &Kerr,
    who: &str,
    events: &[Event],
    taus: &[f64],
    seconds_per_m: f64,
    units: Units,
) -> Result<Plan, String> {
    let passing = events
        .iter()
        .enumerate()
        .map(|(k, event)| {
            let triad =
                triad(kerr, event).map_err(|why| format!("frame {k} cannot be filmed: {why}"))?;
            Ok(travel::passing(kerr, event, &triad))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let (travel_readouts, marks) = travel::declarations(&passing);
    let headings = travel::written_headings(&passing);
    let metric = kerr.equatorial();
    let mut crossing = None;
    let horizon: Vec<Horizon> = events
        .iter()
        .map(|event| Horizon::of(&metric, event, &mut crossing))
        .collect();
    // The clocks, then the place relative to the outer horizon, then the travel.
    let mut readouts = readouts(&displays(units, seconds_per_m, taus, events), who);
    readouts.extend(horizon_readouts(&horizon, units, seconds_per_m));
    readouts.extend(travel_readouts);
    Ok(Plan {
        readouts,
        marks,
        passing,
        headings,
        horizon,
    })
}

/// The manifest entry of frame `index` at `event`, whose proper time is `tau` (the frame's
/// [`frame_tau`]), in a film whose frame 0 is at proper time `tau0` and whose plan is `plan`.
///
/// The stopwatch is `tau - tau0`, which is the specification's definition to the bit (6: a frame's
/// stopwatch equals its proper time less frame 0's). The watch is `tau` itself and the coordinate
/// time the chart's t, both as the app counts them (see [`readouts`]). The position is the event
/// in ingoing Kerr-Schild (t, r, theta, phi) with theta = pi/2 on the plane, which is where the
/// chart's r is kept: it is not a read-out. Then the frame's distance from the outer horizon or its
/// time since crossing it, whichever its side of r+ has ([`Horizon`]), and the speed, or the
/// Lorentz factor, past each reference observer that exists at the event, and the heading and mark
/// of travel past each wherever there is a direction (`travel::values`).
pub fn entry(index: u32, tau: f64, tau0: f64, event: &Event, plan: &Plan) -> FrameEntry {
    let mut entry = FrameEntry {
        position: Some(Position {
            chart: "kerr-schild".into(),
            coords: [Num(event.t), Num(event.r), Num(FRAC_PI_2), Num(event.phi)],
        }),
        ..FrameEntry::new(index, tau)
            .with_readout(STOPWATCH, tau - tau0)
            .with_readout(WATCH, tau)
            .with_readout(COORDINATE_TIME, event.t)
    };
    let k = index as usize;
    let (id, value) = plan.horizon[k].readout();
    entry = entry.with_readout(id, value);
    let (readouts, marks) = travel::values(&plan.passing[k], &plan.headings[k]);
    for (id, value) in readouts {
        entry = entry.with_readout(&id, value);
    }
    for (id, n) in marks {
        entry = entry.with_mark(&id, n);
    }
    entry
}

/// What a film's manifest says about the run.
#[derive(Debug, Clone, PartialEq)]
pub struct Setup {
    pub source: Source,
    /// The observer's name, for `observer.name`.
    pub observer: String,
    /// The hole: its spin (a >= 0) and its mass in solar masses. M is 1.
    pub spin: f64,
    pub solar_masses: f64,
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    /// Proper time per second of video, in M.
    pub rate: f64,
    /// Proper time and radius of frame 0.
    pub tau0: f64,
    pub r0: f64,
    /// Frames the film has.
    pub frames: u32,
    /// The read-outs and marks the manifest declares: a [`Plan`]'s.
    pub readouts: Vec<ReadoutDecl>,
    pub marks: Vec<MarkDecl>,
}

/// The sentence for `observer.triad`: how the triad is built and carried (specification 4.2),
/// and the run's parameters that the manifest has no other place for, so that a resume with
/// other settings is caught.
pub fn triad_sentence(setup: &Setup) -> String {
    format!(
        "Rebuilt at every frame from the hole's directions at the observer's event, not carried \
         by gyroscopes, and built in the observer's own rest space with no reference observer and \
         no boost: z is d/dz of the Cartesian ingoing Kerr-Schild chart, the unit vector along \
         the spin axis in the sense of the hole's angular momentum, which on the equatorial plane \
         is orthogonal to the observer's 4-velocity; x, heading zero, faces the hole, along minus \
         the gradient of r projected into the rest space (the direction in which r falls fastest, \
         whatever the chart); and y = z cross x completes the triad, the rest-space direction \
         along the surface r = const on the retrograde side, the viewer's left. Frame k is at the \
         observer's proper time {tau0} + k * {rate} / {fps} M, from r = {r0} M.",
        tau0 = setup.tau0,
        rate = setup.rate,
        fps = setup.fps,
        r0 = setup.r0,
    )
}

/// The manifest of a film, with no frames listed yet.
pub fn manifest(setup: &Setup) -> Manifest {
    Manifest {
        format: FORMAT.into(),
        version: VERSION,
        writer: WriterInfo {
            program: PROGRAM.into(),
            version: env!("CARGO_PKG_VERSION").into(),
            git: None,
        },
        source: setup.source.clone(),
        geometry: Geometry {
            kind: "kerr".into(),
            mass: Some(Num(1.0)),
            spin: Some(Num(setup.spin)),
        },
        time_unit: TimeUnit {
            name: "M".into(),
            seconds: Num(setup.solar_masses * bhl::GM_SUN_OVER_C3_SECONDS),
        },
        observer: Some(sky_format::Observer {
            name: Some(setup.observer.clone()),
            triad: Some(triad_sentence(setup)),
        }),
        grid: GridSpec::new(setup.width, setup.height),
        far_sky: FarSky::galactic(),
        playback: Playback {
            frames_per_second: Num(setup.fps),
            proper_time_per_video_second: Num(setup.rate),
        },
        readouts: setup.readouts.clone(),
        labels: Vec::new(),
        marks: setup.marks.clone(),
        frames_planned: Some(setup.frames),
        frames: Vec::new(),
    }
}

#[cfg(test)]
mod tests;
