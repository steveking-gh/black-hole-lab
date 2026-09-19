//! The one rule every ruled axis in the app is ruled by: a round step near the spacing the zoom
//! asks for, and a label carrying exactly the digits that step needs and no more.
//!
//! A grid is a promise that the eye can read a number off the picture without counting pixels, and
//! that promise has two halves. The lines have to be somewhere round - a reader who sees 2, 4, 6
//! knows where 5 is, and a reader who sees 2.13, 4.26, 6.39 knows nothing - and there have to be
//! enough of them on the canvas to be a grid at all. Both halves come out of the same choice: the
//! caller says how far apart it would like the lines, and `round_step` moves that to the nearest
//! number a reader can do arithmetic with, never by more than a factor of 1.5.
//!
//! The rounding used to be written out four times over, once per axis, and the time axis of the
//! (t, r) diagram - the one axis that can be zoomed across five decades - had a fixed six-rung
//! ladder instead. The ladder's smallest rung was half an M, so below a window of about 3.5 M it
//! was wider than the canvas: the grid became one line that crossed the screen when the clock
//! happened to pass a multiple of half an M and nothing at all the rest of the time. The label had
//! the mirror-image fault, printing an integer number of M whatever the step, so a grid ruled every
//! half M read "+0M, +0M, +1M, +1M". Both faults are the same fault - a grid that does not know
//! what its own step is - and this module is where the step is known.
//!
//! A grid also has to be somewhere, and where the foliation chart's time grid is has the same two
//! halves. Lines at fixed coordinate times on a canvas that scrolls with the clock move down the
//! screen at (play rate)/(time window): at the tightest zoom the wheel allows that is seven screens
//! a second, and every label on them has to be an absolute reading deep enough to tell one fine
//! line from the next - "22.383 min" against "22.376 min". `TimeGrid` anchors the lines to the
//! present instead, at t = now + k * step, exactly as the rest-frame view's clock grid is anchored
//! to the observer's own event. They are the same surfaces t = const they always were; what changes
//! is that a playing run now moves the picture under a grid that stands still, and that each label
//! is a short offset - "now", "+0.02M", "-80 s" - instead of a long absolute time. The absolute
//! reading is not lost: the chart prints it once, at the head of the time axis, where a fast-moving
//! number is a clock rather than a blur.

use std::ops::RangeInclusive;

use crate::physics::kerr_schild::KerrSchild;

/// The roundest step near `target`: 1, 2, 5 or 10 times a power of ten.
///
/// The thresholds are 1.5, 3.5 and 7.5 on the mantissa, so the step chosen is never more than a
/// factor of 1.5 away from what the caller asked for in either direction, and a caller that divided
/// its window by n to get the target gets back between 2n/3 and 3n/2 lines across that window.
///
/// A target that is not positive and finite has no round step near it, and a caller is about to
/// divide its window by whatever comes back. One is the answer in that case, because it is the one
/// value that can neither divide by zero nor put a grid loop into a run it cannot finish.
pub(crate) fn round_step(target: f64) -> f64 {
    if !target.is_finite() || target <= 0.0 {
        return 1.0;
    }
    let power = 10.0_f64.powf(target.log10().floor());
    let mantissa = target / power;
    if mantissa < 1.5 {
        power
    } else if mantissa < 3.5 {
        2.0 * power
    } else if mantissa < 7.5 {
        5.0 * power
    } else {
        10.0 * power
    }
}

/// How many decimal places it takes to tell two readings one `step` apart from one another.
///
/// Rounding to d places puts a grid of 10^-d under the reading, so d is minus the step's own
/// decimal exponent: that grid is then at or below the step, and two readings a step apart cannot
/// land on the same one. A step of 0.002 needs three places, one of 0.5 needs one, and anything at
/// 1 or above needs none. Capped at `max`, since past some depth the digits are noise rather than
/// information and the label has to end somewhere.
fn decimals_for_step(step: f64, max: usize) -> usize {
    if !step.is_finite() || step <= 0.0 {
        return max.min(2);
    }
    // The epsilon is for the exact powers of ten, where log10 can land a hair under the integer it
    // should be and the floor drops a whole place: without it a step of 0.001 asks for four
    // decimals on some libms and three on others.
    let exponent = (step.log10() + 1e-9).floor();
    if exponent >= 0.0 {
        0
    } else {
        ((-exponent) as usize).min(max)
    }
}

/// The deepest a time label is allowed to go. Nine places of a unit is a part in a billion of it,
/// which is finer than any grid this app can be zoomed to needs and far finer than a reader reads.
const MAX_TIME_DECIMALS: usize = 9;

/// A time grid line labelled in M, carrying the digits its own step needs: "+12M" at a step of 1 M,
/// "+12.5M" at half an M, "+12.346M" at two thousandths.
///
/// The sign is explicit, as it has always been on this axis, because the window straddles the
/// present and the reader is being told which side of it a line is on. The line through zero reads
/// "+0M" and never "-0M": a value that rounds to nothing at this step is nothing, and is snapped to
/// a positive zero before the sign is printed.
pub(crate) fn time_label_m(t: f64, step: f64) -> String {
    let decimals = decimals_for_step(step, MAX_TIME_DECIMALS);
    let t = if t.abs() < step.abs() * 1e-9 { 0.0 } else { t };
    format!("{t:+.decimals$}M")
}

/// A time grid line labelled in physical units, carrying the digits its own step needs.
///
/// The unit ladder is `KerrSchild::format_physical_time`'s, down to the thresholds, so a grid line
/// is quoted in the same units as every other time in the app and against the same value - it is
/// only the number of decimals that is decided here instead of being fixed at two. Two is the
/// floor, so every label this can be asked for at a zoom that already worked comes out byte for
/// byte as it did; the places above that are the ones the step needs, and without them a grid ruled
/// every ten milliseconds around the six-minute mark printed "t = 6.82 min" on every one of its
/// lines.
///
/// It lives here and not beside `format_physical_time` because it is about an axis: the caller has
/// to know the step to ask the question at all, and every caller that knows a step is on a canvas.
pub(crate) fn time_label_physical(metric: &KerrSchild, t_in_m: f64, step_in_m: f64) -> String {
    let per_m = metric.t_grav_seconds() / metric.m.abs().max(1e-12);
    let secs = t_in_m.abs() * per_m;
    let step_secs = step_in_m.abs() * per_m;
    let sign = if t_in_m < 0.0 { "-" } else { "" };

    let (unit, name) = if secs >= 86400.0 * 365.25 {
        (86400.0 * 365.25, "yr")
    } else if secs >= 86400.0 {
        (86400.0, "days")
    } else if secs >= 3600.0 {
        (3600.0, "hrs")
    } else if secs >= 60.0 {
        (60.0, "min")
    } else if secs >= 1.0 {
        (1.0, "s")
    } else if secs >= 1e-3 {
        (1e-3, "ms")
    } else {
        (1e-6, "µs")
    };

    let decimals = decimals_for_step(step_secs / unit, MAX_TIME_DECIMALS).max(2);
    format!("{sign}{:.decimals$} {name}", secs / unit)
}

/// Seconds in a Julian year, the unit the top of the clock ladder is counted in.
pub(crate) const SECONDS_PER_YEAR: f64 = 86400.0 * 365.25;

/// The rungs of the clock ladder below a year, as (seconds in the unit, the multiples of it that
/// are used, the unit's name). Within each unit the 1-2-5 pattern is carried on up through the
/// decades until the next unit takes over, so the ladder has no holes in it: no two neighbouring
/// rungs are more than a factor of 2.5 apart, and the step actually chosen is therefore never more
/// than a factor of sqrt(2.5) from the one the caller asked for. (1, 2, 5 once per unit would leave
/// gaps of 200 between 5 µs and 1 ms and of 73 between 5 days and a year, and in those gaps a grid
/// asked for a line every 36 µs would get one every 1 ms - two hundred times too coarse, which on a
/// canvas a
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
///
/// It lives in this module because both time grids in the app are ruled by it: the rest-frame
/// view's distant clock grid climbs it as u^t runs away, and the foliation chart's time axis picks
/// the rung nearest the step its zoom asks for. One table, so the two pictures name their units the
/// same way.
pub(crate) const CLOCK_UNITS: [(f64, &[f64], &str); 9] = [
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

/// One rung of the clock ladder: a round amount of time, and the unit it is a round number of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClockRung {
    /// The rung itself, in seconds.
    pub seconds: f64,
    /// How many of its own units the rung is: 1, 2, 5, 10, 20, 50, ... - always a whole number, so
    /// a label counting in this unit needs no decimals.
    pub multiple: f64,
    /// Seconds in that unit.
    pub unit_seconds: f64,
    /// The unit's name, as it is printed.
    pub unit: &'static str,
}

/// The whole ladder, in ascending order. Years run on in the same 1-2-5 pattern decade after
/// decade, which is where the powers of ten come from: 1 yr, 2 yr, 5 yr, 10 yr, ..., 1e6 yr,
/// 2e6 yr, and so on.
pub(crate) fn clock_ladder() -> Vec<ClockRung> {
    let mut rungs = Vec::new();
    for (unit_seconds, multiples, unit) in CLOCK_UNITS {
        for &multiple in multiples {
            let seconds = multiple * unit_seconds;
            rungs.push(ClockRung { seconds, multiple, unit_seconds, unit });
        }
    }
    for decade in 0..=CLOCK_YEAR_DECADES {
        for mantissa in [1.0, 2.0, 5.0] {
            let years = mantissa * 10.0_f64.powi(decade);
            rungs.push(ClockRung {
                seconds: years * SECONDS_PER_YEAR,
                multiple: years,
                unit_seconds: SECONDS_PER_YEAR,
                unit: "yr",
            });
        }
    }
    rungs
}

/// The ladder's units alone, ascending, with the year on the end: what a label climbs when its own
/// rung's unit would take more digits than a reader wants to count.
fn clock_unit_scales() -> impl Iterator<Item = (f64, &'static str)> {
    CLOCK_UNITS
        .iter()
        .map(|(seconds, _, name)| (*seconds, *name))
        .chain(std::iter::once((SECONDS_PER_YEAR, "yr")))
}

/// The rung nearest `target_seconds`, measured as a ratio rather than as a difference, so that the
/// grid is as likely to be a little finer than what was asked for as a little coarser.
///
/// Taking the smallest rung at or above the target instead would be off by up to the widest gap in
/// the ladder, a factor of 2.5, in one direction only: a seven-line window would come back ruled by
/// under three lines, which is not a grid. Nearest in ratio is never worse than sqrt(2.5) = 1.58
/// either way, which is `round_step`'s own tolerance to within a hair, so both unit systems put
/// between four and twelve lines across the window.
fn nearest_clock_rung(target_seconds: f64) -> ClockRung {
    let ladder = clock_ladder();
    let distance =
        |rung: &ClockRung| (rung.seconds / target_seconds).max(target_seconds / rung.seconds);
    let mut best = ladder[0];
    for rung in &ladder {
        if distance(rung) < distance(&best) {
            best = *rung;
        }
    }
    best
}

/// How many lines the foliation chart's time axis aims to put across its window. Seven is what the
/// old fixed ladder was cut for, and with the tolerance either side of it - `round_step`'s 1.5, the
/// clock ladder's 1.58 - the grid holds between four and twelve lines at every zoom the wheel can
/// reach, in either unit system.
const TIME_AXIS_LINES: f64 = 7.0;

/// The most lines the time axis will ever walk. The step is derived from the window, so the honest
/// count is twelve; this is the backstop for a window that arithmetic has already ruined - a
/// subnormal span, say, whose first index lands past what an f64 can tell apart from the last.
/// Without it such a window asks for a loop of 10^18 iterations, and the frame never ends.
const MAX_TIME_TICKS: i64 = 64;

/// How far into the next unit a label is allowed to count before it climbs the ladder. Three digits
/// is what a reader takes in at a glance, and it is what keeps a grid ruled every 20 s reading
/// "+20 s, +40 s, +60 s, +80 s" instead of "+20 s, +40 s, +1.0 min, +1.3 min", where the decimal
/// hides the step the lines are actually spaced by.
const MAX_UNIT_COUNT: f64 = 1000.0;

/// Which units the foliation chart's time grid names its offsets in: the hole's own M, or the
/// physical seconds the units box switches every reading in the app to.
#[derive(Clone, Copy)]
pub(crate) enum TimeUnits<'a> {
    M,
    Physical(&'a KerrSchild),
}

/// What a label is counted in once the step is known. In M there is nothing to decide; in physical
/// units the step is a round number of some unit, and that unit is where its labels start.
enum Printed {
    M,
    Physical { seconds_per_m: f64, unit_seconds: f64 },
}

/// The time grid of the foliation chart: a ruler anchored to the present. Line k is the surface
/// t = now + k * step, and it carries the offset k * step as its label, with "now" on line zero.
///
/// Anchoring is the whole point. The lines are surfaces t = const either way - nothing physical
/// turns on which of them are drawn - but a grid at fixed coordinate times on a canvas that follows
/// the clock scrolls at (play rate)/(time window), which at the tight end of the wheel is several
/// screens a second, and its labels have to carry enough digits to separate absolute readings a
/// fine step apart. Anchored to now, the same lines stand still while the run plays, and each label
/// is an offset short enough to read at a glance. The user can still pan in time, and the anchor
/// stays at now while they do: panning slides the window over a ruler pinned to the present, so the
/// lines that fall inside the window are whichever k the window reaches, and the offsets the reader
/// sees grow as they travel away from now.
pub(crate) struct TimeGrid {
    now: f64,
    step_m: f64,
    ticks: RangeInclusive<i64>,
    printed: Printed,
}

impl TimeGrid {
    /// The grid for a window `t_min ..= t_max` of a chart whose clock reads `now`, in `units`.
    ///
    /// The step is the round one nearest a seventh of the window, rounded *in the unit its labels
    /// are printed in*: `round_step` of M, or the nearest rung of the clock ladder in physical
    /// units. Rounding in the printed unit is what keeps the offsets short - a step of 0.02 M is
    /// 0.409 s on Sgr A*, and a grid ruled by it would count "+0.41 s, +0.82 s, +1.23 s" where the
    /// ladder's own 0.5 s counts "+500 ms, +1 s, +1.5 s".
    ///
    /// A window that is empty, backwards or not a number draws no grid rather than guessing at one;
    /// that is what the empty range means.
    pub(crate) fn for_window(now: f64, t_min: f64, t_max: f64, units: TimeUnits<'_>) -> TimeGrid {
        // `1 ..= 0` spelled out, which is the empty range: written as a literal it reads to clippy
        // as a range somebody meant to walk backwards.
        let empty =
            |printed| TimeGrid { now, step_m: 1.0, ticks: RangeInclusive::new(1, 0), printed };
        let span = t_max - t_min;
        if !span.is_finite() || span <= 0.0 || !now.is_finite() || !t_min.is_finite() {
            return empty(Printed::M);
        }
        let target = span / TIME_AXIS_LINES;
        let (step_m, printed) = match units {
            TimeUnits::M => (round_step(target), Printed::M),
            TimeUnits::Physical(metric) => {
                let seconds_per_m = metric.t_grav_seconds() / metric.m.abs().max(1e-12);
                if !(seconds_per_m.is_finite() && seconds_per_m > 0.0) {
                    return empty(Printed::M);
                }
                let rung = nearest_clock_rung(target * seconds_per_m);
                (
                    rung.seconds / seconds_per_m,
                    Printed::Physical { seconds_per_m, unit_seconds: rung.unit_seconds },
                )
            }
        };
        if !step_m.is_finite() || step_m <= 0.0 {
            return empty(printed);
        }
        // The first and last lines that reach the window, counted from now rather than from the
        // origin of t. A float too large for an i64 saturates on the cast rather than wrapping, so
        // the pair is ordered whatever the window was, and `MAX_TIME_TICKS` bounds the walk between
        // them.
        let first = ((t_min - now) / step_m).ceil() as i64;
        let last = (((t_max - now) / step_m).floor() as i64)
            .min(first.saturating_add(MAX_TIME_TICKS));
        TimeGrid { now, step_m, ticks: first..=last, printed }
    }

    /// The lines that reach the window, as offsets from now in whole steps.
    pub(crate) fn ticks(&self) -> RangeInclusive<i64> {
        self.ticks.clone()
    }

    /// Where line k is, in the chart's own coordinate time. Line zero is the present exactly.
    pub(crate) fn time_of(&self, k: i64) -> f64 {
        self.now + (k as f64) * self.step_m
    }

    /// What line k is labelled: "now" on the line through the present, and otherwise the signed
    /// offset from it, carrying the digits its own step needs and no more.
    pub(crate) fn label(&self, k: i64) -> String {
        if k == 0 {
            return "now".to_string();
        }
        let offset = (k as f64) * self.step_m;
        match self.printed {
            Printed::M => time_label_m(offset, self.step_m),
            Printed::Physical { seconds_per_m, unit_seconds } => offset_label_physical(
                offset * seconds_per_m,
                self.step_m * seconds_per_m,
                unit_seconds,
            ),
        }
    }
}

/// An offset of `secs` seconds from now on a grid ruled every `step_secs`, whose step is a whole
/// number of the unit `unit_seconds` long.
///
/// The rule: count in the step's own unit, where the step is a whole number and the offsets
/// therefore need no decimals at all - "+80 s", "-250 ms" - and climb the ladder only when the
/// count reaches `MAX_UNIT_COUNT`, at which point it is the digits rather than the unit that are in
/// the way. A climb takes the decimals the step needs in the unit climbed to, so neighbouring lines
/// stay apart: a 500 µs step counting past a thousand of its own units reads "+2.5 ms", never
/// "+2 ms" twice. Panning far from now walks the count up through the ladder this way and nowhere
/// else, so a reader who has travelled hours into the past reads "-3.2 hr" rather than eleven
/// thousand seconds.
///
/// It is a second label rule beside `time_label_physical` because it answers a different question.
/// That one prints an absolute reading of the chart's clock, in the units the rest of the app
/// quotes that clock in and to at least two decimals, which is what the 2D+1 volume's rungs carry.
/// This one prints a difference, where the unit follows the step rather than the magnitude and a
/// whole number is the normal case.
fn offset_label_physical(secs: f64, step_secs: f64, unit_seconds: f64) -> String {
    let step_secs = step_secs.abs();
    let (unit_seconds, unit) = clock_unit_scales()
        .filter(|(scale, _)| *scale >= unit_seconds)
        .find(|(scale, _)| (secs / scale).abs() < MAX_UNIT_COUNT)
        // Off the top of the ladder: a year it is, and the number below goes to an exponent.
        .unwrap_or((SECONDS_PER_YEAR, "yr"));
    let value = secs / unit_seconds;
    let decimals = decimals_for_step(step_secs / unit_seconds, MAX_TIME_DECIMALS);
    let number = if value.abs() >= 1e4 {
        format!("{value:+.0e}")
    } else {
        format!("{value:+.decimals$}")
    };
    format!("{number} {unit}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_round_step_sits_on_one_two_or_five_at_every_decade() {
        // The thresholds themselves, walked across seven decades: the step must jump exactly at
        // 1.5, 3.5 and 7.5 times the power of ten, and land on that power times 1, 2, 5 or 10.
        for decade in -4i32..=3 {
            let p = 10.0_f64.powi(decade);
            for (target, want) in [
                (1.0 * p, 1.0 * p),
                (1.4999 * p, 1.0 * p),
                (1.5001 * p, 2.0 * p),
                (3.4999 * p, 2.0 * p),
                (3.5001 * p, 5.0 * p),
                (7.4999 * p, 5.0 * p),
                (7.5001 * p, 10.0 * p),
                (9.9 * p, 10.0 * p),
            ] {
                let got = round_step(target);
                assert!(
                    (got - want).abs() <= want * 1e-9,
                    "round_step({target:e}) = {got:e}, wanted {want:e}"
                );
            }
        }
        // The two ends the time axis actually reaches, spelled out: the tightest window the wheel
        // allows is 0.005 M and the widest 500 M, and seven lines across either one must come back
        // as a round step rather than as a rung of some ladder that ran out.
        assert_eq!(round_step(0.005 / 7.0), 5e-4);
        assert_eq!(round_step(500.0 / 7.0), 50.0);
        // The window the app opens on still rules itself every 2 M, as it always has.
        assert_eq!(round_step(14.0 / 7.0), 2.0);
        // Nothing a caller can hand it makes it hand back a step that cannot be divided by.
        for bad in [0.0, -1.0, -0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let got = round_step(bad);
            assert!(got > 0.0 && got.is_finite(), "round_step({bad}) = {got}");
        }
    }

    #[test]
    fn test_the_time_axis_rules_and_labels_itself_at_every_zoom() {
        // The two defects this grid was written for, across the whole range the wheel can reach.
        // The grid has to hold a readable number of lines at every window between 0.005 M and
        // 500 M, no two of those lines may carry the same text, and - the owner's "should not need
        // 5 digits", as an assertion - no label may run past ten characters while the view sits
        // where it opens. Checked in M and in physical units at both ends of the unit ladder:
        // Sgr A* is the mass the app opens on, where one M is 20.4 s, and the stellar-mass hole is
        // the other end, where one M is 49 µs. Checked at a clock just started and at one running
        // running a while, and with the window panned away from now as well as centred on it, since
        // the anchor is the clock and the labels are offsets from it.
        let sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        let stellar = KerrSchild::with_solar_mass(1.0, 0.9, 10.0);
        let mut windows = 0;
        for now in [20.0, 1234.567] {
            for pan in [0.0, 3.25] {
                let mut window = 0.005;
                while window <= 500.0 {
                    // The split `SpacetimeCanvas::render` takes: most of the window is the past,
                    // and `time_offset` slides the whole of it.
                    let offset = now + pan * window;
                    let (t_min, t_max) = (offset - window * 0.7, offset + window * 0.3);
                    for (units, what) in [
                        (TimeUnits::M, "M"),
                        (TimeUnits::Physical(&sgr), "Sgr A*"),
                        (TimeUnits::Physical(&stellar), "stellar"),
                    ] {
                        let grid = TimeGrid::for_window(now, t_min, t_max, units);
                        let ks: Vec<i64> = grid.ticks().collect();
                        assert!(
                            (4..=12).contains(&ks.len()),
                            "{what}: a {window} M window at t = {now} panned {pan} windows drew \
                             {} lines",
                            ks.len()
                        );
                        for k in &ks {
                            let t = grid.time_of(*k);
                            let slack = window * 1e-9;
                            assert!(
                                t >= t_min - slack && t <= t_max + slack,
                                "{what}: line {k} is off the window"
                            );
                        }
                        let labels: Vec<String> = ks.iter().map(|k| grid.label(*k)).collect();
                        let mut seen = labels.clone();
                        seen.sort();
                        seen.dedup();
                        assert_eq!(
                            seen.len(),
                            labels.len(),
                            "{what} at a {window} M window repeats a label: {labels:?}"
                        );
                        if pan == 0.0 {
                            // The present is in the window, is a line of the grid, and is that line
                            // exactly - not a rounding of it.
                            assert!(ks.contains(&0), "{what}: no line through now: {labels:?}");
                            assert_eq!(grid.time_of(0), now, "{what}: line zero is not now");
                            for label in &labels {
                                assert!(
                                    label.chars().count() <= 10,
                                    "{what} at a {window} M window: {label:?} is too long to read \
                                     at a glance ({labels:?})"
                                );
                            }
                        }
                        let landmark = [0.005, 0.1, 14.0, 500.0]
                            .iter()
                            .any(|w| (window - w).abs() < window * 0.1);
                        if pan == 0.0 && now == 20.0 && landmark {
                            println!("{what}, {window:.3} M window: {labels:?}");
                        }
                    }
                    windows += 1;
                    window *= 1.2;
                }
            }
        }
        assert!(windows >= 4 * 64, "only {windows} windows tried");

        // The label rules by example, so that a change to any of them shows up here as a diff
        // rather than as a screenshot nobody took. The absolute labels first, which the 2D+1
        // volume's rungs still carry.
        assert_eq!(time_label_m(12.0, 2.0), "+12M");
        assert_eq!(time_label_m(0.0, 2.0), "+0M");
        assert_eq!(time_label_m(-12.5, 0.5), "-12.5M");
        assert_eq!(time_label_m(12.3456, 0.002), "+12.346M");
        assert_eq!(time_label_physical(&sgr, 14.0, 2.0), "4.77 min");
        assert_eq!(time_label_physical(&sgr, 20.0, 5e-4), "6.8157 min");
        assert_eq!(time_label_physical(&sgr, -14.0, 2.0), "-4.77 min");
        // Then the offsets, and the unit boundary in particular: a grid ruled every 20 s counts in
        // seconds until the count reaches a thousand of them, and climbs only then.
        assert_eq!(offset_label_physical(80.0, 20.0, 1.0), "+80 s");
        assert_eq!(offset_label_physical(-980.0, 20.0, 1.0), "-980 s");
        assert_eq!(offset_label_physical(1000.0, 20.0, 1.0), "+16.7 min");
        assert_eq!(offset_label_physical(2.5e-3, 5e-4, 1e-6), "+2.5 ms");
        assert_eq!(offset_label_physical(-0.25, 0.05, 1e-3), "-250 ms");
        // A window that arithmetic has ruined draws nothing, and does not spin doing it.
        for (lo, hi) in [(1.0, 1.0), (2.0, 1.0), (f64::NAN, 1.0), (0.0, f64::INFINITY)] {
            let grid = TimeGrid::for_window(20.0, lo, hi, TimeUnits::Physical(&sgr));
            assert!(
                grid.ticks().count() <= MAX_TIME_TICKS as usize + 1,
                "{lo}..{hi} ran away"
            );
        }
    }

    #[test]
    fn test_the_time_grid_is_anchored_to_the_clock_and_not_to_the_canvas() {
        // The owner's complaint, as an assertion: with the run playing and the window following the
        // clock, the grid must not move on the screen. The clock advances by a deliberately
        // unround amount - a play rate times a frame time is never a round number of M - and every
        // line has to come back at the same offset from now, and therefore at the same fraction of
        // the way up the canvas, as it was on the frame before.
        let sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        let window = 0.14;
        let dt = 0.016_666_666_666_7 * 1.37;
        for units in [TimeUnits::M, TimeUnits::Physical(&sgr)] {
            let mut positions: Vec<Vec<(i64, f64)>> = Vec::new();
            for frame in 0..3 {
                let now = 20.0 + (frame as f64) * dt;
                let (t_min, t_max) = (now - window * 0.7, now + window * 0.3);
                let grid = TimeGrid::for_window(now, t_min, t_max, units);
                positions.push(
                    grid.ticks()
                        // Where up the canvas the line lands, which is what the eye watches.
                        .map(|k| (k, (grid.time_of(k) - t_min) / (t_max - t_min)))
                        .collect(),
                );
            }
            assert!(positions[0].len() >= 4, "too few lines to say anything: {positions:?}");
            assert_eq!(positions[0], positions[1], "the grid moved between frames");
            assert_eq!(positions[1], positions[2], "the grid moved between frames");
        }
    }
}
