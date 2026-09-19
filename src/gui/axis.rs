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

/// How many lines the global foliation's time axis aims to put across its window. Seven is what the
/// old ladder was cut for, and with `round_step`'s tolerance either side of it the grid holds
/// between five and eleven lines at every zoom the wheel can reach.
const TIME_AXIS_LINES: f64 = 7.0;

/// The most tick indices the time axis will ever walk. The step is derived from the window, so the
/// honest count is eleven; this is the backstop for a window that arithmetic has already ruined -
/// a subnormal span, say, whose first index lands past what an f64 can tell apart from the last.
/// Without it such a window asks for a loop of 10^18 iterations, and the frame never ends.
const MAX_TIME_TICKS: i64 = 64;

/// The time grid of the global foliation: the step between two lines, in M, and the indices of the
/// lines that reach the window `t_min ..= t_max`. Line k sits at `k * step`.
///
/// Indices rather than values, and i64 rather than i32, because the chart's t is absolute
/// coordinate time and not an offset from the present: a run that has been playing for a while at a
/// window of 0.005 M is at index ten million, and one that has been left running is past two
/// billion, where an i32 index wraps and the grid folds back on itself.
///
/// A window that is empty, backwards or not a number draws no grid rather than guessing at one;
/// that is what the empty range means.
pub(crate) fn time_axis_ticks(t_min: f64, t_max: f64) -> (f64, std::ops::RangeInclusive<i64>) {
    let span = t_max - t_min;
    if !span.is_finite() || span <= 0.0 || !t_min.is_finite() {
        // `1 ..= 0` spelled out, which is the empty range: written as a literal it reads to clippy
        // as a range somebody meant to walk backwards.
        return (1.0, std::ops::RangeInclusive::new(1, 0));
    }
    let step = round_step(span / TIME_AXIS_LINES);
    // A float too large for an i64 saturates on the cast rather than wrapping, so the pair below is
    // ordered whatever the window was, and `MAX_TIME_TICKS` bounds the walk between them.
    let first = (t_min / step).floor() as i64;
    let last = ((t_max / step).ceil() as i64).min(first.saturating_add(MAX_TIME_TICKS));
    (step, first..=last)
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
        // The defect this module was written for, across the whole range the wheel can reach: the
        // grid has to hold a readable number of lines at every window between 0.005 M and 500 M,
        // and no two of those lines may carry the same text - in M, and in physical units at both
        // ends of the unit ladder. Sgr A* is the mass the app opens on, where one M is 20.4 s and
        // a grid at the tight end is ruled in tens of milliseconds around the six-minute mark; the
        // stellar-mass hole is the other end, where one M is 49 µs and the whole grid is in
        // microseconds.
        let sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        let stellar = KerrSchild::with_solar_mass(1.0, 0.9, 10.0);
        // The clock has been running: an absolute t, which is what the axis is ruled in.
        let now = 20.0;
        let mut window = 0.005;
        let mut windows = 0;
        while window <= 500.0 {
            // The split `SpacetimeCanvas::render` takes: most of the window is the past.
            let (t_min, t_max) = (now - window * 0.7, now + window * 0.3);
            let (step, ticks) = time_axis_ticks(t_min, t_max);
            let inside: Vec<f64> = ticks
                .map(|k| k as f64 * step)
                .filter(|t| *t >= t_min && *t <= t_max)
                .collect();
            assert!(
                (4..=12).contains(&inside.len()),
                "a {window} M window drew {} grid lines at a step of {step} M",
                inside.len()
            );
            for (metric, what) in [(&sgr, "Sgr A*"), (&stellar, "stellar")] {
                let labels: Vec<String> = inside
                    .iter()
                    .map(|t| time_label_physical(metric, *t, step))
                    .collect();
                let mut seen = labels.clone();
                seen.dedup();
                assert_eq!(
                    seen.len(),
                    labels.len(),
                    "{what} at a {window} M window repeats a label: {labels:?}"
                );
            }
            let labels: Vec<String> =
                inside.iter().map(|t| time_label_m(*t, step)).collect();
            let mut seen = labels.clone();
            seen.dedup();
            assert_eq!(seen.len(), labels.len(), "M labels repeat at {window} M: {labels:?}");
            windows += 1;
            window *= 1.2;
        }
        assert!(windows > 60, "only {windows} windows tried");

        // The label rules by example, at the coarse end and the fine, so that a change to either
        // shows up here as a diff rather than as a screenshot nobody took.
        assert_eq!(time_label_m(12.0, 2.0), "+12M");
        assert_eq!(time_label_m(0.0, 2.0), "+0M");
        assert_eq!(time_label_m(-12.5, 0.5), "-12.5M");
        assert_eq!(time_label_m(12.3456, 0.002), "+12.346M");
        assert_eq!(time_label_physical(&sgr, 14.0, 2.0), "4.77 min");
        assert_eq!(time_label_physical(&sgr, 20.0, 5e-4), "6.8157 min");
        assert_eq!(time_label_physical(&sgr, -14.0, 2.0), "-4.77 min");
        // A window that arithmetic has ruined draws nothing, and does not spin doing it.
        for (lo, hi) in [(1.0, 1.0), (2.0, 1.0), (f64::NAN, 1.0), (0.0, f64::INFINITY)] {
            let (_, ticks) = time_axis_ticks(lo, hi);
            assert!(ticks.count() <= MAX_TIME_TICKS as usize + 1, "{lo}..{hi} ran away");
        }
    }
}
