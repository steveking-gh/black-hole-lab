//! The read-outs' values: which number each line of a panel shows at a video frame, and how it is
//! written.
//!
//! The manifest declares each read-out once (label, unit, decimals) and gives its value at each
//! bundle frame (the specification, section 6). A video frame falls on a bundle frame or between
//! two, as `crate::timeline` decides for the rays, and a read-out between two bundle frames is
//! interpolated linearly by the same weight the rays are. Two exceptions:
//!
//! - The stopwatch is not interpolated. It is the video frame's own stopwatch time, which the
//!   timeline knows exactly, and it is what decides which bundle frames are shown in the first
//!   place: interpolating the bundle's stopwatch values would give the same number less exactly.
//! - Where either bracketing value is missing or not finite there is nothing to interpolate
//!   between, and the nearer frame's value is shown as it is: a NaN, an infinity, or nothing.
//!
//! Numbers are written by the app's own `readout` crate, so that `--decimal-comma` gives the
//! same marks the app's "Decimal is comma" setting does.
//!
//! # Display units
//!
//! A declaration may carry a `display` (the specification, section 6.1): a unit a person would
//! rather read, and how many of it one stored unit is. With `--units display`, the default, such a
//! line is shown as its value times `display.scale`, followed by `display.unit`, to
//! `display.decimals` places; with `--units stored`, or on a line without a `display`, as stored.
//! The frames' values never change: a [`Line`] carries the scale it is shown at, and every value
//! this module hands out, the stopwatch's included, is already multiplied by it. The scale is
//! applied last, after any interpolation between bundle frames, so that the number shown is the
//! stored number the frame would show, converted, and not a blend of two converted numbers. (The
//! two agree to rounding, but only one of them is what the bundle says.) A line shown as stored has
//! the scale 1, and a number times 1 is that number to the bit: a bundle without `display` shows
//! exactly what it showed before there was one.
//!
//! The values that size a panel's value column and a subtitle's padding come from here too, so
//! the widths are those of the numbers as displayed.

use std::collections::BTreeMap;

use readout::Style;
use sky_format::{Num, ReadoutDecl};

use crate::timeline::Pick;

/// Which unit a read-out is shown in: `--units`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    /// The declaration's `display`, where it has one; the stored unit otherwise. The default.
    Display,
    /// The stored unit, always.
    Stored,
}

/// One line of a panel: one declared read-out, as it is shown.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub label: String,
    /// The unit written after the value: the declaration's, or its display unit.
    pub unit: String,
    pub decimals: usize,
    /// What a stored value is multiplied by before it is shown: 1 for a line shown as stored.
    pub scale: f64,
    /// The observer's stopwatch, whose value comes from the timeline rather than the bundle.
    pub stopwatch: bool,
}

impl Line {
    /// The line of declaration `d`, shown in `units`.
    pub fn of(d: &ReadoutDecl, units: Units) -> Self {
        let stopwatch = d.id == sky_format::STOPWATCH;
        match (&d.display, units) {
            (Some(display), Units::Display) => Self {
                label: d.label.clone(),
                unit: display.unit.clone(),
                decimals: display.decimals as usize,
                scale: display.scale.0,
                stopwatch,
            },
            _ => Self {
                label: d.label.clone(),
                unit: d.unit.clone(),
                decimals: d.decimals as usize,
                scale: 1.0,
                stopwatch,
            },
        }
    }
}

/// The read-outs of a run: the lines, and each line's value at each bundle frame being played.
#[derive(Debug, Clone, PartialEq)]
pub struct Series {
    pub lines: Vec<Line>,
    /// `values[p][l]`: line `l` at the `p`th bundle frame played (a position in the list of
    /// complete frames, as `Pick` counts them). None where the frame omits the read-out.
    values: Vec<Vec<Option<f64>>>,
}

impl Series {
    /// `frames` holds the `readouts` object of each bundle frame played, in order. Each line is
    /// shown in its display unit where it has one, as `--units display` (the default) asks.
    #[cfg(test)]
    pub fn new(decls: &[ReadoutDecl], frames: &[&BTreeMap<String, Num>]) -> Self {
        Self::in_units(decls, frames, Units::Display)
    }

    /// [`Series::new`], each line shown in `units`.
    pub fn in_units(
        decls: &[ReadoutDecl],
        frames: &[&BTreeMap<String, Num>],
        units: Units,
    ) -> Self {
        let lines = decls.iter().map(|d| Line::of(d, units)).collect();
        let values = frames
            .iter()
            .map(|values| {
                decls
                    .iter()
                    .map(|d| values.get(&d.id).map(|n| n.0))
                    .collect()
            })
            .collect();
        Self { lines, values }
    }

    /// Every value a line holds at a bundle frame, as displayed, for sizing the panel's value
    /// column.
    pub fn values_of(&self, line: usize) -> impl Iterator<Item = f64> + '_ {
        let scale = self.lines[line].scale;
        self.values
            .iter()
            .filter_map(move |frame| frame[line].map(|x| x * scale))
    }

    /// Each line's value at a video frame made of `pick`, whose stopwatch time (in the bundle's
    /// time unit) is `stopwatch`, as displayed. None for a line the panel leaves blank.
    pub fn at(&self, pick: Pick, stopwatch: f64) -> Vec<Option<f64>> {
        (0..self.lines.len())
            .map(|l| {
                let line = &self.lines[l];
                let stored = if line.stopwatch {
                    Some(stopwatch)
                } else {
                    match pick {
                        Pick::One(a) => self.values[a][l],
                        Pick::Two(a, b, w) => interpolate(self.values[a][l], self.values[b][l], w),
                    }
                };
                // Interpolated as stored, then converted (the module's comment).
                stored.map(|x| x * line.scale)
            })
            .collect()
    }
}

/// The value `w` of the way from `a` to `b`: linear between two finite values, and otherwise the
/// nearer one's, missing or not finite as it is. Exactly half way counts as nearer the second.
pub fn interpolate(a: Option<f64>, b: Option<f64>, w: f64) -> Option<f64> {
    match (a, b) {
        (Some(x), Some(y)) if x.is_finite() && y.is_finite() => Some(x + w * (y - x)),
        _ if w < 0.5 => a,
        _ => b,
    }
}

/// Values above this are written with an exponent; at and below it, in full.
const EXPONENT_ABOVE: f64 = readout::EXPONENT_ABOVE;

/// `x` at exactly `decimals` places, in `style`: `4.57`, `1,234.50`, `n/a` for NaN, `∞` and `-∞`.
///
/// `Style::exact` rather than the app's usual `Style::fixed`: `fixed` drops the decimals from ten
/// thousand up, and in a video a stopwatch passing 9,999.99 would lose two digits at once and jump
/// sideways. Exact keeps every decimal the manifest asks for, so a value keeps its shape as it
/// grows. Past a billion, where writing every digit would widen the panel beyond use, it turns to
/// `fixed`'s exponent.
///
/// A number that rounds to zero at the places it is shown to is written as zero without a sign:
/// `0.0`, never `-0.0`. A writer's value can be zero to rounding with either sign (a heading of
/// -4e-7 degrees), and a minus on a zero says nothing a reader can use. The decision is made on
/// the text as printed, not on the value: `-0.05` at one place rounds to `-0.1` and keeps its
/// sign, `-0.04` rounds to `-0.0` and loses it, whatever the rounding rule. A text with no digit
/// at all (`n/a`, `∞`, `-∞`) is left as it is. Every width a panel or a subtitle is sized by is
/// this function's text, so the widths follow the same rule.
pub fn format(style: Style, x: f64, decimals: usize) -> String {
    let text = if x.is_finite() && x.abs() > EXPONENT_ABOVE {
        style.fixed(x, decimals)
    } else {
        style.exact(x, decimals)
    };
    let digits = || text.chars().filter(char::is_ascii_digit);
    match text.strip_prefix('-') {
        Some(unsigned) if digits().next().is_some() && digits().all(|d| d == '0') => {
            unsigned.to_string()
        }
        _ => text,
    }
}
