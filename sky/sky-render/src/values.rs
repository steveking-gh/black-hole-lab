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

use std::collections::BTreeMap;

use readout::Style;
use sky_format::{Num, ReadoutDecl};

use crate::timeline::Pick;

/// One line of a panel: one declared read-out.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub label: String,
    pub unit: String,
    pub decimals: usize,
    /// The observer's stopwatch, whose value comes from the timeline rather than the bundle.
    pub stopwatch: bool,
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
    /// `frames` holds the `readouts` object of each bundle frame played, in order.
    pub fn new(decls: &[ReadoutDecl], frames: &[&BTreeMap<String, Num>]) -> Self {
        let lines = decls
            .iter()
            .map(|d| Line {
                label: d.label.clone(),
                unit: d.unit.clone(),
                decimals: d.decimals as usize,
                stopwatch: d.id == sky_format::STOPWATCH,
            })
            .collect();
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

    /// Every value a line holds at a bundle frame, for sizing the panel's value column.
    pub fn values_of(&self, line: usize) -> impl Iterator<Item = f64> + '_ {
        self.values.iter().filter_map(move |frame| frame[line])
    }

    /// Each line's value at a video frame made of `pick`, whose stopwatch time is `stopwatch`.
    /// None for a line the panel leaves blank.
    pub fn at(&self, pick: Pick, stopwatch: f64) -> Vec<Option<f64>> {
        (0..self.lines.len())
            .map(|l| {
                if self.lines[l].stopwatch {
                    return Some(stopwatch);
                }
                match pick {
                    Pick::One(a) => self.values[a][l],
                    Pick::Two(a, b, w) => interpolate(self.values[a][l], self.values[b][l], w),
                }
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
pub fn format(style: Style, x: f64, decimals: usize) -> String {
    if x.is_finite() && x.abs() > EXPONENT_ABOVE {
        style.fixed(x, decimals)
    } else {
        style.exact(x, decimals)
    }
}
