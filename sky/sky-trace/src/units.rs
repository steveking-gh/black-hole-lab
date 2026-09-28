//! Read-outs in seconds and kilometres: the `display` of specification 6.1.
//!
//! The values a bundle stores stay in M, the hole's mass in geometric units. What this module
//! chooses is how a renderer that knows `display` shows them: a unit, the number of that unit one
//! M is, and how many decimals.
//!
//! # The conversion
//!
//! One M of time is `m_solar` G M_sun / c^3 seconds (`bhl::Hole::seconds_per_unit`), from the
//! accepted G M_sun / c^3 = 4.925490947e-6 s that `bhl` already uses. One M of length is the
//! distance light goes in that time, the same number of seconds times c = 299 792.458 km/s, which
//! is exact by the definition of the metre. So the second and the kilometre come from one number
//! and agree with each other: a radius of 1 M is, in kilometres, exactly c times the duration of
//! 1 M in seconds.
//!
//! The app's own constants are 4.927038e-6 s and 1.477 km per solar mass. They are 3.1 and 2.5
//! parts in 10^4 above the accepted values, by different amounts (1.477 km / c is 4.9267e-6 s, not
//! 4.927038e-6 s), so they do not agree with each other either. This program keeps to the accepted
//! values, and its read-outs therefore differ from the app's in the fourth significant figure: the
//! app's Sagittarius A* lasts 20.45 s an M where this one's lasts 20.44 s.
//!
//! # The unit
//!
//! A read-out keeps one unit for the whole film (specification 6.1: `display.scale` is one number
//! for the bundle), so the unit is chosen once, from the largest magnitude the read-out reaches
//! over the film's frames - which is known before anything is traced, because the worldline is
//! walked first. It is never chosen from less than 1 M: a watch that reads 0 on the only frame
//! there is is shown in the unit natural to one M of time, not in microseconds.
//!
//! The time ladder is the app's (`format_physical_time` in the app's `src/gui/units.rs`), with the
//! app's thresholds, so that a clock reads as the app's does: microseconds below a millisecond,
//! then ms, s, min from a minute, hrs from an hour, days from a day, yr from a Julian year of
//! 365.25 days.
//!
//! The length ladder is not the app's, and on purpose. The app's `format_physical_distance` goes
//! from km to "M km" to AU from 0.05 AU and to light-years from 0.01 ly, and writes a second unit
//! in brackets beside the first; a display has one unit and at most three decimals, and with
//! those rungs a radius that has just passed one of them reads with two significant figures: Bob's
//! 13.89 million km at Sagittarius A* would be 0.093 AU, and TON 618's 1 M would be 0.010 ly. The
//! owner asked for kilometres, so the ladder here is metres below a kilometre, then km, "million
//! km" from 10^6 km and "billion km" from 10^9 km (the app writes "M km", which beside a mass unit
//! called M reads as a number of masses: here the words are written out), and ly from one
//! light-year. Every rung starts at one of its own unit, so that with four significant figures
//! and at most three decimals every reading at or above its unit has at least four figures. There
//! is no astronomical-unit rung.
//!
//! The light-year is not the app's either. The app rounds it to four figures (9.461e12 km), 2.9
//! parts in 10^5 off, enough to move the fourth figure now and then; here it is the IAU's, the
//! distance light goes in a Julian year: 299 792.458 km/s times 365.25 x 86 400 s, exactly
//! 9 460 730 472 580.8 km.
//!
//! # The decimals
//!
//! Enough for four significant figures of the largest magnitude in the chosen unit, at most 3 and
//! at least 0: 9.99 is shown to 3 decimals, 13.89 to 2, 188.2 to 1 and 1234 to 0.

use sky_format::{Display, Num};

/// The speed of light, in km/s: exact, by the definition of the metre.
pub const C_KM_PER_S: f64 = 299_792.458;

/// Seconds in a Julian year of 365.25 days.
pub const SECONDS_PER_YEAR: f64 = 365.25 * 86_400.0;

/// The light-year, in km: the distance light goes in a Julian year, 299 792.458 x 31 557 600 km,
/// written out because the f64 product of the two rounds to the neighbouring f64 below it.
pub const KM_PER_LY: f64 = 9_460_730_472_580.8;

/// Which units the read-outs are declared in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    /// Seconds and kilometres (and their multiples) as a `display` on each time and length
    /// read-out; the stored values stay in M.
    Physical,
    /// M alone, with no `display`: the bundle as it was before displays existed.
    Geometric,
}

/// The time ladder: each unit, and how many seconds it is, from the largest down. A magnitude of
/// at least the unit's length in seconds is shown in that unit; below a millisecond, in µs.
const TIME_LADDER: [(&str, f64); 7] = [
    ("yr", SECONDS_PER_YEAR),
    ("days", 86_400.0),
    ("hrs", 3_600.0),
    ("min", 60.0),
    ("s", 1.0),
    ("ms", 1e-3),
    ("µs", 1e-6),
];

/// The length ladder: each unit and how many km it is, from the largest down. A magnitude of at
/// least one of the unit is shown in that unit; below a kilometre, in metres.
const LENGTH_LADDER: [(&str, f64); 5] = [
    ("ly", KM_PER_LY),
    ("billion km", 1e9),
    ("million km", 1e6),
    ("km", 1.0),
    ("m", 1e-3),
];

/// How many decimals show four significant figures of a magnitude, clamped to [0, 3].
///
/// Counted by comparison with powers of ten rather than by a logarithm, whose rounding would put
/// 1000 on the wrong side of a decade now and then.
pub fn decimals_for(magnitude: f64) -> u32 {
    let mut decimals = 3u32;
    let mut decade = 10.0;
    while decimals > 0 && magnitude >= decade {
        decimals -= 1;
        decade *= 10.0;
    }
    decimals
}

/// The display of a time read-out whose largest magnitude over the film is `largest` M, for a
/// hole whose M lasts `seconds_per_m` seconds.
pub fn time_display(largest: f64, seconds_per_m: f64) -> Display {
    // Never from less than one M; and a NaN, which no read-out of this program is, would fall to
    // one M as well rather than choose a unit from nothing.
    let seconds = largest.abs().max(1.0) * seconds_per_m;
    let (unit, length) = TIME_LADDER
        .into_iter()
        .find(|&(_, length)| seconds >= length)
        .unwrap_or(TIME_LADDER[TIME_LADDER.len() - 1]);
    Display {
        unit: unit.into(),
        scale: Num(seconds_per_m / length),
        decimals: decimals_for(seconds / length),
    }
}

/// The display of a length read-out whose largest magnitude over the film is `largest` M, for a
/// hole whose M lasts `seconds_per_m` seconds, and is therefore `seconds_per_m` c long.
pub fn length_display(largest: f64, seconds_per_m: f64) -> Display {
    let km_per_m = seconds_per_m * C_KM_PER_S;
    let km = largest.abs().max(1.0) * km_per_m;
    let (unit, length) = LENGTH_LADDER
        .into_iter()
        .find(|&(_, length)| km >= length)
        .unwrap_or(LENGTH_LADDER[LENGTH_LADDER.len() - 1]);
    Display {
        unit: unit.into(),
        scale: Num(km_per_m / length),
        decimals: decimals_for(km / length),
    }
}

#[cfg(test)]
mod tests;
