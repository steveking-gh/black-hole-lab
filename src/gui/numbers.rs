//! How the app prints a number: the `readout` crate, in the style the panel has chosen.
//!
//! The style is the "Decimal is comma" box under UNITS & COORDINATE SYSTEM, and every number on
//! the screen follows it. It is held here as the style of the current frame rather than passed
//! down to each of the hundred-odd places that print a number, the way a locale is: the app sets
//! it once at the top of each frame (`set_style`) and every formatter reads it. It is per thread,
//! so the tests - each on its own thread - cannot see one another's setting, and a thread that
//! never sets it prints in point style.
//!
//! Every function here is the `readout` function of the same name in the current style. See that
//! crate for the rules: grouped from a thousand, an exponent only above a billion (or below a
//! millionth for a pure ratio), SI prefixes for a quantity with a unit.

use std::borrow::Cow;
use std::cell::Cell;
use std::ops::RangeInclusive;

pub(crate) use readout::Style;
use readout::{DecimalMark, Prefixes};

thread_local! {
    static STYLE: Cell<Style> = const { Cell::new(Style::POINT) };
}

/// The style this frame prints in.
pub(crate) fn style() -> Style {
    STYLE.with(Cell::get)
}

/// Print in `style` from here on, on this thread.
pub(crate) fn set_style(style: Style) {
    STYLE.with(|cell| cell.set(style));
}

/// The style the panel's "Decimal is comma" box asks for.
pub(crate) fn style_for(decimal_is_comma: bool) -> Style {
    Style::new(if decimal_is_comma { DecimalMark::Comma } else { DecimalMark::Point })
}

/// `x` at `decimals` places, grouped from a thousand and whole from ten thousand.
pub(crate) fn fixed(x: impl Into<f64>, decimals: usize) -> String {
    style().fixed(x.into(), decimals)
}

/// `x` at exactly `decimals` places, grouped, and never an exponent: for axis and grid labels,
/// which must differ from their neighbours in the last place.
pub(crate) fn exact(x: impl Into<f64>, decimals: usize) -> String {
    style().exact(x.into(), decimals)
}

/// `exact` with a + in front of a value that is not negative, for the signed axis labels.
pub(crate) fn exact_signed(x: impl Into<f64>, decimals: usize) -> String {
    style().exact_signed(x.into(), decimals)
}

/// `fixed` with a + in front of a value that is not negative, for the signed rates.
pub(crate) fn fixed_signed(x: impl Into<f64>, decimals: usize) -> String {
    style().fixed_signed(x.into(), decimals)
}

/// A small pure ratio at four significant digits, in plain decimals down to a millionth.
pub(crate) fn small(x: f64) -> String {
    style().significant(x, 4)
}

/// `small` with a + in front of a value that is not negative.
pub(crate) fn small_signed(x: f64) -> String {
    style().significant_signed(x, 4)
}

/// `x` at `digits` significant digits.
pub(crate) fn significant(x: f64, digits: usize) -> String {
    style().significant(x, digits)
}

/// `x` as an exponent with `decimals` places on the mantissa.
pub(crate) fn exponent(x: f64, decimals: usize) -> String {
    style().exponent(x, decimals)
}

/// An angular velocity in radians per second, signed, at four significant digits under an SI
/// prefix: +241.8 mrad/s, -3.204 µrad/s. The span is wide - a stellar-mass hole's horizon turns
/// ten thousand radians a second and a supermassive one's far field a few picoradians - and a
/// prefix per factor of a thousand keeps every one of them to a handful of characters.
pub(crate) fn rad_per_second(per_s: f64) -> String {
    style().si_signed(per_s, "rad/s", 4, Prefixes::Small)
}

/// A frequency in hertz at four significant digits under an SI prefix, pico to tera.
pub(crate) fn hertz(hz: f64) -> String {
    style().si(hz, "Hz", 4, Prefixes::All)
}

/// Fixed prose written in point style - a hover tip, the theory window, a preset's name - in the
/// current style. Borrowed, and free, in point style.
///
/// For text that is written once in the source and never for a label assembled at run time: the
/// conversion swaps the two marks, so a label whose numbers were already printed by the functions
/// above would have them swapped back. A label built with `format!` prints each of its numbers
/// through `fixed` and the rest instead.
pub(crate) fn text(text: &str) -> Cow<'_, str> {
    style().localize(text)
}

/// What goes between the numbers of a list: ", ", or "; " where the comma is the decimal mark.
pub(crate) fn list_separator() -> &'static str {
    style().list_separator()
}

/// A number typed into one of the panel's fields, in the current style.
pub(crate) fn parse(text: &str) -> Option<f64> {
    style().parse(text)
}

/// A slider that shows its number in the current style and reads a typed one back in it.
///
/// The decimals stay the ones egui would have chosen for the slider's range and step, so ticking
/// the box changes the marks and never the precision: see `widget_number`.
pub(crate) trait Styled {
    fn styled(self) -> Self;
}

impl Styled for egui::Slider<'_> {
    fn styled(self) -> Self {
        self.custom_formatter(widget_number).custom_parser(parse)
    }
}

/// A widget's number at the decimals egui picked for it, grouped and in the current style.
fn widget_number(value: f64, decimals: RangeInclusive<usize>) -> String {
    let plain = egui::emath::format_with_decimals_in_range(value, decimals);
    let places = plain.split_once('.').map_or(0, |(_, fraction)| fraction.len());
    exact(value, places)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_style_is_per_thread_and_starts_as_point() {
        assert_eq!(style(), Style::POINT);
        set_style(style_for(true));
        assert_eq!(fixed(1234.5, 1), "1.234,5");
        let other = std::thread::spawn(|| fixed(1234.5, 1)).join().unwrap();
        assert_eq!(other, "1,234.5", "another thread still prints in point style");
        set_style(style_for(false));
        assert_eq!(fixed(1234.5, 1), "1,234.5");
    }

    #[test]
    fn test_a_slider_keeps_egui_s_decimals_and_takes_the_style() {
        assert_eq!(widget_number(6.6182, 3..=3), "6.618");
        assert_eq!(widget_number(12_345.0, 0..=0), "12,345");
        set_style(style_for(true));
        assert_eq!(widget_number(6.6182, 3..=3), "6,618");
        assert_eq!(parse("6,618"), Some(6.618));
        set_style(style_for(false));
    }

    #[test]
    fn test_the_app_wrappers_keep_their_units() {
        assert_eq!(rad_per_second(-0.2418), "-241.8 mrad/s");
        assert_eq!(rad_per_second(5.38e-2), "+53.80 mrad/s");
        assert_eq!(rad_per_second(10_150.0), "+10,150 rad/s");
        assert_eq!(hertz(136_150.846), "136.2 kHz");
        assert_eq!(small(2.418e-4), "0.0002418");
    }
}
