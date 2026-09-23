//! Numbers as a person reads them off a display.
//!
//! Rust's `format!` writes a number the way a program reads it back: `-1355000.0`, `2.418e-1`,
//! always with a point for the decimal mark. This crate writes it the way a reader takes it in:
//!
//! - **Grouped digits.** From a thousand up the whole part is grouped in threes: `4,521.37`,
//!   `135,500,000`. From ten thousand up the decimals are dropped, since at that size they are
//!   counted rather than read.
//! - **An exponent only where plain notation cannot fit:** above [`EXPONENT_ABOVE`] (a billion),
//!   and for a small pure number below [`EXPONENT_BELOW`] (a millionth).
//! - **SI prefixes** for a small or large value that has a unit: `-241.8 mrad/s`, `136.2 MHz`.
//! - **Either decimal mark.** [`Style::POINT`] writes `1,234.5`; [`Style::COMMA`] writes
//!   `1.234,5`, the convention of most of continental Europe and South America.
//!
//! Every method is a pure function of the [`Style`] and its arguments: no global locale, no state.
//! [`Style::localize`] converts text already written in point style, for prose and labels that
//! carry numbers inside them, and [`Style::parse`] reads a number typed in either style back.
//!
//! ```
//! use readout::Style;
//!
//! assert_eq!(Style::POINT.fixed(4521.374, 2), "4,521.37");
//! assert_eq!(Style::COMMA.fixed(4521.374, 2), "4.521,37");
//! assert_eq!(Style::POINT.fixed(-1.355e8, 2), "-135,500,000");
//! assert_eq!(Style::POINT.significant(2.418e-4, 4), "0.0002418");
//! assert_eq!(Style::COMMA.si(-0.2418, "rad/s", 4, readout::Prefixes::Small), "-241,8 mrad/s");
//! assert_eq!(Style::COMMA.parse("1.234,5"), Some(1234.5));
//! ```

use std::borrow::Cow;

/// Magnitudes above this are written as an exponent: `1.50e9`. At and below it they are plain.
pub const EXPONENT_ABOVE: f64 = 1e9;

/// Magnitudes below this are written as an exponent by [`Style::significant`]: six zeroes after
/// the decimal mark are as many as an eye counts.
pub const EXPONENT_BELOW: f64 = 1e-6;

/// From this magnitude up, the whole part of a number is grouped in threes.
pub const GROUP_FROM: f64 = 1e3;

/// From this magnitude up, [`Style::fixed`] drops the decimals.
pub const WHOLE_FROM: f64 = 1e4;

/// Which character separates the whole part of a number from its fraction.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum DecimalMark {
    /// `1,234.5`: a point for the decimal mark and a comma between groups.
    #[default]
    Point,
    /// `1.234,5`: a comma for the decimal mark and a point between groups.
    Comma,
}

/// Which SI prefixes [`Style::si`] may choose from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Prefixes {
    /// Pico to milli, and the bare unit from 1 up, grouped: `3.204 µrad/s`, `10,150 rad/s`.
    /// For a unit that no one prefixes upwards.
    Small,
    /// Pico to tera: `489.0 mHz`, `136.2 MHz`, `3.000 THz`.
    All,
}

/// How numbers are written: at present, the decimal mark and the group separator that goes with
/// it. `Style::default()` is [`Style::POINT`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct Style {
    decimal_mark: DecimalMark,
}

/// The prefixes [`Style::si`] knows, smallest first.
const PREFIX_LADDER: [(&str, f64); 9] = [
    ("p", 1e-12),
    ("n", 1e-9),
    ("µ", 1e-6),
    ("m", 1e-3),
    ("", 1.0),
    ("k", 1e3),
    ("M", 1e6),
    ("G", 1e9),
    ("T", 1e12),
];

impl Style {
    /// `1,234.5`.
    pub const POINT: Self = Self {
        decimal_mark: DecimalMark::Point,
    };

    /// `1.234,5`.
    pub const COMMA: Self = Self {
        decimal_mark: DecimalMark::Comma,
    };

    /// The style with this decimal mark.
    pub const fn new(decimal_mark: DecimalMark) -> Self {
        Self { decimal_mark }
    }

    /// The style's decimal mark.
    pub const fn decimal_mark(self) -> DecimalMark {
        self.decimal_mark
    }

    /// The character between the whole part and the fraction: `.` or `,`.
    pub const fn decimal_char(self) -> char {
        match self.decimal_mark {
            DecimalMark::Point => '.',
            DecimalMark::Comma => ',',
        }
    }

    /// The character between groups of three digits: `,` or `.`.
    pub const fn group_char(self) -> char {
        match self.decimal_mark {
            DecimalMark::Point => ',',
            DecimalMark::Comma => '.',
        }
    }

    /// What goes between the numbers of a list: `", "`, or `"; "` where the comma is the decimal
    /// mark and `1,5, 2,5` would not say how many numbers it holds.
    pub const fn list_separator(self) -> &'static str {
        match self.decimal_mark {
            DecimalMark::Point => ", ",
            DecimalMark::Comma => "; ",
        }
    }

    /// `x` at `decimals` places, grouped from [`GROUP_FROM`], whole from [`WHOLE_FROM`], and an
    /// exponent above [`EXPONENT_ABOVE`]. Infinity is `∞` and NaN is `n/a`.
    ///
    /// The thresholds are judged on the value as it will print, so 9,999.996 at two places is
    /// `10,000` and not `10,000.00`.
    pub fn fixed(self, x: f64, decimals: usize) -> String {
        if let Some(text) = non_finite(x) {
            return text;
        }
        let rounded: f64 = format!("{x:.decimals$}").parse().unwrap_or(x);
        let mag = rounded.abs();
        if mag > EXPONENT_ABOVE {
            return self.exponent(x, 2);
        }
        let decimals = if mag >= WHOLE_FROM { 0 } else { decimals };
        self.localize_plain(&format!("{x:.decimals$}"))
    }

    /// `x` at exactly `decimals` places, grouped from [`GROUP_FROM`], and never an exponent:
    /// `12,345.6`. For a label that has to differ from its neighbour in the last place shown,
    /// such as the tick of an axis ruled at a step of 0.1, where [`Style::fixed`] would drop the
    /// digit that tells two ticks apart.
    pub fn exact(self, x: f64, decimals: usize) -> String {
        if let Some(text) = non_finite(x) {
            return text;
        }
        self.localize_plain(&format!("{x:.decimals$}"))
    }

    /// [`Style::exact`] with a `+` on a value that does not print as negative.
    pub fn exact_signed(self, x: f64, decimals: usize) -> String {
        with_plus(self.exact(x, decimals))
    }

    /// [`Style::fixed`] with a `+` on a value that does not print as negative.
    pub fn fixed_signed(self, x: f64, decimals: usize) -> String {
        with_plus(self.fixed(x, decimals))
    }

    /// `x` to `digits` significant digits in plain notation: `0.0002418`, `4.521`, `4,521`.
    /// Below [`EXPONENT_BELOW`] and above [`EXPONENT_ABOVE`] it is an exponent. Zero is `0`.
    ///
    /// For a pure number with no unit to put a prefix on; one that has a unit wants
    /// [`Style::si`].
    pub fn significant(self, x: f64, digits: usize) -> String {
        if let Some(text) = non_finite(x) {
            return text;
        }
        let digits = digits.max(1);
        if x == 0.0 {
            return "0".to_string();
        }
        let rounded = round_significant(x, digits);
        let mag = rounded.abs();
        if !(EXPONENT_BELOW..=EXPONENT_ABOVE).contains(&mag) {
            return self.exponent(x, digits - 1);
        }
        let decimals = decimals_for(mag, digits);
        self.localize_plain(&format!("{rounded:.decimals$}"))
    }

    /// [`Style::significant`] with a `+` on a value that does not print as negative.
    pub fn significant_signed(self, x: f64, digits: usize) -> String {
        with_plus(self.significant(x, digits))
    }

    /// `x` as an exponent with `decimals` places on the mantissa: `1.50e9`, `3.000e-10`.
    pub fn exponent(self, x: f64, decimals: usize) -> String {
        if let Some(text) = non_finite(x) {
            return text;
        }
        self.localize_plain(&format!("{x:.decimals$e}"))
    }

    /// `x` of `unit` to `digits` significant digits under the SI prefix that puts the mantissa in
    /// [1, 1000): `-241.8 mrad/s`, `136.2 MHz`. The number is signed as `x` is, with no `+`.
    ///
    /// Past the ends of `prefixes` it falls back: below pico to an exponent, and above the
    /// largest prefix to [`Style::fixed`] in that prefix, which turns to an exponent in its turn
    /// above [`EXPONENT_ABOVE`]. Zero prints in the bare unit.
    pub fn si(self, x: f64, unit: &str, digits: usize, prefixes: Prefixes) -> String {
        if let Some(text) = non_finite(x) {
            return format!("{text} {unit}");
        }
        let digits = digits.max(1);
        if x == 0.0 {
            return format!("{} {unit}", self.fixed(0.0, digits - 1));
        }
        let rounded = round_significant(x, digits).abs();
        let ladder: &[(&str, f64)] = match prefixes {
            Prefixes::Small => &PREFIX_LADDER[..5],
            Prefixes::All => &PREFIX_LADDER[..],
        };
        let Some(&(prefix, scale)) = ladder.iter().rev().find(|(_, scale)| rounded >= *scale)
        else {
            return format!("{} {unit}", self.exponent(x, digits - 1));
        };
        let mantissa = rounded / scale;
        if mantissa >= 1000.0 {
            // Past the top of the ladder: whole numbers of the largest prefix.
            return format!("{} {prefix}{unit}", self.fixed(x / scale, 0));
        }
        let decimals = decimals_for(mantissa, digits);
        let text = format!("{:.decimals$}", x.signum() * mantissa);
        format!("{} {prefix}{unit}", self.localize_plain(&text))
    }

    /// `x` of `unit` like [`Style::si`], with a `+` on a value that does not print as negative.
    pub fn si_signed(self, x: f64, unit: &str, digits: usize, prefixes: Prefixes) -> String {
        with_plus(self.si(x, unit, digits, prefixes))
    }

    /// Text written in point style, rewritten in this style.
    ///
    /// A `.` or `,` with a digit on both sides is part of a number, and swaps for the other one;
    /// every other character is left alone, so a list comma followed by a space, a full stop at
    /// the end of a sentence and an abbreviation such as `e.g.` all survive. Point style returns
    /// the text as it came, borrowed.
    ///
    /// The one thing it cannot tell apart is a dotted string of digits that is not a number,
    /// such as a version `0.1.12`: text carrying one should not be passed through it.
    pub fn localize(self, text: &str) -> Cow<'_, str> {
        if self.decimal_mark == DecimalMark::Point || !text.contains(['.', ',']) {
            return Cow::Borrowed(text);
        }
        let chars: Vec<char> = text.chars().collect();
        let mut out = String::with_capacity(text.len());
        for (i, &c) in chars.iter().enumerate() {
            let between_digits = i > 0
                && chars[i - 1].is_ascii_digit()
                && chars.get(i + 1).is_some_and(char::is_ascii_digit);
            out.push(match c {
                '.' if between_digits => ',',
                ',' if between_digits => '.',
                other => other,
            });
        }
        Cow::Owned(out)
    }

    /// A number typed in this style, or None if the text is not one.
    ///
    /// Group separators are ignored wherever they fall, the decimal mark may be either this
    /// style's or - where the text is otherwise unambiguous - a point, and a leading `+`, a
    /// Unicode minus `−` and surrounding whitespace are accepted. Exponents are read in either
    /// style: `1,5e9` in comma style is 1.5e9.
    pub fn parse(self, text: &str) -> Option<f64> {
        let text = text.trim().replace('−', "-");
        let text = text.strip_prefix('+').unwrap_or(&text);
        let plain: String = match self.decimal_mark {
            DecimalMark::Point => text.chars().filter(|&c| c != ',').collect(),
            DecimalMark::Comma => {
                // "1.234,5" and "1,5" are comma style; "1.5" with no comma anywhere is a point
                // decimal typed out of habit, which is what a single point next to a fraction
                // shorter than a group almost always is.
                let has_comma = text.contains(',');
                let lone_point = !has_comma
                    && text.matches('.').count() == 1
                    && text.split('.').nth(1).is_some_and(|frac| {
                        frac.chars().take_while(char::is_ascii_digit).count() != 3
                    });
                if lone_point {
                    text.to_string()
                } else {
                    text.chars()
                        .filter(|&c| c != '.')
                        .map(|c| if c == ',' { '.' } else { c })
                        .collect()
                }
            }
        };
        plain.parse::<f64>().ok().filter(|v| !v.is_nan())
    }

    /// A plain point-style number string (a sign, digits, one optional point, an optional
    /// exponent) with its whole part grouped from [`GROUP_FROM`] and its marks in this style.
    fn localize_plain(self, plain: &str) -> String {
        let (sign, rest) = match plain.strip_prefix('-') {
            Some(rest) => ("-", rest),
            None => ("", plain),
        };
        let (mantissa, exponent) = match rest.find(['e', 'E']) {
            Some(at) => rest.split_at(at),
            None => (rest, ""),
        };
        let (whole, fraction) = match mantissa.split_once('.') {
            Some((whole, fraction)) => (whole, Some(fraction)),
            None => (mantissa, None),
        };
        let mut out = String::with_capacity(plain.len() + whole.len() / 3);
        out.push_str(sign);
        let group = whole.len() > 3 && exponent.is_empty();
        for (i, ch) in whole.chars().enumerate() {
            if group && i > 0 && (whole.len() - i) % 3 == 0 {
                out.push(self.group_char());
            }
            out.push(ch);
        }
        if let Some(fraction) = fraction {
            out.push(self.decimal_char());
            out.push_str(fraction);
        }
        out.push_str(exponent);
        out
    }
}

/// The text for a value with no digits to print, or None for a finite one.
fn non_finite(x: f64) -> Option<String> {
    if x.is_nan() {
        Some("n/a".to_string())
    } else if x.is_infinite() {
        Some(if x > 0.0 { "∞" } else { "-∞" }.to_string())
    } else {
        None
    }
}

/// `x` rounded to `digits` significant digits, so that a value which rounds up across a power of
/// ten is judged at its rounded size.
fn round_significant(x: f64, digits: usize) -> f64 {
    let quantum = 10f64.powf(x.abs().log10().floor() + 1.0 - digits as f64);
    (x / quantum).round() * quantum
}

/// How many decimals put a value of magnitude `mag` at `digits` significant digits.
fn decimals_for(mag: f64, digits: usize) -> usize {
    (digits as f64 - 1.0 - mag.log10().floor()).max(0.0) as usize
}

/// `text` with a `+` in front unless it already starts with a minus.
fn with_plus(text: String) -> String {
    if text.starts_with('-') {
        text
    } else {
        format!("+{text}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: Style = Style::POINT;
    const C: Style = Style::COMMA;

    #[test]
    fn test_the_default_style_is_point() {
        assert_eq!(Style::default(), P);
        assert_eq!(Style::new(DecimalMark::Comma), C);
        assert_eq!((P.decimal_char(), P.group_char()), ('.', ','));
        assert_eq!((C.decimal_char(), C.group_char()), (',', '.'));
    }

    #[test]
    fn test_fixed_keeps_small_numbers_as_format_writes_them() {
        assert_eq!(P.fixed(0.0, 2), "0.00");
        assert_eq!(P.fixed(-5.444, 2), "-5.44");
        assert_eq!(P.fixed(999.994, 2), "999.99");
        assert_eq!(P.fixed(-0.004, 2), "-0.00");
        assert_eq!(C.fixed(-5.444, 2), "-5,44");
        assert_eq!(P.fixed(12.0, 0), "12");
    }

    #[test]
    fn test_fixed_groups_from_a_thousand_and_drops_decimals_from_ten_thousand() {
        assert_eq!(P.fixed(1234.5, 1), "1,234.5");
        assert_eq!(C.fixed(1234.5, 1), "1.234,5");
        assert_eq!(P.fixed(9999.994, 2), "9,999.99");
        assert_eq!(P.fixed(9999.996, 2), "10,000");
        assert_eq!(P.fixed(61_440.4, 2), "61,440");
        assert_eq!(P.fixed(-1.355e8, 2), "-135,500,000");
        assert_eq!(C.fixed(-1.355e8, 2), "-135.500.000");
    }

    #[test]
    fn test_fixed_turns_to_an_exponent_only_past_a_billion() {
        assert_eq!(P.fixed(1e9, 2), "1,000,000,000");
        assert_eq!(P.fixed(1.5e9, 2), "1.50e9");
        assert_eq!(C.fixed(-1.5e9, 2), "-1,50e9");
    }

    #[test]
    fn test_exact_keeps_every_decimal_it_is_asked_for() {
        assert_eq!(P.exact(12_345.66, 1), "12,345.7");
        assert_eq!(C.exact(12_345.66, 1), "12.345,7");
        assert_eq!(P.exact(6.1301e9, 0), "6,130,100,000");
        assert_eq!(P.exact(0.43581, 4), "0.4358");
        assert_ne!(P.exact(12_345.6, 1), P.exact(12_345.7, 1));
        assert_eq!(P.exact_signed(0.0, 0), "+0");
        assert_eq!(C.exact_signed(-2.5, 1), "-2,5");
    }

    #[test]
    fn test_non_finite_values_have_names() {
        assert_eq!(P.fixed(f64::INFINITY, 2), "∞");
        assert_eq!(P.fixed(f64::NEG_INFINITY, 2), "-∞");
        assert_eq!(P.fixed(f64::NAN, 2), "n/a");
        assert_eq!(P.significant(f64::NAN, 4), "n/a");
        assert_eq!(P.si(f64::INFINITY, "Hz", 4, Prefixes::All), "∞ Hz");
    }

    #[test]
    fn test_signed_variants_put_a_plus_on_everything_not_printed_negative() {
        assert_eq!(P.fixed_signed(0.0, 2), "+0.00");
        assert_eq!(P.fixed_signed(-0.004, 2), "-0.00");
        assert_eq!(P.fixed_signed(452.0, 2), "+452.00");
        assert_eq!(C.fixed_signed(-452.0, 2), "-452,00");
        assert_eq!(P.significant_signed(1e-5, 4), "+0.00001000");
        assert_eq!(
            P.si_signed(0.0538, "rad/s", 4, Prefixes::Small),
            "+53.80 mrad/s"
        );
        assert_eq!(P.fixed_signed(f64::INFINITY, 2), "+∞");
    }

    #[test]
    fn test_significant_stays_plain_between_a_millionth_and_a_billion() {
        assert_eq!(P.significant(2.418e-4, 4), "0.0002418");
        assert_eq!(C.significant(2.418e-4, 4), "0,0002418");
        assert_eq!(P.significant(0.009, 4), "0.009000");
        assert_eq!(P.significant(1.2e-6, 4), "0.000001200");
        assert_eq!(P.significant(4521.37, 4), "4,521");
        assert_eq!(P.significant(0.99996, 4), "1.000");
        assert_eq!(P.significant(0.0, 4), "0");
    }

    #[test]
    fn test_significant_turns_to_an_exponent_outside_that_range() {
        assert_eq!(P.significant(3.0e-10, 4), "3.000e-10");
        assert_eq!(C.significant(3.0e-10, 4), "3,000e-10");
        assert_eq!(P.significant(2.5e12, 3), "2.50e12");
    }

    #[test]
    fn test_si_picks_the_prefix_that_puts_the_mantissa_under_a_thousand() {
        assert_eq!(P.si(-0.2418, "rad/s", 4, Prefixes::Small), "-241.8 mrad/s");
        assert_eq!(C.si(-0.2418, "rad/s", 4, Prefixes::Small), "-241,8 mrad/s");
        assert_eq!(P.si(3.204e-6, "rad/s", 4, Prefixes::Small), "3.204 µrad/s");
        assert_eq!(P.si(2.0e-11, "rad/s", 4, Prefixes::Small), "20.00 prad/s");
        assert_eq!(P.si(136_150.846, "Hz", 4, Prefixes::All), "136.2 kHz");
        assert_eq!(P.si(7.77e10, "Hz", 4, Prefixes::All), "77.70 GHz");
        assert_eq!(P.si(12.345, "Hz", 4, Prefixes::All), "12.35 Hz");
    }

    #[test]
    fn test_si_judges_the_prefix_on_the_rounded_value() {
        assert_eq!(P.si(999.96, "Hz", 4, Prefixes::All), "1.000 kHz");
        assert_eq!(P.si(0.99996, "rad/s", 4, Prefixes::Small), "1.000 rad/s");
    }

    #[test]
    fn test_si_falls_back_past_either_end_of_its_prefixes() {
        assert_eq!(P.si(10_150.0, "rad/s", 4, Prefixes::Small), "10,150 rad/s");
        assert_eq!(
            P.si(3.0e-16, "rad/s", 4, Prefixes::Small),
            "3.000e-16 rad/s"
        );
        assert_eq!(P.si(4.2e15, "Hz", 4, Prefixes::All), "4,200 THz");
        assert_eq!(P.si(0.0, "rad/s", 4, Prefixes::Small), "0.000 rad/s");
    }

    #[test]
    fn test_localize_swaps_marks_inside_numbers_and_nowhere_else() {
        let text = "dr/dt = -5.44c (γ 1.00, γv 12,345.6c). E.g. 1e-9, 2.5e9.";
        assert_eq!(
            C.localize(text),
            "dr/dt = -5,44c (γ 1,00, γv 12.345,6c). E.g. 1e-9, 2,5e9."
        );
        assert!(matches!(P.localize(text), Cow::Borrowed(_)));
        assert!(matches!(C.localize("no numbers here"), Cow::Borrowed(_)));
    }

    #[test]
    fn test_parse_reads_either_style_back() {
        assert_eq!(P.parse("1,234.5"), Some(1234.5));
        assert_eq!(P.parse(" -2.5 "), Some(-2.5));
        assert_eq!(P.parse("+1.5e9"), Some(1.5e9));
        assert_eq!(P.parse("−3"), Some(-3.0));
        assert_eq!(C.parse("1.234,5"), Some(1234.5));
        assert_eq!(C.parse("1,5"), Some(1.5));
        assert_eq!(C.parse("1,5e9"), Some(1.5e9));
        assert_eq!(
            C.parse("2.5"),
            Some(2.5),
            "a lone point next to a short fraction is a decimal"
        );
        assert_eq!(
            C.parse("1.234"),
            Some(1234.0),
            "a point before three digits is a group"
        );
        assert_eq!(P.parse("abc"), None);
        assert_eq!(P.parse("NaN"), None);
    }

    #[test]
    fn test_what_is_printed_parses_back_to_the_printed_value() {
        for style in [P, C] {
            for x in [0.0, -5.44, 1234.5, -61_440.0, 1.5e9, 2.418e-4, 3.0e-10] {
                let printed = style.significant(x, 4);
                let back = style
                    .parse(&printed)
                    .unwrap_or_else(|| panic!("{printed:?} did not parse"));
                assert!(
                    (back - x).abs() <= 1e-3 * x.abs(),
                    "{x} printed {printed:?}, read {back}"
                );
            }
        }
    }

    #[test]
    fn test_a_list_separator_is_never_a_decimal_mark() {
        assert_eq!(P.list_separator(), ", ");
        assert_eq!(C.list_separator(), "; ");
    }
}
