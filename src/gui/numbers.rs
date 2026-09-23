//! How the info boxes print a number.
//!
//! One rule for every box, so that no two cards disagree about when a number turns into an
//! exponent. Plain notation up to a billion: a reader holds 452 or 61,440 in their head and does
//! not hold 4.52e2 or 6.144e4, and an exponent is kept for the values past 1e9 that no plain
//! notation can fit in a box a few characters wide. At the small end a quantity with a unit takes
//! an SI prefix instead (`rad_per_second`, and `format_frequency` for hertz), and a pure ratio
//! prints in plain decimals down to a millionth before it too gives way to an exponent.

/// Above this magnitude a number is printed as an exponent.
pub(crate) const EXPONENT_ABOVE: f64 = 1e9;

/// Below this magnitude a pure ratio is printed as an exponent: six zeroes after the point is as
/// many as an eye counts.
pub(crate) const EXPONENT_BELOW: f64 = 1e-6;

/// `x` at `decimals` places, the way a box prints a number of ordinary size.
///
/// From ten thousand up the decimals are dropped and the digits grouped in threes - 61,440 and
/// 1,355,000,000 are read at a glance where 61440.00 is counted - and past `EXPONENT_ABOVE` the
/// value is an exponent. The caller keeps its own format below that, so this touches nothing a
/// box already printed well.
pub(crate) fn fixed(x: f64, decimals: usize) -> String {
    if !x.is_finite() {
        return if x.is_nan() { "n/a".to_string() } else if x > 0.0 { "∞".to_string() } else { "-∞".to_string() };
    }
    let mag = x.abs();
    if mag > EXPONENT_ABOVE {
        format!("{x:.2e}")
    } else if mag >= 1e4 {
        let sign = if x < 0.0 { "-" } else { "" };
        format!("{sign}{}", grouped(mag))
    } else {
        format!("{x:.decimals$}")
    }
}

/// `fixed` with a + in front of a value that is not negative, for the signed rates.
pub(crate) fn fixed_signed(x: f64, decimals: usize) -> String {
    signed(x, fixed(x, decimals))
}

/// A small pure ratio at four significant digits in plain decimals - 0.0002418 - down to
/// `EXPONENT_BELOW`, and as an exponent below that. Zero is "0".
pub(crate) fn small(x: f64) -> String {
    let mag = x.abs();
    if mag == 0.0 {
        return "0".to_string();
    }
    if mag < EXPONENT_BELOW {
        return format!("{x:.3e}");
    }
    let decimals = (3.0 - mag.log10().floor()).max(0.0) as usize;
    format!("{x:.decimals$}")
}

/// `small` with a + in front of a value that is not negative.
pub(crate) fn small_signed(x: f64) -> String {
    signed(x, small(x))
}

/// An angular velocity in radians per second, signed, at four significant digits with an SI
/// prefix: +241.8 mrad/s, -3.204 µrad/s. The span is wide - a stellar-mass hole's horizon turns
/// ten thousand radians a second and a supermassive one's far field a few picoradians - and a
/// prefix per decade of a thousand keeps every one of them to a handful of characters. Above a
/// radian per second it is `fixed`, and below a picoradian per second an exponent.
pub(crate) fn rad_per_second(per_s: f64) -> String {
    const PREFIXES: [(&str, f64); 4] = [("m", 1e-3), ("µ", 1e-6), ("n", 1e-9), ("p", 1e-12)];
    let mag = per_s.abs();
    if mag == 0.0 || !mag.is_finite() {
        return format!("{} rad/s", fixed_signed(per_s, 3));
    }
    // Rounded to four significant digits first, so that a value which rounds up to the next
    // power of a thousand takes the larger unit rather than printing as 1000.
    let quantum = 10f64.powf(mag.log10().floor() - 3.0);
    let rounded = (mag / quantum).round() * quantum;
    if rounded >= 1.0 {
        let decimals = (3.0 - rounded.log10().floor()).max(0.0) as usize;
        return format!("{} rad/s", fixed_signed(per_s, decimals));
    }
    let Some(&(prefix, scale)) = PREFIXES.iter().find(|(_, scale)| rounded >= *scale) else {
        return format!("{per_s:+.3e} rad/s");
    };
    let mantissa = per_s / scale;
    let decimals = (3.0 - (rounded / scale).log10().floor()).max(0.0) as usize;
    format!("{mantissa:+.decimals$} {prefix}rad/s")
}

/// A + in front of `text` when `x` is not negative; `text` is `x` already formatted.
fn signed(x: f64, text: String) -> String {
    if x < 0.0 || text.starts_with('-') { text } else { format!("+{text}") }
}

/// The integer part of a non-negative value with its digits grouped in threes by commas.
fn grouped(mag: f64) -> String {
    let digits = format!("{:.0}", mag);
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, ch) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(ch);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_exponent_until_a_billion() {
        assert_eq!(fixed(-452.004, 2), "-452.00");
        assert_eq!(fixed(61_440.4, 2), "61,440");
        assert_eq!(fixed(-1.355e8, 2), "-135,500,000");
        assert_eq!(fixed(1e9, 2), "1,000,000,000");
        assert_eq!(fixed(1.5e9, 2), "1.50e9");
        assert_eq!(fixed_signed(0.0, 2), "+0.00");
        assert_eq!(fixed_signed(-0.004, 2), "-0.00");
        assert_eq!(fixed(f64::INFINITY, 2), "∞");
    }

    #[test]
    fn test_small_ratios_stay_decimal_to_a_millionth() {
        assert_eq!(small(2.418e-4), "0.0002418");
        assert_eq!(small(0.009), "0.009000");
        assert_eq!(small(1.2e-6), "0.000001200");
        assert_eq!(small(3.0e-10), "3.000e-10");
        assert_eq!(small_signed(1.0e-5), "+0.00001000");
        assert_eq!(small(0.0), "0");
    }

    #[test]
    fn test_angular_velocity_takes_a_prefix() {
        assert_eq!(rad_per_second(-0.2418), "-241.8 mrad/s");
        assert_eq!(rad_per_second(5.38e-2), "+53.80 mrad/s");
        assert_eq!(rad_per_second(3.204e-6), "+3.204 µrad/s");
        assert_eq!(rad_per_second(2.0e-11), "+20.00 prad/s");
        assert_eq!(rad_per_second(0.99996), "+1.000 rad/s");
        assert_eq!(rad_per_second(10_150.0), "+10,150 rad/s");
        assert_eq!(rad_per_second(0.0), "+0.000 rad/s");
        assert_eq!(rad_per_second(3.0e-16), "+3.000e-16 rad/s");
    }
}
