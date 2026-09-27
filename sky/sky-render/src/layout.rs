//! Where each character of a panel goes: three columns, label, value and unit, one line per
//! read-out.
//!
//! ```text
//!   Stopwatch    4.57 s
//!   Radius      12.345 M
//! ```
//!
//! A still can get away with numbers set in proportional figures; a video cannot. Each frame's
//! number is set afresh, and if a 1 is narrower than a 4 the digits and the unit after them
//! shuffle sideways from frame to frame, a shimmer the eye goes straight to. So:
//!
//! - every digit is set in a cell of one fixed width, the widest digit's advance, and centred in
//!   it (tabular figures, made from the font's proportional ones);
//! - the values are right-aligned to one column edge, so that with a fixed number of decimals the
//!   decimal mark and every digit keep their places as the value changes, `9.99` becoming `10.00`
//!   included: the new digit appears on the left;
//! - the column's width is fixed for the whole run, from the widest value the run will show, so
//!   the unit never moves either, and the panel never changes size (its place on the sphere is
//!   computed once, see `crate::panel`).
//!
//! Everything here is in panel pixels and knows nothing of fonts beyond [`Metrics`].

/// What layout needs to know about the typeface at the size it is set in, in panel pixels.
pub trait Metrics {
    /// How far the pen moves after `c`, in its proportional form.
    fn advance(&self, c: char) -> f32;
    /// The height of the tallest ascender above the baseline, positive.
    fn ascent(&self) -> f32;
    /// The depth of the deepest descender below the baseline, negative.
    fn descent(&self) -> f32;
    /// The width of a tabular digit cell: the widest digit's advance.
    fn digit_width(&self) -> f32 {
        ('0'..='9').map(|d| self.advance(d)).fold(0.0, f32::max)
    }
}

/// One character, placed: its pen position on the baseline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placed {
    pub c: char,
    pub x: f32,
    pub baseline: f32,
}

/// The panel's size and the positions of its columns and lines.
#[derive(Debug, Clone, PartialEq)]
pub struct Layout {
    pub width: usize,
    pub height: usize,
    /// The radius of the panel's rounded corners.
    pub radius: f64,
    label_x: f32,
    /// The right edge of the value column.
    value_right: f32,
    unit_x: f32,
    baselines: Vec<f32>,
}

/// Proportions of the panel, as fractions of the line pitch.
const MARGIN_ACROSS: f32 = 0.5;
const MARGIN_DOWN: f32 = 0.3;
const LABEL_GAP: f32 = 0.7;
const CORNER: f32 = 0.3;

/// The width `text` takes, digits at the tabular width.
pub fn width_of(m: &impl Metrics, text: &str) -> f32 {
    let digit = m.digit_width();
    text.chars()
        .map(|c| {
            if c.is_ascii_digit() {
                digit
            } else {
                m.advance(c)
            }
        })
        .sum()
}

/// `text` set from pen position `x` on `baseline`, digits centred in their cells.
fn set(m: &impl Metrics, text: &str, mut x: f32, baseline: f32, out: &mut Vec<Placed>) {
    let digit = m.digit_width();
    for c in text.chars() {
        if c.is_ascii_digit() {
            let advance = m.advance(c);
            out.push(Placed {
                c,
                x: x + 0.5 * (digit - advance),
                baseline,
            });
            x += digit;
        } else {
            out.push(Placed { c, x, baseline });
            x += m.advance(c);
        }
    }
}

impl Layout {
    /// A panel for lines with these `labels` and `units`, whose values are never wider than
    /// `widest_value`, set `pitch` panel pixels from baseline to baseline.
    pub fn new(
        m: &impl Metrics,
        pitch: f32,
        labels: &[&str],
        units: &[&str],
        widest_value: f32,
    ) -> Self {
        let widest = |texts: &[&str]| texts.iter().map(|t| width_of(m, t)).fold(0.0, f32::max);
        let (label_w, unit_w) = (widest(labels), widest(units));
        let margin = MARGIN_ACROSS * pitch;
        let label_x = margin;
        let gap = if label_w > 0.0 {
            LABEL_GAP * pitch
        } else {
            0.0
        };
        let value_right = label_x + label_w + gap + widest_value;
        // A unit follows its number after a space, as in running text.
        let unit_x = if unit_w > 0.0 {
            value_right + m.advance(' ')
        } else {
            value_right
        };
        let width = (unit_x + unit_w + margin).ceil() as usize;
        let top = MARGIN_DOWN * pitch;
        let lines = labels.len();
        let height = (2.0 * top + lines as f32 * pitch).ceil() as usize;
        // The text's own height centred in each line's pitch.
        let (ascent, descent) = (m.ascent(), m.descent());
        let baselines = (0..lines)
            .map(|k| top + k as f32 * pitch + 0.5 * (pitch - (ascent - descent)) + ascent)
            .collect();
        Self {
            width,
            height,
            radius: f64::from(CORNER * pitch),
            label_x,
            value_right,
            unit_x,
            baselines,
        }
    }

    /// The characters that do not change during a run: every line's label and unit.
    pub fn fixed_text(&self, m: &impl Metrics, labels: &[&str], units: &[&str]) -> Vec<Placed> {
        let mut out = Vec::new();
        for (k, (label, unit)) in labels.iter().zip(units).enumerate() {
            set(m, label, self.label_x, self.baselines[k], &mut out);
            set(m, unit, self.unit_x, self.baselines[k], &mut out);
        }
        out
    }

    /// The characters of a value on line `line`, right-aligned to the value column.
    pub fn value(&self, m: &impl Metrics, line: usize, text: &str) -> Vec<Placed> {
        let mut out = Vec::with_capacity(text.len());
        let x = self.value_right - width_of(m, text);
        set(m, text, x, self.baselines[line], &mut out);
        out
    }

    #[cfg(test)]
    /// The pen position at which every line's unit starts.
    pub fn unit_x(&self) -> f32 {
        self.unit_x
    }
}
