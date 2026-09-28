//! The read-outs as a subtitle track, which the player draws on its window rather than on the
//! sphere.
//!
//! A 360-degree video cannot hold anything fixed on the screen: every pixel of it is a direction
//! on the sphere, and a panel painted into it (`crate::overlay`) is seen only when the viewer
//! looks that way. A player draws subtitles in screen space, over whatever part of the sphere is
//! in view, so the read-outs go into an ASS subtitle track: one cue per video frame, aligned to
//! the bottom right of the window. The owner saw a hand-made trial of exactly this in VLC 3.0.23
//! and asked for it at half the trial's size, which is the default here.
//!
//! # The script
//!
//! An ASS script states its own resolution (`PlayResX`, `PlayResY`), and the player scales every
//! size and margin in it by the height of the window over `PlayResY`. The script here is 1920 by
//! 1080, as the trial's was, so a font size of 18 is 18 / 1080 = 1.67 % of the window's height,
//! whatever the window. A size in ASS is roughly the height of a line of text (libass, which
//! VLC uses, sets it as the typeface's ascent plus descent), so `--overlay-size` is the height of
//! one line in percent of the screen's height, and the approved size is 1.67.
//!
//! The style is the trial's: light grey text (the panel's 240 of 255) in an opaque-box border
//! style (`BorderStyle` 3), whose box is padded by the `Outline` width, 2 at size 18 and scaled
//! with the size; alignment 3, bottom right; margins of 40 script pixels. `WrapStyle` 2 turns
//! the player's automatic line wrapping off, so that a line breaks only where the script breaks
//! it.
//!
//! # Digits that do not jitter
//!
//! The panel sets each digit in a cell of one width (`crate::layout`); a subtitle can do that
//! only by naming a typeface in which every character already has one width, and padding the
//! columns with spaces. Every line is the same number of characters long: the label padded to the
//! widest label, the value padded on the left to the widest value any cue shows (so that it is
//! right-aligned, and a digit keeps its place as the value grows), the unit padded to the widest
//! unit. The lines are right-aligned by the style, so in a monospaced face they are then aligned
//! column for column. The padding is written as `\h`, the hard space, not as a plain space: the
//! player trims plain spaces from the end of a line before aligning it, which would throw a line
//! with a shorter unit out of the column.
//!
//! ASS names one typeface per style and has no list of alternatives (the style line's fields are
//! separated by commas, so a list could not even be written there). The player looks the name up
//! among the machine's fonts and, when it is missing, substitutes its default face, which is
//! proportional: the digits of most such faces are still all one width, but spaces are narrower
//! than digits, so the columns would no longer line up exactly.
//!
//! The default is Consolas, because that is the face of the trial the owner looked at and
//! approved, and what was approved is what is made. It ships with Windows and with nothing else.
//! The monospaced face most likely to be found under its own name everywhere is Courier New: it
//! ships with Windows and with macOS, and on Linux fontconfig's standard metric aliases map the
//! name to Liberation Mono (or Cousine), which most desktops install, and libass follows those
//! aliases. `--overlay-font "Courier New"` is the choice for a film that will be watched on other
//! machines; it is a thinner face than Consolas.
//!
//! # Times
//!
//! ASS writes a time as H:MM:SS.cc, in hundredths of a second. At 30 frames a second a frame
//! lasts 3 1/3 hundredths, so frame boundaries cannot be written exactly: each is written as the
//! nearest hundredth to its true time k / fps, at most half a hundredth (5 ms, 0.15 of a
//! frame) off. A cue runs from its frame's boundary to the next frame's, both rounded the same
//! way, so consecutive cues share their boundary exactly: none overlaps the next and there is no
//! gap between them. The cues last 3 or 4 hundredths in the pattern 3, 4, 3, 3, 4, 3, ... Above
//! 100 frames a second two boundaries can round to the same hundredth, and the cue between them
//! would last no time and never be shown; such a cue is left out.
//!
//! # What a manifest cannot do
//!
//! Labels and units come from the bundle's manifest, and ASS gives meaning to some characters in
//! a cue's text: `{` opens a block of style overrides, and a backslash followed by `N`, `n` or
//! `h` is a line break or a hard space. A label must be shown as it is and must not restyle the
//! cue, so [`escape`] writes a brace as `\{` or `\}` (libass's escapes for a literal brace), puts
//! a zero-width word joiner (U+2060) after every backslash so that no backslash and letter after
//! it can read as an escape, and turns line breaks, tabs and other control characters into
//! spaces, since a line break in the file would end the cue's line early.

use std::ops::Range;

use readout::Style;

use crate::timeline::Timeline;
use crate::values::{Line, Series, format};

/// The typeface named when `--overlay-font` is not given.
pub const DEFAULT_FONT: &str = "Consolas";

/// The script's resolution: the trial's, a 16:9 screen of 1080 lines.
pub const SCRIPT_WIDTH: u32 = 1920;
pub const SCRIPT_HEIGHT: u32 = 1080;

/// The approved font size, at the script's height of 1080.
const APPROVED_SIZE: f64 = 18.0;

/// `--overlay-size` by default: the approved size as a percentage of the screen's height, 1.67.
pub const DEFAULT_PERCENT: f64 = 100.0 * APPROVED_SIZE / SCRIPT_HEIGHT as f64;

/// The padding of the text's box at the approved size; it scales with the size.
const APPROVED_PADDING: f64 = 2.0;

/// The distance from the window's edges, in script pixels.
const MARGIN: u32 = 40;

/// The name of the one style.
const STYLE: &str = "Readout";

/// The hard space, which the player does not trim from the end of a line.
const HARD_SPACE: &str = "\\h";

/// How the read-outs are drawn.
#[derive(Debug, Clone, PartialEq)]
pub struct Look {
    pub font: String,
    /// The height of a line, in percent of the screen's height.
    pub percent: f64,
    /// The decimal mark.
    pub style: Style,
}

/// One cue: from `start` to `end`, in hundredths of a second, showing each line's value, None for
/// a line the cue leaves out.
#[derive(Debug, Clone, PartialEq)]
pub struct Cue {
    pub start: u64,
    pub end: u64,
    pub values: Vec<Option<f64>>,
}

/// The font size for a line `percent` of the screen's height, on the script's 1080 lines.
pub fn font_size(percent: f64) -> f64 {
    percent / 100.0 * f64::from(SCRIPT_HEIGHT)
}

/// A number for a style field: at most two decimals, and none that are zero (`18`, `2.4`).
fn field(x: f64) -> String {
    let text = format!("{x:.2}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// The style line: the typeface, the size, light text in a dark box, bottom right.
pub fn style_line(look: &Look) -> String {
    let size = font_size(look.percent);
    let padding = APPROVED_PADDING * size / APPROVED_SIZE;
    // Fields: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour,
    // BackColour (colours &HAABBGGRR, alpha 0 opaque), Bold, Italic, Underline, StrikeOut,
    // ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment (3: bottom right,
    // as on a numeric keypad), MarginL, MarginR, MarginV, Encoding.
    format!(
        "Style: {STYLE},{},{},&H00F0F0F0,&H00F0F0F0,&H00000000,&H99000000,0,0,0,0,100,100,0,0,3,{},\
         0,3,{MARGIN},{MARGIN},{MARGIN},1",
        look.font,
        field(size),
        field(padding)
    )
}

/// `seconds` to the nearest hundredth, as ASS counts time.
pub fn centiseconds(seconds: f64) -> u64 {
    (seconds * 100.0).round() as u64
}

/// A time in hundredths of a second as ASS writes it: `H:MM:SS.cc`.
pub fn time(centiseconds: u64) -> String {
    let c = centiseconds;
    format!(
        "{}:{:02}:{:02}.{:02}",
        c / 360_000,
        c / 6_000 % 60,
        c / 100 % 60,
        c % 100
    )
}

/// The start and end of the cue of each of `count` video frames at `frames_per_second`, counted
/// from the video's start: each boundary the nearest hundredth to its true time, so that each
/// cue ends where the next begins.
pub fn frame_times(count: u64, frames_per_second: f64) -> Vec<(u64, u64)> {
    let boundary = |k: u64| centiseconds(k as f64 / frames_per_second);
    (0..count).map(|k| (boundary(k), boundary(k + 1))).collect()
}

/// The cues of video frames `frames` of a film at `frames_per_second`, timed from the first of
/// them, each showing the values the panel would paint on its frame (`crate::values`: the
/// stopwatch exact from the timeline, the others interpolated between bundle frames).
pub fn film_cues(
    series: &Series,
    timeline: &Timeline,
    frames: Range<u64>,
    frames_per_second: f64,
) -> Vec<Cue> {
    frame_times(frames.end - frames.start, frames_per_second)
        .into_iter()
        .zip(frames)
        .map(|((start, end), k)| Cue {
            start,
            end,
            values: series.at(timeline.pick(k), timeline.stopwatch(k)),
        })
        .collect()
}

/// The one cue of a still of video frame `k`, held `seconds`: its values for the whole length.
pub fn still_cue(series: &Series, timeline: &Timeline, k: u64, seconds: f64) -> Cue {
    Cue {
        start: 0,
        end: centiseconds(seconds),
        values: series.at(timeline.pick(k), timeline.stopwatch(k)),
    }
}

/// `text` as a cue shows it literally: no override block, no escape, on one line.
pub fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '{' => out.push_str("\\{"),
            '}' => out.push_str("\\}"),
            '\\' => out.push_str("\\\u{2060}"),
            c if c.is_control() => out.push(' '),
            c => out.push(c),
        }
    }
    out
}

/// `text` escaped, with `width - its length` hard spaces before it (`right`) or after it.
fn padded(text: &str, width: usize, right: bool) -> String {
    let pad = HARD_SPACE.repeat(width.saturating_sub(text.chars().count()));
    if right {
        pad + &escape(text)
    } else {
        escape(text) + &pad
    }
}

/// The whole script: the header, the style, and one dialogue line per cue that lasts any time.
pub fn script(lines: &[Line], cues: &[Cue], look: &Look) -> String {
    let widest = |texts: &mut dyn Iterator<Item = usize>| texts.max().unwrap_or(0);
    let label_width = widest(&mut lines.iter().map(|l| l.label.chars().count()));
    let unit_width = widest(&mut lines.iter().map(|l| l.unit.chars().count()));
    let text = |l: usize, x: f64| format(look.style, x, lines[l].decimals);
    let value_width = widest(&mut cues.iter().flat_map(|cue| {
        cue.values
            .iter()
            .enumerate()
            .filter_map(|(l, v)| v.map(|x| text(l, x).chars().count()))
            .collect::<Vec<_>>()
    }));

    let mut out = String::new();
    // A byte-order mark: VLC reads a sidecar file without it by guessing its encoding, and the
    // infinity sign and a unit's Greek letters are not ASCII.
    out.push('\u{feff}');
    out.push_str(&format!(
        "[Script Info]\n\
         ; The read-outs of a Black Hole Lab sky film, one cue per video frame, written by \
         sky-render {}.\n\
         Title: Read-outs\n\
         ScriptType: v4.00+\n\
         WrapStyle: 2\n\
         PlayResX: {SCRIPT_WIDTH}\n\
         PlayResY: {SCRIPT_HEIGHT}\n\
         ScaledBorderAndShadow: yes\n\
         YCbCr Matrix: None\n\
         \n\
         [V4+ Styles]\n\
         Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, \
         BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, \
         BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n\
         {}\n\
         \n\
         [Events]\n\
         Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n",
        env!("CARGO_PKG_VERSION"),
        style_line(look)
    ));
    for cue in cues.iter().filter(|c| c.end > c.start) {
        let shown: Vec<String> = cue
            .values
            .iter()
            .enumerate()
            .filter_map(|(l, v)| {
                let x = (*v)?;
                let line = &lines[l];
                let mut s = String::new();
                if label_width > 0 {
                    s += &padded(&line.label, label_width, false);
                    s += &HARD_SPACE.repeat(2);
                }
                s += &padded(&text(l, x), value_width, true);
                if unit_width > 0 {
                    s += HARD_SPACE;
                    s += &padded(&line.unit, unit_width, false);
                }
                Some(s)
            })
            .collect();
        out.push_str(&format!(
            "Dialogue: 0,{},{},{STYLE},,0,0,0,,{}\n",
            time(cue.start),
            time(cue.end),
            shown.join("\\N")
        ));
    }
    out
}
