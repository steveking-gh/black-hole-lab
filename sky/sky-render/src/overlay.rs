//! The read-out panels, painted over each finished frame.
//!
//! Set up once per run: the typefaces at the size the panel has in the output frame, the layout
//! (whose value column is as wide as the widest value the run will show), the labels and units
//! drawn once into a coverage image, and for each panel the list of output pixels it covers and
//! where on the panel each one reads (`crate::panel`). Per frame only the values are drawn, into
//! a copy of that image, and composited through each panel's list.
//!
//! # Resolution
//!
//! A panel pixel is as long, at the panel's centre, as one row of the output frame: 2 pi / W of
//! the tangent plane, since an equirectangular frame twice as wide as high has square pixels in
//! angle at the equator. Across, the frame's pixels are narrower than that away from the equator
//! (by cos e at elevation e), so there the panel is magnified slightly (1.15 times at the default
//! elevation of -30) and never minified: magnifying softens a little, minifying would alias, and
//! aliasing is what a video shows up.
//!
//! # Look and blending
//!
//! Light text on a dark, partly transparent backing: over empty sky the backing is nearly black
//! and changes nothing a viewer notices, and over the Milky Way it darkens the stars enough for
//! the text to stand out. The panel is an overlay and not light: it is composited after the tone
//! curve, so neither the exposure nor a shift's g^4 touches it, and its colours are the codes
//! written here.
//!
//! The blend is done on the frame's sRGB-encoded 16-bit codes, not on linear light. For light that
//! would be wrong (a half-covered pixel of a star should carry half its light, not half its code),
//! but for an overlay it only shapes the antialiased edges of the glyphs and of the backing: in
//! encoded space an edge pixel of light text on a dark backing looks a little darker, so the text
//! reads marginally thinner than a linear blend would draw it. That is how most user interfaces,
//! the app's own included, composite text, and the fonts are designed to be seen that way.

use readout::Style;

use crate::cli::Options;
use crate::layout::{Layout, width_of};
use crate::panel::{Panel, Placement, Tap};
use crate::render::Size;
use crate::shadow::Outline;
use crate::text::{Coverage, Fonts};
use crate::values::{Line, Series, format};

/// What the command line says about the panels.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// One panel at each of these; all show the same text.
    pub placements: Vec<Placement>,
    /// The line pitch, as an angle in degrees.
    pub line_degrees: f64,
    pub style: Style,
}

/// The text's colour, sRGB-encoded 16-bit codes: a light grey rather than white, which on a
/// dark backing reads as white without glaring.
const TEXT: [f32; 3] = [61_680.0, 61_680.0, 61_680.0]; // 240 of 255
/// How much of the sky the backing hides: black at this opacity.
const BACKING: f32 = 0.6;
/// The line pitch over the typeface's ascent-to-descent height.
const LINE_SPACING: f32 = 1.25;
/// The largest angle from a panel's centre to its corner; beyond it the gnomonic projection
/// stretches the panel's edges more than a reader would accept, and a panel that large is a
/// mistake in `--readout-size`.
const LARGEST_RADIUS_DEGREES: f64 = 60.0;

pub struct Overlay {
    fonts: Fonts,
    layout: Layout,
    lines: Vec<Line>,
    style: Style,
    /// The labels and units, drawn once.
    fixed: Coverage,
    /// This frame's panel: `fixed` and the values.
    raster: Coverage,
    /// For each panel, the output pixels it covers.
    panels: Vec<Vec<Tap>>,
    /// Which of the run's read-outs the panel shows, as positions in the manifest's list: all of
    /// them, except on a still's panel (see [`shown_lines`]).
    shown: Vec<usize>,
}

/// The read-outs a panel shows, as positions in the manifest's list. A film's panel (`still`
/// None) shows them all, and a line without a value on some frame is drawn there without one. A
/// still's, whose values at its one frame are `still`, leaves out altogether, label and all, the
/// stopwatch, for the reason `crate::subtitles::still_cue` gives (it counts the time since a
/// film's first frame, and a still is one moment), and every read-out that has no value at that
/// frame: a writer declares some read-outs that exist only at some frames, such as a speed past a
/// reference observer who exists only outside the static limit. A line is left out rather than
/// left blank, so the panel is no larger than what it says.
pub fn shown_lines(series: &Series, still: Option<&[Option<f64>]>) -> Vec<usize> {
    (0..series.lines.len())
        .filter(|&l| match still {
            None => true,
            Some(values) => !series.lines[l].stopwatch && values[l].is_some(),
        })
        .collect()
}

/// What the command line says about the panels: where `--readout-at` puts them (for
/// `--readout-at dark`, `Placement::DEFAULT`, where the panel goes when the dark region cannot
/// hold it), their size and the decimal mark.
pub fn settings_of(o: &Options) -> Settings {
    Settings {
        placements: o.readout_at.clone(),
        line_degrees: o.readout_size,
        style: if o.decimal_comma {
            Style::COMMA
        } else {
            Style::POINT
        },
    }
}

impl Overlay {
    /// The panels a run's command line asks for, or None with `--readouts off`, which leaves
    /// every frame exactly as the renderer drew it. `still` holds a still's values at its frame
    /// (see [`shown_lines`]); it is None for a film.
    pub fn for_run(
        o: &Options,
        series: &Series,
        still: Option<&[Option<f64>]>,
        stopwatch_span: [f64; 2],
    ) -> Result<Option<Self>, String> {
        if !o.readouts {
            return Ok(None);
        }
        let shown = shown_lines(series, still);
        // A still's panel with nothing left to show is no panel at all.
        if o.still.is_some() && shown.is_empty() {
            return Ok(None);
        }
        let size = Size {
            width: o.width,
            height: o.height,
        };
        Self::showing(&settings_of(o), series, &shown, stopwatch_span, size).map(Some)
    }

    /// The panels for a frame of `size`, showing every read-out. `stopwatch_span` is the
    /// stopwatch time of the first and last video frames to be drawn, for the width of the value
    /// column.
    #[cfg(test)]
    pub fn new(
        settings: &Settings,
        series: &Series,
        stopwatch_span: [f64; 2],
        size: Size,
    ) -> Result<Self, String> {
        let all: Vec<usize> = (0..series.lines.len()).collect();
        Self::showing(settings, series, &all, stopwatch_span, size)
    }

    /// [`Overlay::new`], showing only the read-outs at positions `shown` of the manifest's list.
    pub fn showing(
        settings: &Settings,
        series: &Series,
        shown: &[usize],
        stopwatch_span: [f64; 2],
        size: Size,
    ) -> Result<Self, String> {
        let pixel = 2.0 * std::f64::consts::PI / size.width as f64;
        let pitch = (settings.line_degrees.to_radians() / pixel) as f32;
        let fonts = Fonts::new(pitch / LINE_SPACING);
        let lines: Vec<Line> = shown.iter().map(|&l| series.lines[l].clone()).collect();
        let style = settings.style;

        // The widest value any line will show, as displayed. Values between two bundle frames
        // lie between theirs, and so print no wider than the wider of the two.
        let mut widest: f32 = 0.0;
        for (line, &l) in lines.iter().zip(shown) {
            let values: Vec<f64> = if line.stopwatch {
                stopwatch_span.iter().map(|t| t * line.scale).collect()
            } else {
                series.values_of(l).collect()
            };
            for x in values {
                widest = widest.max(width_of(&fonts, &format(style, x, line.decimals)));
            }
        }
        let labels: Vec<&str> = lines.iter().map(|l| l.label.as_str()).collect();
        let units: Vec<&str> = lines.iter().map(|l| l.unit.as_str()).collect();
        let layout = Layout::new(&fonts, pitch, &labels, &units, widest);
        let mut fixed = Coverage::new(layout.width, layout.height);
        fixed.draw(&fonts, &layout.fixed_text(&fonts, &labels, &units));

        let mut panels = Vec::with_capacity(settings.placements.len());
        for &at in &settings.placements {
            let panel = Panel::new(at, pixel, layout.width, layout.height, layout.radius);
            let radius = panel.angular_radius().to_degrees();
            if radius > LARGEST_RADIUS_DEGREES {
                return Err(format!(
                    "at --readout-size {} the read-out panel reaches {radius:.0} degrees from its \
                     centre, more than {LARGEST_RADIUS_DEGREES}; choose a smaller size",
                    settings.line_degrees
                ));
            }
            if panel.reaches_pole() {
                return Err(format!(
                    "the read-out panel at elevation {} reaches over the pole at --readout-size \
                     {}; choose an elevation nearer the equator or a smaller size",
                    at.elevation, settings.line_degrees
                ));
            }
            panels.push(panel.cover(size));
        }
        let raster = fixed.clone();
        Ok(Self {
            fonts,
            layout,
            lines,
            style,
            fixed,
            raster,
            panels,
            shown: shown.to_vec(),
        })
    }

    /// Draws this frame's panel: the labels and units, and each line's value, or nothing on a
    /// line whose value is None. `values` holds one value per read-out of the manifest, shown or
    /// not.
    pub fn draw(&mut self, values: &[Option<f64>]) -> &Coverage {
        self.raster.data.copy_from_slice(&self.fixed.data);
        for (l, &from) in self.shown.iter().enumerate() {
            let Some(x) = values[from] else { continue };
            let text = format(self.style, x, self.lines[l].decimals);
            let glyphs = self.layout.value(&self.fonts, l, &text);
            self.raster.draw(&self.fonts, &glyphs);
        }
        &self.raster
    }

    /// Paints the panels, showing `values` (one per line, in the manifest's order), over a
    /// finished frame of 16-bit sRGB-encoded codes.
    pub fn paint(&mut self, frame: &mut [u16], values: &[Option<f64>]) {
        self.draw(values);
        for taps in &self.panels {
            composite(frame, taps, &self.raster);
        }
    }

    #[cfg(test)]
    pub fn layout(&self) -> &Layout {
        &self.layout
    }

    /// How many lines the panel shows.
    pub fn line_count(&self) -> usize {
        self.lines.len()
    }

    /// The panel's size and corners, for `crate::shadow` to fit it.
    pub fn outline(&self) -> Outline {
        Outline {
            width: self.layout.width,
            height: self.layout.height,
            radius: self.layout.radius,
        }
    }

    #[cfg(test)]
    pub fn fonts(&self) -> &Fonts {
        &self.fonts
    }
}

/// Composites a panel whose text covers `text` over the pixels `taps` lists: first the backing,
/// then the text over it, each scaled down by the tap's edge coverage at the panel's outline.
/// Pixels not in `taps` are not touched.
pub fn composite(frame: &mut [u16], taps: &[Tap], text: &Coverage) {
    for t in taps {
        let ink = text.sample(t.x, t.y) * t.edge;
        let keep = 1.0 - BACKING * t.edge;
        let at = 3 * t.pixel as usize;
        for (c, code) in frame[at..at + 3].iter_mut().enumerate() {
            let under = f32::from(*code) * keep;
            *code = (under + ink * (TEXT[c] - under) + 0.5) as u16;
        }
    }
}
