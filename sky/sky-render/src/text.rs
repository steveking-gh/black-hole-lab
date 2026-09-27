//! The app's typefaces, and text drawn with them into a coverage image.
//!
//! Atkinson Hyperlegible is the app's text face; DejaVu Sans stands in for any character it lacks
//! (the Greek letters a unit may use, the infinity sign), as it does in the app. Both are compiled
//! in, so the renderer needs no fonts on the machine it runs on. Their licences are in
//! assets/fonts/ and THIRD-PARTY-NOTICES.md.
//!
//! `ab_glyph` turns each glyph's outline into exact area coverage per pixel, which is its
//! antialiasing; glyphs are placed at fractional pen positions, not snapped to whole pixels, so
//! the spacing is the font's own.

use ab_glyph::{Font, FontRef, GlyphId, PxScale, ScaleFont, point};

use crate::layout::{Metrics, Placed};

static ATKINSON: &[u8] = include_bytes!("../../../assets/fonts/AtkinsonHyperlegible-Regular.ttf");
static DEJAVU: &[u8] = include_bytes!("../../../assets/fonts/DejaVuSans.ttf");

/// The two faces at one size.
pub struct Fonts {
    main: FontRef<'static>,
    fallback: FontRef<'static>,
    main_scale: PxScale,
    /// The scale that gives the fallback face the same em as the main face: `PxScale` sets a
    /// face's ascent-to-descent height, and the two faces divide their em differently.
    fallback_scale: PxScale,
}

impl Fonts {
    /// The faces at `px`: the main face's ascent-to-descent height, in pixels.
    pub fn new(px: f32) -> Self {
        let main = FontRef::try_from_slice(ATKINSON).expect("the embedded Atkinson face parses");
        let fallback = FontRef::try_from_slice(DEJAVU).expect("the embedded DejaVu face parses");
        let em_height = |f: &FontRef| f.height_unscaled() / f.units_per_em().unwrap_or(1000.0);
        let fallback_px = px * em_height(&fallback) / em_height(&main);
        Self {
            main,
            fallback,
            main_scale: PxScale::from(px),
            fallback_scale: PxScale::from(fallback_px),
        }
    }

    /// The face that has `c`, its glyph there, and that face's scale. A character neither face
    /// has is drawn as the main face's missing-glyph box, so that it shows rather than vanishes.
    fn face(&self, c: char) -> (&FontRef<'static>, GlyphId, PxScale) {
        let id = self.main.glyph_id(c);
        if id.0 != 0 {
            return (&self.main, id, self.main_scale);
        }
        let id = self.fallback.glyph_id(c);
        if id.0 != 0 {
            (&self.fallback, id, self.fallback_scale)
        } else {
            (&self.main, GlyphId(0), self.main_scale)
        }
    }

    #[cfg(test)]
    /// Whether the main face, rather than the fallback, draws `c`.
    pub fn main_face_has(&self, c: char) -> bool {
        self.main.glyph_id(c).0 != 0
    }
}

impl Metrics for Fonts {
    fn advance(&self, c: char) -> f32 {
        let (face, id, scale) = self.face(c);
        face.as_scaled(scale).h_advance(id)
    }

    fn ascent(&self) -> f32 {
        self.main.as_scaled(self.main_scale).ascent()
    }

    fn descent(&self) -> f32 {
        self.main.as_scaled(self.main_scale).descent()
    }
}

/// A grey image of how much of each pixel is covered, in [0, 1], rows from the top.
#[derive(Debug, Clone, PartialEq)]
pub struct Coverage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
}

impl Coverage {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![0.0; width * height],
        }
    }

    /// Draws `glyphs` over what is there, adding coverage and saturating at 1. Anything outside
    /// the image is cut off.
    pub fn draw(&mut self, fonts: &Fonts, glyphs: &[Placed]) {
        for g in glyphs {
            let (face, id, scale) = fonts.face(g.c);
            let glyph = id.with_scale_and_position(scale, point(g.x, g.baseline));
            let Some(outline) = face.outline_glyph(glyph) else {
                continue; // a space
            };
            let bounds = outline.px_bounds();
            let (left, top) = (bounds.min.x as i64, bounds.min.y as i64);
            outline.draw(|x, y, c| {
                let (px, py) = (left + i64::from(x), top + i64::from(y));
                if px >= 0 && py >= 0 && (px as usize) < self.width && (py as usize) < self.height {
                    let at = py as usize * self.width + px as usize;
                    self.data[at] = (self.data[at] + c).min(1.0);
                }
            });
        }
    }

    /// The bilinear value at image coordinates (x, y), pixel centres at half-integers, 0 outside.
    pub fn sample(&self, x: f32, y: f32) -> f32 {
        let (fx, fy) = (x - 0.5, y - 0.5);
        let (x0, y0) = (fx.floor(), fy.floor());
        let (ax, ay) = (fx - x0, fy - y0);
        let (x0, y0) = (x0 as i64, y0 as i64);
        let at = |i: i64, j: i64| {
            if i < 0 || j < 0 || i >= self.width as i64 || j >= self.height as i64 {
                0.0
            } else {
                self.data[j as usize * self.width + i as usize]
            }
        };
        let top = at(x0, y0) + ax * (at(x0 + 1, y0) - at(x0, y0));
        let bottom = at(x0, y0 + 1) + ax * (at(x0 + 1, y0 + 1) - at(x0, y0 + 1));
        top + ay * (bottom - top)
    }
}
