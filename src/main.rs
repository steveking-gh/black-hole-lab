// Black Hole Lab: a visualiser for Kerr geodesics and the Cauchy horizon.
// Copyright (C) 2026 Steve King
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If
// not, see <https://www.gnu.org/licenses/>. The full text is in LICENSE.TXT; the licences of the
// crates and fonts this program is built from are in THIRD-PARTY-NOTICES.md.

// Tensor expressions are written with their indices, because the index is the physics: a line
// like `sum += g[mu][nu] * u[mu] * u[nu]` is the formula it implements, and the iterator form
// clippy asks for hides which slot of the metric is being contracted with which component of the
// 4-velocity. Every one of these loops runs over the same fixed range 0..3, the equatorial
// (t, r, phi) chart, so there is no bounds-checking argument for the rewrite either.
#![allow(clippy::needless_range_loop)]

mod app;
mod gui;
mod physics;

use app::SpacetimeApp;
use eframe::NativeOptions;

fn main() -> eframe::Result<()> {
    // Native window options
    let native_options = NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Black Hole Lab")
            .with_inner_size([1280.0, 820.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Black Hole Lab",
        native_options,
        Box::new(|cc| {
            install_fonts(&cc.egui_ctx);
            install_images(&cc.egui_ctx);
            Ok(Box::new(SpacetimeApp::default()))
        }),
    )
}

/// egui's bundled proportional font has no subscripts, Greek or maths symbols, so r₋, τ, ν, ξ
/// and friends rendered as boxes. Atkinson Hyperlegible (SIL OFL) is the primary text face;
/// DejaVu Sans (Bitstream Vera licence) sits behind it as the symbol fallback for both families.
/// Licences are in assets/fonts.
fn install_fonts(ctx: &egui::Context) {
    use egui::{FontData, FontDefinitions, FontFamily};
    use std::sync::Arc;

    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert(
        "atkinson".to_owned(),
        Arc::new(FontData::from_static(include_bytes!("../assets/fonts/AtkinsonHyperlegible-Regular.ttf"))),
    );
    fonts.font_data.insert(
        "dejavu".to_owned(),
        Arc::new(FontData::from_static(include_bytes!("../assets/fonts/DejaVuSans.ttf"))),
    );
    fonts.font_data.insert(
        "atkinson-bold".to_owned(),
        Arc::new(FontData::from_static(include_bytes!("../assets/fonts/AtkinsonHyperlegible-Bold.ttf"))),
    );
    let proportional = fonts.families.entry(FontFamily::Proportional).or_default();
    proportional.insert(0, "atkinson".to_owned());
    proportional.insert(1, "dejavu".to_owned());
    fonts.families.entry(FontFamily::Monospace).or_default().push("dejavu".to_owned());
    // A bold face of its own, for the one line an info box has to shout: see
    // `gui::spacetime_canvas::BOLD_FAMILY`. DejaVu behind it for the symbols the face lacks.
    fonts.families.insert(
        FontFamily::Name(crate::gui::spacetime_canvas::BOLD_FAMILY.into()),
        vec!["atkinson-bold".to_owned(), "dejavu".to_owned()],
    );
    ctx.set_fonts(fonts);
}

/// Register the image loaders the transport icons are drawn through.
///
/// The icons are SVG files under `assets/images`, embedded at compile time by
/// `egui::include_image!` and rasterised by `egui_extras`' resvg loader at the size they are drawn
/// at. They used to be Unicode transport glyphs, and every one of them was a missing-glyph box:
/// neither Atkinson Hyperlegible nor DejaVu Sans carries U+23EE, U+23F8 or U+23EA, so the Reset and
/// Pause buttons rendered as empty rectangles. A drawing is a drawing and does not depend on what
/// a text face happens to cover.
fn install_images(ctx: &egui::Context) {
    egui_extras::install_image_loaders(ctx);
}
