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
            .with_title("Relativistic Spacetime & Cauchy Horizon Visualizer")
            .with_inner_size([1280.0, 820.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Relativistic Spacetime & Cauchy Horizon Visualizer",
        native_options,
        Box::new(|cc| {
            install_fonts(&cc.egui_ctx);
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
    let proportional = fonts.families.entry(FontFamily::Proportional).or_default();
    proportional.insert(0, "atkinson".to_owned());
    proportional.insert(1, "dejavu".to_owned());
    fonts.families.entry(FontFamily::Monospace).or_default().push("dejavu".to_owned());
    ctx.set_fonts(fonts);
}
