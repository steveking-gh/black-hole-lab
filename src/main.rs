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
        Box::new(|_cc| Ok(Box::new(SpacetimeApp::default()))),
    )
}
