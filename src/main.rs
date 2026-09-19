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
mod perf;
mod physics;
mod save;
mod stamp;
mod version;

use std::path::PathBuf;

use app::SpacetimeApp;
use eframe::NativeOptions;

fn main() -> eframe::Result<()> {
    // Everything headless is decided here, before a window is asked for. Two things are:
    //
    // `--perf` anywhere on the line runs the performance harness and exits with its status code -
    // the harness lives inside this binary because the crate has no library target and is not
    // growing one for it; see `perf` for the tiers and the A/B workflow.
    //
    // `--save-info <path>` prints what a save file holds and exits, which is how a directory of
    // them is told apart without opening each one in the app.
    //
    // What is left is a window, optionally opening on a save: a first argument that is not a flag
    // is a path to one. A save that will not load is reported on stderr and the app opens as it
    // otherwise would, because a bad path is not a reason to refuse to run.
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--perf") {
        std::process::exit(perf::run(&args));
    }
    if let Some(index) = args.iter().position(|arg| arg == "--save-info") {
        std::process::exit(match args.get(index + 1) {
            Some(path) => match save::describe(std::path::Path::new(path)) {
                Ok(report) => {
                    print!("{report}");
                    0
                }
                Err(message) => {
                    eprintln!("{path}: {message}");
                    2
                }
            },
            None => {
                eprintln!("--save-info: needs a path to a save file");
                2
            }
        });
    }
    let opening: Option<PathBuf> =
        args.first().filter(|arg| !arg.starts_with("--")).map(PathBuf::from);

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
        Box::new(move |cc| {
            install_fonts(&cc.egui_ctx);
            let mut app = SpacetimeApp::default();
            if let Some(path) = opening.as_ref() {
                // Through exactly the path the Load button takes, so that a window opened on a
                // save comes up with the same line under the buttons that a load from inside the
                // app leaves - and, when the file will not open, says why there rather than only on
                // a stderr nobody launching from a desktop icon ever sees. Nothing is autosaved:
                // the run being replaced is the one `default` has just built and never stepped.
                app.load_chosen(path, None);
                if let Some(status) = app.controls.file_status.as_ref().filter(|s| s.failed) {
                    eprintln!("{}", status.text);
                }
            }
            Ok(Box::new(app))
        }),
    )
}

/// egui's bundled proportional font has no subscripts, Greek or maths symbols, so r₋, τ, ν, ξ
/// and friends rendered as boxes. Atkinson Hyperlegible (SIL OFL) is the primary text face;
/// DejaVu Sans (Bitstream Vera licence) sits behind it as the symbol fallback for both families.
/// Licences are in assets/fonts.
///
/// Visible to the crate because the performance harness installs the same faces into its headless
/// context: laying out r₋, τ and Ω against a fallback stack is not the same work as laying them out
/// against one face, and a frame measured with egui's bundled font would be measuring a different
/// frame from the one the window paints.
pub(crate) fn install_fonts(ctx: &egui::Context) {
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

