//! The dialog a press of Look Around puts up when a piece the view needs is missing.
//!
//! A modal dialog over the whole window, because the user has asked for something they cannot
//! have until they have answered it: it names each missing piece in type larger than the panel's,
//! asks whether to download what can be downloaded, and has an OK and a Cancel. What each button
//! does is the app's (`SpacetimeApp::look_dialog_ok`, `look_dialog_cancel`), since the download
//! is a process the app owns; this module holds what the dialog knows and draws it. See
//! `crate::look_around`, "What a view needs", for the arrangement as a whole.

use crate::gui::controls::FileStatus;
use crate::gui::theme::Theme;
use crate::look_around::Readiness;
use crate::physics::observer::Who;

/// How much larger than the panel's body text the dialog's text is: the panel's small red status
/// line was hard to read, and this is the one place the user has to read every word.
const TEXT_SCALE: f32 = 1.25;

/// The dialog's width, in widths of its own text size.
const WIDTH_IN_EMS: f32 = 36.0;

/// What the dialog says of a star map the app can download.
const MAP_MISSING: &str = "The star map is missing. Look Around draws the stars from one of NASA's Deep Star Maps, which Black Hole Lab can download from NASA's server: one file of 153 MB.";

/// What the dialog says of an ffmpeg that is missing, above the command that installs ffmpeg.
const FFMPEG_MISSING: &str = "ffmpeg is missing. ffmpeg is the program that writes the photograph, and only you can install ffmpeg: the installation may ask for your password. Run this command in a terminal:";

/// What the dialog says once OK has been pressed and ffmpeg is still missing.
const FFMPEG_STILL_MISSING: &str = "ffmpeg is still not installed. Run the command above in a terminal, wait for the installation to finish, then press OK.";

/// The same, on a system whose install command the app does not know.
const FFMPEG_STILL_MISSING_NO_COMMAND: &str = "ffmpeg is still not installed. Install ffmpeg, wait for the installation to finish, then press OK.";

/// What a button of the dialog asked for this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DialogAction {
    Ok,
    Cancel,
}

/// The dialog's state: whose view was asked for, what is missing, and how the download is going.
#[derive(Debug, Clone)]
pub struct LookDialog {
    /// Whose view the press asked for, which is made once nothing is missing.
    pub who: Who,
    /// What is missing, as `sky-look --check` last said.
    pub needs: Readiness,
    /// How far the download of the star map has got, or what the last one came to.
    pub fetch: Option<FileStatus>,
    /// Whether the star map is being downloaded at this moment.
    pub fetching: bool,
    /// Whether OK has been pressed: the user has agreed to the download and wants the view, so a
    /// piece still missing is now one to be reminded of.
    pub asked: bool,
}

impl LookDialog {
    /// The dialog as a press opens it: nothing downloaded, nothing agreed to.
    pub fn new(who: Who, needs: Readiness) -> Self {
        Self { who, needs, fetch: None, fetching: false, asked: false }
    }

    /// The line above the buttons: what OK will do now.
    ///
    /// The question the dialog exists to ask comes first: while there is a star map to download
    /// and no download running, OK starts one, and nothing has been downloaded before it. After
    /// that the line says what the dialog is waiting for.
    pub fn question(&self) -> String {
        let name = self.who.name();
        if self.needs.map_fetchable && !self.fetching {
            "Download missing dependencies?".to_string()
        } else if self.needs.ffmpeg.is_some() {
            "Press OK when ffmpeg is installed.".to_string()
        } else if self.fetching && self.asked {
            format!("Black Hole Lab makes {name}'s view when the download ends.")
        } else {
            "Press OK to check again.".to_string()
        }
    }

    /// The reminder for a user who pressed OK with ffmpeg still missing, or None.
    pub fn reminder(&self) -> Option<&'static str> {
        (self.asked && self.needs.ffmpeg.is_some()).then_some(
            if self.needs.ffmpeg_command.is_some() {
                FFMPEG_STILL_MISSING
            } else {
                FFMPEG_STILL_MISSING_NO_COMMAND
            },
        )
    }

    /// Draw the dialog over everything else, and return the button pressed this frame, if one
    /// was. Escape is Cancel. A click outside the dialog is nothing: a user paying half attention
    /// must not cancel a download by missing a button.
    pub fn show(&self, ctx: &egui::Context) -> Option<DialogAction> {
        let size = ctx.global_style().text_styles[&egui::TextStyle::Body].size * TEXT_SCALE;
        let text = |text: &str, colour| egui::RichText::new(text).size(size).color(colour);
        let mut action = None;
        egui::Modal::new(egui::Id::new("look_around_dialog")).show(ctx, |ui| {
            ui.set_width(size * WIDTH_IN_EMS);
            ui.label(
                egui::RichText::new(format!(
                    "Look Around cannot make {}'s view yet",
                    self.who.name()
                ))
                .size(size * 1.2)
                .strong()
                .color(Theme::HEADING),
            );
            ui.add_space(size * 0.5);

            // A copy of the app that is incomplete, or a map only its owner can supply: sky-look's
            // own sentence, since nothing here mends either.
            for why in [&self.needs.tools, &self.needs.map].into_iter().flatten() {
                ui.label(text(why, Theme::TEXT_BRIGHT));
                ui.add_space(size * 0.5);
            }
            if self.needs.map_fetchable || self.fetch.is_some() {
                if self.needs.map_fetchable {
                    ui.label(text(MAP_MISSING, Theme::TEXT_BRIGHT));
                }
                if let Some(status) = &self.fetch {
                    let colour = if status.failed { Theme::WARNING_RED } else { Theme::HEADING };
                    ui.label(text(&status.text, colour));
                }
                ui.add_space(size * 0.5);
            }
            if let Some(why) = &self.needs.ffmpeg {
                match &self.needs.ffmpeg_command {
                    Some(command) => {
                        ui.label(text(FFMPEG_MISSING, Theme::TEXT_BRIGHT));
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new(command)
                                    .size(size)
                                    .monospace()
                                    .color(Theme::BOB_COLOR),
                            );
                            if ui.button(text("Copy", Theme::TEXT_BRIGHT)).clicked() {
                                ui.ctx().copy_text(command.clone());
                            }
                        });
                    }
                    None => {
                        ui.label(text(why, Theme::TEXT_BRIGHT));
                    }
                }
                if let Some(reminder) = self.reminder() {
                    ui.label(text(reminder, Theme::WARNING_RED).strong());
                }
                ui.add_space(size * 0.5);
            }

            ui.separator();
            ui.label(text(&self.question(), Theme::TEXT_BRIGHT).strong());
            ui.add_space(size * 0.25);
            ui.horizontal(|ui| {
                if ui.button(text("OK", Theme::TEXT_BRIGHT)).clicked() {
                    action = Some(DialogAction::Ok);
                }
                if ui.button(text("Cancel", Theme::TEXT_BRIGHT)).clicked() {
                    action = Some(DialogAction::Cancel);
                }
            });
        });
        if action.is_none() && ctx.input(|input| input.key_pressed(egui::Key::Escape)) {
            action = Some(DialogAction::Cancel);
        }
        action
    }
}
