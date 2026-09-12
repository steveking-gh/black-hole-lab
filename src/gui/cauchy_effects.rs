use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;

pub struct CauchyEffects;

/// Format the measured shift of ingoing principal null light, nu_obs / nu_inf = -k.u.
/// Two decimals, dropping to scientific notation only where the ratio gets very small.
fn fmt_nu(ratio: f64) -> String {
    if ratio < 0.01 {
        format!("{:.2e}", ratio)
    } else {
        format!("{:.2}", ratio)
    }
}

impl CauchyEffects {
    /// Render the dedicated relativistic telemetry HUD panel: proper clocks, radial separation and
    /// the shift each observer measures for ingoing light.
    pub fn render_hud(
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Observer,
        alice: &Option<Observer>,
        current_time: f64,
        use_km: bool,
    ) {
        ui.vertical(|ui| {
            ui.add_space(2.0);

            ui.group(|ui| {
                ui.set_width(ui.available_width());

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("⏱ RELATIVISTIC TELEMETRY & PROPER CLOCKS").strong().color(Theme::HORIZON_OUTER));

                    // Radial separation readout aligned to the right
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if let Some(al) = alice {
                            if al.is_active && bob.is_active {
                                let diff = (bob.r - al.r).abs();
                                let sep_str = if use_km {
                                    metric.format_km(metric.r_to_km(diff))
                                } else {
                                    format!("{:.3}M", diff)
                                };
                                ui.label(format!("Radial Separation: Δr = {}", sep_str));
                            }
                        }
                    });
                });

                ui.horizontal(|ui| {
                    if let Some(al) = alice {
                        let al_r_str = if use_km {
                            metric.format_km(metric.r_to_km(al.r))
                        } else {
                            format!("{:.2}M", al.r)
                        };
                        let tau_str = if use_km {
                            metric.format_physical_time(al.tau)
                        } else {
                            format!("{:.2}M ({})", al.tau, metric.format_physical_time(al.tau))
                        };
                        ui.label(egui::RichText::new(format!("Alice τ: {} [r={}]", tau_str, al_r_str)).color(Theme::ALICE_COLOR));
                        ui.label(
                            egui::RichText::new(format!("ν_in/ν_∞ = {}", fmt_nu(al.ingoing_frequency_ratio(metric))))
                                .color(Theme::ALICE_COLOR),
                        );
                        ui.separator();
                    }

                    let bob_r_str = if use_km {
                        metric.format_km(metric.r_to_km(bob.r))
                    } else {
                        format!("{:.2}M", bob.r)
                    };
                    let bob_tau_str = if use_km {
                        metric.format_physical_time(bob.tau)
                    } else {
                        format!("{:.2}M ({})", bob.tau, metric.format_physical_time(bob.tau))
                    };
                    ui.label(egui::RichText::new(format!("Bob τ: {} [r={}]", bob_tau_str, bob_r_str)).color(Theme::BOB_COLOR));
                    ui.label(
                        egui::RichText::new(format!("ν_in/ν_∞ = {}", fmt_nu(bob.ingoing_frequency_ratio(metric))))
                            .color(Theme::BOB_COLOR),
                    );
                    ui.separator();

                    let ext_t_str = if use_km {
                        metric.format_physical_time(current_time)
                    } else {
                        format!("{:.2}M ({})", current_time, metric.format_physical_time(current_time))
                    };
                    ui.label(format!("Exterior Time t: {}", ext_t_str));
                    if bob.release_t > 0.0 {
                        let delay_str = if use_km {
                            metric.format_physical_time(bob.release_t)
                        } else {
                            format!("{:.1}M", bob.release_t)
                        };
                        let status = if bob.is_active { "released" } else { "hovering" };
                        ui.label(format!("(Bob release t = {} • {})", delay_str, status));
                    }
                });
            });
        });
    }
}
