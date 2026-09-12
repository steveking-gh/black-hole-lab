use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;

pub struct CauchyEffects;

impl CauchyEffects {
    /// Render the dedicated Cauchy Horizon Relativistic Telemetry HUD panel
    pub fn render_hud(
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Observer,
        alice: &Option<Observer>,
        current_time: f64,
        delta_t_delay: f64,
        use_km: bool,
    ) {
        let rm = metric.inner_horizon();

        ui.vertical(|ui| {
            ui.add_space(2.0);

            ui.group(|ui| {
                ui.set_width(ui.available_width());

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("⏱ RELATIVISTIC TELEMETRY & PROPER CLOCKS").strong().color(Theme::HORIZON_OUTER));

                    // Coalescence radial distance readout aligned to the right
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if let Some(al) = alice {
                            if al.is_active && bob.is_active {
                                let diff = (bob.r - al.r).abs();
                                let sep_str = if use_km {
                                    metric.format_km(metric.r_to_km(diff))
                                } else {
                                    format!("{:.3}M", diff)
                                };
                                if diff < 0.15 && al.r <= rm + 0.15 {
                                    ui.label(
                                        egui::RichText::new(format!("Radial Separation: Δr = {} ➔ COALESCED AT r₋!", sep_str))
                                            .strong()
                                            .color(Theme::WARNING_RED),
                                    );
                                } else {
                                    ui.label(format!("Radial Separation: Δr = {}", sep_str));
                                }
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
                    ui.separator();

                    let ext_t_str = if use_km {
                        metric.format_physical_time(current_time)
                    } else {
                        format!("{:.2}M ({})", current_time, metric.format_physical_time(current_time))
                    };
                    let delay_str = if use_km {
                        metric.format_physical_time(delta_t_delay)
                    } else {
                        format!("{:.1}M", delta_t_delay)
                    };
                    ui.label(format!("Exterior Time t: {}", ext_t_str));
                    ui.label(format!("(Bob Delay Δt: {})", delay_str));
                });
            });
        });
    }
}
