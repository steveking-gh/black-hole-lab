use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::wavefront::{SignalField, limiting_blueshift};

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

/// Format a measured frequency ratio for the signal line: three decimals while it is a number a
/// reader can hold in their head, scientific notation once the stack against r- takes over, and the
/// infinity symbol for the degenerate a = 0 case, where there is no inner horizon and exp(kappa_-
/// Delta t) has no finite value.
fn fmt_shift(ratio: f64) -> String {
    if !ratio.is_finite() {
        "∞".to_string()
    } else if ratio.abs() < 100.0 {
        format!("{:.3}", ratio)
    } else {
        format!("{:.2e}", ratio)
    }
}

impl CauchyEffects {
    /// Render the dedicated relativistic telemetry HUD panel: proper clocks, radial separation and
    /// the shift each observer measures for ingoing light.
    #[allow(clippy::too_many_arguments)]
    pub fn render_hud(
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Observer,
        alice: &Option<Observer>,
        signal: &SignalField,
        delta_t: f64,
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

                // Alice's signal: how much of it Bob has caught, and the exponential scale
                // exp(kappa_- Delta t) that the crossing of the stack on r- is measured against.
                if alice.is_some() {
                    ui.horizontal(|ui| {
                        let received = signal.received_count();
                        let scale = fmt_shift(limiting_blueshift(metric, delta_t));
                        match (signal.last_reception(), signal.max_ratio()) {
                            (Some(r), Some(max)) => {
                                ui.label(format!("Alice → Bob: {} receptions, last ν_B/ν_A = ", received));
                                ui.label(
                                    egui::RichText::new(fmt_shift(r.ratio))
                                        .strong()
                                        .color(Theme::shift_colour(r.ratio, 255)),
                                );
                                ui.label(", max = ");
                                ui.label(
                                    egui::RichText::new(fmt_shift(max))
                                        .strong()
                                        .color(Theme::shift_colour(max, 255)),
                                );
                                ui.label(format!(", scale e^(κ₋Δt) = {}", scale));
                            }
                            _ => {
                                ui.label(format!(
                                    "Alice → Bob: no pulse received yet, scale e^(κ₋Δt) = {}",
                                    scale
                                ));
                            }
                        }
                    });
                }
            });
        });
    }
}
