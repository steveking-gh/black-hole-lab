use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use crate::physics::wavefront::{SignalField, limiting_blueshift};

pub struct CauchyEffects;

/// Format the measured shift of ingoing principal null light, nu_obs / nu_inf = -k.u.
/// Two decimals while the ratio is a number a reader can hold in their head, scientific notation
/// at either end of that: the shift is proportional to u^t on the approach to the far branch of
/// r-, where `geodesic::U_T_STALL` follows the worldline out to u^t = 1e10, and a plain decimal
/// would be an unreadable run of ten digits in a line a few characters wide.
fn fmt_nu(ratio: f64) -> String {
    if !(0.01..1e4).contains(&ratio) {
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
    ///
    /// Either observer may be absent - the "Enable Observer" box on their card unticked - and every
    /// line here is a statement about somebody, so each one is drawn only where the observers it
    /// names are in the simulation. `release_gap` is the coordinate time between the two releases,
    /// which is the Delta t the blueshift scale exp(kappa_- Delta t) is quoted against.
    #[allow(clippy::too_many_arguments)]
    pub fn render_hud(
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        signal: &SignalField,
        bob_signal: &SignalField,
        release_gap: f64,
        current_time: f64,
        use_physical_units: bool,
    ) {
        ui.vertical(|ui| {
            ui.add_space(2.0);

            ui.group(|ui| {
                ui.set_width(ui.available_width());

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("⏱ RELATIVISTIC TELEMETRY & PROPER CLOCKS").strong().color(Theme::HORIZON_OUTER));

                    // Radial separation readout aligned to the right
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if let (Some(al), Some(bob)) = (alice, bob)
                            && al.is_active
                            && bob.is_active
                        {
                            let diff = (bob.r - al.r).abs();
                            let sep_str = if use_physical_units {
                                metric.format_km(metric.r_to_km(diff))
                            } else {
                                format!("{:.3}M", diff)
                            };
                            ui.label(format!("Radial Separation: Δr = {}", sep_str));
                        }
                    });
                });

                ui.horizontal(|ui| {
                    if let Some(al) = alice {
                        let al_r_str = if use_physical_units {
                            metric.format_km(metric.r_to_km(al.r))
                        } else {
                            format!("{:.2}M", al.r)
                        };
                        let tau_str = if use_physical_units {
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

                    if let Some(bob) = bob {
                        let bob_r_str = if use_physical_units {
                            metric.format_km(metric.r_to_km(bob.r))
                        } else {
                            format!("{:.2}M", bob.r)
                        };
                        let bob_tau_str = if use_physical_units {
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
                    }

                    let ext_t_str = if use_physical_units {
                        metric.format_physical_time(current_time)
                    } else {
                        format!("{:.2}M ({})", current_time, metric.format_physical_time(current_time))
                    };
                    ui.label(format!("Exterior Time t: {}", ext_t_str));
                    if let Some(bob) = bob.as_ref().filter(|b| b.release_t > 0.0) {
                        let delay_str = if use_physical_units {
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
                if alice.is_some() && bob.is_some() {
                    ui.horizontal(|ui| {
                        let received = signal.received_count();
                        let scale = fmt_shift(limiting_blueshift(metric, release_gap));
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
                                ui.label(format!(", scale e^(κ₋Δt_release) = {}", scale));
                            }
                            _ => {
                                ui.label(format!(
                                    "Alice → Bob: no pulse received yet, scale e^(κ₋Δt_release) = {}",
                                    scale
                                ));
                            }
                        }
                    });

                    // The return path, and it is not the mirror image. Bob transmits from t = 0,
                    // first as the static observer he is while he hovers - his proper time runs
                    // there at sqrt(-g_tt) dt, which is a perfectly good clock to pace a
                    // transmission by - and then in free fall once he is released. Where he trails
                    // her - a Release Delay on his card, rather than the layout the app opens on,
                    // where he falls past a ZAMO Alice - his pulses have to chase her inward and
                    // the only rays that catch her are the ingoing ones, whose shift is finite on
                    // the branch of r₋ she crosses; the rays of his that pile onto r₋ settle there
                    // behind her, after she has already crossed, so she never meets a stack and
                    // there is no e^(κ₋Δt) scale to quote on this line. Give him the shorter delay
                    // of the two and he is the deeper one instead, his light climbing to her, and
                    // the shift starts as a small blueshift and turns over into a redshift as he
                    // falls away below her. Either way the transmission has an end: once her worldline
                    // has finished - on the ring, or frozen on r₋ - the last pulse of his that
                    // arrived marks the event on *his* worldline past which nothing he sends can
                    // ever reach her.
                    ui.horizontal(|ui| {
                        let received = bob_signal.received_count();
                        match bob_signal.last_reception() {
                            Some(r) => {
                                ui.label(format!(
                                    "Bob → Alice: {} receptions, last ν_A/ν_B = ",
                                    received
                                ));
                                ui.label(
                                    egui::RichText::new(fmt_shift(r.ratio))
                                        .strong()
                                        .color(Theme::shift_colour(r.ratio, 255)),
                                );
                            }
                            None => {
                                ui.label("Bob → Alice: no pulse received yet".to_string());
                            }
                        }
                        // Before her worldline ends nothing is said about reachability: a pulse
                        // still in flight may yet arrive, and the simulation is the only criterion.
                        if alice.as_ref().is_some_and(|al| al.has_ended()) {
                            match bob_signal.last_delivered_pulse() {
                                Some(pulse) => {
                                    let never = bob_signal.pulses_after(pulse.pulse_index);
                                    let r_str = if use_physical_units {
                                        metric.format_km(metric.r_to_km(pulse.emitted_r))
                                    } else {
                                        format!("{:.3}M", pulse.emitted_r)
                                    };
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "· last pulse to reach her: #{} sent at t = {:.2}M, r = {} (Bob's τ = {:.2}M); {} later pulses never arrive",
                                            pulse.pulse_index,
                                            pulse.emitted_t,
                                            r_str,
                                            pulse.emitted_tau,
                                            never
                                        ))
                                        .color(Theme::BOB_COLOR),
                                    );
                                }
                                None => {
                                    ui.label(
                                        egui::RichText::new(
                                            "· her worldline has ended and nothing of his ever reached her",
                                        )
                                        .color(Theme::BOB_COLOR),
                                    );
                                }
                            }
                        }
                    });
                }
            });
        });
    }
}
