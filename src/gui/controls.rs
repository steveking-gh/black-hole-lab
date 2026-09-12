use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceFrame {
    DistantObserver,
    Bob,
    Alice,
}

impl ReferenceFrame {
    pub fn label(&self) -> &'static str {
        match self {
            Self::DistantObserver => "Global Foliation (Kerr-Schild)",
            Self::Bob => "Bob's Rest Frame (45° Cones)",
            Self::Alice => "Alice's Rest Frame (45° Cones)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepMode {
    Time,     // Fixed Δt
    Distance, // Fixed Δr (km)
}

#[derive(Debug, Clone)]
pub struct AppControls {
    pub is_playing: bool,
    pub step_mode: StepMode,
    pub step_size: f64,
    /// Playback rate while playing: coordinate time (units of M) per wall-clock second.
    pub play_speed: f64,
    pub step_distance_km: f64,
    pub auto_slow_cauchy: bool,
    pub show_wavefronts: bool,
    pub show_streamlines: bool,
    pub enable_dual_infall: bool,
    pub delta_t_delay: f64,
    pub show_theory_modal: bool,
    pub use_km: bool,
    pub frame_of_ref: ReferenceFrame,
    pub font_scale: f32,
}

impl Default for AppControls {
    fn default() -> Self {
        Self {
            is_playing: true,
            step_mode: StepMode::Time,
            step_size: 0.1,
            play_speed: 1.0,
            step_distance_km: 1000.0,
            auto_slow_cauchy: true,
            show_wavefronts: true,
            show_streamlines: true,
            enable_dual_infall: true,
            delta_t_delay: 8.0,
            show_theory_modal: false,
            use_km: false,
            frame_of_ref: ReferenceFrame::DistantObserver,
            font_scale: 1.0,
        }
    }
}

impl AppControls {
    pub fn render_panel(
        &mut self,
        ui: &mut egui::Ui,
        metric: &mut KerrSchild,
        bob: &mut Observer,
        alice: &mut Option<Observer>,
        current_time: &mut f64,
    ) {
        ui.heading(egui::RichText::new("SPACETIME LAB").strong().color(Theme::HORIZON_OUTER));
        ui.label(egui::RichText::new("Ingoing Kerr-Schild Foliation").small().color(Theme::TEXT_MUTED));
        ui.separator();

        // 0. Frame of Reference Selector
        ui.group(|ui| {
            ui.label(egui::RichText::new("🔭 FRAME OF REFERENCE").strong().color(Theme::HORIZON_CAUCHY));
            ui.label(egui::RichText::new("Observer rest frame & coordinate perspective:").small().color(Theme::TEXT_MUTED));
            egui::ComboBox::from_id_salt("frame_of_ref_controls_combo")
                .selected_text(self.frame_of_ref.label())
                .width(250.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.frame_of_ref, ReferenceFrame::DistantObserver, ReferenceFrame::DistantObserver.label());
                    ui.selectable_value(&mut self.frame_of_ref, ReferenceFrame::Bob, ReferenceFrame::Bob.label());
                    ui.selectable_value(&mut self.frame_of_ref, ReferenceFrame::Alice, ReferenceFrame::Alice.label());
                });
        });

        ui.add_space(4.0);

        // 1. Playback & Simulation Transport
        ui.group(|ui| {
            ui.label(egui::RichText::new("SIMULATION TRANSPORT").strong().color(Theme::TEXT_BRIGHT));
            ui.horizontal(|ui| {
                let play_btn_text = if self.is_playing { "⏸ Pause (Space)" } else { "▶ Play (Space)" };
                if ui
                    .button(play_btn_text)
                    .on_hover_text("Toggle Play/Pause simulation (Spacebar)")
                    .clicked()
                {
                    self.is_playing = !self.is_playing;
                }
                if ui.button("⏮ Reset").clicked() {
                    *current_time = 0.0;
                    bob.reset(0.0, 3.8);
                    if let Some(al) = alice {
                        al.reset_with_phi(0.0, 4.5, 0.25);
                    }
                }
                let current_step = match self.step_mode {
                    StepMode::Time => self.step_size,
                    StepMode::Distance => {
                        let delta_r_m = metric.km_to_r(self.step_distance_km);
                        let v_coord = bob.velocity_c(metric).abs().max(0.01);
                        (delta_r_m / v_coord).clamp(1e-8, 500.0)
                    }
                };
                if ui
                    .button("⏪ Step Back (←)")
                    .on_hover_text("Step back by Step Size / Distance (Left Arrow key)")
                    .clicked()
                {
                    *current_time = (*current_time - current_step).max(0.0);
                    bob.step_back(metric, current_step);
                    if let Some(al) = alice {
                        al.step_back(metric, current_step);
                    }
                }
                if ui
                    .button("⏭ Step Fwd (→)")
                    .on_hover_text("Step forward by Step Size / Distance (Right Arrow key)")
                    .clicked()
                {
                    *current_time += current_step;
                    bob.step(metric, *current_time, current_step);
                    if let Some(al) = alice {
                        al.step(metric, *current_time, current_step);
                    }
                }
            });

            ui.add(
                egui::Slider::new(&mut self.play_speed, 0.05..=20.0)
                    .logarithmic(true)
                    .text("Play Speed (M / real second)"),
            );

            ui.horizontal(|ui| {
                ui.label("Step Mode:");
                ui.selectable_value(&mut self.step_mode, StepMode::Time, "⏱ Time (Δt)");
                ui.selectable_value(&mut self.step_mode, StepMode::Distance, "📏 Distance (Δr)");
            });

            match self.step_mode {
                StepMode::Time => {
                    ui.add(
                        egui::Slider::new(&mut self.step_size, 0.0005..=0.5)
                            .logarithmic(true)
                            .text("Step Size (Δt)"),
                    );
                }
                StepMode::Distance => {
                    let r_grav = metric.r_grav_km();
                    let max_dist = (r_grav * 2.0).max(100_000.0);
                    let min_dist = (r_grav * 1e-6).max(0.1);
                    ui.add(
                        egui::Slider::new(&mut self.step_distance_km, min_dist..=max_dist)
                            .logarithmic(true)
                            .text("Step Dist (km)"),
                    );
                    ui.horizontal(|ui| {
                        if ui.button("10k km").clicked() {
                            self.step_distance_km = 10_000.0;
                        }
                        if ui.button("1,000 km").clicked() {
                            self.step_distance_km = 1000.0;
                        }
                        if ui.button("100 km").clicked() {
                            self.step_distance_km = 100.0;
                        }
                        if ui.button("10 km").clicked() {
                            self.step_distance_km = 10.0;
                        }
                    });
                }
            }
            ui.checkbox(
                &mut self.auto_slow_cauchy,
                "⚡ Auto-slow near Cauchy Horizon (preserve detail)",
            );
            ui.checkbox(&mut self.show_wavefronts, "Incoming Wavefront Pulses");
            ui.checkbox(&mut self.show_streamlines, "Frame-Dragging Streamlines");

            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.label("🔤 Font Size:");
                if ui.button("➖").on_hover_text("Decrease Font Size").clicked() {
                    self.font_scale = (self.font_scale - 0.1).clamp(0.7, 1.8);
                }
                let pct_label = format!("{:.0}%", self.font_scale * 100.0);
                ui.add(
                    egui::Slider::new(&mut self.font_scale, 0.7..=1.8)
                        .show_value(false)
                        .text(pct_label),
                );
                if ui.button("➕").on_hover_text("Increase Font Size").clicked() {
                    self.font_scale = (self.font_scale + 0.1).clamp(0.7, 1.8);
                }
            });
        });

        ui.add_space(4.0);

        // 2. Black Hole Parameters & Presets
        ui.group(|ui| {
            ui.label(egui::RichText::new("🌌 BLACK HOLE GEOMETRY").strong().color(Theme::TEXT_BRIGHT));

            // Logarithmic Mass input
            let mut log_mass = metric.m_solar.log10();
            if ui.add(egui::Slider::new(&mut log_mass, 0.0..=11.0).text("Mass log₁₀(M☉)")).changed() {
                let m_solar = 10.0_f64.powf(log_mass);
                *metric = KerrSchild::with_solar_mass(metric.m, metric.a, m_solar);
            }
            ui.label(format!("Mass: {:.2e} M☉", metric.m_solar));

            let mut spin_ratio = metric.a_star();
            if ui.add(egui::Slider::new(&mut spin_ratio, 0.0..=0.999).text("Spin a/M")).changed() {
                *metric = KerrSchild::with_solar_mass(metric.m, spin_ratio * metric.m, metric.m_solar);
            }

            ui.label(egui::RichText::new("Presets (Sets Mass & Spin):").small());
            ui.horizontal_wrapped(|ui| {
                let mut preset_changed = false;
                if ui.button("Schwarzschild (10 M☉)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.0, 10.0);
                    preset_changed = true;
                }
                if ui.button("Cygnus X-1 (21.2 M☉)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.97, 21.2);
                    preset_changed = true;
                }
                if ui.button("Sagittarius A* (4.15M M☉)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.90, 4.15e6);
                    preset_changed = true;
                }
                if ui.button("M87* (6.5B M☉)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.90, 6.5e9);
                    preset_changed = true;
                }
                if ui.button("TON 618 (66B M☉)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.88, 6.6e10);
                    preset_changed = true;
                }
                if ui.button("Extreme Kerr (a=0.998)").clicked() {
                    *metric = KerrSchild::with_solar_mass(1.0, 0.998, 10.0);
                    preset_changed = true;
                }
                if preset_changed {
                    *current_time = 0.0;
                    bob.reset(0.0, 3.8);
                    if let Some(al) = alice {
                        al.reset_with_phi(0.0, 4.5, 0.25);
                    }
                }
            });

            ui.separator();
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            let re = metric.ergosphere_equatorial();
            if self.use_km {
                ui.label(format!("• Outer Horizon r₊: {} ({:.3} M)", metric.format_km(metric.r_to_km(rp)), rp));
                ui.label(format!("• Cauchy Horizon r₋: {} ({:.3} M)", metric.format_km(metric.r_to_km(rm)), rm));
                ui.label(format!("• Ergosphere r_E:   {} ({:.3} M)", metric.format_km(metric.r_to_km(re)), re));
            } else {
                ui.label(format!("• Outer Horizon r₊: {:.3} M ({})", rp, metric.format_physical_distance(rp)));
                ui.label(format!("• Cauchy Horizon r₋: {:.3} M ({})", rm, metric.format_physical_distance(rm)));
                ui.label(format!("• Ergosphere r_E:   {:.3} M ({})", re, metric.format_physical_distance(re)));
            }
            ui.separator();
            ui.label(egui::RichText::new("📐 UNITS & COORDINATE SYSTEM").small().strong().color(Theme::TEXT_BRIGHT));
            ui.checkbox(&mut self.use_km, "📏 Display in Kilometers (km) instead of M");
            if self.use_km {
                ui.label(egui::RichText::new(format!("• Scale: 1M = {}", metric.format_physical_distance(1.0))).small().color(Theme::TEXT_MUTED));
                ui.label(egui::RichText::new(format!("• Time:  1M = {}", metric.format_physical_time(1.0))).small().color(Theme::TEXT_MUTED));
            } else {
                ui.label(egui::RichText::new(format!("• 1M [Distance] = GM/c² = {}", metric.format_physical_distance(1.0))).small());
                ui.label(egui::RichText::new(format!("• 1M [Time]     = GM/c³ = {}", metric.format_physical_time(1.0))).small());
            }
        });

        ui.add_space(4.0);

        // 3. Observer "Bob" Controls
        ui.group(|ui| {
            ui.label(egui::RichText::new("🧑 OBSERVER BOB").strong().color(Theme::BOB_COLOR));

            ui.horizontal(|ui| {
                ui.selectable_value(&mut bob.mode, ObserverMode::FreeFall, "Free Fall");
                ui.selectable_value(&mut bob.mode, ObserverMode::ManualDrag, "Drag / Manual");
                ui.selectable_value(&mut bob.mode, ObserverMode::Stationary, "Stationary");
            });

            if bob.mode == ObserverMode::ManualDrag {
                if self.use_km {
                    let mut bob_km = metric.r_to_km(bob.r);
                    let min_km = metric.r_to_km(0.05);
                    let max_km = metric.r_to_km(5.5);
                    if ui.add(egui::Slider::new(&mut bob_km, min_km..=max_km).text("Radial Position r (km)")).changed() {
                        bob.r = metric.km_to_r(bob_km);
                    }
                } else {
                    ui.add(egui::Slider::new(&mut bob.r, 0.05..=5.5).text("Radial Position r (M)"));
                }
            }

            ui.add(egui::Slider::new(&mut bob.beta_r, -0.95..=0.95).text("Radial Boost β_r"));
            ui.add(egui::Slider::new(&mut bob.beta_phi, -0.95..=0.95).text("Azimuthal Boost β_ϕ"));

            if ui.button("Reset Bob's Thrusters").clicked() {
                bob.beta_r = 0.0;
                bob.beta_phi = 0.0;
            }
        });

        ui.add_space(4.0);

        // 4. Dual Infall Simulation (Alice & Bob delta-T)
        ui.group(|ui| {
            ui.label(egui::RichText::new("👥 DUAL OBSERVER (Δt RACE)").strong().color(Theme::ALICE_COLOR));
            ui.checkbox(&mut self.enable_dual_infall, "Enable Alice & Bob Infall");

            if self.enable_dual_infall {
                ui.add(egui::Slider::new(&mut self.delta_t_delay, 2.0..=30.0).text("Release Delay Δt"));
                ui.label(egui::RichText::new("Alice drops from r = 4.5M at t = 0; Bob hovers there and is released at t = Δt.").small().color(Theme::TEXT_MUTED));
                if ui.button("Drop Observers").clicked() {
                    *current_time = 0.0;
                    *alice = Some(Observer::new_with_phi("Alice", 0.0, 4.5, 0.0, 0.25));
                    *bob = Observer::new("Bob", 0.0, 4.5, self.delta_t_delay);
                }
            } else {
                *alice = None;
            }
        });

        ui.add_space(6.0);

        // 5. Theory Explanations
        if ui.button("📖 Relativistic Theory & Horizons").clicked() {
            self.show_theory_modal = !self.show_theory_modal;
        }
    }
}
