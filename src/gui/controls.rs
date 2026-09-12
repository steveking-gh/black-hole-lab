use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode, WorldlineParams};
use crate::physics::wavefront::SignalField;

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
    /// Draw the animated raindrop flow (the Painlevé-Gullstrand / Doran river) on the
    /// equatorial view.
    pub show_river: bool,
    /// Draw Alice's signal pulses on the equatorial view and their principal-null tracks
    /// in the (t, r) diagram.
    pub show_signal: bool,
    /// Draw the family of outgoing principal null rays trapped between r+ and r- in the (t, r)
    /// diagram.
    pub show_outgoing_rays: bool,
    pub show_streamlines: bool,
    pub enable_dual_infall: bool,
    pub delta_t_delay: f64,
    /// Conserved energy per unit mass E = -u_t of the dropped observers.
    pub energy: f64,
    /// Conserved axial angular momentum per unit mass L = u_phi, in units of M.
    pub l_ang: f64,
    /// Start the dropped observers on the outgoing root of r^4 (dr/dtau)^2 = R(r).
    pub outgoing_start: bool,
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
            show_river: true,
            show_signal: true,
            show_outgoing_rays: true,
            show_streamlines: true,
            enable_dual_infall: true,
            delta_t_delay: 8.0,
            energy: 1.0,
            l_ang: 0.0,
            outgoing_start: false,
            show_theory_modal: false,
            use_km: true,
            frame_of_ref: ReferenceFrame::DistantObserver,
            font_scale: 1.0,
        }
    }
}

/// The black hole presets, as (label, M, a/M, M_solar). One of them is highlighted when the metric
/// is that preset's, which is read off the metric rather than remembered: nothing can then drift out
/// of step with the geometry, and moving the mass or spin slider drops the highlight by itself.
const PRESETS: [(&str, f64, f64, f64); 6] = [
    ("Schwarzschild (10 M☉)", 1.0, 0.0, 10.0),
    ("Cygnus X-1 (21.2 M☉)", 1.0, 0.97, 21.2),
    ("Sagittarius A* (4.15M M☉)", 1.0, 0.90, 4.15e6),
    ("M87* (6.5B M☉)", 1.0, 0.90, 6.5e9),
    ("TON 618 (66B M☉)", 1.0, 0.88, 6.6e10),
    ("Extreme Kerr (a=0.998)", 1.0, 0.998, 10.0),
];

/// Agreement of two of those parameters to one part in a million, with an absolute floor of that
/// same size so that a spin of zero can be compared at all. Every slider step is larger than this.
fn same_to_a_millionth(x: f64, y: f64) -> bool {
    (x - y).abs() <= 1e-6 * y.abs().max(1.0)
}

impl AppControls {
    /// The constants of motion the dual-observer group is currently asking for.
    fn worldline_params(&self) -> WorldlineParams {
        WorldlineParams::new(self.energy, self.l_ang, self.outgoing_start)
    }

    pub fn render_panel(
        &mut self,
        ui: &mut egui::Ui,
        metric: &mut KerrSchild,
        bob: &mut Observer,
        alice: &mut Option<Observer>,
        signal: &mut SignalField,
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
                    signal.clear();
                    let params = self.worldline_params();
                    bob.reset_with_phi(metric, 0.0, 3.8, 0.0, params);
                    if let Some(al) = alice {
                        al.reset_with_phi(metric, 0.0, 4.5, 0.25, params);
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
                    // A wavefront cannot be run backwards, so the field is dropped and re-emitted
                    // as time advances again. See the same note at the app's arrow-key handler.
                    signal.clear();
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
                    signal.advance(metric, current_step);
                    if let Some(al) = alice {
                        signal.emit_if_due(metric, al);
                    }
                    signal.detect_receptions(metric, bob);
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
            ui.checkbox(&mut self.show_river, "River of Space (raindrop flow)")
                .on_hover_text(
                    "Each drop is an element of the E = 1, L = 0 raindrop flow of the Painlevé-Gullstrand / Doran river model, drawn at proper size and entering the field at r = 12M as a circle of proper diameter 0.1 M. The flow alone deforms that circle after that: length along the flow grows as √(12M/r), the ratio of Doran river speeds, and width across the flow shrinks as neighbouring flow lines converge, √g_φφ δφ. The drawn aspect ratio is therefore the tidal stretching of the fluid element, reaching about 16 at r₊ for a = 0.65. Colour is the flow speed past a local ZAMO, β = √(1 − α²), reaching c at r₊.",
                );
            ui.checkbox(&mut self.show_signal, "Alice's Signal (pulses)")
                .on_hover_text(
                    "Alice broadcasts a pulse into the whole of her own light cone every 0.1 M of her proper time, and every ray of it is an exact null geodesic of the coded metric. Colour is the frequency a local raindrop measures against Alice's emission, from a tenfold redshift through white to a thousandfold blueshift. Inside r₊ the rays that never reach r₋ are the prograde ones, dragged forward in ϕ: that is the arc of the pulse around α = 90°, running from about 45° to 135° well inside r₊, wider than that just below r₊ and narrowing as Alice nears r₋, its edges lying exactly where E − Ω₋L changes sign. Those arcs stack up against the Cauchy horizon while the rest of the pulse falls through it, and because the pulses are close enough together for consecutive arcs to overlap there, an infaller crossing r₋ where they stand cuts through several sheets in a row, each blueshifted on the scale exp(κ₋Δt).",
                );
            ui.checkbox(&mut self.show_outgoing_rays, "Outgoing Light Inside r₊")
                .on_hover_text(
                    "The pink lines in the (t, r) diagram are outgoing principal null rays of the interior. Each leaves r₊, falls, and piles onto r₋ without ever crossing it. Every line is the same ray translated in t, and r − r₋ shrinks like exp(−κ₋t), so the pile-up is exponential and the last stretch of every ray lies within a pixel of r₋. The Cauchy horizon in this chart is where the outgoing light of the whole interior accumulates, and a worldline falling through r₋ cuts the whole pile in finite proper time.",
                );
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
                for (label, m, a_star, m_solar) in PRESETS {
                    let active = same_to_a_millionth(metric.m, m)
                        && same_to_a_millionth(metric.a_star(), a_star)
                        && same_to_a_millionth(metric.m_solar, m_solar);
                    if ui.selectable_label(active, label).clicked() {
                        *metric = KerrSchild::with_solar_mass(m, a_star * m, m_solar);
                        preset_changed = true;
                    }
                }
                if preset_changed {
                    *current_time = 0.0;
                    signal.clear();
                    let params = self.worldline_params();
                    bob.reset_with_phi(metric, 0.0, 3.8, 0.0, params);
                    if let Some(al) = alice {
                        al.reset_with_phi(metric, 0.0, 4.5, 0.25, params);
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
                ui.selectable_value(&mut bob.mode, ObserverMode::Static, "Static");
                ui.selectable_value(&mut bob.mode, ObserverMode::Zamo, "ZAMO");
            });

            // A static observer needs r > 2M (timelike d/dt); a ZAMO needs r > r+ (a fixed-r
            // worldline can only be timelike outside the outer horizon). When the selection is
            // impossible where Bob actually is, say so and fall back to the free-fall worldline.
            if !bob.mode_admissible(metric) {
                let why = match bob.mode {
                    ObserverMode::Static => "(no static observer can exist here: r ≤ 2M)",
                    ObserverMode::Zamo => "(no ZAMO inside r₊)",
                    _ => "",
                };
                ui.label(
                    egui::RichText::new(format!("{} — showing free fall", why))
                        .small()
                        .color(Theme::TEXT_MUTED),
                );
            }

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

                // The boost only defines a worldline in ManualDrag mode: free fall, static and
                // ZAMO observers each pin down their own 4-velocity, so a beta there would be
                // ignored. Hide the sliders rather than show dead controls.
                ui.add(egui::Slider::new(&mut bob.beta_r, -0.95..=0.95).text("Radial Boost β_r"));
                ui.add(egui::Slider::new(&mut bob.beta_phi, -0.95..=0.95).text("Azimuthal Boost β_ϕ"));
                ui.label(
                    egui::RichText::new(
                        "Velocity relative to a raindrop observer (dropped from rest at infinity) at Bob's r",
                    )
                    .small()
                    .color(Theme::TEXT_MUTED),
                );

                if ui.button("Reset Bob's Thrusters").clicked() {
                    bob.beta_r = 0.0;
                    bob.beta_phi = 0.0;
                }
            }
        });

        ui.add_space(4.0);

        // 4. Dual Infall Simulation (Alice & Bob delta-T)
        ui.group(|ui| {
            ui.label(egui::RichText::new("👥 DUAL OBSERVER (Δt RELEASE)").strong().color(Theme::ALICE_COLOR));
            ui.checkbox(&mut self.enable_dual_infall, "Enable Alice & Bob Infall");

            if self.enable_dual_infall {
                ui.add(egui::Slider::new(&mut self.delta_t_delay, 2.0..=30.0).text("Release Delay Δt"));
                ui.add(egui::Slider::new(&mut self.energy, 0.90..=1.60).text("Energy E (per unit mass)"));
                ui.add(egui::Slider::new(&mut self.l_ang, -4.0..=4.0).text("Angular momentum L (per unit mass, M)"));
                ui.checkbox(&mut self.outgoing_start, "Start on the outgoing root (dr/dτ > 0)");

                // Below the effective potential V(r, L) there is no timelike geodesic through
                // r = 4.5M at all, so the drop raises E to the floor instead of refusing.
                let floor = GeodesicState::energy_floor(metric, 4.5, self.l_ang);
                if self.energy < floor {
                    ui.label(
                        egui::RichText::new(format!(
                            "E raised to {:.3}: below that, r = 4.5M is forbidden for this L",
                            floor
                        ))
                        .small()
                        .color(Theme::TEXT_MUTED),
                    );
                }
                if self.outgoing_start {
                    ui.label(
                        egui::RichText::new(
                            "Outgoing start: the ingoing chart cannot follow an outward crossing of r₋",
                        )
                        .small()
                        .color(Theme::TEXT_MUTED),
                    );
                }
                ui.label(egui::RichText::new("Alice drops from r = 4.5M at t = 0; Bob hovers there and is released at t = Δt, so his worldline trails hers by about Δt in coordinate time the whole way in.").small().color(Theme::TEXT_MUTED));
                if ui.button("Drop Observers").clicked() {
                    *current_time = 0.0;
                    signal.clear();
                    let params = self.worldline_params();
                    *alice = Some(Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
                    *bob = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, self.delta_t_delay, 0.0, params);
                }
            } else {
                *alice = None;
                signal.clear();
            }
        });

        ui.add_space(6.0);

        // 5. Theory Explanations
        if ui.button("📖 Relativistic Theory & Horizons").clicked() {
            self.show_theory_modal = !self.show_theory_modal;
        }
    }
}
