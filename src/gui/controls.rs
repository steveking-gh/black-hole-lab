use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode, ObserverPair, WorldlineParams};
use crate::physics::wavefront::{SignalField, SignalPair};

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

/// The two transmissions a canvas draws, each with the state of its own checkbox: Alice's signal,
/// which Bob receives, and Bob's, which Alice receives. They travel together because every drawing
/// path needs all four, and because the pair is one idea - the same field code run once in each
/// direction, so that the user can see both what reaches Bob and what stops reaching Alice.
#[derive(Clone, Copy)]
pub struct SignalViews<'a> {
    /// Alice's transmission, emitted by Alice and received by Bob.
    pub alice: &'a SignalField,
    /// Whether "Alice's Signal (pulses)" is ticked.
    pub show_alice: bool,
    /// Bob's transmission, emitted by Bob and received by Alice.
    pub bob: &'a SignalField,
    /// Whether "Bob's Signal (pulses)" is ticked.
    pub show_bob: bool,
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
    /// Draw Alice's signal pulses on the equatorial view and their radial extent, as wedges, in
    /// the (t, r) diagram.
    pub show_signal: bool,
    /// The same for Bob's own transmission, which Alice receives.
    pub show_bob_signal: bool,
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
            show_bob_signal: true,
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

/// The step-distance quick-picks in Distance step mode, as (label, km).
const STEP_DISTANCE_PRESETS: [(&str, f64); 4] =
    [("10k km", 10_000.0), ("1,000 km", 1000.0), ("100 km", 100.0), ("10 km", 10.0)];

/// The preset the metric currently *is*, by label, or None if it is not one of them.
///
/// The highlight in the preset row is read off the geometry through this rather than remembered,
/// so nothing can drift out of step with the hole and dragging the mass or spin slider drops the
/// highlight by itself. It is also how a test can ask what the app's default hole is without
/// duplicating the table.
pub fn active_preset(metric: &KerrSchild) -> Option<&'static str> {
    PRESETS
        .iter()
        .find(|&&(_, m, a_star, m_solar)| {
            same_to_a_millionth(metric.m, m)
                && same_to_a_millionth(metric.a_star(), a_star)
                && same_to_a_millionth(metric.m_solar, m_solar)
        })
        .map(|&(label, ..)| label)
}

/// Agreement of two values to one part in a million, with an absolute floor of that
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
        mut signals: SignalPair<'_>,
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
                    signals.clear();
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
                    .button("← Step Back")
                    .on_hover_text("Step back by Step Size / Distance (Left Arrow key)")
                    .clicked()
                {
                    // The clock stops at t = 0, so everything that is stepped back with it is
                    // stepped back by however much of the step is left above zero.
                    let back = current_step.min(*current_time);
                    *current_time -= back;
                    // The worldlines first, by the same `ObserverPair::rewind_to` that
                    // `SpacetimeApp::step_backward` calls, then the fields, which are rewound
                    // rather than dropped: `SignalField::step_back` integrates every ray back along
                    // the null geodesic it came in on, revives the ones that died inside the
                    // interval, un-sends the pulses emitted inside it, and re-establishes each
                    // receiver's side of every wavefront at the rewound state. That order is why
                    // the observers move first, and going through `SignalPair` is why this button
                    // and the left arrow key cannot mean different things.
                    ObserverPair { bob, alice: alice.as_mut() }.rewind_to(metric, *current_time);
                    signals.step_back(metric, back, alice.as_ref(), bob);
                }
                if ui
                    .button("Step Fwd →")
                    .on_hover_text("Step forward by Step Size / Distance (Right Arrow key)")
                    .clicked()
                {
                    *current_time += current_step;
                    ObserverPair { bob, alice: alice.as_mut() }
                        .step(metric, *current_time, current_step);
                    // One description of a step forward, shared with the play loop and the arrow
                    // keys: carry both transmissions, let each emitter emit, then let each receiver
                    // listen.
                    signals.advance(metric, current_step, alice.as_ref(), bob);
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
                    // The quick-pick that matches the slider's value is filled like the active
                    // step mode above it, and read off the value rather than remembered, so
                    // dragging the slider off a preset drops the fill by itself.
                    ui.horizontal(|ui| {
                        for (label, km) in STEP_DISTANCE_PRESETS {
                            let active = same_to_a_millionth(self.step_distance_km, km);
                            if ui.selectable_label(active, label).clicked() {
                                self.step_distance_km = km;
                            }
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
                    "Alice broadcasts a pulse into the whole of her own light cone every 0.1 M of her proper time, and every ray of it is an exact null geodesic of the coded metric. Colour is the frequency a local raindrop measures against Alice's emission, from a tenfold redshift through white to a thousandfold blueshift. Inside r₊ the rays that never reach r₋ are the prograde ones, dragged forward in ϕ: that is the arc of the pulse around α = 90°, running from about 45° to 135° well inside r₊, wider than that just below r₊ and narrowing as Alice nears r₋, its edges lying exactly where E − Ω₋L changes sign. On the equatorial view that arc is drawn in salmon: the part of each ring sent prograde enough to have negative energy along the inner horizon's rotating generator, E − Ω₋L < 0, which never crosses the drawn r₋ circle but piles onto it from outside while co-rotating at Ω₋, whereas the rest of the ring crosses at finite time. The salmon arc is about a third of the ring for a pulse sent just inside r₊ and only a sliver for one sent close to r₋, and it is beaded with dots because the arc collapses onto r₋ faster than a pixel can show. Those arcs stack up against the Cauchy horizon while the rest of the pulse falls through it, and because the pulses are close enough together for consecutive arcs to overlap there, an infaller crossing r₋ where they stand cuts through several sheets in a row, each blueshifted on the scale exp(κ₋Δt). Each loop is one pulse and encloses Alice, since light is isotropic in Alice's own frame, and the dot on the loop marks the emission event on Alice's trail. Inside r₊ the flow carries the whole loop inward, so the loop's outer edge never gets further from the hole than that dot: the river model, drawn with light. On the (t, r) diagram, where azimuth cannot be drawn at all, a pulse is its radial extent: a wedge from the emission event, filled faintly in her amber, whose lower edge is the most ingoing ray of the pulse and whose upper edge is the outermost one. The lower edge is the ingoing edge of Alice's own light cone carried forward - the 45° line dr/dt = −1 for a hole with no spin, a little steeper for one that spins, and steeper again the deeper it goes - and inside r₊ it runs on to the ring while the upper edge freezes on r₋, so the upper edges of her interior pulses stack up on the Cauchy horizon, which in this chart is where the outgoing light of the whole interior accumulates, and that stack is what a later infaller cuts through. A worldline inside a wedge is in range of that pulse, not necessarily receiving it: the diagram cannot say whether the ray standing at that radius is at the receiver's azimuth. The dots on a worldline are the actual receptions, and they are the only marks of one.",
                );
            ui.checkbox(&mut self.show_bob_signal, "Bob's Signal (pulses)")
                .on_hover_text(
                    "Bob broadcasts exactly as Alice does, a whole light cone of exact null geodesics every 0.1 M of his own proper time, and he starts at t = 0, before he is released: while he waits he is the static observer at his hover radius, with a clock ticking at √(−g_tt) of coordinate time and an orthonormal frame to broadcast into, and nothing in the geometry stops him transmitting from it. His pulses come every 0.134 M of coordinate time while he hovers at r = 4.5M and every 0.1 M of his own once he falls. The colours mean the same thing as Alice's: the shift a local raindrop measures against his emission. His fronts are drawn at half stroke width and his emission dots in his own mint, so the two transmissions can be told apart without touching the shift colouring, which is a measurement. What is not the same is the physics of the return path. In the layout Drop Observers builds, Bob is behind Alice on the same infall, so his pulses chase her inward, and the only part of each one that ever catches her is the ingoing part of his cone: it runs at up to dr/dt = −1 in this chart, which no timelike worldline can match. That is the light whose shift is finite on the branch of r₋ she actually crosses, so unlike Alice → Bob there is no stack for her to cut through. His frozen family, E − Ω₋L < 0, does pile onto r₋ from outside, but it settles there behind her, after she has already gone through, so she never meets it. (In the layout the app starts in he is the deeper of the two instead, and his light climbs to her: the shift then starts as a small blueshift, because the fall toward the light beats the recession, and turns over into a redshift as he drops away below her.) And because her worldline ends — on the ring, or frozen on r₋ — his transmission stops arriving: the last pulse of his that reached her is ringed on his worldline in both views, and it marks the boundary of the causal past of the end of her worldline. At the default Δt = 8 that ring sits on his vertical hover segment: at the app's default hole it marks the pulse he sends at t = 1.84, which reaches her at t = 5.20, most of an M before her worldline ends at t = 6.06. He is released long after that, so nothing of his release or of his own fall ever reaches her: everything he sends past that event never arrives, however long he goes on sending. On the (t, r) diagram his pulses are drawn exactly as hers are, each as the wedge of its own radial extent but in his mint: lower edge the most ingoing ray, upper edge the outermost, a worldline inside the wedge in range of the pulse rather than receiving it, and the dots the actual arrivals.",
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
                let highlighted = active_preset(metric);
                for (label, m, a_star, m_solar) in PRESETS {
                    if ui.selectable_label(highlighted == Some(label), label).clicked() {
                        *metric = KerrSchild::with_solar_mass(m, a_star * m, m_solar);
                        preset_changed = true;
                    }
                }
                if preset_changed {
                    *current_time = 0.0;
                    signals.clear();
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
                    signals.clear();
                    let params = self.worldline_params();
                    *alice = Some(Observer::new_with_phi(metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
                    *bob = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, self.delta_t_delay, 0.0, params);
                }
            } else {
                *alice = None;
                signals.clear();
            }
        });

        ui.add_space(6.0);

        // 5. Theory Explanations
        if ui.button("📖 Relativistic Theory & Horizons").clicked() {
            self.show_theory_modal = !self.show_theory_modal;
        }
    }
}
