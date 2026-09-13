use crate::gui::cauchy_effects::CauchyEffects;
use crate::gui::controls::{AppControls, ReferenceFrame, SignalViews, StepMode};
use crate::gui::spacetime_canvas::SpacetimeCanvas;
use crate::gui::spatial_canvas::SpatialCanvas;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverPair, WorldlineParams};
use crate::physics::wavefront::{SignalField, SignalPair};
use std::time::Instant;

pub struct SpacetimeApp {
    metric: KerrSchild,
    bob: Observer,
    alice: Option<Observer>,
    spacetime_canvas: SpacetimeCanvas,
    spatial_canvas: SpatialCanvas,
    /// Alice's signal pulses, which Bob receives. They live here rather than in a canvas because
    /// they are advanced on the simulation clock and read by both diagrams and the HUD.
    signal: SignalField,
    /// Bob's own transmission, which Alice receives. It is the same object driven the other way
    /// round, and the two are advanced, rewound and cleared together through `SignalPair`.
    bob_signal: SignalField,
    controls: AppControls,
    current_time: f64,
    last_update: Instant,
}

impl Default for SpacetimeApp {
    fn default() -> Self {
        // Sagittarius A*, the `PRESETS` row: M = 1 in geometric units, a/M = 0.90, 4.15e6 M_sun.
        // The hole the app opens on is the one at the centre of this galaxy rather than a generic
        // stellar-mass one, and the preset row lights up by itself because the highlight is read
        // off the metric (`active_preset`). At this spin r+ = 1.436M and r- = 0.564M, so the
        // trapped region is narrow and the Cauchy horizon is well clear of the ring.
        let metric = KerrSchild::with_solar_mass(1.0, 0.90, 4.15e6);
        // Both start as raindrops: E = 1, L = 0, ingoing.
        let bob = Observer::new(&metric, "Bob", 0.0, 3.8, 0.0);
        let alice = Some(Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            4.5,
            0.0,
            0.25,
            WorldlineParams::default(),
        ));

        Self {
            metric,
            bob,
            alice,
            spacetime_canvas: SpacetimeCanvas::default(),
            spatial_canvas: SpatialCanvas::default(),
            signal: SignalField::default(),
            bob_signal: SignalField::default(),
            controls: AppControls::default(),
            current_time: 0.0,
            last_update: Instant::now(),
        }
    }
}

impl SpacetimeApp {
    /// Carry both transmissions forward by dt of the simulation clock. `SignalPair::advance` owns
    /// the order - carry the light, then emit, then listen - and the panel's Step Fwd button goes
    /// through the same call, so the two paths cannot drift apart.
    fn advance_signal(&mut self, dt: f64) {
        SignalPair {
            alice: &mut self.signal,
            bob: &mut self.bob_signal,
        }
        .advance(&self.metric, dt, self.alice.as_ref(), &self.bob);
    }

    /// What one press of an arrow key is worth in coordinate time: the Δt slider in Time mode, and
    /// in Distance mode the time Bob needs to cover the requested Δr at his current coordinate
    /// speed.
    fn arrow_step(&self) -> f64 {
        match self.controls.step_mode {
            StepMode::Time => self.controls.step_size,
            StepMode::Distance => {
                let delta_r_m = self.metric.km_to_r(self.controls.step_distance_km);
                let v_coord = self.bob.velocity_c(&self.metric).abs().max(0.01);
                (delta_r_m / v_coord).clamp(1e-8, 500.0)
            }
        }
    }

    /// One step forward by hand, in the same order as a played frame.
    fn step_forward(&mut self, step: f64) {
        self.current_time += step;
        self.spatial_canvas.river.advance(&self.metric, step);
        // Both worldlines move as one object, so that this path, the play loop and the panel's
        // buttons cannot mean different things by a step. See `ObserverPair`.
        ObserverPair { bob: &mut self.bob, alice: self.alice.as_mut() }
            .step(&self.metric, self.current_time, step);
        self.advance_signal(step);
    }

    /// One step back by hand, undoing a step forward rather than approximating one.
    ///
    /// The observers and both transmissions are integrated backwards: `SignalField::step_back`
    /// runs every ray back along the null geodesic it came in on, revives the ones that reached the
    /// ring inside the interval, and un-sends the pulses emitted inside it, while
    /// `ObserverPair::rewind_to` puts both worldlines back on the clock's new value. The observers
    /// are given the *target time* rather than the interval, which is what keeps them locked to
    /// the clock when the two differ - the clock stops at zero, a Distance-mode step can be
    /// hundreds of M, and a worldline that has already ended has no interval left to undo.
    ///
    /// The river is the exception, and deliberately. Stepping back advances the congruence forward
    /// by the same amount instead of reversing it. The flow is stationary, so its picture is the
    /// same at every t and "backwards" carries no information about it; running the advection in
    /// reverse would only show particles climbing outward, which no raindrop does.
    fn step_backward(&mut self, step: f64) {
        // The clock stops at t = 0, so whatever is wound back is wound back by however much of the
        // step is left above zero, and the field's clock stays equal to the simulation clock.
        let back = step.min(self.current_time);
        self.current_time -= back;
        self.spatial_canvas.river.advance(&self.metric, step);
        SignalPair {
            alice: &mut self.signal,
            bob: &mut self.bob_signal,
        }
        .step_back(&self.metric, back);
        ObserverPair { bob: &mut self.bob, alice: self.alice.as_mut() }
            .rewind_to(&self.metric, self.current_time);
    }
}

impl eframe::App for SpacetimeApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // Calculate frame delta time
        let now = Instant::now();
        let dt = (now - self.last_update).as_secs_f64().clamp(1.0 / 240.0, 0.1);
        self.last_update = now;

        // Advance simulation if playing
        if self.controls.is_playing {
            // Time mode: frame-rate independent playback at `play_speed` units of M per real second.
            // Distance mode: per-frame step chosen so Bob moves a fixed Δr (normalised to 60 fps).
            // Nothing throttles the step near r₋: to study the crossing, pause and step by hand.
            let sim_dt = match self.controls.step_mode {
                StepMode::Time => dt * self.controls.play_speed,
                StepMode::Distance => {
                    let delta_r_m = self.metric.km_to_r(self.controls.step_distance_km);
                    let v_coord = self.bob.velocity_c(&self.metric).abs().max(0.01);
                    let base_step = (delta_r_m / v_coord).clamp(1e-8, 500.0);
                    (dt / 0.01667).clamp(0.2, 3.0) * base_step
                }
            };

            self.current_time += sim_dt;
            // The river runs on the simulation clock, not the frame clock, so it freezes when
            // paused and speeds up with the playback rate.
            self.spatial_canvas.river.advance(&self.metric, sim_dt);
            ObserverPair { bob: &mut self.bob, alice: self.alice.as_mut() }
                .step(&self.metric, self.current_time, sim_dt);
            self.advance_signal(sim_dt);
            ctx.request_repaint();
        }

        // Handle Spacebar (Play/Pause), ArrowLeft (<-), and ArrowRight (->) keyboard navigation
        let space_pressed = ctx.input(|i| i.key_pressed(egui::Key::Space));
        let step_back_pressed = ctx.input(|i| i.key_pressed(egui::Key::ArrowLeft));
        let step_fwd_pressed = ctx.input(|i| i.key_pressed(egui::Key::ArrowRight));

        if space_pressed {
            self.controls.is_playing = !self.controls.is_playing;
            ctx.request_repaint();
        } else if step_back_pressed {
            self.step_backward(self.arrow_step());
            ctx.request_repaint();
        } else if step_fwd_pressed {
            self.step_forward(self.arrow_step());
            ctx.request_repaint();
        }

        // Apply dark relativity theme styling
        let mut visuals = egui::Visuals::dark();
        visuals.override_text_color = Some(Theme::TEXT_BRIGHT);
        visuals.panel_fill = Theme::PANEL_BG;
        ctx.set_visuals(visuals);

        // 1. Top Panel
        egui::Panel::top("top_bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("🌌 ANTIGRAVITY // RELATIVISTIC SPACETIME LAB").heading().strong().color(Theme::HORIZON_OUTER));
                ui.separator();
                ui.label(
                    egui::RichText::new(format!(
                        "Kerr Metric (Mass = {:.2e} M☉, a/M = {:.3})",
                        self.metric.m_solar,
                        self.metric.a_star()
                    ))
                    .monospace()
                    .color(Theme::TEXT_BRIGHT),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Theory Guide").clicked() {
                        self.controls.show_theory_modal = !self.controls.show_theory_modal;
                    }
                    if ui.button("🔍 Reset Zoom").clicked() {
                        self.spacetime_canvas.reset_zoom();
                        self.spatial_canvas.zoom = 48.0;
                        self.spatial_canvas.pan_offset = egui::Vec2::ZERO;
                    }
                    if ui.button("🎯 Focus r₋").on_hover_text("Zoom and focus directly on the Cauchy horizon (r₋)").clicked() {
                        let rm = self.metric.inner_horizon();
                        self.spacetime_canvas.focus_horizon(rm);
                        self.spatial_canvas.zoom = 400.0;
                        self.spatial_canvas.pan_offset = egui::Vec2::new(- (rm as f32) * 400.0, 0.0);
                    }
                    if ui.button("🎯 Focus Bob").on_hover_text("Zoom and focus directly on Bob's current radius").clicked() {
                        self.spacetime_canvas.focus_bob(self.bob.r);
                        self.spatial_canvas.zoom = 400.0;
                        // Bob's screen position is the Kerr-Schild embedding of (r, phi); the canvas
                        // draws Cartesian y upward (screen y is flipped), so the centring pan is (-x, +y).
                        let (bx, by) = self.bob.cartesian_position(&self.metric);
                        self.spatial_canvas.pan_offset =
                            egui::Vec2::new(-(bx as f32) * 400.0, (by as f32) * 400.0);
                    }
                    ui.checkbox(&mut self.controls.use_km, "📏 Kilometers (km)");

                    // Font Size quick adjustment buttons
                    ui.label(format!("🔤 {:.0}%", self.controls.font_scale * 100.0));
                    if ui.button("➕").on_hover_text("Increase Font Size").clicked() {
                        self.controls.font_scale = (self.controls.font_scale + 0.1).clamp(0.7, 1.8);
                    }
                    if ui.button("➖").on_hover_text("Decrease Font Size").clicked() {
                        self.controls.font_scale = (self.controls.font_scale - 0.1).clamp(0.7, 1.8);
                    }
                    egui::ComboBox::from_id_salt("top_frame_selector")
                        .selected_text(match self.controls.frame_of_ref {
                            ReferenceFrame::DistantObserver => "🌐 Global Foliation",
                            ReferenceFrame::Bob => "👤 Bob's Frame",
                            ReferenceFrame::Alice => "👩 Alice's Frame",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.controls.frame_of_ref, ReferenceFrame::DistantObserver, "🌐 Global Foliation (Kerr-Schild)");
                            ui.selectable_value(&mut self.controls.frame_of_ref, ReferenceFrame::Bob, "👤 Bob's Rest Frame (45° Cones)");
                            ui.selectable_value(&mut self.controls.frame_of_ref, ReferenceFrame::Alice, "👩 Alice's Rest Frame (45° Cones)");
                        });
                    ui.label(egui::RichText::new("Ingoing Kerr-Schild Chart").small().color(Theme::TEXT_MUTED));
                });
            });
        });

        // 2. Bottom Panel: Cauchy Effects HUD
        egui::Panel::bottom("bottom_hud").default_size(68.0).show(ui, |ui| {
            CauchyEffects::render_hud(
                ui,
                &self.metric,
                &self.bob,
                &self.alice,
                &self.signal,
                &self.bob_signal,
                self.controls.delta_t_delay,
                self.current_time,
                self.controls.use_km,
            );
        });

        // 3. Left Dock Panel: Controls
        egui::Panel::left("controls_panel").default_size(300.0).show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                self.controls.render_panel(
                    ui,
                    &mut self.metric,
                    &mut self.bob,
                    &mut self.alice,
                    SignalPair {
                        alice: &mut self.signal,
                        bob: &mut self.bob_signal,
                    },
                    &mut self.current_time,
                );
            });
        });

        // 4. Central Panel: Split View between Spacetime (t, r) and Spatial (x, y)
        egui::CentralPanel::default().show(ui, |ui| {
            let avail = ui.available_size();
            let header_height = 24.0;
            let canvas_height = (avail.y - header_height - 10.0).max(250.0);
            let left_width = (avail.x * 0.53).max(200.0);
            let right_width = (avail.x - left_width - 12.0).max(200.0);

            let (left_title, left_sub) = match self.controls.frame_of_ref {
                ReferenceFrame::DistantObserver => ("GLOBAL FOLIATION (t, r)", "Ingoing Kerr-Schild (Smooth across r₊ & r₋)"),
                ReferenceFrame::Bob => ("BOB'S REST FRAME (45° CONES)", "Local Minkowski Space (c ≡ 1)"),
                ReferenceFrame::Alice => ("ALICE'S REST FRAME (45° CONES)", "Local Minkowski Space (c ≡ 1)"),
            };

            ui.horizontal(|ui| {
                // Left Column: Spacetime foliation (t, r) with aligned 1D track
                ui.allocate_ui_with_layout(
                    egui::Vec2::new(left_width, avail.y),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new(left_title).strong().color(Theme::HORIZON_OUTER));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                ui.label(egui::RichText::new(left_sub).small().color(Theme::BOB_COLOR));
                            });
                        });
                        self.spacetime_canvas.render(
                            ui,
                            &self.metric,
                            &mut self.bob,
                            &self.alice,
                            self.current_time,
                            canvas_height,
                            self.controls.use_km,
                            self.controls.frame_of_ref,
                            self.controls.font_scale,
                            SignalViews {
                                alice: &self.signal,
                                show_alice: self.controls.show_signal,
                                bob: &self.bob_signal,
                                show_bob: self.controls.show_bob_signal,
                            },
                            self.controls.show_outgoing_rays,
                        );
                    },
                );

                ui.separator();

                // Right Column: 2D Spatial disk (x, y)
                ui.allocate_ui_with_layout(
                    egui::Vec2::new(right_width, avail.y),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("EQUATORIAL PLANE (x, y)").strong().color(Theme::HORIZON_CAUCHY));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                ui.label(egui::RichText::new("Top-Down View").small().color(Theme::TEXT_MUTED));
                            });
                        });
                        self.spatial_canvas.render(
                            ui,
                            &self.metric,
                            &self.bob,
                            &self.alice,
                            self.controls.show_river,
                            self.controls.show_streamlines,
                            SignalViews {
                                alice: &self.signal,
                                show_alice: self.controls.show_signal,
                                bob: &self.bob_signal,
                                show_bob: self.controls.show_bob_signal,
                            },
                            canvas_height,
                            self.controls.use_km,
                            self.controls.frame_of_ref,
                            self.controls.font_scale,
                        );
                    },
                );
            });
        });

        // 5. Theory & Horizon Explanation Modal
        if self.controls.show_theory_modal {
            egui::Window::new("Relativistic Spacetime & Cauchy Horizon Theory")
                .open(&mut self.controls.show_theory_modal)
                .min_width(550.0)
                .show(&ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        ui.heading("Ingoing Kerr-Schild Coordinates & Horizon Dynamics");
                        ui.add_space(6.0);
                        ui.label(
                            "In standard Boyer-Lindquist coordinates, the coordinate time blows up at both the outer event horizon (r₊) and the inner Cauchy horizon (r₋) due to 1/Δ factors. \
                            By transforming to Ingoing Kerr-Schild coordinates, the metric tensor remains completely smooth, finite, and well-behaved across both horizons."
                        );
                        ui.add_space(8.0);
                        ui.heading("The Three Spacetime Regions");
                        ui.label(egui::RichText::new("1. Region I (r > r₊): Asymptotically Flat External Universe").strong().color(Theme::HORIZON_OUTER));
                        ui.label("The radial coordinate r is spacelike (g^rr > 0). Light cones are upright and observers can maneuver freely in both +r and -r directions.");
                        ui.add_space(4.0);

                        ui.label(egui::RichText::new("2. Crossing r₊: The Event Horizon (Δ = 0)").strong().color(Theme::HORIZON_OUTER));
                        ui.label("The outgoing boundary of Bob's light cone tilts to coordinate slope dr/dt ≤ 0. Outgoing light is frozen at the boundary.");
                        ui.add_space(4.0);

                        ui.label(egui::RichText::new("3. Region II (r₋ < r < r₊): The Inward-Trapped Interior").strong().color(Theme::WARNING_RED));
                        ui.label("Curvature tilts all light cones past the vertical (dr/dt < 0). The radial coordinate r is timelike (g^rr < 0). Any particle or photon MUST propagate toward smaller r.");
                        ui.add_space(4.0);

                        ui.label(egui::RichText::new("4. Crossing r₋: The Inner Cauchy Horizon (Δ = 0)").strong().color(Theme::HORIZON_CAUCHY));
                        ui.label("The light cone reaches maximum tilt and UN-TIPS! Outside this boundary, Δ > 0 again.");
                        ui.add_space(4.0);

                        ui.label(egui::RichText::new("5. Region III (0 < r < r₋): Inner Maneuverable Core").strong().color(Theme::BOB_COLOR));
                        ui.label("The radial coordinate r reverts to being spacelike again (g^rr > 0). Bob's light cone un-tips, allowing dr/dt ≥ 0, so in classical Kerr geometry thrusters can stop his descent. The ring singularity at r = 0 is timelike rather than spacelike, so it can be steered around; but the equatorial L = 0 infall drawn here is aimed straight at it, and this worldline still ends on it.");
                        ui.add_space(8.0);

                        ui.heading("The river model");
                        ui.label(
                            "The pale drops on the equatorial view are the raindrop congruence, E = 1 and L = 0, dropped from rest at infinity, and that congruence is the reference frame the whole app is built on: the Manual-drag boost β is defined against it, because it is the one frame that exists at every radius, inside the horizons included. Painlevé-Gullstrand time is the raindrop's own proper time, and Doran generalises that slicing to Kerr, which is why the drops spiral: L = 0 raindrops are still frame-dragged. The river's speed relative to the local ZAMO is β = √(1 − α²), which is √(2M/r) without spin and exactly 1 at r₊; Bob's own motion through the river is a boost of at most c on top of it, so inside r₊ the inward flow always wins, whatever the thrust. One caveat: the flat-space background the river picture paints is exact only under spherical symmetry. Hamilton and Lisle (Am. J. Phys. 76, 519, 2008) extend it to Kerr with a twisting tetrad, and in that construction the Doran background speed √(2Mr)/ρ reaches c at the ergosurface rather than at r₊; quoting the ZAMO-relative speed instead, as this view does, puts the horizon statement back into an invariant. Each drop is drawn at its proper size, the length along the flow and the width across it both obtained by projecting the separation of two nearby raindrops orthogonal to the four-velocity, so the stretching along the flow and the thinning across it are the tidal deformation of a fluid element of the river, taken exactly from the congruence rather than from the tidal tensor. Every drop enters the field at 12M as a circle of proper diameter 0.1M, and from there the flow alone deforms that circle: the length grows as √(12M/r) while the width shrinks as the flow lines converge, so the drawn aspect ratio is the spaghettification factor of the fluid element, about 16 at r₊ for a = 0.65. The length reports the Doran speed √(2M/r), which reaches c at the static limit 2M, while the colour reports the ZAMO-relative speed, which reaches c at r₊, so the two river speeds and the difference between the ergosurface and the horizon are both on screen at once. Near the ring the width grows again, because g_φφ → 2Ma²/r there and the ring is a circle of infinite proper circumference."
                        );
                        ui.add_space(8.0);

                        ui.heading("Alice's signal and the two branches of r₋");
                        ui.label(
                            "Alice's pulses are exact null geodesics, and she broadcasts each one into the whole of her light cone, every direction at once. Which rays of a pulse cross r₋ and which never do is decided by the sign of E − Ω₋L, the ray's energy relative to the null generator of the inner horizon, with Ω₋ = a/(r₋² + a²). Rays with positive relative energy fall straight through; rays with negative relative energy take infinite coordinate time and accumulate on r₋, so in this chart the inner horizon is the stack of all the outgoing light of the interior. In her own frame the accumulating rays are the prograde ones, the arc dragged forward in ϕ around α = 90°: it runs from about α = 45° to α = 135° well inside r₊, is wider than that just below r₊, and narrows as she approaches r₋, its edges being exactly where E − Ω₋L changes sign. Bob meets each pulse twice: first its crossing sheet sweeps over him on the way down with an ordinary shift, then he cuts through its frozen arc, standing on r₋, in the last twentieth of an M above the horizon. Alice sends a pulse every 0.1 M of her proper time, so consecutive arcs overlap and he crosses several sheets in a row, each blueshifted on the scale exp(κ₋Δt) with κ₋ = (r₊ − r₋)/(2(r₋² + a²)): about 560 for Δt = 4M and 3×10⁵ for Δt = 8M at a = 0.65, and a far gentler 4.7 and 22 at the app's default a = 0.90, where κ₋ is 0.386/M rather than 1.58/M. The light she sends as she crosses is shifted by exactly that factor; a pulse sent earlier by some lead time is shifted by exp(κ₋ × lead) more, having had that long to freeze as well. Each arc co-rotates at Ω₋ while it waits, so one emitter's transmission illuminates a band of r₋ rather than all of it, and how much of the stack Bob meets depends on where he crosses; the surface that covers every azimuth is built from the whole history of the interior. The ratio is finite because both observers cross the same smooth surface of exact Kerr. It diverges only as Δt → ∞, which is the Marolf and Ori (2012) statement that a hole which lives forever meets every late infaller with an outgoing null shock on this branch of r₋. The other branch, reached only as v → ∞, suffers Poisson and Israel mass inflation instead. Both make the exact continuation past r₋ physically untrustworthy, which is the content of strong cosmic censorship."
                        );
                        ui.add_space(8.0);

                        ui.heading("What an infaller sees near the Cauchy horizon");
                        ui.label(
                            "Ingoing light follows the principal null rays, lines of constant advanced time v = t + r, running at dr/dt = -1 everywhere in this chart. The ingoing Kerr-Schild chart is regular on the branch of r₋ that an infalling observer actually crosses, so Alice and Bob cross it at finite t and finite v: no signal stacks up there, and the exterior universe's whole future does not arrive as one flash. The shift they measure for that ingoing light is ν_obs/ν_∞ = -k·u = uᵗ + uʳ - a u^φ, finite and positive everywhere shown; for a Schwarzschild raindrop it is 1/(1 + √(2M/r)), exactly 1/2 at the horizon, a redshift, because running away from the light beats the gravitational blueshift. The infinite blueshift of Penrose and of Poisson-Israel lives on the OTHER branch of r₋, reached only as v → ∞, which this chart does not cover and which an infalling geodesic of finite v never reaches. In a real collapse that instability (mass inflation) is expected to turn r₋ into a singular surface, but that is a statement about the full spacetime, not about the worldlines drawn here."
                        );
                    });
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gui::controls::active_preset;
    use crate::physics::geodesic::GeodesicState;
    use eframe::App;

    #[test]
    fn test_canvas_layout_sizing() {
        let mut app = SpacetimeApp::default();
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
    }

    #[test]
    fn test_the_default_hole_is_the_sagittarius_a_star_preset() {
        // The highlight in the control panel's preset row is read off the metric, so this is also
        // the assertion that the app opens with that row lit.
        let app = SpacetimeApp::default();
        assert_eq!(active_preset(&app.metric), Some("Sagittarius A* (4.15M M☉)"));
        assert!((app.metric.a_star() - 0.90).abs() < 1e-12);
        assert!((app.metric.m_solar - 4.15e6).abs() < 1.0);
        // The horizons every other default-metric test is measured against: r+- = M +- sqrt(M^2 -
        // a^2) = 1 +- 0.43589.
        assert!((app.metric.outer_horizon() - 1.43589).abs() < 1e-5, "{}", app.metric.outer_horizon());
        assert!((app.metric.inner_horizon() - 0.56411).abs() < 1e-5, "{}", app.metric.inner_horizon());
        // 1 M is 6.1e6 km at this mass, which is what the km readouts and the Distance step mode
        // are scaled by.
        assert!((app.metric.r_grav_km() / 6.13e6 - 1.0).abs() < 0.01, "{}", app.metric.r_grav_km());

        // Distance step mode still has room to work at that scale. The default 1,000 km step is
        // 1.63e-4 M here rather than the 0.068 M it was at ten solar masses, and the arrow step it
        // implies sits well inside the [1e-8, 500] clamp instead of being pinned to an end of it.
        // The slider's own range is derived from r_grav (min = max(r_grav_km * 1e-6, 0.1) = 6.1 km,
        // max = max(r_grav_km * 2, 1e5) = 1.2e7 km), so all four quick-picks, 10 km included, stay
        // inside it.
        let mut app = app;
        assert!((app.metric.km_to_r(1000.0) / 1.63e-4 - 1.0).abs() < 0.01);
        app.controls.step_mode = StepMode::Distance;
        app.controls.step_distance_km = 1000.0;
        let step = app.arrow_step();
        assert!(step > 1e-8 && step < 500.0, "the distance step is clamped: {step}");
        assert!((app.metric.r_grav_km() * 1e-6) < 10.0, "the 10 km quick-pick is below the slider floor");
    }

    #[test]
    fn test_ton_618_preset() {
        let ton618 = KerrSchild::with_solar_mass(1.0, 0.88, 6.6e10);
        assert!((ton618.m_solar - 6.6e10).abs() < 1e5);
        assert!((ton618.a_star() - 0.88).abs() < 1e-4);
        let rp = ton618.outer_horizon();
        let formatted = ton618.format_physical_distance(rp);
        println!("TON 618 Event Horizon radius: {}", formatted);
        assert!(formatted.contains("AU") || formatted.contains("ly"));
    }

    #[test]
    fn test_arrow_key_stepping() {
        // The arrow keys through the app's own transport: one step forward and one back has to
        // land on the event it started from, on the clock and on the worldline alike.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        let initial = (app.bob.t, app.bob.r, app.bob.phi, app.bob.tau);
        let step = app.arrow_step();

        app.step_forward(step);
        assert!(app.bob.r < initial.1, "Stepping forward should advance inward infall");
        assert!((app.bob.t - app.current_time).abs() < 1e-12, "and stay on the clock");

        app.step_backward(step);
        assert!((app.current_time - initial.0).abs() < 1e-12, "the clock comes back: {}", app.current_time);
        let back = (app.bob.t, app.bob.r, app.bob.phi, app.bob.tau);
        for (a, b, what) in [
            (back.0, initial.0, "t"),
            (back.1, initial.1, "r"),
            (back.2, initial.2, "phi"),
            (back.3, initial.3, "tau"),
        ] {
            assert!((a - b).abs() < 1e-9, "stepping back must restore {what}: {a} vs {b}");
        }
    }

    /// (t, r, phi, tau) of an observer, the four numbers a rewind has to get right.
    fn observer_state(obs: &Observer) -> (f64, f64, f64, f64) {
        (obs.t, obs.r, obs.phi, obs.tau)
    }

    #[test]
    fn test_observers_rewind_with_the_clock() {
        // Three scenarios the old step-back got wrong, because it popped one recorded event per
        // call whatever interval the caller had undone. All three are run through the app's own
        // transport, `step_forward` / `step_backward`, which is what the arrow keys and the
        // panel's buttons call.

        // 1. A backstep smaller than the steps that built the trail. The clock moves by 0.1; the
        //    observers used to move by a whole 0.3 M step, or, for one that had already ended, by
        //    a jump back to whatever its last recorded event happened to be.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for _ in 0..20 {
            app.step_forward(0.3);
        }
        assert!((app.current_time - 6.0).abs() < 1e-9);
        let alice_before = observer_state(app.alice.as_ref().unwrap());
        let bob_before = observer_state(&app.bob);
        let alice_ended = app.alice.as_ref().unwrap().has_ended();
        let bob_ended = app.bob.has_ended();
        app.step_backward(0.1);
        println!(
            "back 0.1 from t = 6.0: clock {:.4}; Alice {:?} (ended {alice_ended}) -> {:?}; \
             Bob {:?} (ended {bob_ended}) -> {:?}",
            app.current_time,
            alice_before,
            observer_state(app.alice.as_ref().unwrap()),
            bob_before,
            observer_state(&app.bob)
        );
        assert!((app.current_time - 5.9).abs() < 1e-9);
        // An observer that has ended before the target does not move at all; one that has not is
        // on the clock, to the last bit.
        for (obs, before, ended) in [
            (app.alice.as_ref().unwrap(), alice_before, alice_ended),
            (&app.bob, bob_before, bob_ended),
        ] {
            if ended && before.0 <= app.current_time {
                assert_eq!(
                    observer_state(obs),
                    before,
                    "{} ended at t = {} and must not be moved by a rewind to t = {}",
                    obs.name,
                    before.0,
                    app.current_time
                );
            } else {
                assert!(
                    (obs.t - app.current_time).abs() < 1e-12,
                    "{} is at t = {} with the clock at {}",
                    obs.name,
                    obs.t,
                    app.current_time
                );
            }
        }

        // 2. An observer who reached the ring long before the clock did. A rewind that does not
        //    reach her death event must leave her on the ring; one that does must put her back on
        //    her worldline, on the clock.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for _ in 0..100 {
            app.step_forward(0.1);
        }
        assert!((app.current_time - 10.0).abs() < 1e-9);
        let alice = app.alice.as_ref().unwrap();
        assert!(alice.has_ended(), "Alice must have reached the ring: r = {}", alice.r);
        let ended_at = observer_state(alice);
        assert!(ended_at.0 < 7.0, "and done it well before t = 10: t_end = {}", ended_at.0);
        app.step_backward(0.1);
        println!(
            "Alice ended at t = {:.4}, r = {:.4}; after a backstep to t = {:.2} she is at \
             t = {:.4}, r = {:.4}",
            ended_at.0,
            ended_at.1,
            app.current_time,
            app.alice.as_ref().unwrap().t,
            app.alice.as_ref().unwrap().r
        );
        assert_eq!(
            observer_state(app.alice.as_ref().unwrap()),
            ended_at,
            "a rewind to t = {} is still past her end at t = {}: she stays on the ring",
            app.current_time,
            ended_at.0
        );
        // Now back past the end of her worldline: she comes off the ring, and onto the clock.
        while app.current_time > ended_at.0 - 0.25 {
            app.step_backward(0.1);
        }
        let alice = app.alice.as_ref().unwrap();
        assert!(!alice.has_ended(), "below her end she is falling again: r = {}", alice.r);
        assert!(alice.r > ended_at.1, "and above the ring: r = {}", alice.r);
        assert!(
            (alice.t - app.current_time).abs() < 1e-12,
            "on the clock: {} vs {}",
            alice.t,
            app.current_time
        );

        // 3. One backstep far longer than any step that built the trail, which is what Distance
        //    mode hands the transport at a supermassive hole. The observers used to move back a
        //    single recorded event and stay there, which is the "Alice is frozen in place on
        //    backstep" the user sees.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for _ in 0..20 {
            app.step_forward(0.3);
        }
        app.step_backward(4.0);
        assert!((app.current_time - 2.0).abs() < 1e-9);
        let wound = observer_state(app.alice.as_ref().unwrap());
        assert!((wound.0 - 2.0).abs() < 1e-12, "Alice lands on the clock: {}", wound.0);
        assert!((app.bob.t - 2.0).abs() < 1e-12, "and so does Bob: {}", app.bob.t);

        // And where it lands is the worldline, not merely the clock: compare with a fresh run to
        // the same time in different steps. The two differ only by the integrator, which is
        // stepping the same geodesic over differently cut intervals.
        let mut fresh = SpacetimeApp::default();
        fresh.controls.is_playing = false;
        for _ in 0..40 {
            fresh.step_forward(0.05);
        }
        let straight = observer_state(fresh.alice.as_ref().unwrap());
        println!(
            "one 4 M backstep vs a fresh run to t = 2: dr = {:.3e}, dphi = {:.3e}, dtau = {:.3e}",
            (wound.1 - straight.1).abs(),
            (wound.2 - straight.2).abs(),
            (wound.3 - straight.3).abs()
        );
        assert!((wound.1 - straight.1).abs() < 1e-9, "r: {} vs {}", wound.1, straight.1);
        assert!((wound.2 - straight.2).abs() < 1e-9, "phi: {} vs {}", wound.2, straight.2);
        assert!((wound.3 - straight.3).abs() < 1e-9, "tau: {} vs {}", wound.3, straight.3);
    }

    #[test]
    fn test_diagnostic_bob_alice() {
        let mut app = SpacetimeApp::default();
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            for frame_idx in 0..1500 {
                app.ui(ui, &mut frame);
                let al = app.alice.as_ref().unwrap();
                if frame_idx % 50 == 0 || (app.bob.r - al.r).abs() < 0.05 || app.bob.r < 1.0 {
                    println!("Frame {}: Bob r={:.6} t={:.4}, Alice r={:.6} t={:.4}, diff_r={:.6}", frame_idx, app.bob.r, app.bob.t, al.r, al.t, (app.bob.r - al.r).abs());
                }
                if app.bob.r <= 0.03 && al.r <= 0.03 {
                    println!("Both hit singularity at frame {}", frame_idx);
                    break;
                }
            }
        });
    }

    #[test]
    fn test_alice_signal_is_received_through_the_app_loop() {
        // The dual-observer layout the Drop Observers button builds: Alice released from r = 4.5M
        // at t = 0 and Bob held at the same radius until t = Delta t, so his worldline trails hers
        // and her signal climbs to him. This walks the app's own wiring rather than the physics
        // module: the field is advanced on the simulation clock, Alice emits on her proper
        // clock, Bob's receptions are detected, and both canvases and the HUD draw the result.
        //
        // Alice broadcasts into her whole cone, so a receiver below her hears her too: the ingoing
        // principal null ray of that cone runs at dr/dt = -1 in this chart, which no timelike
        // worldline can match, so it overtakes a raindrop that is already deeper. What such a
        // receiver never sees is the outward arc, which falls no faster than the raindrop
        // congruence itself.
        let mut app = SpacetimeApp::default();
        let params = WorldlineParams::default();
        app.alice = Some(Observer::new_with_phi(&app.metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
        app.bob = Observer::new_with_phi(&app.metric, "Bob", 0.0, 4.5, 4.0, 0.0, params);
        app.controls.delta_t_delay = 4.0;
        app.current_time = 0.0;
        app.signal.clear();
        app.bob_signal.clear();
        // Test frames arrive as fast as the harness can render them, so the frame clock sits at its
        // floor of 1/240 s; the playback rate is what buys enough simulation time to reach Bob's
        // crossing without running thousands of frames.
        app.controls.play_speed = 4.0;

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            for _ in 0..800 {
                app.ui(ui, &mut frame);
            }
        });

        assert!(!app.signal.pulses.is_empty(), "Alice should have pulses in flight");
        assert!(
            app.signal.received_count() >= 4,
            "Bob should have caught several of them by t = {}",
            app.current_time
        );
        let last = app.signal.last_reception().expect("a reception was just asserted");
        assert!(last.ratio.is_finite() && last.ratio > 0.0, "{last:?}");

        // `clear` is the reset, not the rewind: it drops the field outright, which is what
        // re-dropping the observers or changing the geometry needs. The rewind is
        // `test_stepping_back_through_the_app_keeps_the_signal`.
        app.signal.clear();
        assert_eq!(app.signal.received_count(), 0);
        assert_eq!(app.signal.t, 0.0);
    }

    #[test]
    fn test_stepping_back_through_the_app_keeps_the_signal() {
        // The step-back path used to call `SignalField::clear`, so every front on screen vanished
        // the moment the left arrow was pressed. It now runs the field backwards along the same
        // null geodesics it came in on, so the transmission survives the rewind and the field's own
        // clock stays locked to the simulation clock, which is what makes stepping back and forward
        // over the same interval land on the same picture.
        let mut app = SpacetimeApp::default();
        app.controls.play_speed = 4.0;
        app.controls.step_size = 0.01;

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            for _ in 0..200 {
                app.ui(ui, &mut frame);
            }
        });
        let forward_pulses = app.signal.pulses.len();
        assert!(forward_pulses > 0, "Alice should be transmitting by t = {}", app.current_time);
        assert!(
            (app.signal.t - app.current_time).abs() < 1e-9,
            "the field rides the simulation clock: {} vs {}",
            app.signal.t,
            app.current_time
        );

        let step = app.arrow_step();
        for _ in 0..50 {
            app.step_backward(step);
        }

        assert!(
            !app.signal.pulses.is_empty(),
            "stepping back must rewind the signal, not delete it"
        );
        assert!(
            (app.signal.t - app.current_time).abs() < 1e-9,
            "and must keep the field's clock on the simulation clock: {} vs {}",
            app.signal.t,
            app.current_time
        );
        // The observers came back with the clock too. Fifty presses of the left arrow at a
        // step of 0.01 undo half an M, and each press moves the worldlines by exactly what it
        // moves the clock by, whatever the steps that built the trail were.
        for obs in [Some(&app.bob), app.alice.as_ref()].into_iter().flatten() {
            assert!(
                obs.has_ended() || (obs.t - app.current_time).abs() < 1e-12,
                "{} is at t = {} with the clock at {}",
                obs.name,
                obs.t,
                app.current_time
            );
        }
        // Nothing survives that had not been emitted by the time stepped back to.
        for pulse in app.signal.pulses.iter() {
            assert!(
                pulse.emitted_t <= app.current_time + 1e-9,
                "a pulse emitted at t = {} is still in a field wound back to t = {}",
                pulse.emitted_t,
                app.current_time
            );
        }
    }

    #[test]
    fn test_signal_overlays_render_without_an_inner_horizon() {
        // A non-spinning hole has r- at the origin, so there is no Cauchy horizon for outgoing
        // light to accumulate on and kappa_- diverges. The overlays must draw nothing rather than
        // something misleading, and the HUD must print the divergence rather than a number, so this
        // walks a frame at a = 0 with both overlays on and again with both off.
        let schwarzschild = KerrSchild::with_solar_mass(1.0, 0.0, 10.0);
        assert!(!schwarzschild.inner_surface_gravity().is_finite());
        let mut app = SpacetimeApp::default();
        // The presets re-drop the observers when the geometry changes, because a 4-velocity
        // integrated in one metric is not a unit timelike vector in another; do the same here.
        let params = WorldlineParams::default();
        app.metric = schwarzschild;
        app.bob.reset_with_phi(&app.metric, 0.0, 3.8, 0.0, params);
        if let Some(al) = &mut app.alice {
            al.reset_with_phi(&app.metric, 0.0, 4.5, 0.25, params);
        }
        app.signal.clear();
        app.bob_signal.clear();
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
            app.controls.show_signal = false;
            app.controls.show_bob_signal = false;
            app.controls.show_outgoing_rays = false;
            app.ui(ui, &mut frame);
        });
    }

    #[test]
    fn test_font_scale_and_telemetry_sizing() {
        let mut app = SpacetimeApp::default();
        assert_eq!(app.controls.font_scale, 1.0);

        // Verify font scale limits
        app.controls.font_scale = 1.5;
        assert_eq!(app.controls.font_scale, 1.5);

        // Test running UI with enlarged font scale (150%)
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        // Test running UI with reduced font scale (70%)
        app.controls.font_scale = 0.7;
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
    }

    #[test]
    fn test_dual_observer_energy_and_angular_momentum_controls() {
        // The E / L sliders drive the dropped worldlines, and the panel renders both muted
        // hints: the energy-floor clamp and the outgoing-start caveat.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.energy = 0.92;
        app.controls.l_ang = 3.5;
        app.controls.outgoing_start = true;

        let floor = GeodesicState::energy_floor(&app.metric, 4.5, app.controls.l_ang);
        assert!(floor > app.controls.energy, "the test needs a clamped case: floor = {floor}");

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        // Building an observer from those settings clamps E up to the floor and leaves the
        // observer on a legal (turning-point) worldline.
        let params = WorldlineParams::new(app.controls.energy, app.controls.l_ang, app.controls.outgoing_start);
        app.bob.reset_with_phi(&app.metric, 0.0, 4.5, 0.0, params);
        let geo = app.bob.geodesic.expect("free-fall observers carry a geodesic state");
        assert!((geo.energy - floor).abs() < 1e-12, "E = {} vs floor {floor}", geo.energy);
        assert!((geo.l_ang - 3.5).abs() < 1e-12);
        assert!((app.metric.norm(4.5, &app.bob.four_velocity(&app.metric)) + 1.0).abs() < 1e-9);

        // Defaults stay the raindrop.
        let d = AppControls::default();
        assert_eq!((d.energy, d.l_ang, d.outgoing_start), (1.0, 0.0, false));
    }

    #[test]
    fn test_control_defaults() {
        let d = AppControls::default();
        // Kilometres are the default unit, and nothing throttles the step near r₋ any more:
        // the play speed and the step size are the only things that set sim_dt.
        assert!(d.use_km, "distances are shown in km out of the box");
        assert!(d.is_playing);
        assert_eq!(d.play_speed, 1.0);
        assert_eq!(d.step_size, 0.1);
    }

    #[test]
    fn test_km_telemetry_layout_renders_in_both_views() {
        // Both observers present, in kilometres, with the new one-metric-per-line info boxes and
        // Alice's light cone in the (t, r) diagram: the boxes register widgets on the canvas, so
        // this walks the interaction path as well as the paint path.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.use_km = true;
        app.controls.frame_of_ref = ReferenceFrame::DistantObserver;
        assert!(app.alice.is_some(), "the dual-observer default gives us Alice");

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            // Two passes: the second one reads back the widgets the first one registered.
            for _ in 0..2 {
                app.ui(ui, &mut frame);
            }
            // A ManualDrag observer is the non-geodesic case, which prints a_thrust and no E / L.
            app.bob.mode = crate::physics::observer::ObserverMode::ManualDrag;
            app.ui(ui, &mut frame);
        });

        assert!(app.controls.use_km);
    }

    #[test]
    fn test_fixed_distance_stepping() {
        // The default hole is Sagittarius A* (4.15e6 M_solar), where 1000 km is a fine step
        // (1.63e-4 M); no metric override is needed any more.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.step_mode = StepMode::Distance;
        app.controls.step_distance_km = 1000.0;

        let delta_r_m = app.metric.km_to_r(app.controls.step_distance_km);
        let v_coord = app.bob.velocity_c(&app.metric).abs().max(0.01);
        let sim_dt = (delta_r_m / v_coord).clamp(1e-8, 500.0);

        let initial_r = app.bob.r;
        app.bob.step(&app.metric, app.current_time + sim_dt, sim_dt);
        let actual_dr = (initial_r - app.bob.r).abs();
        let actual_dr_km = app.metric.r_to_km(actual_dr);

        // Verify that stepping by distance moves Bob by approximately 1000 km (within numerical integration tolerance)
        assert!(
            (actual_dr_km - 1000.0).abs() < 50.0,
            "Expected ~1000 km movement, got {:.2} km",
            actual_dr_km
        );
    }

    #[test]
    fn test_extreme_zoom_focus() {
        let mut app = SpacetimeApp::default();
        let rm = app.metric.inner_horizon();

        app.spacetime_canvas.focus_horizon(rm);
        assert!(app.spacetime_canvas.max_r <= 0.05);
        assert!((app.spacetime_canvas.r_offset - (rm - 0.025)).abs() < 0.01);

        // Test running UI in extreme zoom state
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
    }

    #[test]
    fn test_observer_rest_frame_renders_in_every_region() {
        // One frame in each reference frame with the observers parked in Region I, exactly on r+,
        // in Region II and in Region III. The rest-frame view places every surface with the dual
        // tetrad, so this walks the timelike / null / spacelike branches of that code, and the
        // Alice-less pass checks the fallback to Bob.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        let rp = app.metric.outer_horizon();
        let rm = app.metric.inner_horizon();
        let radii = [4.5, rp, 0.5 * (rp + rm), 0.5 * rm];

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            for &frame_of_ref in &[
                ReferenceFrame::DistantObserver,
                ReferenceFrame::Bob,
                ReferenceFrame::Alice,
            ] {
                app.controls.frame_of_ref = frame_of_ref;
                for &r in radii.iter() {
                    app.bob.reset_with_phi(&app.metric, 0.0, r, 0.0, WorldlineParams::default());
                    if let Some(ref mut al) = app.alice {
                        al.reset_with_phi(&app.metric, 0.0, r * 1.08, 0.35, WorldlineParams::default());
                    }
                    app.ui(ui, &mut frame);
                    assert!(app.bob.r > 0.0);
                }
            }

            app.controls.frame_of_ref = ReferenceFrame::Alice;
            app.alice = None;
            for &r in radii.iter() {
                app.bob.reset_with_phi(&app.metric, 0.0, r, 0.0, WorldlineParams::default());
                app.ui(ui, &mut frame);
            }
        });
    }

    #[test]
    fn test_bobs_signal_reaches_alice_and_rewinds_through_the_app_loop() {
        // Bob's transmission through the app's own wiring, in the layout Drop Observers builds and
        // at the default delay: Alice released from r = 4.5M at t = 0, Bob hovering at the same
        // radius until t = 8. He transmits throughout the wait - a static observer with a clock and
        // a frame - and this walks the whole loop over the hover: `SignalPair` advances his field
        // on the simulation clock, his proper time paces the emissions at sqrt(-g_tt) = 0.745 of
        // coordinate time, and Alice's worldline collects the arrivals.
        //
        // The clock is driven by `step_forward`, the app's own hand-stepping path and the one the
        // arrow keys use, rather than by played frames: a played frame's step is the wall-clock
        // frame time, which in a test harness is whatever the machine happens to give and is far
        // too coarse and too irregular to resolve a front sweeping over a worldline. One rendered
        // frame at the end walks both canvases and the HUD over the state that produced.
        //
        // Then the rewind, which for a hovering emitter has one more thing to undo than for a
        // falling one: his proper time. `Observer::rewind_to` puts a hover back on the closed form
        // sqrt(-g_tt) (t - t_start) that the forward hover accumulates, so the cadence comes back
        // exactly where it was and running forward again re-sends the same pulses at the same
        // events - which is the last thing this test measures.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.step_size = 0.02;
        app.controls.delta_t_delay = 8.0;
        let params = WorldlineParams::default();
        app.alice = Some(Observer::new_with_phi(&app.metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
        app.bob = Observer::new_with_phi(&app.metric, "Bob", 0.0, 4.5, 8.0, 0.0, params);
        app.current_time = 0.0;
        app.signal.clear();
        app.bob_signal.clear();

        let step = 0.02;
        for _ in 0..150 {
            app.step_forward(step);
        }
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        assert!(!app.bob.is_active, "Bob is still hovering at t = {}", app.current_time);
        assert!((app.bob.r - 4.5).abs() < 1e-12, "and has not moved: r = {}", app.bob.r);
        let heard = app.bob_signal.received_count();
        let emitted: Vec<(usize, f64, f64)> = app
            .bob_signal
            .pulses
            .iter()
            .map(|p| (p.index, p.emitted_t, p.emitted_tau))
            .collect();
        println!(
            "app loop to t = {:.2}: the hovering Bob has sent {} pulses, Alice has heard {heard}",
            app.current_time,
            emitted.len()
        );
        assert!(heard >= 3, "Alice should have caught several of his hover pulses: {heard}");
        assert!(emitted.len() >= 10, "and he should be transmitting steadily: {}", emitted.len());
        assert!(
            emitted.iter().all(|&(_, t, _)| t <= app.current_time + 1e-9),
            "no pulse can be dated after the clock: {emitted:?}"
        );
        assert!(
            (app.bob_signal.t - app.current_time).abs() < 1e-9,
            "Bob's field rides the simulation clock: {} vs {}",
            app.bob_signal.t,
            app.current_time
        );
        let last_arrival = app.bob_signal.last_reception().expect("just asserted").t;
        let forward_time = app.current_time;

        // A full M of coordinate time back, one arrow press at a time: past the last arrival, and
        // past several of his emissions.
        let back_steps = 50;
        for _ in 0..back_steps {
            app.step_backward(step);
        }
        assert!(
            app.current_time < last_arrival,
            "the rewind must go back past the arrival at t = {last_arrival}: now t = {}",
            app.current_time
        );
        assert!(!app.bob.is_active && (app.bob.r - 4.5).abs() < 1e-12, "he is still hovering");
        assert!(
            (app.bob.t - app.current_time).abs() < 1e-9,
            "and his own clock came back with the simulation's: {} vs {}",
            app.bob.t,
            app.current_time
        );

        // Alice is released and falling, so she is on the clock to the last bit as well.
        let alice = app.alice.as_ref().expect("the dual-observer layout gives us Alice");
        assert!(
            (alice.t - app.current_time).abs() < 1e-12,
            "Alice is at t = {} with the clock at {}",
            alice.t,
            app.current_time
        );
        assert!(
            !app.bob_signal.pulses.is_empty(),
            "stepping back must rewind Bob's signal, not delete it"
        );
        assert!(
            (app.bob_signal.t - app.current_time).abs() < 1e-9,
            "and must keep its clock on the simulation clock: {} vs {}",
            app.bob_signal.t,
            app.current_time
        );
        assert!(
            app.bob_signal.received_count() < heard,
            "the arrivals inside the rewound interval must be unrecorded: {} of {heard}",
            app.bob_signal.received_count()
        );
        assert!(
            app.bob_signal.pulses.len() + 5 <= emitted.len(),
            "and the pulses emitted inside it un-sent: {} of {}",
            app.bob_signal.pulses.len(),
            emitted.len()
        );
        for pulse in app.bob_signal.pulses.iter() {
            assert!(
                pulse.emitted_t <= app.current_time + 1e-9,
                "a pulse emitted at t = {} is still in a field wound back to t = {}",
                pulse.emitted_t,
                app.current_time
            );
            for reception in pulse.receptions.iter() {
                assert!(
                    reception.t <= app.current_time + 1e-9,
                    "an arrival recorded at t = {} survived a rewind to t = {}",
                    reception.t,
                    app.current_time
                );
            }
        }
        for reception in app.bob_signal.receptions() {
            assert!(
                reception.t <= app.current_time + 1e-9,
                "the field's own record of an arrival at t = {} survived the rewind",
                reception.t
            );
        }

        // Forward again over the same interval: the same pulses, at the same events. This is what
        // the analytic hover rewind buys - a hovering emitter's proper time is put back exactly, so
        // the cadence does not slip and the transmission is not re-cut at different events.
        for _ in 0..back_steps {
            app.step_forward(step);
        }
        assert!(
            (app.current_time - forward_time).abs() < 1e-9,
            "back to the same clock: {} vs {forward_time}",
            app.current_time
        );
        let again: Vec<(usize, f64, f64)> = app
            .bob_signal
            .pulses
            .iter()
            .map(|p| (p.index, p.emitted_t, p.emitted_tau))
            .collect();
        assert_eq!(
            again.len(),
            emitted.len(),
            "the second pass must re-send the same number of pulses: {again:?} vs {emitted:?}"
        );
        for (second, first) in again.iter().zip(emitted.iter()) {
            assert!(
                (second.1 - first.1).abs() < 1e-9 && (second.2 - first.2).abs() < 1e-9,
                "re-emitted at a different event: {second:?} vs {first:?}"
            );
        }
        println!(
            "round trip over {back_steps} steps: {} pulses re-sent at the same events",
            again.len()
        );
    }

    #[test]
    fn test_bobs_transmission_ends_when_alices_worldline_does() {
        // The feature's own statement, through the app loop: once Alice has reached the ring there
        // is a last pulse of Bob's that reached her, and everything he sends after it is sent to
        // nobody. The HUD and both canvases read exactly these two calls, and the frame drawn at
        // the end is the state that puts the ring on his worldline.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        let mut ended_at = None;
        for _ in 0..400 {
            app.step_forward(0.02);
            if ended_at.is_none() && app.alice.as_ref().is_some_and(|al| al.has_ended()) {
                ended_at = Some(app.current_time);
            }
        }
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        let alice = app.alice.as_ref().expect("the dual-observer default gives us Alice");
        assert!(alice.has_ended(), "the run must reach the end of Alice's worldline: r = {}", alice.r);
        let last = app
            .bob_signal
            .last_delivered_pulse()
            .expect("Bob starts below her and his light climbs to her from t = 0");
        let never = app.bob_signal.pulses_after(last.pulse_index);
        println!(
            "app loop: Alice ends at t = {:?}; last delivered #{} at t = {:.3}, r = {:.4},              Bob's tau = {:.3}; {never} later pulses never arrive",
            ended_at, last.pulse_index, last.emitted_t, last.emitted_r, last.emitted_tau
        );
        assert!(never >= 1, "Bob goes on transmitting after the last delivery: {never}");
        assert!(
            last.emitted_t < app.current_time,
            "the boundary event is in the past of the current frame"
        );
        for pulse in app.bob_signal.pulses.iter().filter(|p| p.index > last.pulse_index) {
            assert!(pulse.receptions.is_empty(), "pulse {} cannot have arrived", pulse.index);
        }
    }
}
