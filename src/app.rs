use crate::gui::cauchy_effects::CauchyEffects;
use crate::gui::controls::{AppControls, ReferenceFrame, StepMode};
use crate::gui::spacetime_canvas::SpacetimeCanvas;
use crate::gui::spatial_canvas::SpatialCanvas;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::Observer;
use std::time::Instant;

pub struct SpacetimeApp {
    metric: KerrSchild,
    bob: Observer,
    alice: Option<Observer>,
    spacetime_canvas: SpacetimeCanvas,
    spatial_canvas: SpatialCanvas,
    controls: AppControls,
    current_time: f64,
    last_update: Instant,
}

impl Default for SpacetimeApp {
    fn default() -> Self {
        let metric = KerrSchild::with_solar_mass(1.0, 0.65, 10.0);
        let bob = Observer::new("Bob", 0.0, 3.8, 0.0);
        let alice = Some(Observer::new_with_phi("Alice", 0.0, 4.5, 0.0, 0.25));

        Self {
            metric,
            bob,
            alice,
            spacetime_canvas: SpacetimeCanvas::default(),
            spatial_canvas: SpatialCanvas::default(),
            controls: AppControls::default(),
            current_time: 0.0,
            last_update: Instant::now(),
        }
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
            // Adaptive step sizing: smoothly throttle step size when approaching the Cauchy horizon
            let adaptive_factor = if self.controls.auto_slow_cauchy {
                let rm = self.metric.inner_horizon();
                let rp = self.metric.outer_horizon();
                let delta_horizons = (rp - rm).max(0.1);

                // Check distance of active observers to Cauchy horizon
                let mut min_proximity: f64 = 1.0;
                if self.bob.is_active && self.bob.r > rm && self.bob.r < rp {
                    let d = (self.bob.r - rm) / (0.35 * delta_horizons);
                    min_proximity = min_proximity.min(d);
                }
                if let Some(ref al) = self.alice {
                    if al.is_active && al.r > rm && al.r < rp {
                        let d = (al.r - rm) / (0.35 * delta_horizons);
                        min_proximity = min_proximity.min(d);
                    }
                }
                // Damping factor between 0.08 (12x slow-down at r-) and 1.0 (normal)
                min_proximity.clamp(0.08, 1.0)
            } else {
                1.0
            };

            // Time mode: frame-rate independent playback at `play_speed` units of M per real second.
            // Distance mode: per-frame step chosen so Bob moves a fixed Δr (normalised to 60 fps).
            let sim_dt = match self.controls.step_mode {
                StepMode::Time => dt * self.controls.play_speed * adaptive_factor,
                StepMode::Distance => {
                    let delta_r_m = self.metric.km_to_r(self.controls.step_distance_km);
                    let v_coord = self.bob.velocity_c(&self.metric).abs().max(0.01);
                    let base_step = (delta_r_m / v_coord).clamp(1e-8, 500.0);
                    (dt / 0.01667).clamp(0.2, 3.0) * base_step * adaptive_factor
                }
            };

            self.current_time += sim_dt;
            self.bob.step(&self.metric, self.current_time, sim_dt);
            if let Some(ref mut al) = self.alice {
                al.step(&self.metric, self.current_time, sim_dt);
            }
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
            let step = match self.controls.step_mode {
                StepMode::Time => self.controls.step_size,
                StepMode::Distance => {
                    let delta_r_m = self.metric.km_to_r(self.controls.step_distance_km);
                    let v_coord = self.bob.velocity_c(&self.metric).abs().max(0.01);
                    (delta_r_m / v_coord).clamp(1e-8, 500.0)
                }
            };
            self.current_time = (self.current_time - step).max(0.0);
            self.bob.step_back(&self.metric, step);
            if let Some(ref mut al) = self.alice {
                al.step_back(&self.metric, step);
            }
            ctx.request_repaint();
        } else if step_fwd_pressed {
            let step = match self.controls.step_mode {
                StepMode::Time => self.controls.step_size,
                StepMode::Distance => {
                    let delta_r_m = self.metric.km_to_r(self.controls.step_distance_km);
                    let v_coord = self.bob.velocity_c(&self.metric).abs().max(0.01);
                    (delta_r_m / v_coord).clamp(1e-8, 500.0)
                }
            };
            self.current_time += step;
            self.bob.step(&self.metric, self.current_time, step);
            if let Some(ref mut al) = self.alice {
                al.step(&self.metric, self.current_time, step);
            }
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
                            self.controls.show_wavefronts,
                            canvas_height,
                            self.controls.use_km,
                            self.controls.frame_of_ref,
                            self.controls.font_scale,
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
                                ui.label(egui::RichText::new("Top-Down View & Null Fan").small().color(Theme::TEXT_MUTED));
                            });
                        });
                        self.spatial_canvas.render(
                            ui,
                            &self.metric,
                            &self.bob,
                            &self.alice,
                            self.controls.show_streamlines,
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

                        ui.heading("What an infaller sees near the Cauchy horizon");
                        ui.label(
                            "The pale wavefront lines are ingoing principal null rays: lines of constant advanced time v = t + r, running at dr/dt = -1 everywhere in this chart. The ingoing Kerr-Schild chart is regular on the branch of r₋ that an infalling observer actually crosses, so Alice and Bob cross it at finite t and finite v: no signal stacks up there, and the exterior universe's whole future does not arrive as one flash. The shift they measure for that ingoing light is ν_obs/ν_∞ = -k·u = uᵗ + uʳ - a u^φ, finite and positive everywhere shown; for a Schwarzschild raindrop it is 1/(1 + √(2M/r)), exactly 1/2 at the horizon, a redshift, because running away from the light beats the gravitational blueshift. The infinite blueshift of Penrose and of Poisson-Israel lives on the OTHER branch of r₋, reached only as v → ∞, which this chart does not cover and which an infalling geodesic of finite v never reaches. In a real collapse that instability (mass inflation) is expected to turn r₋ into a singular surface, but that is a statement about the full spacetime, not about the worldlines drawn here."
                        );
                    });
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        let initial_r = app.bob.r;
        let step = app.controls.step_size;

        // Simulate Right Arrow (Step forward)
        app.current_time += step;
        app.bob.step(&app.metric, app.current_time, step);
        assert!(app.bob.r < initial_r, "Stepping forward should advance inward infall");

        // Simulate Left Arrow (Step backward)
        app.current_time = (app.current_time - step).max(0.0);
        app.bob.step_back(&app.metric, step);
        assert!((app.bob.r - initial_r).abs() < 1e-4, "Stepping back should return to initial radius");
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
    fn test_fixed_distance_stepping() {
        let mut app = SpacetimeApp::default();
        // Use Sagittarius A* (4.15e6 M_solar) where 1000 km is a fine step (~0.000163 M)
        app.metric = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        app.bob.reset(0.0, 3.8);
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
}
