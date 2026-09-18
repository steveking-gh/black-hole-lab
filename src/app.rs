use crate::gui::cauchy_effects::CauchyEffects;
use crate::gui::controls::{
    AppControls, DISTANT_CLOCK_GRID_TIP, GLOBAL_VOLUME_TIP, OPENING_SPIN, ReferenceFrame, SignalViews,
    StepMode, VIEW_TIP,
};
use crate::gui::spacetime_canvas::{KEEP_SURFACE_FRAMED_TIP, REST_FRAME_TIP, SpacetimeCanvas};
use crate::gui::spatial_canvas::{FrontStyle, SpatialCanvas};
use crate::gui::volume_canvas::VolumeCanvas;
use crate::gui::theme::Theme;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverPair};
use crate::physics::wavefront::{Endpoint, SignalField, SignalPair};
use std::time::Instant;

/// The height of the View row that heads the foliation column: one combo box and one checkbox,
/// plus the spacing egui puts between rows. It is taken out of the canvas height so that the row is
/// added above the picture rather than pushing the foot of it out of the window.
const FRAME_ROW_HEIGHT: f32 = 26.0;

pub struct SpacetimeApp {
    metric: KerrSchild,
    /// The two observers, either of whom may be out of the simulation: the "Enable Observer" box
    /// on a card unticked means there is no worldline at all rather than a hidden one, and the two
    /// are optional in the same way because the cards are the same card twice.
    bob: Option<Observer>,
    alice: Option<Observer>,
    spacetime_canvas: SpacetimeCanvas,
    /// The same foliation drawn as a volume: the equatorial plane as a floor and coordinate time
    /// standing up out of it. It stands in for `spacetime_canvas` in the left column while the View
    /// selector names `GlobalVolume`, and keeps its own camera, window and telemetry boxes while it
    /// is not being drawn, so that switching back and forth does not throw the view away.
    volume_canvas: VolumeCanvas,
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
        let metric = KerrSchild::with_solar_mass(1.0, OPENING_SPIN, 4.15e6);

        // The layout the app opens on is built by the same call the transport's Reset and Drop
        // Observers buttons make, from the same two cards, so the three cannot disagree about
        // where a run starts: Alice in free fall on the prograde ISCO, Bob a raindrop from
        // r = 4.5M, both let go at once.
        let mut controls = AppControls::default();
        let (mut alice, mut bob) = (None, None);
        let mut signal = SignalField::default();
        let mut bob_signal = SignalField::default();
        let mut current_time = 0.0;
        controls.drop_observers(
            &metric,
            &mut alice,
            &mut bob,
            &mut SignalPair { alice: &mut signal, bob: &mut bob_signal },
            &mut current_time,
        );
        // A view that has never been panned has nothing to put back; the request a drop raises is
        // for the buttons, which are pressed part-way through a run.
        controls.view_reset_requested = false;

        Self {
            metric,
            bob,
            alice,
            spacetime_canvas: SpacetimeCanvas::default(),
            volume_canvas: VolumeCanvas::default(),
            spatial_canvas: SpatialCanvas::default(),
            signal,
            bob_signal,
            controls,
            current_time,
            last_update: Instant::now(),
        }
    }
}

impl SpacetimeApp {
    /// Carry both transmissions forward by dt of the simulation clock. `SignalPair::advance` owns
    /// the order - carry the light, then emit, then listen - and the panel's Step Fwd button goes
    /// through the same call, so the two paths cannot drift apart.
    fn advance_signal(&mut self, dt: f64) {
        let mut signals = SignalPair {
            alice: &mut self.signal,
            bob: &mut self.bob_signal,
        };
        // The panel's Wavefront points slider is a standing request about the next emission, so it
        // is pushed into both fields here, on the way into the step, rather than at the click that
        // moved it: that way a played frame, an arrow key and a test that steps the app by hand all
        // emit at the count the panel is currently showing, and the two transmissions cannot end up
        // sampled differently. It reaches the emission and nothing else - pulses already in flight
        // keep their own count.
        signals.set_rays_per_pulse(self.controls.rays_per_pulse);
        signals.advance(
            &self.metric,
            dt,
            Endpoint {
                observer: self.alice.as_ref(),
                transmitting: self.controls.alice.transmit,
            },
            Endpoint {
                observer: self.bob.as_ref(),
                transmitting: self.controls.bob.transmit,
            },
        );
    }

    /// What one press of an arrow key is worth in coordinate time: the Δt slider in Time mode, in
    /// Distance mode the time an observer who is moving needs to cover the requested Δr at their
    /// current coordinate speed, and in Watch mode the coordinate time the focus observer's own
    /// watch spends the slider's Δτ over.
    ///
    /// Which observer paces a distance step, and what happens when nobody is moving in r - a
    /// hovering Bob, or a Static or ZAMO Bob holding his radius, has no time in which he covers
    /// Δr, and there is no honest number to divide by - is `AppControls::distance_step`; the
    /// conversion of a watch tick, and the cap on it, is `AppControls::watch_step`. The panel's
    /// Step Back and Step Fwd buttons and the play loop ask the same `step_for`, so a keypress, a
    /// click and a played frame cannot mean different intervals.
    fn arrow_step(&self) -> f64 {
        self.controls.step_for(
            &self.metric,
            self.bob.as_ref(),
            self.alice.as_ref(),
            self.controls.step_size,
        )
    }

    /// One step forward by hand, in the same order as a played frame.
    fn step_forward(&mut self, step: f64) {
        self.current_time += step;
        // Both worldlines move as one object, so that this path, the play loop and the panel's
        // buttons cannot mean different things by a step. See `ObserverPair`.
        ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
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
    /// The worldlines go first and the fields second. A worldline needs nothing from a field to be
    /// wound back, and a field needs its receiver at the rewound state to re-establish which side
    /// of each wavefront they stand on - `SignalPair::step_back` does that priming, without which
    /// a crossing that happens inside the next step forward is never seen. The panel's Step Back
    /// button runs the same two calls in the same order.
    fn step_backward(&mut self, step: f64) {
        // The clock stops at t = 0, and again at the oldest event the recorded worldlines can
        // still be put back on: `ObserverPair::rewind_floor` is zero until a run is long enough to
        // evict the start of a trail, and that trail's oldest event afterwards. Whatever is wound
        // back is wound back by however much of the step is left above that floor, which is what
        // keeps the field's clock equal to the simulation clock and both worldlines on both.
        let floor = ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
            .rewind_floor(&self.metric);
        let back = step.min(self.current_time - floor).max(0.0);
        if back <= 0.0 {
            return;
        }
        self.current_time -= back;
        ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
            .rewind_to(&self.metric, self.current_time);
        SignalPair {
            alice: &mut self.signal,
            bob: &mut self.bob_signal,
        }
        .step_back(&self.metric, back, self.alice.as_ref(), self.bob.as_ref());
    }
}

impl eframe::App for SpacetimeApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // Calculate frame delta time
        let now = Instant::now();
        let dt = (now - self.last_update).as_secs_f64().clamp(1.0 / 240.0, 0.1);
        self.last_update = now;

        // Nothing to report about a watch that is not being kept: the readout is written below
        // only by a frame that is actually played in Watch mode.
        self.controls.achieved_watch_rate = None;

        // The Wavefronts kept slider is a statement about what is on the screen, not a request
        // about the next emission, so it goes in here - once a frame, before the played step and
        // before anything is drawn - rather than on the stepping path the ray count takes. That is
        // what makes lowering it bite while the run is paused, which is when a user reaching for it
        // to thin a crowded picture is most likely to be. Raising it only widens the window from
        // here on: the pulses already evicted are gone. See `SignalField::max_pulses`.
        SignalPair { alice: &mut self.signal, bob: &mut self.bob_signal }
            .set_max_pulses(&self.metric, self.controls.max_pulses);

        // Advance simulation if playing
        if self.controls.is_playing {
            // Time mode: frame-rate independent playback at `play_speed` units of M per real second.
            // Distance mode: per-frame step chosen so Bob moves a fixed Δr (normalised to 60 fps).
            // Watch mode: `play_speed` units of M per real second of the *focus observer's* proper
            // time, so the rest-frame view runs at 1 s/s of the watch it is drawn for.
            // Nothing throttles the step near r₋ but Watch mode's own cap: to study the crossing,
            // pause and step by hand.
            let base = dt * self.controls.play_speed;
            let mut sim_dt = self.controls.step_for(
                &self.metric,
                self.bob.as_ref(),
                self.alice.as_ref(),
                base,
            );
            match self.controls.step_mode {
                // A distance step is a fixed Δr however long the frame took, so it is the one
                // mode whose step does not already carry the frame's own dt: it is normalised to
                // 60 fps here instead, which is what keeps a slow frame from crawling.
                StepMode::Distance => sim_dt *= (dt / 0.01667).clamp(0.2, 3.0),
                // What that step was worth on the watch it was asked for in. The same
                // `watch_step` `step_for` just used, so the figure reported is the step taken.
                StepMode::Watch => {
                    let watch = self.controls.watch_step(
                        &self.metric,
                        self.bob.as_ref(),
                        self.alice.as_ref(),
                        base,
                    );
                    let asked = watch.u_t * base;
                    self.controls.achieved_watch_rate =
                        Some(if asked > 0.0 { watch.dt / asked } else { 1.0 });
                }
                StepMode::Time => {}
            }

            self.current_time += sim_dt;
            ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
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

        // Nothing in the UI prints below the size a telemetry box titles itself at. egui's own
        // styles are fixed points, so the canvases had grown past them: its default Small is 9 pt
        // against the 10 pt the boxes now use, and the `.small()` captions on an observer card came
        // out smaller than anything beside them. The floor tracks the Font Size slider, so raising
        // that raises the panel with the canvases instead of only half the window.
        //
        // Each size is recomputed from egui's defaults rather than clamped in place, or lowering
        // the slider again would leave every style stuck at its high-water mark.
        {
            let floor = Theme::MIN_FONT_PT * self.controls.font_scale;
            let defaults = egui::Style::default().text_styles;
            for theme in [egui::Theme::Dark, egui::Theme::Light] {
                ctx.style_mut_of(theme, |style| {
                    for (key, font) in style.text_styles.iter_mut() {
                        if let Some(base) = defaults.get(key) {
                            font.size = base.size.max(floor);
                        }
                    }
                });
            }
        }

        // Apply dark relativity theme styling
        let mut visuals = egui::Visuals::dark();
        visuals.override_text_color = Some(Theme::TEXT_BRIGHT);
        visuals.panel_fill = Theme::PANEL_BG;
        ctx.set_visuals(visuals);

        // 1. Top Panel. The app's name and nothing else. It used to carry a strip of controls -
        // the frame selector, the units and font-size widgets, the Theory Guide button - every one
        // of which the left panel also has, and three view buttons (Reset Zoom, Focus r-, Focus
        // Bob) that the user never reached for: the equatorial view's right-click menu centres on
        // either observer and on the hole, and both canvases zoom on the wheel.
        egui::Panel::top("top_bar").show(ui, |ui| {
            ui.label(egui::RichText::new("BLACK HOLE LAB").heading().strong().color(Theme::HORIZON_OUTER));
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
                self.controls.release_gap(),
                self.current_time,
                self.controls.use_physical_units,
            );
        });

        // Where the observers actually stand, folded into the cards, before anything reads a card.
        //
        // The order is the whole of it. While the clock reads zero the card and the observer are
        // the same thing, and two things can move one of them: a slider on the panel, which writes
        // the card, and a marker drag on a canvas, which writes the observer. Taking the observers'
        // own positions first means the drag of the previous frame is part of the card by the time
        // the panel enforces it, so the card never reverts a drag; and the panel's own changes take
        // effect in the same frame they are made, so a slider never springs back. Reversed - which
        // is where this call used to sit, after the canvases - the two fight, and which one wins
        // depends on which of them the user touched last, which is exactly what neither of them can
        // see. See `AppControls::remember_drop_positions` and `ObserverCard::describes`.
        self.controls.remember_drop_positions(
            self.alice.as_ref(),
            self.bob.as_ref(),
            self.current_time,
        );

        // 3. Left Dock Panel: Controls
        egui::Panel::left("controls_panel").default_size(300.0).show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                self.controls.render_panel(
                    ui,
                    &mut self.metric,
                    &mut self.alice,
                    &mut self.bob,
                    SignalPair {
                        alice: &mut self.signal,
                        bob: &mut self.bob_signal,
                    },
                    &mut self.current_time,
                );
            });
        });
        // Reset and a preset change both put the clock back to zero, and the (t, r)
        // canvas is panned in time by an offset from the clock, so the pan has to go back with it
        // or the user is left looking at an empty stretch of diagram above the run they have just
        // restarted. The panel raises the request and it is taken here, before the canvases are
        // drawn, so the reset shows in the same frame as the click.
        //
        // Only the pan in time is put back. The radial pan and the zoom are a choice about what
        // part of the geometry to look at - the Focus r- and Focus Bob buttons in the header set
        // them deliberately - and none of these actions changes the geometry the way it changes the
        // clock; Reset Zoom in the header is what puts those back.
        if self.controls.take_view_reset() {
            self.spacetime_canvas.time_offset = 0.0;
            // The volume view is panned in time the same way, by an offset from the same clock, so
            // it goes back with it whether or not it is the picture currently on screen.
            self.volume_canvas.time_offset = 0.0;
            // The observers under the pointer have just been replaced by fresh ones, so a drag of
            // the old worldline is not carried into the new run: both markers are pickable again
            // from the moment the reset lands.
            self.spatial_canvas.end_drag();
        }

        // 4. Central Panel: Split View between Spacetime (t, r) and Spatial (x, y)
        egui::CentralPanel::default().show(ui, |ui| {
            let avail = ui.available_size();
            // One row above each canvas: the frame selector on the left, the plain title on the
            // right. Both canvases are given the same height, so the height taken out is the
            // taller of the two headers - the combo box - and the equatorial view carries the few
            // points of difference as slack at its foot.
            let header_height = FRAME_ROW_HEIGHT;
            let canvas_height = (avail.y - header_height - 10.0).max(250.0);
            let left_width = (avail.x * 0.53).max(200.0);
            let right_width = (avail.x - left_width - 12.0).max(200.0);

            ui.horizontal(|ui| {
                // Left Column: Spacetime foliation (t, r) with aligned 1D track
                ui.allocate_ui_with_layout(
                    egui::Vec2::new(left_width, avail.y),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| {
                        // Which picture of the spacetime this column draws: one of the two charts
                        // of the global foliation, or one observer's rest frame. It sits on the
                        // view it governs rather than on the control panel: every other thing the
                        // choice changes - the axes, the light cones, the clock grid - is in this
                        // column. It is also the only title this column has. There used to be a row
                        // under it naming the frame again in cyan, which said nothing the selected
                        // item does not, and in a rest frame said nothing the canvas's own banner
                        // does not either.
                        //
                        // The two global charts come first and the two rest frames after them,
                        // because the volume is a chart of the same foliation as the item above it
                        // and not a frame of reference at all.
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("View:").small().color(Theme::TEXT_MUTED))
                                .on_hover_text(VIEW_TIP);
                            egui::ComboBox::from_id_salt("frame_of_ref_foliation_combo")
                                .selected_text(self.controls.frame_of_ref.label())
                                // Wide enough for the longest label, "Global Foliation Chart 1D+1
                                // (Kerr-Schild)", so the selected item is never cut off.
                                .width(290.0)
                                .show_ui(ui, |ui| {
                                    for frame in [
                                        ReferenceFrame::DistantObserver,
                                        ReferenceFrame::GlobalVolume,
                                        ReferenceFrame::Bob,
                                        ReferenceFrame::Alice,
                                    ] {
                                        let item = ui.selectable_value(
                                            &mut self.controls.frame_of_ref,
                                            frame,
                                            frame.label(),
                                        );
                                        match frame {
                                            ReferenceFrame::GlobalVolume => {
                                                item.on_hover_text(GLOBAL_VOLUME_TIP);
                                            }
                                            ReferenceFrame::Bob | ReferenceFrame::Alice => {
                                                item.on_hover_text(REST_FRAME_TIP);
                                            }
                                            ReferenceFrame::DistantObserver => {}
                                        }
                                    }
                                });
                            ui.checkbox(&mut self.controls.show_distant_clock_grid, "Distant clock grid")
                                .on_hover_text(DISTANT_CLOCK_GRID_TIP);
                            // Only the rest frames have a window of their own to keep: the surface
                            // the observer is about to reach, kept on the canvas. Either global
                            // chart's zoom is a window the user pans - on r in the (t, r) diagram,
                            // on x and y in the volume - with nobody at its origin and nothing
                            // ahead of anybody in particular to frame, so there the box is not
                            // offered at all.
                            if matches!(
                                self.controls.frame_of_ref,
                                ReferenceFrame::Bob | ReferenceFrame::Alice
                            ) {
                                ui.checkbox(
                                    &mut self.spacetime_canvas.keep_surface_framed,
                                    "Auto-zoom",
                                )
                                .on_hover_text(KEEP_SURFACE_FRAMED_TIP);
                            }
                        });
                        // One picture at a time: the volume replaces the flat diagram rather than
                        // being squeezed in beside it, since they are two charts of the same
                        // foliation and the column is only wide enough for one of them to be read.
                        if self.controls.frame_of_ref == ReferenceFrame::GlobalVolume {
                            self.volume_canvas.render(
                                ui,
                                &self.metric,
                                self.bob.as_ref(),
                                self.alice.as_ref(),
                                self.current_time,
                                canvas_height,
                                self.controls.use_physical_units,
                                self.controls.font_scale,
                                SignalViews { alice: &self.signal, bob: &self.bob_signal },
                                self.controls.show_distant_clock_grid,
                                FrontStyle {
                                    arcs: self.controls.draw_front_arcs,
                                    hide_wound: self.controls.hide_wound_segments,
                                },
                            );
                        } else {
                            self.spacetime_canvas.render(
                                ui,
                                &self.metric,
                                self.bob.as_ref(),
                                self.alice.as_ref(),
                                self.current_time,
                                canvas_height,
                                self.controls.use_physical_units,
                                self.controls.frame_of_ref,
                                self.controls.font_scale,
                                SignalViews { alice: &self.signal, bob: &self.bob_signal },
                                self.controls.show_distant_clock_grid,
                            );
                        }
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
                            &mut self.bob,
                            &mut self.alice,
                            self.current_time,
                            SignalViews { alice: &self.signal, bob: &self.bob_signal },
                            canvas_height,
                            self.controls.use_physical_units,
                            self.controls.frame_of_ref,
                            self.controls.font_scale,
                            FrontStyle {
                                arcs: self.controls.draw_front_arcs,
                                hide_wound: self.controls.hide_wound_segments,
                            },
                            &mut self.controls.show_spatial_details,
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
    use crate::gui::controls::{WATCH_DT_CAP, active_preset};
    use crate::physics::geodesic::GeodesicState;
    use crate::physics::observer::{ObserverMode, Release, WorldlineParams};
    use crate::physics::wavefront::Pulse;
    use eframe::App;

    /// Bob, in a test that has him in the simulation. Either observer's card can be unticked now,
    /// which takes them out of the run entirely, so the tests say once - here - that they expect
    /// him to be there instead of unwrapping the Option on every line.
    fn bob_of(app: &SpacetimeApp) -> &Observer {
        app.bob.as_ref().expect("Bob's card is ticked in this test")
    }

    /// Alice, the same way.
    fn alice_of(app: &SpacetimeApp) -> &Observer {
        app.alice.as_ref().expect("Alice's card is ticked in this test")
    }

    /// Rebuild the run from the two cards, which is what ⏮ Reset and the app's own startup do. A test that wants a layout other than the default one sets the cards and
    /// calls this, rather than assembling observers by hand behind the panel's back.
    fn drop_observers(app: &mut SpacetimeApp) {
        let SpacetimeApp { metric, alice, bob, signal, bob_signal, controls, current_time, .. } =
            app;
        controls.drop_observers(
            metric,
            alice,
            bob,
            &mut SignalPair { alice: signal, bob: bob_signal },
            current_time,
        );
    }

    /// The layout most of these tests were written against: both observers dropped as raindrops,
    /// Alice released at once and Bob 8 M of coordinate time later, so he trails her down the same
    /// infall and hovers while he waits.
    ///
    /// It is no longer what the app opens on - Alice circles the prograde ISCO and Bob falls at
    /// once - so a test that needs somebody falling in r, or needs the trailing hover,
    /// asks for it here rather than inheriting it from the defaults. The tests that are *about* the
    /// defaults do not call this: `test_control_defaults` and
    /// `test_startup_reset_and_drop_observers_all_build_the_same_layout` state them directly.
    fn trailing_raindrops(app: &mut SpacetimeApp) {
        // Stated in full rather than inherited: a test that needs this layout gets it whatever
        // the app happens to open on.
        raindrop_cards(app);
        app.controls.bob.delta_t_delay = 8.0;
        drop_observers(app);
        set_free_fall(app);
    }

    /// Both cards as raindrops from 4.5 M: E = 1, L = 0, released from infinity, in free fall,
    /// no delay. The azimuths are left as the cards have them.
    fn raindrop_cards(app: &mut SpacetimeApp) {
        for card in [&mut app.controls.alice, &mut app.controls.bob] {
            card.release = Release::FromInfinity;
            card.l_ang = 0.0;
            card.drop_r = 4.5;
            card.mode = ObserverMode::FreeFall;
            card.delta_t_delay = 0.0;
        }
    }

    /// The layout the app used to open on, stated in full: both raindrops from 4.5 M let go at
    /// once, Bob falling and Alice holding the ZAMO circle at the drop radius.
    fn zamo_alice_falling_bob(app: &mut SpacetimeApp) {
        raindrop_cards(app);
        app.controls.alice.mode = ObserverMode::Zamo;
        drop_observers(app);
        // Motion is the one thing a re-drop inherits from the observer being replaced rather
        // than reading off the card, so it is set on the observers themselves, as
        // `set_free_fall` does.
        if let Some(alice) = app.alice.as_mut() {
            alice.mode = ObserverMode::Zamo;
        }
        if let Some(bob) = app.bob.as_mut() {
            bob.mode = ObserverMode::FreeFall;
        }
    }

    /// Put both observers on their geodesics, the way the Motion radio on their cards does.
    ///
    /// It has to be done to the observers rather than to the cards: Motion is the one thing a
    /// re-drop takes from the observer it replaces instead of reading off the card, so that a
    /// Reset never answers a question about how somebody moves that the user has not asked (see
    /// `ObserverCard::redropped`). A card's `mode` is only what a *fresh* observer starts on.
    fn set_free_fall(app: &mut SpacetimeApp) {
        for obs in [app.alice.as_mut(), app.bob.as_mut()].into_iter().flatten() {
            obs.mode = ObserverMode::FreeFall;
        }
    }

    /// Every word the app paints in one frame, in the order it is painted.
    ///
    /// `egui::__run_test_ui` throws the frame's shapes away, and what a control or a telemetry box
    /// *says* is exactly what a test of "this observer is not in the picture" has to read. This is
    /// that helper with the output kept: one real frame of the whole app - top bar, HUD, panel and
    /// both canvases - with the text of every painted galley collected. Fonts are left empty, as
    /// `__run_test_ui` leaves them, because a galley carries its string whether or not there are
    /// glyphs to draw it with.
    fn painted_text(app: &mut SpacetimeApp) -> String {
        fn collect(shape: &egui::Shape, out: &mut String) {
            match shape {
                egui::Shape::Text(text) => {
                    out.push_str(text.galley.text());
                    out.push('\n');
                }
                egui::Shape::Vec(shapes) => {
                    for shape in shapes {
                        collect(shape, out);
                    }
                }
                _ => {}
            }
        }

        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        let mut text = String::new();
        for clipped in output.shapes.iter() {
            collect(&clipped.shape, &mut text);
        }
        output.drop_without_applying_deltas();
        text
    }

    /// One observer's place in a layout: their name, the event they stand at (t, r, phi), their
    /// own clock, the release they are waiting for and whether it has happened, and then the pulse
    /// count and clock of the transmission that belongs to them.
    type LayoutRow = (String, f64, f64, f64, f64, f64, bool, usize, f64);

    /// The layout the app opens on, as one row per observer: what "startup, Reset and Drop
    /// Observers agree" is measured on.
    fn layout(app: &SpacetimeApp) -> Vec<LayoutRow> {
        [app.alice.as_ref(), app.bob.as_ref()]
            .into_iter()
            .flatten()
            .zip([&app.signal, &app.bob_signal])
            .map(|(obs, field)| {
                (
                    obs.name.clone(),
                    obs.t,
                    obs.r,
                    obs.phi,
                    obs.tau,
                    obs.release_t,
                    obs.is_active,
                    field.pulses.len(),
                    field.t,
                )
            })
            .collect()
    }

    #[test]
    fn test_canvas_layout_sizing() {
        let mut app = SpacetimeApp::default();
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
    }

    #[test]
    fn test_resetting_the_run_puts_the_time_pan_back_to_the_start() {
        // Dragging the (t, r) canvas pans it in time, and that pan is an offset from the simulation
        // clock rather than an absolute time. Reset and a preset change both put the clock back to
        // zero; before this, the pan stayed where it was, and the user was left
        // looking at a stretch of empty diagram with the restarted run somewhere off the edge of
        // it. Every control that zeroes the clock now raises `view_reset_requested`, and this walks
        // the whole path: the action, the request, and the frame that consumes it.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for _ in 0..40 {
            app.step_forward(0.1);
        }
        // A pan of two M up the diagram, as a drag would leave it, and a radial pan and zoom with
        // it so that the two can be told apart.
        app.spacetime_canvas.time_offset = 2.0;
        app.spacetime_canvas.r_offset = 1.25;
        app.spacetime_canvas.max_r = 0.05;
        // The volume view is panned in time by the same kind of offset from the same clock, and it
        // goes back with it whether or not it is the picture currently on screen.
        app.volume_canvas.time_offset = 2.0;

        // The Reset button's own action, through the panel rather than around it.
        drop_observers(&mut app);
        assert_eq!(app.current_time, 0.0, "the reset puts the clock back");
        assert!(app.controls.view_reset_requested, "and asks for the view to go back with it");
        assert_eq!(
            app.spacetime_canvas.time_offset, 2.0,
            "the panel cannot reach the canvas itself: the request is still standing"
        );

        // One frame of the app, which is where the request is taken.
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        assert_eq!(app.spacetime_canvas.time_offset, 0.0, "the pan in time is back at the start");
        assert_eq!(app.volume_canvas.time_offset, 0.0, "and so is the volume view's");
        assert!(!app.controls.view_reset_requested, "and the request has been consumed");
        // The radial pan and the zoom are a separate choice and are deliberately left standing.
        assert_eq!(app.spacetime_canvas.r_offset, 1.25);
        assert_eq!(app.spacetime_canvas.max_r, 0.05);

        // A second frame changes nothing: the request is a one-shot, not a mode that would fight
        // the user's next drag.
        app.spacetime_canvas.time_offset = 1.0;
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        assert_eq!(app.spacetime_canvas.time_offset, 1.0, "a consumed request does not fire twice");
    }

    #[test]
    fn test_choosing_the_2d_plus_1_chart_swaps_the_left_canvas() {
        // The two charts of the foliation are alternatives rather than neighbours: the column is
        // only wide enough for one of them to be read, so choosing the 2D+1 chart puts the volume
        // where the (t, r) diagram was rather than beside it. The right-hand column is not part of
        // the choice and is drawn either way, which is what the third assertion of each pair holds
        // on to - otherwise "the diagram is gone" would be satisfied by a frame that drew nothing.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        assert_eq!(
            app.controls.frame_of_ref,
            ReferenceFrame::DistantObserver,
            "the app opens on the flat diagram"
        );

        let flat = painted_text(&mut app);
        assert!(
            flat.contains("Coordinate Time t"),
            "the (t, r) diagram's own time axis should be drawn in the 1D+1 chart"
        );
        assert!(!flat.contains("2D+1 Volume"), "and the volume view should not be");
        assert!(flat.contains("Spatial x"), "the equatorial view is drawn either way");

        app.controls.frame_of_ref = ReferenceFrame::GlobalVolume;
        let volume = painted_text(&mut app);
        assert!(
            volume.contains("2D+1 Volume"),
            "choosing the 2D+1 chart should put the volume view in the left column"
        );
        assert!(
            !volume.contains("Coordinate Time t"),
            "and take the (t, r) diagram out of it: the two share the column"
        );
        assert!(volume.contains("Spatial x"), "the equatorial view is untouched by the swap");
    }

    #[test]
    fn test_both_charts_say_they_are_charts_and_a_rest_frame_does_not() {
        // The one line that decides what a viewer may read off the picture: a chart places every
        // event where the coordinates put it and claims nothing about distance or simultaneity
        // for anybody, a rest frame claims exactly that about one observer. Both charts of the
        // foliation carry the line; neither rest frame does, because there it would be false.
        use crate::gui::spacetime_canvas::CHART_BANNER;
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for (view, chart) in [
            (ReferenceFrame::DistantObserver, true),
            (ReferenceFrame::GlobalVolume, true),
            (ReferenceFrame::Bob, false),
            (ReferenceFrame::Alice, false),
        ] {
            app.controls.frame_of_ref = view;
            let text = painted_text(&mut app);
            assert_eq!(
                text.contains(CHART_BANNER),
                chart,
                "{view:?}: the banner should be drawn exactly on the two charts"
            );
        }
    }

    #[test]
    fn test_auto_zoom_is_offered_only_in_a_rest_frame() {
        // A rest frame has a zoom that can be derived from the geometry rather than chosen - the
        // surface the observer is about to reach, kept on the canvas - so the box that asks for it
        // belongs to the rest frame. Neither chart of the global foliation has one: there is nobody
        // at the origin of either and nothing ahead of anybody in particular to frame.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;

        for frame in [ReferenceFrame::DistantObserver, ReferenceFrame::GlobalVolume] {
            app.controls.frame_of_ref = frame;
            let text = painted_text(&mut app);
            assert!(
                !text.contains("Auto-zoom"),
                "the global foliation has nothing to frame, in {frame:?}"
            );
        }

        for frame in [ReferenceFrame::Bob, ReferenceFrame::Alice] {
            app.controls.frame_of_ref = frame;
            let text = painted_text(&mut app);
            assert!(
                text.contains("Auto-zoom"),
                "a rest frame is offered the automatic framing, in {frame:?}"
            );
        }

        // And the box is the (t, r) diagram's own flag, which is the only framing there is.
        app.spacetime_canvas.keep_surface_framed = true;
        painted_text(&mut app);
        assert!(app.spacetime_canvas.keep_surface_framed, "the flat diagram's flag is the one shown");
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
        //
        // Measured on Alice, because she is the one moving. In the layout the app opens on Bob
        // hovers at r = 4.5M until t = 8, so a step of 0.1 M moves him in t and in nothing else,
        // which would prove nothing about the integrator; his hover is checked separately below.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
        let al = alice_of(&app);
        let initial = (al.t, al.r, al.phi, al.tau);
        let step = app.arrow_step();

        app.step_forward(step);
        assert!(alice_of(&app).r < initial.1, "Stepping forward should advance inward infall");
        assert!((alice_of(&app).t - app.current_time).abs() < 1e-12, "and stay on the clock");
        assert!(
            !bob_of(&app).is_active && (bob_of(&app).r - 4.5).abs() < 1e-12,
            "Bob is still hovering at r = {}",
            bob_of(&app).r
        );

        app.step_backward(step);
        assert!((app.current_time - initial.0).abs() < 1e-12, "the clock comes back: {}", app.current_time);
        let al = alice_of(&app);
        let back = (al.t, al.r, al.phi, al.tau);
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
    fn test_a_backstep_stops_where_the_recorded_history_stops() {
        // The trail has a cap, so a run long enough to reach it evicts its own oldest events and
        // there is a time below which no worldline can be put back at all. That floor used to be
        // silent, and worse, it used to be each observer's own: `rewind_to` landed an observer on
        // the oldest event it still held while the clock carried on down past it, so the observer
        // came off the clock and stayed exactly where it was, redrawn in the same place for every
        // further backstep - one worldline frozen while the other was still moving. It is now the
        // clock's floor, asked for once for the pair, so the clock stops where the history stops
        // and both worldlines stop on it.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        // Dropped high, so neither reaches the ring inside a run long enough to overflow the
        // trail: from 60 M the fall lasts 219 M and this run is 170.
        app.controls.alice.drop_r = 60.0;
        app.controls.bob.drop_r = 60.0;
        drop_observers(&mut app);
        // Played on until the trails have evicted their own start, at the step a played frame
        // takes. Driven by the floor itself rather than by a step count, so that the test says
        // what it means and goes on meaning it whatever `TRAIL_MAX_POINTS` is set to.
        let floor_now = |app: &mut SpacetimeApp| {
            ObserverPair { bob: app.bob.as_mut(), alice: app.alice.as_mut() }
                .rewind_floor(&app.metric)
        };
        let mut floor = 0.0;
        let mut steps = 0;
        while floor <= 0.0 {
            app.step_forward(0.0167);
            steps += 1;
            assert!(steps < 200_000, "the trail never evicted its own start");
            floor = floor_now(&mut app);
        }
        println!(
            "{steps} recorded events later, the history reaches back only to t = {floor:.4} M of \
             a run that has reached {:.4}",
            app.current_time
        );
        assert!(floor < app.current_time, "the floor is behind the clock: {floor}");

        // All the way down: at every step both worldlines are on the clock, which is the invariant
        // the freeze broke.
        let mut backsteps = 0;
        while app.current_time > floor + 1e-9 {
            app.step_backward(5.0);
            backsteps += 1;
            assert!(backsteps < 100_000, "the clock never reached the floor");
            for obs in [alice_of(&app), bob_of(&app)] {
                assert!(
                    (obs.t - app.current_time).abs() < 1e-9,
                    "{} came off the clock: {} against {}",
                    obs.name,
                    obs.t,
                    app.current_time
                );
            }
        }
        // The floor does not move as the clock comes down to it: a rewind truncates a trail from
        // the back, so the oldest event held is the oldest event held throughout.
        assert!(
            (floor_now(&mut app) - floor).abs() < 1e-12,
            "the floor moved under the rewind: {} against {floor}",
            floor_now(&mut app)
        );
        assert!(
            (app.current_time - floor).abs() < 1e-9,
            "the clock stops on the floor rather than below it: {} against {floor}",
            app.current_time
        );

        // And at the floor a backstep does nothing at all, rather than moving the clock and
        // leaving the observers behind. Nothing to undo is nothing to undo.
        let held = (app.current_time, observer_state(alice_of(&app)), observer_state(bob_of(&app)));
        app.step_backward(5.0);
        assert_eq!(
            (app.current_time, observer_state(alice_of(&app)), observer_state(bob_of(&app))),
            held,
            "a backstep at the floor moved something"
        );
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
        let alice_before = observer_state(alice_of(&app));
        let bob_before = observer_state(bob_of(&app));
        let alice_ended = alice_of(&app).has_ended();
        let bob_ended = bob_of(&app).has_ended();
        app.step_backward(0.1);
        println!(
            "back 0.1 from t = 6.0: clock {:.4}; Alice {:?} (ended {alice_ended}) -> {:?}; \
             Bob {:?} (ended {bob_ended}) -> {:?}",
            app.current_time,
            alice_before,
            observer_state(alice_of(&app)),
            bob_before,
            observer_state(bob_of(&app))
        );
        assert!((app.current_time - 5.9).abs() < 1e-9);
        // An observer that has ended before the target does not move at all; one that has not is
        // on the clock, to the last bit.
        for (obs, before, ended) in [
            (alice_of(&app), alice_before, alice_ended),
            (bob_of(&app), bob_before, bob_ended),
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
        trailing_raindrops(&mut app);
        for _ in 0..100 {
            app.step_forward(0.1);
        }
        assert!((app.current_time - 10.0).abs() < 1e-9);
        let alice = alice_of(&app);
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
            alice_of(&app).t,
            alice_of(&app).r
        );
        assert_eq!(
            observer_state(alice_of(&app)),
            ended_at,
            "a rewind to t = {} is still past her end at t = {}: she stays on the ring",
            app.current_time,
            ended_at.0
        );
        // Now back past the end of her worldline: she comes off the ring, and onto the clock.
        while app.current_time > ended_at.0 - 0.25 {
            app.step_backward(0.1);
        }
        let alice = alice_of(&app);
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
        let wound = observer_state(alice_of(&app));
        assert!((wound.0 - 2.0).abs() < 1e-12, "Alice lands on the clock: {}", wound.0);
        assert!(
            (bob_of(&app).t - 2.0).abs() < 1e-12,
            "and so does Bob, hovering: {}",
            bob_of(&app).t
        );

        // And where it lands is the worldline, not merely the clock: compare with a fresh run to
        // the same time in different steps. The two differ only by the integrator, which is
        // stepping the same geodesic over differently cut intervals.
        let mut fresh = SpacetimeApp::default();
        fresh.controls.is_playing = false;
        for _ in 0..40 {
            fresh.step_forward(0.05);
        }
        let straight = observer_state(alice_of(&fresh));
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
                let (b, al) = (bob_of(&app), alice_of(&app));
                if frame_idx % 50 == 0 || (b.r - al.r).abs() < 0.05 || b.r < 1.0 {
                    println!("Frame {}: Bob r={:.6} t={:.4}, Alice r={:.6} t={:.4}, diff_r={:.6}", frame_idx, b.r, b.t, al.r, al.t, (b.r - al.r).abs());
                }
                if b.r <= 0.03 && al.r <= 0.03 {
                    println!("Both hit singularity at frame {}", frame_idx);
                    break;
                }
            }
        });
    }

    #[test]
    fn test_alice_signal_is_received_through_the_app_loop() {
        // The layout a Reset builds, at a shorter delay: Alice released from r = 4.5M
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
        app.controls.bob.delta_t_delay = 4.0;
        drop_observers(&mut app);
        // Test frames arrive as fast as the harness can render them, so the frame clock sits at its
        // floor of 1/240 s; the playback rate is what buys enough simulation time to reach Bob's
        // crossing without running thousands of frames.
        app.controls.play_speed = 4.0;
        // The app opens paused; this test is about what the play loop does, so it presses Play.
        app.controls.is_playing = true;

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
        app.controls.is_playing = true;

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
        for obs in [app.bob.as_ref(), app.alice.as_ref()].into_iter().flatten() {
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
        // The presets re-drop the observers when the geometry changes, because a 4-velocity
        // integrated in one metric is not a unit timelike vector in another; do the same here.
        let mut app = SpacetimeApp { metric: schwarzschild, ..SpacetimeApp::default() };
        drop_observers(&mut app);
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
            // And again with neither of them transmitting, which is now what empties the two
            // fields: there is no separate "draw the pulses" flag to turn off, because a field
            // nobody is filling has nothing in it to draw.
            app.controls.alice.transmit = false;
            app.controls.bob.transmit = false;
            app.ui(ui, &mut frame);
        });
        assert!(app.signal.pulses.is_empty() && app.bob_signal.pulses.is_empty());
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
    fn test_each_observer_gets_their_own_release_and_angular_momentum() {
        // A release and an L per card, and a drop has to put each observer on their own geodesic
        // rather than on a shared one. E is not on the card any more: it is whatever the release
        // implies at the radius it happens at, so what is checked here is that the worldline each
        // observer ends up carrying is the one their own card asked for.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
        // Alice as she opens: from rest at infinity, so E = 1 exactly whatever her radius.
        app.controls.alice.release = Release::FromInfinity;
        // Bob let go at rest, with angular momentum, from further out.
        app.controls.bob.release = Release::AtRest;
        app.controls.bob.l_ang = 1.0;
        app.controls.bob.drop_r = 6.0;
        app.controls.bob.delta_t_delay = 8.0;
        drop_observers(&mut app);

        let al = alice_of(&app).geodesic.expect("free-fall observers carry a geodesic state");
        let b = bob_of(&app).geodesic.expect("free-fall observers carry a geodesic state");
        assert!((al.energy - 1.0).abs() < 1e-12, "a raindrop has E = 1: {}", al.energy);
        assert!(al.l_ang.abs() < 1e-12, "and her L is still her own: {}", al.l_ang);
        // At rest means at rest: E is the effective potential at his own drop radius, and his
        // worldline starts on a turning point of it.
        let floor = GeodesicState::energy_floor(&app.metric, 6.0, 1.0);
        assert!((b.energy - floor).abs() < 1e-12, "Bob's E = {} against V(6, 1) = {floor}", b.energy);
        assert!((b.l_ang - 1.0).abs() < 1e-12, "and his L = {}", b.l_ang);
        assert!(b.u[1].abs() < 1e-6, "released at rest: dr/dtau = {}", b.u[1]);
        println!(
            "Alice from infinity: E = {:.4}; Bob at rest from 6M with L = 1: E = {:.4}",
            al.energy, b.energy
        );
        for obs in [alice_of(&app), bob_of(&app)] {
            assert!(
                (app.metric.norm(obs.r, &obs.four_velocity(&app.metric)) + 1.0).abs() < 1e-9,
                "{}'s 4-velocity must stay a unit timelike vector",
                obs.name
            );
        }

        // The two worldlines are genuinely different: run them and they separate.
        for _ in 0..50 {
            app.step_forward(0.05);
        }
        println!(
            "after 2.5 M: Alice (raindrop) at r = {:.4}, Bob (at rest from 6M, hovering until \
             t = 8) at r = {:.4}",
            alice_of(&app).r,
            bob_of(&app).r
        );
        assert!(
            alice_of(&app).r < 4.5,
            "the raindrop is already moving when the run starts: r = {}",
            alice_of(&app).r
        );
        assert!(
            (bob_of(&app).r - 6.0).abs() < 1e-9,
            "while Bob is still holding his radius: r = {}",
            bob_of(&app).r
        );

        // The one radius where "at rest" has no meaning: between the horizons nothing holds r, so
        // the release falls back to the raindrop and the panel says so.
        app.controls.bob.drop_r = 1.0;
        app.controls.bob.l_ang = 0.0;
        drop_observers(&mut app);
        let geo = bob_of(&app).geodesic.expect("free-fall observers carry a geodesic state");
        assert!((geo.energy - 1.0).abs() < 1e-12, "inside r+ a rest release is a raindrop: {}", geo.energy);
        assert!(
            painted_text(&mut app).contains("Nothing can be at rest between the horizons"),
            "and the card says which release it actually made"
        );

        // Both let go at once, both in free fall; the difference is the worldline. Bob is the
        // raindrop from 4.5 M, Alice circles the prograde ISCO of the hole the app opens on.
        let d = AppControls::default();
        for card in [d.alice, d.bob] {
            assert!(card.enabled && card.transmit);
            assert_eq!(card.delta_t_delay, 0.0);
            assert_eq!(card.mode, ObserverMode::FreeFall);
        }
        assert_eq!((d.bob.l_ang, d.bob.release, d.bob.drop_r), (0.0, Release::FromInfinity, 4.5));
        assert_eq!(d.alice.release, Release::CircularPrograde);
        let isco = KerrSchild::new(1.0, OPENING_SPIN).isco(true);
        assert!((d.alice.drop_r - isco).abs() < 1e-12, "Alice drops onto the ISCO at {isco} M");
        assert_eq!(d.release_gap(), 0.0, "no trailing delay, so no stack for anyone to cut");
    }

    /// The drag the (t, r) canvas takes on a marker, without the pointer: `set_drag_position`
    /// while it is held and `release_from_drag` when it is let go, which is exactly the pair
    /// `SpacetimeCanvas::drag_markers` calls.
    fn drag_bob_to(app: &mut SpacetimeApp, r: f64) {
        let mode = bob_of(app).mode;
        {
            let SpacetimeApp { bob, current_time, .. } = app;
            bob.as_mut().expect("Bob is in this run").set_drag_position(*current_time, r);
        }
        // A frame with the pointer still down. It is a real part of the gesture, not a detail of
        // the harness: a drag writes the observer, and it is this frame that folds the new position
        // into his card, where `ObserverCard::describes` will then keep it.
        painted_text(app);
        {
            let SpacetimeApp { metric, bob, .. } = app;
            bob.as_mut().expect("Bob is in this run").release_from_drag(metric, mode);
        }
        painted_text(app);
    }

    #[test]
    fn test_an_observer_moved_at_the_start_of_a_run_is_dropped_there_by_the_next_reset() {
        // Where somebody is dropped from is a property of the run, and the only way to say it is
        // to put them there: drag the marker while the clock reads zero and that is where Reset
        // builds them from, for as long as the app is open. Before this, the drag was thrown away
        // by the next Reset and the run always restarted at 4.5M, which made the drag useless for
        // the thing it is most wanted for - starting an observer from somewhere else.
        let mut app = SpacetimeApp::default();
        assert_eq!(app.current_time, 0.0, "the app opens at the start of the run");
        drag_bob_to(&mut app, 7.25);
        // One real frame, because it is the frame that takes the position: see
        // `AppControls::remember_drop_positions`.
        painted_text(&mut app);
        assert!((app.controls.bob.drop_r - 7.25).abs() < 1e-9, "the card took the drag");

        drop_observers(&mut app);
        let dropped = bob_of(&app).r;
        println!("dragged to 7.25M at t = 0, Reset drops him at {dropped:.4}M");
        assert!((dropped - 7.25).abs() < 1e-9, "Reset puts him back where he was put: {dropped}");
        // Alice, who was not touched, is still dropped where her own card says.
        assert!(
            (alice_of(&app).r - app.controls.alice.drop_r).abs() < 1e-9,
            "and nobody else moves: {}",
            alice_of(&app).r
        );

        // A drag taken once the run is moving is a change to a worldline in progress, not to where
        // the run starts, so the next Reset goes back to the remembered drop instead of to it.
        app.step_forward(0.5);
        drag_bob_to(&mut app, 2.0);
        painted_text(&mut app);
        assert!(
            (app.controls.bob.drop_r - 7.25).abs() < 1e-9,
            "a drag off the start is not a drop position: {}",
            app.controls.bob.drop_r
        );
        drop_observers(&mut app);
        assert!(
            (bob_of(&app).r - 7.25).abs() < 1e-9,
            "so Reset still builds him at 7.25M: {}",
            bob_of(&app).r
        );
    }

    #[test]
    fn test_letting_go_of_a_marker_releases_them_again_where_they_were_dropped() {
        // A drag is a teleport followed by an engine cut, and the cut means the same thing wherever
        // it happens: the observer's own release is re-read at the new radius. Dragged by somebody
        // released at rest, the marker comes to rest there; dragged by a raindrop, it is still a
        // raindrop. It used to carry the old E across instead, which is the energy of a release
        // that happened somewhere else - so a marker pulled outward would shoot back in, and one
        // pushed inward would hang.
        let mut app = SpacetimeApp::default();
        app.controls.bob.release = Release::AtRest;
        app.controls.bob.l_ang = 0.5;
        drop_observers(&mut app);
        // Part-way into the run, so this is a re-release rather than the original drop.
        for _ in 0..20 {
            app.step_forward(0.05);
        }
        drag_bob_to(&mut app, 9.0);
        let geo = bob_of(&app).geodesic.expect("a dragged observer keeps their geodesic");
        let floor = GeodesicState::energy_floor(&app.metric, 9.0, 0.5);
        println!("let go at 9M: E = {:.4} against V(9, 0.5) = {floor:.4}", geo.energy);
        assert!((geo.energy - floor).abs() < 1e-12, "released at rest there: E = {}", geo.energy);
        assert!(geo.u[1].abs() < 1e-6, "which is what at rest means: dr/dtau = {}", geo.u[1]);
        assert!((geo.l_ang - 0.5).abs() < 1e-12, "and L is carried across: {}", geo.l_ang);

        // The raindrop keeps its own meaning under the same gesture.
        app.controls.bob.release = Release::FromInfinity;
        drop_observers(&mut app);
        for _ in 0..20 {
            app.step_forward(0.05);
        }
        drag_bob_to(&mut app, 9.0);
        let geo = bob_of(&app).geodesic.expect("a dragged observer keeps their geodesic");
        assert!((geo.energy - 1.0).abs() < 1e-12, "still a raindrop: E = {}", geo.energy);
        assert!(geo.u[1] < 0.0, "and still falling: dr/dtau = {}", geo.u[1]);
    }

    #[test]
    fn test_a_card_restated_at_the_start_of_a_run_moves_the_observer_at_once() {
        // While the clock reads zero the card and the observer are the same thing, so a drop
        // radius, a release, an L or a delay stated on the panel takes effect there and then - the
        // marker moves in both views as the slider moves, which is the other half of a marker drag
        // moving the slider. Nothing is under way for it to contradict.
        //
        // It is enforced as a state of affairs rather than on the slider's own `changed()` edge, so
        // that writing the field directly - a test, a keybinding, a preset - gets the same
        // simulation as a user dragging the widget.
        let mut app = SpacetimeApp::default();
        assert_eq!(app.current_time, 0.0);
        let alice_before = alice_of(&app).r;

        app.controls.bob.drop_r = 9.0;
        painted_text(&mut app);
        assert!((bob_of(&app).r - 9.0).abs() < 1e-9, "he is there now: {}", bob_of(&app).r);
        assert!(
            (alice_of(&app).r - alice_before).abs() < 1e-12,
            "and nobody else moved: {}",
            alice_of(&app).r
        );

        // The release and L are restated the same way, and E follows them.
        app.controls.bob.release = Release::AtRest;
        app.controls.bob.l_ang = 1.5;
        painted_text(&mut app);
        let geo = bob_of(&app).geodesic.expect("free-fall observers carry a geodesic state");
        let floor = GeodesicState::energy_floor(&app.metric, 9.0, 1.5);
        println!("restated to at-rest from 9M with L = 1.5: E = {:.4} against {floor:.4}", geo.energy);
        assert!((geo.energy - floor).abs() < 1e-12, "E follows the release: {}", geo.energy);
        assert!((geo.l_ang - 1.5).abs() < 1e-12, "L is his own: {}", geo.l_ang);
        assert!(geo.u[1].abs() < 1e-6, "and he is at rest: dr/dtau = {}", geo.u[1]);

        // So is the delay, which is a statement about the same worldline.
        app.controls.bob.delta_t_delay = 5.0;
        painted_text(&mut app);
        assert_eq!(bob_of(&app).release_t, 5.0, "he waits until t = 5 now");
        assert!(!bob_of(&app).is_active, "so he is still holding his radius");

        // Once the clock is running they are standing requests again: there is a worldline with a
        // history now, light in flight from it and arrivals recorded against it, and a slider does
        // not reach back and change where any of that came from.
        app.step_forward(0.5);
        app.controls.bob.drop_r = 3.0;
        painted_text(&mut app);
        assert!(
            (bob_of(&app).r - 9.0).abs() < 1e-9,
            "the run is under way, so he stays where he is: {}",
            bob_of(&app).r
        );
        // And the next Reset takes it, as the card has said all along.
        drop_observers(&mut app);
        assert!((bob_of(&app).r - 3.0).abs() < 1e-9, "Reset builds him there: {}", bob_of(&app).r);
    }

    #[test]
    fn test_a_marker_drag_says_where_not_when() {
        // The (t, r) diagram has two axes and only one of them is something an observer can be
        // moved along: every worldline in the run stands at the simulation clock, because that is
        // what a clock is. A drag therefore places them in r and leaves their t where it was.
        //
        // It used to take the pointer's height as well, and that showed. Dragged upward an observer
        // stood in the future of the light drawn around them; dragged below the clock line they
        // took `Observer::release_from_drag`'s `release_t = min(release_t, t)` with them, which at
        // the start of a run no longer matched their card, so the card put them back on the clock
        // line - a drag that wandered downward snapped back while one that wandered up did not.
        let mut app = SpacetimeApp::default();
        drag_bob_to(&mut app, 9.0);
        assert!((bob_of(&app).r - 9.0).abs() < 1e-9, "the radius is his: {}", bob_of(&app).r);
        assert_eq!(bob_of(&app).t, app.current_time, "and he still stands at the clock");
        assert_eq!(bob_of(&app).t, alice_of(&app).t, "on the same line everybody else is on");
        assert_eq!(bob_of(&app).release_t, 0.0, "with the release his card asked for");

        // And with the run under way, where nothing restates him afterwards: the drag itself has
        // to leave the clock alone, rather than being tidied up by the card.
        for _ in 0..20 {
            app.step_forward(0.05);
        }
        let clock = app.current_time;
        drag_bob_to(&mut app, 3.0);
        assert!((bob_of(&app).r - 3.0).abs() < 1e-9, "moved in r: {}", bob_of(&app).r);
        assert!(
            (bob_of(&app).t - clock).abs() < 1e-9,
            "and not in t: {} against a clock of {clock}",
            bob_of(&app).t
        );
        assert!(
            (app.controls.bob.drop_r - 9.0).abs() < 1e-9,
            "a drag once the run is moving is not a drop position either: {}",
            app.controls.bob.drop_r
        );
    }

    #[test]
    fn test_control_defaults() {
        let d = AppControls::default();
        // Physical units out of the box: kilometres, seconds and radians per second. The panel's
        // units checkbox is the opt-in to M, not an escape from it, so it opens unticked - which
        // is this flag being true. Nothing throttles the step near r₋ any more either: the play
        // speed and the step size are the only things that set sim_dt.
        assert!(d.use_physical_units, "the app opens in km and seconds, not in M");
        // Paused: the run the user is handed is a standing start, so that nothing has happened
        // before they have had a chance to look at it or to move anybody.
        assert!(!d.is_playing, "the app opens paused");
        assert_eq!(d.play_speed, 1.0);
        assert_eq!(d.step_size, 0.1);
        // Both observers are in the run and both are transmitting out of the box.
        assert!(d.alice.enabled && d.alice.transmit);
        assert!(d.bob.enabled && d.bob.transmit);
        // Nothing has been played yet, so no frame has a watch rate to report.
        assert_eq!(d.achieved_watch_rate, None);
    }

    #[test]
    fn test_km_telemetry_layout_renders_in_both_views() {
        // Both observers present, in kilometres, with the new one-metric-per-line info boxes and
        // Alice's light cone in the (t, r) diagram: the boxes register widgets on the canvas, so
        // this walks the interaction path as well as the paint path.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.use_physical_units = true;
        app.controls.frame_of_ref = ReferenceFrame::DistantObserver;
        assert!(app.alice.is_some(), "both cards are ticked out of the box");

        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            // Two passes: the second one reads back the widgets the first one registered.
            for _ in 0..2 {
                app.ui(ui, &mut frame);
            }
            // The held state a drag puts an observer in is the non-geodesic case, which prints
            // a_thrust and no E / L.
            app.bob.as_mut().expect("Bob is enabled").mode = ObserverMode::ManualDrag;
            app.ui(ui, &mut frame);
        });

        assert!(app.controls.use_physical_units);
    }

    #[test]
    fn test_fixed_distance_stepping() {
        // The default hole is Sagittarius A* (4.15e6 M_solar), where 1000 km is a fine step
        // (1.63e-4 M); no metric override is needed any more.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
        app.controls.step_mode = StepMode::Distance;
        app.controls.step_distance_km = 1000.0;

        // The step is quoted for whoever is moving in r, and in the trailing-raindrop layout that
        // is Alice: Bob hovers at r = 4.5M until t = 8, so he has no coordinate speed to divide by
        // and the chain falls through to her (see `test_a_distance_step_needs_somebody_who_is_
        // moving_in_r`). So it is her the distance is honoured for.
        let sim_dt = app.arrow_step();

        let initial_r = alice_of(&app).r;
        app.alice.as_mut().expect("Alice is enabled").step(
            &app.metric,
            app.current_time + sim_dt,
            sim_dt,
        );
        let actual_dr = (initial_r - alice_of(&app).r).abs();
        let actual_dr_km = app.metric.r_to_km(actual_dr);

        // Verify that stepping by distance moves Bob by approximately 1000 km (within numerical integration tolerance)
        assert!(
            (actual_dr_km - 1000.0).abs() < 50.0,
            "Expected ~1000 km movement, got {:.2} km",
            actual_dr_km
        );
    }

    #[test]
    fn test_a_distance_step_needs_somebody_who_is_moving_in_r() {
        // Distance mode asks "how long does Bob need to cover Δr?", and for a Bob who is not
        // moving in r that question has no answer. It used to get one anyway: `four_velocity`
        // handed a hovering Bob the free-fall value at his radius, so the step was quoted off a
        // speed he did not have. Now his dr/dt is exactly zero - it is the static worldline he is
        // on - and the fallbacks are stated rather than fallen into.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.step_mode = StepMode::Distance;
        app.controls.step_distance_km = 1000.0;

        // 1. Both falling in r: Bob's own speed, as before. That is the layout the app opens on
        //    for him - no release delay - but not for Alice, who circles the ISCO until she is
        //    put on an infall here.
        set_free_fall(&mut app);
        app.step_forward(0.05);
        let falling = app.arrow_step();
        assert!(bob_of(&app).velocity_c(&app.metric) < 0.0);
        assert!(falling > 1e-8 && falling < 500.0, "a real step: {falling}");

        // 2. Bob hovering, Alice falling: Alice's speed, and a step of the same order rather than
        //    the 500 M the old 0.01c floor clamped a stationary Bob to.
        app.controls.bob.delta_t_delay = 8.0;
        drop_observers(&mut app);
        app.step_forward(0.5);
        assert!(
            !bob_of(&app).is_active && bob_of(&app).velocity_c(&app.metric) == 0.0,
            "he is hovering"
        );
        let alice_paced = app.arrow_step();
        let alice_speed = alice_of(&app).velocity_c(&app.metric).abs();
        let expected = app.metric.km_to_r(1000.0) / alice_speed.max(0.01);
        assert!(
            (alice_paced - expected).abs() < 1e-12,
            "a hovering Bob is paced by Alice: {alice_paced} vs {expected}"
        );
        assert!(alice_paced < 1.0, "and not by the 500 M clamp: {alice_paced}");

        // 3. Nobody moving in r: the fixed fallback, which claims nothing about a distance. An
        //    Alice who is not in the simulation at all leaves the chain the same way a hovering
        //    one does, and so does a Bob.
        app.alice = None;
        assert!((app.arrow_step() - 0.1).abs() < 1e-12, "step = {}", app.arrow_step());
        app.bob = None;
        assert!((app.arrow_step() - 0.1).abs() < 1e-12, "step = {}", app.arrow_step());

        // 4. A Static Bob outside the static limit holds his radius by choice, and is treated the
        //    same way: there is no time in which he covers Δr either.
        let mut bob = Observer::new(&app.metric, "Bob", 0.0, 5.0, 0.0);
        bob.mode = ObserverMode::Static;
        assert!(bob.mode_admissible(&app.metric) && bob.velocity_c(&app.metric) == 0.0);
        app.bob = Some(bob);
        assert!((app.arrow_step() - 0.1).abs() < 1e-12);

        // 5. ...but a Static selection at a radius where it is impossible is falling, so it paces
        //    the step like any other faller.
        let mut bob = Observer::new(&app.metric, "Bob", 0.0, 1.9, 0.0);
        bob.mode = ObserverMode::Static;
        assert!(!bob.mode_admissible(&app.metric));
        assert!(bob.velocity_c(&app.metric) < 0.0, "he is falling, and the step follows him");
        app.bob = Some(bob);
        assert!(app.arrow_step() < 0.1);
    }

    #[test]
    fn test_a_circular_release_puts_the_observer_on_a_circle_and_keeps_them_there() {
        // The card's own path, end to end: choose the prograde circular release at 6 M, drop,
        // play a few thousand M of coordinate time, and the observer is still at 6 M and has gone
        // round several times, with no thrust anywhere in it. Then the retrograde one, which
        // goes round the other way.
        let metric = KerrSchild::new(1.0, 0.9);
        for (release, prograde) in [
            (Release::CircularPrograde, true),
            (Release::CircularRetrograde, false),
        ] {
            let mut app = SpacetimeApp { metric: KerrSchild::new(1.0, 0.9), ..SpacetimeApp::default() };
            app.controls.is_playing = false;
            app.controls.bob.enabled = true;
            app.controls.bob.drop_r = 6.0;
            app.controls.bob.delta_t_delay = 0.0;
            app.controls.bob.release = release;
            app.controls.bob.l_ang = 3.0; // ignored: the orbit's own L is used
            drop_observers(&mut app);
            let expected_l = metric.circular_orbit(6.0, prograde).unwrap().1;
            let geo = bob_of(&app).geodesic.expect("a free-fall worldline");
            assert!((geo.l_ang - expected_l).abs() < 1e-12, "L is the orbit's: {} vs {expected_l}", geo.l_ang);
            let period = std::f64::consts::TAU / metric.orbital_angular_velocity(6.0, prograde).unwrap().abs();
            // One orbit, stepping the worldline itself: the card's path is what is under test,
            // and the transmissions the app would carry along cost a hundred times as much.
            let phi0 = bob_of(&app).phi;
            let mut worst = 0.0f64;
            let (mut t, dt) = (app.current_time, 0.1);
            while t < app.current_time + period {
                t += dt;
                app.bob.as_mut().unwrap().step(&metric, t, dt);
                worst = worst.max((bob_of(&app).r - 6.0).abs());
            }
            let turned = bob_of(&app).phi - phi0;
            println!(
                "{release:?}: one period of {period:.2} M, r wandered {worst:.2e} M, phi moved {turned:.3}"
            );
            assert!(worst < 2e-3, "{release:?} stays at 6 M: wandered {worst}");
            assert!(bob_of(&app).is_active, "and is in free fall");
        }
    }

    #[test]
    fn test_the_watch_step_is_u_t_times_the_proper_step_until_the_cap() {
        // Watch mode asks for a step of the focus observer's own proper time and converts it with
        // the one exact factor there is: dτ/dt = 1/u^t along their worldline.
        let metric = KerrSchild::new(1.0, 0.90);
        let controls = AppControls {
            step_mode: StepMode::Watch,
            frame_of_ref: ReferenceFrame::Bob,
            ..AppControls::default()
        };

        // A static Bob well outside the hole. Equatorial Kerr has g_tt = -(1 - 2M/r) exactly as
        // Schwarzschild does, so his u^t = 1/sqrt(1 - 2M/r) in closed form, and at r = 20 M that
        // is 1.054: a tenth of his watch is a tenth of a distant M and a bit.
        let mut bob = Observer::new(&metric, "Bob", 0.0, 20.0, 0.0);
        bob.mode = ObserverMode::Static;
        assert!(bob.mode_admissible(&metric), "static is possible at r = 20 M");
        let u_t = 1.0 / (1.0 - 2.0 / 20.0f64).sqrt();
        let step = controls.watch_step(&metric, Some(&bob), None, 0.1);
        assert!((step.u_t - u_t).abs() < 1e-9, "u^t = {} rather than {u_t}", step.u_t);
        assert!((step.dt - u_t * 0.1).abs() < 1e-9, "Δt = {} rather than {}", step.dt, u_t * 0.1);
        assert!(!step.capped, "0.105 M is well inside the {WATCH_DT_CAP} M budget");
        // And the step every path in the app takes is that number.
        assert_eq!(controls.step_for(&metric, Some(&bob), None, 0.1), step.dt);

        // The frozen worldline is where the honest answer has to be refused. Bob asymptoting to
        // the far branch of r₋ has u^t of order 1e10, so one tick of his watch is more of the
        // outside future than any integrator here can step through, and the cap bites instead.
        let frozen = Observer::frozen_bob(&metric);
        let stalled = controls.watch_step(&metric, Some(&frozen), None, 0.1);
        assert!(stalled.u_t > 1e6, "the stall is a huge dilation: u^t = {}", stalled.u_t);
        assert_eq!(stalled.dt, WATCH_DT_CAP, "the cap, not the 1e9 M that was asked for");
        assert!(stalled.capped, "and it says so");
        // Which is what the readout quotes: a rate of 2 M per 1e9 M, not 1 s/s.
        assert!(stalled.dt / (stalled.u_t * 0.1) < 1e-6);
    }

    #[test]
    fn test_the_watch_of_the_distant_observer_is_coordinate_time() {
        // The distant observer has no worldline in the simulation and their watch is the chart's
        // own Killing time, so Watch mode and Time mode are the same thing in their frame. The
        // fallback is stated rather than fallen into: u^t = 1 and Δt = Δτ.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut controls = AppControls { step_mode: StepMode::Watch, ..AppControls::default() };
        assert_eq!(controls.frame_of_ref, ReferenceFrame::DistantObserver, "the default frame");

        // Bob is in the simulation and deep enough for a dilation of 1.7, and none of it is used.
        let mut bob = Observer::new(&metric, "Bob", 0.0, 3.0, 0.0);
        bob.mode = ObserverMode::Static;
        assert!(bob.four_velocity(&metric)[0] > 1.5, "he has a dilation to ignore");

        let step = controls.watch_step(&metric, Some(&bob), None, 0.1);
        assert_eq!((step.dt, step.u_t, step.capped), (0.1, 1.0, false));
        controls.step_mode = StepMode::Watch;
        let watching = controls.step_for(&metric, Some(&bob), None, 0.1);
        controls.step_mode = StepMode::Time;
        assert_eq!(watching, controls.step_for(&metric, Some(&bob), None, 0.1));

        // A focus observer whose card is unticked leaves the chain the same way: there is no watch
        // to keep, so the step is the one that was asked for.
        controls.step_mode = StepMode::Watch;
        controls.frame_of_ref = ReferenceFrame::Alice;
        let orphaned = controls.watch_step(&metric, Some(&bob), None, 0.1);
        assert_eq!((orphaned.dt, orphaned.u_t, orphaned.capped), (0.1, 1.0, false));
    }

    #[test]
    fn test_a_played_watch_frame_sets_the_achieved_rate() {
        // The readout under the step-mode chips, through the app's own play loop: a frame played
        // in Watch mode reports what fraction of the asked-for tick it carried, and any other
        // frame reports nothing at all.
        let mut app = SpacetimeApp::default();
        app.controls.frame_of_ref = ReferenceFrame::Bob;
        app.controls.step_mode = StepMode::Watch;
        app.controls.is_playing = true;
        set_free_fall(&mut app);

        let t0 = app.current_time;
        let tau0 = bob_of(&app).tau;
        let u_t = bob_of(&app).four_velocity(&app.metric)[0];
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        let rate = app.controls.achieved_watch_rate.expect("a played watch frame reports a rate");
        assert!(
            (rate - 1.0).abs() < 1e-12,
            "nothing is capped at r = 4.5 M, so the watch runs at the rate asked: {rate}"
        );
        // And the clock really was advanced on Bob's watch rather than on the distant one: the
        // coordinate time the frame took is u^t times the proper time he aged over it.
        let (d_t, d_tau) = (app.current_time - t0, bob_of(&app).tau - tau0);
        assert!(d_tau > 0.0 && d_t > d_tau, "Δt = {d_t}, Δτ = {d_tau}, u^t = {u_t}");
        assert!(
            (d_t / d_tau / u_t - 1.0).abs() < 0.02,
            "Δt/Δτ = {} rather than u^t = {u_t}",
            d_t / d_tau
        );

        // Paused, there is no rate to report, and neither is there in another step mode.
        app.controls.is_playing = false;
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        assert_eq!(app.controls.achieved_watch_rate, None, "a paused run keeps no watch");

        app.controls.is_playing = true;
        app.controls.step_mode = StepMode::Time;
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });
        assert_eq!(app.controls.achieved_watch_rate, None, "and neither does Time mode");
    }

    /// Is (x, y) inside the closed polygon? Ray casting along +x, counting edge crossings; a point
    /// with an odd number of them is inside. Only ever handed a wavefront loop, which is a closed
    /// polygon by construction: the emission angles divide the turn exactly, so the last ray joins
    /// back to the first.
    fn point_in_polygon(poly: &[(f64, f64)], x: f64, y: f64) -> bool {
        let mut inside = false;
        let mut j = poly.len() - 1;
        for i in 0..poly.len() {
            let (xi, yi) = poly[i];
            let (xj, yj) = poly[j];
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// The pulse's current wavefront as a closed polygon in Kerr-Schild Cartesian coordinates,
    /// x + i y = (r + i a) e^{i phi}, or None once any ray of it has died.
    ///
    /// A dead ray is left standing at the boundary it reached rather than removed, so dropping it
    /// from the polyline would close the loop across a chord that is not part of the front and
    /// keeping it would put a vertex on a ray that is no longer advancing. Either way the loop has
    /// stopped being the pulse's wavefront, so the pulse is simply not asked.
    fn closed_front(metric: &KerrSchild, pulse: &Pulse) -> Option<Vec<(f64, f64)>> {
        pulse
            .rays
            .iter()
            .map(|ray| ray.alive().then(|| metric.cartesian_position(ray.r, ray.phi)))
            .collect()
    }

    #[test]
    fn test_an_observer_never_leaves_the_light_they_have_emitted() {
        // The user-visible fact the fix restores. A pulse is emitted isotropically in the emitter's
        // own frame, so the emitter is at the centre of it; a timelike worldline can never overtake
        // its own light, so the emitter stays inside every front they have sent for as long as that
        // front is a closed loop. Drawn, that is the loops staying wrapped around the marker.
        //
        // Bob selected Static at r = 1.9M with a = 0.90 - inside the ergosphere, where no static
        // observer exists - is the configuration from the bug report. Held at fixed r while his
        // pulses were emitted into a frame falling inward at 0.73c, every one of his fronts ran
        // away from him and he ended up outside his own past light cones: a causality violation
        // manufactured by the inconsistency, since the light and the worldline were being drawn
        // from two different Bobs.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut bob = Observer::new(&metric, "Bob", 0.0, 1.9, 0.0);
        bob.mode = ObserverMode::Static;
        assert!(!bob.mode_admissible(&metric), "no static observer exists at r = 1.9M");

        let mut alice = Observer::new(&metric, "Alice", 0.0, 4.5, 0.0);
        assert!(alice.mode_admissible(&metric));

        for (who, obs) in [("Bob (Static, refused)", &mut bob), ("Alice (free fall)", &mut alice)] {
            let mut field = SignalField::default();
            let dt = 0.01;
            let mut t = 0.0;
            let (mut checked, mut skipped, mut pulses) = (0usize, 0usize, 0usize);
            while t < 3.0 - 1e-12 {
                t += dt;
                obs.step(&metric, t, dt);
                // The order the app advances a field in: carry the light, then emit.
                field.advance(&metric, dt);
                field.emit_if_due(&metric, obs);
                pulses = pulses.max(field.pulses.len());

                let (x, y) = obs.cartesian_position(&metric);
                for pulse in field.pulses.iter() {
                    // A pulse emitted on this very step is still a point, so there is no polygon
                    // to be inside of yet.
                    if pulse.emitted_t >= field.t - 1e-12 {
                        continue;
                    }
                    match closed_front(&metric, pulse) {
                        Some(front) => {
                            assert!(
                                point_in_polygon(&front, x, y),
                                "{who}: at t = {t:.2} (r = {:.4}) he is outside the front of \
                                 pulse #{} emitted at t = {:.2}, r = {:.4}",
                                obs.r,
                                pulse.index,
                                pulse.emitted_t,
                                pulse.emitted_r
                            );
                            checked += 1;
                        }
                        None => skipped += 1,
                    }
                }
                if obs.has_ended() {
                    break;
                }
            }
            println!(
                "{who}: {checked} in-front checks over {pulses} pulses, {skipped} skipped for \
                 dead rays; ended at t = {:.2}, r = {:.4}",
                obs.t, obs.r
            );
            assert!(checked > 200, "{who}: only {checked} checks - the test proved nothing");
        }
    }

    #[test]
    fn test_extreme_zoom_focus() {
        // A full frame with the radial window wound down to a strip 0.05 M wide straddling the
        // Cauchy horizon, which is as far as the wheel will take it. Every tick, every worldline
        // and every wavefront in the (t, r) view is then placed by arithmetic on differences far
        // smaller than r itself, so this is where a projection that only works at the default
        // scale would show it.
        let mut app = SpacetimeApp::default();
        let rm = app.metric.inner_horizon();
        app.spacetime_canvas.max_r = 0.05;
        app.spacetime_canvas.r_offset = rm - 0.025;

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
                    app.bob.as_mut().expect("enabled").reset_with_phi(
                        &app.metric,
                        0.0,
                        r,
                        0.0,
                        WorldlineParams::default(),
                    );
                    if let Some(ref mut al) = app.alice {
                        al.reset_with_phi(&app.metric, 0.0, r * 1.08, 0.35, WorldlineParams::default());
                    }
                    app.ui(ui, &mut frame);
                    assert!(bob_of(&app).r > 0.0);
                }
            }

            // Alice's frame with no Alice in it falls back to Bob, and Bob's frame with no Bob in
            // it falls back to Alice; with neither of them there the rest-frame view has no frame
            // to be and says so instead of drawing one.
            app.controls.frame_of_ref = ReferenceFrame::Alice;
            app.controls.alice.enabled = false;
            for &r in radii.iter() {
                app.bob.as_mut().expect("enabled").reset_with_phi(
                    &app.metric,
                    0.0,
                    r,
                    0.0,
                    WorldlineParams::default(),
                );
                app.ui(ui, &mut frame);
            }
            app.controls.alice.enabled = true;
            app.controls.bob.enabled = false;
            app.controls.frame_of_ref = ReferenceFrame::Bob;
            app.ui(ui, &mut frame);
            app.controls.alice.enabled = false;
            app.ui(ui, &mut frame);
            app.controls.frame_of_ref = ReferenceFrame::Alice;
            app.ui(ui, &mut frame);
            assert!(app.alice.is_none() && app.bob.is_none());
        });
    }

    #[test]
    fn test_bobs_signal_reaches_alice_and_rewinds_through_the_app_loop() {
        // Bob's transmission through the app's own wiring, in the layout a Reset builds and at the
        // default delay: Alice released from r = 4.5M at t = 0, Bob hovering at the same
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
        trailing_raindrops(&mut app);
        app.controls.step_size = 0.02;
        assert_eq!(app.controls.bob.delta_t_delay, 8.0, "a hovering emitter to rewind");

        let step = 0.02;
        for _ in 0..150 {
            app.step_forward(step);
        }
        egui::__run_test_ui(|ui| {
            let mut frame = eframe::Frame::_new_kittest();
            app.ui(ui, &mut frame);
        });

        assert!(!bob_of(&app).is_active, "Bob is still hovering at t = {}", app.current_time);
        assert!((bob_of(&app).r - 4.5).abs() < 1e-12, "and has not moved: r = {}", bob_of(&app).r);
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
        assert!(
            !bob_of(&app).is_active && (bob_of(&app).r - 4.5).abs() < 1e-12,
            "he is still hovering"
        );
        assert!(
            (bob_of(&app).t - app.current_time).abs() < 1e-9,
            "and his own clock came back with the simulation's: {} vs {}",
            bob_of(&app).t,
            app.current_time
        );

        // Alice is released and falling, so she is on the clock to the last bit as well.
        let alice = alice_of(&app);
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
        // nobody. The HUD line that names that pulse reads exactly these two calls, and the
        // frame drawn at the end is the state it reads them in.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
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

        let alice = alice_of(&app);
        assert!(alice.has_ended(), "the run must reach the end of Alice's worldline: r = {}", alice.r);
        let last = app
            .bob_signal
            .last_delivered_pulse()
            .expect("he transmits from t = 0 while he hovers, and the ingoing part of each cone catches her");
        let never = app.bob_signal.pulses_after(last.pulse_index);
        println!(
            "app loop: Alice ends at t = {:?}; last delivered #{} sent at t = {:.3}, r = {:.4},              Bob's tau = {:.3}, reaching her at t = {:.3}; {never} later pulses never arrive",
            ended_at,
            last.pulse_index,
            last.emitted_t,
            last.emitted_r,
            last.emitted_tau,
            last.received_t
        );
        // The numbers of the layout the app opens on, which are the ones the hover text on Bob's
        // Transmit Signal box quotes: the boundary pulse is one he sends while still hovering at
        // r = 4.5M, well before his own release at t = 8, and everything after it is sent to
        // nobody. Re-measured for this layout - the app used to open with him at r = 3.8M released
        // at t = 0, where the boundary was pulse #25, sent at t = 3.84 from r = 1.23, with 3
        // pulses after it.
        assert!(
            (last.emitted_t - 1.84).abs() < 0.02,
            "the boundary pulse is the one of t = 1.84: {}",
            last.emitted_t
        );
        assert!(
            (last.emitted_r - 4.5).abs() < 1e-9,
            "sent from the hover radius, before his release: r = {}",
            last.emitted_r
        );
        assert!(
            (last.received_t - 5.20).abs() < 0.02,
            "and reaching her at t = 5.20, most of an M before her worldline ends: {}",
            last.received_t
        );
        assert!(never >= 40, "Bob goes on transmitting after the last delivery: {never}");
        assert!(
            last.emitted_t < app.current_time,
            "the boundary event is in the past of the current frame"
        );
        for pulse in app.bob_signal.pulses.iter().filter(|p| p.index > last.pulse_index) {
            assert!(pulse.receptions.is_empty(), "pulse {} cannot have arrived", pulse.index);
        }
    }

    /// Every arrival a transmission has recorded, ordered by its own crossing time: the pulse it
    /// belongs to, when and where it crossed the receiver, and the shift it carried. This is the
    /// whole reception record of a field, and it is what a rewind followed by the same steps
    /// forward has to reproduce.
    fn reception_list(field: &SignalField) -> Vec<(usize, f64, f64, f64)> {
        let mut out: Vec<(usize, f64, f64, f64)> = field
            .receptions()
            .map(|rec| (rec.pulse_index, rec.t, rec.r, rec.ratio))
            .collect();
        out.sort_by(|a, b| a.1.total_cmp(&b.1));
        out
    }

    /// Assert that two reception records are the same events: the same pulses in the same order, at
    /// the same crossing times to `t_tol` and with the same shifts to a relative `ratio_tol`.
    fn assert_same_receptions(
        label: &str,
        now: &[(usize, f64, f64, f64)],
        then: &[(usize, f64, f64, f64)],
        t_tol: f64,
        ratio_tol: f64,
    ) -> (f64, f64) {
        assert_eq!(
            now.len(),
            then.len(),
            "{label}: the run recorded {} arrivals where it had recorded {}:\n\
             {now:?}\nvs\n{then:?}",
            now.len(),
            then.len()
        );
        let mut worst_t = 0.0f64;
        let mut worst_ratio = 0.0f64;
        for (a, b) in now.iter().zip(then.iter()) {
            assert_eq!(a.0, b.0, "{label}: a different pulse: {a:?} vs {b:?}");
            worst_t = worst_t.max((a.1 - b.1).abs());
            worst_ratio = worst_ratio.max((a.3 - b.3).abs() / b.3);
            assert!(
                (a.1 - b.1).abs() < t_tol,
                "{label}: the crossing moved by {}: {a:?} vs {b:?}",
                (a.1 - b.1).abs()
            );
            assert!(
                (a.3 - b.3).abs() < ratio_tol * b.3,
                "{label}: the shift moved by {}: {a:?} vs {b:?}",
                (a.3 - b.3).abs()
            );
        }
        (worst_t, worst_ratio)
    }

    #[test]
    fn test_receptions_come_back_unchanged_after_a_rewind() {
        // Stepping back and forward over the same interval has to leave the same arrivals on the
        // record. Two separate defects used to stop that happening, and both are through this
        // path - the app's own transport, which is what the arrow keys and the panel's buttons
        // call.
        //
        // The first was the sides. `SignalField::step_back` drops the per-sheet record of which
        // side of each wavefront the receiver stands on, because a side is a statement about two
        // events and cannot be wound back; but with nothing re-establishing them, the first step
        // forward was spent finding out which side he was on, and any crossing inside that step was
        // never seen. `SignalPair::step_back` now primes both fields at the rewound state, which is
        // why the observers are rewound before the fields.
        //
        // The second was the stamp. An arrival used to carry the time of the pass that noticed it
        // rather than the time of the crossing, and a rewind drops the arrivals later than its
        // target: a crossing that happened just before the target but was noticed just after it was
        // dropped and then never re-detected, because the side re-established on the way forward
        // was already the far one. The stamp is now the interpolated crossing event.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        zamo_alice_falling_bob(&mut app);
        let step = 0.02;
        for _ in 0..200 {
            app.step_forward(step);
        }
        let forward_time = app.current_time;
        let alice_then = reception_list(&app.signal);
        let bob_then = reception_list(&app.bob_signal);
        assert!(
            alice_then.len() >= 8 && bob_then.len() >= 8,
            "both transmissions must have been heard by t = {forward_time}: {} and {}",
            alice_then.len(),
            bob_then.len()
        );

        // Two thirds of an M back, one arrow press at a time. The count is chosen so that the
        // rewind reaches past several arrivals in each field *and* leaves one of Alice's crossings
        // strictly inside the first step forward, which is the step that used to lose it: a field
        // whose sides have been dropped and not re-primed spends that step working out which side
        // of each sheet the receiver is on, and sees no crossing at all. In the layout the app
        // opens on, Bob hears her at t = 1.774, 2.001, 2.243, 2.499, 2.770, 3.057, 3.358 and 3.673
        // - every one of them at r = 4.5M, since he is hovering there - and 33 steps back from
        // t = 4 lands on t = 3.34, which puts the arrival at t = 3.358 inside the first step
        // forward and leaves two arrivals in each field to be retracted and re-recorded.
        let back = 33;
        for _ in 0..back {
            app.step_backward(step);
        }
        let in_the_first_step = alice_then
            .iter()
            .filter(|rec| rec.1 > app.current_time && rec.1 < app.current_time + step)
            .count();
        assert!(
            in_the_first_step >= 1,
            "the rewind must leave a crossing inside the first step forward from t = {}: \
             {alice_then:?}",
            app.current_time
        );
        let retracted_alice = alice_then.len() - app.signal.received_count();
        let retracted_bob = bob_then.len() - app.bob_signal.received_count();
        assert!(
            retracted_alice >= 2 && retracted_bob >= 2,
            "the rewind must reach past several arrivals: {retracted_alice} and {retracted_bob}"
        );
        for (field, then) in [(&app.signal, &alice_then), (&app.bob_signal, &bob_then)] {
            for rec in field.receptions() {
                assert!(
                    rec.t <= app.current_time + 1e-12,
                    "an arrival at t = {} survived a rewind to t = {}",
                    rec.t,
                    app.current_time
                );
            }
            let kept = reception_list(field);
            assert_same_receptions("kept by the rewind", &kept, &then[..kept.len()], 1e-12, 1e-12);
        }

        // Forward over the same interval again: the same arrivals, at the same events.
        for _ in 0..back {
            app.step_forward(step);
        }
        assert!(
            (app.current_time - forward_time).abs() < 1e-9,
            "back on the same clock: {} vs {forward_time}",
            app.current_time
        );
        let (alice_t, alice_ratio) = assert_same_receptions(
            "Alice -> Bob",
            &reception_list(&app.signal),
            &alice_then,
            1e-9,
            1e-6,
        );
        let (bob_t, bob_ratio) = assert_same_receptions(
            "Bob -> Alice",
            &reception_list(&app.bob_signal),
            &bob_then,
            1e-9,
            1e-6,
        );
        println!(
            "{} + {} arrivals over {back} steps of {step} rewound and re-run: worst crossing-time \
             difference {:.3e} M, worst relative shift difference {:.3e} ({retracted_alice} and \
             {retracted_bob} arrivals were retracted by the rewind and re-recorded, \
             {in_the_first_step} of them inside the first step forward)",
            alice_then.len(),
            bob_then.len(),
            alice_t.max(bob_t),
            alice_ratio.max(bob_ratio)
        );
    }

    #[test]
    fn test_a_rewind_that_lands_exactly_on_an_arrival_neither_drops_it_nor_repeats_it() {
        // The boundary case of the retraction rule. A rewind keeps the arrivals with t <= target
        // and drops the rest, so an arrival stamped exactly at the target is on the edge of it: it
        // must survive as one arrival, not vanish and not be recorded twice when the same interval
        // is stepped forward again.
        //
        // The stamp is the interpolated crossing, which is first-order accurate in the pass
        // interval, so landing on it leaves the record and the geometry free to disagree over an
        // O(dt^2) window: the arrival is on the record, and the receiver may or may not have
        // reached that sheet yet at the rewound state. `SignalField::prime` settles it by looking,
        // and both outcomes give exactly one arrival. If the crossing has happened, the primed side
        // is the far one, the arrival stands and nothing calls it a crossing again. If it has not,
        // the arrival is retracted - the pass that recorded it is inside the interval being
        // undone - and the next step forward brackets the same crossing and records it again,
        // moved by the O(dt^2) the stamp was uncertain by. Measured here: the arrival is retracted
        // and comes back 1.2e-5 M later, which is 0.03 dt^2.
        //
        // Everything before the target is untouched to round-off. Nothing follows the target here,
        // by choice; see the note on the sheet key below.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        let step = 0.02;
        for _ in 0..200 {
            app.step_forward(step);
        }
        let forward_time = app.current_time;
        let alice_then = reception_list(&app.signal);
        let bob_then = reception_list(&app.bob_signal);

        // The last arrival of Alice's transmission that is far enough back for the rewind to be a
        // rewind of several steps, and its exact crossing time is the target.
        //
        // Deliberately the last one. Landing further back moves the whole pass grid of everything
        // that follows, and what a re-interpolated crossing time is worth at a given step size is
        // measured where it belongs, in
        // `wavefront::tests::test_one_pulse_crossings_are_the_same_at_a_quarter_of_the_step`.
        let target = alice_then
            .iter()
            .map(|rec| rec.1)
            .filter(|t| *t < forward_time - 0.05)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(target.is_finite(), "the run must have an arrival to land on: {alice_then:?}");
        let interval = forward_time - target;
        app.step_backward(interval);
        assert!(
            (app.current_time - target).abs() < 1e-12,
            "the clock must land on the arrival: {} vs {target}",
            app.current_time
        );
        let on_the_boundary = app
            .signal
            .receptions()
            .filter(|rec| (rec.t - target).abs() < 1e-9)
            .count();
        println!(
            "rewound {interval:.5} M onto the arrival at t = {target}: the priming pass {} it",
            if on_the_boundary == 1 {
                "finds the crossing already made and keeps"
            } else {
                "finds the receiver still short of the sheet and retracts"
            }
        );
        assert!(on_the_boundary <= 1, "an arrival cannot be kept twice: {on_the_boundary}");

        // Forward again on the same step size, not in one jump: a detection pass half an M long
        // would sweep whole sheets past the receiver between two looks and see none of them, which
        // is a statement about the pass cadence rather than about the rewind. The grid is now
        // offset from the first run's by the fraction of a step the target sat at, which is what
        // makes the arrivals after it a genuine re-interpolation.
        while app.current_time < forward_time - 1e-12 {
            app.step_forward(step.min(forward_time - app.current_time));
        }
        assert!(
            (app.current_time - forward_time).abs() < 1e-9,
            "back on the same clock: {} vs {forward_time}",
            app.current_time
        );
        let alice_now = reception_list(&app.signal);
        let bob_now = reception_list(&app.bob_signal);
        let boundary: Vec<_> = alice_now
            .iter()
            .filter(|rec| (rec.1 - target).abs() < 1e-3)
            .copied()
            .collect();
        assert_eq!(
            boundary.len(),
            1,
            "the arrival on the boundary must be there exactly once: {alice_now:?}"
        );
        println!(
            "after the same interval forward it is back once, {:.3e} M from where it was, which is \
             {:.2} dt^2",
            (boundary[0].1 - target).abs(),
            (boundary[0].1 - target).abs() / (step * step)
        );

        // The whole record, either side of the target: the same arrivals in the same order, the
        // ones before the target to round-off and the one on it to the O(dt^2) its stamp was
        // uncertain by.
        for (label, now, then) in [
            ("Alice -> Bob", &alice_now, &alice_then),
            ("Bob -> Alice", &bob_now, &bob_then),
        ] {
            let strictly_before: Vec<_> =
                now.iter().copied().filter(|rec| rec.1 < target - 1e-3).collect();
            let then_before: Vec<_> =
                then.iter().copied().filter(|rec| rec.1 < target - 1e-3).collect();
            assert_same_receptions(label, &strictly_before, &then_before, 1e-9, 1e-9);
            let (worst_t, worst_ratio) = assert_same_receptions(label, now, then, 1e-3, 1e-3);
            println!(
                "{label}: {} arrivals, {} of them before the target and unchanged to 1e-9; over \
                 the whole record nothing moved more than {worst_t:.3e} M in time or \
                 {worst_ratio:.3e} relative in shift, against dt^2 = {:.1e}",
                now.len(),
                strictly_before.len(),
                step * step
            );
        }
    }

    /// Two layouts are the same run: the same observers, at the same events, on the same clocks,
    /// with the same two transmissions behind them.
    fn assert_same_layout(label: &str, now: &[LayoutRow], then: &[LayoutRow]) {
        assert_eq!(now.len(), then.len(), "{label}: a different set of observers");
        for (a, b) in now.iter().zip(then.iter()) {
            assert_eq!(a.0, b.0, "{label}: {a:?} vs {b:?}");
            assert_eq!((a.6, a.7), (b.6, b.7), "{label}: {a:?} vs {b:?}");
            for (x, y, what) in [
                (a.1, b.1, "t"),
                (a.2, b.2, "r"),
                (a.3, b.3, "phi"),
                (a.4, b.4, "tau"),
                (a.5, b.5, "release_t"),
                (a.8, b.8, "field clock"),
            ] {
                assert!(
                    (x - y).abs() <= 1e-12,
                    "{label}: {}'s {what} is {x} where it was {y}",
                    a.0
                );
            }
        }
    }

    #[test]
    fn test_startup_and_reset_build_the_same_layout() {
        // Two ways into the same run. There used to be a third, a Drop Observers button that put
        // Bob at 4.5M with a delay while Reset put him at 3.8M released at once, so the app opened
        // on one layout, one button rebuilt it and the other built a different one. That button is
        // gone and what is left goes through `AppControls::drop_observers`, which this pins: a
        // Reset, a Reset of a Reset, and `SpacetimeApp::default` all agree.
        let app = SpacetimeApp::default();
        let opening = layout(&app);
        assert_eq!(opening.len(), 2, "both cards are ticked out of the box");
        assert_eq!(opening[0].0, "Alice");
        let isco = app.metric.isco(true);
        assert!((opening[0].2 - isco).abs() < 1e-12, "Alice drops onto the prograde ISCO");
        assert!(opening[0].5 == 0.0 && opening[0].6, "released at once");
        assert!((opening[1].2 - 4.5).abs() < 1e-12, "and Bob from r = 4.5M");
        assert!(opening[1].5 == 0.0 && opening[1].6, "and at the same moment");
        // Both in free fall; the difference between them is the worldline each is released on.
        assert_eq!(alice_of(&app).mode, ObserverMode::FreeFall, "Alice circles the ISCO");
        assert_eq!(alice_of(&app).release, Release::CircularPrograde);
        assert_eq!(bob_of(&app).mode, ObserverMode::FreeFall, "Bob falls");

        // Part-way through a run, with light in flight and Bob's worldline well below the drop.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        for _ in 0..120 {
            app.step_forward(0.05);
        }
        assert!(bob_of(&app).r < 2.0 && !app.signal.pulses.is_empty(), "a run in progress");

        // The action behind ⏮ Reset, and behind the preset row.
        drop_observers(&mut app);
        assert_eq!(app.current_time, 0.0, "the clock goes back with it");
        assert_same_layout("Reset vs startup", &layout(&app), &opening);

        // And again from the rebuilt state: a Reset of a Reset is the same Reset.
        for _ in 0..30 {
            app.step_forward(0.05);
        }
        drop_observers(&mut app);
        assert_same_layout("Reset of a Reset vs startup", &layout(&app), &opening);
    }

    #[test]
    fn test_an_unticked_observer_is_not_in_the_simulation_at_all() {
        // Bob's card unticked: no worldline to step, nothing of him in either view, no telemetry
        // box, no HUD line, nothing transmitted and nothing received. Alice's transmission goes on
        // - light already sent does not care whether anyone is left to hear it - but records not
        // one arrival, because an arrival is a crossing of a receiver's worldline and there is no
        // receiver.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
        app.controls.bob.enabled = false;
        // The panel is what applies a card, so one frame has to run before he goes.
        let opening = painted_text(&mut app);
        assert!(app.bob.is_none(), "the unticked card takes him out of the run");
        assert!(opening.contains("OBSERVER BOB"), "his card is still there to tick back on");

        for _ in 0..150 {
            app.step_forward(0.02);
        }
        let painted = painted_text(&mut app);
        println!(
            "3 M with Bob's card unticked: Alice at r = {:.4} with {} pulses in flight; Bob's \
             field holds {} pulses and {} arrivals, hers {} arrivals",
            alice_of(&app).r,
            app.signal.pulses.len(),
            app.bob_signal.pulses.len(),
            app.bob_signal.received_count(),
            app.signal.received_count()
        );

        assert!(app.bob.is_none(), "and keeps him out of it");
        assert!(app.bob_signal.pulses.is_empty(), "he transmits nothing");
        assert_eq!(app.bob_signal.received_count(), 0, "and there is nothing of his to hear");
        assert_eq!(app.signal.received_count(), 0, "nobody is there to hear Alice either");
        assert!(!app.signal.pulses.is_empty(), "though she goes on transmitting");
        assert!(alice_of(&app).r < 4.0, "and goes on falling: r = {}", alice_of(&app).r);

        // What the frame says. "Alice [Reg II (Trapped)]" is the title line of a telemetry box and
        // "Bob τ:" the HUD's clock; the word "Bob" by itself is no test at all, because his card is
        // on the panel whether he is in the run or not.
        assert!(painted.contains("Alice ["), "her telemetry box is drawn: {painted}");
        assert!(!painted.contains("Bob ["), "his is not");
        assert!(!painted.contains("Bob τ"), "nor his HUD clock");
        // The colon is the HUD's line; the equatorial view's legend says "amber = Alice → Bob"
        // whatever is on the canvas, because it is naming a colour rather than a reception.
        assert!(!painted.contains("Alice → Bob:"), "nor the line about what he has heard");

        // Ticking the card back on drops him afresh from it, on the clock the run has reached
        // rather than behind it, and hovering until his own delay has passed from there.
        app.controls.bob.enabled = true;
        let painted = painted_text(&mut app);
        let bob = bob_of(&app);
        assert!((bob.t - app.current_time).abs() < 1e-12, "he starts on the clock: {}", bob.t);
        assert!((bob.r - 4.5).abs() < 1e-12, "at the drop radius: {}", bob.r);
        assert!(
            (bob.release_t - (app.current_time + 8.0)).abs() < 1e-12,
            "released his own delay later: {} with the clock at {}",
            bob.release_t,
            app.current_time
        );
        assert!(painted.contains("Bob ["), "and he is back in the telemetry: {painted}");
    }

    #[test]
    fn test_an_untransmitting_observer_empties_their_field_and_is_never_heard() {
        // The Transmit Signal box unticked: the field is dropped and stays empty, so there is
        // nothing of theirs to draw and the other observer records nothing from them. The observer
        // themselves is untouched - still falling, still listening - which is the difference
        // between this box and the one above it.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        trailing_raindrops(&mut app);
        for _ in 0..100 {
            app.step_forward(0.02);
        }
        let heard = app.signal.received_count();
        assert!(heard > 0 && !app.signal.pulses.is_empty(), "Bob has been hearing her: {heard}");

        app.controls.alice.transmit = false;
        for _ in 0..100 {
            app.step_forward(0.02);
        }
        println!(
            "2 M after Alice's transmission was switched off: her field holds {} pulses and {} \
             arrivals (it held {heard}), his {} pulses and {} arrivals",
            app.signal.pulses.len(),
            app.signal.received_count(),
            app.bob_signal.pulses.len(),
            app.bob_signal.received_count()
        );
        assert!(app.signal.pulses.is_empty(), "her field is dropped and stays empty");
        assert_eq!(app.signal.received_count(), 0, "and the arrivals go with the light");
        assert!(
            (app.signal.t - app.current_time).abs() < 1e-9,
            "a silent field still rides the simulation clock: {} vs {}",
            app.signal.t,
            app.current_time
        );

        // She is otherwise exactly where she would have been, and still hearing him.
        assert!(alice_of(&app).r < 4.0 && !alice_of(&app).has_ended());
        assert!(!app.bob_signal.pulses.is_empty(), "he is still transmitting");
        assert!(app.bob_signal.received_count() > 0, "and she is still hearing him");

        // Nothing of hers is left to draw, and the frame still renders.
        let painted = painted_text(&mut app);
        assert!(painted.contains("Alice ["), "she is still on the canvas: {painted}");

        // Ticked back on, she starts sending again from where she now is.
        app.controls.alice.transmit = true;
        for _ in 0..20 {
            app.step_forward(0.02);
        }
        assert!(!app.signal.pulses.is_empty(), "the transmission resumes");
        for pulse in app.signal.pulses.iter() {
            assert!(
                pulse.emitted_t > 4.0 - 1e-9,
                "and everything in the field was sent after the silence, not before it: {}",
                pulse.emitted_t
            );
        }
    }

    #[test]
    fn test_the_panel_renders_with_every_combination_of_the_two_cards() {
        // Four runs of the panel, one per combination of the two Enable boxes, each in all three
        // reference frames, and with either observer in the held state a drag puts them in.
        // The rest-frame views in particular have to cope with the observer whose frame was asked
        // for being absent, and with both of them being absent.
        for alice_on in [true, false] {
            for bob_on in [true, false] {
                let mut app = SpacetimeApp::default();
                app.controls.is_playing = false;
                app.controls.alice.enabled = alice_on;
                app.controls.bob.enabled = bob_on;

                for &frame_of_ref in &[
                    ReferenceFrame::DistantObserver,
                    ReferenceFrame::Bob,
                    ReferenceFrame::Alice,
                ] {
                    app.controls.frame_of_ref = frame_of_ref;
                    for use_physical_units in [true, false] {
                        app.controls.use_physical_units = use_physical_units;
                        let painted = painted_text(&mut app);
                        assert_eq!(app.alice.is_some(), alice_on, "Alice's card is the authority");
                        assert_eq!(app.bob.is_some(), bob_on, "and Bob's is his");
                        assert!(
                            painted.contains("OBSERVER ALICE") && painted.contains("OBSERVER BOB"),
                            "both cards are always on the panel: {painted}"
                        );
                        // And with both of them in the held state a marker drag puts them in,
                        // which no control selects any more but every drag passes through.
                        for obs in [app.alice.as_mut(), app.bob.as_mut()].into_iter().flatten() {
                            obs.mode = ObserverMode::ManualDrag;
                        }
                        painted_text(&mut app);
                    }
                }

                // And with the run moving, which is where an absent observer would be stepped.
                for _ in 0..20 {
                    app.step_forward(0.05);
                }
                painted_text(&mut app);
            }
        }
    }

    #[test]
    fn test_the_wavefront_slider_reaches_the_next_pulse_of_both_transmissions() {
        // The panel's "Wavefront points" is a standing request read on the way into a step, so a
        // test that steps the app by hand sees exactly what a played frame sees. Both transmissions
        // take it - one slider, one sampling, or the two pictures would not be comparable - and
        // both leave the light already in flight alone.
        let mut app = SpacetimeApp::default();
        app.controls.is_playing = false;
        app.controls.rays_per_pulse = 64;
        app.step_forward(0.05);
        let first: Vec<usize> =
            [&app.signal, &app.bob_signal].iter().map(|f| f.pulses.len()).collect();
        assert_eq!(first, vec![1, 1], "Alice falling and Bob hovering both transmit at once");
        assert_eq!(app.signal.pulses[0].rays.len(), 64);
        assert_eq!(app.bob_signal.pulses[0].rays.len(), 64);

        // Turned up part-way through the run. Bob is still hovering, so his clock runs slower than
        // Alice's and his second pulse is a little later than hers; stepping until both fields hold
        // two pulses is what "the next pulse of each" means.
        app.controls.rays_per_pulse = 256;
        for _ in 0..40 {
            if app.signal.pulses.len() > 1 && app.bob_signal.pulses.len() > 1 {
                break;
            }
            app.step_forward(0.05);
        }
        for (who, field) in [("Alice", &app.signal), ("Bob", &app.bob_signal)] {
            assert!(field.pulses.len() > 1, "{who} sent a second pulse by t = {}", app.current_time);
            assert_eq!(
                field.pulses.back().unwrap().rays.len(),
                256,
                "{who}'s newest pulse carries the count the slider now shows"
            );
            assert_eq!(
                field.pulses[0].rays.len(),
                64,
                "{who}'s first pulse keeps the count it was emitted with"
            );
            assert_eq!(field.rays_per_pulse, 256, "and both fields agree on what comes next");
        }
    }

    #[test]
    fn test_the_panel_renders_at_both_ends_of_the_wavefront_slider() {
        // The slider's own range, laid out and stepped through. 1024 rays a pulse is the expensive
        // end and the one a layout mistake would show up at first.
        for rays in [64usize, 1024] {
            let mut app = SpacetimeApp::default();
            app.controls.is_playing = false;
            app.controls.rays_per_pulse = rays;
            let painted = painted_text(&mut app);
            assert!(painted.contains("Wavefront points"), "the slider is on the panel: {painted}");
            app.step_forward(0.05);
            assert_eq!(app.signal.pulses[0].rays.len(), rays);
            assert_eq!(app.bob_signal.pulses[0].rays.len(), rays);
            // And a frame with that pulse standing in both fields, which is the drawing path.
            painted_text(&mut app);
        }
    }
}
