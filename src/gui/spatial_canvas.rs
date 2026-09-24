use crate::gui::units::UnitLabels;
use crate::gui::controls::{ReferenceFrame, SignalViews};
use crate::gui::polyline::{SCREEN_SPACING, thin_to_pixels};
use crate::gui::numbers;
use crate::gui::ruler;
use crate::gui::spacetime_canvas::{BoxId, Canvas, PendingBox, TelemetryBoxes};
use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode};
/// Which of the two observers, re-exported here because the equatorial view is where most of the
/// per-observer drawing is and every user of it in the gui reaches it through this module.
pub use crate::physics::observer::Who;
use crate::physics::wavefront::{NullRay, SignalField};
use egui::{Color32, Pos2, Stroke, Vec2};

/// How the wavefronts of a transmission are drawn on this canvas: the two view settings of the
/// Simulation Control panel that decide what is put on screen *between* the calculated rays.
///
/// Both are drawing choices in the strict sense. `Pulse::scan` interpolates in (r, phi) along the
/// same segments whatever this struct says, so nothing here can move an arrival, change a measured
/// shift or touch the integration; what they change is how much of the picture is inference and how
/// much is the raw output of the integrator. They travel together in one value because they answer
/// the same question, and because two bare bools threaded through two call layers is how the wrong
/// one eventually gets passed.
#[derive(Clone, Copy)]
pub struct FrontStyle {
    /// Draw each segment of a front as the curve linear in (r, phi) between its two rays
    /// (`segment_arc`), or draw nothing between them and leave the front as its calculated points.
    /// See `FRONT_POINT_RADIUS`.
    pub arcs: bool,
    /// Drop the segments whose two rays have wound more than `MAX_RESOLVED_WINDING` apart, which
    /// the sampling cannot resolve and which the interpolation draws along a curve no ray took.
    pub hide_wound: bool,
}

impl Who {
    /// The drawn radius of this observer's marker on the equatorial view, which also sets how
    /// close the pointer has to come to open their menu.
    pub(crate) fn marker_radius(self) -> f32 {
        match self {
            Self::Alice => 5.0,
            Self::Bob => 7.0,
        }
    }
}

/// How close to the ring a drag may put an observer. The ring is the curvature singularity and
/// the end of every worldline that reaches it, so a drop onto r = 0 is a drop onto nothing there is
/// a frame at.
pub(crate) const RING_DROP_FLOOR: f64 = 0.04;

/// A marker drag in progress: whose it is, and the Motion they were on when it started.
///
/// The Motion is kept because `Observer::set_drag_position` puts whoever is being moved into
/// `ObserverMode::ManualDrag` for as long as the pointer holds them - that is what a hand on the
/// marker means - and dropping them has to give back the worldline they were on. A free-faller is
/// released again at the event they were dropped at, on their own `Release`; a ZAMO goes back to
/// holding the new radius. Nothing selects that held state from the panel: it is the mechanism of
/// the drag and nothing else.
#[derive(Clone, Copy, PartialEq, Debug)]
struct MarkerDrag {
    who: Who,
    mode_before: ObserverMode,
    /// Marker centre minus pointer at the moment the pointer took hold, in screen pixels, added
    /// back on every frame of the drag. Without it the observer is placed *at* the pointer, so
    /// picking a marker up anywhere but dead centre teleports it under the cursor by as much as
    /// three marker radii before the drag has moved at all.
    grab: Vec2,
    /// The point of the plane the view was anchored to when the pointer took hold - Cartesian, in
    /// M, as `Observer::cartesian_position` gives it, or None for the hole - held as the anchor
    /// until the pointer lets go. The picture has to stand still under the pointer for the drag to
    /// mean what it says. A view following the observer being moved - their rest frame from the
    /// View selector, or Keep Centered - would re-centre on them every frame, the pointer would
    /// then stand off the marker by whatever it had moved, and the next frame would move them by
    /// that much again: a nudge of a few pixels carried them across the plane for as long as the
    /// button was down. Held in M rather than in pixels, so a wheel zoom during the drag scales it.
    anchor: Option<(f64, f64)>,
}

pub struct SpatialCanvas {
    pub zoom: f32,
    pub pan_offset: Vec2,
    /// The observer the view is being kept centred on, set from the canvas's right-click menu.
    ///
    /// It is a view setting, held here with the pan and the zoom rather than on the panel: it says
    /// where the canvas is looking and nothing about the physics, and like the zoom it survives a
    /// Reset. It is independent of the View selector, which centres the view on
    /// whoever's rest frame is being drawn; this one can keep Bob in the middle of a view drawn in
    /// the global foliation, which is the case the selector cannot express. Where the two disagree
    /// this one wins, being the more particular request, and it falls back to the selector's
    /// choice while the observer it names is not in the simulation.
    pub(crate) centred_on: Option<Who>,
    /// A standing request to keep the black hole in the middle of the view: the origin of the
    /// embedding, the centre of the ring. Like `centred_on` it is about where the canvas is
    /// looking and survives a Reset, and the two are exclusive - the menu clears one when the
    /// other is set. It overrides the View selector's tracking of a rest frame's observer, which
    /// is the case it exists for: a rest frame with the hole held still and the observer falling
    /// across the picture. While it holds, the pan is pinned at zero, so a drag or a wheel zoom
    /// about the cursor cannot carry the hole away from the middle; the zoom is about the hole.
    pub(crate) keep_hole_centred: bool,
    /// The marker drag in progress on this canvas, if any. Cleared when the pointer is released,
    /// when the observer being dragged leaves the simulation, and by `SpatialCanvas::end_drag` when
    /// the run is rebuilt under it.
    dragging: Option<MarkerDrag>,
    /// Where the user has dragged each info box on this canvas, per observer.
    pub telemetry: TelemetryBoxes,
}

impl Default for SpatialCanvas {
    fn default() -> Self {
        Self {
            zoom: 48.0, // pixels per M
            pan_offset: Vec2::ZERO,
            centred_on: None,
            keep_hole_centred: false,
            dragging: None,
            telemetry: TelemetryBoxes::pinning().starting_shut(&[Canvas::Spatial]),
        }
    }
}

impl SpatialCanvas {
    /// Forget any marker drag in progress. Called when the run is rebuilt under the canvas - the
    /// observer being held is replaced by a fresh one from their card - so that the next pointer
    /// press starts a drag of the new worldline rather than continuing one of a worldline that no
    /// longer exists. Both markers are pickable again the moment the reset lands.
    pub fn end_drag(&mut self) {
        self.dragging = None;
    }

    /// Start, continue and finish a drag of either observer's marker.
    ///
    /// One set of rules for both of them. A press within three marker radii picks up the nearest
    /// marker under it; while the pointer holds it the observer stands at the chart point under the
    /// pointer, put there by `Observer::set_drag_position`, which is what a hand on the marker
    /// means: the worldline is being placed rather than integrated. Releasing hands them back the
    /// Motion they were on through `Observer::release_from_drag`, which releases them again at the
    /// event they were dropped at, so a drag asks "what if they were here" without answering the
    /// separate question of how they move.
    ///
    /// This is the view to ask it in. The plane it draws is where the two observers actually stand
    /// relative to one another, so a drag here sets both coordinates an equatorial observer has -
    /// the radius and the azimuth - and the pair can be put on opposite sides of the hole, which no
    /// control could say before. The (t, r) diagram, where this used to live, squeezes the whole
    /// plane onto one axis: a drag there could only ever slide somebody along the radius, and the
    /// other axis was a time, which is not a thing an observer can be moved along at all.
    ///
    /// The azimuth is free in a way the radius is not: Kerr is axisymmetric, so moving an observer
    /// in phi changes none of their constants - E and L and the whole radial problem are untouched
    /// - and only the *difference* in azimuth between the two observers means anything.
    ///
    /// A drag whose observer has left the simulation - their card unticked while the pointer is
    /// down - is dropped rather than carried, because there is no worldline left to place.
    #[allow(clippy::too_many_arguments)] // the same count the drawing paths carry
    fn drag_markers(
        &mut self,
        metric: &KerrSchild,
        response: &egui::Response,
        alice: Option<&mut Observer>,
        bob: Option<&mut Observer>,
        current_time: f64,
        anchor: Option<(f64, f64)>,
        to_screen: impl Fn(&Observer) -> Pos2,
        to_chart: impl Fn(Pos2) -> (f64, f64),
    ) {
        let mut markers: Vec<(Who, &mut Observer)> = Vec::new();
        if let Some(al) = alice {
            markers.push((Who::Alice, al));
        }
        if let Some(b) = bob {
            markers.push((Who::Bob, b));
        }

        if response.drag_started()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            let mut nearest: Option<(MarkerDrag, f32)> = None;
            for (who, obs) in markers.iter() {
                let reach = who.marker_radius() * 3.0;
                let at = to_screen(obs);
                let distance = at.distance(pointer);
                if distance < reach && nearest.is_none_or(|(_, best)| distance < best) {
                    let drag = MarkerDrag { who: *who, mode_before: obs.mode, grab: at - pointer, anchor };
                    nearest = Some((drag, distance));
                }
            }
            self.dragging = nearest.map(|(drag, _)| drag);
        }

        let Some(drag) = self.dragging else {
            return;
        };
        match markers.iter_mut().find(|(who, _)| *who == drag.who) {
            Some((_, obs)) => {
                if response.dragged()
                    && let Some(pointer) = response.interact_pointer_pos()
                {
                    let (r, phi) = to_chart(pointer + drag.grab);
                    obs.set_drag_position(current_time, r);
                    obs.phi = phi;
                }
                if response.drag_stopped() {
                    obs.release_from_drag(metric, drag.mode_before);
                    self.dragging = None;
                }
            }
            None => self.dragging = None,
        }
    }

    /// The observer the view is anchored to this frame, or None when it is anchored to the hole.
    ///
    /// Following an observer who is not in the simulation is following nobody, so the view stays on
    /// the hole rather than on a remembered position. A standing request to keep one of them
    /// centred is answered first, and the View selector's own tracking is what is
    /// left when there is no such request or the observer it names has gone.
    fn followed<'a>(
        &self,
        bob: &'a Option<Observer>,
        alice: &'a Option<Observer>,
        frame_of_ref: ReferenceFrame,
    ) -> Option<&'a Observer> {
        if self.keep_hole_centred {
            return None;
        }
        frame_focus(self.centred_on, frame_of_ref, bob.as_ref(), alice.as_ref())
    }

    /// The point of the plane the view is anchored to this frame, Cartesian in M, or None for the
    /// hole.
    ///
    /// While a marker is held it is the anchor there was when the pointer took hold, and nothing
    /// that happens during the drag moves it; see `MarkerDrag::anchor`. It resumes the moment the
    /// pointer lets go, so a view following the observer who was moved re-centres on wherever they
    /// were put. That clause is this canvas's alone - it is about the marker drag, which only this
    /// canvas offers - and the rest of the rule is `frame_focus`, shared with the volume view.
    fn anchor(
        &self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
    ) -> Option<(f64, f64)> {
        match self.dragging {
            Some(drag) => drag.anchor,
            None => self
                .followed(bob, alice, frame_of_ref)
                .map(|obs| obs.cartesian_position(metric)),
        }
    }

    /// Where the view's anchor sits in the drawn plane, in screen pixels.
    ///
    /// This is the offset `render` subtracts to place the canvas, and it is what every pan is
    /// measured against: the world point in the middle of the canvas is the one whose own offset is
    /// `tracking - pan_offset`.
    fn tracking(
        &self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
    ) -> Vec2 {
        let zoom = self.zoom;
        self.anchor(metric, bob, alice, frame_of_ref)
            .map_or(Vec2::ZERO, |(x, y)| Vec2::new(x as f32 * zoom, -(y as f32) * zoom))
    }

    /// Pan the view so that the point `target` of the equatorial plane - Cartesian, in M, as
    /// `Observer::cartesian_position` gives it - sits in the middle of the canvas.
    ///
    /// One pan, not a standing request: whoever is there is free to move out of the middle again,
    /// which is what separates the menu's Goto items from its Keep Centered ones. The view's centre
    /// is `rect.center() + pan_offset - tracking`, so putting `target` in the middle means a pan of
    /// `tracking - target`, and when the view is already tracking that observer it is zero.
    pub fn look_at(
        &mut self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
        target: (f64, f64),
    ) {
        let zoom = self.zoom;
        let tracking = self.tracking(metric, bob, alice, frame_of_ref);
        self.pan_offset =
            tracking - Vec2::new(target.0 as f32 * zoom, -(target.1 as f32) * zoom);
    }

    /// Change what the view is anchored to, and answer for the pan while doing it.
    ///
    /// The anchor enters the placement as `centre = rect.center() + pan_offset - tracking`, which
    /// puts the anchor itself at `rect.center() + pan_offset`. So moving the anchor and leaving the
    /// pan alone does two things the user did not ask for: it slides the whole picture by however
    /// far the new anchor is from the old one, and it leaves whatever was just taken hold of
    /// sitting at the accumulated pan rather than in the middle. The pan is not usually zero - a
    /// drag writes it, and so does every wheel zoom about a cursor that is not dead centre, which
    /// compounds as the view zooms in - so "Keep Bob Centered" could put Bob clean off the canvas.
    /// Taking hold of an anchor has to state where the view should then be looking.
    ///
    /// Two answers, and the caller says which:
    ///
    /// * Taking hold brings the new anchor to the middle, so the pan measured from it is zero.
    ///   "Keep Bob Centered" means Bob in the middle, whatever the view was doing beforehand.
    /// * Letting go keeps the same world point in the middle, so the view stays where the user was
    ///   looking and the observer drifts out of it, rather than the hole snapping back under the
    ///   cursor. That point's offset is `tracking - pan_offset`, so holding it still across the
    ///   change means `pan_offset += tracking_after - tracking_before`.
    ///
    /// A Goto is neither of those and stays `look_at`: it moves the view once and changes no
    /// anchor, so nothing about what the view holds on to is being restated.
    fn re_anchor(
        &mut self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
        take_hold: bool,
        change: impl FnOnce(&mut Self),
    ) {
        let before = self.tracking(metric, bob, alice, frame_of_ref);
        change(self);
        let after = self.tracking(metric, bob, alice, frame_of_ref);
        self.pan_offset =
            if take_hold { Vec2::ZERO } else { self.pan_offset + after - before };
    }

    /// Keep `who` in the middle of the view, or stop doing so: the right-click menu's "Keep
    /// Centered" checkbox for an observer, and the one place the rule lives so that the menu and
    /// the tests cannot disagree about it. Taking hold of an observer lets go of the hole, the two
    /// being exclusive.
    pub(crate) fn hold_observer(
        &mut self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
        who: Who,
        hold: bool,
    ) {
        self.re_anchor(metric, bob, alice, frame_of_ref, hold, |canvas| {
            canvas.centred_on = hold.then_some(who);
            if hold {
                canvas.keep_hole_centred = false;
            }
        });
    }

    /// Keep the hole in the middle of the view, or stop doing so. The hole is always there to be
    /// held, and holding it lets go of anybody else.
    pub(crate) fn hold_hole(
        &mut self,
        metric: &KerrSchild,
        bob: &Option<Observer>,
        alice: &Option<Observer>,
        frame_of_ref: ReferenceFrame,
        hold: bool,
    ) {
        self.re_anchor(metric, bob, alice, frame_of_ref, hold, |canvas| {
            canvas.keep_hole_centred = hold;
            if hold {
                canvas.centred_on = None;
            }
        });
    }

    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        bob: &mut Option<Observer>,
        alice: &mut Option<Observer>,
        current_time: f64,
        signals: SignalViews<'_>,
        canvas_height: f32,
        use_physical_units: bool,
        frame_of_ref: ReferenceFrame,
        font_scale: f32,
        style: FrontStyle,
        show_details: &mut bool,
    ) {
        let desired_size = egui::Vec2::new(ui.available_width(), canvas_height);
        let (response, painter) =
            ui.allocate_painter(desired_size, egui::Sense::click_and_drag());
        let rect = response.rect;

        if rect.width() < 30.0 || rect.height() < 30.0 {
            return;
        }

        // Mouse wheel zoom (refined to 1/8 step size for smooth control, cursor-centered)
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let step = 0.01875;
                let zoom_mult = if scroll > 0.0 { 1.0 + step } else { 1.0 - step };
                let old_zoom = self.zoom;
                let new_zoom = (old_zoom * zoom_mult).clamp(8.0, 500_000.0);
                if let Some(mpos) = response.hover_pos() {
                    let center_nom = rect.center() + self.pan_offset;
                    let cursor_vec = mpos - center_nom;
                    let ratio = new_zoom / old_zoom;
                    self.pan_offset += cursor_vec * (1.0 - ratio);
                }
                self.zoom = new_zoom;
            }
        }
        if self.keep_hole_centred {
            self.pan_offset = Vec2::ZERO;
        }

        // Center of the canvas with frame of reference tracking
        // Every point of the equatorial plane is placed by the Kerr-Schild embedding
        // x + i y = (r + i a) e^{i phi}, never by (r cos psi, r sin psi).
        // Screen y grows downward; Cartesian y grows upward (matching the +Y tick labels), so flip it.
        // In a local rather than read through `self`, so that the closures built on it do not
        // hold a borrow of the canvas for as long as they live: the marker menu below takes
        // `&mut self` while they are still in scope.
        let zoom = self.zoom;
        let zoom_px = f64::from(zoom);
        let to_offset = |(x, y): (f64, f64)| (x * zoom_px, -y * zoom_px);
        // Following an observer who is not in the simulation is following nobody, so the view
        // stays on the hole rather than jumping to a remembered position. A standing request to
        // keep one of them centred is answered first, and the View selector's own
        // tracking is what is left when there is no such request or the observer it names has
        // gone.
        let anchor = self.anchor(metric, bob, alice, frame_of_ref);
        let frame_tracking_offset = anchor.map_or((0.0, 0.0), to_offset);
        // The centre of the geometry is carried in f64, because the zone fills and the boundary
        // lines below are built about it and this view zooms to half a million pixels per M. Out
        // there the centre of the hole stands a hundred million pixels off the canvas, where the
        // f32 of a `Pos2` has eight whole pixels between one value and the next, and the drawn
        // radii of r- and r+ differ by a millionth of themselves. Formed in f32, centre + R(cos,
        // sin) loses that difference to the cancellation of two huge numbers; formed in f64 and
        // narrowed only once the point is back on the canvas, it costs 1e-8 px. See `ZoneArc`.
        let centre = (
            f64::from(rect.center().x) + f64::from(self.pan_offset.x) - frame_tracking_offset.0,
            f64::from(rect.center().y) + f64::from(self.pan_offset.y) - frame_tracking_offset.1,
        );
        // Everything else on this canvas is drawn within a screen's width of the middle, where f32
        // says all there is to say, and this is that same point narrowed once for all of them.
        let center = Pos2::new(centre.0 as f32, centre.1 as f32);

        // Background
        painter.rect_filled(rect, 4.0, Theme::CANVAS_BG);

        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        let re = metric.ergosphere_equatorial();

        // A surface of constant r is the circle of Cartesian radius rho = sqrt(r^2 + a^2);
        // the ring singularity r = 0 is the circle rho = |a|.
        let rho_p = metric.cartesian_radius(rp);
        let rho_m = metric.cartesian_radius(rm);
        let rho_e = metric.cartesian_radius(re);
        let rho_ring = metric.a.abs();

        let r_to_px = |r: f64| -> f64 { r * zoom_px };
        let to_screen =
            |(x, y): (f64, f64)| center + Vec2::new(x as f32 * zoom, -(y as f32) * zoom);
        // Direction vectors (velocities, tangents) need the same y flip as positions.

        // 1. Concentric Zone Fills, every boundary at its Cartesian radius sqrt(r^2 + a^2).
        // Each zone is its own band between two boundaries rather than a disc laid over the discs
        // outside it, so that a region's pixel colour is its fill over the canvas background - the
        // colour the (t, r) diagram paints the same region in - and not its fill stacked on every
        // region it sits inside.
        //
        // The bands and the lines drawn at their edges are cut from one and the same tessellation
        // of the circles: same centre, same angles, same arithmetic, so a fill and the boundary at
        // its edge cannot part company however deep the zoom goes. They used to be cut by two
        // different rules - a fixed 72-gon for a fill, whatever egui chose for a stroke - and a
        // 72-gon's chords fall 9.5e-4 R inside the circle, which is 19 px at a drawn radius of
        // 20 000 px: enough to carry the fill of region II over the r- line and into region III.
        // See `ZoneArc`.
        let ring_px = r_to_px(rho_ring);
        // The boundaries of the picture from the middle outward: the inner edge of the disc inside
        // the ring, then the ring r = 0 at rho = |a|, the Cauchy horizon r-, the outer horizon r+
        // and the static limit. The disc rho < a is the hole of the ring - it is not part of this
        // sheet of the equatorial plane at r > 0 at all, so it gets its own fill rather than a
        // region colour - and a hole with no spin has no ring, so that fill is floored at two
        // pixels of drawn radius and marks the middle of the picture either way.
        let bounds = [0.0, ring_px.max(2.0), r_to_px(rho_m), r_to_px(rho_p), r_to_px(rho_e)];
        let arc = ZoneArc::over(rect, centre, bounds[4]);
        let edges: [Vec<Pos2>; 5] = bounds.map(|radius| arc.ring(arc.held(radius)));

        // Each band in turn, from the ring outward. A band whose two radii are held to the same
        // value has no part of the canvas in it - the whole view lies inside it, or outside it -
        // and is not drawn at all.
        let fills = [
            Theme::SINGULARITY_FILL,
            Theme::REGION_III_FILL,
            Theme::REGION_II_FILL,
            Theme::ERGOSPHERE_FILL,
        ];
        for (band, fill) in fills.into_iter().enumerate() {
            if arc.held(bounds[band]) < arc.held(bounds[band + 1]) {
                painter.add(egui::Shape::mesh(arc.band(&edges[band], &edges[band + 1], fill)));
            }
        }

        // 2. Concentric Boundary Rings, each stroked through the very points the fills either side
        // of it are built from. A boundary the canvas cannot reach is not drawn: out there its
        // radius is held to the rim of the view, which is not where the surface is.
        // Ergosphere boundary
        if arc.shows(bounds[4]) {
            painter.add(arc.outline(&edges[4], Stroke::new(1.5, Theme::ERGOSPHERE_LINE)));
        }

        // Outer Horizon r+
        if arc.shows(bounds[3]) {
            painter.add(arc.outline(&edges[3], Stroke::new(2.5, Theme::HORIZON_OUTER)));
        }

        // Inner Cauchy Horizon r-
        if arc.shows(bounds[2]) {
            painter.add(arc.outline(&edges[2], Stroke::new(2.0, Theme::HORIZON_CAUCHY)));
        }

        // Ring singularity r = 0: the circle of Cartesian radius exactly a, and inside it the
        // arrow that says which way the hole turns.
        let ring_px = ring_px as f32;
        draw_ring_spin_arrow(&painter, center, ring_px, metric.a);
        if arc.shows(bounds[1]) {
            painter.add(arc.outline(&edges[1], Stroke::new(2.0, Theme::SINGULARITY_LINE)));
        }

        // 3. The two transmissions, drawn under the worldlines and the markers,
        // so the fronts read as something moving through the field rather than as part of the
        // observers' own trajectories. Bob's goes down first and Alice's over it, so where the two
        // overlap it is the heavier, primary field that stays legible.
        draw_signal_field(
            &painter,
            metric,
            signals.bob,
            Theme::BOB_COLOR,
            Theme::SECONDARY_FRONT_WIDTH,
            style,
            &to_screen,
        );
        draw_signal_field(
            &painter,
            metric,
            signals.alice,
            Theme::ALICE_COLOR,
            1.0,
            style,
            &to_screen,
        );

        // 5. Both worldline trails, drawn together and before anything that sits on them: the
        // reception ticks below and the observers' own markers.
        if let Some(al) = alice {
            draw_spatial_trail(&painter, metric, al, Theme::ALICE_COLOR, 1.2, &to_screen);
        }
        if let Some(b) = bob {
            draw_spatial_trail(&painter, metric, b, Theme::BOB_COLOR, 1.5, &to_screen);
        }

        // 6b. Every arrival, marked on the *receiver's* trail in the *sender's* colour: amber
        // triangles where Bob received one of Alice's pulses, mint ones where Alice received one
        // of Bob's. Each is drawn at the receiver's own event, (r, phi) at the detection pass,
        // pushed through the same Kerr-Schild embedding as everything else on this canvas, so a
        // tick sits exactly on the trail it belongs to.
        //
        // The pairing is worth stating: the emission dots on a trail say what that observer sent,
        // the triangles on it say what they heard, and the colour of a triangle names who they
        // heard it from. Inside r+ the ticks bunch onto the r- circle, and that is the physics:
        // the frozen family of every pulse waits there, at fixed radius and co-rotating, until the
        // receiver falls through the stack, so a whole run of arrivals happens at one radius in
        // the last fraction of an M of the fall. The (t, r) diagram keeps colouring its reception
        // dots by the measured shift, which is a different question and stays where it can be read.
        let draw_reception_ticks = |field: &SignalField, sender: Color32| {
            for reception in field.receptions() {
                let at = to_screen(metric.cartesian_position(reception.r, reception.phi));
                if rect.contains(at) {
                    draw_reception_tick(&painter, at, sender);
                }
            }
        };
        // Each tick sits on the receiver's trail, so it is drawn only while that receiver is in
        // the simulation: Bob's transmission is received by Alice, and hers by him.
        if alice.is_some() {
            draw_reception_ticks(signals.bob, Theme::BOB_COLOR);
        }
        if bob.is_some() {
            draw_reception_ticks(signals.alice, Theme::ALICE_COLOR);
        }

        // 6. The observers themselves. Alice's info box is registered at the end of the frame,
        // after every other interaction on this canvas, so a drag on it does not pan. Bob's local
        // null cone is not drawn as a fan of stubs any more: he broadcasts the same pulses Alice
        // does, and a whole light cone integrated as exact null geodesics says everything the
        // twenty-four stubs said and keeps saying it as the light travels.

        // A worldline frozen on the far branch of r- has not stopped: it is riding the horizon's
        // own null generator, so its marker goes on creeping round the r- circle at Omega_- while
        // the radius and the observer's own clock stand still. Drawn, that is a dot moving
        // steadily along a circle, which is exactly what an ordinary orbit looks like from here.
        // What tells the two apart is the bold line the observer's own info box carries while
        // they are frozen; see `telemetry_lines`.
        let mut alice_box: Option<Pos2> = None;
        if let Some(al) = alice
            && al.is_active
        {
            let al_pos = to_screen(al.cartesian_position(metric));
            painter.circle_filled(al_pos, Who::Alice.marker_radius(), Theme::ALICE_COLOR);
            alice_box = Some(al_pos);
        }
        let bob_box = bob.as_ref().map(|b| {
            // A plain dot, as Alice's is. He used to wear a white ring as well, which made the
            // ring below - the one that means something - read as a second decoration on a marker
            // that already had one, and gave two observers drawn from the same code two different
            // liveries for no reason. The colour and the radius tell them apart.
            let bob_pos = to_screen(b.cartesian_position(metric));
            painter.circle_filled(bob_pos, Who::Bob.marker_radius(), Theme::BOB_COLOR);
            bob_pos
        });

        // The observer the view is holding on to wears a ring, and now it is the only ring either
        // marker can have, so that a picture which is no longer moving under a falling observer
        // says which one it is following.
        if let Some(centred) = self.centred_on
            && let Some(at) = match centred {
                Who::Alice => alice_box,
                Who::Bob => bob_box,
            }
        {
            let colour =
                if centred == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
            painter.circle_stroke(
                at,
                centred.marker_radius() + CENTRED_RING_GAP,
                Stroke::new(1.0, colour),
            );
        }

        // 6a. The drag, registered after the markers are drawn - their screen positions are what
        // a press is tested against - and before the canvas's own pan below, so that a press on a
        // marker moves the observer instead of the view.
        //
        // The pointer is mapped back through the embedding: a screen point is a Cartesian (x, y) of
        // the drawn plane, and `KerrSchild::chart_point` inverts x + iy = (r + ia)e^{i phi} to the
        // (r, phi) it came from. Inside the ring circle, rho < a, there is no equatorial point at
        // all, so that whole disc of the picture maps to the floor rather than to nothing.
        self.drag_markers(
            metric,
            &response,
            alice.as_mut(),
            bob.as_mut(),
            current_time,
            anchor,
            |obs| to_screen(obs.cartesian_position(metric)),
            |at| {
                let x = f64::from(at.x - center.x) / f64::from(zoom);
                let y = -f64::from(at.y - center.y) / f64::from(zoom);
                metric.chart_point(x, y, RING_DROP_FLOOR)
            },
        );

        // Pan with the mouse, unless a marker has the drag. It takes effect on the next frame,
        // since this frame's projection was fixed above.
        if response.dragged() && self.dragging.is_none() {
            self.pan_offset += response.drag_delta();
        }

        // 6b. The canvas's right-click menu: where the view should look, and what it should go
        // on looking at. It is offered anywhere on the canvas rather than on the markers, because
        // "take me to Bob" is most useful exactly when Bob is not on the screen to be pointed at,
        // and it is offered whether or not the run is moving, since none of it touches the physics.
        // It is registered on the canvas's own response, after the markers are drawn and before the
        // telemetry boxes, which take their drags last.
        //
        // Going to somebody and keeping them centred are different requests and both are here: the
        // first moves the view once and lets them walk out of it again, the second re-answers
        // itself every frame. A Goto while somebody is being followed pans away from them without
        // letting go, since the pan is measured from whatever the view is tracking.
        response.context_menu(|ui| {
            for (label, target) in [
                ("Goto Bob", bob.as_ref().map(|o| o.cartesian_position(metric))),
                ("Goto Alice", alice.as_ref().map(|o| o.cartesian_position(metric))),
                // The hole is at the origin of the embedding x + iy = (r + ia)e^{i phi}, which is
                // the centre of the ring rather than a point of the spacetime.
                ("Goto Black Hole", Some((0.0, 0.0))),
            ] {
                if ui.add_enabled(target.is_some(), egui::Button::new(label)).clicked() {
                    if let Some(at) = target {
                        self.look_at(metric, bob, alice, frame_of_ref, at);
                    }
                    ui.close();
                }
            }
            ui.separator();
            for who in [Who::Bob, Who::Alice] {
                let present = match who {
                    Who::Alice => alice.is_some(),
                    Who::Bob => bob.is_some(),
                };
                let mut centred = self.centred_on == Some(who);
                let label = format!("Keep {} Centered", who.name());
                if ui.add_enabled(present, egui::Checkbox::new(&mut centred, label)).changed() {
                    self.hold_observer(metric, bob, alice, frame_of_ref, who, centred);
                    ui.close();
                }
            }
            let mut hole = self.keep_hole_centred;
            if ui.checkbox(&mut hole, "Keep Black Hole Centered").changed() {
                self.hold_hole(metric, bob, alice, frame_of_ref, hole);
                ui.close();
            }
        });

        // 7. Title and Legend Overlay
        // Horizon angular velocity Ω_H = a / (2 M r₊) is a rate per unit coordinate time, so in
        // geometric units it is a number per M; only dividing by t_g = GM/c³ makes it rad/s.
        let omega_h = metric.a / (2.0 * metric.m * rp);
        // The cut, stated on the canvas as well as in the panel's tip: a gap in a drawn front has
        // to say for itself that it is deliberate.
        let wound_line = if style.hide_wound {
            "Segments wound past a full turn are not drawn: the front there straddles\n\
             a photon orbit and two rays cannot resolve it\n"
        } else {
            ""
        };
        // What the view is holding on to, and how to ask it to hold on to somebody: a picture
        // that has stopped moving under a falling observer should say why, and the menu that did
        // it is not discoverable by looking at the canvas.
        let centred_line = match (self.keep_hole_centred, self.centred_on) {
            (true, _) => {
                "Keeping the black hole centered (right-click anywhere to change)\n".to_string()
            }
            (false, Some(who)) => {
                format!("Keeping {} centered (right-click anywhere to change)\n", who.name())
            }
            (false, None) => {
                "Right-click anywhere: go to Bob, Alice or the hole, or keep one centered\n"
                    .to_string()
            }
        };
        let legend_text = if use_physical_units {
            format!("θ = π/2, x + iy = (r + ia) e^{{iϕ}}\n\
                 Cartesian radius ρ = √(r²+a²); ring singularity at ρ = a\n\
                 Units: Kilometers (km) & Seconds (s)\n\
                 Scale: 1M = {}\n\
                 Mass: {} M☉\n\
                 Outer Horizon r₊: {} ({}M, ρ = {}M)\n\
                 Cauchy Horizon r₋: {} ({}M, ρ = {}M)\n\
                 Spin a/M: {}\n\
                 Drag: Ω_H = {}/M = {}\n\
                 Front colour: ν an infaller here measures ÷ ν the infaller passing the emitter\n\
                 measured as it left: red ×1 (every front is born red), orange ×3, yellow ×10,\n\
                 green ×30, blue ×1000, violet ×100000, grey below ×1. One lightness throughout,\n\
                 so the colour carries the shift and nothing else\n\
                 Arcs piled on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 {}\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour\n\
                 (amber = Alice → Bob, mint = Bob → Alice)\n\
                 {}\
                 Zoom: {} px/M (scroll to zoom, drag the background to pan,\n\
                 drag either observer's marker to put them anywhere in the plane)", metric.format_physical_distance(1.0), numbers::fixed(metric.m_solar, 2), metric.format_km(metric.r_to_km(rp)), numbers::fixed(rp, 2), numbers::fixed(rho_p, 2), metric.format_km(metric.r_to_km(rm)), numbers::fixed(rm, 2), numbers::fixed(rho_m, 2), numbers::fixed(metric.a_star(), 3), numbers::fixed(omega_h, 3), numbers::rad_per_second(omega_h / metric.t_grav_seconds()), wound_line, centred_line, numbers::fixed(self.zoom, 0))
        } else {
            format!("θ = π/2, x + iy = (r + ia) e^{{iϕ}}\n\
                 Cartesian radius ρ = √(r²+a²); ring singularity at ρ = a\n\
                 Physical Scale: 1M = GM/c² = {}\n\
                 Time Scale:     1M/c = GM/c³ = {}\n\
                 Mass: {} M☉\n\
                 Outer Horizon r₊: {}M ({}), ρ = {}M\n\
                 Cauchy Horizon r₋: {}M ({}), ρ = {}M\n\
                 Spin a/M: {}\n\
                 Drag: Ω_H = {}/M\n\
                 Front colour: ν an infaller here measures ÷ ν the infaller passing the emitter\n\
                 measured as it left: red ×1 (every front is born red), orange ×3, yellow ×10,\n\
                 green ×30, blue ×1000, violet ×100000, grey below ×1. One lightness throughout,\n\
                 so the colour carries the shift and nothing else\n\
                 Arcs piled on r₋: the frozen family (E − Ω₋L < 0, never crosses this branch)\n\
                 {}\
                 Bob's fronts: same gain colours at half stroke, mint emission dots\n\
                 Receptions: triangle on the receiver's trail in the sender's colour\n\
                 (amber = Alice → Bob, mint = Bob → Alice)\n\
                 {}\
                 Zoom: {} px/M (scroll to zoom, drag the background to pan,\n\
                 drag either observer's marker to put them anywhere in the plane)", metric.format_physical_distance(1.0), metric.format_physical_time(1.0), numbers::fixed(metric.m_solar, 2), numbers::fixed(rp, 2), metric.format_physical_distance(rp), numbers::fixed(rho_p, 2), numbers::fixed(rm, 2), metric.format_physical_distance(rm), numbers::fixed(rho_m, 2), numbers::fixed(metric.a_star(), 3), numbers::fixed(omega_h, 3), wound_line, centred_line, numbers::fixed(self.zoom, 0))
        };

        // The distance marker: a ruler along the bottom edge, in the units in force and at this
        // view's own scale. It measures the plane the way the plane is drawn, which is Cartesian
        // distance in the Kerr-Schild embedding - the Details block says how that differs from r -
        // so it needs no caption to say what it measures. It belongs to the canvas rather than to
        // the hole, so it stays put under any pan and is never off the view. The drawing itself is
        // `gui::ruler`, shared with both modes of the (t, r) canvas.
        ruler::draw_distance_ruler(
            &painter, rect, zoom_px, use_physical_units, metric, font_scale, None,
        );

        // The legend is an info box like any other on this canvas, shut to its title until the
        // triangle opens it. Whether it stands open is the panel's `show_details`, which is what a
        // save file has always carried, so the box is told that state before it is drawn and the
        // state is read back from it afterwards: one fact, held in one place.
        self.telemetry.set_shut(Canvas::Spatial, BoxId::Legend, !*show_details);

        // 8. Draggable info boxes, registered last so they take the drag instead of the canvas.
        // They go through the queue every canvas uses, so that an untouched one is slid clear of
        // the other observer's.
        let mut pending = vec![PendingBox::legend(
            rect.left_top() + Vec2::new(8.0, 6.0),
            "Equatorial View",
            &legend_text,
            "What this view draws and how to read it: the embedding, the scale, the horizon radii, \
             the spin and the drag rate, the colour keys of the fronts, and the mouse controls.",
        )];
        if let (Some(al), Some(al_pos)) = (alice.as_ref(), alice_box) {
            pending.push(PendingBox::observer(
                &painter, rect, Who::Alice, al_pos, "Alice", Theme::ALICE_COLOR, al, metric,
                use_physical_units, font_scale, Vec::new(),
            ));
        }
        if let (Some(b), Some(bob_pos)) = (bob.as_ref(), bob_box) {
            pending.push(PendingBox::observer(
                &painter, rect, Who::Bob, bob_pos, "Bob", Theme::BOB_COLOR, b, metric,
                use_physical_units, font_scale, Vec::new(),
            ));
        }
        self.telemetry.flush(ui, &painter, Canvas::Spatial, rect, &pending, font_scale);
        *show_details = !self.telemetry.is_shut(Canvas::Spatial, BoxId::Legend);
    }
}

/// The fewest segments a whole turn of a zone boundary is cut into, however small the circle.
///
/// Seventy-two, as it has always been here and as the volume cuts its rings. The sagitta rule
/// below asks for fewer than this at any radius under 262 px, and a circle of ten pixels should
/// still read as a circle rather than as the fourteen-sided figure the tolerance alone would
/// allow: at that size the error is not what the eye is objecting to.
const ZONE_SEGMENTS: usize = 72;

/// How far inside the true circle the chord of one drawn segment may fall, in screen pixels.
///
/// A regular n-gon on a circle of on-screen radius R misses it by the sagitta R(1 - cos(pi/n)),
/// which is R pi^2 / (2 n^2) to the accuracy that matters, so a *fixed* n is a fixed fraction of R
/// and grows without bound as the view zooms in. Seventy-two segments cut the chords 9.5e-4 R
/// inside the circle: a quarter of a pixel at R = 260 px, 19 px at R = 20 000 px, and this view
/// reaches drawn radii of 1e6 px and beyond. Holding the error in pixels instead inverts that to
/// n = pi sqrt(R / (2 eps)), and a quarter of a pixel is below anything an anti-aliased stroke can
/// show.
const ZONE_SAGITTA_PX: f64 = 0.25;

/// The fewest segments any stretch of a boundary is cut into, however short the stretch.
///
/// At a drawn radius of a million pixels the canvas sees less than a milliradian of the circle and
/// the tolerance is met by a single chord, which is true and looks like nothing: four segments
/// cost nothing and leave the ends of the arc and the margins around it comfortable.
const ZONE_ARC_MIN: usize = 4;

/// Most segments one stretch of a boundary is cut into.
///
/// This is a guarantee rather than a working limit. The count the tolerance asks for is bounded
/// twice over - by the radius, through the square root, and by the stretch of the circle the
/// canvas can see, which shrinks as 1/R once the centre is off the view - so what it actually
/// asks for peaks at a few hundred. On a 900 px canvas
/// `test_a_zone_boundary_is_cut_to_a_quarter_pixel_from_ten_pixels_to_a_billion` measures 72 for
/// the whole turn of a small circle, 40 for the widest arc there is at 316 px, 27 at 1000 px, 7 at
/// 10 000 px, and the floor of four from 30 000 px to a billion. A 4K canvas with the centre just
/// off one corner is the worst case there is, at about 300. The cap is here so that no zoom, no
/// pan and no window size can turn a frame into unbounded work.
const ZONE_SEGMENTS_MAX: usize = 2048;

/// How far outside the canvas the zone geometry is carried, in screen pixels.
const ZONE_MARGIN_PX: f64 = 4.0;

/// What one frame of this view can see of the family of circles about the hole: which angles of
/// them reach the canvas, which radii reach it, and how finely they have to be cut.
///
/// Every zone boundary of the equatorial view is a circle about one centre - a surface of constant
/// r is drawn at the Cartesian radius sqrt(r^2 + a^2), and the ring r = 0 at rho = |a| - so one of
/// these answers for the whole picture. Answering it once is what lets a band and the boundary at
/// its edge be built from a single list of points, which is the only way the two can be made
/// incapable of disagreeing.
///
/// The second thing it does is keep the work bounded. At a drawn radius of a million pixels the
/// canvas sees a fraction of a milliradian of the circle, and tessellating the whole turn to a
/// quarter of a pixel would ask for four thousand segments to draw four of them. Only the stretch
/// that can reach the canvas is cut.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ZoneArc {
    /// The centre of the family, in screen pixels and in f64. See the comment on `centre` in
    /// `render`: at these zooms it is millions of pixels off the canvas.
    centre: (f64, f64),
    /// The first and the last angle of the stretch that can reach the canvas, measured in screen
    /// coordinates - x right, y down - and never folded into a range, so that a stretch straddling
    /// the seam at +-pi is one interval and not two and `to` is always the greater of the pair.
    from: f64,
    to: f64,
    /// Whether the whole turn shows, which is what makes an outline a loop rather than a line and
    /// a band's last segment join back to its first.
    closed: bool,
    /// The distance from the centre to the nearest and to the farthest point of the canvas. A
    /// circle smaller than `near`, or larger than `far`, misses the canvas altogether; one between
    /// them crosses it, and is a boundary there is something to draw of.
    near: f64,
    far: f64,
    /// How many segments the stretch from `from` to `to` is cut into.
    steps: usize,
}

impl ZoneArc {
    /// The stretch of the circles about `centre` that `rect` can show, cut fine enough for the
    /// circle of on-screen radius `outermost` - the largest of the family that will be drawn.
    fn over(rect: egui::Rect, centre: (f64, f64), outermost: f64) -> Self {
        use std::f64::consts::{PI, TAU};
        // Everything is measured against the canvas grown by a few pixels, so that a fill runs
        // under the rim of the view rather than up to it and no rounding of an angle or a radius
        // can leave a hairline of background along an edge or at a corner. The margin is
        // comfortably more than the quarter pixel a chord may cut in by plus half of the widest
        // stroke. The spill costs nothing: `Ui::allocate_painter` hands back a painter clipped to
        // the rect it allocated, so what goes past the rim is never rasterised.
        let probe = rect.expand(ZONE_MARGIN_PX as f32);
        let (left, right) = (f64::from(probe.left()), f64::from(probe.right()));
        let (top, bottom) = (f64::from(probe.top()), f64::from(probe.bottom()));
        let leg = |x: f64, y: f64| (x - centre.0, y - centre.1);
        let corners = [(left, top), (right, top), (right, bottom), (left, bottom)];
        let far = corners.iter().fold(0.0_f64, |widest, &(x, y)| {
            let (dx, dy) = leg(x, y);
            widest.max(dx.hypot(dy))
        });
        // The nearest point of the canvas is the centre's own position held to it, which is the
        // centre itself - and a distance of zero - whenever the centre is on the canvas.
        let (nx, ny) = leg(centre.0.clamp(left, right), centre.1.clamp(top, bottom));
        let near = nx.hypot(ny);
        let (from, to, closed) = if near == 0.0 {
            (0.0, TAU, true)
        } else {
            // A convex figure seen from outside subtends less than half a turn, so the smallest
            // stretch of angles holding the canvas is the one its four corners span. Each corner
            // is measured against the direction of the canvas's own middle, which lies inside
            // that stretch and so is less than half a turn from every one of them: folding those
            // differences into [-pi, pi) cannot be the fold that carries a corner to the wrong end
            // of the stretch, because the seam sits behind the viewer and this arithmetic never
            // reaches it.
            let (bx, by) = leg(f64::from(probe.center().x), f64::from(probe.center().y));
            let base = by.atan2(bx);
            let (mut first, mut last) = (0.0_f64, 0.0_f64);
            for (x, y) in corners {
                let (dx, dy) = leg(x, y);
                let turned = (dy.atan2(dx) - base + PI).rem_euclid(TAU) - PI;
                first = first.min(turned);
                last = last.max(turned);
            }
            (base + first, base + last, false)
        };
        // Segments enough that no chord falls further than `ZONE_SAGITTA_PX` inside the circle it
        // stands for: n = pi sqrt(R / (2 eps)) over a whole turn, and that share of it over a
        // shorter stretch. The radius it is cut for is the largest that will be drawn, held to
        // what the canvas can show, so that a circle far outside the view does not buy segments
        // nobody could see. Every smaller circle of the family then comes out finer than the
        // tolerance asks rather than coarser, which is why one cut serves the whole picture.
        let share = (to - from) / TAU;
        let reference = outermost.clamp(near, far);
        let by_error = (PI * (reference / (2.0 * ZONE_SAGITTA_PX)).sqrt() * share).ceil();
        let by_floor = ((ZONE_SEGMENTS as f64) * share).ceil();
        let steps = (by_error.max(by_floor) as usize).clamp(ZONE_ARC_MIN, ZONE_SEGMENTS_MAX);
        Self { centre, from, to, closed, near, far, steps }
    }

    /// The points of the circle of on-screen radius `radius`, over the visible stretch and in
    /// order. A closed stretch leaves out the repeat of its first point, as a loop does.
    ///
    /// Each point is formed in f64 about a centre carried in f64, and only the result is narrowed
    /// to the f32 of a `Pos2`. That result is always on or near the canvas - `held` keeps the
    /// radius between the nearest and the farthest point of the view, and the stretch of angles
    /// covers no more than the view subtends - so the narrowing costs about 1e-4 px, while the
    /// same arithmetic carried out in f32 about a centre 1e8 px away would cost eight.
    fn ring(&self, radius: f64) -> Vec<Pos2> {
        let count = if self.closed { self.steps } else { self.steps + 1 };
        let step = (self.to - self.from) / (self.steps as f64);
        (0..count)
            .map(|i| {
                let (sin, cos) = (self.from + step * (i as f64)).sin_cos();
                Pos2::new(
                    (self.centre.0 + radius * cos) as f32,
                    (self.centre.1 + radius * sin) as f32,
                )
            })
            .collect()
    }

    /// Whether the circle of this radius crosses the canvas, which is whether there is a boundary
    /// on the screen to stroke.
    fn shows(&self, radius: f64) -> bool {
        radius >= self.near && radius <= self.far
    }

    /// This radius held to the stretch of radii the canvas can show.
    ///
    /// A fill is a region of the plane and not a circle, so a band whose far edge is ten million
    /// pixels out is still the band that has to cover this canvas. Holding both of its radii to
    /// [near, far] leaves the covered part of the view exactly as it was - every point outside
    /// that range is off the canvas - while keeping every vertex where f32 can carry it. Two radii
    /// held to the same value mean the band has no part of the view in it.
    fn held(&self, radius: f64) -> f64 {
        radius.clamp(self.near, self.far)
    }

    /// The filled band between two circles of this family, as one mesh.
    ///
    /// The zones are painted as disjoint bands rather than as nested discs so that a region's
    /// pixel colour is its own fill over the canvas background - the colour the (t, r) diagram
    /// paints the same region in, where the regions are side-by-side strips - rather than its fill
    /// stacked on the fill of every region outside it. Region II is then the same deep purple, and
    /// region III the same dark sea green, in every view.
    ///
    /// The two edges are given as points rather than as radii because the whole point of them is
    /// that they are shared: the outer edge of one band is the inner edge of the next and is the
    /// list of points its boundary is stroked through, all three the same `Vec`.
    fn band(&self, inner: &[Pos2], outer: &[Pos2], fill: Color32) -> egui::Mesh {
        let mut mesh = egui::Mesh::default();
        for (&inside, &outside) in inner.iter().zip(outer) {
            mesh.colored_vertex(inside, fill);
            mesh.colored_vertex(outside, fill);
        }
        let count = inner.len().min(outer.len()) as u32;
        let steps = if self.closed { count } else { count.saturating_sub(1) };
        for i in 0..steps {
            let j = (i + 1) % count;
            let (a_in, a_out, b_in, b_out) = (2 * i, 2 * i + 1, 2 * j, 2 * j + 1);
            mesh.add_triangle(a_in, a_out, b_out);
            mesh.add_triangle(a_in, b_out, b_in);
        }
        mesh
    }

    /// One boundary, stroked through the points a band's edge is built from: a loop when the whole
    /// turn shows and an open line when the canvas sees only an arc of it.
    ///
    /// This is what `circle_stroke` could not do. egui cuts a circle by a rule of its own, at most
    /// 128 segments of a whole turn however large the radius, which at R = 20 000 px stands 6 px
    /// clear of where a 72-gon fill ended - so the fill of a region crossed the line at its edge
    /// and ran into its neighbour. Through one list of points there is no rule to disagree with.
    fn outline(&self, points: &[Pos2], stroke: Stroke) -> egui::Shape {
        if self.closed {
            egui::Shape::closed_line(points.to_vec(), stroke)
        } else {
            egui::Shape::line(points.to_vec(), stroke)
        }
    }
}

/// Which observer a canvas is anchored to, given a standing request to keep one centred and the
/// View selector: the rule both the equatorial view and the volume view follow.
///
/// Following an observer who is not in the simulation is following nobody, so the caller is handed
/// None and the view stays on the hole rather than on a remembered position. A standing request is
/// answered first, and the selector's own tracking is what is left when there is no such request or
/// the observer it names has gone. Where the two disagree the standing request wins, being the more
/// particular of the two: it can keep Bob in the middle of a view drawn in the global foliation,
/// which the selector cannot say.
pub(crate) fn frame_focus<'a>(
    centred: Option<Who>,
    frame: ReferenceFrame,
    bob: Option<&'a Observer>,
    alice: Option<&'a Observer>,
) -> Option<&'a Observer> {
    let observer = |who: Who| match who {
        Who::Alice => alice,
        Who::Bob => bob,
    };
    centred.and_then(observer).or(match frame {
        ReferenceFrame::Bob => bob,
        ReferenceFrame::Alice => alice,
        // Neither chart of the global foliation is drawn for anybody, so neither names a focus
        // observer of its own: what is left is whatever standing request there is, or nobody.
        ReferenceFrame::DistantObserver | ReferenceFrame::GlobalVolume => None,
    })
}

/// Three quarters of a turn of arrow inside the ring, pointing the way the hole rotates.
///
/// The disc rho < a is not part of this sheet of the equatorial plane at all - it is the hole of
/// the ring, which a worldline reaching r = 0 off the ring passes through into the r < 0 sheet -
/// so there is nothing to draw in there that would compete with it, and the one fact worth putting
/// in that space is the sense of the spin. Everything else on this canvas shows the rotation only
/// through what it does to something else: the ergosphere's bulge, the winding of a front, the
/// prograde arc that freezes on r_-. The arrow says it outright.
///
/// The direction is read off the sign of the spin parameter and from nothing else. Prograde is
/// increasing phi, and the embedding x + iy = (r + ia)e^{i phi} maps that to a counter-clockwise
/// turn in the drawn plane, so the arc is swept with phi and the y flip of `to_screen` is applied
/// here in the same way, by negating the sine. A hole with a = 0 has no ring and no sense of
/// rotation to draw, and one drawn too small to hold an arrowhead gets nothing rather than a blob.
fn draw_ring_spin_arrow(painter: &egui::Painter, center: Pos2, ring_px: f32, spin: f64) {
    const SWEEP: f64 = 1.5 * std::f64::consts::PI;
    const START: f64 = -0.75 * std::f64::consts::PI;
    const STEPS: usize = 96;
    if ring_px < RING_ARROW_MIN_PX || spin == 0.0 {
        return;
    }
    let radius = 0.60 * ring_px;
    let width = (0.10 * ring_px).clamp(1.5, 4.0);
    let sense = spin.signum();
    let at = |theta: f64| -> Pos2 {
        center + Vec2::new((radius as f64 * theta.cos()) as f32, -(radius as f64 * theta.sin()) as f32)
    };
    let points: Vec<Pos2> = (0..=STEPS)
        .map(|i| at(START + sense * SWEEP * (i as f64) / (STEPS as f64)))
        .collect();
    let end_theta = START + sense * SWEEP;
    let tip_end = *points.last().expect("the arc has STEPS + 1 points");
    painter.add(egui::Shape::line(points, Stroke::new(width, Theme::SINGULARITY_SPIN)));

    // The arrowhead, on the tangent at the end of the sweep: d/dtheta of the drawn point is
    // (-sin theta, -cos theta) once the y flip is in, and the sense of travel multiplies it.
    let tangent = Vec2::new(-(end_theta.sin()) as f32, -(end_theta.cos()) as f32) * sense as f32;
    let normal = Vec2::new(-tangent.y, tangent.x);
    let head = (0.34 * ring_px).clamp(4.0, 14.0);
    painter.add(egui::Shape::convex_polygon(
        vec![
            tip_end + tangent * head,
            tip_end - tangent * head * 0.35 + normal * head * 0.45,
            tip_end - tangent * head * 0.35 - normal * head * 0.45,
        ],
        Theme::SINGULARITY_SPIN,
        Stroke::NONE,
    ));
}

/// Draw every live wavefront of one transmission in the equatorial embedding.
///
/// The same code draws Alice's field and Bob's, because it is the same physics either way. What
/// tells them apart on screen is `emission_colour`, the colour of the dot marking each emission
/// event on its emitter's trail, and `width_scale`, which thins the secondary field's strokes (see
/// `Theme::SECONDARY_FRONT_WIDTH`). The gain colouring of the fronts themselves is not available
/// as an identifying mark: it is a measurement, and it has to mean the same thing in both fields.
///
/// The emitter broadcasts into their whole light cone, so each pulse is a *closed* polyline through
/// the Kerr-Schild positions of its surviving rays, ordered by emission angle and with the last ray
/// joined back to the first. Each segment is coloured by the *gain* its light has picked up since it
/// was let go:
///
///     gain = nu(a raindrop at the ray's current event) / nu(the raindrop passing the emitter as it left),
///
/// the two `f_factor` evaluations of `NullRay::gain_between`, with the ray's conserved energy and
/// affine scale cancelling out of the quotient. Both observers are drops of the E = 1, L = 0
/// congruence - free fall from rest at infinity - which is the one family of observers that exists
/// at every radius, inside both horizons included, so the colour means the same thing across the
/// whole picture rather than being quoted against a frame that stops existing at r+. It is a real
/// measured shift, the ordinary gravitational-plus-Doppler one between two members of that
/// congruence along the ray, and nothing about the emitter's own motion enters it.
///
/// That last property is why the front is coloured by this and not by the emitter-relative ratio the
/// receptions and the HUD quote. Held against the emitter, the rays of one pulse are already spread
/// across the whole ramp at the instant they leave - the prograde half aberrated blue, the retrograde
/// half red - so a fresh front is born split in two, which says something true about the emission but
/// nothing at all about where the light has since been. Held between raindrops, every ray of a pulse
/// starts at gain exactly 1, because at the emission event the two f_factors are the same number
/// computed twice: a new front comes out one uniform red and then earns its way up the ramp as
/// it falls, and what the colour then shows is what the light has gained on its way here. The gain
/// is carried *along* each segment rather than averaged over it: log10(gain) is interpolated
/// linearly between the two rays in the same loop coordinate the position is interpolated in, and
/// the polyline is cut into bands of at most `FRONT_BAND_DECADES` each, every band drawn at the
/// colour of its own midpoint (see `banded_segment`). Where the front is being torn apart - one ray
/// freezing onto r- while its neighbour crosses - a single segment spans the whole ramp, and one
/// mean colour said the far end had gained a hundred thousandfold when it had gained nothing.
///
/// Segments with a dead endpoint are skipped: a ray that has reached the ring is gone, and the front
/// genuinely ends there rather than jumping across the gap. Segments wound past
/// `MAX_RESOLVED_WINDING` are skipped too, when `FrontStyle::hide_wound` is set, because two samples
/// that far apart no longer bound a resolved piece of front; each of their two rays is then drawn as
/// its own dot instead, so the cut reads as a gap with marked ends rather than as a silent hole.
///
/// Every pulse also gets a dot at its own emission event, in the emitter's own colour. Without it
/// the nested loops inside r+ read as circles drawn around the hole, which is the wrong picture:
/// each loop is one pulse and encloses its emitter, because light is isotropic in the emitter's own
/// frame, and the flow then carries the whole loop inward. The dot sits on the emitter's trail at
/// the radius the pulse left them at, and the loop's outer edge never gets further from the hole
/// than that dot, which is the statement the drawing exists to make.
///
/// One arc of each pulse is the frozen family: the rays whose `NullRay::inner_horizon_energy` is
/// negative, E - Omega_- L < 0, which approach r- as r - r- ~ exp(-kappa_- t) while co-rotating at
/// Omega_-. Within a few M of coordinate time that arc has collapsed onto the magenta r- circle,
/// and its gain climbs the colour ramp all the way to the 1e5 the ramp runs out to, which is what
/// the top of the ramp is there for. Nothing about it is drawn specially: it takes the same stroke,
/// the same `Theme::SHIFT_ALPHA`, the same gain colouring and the same trail as every other piece
/// of front, and no ray of it carries a bead. It used to be drawn heavily over a second pass, and
/// the sign of E - Omega_- L is fixed from the ray's birth, so that whole arc of every front
/// changed weight the instant it crossed r+ and broke the fade there - a seam at r+ that said
/// nothing about the light, which goes through r+ without noticing it.
///
/// Behind every drawn piece of front lies its trail: a strip in that piece's own colour at the
/// line and transparent at its far edge, laid on the side the light has come *from*. A front on a
/// still frame is a closed curve, and a closed curve says nothing about which way it is moving; the
/// fade says it, so an expanding ring reads as expanding without the run being played. All the
/// trails of one field go into one mesh - `Trails` gathers it - which takes its place on the
/// painter before the first line of the first pulse goes down, so the fades lie under every front
/// of the field and a line stays exactly as crisp as it was.
///
/// How deep the strip is at a ray is the least of three lengths. `Theme::FRONT_TRAIL_PX` is the
/// ceiling, a screen length at every zoom. `Theme::FRONT_TRAIL_GAP_FRACTION` of the gap back to the
/// front behind it is what makes the cue readable at all: a transmission sends every 0.1 M of the
/// emitter's proper time, so at the default 48 px/M its fronts stand about 5 px apart, and a fixed
/// 30 px fade on every one of them lay six deep and turned the whole field into one flat wash with
/// only the leading front legible. Bounded by the gap, no two fades of a field can overlap at any
/// zoom or any spacing: crowded, each line keeps a soft trailing edge a few pixels deep - sharp
/// ahead, fading behind, which is the whole of the cue - and on a deep zoom, or with the pulse
/// count turned down, the full 20 px opens out.
///
/// The third length is how far this ray's light has come since the pulse was let go, and it is
/// what holds a fresh front to where its light has actually been. A pulse emitted a moment ago is
/// a loop a few pixels across hugging its emitter; give each ray the whole ceiling and the strip
/// reaches back *through* the emission event and fans out behind the emitter as a flare, drawn over
/// ground the light has never covered. At zero time in flight the fade is zero and it opens out of
/// the emission event with the front, so the fade stops at the emitter rather than surrounding
/// them. No fraction is applied to this one, unlike the gap: the emission event is where the light
/// really started, and a fade that stopped short of it would be claiming otherwise.
///
/// Both of those lengths are estimated rather than measured: the ray's own screen speed *now*,
/// times the coordinate time between the two emissions in the one case and the time this ray has
/// been in flight - `NullRay::t` less `Pulse::emitted_t` - in the other. Neither integrates the
/// speed along the path the light took, and the emitter's motion between two emissions does not
/// enter. That is the right standard for both, because the number bounds a decoration - nothing in
/// the picture is measured in the length of a fade, and a fade drawn a pixel too short or too long
/// says nothing false about the light. The time in flight is used rather than the straight screen
/// distance from the drawn vertex back to the emission point, which would look like the tighter
/// statement and is not: inside the ergosphere the hole drags a ray round and carries it past its
/// own emission point again long after it left, where the straight distance is small and the
/// journey is not, and the fade would collapse exactly there. The time in flight only grows.
///
/// All three are per ray rather than per front because the rays of one pulse move at wildly
/// different screen speeds, so the fade narrows by itself exactly where that pulse and the one
/// behind it have converged.
///
/// What "behind" means at a point of a front is decided by the *ray's* coordinate velocity there,
/// (dr/dt, dphi/dt) off the integrated `NullRay`, and not by the normal of the drawn polyline. The
/// two agree for a ring expanding evenly and part company exactly where the front is interesting -
/// sheared, folded, or being torn between the frozen family and the crossing one - and only one of
/// them is a quantity the simulation holds: the velocity is integrated, the normal of an
/// interpolated polyline is an artefact of where the sampling happened to put its points. So the
/// direction drawn is the direction the light at that point is travelling, which is also the honest
/// answer where a front is folded over itself and two pieces of it are going opposite ways through
/// the same pixel. `screen_velocity` pushes that velocity through the Jacobian
/// `KerrSchild::cartesian_velocity` and then through `to_screen` itself by a finite difference in
/// its f64 input, which is what makes the cue correct on the 2D+1 volume's tilted floor as well as
/// on the flat equatorial view: both projections are affine, so the difference is the exact screen
/// direction rather than an approximation to it, and a projection that was not affine would still
/// get the tangent it has at that point. Along a segment the two end rays' screen directions are
/// interpolated in the same loop coordinate s that carries the position and the gain, so a band's
/// strip uses the sub-range of s that band covers.
///
/// Nothing is drawn where there is no direction to draw: a ray whose screen velocity vanishes - on
/// the volume's floor seen edge-on, or where the interpolation runs between two ends travelling
/// opposite ways - gets its tail vertex on top of its front vertex, so the strip closes to nothing
/// there instead of jumping to a made-up heading. A segment that draws no line has no trail either,
/// which covers the dead endpoints, the wound segments `hide_wound` cuts, and the whole points-only
/// mode. On r- the frozen family's light glides *along* the front at Omega_-, so its strip lies on
/// the front itself and covers almost no area at all - the fade there says, correctly, that the
/// light is going nowhere across the front it belongs to.
/// Largest azimuthal span of one drawn piece of a wavefront segment, in radians.
///
/// A segment of the front is the piece of null surface between two neighbouring rays, and what it
/// looks like in the equatorial plane is decided by the two rays' (r, phi), not by the straight
/// line between their screen positions. The two differ by nothing worth drawing while the rays are
/// close together in azimuth, and by the whole picture where they are not - which is the deep
/// interior. Inside r- the annulus that Region III occupies is thin (at a = 0.90 the embedding puts
/// r- at rho = 1.06 and the ring at rho = 0.90), and the rays there wind at wildly different rates:
/// dphi/dt reaches about -5 per M for a ray near the ring against +0.8 for one settling onto r-.
/// Neighbouring rays are then most of a radian apart, and the chord between them cuts straight
/// across the annulus and through the disk inside the ring, which drew as spikes into the
/// singularity that no ray ever took.
///
/// So each segment is drawn as the curve linear in (r, phi) between its two ends, over the *raw*
/// azimuth difference `to.1 - from.1`, cut into pieces no wider than this before each is embedded.
/// That is the same interpolation along the same segment that `Pulse::scan` uses to decide where
/// the front crosses a receiver, so what is drawn and what is detected are one thing. Two frozen
/// rays sitting on r- are now joined by an arc of the r- circle rather than by a chord dipping
/// inside it.
const MAX_ARC_STEP: f64 = 0.05;

/// Most pieces one segment of a front may be cut into, however far apart in azimuth its two ends
/// have wound.
///
/// Nothing in the physics bounds that separation. A ray let go on very nearly the critical impact
/// parameter hangs on one of the unstable circular photon orbits outside r+ (at a = 0.90 the
/// equatorial ones are at r = 1.56 prograde and r = 3.91 retrograde) for as long as it takes to
/// fall off them, going round and round while the neighbour it was emitted next to escapes or
/// falls in; the difference between the two grows for as long as that lasts, and all of it is
/// front. `test_a_wound_front_of_a_real_pulse_is_drawn_over_its_raw_azimuth_difference` measures
/// 3.7 turns in one segment of one ordinary pulse within 60 M, and the number has no ceiling in
/// it. The cap is therefore not a physical claim and not a cost measurement; it is the guarantee
/// that no state of the field can turn one segment of one pulse into an unbounded amount of work
/// in a frame that has to be drawn now. It sits far above anything a run has been seen to reach -
/// 2000 pieces at `MAX_ARC_STEP` is 100 radians, sixteen turns of the hole - and a segment that
/// did hit it would still be drawn over the whole of its span, in pieces coarser than
/// `MAX_ARC_STEP`, rather than truncated.
const MAX_ARC_PIECES: usize = 2000;

/// The azimuthal separation past which a neighbouring pair of rays no longer bounds a *resolved*
/// piece of front: one whole turn of the hole.
///
/// A segment joins two rays let go 2.5 degrees apart at the default sampling, and everything drawn
/// between them is the interpolation linear in (r, phi) of `segment_arc`. That is a faithful
/// picture of the front while the pair stays together, and it stops being one the moment the pair
/// straddles a critical impact parameter. At a = 0.90 the prograde equatorial photon orbit sits at
/// r_ph = 1.56, just outside r+ = 1.44: a ray let go marginally inside the critical angle spirals
/// in and freezes on r-, its neighbour marginally outside it hangs on r_ph for tens of M and then
/// escapes, and the real front between them is *pinned on that orbit* - a spiral in from the far
/// ray to r_ph, a pile-up of turns at r_ph that no sampling of the light cone can resolve, and a
/// spiral from r_ph down to r-. Two rays cannot carry that shape. What the interpolation draws
/// instead is an Archimedean spiral with its winding spread evenly over every radius between the
/// two ends, r+ included, and because the outer ray runs away at nearly c while the winding grows
/// only at Omega_-, those turns drift steadily outward across the outer horizon as the run goes on.
/// No ray does that, and nothing else in the picture does either: it is the one place where the arc
/// between two neighbours is not a statement about the front but an artefact of joining two samples
/// that no longer belong to one another.
///
/// A full turn is where the claim becomes indefensible rather than where the error becomes large.
/// Past 2 pi the two rays have gone round the hole relative to one another at least once, so the
/// segment covers every azimuth and the sampling has no information at all about what the front
/// does inside it: the drawn curve is then chosen entirely by the interpolation. Below it the arc
/// still misplaces the winding, but it is one arc between two neighbours on the same sheet, and it
/// is the same curve `Pulse::scan` tests. Only the two segments straddling the prograde and the
/// retrograde critical angle get there - about two per pulse - so dropping them removes those and
/// nothing else. Reception is not one of the things it removes: `Pulse::scan` interpolates along
/// every segment whether or not it is drawn, so an arrival happens at the same event either way.
const MAX_RESOLVED_WINDING: f64 = std::f64::consts::TAU;

/// The drawn polyline of one segment of a front: the curve linear in (r, phi) from one ray to the
/// next, embedded point by point. See `MAX_ARC_STEP`.
///
/// The azimuth difference is taken raw and is never folded into [-pi, pi], and that is exact rather
/// than a choice. Every ray of a pulse leaves the emission event at the emitter's own azimuth and
/// carries phi as a continuously integrated coordinate that is never reduced mod 2 pi (see
/// `NullRay::phi`), so the difference between two neighbouring rays is a continuous function of
/// time starting at zero: the integrated phi already *is* the unwrapped coordinate, and the raw
/// difference already is the physical winding between the pair. Folding agrees with it only while
/// |d_phi| < pi, which is exactly what a front that has wound around the hole violates: past half
/// a turn the fold flips the sign, and the arc that ran the long way round is redrawn the short
/// way, through the near side of the picture - spokes popping into existence across the drawing as
/// the fronts wind. Unfolded, a segment several turns long is drawn as several turns of arc, which
/// is what that piece of the front is. Frame dragging inside r+ pulls neighbouring rays apart by
/// most of a radian, which is what `MAX_ARC_STEP` is for; what takes a pair past half a turn is a
/// ray hung on a circular photon orbit outside r+, which is what `MAX_ARC_PIECES` is for.
fn segment_arc<F: Fn((f64, f64)) -> Pos2>(
    metric: &KerrSchild,
    from: (f64, f64),
    to: (f64, f64),
    to_screen: &F,
) -> Vec<Pos2> {
    let d_phi = to.1 - from.1;
    let pieces = (d_phi.abs() / MAX_ARC_STEP).ceil().max(1.0).min(MAX_ARC_PIECES as f64) as usize;
    (0..=pieces)
        .map(|k| {
            let s = (k as f64) / (pieces as f64);
            let r = from.0 + s * (to.0 - from.0);
            let phi = from.1 + s * d_phi;
            to_screen(metric.cartesian_position(r, phi))
        })
        .collect()
}

/// The radius, times the field's stroke scale, of the dot each calculated point of a front is
/// drawn as: every live ray when the arcs are switched off, and the two ends of a segment the
/// winding cut has withdrawn.
///
/// The Arcs between wavefront points checkbox is a drawing choice and only a drawing choice. On,
/// each segment of a front between two neighbouring rays is drawn as the curve of `segment_arc`,
/// the same interpolation in (r, phi) that `Pulse::scan` uses to test the front against a
/// receiver, so the drawn front is the curve the detector is testing. Off, nothing is drawn between
/// the rays at all: the front is shown as the calculated points themselves, one dot per live ray,
/// which is the raw output of the integrator with no interpolation of any kind laid over it. That
/// is worth being able to see, because everything an arc adds is inference - a segment between two
/// rays most of a radian apart in the deep interior is drawn along a curve no ray was integrated
/// on - and the dots are the part that is not. Nothing about a reception moves when the box is
/// unticked; `Pulse::scan` interpolates along the same segments either way.
///
/// The same dot marks each end of a segment dropped for winding past `MAX_RESOLVED_WINDING`, in
/// that ray's own gain colour and at the ordinary `Theme::SHIFT_ALPHA`: those two calculated points
/// are still calculated points, and it is only the inference between them that has been withdrawn.
/// Every dot on a front is that one dot: in points-only mode a ray already carrying it is not drawn
/// a second time for ending a cut segment.
const FRONT_POINT_RADIUS: f32 = 1.6;

/// Below this drawn radius the ring is a dot on the screen and the spin arrow is not drawn: it
/// would be a smear over the ring's own stroke rather than an arrow. A hole of a = 0.90 at the
/// default zoom is well above it; zooming out far enough takes the arrow away and leaves the ring.
const RING_ARROW_MIN_PX: f32 = 14.0;

/// How far outside their own marker the ring around a centred observer is drawn.
pub(crate) const CENTRED_RING_GAP: f32 = 5.0;

/// How much of the gain ramp one flat-coloured band of a segment may cover, in decades of gain.
///
/// A segment is drawn in the colour of the gain its light carries, and its two rays need not carry
/// anything like the same gain. Where the front is being torn apart - one ray settling onto r- and
/// climbing like exp(kappa_- t) up the ramp while its neighbour crosses and is gone - a single
/// segment runs from gain 1 to gain 1e5: five decades, the whole ramp from red to violet.
/// Painted in one colour that segment is violet along its entire length, which says the light at
/// the far end has gained a hundred thousandfold when it has gained nothing at all, and the ramp
/// then reads as a jump at a ray rather than as the climb along the front that it is.
///
/// So the gain is carried along the segment: log10(gain) is interpolated linearly in the same loop
/// coordinate s that `segment_arc` interpolates the position in, and the polyline is cut into bands
/// of at most this many decades, each drawn at the colour of its own midpoint. A quarter of a
/// decade is under half the narrowest leg of `Theme::FRONT_STOPS` - the half-decade from red
/// to orange - so no band can straddle a stop of the ramp unnoticed, and the five-decade case costs
/// twenty polylines where it used to cost one. The interpolation is in log10 because that is the
/// coordinate the ramp itself is keyed to, so a band is a fixed slice of the drawn ramp rather than
/// a fixed slice of a quantity spanning five orders of magnitude.
const FRONT_BAND_DECADES: f64 = 0.25;

/// Most bands one segment may be cut into: the same kind of guarantee `MAX_ARC_PIECES` makes about
/// the pieces.
///
/// The clamp `Theme::front_colour` puts on log10(gain) already holds any segment to the six decades
/// of the ramp, so twenty-four bands is the most the physics can ask for and this cap is not
/// reached in any state the field can be in. It is here so that no later widening of the ramp, and
/// no unphysical gain that finds its way past the clamps, can turn one segment of one pulse into
/// unbounded work in a frame that has to be drawn now.
const MAX_FRONT_BANDS: usize = 64;

/// log10 of a gain, clamped to the two ends of the wavefront ramp exactly as `Theme::front_colour`
/// clamps it, so that interpolating between two of these and colouring the result agrees with
/// colouring the two ends directly. A non-positive or non-finite gain, which no real measurement
/// produces, sits at the dark end.
fn front_log(gain: f64) -> f64 {
    if gain.is_finite() && gain > 0.0 {
        gain.log10().clamp(Theme::FRONT_LOG_MIN, Theme::FRONT_LOG_MAX)
    } else {
        Theme::FRONT_LOG_MIN
    }
}

/// Cut one drawn segment into bands of nearly constant gain: the polyline of `segment_arc` split at
/// shared boundary points, each piece paired with the gain to colour it by. See
/// `FRONT_BAND_DECADES`.
///
/// `gain_from` belongs to the ray at s = 0 and `gain_to` to the ray at s = 1, the same loop
/// coordinate `segment_arc` walks the position along, so band b covers the s-range from b/bands to
/// (b + 1)/bands of that same curve and is coloured at the gain interpolated to its midpoint. The
/// interpolation is linear in log10(gain), clamped as the ramp clamps it.
///
/// Consecutive bands share their boundary point rather than abutting, so the drawn front has no
/// gaps in it: the concatenation of the bands, each shared point counted once, is the original
/// polyline in order. A segment whose two ends carry the same gain is one band over the whole arc,
/// which is the common case and costs nothing over drawing it directly. Asking for more bands than
/// the arc has pieces would need boundary points that are not on the polyline, so the count falls
/// back to the piece count instead: the colour of a two-point segment is then quantised more
/// coarsely than `FRONT_BAND_DECADES` asks for, which is all two points can carry anyway.
fn banded_segment(arc: Vec<Pos2>, gain_from: f64, gain_to: f64) -> Vec<(Vec<Pos2>, f64)> {
    let (log_from, log_to) = (front_log(gain_from), front_log(gain_to));
    let gain_at = |s: f64| 10.0_f64.powf(log_from + s * (log_to - log_from));
    let pieces = arc.len().saturating_sub(1);
    let wanted = ((log_to - log_from).abs() / FRONT_BAND_DECADES).ceil().max(1.0) as usize;
    let bands = wanted.clamp(1, MAX_FRONT_BANDS).min(pieces.max(1));
    if bands <= 1 || pieces == 0 {
        return vec![(arc, gain_at(0.5))];
    }
    (0..bands)
        .map(|b| {
            // Integer bounds, so band b ends exactly where band b + 1 begins and the last ends on
            // the final point of the arc: the bands tile the segment with no gap and no overlap
            // beyond the single point each consecutive pair shares.
            let start = b * pieces / bands;
            let end = (b + 1) * pieces / bands;
            let s_mid = 0.5 * ((start + end) as f64) / (pieces as f64);
            (arc[start..=end].to_vec(), gain_at(s_mid))
        })
        .collect()
}

/// How far along the light's direction the screen probe of `screen_velocity` steps, in M, and how
/// much it grows by when the step it took was too short to read a direction off.
///
/// Both projections a front is drawn through are affine in the Cartesian chart point - the
/// equatorial view scales and flips it, `Camera::project` takes two dot products of it - so the
/// difference quotient is the exact screen direction at any step size at all, and the size is
/// chosen for arithmetic rather than for accuracy. It has to be large enough that the difference of
/// two f32 screen positions is not mostly rounding, and the zoom it is multiplied by spans five
/// orders of magnitude: 1e-2 M is 0.5 px at the default 48 px/M and 2000 px at the 200 000 px/M a
/// deep zoom onto r- reaches. So the probe starts small and grows by a hundred until the step it
/// produces is at least `TRAIL_PROBE_MIN_PX`, which two growths carry down to a zoom of 1e-4 px/M -
/// far below anything the canvas offers.
const TRAIL_PROBE_EPS: f64 = 1e-2;
const TRAIL_PROBE_GROWTH: f64 = 100.0;
const TRAIL_PROBE_TRIES: usize = 3;

/// The shortest screen step `screen_velocity` will take a direction from. A whole pixel of
/// separation leaves the difference of two f32 positions well clear of their last few bits, even
/// with the picture panned a hundred thousand pixels off the canvas.
const TRAIL_PROBE_MIN_PX: f32 = 1.0;

/// Below this length an interpolated heading is treated as no heading at all: the two ends of the
/// segment are travelling opposite ways and there is no "behind" between them. Nothing physical is
/// near it - every heading being interpolated is a unit vector or exactly zero.
const TRAIL_MIN_HEADING: f32 = 1e-3;

/// The shallowest fade worth laying. A band whose trail is under a pixel deep at both of its ends
/// puts nothing on the screen a viewer could read a direction off, so that band is left out
/// entirely. It is most of a crowded field near the hole, where the gap to the following front is a
/// fraction of a pixel, and it is where the vertices are saved.
const TRAIL_MIN_PX: f32 = 1.0;

/// Most vertices one trail mesh may carry before the next band starts a new one.
///
/// `epaint::Mesh` indexes in u32 and nothing here could approach that, but `Mesh::split_to_u16`
/// still cuts every mesh up for the backends that take 16-bit indices, and it panics outright on a
/// single triangle whose vertices span more than 65 535 of them. Holding a mesh below that leaves
/// the split trivially satisfiable however the bands fall. The default field reaches about 37 000
/// vertices - 64 pulses of 144 segments, four vertices each - so the cap is hit only by the deep
/// interior, where one wound segment is cut into hundreds of pieces, and by the 256-pulse setting.
const MAX_TRAIL_MESH_VERTICES: usize = 65_532;

/// The screen velocity of the light at one ray - which way it is going and how fast, in pixels per
/// M of coordinate time - or `Vec2::ZERO` where the projection leaves it no direction at all.
///
/// The ray carries dr/dt and dphi/dt at its own event. `KerrSchild::cartesian_velocity` is the
/// exact Jacobian of `cartesian_position`, so it turns that pair into the chart-plane velocity
/// d(x, y)/dt - which is not the polar (dr/dt, r dphi/dt) rotated by the polar angle, because the
/// embedding x + iy = (r + ia)e^{i phi} carries the spin's offset. The screen velocity is then read
/// off `to_screen` itself rather than assumed: one step of `TRAIL_PROBE_EPS` along the chart
/// velocity, projected, minus the ray's own projected position, divided by that step and multiplied
/// by the chart speed. That is exact for an affine projection and first-order for any other, and it
/// is the reason the same code draws the cue correctly on the volume's tilted floor.
///
/// The speed is what bounds the fade. The front behind this one was let go an emission interval
/// ago, so the light at this ray has covered its own screen speed times that interval since, and
/// that is the gap the fade is allowed to occupy - see `Theme::FRONT_TRAIL_GAP_FRACTION`. The
/// direction and the speed come out of the one probe because they are the one vector.
///
/// A ray that is not moving on screen has no direction to give. That is not only the ray at rest:
/// the volume's floor seen edge-on projects every chart velocity onto nothing, and a frozen ray on
/// r- has dr/dt -> 0 but goes on co-rotating at Omega_-, so it moves tangentially and does have a
/// heading. Zero is returned for the first case and never invented for it.
///
/// `eps` carries the step that last worked from ray to ray. The zoom cannot change within a frame,
/// so once one ray has found a step long enough to read a direction off, every other ray of the
/// field starts from that step instead of walking up to it again. It matters because a projection
/// is not always cheap: `Camera::project` strikes the camera's basis with two sin_cos calls every
/// time it is asked, so the difference between one probe per ray and three is measurable on the
/// 2D+1 volume's floor.
fn screen_velocity<F: Fn((f64, f64)) -> Pos2>(
    metric: &KerrSchild,
    ray: &NullRay,
    at: Pos2,
    to_screen: &F,
    eps: &mut f64,
) -> Vec2 {
    let (x, y) = metric.cartesian_position(ray.r, ray.phi);
    let (vx, vy) = metric.cartesian_velocity(ray.r, ray.phi, ray.dr_dt, ray.dphi_dt);
    let speed = vx.hypot(vy);
    if !speed.is_finite() || speed <= 0.0 {
        return Vec2::ZERO;
    }
    let (ux, uy) = (vx / speed, vy / speed);
    for _ in 0..TRAIL_PROBE_TRIES {
        let step = to_screen((x + *eps * ux, y + *eps * uy)) - at;
        if step.is_finite() && step.length() >= TRAIL_PROBE_MIN_PX {
            // The step is what one `eps` of chart length along the light's direction is worth on
            // screen, and the light covers `speed` of that chart length per M of coordinate time.
            let velocity = step * ((speed / *eps) as f32);
            return if velocity.is_finite() { velocity } else { Vec2::ZERO };
        }
        *eps *= TRAIL_PROBE_GROWTH;
    }
    // Nothing at any step: the step is put back where it started so that a ray whose own velocity
    // is what vanished cannot leave the whole field probing at a step a hundred thousand M long.
    *eps = TRAIL_PROBE_EPS;
    Vec2::ZERO
}

/// The fade behind one ray: which way the light there is going on screen, and how far back the fade
/// may reach at that ray.
///
/// The length is the least of three: `Theme::FRONT_TRAIL_PX`, `Theme::FRONT_TRAIL_GAP_FRACTION` of
/// the estimated gap to the front behind this one, and the distance this ray's light has covered
/// since it was let go. It is per ray rather than per front because all three are: the rays of one
/// pulse run at wildly different screen speeds - one settling onto r- has almost none, its
/// neighbour crossing has all of it - so the fade narrows exactly where the fronts crowd and opens
/// out where they do not.
#[derive(Clone, Copy)]
struct Trail {
    /// Unit screen direction of travel, or zero where the projection gives none.
    heading: Vec2,
    /// How far behind the front the fade reaches here, in screen pixels.
    length: f32,
}

impl Trail {
    /// No fade at all: a dead ray, or a live one the projection leaves standing still on screen.
    const NONE: Self = Self { heading: Vec2::ZERO, length: 0.0 };

    /// The fade at one live ray, from the screen velocity of its light, `gap`, the coordinate time
    /// back to the front behind this one - None where this front has no neighbour to crowd it,
    /// which is a field carrying a single pulse - and `elapsed`, the coordinate time this ray has
    /// been in flight since its pulse was let go.
    ///
    /// The same screen speed serves both bounds, so both are estimates in the same sense: the speed
    /// the light has here and now, rather than the speed integrated along the path it took. A fade
    /// is a decoration and nothing in the picture is measured in the length of one.
    fn of(velocity: Vec2, gap: Option<f32>, elapsed: f32) -> Self {
        let speed = velocity.length();
        if speed <= 0.0 || !speed.is_finite() {
            return Self::NONE;
        }
        // The distance covered since emission is the bound that holds a fresh pulse to where its
        // light has been: at emission it is zero, and the fade grows out of the emission event with
        // the front rather than reaching back through it.
        let mut length = Theme::FRONT_TRAIL_PX.min(speed * elapsed.max(0.0));
        if let Some(gap) = gap {
            length = length.min(Theme::FRONT_TRAIL_GAP_FRACTION * speed * gap);
        }
        Self { heading: velocity / speed, length }
    }
}

/// The trailing fades of one field, gathered as the bands of its fronts are laid. See the doc above
/// `MAX_ARC_STEP` for what the fade is and why its direction is the ray velocity.
struct Trails {
    /// One mesh while a field's fade fits in one, and a new one past `MAX_TRAIL_MESH_VERTICES`.
    meshes: Vec<egui::Mesh>,
    /// Vertices a fresh mesh takes room for: a field's worth, so that the ordinary field is one
    /// allocation rather than a doubling run that copies more than a megabyte of vertices for
    /// nothing. See where `Trails::new` is called for where the estimate comes from.
    reserve: usize,
}

impl Trails {
    fn new(reserve: usize) -> Self {
        Self { meshes: Vec::new(), reserve }
    }

    /// Lay the fade behind one drawn band of a front.
    ///
    /// The band is points `first ..= first + band.len() - 1` of a segment cut into `pieces` pieces,
    /// so point k of it sits at s = (first + k) / pieces of the segment, which is the coordinate
    /// `segment_arc` interpolated the position in and `banded_segment` cut the colour in. The fade
    /// at that point is the same interpolation of the two end rays' `Trail`s, `from` at s = 0 and
    /// `to` at s = 1: the heading is interpolated and put back on the unit circle, the length is
    /// interpolated as it stands, and the tail vertex is the front vertex pushed that length along
    /// the reverse of that heading. Both are interpolated because both are per ray - the two ends
    /// of one segment can be running at very different screen speeds, so their fades can be very
    /// different depths, and a band between them has to pass from the one to the other.
    ///
    /// A band whose fade is under `TRAIL_MIN_PX` at both ends is not laid at all.
    ///
    /// The tail vertex is `Color32::TRANSPARENT` rather than `colour` at alpha 0. egui carries
    /// vertex colours premultiplied and interpolates them as they are, and the premultiplication of
    /// any colour at alpha 0 is (0, 0, 0, 0): interpolating towards it walks the straight line from
    /// (R a, G a, B a, a) to the origin, which unmultiplies to one constant hue at a falling alpha.
    /// Interpolating towards the same RGB at alpha 0 - (R, G, B, 0) unpremultiplied - would instead
    /// walk through colours brighter than the line itself and put a fringe along the whole fade.
    fn band(
        &mut self,
        band: &[Pos2],
        first: usize,
        pieces: usize,
        (from, to): (Trail, Trail),
        colour: Color32,
    ) {
        if band.len() < 2 || pieces == 0 || from.length.max(to.length) < TRAIL_MIN_PX {
            return;
        }
        let full = |m: &egui::Mesh| m.vertices.len() + 2 * band.len() > MAX_TRAIL_MESH_VERTICES;
        if self.meshes.last().is_none_or(full) {
            let mut mesh = egui::Mesh::default();
            mesh.reserve_vertices(self.reserve);
            mesh.reserve_triangles(self.reserve);
            self.meshes.push(mesh);
        }
        let mesh = self.meshes.last_mut().expect("a mesh was pushed above if there was none");
        // Each end's heading is already a unit vector or exactly zero, so the two ends of the
        // segment need no interpolating and no normalising. At the default sampling a band *is* the
        // one piece between two neighbouring rays, so both of its points take that path and the
        // general case below is paid for only where the arc had to be cut up.
        let (at_start, at_end) = (-from.heading * from.length, -to.heading * to.length);
        for (k, at) in band.iter().enumerate() {
            let step = first + k;
            // No heading, no trail at this vertex: the tail sits on the front and the strip closes
            // to nothing there, which is a fade that has run out rather than a NaN or a guess.
            let offset = if step == 0 {
                at_start
            } else if step == pieces {
                at_end
            } else {
                let s = (step as f64 / pieces as f64) as f32;
                let heading = from.heading * (1.0 - s) + to.heading * s;
                if heading.length() > TRAIL_MIN_HEADING {
                    -heading.normalized() * (from.length * (1.0 - s) + to.length * s)
                } else {
                    Vec2::ZERO
                }
            };
            let base = mesh.vertices.len() as u32;
            mesh.colored_vertex(*at, colour);
            mesh.colored_vertex(*at + offset, Color32::TRANSPARENT);
            if k > 0 {
                // The quad behind piece k - 1: front and tail of the last point, then of this one.
                mesh.add_triangle(base - 2, base - 1, base + 1);
                mesh.add_triangle(base - 2, base + 1, base);
            }
        }
    }

    /// Everything laid, as the one shape that goes into the slot taken before the field's first
    /// line, or None when no band of the field cast a fade at all.
    fn shape(self) -> Option<egui::Shape> {
        (!self.meshes.is_empty())
            .then(|| egui::Shape::Vec(self.meshes.into_iter().map(egui::Shape::mesh).collect()))
    }
}

pub(crate) fn draw_signal_field<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    signal: &SignalField,
    emission_colour: Color32,
    width_scale: f32,
    style: FrontStyle,
    to_screen: &F,
) {
    // `derivatives` reads only (E, L) off the state and takes the radius as an argument, so one
    // instance of the raindrop congruence serves every ray of every pulse.
    let raindrop = GeodesicState::new_infall(metric, 0.0, 12.0, 1.0, 0.0);
    // The emitter's colour, faint: present enough to read as a mark on their trail, quiet enough
    // not to compete with the wavefront it anchors.
    let dot = Color32::from_rgba_unmultiplied(
        emission_colour.r(),
        emission_colour.g(),
        emission_colour.b(),
        150,
    );
    // The trailing fades of the whole field, and the place on the painter they will be put. The
    // slot is taken before any pulse is drawn, so every fade lies under every line of this field
    // however late the pulse that cast it is reached. With the arcs off there are no lines to
    // annotate and no slot is taken at all.
    let trail_slot = style.arcs.then(|| painter.add(egui::Shape::Noop));
    // The probe step `screen_velocity` reads a screen velocity over, carried across every ray of
    // every pulse of the field: see that function.
    let mut probe_eps = TRAIL_PROBE_EPS;
    // How many vertices the mesh is expected to want. Each of a pulse's n rays begins one segment,
    // a segment whose two rays have not wound far apart is one band of the two ray positions, and
    // a band of m points carries 2m vertices: four vertices per ray, which is 36 864 for the
    // default field of 64 pulses of 144 rays and 37 336 measured on a real one. The deep interior
    // cuts a segment into hundreds of pieces and runs past the estimate, and the mesh then grows as
    // any vector does; the estimate is here so that the ordinary field is one allocation rather
    // than a doubling run through a megabyte of vertices.
    let mut trails = Trails::new(if style.arcs {
        (4 * signal.pulses.iter().map(|pulse| pulse.rays.len()).sum::<usize>())
            .min(MAX_TRAIL_MESH_VERTICES)
    } else {
        0
    });
    for (k, pulse) in signal.pulses.iter().enumerate() {
        // A spent pulse is kept in the field so that stepping backwards can bring it back, but it
        // has no front left to draw and no dot to anchor.
        if !pulse.rays.iter().any(|ray| ray.alive()) {
            continue;
        }
        let n = pulse.rays.len();
        if n < 2 {
            continue;
        }
        // The drop that was passing the emitter as this pulse left, which is the denominator of
        // every ray's gain: one per pulse, because every ray of a pulse left the same event.
        let u_emit = {
            let (ut, ur, up) = raindrop.derivatives(metric, pulse.emitted_r);
            [ut, ur, up]
        };
        let gains: Vec<f64> = pulse
            .rays
            .iter()
            .map(|ray| {
                if !ray.alive() {
                    return 1.0;
                }
                let (ut, ur, up) = raindrop.derivatives(metric, ray.r);
                ray.gain_between(metric, pulse.emitted_r, &u_emit, &[ut, ur, up])
            })
            .collect();
        // One projection per ray per frame, which the segment loop would otherwise repeat for each
        // of the two segments a ray belongs to. The ray positions themselves, which the dots of a
        // withdrawn segment sit on; the segments between them are drawn as arcs in (r, phi) between
        // them, or not at all when the arcs are off.
        let points: Vec<Pos2> = pulse
            .rays
            .iter()
            .map(|ray| to_screen(metric.cartesian_position(ray.r, ray.phi)))
            .collect();
        // How long this pulse has been in flight ahead of the front behind it, which is what bounds
        // every fade on it. `SignalField::pulses` is a deque of live pulses in emission order -
        // `emit_if_due` pushes to the back, the cap pops the front, and `step_back` retains - so
        // the neighbour at k + 1 is the next pulse this emitter sent. The newest pulse has no
        // follower yet and takes the interval to the one before it, since a transmission sends at
        // one cadence; a field carrying a single pulse has no interval at all and its fade is held
        // only by `Theme::FRONT_TRAIL_PX`.
        let behind = k.checked_sub(1).and_then(|p| signal.pulses.get(p));
        let gap = match (signal.pulses.get(k + 1), behind) {
            (Some(next), _) => Some(next.emitted_t - pulse.emitted_t),
            (None, Some(previous)) => Some(pulse.emitted_t - previous.emitted_t),
            (None, None) => None,
        }
        .filter(|interval| interval.is_finite() && *interval > 0.0)
        .map(|interval| interval as f32);
        // The fade at each ray: which way the light there is going on screen, and how far back the
        // gap and the flight so far let it reach. One projection probe per live ray per frame, kept
        // here for the same reason the positions are - each ray is an end of two segments and would
        // otherwise be probed twice. Nothing to annotate with the arcs off, so nothing is computed
        // there either. `NullRay::t` is the coordinate time the ray itself stands at, so the time
        // in flight is that clock less the pulse's own emission time, and it is zero on the frame a
        // pulse is emitted.
        let fades: Vec<Trail> = if style.arcs {
            pulse
                .rays
                .iter()
                .zip(points.iter())
                .map(|(ray, at)| {
                    if !ray.alive() {
                        return Trail::NONE;
                    }
                    let elapsed = (ray.t - pulse.emitted_t) as f32;
                    Trail::of(
                        screen_velocity(metric, ray, *at, to_screen, &mut probe_eps),
                        gap,
                        elapsed,
                    )
                })
                .collect()
        } else {
            Vec::new()
        };

        // The two ends of every segment dropped for winding: each is a live calculated point whose
        // segment has been withdrawn, and each is drawn as its own dot below so that the cut reads
        // as a gap with marked ends rather than as a silent hole in the front.
        let mut cut_end = vec![false; n];
        // n segments rather than n - 1: the closing one runs from the last ray back to the first.
        // With the arcs off there are no segments at all, only the points below.
        for i in (0..n).filter(|_| style.arcs) {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            // The raw, unfolded winding between the pair, which is the span `segment_arc` would
            // draw this segment over. Past a whole turn the two samples no longer bound a resolved
            // piece of front and the curve between them is the interpolation's own invention, so
            // nothing is drawn there and the two ends are marked instead. See
            // `MAX_RESOLVED_WINDING`.
            if style.hide_wound
                && (pulse.rays[j].phi - pulse.rays[i].phi).abs() > MAX_RESOLVED_WINDING
            {
                cut_end[i] = true;
                cut_end[j] = true;
                continue;
            }
            let arc = segment_arc(
                metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                to_screen,
            );
            // The gain is carried along the segment rather than averaged over it: one polyline per
            // band of at most `FRONT_BAND_DECADES`, and a single band over the whole arc in the
            // common case where the two rays carry the same gain.
            //
            // How many pieces the bands are cut from, and how far into them each band starts:
            // consecutive bands share their boundary point, so a band of m points advances the
            // start by m - 1. That pair is s of the segment, which the trail interpolates the two
            // rays' headings in exactly as the colour is interpolated in it.
            let pieces = arc.len().saturating_sub(1);
            let mut first = 0usize;
            for (band, gain) in banded_segment(arc, gains[i], gains[j]) {
                let advance = band.len().saturating_sub(1);
                let colour = Theme::front_colour(gain, Theme::SHIFT_ALPHA);
                // The fade first, into the mesh that is painted under every line of this field. Its
                // head is this band's own colour, the very colour and opacity the line beside it is
                // stroked in, and it falls from there to nothing.
                trails.band(&band, first, pieces, (fades[i], fades[j]), colour);
                painter.add(egui::Shape::line(band, Stroke::new(1.2 * width_scale, colour)));
                first += advance;
            }
        }
        for (i, point) in points.iter().enumerate() {
            if !pulse.rays[i].alive() {
                continue;
            }
            if !style.arcs || cut_end[i] {
                // The calculated point itself: with the arcs on it is implied by the two segments
                // meeting there, with them off it is all there is of this ray, and at the end of a
                // segment dropped for winding it is what is left once the inference is withdrawn.
                painter.circle_filled(
                    *point,
                    FRONT_POINT_RADIUS * width_scale,
                    Theme::front_colour(gains[i], Theme::SHIFT_ALPHA),
                );
            }
        }
        // The anchor: where on Alice's trail this loop was let go of.
        let emitted = to_screen(metric.cartesian_position(pulse.emitted_r, pulse.emitted_phi));
        painter.circle_filled(emitted, 2.0, dot);
    }

    // Every fade of the field into the slot taken before the first line went down, so the whole
    // haze sits under the whole of the field it belongs to. One `Shape::Vec` because the mesh is
    // split when it grows past what a 16-bit index can reach; at the default settings it is one
    // mesh of about 37 000 vertices, against the 9 216 line shapes it lies under.
    if let (Some(slot), Some(shape)) = (trail_slot, trails.shape()) {
        painter.set(slot, shape);
    }
}

/// One arrival, as a mark on the receiver's trail: a small filled triangle, apex up, in the
/// colour of whoever sent the pulse.
///
/// The shape is what separates it from the emission dots already on that trail, and the colour is
/// what says which transmission it belongs to; a hairline white outline keeps it legible where it
/// lands on top of a shift-coloured front, which near r- is most of the time. The orientation is
/// fixed rather than aligned with anything: an arrival has a direction on the sky, but not one
/// this projection could draw honestly.
const RECEPTION_TICK_RADIUS: f32 = 4.0;

pub(crate) fn draw_reception_tick(painter: &egui::Painter, at: Pos2, colour: Color32) {
    let (h, w) = (RECEPTION_TICK_RADIUS, RECEPTION_TICK_RADIUS * 0.866);
    painter.add(egui::Shape::convex_polygon(
        vec![
            Pos2::new(at.x, at.y - h),
            Pos2::new(at.x + w, at.y + h * 0.5),
            Pos2::new(at.x - w, at.y + h * 0.5),
        ],
        colour,
        Stroke::new(0.5, Color32::WHITE),
    ));
}

/// Faint spatial trajectory of an observer: the recorded (t, r, phi) trail pushed through the
/// Kerr-Schild embedding x + i y = (r + i a) e^{i phi}. With E = 1, L = 0 the curve spirals in and,
/// for a spinning hole, terminates on the ring rho = a rather than at the origin.
pub(crate) fn draw_spatial_trail<F: Fn((f64, f64)) -> Pos2>(
    painter: &egui::Painter,
    metric: &KerrSchild,
    obs: &Observer,
    color: Color32,
    width: f32,
    to_screen: &F,
) {
    if obs.trail.len() < 2 {
        return;
    }
    // Thinned to the screen: the trail holds one event per stepped frame, so on a long run most
    // consecutive events embed to the same pixel and cost a triangle each. See `thin_to_pixels`.
    let points = thin_to_pixels(
        obs.trail.iter().map(|p| to_screen(metric.cartesian_position(p.r, p.phi))),
        |at| *at,
        SCREEN_SPACING,
    );
    let faint = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 120);
    painter.add(egui::Shape::line(points, Stroke::new(width, faint)));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The embedded radius of a chart point: |x + i y| = |(r + i a) e^{i phi}| = sqrt(r^2 + a^2),
    /// so a curve of constant r is a circle in the drawing and this is its radius.
    fn embedded_radius(point: Pos2) -> f64 {
        ((point.x as f64).powi(2) + (point.y as f64).powi(2)).sqrt()
    }

    /// The arc of the spin arrow as `draw_ring_spin_arrow` puts it on a painter: the polyline in
    /// `Theme::SINGULARITY_SPIN`, in the order it is drawn, taken relative to the ring's centre.
    fn spin_arrow_arc(spin: f64, ring_px: f32) -> Vec<(f64, f64)> {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let centre = Pos2::new(200.0, 200.0);
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            draw_ring_spin_arrow(&painter, centre, ring_px, spin);
        });
        let mut arc = Vec::new();
        fn walk(shape: &egui::Shape, centre: Pos2, arc: &mut Vec<(f64, f64)>) {
            match shape {
                egui::Shape::Path(path) => {
                    let solid = matches!(
                        path.stroke.color,
                        egui::epaint::ColorMode::Solid(c) if c == Theme::SINGULARITY_SPIN
                    );
                    if solid {
                        arc.extend(path.points.iter().map(|p| {
                            ((p.x - centre.x) as f64, (p.y - centre.y) as f64)
                        }));
                    }
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, centre, arc);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, centre, &mut arc);
        }
        output.drop_without_applying_deltas();
        arc
    }

    #[test]
    fn test_the_ring_carries_an_arrow_three_quarters_round_in_the_sense_of_the_spin() {
        // What the arrow inside the ring has to say, measured off the painter rather than off the
        // helper's arithmetic. It has to be *inside* the ring, since the disc rho < a is the space
        // it is drawn in and an arc spilling over the ring's own circle would read as something
        // crossing it. It has to go three quarters of the way round, because that is what says
        // "turning" rather than "a mark at an angle". And it has to turn the way the hole turns:
        // prograde is increasing phi, which the embedding x + iy = (r + ia)e^{i phi} draws
        // counter-clockwise, so on a screen whose y runs downward consecutive points must cross
        // *negatively*, and a hole spun the other way must reverse every one of those crossings.
        // A ring too small to hold an arrowhead gets no arrow at all.
        let ring_px = 60.0_f32;
        for &(spin, name, want_sign) in
            &[(0.90_f64, "prograde", -1.0_f64), (-0.90, "retrograde", 1.0)]
        {
            let arc = spin_arrow_arc(spin, ring_px);
            assert!(arc.len() > 32, "{name}: the arc is a polyline, got {} points", arc.len());
            let mut swept = 0.0;
            for pair in arc.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                let cross = a.0 * b.1 - a.1 * b.0;
                let dot = a.0 * b.0 + a.1 * b.1;
                assert!(
                    cross * want_sign > 0.0,
                    "{name}: the arc must turn one way only, got a crossing of {cross}"
                );
                swept += cross.atan2(dot).abs();
            }
            let turn = swept / std::f64::consts::TAU;
            assert!(
                (turn - 0.75).abs() < 1e-6,
                "{name}: the arrow sweeps {turn} of a turn, and must sweep three quarters"
            );
            let radii: Vec<f64> = arc.iter().map(|(x, y)| (x * x + y * y).sqrt()).collect();
            let widest = radii.iter().fold(0.0_f64, |m, r| m.max(*r));
            assert!(
                widest < ring_px as f64,
                "{name}: the arc reaches {widest} px, outside the ring at {ring_px}"
            );
            println!("{name} spin: {} points, {turn:.3} of a turn at up to {widest:.1} px inside a ring of {ring_px} px", arc.len());
        }
        assert!(
            spin_arrow_arc(0.90, RING_ARROW_MIN_PX - 0.1).is_empty(),
            "a ring too small for an arrowhead is left alone"
        );
        assert!(spin_arrow_arc(0.0, ring_px).is_empty(), "and a hole with no spin has no arrow");
    }

    /// The canvas the zone geometry is measured against: off the origin and not square, so that
    /// nothing in the arithmetic can come out right by a symmetry the real view does not have.
    fn zone_canvas() -> egui::Rect {
        egui::Rect::from_min_size(Pos2::new(13.0, 41.0), Vec2::new(903.0, 617.0))
    }

    /// One boundary of on-screen radius `radius` as the view would cut it, with the centre of the
    /// hole placed straight below the middle of the canvas so that the circle runs through it.
    /// That is the geometry of every deep zoom: the centre far off the view, one boundary crossing
    /// it. At small radii the centre lands on the canvas and the same call gives the whole turn.
    fn zone_ring(radius: f64) -> (ZoneArc, Vec<Pos2>) {
        let rect = zone_canvas();
        let centre = (f64::from(rect.center().x), f64::from(rect.center().y) + radius);
        let arc = ZoneArc::over(rect, centre, radius);
        let points = arc.ring(radius);
        (arc, points)
    }

    /// How far the chords of a drawn boundary fall inside the circle they stand for, and how far
    /// its vertices stray off that circle, both measured in f64 from the points themselves.
    fn zone_error(arc: &ZoneArc, radius: f64, points: &[Pos2]) -> (f64, f64) {
        let radial = |x: f64, y: f64| (x - arc.centre.0).hypot(y - arc.centre.1);
        let mut sagitta = 0.0_f64;
        let mut off_circle = 0.0_f64;
        let mut loop_back = points.to_vec();
        if arc.closed {
            loop_back.push(points[0]);
        }
        for pair in loop_back.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let mid = (
                0.5 * (f64::from(a.x) + f64::from(b.x)),
                0.5 * (f64::from(a.y) + f64::from(b.y)),
            );
            sagitta = sagitta.max(radius - radial(mid.0, mid.1));
            off_circle =
                off_circle.max((radial(f64::from(a.x), f64::from(a.y)) - radius).abs());
        }
        (sagitta, off_circle)
    }

    #[test]
    fn test_a_zone_boundary_is_cut_to_a_quarter_pixel_from_ten_pixels_to_a_billion() {
        // The defect this rule replaces: a boundary cut into a fixed number of segments is a fixed
        // *fraction* of its radius away from the circle, so however good it looks at the default
        // zoom it is wrong by whole pixels once the view is deep. Fixing the error in pixels
        // instead has to hold at every radius the view can reach - the zoom runs to 500 000 px per
        // M, and the ergosphere is 2.2 M out - and the count it asks for has to stay bounded while
        // it does, which it can only do by spending its segments on the stretch of the circle the
        // canvas can actually see.
        let mut worst_sagitta = 0.0_f64;
        let mut most_points = 0usize;
        for step in 0..=16 {
            // Ten pixels to a billion, a factor of sqrt(10) at a time.
            let radius = 10.0 * 10.0_f64.powf(f64::from(step) / 2.0);
            let (arc, points) = zone_ring(radius);
            let (sagitta, off_circle) = zone_error(&arc, radius, &points);
            assert!(
                sagitta <= ZONE_SAGITTA_PX + 1e-3,
                "a chord of the {radius} px boundary falls {sagitta} px inside it"
            );
            assert!(
                off_circle < 1e-3,
                "a vertex of the {radius} px boundary sits {off_circle} px off it"
            );
            assert!(
                points.len() <= ZONE_SEGMENTS_MAX + 1,
                "the {radius} px boundary took {} points",
                points.len()
            );
            worst_sagitta = worst_sagitta.max(sagitta);
            most_points = most_points.max(points.len());
            println!(
                "R = {radius:>12.0} px: {:>3} points over {:.6} of a turn, chords {sagitta:.4} px \
                 inside the circle, vertices {off_circle:.2e} px off it",
                points.len(),
                (arc.to - arc.from) / std::f64::consts::TAU
            );
        }
        // The whole turn of a small circle is the most segments any of this asks for, and it is
        // the floor rather than the tolerance that asks for them.
        assert_eq!(most_points, ZONE_SEGMENTS, "the widest cut is a whole turn at the floor");
        println!("worst chord: {worst_sagitta:.4} px inside; most points: {most_points}");
    }

    #[test]
    fn test_a_boundary_a_hundred_million_pixels_across_is_drawn_across_the_whole_canvas() {
        // The deep zoom, where the centre of the hole is 1e8 px off the canvas and the boundary
        // through the view is a stretch of circle a hundredth of a milliradian long. Two things
        // have to hold: every vertex is on that circle to far better than a pixel, which is what
        // the f64 centre buys - the same points formed in f32 would be out by eight pixels, since
        // that is the spacing of the f32 grid at 1e8 - and the drawn stretch covers every point of
        // the circle that is on the canvas, with no gap left at the edges of the view.
        let radius = 1e8;
        let rect = zone_canvas();
        let (arc, points) = zone_ring(radius);
        let (sagitta, off_circle) = zone_error(&arc, radius, &points);
        assert!(!arc.closed, "the canvas sees an arc of this circle and not a turn of it");
        assert!(off_circle < 1e-3, "a vertex sits {off_circle} px off the circle");
        assert!(sagitta <= ZONE_SAGITTA_PX + 1e-3, "a chord falls {sagitta} px inside it");
        assert!(
            !rect.contains(points[0]) && !rect.contains(*points.last().expect("an arc has ends")),
            "both ends of the arc are drawn past the edge of the canvas"
        );

        // Every angle of the circle that lands on the canvas is inside the drawn stretch, tested
        // over three times that stretch so that a miss at either end would show.
        let (middle, wide) = (0.5 * (arc.from + arc.to), 3.0 * (arc.to - arc.from));
        let samples = 4000;
        let mut on_canvas = 0;
        for i in 0..=samples {
            let theta = middle - 0.5 * wide + wide * f64::from(i) / f64::from(samples);
            let (sin, cos) = theta.sin_cos();
            let at = Pos2::new(
                (arc.centre.0 + radius * cos) as f32,
                (arc.centre.1 + radius * sin) as f32,
            );
            if rect.contains(at) {
                on_canvas += 1;
                assert!(
                    theta > arc.from && theta < arc.to,
                    "the circle is on the canvas at {theta} rad, outside the drawn stretch \
                     [{}, {}]",
                    arc.from,
                    arc.to
                );
            }
        }
        assert!(on_canvas > 100, "only {on_canvas} of {samples} samples landed on the canvas");
        println!(
            "R = 1e8 px: {} points over {:.3e} rad, {on_canvas}/{samples} samples on the canvas, \
             vertices {off_circle:.2e} px off the circle",
            points.len(),
            arc.to - arc.from
        );
    }

    #[test]
    fn test_a_boundary_the_canvas_cannot_reach_is_not_drawn_and_one_around_it_is_a_whole_turn() {
        let rect = zone_canvas();
        let middle = (f64::from(rect.center().x), f64::from(rect.center().y));

        // The centre on the canvas: the whole turn of every circle can show, so an outline is a
        // loop and the cut is the floor of seventy-two.
        let arc = ZoneArc::over(rect, middle, 100.0);
        assert!(arc.closed, "the canvas is around the centre, so the whole turn shows");
        assert_eq!(arc.ring(100.0).len(), ZONE_SEGMENTS, "and is cut at the floor");
        assert!(arc.shows(100.0), "a circle within the corners crosses the canvas");
        assert!(!arc.shows(1e6), "and one far outside them does not");
        assert!(
            arc.held(1e6) < 600.0,
            "a circle outside the view is held to the rim of it, at {} px",
            arc.held(1e6)
        );
        assert!(arc.held(0.0) == 0.0, "and the innermost edge is the centre itself");

        // The centre a million pixels away: only the circles that pass through the canvas are
        // drawn, and the ones short of it and beyond it are not.
        let away = (middle.0, middle.1 + 1e6);
        let arc = ZoneArc::over(rect, away, 1e6);
        assert!(!arc.closed, "the centre is off the canvas, so only an arc of it shows");
        assert!(arc.shows(1e6), "the circle through the canvas is drawn");
        assert!(!arc.shows(1e6 - 5000.0), "one five thousand pixels short of it is not");
        assert!(!arc.shows(1e6 + 5000.0), "nor is one five thousand pixels beyond it");
        // Both radii of a band outside the view are held to the same value, which is how the
        // caller knows there is no band to draw.
        assert_eq!(
            arc.held(1e6 - 5000.0),
            arc.held(1e6 - 4000.0),
            "a band wholly outside the canvas has nothing of it in it"
        );
    }

    /// One filled zone as it reached the painter: the colour of its mesh and every vertex of it.
    type ZoneFill = (Color32, Vec<Pos2>);

    /// One boundary as it reached the painter: its colour, whether it is a loop rather than an
    /// open arc, and the points it is stroked through.
    type ZoneLine = (Color32, bool, Vec<Pos2>);

    /// The zone geometry of one frame of the equatorial view at this zoom and pan.
    fn zone_frame(zoom: f32, pan: Vec2) -> (Vec<ZoneFill>, Vec<ZoneLine>) {
        use crate::physics::wavefront::SignalField;
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let metric = KerrSchild::new(1.0, 0.90);
        let mut canvas = SpatialCanvas { zoom, pan_offset: pan, ..Default::default() };
        let signal = SignalField::default();
        let mut details = false;
        let (mut alice, mut bob): (Option<Observer>, Option<Observer>) = (None, None);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            canvas.render(
                ui,
                &metric,
                &mut bob,
                &mut alice,
                0.0,
                SignalViews { alice: &signal, bob: &signal },
                600.0,
                false,
                ReferenceFrame::DistantObserver,
                1.0,
                FrontStyle { arcs: true, hide_wound: true },
                &mut details,
            );
        });
        let (mut fills, mut lines) = (Vec::new(), Vec::new());
        fn walk(shape: &egui::Shape, fills: &mut Vec<ZoneFill>, lines: &mut Vec<ZoneLine>) {
            match shape {
                egui::Shape::Mesh(mesh) => {
                    if let Some(first) = mesh.vertices.first() {
                        fills.push((
                            first.color,
                            mesh.vertices.iter().map(|v| v.pos).collect(),
                        ));
                    }
                }
                egui::Shape::Path(path) => {
                    if let egui::epaint::ColorMode::Solid(colour) = path.stroke.color {
                        lines.push((colour, path.closed, path.points.clone()));
                    }
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, fills, lines);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut fills, &mut lines);
        }
        output.drop_without_applying_deltas();
        (fills, lines)
    }

    #[test]
    fn test_every_boundary_is_stroked_through_the_points_of_the_fills_it_divides() {
        // The guarantee the whole fix rests on, measured off a painted frame rather than off the
        // helper's arithmetic: a boundary line and the edges of the two fills that meet at it are
        // one list of points, to the bit. Two tessellations of the same circle can be made to
        // agree to a tolerance; one list cannot disagree at all, at any zoom.
        //
        // Two frames. At the default zoom the whole hole is on the canvas and all four boundaries
        // are loops. At 200 000 px per M, with the pan putting r- through the middle of the view,
        // the centre of the geometry is 210 000 px off the canvas, the ring and the two outer
        // boundaries are nowhere near it, and the one boundary that shows is an open arc - which
        // is the state the user's screenshot was taken in.
        let metric = KerrSchild::new(1.0, 0.90);
        let deep_zoom = 200_000.0_f32;
        let r_minus_px =
            metric.cartesian_radius(metric.inner_horizon()) * f64::from(deep_zoom);
        let deep_pan = Vec2::new(0.0, r_minus_px as f32);
        for (zoom, pan, want_closed) in
            [(48.0_f32, Vec2::ZERO, true), (deep_zoom, deep_pan, false)]
        {
            let (fills, lines) = zone_frame(zoom, pan);
            let mut checked = 0;
            for (line_colour, either_side) in [
                (Theme::SINGULARITY_LINE, [Theme::SINGULARITY_FILL, Theme::REGION_III_FILL]),
                (Theme::HORIZON_CAUCHY, [Theme::REGION_III_FILL, Theme::REGION_II_FILL]),
                (Theme::HORIZON_OUTER, [Theme::REGION_II_FILL, Theme::ERGOSPHERE_FILL]),
                (Theme::ERGOSPHERE_LINE, [Theme::ERGOSPHERE_FILL, Theme::ERGOSPHERE_FILL]),
            ] {
                let Some((_, closed, points)) =
                    lines.iter().find(|(colour, _, _)| *colour == line_colour)
                else {
                    continue; // This boundary is off the canvas at this zoom, so it is not drawn.
                };
                assert_eq!(*closed, want_closed, "at zoom {zoom} the boundary is a loop or an arc");
                for fill_colour in either_side {
                    let Some((_, vertices)) = fills.iter().find(|(c, _)| *c == fill_colour) else {
                        continue; // The region on that side has no part of the canvas in it.
                    };
                    for at in points {
                        assert!(
                            vertices.contains(at),
                            "at zoom {zoom} a point of the boundary is not a vertex of the fill \
                             beside it: {at:?}"
                        );
                    }
                    checked += 1;
                }
                println!(
                    "zoom {zoom}: the boundary {line_colour:?} runs through {} shared points",
                    points.len()
                );
            }
            assert!(checked > 0, "at zoom {zoom} no boundary met a fill at all");
        }
    }

    /// One real frame of the equatorial view, returning every filled circle it painted - colour,
    /// radius and centre - and the id of the `Ui` it was drawn in, which is what the marker menus
    /// are keyed off.
    fn spatial_frame(
        canvas: &mut SpatialCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        alice: &mut Option<Observer>,
        bob: &mut Option<Observer>,
        events: Vec<egui::Event>,
    ) -> (Vec<(Color32, f32, Pos2)>, egui::Id) {
        spatial_frame_in(ReferenceFrame::DistantObserver, canvas, ctx, metric, alice, bob, events)
    }

    /// `spatial_frame`, drawn with the View selector at `frame`.
    fn spatial_frame_in(
        frame: ReferenceFrame,
        canvas: &mut SpatialCanvas,
        ctx: &egui::Context,
        metric: &KerrSchild,
        alice: &mut Option<Observer>,
        bob: &mut Option<Observer>,
        events: Vec<egui::Event>,
    ) -> (Vec<(Color32, f32, Pos2)>, egui::Id) {
        use crate::physics::wavefront::SignalField;
        let signal = SignalField::default();
        let mut details = true;
        let mut ui_id = egui::Id::NULL;
        // The observers' own t, because in the app every worldline stands at the simulation clock,
        // and a drag places them at it.
        let clock = alice.as_ref().or(bob.as_ref()).map_or(0.0, |obs| obs.t);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            events,
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            ui_id = ui.id();
            canvas.render(
                ui,
                metric,
                bob,
                alice,
                clock,
                SignalViews { alice: &signal, bob: &signal },
                600.0,
                false,
                frame,
                1.0,
                FrontStyle { arcs: true, hide_wound: true },
                &mut details,
            );
        });
        let mut circles = Vec::new();
        fn walk(shape: &egui::Shape, out: &mut Vec<(Color32, f32, Pos2)>) {
            match shape {
                egui::Shape::Circle(c) => out.push((c.fill, c.radius, c.center)),
                // The zone fills are meshes cut to what the canvas can show rather than circles,
                // so each is entered here as a circle of no radius at its first vertex. That
                // vertex is the inner edge of the innermost band, which is the centre of the
                // geometry itself while the centre is on the canvas: see `hole_of`.
                egui::Shape::Mesh(mesh) => {
                    if let Some(first) = mesh.vertices.first() {
                        out.push((first.color, 0.0, first.pos));
                    }
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, out);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut circles);
        }
        output.drop_without_applying_deltas();
        (circles, ui_id)
    }

    /// Where the hole was painted this frame: the fill of the disc inside the ring is laid down
    /// once, and its inner edge is the centre of the geometry itself whenever that centre is on
    /// the canvas - which it is in every frame these tests run, none of them panning the hole off
    /// the view. With no pan and nobody being tracked it is also the middle of the canvas, which
    /// is how a test knows where the middle is without knowing the layout.
    fn hole_of(circles: &[(Color32, f32, Pos2)]) -> Pos2 {
        circles
            .iter()
            .find(|(fill, _, _)| *fill == Theme::SINGULARITY_FILL)
            .map(|(_, _, at)| *at)
            .expect("the ring's fill marks the centre of the hole")
    }

    /// Where the observer's own marker was painted this frame.
    fn marker_of(circles: &[(Color32, f32, Pos2)], who: Who) -> Pos2 {
        let colour = if who == Who::Alice { Theme::ALICE_COLOR } else { Theme::BOB_COLOR };
        circles
            .iter()
            .find(|(fill, r, _)| *fill == colour && (*r - who.marker_radius()).abs() < 1e-6)
            .map(|(_, _, at)| *at)
            .unwrap_or_else(|| panic!("{}'s marker is painted", who.name()))
    }

    /// The text of every galley the equatorial view painted for one frame with this observer on
    /// it. Fonts are left empty, as they are everywhere else a frame is run in a test: a galley
    /// carries its string whether or not there are glyphs to draw it with.
    fn spatial_frame_text(metric: &KerrSchild, bob: &mut Option<Observer>) -> String {
        use crate::physics::wavefront::SignalField;
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        // The boxes start shut, and the text wanted here is what an open one prints.
        canvas.telemetry.collapsed.clear();
        let signal = SignalField::default();
        let mut details = true;
        let mut alice: Option<Observer> = None;
        let clock = bob.as_ref().map_or(0.0, |obs| obs.t);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, egui::Vec2::new(800.0, 700.0))),
            ..Default::default()
        };
        let output = ctx.clone().run_ui(input, |ui| {
            canvas.render(
                ui,
                metric,
                bob,
                &mut alice,
                clock,
                SignalViews { alice: &signal, bob: &signal },
                600.0,
                false,
                ReferenceFrame::DistantObserver,
                1.0,
                FrontStyle { arcs: true, hide_wound: true },
                &mut details,
            );
        });
        fn collect(shape: &egui::Shape, out: &mut String) {
            match shape {
                egui::Shape::Text(text) => {
                    out.push_str(text.galley.text());
                    out.push('\n');
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        collect(shape, out);
                    }
                }
                _ => {}
            }
        }
        let mut text = String::new();
        for clipped in output.shapes.iter() {
            collect(&clipped.shape, &mut text);
        }
        output.drop_without_applying_deltas();
        text
    }

    #[test]
    fn test_a_frozen_observer_is_labelled_frozen_on_the_equatorial_view() {
        // On this view a worldline frozen on the far branch of r- is a dot creeping round a
        // circle at a steady rate, which is exactly what an ordinary orbit looks like. It is not
        // one - the radius and the observer's own clock have stopped, and the motion left is the
        // horizon's null generator carrying him - so the marker has to say so, and only when it
        // is true: a Bob still falling gets no such label.
        let metric = KerrSchild::new(1.0, 0.90);
        let mut frozen = Some(Observer::frozen_bob(&metric));
        let text = spatial_frame_text(&metric, &mut frozen);
        assert!(
            text.contains("Frozen"),
            "the frozen marker is unlabelled; the view painted:\n{text}"
        );

        let mut falling = Some(Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            9.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::new(1.0, 2.2, false),
        ));
        falling.as_mut().unwrap().step(&metric, 0.25, 0.25);
        assert!(!falling.as_ref().unwrap().is_frozen(), "he has only just been let go of");
        let text = spatial_frame_text(&metric, &mut falling);
        assert!(
            !text.contains("Frozen"),
            "a falling observer is labelled frozen; the view painted:\n{text}"
        );
    }

    #[test]
    fn test_keeping_an_observer_centred_pans_the_view_under_them_instead_of_moving_them() {
        // What the toggle on the marker's right-click menu has to do: with it off, an observer
        // falling inward crosses the canvas while the hole stays put, which is the ordinary view;
        // with it on, the observer holds still in the middle and the hole - and with it every
        // horizon, the ring and everything else drawn in the geometry - slides past instead. The
        // two are the same picture from a different place, which is the whole content of "keep
        // this one centred", and it is independent of the View selector: this is
        // drawn in the global foliation throughout.
        let metric = KerrSchild::new(1.0, 0.65);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::default(),
        ));
        let fall = |bob: &mut Option<Observer>| {
            let b = bob.as_mut().expect("Bob is in this run");
            for _ in 0..10 {
                b.step(&metric, b.t + 0.1, 0.1);
            }
        };
        // Off: Bob moves, the hole does not.
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let bob_moved = (marker_of(&after, Who::Bob) - marker_of(&before, Who::Bob)).length();
        let hole_moved = (hole_of(&after) - hole_of(&before)).length();
        assert!(bob_moved > 5.0, "he crosses the canvas as he falls: {bob_moved} px");
        assert!(hole_moved < 1e-3, "and the hole stays where it is: {hole_moved} px");

        // On: Bob does not move, the hole does, and by exactly what his own motion would have been.
        canvas.centred_on = Some(Who::Bob);
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (moved, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let bob_held = (marker_of(&moved, Who::Bob) - marker_of(&before, Who::Bob)).length();
        let hole_slid = hole_of(&moved) - hole_of(&before);
        println!(
            "uncentred: Bob moved {bob_moved:.1} px and the hole {hole_moved:.3} px; centred: Bob \
             moved {bob_held:.3} px and the hole {:.1} px",
            hole_slid.length()
        );
        assert!(bob_held < 1e-3, "centred, he holds still: {bob_held} px");
        assert!(hole_slid.length() > 5.0, "and the geometry slides past him: {hole_slid:?}");
        // He is held where the ring around his marker says he is: the same point of the canvas as
        // the frame before, whatever the observer did in between.
        let his_marker = marker_of(&moved, Who::Bob);
        assert!(
            moved.iter().any(|(_, r, at)| {
                (*r - (Who::Bob.marker_radius() + CENTRED_RING_GAP)).abs() < 1e-6
                    && (*at - his_marker).length() < 1e-6
            }),
            "and the ring that says he is the one being followed is drawn on him: {moved:?}"
        );

        // Switched off again, he goes back to crossing the canvas.
        canvas.centred_on = None;
        let (parked, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        fall(&mut bob);
        let (released, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (marker_of(&released, Who::Bob) - marker_of(&parked, Who::Bob)).length() > 5.0,
            "with the toggle off he is on the move again"
        );
        assert!(
            (hole_of(&released) - hole_of(&parked)).length() < 1e-3,
            "and the hole is back to standing still"
        );
    }

    #[test]
    fn test_keeping_an_observer_centred_brings_them_to_the_middle_from_any_pan() {
        // Taking hold used to set the standing request and nothing else. The view's placement is
        // `rect.center() + pan_offset - tracking`, which puts the observer being held at
        // `rect.center() + pan_offset`, so the pan had to be zero for "Keep Bob Centered" to mean
        // what it says. It usually is not: a drag writes it, and so does every wheel zoom about a
        // cursor that is not dead centre, which compounds as the view zooms in. Taking hold
        // therefore slid the whole picture by Bob's own offset and then left him at the accumulated
        // pan - off an 800 x 600 canvas entirely for a pan of a few hundred pixels, which is the
        // reported "jumped into space where Bob was not visible".
        //
        // Nothing here steps the clock after the fall: the whole defect is in the view, and it
        // showed up with the run paused.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 8.0, 0.0, 0.0, params));
        let dropped = bob.as_mut().expect("Bob is in this run");
        for _ in 0..20 {
            dropped.step(&metric, dropped.t + 0.05, 0.05);
        }
        let centre = Pos2::new(400.0, 300.0);
        let frame = ReferenceFrame::DistantObserver;

        // The pan the user's dragging and zooming had accumulated by then, set directly here for
        // the same reason the hole's case sets it directly: what is under test is the hold, not the
        // arithmetic of the wheel.
        canvas.pan_offset = Vec2::new(520.0, -260.0);
        let (panned, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let adrift = (marker_of(&panned, Who::Bob) - centre).length();
        assert!(adrift > 300.0, "the pan has carried Bob {adrift} px off the middle");

        // Held, he is in the middle, and the pan that was measured against the old anchor is spent.
        let pan_before = canvas.pan_offset;
        canvas.hold_observer(&metric, &bob, &alice, frame, Who::Bob, true);
        let (held, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let at = marker_of(&held, Who::Bob);
        println!(
            "a pan of {pan_before:?} left Bob {adrift:.0} px off centre; held, he is at {at:?} \
             against the middle {centre:?}"
        );
        assert!((at - centre).length() < 1.0, "Bob is in the middle: {at:?}");
        assert_eq!(canvas.centred_on, Some(Who::Bob), "and the view is holding him");

        // Letting go does not jump the picture either: the same world point stays in the middle,
        // so the view is left looking where the user was looking and Bob drifts out of it from
        // there, which is the difference from a Goto that this keeps.
        let hole_held = hole_of(&held);
        canvas.hold_observer(&metric, &bob, &alice, frame, Who::Bob, false);
        let (freed, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let hole_freed = hole_of(&freed);
        println!("let go, the hole stays at {hole_freed:?} against {hole_held:?}");
        assert!(
            (hole_freed - hole_held).length() < 1.0,
            "letting go leaves the view where it was: the hole moved {hole_held:?} -> {hole_freed:?}"
        );
        assert_eq!(canvas.centred_on, None, "and nobody is held any more");

        // The hole's own hold answers for the pan the same way: it comes to the middle whatever the
        // pan was, and it takes over from Bob.
        canvas.pan_offset = Vec2::new(-310.0, 190.0);
        canvas.hold_observer(&metric, &bob, &alice, frame, Who::Bob, true);
        canvas.hold_hole(&metric, &bob, &alice, frame, true);
        let (hole_held_now, _) =
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert_eq!(canvas.centred_on, None, "holding the hole lets go of Bob");
        assert!(
            (hole_of(&hole_held_now) - centre).length() < 1.0,
            "and puts the hole in the middle: {:?}",
            hole_of(&hole_held_now)
        );
    }

    /// The pointer events one step of a drag is made of: a move, optionally with the button going
    /// down or coming up.
    #[test]
    fn test_keeping_the_black_hole_centred_pins_it_to_the_middle_whatever_else_is_asked() {
        // The hole's hold is the strongest request the view takes: it overrides a standing hold
        // on an observer, and it pins the pan, so a view that has been dragged or zoomed about
        // the cursor puts the hole back in the middle on the next frame and keeps it there.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));

        // Dragged off to one side, the hole is off centre.
        canvas.pan_offset = Vec2::new(120.0, -60.0);
        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = Pos2::new(400.0, 300.0);
        let off = (hole_of(&painted) - centre).length();
        assert!(off > 100.0, "the pan carried the hole {off} px from the middle");

        // Held, it is back in the middle and the pan is gone; and a standing hold on Bob, which
        // would put him in the middle instead, yields to it.
        canvas.centred_on = Some(Who::Bob);
        canvas.keep_hole_centred = true;
        let (held, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let hole = hole_of(&held);
        println!("hole held: at {hole:?} against the middle {centre:?}");
        assert!((hole - centre).length() < 1.0, "the hole is in the middle: {hole:?}");
        assert_eq!(canvas.pan_offset, Vec2::ZERO, "and the pan is pinned at zero");
        let bob_at = marker_of(&held, Who::Bob);
        assert!(
            (bob_at - centre).length() > 100.0,
            "Bob, at r = 6, is not in the middle while the hole is: {bob_at:?}"
        );

        // A drag across the canvas does not move it.
        let travel = Vec2::new(-90.0, 40.0);
        let start = centre + Vec2::new(150.0, 150.0);
        for (pos, pressed) in [(start, Some(true)), (start + travel, None), (start + travel, Some(false))] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (hole_of(&after) - centre).length() < 1.0,
            "dragged, the hole stays in the middle: {:?}",
            hole_of(&after)
        );

        // Let go, the pan is free again and the hold on Bob is what remains only if it was set
        // after: here it was cleared by nothing, so the view simply stops holding the hole.
        canvas.keep_hole_centred = false;
        canvas.centred_on = None;
        canvas.pan_offset = Vec2::new(80.0, 0.0);
        let (free, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!((hole_of(&free) - centre).length() > 50.0, "released, the pan moves it again");
    }

    fn pointer(pos: Pos2, pressed: Option<bool>) -> Vec<egui::Event> {
        let mut events = vec![egui::Event::PointerMoved(pos)];
        if let Some(pressed) = pressed {
            events.push(egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: Default::default(),
            });
        }
        events
    }

    #[test]
    fn test_either_marker_can_be_dragged_anywhere_in_the_plane() {
        // The drag lives here now, and this is why: the equatorial view draws the plane the two
        // observers actually stand in, so a drag on it sets both coordinates an equatorial observer
        // has. On the (t, r) diagram, where this used to be, the whole plane is squeezed onto one
        // axis and a drag could only ever slide somebody along the radius - the pair could never be
        // put on opposite sides of the hole, which is the arrangement that decides how long light
        // takes to cross between them and how much of the other's frozen light a crosser meets.
        //
        // Bob is dragged to the point diametrically opposite Alice. Both are then at the same
        // drawn radius, so the embedding turns both by the same atan2(a, r) and the azimuths differ
        // by exactly half a turn.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice =
            Some(Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.25, params));
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, params));
        let mode_before = bob.as_ref().expect("Bob is in this run").mode;

        // Frame 1: nothing but the painting, to find out where the markers are.
        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&painted);
        let her = marker_of(&painted, Who::Alice);
        let his = marker_of(&painted, Who::Bob);
        assert!((his - centre).length() > 50.0, "his marker is well clear of the hole");

        // The antipode of Alice's marker, which is where Bob is going. egui calls the drag
        // started on the frame that crosses its own threshold, and the grab offset is taken from
        // the pointer *there*, so the marker moves by what the pointer has moved since that frame:
        // to land him on the target, the pointer finishes that much past it.
        let target = centre - (her - centre);
        let nudge = his + Vec2::new(12.0, 0.0);
        let finish = target + (nudge - his);
        for (pos, pressed) in [
            (his, Some(true)),      // press on him
            (nudge, None),          // past egui's drag threshold: the pick happens here
            (finish, None),         // the move itself
            (finish, Some(false)),  // and the release
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }

        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = marker_of(&after, Who::Bob);
        let al = alice.as_ref().expect("Alice is in this run");
        let b = bob.as_ref().expect("Bob is in this run");
        let turn = std::f64::consts::TAU;
        let apart = (b.phi - al.phi).rem_euclid(turn);
        println!(
            "Bob dragged to Alice's antipode: r = {:.4} against her {:.4}, and their azimuths are \
             {:.4} rad apart",
            b.r,
            al.r,
            apart.min(turn - apart)
        );
        assert!((landed - target).length() < 2.0, "the marker went where the pointer did: {landed:?}");
        assert!((b.r - al.r).abs() < 0.05, "at the same radius she is at: {} vs {}", b.r, al.r);
        assert!(
            (apart.min(turn - apart) - std::f64::consts::PI).abs() < 0.02,
            "half a turn apart: {apart} rad"
        );
        assert_eq!(b.mode, mode_before, "and dropped back onto the Motion he was on");
        assert!(canvas.dragging.is_none(), "with the canvas no longer holding him");
        // Alice, who was not touched, has not moved at all.
        assert!((al.r - 4.5).abs() < 1e-12 && (al.phi - 0.25).abs() < 1e-12, "Alice stayed put");
    }

    #[test]
    fn test_a_drag_keeps_the_grab_offset_and_stops_at_the_ring() {
        // Two things the drag has to get right about the pointer. It is picked up wherever it is
        // pressed - press a marker off centre and it must not jump under the cursor - and it cannot
        // be dropped onto the ring: the disc rho < a in the middle of the picture is not a region
        // of the equatorial plane at all, and the ring itself is the curvature singularity, so the
        // radius floors at `RING_DROP_FLOOR` rather than going to nothing.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));

        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&painted);
        let his = marker_of(&painted, Who::Bob);
        // Pressed a little off centre, which is where the grab offset earns its place.
        // The travel is measured from the frame egui calls the drag started on, since that is
        // the pointer the grab offset is taken against.
        let press = his + Vec2::new(5.0, 3.0);
        let nudge = press + Vec2::new(12.0, 0.0);
        let travel = Vec2::new(-60.0, 40.0);
        for (pos, pressed) in [
            (press, Some(true)),
            (nudge, None),
            (nudge + travel, None),
            (nudge + travel, Some(false)),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = marker_of(&after, Who::Bob);
        println!(
            "pressed {:.0} px off his centre and moved {:?}: the marker moved to {landed:?}, \
             wanted {:?}",
            (press - his).length(),
            travel,
            his + travel
        );
        assert!(
            (landed - (his + travel)).length() < 2.0,
            "the marker moves by what the pointer moved, not to where it is: {landed:?}"
        );

        // And now into the middle of the hole, which is not a place.
        let nudge = landed + Vec2::new(12.0, 0.0);
        let into_the_hole = centre + (nudge - landed);
        for (pos, pressed) in [
            (landed, Some(true)),
            (nudge, None),
            (into_the_hole, None),
            (into_the_hole, Some(false)),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let r = bob.as_ref().expect("Bob is in this run").r;
        println!("dragged onto the middle of the ring: r = {r} against a floor of {RING_DROP_FLOOR}");
        assert!((r - RING_DROP_FLOOR).abs() < 1e-9, "he stops at the ring floor: {r}");
    }

    #[test]
    fn test_centring_lets_go_of_an_observer_while_they_are_being_dragged() {
        // Keep Centered pans the view to hold somebody in the middle, which is exactly the wrong
        // thing to do to the observer under the pointer: the pan would cancel every pixel of the
        // drag, the marker would sit pinned to the centre, and the drag would look like it was
        // doing nothing while in fact moving them. So a drag of the centred observer suspends the
        // centring, and it resumes on release with the view re-centring on wherever they were put.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));
        canvas.centred_on = Some(Who::Bob);

        let (painted, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let at = marker_of(&painted, Who::Bob);
        let travel = Vec2::new(-70.0, 35.0);
        for (pos, pressed) in [
            (at, Some(true)),
            (at + Vec2::new(12.0, 0.0), None),
            (at + travel, None),
        ] {
            spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed));
        }
        let (held, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let moved = marker_of(&held, Who::Bob);
        println!("centred on Bob and dragged {travel:?}: his marker went from {at:?} to {moved:?}");
        assert!(
            (moved - at).length() > 20.0,
            "the drag moves him across the canvas rather than being cancelled by the pan: {moved:?}"
        );

        // Released, the hold comes back and puts him in the middle again.
        spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(at + travel, Some(false)));
        let (released, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let recentred = marker_of(&released, Who::Bob);
        assert_eq!(canvas.centred_on, Some(Who::Bob), "the request was never cancelled");
        assert!(
            (recentred - hole_of(&painted)).length() < 2.0 || (recentred - at).length() < 2.0,
            "and he is back in the middle of the view: {recentred:?}"
        );
    }

    #[test]
    fn test_a_drag_in_the_dragged_observers_own_rest_frame_does_not_run_away() {
        // The View selector's rest frame follows its observer the way Keep Centered does, and a
        // drag of that observer used to be followed too: every frame the view re-centred on where
        // they had been put, the pointer was left standing off the marker by what it had moved,
        // and the next frame moved them by that much again. A nudge held still carried them on
        // across the plane for as long as the button was down. The picture now stands still under
        // the pointer for the length of the drag, so holding the pointer still holds them still.
        let metric = KerrSchild::new(1.0, 0.90);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice: Option<Observer> = None;
        let mut bob = Some(Observer::new_with_phi(&metric, "Bob", 0.0, 6.0, 0.0, 0.0, params));
        let frame = ReferenceFrame::Bob;

        let (painted, _) =
            spatial_frame_in(frame, &mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let at = marker_of(&painted, Who::Bob);
        let hole = hole_of(&painted);
        let nudge = at + Vec2::new(12.0, 0.0);
        let travel = Vec2::new(-40.0, 25.0);
        let mut events = vec![(at, Some(true)), (nudge, None), (nudge + travel, None)];
        // Held there, still pressed, for a good many frames.
        events.extend(std::iter::repeat_n((nudge + travel, None), 20));
        let mut held = Vec::new();
        for (pos, pressed) in events {
            held = spatial_frame_in(
                frame, &mut canvas, &ctx, &metric, &mut alice, &mut bob, pointer(pos, pressed),
            )
            .0;
        }
        let moved = marker_of(&held, Who::Bob);
        println!("Bob's own frame, dragged {travel:?} and held: {at:?} -> {moved:?}");
        assert!(
            (moved - (at + travel)).length() < 2.0,
            "the marker is where the pointer put it, and stays there: {moved:?}"
        );
        assert!((hole_of(&held) - hole).length() < 1.0, "and the picture did not move under it");

        // Released, the rest frame takes hold of him again where he was put.
        spatial_frame_in(
            frame, &mut canvas, &ctx, &metric, &mut alice, &mut bob,
            pointer(nudge + travel, Some(false)),
        );
        let (released, _) =
            spatial_frame_in(frame, &mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (marker_of(&released, Who::Bob) - at).length() < 2.0,
            "and he is back in the middle of his own frame"
        );
    }

    #[test]
    fn test_going_to_an_observer_centres_them_once_and_lets_them_move_again() {
        // The menu's two kinds of item, told apart. Goto is a pan: it puts somebody in the middle
        // of the canvas at the moment it is asked, and from then on they are free to fall out of
        // it again, which is what makes it the right answer to "where did Bob get to" and the
        // wrong one to "follow Bob". Keep Centered is the standing request, and the test above is
        // about that one.
        let metric = KerrSchild::new(1.0, 0.65);
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let mut canvas = SpatialCanvas::default();
        let params = crate::physics::observer::WorldlineParams::default();
        let mut alice =
            Some(Observer::new_with_phi(&metric, "Alice", 0.0, 9.0, 0.0, 2.2, params));
        let mut bob =
            Some(Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 0.0, 0.0, params));
        // Nothing is panned and nothing is being tracked yet, so the hole is in the middle of the
        // canvas and that is the point everything below is measured against.
        let (before, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let centre = hole_of(&before);
        let off_centre = (marker_of(&before, Who::Bob) - centre).length();
        canvas.look_at(
            &metric,
            &bob,
            &alice,
            ReferenceFrame::DistantObserver,
            bob.as_ref().expect("Bob is in this run").cartesian_position(&metric),
        );
        let (gone_to, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let landed = (marker_of(&gone_to, Who::Bob) - centre).length();
        println!(
            "Bob started {off_centre:.0} px off centre and Goto put him {landed:.3} px from it"
        );
        assert!(off_centre > 20.0, "he is not already in the middle: {off_centre} px");
        assert!(landed < 1e-3, "and Goto puts him there exactly: {landed} px");

        // Then he falls, and nothing holds him: that is the difference from Keep Centered.
        let b = bob.as_mut().expect("Bob is in this run");
        for _ in 0..10 {
            b.step(&metric, b.t + 0.1, 0.1);
        }
        let (after, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        let drifted = (marker_of(&after, Who::Bob) - centre).length();
        assert!(drifted > 5.0, "a Goto does not follow him: {drifted} px");

        // And the hole is a target like any other: it sits at the origin of the embedding.
        canvas.look_at(&metric, &bob, &alice, ReferenceFrame::DistantObserver, (0.0, 0.0));
        let (at_hole, _) = spatial_frame(&mut canvas, &ctx, &metric, &mut alice, &mut bob, vec![]);
        assert!(
            (hole_of(&at_hole) - centre).length() < 1e-3,
            "Goto Black Hole centres the hole: {:?}",
            hole_of(&at_hole)
        );
    }

    #[test]
    fn test_the_front_ramp_starts_red_and_ends_violet() {
        // The three statements the wavefront colouring makes to the eye. A front is born at gain 1
        // and must come out at the red stop exactly, because every ray of a fresh pulse is at that
        // gain and the whole point is that the loop is one colour. The frozen family runs to a gain
        // of 1e5 within a run, and that end of the ramp must be the violet stop rather than
        // saturating early or wrapping. And below 1 - a ray can lose frequency between two
        // raindrops, one climbing outward away from the congruence's fall - it must leave the
        // spectrum rather than run further along it.
        //
        // What is *not* asserted here any more is that a loss looks darker. It cannot: every stop
        // of this ramp is struck at one lightness, so that the colour carries the shift and the eye
        // has no brightness to misread. `theme::tests` measures that; this measures the landmarks.
        let red = Theme::front_colour(1.0, 255);
        assert_eq!(
            (red.r(), red.g(), red.b()),
            (
                Theme::FRONT_RED_RGB[0],
                Theme::FRONT_RED_RGB[1],
                Theme::FRONT_RED_RGB[2]
            ),
            "a front at gain 1 must be exactly the red stop"
        );
        let violet = Theme::front_colour(1e5, 255);
        assert_eq!(
            (violet.r(), violet.g(), violet.b()),
            (
                Theme::FRONT_VIOLET_RGB[0],
                Theme::FRONT_VIOLET_RGB[1],
                Theme::FRONT_VIOLET_RGB[2]
            ),
            "a gain of 1e5 must be exactly the violet stop"
        );
        // And past the top of the ramp it stays there rather than running off it: the brightest
        // rays of a real front reach 1e6 within thirty M.
        assert_eq!(Theme::front_colour(1e9, 255), violet, "the ramp is clamped at the top");

        let grey = Theme::front_colour(0.1, 255);
        assert_eq!(
            (grey.r(), grey.g(), grey.b()),
            (
                Theme::FRONT_GREY_RGB[0],
                Theme::FRONT_GREY_RGB[1],
                Theme::FRONT_GREY_RGB[2]
            ),
            "a tenfold loss must be exactly the grey stop"
        );
        println!(
            "the front ramp: gain 1 -> {:?}, 10 -> {:?}, 30 -> {:?}, 1e3 -> {:?}, 1e5 -> {:?}, \
             and 0.1 -> {:?}",
            (red.r(), red.g(), red.b()),
            {
                let c = Theme::front_colour(10.0, 255);
                (c.r(), c.g(), c.b())
            },
            {
                let c = Theme::front_colour(10.0_f64.powf(1.5), 255);
                (c.r(), c.g(), c.b())
            },
            {
                let c = Theme::front_colour(1e3, 255);
                (c.r(), c.g(), c.b())
            },
            (violet.r(), violet.g(), violet.b()),
            (grey.r(), grey.g(), grey.b()),
        );
    }

    /// Count what `draw_signal_field` puts into a painter for one field: the polylines (segments
    /// of front) and the filled circles (calculated points and emission dots).
    fn count_front_shapes(
        metric: &KerrSchild,
        field: &SignalField,
        style: FrontStyle,
    ) -> (usize, usize) {
        fn tally(shape: &egui::Shape, lines: &mut usize, circles: &mut usize) {
            match shape {
                egui::Shape::Path(_) | egui::Shape::LineSegment { .. } => *lines += 1,
                egui::Shape::Circle(_) => *circles += 1,
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        tally(shape, lines, circles);
                    }
                }
                _ => {}
            }
        }
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            let to_screen =
                |(x, y): (f64, f64)| Pos2::new(200.0 + 60.0 * x as f32, 200.0 - 60.0 * y as f32);
            draw_signal_field(&painter, metric, field, Theme::ALICE_COLOR, 1.0, style, &to_screen);
        });
        let (mut lines, mut circles) = (0, 0);
        for clipped in output.shapes.iter() {
            tally(&clipped.shape, &mut lines, &mut circles);
        }
        output.drop_without_applying_deltas();
        (lines, circles)
    }

    #[test]
    fn test_with_the_arcs_turned_off_a_front_is_its_calculated_points_and_nothing_between_them() {
        // The checkbox, at the one place it acts: `draw_signal_field` adds either one polyline per
        // live segment (arcs on) or one filled circle per live ray (arcs off), and never both. A
        // real transmission is drawn both ways into an egui painter and the shapes counted, so what
        // is measured is the drawing the user sees rather than a helper's return value. With the
        // arcs off nothing at all is drawn between neighbouring rays, however far apart they have
        // wound: that is the point of the setting, the raw integrated points with no interpolation
        // laid over them. The emission dot, one per pulse, is there either way.
        use crate::physics::observer::{Observer, WorldlineParams};
        use crate::physics::wavefront::SignalField;
        let metric = KerrSchild::new(1.0, 0.90);
        // An emitter let go inside r+ (1.436 at this spin), so the fronts it sends carry a frozen
        // family settling onto r- as well as a crossing one, and both are counted the same way.
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 1.2, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let mut t = 0.0;
        for _ in 0..40 {
            field.emit_if_due(&metric, &alice);
            alice.step(&metric, t, 0.02);
            field.advance(&metric, 0.02);
            t += 0.02;
        }
        let live_pulses: Vec<_> =
            field.pulses.iter().filter(|p| p.rays.iter().any(|r| r.alive())).collect();
        let live: usize =
            live_pulses.iter().map(|p| p.rays.iter().filter(|r| r.alive()).count()).sum();
        assert!(live > 100, "the field has a front to draw: {live} live rays");

        // The winding cut is held off in both counts, so what is measured here is the one
        // checkbox this test is about and not the two of them together.
        let (lines_on, circles_on) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_off, circles_off) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: false, hide_wound: false });
        println!(
            "arcs on: {lines_on} polylines and {circles_on} circles; arcs off: {lines_off} \
             polylines and {circles_off} circles, over {live} live rays in {} pulses",
            live_pulses.len()
        );
        assert!(lines_on > 0, "with the arcs on the front is drawn as polylines");
        assert_eq!(lines_off, 0, "with the arcs off nothing is drawn between the rays");
        assert_eq!(
            circles_off,
            live + live_pulses.len(),
            "with the arcs off every live ray is one dot, plus one emission dot per pulse"
        );
        assert!(
            circles_off > circles_on,
            "and that is more dots than the arcs-on frame carries, where the only dots are the \
             emission ones and the ends of the segments the winding cut withdrew"
        );
    }

    #[test]
    fn test_a_front_is_drawn_at_one_weight_wherever_it_stands_and_whatever_family_it_is() {
        // One front, drawn one way. The sign of E - Omega_- L is fixed at a ray's birth, so the
        // frozen family is carried by most of the prograde half of every ring an emitter sends,
        // and a pulse let go inside r+ has an arc of it already settling onto r-. That arc once
        // came out at twice the stroke width, at a higher opacity, and beaded with dots, which
        // made a seam of every front at r+ and left a gap in the fade there. Every polyline of a
        // pulse must now come out at the same stroke width and the same opacity, inside r+ as
        // outside it, and no bead at all.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let emitter =
            Observer::new_with_phi(&metric, "Bob", 0.0, 1.2, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &emitter);
        field.advance(&metric, 0.5);
        let rays = &field.pulses[0].rays;
        assert!(rays.iter().all(|ray| ray.alive()), "no ray of this pulse has reached the ring");
        let r_plus = metric.outer_horizon();
        assert!(
            rays.iter().any(|ray| ray.r < r_plus && ray.frozen(&metric)),
            "the pulse carries rays of the frozen family inside r+, which is what the test is about"
        );

        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            let to_screen =
                |(x, y): (f64, f64)| Pos2::new(200.0 + 30.0 * x as f32, 200.0 - 30.0 * y as f32);
            draw_signal_field(
                &painter,
                &metric,
                &field,
                Theme::BOB_COLOR,
                Theme::SECONDARY_FRONT_WIDTH,
                FrontStyle { arcs: true, hide_wound: true },
                &to_screen,
            );
        });
        let mut strokes: Vec<(u32, u8)> = Vec::new();
        let mut circles = 0;
        fn walk(shape: &egui::Shape, strokes: &mut Vec<(u32, u8)>, circles: &mut usize) {
            match shape {
                egui::Shape::Path(path) => {
                    let alpha = match path.stroke.color {
                        egui::epaint::ColorMode::Solid(colour) => colour.a(),
                        egui::epaint::ColorMode::UV(_) => 0,
                    };
                    strokes.push((path.stroke.width.to_bits(), alpha));
                }
                egui::Shape::Circle(_) => *circles += 1,
                egui::Shape::Vec(inner) => inner.iter().for_each(|s| walk(s, strokes, circles)),
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut strokes, &mut circles);
        }
        output.drop_without_applying_deltas();
        assert!(
            strokes.len() >= rays.len(),
            "at least one polyline per segment of the loop: {} over {} rays",
            strokes.len(),
            rays.len()
        );
        let first = strokes[0];
        assert!(
            strokes.iter().all(|s| *s == first),
            "every segment at one width and one opacity: {:?}",
            strokes.iter().collect::<std::collections::HashSet<_>>()
        );
        assert_eq!(first.1, Theme::SHIFT_ALPHA, "the one opacity a front is drawn at");
        assert_eq!(
            f32::from_bits(first.0),
            1.2 * Theme::SECONDARY_FRONT_WIDTH,
            "and the one width, which is the field's own stroke scale and nothing else"
        );
        assert_eq!(circles, 1, "the emission dot, and no beads on the frozen arc");
    }

    /// The projection the trail tests paint through: `px` pixels per M, y up, the hole in the
    /// middle of a 400 px canvas. The scale is a parameter because the fade's length is a screen
    /// quantity bounded by a screen gap, so the zoom is half of what these tests are about.
    fn test_screen(px: f32, (x, y): (f64, f64)) -> Pos2 {
        Pos2::new(200.0 + px * x as f32, 200.0 - px * y as f32)
    }

    /// One stroked band as it reached the painter: its colour, its stroke width, and its points.
    type Band = (Color32, f32, Vec<Pos2>);

    /// One field painted into a real painter at `px` pixels per M, reduced to what a test about the
    /// trailing fade has to read: every vertex of the trail meshes as (position, colour) in paint
    /// order, and every stroked band likewise. The trail slot is taken before the first pulse is
    /// drawn, so the vertices come out in the order the bands were laid however late the pulse that
    /// cast them.
    fn trail_frame(
        metric: &KerrSchild,
        field: &SignalField,
        style: FrontStyle,
        px: f32,
    ) -> (Vec<(Pos2, Color32)>, Vec<Band>) {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::empty());
        let output = ctx.run_ui(Default::default(), |ui| {
            let (_, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 400.0), egui::Sense::hover());
            let to_screen = |p: (f64, f64)| test_screen(px, p);
            draw_signal_field(&painter, metric, field, Theme::ALICE_COLOR, 1.0, style, &to_screen);
        });
        let (mut vertices, mut bands) = (Vec::new(), Vec::new());
        fn walk(shape: &egui::Shape, vertices: &mut Vec<(Pos2, Color32)>, bands: &mut Vec<Band>) {
            match shape {
                egui::Shape::Mesh(mesh) => {
                    vertices.extend(mesh.vertices.iter().map(|v| (v.pos, v.color)));
                }
                egui::Shape::Path(path) => {
                    if let egui::epaint::ColorMode::Solid(colour) = path.stroke.color {
                        bands.push((colour, path.stroke.width, path.points.clone()));
                    }
                }
                egui::Shape::Vec(inner) => {
                    for shape in inner {
                        walk(shape, vertices, bands);
                    }
                }
                _ => {}
            }
        }
        for clipped in output.shapes.iter() {
            walk(&clipped.shape, &mut vertices, &mut bands);
        }
        output.drop_without_applying_deltas();
        (vertices, bands)
    }

    /// The screen speed of the light at one ray, in pixels per M of coordinate time, worked out
    /// independently of the drawing: the chart velocity of `KerrSchild::cartesian_velocity` times
    /// the scale of `test_screen`, which is a plain similarity and so scales every direction alike.
    fn ray_speed_px(metric: &KerrSchild, ray: &NullRay, px: f32) -> f32 {
        let (vx, vy) = metric.cartesian_velocity(ray.r, ray.phi, ray.dr_dt, ray.dphi_dt);
        px * vx.hypot(vy) as f32
    }

    /// The coordinate time back to the front behind pulse `k` of a field, worked out from the
    /// emission times alone: the interval to the pulse that follows it, or, for the newest pulse
    /// that nothing follows yet, the interval to the pulse before it. None where a field carries a
    /// single pulse and there is no interval at all.
    fn gap_behind(field: &SignalField, k: usize) -> Option<f64> {
        let behind = k.checked_sub(1).and_then(|p| field.pulses.get(p));
        match (field.pulses.get(k + 1), behind) {
            (Some(next), _) => Some(next.emitted_t - field.pulses[k].emitted_t),
            (None, Some(previous)) => Some(field.pulses[k].emitted_t - previous.emitted_t),
            (None, None) => None,
        }
    }

    /// What the rule says the fade at one ray must be: the least of the ceiling, a fraction of the
    /// gap to the front behind it, and the distance the light has covered since its pulse was let
    /// go. `gap` is the coordinate time between the two emissions, and None where the field carries
    /// a single pulse and nothing follows it; `elapsed` is the ray's own clock less its pulse's
    /// emission time.
    fn expected_trail_px(speed_px: f32, gap: Option<f64>, elapsed: f64) -> f32 {
        let flown = speed_px * elapsed as f32;
        match gap {
            Some(gap) => (Theme::FRONT_TRAIL_GAP_FRACTION * speed_px * gap as f32)
                .min(Theme::FRONT_TRAIL_PX)
                .min(flown),
            None => Theme::FRONT_TRAIL_PX.min(flown),
        }
    }

    /// A single pulse let go well outside the hole and given 1 M of coordinate time to run, with
    /// the chart position of the event it was let go at. Every ray is alive and none of them is
    /// inside r+, so every segment of the loop is drawn and carries a trail. Nothing follows this
    /// pulse, so the gap does not bind, and the light has covered about 60 px at 60 px per M -
    /// twice the ceiling - so the flight does not bind either and every fade is the full
    /// `Theme::FRONT_TRAIL_PX`. A fade that runs inward therefore cannot reach past the emission
    /// point and still be further from it than the front is.
    fn outside_pulse(metric: &KerrSchild) -> (SignalField, (f64, f64)) {
        use crate::physics::observer::{Observer, WorldlineParams};
        let emitter =
            Observer::new_with_phi(metric, "Alice", 0.0, 8.0, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        field.emit_if_due(metric, &emitter);
        field.advance(metric, 1.0);
        assert_eq!(field.pulses.len(), 1, "one pulse, with nothing following it");
        let emitted = {
            let pulse = &field.pulses[0];
            assert!(pulse.rays.iter().all(|r| r.alive()), "a pulse at r = 8 loses no ray in 1 M");
            metric.cartesian_position(pulse.emitted_r, pulse.emitted_phi)
        };
        (field, emitted)
    }

    #[test]
    fn test_every_trail_runs_back_toward_the_event_its_pulse_was_let_go_at() {
        // The direction the fade is laid in is the ray's own coordinate velocity pushed through
        // the embedding and then through the projection, so for the one case where the answer is
        // known in advance it has to come out right: a fresh ring from a static emitter is
        // expanding away from the event it was let go at, and every trail on it must therefore
        // point back towards that event. Measured off a painted frame rather than off the helper's
        // arithmetic, and measured on the tail vertices the mesh actually carries. This pulse has
        // no front behind it, so the gap does not bind and every fade is the full length.
        let metric = KerrSchild::new(1.0, 0.90);
        let (field, chart) = outside_pulse(&metric);
        let px = 60.0;
        let emitted = test_screen(px, chart);
        let style = FrontStyle { arcs: true, hide_wound: true };
        let (trail, bands) = trail_frame(&metric, &field, style, px);
        let points: usize = bands.iter().map(|(_, _, p)| p.len()).sum();
        assert!(!bands.is_empty(), "the ring is drawn as bands");
        assert_eq!(trail.len(), 2 * points, "one front vertex and one tail vertex per band point");

        let (mut worst_length, mut closest) = (0.0_f32, 0.0_f32);
        for pair in trail.chunks(2) {
            let (front, tail) = (pair[0].0, pair[1].0);
            let length = (tail - front).length();
            worst_length = worst_length.max((length - Theme::FRONT_TRAIL_PX).abs());
            let (out, back) = ((front - emitted).length(), (tail - emitted).length());
            assert!(
                back < out,
                "a trail on an expanding ring must run back towards the emission event: the front \
                 vertex stands {out:.2} px from it and its tail {back:.2} px"
            );
            closest = closest.max(back / out);
        }
        assert!(
            worst_length < 1e-3,
            "with nothing behind this pulse every tail sits {} px behind its front: worst error \
             {worst_length:.2e} px",
            Theme::FRONT_TRAIL_PX
        );
        println!(
            "{} trail vertices over {} bands: every tail {} px behind its front to \
             {worst_length:.1e} px, and the least inward of them is at {:.3} of its front's \
             distance from the emission event",
            trail.len(),
            bands.len(),
            Theme::FRONT_TRAIL_PX,
            closest
        );
    }

    #[test]
    fn test_a_trail_reaches_only_as_far_back_as_the_gap_to_the_front_behind_it() {
        // The rule the whole design rests on. A fixed 30 px fade on fronts 5 px apart lay six deep
        // and washed the field out, so a fade may occupy only `Theme::FRONT_TRAIL_GAP_FRACTION` of
        // the gap to the front behind it: the ray's own screen speed times the coordinate time
        // between the two emissions, held under `Theme::FRONT_TRAIL_PX`. Both factors are worked
        // out here from the physics and the projection alone - `cartesian_velocity` times the
        // scale, and the difference of two `emitted_t` - and measured against the vertices a real
        // painted frame carries.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let mut emitter =
            Observer::new_with_phi(&metric, "Alice", 0.0, 8.0, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let (mut t, dt) = (0.0, 0.02);
        for _ in 0..30 {
            field.emit_if_due(&metric, &emitter);
            emitter.step(&metric, t, dt);
            field.advance(&metric, dt);
            t += dt;
        }
        let pulses = field.pulses.len();
        assert!(pulses >= 4, "a train of fronts, not one front: {pulses} pulses");
        assert!(
            field.pulses.iter().all(|p| p.rays.iter().all(|ray| ray.alive())),
            "every ray of every pulse is still alive out at r = 8"
        );
        // The deque is in emission order, which is what makes the neighbour at k + 1 the front
        // behind this one. The drawing reads it that way, so the test states it.
        let order: Vec<(usize, f64)> =
            field.pulses.iter().map(|p| (p.index, p.emitted_t)).collect();
        assert!(
            order.windows(2).all(|w| w[0].0 < w[1].0 && w[0].1 < w[1].1),
            "the field's pulses are in emission order: {order:?}"
        );

        let px = 60.0;
        let style = FrontStyle { arcs: true, hide_wound: true };
        let (trail, bands) = trail_frame(&metric, &field, style, px);
        let n = field.pulses[0].rays.len();
        assert_eq!(bands.len(), pulses * n, "one band a segment, so the bands run pulse by pulse");
        assert_eq!(trail.len(), 4 * bands.len(), "two points a band, two vertices a point");

        let (mut worst, mut longest, mut shortest) = (0.0_f32, 0.0_f32, f32::MAX);
        for (b, pair) in trail.chunks(4).enumerate() {
            let (k, i) = (b / n, b % n);
            let gap = gap_behind(&field, k);
            for (end, index) in [(0, i), (2, (i + 1) % n)] {
                let ray = &field.pulses[k].rays[index];
                let speed = ray_speed_px(&metric, ray, px);
                let want = expected_trail_px(speed, gap, ray.t - field.pulses[k].emitted_t);
                let got = (pair[end + 1].0 - pair[end].0).length();
                worst = worst.max((got - want).abs());
                longest = longest.max(got);
                shortest = shortest.min(got);
                assert!(
                    got <= Theme::FRONT_TRAIL_PX + 1e-3,
                    "no fade may pass the ceiling: {got:.3} px on pulse {k}, ray {index}"
                );
            }
        }
        assert!(
            worst < 0.05,
            "every fade is the rule's own length to {worst:.3} px, which it is not"
        );
        assert!(
            longest < Theme::FRONT_TRAIL_PX,
            "on a train of fronts the gap binds rather than the ceiling: longest fade {longest:.2} \
             px against a ceiling of {}",
            Theme::FRONT_TRAIL_PX
        );
        println!(
            "{pulses} fronts 0.1 M of proper time apart at {px} px/M: fades from {shortest:.2} to \
             {longest:.2} px, every one of them the gap rule's own value to {worst:.3} px",
        );

        // And zoomed far out the whole field's fades fall under a pixel, so no mesh is built at
        // all: there is nothing there a viewer could read a direction off.
        let (none, bands_out) = trail_frame(&metric, &field, style, 0.5);
        assert!(!bands_out.is_empty(), "the fronts themselves are still drawn out there");
        assert!(none.is_empty(), "but nothing sub-pixel is laid behind them");
    }

    #[test]
    fn test_a_fade_reaches_no_further_back_than_its_light_has_come_since_the_pulse_left() {
        // A pulse a moment old is a loop a few pixels across hugging its emitter, and the gap to
        // the front behind it is a whole emission interval wide, so the gap rule alone let every
        // ray of it wear the full 30 px: the strip reached back *through* the emission event and
        // fanned out behind the emitter as a flare, over ground the light had never covered. The
        // third bound is the distance the light has come since the pulse was let go - the ray's own
        // screen speed times its own clock less `Pulse::emitted_t` - and it holds the fade to
        // exactly the emission event. The older fronts of the same field stand far enough out that
        // the bound does not reach them, and they are measured here to say so.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let mut emitter =
            Observer::new_with_phi(&metric, "Alice", 0.0, 8.0, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let (mut t, dt) = (0.0, 0.02);
        for _ in 0..30 {
            field.emit_if_due(&metric, &emitter);
            emitter.step(&metric, t, dt);
            field.advance(&metric, dt);
            t += dt;
        }
        // Five steps with nothing sent, so the newest pulse below stands a known interval behind
        // the one before it and the gap rule is wide open; then one pulse, then one short step.
        for _ in 0..5 {
            emitter.step(&metric, t, dt);
            field.advance(&metric, dt);
            t += dt;
        }
        let flight = 0.002;
        field.interval_tau = 0.0;
        assert!(field.emit_if_due(&metric, &emitter), "the fresh pulse goes out");
        field.advance(&metric, flight);

        let pulses = field.pulses.len();
        let fresh = pulses - 1;
        let n = field.pulses[0].rays.len();
        assert!(pulses >= 4, "a train of fronts with the fresh one at its back: {pulses} pulses");
        assert!(
            field.pulses.iter().all(|p| p.rays.iter().all(|ray| ray.alive())),
            "every ray of every pulse is still alive out at r = 8"
        );
        for (k, pulse) in field.pulses.iter().enumerate() {
            let elapsed = pulse.rays[0].t - pulse.emitted_t;
            if k == fresh {
                assert!(
                    (elapsed - flight).abs() < 1e-9,
                    "the fresh pulse has been in flight {elapsed:.4} M, which is the short step"
                );
            } else {
                assert!(
                    elapsed >= 5.0 * dt,
                    "every older pulse has been in flight at least the five idle steps: \
                     pulse {k} has {elapsed:.4} M"
                );
            }
        }

        // Deep enough that the gap rule on its own would hand every ray of the fresh pulse the
        // whole ceiling, which is what makes the bound under test the only thing holding it back.
        let px = 4000.0;
        let gap = gap_behind(&field, fresh).expect("the fresh pulse has a front behind it");
        let slowest = (0..n)
            .map(|i| ray_speed_px(&metric, &field.pulses[fresh].rays[i], px))
            .fold(f32::MAX, f32::min);
        assert!(
            Theme::FRONT_TRAIL_GAP_FRACTION * slowest * gap as f32 > Theme::FRONT_TRAIL_PX,
            "the gap rule alone would give the ceiling here: {:.0} px at the slowest ray",
            Theme::FRONT_TRAIL_GAP_FRACTION * slowest * gap as f32
        );

        let style = FrontStyle { arcs: true, hide_wound: true };
        let (trail, bands) = trail_frame(&metric, &field, style, px);
        assert_eq!(bands.len(), pulses * n, "one band a segment, so the bands run pulse by pulse");
        assert_eq!(trail.len(), 4 * bands.len(), "two points a band, two vertices a point");

        let (mut worst_fresh, mut longest_fresh, mut worst_old) = (0.0_f32, 0.0_f32, 0.0_f32);
        for (b, pair) in trail.chunks(4).enumerate() {
            let (k, i) = (b / n, b % n);
            for (end, index) in [(0, i), (2, (i + 1) % n)] {
                let ray = &field.pulses[k].rays[index];
                let elapsed = (ray.t - field.pulses[k].emitted_t) as f32;
                let flown = ray_speed_px(&metric, ray, px) * elapsed;
                let got = (pair[end + 1].0 - pair[end].0).length();
                if k == fresh {
                    // The slack is the probe's: `screen_velocity` reads its speed off a finite
                    // difference of two f32 screen positions, where this test strikes the exact
                    // Jacobian, so the two agree to a twentieth of a pixel rather than to the bit.
                    assert!(
                        got <= flown + 0.05,
                        "the fresh pulse's fade may not outrun its own light: {got:.3} px against \
                         {flown:.3} px covered, on ray {index}"
                    );
                    worst_fresh = worst_fresh.max((got - flown).abs());
                    longest_fresh = longest_fresh.max(got);
                } else {
                    // Every older front is far enough out that the ceiling still binds, so the new
                    // bound leaves those fades exactly where they were.
                    worst_old = worst_old.max((got - Theme::FRONT_TRAIL_PX).abs());
                }
            }
        }
        assert!(
            worst_fresh < 0.05,
            "every fade on the fresh pulse is the distance its own light has covered, to \
             {worst_fresh:.3} px"
        );
        assert!(
            longest_fresh < 0.5 * Theme::FRONT_TRAIL_PX,
            "and that is far under the ceiling the gap rule alone would have given: \
             {longest_fresh:.2} px against {}",
            Theme::FRONT_TRAIL_PX
        );
        assert!(
            worst_old < 0.05,
            "while the older fronts still wear the full ceiling, to {worst_old:.3} px"
        );
        println!(
            "at {px} px/M a pulse {flight} M old wears {longest_fresh:.2} px of fade, the distance \
             its light has covered to {worst_fresh:.1e} px, while the {} fronts behind it keep the \
             full {} px",
            pulses - 1,
            Theme::FRONT_TRAIL_PX
        );
    }

    #[test]
    fn test_a_trail_starts_at_its_own_bands_colour_and_fades_to_a_transparent_vertex() {
        // Full line brightness to nothing, which is what the fade was asked for: the head vertices
        // are the band's own points in the band's own colour - the very colour and opacity the line
        // beside them is stroked in - and the tail vertices are `Color32::TRANSPARENT` rather than
        // that colour at alpha 0. egui carries vertex colours premultiplied, and interpolating
        // towards an unpremultiplied (R, G, B, 0) would put a bright fringe along the whole strip.
        let metric = KerrSchild::new(1.0, 0.90);
        let (field, _) = outside_pulse(&metric);
        let style = FrontStyle { arcs: true, hide_wound: true };
        let (trail, bands) = trail_frame(&metric, &field, style, 60.0);

        let mut at = 0usize;
        for (colour, _, points) in bands.iter() {
            assert_eq!(colour.a(), Theme::SHIFT_ALPHA, "every band at the one opacity");
            for point in points.iter() {
                let (head, tail) = (trail[at], trail[at + 1]);
                assert_eq!(head.0, *point, "the head of a trail is the drawn point itself");
                assert!(tail.0.is_finite(), "no tail vertex is a NaN: {:?}", tail.0);
                assert_eq!(head.1, *colour, "the head is its own band's colour, to the bit");
                assert_eq!(tail.1, Color32::TRANSPARENT, "the far edge of the fade is nothing");
                at += 2;
            }
        }
        assert_eq!(at, trail.len(), "every trail vertex belongs to a band that was drawn");
        println!(
            "{} trail vertices, each head at its band's own colour and alpha {}, each tail at \
             Color32::TRANSPARENT",
            trail.len(),
            Theme::SHIFT_ALPHA
        );
    }

    #[test]
    fn test_nothing_that_draws_no_line_casts_a_trail() {
        // Two ways a piece of front goes undrawn, and neither may leave a fade standing where its
        // line is not: the arcs switched off altogether, and a segment with a dead endpoint or one
        // cut for winding. Every segment that *is* drawn carries a fade, the frozen family on r-
        // included - there is one pass now, and one rule.
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);
        let mut alice =
            Observer::new_with_phi(&metric, "Alice", 0.0, 1.2, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField::default();
        let mut t = 0.0;
        for _ in 0..40 {
            field.emit_if_due(&metric, &alice);
            alice.step(&metric, t, 0.02);
            field.advance(&metric, 0.02);
            t += 0.02;
        }

        // What the drawing rules say this field is: the segments a band is laid for and the ones
        // dropped for a dead endpoint or for winding. The counts are what gives the test its teeth
        // - both cases have to occur, and the drawn ones have to include pairs of the frozen family
        // inside r+, which used to be held back and now are not. The scale is deliberately deep:
        // this field's fronts are a hundredth of an M apart in the deep interior, and the claim
        // being made here is about what is drawn rather than about what is too small to see, so
        // every drawn band has to be over `TRAIL_MIN_PX`.
        let px = 6000.0;
        let r_plus = metric.outer_horizon();
        let (mut drawn, mut dropped, mut frozen_pairs) = (0usize, 0usize, 0usize);
        let mut faintest = f32::MAX;
        for (k, pulse) in field.pulses.iter().enumerate() {
            if !pulse.rays.iter().any(|ray| ray.alive()) {
                continue;
            }
            let n = pulse.rays.len();
            let gap = gap_behind(&field, k);
            let frozen: Vec<bool> = pulse
                .rays
                .iter()
                .map(|ray| ray.alive() && ray.r < r_plus && ray.frozen(&metric))
                .collect();
            for i in 0..n {
                let j = (i + 1) % n;
                let dead = !pulse.rays[i].alive() || !pulse.rays[j].alive();
                let wound = !dead
                    && (pulse.rays[j].phi - pulse.rays[i].phi).abs() > MAX_RESOLVED_WINDING;
                if dead || wound {
                    dropped += 1;
                    continue;
                }
                drawn += 1;
                if frozen[i] && frozen[j] {
                    frozen_pairs += 1;
                }
                let ends = [i, j].map(|end| {
                    let ray = &pulse.rays[end];
                    expected_trail_px(ray_speed_px(&metric, ray, px), gap, ray.t - pulse.emitted_t)
                });
                faintest = faintest.min(ends[0].max(ends[1]));
            }
        }
        assert!(dropped > 0, "the field has segments that draw no line at all");
        assert!(frozen_pairs > 0, "and segments of the frozen family inside r+");
        assert!(drawn > frozen_pairs, "and segments of the crossing family");
        assert!(
            faintest >= TRAIL_MIN_PX,
            "at {px} px/M every drawn band is over the sub-pixel cut, so every one of them must \
             carry a fade: the faintest is {faintest:.2} px"
        );

        let style = FrontStyle { arcs: true, hide_wound: true };
        let (trail, bands) = trail_frame(&metric, &field, style, px);
        // Every band of a front is at the one stroke width, the frozen family's included.
        assert!(
            bands.iter().all(|(_, width, _)| *width == 1.2),
            "one pass, one width: {:?}",
            bands
                .iter()
                .map(|(_, width, _)| width.to_bits())
                .collect::<std::collections::HashSet<_>>()
        );
        let head: usize = bands.iter().map(|(_, _, points)| points.len()).sum();
        assert_eq!(
            trail.len(),
            2 * head,
            "a trail vertex pair for every point of every drawn band, and for nothing else: \
             {drawn} drawn segments, {frozen_pairs} of them frozen pairs, and {dropped} dropped"
        );
        for (pair, point) in trail.chunks(2).zip(bands.iter().flat_map(|(_, _, p)| p)) {
            assert_eq!(pair[0].0, *point, "each trail head sits on its own band's point");
        }

        // And with the arcs off there are no lines at all, so there is nothing to annotate and no
        // mesh is built.
        let points_only = FrontStyle { arcs: false, hide_wound: true };
        let (none, bands_off) = trail_frame(&metric, &field, points_only, px);
        assert!(bands_off.is_empty(), "points only: no bands");
        assert!(none.is_empty(), "and no trail mesh either, not even an empty one");
        println!(
            "{drawn} drawn segments, {frozen_pairs} of them frozen pairs, carry {} trail \
             vertices; {dropped} dropped segments carry none, and the points-only frame carries \
             none at all",
            trail.len()
        );
    }

    #[test]
    fn test_a_segment_is_drawn_in_bands_of_the_gain_along_it() {
        // A segment from a ray at gain 1 to a ray at gain 1e5: the whole ramp in one segment, which
        // is what the pair being torn apart carries - one ray settling onto r- and climbing like
        // exp(kappa_- t), its neighbour crossing at the gain it was born with. Painted in one
        // colour that segment said its deep-red end had gained a hundred thousandfold. Instead the
        // polyline is cut into bands of at most `FRONT_BAND_DECADES` of gain, each coloured at the
        // gain interpolated to its own midpoint in the same loop coordinate `segment_arc` walks the
        // position along, so the ramp runs *along* the segment from one ray to the other.
        let arc: Vec<Pos2> = (0..=100).map(|k| Pos2::new(k as f32, 0.0)).collect();
        let bands = banded_segment(arc.clone(), 1.0, 1e5);
        let n = bands.len();
        assert_eq!(n, 20, "five decades in quarter-decade bands: {n} bands");
        assert!(n <= MAX_FRONT_BANDS);

        // Every band's gain is log10 interpolated to the midpoint of its own s-range, and every
        // band covers exactly the pieces of the arc in that range.
        for (b, (points, gain)) in bands.iter().enumerate() {
            let (start, end) = (b * 100 / n, (b + 1) * 100 / n);
            let s_mid = 0.5 * ((start + end) as f64) / 100.0;
            let expect = 10.0_f64.powf(5.0 * s_mid);
            assert!((gain / expect - 1.0).abs() < 1e-12, "band {b}: {gain} against {expect}");
            assert_eq!(points.len(), end - start + 1, "band {b} covers its own pieces");
        }

        // The two ends of the segment come out at the two ends of the ramp, and each band's colour
        // is `Theme::front_colour` of the gain at that band's midpoint - nothing else.
        let first = Theme::front_colour(bands[0].1, Theme::SHIFT_ALPHA);
        let last = Theme::front_colour(bands[n - 1].1, Theme::SHIFT_ALPHA);
        assert_eq!(
            first,
            Theme::front_colour(10.0_f64.powf(5.0 * 0.025), Theme::SHIFT_ALPHA),
            "the first band is coloured at the gain of its own midpoint, s = 0.025"
        );
        assert!(
            bands[0].1 < 10.0_f64.powf(Theme::FRONT_LOG_ORANGE),
            "the first band is still on the deep-red leg of the ramp: gain {}",
            bands[0].1
        );
        assert!(first.r() > first.b(), "and reads red: {first:?}");
        assert!(
            bands[n - 1].1 > 10.0_f64.powf(Theme::FRONT_LOG_BLUE),
            "the last band is past the blue stop: gain {}",
            bands[n - 1].1
        );
        assert!(last.b() > last.r(), "and reads violet: {last:?}");

        // No gaps: consecutive bands share their boundary point, and the bands laid end to end are
        // the original arc in order with nothing left out and nothing left uncoloured.
        for pair in bands.windows(2) {
            assert_eq!(
                pair[0].0.last(),
                pair[1].0.first(),
                "consecutive bands must share their boundary point"
            );
        }
        let mut union: Vec<Pos2> = vec![bands[0].0[0]];
        for (points, _) in bands.iter() {
            union.extend_from_slice(&points[1..]);
        }
        assert_eq!(union, arc, "the bands are the arc, in order");
        println!(
            "a segment from gain 1 to gain 1e5 over 100 pieces is drawn as {n} bands, the first at \
             gain {:.4} ({:?}) and the last at {:.4e} ({:?})",
            bands[0].1,
            (first.r(), first.g(), first.b()),
            bands[n - 1].1,
            (last.r(), last.g(), last.b()),
        );

        // And the common case costs nothing: both ends at the same gain is one band over the whole
        // arc, drawn exactly as it was before there were bands at all.
        let flat = banded_segment(arc.clone(), 7.0, 7.0);
        assert_eq!(flat.len(), 1, "equal gains at the two ends is a single band");
        assert_eq!(flat[0].0, arc, "and it is the whole arc");
        assert!((flat[0].1 - 7.0).abs() < 1e-12, "at that gain: {}", flat[0].1);
    }

    #[test]
    fn test_a_segment_wound_past_a_full_turn_is_cut_and_its_two_ends_marked() {
        use crate::physics::observer::{Observer, WorldlineParams};
        let metric = KerrSchild::new(1.0, 0.90);

        // First, one pulse with the winding put in by hand, so that exactly one pair of the loop is
        // past a whole turn and every other pair is well inside one. A closed front cannot have a
        // single wound pair and nothing else - the azimuth differences around the loop sum to zero
        // - so the three turns of the one segment are paid back over the other eleven, a sixth of a
        // turn each, which is what a front in the deep interior looks like anyway.
        let emitter =
            Observer::new_with_phi(&metric, "Alice", 0.0, 4.5, 0.0, 0.0, WorldlineParams::default());
        let mut field = SignalField { rays_per_pulse: 12, ..Default::default() };
        field.emit_if_due(&metric, &emitter);
        assert_eq!(field.pulses.len(), 1, "one pulse, let go at r = 4.5");
        let n = field.pulses[0].rays.len();
        assert!(field.pulses[0].launched_with(12), "{n} rays");
        assert!(field.pulses[0].rays.iter().all(|ray| ray.alive()), "a fresh pulse is all alive");
        // Which pair carries the winding does not matter: every dot on a front is the same dot,
        // so the two the cut leaves are the only two the frame gains.
        let i0 = 0;
        let tau = std::f64::consts::TAU;
        let mut phi = 0.0;
        for k in 0..n {
            field.pulses[0].rays[k].phi = phi;
            phi += if k == i0 { 3.0 * tau } else { -3.0 * tau / ((n - 1) as f64) };
        }
        let wound: Vec<usize> = (0..n)
            .filter(|&i| {
                let j = (i + 1) % n;
                (field.pulses[0].rays[j].phi - field.pulses[0].rays[i].phi).abs()
                    > MAX_RESOLVED_WINDING
            })
            .collect();
        assert_eq!(wound, vec![i0], "exactly one pair is past a full turn, by construction");

        let (lines_kept, circles_kept) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_cut, circles_cut) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: true });
        println!(
            "a hand-built pulse of {n} rays with one pair three turns apart: {lines_kept} \
             polylines and {circles_kept} circles drawn whole, {lines_cut} and {circles_cut} with \
             the wound segment cut"
        );
        assert_eq!(lines_kept, n, "every live segment of a fresh pulse is one band, so one polyline");
        assert_eq!(lines_cut, lines_kept - 1, "the wound segment, and only it, is not drawn");
        assert_eq!(
            circles_cut,
            circles_kept + 2,
            "and its two rays are marked as dots, so the cut reads as a gap with ends"
        );

        // Now the same cut on a front nobody built by hand. A whole light cone is let go at r = 4.5
        // at a = 0.90 and integrated for 40 M. What winds a pair there is the pair straddling a
        // photon-orbit critical angle: the ray inside it hangs on the orbit and then settles onto
        // r-, where it co-rotates at Omega_- = 0.9 per M for ever, while its neighbour outside it
        // is long gone outward, so the raw difference between them grows without bound. Those are
        // exactly the segments the interpolation cannot speak for, and they are the ones dropped.
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &emitter);
        let dt = 0.02;
        for _ in 0..2000 {
            field.advance(&metric, dt);
        }
        let pulse = &field.pulses[0];
        let n = pulse.rays.len();

        // The gains the drawing colours by, computed here exactly as `draw_signal_field` computes
        // them, so that the bands counted are the bands drawn.
        let raindrop = GeodesicState::new_infall(&metric, 0.0, 12.0, 1.0, 0.0);
        let u_emit = {
            let (ut, ur, up) = raindrop.derivatives(&metric, pulse.emitted_r);
            [ut, ur, up]
        };
        let gain_of = |i: usize| {
            let ray = &pulse.rays[i];
            let (ut, ur, up) = raindrop.derivatives(&metric, ray.r);
            ray.gain_between(&metric, pulse.emitted_r, &u_emit, &[ut, ur, up])
        };
        let to_screen =
            |(x, y): (f64, f64)| Pos2::new(200.0 + 60.0 * x as f32, 200.0 - 60.0 * y as f32);

        let (mut wound, mut bands_lost, mut largest, mut live_pairs) = (0usize, 0usize, 0.0f64, 0);
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            live_pairs += 1;
            let d_phi = pulse.rays[j].phi - pulse.rays[i].phi;
            if d_phi.abs() <= MAX_RESOLVED_WINDING {
                continue;
            }
            wound += 1;
            largest = largest.max(d_phi.abs());
            let arc = segment_arc(
                &metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                &to_screen,
            );
            // Each band of a drawn segment is one polyline, so this is what the cut removes.
            bands_lost += banded_segment(arc, gain_of(i), gain_of(j)).len();
        }
        assert!(
            wound > 0,
            "within 40 M of a pulse let go at r = 4.5 at a = 0.90, some live pair must have wound \
             past a full turn; none of the {live_pairs} live pairs did"
        );

        let (lines_kept, _) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: false });
        let (lines_cut, _) =
            count_front_shapes(&metric, &field, FrontStyle { arcs: true, hide_wound: true });
        println!(
            "after 40 M a pulse of {n} rays let go at r = 4.5 has {live_pairs} live neighbouring \
             pairs, {wound} of them past a full turn (the largest {:.2} turns); the drawing goes \
             from {lines_kept} polylines to {lines_cut}, the {bands_lost} bands those segments \
             were drawn in",
            largest / tau
        );
        assert_eq!(
            lines_kept - lines_cut,
            bands_lost,
            "the cut must remove exactly the wound segments and nothing else"
        );
    }

    #[test]
    fn test_a_segment_between_two_frozen_rays_is_drawn_along_r_minus() {
        // Two rays of the same front frozen on the Cauchy horizon, most of a radian apart in
        // azimuth: the front between them lies on r-, and that is what has to be drawn. The chord
        // between their two screen positions does not - it cuts the chord of the circle, which at
        // this separation is well inside r- and, deeper in, inside the ring itself.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let to_screen = |(x, y): (f64, f64)| Pos2::new(x as f32, y as f32);
        let circle = (rm * rm + metric.a * metric.a).sqrt();

        let arc = segment_arc(&metric, (rm, 0.3), (rm, 1.5), &to_screen);
        // 1.2 radians in pieces of at most `MAX_ARC_STEP`, so 24 or 25 of them.
        assert!(arc.len() >= 25 && arc.len() <= 26, "{} points", arc.len());
        for pair in arc.windows(2) {
            let step = (pair[1] - pair[0]).length() as f64;
            assert!(step <= MAX_ARC_STEP * circle * 1.01, "a piece spans {step}");
        }
        let worst = arc
            .iter()
            .map(|p| (embedded_radius(*p) - circle).abs())
            .fold(0.0f64, f64::max);

        // The chord for comparison: its midpoint is the sagitta of the arc inside the circle.
        let chord_mid = Pos2::new(
            0.5 * (arc[0].x + arc[arc.len() - 1].x),
            0.5 * (arc[0].y + arc[arc.len() - 1].y),
        );
        let sagitta = circle - embedded_radius(chord_mid);
        println!(
            "an arc of {} pieces over 1.2 rad of r- stays within {worst:.3e} M of the r- circle \
             (rho = {circle:.4}); the chord it replaces dips {sagitta:.4} M inside it",
            arc.len() - 1
        );
        // 1e-6 rather than round-off: the drawn points are f32 screen coordinates.
        assert!(worst < 1e-6, "the drawn arc must lie on the r- circle: {worst}");
        assert!(sagitta > 0.15, "and the chord it replaces must not: {sagitta}");

        // A segment whose ends are close in azimuth is one straight piece, as it always was: the
        // subdivision costs nothing where it buys nothing.
        let short = segment_arc(&metric, (rm, 0.0), (rm, 0.04), &to_screen);
        assert_eq!(short.len(), 2, "a short segment is still a single line");

        // And a segment deep inside r-, where the two rays are far apart in azimuth and a chord
        // would cut through the disk inside the ring: every drawn point stays outside the ring.
        let ring = metric.a;
        let deep = segment_arc(&metric, (0.05, 0.0), (0.5, 2.5), &to_screen);
        let closest = deep
            .iter()
            .map(|p| embedded_radius(*p))
            .fold(f64::INFINITY, f64::min);
        let chord_closest = {
            let mid = Pos2::new(
                0.5 * (deep[0].x + deep[deep.len() - 1].x),
                0.5 * (deep[0].y + deep[deep.len() - 1].y),
            );
            embedded_radius(mid)
        };
        println!(
            "a segment from (r = 0.05, phi = 0) to (r = 0.5, phi = 2.5) is drawn in {} pieces and \
             never comes closer to the centre than rho = {closest:.4}, against the ring at \
             rho = {ring:.4}; the midpoint of the chord it replaces sits at rho = \
             {chord_closest:.4}, inside the ring",
            deep.len() - 1
        );
        assert!(closest > ring, "the drawn front must stay outside the ring: {closest}");
        assert!(chord_closest < ring, "whereas the chord does not: {chord_closest}");
    }

    /// The azimuth of a chart point: x + i y = (r + i a) e^{i phi}, so the argument is phi plus
    /// atan(a/r), and at fixed r a difference of arguments is a difference of phi.
    fn chart_azimuth((x, y): (f64, f64)) -> f64 {
        y.atan2(x)
    }

    /// The total azimuth a polyline of chart points sweeps, signed, counting the turns rather than
    /// folding them away: each piece is at most `MAX_ARC_STEP`, so the piece's own difference is
    /// unambiguous, and the sum of the pieces telescopes to the whole swept angle.
    fn swept_azimuth(points: &[(f64, f64)]) -> f64 {
        let two_pi = 2.0 * std::f64::consts::PI;
        points
            .windows(2)
            .map(|pair| {
                let d = chart_azimuth(pair[1]) - chart_azimuth(pair[0]);
                d - two_pi * (d / two_pi).round()
            })
            .sum()
    }

    #[test]
    fn test_a_segment_that_has_wound_is_drawn_the_long_way_round() {
        // Two neighbouring rays of one front, at the same radius, whose integrated azimuths differ
        // by 2 pi + 0.3: one of them has lapped the other round the hole. Both left the emission
        // event at the emitter's azimuth and both carry phi as a continuously integrated
        // coordinate, so that difference is not a representative of an angle - it is the winding
        // between the pair, and the piece of front between them is one whole turn plus 0.3 rad.
        // All of it has to be drawn.
        let metric = KerrSchild::new(1.0, 0.90);
        let rm = metric.inner_horizon();
        let two_pi = 2.0 * std::f64::consts::PI;
        let span = two_pi + 0.3;

        // The chart points `segment_arc` embeds, in full f64, recorded on the way through the
        // projection: what is measured here is the curve the function generates, not the f32
        // screen coordinates it is finally rounded to.
        let chart = std::cell::RefCell::new(Vec::new());
        let to_screen = |(x, y): (f64, f64)| {
            chart.borrow_mut().push((x, y));
            Pos2::new(x as f32, y as f32)
        };

        let arc = segment_arc(&metric, (rm, 0.3), (rm, 0.3 + span), &to_screen);
        let points = chart.into_inner();
        assert_eq!(points.len(), arc.len(), "one chart point embedded per drawn point");

        // 2 pi + 0.3 = 6.583 rad in pieces of at most `MAX_ARC_STEP` = 0.05, so 132 of them.
        let pieces = (span / MAX_ARC_STEP).ceil() as usize;
        assert_eq!(pieces, 132);
        assert_eq!(
            arc.len(),
            pieces + 1,
            "a segment of {span:.4} rad is {pieces} pieces of at most {MAX_ARC_STEP} rad"
        );

        let swept = swept_azimuth(&points);
        println!(
            "a segment whose two rays differ by 2 pi + 0.3 = {span:.6} rad is drawn as {} pieces \
             sweeping {swept:.15} rad: one whole turn of the hole plus 0.3",
            arc.len() - 1
        );
        assert!(
            (swept - span).abs() < 1e-9,
            "the drawn arc must sweep the raw difference {span}, not {swept}. Folding it into \
             [-pi, pi] gives 2 pi + 0.3 - 2 pi = 0.3 - the same azimuth reached the other way \
             round the hole, which is -(2 pi - 0.3) of travel - and would have drawn 7 pieces of \
             a short arc across the near side of the picture instead of the {} pieces of the turn \
             the front actually made",
            arc.len() - 1
        );

        // Every piece is still on the r- circle and still no wider than `MAX_ARC_STEP`: winding
        // changes how far round the arc goes, not what it is drawn along.
        let circle = (rm * rm + metric.a * metric.a).sqrt();
        for pair in points.windows(2) {
            let d = chart_azimuth(pair[1]) - chart_azimuth(pair[0]);
            let d = d - two_pi * (d / two_pi).round();
            assert!(d.abs() <= MAX_ARC_STEP * (1.0 + 1e-12), "a piece spans {d} rad");
        }
        for point in points.iter() {
            let rho = (point.0 * point.0 + point.1 * point.1).sqrt();
            assert!((rho - circle).abs() < 1e-12, "the arc must stay on r-: {rho} vs {circle}");
        }
    }

    #[test]
    fn test_a_wound_front_of_a_real_pulse_is_drawn_over_its_raw_azimuth_difference() {
        // The same statement about a front nobody built by hand.
        //
        // A whole light cone is let go at r = 2.0 at a = 0.90 and integrated for 60 M of
        // coordinate time at the frame step. What winds a front there is the unstable circular
        // photon orbits outside r+ (at a = 0.90 the equatorial ones sit at r = 1.56 prograde and
        // r = 3.91 retrograde): a ray let go on very nearly the critical impact parameter hangs at
        // one of them for tens of M, going round and round, while the neighbour it was emitted
        // next to has long since escaped outward or spiralled in. Their azimuths are then several
        // whole turns apart, and the segment of front between them is that whole spiral. It is one
        // continuous piece of the null surface, however many times it goes round.
        //
        // (Emission from inside r+ does not do this: the rays that wind fastest there are the ones
        // being carried onto the ring, and they reach it and die within a couple of M. Over 20 M
        // of a pulse let go at r = 1.0, no pair of *live* neighbours ever gets past 1.1 rad.)
        let metric = KerrSchild::new(1.0, 0.90);
        let alice = Observer::new_with_phi(
            &metric,
            "Alice",
            0.0,
            2.0,
            0.0,
            0.0,
            crate::physics::observer::WorldlineParams::default(),
        );
        let mut field = SignalField::default();
        field.emit_if_due(&metric, &alice);
        assert_eq!(field.pulses.len(), 1, "one pulse, let go at r = 2.0");
        let dt = 0.017;
        let steps = 3530; // 60.01 M of coordinate time
        for _ in 0..steps {
            field.advance(&metric, dt);
        }

        let pulse = &field.pulses[0];
        let n = pulse.rays.len();
        let pi = std::f64::consts::PI;
        let mut wound: Vec<(usize, f64)> = Vec::new();
        let mut live_pairs = 0;
        for i in 0..n {
            let j = (i + 1) % n;
            if !pulse.rays[i].alive() || !pulse.rays[j].alive() {
                continue;
            }
            live_pairs += 1;
            let raw = pulse.rays[j].phi - pulse.rays[i].phi;
            if raw.abs() > pi {
                wound.push((i, raw));
            }
        }
        let largest = wound.iter().map(|&(_, d)| d.abs()).fold(0.0f64, f64::max);
        println!(
            "after {:.2} M a pulse of {n} rays let go at r = 2.0 at a = 0.90 has {live_pairs} live \
             neighbouring pairs, {} of them more than half a turn apart; the largest raw |d phi| \
             is {largest:.3} rad ({:.2} turns)",
            (steps as f64) * dt,
            wound.len(),
            largest / (2.0 * pi)
        );
        assert!(
            !wound.is_empty(),
            "a ray hung on a circular photon orbit must wind past half a turn away from its \
             neighbour within 60 M; none of the {live_pairs} live pairs did"
        );

        // Each of them is drawn over exactly that raw difference. The fold would have replaced it
        // with the same angle mod 2 pi, which is a short arc across the near side of the picture
        // and not the spiral the front is.
        for &(i, raw) in wound.iter() {
            let j = (i + 1) % n;
            let chart = std::cell::RefCell::new(Vec::new());
            let to_screen = |(x, y): (f64, f64)| {
                chart.borrow_mut().push((x, y));
                Pos2::new(x as f32, y as f32)
            };
            let arc = segment_arc(
                &metric,
                (pulse.rays[i].r, pulse.rays[i].phi),
                (pulse.rays[j].r, pulse.rays[j].phi),
                &to_screen,
            );
            let points = chart.into_inner();
            assert_eq!(points.len(), arc.len());
            // The two ends are at different radii, and arg = phi + atan(a/r), so the swept
            // argument is the swept phi plus the change in that offset across the segment.
            let offset = (metric.a / pulse.rays[j].r).atan() - (metric.a / pulse.rays[i].r).atan();
            let swept = swept_azimuth(&points) - offset;
            let folded = raw - 2.0 * pi * (raw / (2.0 * pi)).round();
            assert!(
                (swept - raw).abs() < 1e-9,
                "segment {i} must be drawn over its raw difference {raw}, not {swept}; the fold \
                 would have drawn {folded} instead"
            );
            assert!(
                (folded - raw).abs() > 1.0,
                "segment {i}: the raw {raw} and the folded {folded} must be the two different \
                 pictures this test is about"
            );
            // And the piece count is the span, not the fold: pieces of at most `MAX_ARC_STEP`.
            let pieces = (raw.abs() / MAX_ARC_STEP).ceil().min(MAX_ARC_PIECES as f64) as usize;
            assert_eq!(arc.len(), pieces + 1, "segment {i} of {raw} rad in {} pieces", arc.len() - 1);
        }
    }
}
