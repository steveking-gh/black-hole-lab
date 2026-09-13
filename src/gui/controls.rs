use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode, ObserverPair, WorldlineParams};
use crate::physics::wavefront::{Endpoint, RAYS_PER_PULSE, SignalField, SignalPair};

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

/// The two transmissions a canvas draws: Alice's, which Bob receives, and Bob's, which Alice
/// receives. They travel together because every drawing path needs both, and because the pair is
/// one idea - the same field code run once in each direction, so that the user can see both what
/// reaches Bob and what stops reaching Alice.
///
/// There is no "show" flag on either of them any more. Whether a transmission is drawn is whether
/// it exists: an observer whose "Transmit Signal" box is unticked, or who is not in the simulation
/// at all, has an empty field, and an empty field draws nothing without being asked not to.
#[derive(Clone, Copy)]
pub struct SignalViews<'a> {
    /// Alice's transmission, emitted by Alice and received by Bob.
    pub alice: &'a SignalField,
    /// Bob's transmission, emitted by Bob and received by Alice.
    pub bob: &'a SignalField,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepMode {
    Time,     // Fixed Δt
    Distance, // Fixed Δr (km)
}

/// Everything one OBSERVER card asks for. Two of these hang off `AppControls`, one per observer,
/// and they are identical in shape: the two observers are the same idea run twice, so there is one
/// description of what a card holds rather than a Bob-shaped set of fields and an Alice-shaped one.
///
/// None of it is read from the observer. A card is a standing request - what to build the next
/// time this observer is dropped - and the observer, once built, carries its own copy of the
/// constants in its geodesic state. `energy`, `l_ang` and `outgoing_start` therefore take effect at
/// the next drop, exactly as `delta_t_delay` does; `enabled` and `transmit` take effect at once.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObserverSettings {
    /// Whether this observer is in the simulation at all. Unticked, they are not stepped, not
    /// drawn in any view, transmit nothing, receive nothing and are received by nobody.
    pub enabled: bool,
    /// Whether they broadcast pulses. Unticked, their field is dropped and stays empty, so nothing
    /// of theirs is drawn and the other observer records no arrival from them.
    pub transmit: bool,
    /// Coordinate time after the drop at which this observer is released; until then they hover at
    /// the drop radius as a static observer.
    pub delta_t_delay: f64,
    /// Conserved energy per unit mass E = -u_t of the dropped worldline.
    pub energy: f64,
    /// Conserved axial angular momentum per unit mass L = u_phi, in units of M.
    pub l_ang: f64,
    /// Start on the outgoing root of r^4 (dr/dtau)^2 = R(r).
    pub outgoing_start: bool,
}

impl ObserverSettings {
    /// A raindrop (E = 1, L = 0, ingoing) released `delta_t_delay` after the drop.
    fn raindrop(delta_t_delay: f64) -> Self {
        Self {
            enabled: true,
            transmit: true,
            delta_t_delay,
            energy: 1.0,
            l_ang: 0.0,
            outgoing_start: false,
        }
    }

    /// The constants of motion this card is asking for.
    fn worldline_params(&self) -> WorldlineParams {
        WorldlineParams::new(self.energy, self.l_ang, self.outgoing_start)
    }

    /// The observer this card asks for, dropped at `DROP_RADIUS` on the clock's reading `start_t`
    /// and released `delta_t_delay` of coordinate time later, or None when the card is unticked.
    fn dropped(
        &self,
        metric: &KerrSchild,
        name: &str,
        start_phi: f64,
        start_t: f64,
    ) -> Option<Observer> {
        self.enabled.then(|| {
            Observer::new_with_phi(
                metric,
                name,
                start_t,
                DROP_RADIUS,
                start_t + self.delta_t_delay,
                start_phi,
                self.worldline_params(),
            )
        })
    }
}

#[derive(Debug, Clone)]
pub struct AppControls {
    pub is_playing: bool,
    pub step_mode: StepMode,
    pub step_size: f64,
    /// Playback rate while playing: coordinate time (units of M) per wall-clock second.
    pub play_speed: f64,
    pub step_distance_km: f64,
    /// How many rays a newly emitted pulse carries: the sampling of the emitter's light cone, at
    /// alpha = 2 pi i / n, and so the resolution of every wavefront sent from now on.
    ///
    /// It is a standing request, like the worldline constants on an observer card: pushed into both
    /// transmissions on every step through `SignalPair::set_rays_per_pulse`, read by
    /// `SignalField::emit_if_due` and by nothing else. Pulses already in flight keep the count they
    /// were emitted with, because their rays are the null geodesics that were launched.
    pub rays_per_pulse: usize,
    /// Whether the segments of a wavefront between neighbouring rays are drawn at all: on, each is
    /// the curve linear in (r, phi) between its two rays; off, only the rays themselves are drawn,
    /// one dot per calculated point and nothing between them.
    ///
    /// A drawing choice and nothing else: the reception test interpolates in (r, phi) along the
    /// same segments either way, so this cannot move an arrival or change a measured shift.
    /// Default on, because on is the drawing that matches what is being detected. Like every
    /// other control it survives Reset and Drop Observers, which rebuild the run and not the panel.
    pub draw_front_arcs: bool,
    /// Whether the segments whose two rays have wound more than a full turn apart are dropped from
    /// the drawing instead of being interpolated across. See
    /// `spatial_canvas::MAX_RESOLVED_WINDING`.
    ///
    /// Default on. Such a pair straddles a photon-orbit critical angle, the real front between them
    /// is pinned on that orbit, and what the linear interpolation draws instead is a spiral whose
    /// turns are spread evenly over every radius between the two rays and therefore drift outward
    /// across r₊ as the run goes on, which nothing physical does. Like `draw_front_arcs` it is a
    /// drawing choice and cannot move an arrival or change a measured shift, and like every other
    /// control it survives Reset and Drop Observers.
    pub hide_wound_segments: bool,
    /// Draw the animated raindrop flow (the Painlevé-Gullstrand / Doran river) on the
    /// equatorial view.
    pub show_river: bool,
    pub show_streamlines: bool,
    /// Alice's card: whether she is in the simulation, whether she transmits, and the worldline
    /// the next drop puts her on.
    pub alice: ObserverSettings,
    /// Bob's card, identical in shape to Alice's.
    pub bob: ObserverSettings,
    pub show_theory_modal: bool,
    pub use_km: bool,
    pub frame_of_ref: ReferenceFrame,
    pub font_scale: f32,
    /// Standing request from the panel for the (t, r) view to go back to where it starts.
    ///
    /// Set by every control that puts the simulation clock back to zero, and consumed once, in
    /// `SpacetimeApp::ui`, through `take_view_reset`. The canvas is panned in time by dragging it,
    /// and that pan is an offset from the *current* clock; putting the clock back to zero while
    /// leaving the pan standing left the user looking at an empty stretch of diagram above or
    /// below the run they had just restarted, with no obvious way back to it.
    ///
    /// It is a request rather than a direct write because the panel is handed the physics - the
    /// metric, the observers, the two fields and the clock - and not the canvases. Putting the
    /// canvases in its hands as well, so that one button could reach into both, is a wider door
    /// than this needs.
    pub view_reset_requested: bool,
}

impl Default for AppControls {
    fn default() -> Self {
        Self {
            is_playing: true,
            step_mode: StepMode::Time,
            step_size: 0.1,
            play_speed: 1.0,
            step_distance_km: 1000.0,
            rays_per_pulse: RAYS_PER_PULSE,
            draw_front_arcs: true,
            hide_wound_segments: true,
            show_river: true,
            show_streamlines: true,
            // Alice leads and Bob trails her by 8 M. Everything else in the app is written around
            // that order - the HUD's "Alice → Bob" line, the mode tips, the Theory Guide's account
            // of the stack on r₋ that Bob cuts through - so the defaults are the layout those
            // texts describe rather than the mirror image of it.
            alice: ObserverSettings::raindrop(0.0),
            bob: ObserverSettings::raindrop(8.0),
            show_theory_modal: false,
            use_km: true,
            frame_of_ref: ReferenceFrame::DistantObserver,
            font_scale: 1.0,
            view_reset_requested: false,
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

/// What a Distance-mode step is worth in coordinate time when nobody on the canvas is moving in r,
/// which is `AppControls::step_size`'s own default: see `AppControls::distance_step`.
const DISTANCE_STEP_STALLED_DT: f64 = 0.1;

/// The floor put under the coordinate speed a Distance-mode step is divided by. Near a turning
/// point |dr/dt| runs to zero and Δr / |dr/dt| runs away with it; this bounds the step at
/// 100 Δr of coordinate time. It is not a floor on anybody's velocity - nothing physical reads it -
/// only on how long one press of an arrow key is allowed to be worth.
const DISTANCE_STEP_MIN_SPEED: f64 = 0.01;

/// The radius both observers are dropped from, by ⏮ Reset, by Drop Observers and at startup: far
/// enough outside the ergosphere (r_E = 2M on the equator) for a static hover to exist there, close
/// enough in for the whole fall to be a few tens of M.
const DROP_RADIUS: f64 = 4.5;

/// The hover tips on the four Motion buttons of an observer card. Each says what worldline the
/// option is, which 4-velocity it puts at the observer's event, where that worldline exists, what
/// happens where it does not - `Observer::effective_mode`'s fallback - and what the mode is for.
/// The two fallback sentences quote `impossible_mode_note` word for word, so the tip, the note under
/// the buttons and the telemetry box cannot describe the same observer differently.
///
/// They are written about "the observer" rather than about Bob, because both cards show them.
const FREE_FALL_TIP: &str = "A timelike geodesic: the observer falls with no thrust at all and their accelerometer reads exactly zero, which is the whole content of the word. Which geodesic is fixed by the two conserved quantities they were dropped with, the energy per unit mass E = −u_t and the axial angular momentum per unit mass L = u_ϕ on the sliders below, and their four-velocity is the one the integrator is carrying along that curve, so the telemetry, the frame their pulses go out into and the frame their receptions are measured in are all the same object as the worldline being drawn. E = 1 with L = 0 is the raindrop, dropped from rest at infinity and falling straight in; that is the congruence the River of Space is made of, so they are then riding one of the drops. A geodesic exists at every radius and this is the only mode that does: they cross the ergosphere, the outer horizon r₊ and the Cauchy horizon r₋ in finite proper time with nothing local happening to them at any of them, and for the equatorial L = 0 case the fall ends on the ring, where the curvature is genuinely infinite and the chart stops. Give them enough prograde angular momentum and they freeze onto r₋ instead, their proper time reaching a finite limit while the coordinate clock runs on. It is the mode the light cones and both transmissions read most naturally in, because an infaller is the observer the whole interior picture is drawn for.";
const MANUAL_DRAG_TIP: &str = "The observer's position is yours: set their radius on the slider below — and Bob's marker can be dragged straight across either canvas as well — and they stay exactly where you put them while the clock runs. What the two boost sliders set is their velocity — β_r radially and β_ϕ azimuthally, as fractions of c — relative to the local raindrop, the observer dropped from rest at infinity passing through that same point, which is the one reference frame that exists at every radius, between the horizons included. Their four-velocity is that raindrop frame boosted by (β_r, β_ϕ), so the telemetry, the rest-frame view and the pulses they transmit are all drawn for an observer moving at that velocity through the point you are holding them at, while the point itself does not drift. Those two statements are not one worldline, and here that is deliberate: the position is an input rather than an integration, so the drawn marker and the reported velocity are answering different questions, and this is the only mode in which they are allowed to. β = 0 reproduces the free-fall frame exactly; anything else is a rocket, and the thrust that holding it would cost is what the telemetry quotes as a_thrust. Let go of the marker and free fall resumes from the new event with their conserved E and L unchanged, rather than from wherever they were before you picked them up.";
const STATIC_TIP: &str = "The observer hovers: fixed r and fixed ϕ, station-keeping against the distant stars, with a four-velocity along the time-translation Killing vector ∂/∂t normalised to unit length. The thrust that costs is real, it is what the telemetry reports as a_prop, and it grows without bound as they near the static limit. That worldline exists only where ∂/∂t is timelike, g_tt < 0, which on the equator means r > 2M — outside the ergosphere, not merely outside the horizon. Inside the ergosphere the frame dragging is total: holding ϕ fixed is a spacelike motion there and no rocket, however powerful, can do it. The selection is kept rather than refused, because it is a standing request and resumes by itself the moment they are somewhere it can exist again, but what they actually do in the meantime is fall freely — in position as much as in velocity — and this panel and their telemetry box both read “Static impossible here (r ≤ 2M): falling freely” while that lasts. It is the mode for the exterior: gravitational blueshift, the redshift of an infaller's signal and the weight of the hole are all statements about what a static observer measures.";
const ZAMO_TIP: &str = "The zero-angular-momentum observer, the frame in which a spinning hole looks as unrotating as it can. They hold their radius like the static observer but do not fight the frame dragging: they are swept around at the local dragging rate ω = −g_tϕ/g_ϕϕ, exactly fast enough that their own angular momentum L = u_ϕ vanishes, and their four-velocity is γ(1, 0, ω). Light leaves them with no built-in swirl, which is why the River of Space quotes its flow speed past them, β = √(1 − α²), reaching c at the outer horizon, and why they are the observer the lapse α belongs to. A fixed-r worldline is timelike only outside the outer horizon r₊, so unlike the static observer they survive the whole ergosphere — going along with the dragging is precisely what the static observer cannot afford to stop doing. At r₊ and inside it the radial direction is timelike and nothing can hold a radius at all; the selection is kept, they fall freely instead, and this panel and their telemetry box both read “ZAMO impossible inside r₊: falling freely” until they are back outside. Use it to read the ergosphere, where it is the only hovering observer there is.";

/// The hover tip on the Wavefront points slider.
const WAVEFRONT_POINTS_TIP: &str = "How finely a pulse samples the emitter's light cone: n directions at α = 2πi/n, spaced 360/n degrees apart — 2.5° at the default of 144 — with α = 0, the emitter's own outward radial leg, always the first of them whatever n is. Each direction is one exact null geodesic, so this is the resolution of the whole picture the light draws: more points give finer tongues where the front is being swallowed at the ring, more beads along the frozen arcs stacked on r₋, shorter segments around the loop on the equatorial view, and rarer handovers from one sheet of a front to the next in the reception test, since neighbouring rays are then closer together in azimuth. They are not free. Integrating the rays, testing them against the receiver's worldline and drawing them all scale linearly in the count: about 1.1 ms of frame time for each extra 72 rays a pulse with forty pulses in flight, of which the integration and the reception test are 0.3 ms, so 1024 points is about seven times the work of 144 and the play loop is the first thing to feel it. It applies to pulses sent from now on. Light already in flight is the geodesics that were launched, and each pulse keeps the count it went out with, so the slider changes the transmission rather than redrawing it.";

/// The hover tip on the Arcs between wavefront points checkbox.
const FRONT_ARCS_TIP: &str = "Whether the pieces of a wavefront between neighbouring rays are drawn. Ticked, each piece is the curve linear in (r, ϕ) from one ray to the next, cut into steps of at most 0.05 rad and each step put through the embedding x + iy = (r + ia)e^{iϕ} — so a piece joining two rays sitting on r₋ is drawn as an arc of the r₋ circle, and one joining two rays a quarter of a turn apart is drawn going round. Unticked, nothing is drawn between the rays: the front is shown as the calculated points themselves, one dot per ray in the same gain colour the arc would have had, and the frozen family keeps its heavier beads. Nothing physical turns on it. The reception test interpolates in (r, ϕ) along exactly the same pieces whichever way they are drawn, so an arrival happens at the same event, at the same measured shift, in both settings. What the tick buys is that the front you are looking at is the same curve the detector is testing; what unticking buys is the raw output of the integrator with no interpolation laid over it, which is worth being able to see, because everything the arcs add is inference. Inside r₋ the annulus is thin (at a = 0.90 the embedding puts r₋ at ρ = 1.06 against the ring at ρ = 0.90) and neighbouring rays wind at wildly different rates, dϕ/dt running from about −5 per M near the ring to +0.8 for one settling onto r₋, so a pair of neighbours ends up most of a radian apart and the arc between them is drawn along a curve no ray was integrated on; the dots are the part that is not inferred. It is independent of the winding cut on the checkbox below it, which drops the two or so segments per pulse whose rays have wound more than a whole turn apart; with the arcs unticked that cut has nothing left to drop, since every live ray is already drawn as its own dot. The setting is a view setting and is kept across ⏮ Reset and Drop Observers, as every control on this panel is.";

/// The hover tip on the Hide segments wound past a full turn checkbox.
const HIDE_WOUND_TIP: &str = "Whether the pieces of a front whose two rays have wound more than one whole turn apart in ϕ are drawn at all. Such a pair is not an ordinary neighbouring pair that has drifted: it straddles a critical impact parameter, the emission angle either side of which a ray is captured rather than escaping. At a = 0.90 the prograde equatorial photon orbit sits at r_ph = 1.56M, just outside r₊ = 1.44M, and the retrograde one at 3.89M. The ray just inside the critical angle hangs on that unstable orbit for tens of M and then spirals in and freezes onto r₋, where it co-rotates at Ω₋ = 0.9 per M for ever; the ray just outside it hangs there too and then escapes outward at very nearly c. The real front between them is pinned on the photon orbit: a spiral inward from the far ray down to r_ph, a pile-up of turns at r_ph that no sampling of the light cone can resolve, and a spiral from r_ph down to r₋. Two rays 2.5° apart cannot carry that shape, and the drawing does not have the information to invent it. What the interpolation linear in (r, ϕ) draws in its place is an Archimedean spiral with the whole winding spread evenly over every radius between the two ends, r₊ included, so as the outer ray runs away while the winding grows only at Ω₋, the turns drift steadily outward across the outer horizon — a fan of spiral arms crossing r₊ that no ray took and nothing in the spacetime does. That is inference the drawing should not be making, so ticked, those segments are simply not drawn: each of the two rays is marked with its own dot in its own gain colour instead, so the cut reads as a gap with marked ends rather than as a silent hole. One whole turn is the threshold because past 2π the pair has been round the hole relative to one another and the segment covers every azimuth, so the sampling has no information at all about what the front does in there; below it the arc still misplaces the winding but is one arc between two neighbours of the same sheet. It removes about two segments per pulse — the prograde critical angle and the retrograde one, one segment each — and nothing else: every other segment is drawn exactly as before. Nothing physical turns on it. Reception detection is untouched: Pulse::scan interpolates in (r, ϕ) along every segment whether or not it is drawn, so an arrival happens at the same event, at the same measured shift, ticked or unticked. Unticking it puts the spirals back, which is worth being able to do — it is the honest picture of what two samples that far apart actually say. The setting is a view setting and is kept across ⏮ Reset and Drop Observers, as every control on this panel is.";

/// The hover tip on the Enable Observer checkbox, the same on both cards.
const ENABLE_TIP: &str = "Whether this observer is in the simulation at all. Unticked, they are not merely hidden: there is no worldline to step, nothing of them in either view, no telemetry box, no light going out and no arrival coming in, and their transmission is dropped. The other observer's transmission goes on exactly as before — the light already in flight does not care whether anybody is left to hear it — but records no reception, because there is nobody there to make one. Ticking the box back on drops this observer afresh from this card, at r = 4.5M on the clock's current reading, hovering there until their own Release Delay has passed; the rest of the run is left alone, so the other observer is not restarted.";

/// The hover tip on the Transmit Signal checkbox of Alice's card. What her transmission is, and
/// what it does at the Cauchy horizon: the frozen arc on r₋ that Bob later cuts through.
const ALICE_SIGNAL_TIP: &str = "Alice broadcasts a pulse into the whole of her own light cone every 0.1 M of her proper time, and every ray of it is an exact null geodesic of the coded metric. Colour on the equatorial view is the gain each piece of the front has picked up since it was let go: the frequency a local raindrop measures on it here, divided by the frequency the raindrop that was passing Alice measured as it left her. Both are drops of the same E = 1, L = 0 congruence, the one family of observers that exists at every radius, so the number is an ordinary measured shift between two of them along the ray and it means the same thing everywhere in the picture. At emission it is exactly 1 for every ray, whatever Alice is doing, so a fresh pulse comes out one uniform deep red and then works its way up the ramp as it falls: orange at threefold, yellow at tenfold, white at about thirtyfold, blue at a thousandfold and violet at a hundred thousandfold, darkening to maroon on the rare ray that loses frequency instead. The arrivals are reported on a different quantity and keep their own red-white-blue ramp: what a receiver measured against Alice's own emission, which is the question a reception asks. Inside r₊ the rays that never reach r₋ are the prograde ones, dragged forward in ϕ: that is the arc of the pulse around α = 90°, running from about 45° to 135° well inside r₊, wider than that just below r₊ and narrowing as Alice nears r₋, its edges lying exactly where E − Ω₋L changes sign. On the equatorial view that arc is drawn heavier and nearly opaque, and drawn last so nothing paints over it: the part of each ring sent prograde enough to have negative energy along the inner horizon's rotating generator, E − Ω₋L < 0, which never crosses the drawn r₋ circle but piles onto it from outside while co-rotating at Ω₋, whereas the rest of the ring crosses at finite time. The frozen arc is about a third of the ring for a pulse sent just inside r₊ and only a sliver for one sent close to r₋, and it is beaded with dots because the arc collapses onto r₋ faster than a pixel can show. Those arcs stack up against the Cauchy horizon while the rest of the pulse falls through it, and because the pulses are close enough together for consecutive arcs to overlap there, an infaller crossing r₋ where they stand cuts through several sheets in a row, each blueshifted on the scale exp(κ₋Δt). Each loop is one pulse and encloses Alice, since light is isotropic in Alice's own frame, and the dot on the loop marks the emission event on Alice's trail. Inside r₊ the flow carries the whole loop inward, so the loop's outer edge never gets further from the hole than that dot: the river model, drawn with light. On the (t, r) diagram, where azimuth cannot be drawn at all, a pulse is its radial extent: a wedge from the emission event, filled faintly in her amber, whose lower edge is the most ingoing ray of the pulse and whose upper edge is the outermost one. The lower edge is the ingoing edge of Alice's own light cone carried forward - the 45° line dr/dt = −1 for a hole with no spin, a little steeper for one that spins, and steeper again the deeper it goes - and inside r₊ it runs on to the ring while the upper edge freezes on r₋, so the upper edges of her interior pulses stack up on the Cauchy horizon, which in this chart is where the outgoing light of the whole interior accumulates, and that stack is what a later infaller cuts through. A worldline inside a wedge is in range of that pulse, not necessarily receiving it: the diagram cannot say whether the ray standing at that radius is at the receiver's azimuth. The dots on a worldline are the actual receptions, and they are the only marks of one.";

/// The hover tip on the Transmit Signal checkbox of Bob's card. The return path, which is not the
/// mirror image of Alice's: it has an end.
const BOB_SIGNAL_TIP: &str = "Bob broadcasts exactly as Alice does, a whole light cone of exact null geodesics every 0.1 M of his own proper time, and he starts at t = 0, before he is released: while he waits he is the static observer at his hover radius, with a clock ticking at √(−g_tt) of coordinate time and an orthonormal frame to broadcast into, and nothing in the geometry stops him transmitting from it. His pulses come every 0.134 M of coordinate time while he hovers at r = 4.5M and every 0.1 M of his own once he falls. The colours mean the same thing as Alice's: on the equatorial view, the gain between two raindrops along each ray since it left him, so his fronts are born the same uniform deep red hers are and climb the same ramp. His fronts are drawn at half stroke width and his emission dots in his own mint, so the two transmissions can be told apart without touching the colouring, which is a measurement. What is not the same is the physics of the return path. In the layout the app opens on, and that ⏮ Reset and Drop Observers rebuild, Bob is behind Alice on the same infall, so his pulses chase her inward, and the only part of each one that ever catches her is the ingoing part of his cone: it runs at up to dr/dt = −1 in this chart, which no timelike worldline can match. That is the light whose shift is finite on the branch of r₋ she actually crosses, so unlike Alice → Bob there is no stack for her to cut through. His frozen family, E − Ω₋L < 0, does pile onto r₋ from outside, but it settles there behind her, after she has already gone through, so she never meets it. (Give him the shorter delay of the two and he is the deeper one instead, and his light climbs to her: the shift then starts as a small blueshift, because the fall toward the light beats the recession, and turns over into a redshift as he drops away below her.) And because her worldline ends — on the ring, or frozen on r₋ — his transmission stops arriving: there is a last pulse of his that reached her, and its emission event is the boundary, on his own worldline, of the causal past of the end of hers. Neither view marks that event; the HUD names it once her worldline has finished, giving the pulse, when and where he sent it, and how many later ones never arrive. At the app's default hole and delay it is one he sends while still hovering: the pulse of t = 1.84, which reaches her at t = 5.19, most of an M before her worldline ends at t = 6.06. He is released long after that, so nothing of his release or of his own fall ever reaches her: everything he sends past that event never arrives, however long he goes on sending, and by the end of her worldline that is 44 pulses. On the (t, r) diagram his pulses are drawn exactly as hers are, each as the wedge of its own radial extent but in his mint: lower edge the most ingoing ray, upper edge the outermost, a worldline inside the wedge in range of the pulse rather than receiving it, and the dots the actual arrivals.";

/// What to say about an observer whose selected mode cannot exist where they are, or None when the
/// selection is fine.
///
/// One sentence, stated once, so that the observer's panel card and the telemetry box on the canvas
/// cannot describe the same observer differently. It says what is happening rather than what is
/// being drawn: the worldline really is the free-fall one now, in position as much as in velocity,
/// and the selection is still standing and will resume the moment it becomes possible again.
pub fn impossible_mode_note(obs: &Observer, metric: &KerrSchild) -> Option<&'static str> {
    if obs.mode_admissible(metric) {
        return None;
    }
    match obs.mode {
        ObserverMode::Static => Some("Static impossible here (r ≤ 2M): falling freely"),
        ObserverMode::Zamo => Some("ZAMO impossible inside r₊: falling freely"),
        // `mode_admissible` is unconditionally true for these two.
        ObserverMode::FreeFall | ObserverMode::ManualDrag => None,
    }
}

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

/// The part of an observer card that is fixed for that observer: who it is, the colour their name
/// is written in, the azimuth a drop starts them at, and the hover text on their Transmit Signal
/// box, which is about their own transmission and nobody else's.
///
/// Everything else on the card - every control, in the same order, with the same wording - is the
/// same code for both, which is the point: Alice and Bob are one idea run twice.
struct ObserverCard {
    name: &'static str,
    colour: egui::Color32,
    /// The azimuth a drop starts this observer at. The two differ so that the equatorial view can
    /// tell the trails apart; the physics of an equatorial worldline does not depend on it.
    start_phi: f64,
    transmit_tip: &'static str,
}

const ALICE_CARD: ObserverCard = ObserverCard {
    name: "Alice",
    colour: Theme::ALICE_COLOR,
    start_phi: 0.25,
    transmit_tip: ALICE_SIGNAL_TIP,
};

const BOB_CARD: ObserverCard = ObserverCard {
    name: "Bob",
    colour: Theme::BOB_COLOR,
    start_phi: 0.0,
    transmit_tip: BOB_SIGNAL_TIP,
};

impl ObserverCard {
    /// Draw this observer's card and apply what it says.
    ///
    /// Two of the controls act on the spot rather than at the next drop, and both are enforced here
    /// as a state of affairs rather than as an edge: an unticked Enable means the observer *is*
    /// None every frame, and an unticked Transmit means their field *is* empty every frame. That
    /// way a test, or a keybinding, that writes the flag directly gets the same simulation as a
    /// user clicking the box.
    #[allow(clippy::too_many_arguments)]
    fn show(
        &self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        settings: &mut ObserverSettings,
        obs: &mut Option<Observer>,
        field: &mut SignalField,
        current_time: f64,
        use_km: bool,
    ) {
        ui.group(|ui| {
            ui.label(
                egui::RichText::new(format!("OBSERVER {}", self.name.to_uppercase()))
                    .strong()
                    .color(self.colour),
            );

            ui.checkbox(&mut settings.enabled, "Enable Observer")
                .on_hover_text(ENABLE_TIP);
            if !settings.enabled {
                // Out of the simulation entirely: no worldline to step or draw, and the light they
                // had in flight goes with them, since it was emitted by a worldline that is no
                // longer part of the run.
                *obs = None;
                field.silence();
                ui.label(
                    egui::RichText::new("Not in the simulation: no worldline, no transmission, no receptions")
                        .small()
                        .color(Theme::TEXT_MUTED),
                );
                return;
            }
            let obs = obs.get_or_insert_with(|| {
                // Ticked back on part-way through a run: dropped afresh from this card at the
                // clock's current reading, so the worldline starts on the clock rather than
                // somewhere behind it, and hovers until its own delay has passed.
                field.silence();
                self.dropped(metric, settings, current_time)
            });

            ui.checkbox(&mut settings.transmit, "Transmit Signal")
                .on_hover_text(self.transmit_tip);
            if !settings.transmit {
                field.silence();
            }

            ui.horizontal(|ui| {
                ui.label("Motion:");
                ui.selectable_value(&mut obs.mode, ObserverMode::FreeFall, "Free Fall")
                    .on_hover_text(FREE_FALL_TIP);
                ui.selectable_value(&mut obs.mode, ObserverMode::ManualDrag, "Drag / Manual")
                    .on_hover_text(MANUAL_DRAG_TIP);
                ui.selectable_value(&mut obs.mode, ObserverMode::Static, "Static")
                    .on_hover_text(STATIC_TIP);
                ui.selectable_value(&mut obs.mode, ObserverMode::Zamo, "ZAMO")
                    .on_hover_text(ZAMO_TIP);
            });

            // A static observer needs r > 2M (timelike d/dt); a ZAMO needs r > r+ (a fixed-r
            // worldline can only be timelike outside the outer horizon). Where the selection is
            // impossible, the observer does not merely *look* like a free-faller:
            // `Observer::effective_mode` puts them on the free-fall worldline in position as well
            // as in velocity, and the selection is kept so that it resumes by itself if they ever
            // get back out. The wording is the telemetry box's, so the two cannot say different
            // things about the same observer.
            if let Some(note) = impossible_mode_note(obs, metric) {
                ui.label(egui::RichText::new(note).small().color(Theme::TEXT_MUTED));
            }

            if obs.mode == ObserverMode::ManualDrag {
                if use_km {
                    let mut r_km = metric.r_to_km(obs.r);
                    let min_km = metric.r_to_km(0.05);
                    let max_km = metric.r_to_km(5.5);
                    if ui
                        .add(egui::Slider::new(&mut r_km, min_km..=max_km).text("Radial Position r (km)"))
                        .changed()
                    {
                        obs.r = metric.km_to_r(r_km);
                    }
                } else {
                    ui.add(egui::Slider::new(&mut obs.r, 0.05..=5.5).text("Radial Position r (M)"));
                }

                // The boost only defines a worldline in ManualDrag mode: free fall, static and
                // ZAMO observers each pin down their own 4-velocity, so a beta there would be
                // ignored. Hide the sliders rather than show dead controls.
                ui.add(egui::Slider::new(&mut obs.beta_r, -0.95..=0.95).text("Radial Boost β_r"));
                ui.add(egui::Slider::new(&mut obs.beta_phi, -0.95..=0.95).text("Azimuthal Boost β_ϕ"));
                ui.label(
                    egui::RichText::new(format!(
                        "Velocity relative to a raindrop observer (dropped from rest at infinity) at {}'s r",
                        self.name
                    ))
                    .small()
                    .color(Theme::TEXT_MUTED),
                );

                if ui
                    .button("Reset Thrusters")
                    .on_hover_text(
                        "Set both boosts back to zero, which puts this observer at rest in the local raindrop frame: β = 0 is free fall exactly, and their proper acceleration goes back to nothing.",
                    )
                    .clicked()
                {
                    obs.beta_r = 0.0;
                    obs.beta_phi = 0.0;
                }
            }

            ui.add(
                egui::Slider::new(&mut settings.delta_t_delay, 0.0..=30.0)
                    .text("Release Delay Δt"),
            )
            .on_hover_text(
                "How long after the drop this observer is let go. Until then they hover at r = 4.5M as a static observer — a real worldline, with a clock running at √(−g_tt) of coordinate time and a frame to transmit from — and the wait is what puts one observer behind the other on the same infall. It takes effect at the next ⏮ Reset or Drop Observers, since a release time is part of a worldline rather than something that can be changed under one.",
            );
            ui.add(
                egui::Slider::new(&mut settings.energy, 0.90..=1.60)
                    .text("Energy E (per unit mass)"),
            );
            ui.add(
                egui::Slider::new(&mut settings.l_ang, -4.0..=4.0)
                    .text("Angular momentum L (per unit mass, M)"),
            );
            ui.checkbox(&mut settings.outgoing_start, "Start on the outgoing root (dr/dτ > 0)");

            // Below the effective potential V(r, L) there is no timelike geodesic through
            // r = 4.5M at all, so the drop raises E to the floor instead of refusing.
            let floor = GeodesicState::energy_floor(metric, DROP_RADIUS, settings.l_ang);
            if settings.energy < floor {
                ui.label(
                    egui::RichText::new(format!(
                        "E raised to {:.3}: below that, r = {}M is forbidden for this L",
                        floor, DROP_RADIUS
                    ))
                    .small()
                    .color(Theme::TEXT_MUTED),
                );
            }
            if settings.outgoing_start {
                ui.label(
                    egui::RichText::new(
                        "Outgoing start: the ingoing chart cannot follow an outward crossing of r₋",
                    )
                    .small()
                    .color(Theme::TEXT_MUTED),
                );
            }
        });
    }

    /// This observer as the card asks for them, dropped at the clock reading `start_t`.
    fn dropped(
        &self,
        metric: &KerrSchild,
        settings: &ObserverSettings,
        start_t: f64,
    ) -> Observer {
        settings
            .dropped(metric, self.name, self.start_phi, start_t)
            .expect("only called with the card ticked")
    }

    /// The same drop, but keeping whatever Motion the observer being replaced was on.
    ///
    /// The mode lives on the observer rather than in the settings, because a drag on the canvas
    /// sets it too, but it is a standing request about how they move rather than part of the event
    /// they are dropped at: a Reset that put a Static observer back on the free-fall worldline
    /// would be answering a question the user had not asked. A card that has just been ticked on
    /// has nobody to inherit from and starts, like every fresh observer, in free fall.
    fn redropped(
        &self,
        metric: &KerrSchild,
        settings: &ObserverSettings,
        previous: Option<&Observer>,
        start_t: f64,
    ) -> Option<Observer> {
        let mut obs = settings.dropped(metric, self.name, self.start_phi, start_t)?;
        if let Some(prev) = previous {
            obs.mode = prev.mode;
        }
        Some(obs)
    }
}

impl AppControls {
    /// Build the run the app opens on, and that ⏮ Reset and Drop Observers rebuild: the clock at
    /// zero, both transmissions dropped, and every ticked observer re-dropped from r = 4.5M -
    /// Alice at ϕ = 0.25, Bob at ϕ = 0 - each hovering there until their own Release Delay has
    /// passed, on the worldline their own E, L and outgoing flag pick out.
    ///
    /// There is one layout rather than two. Reset used to put Bob at r = 3.8M released at t = 0
    /// while Drop Observers put him at 4.5M with a delay, so the same run had two openings and the
    /// two buttons disagreed about which; with a card per observer that is simply confusing, so
    /// Reset, Drop Observers and `SpacetimeApp::default` all come through here.
    ///
    /// The transmissions are cleared rather than rewound. A rewind keeps the light in flight, and
    /// there is none to keep: the wavefronts standing in the field were emitted by worldlines that
    /// this is about to replace, and where the geometry itself has just changed under them they are
    /// null geodesics of a metric that no longer applies.
    ///
    /// All three callers - the transport's Reset and Drop Observers buttons, and the preset row,
    /// which restarts the run because it has changed the hole - also want the (t, r) view put back
    /// where it starts, so the request is raised here rather than at each of them.
    ///
    /// It is `pub(crate)` because these are egui buttons and a test cannot click one without a
    /// harness that drives the pointer:
    /// `app::tests::test_resetting_the_run_puts_the_time_pan_back_to_the_start` calls the action
    /// the button calls and then runs a real frame of the app over it.
    pub(crate) fn drop_observers(
        &mut self,
        metric: &KerrSchild,
        alice: &mut Option<Observer>,
        bob: &mut Option<Observer>,
        signals: &mut SignalPair<'_>,
        current_time: &mut f64,
    ) {
        *current_time = 0.0;
        signals.clear();
        *alice = ALICE_CARD.redropped(metric, &self.alice, alice.as_ref(), 0.0);
        *bob = BOB_CARD.redropped(metric, &self.bob, bob.as_ref(), 0.0);
        self.view_reset_requested = true;
    }

    /// Whether the (t, r) view has been asked to go back to the start since this was last called,
    /// clearing the request. Called once a frame by `SpacetimeApp::ui`, after the panel has run.
    pub fn take_view_reset(&mut self) -> bool {
        std::mem::take(&mut self.view_reset_requested)
    }

    /// The gap between the two releases, which is the Δt the blueshift scale exp(κ₋Δt) is quoted
    /// against: a pulse Alice sends is stacked on r₋ for exactly as long as Bob trails her before
    /// crossing it. Zero where Bob leads, since then there is no stack of hers for him to cut
    /// through at all and the scale is quoting nothing.
    pub fn release_gap(&self) -> f64 {
        (self.bob.delta_t_delay - self.alice.delta_t_delay).max(0.0)
    }

    /// What one Distance-mode step is worth in coordinate time: the time an observer who is
    /// actually moving needs to cover the requested Δr at their coordinate speed |dr/dt|.
    ///
    /// The step is quoted for Bob, so it is his speed whenever he has one. He does not always have
    /// one. A Bob still waiting for release stands on the static worldline his clock is keeping, a
    /// Static or ZAMO Bob holds his radius by construction, and for all three dr/dt is exactly
    /// zero: "the time for Bob to cover Δr" is then not a long time, it is an undefined one, and
    /// dividing by a floored speed to get one is inventing an answer. A Bob who is not in the
    /// simulation at all has no speed for the same reason and drops out of the chain the same way.
    /// Alice's coordinate speed is used instead whenever she is on the canvas and falling, since
    /// she is then the worldline crossing the radii the user is stepping through; and if neither of
    /// them is moving in r the step falls back to a fixed `DISTANCE_STEP_STALLED_DT` of coordinate
    /// time, which claims nothing about a distance at all.
    ///
    /// `DISTANCE_STEP_MIN_SPEED` bounds a step taken near a turning point, and the result is
    /// clamped into [1e-8, 500] M besides.
    pub fn distance_step(
        &self,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
    ) -> f64 {
        let delta_r_m = metric.km_to_r(self.step_distance_km);
        let moving = [bob, alice]
            .into_iter()
            .flatten()
            .map(|obs| obs.velocity_c(metric).abs())
            .find(|speed| *speed > 0.0);
        match moving {
            Some(speed) => (delta_r_m / speed.max(DISTANCE_STEP_MIN_SPEED)).clamp(1e-8, 500.0),
            None => DISTANCE_STEP_STALLED_DT,
        }
    }

    /// The two transmissions as `SignalPair` wants them: who is where, and who is sending.
    fn endpoints<'a>(
        &self,
        alice: Option<&'a Observer>,
        bob: Option<&'a Observer>,
    ) -> (Endpoint<'a>, Endpoint<'a>) {
        (
            Endpoint { observer: alice, transmitting: self.alice.transmit },
            Endpoint { observer: bob, transmitting: self.bob.transmit },
        )
    }

    pub fn render_panel(
        &mut self,
        ui: &mut egui::Ui,
        metric: &mut KerrSchild,
        alice: &mut Option<Observer>,
        bob: &mut Option<Observer>,
        mut signals: SignalPair<'_>,
        current_time: &mut f64,
    ) {
        ui.label(egui::RichText::new("Ingoing Kerr-Schild Foliation").small().color(Theme::TEXT_MUTED));
        ui.separator();

        // 0. Frame of Reference Selector
        ui.group(|ui| {
            ui.label(egui::RichText::new("FRAME OF REFERENCE").strong().color(Theme::UI_HEADING));
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
            ui.label(egui::RichText::new("SIMULATION CONTROL").strong().color(Theme::UI_HEADING));
            ui.horizontal(|ui| {
                let play_btn_text = if self.is_playing { "⏸ Pause (Space)" } else { "▶ Play (Space)" };
                if ui
                    .button(play_btn_text)
                    .on_hover_text("Toggle Play/Pause simulation (Spacebar)")
                    .clicked()
                {
                    self.is_playing = !self.is_playing;
                }
                if ui
                    .button("⏮ Reset")
                    .on_hover_text("Put the run back to the layout the app opens on, which is the one Drop Observers builds: the clock at zero and every ticked observer dropped afresh from their card.")
                    .clicked()
                {
                    self.drop_observers(metric, alice, bob, &mut signals, current_time);
                }
                let current_step = match self.step_mode {
                    StepMode::Time => self.step_size,
                    StepMode::Distance => self.distance_step(metric, bob.as_ref(), alice.as_ref()),
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
                    ObserverPair { bob: bob.as_mut(), alice: alice.as_mut() }
                        .rewind_to(metric, *current_time);
                    signals.step_back(metric, back, alice.as_ref(), bob.as_ref());
                }
                if ui
                    .button("Step Fwd →")
                    .on_hover_text("Step forward by Step Size / Distance (Right Arrow key)")
                    .clicked()
                {
                    *current_time += current_step;
                    ObserverPair { bob: bob.as_mut(), alice: alice.as_mut() }
                        .step(metric, *current_time, current_step);
                    // One description of a step forward, shared with the play loop and the arrow
                    // keys: carry both transmissions, let each emitter emit, then let each receiver
                    // listen.
                    let (a, b) = self.endpoints(alice.as_ref(), bob.as_ref());
                    // The panel's own step is a step like any other, so the wavefront count goes in
                    // the same way it does on the played frame: see `SignalPair::set_rays_per_pulse`.
                    signals.set_rays_per_pulse(self.rays_per_pulse);
                    signals.advance(metric, current_step, a, b);
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
                    )
                    .on_hover_text(
                        "How far Bob should move in r per step. The step is still taken in coordinate time: Δt = Δr / |dr/dt| at his current coordinate speed. A Bob who is not moving in r — hovering before release, or holding a radius as a Static or ZAMO observer — has no such time, and one who is not in the simulation at all has none either, so Alice's speed is used instead while she is falling, and if neither of them is moving in r the step is a fixed 0.1 M of coordinate time and the distance is not being honoured at all.",
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
            ui.add(
                egui::Slider::new(&mut self.rays_per_pulse, 64..=1024)
                    .integer()
                    .text("Wavefront points"),
            )
            .on_hover_text(WAVEFRONT_POINTS_TIP);
            ui.checkbox(&mut self.draw_front_arcs, "Arcs between wavefront points")
                .on_hover_text(FRONT_ARCS_TIP);
            ui.checkbox(&mut self.hide_wound_segments, "Hide segments wound past a full turn")
                .on_hover_text(HIDE_WOUND_TIP);

            ui.checkbox(&mut self.show_river, "River of Space (raindrop flow)")
                .on_hover_text(
                    "Each drop is an element of the E = 1, L = 0 raindrop flow of the Painlevé-Gullstrand / Doran river model, drawn at proper size and entering the field at r = 12M as a circle of proper diameter 0.1 M. The flow alone deforms that circle after that: length along the flow grows as √(12M/r), the ratio of Doran river speeds, and width across the flow shrinks as neighbouring flow lines converge, √g_φφ δφ. The drawn aspect ratio is therefore the tidal stretching of the fluid element, reaching about 16 at r₊ for a = 0.65. Colour is the flow speed past a local ZAMO, β = √(1 − α²), reaching c at r₊.",
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

            ui.separator();
            ui.label(
                egui::RichText::new(
                    "Drop Observers rebuilds every ticked observer from their card: Alice from r = 4.5M at ϕ = 0.25 and Bob from r = 4.5M at ϕ = 0, each hovering there until their own Release Delay, on the worldline their own E and L pick out. Both transmissions are dropped and the clock goes back to zero. ⏮ Reset builds the same layout, and it is the one the app opens on.",
                )
                .small()
                .color(Theme::TEXT_MUTED),
            );
            if ui.button("Drop Observers").clicked() {
                self.drop_observers(metric, alice, bob, &mut signals, current_time);
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

        // 2. Black Hole Parameters & Presets
        ui.group(|ui| {
            ui.label(egui::RichText::new("BLACK HOLE PROPERTIES").strong().color(Theme::UI_HEADING));

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
                    self.drop_observers(metric, alice, bob, &mut signals, current_time);
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
        });

        ui.add_space(4.0);

        // 3. The two observer cards, Alice first because she is the one released first. They are
        // the same code twice: see `ObserverCard`.
        ALICE_CARD.show(ui, metric, &mut self.alice, alice, signals.alice, *current_time, self.use_km);
        ui.add_space(4.0);
        BOB_CARD.show(ui, metric, &mut self.bob, bob, signals.bob, *current_time, self.use_km);

        ui.add_space(6.0);

        // 4. Theory Explanations
        if ui.button("📖 Relativistic Theory & Horizons").clicked() {
            self.show_theory_modal = !self.show_theory_modal;
        }
    }
}
