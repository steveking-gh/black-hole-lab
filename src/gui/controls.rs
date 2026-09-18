use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode, ObserverPair, Release, WorldlineParams};
use crate::physics::wavefront::{Endpoint, MAX_PULSES, RAYS_PER_PULSE, SignalField, SignalPair};

/// Why one M is a mass, a length and a duration at the same time.
const M_UNITS_TIP: &str =
"M is the hole's mass, and M is the only unit this app has. General relativity works with G = c = 1, which gives mass, length and time the same dimension: multiply a mass by G/c² to read that mass as a length, by G/c³ to read the same mass as a duration. So one number here, quoted three ways.

The two conversions differ by exactly one factor of c, which is the whole point: 1 M of space is the distance light crosses in 1 M of time. For that reason a horizontal M and a vertical M measure the same size on the charts, and a light ray runs at 45 degrees.

For the Sun that unit comes to 1.477 km and 4.927 µs. The charts quote every radius and every interval in multiples of the unit, so moving the Mass slider changes none of the physics on screen - the slider changes what one tick is worth in kilometres and seconds. The chart's own M equals 1 by construction; the slider sets what that 1 means.";

/// One of the panel's chip buttons: a quick pick that sets the slider beside it, or one arm of a
/// small choice - a motion, a release, a step mode. `Ui::selectable_label` and
/// `Ui::selectable_value` draw these with no frame at all until they are the selected one, which
/// leaves a row of them looking like a caption that has been broken into words rather than like a
/// row of things to press. The outline is the same colour on every chip, dimmed to three quarters
/// where the chip is not selected: enough to keep an unpressed chip reading as a thing to press
/// without competing with the fill, which is what actually says which one is on.
fn chip(ui: &mut egui::Ui, selected: bool, label: &str) -> egui::Response {
    let fill = if selected {
        ui.visuals().selection.bg_fill
    } else {
        egui::Color32::TRANSPARENT
    };
    let stroke = if selected {
        Theme::CHIP_OUTLINE_ACTIVE
    } else {
        Theme::dimmed(Theme::CHIP_OUTLINE_ACTIVE, 0.75)
    };
    ui.add(
        egui::Button::selectable(selected, label)
            .frame_when_inactive(true)
            .fill(fill)
            .stroke(egui::Stroke::new(1.0, stroke))
            .corner_radius(6.0),
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceFrame {
    DistantObserver,
    Bob,
    Alice,
    /// The same global Kerr-Schild foliation, drawn as a 2D+1 volume: the equatorial plane laid out
    /// as a floor at the present with coordinate time standing up out of it.
    ///
    /// It sits beside the other three because it is a choice of *chart*, which is what that
    /// selector has always been choosing. It is not a frame of reference and is never drawn in
    /// anybody's rest frame: the picture it gives is the same for everybody, which is what makes it
    /// the right place to read a horizon off. So everything that names a focus observer - whose
    /// watch Watch mode keeps, whose rest frame the flat diagram is drawn in - treats it exactly as
    /// `DistantObserver`.
    GlobalVolume,
}

impl ReferenceFrame {
    pub fn label(&self) -> &'static str {
        match self {
            Self::DistantObserver => "Global Foliation Chart 1D+1 (Kerr-Schild)",
            Self::Bob => "Bob's Frame Of Reference 1D+1",
            Self::Alice => "Alice's Frame Of Reference 1D+1",
            Self::GlobalVolume => "Global Foliation Chart 2D+1 (Kerr-Schild)",
        }
    }

    /// Whose watch Watch step mode is keeping: the observer this frame is drawn for, by name.
    /// The distant observer has no worldline in the simulation, and their watch is the chart's own
    /// Killing time, so Watch mode and Time mode are the same thing there - and the volume is that
    /// same chart, so it answers the same way.
    pub fn watch_owner(&self) -> &'static str {
        match self {
            Self::DistantObserver | Self::GlobalVolume => "the distant clock",
            Self::Bob => "Bob",
            Self::Alice => "Alice",
        }
    }

    /// The caption of the Watch-mode readout: whose clock the rate is measured on.
    pub fn watch_label(&self) -> &'static str {
        match self {
            Self::DistantObserver | Self::GlobalVolume => "Distant Clock",
            Self::Bob => "Bob's watch",
            Self::Alice => "Alice's watch",
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
    /// Fixed Δτ on the focus observer's own watch: the step the *View* selector's
    /// observer measures, converted to the coordinate time the integrators run on by their own
    /// u^t. See `AppControls::watch_step`.
    Watch,
}

/// The most coordinate time one played frame is allowed to take in Watch mode, in M.
///
/// It is not a fudge on the physics: `watch_step` reports what it asked for and what it got, and
/// the panel says so. It is the integrator's per-call budget. Time mode's fastest playback already
/// asks for 20 M/s × 0.1 s = 2 M of a single frame, so this is the largest step the rest of the app
/// is known to take in one go; a deeply time-dilated watch would otherwise ask for 1e10 M of
/// outside future per tick, which no geodesic or wavefront step can honour.
pub const WATCH_DT_CAP: f64 = 2.0;

/// What one Watch-mode step is worth, and what it cost: `dt` of coordinate time for the requested
/// proper interval, whether `WATCH_DT_CAP` bit, and the `u_t` = u^t the conversion used (1 for the
/// distant observer, whose watch *is* coordinate time).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WatchStep {
    /// The coordinate-time step to take: min(u^t Δτ, `WATCH_DT_CAP`).
    pub dt: f64,
    /// Whether the cap bit, i.e. whether this step is less proper time than was asked for.
    pub capped: bool,
    /// The focus observer's u^t at this event, the dilation factor between the two clocks.
    pub u_t: f64,
}

/// Everything one OBSERVER card asks for. Two of these hang off `AppControls`, one per observer,
/// and they are identical in shape: the two observers are the same idea run twice, so there is one
/// description of what a card holds rather than a Bob-shaped set of fields and an Alice-shaped one.
///
/// None of it is read from the observer, with one exception: `drop_r` is written back from the
/// observer's own radius whenever the clock reads zero, so that dragging a marker at the start of a
/// run moves where they are dropped from (`AppControls::remember_drop_positions`). Otherwise a card
/// is a standing request - what to build the next time this observer is dropped - and the observer,
/// once built, carries its own copy of the constants in its geodesic state. `l_ang`, `release`,
/// `drop_r` and `delta_t_delay` therefore take effect at the next drop, *unless* the clock reads
/// zero, where the run has not started and the card and the observer are the same thing: there they
/// take effect at once and the marker moves as the slider moves. `enabled` and `transmit` take
/// effect at once at any time.
///
/// There is no energy on the card. E is what the release implies at the radius it happens at - see
/// `Release` - so it is derived at the drop and reported under the sliders rather than dialled.
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
    /// Conserved axial angular momentum per unit mass L = u_phi, in units of M. The one constant
    /// of the motion the user states directly; E follows from it and from `release`.
    pub l_ang: f64,
    /// Where this observer's fall is released from, which is what sets their energy. See
    /// `Release`: at rest at the drop radius, or from rest at infinity.
    pub release: Release,
    /// The azimuth this observer is dropped at, in radians of the chart coordinate phi.
    ///
    /// Free, in the sense that the radius is not: Kerr is axisymmetric, so phi is a cyclic
    /// coordinate and moving an observer in it changes none of their constants - E, L and the whole
    /// radial problem are untouched. Only the *difference* between the two observers' azimuths
    /// means anything, and what it means is how far apart they are around the hole: how long light
    /// takes to get from one to the other, and where on r- one of them crosses relative to the band
    /// of frozen arcs the other's transmission has left standing there, co-rotating at Omega_-.
    ///
    /// It is not the angle the marker is drawn at. The embedding is x + iy = (r + ia)e^{i phi}, so
    /// the drawn azimuth is phi + atan2(a, r) - 11 degrees further round at r = 4.5M and 42 degrees
    /// at r = 1M, for a = 0.90 - which is why the card reports the drawn position as well.
    pub drop_phi: f64,
    /// The radius this observer is dropped from, by ⏮ Reset and at startup.
    ///
    /// It starts at `DROP_RADIUS` and is then whatever the user last put the observer at while the
    /// clock read zero: `AppControls::remember_drop_positions` copies it off the observer every
    /// frame the run is standing at its start, which is the one moment nothing else is moving them,
    /// so dragging a marker there is a statement about where the run begins rather than a nudge
    /// that the next Reset throws away. A drag taken once the clock is running is not remembered -
    /// that one is a change to a worldline in progress, and the run it began is already behind it.
    pub drop_r: f64,
    /// The Motion a *fresh* drop of this observer starts on.
    ///
    /// It is the only field here that the observer does not simply keep: Motion lives on the
    /// observer because a drag on the canvas sets it too, and `ObserverCard::redropped` hands a
    /// re-dropped observer the mode the one they replace was on, so a Reset never answers a
    /// question about how they move that the user has not asked. This is what an observer with
    /// nobody to inherit from starts as: the app's first build, and a card that has just been
    /// ticked back on.
    pub mode: ObserverMode,
}

impl ObserverSettings {
    /// A raindrop (E = 1, L = 0, ingoing) released `delta_t_delay` after the drop.
    fn raindrop(delta_t_delay: f64) -> Self {
        Self {
            enabled: true,
            transmit: true,
            delta_t_delay,
            l_ang: 0.0,
            release: Release::FromInfinity,
            drop_phi: 0.0,
            drop_r: DROP_RADIUS,
            mode: ObserverMode::FreeFall,
        }
    }

    /// An observer in free fall on the prograde innermost stable circular orbit of the hole the
    /// app opens on (`OPENING_SPIN`): the thrust-free orbit, released the moment the run starts.
    /// The radius is the ISCO's for that spin; a different hole chosen from the presets re-drops
    /// the observer from this radius, which the card then reports as stable or not for it.
    fn isco_orbiter(delta_t_delay: f64) -> Self {
        Self {
            mode: ObserverMode::FreeFall,
            release: Release::CircularPrograde,
            drop_r: KerrSchild::new(1.0, OPENING_SPIN).isco(true),
            ..Self::raindrop(delta_t_delay)
        }
    }

    /// The worldline this card is asking for: the release it names, at the radius it names, with
    /// the angular momentum it names. E is derived rather than dialled - see `Release`.
    fn worldline_params(&self, metric: &KerrSchild) -> WorldlineParams {
        WorldlineParams::released(metric, self.drop_r, self.l_ang, self.release)
    }

    /// The observer this card asks for, dropped at `drop_r` on the clock's reading `start_t`
    /// and released `delta_t_delay` of coordinate time later, or None when the card is unticked.
    fn dropped(&self, metric: &KerrSchild, name: &str, start_t: f64) -> Option<Observer> {
        let start_phi = self.drop_phi;
        self.enabled.then(|| {
            let mut obs = Observer::new_with_phi(
                metric,
                name,
                start_t,
                self.drop_r,
                start_t + self.delta_t_delay,
                start_phi,
                self.worldline_params(metric),
            );
            obs.mode = self.mode;
            obs
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
    /// What fraction of the proper time Watch mode asked for the last played frame actually
    /// carried: `dt / (u^t Δτ)`, which is 1 whenever `WATCH_DT_CAP` did not bite and falls to
    /// 1e-10 at the stall. `None` unless a frame has just been played in Watch mode.
    ///
    /// It is written by the play loop (`SpacetimeApp::ui`) and read by the panel, which prints it
    /// under the step-mode chips. A readout rather than a control: the honest statement of how far
    /// the promise "playback runs at 1 s/s of the focus observer's watch" is from being kept at
    /// the event being drawn, which near the Cauchy horizon is very far indeed.
    pub achieved_watch_rate: Option<f64>,
    /// How many rays a newly emitted pulse carries: the sampling of the emitter's light cone, at
    /// alpha = 2 pi i / n, and so the resolution of every wavefront sent from now on.
    ///
    /// It is a standing request, like the worldline constants on an observer card: pushed into both
    /// transmissions on every step through `SignalPair::set_rays_per_pulse`, read by
    /// `SignalField::emit_if_due` and by nothing else. Pulses already in flight keep the count they
    /// were emitted with, because their rays are the null geodesics that were launched.
    pub rays_per_pulse: usize,
    /// How many wavefronts each transmission keeps at once, from one to 128, starting at
    /// `MAX_PULSES`. Past it the oldest is dropped, so this is the length of the history the
    /// picture holds and, with `rays_per_pulse`, one of the two numbers that decide what a frame
    /// costs to integrate and to draw.
    ///
    /// Not a standing request like the ray count: it is pushed into both transmissions once a
    /// frame by `SpacetimeApp::ui`, played or paused, and `SignalPair::set_max_pulses` trims them
    /// to it as it goes. Lowering it therefore thins the picture immediately and permanently -
    /// `step_back` cannot recover an evicted pulse - while raising it widens the window from that
    /// frame onward. Like every control on this panel it survives Reset.
    pub max_pulses: usize,
    /// Whether the segments of a wavefront between neighbouring rays are drawn at all: on, each is
    /// the curve linear in (r, phi) between its two rays; off, only the rays themselves are drawn,
    /// one dot per calculated point and nothing between them.
    ///
    /// A drawing choice and nothing else: the reception test interpolates in (r, phi) along the
    /// same segments either way, so this cannot move an arrival or change a measured shift.
    /// Default on, because on is the drawing that matches what is being detected. Like every
    /// other control it survives Reset, which rebuilds the run and not the panel.
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
    /// control it survives Reset.
    pub hide_wound_segments: bool,
    /// Whether the equatorial view's block of details (horizon radii, scale, spin, the colour
    /// keys) is drawn in its corner. Off, a single line with the view's name and zoom stands in
    /// for it, so the canvas is clear for the picture. Toggled by the Details button drawn just
    /// above that block; like every control it is untouched by Reset.
    pub show_spatial_details: bool,
    /// Alice's card: whether she is in the simulation, whether she transmits, and the worldline
    /// the next drop puts her on.
    pub alice: ObserverSettings,
    /// Bob's card, identical in shape to Alice's.
    pub bob: ObserverSettings,
    pub show_theory_modal: bool,
    /// Whether distances, times and rates are shown in physical units - kilometres, seconds and
        /// radians per second, each climbing to whatever scale suits the hole - rather than in the
        /// geometric M the metric is written in.
    ///
    /// On by default, and the checkbox is the *opt-out*: it offers M rather than offering an escape
    /// from it. An M is a perfectly good unit of length and of time once you know that it means
    /// GM/c² and GM/c³, and unreadable before that, so the app opens in units anybody can read
    /// and the GR convention is there for whoever wants it. Speeds are quoted in c in both modes,
    /// because c is the one natural unit that needs no introduction.
    pub use_physical_units: bool,
    pub frame_of_ref: ReferenceFrame,
    /// Whether the view in the foliation column draws the distant clock's own moments: the surfaces
    /// t = const of the chart's Killing time, one line per round unit of the distant clock,
    /// labelled with its offset from the observer's now.
    ///
    /// Default on, and like every control it is untouched by Reset. It is a
    /// view setting and nothing else: the lines are read off `LocalFrame::surface_t_const`, which
    /// takes only the observer's tetrad, so turning it off removes drawing and no physics.
    pub show_distant_clock_grid: bool,
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
            is_playing: false,
            step_mode: StepMode::Time,
            step_size: 0.1,
            play_speed: 1.0,
            step_distance_km: 1000.0,
            achieved_watch_rate: None,
            rays_per_pulse: RAYS_PER_PULSE,
            max_pulses: MAX_PULSES,
            draw_front_arcs: true,
            hide_wound_segments: true,
            show_spatial_details: false,
            // Both are let go the moment the run starts, and the difference between them is the
            // worldline rather than the wait: Alice circles the hole on the prograde ISCO, the
            // thrust-free orbit, while Bob falls through. A trailing delay is still what builds
            // the stack on r₋ that the HUD's "Alice → Bob" line and the Theory Guide describe -
            // put a delay on Bob's card and he cuts through her pulses exactly as those texts say
            // - but it is a configuration to reach for rather than the layout the app opens on.
            alice: ObserverSettings {
                drop_phi: ALICE_DROP_PHI,
                ..ObserverSettings::isco_orbiter(0.0)
            },
            bob: ObserverSettings::raindrop(0.0),
            show_theory_modal: false,
            // Kilometres and seconds. See the field's own note: the checkbox offers M rather
            // than offering a way out of it.
            use_physical_units: true,
            frame_of_ref: ReferenceFrame::DistantObserver,
            show_distant_clock_grid: true,
            font_scale: 1.0,
            view_reset_requested: false,
        }
    }
}

/// The black hole presets, as (label, M, a/M, M_solar). One of them is highlighted when the metric
/// is that preset's, which is read off the metric rather than remembered: nothing can then drift out
/// of step with the geometry, and moving the mass or spin slider drops the highlight by itself.
/// The mass and spin quick-picks, as (label, M, a/M, solar masses, a note shown on hover or "" for
/// none). The note is there for a preset that cannot be what it says it is - see Gargantua.
const PRESETS: [(&str, f64, f64, f64, &str); 7] = [
    ("Schwarzschild (10 M☉)", 1.0, 0.0, 10.0, ""),
    ("Cygnus X-1 (21.2 M☉)", 1.0, 0.97, 21.2, ""),
    ("Sagittarius A* (4.15M M☉)", 1.0, 0.90, 4.15e6, ""),
    ("M87* (6.5B M☉)", 1.0, 0.90, 6.5e9, ""),
    ("TON 618 (66B M☉)", 1.0, 0.88, 6.6e10, ""),
    ("Extreme Kerr (a=0.998)", 1.0, 0.998, 10.0, ""),
    ("Gargantua (100M M☉)", 1.0, 0.999, 1.0e8, GARGANTUA_NOTE),
];

/// The presets drawn on the second row of the two: the widest labels, kept together so the
/// first row is no wider than the panel.
const PRESET_SECOND_ROW: [&str; 2] = ["Sagittarius A* (4.15M M☉)", "Extreme Kerr (a=0.998)"];

/// Why the Gargantua preset is not Gargantua's spin.
const GARGANTUA_NOTE: &str = "Kip Thorne's hole from Interstellar, at the mass he gives it: about 10^8 M☉, which is what puts a survivable tidal field at the horizon of something that swallows a solar system.

Its spin is the part this app cannot carry. Thorne needs a/M = 1 - 1.3e-14 for the hour-per-seven-years on Miller's planet, and at that spin r+ - r- = 2 M sqrt(1 - (a/M)^2) is 3.2e-7 M: the two horizons are closer together than a double-precision integrator can keep them apart over a fall, and the Spin slider stops at 0.999 for that reason. What is set here is that 0.999 - a rapidly rotating hole of the right mass, and the right hole to fall into, but not the one on the screen in the film.

Mallary, Khanna & Burko (Phys. Rev. D 98, 104024) study an infaller at a/M = 0.995 with E = 1, L = 4M for the same reason, and say that closeness to extremality is difficult to simulate.";

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

/// The radius both observers are dropped from out of the box: far enough outside the ergosphere
/// (r_E = 2M on the equator) for a static hover to exist there, close enough in for the whole fall
/// to be a few tens of M. After that each card carries its own `ObserverSettings::drop_r`.
const DROP_RADIUS: f64 = 4.5;

/// The spin of the hole the app opens on, Sagittarius A*'s a/M = 0.90: `SpacetimeApp::default`
/// builds that metric, and Alice's default drop radius is its prograde ISCO.
pub const OPENING_SPIN: f64 = 0.90;

/// The azimuth Alice is dropped at out of the box, a quarter of a radian round from Bob. Two
/// observers at the same (r, phi) would be the same observer; the gap is what makes the pair a
/// pair, and it is the one the release gap and the signal travel time are quoted against. After
/// that each card carries its own `ObserverSettings::drop_phi`.
const ALICE_DROP_PHI: f64 = 0.25;

/// The hover tips on the four Motion buttons of an observer card. Each says what worldline the
/// option is, which 4-velocity it puts at the observer's event, where that worldline exists, what
/// happens where it does not - `Observer::effective_mode`'s fallback - and what the mode is for.
/// The two fallback sentences quote `impossible_mode_note` word for word, so the tip, the note under
/// the buttons and the telemetry box cannot describe the same observer differently.
///
/// They are written about "the observer" rather than about Bob, because both cards show them.
const FREE_FALL_TIP: &str = "A timelike geodesic: the observer falls with no thrust at all and their accelerometer reads exactly zero, which is the whole content of the word. Two conserved quantities pick out which geodesic — the energy per unit mass E = −u_t and the axial angular momentum per unit mass L = u_ϕ, both on the sliders below — and the integrator carries their four-velocity along that curve, so the telemetry, the frame their pulses go out into and the frame that measures their receptions all name the same object as the worldline on screen. E = 1 with L = 0 gives the raindrop, dropped from rest at infinity and falling straight in. A geodesic exists at every radius, and free fall is the only mode that does: the observer crosses the ergosphere, the outer horizon r₊ and the Cauchy horizon r₋ in finite proper time with nothing local happening at any of the three, and for the equatorial L = 0 case the fall ends on the ring, where the curvature really does diverge and the chart stops. Give the observer enough prograde angular momentum and they freeze onto r₋ instead, their proper time reaching a finite limit while the coordinate clock runs on. The light cones and both transmissions read most naturally in this mode, because an infaller is the observer the whole interior picture serves.";
const DROP_RADIUS_TIP: &str = "Where this observer starts, and where ⏮ Reset rebuilds them. The slider carries the same number as the position of their marker at t = 0: drag the marker while the clock reads zero and this slider follows; move this slider and the next drop lands there. One drop radius per observer, two ways to say so. While the clock reads zero the change lands at once, since the run has not started and nothing yet contradicts the change; once the clock runs, the slider becomes a standing request like everything else on the card, waiting for the next ⏮ Reset rather than teleporting a run already under way. The radius also sets their energy, since E follows from the release at that radius: released at rest, a drop from further out carries more energy, and E → 1 as the drop radius runs to infinity, which gives the raindrop. The scale runs logarithmic because the interesting range spans the ring at 0.05M and the far field at 30M. Nothing stops you dropping somebody inside a horizon: no observer can rest there, and the card says what happens instead.";
const DROP_AZIMUTH_TIP: &str = "Where round the hole this observer starts, in the chart angle ϕ. This angle is the coordinate the drop radius leaves out, and a marker dragged on the equatorial view sets both at once. By itself the angle changes nothing about the worldline: Kerr is axisymmetric, so ϕ is a cyclic coordinate and rotating an observer leaves every constant alone — E, L, the effective potential and the whole radial problem stay exactly as before. What the angle does change is the *pair*. The difference between the two azimuths says how far apart the two observers stand around the hole, which sets how long light takes to cross between them and from which side; and inside r₊ that difference decides how much of the other's frozen light the crosser actually meets, since each pulse's E − Ω₋L < 0 arc settles onto a band of r₋ and co-rotates there at Ω₋ rather than covering every azimuth. Put the two on opposite sides and Bob crosses somewhere Alice's stack has never reached. The marker does not sit at this angle on screen: the embedding x + iy = (r + ia)e^{iϕ} turns the marker a further atan2(a, r) round — 11° at r = 4.5M, 42° at r = 1M for a = 0.90 — and the line below the sliders reports where the dot actually lands.";
const AT_REST_TIP: &str = "The observer rests at the moment of release: dr/dτ = 0, with the worldline starting exactly on a turning point of the radial potential, R(r) = 0. The release then costs exactly the energy that turning point implies — E = V(r, L), the effective potential at the drop radius, which at 4.5M with L = 0 and a = 0.90 comes to 0.7504 — so the card reports E rather than offering E on a slider, and E moves whenever the drop radius or L moves. Most users mean this release by “dropped”: the run begins when the engines cut. This release is also the only one that joins the hover before without a jump, because the waiting observer holds that same four-velocity under thrust, so nothing in their motion changes at release except the thrust stopping. At rest means at rest in r; with L = 0 in Kerr the hole still carries the observer round at the frame-dragging rate, which makes them the ZAMO. Between the horizons nothing can hold a radius at all, and the release falls back to the raindrop.";
const CIRCULAR_ORBIT_TIP: &str = "The circular geodesic at the drop radius, prograde in the sense of the hole's spin or retrograde against the spin: the one orbit that needs no thrust at all. The radius fixes both constants once you choose the sense (Bardeen, Press and Teukolsky 1972), so the card reports E and L and greys the L slider out. Outside the innermost stable circular orbit the orbit stays stable; between the ISCO and the circular photon orbit the orbit exists but runs unstable, and the integrator's own rounding eventually tips the observer in or out, which is the honest picture of an unstable orbit; inside the photon orbit no circular orbit exists at any energy and the release falls back to the raindrop. An orbit inside the ergosphere remains perfectly realizable - an orbiting observer co-rotates, and nothing asks that observer to stand still - and at high spin the prograde ISCO lies in there. A Release Delay in front of a circular release makes a jump, exactly as for the raindrop: no observer hovers and then orbits without a kick.";
const ISCO_TIP: &str = "Put the drop radius on the innermost stable circular orbit of that sense and select the circular release: 6M with no spin, 1M prograde and 9M retrograde at the extreme spin, and in between whatever the formula says. The single most-quoted orbit in black hole physics, in one click.";

const FROM_INFINITY_TIP: &str = "The observer arrives having fallen from rest infinitely far away: E = 1 exactly, whatever radius the card drops them at, which means they already move when the run starts. At 4.5M that means two thirds of the speed of light inward past a static observer — nothing accelerated the observer to that speed, and the figure states what the initial condition says about their history. With L = 0 this release gives the raindrop, a member of the E = 1 congruence that every wavefront colour and every measured shift in the app quotes against. The price: a Release Delay in front of this release is a fiction, since no observer hovers and then moves at 0.667c without an infinite acceleration, so the release makes a genuine discontinuity in the worldline — the honest statement that this observer did not come from here. Choose At rest instead if you want the wait and the fall to join.";
const ANGULAR_MOMENTUM_TIP: &str = "The conserved angular momentum per unit mass, L = u_ϕ, in units of M. This constant is the one the card sets directly, because the app's central result speaks in L: the sign of E − Ω₋L decides which branch of the inner horizon an infaller reaches, with Ω₋ = a/(r₋²+a²) = 0.798/M at a = 0.90. Released at rest from 4.5M, the crossover sits at L = 0.985 — below that value the observer crosses the near branch of r₋ at finite coordinate time, and above that value the observer settles onto the far branch, where t → ∞ and their own clock reaches r₋ in finite proper time while the outside universe's whole future arrives at once. Walk the slider across that value and the picture changes character. L also decides whether the observer falls at all: from rest, enough L and the centrifugal barrier throws them outward instead, and past about L = 4 at 4.5M the energy that costs exceeds 1 and they escape to infinity. Prograde runs positive, retrograde negative, and the two are not mirror images around a spinning hole.";
const STATIC_TIP: &str = "The observer hovers: fixed r and fixed ϕ, station-keeping against the distant stars, with a four-velocity along the time-translation Killing vector ∂/∂t normalised to unit length. The thrust that costs is real, the telemetry reports the thrust as a_prop, and the figure grows without bound as the observer nears the static limit. That worldline exists only where ∂/∂t stays timelike, g_tt < 0, which on the equator means r > 2M — outside the ergosphere, not merely outside the horizon. Inside the ergosphere the frame dragging is total: holding ϕ fixed becomes a spacelike motion there, and no rocket, however powerful, manages the feat. The app keeps the selection rather than refusing the selection, because a mode choice is a standing request and resumes by itself the moment the observer reaches somewhere the mode can exist again. Meanwhile the observer falls freely — in position as much as in velocity — and this panel and their telemetry box both read “Static impossible here (r ≤ 2M): falling freely” while that lasts. Static is the mode for the exterior: gravitational blueshift, the redshift of an infaller's signal and the weight of the hole all state what a static observer measures.";
const ZAMO_TIP: &str = "The zero-angular-momentum observer, the frame in which a spinning hole looks as unrotating as a spinning hole can. A ZAMO holds a radius like the static observer but does not fight the frame dragging: the hole sweeps a ZAMO around at the local dragging rate ω = −g_tϕ/g_ϕϕ, exactly fast enough to cancel their own angular momentum L = u_ϕ, which gives the four-velocity γ(1, 0, ω). Light leaves a ZAMO with no built-in swirl, and the lapse α belongs to this observer. A fixed-r worldline stays timelike only outside the outer horizon r₊, so unlike the static observer a ZAMO survives the whole ergosphere — going along with the dragging is precisely what the static observer cannot afford to stop doing. At r₊ and inside, the radial direction turns timelike and nothing holds a radius at all; the app keeps the selection, the observer falls freely instead, and this panel and their telemetry box both read “ZAMO impossible inside r₊: falling freely” until the observer returns outside. Reach for this mode to read the ergosphere, where a ZAMO is the only hovering observer there is.";

/// The hover tip on the Wavefront points slider.
const WAVEFRONT_POINTS_TIP: &str = "How finely a pulse samples the emitter's light cone: n directions at α = 2πi/n, spaced 360/n degrees apart — 2.5° at the default of 144 — with α = 0, the emitter's own outward radial leg, always first whatever n is. Each direction is one exact null geodesic, so this count sets the resolution of the whole picture the light draws: more points give finer tongues where the ring swallows the front, more beads along the frozen arcs stacked on r₋, shorter segments around the loop on the equatorial view, and rarer handovers from one sheet of a front to the next in the reception test, since neighbouring rays then sit closer together in azimuth. Points cost. Integrating the rays, testing the rays against the receiver's worldline and drawing every one of them all scale linearly in the count: about 1.1 ms of frame time for each extra 72 rays a pulse with forty pulses in flight, of which the integration and the reception test take 0.3 ms, so 1024 points costs about seven times the work of 144 and the play loop feels the difference first. The count applies to pulses sent from now on. Light already in flight is the geodesics the app launched, and each pulse keeps the count that pulse went out with, so the slider changes the transmission rather than redrawing the transmission.";

/// The hover tip on the Wavefronts kept slider.
const WAVEFRONTS_KEPT_TIP: &str = "How many wavefronts each transmission holds at once. Past this count the field drops the oldest, so the count sets the length of the history the picture keeps. At the default of 64 a single infall never loses a pulse - a whole fall from r = 4.5M sends about forty pulses at the emission interval of 0.1M of the emitter's proper time - while a hovering emitter, who transmits for as long as the wait lasts, runs past 64 and draws the oldest arcs from light sent long before the release. Turn the count down to read one front at a time, or to watch a single pulse break on r₋ with no sixty others stacked over that pulse; turn the count up to see the whole stack a long transmission builds against the Cauchy horizon.

Together with the points slider above, this count is the other half of what a frame costs: the integration, the reception test and the drawing all scale as the product of the two, so 128 fronts at 1024 points carries 131 k exact null geodesics every step, against 9 k at the pair of defaults. The 2D+1 volume view couples to the count as well - every eighth pulse by serial number carries a swept surface, so the cap fixes how many of those sheets can fly at once, eight at the default and sixteen at the top.

Two things to know before moving this slider. Lowering the count takes effect at once, on the next frame, playing or paused. And the drop is permanent: an evicted pulse is gone from the field, and ⏪ Step Back reintegrates the rays the field still holds rather than re-emitting the pulses the field let go, so raising the count again widens the window from here on rather than restoring anything.

No cap can erase what the transmission measured. An arrival is an event that happened, and the field keeps the receptions list and the last delivery outside the pulses, so even at a cap of one front the HUD's arrival lines and measured shifts match what a cap of 128 gives. The setting survives ⏮ Reset, as every control on this panel does.";

/// The hover tip on the units checkbox.
const M_CHECKBOX_TIP: &str = "Whether every distance and time in the app reads in M, the geometric unit that the Kerr metric actually uses, instead of in kilometres and seconds. Unticked — the default — radii read in km, AU or light-years, times read in µs, seconds, days or years, each scale chosen to suit the hole the mass slider names, and angular rates read in radians per second. Ticked, all of them read in M.

One M is two units at once, and that doubling is the whole reason the convention exists: as a length, M is GM/c², and as a time, M is GM/c³ — the two lines printed under this box for the hole you have dialled up. Setting both to 1 makes the equations readable. The outer horizon sits at r₊ = M + √(M² − a²) whatever the mass, the ISCO of a non-spinning hole sits at 6M, and light covers 1 M of distance in 1 M of time, so a 45° line on the (t, r) diagram is a light ray. Kilometres and seconds hide all of that, because every one of those numbers scales with the mass; in M, a stellar-mass hole and a supermassive one draw the same picture. Hence an offer rather than an imposition: M is the right unit for reading the geometry and the wrong unit for knowing how far away anything lies.

Speeds read in c either way. Two readings keep a foot in both camps whichever way you set the box: the E and L of a free-faller's geodesic stay in M, because those two are the constants of the motion the sliders set and the L slider carries M in the slider's own label, and the horizon list under BLACK HOLE PROPERTIES prints each radius both ways, so the correspondence stays on screen somewhere.";

/// The hover tip on the Arcs between wavefront points checkbox.
const FRONT_ARCS_TIP: &str = "Whether the app draws the pieces of a wavefront between neighbouring rays. Ticked, each piece is the curve linear in (r, ϕ) from one ray to the next, cut into steps of at most 0.05 rad, with each step put through the embedding x + iy = (r + ia)e^{iϕ} — so a piece joining two rays sitting on r₋ comes out as an arc of the r₋ circle, and a piece joining two rays a quarter of a turn apart comes out going round. Unticked, nothing appears between the rays: the front shows as the calculated points themselves, one dot per ray in the same gain colour the arc would have carried, and the frozen family keeps the heavier beads.

No physics turns on this checkbox. The reception test interpolates in (r, ϕ) along exactly the same pieces either way, so an arrival happens at the same event, at the same measured shift, in both settings. Ticking buys agreement between the front you are looking at and the curve the detector tests. Unticking buys the raw output of the integrator with no interpolation laid over that output, which is worth seeing, because everything the arcs add is inference. Inside r₋ the annulus runs thin — at a = 0.90 the embedding puts r₋ at ρ = 1.06 against the ring at ρ = 0.90 — and neighbouring rays wind at wildly different rates, dϕ/dt running from about −5 per M near the ring to +0.8 for a ray settling onto r₋, so a pair of neighbours ends up most of a radian apart and the arc between that pair follows a curve no ray ever travelled. The dots are the part free of inference.

This checkbox works independently of the winding cut below, which drops the two or so segments per pulse whose rays have wound more than a whole turn apart; with the arcs unticked that cut finds nothing left to drop, since every live ray already shows as a dot. The setting is a view setting and survives ⏮ Reset, as every control on this panel does.";

/// The hover tip on the Hide segments wound past a full turn checkbox.
const HIDE_WOUND_TIP: &str = "Whether the app draws the pieces of a front whose two rays have wound more than one whole turn apart in ϕ. Such a pair is no ordinary neighbouring pair that has drifted: the pair straddles a critical impact parameter, the emission angle either side of which the hole captures a ray rather than releasing the ray. At a = 0.90 the prograde equatorial photon orbit sits at r_ph = 1.56M, just outside r₊ = 1.44M, and the retrograde one at 3.91M. The ray just inside the critical angle hangs on that unstable orbit for tens of M, then spirals in and freezes onto r₋, co-rotating at Ω₋ = 0.9 per M for ever; the ray just outside hangs there too, then escapes outward at very nearly c.

The photon orbit pins the real front between those two rays: a spiral inward from the far ray down to r_ph, a pile-up of turns at r_ph that no sampling of the light cone can resolve, and a spiral from r_ph down to r₋. Two rays 2.5° apart cannot carry that shape, and the drawing holds no information from which to invent the shape. What the interpolation linear in (r, ϕ) draws instead is an Archimedean spiral with the whole winding spread evenly over every radius between the two ends, r₊ included, so as the outer ray runs away while the winding grows only at Ω₋, the turns drift steadily outward across the outer horizon — a fan of spiral arms crossing r₊ that no ray ever took and nothing in the spacetime does.

That picture infers what a drawing should not infer, so ticked, the app omits those segments and marks each of the two rays with a dot in that ray's own gain colour, which reads as a gap with marked ends rather than as a silent hole. One whole turn sets the threshold because past 2π the pair has gone round the hole relative to one another and the segment covers every azimuth, leaving the sampling no information at all about the front in there; below 2π the arc still misplaces the winding but remains one arc between two neighbours of the same sheet. The cut removes about two segments per pulse — the prograde critical angle and the retrograde one, one segment each — and nothing else: every other segment appears exactly as before.

No physics turns on this checkbox. Reception detection runs unchanged: Pulse::scan interpolates in (r, ϕ) along every segment whether or not the app draws that segment, so an arrival happens at the same event, at the same measured shift, ticked or unticked. Unticking puts the spirals back, which is worth doing, because those spirals are the honest picture of what two samples that far apart actually say. The setting is a view setting and survives ⏮ Reset, as every control on this panel does.";

/// The hover tip on the Enable Observer checkbox, the same on both cards.
pub const DISTANT_CLOCK_GRID_TIP: &str = "Whether the rest-frame view draws the distant clock's own moments. The chart's time t is a Killing time: a difference of t along any static worldline equals exactly the proper time a clock at rest at infinity records between the same two moments, so the surfaces t = const are that far-away clock's tick marks, carried inward. Ticked, the app draws those surfaces as a muted grid across the observer's local frame, one line per round unit of distant time — 50 µs, 200 ms, 30 min, 5e6 yr — each line labelled with an offset from the observer's now, and the legend names the unit.

The observer's u^t and the pixel scale of the view choose that unit, and nothing else does, so as the outside clock runs faster and faster on their screen the grid climbs the ladder from milliseconds through seconds to years rather than collapsing into a solid block. Where each line meets the worldline the app stays exact rather than linearised: consecutive lines Δt apart cross the worldline Δt/u^t of the observer's own proper time apart, which is the whole content of the statement that the distant clock runs fast by u^t.

Every one of these lines lies flatter than 45°, in every region, because dt stays timelike everywhere in this chart (g^tt = −(1 + 2M/r) < 0). So unlike a surface r = const, these lines never turn null at a horizon, and the grid reads the same way on both sides of r₊. Going through r₊ on the raindrop, or crossing the near branch of r₋, u^t stays finite and the spacing barely moves. Aim instead at the far branch of r₋ — E − Ω₋L < 0, where E = 1 with L = 2.2 goes at a = 0.90 — and u^t grows like exp(κ₋t) while the lines pile up on the worldline without limit: the observer crosses infinitely many of the distant clock's moments in a finite amount of their own time, and the grid shows that as the lines bunching against the origin.

What the grid does not show is anything the observer sees. A slice of constant t is a simultaneity convention, a choice of which far-away events to call “now”, and no measurement singles that choice out. What an observer sees is light, and the ingoing blueshift ν_in/ν_∞ in the telemetry box diverges on that same approach at the same rate — near the far branch, u^t times r₋²/(r₋² + a²). The lines label the geometry; the blueshift is the observation. This setting is a view setting and survives ⏮ Reset, as every control on this panel does.";

/// The hover tip on the Global Foliation Chart 2D+1 item of the View selector.
/// The hover text of the View selector's caption: what the four choices are, and the one
/// distinction that orders them.
pub const VIEW_TIP: &str = "Which picture of the spacetime the left column draws. The first two choices chart the global Kerr-Schild foliation - the same picture for everybody, once as a (t, r) diagram and once as a 2D+1 volume - and the last two draw one observer's own frame of reference, as a (t, r) diagram about their worldline. A chart places every event where the coordinates put that event and claims nothing about distance; a frame of reference stays exact at the observer's own event and linearises away from that event. Hence no 2D+1 frame of reference: at the boosts of a late fall, the region such a frame can speak for is smaller than the picture.";

pub const GLOBAL_VOLUME_TIP: &str ="The same global Kerr-Schild foliation the item above names, drawn as a volume rather than as a (t, r) diagram: the equatorial plane laid out as a floor at the present, with ingoing Kerr-Schild time standing up out of the floor, so a worldline becomes a curve rising through the picture and the horizons become pipes of constant r. The cone at each observer’s event comes from the exact null generators of the metric there rather than from a 45° stencil, which is what lets the eye watch the cones tip over as the observers fall — the thing the flat diagram can only say in words.

This view is a chart, not a frame of reference: like the foliation above, the view shows the same picture for everybody, and the app never draws the view in anybody's rest frame. Drag to pan, shift-drag to orbit, the wheel to zoom, ctrl-wheel to zoom twenty notches at a time, shift-wheel for the vertical time scale, and right-click for the menu of camera presets and of who to keep centred.";

const ENABLE_TIP: &str = "Whether this observer joins the simulation at all. Unticked, the app does not merely hide the observer: no worldline steps, nothing of the observer appears in either view, no telemetry box opens, no light goes out, no arrival comes in, and the app drops their transmission. The other observer's transmission carries on exactly as before — light already in flight does not care whether anybody remains to hear the light — but records no reception, because nobody stands there to make one. Ticking the box back on drops this observer afresh from this card, at r = 4.5M on the clock's current reading, hovering there until their own Release Delay passes. The rest of the run carries on untouched, so the other observer keeps going rather than restarting.";

/// The hover tip on the Transmit Signal checkbox of Alice's card. What her transmission is, and
/// what it does at the Cauchy horizon: the frozen arc on r₋ that Bob later cuts through.
const ALICE_SIGNAL_TIP: &str = "Alice broadcasts a pulse into the whole of her own light cone every 0.1 M of her proper time, and every ray of that pulse is an exact null geodesic of the coded metric.

Colour on the equatorial view is the gain each piece of the front has picked up since release: the frequency a local raindrop measures on that piece here, divided by the frequency the raindrop passing Alice measured as the light left her. Both observers are drops of the same E = 1, L = 0 congruence, the one family that exists at every radius, so the number is an ordinary measured shift between two of them along the ray, and the number carries the same meaning everywhere in the picture. At emission the gain equals exactly 1 for every ray, whatever Alice is doing, so a fresh pulse comes out one uniform red and then works up the ramp as the pulse falls: orange at threefold, yellow at tenfold, green at about thirtyfold, blue at a thousandfold, violet at a hundred thousandfold, draining to a colourless grey on the rare ray that loses frequency instead. Every stop of that ramp sits at the same lightness, so the colour says the shift and only the shift; the earlier ramp ran from a near-black maroon to white, which made a merely brighter stroke read as a stroke that mattered more. Arrivals report a different quantity and keep their own red-white-blue ramp: what a receiver measured against Alice's own emission, which is the question a reception asks.

Inside r₊ the rays that never reach r₋ are the prograde ones, which the hole drags forward in ϕ: the arc of the pulse around α = 90°, running from about 45° to 135° well inside r₊, wider just below r₊ and narrowing as Alice nears r₋, with edges lying exactly where E − Ω₋L changes sign. The equatorial view draws that arc heavier and nearly opaque, and draws the arc last so that nothing paints over the arc — the part of each ring sent prograde enough to carry negative energy along the inner horizon's rotating generator, E − Ω₋L < 0, which never crosses the drawn r₋ circle but piles onto r₋ from outside while co-rotating at Ω₋, while the rest of the ring crosses at finite time. The frozen arc covers about a third of the ring for a pulse sent just inside r₊ and only a sliver for a pulse sent close to r₋, and dots bead the arc because the arc collapses onto r₋ faster than a pixel can show. Those arcs stack up against the Cauchy horizon while the rest of each pulse falls through, and because consecutive arcs overlap there, an infaller crossing r₋ where the arcs stand cuts through several sheets in a row, each blueshifted on the scale exp(κ₋Δt).

Each loop is one pulse and encloses Alice, since light leaves her isotropically in her own frame, and the dot on the loop marks the emission event on Alice's trail. Inside r₊ the flow carries the whole loop inward, so the loop's outer edge never gets further from the hole than that dot.

On the (t, r) diagram, which cannot draw azimuth at all, a pulse appears as a radial extent: a wedge from the emission event, filled faintly in her amber, carrying the most ingoing ray of the pulse as the lower edge and the outermost ray as the upper edge. The lower edge is the ingoing edge of Alice's own light cone carried forward - the 45° line dr/dt = −1 for a hole with no spin, a little steeper for a hole that spins, and steeper again the deeper the light goes - and inside r₊ that edge runs on to the ring while the upper edge freezes on r₋. So the upper edges of her interior pulses stack up on the Cauchy horizon, where in this chart the outgoing light of the whole interior accumulates, and that stack is what a later infaller cuts through. A worldline inside a wedge sits in range of that pulse rather than receiving the pulse: the diagram cannot say whether the ray standing at that radius sits at the receiver's azimuth. The dots on a worldline are the actual receptions, and those dots are the only marks of one.";

/// The hover tip on the Transmit Signal checkbox of Bob's card. The return path, which is not the
/// mirror image of Alice's: it has an end.
const BOB_SIGNAL_TIP: &str = "Bob broadcasts exactly as Alice does, a whole light cone of exact null geodesics every 0.1 M of his own proper time, and he starts at t = 0, before his release: while he waits he is the static observer at his hover radius, with a clock ticking at √(−g_tt) of coordinate time and an orthonormal frame to broadcast into, and nothing in the geometry stops him transmitting from there. His pulses come every 0.134 M of coordinate time while he hovers at r = 4.5M, and every 0.1 M of his own once he falls.

His colours mean what Alice's mean: on the equatorial view, the gain between two raindrops along each ray since that ray left him, so his fronts are born the same uniform red as hers and climb the same ramp. The app draws his fronts at half stroke width and his emission dots in his own mint, so a viewer can tell the two transmissions apart without touching the colouring, which is a measurement.

What differs is the physics of the return path, and which way that path runs depends on which observer sits deeper. In the layout the app opens on, Alice circles the hole on the prograde ISCO while Bob falls past her, so his light has to climb: the shift she measures starts as a small blueshift, because his fall toward the light beats his recession from her, and turns over into a redshift as he drops away below her.

The climb has a limit. The last pulse of his that can reach her at all is one he sends just outside r₊. The outgoing edge of a pulse sent exactly on the horizon stays on the horizon for ever, and every ray of a pulse sent inside r₊ falls, so from his crossing onward he transmits to nobody.

Put a Release Delay on his card instead and he trails her down the same infall. His pulses then chase her inward, and the only part of each pulse that ever catches her is the ingoing part of his cone, which runs at up to dr/dt = −1 in this chart, a rate no timelike worldline can match. That light keeps a finite shift on the branch of r₋ she actually crosses, so unlike Alice → Bob no stack forms for her to cut through. His frozen family, E − Ω₋L < 0, does pile onto r₋ from outside, but that family settles there behind her, after she has already gone through, so she never meets the family.

Where her worldline ends — on the ring, or frozen on r₋ — his transmission stops arriving for that reason instead: one last pulse of his reached her, and the emission event of that pulse is the boundary, on his own worldline, of the causal past of the end of hers. Neither view marks that event; the HUD names the event once her worldline has finished, giving the pulse, when and where he sent the pulse, and how many later ones never arrive.

On the (t, r) diagram the app draws his pulses exactly as hers, each as the wedge of that pulse's own radial extent but in his mint: lower edge the most ingoing ray, upper edge the outermost, a worldline inside the wedge in range of the pulse rather than receiving the pulse, and the dots the actual arrivals.";

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
        .find(|&&(_, m, a_star, m_solar, _)| {
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
    transmit_tip: &'static str,
}

const ALICE_CARD: ObserverCard = ObserverCard {
    name: "Alice",
    colour: Theme::ALICE_COLOR,
    transmit_tip: ALICE_SIGNAL_TIP,
};

const BOB_CARD: ObserverCard = ObserverCard {
    name: "Bob",
    colour: Theme::BOB_COLOR,
    transmit_tip: BOB_SIGNAL_TIP,
};

impl ObserverCard {
    /// Draw this observer's card and apply what it says.
    ///
    /// Three of them act on the spot rather than at the next drop, and all three are enforced here
    /// as a state of affairs rather than as an edge: an unticked Enable means the observer *is*
    /// None every frame, an unticked Transmit means their field *is* empty every frame, and while
    /// the clock reads zero the observer *is* the one the card describes every frame - see
    /// `describes`. That way a test, or a keybinding, that writes the field directly gets the same
    /// simulation as a user moving the slider, which an edge-triggered `Response::changed` would
    /// not: it would answer the click and ignore the write.
    #[allow(clippy::too_many_arguments)]
    fn show(
        &self,
        ui: &mut egui::Ui,
        metric: &KerrSchild,
        settings: &mut ObserverSettings,
        obs: &mut Option<Observer>,
        field: &mut SignalField,
        current_time: f64,
        use_physical_units: bool,
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
                for (mode, label, tip) in [
                    (ObserverMode::FreeFall, "Free Fall", FREE_FALL_TIP),
                    (ObserverMode::Static, "Static", STATIC_TIP),
                    (ObserverMode::Zamo, "ZAMO", ZAMO_TIP),
                ] {
                    if chip(ui, obs.mode == mode, label).on_hover_text(tip).clicked() {
                        obs.mode = mode;
                    }
                }
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

            ui.add(
                egui::Slider::new(&mut settings.delta_t_delay, 0.0..=30.0)
                    .text("Release Delay Δt"),
            )
            .on_hover_text(
                "How long after the drop this observer is let go. Until then they hold the drop radius on the worldline they are about to fall on — a real worldline, under thrust, with a clock of its own and a frame to transmit from — and the release is the moment that thrust stops. The wait is what puts one observer behind the other on the same infall. Released at rest, nothing in their motion changes at the release except the thrust: the hover and the fall are the same four-velocity. Released from rest at infinity there is nothing to hold, since that worldline is already moving in r, so they wait as a static observer and the release is a jump. It takes effect at the next ⏮ Reset, or at once while the clock reads zero, since a release time is part of a worldline rather than something that can be changed under one.",
            );
            // Where they are dropped from. The same number a drag at t = 0 sets, and the same
            // number Reset builds them at, so the slider and the marker are two ways to say one
            // thing. It is a standing request like the rest of the card: it takes effect at the
            // next drop, which is why moving it does not teleport a run already under way.
            if use_physical_units {
                let mut r_km = metric.r_to_km(settings.drop_r);
                let min_km = metric.r_to_km(0.05);
                let max_km = metric.r_to_km(30.0);
                if ui
                    .add(
                        egui::Slider::new(&mut r_km, min_km..=max_km)
                            .logarithmic(true)
                            .text("Drop radius r (km)"),
                    )
                    .on_hover_text(DROP_RADIUS_TIP)
                    .changed()
                {
                    settings.drop_r = metric.km_to_r(r_km);
                }
            } else {
                ui.add(
                    egui::Slider::new(&mut settings.drop_r, 0.05..=30.0)
                        .logarithmic(true)
                        .text("Drop radius r (M)"),
                )
                .on_hover_text(DROP_RADIUS_TIP);
            }

            // And at what azimuth. In degrees, because nobody thinks in radians, and free to run
            // the whole turn: the pair can be put on opposite sides of the hole.
            let mut degrees = settings.drop_phi.to_degrees();
            if ui
                .add(
                    egui::Slider::new(&mut degrees, -180.0..=180.0)
                        .suffix("°")
                        .text("Drop azimuth ϕ"),
                )
                .on_hover_text(DROP_AZIMUTH_TIP)
                .changed()
            {
                settings.drop_phi = degrees.to_radians();
            }

            // How they are let go of, which is what fixes E - and, for a circular orbit, L too.
            ui.horizontal(|ui| {
                ui.label("Release:");
                for (release, label, tip) in [
                    (Release::AtRest, "At rest here", AT_REST_TIP),
                    (Release::FromInfinity, "From rest at ∞", FROM_INFINITY_TIP),
                ] {
                    if chip(ui, settings.release == release, label).on_hover_text(tip).clicked() {
                        settings.release = release;
                    }
                }
            });
            // An orbit is a free-fall worldline, so asking for one puts the Motion on free fall
            // as well: a circular release held on the static or ZAMO worldline would be a request
            // the card could not honour. Four chips in one category: the circular orbit of either
            // sense at the drop radius as it stands, and the innermost stable one of either sense,
            // which is the same release with the drop radius moved onto the ISCO. One of the four
            // is lit at a time: an ISCO chip while the radius is on its ISCO, the plain circular
            // chip otherwise, so nudging the radius off the ISCO passes the light across.
            ui.horizontal_wrapped(|ui| {
                ui.label("Orbit:");
                for (prograde, at_isco, label) in [
                    (true, false, "Circular, prograde"),
                    (false, false, "Circular, retrograde"),
                    (true, true, "ISCO, prograde"),
                    (false, true, "ISCO, retrograde"),
                ] {
                    let release = if prograde {
                        Release::CircularPrograde
                    } else {
                        Release::CircularRetrograde
                    };
                    let on_isco = (settings.drop_r - metric.isco(prograde)).abs() < 1e-9;
                    let selected = settings.release == release && on_isco == at_isco;
                    let tip = if at_isco { ISCO_TIP } else { CIRCULAR_ORBIT_TIP };
                    if chip(ui, selected, label).on_hover_text(tip).clicked() {
                        if at_isco {
                            settings.drop_r = metric.isco(prograde);
                        }
                        settings.release = release;
                        settings.mode = ObserverMode::FreeFall;
                    }
                }
            });

            let circular = settings.release.circular_sense();
            // On a circular orbit L is the orbit's, not the slider's: the greyed slider shows the
            // orbit's value, and the card's own L is kept untouched behind it for when the
            // release is changed back.
            let mut shown = match circular {
                Some(_) => settings.worldline_params(metric).l_ang.clamp(-4.0, 4.0),
                None => settings.l_ang,
            };
            ui.add_enabled(
                circular.is_none(),
                egui::Slider::new(&mut shown, -4.0..=4.0)
                    .text("Angular momentum L (per unit mass, M)"),
            )
            .on_hover_text(ANGULAR_MOMENTUM_TIP);
            if circular.is_none() {
                settings.l_ang = shown;
            }

            // What the card has actually asked for, in the two numbers the physics uses and the
            // one a user can picture. E is derived, so it is reported rather than dialled, and the
            // speed is what a static observer at the drop radius would clock them at as they pass:
            // gamma = E / sqrt(-g_tt) against that observer, so v = sqrt(1 - 1/gamma^2).
            let params = settings.worldline_params(metric);
            let g_tt = metric.metric_components(settings.drop_r)[0][0];
            let speed = if g_tt < 0.0 {
                let gamma = params.energy / (-g_tt).sqrt();
                (1.0 - 1.0 / (gamma * gamma).max(1.0)).max(0.0).sqrt()
            } else {
                f64::NAN
            };
            let bound = if params.energy < 1.0 { "bound" } else { "unbound" };
            ui.label(
                egui::RichText::new(if speed.is_finite() {
                    format!(
                        "E = {:.4} ({bound}), starting at {:.3}c past a static observer there",
                        params.energy, speed
                    )
                } else {
                    format!("E = {:.4} ({bound})", params.energy)
                })
                .small()
                .color(Theme::TEXT_MUTED),
            );
            // The orbit's own numbers: its L, its period on both clocks, and whether it is stable.
            // The stability is read off the ISCO, and the existence off the photon orbit; where
            // there is no orbit the release fell back to the raindrop and this says so.
            if let Some(prograde) = circular {
                let r = settings.drop_r;
                let (photon, isco) = (metric.photon_orbit(prograde), metric.isco(prograde));
                let text = match (
                    metric.circular_orbit(r, prograde),
                    metric.orbital_angular_velocity(r, prograde),
                    metric.circular_orbit_dilation(r, prograde),
                ) {
                    (Some((_, l_ang)), Some(omega), Some(dilation)) => {
                        let period = std::f64::consts::TAU / omega.abs();
                        let fmt = |m: f64| {
                            if use_physical_units { metric.format_physical_time(m) } else { format!("{m:.2} M") }
                        };
                        let stability = if r >= isco {
                            format!("stable (ISCO at {:.3} M)", isco)
                        } else {
                            format!(
                                "UNSTABLE: between the photon orbit ({:.3} M) and the ISCO ({:.3} M)",
                                photon, isco
                            )
                        };
                        format!(
                            "L = {l_ang:.4} M — one orbit takes {} on the distant clock, {} on their \
                             own — {stability}",
                            fmt(period),
                            fmt(period / dilation),
                        )
                    }
                    _ => format!(
                        "No circular orbit inside the photon orbit ({:.3} M): released as a \
                         raindrop (E = 1, L = 0)",
                        photon
                    ),
                };
                ui.label(egui::RichText::new(text).small().color(Theme::TEXT_MUTED));
            }
            // Where that lands on the equatorial view, which is not where the naive polar reading
            // of (r, phi) would put it: the embedding x + iy = (r + ia)e^{i phi} turns the marker a
            // further atan2(a, r) round and draws it at radius sqrt(r^2 + a^2). The card says the
            // chart coordinates, which are what the physics is stated in, and this says the dot.
            let (x, y) = metric.cartesian_position(settings.drop_r, settings.drop_phi);
            ui.label(
                egui::RichText::new(if use_physical_units {
                    format!(
                        "Drawn at x = {}, y = {} — drag the marker on the equatorial view to move it",
                        metric.format_km(metric.r_to_km(x)),
                        metric.format_km(metric.r_to_km(y))
                    )
                } else {
                    format!(
                        "Drawn at x = {x:.2}M, y = {y:.2}M — drag the marker on the equatorial \
                         view to move it"
                    )
                })
                .small()
                .color(Theme::TEXT_MUTED),
            );
            // The one place "at rest" has no meaning: between the horizons r is timelike and
            // nothing holds a radius, so the release falls back to the raindrop. `release_energy`
            // makes that decision; this says it out loud where the user made the request.
            if settings.release == Release::AtRest
                && GeodesicState::energy_floor(metric, settings.drop_r, settings.l_ang) <= 0.0
            {
                ui.label(
                    egui::RichText::new(
                        "Nothing can be at rest between the horizons: released as a raindrop (E = 1)",
                    )
                    .small()
                    .color(Theme::TEXT_MUTED),
                );
            }

            // While the run is standing at its start, the card and the observer are the same
            // thing. A drop radius, a release, an L or a delay stated there is a statement about
            // the run that is about to happen, and there is nothing in progress for it to
            // contradict, so it takes effect at once: the marker moves in both views as the slider
            // moves, exactly as dragging the marker moves the slider
            // (`AppControls::remember_drop_positions`). Without this the two disagree and the
            // card loses - `remember_drop_positions` copies the observer's radius back over the
            // slider on the next frame, and the slider springs back to where it was.
            //
            // Once the clock is running they go back to being standing requests, because by then
            // there *is* something to contradict: a worldline with a history, light in flight from
            // it, and arrivals recorded against it. Those wait for the next Reset.
            if current_time == 0.0 && !self.describes(metric, settings, obs, current_time) {
                // The light they had out was emitted by the worldline being replaced.
                field.silence();
                let phi = obs.phi;
                obs.release_t = current_time + settings.delta_t_delay;
                obs.reset_with_phi(
                    metric,
                    current_time,
                    settings.drop_r,
                    phi,
                    settings.worldline_params(metric),
                );
            }
        });
    }

    /// Whether `obs` is the observer this card describes: the radius it names, the release it
    /// names, the angular momentum it names, and let go at the moment its delay says.
    ///
    /// E is not compared, being a function of the other three (`release_energy`), and neither is
    /// anything the observer has picked up since - their Motion, which a re-drop inherits from them
    /// rather than reading off the card, or where they have got to, which is what the run is for.
    fn describes(
        &self,
        metric: &KerrSchild,
        settings: &ObserverSettings,
        obs: &Observer,
        start_t: f64,
    ) -> bool {
        let wanted = settings.worldline_params(metric);
        let carried = obs.geodesic.map_or(wanted.l_ang, |geo| geo.l_ang);
        let same = |a: f64, b: f64| (a - b).abs() <= 1e-9 * (1.0 + a.abs().max(b.abs()));
        // The azimuth is compared as an angle: a drag reads it out of `atan2` in (-pi, pi], and a
        // card can be holding the same direction written as 6.0 rather than -0.28.
        let turn = std::f64::consts::TAU;
        let apart = (obs.phi - settings.drop_phi).rem_euclid(turn);
        same(obs.r, settings.drop_r)
            && apart.min(turn - apart) <= 1e-9

            && same(carried, wanted.l_ang)
            && same(obs.release_t, start_t + settings.delta_t_delay)
            && obs.release == wanted.release
    }

    /// This observer as the card asks for them, dropped at the clock reading `start_t`.
    fn dropped(
        &self,
        metric: &KerrSchild,
        settings: &ObserverSettings,
        start_t: f64,
    ) -> Observer {
        settings
            .dropped(metric, self.name, start_t)
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
        let mut obs = settings.dropped(metric, self.name, start_t)?;
        if let Some(prev) = previous {
            obs.mode = prev.mode;
        }
        Some(obs)
    }
}

/// The four transport buttons - Play/Pause, Reset, Step Back, Step Fwd - are the controls a user
/// reaches for most and the ones they reach for in a hurry, so they are drawn large, with the glyph
/// on the button and the name underneath it rather than crowded onto it.
///
/// `TRANSPORT_BUTTON` is sized so that four of them and the spacing between them fit across the
/// panel, which is 300 points wide, and so that the longest caption - "Step Back" - fits under one.
const TRANSPORT_BUTTON: egui::Vec2 = egui::vec2(60.0, 44.0);

/// One of them. Returns whether it was clicked, so the caller reads exactly as it did when these
/// were `ui.button(..).clicked()`.
fn transport_button(ui: &mut egui::Ui, glyph: &str, caption: &str, tip: &str) -> bool {
    ui.vertical(|ui| {
        ui.set_width(TRANSPORT_BUTTON.x);
        let clicked = ui
            .add_sized(
                TRANSPORT_BUTTON,
                egui::Button::new(egui::RichText::new(glyph).size(22.0)),
            )
            .on_hover_text(tip)
            .clicked();
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new(caption).size(11.0).color(Theme::TEXT_MUTED));
        });
        clicked
    })
    .inner
}

impl AppControls {
    /// Build the run the app opens on, and that ⏮ Reset rebuilds: the clock at zero, both
    /// transmissions dropped, and every ticked observer re-dropped from their card's own
    /// `ObserverSettings::drop_r` - Alice at ϕ = 0.25, Bob at ϕ = 0 - each hovering there until
    /// their own Release Delay has passed, on the worldline their own E, L and outgoing flag pick
    /// out.
    ///
    /// There is one layout rather than two. Reset used to put Bob at r = 3.8M released at t = 0
    /// while a second button, Drop Observers, put him at 4.5M with a delay, so the same run had two
    /// openings and the two buttons disagreed about which; that button is gone and Reset, the
    /// preset row and `SpacetimeApp::default` all come through here.
    ///
    /// The transmissions are cleared rather than rewound. A rewind keeps the light in flight, and
    /// there is none to keep: the wavefronts standing in the field were emitted by worldlines that
    /// this is about to replace, and where the geometry itself has just changed under them they are
    /// null geodesics of a metric that no longer applies.
    ///
    /// Both callers - the transport's Reset button, and the preset row, which restarts the run
    /// because it has changed the hole - also want the (t, r) view put back where it starts, so the
    /// request is raised here rather than at each of them.
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

    /// Take the position of any observer with a hand on them as the position they are dropped
    /// from, if the run is standing at its start. Both coordinates: the equatorial view drags them
    /// about the plane, so a drag says a radius and an azimuth.
    ///
    /// This is the drag's half of one rule: while the clock reads zero, the card and the observer
    /// are the same thing. The card is the one that says so - `ObserverCard::describes` puts the
    /// observer back on it every frame - so a drag has to write the *card*, or the panel would undo
    /// it on the next pass. That is what this does, and it is why it runs before the panel rather
    /// than after it: the pointer's position is folded into the card, and the card is then enforced.
    ///
    /// The test for "is there a hand on them" is `ObserverMode::ManualDrag`, which is the held state
    /// `Observer::set_drag_position` puts them in for exactly as long as the pointer holds them and
    /// which nothing on the panel can select. Without that test this would copy every observer's
    /// radius over their card every frame, and a drop radius typed into the card - by a slider, a
    /// preset or a test - would be overwritten by the position it was trying to change.
    ///
    /// Only the radius. The drag also sets the observer's t, and can release them early through
    /// `Observer::release_from_drag`, but neither is a property of where the run starts: the clock
    /// goes back to zero on a Reset and the release delay is the card's own.
    pub fn remember_drop_positions(
        &mut self,
        alice: Option<&Observer>,
        bob: Option<&Observer>,
        current_time: f64,
    ) {
        if current_time != 0.0 {
            return;
        }
        for (card, observer) in [(&mut self.alice, alice), (&mut self.bob, bob)] {
            if let Some(obs) = observer
                && obs.mode == ObserverMode::ManualDrag
            {
                card.drop_r = obs.r;
                card.drop_phi = obs.phi;
            }
        }
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

    /// What one Watch-mode step of `d_tau` on the focus observer's watch is worth in the
    /// coordinate time everything in the app is integrated against.
    ///
    /// The conversion is exact and is one number: dτ/dt = 1/u^t along their worldline, so a step
    /// of Δτ of their watch is u^t Δτ of the chart's time. The focus observer is whoever the
    /// View selector names, because that is whose rest frame is being drawn and
    /// whose cone is the 45 degree one; the distant observer has no worldline here and their watch
    /// *is* t, so Watch mode is Time mode for them and `dt = d_tau` at u^t = 1. A focus observer
    /// who is not in the simulation, or whose u^t is not a finite positive number, falls back the
    /// same way rather than inventing a dilation.
    ///
    /// `WATCH_DT_CAP` is the one place the honest answer is refused, and it is refused loudly: an
    /// observer frozen on the far branch of r₋ has u^t ~ 1e10, so one tick of their watch is more
    /// of the outside future than any integrator in the app can step through in one call. The
    /// returned `capped` flag and `u_t` are what the panel's readout says so with.
    pub fn watch_step(
        &self,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        d_tau: f64,
    ) -> WatchStep {
        let focus = match self.frame_of_ref {
            ReferenceFrame::DistantObserver | ReferenceFrame::GlobalVolume => None,
            ReferenceFrame::Bob => bob,
            ReferenceFrame::Alice => alice,
        };
        let dilation = focus
            .map(|obs| obs.four_velocity(metric)[0])
            .filter(|u_t| u_t.is_finite() && *u_t > 0.0);
        match dilation {
            Some(u_t) => {
                let asked = u_t * d_tau;
                let dt = asked.min(WATCH_DT_CAP);
                WatchStep { dt, capped: dt < asked, u_t }
            }
            None => WatchStep { dt: d_tau, capped: false, u_t: 1.0 },
        }
    }

    /// The one description of what a step means, in coordinate time: Time mode takes `base` as it
    /// stands, Distance mode ignores it for the time the requested Δr costs, and Watch mode reads
    /// it as proper time on the focus observer's watch.
    ///
    /// `base` is the frame's own `dt × play_speed` for the play loop and the Step Size slider for
    /// the arrow keys and the panel's Step Back / Step Fwd buttons. All three go through here, so
    /// a keypress, a click and a played frame cannot mean different things by a step.
    pub fn step_for(
        &self,
        metric: &KerrSchild,
        bob: Option<&Observer>,
        alice: Option<&Observer>,
        base: f64,
    ) -> f64 {
        match self.step_mode {
            StepMode::Time => base,
            StepMode::Distance => self.distance_step(metric, bob, alice),
            StepMode::Watch => self.watch_step(metric, bob, alice, base).dt,
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

        // 1. Playback & Simulation Transport. The frame of reference used to be chosen here,
        // above it; it is now chosen at the head of the foliation view itself, which is the view
        // the choice redraws. See `SpacetimeApp::update`.
        ui.group(|ui| {
            ui.label(egui::RichText::new("SIMULATION CONTROL").strong().color(Theme::UI_HEADING));
            ui.horizontal(|ui| {
                let (play_glyph, play_caption) =
                    if self.is_playing { ("⏸", "Pause") } else { ("▶", "Play") };
                if transport_button(
                    ui,
                    play_glyph,
                    play_caption,
                    "Toggle Play/Pause simulation (Spacebar)",
                ) {
                    self.is_playing = !self.is_playing;
                }
                if transport_button(
                    ui,
                    "⏮",
                    "Reset",
                    "Put the run back to its start: the clock to zero, both transmissions dropped, and every ticked observer dropped afresh from their card - Alice at ϕ = 0.25 and Bob at ϕ = 0, each from their own drop radius, each hovering there until their own Release Delay, on the worldline their own E and L pick out. Two things are not read off the card. Motion is inherited: an observer being replaced hands their own Motion to the one replacing them, so a Reset never answers a question about how somebody moves that the user has not asked, and a card that has just been ticked on starts as it does out of the box. And the drop radius is wherever that observer was standing the last time the clock read zero, so dragging a marker at the start of a run moves where they are dropped from.",
                ) {
                    self.drop_observers(metric, alice, bob, &mut signals, current_time);
                }
                // The same `step_for` the arrow keys and the play loop ask, so the three paths
                // cannot disagree about what one step is.
                let current_step =
                    self.step_for(metric, bob.as_ref(), alice.as_ref(), self.step_size);
                // How far back the clock can go, and why it can go no further: zero until a
                // run is long enough for a trail to evict its own start, and that trail's oldest
                // event afterwards. See `ObserverPair::rewind_floor`.
                let floor =
                    ObserverPair { bob: bob.as_mut(), alice: alice.as_mut() }.rewind_floor(metric);
                let room = *current_time - floor;
                let back_tip = if room > 1e-9 {
                    "Step back by Step Size / Distance (Left Arrow key)".to_string()
                } else if floor > 0.0 {
                    format!(
                        "The recorded worldlines reach back only to t = {floor:.2} M. Earlier \
                         events have been evicted from the history, so there is nothing to put \
                         the observers back on: use ⏮ Reset to run again from t = 0."
                    )
                } else {
                    "Already at t = 0, the start of the run.".to_string()
                };
                // Disabled rather than silently doing nothing, so that the arrow key and this
                // button agree with each other and with what the clock is about to do.
                let step_back = ui
                    .add_enabled_ui(room > 1e-9, |ui| {
                        transport_button(ui, "←", "Step Back", &back_tip)
                    })
                    .inner;
                if step_back {
                    // Stepped back by however much of the step is left above the floor.
                    let back = current_step.min(room).max(0.0);
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
                if transport_button(
                    ui,
                    "→",
                    "Step Fwd",
                    "Step forward by Step Size / Distance (Right Arrow key)",
                ) {
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
        });

        ui.add_space(4.0);

        // 1b. Everything that is a setting rather than a press. The transport above is the handful
        // of controls a user reaches for constantly; these are the ones they set once and leave, so
        // they get their own frame and their own heading rather than trailing off the same one.
        ui.group(|ui| {
            ui.label(egui::RichText::new("SIMULATION SETTINGS").strong().color(Theme::UI_HEADING));

            ui.add(
                egui::Slider::new(&mut self.play_speed, 0.05..=20.0)
                    .logarithmic(true)
                    .text("Play Speed (M / real second)"),
            );

            ui.horizontal(|ui| {
                ui.label("Step Mode:");
                if chip(ui, self.step_mode == StepMode::Time, "⏱ Time (Δt)").clicked() {
                    self.step_mode = StepMode::Time;
                }
                if chip(ui, self.step_mode == StepMode::Distance, "📏 Distance (Δr)").clicked() {
                    self.step_mode = StepMode::Distance;
                }
                if chip(ui, self.step_mode == StepMode::Watch, "⌚ Watch (Δτ)").clicked() {
                    self.step_mode = StepMode::Watch;
                }
            });

            // What the last played frame actually managed on that watch. Nothing is printed while
            // the run is paused or in another step mode, because there is then no rate to report:
            // see `achieved_watch_rate`.
            if let Some(rate) = self.achieved_watch_rate {
                let capped = rate < 1.0;
                let figure =
                    if rate >= 0.01 { format!("{rate:.2}") } else { format!("{rate:.1e}") };
                let text = if capped {
                    format!(
                        "{}: {figure} s/s  (Δt capped at {WATCH_DT_CAP:.0} M per frame: \
                         one tick of the watch here holds more of the outside future than can be \
                         integrated)",
                        self.frame_of_ref.watch_label()
                    )
                } else {
                    format!("{}: {figure} s/s", self.frame_of_ref.watch_label())
                };
                let colour = if capped { Theme::WARNING_RED } else { Theme::TEXT_MUTED };
                ui.label(egui::RichText::new(text).small().color(colour));
            }

            match self.step_mode {
                StepMode::Time => {
                    // The slider's own number is in M whichever mode the app is in, since that is
                    // the quantity the stepper takes; what the label adds is what the current
                    // setting is worth on the distant clock, which is the part a reader who does
                    // not think in M needs. It used to carry no unit at all. Built before the
                    // slider, which takes a &mut to the field the label reads.
                    let label = if self.use_physical_units {
                        format!("Step Size (Δt = {})", metric.format_physical_time(self.step_size))
                    } else {
                        "Step Size (Δt, in M)".to_string()
                    };
                    ui.add(
                        egui::Slider::new(&mut self.step_size, 0.0005..=0.5)
                            .logarithmic(true)
                            .text(label),
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
                            if chip(ui, active, label).clicked() {
                                self.step_distance_km = km;
                            }
                        }
                    });
                }
                // The same slider as Time mode, read as proper time instead of coordinate time:
                // one number for "how big is a step", whichever clock is being kept.
                StepMode::Watch => {
                    let label = if self.use_physical_units {
                        format!(
                            "Step Size (Δτ = {} on {}'s watch)",
                            metric.format_physical_time(self.step_size),
                            self.frame_of_ref.watch_owner()
                        )
                    } else {
                        "Step Size (Δτ in M, focus watch)".to_string()
                    };
                    ui.add(
                        egui::Slider::new(&mut self.step_size, 0.0005..=0.5)
                            .logarithmic(true)
                            .text(label),
                    )
                    .on_hover_text(format!(
                        "How much proper time one step is worth on {}'s own watch - the observer \
                         the View selector names, whose rest frame the left column \
                         is drawn in. The step the app actually takes is Δt = u^t Δτ of \
                         coordinate time, u^t being their time dilation at the event they are at, \
                         so playback runs at the requested rate on their watch rather than on the \
                         distant clock. {}",
                        self.frame_of_ref.watch_owner(),
                        match self.frame_of_ref {
                            ReferenceFrame::DistantObserver | ReferenceFrame::GlobalVolume =>
                                "The distant observer's watch is the chart's own Killing time t, \
                                 so here Watch mode is Time mode: u^t = 1 and Δt = Δτ.",
                            ReferenceFrame::Bob | ReferenceFrame::Alice =>
                                "Deep in the well one tick of that watch is a great many M of the \
                                 outside future, and on the far branch of r₋ it is about 1e10 of \
                                 them; the step is capped at 2 M per frame and the line under the \
                                 chips says what fraction of the asked-for tick each played frame \
                                 carried.",
                        }
                    ));
                }
            }
            ui.add(
                egui::Slider::new(&mut self.rays_per_pulse, 64..=1024)
                    .integer()
                    .text("Wavefront points"),
            )
            .on_hover_text(WAVEFRONT_POINTS_TIP);
            ui.add(
                egui::Slider::new(&mut self.max_pulses, 1..=128)
                    .integer()
                    .text("Wavefronts kept"),
            )
            .on_hover_text(WAVEFRONTS_KEPT_TIP);
            ui.checkbox(&mut self.draw_front_arcs, "Arcs between wavefront points")
                .on_hover_text(FRONT_ARCS_TIP);
            ui.checkbox(&mut self.hide_wound_segments, "Hide segments wound past a full turn")
                .on_hover_text(HIDE_WOUND_TIP);

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
            ui.label(egui::RichText::new("📐 UNITS & COORDINATE SYSTEM").small().strong().color(Theme::TEXT_BRIGHT));
            // Shown inverted. The stored flag says "physical units", which is what the whole
            // app reads; the checkbox asks the opposite question, because M is the thing a user
            // has to opt into and a box that is ticked out of the box reads as the exception
            // rather than the default.
            let mut use_m = !self.use_physical_units;
            if ui
                .checkbox(&mut use_m, "📐 Show distances and times in M (GR convention)")
                .on_hover_text(M_CHECKBOX_TIP)
                .changed()
            {
                self.use_physical_units = !use_m;
            }
            // One pair of lines whichever unit the charts are labelled in. The two branches this
            // replaces printed the same two numbers under different captions, and only one of them
            // carried the conversions - which are the part that answers what M actually is. They
            // are the same brightness as every other control: this is the key to every number on
            // the screen, not a footnote to them.
            ui.label(egui::RichText::new(format!("• 1M [Distance] = GM/c² = {}", metric.format_physical_distance(1.0))).small())
                .on_hover_text(M_UNITS_TIP);
            ui.label(egui::RichText::new(format!("• 1M [Time]     = GM/c³ = {}", metric.format_physical_time(1.0))).small())
                .on_hover_text(M_UNITS_TIP);
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
            // Two rows rather than one that wraps where it likes: the two widest labels have a
            // row of their own, so the panel stays as narrow as the rest of it.
            let mut preset_changed = false;
            let highlighted = active_preset(metric);
            for second_row in [false, true] {
                ui.horizontal_wrapped(|ui| {
                    for (label, m, a_star, m_solar, note) in PRESETS {
                        if PRESET_SECOND_ROW.contains(&label) != second_row {
                            continue;
                        }
                        let pick = chip(ui, highlighted == Some(label), label);
                        let pick = if note.is_empty() { pick } else { pick.on_hover_text(note) };
                        if pick.clicked() {
                            *metric = KerrSchild::with_solar_mass(m, a_star * m, m_solar);
                            preset_changed = true;
                        }
                    }
                });
            }
            if preset_changed {
                self.drop_observers(metric, alice, bob, &mut signals, current_time);
            }

            ui.separator();
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            let re = metric.ergosphere_equatorial();
            if self.use_physical_units {
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

        // 3. The two observer cards, Bob's first: he is the infaller the interior picture is drawn
        // for - the light cones, the rest-frame view and the stack on r₋ are all his - so his card
        // is the one reached for most often and it sits at the top of the pair. They are the same
        // code twice: see `ObserverCard`.
        BOB_CARD.show(ui, metric, &mut self.bob, bob, signals.bob, *current_time, self.use_physical_units);
        ui.add_space(4.0);
        ALICE_CARD.show(ui, metric, &mut self.alice, alice, signals.alice, *current_time, self.use_physical_units);

        ui.add_space(6.0);

        // 4. Theory Explanations
        if ui.button("📖 Relativistic Theory & Horizons").clicked() {
            self.show_theory_modal = !self.show_theory_modal;
        }
    }
}
