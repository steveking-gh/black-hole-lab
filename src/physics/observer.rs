use std::collections::VecDeque;

use crate::physics::geodesic::{GeodesicState, R_STOP};
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::tetrad::{Tetrad, inner};

/// A local observer that another observer's speed can be quoted against.
///
/// There is no such thing as *the* velocity of a worldline in general relativity: a speed is
/// always a statement about two worldlines crossing at one event, and naming the other one is not
/// a detail but the whole content of the number. The three here are the hovering observers this
/// chart has, in the order of how far in they survive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalRestFrame {
    /// The static observer, at rest with respect to the distant stars: fixed r and fixed phi.
    /// Exists only outside the equatorial static limit r = 2M, and its own Lorentz factor against
    /// anything diverges as that surface is approached, because the frame itself is turning null.
    Static,
    /// The zero-angular-momentum observer, holding r while being swept round at the local dragging
    /// rate. Exists through the whole ergosphere, down to r+, which is as far as any fixed-r
    /// worldline goes.
    Zamo,
    /// The raindrop: the E = 1, L = 0 ingoing geodesic, dropped from rest at infinity. Not a
    /// hovering observer at all, and the only one of the three that exists at every r > 0 - which
    /// is why `ObserverMode::ManualDrag` already defines its boost against this frame.
    Raindrop,
}


/// One observer's motion as a local observer of `frame` actually measures it, off their own ruler
/// and their own clock.
#[derive(Debug, Clone, Copy)]
pub struct LocalSpeed {
    /// Who did the measuring.
    pub frame: LocalRestFrame,
    /// The speed, as a fraction of c. Strictly below 1 for any two timelike worldlines, however
    /// deep in the well the event is and whatever the chart says about coordinate rates.
    pub v: f64,
    /// The Lorentz factor between the two worldlines, gamma = -g(u, u_frame). This is the time
    /// dilation *between the observers*, not the u^t of either of them against the distant clock.
    pub gamma: f64,
}

impl LocalSpeed {
    /// Proper velocity, or celerity: the local observer's ruler distance per unit of the *moving*
    /// observer's proper time, gamma v. It is unbounded - it is what passes c without anything
    /// physical happening, since the two factors belong to different clocks - and it is the
    /// quantity `dr/dtau` is the radial part of.
    pub fn celerity(self) -> f64 {
        self.gamma * self.v
    }
}

/// How an observer's worldline is generated. Every mode pins down a contravariant 4-velocity
/// u^mu at the observer's current event, and all telemetry (coordinate velocity, proper velocity,
/// proper acceleration) is derived from that single object rather than from per-mode formulas.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObserverMode {
    /// Timelike geodesic: ingoing free fall with conserved energy E and angular momentum L.
    /// Exists everywhere in the equatorial plane, including at and inside both horizons, and has
    /// identically zero proper acceleration (that is what "geodesic" means).
    FreeFall,
    /// Worldline positioned by the user's mouse. Its 4-velocity is the boost of the local
    /// *raindrop* frame (the E = 1, L = 0 ingoing geodesic, which exists at every r > 0) by the
    /// local velocity (beta_r, beta_phi). beta = 0 reproduces free fall exactly; any other beta is
    /// a rocket, and carries the proper acceleration that keeping it up requires.
    ManualDrag,
    /// Static observer: fixed r *and* fixed phi, u^mu = (1, 0, 0) / sqrt(-g_tt).
    /// The Killing vector d/dt is timelike only outside the equatorial static limit, so this
    /// observer exists only where g_tt = -(1 - 2M/r) < 0, i.e. r > 2M. Inside the ergosphere
    /// frame dragging makes "holding phi fixed" a spacelike motion, so no rocket can do it.
    Static,
    /// Zero-angular-momentum observer (ZAMO): fixed r, but swept around by frame dragging at
    /// omega(r) = -g_tphi / g_phiphi, so that u_phi = 0. A fixed-r worldline is timelike only where
    /// the r direction is spacelike, i.e. outside the outer horizon r+ (between r+ and r- the
    /// coordinate r is timelike and nothing can hover). Unlike the static observer, the ZAMO
    /// survives the whole ergosphere 2M > r > r+.
    Zamo,
}

/// Where an observer's fall is released from, which is what fixes their energy.
///
/// E is not a thing a user can sensibly dial. It is the *history* of the worldline - E = 1 means
/// "has already fallen from rest at infinity", and an observer dropped at 4.5M with E = 1 starts
/// the run doing two thirds of the speed of light - so asking for it directly makes the initial
/// condition something to be discovered rather than stated. These two are the statements a user
/// actually means, and `release_energy` turns either into the E it implies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Release {
    /// At rest at the radius they are dropped from: dr/dtau = 0, the worldline starting exactly on
    /// a turning point of R(r). This is the run beginning when they cut their engines, and it is
    /// the one release that joins the hover before it without a jump in velocity.
    AtRest,
    /// From rest at infinity: E = 1, the raindrop. They arrive at the drop radius already moving,
    /// which is what "fell from far away" means, and they are then a member of the E = 1,
    /// L = 0 raindrop congruence every shift in the app is quoted against.
    FromInfinity,
    /// On the circular geodesic at the drop radius, in the sense of the hole's spin: both E and
    /// L are fixed by the radius (`KerrSchild::circular_orbit`), and the card's L is ignored. The
    /// only thrust-free orbit there is. Where none exists - inside the photon orbit - the release
    /// falls back to the raindrop, and the card says so.
    CircularPrograde,
    /// The same, against the spin.
    CircularRetrograde,
}

impl Release {
    /// Whether this release is one of the two circular orbits, and if so which sense.
    pub fn circular_sense(self) -> Option<bool> {
        match self {
            Self::CircularPrograde => Some(true),
            Self::CircularRetrograde => Some(false),
            Self::AtRest | Self::FromInfinity => None,
        }
    }
}

/// The conserved energy of a worldline with angular momentum `l_ang` released at radius `r` under
/// `release`.
///
/// `AtRest` is the effective potential V(r, L) - the smallest E for which R(r) >= 0, so R(r) = 0
/// and the worldline starts at a turning point. It does not exist everywhere: between the horizons
/// r is timelike and nothing can be at rest in it, and there `energy_floor` has no root to return.
/// The fall back there is the raindrop, because E = 1 is the one release that means something at
/// every radius, and the card says so where it applies.
pub fn release_energy(metric: &KerrSchild, r: f64, l_ang: f64, release: Release) -> f64 {
    match release {
        Release::FromInfinity => 1.0,
        Release::AtRest => {
            let floor = GeodesicState::energy_floor(metric, r, l_ang);
            if floor > 0.0 { floor } else { 1.0 }
        }
        Release::CircularPrograde | Release::CircularRetrograde => {
            release_constants(metric, r, l_ang, release).0
        }
    }
}

/// (E, L) of a release at `r`. A circular release with no orbit at that radius - inside the
/// photon orbit - is the raindrop, E = 1 and L = 0, which is the one release that means
/// something everywhere.
fn release_constants(metric: &KerrSchild, r: f64, l_ang: f64, release: Release) -> (f64, f64) {
    match release.circular_sense() {
        Some(prograde) => metric.circular_orbit(r, prograde).unwrap_or((1.0, 0.0)),
        None => (release_energy(metric, r, l_ang, release), l_ang),
    }
}

/// The constants of motion that define an observer's free-fall worldline: the conserved energy
/// per unit mass E = -u_t, the conserved axial angular momentum per unit mass L = u_phi (in units
/// of M), and which root of r^4 (dr/dtau)^2 = R(r) the worldline starts on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldlineParams {
    pub energy: f64,
    pub l_ang: f64,
    /// Start on the outgoing root. Only meaningful where Delta > 0; see
    /// `GeodesicState::new_with_direction` for why the ingoing chart refuses it on a horizon and
    /// between the horizons. No control sets it any more - a release is at rest or from infinity,
    /// and at a turning point the two roots are the same point - but the worldline exists and the
    /// geodesic tests pin it.
    pub outgoing: bool,
    /// How to re-derive E if this worldline is released again somewhere else, which is what letting
    /// go of a dragged marker does. `energy` is what it currently is; this is what it means.
    pub release: Release,
}

impl Default for WorldlineParams {
    /// The raindrop: dropped from rest at infinity, straight in.
    fn default() -> Self {
        Self {
            energy: 1.0,
            l_ang: 0.0,
            outgoing: false,
            release: Release::FromInfinity,
        }
    }
}

impl WorldlineParams {
    /// Raw constants, for a test that wants a particular worldline rather than a particular
    /// release. Re-releasing one of these - a drag - puts them on the raindrop, since an energy
    /// stated as a number says nothing about where it came from.
    ///
    /// Test-only: the app states a release and lets E follow, through `released`.
    #[cfg(test)]
    pub fn new(energy: f64, l_ang: f64, outgoing: bool) -> Self {
        Self { energy, l_ang, outgoing, release: Release::FromInfinity }
    }

    /// The worldline a release at `r` with angular momentum `l_ang` puts an observer on. This is
    /// the app's own path: the card says where and how, and E follows.
    pub fn released(metric: &KerrSchild, r: f64, l_ang: f64, release: Release) -> Self {
        let (energy, l_ang) = release_constants(metric, r, l_ang, release);
        Self { energy, l_ang, outgoing: false, release }
    }
}

/// One recorded event of a worldline, carrying everything needed to put the observer back on it.
///
/// The drawing only wants (t, r, phi). The other two fields are what make a rewind exact rather
/// than approximate: `tau` is the proper time the integrator actually accumulated, where the old
/// step-back re-derived it as Delta t / u^t and drifted, and `u` is the integrated 4-velocity,
/// where the old step-back rebuilt it from the conserved (E, L) at the recorded radius and so
/// restarted the worldline on a neighbouring one, off by the integration error. With both
/// recorded, winding back to a recorded event and running forward again reproduces the forward
/// pass exactly rather than approximately.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrailPoint {
    /// Coordinate time of the event.
    pub t: f64,
    /// Radius of the event.
    pub r: f64,
    /// Azimuth of the event, folded into [0, 2 pi) exactly as `GeodesicState` keeps it.
    pub phi: f64,
    /// The observer's own clock at the event.
    pub tau: f64,
    /// The integrated 4-velocity u^mu = (u^t, u^r, u^phi) at the event.
    u: [f64; 3],
    /// Whether the geodesic had already frozen onto r- (`GeodesicState::stalled`) at the event.
    /// A frozen worldline goes on climbing in t at fixed (r, phi, tau), so this is the one bit
    /// that tells the vertical segment apart from the fall that led into it, and it is what stops
    /// a rewind inside that segment from trying to integrate a worldline that has stopped.
    stalled: bool,
}

/// Trail entries kept for a moving worldline, and for a dragged one. Past these the oldest entry
/// is dropped, which bounds the drawing and, with it, how far back a rewind can reach: the
/// reversible window is the window the trail keeps, exactly as it is for `SignalField`.
const TRAIL_MAX_POINTS: usize = 800;
const TRAIL_MAX_DRAG_POINTS: usize = 500;

#[derive(Debug, Clone)]
pub struct Observer {
    pub name: String,
    pub mode: ObserverMode,
    /// Coordinate time t
    pub t: f64,
    /// Radius r
    pub r: f64,
    /// Azimuth phi in radians
    pub phi: f64,
    /// Accumulated proper time tau
    pub tau: f64,
    /// Local rest frame radial boost beta_r in (-0.99, 0.99)
    pub beta_r: f64,
    /// Local rest frame azimuthal boost beta_phi in (-0.99, 0.99)
    pub beta_phi: f64,
    /// Geodesic state for automated infall simulation
    pub geodesic: Option<GeodesicState>,
    /// The worldline as it has actually been drawn: one entry per step taken, oldest first,
    /// ending on the observer's current event. The (t, r) diagram reads (t, r) off it, the
    /// top-down view (r, phi), and `rewind_to` reads all of it.
    ///
    /// A deque rather than a vector because the cap evicts from the front on every step once the
    /// run is long enough to reach it, and dropping the front of a vector moves everything behind
    /// it. See `Observer::record`.
    pub trail: VecDeque<TrailPoint>,
    /// The event the observer was created at: the hover position, the clock it started on and the
    /// release seed of its geodesic. It is kept out of the trail because the trail has a cap and
    /// can drop its own first entry on a long run, while a rewind back into the hover needs this
    /// event exactly.
    start: TrailPoint,
    /// Release coordinate time t_release (e.g. 0 for Alice, delta_t for Bob)
    pub release_t: f64,
    /// What this observer's release means, so that a re-release at another radius - letting go of a
    /// dragged marker - can work out the E that belongs there. See `Release`.
    pub release: Release,
    /// Is the observer active/released yet?
    pub is_active: bool,
}

impl Observer {
    /// A raindrop observer (E = 1, L = 0, ingoing) at phi = 0.
    ///
    /// Nothing in the app builds an observer this way any more: every drop comes off an observer
    /// card, which always has an E, an L and an azimuth to say, so the app's own path is
    /// `new_with_phi`. It is kept for the tests, which are full of plain raindrops and have no use
    /// for the other four arguments.
    #[cfg(test)]
    pub fn new(metric: &KerrSchild, name: &str, start_t: f64, start_r: f64, release_t: f64) -> Self {
        Self::new_with_phi(
            metric,
            name,
            start_t,
            start_r,
            release_t,
            0.0,
            WorldlineParams::default(),
        )
    }

    /// An observer on the free-fall worldline picked out by `params`, starting at azimuth
    /// `start_phi`. The initial 4-velocity depends on the geometry, hence the metric argument.
    pub fn new_with_phi(
        metric: &KerrSchild,
        name: &str,
        start_t: f64,
        start_r: f64,
        release_t: f64,
        start_phi: f64,
        params: WorldlineParams,
    ) -> Self {
        let release_t = release_t.max(start_t);
        let mut geodesic = GeodesicState::new_with_direction(
            metric,
            release_t,
            start_r,
            params.energy,
            params.l_ang,
            params.outgoing,
        );
        geodesic.phi = start_phi;
        let start = TrailPoint {
            t: start_t,
            r: start_r,
            phi: start_phi,
            tau: 0.0,
            u: geodesic.u,
            stalled: false,
        };
        Self {
            name: name.to_string(),
            mode: ObserverMode::FreeFall,
            t: start_t,
            r: start_r,
            phi: start_phi,
            tau: 0.0,
            beta_r: 0.0,
            beta_phi: 0.0,
            geodesic: Some(geodesic),
            trail: VecDeque::from([start]),
            start,
            release_t,
            release: params.release,
            is_active: start_t >= release_t,
        }
    }

    /// Reset observer with initial radius, coordinate time, azimuth phi and worldline constants.
    ///
    /// Put this observer back at the start of a worldline: a new radius, a new set of constants,
    /// the clock at `start_t` and the trail thrown away.
    ///
    /// Restarting a *run* does not go through here - that builds both observers afresh from their
    /// cards, in `AppControls::drop_observers`, because a card carries a release delay and this
    /// cannot set one. What does go through here is restating one observer while the run is
    /// standing at its start: with the clock at zero the card and the observer are the same thing,
    /// so moving the drop radius, the release or L on the panel moves the observer at once instead
    /// of waiting for a Reset that would change nothing else.
    pub fn reset_with_phi(
        &mut self,
        metric: &KerrSchild,
        start_t: f64,
        start_r: f64,
        start_phi: f64,
        params: WorldlineParams,
    ) {
        self.t = start_t;
        self.r = start_r;
        self.phi = start_phi;
        self.tau = 0.0;
        self.beta_r = 0.0;
        self.beta_phi = 0.0;
        self.release_t = self.release_t.max(start_t);
        let mut geo = GeodesicState::new_with_direction(
            metric,
            self.release_t,
            start_r,
            params.energy,
            params.l_ang,
            params.outgoing,
        );
        geo.phi = start_phi;
        self.start = TrailPoint {
            t: start_t,
            r: start_r,
            phi: start_phi,
            tau: 0.0,
            u: geo.u,
            stalled: false,
        };
        self.geodesic = Some(geo);
        self.release = params.release;
        self.trail.clear();
        self.trail.push_back(self.start);
        self.is_active = self.t >= self.release_t;
    }

    /// Set position directly from user mouse dragging
    pub fn set_drag_position(&mut self, t: f64, r: f64) {
        self.mode = ObserverMode::ManualDrag;
        self.t = t;
        self.r = r.max(0.01);
        self.is_active = true;
        self.record(TRAIL_MAX_DRAG_POINTS);
    }

    /// The observer's current event as a trail entry.
    fn current_point(&self) -> TrailPoint {
        let (u, stalled) = match self.geodesic {
            Some(geo) => (geo.u, geo.stalled),
            None => ([1.0, 0.0, 0.0], false),
        };
        TrailPoint { t: self.t, r: self.r, phi: self.phi, tau: self.tau, u, stalled }
    }

    /// Record the current event on the trail, dropping the oldest entry once `cap` is passed.
    ///
    /// Both halves are O(1), which is the reason the trail is a `VecDeque`. Evicting the front of a
    /// vector shifts every remaining entry down one, so the cost of a step was linear in the cap
    /// and the cost of a run quadratic in it: measured at a cap of 80 000, one step took 259 us
    /// against 0.62 us at 800 - a 417-fold rise for a 100-fold buffer, all of it memmove, about
    /// 293 MB/s of copying at 60 fps. Nothing about the trail wants random access, so nothing was
    /// buying that.
    fn record(&mut self, cap: usize) {
        if self.trail.len() > cap {
            self.trail.pop_front();
        }
        self.trail.push_back(self.current_point());
    }

    /// Put the observer, and the geodesic driving them, back on a recorded event exactly. Nothing
    /// is re-derived: the proper time and the 4-velocity come off the record, so the worldline
    /// resumes as the same solution of the same equation rather than as a nearby one.
    fn restore(&mut self, point: TrailPoint) {
        self.t = point.t;
        self.r = point.r;
        self.phi = point.phi;
        self.tau = point.tau;
        if let Some(ref mut geo) = self.geodesic {
            geo.t = point.t;
            geo.r = point.r;
            geo.phi = point.phi;
            geo.tau = point.tau;
            geo.u = point.u;
            geo.stalled = point.stalled;
        }
    }

    /// Let go of a dragged observer: release them again, at the event the marker was dropped on.
    ///
    /// A drag is a teleport followed by an engine cut, and the cut means the same thing wherever it
    /// happens: the observer's own `Release` is re-read at the new radius, so a marker let go of by
    /// somebody released at rest is at rest *there*, and one let go of by a raindrop is still a
    /// raindrop. L is carried across unchanged, being the one constant the user chose directly.
    ///
    /// It used to carry E across instead. That made a drag into a statement about a worldline the
    /// observer is no longer on - E is the energy of a release that happened at a different radius -
    /// so the marker came to rest somewhere it had no business being at rest, or shot inward from a
    /// radius it should have been hanging at.
    pub fn release_from_drag(&mut self, metric: &KerrSchild, mode: ObserverMode) {
        if let Some(old) = self.geodesic {
            // Both constants are re-read: L is carried across as the user's own choice, except
            // for a circular release, where it belongs to the radius.
            let (energy, l_ang) = release_constants(metric, self.r, old.l_ang, self.release);
            let mut geo = GeodesicState::new_infall(metric, self.t, self.r, energy, l_ang);
            geo.phi = self.phi;
            geo.tau = self.tau;
            self.geodesic = Some(geo);
        }
        self.mode = mode;
        self.release_t = self.release_t.min(self.t);
        self.is_active = true;
    }

    /// While waiting for release the observer holds their radius on the worldline of
    /// `hover_four_velocity`: coordinate time follows the simulation clock, phi turns at
    /// u^phi/u^t and proper time ticks at dt/u^t. For the static hover that rate is exactly the
    /// sqrt(-g_tt) dt it has always been, u^t there being 1/sqrt(-g_tt); for the at-rest release it
    /// is the rate of the worldline they are about to fall on, which is what makes the release
    /// smooth. The trail is therefore a vertical segment in (t, r) either way, and turns into the
    /// infall curve at t = release_t.
    fn hover(&mut self, metric: &KerrSchild, current_sim_time: f64, dt: f64) {
        self.is_active = false;
        self.t = current_sim_time;
        let u = self.hover_four_velocity(metric);
        if u[0] > 0.0 {
            self.phi += (u[2] / u[0]) * dt.max(0.0);
            self.tau += dt.max(0.0) / u[0];
        }
        if let Some(ref mut geo) = self.geodesic {
            geo.t = self.release_t;
            geo.tau = self.tau;
            geo.phi = self.phi;
        }
        // Keep the trail as [start point, current hover point]
        self.trail.clear();
        self.trail.push_back(self.start);
        self.trail.push_back(self.current_point());
    }

    /// Has this worldline ended, as far as the simulation is concerned?
    ///
    /// Two ways out, and both are the end of the observer's future in this chart. Either the
    /// worldline has reached the ring, r <= `R_STOP`, where `GeodesicState` stops integrating
    /// because the equatorial L = 0 infall runs into the curvature singularity; or its geodesic has
    /// stalled, which is the E - Omega_- L < 0 case of freezing onto r-, where the worldline
    /// asymptotes to a surface of constant r and its proper time to a finite limit while only the
    /// coordinate clock runs on.
    ///
    /// It is a statement about the drawn worldline, not a prediction. Once it is true, no signal
    /// emitted anywhere later can be received by this observer, because there is no more of their
    /// worldline left for a ray to cross: the last signal that did arrive marks, on the emitter's
    /// worldline, the boundary of the causal past of the end of this one.
    pub fn has_ended(&self) -> bool {
        self.r <= R_STOP || self.is_frozen()
    }

    /// Has this worldline frozen onto the far branch of the Cauchy horizon?
    ///
    /// The E - Omega_- L < 0 case: the worldline asymptotes to r- as r - r- ~ exp(-kappa_- t),
    /// reaching it at infinite coordinate time and at a finite proper time it never gets to spend,
    /// and `geodesic::U_T_STALL` is where the integration is stopped because the chart can no
    /// longer resolve the gap. From there the observer is not integrated but *carried*: the
    /// surface's own null generator is what is left of their future in this chart, and `advance`
    /// slides them along it at Omega_- (`KerrSchild::inner_horizon_omega`).
    pub fn is_frozen(&self) -> bool {
        self.geodesic.map(|geo| geo.stalled).unwrap_or(false)
    }

    /// Bob on the one worldline that freezes onto the far branch of r-, run there: E = 1,
    /// L = 2.2 at a = 0.90, released at r = 9 M and stepped at dt = 0.25 M until `is_frozen`.
    ///
    /// These constants have E - Omega_- L < 0, so he never crosses r-. He asymptotes to it with
    /// u^t growing like exp(kappa_- t) until `geodesic::U_T_STALL` stops the integration, about
    /// 7e-10 M above the surface. Several tests need an observer who is *actually* frozen rather
    /// than one told that he is, and this is the run that produces one, written once.
    #[cfg(test)]
    pub(crate) fn frozen_bob(metric: &KerrSchild) -> Observer {
        let dt = 0.25;
        let mut bob = Observer::new_with_phi(
            metric,
            "Bob",
            0.0,
            9.0,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.2, false),
        );
        let mut t = 0.0;
        while !bob.is_frozen() && t < 200.0 {
            t += dt;
            bob.step(metric, t, dt);
        }
        assert!(
            bob.is_frozen(),
            "the freeze must happen inside 200 M; at t = {t} Bob is still at r = {}",
            bob.r
        );
        bob
    }

    /// Kerr-Schild Cartesian azimuth psi of the observer, x + i y = (r + i a) e^{i phi}, i.e. the
    /// polar angle of `cartesian_position`. It is a correct quantity, but it is *not* enough to
    /// plot with: the matching radius is sqrt(r^2 + a^2), never r, so drawing at (r, psi) is what
    /// `cartesian_position` exists to replace.
    #[allow(dead_code)] // cross-checked by the tests against `cartesian_position`
    pub fn azimuth(&self, metric: &KerrSchild) -> f64 {
        self.phi + metric.a.atan2(self.r.max(1e-9))
    }

    /// Kerr-Schild Cartesian position (x, y) of the observer, x + i y = (r + i a) e^{i phi}.
    /// This, not (r cos psi, r sin psi), is where the observer belongs in a top-down view: the
    /// point sits at Cartesian radius sqrt(r^2 + a^2), which is a for a body on the ring.
    pub fn cartesian_position(&self, metric: &KerrSchild) -> (f64, f64) {
        metric.cartesian_position(self.r, self.phi)
    }

    /// Can the currently selected mode exist at the observer's radius?
    /// A static observer needs a timelike d/dt (g_tt < 0, i.e. r > 2M on the equator); a ZAMO
    /// needs a timelike fixed-r worldline, which only exists outside the outer horizon r+.
    /// Free fall and manual drag are always admissible.
    pub fn mode_admissible(&self, metric: &KerrSchild) -> bool {
        Self::mode_admissible_at(self.mode, metric, self.r)
    }

    fn mode_admissible_at(mode: ObserverMode, metric: &KerrSchild, r: f64) -> bool {
        match mode {
            ObserverMode::Static => metric.metric_components(r)[0][0] < 0.0,
            ObserverMode::Zamo => r > metric.outer_horizon(),
            ObserverMode::FreeFall | ObserverMode::ManualDrag => true,
        }
    }

    /// The mode the observer is *actually* following at the event they now stand on: the selected
    /// `mode` where it is admissible there, and free fall where it is not.
    ///
    /// The selection itself is never touched. It is the user's standing request, so a Static or
    /// ZAMO choice made at a radius where that worldline does not exist resumes by itself the
    /// moment the observer is back somewhere it does.
    ///
    /// Everything that says what the worldline *does* dispatches on this rather than on `mode`:
    /// `four_velocity_at`, `advance`, `rewind_to`, and the release test in `step`. That is the
    /// whole of the fix. Dispatching the 4-velocity on admissibility while dispatching the motion
    /// on the raw selection is what drew a Static observer at fixed r inside the ergosphere while
    /// every quantity derived from his u - the telemetry dr/dt, the tetrad his pulses were emitted
    /// into, the frame his receptions were measured in - belonged to a worldline falling inward at
    /// 0.73c. His pulses, isotropic in that falling frame, then ran away from the point he was
    /// drawn at until he stood outside his own past light cones, which no timelike worldline can
    /// ever do.
    pub fn effective_mode(&self, metric: &KerrSchild) -> ObserverMode {
        if Self::mode_admissible_at(self.mode, metric, self.r) {
            self.mode
        } else {
            ObserverMode::FreeFall
        }
    }

    /// u^mu = (1, 0, 0) / sqrt(-g_tt), the normalised time-translation Killing vector, at every
    /// radius where d/dt is timelike, and None inside the equatorial static limit r = 2M where it
    /// is not and no static observer exists.
    fn static_four_velocity(metric: &KerrSchild, r: f64) -> Option<[f64; 3]> {
        let g_tt = metric.metric_components(r)[0][0];
        (g_tt < 0.0).then(|| [1.0 / (-g_tt).sqrt(), 0.0, 0.0])
    }

    /// Contravariant 4-velocity u^mu = (u^t, u^r, u^phi) at the observer's current event,
    /// normalised so that g_{mu nu} u^mu u^nu = -1.
    ///
    /// It is the 4-velocity of the worldline the observer is *on*, not of the one they asked for.
    /// The dispatch is on `effective_mode`, so a Static or ZAMO selection at a radius where that
    /// worldline cannot exist reports the free-fall value - and `advance` moves the observer along
    /// free fall to match. The two are one object; nothing here may describe a motion the stepper
    /// does not take.
    ///
    /// A released free-faller reports the *integrated* 4-velocity carried by its geodesic state,
    /// so every derived quantity (`velocity_c`, `proper_velocity_c`, the signal pulses, the
    /// rest-frame view) follows the worldline actually being drawn, outgoing phases and turning
    /// points included. It has to be standing on the observer's current event to be read, because
    /// that is the only state in which it describes them: a mode that holds r fixed leaves it
    /// parked at whatever event it was last integrated to, and the closed-form congruence value at
    /// the current radius is the honest answer until the next step re-seeds it there.
    ///
    /// An observer waiting for release is on neither. `hover` holds them at fixed (r, phi) and
    /// ticks their clock at dtau = sqrt(-g_tt) dt, so the worldline they are on while they wait is
    /// an integral curve of the time-translation Killing vector, and that is what is reported:
    /// `static_four_velocity` at the hover radius, which is the frame their clock, their pulses and
    /// their telemetry all belong to. Waiting at or inside the static limit is the one case with no
    /// worldline under it at all - no rocket can hold phi fixed there, `hover` does not advance the
    /// clock, and `wavefront::emit_if_due` keeps the observer silent - and the closed-form free-fall
    /// value at that radius is reported instead, which is the worldline they join the instant they
    /// are released.
    pub fn four_velocity(&self, metric: &KerrSchild) -> [f64; 3] {
        if !self.is_active {
            return self.hover_four_velocity(metric);
        } else if self.effective_mode(metric) == ObserverMode::FreeFall
            && self.geodesic_stands_on_current_event()
            && let Some(geo) = self.geodesic
        {
            return geo.u;
        }
        self.four_velocity_at(metric, self.r)
    }

    /// The worldline an observer still waiting for release is actually on.
    ///
    /// It is the one they are about to join, whenever that is a worldline of fixed r: an observer
    /// released at rest has dr/dtau = 0 at the drop radius, so holding them there at exactly that
    /// four-velocity is a real worldline - a platform under thrust, turning with whatever angular
    /// momentum they were given - and the release is then an engine cut, continuous in every
    /// component. Hovering as a *static* observer instead, which is what this used to do, left a
    /// jump at the release: 0.116c of azimuthal velocity even for L = 0 at 4.5M, because at rest in
    /// r is not the same as at rest in phi where the frame is dragged.
    ///
    /// Where the fall they are waiting for is already moving in r - a release from infinity, which
    /// arrives at the drop radius at two thirds of the speed of light - no fixed-r worldline can
    /// join it, and the wait falls back to the static observer as before. That discontinuity is not
    /// an artefact to be smoothed away: it is the statement that they did not come from here.
    fn hover_four_velocity(&self, metric: &KerrSchild) -> [f64; 3] {
        if let Some(geo) = self.geodesic
            && geo.u[1].abs() <= 1e-6 * (1.0 + geo.u[0].abs())
        {
            return geo.u;
        }
        Self::static_four_velocity(metric, self.r)
            .unwrap_or_else(|| self.four_velocity_at(metric, self.r))
    }

    /// `four_velocity` for the same family of worldlines evaluated at an arbitrary radius.
    /// Because the metric depends on r alone, every mode's components are functions of r only,
    /// which is what makes the 4-acceleration computable by differentiating in r. That is why
    /// free fall stays on the closed-form *ingoing* solution here rather than on the integrated
    /// state: `four_acceleration` needs u^mu(r), a function of the radius alone, and the
    /// integrated u is a function of the worldline parameter instead.
    ///
    /// Two radii are in play and they are not the same one. The *family* is chosen by
    /// `effective_mode`, which is a statement about the observer's current event, because the
    /// family differentiated here has to be the family `advance` is stepping along; the closed
    /// form is then evaluated at the radius asked for, and each mode arm still guards on
    /// admissibility *there*, because that is where its square roots have to be real. The two can
    /// only disagree when the finite-difference stencil straddles a boundary - h ~ 1e-5 r, so
    /// nowhere but within a whisker of r = 2M or r = r+ - and the fixed-r modes have u^r = 0
    /// exactly, so `four_acceleration` takes no finite difference for them at all and the stencil
    /// never gets built. Where the guard does fire, on a direct call at a radius the mode cannot
    /// exist at, free fall is the honest answer: there is no such observer there to quote.
    fn four_velocity_at(&self, metric: &KerrSchild, r: f64) -> [f64; 3] {
        let r = r.max(1e-4);
        match self.effective_mode(metric) {
            ObserverMode::Static if Self::mode_admissible_at(ObserverMode::Static, metric, r) => {
                // u^mu = (1, 0, 0) / sqrt(-g_tt): the normalised time-translation Killing vector.
                Self::static_four_velocity(metric, r).expect("admissible above the static limit")
            }
            ObserverMode::Zamo if Self::mode_admissible_at(ObserverMode::Zamo, metric, r) => {
                Self::zamo_four_velocity(metric, r).expect("admissible outside r+")
            }
            ObserverMode::ManualDrag => {
                // The dragged observer is defined as a *boost of the local raindrop frame*:
                // u = gamma (e0 + beta_r e1 + beta_phi e2) built on the orthonormal tetrad of the
                // E = 1, L = 0 ingoing geodesic at this radius. (beta_r, beta_phi) is therefore
                // the observer's velocity, as a fraction of c, relative to an observer dropped
                // from rest at infinity and passing through the same event.
                //
                // The raindrop frame is the reference because it exists at every r > 0, including
                // between the horizons where no static or ZAMO frame exists; beta = 0 reproduces
                // free fall exactly, and any beta != 0 is a rocket with real proper acceleration.
                Self::raindrop_tetrad(metric, r).boost(self.beta_r, self.beta_phi)
            }
            // Free fall, whether it was selected or whether it is what an inadmissible Static /
            // ZAMO selection has been reduced to by `effective_mode`, plus the boundary case
            // above where the closed form of an admissible fixed-r mode runs out at the radius
            // asked for.
            _ => self.free_fall_four_velocity(metric, r),
        }
    }

    /// u^mu = gamma (1, 0, omega) with omega = -g_tphi/g_phiphi, so that u_phi = 0: the
    /// zero-angular-momentum observer, holding r while going along with the frame dragging. None
    /// at and inside r+, where no worldline of fixed r is timelike.
    fn zamo_four_velocity(metric: &KerrSchild, r: f64) -> Option<[f64; 3]> {
        if r <= metric.outer_horizon() {
            return None;
        }
        let g = metric.metric_components(r);
        let omega = metric.frame_dragging_omega(r);
        // gamma = 1 / sqrt(-(g_tt + 2 omega g_tphi + omega^2 g_phiphi)).
        let norm_sq = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma = 1.0 / norm_sq.max(1e-14).sqrt();
        Some([gamma, 0.0, gamma * omega])
    }

    /// The raindrop's own 4-velocity at radius r: the E = 1, L = 0 ingoing geodesic. It exists at
    /// every r > 0, which is the whole reason this frame is in the app twice over - as the
    /// reference for `ObserverMode::ManualDrag` and as the last frame a speed can be quoted
    /// against once both hovering observers have run out.
    pub fn raindrop_four_velocity(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let raindrop = GeodesicState::new_infall(metric, 0.0, r, 1.0, 0.0);
        let (dt_dtau, dr_dtau, dphi_dtau) = raindrop.derivatives(metric, r);
        [dt_dtau, dr_dtau, dphi_dtau]
    }

    /// Orthonormal tetrad of the raindrop (E = 1, L = 0 ingoing geodesic) observer at radius r.
    ///
    /// It is public because it is also the frame anything drawn *at* an observer's event should be
    /// sampled in, whoever is standing there: the raindrop exists at every r > 0 and its u^mu is
    /// of order 1, while a worldline frozen on r- carries u^t out to `geodesic::U_T_STALL` = 1e10,
    /// and uniform sampling in a frame boosted that hard is aberrated into a single point.
    pub fn raindrop_tetrad(metric: &KerrSchild, r: f64) -> Tetrad {
        Tetrad::from_four_velocity(metric, r, &Self::raindrop_four_velocity(metric, r))
    }

    /// The closed-form geodesic congruence with this observer's conserved (E, L), evaluated at
    /// radius r on the root the observer is currently travelling on. It is a function of r alone,
    /// which is what lets `four_acceleration` differentiate it; on the ingoing root (the usual
    /// case) it is the exact regular solution of `GeodesicState::derivatives`.
    fn free_fall_four_velocity(&self, metric: &KerrSchild, r: f64) -> [f64; 3] {
        let geo = self
            .geodesic
            .unwrap_or_else(|| GeodesicState::new_infall(metric, self.t, r, 1.0, 0.0));
        geo.branch_four_velocity_at(metric, r, geo.u[1] > 0.0)
    }

    /// Proper 4-acceleration a^mu = du^mu/dtau + Gamma^mu_{alpha beta} u^alpha u^beta.
    ///
    /// Static and ZAMO observers sit at fixed r and their components depend on r alone, so
    /// du^mu/dtau = 0 and only the connection term survives: their acceleration is exactly the
    /// thrust needed to resist gravity. For free fall (and, for now, manual drag) u^r != 0, and
    /// since u^mu = u^mu(r) along the worldline, du^mu/dtau = (du^mu/dr) u^r. The r derivative is
    /// taken by a central difference with step h ~ 1e-5 max(r, 0.1), using the 5-point stencil
    /// (-u(r+2h) + 8u(r+h) - 8u(r-h) + u(r-2h)) / 12h; its O(h^4) truncation error keeps the
    /// residual below 1e-7 even at r ~ r-/2, where u varies on the scale of r itself. The two
    /// terms then cancel to numerical noise, which is the statement that the coded geodesic
    /// really is a geodesic of the coded metric.
    ///
    /// That cancellation is a diagnostic of the integrator, not a reading of the observer's
    /// accelerometer, and the two part company where the stencil cannot follow: on a worldline
    /// bound for the far branch of r-, u^t has a pole at r-, the two terms are each of order
    /// (u^t)^2, and the truncation error of a difference taken across a pole is unbounded. The
    /// accelerometer of an observer in free fall is `accelerometer_geom`, which is zero there by
    /// definition; this residual is what the tests check.
    pub fn four_acceleration(&self, metric: &KerrSchild) -> [f64; 3] {
        let u = self.four_velocity(metric);
        let gamma = metric.christoffel(self.r);

        let mut accel = [0.0f64; 3];
        for mu in 0..3 {
            let mut sum = 0.0;
            for alpha in 0..3 {
                for beta in 0..3 {
                    sum += gamma[mu][alpha][beta] * u[alpha] * u[beta];
                }
            }
            accel[mu] = sum;
        }

        // du^mu/dtau = (du^mu/dr) u^r; identically zero for the fixed-r modes.
        if u[1] != 0.0 {
            let h = 1e-5 * self.r.max(0.1);
            let u_p1 = self.four_velocity_at(metric, self.r + h);
            let u_p2 = self.four_velocity_at(metric, self.r + 2.0 * h);
            let u_m1 = self.four_velocity_at(metric, self.r - h);
            let u_m2 = self.four_velocity_at(metric, self.r - 2.0 * h);
            for mu in 0..3 {
                let du_dr =
                    (-u_p2[mu] + 8.0 * u_p1[mu] - 8.0 * u_m1[mu] + u_m2[mu]) / (12.0 * h);
                accel[mu] += du_dr * u[1];
            }
        }

        accel
    }

    /// Magnitude sqrt(g_{mu nu} a^mu a^nu) of `four_acceleration`, in geometric units (1/M).
    /// The 4-acceleration of a timelike worldline is spacelike, so the radicand is non-negative
    /// up to round-off; it is clamped at zero. For the fixed-r modes this is the thrust; for a
    /// free-faller it is the integrator residual described on `four_acceleration`, and the
    /// accelerometer reading to show is `accelerometer_geom`.
    pub fn proper_acceleration_geom(&self, metric: &KerrSchild) -> f64 {
        let accel = self.four_acceleration(metric);
        metric.norm(self.r, &accel).max(0.0).sqrt()
    }

    /// What the observer's accelerometer reads, in geometric units (1/M).
    ///
    /// An observer in free fall is on a geodesic - that is what the mode means, and it is the
    /// geodesic equation the integrator is stepping - so the reading is exactly zero, and not the
    /// finite-difference residual of `four_acceleration`, which is a check on the integrator
    /// rather than a measurement and blows up on the approach to the far branch of r-. Every
    /// other mode is held on its worldline by thrust, and the reading is that thrust; so is a
    /// free-faller still waiting for release, who is being held at fixed r until then and
    /// reports the static worldline's 4-velocity (see `four_velocity`).
    pub fn accelerometer_geom(&self, metric: &KerrSchild) -> f64 {
        if self.is_active && self.effective_mode(metric) == ObserverMode::FreeFall {
            0.0
        } else {
            self.proper_acceleration_geom(metric)
        }
    }

    /// Is this worldline weightless, i.e. a geodesic?
    /// Free fall reads as weightless outright. For the other modes the comparison is against a
    /// curvature-relative floor rather than an absolute one: the residual left by the
    /// finite-difference du^mu/dtau in `four_acceleration` grows with the local curvature scale
    /// sqrt(K) = sqrt(48) M / r^3, and so does every honest acceleration near the singularity.
    pub fn is_free_falling(&self, metric: &KerrSchild) -> bool {
        let curvature_scale = metric.kretschmann_scalar(self.r).sqrt().max(1e-12);
        self.accelerometer_geom(metric) < 1e-6 * curvature_scale
    }

    /// Advance the worldline to the simulation clock's new value `current_sim_time`, which is
    /// `dt` of coordinate time later than it was.
    pub fn step(&mut self, metric: &KerrSchild, current_sim_time: f64, dt: f64) {
        if current_sim_time < self.release_t {
            self.hover(metric, current_sim_time, dt);
            return;
        }
        // On the step that crosses the release, a free-faller's worldline starts at t = release_t,
        // which is where `hover` has been holding its geodesic clock, and not at the simulation
        // clock's previous value: the release almost never lands on a step boundary, and
        // integrating a whole dt from release_t would put the worldline that far ahead of the
        // clock and keep it there for the rest of the run (0.03 M of it at a release of t = 4.03
        // stepped at 0.1). The other modes have no separate clock to start - they were already
        // moving in t while they waited - so they cover the full step.
        let releasing = !self.is_active && self.effective_mode(metric) == ObserverMode::FreeFall;
        self.is_active = true;
        let interval = if releasing { (current_sim_time - self.release_t).max(0.0) } else { dt };
        self.advance(metric, interval);
    }

    /// Is the geodesic state standing on the observer's current event?
    ///
    /// The geodesic *is* the free-fall step, so before one is taken the two have to be the same
    /// event or the fall starts somewhere the observer is not. They part company whenever the
    /// observer has been moved by something else: a Static or ZAMO step, a drag, or a mode the
    /// user has just switched away from. The one case where they legitimately differ is the wait
    /// before release, where `hover` parks the geodesic at t = release_t - the event the fall
    /// starts at - while the observer's own clock tracks the simulation below it. Hence the `max`.
    fn geodesic_stands_on_current_event(&self) -> bool {
        match self.geodesic {
            None => false,
            Some(geo) => {
                let t_ref = self.t.max(self.release_t);
                (geo.t - t_ref).abs() <= 1e-9 * (1.0 + t_ref.abs())
                    && (geo.r - self.r).abs() <= 1e-9 * (1.0 + self.r.abs())
            }
        }
    }

    /// Put a geodesic under the observer at the event they now stand on, unless the one they carry
    /// is already there.
    ///
    /// This is what lets an impossible Static or ZAMO selection fall: the mode that was holding r
    /// fixed leaves no geodesic behind it, so one is seeded here with the observer's own conserved
    /// (E, L) on the ingoing root, exactly as `release_from_drag` re-seeds when the user lets go of
    /// the marker, and proper time carries on from where the observer's clock stands. It is also
    /// what stops a mode switch from teleporting anybody: the new worldline starts at the current
    /// event rather than wherever the old geodesic state happened to be left.
    fn seed_geodesic_at_current_event(&mut self, metric: &KerrSchild) {
        if self.geodesic_stands_on_current_event() {
            return;
        }
        let (energy, l_ang) = self
            .geodesic
            .map(|geo| (geo.energy, geo.l_ang))
            .unwrap_or((1.0, 0.0));
        let mut geo = GeodesicState::new_infall(metric, self.t, self.r, energy, l_ang);
        geo.phi = self.phi;
        geo.tau = self.tau;
        self.geodesic = Some(geo);
    }

    /// One released step, along the worldline the observer is actually on - `effective_mode`, not
    /// the raw selection. A Static or ZAMO choice at a radius where that worldline does not exist
    /// therefore falls freely in *position* as well as in velocity, which is the whole point: the
    /// 4-velocity every other part of the app reads is the 4-velocity of the curve stepped here.
    ///
    /// It is a separate function because `rewind_to` finishes on it: after dropping the recorded
    /// events past the target the observer is put back on the last one it kept and then carried
    /// forward onto the target with *this* code, so the landing integrates the same equation with
    /// the same integrator and the same guards that got the observer there in the first place.
    /// Anything the forward step refuses to do - moving a worldline that has reached the ring, for
    /// one - the landing refuses in the same way, without a second statement of the rule.
    fn advance(&mut self, metric: &KerrSchild, dt: f64) {
        match self.effective_mode(metric) {
            ObserverMode::FreeFall => {
                self.seed_geodesic_at_current_event(metric);
                if let Some(ref mut geo) = self.geodesic {
                    if geo.stalled {
                        // Frozen on r-, which is not the same as stopped. r and tau really are
                        // fixed: the gap r - r- ~ exp(-kappa_- t) is below what a double can
                        // resolve by the time the stall is declared, and the proper time has its
                        // finite limit. But the surface is a null surface whose generators turn,
                        // and the worldline is now riding one of them, so phi keeps winding at
                        // the generators' own rate Omega_- per unit t. The frozen family of rays
                        // is already drawn co-rotating this way (`NullRay::frozen`), and an
                        // observer settling onto the same branch settles onto the same motion:
                        // anything else would have him drift across the generators of a surface
                        // he can no longer cross. It is exact to the precision at which the stall
                        // was declared, the departure from Omega_- falling off with the gap.
                        geo.t += dt;
                        geo.phi = (geo.phi + metric.inner_horizon_omega() * dt)
                            .rem_euclid(std::f64::consts::TAU);
                        self.t = geo.t;
                        self.phi = geo.phi;
                        self.record(TRAIL_MAX_POINTS);
                    } else if geo.r > R_STOP {
                        geo.step_coord_time(metric, dt);
                        self.t = geo.t;
                        self.r = geo.r;
                        self.phi = geo.phi;
                        self.tau = geo.tau;
                        self.record(TRAIL_MAX_POINTS);
                    }
                }
            }
            ObserverMode::ManualDrag => {
                // Keep manual position, just advance t slightly if playing
                self.t += dt;
            }
            ObserverMode::Static | ObserverMode::Zamo => {
                // Fixed r, and reached only where that worldline exists, so `four_velocity` is
                // the fixed-r one: advance along u^mu. dphi/dt = u^phi/u^t (zero for Static, the
                // frame-dragging rate omega for the ZAMO) and dtau/dt = 1/u^t.
                let u = self.four_velocity(metric);
                let ut = u[0].max(1e-9);
                self.t += dt;
                self.phi += (u[2] / ut) * dt;
                self.tau += dt / ut;
            }
        }
    }

    /// Put the worldline back where it stood when the simulation clock read `t_target`.
    ///
    /// The rewind is stated in *time*, not in steps, and that is the whole point of it. The
    /// observer used to be wound back by popping one recorded event per call, whatever interval
    /// the caller had actually undone; since an event is recorded once per forward step and a step
    /// is anything from a played frame to the hundreds of M a Distance-mode step can be, the
    /// observers came off the simulation clock the moment anything was stepped back, and an
    /// observer who had already reached the ring was dragged back off it by a rewind that had not
    /// reached its death event at all. Everything below is expressed against `t_target` instead,
    /// so the worldline lands on the clock however the caller got there.
    ///
    /// The cases, all exact:
    ///
    /// * A worldline that has ended before `t_target` does not move. For one that reached the ring
    ///   this falls out of the guard below: forward steps leave its clock at the death event, so
    ///   `t_target` is already at or past `self.t` and there is nothing to undo. For one frozen on
    ///   r- the coordinate clock does keep running - `advance` climbs it vertically - so the
    ///   vertical segment is wound back like any other, and the worldline comes off r- only when
    ///   `t_target` drops below the event it froze at.
    /// * Below `release_t` the observer is put back to hovering, at the event and on the clock
    ///   that `hover` would have given it: see `rewind_into_hover`.
    /// * Otherwise the recorded events after `t_target` are dropped, the observer is restored onto
    ///   the last one kept, and `advance` carries it the rest of the way.
    ///
    /// The fixed-r modes never record anything - `advance` moves them analytically at constant
    /// u^mu - so they are wound back analytically too, by subtracting exactly what a forward step
    /// of the same interval adds. Which arm applies is decided by `effective_mode`, the same
    /// question `advance` asks, so an impossible Static or ZAMO selection is wound back along the
    /// free fall it was actually stepped along, off the trail its forward steps recorded.
    pub fn rewind_to(&mut self, metric: &KerrSchild, t_target: f64) {
        if t_target >= self.t {
            return;
        }
        if t_target < self.release_t {
            self.rewind_into_hover(metric, t_target);
            return;
        }
        self.is_active = true;
        match self.effective_mode(metric) {
            ObserverMode::FreeFall => {
                // The trail is recorded in order, so it is sorted in t and the events to keep are
                // a prefix of it: `partition_point` is the standard library's binary search for
                // exactly that, and it answers in O(log n) where walking the prefix was O(n).
                // The floor of one is the case where the target precedes every event still held -
                // the trail has a cap - and it lands the worldline on the oldest event kept.
                let keep = self.trail.partition_point(|point| point.t <= t_target + 1e-9).max(1);
                self.trail.truncate(keep);
                let last = self.trail[keep - 1];
                self.restore(last);
                if last.t < self.release_t {
                    // The kept event is the hover, where the worldline had not started yet. It
                    // starts at t = release_t, exactly as `step` starts it there, so that is where
                    // the fall onto the target is integrated from - with the hover's proper time,
                    // which is what the forward run carried into the release.
                    self.t = self.release_t;
                    if let Some(ref mut geo) = self.geodesic {
                        geo.t = self.release_t;
                    }
                }
                let remaining = t_target - self.t;
                if remaining > 1e-12 {
                    self.advance(metric, remaining);
                }
            }
            ObserverMode::Static | ObserverMode::Zamo => {
                // The exact inverse of the forward step: at fixed r the 4-velocity is a constant,
                // so the interval that was added to phi and tau is the interval to take back.
                let dt = self.t - t_target;
                let u = self.four_velocity(metric);
                let ut = u[0].max(1e-9);
                self.t = t_target;
                self.phi -= (u[2] / ut) * dt;
                self.tau -= dt / ut;
            }
            ObserverMode::ManualDrag => self.t = t_target,
        }
    }

    /// The event this worldline stood at when the simulation clock read `t`, re-integrated rather
    /// than interpolated.
    ///
    /// `Pulse::sweep` locates a reception somewhere strictly inside a detection pass interval, and
    /// then has to say where on the receiver's worldline that was. Interpolating between the two
    /// ends of the interval is first-order accurate and gets phi outright wrong whenever the
    /// receiver crossed the seam at phi = 2 pi inside it, because the integrator hands out phi
    /// folded into [0, 2 pi) and the average of 6.28 and 0.01 is the opposite side of the hole.
    /// This answers the question properly instead: put a copy of the observer back on the last
    /// recorded event at or before `t` and integrate it the rest of the way with `advance`, which
    /// is the same equation, the same integrator and the same guards that got the worldline there
    /// in the first place. `rewind_to` already does exactly that, so this is that landing with the
    /// result read off instead of kept.
    ///
    /// The cost is a clone of the observer and one short integration, which is why the caller only
    /// asks once a crossing has been found rather than once per pass: a pass that crosses nobody
    /// never calls this at all, and the arithmetic of finding the crossing needs none of it.
    ///
    /// Two limits, both inherited from the trail rather than from here. A `t` at or after the
    /// observer's own clock returns the current event, since there is nothing to wind back. A `t`
    /// earlier than the oldest event the trail still keeps lands on that oldest event instead,
    /// exactly as a rewind that far back would: the reversible window is the window the trail
    /// keeps.
    pub fn event_at(&self, metric: &KerrSchild, t: f64) -> TrailPoint {
        if t >= self.t {
            return self.current_point();
        }
        let mut probe = self.clone();
        probe.rewind_to(metric, t);
        probe.current_point()
    }

    /// Wind an observer back to before their release, onto the hovering worldline.
    ///
    /// `hover` holds them at the radius they were created at and ticks their clock at the rate of
    /// the worldline they are holding, so its accumulated proper time is exactly (t - t_start)/u^t
    /// and its azimuth start.phi + (u^phi/u^t)(t - t_start), with u the hover four-velocity there:
    /// those closed forms are what is restored here, rather than a subtraction, so a rewind that
    /// crosses the release event lands on the same clock the forward run had at that time whatever
    /// route it took. The geodesic goes back to its release seed, which is where it stood
    /// throughout the wait, and the trail back to [start, now].
    ///
    /// Getting the proper time exactly right is not cosmetic: a hovering observer transmits, and
    /// `SignalField` paces the emissions by their proper time, so a cadence put back even slightly
    /// wrong re-cuts the whole transmission at different events when time runs forward again.
    fn rewind_into_hover(&mut self, metric: &KerrSchild, t_target: f64) {
        self.is_active = false;
        self.t = t_target;
        self.r = self.start.r;
        self.phi = self.start.phi;
        let waited = (t_target - self.start.t).max(0.0);
        let u = self.hover_four_velocity(metric);
        if u[0] > 0.0 {
            self.tau = waited / u[0];
            self.phi = self.start.phi + (u[2] / u[0]) * waited;
        } else {
            self.tau = self.start.tau;
        }
        if let Some(ref mut geo) = self.geodesic {
            geo.t = self.release_t;
            geo.r = self.start.r;
            geo.phi = self.phi;
            geo.tau = self.tau;
            geo.u = self.start.u;
            geo.stalled = false;
        }
        self.trail.clear();
        self.trail.push_back(self.start);
        self.trail.push_back(self.current_point());
    }

    /// Generate polygon coordinates for the light cone at the observer's event on the (t, r)
    /// diagram. `time_height`: height in coordinate time units to extend the cone upward (future)
    /// and downward (past).
    ///
    /// The wedge drawn here is `KerrSchild::null_wedge`, the projection of the full null cone onto
    /// the (t, r) plane. That projection belongs to the *event*, not to the observer: boosting the
    /// observer re-labels which local angle alpha emits the extreme ray, but the extreme values of
    /// dr/dt are unchanged. So the cone drawn in the diagram deliberately does not depend on the
    /// observer's mode or on (beta_r, beta_phi).
    pub fn compute_lightcone_polygon(
        &self,
        metric: &KerrSchild,
        time_height: f64,
    ) -> LightConePolygon {
        let r0 = self.r;
        let t0 = self.t;

        let wedge = metric.null_wedge(r0);
        let (dr_dt_in, dphi_dt_in) = (wedge.dr_dt_in, wedge.dphi_dt_in);
        let (dr_dt_out, dphi_dt_out) = (wedge.dr_dt_out, wedge.dphi_dt_out);

        // Future cone endpoints at t = t0 + time_height.
        // If a ray reaches r = 0 before time_height, terminate the ray at the singularity
        // preserving exact dr/dt slope rather than clamping r with fixed t.
        let (t_fut_in, r_fut_in) = if dr_dt_in < -1e-7 && (r0 + dr_dt_in * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_in.abs();
            (t0 + dt_sing, 0.0)
        } else {
            (t0 + time_height, (r0 + dr_dt_in * time_height).max(0.0))
        };

        let (t_fut_out, r_fut_out) = if dr_dt_out < -1e-7 && (r0 + dr_dt_out * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_out.abs();
            (t0 + dt_sing, 0.0)
        } else {
            (t0 + time_height, (r0 + dr_dt_out * time_height).max(0.0))
        };

        // Past cone endpoints at t = t0 - time_height
        let (t_pst_in, r_pst_in) = if dr_dt_in > 1e-7 && (r0 - dr_dt_in * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_in;
            (t0 - dt_sing, 0.0)
        } else {
            (t0 - time_height, (r0 - dr_dt_in * time_height).max(0.0))
        };

        let (t_pst_out, r_pst_out) = if dr_dt_out > 1e-7 && (r0 - dr_dt_out * time_height) < 0.0 {
            let dt_sing = r0 / dr_dt_out;
            (t0 - dt_sing, 0.0)
        } else {
            (t0 - time_height, (r0 - dr_dt_out * time_height).max(0.0))
        };

        LightConePolygon {
            apex: [t0, r0],
            future_in: [t_fut_in, r_fut_in],
            future_out: [t_fut_out, r_fut_out],
            past_in: [t_pst_in, r_pst_in],
            past_out: [t_pst_out, r_pst_out],
            dr_dt_in,
            dr_dt_out,
            dphi_dt_out,
            dphi_dt_in,
        }
    }

    /// Instantaneous radial coordinate velocity dr/dt = u^r / u^t as a fraction of c, in the
    /// global Kerr-Schild foliation. Always inside the local light cone (dr/dt > -1), because
    /// the ingoing principal null ray travels at exactly dr/dt = -1 in this chart.
    pub fn velocity_c(&self, metric: &KerrSchild) -> f64 {
        let u = self.four_velocity(metric);
        u[1] / u[0].max(1e-9)
    }

    /// Instantaneous proper radial velocity dr/dtau = u^r. Unbounded: it exceeds 1.0c inside the
    /// horizon, where r is a time coordinate and no local frame can hold r still.
    pub fn proper_velocity_c(&self, metric: &KerrSchild) -> f64 {
        self.four_velocity(metric)[1]
    }

    /// Physical radial velocity in km/s.
    ///
    /// Nothing on screen reads it any more: the telemetry quotes dr/dt in c in both unit modes,
    /// because a six-digit km/s said less in more space. It stays because it is the conversion
    /// the tests bound against c, and because it is the honest physical form of the quantity.
    #[allow(dead_code)] // the readout is in c; the tests check the km/s conversion stays under c
    pub fn velocity_km_s(&self, metric: &KerrSchild) -> f64 {
        self.velocity_c(metric) * 299792.458
    }

    /// Angular velocity dphi/dt = u^phi / u^t: how fast the observer goes round the hole per unit
    /// of the chart's shared clock, signed, prograde positive.
    ///
    /// Unlike a coordinate *speed* this is worth printing. It is a rate of one coordinate against
    /// another with nothing pretending to be a length in it, it is the quantity the hole's own
    /// rotation rates are quoted in - `KerrSchild::frame_dragging_omega` at this radius,
    /// `inner_horizon_omega` on r-, and the circular-orbit Omega of `orbital_angular_velocity` -
    /// and comparing it to the local dragging rate is what makes the ergosphere legible: inside
    /// the static limit every timelike worldline has the same sign of Omega as the hole, whatever
    /// its thrust.
    pub fn angular_velocity(&self, metric: &KerrSchild) -> f64 {
        let u = self.four_velocity(metric);
        u[2] / u[0].max(1e-9)
    }

    /// How fast this observer is moving past a local observer of `frame`, as that observer
    /// measures it - or None where no such observer exists at this event.
    ///
    /// The speed is got from the one invariant the pair of worldlines has, the Lorentz factor
    /// gamma = -g(u, u_frame), and never from a difference of coordinate rates: v = sqrt(1 -
    /// gamma^-2) follows from gamma alone and is below c by construction for any two timelike
    /// vectors, at every radius and in every region. That is the point of routing it this way. The
    /// chart's own rates say nothing directly useful - `velocity_c` is a ratio of coordinate
    /// differentials and `proper_velocity_c` divides a coordinate by a proper time - and inside r+
    /// the radial coordinate is timelike, so a "speed" read off the chart there is not one.
    ///
    /// What it cannot do is invent a frame. A static observer exists only outside the equatorial
    /// static limit r = 2M, a ZAMO only outside r+, and inside r+ there is no hovering observer at
    /// all; those cases are None rather than a number, and the caller says so in as many words
    /// instead of quoting a speed against a worldline that is not there.
    pub fn local_speed(&self, metric: &KerrSchild, frame: LocalRestFrame) -> Option<LocalSpeed> {
        let u_frame = match frame {
            LocalRestFrame::Static => Self::static_four_velocity(metric, self.r)?,
            LocalRestFrame::Zamo => Self::zamo_four_velocity(metric, self.r)?,
            LocalRestFrame::Raindrop => Self::raindrop_four_velocity(metric, self.r),
        };
        let u = self.four_velocity(metric);
        // -g(u, u_frame) is the Lorentz factor for any two future-directed unit timelike vectors,
        // and is >= 1 with equality only when they are the same worldline. It is clamped at 1
        // because the floating-point value of a pair that *is* the same worldline - a ZAMO
        // observer measured against the ZAMO, say - lands a few ulp either side of it, and 1 - eps
        // would take the square root of a negative number.
        let gamma = (-inner(metric, self.r, &u, &u_frame)).max(1.0);
        let v = (1.0 - 1.0 / (gamma * gamma)).max(0.0).sqrt();
        Some(LocalSpeed { frame, v, gamma })
    }

    /// Every local observer at this event who can measure a speed, outermost frame first: the
    /// static observer and the ZAMO where each exists, and the raindrop alone where neither does.
    ///
    /// Both hovering frames are reported outside the static limit rather than one of them, because
    /// they disagree by a great deal where the dragging is strong and neither is the right answer
    /// to the exclusion of the other. At a = 0.90 the prograde ISCO sits at r = 2.32M, a whisker
    /// outside the static limit: an orbiter there passes the static observer at 0.898c and the
    /// ZAMO at 0.625c, because the ZAMO is itself being carried round at half the orbiter's own
    /// Omega. Printing one number would be picking a side of that; printing both is the honest
    /// statement, and their divergence *is* the frame dragging, read off directly.
    pub fn local_speeds(&self, metric: &KerrSchild) -> Vec<LocalSpeed> {
        let hovering: Vec<LocalSpeed> = [LocalRestFrame::Static, LocalRestFrame::Zamo]
            .into_iter()
            .filter_map(|f| self.local_speed(metric, f))
            .collect();
        if hovering.is_empty() {
            self.local_speed(metric, LocalRestFrame::Raindrop).into_iter().collect()
        } else {
            hovering
        }
    }

    /// The accelerometer reading in Earth g's (weightlessness = 0.0), for every mode. This is
    /// `accelerometer_geom` in geometric units (1/M) converted with a_SI = a_geom c^2 / r_g,
    /// r_g = GM/c^2 in metres, then divided by 9.80665 m/s^2.
    pub fn proper_acceleration_g(&self, metric: &KerrSchild) -> f64 {
        let accel_geom = self.accelerometer_geom(metric);
        let c = 299792458.0;
        let rg_m = metric.r_grav_km() * 1000.0;
        (accel_geom * c * c / rg_m) / 9.80665
    }

    /// Radial tidal stretching force across a 2-meter body in Earth g's
    #[allow(dead_code)] // the badge shows the gradient; the tests check the 2 m force too
    pub fn tidal_force_g(&self, metric: &KerrSchild) -> f64 {
        metric.tidal_acceleration_g(self.r, 2.0)
    }

    /// Radial tidal gradient in Earth gravities per meter (g/m)
    pub fn tidal_gradient_g_per_m(&self, metric: &KerrSchild) -> f64 {
        metric.tidal_gradient_g_per_m(self.r)
    }

    /// Frequency this observer measures for an ingoing principal null ray, as a multiple of the
    /// frequency the same ray has at infinity: nu_obs / nu_inf = -k_mu u^mu = u^t + u^r - a u^phi.
    ///
    /// This is the honest replacement for the old exterior-time heuristic. It is exact for every
    /// mode, and finite and positive everywhere in this chart, r+ and r- included: an infalling
    /// observer sees the exterior universe *red*shifted (1/2 at the Schwarzschild horizon for a
    /// raindrop), not squeezed into a flash.
    pub fn ingoing_frequency_ratio(&self, metric: &KerrSchild) -> f64 {
        metric.ingoing_frequency_ratio(self.r, &self.four_velocity(metric))
    }
}

/// The observers the app carries at once, either of whom may not be there: an observer whose
/// "Enable Observer" box is unticked is not in the simulation at all, and the two are optional in
/// the same way because they are the same idea run twice.
///
/// It exists for the same reason `SignalPair` does. Every path that moves the simulation - the
/// play loop, the arrow keys, the panel's transport buttons - has to move both worldlines the same
/// way, and a step backwards in particular has to hand both of them the same target time as the
/// clock they are being drawn against. Stating that once here is what keeps the panel's Step Back
/// button and `SpacetimeApp::step_backward` from drifting apart.
pub struct ObserverPair<'a> {
    pub bob: Option<&'a mut Observer>,
    pub alice: Option<&'a mut Observer>,
}

impl ObserverPair<'_> {
    /// Carry both worldlines to the simulation clock's new value, `dt` later than its last.
    pub fn step(&mut self, metric: &KerrSchild, current_sim_time: f64, dt: f64) {
        for obs in [self.bob.as_deref_mut(), self.alice.as_deref_mut()]
            .into_iter()
            .flatten()
        {
            obs.step(metric, current_sim_time, dt);
        }
    }

    /// Put both worldlines back where they stood when the clock read `t_target`. See
    /// `Observer::rewind_to`: the target is a time, not a number of steps, so the observers stay
    /// on the clock whatever interval the caller undid.
    pub fn rewind_to(&mut self, metric: &KerrSchild, t_target: f64) {
        for obs in [self.bob.as_deref_mut(), self.alice.as_deref_mut()]
            .into_iter()
            .flatten()
        {
            obs.rewind_to(metric, t_target);
        }
    }
}

#[allow(dead_code)] // a complete record of the drawn cone; the canvas reads part of it
#[derive(Debug, Clone, Copy)]
pub struct LightConePolygon {
    pub apex: [f64; 2],
    pub future_in: [f64; 2],
    pub future_out: [f64; 2],
    pub past_in: [f64; 2],
    pub past_out: [f64; 2],
    pub dr_dt_in: f64,
    pub dr_dt_out: f64,
    pub dphi_dt_out: f64,
    pub dphi_dt_in: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observer_manual_drag_and_cone() {
        // The drawn cone is the zero-angular-momentum wedge of the event, so it must reproduce
        // `null_wedge` exactly and tip according to the sign of Delta, in every region.
        let metric = KerrSchild::new(1.0, 0.7);
        let mut bob = Observer::new(&metric, "Bob", 0.0, 3.0, 0.0);

        let check = |cone: &LightConePolygon, r: f64| {
            let w = metric.null_wedge(r);
            assert!((cone.dr_dt_in - w.dr_dt_in).abs() < 1e-12);
            assert!((cone.dr_dt_out - w.dr_dt_out).abs() < 1e-12);
            assert!((cone.dphi_dt_in - w.dphi_dt_in).abs() < 1e-12);
            assert!((cone.dphi_dt_out - w.dphi_dt_out).abs() < 1e-12);
            assert!(cone.dr_dt_in <= -1.0 + 1e-12, "inner edge = {}", cone.dr_dt_in);
        };

        // Region I: outgoing edge is positive.
        let cone_out = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_out, 3.0);
        assert!(cone_out.dr_dt_out > 0.0);

        // Region II (between r- = 0.286 and r+ = 1.714): trapped, outgoing edge dr/dt < 0.
        bob.set_drag_position(5.0, 1.0);
        let cone_in = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_in, 1.0);
        assert!(cone_in.dr_dt_out < 0.0);

        // Region III (r < 0.286): un-tipped again, outgoing edge dr/dt > 0.
        bob.set_drag_position(8.0, 0.15);
        let cone_core = bob.compute_lightcone_polygon(&metric, 1.0);
        check(&cone_core, 0.15);
        assert!(cone_core.dr_dt_out > 0.0);

        // The wedge belongs to the event: thrusting must not change it.
        bob.set_drag_position(8.0, 3.0);
        let rest = bob.compute_lightcone_polygon(&metric, 1.0);
        bob.beta_r = 0.8;
        bob.beta_phi = -0.4;
        let boosted = bob.compute_lightcone_polygon(&metric, 1.0);
        assert_eq!(rest.dr_dt_in, boosted.dr_dt_in);
        assert_eq!(rest.dr_dt_out, boosted.dr_dt_out);
    }

    #[test]
    fn test_observer_tetrads_are_orthonormal_everywhere() {
        // Every mode the UI can select must hand `Tetrad::from_four_velocity` a unit timelike
        // vector, and the resulting frame must be exactly orthonormal in all three regions.
        use crate::physics::tetrad::inner;
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let radii = [8.0, 3.0, rp, 0.5 * (rp + rm), rm, (0.5 * rm).max(0.05)];
            for &r in radii.iter() {
                for &mode in &[
                    ObserverMode::FreeFall,
                    ObserverMode::ManualDrag,
                    ObserverMode::Static,
                    ObserverMode::Zamo,
                ] {
                    let mut obs = observer_at(&metric, mode, r);
                    if mode == ObserverMode::ManualDrag {
                        obs.beta_r = 0.5;
                        obs.beta_phi = -0.3;
                    }
                    let u = obs.four_velocity(&metric);
                    let uu = metric.norm(r, &u);
                    assert!((uu + 1.0).abs() < 1e-9, "u.u = {uu} for {mode:?} at r={r} (a={a})");

                    // The frame light leaves this observer isotropically in: the Gram-Schmidt
                    // tetrad on their own 4-velocity, which is what `SignalField::emit_if_due`
                    // builds a pulse out of.
                    let t = Tetrad::from_four_velocity(&metric, r, &u);
                    let legs = [t.e0, t.e1, t.e2];
                    for i in 0..3 {
                        for j in 0..3 {
                            let expected = match (i == j, i) {
                                (true, 0) => -1.0,
                                (true, _) => 1.0,
                                (false, _) => 0.0,
                            };
                            let got = inner(&metric, r, &legs[i], &legs[j]);
                            assert!(
                                (got - expected).abs() < 1e-9,
                                "g(e{i}, e{j}) = {got} (want {expected}) for {mode:?} at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_manual_drag_zero_boost_is_free_fall() {
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            for &r in &[9.0, 3.0, rp, 0.5 * (rp + rm), (0.5 * rm).max(0.05)] {
                let drag = observer_at(&metric, ObserverMode::ManualDrag, r);
                let free = observer_at(&metric, ObserverMode::FreeFall, r);
                let ud = drag.four_velocity(&metric);
                let uf = free.four_velocity(&metric);
                for mu in 0..3 {
                    assert!(
                        (ud[mu] - uf[mu]).abs() < 1e-10 * (1.0 + uf[mu].abs()),
                        "beta = 0 must reproduce free fall: u^{mu} {ud:?} vs {uf:?} at r={r} (a={a})"
                    );
                }
                // ...and a weightless one at that.
                assert!(drag.is_free_falling(&metric), "unboosted drag at r={r} (a={a})");
            }
        }
    }

    #[test]
    fn test_manual_drag_boost_stays_inside_the_null_wedge() {
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();
        for &r in &[8.0, 3.0, rp, 0.5 * (rp + rm), 0.5 * rm] {
            let wedge = metric.null_wedge(r);
            for &(b_r, b_phi) in &[
                (0.0, 0.0),
                (0.5, 0.0),
                (-0.5, 0.0),
                (0.9, 0.0),
                (-0.9, 0.0),
                (0.5, -0.3),
                (-0.4, 0.7),
            ] {
                let mut obs = observer_at(&metric, ObserverMode::ManualDrag, r);
                obs.beta_r = b_r;
                obs.beta_phi = b_phi;
                let u = obs.four_velocity(&metric);
                let n = metric.norm(r, &u);
                assert!((n + 1.0).abs() < 1e-9, "u.u = {n} for beta=({b_r},{b_phi}) at r={r}");
                assert!(u[0] > 0.0, "must move forward in t: {u:?}");

                let v = obs.velocity_c(&metric);
                assert!(
                    v > wedge.dr_dt_in && v < wedge.dr_dt_out,
                    "dr/dt = {v} escapes the wedge ({}, {}) for beta=({b_r},{b_phi}) at r={r}",
                    wedge.dr_dt_in,
                    wedge.dr_dt_out
                );
            }
        }
    }

    #[test]
    fn test_manual_drag_boost_costs_proper_acceleration() {
        // Because `four_acceleration` differentiates `four_velocity_at` in r, a boosted drag
        // observer automatically picks up the thrust its worldline family requires.
        let metric = KerrSchild::with_solar_mass(1.0, 0.65, 10.0);
        let r = 3.0;

        let mut boosted = observer_at(&metric, ObserverMode::ManualDrag, r);
        boosted.beta_r = 0.5;
        let a_boost = boosted.proper_acceleration_geom(&metric);
        assert!(a_boost.is_finite() && a_boost > 0.0, "|a| = {a_boost} for beta_r = 0.5");
        assert!(!boosted.is_free_falling(&metric));

        let rest = observer_at(&metric, ObserverMode::ManualDrag, r);
        let a_rest = rest.proper_acceleration_geom(&metric);
        assert!(a_rest < 1e-6, "|a| = {a_rest} for beta = 0 must vanish");
        assert!(rest.is_free_falling(&metric));
    }

    #[test]
    fn test_ingoing_frequency_ratio_schwarzschild_raindrop() {
        // Raindrop (E = 1, L = 0) in Schwarzschild: u^t = 1 + 2M/r ... but the invariant answer is
        // nu_obs / nu_inf = 1 / (1 + sqrt(2M/r)), exactly 1/2 at the horizon. The infaller
        // *red*shifts the ingoing ray: running away from it beats the gravitational blueshift.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 4.0, 2.0, 1.0, 0.3] {
            let obs = observer_at(&metric, ObserverMode::FreeFall, r);
            let got = obs.ingoing_frequency_ratio(&metric);
            let expected = 1.0 / (1.0 + (2.0 * metric.m / r).sqrt());
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs {expected} at r={r}"
            );
            assert!(got < 1.0, "an ingoing raindrop must see a redshift, got {got} at r={r}");
        }
        // The horizon value is exactly 1/2, and the ratio -> 0 at the singularity.
        let at_horizon = observer_at(&metric, ObserverMode::FreeFall, 2.0).ingoing_frequency_ratio(&metric);
        assert!((at_horizon - 0.5).abs() < 1e-12, "at r+ = 2M the ratio is 1/2: {at_horizon}");
        let deep = observer_at(&metric, ObserverMode::FreeFall, 0.01).ingoing_frequency_ratio(&metric);
        assert!(deep > 0.0 && deep < 0.1, "ratio -> 0 as r -> 0, got {deep}");
    }

    #[test]
    fn test_ingoing_frequency_ratio_static_observer() {
        // Static observer: u^mu = (1, 0, 0)/sqrt(-g_tt) and a u^phi = 0, so the ratio is
        // 1 / sqrt(1 - 2M/r): a blueshift that diverges only at the static limit r = 2M.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 4.0, 2.5] {
            let obs = observer_at(&metric, ObserverMode::Static, r);
            assert!(obs.mode_admissible(&metric));
            let got = obs.ingoing_frequency_ratio(&metric);
            let expected = 1.0 / (1.0 - 2.0 * metric.m / r).sqrt();
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs {expected} at r={r}"
            );
            assert!(got > 1.0, "a hovering observer must see a blueshift, got {got} at r={r}");
        }
    }

    #[test]
    fn test_ingoing_frequency_ratio_zamo() {
        // ZAMO: u^mu = gamma (1, 0, omega), so the ratio is gamma (1 - a omega).
        let metric = KerrSchild::new(1.0, 0.9);
        for &r in &[3.0, 5.0] {
            let obs = observer_at(&metric, ObserverMode::Zamo, r);
            assert!(obs.mode_admissible(&metric));
            let u = obs.four_velocity(&metric);
            let omega = metric.frame_dragging_omega(r);
            let gamma = u[0];
            let expected = gamma * (1.0 - metric.a * omega);
            let got = obs.ingoing_frequency_ratio(&metric);
            assert!(
                (got - expected).abs() < 1e-10,
                "nu ratio = {got} vs gamma (1 - a omega) = {expected} at r={r}"
            );
            assert!(got > 0.0 && got.is_finite());
        }
    }

    #[test]
    fn test_ingoing_frequency_ratio_kerr_raindrop_is_finite_across_both_horizons() {
        // There is no divergence on the branch of r- that an infalling observer actually crosses:
        // the ratio stays finite and positive at r+, between the horizons, at r-, and below it.
        for &a in &[0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[rp, 0.5 * (rp + rm), rm, 0.5 * rm] {
                let ratio = observer_at(&metric, ObserverMode::FreeFall, r).ingoing_frequency_ratio(&metric);
                assert!(
                    ratio.is_finite() && ratio > 0.0,
                    "nu ratio = {ratio} at r={r} (a={a}) must be finite and positive"
                );
                assert!(ratio < 1.0, "an infaller still sees a redshift: {ratio} at r={r} (a={a})");
            }
            // Closed form away from the horizons, where Delta != 0. With E = 1, L = 0 the only
            // non-trivial covariant component is u_r = (2Mr - sqrt(2Mr(r^2 + a^2))) / Delta, and
            // nu_obs/nu_inf = -k.u = E + u_r (because k^mu = (1, -1, 0) gives k^mu u_mu = u_t - u_r
            // and u_t = -E). Its numerator has the opposite sign to Delta at every radius, which is
            // why an infaller always measures a redshift, inside the horizons included.
            let closed_form = |r: f64| {
                let m = metric.m;
                1.0 + (2.0 * m * r - (2.0 * m * r * (r * r + a * a)).sqrt()) / metric.delta(r)
            };
            for &r in &[9.0, 3.0, 0.5 * (rp + rm), 0.5 * rm, 0.05] {
                let got = observer_at(&metric, ObserverMode::FreeFall, r).ingoing_frequency_ratio(&metric);
                assert!(
                    (got - closed_form(r)).abs() < 1e-10,
                    "nu ratio = {got} vs {} at r={r} (a={a})",
                    closed_form(r)
                );
            }
            // Deep inside, the a = 0 raindrop's ratio runs to 0, but with spin Delta -> a^2 and the
            // shift runs back up to 1: the ring's repulsion, not a blueshift catastrophe.
            let deep = observer_at(&metric, ObserverMode::FreeFall, 0.02).ingoing_frequency_ratio(&metric);
            assert!(deep > 0.5 && deep < 1.0, "spun-up core ratio -> 1, got {deep} (a={a})");
        }
    }

    #[test]
    fn test_observer_velocity_and_acceleration_telemetry() {
        let metric = KerrSchild::with_solar_mass(1.0, 0.7, 10.0);
        let obs = Observer::new(&metric, "Bob", 0.0, 3.0, 0.0);

        let v_c = obs.velocity_c(&metric);
        assert!(v_c.abs() < 1.0, "Velocity fraction of c magnitude must be < 1: {}", v_c);
        assert!(v_c < 0.0, "Inward infalling observer must have negative radial velocity: {}", v_c);

        let v_kms = obs.velocity_km_s(&metric);
        assert!(v_kms.abs() <= 300_000.0, "Velocity magnitude in km/s must be <= c: {}", v_kms);
        assert!(v_kms < 0.0, "Inward velocity in km/s must be negative: {}", v_kms);

        // A geodesic observer is weightless: the proper acceleration vanishes to the accuracy of
        // the central difference used for du^mu/dtau (see `four_acceleration`).
        let a_geom = obs.proper_acceleration_geom(&metric);
        assert!(
            a_geom < 1e-6,
            "Free-falling geodesic observer must have zero proper acceleration, got {a_geom}/M"
        );

        let tidal = obs.tidal_force_g(&metric);
        assert!(tidal > 0.0, "Tidal force must be positive");

        let grad = obs.tidal_gradient_g_per_m(&metric);
        assert!(grad > 0.0, "Tidal gradient must be positive");
        assert!((grad * 2.0 - tidal).abs() < 1e-6, "2-meter tidal force should be 2x the 1-meter gradient");

        // Verify inside horizon dynamics: coordinate velocity dr/dt stays within light cone, proper velocity dr/dtau exceeds -1.0c
        let obs_inside = Observer::new(&metric, "Bob", 0.0, 1.0, 0.0);
        let v_c_inside = obs_inside.velocity_c(&metric);
        let u_inside = obs_inside.proper_velocity_c(&metric);
        assert!(v_c_inside < 0.0 && v_c_inside > -1.0, "Coordinate velocity dr/dt stays causal: {}", v_c_inside);
        assert!(u_inside < -1.0, "Proper velocity dr/dtau exceeds -1.0c inside horizon: {}", u_inside);
    }

    #[test]
    fn test_delayed_release_worldline_starts_at_release_event() {
        // Bob released at t = 4 must sit at t = 4 (not t = 0) when he starts falling, so his
        // worldline is NOT a copy of Alice's shifted down the diagram.
        let metric = KerrSchild::new(1.0, 0.65);
        let mut alice = Observer::new(&metric, "Alice", 0.0, 4.5, 0.0);
        let mut bob = Observer::new(&metric, "Bob", 0.0, 4.5, 4.0);
        let dt = 0.05;
        let mut t = 0.0;
        while t < 3.95 {
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
        }
        assert!(!bob.is_active);
        assert!((bob.t - t).abs() < 1e-9, "hovering Bob must track the simulation clock");
        assert!((bob.r - 4.5).abs() < 1e-12);
        assert!(bob.tau > 0.0, "a hovering observer's clock still runs");
        while t < 6.0 {
            t += dt;
            alice.step(&metric, t, dt);
            bob.step(&metric, t, dt);
        }
        assert!(bob.is_active);
        assert!((bob.t - t).abs() < 1e-9, "Bob's coordinate time must equal the simulation clock: {} vs {}", bob.t, t);
        assert!(bob.r > alice.r + 0.5, "Bob released later must trail Alice: bob.r={} alice.r={}", bob.r, alice.r);
        // First trail point is the initial hover event, then the release event follows.
        assert_eq!((bob.trail[0].t, bob.trail[0].r, bob.trail[0].phi), (0.0, 4.5, 0.0));
        assert!(
            (bob.trail[1].t - 4.0).abs() < 0.06,
            "trail must show the release event near t=4: {:?}",
            bob.trail[1]
        );
    }

    #[test]
    fn test_observer_cartesian_position_and_trail_carries_phi() {
        // The observer's top-down position is the Kerr-Schild embedding of (r, phi), so it sits at
        // Cartesian radius sqrt(r^2 + a^2) and polar angle `azimuth`, and the trail records the
        // azimuth needed to redraw that curve.
        let metric = KerrSchild::new(1.0, 0.8);
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, 5.0, 0.0, 0.4, WorldlineParams::default());
        assert_eq!((bob.trail[0].t, bob.trail[0].r, bob.trail[0].phi), (0.0, 5.0, 0.4));

        let mut t = 0.0;
        while bob.r > 0.021 && t < 400.0 {
            t += 0.1;
            bob.step(&metric, t, 0.1);
        }
        assert!(bob.r < 0.03, "the raindrop must reach the ring: r = {}", bob.r);
        assert!(
            (bob.phi - 0.4).abs() > 1e-3,
            "the worldline must wind in phi, not fall straight in: {}",
            bob.phi
        );

        for point in bob.trail.iter() {
            let (t, r) = (point.t, point.r);
            let (x, y) = metric.cartesian_position(r, point.phi);
            let rho = (x * x + y * y).sqrt();
            assert!(
                (rho - metric.cartesian_radius(r)).abs() < 1e-12,
                "trail point (t={t}, r={r}) is off the constant-r circle"
            );
        }
        // The last trail entry is the current event, and matches `cartesian_position`.
        let last = *bob.trail.back().unwrap();
        let (x, y) = bob.cartesian_position(&metric);
        let (ex, ey) = metric.cartesian_position(last.r, last.phi);
        assert!((x - ex).abs() < 1e-12 && (y - ey).abs() < 1e-12);
        let psi = bob.azimuth(&metric);
        assert!((y.atan2(x) - psi).sin().abs() < 1e-12, "polar angle must be `azimuth`");
        // The infall ends *on the ring* rho = a, not at the Cartesian origin.
        let rho_end = (x * x + y * y).sqrt();
        assert!(
            rho_end >= metric.a.abs() - 1e-12,
            "no equatorial point lies inside the ring: rho = {rho_end}"
        );
        assert!(
            (rho_end - metric.a.abs()).abs() < 1e-3,
            "the trail must terminate on the ring rho = a = {}, got {rho_end}",
            metric.a.abs()
        );
    }

    /// The size of the kink in the worldline at the release: |u after - u before|, in the local
    /// sense that matters, which is the relative speed between the two four-velocities. Zero means
    /// the fall carries on from the hover without anything happening to the observer's motion.
    fn release_jump(metric: &KerrSchild, release: Release, l_ang: f64) -> f64 {
        let params = WorldlineParams::released(metric, 4.5, l_ang, release);
        let mut obs = Observer::new_with_phi(metric, "Bob", 0.0, 4.5, 4.0, 0.0, params);
        // Sampled either side of the release: the last frame of the wait, and the first frame of
        // the fall. The step across it is small enough that the falling itself moves the
        // four-velocity by ~1e-4, so anything larger than that is the kink.
        let mut t = 0.0;
        while t < 3.999 - 1e-12 {
            let step = 0.1f64.min(3.999 - t);
            t += step;
            obs.step(metric, t, step);
        }
        assert!(!obs.is_active, "the sample before the release must be taken while waiting");
        let before = obs.four_velocity(metric);
        t += 0.002;
        obs.step(metric, t, 0.002);
        assert!(obs.is_active, "and the sample after it once they are let go");
        let after = obs.four_velocity(metric);
        // gamma = -u_before . u_after is 1 for identical vectors and grows with the relative
        // speed between them; the speed itself is the legible number.
        let g = metric.metric_components(obs.r);
        let mut gamma = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                gamma -= g[i][j] * before[i] * after[j];
            }
        }
        (1.0 - 1.0 / (gamma * gamma).max(1.0)).max(0.0).sqrt()
    }

    #[test]
    fn test_a_release_at_rest_carries_on_from_the_hover_without_a_jump() {
        // What "the run begins when they cut their engines" has to mean. An observer waiting out a
        // release delay holds their radius on the worldline they are about to fall on, so at the
        // release nothing happens to their motion at all: the thrust stops and the same
        // four-velocity carries on as a geodesic. Anything else is an infinite acceleration drawn
        // as a corner in the worldline - which is what this used to be, because the wait was spent
        // as a *static* observer and the fall began at rest in r, and those differ by the frame
        // dragging even for L = 0.
        //
        // Released from rest at infinity there is nothing to hold: that worldline is already doing
        // two thirds of the speed of light at 4.5M, so the jump is real and is the honest statement
        // that the observer did not come from here. Both are measured, and the point is the ratio.
        let metric = KerrSchild::new(1.0, 0.90);
        let at_rest = release_jump(&metric, Release::AtRest, 0.0);
        let spun = release_jump(&metric, Release::AtRest, 2.0);
        let raindrop = release_jump(&metric, Release::FromInfinity, 0.0);
        println!(
            "the kink at the release, as a relative speed: at rest {at_rest:.2e}, at rest with \
             L = 2 {spun:.2e}, from rest at infinity {raindrop:.4}"
        );
        assert!(at_rest < 1e-3, "an at-rest release is smooth: {at_rest}");
        assert!(spun < 1e-3, "with angular momentum too: {spun}");
        assert!(raindrop > 0.6, "and a raindrop arrives already moving: {raindrop}");
    }

    #[test]
    fn test_released_drag_resumes_free_fall_from_the_new_event() {
        let metric = KerrSchild::new(1.0, 0.65);
        let mut bob = Observer::new(&metric, "Bob", 0.0, 3.8, 0.0);
        bob.step(&metric, 0.5, 0.5);
        let r_before_drag = bob.r;

        bob.set_drag_position(2.0, 4.6);
        assert_eq!(bob.mode, ObserverMode::ManualDrag);
        bob.step(&metric, 2.5, 0.5);
        assert!((bob.r - 4.6).abs() < 1e-12, "manual drag holds r while dragging");

        bob.release_from_drag(&metric, ObserverMode::FreeFall);
        assert_eq!(bob.mode, ObserverMode::FreeFall);
        assert!(bob.is_active);
        let u = bob.four_velocity(&metric);
        assert!((metric.norm(bob.r, &u) + 1.0).abs() < 1e-9, "re-seeded u must be unit timelike");
        assert!(u[1] < 0.0, "resumes on the ingoing root");

        bob.step(&metric, 3.0, 0.5);
        assert!(bob.r < 4.6, "free fall resumes from the dropped radius: r = {}", bob.r);
        assert!(bob.r > r_before_drag, "and from the new event, not the pre-drag one");
        assert!((bob.t - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_rewind_lands_on_the_clock_whatever_the_step_was() {
        // The rewind is stated in time, so the worldline lands on the target however the caller
        // got there: the observer's own clock equals the target afterwards, and it does so whether
        // the interval undone is a whole recorded step, a fraction of one, or several at once.
        // Popping one recorded event per call - what this used to do - gets all three wrong.
        let metric = KerrSchild::new(1.0, 0.6);
        for &back in &[0.05, 0.1, 0.37, 1.0] {
            let mut obs = Observer::new(&metric, "Bob", 0.0, 4.0, 0.0);
            for i in 1..=20 {
                obs.step(&metric, (i as f64) * 0.1, 0.1);
            }
            let forward_r = obs.r;
            let target = obs.t - back;
            obs.rewind_to(&metric, target);
            assert!(
                (obs.t - target).abs() < 1e-12,
                "rewind by {back} left the observer at t = {} instead of {target}",
                obs.t
            );
            assert!(obs.r > forward_r, "and it must be back up the worldline: r = {}", obs.r);
            assert!(
                obs.trail.back().is_some_and(|p| (p.t - target).abs() < 1e-12),
                "the trail must end on the current event: {:?}",
                obs.trail.back()
            );
        }
    }

    #[test]
    fn test_rewind_round_trip_is_exact_for_a_faller_and_for_a_hoverer() {
        // Forward N, back M, forward M: the second pass has to land on the first, exactly. It does
        // because the trail records the proper time and the integrated 4-velocity, so a rewind
        // onto a recorded event restores the state rather than re-deriving it, and the steps that
        // follow are then the same steps over the same intervals.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let dt = 0.05;
        for &release_t in &[0.0, 3.0, 1.07] {
            let mut obs =
                Observer::new_with_phi(&metric, "Probe", 0.0, 4.5, release_t, 0.25, params);
            let mut t = 0.0;
            for _ in 0..40 {
                t += dt;
                obs.step(&metric, t, dt);
            }
            let reference = (obs.t, obs.r, obs.phi, obs.tau);
            let back = 12;
            obs.rewind_to(&metric, t - (back as f64) * dt);
            let mid = obs.t;
            assert!(
                (mid - (t - (back as f64) * dt)).abs() < 1e-12,
                "the rewind must land on the target: {mid}"
            );
            for _ in 0..back {
                let step_to = obs.t + dt;
                obs.step(&metric, step_to, dt);
            }
            let again = (obs.t, obs.r, obs.phi, obs.tau);
            for (a, b, what) in [
                (again.0, reference.0, "t"),
                (again.1, reference.1, "r"),
                (again.2, reference.2, "phi"),
                (again.3, reference.3, "tau"),
            ] {
                assert!(
                    (a - b).abs() < 1e-9,
                    "release_t = {release_t}: {what} came back as {a}, not {b}"
                );
            }
            println!(
                "release_t = {release_t}: round trip residuals dt = {:.2e}, dr = {:.2e}, \
                 dphi = {:.2e}, dtau = {:.2e}",
                (again.0 - reference.0).abs(),
                (again.1 - reference.1).abs(),
                (again.2 - reference.2).abs(),
                (again.3 - reference.3).abs()
            );
        }
    }

    #[test]
    fn test_a_released_worldline_stays_on_the_simulation_clock() {
        // The fall starts at t = release_t, so an observer released between two steps is on the
        // clock from the first released step onward. Integrating a whole step from release_t
        // instead put the worldline permanently ahead of the clock by whatever the release missed
        // the step boundary by - 0.03 M in the case below - which then showed up as an observer
        // whose marker sat above their own light cone's apex.
        let metric = KerrSchild::new(1.0, 0.65);
        for &(release_t, dt) in &[(4.0, 0.05), (4.03, 0.1), (1.234, 0.25)] {
            let mut bob = Observer::new(&metric, "Bob", 0.0, 4.5, release_t);
            let mut t = 0.0;
            while t < 6.0 {
                t += dt;
                bob.step(&metric, t, dt);
                assert!(
                    bob.has_ended() || (bob.t - t).abs() < 1e-9,
                    "release_t = {release_t}, dt = {dt}: at clock {t} the observer is at {}",
                    bob.t
                );
            }
        }
    }

    #[test]
    fn test_rewind_into_the_hover_restores_the_static_clock() {
        // A hovering observer's proper time is sqrt(-g_tt) (t - t_start) exactly, with g_tt at the
        // radius they are standing at, and a rewind that crosses their release has to put them
        // back on that clock rather than on a subtraction: `SignalField` paces their transmission
        // by it, so a cadence put back wrong re-cuts the whole transmission somewhere else.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 4.0, 0.0, params);
        let dt = 0.05;
        let mut t = 0.0;
        while t < 6.0 {
            t += dt;
            bob.step(&metric, t, dt);
        }
        assert!(bob.is_active && bob.r < 4.5, "he must have been released and fallen");

        let target = 2.5;
        bob.rewind_to(&metric, target);
        let g_tt = metric.metric_components(4.5)[0][0];
        let expected_tau = (-g_tt).sqrt() * target;
        assert!(!bob.is_active, "below his release he is hovering again");
        assert!((bob.t - target).abs() < 1e-12, "on the clock: {}", bob.t);
        assert!((bob.r - 4.5).abs() < 1e-12, "back at his hover radius: {}", bob.r);
        assert!(
            (bob.tau - expected_tau).abs() < 1e-12,
            "hover clock {} vs sqrt(-g_tt)(t - t_start) = {expected_tau}",
            bob.tau
        );
        // And the incremental hover agrees with that closed form to the same accuracy, which is
        // what makes the rewind an inverse of the forward run and not merely a plausible state.
        let mut fresh = Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 4.0, 0.0, params);
        let mut t = 0.0;
        while t < target - 1e-12 {
            t += dt;
            fresh.step(&metric, t, dt);
        }
        assert!(
            (fresh.tau - bob.tau).abs() < 1e-12,
            "rewound hover clock {} vs the forward run's {}",
            bob.tau,
            fresh.tau
        );
    }

    #[test]
    fn test_a_worldline_that_has_ended_stays_ended_until_the_clock_drops_below_it() {
        // Alice reaches the ring and her clock stops there while the simulation clock runs on.
        // A rewind that does not reach her death event must leave her on the ring - the old
        // step-back pulled her off it, one recorded event per call, however small the interval -
        // and one that does reach past it must put her back on the worldline.
        let metric = KerrSchild::new(1.0, 0.9);
        let mut alice = Observer::new(&metric, "Alice", 0.0, 4.5, 0.0);
        let dt = 0.1;
        let mut t = 0.0;
        while t < 10.0 {
            t += dt;
            alice.step(&metric, t, dt);
        }
        assert!(alice.has_ended(), "she must have reached the ring: r = {}", alice.r);
        let end = (alice.t, alice.r, alice.phi, alice.tau);
        assert!(end.0 < 9.0, "and done it well before the clock stopped: t_end = {}", end.0);

        for &target in &[9.9, 8.0, end.0 + 1e-6] {
            let mut wound = alice.clone();
            wound.rewind_to(&metric, target);
            assert_eq!(
                (wound.t, wound.r, wound.phi, wound.tau),
                end,
                "a rewind to t = {target}, still past her end at t = {}, moved her",
                end.0
            );
            assert!(wound.has_ended());
        }

        let mut wound = alice.clone();
        wound.rewind_to(&metric, end.0 - 0.25);
        assert!((wound.t - (end.0 - 0.25)).abs() < 1e-12, "on the clock: {}", wound.t);
        assert!(!wound.has_ended(), "a quarter of an M before the ring she is still falling");
        assert!(wound.r > end.1, "and above it: r = {} vs {}", wound.r, end.1);
    }


    #[test]
    fn test_free_fall_telemetry_follows_the_integrated_worldline() {
        // A released free-faller reports the 4-velocity of the worldline being drawn, not the
        // closed-form ingoing solution at the same radius. On a bound orbit the two part company
        // completely: after perihelion the integrated u^r is positive while the ingoing closed
        // form is, by construction, still negative.
        let metric = KerrSchild::new(1.0, 0.0);
        let params = WorldlineParams::new(0.97, 4.0, false);
        let mut obs = Observer::new_with_phi(&metric, "Probe", 0.0, 12.0, 0.0, 0.0, params);
        assert!(obs.is_active);

        let mut saw_outgoing = false;
        let mut t = 0.0;
        while t < 400.0 {
            t += 0.5;
            obs.step(&metric, t, 0.5);
            let geo = obs.geodesic.expect("a free-faller carries a geodesic state");
            let u = obs.four_velocity(&metric);
            assert_eq!(u, geo.u, "telemetry must read the integrated 4-velocity");
            assert!((metric.norm(obs.r, &u) + 1.0).abs() < 1e-8, "u.u = {}", metric.norm(obs.r, &u));
            assert!(
                (obs.velocity_c(&metric) - u[1] / u[0]).abs() < 1e-12
                    && obs.proper_velocity_c(&metric) == u[1],
                "velocity telemetry must come from the same u"
            );
            if u[1] > 0.05 {
                saw_outgoing = true;
                // The old first-order solution only ever describes the ingoing root...
                let (_, dr_ingoing, _) = geo.derivatives(&metric, obs.r);
                assert!(dr_ingoing < 0.0, "the ingoing closed form stays ingoing: {dr_ingoing}");
                // ...while the congruence the proper acceleration is measured against follows
                // the root the observer is actually on, so the climb is still weightless.
                assert!(obs.four_velocity_at(&metric, obs.r)[1] > 0.0);
                assert!(obs.is_free_falling(&metric), "a geodesic stays weightless after the turn");
                let residual = obs.proper_acceleration_geom(&metric);
                assert!(
                    residual < 1e-6 * metric.kretschmann_scalar(obs.r).sqrt(),
                    "and the integrator residual says so too: {residual}"
                );
                assert!(obs.r > 7.0, "climbing away from perihelion, r = {}", obs.r);
            }
        }
        assert!(saw_outgoing, "the bound orbit must turn around within t = 400");
        assert!(obs.r <= 23.2 && obs.r >= 7.6, "r = {} left the bound band", obs.r);
    }

    #[test]
    fn test_a_forbidden_start_is_clamped_up_to_the_energy_floor() {
        // Dropping with too little energy for the chosen L is not an error: E is raised to the
        // floor, so the observer starts at a turning point instead of nowhere.
        let metric = KerrSchild::new(1.0, 0.65);
        let floor = GeodesicState::energy_floor(&metric, 4.5, 3.5);
        let obs = Observer::new_with_phi(
            &metric,
            "Probe",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(0.90, 3.5, false),
        );
        let geo = obs.geodesic.unwrap();
        assert!(floor > 0.90 && (geo.energy - floor).abs() < 1e-12, "E = {}", geo.energy);
        assert!(geo.u[1].abs() < 1e-6, "a clamped start is a turning point: {:?}", geo.u);
        assert!((metric.norm(4.5, &obs.four_velocity(&metric)) + 1.0).abs() < 1e-9);
    }

    /// Build an observer parked at radius r in the given mode (E = 1, L = 0 free-fall data).
    fn observer_at(metric: &KerrSchild, mode: ObserverMode, r: f64) -> Observer {
        let mut obs = Observer::new(metric, "Probe", 0.0, r, 0.0);
        obs.mode = mode;
        obs
    }

    #[test]
    fn test_static_observer_acceleration_schwarzschild() {
        // Schwarzschild static observer: |a| = M / (r^2 sqrt(1 - 2M/r)).
        let metric = KerrSchild::new(1.0, 0.0);
        let r = 4.0;
        let obs = observer_at(&metric, ObserverMode::Static, r);
        assert!(obs.mode_admissible(&metric));

        let u = obs.four_velocity(&metric);
        assert!(u[1] == 0.0 && u[2] == 0.0, "static observer must not move: {u:?}");
        assert!((metric.norm(r, &u) + 1.0).abs() < 1e-12, "u.u = {}", metric.norm(r, &u));

        let expected = metric.m / (r * r * (1.0 - 2.0 * metric.m / r).sqrt());
        let got = obs.proper_acceleration_geom(&metric);
        assert!((got - expected).abs() < 1e-8, "|a| = {got} vs {expected}");
    }

    #[test]
    fn test_static_observer_acceleration_kerr() {
        // Equatorial Kerr static observer: |a| = sqrt(Delta) M / (r^2 (r - 2M)).
        let metric = KerrSchild::new(1.0, 0.65);
        let r = 5.0;
        let obs = observer_at(&metric, ObserverMode::Static, r);
        assert!(obs.mode_admissible(&metric));
        assert!((metric.norm(r, &obs.four_velocity(&metric)) + 1.0).abs() < 1e-12);

        let expected = metric.delta(r).sqrt() * metric.m / (r * r * (r - 2.0 * metric.m));
        let got = obs.proper_acceleration_geom(&metric);
        assert!((got - expected).abs() < 1e-8, "|a| = {got} vs {expected}");
    }

    #[test]
    fn test_zamo_reduces_to_static_without_spin() {
        // With a = 0 there is no frame dragging, so the ZAMO is the static observer.
        let metric = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 6.0, 4.0, 2.5, 2.05] {
            let stat = observer_at(&metric, ObserverMode::Static, r);
            let zamo = observer_at(&metric, ObserverMode::Zamo, r);
            assert!(stat.mode_admissible(&metric) && zamo.mode_admissible(&metric));
            let us = stat.four_velocity(&metric);
            let uz = zamo.four_velocity(&metric);
            for mu in 0..3 {
                assert!((us[mu] - uz[mu]).abs() < 1e-12, "u^{mu} differs at r={r}: {us:?} {uz:?}");
            }
            let a_s = stat.proper_acceleration_geom(&metric);
            let a_z = zamo.proper_acceleration_geom(&metric);
            assert!((a_s - a_z).abs() < 1e-12, "|a| differs at r={r}: {a_s} vs {a_z}");
        }
    }

    #[test]
    fn test_zamo_acceleration_is_the_lapse_gradient() {
        // For an equatorial ZAMO the proper acceleration is the gradient of the lapse:
        //     |a| = sqrt(g^rr) d(ln alpha)/dr,   alpha^2 = Delta r^2 / ((r^2 + a^2)^2 - a^2 Delta).
        let metric = KerrSchild::new(1.0, 0.9);
        let ln_alpha = |r: f64| {
            let a2 = metric.a * metric.a;
            let d = metric.delta(r);
            let alpha_sq = d * r * r / ((r * r + a2) * (r * r + a2) - a2 * d);
            0.5 * alpha_sq.ln()
        };

        for &r in &[3.0, 5.0, 8.0] {
            let zamo = observer_at(&metric, ObserverMode::Zamo, r);
            assert!(zamo.mode_admissible(&metric));

            let u = zamo.four_velocity(&metric);
            let n = metric.norm(r, &u);
            assert!((n + 1.0).abs() < 1e-10, "ZAMO u.u = {n} at r={r}");
            assert!(u[1] == 0.0 && u[2] > 0.0, "ZAMO must co-rotate at fixed r: {u:?}");

            let got = zamo.proper_acceleration_geom(&metric);
            assert!(got.is_finite() && got > 0.0, "|a| = {got} at r={r}");

            let h = 1e-5 * r;
            let dln = (ln_alpha(r + h) - ln_alpha(r - h)) / (2.0 * h);
            let expected = metric.g_upper_rr(r).sqrt() * dln;
            assert!((got - expected).abs() < 1e-6, "|a| = {got} vs lapse gradient {expected} at r={r}");
        }
    }

    #[test]
    fn test_free_fall_worldline_is_a_geodesic_of_the_coded_metric() {
        // a^mu = du^mu/dtau + Gamma^mu_{ab} u^a u^b must vanish for the exact infall solution,
        // everywhere including at r+ and deep inside the Cauchy horizon.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon();
            for &r in &[6.0, 3.0, rp, 1.0, (0.5 * rm).max(0.05)] {
                let obs = observer_at(&metric, ObserverMode::FreeFall, r);
                let u = obs.four_velocity(&metric);
                assert!((metric.norm(r, &u) + 1.0).abs() < 1e-8, "u.u at r={r} (a={a})");

                let acc = obs.four_acceleration(&metric);
                let biggest = acc.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                assert!(biggest < 1e-6, "a^mu = {acc:?} at r={r} (a={a})");
                assert!(
                    metric.norm(r, &acc).abs() < 1e-6,
                    "|a|^2 = {} at r={r} (a={a})",
                    metric.norm(r, &acc)
                );
                assert!(obs.proper_acceleration_geom(&metric) < 1e-6);
            }
        }
    }

    #[test]
    fn test_inadmissible_modes_fall_back_to_free_fall() {
        let metric = KerrSchild::new(1.0, 0.65);

        // r = 1.5 is inside the equatorial static limit 2M: no static observer exists.
        let stat = observer_at(&metric, ObserverMode::Static, 1.5);
        assert!(!stat.mode_admissible(&metric));
        let free = observer_at(&metric, ObserverMode::FreeFall, 1.5);
        assert_eq!(stat.four_velocity(&metric), free.four_velocity(&metric));

        // Inside r+ nothing can hover, so the ZAMO is inadmissible too.
        let inside = 0.5 * (metric.outer_horizon() + metric.inner_horizon());
        let zamo = observer_at(&metric, ObserverMode::Zamo, inside);
        assert!(!zamo.mode_admissible(&metric));
        let free_in = observer_at(&metric, ObserverMode::FreeFall, inside);
        assert_eq!(zamo.four_velocity(&metric), free_in.four_velocity(&metric));

        // ... but a ZAMO in the ergosphere (r+ < r < 2M), where no static observer exists, is fine.
        let ergo = 0.5 * (metric.outer_horizon() + 2.0 * metric.m);
        let zamo_ergo = observer_at(&metric, ObserverMode::Zamo, ergo);
        assert!(zamo_ergo.mode_admissible(&metric));
        assert!(!observer_at(&metric, ObserverMode::Static, ergo).mode_admissible(&metric));
    }

    #[test]
    fn test_static_and_zamo_stepping_uses_the_four_velocity() {
        let metric = KerrSchild::new(1.0, 0.8);
        let dt = 0.1;

        let mut stat = observer_at(&metric, ObserverMode::Static, 5.0);
        let u_s = stat.four_velocity(&metric);
        stat.step(&metric, dt, dt);
        assert!((stat.r - 5.0).abs() < 1e-15, "static observer must not move radially");
        assert!(stat.phi.abs() < 1e-15, "static observer must not rotate");
        assert!((stat.tau - dt / u_s[0]).abs() < 1e-12);

        let mut zamo = observer_at(&metric, ObserverMode::Zamo, 5.0);
        let u_z = zamo.four_velocity(&metric);
        zamo.step(&metric, dt, dt);
        assert!((zamo.r - 5.0).abs() < 1e-15);
        let omega = metric.frame_dragging_omega(5.0);
        assert!((zamo.phi - omega * dt).abs() < 1e-12, "ZAMO must drift at omega");
        assert!((zamo.tau - dt / u_z[0]).abs() < 1e-12);
    }

    #[test]
    fn test_static_observer_acceleration_diverges_at_the_static_limit() {
        // Hovering costs more and more thrust as r -> 2M, and becomes impossible below it.
        let metric = KerrSchild::new(1.0, 0.5);
        let far = observer_at(&metric, ObserverMode::Static, 20.0).proper_acceleration_geom(&metric);
        let near = observer_at(&metric, ObserverMode::Static, 2.01).proper_acceleration_geom(&metric);
        assert!(near > far * 100.0, "near = {near}, far = {far}");
        // ZAMO acceleration stays finite through the ergosphere and diverges only at r+.
        let ergo = observer_at(&metric, ObserverMode::Zamo, 2.0).proper_acceleration_geom(&metric);
        assert!(ergo.is_finite() && ergo > 0.0, "ZAMO |a| at the static limit = {ergo}");
    }

    #[test]
    fn test_an_impossible_fixed_r_selection_falls_in_position_as_well_as_in_velocity() {
        // The bug this test exists for: `four_velocity_at` fell back to free fall where a Static
        // or ZAMO selection was impossible, while `advance` went on running the fixed-r arm from
        // that borrowed 4-velocity. The observer was then drawn standing still at a radius nothing
        // can stand still at, while his telemetry, his emission tetrad and the frame his arrivals
        // were measured in all belonged to a worldline falling inward at 0.73c.
        //
        // At a = 0.90 the outer horizon is r+ = 1.436M and the equatorial static limit is 2M, so
        // r = 1.9M is inside the ergosphere (no static observer, but a perfectly good ZAMO) and
        // r = 1.2M is inside r+ (no ZAMO either). Both selections must now fall, and fall along
        // exactly the geodesic a Free Fall selection would have followed from the same event.
        let metric = KerrSchild::new(1.0, 0.90);
        assert!(metric.outer_horizon() < 1.9 && 1.9 < 2.0 * metric.m);
        assert!(1.2 < metric.outer_horizon());

        for &(mode, r0) in &[(ObserverMode::Static, 1.9), (ObserverMode::Zamo, 1.2)] {
            let mut obs = observer_at(&metric, mode, r0);
            let mut reference = observer_at(&metric, ObserverMode::FreeFall, r0);
            assert!(!obs.mode_admissible(&metric), "{mode:?} must be impossible at r = {r0}");
            assert_eq!(obs.effective_mode(&metric), ObserverMode::FreeFall);

            let dt = 0.01;
            let mut t = 0.0;
            let mut steps_that_moved = 0;
            while t < 2.0 - 1e-12 {
                let r_before = obs.r;
                let tau_before = obs.tau;
                t += dt;
                obs.step(&metric, t, dt);
                reference.step(&metric, t, dt);

                // The selection is kept: it is the user's standing request, and it would resume by
                // itself if the observer ever got back out to a radius where it exists.
                assert_eq!(obs.mode, mode, "the selection must survive the fallback");

                if obs.has_ended() {
                    break;
                }
                steps_that_moved += 1;
                assert!(obs.r < r_before, "{mode:?} at r0 = {r0}: r = {} did not fall", obs.r);

                let geo = obs.geodesic.expect("the fallback must be riding a geodesic");
                let u = obs.four_velocity(&metric);
                assert_eq!(u, geo.u, "{mode:?} at r0 = {r0}: telemetry is off the worldline");
                let uu = metric.norm(obs.r, &u);
                assert!((uu + 1.0).abs() < 1e-8, "{mode:?} at r0 = {r0}: u.u = {uu}");
                assert_eq!(obs.tau, geo.tau, "{mode:?}: the clock must be the geodesic's");
                assert!(obs.tau > tau_before, "{mode:?}: and it must run");

                // Same event, same equation, same integrator as a Free Fall selection would give.
                assert_eq!(
                    (obs.t, obs.r, obs.phi, obs.tau),
                    (reference.t, reference.r, reference.phi, reference.tau),
                    "{mode:?} at r0 = {r0}: the fallback worldline is not the free-fall one"
                );
            }
            assert!(steps_that_moved > 20, "{mode:?} at r0 = {r0}: nothing was integrated");
            println!(
                "{mode:?} refused at r0 = {r0}: fell to r = {:.4} in {steps_that_moved} steps, \
                 tau = {:.4}",
                obs.r, obs.tau
            );
        }
    }

    #[test]
    fn test_every_mode_steps_along_the_four_velocity_it_reports() {
        // The invariant that would have caught the bug, and the one that keeps it caught: over one
        // step of dt the observer's (r, phi) must change by (u^r/u^t, u^phi/u^t) dt to first order,
        // with u the 4-velocity `four_velocity` reported at the *start* of the step. It is the
        // statement that the drawn worldline and the quoted 4-velocity are one object, which is
        // what every derived quantity - the emission tetrad, the reception frame, the telemetry -
        // silently assumes.
        //
        // The residual is O(dt^2) by construction, since the exact change is
        // (u^r/u^t) dt + O(dt^2) whatever the mode; with dt = 1e-4 the measured worst case over
        // this grid is 2.41e-8 in r and 2.39e-8 in phi, both for free fall at r = 0.3M where u
        // varies on the scale of r itself, i.e. 2.4 dt^2. The tolerance below is 50 dt^2, a factor
        // of 20 of headroom over that and still three orders of magnitude tighter than the failure
        // this test is here to catch: a Static selection at r = 1.9M, held at fixed r while
        // reporting dr/dt = -0.73c, misses by 7.3e-5 = 7300 dt^2.
        let metric = KerrSchild::new(1.0, 0.90);
        let dt = 1e-4;
        let radii = [0.3, 0.5, 0.8, 1.0, 1.2, 1.436, 1.5, 1.9, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0];
        let mut worst = (0.0f64, 0.0f64, ObserverMode::FreeFall, 0.0f64);

        for &mode in &[
            ObserverMode::FreeFall,
            ObserverMode::Static,
            ObserverMode::Zamo,
            ObserverMode::ManualDrag,
        ] {
            for &r0 in radii.iter() {
                let mut obs = observer_at(&metric, mode, r0);
                let u = obs.four_velocity(&metric);
                let ut = u[0];
                assert!(ut > 0.0, "{mode:?} at r = {r0}: u^t = {ut} does not move forward in t");
                assert!(
                    (metric.norm(r0, &u) + 1.0).abs() < 1e-8,
                    "{mode:?} at r = {r0}: u.u = {}",
                    metric.norm(r0, &u)
                );

                obs.step(&metric, dt, dt);

                if mode == ObserverMode::ManualDrag {
                    // The one mode whose worldline is an *input*. Its position is the user's mouse,
                    // not an integration, so there is no step for the invariant to be about: beta
                    // states a 4-velocity at the event under the cursor, and `advance` carries the
                    // clock and nothing else. The consistency that can be demanded of it is that
                    // the position does not drift out from under the user's hand.
                    assert_eq!(
                        (obs.r, obs.phi),
                        (r0, 0.0),
                        "a dragged observer is positioned, not stepped"
                    );
                    continue;
                }

                // `GeodesicState` folds phi into [0, 2 pi), so the step's change in azimuth is the
                // shortest way round from where it started, not the raw difference.
                let two_pi = 2.0 * std::f64::consts::PI;
                let d_phi = (obs.phi - 0.0 + std::f64::consts::PI).rem_euclid(two_pi)
                    - std::f64::consts::PI;
                let res_r = (obs.r - r0 - (u[1] / ut) * dt).abs();
                let res_phi = (d_phi - (u[2] / ut) * dt).abs();
                assert!(
                    res_r < 50.0 * dt * dt && res_phi < 50.0 * dt * dt,
                    "{mode:?} at r = {r0}: stepped to (dr, dphi) = ({}, {}) but reported \
                     ({}, {}) dt - residuals ({res_r:.3e}, {res_phi:.3e})",
                    obs.r - r0,
                    obs.phi,
                    u[1] / ut,
                    u[2] / ut
                );
                if res_r.max(res_phi) > worst.0.max(worst.1) {
                    worst = (res_r, res_phi, mode, r0);
                }
            }
        }
        println!(
            "worst (r, phi) residual over the grid at dt = {dt:.0e}: \
             ({:.3e}, {:.3e}) for {:?} at r = {} - dt^2 = {:.0e}",
            worst.0, worst.1, worst.2, worst.3, dt * dt
        );
    }

    #[test]
    fn test_a_hovering_observer_reports_the_clock_it_is_actually_keeping() {
        // Before release the observer is held at fixed (r, phi) with dtau = sqrt(-g_tt) dt, so the
        // worldline under them is the static one and that is the 4-velocity they must report. This
        // used to report the free-fall value at the hover radius instead - the second, older
        // instance of the same inconsistency - and `wavefront::signalling_four_velocity` corrected
        // for it locally so that the pulses at least went out into the right frame.
        let metric = KerrSchild::new(1.0, 0.65);
        let params = WorldlineParams::default();
        let mut bob = Observer::new_with_phi(&metric, "Bob", 0.0, 4.5, 8.0, 0.0, params);
        bob.step(&metric, 0.5, 0.5);
        assert!(!bob.is_active && bob.tau > 0.0, "he is hovering, and his clock runs");

        let g_tt = metric.metric_components(4.5)[0][0];
        let u = bob.four_velocity(&metric);
        assert_eq!(u, [1.0 / (-g_tt).sqrt(), 0.0, 0.0], "the static 4-velocity: {u:?}");
        assert!((metric.norm(4.5, &u) + 1.0).abs() < 1e-12, "u.u = {}", metric.norm(4.5, &u));
        // And the clock the hover keeps is that worldline's: dtau/dt = 1/u^t.
        assert!(
            (bob.tau - 0.5 / u[0]).abs() < 1e-12,
            "tau = {} vs dt / u^t = {}",
            bob.tau,
            0.5 / u[0]
        );
        // Which makes him a rocket, not a free-faller, and the telemetry now says so.
        assert!(!bob.is_free_falling(&metric), "hovering costs thrust");
        assert!(bob.velocity_c(&metric) == 0.0, "and he is not moving in r while he waits");

        // Once he is released the report follows the worldline he is then on.
        let mut t = 8.0;
        while t < 8.5 {
            t += 0.05;
            bob.step(&metric, t, 0.05);
        }
        assert!(bob.is_active);
        assert_eq!(bob.four_velocity(&metric), bob.geodesic.unwrap().u);
        assert!(bob.velocity_c(&metric) < 0.0, "and he is falling");
    }

    #[test]
    fn test_a_mode_switch_changes_the_worldline_and_not_the_event() {
        // Switching mode is a statement about the future. The fixed-r modes leave the geodesic
        // state parked wherever it was last used, so resuming free fall from it would have
        // teleported the observer back to that event; `seed_geodesic_at_current_event` puts a
        // fresh geodesic under him where he now stands, with his own conserved (E, L).
        let metric = KerrSchild::new(1.0, 0.65);
        let mut bob = observer_at(&metric, ObserverMode::Static, 5.0);
        let mut t = 0.0;
        while t < 6.0 - 1e-12 {
            t += 0.1;
            bob.step(&metric, t, 0.1);
        }
        assert!((bob.r - 5.0).abs() < 1e-12 && (bob.t - 6.0).abs() < 1e-9, "he hovered");

        let event = (bob.t, bob.r, bob.phi, bob.tau);
        bob.mode = ObserverMode::FreeFall;
        assert_eq!(
            (bob.t, bob.r, bob.phi, bob.tau),
            event,
            "the switch alone must not move him"
        );
        bob.step(&metric, 6.1, 0.1);
        assert!((bob.t - 6.1).abs() < 1e-9, "he stays on the clock: t = {}", bob.t);
        assert!(bob.r < 5.0 && bob.r > 4.8, "and falls from r = 5, not from the seed: {}", bob.r);
        assert!(bob.tau > event.3, "his clock runs on rather than restarting: {}", bob.tau);

        // And back again: a radius where Static exists resumes the fixed-r worldline from the
        // event the fall has reached, with no jump.
        let landed = (bob.t, bob.r, bob.phi);
        bob.mode = ObserverMode::Static;
        bob.step(&metric, 6.2, 0.1);
        assert!((bob.r - landed.1).abs() < 1e-12, "the hover resumes at r = {}", bob.r);
        assert!((bob.phi - landed.2).abs() < 1e-12, "and at the azimuth he had reached");
    }

    #[test]
    fn test_a_free_faller_reads_zero_thrust_where_the_residual_check_cannot_follow() {
        // Bob with E = 1, L = 2 at a = 0.90 heads for the far branch of r-: u^t grows like
        // 1/(r - r-) on the way, and the finite difference behind `four_acceleration` is then a
        // derivative taken across a pole, which no stencil survives. It used to print thousands
        // of g of thrust on a worldline that has none. The accelerometer is zero for a geodesic
        // whatever the diagnostic says, all the way to the stall on r-; and the diagnostic is
        // seen to blow up on the same walk, so the two are known to part company here.
        let metric = KerrSchild::with_solar_mass(1.0, 0.90, 10.0);
        let mut bob = Observer::new_with_phi(
            &metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(1.0, 2.0, false),
        );
        let dt = 0.1;
        let mut t = 0.0;
        let mut residual_max = 0.0f64;
        let mut floor_at_max = 0.0f64;
        while t < 50.0 {
            t += dt;
            bob.step(&metric, t, dt);
            assert_eq!(bob.proper_acceleration_g(&metric), 0.0, "thrust at t={t}, r={}", bob.r);
            assert!(bob.is_free_falling(&metric), "weightless at t={t}, r={}", bob.r);
            let residual = bob.proper_acceleration_geom(&metric);
            if residual.is_finite() && residual > residual_max {
                residual_max = residual;
                floor_at_max = 1e-6 * metric.kretschmann_scalar(bob.r).sqrt();
            }
        }
        let gap = bob.r - metric.inner_horizon();
        println!(
            "Bob ends {gap:.2e} above r- at t = {t:.1}; the largest integrator residual on the \
             way was {residual_max:.3e}/M against a free-fall floor of {floor_at_max:.3e}/M"
        );
        assert!(gap < 1e-3, "the walk reached r-: r - r- = {gap}");
        assert!(
            residual_max > floor_at_max,
            "the diagnostic does blow up on this worldline: {residual_max} vs {floor_at_max}"
        );
    }

    #[test]
    fn test_is_free_falling_separates_geodesics_from_hovering() {
        // The telemetry label "(Free Fall)" is driven by this predicate, so it must not be fooled
        // by the numerical residual of a geodesic, nor call a hovering rocket weightless.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::with_solar_mass(1.0, a, 10.0);
            let rp = metric.outer_horizon();
            for &r in &[20.0, 6.0, 3.0, rp, 1.0, 0.2] {
                let obs = observer_at(&metric, ObserverMode::FreeFall, r);
                assert!(obs.is_free_falling(&metric), "geodesic at r={r} (a={a}) reads weightless");
                assert_eq!(obs.proper_acceleration_g(&metric), 0.0, "and shows no thrust");
                // The integrator residual agrees here, where the stencil is sound.
                let residual = obs.proper_acceleration_geom(&metric);
                assert!(
                    residual < 1e-6 * metric.kretschmann_scalar(r).sqrt(),
                    "residual {residual} at r={r} (a={a})"
                );
            }
            for &r in &[20.0, 6.0, 2.5] {
                let stat = observer_at(&metric, ObserverMode::Static, r);
                assert!(!stat.is_free_falling(&metric), "hovering at r={r} (a={a}) is not free fall");
                assert!(stat.proper_acceleration_g(&metric) > 1.0);
            }
        }
    }

    /// How far apart two azimuths are, the short way round. `GeodesicState` folds phi into
    /// [0, 2 pi), so a winding observer's angle wraps and a plain subtraction reads the wrap as a
    /// jump of a whole turn.
    fn angle_gap(a: f64, b: f64) -> f64 {
        let turn = std::f64::consts::TAU;
        let d = (a - b).rem_euclid(turn);
        d.min(turn - d)
    }

    #[test]
    fn test_a_frozen_worldline_winds_at_omega_minus_on_the_cauchy_horizon() {
        // A worldline that has frozen onto the far branch of r- has stopped falling, not stopped
        // moving. The surface it has settled onto is null, and its generators are the orbits of
        // chi = d_t + Omega_- d_phi, so an observer carried along one of them holds r and tau and
        // winds in phi at exactly Omega_- per unit t. Anything else would drift him across the
        // generators of a surface he can no longer cross.
        //
        // r and tau are compared for *exact* equality, because nothing is being integrated any
        // more: the gap r - r- is already below what a double can resolve when the stall is
        // declared and the proper time has reached its finite limit, so a step that moved either
        // by one ulp would be the integrator still running after it was stopped.
        let metric = KerrSchild::new(1.0, 0.90);
        let omega = metric.inner_horizon_omega();
        let mut bob = Observer::frozen_bob(&metric);
        let dt = 0.5;
        let wind = omega * dt;
        // Or the check could not tell Omega_- from standing still.
        let turns = wind / std::f64::consts::TAU;
        assert!(
            (turns - turns.round()).abs() * std::f64::consts::TAU > 1e-6,
            "a step of {dt} M winds {wind} rad, a whole number of turns"
        );

        let (r0, tau0, phi0, trail0) = (bob.r, bob.tau, bob.phi, bob.trail.len());
        println!(
            "frozen at t = {:.2} M, r - r- = {:.3e} M, tau = {tau0:.4} M, after {trail0} recorded \
             events; Omega_- = {omega:.6}, so {dt} M of coordinate time winds {wind:.6} rad",
            bob.t,
            r0 - metric.inner_horizon()
        );

        let mut t = bob.t;
        let mut times = Vec::new();
        let mut phis = Vec::new();
        for step in 1..=10 {
            t += dt;
            bob.step(&metric, t, dt);
            times.push(t);
            phis.push(bob.phi);
            assert_eq!(bob.r, r0, "r moved on step {step} of a frozen worldline");
            assert_eq!(bob.tau, tau0, "tau moved on step {step} of a frozen worldline");
            let want = phi0 + wind * f64::from(step);
            assert!(
                angle_gap(bob.phi, want) < 1e-12,
                "after {step} steps phi = {} and the generator is at {want}",
                bob.phi
            );
        }
        assert_eq!(bob.trail.len(), trail0 + 10, "each step is still a recorded event");

        // And the glide is reversible like any other stretch of worldline: wound back to the
        // third step and run forward again, the observer lands on the same azimuth.
        let landed = bob.phi;
        bob.rewind_to(&metric, times[2]);
        assert!(
            angle_gap(bob.phi, phis[2]) < 1e-12,
            "the rewind landed at phi = {} instead of {}",
            bob.phi,
            phis[2]
        );
        assert!(bob.is_frozen(), "and it landed inside the frozen segment");
        for &t in times.iter().skip(3) {
            bob.step(&metric, t, dt);
        }
        assert!(
            angle_gap(bob.phi, landed) < 1e-12,
            "re-run from the third step, phi = {} against {landed}",
            bob.phi
        );
    }

    /// An observer standing on the circular orbit at `r`, prograde or retrograde: the thrust-free
    /// orbit's own (E, L), with the geodesic state seeded at that radius.
    fn orbiter(metric: &KerrSchild, r: f64, prograde: bool) -> Observer {
        let (energy, l_ang) = metric.circular_orbit(r, prograde).expect("a circular orbit here");
        let params = WorldlineParams {
            energy,
            l_ang,
            outgoing: false,
            release: if prograde { Release::CircularPrograde } else { Release::CircularRetrograde },
        };
        Observer::new_with_phi(metric, "orbiter", 0.0, r, 0.0, 0.0, params)
    }

    #[test]
    fn test_a_circular_orbit_measures_0_5c_at_the_schwarzschild_isco() {
        // The textbook case, and the one number in this whole readout that can be checked against
        // a closed form by eye: at a = 0 the ISCO is at r = 6M and an orbiter passes the static
        // observer there at exactly v = sqrt(M / (r - 2M)) = 1/2, with gamma = 2/sqrt(3).
        let metric = KerrSchild::new(1.0, 0.0);
        let orbiter = orbiter(&metric, 6.0, true);
        let stat = orbiter.local_speed(&metric, LocalRestFrame::Static).expect("static at 6M");
        assert!((stat.v - 0.5).abs() < 1e-9, "v = {} against an exact 0.5", stat.v);
        assert!(
            (stat.gamma - 2.0 / 3.0_f64.sqrt()).abs() < 1e-9,
            "gamma = {} against an exact 2/sqrt(3)",
            stat.gamma
        );
        assert!((stat.celerity() - 0.5 * 2.0 / 3.0_f64.sqrt()).abs() < 1e-9);

        // With no spin there is no dragging, so the ZAMO *is* the static observer and the two
        // readings coincide. That is the degenerate case of the pair the box prints.
        let zamo = orbiter.local_speed(&metric, LocalRestFrame::Zamo).expect("ZAMO at 6M");
        assert!((zamo.v - stat.v).abs() < 1e-9, "no spin, no disagreement: {} vs {}", zamo.v, stat.v);
    }

    #[test]
    fn test_the_two_hovering_frames_disagree_by_the_dragging() {
        // At a = 0.90 the prograde ISCO is at r = 2.32M, a whisker outside the static limit at 2M,
        // and the two hovering observers give very different answers for the same orbit: the ZAMO
        // is being carried round at half the orbiter's own Omega, so it sees much less of the
        // motion than the static observer does. Both numbers are right; the gap between them is
        // the frame dragging, which is why the box prints the pair.
        let metric = KerrSchild::new(1.0, 0.9);
        let r = metric.isco(true);
        assert!((r - 2.321).abs() < 1e-3, "the prograde ISCO at a = 0.90: r = {r}");
        let orbiter = orbiter(&metric, r, true);
        let stat = orbiter.local_speed(&metric, LocalRestFrame::Static).expect("just outside 2M");
        let zamo = orbiter.local_speed(&metric, LocalRestFrame::Zamo).expect("well outside r+");
        assert!((stat.v - 0.898).abs() < 1e-3, "v against the static observer: {}", stat.v);
        assert!((zamo.v - 0.625).abs() < 1e-3, "v against the ZAMO: {}", zamo.v);
        assert!(zamo.v < stat.v, "going along with the dragging always sees less of the motion");

        // The orbiter's own Omega against the rate the ZAMO is dragged at, which is the reason for
        // the gap and the second number on the box's Omega line.
        let omega_orbit = orbiter.angular_velocity(&metric);
        let omega_drag = metric.frame_dragging_omega(r);
        assert!((omega_orbit - 0.2254).abs() < 1e-3, "Omega = {omega_orbit}");
        assert!((omega_drag - 0.1125).abs() < 1e-3, "dragging rate = {omega_drag}");

        // Whatever the frame, a timelike worldline measured by a timelike worldline is below c.
        for s in orbiter.local_speeds(&metric) {
            assert!(s.v < 1.0, "{:?} reads {}c", s.frame, s.v);
            assert!(s.gamma >= 1.0);
        }
    }

    #[test]
    fn test_a_raindrop_passes_a_static_observer_at_the_newtonian_escape_speed() {
        // The other closed form: the E = 1, L = 0 infall passes the static observer at r with
        // v = sqrt(2M/r), the Newtonian escape speed, exactly - a = 0 here, since with spin the
        // raindrop picks up a dragged u^phi and the coincidence is not expected to survive it.
        let metric = KerrSchild::new(1.0, 0.0);
        for r in [8.0, 4.5, 3.0, 2.5] {
            let drop =
                Observer::new_with_phi(&metric, "drop", 0.0, r, 0.0, 0.0, WorldlineParams::default());
            let stat = drop.local_speed(&metric, LocalRestFrame::Static).expect("outside 2M");
            let expect = (2.0 / r).sqrt();
            assert!(
                (stat.v - expect).abs() < 1e-9,
                "at r = {r}: v = {} against sqrt(2M/r) = {expect}",
                stat.v
            );
        }
    }

    #[test]
    fn test_a_frame_is_never_invented_where_its_observer_cannot_exist() {
        // The readout may not quote a speed against a worldline that is not there. The static
        // observer runs out at the equatorial static limit r = 2M, the ZAMO at r+, and inside r+
        // the pair falls back to the raindrop, which is the one frame that exists at every r > 0.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, re) = (metric.outer_horizon(), metric.ergosphere_equatorial());
        assert!(rp < re, "r+ = {rp} inside the static limit {re}");
        let at = |r: f64| {
            Observer::new_with_phi(&metric, "o", 0.0, r, 0.0, 0.0, WorldlineParams::default())
        };

        let outside = at(3.0);
        assert_eq!(outside.local_speeds(&metric).len(), 2, "both hovering frames outside 2M");
        assert!(outside.local_speeds(&metric).iter().all(|s| s.frame != LocalRestFrame::Raindrop));

        // In the ergosphere no rocket can hold phi fixed, so there is no static observer to be
        // measured against and the ZAMO is the only hovering frame left.
        let ergo = at(0.5 * (rp + re));
        let speeds = ergo.local_speeds(&metric);
        assert_eq!(speeds.len(), 1, "one frame inside the static limit");
        assert_eq!(speeds[0].frame, LocalRestFrame::Zamo);
        assert!(ergo.local_speed(&metric, LocalRestFrame::Static).is_none());

        // Inside r+ nothing can hold a radius at all.
        let inside = at(0.5 * (metric.inner_horizon() + rp));
        let speeds = inside.local_speeds(&metric);
        assert_eq!(speeds.len(), 1, "no hovering observer between the horizons");
        assert_eq!(speeds[0].frame, LocalRestFrame::Raindrop);
        assert!(speeds[0].v < 1.0, "and even there the measured speed is below c: {}", speeds[0].v);
        assert!(inside.local_speed(&metric, LocalRestFrame::Zamo).is_none());
    }

    #[test]
    fn test_the_frame_measured_against_itself_reads_zero() {
        // The degenerate case the clamp in `local_speed` is there for: gamma = -g(u, u) = 1 for a
        // unit timelike vector, to within a few ulp either side, and 1 - eps must not come back as
        // a NaN speed.
        let metric = KerrSchild::new(1.0, 0.9);
        for (mode, r) in [(ObserverMode::Static, 3.0), (ObserverMode::Zamo, 1.8)] {
            let mut obs =
                Observer::new_with_phi(&metric, "o", 0.0, r, 0.0, 0.0, WorldlineParams::default());
            obs.mode = mode;
            let frame = if mode == ObserverMode::Static {
                LocalRestFrame::Static
            } else {
                LocalRestFrame::Zamo
            };
            let s = obs.local_speed(&metric, frame).expect("the frame exists at this radius");
            assert!(s.v.is_finite() && s.v < 1e-7, "{:?} against itself: v = {}", mode, s.v);
            assert!((s.gamma - 1.0).abs() < 1e-7, "and gamma = {}", s.gamma);
        }
    }
}
