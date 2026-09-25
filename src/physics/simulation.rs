//! The run, as one object: the geometry, the two worldlines, the two transmissions and the clock
//! they are all carried on.
//!
//! It exists for the reason `ObserverPair` and `SignalPair` exist, one level up. Those two say
//! once what a step does to *both* worldlines and to *both* transmissions; this says once what a
//! step does to the simulation, which is the two of them in a fixed order against a single clock.
//! Every path that moves the run - a played frame, an arrow key, the panel's transport buttons, a
//! scripted replay in `crate::perf` - goes through the methods below, so none of them can mean a
//! different thing by "one step" than any other.
//!
//! Holding it as one object is also what makes a run something that can be handed about whole: a
//! state can be fingerprinted (`fingerprint`), checked (`check_invariants`), and - which is what
//! this was pulled together for - saved and put back by assignment, without six fields having to be
//! kept in step by hand at the call site.
//!
//! Nothing here knows about the user interface. The panel's settings are not part of the run: what
//! a step needs to be told about them is passed in as `Transmit`, and the geometry, the observers,
//! the light and the clock are all that is kept.

use crate::physics::geodesic::R_STOP;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverPair};
use crate::physics::wavefront::{Endpoint, SignalField, SignalPair};

/// What the panel has to say about the step that is about to be taken: how finely the next
/// emission samples the emitter's light cone, and which of the two is broadcasting at all.
///
/// It is passed in rather than kept, because none of it is a property of the run. The two flags
/// are the "Transmit Signal" boxes, which a user can tick between any two steps, and the ray count
/// is a standing request about the *next* emission - see `Simulation::step_forward` for why it is
/// pushed in on the way into a step rather than at the click that moved the slider.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Transmit {
    pub rays_per_pulse: usize,
    /// Whether Alice is broadcasting. Her transmission is the one Bob receives.
    pub alice: bool,
    /// Whether Bob is broadcasting, which is the same idea run the other way round.
    pub bob: bool,
}

/// Shortest sub-step `Simulation::step_forward` will cut, in M of coordinate time.
///
/// It is a termination floor and not a resolution: it is there so that two emission events a
/// rounding apart cannot be split into a sub-step of 1e-17 M that advances nothing and leaves the
/// loop asking the same question for ever. At 1e-12 M it is four orders below the smallest interval
/// any ray or worldline integrator in the app resolves - `GeodesicState::step_coord_time` floors its
/// own substeps at 1e-7 M - so a sub-step that hits it moves nothing anybody can measure and the
/// next one carries on from where it stood.
const MIN_SUB_STEP: f64 = 1e-12;

/// Hard ceiling on sub-steps per call, for the same reason `GeodesicState::step_coord_time` has one:
/// a state the arithmetic cannot get through costs one bounded call rather than hanging the frame.
///
/// It is not a bound on the work a long step is allowed to do. A step is split at every pulse due
/// inside it, so a Distance-mode step of hundreds of M legitimately takes thousands of sub-steps and
/// is meant to; at the fastest cadence the panel allows that is still far short of this. If it ever
/// binds, the call returns having covered less than the whole `dt` and the clock stops where it got
/// to, exactly as a worldline that exhausts its own substep budget stops where it got to.
const MAX_SUB_STEPS: usize = 100_000;

/// Everything the run is.
///
/// `Clone` because a whole run is a value: `crate::save` builds one off a file beside the one the
/// app is running, and a test that asks whether a saved run continues identically needs the same
/// state in two places at once. Nothing in the app clones one per frame - the trails and the rays
/// make it an expensive copy - and nothing should.
#[derive(Clone)]
pub struct Simulation {
    pub metric: KerrSchild,
    /// The two observers, either of whom may be out of the simulation: the "Enable Observer" box
    /// on a card unticked means there is no worldline at all rather than a hidden one, and the two
    /// are optional in the same way because the cards are the same card twice.
    pub alice: Option<Observer>,
    pub bob: Option<Observer>,
    /// Alice's signal pulses, which Bob receives.
    pub alice_signal: SignalField,
    /// Bob's transmission, which Alice receives. It is the same object driven the other way round,
    /// and the two are advanced, rewound and cleared together through `SignalPair`.
    pub bob_signal: SignalField,
    /// The simulation clock: the coordinate time of the ingoing Kerr-Schild chart everything here
    /// is integrated against, and the one number both worldlines and both fields are kept on.
    pub clock: f64,
}

impl Simulation {
    /// An empty run on a given geometry: the clock at zero, nobody in the simulation and nothing
    /// in flight. The observers are put in by `restart`, which is what the panel's cards drive.
    pub fn new(metric: KerrSchild) -> Self {
        Self {
            metric,
            alice: None,
            bob: None,
            alice_signal: SignalField::default(),
            bob_signal: SignalField::default(),
            clock: 0.0,
        }
    }

    /// One step forward, cut at the events where the emitters' own watches say a pulse is due.
    ///
    /// The step the caller asked for is covered by a run of *sub-steps*, each one the whole step
    /// below, and each one ending either at the end of `dt` or exactly on the event where an
    /// emitter's proper time reads the value its next pulse is due at. `SignalPair` says when that
    /// is and `Observer::time_until_tau` says how far away it is; nothing here decides anything but
    /// the lengths.
    ///
    /// **Why.** An observer transmits every `EMISSION_INTERVAL_TAU` of *their own* proper time, so
    /// where a pulse leaves is an event on their worldline and on nothing else. Emitting at the end
    /// of whatever step the caller happened to take made it an event on the user's step grid
    /// instead: the pulse went out at the first step boundary at or past the due reading, the
    /// cadence then counted from that boundary rather than from the due value so the comb drifted,
    /// and a step longer than the interval - which is most of them, one press reaching 20 M at the
    /// fastest play rate and a played frame 0.33 M against an interval of 0.1 M - sent one pulse
    /// where several were due. Measured over the same 6 M of the default layout, a frame step put
    /// 22 pulses on the wire and a 0.5 M step 12. The physics cannot depend on how finely the user
    /// asked to watch it.
    ///
    /// Nothing is done about very long steps, deliberately: the loop splits at every pulse due
    /// inside one, which is exactly what makes a long step exact, and a Distance-mode step of
    /// hundreds of M is simply slow. The floor and the guard below are there so that a degenerate
    /// length cannot spin, and not to bound the work.
    ///
    /// The ray count is pushed into both fields here, on the way into the step, rather than at the
    /// click that moved the slider: that way a played frame, an arrow key, a panel button and a
    /// test that steps by hand all emit at the count the panel is currently showing, and the two
    /// transmissions cannot end up sampled differently. It reaches the emission and nothing else -
    /// pulses already in flight keep their own count.
    pub fn step_forward(&mut self, dt: f64, tx: Transmit) {
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }
            .set_rays_per_pulse(tx.rays_per_pulse);
        let mut remaining = dt.max(0.0);
        let mut guard = 0;
        // A step of nothing is still a step - `AppControls::set_spin` takes one to settle a run at
        // t = 0 - so the body runs once whatever `dt` is and the loop is what repeats. Each pass
        // sends whatever is already due, then carries the run as far as the next thing that falls
        // due, so an emission is always the *end* of a sub-step or the start of one and never
        // something that happened in the middle of a piece of integration.
        loop {
            guard += 1;
            self.emit_due_now(tx);
            let sub = self.sub_step_length(remaining, tx);
            self.sub_step(sub, tx);
            remaining -= sub;
            if remaining <= 0.0 || guard >= MAX_SUB_STEPS {
                break;
            }
        }
        self.debug_check();
    }

    /// Send the pulses that are due at the event the next sub-step starts from. See
    /// `SignalPair::emit_due_now`: the first pulse of a transmission is one of them, and so is a
    /// pulse the sub-step before landed within `DUE_TOLERANCE` of.
    fn emit_due_now(&mut self, tx: Transmit) {
        let alice = Endpoint { observer: self.alice.as_ref(), transmitting: tx.alice };
        let bob = Endpoint { observer: self.bob.as_ref(), transmitting: tx.bob };
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }
            .emit_due_now(&self.metric, alice, bob);
    }

    /// How long the next sub-step may be: up to the end of what is left of the step, and no further
    /// than the nearer of the two emitters' next due emission events.
    ///
    /// The floor is what makes the loop terminate. Every iteration either emits - which moves that
    /// emitter's due value on by a whole interval - or advances the clock by something strictly
    /// positive, and a due value already inside `DUE_TOLERANCE` of the emitter's present reading has
    /// been sent by `emit_due_now` before this is asked, so what is left is at least a tolerance of
    /// proper time away. The floor catches the case where that is still a hair of coordinate time,
    /// deep in the strong field where dt/dtau runs large; a sub-step of 1e-12 M is below anything
    /// the physics resolves and the next one carries on from it.
    fn sub_step_length(&mut self, remaining: f64, tx: Transmit) -> f64 {
        if remaining <= 0.0 {
            return 0.0;
        }
        let alice = Endpoint { observer: self.alice.as_ref(), transmitting: tx.alice };
        let bob = Endpoint { observer: self.bob.as_ref(), transmitting: tx.bob };
        let span = SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }
            .time_to_next_emission(&self.metric, self.clock, remaining, alice, bob);
        span.max(MIN_SUB_STEP).min(remaining)
    }

    /// One sub-step: the clock, then both worldlines, then both transmissions.
    ///
    /// This is the only description of a step forward there is; `step_forward` chooses the lengths
    /// and this says what a step of one *is*. The order is the whole of it. The clock moves first
    /// because both of the calls below are told where it now stands rather than how far it moved -
    /// `ObserverPair::step` lands the worldlines *on* the clock, which is what keeps a released
    /// free-faller from running a release's worth of time ahead of it for the rest of the run. The
    /// worldlines move before the light because a pulse emitted on this step is emitted at its
    /// emitter's new event: `SignalPair::advance` then owns the rest of the order - carry the light,
    /// then emit, then listen.
    fn sub_step(&mut self, dt: f64, tx: Transmit) {
        self.clock += dt;
        ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
            .step(&self.metric, self.clock, dt);
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }.advance(
            &self.metric,
            dt,
            Endpoint { observer: self.alice.as_ref(), transmitting: tx.alice },
            Endpoint { observer: self.bob.as_ref(), transmitting: tx.bob },
        );
    }

    /// The earliest coordinate time the clock can still be wound back to.
    ///
    /// Zero until a run is long enough for a worldline to evict the start of its own trail, and
    /// that trail's oldest event afterwards: past it there is no recorded event to put the observer
    /// back on and no honest way to invent one. See `ObserverPair::rewind_floor`, which takes the
    /// later of the two observers' floors, because it is the *clock* that gets clamped and both of
    /// them have to stay on it.
    ///
    /// It takes `&mut self` because `ObserverPair` is built from mutable borrows; it changes
    /// nothing.
    pub fn rewind_floor(&mut self) -> f64 {
        ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
            .rewind_floor(&self.metric)
    }

    /// How much of the run is left to step back through: the clock above its floor. What the
    /// panel's Step Back button is enabled by, and what a step back is clamped to.
    pub fn rewind_room(&mut self) -> f64 {
        self.clock - self.rewind_floor()
    }

    /// One step back, undoing a step forward rather than approximating one, and returning how far
    /// the clock actually went - which is `step` unless the floor was in the way, and 0.0 if there
    /// was no room at all.
    ///
    /// The clock stops at t = 0, and again at `rewind_floor`. Whatever is wound back is wound back
    /// by however much of the step is left above that floor, which is what keeps the fields' clock
    /// equal to the simulation clock and both worldlines on both.
    ///
    /// The worldlines go first and the fields second. A worldline needs nothing from a field to be
    /// wound back, and a field needs its receiver at the rewound state to re-establish which side
    /// of each wavefront they stand on - `SignalPair::step_back` does that priming, without which a
    /// crossing that happens inside the next step forward is never seen.
    ///
    /// The observers are given the *target time* rather than the interval, which is what keeps them
    /// locked to the clock when the two differ - the clock stops at zero, a Distance-mode step can
    /// be hundreds of M, and a worldline that has already ended has no interval left to undo. The
    /// fields are given the interval, which is what `SignalField::step_back` integrates its rays
    /// back over.
    pub fn step_back(&mut self, step: f64) -> f64 {
        let back = step.min(self.rewind_room()).max(0.0);
        if back <= 0.0 {
            return 0.0;
        }
        self.clock -= back;
        ObserverPair { bob: self.bob.as_mut(), alice: self.alice.as_mut() }
            .rewind_to(&self.metric, self.clock);
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }.step_back(
            &self.metric,
            back,
            self.alice.as_ref(),
            self.bob.as_ref(),
        );
        self.debug_check();
        back
    }

    /// How many wavefronts each transmission keeps at once, trimming both to it at once.
    ///
    /// Unlike the ray count, which rides in with a step, this is a statement about what is on the
    /// screen rather than a request about the next emission, so the app pushes it in once a frame,
    /// played or paused. That is what makes lowering the slider bite while the run is paused. See
    /// `SignalField::max_pulses`: lowering it is not reversible, an evicted pulse being gone.
    pub fn set_max_pulses(&mut self, pulses: usize) {
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }
            .set_max_pulses(&self.metric, pulses);
    }

    /// Start the run again with these two observers: the clock back to zero, both transmissions
    /// dropped, and the given worldlines in place of whoever was there.
    ///
    /// The transmissions are cleared rather than rewound, and that is the difference between this
    /// and `step_back`. A rewind keeps the light in flight, and here there is none to keep: the
    /// wavefronts standing in the field were emitted by worldlines that this is replacing, and
    /// where the geometry itself has just changed under them they are null geodesics of a metric
    /// that no longer applies.
    ///
    /// Who the two observers are is not decided here. They are built by the caller - from the
    /// panel's cards, which is the only place that knows what a fresh Alice and Bob are - and this
    /// is handed the result.
    pub fn restart(&mut self, alice: Option<Observer>, bob: Option<Observer>) {
        self.clock = 0.0;
        SignalPair { alice: &mut self.alice_signal, bob: &mut self.bob_signal }.clear();
        self.alice = alice;
        self.bob = bob;
        self.debug_check();
    }

    /// Whether the hole itself may still be changed, which is true only while the run stands at
    /// its start.
    ///
    /// A run is a run of one geometry. Every ray in flight is a null geodesic of this metric, and
    /// each observer carries a four-velocity, an energy E and an angular momentum L that only this
    /// mass and this spin normalise, so a new hole under a run in progress leaves all of that light
    /// and both of those worldlines solving the old hole's equations in a new hole's field. That is
    /// the same reason `restart` above clears the light rather than keeping it. The panel greys the
    /// Mass and Spin sliders out wherever this reads false, and at t = 0 a spin change goes through
    /// `AppControls::set_spin`, which starts the run again in the new hole.
    ///
    /// The clock is compared with zero exactly, as `AppControls::remember_drop_positions` compares
    /// it: `restart` assigns 0.0, and `step_back` subtracts from the clock the room the clock has
    /// above its floor, so a run wound back to its start reads exactly zero rather than nearly
    /// zero.
    pub fn may_change_geometry(&self) -> bool {
        self.clock == 0.0
    }

    /// `check_invariants` as an assertion, in the builds that carry them.
    ///
    /// The check itself sits inside the `debug_assert!`'s condition, so a release build never runs
    /// it at all - which is the point, since it walks every ray of both transmissions. The second
    /// call is what puts the broken statement into the panic message, and it is reached only by a
    /// state that is about to bring the run down anyway.
    fn debug_check(&self) {
        debug_assert!(
            self.check_invariants().is_ok(),
            "the simulation reached a state it says is unreachable: {}",
            self.check_invariants().unwrap_err()
        );
    }

    /// The 64-bit state hash of a run: the clock, both worldlines, every ray of both transmissions
    /// and every arrival recorded.
    ///
    /// Every number here is one the physics produced. Nothing about the drawing is in it - no
    /// camera, no zoom, no canvas state - so a change to a painter cannot move the fingerprint and
    /// a change to an integrator cannot fail to. `crate::perf` prints it at the end of every replay
    /// and `--compare` says, for each scenario, whether an optimisation that claimed to change only
    /// the speed left it alone.
    pub fn fingerprint(&self) -> u64 {
        let mut h = Fnv::new();
        h.f64(self.clock);
        for observer in [self.alice.as_ref(), self.bob.as_ref()] {
            match observer {
                Some(obs) => {
                    h.bits(1);
                    h.f64(obs.t);
                    h.f64(obs.r);
                    h.f64(obs.phi);
                    h.f64(obs.tau);
                }
                None => h.bits(0),
            }
        }
        for field in [&self.alice_signal, &self.bob_signal] {
            h.f64(field.t);
            h.bits(field.pulses.len() as u64);
            for pulse in field.pulses.iter() {
                h.bits(pulse.index as u64);
                for ray in pulse.rays.iter() {
                    h.f64(ray.r);
                    h.f64(ray.phi);
                    h.f64(ray.dr_dt);
                    h.f64(ray.dphi_dt);
                    h.maybe(ray.death_t);
                }
            }
            for reception in field.receptions() {
                h.f64(reception.t);
                h.f64(reception.r);
                h.f64(reception.ratio);
            }
        }
        h.0
    }

    /// Everything that is true of every state this simulation can reach, stated so that a state
    /// built by something other than the methods above - a loader, reading a run back off a file -
    /// can be checked before it is let anywhere near a step.
    ///
    /// It is deliberately a list of things that *hold*, not a list of things that look sensible.
    /// Each of these was measured over the whole test suite and the four `crate::perf` scenarios,
    /// which between them cover an infall onto the ring, a pair transmitting at both pulse caps,
    /// the freeze onto the far branch of r₋, hand steps forwards and backwards, and the cards being
    /// ticked on and off mid-run. Three candidates that did not survive that are named at the
    /// bottom.
    ///
    /// **The clocks agree to 1e-9 rather than exactly.** A field's clock and the simulation clock
    /// are advanced by the same `dt` in the same call, so they would be equal to the bit were that
    /// all that happened to them. Two things spoil it. A pulse is born carrying its *emitter's*
    /// clock rather than the field's, and a worldline released part-way through a step lands on
    /// `release_t + (clock - release_t)`, which is the clock only to within a rounding; and
    /// `SignalField::step_back` takes its target from the latest ray in the field rather than from
    /// the field's own clock, so a rounding once made is carried through every rewind after it.
    /// The gap is a rounding of the clock's own magnitude and does not accumulate beyond that:
    /// measured over the four `crate::perf` scenarios it is 0 for the two ISCO pairs, 7e-15 over
    /// the 40 M of `default-infall`, and 8e-13 over the 100 M of `far-branch-freeze`, for both the
    /// field against the clock and every ray against its field. The tolerance is therefore the one
    /// the app's own tests use for "the same event", 1e-9, which leaves three orders of headroom
    /// over the longest run measured rather than a bound that would have to be rediscovered every
    /// time the arithmetic moves.
    ///
    /// Called under `debug_assert!` at the end of `step_forward`, `step_back` and `restart`, which
    /// puts it behind the whole test suite (the test profile has debug assertions on) and out of
    /// the release build entirely.
    ///
    /// Three candidates are **not** here, because they are not true:
    ///
    /// * *An observer's clock reads the simulation clock.* A worldline that has reached the ring
    ///   stops being advanced at all - `Observer::advance` moves nothing below `R_STOP` - while the
    ///   simulation clock runs on, so a run played past an infall has an observer tens of M behind
    ///   it: 33.6 M of the 40 M `default-infall` plays. A worldline that freezes onto r₋ falls
    ///   behind by the unspent remainder of the step the stall was declared on, 3.1e-3 M in
    ///   `far-branch-freeze`, and keeps that gap for the rest of the run. `is_active` separates
    ///   neither case out: it is set on the first released step and stays set.
    /// * *A worldline never runs ahead of the clock.* True to a rounding, and only to a rounding:
    ///   the release step above can land a free-faller an ulp past the clock. Stated with a
    ///   tolerance it says nothing the clock agreement above does not.
    /// * *A field holds at least one pulse while its emitter is transmitting.* False for a whole
    ///   emission interval at the start of a run, false for ever for an emitter who is inside
    ///   `R_STOP` or stalled, and false for an emitter hovering at or inside the static limit, who
    ///   has no worldline to broadcast from.
    pub fn check_invariants(&self) -> Result<(), String> {
        /// What the app's own tests call the same event. See the doc comment above.
        const SAME_EVENT: f64 = 1e-9;

        if !self.clock.is_finite() {
            return Err(format!("the simulation clock is {}", self.clock));
        }
        for (name, observer) in [("Alice", self.alice.as_ref()), ("Bob", self.bob.as_ref())] {
            let Some(obs) = observer else { continue };
            for (what, value) in
                [("t", obs.t), ("r", obs.r), ("phi", obs.phi), ("tau", obs.tau)]
            {
                if !value.is_finite() {
                    return Err(format!("{name}'s {what} is {value}"));
                }
            }
        }
        for (name, field) in
            [("Alice's", &self.alice_signal), ("Bob's", &self.bob_signal)]
        {
            if !field.t.is_finite() {
                return Err(format!("{name} transmission is dated {}", field.t));
            }
            // The field rides the simulation clock: it is advanced by the same step in the same
            // call and wound back to the same target, so a field that has come adrift of the clock
            // is a field whose light is being carried against a different time from the worldlines
            // it is being measured against.
            if (field.t - self.clock).abs() > SAME_EVENT {
                return Err(format!(
                    "{name} transmission is at t = {} and the clock reads {}",
                    field.t, self.clock
                ));
            }
            // A cap of zero is not reachable from the panel and would leave a transmitting emitter
            // with nothing in flight at all, so `SignalField::trim_to_cap` floors it at one; the
            // invariant is stated against the same floor it trims to.
            if field.pulses.len() > field.max_pulses.max(1) {
                return Err(format!(
                    "{name} transmission holds {} pulses at a cap of {}",
                    field.pulses.len(),
                    field.max_pulses
                ));
            }
            let mut previous: Option<usize> = None;
            for pulse in field.pulses.iter() {
                // Emission order, and no reuse: `step_back` un-sends the pulses emitted inside the
                // interval but never rewinds the serial number, so a re-emitted pulse is a new
                // pulse and the indices of a field are strictly increasing and below the next one
                // to be handed out.
                if previous.is_some_and(|last| pulse.index <= last) {
                    return Err(format!(
                        "{name} transmission holds pulse {} after pulse {}",
                        pulse.index,
                        previous.unwrap_or_default()
                    ));
                }
                if pulse.index >= field.next_index {
                    return Err(format!(
                        "{name} transmission holds pulse {} and hands out {} next",
                        pulse.index, field.next_index
                    ));
                }
                previous = Some(pulse.index);
                // A role names a ray of its own pulse by index, and the canvas reads that ray's
                // radius every frame: an index past the end is a file that has been edited, not a
                // state the emission can produce.
                let n = pulse.rays.len();
                if let Some(role) = pulse.role_rays.iter().flatten().find(|i| **i >= n) {
                    return Err(format!(
                        "{name} pulse {} follows ray {role} of {}",
                        pulse.index,
                        pulse.rays.len()
                    ));
                }
                for ray in pulse.rays.iter() {
                    for (what, value) in [
                        ("t", ray.t),
                        ("r", ray.r),
                        ("phi", ray.phi),
                        ("dr/dt", ray.dr_dt),
                        ("dphi/dt", ray.dphi_dt),
                    ] {
                        if !value.is_finite() {
                            return Err(format!(
                                "a ray of {name} pulse {} has {what} = {value}",
                                pulse.index
                            ));
                        }
                    }
                    // Dead or alive, a ray stands on a state the integrator committed, and
                    // `ray_dopri5` refuses to build a stage below the ring: nothing in the field is
                    // ever left at a radius the equation does not hold at.
                    if ray.r < R_STOP {
                        return Err(format!(
                            "a ray of {name} pulse {} stands at r = {}, inside the ring at {R_STOP}",
                            pulse.index, ray.r
                        ));
                    }
                    // Every ray of the field is carried on the field's clock, the dead ones
                    // included: that is what lets `step_back` ask each of them whether it was still
                    // alive dt ago. A ray born on its emitter's clock inherits that clock's
                    // rounding, hence the tolerance.
                    if (ray.t - field.t).abs() > SAME_EVENT {
                        return Err(format!(
                            "a ray of {name} pulse {} is dated {} in a field at t = {}",
                            pulse.index, ray.t, field.t
                        ));
                    }
                    // Both are set by `NullRay::die` and cleared by `NullRay::revive`, and by
                    // nothing else, so a state where one is present without the other is a state
                    // nothing in the physics can produce.
                    if ray.death_t.is_some() != ray.death_end.is_some() {
                        return Err(format!(
                            "a ray of {name} pulse {} died at {:?} at boundary {:?}",
                            pulse.index, ray.death_t, ray.death_end
                        ));
                    }
                    if ray.death_t.is_some_and(|t| !t.is_finite()) {
                        return Err(format!(
                            "a ray of {name} pulse {} died at {:?}",
                            pulse.index, ray.death_t
                        ));
                    }
                }
            }
        }
        Ok(())
    }
}

/// A 64-bit FNV-1a over the bit patterns of whatever it is fed.
///
/// Hand-written because the crate takes no dependency it does not need, and FNV because the job is
/// to notice that two states differ, not to resist anybody trying to make them collide. The bits go
/// in as bits rather than as rounded decimals: the claim a fingerprint is checking is that an
/// optimisation left the arithmetic alone, and an optimisation that moved the last bit of a radius
/// has not left the arithmetic alone.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    fn bits(&mut self, value: u64) {
        for byte in value.to_le_bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    fn f64(&mut self, value: f64) {
        self.bits(value.to_bits());
    }

    /// An optional number goes in as a present/absent tag and then, if present, its bits, so that
    /// a ray that has just died and one that died a moment ago cannot hash the same.
    fn maybe(&mut self, value: Option<f64>) {
        match value {
            Some(v) => {
                self.bits(1);
                self.f64(v);
            }
            None => self.bits(0),
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::wavefront::RayEnd;

    /// A short run of the real thing: two raindrops, both transmitting, stepped by hand at the
    /// size a played frame takes and played long enough for the first wavefronts to have reached
    /// the ring. Every state the checks below are made against is one the physics produced rather
    /// than one assembled by the test.
    fn stepped_run() -> Simulation {
        let metric = KerrSchild::new(1.0, 0.9);
        let mut sim = Simulation::new(metric);
        sim.restart(
            Some(Observer::new(&metric, "Alice", 0.0, 6.0, 0.0)),
            Some(Observer::new(&metric, "Bob", 0.0, 4.5, 0.0)),
        );
        for _ in 0..200 {
            sim.step_forward(0.05, Transmit { rays_per_pulse: 64, alice: true, bob: true });
        }
        sim
    }

    /// The complaint `check_invariants` makes about a run that has been broken in one stated way.
    fn complaint(break_it: impl FnOnce(&mut Simulation)) -> String {
        let mut sim = stepped_run();
        break_it(&mut sim);
        sim.check_invariants()
            .expect_err("this state is one the physics cannot reach and must be refused")
    }

    #[test]
    fn test_check_invariants_refuses_states_the_physics_cannot_reach() {
        // First that it passes what it is meant to pass, and on something worth passing: a run
        // with light in flight, arrivals recorded and rays that have already reached the ring. A
        // check that held vacuously would pass every one of the broken states below as well.
        let sim = stepped_run();
        sim.check_invariants().expect("a run built by stepping satisfies its own invariants");
        assert!(sim.alice_signal.pulses.len() > 1, "the run has more than one wavefront in flight");
        assert!(
            sim.alice_signal.pulses.iter().flat_map(|p| p.rays.iter()).any(|ray| !ray.alive()),
            "and rays that have already left the field, so the death checks have something to say"
        );

        // A loader reading a state back has to be told about each of these, because each of them
        // is a state the app would go on to draw and integrate as though it meant something.
        for (what, broken) in [
            ("simulation clock is NaN", complaint(|sim| sim.clock = f64::NAN)),
            ("Alice's r is inf", complaint(|sim| sim.alice.as_mut().unwrap().r = f64::INFINITY)),
            ("and the clock reads", complaint(|sim| sim.clock += 1.0)),
            ("dated", complaint(|sim| sim.alice_signal.pulses[0].rays[0].t += 1e-3)),
            (
                "inside the ring",
                complaint(|sim| sim.alice_signal.pulses[0].rays[0].r = R_STOP * 0.5),
            ),
            (
                "died at",
                complaint(|sim| sim.alice_signal.pulses[0].rays[0].death_end = Some(RayEnd::Ring)),
            ),
            (
                "after pulse",
                complaint(|sim| {
                    let last = sim.alice_signal.pulses.len() - 1;
                    sim.alice_signal.pulses.swap(0, last);
                }),
            ),
            (
                "hands out",
                complaint(|sim| sim.alice_signal.next_index = 0),
            ),
            ("at a cap of", complaint(|sim| sim.alice_signal.max_pulses = 1)),
        ] {
            assert!(
                broken.contains(what),
                "the refusal should say what is wrong with it, and says {broken:?} instead"
            );
        }
    }
}
