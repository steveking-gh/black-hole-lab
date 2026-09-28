//! The observer's worldline, forward from the saved moment, as a sequence of events at equal steps
//! of the observer's own proper time.
//!
//! This is a second statement of how the app moves an observer (`Observer::step` and `advance` in
//! the app's `src/physics/observer.rs`), written on the same geometry core so that the two agree,
//! and held to the app by the tests in `tests.rs`, which walk from events of a trail the app
//! recorded and land on the later ones. It cannot call the app's code: that lives in the app's
//! binary crate.
//!
//! # What the app does, and what is done here
//!
//! An observer is in one of three kinds of motion, and may pass from the first into one of the
//! others once:
//!
//! * **Holding**, before the release. The app's `hover` keeps r fixed and moves the observer along
//!   the fixed-r worldline of `hover_four_velocity`: the worldline the observer is about to fall on
//!   where that is at rest in r (a release at rest, or onto a circular orbit), and the static
//!   observer otherwise. u is a constant of that worldline, so t and phi are linear in tau,
//!   t = t0 + u^t (tau - tau0) and phi = phi0 + u^phi (tau - tau0). The app accumulates the same
//!   thing frame by frame, dtau = dt / u^t; here it is the closed form.
//! * **Free fall**: the geodesic, integrated by the core's `step_coord_time_until_tau`, the
//!   integrator the app's `advance` uses, stopped on the frame's own proper time. The frame is a
//!   point *on* the integrated curve - the core re-integrates the last substep from the same state
//!   with a shortened h until tau lands within 1e-13 above the target - and never an interpolation
//!   between two points of it.
//! * **Fixed radius**, Static or ZAMO: u is a function of r alone, so the motion is the same closed
//!   form as the hold with the mode's own u, exactly as the app's `advance` adds dtau = dt / u^t.
//!
//! The hand-over at the release is the app's: the hold runs up to t = `release_t` exactly, and the
//! fall starts from the release event itself, with the geodesic the app parked there - the saved
//! seed 4-velocity, at the hold radius, carrying the proper time and azimuth the whole wait left on
//! the observer. A release from infinity therefore jumps in velocity (the hold was static and the
//! fall starts at two thirds of c) and a release at rest does not, as in the app.
//!
//! # Where the app depends on its frame pacing, and what is chosen here
//!
//! Each of these is a first-order effect of the app's step size. The exact behaviour - the limit
//! of small steps - is what is done here, and the difference is stated so that nobody mistakes it
//! for a disagreement:
//!
//! * A Static or ZAMO observer still waiting for release. The app's step that crosses `release_t`
//!   moves the observer the *whole* step at the fixed mode's rate (`step` only splits the crossing
//!   step for free fall), so the sliver of hold before `release_t` in that step is ticked at the
//!   wrong rate. Here the hold ends at `release_t` exactly and the fixed mode starts there.
//! * A Static or ZAMO selection that the observer's radius does not admit is followed in free fall
//!   (`effective_mode`), and the app switches back to the selection at the first frame boundary at
//!   which the radius admits it again, with a jump in velocity whose size depends on where that
//!   boundary fell. That is not a motion with a small-step limit worth filming - at r = 2M the
//!   static observer is null, so the jump is unbounded - and the film ends there instead, with the
//!   reason stated (`End::ModeResumes`).
//! * The free fall itself. The app integrates in windows of one frame of coordinate time, and each
//!   window ends by truncating a substep; here the windows end on frame proper times instead. The
//!   two are the same integrator with the same caps and differ only in where substeps are cut,
//!   which moves the result at the integrator's truncation error, and the tests measure how much.
//!
//! # Where the film ends
//!
//! At the requested frame count, or earlier at one of the worldline's own ends: the ring (the core
//! stops integrating at r = `R_STOP`), a freeze onto the inner horizon (the core's `stalled`,
//! after which the app holds r and tau fixed while only t and phi run on), or a crossing of r- -
//! the view from inside r- sees light that this phase has no fate codes for, so the film stops at
//! the last frame outside it. Each is an `End` the caller can print.

use kerr_equatorial::geodesic::R_STOP;
use kerr_equatorial::{GeodesicState, KerrSchild};

use crate::bhl::{Mode, SavedObserver};

/// The tolerance within which two events are "the same event" when the app asks whether the
/// geodesic it carries stands on the observer's current event (`geodesic_stands_on_current_event`,
/// 1e-9 relative). The same number, so that a save the app would continue from its geodesic is
/// continued from its geodesic here too.
const SAME_EVENT: f64 = 1e-9;

/// How far |u^r| may be from zero, relative to 1 + |u^t|, for the geodesic's 4-velocity to be
/// taken as at rest in r and therefore holdable at fixed r: `hover_four_velocity`'s own test.
const AT_REST_IN_R: f64 = 1e-6;

/// One event of the observer's worldline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Event {
    /// The observer's proper time, in M.
    pub tau: f64,
    /// The chart's coordinate time (ingoing Kerr-Schild), in M.
    pub t: f64,
    /// The chart's radius, in M.
    pub r: f64,
    /// The chart's azimuth, folded into [0, 2 pi) as the core's geodesic keeps it.
    pub phi: f64,
    /// The 4-velocity (u^t, u^r, u^phi) in the chart's components.
    pub u: [f64; 3],
    /// Which kind of motion the observer is in at this event.
    pub motion: Motion,
}

/// The kind of motion at an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Motion {
    /// Held at fixed r, waiting for the release.
    Holding,
    /// On a timelike geodesic.
    FreeFall,
    /// At fixed r and fixed phi.
    Static,
    /// At fixed r, turning with the frame dragging.
    Zamo,
}

impl Motion {
    pub fn name(self) -> &'static str {
        match self {
            Self::Holding => "held at fixed r, waiting for the release",
            Self::FreeFall => "in free fall",
            Self::Static => "static",
            Self::Zamo => "a ZAMO",
        }
    }
}

/// Why a film ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum End {
    /// The requested number of frames was made.
    Frames { count: usize },
    /// The observer reached the ring short of the next frame. The core stops integrating at the
    /// first substep that ends at or inside r = `R_STOP`, so `tau` and `r` are that substep's end:
    /// the ring's proper time is known to within one substep, which near the ring is a few 1e-6 M.
    Ring { tau: f64, r: f64 },
    /// The geodesic froze onto the inner horizon at proper time `tau`, at radius `r`.
    Stalled { tau: f64, r: f64 },
    /// The worldline is at or inside r- = `r_minus` by the frame at proper time `tau`, which is
    /// therefore not made.
    InnerHorizon { tau: f64, r_minus: f64 },
    /// The observer's Static or ZAMO selection became possible again at radius `r`, at the frame
    /// at proper time `tau`, where the app would leave free fall for it with a jump in velocity.
    ModeResumes { mode: Mode, tau: f64, r: f64 },
    /// The integrator stopped advancing without reaching any of the ends above. The core's caps
    /// are meant to make this impossible; it is here so that a failure is a sentence, not a hang.
    NoProgress { tau: f64, r: f64 },
}

impl std::fmt::Display for End {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            Self::Frames { count } => write!(f, "the requested {count} frames were made"),
            Self::Ring { tau, r } => write!(
                f,
                "the observer reaches the ring before the next frame: the integration stops at \
                 r = {r:.6} M, on its first step inside r = {R_STOP} M, at tau = {tau:.9} M"
            ),
            Self::Stalled { tau, r } => write!(
                f,
                "the observer's worldline freezes onto the inner horizon at tau = {tau:.9} M, \
                 r = {r:.12} M; past that the app holds r and tau fixed while only t runs on, so \
                 the film ends at the last frame before the freeze"
            ),
            Self::InnerHorizon { tau, r_minus } => write!(
                f,
                "the observer crosses the inner horizon r- = {r_minus:.9} M before the frame at \
                 tau = {tau:.9} M; the view from inside r- is outside this phase's scope, so the \
                 film ends at the last frame outside it"
            ),
            Self::ModeResumes { mode, tau, r } => write!(
                f,
                "at tau = {tau:.9} M the observer is back at r = {r:.9} M, where the {} worldline \
                 that was selected exists again; the app would leave free fall for it there with \
                 a jump in velocity, so the film ends at the frame before",
                mode.name()
            ),
            Self::NoProgress { tau, r } => write!(
                f,
                "the integrator stopped advancing at tau = {tau:.9} M, r = {r:.9} M without \
                 reaching the ring or the inner horizon"
            ),
        }
    }
}

/// The events of a film and why it ends.
#[derive(Debug, Clone, PartialEq)]
pub struct Film {
    /// Frame k is `events[k]`, at proper time tau_0 + k * step.
    pub events: Vec<Event>,
    pub end: End,
}

/// Why an observer cannot be walked, as a sentence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refused(pub String);

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Refused {}

/// The fixed-r worldline the observer holds until the release.
#[derive(Debug, Clone, Copy)]
struct Hold {
    t0: f64,
    tau0: f64,
    phi0: f64,
    r: f64,
    u: [f64; 3],
    /// The proper time at which the hold ends: the observer's watch at t = `release_t`.
    tau_release: f64,
}

/// A worldline of constant u at fixed r - the hold, or a Static or ZAMO observer - at proper time
/// `tau`, from the anchor event (t0, tau0, phi0).
fn fixed_r_event(
    t0: f64,
    tau0: f64,
    phi0: f64,
    r: f64,
    u: [f64; 3],
    tau: f64,
    motion: Motion,
) -> Event {
    let dtau = tau - tau0;
    Event {
        tau,
        t: t0 + u[0] * dtau,
        r,
        phi: (phi0 + u[2] * dtau).rem_euclid(std::f64::consts::TAU),
        u,
        motion,
    }
}

/// What the observer does after the hold, or from the start if there is no hold.
#[derive(Debug, Clone, Copy)]
enum Course {
    Fall(GeodesicState),
    Fixed {
        mode: Mode,
        t0: f64,
        tau0: f64,
        phi0: f64,
        r: f64,
        u: [f64; 3],
    },
}

/// An observer's worldline from the saved moment on, ready to be walked.
#[derive(Debug, Clone)]
pub struct Worldline {
    metric: KerrSchild,
    /// The proper time at the saved moment: frame 0.
    tau0: f64,
    hold: Option<Hold>,
    motion: Course,
    /// The mode the save selected. Differs from the motion followed where a Static or ZAMO
    /// selection is not possible at the observer's radius and the observer falls instead.
    selected: Mode,
    /// The longest stretch of coordinate time handed to the integrator in one call. See
    /// `with_call_window`.
    window: f64,
}

/// Can the `mode` worldline exist at radius r? The app's `mode_admissible_at`: a static observer
/// needs d/dt timelike (g_tt < 0, outside r = 2M on the equator), a ZAMO needs a timelike fixed-r
/// worldline (outside r+). Free fall always exists; a drag is refused before this is asked.
fn admissible(metric: &KerrSchild, mode: Mode, r: f64) -> bool {
    match mode {
        Mode::Static => metric.metric_components(r)[0][0] < 0.0,
        Mode::Zamo => r > metric.outer_horizon(),
        Mode::FreeFall | Mode::ManualDrag => true,
    }
}

/// u = (1, 0, 0) / sqrt(-g_tt), the static observer, where it exists.
fn static_u(metric: &KerrSchild, r: f64) -> Option<[f64; 3]> {
    let g_tt = metric.metric_components(r)[0][0];
    (g_tt < 0.0).then(|| [1.0 / (-g_tt).sqrt(), 0.0, 0.0])
}

/// u = gamma (1, 0, omega), omega = -g_tphi / g_phiphi, the ZAMO, outside r+. The app's
/// `zamo_four_velocity`, including its floor on the norm, so that both give the same u to the bit.
fn zamo_u(metric: &KerrSchild, r: f64) -> Option<[f64; 3]> {
    if r <= metric.outer_horizon() {
        return None;
    }
    let g = metric.metric_components(r);
    let omega = metric.frame_dragging_omega(r);
    let norm_sq = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
    let gamma = 1.0 / norm_sq.max(1e-14).sqrt();
    Some([gamma, 0.0, gamma * omega])
}

/// The 4-velocity of a fixed-r mode at r, where that mode exists.
fn fixed_u(metric: &KerrSchild, mode: Mode, r: f64) -> Option<[f64; 3]> {
    match mode {
        Mode::Static => static_u(metric, r),
        Mode::Zamo => zamo_u(metric, r),
        Mode::FreeFall | Mode::ManualDrag => None,
    }
}

fn motion_of(mode: Mode) -> Motion {
    match mode {
        Mode::Static => Motion::Static,
        Mode::Zamo => Motion::Zamo,
        Mode::FreeFall | Mode::ManualDrag => Motion::FreeFall,
    }
}

impl Worldline {
    /// The worldline of a saved observer, from the saved moment on.
    ///
    /// Refused, with a sentence: a dragged observer (its worldline is wherever the mouse put it,
    /// not a physical one); an observer at or inside r- (outside this phase's scope) or at the
    /// ring; one whose geodesic has already frozen onto r-; one waiting for release at a radius
    /// where nothing can hold it; and a state with a number in it that is not finite.
    pub fn from_saved(metric: &KerrSchild, obs: &SavedObserver) -> Result<Self, Refused> {
        let who = &obs.name;
        if obs.mode == Mode::ManualDrag {
            return Err(Refused(format!(
                "{who} is being dragged by hand in this save, and a worldline positioned by the \
                 mouse is not a physical one; let go of the marker in the app, save again, and \
                 film that"
            )));
        }
        if ![obs.t, obs.r, obs.phi, obs.tau]
            .iter()
            .all(|v| v.is_finite())
            || obs.release_t.is_nan()
        {
            return Err(Refused(format!(
                "{who}'s saved event (t = {}, r = {}, phi = {}, tau = {}) has a number in it that \
                 is not finite, so there is no event to continue from",
                obs.t, obs.r, obs.phi, obs.tau
            )));
        }
        check_radius(metric, who, obs.r)?;

        let saved_geo = obs.geodesic.map(|g| g.state());
        let (tau0, hold, motion) = if obs.is_active {
            let motion = if admissible(metric, obs.mode, obs.r) && obs.mode != Mode::FreeFall {
                let u = fixed_u(metric, obs.mode, obs.r).expect("admissible");
                Course::Fixed {
                    mode: obs.mode,
                    t0: obs.t,
                    tau0: obs.tau,
                    phi0: obs.phi,
                    r: obs.r,
                    u,
                }
            } else {
                Course::Fall(seeded_geodesic(
                    metric,
                    obs,
                    saved_geo,
                    obs.t.max(obs.release_t),
                ))
            };
            let tau0 = match motion {
                Course::Fall(geo) => geo.tau,
                Course::Fixed { .. } => obs.tau,
            };
            (tau0, None, motion)
        } else {
            let u = hold_u(metric, obs, saved_geo)?;
            if obs.release_t < obs.t {
                return Err(Refused(format!(
                    "{who} is waiting to be released at t = {} M, and the saved clock is already \
                     past that, at t = {} M; the save is not one the app can have written",
                    obs.release_t, obs.t
                )));
            }
            // The wait, closed form: the app's `hover` adds dtau = dt / u^t and dphi = (u^phi /
            // u^t) dt over the same interval. An infinite release_t is a hold that never ends.
            let wait = obs.release_t - obs.t;
            let tau_release = obs.tau + wait / u[0];
            let phi_release = obs.phi + (u[2] / u[0]) * wait;
            let hold = Hold {
                t0: obs.t,
                tau0: obs.tau,
                phi0: obs.phi,
                r: obs.r,
                u,
                tau_release,
            };
            let motion = if admissible(metric, obs.mode, obs.r) && obs.mode != Mode::FreeFall {
                let u = fixed_u(metric, obs.mode, obs.r).expect("admissible");
                Course::Fixed {
                    mode: obs.mode,
                    t0: obs.release_t,
                    tau0: tau_release,
                    phi0: phi_release,
                    r: obs.r,
                    u,
                }
            } else {
                // The geodesic the app parks at the release event: `hover` leaves it at t =
                // release_t with the observer's proper time and azimuth, and its 4-velocity and
                // radius are the seed's, untouched since the observer was created.
                let mut geo = saved_geo.unwrap_or_else(|| {
                    GeodesicState::new_infall(metric, obs.release_t, obs.r, 1.0, 0.0)
                });
                if (geo.r - obs.r).abs() > SAME_EVENT * (1.0 + obs.r.abs()) {
                    // The app would not continue from a geodesic parked anywhere but under the
                    // observer; it seeds a new one there, on the ingoing root, with the same
                    // constants (`seed_geodesic_at_current_event`).
                    geo = GeodesicState::new_infall(
                        metric,
                        obs.release_t,
                        obs.r,
                        geo.energy,
                        geo.l_ang,
                    );
                }
                geo.t = obs.release_t;
                geo.tau = tau_release;
                geo.phi = phi_release;
                geo.stalled = false;
                Course::Fall(geo)
            };
            (obs.tau, Some(hold), motion)
        };

        if let Course::Fall(geo) = motion {
            if geo.stalled {
                return Err(Refused(format!(
                    "{who}'s worldline has already frozen onto the inner horizon at the saved \
                     moment (r = {} M); there is no more of it to film",
                    geo.r
                )));
            }
            if ![geo.t, geo.r, geo.phi, geo.tau, geo.u[0], geo.u[1], geo.u[2]]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err(Refused(format!(
                    "{who}'s saved geodesic has a number in it that is not finite, so there is \
                     no state to integrate from"
                )));
            }
        }

        Ok(Self {
            metric: *metric,
            tau0,
            hold,
            motion,
            selected: obs.mode,
            window: f64::INFINITY,
        })
    }

    /// A free-falling observer on the given geodesic state, from that event on. For a state the
    /// caller has built: the tests start the walker from events of a trail the app recorded, and
    /// from orbits whose answer is known in closed form.
    pub fn free_fall(metric: &KerrSchild, geo: GeodesicState) -> Result<Self, Refused> {
        check_radius(metric, "The observer", geo.r)?;
        if geo.stalled {
            return Err(Refused(
                "the geodesic has already frozen onto the inner horizon".into(),
            ));
        }
        Ok(Self {
            metric: *metric,
            tau0: geo.tau,
            hold: None,
            motion: Course::Fall(geo),
            selected: Mode::FreeFall,
            window: f64::INFINITY,
        })
    }

    /// The same worldline with the integrator handed at most `dt` of coordinate time per call.
    ///
    /// This is the step control this walker has over the core's integrator, whose own caps (0.05 M,
    /// 0.008 r, and a 0.4 % change of u per substep) are fixed. Every call ends by truncating a
    /// substep, so a smaller window can only make substeps shorter; with the default, infinite
    /// window, the calls end on frame proper times alone. The tests walk the same worldline both
    /// ways and show that the answer does not move beyond the integrator's truncation error.
    pub fn with_call_window(mut self, dt: f64) -> Self {
        assert!(dt > 0.0, "a call window must be positive");
        self.window = dt;
        self
    }

    /// The observer's proper time at the saved moment: the proper time of frame 0.
    pub fn tau0(&self) -> f64 {
        self.tau0
    }

    /// The proper time at which the hold ends, if the observer is still held at the saved moment.
    pub fn tau_release(&self) -> Option<f64> {
        self.hold.map(|h| h.tau_release)
    }

    /// The motion the observer follows once any hold is over, which is what the app's
    /// `effective_mode` reports then.
    pub fn motion_after_release(&self) -> Motion {
        match self.motion {
            Course::Fall(_) => Motion::FreeFall,
            Course::Fixed { mode, .. } => motion_of(mode),
        }
    }

    /// The constants (E, L) of the geodesic the observer falls on, if the observer falls.
    pub fn constants(&self) -> Option<(f64, f64)> {
        match self.motion {
            Course::Fall(geo) => Some((geo.energy, geo.l_ang)),
            Course::Fixed { .. } => None,
        }
    }

    /// The event at which the observer's watch reads `tau`, or the end the worldline meets first.
    ///
    /// Successive calls must not ask for an earlier `tau` than the last: the free fall is carried
    /// forward by integration and is not wound back. A request for an earlier one is answered with
    /// the current event.
    pub fn event_at(&mut self, tau: f64) -> Result<Event, End> {
        if let Some(hold) = self.hold
            && tau < hold.tau_release
        {
            return Ok(fixed_r_event(
                hold.t0,
                hold.tau0,
                hold.phi0,
                hold.r,
                hold.u,
                tau,
                Motion::Holding,
            ));
        }
        let metric = self.metric;
        match &mut self.motion {
            Course::Fixed {
                mode,
                t0,
                tau0,
                phi0,
                r,
                u,
            } => Ok(fixed_r_event(
                *t0,
                *tau0,
                *phi0,
                *r,
                *u,
                tau,
                motion_of(*mode),
            )),
            Course::Fall(geo) => {
                let r_minus = metric.inner_horizon();
                while geo.tau < tau && !geo.stalled && geo.r > R_STOP {
                    let before = (geo.t, geo.tau);
                    geo.step_coord_time_until_tau(&metric, self.window, tau);
                    if (geo.t, geo.tau) == before && !geo.stalled && geo.r > R_STOP {
                        return Err(End::NoProgress {
                            tau: geo.tau,
                            r: geo.r,
                        });
                    }
                }
                // r- first: a worldline that has crossed it has left this phase's scope whatever
                // it did next. A freeze onto the right branch of r- stays above r- (about 7e-10 M
                // above it when the core declares the stall) and is reported as the freeze.
                if r_minus > 0.0 && geo.r <= r_minus {
                    return Err(End::InnerHorizon { tau, r_minus });
                }
                if geo.stalled {
                    return Err(End::Stalled {
                        tau: geo.tau,
                        r: geo.r,
                    });
                }
                if geo.r <= R_STOP {
                    return Err(End::Ring {
                        tau: geo.tau,
                        r: geo.r,
                    });
                }
                if matches!(self.selected, Mode::Static | Mode::Zamo)
                    && admissible(&metric, self.selected, geo.r)
                {
                    return Err(End::ModeResumes {
                        mode: self.selected,
                        tau: geo.tau,
                        r: geo.r,
                    });
                }
                Ok(Event {
                    tau: geo.tau,
                    t: geo.t,
                    r: geo.r,
                    phi: geo.phi,
                    u: geo.u,
                    motion: Motion::FreeFall,
                })
            }
        }
    }

    /// Frames 0 to `frames - 1`, frame k at proper time tau_0 + k * `step`, each computed from k
    /// afresh so that no rounding accumulates along the film; fewer if the worldline ends first.
    pub fn walk(&mut self, step: f64, frames: usize) -> Film {
        assert!(
            step > 0.0 && step.is_finite(),
            "a frame step must be positive and finite"
        );
        let mut events = Vec::with_capacity(frames.min(1 << 16));
        for k in 0..frames {
            let tau = self.tau0 + k as f64 * step;
            match self.event_at(tau) {
                Ok(event) => events.push(event),
                Err(end) => return Film { events, end },
            }
        }
        Film {
            events,
            end: End::Frames { count: frames },
        }
    }
}

/// Refuse a radius at the ring or at or inside r-.
fn check_radius(metric: &KerrSchild, who: &str, r: f64) -> Result<(), Refused> {
    if r <= R_STOP {
        return Err(Refused(format!(
            "{who} is at the ring (r = {r} M) at the saved moment; there is no worldline left to film"
        )));
    }
    let r_minus = metric.inner_horizon();
    if r_minus > 0.0 && r <= r_minus {
        return Err(Refused(format!(
            "{who} is at r = {r} M, at or inside the inner horizon r- = {r_minus} M; an observer \
             there sees light that came through the ring and from the other sheet of r-, which this \
             program cannot yet trace"
        )));
    }
    Ok(())
}

/// The 4-velocity of the fixed-r worldline a waiting observer is held on: the app's
/// `hover_four_velocity`. The geodesic's own 4-velocity where it is at rest in r (a release at rest
/// or onto a circular orbit, joined without a jump), and the static observer otherwise.
///
/// Refused where neither exists: waiting inside the static limit for a release that arrives
/// already moving. The app still holds r fixed there and ticks the watch at the u^t of the
/// ingoing geodesic, whose u^r is not zero, so its clock belongs to a worldline that is not the one
/// drawn, and no rocket can hold that radius with the observer's azimuth fixed.
fn hold_u(
    metric: &KerrSchild,
    obs: &SavedObserver,
    geo: Option<GeodesicState>,
) -> Result<[f64; 3], Refused> {
    if let Some(geo) = geo
        && geo.u[1].abs() <= AT_REST_IN_R * (1.0 + geo.u[0].abs())
    {
        return Ok(geo.u);
    }
    static_u(metric, obs.r).ok_or_else(|| {
        Refused(format!(
            "{} is waiting at r = {} M, inside the static limit, to be released at t = {} M on a \
             fall that is already moving there; no worldline holds that radius until then, so \
             scrub past the release and save again",
            obs.name, obs.r, obs.release_t
        ))
    })
}

/// The geodesic a released free-faller's next step integrates from: the app's `seeded_geodesic`.
/// The one the observer carries where it stands on the observer's event (to the app's 1e-9), and
/// otherwise a new one seeded there on the ingoing root with the carried constants - which is what
/// happens when a Static or ZAMO selection the radius does not admit falls.
fn seeded_geodesic(
    metric: &KerrSchild,
    obs: &SavedObserver,
    geo: Option<GeodesicState>,
    t_ref: f64,
) -> GeodesicState {
    if let Some(geo) = geo
        && (geo.t - t_ref).abs() <= SAME_EVENT * (1.0 + t_ref.abs())
        && (geo.r - obs.r).abs() <= SAME_EVENT * (1.0 + obs.r.abs())
    {
        return geo;
    }
    let (energy, l_ang) = geo.map(|g| (g.energy, g.l_ang)).unwrap_or((1.0, 0.0));
    let mut seeded = GeodesicState::new_infall(metric, obs.t, obs.r, energy, l_ang);
    seeded.phi = obs.phi;
    seeded.tau = obs.tau;
    seeded
}

#[cfg(test)]
mod tests;
