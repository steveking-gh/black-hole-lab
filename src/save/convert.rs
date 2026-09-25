//! The live types on one side, `v1` on the other, and one function per direction.
//!
//! Every field of every saved record is written across by hand here. That is the cost of keeping
//! the schema out of the physics and the GUI, and it is the cost that buys the guarantee: rename a
//! field of `Observer`, split one in two, change what one means, and the build breaks *in this
//! file*, at the line where somebody has to decide what the file should now say. Nothing can drift
//! silently, because nothing is derived.
//!
//! Two rules hold throughout.
//!
//! **Nothing is recomputed.** A conversion copies numbers; it does not re-derive them. The
//! temptation is real - an observer's `geodesic` could be rebuilt from its release constants, a
//! pulse's extent track from its rays - and every one of those would put a restored run on a
//! neighbouring worldline rather than on the one that was saved. The one exception is
//! `KerrSchild::with_solar_mass`, which is used rather than a struct literal because it enforces
//! the geometry's own bounds on M and a, and those clamps are idempotent on anything that has
//! already been through them.
//!
//! **`usize` goes out as u64 and comes back with `as usize`.** The app is built for 64-bit
//! machines and every one of these counts is a ray index or a pulse serial number; a file that
//! could not be read on a 32-bit build is a problem this format does not have to solve today, and
//! `Simulation::check_invariants` would catch a truncated index before it reached a step.

use std::collections::VecDeque;

use crate::gui::controls::{AppControls, ObserverSettings, ReferenceFrame, StepGrain, StepMode};
use crate::gui::spacetime_canvas::{BoxId, Canvas, Placement, SpacetimeCanvas, TelemetryBoxes};
use crate::gui::spatial_canvas::SpatialCanvas;
use crate::gui::volume_canvas::{Camera, VolumeCanvas};
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use crate::physics::observer::{Observer, ObserverMode, Release, TrailPoint, Who};
use crate::physics::simulation::Simulation;
use crate::physics::wavefront::{
    Delivery, FrontMark, NullRay, Pulse, RayEnd, RayMark, RaySample, Reception, RingHistory,
    RingRow, SignalField, TrackPoint,
};

use super::v1;

/// Shorthand for the newtype every f64 and f32 goes through. See `v1::Num`.
fn n(value: impl Into<v1::Num>) -> v1::Num {
    value.into()
}

// ---------------------------------------------------------------------------------------------
// The run, out
// ---------------------------------------------------------------------------------------------

pub fn sim_to_v1(sim: &Simulation) -> v1::Sim {
    v1::Sim {
        metric: v1::Metric {
            m: n(sim.metric.m),
            m_solar: n(sim.metric.m_solar),
            a: n(sim.metric.a),
        },
        clock: n(sim.clock),
        alice: sim.alice.as_ref().map(observer_to_v1),
        bob: sim.bob.as_ref().map(observer_to_v1),
        alice_signal: field_to_v1(&sim.alice_signal),
        bob_signal: field_to_v1(&sim.bob_signal),
    }
}

fn observer_to_v1(obs: &Observer) -> v1::Observer {
    v1::Observer {
        name: obs.name.clone(),
        mode: mode_to_v1(obs.mode),
        t: n(obs.t),
        r: n(obs.r),
        phi: n(obs.phi),
        tau: n(obs.tau),
        beta_r: n(obs.beta_r),
        beta_phi: n(obs.beta_phi),
        geodesic: obs.geodesic.map(|geo| v1::Geodesic {
            t: n(geo.t),
            r: n(geo.r),
            phi: n(geo.phi),
            tau: n(geo.tau),
            energy: n(geo.energy),
            l_ang: n(geo.l_ang),
            u: triple(&geo.u),
            stalled: geo.stalled,
        }),
        trail: obs.trail.iter().map(trail_point_to_v1).collect(),
        start: trail_point_to_v1(&obs.start),
        release_t: n(obs.release_t),
        release: release_to_v1(obs.release),
        is_active: obs.is_active,
    }
}

fn trail_point_to_v1(point: &TrailPoint) -> v1::TrailPoint {
    v1::TrailPoint {
        t: n(point.t),
        r: n(point.r),
        phi: n(point.phi),
        tau: n(point.tau),
        u: triple(&point.u),
        stalled: point.stalled,
    }
}

fn field_to_v1(field: &SignalField) -> v1::SignalField {
    v1::SignalField {
        pulses: field.pulses.iter().map(pulse_to_v1).collect(),
        t: n(field.t),
        next_index: field.next_index as u64,
        last_emit_tau: field.last_emit_tau.map(n),
        interval_tau: n(field.interval_tau),
        rays_per_pulse: field.rays_per_pulse as u64,
        max_pulses: field.max_pulses as u64,
        last_delivered: field.last_delivered.map(delivery_to_v1),
        heard: field.heard.iter().map(reception_to_v1).collect(),
        budget_exhausted: field.budget_exhausted as u64,
        dropped_in_flight: field.dropped_in_flight as u64,
    }
}

fn pulse_to_v1(pulse: &Pulse) -> v1::Pulse {
    v1::Pulse {
        index: pulse.index as u64,
        emitted_t: n(pulse.emitted_t),
        emitted_tau: n(pulse.emitted_tau),
        emitted_r: n(pulse.emitted_r),
        emitted_phi: n(pulse.emitted_phi),
        rays: pulse.rays.iter().map(ray_to_v1).collect(),
        extent_track: pulse
            .extent_track
            .iter()
            .map(|point| v1::TrackPoint {
                t: n(point.t),
                r_min: n(point.lo),
                r_max: n(point.hi),
                // A row with no role at all - every pulse saved before roles existed, and every
                // pulse with no ray of either kind - writes nothing, rather than a list of nulls
                // on each of up to four thousand rows. Reading an empty list gives back all NaN,
                // which is the row that was written.
                roles: if point.roles.iter().all(|r| r.is_nan()) {
                    Vec::new()
                } else {
                    point.roles.iter().map(|r| (!r.is_nan()).then(|| n(*r))).collect()
                },
            })
            .collect(),
        role_rays: if pulse.role_rays.iter().all(Option::is_none) {
            Vec::new()
        } else {
            pulse.role_rays.iter().map(|role| role.map(|i| i as u64)).collect()
        },
        track_dt: n(pulse.track_dt),
        history: pulse.history.as_ref().map(history_to_v1),
        prev: pulse.prev.as_ref().map(|mark| v1::FrontMark {
            rays: mark
                .rays
                .iter()
                .map(|ray| v1::RayMark {
                    r: n(ray.r),
                    rel: n(ray.rel),
                    dr_dt: n(ray.dr_dt),
                    dphi_dt: n(ray.dphi_dt),
                    alive: ray.alive,
                })
                .collect(),
            t: n(mark.t),
            r: n(mark.r),
            u_receiver: triple(&mark.u_receiver),
        }),
        receptions: pulse.receptions.iter().map(reception_to_v1).collect(),
    }
}

fn ray_to_v1(ray: &NullRay) -> v1::Ray {
    v1::Ray {
        t: n(ray.t),
        r: n(ray.r),
        phi: n(ray.phi),
        dr_dt: n(ray.dr_dt),
        dphi_dt: n(ray.dphi_dt),
        f_emit: n(ray.f_emit),
        v_emit: triple(&ray.v_emit),
        death_t: ray.death_t.map(n),
        death_end: ray.death_end.map(|end| match end {
            RayEnd::Ring => v1::RayEnd::Ring,
            RayEnd::Escape => v1::RayEnd::Escape,
            RayEnd::Unintegrable => v1::RayEnd::Unintegrable,
        }),
    }
}

fn history_to_v1(history: &RingHistory) -> v1::RingHistory {
    v1::RingHistory {
        rows: history
            .rows
            .iter()
            .map(|row| v1::RingRow {
                t: n(row.t),
                samples: row
                    .samples
                    .iter()
                    .map(|s| v1::RaySample { r: n(s.r), phi: n(s.phi), gain: n(s.gain) })
                    .collect(),
            })
            .collect(),
        history_dt: n(history.history_dt),
        stride: history.stride as u64,
    }
}

fn reception_to_v1(rec: &Reception) -> v1::Reception {
    v1::Reception {
        pulse_index: rec.pulse_index as u64,
        t: n(rec.t),
        tau_receiver: n(rec.tau_receiver),
        r: n(rec.r),
        phi: n(rec.phi),
        ratio: n(rec.ratio),
        dr_dt: n(rec.dr_dt),
        dphi_dt: n(rec.dphi_dt),
        frozen_family: rec.frozen_family,
        segment: rec.segment as u64,
        turn: rec.turn,
        side_after: n(rec.side_after),
        t_pass: n(rec.t_pass),
    }
}

fn delivery_to_v1(d: Delivery) -> v1::Delivery {
    v1::Delivery {
        pulse_index: d.pulse_index as u64,
        emitted_t: n(d.emitted_t),
        emitted_tau: n(d.emitted_tau),
        emitted_r: n(d.emitted_r),
        received_t: n(d.received_t),
    }
}

fn triple(u: &[f64; 3]) -> [v1::Num; 3] {
    [n(u[0]), n(u[1]), n(u[2])]
}

// ---------------------------------------------------------------------------------------------
// The run, back
// ---------------------------------------------------------------------------------------------

/// The run a file describes, built but not yet checked. `super::rebuild` is the only caller, and it
/// runs `Simulation::check_invariants` and the state hash over the result before anything is
/// assigned anywhere.
pub fn sim_from_v1(sim: &v1::Sim) -> Simulation {
    Simulation {
        metric: KerrSchild::with_solar_mass(sim.metric.m.0, sim.metric.a.0, sim.metric.m_solar.0),
        alice: sim.alice.as_ref().map(observer_from_v1),
        bob: sim.bob.as_ref().map(observer_from_v1),
        alice_signal: field_from_v1(&sim.alice_signal),
        bob_signal: field_from_v1(&sim.bob_signal),
        clock: sim.clock.0,
    }
}

fn observer_from_v1(obs: &v1::Observer) -> Observer {
    Observer {
        name: obs.name.clone(),
        mode: mode_from_v1(obs.mode),
        t: obs.t.0,
        r: obs.r.0,
        phi: obs.phi.0,
        tau: obs.tau.0,
        beta_r: obs.beta_r.0,
        beta_phi: obs.beta_phi.0,
        geodesic: obs.geodesic.as_ref().map(|geo| GeodesicState {
            t: geo.t.0,
            r: geo.r.0,
            phi: geo.phi.0,
            tau: geo.tau.0,
            energy: geo.energy.0,
            l_ang: geo.l_ang.0,
            u: untriple(&geo.u),
            stalled: geo.stalled,
        }),
        trail: obs.trail.iter().map(trail_point_from_v1).collect::<VecDeque<_>>(),
        start: trail_point_from_v1(&obs.start),
        release_t: obs.release_t.0,
        release: release_from_v1(obs.release),
        is_active: obs.is_active,
    }
}

fn trail_point_from_v1(point: &v1::TrailPoint) -> TrailPoint {
    TrailPoint {
        t: point.t.0,
        r: point.r.0,
        phi: point.phi.0,
        tau: point.tau.0,
        u: untriple(&point.u),
        stalled: point.stalled,
    }
}

fn field_from_v1(field: &v1::SignalField) -> SignalField {
    SignalField {
        pulses: field.pulses.iter().map(pulse_from_v1).collect::<VecDeque<_>>(),
        t: field.t.0,
        next_index: field.next_index as usize,
        last_emit_tau: field.last_emit_tau.map(|v| v.0),
        interval_tau: field.interval_tau.0,
        rays_per_pulse: field.rays_per_pulse as usize,
        max_pulses: field.max_pulses as usize,
        last_delivered: field.last_delivered.map(delivery_from_v1),
        heard: field.heard.iter().map(reception_from_v1).collect(),
        budget_exhausted: field.budget_exhausted as usize,
        dropped_in_flight: field.dropped_in_flight as usize,
    }
}

fn pulse_from_v1(pulse: &v1::Pulse) -> Pulse {
    Pulse {
        index: pulse.index as usize,
        emitted_t: pulse.emitted_t.0,
        emitted_tau: pulse.emitted_tau.0,
        emitted_r: pulse.emitted_r.0,
        emitted_phi: pulse.emitted_phi.0,
        rays: pulse.rays.iter().map(ray_from_v1).collect(),
        extent_track: pulse
            .extent_track
            .iter()
            .map(|p| TrackPoint {
                t: p.t.0,
                lo: p.r_min.0,
                hi: p.r_max.0,
                // A file older than the roles carries none, and a role past the end of the list
                // is one the file does not name: NaN, "no live ray in that role", either way.
                roles: std::array::from_fn(|i| {
                    p.roles.get(i).copied().flatten().map_or(f64::NAN, |r| r.0)
                }),
            })
            .collect(),
        role_rays: std::array::from_fn(|i| {
            pulse.role_rays.get(i).copied().flatten().map(|role| role as usize)
        }),
        track_dt: pulse.track_dt.0,
        history: pulse.history.as_ref().map(history_from_v1),
        prev: pulse.prev.as_ref().map(|mark| FrontMark {
            rays: mark
                .rays
                .iter()
                .map(|ray| RayMark {
                    r: ray.r.0,
                    rel: ray.rel.0,
                    dr_dt: ray.dr_dt.0,
                    dphi_dt: ray.dphi_dt.0,
                    alive: ray.alive,
                })
                .collect(),
            t: mark.t.0,
            r: mark.r.0,
            u_receiver: untriple(&mark.u_receiver),
        }),
        receptions: pulse.receptions.iter().map(reception_from_v1).collect(),
    }
}

fn ray_from_v1(ray: &v1::Ray) -> NullRay {
    NullRay {
        t: ray.t.0,
        r: ray.r.0,
        phi: ray.phi.0,
        dr_dt: ray.dr_dt.0,
        dphi_dt: ray.dphi_dt.0,
        f_emit: ray.f_emit.0,
        v_emit: untriple(&ray.v_emit),
        death_t: ray.death_t.map(|v| v.0),
        death_end: ray.death_end.map(|end| match end {
            v1::RayEnd::Ring => RayEnd::Ring,
            v1::RayEnd::Escape => RayEnd::Escape,
            v1::RayEnd::Unintegrable => RayEnd::Unintegrable,
        }),
        // The carried Dormand-Prince slope is derived from the four state fields above and the
        // metric, so a file has no business storing it and v1 does not: the first step after a
        // load evaluates `ray_rhs` at the restored state, which is the very number the saving run
        // was carrying. `test_a_loaded_run_plays_on_bit_identically` is where that is checked.
        slope: None,
    }
}

fn history_from_v1(history: &v1::RingHistory) -> RingHistory {
    RingHistory {
        rows: history
            .rows
            .iter()
            .map(|row| RingRow {
                t: row.t.0,
                samples: row
                    .samples
                    .iter()
                    .map(|s| RaySample { r: s.r.f32(), phi: s.phi.f32(), gain: s.gain.f32() })
                    .collect::<Vec<_>>()
                    .into_boxed_slice(),
            })
            .collect(),
        history_dt: history.history_dt.0,
        stride: history.stride as usize,
    }
}

fn reception_from_v1(rec: &v1::Reception) -> Reception {
    Reception {
        pulse_index: rec.pulse_index as usize,
        t: rec.t.0,
        tau_receiver: rec.tau_receiver.0,
        r: rec.r.0,
        phi: rec.phi.0,
        ratio: rec.ratio.0,
        dr_dt: rec.dr_dt.0,
        dphi_dt: rec.dphi_dt.0,
        frozen_family: rec.frozen_family,
        segment: rec.segment as usize,
        turn: rec.turn,
        side_after: rec.side_after.0,
        t_pass: rec.t_pass.0,
    }
}

fn delivery_from_v1(d: v1::Delivery) -> Delivery {
    Delivery {
        pulse_index: d.pulse_index as usize,
        emitted_t: d.emitted_t.0,
        emitted_tau: d.emitted_tau.0,
        emitted_r: d.emitted_r.0,
        received_t: d.received_t.0,
    }
}

fn untriple(u: &[v1::Num; 3]) -> [f64; 3] {
    [u[0].0, u[1].0, u[2].0]
}

// ---------------------------------------------------------------------------------------------
// The enums the physics and the panel share
// ---------------------------------------------------------------------------------------------

fn mode_to_v1(mode: ObserverMode) -> v1::ObserverMode {
    match mode {
        ObserverMode::FreeFall => v1::ObserverMode::FreeFall,
        ObserverMode::ManualDrag => v1::ObserverMode::ManualDrag,
        ObserverMode::Static => v1::ObserverMode::Static,
        ObserverMode::Zamo => v1::ObserverMode::Zamo,
    }
}

fn mode_from_v1(mode: v1::ObserverMode) -> ObserverMode {
    match mode {
        v1::ObserverMode::FreeFall => ObserverMode::FreeFall,
        v1::ObserverMode::ManualDrag => ObserverMode::ManualDrag,
        v1::ObserverMode::Static => ObserverMode::Static,
        v1::ObserverMode::Zamo => ObserverMode::Zamo,
    }
}

fn release_to_v1(release: Release) -> v1::Release {
    match release {
        Release::AtRest => v1::Release::AtRest,
        Release::FromInfinity => v1::Release::FromInfinity,
        Release::CircularPrograde => v1::Release::CircularPrograde,
        Release::CircularRetrograde => v1::Release::CircularRetrograde,
    }
}

fn release_from_v1(release: v1::Release) -> Release {
    match release {
        v1::Release::AtRest => Release::AtRest,
        v1::Release::FromInfinity => Release::FromInfinity,
        v1::Release::CircularPrograde => Release::CircularPrograde,
        v1::Release::CircularRetrograde => Release::CircularRetrograde,
    }
}

fn who_to_v1(who: Who) -> v1::Who {
    match who {
        Who::Alice => v1::Who::Alice,
        Who::Bob => v1::Who::Bob,
    }
}

fn who_from_v1(who: v1::Who) -> Who {
    match who {
        v1::Who::Alice => Who::Alice,
        v1::Who::Bob => Who::Bob,
    }
}

// ---------------------------------------------------------------------------------------------
// The panel
// ---------------------------------------------------------------------------------------------

/// `metric` is the run's own, and is here for one field: `step_distance_km` is the press amount in
/// kilometres, and M becomes kilometres only through the geometry the run was saved in. See that
/// field, and `step_size` above it, for why a write-only field is written at all.
pub fn controls_to_v1(controls: &AppControls, metric: &KerrSchild) -> v1::Controls {
    let press = controls.press_amount();
    v1::Controls {
        step_mode: match controls.step_mode {
            StepMode::Time => v1::StepMode::Time,
            StepMode::Distance => v1::StepMode::Distance,
            StepMode::Watch => v1::StepMode::Watch,
        },
        step_size: n(press),
        play_speed: n(controls.play_speed),
        step_distance_km: n(metric.r_to_km(press)),
        rays_per_pulse: controls.rays_per_pulse as u64,
        max_pulses: controls.max_pulses as u64,
        draw_front_arcs: controls.draw_front_arcs,
        hide_wound_segments: controls.hide_wound_segments,
        show_spatial_details: controls.show_spatial_details,
        alice: card_to_v1(&controls.alice),
        bob: card_to_v1(&controls.bob),
        show_theory_modal: controls.show_theory_modal,
        use_physical_units: controls.use_physical_units,
        decimal_is_comma: controls.decimal_is_comma,
        frame_of_ref: match controls.frame_of_ref {
            ReferenceFrame::DistantObserver => v1::ReferenceFrame::DistantObserver,
            ReferenceFrame::Bob => v1::ReferenceFrame::Bob,
            ReferenceFrame::Alice => v1::ReferenceFrame::Alice,
            ReferenceFrame::GlobalVolume => v1::ReferenceFrame::GlobalVolume,
        },
        show_distant_clock_grid: controls.show_distant_clock_grid,
        font_scale: n(controls.font_scale),
        step_grain: Some(controls.step_grain.key().to_string()),
    }
}

/// The panel a file describes, laid over the panel the app opens with.
///
/// Starting from `AppControls::default()` rather than from the running panel is what makes the
/// three transients - the transport flash, the watch read-out and the standing view-reset request -
/// come back at their resting values instead of being carried over from whatever the app happened
/// to be doing when the file was opened. `is_playing` comes back false for the reason
/// `v1::Controls` gives: a load comes up paused.
pub fn controls_from_v1(controls: &v1::Controls) -> AppControls {
    AppControls {
        is_playing: false,
        step_mode: match controls.step_mode {
            v1::StepMode::Time => StepMode::Time,
            v1::StepMode::Distance => StepMode::Distance,
            v1::StepMode::Watch => StepMode::Watch,
        },
        // The two amount fields are not read: see `v1::Controls::step_size`. A file from before the
        // Step Size dropdown, or one naming a grain this build has never heard of, comes up at the
        // panel's own default grain, and `play_speed` below then says what a press is worth.
        step_grain: controls
            .step_grain
            .as_deref()
            .and_then(StepGrain::from_key)
            .unwrap_or(AppControls::default().step_grain),
        play_speed: controls.play_speed.0,
        rays_per_pulse: controls.rays_per_pulse as usize,
        max_pulses: controls.max_pulses as usize,
        draw_front_arcs: controls.draw_front_arcs,
        hide_wound_segments: controls.hide_wound_segments,
        show_spatial_details: controls.show_spatial_details,
        alice: card_from_v1(&controls.alice),
        bob: card_from_v1(&controls.bob),
        show_theory_modal: controls.show_theory_modal,
        use_physical_units: controls.use_physical_units,
        decimal_is_comma: controls.decimal_is_comma,
        frame_of_ref: match controls.frame_of_ref {
            v1::ReferenceFrame::DistantObserver => ReferenceFrame::DistantObserver,
            v1::ReferenceFrame::Bob => ReferenceFrame::Bob,
            v1::ReferenceFrame::Alice => ReferenceFrame::Alice,
            v1::ReferenceFrame::GlobalVolume => ReferenceFrame::GlobalVolume,
        },
        show_distant_clock_grid: controls.show_distant_clock_grid,
        font_scale: controls.font_scale.f32(),
        ..AppControls::default()
    }
}

fn card_to_v1(card: &ObserverSettings) -> v1::ObserverSettings {
    v1::ObserverSettings {
        enabled: card.enabled,
        transmit: card.transmit,
        delta_t_delay: n(card.delta_t_delay),
        l_ang: n(card.l_ang),
        release: release_to_v1(card.release),
        drop_phi: n(card.drop_phi),
        drop_r: n(card.drop_r),
        mode: mode_to_v1(card.mode),
    }
}

fn card_from_v1(card: &v1::ObserverSettings) -> ObserverSettings {
    ObserverSettings {
        enabled: card.enabled,
        transmit: card.transmit,
        delta_t_delay: card.delta_t_delay.0,
        l_ang: card.l_ang.0,
        release: release_from_v1(card.release),
        drop_phi: card.drop_phi.0,
        drop_r: card.drop_r.0,
        mode: mode_from_v1(card.mode),
    }
}

// ---------------------------------------------------------------------------------------------
// The canvases
// ---------------------------------------------------------------------------------------------

pub fn view_to_v1(
    spacetime: &SpacetimeCanvas,
    spatial: &SpatialCanvas,
    volume: &VolumeCanvas,
) -> v1::View {
    v1::View {
        spacetime: v1::SpacetimeView {
            max_r: n(spacetime.max_r),
            r_offset: n(spacetime.r_offset),
            time_window: n(spacetime.time_window),
            time_offset: n(spacetime.time_offset),
            frame_max_r: n(spacetime.frame_max_r),
            keep_surface_framed: spacetime.keep_surface_framed,
            telemetry: telemetry_to_v1(&spacetime.telemetry),
        },
        spatial: v1::SpatialView {
            zoom: n(spatial.zoom),
            pan_offset: vec2_to_v1(spatial.pan_offset),
            centred_on: spatial.centred_on.map(who_to_v1),
            keep_hole_centred: spatial.keep_hole_centred,
            telemetry: telemetry_to_v1(&spatial.telemetry),
        },
        volume: v1::VolumeView {
            camera: v1::Camera {
                yaw: n(volume.camera.yaw),
                pitch: n(volume.camera.pitch),
                scale: n(volume.camera.scale),
                pan: vec2_to_v1(volume.camera.pan),
                t_scale: n(volume.camera.t_scale),
            },
            time_window: n(volume.time_window),
            time_offset: n(volume.time_offset),
            centred_on: volume.centred_on.map(who_to_v1),
            show_ghost_cones: volume.show_ghost_cones,
            show_past_cone: volume.show_past_cone,
            show_pulse_surfaces: volume.show_pulse_surfaces,
            focus_offset: vec2_to_v1(volume.focus_offset),
            telemetry: telemetry_to_v1(&volume.telemetry),
        },
    }
}

/// Put the saved view back on the three canvases, leaving every cache and every gesture alone.
///
/// The canvases are written in place rather than replaced, which is the point: `past_cone`,
/// `dragging`, the rest-frame view's sampled surface curves and its as-seen seeds are not in the
/// file and must not be reset by a load either. The cone will be rebuilt against the event it now
/// belongs to on the next frame, a drag in progress over a run that has just been replaced is ended
/// by the caller, the curves are re-sampled the moment their key no longer matches the restored
/// observer, and a stale seed costs one failed Newton before the cold solve picks the image up.
pub fn apply_view_v1(
    view: &v1::View,
    spacetime: &mut SpacetimeCanvas,
    spatial: &mut SpatialCanvas,
    volume: &mut VolumeCanvas,
) {
    spacetime.max_r = view.spacetime.max_r.0;
    spacetime.r_offset = view.spacetime.r_offset.0;
    spacetime.time_window = view.spacetime.time_window.0;
    spacetime.time_offset = view.spacetime.time_offset.0;
    spacetime.frame_max_r = view.spacetime.frame_max_r.0;
    spacetime.keep_surface_framed = view.spacetime.keep_surface_framed;
    apply_telemetry_v1(&view.spacetime.telemetry, &mut spacetime.telemetry);

    spatial.zoom = view.spatial.zoom.f32();
    spatial.pan_offset = vec2_from_v1(view.spatial.pan_offset);
    spatial.centred_on = view.spatial.centred_on.map(who_from_v1);
    spatial.keep_hole_centred = view.spatial.keep_hole_centred;
    apply_telemetry_v1(&view.spatial.telemetry, &mut spatial.telemetry);

    volume.camera = Camera {
        yaw: view.volume.camera.yaw.f32(),
        pitch: view.volume.camera.pitch.f32(),
        scale: view.volume.camera.scale.f32(),
        pan: vec2_from_v1(view.volume.camera.pan),
        t_scale: view.volume.camera.t_scale.0,
    };
    volume.time_window = view.volume.time_window.0;
    volume.time_offset = view.volume.time_offset.0;
    volume.centred_on = view.volume.centred_on.map(who_from_v1);
    volume.show_ghost_cones = view.volume.show_ghost_cones;
    volume.show_past_cone = view.volume.show_past_cone;
    volume.show_pulse_surfaces = view.volume.show_pulse_surfaces;
    volume.focus_offset = vec2_from_v1(view.volume.focus_offset);
    apply_telemetry_v1(&view.volume.telemetry, &mut volume.telemetry);
}

/// The dragged box positions of one canvas, in a fixed order.
///
/// The live collection is a hash map, whose iteration order changes between runs, so a save of it
/// would be different bytes every time and a round-trip could not be compared. Sorting on the slugs,
/// which are the names the file is written in rather than the variants' declaration order, makes two
/// saves of one state the same file.
fn telemetry_to_v1(boxes: &TelemetryBoxes) -> v1::Telemetry {
    let mut placements: Vec<v1::Placement> = boxes
        .placements
        .iter()
        .map(|((canvas, id), placement)| v1::Placement {
            canvas: canvas.key().to_string(),
            box_id: id.key().to_string(),
            at: match *placement {
                Placement::Offset(v) => v1::PlacementAt::Offset { x: n(v.x), y: n(v.y) },
                Placement::Pinned(v) => v1::PlacementAt::Pinned { x: n(v.x), y: n(v.y) },
            },
        })
        .collect();
    placements.sort_by(|a, b| (&a.canvas, &a.box_id).cmp(&(&b.canvas, &b.box_id)));
    let mut collapsed: Vec<v1::CollapsedBox> = boxes
        .collapsed
        .iter()
        .map(|(canvas, id)| v1::CollapsedBox {
            canvas: canvas.key().to_string(),
            box_id: id.key().to_string(),
        })
        .collect();
    collapsed.sort_by(|a, b| (&a.canvas, &a.box_id).cmp(&(&b.canvas, &b.box_id)));
    v1::Telemetry { pin_on_drag: boxes.pin_on_drag, placements, collapsed }
}

/// A placement whose canvas or box slug this build has never heard of is dropped rather than
/// refused. A dragged box is a convenience, and a file written by a build with one more picture in
/// it should still open every other thing it holds.
fn apply_telemetry_v1(saved: &v1::Telemetry, boxes: &mut TelemetryBoxes) {
    boxes.pin_on_drag = saved.pin_on_drag;
    boxes.placements.clear();
    for placement in &saved.placements {
        let (Some(canvas), Some(id)) =
            (Canvas::from_key(&placement.canvas), BoxId::from_key(&placement.box_id))
        else {
            continue;
        };
        let at = match placement.at {
            v1::PlacementAt::Offset { x, y } => {
                Placement::Offset(egui::vec2(x.f32(), y.f32()))
            }
            v1::PlacementAt::Pinned { x, y } => {
                Placement::Pinned(egui::vec2(x.f32(), y.f32()))
            }
        };
        boxes.placements.insert((canvas, id), at);
    }
    // A shut box is dropped on an unknown slug exactly as a moved one is, and for the same reason.
    // An older file has no list here at all and every box opens, which is how that file was saved.
    boxes.collapsed.clear();
    for shut in &saved.collapsed {
        if let (Some(canvas), Some(id)) =
            (Canvas::from_key(&shut.canvas), BoxId::from_key(&shut.box_id))
        {
            boxes.collapsed.insert((canvas, id));
        }
    }
}

fn vec2_to_v1(v: egui::Vec2) -> v1::Vec2 {
    v1::Vec2 { x: n(v.x), y: n(v.y) }
}

fn vec2_from_v1(v: v1::Vec2) -> egui::Vec2 {
    egui::vec2(v.x.f32(), v.y.f32())
}
