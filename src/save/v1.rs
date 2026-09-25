//! The on-disk schema, version 1, and nothing else.
//!
//! Every type here is a plain record with serde derives on it. None of them has a method that does
//! anything but move bytes, none of them knows what a Kerr-Schild metric is, and none of the live
//! types they mirror knows that this file exists. That separation is the whole point of the module:
//! the file format is a thing the program has promised to keep reading, and the live types are
//! things the program is free to refactor. Joining the two - putting `Serialize` on `Observer` -
//! would make every rename of a field a silent change to a format somebody has files in.
//!
//! What it costs is `super::convert`, which writes each field across by hand. What it buys is that
//! moving a field on `Observer` *breaks the build* in `convert.rs`, at the exact line where
//! somebody has to decide what the file should now say. That is the trade, and it is deliberate.
//!
//! **This file is frozen.** When the format changes, nothing here is edited: `v2.rs` is added
//! beside it with an `upgrade(v1::Save) -> v2::Save`, and `super`'s version dispatch grows an arm.
//! See the module documentation of `super` for the procedure in full.
//!
//! The one change this file may still take is an *additive* one: a new field with
//! `#[serde(default)]` on it, which an older file simply does not carry and which therefore needs
//! no version bump. Nothing here carries `deny_unknown_fields`, so a file written by a newer build
//! of the same format version - one that added such a field - still reads here, minus the field
//! this build has never heard of. A bump is for a change of structure or of meaning: a field whose
//! units change, a field that splits in two, a field that goes away.

use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// One f64 of the state, written as a JSON number where JSON can hold it and as a string where it
/// cannot.
///
/// JSON has no NaN and no infinity, and this state has both. `SignalField::interval_tau` is set to
/// infinity by at least one test; a `Reception::ratio` measured on light met *on* r₋ is a
/// blueshift factor going like 1/(r - r₋) at a radius that has rounded onto r₋ exactly. Left to
/// serde_json a non-finite f64 is written as `null` and read back as an error, so a run that had
/// reached one of those states could be saved and never opened.
///
/// So: a finite value is a JSON number, and the three others are the strings `"inf"`, `"-inf"` and
/// `"nan"`. Reading accepts both spellings of every value, which is what lets a file hand-edited
/// with `1e10` in place of `10000000000.0` still load.
///
/// Finite values round-trip to the bit. serde_json writes the shortest decimal that reads back as
/// the same f64, and the crate's `float_roundtrip` feature - see `Cargo.toml` - makes the parser
/// honour that instead of taking a fast path that can land an ulp away. Negative zero survives as
/// well: `-0.0` is written `-0.0` and read back with its sign bit.
///
/// The f32 fields of the view state (a camera angle, a zoom, a dragged box offset) come through
/// here too. Widening an f32 to f64 is exact and narrowing it back is exact for any value that
/// started as an f32, so one newtype serves both and the file has one spelling of a number rather
/// than two.
#[derive(Debug, Clone, Copy)]
pub struct Num(pub f64);

impl Num {
    /// The f32 this holds, for the view fields that are f32 on the live types.
    pub fn f32(self) -> f32 {
        self.0 as f32
    }
}

impl From<f64> for Num {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f32> for Num {
    fn from(value: f32) -> Self {
        Self(f64::from(value))
    }
}

/// Equality by bit pattern, with NaN equal to NaN.
///
/// The bits, because that is the claim a round-trip test is making: a save that moved the last bit
/// of a radius did not save the run. -0.0 and 0.0 are therefore different values here, which is
/// right - they are different f64s, and the format keeps them apart.
///
/// NaN is the one exception. A NaN's payload is not carried by the decimal spelling `"nan"`, so two
/// NaNs that differ only in their payload come back from a file as the same one. Comparing them as
/// equal says what the format actually promises about them, which is that a NaN stays a NaN.
impl PartialEq for Num {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits() || (self.0.is_nan() && other.0.is_nan())
    }
}

impl Serialize for Num {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if self.0.is_finite() {
            serializer.serialize_f64(self.0)
        } else if self.0.is_nan() {
            serializer.serialize_str("nan")
        } else if self.0 > 0.0 {
            serializer.serialize_str("inf")
        } else {
            serializer.serialize_str("-inf")
        }
    }
}

impl<'de> Deserialize<'de> for Num {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct NumVisitor;

        impl<'de> Visitor<'de> for NumVisitor {
            type Value = Num;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a number, or one of the strings \"inf\", \"-inf\", \"nan\"")
            }

            fn visit_f64<E: de::Error>(self, value: f64) -> Result<Num, E> {
                Ok(Num(value))
            }

            // An integer-valued number is written by serde_json without a decimal point only if
            // something else produced the file, but a hand-edited `0` has to read as 0.0.
            fn visit_i64<E: de::Error>(self, value: i64) -> Result<Num, E> {
                Ok(Num(value as f64))
            }

            fn visit_u64<E: de::Error>(self, value: u64) -> Result<Num, E> {
                Ok(Num(value as f64))
            }

            fn visit_str<E: de::Error>(self, value: &str) -> Result<Num, E> {
                match value {
                    "inf" | "+inf" | "Infinity" => Ok(Num(f64::INFINITY)),
                    "-inf" | "-Infinity" => Ok(Num(f64::NEG_INFINITY)),
                    "nan" | "NaN" => Ok(Num(f64::NAN)),
                    // A number that arrived quoted, which nothing here writes and a hand edit
                    // might: read it rather than refuse it.
                    other => other.parse::<f64>().map(Num).map_err(|_| {
                        E::custom(format!("{other:?} is not a number this format can read"))
                    }),
                }
            }
        }

        deserializer.deserialize_any(NumVisitor)
    }
}

/// The whole document, envelope and all.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Save {
    /// `super::FORMAT`. A file whose format string is anything else is not one of ours, whatever
    /// its extension says.
    pub format: String,
    /// The schema version, 1 for everything in this file.
    pub version: u32,
    pub written_by: WrittenBy,
    /// When the save was written, ISO-8601 UTC. For a human reading a directory of them; nothing
    /// in the loader reads it.
    pub saved_at_utc: String,
    /// Whatever the user wanted to say about this state. Empty is the normal case.
    #[serde(default)]
    pub note: String,
    /// `Simulation::fingerprint` of the state below, as sixteen hex digits. Recomputed on load and
    /// compared: a mismatch means the file has been corrupted or edited, or that a number in it did
    /// not come back as the number that went in, and the load is refused rather than the run being
    /// quietly wrong.
    pub state_hash: String,
    pub sim: Sim,
    pub controls: Controls,
    pub view: View,
}

/// Which build wrote the file. Both are best effort and neither is read by the loader; they are
/// there for the person holding a file that will not open.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrittenBy {
    pub app_version: String,
    /// The short commit hash, or "unknown".
    pub git: String,
}

// ---------------------------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------------------------

/// `physics::simulation::Simulation`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sim {
    pub metric: Metric,
    pub clock: Num,
    pub alice: Option<Observer>,
    pub bob: Option<Observer>,
    /// Alice's transmission, which Bob receives.
    pub alice_signal: SignalField,
    /// Bob's, which Alice receives.
    pub bob_signal: SignalField,
}

/// `physics::kerr_schild::KerrSchild`. It has a serde derive of its own already, for the
/// performance harness's reports; this is deliberately a second, separate description, so that the
/// harness's format and the save format can move independently of each other.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metric {
    pub m: Num,
    pub m_solar: Num,
    pub a: Num,
}

/// `physics::observer::Observer`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Observer {
    pub name: String,
    pub mode: ObserverMode,
    pub t: Num,
    pub r: Num,
    pub phi: Num,
    pub tau: Num,
    pub beta_r: Num,
    pub beta_phi: Num,
    pub geodesic: Option<Geodesic>,
    /// The worldline as it has been drawn, oldest first. It is the largest thing in a save after
    /// the rays, and it is not optional: a rewind reads it, and so does every arrival, which
    /// re-integrates the receiver's event out of it.
    pub trail: Vec<TrailPoint>,
    /// The event the observer was created at, which the trail's cap can evict and which a rewind
    /// into the hover needs exactly.
    pub start: TrailPoint,
    pub release_t: Num,
    pub release: Release,
    pub is_active: bool,
}

/// `physics::geodesic::GeodesicState`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Geodesic {
    pub t: Num,
    pub r: Num,
    pub phi: Num,
    pub tau: Num,
    pub energy: Num,
    pub l_ang: Num,
    /// u^mu = (u^t, u^r, u^phi).
    pub u: [Num; 3],
    pub stalled: bool,
}

/// `physics::observer::TrailPoint`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrailPoint {
    pub t: Num,
    pub r: Num,
    pub phi: Num,
    pub tau: Num,
    pub u: [Num; 3],
    pub stalled: bool,
}

/// `physics::observer::ObserverMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ObserverMode {
    FreeFall,
    ManualDrag,
    Static,
    Zamo,
}

/// `physics::observer::Release`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Release {
    AtRest,
    FromInfinity,
    CircularPrograde,
    CircularRetrograde,
}

/// `physics::wavefront::SignalField`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalField {
    /// Live pulses, oldest first.
    pub pulses: Vec<Pulse>,
    /// The field's own coordinate clock.
    pub t: Num,
    pub next_index: u64,
    pub last_emit_tau: Option<Num>,
    pub interval_tau: Num,
    pub rays_per_pulse: u64,
    pub max_pulses: u64,
    pub last_delivered: Option<Delivery>,
    /// Every arrival this transmission has made, including those whose pulse the cap has dropped.
    pub heard: Vec<Reception>,
    pub budget_exhausted: u64,
    pub dropped_in_flight: u64,
}

/// `physics::wavefront::Pulse`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pulse {
    pub index: u64,
    pub emitted_t: Num,
    pub emitted_tau: Num,
    pub emitted_r: Num,
    pub emitted_phi: Num,
    pub rays: Vec<Ray>,
    pub extent_track: Vec<TrackPoint>,
    /// `Pulse::role_rays`: the index into `rays` of each ray the (t, r) chart follows beside the
    /// two edges, role 0 the steepest freezer and role 1 the highest climber, null for a role
    /// the pulse has no ray for. Added after the format shipped: a file without it loads with no
    /// roles, and a pulse with no role at all writes an empty list.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub role_rays: Vec<Option<u64>>,
    pub track_dt: Num,
    pub history: Option<RingHistory>,
    /// Where this pulse's front and its receiver stood at the last detection pass. See `FrontMark`
    /// for why it is in the file rather than being re-taken on load.
    pub prev: Option<FrontMark>,
    pub receptions: Vec<Reception>,
}

/// `physics::wavefront::FrontMark`: one pulse's front and its receiver as both stood at the last
/// detection pass, which is the near end of every crossing the next step can find.
///
/// It is in the file, and that was not the first design. `SignalField::prime` exists to re-take
/// these marks after a `step_back`, and re-taking them after a load looked like the same job.
/// It is not, by four parts in 1e16, and the continuation test in `super::tests` is what said so.
/// The difference is one number: `Pulse::scan` chooses which turn of the azimuth to measure `rel`
/// from by pinning it to the previous mark's, and a mark taken fresh has no previous to pin to, so
/// it folds onto the nearest turn instead. For a front that has wound past half a turn relative to
/// the receiver those two differ by 2πk. That shift cancels out of the crossing algebra
/// *mathematically* - the turn index the sweep finds moves by the same k - and it does not cancel
/// in floating point: `a00 + 2πk - 2π(turn + k)` is a few ulps away from `a00 - 2π·turn`. Measured
/// on the app's opening layout, six frames after a load, that put one arrival's crossing time four
/// ulps off the run that was never interrupted, and the fingerprints parted company.
///
/// So the mark is saved. Four of its five per-ray numbers are the ray's own current state and could
/// in principle be rebuilt; they are written anyway, because what is in a mark is the physics's
/// decision and not this module's, and a `FrontMark` that grew a field would otherwise be restored
/// wrong without anything failing to compile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrontMark {
    /// One entry per ray of the pulse, in ray order.
    pub rays: Vec<RayMark>,
    /// The receiver's coordinate time at that pass.
    pub t: Num,
    /// The receiver's radius at that pass.
    pub r: Num,
    /// The receiver's 4-velocity at that pass.
    pub u_receiver: [Num; 3],
}

/// `physics::wavefront::RayMark`: where one ray of a front stood at that pass, and which way it was
/// going.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RayMark {
    pub r: Num,
    /// The ray's azimuth relative to the receiver, unwrapped along the ray loop and pinned to the
    /// branch the previous pass was measured on. The one number of a mark that cannot be recovered
    /// from the ray, and the reason marks are saved at all.
    pub rel: Num,
    pub dr_dt: Num,
    pub dphi_dt: Num,
    pub alive: bool,
}

/// `physics::wavefront::NullRay`.
///
/// Named fields, one ray per object, and no positional packing. A ray is the most repeated record
/// in the file by two orders of magnitude, which is exactly the argument that used to be made for
/// packing it into an array; gzip answers that argument, the repeated keys of a hundred thousand
/// identical objects being the most compressible thing a file can contain. What named fields buy
/// is a file somebody can read in an editor and a format a later version can add a field to.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    pub t: Num,
    pub r: Num,
    pub phi: Num,
    pub dr_dt: Num,
    pub dphi_dt: Num,
    pub f_emit: Num,
    /// The direction v^mu = (1, dr/dt, dphi/dt) at the emission event.
    pub v_emit: [Num; 3],
    pub death_t: Option<Num>,
    pub death_end: Option<RayEnd>,
}

/// `physics::wavefront::RayEnd`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RayEnd {
    Ring,
    Escape,
    Unintegrable,
}

/// One entry of a pulse's extent track: the radial interval the front spanned at that time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrackPoint {
    pub t: Num,
    pub r_min: Num,
    pub r_max: Num,
    /// The radius of each of the pulse's role rays at that time, in the order of
    /// `Pulse::role_rays`, null where the role has no live ray. Added after the format shipped: a
    /// file without it loads with no role recorded, and a row with no live role writes an empty
    /// list. A null rather than `Num`'s `"nan"` because a missing ray is not a number at all.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub roles: Vec<Option<Num>>,
}

/// `physics::wavefront::RingHistory`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RingHistory {
    pub rows: Vec<RingRow>,
    pub history_dt: Num,
    pub stride: u64,
}

/// `physics::wavefront::RingRow`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RingRow {
    pub t: Num,
    pub samples: Vec<RaySample>,
}

/// `physics::wavefront::RaySample`. The three are f32 on the live type - it is display data, held
/// small on purpose - and are widened here for the reason `Num` gives.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RaySample {
    pub r: Num,
    pub phi: Num,
    pub gain: Num,
}

/// `physics::wavefront::Reception`. The last four fields are the ones nothing draws: they are what
/// `SignalField::prime` needs to decide whether a rewind has undone the pass that noticed an
/// arrival, so a save that left them out would leave a reloaded run able to record an arrival
/// twice.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Reception {
    pub pulse_index: u64,
    pub t: Num,
    pub tau_receiver: Num,
    pub r: Num,
    pub phi: Num,
    pub ratio: Num,
    pub dr_dt: Num,
    pub dphi_dt: Num,
    pub frozen_family: bool,
    pub segment: u64,
    pub turn: i64,
    pub side_after: Num,
    pub t_pass: Num,
}

/// `physics::wavefront::Delivery`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Delivery {
    pub pulse_index: u64,
    pub emitted_t: Num,
    pub emitted_tau: Num,
    pub emitted_r: Num,
    pub received_t: Num,
}

// ---------------------------------------------------------------------------------------------
// The panel
// ---------------------------------------------------------------------------------------------

/// The *settings* of `gui::controls::AppControls`, and only those.
///
/// Three fields of the live type are not here and will not be: `transport_flash` is which button
/// was pressed a moment ago, `achieved_watch_rate` is a read-out written by the frame that has just
/// been played, and `view_reset_requested` is a one-shot request between the panel and the next
/// frame. None of them is a setting, and none of them means anything an hour later.
///
/// `is_playing` is absent for a different reason: a load always comes up paused. Restoring a run
/// into a window that immediately starts moving it takes the state away from the user before they
/// have seen it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Controls {
    pub step_mode: StepMode,
    /// What one press of Step Back, Step Fwd or an arrow key covers, in M of the step mode's own
    /// quantity.
    ///
    /// Written, and never read back. The panel's Step Size control is a *grain* - a multiple of one
    /// played frame, in `step_grain` below - and the amount a press comes to follows from the grain
    /// and the play speed, so a load takes the grain and works the amount out again. What goes in
    /// here is the amount this build's panel would take on a press, which is what the field has
    /// always meant, so a build from before the dropdown opens one of today's files and steps by
    /// about the same amount.
    pub step_size: Num,
    pub play_speed: Num,
    /// What one press covers in Distance mode, in kilometres. Written and never read back, for the
    /// reason `step_size` gives: this is `step_size` in the unit the old Step Dist slider was
    /// dialled in, converted through the metric of the run being saved.
    pub step_distance_km: Num,
    pub rays_per_pulse: u64,
    pub max_pulses: u64,
    pub draw_front_arcs: bool,
    pub hide_wound_segments: bool,
    pub show_spatial_details: bool,
    pub alice: ObserverSettings,
    pub bob: ObserverSettings,
    pub show_theory_modal: bool,
    pub use_physical_units: bool,
    pub frame_of_ref: ReferenceFrame,
    pub show_distant_clock_grid: bool,
    pub font_scale: Num,
    /// How big one press is, as `gui::controls::StepGrain::key` spells it: the Step Size dropdown.
    ///
    /// Additive, and the reason the two fields above are write-only. A file written before the
    /// dropdown existed carries no `step_grain` at all and loads with the panel's default grain,
    /// one played frame; a slug this build has never heard of loads the same way. The amount a
    /// press comes to is the grain against `play_speed`, which every version of this schema has
    /// carried.
    #[serde(default)]
    pub step_grain: Option<String>,
    /// The "Decimal is comma" box. Additive: a file from before the box carries no such field and
    /// loads in point style, which is what every build before the box wrote in.
    #[serde(default)]
    pub decimal_is_comma: bool,
}

/// `gui::controls::ObserverSettings`: one OBSERVER card.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ObserverSettings {
    pub enabled: bool,
    pub transmit: bool,
    pub delta_t_delay: Num,
    pub l_ang: Num,
    pub release: Release,
    pub drop_phi: Num,
    pub drop_r: Num,
    pub mode: ObserverMode,
}

/// `gui::controls::StepMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StepMode {
    Time,
    Distance,
    Watch,
}

/// `gui::controls::ReferenceFrame`, the View selector.
///
/// The live type has display labels and no key, and a label is exactly what a file must not be
/// written in: "Global Foliation Chart 1D+1 (Kerr-Schild)" is a sentence somebody is free to
/// reword. These four names are the file's own, and are fixed for as long as format 1 is read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ReferenceFrame {
    DistantObserver,
    Bob,
    Alice,
    GlobalVolume,
}

/// `physics::observer::Who`, for the canvases' "keep this observer centred" setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Who {
    Alice,
    Bob,
}

// ---------------------------------------------------------------------------------------------
// The canvases
// ---------------------------------------------------------------------------------------------

/// What each canvas is looking at.
///
/// Caches and gestures are not here and never will be: the volume view's `past_cone` is an
/// integration the next frame will redo, the equatorial view's `dragging` is a pointer that is not
/// down any more, and the rest-frame view's sampled surface curves and as-seen seeds are answers
/// about an event the file already carries. All of them rebuild themselves, and a file that carried
/// them would be a file that could restore a drag that nobody is performing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct View {
    pub spacetime: SpacetimeView,
    pub spatial: SpatialView,
    pub volume: VolumeView,
}

/// `gui::spacetime_canvas::SpacetimeCanvas`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpacetimeView {
    pub max_r: Num,
    pub r_offset: Num,
    pub time_window: Num,
    pub time_offset: Num,
    /// The rest-frame view's own window, in M of the local chart's xi.
    pub frame_max_r: Num,
    pub keep_surface_framed: bool,
    pub telemetry: Telemetry,
}

/// `gui::spatial_canvas::SpatialCanvas`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpatialView {
    /// Pixels per M.
    pub zoom: Num,
    pub pan_offset: Vec2,
    pub centred_on: Option<Who>,
    pub keep_hole_centred: bool,
    pub telemetry: Telemetry,
}

/// `gui::volume_canvas::VolumeCanvas`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VolumeView {
    pub camera: Camera,
    pub time_window: Num,
    pub time_offset: Num,
    pub centred_on: Option<Who>,
    pub show_ghost_cones: bool,
    pub show_past_cone: bool,
    pub show_pulse_surfaces: bool,
    /// The screen offset the last frame projected the followed observer's floor point to. Carried
    /// because the menu's "look at" arithmetic reads it before any frame has had a chance to write
    /// it again, and a restored view that is being followed would otherwise jump on the first
    /// click.
    pub focus_offset: Vec2,
    pub telemetry: Telemetry,
}

/// `gui::volume_canvas::Camera`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Camera {
    pub yaw: Num,
    pub pitch: Num,
    /// Pixels per M, the same meaning as the equatorial view's zoom.
    pub scale: Num,
    pub pan: Vec2,
    /// M of vertical world distance per M of coordinate time.
    pub t_scale: Num,
}

/// A screen vector, in points.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: Num,
    pub y: Num,
}

/// `gui::spacetime_canvas::TelemetryBoxes`: where the user has dragged each info box on one canvas.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Telemetry {
    /// Whether a drag pins the box where it was dropped or leaves it following its observer. A
    /// property of the canvas rather than of the user's dragging, written so that the file
    /// describes the whole object rather than half of it.
    pub pin_on_drag: bool,
    /// One entry per box the user has moved, sorted by (canvas, box) so that two saves of the same
    /// state are the same bytes. The live collection is a hash map, whose order is not.
    pub placements: Vec<Placement>,
    /// One entry per box the user has shut down to its title line, in the same order and for the
    /// same reason as `placements`.
    ///
    /// Additive: a file written before the disclosure triangle existed carries no list at all and
    /// loads with every box open, which is the state that file was saved in. A box named here and
    /// nowhere in `placements` is an ordinary case - shutting a box is not moving it.
    #[serde(default)]
    pub collapsed: Vec<CollapsedBox>,
}

/// One box standing shut, named by the same stable slugs a `Placement` is named by, and dropped on
/// load for the same reason where this build knows neither slug.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollapsedBox {
    pub canvas: String,
    #[serde(rename = "box")]
    pub box_id: String,
}

/// One dragged box, named by the stable slugs of `Canvas::key` and `BoxId::key`.
///
/// A slug this build does not know is dropped on load rather than refused: a placement is a
/// convenience, and a file from a build with a canvas this one has never had should still open.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub canvas: String,
    #[serde(rename = "box")]
    pub box_id: String,
    pub at: PlacementAt,
}

/// `gui::spacetime_canvas::Placement`: displaced from the box's anchor, or pinned to the canvas.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum PlacementAt {
    Offset { x: Num, y: Num },
    Pinned { x: Num, y: Num },
}
