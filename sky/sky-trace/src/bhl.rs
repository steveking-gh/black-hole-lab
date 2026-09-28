//! A reader for the part of a Black Hole Lab save that says where the observers are.
//!
//! The app's own reader cannot be linked from here: it lives in the app's binary crate, which has
//! no library target, and the app is not to be touched for this program's sake. So this is a
//! second reader of the same format, written against the format's description (the module comment
//! of the app's `src/save/mod.rs`) and its schema (`src/save/v1.rs`), and held to them by the tests
//! in `tests.rs`, which open the two saves the repository keeps.
//!
//! It reads the envelope's `format` and `version` first and refuses anything else with a sentence,
//! exactly as the app does, and then only the `sim` section's hole, clock and observers. The rest
//! of a save - the panel's settings, the canvases' views, and above all the two transmissions,
//! whose rays are nearly the whole of a file - is named by no field here, so serde_json skips it
//! token by token as it parses and none of it is ever built in memory.
//!
//! An unknown field is ignored rather than refused, which is the format's own compatibility rule:
//! the app adds an optional field to version 1 without raising the version, so a file written by a
//! newer build of the same version must still open here.

use std::io::Read;
use std::path::Path;

use kerr_equatorial::KerrSchild;
use serde::Deserialize;
use serde::de::{self, Deserializer, Visitor};

/// The string every save carries in `format`.
pub const FORMAT: &str = "black-hole-lab-save";

/// The newest version of the save format this reader knows. The app is at 1 as well; when it moves
/// to 2 the schema below is the one that has to be looked at again, because the app's rule is that
/// a new version is a change of structure or meaning, not an added field.
pub const VERSION: u32 = 1;

/// G M_sun / c^3, in seconds: the time one M of the chart's clock lasts for a hole of one solar
/// mass. It is the geometry crate's own number, which the app converts with too
/// (`KerrSchild::t_grav_seconds`), so that a reading here and the same reading in the app are
/// the same number.
pub const GM_SUN_OVER_C3_SECONDS: f64 = kerr_equatorial::kerr_schild::GM_SUN_OVER_C3_SECONDS;

/// Everything that can go wrong opening a save, each with its own sentence.
#[derive(Debug)]
pub enum ReadError {
    /// The file could not be read at all.
    Io(std::io::Error),
    /// The bytes start like a gzip stream and do not decompress.
    Gzip(String),
    /// The text is not JSON, or not JSON of the shape a save has.
    Json(String),
    /// The document is JSON but not a Black Hole Lab save.
    NotASave { found: String },
    /// A save of a version this reader does not know.
    UnknownVersion { version: u32 },
}

impl std::fmt::Display for ReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(cause) => write!(f, "the file could not be read: {cause}"),
            Self::Gzip(why) => write!(
                f,
                "the file starts like a compressed save but will not decompress ({why}); it is \
                 probably truncated"
            ),
            Self::Json(why) => write!(f, "the file is not readable as a save: {why}"),
            Self::NotASave { found } => write!(
                f,
                "the file is not a Black Hole Lab save: it calls itself {found:?} and a save \
                 calls itself {FORMAT:?}"
            ),
            Self::UnknownVersion { version } => write!(
                f,
                "the file is a Black Hole Lab save of format version {version}, and this reader \
                 knows only version {VERSION}"
            ),
        }
    }
}

impl std::error::Error for ReadError {}

/// What a save says about the run: the hole, the clock, and each observer present.
#[derive(Debug, Clone, PartialEq)]
pub struct Save {
    /// The app version that wrote the file, and its short commit hash, where the file says.
    pub written_by: Option<(String, String)>,
    /// When the file was written, as the file spells it.
    pub saved_at_utc: String,
    /// The note the user gave the save; empty is normal.
    pub note: String,
    /// The save's `state_hash`: the app's fingerprint of the run, in hex, which the app checks on
    /// load. Carried into a bundle's manifest so that the bundle names the state it was filmed
    /// from. Empty if the file has none.
    pub state_hash: String,
    pub hole: Hole,
    /// The simulation clock at the saved moment: the chart's coordinate time t, in M.
    pub clock: f64,
    pub alice: Option<SavedObserver>,
    pub bob: Option<SavedObserver>,
}

/// The hole, as the save states it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hole {
    /// M in the chart's units. The app keeps it at 1.
    pub m: f64,
    /// The spin a, in the chart's units.
    pub a: f64,
    /// The mass in solar masses, which is what turns M of time into seconds.
    pub m_solar: f64,
}

impl Hole {
    /// The metric, built as the app builds it on load (`KerrSchild::with_solar_mass`, whose clamps
    /// on M and a are idempotent on anything the app wrote), so that both programs integrate in
    /// the same geometry to the bit.
    pub fn metric(&self) -> KerrSchild {
        KerrSchild::with_solar_mass(self.m, self.a, self.m_solar)
    }

    /// Seconds in one unit of the chart's time. The chart's unit is M / m, so for a hole of
    /// `m_solar` solar masses it lasts (m_solar / m) G M_sun / c^3.
    pub fn seconds_per_unit(&self) -> f64 {
        self.m_solar / self.m * GM_SUN_OVER_C3_SECONDS
    }
}

/// How an observer's worldline is generated: the app's `ObserverMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Mode {
    /// A timelike geodesic.
    FreeFall,
    /// Positioned by the mouse.
    ManualDrag,
    /// Fixed r and fixed phi.
    Static,
    /// Fixed r, turning with the frame dragging so that u_phi = 0.
    Zamo,
}

impl Mode {
    /// The mode as the app's panel names it.
    pub fn name(self) -> &'static str {
        match self {
            Self::FreeFall => "free fall",
            Self::ManualDrag => "dragged by hand",
            Self::Static => "static (fixed r and phi)",
            Self::Zamo => "ZAMO (fixed r, turning with the dragging)",
        }
    }
}

/// What an observer's release means: the app's `Release`. It decided the observer's (E, L) when
/// the observer was created, and the save carries those constants in `geodesic`; the release is
/// read so that it can be reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Release {
    AtRest,
    FromInfinity,
    CircularPrograde,
    CircularRetrograde,
}

impl Release {
    /// The release as a phrase: "released at rest".
    pub fn phrase(self) -> &'static str {
        match self {
            Self::AtRest => "released at rest",
            Self::FromInfinity => "released as if fallen from rest at infinity",
            Self::CircularPrograde => "released onto the prograde circular orbit",
            Self::CircularRetrograde => "released onto the retrograde circular orbit",
        }
    }
}

/// The geodesic state the app carries under an observer: `GeodesicState`, field for field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SavedGeodesic {
    pub t: f64,
    pub r: f64,
    pub phi: f64,
    pub tau: f64,
    /// E = -u_t.
    pub energy: f64,
    /// L = u_phi.
    pub l_ang: f64,
    /// u^mu = (u^t, u^r, u^phi).
    pub u: [f64; 3],
    pub stalled: bool,
}

impl SavedGeodesic {
    /// The state as the core's type, which is how the app holds it.
    pub fn state(&self) -> kerr_equatorial::GeodesicState {
        kerr_equatorial::GeodesicState {
            t: self.t,
            r: self.r,
            phi: self.phi,
            tau: self.tau,
            energy: self.energy,
            l_ang: self.l_ang,
            u: self.u,
            stalled: self.stalled,
        }
    }
}

/// One event of an observer's recorded trail, as the app's integrator produced it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrailPoint {
    pub t: f64,
    pub r: f64,
    pub phi: f64,
    pub tau: f64,
    pub u: [f64; 3],
    pub stalled: bool,
}

/// One observer as saved: everything the app needs to carry its worldline forward.
///
/// `beta_r` and `beta_phi` are not read. They matter only to a dragged observer, and a dragged
/// observer is refused by the walker.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedObserver {
    pub name: String,
    pub mode: Mode,
    /// The observer's own event: coordinate time, radius, azimuth and proper time.
    pub t: f64,
    pub r: f64,
    pub phi: f64,
    pub tau: f64,
    pub geodesic: Option<SavedGeodesic>,
    /// The recorded worldline, oldest first, ending on the observer's current event.
    pub trail: Vec<TrailPoint>,
    /// The event the observer was created at.
    pub start: TrailPoint,
    /// The coordinate time of the release; the observer holds its radius until then.
    pub release_t: f64,
    pub release: Release,
    /// Whether the observer has been released.
    pub is_active: bool,
}

/// Open a save from a file.
pub fn read_file(path: &Path) -> Result<Save, ReadError> {
    let bytes = std::fs::read(path).map_err(ReadError::Io)?;
    read_bytes(&bytes)
}

/// Read a save out of a file's bytes: gzip or plain JSON, told apart by the two gzip magic bytes
/// as the app tells them apart, then the header, then the part of the document this program uses.
///
/// Two passes over the text, as in the app: the header decides which schema the rest is read with,
/// and parsing it on its own is what turns "missing field" into a sentence about versions.
pub fn read_bytes(bytes: &[u8]) -> Result<Save, ReadError> {
    let decompressed;
    let text: &[u8] = if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(bytes)
            .read_to_end(&mut out)
            .map_err(|e| ReadError::Gzip(e.to_string()))?;
        decompressed = out;
        &decompressed
    } else {
        bytes
    };

    let header: Header =
        serde_json::from_slice(text).map_err(|e| ReadError::Json(e.to_string()))?;
    if header.format != FORMAT {
        return Err(ReadError::NotASave {
            found: header.format,
        });
    }
    if header.version != VERSION {
        return Err(ReadError::UnknownVersion {
            version: header.version,
        });
    }
    let doc: Document = serde_json::from_slice(text).map_err(|e| ReadError::Json(e.to_string()))?;
    Ok(doc.into_save())
}

// ---------------------------------------------------------------------------------------------
// The schema: the fields of the app's v1 that this program reads, and nothing else
// ---------------------------------------------------------------------------------------------

/// The two fields that decide how the rest is read. Both default, so that a JSON object that is
/// not a save at all reaches the "not a save" sentence rather than a parse error.
#[derive(Deserialize)]
struct Header {
    #[serde(default)]
    format: String,
    #[serde(default)]
    version: u32,
}

#[derive(Deserialize)]
struct Document {
    #[serde(default)]
    written_by: Option<WrittenBy>,
    #[serde(default)]
    saved_at_utc: String,
    #[serde(default)]
    note: String,
    #[serde(default)]
    state_hash: String,
    sim: Sim,
}

#[derive(Deserialize)]
struct WrittenBy {
    #[serde(default)]
    app_version: String,
    #[serde(default)]
    git: String,
}

/// `v1::Sim` without `alice_signal` and `bob_signal`, which are skipped unread.
#[derive(Deserialize)]
struct Sim {
    metric: Metric,
    clock: Num,
    alice: Option<Observer>,
    bob: Option<Observer>,
}

#[derive(Deserialize)]
struct Metric {
    m: Num,
    m_solar: Num,
    a: Num,
}

/// `v1::Observer` without `beta_r` and `beta_phi`.
#[derive(Deserialize)]
struct Observer {
    name: String,
    mode: Mode,
    t: Num,
    r: Num,
    phi: Num,
    tau: Num,
    geodesic: Option<Geodesic>,
    trail: Vec<Point>,
    start: Point,
    release_t: Num,
    release: Release,
    is_active: bool,
}

#[derive(Deserialize)]
struct Geodesic {
    t: Num,
    r: Num,
    phi: Num,
    tau: Num,
    energy: Num,
    l_ang: Num,
    u: [Num; 3],
    stalled: bool,
}

#[derive(Deserialize)]
struct Point {
    t: Num,
    r: Num,
    phi: Num,
    tau: Num,
    u: [Num; 3],
    stalled: bool,
}

impl Document {
    fn into_save(self) -> Save {
        Save {
            written_by: self.written_by.map(|w| (w.app_version, w.git)),
            saved_at_utc: self.saved_at_utc,
            note: self.note,
            state_hash: self.state_hash,
            hole: Hole {
                m: self.sim.metric.m.0,
                a: self.sim.metric.a.0,
                m_solar: self.sim.metric.m_solar.0,
            },
            clock: self.sim.clock.0,
            alice: self.sim.alice.map(Observer::into_saved),
            bob: self.sim.bob.map(Observer::into_saved),
        }
    }
}

impl Observer {
    fn into_saved(self) -> SavedObserver {
        SavedObserver {
            name: self.name,
            mode: self.mode,
            t: self.t.0,
            r: self.r.0,
            phi: self.phi.0,
            tau: self.tau.0,
            geodesic: self.geodesic.map(|g| SavedGeodesic {
                t: g.t.0,
                r: g.r.0,
                phi: g.phi.0,
                tau: g.tau.0,
                energy: g.energy.0,
                l_ang: g.l_ang.0,
                u: g.u.map(|n| n.0),
                stalled: g.stalled,
            }),
            trail: self.trail.into_iter().map(Point::into_point).collect(),
            start: self.start.into_point(),
            release_t: self.release_t.0,
            release: self.release,
            is_active: self.is_active,
        }
    }
}

impl Point {
    fn into_point(self) -> TrailPoint {
        TrailPoint {
            t: self.t.0,
            r: self.r.0,
            phi: self.phi.0,
            tau: self.tau.0,
            u: self.u.map(|n| n.0),
            stalled: self.stalled,
        }
    }
}

/// One f64 as the save spells it: a JSON number where JSON can hold the value, and the strings
/// "inf", "-inf" and "nan" where it cannot. The app's `v1::Num` reads the same spellings and a few
/// hand-edited ones ("Infinity", a quoted number); this accepts all of them too, so that anything
/// the app opens, this opens.
#[derive(Debug, Clone, Copy)]
struct Num(f64);

impl<'de> Deserialize<'de> for Num {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct NumVisitor;

        impl Visitor<'_> for NumVisitor {
            type Value = Num;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a number, or one of the strings \"inf\", \"-inf\", \"nan\"")
            }

            fn visit_f64<E: de::Error>(self, value: f64) -> Result<Num, E> {
                Ok(Num(value))
            }

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
                    other => other.parse::<f64>().map(Num).map_err(|_| {
                        E::custom(format!("{other:?} is not a number a save can hold"))
                    }),
                }
            }
        }

        deserializer.deserialize_any(NumVisitor)
    }
}

#[cfg(test)]
mod tests;
