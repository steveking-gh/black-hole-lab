//! `manifest.json`: everything about a bundle that is not a plane of rays.
//!
//! The manifest is read in two passes. The first parses the text as untyped JSON and looks only at
//! `format` and `version`, so that a manifest from a newer version is refused as newer - with a
//! sentence that says so - rather than as a schema error about whichever field changed shape. The
//! second deserialises into the types below and checks the rules the types cannot state.
//!
//! Nothing here carries `deny_unknown_fields`. A field added in a later build of version 1 is
//! ignored by this one, which is the compatibility rule: an added optional field is not a new
//! version.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{Error, FORMAT, Grid, Num, VERSION};

/// The value of `grid.projection`: the only projection version 1 defines.
pub const PROJECTION: &str = "equirectangular";
/// The value of `grid.pole`: the top of the frame is the observer's +z.
pub const POLE: &str = "observer-z";
/// The value of `grid.heading_zero`: the centre of the frame is the observer's +x.
pub const HEADING_ZERO: &str = "observer-x";
/// The value of `grid.pixel_centres`: sample (i, j) sits at (i + 0.5, j + 0.5) of the frame.
pub const PIXEL_CENTRES: &str = "half-integer";
/// The id the first read-out must have.
pub const STOPWATCH: &str = "stopwatch";
/// The label a point source carries when it has none. No declared label may use it.
pub const NO_LABEL: u32 = u32::MAX;
/// How far a mark's direction may be from unit length before a manifest is refused: loose enough
/// for a vector written to seven figures, tight enough that a vector nobody normalised fails.
pub const MARK_TOLERANCE: f64 = 1e-6;

/// How far the far-sky axes may stray from an orthonormal right-handed set before a manifest is
/// refused. The published galactic matrix is orthonormal to about 1e-10; this is loose enough
/// for any honest rounding and tight enough that a transposed or mistyped matrix fails.
const AXES_TOLERANCE: f64 = 1e-9;

/// The whole of `manifest.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    /// Always [`FORMAT`]. `BundleWriter::create` sets it.
    pub format: String,
    /// The format version the bundle was written in. `BundleWriter::create` sets it to
    /// [`VERSION`].
    pub version: u32,
    pub writer: WriterInfo,
    pub source: Source,
    pub geometry: Geometry,
    pub time_unit: TimeUnit,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observer: Option<Observer>,
    pub grid: GridSpec,
    pub far_sky: FarSky,
    pub playback: Playback,
    /// The read-outs, each declared once. The first is always the stopwatch.
    pub readouts: Vec<ReadoutDecl>,
    /// The names point sources can carry, by id. Empty when there are no point sources.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub labels: Vec<Label>,
    /// The marks, each declared once: directions on the observer's sky that the renderer draws a
    /// sign at (section 12). Empty when there are none.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub marks: Vec<MarkDecl>,
    /// How many frames the run means to write, for a progress bar. Not a promise.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frames_planned: Option<u32>,
    /// The frames written so far, in increasing order of index.
    pub frames: Vec<FrameEntry>,
}

/// The program that wrote the bundle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WriterInfo {
    pub program: String,
    pub version: String,
    /// The source revision the writer was built from, when it knows.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,
}

/// What the bundle is a picture of.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Source {
    /// `"scenario"` for a run of Black Hole Lab, `"flat-space test"` for a generated test case.
    pub kind: String,
    /// The scenario's or the test case's name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// The state hash of the `.bhl` save the run started from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub state_hash: Option<String>,
}

/// The spacetime the rays were traced through. A renderer needs none of this; it is here so that
/// a bundle says what it is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Geometry {
    /// `"flat"` or `"kerr"`.
    pub kind: String,
    /// M, in the bundle's time unit (c = G = 1, so a length is its light-travel time). Present
    /// for Kerr.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mass: Option<Num>,
    /// a = J / M, in the bundle's time unit, never negative: the far-sky Z axis points along the
    /// hole's angular momentum. Present for Kerr.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spin: Option<Num>,
}

/// The one unit every time in the bundle is expressed in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeUnit {
    /// What a person calls it: `"M"`, `"s"`.
    pub name: String,
    /// How many SI seconds one unit is: GM/c^3 for M, 1 for the second.
    pub seconds: Num,
}

/// Who is looking, and how their triad was built. Descriptive only.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Observer {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// A sentence saying how the writer built the observer's triad: which tetrad it started from
    /// and how it was boosted to the observer's velocity.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub triad: Option<String>,
}

/// The observer-sky grid as the manifest states it. Four of the six fields have one legal value
/// in version 1; they are written out so that a person reading the manifest does not need the
/// specification to know how the frames are laid out.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GridSpec {
    pub projection: String,
    pub width: u32,
    pub height: u32,
    pub pole: String,
    pub heading_zero: String,
    pub pixel_centres: String,
}

impl GridSpec {
    /// The version-1 grid of this size.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            projection: PROJECTION.into(),
            width,
            height,
            pole: POLE.into(),
            heading_zero: HEADING_ZERO.into(),
            pixel_centres: PIXEL_CENTRES.into(),
        }
    }

    pub fn grid(&self) -> Grid {
        Grid::new(self.width, self.height)
    }
}

/// The far-sky frame's three axes, each written as a unit vector in ICRS.
///
/// Listing the axes rather than a matrix leaves nothing to guess: `x` is where the far-sky X axis
/// points on the real sky, and a direction d = (dX, dY, dZ) in the far-sky frame is
/// dX x + dY y + dZ z in ICRS.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FarSky {
    /// A name for the orientation: `"galactic"` for the preset.
    pub name: String,
    pub axes_in_icrs: Axes,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Axes {
    pub x: [Num; 3],
    pub y: [Num; 3],
    pub z: [Num; 3],
}

impl FarSky {
    /// The default orientation: the Milky Way on the frame's equator. X points at the galactic
    /// centre, Z at the north galactic pole, Y at galactic longitude 90 degrees.
    ///
    /// The numbers are the rows of the Hipparcos matrix A_G (ESA 1997, The Hipparcos and Tycho
    /// Catalogues, vol. 1, section 1.5.3), which takes an ICRS vector to galactic coordinates; its
    /// rows are therefore the galactic axes written in ICRS. The catalogue prints ten decimals;
    /// these sixteen are the matrix built from its defining angles (north galactic pole at RA
    /// 192.85948 deg, Dec +27.12825 deg; the ascending node at l = 32.93192 deg), which they match
    /// to within 3e-16.
    pub fn galactic() -> Self {
        let n = |v: [f64; 3]| v.map(Num);
        Self {
            name: "galactic".into(),
            axes_in_icrs: Axes {
                // Two of the values as usually quoted end in a digit that does not change the
                // f64, and are written here without it: -0.8734370902348850 and
                // 0.7469822444972189.
                x: n([-0.0548755604162154, -0.873437090234885, -0.4838350155487132]),
                y: n([0.4941094278755837, -0.4448296299600112, 0.746982244497219]),
                z: n([-0.8676661490190047, -0.1980763734312015, 0.4559837761750669]),
            },
        }
    }

    /// A far-sky direction written in ICRS.
    pub fn to_icrs(&self, d: [f64; 3]) -> [f64; 3] {
        let Axes { x, y, z } = &self.axes_in_icrs;
        std::array::from_fn(|k| d[0] * x[k].0 + d[1] * y[k].0 + d[2] * z[k].0)
    }
}

/// How the frames map onto a video.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Playback {
    /// Frames per second of the video the renderer makes.
    pub frames_per_second: Num,
    /// How much of the observer's proper time, in the bundle's time unit, one second of video
    /// shows.
    pub proper_time_per_video_second: Num,
}

/// One read-out, declared once for the whole bundle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReadoutDecl {
    /// The key each frame's `readouts` object uses.
    pub id: String,
    /// What the renderer writes beside the value.
    pub label: String,
    /// The unit the renderer writes after the value; empty for a pure number.
    pub unit: String,
    /// How many decimals the renderer should show.
    pub decimals: u32,
    /// The read-out in a unit a person would rather read, when the writer offers one (section
    /// 6.1). The values the frames carry stay in `unit` either way.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub display: Option<Display>,
}

/// A read-out as a renderer may show it instead of as it is stored: the stored value times
/// `scale`, followed by `unit`, to `decimals` places.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Display {
    pub unit: String,
    /// How many of `unit` one of the declaration's own unit is. Positive and finite.
    pub scale: Num,
    pub decimals: u32,
}

/// One mark, declared once for the whole bundle: a direction on the observer's sky that the
/// renderer draws a sign at, such as the direction the observer is travelling in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkDecl {
    /// The key each frame's `marks` object uses.
    pub id: String,
    /// What the mark shows, in words.
    pub label: String,
    /// The sign the renderer draws: `"ring"`, `"diamond"` or `"triangle"`. A renderer draws a
    /// shape it does not know as a ring.
    pub shape: String,
}

/// A name a point source can carry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Label {
    pub id: u32,
    pub text: String,
}

/// The manifest's record of one written frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameEntry {
    pub index: u32,
    /// The observer's proper time at this frame, in the bundle's time unit, from whatever origin
    /// the writer uses. The stopwatch read-out is this less the first frame's.
    pub proper_time: Num,
    /// Where the observer is, when the writer says.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position: Option<Position>,
    /// Always `frames/NNNNNN.skyframe` for this index; `BundleWriter::write_frame` sets it.
    pub file: String,
    /// The frame file's length; `BundleWriter::write_frame` sets it.
    pub bytes: u64,
    /// The value of each declared read-out at this frame, by id. A read-out missing here is not
    /// shown on this frame.
    pub readouts: BTreeMap<String, Num>,
    /// Where the observer looks to see each declared mark at this frame, by id: a unit vector in
    /// the triad, like a pixel's n. A mark missing here is not drawn on this frame.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub marks: BTreeMap<String, [Num; 3]>,
}

impl FrameEntry {
    /// An entry with no position and no read-outs, ready for `BundleWriter::write_frame` to fill
    /// in the file name and size.
    pub fn new(index: u32, proper_time: f64) -> Self {
        Self {
            index,
            proper_time: Num(proper_time),
            position: None,
            file: frame_file_name(index),
            bytes: 0,
            readouts: BTreeMap::new(),
            marks: BTreeMap::new(),
        }
    }

    /// Sets a read-out's value, for chaining.
    pub fn with_readout(mut self, id: &str, value: f64) -> Self {
        self.readouts.insert(id.into(), Num(value));
        self
    }

    /// Sets the direction the observer looks in to see a mark, for chaining.
    pub fn with_mark(mut self, id: &str, n: [f64; 3]) -> Self {
        self.marks.insert(id.into(), n.map(Num));
        self
    }
}

/// The observer's event: four coordinates in a named chart.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Position {
    /// `"cartesian"` for (t, x, y, z) along the far-sky axes, `"kerr-schild"` for ingoing
    /// Kerr-Schild (t, r, theta, phi).
    pub chart: String,
    pub coords: [Num; 4],
}

/// The file name, relative to the bundle, of frame `index`: six digits, zero-padded, and more
/// digits from a million frames on.
pub fn frame_file_name(index: u32) -> String {
    format!("frames/{index:06}.skyframe")
}

impl Manifest {
    /// The manifest as the pretty-printed JSON a bundle stores.
    pub fn to_json(&self) -> String {
        // Serialising these types cannot fail: every map key is a string and every number is a
        // `Num`, which never hands serde_json a non-finite f64.
        serde_json::to_string_pretty(self).expect("a manifest always serialises")
    }

    /// Reads and checks a manifest.
    pub fn from_json(text: &str) -> Result<Self, Error> {
        Self::parse(text, Path::new("manifest.json"))
    }

    /// [`Manifest::from_json`], naming the file it came from in any refusal.
    pub(crate) fn parse(text: &str, path: &Path) -> Result<Self, Error> {
        let value: serde_json::Value =
            serde_json::from_str(text).map_err(|e| Error::ManifestUnreadable {
                path: path.to_path_buf(),
                why: e.to_string(),
            })?;
        let not_a_bundle = |why: String| Error::NotABundle {
            // The bundle is the manifest's directory, when the manifest came from one.
            path: path
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(path)
                .to_path_buf(),
            why,
        };
        match value.get("format").and_then(serde_json::Value::as_str) {
            Some(FORMAT) => {}
            Some(other) => {
                return Err(not_a_bundle(format!(
                    "its manifest calls itself {other:?} and a sky bundle calls itself {FORMAT:?}"
                )));
            }
            None => {
                return Err(not_a_bundle(
                    "its manifest.json has no \"format\" field naming it a sky bundle".into(),
                ));
            }
        }
        let version = value
            .get("version")
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| Error::InvalidManifest {
                why: "it has no whole-number \"version\" field".into(),
            })?;
        if version > u64::from(VERSION) {
            return Err(Error::NewerManifest { version });
        }
        if version == 0 {
            return Err(Error::InvalidManifest {
                why: "it says it is version 0, and the first version is 1".into(),
            });
        }
        let manifest: Self = serde_json::from_value(value)
            .map_err(|e| Error::InvalidManifest { why: e.to_string() })?;
        manifest.validate()?;
        Ok(manifest)
    }

    /// Checks the rules of the format that the types do not enforce. A reader calls this on every
    /// manifest it opens and a writer on every manifest it writes, so that a bundle this library
    /// wrote is always one it can read.
    pub fn validate(&self) -> Result<(), Error> {
        let bad = |why: String| Err(Error::InvalidManifest { why });
        let g = &self.grid;
        for (field, found, wanted) in [
            ("projection", &g.projection, PROJECTION),
            ("pole", &g.pole, POLE),
            ("heading_zero", &g.heading_zero, HEADING_ZERO),
            ("pixel_centres", &g.pixel_centres, PIXEL_CENTRES),
        ] {
            if found != wanted {
                return bad(format!(
                    "the grid's {field} is {found:?}, and version 1 defines only {wanted:?}"
                ));
            }
        }
        if g.width == 0 || g.height == 0 {
            return bad(format!(
                "the grid is {} x {}, and a grid needs at least one ray",
                g.width, g.height
            ));
        }
        if !(self.time_unit.seconds.0.is_finite() && self.time_unit.seconds.0 > 0.0) {
            return bad(format!(
                "the time unit is {} seconds long, and it must be a positive number of seconds",
                self.time_unit.seconds.0
            ));
        }
        if self.geometry.kind == "kerr"
            && (self.geometry.mass.is_none() || self.geometry.spin.is_none())
        {
            return bad("a Kerr geometry needs both its mass and its spin".into());
        }
        self.validate_axes()?;
        let p = &self.playback;
        for (field, value) in [
            ("frames_per_second", p.frames_per_second.0),
            (
                "proper_time_per_video_second",
                p.proper_time_per_video_second.0,
            ),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return bad(format!(
                    "playback.{field} is {value}, and it must be a positive number"
                ));
            }
        }
        match self.readouts.first() {
            Some(first) if first.id == STOPWATCH => {}
            Some(first) => {
                return bad(format!(
                    "the first read-out is {:?}, and the first read-out is always {STOPWATCH:?}",
                    first.id
                ));
            }
            None => return bad(format!("it declares no read-outs, not even {STOPWATCH:?}")),
        }
        for (k, r) in self.readouts.iter().enumerate() {
            if self.readouts[..k].iter().any(|s| s.id == r.id) {
                return bad(format!("the read-out {:?} is declared twice", r.id));
            }
            if let Some(display) = &r.display
                && !(display.scale.0.is_finite() && display.scale.0 > 0.0)
            {
                return bad(format!(
                    "the read-out {:?} is displayed at a scale of {}, and a scale is a positive \
                     number",
                    r.id, display.scale.0
                ));
            }
        }
        for (k, m) in self.marks.iter().enumerate() {
            if self.marks[..k].iter().any(|n| n.id == m.id) {
                return bad(format!("the mark {:?} is declared twice", m.id));
            }
        }
        for (k, l) in self.labels.iter().enumerate() {
            if l.id == NO_LABEL {
                return bad(format!(
                    "the label {:?} has the id {NO_LABEL}, which means no label",
                    l.text
                ));
            }
            if self.labels[..k].iter().any(|m| m.id == l.id) {
                return bad(format!("the label id {} is declared twice", l.id));
            }
        }
        for (k, entry) in self.frames.iter().enumerate() {
            self.check_entry(entry)?;
            if let Some(before) = k.checked_sub(1).map(|b| &self.frames[b]) {
                check_order(before, entry)?;
            }
        }
        Ok(())
    }

    /// The rules one frame entry must meet on its own.
    pub(crate) fn check_entry(&self, entry: &FrameEntry) -> Result<(), Error> {
        let bad = |why: String| Err(Error::InvalidManifest { why });
        let wanted = frame_file_name(entry.index);
        if entry.file != wanted {
            return bad(format!(
                "frame {} is filed as {:?}, and its file is always {wanted:?}",
                entry.index, entry.file
            ));
        }
        if !entry.proper_time.0.is_finite() {
            return bad(format!(
                "frame {} has the proper time {}, and a proper time is a finite number",
                entry.index, entry.proper_time.0
            ));
        }
        for id in entry.readouts.keys() {
            if !self.readouts.iter().any(|r| &r.id == id) {
                return bad(format!(
                    "frame {} has a value for the read-out {id:?}, which is not declared",
                    entry.index
                ));
            }
        }
        for (id, n) in &entry.marks {
            if !self.marks.iter().any(|m| &m.id == id) {
                return bad(format!(
                    "frame {} has a direction for the mark {id:?}, which is not declared",
                    entry.index
                ));
            }
            let length = n.iter().map(|c| c.0 * c.0).sum::<f64>().sqrt();
            // Written as a negation so that a NaN, which passes no comparison, is refused too.
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !((length - 1.0).abs() <= MARK_TOLERANCE) {
                return bad(format!(
                    "frame {} gives the mark {id:?} a direction of length {length}, and a \
                     direction is a unit vector",
                    entry.index
                ));
            }
        }
        Ok(())
    }

    fn validate_axes(&self) -> Result<(), Error> {
        let Axes { x, y, z } = &self.far_sky.axes_in_icrs;
        let [x, y, z] = [x, y, z].map(|v| v.map(|c| c.0));
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        // Unit lengths and right angles cover orthonormality; x cross y = z then settles the
        // handedness, which a mirrored star map would otherwise get past both checks.
        let worst = [
            dot(x, x) - 1.0,
            dot(y, y) - 1.0,
            dot(z, z) - 1.0,
            dot(x, y),
            dot(y, z),
            dot(z, x),
            dot(cross(x, y), z) - 1.0,
        ]
        .into_iter()
        .map(f64::abs)
        // Not `f64::max`, which drops a NaN: a NaN anywhere in the axes has to reach the test
        // below and be refused.
        .fold(0.0, |worst, miss| {
            if miss.is_nan() || miss > worst {
                miss
            } else {
                worst
            }
        });
        if worst.is_nan() || worst > AXES_TOLERANCE {
            return Err(Error::InvalidManifest {
                why: format!(
                    "the far-sky axes are not an orthonormal right-handed set (they miss by {worst:e})"
                ),
            });
        }
        Ok(())
    }
}

/// Two consecutive entries of the frame list must rise in index and in proper time.
pub(crate) fn check_order(before: &FrameEntry, after: &FrameEntry) -> Result<(), Error> {
    if after.index <= before.index {
        return Err(Error::InvalidManifest {
            why: format!(
                "frame {} is listed after frame {}, and frames are listed in increasing order",
                after.index, before.index
            ),
        });
    }
    if after.proper_time.0 <= before.proper_time.0 {
        return Err(Error::InvalidManifest {
            why: format!(
                "frame {} is at proper time {} and frame {} before it at {}; proper time rises \
                 with the frame index",
                after.index, after.proper_time.0, before.index, before.proper_time.0
            ),
        });
    }
    Ok(())
}
