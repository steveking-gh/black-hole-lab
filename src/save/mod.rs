//! Saving a run and putting it back: the file format, what is in it, and how to move it forward.
//!
//! A save is a **snapshot, not a recipe**. It would be smaller to write down the setup - the two
//! cards, the geometry, the sliders - and replay it, and it would be wrong: how many steps a run has
//! taken depends on how long each frame took to draw, and where an observer stands can depend on a
//! marker having been dragged there. Two windows given the same setup do not reach the same state,
//! so the state itself is what is written: the clock, both worldlines with their whole trails, both
//! transmissions with every ray of every wavefront and every arrival recorded, the panel's settings
//! and what each canvas is looking at.
//!
//! # The file
//!
//! JSON, gzipped, extension `.bhl`. The reader sniffs the two gzip magic bytes and falls through to
//! plain JSON if they are absent, which keeps a save something a person can open in an editor and
//! lets the golden file in `golden/` be readable text in the repository.
//!
//! The document is one object:
//!
//! ```text
//! {
//!   "format": "black-hole-lab-save",
//!   "version": 1,
//!   "written_by": { "app_version": "0.1.0", "git": "e40cc99" },
//!   "saved_at_utc": "2026-09-18T11:04:07Z",
//!   "note": "",
//!   "state_hash": "9f3c1d...",
//!   "sim":      { ... },
//!   "controls": { ... },
//!   "view":     { ... }
//! }
//! ```
//!
//! `format` and `version` are read first, out of the same bytes, before anything else is parsed: a
//! file that is not one of ours, or one from a format this build has never heard of, is refused
//! with a sentence rather than with a parse error about a missing field. Parsing the bytes twice
//! costs a few milliseconds on a file this size and is what makes the refusal legible.
//!
//! `state_hash` is `Simulation::fingerprint` of the run, in hex. It is recomputed after the run has
//! been rebuilt and compared: a mismatch means the file was corrupted or edited, or that some number
//! in it did not come back as the number that went in, and the load is refused. It is the reason
//! this format can claim that a restored run is the run that was saved rather than one that looks
//! like it.
//!
//! # Numbers
//!
//! Every f64 in the schema goes through `v1::Num`, which writes a finite value as a JSON number and
//! the other three as the strings `"inf"`, `"-inf"` and `"nan"`. This state really does contain
//! non-finite values - see that type - and JSON has no spelling for them. Finite values round-trip
//! to the bit, which is why the crate takes serde_json's `float_roundtrip` feature.
//!
//! # What is *not* in a file
//!
//! * The panel's transients: which transport button flashed, what the last played frame achieved on
//!   the watch, and the standing request for the view to go back to the start. None of them is a
//!   setting.
//! * `is_playing`. **A load always comes up paused.** A run that started moving before the user had
//!   looked at it would be a state they could not get back.
//! * The canvases' caches and gestures: the volume view's integrated past cone, a marker drag in
//!   progress, one frame of telemetry-box hysteresis. All three rebuild themselves.
//!
//! # Compatibility, and how to add version 2
//!
//! The promise is one-way: **build N reads every file of version <= N**, and nothing is claimed
//! about an older build reading a newer file. That is what makes the rule below workable.
//!
//! *An added field is not a new version.* Nothing in the schema carries `deny_unknown_fields`, so a
//! file with a field this build has never heard of loads and the field is ignored. A new optional
//! field is therefore added to `v1` with `#[serde(default)]` and the version stays at 1. Older
//! builds ignore it; this one uses it where it is there.
//!
//! `Controls::step_grain` is the one such field so far: the Step Size dropdown counts a press in
//! played frames, and a file written before that dropdown existed carries no grain and loads at the
//! panel's default of one frame. Its two neighbours `step_size` and `step_distance_km` are what the
//! same change left behind. A press has no amount of its own any more, so this build writes the
//! amount a press comes to into both of them and reads neither back - which keeps a file readable
//! by a build from before the dropdown, stepping by about the same amount, without letting two
//! spellings of the same setting disagree on the way in.
//!
//! `Telemetry::collapsed` is the second: the disclosure triangle on a box's title line shuts that
//! box down to the line, and the list names every box left shut, under the same slugs `placements`
//! uses. A file written before the triangle existed carries no list and opens with every box open,
//! which is the state that file was saved in, and a slug this build has never heard of is dropped
//! from the list rather than refused, exactly as an unknown placement is.
//!
//! *A changed field is a new version.* A field whose units change, whose meaning changes, that
//! splits in two, or that goes away, is a structural change, and a reader has to be told which
//! spelling it is looking at. Then:
//!
//! 1. `v1.rs` is **frozen**. Not edited, not tidied, not renamed. It is the description of files
//!    that already exist on disk.
//! 2. Add `v2.rs`, the new schema, and `fn upgrade(old: v1::Save) -> v2::Save` beside it.
//! 3. Raise `VERSION` to 2, point `convert.rs` at `v2`, and add the arm to `read_document` below:
//!    version 1 parses as `v1::Save` and is upgraded, version 2 parses as `v2::Save`, anything
//!    higher is the "saved by a newer version" refusal.
//! 4. Leave `golden/v1.json` exactly as it is and add `golden/v2.json` beside it. The old golden is
//!    the test that the upgrade path still works; regenerating it would delete the only evidence
//!    that a version-1 file can still be opened.
//!
//! With one version in existence there is no chain to walk and no machinery for walking one. The
//! dispatch is a `match` on an integer and the procedure above is the design; building the general
//! case before there is a second version would be building it against a guess.

pub(crate) mod convert;
pub(crate) mod v1;

use std::io::{Read, Write};
use std::path::Path;

use crate::gui::controls::AppControls;
use crate::gui::spacetime_canvas::SpacetimeCanvas;
use crate::gui::spatial_canvas::SpatialCanvas;
use crate::gui::volume_canvas::VolumeCanvas;
use crate::physics::simulation::Simulation;

/// The string every file of this format carries. A file whose `format` is anything else is not one
/// of ours whatever its extension says, and is refused before a single field of it is read.
pub const FORMAT: &str = "black-hole-lab-save";

/// The newest schema version this build writes, and the highest it can read.
pub const VERSION: u32 = 1;

/// The extension a save is written with.
pub const EXTENSION: &str = "bhl";

/// Everything that can go wrong reading or writing a save, with a message fit to put in front of a
/// user.
///
/// Nothing here panics and nothing here is a `Box<dyn Error>`: each arm is a thing that actually
/// happens to a file - the wrong file was chosen, a download was truncated, a disk was full, a
/// number was edited by hand - and each of them has a different sentence to say about it.
#[derive(Debug)]
pub enum Error {
    /// The file could not be read or written at all. It carries no path: every caller has one in
    /// hand and prints it, and an error that named the file as well would say it twice.
    Io(std::io::Error),
    /// The gzip wrapper would not come apart: a truncated download, or bytes that only start like a
    /// gzip stream.
    Gzip(String),
    /// The JSON inside would not parse.
    Json(String),
    /// The document parsed but is not a save of this program.
    NotASave { found: String },
    /// The document is a save of this program, from a format this build does not know.
    Newer { version: u32 },
    /// The run in the file is not a run the physics can reach: `Simulation::check_invariants` said
    /// so, and it is refused rather than stepped.
    Invalid(String),
    /// The state hash does not match the run that was rebuilt from the file.
    HashMismatch { stored: String, found: String },
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(cause) => write!(f, "{cause}"),
            Self::Gzip(why) => write!(
                f,
                "this file starts like a compressed save but will not decompress ({why}); it is \
                 probably truncated"
            ),
            Self::Json(why) => write!(f, "this file is not readable as a save: {why}"),
            Self::NotASave { found } => write!(
                f,
                "this is not a Black Hole Lab save: it calls itself {found:?} and a save calls \
                 itself {FORMAT:?}"
            ),
            Self::Newer { version } => write!(
                f,
                "saved by a newer version of Black Hole Lab (format {version}); this build reads \
                 up to format {VERSION}"
            ),
            Self::Invalid(why) => write!(
                f,
                "the run in this file is not one the simulation can reach, so it has not been \
                 loaded: {why}"
            ),
            Self::HashMismatch { stored, found } => write!(
                f,
                "this file has been altered since it was written: it says the run hashes to \
                 {stored} and the run it describes hashes to {found}"
            ),
        }
    }
}

impl std::error::Error for Error {}

/// A run, a panel and three views read off a file, checked, and ready to be assigned.
///
/// It exists so that a load is *build, validate, swap*: everything below is constructed and checked
/// against a copy, and the running app is only written to once there is nothing left that can fail.
/// A load that goes wrong leaves the window exactly as it was.
pub struct Loaded {
    pub sim: Simulation,
    pub controls: AppControls,
    /// The canvases' state, still in schema form: the app owns the three canvases and lays this
    /// over them with `convert::apply_view_v1`, so that their caches and gestures are untouched.
    pub view: v1::View,
}

/// Who wrote a file and when, kept back from a document that a load has already consumed.
///
/// A load swallows the document: the run, the panel and the views are taken out of it and the rest
/// is dropped. These two fields are the rest, and they are the two the status line quotes, so
/// `SpacetimeApp::load_from` hands them back rather than making its caller parse the file a second
/// time to find out what it had just opened.
#[derive(Debug)]
pub struct Provenance {
    /// The `app_version` of the build that wrote the file.
    pub app_version: String,
    /// When the file was written, as the document spells it: `2026-09-18T11:04:07Z`.
    pub saved_at_utc: String,
}

impl Provenance {
    /// The date alone, for a status line with no room for the hour. The instant is ISO 8601 with
    /// the date first, so the date is its first ten characters; anything shorter than that is not
    /// one of ours and is quoted whole rather than sliced into.
    pub fn date(&self) -> &str {
        self.saved_at_utc.get(..10).unwrap_or(&self.saved_at_utc)
    }
}

/// The whole state of a run as a document, ready to be written.
pub fn document(
    sim: &Simulation,
    controls: &AppControls,
    spacetime: &SpacetimeCanvas,
    spatial: &SpatialCanvas,
    volume: &VolumeCanvas,
    note: &str,
) -> v1::Save {
    v1::Save {
        format: FORMAT.to_string(),
        version: VERSION,
        written_by: v1::WrittenBy {
            app_version: crate::version::build_version().to_string(),
            git: crate::stamp::git_short_hash(),
        },
        saved_at_utc: crate::stamp::now_utc_iso(),
        note: note.to_string(),
        state_hash: format!("{:016x}", sim.fingerprint()),
        sim: convert::sim_to_v1(sim),
        controls: convert::controls_to_v1(controls, &sim.metric),
        view: convert::view_to_v1(spacetime, spatial, volume),
    }
}

/// The bytes of a document: JSON, gzipped.
///
/// Pretty-printed before it is compressed, because the whole reason to keep the format textual is
/// that somebody can look at it, and gzip removes the indentation's cost along with the repeated
/// keys. Measured on a steady-state field the pretty form compresses to within a few percent of the
/// compact one.
pub fn to_bytes(save: &v1::Save) -> Result<Vec<u8>, Error> {
    let json = serde_json::to_vec_pretty(save).map_err(|e| Error::Json(e.to_string()))?;
    let mut encoder =
        flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(&json).map_err(|e| Error::Gzip(e.to_string()))?;
    encoder.finish().map_err(|e| Error::Gzip(e.to_string()))
}

/// The JSON of a document, uncompressed and readable. What the golden file is written with.
#[allow(dead_code)] // the readable half of the writer: `--save-info` and the golden-file
// generator are its callers, and the app itself always writes the compressed form
pub fn to_json(save: &v1::Save) -> Result<String, Error> {
    serde_json::to_string_pretty(save).map_err(|e| Error::Json(e.to_string()))
}

/// The JSON text inside a file's bytes, decompressed if it needs to be.
///
/// The sniff is the two-byte gzip magic. Anything else is taken as text, which is what lets a save
/// be hand-edited, kept in a repository, or piped through `gunzip` by somebody who would rather.
fn text_of(bytes: &[u8]) -> Result<String, Error> {
    if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut text = String::new();
        flate2::read::GzDecoder::new(bytes)
            .read_to_string(&mut text)
            .map_err(|e| Error::Gzip(e.to_string()))?;
        return Ok(text);
    }
    String::from_utf8(bytes.to_vec()).map_err(|_| {
        Error::Json("it is neither gzip nor UTF-8 text, so it is not a save".to_string())
    })
}

/// Just the two fields that decide how the rest is read. Everything else is ignored here, which is
/// the point: a file from a format this build cannot parse still has to be able to say so.
#[derive(serde::Deserialize)]
struct Header {
    #[serde(default)]
    format: String,
    #[serde(default)]
    version: u32,
}

/// Read a document out of a file's bytes: sniff, decompress, read the header, then parse the rest
/// according to it.
///
/// The two-pass parse is deliberate. The header decides the schema, and a schema cannot be chosen
/// half way through parsing itself; going over the same text twice costs milliseconds on a file of
/// this size and turns "missing field `emitted_tau`" into "saved by a newer version of Black Hole
/// Lab".
pub fn read_document(bytes: &[u8]) -> Result<v1::Save, Error> {
    let text = text_of(bytes)?;
    let header: Header =
        serde_json::from_str(&text).map_err(|e| Error::Json(e.to_string()))?;
    if header.format != FORMAT {
        return Err(Error::NotASave { found: header.format });
    }
    match header.version {
        // One version exists, so the dispatch is one arm. See the module documentation for what a
        // second one looks like and why the chain is not built before there is a chain.
        1 => serde_json::from_str(&text).map_err(|e| Error::Json(e.to_string())),
        version => Err(Error::Newer { version }),
    }
}

/// Turn a document into a run: build it, check it, hash it.
///
/// `check_invariants` comes first because it is exactly what it was written for - a state assembled
/// by something other than a step, which is what a loader produces - and because everything after it
/// assumes a run the physics could have reached: a NaN clock or a ray standing inside the ring would
/// otherwise go straight into an integrator.
///
/// The state hash comes second, over the rebuilt run rather than over the text of the file. That is
/// the stronger statement of the two: it says not only that the bytes are intact but that every
/// number in them came back as the number that went in.
///
/// Nothing is primed. An earlier draft rebuilt each pulse's front mark here with
/// `SignalField::prime`, the way `SignalPair::step_back` does after a rewind, and the continuation
/// test refused it: a freshly taken mark folds its azimuth onto a different turn from the one the
/// uninterrupted run was measuring on, and the crossing algebra that cancels the difference exactly
/// in real arithmetic does not cancel it in floating point. The marks are in the file instead. See
/// `v1::FrontMark`.
pub fn rebuild(save: &v1::Save) -> Result<Loaded, Error> {
    let sim = convert::sim_from_v1(&save.sim);
    sim.check_invariants().map_err(Error::Invalid)?;
    let found = format!("{:016x}", sim.fingerprint());
    if found != save.state_hash {
        return Err(Error::HashMismatch { stored: save.state_hash.clone(), found });
    }
    Ok(Loaded {
        sim,
        controls: convert::controls_from_v1(&save.controls),
        view: save.view.clone(),
    })
}

/// Write bytes to `path` through a sibling temporary file and a rename.
///
/// A save is the only copy of a state somebody has, and writing it in place means a full disk or a
/// crash half way through leaves them with neither the old file nor the new one. The temporary
/// sits beside the target rather than in the system temp directory so that the rename is within one
/// filesystem and is therefore the atomic operation it is being relied on to be.
pub fn write_atomically(path: &Path, bytes: &[u8]) -> Result<(), Error> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(Error::Io)?;
    }
    let mut temporary = path.to_path_buf();
    temporary.as_mut_os_string().push(".writing");
    let write = || -> std::io::Result<()> {
        let mut file = std::fs::File::create(&temporary)?;
        file.write_all(bytes)?;
        file.sync_all()
    };
    if let Err(cause) = write() {
        let _ = std::fs::remove_file(&temporary);
        return Err(Error::Io(cause));
    }
    // The rename replaces an existing target on every platform this runs on - on Windows
    // `std::fs::rename` is MoveFileExW with MOVEFILE_REPLACE_EXISTING - so the old save is never
    // removed by hand first. Doing that would open exactly the window this function exists to
    // close: a crash between the remove and the rename would leave no save at all.
    std::fs::rename(&temporary, path).map_err(|cause| {
        let _ = std::fs::remove_file(&temporary);
        Error::Io(cause)
    })
}

/// Read a file's bytes.
pub fn read_file(path: &Path) -> Result<Vec<u8>, Error> {
    std::fs::read(path).map_err(Error::Io)
}

/// What `--save-info` prints: the envelope, the shape of the run, and what the file costs.
///
/// Headless and read-only. It exists because the first question about a directory of saves is which
/// one is which, and the second - the one that decides whether save points can be committed to the
/// repository - is how big they are.
pub fn describe(path: &Path) -> Result<String, Error> {
    let bytes = read_file(path)?;
    let text = text_of(&bytes)?;
    let save = read_document(&bytes)?;
    let mut out = String::new();
    let mut line = |s: String| {
        out.push_str(&s);
        out.push('\n');
    };
    line(format!("{}", path.display()));
    line(format!("  format          {} version {}", save.format, save.version));
    line(format!(
        "  written by      Black Hole Lab {} (git {})",
        save.written_by.app_version, save.written_by.git
    ));
    line(format!("  saved at        {}", save.saved_at_utc));
    line(format!("  note            {}", if save.note.is_empty() { "-" } else { &save.note }));
    line(format!("  state hash      {}", save.state_hash));
    line(format!(
        "  hole            M = {} M_sun = {:.4e} a/M = {:.6}",
        save.sim.metric.m.0,
        save.sim.metric.m_solar.0,
        save.sim.metric.a.0 / save.sim.metric.m.0
    ));
    line(format!("  clock           t = {} M", save.sim.clock.0));
    for (who, observer) in
        [("Alice", save.sim.alice.as_ref()), ("Bob", save.sim.bob.as_ref())]
    {
        match observer {
            Some(obs) => line(format!(
                "  {who:<15} t = {:.6} r = {:.6} phi = {:.6} tau = {:.6}, {} trail points",
                obs.t.0,
                obs.r.0,
                obs.phi.0,
                obs.tau.0,
                obs.trail.len()
            )),
            None => line(format!("  {who:<15} not in the run")),
        }
    }
    for (whose, field) in
        [("Alice's", &save.sim.alice_signal), ("Bob's", &save.sim.bob_signal)]
    {
        let rays: usize = field.pulses.iter().map(|p| p.rays.len()).sum();
        line(format!(
            "  {whose:<15} {} pulses, {rays} rays, {} arrivals recorded",
            field.pulses.len(),
            field.heard.len()
        ));
    }
    // Stated as "on disk" and "as JSON" rather than "compressed" and "uncompressed", because the
    // reader accepts a plain-text save as well and for one of those the two are the same number.
    line(format!(
        "  size            {} on disk, {} as JSON ({:.1}x)",
        human_bytes(bytes.len()),
        human_bytes(text.len()),
        text.len() as f64 / bytes.len().max(1) as f64
    ));
    Ok(out)
}

/// A byte count as somebody would say it, in powers of a thousand because that is what the prefix
/// on the unit means and because a file size is being quoted rather than an allocation.
pub(crate) fn human_bytes(bytes: usize) -> String {
    let n = bytes as f64;
    if n >= 1e6 {
        format!("{:.2} MB", n / 1e6)
    } else if n >= 1e3 {
        format!("{:.1} kB", n / 1e3)
    } else {
        format!("{bytes} B")
    }
}

#[cfg(test)]
mod tests;
