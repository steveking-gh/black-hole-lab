//! The sky bundle: what an observer near a black hole sees of the distant sky, frame by frame.
//!
//! A batch program traces light rays backward from the observer's eye and, for each moment of the
//! observer's watch and each direction on the observer's sky, records where on the distant
//! celestial sphere that light came from, how its frequency was shifted, and whether it came from
//! the sky at all. A renderer turns that record into a 360-degree video. The bundle is the record,
//! and this crate is the only code the two programs share: it reads and writes bundles and
//! implements the grid formulae, and computes no physics.
//!
//! A bundle is a directory:
//!
//!     manifest.json            what the run is, and which frames are written
//!     frames/000000.skyframe   one file per frame: planes of fate, direction, shift, winding
//!     frames/000001.skyframe
//!     ...
//!
//! The normative definition is `physics_simulation_specification.md` at the root of the
//! workspace. Where this code and that document disagree, the document is right and this is a
//! bug.
//!
//! - [`BundleWriter`] creates a bundle, writes each frame atomically and rewrites the manifest;
//!   [`BundleWriter::resume`] picks up a bundle a crashed run left.
//! - [`BundleReader`] opens and checks a bundle and reads frames by index.
//! - [`Grid`] turns a pixel into the direction the observer looks along and back.
//! - [`Error`] is every refusal, each a sentence a person can act on.

mod bundle;
mod error;
mod frame;
mod grid;
mod manifest;
mod num;

pub use bundle::{BundleReader, BundleWriter, FRAMES, MANIFEST, TEMPORARY_SUFFIX};
pub use error::Error;
pub use frame::{CHUNK_HEADER_LEN, Codec, Frame, HEADER_LEN, MAGIC, PointSource, crc32, fate, tag};
pub use grid::Grid;
pub use manifest::{
    Axes, FarSky, FrameEntry, Geometry, GridSpec, HEADING_ZERO, Label, Manifest, NO_LABEL,
    Observer, PIXEL_CENTRES, POLE, PROJECTION, Playback, Position, ReadoutDecl, STOPWATCH, Source,
    TimeUnit, WriterInfo, frame_file_name,
};
pub use num::Num;

/// The value of a manifest's `format` field. A manifest that says anything else is not a sky
/// bundle's, whatever its directory is called.
pub const FORMAT: &str = "black-hole-lab-sky";

/// The format version this build writes, and the newest it reads. It rises only for a change of
/// structure or meaning; an added optional field or chunk is not a new version.
pub const VERSION: u32 = 1;

#[cfg(test)]
mod tests;
