//! Every way reading or writing a bundle can fail, each with a sentence a person can act on.

use std::path::PathBuf;

use crate::VERSION;

/// Why a bundle, a manifest or a frame was refused.
///
/// Each variant is a different thing the person running the program has to do about it, which is
/// why a truncated frame and a damaged one are not the same variant even though both end in
/// "write the frame again": a truncated frame is the ordinary result of a crash and a resumed run
/// repairs it, while a checksum mismatch in a file that is the right length means something other
/// than this library touched the file. `Display` is a whole sentence, without a capital or a full
/// stop so that a caller can prefix it or embed it.
#[derive(Debug)]
pub enum Error {
    /// The operating system refused to read, write, create or rename a file. `action` is the
    /// verb, so that the sentence says which.
    Io {
        action: &'static str,
        path: PathBuf,
        source: std::io::Error,
    },
    /// The path is not a sky bundle at all: not a directory, no manifest in it, or a manifest that
    /// names some other format.
    NotABundle { path: PathBuf, why: String },
    /// `manifest.json` is there but is not JSON.
    ManifestUnreadable { path: PathBuf, why: String },
    /// The manifest is a sky manifest from a format version this build does not know.
    NewerManifest { version: u64 },
    /// The manifest parsed and is of a version this build reads, but breaks a rule of the format:
    /// a missing field, a grid convention version 1 does not define, frames out of order.
    InvalidManifest { why: String },
    /// `BundleWriter::create` was pointed at a directory that already holds a bundle.
    AlreadyABundle { path: PathBuf },
    /// The file does not start with the frame signature.
    NotAFrame { path: PathBuf },
    /// A frame file from a format version this build does not know.
    NewerFrame { path: PathBuf, version: u32 },
    /// The file ends before the header or a chunk that it declares.
    Truncated {
        path: PathBuf,
        needed: u64,
        found: u64,
    },
    /// The file is the right length for what it declares but its contents are wrong: a chunk of
    /// the wrong size, a deflate stream that will not decompress, a required chunk missing.
    Corrupt { path: PathBuf, why: String },
    /// A chunk's payload does not have the CRC-32 its header records.
    CrcMismatch {
        path: PathBuf,
        tag: String,
        stored: u32,
        computed: u32,
    },
    /// The frame's grid is not the grid the manifest declares.
    DimensionMismatch {
        path: PathBuf,
        found: (u32, u32),
        expected: (u32, u32),
    },
    /// The frame's header names a different frame from the one its file name does.
    IndexMismatch {
        path: PathBuf,
        found: u32,
        expected: u32,
    },
    /// A frame the manifest does not list as written was asked for.
    FrameNotWritten { index: u32 },
    /// A frame, or its manifest entry, handed to the writer is inconsistent with itself or with
    /// the bundle, and nothing has been written.
    InvalidFrame { why: String },
}

/// Which versions this build reads, as the end of a sentence.
fn readable() -> String {
    if VERSION == 1 {
        "this build reads only version 1".into()
    } else {
        format!("this build reads versions 1 to {VERSION}")
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io {
                action,
                path,
                source,
            } => write!(f, "could not {action} {}: {source}", path.display()),
            Self::NotABundle { path, why } => {
                write!(f, "{} is not a sky bundle: {why}", path.display())
            }
            Self::ManifestUnreadable { path, why } => write!(
                f,
                "the manifest {} is not readable as JSON ({why}); it is probably truncated, or was \
                 not written by a sky-bundle writer",
                path.display()
            ),
            Self::NewerManifest { version } => write!(
                f,
                "this bundle was written in version {version} of the sky format and {}; read it \
                 with a newer build",
                readable()
            ),
            Self::InvalidManifest { why } => write!(
                f,
                "the bundle's manifest breaks a rule of the sky format, so the bundle has not been \
                 opened: {why}"
            ),
            Self::AlreadyABundle { path } => write!(
                f,
                "{} already holds a sky bundle; resume it, or choose an empty directory, rather \
                 than write a new bundle over it",
                path.display()
            ),
            Self::NotAFrame { path } => write!(
                f,
                "{} is not a sky frame file: it does not start with the frame signature",
                path.display()
            ),
            Self::NewerFrame { path, version } => write!(
                f,
                "{} was written in version {version} of the sky format and {}; read it with a \
                 newer build",
                path.display(),
                readable()
            ),
            Self::Truncated {
                path,
                needed,
                found,
            } => write!(
                f,
                "{} is truncated: it needs at least {needed} bytes and has {found}; the frame was \
                 not finished, and a resumed run writes it again",
                path.display()
            ),
            Self::Corrupt { path, why } => write!(
                f,
                "{} is damaged: {why}; write the frame again",
                path.display()
            ),
            Self::CrcMismatch {
                path,
                tag,
                stored,
                computed,
            } => write!(
                f,
                "{} is damaged: its {tag} chunk has the checksum {computed:08x} and the file \
                 records {stored:08x}; the file was changed after it was written, so write the \
                 frame again",
                path.display()
            ),
            Self::DimensionMismatch {
                path,
                found,
                expected,
            } => write!(
                f,
                "{} holds a grid of {} x {} rays but the manifest declares {} x {}; the frame \
                 belongs to a different run",
                path.display(),
                found.0,
                found.1,
                expected.0,
                expected.1
            ),
            Self::IndexMismatch {
                path,
                found,
                expected,
            } => write!(
                f,
                "{} says it is frame {found} but is filed as frame {expected}; the file was \
                 renamed or copied from another place in the run",
                path.display()
            ),
            Self::FrameNotWritten { index } => write!(
                f,
                "frame {index} is not in this bundle yet: the manifest does not list it as written"
            ),
            Self::InvalidFrame { why } => {
                write!(f, "this frame has not been written: {why}")
            }
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}
