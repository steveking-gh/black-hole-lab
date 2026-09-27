//! A bundle on disk: the directory, its manifest and its frame files.
//!
//! The one rule that makes a bundle survive a crash is that no file is ever written in place.
//! Every file - a frame or the manifest - is written in full under a temporary name, flushed to
//! the disk, and renamed over its real name. A rename within one directory is atomic, so a file
//! under its real name is always a whole file, and a crash leaves at worst a stray `.tmp`.
//!
//! The manifest is the record of what is finished. A frame counts as written when the manifest
//! lists it and its file is there with the length the manifest records. A frame file the
//! manifest does not list - the run crashed after renaming the frame and before rewriting the
//! manifest - is an orphan: nothing reads it, and a resumed run writes it again.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use crate::frame::decode;
use crate::manifest::{check_order, frame_file_name};
use crate::{Error, FORMAT, Frame, FrameEntry, Manifest, VERSION};

/// The manifest's file name within a bundle.
pub const MANIFEST: &str = "manifest.json";
/// The directory of frame files within a bundle.
pub const FRAMES: &str = "frames";
/// What a file being written carries after its real name until it is renamed.
pub const TEMPORARY_SUFFIX: &str = ".tmp";

fn io<'a>(action: &'static str, path: &'a Path) -> impl FnOnce(std::io::Error) -> Error + 'a {
    move |source| Error::Io {
        action,
        path: path.to_path_buf(),
        source,
    }
}

/// Writes `bytes` to `path` so that `path` is only ever absent, its old contents, or `bytes`.
fn write_atomically(path: &Path, bytes: &[u8]) -> Result<(), Error> {
    let mut name = path.as_os_str().to_owned();
    name.push(TEMPORARY_SUFFIX);
    let temporary = PathBuf::from(name);
    let mut file = fs::File::create(&temporary).map_err(io("create", &temporary))?;
    file.write_all(bytes).map_err(io("write", &temporary))?;
    // Flushed before the rename, not after: otherwise a power cut can leave the rename on disk
    // and the contents not, which is a whole-length file of zeroes under the real name.
    file.sync_all().map_err(io("flush", &temporary))?;
    drop(file);
    // `fs::rename` replaces an existing file on Windows as well as on Unix.
    fs::rename(&temporary, path).map_err(io("rename", &temporary))
}

/// Reads a bundle's manifest, refusing a path that is not a bundle.
fn open_manifest(dir: &Path) -> Result<Manifest, Error> {
    let not_a_bundle = |why: &str| Error::NotABundle {
        path: dir.to_path_buf(),
        why: why.into(),
    };
    if !dir.exists() {
        return Err(not_a_bundle("there is nothing there"));
    }
    if !dir.is_dir() {
        return Err(not_a_bundle(
            "it is a file, and a bundle is a directory holding manifest.json and frames/",
        ));
    }
    let path = dir.join(MANIFEST);
    if !path.is_file() {
        return Err(not_a_bundle("it has no manifest.json"));
    }
    let text = fs::read_to_string(&path).map_err(io("read", &path))?;
    Manifest::parse(&text, &path)
}

/// Opens a bundle to read frames from it. The bundle may still be being written: `complete`
/// says which frames are there to be read.
#[derive(Debug)]
pub struct BundleReader {
    dir: PathBuf,
    manifest: Manifest,
}

impl BundleReader {
    /// Opens and checks the bundle at `dir`. No frame is read until one is asked for.
    pub fn open(dir: impl AsRef<Path>) -> Result<Self, Error> {
        let dir = dir.as_ref().to_path_buf();
        let manifest = open_manifest(&dir)?;
        Ok(Self { dir, manifest })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// Where frame `index` is, or will be.
    pub fn frame_path(&self, index: u32) -> PathBuf {
        frame_path(&self.dir, index)
    }

    /// The indices of the frames that can be read: listed in the manifest, with a file of the
    /// recorded length. Checking the length and not the contents keeps this cheap enough to call
    /// on a bundle of thousands of frames; `read_frame` checks the rest.
    pub fn complete(&self) -> Vec<u32> {
        self.manifest
            .frames
            .iter()
            .filter(|entry| file_matches(&self.dir, entry))
            .map(|entry| entry.index)
            .collect()
    }

    /// Reads frame `index` and checks it against the manifest.
    pub fn read_frame(&self, index: u32) -> Result<Frame, Error> {
        if self
            .manifest
            .frames
            .binary_search_by_key(&index, |e| e.index)
            .is_err()
        {
            return Err(Error::FrameNotWritten { index });
        }
        let path = self.frame_path(index);
        let bytes = fs::read(&path).map_err(io("read", &path))?;
        // The recorded length is not checked against the file's here. A cut-short file is short
        // of a chunk or a header it declares, so the parser reports it as truncated anyway, and a
        // file that is the wrong length for another reason - a frame from another run copied
        // over it - is better named by what the parser finds wrong with it.
        decode(&bytes, &path, Some((self.manifest.grid.grid(), index)))
    }
}

/// Frame `index`'s file within the bundle at `dir`. The manifest spells the name with a forward
/// slash on every system; this builds it from components, so that a message on Windows names the
/// file with one kind of separator.
fn frame_path(dir: &Path, index: u32) -> PathBuf {
    let name = frame_file_name(index);
    let leaf = name
        .rsplit('/')
        .next()
        .expect("a frame file name has a leaf");
    dir.join(FRAMES).join(leaf)
}

/// True when the frame file the entry describes is there with the recorded length.
fn file_matches(dir: &Path, entry: &FrameEntry) -> bool {
    fs::metadata(frame_path(dir, entry.index)).is_ok_and(|m| m.len() == entry.bytes)
}

/// Writes a bundle: its frames one at a time, and its manifest whenever the caller asks.
///
/// Rewriting the manifest after every frame is what makes a crash cost one frame, and it is the
/// caller's choice because the manifest grows with the run: a long run may prefer to rewrite it
/// every few frames and accept losing those few to a crash.
#[derive(Debug)]
pub struct BundleWriter {
    dir: PathBuf,
    manifest: Manifest,
}

impl BundleWriter {
    /// Starts a new bundle at `dir`, creating the directory if it is not there, and writes its
    /// manifest. `format` and `version` are set here; any frames listed are dropped, since there
    /// are no frame files yet. Refuses a directory that already holds a bundle.
    pub fn create(dir: impl AsRef<Path>, mut manifest: Manifest) -> Result<Self, Error> {
        let dir = dir.as_ref().to_path_buf();
        if dir.join(MANIFEST).exists() {
            return Err(Error::AlreadyABundle { path: dir });
        }
        manifest.format = FORMAT.into();
        manifest.version = VERSION;
        manifest.frames.clear();
        manifest.validate()?;
        let frames = dir.join(FRAMES);
        fs::create_dir_all(&frames).map_err(io("create", &frames))?;
        let writer = Self { dir, manifest };
        writer.write_manifest()?;
        Ok(writer)
    }

    /// Picks up a bundle a previous run left, to go on writing it.
    ///
    /// Frames the manifest lists but whose files are missing or of the wrong length are dropped
    /// from the manifest, and temporary files are deleted; `is_complete` then says which frames
    /// the run can skip. The manifest on disk is not rewritten until the next `write_manifest`.
    pub fn resume(dir: impl AsRef<Path>) -> Result<Self, Error> {
        let dir = dir.as_ref().to_path_buf();
        let mut manifest = open_manifest(&dir)?;
        if manifest.version != VERSION {
            // Appending version-1 frames to an older bundle would make a bundle of two versions.
            return Err(Error::InvalidManifest {
                why: format!(
                    "it is version {}, and this build resumes only version {VERSION} bundles",
                    manifest.version
                ),
            });
        }
        manifest.frames.retain(|entry| file_matches(&dir, entry));
        let frames = dir.join(FRAMES);
        fs::create_dir_all(&frames).map_err(io("create", &frames))?;
        for folder in [&dir, &frames] {
            let listing = fs::read_dir(folder).map_err(io("list", folder))?;
            for item in listing.flatten() {
                let path = item.path();
                if path.to_string_lossy().ends_with(TEMPORARY_SUFFIX) && path.is_file() {
                    fs::remove_file(&path).map_err(io("delete", &path))?;
                }
            }
        }
        Ok(Self { dir, manifest })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// True when frame `index` is written and listed, so a resumed run need not trace it again.
    pub fn is_complete(&self, index: u32) -> bool {
        self.manifest
            .frames
            .binary_search_by_key(&index, |e| e.index)
            .is_ok()
    }

    /// Writes one frame's file and records it in the manifest held in memory. The manifest on
    /// disk changes only at the next `write_manifest`.
    ///
    /// `entry` supplies the proper time, position and read-outs; its `file` and `bytes` are set
    /// here. A frame already written under the same index is replaced. Nothing is written if the
    /// frame or the entry breaks a rule of the format.
    pub fn write_frame(&mut self, frame: &Frame, mut entry: FrameEntry) -> Result<(), Error> {
        let path = frame_path(&self.dir, frame.index);
        if entry.index != frame.index {
            return Err(Error::InvalidFrame {
                why: format!(
                    "the frame is frame {} and its manifest entry is for frame {}",
                    frame.index, entry.index
                ),
            });
        }
        let grid = self.manifest.grid.grid();
        if frame.grid() != grid {
            return Err(Error::DimensionMismatch {
                path,
                found: (frame.width, frame.height),
                expected: (grid.width, grid.height),
            });
        }
        entry.file = frame_file_name(frame.index);
        self.manifest.check_entry(&entry)?;
        let frames = &self.manifest.frames;
        let slot = frames.partition_point(|e| e.index < entry.index);
        let replacing = frames.get(slot).is_some_and(|e| e.index == entry.index);
        if let Some(before) = slot.checked_sub(1).map(|k| &frames[k]) {
            check_order(before, &entry)?;
        }
        if let Some(after) = frames.get(slot + usize::from(replacing)) {
            check_order(&entry, after)?;
        }

        let bytes = frame.encode()?;
        write_atomically(&path, &bytes)?;
        entry.bytes = bytes.len() as u64;
        if replacing {
            self.manifest.frames[slot] = entry;
        } else {
            self.manifest.frames.insert(slot, entry);
        }
        Ok(())
    }

    /// Writes the manifest as it stands in memory, atomically.
    pub fn write_manifest(&self) -> Result<(), Error> {
        self.manifest.validate()?;
        write_atomically(&self.dir.join(MANIFEST), self.manifest.to_json().as_bytes())
    }

    /// `write_frame` and then `write_manifest`: a crash after this returns loses nothing.
    pub fn write_frame_and_manifest(
        &mut self,
        frame: &Frame,
        entry: FrameEntry,
    ) -> Result<(), Error> {
        self.write_frame(frame, entry)?;
        self.write_manifest()
    }
}
