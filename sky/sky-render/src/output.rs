//! The files a run writes, the intermediate files it makes on the way, and how each reaches its
//! final name.
//!
//! ffmpeg encodes the video into an MP4 under a name of its own, which `crate::mp4` tags as
//! spherical. Then:
//!
//! - for `--out <name>.mp4`, the tagged file is the video, and with `--readouts overlay` the
//!   subtitles go beside it as `<name>.ass`, which VLC loads by itself because the names match;
//! - for `--out <name>.mkv`, the tagged MP4 and the subtitles are copied, not re-encoded, into a
//!   Matroska file by a second run of ffmpeg (`crate::encode::remux_matroska`), which carries the
//!   spherical tag across. That is how the owner's approved trial was made.
//!
//! Every file a viewer is given is written under a temporary name and renamed into place only
//! when it is whole, so that its name holds either nothing, the old file (with `--overwrite`) or
//! the finished new one. Every intermediate and temporary name is listed in [`Plan::scratch`] and
//! removed when the run ends, whether it succeeded or failed.

use std::io::Write;
use std::path::{Path, PathBuf};

use crate::cli::Container;
use crate::encode::remux_matroska;

/// Where a run's files go.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub video: PathBuf,
    pub container: Container,
    /// The untagged MP4 ffmpeg writes.
    pub partial: PathBuf,
    /// The subtitle file beside an MP4, when there are subtitles to put there.
    pub sidecar: Option<PathBuf>,
    /// Whether a Matroska file carries a subtitle track.
    pub subtitle_track: bool,
}

/// `path` with `suffix` added to its whole name: `a.mkv` and `.partial` make `a.mkv.partial`.
pub fn with_suffix(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.as_os_str().to_owned();
    name.push(suffix);
    PathBuf::from(name)
}

/// The name under which [`write_atomically`] writes `path` before renaming it.
fn temporary(path: &Path) -> PathBuf {
    with_suffix(path, ".tmp")
}

impl Plan {
    /// The files for a video at `video`, in `container`, with or without a subtitle track.
    pub fn new(video: &Path, container: Container, subtitles: bool) -> Self {
        Self {
            video: video.to_path_buf(),
            container,
            partial: with_suffix(video, ".partial"),
            sidecar: (subtitles && container == Container::Mp4)
                .then(|| video.with_extension("ass")),
            subtitle_track: subtitles && container == Container::Mkv,
        }
    }

    /// The tagged MP4 a Matroska file is made from.
    fn tagged(&self) -> PathBuf {
        with_suffix(&self.video, ".tagged.mp4")
    }

    /// The subtitles a Matroska file is made from.
    fn subtitles(&self) -> PathBuf {
        with_suffix(&self.video, ".readouts.ass")
    }

    /// The files the viewer is given.
    pub fn finals(&self) -> Vec<PathBuf> {
        let mut out = vec![self.video.clone()];
        out.extend(self.sidecar.clone());
        out
    }

    /// Every other name the run may write under, all to be removed when it ends.
    pub fn scratch(&self) -> Vec<PathBuf> {
        let mut out = vec![
            self.partial.clone(),
            self.tagged(),
            temporary(&self.tagged()),
            self.subtitles(),
            temporary(&self.video),
        ];
        out.extend(self.sidecar.as_deref().map(temporary));
        out
    }

    /// Makes the finished files from ffmpeg's untagged MP4 and the subtitle script, and returns
    /// what the run should say about them.
    pub fn finish(&self, ffmpeg: &Path, script: Option<&str>) -> Result<Vec<String>, String> {
        let mut said = Vec::new();
        match self.container {
            Container::Mp4 => {
                // The sidecar is written first: a failure then leaves the old video as it was,
                // and the video's name is taken last.
                if let (Some(sidecar), Some(script)) = (&self.sidecar, script) {
                    write_atomically(sidecar, script.as_bytes())?;
                    said.push(format!(
                        "wrote {} beside the video: the read-outs as subtitles, which VLC loads by \
                         itself because the names match (other players may need to be given it)",
                        sidecar.display()
                    ));
                }
                crate::mp4::tag_file(&self.partial, &self.video)?;
            }
            Container::Mkv => {
                let tagged = self.tagged();
                crate::mp4::tag_file(&self.partial, &tagged)?;
                let subtitles = match (self.subtitle_track, script) {
                    (true, Some(script)) => {
                        let path = self.subtitles();
                        std::fs::write(&path, script)
                            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
                        Some(path)
                    }
                    _ => None,
                };
                let temporary = temporary(&self.video);
                remux_matroska(ffmpeg, &tagged, subtitles.as_deref(), &temporary)?;
                sync(&temporary)?;
                rename(&temporary, &self.video)?;
                if subtitles.is_some() {
                    said.push(
                        "the read-outs are its subtitle track, marked as the one to show".into(),
                    );
                }
            }
        }
        Ok(said)
    }
}

/// Removes the listed files when dropped: whatever a run leaves on the way, it does not leave.
pub struct Cleanup(pub Vec<PathBuf>);

impl Drop for Cleanup {
    fn drop(&mut self) {
        for path in &self.0 {
            let _ = std::fs::remove_file(path);
        }
    }
}

fn rename(from: &Path, to: &Path) -> Result<(), String> {
    std::fs::rename(from, to).map_err(|e| {
        format!(
            "could not rename {} to {}: {e}",
            from.display(),
            to.display()
        )
    })
}

/// Flushes a file another program wrote. Opened for writing, not reading: Windows flushes only
/// through a handle with write access, and refuses a read-only one ("Access is denied").
fn sync(path: &Path) -> Result<(), String> {
    std::fs::OpenOptions::new()
        .write(true)
        .open(path)
        .and_then(|f| f.sync_all())
        .map_err(|e| format!("could not flush {}: {e}", path.display()))
}

/// Writes `bytes` to `path` under a temporary name, flushed, then renamed: `path` is only ever
/// absent, its old contents, or all of the new.
pub fn write_atomically(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let temporary = temporary(path);
    let say =
        |what: &str, e: std::io::Error| format!("could not {what} {}: {e}", temporary.display());
    let result = (|| {
        let mut file = std::fs::File::create(&temporary).map_err(|e| say("create", e))?;
        file.write_all(bytes).map_err(|e| say("write", e))?;
        file.sync_all().map_err(|e| say("flush", e))?;
        drop(file);
        rename(&temporary, path)
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temporary);
    }
    result
}
