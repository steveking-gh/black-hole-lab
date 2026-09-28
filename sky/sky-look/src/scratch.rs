//! The one place this program keeps what it makes on the way: the traced bundle, and the video and
//! photograph before they are given their names.
//!
//! Every run works in a directory of its own, `run-<ms>-<pid>-<n>`, inside [`ROOT`] in the
//! system's temporary directory: the start time in milliseconds, the process id and a counter make
//! two runs at once, of one copy of the program or of two, use two directories. The run's
//! directory is deleted when its [`Scratch`] is dropped, which is at the end of the run however it
//! ends.
//!
//! A run that is killed - the app closing kills this program and its children, the whole tree on
//! Windows - cannot delete anything, so whatever it had made stays in its directory under
//! [`ROOT`], and nowhere else. The next run clears [`ROOT`] of whatever is older than [`STALE`]
//! before it starts; an hour, because no run takes that long (a press is answered in under half a
//! minute, and even the longest `--hold` encodes in minutes), so that a directory that old cannot
//! belong to a run still going, of this copy of the app or another.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// The directory of this program's own under the system's temporary directory.
pub const ROOT: &str = "black-hole-lab-sky-look";

/// How old a leftover in [`ROOT`] must be before a run deletes it.
pub const STALE: Duration = Duration::from_secs(3600);

/// A run's own directory, deleted when this is dropped.
#[derive(Debug)]
pub struct Scratch {
    dir: PathBuf,
}

impl Scratch {
    /// Clears `root` of what is stale, then makes a new directory for this run inside it.
    /// `temp` is the system's temporary directory, or a test's stand-in for it.
    pub fn new(temp: &Path) -> std::io::Result<Self> {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let root = temp.join(ROOT);
        std::fs::create_dir_all(&root)?;
        clear_stale(&root, SystemTime::now());
        let ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis();
        loop {
            let n = NEXT.fetch_add(1, Ordering::Relaxed);
            let dir = root.join(format!("run-{ms}-{}-{n}", std::process::id()));
            // `create_dir`, not `create_dir_all`: it fails on a directory that is there, so a name
            // left by an earlier process with the same id is passed over and never shared.
            match std::fs::create_dir(&dir) {
                Ok(()) => return Ok(Self { dir }),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(e),
            }
        }
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// Deletes every entry of `root` last modified more than [`STALE`] before `now`, and nothing
/// else: not `root` itself, not anything outside it.
///
/// An entry is judged by its own modification time, read without following links, and a link is
/// deleted as a link: `remove_dir_all` does not follow a symbolic link or a junction out of `root`
/// (the standard library fixed that in 1.58.1, CVE-2022-21658), and a link is not a directory to
/// it. An entry dated in the future is left alone rather than taken for stale. Failures are
/// passed over: a leftover that cannot be deleted now is tried again by the next run, and is no
/// reason to refuse this one.
pub fn clear_stale(root: &Path, now: SystemTime) {
    let Ok(listing) = std::fs::read_dir(root) else {
        return;
    };
    for entry in listing.flatten() {
        let path = entry.path();
        let Ok(meta) = std::fs::symlink_metadata(&path) else {
            continue;
        };
        let stale = meta
            .modified()
            .ok()
            .and_then(|m| now.duration_since(m).ok())
            .is_some_and(|age| age > STALE);
        if !stale {
            continue;
        }
        if meta.is_dir() {
            let _ = std::fs::remove_dir_all(&path);
        } else {
            // A file, or a link of either kind: removed as itself. A directory symlink on Windows
            // is removed with `remove_dir`, a file symlink with `remove_file`.
            let _ = std::fs::remove_file(&path).or_else(|_| std::fs::remove_dir(&path));
        }
    }
}

/// `path` with `suffix` added to its whole name: `a.mkv` and `.partial` make `a.mkv.partial`.
pub fn with_suffix(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.as_os_str().to_owned();
    name.push(suffix);
    PathBuf::from(name)
}

/// Moves the directory `from` to `to`, a name that must not be taken: renamed within a volume,
/// copied and then deleted across volumes.
pub fn move_dir(from: &Path, to: &Path) -> std::io::Result<()> {
    if to.exists() {
        return Err(std::io::ErrorKind::AlreadyExists.into());
    }
    if std::fs::rename(from, to).is_ok() {
        return Ok(());
    }
    copy_dir(from, to)?;
    let _ = std::fs::remove_dir_all(from);
    Ok(())
}

fn copy_dir(from: &Path, to: &Path) -> std::io::Result<()> {
    std::fs::create_dir(to)?;
    for entry in std::fs::read_dir(from)? {
        let entry = entry?;
        let target = to.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_dir(&entry.path(), &target)?;
        } else {
            std::fs::copy(entry.path(), &target)?;
        }
    }
    Ok(())
}
