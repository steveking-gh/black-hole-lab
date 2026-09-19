//! What version this build is, decided once and stamped into the binary as
//! `BLACK_HOLE_LAB_VERSION`: `MAJOR.MINOR` from `Cargo.toml` with the number of commits behind HEAD
//! as the third segment, so that two builds of the same working afternoon can be told apart by
//! reading the title line. The rule itself is `src/version/compose.rs`, included below and shared
//! with the crate rather than written out twice.
//!
//! Best effort, like `crate::stamp`: no git, no checkout, a `git` that fails for any reason at all,
//! and the version is the bare package version with a `cargo::warning` beside it. A build is never
//! failed over a decoration.

use std::path::{Path, PathBuf};
use std::process::Command;

#[path = "src/version/compose.rs"]
mod compose;

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    watch_head(&root);
    println!("cargo::rerun-if-changed=build.rs");

    let count = commit_count(&root);
    if count.is_none() {
        println!(
            "cargo::warning=no git commit count available for {}; this build reports the bare \
             package version with no third segment",
            root.display()
        );
    }
    let version = compose::compose(env!("CARGO_PKG_VERSION"), count);
    println!("cargo::rustc-env=BLACK_HOLE_LAB_VERSION={version}");
}

/// Ask cargo to rerun this script exactly when the commit count can have changed.
///
/// Without these the script reruns on every source change; with them it reruns when HEAD moves - a
/// commit, a branch switch, a `git gc` that repacks the refs - and at no other time. Each path is
/// named only if it is there, because cargo treats a missing `rerun-if-changed` file as permanently
/// stale, and naming `packed-refs` in a repository that has never packed its refs would force a
/// rebuild of the whole crate on every single build. A repack creates `packed-refs` and removes the
/// loose refs under `refs/heads`, so that directory's change is what triggers the rerun that starts
/// watching the packed file.
///
/// The two directories are asked for rather than assumed to be `<root>/.git`, because in a git
/// worktree `.git` is a *file* pointing elsewhere: HEAD lives in the worktree's own git directory
/// and the refs live in the common one shared with the main checkout. Assuming `<root>/.git` there
/// names three paths that do not exist, watches nothing, and leaves the count frozen at whatever it
/// was when the crate was last built for another reason.
fn watch_head(root: &Path) {
    let git_dir = git_path(root, "--git-dir").unwrap_or_else(|| root.join(".git"));
    let common_dir = git_path(root, "--git-common-dir").unwrap_or_else(|| root.join(".git"));
    for path in [
        git_dir.join("HEAD"),
        common_dir.join("refs").join("heads"),
        common_dir.join("packed-refs"),
    ] {
        if path.exists() {
            println!("cargo::rerun-if-changed={}", path.display());
        }
    }
}

/// One of git's own answers to where a directory of the repository is, resolved against `root`
/// because git answers relatively when it can. `None` when git is not there or this is not a
/// checkout, which leaves the caller with its own fallback.
fn git_path(root: &Path, which: &str) -> Option<PathBuf> {
    let out = Command::new("git").args(["rev-parse", which]).current_dir(root).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let path = String::from_utf8_lossy(&out.stdout).trim().to_string();
    (!path.is_empty()).then(|| root.join(path))
}

/// `git rev-list --count HEAD` in `root`, or `None` when git is not there, the directory is not a
/// checkout, or the answer is not a count.
fn commit_count(root: &Path) -> Option<u64> {
    let out =
        Command::new("git").args(["rev-list", "--count", "HEAD"]).current_dir(root).output().ok()?;
    if !out.status.success() {
        return None;
    }
    String::from_utf8_lossy(&out.stdout).trim().parse().ok()
}
