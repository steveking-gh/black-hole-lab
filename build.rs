//! What version this build is, decided once and stamped into the binary as
//! `BLACK_HOLE_LAB_VERSION`: `MAJOR.MINOR` from `Cargo.toml` with the number of commits behind HEAD
//! as the third segment, so that two builds of the same working afternoon can be told apart by
//! reading the title line. The rule itself is `src/version/compose.rs`, included below and shared
//! with the crate rather than written out twice.
//!
//! Best effort, like `crate::stamp`: no git, no checkout, a `git` that fails for any reason at all,
//! and the version is the bare package version with a `cargo::warning` beside it. A build is never
//! failed over a decoration.
//!
//! The second thing this script does is build the sky tools, `sky/`'s own cargo workspace, in
//! release: see [`build_sky`]. That one is not best effort. A sky crate that does not compile fails
//! the app's build, because an app whose Look Around button runs a stale `sky-look` is the
//! surprise this arrangement exists to prevent.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

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

    build_sky(&root);
}

/// The environment variable that, when set to anything, keeps this script from building the sky
/// tools: for a machine that only ever builds the app, or a tool that checks the app on every
/// keystroke and has no use for a release build of seven crates each time a sky file is saved.
const SKIP_SKY: &str = "BLACK_HOLE_LAB_NO_SKY_BUILD";

/// A nested build that took longer than this is reported with a `cargo::warning`, so that the
/// seconds cargo showed as "black-hole-lab (build script)" are accounted for.
const WORTH_MENTIONING: Duration = Duration::from_secs(3);

/// Build the sky tools with `cargo build --release` in `<root>/sky`, into `sky/target`, which is
/// where the app looks for `sky-look` (`crate::look_around::find_sky_look`).
///
/// # One way only
///
/// The sky crates are a workspace of their own, beside the app's, so that nothing in them can
/// change the app's manifest, its lock file, which features its dependencies are built with, or
/// what `cargo run` means at the repository root (`sky/Cargo.toml` says how that last one was
/// learnt). The other direction is wanted: a build of the app must leave the sky tools as fresh as
/// the app, because the app runs them, and a `cargo run --release` that quietly ran last week's
/// `sky-look` was the surprise that led here. Cargo has no way to say "build that other workspace
/// too", so the app's build script does it, in the one direction: the sky tools never build the
/// app.
///
/// # When it runs
///
/// Cargo reruns this script when any watched path changes, and at no other time; so the nested
/// build costs nothing while the sky sources are untouched, about half a second when they are
/// touched but already built, and the compile itself otherwise. Watched are the sky workspace's
/// manifest and lock file and each member crate's manifest and `src` directory, found by looking
/// for a `Cargo.toml` in each directory of `sky/`. Not `sky/` itself: that would watch
/// `sky/target` and rerun on every build.
///
/// # The nested cargo
///
/// It is the same `cargo` that is running this script (the `CARGO` variable), told its target
/// directory outright, because a `CARGO_TARGET_DIR` in the environment would otherwise send the
/// sky build into the app's own target directory, whose lock the outer build holds: a deadlock.
/// The two wrapper variables are cleared. Under `cargo clippy` the outer build sets
/// `RUSTC_WORKSPACE_WRAPPER` to clippy's driver, which would lint the sky crates as part of the
/// app's check and fail it on their warnings; and rust-analyzer's `RUSTC_WRAPPER` short-circuits
/// compilation for its own check, which is not what a release build of the sky tools wants.
/// Everything else is inherited: the jobserver, so that the two builds share the machine's cores
/// rather than each taking all of them, and any `RUSTFLAGS` the user set.
///
/// # When it fails
///
/// The app's build fails with the nested cargo's last lines, which name the sky crate and the
/// error. The likeliest cause that is not a compile error is a running `sky-look`: Windows keeps
/// an executable locked while it runs, so a Look Around in progress holds `sky-look.exe` and the
/// linker cannot replace it. Wait for the view, or cancel it, and build again.
fn build_sky(root: &Path) {
    println!("cargo::rerun-if-env-changed={SKIP_SKY}");
    let sky = root.join("sky");
    if !sky.join("Cargo.toml").is_file() || std::env::var_os(SKIP_SKY).is_some() {
        return;
    }
    watch_sky(&sky);

    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let started = Instant::now();
    let out = Command::new(cargo)
        .args(["build", "--release", "--target-dir"])
        .arg(sky.join("target"))
        .current_dir(&sky)
        .env_remove("RUSTC_WORKSPACE_WRAPPER")
        .env_remove("RUSTC_WRAPPER")
        .output();
    let took = started.elapsed();
    match out {
        Ok(out) if out.status.success() => {
            if took >= WORTH_MENTIONING {
                println!(
                    "cargo::warning=built the sky tools (sky/target/release) in {:.0} s",
                    took.as_secs_f64()
                );
            }
        }
        Ok(out) => {
            let said = String::from_utf8_lossy(&out.stderr);
            let lines: Vec<&str> = said.lines().collect();
            let tail = &lines[lines.len().saturating_sub(40)..];
            panic!(
                "the sky tools did not build (cargo build --release in {}, {}); set {SKIP_SKY} to \
                 build the app without them:\n{}",
                sky.display(),
                out.status,
                tail.join("\n")
            );
        }
        Err(cause) => panic!(
            "could not start cargo to build the sky tools in {}: {cause}; set {SKIP_SKY} to build \
             the app without them",
            sky.display()
        ),
    }
}

/// Ask cargo to rerun this script when a sky source can have changed: the workspace's manifest and
/// lock file, and each member crate's manifest and `src` directory (`build_sky`, "When it runs").
fn watch_sky(sky: &Path) {
    for file in ["Cargo.toml", "Cargo.lock"] {
        let path = sky.join(file);
        if path.exists() {
            println!("cargo::rerun-if-changed={}", path.display());
        }
    }
    let Ok(entries) = std::fs::read_dir(sky) else { return };
    for entry in entries.flatten() {
        let dir = entry.path();
        if !dir.join("Cargo.toml").is_file() {
            continue;
        }
        for watched in [dir.join("Cargo.toml"), dir.join("src")] {
            if watched.exists() {
                println!("cargo::rerun-if-changed={}", watched.display());
            }
        }
    }
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
