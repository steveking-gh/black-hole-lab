//! Where the pieces are: the two programs this one runs, the ffmpeg the renderer runs, and the star
//! map.
//!
//! Every piece is looked for before any work is done, so that a missing one costs the user nothing
//! but the sentence saying what to do about it, and not a trace thrown away when the renderer turns
//! out to have no map. [`find`] is a pure function of a [`Search`], which says where to look,
//! so that the tests can run it against directory layouts made for the purpose; `Search::of` fills
//! one in from the process.

use std::ffi::{OsStr, OsString};
use std::path::{Path, PathBuf};

/// The environment variable that names the star map, ahead of the search.
pub const SKY_MAP_ENV: &str = "BLACK_HOLE_LAB_SKY_MAP";

/// The default star map, relative to the `sky` directory: NASA's 8K map in galactic coordinates,
/// the map the owner's approved films were made with.
pub const SKY_MAP: &str = "maps/starmap_2020_8k_gal.exr";

/// The script that fetches the maps, relative to the `sky` directory.
pub const FETCH_SCRIPT: &str = "maps/fetch-sky.ps1";

/// The file names of the programs, as this platform spells them. The tests put scripts in their
/// place, which on Windows cannot be named `.exe`, so the names are part of the search rather than
/// constants inside it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Names {
    pub trace: String,
    pub render: String,
    pub ffmpeg: String,
}

impl Names {
    /// The real programs' names on this platform.
    pub fn native() -> Self {
        let exe = std::env::consts::EXE_SUFFIX;
        Self {
            trace: format!("sky-trace{exe}"),
            render: format!("sky-render{exe}"),
            ffmpeg: format!("ffmpeg{exe}"),
        }
    }
}

/// Where to look, and what the command line named outright.
#[derive(Debug, Clone, Default)]
pub struct Search {
    /// This program's own directory.
    pub exe_dir: Option<PathBuf>,
    /// `--tools <dir>`.
    pub tools: Option<PathBuf>,
    /// `--ffmpeg <path>`.
    pub ffmpeg: Option<PathBuf>,
    /// The value of `PATH`.
    pub path: Option<OsString>,
    /// `--sky <map.exr>`.
    pub sky: Option<PathBuf>,
    /// The value of [`SKY_MAP_ENV`].
    pub sky_env: Option<OsString>,
}

/// The pieces, every one of them a file that is there.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pieces {
    pub trace: PathBuf,
    pub render: PathBuf,
    pub ffmpeg: PathBuf,
    pub sky: PathBuf,
}

/// Finds every piece, or says in one sentence which is missing and how to get it. The pieces are
/// looked for in the order the run needs them, and the first one missing is the one reported.
pub fn find(search: &Search, names: &Names) -> Result<Pieces, String> {
    let (trace, render) = tools(search, names)?;
    let ffmpeg = ffmpeg(search, names)?;
    let sky = sky_map(search)?;
    Ok(Pieces {
        trace,
        render,
        ffmpeg,
        sky,
    })
}

/// `sky-trace` and `sky-render`, from `--tools` or beside this program. Not looked for anywhere
/// else, and not on the PATH: the three programs are built together from one workspace, and a
/// tracer or renderer of another build could write or expect a bundle this one does not.
fn tools(search: &Search, names: &Names) -> Result<(PathBuf, PathBuf), String> {
    let (dir, how) = match (&search.tools, &search.exe_dir) {
        (Some(dir), _) => (dir.clone(), "the directory --tools names"),
        (None, Some(dir)) => (dir.clone(), "the directory sky-look is in"),
        (None, None) => {
            return Err(
                "sky-look cannot tell which directory it is in, so it cannot find sky-trace and \
                 sky-render beside it; name their directory with --tools <dir>."
                    .into(),
            );
        }
    };
    let program = |name: &str| {
        let path = dir.join(name);
        if path.is_file() {
            Ok(path)
        } else {
            Err(format!(
                "{name} was not found in {}, {how}; build the sky tools with cargo build --release \
                 in the sky directory, or name the directory that holds them with --tools <dir>.",
                dir.display()
            ))
        }
    };
    Ok((program(&names.trace)?, program(&names.render)?))
}

/// ffmpeg: `--ffmpeg`, or the first on the PATH.
fn ffmpeg(search: &Search, names: &Names) -> Result<PathBuf, String> {
    if let Some(named) = &search.ffmpeg {
        return if named.is_file() {
            Ok(named.clone())
        } else {
            Err(format!(
                "--ffmpeg names {}, and there is no program there; give the full path of \
                 ffmpeg, or leave --ffmpeg out to use the one on the PATH.",
                named.display()
            ))
        };
    }
    search
        .path
        .as_deref()
        .into_iter()
        .flat_map(std::env::split_paths)
        .map(|dir| dir.join(&names.ffmpeg))
        .find(|candidate| candidate.is_file())
        .ok_or_else(|| {
            format!(
                "{} was not found on the PATH, and sky-render needs it to encode the video; \
                 install ffmpeg (on Windows: winget install Gyan.FFmpeg), or name it with \
                 --ffmpeg <path>.",
                names.ffmpeg
            )
        })
}

/// The star map: `--sky`, else the variable, else [`SKY_MAP`] in the nearest `sky` directory at
/// or above this program's directory.
///
/// "A `sky` directory at or above" is read both ways a layout can have it: a directory above that
/// is itself named `sky` (this program runs from `sky/target/release`), and a directory above that
/// holds one named `sky` (a program beside the app, in the repository's `target/release`). At each
/// level going up, the first is tried before the second, so the nearest wins.
fn sky_map(search: &Search) -> Result<PathBuf, String> {
    if let Some(named) = &search.sky {
        return if named.is_file() {
            Ok(named.clone())
        } else {
            Err(format!(
                "--sky names {}, and there is no star map there; give the path of an \
                 equirectangular .exr map, or fetch the default one with {FETCH_SCRIPT} in the sky \
                 directory.",
                named.display()
            ))
        };
    }
    if let Some(named) = search.sky_env.as_deref().filter(|v| !v.is_empty()) {
        let named = PathBuf::from(named);
        // Reported rather than passed over, as the app treats its own variable: whoever set it
        // meant that map, and quietly using another would hide the mistake.
        return if named.is_file() {
            Ok(named)
        } else {
            Err(format!(
                "{SKY_MAP_ENV} names {}, and there is no star map there; correct the variable, or \
                 remove it to use {SKY_MAP} in the sky directory.",
                named.display()
            ))
        };
    }
    let Some(exe_dir) = &search.exe_dir else {
        return Err(format!(
            "sky-look cannot tell which directory it is in, so it cannot find the star map; name \
             it with --sky <map.exr> or {SKY_MAP_ENV}."
        ));
    };
    let Some(sky_dir) = nearest_sky_dir(exe_dir) else {
        return Err(format!(
            "The star map was not found: there is no sky directory at or above {}. Fetch the map \
             with the script sky/{FETCH_SCRIPT} of the Black Hole Lab repository, or name a map \
             with --sky <map.exr> or {SKY_MAP_ENV}.",
            exe_dir.display()
        ));
    };
    let map = sky_dir.join(SKY_MAP);
    if map.is_file() {
        Ok(map)
    } else {
        Err(format!(
            "The star map {} is not there; fetch it once with pwsh {}, which downloads NASA's \
             maps, or name another map with --sky <map.exr> or {SKY_MAP_ENV}.",
            map.display(),
            sky_dir.join(FETCH_SCRIPT).display()
        ))
    }
}

/// The nearest directory at or above `dir` that is named `sky`, or that is the `sky` inside one.
fn nearest_sky_dir(dir: &Path) -> Option<PathBuf> {
    dir.ancestors().find_map(|above| {
        if above.file_name() == Some(OsStr::new("sky")) && above.is_dir() {
            return Some(above.to_path_buf());
        }
        let inside = above.join("sky");
        inside.is_dir().then_some(inside)
    })
}
