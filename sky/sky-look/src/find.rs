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

/// Where NASA serves the default star map: the address `fetch-sky.ps1` downloads it from, for a
/// reader of a sentence who has no PowerShell to run that script with.
pub const SKY_MAP_URL: &str =
    "https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_8k_gal.exr";

/// The size of the default star map as a reader is told it: 160 735 772 bytes, in the binary
/// megabytes `fetch-sky.ps1` and a file manager both count in.
pub const SKY_MAP_SIZE: &str = "153 MB";

/// How ffmpeg is got on the platform this program was built for, as the words that follow
/// "install ffmpeg". A sentence that names another system's package manager sends its reader to
/// look for a program that is not there.
const FFMPEG_INSTALL: &str = if cfg!(windows) {
    "with winget install Gyan.FFmpeg"
} else if cfg!(target_os = "macos") {
    "with brew install ffmpeg, which needs Homebrew (https://brew.sh)"
} else {
    "with the system's package manager, for example sudo apt install ffmpeg"
};

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
    // A copy that was downloaded has no source to build from, and telling its user to run cargo
    // sends them after a compiler for what is a file missing from an archive. `--tools` is given
    // by a person at a terminal, who is told how to build.
    let installed = search.tools.is_none() && nearest_sky_dir(&dir).is_none();
    let program = |name: &str| {
        let path = dir.join(name);
        if path.is_file() {
            Ok(path)
        } else if installed {
            Err(format!(
                "{name} was not found in {}, {how}, so this copy of Black Hole Lab is incomplete; \
                 download Black Hole Lab again and extract every file of the archive into one \
                 folder, or name the directory that holds the sky tools with --tools <dir>.",
                dir.display()
            ))
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
                "{} was not found on the PATH, and sky-render needs it to write the photograph; \
                 install ffmpeg {FFMPEG_INSTALL}, or name it with --ffmpeg <path>.",
                names.ffmpeg
            )
        })
}

/// The star map: `--sky`, else the variable, else [`SKY_MAP`] where [`default_map`] says this
/// layout keeps it.
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
    let map = default_map(exe_dir);
    if map.is_file() {
        return Ok(map);
    }
    // The script is named only where it is: a copy that was downloaded may have been extracted
    // without it. The address serves a reader who has no PowerShell, script or no script.
    let maps = map.parent().unwrap_or(exe_dir);
    let script = under(maps.parent().unwrap_or(exe_dir), FETCH_SCRIPT);
    let by_script = if script.is_file() {
        format!("fetch it once with pwsh {}, or ", script.display())
    } else {
        String::new()
    };
    Err(format!(
        "The star map {} is not there; {by_script}download {SKY_MAP_URL} ({SKY_MAP_SIZE}, one of \
         NASA's Deep Star Maps) into {}, or name another map with --sky <map.exr> or \
         {SKY_MAP_ENV}.",
        map.display(),
        maps.display()
    ))
}

/// Where the default star map is kept for a program in `exe_dir`, whether or not it is there.
///
/// Two layouts, told apart by whether there is a `sky` directory at or above the program:
///
/// - a checkout of the repository, where there is one, and the map is [`SKY_MAP`] inside it. "A
///   `sky` directory at or above" is read both ways a checkout can have it: a directory above
///   that is itself named `sky` (this program runs from `sky/target/release`), and a directory
///   above that holds one named `sky` (a program beside the app, in the repository's
///   `target/release`). At each level going up, the first is tried before the second, so the
///   nearest wins;
/// - a copy that was downloaded, where there is none, and the map is [`SKY_MAP`] beside the
///   program: the archive is one folder, and its `maps` folder is where the fetch script is.
///
/// A map beside the program is used wherever it is found, checkout or not, as the nearer one.
pub fn default_map(exe_dir: &Path) -> PathBuf {
    let beside = under(exe_dir, SKY_MAP);
    if beside.is_file() {
        return beside;
    }
    match nearest_sky_dir(exe_dir) {
        Some(sky_dir) => under(&sky_dir, SKY_MAP),
        None => beside,
    }
}

/// `relative`, one of this module's paths written with `/`, under `base`, joined a name at a time
/// so that the path is shown to its reader with one kind of separator.
fn under(base: &Path, relative: &str) -> PathBuf {
    relative
        .split('/')
        .fold(base.to_path_buf(), |path, name| path.join(name))
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
