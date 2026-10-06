//! Where the pieces are: the two programs this one runs, and the star map.
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

/// Where NASA serves the default star map: the address `--fetch-map` downloads it from, and the
/// one a sentence gives a reader who would rather fetch it by hand.
pub const SKY_MAP_URL: &str =
    "https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_8k_gal.exr";

/// The size of the default star map as a reader is told it: 160 735 772 bytes, in the binary
/// megabytes a file manager counts in.
pub const SKY_MAP_SIZE: &str = "153 MB";

/// The file names of the programs, as this platform spells them. The tests put scripts in their
/// place, which on Windows cannot be named `.exe`, so the names are part of the search rather than
/// constants inside it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Names {
    pub trace: String,
    pub render: String,
}

impl Names {
    /// The real programs' names on this platform.
    pub fn native() -> Self {
        let exe = std::env::consts::EXE_SUFFIX;
        Self {
            trace: format!("sky-trace{exe}"),
            render: format!("sky-render{exe}"),
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
    /// `--sky <map.exr>`.
    pub sky: Option<PathBuf>,
    /// The value of [`SKY_MAP_ENV`].
    pub sky_env: Option<OsString>,
    /// This user's own data folder for Black Hole Lab, where the star map is downloaded to and
    /// looked for first: [`user_data_dir_here`].
    pub data_dir: Option<PathBuf>,
}

/// The pieces, every one of them a file that is there.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pieces {
    pub trace: PathBuf,
    pub render: PathBuf,
    pub sky: PathBuf,
}

/// Finds every piece, or says in one sentence which is missing and how to get it. The pieces are
/// looked for in the order the run needs them, and the first one missing is the one reported.
pub fn find(search: &Search, names: &Names) -> Result<Pieces, String> {
    let (trace, render) = tools(search, names)?;
    let sky = sky_map(search).map_err(|no_map| no_map.why)?;
    Ok(Pieces { trace, render, sky })
}

/// What `--check` reports: for each piece, the sentence [`find`] would give for it if it were the
/// first one missing, or `None` for a piece that is there. Every piece is looked for, where
/// [`find`] stops at the first, because the app shows what is missing before a view is asked for,
/// and a user told of one piece at a time finds out about the next only after getting the first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// `sky-trace` and `sky-render`.
    pub tools: Option<String>,
    pub map: Option<String>,
    /// Whether the map that is missing is the default one in its own place, which `--fetch-map`
    /// puts there; false for a map `--sky` or the variable names, which only its owner can supply.
    pub map_fetchable: bool,
}

impl Report {
    /// Whether every piece is there.
    pub fn complete(&self) -> bool {
        self.tools.is_none() && self.map.is_none()
    }

    /// The report as `--check` prints it, a piece a line: the piece's word (`tools`, `map`), a
    /// space, and `ok`, or `missing` and the sentence, or for a map `--fetch-map` would supply,
    /// `fetchable` and the sentence.
    pub fn lines(&self) -> Vec<String> {
        let line = |piece: &str, missing: &Option<String>, word: &str| match missing {
            None => format!("{piece} ok"),
            Some(why) => format!("{piece} {word} {why}"),
        };
        let map_word = if self.map_fetchable {
            "fetchable"
        } else {
            "missing"
        };
        vec![
            line("tools", &self.tools, "missing"),
            line("map", &self.map, map_word),
        ]
    }
}

/// Looks for every piece and says of each whether it is there: see [`Report`].
pub fn check(search: &Search, names: &Names) -> Report {
    let map = sky_map(search).err();
    Report {
        tools: tools(search, names).err(),
        map_fetchable: map.as_ref().is_some_and(|no_map| no_map.fetchable),
        map: map.map(|no_map| no_map.why),
    }
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

/// This user's own data folder for Black Hole Lab, whether or not it exists yet: where the star
/// map is downloaded to.
///
/// A folder of the user's and not the program's own, because the program's may not be the
/// user's to write in: a copy installed from the Microsoft Store is in a folder nobody can write
/// to, and so is one installed under Program Files, /usr or /Applications. The place is each
/// system's own for an application's data:
///
/// - Windows: `%LOCALAPPDATA%\Black Hole Lab`, which a Store app also writes to, in a folder of
///   the package's that Windows keeps for it;
/// - macOS: `~/Library/Application Support/Black Hole Lab`;
/// - elsewhere: `$XDG_DATA_HOME/black-hole-lab`, or `~/.local/share/black-hole-lab` where the
///   variable is not set to an absolute path, as the XDG Base Directory specification says.
pub fn user_data_dir_here() -> Option<PathBuf> {
    let var = |name: &str| {
        std::env::var_os(name)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    if cfg!(windows) {
        var("LOCALAPPDATA").map(|local| local.join("Black Hole Lab"))
    } else if cfg!(target_os = "macos") {
        var("HOME").map(|home| {
            home.join("Library")
                .join("Application Support")
                .join("Black Hole Lab")
        })
    } else {
        var("XDG_DATA_HOME")
            .filter(|dir| dir.is_absolute())
            .or_else(|| var("HOME").map(|home| home.join(".local").join("share")))
            .map(|share| share.join("black-hole-lab"))
    }
}

/// Where `--fetch-map` puts the default star map: in this user's data folder when there is one,
/// and otherwise where [`default_map`] keeps it for a program in `exe_dir`. `None` when neither is
/// known.
pub fn fetch_target(data_dir: Option<&Path>, exe_dir: Option<&Path>) -> Option<PathBuf> {
    data_dir
        .map(|dir| under(dir, SKY_MAP))
        .or_else(|| exe_dir.map(default_map))
}

/// A star map that is not there.
struct NoMap {
    /// The sentence saying so, and what to do.
    why: String,
    /// Whether it is the default map in its own place, which `--fetch-map` puts there.
    fetchable: bool,
}

/// The star map: `--sky`, else the variable, else [`SKY_MAP`] in this user's data folder, else
/// where [`default_map`] says this program's layout keeps it.
fn sky_map(search: &Search) -> Result<PathBuf, NoMap> {
    let named_wrongly = |why: String| NoMap {
        why,
        fetchable: false,
    };
    if let Some(named) = &search.sky {
        return if named.is_file() {
            Ok(named.clone())
        } else {
            Err(named_wrongly(format!(
                "--sky names {}, and there is no star map there; give the path of an \
                 equirectangular .exr map, or leave --sky out to use the default map, which \
                 sky-look --fetch-map downloads.",
                named.display()
            )))
        };
    }
    if let Some(named) = search.sky_env.as_deref().filter(|v| !v.is_empty()) {
        let named = PathBuf::from(named);
        // Reported rather than passed over, as the app treats its own variable: whoever set it
        // meant that map, and quietly using another would hide the mistake.
        return if named.is_file() {
            Ok(named)
        } else {
            Err(named_wrongly(format!(
                "{SKY_MAP_ENV} names {}, and there is no star map there; correct the variable, or \
                 remove it to use the default map.",
                named.display()
            )))
        };
    }
    // The user's own copy first, which is where a download puts it; then the program's, where a
    // checkout keeps it and where a map put by hand beside a downloaded copy goes.
    let user_map = search.data_dir.as_deref().map(|dir| under(dir, SKY_MAP));
    let program_map = search.exe_dir.as_deref().map(default_map);
    if let Some(found) = [&user_map, &program_map]
        .into_iter()
        .flatten()
        .find(|map| map.is_file())
    {
        return Ok(found.clone());
    }
    let Some(target) = fetch_target(search.data_dir.as_deref(), search.exe_dir.as_deref()) else {
        return Err(named_wrongly(format!(
            "sky-look cannot tell which directory it is in, so it cannot find the star map; name \
             it with --sky <map.exr> or {SKY_MAP_ENV}."
        )));
    };
    let folder = target.parent().unwrap_or(&target);
    Err(NoMap {
        why: format!(
            "The star map {} is not there; download it ({SKY_MAP_SIZE}, one of NASA's Deep Star \
             Maps) with the OK of Black Hole Lab's Look Around dialog or with sky-look \
             --fetch-map, or put {SKY_MAP_URL} into {} by hand, or name another map \
             with --sky <map.exr> or {SKY_MAP_ENV}.",
            target.display(),
            folder.display()
        ),
        fetchable: true,
    })
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
/// Both come after this user's own copy ([`user_data_dir_here`]), which is where a download puts
/// the map.
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
