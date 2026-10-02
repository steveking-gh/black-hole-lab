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

/// The command that installs ffmpeg on this system, as a user types it into a terminal, or `None`
/// where this program cannot tell which package manager the system has.
///
/// A command that names another system's package manager sends its reader to look for a program
/// that is not there, so the command is the one for the platform this program was built for, and
/// on Linux for the distribution that `os_release`, the text of `/etc/os-release`, names: `ID`
/// is the distribution and `ID_LIKE` the ones it derives from, so that Mint is taken for the
/// Ubuntu it is built on and Rocky for the Fedora. Fedora's own repositories carry ffmpeg as
/// `ffmpeg-free`; the package named `ffmpeg` there is RPM Fusion's, which a clean system has not
/// been told about.
pub fn ffmpeg_install_command(os_release: Option<&str>) -> Option<&'static str> {
    if cfg!(windows) {
        return Some("winget install Gyan.FFmpeg");
    }
    if cfg!(target_os = "macos") {
        return Some("brew install ffmpeg");
    }
    linux_install_command(os_release?)
}

/// The command that installs ffmpeg on the Linux distribution `os_release` describes, or `None`
/// for one whose package manager this program does not know.
pub fn linux_install_command(os_release: &str) -> Option<&'static str> {
    let family: Vec<String> = os_release
        .lines()
        .filter_map(|line| line.trim().split_once('='))
        .filter(|(key, _)| matches!(*key, "ID" | "ID_LIKE"))
        .flat_map(|(_, value)| {
            value
                .trim_matches(|c| c == '"' || c == '\'')
                .split_whitespace()
                .map(str::to_ascii_lowercase)
                .collect::<Vec<_>>()
        })
        .collect();
    let is = |names: &[&str]| family.iter().any(|name| names.contains(&name.as_str()));
    if is(&["debian", "ubuntu"]) {
        Some("sudo apt install ffmpeg")
    } else if is(&["fedora", "rhel", "centos"]) {
        Some("sudo dnf install ffmpeg-free")
    } else if is(&["arch"]) {
        Some("sudo pacman -S ffmpeg")
    } else if is(&["suse", "opensuse"]) {
        Some("sudo zypper install ffmpeg")
    } else {
        None
    }
}

/// The text of `/etc/os-release`, which says which Linux distribution this is; `None` on a system
/// that has no such file.
pub fn os_release_here() -> Option<String> {
    std::fs::read_to_string("/etc/os-release").ok()
}

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
    /// Directories ffmpeg is installed into, looked in after the PATH: [`ffmpeg_places_here`].
    pub ffmpeg_places: Vec<PathBuf>,
    /// The text of `/etc/os-release`, for the command that installs ffmpeg: [`os_release_here`].
    pub os_release: Option<String>,
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
    let sky = sky_map(search).map_err(|no_map| no_map.why)?;
    Ok(Pieces {
        trace,
        render,
        ffmpeg,
        sky,
    })
}

/// What `--check` reports: for each piece, the sentence [`find`] would give for it if it were the
/// first one missing, or `None` for a piece that is there. Every piece is looked for, where
/// [`find`] stops at the first, because the app shows what is missing before a view is asked for,
/// and a user told of one piece at a time finds out about the next only after getting the first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// `sky-trace` and `sky-render`.
    pub tools: Option<String>,
    pub ffmpeg: Option<String>,
    /// The command that installs ffmpeg on this system, when ffmpeg is missing and the command is
    /// known: for the app to show by itself, where it can be copied.
    pub ffmpeg_command: Option<&'static str>,
    pub map: Option<String>,
    /// Whether the map that is missing is the default one in its own place, which `--fetch-map`
    /// puts there; false for a map `--sky` or the variable names, which only its owner can supply.
    pub map_fetchable: bool,
}

impl Report {
    /// Whether every piece is there.
    pub fn complete(&self) -> bool {
        self.tools.is_none() && self.ffmpeg.is_none() && self.map.is_none()
    }

    /// The report as `--check` prints it, a piece a line: the piece's word (`tools`, `ffmpeg`,
    /// `map`), a space, and `ok`, or `missing` and the sentence, or for a map `--fetch-map` would
    /// supply, `fetchable` and the sentence. A fourth line, `ffmpeg-install` and the command,
    /// follows when ffmpeg is missing and the command that installs it here is known.
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
        let mut lines = vec![
            line("tools", &self.tools, "missing"),
            line("ffmpeg", &self.ffmpeg, "missing"),
            line("map", &self.map, map_word),
        ];
        if let Some(command) = self.ffmpeg_command {
            lines.push(format!("ffmpeg-install {command}"));
        }
        lines
    }
}

/// Looks for every piece and says of each whether it is there: see [`Report`].
pub fn check(search: &Search, names: &Names) -> Report {
    let map = sky_map(search).err();
    let ffmpeg = ffmpeg(search, names).err();
    Report {
        tools: tools(search, names).err(),
        ffmpeg_command: ffmpeg
            .as_ref()
            .and_then(|_| ffmpeg_install_command(search.os_release.as_deref())),
        ffmpeg,
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

/// ffmpeg: `--ffmpeg`, or the first on the PATH, or the first in `search.ffmpeg_places`.
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
        .chain(search.ffmpeg_places.iter().cloned())
        .map(|dir| dir.join(&names.ffmpeg))
        .find(|candidate| candidate.is_file())
        .ok_or_else(|| {
            let how = match ffmpeg_install_command(search.os_release.as_deref()) {
                Some(command) => format!("with {command}"),
                None => "with the system's package manager".to_string(),
            };
            format!(
                "{} was not found on the PATH, and sky-render needs it to write the photograph; \
                 install ffmpeg {how}, or name it with --ffmpeg <path>.",
                names.ffmpeg
            )
        })
}

/// The directories ffmpeg is installed into on this platform by the means its missing sentence
/// names, whether or not the PATH this program was given holds them.
///
/// It often does not. An installer adds its directory to the PATH of programs started afterwards,
/// and the app that starts this program was started before: without these places a user who
/// installs ffmpeg as the sentence says, and presses Look Around again, is told again that ffmpeg
/// is missing, until they think to restart the app. On macOS a program started from the Finder is
/// given a PATH without Homebrew's directory at all.
///
/// - Windows: winget's `Links` directory, and the `bin` of each version of the `Gyan.FFmpeg`
///   package under winget's `Packages`, the newest name first;
/// - macOS: Homebrew's two directories, for Apple silicon and for Intel, and MacPorts';
/// - elsewhere: the directories a package manager and a local install put programs in.
pub fn ffmpeg_places_here() -> Vec<PathBuf> {
    if cfg!(windows) {
        let Some(winget) = std::env::var_os("LOCALAPPDATA")
            .filter(|v| !v.is_empty())
            .map(|local| PathBuf::from(local).join("Microsoft").join("WinGet"))
        else {
            return Vec::new();
        };
        let mut versions: Vec<PathBuf> = subdirectories(&winget.join("Packages"))
            .into_iter()
            .filter(|package| {
                package
                    .file_name()
                    .is_some_and(|name| name.to_string_lossy().starts_with("Gyan.FFmpeg"))
            })
            .flat_map(|package| subdirectories(&package))
            .collect();
        versions.sort();
        versions.reverse();
        std::iter::once(winget.join("Links"))
            .chain(versions.into_iter().map(|version| version.join("bin")))
            .collect()
    } else if cfg!(target_os = "macos") {
        ["/opt/homebrew/bin", "/usr/local/bin", "/opt/local/bin"]
            .map(PathBuf::from)
            .to_vec()
    } else {
        ["/usr/bin", "/usr/local/bin", "/snap/bin"]
            .map(PathBuf::from)
            .to_vec()
    }
}

/// The directories directly inside `dir`, or none when it cannot be read.
fn subdirectories(dir: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect()
}

/// A star map that is not there.
struct NoMap {
    /// The sentence saying so, and what to do.
    why: String,
    /// Whether it is the default map in its own place, which `--fetch-map` puts there.
    fetchable: bool,
}

/// The star map: `--sky`, else the variable, else [`SKY_MAP`] where [`default_map`] says this
/// layout keeps it.
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
    let Some(exe_dir) = &search.exe_dir else {
        return Err(named_wrongly(format!(
            "sky-look cannot tell which directory it is in, so it cannot find the star map; name \
             it with --sky <map.exr> or {SKY_MAP_ENV}."
        )));
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
        format!("run pwsh {}, or ", script.display())
    } else {
        String::new()
    };
    Err(NoMap {
        why: format!(
            "The star map {} is not there; download it ({SKY_MAP_SIZE}, one of NASA's Deep Star \
             Maps) with the Download Star Map button of Black Hole Lab or with sky-look \
             --fetch-map, or {by_script}put {SKY_MAP_URL} into {} by hand, or name another map \
             with --sky <map.exr> or {SKY_MAP_ENV}.",
            map.display(),
            maps.display()
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
