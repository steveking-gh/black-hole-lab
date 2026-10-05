//! `--fetch-map`: download the default star map into this user's data folder, where the search
//! looks for it first, and check it.
//!
//! What `maps/fetch-sky.ps1` does for the one map this program uses by default, for a user who has
//! no PowerShell and no checkout: the app's Download Star Map button starts this program so. The
//! script remains the way to the other maps and sizes.
//!
//! The download is curl's, started as a child: curl is part of Windows 10 and 11 and of macOS, and
//! it brings its own TLS, which this program would otherwise have to link. Where there is no curl,
//! wget does the same work: a clean Ubuntu desktop has wget and not curl, and a first download
//! that began by asking for another program to be installed would not be one. The file is written under a name ending in `.part` and renamed only once its size
//! and its SHA-256 are the ones on record, so a download that was stopped, cut short or corrupted
//! is never found by the search and taken for the map.
//!
//! Progress is the size of the partial file against the size on record, said as sentences in the
//! way the tracer's and the renderer's percentages are, to the same status file.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

use crate::find::{self, SKY_MAP_ENV, SKY_MAP_SIZE, SKY_MAP_URL};
use crate::run::{Environment, Say};
use crate::sha256;

/// The size of the default star map in bytes, as NASA's server reported it on 2026-09-27: the
/// figure `maps/fetch-sky.ps1` holds for the same file.
pub const SKY_MAP_BYTES: u64 = 160_735_772;

/// The SHA-256 of the default star map: its line of `maps/checksums.sha256`.
pub const SKY_MAP_SHA256: &str = "d70924422d3e0159764b16a19658784befcf73b97e06e5c16614860c1514bb58";

/// The least rise in the percentage downloaded that is said as a sentence: the step the tracer's
/// and the renderer's percentages are passed on at.
const PERCENT_STEP: u64 = 5;

/// The program that does the download.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Tool {
    Curl(PathBuf),
    Wget(PathBuf),
}

impl Tool {
    /// The program's path.
    fn program(&self) -> &Path {
        match self {
            Self::Curl(program) | Self::Wget(program) => program,
        }
    }

    /// The program's name, for a sentence.
    fn name(&self) -> &'static str {
        match self {
            Self::Curl(_) => "curl",
            Self::Wget(_) => "wget",
        }
    }

    /// The first of curl and wget that is in a directory of `path`, the value of `PATH`.
    fn on(path: Option<&std::ffi::OsStr>) -> Option<Tool> {
        let exe = std::env::consts::EXE_SUFFIX;
        let find = |name: &str| {
            path.into_iter()
                .flat_map(std::env::split_paths)
                .map(|dir| dir.join(format!("{name}{exe}")))
                .find(|candidate| candidate.is_file())
        };
        find("curl")
            .map(Tool::Curl)
            .or_else(|| find("wget").map(Tool::Wget))
    }
}

/// How often the partial file's size is read while the download runs.
const POLL: Duration = Duration::from_millis(200);

/// One download: what to fetch, with what, to where, and what the file must turn out to be.
#[derive(Debug, Clone)]
pub struct Download {
    pub tool: Tool,
    pub url: String,
    pub target: PathBuf,
    pub bytes: u64,
    pub sha256: String,
}

/// Downloads the default star map to where [`find::fetch_target`] says it goes - this user's
/// data folder - and returns its path; or says in one sentence what went wrong and what to do.
pub fn fetch_default(env: &Environment, say: &mut Say) -> Result<PathBuf, String> {
    let Some(target) = find::fetch_target(env.data_dir.as_deref(), env.exe_dir.as_deref()) else {
        return Err(format!(
            "sky-look cannot tell where this user's data folder or its own directory is, so it \
             cannot tell where the star map goes; download {SKY_MAP_URL} by hand and name the \
             file with {SKY_MAP_ENV}."
        ));
    };
    let by_hand = format!(
        "put {SKY_MAP_URL} into {} by hand",
        target.parent().unwrap_or(&target).display()
    );
    let tool = Tool::on(env.path.as_deref()).ok_or_else(|| {
        format!(
            "Neither curl nor wget was found on the PATH, and sky-look downloads the star map \
             with one of them; install curl, or {by_hand}."
        )
    })?;
    download(
        &Download {
            tool,
            url: SKY_MAP_URL.into(),
            target,
            bytes: SKY_MAP_BYTES,
            sha256: SKY_MAP_SHA256.into(),
        },
        say,
    )
    .map_err(|why| format!("{why}; try again, or {by_hand}."))
}

/// Does one download and returns the file's path. A failure is the first half of a sentence, with
/// no full stop: what went wrong, for the caller to follow with what to do.
///
/// A file already at the target with the size and the hash on record is left alone and reported as
/// there. Anything else at the target is replaced, but only by a download that checks out: the
/// rename is the last step.
pub fn download(d: &Download, say: &mut Say) -> Result<PathBuf, String> {
    let name = d.target.file_name().map_or_else(
        || "the star map".into(),
        |n| n.to_string_lossy().into_owned(),
    );
    if d.target.is_file() && size_of(&d.target) == d.bytes && hash_of(&d.target)? == d.sha256 {
        say.line(&format!(
            "The star map is already at {}, and its SHA-256 matches the record.",
            d.target.display()
        ));
        return Ok(d.target.clone());
    }
    let folder = d.target.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(folder)
        .map_err(|e| format!("Could not make the folder {} ({e})", folder.display()))?;
    let mut partial = d.target.clone().into_os_string();
    partial.push(".part");
    let partial = PathBuf::from(partial);
    let _ = std::fs::remove_file(&partial);
    let discard = |why: String| {
        let _ = std::fs::remove_file(&partial);
        why
    };

    say.line(&format!(
        "Downloading the star map {name} ({SKY_MAP_SIZE}) from NASA into {}.",
        folder.display()
    ));
    // The output's flag, the partial file and the address first, so that a stand-in for either
    // program reads the last two as its second and third arguments whatever follows. curl's
    // `--silent --show-error` and wget's `--no-verbose` leave standard error holding little but the
    // reason for a failure, which is short enough to be read after the program has exited.
    let tool = d.tool.name();
    let mut command = Command::new(d.tool.program());
    match &d.tool {
        Tool::Curl(_) => command
            .arg("--output")
            .arg(&partial)
            .arg(&d.url)
            .args(["--fail", "--location", "--silent", "--show-error"])
            .args(["--retry", "3", "--connect-timeout", "30"]),
        Tool::Wget(_) => command.arg("-O").arg(&partial).arg(&d.url).args([
            "--no-verbose",
            "--tries=3",
            "--timeout=30",
        ]),
    };
    command
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(crate::run::CREATE_NO_WINDOW);
    }
    let mut child = command
        .spawn()
        .map_err(|e| format!("Could not start {} ({e})", d.tool.program().display()))?;
    let mut shown = 0;
    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) => {}
            Err(e) => return Err(discard(format!("Lost track of {tool} ({e})"))),
        }
        let percent = (size_of(&partial).saturating_mul(100) / d.bytes.max(1)).min(99);
        if percent >= shown + PERCENT_STEP {
            shown = percent;
            say.line(&format!("Downloading the star map from NASA: {percent}%."));
        }
        std::thread::sleep(POLL);
    };
    if !status.success() {
        let mut said = String::new();
        if let Some(mut pipe) = child.stderr.take() {
            let _ = std::io::Read::read_to_string(&mut pipe, &mut said);
        }
        let said = said.lines().last().unwrap_or("").trim().to_string();
        let reason = if said.is_empty() {
            match status.code() {
                Some(code) => format!("{tool} exit code {code}"),
                None => format!("{tool} was stopped"),
            }
        } else {
            said
        };
        return Err(discard(format!(
            "The download of {} failed ({reason})",
            d.url
        )));
    }
    let got = size_of(&partial);
    if got != d.bytes {
        return Err(discard(format!(
            "The download of {name} gave {got} bytes where the record has {}, and was deleted",
            d.bytes
        )));
    }
    say.line("Checking the star map against the SHA-256 on record.");
    let hash = hash_of(&partial).map_err(discard)?;
    if hash != d.sha256 {
        return Err(discard(format!(
            "The SHA-256 of the downloaded {name} is {hash} where the record has {}, so the \
             download was deleted: NASA may have replaced the file, which \
             https://svs.gsfc.nasa.gov/4851 would say",
            d.sha256
        )));
    }
    // Windows does not rename over a file that is there.
    let _ = std::fs::remove_file(&d.target);
    std::fs::rename(&partial, &d.target).map_err(|e| {
        discard(format!(
            "Could not rename the download to {} ({e})",
            d.target.display()
        ))
    })?;
    Ok(d.target.clone())
}

/// The size of the file at `path`, or 0 while there is none.
fn size_of(path: &Path) -> u64 {
    std::fs::metadata(path).map_or(0, |m| m.len())
}

/// The SHA-256 of the file at `path`.
fn hash_of(path: &Path) -> Result<String, String> {
    File::open(path)
        .and_then(|mut file| sha256::hex_of(&mut file))
        .map_err(|e| format!("Could not read {} to check it ({e})", path.display()))
}
