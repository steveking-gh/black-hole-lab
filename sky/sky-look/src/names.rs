//! Where the finished files go and what they are called.
//!
//! A view's files are named
//!
//! ```text
//! 2026-09-27 21.35.03.456 UTC Bob at tau 12.345 M.mkv
//! ```
//!
//! the moment the view was made, then whose it is and the reading of that observer's watch (the
//! proper time the app shows as τ) at the saved moment. The time comes first so that a listing by
//! name is a listing by age, and it is UTC so that the order survives the clocks going back in the
//! autumn, when an hour of local times repeats. Milliseconds tell apart two presses in one second;
//! two in one millisecond (two copies of the app) are told apart by [`place`], which never writes
//! over a file that is there.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

/// The environment variable that names the directory for the views, ahead of the Videos folder.
pub const VIEWS_ENV: &str = "BLACK_HOLE_LAB_VIEWS";

/// The directory made for the views, in the Videos folder or the home directory.
pub const VIEWS_DIR: &str = "Black Hole Lab views";

/// The longest observer name put into a file name, in characters. The app's names are three and
/// five; the limit is for whatever a later save might carry, and keeps the whole path well inside
/// Windows' 260.
const MAX_NAME: usize = 40;

/// The directory the views go into: `--out-dir`, else [`VIEWS_ENV`], else [`VIEWS_DIR`] in the
/// Videos folder if there is one, else in the home directory.
pub fn out_dir(
    flag: Option<&Path>,
    env: Option<&std::ffi::OsStr>,
    videos: Option<&Path>,
    home: Option<&Path>,
) -> Result<PathBuf, String> {
    if let Some(dir) = flag {
        return Ok(dir.to_path_buf());
    }
    if let Some(dir) = env.filter(|v| !v.is_empty()) {
        return Ok(PathBuf::from(dir));
    }
    if let Some(videos) = videos.filter(|v| v.is_dir()) {
        return Ok(videos.join(VIEWS_DIR));
    }
    match home {
        Some(home) => Ok(home.join(VIEWS_DIR)),
        None => Err(format!(
            "There is no Videos folder or home directory to put the view in; name a directory \
             with --out-dir <dir> or {VIEWS_ENV}."
        )),
    }
}

/// The name of a view's files, without the extension: when, whose, and the watch's reading.
///
/// `tau` is left out when the save did not say it (the tracer, which reads the save properly, has
/// by then accepted it, so this is a save this program's lighter reading did not follow). The
/// reading is written as the video's read-outs write the stopwatch, to three decimals, with the
/// decimal mark the app was set to.
pub fn stem(when: SystemTime, who: &str, tau: Option<f64>, comma: bool) -> String {
    let mut name = format!("{} {}", utc_stamp(when), safe_name(who));
    if let Some(tau) = tau.filter(|t| t.is_finite()) {
        let reading = format!("{tau:.3}");
        let reading = if comma {
            reading.replace('.', ",")
        } else {
            reading
        };
        name.push_str(&format!(" at tau {reading} M"));
    }
    name
}

/// `YYYY-MM-DD HH.MM.SS.mmm UTC`: fixed width, so that names sort as times do, with no colon,
/// which Windows does not allow in a file name.
pub fn utc_stamp(when: SystemTime) -> String {
    // A clock set before 1970 is written as 1970: the name still sorts, and is still a name.
    let since = when.duration_since(UNIX_EPOCH).unwrap_or_default();
    let seconds = since.as_secs();
    let (year, month, day) = civil_from_days((seconds / 86_400) as i64);
    let of_day = seconds % 86_400;
    format!(
        "{year:04}-{month:02}-{day:02} {:02}.{:02}.{:02}.{:03} UTC",
        of_day / 3600,
        of_day / 60 % 60,
        of_day % 60,
        since.subsec_millis()
    )
}

/// The proleptic Gregorian date of a day counted from 1970-01-01 (Howard Hinnant's
/// `civil_from_days`, which is exact for every day an `i64` can count here).
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

/// An observer's name as it can stand in a file name on every system this runs on: the characters
/// Windows forbids (`<>:"/\|?*`), control characters and `%` (which the Windows shell would read
/// as the start of a variable when the file is opened) become `_`; trailing dots and spaces, which
/// Windows silently drops, go; and a name left empty becomes "observer". The name never stands at
/// the start of the file name, so a device name such as `CON` is harmless.
pub fn safe_name(who: &str) -> String {
    let cleaned: String = who
        .chars()
        .map(|c| {
            if c.is_control() || "<>:\"/\\|?*%".contains(c) {
                '_'
            } else {
                c
            }
        })
        .take(MAX_NAME)
        .collect();
    let trimmed = cleaned.trim().trim_end_matches(['.', ' ']).trim();
    if trimmed.is_empty() {
        "observer".into()
    } else {
        trimmed.to_string()
    }
}

/// The files of one view, where they ended up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Placed {
    pub video: PathBuf,
    pub photo: PathBuf,
}

/// Moves a finished video and photograph from the scratch directory into `dir`, named `stem` and
/// its extensions, or `stem (2)` and so on when either name is taken: a file already there is
/// never written over.
///
/// Each file arrives whole or not at all. Within one volume a file is hard-linked to its new name,
/// which fails if the name is taken, and so claims the name and writes the file in one step with
/// no moment at which another run could take it too; the scratch directory's link is deleted with
/// the scratch directory. Across volumes (an output directory on another drive) the file is copied
/// under a `.partial` name and renamed when it is whole, and the one thing a run stopped then can
/// leave in the output directory is that `.partial` file.
pub fn place(video: &Path, photo: &Path, dir: &Path, stem: &str) -> std::io::Result<Placed> {
    for n in 1..=1000u32 {
        let name = if n == 1 {
            stem.to_string()
        } else {
            format!("{stem} ({n})")
        };
        let placed = Placed {
            video: dir.join(format!("{name}.mkv")),
            photo: dir.join(format!("{name}.jpg")),
        };
        if placed.video.exists() || placed.photo.exists() {
            continue;
        }
        // The photograph first, so that the video - the file the app is told about - is the last
        // to arrive, and a name whose video is there has its photograph beside it.
        match claim(photo, &placed.photo) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e),
        }
        match claim(video, &placed.video) {
            Ok(()) => return Ok(placed),
            Err(e) => {
                let _ = std::fs::remove_file(&placed.photo);
                if e.kind() == std::io::ErrorKind::AlreadyExists {
                    continue;
                }
                return Err(e);
            }
        }
    }
    Err(std::io::Error::other(format!(
        "a thousand files in {} already start with {stem}",
        dir.display()
    )))
}

/// Gives `from` the name `to`, failing with `AlreadyExists` if `to` is taken.
fn claim(from: &Path, to: &Path) -> std::io::Result<()> {
    match std::fs::hard_link(from, to) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => Err(e),
        // Another volume, or a file system without hard links: copy, whole, then rename. The
        // check and the rename are not one step, so two runs racing for one name across volumes
        // could meet here; the millisecond in the name makes that a race nobody runs.
        Err(_) => {
            let partial = crate::scratch::with_suffix(to, ".partial");
            let copied = std::fs::copy(from, &partial).and_then(|_| {
                if to.exists() {
                    Err(std::io::ErrorKind::AlreadyExists.into())
                } else {
                    std::fs::rename(&partial, to)
                }
            });
            if copied.is_err() {
                let _ = std::fs::remove_file(&partial);
            }
            copied
        }
    }
}

/// The user's Videos folder, where the system says it is: on Windows the known folder, which
/// OneDrive or the user may have moved; on macOS `~/Movies`; elsewhere `XDG_VIDEOS_DIR` or
/// `~/Videos`. `None` when there is none.
pub fn videos_folder(home: Option<&Path>) -> Option<PathBuf> {
    #[cfg(windows)]
    {
        known_videos_folder().or_else(|| home.map(|h| h.join("Videos")))
    }
    #[cfg(target_os = "macos")]
    {
        home.map(|h| h.join("Movies"))
    }
    #[cfg(not(any(windows, target_os = "macos")))]
    {
        std::env::var_os("XDG_VIDEOS_DIR")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
            .or_else(|| home.map(|h| h.join("Videos")))
    }
}

/// Windows' own answer for FOLDERID_Videos. The folder can be redirected (OneDrive's folder
/// backup does it, and so does the folder's Location tab), and `%USERPROFILE%\Videos` would then
/// name an empty leftover rather than the folder the user sees as Videos.
#[cfg(windows)]
fn known_videos_folder() -> Option<PathBuf> {
    use std::ffi::c_void;
    use std::os::windows::ffi::OsStringExt;

    #[repr(C)]
    struct Guid {
        data1: u32,
        data2: u16,
        data3: u16,
        data4: [u8; 8],
    }
    // FOLDERID_Videos, {18989B1D-99B5-455B-841C-AB7C74E4DDFC}, from KnownFolders.h.
    const FOLDERID_VIDEOS: Guid = Guid {
        data1: 0x1898_9B1D,
        data2: 0x99B5,
        data3: 0x455B,
        data4: [0x84, 0x1C, 0xAB, 0x7C, 0x74, 0xE4, 0xDD, 0xFC],
    };
    #[link(name = "shell32")]
    unsafe extern "system" {
        fn SHGetKnownFolderPath(
            rfid: *const Guid,
            flags: u32,
            token: *mut c_void,
            path: *mut *mut u16,
        ) -> i32;
    }
    #[link(name = "ole32")]
    unsafe extern "system" {
        fn CoTaskMemFree(block: *mut c_void);
    }
    let mut path: *mut u16 = std::ptr::null_mut();
    // SAFETY: the GUID and the out-pointer are valid for the call; on success the shell hands back
    // a NUL-terminated wide string that the caller must free with CoTaskMemFree, and the string is
    // copied out before it is freed. On failure the pointer is null or must still be freed, and
    // CoTaskMemFree accepts null.
    let found = unsafe {
        let status = SHGetKnownFolderPath(&FOLDERID_VIDEOS, 0, std::ptr::null_mut(), &mut path);
        let found = (status >= 0 && !path.is_null()).then(|| {
            let len = (0..).take_while(|&i| *path.add(i) != 0).count();
            std::ffi::OsString::from_wide(std::slice::from_raw_parts(path, len))
        });
        CoTaskMemFree(path.cast());
        found
    };
    found.map(PathBuf::from).filter(|p| p.is_dir())
}
