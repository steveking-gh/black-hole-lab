//! Where a view's files go and what they are called.
//!
//! Every view gets a folder of its own in the views directory, named
//!
//! ```text
//! 2026-09-27 21.35.03.456 UTC Bob at tau 12.345 M
//! ```
//!
//! the moment the view was made, then whose it is and the reading of that observer's watch (the
//! proper time the app shows as τ) at the saved moment. The time comes first so that a listing by
//! name is a listing by age, and it is UTC so that the order survives the clocks going back in the
//! autumn, when an hour of local times repeats. Milliseconds tell apart two presses in one second;
//! two in one millisecond (two copies of the app) are told apart by [`make_folder`], which never
//! takes a folder that is there.
//!
//! Everything of the view is in its folder, and nothing in it is ever deleted by this program or
//! by the app, whether the view was made or not: the photograph, named as the folder is with
//! `.jpg` after it so that a search across folders still reads; the save it was made from, with
//! `.bhl`; the traced sky bundle in `bundle`; the commands that were run, in `commands.txt`; and
//! every line of progress, in `log.txt`. A folder of its own rather than files side by side,
//! because a view is now five things, and a bundle alone is a folder of its own already.

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
///
/// A photograph in the Videos folder wants a reason, and the reason is history: until 2026-09-27
/// a view was a video held still for a minute, with the photograph beside it, and the owner's
/// earlier views are in this directory. Moving the new ones to the Pictures folder would split one
/// collection of views, named to sort by the time they were made, into two places.
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

/// The name of a view's file, without the extension: when, whose, and the watch's reading.
///
/// `tau` is left out when the save did not say it (the tracer, which reads the save properly, has
/// by then accepted it, so this is a save this program's lighter reading did not follow). The
/// reading is written as the photograph's read-outs write the watch, to three decimals, with the
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

/// Makes the folder of one view in `dir`: `stem`, or `stem (2)` and so on when that name is taken.
/// A folder already there is never used, so two views never share one.
///
/// `create_dir` claims the name and makes the folder in one step, failing if the name is taken,
/// so two runs racing for one name - two copies of the app pressed in the same millisecond - get
/// two folders, with no moment at which both could think they had the same one.
pub fn make_folder(dir: &Path, stem: &str) -> std::io::Result<PathBuf> {
    for n in 1..=1000u32 {
        let name = if n == 1 {
            stem.to_string()
        } else {
            format!("{stem} ({n})")
        };
        let folder = dir.join(name);
        match std::fs::create_dir(&folder) {
            Ok(()) => return Ok(folder),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e),
        }
    }
    Err(std::io::Error::other(format!(
        "a thousand folders in {} already start with {stem}",
        dir.display()
    )))
}

/// Moves the file `from` to `to`, a name that must not be taken: renamed within a volume, and
/// across volumes (the temporary directory on one drive, the views on another) copied and then
/// deleted. A copy that fails part-way is removed and `from` is left as it was, so the file is in
/// one place or the other and never lost.
pub fn move_file(from: &Path, to: &Path) -> std::io::Result<()> {
    if to.exists() {
        return Err(std::io::ErrorKind::AlreadyExists.into());
    }
    if std::fs::rename(from, to).is_ok() {
        return Ok(());
    }
    if let Err(e) = std::fs::copy(from, to) {
        let _ = std::fs::remove_file(to);
        return Err(e);
    }
    // The copy is whole; a source that cannot be deleted is left for whoever owns it.
    let _ = std::fs::remove_file(from);
    Ok(())
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
