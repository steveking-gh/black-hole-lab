//! Handing the photograph to a viewer, without waiting for the viewer and without letting it hold
//! anything of this program's.
//!
//! # Which viewer
//!
//! A 360-degree photograph shown flat is one wide picture of a sky stretched out of shape; shown by
//! a viewer that pans, it is the sky round the observer, looked at a part at a time. The program a
//! system opens `.jpg` files with may be either kind: on the owner's machine it is Windows Photos,
//! which shows the photograph flat, and VLC (3.0.23 tried) pans it. So [`choose`] prefers, in
//! order: the program `--viewer` names; else the one [`VIEWER_ENV`] names; else VLC, where VLC
//! installs itself; else the system's default program for the file, with a sentence saying that
//! program may show the photograph flat and VLC would pan it. A viewer that is named and is not
//! there is reported, never passed over for the next choice: whoever named it meant that program,
//! and quietly starting another would hide the mistake. [`choose`] is a pure function of a
//! [`ViewerSearch`] - what is named, and where VLC would be - so that the tests can run it against
//! a directory layout made for the purpose, as `find` is.
//!
//! VLC shows a picture for 10 seconds and then stops, unless it is told otherwise; its image
//! module (`plugins/demux/libimage_plugin.dll`) documents for `--image-duration` that "a negative
//! value means an unlimited play time". So VLC, and a program named as the viewer whose file is
//! called `vlc`, is started with `--image-duration=-1` before the path, and with no other option.
//!
//! # Holding nothing of this program's
//!
//! A caller that reads this program's standard output and error through pipes - a script, and the
//! app itself until it began reading a status file instead - learns that the run is over when both
//! have closed. A pipe closes when every process holding an end of it has let go, and a
//! process started from this one can be holding an end without anyone having meant it to: on
//! Windows a child inherits every handle of its parent that is marked inheritable, not only the
//! three it is given as its standard streams. This program's own standard output and error are
//! such handles, inherited from the caller. So the viewer is kept off them twice over: the viewer, or
//! the shell that starts the system's default, is started with its standard streams set to null,
//! and before that [`keep_own_pipes_to_ourselves`] has made this program's handles to the app's
//! pipes uninheritable, so that no process started from here - the viewer, the tracer or the
//! renderer - can hold the caller's pipes open after this program has exited.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// The environment variable that names the viewer, after `--viewer` and ahead of VLC.
pub const VIEWER_ENV: &str = "BLACK_HOLE_LAB_VIEWER";

/// The option VLC is given before the path: show the picture until the window is closed.
pub const VLC_FOREVER: &str = "--image-duration=-1";

/// The environment variable through which the Windows shell is given the photograph's path.
#[cfg(windows)]
const VIEW_VAR: &str = "BLACK_HOLE_LAB_VIEW";

/// Where to look for a viewer, and what was named outright.
#[derive(Debug, Clone, Default)]
pub struct ViewerSearch {
    /// `--viewer <program>`.
    pub flag: Option<PathBuf>,
    /// The value of [`VIEWER_ENV`].
    pub env: Option<OsString>,
    /// Where VLC would be, in the order to try: [`vlc_places`] for this machine.
    pub vlc: Vec<PathBuf>,
}

/// The program that is to show the photograph.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Viewer {
    /// A program named outright; `by` is what named it, `--viewer` or [`VIEWER_ENV`].
    Named { program: PathBuf, by: &'static str },
    /// VLC, found where it installs itself.
    Vlc(PathBuf),
    /// The program the system opens `.jpg` files with.
    Default,
}

/// Chooses the viewer, or says in a clause (to follow "but") why the one named cannot be used.
pub fn choose(search: &ViewerSearch) -> Result<Viewer, String> {
    if let Some(named) = &search.flag {
        return if named.is_file() {
            Ok(Viewer::Named {
                program: named.clone(),
                by: "--viewer",
            })
        } else {
            Err(format!(
                "--viewer names {}, and there is no program there; give the full path of the \
                 viewer, or leave --viewer out to open the photograph in VLC or the system's \
                 default program",
                named.display()
            ))
        };
    }
    if let Some(named) = search.env.as_deref().filter(|v| !v.is_empty()) {
        let named = PathBuf::from(named);
        return if named.is_file() {
            Ok(Viewer::Named {
                program: named,
                by: VIEWER_ENV,
            })
        } else {
            Err(format!(
                "{VIEWER_ENV} names {}, and there is no program there; correct the variable, or \
                 remove it to open the photograph in VLC or the system's default program",
                named.display()
            ))
        };
    }
    Ok(search
        .vlc
        .iter()
        .find(|place| place.is_file())
        .map_or(Viewer::Default, |vlc| Viewer::Vlc(vlc.clone())))
}

/// Where VLC would be on this machine, in the order to try, from what the system says: a pure
/// function of its inputs, which [`vlc_places_here`] reads from the process.
///
/// On Windows: `vlc.exe` in the directory VLC's installer wrote to the registry (`registered`),
/// then under `VideoLAN\VLC` in each of `program_files` (`%ProgramFiles%`, then
/// `%ProgramFiles(x86)%`, for a 32-bit VLC). On macOS the application bundle's program. Elsewhere
/// `vlc` in each directory of `path`.
pub fn vlc_places(
    registered: &[PathBuf],
    program_files: &[PathBuf],
    path: Option<&std::ffi::OsStr>,
) -> Vec<PathBuf> {
    if cfg!(windows) {
        let _ = path;
        registered
            .iter()
            .map(|dir| dir.join("vlc.exe"))
            .chain(
                program_files
                    .iter()
                    .map(|dir| dir.join("VideoLAN").join("VLC").join("vlc.exe")),
            )
            .collect()
    } else if cfg!(target_os = "macos") {
        let _ = (registered, program_files, path);
        vec![PathBuf::from("/Applications/VLC.app/Contents/MacOS/VLC")]
    } else {
        let _ = (registered, program_files);
        path.into_iter()
            .flat_map(std::env::split_paths)
            .map(|dir| dir.join("vlc"))
            .collect()
    }
}

/// [`vlc_places`] for this process: the registry, the two Program Files directories, the PATH.
pub fn vlc_places_here() -> Vec<PathBuf> {
    let program_files: Vec<PathBuf> = ["ProgramFiles", "ProgramFiles(x86)"]
        .iter()
        .filter_map(std::env::var_os)
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .collect();
    vlc_places(
        &registered_vlc(),
        &program_files,
        std::env::var_os("PATH").as_deref(),
    )
}

/// Whether `program` is VLC by its file name, so that a VLC named as the viewer is given the
/// option that keeps the picture up.
fn is_vlc(program: &Path) -> bool {
    program
        .file_stem()
        .is_some_and(|stem| stem.eq_ignore_ascii_case("vlc"))
}

/// The command that opens `photo` in `viewer`, and returns at once.
///
/// A program chosen by name or found as VLC is started directly, `program [--image-duration=-1]
/// <path>`, with the path one argument: no shell reads the command line, so nothing in the path
/// (`&`, `^`, `%`, `!`, spaces) can be taken for syntax.
///
/// The system's default program is reached through the system's opener. On Windows that is the
/// shell's `start` with an empty title - `start "" "<path>"` - since `start`'s first quoted
/// argument is the window's title, and a path given first would be taken for one. The path
/// reaches `cmd` through an environment variable, expanded inside the quotes: a path can hold `&`,
/// `^`, `%` or spaces, which the shell would read as syntax if the path were spelled out on its
/// command line, and an expanded variable is not expanded again. `/D` skips any AutoRun command
/// the user's registry gives `cmd`, and `/V:OFF` keeps a `!` in the path literal even where the
/// registry turns delayed expansion on. Elsewhere `open` (macOS) or `xdg-open`.
///
/// Every one of them is started with `CREATE_NO_WINDOW` on Windows. The flag gives a console
/// program no console window, so neither `cmd` nor a console program named as the viewer flashes a
/// black window up; Windows ignores the flag for a program that is not a console program, and VLC
/// is not one (`vlc.exe` 3.0.23 is marked for the Windows GUI subsystem), so VLC's own window opens
/// as it always does.
pub fn command(viewer: &Viewer, photo: &Path) -> Command {
    let mut command = match viewer {
        Viewer::Named { program, .. } | Viewer::Vlc(program) => {
            let mut command = Command::new(program);
            if matches!(viewer, Viewer::Vlc(_)) || is_vlc(program) {
                command.arg(VLC_FOREVER);
            }
            command.arg(photo);
            command
        }
        Viewer::Default => default_opener(photo),
    };
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(crate::run::CREATE_NO_WINDOW);
    }
    #[cfg(not(windows))]
    {
        // A process group of its own, so that whatever stops this program's group - a Ctrl-C at a
        // terminal - does not reach the viewer too.
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    command
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    command
}

/// The system's opener, handed `photo`: see [`command`].
fn default_opener(photo: &Path) -> Command {
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        let shell = std::env::var_os("ComSpec").unwrap_or_else(|| "cmd.exe".into());
        let mut command = Command::new(shell);
        command
            .env(VIEW_VAR, photo)
            .raw_arg(format!("/D /V:OFF /C start \"\" \"%{VIEW_VAR}%\""));
        command
    }
    #[cfg(not(windows))]
    {
        let program = if cfg!(target_os = "macos") {
            "open"
        } else {
            "xdg-open"
        };
        let mut command = Command::new(program);
        command.arg(photo);
        command
    }
}

/// Starts the viewer on `photo` and returns without waiting for it: VLC or a named viewer runs on
/// after this program has gone, and `start`, `open` and `xdg-open` hand the file on and exit.
pub fn open(viewer: &Viewer, photo: &Path) -> std::io::Result<()> {
    command(viewer, photo).spawn().map(drop)
}

/// The progress sentence once `viewer` has been started, naming the program.
pub fn opened_sentence(viewer: &Viewer) -> String {
    const PAN: &str = "drag with the mouse to look in every direction";
    match viewer {
        Viewer::Named { program, by } if is_vlc(program) => format!(
            "Opened the photograph in {}, the viewer {by} names; {PAN}.",
            program.display()
        ),
        Viewer::Named { program, by } => format!(
            "Opened the photograph in {}, the viewer {by} names.",
            program.display()
        ),
        Viewer::Vlc(program) => format!(
            "Opened the photograph in VLC ({}); {PAN}.",
            program.display()
        ),
        Viewer::Default => "Opened the photograph in the program the system opens .jpg files \
                            with, which may show the photograph flat, as one wide picture; VLC \
                            pans a 360-degree photograph, and sky-look opens the next view in VLC \
                            once VLC is installed."
            .into(),
    }
}

/// What the viewer is called in a sentence saying it could not be started.
pub fn viewer_name(viewer: &Viewer) -> String {
    match viewer {
        Viewer::Named { program, .. } | Viewer::Vlc(program) => program.display().to_string(),
        Viewer::Default => "the program the system opens .jpg files with".into(),
    }
}

/// The directories VLC's installer recorded, from `InstallDir` under `HKLM\SOFTWARE\VideoLAN\VLC`:
/// in the registry's own view for this program first, and then in the 32-bit view, where a 32-bit
/// VLC on a 64-bit Windows records itself. None, one or two, in that order.
#[cfg(windows)]
pub fn registered_vlc() -> Vec<PathBuf> {
    use std::ffi::c_void;
    use std::os::windows::ffi::{OsStrExt, OsStringExt};

    // HKEY_LOCAL_MACHINE is the 32-bit value 0x80000002 sign-extended to a pointer, as WinReg.h
    // defines it: ((HKEY)(ULONG_PTR)((LONG)0x80000002)).
    let hklm = 0x8000_0002_u32 as i32 as isize as *mut c_void;
    // RRF_RT_REG_SZ: only a string, which RegGetValueW returns NUL-terminated.
    const RRF_RT_REG_SZ: u32 = 0x0000_0002;
    // RRF_SUBKEY_WOW6432KEY: the 32-bit view of the registry.
    const RRF_SUBKEY_WOW6432KEY: u32 = 0x0002_0000;
    #[link(name = "advapi32")]
    unsafe extern "system" {
        fn RegGetValueW(
            key: *mut c_void,
            sub_key: *const u16,
            value: *const u16,
            flags: u32,
            kind: *mut u32,
            data: *mut c_void,
            bytes: *mut u32,
        ) -> i32;
    }
    let wide = |s: &str| {
        std::ffi::OsStr::new(s)
            .encode_wide()
            .chain([0])
            .collect::<Vec<u16>>()
    };
    let (sub_key, value) = (wide(r"SOFTWARE\VideoLAN\VLC"), wide("InstallDir"));
    let read = |flags: u32| -> Option<PathBuf> {
        let mut bytes: u32 = 0;
        // SAFETY: the key is a predefined handle; the two names are NUL-terminated wide strings
        // that outlive the calls. The first call passes no buffer and is told the size in bytes;
        // the second passes a buffer of at least that size and says so in `bytes`. Anything but
        // success (0) - no such key or value, a value that is not a string, a value that grew
        // between the calls - is taken for no answer.
        unsafe {
            let status = RegGetValueW(
                hklm,
                sub_key.as_ptr(),
                value.as_ptr(),
                RRF_RT_REG_SZ | flags,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                &mut bytes,
            );
            if status != 0 || bytes < 2 {
                return None;
            }
            let mut buffer = vec![0u16; (bytes as usize).div_ceil(2)];
            let status = RegGetValueW(
                hklm,
                sub_key.as_ptr(),
                value.as_ptr(),
                RRF_RT_REG_SZ | flags,
                std::ptr::null_mut(),
                buffer.as_mut_ptr().cast(),
                &mut bytes,
            );
            if status != 0 {
                return None;
            }
            let len = buffer.iter().position(|&c| c == 0).unwrap_or(buffer.len());
            (len > 0).then(|| PathBuf::from(OsString::from_wide(&buffer[..len])))
        }
    };
    let mut found: Vec<PathBuf> = Vec::new();
    for dir in [read(0), read(RRF_SUBKEY_WOW6432KEY)].into_iter().flatten() {
        if !found.contains(&dir) {
            found.push(dir);
        }
    }
    found
}

/// No registry elsewhere.
#[cfg(not(windows))]
pub fn registered_vlc() -> Vec<PathBuf> {
    Vec::new()
}

/// Marks this program's standard output and error as not to be inherited by what it starts. The
/// handles still work for this program; only children no longer receive copies of them.
#[cfg(windows)]
pub fn keep_own_pipes_to_ourselves() {
    use std::ffi::c_void;
    use std::os::windows::io::AsRawHandle;

    const HANDLE_FLAG_INHERIT: u32 = 1;
    unsafe extern "system" {
        fn SetHandleInformation(handle: *mut c_void, mask: u32, flags: u32) -> i32;
    }
    for handle in [
        std::io::stdout().as_raw_handle(),
        std::io::stderr().as_raw_handle(),
    ] {
        if !handle.is_null() {
            // SAFETY: the handle is this process's own standard stream, open for the life of the
            // process; clearing its inherit flag changes nothing but what a child is given. A
            // handle that is not a kernel object (none, or a console pseudo-handle) makes the call
            // fail, which is harmless and ignored.
            unsafe {
                SetHandleInformation(handle.cast(), HANDLE_FLAG_INHERIT, 0);
            }
        }
    }
}

/// Elsewhere every descriptor this program opens is close-on-exec, and a child's standard streams
/// are the ones it is given, so there is nothing to do.
#[cfg(not(windows))]
pub fn keep_own_pipes_to_ourselves() {}
