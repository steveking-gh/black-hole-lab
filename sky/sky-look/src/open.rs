//! Handing the video to the system's default player, without waiting for it and without letting it
//! hold anything of this program's.
//!
//! The app reads this program's standard output and error through pipes, and gives its verdict when
//! both have closed. A pipe closes when every process holding an end of it has let go, and a
//! process started from this one can be holding an end without anyone having meant it to: on
//! Windows a child inherits every handle of its parent that is marked inheritable, not only the
//! three it is given as its standard streams. This program's own standard output and error are
//! such handles, inherited from the app. So the player is kept off them twice over: the opener is
//! started with its standard streams set to null, and before that [`keep_own_pipes_to_ourselves`]
//! has made this program's handles to the app's pipes uninheritable, so that no process started
//! from here - the opener, the player it starts, the tracer, the renderer or its ffmpeg - can hold
//! the app's pipes open after this program has exited.

use std::path::Path;
use std::process::{Command, Stdio};

/// The environment variable through which the Windows shell is given the video's path.
#[cfg(windows)]
const VIEW_VAR: &str = "BLACK_HOLE_LAB_VIEW";

/// The command that opens `video` in the default player, and returns at once.
///
/// On Windows, the shell's `start` with an empty title - `start "" "<path>"` - since `start`'s first
/// quoted argument is the window's title, and a path given first would be taken for one. The path
/// reaches `cmd` through an environment variable, expanded inside the quotes: a path can hold `&`,
/// `^`, `%` or spaces, which the shell would read as syntax if the path were spelled out on its
/// command line, and an expanded variable is not expanded again. `/D` skips any AutoRun command
/// the user's registry gives `cmd`, and `/V:OFF` keeps a `!` in the path literal even where the
/// registry turns delayed expansion on.
pub fn command(video: &Path) -> Command {
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        let shell = std::env::var_os("ComSpec").unwrap_or_else(|| "cmd.exe".into());
        let mut command = Command::new(shell);
        command
            .env(VIEW_VAR, video)
            .raw_arg(format!("/D /V:OFF /C start \"\" \"%{VIEW_VAR}%\""))
            .creation_flags(crate::run::CREATE_NO_WINDOW);
        quiet(&mut command);
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
        command.arg(video);
        // A process group of its own, so that whatever stops this program's group - a Ctrl-C at a
        // terminal - does not reach the player too.
        {
            use std::os::unix::process::CommandExt;
            command.process_group(0);
        }
        quiet(&mut command);
        command
    }
}

fn quiet(command: &mut Command) {
    command
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
}

/// Starts the opener and returns without waiting for it: `start`, `open` and `xdg-open` hand the
/// file on and exit, and the player they start runs on after this program has gone.
pub fn open(video: &Path) -> std::io::Result<()> {
    command(video).spawn().map(drop)
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
