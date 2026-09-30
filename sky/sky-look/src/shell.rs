//! The rendering terminal: this program's own console window, and the command prompt left running
//! in it, in the view's folder, when the run ends (`--shell`).
//!
//! # Why the console is opened by name
//!
//! The app starts this program in a console window of its own (Windows' `CREATE_NEW_CONSOLE`), so
//! that the user can watch the view being made. A new console is not enough for the output to
//! reach it: the standard library always hands a child its three standard handles explicitly, and
//! a handle the app passes on is whatever the app's own is - the null device, a pipe, or the
//! console window the app itself was started in, which would get this program's lines instead of
//! the new window. So with `--shell` this program does not trust the standard handles it was
//! given. [`use_own_console`] opens `CONOUT$` and `CONIN$`, which always name the console the
//! calling process is attached to, and makes them its standard streams; the standard library looks
//! the handle up afresh on every write, so everything printed from then on reaches the window.
//!
//! # Why the prompt is not waited for
//!
//! A console window stays open for as long as some process is attached to it. The command prompt
//! is started attached to this program's console, with the console's own input and output as its
//! standard streams, and this program then exits without waiting: the window stays, with the
//! prompt in it, until the user closes it, and the app, which has already read the run's verdict
//! from the status file, is not kept waiting on a prompt nobody may ever close. Each view gets its
//! own window, so windows accumulate until the user closes them; that is the point of asking for
//! them.
//!
//! The prompt is `cmd.exe` (`%ComSpec%`) with `/K`, which runs nothing and stays. PowerShell was the
//! other choice and was not taken: it takes a second or more to start where `cmd` is instant,
//! and the one thing a user is likely to type first - `dir`, or `type log.txt` - reads the same
//! in both. The lines in `commands.txt` are written for PowerShell, and `powershell` typed at the
//! prompt starts it there in the same folder.
//!
//! Elsewhere than Windows there is no second console to open. `--shell` there starts the user's
//! `$SHELL` (else `/bin/sh`) in the folder on this program's own terminal and waits for it, since a
//! shell left running behind an exited parent would fight the parent's shell for the terminal.

use std::path::Path;
use std::process::Command;

/// Takes this program's console window for its standard input, output and error, whatever handles
/// it was started with. Nothing changes where there is no console to open.
#[cfg(windows)]
pub fn use_own_console() {
    use std::ffi::c_void;
    use std::os::windows::io::IntoRawHandle;

    // STD_INPUT_HANDLE, STD_OUTPUT_HANDLE and STD_ERROR_HANDLE: (DWORD)-10, -11 and -12.
    const STD_INPUT: u32 = -10i32 as u32;
    const STD_OUTPUT: u32 = -11i32 as u32;
    const STD_ERROR: u32 = -12i32 as u32;
    unsafe extern "system" {
        fn SetStdHandle(which: u32, handle: *mut c_void) -> i32;
    }
    let open = |name: &str| {
        std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(name)
    };
    for (name, which) in [
        ("CONOUT$", STD_OUTPUT),
        ("CONOUT$", STD_ERROR),
        ("CONIN$", STD_INPUT),
    ] {
        if let Ok(file) = open(name) {
            // SAFETY: the handle is a console handle this process has just opened, and is handed
            // over for good: it is never closed, and stays valid for the life of the process, as
            // a standard handle must.
            unsafe {
                SetStdHandle(which, file.into_raw_handle().cast());
            }
        }
    }
}

/// Elsewhere the terminal a program runs in is already its standard streams' own.
#[cfg(not(windows))]
pub fn use_own_console() {}

/// The command that starts the prompt in `dir`: see the module's notes.
pub fn command(dir: &Path) -> Command {
    #[cfg(windows)]
    let mut command = {
        let shell = std::env::var_os("ComSpec")
            .filter(|v| !v.is_empty())
            .unwrap_or_else(|| "cmd.exe".into());
        let mut command = Command::new(shell);
        command.arg("/K");
        command
    };
    #[cfg(not(windows))]
    let mut command = Command::new(
        std::env::var_os("SHELL")
            .filter(|v| !v.is_empty())
            .unwrap_or_else(|| "/bin/sh".into()),
    );
    command.current_dir(dir);
    command
}

/// Starts the prompt in `dir`, on this program's console window, and returns: at once on Windows,
/// and when the user leaves the shell elsewhere.
pub fn start(dir: &Path) -> std::io::Result<()> {
    let mut command = command(dir);
    #[cfg(windows)]
    {
        // The console's own handles, opened for the prompt, rather than this program's standard
        // streams: those were made uninheritable at the start (`open::keep_own_pipes_to_ourselves`),
        // and whatever the standard library would pass on for them, the console by name is the
        // window the prompt has to read from and write to. Where there is no console the prompt
        // is given what this program has.
        let open = |name: &str| {
            std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(name)
        };
        if let (Ok(input), Ok(output), Ok(error)) =
            (open("CONIN$"), open("CONOUT$"), open("CONOUT$"))
        {
            command.stdin(input).stdout(output).stderr(error);
        }
        command.spawn().map(drop)
    }
    #[cfg(not(windows))]
    {
        command.status().map(drop)
    }
}
