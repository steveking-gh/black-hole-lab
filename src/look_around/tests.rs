//! What Look Around has to be true of, tested without the sky tools.
//!
//! None of these tests runs the real `sky-look`, and none of them depends on its having been built.
//! The search is tested as the pure function it is, against directory layouts made for the purpose.
//! Everything that runs a child runs a stand-in: a script written by the test into a scratch
//! directory of its own - a batch file on Windows, a shell script elsewhere - which does what the
//! test tells it to do, in order: append a line to the status file it was handed, wait for a file
//! to appear, exit with a chosen code. A script rather than a test binary because a test binary is
//! driven by libtest's own command line, which the contract's `<file.bhl> --observer bob --status
//! <file> ...` would be refused by, and because `cmd.exe` and `sh` are on every machine these tests
//! run on. The waits are handshakes on files rather than sleeps, so that a test says "now" instead
//! of guessing how long a line takes to arrive.
//!
//! Every stand-in also copies the save it was handed, the path of the status file and the other
//! arguments, before anything else, so that a test can check what the child actually received.
//! None of these tests starts the rendering terminal, which would open a console window on the
//! desktop of whoever runs them: what the app gives `sky-look` for it is tested as the command
//! line and the flags it is.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use super::*;
use crate::app::SpacetimeApp;
use crate::gui::controls::{FileStatus, LookStatus, look_around_blocked};
use crate::perf::replay::play_sim;
use crate::physics::observer::ObserverMode;

/// A directory of its own for one test, empty, in the system's temporary directory.
fn scratch_dir(what: &str) -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let dir = std::env::temp_dir().join(format!(
        "black-hole-lab-look-test-{}-{}-{what}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("a scratch directory");
    dir
}

/// One thing a stand-in does.
enum Step<'a> {
    /// Append a line to the status file.
    Say(&'a str),
    /// Wait until this file exists.
    WaitFor(&'a Path),
    /// Write this file, as proof of having got this far.
    Touch(&'a Path),
    /// Exit with this code.
    Exit(i32),
}

/// Write a stand-in for `sky-look` into `dir` that copies the save it is handed to `seen.bhl`, the
/// path of the status file to `status-path.txt` and the rest of its arguments to `args.txt`, and
/// then does `steps`.
///
/// The lines the tests write hold letters, spaces, full stops, colons and backslashes and nothing
/// either shell would read as syntax, so they are written out unquoted by `cmd` and in single
/// quotes by `sh`.
fn stand_in(dir: &Path, steps: &[Step]) -> PathBuf {
    let seen = dir.join("seen.bhl");
    let args = dir.join("args.txt");
    let status_path = dir.join("status-path.txt");
    let mut script = String::new();
    if cfg!(windows) {
        script.push_str("@echo off\r\n");
        script.push_str(&format!("copy /y \"%~1\" \"{}\" >nul\r\n", seen.display()));
        script.push_str(&format!(">\"{}\" echo %~5\r\n", status_path.display()));
        script.push_str(&format!(">\"{}\" echo %2 %3 %4 %6 %7 %8\r\n", args.display()));
        for (index, step) in steps.iter().enumerate() {
            match step {
                // `cmd` reads `%` as the start of a variable, so it is doubled.
                Step::Say(line) => script
                    .push_str(&format!(">>\"%~5\" echo {}\r\n", line.replace('%', "%%"))),
                Step::WaitFor(file) => script.push_str(&format!(
                    ":wait{index}\r\nif not exist \"{}\" goto wait{index}\r\n",
                    file.display()
                )),
                Step::Touch(file) => {
                    script.push_str(&format!(">\"{}\" echo here\r\n", file.display()))
                }
                Step::Exit(code) => script.push_str(&format!("exit {code}\r\n")),
            }
        }
    } else {
        script.push_str("#!/bin/sh\n");
        script.push_str(&format!("cp \"$1\" '{}'\n", seen.display()));
        script.push_str(&format!("echo \"$5\" > '{}'\n", status_path.display()));
        script.push_str(&format!("echo \"$2 $3 $4 $6 $7 $8\" > '{}'\n", args.display()));
        for step in steps {
            match step {
                Step::Say(line) => script.push_str(&format!("echo '{line}' >> \"$5\"\n")),
                Step::WaitFor(file) => script.push_str(&format!(
                    "while [ ! -e '{}' ]; do sleep 0.01; done\n",
                    file.display()
                )),
                Step::Touch(file) => {
                    script.push_str(&format!("echo here > '{}'\n", file.display()))
                }
                Step::Exit(code) => script.push_str(&format!("exit {code}\n")),
            }
        }
    }
    let name = if cfg!(windows) { "sky-look-stand-in.cmd" } else { "sky-look-stand-in" };
    let program = dir.join(name);
    std::fs::write(&program, script).expect("the stand-in");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&program, std::fs::Permissions::from_mode(0o755))
            .expect("the stand-in is executable");
    }
    program
}

/// The temporary files a press has left in `dir` with the extension `extension`, including one
/// still being written.
fn press_files(dir: &Path, extension: &str) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .expect("the scratch directory")
        .map(|entry| entry.expect("an entry").path())
        .filter(|path| {
            path.file_name()
                .is_some_and(|name| name.to_string_lossy().starts_with("black-hole-lab-look-"))
                && path.extension().is_some_and(|e| e == extension)
        })
        .collect()
}

/// The temporary saves a press has left in `dir`.
fn saves_in(dir: &Path) -> Vec<PathBuf> {
    press_files(dir, crate::save::EXTENSION)
}

/// Whether a press has left nothing at all in `dir`: no save and no status file.
fn nothing_left_in(dir: &Path) -> bool {
    saves_in(dir).is_empty() && press_files(dir, STATUS_EXTENSION).is_empty()
}

/// Run the app's once-a-frame poll until `done` says so, the way frames would, and fail the test
/// rather than hang it if that takes longer than any stand-in could.
fn poll_until(app: &mut SpacetimeApp, what: &str, done: impl Fn(&SpacetimeApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        app.poll_look_around();
        if done(app) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "waited 30 s for {what}; the status is {:?}",
            app.controls.look_status
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The Look Around status the app is showing, which every test here expects there to be.
fn status(app: &SpacetimeApp) -> &LookStatus {
    app.controls.look_status.as_ref().expect("the card is told what happened")
}

/// Whether the view the app is making has finished, one way or the other.
fn finished(app: &SpacetimeApp) -> bool {
    app.look_around.is_none() && app.controls.look_making.is_none()
}

#[test]
fn test_the_search_for_sky_look_takes_the_variable_then_the_app_folder_then_the_nearest_sky_build() {
    // A repository with a release build of the sky tools in it, and the app running from a
    // directory three levels under the root, as `target/probe/release` is.
    let root = scratch_dir("search");
    let app_dir = root.join("repo/target/probe/release");
    std::fs::create_dir_all(&app_dir).expect("the app's directory");
    let built = root.join("repo").join(SKY_RELEASE_DIR).join(SKY_LOOK_FILE);
    std::fs::create_dir_all(built.parent().expect("a directory")).expect("the sky build");
    std::fs::write(&built, b"").expect("the built program");
    let elsewhere = root.join("elsewhere").join(SKY_LOOK_FILE);
    std::fs::create_dir_all(elsewhere.parent().expect("a directory")).expect("another directory");
    std::fs::write(&elsewhere, b"").expect("a program somewhere else");

    // Nothing beside the app, no variable: the walk up finds the sky build.
    assert_eq!(find_sky_look(None, Some(&app_dir)), Ok(built.clone()));
    // A variable set to nothing is a variable not set.
    assert_eq!(find_sky_look(Some(OsStr::new("")), Some(&app_dir)), Ok(built.clone()));

    // A program beside the app comes before the walk.
    let beside = app_dir.join(SKY_LOOK_FILE);
    std::fs::write(&beside, b"").expect("a program beside the app");
    assert_eq!(find_sky_look(None, Some(&app_dir)), Ok(beside.clone()));

    // And the variable comes before both.
    assert_eq!(find_sky_look(Some(elsewhere.as_os_str()), Some(&app_dir)), Ok(elsewhere.clone()));

    // A variable naming a file that is not there is reported rather than passed over for one of
    // the other two, which would run a program the user had not named.
    let missing = root.join("nowhere").join(SKY_LOOK_FILE);
    let why = find_sky_look(Some(missing.as_os_str()), Some(&app_dir)).expect_err("nothing there");
    assert!(why.contains(SKY_LOOK_ENV) && why.contains("no program there"), "{why}");

    // A directory that happens to carry the program's name is not the program.
    let decoy_dir = root.join("decoy/target/release");
    std::fs::create_dir_all(decoy_dir.join(SKY_LOOK_FILE)).expect("a directory named like it");
    assert!(find_sky_look(None, Some(&decoy_dir)).is_err());

    // Nothing anywhere in a checkout whose sky tools were never built: one plain sentence, which
    // says how to build the program.
    let bare = root.join("bare/target/release");
    std::fs::create_dir_all(&bare).expect("an app with no sky tools");
    std::fs::create_dir_all(root.join("bare/sky")).expect("the sky sources");
    for exe_dir in [Some(bare.as_path()), None] {
        let why = find_sky_look(None, exe_dir).expect_err("there is no program to find");
        assert!(
            why.contains("was not found")
                && why.contains("cargo build --release")
                && why.contains("sky directory")
                && why.contains(SKY_LOOK_ENV),
            "{why}"
        );
    }

    // Nothing beside a downloaded copy, which has no sky directory above it to build in: the
    // sentence says the copy is incomplete, and does not send its reader after a compiler.
    let copy = root.join("downloaded/Black Hole Lab");
    std::fs::create_dir_all(&copy).expect("a downloaded copy");
    let why = find_sky_look(None, Some(&copy)).expect_err("there is no program to find");
    assert!(
        why.contains("was not found beside Black Hole Lab")
            && why.contains("incomplete")
            && why.contains(SKY_LOOK_ENV)
            && !why.contains("cargo"),
        "{why}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_a_press_hands_sky_look_a_save_of_this_moment_and_changes_nothing_in_the_app() {
    // What the press writes, read back by the app's own loader, has to be the run standing at the
    // press; and the press has to leave behind nothing but its own status. The file status line of
    // Save and Load, the run and the play state are the three things a careless press would move.
    let mut app = SpacetimeApp::default();
    play_sim(&mut app, 2.0);
    // Playing, which is the harder case: a press that paused the run, or stepped it, would show.
    // Nothing here runs a frame, so a playing run is not stepped by the test either.
    app.controls.is_playing = true;
    let earlier = FileStatus { text: "Saved an earlier file.".to_string(), failed: false };
    app.controls.file_status = Some(earlier.clone());
    let standing = app.sim.fingerprint();
    let document = app.snapshot("");

    let dir = scratch_dir("press");
    let photo = dir.join("view.jpg");
    let done = format!("done {}", photo.display());
    let program =
        stand_in(&dir, &[Step::Say("Tracing the light."), Step::Say(&done), Step::Exit(0)]);
    app.look_around(Who::Bob, Ok(program), &dir);

    let unmoved = |app: &SpacetimeApp, when: &str| {
        assert_eq!(app.sim.fingerprint(), standing, "{when}: the run moved");
        assert!(app.controls.is_playing, "{when}: the play state changed");
        let file_status =
            app.controls.file_status.as_ref().expect("the file status is still there");
        assert_eq!(
            (&file_status.text, file_status.failed),
            (&earlier.text, earlier.failed),
            "{when}: the file status line changed"
        );
    };
    unmoved(&app, "straight after the press");
    assert_eq!(app.controls.look_making, Some(Who::Bob), "and a view is being made");
    poll_until(&mut app, "the stand-in to finish", finished);
    unmoved(&app, "once the view was made");

    // What the child was handed: the save, with the observer, the status file beside the save,
    // the leave to move the save, and `--open` after it; and no terminal, which is off unless the
    // box is ticked.
    let args = std::fs::read_to_string(dir.join("args.txt")).expect("the stand-in's arguments");
    assert_eq!(args.trim(), "--observer bob --status --move-save --open");
    let status = std::fs::read_to_string(dir.join("status-path.txt")).expect("the status path");
    let status = PathBuf::from(status.trim());
    assert_eq!(status.parent(), Some(dir.as_path()), "beside the save");
    assert_eq!(status.extension().and_then(|e| e.to_str()), Some(STATUS_EXTENSION));
    let mut restored = SpacetimeApp::default();
    restored
        .load_from(&dir.join("seen.bhl"))
        .expect("the app's own loader reads what sky-look was given");
    assert_eq!(restored.sim.fingerprint(), standing, "the save is the run at the press");
    let reread = restored.snapshot("");
    assert_eq!(reread.sim, document.sim, "the run came back changed");
    assert_eq!(reread.controls, document.controls, "the panel came back changed");
    assert_eq!(reread.view, document.view, "the views came back changed");

    assert!(nothing_left_in(&dir), "the temporary files are gone");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_the_status_follows_sky_look_and_a_second_press_waits_for_the_first() {
    // The stand-in stops after each of its first two lines until the test lets it go on, so the
    // test can see each line on the card while the child is still running - which is the status
    // following the child rather than reporting on it afterwards.
    let dir = scratch_dir("progress");
    let (go_on, go_to_end) = (dir.join("go-on"), dir.join("go-to-end"));
    let photo = dir.join("view.jpg");
    let program = stand_in(
        &dir,
        &[
            Step::Say("Tracing the light."),
            Step::WaitFor(&go_on),
            Step::Say("Rendering the view."),
            Step::WaitFor(&go_to_end),
            Step::Say("Writing the photograph."),
            Step::Say(&format!("done {}", photo.display())),
            Step::Exit(0),
        ],
    );
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Alice, Ok(program.clone()), &dir);
    assert_eq!(status(&app).text, "Starting sky-look for Alice's view.");

    poll_until(&mut app, "the first line", |app| status(app).text == "Tracing the light.");
    assert!(app.look_around.is_some(), "the child is still running");
    assert_eq!(app.controls.look_making, Some(Who::Alice), "the panel is told whose view it is");
    assert_eq!((status(&app).who, status(&app).failed), (Who::Alice, false));

    // While Alice's view is being made, neither card will start another. The buttons are greyed
    // out, and a press that got past them anyway changes nothing: no second child, no second
    // save, and the status still about Alice.
    for (name, settings, obs) in [
        ("Alice", &app.controls.alice, app.sim.alice.as_ref()),
        ("Bob", &app.controls.bob, app.sim.bob.as_ref()),
    ] {
        let why = look_around_blocked(name, settings, obs, app.controls.look_making)
            .unwrap_or_else(|| panic!("{name}'s button is greyed out while a view is being made"));
        assert!(why.contains("making Alice's view") && why.contains("one view at a time"), "{why}");
    }
    app.look_around(Who::Bob, Ok(program), &dir);
    assert_eq!(saves_in(&dir).len(), 1, "one save, for the one view being made");
    assert_eq!(status(&app).who, Who::Alice, "the status is still the first view's");

    std::fs::write(&go_on, b"").expect("the first handshake");
    poll_until(&mut app, "the second line", |app| status(app).text == "Rendering the view.");
    assert_eq!(app.controls.look_making, Some(Who::Alice));

    std::fs::write(&go_to_end, b"").expect("the second handshake");
    poll_until(&mut app, "the view to be made", finished);
    let done = status(&app);
    assert!(!done.failed, "a view that was made is not a failure: {:?}", done.text);
    assert_eq!(done.text, format!("Made Alice's view and opened it: {}", photo.display()));
    assert!(nothing_left_in(&dir), "the temporary files are gone");
    let (card, alice) = (&app.controls.alice, app.sim.alice.as_ref());
    assert!(
        look_around_blocked("Alice", card, alice, app.controls.look_making).is_none(),
        "and the buttons are back"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_a_refusal_and_a_failure_each_end_in_the_programs_own_sentence() {
    // The verdict is the status file's last line, and both a refusal and a failure put the
    // child's sentence on the card, marked as a failure. A child that exits without a verdict
    // still leaves a sentence, and so does a program that could not be found at all.
    let refusal = "Bob is inside the inner horizon, where sky-look cannot follow him.";
    let failure = "The star map could not be read.";
    let refused = format!("refused {refusal}");
    let failed = format!("failed {failure}");
    let cases: [(&str, Vec<Step>, String); 3] = [
        (
            "a refusal",
            vec![Step::Say("Reading the save."), Step::Say(&refused), Step::Exit(2)],
            format!("No view from Bob's place: {refusal}"),
        ),
        (
            "a failure",
            vec![Step::Say("Tracing the light."), Step::Say(&failed), Step::Exit(1)],
            format!("Could not make Bob's view: {failure}"),
        ),
        (
            "an exit without a verdict",
            vec![Step::Say("Tracing the light."), Step::Exit(1)],
            "Could not make Bob's view: sky-look stopped with exit code 1 without saying why."
                .to_string(),
        ),
    ];
    for (what, steps, expected) in cases {
        let dir = scratch_dir("refusal");
        let program = stand_in(&dir, &steps);
        let mut app = SpacetimeApp::default();
        app.look_around(Who::Bob, Ok(program), &dir);
        poll_until(&mut app, what, finished);
        let ended = status(&app);
        assert!(ended.failed, "{what} is marked as a failure: {:?}", ended.text);
        assert_eq!(ended.text, expected, "{what}");
        assert_eq!(ended.who, Who::Bob, "{what} is on Bob's card");
        assert!(nothing_left_in(&dir), "{what}: the temporary files are gone");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // No program to run: nothing is saved, nothing is started, and the card says how to get it.
    let dir = scratch_dir("missing");
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Bob, find_sky_look(None, None), &dir);
    let ended = status(&app);
    assert!(ended.failed && ended.text.contains("cargo build --release"), "{:?}", ended.text);
    assert!(app.look_around.is_none() && app.controls.look_making.is_none());
    assert!(nothing_left_in(&dir), "nothing was written");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_a_verdict_ends_the_view_at_once_and_what_sky_look_does_after_it_is_left_alone() {
    // After its verdict, `sky-look` may still be starting the command prompt the user asked to be
    // left running. The card reports the view as soon as the verdict is read, and dropping the
    // finished job stops nothing: the stand-in, still waiting after its verdict, is let go on and
    // proves it was not killed by leaving a file behind.
    let dir = scratch_dir("after");
    let (go_on, survived) = (dir.join("go-on"), dir.join("survived"));
    let photo = dir.join("view.jpg");
    let done = format!("done {}", photo.display());
    let program = stand_in(
        &dir,
        &[Step::Say(&done), Step::WaitFor(&go_on), Step::Touch(&survived), Step::Exit(0)],
    );
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Alice, Ok(program), &dir);
    poll_until(&mut app, "the verdict", finished);
    assert_eq!(status(&app).text, format!("Made Alice's view and opened it: {}", photo.display()));
    assert!(!survived.exists(), "the stand-in is still waiting");

    std::fs::write(&go_on, b"").expect("the handshake");
    let deadline = Instant::now() + Duration::from_secs(30);
    while !survived.exists() {
        assert!(Instant::now() < deadline, "sky-look was stopped after its verdict");
        std::thread::sleep(Duration::from_millis(10));
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_cancel_stops_sky_look_says_so_and_deletes_the_temporary_files() {
    let dir = scratch_dir("cancel");
    let (go_on, survived) = (dir.join("go-on"), dir.join("survived"));
    let program = stand_in(
        &dir,
        &[
            Step::Say("Tracing the light that reaches Bob: 35%."),
            Step::WaitFor(&go_on),
            Step::Touch(&survived),
            Step::Exit(0),
        ],
    );
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Bob, Ok(program), &dir);
    poll_until(&mut app, "the percentage", |app| {
        status(app).text == "Tracing the light that reaches Bob: 35%."
    });

    // The card's Cancel is a request the app takes after the panel, as Look Around is.
    app.controls.look_cancel = true;
    assert!(app.controls.take_look_cancel(), "the request is taken");
    assert!(!app.controls.take_look_cancel(), "once");
    app.cancel_look_around();
    assert!(app.look_around.is_none() && app.controls.look_making.is_none());
    let said = status(&app);
    assert_eq!(
        (said.who, said.failed, said.text.as_str()),
        (
            Who::Bob,
            false,
            "Cancelled Bob's view: Black Hole Lab stopped sky-look and every program sky-look \
             started."
        )
    );
    assert!(nothing_left_in(&dir), "the temporary files are gone");
    // And the next frame's poll leaves the card as the cancel left it.
    app.poll_look_around();
    assert!(status(&app).text.starts_with("Cancelled Bob's view"));

    std::fs::write(&go_on, b"").expect("the handshake");
    std::thread::sleep(Duration::from_millis(500));
    assert!(!survived.exists(), "sky-look ran on after the cancel");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_closing_the_app_while_a_view_is_being_made_stops_sky_look_and_deletes_the_save() {
    // The stand-in waits part-way through and, if it is ever let go on, leaves a file behind to
    // say that it outlived the app. Dropping the app is what closing the window does to it.
    let dir = scratch_dir("close");
    let (go_on, survived) = (dir.join("go-on"), dir.join("survived"));
    let program = stand_in(
        &dir,
        &[
            Step::Say("Tracing the light."),
            Step::WaitFor(&go_on),
            Step::Touch(&survived),
            Step::Exit(0),
        ],
    );
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Bob, Ok(program), &dir);
    poll_until(&mut app, "the first line", |app| status(app).text == "Tracing the light.");
    assert_eq!(saves_in(&dir).len(), 1, "the save is there while the view is being made");

    drop(app);
    assert!(nothing_left_in(&dir), "the temporary files are gone");

    // Let a stand-in that was still running go on, and give it far longer than it needs.
    std::fs::write(&go_on, b"").expect("the handshake");
    std::thread::sleep(Duration::from_millis(500));
    assert!(!survived.exists(), "sky-look ran on after the app had closed");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_the_look_around_buttons_are_greyed_out_exactly_when_the_app_can_tell_no_view_can_be_made() {
    let mut app = SpacetimeApp::default();
    let blocked = |app: &SpacetimeApp, making: Option<Who>| {
        look_around_blocked("Bob", &app.controls.bob, app.sim.bob.as_ref(), making)
    };
    assert_eq!(blocked(&app, None), None, "Bob as the app opens can be looked around from");

    // A view already being made, from either observer, and the reason names whose it is.
    for making in [Who::Alice, Who::Bob] {
        let why = blocked(&app, Some(making)).expect("a view is already being made");
        let whose = format!("making {}'s view", making.name());
        assert!(why.contains(&whose) && why.contains("one view at a time"), "{why}");
    }

    // A hand on his marker.
    app.sim.bob.as_mut().expect("Bob is in the run").mode = ObserverMode::ManualDrag;
    let why = blocked(&app, None).expect("the pointer is holding Bob");
    assert!(why.contains("marker drag holds Bob"), "{why}");
    app.sim.bob.as_mut().expect("Bob is in the run").mode = ObserverMode::FreeFall;

    // His card unticked, and the frame after it, when the panel has taken him out of the run.
    app.controls.bob.enabled = false;
    let why = blocked(&app, None).expect("Bob's card is unticked");
    assert!(why.contains("Bob is not in the simulation"), "{why}");
    app.sim.bob = None;
    assert!(blocked(&app, None).is_some(), "and there is no Bob to look around from");

    // Where he stands is not the app's question. Deep inside r₋, where sky-look will refuse, the
    // button is still offered, so that the refusal is sky-look's own sentence.
    let mut deep = SpacetimeApp::default();
    let rm = deep.sim.metric.inner_horizon();
    deep.sim.bob.as_mut().expect("Bob is in the run").r = 0.5 * rm;
    assert_eq!(
        look_around_blocked("Bob", &deep.controls.bob, deep.sim.bob.as_ref(), None),
        None,
        "the app does not judge the tracer's refusals"
    );
}

#[test]
fn test_a_line_is_taken_only_once_its_line_feed_has_arrived() {
    // The status file is read while `sky-look` writes it, so a read can land in the middle of a
    // line. The half is kept back, never shown as a status, and the whole line comes out once the
    // rest arrives; a carriage return is trimmed, a blank line skipped, and the half a writer
    // left when it exited is given up at the end.
    let dir = scratch_dir("tail");
    let path = dir.join("x.status");
    let mut tail = StatusTail::new(path.clone());
    assert!(tail.lines().is_empty(), "no file yet, and no lines");
    let mut file = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(&path)
        .expect("the status file");
    use std::io::Write as _;
    file.write_all(b"Tracing the li").expect("half a line");
    assert!(tail.lines().is_empty(), "half a line is not a line");
    file.write_all(b"ght: 35%.\r\n\nRendering").expect("the rest, and more");
    assert_eq!(tail.lines(), ["Tracing the light: 35%."]);
    assert!(tail.lines().is_empty(), "nothing new");
    file.write_all(b" the view: 15%.\ndone C:\\a b\\x.jpg").expect("a verdict, cut short");
    assert_eq!(tail.lines(), ["Rendering the view: 15%."]);
    assert_eq!(tail.rest().as_deref(), Some("done C:\\a b\\x.jpg"));
    assert_eq!(tail.rest(), None, "given up once");
    drop(file);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_a_verdict_is_a_lowercase_word_and_a_space_and_nothing_else_is() {
    assert_eq!(Verdict::of("done C:\\v\\x.jpg"), Some(Verdict::Done("C:\\v\\x.jpg".into())));
    assert_eq!(Verdict::of("refused Bob is at r = 0.3 M."), Some(Verdict::Refused("Bob is at r = 0.3 M.".into())));
    assert_eq!(Verdict::of("failed The map could not be read."), Some(Verdict::Failed("The map could not be read.".into())));
    for progress in [
        "Tracing the light that reaches Bob: 35%.",
        "Done.",
        "16888 pixels (0.0503 % of the picture) could not be determined.",
        "done",
        "failedly",
    ] {
        assert_eq!(Verdict::of(progress), None, "{progress}");
    }
}

#[test]
fn test_the_rendering_terminal_adds_shell_and_a_console_and_every_run_is_below_normal_priority() {
    let (save, status) = (Path::new("a.bhl"), Path::new("a.status"));
    let words = |terminal| -> Vec<String> {
        arguments(save, status, Who::Alice, terminal)
            .iter()
            .map(|a| a.to_string_lossy().into_owned())
            .collect()
    };
    assert_eq!(
        words(false),
        ["a.bhl", "--observer", "alice", "--status", "a.status", "--move-save", "--open"]
    );
    assert_eq!(
        words(true),
        ["a.bhl", "--observer", "alice", "--status", "a.status", "--move-save", "--open", "--shell"]
    );
    assert_eq!(status_path_for(Path::new("d/x-1-2.bhl")), Path::new("d/x-1-2.status"));
    #[cfg(windows)]
    {
        assert_eq!(creation_flags(false), 0x0800_0000 | 0x0000_4000, "no window, below normal");
        assert_eq!(creation_flags(true), 0x0000_0010 | 0x0000_4000, "a console, below normal");
    }
    assert_eq!(TERMINAL_AVAILABLE, cfg!(windows));
}

#[test]
fn test_show_rendering_terminal_is_one_setting_on_both_cards_off_at_first_and_never_in_a_file() {
    let mut app = SpacetimeApp::default();
    assert!(!app.controls.look_terminal, "off as the app opens");
    let painted = panel_text(&mut app);
    assert_eq!(painted.matches("Show Rendering Terminal").count(), 2, "{painted}");

    // A file neither carries the box nor changes it on a Load.
    app.controls.look_terminal = true;
    let json = crate::save::to_json(&app.snapshot("")).expect("writable");
    assert!(!json.contains("look_terminal") && !json.contains("terminal"), "{json}");
    let dir = scratch_dir("terminal");
    let path = dir.join("run.bhl");
    app.save_to(&path, "").expect("saved");
    for ticked in [true, false] {
        let mut other = SpacetimeApp::default();
        other.controls.look_terminal = ticked;
        other.load_from(&path).expect("loaded");
        assert_eq!(other.controls.look_terminal, ticked, "a Load keeps the box as it was");
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// The text the controls panel paints, for a check of what the cards show.
fn panel_text(app: &mut SpacetimeApp) -> String {
    fn collect(shape: &egui::Shape, out: &mut String) {
        match shape {
            egui::Shape::Text(text) => {
                out.push_str(text.galley.text());
                out.push('\n');
            }
            egui::Shape::Vec(shapes) => shapes.iter().for_each(|shape| collect(shape, out)),
            _ => {}
        }
    }
    let ctx = egui::Context::default();
    ctx.set_fonts(egui::FontDefinitions::empty());
    let output = ctx.run_ui(Default::default(), |ui| {
        app.controls.render_panel(ui, &mut app.sim);
    });
    let mut text = String::new();
    for clipped in output.shapes.iter() {
        collect(&clipped.shape, &mut text);
    }
    output.drop_without_applying_deltas();
    text
}

#[test]
fn test_the_card_whose_view_is_being_made_offers_cancel_where_look_around_was() {
    let dir = scratch_dir("cancel-button");
    let go_on = dir.join("go-on");
    let program = stand_in(&dir, &[Step::WaitFor(&go_on), Step::Exit(0)]);
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Alice, Ok(program), &dir);
    let painted = panel_text(&mut app);
    assert_eq!(painted.matches("Cancel").count(), 1, "{painted}");
    assert_eq!(painted.matches("Look Around").count(), 1, "Bob's, greyed out: {painted}");
    let alice_title = painted.find("OBSERVER ALICE").expect("Alice's card");
    let cancel = painted.find("Cancel").expect("the button");
    assert!(cancel > alice_title, "on Alice's card: {painted}");
    drop(app);
    std::fs::write(&go_on, b"").expect("the handshake");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_the_check_output_becomes_what_is_missing_and_what_a_download_would_supply() {
    // Everything there: nothing to say.
    assert!(Readiness::of("tools ok\nmap ok\n").complete());

    // A star map a download would supply gives the offer of the download in place of its
    // sentence.
    let readiness = Readiness::of("tools ok\r\nmap fetchable The star map is not there.\r\n");
    assert_eq!(readiness, Readiness { tools: None, map: None, map_fetchable: true });
    assert!(!readiness.complete());

    // A star map only its owner can supply is a sentence, whole, and so is an incomplete copy.
    let readiness =
        Readiness::of("tools missing No tracer, here.\nmap missing The variable is wrong.\n");
    assert_eq!(readiness.tools.as_deref(), Some("No tracer, here."));
    assert_eq!(readiness.map.as_deref(), Some("The variable is wrong."));
    assert!(!readiness.map_fetchable);

    // A sky-look older than --check prints nothing on standard output, and lines that are not the
    // report's - an older sky-look's line for ffmpeg among them - are passed over: nothing is
    // known to be missing.
    for output in [
        "",
        "\n",
        "There is no option --check.\n",
        "tools\n",
        "map missing\n",
        "ffmpeg missing ffmpeg was not found.\nffmpeg-install sudo apt install ffmpeg\n",
    ] {
        assert!(Readiness::of(output).complete(), "{output:?}");
    }
}

/// What `sky-look --check` is to say, for `setup_stand_in`'s `check.txt`.
fn check_lines(map: bool) -> String {
    let map = if map { "map ok" } else { "map fetchable The star map is not there." };
    format!("tools ok\n{map}\n")
}

/// Write a stand-in for `sky-look` into `dir` that answers the three command lines the dialog's
/// flow gives it:
///
/// - `--check` prints the file `check.txt` of `dir`, which the test rewrites as the map arrives;
/// - `--fetch-map --status <file>` appends a line of progress to the status file, waits for the
///   file `go` of `dir` to appear, appends `done the-map.exr`, and exits;
/// - anything else, which is a view being asked for, exits at once and says nothing.
fn setup_stand_in(dir: &Path) -> PathBuf {
    let (check, go) = (dir.join("check.txt"), dir.join("go"));
    let script = if cfg!(windows) {
        format!(
            "@echo off\r\nif \"%~1\"==\"--check\" goto check\r\n\
             if \"%~1\"==\"--fetch-map\" goto fetch\r\nexit /b 0\r\n\
             :check\r\ntype \"{}\"\r\nexit /b 0\r\n\
             :fetch\r\n>>\"%~3\" echo Downloading the star map from NASA: 40%%.\r\n\
             :wait\r\nif not exist \"{}\" goto wait\r\n>>\"%~3\" echo done the-map.exr\r\n",
            check.display(),
            go.display()
        )
    } else {
        format!(
            "#!/bin/sh\nif [ \"$1\" = \"--check\" ]; then cat '{}'; exit 0; fi\n\
             if [ \"$1\" = \"--fetch-map\" ]; then\n\
             echo 'Downloading the star map from NASA: 40%.' >> \"$3\"\n\
             while [ ! -e '{}' ]; do sleep 0.01; done\n\
             echo 'done the-map.exr' >> \"$3\"\nfi\nexit 0\n",
            check.display(),
            go.display()
        )
    };
    let name = if cfg!(windows) { "setup-stand-in.cmd" } else { "setup-stand-in" };
    let program = dir.join(name);
    std::fs::write(&program, script).expect("the stand-in");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&program, std::fs::Permissions::from_mode(0o755))
            .expect("the stand-in is executable");
    }
    program
}

/// Run the app's once-a-frame poll of the dialog's download until it has ended, the way frames
/// would, and fail the test rather than hang it.
fn poll_until_downloaded(app: &mut SpacetimeApp, program: &Path, dir: &Path) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while app.poll_look_dialog(|| Ok(program.to_path_buf()), dir) {
        assert!(Instant::now() < deadline, "waited 30 s for the download to end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn test_a_missing_star_map_puts_up_the_dialog_and_ok_downloads_it_then_makes_the_view() {
    let dialog = |app: &SpacetimeApp| app.look_dialog.clone().expect("the dialog is up");

    // The star map is missing. The press starts no view and downloads nothing: it asks.
    let dir = scratch_dir("dialog");
    let program = setup_stand_in(&dir);
    let (check, go) = (dir.join("check.txt"), dir.join("go"));
    std::fs::write(&check, check_lines(false)).expect("what the check says");
    let mut app = SpacetimeApp::default();
    app.press_look_around(Who::Alice, Ok(program.clone()), &dir);
    let asked = dialog(&app);
    assert_eq!(asked.who, Who::Alice);
    assert_eq!(asked.question(), "Download missing dependencies?");
    assert!(app.look_around.is_none() && app.map_fetch.is_none());
    assert!(app.controls.look_status.is_none(), "nothing is said on the card");

    // Cancel: nothing was downloaded, no view is made, and the app is as it was.
    app.look_dialog_cancel();
    assert!(app.look_dialog.is_none() && app.look_around.is_none() && app.map_fetch.is_none());
    assert!(nothing_left_in(&dir), "nothing was written");

    // OK: the download starts, and the dialog stays up while it runs, saying what comes next.
    app.press_look_around(Who::Alice, Ok(program.clone()), &dir);
    app.look_dialog_ok(Ok(program.clone()), &dir);
    let waiting = dialog(&app);
    assert!(waiting.fetching && app.map_fetch.is_some(), "the map is being downloaded");
    assert_eq!(
        waiting.question(),
        "Black Hole Lab makes Alice's view when the download ends."
    );
    assert!(app.look_around.is_none());
    // Another OK while the download runs starts no second one.
    app.look_dialog_ok(Ok(program.clone()), &dir);
    assert!(dialog(&app).fetching);

    // The download ends: the dialog goes, and the view that was asked for is made by itself.
    std::fs::write(&check, check_lines(true)).expect("what the check says");
    std::fs::write(&go, b"").expect("the signal to finish");
    poll_until_downloaded(&mut app, &program, &dir);
    assert!(app.look_dialog.is_none(), "the dialog is gone");
    assert_eq!(status(&app).who, Who::Alice);
    assert!(!status(&app).failed, "{:?}", status(&app).text);
    poll_until(&mut app, "the stand-in view to end", finished);
    let _ = std::fs::remove_dir_all(&dir);

    // Cancel while the star map is being downloaded stops the download and leaves no file.
    let dir = scratch_dir("dialog-cancel");
    let program = setup_stand_in(&dir);
    std::fs::write(dir.join("check.txt"), check_lines(false)).expect("the check");
    let mut app = SpacetimeApp::default();
    app.press_look_around(Who::Bob, Ok(program.clone()), &dir);
    app.look_dialog_ok(Ok(program.clone()), &dir);
    assert!(app.map_fetch.is_some());
    app.look_dialog_cancel();
    assert!(app.look_dialog.is_none() && app.map_fetch.is_none() && app.look_around.is_none());
    assert!(!app.poll_look_dialog(|| Ok(program.clone()), &dir));
    assert!(nothing_left_in(&dir), "the status file is gone");
    let _ = std::fs::remove_dir_all(&dir);

    // Nothing missing: the press makes the view, and no dialog is seen.
    let dir = scratch_dir("dialog-none");
    let program = setup_stand_in(&dir);
    std::fs::write(dir.join("check.txt"), check_lines(true)).expect("the check");
    let mut app = SpacetimeApp::default();
    app.press_look_around(Who::Bob, Ok(program), &dir);
    assert!(app.look_dialog.is_none());
    assert_eq!(status(&app).who, Who::Bob);
    poll_until(&mut app, "the stand-in view to end", finished);
    let _ = std::fs::remove_dir_all(&dir);

    // No sky-look at all: the dialog, with the sentence saying so, and OK changes nothing.
    let mut app = SpacetimeApp::default();
    let nowhere = || Err("sky-look was not found.".to_string());
    app.press_look_around(Who::Bob, nowhere(), &std::env::temp_dir());
    assert_eq!(dialog(&app).needs.tools.as_deref(), Some("sky-look was not found."));
    assert_eq!(dialog(&app).question(), "Press OK to check again.");
    app.look_dialog_ok(nowhere(), &std::env::temp_dir());
    assert!(app.look_dialog.is_some() && app.map_fetch.is_none() && app.look_around.is_none());
}
