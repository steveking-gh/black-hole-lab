//! What Look Around has to be true of, tested without the sky tools.
//!
//! None of these tests runs the real `sky-look`, and none of them depends on its having been built.
//! The search is tested as the pure function it is, against directory layouts made for the purpose.
//! Everything that runs a child runs a stand-in: a script written by the test into a scratch
//! directory of its own - a batch file on Windows, a shell script elsewhere - which does what the
//! test tells it to do, in order: print a line, complain on standard error, wait for a file to
//! appear, exit with a chosen code. A script rather than a test binary because a test binary is
//! driven by libtest's own command line, which the contract's `<file.bhl> --observer bob --open`
//! would be refused by, and because `cmd.exe` and `sh` are on every machine these tests run on. The
//! waits are handshakes on files rather than sleeps, so that a test says "now" instead of guessing
//! how long a line takes to arrive.
//!
//! Every stand-in also copies the save it was handed and the arguments after it, before anything
//! else, so that a test can check what the child actually received.

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
    /// Print a line on standard output.
    Say(&'a str),
    /// Print a line on standard error.
    Complain(&'a str),
    /// Wait until this file exists.
    WaitFor(&'a Path),
    /// Write this file, as proof of having got this far.
    Touch(&'a Path),
    /// Exit with this code.
    Exit(i32),
}

/// Write a stand-in for `sky-look` into `dir` that copies the save it is handed to `seen.bhl`, and
/// the rest of its arguments to `args.txt`, and then does `steps`.
///
/// The lines the tests print hold letters, spaces, full stops, colons and backslashes and nothing
/// either shell would read as syntax, so they are written out unquoted by `cmd` and in single
/// quotes by `sh`.
fn stand_in(dir: &Path, steps: &[Step]) -> PathBuf {
    let seen = dir.join("seen.bhl");
    let args = dir.join("args.txt");
    let mut script = String::new();
    if cfg!(windows) {
        script.push_str("@echo off\r\n");
        script.push_str(&format!("copy /y \"%~1\" \"{}\" >nul\r\n", seen.display()));
        script.push_str(&format!(">\"{}\" echo %2 %3 %4\r\n", args.display()));
        for (index, step) in steps.iter().enumerate() {
            match step {
                Step::Say(line) => script.push_str(&format!("echo {line}\r\n")),
                Step::Complain(line) => script.push_str(&format!(">&2 echo {line}\r\n")),
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
        script.push_str(&format!("echo \"$2 $3 $4\" > '{}'\n", args.display()));
        for step in steps {
            match step {
                Step::Say(line) => script.push_str(&format!("echo '{line}'\n")),
                Step::Complain(line) => script.push_str(&format!("echo '{line}' >&2\n")),
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

/// The temporary saves a press has left in `dir`, including one still being written.
fn saves_in(dir: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .expect("the scratch directory")
        .map(|entry| entry.expect("an entry").path())
        .filter(|path| {
            path.file_name()
                .is_some_and(|name| name.to_string_lossy().starts_with("black-hole-lab-look-"))
        })
        .collect()
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

    // Nothing anywhere: one plain sentence, which says how to get the program.
    let bare = root.join("bare/target/release");
    std::fs::create_dir_all(&bare).expect("an app with no sky tools");
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
    let video = dir.join("view.mp4");
    let program = stand_in(
        &dir,
        &[Step::Say("Tracing the light."), Step::Say(&video.display().to_string()), Step::Exit(0)],
    );
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

    // What the child was handed: the save, with the observer and `--open` after it.
    let args = std::fs::read_to_string(dir.join("args.txt")).expect("the stand-in's arguments");
    assert_eq!(args.trim(), "--observer bob --open");
    let mut restored = SpacetimeApp::default();
    restored
        .load_from(&dir.join("seen.bhl"))
        .expect("the app's own loader reads what sky-look was given");
    assert_eq!(restored.sim.fingerprint(), standing, "the save is the run at the press");
    let reread = restored.snapshot("");
    assert_eq!(reread.sim, document.sim, "the run came back changed");
    assert_eq!(reread.controls, document.controls, "the panel came back changed");
    assert_eq!(reread.view, document.view, "the views came back changed");

    assert!(saves_in(&dir).is_empty(), "the temporary save is gone: {:?}", saves_in(&dir));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_the_status_follows_sky_look_and_a_second_press_waits_for_the_first() {
    // The stand-in stops after each of its first two lines until the test lets it go on, so the
    // test can see each line on the card while the child is still running - which is the status
    // following the child rather than reporting on it afterwards.
    let dir = scratch_dir("progress");
    let (go_on, go_to_end) = (dir.join("go-on"), dir.join("go-to-end"));
    let video = dir.join("view.mp4");
    let program = stand_in(
        &dir,
        &[
            Step::Say("Tracing the light."),
            Step::WaitFor(&go_on),
            Step::Say("Rendering the view."),
            Step::WaitFor(&go_to_end),
            Step::Say("Writing the video."),
            Step::Say(&video.display().to_string()),
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
    assert_eq!(done.text, format!("Made Alice's view and opened it: {}", video.display()));
    assert!(saves_in(&dir).is_empty(), "the temporary save is gone: {:?}", saves_in(&dir));
    let (card, alice) = (&app.controls.alice, app.sim.alice.as_ref());
    assert!(
        look_around_blocked("Alice", card, alice, app.controls.look_making).is_none(),
        "and the buttons are back"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_a_refusal_and_a_failure_each_end_in_the_programs_own_sentence() {
    // Code 2 is a refusal and code 1 a failure, and both put the child's sentence on the card,
    // marked as a failure. A child that fails and says nothing still leaves a sentence, and so does
    // a program that could not be found at all.
    let refusal = "Bob is inside the inner horizon, where sky-look cannot follow him.";
    let failure = "The star map could not be read.";
    let cases: [(&str, Vec<Step>, String); 3] = [
        (
            "a refusal",
            vec![Step::Say("Reading the save."), Step::Complain(refusal), Step::Exit(2)],
            format!("No view from Bob's place: {refusal}"),
        ),
        (
            "a failure",
            vec![Step::Say("Tracing the light."), Step::Complain(failure), Step::Exit(1)],
            format!("Could not make Bob's view: {failure}"),
        ),
        (
            "a silent failure",
            vec![Step::Exit(1)],
            "Could not make Bob's view: sky-look stopped with exit code 1 and gave no reason."
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
        let left = saves_in(&dir);
        assert!(left.is_empty(), "{what}: the temporary save is gone: {left:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // No program to run: nothing is saved, nothing is started, and the card says how to get it.
    let dir = scratch_dir("missing");
    let mut app = SpacetimeApp::default();
    app.look_around(Who::Bob, find_sky_look(None, None), &dir);
    let ended = status(&app);
    assert!(ended.failed && ended.text.contains("cargo build --release"), "{:?}", ended.text);
    assert!(app.look_around.is_none() && app.controls.look_making.is_none());
    assert!(saves_in(&dir).is_empty(), "nothing was written: {:?}", saves_in(&dir));
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
    assert!(saves_in(&dir).is_empty(), "the temporary save is gone: {:?}", saves_in(&dir));

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
