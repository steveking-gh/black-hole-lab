//! What a save has to be true of, in nine tests.
//!
//! The first is the one that matters and the one the others lean on: a run saved half way through,
//! loaded into a fresh app and played on, must end on the same fingerprint as the run that was
//! never interrupted. That is a statement about the arithmetic - every f64 came back as itself -
//! *and* a statement about completeness, because anything the file forgot to carry shows up as a
//! divergence a few hundred frames later without anybody having had to think of it.
//!
//! The rest cover what a fingerprint cannot see (the panel, the views, the trails and the counters),
//! the spelling of the numbers JSON has no spelling for, the refusals, one file committed to the
//! repository that this build has to go on being able to open, the two fields the format has added
//! since it shipped, and the two methods the Save and Load buttons call once a path has been
//! chosen - which is as far into those buttons as a test can reach, because nothing headless can
//! answer a native dialog.

use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

use super::*;
use crate::app::SpacetimeApp;
use crate::gui::controls::{
    ChartComets, FileRequest, FileStatus, ReferenceFrame, StepGrain, StepMode,
};
use crate::gui::spacetime_canvas::{BoxId, Canvas, Placement};
use crate::perf::replay::{FRAME_DT, app_for_fixtures, play_sim};
use crate::physics::observer::{ObserverMode, Release, Who};

/// One replay frame of the app, the way `crate::perf` takes one: the pulse cap pushed in first, and
/// then the step. Written out here rather than borrowed so that a test states the run it makes.
fn frame(app: &mut SpacetimeApp) {
    app.sim.set_max_pulses(app.controls.max_pulses);
    app.step_forward(FRAME_DT);
}

/// The app as it opens, with nothing playing.
fn default_app() -> SpacetimeApp {
    let mut app = SpacetimeApp::default();
    app.controls.is_playing = false;
    app
}

/// Bob released at r = 9 M with E = 1, L = 2.2 at a = 0.9: the worldline that freezes onto the far
/// branch of r₋ at about t = 85 M, with Alice in the run as a silent receiver. The
/// `far-branch-freeze` scenario of `crate::perf::replay`, stated here the way a card states one, so
/// that this test goes on testing the same physics whatever that module does to its scenario list.
fn far_branch_freeze_app() -> SpacetimeApp {
    let mut app = default_app();
    app.controls.alice.enabled = true;
    app.controls.alice.transmit = false;
    let bob = &mut app.controls.bob;
    bob.enabled = true;
    bob.transmit = true;
    bob.mode = ObserverMode::FreeFall;
    bob.release = Release::FromInfinity;
    bob.l_ang = 2.2;
    bob.drop_r = 9.0;
    bob.drop_phi = 0.0;
    bob.delta_t_delay = 0.0;
    app.controls.drop_observers(&mut app.sim);
    app.controls.view_reset_requested = false;
    app
}

/// A path in the system's temporary directory that no other test in this process is using.
fn scratch_file() -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    std::env::temp_dir().join(format!(
        "black-hole-lab-test-{}-{}.{EXTENSION}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ))
}

/// Save this app to a file and load the file into a fresh one, along the path a user's Save and
/// Load buttons will take: the atomic write, the gzip, the JSON, the invariants and the state
/// hash.
fn round_trip(app: &SpacetimeApp) -> SpacetimeApp {
    let path = scratch_file();
    app.save_to(&path, "").expect("a run of the app is writable");
    let mut restored = SpacetimeApp::default();
    restored.load_from(&path).expect("and what this build wrote, this build reads");
    let _ = std::fs::remove_file(&path);
    restored
}

/// The same run again in a fresh app: the control the continuation test is measured against.
///
/// `SpacetimeApp` is not `Clone` - it holds an `Instant` and three canvases full of caches - and is
/// not about to become so for a test. What a replay needs is the run and the panel, and those are
/// what this carries over.
fn twin_of(app: &SpacetimeApp) -> SpacetimeApp {
    let mut twin = SpacetimeApp::default();
    twin.sim = app.sim.clone();
    twin.controls = app.controls.clone();
    twin
}

#[test]
fn test_a_loaded_run_plays_on_bit_identically() {
    // The test this module exists for. Two states, each played to a frame K, saved, loaded into a
    // fresh app and played on; the same two states played straight through to K + N with no
    // interruption. The fingerprints have to agree to the bit.
    //
    // It catches two different things at once, which is why it earns its place over a pile of
    // field-by-field comparisons. A number that did not round-trip moves the fingerprint on the
    // first frame after the load. A piece of state the file forgot - an emission cadence, a serial
    // number, a death that was not recorded, a front mark that was not re-established - moves it
    // later, at the first step that reads the missing thing, and no test author had to have thought
    // of it in advance.
    //
    // 400 frames is 6.7 M at the replay step: several emissions at the default 0.1 M of the
    // emitter's proper time, and the arrivals those make.
    const AFTER: usize = 400;

    for (what, mut app, until) in [
        // (a) The app exactly as it opens, played to 12 M: Bob is through r₊ with both
        // transmissions in flight and arrivals on the record.
        ("the default layout at 12 M", default_app(), 12.0),
        // (b) The far-branch freeze at 90 M, past the stall at about 85: the frozen family is
        // standing on r₋, u^t is of order 1e10, and the worldline has stopped advancing while the
        // clock runs on. The nastiest state the app can reach.
        ("the far-branch freeze at 90 M", far_branch_freeze_app(), 90.0),
    ] {
        play_sim(&mut app, until);
        assert!(
            app.sim.alice_signal.pulses.len() + app.sim.bob_signal.pulses.len() > 4,
            "{what}: there has to be light in flight for this to be testing anything"
        );

        let mut uninterrupted = twin_of(&app);
        let mut restored = round_trip(&app);
        assert_eq!(
            restored.sim.fingerprint(),
            app.sim.fingerprint(),
            "{what}: the state came back changed before a single further step"
        );
        assert!(!restored.controls.is_playing, "{what}: a load comes up paused");

        let arrivals = |a: &SpacetimeApp| {
            a.sim.alice_signal.received_count() + a.sim.bob_signal.received_count()
        };
        let before = arrivals(&uninterrupted);
        for _ in 0..AFTER {
            frame(&mut uninterrupted);
            frame(&mut restored);
        }
        assert_eq!(
            format!("{:016x}", restored.sim.fingerprint()),
            format!("{:016x}", uninterrupted.sim.fingerprint()),
            "{what}: {AFTER} frames after the load the two runs have parted company"
        );
        // And the stretch that was replayed is a stretch with something in it: a continuation over
        // which nothing arrived and nothing was emitted would agree for reasons that say nothing
        // about the front marks the load had to rebuild.
        assert!(
            arrivals(&uninterrupted) > before,
            "{what}: the replayed stretch crossed no arrival, so it proves less than it should"
        );
        assert_eq!(
            arrivals(&restored),
            arrivals(&uninterrupted),
            "{what}: the two runs recorded different numbers of arrivals"
        );
    }
}

#[test]
fn test_a_save_round_trips_through_its_own_bytes() {
    // The fingerprint covers the physics and nothing else: not a trail, not an extent track, not a
    // ring history, not a counter, and nothing at all about the panel or the views. This is the
    // test for the rest of it. Comparing the *schema* values rather than the live ones is what
    // makes it complete by construction - every field of the file is in the comparison, because the
    // comparison is of the file.
    let mut app = app_for_fixtures();
    play_sim(&mut app, 8.0);

    // A panel and a set of views moved well away from their defaults, so that no field passes this
    // test by carrying the same default through twice.
    app.controls.font_scale = 1.35;
    app.controls.use_physical_units = false;
    app.controls.decimal_is_comma = true;
    app.controls.show_distant_clock_grid = false;
    app.controls.step_mode = StepMode::Watch;
    app.controls.play_speed = 3.25;
    app.controls.step_grain = StepGrain::Hundredth;
    app.controls.frame_of_ref = ReferenceFrame::Alice;
    app.controls.alice.mode = ObserverMode::Zamo;
    app.controls.bob.release = Release::AtRest;
    app.spacetime_canvas.max_r = 3.75;
    app.spacetime_canvas.time_offset = -2.5;
    app.spacetime_canvas.keep_surface_framed = false;
    app.spacetime_canvas.telemetry.placements.insert(
        (Canvas::Spacetime, BoxId::Observer(Who::Alice)),
        Placement::Offset(egui::vec2(12.5, -8.0)),
    );
    app.spatial_canvas.zoom = 91.5;
    app.spatial_canvas.pan_offset = egui::vec2(-14.0, 6.5);
    app.spatial_canvas.centred_on = Some(Who::Bob);
    app.spatial_canvas.telemetry.placements.insert(
        (Canvas::Spatial, BoxId::CauchyHorizon),
        Placement::Pinned(egui::vec2(40.0, 40.0)),
    );
    // One box shut where another was moved, and one shut on a canvas where nothing was moved at
    // all: shutting a box is not moving it, and the two lists are written and read apart.
    app.spacetime_canvas.telemetry.collapsed.insert((Canvas::Spacetime, BoxId::Observer(Who::Alice)));
    app.spacetime_canvas.telemetry.collapsed.insert((Canvas::RestFrame, BoxId::Signal(Who::Bob)));
    app.volume_canvas.telemetry.collapsed.insert((Canvas::Volume, BoxId::Observer(Who::Bob)));
    app.volume_canvas.camera.yaw = 0.42;
    app.volume_canvas.camera.t_scale = 2.5;
    app.volume_canvas.show_ghost_cones = true;
    app.volume_canvas.show_past_cone = false;
    app.volume_canvas.time_offset = 1.5;

    let note = "a note that has to survive";
    let before = app.snapshot(note);
    let bytes = to_bytes(&before).expect("writable");
    assert!(bytes.starts_with(&[0x1f, 0x8b]), "a .bhl file is gzip");

    // Written twice to the same path, because saving over an existing save is what a user does
    // most and is the case the atomic write has to get right: the rename replaces the old file, and
    // nothing removes it beforehand.
    let path = scratch_file();
    app.save_to(&path, "an earlier save in the same place").expect("writable to a file");
    app.save_to(&path, note).expect("and writable over itself");
    let mut restored = SpacetimeApp::default();
    restored.load_from(&path).expect("and readable back");
    let _ = std::fs::remove_file(&path);
    let after = restored.snapshot(note);

    // The envelope is not compared as a whole: `saved_at_utc` is the moment of the call and moves
    // between two snapshots by design. Everything under it is compared exactly.
    assert_eq!(after.format, before.format);
    assert_eq!(after.version, before.version);
    assert_eq!(after.state_hash, before.state_hash);
    assert_eq!(after.note, before.note, "the note is part of the file");
    assert_eq!(after.sim, before.sim, "the run came back changed");
    assert_eq!(after.controls, before.controls, "the panel came back changed");
    assert_eq!(after.view, before.view, "the views came back changed");
    assert!(!restored.controls.is_playing, "a load comes up paused");

    // The role rays and their columns of the track, on the live types, because they are the two
    // fields the format added for them and the schema comparison above would pass just as well if
    // both halves of the conversion dropped them. Alice on the ISCO sends pulses with both roles,
    // so there is something to lose.
    let bits = |r: f64| if r.is_nan() { u64::MAX } else { r.to_bits() };
    let roles_of = |app: &SpacetimeApp| {
        app.sim
            .alice_signal
            .pulses
            .iter()
            .chain(app.sim.bob_signal.pulses.iter())
            .map(|pulse| {
                let columns: Vec<_> =
                    pulse.extent_track.iter().map(|p| p.roles.map(bits)).collect();
                (pulse.role_rays, columns)
            })
            .collect::<Vec<_>>()
    };
    let (saved_roles, restored_roles) = (roles_of(&app), roles_of(&restored));
    assert!(
        saved_roles.iter().any(|(roles, columns)| roles.iter().all(Option::is_some)
            && columns.iter().flatten().any(|b| *b != u64::MAX)),
        "the run has to carry role rays with recorded radii for this to test anything"
    );
    assert_eq!(restored_roles, saved_roles, "the role rays or their columns came back changed");

    // The plain-text spelling is the same document as the compressed one, which is what lets the
    // golden file live in the repository as readable JSON.
    let text = to_json(&before).expect("writable as text");
    assert_eq!(
        read_document(text.as_bytes()).expect("plain JSON is accepted too").sim,
        before.sim
    );
}

#[test]
fn test_every_f64_this_state_can_hold_comes_back_as_itself() {
    // JSON has no NaN and no infinity, and this state has both: `interval_tau` is set to infinity
    // by one of the wavefront tests, and a blueshift measured on light met *on* r₋ divides by a
    // radius that has rounded onto r₋ exactly. So the schema spells those three as strings and
    // everything else as a number, and every finite value has to come back to the bit - which is
    // what serde_json's `float_roundtrip` feature is in Cargo.toml for.
    let nasty = [
        0.0,
        -0.0,
        1.0,
        -1.0,
        // u^t at the far-branch stall, and the scale a `Reception::ratio` reaches beside it.
        1e10,
        9.999_999_999_999_998e9,
        // One ulp above a tenth: seventeen significant digits when written out, which is exactly
        // where a parser that rounds instead of reading lands back on the tenth itself.
        f64::from_bits(0.1f64.to_bits() + 1),
        std::f64::consts::PI,
        1.0 / 3.0,
        // The smallest normal, the largest subnormal and the smallest subnormal there is.
        f64::MIN_POSITIVE,
        f64::MIN_POSITIVE - f64::from_bits(1),
        f64::from_bits(1),
        -f64::from_bits(1),
        f64::MAX,
        f64::MIN,
        1e-300,
        1e300,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for value in nasty {
        let text = serde_json::to_string(&v1::Num(value)).expect("writable");
        let back: v1::Num = serde_json::from_str(&text).expect("readable");
        assert_eq!(
            back.0.to_bits(),
            value.to_bits(),
            "{value:e} was written {text} and came back {} - a different f64",
            back.0
        );
    }
    // -0.0 is not 0.0, and the format knows it. Checked here as well as above, because the loop
    // would pass a format that lost the sign in both directions at once.
    let zero: v1::Num = serde_json::from_str("-0.0").expect("readable");
    assert!(zero.0.is_sign_negative(), "negative zero lost its sign");

    // NaN stays NaN. Its payload does not survive the spelling `"nan"`, and nothing in the app
    // reads a NaN's payload, so what the format promises about it is NaN-ness.
    let text = serde_json::to_string(&v1::Num(f64::NAN)).expect("writable");
    assert_eq!(text, "\"nan\"");
    let back: v1::Num = serde_json::from_str(&text).expect("readable");
    assert!(back.0.is_nan());
    assert_eq!(serde_json::to_string(&v1::Num(f64::INFINITY)).unwrap(), "\"inf\"");
    assert_eq!(serde_json::to_string(&v1::Num(f64::NEG_INFINITY)).unwrap(), "\"-inf\"");
    // And a value written as a string is read as one, which is what makes a hand-edited file load.
    assert!(serde_json::from_str::<v1::Num>("\"inf\"").unwrap().0.is_infinite());
    assert_eq!(serde_json::from_str::<v1::Num>("7").unwrap().0, 7.0);
}

#[test]
fn test_a_bad_file_is_refused_with_something_a_user_can_read() {
    // Five ways a file can be wrong, each with its own sentence, and in every case the app that was
    // asked to open it is left exactly as it was. That last clause is what "build, validate, swap"
    // means, and it is the half of the design a message test alone would not check.
    let mut app = default_app();
    play_sim(&mut app, 3.0);
    let standing = app.sim.fingerprint();
    let good = to_bytes(&app.snapshot("")).expect("writable");

    // The three text cases are made by editing the document rather than by patching its text, so
    // that no test breaks because a number came out spelled differently from the way this file
    // spelled it.
    let edited = |change: fn(&mut serde_json::Value)| -> Vec<u8> {
        let mut value: serde_json::Value =
            serde_json::from_str(&to_json(&app.snapshot("")).expect("writable")).expect("valid");
        change(&mut value);
        serde_json::to_vec(&value).expect("writable")
    };
    let wrong_format = edited(|v| v["format"] = serde_json::json!("some-other-program"));
    let newer = edited(|v| v["version"] = serde_json::json!(3));
    // One number changed, on a field no invariant constrains beyond being finite: the document
    // still parses and the run it describes still satisfies the physics, and the state hash is what
    // notices. Alice's own clock is exactly such a field, and it is in the fingerprint.
    let tampered = edited(|v| {
        let tau = v["sim"]["alice"]["tau"].as_f64().expect("Alice is in this run");
        v["sim"]["alice"]["tau"] = serde_json::json!(tau + 1e-6);
    });

    let cases: [(&str, Vec<u8>, &str); 5] = [
        ("a file from another program", wrong_format, "not a Black Hole Lab save"),
        ("a file from a newer build", newer, "saved by a newer version"),
        ("a truncated download", good[..good.len() / 2].to_vec(), "truncated"),
        ("something that is not a save at all", b"not json at all".to_vec(), "not readable"),
        ("a file somebody has edited", tampered, "has been altered"),
    ];
    for (what, bytes, expected) in cases {
        let complaint = read_document(&bytes)
            .and_then(|document| rebuild(&document).map(|_| ()))
            .expect_err(&format!("{what} must not load"))
            .to_string();
        assert!(
            complaint.contains(expected),
            "{what}: the refusal should mention {expected:?} and says {complaint:?}"
        );
        assert_eq!(app.sim.fingerprint(), standing, "{what}: the running app was touched");
    }

    // The same through the app's own door, which is the one a user reaches: a path that is not
    // there at all, and a file that is there and is rubbish.
    let mut running = default_app();
    play_sim(&mut running, 1.0);
    let standing = running.sim.fingerprint();
    assert!(running.load_from(std::path::Path::new("no-such-file.bhl")).is_err());
    let path = scratch_file();
    std::fs::write(&path, b"not a save").expect("the scratch file");
    let complaint = running.load_from(&path).expect_err("rubbish must not load").to_string();
    let _ = std::fs::remove_file(&path);
    assert!(complaint.contains("not readable"), "{complaint:?}");
    assert_eq!(running.sim.fingerprint(), standing, "a failed load left the run alone");
}

#[test]
fn test_a_load_the_user_asked_for_that_fails_says_so_and_changes_nothing() {
    // `load_chosen` is the whole of the Load button except the dialog: what happens once a path has
    // been chosen, whether the user chose it in the dialog, dropped the file on the window or named
    // it on the command line. A native dialog cannot be driven from a test, so this is the seam the
    // button is cut at and this is what a test can hold to account.
    let mut app = default_app();
    play_sim(&mut app, 1.0);
    app.controls.is_playing = true;
    let standing = app.sim.fingerprint();
    let path = scratch_file();
    std::fs::write(&path, b"not a save").expect("the scratch file");

    // No autosave directory, so this test writes nowhere but its own scratch file. That is the
    // reason the directory is a parameter: the window passes the user's own and a test does not.
    app.load_chosen(&path, None);
    let _ = std::fs::remove_file(&path);

    let status = app.controls.file_status.as_ref().expect("the panel is told what happened");
    assert!(status.failed, "and told that the load failed: {:?}", status.text);
    assert!(
        status.text.contains("Could not load") && status.text.contains("not readable"),
        "in the words the error itself uses: {:?}",
        status.text
    );
    assert_eq!(app.sim.fingerprint(), standing, "the run is the run that was standing");
    assert!(app.controls.is_playing, "and a run that was playing is playing still");
}

#[test]
fn test_the_two_buttons_round_trip_a_run_and_keep_their_own_state_out_of_the_file() {
    let mut app = default_app();
    play_sim(&mut app, 2.0);
    let standing = app.sim.fingerprint();
    // A request standing on the panel and a status line left by an earlier action. Both are
    // transients of this session; neither is a setting, and neither may reach the file.
    app.controls.file_request = Some(FileRequest::Save);
    app.controls.file_status =
        Some(FileStatus { text: "Saved some earlier file".to_string(), failed: false });

    let path = scratch_file();
    app.save_chosen(&path);
    let saved = app.controls.file_status.as_ref().expect("the panel is told what happened");
    assert!(!saved.failed, "the save should succeed and says {:?}", saved.text);
    assert!(saved.text.starts_with("Saved "), "and say so: {:?}", saved.text);

    // The schema itself, as text. `v1::Controls` has no field for either transient, so the two
    // strings below appear nowhere in a document - and neither does the status the panel was
    // carrying when the snapshot was taken.
    let json = to_json(&app.snapshot("")).expect("writable");
    for absent in ["file_request", "file_status", "Saved some earlier file"] {
        assert!(!json.contains(absent), "a document must not carry {absent:?}");
    }

    let mut restored = default_app();
    restored.controls.is_playing = true;
    restored.controls.file_request = Some(FileRequest::Load);
    restored.load_chosen(&path, None);
    let _ = std::fs::remove_file(&path);

    let status = restored.controls.file_status.as_ref().expect("the panel is told what happened");
    assert!(!status.failed, "the load should succeed and says {:?}", status.text);
    assert!(
        status.text.starts_with("Loaded ") && status.text.contains("The run is paused."),
        "and name the file and say what a load did to the run: {:?}",
        status.text
    );
    assert_eq!(restored.sim.fingerprint(), standing, "the run is the run that was saved");
    assert!(!restored.controls.is_playing, "and a load comes up paused");
    assert!(
        restored.controls.file_request.is_none(),
        "and `controls_from_v1` leaves the request at its default rather than at the file's"
    );
}

/// Where the committed golden file lives, for the generator below.
fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/save/golden/v1.json")
}

#[test]
fn test_this_build_still_opens_the_committed_version_1_save() {
    // `golden/v1.json` is a real version-1 save of a deliberately tiny run, committed as readable
    // JSON. It is the only test here that is about a file rather than about a round trip: every
    // other one writes with this build and reads with this build, and would go on passing if both
    // halves changed together.
    //
    // **The file is never regenerated.** When the format moves to version 2 it stays exactly as it
    // is and `golden/v2.json` is added beside it; the whole value of this file is that it was
    // written by a build that no longer exists, and rewriting it with today's writer throws that
    // away and leaves a test that proves nothing. `test_write_the_golden_file` below, which is
    // `#[ignore]`d, is how the next version's golden is made.
    let text = include_str!("golden/v1.json");
    let document = read_document(text.as_bytes()).expect("the committed golden file must load");
    assert_eq!(document.version, 1);
    let loaded = rebuild(&document).expect("and rebuild into a run");
    loaded.sim.check_invariants().expect("that satisfies the invariants of the physics");
    assert!(loaded.sim.clock > 0.0, "it is a run that has been played, not a fresh one");
    assert!(
        loaded.sim.alice_signal.pulses.iter().any(|p| !p.rays.is_empty()),
        "with light in it"
    );
    // And an arrival, which is the schema a golden file is most worth having: four of a
    // `Reception`'s fields are never drawn and exist only for the pass that reconciles a rewind,
    // so nothing on screen would show that they had stopped being read correctly.
    assert!(
        loaded.sim.alice_signal.received_count() > 0,
        "and an arrival on the record, with its segment, turn, side and pass time"
    );
    assert!(!loaded.controls.is_playing, "and it comes up paused");
}

#[test]
fn test_the_step_grain_comes_back_from_a_file_and_an_older_file_opens_at_the_default() {
    // The Step Size dropdown is the first *additive* field this format has taken, and the three
    // things that makes true are checked together: a file carries the grain by a slug of its own,
    // every slug this build writes is one it reads, and a file from before the dropdown existed
    // opens at the panel's default instead of refusing to open.
    for grain in StepGrain::ALL {
        assert_eq!(StepGrain::from_key(grain.key()), Some(grain), "{}", grain.key());
    }

    let metric = default_app().sim.metric;
    let controls =
        AppControls { step_grain: StepGrain::TenFrames, play_speed: 3.0, ..AppControls::default() };
    let written = convert::controls_to_v1(&controls, &metric);
    assert_eq!(written.step_grain.as_deref(), Some("ten-frames"), "a slug, not a label");
    // The two amount fields are written for a build that has never heard of the dropdown: ten
    // frames at 3 M a real second is half an M a press, quoted in M and in kilometres.
    assert_eq!(controls.press_amount(), 0.5);
    assert_eq!(written.step_size.0, 0.5);
    assert_eq!(written.step_distance_km.0, metric.r_to_km(0.5));

    // Through the JSON and back, because what a file carries is the text.
    let text = serde_json::to_string(&written).expect("writable");
    let read: v1::Controls = serde_json::from_str(&text).expect("readable");
    assert_eq!(convert::controls_from_v1(&read).step_grain, StepGrain::TenFrames);

    // The committed golden file predates the dropdown, so it is the real version-1 document
    // without the field, and it loads at one played frame.
    let golden = read_document(include_str!("golden/v1.json").as_bytes()).expect("the golden loads");
    assert_eq!(golden.controls.step_grain, None, "the golden file carries no grain");
    let default_grain = AppControls::default().step_grain;
    assert_eq!(convert::controls_from_v1(&golden.controls).step_grain, default_grain);

    // A slug from some later version falls back the same way rather than refusing the file.
    let unknown = v1::Controls { step_grain: Some("half-a-frame".to_string()), ..written };
    assert_eq!(convert::controls_from_v1(&unknown).step_grain, default_grain);
}

#[test]
fn test_the_comets_setting_comes_back_from_a_file_and_an_older_file_opens_with_the_columns_on() {
    // The Comets dropdown is two additive fields: the stride "Ray comets" always wrote, which
    // keeps its meaning, and a key that says which entry the panel was on, the only way a file
    // can say Off. Every entry the panel offers survives the text of a file.
    let metric = default_app().sim.metric;
    for comets in ChartComets::ALL {
        let controls = AppControls { comets, ..AppControls::default() };
        let written = convert::controls_to_v1(&controls, &metric);
        assert_eq!(written.ray_comet_stride, comets.stride() as u64, "{comets:?}");
        assert_eq!(written.comets.as_deref(), Some(comets.key()), "a slug, not a label");
        let text = serde_json::to_string(&written).expect("writable");
        let read: v1::Controls = serde_json::from_str(&text).expect("readable");
        assert_eq!(convert::controls_from_v1(&read).comets, comets);
    }
    assert_eq!(
        AppControls::default().comets,
        ChartComets::Columns,
        "the panel opens on the column comets alone, the picture every earlier build drew"
    );

    // A file from before the Off entry carries a stride and no key. A stride of 0 meant the
    // column comets alone, because every build that wrote one drew them, and a stride of k meant
    // the column comets with every k-th ray's comet over them.
    let old = convert::controls_to_v1(&AppControls::default(), &metric);
    for stride in [0, 32, 8, 2, 1, 5] {
        let file = v1::Controls { ray_comet_stride: stride, ..old.clone() };
        let mut json = serde_json::to_value(&file).expect("writable");
        let fields = json.as_object_mut().expect("the panel is a JSON object");
        assert!(fields.remove("comets").is_some(), "this build writes the key");
        let read: v1::Controls = serde_json::from_value(json).expect("readable");
        assert_eq!(read.comets, None, "an older file has no key at all");
        let expected = if stride == 0 {
            ChartComets::Columns
        } else {
            ChartComets::Rays(stride as usize)
        };
        assert_eq!(convert::controls_from_v1(&read).comets, expected, "stride {stride}");
    }

    // A slug from some later version falls back on the stride rather than refusing the file,
    // and a "rays" key with no stride to draw reads as the column comets.
    let sparkles = Some("sparkles".to_string());
    let unknown = v1::Controls { comets: sparkles, ray_comet_stride: 8, ..old.clone() };
    assert_eq!(convert::controls_from_v1(&unknown).comets, ChartComets::Rays(8));
    let strideless = v1::Controls { comets: Some("rays".to_string()), ray_comet_stride: 0, ..old };
    assert_eq!(convert::controls_from_v1(&strideless).comets, ChartComets::Columns);

    // The committed golden file predates both fields, so it is the real version-1 document
    // without them, and it opens with the column comets on.
    let golden = read_document(include_str!("golden/v1.json").as_bytes()).expect("the golden loads");
    assert_eq!(golden.controls.ray_comet_stride, 0, "the golden file carries no stride");
    assert_eq!(golden.controls.comets, None, "the golden file carries no key");
    assert_eq!(convert::controls_from_v1(&golden.controls).comets, ChartComets::Columns);
}

#[test]
fn test_the_decimal_mark_travels_with_the_run_and_an_older_file_opens_in_point_style() {
    // "Decimal is comma" is saved with the run, and like the Step Size dropdown it is an additive
    // field: a file from before the box existed has no such field and opens with the box unticked,
    // which is the style every build before it wrote in.
    let metric = default_app().sim.metric;
    for comma in [false, true] {
        let controls = AppControls { decimal_is_comma: comma, ..AppControls::default() };
        let written = convert::controls_to_v1(&controls, &metric);
        let text = serde_json::to_string(&written).expect("writable");
        let read: v1::Controls = serde_json::from_str(&text).expect("readable");
        assert_eq!(convert::controls_from_v1(&read).decimal_is_comma, comma);
    }

    let golden = read_document(include_str!("golden/v1.json").as_bytes()).expect("the golden loads");
    assert!(!golden.controls.decimal_is_comma, "the golden file predates the box");
    assert!(!convert::controls_from_v1(&golden.controls).decimal_is_comma);
}

#[test]
fn test_a_shut_box_comes_back_from_a_file_and_an_older_file_opens_every_box() {
    // The second additive field, and the same three things asked of it: a shut box is carried by
    // the slugs it is already filed under, a file written before the disclosure triangle existed
    // opens with every box open, and a slug this build has never heard of is dropped rather than
    // refusing the file.
    let mut app = default_app();
    let shut = &mut app.spatial_canvas.telemetry.collapsed;
    // A fresh app starts with every box shut; this test files exactly two.
    shut.clear();
    shut.insert((Canvas::Spatial, BoxId::Observer(Who::Bob)));
    shut.insert((Canvas::Spatial, BoxId::CauchyHorizon));
    let view =
        convert::view_to_v1(&app.spacetime_canvas, &app.spatial_canvas, &app.volume_canvas);
    assert_eq!(
        view.spatial
            .telemetry
            .collapsed
            .iter()
            .map(|c| (c.canvas.as_str(), c.box_id.as_str()))
            .collect::<Vec<_>>(),
        vec![("spatial", "bob"), ("spatial", "cauchy-horizon")],
        "sorted on the slugs, so two saves of one state are one file"
    );
    assert!(view.spatial.telemetry.placements.is_empty(), "and a shut box is not a moved one");

    // Through the JSON and back, because what a file carries is the text.
    let text = serde_json::to_string(&view).expect("writable");
    let read: v1::View = serde_json::from_str(&text).expect("readable");
    let mut restored = SpacetimeApp::default();
    let put = |view: &v1::View, app: &mut SpacetimeApp| {
        convert::apply_view_v1(
            view,
            &mut app.spacetime_canvas,
            &mut app.spatial_canvas,
            &mut app.volume_canvas,
        );
    };
    put(&read, &mut restored);
    assert_eq!(
        restored.spatial_canvas.telemetry.collapsed, app.spatial_canvas.telemetry.collapsed,
        "the shut boxes came back shut"
    );

    // A box named by a canvas or a subject from some later version is skipped, exactly as an
    // unknown placement is, and the boxes this build does know still come back.
    let mut unknown = read.clone();
    unknown.spatial.telemetry.collapsed = vec![
        v1::CollapsedBox { canvas: "hyperbolic".to_string(), box_id: "bob".to_string() },
        v1::CollapsedBox { canvas: "spatial".to_string(), box_id: "carol".to_string() },
        v1::CollapsedBox { canvas: "spatial".to_string(), box_id: "bob".to_string() },
    ];
    put(&unknown, &mut restored);
    assert_eq!(restored.spatial_canvas.telemetry.collapsed.len(), 1, "two dropped, one kept");
    assert!(
        restored
            .spatial_canvas
            .telemetry
            .collapsed
            .contains(&(Canvas::Spatial, BoxId::Observer(Who::Bob)))
    );

    // The committed golden file predates the triangle, so it is the real version-1 document
    // without the field, and every box in it opens.
    let golden = read_document(include_str!("golden/v1.json").as_bytes()).expect("the golden loads");
    for telemetry in [
        &golden.view.spacetime.telemetry,
        &golden.view.spatial.telemetry,
        &golden.view.volume.telemetry,
    ] {
        assert!(telemetry.collapsed.is_empty(), "the golden file shuts no box");
    }
    put(&golden.view, &mut restored);
    assert!(
        restored.spacetime_canvas.telemetry.collapsed.is_empty()
            && restored.spatial_canvas.telemetry.collapsed.is_empty()
            && restored.volume_canvas.telemetry.collapsed.is_empty(),
        "so the run comes up with every box open"
    );
}

/// How the golden file was made, and how the next one is made.
///
/// ```text
/// cargo test --target-dir target/probe -- --ignored save::tests::test_write_the_golden_file
/// ```
///
/// writes `src/save/golden/v1.json` from the run below. It is `#[ignore]`d rather than deleted so
/// that the recipe for a golden file is the code that produced one and not a paragraph describing
/// it.
///
/// The run is small on purpose, and small in the three ways a save is actually large.
///
/// **Steps**, because the two trails grow by one entry per step and are the largest single thing in
/// a small file: this takes thirty hand steps of 0.1 M, which is a run the app can be in - somebody
/// holding the right arrow down at the default Δt - rather than the 180 frames a played 3 M leaves
/// behind. **Rays**, at eight a pulse instead of 144. **Wavefronts**, at four instead of 64, with
/// the emission interval opened out to 0.4 of Alice's proper time so that four of them still span
/// the time a pulse needs to reach Bob; at the standing 0.1 the cap evicts every pulse before it
/// arrives.
///
/// The layout is chosen so that an arrival happens inside all that. `Reception` is a whole schema,
/// and the four fields of it that nothing draws - the segment, the turn, the side and the time of
/// the pass - are exactly the ones a future version is most likely to get wrong, so a golden file
/// with no arrival in it leaves them untested. Alice falls from 5.5 M and Bob hovers at 4 M below
/// her: her ingoing light runs at dr/dt = -1 onto a receiver who is not running away from it, and
/// the first sheet reaches him before t = 2 M. The app's own opening layout, where Bob is falling
/// too, takes until 4 M and needs 32 rays before the swept patch finds him at all.
///
/// The result is a few tens of kilobytes of readable JSON - a file somebody can scroll through -
/// against the megabytes a steady-state field comes to.
///
/// For version 2, copy this, point it at `golden/v2.json`, and leave both the old file and the test
/// above it exactly alone.
#[test]
#[ignore = "writes into the source tree; run by hand when a new format version needs a golden"]
fn test_write_the_golden_file() {
    let mut app = default_app();
    app.controls.rays_per_pulse = 8;
    app.controls.max_pulses = 4;
    app.controls.alice.release = Release::FromInfinity;
    app.controls.alice.mode = ObserverMode::FreeFall;
    app.controls.alice.l_ang = 0.0;
    app.controls.alice.drop_r = 5.5;
    app.controls.alice.drop_phi = 0.0;
    app.controls.bob.drop_r = 4.0;
    app.controls.bob.drop_phi = 0.0;
    // Bob waits out the whole run, which is what makes him a receiver standing still under the
    // light rather than one falling away from it. He sends nothing: one transmission is enough to
    // exercise the schema, and the second would double the rays in the file.
    app.controls.bob.delta_t_delay = 1000.0;
    app.controls.bob.transmit = false;
    app.controls.drop_observers(&mut app.sim);
    app.controls.view_reset_requested = false;
    app.sim.alice_signal.interval_tau = 0.4;
    for _ in 0..30 {
        app.sim.set_max_pulses(app.controls.max_pulses);
        app.step_forward(0.1);
    }
    assert!(
        app.sim.alice_signal.received_count() > 0,
        "a golden file with no arrival in it leaves a whole schema untested"
    );
    let text = to_json(&app.snapshot("the committed golden file: a tiny run, never regenerated"))
        .expect("writable");
    std::fs::create_dir_all(golden_path().parent().expect("golden has a directory"))
        .expect("the golden directory");
    std::fs::write(golden_path(), &text).expect("the golden file");
    println!("wrote {} ({} bytes)", golden_path().display(), text.len());
}
