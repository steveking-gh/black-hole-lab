//! The program as a whole: it films a save into a bundle the format's reader opens, with the
//! frames at the proper times it promises and the read-outs right; it resumes and refuses the way
//! it says; and each bad command line has a sentence of its own. Grids of a few pixels throughout.

use std::io::Read as _;
use std::path::{Path, PathBuf};

use kerr_equatorial::KerrSchild;
use sky_format::{BundleReader, Num, STOPWATCH};
use sky_trace::bhl;
use sky_trace::film::{self, COORDINATE_TIME, HORIZON_DISTANCE, WATCH};
use sky_trace::horizon;
use sky_trace::travel;
use sky_trace::worldline::Worldline;

use crate::run;
use crate::trace::{Failure, Summary};

/// A directory of its own for one test, emptied first and removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir()
            .join("sky-trace-tests")
            .join(format!("{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Self(dir)
    }

    fn join(&self, leaf: &str) -> String {
        self.0.join(leaf).to_string_lossy().into_owned()
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn repo_file(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

/// Runs the program, discarding what it prints.
fn quiet(list: &[&str]) -> Result<Option<Summary>, Failure> {
    let args: Vec<String> = list.iter().map(|s| s.to_string()).collect();
    run(&args, &mut std::io::sink())
}

/// The sentence a command line is refused with before anything is written.
fn refusal(list: &[&str]) -> String {
    match quiet(list) {
        Err(Failure::Usage(message)) => message,
        other => panic!("{list:?} should be refused, and gave {other:?}"),
    }
}

/// The JSON of a save, gzipped or not, read without this crate's reader.
fn raw_save(path: &Path) -> serde_json::Value {
    let bytes = std::fs::read(path).unwrap();
    let mut text = Vec::new();
    if bytes.starts_with(&[0x1f, 0x8b]) {
        flate2::read::GzDecoder::new(&bytes[..])
            .read_to_end(&mut text)
            .unwrap();
    } else {
        text = bytes;
    }
    serde_json::from_slice(&text).unwrap()
}

#[test]
fn test_a_save_is_filmed_at_the_promised_proper_times_with_the_right_read_outs() {
    let scratch = Scratch::new("save");
    let out = scratch.join("bob");
    let save_path = repo_file("demos/near_fall.bhl");
    let save_arg = save_path.to_string_lossy().into_owned();
    let summary = quiet(&[
        &save_arg,
        "--out",
        &out,
        "--grid",
        "16x8",
        "--frames",
        "6",
        "--threads",
        "3",
    ])
    .unwrap()
    .unwrap();
    assert_eq!(
        (summary.frames, summary.written, summary.skipped),
        (6, 6, 0)
    );
    assert_eq!(summary.unresolved, 0);

    let reader = BundleReader::open(&out).unwrap();
    let m = reader.manifest();
    assert_eq!(reader.complete(), vec![0, 1, 2, 3, 4, 5]);
    assert_eq!(m.writer.program, "sky-trace");
    assert_eq!(m.source.kind, "scenario");
    assert_eq!(m.source.name.as_deref(), Some("near_fall.bhl"));
    let hash = raw_save(&save_path)["state_hash"]
        .as_str()
        .unwrap()
        .to_string();
    assert_eq!(hash.len(), 16);
    assert_eq!(m.source.state_hash.as_deref(), Some(hash.as_str()));
    assert_eq!(m.geometry.kind, "kerr");
    assert_eq!(m.geometry.mass, Some(Num(1.0)));
    assert_eq!(m.geometry.spin, Some(Num(0.9)));
    assert_eq!(m.time_unit.name, "M");
    assert_eq!(m.time_unit.seconds.0, 4.15e6 * bhl::GM_SUN_OVER_C3_SECONDS);
    assert_eq!(m.far_sky.name, "galactic");
    assert_eq!(m.playback.frames_per_second, Num(30.0));
    assert_eq!(m.playback.proper_time_per_video_second, Num(0.25));
    assert_eq!(m.frames_planned, Some(6));
    let observer = m.observer.as_ref().unwrap();
    assert_eq!(observer.name.as_deref(), Some("Bob"));
    assert!(observer.triad.as_ref().unwrap().contains("gradient of r"));
    let declared: Vec<(&str, &str, &str, u32)> = m
        .readouts
        .iter()
        .map(|r| (r.id.as_str(), r.label.as_str(), r.unit.as_str(), r.decimals))
        .collect();
    // The clocks, the watch under Bob's name; then, Bob being outside r+ on every frame, the
    // distance from it and no time since crossing it; the chart's radius is not a read-out.
    assert_eq!(
        declared[..4],
        [
            (STOPWATCH, "Stopwatch", "M", 3),
            (WATCH, "Bob's Watch", "M", 3),
            (COORDINATE_TIME, "Coordinate time", "M", 2),
            (
                HORIZON_DISTANCE,
                "Proper distance from the outer event horizon",
                "M",
                3
            ),
        ]
    );
    assert!(declared.iter().all(|d| d.0 != "radius"));
    // The travel read-outs follow: Bob starts at 2.27 M, outside the static limit, and in six
    // frames of 1/120 M is nowhere near it, so the static observer and the ZAMO are both there.
    let ids: Vec<&str> = declared[4..].iter().map(|d| d.0).collect();
    assert_eq!(
        ids,
        [
            "speed_static",
            "heading_static",
            "speed_zamo",
            "heading_zamo"
        ]
    );
    let marks: Vec<(&str, &str, &str)> = m
        .marks
        .iter()
        .map(|k| (k.id.as_str(), k.label.as_str(), k.shape.as_str()))
        .collect();
    assert_eq!(
        marks,
        [
            (
                "travel_static",
                "Direction of travel past the static observer",
                "ring"
            ),
            (
                "travel_zamo",
                "Direction of travel past the ZAMO",
                "diamond"
            ),
        ]
    );

    // The walker's own walk, at its own step: the same events to the integrator's rounding, and a
    // check on the read-outs that does not go through `film`.
    let save = bhl::read_file(&save_path).unwrap();
    let bob = save.bob.as_ref().unwrap();
    let metric = save.hole.metric();
    let walked = Worldline::from_saved(&metric, bob)
        .unwrap()
        .walk(0.25 / 30.0, 6);
    let tau0 = bob.tau;
    for (k, entry) in m.frames.iter().enumerate() {
        let e = walked.events[k];
        assert_eq!(entry.index, k as u32);
        // The promise, to the bit: tau_0 + k * rate / fps.
        assert_eq!(
            entry.proper_time.0.to_bits(),
            (tau0 + k as f64 * 0.25 / 30.0).to_bits(),
            "frame {k}"
        );
        let readout = |id: &str| entry.readouts[id].0;
        assert_eq!(
            readout(STOPWATCH),
            entry.proper_time.0 - m.frames[0].proper_time.0
        );
        assert!((readout(STOPWATCH) - k as f64 / 120.0).abs() < 1e-15);
        // The watch and the coordinate time are the app's own: the observer's proper time and the
        // chart's time as the save counts them, not since the film's first frame.
        assert_eq!(readout(WATCH), entry.proper_time.0);
        let d = horizon::proper_distance_from_outer_horizon(&metric, e.r).unwrap();
        assert!((readout(HORIZON_DISTANCE) - d).abs() < 1e-12, "frame {k}");
        assert!((readout(COORDINATE_TIME) - e.t).abs() < 1e-12, "frame {k}");
        // The chart's radius is carried by the position alone.
        assert!(!entry.readouts.contains_key("radius"));
        let position = entry.position.as_ref().unwrap();
        assert_eq!(position.chart, "kerr-schild");
        let [t, r, theta, phi] = position.coords.map(|c| c.0);
        assert!((r - e.r).abs() < 1e-12, "frame {k}");
        assert_eq!(theta, std::f64::consts::FRAC_PI_2);
        assert_eq!(t, readout(COORDINATE_TIME));
        assert!((phi - e.phi).abs() < 1e-12);

        // The travel past each reference observer, as `travel` computes it at the event.
        let kerr = kerr_sky::Kerr::from_equatorial(&metric);
        let passings = travel::passing(&kerr, &e, &film::triad(&kerr, &e).unwrap());
        for p in &passings {
            let id = p.reference.id();
            assert_eq!(readout(&format!("speed_{id}")), p.speed, "frame {k}");
            let n = entry
                .marks
                .get(&format!("travel_{id}"))
                .map(|n| n.map(|c| c.0));
            assert_eq!(n, p.direction, "frame {k}");
            // The written heading is the direction's, up to whole turns (a film's heading is
            // continuous) and up to its rounding bound (zero to rounding is written +0.0).
            if let Some(heading) = p.heading() {
                let written = readout(&format!("heading_{id}"));
                let off = written - heading;
                let off = off - 360.0 * (off / 360.0).round();
                assert!(
                    off.abs() <= p.heading_rounding().max(1e-12),
                    "frame {k}: {written} written for {heading}"
                );
            }
        }

        // The specification's rule for rays not of the far sky, in every frame written.
        let frame = reader.read_frame(k as u32).unwrap();
        film::check_rays(&frame).unwrap();
        assert!(frame.fate.contains(&sky_format::fate::DARK));
        assert!(frame.fate.contains(&sky_format::fate::FAR_SKY));
    }
}

#[test]
fn test_resume_traces_only_the_missing_frame_and_refuses_other_settings() {
    let scratch = Scratch::new("resume");
    let out = scratch.join("hover");
    let line = [
        "--hover", "6", "--spin", "0.5", "--frames", "3", "--grid", "8x4", "--out", &out,
    ];
    assert_eq!(quiet(&line).unwrap().unwrap().written, 3);
    let frame_1 = Path::new(&out).join("frames").join("000001.skyframe");
    let before = std::fs::read(&frame_1).unwrap();
    std::fs::remove_file(&frame_1).unwrap();

    let again = refusal(&line);
    assert!(again.contains("already holds a sky bundle") && again.contains("--resume"));

    let mut resumed = line.to_vec();
    resumed.push("--resume");
    let summary = quiet(&resumed).unwrap().unwrap();
    assert_eq!((summary.written, summary.skipped), (1, 2));
    assert_eq!(
        std::fs::read(&frame_1).unwrap(),
        before,
        "the same frame, to the byte"
    );
    assert_eq!(BundleReader::open(&out).unwrap().complete(), vec![0, 1, 2]);
    let summary = quiet(&resumed).unwrap().unwrap();
    assert_eq!((summary.written, summary.skipped), (0, 3));

    for (change, value, named) in [
        ("--rate", "0.5", "--fps or --rate"),
        ("--fps", "24", "--fps or --rate"),
        ("--grid", "16x8", "grid (--grid)"),
        ("--spin", "0.6", "different hole"),
        ("--solar-masses", "10", "mass in solar masses"),
        ("--frames", "4", "length (--frames or --seconds)"),
        ("--hover", "7", "save or hover test"),
        ("--units", "geometric", "read-outs or their units (--units)"),
    ] {
        let mut other = resumed.clone();
        let at = other.iter().position(|a| *a == change);
        match at {
            Some(k) => other[k + 1] = value,
            None => other.extend([change, value]),
        }
        let why = refusal(&other);
        assert!(
            why.contains(named) && why.contains("resume it with the settings"),
            "{change} {value}: {why}"
        );
    }
}

#[test]
fn test_a_save_whose_hole_spins_the_other_way_is_refused() {
    let scratch = Scratch::new("negative-spin");
    let mut doc = raw_save(&repo_file("demos/near_fall.bhl"));
    doc["sim"]["metric"]["a"] = serde_json::json!(-0.9);
    let path = scratch.join("mirrored.bhl");
    std::fs::write(&path, serde_json::to_vec(&doc).unwrap()).unwrap();
    let why = refusal(&[&path, "--out", &scratch.join("out"), "--frames", "1"]);
    assert!(
        why.contains("spins the other way") && why.contains("+z"),
        "{why}"
    );
    assert!(
        !Path::new(&scratch.join("out")).exists(),
        "nothing is written"
    );
}

#[test]
fn test_each_bad_command_line_is_refused_with_its_own_sentence() {
    let scratch = Scratch::new("refusals");
    let out = scratch.join("never");
    let save = repo_file("demos/near_fall.bhl")
        .to_string_lossy()
        .into_owned();
    let golden = repo_file("src/save/golden/v1.json")
        .to_string_lossy()
        .into_owned();
    fn hover<'a>(more: &[&'a str]) -> Vec<&'a str> {
        let mut line = vec!["--hover", "6", "--spin", "0", "--frames", "2"];
        line.extend_from_slice(more);
        line
    }
    let cases: Vec<(Vec<&str>, &str)> = vec![
        (vec![], "nothing to do"),
        (vec!["--out", &out], "no observer is named"),
        (vec![&save], "--out is required"),
        (vec![&save, &save, "--out", &out], "two saves"),
        (
            vec![&save, "--hover", "6", "--out", &out],
            "both a save and --hover",
        ),
        (
            vec![&save, "--out", &out, "--spin", "0.5"],
            "describe the hole",
        ),
        (
            vec![&save, "--out", &out, "--step", "0.1"],
            "belongs to the dry run",
        ),
        (vec![&save, "--out", &out, "--colour", "red"], "no option"),
        (vec![&save, "--out"], "needs a value"),
        (vec![&save, "--out", &out, "--out", &out], "given twice"),
        (
            vec![&save, "--out", &out, "--resume", "--resume"],
            "--resume is given twice",
        ),
        (
            vec![&save, "--out", &out, "--observer", "carol"],
            "no observer \"carol\"",
        ),
        (
            vec![&save, "--out", &out, "--grid", "64x64"],
            "twice as wide",
        ),
        (vec![&save, "--out", &out, "--grid", "64by32"], "<W>x<H>"),
        (vec![&save, "--out", &out, "--grid", "0x0"], "no rays"),
        (vec![&save, "--out", &out, "--fps", "0"], "--fps wants"),
        (vec![&save, "--out", &out, "--rate", "-1"], "--rate wants"),
        (vec![&save, "--out", &out, "--frames", "0"], "at least 1"),
        (
            vec![&save, "--out", &out, "--seconds", "nan"],
            "--seconds wants",
        ),
        (
            vec![&save, "--out", &out, "--seconds", "0.001"],
            "has no frames",
        ),
        (
            vec![&save, "--out", &out, "--frames", "2", "--seconds", "1"],
            "both set the film's length",
        ),
        (
            vec![&save, "--out", &out, "--threads", "0"],
            "--threads wants",
        ),
        (
            vec![&save, "--out", &out, "--units", "metric"],
            "--units wants physical",
        ),
        (vec![&golden, "--out", &out], "does not end within"),
        (
            vec!["--info", &save, "--out", &out],
            "no option --out in a dry run",
        ),
        (
            vec!["--hover", "6", "--frames", "2", "--out", &out],
            "needs --spin",
        ),
        (
            vec![
                "--hover", "six", "--spin", "0", "--frames", "2", "--out", &out,
            ],
            "is not a number",
        ),
        (
            vec![
                "--hover", "6", "--spin", "1", "--frames", "2", "--out", &out,
            ],
            "less than 1, and",
        ),
        (
            vec!["--hover", "6", "--spin", "0", "--out", &out],
            "needs a length",
        ),
        (
            hover(&["--observer", "bob", "--out", &out]),
            "has one; leave it out",
        ),
        (
            hover(&["--solar-masses", "-3", "--out", &out]),
            "--solar-masses wants",
        ),
        (
            vec![
                "--hover", "2", "--spin", "0.5", "--frames", "2", "--out", &out,
            ],
            "no static observer exists at r = 2 M",
        ),
    ];
    let mut seen: Vec<String> = Vec::new();
    for (line, says) in &cases {
        let why = refusal(line);
        assert!(why.contains(says), "{line:?}: {why}");
        assert!(!seen.contains(&why), "{line:?} shares its sentence: {why}");
        assert!(!why.contains('\n'), "one sentence: {why}");
        seen.push(why);
    }
    assert!(!Path::new(&out).exists(), "no refusal wrote anything");
    // A hover test is filmed whatever the save's hole, from the metric it names itself.
    let metric = KerrSchild {
        m: 1.0,
        a: 0.5,
        m_solar: 1.0,
    };
    assert!(film::hover_observer(&metric, 2.0001).is_ok());
}

#[test]
fn test_the_read_outs_are_displayed_in_seconds_and_kilometres_unless_geometric_is_asked_for() {
    // The same film twice, --units physical by default and --units geometric: the values stored
    // are the same to the bit, and only the declarations differ, by a display on each of the three
    // clocks and the two rulers. Bob's six frames at Sagittarius A* span 0.04 M of proper time, so
    // the stopwatch and the watch are shown in the unit of one M, 20.44 s: seconds; the distant
    // clock, t from 0 to 0.06 M, in seconds too. The radius, 2.27 M, is 13.89 million km, and the
    // proper distance from r+ there, 2.92 M, 17.9 million km.
    let scratch = Scratch::new("units");
    let save = repo_file("demos/near_fall.bhl")
        .to_string_lossy()
        .into_owned();
    let (physical, geometric) = (scratch.join("physical"), scratch.join("geometric"));
    for (out, more) in [
        (&physical, vec![]),
        (&geometric, vec!["--units", "geometric"]),
    ] {
        let mut args = vec![
            save.as_str(),
            "--out",
            out,
            "--grid",
            "8x4",
            "--frames",
            "6",
        ];
        args.extend(more);
        quiet(&args).unwrap().unwrap();
    }
    let physical = BundleReader::open(&physical).unwrap().manifest().clone();
    let geometric = BundleReader::open(&geometric).unwrap().manifest().clone();
    assert!(geometric.readouts.iter().all(|r| r.display.is_none()));
    let seconds = 4.15e6 * bhl::GM_SUN_OVER_C3_SECONDS;
    let shown: Vec<(&str, &str, f64, u32)> = physical.readouts[..4]
        .iter()
        .map(|r| {
            let d = r.display.as_ref().expect("a display on each of the four");
            (r.id.as_str(), d.unit.as_str(), d.scale.0, d.decimals)
        })
        .collect();
    let million_km = seconds * 299_792.458 / 1e6;
    assert_eq!(
        shown,
        [
            (STOPWATCH, "s", seconds, 2),
            (WATCH, "s", seconds, 2),
            (COORDINATE_TIME, "s", seconds, 2),
            (HORIZON_DISTANCE, "million km", million_km, 2),
        ]
    );
    assert!(physical.readouts[4..].iter().all(|r| r.display.is_none()));
    let strip = |m: &sky_format::Manifest| {
        let mut m = m.clone();
        for r in &mut m.readouts {
            r.display = None;
        }
        m
    };
    assert_eq!(
        strip(&physical),
        strip(&geometric),
        "the same bundle in other units"
    );
    // Either way it is a valid manifest, and it survives JSON to the bit.
    for m in [&physical, &geometric] {
        m.validate().unwrap();
        assert_eq!(&sky_format::Manifest::from_json(&m.to_json()).unwrap(), m);
    }
}

#[test]
fn test_a_still_of_bob_says_which_side_its_headings_are_on() {
    // One frame of Bob, as the app's Look Around makes it: the headings are magnitudes, each with
    // the side in its unit, and the manifest is valid and survives JSON. And a longer film of the
    // same save, through the ergosphere and r+, gives signed headings in one unit, and quotes
    // each reference observer exactly where it exists.
    let scratch = Scratch::new("still");
    let save = repo_file("demos/near_fall.bhl")
        .to_string_lossy()
        .into_owned();
    let still = scratch.join("still");
    quiet(&[&save, "--out", &still, "--grid", "8x4", "--frames", "1"])
        .unwrap()
        .unwrap();
    let m = BundleReader::open(&still).unwrap().manifest().clone();
    m.validate().unwrap();
    assert_eq!(sky_format::Manifest::from_json(&m.to_json()).unwrap(), m);
    let entry = &m.frames[0];
    let headings: Vec<_> = m
        .readouts
        .iter()
        .filter(|r| r.id.starts_with("heading_"))
        .collect();
    assert!(!headings.is_empty());
    for r in headings {
        let value = entry.readouts[&r.id].0;
        assert!(value >= 0.0, "{}: {value}", r.id);
        assert!(
            r.unit == travel::RIGHT_OF_THE_HOLE || r.unit == travel::LEFT_OF_THE_HOLE,
            "{}",
            r.unit
        );
        // The side agrees with the mark: to the viewer's right is -y.
        let n = entry.marks[&format!("travel_{}", &r.id["heading_".len()..])];
        assert_eq!(
            n[1].0 <= 0.0,
            r.unit == travel::RIGHT_OF_THE_HOLE,
            "{}",
            r.id
        );
    }

    let film = scratch.join("film");
    quiet(&[
        &save, "--out", &film, "--grid", "8x4", "--frames", "4", "--rate", "36",
    ])
    .unwrap()
    .unwrap();
    let m = BundleReader::open(&film).unwrap().manifest().clone();
    m.validate().unwrap();
    for r in m.readouts.iter().filter(|r| r.id.starts_with("heading_")) {
        assert_eq!(r.unit, travel::RIGHT_OF_THE_HOLE);
    }
    let r_plus = KerrSchild {
        m: 1.0,
        a: 0.9,
        m_solar: 1.0,
    }
    .outer_horizon();
    let mut seen = [false; 3];
    for entry in &m.frames {
        let r = entry.position.as_ref().unwrap().coords[1].0;
        let quoted = [
            entry.readouts.contains_key("speed_static"),
            entry.readouts.contains_key("speed_zamo"),
            entry.readouts.contains_key("speed_raindrop"),
        ];
        assert_eq!(quoted, [r > 2.0, r > r_plus, r <= r_plus], "r = {r}");
        for (s, q) in seen.iter_mut().zip(quoted) {
            *s |= q;
        }
    }
    assert_eq!(seen, [true; 3], "the film passes through all three regions");
}

#[test]
fn test_a_resumed_film_writes_the_same_continuous_headings() {
    // Bob's whole fall in 60 frames: his heading past the raindrop sits at 180 from frame 46 on,
    // where the written values are unwrapped from frame 46's. Frames lost on either side of that
    // run and traced again by --resume must come back with the same values to the bit, since
    // they follow from the walk of the whole worldline and not from what was written before.
    let scratch = Scratch::new("resume-headings");
    let out = scratch.join("bob");
    let save = repo_file("demos/near_fall.bhl")
        .to_string_lossy()
        .into_owned();
    let line = [
        save.as_str(),
        "--out",
        &out,
        "--grid",
        "8x4",
        "--frames",
        "60",
        "--rate",
        "1.98",
    ];
    quiet(&line).unwrap().unwrap();
    let whole = BundleReader::open(&out).unwrap().manifest().clone();
    let raindrop: Vec<f64> = whole.frames[46..]
        .iter()
        .map(|f| f.readouts["heading_raindrop"].0)
        .collect();
    assert!(
        raindrop
            .iter()
            .all(|h| (h.abs() - 180.0).abs() < 1e-6 && h.signum() == raindrop[0].signum()),
        "{raindrop:?}"
    );
    for k in [10, 47, 59] {
        std::fs::remove_file(Path::new(&out).join(sky_format::frame_file_name(k))).unwrap();
    }
    let mut resumed = line.to_vec();
    resumed.push("--resume");
    let summary = quiet(&resumed).unwrap().unwrap();
    assert_eq!((summary.written, summary.skipped), (3, 57));
    let again = BundleReader::open(&out).unwrap().manifest().clone();
    assert_eq!(
        again, whole,
        "the resumed bundle's manifest is the first one, to the bit"
    );
}

#[test]
fn test_the_percentage_traced_climbs_a_whole_point_at_a_time_to_100_across_the_frames() {
    // `sky-look` shows how far a view has got from these lines, so they must rise and end at 100:
    // three frames of a hover test, each a few rows, and a resumed run that has one frame left to
    // trace starts its count from the two already there.
    let scratch = Scratch::new("percent");
    let out = scratch.join("hover");
    let line: Vec<String> = [
        "--hover",
        "6",
        "--spin",
        "0.5",
        "--frames",
        "3",
        "--grid",
        "64x32",
        "--out",
        &out,
        "--verbose",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    let percentages = |args: &[String]| -> Vec<u32> {
        let mut printed = Vec::new();
        run(args, &mut printed).expect("a film");
        String::from_utf8(printed)
            .expect("UTF-8")
            .lines()
            .filter_map(|l| l.strip_prefix("progress ")?.strip_suffix('%')?.parse().ok())
            .collect()
    };
    let whole = percentages(&line);
    assert_eq!(whole.last(), Some(&100), "{whole:?}");
    assert!(whole.windows(2).all(|w| w[0] < w[1]), "rising: {whole:?}");
    assert!(whole.len() > 10, "more than a line a frame: {whole:?}");

    std::fs::remove_file(Path::new(&out).join("frames").join("000002.skyframe")).unwrap();
    let mut resumed = line.clone();
    resumed.push("--resume".into());
    let rest = percentages(&resumed);
    assert_eq!(rest.last(), Some(&100), "{rest:?}");
    assert!(
        rest[0] > 66,
        "counted from the two frames already traced: {rest:?}"
    );
}

#[test]
fn test_without_verbose_a_film_prints_its_progress_every_ten_percent_and_what_it_wrote() {
    // Nothing the command line already says: no summary, no rate lines, no bundle report.
    let scratch = Scratch::new("concise");
    let out = scratch.join("hover");
    let args: Vec<String> = [
        "--hover", "6", "--spin", "0.5", "--frames", "3", "--grid", "64x32", "--out", &out,
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    let mut printed = Vec::new();
    run(&args, &mut printed).expect("a film");
    let printed = String::from_utf8(printed).expect("UTF-8");
    let lines: Vec<&str> = printed.lines().collect();
    let (last, progress) = lines.split_last().expect("some output");
    let expected: Vec<String> = (1..=10).map(|k| format!("progress {}%", 10 * k)).collect();
    assert_eq!(progress, expected, "{printed}");
    assert!(
        last.starts_with("wrote 3 frames in ") && last.ends_with(" s"),
        "{printed}"
    );
}

#[test]
fn test_a_film_through_r_plus_gives_the_distance_outside_and_the_time_since_crossing_inside() {
    // Bob's fall in the demonstration save, a tenth of an M of his time a frame, from 2.27 M through
    // r+ = 1.44 M until the film ends before r-. Each frame carries one of the two read-outs, the one
    // for its side of r+; the time since the crossing runs as his watch does; and the crossing it
    // counts from lies between the last frame outside and the first inside, while the distance
    // falls toward 0 frame by frame on the way down to r+.
    let scratch = Scratch::new("through-r-plus");
    let out = scratch.join("fall");
    let save = repo_file("demos/near_fall.bhl")
        .to_string_lossy()
        .into_owned();
    quiet(&[
        &save, "--out", &out, "--grid", "8x4", "--frames", "40", "--rate", "3",
    ])
    .unwrap()
    .unwrap();
    let m = BundleReader::open(&out).unwrap().manifest().clone();
    let ids: Vec<&str> = m.readouts.iter().map(|r| r.id.as_str()).collect();
    assert_eq!(
        ids[2..5],
        [COORDINATE_TIME, HORIZON_DISTANCE, film::HORIZON_TIME]
    );
    let r_plus = bhl::read_file(Path::new(&save))
        .unwrap()
        .hole
        .metric()
        .outer_horizon();
    let (mut outside, mut inside) = (Vec::new(), Vec::new());
    for entry in &m.frames {
        let r = entry.position.as_ref().unwrap().coords[1].0;
        let has = |id: &str| entry.readouts.contains_key(id);
        let tau = entry.proper_time.0;
        if r > r_plus {
            assert!(has(HORIZON_DISTANCE) && !has(film::HORIZON_TIME), "r = {r}");
            outside.push((tau, entry.readouts[HORIZON_DISTANCE].0));
        } else {
            assert!(has(film::HORIZON_TIME) && !has(HORIZON_DISTANCE), "r = {r}");
            inside.push((tau, entry.readouts[film::HORIZON_TIME].0));
        }
    }
    assert!(
        outside.len() >= 3 && inside.len() >= 3,
        "{outside:?} {inside:?}"
    );
    // The distance falls toward 0; the time rises as the watch does, by the same amounts.
    assert!(outside.windows(2).all(|w| w[1].1 < w[0].1));
    for w in inside.windows(2) {
        assert!(
            ((w[1].1 - w[0].1) - (w[1].0 - w[0].0)).abs() < 1e-9,
            "{w:?}"
        );
    }
    let crossing = inside[0].0 - inside[0].1;
    let last_outside = outside.last().unwrap().0;
    assert!(
        last_outside < crossing && crossing < inside[0].0,
        "the crossing at tau = {crossing} lies between the frames at {last_outside} and {}",
        inside[0].0
    );
}
