//! The program as a whole: it films a save into a bundle the format's reader opens, with the
//! frames at the proper times it promises and the read-outs right; it resumes and refuses the way
//! it says; and each bad command line has a sentence of its own. Grids of a few pixels throughout.

use std::io::Read as _;
use std::path::{Path, PathBuf};

use kerr_equatorial::KerrSchild;
use sky_format::{BundleReader, Num, STOPWATCH};
use sky_trace::bhl;
use sky_trace::film::{self, DISTANT_CLOCK, RADIUS};
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
    assert_eq!(
        declared,
        vec![
            (STOPWATCH, "Stopwatch", "M", 3),
            (RADIUS, "Radius", "M", 3),
            (DISTANT_CLOCK, "Distant clock", "M", 2),
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
    let (tau0, t0) = (bob.tau, walked.events[0].t);
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
        assert!((readout(RADIUS) - e.r).abs() < 1e-12, "frame {k}");
        assert!(
            (readout(DISTANT_CLOCK) - (e.t - t0)).abs() < 1e-12,
            "frame {k}"
        );
        let position = entry.position.as_ref().unwrap();
        assert_eq!(position.chart, "kerr-schild");
        let [t, r, theta, phi] = position.coords.map(|c| c.0);
        assert_eq!((r, theta), (readout(RADIUS), std::f64::consts::FRAC_PI_2));
        assert_eq!(t - t0, readout(DISTANT_CLOCK));
        assert!((phi - e.phi).abs() < 1e-12);

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
