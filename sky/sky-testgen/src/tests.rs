//! The cases' promises, checked on grids of a few pixels: `still` is the identity, `turn` turns the
//! way its definition says and comes back after a whole turn, and the program writes, resumes and
//! refuses the way it says it does.

use std::path::{Path, PathBuf};

use sky_format::{BundleReader, Frame, Grid, Num, STOPWATCH, fate};

use crate::cases::{Case, still_frame, turn_frame};
use crate::{Failure, Summary, run};

/// How far two f32 directions computed along different f64 paths may differ. Each path is exact
/// to about 1e-16 in f64 and then rounds once to f32, by at most half a unit in the last place; for
/// components no larger than 1 in size the two roundings together are at most one f32 epsilon
/// apart. Two epsilons leave room for the f64 error of an angle of 2 pi without hiding a pixel's
/// worth of turn, which on these grids is about 0.8.
const F32_ROUNDING: f32 = 2.0 * f32::EPSILON;

/// A grid small enough to be quick and large enough to have a seam, two poles and an equator.
const GRID: Grid = Grid::new(16, 8);

/// A directory of its own for one test, emptied first and removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir()
            .join("sky-testgen-tests")
            .join(format!("{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Self(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }

    fn arg(&self) -> String {
        self.0.to_string_lossy().into_owned()
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn args(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| s.to_string()).collect()
}

/// Runs the program, discarding its progress lines.
fn quiet(list: &[&str]) -> Result<Option<Summary>, Failure> {
    run(&args(list), &mut std::io::sink())
}

/// The sentence a bad command line is refused with.
fn refusal(list: &[&str]) -> String {
    match quiet(list) {
        Err(Failure::Usage(message)) => message,
        other => panic!("{list:?} should be refused as usage, and gave {other:?}"),
    }
}

fn close(a: [f32; 3], b: [f32; 3]) -> bool {
    a.iter().zip(b).all(|(p, q)| (p - q).abs() <= F32_ROUNDING)
}

fn assert_far_sky_unshifted(frame: &Frame) {
    assert!(frame.fate.iter().all(|&f| f == fate::FAR_SKY), "fate");
    assert!(frame.shift.iter().all(|&g| g == 1.0), "shift");
    assert!(frame.winding.iter().all(|&w| w == 0), "winding");
}

#[test]
fn test_still_looks_where_each_pixel_points_unshifted_and_unwound() {
    let frame = still_frame(GRID, 3);
    assert_eq!(frame.index, 3);
    assert_far_sky_unshifted(&frame);
    for j in 0..GRID.height {
        for i in 0..GRID.width {
            let n = GRID.pixel_direction(i, j);
            let d = frame.direction_at(i, j);
            // One rounding from f64 to f32 of a component no larger than 1 in size moves it by
            // at most half of f32's epsilon.
            for c in 0..3 {
                assert!(
                    (f64::from(d[c]) - n[c]).abs() <= f64::from(f32::EPSILON) / 2.0,
                    "pixel ({i}, {j}) component {c}: d = {} and n = {}",
                    d[c],
                    n[c]
                );
            }
        }
    }
}

#[test]
fn test_turn_at_time_zero_is_still_to_the_bit() {
    let still = still_frame(GRID, 0);
    let axis = [0.6, 0.0, 0.8];
    let turn = turn_frame(GRID, 0, 0.0, axis, 37.0);
    assert_far_sky_unshifted(&turn);
    for c in 0..3 {
        let bits = |f: &Frame| {
            f.direction[c]
                .iter()
                .map(|v| v.to_bits())
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(&turn), bits(&still), "component {c}");
    }
}

#[test]
fn test_turn_left_by_one_column_moves_the_sky_one_column_right() {
    // The derivation. Pixel i looks along n_i, whose azimuth about the triad's z is
    // phi_i = -lambda_i, with lambda_i = ((i + 0.5) / W - 0.5) 2 pi; consecutive columns differ
    // in lambda by 2 pi / W. A turn by the angle a about +Z, right-handed, adds a to the azimuth
    // and leaves the height alone, so pixel i sees the far-sky direction of azimuth
    //
    //     phi_i + a = -lambda_i + 2 pi / W = -(lambda_i - 2 pi / W) = -lambda_(i-1)
    //
    // at a = 2 pi / W: exactly the direction `still` shows at pixel i - 1, in the same row. At
    // i = 0 that is lambda_0 - 2 pi / W = lambda_(W-1) - 2 pi, the same direction as column W - 1,
    // so the comparison wraps at the seam.
    //
    // Read the other way, a star that `still` shows at column i - 1 appears at column i: the
    // observer turned left (a > 0 about +Z) and the sky moved right across the frame, as the
    // definition of `turn` says it must. A turn in the wrong sense would match column i + 1.
    let w = GRID.width;
    let still = still_frame(GRID, 0);
    // One column is 360 / W degrees; one second at 360 / W degrees per second turns by that.
    let turn = turn_frame(GRID, 1, 1.0, [0.0, 0.0, 1.0], 360.0 / f64::from(w));
    assert_far_sky_unshifted(&turn);
    for j in 0..GRID.height {
        for i in 0..w {
            let left = (i + w - 1) % w;
            assert!(
                close(turn.direction_at(i, j), still.direction_at(left, j)),
                "pixel ({i}, {j}) looks along {:?}, and still's pixel ({left}, {j}) along {:?}",
                turn.direction_at(i, j),
                still.direction_at(left, j)
            );
        }
    }
    // And not merely a symmetry of a small grid: the turn is not the identity, and not the turn
    // the other way.
    let right = |i: u32| (i + 1) % w;
    assert!(!close(turn.direction_at(3, 2), still.direction_at(3, 2)));
    assert!(!close(
        turn.direction_at(3, 2),
        still.direction_at(right(3), 2)
    ));
}

#[test]
fn test_turn_after_a_whole_period_is_still() {
    let still = still_frame(GRID, 0);
    for (axis, rate) in [([0.0, 0.0, 1.0], 6.0), ([0.0, 0.6, -0.8], -45.0)] {
        // A whole turn takes 360 / |rate| seconds.
        let turn = turn_frame(GRID, 1, 360.0 / f64::abs(rate), axis, rate);
        assert_far_sky_unshifted(&turn);
        for j in 0..GRID.height {
            for i in 0..GRID.width {
                assert!(
                    close(turn.direction_at(i, j), still.direction_at(i, j)),
                    "axis {axis:?}, pixel ({i}, {j})"
                );
            }
        }
    }
}

#[test]
fn test_turn_about_a_tilted_axis_keeps_directions_unit_and_moves_the_poles() {
    let tilted = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
    ];
    // 40 degrees: far enough that every pixel has moved well past rounding.
    let about_tilt = turn_frame(GRID, 1, 1.0, tilted, 40.0);
    let about_z = turn_frame(GRID, 1, 1.0, [0.0, 0.0, 1.0], 40.0);
    let still = still_frame(GRID, 0);
    for frame in [&about_tilt, &about_z] {
        assert_far_sky_unshifted(frame);
        for k in 0..GRID.len() {
            let d = frame.direction.each_ref().map(|plane| f64::from(plane[k]));
            let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            assert!(
                (length - 1.0).abs() <= 4.0 * f64::from(f32::EPSILON),
                "ray {k} has length {length}"
            );
        }
    }
    // The pole rows: a turn about Z slides them sideways and leaves their height alone, while a
    // turn about a tilted axis carries them away from the pole.
    let height_moved = |frame: &Frame, j: u32| {
        (0..GRID.width)
            .map(|i| (frame.direction_at(i, j)[2] - still.direction_at(i, j)[2]).abs())
            .fold(0.0_f32, f32::max)
    };
    for j in [0, GRID.height - 1] {
        assert!(
            height_moved(&about_z, j) <= F32_ROUNDING,
            "row {j} changed height under a turn about Z by {}",
            height_moved(&about_z, j)
        );
        assert!(
            height_moved(&about_tilt, j) > 0.1,
            "row {j} changed height under a tilted turn by only {}",
            height_moved(&about_tilt, j)
        );
    }
}

#[test]
fn test_case_frame_dispatches_to_the_case_functions() {
    let turn = Case::Turn {
        axis: [0.0, 0.0, 1.0],
        degrees_per_second: 90.0,
    };
    let bits = |f: &Frame| {
        f.direction[0]
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    };
    assert_eq!(
        bits(&turn.frame(GRID, 2, 0.5)),
        bits(&turn_frame(GRID, 2, 0.5, [0.0, 0.0, 1.0], 90.0))
    );
    assert_eq!(
        bits(&Case::Still.frame(GRID, 2, 0.5)),
        bits(&still_frame(GRID, 2))
    );
    // A quarter turn about +Z takes the forward direction to the left, +x to +y: the pixel just
    // right of the frame's centre on the equator, which looks nearly along +x, now sees nearly +Y.
    let centre = (GRID.width / 2, GRID.height / 2);
    assert!(still_frame(GRID, 0).direction_at(centre.0, centre.1)[0] > 0.9);
    assert!(turn.frame(GRID, 0, 1.0).direction_at(centre.0, centre.1)[1] > 0.9);
}

#[test]
fn test_a_written_bundle_reads_back_with_exact_proper_times_and_stopwatch() {
    let scratch = Scratch::new("end-to-end");
    let out = scratch.path().join("turn");
    let out_arg = out.to_string_lossy().into_owned();
    // 0.7 s at 10 fps: seven frames, at proper times that are not all exact in binary, so the
    // division k / fps is what the bundle must hold, not an accumulated sum of tenths.
    let summary = quiet(&[
        "--case",
        "turn",
        "--out",
        &out_arg,
        "--grid",
        "8x4",
        "--seconds",
        "0.7",
        "--fps",
        "10",
        "--rate",
        "30",
        "--axis",
        "1,1,1",
    ])
    .expect("the run succeeds")
    .expect("and is not a help request");
    assert_eq!(
        summary,
        Summary {
            written: 7,
            skipped: 0
        }
    );

    let reader = BundleReader::open(&out).expect("the bundle opens");
    let manifest = reader.manifest();
    assert_eq!(reader.complete(), (0..7).collect::<Vec<u32>>());
    assert_eq!(manifest.frames_planned, Some(7));
    assert_eq!(manifest.source.kind, "flat-space test");
    assert_eq!(manifest.source.name.as_deref(), Some("turn"));
    assert_eq!(manifest.geometry.kind, "flat");
    assert_eq!(manifest.far_sky.name, "galactic");
    assert_eq!(manifest.playback.frames_per_second, Num(10.0));
    assert_eq!(manifest.playback.proper_time_per_video_second, Num(1.0));
    assert_eq!(manifest.readouts[0].id, STOPWATCH);
    for (k, entry) in manifest.frames.iter().enumerate() {
        let t = k as f64 / 10.0;
        assert_eq!(entry.proper_time, Num(t), "frame {k}'s proper time");
        assert_eq!(
            entry.readouts.get(STOPWATCH),
            Some(&Num(t)),
            "frame {k}'s stopwatch"
        );
        let position = entry.position.as_ref().expect("a position");
        assert_eq!(position.chart, "cartesian");
        assert_eq!(position.coords, [Num(t), Num(0.0), Num(0.0), Num(0.0)]);
        let frame = reader.read_frame(k as u32).expect("the frame reads");
        let axis = [1.0 / 3f64.sqrt(); 3];
        let expected = turn_frame(Grid::new(8, 4), k as u32, t, axis, 30.0);
        for c in 0..3 {
            assert!(
                frame.direction[c]
                    .iter()
                    .zip(&expected.direction[c])
                    .all(|(a, b)| (a - b).abs() <= F32_ROUNDING),
                "frame {k} component {c}"
            );
        }
    }
    // The sum of tenths would have drifted by frame 3: 0.1 + 0.1 + 0.1 is not 0.3.
    assert_eq!(manifest.frames[3].proper_time, Num(0.3));
}

#[test]
fn test_resume_rewrites_only_the_missing_frame() {
    let scratch = Scratch::new("resume");
    let out = scratch.arg();
    let line = [
        "--case",
        "still",
        "--out",
        &out,
        "--grid",
        "6x3",
        "--seconds",
        "1",
        "--fps",
        "4",
    ];
    quiet(&line).expect("the first run succeeds");

    let frames = scratch.path().join("frames");
    let file = |k: u32| frames.join(format!("{k:06}.skyframe"));
    let before: Vec<(Vec<u8>, std::time::SystemTime)> = (0..4)
        .map(|k| {
            let path = file(k);
            (
                std::fs::read(&path).unwrap(),
                std::fs::metadata(&path).unwrap().modified().unwrap(),
            )
        })
        .collect();
    std::fs::remove_file(file(2)).unwrap();

    let mut resumed = line.to_vec();
    resumed.push("--resume");
    let summary = quiet(&resumed).unwrap().unwrap();
    assert_eq!(
        summary,
        Summary {
            written: 1,
            skipped: 3
        }
    );

    for (k, (bytes, modified)) in before.iter().enumerate() {
        let path = file(k as u32);
        assert_eq!(&std::fs::read(&path).unwrap(), bytes, "frame {k}'s bytes");
        if k != 2 {
            // Same bytes could be the same frame written again; the same modification time says
            // it was not written at all.
            let now = std::fs::metadata(&path).unwrap().modified().unwrap();
            assert_eq!(&now, modified, "frame {k} was rewritten");
        }
    }
    let reader = BundleReader::open(scratch.path()).unwrap();
    assert_eq!(reader.complete(), vec![0, 1, 2, 3]);

    // A second resume of a finished bundle writes nothing.
    assert_eq!(
        quiet(&resumed).unwrap().unwrap(),
        Summary {
            written: 0,
            skipped: 4
        }
    );
}

#[test]
fn test_each_bad_command_line_is_refused_with_its_own_sentence() {
    let scratch = Scratch::new("refusals");
    let out = scratch.arg();
    let fresh = scratch.path().join("never").to_string_lossy().into_owned();

    let unknown_case = refusal(&["--case", "spin", "--out", &fresh]);
    let bad_grid = refusal(&["--case", "still", "--out", &fresh, "--grid", "1024by512"]);
    let zero_axis = refusal(&["--case", "turn", "--out", &fresh, "--axis", "0,0,0"]);
    quiet(&[
        "--case",
        "still",
        "--out",
        &out,
        "--grid",
        "4x2",
        "--seconds",
        "0.1",
    ])
    .expect("a bundle to find in the way");
    let existing = refusal(&[
        "--case",
        "still",
        "--out",
        &out,
        "--grid",
        "4x2",
        "--seconds",
        "0.1",
    ]);

    assert!(unknown_case.contains("no case \"spin\""), "{unknown_case}");
    assert!(bad_grid.contains("--grid wants"), "{bad_grid}");
    assert!(zero_axis.contains("no direction"), "{zero_axis}");
    assert!(
        existing.contains("already holds a sky bundle"),
        "{existing}"
    );
    assert!(existing.contains("--resume"), "{existing}");
    let all = [&unknown_case, &bad_grid, &zero_axis, &existing];
    for (a, first) in all.iter().enumerate() {
        for second in &all[a + 1..] {
            assert_ne!(first, second);
        }
    }
    // None of these runs wrote anything where it was pointed.
    assert!(!Path::new(&fresh).exists());

    // The others the parser knows, each with a sentence of its own.
    let more = [
        refusal(&[]),
        refusal(&["--out", &fresh]),
        refusal(&["--case", "still"]),
        refusal(&["--case", "still", "--out", &fresh, "--colour", "red"]),
        refusal(&["--case", "still", "--out"]),
        refusal(&["--case", "still", "--case", "turn", "--out", &fresh]),
        refusal(&["--case", "still", "--out", &fresh, "--rate", "6"]),
        refusal(&["--case", "turn", "--out", &fresh, "--rate", "nan"]),
        refusal(&["--case", "turn", "--out", &fresh, "--axis", "1,2"]),
        refusal(&["--case", "still", "--out", &fresh, "--fps", "0"]),
        refusal(&["--case", "still", "--out", &fresh, "--grid", "0x4"]),
        refusal(&["--case", "still", "--out", &fresh, "--grid", "65536x65536"]),
        refusal(&["--case", "still", "--out", &fresh, "--seconds", "0.001"]),
        // Resuming with other settings than the bundle was started with.
        refusal(&[
            "--case",
            "turn",
            "--out",
            &out,
            "--grid",
            "4x2",
            "--seconds",
            "0.1",
            "--resume",
        ]),
    ];
    for (a, first) in more.iter().enumerate() {
        for second in more[a + 1..].iter().chain(all) {
            assert_ne!(first, second);
        }
    }
    assert!(
        more.last().unwrap().contains("different case"),
        "{}",
        more.last().unwrap()
    );
}

#[test]
fn test_help_is_printed_whatever_else_is_on_the_line() {
    let mut out = Vec::new();
    let result = run(&args(&["--case", "nonsense", "--help"]), &mut out);
    assert!(matches!(result, Ok(None)));
    let text = String::from_utf8(out).unwrap();
    // The sense of the turn is part of the help, because it is part of the definition.
    assert!(text.contains("turns the observer to the LEFT"));
    assert!(text.contains("move to the RIGHT"));
}
