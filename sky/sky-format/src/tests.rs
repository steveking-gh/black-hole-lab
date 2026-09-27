//! The format's promises, checked: bit-exact round trips, the grid's orientation, a distinct
//! refusal for each way a bundle can be wrong, and a committed bundle this build must go on
//! reading.
//!
//! Every grid here is a handful of pixels. The suite is part of a workspace run that takes about
//! ten seconds, and nothing about a format needs a large grid to be tested.

use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use crate::frame::{Chunk, assemble};
use crate::*;

/// A directory of its own for one test, emptied first and removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir()
            .join("sky-format-tests")
            .join(format!("{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Self(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn sample_manifest(width: u32, height: u32) -> Manifest {
    Manifest {
        format: FORMAT.into(),
        version: VERSION,
        writer: WriterInfo {
            program: "sky-format tests".into(),
            version: env!("CARGO_PKG_VERSION").into(),
            git: None,
        },
        source: Source {
            kind: "flat-space test".into(),
            name: Some("sample".into()),
            state_hash: None,
        },
        geometry: Geometry {
            kind: "flat".into(),
            mass: None,
            spin: None,
        },
        time_unit: TimeUnit {
            name: "s".into(),
            seconds: Num(1.0),
        },
        observer: None,
        grid: GridSpec::new(width, height),
        far_sky: FarSky::galactic(),
        playback: Playback {
            frames_per_second: Num(30.0),
            proper_time_per_video_second: Num(1.0),
        },
        readouts: vec![ReadoutDecl {
            id: STOPWATCH.into(),
            label: "Stopwatch".into(),
            unit: "s".into(),
            decimals: 2,
        }],
        labels: Vec::new(),
        frames_planned: None,
        frames: Vec::new(),
    }
}

/// Rotates `n` by `angle` about +z.
fn turned(n: [f64; 3], angle: f64) -> [f64; 3] {
    let (s, c) = angle.sin_cos();
    [n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]]
}

/// A frame with every kind of ray in it: the far sky everywhere, turned by `turn` about z, except
/// for a shadow in the bottom row and one unresolved ray, whose NaNs carry payloads - quiet,
/// signalling, negative - that a careless reader or writer would lose.
fn sample_frame(width: u32, height: u32, index: u32, turn: f64, shift: f32) -> Frame {
    let grid = Grid::new(width, height);
    let mut frame = Frame::new(width, height, index);
    for j in 0..height {
        for i in 0..width {
            let k = grid.offset(i, j);
            let d = turned(grid.pixel_direction(i, j), turn);
            frame.fate[k] = fate::FAR_SKY;
            for (c, plane) in frame.direction.iter_mut().enumerate() {
                plane[k] = d[c] as f32;
            }
            frame.shift[k] = shift;
        }
    }
    let bottom = height - 1;
    for i in [width / 2 - 1, width / 2] {
        let k = grid.offset(i, bottom);
        frame.fate[k] = fate::PAST_HORIZON;
        frame.direction[0][k] = f32::from_bits(0x7fc0_0002);
        frame.direction[1][k] = f32::from_bits(0xffc0_0003);
        frame.direction[2][k] = f32::from_bits(0x7f80_0004);
        frame.shift[k] = f32::from_bits(0x7fc0_0005);
        frame.winding[k] = -1;
    }
    let k = grid.offset(0, bottom);
    frame.fate[k] = fate::UNRESOLVED;
    frame.direction[0][k] = f32::from_bits(0x7fc0_0001);
    frame.direction[1][k] = f32::from_bits(0x7fc0_0001);
    frame.direction[2][k] = f32::from_bits(0x7fc0_0001);
    frame.shift[k] = f32::from_bits(0x7fc0_0001);
    frame
}

fn bits(plane: &[f32]) -> Vec<u32> {
    plane.iter().map(|v| v.to_bits()).collect()
}

/// Every field of two frames equal to the bit, NaN payloads included.
fn assert_same_bits(a: &Frame, b: &Frame) {
    assert_eq!((a.width, a.height, a.index), (b.width, b.height, b.index));
    assert_eq!(a.fate, b.fate, "fate");
    for c in 0..3 {
        assert_eq!(
            bits(&a.direction[c]),
            bits(&b.direction[c]),
            "direction {c}"
        );
    }
    assert_eq!(bits(&a.shift), bits(&b.shift), "shift");
    assert_eq!(a.winding, b.winding, "winding");
    let point_bits = |p: &PointSource| {
        (
            p.n.map(f32::to_bits),
            p.shift.to_bits(),
            p.magnification.to_bits(),
            p.label,
        )
    };
    assert_eq!(
        a.points
            .as_ref()
            .map(|v| v.iter().map(point_bits).collect::<Vec<_>>()),
        b.points
            .as_ref()
            .map(|v| v.iter().map(point_bits).collect::<Vec<_>>()),
        "point sources"
    );
}

#[test]
fn test_a_frame_round_trips_to_the_bit_nan_payloads_and_all() {
    let mut frame = sample_frame(8, 4, 5, 0.3, 1.0625);
    frame.points = Some(vec![
        PointSource {
            n: [1.0, 0.0, 0.0],
            shift: 1.5,
            magnification: -2.25,
            label: 0,
        },
        PointSource {
            n: [0.0, -0.6, 0.8],
            shift: f32::from_bits(0x7fc0_1234),
            magnification: 0.125,
            label: NO_LABEL,
        },
    ]);
    // Once per way of storing a chunk, so that neither codec's path goes untested whichever one
    // the automatic choice happens to pick for a grid this small.
    for codec in [None, Some(Codec::Raw), Some(Codec::Deflate)] {
        let bytes = frame
            .encode_with(codec)
            .expect("a consistent frame encodes");
        let back = Frame::decode(&bytes, Path::new("memory")).expect("and decodes");
        assert_same_bits(&frame, &back);
    }
    // An empty point-source list is a chunk, and no list is no chunk: the two stay apart.
    for points in [Some(Vec::new()), None] {
        frame.points = points;
        let back = Frame::decode(&frame.encode().unwrap(), Path::new("memory")).unwrap();
        assert_same_bits(&frame, &back);
    }
}

#[test]
fn test_a_manifest_round_trips_through_json_non_finite_numbers_and_all() {
    let mut manifest = sample_manifest(8, 4);
    manifest.geometry = Geometry {
        kind: "kerr".into(),
        mass: Some(Num(1.0)),
        spin: Some(Num(0.9375)),
    };
    manifest.time_unit = TimeUnit {
        name: "M".into(),
        seconds: Num(4.925490947e-5),
    };
    manifest.observer = Some(Observer {
        name: Some("Bob".into()),
        triad: Some("the ZAMO frame boosted along Bob's velocity".into()),
    });
    manifest.readouts.push(ReadoutDecl {
        id: "blueshift".into(),
        label: "Blueshift".into(),
        unit: String::new(),
        decimals: 3,
    });
    manifest.labels.push(Label {
        id: 7,
        text: "Sirius".into(),
    });
    manifest.frames_planned = Some(900);
    let values = [f64::INFINITY, f64::NEG_INFINITY, f64::NAN, -0.0, 0.1 + 0.2];
    for (k, value) in values.into_iter().enumerate() {
        let mut entry = FrameEntry::new(k as u32, 10.0 + k as f64 / 3.0)
            .with_readout(STOPWATCH, k as f64 / 3.0)
            .with_readout("blueshift", value);
        entry.position = Some(Position {
            chart: "kerr-schild".into(),
            coords: [1.0 / 3.0, 6.0, PI / 2.0, -1e-300].map(Num),
        });
        entry.bytes = 1234 + k as u64;
        manifest.frames.push(entry);
    }
    let text = manifest.to_json();
    assert!(text.contains("\"inf\"") && text.contains("\"-inf\"") && text.contains("\"nan\""));
    let back = Manifest::from_json(&text).expect("a manifest this build wrote reads back");
    // `Num` compares by bits, so this is a bit-exact comparison of every number but the NaN.
    assert_eq!(back, manifest);
    assert!(back.frames[2].readouts["blueshift"].0.is_nan());
    assert!(
        back.frames[3].readouts["blueshift"].0.is_sign_negative(),
        "-0.0 keeps its sign"
    );
}

#[test]
fn test_a_manifest_with_a_field_this_build_does_not_know_still_opens() {
    let mut value: serde_json::Value =
        serde_json::from_str(&sample_manifest(8, 4).to_json()).unwrap();
    value["added_by_a_later_build"] = serde_json::json!({ "anything": [1, 2, 3] });
    value["grid"]["also_new"] = serde_json::json!("ignored");
    let back = Manifest::from_json(&value.to_string()).expect("unknown fields are ignored");
    assert_eq!(back, sample_manifest(8, 4));
}

#[test]
fn test_pixel_to_direction_to_pixel_is_the_identity() {
    for (width, height) in [(8, 4), (7, 3), (1, 1), (64, 32), (5, 9)] {
        let grid = Grid::new(width, height);
        for j in 0..height {
            for i in 0..width {
                // Off-centre points too, since a renderer asks for places between the rays. The
                // pixel's centre is half a pixel in from its corner.
                for (du, dv) in [(0.5, 0.5), (0.75, 0.2), (0.05, 0.95)] {
                    let (u, v) = (f64::from(i) + du, f64::from(j) + dv);
                    let n = grid.direction(u, v);
                    let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    assert!((length - 1.0).abs() < 1e-15, "n is a unit vector");
                    let (bu, bv) = grid.frame_coordinates(n);
                    assert!(
                        (bu - u).abs() < 1e-12 && (bv - v).abs() < 1e-12,
                        "{width} x {height}: ({u}, {v}) came back as ({bu}, {bv})"
                    );
                }
            }
        }
    }
}

#[test]
fn test_the_centre_column_looks_along_plus_x() {
    // An odd width has a column whose centre is exactly the frame's centre.
    let n = Grid::new(7, 3).pixel_direction(3, 1);
    assert!(
        (n[0] - 1.0).abs() < 1e-15 && n[1].abs() < 1e-15 && n[2].abs() < 1e-15,
        "{n:?}"
    );
    // An even width puts the centre on the line between two columns: the middle of the frame,
    // which is where the star maps put longitude zero as well.
    let (u, v) = Grid::new(8, 4).frame_coordinates([1.0, 0.0, 0.0]);
    assert!(
        (u - 4.0).abs() < 1e-12 && (v - 2.0).abs() < 1e-12,
        "({u}, {v})"
    );
}

#[test]
fn test_moving_right_in_the_frame_turns_toward_minus_y() {
    let grid = Grid::new(7, 3);
    assert!(
        grid.pixel_direction(4, 1)[1] < 0.0,
        "right of centre looks toward -y"
    );
    assert!(
        grid.pixel_direction(2, 1)[1] > 0.0,
        "left of centre looks toward +y"
    );
    // And the azimuth from x toward y falls steadily across a whole row.
    let grid = Grid::new(8, 4);
    let azimuths: Vec<f64> = (0..8)
        .map(|i| {
            let n = grid.pixel_direction(i, 1);
            n[1].atan2(n[0])
        })
        .collect();
    assert!(azimuths.windows(2).all(|w| w[1] < w[0]), "{azimuths:?}");
}

#[test]
fn test_the_top_row_is_nearest_plus_z() {
    let grid = Grid::new(8, 4);
    let heights: Vec<f64> = (0..4).map(|j| grid.pixel_direction(0, j)[2]).collect();
    assert!(
        heights[0] > 0.0 && heights.windows(2).all(|w| w[1] < w[0]),
        "{heights:?}"
    );
    // Row 0 of 4 is centred at latitude 67.5 degrees.
    assert!((heights[0] - (3.0 * PI / 8.0).sin()).abs() < 1e-15);
}

#[test]
fn test_the_checksum_is_the_crc_32_of_zlib_and_png() {
    // The standard check value: CRC-32 of the nine ASCII digits.
    assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
}

#[test]
fn test_a_reader_skips_an_unknown_chunk_and_reads_the_known_ones() {
    let frame = sample_frame(8, 4, 0, 0.0, 1.0);
    let mut chunks: Vec<Chunk> = frame
        .payloads()
        .into_iter()
        .map(|(tag, raw)| Chunk::pack(tag, &raw, None))
        .collect();
    // One chunk from a later build, one of the reserved refinement tag, and one whose codec this
    // version does not define: none of them known, so none of them read.
    chunks.insert(
        1,
        Chunk::pack(*b"XTRA", b"from a later build", Some(Codec::Deflate)),
    );
    chunks.push(Chunk::pack(tag::PTCH, &[0u8; 40], Some(Codec::Raw)));
    let mut odd = Chunk::pack(*b"ODDC", b"some future codec", Some(Codec::Raw));
    odd.crc ^= 1;
    chunks.push(odd);
    let mut bytes = assemble(8, 4, 0, &chunks);
    // Codec 9, patched into the last chunk's header.
    let last = bytes.len() - b"some future codec".len() - CHUNK_HEADER_LEN;
    bytes[last + 4] = 9;
    let back = Frame::decode(&bytes, Path::new("memory")).expect("unknown chunks are skipped");
    assert_same_bits(&frame, &back);
}

/// A bundle of `frames` sample frames, the manifest rewritten after each.
fn write_sample_bundle(dir: &Path, width: u32, height: u32, frames: u32) -> BundleWriter {
    let mut writer = BundleWriter::create(dir, sample_manifest(width, height)).unwrap();
    for index in 0..frames {
        let frame = sample_frame(width, height, index, 0.1 * f64::from(index), 1.0);
        let entry = FrameEntry::new(index, f64::from(index) / 30.0)
            .with_readout(STOPWATCH, f64::from(index) / 30.0);
        writer.write_frame_and_manifest(&frame, entry).unwrap();
    }
    writer
}

#[test]
fn test_a_bundle_round_trips_through_the_writer_and_the_reader() {
    let scratch = Scratch::new("round-trip");
    let dir = scratch.path().join("bundle");
    let writer = write_sample_bundle(&dir, 8, 4, 3);
    let reader = BundleReader::open(&dir).unwrap();
    assert_eq!(reader.manifest(), writer.manifest());
    assert_eq!(reader.complete(), vec![0, 1, 2]);
    for index in 0..3 {
        let expected = sample_frame(8, 4, index, 0.1 * f64::from(index), 1.0);
        assert_same_bits(&expected, &reader.read_frame(index).unwrap());
    }
    assert!(matches!(
        reader.read_frame(3),
        Err(Error::FrameNotWritten { index: 3 })
    ));
    assert!(
        matches!(
            BundleWriter::create(&dir, sample_manifest(8, 4)),
            Err(Error::AlreadyABundle { .. })
        ),
        "a new bundle is never written over an old one"
    );
}

#[test]
fn test_a_resumed_run_keeps_its_finished_frames_and_writes_the_rest_again() {
    let scratch = Scratch::new("resume");
    let dir = scratch.path().join("bundle");
    let mut writer = write_sample_bundle(&dir, 8, 4, 3);
    // Frame 3 reached its file and the run died before the manifest was rewritten: an orphan.
    let orphan = FrameEntry::new(3, 0.1).with_readout(STOPWATCH, 0.1);
    writer
        .write_frame(&sample_frame(8, 4, 3, 0.3, 1.0), orphan)
        .unwrap();
    drop(writer);
    // Frame 1 was cut short by something outside the writer, and a write was caught half done.
    let one = dir.join(frame_file_name(1));
    let bytes = std::fs::read(&one).unwrap();
    std::fs::write(&one, &bytes[..bytes.len() - 10]).unwrap();
    let stray = dir.join("frames/000004.skyframe.tmp");
    std::fs::write(&stray, b"half a frame").unwrap();

    let mut writer = BundleWriter::resume(&dir).unwrap();
    let done: Vec<bool> = (0..5).map(|k| writer.is_complete(k)).collect();
    assert_eq!(done, [true, false, true, false, false]);
    assert!(!stray.exists(), "temporary files are cleared");
    for index in [1, 3] {
        let frame = sample_frame(8, 4, index, 0.1 * f64::from(index), 1.0);
        let t = f64::from(index) / 30.0;
        let entry = FrameEntry::new(index, t).with_readout(STOPWATCH, t);
        writer.write_frame_and_manifest(&frame, entry).unwrap();
    }
    assert_eq!(
        BundleReader::open(&dir).unwrap().complete(),
        vec![0, 1, 2, 3]
    );
}

#[test]
fn test_each_way_a_bundle_can_be_wrong_is_refused_with_its_own_sentence() {
    let scratch = Scratch::new("refusals");
    let root = scratch.path();
    let mut complaints: Vec<(&str, String)> = Vec::new();
    let mut refuse = |what: &'static str, result: Result<(), Error>, fits: fn(&Error) -> bool| {
        let error = result.expect_err(what);
        assert!(fits(&error), "{what}: the wrong refusal, {error:?}");
        complaints.push((what, error.to_string()));
    };

    // Not a bundle: a plain file, a directory with no manifest, and a manifest of another format.
    let file = root.join("a-file.txt");
    std::fs::write(&file, b"hello").unwrap();
    refuse("a plain file", BundleReader::open(&file).map(drop), |e| {
        matches!(e, Error::NotABundle { .. })
    });
    let empty = root.join("empty");
    std::fs::create_dir_all(&empty).unwrap();
    refuse(
        "an empty directory",
        BundleReader::open(&empty).map(drop),
        |e| matches!(e, Error::NotABundle { .. }),
    );
    let foreign = root.join("foreign");
    std::fs::create_dir_all(&foreign).unwrap();
    std::fs::write(
        foreign.join(MANIFEST),
        r#"{"format": "something-else", "version": 1}"#,
    )
    .unwrap();
    refuse(
        "another format's manifest",
        BundleReader::open(&foreign).map(drop),
        |e| matches!(e, Error::NotABundle { .. }),
    );

    // A manifest from a newer version, whose shape this build cannot know.
    let newer = root.join("newer");
    std::fs::create_dir_all(&newer).unwrap();
    std::fs::write(
        newer.join(MANIFEST),
        format!(r#"{{"format": "{FORMAT}", "version": 2, "shape": "unknown"}}"#),
    )
    .unwrap();
    refuse(
        "a newer manifest",
        BundleReader::open(&newer).map(drop),
        |e| matches!(e, Error::NewerManifest { version: 2 }),
    );

    // The frame files of one good bundle, each spoilt in its own way.
    let dir = root.join("bundle");
    write_sample_bundle(&dir, 8, 4, 4);
    let reader = BundleReader::open(&dir).unwrap();
    let spoil = |index: u32, change: &dyn Fn(Vec<u8>) -> Vec<u8>| {
        let path = reader.frame_path(index);
        let bytes = std::fs::read(&path).unwrap();
        std::fs::write(&path, change(bytes)).unwrap();
    };
    spoil(0, &|bytes| bytes[..bytes.len() * 2 / 3].to_vec());
    refuse("a truncated frame", reader.read_frame(0).map(drop), |e| {
        matches!(e, Error::Truncated { .. })
    });
    // The CRC field of the first chunk, one bit of it: the file keeps its length and its
    // payload, and only the checksum can tell.
    spoil(1, &|mut bytes| {
        bytes[HEADER_LEN as usize + 8] ^= 0x01;
        bytes
    });
    refuse("a checksum mismatch", reader.read_frame(1).map(drop), |e| {
        matches!(e, Error::CrcMismatch { .. })
    });
    spoil(2, &|_| sample_frame(4, 2, 2, 0.0, 1.0).encode().unwrap());
    refuse(
        "a frame of another grid",
        reader.read_frame(2).map(drop),
        |e| {
            matches!(
                e,
                Error::DimensionMismatch {
                    found: (4, 2),
                    expected: (8, 4),
                    ..
                }
            )
        },
    );
    spoil(3, &|_| b"GIF89a, as it happens".to_vec());
    refuse(
        "a file that is not a frame",
        reader.read_frame(3).map(drop),
        |e| matches!(e, Error::NotAFrame { .. }),
    );

    // And a payload damaged where the codec cannot notice: a raw chunk with one bit flipped.
    let frame = sample_frame(8, 4, 0, 0.0, 1.0);
    let mut bytes = frame.encode_with(Some(Codec::Raw)).unwrap();
    bytes[HEADER_LEN as usize + CHUNK_HEADER_LEN] ^= 0x01;
    refuse(
        "a flipped bit",
        Frame::decode(&bytes, Path::new("x")).map(drop),
        |e| matches!(e, Error::CrcMismatch { .. }),
    );

    // A person reads these, so each has to say its own thing: no two alike, none panicking.
    for (k, (what, text)) in complaints.iter().enumerate() {
        for (other, other_text) in &complaints[..k] {
            assert_ne!(text, other_text, "{what} and {other} read the same");
        }
    }
}

/// Where the committed golden bundle lives.
fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("golden/v1")
}

#[test]
fn test_this_build_still_reads_the_committed_version_1_bundle() {
    // `golden/v1` is a real version-1 bundle, 8 x 4 rays and two frames, committed to the
    // repository. Every other test here writes with this build and reads with this build, and
    // would go on passing if both halves changed together; this one fails when the reader stops
    // reading what an earlier writer wrote.
    //
    // **The bundle is never regenerated.** When the format reaches version 2 it stays as it is and
    // `golden/v2` is added beside it. `test_write_the_golden_bundle`, which is `#[ignore]`d, is
    // how the next version's golden is made.
    //
    // The values below are written out as literals rather than recomputed from the grid formulae,
    // so that a change to the formulae fails here as well.
    let reader = BundleReader::open(golden_dir()).expect("the committed golden bundle opens");
    let manifest = reader.manifest();
    assert_eq!(manifest.version, 1);
    assert_eq!((manifest.grid.width, manifest.grid.height), (8, 4));
    assert_eq!(reader.complete(), vec![0, 1]);
    let stopwatch: Vec<f64> = manifest
        .frames
        .iter()
        .map(|f| f.readouts[STOPWATCH].0)
        .collect();
    assert_eq!(stopwatch, [0.0, 0.5]);
    assert_eq!(manifest.frames[1].proper_time.0, 10.5);
    assert_eq!(manifest.labels[0].text, "Test star");
    assert!(manifest.frames[0].readouts["peak_shift"].0.is_infinite());

    let first = reader.read_frame(0).expect("frame 0 reads");
    // The top-left ray looks up at 67.5 degrees and 157.5 degrees round to the left, and in
    // frame 0 the far sky is the observer's sky: (-sqrt 2 / 4, (2 - sqrt 2) / 4, cos 22.5 deg).
    let d = first.direction_at(0, 0);
    let expected = [-0.353_553_39_f32, 0.146_446_61, 0.923_879_5];
    assert!(
        d.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-6),
        "{d:?}"
    );
    let k = |i, j| first.grid().offset(i, j);
    assert_eq!(first.fate[k(3, 3)], fate::PAST_HORIZON);
    assert_eq!(first.winding[k(3, 3)], -1);
    assert_eq!(first.shift[k(3, 3)].to_bits(), 0x7fc0_0005);
    assert_eq!(first.fate[k(0, 3)], fate::UNRESOLVED);
    assert_eq!(first.direction[0][k(0, 3)].to_bits(), 0x7fc0_0001);
    assert_eq!(first.shift[k(5, 1)], 1.0);
    assert!(first.points.is_none());

    let second = reader.read_frame(1).expect("frame 1 reads");
    assert_eq!(second.shift[k(5, 1)], 1.25);
    // Turned 45 degrees about z, one column's width: the ray of column 1 now points where the
    // ray of column 0 looked in frame 0.
    let d = second.direction_at(1, 0);
    assert!(
        d.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-6),
        "{d:?}"
    );
    let points = second
        .points
        .as_ref()
        .expect("frame 1 carries point sources");
    assert_eq!(points.len(), 2);
    assert_eq!(points[0].n, [1.0, 0.0, 0.0]);
    assert_eq!(
        (points[0].shift, points[0].magnification, points[0].label),
        (1.5, -2.0, 0)
    );
    assert_eq!(points[1].label, NO_LABEL);
}

/// Writes `golden/v1`, the bundle the test above reads.
///
/// For version 2, copy this, point it at `golden/v2`, and leave both `golden/v1` and the test
/// above exactly alone.
#[test]
#[ignore = "writes into the source tree; run by hand when a new format version needs a golden"]
fn test_write_the_golden_bundle() {
    let dir = golden_dir();
    let _ = std::fs::remove_dir_all(&dir);
    let mut manifest = sample_manifest(8, 4);
    manifest.writer.program = "sky-format golden generator".into();
    manifest.source.name = Some("golden".into());
    manifest.readouts.push(ReadoutDecl {
        id: "peak_shift".into(),
        label: "Peak shift".into(),
        unit: String::new(),
        decimals: 3,
    });
    manifest.labels.push(Label {
        id: 0,
        text: "Test star".into(),
    });
    manifest.frames_planned = Some(2);
    let mut writer = BundleWriter::create(&dir, manifest).unwrap();

    let first = sample_frame(8, 4, 0, 0.0, 1.0);
    let entry = FrameEntry::new(0, 10.0)
        .with_readout(STOPWATCH, 0.0)
        .with_readout("peak_shift", f64::INFINITY);
    writer.write_frame_and_manifest(&first, entry).unwrap();

    let mut second = sample_frame(8, 4, 1, PI / 4.0, 1.25);
    second.points = Some(vec![
        PointSource {
            n: [1.0, 0.0, 0.0],
            shift: 1.5,
            magnification: -2.0,
            label: 0,
        },
        PointSource {
            n: [0.0, 0.0, 1.0],
            shift: 0.75,
            magnification: 0.5,
            label: NO_LABEL,
        },
    ]);
    let entry = FrameEntry::new(1, 10.5)
        .with_readout(STOPWATCH, 0.5)
        .with_readout("peak_shift", 1.5);
    writer.write_frame_and_manifest(&second, entry).unwrap();
    println!("wrote {}", dir.display());
}

#[test]
fn test_the_example_manifest_in_the_specification_is_a_valid_manifest() {
    // The specification's example is what a person writing their own reader copies, so it has to
    // be a manifest this reader accepts, not an illustration of one.
    let spec = include_str!("../../../physics_simulation_specification.md");
    let start = spec
        .find("```json\n")
        .expect("the specification has a JSON example")
        + 8;
    let length = spec[start..].find("```").expect("the example ends");
    let manifest =
        Manifest::from_json(&spec[start..start + length]).expect("the example is a valid manifest");
    assert_eq!(manifest.grid.grid(), Grid::new(2048, 1024));
    assert_eq!(manifest.far_sky, FarSky::galactic());
}
