//! The reader against the two saves the repository keeps, read in place: the app's golden file
//! (`src/save/golden/v1.json`, plain JSON, never regenerated) and the demo `demos/near_fall.bhl`
//! (gzipped). The expected values below are the numbers those files hold, read off them by eye;
//! the point is that this reader gets the same f64s out, to the bit.

use std::io::Write;
use std::path::PathBuf;

use super::*;

/// A file of the repository, by its path from the repository root.
fn repo_file(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

fn golden_bytes() -> Vec<u8> {
    std::fs::read(repo_file("src/save/golden/v1.json"))
        .expect("the golden save is in the repository")
}

fn gzip(bytes: &[u8]) -> Vec<u8> {
    let mut encoder = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(bytes).unwrap();
    encoder.finish().unwrap()
}

#[test]
fn test_the_golden_save_reads_to_the_numbers_it_holds() {
    let save = read_file(&repo_file("src/save/golden/v1.json")).expect("the golden save opens");
    assert_eq!(
        save.written_by,
        Some(("0.1.0".to_string(), "e40cc99".to_string()))
    );
    assert_eq!(
        save.hole,
        Hole {
            m: 1.0,
            a: 0.9,
            m_solar: 4150000.0
        }
    );
    assert_eq!(save.clock.to_bits(), 3.0000000000000013f64.to_bits());

    let alice = save.alice.as_ref().expect("Alice is in the golden run");
    assert_eq!(alice.name, "Alice");
    assert_eq!(alice.mode, Mode::FreeFall);
    assert_eq!(alice.release, Release::FromInfinity);
    assert!(alice.is_active);
    assert_eq!(alice.trail.len(), 31);
    assert_eq!(alice.r.to_bits(), 3.9174184534836916f64.to_bits());
    let geo = alice.geodesic.expect("Alice carries a geodesic");
    assert_eq!((geo.energy, geo.l_ang, geo.stalled), (1.0, 0.0, false));
    assert_eq!(geo.u[1].to_bits(), (-0.7331354536634381f64).to_bits());
    // An inner trail point, every field of it.
    let p = alice.trail[15];
    assert_eq!(p.t.to_bits(), 1.5000000000000007f64.to_bits());
    assert_eq!(p.r.to_bits(), 4.73313784535207f64.to_bits());
    assert_eq!(p.phi.to_bits(), 6.267288356605786f64.to_bits());
    assert_eq!(p.tau.to_bits(), 1.207674192587378f64.to_bits());
    assert_eq!(
        p.u.map(f64::to_bits),
        [
            1.2578744714313608f64,
            -0.6616877660681618,
            -0.015656671861647528
        ]
        .map(f64::to_bits)
    );
    assert!(!p.stalled);
    // The last trail point is the observer's current event.
    let last = alice.trail.last().unwrap();
    assert_eq!(
        (last.t, last.r, last.phi, last.tau),
        (alice.t, alice.r, alice.phi, alice.tau)
    );

    let bob = save.bob.as_ref().expect("Bob is in the golden run");
    assert_eq!(bob.mode, Mode::FreeFall);
    assert!(!bob.is_active, "Bob is still waiting for his release");
    assert_eq!(bob.release_t, 1000.0);
    assert_eq!(bob.release, Release::FromInfinity);
    assert_eq!(bob.trail.len(), 2);
    assert_eq!(bob.start.tau, 0.0);
    assert_eq!(bob.tau.to_bits(), 2.1213203435596433f64.to_bits());
}

#[test]
fn test_the_demo_save_is_gzipped_and_reads() {
    let bytes = std::fs::read(repo_file("demos/near_fall.bhl")).unwrap();
    assert!(
        bytes.starts_with(&[0x1f, 0x8b]),
        "the demo is a compressed save"
    );
    let save = read_bytes(&bytes).expect("the demo opens");
    assert_eq!(
        save.hole,
        Hole {
            m: 1.0,
            a: 0.9,
            m_solar: 4150000.0
        }
    );
    assert_eq!(save.clock, 0.0);
    for (obs, r, energy) in [
        (
            save.alice.as_ref().unwrap(),
            2.255988524448915,
            0.457897475092809,
        ),
        (
            save.bob.as_ref().unwrap(),
            2.2661110504431003,
            0.46061184699548874,
        ),
    ] {
        assert_eq!(obs.mode, Mode::FreeFall);
        assert_eq!(obs.release, Release::AtRest);
        assert!(obs.is_active);
        assert_eq!(obs.release_t, 0.0);
        assert_eq!(obs.trail.len(), 1);
        assert_eq!(obs.r.to_bits(), f64::to_bits(r));
        assert_eq!(obs.geodesic.unwrap().energy.to_bits(), f64::to_bits(energy));
    }
}

#[test]
fn test_the_same_save_reads_the_same_compressed_or_not() {
    let plain = read_bytes(&golden_bytes()).unwrap();
    let compressed = read_bytes(&gzip(&golden_bytes())).unwrap();
    assert_eq!(plain, compressed);
}

#[test]
fn test_an_unknown_field_is_ignored_at_every_level() {
    // A later build of format 1 may add optional fields anywhere; the app's rule is that that is
    // not a new version, so this reader has to open such a file and read the rest as before.
    let mut doc: serde_json::Value = serde_json::from_slice(&golden_bytes()).unwrap();
    let extra = serde_json::json!({ "a": [1, 2, "three"], "b": null });
    doc["added_at_top"] = extra.clone();
    doc["sim"]["added_to_sim"] = extra.clone();
    doc["sim"]["metric"]["added_to_metric"] = extra.clone();
    doc["sim"]["bob"]["added_to_observer"] = extra.clone();
    doc["sim"]["bob"]["geodesic"]["added_to_geodesic"] = extra.clone();
    doc["sim"]["alice"]["trail"][3]["added_to_trail_point"] = extra.clone();
    doc["sim"]["alice"]["start"]["added_to_start"] = extra;
    let bytes = serde_json::to_vec(&doc).unwrap();
    assert_eq!(
        read_bytes(&bytes).unwrap(),
        read_bytes(&golden_bytes()).unwrap()
    );
}

#[test]
fn test_non_finite_numbers_are_read_from_their_strings() {
    let mut doc: serde_json::Value = serde_json::from_slice(&golden_bytes()).unwrap();
    doc["sim"]["bob"]["release_t"] = "inf".into();
    doc["sim"]["bob"]["beta_r"] = "nan".into();
    doc["sim"]["bob"]["phi"] = "-inf".into();
    doc["sim"]["bob"]["geodesic"]["l_ang"] = "nan".into();
    let save = read_bytes(&serde_json::to_vec(&doc).unwrap()).unwrap();
    let bob = save.bob.unwrap();
    assert_eq!(bob.release_t, f64::INFINITY);
    assert_eq!(bob.phi, f64::NEG_INFINITY);
    assert!(bob.geodesic.unwrap().l_ang.is_nan());
}

#[test]
fn test_a_file_that_is_not_a_save_of_this_version_is_refused_with_a_sentence() {
    let mut doc: serde_json::Value = serde_json::from_slice(&golden_bytes()).unwrap();
    doc["format"] = "something-else".into();
    let err = read_bytes(&serde_json::to_vec(&doc).unwrap()).unwrap_err();
    assert!(matches!(err, ReadError::NotASave { .. }), "{err}");
    assert!(
        err.to_string().contains("not a Black Hole Lab save"),
        "{err}"
    );

    doc["format"] = FORMAT.into();
    doc["version"] = 2.into();
    // A version-2 file may have any structure at all; it must be refused on its version, before
    // anything else in it is looked at.
    doc["sim"] = "restructured".into();
    let err = read_bytes(&serde_json::to_vec(&doc).unwrap()).unwrap_err();
    assert!(
        matches!(err, ReadError::UnknownVersion { version: 2 }),
        "{err}"
    );
    assert!(err.to_string().contains("version 2"), "{err}");

    let err = read_bytes(b"[1, 2, 3]").unwrap_err();
    assert!(matches!(err, ReadError::Json(_)), "{err}");
    let err = read_bytes(b"{\"hello\": 1}").unwrap_err();
    assert!(matches!(err, ReadError::NotASave { .. }), "{err}");

    let compressed = gzip(&golden_bytes());
    let err = read_bytes(&compressed[..compressed.len() / 2]).unwrap_err();
    assert!(matches!(err, ReadError::Gzip(_)), "{err}");

    let err = read_file(&repo_file("no/such/save.bhl")).unwrap_err();
    assert!(matches!(err, ReadError::Io(_)), "{err}");
}

#[test]
fn test_the_time_unit_is_the_solar_mass_times_the_accepted_constant() {
    let hole = Hole {
        m: 1.0,
        a: 0.9,
        m_solar: 4150000.0,
    };
    assert!((hole.seconds_per_unit() - 20.44078743).abs() < 1e-8);
    // A chart with M = 2 has a unit of M / 2: half as long.
    let doubled = Hole { m: 2.0, ..hole };
    assert_eq!(doubled.seconds_per_unit(), 0.5 * hole.seconds_per_unit());
}
