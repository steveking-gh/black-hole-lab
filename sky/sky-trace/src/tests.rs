//! The command line and the dry run's report.

use std::path::PathBuf;

use super::*;
use args::parse;

fn strings(args: &[&str]) -> Vec<String> {
    args.iter().map(|s| s.to_string()).collect()
}

fn repo_file(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

#[test]
fn test_the_command_line_takes_the_four_options_and_their_defaults() {
    assert_eq!(parse(&strings(&["--help"])), Ok(Command::Help));
    assert_eq!(
        parse(&strings(&["--info", "x.bhl", "-h"])),
        Ok(Command::Help)
    );
    let Ok(Command::Info(o)) = parse(&strings(&["--info", "x.bhl"])) else {
        panic!()
    };
    assert_eq!(
        (o.who, o.step, o.frames, o.frames_given),
        (
            Who::Bob,
            args::DEFAULT_STEP,
            args::DEFAULT_MAX_FRAMES,
            false
        )
    );
    let Ok(Command::Info(o)) = parse(&strings(&[
        "--observer",
        "alice",
        "--frames",
        "12",
        "--step",
        "0.25",
        "--info",
        "y.bhl",
    ])) else {
        panic!()
    };
    assert_eq!(
        (o.who, o.step, o.frames, o.frames_given),
        (Who::Alice, 0.25, 12, true)
    );
    assert_eq!(o.save, PathBuf::from("y.bhl"));
}

#[test]
fn test_a_wrong_command_line_is_refused_with_a_sentence() {
    for (args, says) in [
        (vec![], "dry run"),
        (vec!["--info"], "needs a value"),
        (vec!["--info", "a", "--info", "b"], "given twice"),
        (vec!["--info", "a", "--observer", "carol"], "no observer"),
        (vec!["--info", "a", "--step", "0"], "greater than zero"),
        (vec!["--info", "a", "--step", "nan"], "greater than zero"),
        (vec!["--info", "a", "--frames", "0"], "at least 1"),
        (vec!["--info", "a", "--frames", "2.5"], "whole number"),
        (vec!["--info", "a", "--out", "b"], "no option"),
    ] {
        let err = parse(&strings(&args)).unwrap_err();
        assert!(err.contains(says), "{args:?}: {err}");
    }
}

#[test]
fn test_the_dry_run_reports_the_film_of_each_repository_save() {
    for (file, who, reason) in [
        ("demos/near_fall.bhl", Who::Bob, "inner horizon"),
        ("demos/near_fall.bhl", Who::Alice, "inner horizon"),
        ("src/save/golden/v1.json", Who::Alice, "inner horizon"),
        (
            "src/save/golden/v1.json",
            Who::Bob,
            "between frames 7049 and 7050",
        ),
    ] {
        let options = Options {
            save: repo_file(file),
            who,
            step: args::DEFAULT_STEP,
            frames: args::DEFAULT_MAX_FRAMES,
            frames_given: false,
        };
        let text = info(&options).unwrap();
        assert!(text.contains("20.440787 s"), "{text}");
        assert!(text.contains(reason), "{file} {who:?}: {text}");
        assert!(text.contains("nothing has been traced"), "{text}");
    }
    let missing = Options {
        save: repo_file("no/such.bhl"),
        who: Who::Bob,
        step: 0.1,
        frames: 1,
        frames_given: true,
    };
    assert!(info(&missing).unwrap_err().contains("could not be read"));
}
