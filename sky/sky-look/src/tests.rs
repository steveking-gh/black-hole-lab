//! The pure parts: the search for the pieces, the name of a view and its folder, the reading of the
//! save and of the children's lines, the commands as PowerShell lines, the choice of viewer, and
//! the commands that start the viewer and the prompt. Each against files made for the test in a scratch
//! directory of its own, and none needing the real tools or starting a viewer.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, UNIX_EPOCH};

use crate::args::{self, Request, Units, Who};
use crate::find::{self, FETCH_SCRIPT, Names, SKY_MAP, SKY_MAP_ENV, SKY_MAP_URL, Search};
use crate::names;
use crate::open::{self, Viewer};
use crate::run::{Tally, parse_tally, percent_sentence, powershell_line, red_sentence};
use crate::save;
use crate::sha256;
use crate::shell;

/// A directory of its own for one test, empty, in the system's temporary directory.
pub fn scratch_dir(what: &str) -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let dir = std::env::temp_dir().join(format!(
        "sky-look-test-{}-{}-{what}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("a scratch directory");
    dir
}

fn touch(path: &Path) {
    std::fs::create_dir_all(path.parent().expect("a parent")).expect("the parent directory");
    std::fs::write(path, b"").expect("the file");
}

fn names() -> Names {
    Names {
        trace: "sky-trace.exe".into(),
        render: "sky-render.exe".into(),
        ffmpeg: "ffmpeg.exe".into(),
    }
}

/// A repository with a release build of the sky tools, the star map and the script that fetches
/// it in `sky/maps`, and ffmpeg in a directory of its own named by the returned PATH value.
fn layout(root: &Path) -> (PathBuf, OsString) {
    let release = root.join("repo/sky/target/release");
    touch(&release.join("sky-trace.exe"));
    touch(&release.join("sky-render.exe"));
    touch(&root.join("repo/sky").join(SKY_MAP));
    touch(&root.join("repo/sky").join(FETCH_SCRIPT));
    let bin = root.join("ffmpeg/bin");
    touch(&bin.join("ffmpeg.exe"));
    let path = std::env::join_paths([root.join("empty"), bin]).expect("a PATH");
    (release, path)
}

#[test]
fn test_every_piece_is_found_where_a_release_build_of_the_sky_tools_puts_it() {
    let root = scratch_dir("found");
    let (release, path) = layout(&root);
    let search = Search {
        exe_dir: Some(release.clone()),
        path: Some(path),
        ..Search::default()
    };
    let pieces = find::find(&search, &names()).expect("everything is there");
    assert_eq!(pieces.trace, release.join("sky-trace.exe"));
    assert_eq!(pieces.render, release.join("sky-render.exe"));
    assert_eq!(pieces.ffmpeg, root.join("ffmpeg/bin/ffmpeg.exe"));
    assert_eq!(pieces.sky, root.join("repo/sky").join(SKY_MAP));

    // A program beside the app, in the repository's own target/release, finds the map in the
    // `sky` directory beside its ancestor.
    let beside_app = root.join("repo/target/release");
    touch(&beside_app.join("sky-trace.exe"));
    touch(&beside_app.join("sky-render.exe"));
    let search = Search {
        exe_dir: Some(beside_app),
        ..search
    };
    assert_eq!(
        find::find(&search, &names()).expect("found").sky,
        root.join("repo/sky").join(SKY_MAP)
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_a_piece_named_outright_comes_before_the_search_and_a_wrong_name_is_reported() {
    let root = scratch_dir("named");
    let (release, path) = layout(&root);
    let base = Search {
        exe_dir: Some(release),
        path: Some(path),
        ..Search::default()
    };

    // --tools, --ffmpeg and --sky each win over the search.
    let tools = root.join("other-tools");
    touch(&tools.join("sky-trace.exe"));
    touch(&tools.join("sky-render.exe"));
    let ffmpeg = root.join("elsewhere/ffmpeg.exe");
    touch(&ffmpeg);
    let map = root.join("maps/mine.exr");
    touch(&map);
    let env_map = root.join("maps/from-env.exr");
    touch(&env_map);
    let named = Search {
        tools: Some(tools.clone()),
        ffmpeg: Some(ffmpeg.clone()),
        sky: Some(map.clone()),
        sky_env: Some(env_map.clone().into()),
        ..base.clone()
    };
    let pieces = find::find(&named, &names()).expect("everything is named");
    assert_eq!(
        (pieces.trace, pieces.ffmpeg, pieces.sky),
        (tools.join("sky-trace.exe"), ffmpeg, map)
    );
    // The variable comes before the search, and a variable set to nothing is a variable not set.
    let from_env = Search {
        sky_env: Some(env_map.clone().into()),
        ..base.clone()
    };
    assert_eq!(find::find(&from_env, &names()).expect("found").sky, env_map);
    let empty_env = Search {
        sky_env: Some(OsString::new()),
        ..base.clone()
    };
    assert_eq!(
        find::find(&empty_env, &names()).expect("found").sky,
        root.join("repo/sky").join(SKY_MAP)
    );

    // A name that points at nothing is reported as such, not passed over for the search.
    let nowhere = root.join("nowhere/x.exe");
    let cases: [(&str, Search, &[&str]); 4] = [
        (
            "--ffmpeg",
            Search {
                ffmpeg: Some(nowhere.clone()),
                ..base.clone()
            },
            &["--ffmpeg names", "no program there"],
        ),
        (
            "--sky",
            Search {
                sky: Some(nowhere.clone()),
                ..base.clone()
            },
            &["--sky names", "no star map there"],
        ),
        (
            "the variable",
            Search {
                sky_env: Some(nowhere.clone().into()),
                ..base.clone()
            },
            &[SKY_MAP_ENV, "no star map there"],
        ),
        (
            "--tools",
            Search {
                tools: Some(root.join("nowhere")),
                ..base.clone()
            },
            &[
                "sky-trace.exe was not found",
                "--tools <dir>",
                "cargo build --release",
            ],
        ),
    ];
    for (what, search, words) in cases {
        let why = find::find(&search, &names()).expect_err(what);
        for word in words {
            assert!(why.contains(word), "{what}: {why}");
        }
    }
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_each_missing_piece_has_its_own_sentence_saying_what_to_do() {
    let root = scratch_dir("missing");
    let (release, path) = layout(&root);
    let search = Search {
        exe_dir: Some(release.clone()),
        path: Some(path),
        ..Search::default()
    };
    let map = root.join("repo/sky").join(SKY_MAP);
    let missing: [(PathBuf, &[&str]); 4] = [
        (
            release.join("sky-trace.exe"),
            &["sky-trace.exe was not found", "cargo build --release"],
        ),
        (
            release.join("sky-render.exe"),
            &["sky-render.exe was not found", "cargo build --release"],
        ),
        (
            root.join("ffmpeg/bin/ffmpeg.exe"),
            &["ffmpeg.exe was not found on the PATH", "--ffmpeg <path>"],
        ),
        (
            map.clone(),
            &["star map", "fetch-sky.ps1", "--sky <map.exr>", SKY_MAP_ENV],
        ),
    ];
    let mut sentences = Vec::new();
    for (piece, words) in missing {
        let mut hidden = piece.clone().into_os_string();
        hidden.push(".away");
        std::fs::rename(&piece, &hidden).expect("the piece is moved away");
        let why = find::find(&search, &names()).expect_err("a piece is missing");
        std::fs::rename(&hidden, &piece).expect("the piece is put back");
        for word in words {
            assert!(why.contains(word), "{}: {why}", piece.display());
        }
        assert_eq!(why.lines().count(), 1, "one sentence: {why}");
        assert!(why.ends_with('.'), "a complete sentence: {why}");
        sentences.push(why);
    }
    sentences.dedup();
    assert_eq!(
        sentences.len(),
        4,
        "each piece its own sentence: {sentences:#?}"
    );
    // The script is named where it is, in the sky directory that was found.
    assert!(
        sentences[3].contains(
            &root
                .join("repo/sky")
                .join("maps")
                .join("fetch-sky.ps1")
                .display()
                .to_string()
        )
    );

    // A downloaded copy, with no sky directory anywhere above: the map is looked for beside the
    // program, and the sentence gives the address to download it from and the folder to put it
    // in, and names no script, since none is there.
    let copy = root.join("downloaded/Black Hole Lab");
    touch(&copy.join("sky-trace.exe"));
    touch(&copy.join("sky-render.exe"));
    let installed = Search {
        exe_dir: Some(copy.clone()),
        ..search
    };
    let why = find::find(&installed, &names()).expect_err("no map anywhere");
    let maps = copy.join("maps").display().to_string();
    assert!(
        why.contains(SKY_MAP_URL) && why.contains(&maps) && !why.contains("fetch-sky.ps1"),
        "{why}"
    );
    // With the script extracted beside the map's place, the sentence names it where it is.
    touch(&copy.join(FETCH_SCRIPT));
    let why = find::find(&installed, &names()).expect_err("no map anywhere");
    assert!(
        why.contains(
            &copy
                .join("maps")
                .join("fetch-sky.ps1")
                .display()
                .to_string()
        ) && why.contains(SKY_MAP_URL),
        "{why}"
    );
    // The map, once it is there, is found.
    touch(&copy.join(SKY_MAP));
    assert_eq!(
        find::find(&installed, &names()).expect("found").sky,
        copy.join(SKY_MAP)
    );
    // A tool missing from a downloaded copy is a file missing from an archive, not a program to
    // build.
    std::fs::remove_file(copy.join("sky-render.exe")).expect("the renderer is taken away");
    let why = find::find(&installed, &names()).expect_err("no renderer");
    assert!(
        why.contains("sky-render.exe was not found")
            && why.contains("incomplete")
            && !why.contains("cargo"),
        "{why}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_a_view_is_named_by_its_observer_and_watch_reading_and_its_log_by_the_time() {
    assert_eq!(names::stem("Bob", Some(0.0), false), "Bob_0.000");
    assert_eq!(names::stem("Alice", Some(12.3456), false), "Alice_12.346");
    assert_eq!(
        names::stem("Alice", Some(12.3456), true),
        "Alice_12,346",
        "the decimal mark follows the app's setting"
    );
    assert_eq!(names::stem("Bob", Some(-0.0004), false), "Bob_-0.000");
    assert_eq!(
        names::stem("Bob", None, false),
        "Bob",
        "no reading, no underscore"
    );
    assert_eq!(names::stem("Bob", Some(f64::NAN), false), "Bob");

    // The time the log records, across the boundaries a careless format gets wrong: a digit count
    // changing, a month and a year turning, a leap day, the millisecond.
    let at = |s: u64, ms: u64| UNIX_EPOCH + Duration::from_millis(s * 1000 + ms);
    assert_eq!(names::utc_stamp(at(0, 0)), "1970-01-01 00:00:00.000 UTC");
    assert_eq!(
        names::utc_stamp(at(951_782_400, 0)),
        "2000-02-29 00:00:00.000 UTC"
    );
    assert_eq!(
        names::utc_stamp(at(1_798_761_599, 999)),
        "2026-12-31 23:59:59.999 UTC"
    );
    assert_eq!(
        names::utc_stamp(at(1_799_539_200, 5)),
        "2027-01-10 00:00:00.005 UTC"
    );
}

#[test]
fn test_two_views_from_one_reading_make_two_folders_and_neither_uses_the_others() {
    let root = scratch_dir("folders");
    let out = root.join("views");
    std::fs::create_dir_all(&out).expect("the views directory");
    let stem = names::stem("Bob", Some(0.0), false);
    // An earlier view's photograph of the same name, from before views had folders: a folder of
    // that name is another thing, and making it touches nothing else.
    let old_photo = out.join(format!("{stem}.jpg"));
    std::fs::write(&old_photo, "an earlier photograph").expect("an earlier view");
    let folders: Vec<PathBuf> = (0..3)
        .map(|_| names::make_folder(&out, &stem).expect("a folder"))
        .collect();
    assert_eq!(
        folders,
        [
            out.join("Bob_0.000"),
            out.join("Bob_0.000 (2)"),
            out.join("Bob_0.000 (3)")
        ]
    );
    assert!(folders.iter().all(|f| f.is_dir()));
    assert_eq!(
        std::fs::read_to_string(&old_photo).expect("the earlier view"),
        "an earlier photograph"
    );

    // A file moved into a folder arrives whole, and is no longer where it was; a name that is
    // taken is refused rather than written over.
    let save = root.join("the run.bhl");
    std::fs::write(&save, "a save").expect("a save");
    let moved = folders[0].join("view.bhl");
    names::move_file(&save, &moved).expect("moved");
    assert!(!save.exists(), "the save is no longer where it was");
    assert_eq!(std::fs::read_to_string(&moved).expect("moved"), "a save");
    std::fs::write(&save, "another save").expect("another save");
    let taken = names::move_file(&save, &moved).expect_err("the name is taken");
    assert_eq!(taken.kind(), std::io::ErrorKind::AlreadyExists);
    assert_eq!(std::fs::read_to_string(&moved).expect("kept"), "a save");
    assert!(
        save.is_file(),
        "a file that could not be moved stays where it was"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_any_observer_name_makes_a_file_name_windows_accepts() {
    let names_given = [
        "Bob",
        "Alice",
        "a<b>c:d\"e/f\\g|h?i*j",
        "trailing dots...",
        "trailing space ",
        "   ",
        "",
        "CON",
        "%PATH%",
        "tab\there\nnewline\u{7}",
        "Ångström τ 黒",
        &"x".repeat(300),
    ];
    let root = scratch_dir("safe");
    for given in names_given {
        let stem = names::stem(given, Some(1.5), false);
        assert!(
            !stem
                .chars()
                .any(|c| c.is_control() || "<>:\"/\\|?*%".contains(c)),
            "{given:?} made {stem:?}"
        );
        assert!(!stem.ends_with(['.', ' ']), "{given:?} made {stem:?}");
        assert!(
            stem.len() < 120,
            "{given:?} made a name {} bytes long",
            stem.len()
        );
        // And the system agrees: the file can be made, and is found again under that name.
        let file = root.join(format!("{stem}.jpg"));
        std::fs::write(&file, b"").unwrap_or_else(|e| panic!("{given:?} made {stem:?}: {e}"));
        assert!(
            std::fs::read_dir(&root)
                .expect("the directory")
                .any(|e| e.expect("an entry").file_name() == file.file_name().expect("a name")),
            "{stem:?} was stored under another name"
        );
        std::fs::remove_file(&file).expect("removed");
    }
    assert_eq!(names::safe_name("  "), "observer");
    assert_eq!(names::safe_name("a:b"), "a_b");
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_the_views_go_to_the_flag_then_the_variable_then_videos_then_home() {
    let root = scratch_dir("outdir");
    let (flag, env, videos, home) = (
        root.join("flag"),
        root.join("env"),
        root.join("Videos"),
        root.join("home"),
    );
    std::fs::create_dir_all(&videos).expect("a Videos folder");
    let pick = |f: Option<&Path>, e: Option<&Path>, v: Option<&Path>, h: Option<&Path>| {
        names::out_dir(f, e.map(Path::as_os_str), v, h)
    };
    let all = pick(Some(&flag), Some(&env), Some(&videos), Some(&home));
    assert_eq!(all, Ok(flag.clone()));
    assert_eq!(
        pick(None, Some(&env), Some(&videos), Some(&home)),
        Ok(env.clone())
    );
    assert_eq!(
        pick(None, Some(Path::new("")), Some(&videos), Some(&home)),
        Ok(videos.join(names::VIEWS_DIR))
    );
    assert_eq!(
        pick(None, None, Some(&videos), Some(&home)),
        Ok(videos.join(names::VIEWS_DIR))
    );
    // A Videos folder the system names but that is not there is no Videos folder.
    let gone = root.join("gone");
    assert_eq!(
        pick(None, None, Some(&gone), Some(&home)),
        Ok(home.join(names::VIEWS_DIR))
    );
    let why = pick(None, None, None, None).expect_err("nowhere to go");
    assert!(
        why.contains("--out-dir") && why.contains(names::VIEWS_ENV),
        "{why}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_the_save_gives_the_observers_name_watch_and_decimal_mark_gzipped_or_not() {
    let root = scratch_dir("save");
    let doc = r#"{"format":"black-hole-lab-save","sim":{"bob":{"name":"Bob","tau":1.25},
        "alice":{"name":"Alice","tau":0.5}},"controls":{"decimal_is_comma":true}}"#;
    let plain = root.join("plain.bhl");
    std::fs::write(&plain, doc).expect("a plain save");
    let packed = root.join("packed.bhl");
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
    std::io::Write::write_all(&mut gz, doc.as_bytes()).expect("compressed");
    std::fs::write(&packed, gz.finish().expect("finished")).expect("a compressed save");
    for path in [&plain, &packed] {
        let bob = save::facts(path, Who::Bob);
        assert_eq!(
            bob,
            save::Facts {
                name: Some("Bob".into()),
                tau: Some(1.25),
                comma: true
            }
        );
        assert_eq!(save::facts(path, Who::Alice).tau, Some(0.5));
    }
    // A save older than the decimal setting reads as point style, as the app reads it; a file
    // that is not a save gives nothing, and is left to the tracer to refuse.
    std::fs::write(&plain, r#"{"sim":{"bob":{"tau":2.0}}}"#).expect("an older save");
    assert_eq!(
        save::facts(&plain, Who::Bob),
        save::Facts {
            name: None,
            tau: Some(2.0),
            comma: false
        }
    );
    std::fs::write(&plain, "not a save").expect("junk");
    assert_eq!(save::facts(&plain, Who::Bob), save::Facts::default());
    assert_eq!(
        save::facts(&root.join("absent.bhl"), Who::Bob),
        save::Facts::default()
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_the_renderers_counts_become_one_sentence_explaining_the_red() {
    let line = "pixels drawn over the 1 frame(s): unresolved 0 (0 %), under-sampled 16888 \
                (0.0503 %), dark 20422816 (60.9 %)";
    let tally = parse_tally(line).expect("the counts");
    assert_eq!(
        tally,
        Tally {
            unresolved: 0,
            undersampled: 16888
        }
    );
    let sentence = red_sentence(tally, false).expect("there is red");
    assert_eq!(
        sentence,
        "16888 pixels (0.0503 % of the picture) could not be determined and are drawn in red \
         along the edge of the dark region, where the traced rays are too far apart to say which \
         part of the sky the light came from."
    );
    assert!(
        red_sentence(tally, true)
            .expect("red")
            .contains("(0,0503 %")
    );
    let both = red_sentence(
        Tally {
            unresolved: 10,
            undersampled: 20,
        },
        false,
    )
    .expect("red");
    assert!(
        both.starts_with("30 pixels")
            && both.contains("20 along the edge")
            && both.contains("10 where the tracer"),
        "{both}"
    );
    let lost = red_sentence(
        Tally {
            unresolved: 7,
            undersampled: 0,
        },
        false,
    )
    .expect("red");
    assert!(
        !lost.contains("edge") && lost.contains("could not follow"),
        "{lost}"
    );
    assert_eq!(
        red_sentence(
            Tally {
                unresolved: 0,
                undersampled: 0
            },
            false
        ),
        None,
        "no red, nothing to say"
    );
    assert_eq!(parse_tally("read-outs: 3 line(s)"), None);
}

#[test]
fn test_the_command_line_needs_a_save_and_an_observer_and_refuses_what_the_tools_would() {
    let parse = |line: &str| {
        args::parse(
            &line
                .split_whitespace()
                .map(OsString::from)
                .collect::<Vec<_>>(),
        )
    };
    let Ok(Request::Look(o)) = parse("x.bhl --observer alice --open") else {
        panic!("a look")
    };
    assert_eq!(
        (o.who, o.open, o.grid, o.viewer.as_deref()),
        (Who::Alice, true, args::DEFAULT_GRID, None)
    );
    assert_eq!(
        (o.status.as_deref(), o.move_save, o.shell),
        (None, false, false),
        "nothing of the app's own unless it is asked for"
    );
    // What the app gives.
    let Ok(Request::Look(o)) =
        parse(r"x.bhl --observer bob --status C:\temp\x.status --move-save --open --shell")
    else {
        panic!("the app's command line")
    };
    assert_eq!(
        (o.status, o.move_save, o.open, o.shell),
        (Some(PathBuf::from(r"C:\temp\x.status")), true, true, true)
    );
    let Ok(Request::Look(o)) = parse(r"x.bhl --observer bob --open --viewer C:\tools\viewer.exe")
    else {
        panic!("a look with a viewer")
    };
    assert_eq!(o.viewer, Some(PathBuf::from(r"C:\tools\viewer.exe")));
    assert_eq!(parse("--help"), Ok(Request::Help));
    // The read-outs are in seconds and kilometres unless the command line asks for M.
    for (line, units) in [
        ("x.bhl --observer bob", Units::Physical),
        ("x.bhl --observer bob --units physical", Units::Physical),
        ("x.bhl --units geometric --observer bob", Units::Geometric),
    ] {
        let Ok(Request::Look(o)) = parse(line) else {
            panic!("a look: {line}")
        };
        assert_eq!(o.units, units, "{line}");
    }
    assert_eq!(
        (Units::Physical.flag(), Units::Geometric.flag()),
        ("physical", "geometric"),
        "the words sky-trace --units takes"
    );
    for (line, words) in [
        ("--observer bob", "No save"),
        ("x.bhl", "--observer is required"),
        ("x.bhl --observer carol", "no observer"),
        ("x.bhl --observer bob --grid 100x100", "twice as wide"),
        ("x.bhl --observer bob --grid 32768x16384", "more rays"),
        ("x.bhl --observer bob --exposure nan", "--exposure"),
        (
            "x.bhl --observer bob --units si",
            "There are no units \"si\"; --units is physical",
        ),
        ("x.bhl --observer bob --units", "--units needs a value"),
        (
            "x.bhl --observer bob --units physical --units geometric",
            "--units is given twice",
        ),
        ("x.bhl --observer bob --fast", "no option --fast"),
        // Every view keeps its bundle now, so the option that asked for that is gone.
        ("x.bhl --observer bob --keep", "There is no option --keep;"),
        ("x.bhl --observer bob --status", "--status needs a value"),
        // The held video is gone, and its option with it: refused as any unknown option is.
        (
            "x.bhl --observer bob --hold 60",
            "There is no option --hold;",
        ),
        ("x.bhl --observer bob --viewer", "--viewer needs a value"),
        (
            "x.bhl --observer bob --viewer a --viewer b",
            "--viewer is given twice",
        ),
        ("x.bhl y.bhl --observer bob", "Two saves"),
    ] {
        let why = parse(line).expect_err(line);
        assert!(why.contains(words) && why.ends_with('.'), "{line}: {why}");
    }
}

/// A layout for the choice of viewer: a named program, one the variable names, and VLC where its
/// installer and the two Program Files directories would put it.
#[cfg(windows)]
struct Viewers {
    root: PathBuf,
    flag: PathBuf,
    env: PathBuf,
    registered: PathBuf,
    program_files: PathBuf,
    program_files_x86: PathBuf,
}

#[cfg(windows)]
impl Viewers {
    fn new(what: &str) -> Self {
        let root = scratch_dir(what);
        Self {
            flag: root.join("tools/viewer.exe"),
            env: root.join("from-env/other.exe"),
            registered: root.join("Custom/VLC"),
            program_files: root.join("Program Files"),
            program_files_x86: root.join("Program Files (x86)"),
            root,
        }
    }

    fn search(&self, flag: bool, env: Option<&Path>) -> open::ViewerSearch {
        open::ViewerSearch {
            flag: flag.then(|| self.flag.clone()),
            env: env.map(|e| e.as_os_str().to_owned()),
            vlc: open::vlc_places(
                std::slice::from_ref(&self.registered),
                &[self.program_files.clone(), self.program_files_x86.clone()],
                None,
            ),
        }
    }
}

#[cfg(windows)]
impl Drop for Viewers {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}

#[cfg(windows)]
#[test]
fn test_the_viewer_is_the_one_named_then_vlc_where_it_installs_then_the_systems_default() {
    let v = Viewers::new("viewer");
    let registered_vlc = v.registered.join("vlc.exe");
    let x86_vlc = v.program_files_x86.join(r"VideoLAN\VLC\vlc.exe");
    let x64_vlc = v.program_files.join(r"VideoLAN\VLC\vlc.exe");
    let choose = |flag: bool, env: Option<&Path>| open::choose(&v.search(flag, env));

    // Nothing named and no VLC: the system's default.
    assert_eq!(choose(false, None), Ok(Viewer::Default));
    // VLC in the 32-bit Program Files, then in the 64-bit one, which comes first, then where the
    // registry says, which comes before both.
    touch(&x86_vlc);
    assert_eq!(choose(false, None), Ok(Viewer::Vlc(x86_vlc.clone())));
    touch(&x64_vlc);
    assert_eq!(choose(false, None), Ok(Viewer::Vlc(x64_vlc.clone())));
    touch(&registered_vlc);
    assert_eq!(choose(false, None), Ok(Viewer::Vlc(registered_vlc.clone())));
    // A directory named like the program is not the program.
    let decoy = Viewers::new("decoy");
    std::fs::create_dir_all(decoy.program_files.join(r"VideoLAN\VLC\vlc.exe")).expect("a decoy");
    assert_eq!(
        open::choose(&decoy.search(false, None)),
        Ok(Viewer::Default)
    );

    // The variable comes before VLC, and a variable set to nothing is a variable not set.
    touch(&v.env);
    assert_eq!(
        choose(false, Some(&v.env)),
        Ok(Viewer::Named {
            program: v.env.clone(),
            by: open::VIEWER_ENV
        })
    );
    assert_eq!(
        choose(false, Some(Path::new(""))),
        Ok(Viewer::Vlc(registered_vlc.clone()))
    );
    // And --viewer comes before the variable.
    touch(&v.flag);
    assert_eq!(
        choose(true, Some(&v.env)),
        Ok(Viewer::Named {
            program: v.flag.clone(),
            by: "--viewer"
        })
    );

    // A named program that is not there is reported, and not passed over for VLC, which is there.
    let nowhere = v.root.join("nowhere/viewer.exe");
    let why = open::choose(&open::ViewerSearch {
        flag: Some(nowhere.clone()),
        ..v.search(false, Some(&v.env))
    })
    .expect_err("--viewer names nothing");
    assert!(
        why.starts_with("--viewer names")
            && why.contains(&nowhere.display().to_string())
            && why.contains("no program there")
            && why.contains("full path"),
        "{why}"
    );
    let why = choose(false, Some(&nowhere)).expect_err("the variable names nothing");
    assert!(
        why.starts_with(open::VIEWER_ENV) && why.contains("no program there"),
        "{why}"
    );
}

#[cfg(windows)]
#[test]
fn test_vlc_is_looked_for_where_the_registry_says_then_in_each_program_files_directory() {
    let places = open::vlc_places(
        &[PathBuf::from(r"D:\Apps\VLC")],
        &[
            PathBuf::from(r"C:\Program Files"),
            PathBuf::from(r"C:\Program Files (x86)"),
        ],
        Some(std::ffi::OsStr::new(r"C:\bin")),
    );
    assert_eq!(
        places,
        [
            PathBuf::from(r"D:\Apps\VLC\vlc.exe"),
            PathBuf::from(r"C:\Program Files\VideoLAN\VLC\vlc.exe"),
            PathBuf::from(r"C:\Program Files (x86)\VideoLAN\VLC\vlc.exe"),
        ],
        "and not on the PATH, where Windows' VLC does not put itself"
    );
    assert_eq!(open::vlc_places(&[], &[], None), Vec::<PathBuf>::new());
}

/// A photograph's path with every character the Windows shell reads as syntax in it.
const AWKWARD: &str = r"C:\a b\100% & more ^ (x)!.jpg";

#[test]
fn test_vlc_is_started_directly_on_the_photograph_with_one_option_to_keep_the_picture_up() {
    let photo = Path::new(AWKWARD);
    let vlc = PathBuf::from(r"C:\Program Files\VideoLAN\VLC\vlc.exe");
    let args_of = |command: &std::process::Command| -> Vec<std::ffi::OsString> {
        command.get_args().map(|a| a.to_owned()).collect()
    };
    let command = open::command(&Viewer::Vlc(vlc.clone()), photo);
    assert_eq!(
        command.get_program(),
        vlc.as_os_str(),
        "no shell in between"
    );
    assert_eq!(
        args_of(&command),
        [
            std::ffi::OsString::from("--image-duration=-1"),
            photo.as_os_str().to_owned()
        ],
        "the option, then the path as one argument, whole"
    );
    assert_eq!(command.get_envs().count(), 0);

    // A viewer named outright is given the path and nothing else, unless its file is VLC's.
    let other = PathBuf::from(r"C:\tools\pano viewer.exe");
    let named = open::command(
        &Viewer::Named {
            program: other.clone(),
            by: "--viewer",
        },
        photo,
    );
    assert_eq!(named.get_program(), other.as_os_str());
    assert_eq!(args_of(&named), [photo.as_os_str().to_owned()]);
    let named_vlc = open::command(
        &Viewer::Named {
            program: PathBuf::from(r"E:\portable\VLC.EXE"),
            by: open::VIEWER_ENV,
        },
        photo,
    );
    assert_eq!(
        args_of(&named_vlc),
        [
            std::ffi::OsString::from("--image-duration=-1"),
            photo.as_os_str().to_owned()
        ]
    );
}

#[test]
fn test_the_sentence_after_opening_names_the_program_and_the_default_warns_of_a_flat_picture() {
    let vlc = PathBuf::from(r"C:\Program Files\VideoLAN\VLC\vlc.exe");
    let said = open::opened_sentence(&Viewer::Vlc(vlc.clone()));
    assert_eq!(
        said,
        format!(
            "Opened the photograph in VLC ({}); drag with the mouse to look in every direction.",
            vlc.display()
        )
    );
    let named = open::opened_sentence(&Viewer::Named {
        program: PathBuf::from(r"C:\tools\viewer.exe"),
        by: "--viewer",
    });
    assert_eq!(
        named,
        r"Opened the photograph in C:\tools\viewer.exe, the viewer --viewer names."
    );
    let default = open::opened_sentence(&Viewer::Default);
    assert!(
        default.contains("the program the system opens .jpg files with")
            && default.contains("may show the photograph flat")
            && default.contains("VLC pans a 360-degree photograph"),
        "{default}"
    );
    for sentence in [said, named, default] {
        assert!(
            sentence.starts_with("Opened") && sentence.ends_with('.'),
            "{sentence}"
        );
        assert_eq!(sentence.lines().count(), 1);
    }
}

#[cfg(windows)]
#[test]
fn test_the_systems_default_is_reached_by_the_shells_start_with_an_empty_title_and_the_path_in_a_variable()
 {
    let photo = Path::new(AWKWARD);
    let command = open::command(&Viewer::Default, photo);
    let args: Vec<_> = command
        .get_args()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();
    assert_eq!(
        args.join(" "),
        r#"/D /V:OFF /C start "" "%BLACK_HOLE_LAB_VIEW%""#
    );
    let envs: Vec<_> = command.get_envs().collect();
    assert_eq!(
        envs,
        [(
            std::ffi::OsStr::new("BLACK_HOLE_LAB_VIEW"),
            Some(photo.as_os_str())
        )]
    );

    // What `cmd` makes of that line, with `start` swapped for `echo`: the path, whole and literal,
    // inside its quotes, however many characters in it are the shell's syntax.
    use std::os::windows::process::CommandExt;
    let echoed = std::process::Command::new("cmd")
        .env("BLACK_HOLE_LAB_VIEW", photo)
        .raw_arg(r#"/D /V:OFF /C echo "" "%BLACK_HOLE_LAB_VIEW%""#)
        .output()
        .expect("cmd runs");
    let said = String::from_utf8_lossy(&echoed.stdout);
    assert_eq!(said.trim_end(), format!(r#""" "{}""#, photo.display()));
}

#[cfg(windows)]
#[test]
fn test_the_registry_answer_for_vlc_is_a_directory_or_nothing() {
    // Whatever this machine has installed: the read either finds nothing or names absolute
    // directories, and never panics or returns a string cut short at the wrong length.
    let found = open::registered_vlc();
    eprintln!("VLC's InstallDir in the registry: {found:?}");
    assert!(found.len() <= 2, "{found:?}");
    for dir in &found {
        assert!(dir.is_absolute(), "{}", dir.display());
        assert!(!dir.as_os_str().to_string_lossy().contains('\0'));
    }
}

#[test]
fn test_a_command_is_written_as_a_powershell_line_that_runs_as_it_stands() {
    let args: Vec<OsString> = [
        r"C:\Users\me\Videos\Black Hole Lab views\2026-09-30 10.00.00.000 UTC Bob at tau 1.500 M\2026-09-30 10.00.00.000 UTC Bob at tau 1.500 M.bhl",
        "--out",
        r"D:\Bob's views\bundle",
        "--grid",
        "4096x2048",
        "--exposure",
        "-1.5",
        "--decimal-comma",
        "a\u{2019}b",
    ]
    .iter()
    .map(OsString::from)
    .collect();
    let line = powershell_line(Path::new(r"C:\sky\sky-trace.exe"), &args);
    assert_eq!(
        line,
        concat!(
            r"& 'C:\sky\sky-trace.exe' 'C:\Users\me\Videos\Black Hole Lab views\2026-09-30 ",
            r"10.00.00.000 UTC Bob at tau 1.500 M\2026-09-30 10.00.00.000 UTC Bob at tau 1.500 ",
            r"M.bhl' --out 'D:\Bob''s views\bundle' --grid 4096x2048 --exposure -1.5 ",
            "--decimal-comma 'a\u{2019}\u{2019}b'"
        )
    );
}

#[test]
fn test_a_childs_percentage_becomes_a_sentence_every_five_points_and_never_at_100() {
    let mut shown = 0;
    let mut said = Vec::new();
    for p in 0..=100 {
        if let Some(sentence) =
            percent_sentence(&format!("progress {p}%"), &mut shown, || "Tracing".into())
        {
            said.push(sentence);
        }
    }
    let expected: Vec<String> = (1..20).map(|k| format!("Tracing: {}%.", 5 * k)).collect();
    assert_eq!(said, expected);
    // Any other line is not a percentage, and a percentage that does not read is passed over.
    for line in [
        "frame 1 of 1",
        "progress",
        "progress x%",
        "progress 50",
        "wrote 50%",
    ] {
        assert_eq!(
            percent_sentence(line, &mut 0, || "x".into()),
            None,
            "{line}"
        );
    }
}

#[test]
fn test_the_prompt_starts_in_the_views_folder() {
    let folder = Path::new("views").join("a view");
    let command = shell::command(&folder);
    assert_eq!(command.get_current_dir(), Some(folder.as_path()));
    let args: Vec<&std::ffi::OsStr> = command.get_args().collect();
    if cfg!(windows) {
        let program = Path::new(command.get_program())
            .file_name()
            .expect("a program")
            .to_string_lossy()
            .to_lowercase();
        assert_eq!(program, "cmd.exe", "the command prompt");
        assert_eq!(args, ["/K"], "which runs nothing and stays");
    } else {
        assert!(args.is_empty(), "{args:?}");
    }
}

#[test]
fn test_the_check_reports_every_piece_and_says_which_map_a_download_would_supply() {
    let root = scratch_dir("check");
    let (release, path) = layout(&root);
    let search = Search {
        exe_dir: Some(release),
        path: Some(path),
        ..Search::default()
    };
    let report = find::check(&search, &names());
    assert!(report.complete(), "{report:?}");
    assert_eq!(report.lines(), ["tools ok", "ffmpeg ok", "map ok"]);

    // Everything missing at once, where `find` would stop at the tools: each piece has its line,
    // and the default map in its own place is one a download would supply.
    let copy = root.join("downloaded/Black Hole Lab");
    std::fs::create_dir_all(&copy).expect("a downloaded copy");
    let bare = Search {
        exe_dir: Some(copy),
        ..Search::default()
    };
    let report = find::check(&bare, &names());
    assert!(!report.complete() && report.map_fetchable, "{report:?}");
    let lines = report.lines();
    assert!(
        lines[0].starts_with("tools missing sky-trace.exe was not found"),
        "{lines:#?}"
    );
    assert!(
        lines[1].starts_with("ffmpeg missing ffmpeg.exe was not found"),
        "{lines:#?}"
    );
    assert!(
        lines[2].starts_with("map fetchable The star map"),
        "{lines:#?}"
    );
    assert!(find::find(&bare, &names()).is_err());

    // A map named outright that is not there is one only whoever named it can supply.
    let named = Search {
        sky_env: Some(root.join("nowhere.exr").into()),
        ..bare
    };
    let report = find::check(&named, &names());
    assert!(!report.map_fetchable, "{report:?}");
    assert!(
        report.lines()[2].starts_with(&format!("map missing {SKY_MAP_ENV} names")),
        "{report:?}"
    );

    // ffmpeg that is on no PATH is found in a place it is installed to.
    let installed = root.join("winget/bin");
    touch(&installed.join("ffmpeg.exe"));
    let report = find::check(
        &Search {
            ffmpeg_places: vec![root.join("empty"), installed],
            ..named
        },
        &names(),
    );
    assert_eq!(report.ffmpeg, None);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn test_the_two_command_lines_that_make_no_view_take_only_their_own_options() {
    let parse = |args: &[&str]| args::parse(&args.iter().map(OsString::from).collect::<Vec<_>>());
    assert_eq!(
        parse(&["--check"]),
        Ok(Request::Check(args::CheckOptions::default()))
    );
    assert_eq!(
        parse(&["--sky", "m.exr", "--check", "--ffmpeg", "f"]),
        Ok(Request::Check(args::CheckOptions {
            tools: None,
            ffmpeg: Some("f".into()),
            sky: Some("m.exr".into()),
        }))
    );
    assert_eq!(
        parse(&["--fetch-map"]),
        Ok(Request::FetchMap { status: None })
    );
    assert_eq!(
        parse(&["--fetch-map", "--status", "s.txt"]),
        Ok(Request::FetchMap {
            status: Some("s.txt".into())
        })
    );
    for (args, word) in [
        (&["--check", "run.bhl"][..], "not run.bhl"),
        (&["--check", "--observer", "bob"][..], "not --observer"),
        (&["--fetch-map", "--sky", "m.exr"][..], "not --sky"),
        (&["--fetch-map", "--status"][..], "--status needs a value"),
    ] {
        let why = parse(args).expect_err("refused");
        assert!(why.contains(word) && why.ends_with('.'), "{args:?}: {why}");
    }
}

#[test]
fn test_the_sha_256_is_the_standards_for_its_own_examples() {
    let hex = |bytes: &[u8]| sha256::hex_of(&mut &bytes[..]).expect("a slice reads");
    assert_eq!(
        hex(b""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
    // Two blocks' worth, which the padding spills into a third.
    assert_eq!(
        hex(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
    );
    // A million bytes, read in pieces that do not end on a block.
    assert_eq!(
        hex(&vec![b'a'; 1_000_000]),
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"
    );
}

#[test]
fn test_the_command_that_installs_ffmpeg_is_the_distributions_own() {
    let command = find::linux_install_command;
    // Ubuntu and what derives from it or from Debian: apt.
    let ubuntu = "PRETTY_NAME=\"Ubuntu 24.04.1 LTS\"\nNAME=\"Ubuntu\"\nID=ubuntu\nID_LIKE=debian\n";
    assert_eq!(command(ubuntu), Some("sudo apt install ffmpeg"));
    assert_eq!(command("ID=debian\n"), Some("sudo apt install ffmpeg"));
    assert_eq!(
        command("ID=linuxmint\nID_LIKE=\"ubuntu debian\"\n"),
        Some("sudo apt install ffmpeg")
    );
    // Fedora and what derives from it: dnf, and the package Fedora's own repositories carry.
    assert_eq!(
        command("NAME=\"Fedora Linux\"\nID=fedora\nVERSION_ID=42\n"),
        Some("sudo dnf install ffmpeg-free")
    );
    assert_eq!(
        command("ID=\"rocky\"\nID_LIKE=\"rhel centos fedora\"\n"),
        Some("sudo dnf install ffmpeg-free")
    );
    assert_eq!(command("ID=arch\n"), Some("sudo pacman -S ffmpeg"));
    assert_eq!(
        command("ID=\"opensuse-tumbleweed\"\nID_LIKE=\"opensuse suse\"\n"),
        Some("sudo zypper install ffmpeg")
    );
    // A distribution this program does not know, and a name that only looks like one it does.
    assert_eq!(command("ID=nixos\n"), None);
    assert_eq!(command("NAME=fedora\nVERSION_ID=ubuntu\n"), None);
    assert_eq!(command(""), None);

    // The sentence carries the command, and the check gives it a line of its own.
    let root = scratch_dir("install");
    let search = Search {
        exe_dir: Some(root.clone()),
        os_release: Some("ID=fedora\n".into()),
        ..Search::default()
    };
    let report = find::check(&search, &names());
    let expected = find::ffmpeg_install_command(Some("ID=fedora\n")).expect("a command");
    assert_eq!(report.ffmpeg_command, Some(expected));
    assert!(
        report
            .ffmpeg
            .as_ref()
            .is_some_and(|why| why.contains(expected)),
        "{report:?}"
    );
    assert_eq!(report.lines()[3], format!("ffmpeg-install {expected}"));
    let _ = std::fs::remove_dir_all(&root);
}
