//! The read-outs as subtitles, the still, the photograph, and the files a run leaves.
//!
//! The subtitle script is read back here the way libass reads it (an unescaped `{` opens a
//! style override, a backslash and `N`, `n`, `h`, `{` or `}` is an escape, any other backslash is
//! itself), so that what a test checks is what a player would show. The tests through ffmpeg
//! write a bundle of a few rays and a map of a few texels, render them at 128 x 64, and read the
//! results back with ffprobe; without ffmpeg they pass with a note.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use readout::Style;
use sky_format::{
    BundleWriter, FORMAT, FarSky, Frame, FrameEntry, Geometry, Grid, GridSpec, Manifest, Num,
    Playback, ReadoutDecl, Source, TimeUnit, VERSION, WriterInfo, fate,
};

use crate::cli::{Container, Request, Still};
use crate::encode::ffmpeg_available;
use crate::photo;
use crate::subtitles::{self, Cue, Look};
use crate::timeline::Timeline;
use crate::values::{Series, format};

// ---- helpers -------------------------------------------------------------------------------------

/// A directory of its own for one test, emptied first and removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir()
            .join("sky-render-tests")
            .join(format!("{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Self(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }

    /// The names of the files in the directory, sorted.
    fn listing(&self) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(&self.0)
            .expect("the directory is there")
            .map(|e| {
                e.expect("an entry")
                    .file_name()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect();
        names.sort();
        names
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn decl(id: &str, label: &str, unit: &str, decimals: u32) -> ReadoutDecl {
    ReadoutDecl {
        id: id.into(),
        label: label.into(),
        unit: unit.into(),
        decimals,
    }
}

/// The read-outs of the tests: a stopwatch, a radius, and a shift with no unit.
fn decls() -> Vec<ReadoutDecl> {
    vec![
        decl("stopwatch", "Stopwatch", "s", 2),
        decl("r", "Radius", "M", 3),
        decl("g", "Shift", "", 1),
    ]
}

/// Three bundle frames a tenth of a second apart. The radius falls from 6 to 5 and then to minus
/// infinity; the shift is 1.5, then NaN, then missing. The bundle's own stopwatch values are
/// deliberately wrong (the subtitles must show the timeline's).
fn frame_values() -> Vec<BTreeMap<String, Num>> {
    let map = |pairs: &[(&str, f64)]| {
        pairs
            .iter()
            .map(|(id, v)| (id.to_string(), Num(*v)))
            .collect::<BTreeMap<_, _>>()
    };
    vec![
        map(&[("stopwatch", 9.0), ("r", 6.0), ("g", 1.5)]),
        map(&[("stopwatch", 9.0), ("r", 5.0), ("g", f64::NAN)]),
        map(&[("stopwatch", 9.0), ("r", f64::NEG_INFINITY)]),
    ]
}

/// The series and timeline of those frames, played at 30 frames a second, one second of proper
/// time a second: seven video frames.
fn film(decls: &[ReadoutDecl]) -> (Series, Timeline) {
    let values = frame_values();
    let series = Series::new(decls, &values.iter().collect::<Vec<_>>());
    let timeline = Timeline::new(&[0.0, 0.1, 0.2], 0.0, 30.0, 1.0);
    (series, timeline)
}

fn look(style: Style) -> Look {
    Look {
        font: subtitles::DEFAULT_FONT.into(),
        percent: subtitles::DEFAULT_PERCENT,
        style,
    }
}

/// One dialogue line read back: its start and end in hundredths, and the lines a player shows.
#[derive(Debug)]
struct Shown {
    start: u64,
    end: u64,
    lines: Vec<String>,
}

/// `H:MM:SS.cc` in hundredths.
fn parse_time(text: &str) -> u64 {
    let (hms, cc) = text.split_once('.').expect("hundredths");
    let parts: Vec<u64> = hms
        .split(':')
        .map(|p| p.parse().expect("a number"))
        .collect();
    ((parts[0] * 60 + parts[1]) * 60 + parts[2]) * 100 + cc.parse::<u64>().expect("hundredths")
}

/// A cue's text as libass shows it: its lines, hard spaces as spaces, zero-width joiners gone.
/// Panics on an override block, which a cue written here must never contain.
fn displayed(text: &str) -> Vec<String> {
    let mut lines = vec![String::new()];
    let chars: Vec<char> = text.chars().collect();
    let mut k = 0;
    while k < chars.len() {
        let c = chars[k];
        if c == '{' {
            panic!("the cue {text:?} opens a style override at character {k}");
        }
        if c == '\\' && k + 1 < chars.len() {
            let shown = match chars[k + 1] {
                'N' => {
                    lines.push(String::new());
                    k += 2;
                    continue;
                }
                // With WrapStyle 2, `\n` is a line break too.
                'n' => panic!("the cue {text:?} holds a \\n"),
                'h' => Some(' '),
                '{' => Some('{'),
                '}' => Some('}'),
                _ => None,
            };
            if let Some(s) = shown {
                lines.last_mut().unwrap().push(s);
                k += 2;
                continue;
            }
        }
        if c != '\u{2060}' {
            lines.last_mut().unwrap().push(c);
        }
        k += 1;
    }
    lines
}

/// Every dialogue line of a script, read back.
fn dialogues(script: &str) -> Vec<Shown> {
    script
        .lines()
        .filter_map(|line| line.strip_prefix("Dialogue: "))
        .map(|rest| {
            // Nine commas separate ten fields; the text, last, may hold commas of its own.
            let fields: Vec<&str> = rest.splitn(10, ',').collect();
            assert_eq!(fields.len(), 10, "{rest}");
            assert_eq!(fields[3], "Readout");
            Shown {
                start: parse_time(fields[1]),
                end: parse_time(fields[2]),
                lines: displayed(fields[9]),
            }
        })
        .collect()
}

fn args(extra: &[&str]) -> Vec<String> {
    let mut a: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--out", "v.mkv"]
        .map(String::from)
        .to_vec();
    a.extend(extra.iter().map(|s| s.to_string()));
    a
}

fn parse(extra: &[&str]) -> Result<crate::cli::Options, String> {
    match crate::cli::parse(&args(extra))? {
        Request::Render(o) => Ok(*o),
        Request::Help => panic!("not help"),
    }
}

// ---- the subtitle script -------------------------------------------------------------------------

#[test]
fn test_each_video_frame_has_one_cue_and_the_cues_tile_the_film_at_the_nearest_hundredths() {
    let (series, timeline) = film(&decls());
    let frames = timeline.video_frames();
    assert_eq!(frames, 7);
    let cues = subtitles::film_cues(&series, &timeline, 0..frames, 30.0);
    let shown = dialogues(&subtitles::script(
        &series.lines,
        &cues,
        &look(Style::POINT),
    ));
    assert_eq!(shown.len(), 7, "one cue per video frame");
    for (k, cue) in shown.iter().enumerate() {
        // Each boundary is the hundredth nearest to k / 30 s, computed here independently.
        let nearest = |k: usize| (k as f64 * 100.0 / 30.0 + 0.5).floor() as u64;
        assert_eq!(
            (cue.start, cue.end),
            (nearest(k), nearest(k + 1)),
            "cue {k}"
        );
        assert!(cue.end > cue.start);
        assert!(cue.end - cue.start == 3 || cue.end - cue.start == 4);
        if let Some(next) = shown.get(k + 1) {
            assert_eq!(cue.end, next.start, "a gap or an overlap after cue {k}");
        }
    }
    assert_eq!(shown[0].start, 0);
    assert_eq!(shown[6].end, 23); // 7 frames of 1/30 s: 0.2333 s
    // A later start: the cues are timed from the video's own first frame.
    let later = subtitles::film_cues(&series, &timeline, 1..7, 30.0);
    assert_eq!((later[0].start, later[0].end), (0, 3));
    assert_eq!(
        later[0].values,
        series.at(timeline.pick(1), timeline.stopwatch(1))
    );
    // Times are written H:MM:SS.cc.
    assert_eq!(subtitles::time(0), "0:00:00.00");
    assert_eq!(subtitles::time(360_000 + 61 * 100 + 7), "1:01:01.07");
    // Above 100 frames a second a frame can round to no time at all; its cue is left out.
    let fast = subtitles::frame_times(10, 240.0);
    assert!(fast.iter().any(|(s, e)| s == e));
    let cues: Vec<Cue> = fast
        .iter()
        .map(|&(start, end)| Cue {
            start,
            end,
            values: vec![Some(1.0), None, None],
        })
        .collect();
    let shown = dialogues(&subtitles::script(
        &series.lines,
        &cues,
        &look(Style::POINT),
    ));
    assert!(shown.iter().all(|c| c.end > c.start));
    assert_eq!(shown.len(), fast.iter().filter(|(s, e)| e > s).count());
}

#[test]
fn test_the_cues_show_the_values_the_panel_shows_including_nan_infinity_and_a_missing_one() {
    let (series, timeline) = film(&decls());
    let cues = subtitles::film_cues(&series, &timeline, 0..7, 30.0);
    let shown = dialogues(&subtitles::script(
        &series.lines,
        &cues,
        &look(Style::POINT),
    ));
    for (k, cue) in shown.iter().enumerate() {
        // What `values.rs` gives for this frame, written as `values.rs` writes it.
        let values = series.at(timeline.pick(k as u64), timeline.stopwatch(k as u64));
        let wanted: Vec<String> = values
            .iter()
            .zip(&series.lines)
            .filter_map(|(v, line)| v.map(|x| format(Style::POINT, x, line.decimals)))
            .collect();
        assert_eq!(cue.lines.len(), wanted.len(), "frame {k}: {:?}", cue.lines);
        for (line, value) in cue.lines.iter().zip(&wanted) {
            assert!(
                line.split_whitespace().any(|word| word == value),
                "frame {k}: {line:?} does not show {value:?}"
            );
        }
    }
    let text = |k: usize| shown[k].lines.join(" | ");
    // The stopwatch is the timeline's (k / 30 s), not the bundle's 9.
    assert!(text(1).contains("0.03"), "{}", text(1));
    assert!(!text(1).contains("9.00"));
    // A third of the way from 6 to 5.
    assert!(text(1).contains("5.667"), "{}", text(1));
    // NaN is "n/a"; minus infinity keeps its sign; a missing read-out is left out entirely.
    assert!(text(3).contains("n/a"), "{}", text(3));
    assert!(text(6).contains("-∞"), "{}", text(6));
    assert_eq!(shown[6].lines.len(), 2, "{}", text(6));
    assert!(!text(6).contains("Shift"));
    // The lines come in the manifest's order.
    assert!(shown[0].lines[0].starts_with("Stopwatch"));
    assert!(shown[0].lines[1].starts_with("Radius"));
    assert!(shown[0].lines[2].starts_with("Shift"));
}

#[test]
fn test_every_line_is_padded_into_the_same_columns_so_the_digits_hold_still() {
    // Units of different lengths, and a value that grows a digit during the film.
    let decls = vec![
        decl("stopwatch", "T", "s", 2),
        decl("r", "Radius", "km/s", 3),
        decl("g", "Shift", "", 1),
    ];
    let (series, timeline) = film(&decls);
    let mut cues = subtitles::film_cues(&series, &timeline, 0..7, 30.0);
    cues[2].values[1] = Some(12.5);
    let script = subtitles::script(&series.lines, &cues, &look(Style::POINT));
    let shown = dialogues(&script);
    let all: Vec<&String> = shown.iter().flat_map(|c| &c.lines).collect();
    let length = all[0].chars().count();
    for line in &all {
        assert_eq!(
            line.chars().count(),
            length,
            "{line:?} is not {length} long"
        );
    }
    // The values' last characters are in one column, so a digit's place never changes; and the
    // label column starts every line.
    let value_end = |line: &str| -> usize {
        let chars: Vec<char> = line.chars().collect();
        let label = chars.iter().position(|c| *c == ' ').unwrap();
        let start = (label..chars.len()).find(|&i| chars[i] != ' ').unwrap();
        (start..chars.len())
            .find(|&i| chars[i] == ' ')
            .unwrap_or(chars.len())
    };
    let column = value_end(all[0]);
    for line in &all {
        assert_eq!(value_end(line), column, "{line:?}");
    }
    // The padding at the end of a line is hard spaces, which the player does not trim before it
    // right-aligns the line.
    let dialogue = script.lines().find(|l| l.starts_with("Dialogue")).unwrap();
    assert!(!dialogue.ends_with(' '));
    assert!(dialogue.contains("\\h"));
}

#[test]
fn test_the_decimal_comma_reaches_the_subtitles() {
    let (series, timeline) = film(&decls());
    let cues = subtitles::film_cues(&series, &timeline, 0..7, 30.0);
    let shown = dialogues(&subtitles::script(
        &series.lines,
        &cues,
        &look(Style::COMMA),
    ));
    let text = shown[1].lines.join(" | ");
    assert!(text.contains("5,667") && text.contains("0,03"), "{text}");
    assert!(!text.contains("5.667"));
}

#[test]
fn test_a_label_or_unit_cannot_restyle_or_break_the_cue_and_is_shown_as_written() {
    let hostile = "Bad {\\b1\\c&H0000FF&} label\\N\\h\\n}\nnext\tline\\";
    let decls = vec![
        decl("stopwatch", hostile, "{\\i1}s\\", 2),
        decl("r", "Radius", "M", 3),
    ];
    let (series, timeline) = film(&decls);
    let cues = subtitles::film_cues(&series, &timeline, 0..2, 30.0);
    let script = subtitles::script(&series.lines, &cues, &look(Style::POINT));
    // Two cues of two lines each, one file line per cue: the newline in the label did not end it.
    assert_eq!(
        script.lines().filter(|l| l.starts_with("Dialogue")).count(),
        2
    );
    // `displayed` panics on an override block and on `\n`; it finds neither.
    let shown = dialogues(&script);
    assert_eq!(shown[0].lines.len(), 2);
    let first = &shown[0].lines[0];
    // The label as written, its control characters as spaces.
    let label = "Bad {\\b1\\c&H0000FF&} label\\N\\h\\n} next line\\";
    assert!(first.starts_with(label), "{first:?}");
    assert!(first.trim_end().ends_with("{\\i1}s\\"), "{first:?}");
    // Escaping itself.
    assert_eq!(subtitles::escape("a{b}c"), "a\\{b\\}c");
    assert_eq!(subtitles::escape("\\N"), "\\\u{2060}N");
    assert_eq!(subtitles::escape("x\r\ny"), "x  y");
}

#[test]
fn test_the_style_line_is_the_approved_trials_at_the_default_size_and_scales_with_the_option() {
    let default = look(Style::POINT);
    // The half-size trial's line, to the character: the typeface too is the one approved.
    assert_eq!(
        subtitles::style_line(&default),
        "Style: Readout,Consolas,18,&H00F0F0F0,&H00F0F0F0,&H00000000,&H99000000,0,0,0,0,100,\
         100,0,0,3,2,0,3,40,40,40,1"
    );
    assert!((subtitles::DEFAULT_PERCENT - 100.0 * 18.0 / 1080.0).abs() < 1e-12);
    // --overlay-size 5: 54 of 1080, the box's padding scaled with it; alignment 3 (bottom right)
    // and the margins unchanged.
    let big = Look {
        font: "Courier New".into(),
        percent: 5.0,
        style: Style::POINT,
    };
    let line = subtitles::style_line(&big);
    let fields: Vec<&str> = line.trim_start_matches("Style: ").split(',').collect();
    assert_eq!(fields.len(), 23);
    assert_eq!(
        (fields[1], fields[2], fields[16]),
        ("Courier New", "54", "6")
    );
    assert_eq!((fields[15], fields[18]), ("3", "3"));
    // The script states the resolution the size is measured against, and turns wrapping off.
    let (series, timeline) = film(&decls());
    let cues = subtitles::film_cues(&series, &timeline, 0..1, 30.0);
    let script = subtitles::script(&series.lines, &cues, &big);
    for wanted in ["PlayResX: 1920", "PlayResY: 1080", "WrapStyle: 2", &line] {
        assert!(script.contains(wanted), "the script lacks {wanted:?}");
    }
    // Through the command line.
    let o = parse(&[
        "--overlay-size",
        "2.5",
        "--overlay-font",
        "DejaVu Sans Mono",
    ])
    .unwrap();
    assert_eq!(
        (o.overlay_size, o.overlay_font.as_str()),
        (2.5, "DejaVu Sans Mono")
    );
    assert!(crate::cli::USAGE.contains("default 1.67"));
}

// ---- the command line ----------------------------------------------------------------------------

#[test]
fn test_readouts_on_means_panel_and_each_bad_mode_extension_still_and_hold_has_its_own_sentence() {
    let o = parse(&[]).unwrap();
    assert!(o.subtitles && !o.readouts, "the overlay is the default");
    assert_eq!(o.container, Some(Container::Mkv));
    let on = parse(&["--readouts", "on"]).unwrap();
    let panel = parse(&["--readouts", "panel"]).unwrap();
    assert_eq!(on, panel);
    assert!(panel.readouts && !panel.subtitles);
    let off = parse(&["--readouts", "off"]).unwrap();
    assert!(!off.readouts && !off.subtitles);
    let mut a = args(&[]);
    a[5] = "v.MP4".into();
    let Request::Render(mp4) = crate::cli::parse(&a).unwrap() else {
        panic!()
    };
    assert_eq!(mp4.container, Some(Container::Mp4));

    let refusal = |extra: &[&str], out: &str| {
        let mut a = args(extra);
        a[5] = out.into();
        crate::cli::parse(&a).expect_err(&format!("{extra:?} {out}"))
    };
    let sentences = [
        (
            refusal(&["--readouts", "maybe"], "v.mkv"),
            "overlay, panel or off",
        ),
        (refusal(&[], "v.avi"), "ends neither in .mkv"),
        (refusal(&[], "video"), "ends neither in .mkv"),
        (
            refusal(&["--still", "3", "--hold", "0"], "v.mkv"),
            "--hold is how many seconds",
        ),
        (
            refusal(&["--still", "3", "--hold", "-2"], "v.mkv"),
            "--hold is how many seconds",
        ),
        (
            refusal(&["--still", "3", "--hold", "nan"], "v.mkv"),
            "--hold is how many seconds",
        ),
        (
            refusal(&["--still", "x"], "v.mkv"),
            "--still takes a video frame number",
        ),
        (
            refusal(&["--hold", "5"], "v.mkv"),
            "--hold belongs to a still",
        ),
        (
            refusal(&["--photo", "p.jpg"], "v.mkv"),
            "--photo belongs to a still",
        ),
        (
            refusal(&["--still", "1", "--photo", "p.png"], "v.mkv"),
            "ending .jpg or .jpeg",
        ),
        (
            refusal(&["--still", "1", "--frames", "0..3"], "v.mkv"),
            "give one",
        ),
        (refusal(&["--overlay-size", "0"], "v.mkv"), "--overlay-size"),
        (
            refusal(&["--overlay-font", "A,B"], "v.mkv"),
            "--overlay-font",
        ),
    ];
    for (message, wanted) in &sentences {
        assert!(
            message.contains(wanted),
            "{message:?} does not say {wanted:?}"
        );
    }
    // A still past the film's end, and a still of a longer film with no frame named, are refused
    // when the film's length is known, each in its own words.
    let still = |frame| Still {
        frame,
        hold: 60.0,
        rate: 0.5,
        photo: None,
    };
    let past = still(Some(7)).frame_of(7).unwrap_err();
    assert!(
        past.contains("past the end") && past.contains("0 to 6"),
        "{past}"
    );
    let unnamed = still(None).frame_of(7).unwrap_err();
    assert!(unnamed.contains("needs a video frame number"), "{unnamed}");
    assert_ne!(past, unnamed);
    let distinct: std::collections::BTreeSet<&str> = [
        sentences[0].0.as_str(),
        sentences[1].0.as_str(),
        sentences[3].0.as_str(),
        past.as_str(),
    ]
    .into();
    assert_eq!(distinct.len(), 4);
}

#[test]
fn test_a_still_is_one_frame_held_for_whole_frames_at_its_own_rate() {
    let o = parse(&["--still", "237", "--photo", "view.jpg"]).unwrap();
    let still = o.still.clone().expect("a still");
    assert_eq!(still.frame, Some(237));
    assert_eq!(still.photo, Some(PathBuf::from("view.jpg")));
    // A minute at half a frame a second: thirty frames.
    assert_eq!(
        (still.hold, still.rate, still.repeats(), still.seconds()),
        (60.0, 0.5, 30, 60.0)
    );
    assert_eq!(still.frame_of(474), Ok(237));
    // The frame number may be left out, before another option or at the end.
    let o = parse(&["--still", "--hold", "5"]).unwrap();
    let still = o.still.clone().unwrap();
    assert_eq!(still.frame, None);
    assert_eq!(still.frame_of(1), Ok(0));
    // 5 s at half a frame a second is 2.5 frames: three, and the video lasts 6 s.
    assert_eq!((still.repeats(), still.seconds()), (3, 6.0));
    assert!(parse(&["--still"]).unwrap().still.is_some());
    let o = parse(&["--still", "0", "--hold", "0.1", "--hold-rate", "30"]).unwrap();
    assert_eq!(o.still.unwrap().repeats(), 3);
}

// ---- the photograph ------------------------------------------------------------------------------

/// A JPEG's header segments and a scan, enough for the segment walker: SOI, a comment, a SOF0
/// of `width` x `height`, SOS, two bytes of scan data, EOI.
fn tiny_jpeg(width: u16, height: u16, app0: bool) -> Vec<u8> {
    let mut out = vec![0xFF, 0xD8];
    if app0 {
        out.extend([0xFF, 0xE0, 0x00, 0x10]);
        out.extend(b"JFIF\0\x01\x02\0\0\x01\0\x01\0\0");
    }
    out.extend([0xFF, 0xFE, 0x00, 0x06]);
    out.extend(b"Lavc");
    let [h1, h0] = height.to_be_bytes();
    let [w1, w0] = width.to_be_bytes();
    out.extend([0xFF, 0xC0, 0x00, 0x11, 8, h1, h0, w1, w0, 3]);
    out.extend([1, 0x11, 0, 2, 0x11, 1, 3, 0x11, 1]);
    out.extend([0xFF, 0xDA, 0x00, 0x0C, 3, 1, 0, 2, 0x11, 3, 0x11, 0, 63, 0]);
    out.extend([0x12, 0x34, 0xFF, 0xD9]);
    out
}

/// Checks that `xml` is well formed as far as its elements go: every element opened is closed,
/// in order.
fn assert_balanced(xml: &str) {
    let mut open: Vec<String> = Vec::new();
    let mut rest = xml;
    while let Some(at) = rest.find('<') {
        let end = rest[at..].find('>').expect("a tag ends") + at;
        let tag = &rest[at + 1..end];
        rest = &rest[end + 1..];
        if tag.starts_with('?') {
            assert!(tag.ends_with('?'), "{tag}");
        } else if let Some(name) = tag.strip_prefix('/') {
            assert_eq!(
                open.pop().as_deref(),
                Some(name),
                "</{name}> closes the wrong element"
            );
        } else if !tag.ends_with('/') {
            open.push(tag.split_whitespace().next().unwrap().to_string());
        }
    }
    assert!(open.is_empty(), "{open:?} are never closed");
}

/// The value of the element `name` in `xml`.
fn property<'a>(xml: &'a str, name: &str) -> &'a str {
    let open = format!("<{name}>");
    let from = xml.find(&open).unwrap_or_else(|| panic!("no {name}")) + open.len();
    let to = xml[from..].find('<').unwrap() + from;
    &xml[from..to]
}

/// Checks a JPEG's Photo Sphere metadata against its own size.
fn assert_photo_sphere(jpeg: &[u8], width: u32, height: u32) {
    assert_eq!(photo::dimensions(jpeg), Ok((width, height)));
    let segments = photo::segments(jpeg).expect("a JPEG");
    let xmp_segment = segments
        .iter()
        .find(|s| s.marker == 0xE1 && s.payload(jpeg).starts_with(photo::XMP_NAMESPACE))
        .expect("an XMP APP1 segment");
    // The segment's length field counts itself and the payload.
    let length = u16::from_be_bytes([jpeg[xmp_segment.start + 2], jpeg[xmp_segment.start + 3]]);
    assert_eq!(usize::from(length), xmp_segment.end - xmp_segment.start - 2);
    let xmp = photo::xmp_of(jpeg).expect("XMP");
    assert_balanced(&xmp);
    let (w, h) = (width.to_string(), height.to_string());
    for (name, wanted) in [
        ("GPano:ProjectionType", "equirectangular"),
        ("GPano:UsePanoramaViewer", "True"),
        ("GPano:FullPanoWidthPixels", w.as_str()),
        ("GPano:FullPanoHeightPixels", h.as_str()),
        ("GPano:CroppedAreaImageWidthPixels", w.as_str()),
        ("GPano:CroppedAreaImageHeightPixels", h.as_str()),
        ("GPano:CroppedAreaLeftPixels", "0"),
        ("GPano:CroppedAreaTopPixels", "0"),
    ] {
        assert_eq!(property(&xmp, name), wanted, "{name}");
    }
    assert!(xmp.contains("xmlns:GPano=\"http://ns.google.com/photos/1.0/panorama/\""));
    // The file still ends as a JPEG does.
    assert_eq!(&jpeg[jpeg.len() - 2..], &[0xFF, 0xD9]);
}

#[test]
fn test_the_photo_sphere_segment_follows_the_start_of_image_and_states_the_images_size() {
    let plain = tiny_jpeg(8192, 4096, false);
    let marked = photo::with_photo_sphere(&plain).expect("marked");
    // Straight after the start of image, and everything after it as it was.
    assert_eq!(&marked[2..4], &[0xFF, 0xE1]);
    let added = marked.len() - plain.len();
    assert_eq!(&marked[2 + added..], &plain[2..]);
    assert_photo_sphere(&marked, 8192, 4096);
    // After a JFIF APP0, which must stay first.
    let jfif = tiny_jpeg(640, 320, true);
    let marked = photo::with_photo_sphere(&jfif).expect("marked");
    assert_eq!(&marked[2..4], &[0xFF, 0xE0]);
    assert_eq!(&marked[20..22], &[0xFF, 0xE1]);
    assert_photo_sphere(&marked, 640, 320);
    // Not twice, and not on something that is not a JPEG.
    assert!(
        photo::with_photo_sphere(&marked)
            .unwrap_err()
            .contains("already")
    );
    assert!(photo::with_photo_sphere(b"PNG").is_err());
}

// ---- through ffmpeg ------------------------------------------------------------------------------

/// Whether ffmpeg and ffprobe run; if not, says so for a test that then passes.
fn have_ffmpeg(test: &str) -> bool {
    let ok = ffmpeg_available(Path::new("ffmpeg")) && ffmpeg_available(Path::new("ffprobe"));
    if !ok {
        eprintln!("note: ffmpeg or ffprobe is not on PATH, so {test} was skipped");
    }
    ok
}

/// Writes a bundle of three frames (as `frame_values`) on a 16 x 8 grid, every ray from the far
/// sky along the direction it was traced, and a 64 x 32 star map; returns their paths.
fn write_inputs(dir: &Path) -> (PathBuf, PathBuf) {
    let bundle = dir.join("bundle");
    let manifest = Manifest {
        format: FORMAT.into(),
        version: VERSION,
        writer: WriterInfo {
            program: "sky-render tests".into(),
            version: "0".into(),
            git: None,
        },
        source: Source {
            kind: "flat-space test".into(),
            name: Some("still".into()),
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
        grid: GridSpec::new(16, 8),
        far_sky: FarSky::galactic(),
        playback: Playback {
            frames_per_second: Num(30.0),
            proper_time_per_video_second: Num(1.0),
        },
        readouts: decls(),
        labels: Vec::new(),
        frames_planned: Some(3),
        frames: Vec::new(),
    };
    let mut writer = BundleWriter::create(&bundle, manifest).expect("a bundle");
    let grid = Grid::new(16, 8);
    for (index, values) in frame_values().into_iter().enumerate() {
        let mut frame = Frame::new(16, 8, index as u32);
        for j in 0..8 {
            for i in 0..16 {
                let k = grid.offset(i, j);
                let d = grid.pixel_direction(i, j);
                frame.fate[k] = fate::FAR_SKY;
                for (plane, component) in frame.direction.iter_mut().zip(d) {
                    plane[k] = component as f32;
                }
                frame.shift[k] = 1.0;
            }
        }
        let mut entry = FrameEntry::new(index as u32, index as f64 * 0.1);
        entry.readouts = values;
        writer
            .write_frame_and_manifest(&frame, entry)
            .expect("a frame");
    }
    let map = dir.join("map.exr");
    exr::prelude::write_rgb_file(&map, 64, 32, |x, y| {
        let v = ((x * 7 + y * 13) % 17) as f32 / 17.0;
        (v, 0.5 * v, 0.25)
    })
    .expect("a map");
    (bundle, map)
}

/// Renders with `extra` options into `out`.
fn render(bundle: &Path, map: &Path, out: &Path, extra: &[&str]) -> Result<(), String> {
    let mut a: Vec<String> = vec![
        "--bundle".into(),
        bundle.to_string_lossy().into_owned(),
        "--sky".into(),
        map.to_string_lossy().into_owned(),
        "--out".into(),
        out.to_string_lossy().into_owned(),
        "--size".into(),
        "128x64".into(),
        "--preset".into(),
        "12".into(),
        "--threads".into(),
        "2".into(),
    ];
    a.extend(extra.iter().map(|s| s.to_string()));
    let Request::Render(o) = crate::cli::parse(&a)? else {
        panic!("not a render")
    };
    crate::run(&o)
}

/// ffprobe's report on a file, flat: `streams.stream.0.codec_name="av1"` and so on.
fn probe(path: &Path) -> String {
    let out = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-show_entries",
            "format=duration:stream=index,codec_type,codec_name,width,height,r_frame_rate:\
             stream_disposition=default:stream_side_data_list",
            "-of",
            "flat",
        ])
        .arg(path)
        .output()
        .expect("ffprobe runs");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// The subtitle track of a Matroska file, as ASS text.
fn extract_subtitles(path: &Path) -> String {
    let out = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(path)
        .args(["-map", "0:s:0", "-c:s", "copy", "-f", "ass", "-"])
        .output()
        .expect("ffmpeg runs");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// Checks that a Matroska file holds one AV1 stream tagged as equirectangular and one ASS
/// stream marked as the default.
fn assert_spherical_mkv_with_subtitles(report: &str) {
    for wanted in [
        "streams.stream.0.codec_type=\"video\"",
        "streams.stream.0.codec_name=\"av1\"",
        "streams.stream.0.side_data_list.side_data.0.side_data_type=\"Spherical Mapping\"",
        "streams.stream.0.side_data_list.side_data.0.projection=\"equirectangular\"",
        "streams.stream.1.codec_type=\"subtitle\"",
        "streams.stream.1.codec_name=\"ass\"",
        "streams.stream.1.disposition.default=1",
    ] {
        assert!(
            report.contains(wanted),
            "ffprobe does not report {wanted}:\n{report}"
        );
    }
    assert!(
        !report.contains("streams.stream.2."),
        "more than two streams:\n{report}"
    );
}

#[test]
fn test_a_small_bundle_makes_a_matroska_film_with_a_default_subtitle_track_tagged_spherical() {
    if !have_ffmpeg("the Matroska test") {
        return;
    }
    let scratch = Scratch::new("mkv");
    let (bundle, map) = write_inputs(scratch.path());
    let out = scratch.path().join("film.mkv");
    render(&bundle, &map, &out, &[]).expect("the film is made");
    let report = probe(&out);
    assert_spherical_mkv_with_subtitles(&report);
    assert!(
        report.contains("width=128") && report.contains("height=64"),
        "{report}"
    );
    // The track read back holds the seven cues the script had.
    let track = extract_subtitles(&out);
    let shown = dialogues(&track);
    assert_eq!(shown.len(), 7, "{track}");
    assert_eq!(shown[1].lines.len(), 3);
    assert!(shown[1].lines[1].contains("5.667"), "{:?}", shown[1].lines);
    // Nothing but the bundle, the map and the film is left.
    assert_eq!(scratch.listing(), ["bundle", "film.mkv", "map.exr"]);
    // With the read-outs off, a Matroska file of the video alone.
    let bare = scratch.path().join("bare.mkv");
    render(&bundle, &map, &bare, &["--readouts", "off"]).expect("the film is made");
    let report = probe(&bare);
    assert!(
        report.contains("projection=\"equirectangular\""),
        "{report}"
    );
    assert!(!report.contains("subtitle"), "{report}");
}

#[test]
fn test_an_mp4_gets_its_subtitles_beside_it_and_a_failed_run_leaves_nothing() {
    if !have_ffmpeg("the sidecar test") {
        return;
    }
    let scratch = Scratch::new("sidecar");
    let (bundle, map) = write_inputs(scratch.path());
    let out = scratch.path().join("film.mp4");
    render(&bundle, &map, &out, &["--decimal-comma"]).expect("the film is made");
    let report = probe(&out);
    assert!(
        report.contains("projection=\"equirectangular\""),
        "{report}"
    );
    assert!(
        !report.contains("subtitle"),
        "an MP4 carries no subtitle track:\n{report}"
    );
    let sidecar = std::fs::read_to_string(scratch.path().join("film.ass")).expect("a sidecar");
    let shown = dialogues(&sidecar);
    assert_eq!(shown.len(), 7);
    assert!(shown[1].lines[1].contains("5,667"), "{:?}", shown[1].lines);
    assert_eq!(
        scratch.listing(),
        ["bundle", "film.ass", "film.mp4", "map.exr"]
    );
    // A sidecar is not replaced without --overwrite, even when the video it belonged to is gone.
    std::fs::remove_file(&out).expect("the film is there");
    let again = render(&bundle, &map, &out, &[]).unwrap_err();
    assert!(again.contains("film.ass already exists"), "{again}");
    // A run that fails in ffmpeg (an impossible preset) leaves no intermediate file behind.
    let failed = render(
        &bundle,
        &map,
        &scratch.path().join("broken.mkv"),
        &["--preset", "99"],
    );
    let failed = failed.unwrap_err();
    assert!(
        failed.contains("ffmpeg") && failed.contains("preset"),
        "{failed}"
    );
    assert_eq!(scratch.listing(), ["bundle", "film.ass", "map.exr"]);
}

#[test]
fn test_a_still_makes_a_held_video_and_a_360_degree_photograph() {
    if !have_ffmpeg("the still test") {
        return;
    }
    let scratch = Scratch::new("still");
    let (bundle, map) = write_inputs(scratch.path());
    let out = scratch.path().join("view.mkv");
    let jpg = scratch.path().join("view.jpg");
    let jpg_text = jpg.to_string_lossy().into_owned();
    render(
        &bundle,
        &map,
        &out,
        &["--still", "3", "--hold", "6", "--photo", &jpg_text],
    )
    .expect("the still is made");
    let report = probe(&out);
    assert_spherical_mkv_with_subtitles(&report);
    // Six seconds at half a frame a second, stated truly.
    assert!(report.contains("format.duration=\"6.000000\""), "{report}");
    assert!(report.contains("r_frame_rate=\"1/2\""), "{report}");
    // One cue, the frame's values for the whole length.
    let shown = dialogues(&extract_subtitles(&out));
    assert_eq!(shown.len(), 1);
    assert_eq!((shown[0].start, shown[0].end), (0, 600));
    assert!(shown[0].lines[0].contains("0.10"), "{:?}", shown[0].lines);
    assert!(shown[0].lines[2].contains("n/a"), "{:?}", shown[0].lines);
    // The photograph: a JPEG of the frame's size, marked as a 360-degree one.
    let jpeg = std::fs::read(&jpg).expect("a photograph");
    assert_photo_sphere(&jpeg, 128, 64);
    let report = probe(&jpg);
    assert!(report.contains("codec_name=\"mjpeg\""), "{report}");
    assert_eq!(
        scratch.listing(),
        ["bundle", "map.exr", "view.jpg", "view.mkv"]
    );
    // A frame past the film's end is refused before anything is written.
    let past = render(
        &bundle,
        &map,
        &scratch.path().join("x.mkv"),
        &["--still", "7"],
    );
    assert!(past.unwrap_err().contains("past the end"));
    assert_eq!(
        scratch.listing(),
        ["bundle", "map.exr", "view.jpg", "view.mkv"]
    );
}
