//! Marks and display units: signs that are small, hollow and where the bundle says, at any
//! direction; directions that turn along great circles between frames; read-outs shown in the unit
//! the bundle offers; a still that shows only what has a value; and runs without any of it that
//! are what they always were.
//!
//! Directions are taken from `sky_format::Grid`, the format's own statement of the grid formulae,
//! and angles are measured on the sphere with `crate::field::angle`, not with the code under test.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use readout::Style;
use sky_format::{
    BundleWriter, Display, FORMAT, FarSky, Frame, FrameEntry, Geometry, Grid, GridSpec, Manifest,
    MarkDecl, Num, Playback, ReadoutDecl, Source, TimeUnit, VERSION, WriterInfo, fate,
};

use crate::cli::Request;
use crate::field::angle;
use crate::marks::{
    self, DEFAULT_COLOUR, DEFAULT_SIZE_DEGREES, Marks, OUTLINE_PER_SIZE, Pen, STROKE_PER_SIZE,
    Shape, Sign, check_colour, colour_name, film_line, heading_elevation, still_lines, turn,
};
use crate::overlay::{Overlay, Settings, shown_lines};
use crate::panel::Placement;
use crate::render::Size;
use crate::subtitles::{self, Look};
use crate::timeline::{Pick, Timeline};
use crate::values::{Series, Units, format};

// ---- helpers -------------------------------------------------------------------------------------

fn size(width: usize) -> Size {
    Size {
        width,
        height: width / 2,
    }
}

/// The direction of heading `h` (to the right) and elevation `e`, in degrees.
fn towards(h: f64, e: f64) -> [f64; 3] {
    let (h, e) = (h.to_radians(), e.to_radians());
    [e.cos() * h.cos(), -e.cos() * h.sin(), e.sin()]
}

/// Each pixel a sign touches, with its direction, its dark ink and its colour ink.
fn inked(sign: &Sign, size: Size) -> Vec<(usize, usize, [f64; 3], f32, f32)> {
    let grid = Grid::new(size.width as u32, size.height as u32);
    sign.cover(size)
        .into_iter()
        .map(|ink| {
            let (i, j) = (
                ink.pixel as usize % size.width,
                ink.pixel as usize / size.width,
            );
            (
                i,
                j,
                grid.pixel_direction(i as u32, j as u32),
                ink.dark,
                ink.colour,
            )
        })
        .collect()
}

fn decl(id: &str, label: &str, unit: &str, decimals: u32) -> ReadoutDecl {
    ReadoutDecl {
        id: id.into(),
        label: label.into(),
        unit: unit.into(),
        decimals,
        display: None,
    }
}

fn shown(
    id: &str,
    label: &str,
    unit: &str,
    decimals: u32,
    display: (&str, f64, u32),
) -> ReadoutDecl {
    ReadoutDecl {
        display: Some(Display {
            unit: display.0.into(),
            scale: Num(display.1),
            decimals: display.2,
        }),
        ..decl(id, label, unit, decimals)
    }
}

fn values(pairs: &[(&str, f64)]) -> BTreeMap<String, Num> {
    pairs
        .iter()
        .map(|(id, v)| (id.to_string(), Num(*v)))
        .collect()
}

/// A stopwatch in M shown in seconds, a radius in M shown in million km, and a speed with no
/// display, over three bundle frames; the speed is missing from the last.
fn kerr_decls() -> Vec<ReadoutDecl> {
    vec![
        shown("stopwatch", "Stopwatch", "M", 3, ("s", 20.44, 2)),
        shown("r", "Radius", "M", 3, ("million km", 6.128, 3)),
        decl("v", "Speed past the static observer", "c", 4),
    ]
}

fn kerr_frames() -> Vec<BTreeMap<String, Num>> {
    vec![
        values(&[("stopwatch", 0.0), ("r", 2.0), ("v", 0.5)]),
        values(&[("stopwatch", 0.1), ("r", 1.9), ("v", 0.6)]),
        values(&[("stopwatch", 0.2), ("r", 1.8)]),
    ]
}

fn kerr_series(units: Units) -> Series {
    let frames = kerr_frames();
    Series::in_units(&kerr_decls(), &frames.iter().collect::<Vec<_>>(), units)
}

fn settings(line_degrees: f64) -> Settings {
    Settings {
        placements: vec![Placement::DEFAULT],
        line_degrees,
        style: Style::POINT,
    }
}

fn look() -> Look {
    Look {
        font: subtitles::DEFAULT_FONT.into(),
        percent: subtitles::DEFAULT_PERCENT,
        style: Style::POINT,
    }
}

/// The lines of every dialogue of a script, as a player shows them (hard spaces as spaces).
fn cue_lines(script: &str) -> Vec<Vec<String>> {
    script
        .lines()
        .filter_map(|l| l.strip_prefix("Dialogue: "))
        .map(|rest| {
            let text = rest.splitn(10, ',').nth(9).expect("a text field");
            text.split("\\N").map(|l| l.replace("\\h", " ")).collect()
        })
        .collect()
}

fn parse(extra: &[&str]) -> Result<crate::cli::Options, String> {
    let mut args: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--encoder", "none"]
        .map(String::from)
        .to_vec();
    args.extend(extra.iter().map(|s| s.to_string()));
    match crate::cli::parse(&args)? {
        Request::Render(o) => Ok(*o),
        Request::Help => panic!("not help"),
    }
}

// ---- display units -------------------------------------------------------------------------------

#[test]
fn test_a_read_out_with_a_display_is_shown_scaled_in_its_display_unit_and_decimals() {
    let s = kerr_series(Units::Display);
    assert_eq!(s.lines[1].unit, "million km");
    assert_eq!(s.lines[1].decimals, 3);
    assert_eq!(s.lines[2].unit, "c", "a line without a display changed");
    assert_eq!(s.lines[2].scale, 1.0);
    let at = s.at(Pick::One(0), 0.0);
    assert_eq!(at[1], Some(2.0 * 6.128));
    assert_eq!(at[2], Some(0.5));
    // Interpolated as stored, then scaled: a quarter of the way from 2 to 1.9 M, in million km.
    let between = s.at(Pick::Two(0, 1, 0.25), 0.025)[1].unwrap();
    assert_eq!(between, (2.0 + 0.25 * (1.9 - 2.0)) * 6.128);
    // The stopwatch, from the timeline in M, is shown in seconds.
    assert_eq!(s.at(Pick::One(1), 0.1)[0], Some(0.1 * 20.44));
    // And the values that size the column are the displayed ones.
    let widest: Vec<f64> = s.values_of(1).collect();
    assert_eq!(widest, vec![2.0 * 6.128, 1.9 * 6.128, 1.8 * 6.128]);
    // Non-finite values stay what they are.
    let odd = Series::new(
        &kerr_decls(),
        &[&values(&[("r", f64::NAN), ("v", f64::NEG_INFINITY)])],
    );
    let at = odd.at(Pick::One(0), 0.0);
    assert!(at[1].unwrap().is_nan());
    assert_eq!(at[2], Some(f64::NEG_INFINITY));
    assert_eq!(format(Style::POINT, at[1].unwrap(), 3), "n/a");
}

#[test]
fn test_units_stored_shows_every_read_out_as_the_bundle_stores_it() {
    let s = kerr_series(Units::Stored);
    for (line, d) in s.lines.iter().zip(kerr_decls()) {
        assert_eq!(line.unit, d.unit, "{}", d.label);
        assert_eq!(line.decimals, d.decimals as usize, "{}", d.label);
        assert_eq!(line.scale, 1.0, "{}", d.label);
    }
    assert_eq!(
        s.at(Pick::One(0), 0.1),
        vec![Some(0.1), Some(2.0), Some(0.5)]
    );
    assert_eq!(parse(&[]).unwrap().units, Units::Display);
    assert_eq!(parse(&["--units", "stored"]).unwrap().units, Units::Stored);
    let refused = parse(&["--units", "si"]).unwrap_err();
    assert!(
        refused.contains("--units is display or stored"),
        "{refused}"
    );
}

#[test]
fn test_the_panels_value_column_is_as_wide_as_the_widest_displayed_value() {
    let size = size(4096);
    // Stored, the radius is at most 2.000 (five characters); displayed, 12.256 (six): the
    // displayed panel's value column, and so the panel, is wider by at least one digit.
    // The stopwatch and the radius alone: the speed's 0.6000 would be the widest either way.
    let frames = kerr_frames();
    let two = |units| {
        Series::in_units(
            &kerr_decls()[..2],
            &frames.iter().collect::<Vec<_>>(),
            units,
        )
    };
    let stored =
        Overlay::new(&settings(2.0), &two(Units::Stored), [0.0, 0.2], size).expect("a panel");
    let displayed =
        Overlay::new(&settings(2.0), &two(Units::Display), [0.0, 0.2], size).expect("a panel");
    let digit = crate::layout::Metrics::digit_width(displayed.fonts());
    let fonts = displayed.fonts();
    let unit_gap = |o: &Overlay, unit: &str| {
        o.layout().width as f32 - o.layout().unit_x() - crate::layout::width_of(fonts, unit)
    };
    // The unit column is as wide as its widest unit, so compare where it starts.
    assert!(
        displayed.layout().unit_x() >= stored.layout().unit_x() + digit - 0.5,
        "the displayed value column starts its unit at {} and the stored one at {}",
        displayed.layout().unit_x(),
        stored.layout().unit_x()
    );
    assert!(unit_gap(&displayed, "million km") >= 0.0);
    // A stopwatch in seconds past 10 is a digit wider than in M: the span is scaled too.
    let decls = [shown("stopwatch", "T", "M", 2, ("s", 20.44, 2))];
    let only = Series::new(&decls, &[&values(&[("stopwatch", 0.0)])]);
    let short = Overlay::new(&settings(2.0), &only, [0.0, 0.4], size).expect("a panel");
    let unscaled = Series::in_units(&decls, &[&values(&[("stopwatch", 0.0)])], Units::Stored);
    let bare = Overlay::new(&settings(2.0), &unscaled, [0.0, 0.4], size).expect("a panel");
    // 0.4 M is 0.40 stored and 8.18 s displayed: the same width. 0.6 M is 12.26 s.
    assert_eq!(short.layout().unit_x(), bare.layout().unit_x());
    let long = Overlay::new(&settings(2.0), &only, [0.0, 0.6], size).expect("a panel");
    assert!(
        long.layout().unit_x() > short.layout().unit_x(),
        "a stopwatch reaching 12.26 s did not widen the value column"
    );
}

#[test]
fn test_the_subtitles_show_displayed_values_in_columns_as_wide_as_the_displayed_values() {
    let frames = kerr_frames();
    let s = Series::new(&kerr_decls(), &frames.iter().collect::<Vec<_>>());
    let t = Timeline::new(&[0.0, 0.1, 0.2], 0.0, 30.0, 1.0);
    let cues = subtitles::film_cues(&s, &t, 0..t.video_frames(), 30.0);
    let script = subtitles::script(&s.lines, &cues, &look());
    let shown = cue_lines(&script);
    // Frame 3 is on bundle frame 1: 0.1 M is 2.04 s, 1.9 M is 11.643 million km.
    let text = shown[3].join(" | ");
    assert!(text.contains("2.04 s"), "{text}");
    assert!(text.contains("11.643 million km"), "{text}");
    assert!(text.contains("0.6000 c"), "{text}");
    // Every line is one length: the value column is as wide as the widest displayed value
    // (12.256, six characters), and the unit column as the widest displayed unit.
    let length = shown[0][0].chars().count();
    for line in shown.iter().flatten() {
        assert_eq!(line.chars().count(), length, "{line:?}");
    }
    let radius = shown[0].iter().find(|l| l.starts_with("Radius")).unwrap();
    assert!(radius.contains(" 12.256 million km"), "{radius:?}");
    // With the units as stored the radius column is five wide and the unit is M.
    let stored = kerr_series(Units::Stored);
    let cues = subtitles::film_cues(&stored, &t, 0..t.video_frames(), 30.0);
    let shown = cue_lines(&subtitles::script(&stored.lines, &cues, &look()));
    let radius = shown[0].iter().find(|l| l.starts_with("Radius")).unwrap();
    assert!(radius.contains(" 2.000 M"), "{radius:?}");
    assert!(!radius.contains("million"));
}

// ---- a still shows only what has a value ---------------------------------------------------------

#[test]
fn test_a_stills_panel_and_cue_leave_out_a_read_out_without_a_value_and_a_films_keep_its_line() {
    let s = kerr_series(Units::Display);
    // At bundle frame 2 the speed has no value.
    let still = s.at(Pick::One(2), 0.2);
    assert_eq!(still[2], None);
    assert_eq!(shown_lines(&s, Some(&still)), vec![1]);
    assert_eq!(shown_lines(&s, None), vec![0, 1, 2]);
    let at_first = s.at(Pick::One(0), 0.0);
    assert_eq!(shown_lines(&s, Some(&at_first)), vec![1, 2]);
    // The still's panel is the radius's alone; the film's has all three lines.
    let o = parse(&["--still", "6", "--readouts", "panel", "--size", "2048x1024"]).unwrap();
    let panel = Overlay::for_run(&o, &s, Some(&still), [0.2, 0.2])
        .expect("no refusal")
        .expect("a panel");
    assert_eq!(panel.line_count(), 1);
    let o = parse(&["--readouts", "panel", "--size", "2048x1024"]).unwrap();
    let film = Overlay::for_run(&o, &s, None, [0.0, 0.2])
        .expect("no refusal")
        .expect("a panel");
    assert_eq!(film.line_count(), 3);
    // The still's cue: no speed, label and all, and its label does not widen the label column.
    let t = Timeline::new(&[0.0, 0.1, 0.2], 0.0, 30.0, 1.0);
    let cue = subtitles::still_cue(&s, &t, 6, 60.0);
    let (lines, cue) = subtitles::still_only(&s.lines, &cue);
    assert_eq!(
        lines.iter().map(|l| l.label.as_str()).collect::<Vec<_>>(),
        ["Stopwatch", "Radius"],
        "the still keeps a line with no value, or lost one with a value"
    );
    let shown = cue_lines(&subtitles::script(&lines, &[cue], &look()));
    assert_eq!(shown.len(), 1);
    assert_eq!(shown[0].len(), 1, "{:?}", shown[0]);
    assert!(shown[0][0].starts_with("Radius  "), "{:?}", shown[0]);
    assert!(!shown[0][0].contains("static"), "{:?}", shown[0]);
    // A film's cue on that frame keeps the line's place in the columns and simply has no value.
    let cues = subtitles::film_cues(&s, &t, 0..t.video_frames(), 30.0);
    let film = cue_lines(&subtitles::script(&s.lines, &cues, &look()));
    assert_eq!(film[6].len(), 2, "{:?}", film[6]);
    assert_eq!(film[0].len(), 3, "{:?}", film[0]);
    assert_eq!(film[0][0].chars().count(), film[6][0].chars().count());
}

// ---- the signs -----------------------------------------------------------------------------------

#[test]
fn test_each_sign_is_hollow_within_its_size_and_symmetric_and_the_triangle_points_down() {
    // A sign at heading 0 on the equator sits on the corner of four pixels, so the frame's
    // mirror images about that point are the sign's own.
    let size = size(4096);
    let (w, h) = (size.width, size.height);
    let pixel = 360.0 / w as f64;
    let s = DEFAULT_SIZE_DEGREES;
    let centre = towards(0.0, 0.0);
    for shape in [Shape::Ring, Shape::Diamond, Shape::Triangle] {
        let sign = Sign::new(shape, centre, s);
        let ink = inked(&sign, size);
        assert!(ink.len() > 200, "{shape:?} touches {} pixels", ink.len());
        let mut dark = vec![0f32; w * h];
        for &(i, j, n, d, c) in &ink {
            let off = angle(n, centre).to_degrees();
            assert!(
                off <= s + 1e-9,
                "{shape:?}: pixel ({i}, {j}) is inked {off:.4} degrees from the centre"
            );
            assert!(
                c <= d + 1e-6,
                "{shape:?}: the stroke is outside the outline"
            );
            dark[j * w + i] = d;
        }
        // Hollow: nothing within the inradius less the bands, and the centre's pixels clear.
        let bands = (2.0 * OUTLINE_PER_SIZE + STROKE_PER_SIZE) * s + pixel;
        let inradius = match shape {
            Shape::Ring => s,
            Shape::Diamond => s / 2f64.sqrt(),
            Shape::Triangle => s / 2.0,
        };
        for &(i, j, n, ..) in &ink {
            let off = angle(n, centre).to_degrees();
            assert!(
                off >= inradius - bands - 1e-9,
                "{shape:?}: pixel ({i}, {j}) is inked {off:.3} degrees from the centre, inside \
                 the hollow"
            );
        }
        assert!(
            inradius - bands > 0.3,
            "{shape:?}: the hollow is under 0.3 degree"
        );
        // Symmetric left and right; the ring and the diamond up and down too.
        for &(i, j, _, d, _) in &ink {
            let mirror = dark[j * w + (w - 1 - i)];
            assert!(
                (mirror - d).abs() < 1e-3,
                "{shape:?}: pixel ({i}, {j}) has {d} and its mirror {mirror}"
            );
            if shape != Shape::Triangle {
                let flipped = dark[(h - 1 - j) * w + i];
                assert!(
                    (flipped - d).abs() < 1e-3,
                    "{shape:?}: pixel ({i}, {j}) has {d} and its reflection {flipped}"
                );
            }
        }
        // It reaches out to its size: the outermost inked pixel is within two pixels of it.
        let reach = ink
            .iter()
            .map(|p| angle(p.2, centre).to_degrees())
            .fold(0.0, f64::max);
        assert!(
            reach > s - 2.0 * pixel,
            "{shape:?} reaches only {reach:.3} of {s} degrees"
        );
        if shape == Shape::Triangle {
            // Point down: the lowest ink is a corner, the size below; the highest an edge, half
            // the size above.
            let rows: Vec<usize> = ink.iter().map(|p| p.1).collect();
            let (top, bottom) = (*rows.iter().min().unwrap(), *rows.iter().max().unwrap());
            let above = (h / 2 - top) as f64 * pixel;
            let below = (bottom + 1 - h / 2) as f64 * pixel;
            assert!(
                below > 1.6 * above,
                "the triangle reaches {above:.2} degrees up and {below:.2} down: not point down"
            );
        }
    }
    // A shape the renderer does not know is a ring.
    assert_eq!(Shape::of("star"), Shape::Ring);
    assert_eq!(Shape::of("diamond").name(), "diamond");
}

/// The solid angle of a sign's ink, in square degrees: each pixel's ink times its area, which
/// shrinks with the cosine of its latitude.
fn inked_area(sign: &Sign, size: Size) -> f64 {
    let pixel = 360.0 / size.width as f64;
    inked(sign, size)
        .iter()
        .map(|&(_, _, n, d, _)| f64::from(d) * pixel * pixel * n[0].hypot(n[1]))
        .sum()
}

#[test]
fn test_a_sign_is_drawn_whole_across_the_seam_and_over_either_pole() {
    let size = size(2048);
    let s = DEFAULT_SIZE_DEGREES;
    let reference = inked_area(&Sign::new(Shape::Ring, towards(0.0, 0.0), s), size);
    for (what, n) in [
        ("behind, at heading 180", towards(180.0, 0.0)),
        ("at the zenith", [0.0, 0.0, 1.0]),
        ("at the nadir", [0.0, 0.0, -1.0]),
        ("a degree from the zenith", towards(33.0, 89.0)),
        ("on the seam, low", towards(-180.0, -40.0)),
    ] {
        let sign = Sign::new(Shape::Ring, n, s);
        let ink = inked(&sign, size);
        for &(i, j, m, ..) in &ink {
            let off = angle(m, n).to_degrees();
            assert!(
                off <= s + 1e-9,
                "{what}: pixel ({i}, {j}) is {off:.3} degrees off"
            );
        }
        let area = inked_area(&sign, size);
        assert!(
            (area / reference - 1.0).abs() < 0.03,
            "{what}: the sign covers {area:.4} square degrees, and on the equator {reference:.4}"
        );
    }
    // Behind the observer the sign is split by the seam and whole: ink at both edges, centred
    // on the seam.
    let ink = inked(&Sign::new(Shape::Diamond, towards(180.0, 0.0), s), size);
    let left = ink.iter().filter(|p| p.0 < 100).count();
    let right = ink.iter().filter(|p| p.0 > size.width - 100).count();
    assert!(
        left > 50 && left == right,
        "{left} pixels at the left edge, {right} at the right"
    );
    // At a pole every column meets the sign.
    let ink = inked(&Sign::new(Shape::Triangle, [0.0, 0.0, -1.0], s), size);
    let mut columns: Vec<usize> = ink.iter().map(|p| p.0).collect();
    columns.sort_unstable();
    columns.dedup();
    assert!(
        columns.len() > size.width / 2,
        "{} columns at the nadir",
        columns.len()
    );
    // And the heading of a direction at 180 is written 180, never -180.
    assert_eq!(heading_elevation(towards(180.0, 0.0)).0, 180.0);
    assert_eq!(heading_elevation(towards(-180.0, 0.0)).0, 180.0);
    let (h, e) = heading_elevation(towards(-90.0, 12.5));
    assert!((h + 90.0).abs() < 1e-9 && (e - 12.5).abs() < 1e-9);
}

#[test]
fn test_painting_touches_only_the_signs_pixels_and_draws_them_in_the_mark_colour() {
    let size = size(4096);
    let base: Vec<u16> = (0..size.width * size.height * 3)
        .map(|k| (k * 7919 % 65_536) as u16)
        .collect();
    let signs = [
        Sign::new(Shape::Ring, towards(20.0, 0.0), 3.0),
        Sign::new(Shape::Diamond, towards(21.0, 1.0), 3.0),
    ];
    let mut frame = base.clone();
    marks::paint(&mut frame, size, &signs, DEFAULT_COLOUR);
    let grid = Grid::new(size.width as u32, size.height as u32);
    let mut full = 0;
    for k in 0..size.width * size.height {
        if frame[3 * k..3 * k + 3] == base[3 * k..3 * k + 3] {
            continue;
        }
        let n = grid.pixel_direction((k % size.width) as u32, (k / size.width) as u32);
        let near = signs
            .iter()
            .any(|s| angle(n, s.centre()) <= 3f64.to_radians());
        assert!(near, "pixel {k} changed and is in no sign");
        if frame[3 * k..3 * k + 3] == DEFAULT_COLOUR {
            full += 1;
        }
    }
    assert!(full > 100, "only {full} pixels are wholly the mark colour");
    // The two overlap and both are drawn, the diamond (declared second) over the ring: where
    // the diamond's stroke is solid its colour is there whatever the ring did.
    let diamond = signs[1].cover(size);
    for ink in diamond.iter().filter(|i| i.colour >= 1.0) {
        let at = 3 * ink.pixel as usize;
        assert_eq!(frame[at..at + 3], DEFAULT_COLOUR);
    }
}

// ---- directions between frames -------------------------------------------------------------------

#[test]
fn test_a_mark_turns_along_the_great_circle_by_the_rays_weight_or_shows_the_nearer_frame() {
    let a = towards(10.0, 5.0);
    let b = towards(70.0, -30.0);
    let theta = angle(a, b);
    let normal = {
        let c = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let l = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        c.map(|x| x / l)
    };
    for w in [0.1, 0.25, 0.5, 0.9] {
        let n = turn(Some(a), Some(b), w).expect("both given");
        let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!((length - 1.0).abs() < 1e-12, "w {w}: length {length}");
        let off_circle = n.iter().zip(normal).map(|(x, y)| x * y).sum::<f64>();
        assert!(
            off_circle.abs() < 1e-12,
            "w {w}: {off_circle} off the great circle"
        );
        assert!(
            (angle(a, n) - w * theta).abs() < 1e-12
                && (angle(n, b) - (1.0 - w) * theta).abs() < 1e-12,
            "w {w}: not {w} of the way along"
        );
    }
    // Only one given: the nearer frame's, as a read-out's (exactly half way is the second's).
    assert_eq!(turn(Some(a), None, 0.4), Some(a));
    assert_eq!(turn(Some(a), None, 0.5), None);
    assert_eq!(turn(None, Some(b), 0.5), Some(b));
    assert_eq!(turn(None, Some(b), 0.3), None);
    assert_eq!(turn(None, None, 0.3), None);
    // Opposite directions have no one great circle: the nearer.
    let back = a.map(|c| -c);
    let near = |n: Option<[f64; 3]>, m: [f64; 3]| angle(n.expect("a direction"), m) < 1e-12;
    assert!(near(turn(Some(a), Some(back), 0.3), a));
    assert!(near(turn(Some(a), Some(back), 0.7), back));
    // Through the marks of three bundle frames, the second omitting the diamond.
    let decls = [
        MarkDecl {
            id: "s".into(),
            label: "past the static observer".into(),
            shape: "ring".into(),
        },
        MarkDecl {
            id: "z".into(),
            label: "past the ZAMO".into(),
            shape: "diamond".into(),
        },
    ];
    let frame = |pairs: &[(&str, [f64; 3])]| -> BTreeMap<String, [Num; 3]> {
        pairs
            .iter()
            .map(|(id, n)| (id.to_string(), n.map(Num)))
            .collect()
    };
    let frames = [
        frame(&[("s", a), ("z", a)]),
        frame(&[("s", b)]),
        frame(&[("s", a), ("z", b)]),
    ];
    let m = Marks::new(&decls, &frames.iter().collect::<Vec<_>>());
    assert_eq!(m.at(Pick::One(1)), vec![Some(b), None]);
    let at = m.at(Pick::Two(0, 1, 0.25));
    assert_eq!(at[0], turn(Some(a), Some(b), 0.25));
    assert_eq!(
        at[1],
        Some(a),
        "the diamond is not drawn from the nearer frame"
    );
    assert_eq!(m.at(Pick::Two(0, 1, 0.75))[1], None);
    assert_eq!(m.at(Pick::Two(1, 2, 0.75))[1], Some(b));
}

// ---- what the run says, and the command line -----------------------------------------------------

#[test]
fn test_the_run_names_each_mark_of_a_still_and_counts_a_films_in_the_words_the_caller_reads() {
    let decls = [
        MarkDecl {
            id: "s".into(),
            label: "direction of travel past the static observer".into(),
            shape: "ring".into(),
        },
        MarkDecl {
            id: "z".into(),
            label: "direction of travel past the ZAMO".into(),
            shape: "hexagon".into(),
        },
        MarkDecl {
            id: "t".into(),
            label: "absent".into(),
            shape: "triangle".into(),
        },
    ];
    let m = Marks::new(&decls, &[&BTreeMap::new()]);
    let pen = Pen {
        size_degrees: 1.5,
        colour: DEFAULT_COLOUR,
    };
    let lines = still_lines(
        &m,
        &[Some(towards(-180.0, 0.0)), Some(towards(12.25, -3.5)), None],
        pen,
    );
    assert_eq!(
        lines,
        [
            "mark: a ring in green at 180 degrees right of the opening view and 0 degrees up: \
             direction of travel past the static observer",
            "mark: a ring in green at 12.25 degrees right of the opening view and -3.5 degrees \
             up: direction of travel past the ZAMO",
        ]
    );
    assert_eq!(
        film_line(&m, pen).as_deref(),
        Some("marks: 3 declared, drawn in green")
    );
    let magenta = Pen {
        colour: [65_535, 0, 65_535],
        ..pen
    };
    assert_eq!(colour_name(magenta.colour), "#FF00FF");
    assert_eq!(
        film_line(&m, magenta).as_deref(),
        Some("marks: 3 declared, drawn in #FF00FF")
    );
    assert_eq!(film_line(&Marks::new(&[], &[&BTreeMap::new()]), pen), None);
}

#[test]
fn test_the_mark_options_parse_and_each_refusal_is_a_sentence_naming_its_flag() {
    let o = parse(&[]).unwrap();
    assert!(o.marks);
    assert_eq!(o.mark_size, 1.5);
    assert_eq!(o.mark_colour, DEFAULT_COLOUR);
    assert_eq!(
        DEFAULT_COLOUR,
        crate::tone::parse_hex_colour("33FF66").unwrap()
    );
    let o = parse(&[
        "--marks",
        "off",
        "--mark-size",
        "2.5",
        "--mark-colour",
        "#FFAA00",
    ])
    .unwrap();
    assert!(!o.marks);
    assert_eq!(o.mark_size, 2.5);
    assert_eq!(o.mark_colour, [65_535, 0xAA * 257, 0]);
    for (args, flag) in [
        (&["--marks", "maybe"][..], "--marks is on or off"),
        (&["--mark-size", "0"][..], "--mark-size"),
        (&["--mark-size", "5.5"][..], "at most 5"),
        (&["--mark-size", "nan"][..], "--mark-size"),
        (
            &["--mark-colour", "green"][..],
            "--mark-colour takes six hex digits",
        ),
        (&["--mark-size"][..], "--mark-size needs a value"),
    ] {
        let refused = parse(args).unwrap_err();
        assert!(refused.contains(flag), "{args:?}: {refused}");
    }
}

#[test]
fn test_a_mark_colour_that_is_the_colour_of_unknown_pixels_is_refused() {
    let red = [65_535, 0, 0];
    let refused = parse(&["--mark-colour", "FF0000"]).unwrap_err();
    assert!(
        refused.contains("--mark-colour FF0000") && refused.contains("--unresolved-colour"),
        "{refused}"
    );
    // The default green, when the unknown colour is made green.
    let refused = parse(&["--unresolved-colour", "33FF66"]).unwrap_err();
    assert!(refused.contains("--mark-colour 33FF66"), "{refused}");
    let refused = parse(&[
        "--unresolved-colour",
        "0000FF",
        "--undersampled-colour",
        "33ff66",
    ])
    .unwrap_err();
    assert!(refused.contains("--undersampled-colour"), "{refused}");
    // Under-sampled pixels not marked: their colour is not in force.
    assert!(
        parse(&[
            "--undersampled",
            "interpolate",
            "--undersampled-colour",
            "33FF66"
        ])
        .is_ok()
    );
    // No marks, no sign to confuse.
    assert!(parse(&["--marks", "off", "--mark-colour", "FF0000"]).is_ok());
    assert!(check_colour(DEFAULT_COLOUR, red, Some(red)).is_ok());
    assert!(check_colour(red, [0, 0, 65_535], None).is_ok());
    assert!(check_colour(red, [0, 0, 65_535], Some(red)).is_err());
}

// ---- runs without marks are what they were --------------------------------------------------------

/// A bundle of two frames on a 32 x 16 grid, every ray from the far sky along its own direction,
/// with the given marks declared and given at both frames, and a 64 x 32 map; their paths.
fn write_bundle(dir: &Path, marks: &[(&str, &str, [f64; 3])]) -> (PathBuf, PathBuf) {
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
            name: Some("marks".into()),
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
        grid: GridSpec::new(32, 16),
        far_sky: FarSky::galactic(),
        playback: Playback {
            frames_per_second: Num(30.0),
            proper_time_per_video_second: Num(1.0),
        },
        readouts: vec![decl("stopwatch", "Stopwatch", "s", 2)],
        labels: Vec::new(),
        marks: marks
            .iter()
            .map(|(id, shape, _)| MarkDecl {
                id: id.to_string(),
                label: id.to_string(),
                shape: shape.to_string(),
            })
            .collect(),
        frames_planned: Some(2),
        frames: Vec::new(),
    };
    let mut writer = BundleWriter::create(&bundle, manifest).expect("a bundle");
    let grid = Grid::new(32, 16);
    for index in 0..2u32 {
        let mut frame = Frame::new(32, 16, index);
        for j in 0..16 {
            for i in 0..32 {
                let k = grid.offset(i, j);
                frame.fate[k] = fate::FAR_SKY;
                for (plane, c) in frame.direction.iter_mut().zip(grid.pixel_direction(i, j)) {
                    plane[k] = c as f32;
                }
                frame.shift[k] = 1.0;
            }
        }
        let mut entry = FrameEntry::new(index, f64::from(index) * 0.1);
        entry.readouts = values(&[("stopwatch", f64::from(index) * 0.1)]);
        for (id, _, n) in marks {
            entry = entry.with_mark(id, *n);
        }
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

/// Renders video frames 0..4 at 256 x 128 with `extra`, keeping them as PNGs in `frames`, and
/// returns the PNGs' bytes.
fn render_frames(bundle: &Path, map: &Path, frames: &Path, extra: &[&str]) -> Vec<Vec<u8>> {
    let _ = std::fs::remove_dir_all(frames);
    let mut a: Vec<String> = [
        "--bundle",
        &bundle.to_string_lossy(),
        "--sky",
        &map.to_string_lossy(),
        "--encoder",
        "none",
        "--size",
        "256x128",
        "--threads",
        "2",
        "--frames",
        "0..4",
        "--keep-frames",
        &frames.to_string_lossy(),
    ]
    .map(String::from)
    .to_vec();
    a.extend(extra.iter().map(|s| s.to_string()));
    let Request::Render(o) = crate::cli::parse(&a).expect("parses") else {
        panic!("not a render")
    };
    crate::run(&o).expect("the frames are rendered");
    (0..4)
        .map(|k| std::fs::read(frames.join(format!("frame_{k:06}.png"))).expect("a frame"))
        .collect()
}

#[test]
fn test_marks_off_or_a_bundle_without_marks_gives_the_frames_it_always_gave() {
    let dir = std::env::temp_dir()
        .join("sky-render-tests")
        .join(format!("{}-marks-off", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let (plain, map) = write_bundle(&dir.join("plain"), &[]);
    let (marked, _) = write_bundle(
        &dir.join("marked"),
        &[
            ("s", "ring", towards(20.0, 0.0)),
            ("z", "diamond", towards(-60.0, 30.0)),
        ],
    );
    let panel = ["--readouts", "panel"];
    let without = render_frames(&plain, &map, &dir.join("a"), &panel);
    let off = render_frames(
        &marked,
        &map,
        &dir.join("b"),
        &["--readouts", "panel", "--marks", "off"],
    );
    let on = render_frames(&marked, &map, &dir.join("c"), &panel);
    // `--marks on` is the default, and a bundle that declares none is drawn with none.
    let plain_on = render_frames(
        &plain,
        &map,
        &dir.join("d"),
        &["--readouts", "panel", "--marks", "on"],
    );
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(off, without, "--marks off changed the frames");
    assert_eq!(plain_on, without, "a bundle without marks drew differently");
    for (k, (a, b)) in on.iter().zip(&without).enumerate() {
        assert_ne!(a, b, "frame {k}: the marks drew nothing");
    }
}

#[test]
fn test_the_mark_size_scales_the_whole_sign() {
    let size = size(4096);
    let small = inked_area(&Sign::new(Shape::Ring, towards(0.0, 0.0), 1.5), size);
    let large = inked_area(&Sign::new(Shape::Ring, towards(0.0, 0.0), 3.0), size);
    // The stroke and outlines scale with the size, so the ink scales as the size squared.
    assert!((large / small - 4.0).abs() < 0.2, "{large} over {small}");
    let o = parse(&["--mark-size", "3"]).unwrap();
    assert_eq!(o.mark_size, 3.0);
}

// ---- the degree sign -----------------------------------------------------------------------------

#[test]
fn test_a_unit_beginning_with_the_degree_sign_follows_its_number_with_no_space_in_panel_and_cue() {
    assert!(crate::layout::hangs("° right of the hole"));
    assert!(crate::layout::hangs("°"));
    for unit in ["c", "M", "", "million km", " °", "deg"] {
        assert!(!crate::layout::hangs(unit), "{unit:?} hangs");
    }
    let decls = [
        decl("stopwatch", "Stopwatch", "s", 2),
        decl("v", "Speed past the ZAMO", "c", 4),
        decl("h", "Heading of travel (diamond)", "° right of the hole", 1),
    ];
    let frames = [values(&[("stopwatch", 0.0), ("v", 0.48), ("h", 44.0)])];
    let s = Series::new(&decls, &frames.iter().collect::<Vec<_>>());

    // The panel: the degree sign starts at the value column's right edge, where the space would
    // be; the other units start a space later, in one column.
    let size = size(4096);
    let o = Overlay::new(&settings(2.0), &s, [0.0, 0.0], size).expect("a panel");
    let (layout, fonts) = (o.layout(), o.fonts());
    let space = crate::layout::Metrics::advance(fonts, ' ');
    assert_eq!(layout.unit_start(2), layout.value_right());
    assert_eq!(layout.unit_start(0), layout.unit_x());
    assert_eq!(layout.unit_start(1), layout.unit_x());
    assert!((layout.unit_x() - layout.value_right() - space).abs() < 1e-4);
    // The whole unit is on the panel: its right edge is inside the panel's width.
    let right = layout.unit_start(2) + crate::layout::width_of(fonts, "° right of the hole");
    assert!(
        right <= layout.width as f32,
        "{right} past a width of {}",
        layout.width
    );
    // And without a hanging unit the layout is what it was: every unit at the column.
    let plain = Series::new(&decls[..2], &[&values(&[("stopwatch", 0.0), ("v", 0.48)])]);
    let p = Overlay::new(&settings(2.0), &plain, [0.0, 0.0], size).expect("a panel");
    assert_eq!(p.layout().unit_start(0), p.layout().unit_x());
    assert_eq!(p.layout().unit_start(1), p.layout().unit_x());

    // The cue: "44.0° right of the hole" with no space; every line one length; the numbers
    // right-aligned in one column; the other units a space after the numbers, in one column.
    let t = Timeline::new(&[0.0], 0.0, 30.0, 1.0);
    let cues = subtitles::film_cues(&s, &t, 0..1, 30.0);
    let shown = cue_lines(&subtitles::script(&s.lines, &cues, &look()));
    let lines = &shown[0];
    assert!(lines[2].ends_with("44.0° right of the hole"), "{lines:?}");
    assert!(lines[1].contains("0.4800 c"), "{lines:?}");
    let length = lines[0].chars().count();
    assert!(
        lines.iter().all(|l| l.chars().count() == length),
        "{lines:?}"
    );
    let column = |l: &str, text: &str| l.find(text).map(|b| l[..b].chars().count());
    let end_of = |l: &str, number: &str| column(l, number).unwrap() + number.chars().count();
    assert_eq!(
        end_of(&lines[1], "0.4800"),
        end_of(&lines[2], "44.0"),
        "{lines:?}"
    );
    assert_eq!(
        end_of(&lines[0], "0.00"),
        end_of(&lines[2], "44.0"),
        "{lines:?}"
    );
    assert_eq!(
        column(&lines[1], "c "),
        column(&lines[0], "s "),
        "{lines:?}"
    );
    // The degree sign is in the column where the other lines have their space.
    assert_eq!(
        column(&lines[2], "°"),
        Some(end_of(&lines[1], "0.4800")),
        "{lines:?}"
    );
}

// ---- a zero has no sign ----------------------------------------------------------------------------

#[test]
fn test_a_value_that_rounds_to_zero_is_written_without_a_minus_sign() {
    for style in [Style::POINT, Style::COMMA] {
        let zero = format(style, 0.0, 1);
        assert_eq!(format(style, -0.0, 1), zero);
        assert_eq!(format(style, -4e-7, 1), zero);
        assert_eq!(format(style, -0.04, 1), zero);
        assert_eq!(format(style, -0.0, 0), "0");
        assert_eq!(format(style, -0.4, 0), "0");
        // Decided by the printed digits: -0.05 at one place prints -0.1 (or would keep its sign
        // if it printed any digit but zero), -0.06 certainly does.
        let half = format(style, -0.05, 1);
        assert!(
            half == zero || half.starts_with('-') && half.chars().any(|c| ('1'..='9').contains(&c)),
            "{half:?}"
        );
        assert!(
            format(style, -0.06, 1).starts_with("-0"),
            "{}",
            format(style, -0.06, 1)
        );
        // A small value that does not round to zero keeps its sign; one that does, loses it.
        assert!(format(style, -0.004, 3).starts_with('-'));
        assert_eq!(format(style, -0.004, 2), format(style, 0.0, 2));
        assert_eq!(
            format(style, -12.5, 1),
            format!("-{}", format(style, 12.5, 1))
        );
    }
    assert_eq!(format(Style::POINT, -0.04, 1), "0.0");
    assert_eq!(format(Style::COMMA, -0.04, 1), "0,0");
    assert_eq!(format(Style::POINT, -0.05, 1), "-0.1");
    assert_eq!(format(Style::POINT, f64::NAN, 2), "n/a");
    assert_eq!(format(Style::POINT, f64::INFINITY, 2), "∞");
    assert_eq!(format(Style::POINT, f64::NEG_INFINITY, 2), "-∞");
    // And through a cue: a heading of -4e-7 degrees shows 0.0, its column no wider than 0.0's.
    let decls = [
        decl("stopwatch", "Stopwatch", "s", 2),
        decl("h", "Heading", "°", 1),
    ];
    let t = Timeline::new(&[0.0], 0.0, 30.0, 1.0);
    let lines_of = |h: f64| {
        let frames = [values(&[("stopwatch", 0.0), ("h", h)])];
        let s = Series::new(&decls, &frames.iter().collect::<Vec<_>>());
        let cues = subtitles::film_cues(&s, &t, 0..1, 30.0);
        cue_lines(&subtitles::script(&s.lines, &cues, &look())).remove(0)
    };
    assert_eq!(lines_of(-4e-7), lines_of(0.0));
    assert!(
        lines_of(-4e-7)[1].contains(" 0.0° "),
        "{:?}",
        lines_of(-4e-7)
    );
}
