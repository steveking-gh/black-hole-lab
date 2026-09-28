//! The read-outs' promises: panels that read flat and upright on the sphere, values that are the
//! right numbers, and digits that hold still.
//!
//! Frames here are a few hundred pixels wide, so that a panel covers a few thousand of them and
//! every test runs in milliseconds. Directions are taken from `sky_format::Grid`, the format's own
//! statement of the grid formulae, not from the code under test.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use readout::Style;
use sky_format::{Grid, Num, ReadoutDecl};

use crate::layout::{Metrics, Placed};
use crate::overlay::{Overlay, Settings, composite};
use crate::panel::{Panel, Placement};
use crate::render::Size;
use crate::text::{Coverage, Fonts};
use crate::timeline::{Pick, Timeline};
use crate::values::{Series, format, interpolate};

// ---- helpers -------------------------------------------------------------------------------------

fn at(heading: f64, elevation: f64) -> Placement {
    Placement { heading, elevation }
}

/// A frame of `width` x `width / 2` pixels, and the panel pixel that matches its rows.
fn frame(width: usize) -> (Size, f64) {
    (
        Size {
            width,
            height: width / 2,
        },
        2.0 * PI / width as f64,
    )
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The pixel of a `size` frame that looks along `n`, by the specification's inverse formulae.
fn pixel_of(size: Size, n: [f64; 3]) -> (usize, usize) {
    let (u, v) = Grid::new(size.width as u32, size.height as u32).frame_coordinates(n);
    (
        (u.floor() as usize).min(size.width - 1),
        (v.floor() as usize).min(size.height - 1),
    )
}

/// A coverage image `width` x `height` with the rectangle of columns `xs` and rows `ys` inked.
fn marks(
    width: usize,
    height: usize,
    rects: &[(std::ops::Range<usize>, std::ops::Range<usize>)],
) -> Coverage {
    let mut c = Coverage::new(width, height);
    for (xs, ys) in rects {
        for y in ys.clone() {
            for x in xs.clone() {
                c.data[y * width + x] = 1.0;
            }
        }
    }
    c
}

/// Paints `text` through `panel` onto a black frame and returns the pixels the text lit, as
/// (column, row). On black the backing leaves every pixel at 0, so what is lit is the text.
fn lit(panel: &Panel, size: Size, text: &Coverage) -> Vec<(usize, usize)> {
    let mut out = vec![0u16; size.width * size.height * 3];
    composite(&mut out, &panel.cover(size), text);
    (0..size.width * size.height)
        .filter(|&k| out[3 * k] > 30_000)
        .map(|k| (k % size.width, k / size.width))
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

/// The values of one bundle frame, None meaning the frame omits that read-out.
fn values(pairs: &[(&str, Option<f64>)]) -> BTreeMap<String, Num> {
    pairs
        .iter()
        .filter_map(|(id, v)| v.map(|v| (id.to_string(), Num(v))))
        .collect()
}

/// A series of a stopwatch and a radius over the given bundle frames' radii.
fn series(radii: &[Option<f64>]) -> Series {
    let decls = [
        decl("stopwatch", "Stopwatch", "s", 2),
        decl("r", "Radius", "M", 3),
    ];
    let frames: Vec<BTreeMap<String, Num>> = radii
        .iter()
        .enumerate()
        .map(|(k, r)| values(&[("stopwatch", Some(k as f64)), ("r", *r)]))
        .collect();
    Series::new(&decls, &frames.iter().collect::<Vec<_>>())
}

fn settings(placements: &[Placement], line_degrees: f64) -> Settings {
    Settings {
        placements: placements.to_vec(),
        line_degrees,
        style: Style::POINT,
    }
}

// ---- where the panel is --------------------------------------------------------------------------

#[test]
fn test_the_panel_centre_reads_the_panel_centre_and_lands_where_the_specification_puts_it() {
    let (size, pixel) = frame(720);
    for (h, e) in [(0.0, -30.0), (90.0, 10.0), (-135.0, 45.0), (37.5, -62.0)] {
        let panel = Panel::new(at(h, e), pixel, 40, 20, 3.0);
        // The specification's frame coordinates for longitude h (heading is longitude: both
        // increase to the right) and latitude e.
        let (u, v) = (
            ((h / 360.0 + 0.5) * 720.0).rem_euclid(720.0),
            (0.5 - e / 180.0) * 360.0,
        );
        let n = Grid::new(720, 360).direction(u, v);
        let (x, y) = panel
            .locate(n)
            .expect("the centre is in front of the panel");
        assert!(
            (x - 20.0).abs() < 1e-9 && (y - 10.0).abs() < 1e-9,
            "heading {h}, elevation {e}: the centre direction reads ({x}, {y})"
        );
        // The pixel containing (u, v) is covered, and reads within a pixel of the centre.
        let (i, j) = (u.floor() as usize, v.floor() as usize);
        let tap = panel
            .cover(size)
            .into_iter()
            .find(|t| t.pixel as usize == j * 720 + i)
            .unwrap_or_else(|| panic!("heading {h}: pixel ({i}, {j}) is not covered"));
        assert!(
            (f64::from(tap.x) - 20.0).abs() < 1.0 && (f64::from(tap.y) - 10.0).abs() < 1.0,
            "heading {h}: pixel ({i}, {j}) reads ({}, {})",
            tap.x,
            tap.y
        );
        // And the inverse agrees with the forward mapping.
        let back = panel.direction(20.0, 10.0);
        assert!(dot(back, n) > 1.0 - 1e-12);
    }
}

#[test]
fn test_the_left_of_the_panel_is_to_the_left_in_the_frame_and_the_top_is_toward_the_top() {
    let (size, pixel) = frame(720);
    let (w, h) = (40, 20);
    let left = marks(w, h, &[(1..4, 8..12)]);
    let top = marks(w, h, &[(18..22, 1..3)]);
    for heading in [0.0, 90.0, -90.0] {
        let panel = Panel::new(at(heading, -20.0), pixel, w, h, 0.0);
        let (ci, cj) = pixel_of(size, panel.direction(20.0, 10.0));
        let mean = |pixels: &[(usize, usize)]| {
            assert!(
                !pixels.is_empty(),
                "heading {heading}: the mark lit nothing"
            );
            let n = pixels.len() as f64;
            (
                pixels.iter().map(|p| p.0 as f64).sum::<f64>() / n,
                pixels.iter().map(|p| p.1 as f64).sum::<f64>() / n,
            )
        };
        let (li, lj) = mean(&lit(&panel, size, &left));
        let (ti, tj) = mean(&lit(&panel, size, &top));
        // The left mark is 17 to 19 panel pixels left of the centre, at its height.
        assert!(
            li < ci as f64 - 15.0 && (lj - cj as f64).abs() < 3.0,
            "heading {heading}: the left mark is at ({li}, {lj}) and the centre at ({ci}, {cj})"
        );
        // The top mark is 8 panel pixels above the centre, at its column.
        assert!(
            tj < cj as f64 - 6.0 && (ti - ci as f64).abs() < 3.0,
            "heading {heading}: the top mark is at ({ti}, {tj}) and the centre at ({ci}, {cj})"
        );
    }
}

#[test]
fn test_a_straight_line_on_the_panel_lies_on_a_great_circle_of_the_sphere() {
    // A panel 30 degrees by 15, low and to the side, where a straight line on it is a visible
    // curve in the frame.
    let (size, pixel) = frame(1440);
    let (w, h) = (120, 60);
    let panel = Panel::new(at(40.0, -35.0), pixel, w, h, 0.0);
    let line_y = 45.5;
    let line = marks(w, h, &[(4..116, 44..47)]);
    let pixels = lit(&panel, size, &line);
    let mut painted = vec![false; size.width * size.height];
    for &(i, j) in &pixels {
        painted[j * size.width + i] = true;
    }
    // Along the great circle the line should lie on: every point of it is painted.
    let mut rows = Vec::new();
    for k in 0..=50 {
        let x = 10.0 + 2.0 * k as f64;
        let (i, j) = pixel_of(size, panel.direction(x, line_y));
        assert!(
            painted[j * size.width + i],
            "the great circle at panel x = {x} passes through pixel ({i}, {j}), which is not painted"
        );
        rows.push(j);
    }
    // In the frame that circle is not a row: the line is drawn pre-bent.
    let (lowest, highest) = (rows.iter().min().unwrap(), rows.iter().max().unwrap());
    assert!(
        highest - lowest >= 3,
        "the line spans rows {lowest} to {highest}, which does not test the bending"
    );
    // And nothing painted is off that circle by more than the line's half-thickness and a pixel.
    let normal = {
        let n = cross(
            panel.direction(10.0, line_y),
            panel.direction(110.0, line_y),
        );
        let length = dot(n, n).sqrt();
        n.map(|c| c / length)
    };
    let grid = Grid::new(size.width as u32, size.height as u32);
    for &(i, j) in &pixels {
        let off = dot(grid.pixel_direction(i as u32, j as u32), normal)
            .asin()
            .abs()
            / pixel;
        assert!(
            off < 2.6,
            "pixel ({i}, {j}) is {off:.2} pixels off the great circle"
        );
    }
}

#[test]
fn test_no_pixel_outside_the_panel_changes_and_with_readouts_off_none_does() {
    let (size, _) = frame(720);
    let s = series(&[Some(6.0), Some(5.5)]);
    let mut overlay =
        Overlay::new(&settings(&[Placement::DEFAULT], 2.0), &s, [0.0, 1.0], size).expect("a panel");
    let base: Vec<u16> = (0..size.width * size.height * 3)
        .map(|k| (k * 7919 % 65_536) as u16)
        .collect();
    let mut painted = base.clone();
    overlay.paint(&mut painted, &[Some(0.5), Some(5.75)]);
    let layout = overlay.layout();
    let panel = Panel::new(
        Placement::DEFAULT,
        2.0 * PI / size.width as f64,
        layout.width,
        layout.height,
        layout.radius,
    );
    let grid = Grid::new(size.width as u32, size.height as u32);
    let mut changed = 0;
    for k in 0..size.width * size.height {
        if painted[3 * k..3 * k + 3] == base[3 * k..3 * k + 3] {
            continue;
        }
        changed += 1;
        let n = grid.pixel_direction((k % size.width) as u32, (k / size.width) as u32);
        let (x, y) = panel
            .locate(n)
            .unwrap_or_else(|| panic!("pixel {k} changed, and is behind the panel"));
        assert!(
            (0.0..=panel.width()).contains(&x) && (0.0..=panel.height()).contains(&y),
            "pixel {k} changed, and reads ({x}, {y}) outside the panel"
        );
    }
    assert!(changed > 100, "only {changed} pixels changed");

    // Off: no overlay, so the frame is the renderer's to the bit.
    let args: Vec<String> = [
        "--bundle",
        "b",
        "--sky",
        "s.exr",
        "--out",
        "v.mp4",
        "--size",
        "720x360",
        "--readouts",
        "off",
    ]
    .map(String::from)
    .to_vec();
    let crate::cli::Request::Render(options) = crate::cli::parse(&args).expect("parses") else {
        panic!("not a render")
    };
    assert!(
        Overlay::for_run(&options, &s, [0.0, 1.0])
            .expect("no refusal")
            .is_none()
    );
    // And on, the same options make an overlay.
    let on = crate::cli::Options {
        readouts: true,
        ..*options
    };
    assert!(
        Overlay::for_run(&on, &s, [0.0, 1.0])
            .expect("a panel")
            .is_some()
    );
}

#[test]
fn test_a_panel_behind_the_observer_is_drawn_whole_across_the_seam() {
    let (size, _) = frame(720);
    let s = series(&[Some(6.0)]);
    let mut overlay =
        Overlay::new(&settings(&[at(180.0, 0.0)], 2.0), &s, [0.0, 1.0], size).expect("a panel");
    let base = vec![30_000u16; size.width * size.height * 3];
    let mut painted = base.clone();
    overlay.paint(&mut painted, &[Some(0.0), Some(6.0)]);
    let changed: Vec<(usize, usize)> = (0..size.width * size.height)
        .filter(|&k| painted[3 * k..3 * k + 3] != base[3 * k..3 * k + 3])
        .map(|k| (k % size.width, k / size.width))
        .collect();
    let left = changed.iter().filter(|p| p.0 < size.width / 2).count();
    let right = changed.len() - left;
    assert!(
        left > 0 && right > 0,
        "{left} pixels changed at the left edge and {right} at the right"
    );
    // The two halves are one panel: unwrapped across the seam its columns and rows run without
    // a gap, and it is centred on the seam to within a column.
    let unwrap = |i: usize| {
        if i < size.width / 2 {
            i + size.width
        } else {
            i
        }
    };
    let mut columns: Vec<usize> = changed.iter().map(|p| unwrap(p.0)).collect();
    columns.sort_unstable();
    columns.dedup();
    assert_eq!(
        columns.len(),
        columns.last().unwrap() - columns.first().unwrap() + 1,
        "the panel's columns have a gap"
    );
    let middle = (columns.first().unwrap() + columns.last().unwrap() + 1) as f64 / 2.0;
    assert!(
        (middle - size.width as f64).abs() <= 1.0,
        "the panel is centred at column {middle}, not on the seam"
    );
    let mut rows: Vec<usize> = changed.iter().map(|p| p.1).collect();
    rows.sort_unstable();
    rows.dedup();
    assert_eq!(rows.len(), rows.last().unwrap() - rows.first().unwrap() + 1);
}

// ---- values --------------------------------------------------------------------------------------

#[test]
fn test_a_value_between_bundle_frames_is_interpolated_by_the_timelines_weight() {
    let s = series(&[Some(6.0), Some(5.0), Some(2.0)]);
    assert_eq!(s.at(Pick::One(1), 1.0), vec![Some(1.0), Some(5.0)]);
    assert_eq!(
        s.at(Pick::Two(0, 1, 0.25), 0.25),
        vec![Some(0.25), Some(5.75)]
    );
    assert_eq!(s.at(Pick::Two(1, 2, 0.5), 1.5), vec![Some(1.5), Some(3.5)]);
}

#[test]
fn test_the_stopwatch_is_the_timelines_own_time_and_not_the_bundles() {
    // The bundle's stopwatch values are deliberately wrong (they count bundle frames, 1 a frame,
    // where the frames are a tenth of a second apart): the panel must not show them.
    let s = series(&[Some(1.0), Some(1.0), Some(1.0)]);
    let t = Timeline::new(&[0.0, 0.1, 0.2], 0.0, 30.0, 1.0);
    for k in [0u64, 1, 2, 3, 5, 6] {
        let shown = s.at(t.pick(k), t.stopwatch(k))[0];
        assert_eq!(shown, Some(k as f64 / 30.0), "video frame {k}");
    }
}

#[test]
fn test_nan_infinity_and_a_missing_value_show_the_nearer_frame_as_it_is() {
    let nan = Some(f64::NAN);
    let inf = Some(f64::INFINITY);
    assert_eq!(interpolate(Some(3.0), None, 0.3), Some(3.0));
    assert_eq!(interpolate(Some(3.0), None, 0.7), None);
    assert_eq!(interpolate(None, Some(3.0), 0.7), Some(3.0));
    assert_eq!(interpolate(Some(3.0), inf, 0.4), Some(3.0));
    assert_eq!(interpolate(Some(3.0), inf, 0.6), inf);
    assert!(interpolate(nan, Some(3.0), 0.2).unwrap().is_nan());
    assert_eq!(interpolate(nan, Some(3.0), 0.8), Some(3.0));
    // Through a series: a frame that omits the radius leaves its line blank.
    let s = series(&[Some(6.0), None, Some(f64::NEG_INFINITY)]);
    assert_eq!(s.at(Pick::One(1), 1.0)[1], None);
    assert_eq!(s.at(Pick::Two(1, 2, 0.9), 1.9)[1], Some(f64::NEG_INFINITY));
    // How they are written.
    assert_eq!(format(Style::POINT, f64::NAN, 2), "n/a");
    assert_eq!(format(Style::POINT, f64::INFINITY, 2), "∞");
    assert_eq!(format(Style::POINT, f64::NEG_INFINITY, 2), "-∞");
}

#[test]
fn test_values_are_written_with_the_declared_decimals_and_the_chosen_decimal_mark() {
    assert_eq!(format(Style::POINT, 4.567, 2), "4.57");
    assert_eq!(format(Style::COMMA, 4.567, 2), "4,57");
    assert_eq!(format(Style::POINT, 4.5, 3), "4.500");
    assert_eq!(format(Style::POINT, 4.6, 0), "5");
    assert_eq!(format(Style::POINT, 12.0, 1), "12.0");
    // Past ten thousand every decimal stays, so a growing value keeps its shape.
    assert_eq!(format(Style::POINT, 12_345.678, 2), "12,345.68");
    assert_eq!(format(Style::COMMA, 12_345.678, 2), "12.345,68");
    // Past a billion, an exponent.
    assert_eq!(format(Style::POINT, 2.5e9, 2), "2.50e9");
}

#[test]
fn test_a_frame_that_omits_a_read_out_leaves_its_line_blank_and_the_others_where_they_were() {
    let (size, _) = frame(2048);
    let s = series(&[Some(6.0)]);
    let mut overlay =
        Overlay::new(&settings(&[Placement::DEFAULT], 4.0), &s, [0.0, 1.0], size).expect("a panel");
    let both = overlay.draw(&[Some(4.57), Some(6.0)]).clone();
    let one = overlay.draw(&[Some(4.57), None]).clone();
    let half = both.height / 2;
    // The panel's upper half holds the stopwatch's line, the lower half the radius's.
    assert_eq!(
        both.data[..half * both.width],
        one.data[..half * both.width],
        "the stopwatch's line moved"
    );
    assert_ne!(
        both.data[half * both.width..],
        one.data[half * both.width..]
    );
}

// ---- digits that hold still ----------------------------------------------------------------------

#[test]
fn test_the_unit_and_the_decimal_mark_do_not_move_as_the_value_changes() {
    let (size, _) = frame(4096);
    let decls = [decl("stopwatch", "Stopwatch", "s", 2)];
    let frames = [values(&[("stopwatch", Some(0.0))])];
    let s = Series::new(&decls, &frames.iter().collect::<Vec<_>>());
    let mut overlay = Overlay::new(&settings(&[Placement::DEFAULT], 2.0), &s, [0.0, 10.0], size)
        .expect("a panel");
    let fonts = overlay.fonts();
    // The digits of this face are not all one width: without tabular cells they would shift.
    let advances: Vec<f32> = ('0'..='9').map(|d| fonts.advance(d)).collect();
    assert!(
        advances.iter().any(|&a| a != advances[1]),
        "the face's digits are all one width, so this test proves nothing"
    );
    let layout = overlay.layout();
    let dot_at = |text: &str| -> f32 {
        let placed: Vec<Placed> = layout.value(fonts, 0, text);
        placed
            .iter()
            .find(|p| p.c == '.')
            .expect("a decimal mark")
            .x
    };
    let right_edge = |text: &str| -> f32 {
        let placed = layout.value(fonts, 0, text);
        let last = placed.last().unwrap();
        // Digits sit centred in their cells, so the cell's right edge is past the glyph's advance.
        last.x + 0.5 * (fonts.digit_width() + fonts.advance(last.c))
    };
    for text in ["1.11", "4.57", "9.99", "10.00"] {
        assert_eq!(dot_at(text), dot_at("9.99"), "{text}'s decimal mark moved");
        assert!((right_edge(text) - right_edge("9.99")).abs() < 1e-3);
    }
    // In pixels: the unit is drawn identically whatever the value.
    let unit_from = layout.unit_x().floor() as usize;
    let pixels_of_unit = |c: &Coverage| -> Vec<f32> {
        (0..c.height)
            .flat_map(|y| (unit_from..c.width).map(move |x| (x, y)))
            .map(|(x, y)| c.data[y * c.width + x])
            .collect()
    };
    let reference = pixels_of_unit(&overlay.draw(&[Some(1.11)]).clone());
    assert!(reference.iter().any(|&c| c > 0.5), "no unit was drawn");
    for x in [4.57, 9.99] {
        assert_eq!(
            pixels_of_unit(&overlay.draw(&[Some(x)]).clone()),
            reference,
            "the unit moved for {x}"
        );
    }
}

#[test]
fn test_the_infinity_sign_comes_from_the_fallback_face_and_draws_ink() {
    let fonts = Fonts::new(40.0);
    assert!(fonts.main_face_has('7'));
    let mut c = Coverage::new(80, 60);
    c.draw(
        &fonts,
        &[Placed {
            c: '∞',
            x: 10.0,
            baseline: 45.0,
        }],
    );
    let ink: f32 = c.data.iter().sum();
    assert!(ink > 100.0, "the infinity sign drew {ink} pixels of ink");
}

// ---- the command line ----------------------------------------------------------------------------

#[test]
fn test_readout_placements_near_a_pole_or_malformed_are_refused_each_with_its_own_sentence() {
    assert_eq!(Placement::parse("90,-30"), Ok(at(90.0, -30.0)));
    assert_eq!(Placement::parse(" -12.5 , 70 "), Ok(at(-12.5, 70.0)));
    let refusal = |text: &str| Placement::parse(text).expect_err(text);
    let pole = refusal("0,75");
    assert!(pole.contains("within 20 degrees of a pole"), "{pole}");
    assert!(refusal("10,-89").contains("within 20 degrees of a pole"));
    let past = refusal("0,95");
    assert!(past.contains("past a pole"), "{past}");
    let not_finite = refusal("nan,0");
    assert!(not_finite.contains("finite"), "{not_finite}");
    for malformed in ["", "90", "90,-30,5", "ninety,0", "90;-30", "90,"] {
        let message = refusal(malformed);
        assert!(message.contains("<heading>,<elevation>"), "{message}");
    }
    let sentences = [pole, past, not_finite, refusal("90")];
    for (a, first) in sentences.iter().enumerate() {
        for second in &sentences[a + 1..] {
            assert_ne!(first, second);
        }
    }
    // Through the whole command line: the first --readout-at replaces the default panel, and
    // the others add to it.
    let parse = |extra: &[&str]| {
        let mut args: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--out", "v.mp4"]
            .map(String::from)
            .to_vec();
        args.extend(extra.iter().map(|s| s.to_string()));
        crate::cli::parse(&args)
    };
    let crate::cli::Request::Render(o) = parse(&[]).unwrap() else {
        panic!()
    };
    assert_eq!(o.readout_at, vec![Placement::DEFAULT]);
    // The overlay is the default: subtitles, and no panel painted on the picture.
    assert!(o.subtitles && !o.readouts && !o.decimal_comma && o.readout_size == 2.0);
    let crate::cli::Request::Render(o) = parse(&[
        "--readout-at",
        "90,0",
        "--readout-at",
        "180,-10",
        "--decimal-comma",
    ])
    .unwrap() else {
        panic!()
    };
    assert_eq!(o.readout_at, vec![at(90.0, 0.0), at(180.0, -10.0)]);
    assert!(o.decimal_comma);
    assert!(
        parse(&["--readout-at", "0,80"])
            .unwrap_err()
            .contains("pole")
    );
    assert!(parse(&["--readouts", "maybe"]).is_err());
    assert!(parse(&["--readout-size", "0"]).is_err());
    assert!(parse(&["--readout-size", "30"]).is_err());
}

#[test]
fn test_a_panel_too_large_to_stay_off_the_pole_is_refused() {
    // Three short lines 12 degrees high make a panel some 43 degrees high and about as wide: at
    // elevation -70, the lowest the command line allows, its bottom edge passes the pole.
    let (size, _) = frame(720);
    let decls = [
        decl("stopwatch", "T", "", 2),
        decl("r", "r", "", 2),
        decl("q", "q", "", 2),
    ];
    let frames = [values(&[("stopwatch", Some(0.0))])];
    let s = Series::new(&decls, &frames.iter().collect::<Vec<_>>());
    let refused = Overlay::new(&settings(&[at(0.0, -70.0)], 12.0), &s, [0.0, 1.0], size)
        .err()
        .expect("the panel reaches the pole");
    assert!(refused.contains("over the pole"), "{refused}");
    // The same panel on the equator is drawn.
    assert!(Overlay::new(&settings(&[at(0.0, 0.0)], 12.0), &s, [0.0, 1.0], size).is_ok());
}

// ---- a still's panel -----------------------------------------------------------------------------

#[test]
fn test_a_stills_panel_leaves_the_stopwatch_line_out_label_and_all_and_a_films_keeps_it() {
    let (size, _) = frame(2048);
    let s = series(&[Some(6.0)]);
    assert_eq!(crate::overlay::shown_lines(&s, true), vec![1]);
    assert_eq!(crate::overlay::shown_lines(&s, false), vec![0, 1]);
    let options = |extra: &[&str]| {
        let mut args: Vec<String> = [
            "--bundle",
            "b",
            "--sky",
            "s.exr",
            "--encoder",
            "none",
            "--size",
            "2048x1024",
            "--readouts",
            "panel",
        ]
        .map(String::from)
        .to_vec();
        args.extend(extra.iter().map(|a| a.to_string()));
        let crate::cli::Request::Render(o) = crate::cli::parse(&args).expect("parses") else {
            panic!("not a render")
        };
        *o
    };
    let mut still = Overlay::for_run(&options(&["--still", "0"]), &s, [0.0, 0.0])
        .expect("no refusal")
        .expect("a panel");
    let film = Overlay::for_run(&options(&[]), &s, [0.0, 1.0])
        .expect("no refusal")
        .expect("a panel");
    assert_eq!(
        still.line_count(),
        1,
        "the still's panel shows the stopwatch"
    );
    assert_eq!(film.line_count(), 2, "the film's panel lost a line");
    // The still's panel is the panel of the radius alone: one line high, not a blank line and a
    // radius, and what it draws is the radius alone, whatever the stopwatch's value.
    let decls = [decl("r", "Radius", "M", 3)];
    let frames = [values(&[("r", Some(6.0))])];
    let radius_only = Series::new(&decls, &frames.iter().collect::<Vec<_>>());
    let mut alone = Overlay::new(
        &settings(&[Placement::DEFAULT], 2.0),
        &radius_only,
        [0.0, 0.0],
        size,
    )
    .expect("a panel");
    assert_eq!(still.layout(), alone.layout());
    assert!(still.layout().height < film.layout().height);
    let drawn = still.draw(&[Some(4.57), Some(6.0)]).clone();
    assert_eq!(
        drawn,
        *alone.draw(&[Some(6.0)]),
        "the still's panel drew more"
    );
    assert!(
        drawn.data.iter().any(|&c| c > 0.5),
        "the radius was not drawn"
    );
    // A still whose only read-out is the stopwatch has no panel at all.
    let only = Series::new(
        &[decl("stopwatch", "Stopwatch", "s", 2)],
        &[&values(&[("stopwatch", Some(0.0))])],
    );
    assert!(
        Overlay::for_run(&options(&["--still", "0"]), &only, [0.0, 0.0])
            .expect("no refusal")
            .is_none()
    );
    // A film's panel of the stopwatch alone is drawn as before.
    assert!(
        Overlay::for_run(&options(&[]), &only, [0.0, 1.0])
            .expect("no refusal")
            .is_some()
    );
}
