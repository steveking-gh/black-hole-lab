//! `--readout-at dark`'s promises: the panel sits where the dark region is deepest, shrinks to
//! fit, goes below the opening view when it cannot, and covers only pixels the renderer draws as
//! dark, with a margin of dark all round it.
//!
//! Frames here are 720 pixels wide (half a degree a pixel) unless a test says otherwise. Dark
//! regions are drawn straight into a mask from their geometry, or traced into a bundle frame and
//! found through `RayField` as the renderer finds them. Whether a panel keeps clear of every pixel
//! that is not dark is checked independently of `crate::shadow`: the panel's outline is walked in
//! steps of a twentieth of a panel pixel with `Panel::direction`, and every pixel that is not dark
//! must lie outside the outline and at least the margin (less the walk's step) from it.

use std::f64::consts::PI;

use readout::Style;
use sky_format::{Frame, Grid, Num, ReadoutDecl, fate};

use crate::field::{Judge, angle};
use crate::load::prepare;
use crate::overlay::{Overlay, Settings, shown_lines};
use crate::panel::{Panel, Placement};
use crate::render::{Fields, Look, Scene, Shade, Size, render, shades};
use crate::shadow::{
    DarkMap, FLOOR_DEGREES, MARGIN_DEGREES, Outline, Placing, Why, degrees, fits, line_heights,
    place, sentence,
};
use crate::sky::{MapFrame, Mat3, SkyMap, orientation};
use crate::tone::Encoder;
use crate::values::Series;

// ---- helpers -------------------------------------------------------------------------------------

fn size(width: usize) -> Size {
    Size {
        width,
        height: width / 2,
    }
}

/// The direction of heading `h` (to the right) and elevation `e`, in degrees, by the formulae of
/// `crate::panel`'s comment.
fn towards(h: f64, e: f64) -> [f64; 3] {
    let (h, e) = (h.to_radians(), e.to_radians());
    [e.cos() * h.cos(), -e.cos() * h.sin(), e.sin()]
}

/// A mask of `size` whose pixel is dark where `dark` says so of the direction through its centre.
fn mask(size: Size, dark: impl Fn([f64; 3]) -> bool) -> DarkMap {
    let grid = Grid::new(size.width as u32, size.height as u32);
    let pixels = (0..size.height)
        .flat_map(|j| (0..size.width).map(move |i| (i, j)))
        .map(|(i, j)| dark(grid.pixel_direction(i as u32, j as u32)))
        .collect();
    DarkMap::from_mask(size, pixels)
}

/// Dark within `radius` degrees of `centre`.
fn disc(centre: [f64; 3], radius: f64) -> impl Fn([f64; 3]) -> bool {
    move |n| angle(n, centre) < radius.to_radians()
}

/// A panel `across` line heights wide and `down` high, with corners of 0.3 of a line, on a frame
/// of `size`: what a real layout of a few lines comes to, without typefaces.
fn outline(size: Size, across: f64, down: f64) -> impl Fn(f64) -> Outline {
    let per_pixel = 360.0 / size.width as f64;
    move |s| Outline {
        width: (across * s / per_pixel).round() as usize,
        height: (down * s / per_pixel).round() as usize,
        radius: 0.3 * s / per_pixel,
    }
}

/// The panel `placing` puts on a frame of `size`, with the outline `o`.
fn panel_of(size: Size, at: Placement, o: Outline) -> Panel {
    Panel::new(
        at,
        2.0 * PI / size.width as f64,
        o.width,
        o.height,
        o.radius,
    )
}

/// Points along the rounded outline of a panel `o` in panel coordinates, a twentieth of a panel
/// pixel apart.
fn outline_points(o: Outline) -> Vec<(f64, f64)> {
    let (w, h, r) = (o.width as f64, o.height as f64, o.radius);
    let step = 0.05;
    let mut out = Vec::new();
    let along = |from: f64, to: f64| {
        let n = ((to - from) / step).ceil().max(1.0) as usize;
        (0..=n).map(move |k| from + (to - from) * k as f64 / n as f64)
    };
    for x in along(r, w - r) {
        out.push((x, 0.0));
        out.push((x, h));
    }
    for y in along(r, h - r) {
        out.push((0.0, y));
        out.push((w, y));
    }
    let arc = (0.5 * PI * r / step).ceil().max(1.0) as usize;
    for (cx, cy, start) in [
        (r, r, PI),
        (w - r, r, 1.5 * PI),
        (w - r, h - r, 0.0),
        (r, h - r, 0.5 * PI),
    ] {
        for k in 0..=arc {
            let t = start + 0.5 * PI * k as f64 / arc as f64;
            out.push((cx + r * t.cos(), cy + r * t.sin()));
        }
    }
    out
}

/// The pixels (i, j) of a frame of `size` that lie inside `panel` (outline `o`) or within the
/// margin of its outline, found by walking the outline (see the module comment); `slack` is taken
/// off the margin for the walk's step.
fn near_panel(size: Size, panel: &Panel, o: Outline) -> Vec<(usize, usize)> {
    let grid = Grid::new(size.width as u32, size.height as u32);
    let centre = panel.direction(0.5 * o.width as f64, 0.5 * o.height as f64);
    let boundary: Vec<[f64; 3]> = outline_points(o)
        .into_iter()
        .map(|(x, y)| panel.direction(x, y))
        .collect();
    let slack = 0.05 * 2.0 * PI / size.width as f64;
    let reach = panel.angular_radius() + MARGIN_DEGREES.to_radians();
    let mut out = Vec::new();
    for j in 0..size.height {
        for i in 0..size.width {
            let n = grid.pixel_direction(i as u32, j as u32);
            if angle(n, centre) > reach + 0.01 {
                continue;
            }
            let inside = panel
                .locate(n)
                .is_some_and(|(x, y)| panel.coverage(x, y) > 0.0);
            let nearest = boundary
                .iter()
                .map(|&b| angle(n, b))
                .fold(f64::INFINITY, f64::min);
            if inside || nearest < MARGIN_DEGREES.to_radians() - slack {
                out.push((i, j));
            }
        }
    }
    out
}

/// Checks that every pixel inside the panel or within the margin of it is dark in `map`, and
/// that there are some.
fn assert_clear(map: &DarkMap, size: Size, panel: &Panel, o: Outline, what: &str) {
    let near = near_panel(size, panel, o);
    assert!(!near.is_empty(), "{what}: the panel covers no pixel");
    for (i, j) in near {
        assert!(
            map.is_dark(i, j),
            "{what}: pixel ({i}, {j}) is inside the panel or within {MARGIN_DEGREES} degree of it, \
             and is not dark"
        );
    }
}

/// The placing's centre and line height, or a panic naming why it went outside.
fn inside(placing: Placing, what: &str) -> (Placement, f64) {
    match placing {
        Placing::Inside { at, line_degrees } => (at, line_degrees),
        Placing::Outside(why) => panic!("{what}: the panel went outside, because {why:?}"),
    }
}

/// A bundle frame of `width` x `height` rays whose rays within `radius` degrees of `centre` are
/// dark and the rest reach the far sky along the direction they were traced.
fn frame_with_disc(width: u32, height: u32, centre: [f64; 3], radius: f64) -> Frame {
    let grid = Grid::new(width, height);
    let mut frame = Frame::new(width, height, 0);
    for j in 0..height {
        for i in 0..width {
            let k = grid.offset(i, j);
            let n = grid.pixel_direction(i, j);
            let dark = angle(n, centre) < radius.to_radians();
            frame.fate[k] = if dark { fate::DARK } else { fate::FAR_SKY };
            for (plane, c) in frame.direction.iter_mut().zip(n) {
                plane[k] = if dark { f32::NAN } else { c as f32 };
            }
            frame.shift[k] = if dark { f32::NAN } else { 1.0 };
        }
    }
    frame
}

/// The frame prepared as `--undersampled mark` prepares it for a picture of `size` over `sky`.
fn judged(frame: &Frame, size: Size, sky: &SkyMap) -> crate::field::RayField {
    let judge = Judge {
        pixels_per_ray: (
            size.width as f64 / f64::from(frame.width),
            size.height as f64 / f64::from(frame.height),
        ),
        texel: (2.0 * PI / sky.width() as f64).min(PI / sky.height() as f64),
    };
    prepare(frame, Some(judge), 2)
}

fn identity() -> Mat3 {
    let n = |v: [f64; 3]| v.map(Num);
    let icrs = sky_format::FarSky {
        name: "icrs".into(),
        axes_in_icrs: sky_format::Axes {
            x: n([1.0, 0.0, 0.0]),
            y: n([0.0, 1.0, 0.0]),
            z: n([0.0, 0.0, 1.0]),
        },
    };
    orientation(&icrs, MapFrame::Celestial)
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

/// The tracer's three read-outs at one bundle frame.
fn tracer_series() -> Series {
    let decls = [
        decl("stopwatch", "Stopwatch", "s", 2),
        decl("radius", "Radius", "M", 3),
        decl("distant_clock", "Distant clock", "M", 2),
    ];
    let values: std::collections::BTreeMap<String, Num> =
        [("stopwatch", 0.0), ("radius", 6.0), ("distant_clock", 12.5)]
            .into_iter()
            .map(|(k, v)| (k.to_string(), Num(v)))
            .collect();
    Series::new(&decls, &[&values])
}

// ---- where the panel goes ------------------------------------------------------------------------

#[test]
fn test_a_disc_ahead_takes_the_panel_at_its_centre_at_the_size_asked_for() {
    let size = size(720);
    let map = mask(size, disc(towards(0.0, 0.0), 30.0));
    let o = outline(size, 6.0, 2.0);
    let (at, s) = inside(place(&map, 2.0, 3, &o), "a disc of 30 degrees");
    assert!(
        at.heading.abs() < 1e-9 && at.elevation.abs() < 1e-9,
        "the panel is centred at ({}, {}), not at the disc's centre",
        at.heading,
        at.elevation
    );
    assert_eq!(s, 2.0, "the panel was shrunk though it fitted");
    assert_clear(&map, size, &panel_of(size, at, o(s)), o(s), "a disc ahead");
}

#[test]
fn test_a_disc_to_the_right_and_up_takes_the_panel_at_its_centre() {
    let size = size(720);
    let centre = towards(45.0, 10.0);
    let map = mask(size, disc(centre, 25.0));
    let o = outline(size, 6.0, 2.0);
    let (at, s) = inside(place(&map, 2.0, 3, &o), "a disc at 45, 10");
    let off = angle(towards(at.heading, at.elevation), centre).to_degrees();
    assert!(
        off <= 0.5,
        "the panel is centred at ({}, {}), {off:.2} degrees from the disc's centre at (45, 10)",
        at.heading,
        at.elevation
    );
    assert_clear(
        &map,
        size,
        &panel_of(size, at, o(s)),
        o(s),
        "a disc at 45, 10",
    );
}

#[test]
fn test_a_disc_behind_the_observer_is_found_whole_across_the_seam() {
    let size = size(720);
    let centre = towards(180.0, -10.0);
    let map = mask(size, disc(centre, 25.0));
    // The disc is split by the seam: dark pixels at both edges of the frame.
    assert!(map.is_dark(0, 200) && map.is_dark(719, 200));
    let o = outline(size, 6.0, 2.0);
    let (at, s) = inside(place(&map, 2.0, 3, &o), "a disc behind");
    let off = angle(towards(at.heading, at.elevation), centre).to_degrees();
    assert!(
        off <= 0.5,
        "the panel is centred at ({}, {}), {off:.2} degrees from the disc's centre at (180, -10)",
        at.heading,
        at.elevation
    );
    let panel = panel_of(size, at, o(s));
    let columns: Vec<usize> = panel
        .cover(size)
        .iter()
        .map(|t| t.pixel as usize % size.width)
        .collect();
    assert!(
        columns.iter().any(|&i| i < 20) && columns.iter().any(|&i| i > 700),
        "the panel does not reach across the seam"
    );
    assert_clear(&map, size, &panel, o(s), "a disc behind");
}

#[test]
fn test_a_d_shaped_region_takes_the_panel_at_the_centre_of_its_largest_cap() {
    // A disc of 30 degrees about heading 0, cut by the great circle 10 degrees to the left of the
    // meridian through its centre: the largest cap inside it, of 20 degrees, is centred at
    // heading 10, half way between the cut and the far rim.
    let size = size(720);
    let centre = towards(0.0, 0.0);
    let cut = (-10f64).to_radians().sin();
    let map = mask(size, |n| {
        angle(n, centre) < 30f64.to_radians() && -n[1] > cut // -n[1]: toward the right
    });
    let o = outline(size, 6.0, 2.0);
    let (at, s) = inside(place(&map, 2.0, 3, &o), "a D-shaped region");
    assert!(
        (at.heading - 10.0).abs() <= 0.5 && at.elevation.abs() <= 0.5,
        "the panel is centred at ({}, {}), not at (10, 0)",
        at.heading,
        at.elevation
    );
    assert_clear(
        &map,
        size,
        &panel_of(size, at, o(s)),
        o(s),
        "a D-shaped region",
    );
}

#[test]
fn test_an_unresolved_ray_inside_the_dark_region_keeps_the_margin_or_moves_the_panel() {
    // A bundle frame of 128 x 64 rays, dark within 30 degrees of heading 0, but for one ray near
    // the middle which the tracer could not resolve: the picture shows a patch of red there.
    let size = size(720);
    let mut frame = frame_with_disc(128, 64, towards(0.0, 0.0), 30.0);
    let grid = Grid::new(128, 64);
    let (u, v) = grid.frame_coordinates(towards(2.0, 1.0));
    let k = grid.offset(u as u32, v as u32);
    frame.fate[k] = fate::UNRESOLVED;
    let field = crate::field::RayField::from_frame(&frame);
    let map = DarkMap::of(Some(Fields::One(&field)), size, 3);
    let lost = (0..size.height)
        .flat_map(|j| (0..size.width).map(move |i| (i, j)))
        .filter(|&(i, j)| {
            !map.is_dark(i, j)
                && angle(
                    Grid::new(720, 360).pixel_direction(i as u32, j as u32),
                    towards(0.0, 0.0),
                ) < 20f64.to_radians()
        })
        .count();
    assert!(
        lost > 0,
        "the unresolved ray made no pixel inside the disc not dark"
    );
    let o = outline(size, 6.0, 2.0);
    let (at, s) = inside(place(&map, 2.0, 3, &o), "a disc with an unresolved ray");
    assert!(
        at.heading.abs() > 1.0 || at.elevation.abs() > 1.0,
        "the panel stayed at the disc's centre, over the unresolved ray"
    );
    assert_clear(
        &map,
        size,
        &panel_of(size, at, o(s)),
        o(s),
        "a disc with an unresolved ray",
    );
}

// ---- when it does not fit ------------------------------------------------------------------------

#[test]
fn test_the_panel_shrinks_to_fit_in_steps_of_about_a_tenth_and_stops_at_the_floor() {
    assert_eq!(
        line_heights(2.0),
        vec![2.0, 1.8, 1.62, 1.46, 1.31, 1.18, 1.06, 1.0]
    );
    assert_eq!(line_heights(1.05), vec![1.05, 1.0]);
    // At or below the floor only the size asked for is tried.
    assert_eq!(line_heights(1.0), vec![1.0]);
    assert_eq!(line_heights(0.8), vec![0.8]);
    assert_eq!(FLOOR_DEGREES, 1.0);

    let size = size(720);
    let o = outline(size, 6.0, 2.0);
    // A disc of 8 degrees holds a panel 6 lines by 2 with its margin at about 2 degrees a line,
    // not at 3: the panel is shrunk, to a size on the list, and no further than it must be.
    let map = mask(size, disc(towards(0.0, 0.0), 8.0));
    let (at, s) = inside(place(&map, 3.0, 3, &o), "a disc of 8 degrees");
    let heights = line_heights(3.0);
    let k = heights
        .iter()
        .position(|&h| h == s)
        .unwrap_or_else(|| panic!("{s} is not one of the sizes tried, {heights:?}"));
    assert!(k > 0 && s > 1.0, "the panel went in at {s} degrees a line");
    assert!(
        !fits(&map, &panel_of(size, at, o(heights[k - 1]))),
        "the panel would have fitted at {} degrees a line, the size before {s}",
        heights[k - 1]
    );
    assert_clear(
        &map,
        size,
        &panel_of(size, at, o(s)),
        o(s),
        "a disc of 8 degrees",
    );

    // A disc of 3 degrees holds it at no size down to the floor: the panel goes below the
    // opening view, and the run says the smallest size it tried.
    let small = mask(size, disc(towards(0.0, 0.0), 3.0));
    assert_eq!(
        place(&small, 3.0, 3, &o),
        Placing::Outside(Why::TooSmall { line_degrees: 1.0 })
    );
    // Asked for below the floor, only that size is tried.
    assert_eq!(
        place(&small, 0.8, 3, &o),
        Placing::Outside(Why::TooSmall { line_degrees: 0.8 })
    );
    // And at 0.4 degrees a line the same disc holds it.
    let (at, s) = inside(
        place(&small, 0.4, 3, &o),
        "a disc of 3 degrees at 0.4 a line",
    );
    assert_eq!(s, 0.4);
    assert_clear(
        &small,
        size,
        &panel_of(size, at, o(s)),
        o(s),
        "a disc of 3 degrees",
    );
}

#[test]
fn test_no_dark_region_or_one_only_near_a_pole_sends_the_panel_below_the_opening_view() {
    let size = size(720);
    let o = outline(size, 6.0, 2.0);
    let none = mask(size, |_| false);
    assert_eq!(none.count(), 0);
    assert_eq!(place(&none, 2.0, 3, &o), Placing::Outside(Why::NoDark));
    // A frame before the bundle's first complete frame has no rays, and so no dark pixels.
    assert_eq!(DarkMap::of(None, size, 3).count(), 0);
    // A dark disc of 10 degrees about the nadir: no centre 20 degrees or more from the pole is
    // dark.
    let nadir = mask(size, disc([0.0, 0.0, -1.0], 10.0));
    assert!(nadir.count() > 0);
    assert_eq!(place(&nadir, 2.0, 3, &o), Placing::Outside(Why::NearPole));
}

// ---- the renderer's own dark pixels --------------------------------------------------------------

#[test]
fn test_a_pixel_is_dark_exactly_where_the_renderer_draws_the_shadow_from_one_frame_or_between_two()
{
    let size = size(256);
    let sky = SkyMap::new(64, 32, vec![[0.3; 3]; 64 * 32], 2);
    let a = judged(
        &frame_with_disc(48, 24, towards(0.0, 0.0), 30.0),
        size,
        &sky,
    );
    let b = judged(
        &frame_with_disc(48, 24, towards(20.0, 5.0), 25.0),
        size,
        &sky,
    );
    let plain_a = prepare(&frame_with_disc(48, 24, towards(0.0, 0.0), 30.0), None, 2);
    let plain_b = prepare(&frame_with_disc(48, 24, towards(20.0, 5.0), 25.0), None, 2);
    let cases = [
        Fields::One(&a),
        Fields::Two(&a, &b, 0.25),
        Fields::Two(&a, &b, 0.5),
        Fields::Two(&a, &b, 0.75),
        Fields::Two(&plain_a, &plain_b, 0.3),
        Fields::Two(&plain_a, &plain_b, 0.7),
    ];
    for (c, fields) in cases.into_iter().enumerate() {
        let scene = Scene {
            fields,
            rotation: identity(),
            sky: &sky,
        };
        let drawn = shades(&scene, size, 1.0, 2);
        let map = DarkMap::of(Some(fields), size, 2);
        let mut dark = 0;
        for j in 0..size.height {
            for i in 0..size.width {
                let shadow = drawn[j * size.width + i] == Shade::Shadow;
                assert_eq!(
                    map.is_dark(i, j),
                    shadow,
                    "case {c}: pixel ({i}, {j}) is {} in the map and drawn as {:?}",
                    map.is_dark(i, j),
                    drawn[j * size.width + i]
                );
                dark += usize::from(shadow);
            }
        }
        assert!(dark > 1000, "case {c}: only {dark} dark pixels");
    }
}

/// The whole path, for a dark disc of `radius` degrees: a judged bundle frame, a picture drawn
/// from it over a grey map (so that black is the dark region and nothing else), the tracer's
/// read-outs on a still's panel placed by `place` at 2 degrees a line, and the panel painted as
/// the renderer paints it. Checks every pixel the panel covers, and every one within the margin of
/// its outline, was black; returns the line height the panel went in at.
fn prove_black(radius: f64) -> f64 {
    let size = size(1024);
    let sky = SkyMap::new(64, 32, vec![[0.3; 3]; 64 * 32], 2);
    let field = judged(
        &frame_with_disc(128, 64, towards(20.0, -5.0), radius),
        size,
        &sky,
    );
    let scene = Scene {
        fields: Fields::One(&field),
        rotation: identity(),
        sky: &sky,
    };
    let look = Look {
        gain: 1.0,
        unresolved: [65_535, 0, 0],
        undersampled: [65_535, 0, 0],
        encoder: Encoder::new(),
    };
    let mut picture = vec![0u16; size.width * size.height * 3];
    render(&scene, size, &look, 2, &mut picture);
    let black = |i: usize, j: usize| picture[3 * (j * size.width + i)..][..3] == [0, 0, 0];

    let series = tracer_series();
    let shown = shown_lines(&series, true);
    let settings = |placements: Vec<Placement>, line_degrees: f64| Settings {
        placements,
        line_degrees,
        style: Style::POINT,
    };
    let measure = |s: f64| {
        Overlay::showing(&settings(Vec::new(), s), &series, &shown, [0.0, 0.0], size)
            .expect("a panel")
            .outline()
    };
    let map = DarkMap::of(Some(Fields::One(&field)), size, 2);
    let what = format!("a disc of {radius} degrees");
    let (at, s) = inside(place(&map, 2.0, 2, measure), &what);
    let mut overlay = Overlay::showing(&settings(vec![at], s), &series, &shown, [0.0, 0.0], size)
        .expect("a panel");
    let values = series.at(crate::timeline::Pick::One(0), 0.0);

    // What the panel covers: every pixel it changes on a white frame (its backing darkens each).
    let white = vec![65_535u16; picture.len()];
    let mut painted = white.clone();
    overlay.paint(&mut painted, &values);
    let covered: Vec<(usize, usize)> = (0..size.width * size.height)
        .filter(|&k| painted[3 * k..3 * k + 3] != white[3 * k..3 * k + 3])
        .map(|k| (k % size.width, k / size.width))
        .collect();
    assert!(
        covered.len() > 500,
        "the panel covers {} pixels",
        covered.len()
    );
    for &(i, j) in &covered {
        assert!(
            black(i, j),
            "{what}: the panel covers pixel ({i}, {j}), which was not black"
        );
    }
    // And every pixel within the margin of its outline.
    let o = overlay.outline();
    let near = near_panel(size, &panel_of(size, at, o), o);
    assert!(
        near.len() > covered.len() + 100,
        "the margin holds {} pixels beyond the panel's {}",
        near.len() - covered.len(),
        covered.len()
    );
    for (i, j) in near {
        assert!(
            black(i, j),
            "{what}: pixel ({i}, {j}) is within {MARGIN_DEGREES} degree of the panel and was not              black"
        );
    }
    // The picture does hold light and red around the dark region: the check is not vacuous.
    let lit = (0..size.width * size.height)
        .filter(|&k| picture[3 * k + 1] > 0)
        .count();
    let red = (0..size.width * size.height)
        .filter(|&k| picture[3 * k..3 * k + 3] == [65_535, 0, 0])
        .count();
    assert!(
        lit > 100_000 && red > 100,
        "{what}: {lit} lit pixels, {red} red ones"
    );
    s
}

#[test]
fn test_every_pixel_the_panel_covers_or_comes_within_the_margin_of_was_black_without_the_panel() {
    // With room to spare, at the size asked for.
    assert_eq!(prove_black(22.0), 2.0);
    // Tight: shrunk, so that its margin runs close along the rim of red.
    let s = prove_black(9.0);
    assert!(s < 2.0, "the panel was not shrunk in a disc of 9 degrees");
}

// ---- what the run says, and the command line -----------------------------------------------------

#[test]
fn test_the_run_says_where_the_panel_went_in_the_words_the_caller_reads() {
    let inside = Placing::Inside {
        at: Placement {
            heading: -12.25,
            elevation: -0.0,
        },
        line_degrees: 1.62,
    };
    assert_eq!(
        sentence(&inside, 2),
        "read-outs: 2 line(s) on a panel inside the dark region, centred -12.25 degrees right of \
         the opening view and 0 degrees up, each line 1.62 degrees high"
    );
    let too_small = sentence(&Placing::Outside(Why::TooSmall { line_degrees: 1.0 }), 2);
    assert_eq!(
        too_small,
        "read-outs: 2 line(s) on a panel below the opening view, because the dark region is too \
         small to hold them at 1 degree a line and keep 1 degree clear of its rim"
    );
    let below_floor = sentence(&Placing::Outside(Why::TooSmall { line_degrees: 0.8 }), 3);
    assert!(below_floor.ends_with("at 0.8 degrees a line and keep 1 degree clear of its rim"));
    let none = sentence(&Placing::Outside(Why::NoDark), 1);
    assert_eq!(
        none,
        "read-outs: 1 line(s) on a panel below the opening view, because the picture has no dark \
         region"
    );
    let pole = sentence(&Placing::Outside(Why::NearPole), 1);
    assert!(pole.contains("within 20 degrees of a pole"), "{pole}");
    for said in [&too_small, &below_floor, &none, &pole] {
        assert!(
            said.starts_with("read-outs: ")
                && said.contains(" line(s) on a panel below the opening view, because ")
        );
    }
    assert_eq!(degrees(0.0), "0");
    assert_eq!(degrees(-0.0), "0");
    assert_eq!(degrees(-0.001), "0");
    assert_eq!(degrees(180.0), "180");
    assert_eq!(degrees(1.5), "1.5");
    // A still with no video says so, and not that it is held.
    let line = crate::still_unheld_line(237, 474, size(8192), 2.5, 16);
    assert_eq!(
        line,
        "still: video frame 237 of 474, not held, since --encoder none makes no video, 8192 x \
         4096, exposure +2.50 stops, 16 threads"
    );
}

#[test]
fn test_readout_at_dark_is_for_a_still_alone_and_for_one_panel_alone() {
    let parse = |extra: &[&str]| {
        let mut args: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--encoder", "none"]
            .map(String::from)
            .to_vec();
        args.extend(extra.iter().map(|s| s.to_string()));
        match crate::cli::parse(&args) {
            Ok(crate::cli::Request::Render(o)) => Ok(*o),
            Ok(crate::cli::Request::Help) => panic!("not help"),
            Err(e) => Err(e),
        }
    };
    let o = parse(&[
        "--still",
        "3",
        "--photo",
        "p.jpg",
        "--readouts",
        "panel",
        "--readout-at",
        "dark",
    ])
    .expect("a still with its panel in the dark");
    assert!(o.readout_in_dark && o.readouts);
    // The default place stays, for when the dark region cannot hold the panel.
    assert_eq!(o.readout_at, vec![Placement::DEFAULT]);
    assert!(!parse(&["--still", "3"]).unwrap().readout_in_dark);

    let film = parse(&["--readouts", "panel", "--readout-at", "dark"]).unwrap_err();
    assert!(
        film.contains("--readout-at dark belongs to a still") && film.contains("moves and grows"),
        "{film}"
    );
    for extra in [
        ["--readout-at", "dark", "--readout-at", "90,0"],
        ["--readout-at", "90,0", "--readout-at", "dark"],
    ] {
        let mut args = vec!["--still", "3"];
        args.extend(extra);
        let both = parse(&args).unwrap_err();
        assert!(
            both.contains("cannot be combined with a --readout-at <heading>,<elevation>"),
            "{both}"
        );
    }
    assert_ne!(
        film,
        parse(&[
            "--still",
            "1",
            "--readout-at",
            "dark",
            "--readout-at",
            "0,0"
        ])
        .unwrap_err()
    );
    // Any other word is a malformed place, and the sentence says the word dark is allowed.
    let other = parse(&["--still", "1", "--readout-at", "shadow"]).unwrap_err();
    assert!(
        other.contains("<heading>,<elevation>") && other.contains("the word dark"),
        "{other}"
    );
    assert!(crate::cli::USAGE.contains("--readout-at dark"));
}
