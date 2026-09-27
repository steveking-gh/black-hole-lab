//! The marker colours' promises: what the renderer does not know is drawn as not known, and what
//! it does know is drawn exactly as before.
//!
//! Bundles are built in memory with a map from the observer's sky to the far sky chosen for the
//! property under test, and prepared through `load::prepare`, the function the renderer reads
//! frames through, judged for the size of the picture drawn from them.

// The loops over 0..3 are over the components of a vector or a colour.
#![allow(clippy::needless_range_loop)]

use std::f64::consts::PI;

use sky_format::{Frame, Grid, fate};

use crate::bilinear::taps;
use crate::field::{Judge, Ray, RayField, angle, blend_judged};
use crate::load::prepare;
use crate::render::{Fields, Look, Scene, Shade, Size, render, shades};
use crate::sky::{MapFrame, Mat3, SkyMap, orientation};
use crate::tally::{FilmTally, Tally};
use crate::timeline::Timeline;
use crate::tone::{Encoder, shade};

// ---- helpers -------------------------------------------------------------------------------------

const RED: [u16; 3] = [65_535, 0, 0];
const GREEN: [u16; 3] = [0, 65_535, 0];
const BLUE: [u16; 3] = [0, 0, 65_535];

/// A bundle frame whose ray at each grid direction n comes from the far-sky direction `map(n)`,
/// with shift 1 and winding `winding(n)`.
fn frame_of(
    width: u32,
    height: u32,
    map: impl Fn([f64; 3]) -> [f64; 3],
    winding: impl Fn([f64; 3]) -> i16,
) -> Frame {
    let grid = Grid::new(width, height);
    let mut frame = Frame::new(width, height, 0);
    for j in 0..height {
        for i in 0..width {
            let k = grid.offset(i, j);
            let n = grid.pixel_direction(i, j);
            let d = map(n);
            frame.fate[k] = fate::FAR_SKY;
            for c in 0..3 {
                frame.direction[c][k] = d[c] as f32;
            }
            frame.shift[k] = 1.0;
            frame.winding[k] = winding(n);
        }
    }
    frame
}

/// Sets ray (i, j) to `code`, with the NaN direction and shift the specification asks for.
fn set_fate(frame: &mut Frame, i: u32, j: u32, code: u8) {
    let k = frame.grid().offset(i, j);
    frame.fate[k] = code;
    for c in 0..3 {
        frame.direction[c][k] = f32::NAN;
    }
    frame.shift[k] = f32::NAN;
}

/// Rotates `n` by `angle` about +z.
fn turned(n: [f64; 3], angle: f64) -> [f64; 3] {
    let (s, c) = angle.sin_cos();
    [n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]]
}

/// The judge the renderer uses for a frame of `size` drawn from `grid` rays over `sky`.
fn judge(grid: (u32, u32), size: Size, sky: &SkyMap) -> Judge {
    Judge {
        pixels_per_ray: (
            size.width as f64 / f64::from(grid.0),
            size.height as f64 / f64::from(grid.1),
        ),
        texel: (2.0 * PI / sky.width() as f64).min(PI / sky.height() as f64),
    }
}

/// The frame prepared as `--undersampled mark` prepares it for a picture of `size` over `sky`.
fn judged(frame: &Frame, size: Size, sky: &SkyMap) -> RayField {
    prepare(
        frame,
        Some(judge((frame.width, frame.height), size, sky)),
        3,
    )
}

/// The frame prepared as `--undersampled interpolate` prepares it.
fn unjudged(frame: &Frame) -> RayField {
    prepare(frame, None, 3)
}

fn identity() -> Mat3 {
    let n = |v: [f64; 3]| v.map(sky_format::Num);
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

/// A map with structure at every scale, down to single texels.
fn noise_map(width: usize, height: usize) -> SkyMap {
    let mut state = 11u64;
    let texels = (0..width * height)
        .map(|_| {
            [0; 3].map(|_| {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                (state >> 40) as f32 / (1u64 << 24) as f32 * 0.5
            })
        })
        .collect();
    SkyMap::new(width, height, texels, 4)
}

fn uniform_map(width: usize, height: usize, value: f32) -> SkyMap {
    SkyMap::new(width, height, vec![[value; 3]; width * height], 4)
}

fn look(unresolved: [u16; 3], undersampled: [u16; 3]) -> Look {
    Look {
        gain: 1.0,
        unresolved,
        undersampled,
        encoder: Encoder::new(),
    }
}

fn size(width: usize) -> Size {
    Size {
        width,
        height: width / 2,
    }
}

fn picture(fields: Fields, sky: &SkyMap, size: Size, look: &Look) -> (Vec<u16>, Tally) {
    let scene = Scene {
        fields,
        rotation: identity(),
        sky,
    };
    let mut out = vec![0; size.width * size.height * 3];
    let tally = render(&scene, size, look, 3, &mut out);
    (out, tally)
}

fn light(fields: Fields, sky: &SkyMap, size: Size) -> Vec<Shade> {
    let scene = Scene {
        fields,
        rotation: identity(),
        sky,
    };
    shades(&scene, size, 1.0, 3)
}

/// The output pixels (i, j) drawn as under-sampled.
fn marked(shades: &[Shade], size: Size) -> Vec<(usize, usize)> {
    (0..shades.len())
        .filter(|&k| shades[k] == Shade::Undersampled)
        .map(|k| (k % size.width, k / size.width))
        .collect()
}

fn pixel(out: &[u16], size: Size, i: usize, j: usize) -> [u16; 3] {
    let k = 3 * (j * size.width + i);
    [out[k], out[k + 1], out[k + 2]]
}

/// A strong, smooth magnification about +x: the conformal map of the sphere that takes the
/// angle theta from +x to 2 atan(k tan(theta / 2)), keeping the azimuth about +x. It is the
/// aberration of a boost toward -x with Doppler factor k: the sky within a few degrees of +x is
/// spread over half the sphere, and neighbouring rays there are k times further apart than on a
/// flat bundle's grid, yet the map is smooth everywhere.
fn magnified(n: [f64; 3], k: f64) -> [f64; 3] {
    let c = [1.0, 0.0, 0.0];
    let theta = angle(n, c);
    let p = [0.0, n[1], n[2]];
    let sin = (p[1] * p[1] + p[2] * p[2]).sqrt();
    if sin < 1e-12 {
        return n;
    }
    let e = p.map(|x| x / sin);
    let f = 2.0 * (k * (theta / 2.0).tan()).atan();
    let (s, co) = f.sin_cos();
    [
        co * c[0] + s * e[0],
        co * c[1] + s * e[1],
        co * c[2] + s * e[2],
    ]
}

/// A jump of a quarter turn about +z between the northern and southern halves of the sky: rows
/// 0 to h/2 - 1 of the grid see the sky as it is, the rest see it turned.
fn torn(n: [f64; 3]) -> [f64; 3] {
    if n[2] > 0.0 { n } else { turned(n, PI / 2.0) }
}

/// The output rows whose pixels lie in the cell of rays between grid rows `h / 2 - 1` and
/// `h / 2`, for `s` output rows per ray: v = (j + 0.5) / s in (h/2 - 0.5, h/2 + 0.5).
fn equator_cell_rows(h: usize, s: usize) -> std::ops::Range<usize> {
    let from = (h / 2 * s) - s / 2;
    from..from + s
}

// ---- what is not marked --------------------------------------------------------------------------

#[test]
fn test_a_smooth_steep_magnification_about_a_point_is_not_marked_anywhere() {
    let (gw, gh) = (128u32, 64u32);
    let frame = frame_of(gw, gh, |n| magnified(n, 10.0), |_| 0);
    let size = size(256);
    let sky = noise_map(256, 128);
    // Steep: near +x neighbouring rays are more than twenty degrees apart on the far sky, where a
    // flat bundle's are 2.8 degrees apart. A rule that marked by the spread of the taps would
    // paint this region, which the footprint filter draws correctly.
    let grid = Grid::new(gw, gh);
    let spread = (0..gh)
        .flat_map(|j| (0..gw - 1).map(move |i| (i, j)))
        .map(|(i, j)| {
            angle(
                magnified(grid.pixel_direction(i, j), 10.0),
                magnified(grid.pixel_direction(i + 1, j), 10.0),
            )
        })
        .fold(0.0f64, f64::max);
    assert!(
        spread.to_degrees() > 20.0,
        "{} degrees",
        spread.to_degrees()
    );
    let field = judged(&frame, size, &sky);
    let shades = light(Fields::One(&field), &sky, size);
    assert_eq!(marked(&shades, size), vec![]);
    // And what is not marked is drawn exactly as the unjudged renderer draws it.
    let plain = unjudged(&frame);
    assert_eq!(shades, light(Fields::One(&plain), &sky, size));
}

#[test]
fn test_flat_bundles_are_never_marked_and_draw_the_same_bits_as_the_unjudged_renderer() {
    let sky = noise_map(128, 64);
    let still = frame_of(64, 32, |n| n, |_| 0);
    let turn_a = frame_of(64, 32, |n| turned(n, 0.3), |_| 0);
    let turn_b = frame_of(64, 32, |n| turned(n, 0.45), |_| 0);
    // One pixel a ray, four pixels a ray, and a picture coarser than the grid.
    for width in [64, 256, 32] {
        let size = size(width);
        let lk = look(RED, RED);
        for frame in [&still, &turn_a] {
            let (a, ta) = picture(Fields::One(&judged(frame, size, &sky)), &sky, size, &lk);
            let (b, _) = picture(Fields::One(&unjudged(frame)), &sky, size, &lk);
            assert_eq!(ta.marked(), 0, "{width} wide");
            assert!(a == b, "{width} wide: the judged picture differs");
        }
        // Between two frames of a turning sky, too.
        let (a, b) = (judged(&turn_a, size, &sky), judged(&turn_b, size, &sky));
        let (c, d) = (unjudged(&turn_a), unjudged(&turn_b));
        let (x, tx) = picture(Fields::Two(&a, &b, 0.3), &sky, size, &lk);
        let (y, _) = picture(Fields::Two(&c, &d, 0.3), &sky, size, &lk);
        assert_eq!(tx.marked(), 0);
        assert!(x == y, "{width} wide: the judged blend differs");
    }
}

#[test]
fn test_a_winding_step_of_one_over_a_continuous_sky_is_not_marked() {
    // Light that swept a little less than a whole turn about Z and light that swept a little
    // more arrive from neighbouring directions: the winding steps, the sky does not.
    let frame = frame_of(64, 32, |n| n, |n| i16::from(n[2] < 0.0));
    let size = size(256);
    let sky = noise_map(256, 128);
    let shades = light(Fields::One(&judged(&frame, size, &sky)), &sky, size);
    assert_eq!(marked(&shades, size), vec![]);
}

// ---- what is marked ------------------------------------------------------------------------------

#[test]
fn test_a_jump_in_direction_along_the_equator_is_marked_in_the_one_cell_across_it_and_nowhere_else()
{
    // Rays four pixels apart: the band is the one cell of rays that straddles the jump, four
    // pixels wide, across the whole width of the frame, and not the cells beside it, whose
    // interpolation is between rays on one side of the jump.
    let (gw, gh) = (64u32, 32u32);
    let frame = frame_of(gw, gh, torn, |_| 0);
    let size = size(256);
    let sky = noise_map(256, 128);
    let shades = light(Fields::One(&judged(&frame, size, &sky)), &sky, size);
    let wanted: Vec<(usize, usize)> = equator_cell_rows(gh as usize, 4)
        .flat_map(|j| (0..size.width).map(move |i| (i, j)))
        .collect();
    let mut got = marked(&shades, size);
    got.sort_by_key(|&(i, j)| (j, i));
    assert_eq!(got, wanted);
    assert_eq!(equator_cell_rows(32, 4), 62..66);
}

#[test]
fn test_a_winding_step_of_two_is_marked_in_the_one_cell_across_it_even_over_a_continuous_sky() {
    // Rays whose light differs by two turns about Z: between them the rays sweep every azimuth,
    // which no four rays resolve, whatever their directions say.
    let (gw, gh) = (64u32, 32u32);
    let frame = frame_of(gw, gh, |n| n, |n| if n[2] < 0.0 { 2 } else { 0 });
    let size = size(256);
    let sky = noise_map(256, 128);
    let shades = light(Fields::One(&judged(&frame, size, &sky)), &sky, size);
    let mut got = marked(&shades, size);
    got.sort_by_key(|&(i, j)| (j, i));
    let wanted: Vec<(usize, usize)> = equator_cell_rows(gh as usize, 4)
        .flat_map(|j| (0..size.width).map(move |i| (i, j)))
        .collect();
    assert_eq!(got, wanted);
}

#[test]
fn test_a_lone_wild_ray_marks_the_two_cells_that_interpolate_from_it() {
    // One ray pointing at the far side of the sky from its neighbours, as a tracer's glitch
    // would: the cells on either side of it along its row and its column interpolate from it
    // and are marked; the cells one further out interpolate only between good rays and are not.
    let (gw, gh) = (64u32, 32u32);
    let mut frame = frame_of(gw, gh, |n| n, |_| 0);
    let k = frame.grid().offset(20, 10);
    let far = turned(frame.grid().pixel_direction(20, 10), 2.5);
    for c in 0..3 {
        frame.direction[c][k] = far[c] as f32;
    }
    let size = size(256);
    let sky = noise_map(256, 128);
    let shades = light(Fields::One(&judged(&frame, size, &sky)), &sky, size);
    let got = marked(&shades, size);
    // Pixels whose cell has ray (20, 10) as a corner: u and v within one ray of its centre.
    for j in 0..size.height {
        for i in 0..size.width {
            let (u, v) = ((i as f64 + 0.5) / 4.0, (j as f64 + 0.5) / 4.0);
            let inside = (u - 20.5).abs() < 1.0 && (v - 10.5).abs() < 1.0;
            assert_eq!(got.contains(&(i, j)), inside, "pixel ({i}, {j})");
        }
    }
}

#[test]
fn test_unresolved_rays_and_undersampled_pixels_take_their_own_colours_and_the_dark_edge_half_cell_is_marked()
 {
    let (gw, gh) = (64u32, 32u32);
    let mut frame = frame_of(gw, gh, torn, |_| 0);
    set_fate(&mut frame, 10, 5, fate::UNRESOLVED);
    for j in 22..26 {
        for i in 40..48 {
            set_fate(&mut frame, i, j, fate::DARK);
        }
    }
    let size = size(256);
    let sky = uniform_map(256, 128, 0.2);
    let field = judged(&frame, size, &sky);
    let (out, _) = picture(Fields::One(&field), &sky, size, &look(BLUE, GREEN));
    let sky_code = Encoder::new().encode(0.2);
    let (mut blue, mut green) = (0, 0);
    for j in 0..size.height {
        for i in 0..size.width {
            let (u, v) = ((i as f64 + 0.5) / 4.0, (j as f64 + 0.5) / 4.0);
            let t = taps(gw as usize, gh as usize, u, v);
            let nearest = frame.fate[t.index[t.nearest()]];
            let short = (0..4).any(|k| t.weight[k] > 0.0 && frame.fate[t.index[k]] != 1);
            let wanted = if nearest == fate::UNRESOLVED {
                BLUE
            } else if nearest == fate::DARK {
                [0; 3]
            } else if short || equator_cell_rows(32, 4).contains(&j) {
                // Beside the unresolved ray or the dark block (half a cell), and across the jump.
                GREEN
            } else {
                [sky_code; 3]
            };
            let got = pixel(&out, size, i, j);
            // The pixel beside a marked one measures its footprint from the other side; over a
            // uniform map that changes nothing.
            assert!(
                (0..3).all(|c| got[c].abs_diff(wanted[c]) <= 1),
                "pixel ({i}, {j}) is {got:?} and should be {wanted:?}"
            );
            blue += usize::from(got == BLUE);
            green += usize::from(got == GREEN);
        }
    }
    // Four by four pixels round the unresolved ray are nearest it; the half-cell ring round it
    // and round the dark block, and the band across the jump, are the marked ones.
    assert_eq!(blue, 16);
    assert!(green > 4 * size.width, "{green}");
    // With both colours the default, red, the two causes look the same.
    let (out, _) = picture(Fields::One(&field), &sky, size, &look(RED, RED));
    assert_eq!(pixel(&out, size, 41, 21), RED); // the unresolved ray
    assert_eq!(pixel(&out, size, 41, 63), RED); // the band across the jump
}

#[test]
fn test_both_marker_colours_default_to_bright_red_and_marking_is_the_default() {
    let args = |extra: &[&str]| {
        let mut a: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--encoder", "none"]
            .map(String::from)
            .to_vec();
        a.extend(extra.iter().map(|s| s.to_string()));
        match crate::cli::parse(&a).expect("parses") {
            crate::cli::Request::Render(o) => o,
            crate::cli::Request::Help => panic!("not help"),
        }
    };
    let o = args(&[]);
    assert_eq!(
        (o.unresolved, o.undersampled, o.mark_undersampled),
        (RED, RED, true)
    );
    let o = args(&[
        "--undersampled",
        "interpolate",
        "--undersampled-colour",
        "00ff00",
        "--unresolved-colour",
        "#0000FF",
    ]);
    assert_eq!(
        (o.unresolved, o.undersampled, o.mark_undersampled),
        (BLUE, GREEN, false)
    );
    assert!(crate::cli::USAGE.contains("(default FF0000)"));
    assert!(!crate::cli::USAGE.contains("FF00FF"));
    // An exposure beyond 100 stops would make the gain infinite or zero.
    let bad = [
        "--bundle",
        "b",
        "--sky",
        "s",
        "--encoder",
        "none",
        "--exposure",
        "130",
    ];
    assert!(crate::cli::parse(&bad.map(String::from)).is_err());
}

// ---- interpolate, the old renderer ---------------------------------------------------------------

#[test]
fn test_interpolate_draws_the_unjudged_picture_and_marking_changes_only_the_marked_pixels_and_their_neighbours()
 {
    let frame = frame_of(64, 32, torn, |_| 0);
    let size = size(256);
    let sky = noise_map(256, 128);
    let lk = look(RED, GREEN);
    // `--undersampled interpolate` reads frames as `from_frame` makes them: the renderer's own
    // arithmetic, which the tests in `tests` pin down and which a build of the commit before
    // marking was compared with, bit for bit, on real bundles.
    let (interpolated, t) = picture(Fields::One(&unjudged(&frame)), &sky, size, &lk);
    let (plain, _) = picture(Fields::One(&RayField::from_frame(&frame)), &sky, size, &lk);
    assert!(interpolated == plain);
    assert_eq!(t.marked(), 0);
    // Marking replaces the band, and the rows just outside it read their footprint from the far
    // side; every other pixel is the same, to the bit.
    let (marked_out, _) = picture(Fields::One(&judged(&frame, size, &sky)), &sky, size, &lk);
    let band = equator_cell_rows(32, 4);
    for j in 0..size.height {
        for i in 0..size.width {
            let (a, b) = (
                pixel(&interpolated, size, i, j),
                pixel(&marked_out, size, i, j),
            );
            if band.contains(&j) {
                assert_eq!(b, GREEN);
                assert_ne!(a, GREEN);
            } else if j + 1 != band.start && j != band.end {
                assert_eq!(a, b, "pixel ({i}, {j})");
            }
        }
    }
}

// ---- between frames ------------------------------------------------------------------------------

#[test]
fn test_between_frames_a_direction_held_from_one_frame_is_marked_and_fate_follows_the_nearer() {
    let sky_ray = Ray::Sky {
        d: [1.0, 0.0, 0.0],
        g: 1.0,
    };
    let other = Ray::Sky {
        d: [0.0, 1.0, 0.0],
        g: 2.0,
    };
    // Both known: blended as ever.
    let Ray::Sky { d, g } = blend_judged(sky_ray, other, 0.5) else {
        panic!("sky")
    };
    assert!((angle(d, [1.0, 1.0, 0.0]) < 1e-12) && (g - 1.5).abs() < 1e-12);
    // Held from one frame: marked, whichever frame is the nearer.
    for w in [0.1, 0.9] {
        for (a, b) in [(sky_ray, Ray::Shadow), (Ray::Shadow, sky_ray)] {
            let nearer = if w <= 0.5 { a } else { b };
            let wanted = if nearer == Ray::Shadow {
                Ray::Shadow
            } else {
                Ray::Undersampled
            };
            assert_eq!(blend_judged(a, b, w), wanted);
        }
        assert_eq!(
            blend_judged(sky_ray, Ray::Undersampled, w),
            Ray::Undersampled
        );
        assert_eq!(blend_judged(Ray::Unresolved, sky_ray, 0.1), Ray::Unresolved);
    }
    // Half a turn between frames: the blend has no direction.
    let back = Ray::Sky {
        d: [-1.0, 0.0, 0.0],
        g: 1.0,
    };
    assert_eq!(blend_judged(sky_ray, back, 0.5), Ray::Undersampled);
}

#[test]
fn test_a_video_frame_before_the_first_complete_bundle_frame_is_known_to_be_before_it() {
    // Frame 0 at proper time 10 is the stopwatch's origin but only frames at 11 and 12 are
    // complete: video frames before stopwatch 1 have no rays.
    let t = Timeline::new(&[11.0, 12.0], 10.0, 2.0, 1.0);
    assert!(t.before_first(0) && t.before_first(1));
    assert!(!t.before_first(2) && !t.before_first(3));
    // A time an ulp before a frame is on it.
    let t = Timeline::new(&[0.1, 0.2], 0.0, 30.0, 3.0);
    assert!(!t.before_first(1));
}

// ---- the shift's g^4 -----------------------------------------------------------------------------

#[test]
fn test_a_shift_beyond_f32_range_gives_black_for_a_black_texel_and_white_for_a_lit_one() {
    let e = Encoder::new();
    for g in [3e9, 1e10, 1e30, 1e100, 1e200] {
        let light = shade([0.0, 0.5, 1e-7], g, 1.0);
        assert_eq!(light[0], 0.0, "g = {g}");
        assert!(light[1] > 1.0 && light[2] > 1.0, "g = {g}: {light:?}");
        assert_eq!(light.map(|c| e.encode(c)), [0, 65_535, 65_535], "g = {g}");
    }
    // A gain that pushes a finite g^4 past f32's range is the same case.
    let light = shade([0.0, 0.5, 0.0], 1e9, 1e6);
    assert_eq!(light.map(|c| e.encode(c)), [0, 65_535, 0]);
    // A shift so small that g^4 underflows f32: black, and a number.
    for g in [1e-12, 1e-40, 1e-100] {
        let light = shade([0.0, 0.5, 1.0], g, 1.0);
        assert!(
            light.iter().all(|c| c.is_finite() && *c >= 0.0),
            "{light:?}"
        );
        assert_eq!(light.map(|c| e.encode(c)), [0; 3]);
    }
    // Every ordinary shift is computed exactly as before.
    for g in [1e-5, 0.3, 1.0, 1.2247449, 389.0, 1e9] {
        for gain in [1.0f32, 5.656854, 0.01] {
            let old = ((g * g * g * g) as f32 * gain, 0.37f32);
            assert_eq!(shade([old.1; 3], g, gain)[0], old.1 * old.0);
        }
    }
}

#[test]
fn test_a_frame_under_an_enormous_shift_is_black_on_black_sky_and_white_on_lit_sky_never_unresolved()
 {
    let mut frame = frame_of(32, 16, |n| n, |_| 0);
    frame.shift.fill(1e12);
    let size = size(64);
    for (value, code) in [(0.0, 0), (0.2, 65_535)] {
        let sky = uniform_map(64, 32, value);
        let (out, tally) = picture(
            Fields::One(&judged(&frame, size, &sky)),
            &sky,
            size,
            &look(RED, RED),
        );
        assert_eq!(tally.unresolved, 0);
        assert!(out.iter().all(|&c| c == code), "a sky of {value}");
    }
}

// ---- counting ------------------------------------------------------------------------------------

#[test]
fn test_the_counts_agree_with_a_direct_count_of_the_frames_pixels_and_name_the_colour() {
    let (gw, gh) = (64u32, 32u32);
    let mut frame = frame_of(gw, gh, torn, |_| 0);
    set_fate(&mut frame, 10, 5, fate::UNRESOLVED);
    set_fate(&mut frame, 11, 5, 99); // reserved: unresolved
    for j in 22..26 {
        for i in 40..48 {
            set_fate(&mut frame, i, j, fate::DARK);
        }
    }
    let size = size(256);
    let sky = uniform_map(256, 128, 0.2);
    let field = judged(&frame, size, &sky);
    let (out, tally) = picture(Fields::One(&field), &sky, size, &look(BLUE, GREEN));
    let count = |colour: [u16; 3]| {
        out.as_chunks::<3>()
            .0
            .iter()
            .filter(|p| **p == colour)
            .count() as u64
    };
    assert_eq!(tally.pixels, (size.width * size.height) as u64);
    assert_eq!(tally.unresolved, count(BLUE));
    assert_eq!(tally.undersampled, count(GREEN));
    assert_eq!(tally.dark, count([0; 3]));
    assert_eq!(tally.unresolved, 32);
    assert_eq!(tally.dark, 8 * 4 * 16);
    // Over a film: the totals add, and the worst frame is the one with the most marked pixels.
    let plain = frame_of(gw, gh, |n| n, |_| 0);
    let (_, clean) = picture(
        Fields::One(&judged(&plain, size, &sky)),
        &sky,
        size,
        &look(RED, RED),
    );
    let mut film = FilmTally::default();
    film.add(0, clean);
    film.add(1, tally);
    film.add(2, clean);
    assert_eq!(film.total.pixels, 3 * tally.pixels);
    assert_eq!(film.total.undersampled, tally.undersampled);
    assert_eq!(film.worst, Some((1, tally)));
    // The sentence that explains the colour, once for one colour and once each for two.
    let one = film.report(RED, RED, true).join("\n");
    assert!(
        one.contains("Pixels drawn in #FF0000 (red) are ones this program does not know"),
        "{one}"
    );
    assert_eq!(one.matches("Pixels drawn in").count(), 1);
    let two = film.report(BLUE, GREEN, true).join("\n");
    assert!(
        two.contains("#0000FF (blue)") && two.contains("#00FF00 (green)"),
        "{two}"
    );
    // A clean film says nothing about colours.
    let mut quiet = FilmTally::default();
    quiet.add(0, clean);
    assert!(
        !quiet
            .report(RED, RED, true)
            .join("\n")
            .contains("Pixels drawn in")
    );
}
