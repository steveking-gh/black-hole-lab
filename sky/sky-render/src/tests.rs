//! The renderer's promises, checked on grids of a few dozen pixels.
//!
//! Every picture here is small enough to compute in microseconds and to reason about pixel by
//! pixel. Bundles are built in memory from `sky_format::Frame`, the way a writer builds them, and
//! go through the same `RayField` the renderer reads from disk; only the end-to-end test through
//! ffmpeg touches the file system.

// The loops over 0..3 are over the components of a vector or a colour, several arrays at once,
// and read as the formulae they check; the iterator forms clippy suggests would not.
#![allow(clippy::needless_range_loop)]

use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use sky_format::{Axes, FarSky, Frame, Grid, Num, fate};

use crate::bilinear::taps;
use crate::field::{Ray, RayField, blend, norm};
use crate::render::{Fields, Look, Scene, Shade, Size, render, shades};
use crate::sky::{MapFrame, Mat3, SkyMap, orientation};
use crate::timeline::{Pick, Timeline};
use crate::tone::{Encoder, default_exposure_stops, srgb_encode};

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
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Rotates `n` by `angle` about +z: toward +y, the observer's left.
fn turned(n: [f64; 3], angle: f64) -> [f64; 3] {
    let (s, c) = angle.sin_cos();
    [n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]]
}

/// A bundle frame of the flat-space `still` case, with the far sky turned by `turn` about z and
/// every shift `g`: each ray's direction at infinity is the direction it is traced along.
fn still_frame(width: u32, height: u32, turn: f64, g: f32) -> Frame {
    let grid = Grid::new(width, height);
    let mut frame = Frame::new(width, height, 0);
    for j in 0..height {
        for i in 0..width {
            let k = grid.offset(i, j);
            let d = turned(grid.pixel_direction(i, j), turn);
            frame.fate[k] = fate::FAR_SKY;
            for c in 0..3 {
                frame.direction[c][k] = d[c] as f32;
            }
            frame.shift[k] = g;
        }
    }
    frame
}

fn still(width: u32, height: u32, turn: f64, g: f32) -> RayField {
    RayField::from_frame(&still_frame(width, height, turn, g))
}

/// The far-sky frame lined up with ICRS, so that over a celestial map the rotation is exactly the
/// identity and a picture can be compared with the map texel for texel.
fn icrs_far_sky() -> FarSky {
    let n = |v: [f64; 3]| v.map(Num);
    FarSky {
        name: "icrs".into(),
        axes_in_icrs: Axes {
            x: n([1.0, 0.0, 0.0]),
            y: n([0.0, 1.0, 0.0]),
            z: n([0.0, 0.0, 1.0]),
        },
    }
}

fn identity() -> Mat3 {
    orientation(&icrs_far_sky(), MapFrame::Celestial)
}

/// Deterministic values in [0, 1): a map with structure at every scale, down to single texels.
fn noise(count: usize, seed: u64) -> Vec<f32> {
    let mut state = seed;
    (0..count)
        .map(|_| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (state >> 40) as f32 / (1u64 << 24) as f32
        })
        .collect()
}

fn noise_map(width: usize, height: usize, scale: f32) -> SkyMap {
    let v = noise(3 * width * height, 7);
    let texels = v
        .chunks(3)
        .map(|c| [c[0] * scale, c[1] * scale, c[2] * scale])
        .collect();
    SkyMap::new(width, height, texels, 4)
}

fn uniform_map(width: usize, height: usize, value: f32) -> SkyMap {
    SkyMap::new(width, height, vec![[value; 3]; width * height], 4)
}

fn look(gain: f32) -> Look {
    Look {
        gain,
        unresolved: [65_535, 0, 65_535],
        encoder: Encoder::new(),
    }
}

fn size(width: usize) -> Size {
    Size {
        width,
        height: width / 2,
    }
}

/// Renders one frame to 16-bit codes.
fn picture(fields: Fields, rotation: Mat3, sky: &SkyMap, size: Size, gain: f32) -> Vec<u16> {
    let scene = Scene {
        fields,
        rotation,
        sky,
    };
    let mut out = vec![0; size.width * size.height * 3];
    render(&scene, size, &look(gain), 3, &mut out);
    out
}

/// Renders one frame to linear light, before clipping.
fn light(fields: Fields, rotation: Mat3, sky: &SkyMap, size: Size, gain: f32) -> Vec<Shade> {
    let scene = Scene {
        fields,
        rotation,
        sky,
    };
    shades(&scene, size, gain, 3)
}

fn rgb(shade: Shade) -> [f32; 3] {
    match shade {
        Shade::Light(c) => c,
        other => panic!("expected light, found {other:?}"),
    }
}

/// The 16-bit code the tone curve should give linear light `x`, computed the slow exact way.
fn exact_code(x: f32) -> u16 {
    (srgb_encode(f64::from(x).clamp(0.0, 1.0)) * 65_535.0).round() as u16
}

fn angle_between(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    norm(cross).atan2(dot)
}

// ---- the picture ---------------------------------------------------------------------------------

#[test]
fn test_the_still_bundle_over_a_map_of_its_own_size_reproduces_the_map_after_the_tone_curve() {
    let (w, h) = (64, 32);
    let sky = noise_map(w, h, 0.8);
    let field = still(w as u32, h as u32, 0.0, 1.0);
    let gain = 1.5;
    let out = picture(Fields::One(&field), identity(), &sky, size(w), gain);
    let mut worst = 0;
    for (k, texel) in sky.texels().iter().enumerate() {
        for c in 0..3 {
            let wanted = exact_code(texel[c] * gain);
            worst = worst.max(out[3 * k + c].abs_diff(wanted));
        }
    }
    // One code in 65,535: the table's interpolation and the f32 arithmetic of the bilinear
    // weights may round the other way from the exact formula, and nothing more.
    assert!(worst <= 1, "a pixel is {worst} codes from the map's");
}

#[test]
fn test_a_turn_about_the_pole_by_whole_pixels_moves_the_picture_that_many_pixels_and_wraps() {
    let (w, h) = (64, 32);
    let sky = noise_map(w, h, 0.5);
    let base = picture(
        Fields::One(&still(64, 32, 0.0, 1.0)),
        identity(),
        &sky,
        size(w),
        1.0,
    );
    for shift in [5i64, -3, 64 + 7] {
        // Turning the far sky toward +y (the observer's left) by `shift` pixels' worth of
        // azimuth: a star at azimuth psi is then seen by the pixel that looks along psi - turn,
        // further right in the frame. The picture moves right, wrapping at the seam.
        let turn = shift as f64 * 2.0 * PI / w as f64;
        let field = still(64, 32, turn, 1.0);
        let out = picture(Fields::One(&field), identity(), &sky, size(w), 1.0);
        for j in 0..h {
            for i in 0..w {
                let from = (i as i64 - shift).rem_euclid(w as i64) as usize;
                for c in 0..3 {
                    let (a, b) = (out[3 * (j * w + i) + c], base[3 * (j * w + from) + c]);
                    assert!(
                        a.abs_diff(b) <= 1,
                        "turn {shift}: pixel ({i}, {j}) is {a} and should be pixel ({from}, {j})'s {b}"
                    );
                }
            }
        }
    }
}

#[test]
fn test_a_coarse_bundle_grid_gives_the_right_directions_at_the_poles_and_across_the_seam() {
    // 16 x 8 rays for a 64 x 32 frame: every ray serves 16 pixels, and the top and bottom rows of
    // the frame lie above the first row of rays and below the last, where only the pole rule
    // gives them a ray on the far side.
    let coarse = still(16, 8, 0.0, 1.0);
    let frame = Grid::new(64, 32);
    let pixel = 2.0 * PI / 64.0;
    let mut error = vec![[0.0f64; 64]; 32];
    for (j, row) in error.iter_mut().enumerate() {
        for (i, e) in row.iter_mut().enumerate() {
            let u = (i as f64 + 0.5) * 16.0 / 64.0;
            let v = (j as f64 + 0.5) * 8.0 / 32.0;
            let Ray::Sky { d, g } = coarse.sample(u, v) else {
                panic!("pixel ({i}, {j}) is not sky")
            };
            assert!((g - 1.0).abs() < 1e-6);
            *e = angle_between(d, frame.pixel_direction(i as u32, j as u32)) / pixel;
        }
    }
    let worst = |rows: &[usize]| {
        rows.iter()
            .flat_map(|&j| error[j].iter().copied())
            .fold(0.0f64, f64::max)
    };
    // Interpolating unit vectors 22.5 degrees apart and renormalising is not exact: the chord
    // falls inside the sphere, and a component-wise mean of two directions at one latitude lies
    // poleward of it. Measured 0.098 of a pixel at worst, at mid-latitudes.
    let all: Vec<usize> = (0..32).collect();
    assert!(
        worst(&all) < 0.11,
        "a pixel is {:.3} pixels off",
        worst(&all)
    );
    // The pole rows are better than that (measured 0.003): the rows across the pole are the
    // right rays. Clamping to the first row instead would put the top row one and a half pixels
    // off.
    assert!(
        worst(&[0, 31]) < 0.01,
        "a pole row is {:.3} pixels off",
        worst(&[0, 31])
    );
    // The seam is not a place of its own: column 0 sits in its cell of rays exactly as column 32
    // does in its, and column 63 as column 31, and their errors are the same.
    for row in &error {
        assert!((row[0] - row[32]).abs() < 1e-9 && (row[63] - row[31]).abs() < 1e-9);
    }
}

#[test]
fn test_a_coarse_bundle_grid_draws_the_picture_of_a_fine_one_including_poles_and_seam() {
    // A smooth sky - the colour is linear in the direction - so a difference in the picture is a
    // difference in direction, not the map's own detail.
    let (mw, mh) = (256, 128);
    let map_grid = Grid::new(mw as u32, mh as u32);
    let texels = (0..mh as u32)
        .flat_map(|j| (0..mw as u32).map(move |i| (i, j)))
        .map(|(i, j)| {
            let n = map_grid.pixel_direction(i, j);
            [0, 1, 2].map(|c| (0.5 + 0.4 * n[c]) as f32)
        })
        .collect();
    let sky = SkyMap::new(mw, mh, texels, 4);
    let fine = still(64, 32, 0.0, 1.0);
    let coarse = still(16, 8, 0.0, 1.0);
    let a = light(Fields::One(&fine), identity(), &sky, size(64), 1.0);
    let b = light(Fields::One(&coarse), identity(), &sky, size(64), 1.0);
    let mut worst: f32 = 0.0;
    for (x, y) in a.iter().zip(&b) {
        for c in 0..3 {
            worst = worst.max((rgb(*x)[c] - rgb(*y)[c]).abs());
        }
    }
    // The colour changes by 0.4 per radian, so 0.004 is a hundredth of a radian, a tenth of a
    // pixel of this frame; the direction test above bounds the error at a twentieth. What is left
    // is the filter: the coarse grid's footprints differ slightly from the fine one's.
    assert!(worst < 0.004, "the pictures differ by {worst}");
}

#[test]
fn test_the_pole_rule_puts_equal_weight_on_both_sides_of_the_pole() {
    let t = taps(8, 4, 1.5, 0.0);
    // At v = 0 the row "above" is row 0 half a turn round: column 1 + 4 = 5.
    assert_eq!(t.index[0], 5);
    assert_eq!(t.index[2], 1);
    assert!((t.weight[0] - 0.5).abs() < 1e-12 && (t.weight[2] - 0.5).abs() < 1e-12);
    // And at the bottom pole, row 3 half a turn round.
    let t = taps(8, 4, 6.5, 4.0);
    assert_eq!(t.index[0], 3 * 8 + 6);
    assert_eq!(t.index[2], 3 * 8 + 2);
    // The seam: between the last column's centre and the right edge, columns 7 and 0.
    let t = taps(8, 4, 7.75, 1.5);
    assert_eq!((t.index[0], t.index[1]), (8 + 7, 8));
    assert!((t.weight[1] - 0.25).abs() < 1e-12);
}

// ---- fate ----------------------------------------------------------------------------------------

#[test]
fn test_shadow_is_black_unresolved_is_the_marker_and_the_shadow_edge_uses_only_sky_rays() {
    let (bw, bh) = (16u32, 8u32);
    let grid = Grid::new(bw, bh);
    let mut frame = still_frame(bw, bh, 0.0, 1.0);
    let mut set = |i: u32, j: u32, code: u8| {
        let k = grid.offset(i, j);
        frame.fate[k] = code;
        for c in 0..3 {
            frame.direction[c][k] = f32::NAN;
        }
        frame.shift[k] = f32::NAN;
    };
    for j in 3..5 {
        for i in 6..10 {
            set(i, j, fate::PAST_HORIZON);
        }
    }
    set(2, 2, fate::UNRESOLVED);
    set(13, 6, 77); // a reserved code: treated as unresolved
    let field = RayField::from_frame(&frame);
    let sky = uniform_map(32, 16, 0.2);
    let (w, h) = (64usize, 32usize);
    let out = picture(Fields::One(&field), identity(), &sky, size(w), 1.0);
    let sky_code = exact_code(0.2);
    for j in 0..h {
        for i in 0..w {
            // Four pixels a ray, so the nearest ray is (i / 4, j / 4).
            let (ri, rj) = (i / 4, j / 4);
            let wanted = if (6..10).contains(&ri) && (3..5).contains(&rj) {
                [0, 0, 0]
            } else if (ri, rj) == (2, 2) || (ri, rj) == (13, 6) {
                [65_535, 0, 65_535]
            } else {
                // Sky, including every pixel beside the shadow or the unresolved ray, which reads
                // one of them as a neighbour: had a NaN or a zero weight got in, the uniform sky
                // would come out dark or black.
                [sky_code; 3]
            };
            let k = 3 * (j * w + i);
            let got = [out[k], out[k + 1], out[k + 2]];
            let tolerance = if wanted[0] == sky_code { 1 } else { 0 };
            assert!(
                (0..3).all(|c| got[c].abs_diff(wanted[c]) <= tolerance),
                "pixel ({i}, {j}) is {got:?} and should be {wanted:?}"
            );
        }
    }
    // Beside the shadow's left edge the direction is the renormalised mean of the sky rays alone.
    let (u, v) = (23.5 * 16.0 / 64.0, 13.5 * 8.0 / 32.0);
    let t = taps(16, 8, u, v);
    let mut sum = [0.0; 3];
    for k in 0..4 {
        let (i, j) = ((t.index[k] % 16) as u32, (t.index[k] / 16) as u32);
        if frame.fate[t.index[k]] == fate::FAR_SKY {
            let n = grid.pixel_direction(i, j);
            for c in 0..3 {
                sum[c] += t.weight[k] * n[c];
            }
        }
    }
    let wanted = sum.map(|c| c / norm(sum));
    let Ray::Sky { d, g } = field.sample(u, v) else {
        panic!("the pixel beside the shadow is sky")
    };
    assert!(angle_between(d, wanted) < 1e-6, "{d:?} and {wanted:?}");
    assert!((g - 1.0).abs() < 1e-6);
}

// ---- brightness ----------------------------------------------------------------------------------

#[test]
fn test_a_shift_of_two_is_sixteen_times_brighter_in_linear_light_before_clipping() {
    let sky = noise_map(64, 32, 0.1);
    let one = light(
        Fields::One(&still(64, 32, 0.0, 1.0)),
        identity(),
        &sky,
        size(64),
        1.0,
    );
    let two = light(
        Fields::One(&still(64, 32, 0.0, 2.0)),
        identity(),
        &sky,
        size(64),
        1.0,
    );
    for (a, b) in one.iter().zip(&two) {
        for c in 0..3 {
            let (a, b) = (rgb(*a)[c], rgb(*b)[c]);
            // Up to 1.6, well past the clip at 1: the factor is applied before clipping.
            assert!(
                (b - 16.0 * a).abs() <= 1e-6 * b.max(1e-6),
                "{b} is not 16 x {a}"
            );
        }
    }
}

#[test]
fn test_the_default_exposure_is_two_and_a_half_stops_at_8k_and_follows_the_pixel_area() {
    assert_eq!(default_exposure_stops(8192), 2.5);
    assert_eq!(default_exposure_stops(4096), 0.5);
    assert_eq!(default_exposure_stops(16384), 4.5);
}

#[test]
fn test_the_tone_table_agrees_with_the_srgb_formula_to_one_code() {
    let e = Encoder::new();
    for k in 0..=100_000 {
        let x = k as f32 / 80_000.0; // runs past 1 into the clip
        assert!(e.encode(x).abs_diff(exact_code(x)) <= 1, "at {x}");
    }
    assert_eq!(e.encode(f32::NAN), 0);
    assert_eq!(e.encode(-1.0), 0);
    assert_eq!(e.encode(7.0), 65_535);
}

// ---- orientation ---------------------------------------------------------------------------------

/// A celestial map whose colour is its own frame coordinates: red u / W, green v / H. Bilinear
/// interpolation of a linear ramp is exact, so the colour a pixel reads says where it read.
fn coordinate_map(width: usize, height: usize) -> SkyMap {
    let texels = (0..height)
        .flat_map(|j| (0..width).map(move |i| (i, j)))
        .map(|(i, j)| {
            [
                ((i as f64 + 0.5) / width as f64) as f32,
                ((j as f64 + 0.5) / height as f64) as f32,
                0.0,
            ]
        })
        .collect();
    SkyMap::new(width, height, texels, 4)
}

/// A bundle frame in which every ray came from the far-sky direction `d`.
fn all_from(d: [f64; 3]) -> RayField {
    let mut frame = Frame::new(4, 2, 0);
    frame.fate.fill(fate::FAR_SKY);
    for c in 0..3 {
        frame.direction[c].fill(d[c] as f32);
    }
    frame.shift.fill(1.0);
    RayField::from_frame(&frame)
}

#[test]
fn test_the_galactic_preset_over_a_celestial_map_reads_where_the_sky_readme_predicts() {
    let (w, h) = (720usize, 360usize);
    let sky = coordinate_map(w, h);
    let rotation = orientation(&FarSky::galactic(), MapFrame::Celestial);
    // Right ascension and declination (ICRS, degrees) of the galactic centre (l = 0, b = 0) and
    // of the north galactic pole, which defines the galactic frame. Typed in from the Hipparcos
    // definition, not computed from the matrix under test.
    let cases = [
        (
            "the galactic centre",
            [1.0, 0.0, 0.0],
            266.404_988_29,
            -28.936_177_76,
        ),
        (
            "the north galactic pole",
            [0.0, 0.0, 1.0],
            192.859_48,
            27.128_25,
        ),
    ];
    for (name, d, ra, dec) in cases {
        // sky/maps/README.md: longitude = 180 - (i + 0.5) 360 / W, so u = W (180 - RA) / 360,
        // with RA taken into (-180, 180]; latitude = 90 - (j + 0.5) 180 / H.
        let ra = if ra > 180.0 { ra - 360.0 } else { ra };
        let (u, v) = (
            w as f64 * (180.0 - ra) / 360.0,
            h as f64 * (90.0 - dec) / 180.0,
        );
        let field = all_from(d);
        let shades = light(
            Fields::One(&field),
            rotation,
            &sky,
            Size {
                width: 4,
                height: 2,
            },
            1.0,
        );
        let [r, g, _] = rgb(shades[0]);
        let (read_u, read_v) = (f64::from(r) * w as f64, f64::from(g) * h as f64);
        assert!(
            (read_u - u).abs() < 2e-3 && (read_v - v).abs() < 2e-3,
            "{name} reads at ({read_u:.4}, {read_v:.4}), and the README puts it at ({u:.4}, {v:.4})"
        );
    }
}

#[test]
fn test_the_galactic_preset_over_a_galactic_map_is_the_identity_to_rounding() {
    let m = orientation(&FarSky::galactic(), MapFrame::Galactic);
    for r in 0..3 {
        for c in 0..3 {
            let wanted = if r == c { 1.0 } else { 0.0 };
            assert!(
                (m[r][c] - wanted).abs() < 1e-14,
                "entry ({r}, {c}) is {}",
                m[r][c]
            );
        }
    }
}

#[test]
fn test_the_map_frame_is_read_from_nasas_file_names() {
    assert_eq!(
        MapFrame::from_file_name(Path::new("sky/maps/starmap_2020_8k_gal.exr")),
        MapFrame::Galactic
    );
    assert_eq!(
        MapFrame::from_file_name(Path::new("starmap_2020_16k.exr")),
        MapFrame::Celestial
    );
    assert_eq!(
        MapFrame::from_file_name(Path::new("milkyway_2020_4k_gal.exr")),
        MapFrame::Galactic
    );
}

// ---- filtering -----------------------------------------------------------------------------------

#[test]
fn test_single_pixel_columns_at_half_resolution_come_out_uniform_grey_not_aliased() {
    // Columns alternately 1 and 0, drawn into a frame half as wide. The far sky is turned by half
    // a texel, so that every pixel centre lands exactly on a bright texel's centre: a reader that
    // does not filter draws the whole frame white.
    let (mw, mh) = (128usize, 64usize);
    let texels = (0..mw * mh)
        .map(|k| [if k % 2 == 0 { 1.0 } else { 0.0 }; 3])
        .collect();
    let sky = SkyMap::new(mw, mh, texels, 4);
    let turn = 0.5 * 2.0 * PI / mw as f64;
    let field = still(64, 32, turn, 1.0);
    let out = light(Fields::One(&field), identity(), &sky, size(64), 1.0);
    for (k, s) in out.iter().enumerate() {
        let [r, g, b] = rgb(*s);
        assert!(
            [r, g, b].iter().all(|c| (c - 0.5).abs() < 1e-3),
            "pixel {k} is {r}, not grey"
        );
    }
}

#[test]
fn test_a_uniform_map_stays_uniform_at_every_level_of_the_rip_map() {
    // Odd sizes on the way down (100, 50, 25, 12, ...) and the solid-angle weights toward the
    // poles must all be weights that add up to one.
    let sky = uniform_map(100, 50, 0.25);
    for (across, down) in [
        (1.0, 1.0),
        (3.0, 1.0),
        (1.0, 7.0),
        (40.0, 20.0),
        (100.0, 50.0),
    ] {
        for (u, v) in [(0.1, 0.1), (50.0, 25.0), (99.9, 49.9), (13.3, 3.0)] {
            let f = crate::sky::Footprint { u, v, across, down };
            let c = sky.sample(f);
            assert!(c.iter().all(|x| (x - 0.25).abs() < 1e-6), "{c:?} at {f:?}");
        }
    }
}

// ---- time ----------------------------------------------------------------------------------------

#[test]
fn test_video_frames_fall_between_or_on_bundle_frames_and_hold_at_the_ends() {
    // Bundle frames at stopwatch 0, 1 and 3; two video frames a second; one second of proper
    // time per second of video.
    let t = Timeline::new(&[10.0, 11.0, 13.0], 10.0, 2.0, 1.0);
    assert_eq!(t.video_frames(), 7);
    assert_eq!(t.pick(0), Pick::One(0));
    assert_eq!(t.pick(1), Pick::Two(0, 1, 0.5));
    assert_eq!(t.pick(2), Pick::One(1));
    assert_eq!(t.pick(4), Pick::Two(1, 2, 0.5));
    assert_eq!(t.pick(6), Pick::One(2));
    assert_eq!(t.pick(9), Pick::One(2));
    // A stopwatch origin before the first frame holds the first frame.
    let t = Timeline::new(&[10.0, 11.0], 9.0, 1.0, 1.0);
    assert_eq!(t.pick(0), Pick::One(0));
    // A time an ulp off a frame's is on it.
    let t = Timeline::new(&[0.0, 0.1, 0.2], 0.0, 30.0, 3.0);
    assert_eq!(t.pick(1), Pick::One(1));
}

#[test]
fn test_a_video_frame_midway_between_bundle_frames_is_their_interpolation() {
    let sky = noise_map(64, 32, 0.05);
    // The shift: g = 1 and g = 3 blend to g = 2 at the middle, so the light is 16 times g = 1's.
    let (a, b) = (still(64, 32, 0.0, 1.0), still(64, 32, 0.0, 3.0));
    let base = light(Fields::One(&a), identity(), &sky, size(64), 1.0);
    let mid = light(Fields::Two(&a, &b, 0.5), identity(), &sky, size(64), 1.0);
    for (x, y) in base.iter().zip(&mid) {
        for c in 0..3 {
            let (x, y) = (rgb(*x)[c], rgb(*y)[c]);
            assert!((y - 16.0 * x).abs() <= 1e-5 * y.max(1e-6));
        }
    }
    // The direction: halfway between the rays of two frames, equally far from each and in the
    // plane they span.
    let pixel = 2.0 * PI / 64.0;
    let (c, d) = (still(64, 32, 0.0, 1.0), still(64, 32, 2.0 * pixel, 1.0));
    for (u, v) in [(10.5, 3.5), (0.25, 0.1), (40.0, 31.9)] {
        let (Ray::Sky { d: x, .. }, Ray::Sky { d: y, .. }, Ray::Sky { d: m, .. }) = (
            c.sample(u, v),
            d.sample(u, v),
            blend(c.sample(u, v), d.sample(u, v), 0.5),
        ) else {
            panic!("all sky")
        };
        assert!((angle_between(x, m) - angle_between(m, y)).abs() < 1e-9);
        assert!((angle_between(x, m) + angle_between(m, y) - angle_between(x, y)).abs() < 1e-9);
    }
    // Where the two frames disagree about a ray's fate, the nearer frame decides.
    let sky_ray = Ray::Sky {
        d: [1.0, 0.0, 0.0],
        g: 1.0,
    };
    assert_eq!(blend(sky_ray, Ray::Shadow, 0.3), sky_ray);
    assert_eq!(blend(sky_ray, Ray::Shadow, 0.7), Ray::Shadow);
}

// ---- the command line ----------------------------------------------------------------------------

#[test]
fn test_a_size_not_twice_as_wide_as_high_is_refused() {
    assert_eq!(crate::cli::parse_size("8192x4096"), Ok((8192, 4096)));
    assert!(crate::cli::parse_size("8192x4000").is_err());
    assert!(crate::cli::parse_size("64").is_err());
    assert_eq!(crate::cli::parse_range("3..10"), Ok(3..10));
    assert!(crate::cli::parse_range("10..3").is_err());
}

// ---- the 360 tag ---------------------------------------------------------------------------------

fn mp4_box(kind: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let mut out = ((8 + body.len()) as u32).to_be_bytes().to_vec();
    out.extend(kind);
    out.extend(body);
    out
}

/// The same box with a 64-bit size.
fn mp4_box_64(kind: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let mut out = 1u32.to_be_bytes().to_vec();
    out.extend(kind);
    out.extend(((16 + body.len()) as u64).to_be_bytes());
    out.extend(body);
    out
}

fn hdlr(handler: &[u8; 4]) -> Vec<u8> {
    let mut body = vec![0u8; 8]; // version and flags, pre_defined
    body.extend(handler);
    body.extend([0u8; 12]);
    body.extend(b"Handler\0");
    mp4_box(b"hdlr", &body)
}

fn trak(handler: &[u8; 4], wide: bool) -> Vec<u8> {
    let mdia = mp4_box(
        b"mdia",
        &[mp4_box(b"mdhd", &[0; 24]), hdlr(handler)].concat(),
    );
    let body = [mp4_box(b"tkhd", &[0; 84]), mdia].concat();
    if wide {
        mp4_box_64(b"trak", &body)
    } else {
        mp4_box(b"trak", &body)
    }
}

struct Layout {
    faststart: bool,
    wide_moov: bool,
    wide_trak: bool,
    video: bool,
}

const PLAIN: Layout = Layout {
    faststart: false,
    wide_moov: false,
    wide_trak: false,
    video: true,
};

/// A minimal MP4: ftyp, mdat, and a moov holding a sound track and (usually) a video track.
fn sample_mp4(layout: &Layout) -> Vec<u8> {
    let ftyp = mp4_box(b"ftyp", b"isom\0\0\x02\0isomav01");
    let media: Vec<u8> = (0..4000u32).map(|k| (k * 37 % 251) as u8).collect();
    let mdat = mp4_box(b"mdat", &media);
    let mut tracks = vec![mp4_box(b"mvhd", &[0; 100]), trak(b"soun", false)];
    if layout.video {
        tracks.push(trak(b"vide", layout.wide_trak));
    }
    let body = tracks.concat();
    let moov = if layout.wide_moov {
        mp4_box_64(b"moov", &body)
    } else {
        mp4_box(b"moov", &body)
    };
    if layout.faststart {
        [ftyp, moov, mdat].concat()
    } else {
        [ftyp, mdat, moov].concat()
    }
}

fn be32(bytes: &[u8], at: usize) -> usize {
    u32::from_be_bytes(bytes[at..at + 4].try_into().unwrap()) as usize
}

#[test]
fn test_the_360_tag_goes_inside_the_video_trak_with_the_sizes_grown_and_mdat_untouched() {
    use crate::mp4::{SPHERICAL_UUID, spherical_box, tag_spherical};
    let before = sample_mp4(&PLAIN);
    let after = tag_spherical(&before).expect("a plain file is tagged");
    let tag = spherical_box();
    assert_eq!(after.len(), before.len() + tag.len());

    // ftyp and mdat come first and are byte for byte the same.
    let ftyp = be32(&before, 0);
    let mdat_end = ftyp + be32(&before, ftyp);
    assert_eq!(&after[..mdat_end], &before[..mdat_end]);
    assert_eq!(&after[ftyp + 4..ftyp + 8], b"mdat");

    // moov is the rest of the file and says so.
    let moov = mdat_end;
    assert_eq!(&after[moov + 4..moov + 8], b"moov");
    assert_eq!(be32(&after, moov), after.len() - moov);

    // Inside moov: mvhd, the sound trak unchanged, then the video trak, now ending in the tag.
    let mut at = moov + 8;
    let mut kinds = Vec::new();
    while at < after.len() {
        let size = be32(&after, at);
        kinds.push((at, size, after[at + 4..at + 8].to_vec()));
        at += size;
    }
    assert_eq!(at, after.len(), "the boxes in moov add up to moov");
    let (video, size, kind) = kinds.last().unwrap().clone();
    assert_eq!(kind, b"trak");
    let sound = &kinds[1];
    assert_eq!(
        &after[sound.0..sound.0 + sound.1],
        &before[sound.0..sound.0 + sound.1]
    );
    let tag_at = video + size - tag.len();
    assert_eq!(&after[tag_at..video + size], &tag[..]);
    assert_eq!(&after[tag_at + 4..tag_at + 8], b"uuid");
    assert_eq!(&after[tag_at + 8..tag_at + 24], &SPHERICAL_UUID);
    assert_eq!(be32(&after, tag_at), tag.len());
    let xml = String::from_utf8(after[tag_at + 24..video + size].to_vec()).unwrap();
    assert!(xml.contains("<GSpherical:ProjectionType>equirectangular</GSpherical:ProjectionType>"));
    // The video trak's children still add up to it.
    let mut inner = video + 8;
    while inner < video + size {
        inner += be32(&after, inner);
    }
    assert_eq!(inner, video + size);

    // Tagging twice is refused.
    assert!(tag_spherical(&after).unwrap_err().contains("already"));
}

#[test]
fn test_the_360_tag_refuses_what_it_cannot_do_safely() {
    use crate::mp4::tag_spherical;
    let cases = [
        (
            Layout {
                faststart: true,
                ..PLAIN
            },
            "before its media data",
        ),
        (
            Layout {
                wide_moov: true,
                ..PLAIN
            },
            "moov box has a 64-bit size",
        ),
        (
            Layout {
                wide_trak: true,
                ..PLAIN
            },
            "trak box has a 64-bit size",
        ),
        (
            Layout {
                video: false,
                ..PLAIN
            },
            "no video track",
        ),
    ];
    for (layout, why) in cases {
        let message = tag_spherical(&sample_mp4(&layout)).unwrap_err();
        assert!(message.contains(why), "{message:?} does not say {why:?}");
    }
    let mut cut = sample_mp4(&PLAIN);
    cut.truncate(cut.len() - 10);
    assert!(tag_spherical(&cut).unwrap_err().contains("does not fit"));
}

// ---- through ffmpeg ------------------------------------------------------------------------------

#[test]
fn test_a_small_video_encodes_tags_and_reads_back_as_equirectangular() {
    use crate::encode::{Codec, Encoding, Ffmpeg, ffmpeg_available};
    let (ffmpeg, ffprobe) = (PathBuf::from("ffmpeg"), PathBuf::from("ffprobe"));
    if !ffmpeg_available(&ffmpeg) || !ffmpeg_available(&ffprobe) {
        eprintln!("note: ffmpeg or ffprobe is not on PATH, so the encoding test was skipped");
        return;
    }
    let scratch = Scratch::new("encode");
    let (partial, out) = (
        scratch.path().join("v.mp4.partial"),
        scratch.path().join("v.mp4"),
    );
    let size = size(128);
    let sky = noise_map(128, 64, 0.5);
    let mut encoder = Ffmpeg::start(&Encoding {
        ffmpeg,
        codec: Codec::Svt,
        crf: 28,
        preset: 12,
        width: size.width,
        height: size.height,
        frames_per_second: 30.0,
        out: partial.clone(),
    })
    .expect("ffmpeg starts");
    let mut scratch_bytes = Vec::new();
    for k in 0..3 {
        let field = still(32, 16, k as f64 * 0.1, 1.0);
        let frame = picture(Fields::One(&field), identity(), &sky, size, 1.0);
        encoder
            .write(&frame, &mut scratch_bytes)
            .expect("ffmpeg takes a frame");
    }
    encoder.finish().expect("ffmpeg finishes");
    crate::mp4::tag_file(&partial, &out).expect("the file is tagged");
    let probe = std::process::Command::new(ffprobe)
        .args([
            "-v",
            "error",
            "-count_frames",
            "-show_entries",
            "stream=codec_name,pix_fmt,width,height,nb_read_frames,color_transfer,color_primaries,color_space:stream_side_data_list",
        ])
        .arg(&out)
        .output()
        .expect("ffprobe runs");
    let text = String::from_utf8_lossy(&probe.stdout);
    for wanted in [
        "codec_name=av1",
        "pix_fmt=yuv420p10le",
        "width=128",
        "height=64",
        "nb_read_frames=3",
        "color_transfer=iec61966-2-1",
        "color_primaries=bt709",
        "color_space=bt709",
        "projection=equirectangular",
    ] {
        assert!(
            text.contains(wanted),
            "ffprobe does not report {wanted}:\n{text}"
        );
    }
}

#[test]
fn test_a_missing_ffmpeg_is_reported_in_one_sentence() {
    use crate::encode::{Codec, Encoding, Ffmpeg};
    let result = Ffmpeg::start(&Encoding {
        ffmpeg: PathBuf::from("no-such-ffmpeg-here"),
        codec: Codec::Svt,
        crf: 28,
        preset: 8,
        width: 128,
        height: 64,
        frames_per_second: 30.0,
        out: std::env::temp_dir().join("never-written.mp4"),
    });
    let message = result.err().expect("there is no such program");
    assert!(
        message.contains("was not found") && message.contains("--ffmpeg"),
        "{message}"
    );
}
