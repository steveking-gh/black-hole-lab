//! The colour of shifted light: the blackbody model wired into the renderer, checked on maps and
//! bundles of a few thousand pixels. Two tests at the end are `#[ignore]`d: a benchmark, and the
//! measurement of the filter order on the real star map and bundles, which needs them on disk.

// The loops over 0..3 are over the components of a colour, several arrays at once.
#![allow(clippy::needless_range_loop)]

use std::path::PathBuf;

use sky_color::colorimetry::{BT709_WHITE, delta_u_prime_v_prime, luminance, rgb_to_xyz, xy};
use sky_format::{Axes, FarSky, Frame, Grid, Num, fate};

use crate::colour::{
    Class, ColourRule, GLOW_RANGE, NOT_STARLIGHT, STAR_RANGE, VISIBLE_LUMINANCE, background_note,
    background_visible_g, lift_negatives, ln_blackbody,
};
use crate::field::{Judge, RayField};
use crate::parallel::for_each_band;
use crate::render::{Fields, Look, Picture, Scene, Seen, Shade, Size, render_as, shades};
use crate::sky::{Footprint, MapFrame, Mat3, SkyMap, orientation};
use crate::tally::{FilmTally, Tally};
use crate::tone::{Encoder, shade};

// ---- helpers -------------------------------------------------------------------------------------

const MAGENTA: [u16; 3] = [65_535, 0, 65_535];
const GREEN: [u16; 3] = [0, 65_535, 0];

/// A flat-space bundle frame: each ray's direction at infinity is the direction it is traced
/// along, turned by `turn` about z, and its shift is `g(i, j)`.
fn flat_frame(width: u32, height: u32, turn: f64, g: impl Fn(u32, u32) -> f32) -> Frame {
    let grid = Grid::new(width, height);
    let mut frame = Frame::new(width, height, 0);
    let (s, c) = turn.sin_cos();
    for j in 0..height {
        for i in 0..width {
            let k = grid.offset(i, j);
            let n = grid.pixel_direction(i, j);
            let d = [n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]];
            frame.fate[k] = fate::FAR_SKY;
            for c in 0..3 {
                frame.direction[c][k] = d[c] as f32;
            }
            frame.shift[k] = g(i, j);
        }
    }
    frame
}

fn field(width: u32, height: u32, g: f32) -> RayField {
    RayField::from_frame(&flat_frame(width, height, 0.0, |_, _| g))
}

fn identity() -> Mat3 {
    let n = |v: [f64; 3]| v.map(Num);
    let icrs = FarSky {
        name: "icrs".into(),
        axes_in_icrs: Axes {
            x: n([1.0, 0.0, 0.0]),
            y: n([0.0, 1.0, 0.0]),
            z: n([0.0, 0.0, 1.0]),
        },
    };
    orientation(&icrs, MapFrame::Celestial)
}

fn size(width: usize) -> Size {
    Size {
        width,
        height: width / 2,
    }
}

fn look(gain: f32) -> Look {
    Look {
        gain,
        unresolved: MAGENTA,
        undersampled: GREEN,
        encoder: Encoder::new(),
    }
}

/// Deterministic values in [0, 1).
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

fn noise_texels(width: usize, height: usize, scale: f32) -> Vec<[f32; 3]> {
    noise(3 * width * height, 7)
        .chunks(3)
        .map(|c| [c[0] * scale, c[1] * scale, c[2] * scale])
        .collect()
}

fn light_of(fields: Fields, sky: &SkyMap, size: Size, gain: f32) -> Vec<Shade> {
    let scene = Scene {
        fields,
        rotation: identity(),
        sky,
    };
    shades(&scene, size, gain, 3)
}

fn picture(
    fields: Fields,
    sky: &SkyMap,
    size: Size,
    gain: f32,
    what: Picture,
) -> (Vec<u16>, Tally) {
    let scene = Scene {
        fields,
        rotation: identity(),
        sky,
    };
    let mut out = vec![0; size.width * size.height * 3];
    let tally = render_as(&scene, size, &look(gain), what, 3, &mut out);
    (out, tally)
}

fn rgb(shade: Shade) -> [f32; 3] {
    match shade {
        Shade::Light(c) => c,
        other => panic!("expected light, found {other:?}"),
    }
}

/// The BT.709 RGB of a blackbody at `t`, scaled to luminance `y`: a texel of that colour.
fn blackbody_texel(t: f64, y: f64) -> [f64; 3] {
    let b = sky_color::model().blackbody_rgb(t);
    let scale = y / luminance(b);
    b.map(|c| c * scale)
}

// ---- the old rule, and g = 1 ---------------------------------------------------------------------

#[test]
fn test_colour_map_is_the_old_arithmetic_and_either_rule_at_g_one_is_the_old_picture_to_the_bit() {
    // A map with structure at every scale and values up to 0.9, drawn at a gain of 3 so that
    // many pixels are above white: the old renderer clips those channel by channel, and so must
    // both rules at g = 1.
    let (mw, mh) = (128, 64);
    let texels = noise_texels(mw, mh, 0.9);
    let map = SkyMap::new(mw, mh, texels.clone(), 4);
    let blackbody = SkyMap::with_colour(mw, mh, texels, 4, ColourRule::Blackbody);
    assert_eq!(map.colour(), ColourRule::Map);
    for (field_size, turn, out) in [(64, 0.0, 128), (64, 0.1, 64), (32, 0.37, 128)] {
        let frame = flat_frame(field_size, field_size / 2, turn, |_, _| 1.0);
        let f = RayField::from_frame(&frame);
        let (a, _) = picture(Fields::One(&f), &map, size(out), 3.0, Picture::Light);
        let (b, _) = picture(Fields::One(&f), &blackbody, size(out), 3.0, Picture::Light);
        assert!(
            a == b,
            "{out} wide, turned {turn}: the rules differ at g = 1"
        );
        assert!(a.contains(&65_535), "nothing was above white");
    }
    // Under `map`, at any g, each pixel is the old arithmetic: the filtered map through
    // `tone::shade`, g^4 and the gain, as the tests of `tests` pin down.
    for g in [0.3f32, 1.0, 1.7, 389.0] {
        let f = field(64, 32, g);
        let scene = Scene {
            fields: Fields::One(&f),
            rotation: identity(),
            sky: &map,
        };
        let got = shades(&scene, size(128), 3.0, 3);
        let mut k = 0;
        scene.see_rows(size(128), 0..64, |_, _, seen| {
            if let Seen::Sky { footprint, g } = seen {
                assert_eq!(got[k], Shade::Light(shade(map.sample(footprint), g, 3.0)));
            }
            k += 1;
        });
    }
}

// ---- the model through the pipeline --------------------------------------------------------------

#[test]
fn test_a_map_of_blackbody_colours_comes_out_as_the_blackbody_at_g_t_at_the_librarys_brightness() {
    // A uniform map of one blackbody's colour, drawn at a quarter of its resolution (so that the
    // filter reads the rip-map's level (2, 2)) from a bundle of half that: every pixel is the
    // blackbody at g T, scaled as the texel was, at the library's visible factor Y(gT) / Y(T), and
    // brought into gamut. The tolerance, 2e-5 of the pixel's largest channel: the texel is stored
    // in f32 (6e-8), which moves the temperature its colour implies by up to about 1e-6 of itself
    // at 30000 K where the locus moves slowest, and a shifted texel by the difference of the
    // slopes d ln B / d ln T at g T and T times that (below 5 here); the stored ln T and tint add
    // below 5e-6 (`crate::sky`, `shift_forms`), and the f32 output 6e-8.
    let model = sky_color::model();
    for t in [2500.0, 4000.0, 5778.0, 10_000.0, 30_000.0] {
        let texel = blackbody_texel(t, 0.1).map(|c| c as f32);
        let texel64 = texel.map(f64::from);
        let sky = SkyMap::with_colour(256, 128, vec![texel; 256 * 128], 4, ColourRule::Blackbody);
        let y0 = luminance(texel64);
        for g in [0.5, 0.8, 1.25, 2.0, 10.0, 389.0] {
            let gain = 0.5f32;
            let f = field(32, 16, g as f32);
            let g = f64::from(g as f32);
            let shades = light_of(Fields::One(&f), &sky, size(64), gain);
            // The library's answer for this texel, exposed and brought into gamut.
            let want = lift_negatives(model.shift(texel64, g).map(|c| c * f64::from(gain)));
            let visible = model.visible_factor(t, g);
            for s in &shades {
                let got = rgb(*s).map(f64::from);
                let size = want.iter().fold(0.0f64, |m, c| m.max(c.abs()));
                for c in 0..3 {
                    assert!(
                        (got[c] - want[c]).abs() <= 2e-5 * size,
                        "T {t} g {g}: {got:?} against {want:?}"
                    );
                }
                // Its brightness is the library's visible factor times the texel's.
                let y = luminance(got) / (y0 * f64::from(gain));
                assert!(
                    (y / visible - 1.0).abs() < 2e-5,
                    "T {t} g {g}: {y} against {visible}"
                );
            }
            // And the library's shift of a blackbody is the blackbody at g T, so this is it.
            let direct = model.blackbody_rgb(g * t);
            let scale = y0 / luminance(model.blackbody_rgb(t));
            let expected = lift_negatives(direct.map(|c| c * scale * f64::from(gain)));
            let got = rgb(shades[0]).map(f64::from);
            let size = expected.iter().fold(0.0f64, |m, c| m.max(c.abs()));
            for c in 0..3 {
                assert!(
                    (got[c] - expected[c]).abs() <= 2e-5 * size,
                    "T {t} g {g}: {got:?} is not the blackbody at g T, {expected:?}"
                );
            }
        }
    }
}

#[test]
fn test_the_shifted_picture_is_continuous_in_g_through_one() {
    // Away from g = 1 the model is used; at g = 1 exactly the old arithmetic. The change from
    // g = 1 to 1 + e is of the size of e times d ln Q / d ln g = d ln B / d ln T at the texel's
    // temperature, at most about 60 (a 300 K colour, the reddest the search allows) and below 10
    // for most: no jump. The floor, where e is too small to move anything, is rounding: the old
    // path is f32 and the model's f64 with the tint stored in f32 (below 2e-6), so 1e-5.
    let (mw, mh) = (128, 64);
    let sky = SkyMap::with_colour(mw, mh, noise_texels(mw, mh, 0.8), 4, ColourRule::Blackbody);
    let at = |g: f32| light_of(Fields::One(&field(64, 32, g)), &sky, size(128), 1.0);
    let base = at(1.0);
    let change = |g: f32| {
        let mut worst = 0.0f64;
        for (a, b) in base.iter().zip(&at(g)) {
            let (a, b) = (rgb(*a).map(f64::from), rgb(*b).map(f64::from));
            let size = a.iter().fold(1e-3f64, |m, c| m.max(c.abs()));
            for c in 0..3 {
                worst = worst.max((a[c] - b[c]).abs() / size);
            }
        }
        worst
    };
    for e in [1e-2f32, -1e-2, 1e-3, -1e-3, 1e-4, -1e-4, 1e-6, -1e-6] {
        let e64 = f64::from(e);
        let d = change(1.0 + e);
        assert!(d <= 60.0 * e64.abs() + 1e-5, "e {e}: {d}");
    }
    // And it moves in proportion to e, as a continuous function does: a tenth of the step, a
    // tenth of the change (to 10 %, the curvature over the step).
    let (big, small) = (change(1.0 + 1e-2), change(1.0 + 1e-3));
    assert!(big > 1e-3, "{big}");
    assert!((small / big - 0.1).abs() < 0.01, "{big} {small}");
}

// ---- what is not known ---------------------------------------------------------------------------

#[test]
fn test_no_nan_reaches_the_encoder_for_black_saturated_or_far_off_texels_and_every_awkward_shift() {
    // Texels: black, white, the primaries and their mixtures (magenta and cyan are far off the
    // locus, blue beyond its hot end), one tiny, one of the least positive normal f32. Shifts,
    // one a column of rays: 0, negative, infinite, NaN, the least positive f32, small, ordinary,
    // enormous and the largest f32. A ray with a NaN direction too.
    let palette: [[f32; 3]; 10] = [
        [0.0; 3],
        [1.0; 3],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 0.0],
        [1e-30, 0.0, 1e-30],
        [f32::MIN_POSITIVE; 3],
    ];
    let (mw, mh) = (64, 32);
    let texels: Vec<[f32; 3]> = (0..mw * mh)
        .map(|k| palette[(k * 7 + k / mw) % 10])
        .collect();
    let shifts: [f32; 16] = [
        0.0,
        -1.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NAN,
        1e-45,
        1e-3,
        0.03,
        0.5,
        1.0,
        2.0,
        389.0,
        1e12,
        1e30,
        f32::MAX,
        1.2,
    ];
    let mut frame = flat_frame(32, 16, 0.0, |i, _| shifts[i as usize % 16]);
    let k = frame.grid().offset(3, 3);
    frame.direction[0][k] = f32::NAN;
    let f = RayField::from_frame(&frame);
    for rule in [ColourRule::Map, ColourRule::Blackbody] {
        let sky = SkyMap::with_colour(mw, mh, texels.clone(), 4, rule);
        let lights = light_of(Fields::One(&f), &sky, size(64), 5.0);
        // Every texel here is light, so no pixel's light is not known: a NaN would have come from
        // the arithmetic.
        for s in &lights {
            if let Shade::Light(c) = s {
                assert!(
                    c.iter().all(|x| !x.is_nan() && *x >= 0.0),
                    "{rule:?}: {c:?}"
                );
            }
        }
        // Rays with a shift that is not one are unresolved, drawn and counted as such.
        let (out, tally) = picture(Fields::One(&f), &sky, size(64), 5.0, Picture::Light);
        let unresolved = lights.iter().filter(|s| **s == Shade::Unresolved).count() as u64;
        assert!(unresolved >= 4 * 2 * 16, "{rule:?}: {unresolved}");
        assert_eq!(tally.unresolved, unresolved, "{rule:?}");
        let e = Encoder::new();
        for (p, s) in out.chunks(3).zip(&lights) {
            match s {
                Shade::Light(c) => assert_eq!(p, &c.map(|x| e.encode(x))[..]),
                Shade::Unresolved => assert_eq!(p, &MAGENTA[..]),
                other => panic!("{other:?}"),
            }
        }
        // The largest shifts are white on a lit texel and black on a black one, never unknown.
        let white = lights
            .iter()
            .filter(|s| matches!(s, Shade::Light(c) if c.iter().all(|x| *x >= 1.0)))
            .count();
        assert!(white > 0, "{rule:?}");
    }
    // A texel that is not the colour of any light is not known under the model, and is drawn
    // as unresolved, not as black: the library returns NaN for it, and the renderer catches it.
    let mut odd = texels.clone();
    odd.iter_mut().for_each(|t| *t = [-0.5, 0.2, 0.1]);
    let sky = SkyMap::with_colour(mw, mh, odd, 4, ColourRule::Blackbody);
    let (out, tally) = picture(
        Fields::One(&field(32, 16, 2.0)),
        &sky,
        size(64),
        1.0,
        Picture::Light,
    );
    assert_eq!(tally.unresolved, 64 * 32);
    assert!(out.chunks(3).all(|p| p == MAGENTA));
    // And the model's own entry refuses what `crate::field` never lets through.
    let sky = SkyMap::with_colour(mw, mh, texels, 4, ColourRule::Blackbody);
    let foot = sky.footprint((10.0, 10.0), Some((11.0, 10.0)), Some((10.0, 11.0)));
    for g in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(
            crate::colour::light(&sky, foot, g, 1.0)
                .iter()
                .all(|c| c.is_nan())
        );
    }
}

// ---- the display's gamut -------------------------------------------------------------------------

#[test]
fn test_a_shifted_colour_with_a_negative_channel_is_brought_in_with_its_luminance_kept() {
    // Moved toward the grey of its luminance until no channel is negative: the luminance is kept,
    // and the chromaticity stays on the line from the white point through the colour.
    for c in [
        [0.6, 0.3, -0.1],
        [-0.2, 0.5, 0.4],
        [3.0, 0.8, -0.4],
        [0.02, -0.001, 0.01],
    ] {
        let out = lift_negatives(c);
        assert!(out.iter().all(|&x| x >= 0.0), "{c:?} -> {out:?}");
        assert!((luminance(out) / luminance(c) - 1.0).abs() < 1e-12, "{c:?}");
        let (a, b) = xy(rgb_to_xyz(c));
        let (p, q) = xy(rgb_to_xyz(out));
        let (wx, wy) = BT709_WHITE;
        assert!(
            ((a - wx) * (q - wy) - (b - wy) * (p - wx)).abs() < 1e-12,
            "{c:?}"
        );
        // Exactly as far as needed: the lowest channel is zero.
        assert!(out.iter().fold(f64::MAX, |m, &x| m.min(x)) < 1e-15, "{c:?}");
    }
    // A colour with no negative channel is left alone, above white too: the encoder clips that.
    for c in [
        [0.2, 0.5, 0.9],
        [3.0, 2.0, 0.5],
        [0.0; 3],
        [f64::INFINITY; 3],
    ] {
        assert_eq!(lift_negatives(c), c);
    }
    assert!(lift_negatives([f64::NAN, 0.0, 0.0])[0].is_nan());
    // Through the pipeline: a 2500 K star at g = 0.5 is a 1250 K blackbody, whose blue is negative
    // (below 1905 K the locus leaves BT.709). The pixel has none, and the luminance the library
    // gives.
    let texel = blackbody_texel(2500.0, 0.2).map(|c| c as f32);
    let sky = SkyMap::with_colour(64, 32, vec![texel; 64 * 32], 4, ColourRule::Blackbody);
    let raw = sky_color::model().shift(texel.map(f64::from), 0.5);
    assert!(raw[2] < 0.0, "{raw:?}");
    let gain = 1e4f32;
    for s in light_of(Fields::One(&field(64, 32, 0.5)), &sky, size(64), gain) {
        let c = rgb(s).map(f64::from);
        assert!(c.iter().all(|&x| x >= 0.0), "{c:?}");
        let want = luminance(raw) * f64::from(gain);
        assert!((luminance(c) / want - 1.0).abs() < 1e-5, "{c:?}");
    }
}

// ---- where the model holds -----------------------------------------------------------------------

#[test]
fn test_the_shift_counts_agree_with_a_direct_count_and_the_false_colour_picture() {
    // Shifts from 0.1 to 20, log-spaced across the rays, with a dark block and an unresolved ray.
    let (gw, gh) = (64u32, 32u32);
    let mut frame = flat_frame(gw, gh, 0.0, |i, _| {
        (0.1f64 * 200f64.powf(f64::from(i) / f64::from(gw - 1))) as f32
    });
    for j in 10..14 {
        for i in 20..30 {
            let k = frame.grid().offset(i, j);
            frame.fate[k] = fate::DARK;
        }
    }
    let k = frame.grid().offset(50, 5);
    frame.fate[k] = fate::UNRESOLVED;
    let f = RayField::from_frame(&frame);
    let sky = SkyMap::with_colour(
        128,
        64,
        noise_texels(128, 64, 0.5),
        4,
        ColourRule::Blackbody,
    );
    let size = size(128);
    let (_, tally) = picture(Fields::One(&f), &sky, size, 1.0, Picture::Light);
    // The direct count: every pixel drawn as light, by the class of its g.
    let scene = Scene {
        fields: Fields::One(&f),
        rotation: identity(),
        sky: &sky,
    };
    let mut classes = [0u64; 7];
    let (mut least, mut largest) = (f64::INFINITY, f64::NEG_INFINITY);
    scene.shade_rows(size, 1.0, 0..size.height, |_, _, s, g| {
        if let (Shade::Light(c), Some(g)) = (s, g)
            && !c.iter().any(|x| x.is_nan())
        {
            classes[Class::of(g).index()] += 1;
            least = least.min(g);
            largest = largest.max(g);
        }
    });
    assert_eq!(tally.shift.classes, classes);
    assert_eq!((tally.shift.least, tally.shift.largest), (least, largest));
    assert_eq!(
        tally.shift.sky(),
        tally.pixels - tally.dark - tally.unresolved - tally.undersampled
    );
    assert!(
        classes.iter().all(|&n| n > 0),
        "every class occurs: {classes:?}"
    );
    let in_star: u64 = classes[Class::Both.index()] + classes[Class::StarOnly.index()];
    let in_glow: u64 = classes[Class::GlowOnly.index()] + classes[Class::Both.index()];
    assert_eq!(tally.shift.in_star_range(), in_star);
    assert_eq!(tally.shift.in_glow_range(), in_glow);
    assert_eq!(
        tally.shift.beyond(),
        (
            classes[Class::FarRed.index()],
            classes[Class::FarBlue.index()]
        )
    );
    // The false-colour picture draws each class in its colour, with the same counts, and the
    // dark region and the unresolved ray as ever.
    let (out, same) = picture(Fields::One(&f), &sky, size, 1.0, Picture::ModelRange);
    assert_eq!(same, tally);
    for class in Class::ALL {
        let n = out.chunks(3).filter(|p| *p == class.code()).count() as u64;
        assert_eq!(n, classes[class.index()], "{class:?}");
    }
    assert_eq!(
        out.chunks(3).filter(|p| *p == [0; 3]).count() as u64,
        tally.dark
    );
    assert_eq!(
        out.chunks(3).filter(|p| *p == MAGENTA).count() as u64,
        tally.unresolved
    );
    // Over a film: the worst frame for the model is the one with the most sky outside the range
    // for single stars, and the report says so.
    let (_, calm) = picture(
        Fields::One(&field(gw, gh, 1.0)),
        &sky,
        size,
        1.0,
        Picture::Light,
    );
    assert_eq!(calm.shift.in_star_range(), calm.shift.sky());
    let mut film = FilmTally::default();
    film.add(0, calm);
    film.add(1, tally);
    film.add(2, calm);
    assert_eq!(film.worst_shift, Some((1, tally.shift)));
    let text = film.shift_report().join("\n");
    assert!(text.contains("video frame 1"), "{text}");
    assert!(
        text.contains("within 0.9..2 (good for single stars)"),
        "{text}"
    );
    assert!(text.contains(&format!("beyond a shift of {NOT_STARLIGHT} either way")));
}

#[test]
fn test_the_classes_tile_the_shifts_at_the_stated_ends_and_their_colours_are_not_the_markers() {
    assert_eq!(Class::of(0.0), Class::FarRed);
    assert_eq!(Class::of(0.199), Class::FarRed);
    assert_eq!(Class::of(0.2), Class::Red);
    assert_eq!(Class::of(0.8), Class::GlowOnly);
    assert_eq!(Class::of(0.9), Class::Both);
    assert_eq!(Class::of(1.0), Class::Both);
    assert_eq!(Class::of(1.25), Class::Both);
    assert_eq!(Class::of(1.2501), Class::StarOnly);
    assert_eq!(Class::of(2.0), Class::StarOnly);
    assert_eq!(Class::of(2.0001), Class::Blue);
    assert_eq!(Class::of(5.0), Class::Blue);
    assert_eq!(Class::of(5.0001), Class::FarBlue);
    let codes: Vec<[u16; 3]> = Class::ALL.iter().map(|c| c.code()).collect();
    for (k, a) in codes.iter().enumerate() {
        assert!(![[65_535, 0, 0], [0; 3], [65_535; 3]].contains(a), "{a:?}");
        assert!(!codes[k + 1..].contains(a), "{a:?} twice");
    }
    for (k, class) in Class::ALL.iter().enumerate() {
        assert_eq!(class.index(), k);
    }
}

// ---- the order of filtering and shifting ---------------------------------------------------------

#[test]
fn test_shifting_each_texel_read_is_exact_at_level_0_and_errs_as_the_model_does_for_a_coarse_texels_mixture()
 {
    // A checkerboard of 3500 K and 10000 K texels of equal luminance: every 2 x 2 block, and so
    // every texel of the rip-map's level (1, 1), is exactly the library's documented mixture
    // (`test_the_two_temperature_mixture_errs_as_documented` in sky-color). The truth is the
    // mean of the two shifted texels.
    let model = sky_color::model();
    let (a, b) = (blackbody_texel(3500.0, 0.1), blackbody_texel(10_000.0, 0.1));
    let (mw, mh) = (256usize, 128usize);
    let texels: Vec<[f32; 3]> = (0..mw * mh)
        .map(|k| if (k % mw + k / mw) % 2 == 0 { a } else { b }.map(|c| c as f32))
        .collect();
    let sky = SkyMap::with_colour(mw, mh, texels, 4, ColourRule::Blackbody);
    let (a32, b32) = (
        a.map(|c| f64::from(c as f32)),
        b.map(|c| f64::from(c as f32)),
    );
    let error = |got: [f64; 3], g: f64| {
        let (sa, sb) = (model.shift(a32, g), model.shift(b32, g));
        let truth = rgb_to_xyz(std::array::from_fn(|c| 0.5 * (sa[c] + sb[c])));
        let got = rgb_to_xyz(got);
        (got[1] / truth[1] - 1.0, delta_u_prime_v_prime(got, truth))
    };
    let footprint_of = |f: &RayField, out: Size| {
        let scene = Scene {
            fields: Fields::One(f),
            rotation: identity(),
            sky: &sky,
        };
        let mut all = Vec::new();
        scene.see_rows(out, 0..out.height, |_, _, s| {
            if let Seen::Sky { footprint, .. } = s {
                all.push(footprint);
            }
        });
        all
    };
    // The library's documented errors of the mixture, (g, du'v', dY / Y).
    let documented = [
        (0.5, 0.0676, -0.740),
        (0.8, 0.0272, -0.084),
        (1.25, 0.0187, -0.140),
        (2.0, 0.0269, -0.473),
        (10.0, 0.0118, -0.746),
    ];
    // Level 0, the pixel centres half a texel from the texels' (the far sky turned by half a
    // texel): each pixel reads two texels of different temperature. Shifting each is exact;
    // blending first is the mixture's error in full.
    let half = RayField::from_frame(&flat_frame(
        256,
        128,
        0.5 * std::f64::consts::TAU / 256.0,
        |_, _| 1.0,
    ));
    let feet = footprint_of(&half, size(256));
    let middle = feet[64 * 256 + 100];
    // One texel's footprint, read from level 0 between two texels (and, by the f32 rounding of
    // the bundle's directions, a trace of the rows above or below).
    assert!(middle.across.max(middle.down) <= 1.001 && sky.texels_read(middle) >= 2);
    for (g, duv, dy) in documented {
        for f in [middle, feet[3 * 256 + 17], feet[120 * 256 + 255]] {
            let (y, c) = error(sky.sample_shifted(f, g), g);
            assert!(y.abs() < 1e-5 && c < 1e-6, "g {g}: level 0 off by {y} {c}");
        }
        let (y, c) = error(model.shift(sky.sample(middle).map(f64::from), g), g);
        assert!(
            (y - dy).abs() < 5e-3 && (c - duv).abs() < 5e-4,
            "g {g}: {y} {c}"
        );
    }
    // Level (1, 1): a picture half the map's size. Each texel read is the mixture, and shifting
    // it as one is the model's own error for a mixture, no more.
    let coarse = field(128, 64, 1.0);
    let feet = footprint_of(&coarse, size(128));
    for (g, duv, dy) in documented {
        for f in [feet[32 * 128 + 40], feet[5 * 128 + 127]] {
            // Two texels each way, to the f32 rounding of the directions; the filter snaps a span
            // within a thousandth of a level to the level.
            assert!(
                (f.across - 2.0).abs() < 1e-3 && (f.down - 2.0).abs() < 1e-3,
                "{f:?}"
            );
            let (y, c) = error(sky.sample_shifted(f, g), g);
            assert!(
                (y - dy).abs() < 5e-3 && (c - duv).abs() < 5e-4,
                "g {g}: {y} {c}"
            );
        }
    }
}

// ---- the renderer's table ------------------------------------------------------------------------

#[test]
fn test_the_renderers_blackbody_table_is_the_librarys_ratio() {
    // Both are cubic Hermite interpolations of the direct sum at 128 nodes per e-fold (the
    // library's spacing differs slightly, 1474 steps over its range); each is within 3e-9 of the
    // sum above 300 K and 3e-8 at 10 K, so the ratio, a difference of two lookups each way, within
    // about 1e-7.
    let model = sky_color::model();
    let table = ln_blackbody();
    let mut t = 300.0f64;
    while t < 1e6 {
        for g in [0.04, 0.5, 0.9, 1.1, 2.0, 10.0, 389.0, 1e4] {
            if g * t < 10.0 || g * t > 1e10 {
                continue;
            }
            let (from, to) = (table.at(t.ln()), table.at((g * t).ln()));
            let q = model.xyz_ratio(t, g);
            for c in 0..3 {
                let ours = to[c] - from[c];
                if q[c] < 1e-300 {
                    // The library's exponential has underflowed; the logarithm has not.
                    assert!(ours < -690.0, "T {t} g {g} c {c}: {ours}");
                    continue;
                }
                assert!(
                    (ours - q[c].ln()).abs() < 1e-7,
                    "T {t} g {g} c {c}: {ours} {}",
                    q[c]
                );
            }
        }
        t *= 1.13;
    }
    let (lo, hi) = table.domain();
    assert!((lo.exp() - 10.0).abs() < 1e-9 && hi.exp() >= 1e10);
}

// ---- the command line and the run's words ---------------------------------------------------------

#[test]
fn test_the_colour_rule_defaults_to_blackbody_and_the_model_range_view_is_off() {
    let parse = |extra: &[&str]| {
        let mut a: Vec<String> = ["--bundle", "b", "--sky", "s.exr", "--encoder", "none"]
            .map(String::from)
            .to_vec();
        a.extend(extra.iter().map(|s| s.to_string()));
        crate::cli::parse(&a)
    };
    let options = |extra: &[&str]| match parse(extra).expect("parses") {
        crate::cli::Request::Render(o) => o,
        crate::cli::Request::Help => panic!("not help"),
    };
    let o = options(&[]);
    assert_eq!(
        (o.colour, o.show_model_range),
        (ColourRule::Blackbody, false)
    );
    let o = options(&["--colour", "map", "--show-model-range"]);
    assert_eq!((o.colour, o.show_model_range), (ColourRule::Map, true));
    assert!(
        parse(&["--colour", "sepia"])
            .unwrap_err()
            .contains("blackbody or map")
    );
    assert!(crate::cli::USAGE.contains("--colour blackbody|map"));
}

#[test]
fn test_the_microwave_background_is_reported_past_the_shift_at_which_an_eye_would_see_it() {
    // K_M Y(2.7255 K x g) reaches 0.005 cd/m^2 at g = 288.7: between the library's 0.015 cd/m^2
    // at g = 300 and its 2e-8 at 200.
    let g = background_visible_g();
    let seen = sky_color::planck::K_M * sky_color::model().blackbody_xyz(2.7255 * g)[1];
    assert!((seen / VISIBLE_LUMINANCE - 1.0).abs() < 1e-9, "{seen}");
    assert!((g - 288.7).abs() < 0.05, "{g}");
    assert!(background_note(g * 0.999).is_none());
    // At g = 389, the library's 1060 K glow of 10.2 cd/m^2.
    let note = background_note(389.0).expect("past the threshold");
    assert!(
        note.contains("1060 K") && note.contains("10.2 cd/m^2"),
        "{note}"
    );
}

// ---- measurements (ignored) ----------------------------------------------------------------------

#[test]
#[ignore]
fn test_bench_the_per_tap_costs() {
    use std::hint::black_box;
    use std::time::Instant;
    let m = sky_color::model();
    let n = 2_000_000;
    let ts: Vec<f64> = (0..n).map(|k| 2000.0 + (k % 9000) as f64).collect();
    let clock = Instant::now();
    let mut s = 0.0;
    for &t in &ts {
        s += m.xyz_ratio(black_box(t), black_box(1.7))[1];
    }
    println!(
        "xyz_ratio: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let clock = Instant::now();
    let mut s = 0.0;
    for &t in &ts {
        s += m.blackbody_xyz(black_box(t))[1];
    }
    println!(
        "blackbody_xyz: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let clock = Instant::now();
    let mut s = 0.0;
    for &t in &ts {
        s += black_box(t).ln();
    }
    println!(
        "ln: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let clock = Instant::now();
    let mut s = 0.0;
    for &t in &ts {
        s += (black_box(t) * 1e-4).exp();
    }
    println!(
        "exp: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let clock = Instant::now();
    let mut s = 0.0;
    for &t in &ts {
        s += m.shift(black_box([t * 1e-4, 0.5, 0.3]), black_box(1.7))[1];
    }
    println!(
        "shift: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let table = crate::colour::ln_blackbody();
    let ws: Vec<f64> = ts.iter().map(|t| t.ln()).collect();
    let clock = Instant::now();
    let mut s = 0.0;
    for &w in &ws {
        s += table.at(black_box(w))[1];
    }
    println!(
        "table.at: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
    let clock = Instant::now();
    let mut s = 0.0;
    for &w in &ws {
        let (a, b) = (table.at(black_box(w)), table.at(black_box(w) + 0.53));
        let q: [f64; 3] = std::array::from_fn(|c| (b[c] - a[c]).exp());
        s += q[0] + q[1] + q[2];
    }
    println!(
        "ratio by table: {:.1} ns ({s})",
        clock.elapsed().as_nanos() as f64 / n as f64
    );
}

// ---- the filter order on real data (ignored: needs the star map and the bundles) ----------------

/// Footprint classes by the larger of its two spans in level-0 texels.
const SPAN_EDGES: [f64; 7] = [1.001, 2.0, 4.0, 8.0, 16.0, 64.0, f64::INFINITY];

fn span_class(f: &Footprint) -> usize {
    let span = f.across.max(f.down);
    SPAN_EDGES.iter().position(|&e| span <= e).unwrap()
}

fn span_label(k: usize) -> String {
    let lo = if k == 0 { 0.0 } else { SPAN_EDGES[k - 1] };
    match k {
        0 => "<= 1 (level 0)".into(),
        _ if SPAN_EDGES[k].is_infinite() => format!("> {lo}"),
        _ => format!("{lo}..{}", SPAN_EDGES[k]),
    }
}

/// Errors of one method against the truth, summed over pixels.
#[derive(Default, Clone)]
struct Errors {
    n: u64,
    truth_y: f64,
    method_y: f64,
    abs_dy: f64,
    duv_weighted: f64,
    /// Per visible pixel: |dY / Y| and du'v'.
    rel: Vec<f32>,
    duv: Vec<f32>,
}

impl Errors {
    fn add(&mut self, truth: [f64; 3], method: [f64; 3], visible: f64) {
        let (t, m) = (rgb_to_xyz(truth), rgb_to_xyz(method));
        if t[1].is_nan() || t[1] <= 0.0 || !m.iter().all(|c| c.is_finite()) {
            return;
        }
        self.n += 1;
        self.truth_y += t[1];
        self.method_y += m[1];
        self.abs_dy += (m[1] - t[1]).abs();
        let d = delta_u_prime_v_prime(m, t);
        self.duv_weighted += d * t[1];
        if t[1] >= visible {
            self.rel.push(((m[1] - t[1]) / t[1]).abs() as f32);
            self.duv.push(d as f32);
        }
    }

    fn merge(&mut self, o: &Errors) {
        self.n += o.n;
        self.truth_y += o.truth_y;
        self.method_y += o.method_y;
        self.abs_dy += o.abs_dy;
        self.duv_weighted += o.duv_weighted;
        self.rel.extend(&o.rel);
        self.duv.extend(&o.duv);
    }

    fn p95(v: &[f32]) -> f64 {
        if v.is_empty() {
            return f64::NAN;
        }
        let mut v = v.to_vec();
        v.sort_by(|a, b| a.total_cmp(b));
        f64::from(v[(v.len() - 1) * 95 / 100])
    }

    fn line(&self) -> String {
        if self.n == 0 {
            return "-".into();
        }
        format!(
            "bias {:+.2} %, mean |dY| {:.2} %, p95 |dY/Y| {:.2} %, mean du'v' {:.4}, p95 du'v' {:.4}",
            100.0 * (self.method_y / self.truth_y - 1.0),
            100.0 * self.abs_dy / self.truth_y,
            100.0 * Self::p95(&self.rel),
            self.duv_weighted / self.truth_y,
            Self::p95(&self.duv)
        )
    }
}

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from)
}

/// The footprints and shifts of a sample of the sky pixels of bundle frame `index` drawn at
/// `size`, every `stride`-th pixel.
fn sampled_pixels(
    bundle: &std::path::Path,
    index: u32,
    sky: &SkyMap,
    size: Size,
    stride: usize,
) -> Vec<(Footprint, f64)> {
    let reader = sky_format::BundleReader::open(bundle).expect("a bundle");
    let manifest = reader.manifest();
    let judge = Judge {
        pixels_per_ray: (
            size.width as f64 / manifest.grid.width as f64,
            size.height as f64 / manifest.grid.height as f64,
        ),
        texel: (2.0 * std::f64::consts::PI / sky.width() as f64)
            .min(std::f64::consts::PI / sky.height() as f64),
    };
    let field = crate::load::read_field(&reader, index, Some(judge), 16).expect("a frame");
    let scene = Scene {
        fields: Fields::One(&field),
        rotation: orientation(&manifest.far_sky, MapFrame::Galactic),
        sky,
    };
    let mut out = Vec::new();
    let mut k = 0usize;
    scene.see_rows(size, 0..size.height, |_, _, seen| {
        if let Seen::Sky { footprint, g } = seen {
            if k.is_multiple_of(stride) {
                out.push((footprint, g));
            }
            k += 1;
        }
    });
    out
}

/// Measures, on the real star map and real bundles, how far each order of filtering and
/// shifting is from the true one, by footprint size and g. Run with
///
///     SKY_MAP=<map.exr> SKY_BUNDLES=<dir holding bob_near_fall, hover_r6_a09>
///     cargo test --release -p sky-render -- --ignored --nocapture measure_the_filter_order
///
/// The truth: the map shifted texel by texel at one g, then built into its own rip-map. Reading
/// a rip-map is linear in its texels, so that map read over a footprint is exactly the mean of the
/// shifted level-0 texels with the filter's own weights, the true order.
#[test]
#[ignore]
fn test_measure_the_filter_order_on_the_real_map() {
    let (Some(map), Some(bundles)) = (env_path("SKY_MAP"), env_path("SKY_BUNDLES")) else {
        eprintln!("SKY_MAP and SKY_BUNDLES are not set: nothing measured");
        return;
    };
    let image = crate::load::read_map(&map).expect("the map");
    let texels = image.texels.clone();
    let sky = SkyMap::with_colour(
        image.width,
        image.height,
        image.texels,
        16,
        ColourRule::Blackbody,
    );
    let size = Size {
        width: 8192,
        height: 4096,
    };
    let cases = [
        ("bob_near_fall", 0u32),
        ("bob_near_fall", 237),
        ("bob_near_fall", 473),
        ("hover_r6_a09", 0),
    ];
    let mut pixels = Vec::new();
    let mut spans = [0u64; SPAN_EDGES.len()];
    for (name, index) in cases {
        let p = sampled_pixels(&bundles.join(name), index, &sky, size, 61);
        for (f, _) in &p {
            spans[span_class(f)] += 1;
        }
        println!("{name} frame {index}: {} sky pixels sampled", p.len());
        pixels.extend(p);
    }
    let all: u64 = spans.iter().sum();
    let read: usize = pixels.iter().map(|(f, _)| sky.texels_read(*f)).sum();
    println!(
        "texels read a sky pixel, on average: {:.2}",
        read as f64 / pixels.len() as f64
    );
    println!("\nfootprints of the sampled sky pixels, by the larger span in level-0 texels:");
    for (k, n) in spans.iter().enumerate() {
        println!(
            "  {:>16}: {:6.3} %",
            span_label(k),
            100.0 * *n as f64 / all as f64
        );
    }
    let model = sky_color::model();
    // Visible: at the default exposure (2.5 stops) at least 1e-3 of white, about 13 codes of a
    // 10-bit sRGB-encoded video.
    let gain = 2f64.powf(2.5);
    let visible = 1e-3 / gain;
    let gs = [
        0.5, 0.67, 0.8, 0.9, 1.1, 1.25, 1.5, 2.0, 3.0, 5.0, 10.0, 30.0, 100.0, 389.0,
    ];
    for g in gs {
        let mut shifted = texels.clone();
        for_each_band(&mut shifted, 8192, 16, 16, |_, band| {
            for t in band.iter_mut() {
                *t = model.shift_f32(*t, g);
            }
        });
        let truth_map = SkyMap::new(image.width, image.height, shifted, 16);
        let classes = SPAN_EDGES.len();
        let results: std::sync::Mutex<Vec<(Errors, Errors)>> =
            std::sync::Mutex::new(vec![(Errors::default(), Errors::default()); classes]);
        let mut work = pixels.clone();
        for_each_band(&mut work, 4096, 1, 16, |_, band| {
            let mut local = vec![(Errors::default(), Errors::default()); classes];
            for &(f, _) in band.iter() {
                let truth = truth_map.sample(f).map(f64::from);
                let cheap = model.shift(sky.sample(f).map(f64::from), g);
                let per_tap = sky.sample_shifted(f, g);
                let k = span_class(&f);
                local[k].0.add(truth, cheap, visible);
                local[k].1.add(truth, per_tap, visible);
            }
            let mut r = results.lock().unwrap();
            for (a, b) in r.iter_mut().zip(&local) {
                a.0.merge(&b.0);
                a.1.merge(&b.1);
            }
        });
        println!("\ng = {g}");
        let r = results.into_inner().unwrap();
        for (k, (cheap, per_tap)) in r.iter().enumerate() {
            if cheap.n == 0 {
                continue;
            }
            println!("  span {:>16} ({} px)", span_label(k), cheap.n);
            println!("    blend, then shift:    {}", cheap.line());
            println!("    shift each tap, blend: {}", per_tap.line());
            // The bounds `crate::colour` states ("Filtering and shifting").
            let mean = |e: &Errors| e.abs_dy / e.truth_y;
            assert!(mean(per_tap) <= mean(cheap) + 1e-6, "g {g} span {k}");
            if k == 0 {
                assert!(mean(per_tap) < 1e-5, "g {g}: level 0 is exact");
            }
            if (GLOW_RANGE.0..=STAR_RANGE.1).contains(&g) {
                assert!(mean(per_tap) < 0.10, "g {g} span {k}");
                assert!(Errors::p95(&per_tap.duv) < 0.01, "g {g} span {k}");
                if k <= 2 {
                    assert!(mean(per_tap) < 0.03, "g {g} span {k}");
                }
            }
        }
    }
}
