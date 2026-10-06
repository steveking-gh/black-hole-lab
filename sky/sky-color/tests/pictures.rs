//! Pictures for a reviewer: the Planckian locus, and what a shift does to a star's colour and
//! brightness under this model and under the renderer's old rule (the map's colour times g^4).
//!
//!     cargo test -p sky-color --release --test pictures -- --ignored
//!
//! writes PPM files into the directory named by `sky_color_PICTURES` (default: the system's
//! temporary directory) and converts each to PNG with ffmpeg, which must be on the PATH.
//!
//! Every picture is display-referred sRGB: linear BT.709 values brought into [0, 1] by
//! `into_display_gamut` (luminance kept, then hue), except the old rule's chart, which is clipped
//! channel by channel as the renderer clips. Then the sRGB transfer function, 8 bits.

use sky_color::colorimetry::luminance;
use sky_color::{into_display_gamut, model};
use std::path::{Path, PathBuf};

fn srgb(linear: f64) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let e = if c <= 0.003_130_8 {
        12.92 * c
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (e * 255.0 + 0.5) as u8
}

struct Image {
    w: usize,
    h: usize,
    px: Vec<[u8; 3]>,
}

impl Image {
    fn new(w: usize, h: usize, bg: [u8; 3]) -> Self {
        Self {
            w,
            h,
            px: vec![bg; w * h],
        }
    }

    fn rect(&mut self, x: usize, y: usize, w: usize, h: usize, c: [u8; 3]) {
        for j in y..(y + h).min(self.h) {
            for i in x..(x + w).min(self.w) {
                self.px[j * self.w + i] = c;
            }
        }
    }

    /// Text in a 3 x 5 pixel font, each font pixel `s` screen pixels, left edge at x.
    fn text(&mut self, x: usize, y: usize, s: usize, text: &str, c: [u8; 3]) {
        let mut cx = x;
        for ch in text.chars() {
            let rows = glyph(ch);
            for (r, bits) in rows.iter().enumerate() {
                for b in 0..3 {
                    if bits & (4 >> b) != 0 {
                        self.rect(cx + b * s, y + r * s, s, s, c);
                    }
                }
            }
            cx += 4 * s;
        }
    }

    fn save(&self, dir: &Path, name: &str) -> PathBuf {
        let ppm = dir.join(format!("{name}.ppm"));
        let mut bytes = format!("P6\n{} {}\n255\n", self.w, self.h).into_bytes();
        for p in &self.px {
            bytes.extend_from_slice(p);
        }
        std::fs::write(&ppm, bytes).unwrap();
        let png = dir.join(format!("{name}.png"));
        let status = std::process::Command::new("ffmpeg")
            .args(["-v", "error", "-y", "-i"])
            .arg(&ppm)
            .arg(&png)
            .status()
            .expect("ffmpeg on the PATH");
        assert!(status.success());
        std::fs::remove_file(&ppm).ok();
        png
    }
}

fn text_width(text: &str, s: usize) -> usize {
    text.chars().count() * 4 * s - s
}

/// Rows of three bits, top to bottom.
fn glyph(c: char) -> [u8; 5] {
    match c {
        '0' => [7, 5, 5, 5, 7],
        '1' => [2, 6, 2, 2, 7],
        '2' => [7, 1, 7, 4, 7],
        '3' => [7, 1, 7, 1, 7],
        '4' => [5, 5, 7, 1, 1],
        '5' => [7, 4, 7, 1, 7],
        '6' => [7, 4, 7, 5, 7],
        '7' => [7, 1, 1, 2, 2],
        '8' => [7, 5, 7, 5, 7],
        '9' => [7, 5, 7, 1, 7],
        '/' => [1, 1, 2, 4, 4],
        '.' => [0, 0, 0, 0, 2],
        'K' => [5, 5, 6, 5, 5],
        'g' => [0, 7, 5, 7, 1],
        'E' => [7, 4, 7, 4, 7],
        '-' => [0, 0, 7, 0, 0],
        '+' => [0, 2, 7, 2, 0],
        '=' => [0, 7, 0, 7, 0],
        _ => [0, 0, 0, 0, 0],
    }
}

fn out_dir() -> PathBuf {
    let dir = std::env::var_os("sky_color_PICTURES")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn to_pixel(rgb: [f64; 3]) -> [u8; 3] {
    into_display_gamut(rgb).map(srgb)
}

/// A pixel as the map draws a star of temperature t: the blackbody's BT.709 colour with its
/// largest channel 1.
fn star(t: f64) -> [f64; 3] {
    let rgb = model().blackbody_rgb(t);
    let m = rgb[0].max(rgb[1]).max(rgb[2]);
    rgb.map(|c| c / m)
}

const GREY: [u8; 3] = [40, 40, 40];

/// A brightness factor as two significant digits and a power of ten: 7.8E3.
fn factor(f: f64) -> String {
    let e = f.log10().floor();
    let mut m = f / 10f64.powf(e);
    let mut e = e as i32;
    if (m * 10.0).round() >= 100.0 {
        m /= 10.0;
        e += 1;
    }
    format!("{:.1}E{}", m, e)
}
const INK: [u8; 3] = [230, 230, 230];

#[test]
#[ignore]
fn picture_the_planckian_locus_at_equal_luminance() {
    // 1000 K to 40000 K on a logarithmic axis, every column at luminance 0.2 (in display units,
    // where 1 is white; the red primary alone reaches 0.2126); colours outside the gamut (below
    // 1905 K) desaturated toward grey at that luminance. Ticks and labels below the strip.
    let m = model();
    let (w, strip, margin) = (1200usize, 90usize, 40usize);
    let mut img = Image::new(w + 2 * margin, strip + 70, GREY);
    let (lo, hi) = (1000f64.ln(), 40_000f64.ln());
    for i in 0..w {
        let t = (lo + (hi - lo) * (i as f64 + 0.5) / w as f64).exp();
        let rgb = m.blackbody_rgb(t);
        let y = luminance(rgb);
        let c = to_pixel(rgb.map(|v| v * EQUAL / y));
        img.rect(margin + i, 10, 1, strip, c);
    }
    for t in [
        1000.0,
        1500.0,
        2000.0,
        3000.0,
        4000.0,
        5000.0,
        6500.0,
        8000.0,
        10_000.0,
        15_000.0,
        20_000.0,
        30_000.0,
        40_000.0f64,
    ] {
        let x = margin + ((t.ln() - lo) / (hi - lo) * w as f64).round() as usize;
        let x = x.min(margin + w - 1);
        img.rect(x, 10 + strip, 1, 10, INK);
        let label = format!("{}", t as u32);
        let tw = text_width(&label, 2);
        img.text(x.saturating_sub(tw / 2), 26 + strip, 2, &label, INK);
    }
    img.text(margin, 46 + strip, 2, "K", INK);
    let path = img.save(&out_dir(), "locus");
    println!("{}", path.display());
}

/// The chart of shifted stars: rows T = 3000, 5778, 10000 K, columns g = 1/8 ... 256 and 389.
/// `cell(t, g)` gives the linear display colour of a cell.
fn chart(name: &str, cell: impl Fn(f64, f64) -> ([u8; 3], Option<String>)) -> PathBuf {
    let gs: Vec<f64> = (-3..=8)
        .map(|k| 2f64.powi(k))
        .chain(std::iter::once(389.0))
        .collect();
    let labels: Vec<String> = gs
        .iter()
        .map(|&g| {
            if g < 1.0 {
                format!("1/{}", (1.0 / g) as u32)
            } else {
                format!("{}", g as u32)
            }
        })
        .collect();
    let ts = [3000.0, 5778.0, 10_000.0];
    let (cw, ch, left, top, gap) = (84usize, 84usize, 110usize, 40usize, 6usize);
    let mut img = Image::new(
        left + gs.len() * (cw + gap) + 10,
        top + ts.len() * (ch + gap) + 10,
        GREY,
    );
    img.text(10, 12, 3, "g=", INK);
    for (k, label) in labels.iter().enumerate() {
        let x = left + k * (cw + gap) + (cw - text_width(label, 3)) / 2;
        img.text(x, 12, 3, label, INK);
    }
    for (r, &t) in ts.iter().enumerate() {
        let y = top + r * (ch + gap);
        img.text(10, y + ch / 2 - 7, 3, &format!("{}K", t as u32), INK);
        for (k, &g) in gs.iter().enumerate() {
            let (c, label) = cell(t, g);
            let x = left + k * (cw + gap);
            img.rect(x, y, cw, ch, c);
            if let Some(label) = label {
                // Dark text on light cells, light on dark.
                let light = u32::from(c[0]) * 2 + u32::from(c[1]) * 7 + u32::from(c[2]) > 1200;
                let ink = if light { [20, 20, 20] } else { INK };
                img.text(
                    x + (cw - text_width(&label, 2)) / 2,
                    y + ch - 16,
                    2,
                    &label,
                    ink,
                );
            }
        }
    }
    img.save(&out_dir(), name)
}

/// The exposure of the brightness charts: a star at g = 1 shows at luminance 0.02 (display units),
/// so that a cell clips (luminance 1) at 50 times the g = 1 brightness, 5.6 stops up, and is
/// black to the eye (below one 8-bit code, luminance ~3e-4) at 1/65 of it.
const MID: f64 = 0.02;

/// The luminance of the equal-luminance charts and the locus strip.
const EQUAL: f64 = 0.2;

#[test]
#[ignore]
fn picture_the_shift_of_a_star_under_the_model() {
    let m = model();
    // Brightness: the model's shifted colour, at the model's visible brightness relative to the
    // g = 1 cell of the same row. Each cell is labelled with that factor, Y(gT) / Y(T).
    let p = chart("shift_brightness", |t, g| {
        let pixel = star(t);
        let k = MID / luminance(pixel);
        let out = m.shift(pixel, g);
        (
            to_pixel(out.map(|c| c * k)),
            Some(factor(luminance(out) / luminance(pixel))),
        )
    });
    println!("{}", p.display());
    // Hue: the same colours at equal luminance.
    let p = chart("shift_equal_luminance", |t, g| {
        let out = m.shift(star(t), g);
        let y = luminance(out);
        (
            if y > 0.0 {
                to_pixel(out.map(|c| c * EQUAL / y))
            } else {
                [0, 0, 0]
            },
            None,
        )
    });
    println!("{}", p.display());
}

#[test]
#[ignore]
fn picture_the_shift_of_a_star_under_the_old_rule() {
    // The map's colour times g^4, clipped channel by channel, as the renderer did.
    let p = chart("shift_old_rule_g4", |t, g| {
        let pixel = star(t);
        let k = MID / luminance(pixel) * g.powi(4);
        (pixel.map(|c| srgb(c * k)), Some(factor(g.powi(4))))
    });
    println!("{}", p.display());
}
