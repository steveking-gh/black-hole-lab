//! Measurements for the reviewer, not checks: cost per step and per ray, the cost of a whole
//! 1024 x 512 frame, and two fate maps written as images. All ignored by default; run with
//!
//!     cargo test -p kerr-sky --release --test measure -- --ignored --nocapture --test-threads=1
//!
//! (one at a time, so the frame timing has the machine to itself). The fate maps are written to
//! the directory in the environment variable KERR_SKY_MAPS, or to the system temporary directory,
//! as PPM, and converted to PNG with ffmpeg where it is on PATH.

use std::f64::consts::PI;
use std::time::Instant;

use kerr_sky::frame::{pixel_direction, trace_frame};
use kerr_sky::ray::{Fate, TraceOptions, trace_direction};
use kerr_sky::{Kerr, Observer, Triad};

#[test]
#[ignore = "a timing; run with --ignored --nocapture --test-threads=1"]
fn measure_the_cost_of_a_step_a_ray_and_a_frame() {
    let options = TraceOptions::default();
    let kerr = Kerr::new(1.0, 0.9);
    for &r in &[6.0, 3.0] {
        let obs = Observer::stationary(&kerr, r, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let (w, h) = (256, 128);
        // One thread, every pixel, twice: the first warms the caches.
        let mut steps = 0u64;
        let mut integrated = 0u64;
        let mut elapsed = 0.0;
        for pass in 0..2 {
            let start = Instant::now();
            let (mut s, mut n) = (0u64, 0u64);
            for j in 0..h {
                for i in 0..w {
                    let out = trace_direction(&kerr, &triad, pixel_direction(i, j, w, h), &options);
                    s += u64::from(out.steps);
                    n += u64::from(out.steps > 0);
                }
            }
            if pass == 1 {
                elapsed = start.elapsed().as_secs_f64();
                steps = s;
                integrated = n;
            }
        }
        let rays = (w * h) as f64;
        println!(
            "static observer at r = {r} M, a = 0.9 M, facing the hole, {w} x {h} rays on one thread: \
             {:.1} steps a ray on average ({:.1} over the {} rays integrated, the rest decided at the \
             observer), {:.1} us a ray, {:.0} ns an accepted step",
            steps as f64 / rays,
            steps as f64 / integrated as f64,
            integrated,
            elapsed / rays * 1e6,
            elapsed / steps as f64 * 1e9
        );
    }
    let threads = std::thread::available_parallelism().map_or(16, |n| n.get());
    for &r in &[6.0, 3.0] {
        let obs = Observer::stationary(&kerr, r, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let start = Instant::now();
        let frame = trace_frame(&kerr, &triad, 1024, 512, &options, threads, true);
        let elapsed = start.elapsed().as_secs_f64();
        let counts = [0u8, 1, 2].map(|f| frame.fate.iter().filter(|&&x| x == f).count());
        let unresolved: Vec<(usize, usize)> = (0..512 * 1024)
            .filter(|&k| frame.fate[k] == 0)
            .map(|k| (k % 1024, k / 1024))
            .collect();
        println!(
            "1024 x 512 frame at r = {r} M on {threads} threads with the symmetry: {elapsed:.2} s, \
             {} rays traced, {} steps; fates (unresolved, sky, shadow) = {counts:?}; unresolved at \
             {unresolved:?}",
            frame.rays_traced, frame.steps
        );
    }
}

fn write_fate_map(name: &str, frame: &kerr_sky::SkyFrame) {
    let dir = std::env::var("KERR_SKY_MAPS")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir());
    std::fs::create_dir_all(&dir).unwrap();
    let ppm = dir.join(format!("{name}.ppm"));
    let mut bytes = format!("P6\n{} {}\n255\n", frame.width, frame.height).into_bytes();
    for k in 0..frame.width * frame.height {
        // Far sky: a colour from d, so that the lensing shows (hue from the azimuth of d, lighter
        // toward +Z); shadow black; unresolved red; the winding darkens each turn.
        let rgb = match frame.fate[k] {
            1 => {
                let d = [0, 1, 2].map(|c| f64::from(frame.direction[c][k]));
                let az = d[1].atan2(d[0]);
                let turns = f64::from(frame.winding[k]).abs();
                let fade = 1.0 / (1.0 + turns);
                let lift = 0.35 + 0.3 * d[2];
                let channel = |shift: f64| {
                    let v = lift + 0.35 * (az + shift).cos();
                    (255.0 * (v * fade).clamp(0.0, 1.0)) as u8
                };
                [
                    channel(0.0),
                    channel(2.0 * PI / 3.0),
                    channel(4.0 * PI / 3.0),
                ]
            }
            2 => [0, 0, 0],
            _ => [255, 0, 0],
        };
        bytes.extend_from_slice(&rgb);
    }
    std::fs::write(&ppm, bytes).unwrap();
    let png = dir.join(format!("{name}.png"));
    let converted = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-i"])
        .arg(&ppm)
        .arg(&png)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    println!(
        "wrote {}{}",
        ppm.display(),
        if converted {
            format!(" and {}", png.display())
        } else {
            String::new()
        }
    );
}

#[test]
#[ignore = "writes two images; run with --ignored --nocapture"]
fn measure_fate_maps_of_a_static_observer_at_six_m() {
    // A static observer at 6 M, triad facing the hole (heading pi), 512 x 256. Without spin the
    // shadow must be a disc of the Synge radius, sin(alpha) = 3 sqrt(3) (M / r) sqrt(1 - 2M / r),
    // 45 degrees at 6 M, centred on the frame's centre; with a = 0.9 M it must be displaced and
    // flattened on the prograde side. The shadow's extent along the equator is printed for both.
    let threads = std::thread::available_parallelism().map_or(16, |n| n.get());
    for &a in &[0.0, 0.9] {
        let kerr = Kerr::new(1.0, a);
        let obs = Observer::stationary(&kerr, 6.0, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let (w, h) = (512, 256);
        let frame = trace_frame(&kerr, &triad, w, h, &TraceOptions::default(), threads, true);
        // The shadow's edges along the row nearest the equator and down the centre column, in
        // degrees of the frame's longitude and latitude.
        let row = h / 2;
        let shadow_cols: Vec<usize> = (0..w)
            .filter(|&i| frame.fate[row * w + i] == Fate::Dark as u8)
            .collect();
        let col = w / 2;
        let shadow_rows: Vec<usize> = (0..h)
            .filter(|&j| frame.fate[j * w + col] == Fate::Dark as u8)
            .collect();
        let lon = |i: usize| ((i as f64 + 0.5) / w as f64 - 0.5) * 360.0;
        let lat = |j: usize| (0.5 - (j as f64 + 0.5) / h as f64) * 180.0;
        let synge = (3.0 * 3f64.sqrt() / 6.0 * (1.0 - 2.0 / 6.0f64).sqrt())
            .asin()
            .to_degrees();
        println!(
            "a = {a}: shadow along the equator from longitude {:.2} to {:.2} degrees, down the centre \
             column from latitude {:.2} to {:.2}; Synge radius at a = 0: {synge:.2} degrees; unresolved {}",
            lon(shadow_cols[0]),
            lon(*shadow_cols.last().unwrap()),
            lat(*shadow_rows.last().unwrap()),
            lat(shadow_rows[0]),
            frame.unresolved
        );
        // Which way is prograde on the frame: the triad's y leg is -e2 at heading pi (the
        // observer's left is retrograde), so prograde is to the right (-y, increasing longitude).
        let _ = pixel_direction;
        write_fate_map(
            &format!("fate_map_r6_a{}", if a == 0.0 { "0" } else { "0.9" }),
            &frame,
        );
    }
}
