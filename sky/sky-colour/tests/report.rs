//! Prints the numbers the documentation quotes, for a reviewer to repeat:
//!
//!     cargo test -p sky-colour --release --test report -- --ignored --nocapture

use sky_colour::colorimetry::{delta_u_prime_v_prime, rgb_to_xyz, xy, xyz_to_rgb};
use sky_colour::model;
use sky_colour::planck::K_M;

#[test]
#[ignore]
fn report() {
    let m = model();
    println!("Visible-band factor Y(gT)/Y(T) against g^4");
    let gs = [0.125, 0.25, 0.5, 2.0, 10.0, 100.0, 389.0];
    for t in [3000.0, 5778.0, 10_000.0] {
        let row: Vec<String> = gs
            .iter()
            .map(|&g| format!("{:.4e}", m.visible_factor(t, g)))
            .collect();
        println!("  T {t:>6}: {}", row.join("  "));
    }
    let row: Vec<String> = gs
        .iter()
        .map(|&g: &f64| format!("{:.4e}", g.powi(4)))
        .collect();
    println!("  g^4     : {}", row.join("  "));

    println!("Luminance of blackbodies (cd/m^2) and Y relative to 5778 K");
    let sun = m.blackbody_xyz(5778.0)[1];
    for t in [
        300.0, 500.0, 700.0, 800.0, 1000.0, 1060.0, 1200.0, 1400.0, 1500.0, 2000.0, 5778.0,
    ] {
        let y = m.blackbody_xyz(t)[1];
        println!(
            "  {t:>6} K: {:.3e} cd/m^2, {:.3e} of 5778 K",
            K_M * y,
            y / sun
        );
    }

    println!("Implied temperatures");
    for (name, rgb) in [
        ("white", [1.0, 1.0, 1.0]),
        ("red", [1.0, 0.0, 0.0]),
        ("green", [0.0, 1.0, 0.0]),
        ("blue", [0.0, 0.0, 1.0]),
        ("yellow", [1.0, 1.0, 0.0]),
        ("cyan", [0.0, 1.0, 1.0]),
        ("magenta", [1.0, 0.0, 1.0]),
        ("brown", [0.4, 0.25, 0.12]),
        ("Betelgeuse wing", [1.0, 0.65, 0.359]),
        ("Betelgeuse core", [1.0, 0.944, 0.685]),
        ("Vega wing", [0.451, 0.619, 1.0]),
    ] {
        let i = m.implied_temperature(rgb).unwrap();
        println!(
            "  {name:>8} {rgb:?}: {:.1} K, Duv {:+.5}, in CCT domain {}",
            i.kelvin,
            i.duv,
            i.within_cct_domain()
        );
    }

    println!("Two-temperature mixture, equal luminance from 3500 K and 10000 K");
    let a = m.blackbody_xyz(3500.0);
    let b = m.blackbody_xyz(10_000.0);
    let mix: [f64; 3] = std::array::from_fn(|c| a[c] / a[1] + b[c] / b[1]);
    let i = m.implied_temperature_xyz(mix).unwrap();
    println!(
        "  implied {:.1} K, Duv {:+.5}, xy {:?}",
        i.kelvin,
        i.duv,
        xy(mix)
    );
    for g in [0.25, 0.5, 0.8, 1.25, 2.0, 3.0, 10.0, 100.0] {
        let model_xyz = rgb_to_xyz(m.shift(xyz_to_rgb(mix), g));
        let ga = m.blackbody_xyz(3500.0 * g);
        let gb = m.blackbody_xyz(10_000.0 * g);
        let truth: [f64; 3] = std::array::from_fn(|c| ga[c] / a[1] + gb[c] / b[1]);
        println!(
            "  g {g:>5}: du'v' {:.4}, Y model/true - 1 = {:+.3}, true xy ({:.4}, {:.4}), model xy ({:.4}, {:.4})",
            delta_u_prime_v_prime(model_xyz, truth),
            model_xyz[1] / truth[1] - 1.0,
            xy(truth).0,
            xy(truth).1,
            xy(model_xyz).0,
            xy(model_xyz).1
        );
    }

    println!("Timing");
    let start = std::time::Instant::now();
    let fresh = sky_colour::Model::new();
    println!(
        "  Model::new: {:.1} ms",
        start.elapsed().as_secs_f64() * 1e3
    );
    drop(fresh);
    let n = 2_000_000;
    let start = std::time::Instant::now();
    let mut acc = 0.0;
    for k in 0..n {
        let f = k as f64 / n as f64;
        let out = m.shift([0.3 + 0.7 * f, 0.5, 0.9 - 0.6 * f], 0.5 + 3.0 * f);
        acc += out[0];
    }
    let dt = start.elapsed().as_secs_f64();
    println!(
        "  shift: {:.1} ns per pixel ({acc:.3})",
        dt / n as f64 * 1e9
    );
    let start = std::time::Instant::now();
    for k in 0..n {
        let f = k as f64 / n as f64;
        acc += m
            .implied_temperature([0.3 + 0.7 * f, 0.5, 0.9 - 0.6 * f])
            .unwrap()
            .kelvin;
    }
    println!(
        "  implied_temperature: {:.1} ns ({acc:.3})",
        start.elapsed().as_secs_f64() / n as f64 * 1e9
    );
    let start = std::time::Instant::now();
    for k in 0..n {
        let f = k as f64 / n as f64;
        acc += m.xyz_ratio(3000.0 + 5000.0 * f, 0.5 + 3.0 * f)[1];
    }
    println!(
        "  xyz_ratio: {:.1} ns ({acc:.3})",
        start.elapsed().as_secs_f64() / n as f64 * 1e9
    );
}
