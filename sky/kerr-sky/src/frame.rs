//! One traced frame: the observer's whole sky on the specification's equirectangular grid.
//!
//! Pixel (i, j) of a W x H grid looks along
//!
//!     lambda_i = ((i + 0.5) / W - 0.5) 2 pi,     beta_j = (0.5 - (j + 0.5) / H) pi,
//!     phi = -lambda_i,     n = (cos beta_j cos phi, cos beta_j sin phi, sin beta_j)
//!
//! in the triad (specification 4.3), and every plane is stored row by row, top row first, pixel
//! (i, j) at k = j W + i. The planes are those of the frame file (specification 7.3): the fate as a
//! byte, d as three whole planes of f32 (every d_X, then every d_Y, then every d_Z), g as f32 and
//! the winding as i16 clamped to [-32767, 32767]; d and g are NaN and the winding 0 wherever the
//! fate is not 1. They are this crate's own vectors: turning them into a frame file is the caller's.
//!
//! Rows go to `std::thread::scope` threads from a shared counter, so a thread that draws the rows
//! through the shadow - which cost nothing, being decided at the observer - goes back for more.
//!
//! When the triad's z leg is the spin axis (`Triad::is_reflection_symmetric`), row H - 1 - j is the
//! mirror image of row j: the same fate, g and winding, and d with d_Z negated. Only the upper half
//! is traced. The mirror is exact rather than approximate: negating z and p_z negates exactly the
//! terms of the flow that are odd in them and leaves every other operation bit for bit the same,
//! and a test traces both halves and compares.

use std::sync::atomic::{AtomicUsize, Ordering};

use crate::metric::Kerr;
use crate::observer::Triad;
use crate::ray::{Fate, Outcome, TraceOptions, trace_direction};

/// The planes of one traced frame, in the specification's storage order.
#[derive(Debug, Clone)]
pub struct SkyFrame {
    /// W.
    pub width: usize,
    /// H.
    pub height: usize,
    /// Fate per pixel (specification 4.7).
    pub fate: Vec<u8>,
    /// d_X, d_Y and d_Z planes (specification 4.5); NaN unless fate 1.
    pub direction: [Vec<f32>; 3],
    /// g per pixel (specification 4.6); NaN unless fate 1.
    pub shift: Vec<f32>,
    /// Winding per pixel (specification 4.8); 0 unless fate 1.
    pub winding: Vec<i16>,
    /// How many pixels are unresolved (fate 0).
    pub unresolved: usize,
    /// Rays actually integrated or decided (half the grid, or a little over, with the symmetry).
    pub rays_traced: usize,
    /// Accepted integration steps over the rays traced.
    pub steps: u64,
}

/// The direction n in the triad that pixel (i, j) of a W x H grid looks along.
///
/// The two angles are computed as (2i + 1 - W) pi / W and (H - 1 - 2j) pi / (2H), which are the
/// specification's formulae with the halves cleared. Written so, the numerator of beta is an
/// integer that changes sign exactly between row j and row H - 1 - j, and the mirrored rows get
/// exactly negated latitudes; the specification's own form rounds differently for the two.
pub fn pixel_direction(i: usize, j: usize, width: usize, height: usize) -> [f64; 3] {
    let lambda = (2.0 * i as f64 + 1.0 - width as f64) * std::f64::consts::PI / width as f64;
    let beta =
        (height as f64 - 1.0 - 2.0 * j as f64) * std::f64::consts::PI / (2.0 * height as f64);
    let phi = -lambda;
    let (sb, cb) = beta.sin_cos();
    let (sp, cp) = phi.sin_cos();
    [cb * cp, cb * sp, sb]
}

/// Trace a W x H frame for one observer on `threads` threads (at least one). With `symmetry` and a
/// reflection-symmetric triad only the upper half of the rows is traced and mirrored.
pub fn trace_frame(
    kerr: &Kerr,
    triad: &Triad,
    width: usize,
    height: usize,
    options: &TraceOptions,
    threads: usize,
    symmetry: bool,
) -> SkyFrame {
    trace_frame_reporting(
        kerr,
        triad,
        width,
        height,
        options,
        threads,
        symmetry,
        &|_, _| {},
    )
}

/// [`trace_frame`], telling `rows_done` after every row a thread finishes how many rows of the
/// frame are finished and how many there are to trace in all (half the grid, or a little over, with
/// the mirror).
///
/// The count is a second shared counter beside the one that hands the rows out: that one counts
/// rows *started*, and a row started is up to a row's work away from being done. `rows_done` is
/// called on the tracing thread, once a row - a few hundred to a few thousand times a frame, each
/// after a row of rays that took milliseconds - so a caller that prints from it costs the tracing
/// nothing it could measure, provided it does its own throttling and takes no lock for long.
#[allow(clippy::too_many_arguments)]
pub fn trace_frame_reporting(
    kerr: &Kerr,
    triad: &Triad,
    width: usize,
    height: usize,
    options: &TraceOptions,
    threads: usize,
    symmetry: bool,
    rows_done: &(dyn Fn(usize, usize) + Sync),
) -> SkyFrame {
    let mirror = symmetry && triad.is_reflection_symmetric();
    let rows = if mirror { height.div_ceil(2) } else { height };
    let next = AtomicUsize::new(0);
    let finished = AtomicUsize::new(0);
    let mut traced: Vec<(usize, Vec<Outcome>)> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..threads.max(1))
            .map(|_| {
                scope.spawn(|| {
                    let mut mine = Vec::new();
                    loop {
                        let j = next.fetch_add(1, Ordering::Relaxed);
                        if j >= rows {
                            break;
                        }
                        let row = (0..width)
                            .map(|i| {
                                trace_direction(
                                    kerr,
                                    triad,
                                    pixel_direction(i, j, width, height),
                                    options,
                                )
                            })
                            .collect();
                        mine.push((j, row));
                        rows_done(finished.fetch_add(1, Ordering::Relaxed) + 1, rows);
                    }
                    mine
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().expect("a tracing thread panicked"))
            .collect()
    });
    traced.sort_by_key(|(j, _)| *j);

    let pixels = width * height;
    let mut frame = SkyFrame {
        width,
        height,
        fate: vec![0; pixels],
        direction: [
            vec![f32::NAN; pixels],
            vec![f32::NAN; pixels],
            vec![f32::NAN; pixels],
        ],
        shift: vec![f32::NAN; pixels],
        winding: vec![0; pixels],
        unresolved: 0,
        rays_traced: 0,
        steps: 0,
    };
    for (j, row) in &traced {
        for (i, out) in row.iter().enumerate() {
            frame.rays_traced += 1;
            frame.steps += u64::from(out.steps);
            store(&mut frame, i + *j * width, out, 1.0);
            let mirrored = height - 1 - *j;
            if mirror && mirrored != *j {
                store(&mut frame, i + mirrored * width, out, -1.0);
            }
        }
    }
    frame.unresolved = frame
        .fate
        .iter()
        .filter(|&&f| f == Fate::Unresolved as u8)
        .count();
    frame
}

fn store(frame: &mut SkyFrame, k: usize, out: &Outcome, z_sign: f64) {
    frame.fate[k] = out.fate as u8;
    if out.fate == Fate::FarSky {
        frame.direction[0][k] = out.direction[0] as f32;
        frame.direction[1][k] = out.direction[1] as f32;
        frame.direction[2][k] = (z_sign * out.direction[2]) as f32;
        frame.shift[k] = out.shift as f32;
        frame.winding[k] = out.winding.clamp(-32767, 32767) as i16;
    }
}

#[cfg(test)]
// Tensor components are indexed by their indices, as the formulae write them.
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;
    use crate::observer::Observer;

    #[test]
    fn test_pixel_directions_follow_the_specification_grid() {
        // The centre of the frame looks along +x, moving right turns toward -y, the top row is
        // nearest +z, and both edges look behind.
        let (w, h) = (8, 4);
        let centre = pixel_direction(w / 2, h / 2, w, h);
        assert!(centre[0] > 0.8 && centre[1] < 0.0 && centre[2] < 0.0);
        let right = pixel_direction(w - 1, h / 2, w, h);
        let left = pixel_direction(0, h / 2, w, h);
        // The last column is just short of lambda = +pi: behind, and a little to the right (-y).
        assert!(right[0] < -0.8 && right[1] < 0.0 && left[0] < -0.8 && left[1] > 0.0);
        assert!(pixel_direction(3, 0, w, h)[2] > 0.9);
        for j in 0..h {
            for i in 0..w {
                let n = pixel_direction(i, j, w, h);
                assert!(((n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) - 1.0).abs() < 1e-15);
                let m = pixel_direction(i, h - 1 - j, w, h);
                assert!(
                    n[0] == m[0] && n[1] == m[1] && n[2] == -m[2],
                    "rows mirror exactly"
                );
            }
        }
    }

    #[test]
    fn test_tracing_both_halves_gives_the_mirrored_planes_bit_for_bit() {
        // A small grid, traced whole and traced by halves, for observers who see a shadow, the
        // photon ring and the far sky: a static observer, an infaller between the horizons, and a
        // prograde orbiter. Every plane must agree exactly - fate, winding, g, and d with d_Z
        // negated - because the mirror image of a ray is computed by the same operations on
        // negated numbers.
        let kerr = Kerr::new(1.0, 0.9);
        let eq = kerr.equatorial();
        let cases = [
            Observer::stationary(&kerr, 6.0, 0.4).unwrap(),
            Observer::new(
                &kerr,
                1.0,
                1.0,
                kerr_equatorial::GeodesicState::new_infall(&eq, 0.0, 1.0, 1.0, 2.0).u,
            )
            .unwrap(),
            {
                let ut = eq.circular_orbit_dilation(4.0, true).unwrap();
                let om = eq.orbital_angular_velocity(4.0, true).unwrap();
                Observer::new(&kerr, 4.0, -2.0, [ut, 0.0, ut * om]).unwrap()
            },
        ];
        let options = TraceOptions::default();
        for obs in cases {
            let triad = Triad::new(&kerr, &obs, std::f64::consts::PI);
            let (w, h) = (24, 13);
            let whole = trace_frame(&kerr, &triad, w, h, &options, 4, false);
            let half = trace_frame(&kerr, &triad, w, h, &options, 4, true);
            assert_eq!(whole.rays_traced, w * h);
            assert_eq!(half.rays_traced, w * h.div_ceil(2));
            assert_eq!(whole.fate, half.fate);
            assert_eq!(whole.winding, half.winding);
            let bits = |v: &Vec<f32>| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&whole.shift), bits(&half.shift));
            for c in 0..3 {
                assert_eq!(
                    bits(&whole.direction[c]),
                    bits(&half.direction[c]),
                    "d component {c}"
                );
            }
            let counts = [0u8, 1, 2].map(|f| whole.fate.iter().filter(|&&x| x == f).count());
            println!(
                "r = {}: fates (unresolved, sky, shadow) = {counts:?}",
                obs.r
            );
            assert_eq!(counts[0], 0, "no ray unresolved");
            assert!(counts[1] > 0);
        }
    }
    #[test]
    fn test_the_row_count_reaches_every_row_to_trace_once_and_in_order_of_finishing() {
        // Each finished row is counted exactly once, whichever thread finished it, so the counts
        // the callback sees are 1 to the number of rows traced, each once, and the last is all of
        // them: with the mirror, half the grid rounded up.
        let kerr = Kerr::new(1.0, 0.9);
        let obs = Observer::stationary(&kerr, 6.0, 0.4).unwrap();
        let triad = Triad::new(&kerr, &obs, std::f64::consts::PI);
        let (w, h) = (16usize, 9usize);
        for (symmetry, rows) in [(false, h), (true, h.div_ceil(2))] {
            let seen = std::sync::Mutex::new(Vec::new());
            trace_frame_reporting(
                &kerr,
                &triad,
                w,
                h,
                &TraceOptions::default(),
                3,
                symmetry,
                &|done, of| seen.lock().unwrap().push((done, of)),
            );
            let mut seen = seen.into_inner().unwrap();
            seen.sort_unstable();
            let expected: Vec<(usize, usize)> = (1..=rows).map(|k| (k, rows)).collect();
            assert_eq!(seen, expected, "symmetry {symmetry}");
        }
    }
}
