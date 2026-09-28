//! Is f64 enough? The observer who freezes onto the right branch of the inner horizon.
//!
//! An observer with E - Omega_- L < 0 - E = 1, L = 2.2 M at a = 0.9 M, released from 4.5 M, the
//! worldline `kerr_equatorial`'s own tests walk - never crosses r-: in this chart u^t grows like
//! exp(kappa_- t) and the app stalls the worldline at u^t = 1e10. The observer's legs then have
//! components of size u^t, and the direct momentum u - n^i e_i is, in some directions, a
//! difference of such numbers that should come out near 1 / u^t: an expected loss of (u^t)^2 1e-16
//! of its relative precision. `Triad::look` builds p in the raindrop's frame instead (see
//! `observer`). This file measures both along that worldline.
//!
//! `measure_the_view_of_an_observer_freezing_onto_the_inner_horizon` is the measurement; it takes
//! a minute and is ignored by default. Run it with
//!
//!     cargo test -p kerr-sky --release --test precision -- --ignored --nocapture
//!
//! `test_the_conditioned_momentum_is_right_where_the_direct_one_has_failed` is its fast half and
//! runs with the suite.

use std::f64::consts::{PI, TAU};

use kerr_equatorial::GeodesicState;
use kerr_sky::frame::{pixel_direction, trace_frame};
use kerr_sky::ray::{Fate, TraceOptions, trace_covector};
use kerr_sky::{Kerr, Observer, Triad};

const ARCSEC: f64 = PI / 180.0 / 3600.0;

/// Events along the freezing worldline at which u^t first passes each power of ten.
fn freezing_events(kerr: &Kerr, decades: &[i32]) -> Vec<(f64, f64, [f64; 3])> {
    let eq = kerr.equatorial();
    let mut geo = GeodesicState::new_infall(&eq, 0.0, 4.5, 1.0, 2.2);
    let mut out = Vec::new();
    let mut next = 0;
    while next < decades.len() && !geo.stalled {
        geo.step_coord_time(&eq, 0.1);
        if geo.u[0] >= 10f64.powi(decades[next]) {
            out.push((geo.r, geo.phi, geo.u));
            next += 1;
        }
    }
    out
}

fn angle_between(d: [f64; 3], e: [f64; 3]) -> f64 {
    let cross = [
        d[1] * e[2] - d[2] * e[1],
        d[2] * e[0] - d[0] * e[2],
        d[0] * e[1] - d[1] * e[0],
    ];
    let s = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    s.atan2(d[0] * e[0] + d[1] * e[1] + d[2] * e[2])
}

/// Directions spread over the whole sky, and a set closing in on the direction of motion, where
/// the sky crowds and the direct construction cancels worst: at 1e-1 to 1e-12 rad from it, and at
/// multiples of 1 / gamma, around eight position angles.
fn directions(triad: &Triad) -> Vec<[f64; 3]> {
    let mut out: Vec<[f64; 3]> = Vec::new();
    for j in 0..16 {
        for i in 0..32 {
            out.push(pixel_direction(i, j, 32, 16));
        }
    }
    let f = triad.forward();
    let across = [-f[1], f[0], 0.0];
    let (gamma, _) = triad.boost();
    let mut radii: Vec<f64> = (1..=12).map(|k| 10f64.powi(-k)).collect();
    radii.extend([0.3, 1.0, 3.0, 10.0].map(|k| k / gamma));
    for rho in radii {
        for m in 0..8 {
            let chi = TAU * m as f64 / 8.0;
            let (s, c) = rho.sin_cos();
            let (sc, cc) = chi.sin_cos();
            out.push([
                c * f[0] + s * cc * across[0],
                c * f[1] + s * cc * across[1],
                s * sc,
            ]);
        }
    }
    out
}

/// Area of the spherical triangle abc (Van Oosterom and Strackee 1983).
fn triangle_area(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let cross = [
        b[1] * c[2] - b[2] * c[1],
        b[2] * c[0] - b[0] * c[2],
        b[0] * c[1] - b[1] * c[0],
    ];
    let triple = dot(a, cross);
    2.0 * triple.abs().atan2(1.0 + dot(a, b) + dot(b, c) + dot(c, a))
}

/// The angular radius, on the observer's sky, of the cone about the direction of motion whose
/// rays show half of the celestial sphere: rings of rays log-spaced in angle from the direction
/// of motion, the far-sky solid angle of each ring summed from the quadrilaterals whose four
/// corners reached the far sky, until 2 pi is reached.
fn half_sky_radius(kerr: &Kerr, triad: &Triad) -> Option<f64> {
    let f = triad.forward();
    let across = [-f[1], f[0], 0.0];
    let (gamma, _) = triad.boost();
    let rings = 240;
    let start = (1e-3 / gamma).max(1e-15);
    let radii: Vec<f64> = (0..rings)
        .map(|k| start * (PI / start).powf(k as f64 / (rings - 1) as f64))
        .collect();
    let spokes = 48;
    let options = TraceOptions::default();
    let traced: Vec<Vec<Option<[f64; 3]>>> = std::thread::scope(|scope| {
        let handles: Vec<_> = radii
            .chunks(rings / 16)
            .map(|chunk| {
                scope.spawn(move || {
                    chunk
                        .iter()
                        .map(|&rho| {
                            (0..spokes)
                                .map(|m| {
                                    let chi = TAU * m as f64 / spokes as f64;
                                    let (s, c) = rho.sin_cos();
                                    let (sc, cc) = chi.sin_cos();
                                    let n = [
                                        c * f[0] + s * cc * across[0],
                                        c * f[1] + s * cc * across[1],
                                        s * sc,
                                    ];
                                    let out =
                                        kerr_sky::ray::trace_direction(kerr, triad, n, &options);
                                    (out.fate == Fate::FarSky).then_some(out.direction)
                                })
                                .collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap())
            .collect()
    });
    let mut area = 0.0;
    for k in 0..rings - 1 {
        for m in 0..spokes {
            let m2 = (m + 1) % spokes;
            if let (Some(a), Some(b), Some(c), Some(d)) = (
                traced[k][m],
                traced[k][m2],
                traced[k + 1][m2],
                traced[k + 1][m],
            ) {
                area += triangle_area(a, b, c) + triangle_area(a, c, d);
            }
        }
        if area >= TAU {
            return Some(radii[k + 1]);
        }
    }
    None
}

#[test]
#[ignore = "a minute of tracing; run with --ignored --nocapture"]
fn measure_the_view_of_an_observer_freezing_onto_the_inner_horizon() {
    let kerr = Kerr::new(1.0, 0.9);
    let eq = kerr.equatorial();
    let events = freezing_events(&kerr, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    let pixel_8k = TAU / 8192.0;
    println!(
        "{:>8} {:>10} {:>9} {:>9} {:>9} {:>11} {:>11} {:>9} {:>9} {:>9} {:>11} {:>9}",
        "u^t",
        "r - r-",
        "|uu+1|",
        "triad",
        "gamma",
        "direct\"",
        "direct g",
        "cond\"",
        "d apart\"",
        "sky frac",
        "half-sky\"",
        "px(8K)"
    );
    for (r, phi, u) in events {
        let obs = Observer::new(&kerr, r, phi, u).expect("in scope: just above r-");
        let triad = Triad::new(&kerr, &obs, PI);
        let norm_defect = (eq.norm(r, &u) + 1.0).abs();
        let legs = [triad.u, triad.x, triad.y, triad.z];
        let mut ortho = 0.0f64;
        for i in 0..4 {
            for j in 0..4 {
                let want = if i != j {
                    0.0
                } else if i == 0 {
                    -1.0
                } else {
                    1.0
                };
                ortho = ortho.max((kerr.dot(triad.position, legs[i], legs[j]) - want).abs());
            }
        }
        let (gamma, _) = triad.boost();
        let options = TraceOptions::default();
        let (mut direct_angle, mut direct_g, mut cond_angle, mut d_apart) =
            (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for n in directions(&triad) {
            let pa = triad.look_direct(n);
            let pb = triad.look(n);
            // Where each momentum really points on the observer's sky, and at what frequency,
            // by the conditioned inverse.
            let (na, nu_a) = triad.sky_direction(&kerr, pa);
            let (nb, _) = triad.sky_direction(&kerr, pb);
            direct_angle = direct_angle.max(angle_between(na, n));
            cond_angle = cond_angle.max(angle_between(nb, n));
            // g as the direct construction would have it, (-p . u) / E with both from the legs,
            // against the conditioned 1 / E.
            let (qa, qb) = (
                kerr.lower(triad.position, pa),
                kerr.lower(triad.position, pb),
            );
            let ga = -(0..4).map(|mu| qa[mu] * triad.u[mu]).sum::<f64>() / -qa[0];
            let gb = 1.0 / -qb[0];
            let _ = nu_a;
            if ga.is_finite() && gb.is_finite() {
                direct_g = direct_g.max((ga / gb - 1.0).abs());
            }
            // And the far-sky directions the two momenta lead to.
            let (oa, ob) = (
                trace_covector(&kerr, triad.position, qa, &options),
                trace_covector(&kerr, triad.position, qb, &options),
            );
            if oa.fate == Fate::FarSky && ob.fate == Fate::FarSky {
                d_apart = d_apart.max(angle_between(oa.direction, ob.direction));
            }
        }
        // The fraction of the sky's solid angle that reaches the far sky, on a 256 x 128 grid.
        let (w, h) = (256, 128);
        let frame = trace_frame(&kerr, &triad, w, h, &options, 16, true);
        let mut sky = 0.0;
        for j in 0..h {
            let beta = (0.5 - (j as f64 + 0.5) / h as f64) * PI;
            for i in 0..w {
                if frame.fate[j * w + i] == 1 {
                    sky += beta.cos() * (TAU / w as f64) * (PI / h as f64);
                }
            }
        }
        let half = half_sky_radius(&kerr, &triad);
        let diameter = half.map(|rho| 2.0 * rho);
        println!(
            "{:>8.1e} {:>10.2e} {:>9.1e} {:>9.1e} {:>9.2e} {:>11.2e} {:>11.1e} {:>9.1e} {:>9.1e} {:>9.2e} {:>11.3e} {:>9.2e}",
            u[0],
            r - kerr.inner_horizon(),
            norm_defect,
            ortho,
            gamma,
            direct_angle / ARCSEC,
            direct_g,
            cond_angle / ARCSEC,
            d_apart / ARCSEC,
            sky / (4.0 * PI),
            diameter.map_or(f64::NAN, |d| d / ARCSEC),
            diameter.map_or(f64::NAN, |d| d / pixel_8k),
        );
    }
    println!(
        "columns: |g(u,u) + 1|; the triad's worst |g(e_a, e_b) - eta_ab|; gamma relative to the raindrop; \
         worst angle between n and where the direct p really points, arcseconds; worst |g_direct / g - 1|; \
         the same for the conditioned p; worst angle between the far-sky directions the two momenta reach; \
         fraction of the sky's solid angle with fate 1 on a 256 x 128 grid; angular diameter of the cone \
         about the direction of motion that shows half the celestial sphere, in arcseconds and in pixels \
         of an 8192 x 4096 frame."
    );
}

#[test]
fn test_the_conditioned_momentum_is_right_where_the_direct_one_has_failed() {
    // At u^t = 1e2 and 1e8 on the freezing worldline. Each momentum is examined by the conditioned
    // inverse `sky_direction`, which says where on the observer's sky it really points.
    //
    // The conditioned momentum must point where it was asked to, with unit frequency and null to
    // rounding, everywhere, to the limit any coordinate vector has: light the observer sees from
    // behind travels, in the raindrop's frame, within ~1 / gamma of a single null direction, so
    // its coordinate components differ between two such directions only at relative 1 / gamma,
    // and a vector of f64 resolves them to about gamma |legs|^2 1e-16 rad on the observer's sky,
    // |legs| ~ 5 here: 2e-14 gamma is the tolerance, and it is a property of the representation,
    // not of either construction. In the cone of radius ~1 / gamma about the direction of motion,
    // where the whole far sky crowds, there is no such limit, and the conditioned momentum must be
    // right to 1e-5 of the cone's size.
    //
    // The direct construction must agree at u^t = 1e2, and at 1e8 must be wrong in that cone by
    // more than the cone's own size: it still points inside the cone, so its error on the sky
    // is small in arcseconds, but which part of the far sky it shows there is noise. (The
    // measurement in this file traces both, and finds the far-sky directions they reach 16
    // arcseconds apart by u^t ~ 1.5e4.)
    let kerr = Kerr::new(1.0, 0.9);
    let events = freezing_events(&kerr, &[2, 8]);
    for (r, phi, u) in events {
        let obs = Observer::new(&kerr, r, phi, u).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let (gamma, _) = triad.boost();
        let f = triad.forward();
        let (mut worst_b, mut cone_b, mut cone_a) = (0.0f64, 0.0f64, 0.0f64);
        for n in directions(&triad) {
            let pb = triad.look(n);
            let (nb, nu) = triad.sky_direction(&kerr, pb);
            let miss = angle_between(nb, n);
            worst_b = worst_b.max(miss).max((nu - 1.0).abs());
            let size = pb.iter().fold(0.0f64, |m, c| m.max(c.abs()));
            let null = kerr.dot(triad.position, pb, pb);
            assert!(
                null.abs() < 1e-12 * size * size,
                "null: {null} with |p| {size}"
            );
            let (na, _) = triad.sky_direction(&kerr, triad.look_direct(n));
            if angle_between(n, f) < 10.0 / gamma {
                cone_b = cone_b.max(miss * gamma);
                cone_a = cone_a.max(angle_between(na, n) * gamma);
            }
        }
        println!(
            "u^t = {:.1e}: conditioned off by {worst_b:.1e} rad at worst ({:.1e} gamma); in the forward cone, in units of its size 1 / gamma, conditioned {cone_b:.1e}, direct {cone_a:.1e}",
            u[0],
            worst_b / gamma
        );
        assert!(
            worst_b < 1e-13 + 2e-14 * gamma,
            "the conditioned momentum at u^t = {}",
            u[0]
        );
        assert!(
            cone_b < 1e-5,
            "the conditioned momentum in the forward cone at u^t = {}",
            u[0]
        );
        if u[0] < 1e3 {
            assert!(
                cone_a < 1e-5,
                "the direct one is still good at u^t = {}",
                u[0]
            );
        } else {
            assert!(
                cone_a > 1.0,
                "the direct one is noise in the forward cone by u^t = {}",
                u[0]
            );
        }
    }
}
