//! The plan's validation list, item by item: each test states the closed form it holds the tracer
//! to, where the closed form comes from, and why its tolerance is what it is. Item 1 (the metric's
//! pull-back to `kerr_equatorial`) is in `metric`'s unit tests, item 8 (the reflection symmetry) in
//! `frame`'s, and item 9 (an external oracle) in `tests/oracle.rs`.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use kerr_equatorial::GeodesicState;
use kerr_equatorial::geodesic::geodesic_accel;
use kerr_sky::frame::{pixel_direction, trace_frame};
use kerr_sky::ray::{
    Fate, NullRay, RadialPotential, TraceOptions, past_horizon_margin, trace_covector,
    trace_direction,
};
use kerr_sky::{Kerr, Observer, Triad};

/// A direction at angle alpha from the triad's x leg, at position angle chi about it (chi = 0 is
/// the triad's +y side).
fn cone(alpha: f64, chi: f64) -> [f64; 3] {
    // sin(pi) is 1.2e-16, not 0: snapped, so that position angles 0 and pi are exactly in the plane.
    let (s, c) = chi.sin_cos();
    let s = if s.abs() < 1e-15 { 0.0 } else { s };
    [alpha.cos(), alpha.sin() * c, alpha.sin() * s]
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

/// The edge of the shadow along position angle chi: bisect alpha between a ray that is in the
/// shadow (alpha = 0, looking at the hole) and one that is not (alpha = pi/2), to the last
/// representable alpha. Returns (alpha on the shadow side, alpha on the sky side).
fn shadow_edge(kerr: &Kerr, triad: &Triad, chi: f64, options: &TraceOptions) -> (f64, f64) {
    let (mut inside, mut outside) = (0.0f64, FRAC_PI_2);
    assert_eq!(
        trace_direction(kerr, triad, cone(inside, chi), options).fate,
        Fate::Dark
    );
    assert_eq!(
        trace_direction(kerr, triad, cone(outside, chi), options).fate,
        Fate::FarSky
    );
    while outside - inside > 4.0 * f64::EPSILON {
        let mid = 0.5 * (inside + outside);
        if mid <= inside || mid >= outside {
            break;
        }
        match trace_direction(kerr, triad, cone(mid, chi), options).fate {
            Fate::Dark => inside = mid,
            Fate::FarSky => outside = mid,
            Fate::Unresolved => panic!("unresolved ray at the shadow's edge, alpha = {mid}"),
        }
    }
    (inside, outside)
}

// ---------------------------------------------------------------------------------------------
// Item 2: constants of motion.

#[test]
fn test_constants_of_motion_hold_along_rays_including_rays_that_wind() {
    // E = -p_t is constant by construction (p_t is not a state variable). L_z = x p_y - y p_x,
    // Carter's Q (`Kerr::constants`) and the null condition Ham = (1/2) g^{mu nu} p p = 0 are not:
    // they hold only if the flow is right and the integration accurate, and they are followed
    // step by step along whole rays, from the observer to the far radius or the horizon. The
    // rays are a spread over the sky of a static observer at 6 M around a = 0.9 M, and rays within
    // 1e-6 to 1e-9 of the shadow's edge, which wind round the photon region several times before
    // they leave. Drifts are relative: L_z to max(|L_z|, E M), Q to max(Q, E^2 M^2), Ham to
    // (E^2 + |p|^2) / 2, the size of the terms it cancels. The drift is a random walk of the
    // per-step error allowance (1e-10) over up to a thousand steps; the tolerance of 1e-8 is three
    // times the worst ray's (printed), which is the one that winds most.
    let kerr = Kerr::new(1.0, 0.9);
    let obs = Observer::stationary(&kerr, 6.0, 0.0).unwrap();
    let triad = Triad::new(&kerr, &obs, PI);
    let options = TraceOptions::default();
    let mut rays: Vec<[f64; 3]> = Vec::new();
    for j in 0..5 {
        for i in 0..8 {
            rays.push(pixel_direction(i, j, 8, 5));
        }
    }
    for &chi in &[0.0, 1.1, 2.4, PI] {
        let (_, edge) = shadow_edge(&kerr, &triad, chi, &options);
        for &eps in &[1e-6, 1e-9] {
            rays.push(cone(edge + eps, chi));
        }
    }
    let (mut worst_l, mut worst_q, mut worst_h) = (0.0f64, 0.0f64, 0.0f64);
    let mut most_turns = 0.0f64;
    for n in rays {
        let p = kerr.lower(triad.position, triad.look(n));
        let c0 = kerr.constants(triad.position, p);
        let outcome = trace_direction(&kerr, &triad, n, &options);
        if outcome.fate == Fate::FarSky {
            most_turns = most_turns.max(outcome.delta_phi.abs() / TAU);
        }
        let mut ray = NullRay::backward(&kerr, triad.position, p);
        let (r_plus, margin) = (kerr.outer_horizon(), past_horizon_margin(&kerr));
        for _ in 0..20_000 {
            ray.step(&kerr, options.tolerance, f64::INFINITY).unwrap();
            let pos = ray.position();
            let k = ray.covector();
            let c = kerr.constants(pos, k);
            assert_eq!(c.energy, c0.energy, "E is exact by construction");
            let size = c.energy * c.energy + k[1] * k[1] + k[2] * k[2] + k[3] * k[3];
            worst_l = worst_l.max((c.lz - c0.lz).abs() / c0.lz.abs().max(c0.energy));
            worst_q = worst_q
                .max((c.carter_q - c0.carter_q).abs() / c0.carter_q.max(c0.energy * c0.energy));
            worst_h = worst_h.max(c.hamiltonian.abs() / (0.5 * size));
            let r = kerr.radius(pos);
            let rate = kerr.radial_rate(&ray.state, &ray.velocity());
            if (r > 1e4 && rate > 0.0) || (rate < 0.0 && r < r_plus + margin) {
                break;
            }
        }
    }
    println!(
        "worst relative drift along {} rays: L_z {worst_l:.1e}, Q {worst_q:.1e}, null condition \
         {worst_h:.1e}; the most winding ray turned {most_turns:.2} times",
        48
    );
    assert!(most_turns > 3.0, "the set must include rays that wind");
    assert!(worst_l < 1e-8 && worst_q < 1e-8 && worst_h < 1e-8);
}

// ---------------------------------------------------------------------------------------------
// Item 3: equatorial rays against kerr-equatorial's own geodesic equation.

#[test]
fn test_equatorial_rays_agree_with_kerr_equatorials_geodesic_equation() {
    // A ray that starts in the plane with p_z = 0 stays there (dp_z / d lambda is odd in z), so
    // it must be the null geodesic `kerr_equatorial::geodesic::geodesic_accel` integrates in the
    // chart (t, r, phi): d^2 x^mu / d lambda^2 = -Gamma^mu_{alpha beta} v^alpha v^beta with the
    // app's own Christoffel symbols. Both start from the same event and the same tangent
    // v = dx / d lambda = -p^mu (the backward ray), the chart one integrated with classical RK4 at
    // a fixed 1e-3 of affine parameter, and after the same span of lambda the chart position is
    // pushed through `KerrSchild::cartesian_position` and compared. RK4's global error at that step
    // is ~1e-11 here, so the 1e-8 M tolerance is set by neither integration; the printed agreement
    // is the measure. Observers outside r+ and between the horizons, the latter's rays crossing r+
    // outward on the way.
    let kerr = Kerr::new(1.0, 0.9);
    let eq = kerr.equatorial();
    let r_plus = kerr.outer_horizon();
    let omega_h = kerr.a / (r_plus * r_plus + kerr.a * kerr.a);
    let observers = [
        (Observer::stationary(&kerr, 6.0, 0.3).unwrap(), 8.0f64),
        (
            Observer::new(
                &kerr,
                1.0,
                1.0,
                GeodesicState::new_infall(&eq, 0.0, 1.0, 1.0, 0.5).u,
            )
            .unwrap(),
            5.0,
        ),
    ];
    let mut worst = 0.0f64;
    for (obs, span) in observers {
        let triad = Triad::new(&kerr, &obs, PI);
        for &alpha in &[1.0, FRAC_PI_2, 2.2, PI, -1.3, -2.6] {
            let n = [f64::cos(alpha), f64::sin(alpha), 0.0];
            let p_up = triad.look(n);
            let p = kerr.lower(triad.position, p_up);
            let c = kerr.constants(triad.position, p);
            if obs.r < r_plus && c.energy - omega_h * c.lz < 0.0 {
                continue; // bound for the other branch of r+: it never leaves the horizon's side
            }
            // Chart components of the backward tangent v = -p^mu: the inverse of the embedding's
            // Jacobian, (dx + i dy) = (dr - a dphi + i r dphi) e^{i phi}.
            let (s, co) = obs.phi.sin_cos();
            let (vx, vy) = (-p_up[1], -p_up[2]);
            let radial = vx * co + vy * s;
            let v_phi = (-vx * s + vy * co) / obs.r;
            let mut y = [
                0.0,
                obs.r,
                obs.phi,
                -p_up[0],
                radial + kerr.a * v_phi,
                v_phi,
            ];
            let h = 1e-3;
            let steps = (span / h).round() as usize;
            let rhs = |y: &[f64; 6]| {
                let a = geodesic_accel(&eq, y[1], &[y[3], y[4], y[5]]);
                [y[3], y[4], y[5], a[0], a[1], a[2]]
            };
            for _ in 0..steps {
                let k1 = rhs(&y);
                let k2 = rhs(&core::array::from_fn(|i| y[i] + 0.5 * h * k1[i]));
                let k3 = rhs(&core::array::from_fn(|i| y[i] + 0.5 * h * k2[i]));
                let k4 = rhs(&core::array::from_fn(|i| y[i] + h * k3[i]));
                for i in 0..6 {
                    y[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
                }
            }
            let mut ray = NullRay::backward(&kerr, triad.position, p);
            ray.advance(&kerr, 1e-12, steps as f64 * h).unwrap();
            let (x, yy) = eq.cartesian_position(y[1], y[2]);
            let pos = ray.position();
            let miss = ((pos[0] - x).powi(2) + (pos[1] - yy).powi(2)).sqrt();
            assert_eq!(pos[2], 0.0, "the ray stays in the plane exactly");
            println!(
                "observer at r = {}: ray at alpha = {alpha:+.2} went from r = {} to r = {:.4} M; \
                 chart and Cartesian integrations {miss:.1e} M apart",
                obs.r, obs.r, y[1]
            );
            worst = worst.max(miss);
            assert!(miss < 1e-8, "equatorial ray off by {miss} M");
        }
    }
    println!("worst: {worst:.1e} M");
}

// ---------------------------------------------------------------------------------------------
// Item 4: the flat limit.

#[test]
fn test_flat_space_gives_d_equal_to_n_and_no_shift() {
    // M = 0: every ray is a straight line and nothing shifts it. An observer at rest at phi = 0,
    // whose triad at heading 0 is the far-sky frame's own axes (x = d/dx, y = d/dy, z = d/dz), must
    // see d = n and g = 1 in every pixel: the specification's "still" case. To rounding.
    let kerr = Kerr::new(0.0, 0.0);
    let obs = Observer::stationary(&kerr, 6.0, 0.0).unwrap();
    let triad = Triad::new(&kerr, &obs, 0.0);
    let (w, h) = (16, 8);
    let frame = trace_frame(&kerr, &triad, w, h, &TraceOptions::default(), 2, true);
    let mut worst = 0.0f64;
    for j in 0..h {
        for i in 0..w {
            let k = j * w + i;
            assert_eq!(frame.fate[k], 1);
            let n = pixel_direction(i, j, w, h);
            let d = [0, 1, 2].map(|c| f64::from(frame.direction[c][k]));
            worst = worst.max(angle_between(d, n));
            assert_eq!(frame.shift[k], 1.0);
            assert_eq!(frame.winding[k], 0);
        }
    }
    println!("flat space: d = n to {worst:.1e} rad (f32 storage)");
    assert!(
        worst < 2e-7,
        "f32 has 6e-8 of relative precision per component"
    );
    // And in f64, before the storage rounds it.
    for n in [pixel_direction(3, 2, 16, 8), cone(2.0, 0.7)] {
        let out = trace_direction(&kerr, &triad, n, &TraceOptions::default());
        assert!(angle_between(out.direction, n) < 1e-14);
        assert!((out.shift - 1.0).abs() < 1e-15);
    }
}

#[test]
fn test_a_moving_observer_in_flat_space_sees_relativistic_aberration_and_doppler() {
    // Einstein (1905), section 7: an observer moving at beta relative to the stars sees a star
    // that lies at angle theta_d from the direction of motion (in the stars' frame) at angle
    // theta' with
    //     cos theta_d = (cos theta' - beta) / (1 - beta cos theta'),
    // in the same plane through the direction of motion, and at frequency ratio
    //     g = 1 / (gamma (1 - beta cos theta')).
    // Two motions, radial and azimuthal, at beta = 0.6 and 0.9. In flat space the tetrad's legs
    // are the boosted coordinate directions, so the motion is along the triad's x leg (radial) or
    // y leg (azimuthal) and the stars' frame is the chart's. The tolerance is rounding.
    let kerr = Kerr::new(0.0, 0.0);
    let r = 6.0;
    let mut worst_d = 0.0f64;
    let mut worst_g = 0.0f64;
    for &beta in &[0.6, 0.9] {
        let gamma = 1.0 / (1.0f64 - beta * beta).sqrt();
        for (axis, u) in [
            (0usize, [gamma, gamma * beta, 0.0]),
            (1, [gamma, 0.0, gamma * beta / r]),
        ] {
            let obs = Observer::new(&kerr, r, 0.0, u).unwrap();
            let triad = Triad::new(&kerr, &obs, 0.0);
            for j in 0..6 {
                for i in 0..12 {
                    let n = pixel_direction(i, j, 12, 6);
                    let out = trace_direction(&kerr, &triad, n, &TraceOptions::default());
                    let cos_obs = n[axis];
                    let cos_d = (cos_obs - beta) / (1.0 - beta * cos_obs);
                    let sin_d = (1.0 - cos_d * cos_d).max(0.0).sqrt();
                    // The perpendicular part keeps its direction.
                    let mut perp = n;
                    perp[axis] = 0.0;
                    let norm = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
                    let mut want = [0.0; 3];
                    for c in 0..3 {
                        want[c] = if c == axis {
                            cos_d
                        } else {
                            sin_d * perp[c] / norm
                        };
                    }
                    let g = 1.0 / (gamma * (1.0 - beta * cos_obs));
                    worst_d = worst_d.max(angle_between(out.direction, want));
                    worst_g = worst_g.max((out.shift - g).abs() / g);
                }
            }
        }
    }
    println!("aberration to {worst_d:.1e} rad, Doppler to {worst_g:.1e} relative");
    assert!(worst_d < 1e-13 && worst_g < 1e-13);
}

// ---------------------------------------------------------------------------------------------
// Item 5: Schwarzschild.

#[test]
fn test_a_static_observers_shadow_in_schwarzschild_has_the_synge_radius() {
    // Synge (1966): a static observer at r_o > 3M sees the shadow of a Schwarzschild hole as a disc
    // centred on the direction of the hole, of angular radius
    //     sin(alpha) = 3 sqrt(3) (M / r_o) sqrt(1 - 2M / r_o).
    // The edge is found by bisection along several position angles, off the plane as well as in
    // it, so the disc's roundness is tested along with its size. Twice: with the closed
    // past-horizon criterion and without it.
    //
    // With it, every ray on the shadow side is decided at the observer, exactly where R(r) gets
    // its double root. The rays just outside are not: they have to be integrated round the photon
    // sphere, a ray at b - b_c = delta turning some ln(1/delta) / (2 pi) times before it leaves,
    // and a ray within ~1e-10 of b_c takes ~900 steps there and arrives with its constants moved by
    // about that much - which carries it inside the critical curve, and the proximity criterion
    // then (rightly, for the ray it has become) calls it captured. So the edge is found to about
    // 1e-10 rad on the sky side, measured 4e-11 at 4 M, a millionth of a pixel of an 8K frame, and
    // that is the tolerance. Without the closed criterion the capture side is integrated too.
    let kerr = Kerr::new(1.0, 0.0);
    for &r_o in &[4.0, 6.0, 10.0] {
        let synge = (3.0 * 3f64.sqrt() / r_o * (1.0 - 2.0 / r_o).sqrt()).asin();
        let obs = Observer::stationary(&kerr, r_o, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        for (closed, tol) in [(true, 1e-10), (false, 1e-9)] {
            let options = TraceOptions {
                closed_criterion: closed,
                ..TraceOptions::default()
            };
            let mut worst = 0.0f64;
            for &chi in &[0.0, 0.9, FRAC_PI_2, 2.6, 4.0] {
                let (inside, outside) = shadow_edge(&kerr, &triad, chi, &options);
                let edge = 0.5 * (inside + outside);
                worst = worst.max((edge - synge).abs());
            }
            println!(
                "r_o = {r_o}: Synge radius {synge:.15} rad; edge found {} to {worst:.1e} rad",
                if closed {
                    "by the closed criterion"
                } else {
                    "by integration"
                }
            );
            assert!(worst < tol, "shadow edge off by {worst} rad at r_o = {r_o}");
        }
    }
}

/// The exact Schwarzschild orbit integral I(u1, u_p) = int_{u1}^{u_p} du / sqrt(F(u)),
/// F(u) = 1/b^2 - u^2 + 2 M u^3, with u_p a simple zero of F. The substitution u = u_p - w^2 takes
/// the inverse square root out of the integrand, which is then smooth, and composite Simpson with
/// 20000 panels is exact to ~1e-13.
fn orbit_integral(b: f64, u1: f64, u_p: f64, to_turn: bool) -> f64 {
    let f = |u: f64| 1.0 / (b * b) - u * u + 2.0 * u * u * u;
    let panels = 20_000;
    if !to_turn {
        // No turning point in [u1, u_p]: plain Simpson in u.
        let h = (u_p - u1) / panels as f64;
        let g = |u: f64| 1.0 / f(u).sqrt();
        let mut s = g(u1) + g(u_p);
        for i in 1..panels {
            s += if i % 2 == 1 { 4.0 } else { 2.0 } * g(u1 + h * i as f64);
        }
        return s * h / 3.0;
    }
    let w_max = (u_p - u1).sqrt();
    let h = w_max / panels as f64;
    let fp = -2.0 * u_p + 6.0 * u_p * u_p; // F'(u_p) < 0
    let g = |w: f64| {
        if w == 0.0 {
            2.0 / (-fp).sqrt()
        } else {
            2.0 * w / f(u_p - w * w).sqrt()
        }
    };
    let mut s = g(0.0) + g(w_max);
    for i in 1..panels {
        s += if i % 2 == 1 { 4.0 } else { 2.0 } * g(h * i as f64);
    }
    s * h / 3.0
}

/// The periapsis u_p = 1 / r_p of a Schwarzschild ray of impact parameter b > 3 sqrt(3): the
/// smallest positive zero of F(u) = 1/b^2 - u^2 + 2 u^3, which lies below u = 1/3 (M = 1).
fn periapsis(b: f64) -> f64 {
    let f = |u: f64| 1.0 / (b * b) - u * u + 2.0 * u * u * u;
    let (mut lo, mut hi) = (0.0, 1.0 / 3.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi)
}

#[test]
fn test_schwarzschild_directions_at_infinity_match_the_exact_orbit_integral() {
    // The whole direction at infinity, not just the edge. A static observer at r_o in
    // Schwarzschild who looks at angle alpha from the hole sees light of impact parameter
    // b = r_o sin(alpha) / sqrt(1 - 2M / r_o) (the local angle of a static frame; Synge 1966). The
    // light's orbit lies in the plane through the hole, the observer and n, and it turns about the
    // hole, from the observer to infinity, through
    //     Delta = int_0^{u_o} du / sqrt(F)                         looking away from the hole,
    //     Delta = int_{u_o}^{u_p} du / sqrt(F) + int_0^{u_p} du / sqrt(F)   looking toward it,
    // with F(u) = 1/b^2 - u^2 + 2 M u^3 and u_p the periapsis (Chandrasekhar, The Mathematical
    // Theory of Black Holes, section 20). d is then the observer's own direction from the hole
    // turned through Delta in that plane. Rays from far outside the shadow to 1e-4 of its edge,
    // where the light goes round more than once, which also checks the winding: in the plane, the
    // light's Delta phi is exactly Delta (the backward ray turns clockwise for n_y > 0 with the
    // triad facing the hole). The tolerance, 3e-9 rad, is thirty times the worst the oracle test
    // sees on well-conditioned rays; near the edge the rays' own conditioning dominates (see
    // tests/oracle.rs), and those are held to 3e-9 per turn of amplification e^{2 pi}.
    let kerr = Kerr::new(1.0, 0.0);
    let mut worst = 0.0f64;
    for &r_o in &[6.0, 20.0] {
        let obs = Observer::stationary(&kerr, r_o, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let synge = (3.0 * 3f64.sqrt() / r_o * (1.0 - 2.0 / r_o).sqrt()).asin();
        for &alpha in &[
            synge + 1e-4,
            synge + 1e-2,
            0.5 * (synge + FRAC_PI_2),
            1.4,
            2.0,
            2.9,
        ] {
            let b = r_o * alpha.sin() / (1.0 - 2.0 / r_o).sqrt();
            let u_o = 1.0 / r_o;
            let turn = if alpha < FRAC_PI_2 {
                let u_p = periapsis(b);
                orbit_integral(b, u_o, u_p, true) + orbit_integral(b, 0.0, u_p, true)
            } else {
                orbit_integral(b, 0.0, u_o, false)
            };
            for &chi in &[0.0, 0.8, PI] {
                let n = cone(alpha, chi);
                let out = trace_direction(&kerr, &triad, n, &TraceOptions::default());
                assert_eq!(out.fate, Fate::FarSky);
                // Triad (x, y, z) = (-X, -Y, Z) here, so the initial backward direction's part
                // across the hole's direction X is w = -n_y Y + n_z Z.
                let w = {
                    let v = [0.0, -n[1], n[2]];
                    let l = (v[1] * v[1] + v[2] * v[2]).sqrt();
                    [0.0, v[1] / l, v[2] / l]
                };
                let want = [turn.cos(), turn.sin() * w[1], turn.sin() * w[2]];
                let miss = angle_between(out.direction, want);
                let turns = turn / TAU;
                let tol = 3e-9 * (1.0f64).max(535f64.powf(turns));
                println!(
                    "r_o = {r_o}, alpha = {alpha:.6}, chi = {chi:.1}: b = {b:.6}, turned {turn:.9} \
                     rad ({turns:.2} turns); d off by {miss:.1e} rad"
                );
                assert!(miss < tol, "d off by {miss} rad");
                if chi == 0.0 {
                    assert!(
                        (out.delta_phi - turn).abs() < tol,
                        "Delta phi {} vs {turn}",
                        out.delta_phi
                    );
                    assert_eq!(out.winding, (turn / TAU).trunc() as i32);
                }
                if chi == PI {
                    assert!((out.delta_phi + turn).abs() < tol);
                }
                if turns < 0.5 {
                    worst = worst.max(miss);
                }
            }
        }
    }
    println!("worst on rays that turn less than half a turn: {worst:.1e} rad");
}

#[test]
fn test_weak_field_deflection_tends_to_four_m_over_b() {
    // Light passing a Schwarzschild hole at impact parameter b >> M is bent through
    //     alpha = 4 M/b + (15 pi / 4)(M/b)^2 + (128 / 3)(M/b)^3 + O((M/b)^4)
    // (Keeton and Petters, Phys. Rev. D 72, 104006 (2005), eq. 25 with the coefficients of the
    // Schwarzschild metric; the first term is Einstein's 1915 result), with a fourth-order
    // coefficient 3465 pi / 64 ~ 170; and exactly through alpha = 2 int_0^{u_p} du / sqrt(F) - pi.
    //
    // The ray is started at its periapsis r_p, on the x axis, moving along +y: there dr/d lambda = 0,
    // and at a = 0 the chart's spatial coordinates are Schwarzschild's, so that is the periapsis
    // in any chart. From there the backward ray runs out to infinity through the outbound half of
    // the orbit, turning its position angle through int_0^{u_p} du / sqrt(F), and d must lie at that
    // angle from +x: that is the exact check, of half the deflection, which by the orbit's
    // symmetry about its periapsis is the whole of it halved. (Starting far out on an inbound leg
    // instead would test the step control's hold on the impact parameter over 1e8 M of flight,
    // which no ray from a real observer needs; see the `ray` module documentation.) b is the ray's
    // own conserved sqrt(L_z^2 + Q) / E, and u_p = 1 / r_p.
    let kerr = Kerr::new(1.0, 0.0);
    for &r_p in &[30.0, 100.0, 300.0, 1000.0, 3000.0] {
        let start = [r_p, 0.0, 0.0];
        // The backward tangent v = (v^t, 0, 1, 0), null: g_tt (v^t)^2 + 1 = 0 (g_ty = 0 there, l
        // being radial), past-directed; p = -g v.
        let g_tt = kerr.metric(start)[0][0];
        let v = [-1.0 / (-g_tt).sqrt(), 0.0, 1.0, 0.0];
        let k = kerr.lower(start, v);
        let p = [-k[0], -k[1], -k[2], -k[3]];
        let out = trace_covector(&kerr, start, p, &TraceOptions::default());
        assert_eq!(out.fate, Fate::FarSky);
        let c = out.constants;
        let b = (c.lz * c.lz + c.carter_q).sqrt() / c.energy;
        let half = orbit_integral(b, 0.0, 1.0 / r_p, true);
        let turned = out.direction[1].atan2(out.direction[0]);
        let bent = 2.0 * turned - PI;
        let m_b = 1.0 / b;
        let series = 4.0 * m_b + 15.0 * PI / 4.0 * m_b.powi(2) + 128.0 / 3.0 * m_b.powi(3);
        println!(
            "r_p = {r_p:6}, b = {b:.6}: deflection {bent:.12e}, exact {:.12e}, series {series:.12e}, 4M/b = {:.6e}",
            2.0 * half - PI,
            4.0 * m_b
        );
        assert!(out.direction[2] == 0.0);
        assert!(
            (turned - half).abs() < 1e-10,
            "half the deflection: {turned} vs {half}"
        );
        assert!(
            (bent - series).abs() < 200.0 * m_b.powi(4) + 1e-10,
            "series: {bent} vs {series}"
        );
        assert!((bent * b / 4.0 - 1.0).abs() < 4.0 * m_b, "tends to 4M/b");
    }
}

// ---------------------------------------------------------------------------------------------
// Item 6: Kerr's shadow.

/// Bardeen's critical curve (Bardeen 1973, in Black Holes, eds DeWitt and DeWitt; Chandrasekhar
/// section 63): the spherical photon orbit at radius r has
///     xi(r)  = [r^2 (3M - r) - a^2 (r + M)] / [a (r - M)],
///     eta(r) = r^3 [4 a^2 M - r (r - 3M)^2] / [a^2 (r - M)^2],
/// and a ray with those (xi, eta) = (L_z / E, Q / E^2) is on the edge of every observer's shadow.
fn bardeen(a: f64, r: f64) -> (f64, f64) {
    let xi = (r * r * (3.0 - r) - a * a * (r + 1.0)) / (a * (r - 1.0));
    let eta = r.powi(3) * (4.0 * a * a - r * (r - 3.0).powi(2)) / (a * a * (r - 1.0).powi(2));
    (xi, eta)
}

#[test]
fn test_the_kerr_shadow_edge_lies_on_bardeens_critical_curve() {
    // The edge of the shadow is made of the rays that asymptote to the spherical photon orbits, so
    // their constants lie on Bardeen's curve whatever the observer's distance or motion: the test
    // bisects the edge along eight position angles for a static observer at 8 M around a = 0.9 M,
    // takes the (xi, eta) of the last ray that escapes, solves xi(r_s) = xi for the orbit radius
    // on the curve (xi(r) is monotonic between the two equatorial photon orbits) and compares eta
    // with eta(r_s). On the equator (position angles 0 and pi, n_z = 0) eta = 0 exactly and xi must
    // be the equatorial photon orbit's impact parameter, `KerrSchild::photon_orbit_impact`:
    // prograde on one side, retrograde on the other. With the closed criterion the edge is where
    // R(r) has its double root, found to rounding; without it the integration finds it, and
    // tolerances are what each resolves (printed).
    let a = 0.9;
    let kerr = Kerr::new(1.0, a);
    let eq = kerr.equatorial();
    let obs = Observer::stationary(&kerr, 8.0, 0.0).unwrap();
    let triad = Triad::new(&kerr, &obs, PI);
    let (r_pro, r_retro) = (eq.photon_orbit(true), eq.photon_orbit(false));
    for (closed, tol) in [(true, 1e-9), (false, 1e-6)] {
        let options = TraceOptions {
            closed_criterion: closed,
            ..TraceOptions::default()
        };
        let mut worst_eta = 0.0f64;
        let mut worst_xi = 0.0f64;
        for k in 0..8 {
            let chi = PI * k as f64 / 4.0;
            let (_, outside) = shadow_edge(&kerr, &triad, chi, &options);
            let n = cone(outside, chi);
            let p = kerr.lower(triad.position, triad.look(n));
            let c = kerr.constants(triad.position, p);
            let (xi, eta) = (c.lz / c.energy, c.carter_q / (c.energy * c.energy));
            // xi(r) decreases from the prograde orbit to the retrograde one.
            let (mut lo, mut hi) = (r_pro, r_retro);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if bardeen(a, mid).0 > xi {
                    lo = mid
                } else {
                    hi = mid
                }
            }
            let r_s = 0.5 * (lo + hi);
            let (_, eta_s) = bardeen(a, r_s);
            // And the double root itself: R vanishes at its stationary point r_s.
            let pot = RadialPotential::new(&kerr, &c);
            let double = pot.at(r_s).abs() / (c.energy * c.energy * r_s.powi(4));
            println!(
                "{}: chi = {chi:.3}: edge at alpha = {outside:.12}, xi = {xi:+.10}, eta = {eta:.10}; \
                 on the curve at r_s = {r_s:.8}, eta(r_s) = {eta_s:.10}; R(r_s) / E^2 r_s^4 = {double:.1e}",
                if closed { "closed" } else { "integrated" }
            );
            worst_eta = worst_eta.max((eta - eta_s.max(0.0)).abs() / (1.0 + eta));
            if k % 4 == 0 {
                assert_eq!(n[2], 0.0);
                assert!(eta.abs() < 1e-12, "in the plane Q = 0: {eta}");
                // Which side is prograde: the triad's y leg is -e2 here (heading pi), so chi = 0
                // looks retrograde and the light, arriving along -n, travels prograde.
                let prograde = xi > 0.0;
                let b_c = eq.photon_orbit_impact(prograde);
                worst_xi = worst_xi.max((xi - b_c).abs() / b_c.abs());
            }
        }
        println!(
            "worst |eta - eta(r_s)| / (1 + eta) = {worst_eta:.1e}; equatorial xi against photon_orbit_impact to {worst_xi:.1e}"
        );
        assert!(worst_eta < tol && worst_xi < tol);
    }
}

// ---------------------------------------------------------------------------------------------
// Item 7: the shift seen by a static observer.

#[test]
fn test_a_static_observer_sees_the_far_sky_shifted_by_one_over_sqrt_minus_g_tt_everywhere() {
    // For u = d_t / sqrt(-g_tt) every spatial leg of the triad is orthogonal to d_t, so
    // p_t = g(p, d_t) = g(u, d_t) = -sqrt(-g_tt) for light of unit frequency in the observer's frame
    // whatever the direction, and g = (-p . u) / E = 1 / sqrt(-g_tt) = 1 / sqrt(1 - 2M / r) on the
    // equator. Every far-sky pixel, to rounding in f64 and to f32 storage in the frame.
    for &(a, r) in &[(0.9, 6.0), (0.9, 2.5), (0.0, 3.5)] {
        let kerr = Kerr::new(1.0, a);
        let obs = Observer::stationary(&kerr, r, 0.2).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let want = 1.0 / (1.0 - 2.0 / r).sqrt();
        let (w, h) = (16, 8);
        let frame = trace_frame(&kerr, &triad, w, h, &TraceOptions::default(), 2, true);
        let mut sky = 0;
        for k in 0..w * h {
            if frame.fate[k] == 1 {
                sky += 1;
                assert_eq!(frame.shift[k], want as f32, "g at pixel {k}, r = {r}");
            }
        }
        for n in [cone(2.0, 0.3), cone(2.8, 2.0)] {
            let out = trace_direction(&kerr, &triad, n, &TraceOptions::default());
            assert_eq!(out.fate, Fate::FarSky, "these look away from the hole");
            assert!(
                (out.shift - want).abs() < 1e-14 * want,
                "g = {} vs {want}: {:.1e}",
                out.shift,
                (out.shift - want) / want
            );
        }
        println!("a = {a}, r = {r}: {sky} far-sky pixels, all at g = {want:.9}");
        assert!(sky > 0);
    }
}

// ---------------------------------------------------------------------------------------------
// The two past-horizon criteria, and the far radius.

#[test]
fn test_the_closed_and_fallback_past_horizon_criteria_agree_everywhere_both_apply() {
    // Every ray of a 32 x 16 grid traced twice, with the closed criteria (R(r) positive below the
    // ray in region I; E - Omega_H L_z < 0 between the horizons) and with only the fallbacks
    // (proximity to r+ from outside; the stall against r+ from inside), for observers that see
    // every kind of fate: static outside the photon orbit and inside it, in a prograde orbit, and
    // falling between the horizons on both sides of E - Omega_H L_z. The fates must be identical,
    // and no ray may be unresolved.
    let kerr = Kerr::new(1.0, 0.9);
    let eq = kerr.equatorial();
    let orbit = {
        let ut = eq.circular_orbit_dilation(4.0, true).unwrap();
        [
            ut,
            0.0,
            ut * eq.orbital_angular_velocity(4.0, true).unwrap(),
        ]
    };
    let observers = [
        Observer::stationary(&kerr, 6.0, 0.0).unwrap(),
        Observer::stationary(&kerr, 2.5, 0.0).unwrap(),
        Observer::new(&kerr, 4.0, 1.0, orbit).unwrap(),
        Observer::new(
            &kerr,
            1.0,
            0.0,
            GeodesicState::new_infall(&eq, 0.0, 1.0, 1.0, 2.0).u,
        )
        .unwrap(),
        Observer::new(
            &kerr,
            1.3,
            0.0,
            GeodesicState::new_infall(&eq, 0.0, 1.3, 1.0, -1.0).u,
        )
        .unwrap(),
    ];
    let closed = TraceOptions::default();
    let fallback = TraceOptions {
        closed_criterion: false,
        ..closed
    };
    let (w, h) = (32, 16);
    for obs in observers {
        let triad = Triad::new(&kerr, &obs, PI);
        let mut tally = std::collections::BTreeMap::new();
        for j in 0..h {
            for i in 0..w {
                let n = pixel_direction(i, j, w, h);
                let one = trace_direction(&kerr, &triad, n, &closed);
                let two = trace_direction(&kerr, &triad, n, &fallback);
                assert_eq!(
                    one.fate, two.fate,
                    "pixel ({i}, {j}) at r = {}: {:?} vs {:?}",
                    obs.r, one.decided_by, two.decided_by
                );
                assert_ne!(one.fate, Fate::Unresolved);
                *tally
                    .entry(format!("{:?} / {:?}", one.decided_by, two.decided_by))
                    .or_insert(0) += 1;
            }
        }
        println!("r = {}: {tally:?}", obs.r);
        for key in tally.keys() {
            let (_, second) = key.split_once(" / ").unwrap();
            assert!(
                !second.contains("Closed"),
                "the fallback run used a closed criterion: {key}"
            );
        }
    }
}

#[test]
fn test_the_direction_at_infinity_is_right_to_well_under_sixteen_arcseconds() {
    // Sixteen arcseconds is 7.8e-5 rad, a tenth of a pixel of an 8K equirectangular frame. The
    // reference is the same ray followed to 1e8 M, where the bending still to come, of order
    // M b^3 / r^4 in this chart (`ray` module documentation), is below 1e-25 rad. Against it the
    // default far radius of 1e4 M, and 1e3 M to show the fall-off, over the sky of a static
    // observer at 6 M and of one at 30 M, whose impact parameters reach 35 M. And the
    // integration's own share, by the same rays at a hundredth of the tolerance.
    let kerr = Kerr::new(1.0, 0.9);
    let default = TraceOptions::default();
    let near = TraceOptions {
        far_radius: 1e3,
        ..default
    };
    let reference = TraceOptions {
        far_radius: 1e8,
        ..default
    };
    let tight = TraceOptions {
        tolerance: default.tolerance / 100.0,
        ..default
    };
    let (mut worst_default, mut worst_near, mut worst_tol) = (0.0f64, 0.0f64, 0.0f64);
    for &r_o in &[6.0, 30.0] {
        let obs = Observer::stationary(&kerr, r_o, 0.0).unwrap();
        let triad = Triad::new(&kerr, &obs, PI);
        let (w, h) = (16, 8);
        for j in 0..h {
            for i in 0..w {
                let n = pixel_direction(i, j, w, h);
                let want = trace_direction(&kerr, &triad, n, &reference);
                if want.fate != Fate::FarSky {
                    continue;
                }
                let got = trace_direction(&kerr, &triad, n, &default);
                let closer = trace_direction(&kerr, &triad, n, &near);
                let finer = trace_direction(&kerr, &triad, n, &tight);
                worst_default = worst_default.max(angle_between(got.direction, want.direction));
                worst_near = worst_near.max(angle_between(closer.direction, want.direction));
                worst_tol = worst_tol.max(angle_between(finer.direction, got.direction));
                assert_eq!(got.winding, want.winding);
            }
        }
    }
    let arcsec = PI / 180.0 / 3600.0;
    println!(
        "against 1e8 M: far radius 1e4 M {worst_default:.1e} rad ({:.1e}\"), 1e3 M {worst_near:.1e} rad; a hundredth of the tolerance moves d by {worst_tol:.1e} rad",
        worst_default / arcsec
    );
    assert!(
        worst_default < 1e-9,
        "a hundred-thousandth of the budget: {worst_default}"
    );
    assert!(worst_near < 16.0 * arcsec, "even at 1e3 M");
    assert!(worst_tol < 1e-8);
}
