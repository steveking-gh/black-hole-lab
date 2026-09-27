//! The film's frames against answers known in closed form.
//!
//! Each test traces through the same functions the program calls - `hover_observer`, the walker,
//! `triad`, `trace` - on grids of a few hundred pixels, and where a pixel is too coarse to hold the
//! answer to, it bisects for the shadow's edge with the same triad and `kerr_sky::trace_direction`,
//! which is what `trace` does for each pixel.

use std::f64::consts::{FRAC_PI_2, PI};

use kerr_equatorial::{GeodesicState, KerrSchild};
use kerr_sky::{Fate, Kerr, TraceOptions, trace_direction};
use sky_format::{Frame, Grid, fate};

use super::*;

/// The static observer of a hover test at r around spin a, at its first event.
fn hover(r: f64, a: f64) -> (Kerr, Event) {
    let metric = KerrSchild {
        m: 1.0,
        a,
        m_solar: 1.0,
    };
    let obs = hover_observer(&metric, r).unwrap();
    let mut worldline = Worldline::from_saved(&metric, &obs).unwrap();
    let event = worldline.event_at(0.0).unwrap();
    (Kerr::from_equatorial(&metric), event)
}

/// A frame of H rows and 2H columns at an event.
fn frame(kerr: &Kerr, event: &Event, height: u32) -> Frame {
    let traced = trace(kerr, event, 0, 2 * height, height, 8).unwrap();
    assert_eq!(traced.unresolved, 0, "no ray unresolved");
    traced.frame
}

/// Angle between two unit vectors, without the loss of acos near 0 and pi.
fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        .sqrt()
        .atan2(dot)
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / n, v[1] / n, v[2] / n]
}

/// Is the light seen along n (in the triad) dark?
fn dark(kerr: &Kerr, triad: &Triad, n: [f64; 3]) -> bool {
    let out = trace_direction(kerr, triad, unit(n), &TraceOptions::default());
    assert_ne!(
        out.fate,
        Fate::Unresolved,
        "a ray along {n:?} is unresolved"
    );
    out.fate == Fate::Dark
}

/// The parameter s at which the fate of the direction `n(s)` changes, between `inside` (dark) and
/// `outside` (far sky), by bisection to 1e-12.
fn edge(kerr: &Kerr, triad: &Triad, n: impl Fn(f64) -> [f64; 3], inside: f64, outside: f64) -> f64 {
    let (mut lo, mut hi) = (inside, outside);
    assert!(dark(kerr, triad, n(lo)), "the start {lo} is not dark");
    assert!(!dark(kerr, triad, n(hi)), "the end {hi} is not far sky");
    while (hi - lo).abs() > 1e-12 {
        let mid = 0.5 * (lo + hi);
        if dark(kerr, triad, n(mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// The direction of pixel (i, j), by the format's own grid formulae.
fn pixel(frame: &Frame, i: u32, j: u32) -> [f64; 3] {
    Grid::new(frame.width, frame.height).pixel_direction(i, j)
}

/// Synge's angular radius of the shadow for a static observer at r around a = 0, M = 1:
/// sin(alpha) = 3 sqrt(3) (M / r) sqrt(1 - 2M / r), taken on the side alpha < pi/2 for r > 3M.
fn synge(r: f64) -> f64 {
    (3.0 * 3f64.sqrt() / r * (1.0 - 2.0 / r).sqrt()).asin()
}

/// The chart r component of a Cartesian vector v = (v^t, v^x, v^y, v^z) at an event on the plane
/// at (r, phi): from x + iy = (r + ia) e^{i phi}, dx + i dy = e^{i phi} (dr + i (r + ia) dphi), so
/// with A + iB = e^{-i phi} (v^x + i v^y), r dphi = B and dr = A + a B / r.
fn radial_component(kerr: &Kerr, event: &Event, v: [f64; 4]) -> f64 {
    let (s, c) = event.phi.sin_cos();
    let along = c * v[1] + s * v[2];
    let across = -s * v[1] + c * v[2];
    along + kerr.a * across / event.r
}

#[test]
fn test_the_hole_is_ahead_as_a_disc_of_synges_radius() {
    // r = 6, a = 0: sin(alpha) = 3 sqrt(3) / 6 * sqrt(2 / 3) = 1 / sqrt(2), a shadow 45 degrees in
    // radius. An odd number of rows puts a row on the equator. Every pixel whose centre is more
    // than a pixel inside the edge must be dark, every one more than a pixel outside must be far
    // sky; together that says the dark region is a disc of radius alpha about +x, the centre of
    // the frame, to within a pixel.
    let (kerr, event) = hover(6.0, 0.0);
    let alpha = synge(6.0);
    assert!((alpha - PI / 4.0).abs() < 1e-15);
    let f = frame(&kerr, &event, 33);
    let grid = f.grid();
    let pix = PI / f64::from(f.height);
    let centre = grid.offset(f.width / 2, f.height / 2);
    assert_eq!(
        f.fate[centre],
        fate::DARK,
        "the centre of the frame is dark"
    );
    let (mut inside, mut outside) = (0, 0);
    for j in 0..f.height {
        for i in 0..f.width {
            let psi = angle(pixel(&f, i, j), [1.0, 0.0, 0.0]);
            let got = f.fate[grid.offset(i, j)];
            if psi < alpha - pix {
                assert_eq!(got, fate::DARK, "({i}, {j}) at {psi} rad from the centre");
                inside += 1;
            } else if psi > alpha + pix {
                assert_eq!(
                    got,
                    fate::FAR_SKY,
                    "({i}, {j}) at {psi} rad from the centre"
                );
                outside += 1;
            }
        }
    }
    assert!(inside > 50 && outside > 1000, "{inside}, {outside}");
    // Directly behind: the left and right edges of the frame, mid-height.
    for i in [0, f.width - 1] {
        assert_eq!(f.fate[grid.offset(i, f.height / 2)], fate::FAR_SKY);
    }

    // The edge itself, by bisection in eight directions round the centre: a circle of radius
    // alpha about +x, to the tracer's own precision.
    let triad = triad(&kerr, &event).unwrap();
    let mut worst = 0.0f64;
    for k in 0..8 {
        let chi = f64::from(k) * PI / 4.0 + 0.1;
        let (s, c) = chi.sin_cos();
        let psi = edge(
            &kerr,
            &triad,
            |psi| [psi.cos(), psi.sin() * c, psi.sin() * s],
            0.0,
            FRAC_PI_2,
        );
        worst = worst.max((psi - alpha).abs());
    }
    println!("static at 6 M, a = 0: the shadow's edge is within {worst:.1e} rad of Synge's");
    assert!(worst < 1e-8, "{worst}");
}

#[test]
fn test_with_spin_the_shadow_moves_right_and_its_flat_edge_is_on_the_left() {
    // The observer is at phi = 0 facing the hole, z up. Facing -x with z up, the viewer's left is
    // z cross (-x) = -y in the chart: the triad's y = z cross x points to the viewer's left
    // (specification 4.2), and at phi = 0 that is minus the prograde direction d/dphi = +y.
    //
    // Light seen on the left passed the hole on the viewer's left, at y < 0, and was travelling
    // along +x there on its way to the observer, so its angular momentum L_z = x p_y - y p_x =
    // -y p_x is positive: light that passes the hole on the viewer's left is prograde, and light
    // that passes on the right is retrograde. A prograde photon can come closer to the hole before
    // it is captured - its critical impact parameter, from the prograde photon orbit, is the
    // smaller (checked below from `photon_orbit_impact`) - so the shadow's edge on the left lies nearer the
    // hole's direction than the edge on the right: the dark region is pushed to the right, and
    // its left edge, formed by the prograde orbits that hug the hole, is the flattened side of
    // Bardeen's D.
    //
    // As the viewer sees it, the hole's near side moves from left to right: at phi = 0 the
    // rotation d/dphi points along +y, the viewer's right, and the side on the viewer's left
    // (at y < 0, moving along +x) comes toward the viewer.
    let (kerr, event) = hover(6.0, 0.9);
    let (b_pro, b_retro) = (
        kerr.equatorial().photon_orbit_impact(true),
        kerr.equatorial().photon_orbit_impact(false),
    );
    println!(
        "a = 0.9: critical impact parameters {b_pro:.4} M prograde, {b_retro:.4} M retrograde"
    );
    assert!(b_pro.abs() < b_retro.abs());
    // The edge on the sphere: on the equator to the left (lambda < 0) and to the right, and
    // straight up from the middle of those two. The shadow is symmetric under z -> -z, so the
    // bottom point is the top one mirrored.
    let flatness = |kerr: &Kerr, event: &Event| {
        let triad = triad(kerr, event).unwrap();
        let on_equator = |lambda: f64| [lambda.cos(), -lambda.sin(), 0.0];
        let left = edge(kerr, &triad, on_equator, 0.0, -PI + 0.01);
        let right = edge(kerr, &triad, on_equator, 0.0, PI - 0.01);
        let middle = 0.5 * (left + right);
        let (sm, cm) = middle.sin_cos();
        let up = |beta: f64| [beta.cos() * cm, -beta.cos() * sm, beta.sin()];
        let top = up(edge(kerr, &triad, up, 0.0, FRAC_PI_2));
        let bottom = [top[0], top[1], -top[2]];
        let (pl, pr) = (on_equator(left), on_equator(right));
        // The circle on the sky through the right, top and bottom points: its axis is the normal
        // of the plane through them, its radius the angle from that axis to any of them. The left
        // edge falls inside it by `inset`: zero for a circle, positive for a flattened left side.
        let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let (p, q) = (sub(top, pr), sub(bottom, pr));
        let mut axis = unit([
            p[1] * q[2] - p[2] * q[1],
            p[2] * q[0] - p[0] * q[2],
            p[0] * q[1] - p[1] * q[0],
        ]);
        if axis[0] < 0.0 {
            axis = axis.map(|c| -c);
        }
        let inset = angle(axis, pr) - angle(axis, pl);
        (left, right, inset)
    };
    let (left, right, inset) = flatness(&kerr, &event);
    let (kerr0, event0) = hover(6.0, 0.0);
    let (_, _, inset0) = flatness(&kerr0, &event0);
    println!(
        "a = 0.9 at 6 M: the shadow spans {:.4} to {:.4} deg, its middle {:.4} deg to the right; \
         the left edge lies {:.4} deg inside the circle through the other three (a = 0: {inset0:.1e})",
        left.to_degrees(),
        right.to_degrees(),
        (0.5 * (left + right)).to_degrees(),
        inset.to_degrees()
    );
    assert!(inset0.abs() < 1e-8, "at a = 0 the shadow is a circle");
    assert!(left + right > 0.02, "the shadow is displaced to the right");
    assert!(
        -left < right,
        "the left edge is nearer the centre than the right"
    );
    assert!(
        inset > 0.01,
        "the left edge is the flattened one: {inset} rad inside the circle"
    );

    // The same on the frame: on the equator row the dark run's middle is right of centre.
    let f = frame(&kerr, &event, 33);
    let row = f.height / 2;
    let dark_columns: Vec<u32> = (0..f.width)
        .filter(|&i| f.fate[f.grid().offset(i, row)] == fate::DARK)
        .collect();
    let (first, last) = (dark_columns[0], *dark_columns.last().unwrap());
    assert_eq!(dark_columns.len() as u32, last - first + 1, "one dark run");
    assert!(
        first + last + 1 > f.width,
        "the dark run is right of centre: {first} to {last}"
    );
}

#[test]
fn test_a_static_observer_sees_the_far_sky_blueshifted_by_the_static_factor() {
    // g = (-p . u) / E. For u = d_t / sqrt(-g_tt) and light of unit frequency, p_t = -sqrt(-g_tt),
    // so g = 1 / sqrt(-g_tt) for every ray that reaches the far sky, whatever its direction. At
    // a = 0 that is 1 / sqrt(1 - 2M / r). At a = 0.9 the general value comes from the metric; on
    // the equatorial plane of this chart g_tt = -1 + 2 H = -(1 - 2M / r) whatever the spin
    // (H = M r^3 / (r^4 + a^2 z^2) = M / r at z = 0), so the two numbers agree, which the
    // metric is asked, not assumed. Each g is one f64 rounded to f32: within one f32 epsilon.
    for a in [0.0, 0.9] {
        let (kerr, event) = hover(6.0, a);
        let g_tt = kerr.equatorial().metric_components(6.0)[0][0];
        let want = 1.0 / (-g_tt).sqrt();
        if a == 0.0 {
            assert_eq!(want, 1.0 / (1.0 - 2.0 / 6.0f64).sqrt());
        }
        let f = frame(&kerr, &event, 17);
        let mut count = 0;
        for (k, &g) in f.shift.iter().enumerate() {
            if f.fate[k] == fate::FAR_SKY {
                assert!(
                    (f64::from(g) - want).abs() <= f64::from(f32::EPSILON) * want,
                    "a = {a}: g = {g} at ray {k}, want {want}"
                );
                count += 1;
            }
        }
        assert!(count > 300, "{count}");
    }
}

#[test]
fn test_the_rule_for_rays_not_of_the_far_sky_is_checked() {
    let good = || {
        let mut f = Frame::new(4, 2, 0);
        f.fate = vec![1, 1, 2, 0, 1, 2, 2, 1];
        for k in 0..8 {
            if f.fate[k] == 1 {
                for c in 0..3 {
                    f.direction[c][k] = [0.0, 0.6, 0.8][c];
                }
                f.shift[k] = 1.2;
                f.winding[k] = -1;
            }
        }
        f
    };
    assert_eq!(check_rays(&good()), Ok(()));
    let mut f = good();
    f.direction[1][2] = 0.5;
    assert!(
        check_rays(&f)
            .unwrap_err()
            .contains("pixel (2, 0) has fate 2")
    );
    let mut f = good();
    f.shift[3] = 1.0;
    assert!(
        check_rays(&f)
            .unwrap_err()
            .contains("pixel (3, 0) has fate 0")
    );
    let mut f = good();
    f.winding[6] = 2;
    assert!(check_rays(&f).unwrap_err().contains("winding 2"));
    let mut f = good();
    f.fate[0] = 3;
    assert!(check_rays(&f).unwrap_err().contains("reserves"));
}

#[test]
fn test_heading_zero_is_where_r_falls_fastest_in_the_observers_rest_space() {
    // For any unit vector v of the rest space, dr(v) = g(nabla r, v) = g(P nabla r, v), which is
    // least, -|P nabla r| = -sqrt(g^rr + (u^r)^2), for v = -P nabla r / |P nabla r| alone. So the x
    // leg must have r component exactly that. Checked for a static observer in Kerr, a circular
    // orbit, and Bob's fall in the demonstration save from 2.27 M to just outside r-.
    let save = bhl::read_file(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../demos/near_fall.bhl"),
    )
    .unwrap();
    let bob = save.bob.as_ref().unwrap();
    let metric = save.hole.metric();
    let mut bobs = Worldline::from_saved(&metric, bob).unwrap();
    let fall = walk(&mut bobs, 0.25, 1.0, 100);
    assert!(fall.events.len() > 10);
    let (s_kerr, s_event) = hover(6.0, 0.9);
    let (o_kerr, o_event) = circular(8.0);
    let mut cases = vec![(s_kerr, s_event), (o_kerr, o_event)];
    let kerr = Kerr::from_equatorial(&metric);
    cases.extend(fall.events.iter().map(|e| (kerr, *e)));
    let mut worst = 0.0f64;
    for (kerr, event) in &cases {
        let t = triad(kerr, event).unwrap();
        let g_inv = kerr.equatorial().inverse_metric(event.r);
        let want = -(g_inv[1][1] + event.u[1] * event.u[1]).sqrt();
        let got = radial_component(kerr, event, t.x);
        worst = worst.max((got - want).abs() / want.abs());
    }
    println!(
        "x^r = -|P grad r| to {worst:.1e} over {} events, down to r = {:.4} M",
        cases.len(),
        fall.events.last().unwrap().r
    );
    // The x leg is the tetrad's, read in the chart through products of components of the size of
    // u^t, which reaches a few near r-: rounding at 1e-16 (u^t)^2 relative.
    assert!(worst < 1e-10, "{worst}");
}

#[test]
fn test_the_chart_radial_leg_would_turn_the_camera_away_from_the_hole() {
    // What `Triad::new(.., pi)` would have faced - minus the rest-space part of the chart's d/dr -
    // against the gradient of r, for the two observers the module documentation quotes, and for
    // the static observer at a = 0, where the two must agree (d/dr|_KS differs from d/dr|_BL by a
    // multiple of d/dt alone, which a static observer's rest space does not see).
    let tilt = |kerr: &Kerr, event: &Event| {
        let obs = observer(kerr, event).unwrap();
        let chart = Triad::new(kerr, &obs, PI);
        let ours = triad(kerr, event).unwrap();
        kerr.dot(ours.position, chart.x, ours.x)
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees()
    };
    let (kerr, event) = hover(6.0, 0.0);
    let schwarzschild = tilt(&kerr, &event);
    let (kerr, event) = hover(6.0, 0.9);
    let kerr_static = tilt(&kerr, &event);
    let (kerr, event) = circular(8.0);
    let orbit = tilt(&kerr, &event);
    println!(
        "chart d/dr against grad r: static a = 0 {schwarzschild:.2e} deg, static a = 0.9 at 6 M \
         {kerr_static:.3} deg, circular orbit at 8 M {orbit:.3} deg"
    );
    assert!(schwarzschild < 1e-6);
    assert!(kerr_static > 5.0 && orbit > 5.0);
}

/// The circular prograde orbit at r around a = 0, at phi = 0 and t = tau = 0, as a free fall.
fn circular(r: f64) -> (Kerr, Event) {
    let (kerr, mut worldline) = circular_worldline(r);
    (kerr, worldline.event_at(0.0).unwrap())
}

/// Omega = r^{-3/2} and u^t = 1 / sqrt(1 - 3M / r) for M = 1, a = 0; u^r = 0 in either chart.
fn circular_worldline(r: f64) -> (Kerr, Worldline) {
    let metric = KerrSchild {
        m: 1.0,
        a: 0.0,
        m_solar: 1.0,
    };
    let ut = 1.0 / (1.0 - 3.0 / r).sqrt();
    let omega = r.powf(-1.5);
    let (energy, l_ang) = metric.circular_orbit(r, true).unwrap();
    let geo = GeodesicState {
        t: 0.0,
        r,
        phi: 0.0,
        tau: 0.0,
        energy,
        l_ang,
        u: [ut, 0.0, omega * ut],
        stalled: false,
    };
    (
        Kerr::from_equatorial(&metric),
        Worldline::free_fall(&metric, geo).unwrap(),
    )
}

#[test]
fn test_on_a_circular_orbit_the_shadow_holds_still_and_the_sky_turns() {
    // A free fall with angular momentum: the prograde circular orbit at 8 M around a = 0, filmed
    // at 4 M of proper time a frame. The whole scene - the hole, the observer and the camera tied
    // to the hole - turns rigidly about z from frame to frame, by the observer's own change of
    // azimuth, and the metric is invariant under that turn. So every frame must have the same
    // fates, pixel for pixel (the dark region holds still in the frame), while the direction at
    // infinity at every far-sky pixel turns about Z by exactly the observer's azimuth: the sky
    // moves round the hole.
    //
    // Where the shadow sits is set by aberration. A static observer at the same event sees it as
    // a disc of Synge's radius alpha about -r. The orbiter moves across that at v = sqrt(M /
    // (r - 2M)) along the prograde direction, which with x toward the hole and z up is the
    // viewer's right, at longitude lambda = +90 degrees. A direction at angle theta' from the
    // motion in the static frame is seen at cos(theta) = (cos(theta') + v) / (1 + v cos(theta'));
    // the equatorial edges, at theta' = 90 degrees -/+ alpha, are seen at lambda = 90 degrees -
    // theta. The bisected edges must land there.
    let r = 8.0;
    let (kerr, mut worldline) = circular_worldline(r);
    let film = walk(&mut worldline, 4.0, 1.0, 4);
    assert_eq!(film.events.len(), 4);
    let frames: Vec<Frame> = film.events.iter().map(|e| frame(&kerr, e, 17)).collect();
    let grid = frames[0].grid();
    let behind = grid.offset(0, grid.height / 2);
    let mut worst = 0.0f64;
    for (e, f) in film.events.iter().zip(&frames).skip(1) {
        assert_eq!(f.fate, frames[0].fate, "the dark region has not moved");
        let turn = e.phi - film.events[0].phi;
        assert!(turn > 0.2, "the observer has moved on by {turn} rad");
        let (s, c) = turn.sin_cos();
        for k in 0..grid.len() {
            if f.fate[k] != fate::FAR_SKY {
                continue;
            }
            let d0 = [0, 1, 2].map(|i| f64::from(frames[0].direction[i][k]));
            let turned = [c * d0[0] - s * d0[1], s * d0[0] + c * d0[1], d0[2]];
            let d = [0, 1, 2].map(|i| f64::from(f.direction[i][k]));
            let miss = (0..3).map(|i| (d[i] - turned[i]).abs()).fold(0.0, f64::max);
            worst = worst.max(miss);
            if k == behind {
                assert!(angle(d, d0) > 0.2, "the sky behind has moved on");
            }
        }
    }
    println!("circular orbit: far-sky directions turn with the orbit to {worst:.1e}");
    assert!(worst < 1e-6, "{worst}");

    let triad = triad(&kerr, &film.events[0]).unwrap();
    let along_equator = |lambda: f64| [lambda.cos(), -lambda.sin(), 0.0];
    let v = (1.0 / (r - 2.0)).sqrt();
    let alpha = synge(r);
    let seen = |theta_static: f64| {
        let c = theta_static.cos();
        FRAC_PI_2 - ((c + v) / (1.0 + v * c)).acos()
    };
    let (want_right, want_left) = (seen(FRAC_PI_2 - alpha), seen(FRAC_PI_2 + alpha));
    let middle = 0.5 * (want_right + want_left);
    let got_right = edge(&kerr, &triad, along_equator, middle, PI - 0.01);
    let got_left = edge(&kerr, &triad, along_equator, middle, -PI + 0.01);
    println!(
        "circular orbit at {r} M: shadow from {:.4} to {:.4} deg, its middle {:.4} deg to the \
         right; aberration predicts {:.4} to {:.4}",
        got_left.to_degrees(),
        got_right.to_degrees(),
        (0.5 * (got_left + got_right)).to_degrees(),
        want_left.to_degrees(),
        want_right.to_degrees()
    );
    assert!((got_right - want_right).abs() < 1e-8 && (got_left - want_left).abs() < 1e-8);
}

#[test]
fn test_frame_k_is_at_the_saved_proper_time_plus_k_rate_over_fps() {
    assert_eq!(frame_tau(2.5, 0, 0.25, 30.0), 2.5);
    assert_eq!(frame_tau(2.5, 7, 0.25, 30.0), 2.5 + 7.0 * 0.25 / 30.0);
    let (_, mut worldline) = circular_worldline(8.0);
    let film = walk(&mut worldline, 0.25, 30.0, 5);
    for (k, e) in film.events.iter().enumerate() {
        let want = frame_tau(0.0, k, 0.25, 30.0);
        assert!(
            e.tau >= want && e.tau - want < 1e-12,
            "frame {k}: {}",
            e.tau
        );
    }
}

#[test]
fn test_a_hover_inside_the_static_limit_is_refused_and_a_negative_spin_is_refused() {
    for (a, r) in [(0.0, 2.0), (0.9, 1.5), (0.5, -3.0), (0.5, f64::NAN)] {
        let metric = KerrSchild {
            m: 1.0,
            a,
            m_solar: 1.0,
        };
        let err = hover_observer(&metric, r).unwrap_err();
        assert!(err.contains("no static observer exists"), "{err}");
    }
    assert!(check_spin(0.0).is_ok() && check_spin(0.9).is_ok());
    let err = check_spin(-0.5).unwrap_err();
    assert!(
        err.contains("spins the other way") && err.contains("+z"),
        "{err}"
    );
}
