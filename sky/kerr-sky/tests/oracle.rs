//! Off-plane null rays against an oracle that shares none of this crate's algebra.
//!
//! `scripts/offplane_oracle.py` traces light backward from an equatorial observer in
//! Boyer-Lindquist coordinates, with the separated Carter equations in Mino time and scipy's DOP853
//! at rtol 1e-13, and writes `data/offplane_oracle.rs`: for each ray the observer's radius, the
//! ray's (xi, eta) = (L_z / E, Q / E^2) and the signs of its radial motion and p_theta at the
//! observer, and the direction at infinity d and the azimuth Delta phi the ray turned through. The
//! rays are random ones at four spins and three radii, and six that wind from 2 to 6 radians past
//! a whole turn near the shadow's edge.
//!
//! Here the same light is built in this crate's Cartesian chart and traced by `trace_covector`. The
//! only thing the two share is the event and the four numbers (E, xi, eta, signs) that name the ray.

use kerr_sky::Kerr;
use kerr_sky::ray::{Fate, TraceOptions, trace_covector};

pub struct OracleRay {
    pub a: f64,
    pub r_o: f64,
    pub xi: f64,
    pub eta: f64,
    pub sign_r: f64,
    pub sign_theta: f64,
    pub d: [f64; 3],
    pub delta_phi: f64,
}

include!("data/offplane_oracle.rs");

/// The future-directed covector with E = 1, L_z = xi, Q = eta and the given signs, at the
/// equatorial event (r, phi = 0), in Cartesian components.
///
/// In ingoing Kerr coordinates (t, r, theta, phi) on the equator p = (-1, p_r, p_theta, xi), with
/// p_theta = sign_theta sqrt(eta) (Q = p_theta^2 there) and p_r the root of g^{mu nu} p_mu p_nu = 0
/// whose p^r = g^{r mu} p_mu has the sign asked for. The inverse metric there is
/// `KerrSchild::inverse_metric` with g^{theta theta} = 1 / r^2 added: g^{rr} p_r^2 + B p_r + C = 0
/// with B = 2 (g^{tr} p_t + g^{r phi} p_phi) and p^r = B / 2 + g^{rr} p_r = +-sqrt(B^2 - 4 g^{rr} C) / 2.
/// Then the embedding's Jacobian at phi = 0, where the event is (r, a, 0): p_r = p_x,
/// p_phi = -a p_x + r p_y, p_theta = -r p_z.
fn covector(kerr: &Kerr, ray: &OracleRay) -> ([f64; 3], [f64; 4]) {
    let r = ray.r_o;
    let gi = kerr.equatorial().inverse_metric(r);
    let (p_t, p_phi) = (-1.0, ray.xi);
    let p_theta = ray.sign_theta * ray.eta.sqrt();
    let b = 2.0 * (gi[0][1] * p_t + gi[1][2] * p_phi);
    let c = gi[0][0] * p_t * p_t + gi[2][2] * p_phi * p_phi + p_theta * p_theta / (r * r);
    let disc = (b * b - 4.0 * gi[1][1] * c).sqrt();
    let p_r = (-b + ray.sign_r * disc) / (2.0 * gi[1][1]);
    let p_x = p_r;
    let p_y = (p_phi + kerr.a * p_x) / r;
    let p_z = -p_theta / r;
    (kerr.embed_equatorial(r, 0.0), [p_t, p_x, p_y, p_z])
}

#[test]
fn test_off_plane_rays_reach_the_oracles_direction_at_infinity_and_winding() {
    // The tolerance on d is 1e-9 rad for a ray that does not wind, which is where the oracle's own
    // tail and tolerance leave it, times the ray's own sensitivity to its initial data where it
    // does: a ray that passes within delta of the critical curve turns about ln(1/delta) / (2 pi)
    // more times and multiplies every error by e^{2 pi} (~535) a turn in Schwarzschild. That factor
    // is not estimated but measured, by tracing the same ray from constants moved by 1e-12 of
    // themselves, the rounding either integration starts with.
    let options = TraceOptions::default();
    let mut worst_plain = 0.0f64;
    let mut worst_ratio = 0.0f64;
    for ray in ORACLE {
        let kerr = Kerr::new(1.0, ray.a);
        let (pos, p) = covector(&kerr, ray);
        let c = kerr.constants(pos, p);
        assert!(
            (c.lz - ray.xi).abs() < 1e-13 && (c.carter_q - ray.eta).abs() < 1e-12 * (1.0 + ray.eta)
        );
        assert!(c.hamiltonian.abs() < 1e-13, "null: {}", c.hamiltonian);
        let out = trace_covector(&kerr, pos, p, &options);
        assert_eq!(
            out.fate,
            Fate::FarSky,
            "a = {}, r_o = {}, xi = {}",
            ray.a,
            ray.r_o,
            ray.xi
        );
        let angle = |d: [f64; 3], e: [f64; 3]| {
            let cross = [
                d[1] * e[2] - d[2] * e[1],
                d[2] * e[0] - d[0] * e[2],
                d[0] * e[1] - d[1] * e[0],
            ];
            let s = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
            s.atan2(d[0] * e[0] + d[1] * e[1] + d[2] * e[2])
        };
        let miss = angle(out.direction, ray.d);
        // Sensitivity: the same ray from a covector perturbed by 1e-12 relative.
        let mut q = p;
        q[1] *= 1.0 + 1e-12;
        q[3] *= 1.0 - 1e-12;
        let wobble = angle(
            trace_covector(&kerr, pos, q, &options).direction,
            out.direction,
        );
        let tol = 1e-9 + 1e3 * wobble;
        let turns = ray.delta_phi / std::f64::consts::TAU;
        println!(
            "a = {:<4} r_o = {:<4} xi = {:+.3} eta = {:6.3}: d off by {miss:.2e} rad (tol {tol:.1e}), \
             Delta phi {:+.9} vs {:+.9} ({turns:+.2} turns), winding {}",
            ray.a, ray.r_o, ray.xi, ray.eta, out.delta_phi, ray.delta_phi, out.winding
        );
        assert!(miss < tol, "d off by {miss} rad");
        assert!(
            (out.delta_phi - ray.delta_phi).abs()
                < 1e-8 + 1e3 * wobble.max(1e-12) * ray.delta_phi.abs().max(1.0),
            "Delta phi {} vs {}",
            out.delta_phi,
            ray.delta_phi
        );
        assert_eq!(out.winding, turns.trunc() as i32);
        if wobble < 1e-11 {
            worst_plain = worst_plain.max(miss);
        }
        worst_ratio = worst_ratio.max(miss / tol);
    }
    println!(
        "{} rays: worst miss on well-conditioned rays {worst_plain:.2e} rad; worst miss / tolerance {worst_ratio:.3}",
        ORACLE.len()
    );
}
