//! The crate's metric against an oracle that shares none of its algebra.
//!
//! The unit tests beside `KerrSchild` show that its metric, inverse and Christoffel symbols agree
//! with one another. Agreement is not identity: a geometry can be self-consistent without being
//! Kerr, and the test that would tell - that the Ricci tensor vanishes - needs theta-derivatives,
//! which a chart of the equatorial plane does not have.
//!
//! `scripts/kerr_sympy_oracle.py` supplies that from outside. It starts from the Boyer-Lindquist
//! line element, carries it to ingoing Kerr coordinates by the Jacobian alone, verifies in four
//! dimensions that the result is a vacuum solution and that the equatorial plane is totally
//! geodesic, and writes `data/kerr_sympy_oracle.rs`: g, g^-1 and Gamma on the plane, at 50 digits,
//! at points in all three regions, inside the ergosphere, without spin, and with M != 1.

use kerr_equatorial::KerrSchild;

pub struct OraclePoint {
    pub m: f64,
    pub a: f64,
    pub r: f64,
    pub g: [[f64; 3]; 3],
    pub g_inv: [[f64; 3]; 3],
    pub gamma: [[[f64; 3]; 3]; 3],
}

include!("data/kerr_sympy_oracle.rs");

/// |got - want| against the size of the numbers involved. The crate builds each Christoffel symbol
/// as a sum of a few products, so what double precision can deliver is a handful of ulps of the
/// largest entry at that point and not of the entry itself, which may be a cancellation.
fn assert_close(what: &str, p: &OraclePoint, got: f64, want: f64, scale: f64) {
    let tol = 1e-13 * scale.max(1.0);
    assert!(
        (got - want).abs() <= tol,
        "{what} at M = {}, a = {}, r = {}: crate {got:e}, SymPy {want:e}",
        p.m,
        p.a,
        p.r
    );
}

fn largest(entries: impl Iterator<Item = f64>) -> f64 {
    entries.fold(0.0, |m, v| m.max(v.abs()))
}

#[test]
fn test_metric_and_inverse_match_the_sympy_oracle() {
    for p in ORACLE {
        let ks = KerrSchild::new(p.m, p.a);
        assert_eq!((ks.m, ks.a), (p.m, p.a), "the constructor must not have clamped the sample");
        let (g, g_inv) = (ks.metric_components(p.r), ks.inverse_metric(p.r));
        let g_scale = largest(p.g.iter().flatten().copied());
        let inv_scale = largest(p.g_inv.iter().flatten().copied());
        for mu in 0..3 {
            for nu in 0..3 {
                assert_close(&format!("g[{mu}][{nu}]"), p, g[mu][nu], p.g[mu][nu], g_scale);
                assert_close(&format!("g_inv[{mu}][{nu}]"), p, g_inv[mu][nu], p.g_inv[mu][nu], inv_scale);
            }
        }
    }
}

#[test]
fn test_christoffel_symbols_match_the_sympy_oracle() {
    // These are the (t, r, phi) components of the four-dimensional symbols, which the script has
    // checked are all a worldline in the plane feels: Gamma^theta_{alpha beta} vanishes there.
    for p in ORACLE {
        let gamma = KerrSchild::new(p.m, p.a).christoffel(p.r);
        let scale = largest(p.gamma.iter().flatten().flatten().copied());
        for mu in 0..3 {
            for al in 0..3 {
                for be in 0..3 {
                    assert_close(
                        &format!("Gamma[{mu}][{al}][{be}]"),
                        p,
                        gamma[mu][al][be],
                        p.gamma[mu][al][be],
                        scale,
                    );
                }
            }
        }
    }
}
