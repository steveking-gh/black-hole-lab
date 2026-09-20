#!/usr/bin/env python3
"""Oracle values for kerr-equatorial, from the textbook Boyer-Lindquist metric and nothing else.

The crate's own tests show that its metric, inverse and Christoffel symbols agree with one another.
They cannot show that the metric is Kerr, because the vacuum equations need theta-derivatives and
the crate has no theta. This script supplies that from outside:

  1. It writes down the four-dimensional Kerr metric in Boyer-Lindquist coordinates, the form every
     textbook prints (Misner, Thorne and Wheeler eq. 33.2; Visser, arXiv:0706.0622, eq. 60).
  2. It carries that metric and its inverse to ingoing Kerr coordinates (t, r, theta, phi) with the
     Jacobian of
         dt = dt_BL + (2 M r / Delta) dr,     dphi = dphi_BL + (a / Delta) dr,
     and none of the crate's algebra: the Kerr-Schild form g = eta + 2 H l l is not an input.
  3. It checks, symbolically, that the result *is* that Kerr-Schild form, with H = M r / rho^2
     and l = (1, 1, 0, -a sin^2 theta).
  4. It checks that the Ricci tensor vanishes, at 50 digits, at points on and off the equatorial
     plane and in all three regions: the metric is a vacuum solution.
  5. It checks that the plane theta = pi/2 is totally geodesic (Gamma^theta_{alpha beta} = 0 for
     alpha, beta in the plane) and that theta decouples from the metric there, so the (t, r, phi)
     block of the 4D Christoffel symbols is what a worldline in the plane feels.
  6. It prints g, g^-1 and Gamma on the plane at the sample points, as Rust source.

Run it from the crate directory, once, and whenever the sample points change:

    python scripts/kerr_sympy_oracle.py > tests/data/kerr_sympy_oracle.rs

`tests/sympy_oracle.rs` includes that file and holds the crate to it. Needs SymPy and mpmath
(`pip install sympy`); written against SymPy 1.14.
"""

import sys

import mpmath
import sympy as sp

mpmath.mp.dps = 50

# (M, a, r) on the equatorial plane. With M = 1 and a = 0.9 the horizons are at r+ = 1.4359 and
# r- = 0.5641, so the first three rows are regions I, II and III; the fourth is inside the
# ergosphere (r < 2M) and outside the horizon of a slower hole; the fifth is Schwarzschild; the
# last has M != 1, which the unit-mass rows cannot tell from a misplaced factor of M.
SAMPLES = [
    ("1", "0.9", "6"),
    ("1", "0.9", "1"),
    ("1", "0.9", "0.3"),
    ("1", "0.5", "1.95"),
    ("1", "0", "3"),
    ("2", "1.2", "5"),
]

# Off the plane as well, for the vacuum check only: (M, a, r, theta).
OFF_PLANE = [
    ("1", "0.9", "6", "0.7"),
    ("1", "0.9", "1", "1.1"),
    ("1", "0.9", "0.3", "2.3"),
    ("2", "1.2", "5", "0.4"),
]

t, r, th, ph = sp.symbols("t r theta phi", real=True)
M, a = sp.symbols("M a", real=True)
X = [t, r, th, ph]

rho2 = r**2 + a**2 * sp.cos(th) ** 2
Delta = r**2 - 2 * M * r + a**2
s2 = sp.sin(th) ** 2


def boyer_lindquist():
    g = sp.zeros(4, 4)
    g[0, 0] = -(1 - 2 * M * r / rho2)
    g[0, 3] = g[3, 0] = -2 * M * a * r * s2 / rho2
    g[1, 1] = rho2 / Delta
    g[2, 2] = rho2
    g[3, 3] = (r**2 + a**2 + 2 * M * a**2 * r * s2 / rho2) * s2
    return g


def to_ingoing_kerr(g_bl):
    # J[mu][nu] = d x_BL^mu / d x_KS^nu: t_BL = t - int 2Mr/Delta dr, phi_BL = phi - int a/Delta dr.
    J = sp.eye(4)
    J[0, 1] = -2 * M * r / Delta
    J[3, 1] = -a / Delta
    g = (J.T * g_bl * J).applyfunc(sp.simplify)
    # The inverse comes from the Boyer-Lindquist inverse, not from inverting g above, so that the
    # two are independent transcriptions carried through the same change of chart.
    ginv = (J.inv() * g_bl.inv() * J.inv().T).applyfunc(sp.simplify)
    return g, ginv


def kerr_schild_form():
    H = M * r / rho2
    l = sp.Matrix([1, 1, 0, -a * s2])
    eta = sp.zeros(4, 4)
    eta[0, 0] = -1
    eta[1, 1] = 1
    eta[1, 3] = eta[3, 1] = -a * s2
    eta[2, 2] = rho2
    eta[3, 3] = (r**2 + a**2) * s2
    return eta + 2 * H * l * l.T


def christoffel(g, ginv):
    dg = [[[sp.diff(g[i, j], X[k]) for k in range(4)] for j in range(4)] for i in range(4)]
    return [
        [
            [
                sum(ginv[m, n] * (dg[n][b][al] + dg[n][al][b] - dg[al][b][n]) for n in range(4)) / 2
                for b in range(4)
            ]
            for al in range(4)
        ]
        for m in range(4)
    ]


def ricci(G):
    R = sp.zeros(4, 4)
    for b in range(4):
        for d in range(b, 4):
            R[b, d] = R[d, b] = sum(
                sp.diff(G[al][b][d], X[al])
                - sp.diff(G[al][b][al], X[d])
                + sum(G[al][al][l] * G[l][b][d] - G[al][d][l] * G[l][b][al] for l in range(4))
                for al in range(4)
            )
    return R


def evaluator(exprs):
    return sp.lambdify((M, a, r, th), exprs, modules="mpmath")


def rust(x):
    # The f64 nearest the 50-digit value, in the shortest decimal that reads back as that f64.
    x = mpmath.mpf(x)
    if abs(x) < mpmath.mpf(10) ** -40:
        return "0.0"
    return repr(float(x))


def main():
    log = lambda *args: print(*args, file=sys.stderr)

    g, ginv = to_ingoing_kerr(boyer_lindquist())
    assert (g * ginv).applyfunc(sp.simplify) == sp.eye(4), "g g^-1 != 1"
    assert (g - kerr_schild_form()).applyfunc(sp.simplify) == sp.zeros(4, 4), "not Kerr-Schild"
    log("Boyer-Lindquist carried to ingoing Kerr coordinates is eta + 2 H l l: ok")

    G = christoffel(g, ginv)
    Ric = ricci(G)
    f_g, f_ginv, f_G, f_Ric = evaluator(g), evaluator(ginv), evaluator(G), evaluator(Ric)

    half_pi = mpmath.pi / 2
    points = [(*map(mpmath.mpf, s), half_pi) for s in SAMPLES]
    points += [tuple(map(mpmath.mpf, s)) for s in OFF_PLANE]
    worst = max(abs(x) for p in points for x in f_Ric(*p))
    assert worst < mpmath.mpf(10) ** -40, f"Ricci = {worst}"
    log(f"Ricci tensor vanishes at {len(points)} points, worst |R_mu_nu| = {mpmath.nstr(worst, 3)}")

    plane = [0, 1, 3]
    out = []
    for sample in SAMPLES:
        p = (*map(mpmath.mpf, sample), half_pi)
        gv, giv, Gv = f_g(*p), f_ginv(*p), f_G(*p)
        tiny = mpmath.mpf(10) ** -40
        for i in plane:
            assert abs(gv[2, i]) < tiny and abs(giv[2, i]) < tiny, "theta does not decouple"
            for j in plane:
                assert abs(Gv[2][i][j]) < tiny, "the plane is not totally geodesic"
                assert abs(Gv[i][2][j]) < tiny, "a polar nudge would leak into the plane"
        mat = lambda m: ", ".join("[" + ", ".join(rust(m[i, j]) for j in plane) + "]" for i in plane)
        gam = ",\n".join(
            "            [" + ", ".join("[" + ", ".join(rust(Gv[m][i][j]) for j in plane) + "]" for i in plane) + "]"
            for m in plane
        )
        out.append(
            "    OraclePoint {\n"
            f"        m: {rust(p[0])}, a: {rust(p[1])}, r: {rust(p[2])},\n"
            f"        g: [{mat(gv)}],\n"
            f"        g_inv: [{mat(giv)}],\n"
            f"        gamma: [\n{gam},\n        ],\n"
            "    },"
        )
    log("theta decouples and the equatorial plane is totally geodesic at every sample: ok")

    print("// Generated by scripts/kerr_sympy_oracle.py - do not edit by hand.")
    print("//")
    print("// The Kerr metric, its inverse and its Christoffel symbols on the equatorial plane, in ingoing")
    print("// Kerr coordinates (t, r, phi), computed by SymPy at 50 digits from the Boyer-Lindquist line")
    print("// element after checking that the Ricci tensor vanishes. Index order [t, r, phi];")
    print("// gamma[mu][alpha][beta] = Gamma^mu_{alpha beta}.")
    print("pub const ORACLE: &[OraclePoint] = &[")
    print("\n".join(out))
    print("];")


if __name__ == "__main__":
    main()
