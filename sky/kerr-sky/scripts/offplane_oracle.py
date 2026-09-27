#!/usr/bin/env python3
"""Oracle for kerr-sky: off-plane null rays traced in Boyer-Lindquist coordinates, Mino time.

The crate follows light with Hamilton's equations in Cartesian ingoing Kerr-Schild coordinates.
This script shares none of that. It starts from the separated first integrals every textbook
prints (Carter 1968; Chandrasekhar, The Mathematical Theory of Black Holes, section 62), in
Boyer-Lindquist coordinates with E = 1 and Mino time sigma (d lambda = Sigma d sigma):

    (dr/d sigma)^2     = R(r)     = (r^2 + a^2 - a xi)^2 - Delta [eta + (xi - a)^2]
    (dtheta/d sigma)^2 = Theta(th) = eta + a^2 cos^2 th - xi^2 cot^2 th
    dphi/d sigma       = a (r^2 + a^2 - a xi) / Delta - a + xi / sin^2 th

with xi = L_z / E and eta = Q / E^2, and integrates the second-order forms
d^2 r / d sigma^2 = R'(r) / 2, d^2 th / d sigma^2 = Theta'(th) / 2, which pass through turning points
without choosing a square root's sign. scipy's DOP853 at rtol 1e-13 carries each ray backward from
an observer on the equator to r = 1e7 M, and the last stretch to infinity is added in closed form
(theta and phi approach their limits as 1/r there: the remaining change is (d theta / dr) r to
O(1/r^2)).

What it prints, for each ray: the observer's radius and the ray's (xi, eta) and the signs of its
future-directed dr/d lambda and p_theta at the observer; and what it found: the direction at
infinity d in the far-sky frame and Delta phi, the change of the ingoing Kerr azimuth along the
light's travel from the far sky to the observer. Boyer-Lindquist and ingoing Kerr azimuths differ by
F(r) = a / (r+ - r-) ln[(r - r+) / (r - r-)], which vanishes at infinity; the observer sits at
ingoing phi = 0, so at Boyer-Lindquist phi = -F(r_o), and Delta phi = F(r_o) - (change of phi_BL
along the backward ray).

Run it from the crate directory, once, and whenever the sample rays change:

    python scripts/offplane_oracle.py > tests/data/offplane_oracle.rs

`tests/oracle.rs` includes that file. Needs numpy and scipy (written against scipy 1.18).
"""

import sys

import numpy as np
from scipy.integrate import solve_ivp

M = 1.0
R_END = 1e7


def trace(a, r_o, xi, eta, sign_r, sign_th):
    rp = M + np.sqrt(M * M - a * a)
    rm = M - np.sqrt(M * M - a * a)

    def delta(r):
        return r * r - 2 * M * r + a * a

    def big_r(r):
        return (r * r + a * a - a * xi) ** 2 - delta(r) * (eta + (xi - a) ** 2)

    def d_big_r(r):
        return 4 * r * (r * r + a * a - a * xi) - (2 * r - 2 * M) * (eta + (xi - a) ** 2)

    def theta_pot(th):
        c, s = np.cos(th), np.sin(th)
        return eta + a * a * c * c - xi * xi * c * c / (s * s)

    def d_theta_pot(th):
        c, s = np.cos(th), np.sin(th)
        return -2 * a * a * c * s + 2 * xi * xi * c / s ** 3

    def phi_rate(r, th):
        return a * (r * r + a * a - a * xi) / delta(r) - a + xi / np.sin(th) ** 2

    if big_r(r_o) <= 0:
        return None

    # State (r, r', th, th', phi) in backward Mino time: the future-directed rates negated.
    y0 = [r_o, -sign_r * np.sqrt(big_r(r_o)), np.pi / 2, -sign_th * np.sqrt(eta), 0.0]

    def rhs(_s, y):
        r, vr, th, vth, _ = y
        return [vr, 0.5 * d_big_r(r), vth, 0.5 * d_theta_pot(th), -phi_rate(r, th)]

    def far(_s, y):
        return y[0] - R_END

    far.terminal = True

    def horizon(_s, y):
        return y[0] - (rp + 1e-3)

    horizon.terminal = True

    sol = solve_ivp(rhs, [0, 50.0], y0, method="DOP853", rtol=1e-13, atol=1e-15,
                    events=[far, horizon])
    if sol.status != 1 or len(sol.t_events[0]) == 0:
        return None
    r, vr, th, vth, phi_bl = sol.y_events[0][0]
    # The tail from r to infinity: d theta / dr = vth / vr ~ c / r^2 integrates to (vth / vr) r.
    th_inf = th + vth / vr * r
    phi_inf = phi_bl + (-phi_rate(r, th)) / vr * r
    f_o = a / (rp - rm) * np.log((r_o - rp) / (r_o - rm)) if a != 0 else 0.0
    # phi_BL at the observer is -F(r_o); phi_BL at infinity is phi_KS there.
    phi_ks_inf = -f_o + phi_inf
    d = [np.sin(th_inf) * np.cos(phi_ks_inf), np.sin(th_inf) * np.sin(phi_ks_inf), np.cos(th_inf)]
    delta_phi = 0.0 - phi_ks_inf
    return d, delta_phi


def main():
    rng = np.random.default_rng(20260927)
    rays = []
    for a in [0.0, 0.5, 0.9, 0.99]:
        for r_o in [3.5, 6.0, 20.0]:
            found = 0
            tries = 0
            while found < 3 and tries < 200:
                tries += 1
                xi = rng.uniform(-9, 9)
                eta = rng.uniform(0, 40)
                sign_r = rng.choice([-1, 1])
                sign_th = rng.choice([-1, 1])
                out = trace(a, r_o, xi, eta, sign_r, sign_th)
                if out is None:
                    continue
                rays.append((a, r_o, xi, eta, sign_r, sign_th, *out))
                found += 1
    # Rays that wind: close to the critical curve of a static observer's shadow edge, found by
    # bisecting eta at fixed xi between a ray that escapes and one that does not.
    # sign_r = +1: the light reaches the observer moving outward, so traced back it heads inward,
    # and it came from the sky only if eta is large enough for it to turn round above r+.
    # Each seed names an eta that is captured (0) and one that escapes, both allowed at r_o.
    for a, r_o, xi, hi in [(0.9, 6.0, 1.0, 20.0), (0.9, 6.0, 2.5, 5.0), (0.5, 20.0, 3.0, 60.0)]:
        lo = 0.0
        if trace(a, r_o, xi, lo, 1, 1) is not None or trace(a, r_o, xi, hi, 1, 1) is None:
            continue
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if trace(a, r_o, xi, mid, 1, 1) is None:
                lo = mid
            else:
                hi = mid
        for offset in [1e-3, 1e-6]:
            eta = hi * (1 + offset)
            out = trace(a, r_o, xi, eta, 1, 1)
            if out is not None:
                rays.append((a, r_o, xi, eta, 1, 1, *out))

    print("// Generated by scripts/offplane_oracle.py - do not edit by hand.")
    print("//")
    print("// Null rays traced backward from an equatorial observer at ingoing phi = 0, in Boyer-Lindquist")
    print("// coordinates and Mino time with scipy's DOP853 at rtol 1e-13: (a, r_o, xi, eta, sign of")
    print("// dr/d lambda, sign of p_theta, d at infinity, Delta phi from the far sky to the observer).")
    print("pub const ORACLE: &[OracleRay] = &[")
    for (a, r_o, xi, eta, sr, sth, d, dphi) in rays:
        print(
            f"    OracleRay {{ a: {a!r}, r_o: {r_o!r}, xi: {float(xi)!r}, eta: {float(eta)!r}, "
            f"sign_r: {float(sr)!r}, sign_theta: {float(sth)!r}, "
            f"d: [{float(d[0])!r}, {float(d[1])!r}, {float(d[2])!r}], delta_phi: {float(dphi)!r} }},"
        )
    print("];")
    print(f"{len(rays)} rays", file=sys.stderr)


if __name__ == "__main__":
    main()
