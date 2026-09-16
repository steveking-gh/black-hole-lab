use crate::physics::kerr_schild::KerrSchild;
use crate::physics::tetrad::Tetrad;

/// A straight line in the drawn (xi^1, xi^0) plane of the observer's local frame: the point is
/// (xi^1, xi^0) and the direction is a Euclidean-unit vector in the same ordering, so
/// `dir[1] / dir[0]` is the diagram slope d xi^0 / d xi^1 and |slope| = 1 is a 45 degree ray.
#[derive(Debug, Clone, Copy)]
pub struct LocalLine {
    /// A point on the line, as (xi^1, xi^0) = (local outward distance, local time).
    pub point: [f64; 2],
    /// Unit direction along the line, as (d xi^1, d xi^0).
    pub dir: [f64; 2],
}

/// Causal character of a surface as read off the line it draws in the local frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceCharacter {
    /// Steeper than 45 degrees: the surface is timelike, so a rocket can stay off it.
    Timelike,
    /// Exactly 45 degrees: the surface is null; the observer is crossing it now.
    Null,
    /// Flatter than 45 degrees: the surface is spacelike, a moment of the observer's history.
    Spacelike,
}

impl LocalLine {
    /// Diagram slope d xi^0 / d xi^1, infinite for a line of constant xi^1.
    pub fn slope(&self) -> f64 {
        if self.dir[0] == 0.0 {
            f64::INFINITY
        } else {
            self.dir[1] / self.dir[0]
        }
    }

    /// Causal character of the line, i.e. of the surface it depicts: the Minkowski norm of the
    /// (Euclidean-unit) direction, -d0^2 + d1^2, is negative for a timelike line. `tol` is the
    /// half-width of the null band in that norm; it lies in [0, 1].
    pub fn character(&self, tol: f64) -> SurfaceCharacter {
        let norm = -self.dir[1] * self.dir[1] + self.dir[0] * self.dir[0];
        if norm.abs() <= tol {
            SurfaceCharacter::Null
        } else if norm < 0.0 {
            SurfaceCharacter::Timelike
        } else {
            SurfaceCharacter::Spacelike
        }
    }

    /// Local time xi^0 at which the line crosses the observer's own worldline xi^1 = 0.
    /// `None` for a line of constant xi^1 that misses the worldline entirely.
    pub fn xi0_at_axis(&self) -> Option<f64> {
        if self.dir[0] == 0.0 {
            return None;
        }
        Some(self.point[1] - self.point[0] * self.dir[1] / self.dir[0])
    }
}

/// The local inertial chart of an observer, to first order in the displacement from their event.
///
/// An orthonormal tetrad {e_a} defines local Minkowski coordinates through its dual basis: for a
/// coordinate displacement Delta x^mu away from the observer's event,
///
///     xi^a = e^a_mu Delta x^mu,      e^a_mu = eta^{ab} g_{mu nu} e_b^nu,   eta = diag(-1, 1, 1)
///
/// with xi^0 the local time, xi^1 the local outward radial distance and xi^2 the local azimuthal
/// distance. The map is linear, so it takes every coordinate plane to a plane and every light cone
/// to the 45 degree cone; that is what makes the picture drawable.
///
/// It is a *first-order* chart. The tetrad is exact at the observer's own event, so orientations
/// there - the tilt of the light cone, the causal character of a surface r = const, the direction
/// of another worldline - are exact. Finite offsets (where the other observer is drawn, how far
/// away a horizon is) are the linearised answer, accurate while the offset is small compared with
/// the curvature radius. The header banner of the view says so.
#[derive(Debug, Clone, Copy)]
pub struct LocalFrame {
    tetrad: Tetrad,
    /// dual[a][mu] = e^a_mu, the covariant legs.
    dual: [[f64; 3]; 3],
    /// Radius of the observer's event, the origin of the chart.
    r: f64,
}

impl LocalFrame {
    /// Build the chart of an arbitrary orthonormal tetrad carried at radius r.
    pub fn new(metric: &KerrSchild, r: f64, tetrad: Tetrad) -> Self {
        let r = r.max(1e-4);
        let g = metric.metric_components(r);
        let legs = [tetrad.e0, tetrad.e1, tetrad.e2];
        // eta^{ab} is diagonal: the timelike row picks up a minus sign, the spacelike ones do not.
        let eta = [-1.0, 1.0, 1.0];
        let mut dual = [[0.0f64; 3]; 3];
        for a in 0..3 {
            for mu in 0..3 {
                let mut s = 0.0;
                for nu in 0..3 {
                    s += g[mu][nu] * legs[a][nu];
                }
                dual[a][mu] = eta[a] * s;
            }
        }
        Self { tetrad, dual, r }
    }

    /// The chart an observer with 4-velocity u uses for the (time, radius) diagram: the tetrad is
    /// `Tetrad::from_four_velocity_axial`, whose azimuthal leg is tangent to the surfaces
    /// r = const, so the drawn plane xi^2 = 0 is transverse to them and `surface_r_const` returns
    /// their exact causal character.
    pub fn for_observer(metric: &KerrSchild, r: f64, u: &[f64; 3]) -> Self {
        let tetrad = Tetrad::from_four_velocity_axial(metric, r, u);
        Self::new(metric, r, tetrad)
    }

    #[allow(dead_code)] // the tests build null directions from the frame's own tetrad
    pub fn tetrad(&self) -> &Tetrad {
        &self.tetrad
    }

    /// Local coordinates xi^a = e^a_mu Delta x^mu of a coordinate displacement (dt, dr, dphi).
    pub fn to_local(self, dx: &[f64; 3]) -> [f64; 3] {
        let mut xi = [0.0f64; 3];
        for a in 0..3 {
            for mu in 0..3 {
                xi[a] += self.dual[a][mu] * dx[mu];
            }
        }
        xi
    }

    /// The same linear map applied to a vector (a 4-velocity, say) rather than to a displacement:
    /// v^a = e^a_mu v^mu.
    pub fn vector_to_local(&self, v: &[f64; 3]) -> [f64; 3] {
        self.to_local(v)
    }

    /// The line drawn by the surface r = r_h in the (xi^1, xi^0) plane.
    ///
    /// A displacement stays on the surface iff its r-component is Delta r = r_h - r_obs. Writing
    /// Delta x^mu = xi^a e_a^mu, that condition reads
    ///
    ///     n_a xi^a = Delta r,      n_a = e_a^mu (dr)_mu = e_a^r,
    ///
    /// so the surface maps to an affine plane whose normal is the covector dr in local components.
    /// The view draws the slice xi^2 = 0 of that plane, the line n_0 xi^0 + n_1 xi^1 = Delta r,
    /// anchored here at its point closest to the origin. Its direction (n_0, -n_1) annihilates
    /// (n_0, n_1), and its slope is d xi^0 / d xi^1 = -n_1 / n_0.
    ///
    /// With the `for_observer` gauge n_2 = 0, so -n_0^2 + n_1^2 = eta^{ab} n_a n_b = g^rr: the
    /// drawn line is steeper than 45 degrees exactly where the surface is timelike (g^rr > 0), at
    /// 45 degrees on either horizon (Delta = 0), and flatter where it is spacelike. Nothing about
    /// the tilt is put in by hand.
    pub fn surface_r_const(&self, r_h: f64) -> LocalLine {
        let n0 = self.tetrad.e0[1];
        let n1 = self.tetrad.e1[1];
        let d_r = r_h - self.r;

        let norm_sq = n0 * n0 + n1 * n1;
        if norm_sq < 1e-300 {
            // dr has no component in the drawn plane at all: nothing sensible to draw, so return
            // the observer's own worldline. Unreachable for a timelike u (n_0 = n_1 = 0 forces
            // g^rr = -n_2^2 <= 0 with dr tangent to the frame's azimuthal leg).
            return LocalLine {
                point: [0.0, 0.0],
                dir: [0.0, 1.0],
            };
        }

        let point = [d_r * n1 / norm_sq, d_r * n0 / norm_sq];
        let inv = norm_sq.sqrt();
        let mut dir = [n0 / inv, -n1 / inv];
        // Canonical orientation: rightward, or upward for a line of constant xi^1.
        if dir[0] < 0.0 || (dir[0] == 0.0 && dir[1] < 0.0) {
            dir = [-dir[0], -dir[1]];
        }
        LocalLine { point, dir }
    }

    /// The line drawn by the surface t = t_obs + `dt` of the chart's time in the (xi^1, xi^0)
    /// plane: one tick mark of the distant clock.
    ///
    /// The ingoing Kerr-Schild t is a Killing time. A difference of t along any static worldline is
    /// the proper time a clock at rest at infinity records between the same two slices, so the
    /// surfaces t = const *are* the distant observer's clock labels, carried inward.
    ///
    /// A displacement stays on one iff its t-component is Delta t = dt. Writing
    /// Delta x^mu = xi^a e_a^mu, exactly as in `surface_r_const` but with the covector dt in place
    /// of dr, that condition reads
    ///
    ///     n_a xi^a = dt,      n_a = e_a^mu (dt)_mu = e_a^t,
    ///
    /// so n_0 = u^t, n_1 = e1^t, n_2 = e2^t. The view draws the slice xi^2 = 0 of that plane, the
    /// line u^t xi^0 + e1^t xi^1 = dt, anchored at its point closest to the origin; its direction
    /// (n_0, -n_1) annihilates (n_0, n_1) and its slope is d xi^0 / d xi^1 = -n_1 / n_0.
    ///
    /// Two things about the drawing, both of them exact statements rather than approximations:
    ///
    /// * n_2 = e2^t is *not* zero in the `for_observer` gauge - that gauge makes only e2^r vanish -
    ///   so the full surface t = const leaves the drawn slice, and what is drawn is its trace, the
    ///   set of its points with xi^2 = 0. The trace is still entirely made of events the distant
    ///   clock labels with the same reading, and nothing about where it meets the observer is lost,
    ///   because the observer's own worldline has xi^1 = xi^2 = 0 and so lies in the drawn slice.
    /// * The line crosses that worldline at xi^0 = dt / u^t exactly (`LocalLine::xi0_at_axis`
    ///   returns the ratio of the components, and the dropped n_2 never enters it). Consecutive
    ///   slices dt apart are therefore spaced by dt / u^t of the observer's own proper time,
    ///   whatever the tilt of the drawn trace: that is the whole content of "the distant clock
    ///   runs fast by the factor u^t = dt / d tau", and it is the spacing the grid is scaled by.
    ///
    /// Every such line is spacelike, everywhere, for every observer. In this chart
    /// g^tt = -(1 + 2M/r) < 0 at every r > 0, and eta^{ab} n_a n_b = g^tt, so
    /// -n_0^2 + n_1^2 + n_2^2 < 0; dropping n_2^2 >= 0 only strengthens it, leaving
    /// |slope| = |n_1| / n_0 < 1. The lines are always flatter than 45 degrees, they never turn
    /// null at a horizon the way the surfaces r = const do, and they pile up on the worldline
    /// exactly when u^t runs away - which is what happens on the way to the far branch of r-,
    /// where u^t grows like exp(kappa_- t), so infinitely many of the distant clock's slices are
    /// crossed in a finite amount of the observer's own time.
    ///
    /// This is a simultaneity convention and not what the observer sees. What is seen is the light,
    /// and the ingoing blueshift in the telemetry box diverges at the same rate on that approach.
    pub fn surface_t_const(&self, dt: f64) -> LocalLine {
        let n0 = self.tetrad.e0[0];
        let n1 = self.tetrad.e1[0];

        let norm_sq = n0 * n0 + n1 * n1;
        if norm_sq < 1e-300 {
            // Unreachable for a future-directed timelike u: n_0 = u^t > 0 along every such
            // worldline in the ingoing chart. Kept so the function is total.
            return LocalLine {
                point: [0.0, 0.0],
                dir: [0.0, 1.0],
            };
        }

        let point = [dt * n1 / norm_sq, dt * n0 / norm_sq];
        let inv = norm_sq.sqrt();
        let mut dir = [n0 / inv, -n1 / inv];
        // Canonical orientation: rightward, or upward for a line of constant xi^1. Flipping both
        // components leaves the slope, and so `xi0_at_axis`, untouched.
        if dir[0] < 0.0 || (dir[0] == 0.0 && dir[1] < 0.0) {
            dir = [-dir[0], -dir[1]];
        }
        LocalLine { point, dir }
    }
}

/// The shortest signed azimuthal offset, in (-pi, pi].
///
/// Every chart built on a difference of azimuths wants this one: an observer three turns round the
/// hole from the focus event is not three turns' worth of local distance away, they are next door,
/// and the shortest offset is the one a first-order chart is linearised about.
pub fn wrap_pi(d_phi: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (d_phi + PI).rem_euclid(TAU) - PI
}

/// The observer's first-order inertial chart, extended through the Kerr-Schild embedding:
///
///     xi^a = E^a_i (Delta t, Delta x, Delta y)^i
///
/// where the columns of C = E^{-1} are the tetrad legs written in Cartesian components,
/// c_a = (e_a^t, J e_a^{(r, phi)}) with J = d(x, y)/d(r, phi) the Jacobian of the embedding at the
/// focus event (`KerrSchild::cartesian_velocity`).
///
/// It is the same chart as `LocalFrame` to first order - the two agree exactly on vectors at the
/// focus event, and differ at a finite offset by the second-order departure of the embedding from
/// its own tangent map - and it draws a different picture, which is the point. In `LocalFrame` the
/// coordinates are (Delta t, Delta r, Delta phi), so a surface r = const is a flat sheet and a
/// circle is a straight line in phi: pipes become planes and helices become lines, and every one of
/// the five things the foliated volume shows is lost. Applied to (Delta t, Delta x, Delta y)
/// instead, the same tetrad takes a cylinder to a (sheared, elliptic) cylinder and a helix to a
/// helix, while keeping the two properties the rest frame exists for: the focus observer's own
/// 4-velocity is the time axis, and the null cone at their event is the exact 45 degree cone.
///
/// Both of those are exact rather than nearly so, because the map is linear and the tetrad is
/// orthonormal against the *coordinate* metric: a vector at the focus event has
/// (k^t, J k) = C v where v are its tetrad components, so E (k^t, J k) = v and nothing is
/// approximated. Finite offsets are the linearised answer, exactly as in `LocalFrame`; the legend
/// of the view that draws this says so.
#[derive(Debug, Clone, Copy)]
pub struct EmbeddedFrame {
    /// E^a_i, the inverse of C: the row index is the chart axis, the column the embedding axis
    /// (t, x, y).
    inv: [[f64; 3]; 3],
    /// Row t of C, i.e. (e_0^t, e_1^t, e_2^t): the covector dt in chart components, which is the
    /// normal of a slice of the distant observer's clock. The same three numbers `LocalFrame`
    /// reads off the tetrad for `surface_t_const`.
    row_t: [f64; 3],
    /// The focus event, the origin of the chart.
    t0: f64,
    r0: f64,
    phi0: f64,
}

impl EmbeddedFrame {
    /// Build the chart of a frame carried at the event (t0, r0, phi0). `None` at a radius the
    /// embedding's Jacobian is singular at, which is the ring r = 0; the floor is `LocalFrame`'s
    /// own.
    ///
    /// E is not obtained by inverting C. C factorises as K L, where L's columns are the tetrad legs
    /// in (t, r, phi) components and K lifts (dt, dr, dphi) to (dt, dx, dy) through the embedding's
    /// Jacobian, so
    ///
    ///     E = C^{-1} = L^{-1} K^{-1} = (dual tetrad) (K^{-1}),
    ///
    /// and both factors are known in closed form: L^{-1} is `LocalFrame`'s own `dual`, and K^{-1}
    /// is the identity on t with the inverse Jacobian on (x, y). That matters at the boosts this
    /// view is looked at under. A numerical inverse of C would compute cofactors of size gamma^2
    /// whose true value is of order one - at gamma = 1e10 the cancellation is total, f64 has 1e4 of
    /// noise left where the answer is 1 - and the chart would come back as garbage on exactly the
    /// worldline the app is about. Written this way nothing is subtracted: the dual tetrad's own
    /// entries are of order gamma and the inverse Jacobian's of order 1/r.
    pub fn new(
        metric: &KerrSchild,
        frame: &LocalFrame,
        t0: f64,
        r0: f64,
        phi0: f64,
    ) -> Option<Self> {
        // The embedding's Jacobian at (r0, phi0) is a rotation by phi0 of [[1, -a], [0, r0]] - read
        // it off d(x + iy) = (dr - a dphi + i r dphi) e^{i phi} - so its inverse is
        // [[1, a/r0], [0, 1/r0]] applied after the rotation back by phi0.
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // a NaN radius has to take this branch too
        if !(r0 > 1e-4) || !phi0.is_finite() || !t0.is_finite() {
            return None;
        }
        let (s0, c0) = phi0.sin_cos();
        let ar = metric.a / r0;
        let j_inv = [
            [c0 - ar * s0, s0 + ar * c0],
            [-s0 / r0, c0 / r0],
        ];
        let tetrad = frame.tetrad();
        let dual = frame.dual;
        // xi^a = e^a_t dt + e^a_r dr + e^a_phi dphi with (dr, dphi) = J^{-1} (dx, dy).
        let inv: [[f64; 3]; 3] = core::array::from_fn(|a| {
            [
                dual[a][0],
                dual[a][1] * j_inv[0][0] + dual[a][2] * j_inv[1][0],
                dual[a][1] * j_inv[0][1] + dual[a][2] * j_inv[1][1],
            ]
        });
        if !inv.iter().flatten().all(|c| c.is_finite()) {
            return None;
        }
        // Row t of C: the t-components of the legs, which the Jacobian never touches.
        let row_t = [tetrad.e0[0], tetrad.e1[0], tetrad.e2[0]];
        row_t.iter().all(|c| c.is_finite()).then_some(Self { inv, row_t, t0, r0, phi0 })
    }

    /// xi^a = E^a_i d^i for an embedding displacement d = (Delta t, Delta x, Delta y).
    fn apply(&self, d: [f64; 3]) -> [f64; 3] {
        core::array::from_fn(|a| {
            self.inv[a][0] * d[0] + self.inv[a][1] * d[1] + self.inv[a][2] * d[2]
        })
    }

    /// The embedding displacement of a coordinate offset (dt, dr, dphi) already measured from the
    /// focus event, with the Cartesian part computed without cancellation.
    ///
    /// `dphi` is taken as given and is expected to lie in (-pi, pi]. It is *not* wrapped here, and
    /// that is the point: `wrap_pi` adds pi before taking the remainder, so an offset of 1e-18 rad
    /// comes back as zero, and 1e-18 rad is the whole of what a pipe wall spans on the canvas at
    /// the boosts of the stall. A caller that has an offset keeps it; a caller that has two
    /// azimuths wraps their difference itself, where the two are far enough apart to survive it.
    fn offset_from(&self, metric: &KerrSchild, dt: f64, dr: f64, dphi: f64) -> [f64; 3] {
        let (dx, dy) = metric.cartesian_displacement(self.r0, self.phi0, dr, dphi);
        [dt, dx, dy]
    }

    /// The embedding displacement from the focus event to (t, r, phi), with the azimuth taken the
    /// short way round.
    fn offset(&self, metric: &KerrSchild, t: f64, r: f64, phi: f64) -> [f64; 3] {
        self.offset_from(metric, t - self.t0, r - self.r0, wrap_pi(phi - self.phi0))
    }

    /// The chart coordinates xi^a of the event (t, r, phi).
    pub fn event(&self, metric: &KerrSchild, t: f64, r: f64, phi: f64) -> [f64; 3] {
        self.apply(self.offset(metric, t, r, phi))
    }

    /// The chart components of a coordinate vector k^mu = (k^t, k^r, k^phi) carried *at* the event
    /// (r, phi), which is where its Jacobian has to be taken: a vector at another event is pushed
    /// forward by the embedding there and then read in this chart's axes.
    ///
    /// At the focus event itself this is exactly `LocalFrame::vector_to_local` - C takes tetrad
    /// components to embedding components, so E takes them back - which is why a null direction
    /// there maps to a null direction and the 4-velocity to (1, 0, 0), both to rounding.
    pub fn vector(&self, metric: &KerrSchild, r: f64, phi: f64, k: &[f64; 3]) -> [f64; 3] {
        let (vx, vy) = metric.cartesian_velocity(r, phi, k[1], k[2]);
        self.apply([k[0], vx, vy])
    }

    /// The point of the world-tube r = const at azimuth `phi` whose chart time is `xi0`.
    ///
    /// At fixed (r, phi) the chart time is affine in the coordinate time,
    /// xi^0(t) = xi^0(t0) + (t - t0) E^0_t, with slope E^0_t = e^0_t = -u_t = E, the conserved
    /// energy of the focus observer, positive for every future-directed timelike u in the ingoing
    /// chart. So the section of the tube at a given chart height is one division, and a pipe drawn
    /// this way has rows at constant chart height in this frame exactly as the global chart's pipe
    /// has rows at constant t. `None` when the slope has collapsed or anything came out
    /// non-finite.
    /// The azimuth is given as an *offset* `dphi` from the focus event's own rather than as an
    /// absolute angle, and that is not a convenience. At the boosts of the stall the arc of a
    /// surface r = const that lands on the canvas spans about 1e-18 rad, and `phi0 + 1e-18` is
    /// `phi0` in f64: an absolute azimuth cannot carry the offset, so the wall would be sampled
    /// nowhere near the canvas and would vanish from the picture exactly where it matters most. An
    /// offset carries it, because 1e-18 next to a `dphi` of 2e-10 is still eight significant
    /// digits.
    pub fn event_at_height_offset(
        &self,
        metric: &KerrSchild,
        r: f64,
        dphi: f64,
        xi0: f64,
    ) -> Option<[f64; 3]> {
        let d = self.offset_from(metric, 0.0, r - self.r0, dphi);
        let slope = self.inv[0][0];
        let base = self.inv[0][1] * d[1] + self.inv[0][2] * d[2];
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // a NaN slope has to take this branch too
        if !(slope.abs() > 1e-300) || !slope.is_finite() || !base.is_finite() || !xi0.is_finite() {
            return None;
        }
        let dt = (xi0 - base) / slope;
        let xi = self.apply([dt, d[1], d[2]]);
        xi.iter().all(|c| c.is_finite()).then_some(xi)
    }

    /// n_a = e_a^t, the covector dt in chart components: the normal of the slices of the distant
    /// observer's clock. The same three numbers `LocalFrame::surface_t_const` is built on.
    pub fn time_normal(&self) -> [f64; 3] {
        self.row_t
    }
}

/// Steps taken across the gap by `ruler_distance`. The integrator advances r by a fixed fraction of
/// the gap per step and shortens the last one to land on the surface, so this sets the resolution
/// of the answer rather than capping the work.
const RULER_STEPS: usize = 2048;

/// One RK4 derivative of the state (r, v^t, v^r, v^phi) against arclength.
fn ruler_deriv(metric: &KerrSchild, y: &[f64; 4]) -> [f64; 4] {
    let v = [y[1], y[2], y[3]];
    let a = crate::physics::geodesic::geodesic_accel(metric, y[0], &v);
    [y[2], a[0], a[1], a[2]]
}

/// Proper distance from an observer's event to the surface r = `r_target`, measured the way that
/// observer measures it: the arclength of the spacelike geodesic that leaves the event along their
/// own radial leg e1 and runs until it meets the surface. It is the radial coordinate of Fermi
/// normal coordinates built on their tetrad, and to first order in the gap it agrees with the
/// `xi^1` intercept `surface_r_const` draws, which is the same construction linearised.
///
/// This *is* the length contraction, done exactly: the observer's own rest space is tilted against
/// the static observers' by their relative boost, and integrating in it accounts for the tilt, for
/// the way their Lorentz factor varies along the path, and for the curvature, in one pass. It is
/// not the static chain of rulers `int dr / sqrt(g^rr)` divided by a Lorentz factor; that rescales
/// somebody else's ruler and answers a different question. It also survives inside the ergosphere,
/// where there is no static chain to rescale, because it asks for nothing but the 4-velocity.
///
/// What it does not supply is a simultaneity convention - it assumes one. "How far away is that
/// surface right now" needs a slicing, and this takes the observer's own. The convention-free
/// answer is radar distance, which for a horizon is infinite, since the ping crosses and nothing
/// returns. Both are true; they are different questions, and the box says which one it is showing.
///
/// `None` when the geodesic never arrives: a turning point in r before the surface, a run into the
/// ring, or a radial leg with no r-component to travel along. Whether the path *ought* to be asked
/// for is the caller's business - the integral would run through a Region II where Delta < 0 quite
/// happily and hand back a number about a curve that has left the observer's rest space, so
/// `gui::spacetime_canvas` only asks across intervals on which Delta > 0 throughout.
pub fn ruler_distance(metric: &KerrSchild, r0: f64, u: &[f64; 3], r_target: f64) -> Option<f64> {
    let r0 = r0.max(1e-4);
    let gap = r_target - r0;
    if gap.abs() < 1e-14 {
        return Some(0.0);
    }
    let sign = gap.signum();

    let tetrad = Tetrad::from_four_velocity_axial(metric, r0, u);
    // e1 is the unit outward leg; take whichever orientation moves r toward the surface.
    let mut v = tetrad.e1;
    if v[1] * sign < 0.0 {
        v = [-v[0], -v[1], -v[2]];
    }
    // The negation is the point rather than a way of writing <=: a leg whose r-component has come
    // out NaN has to take this branch too, and `v[1].abs() <= 1e-12` would let it through. Same
    // below for the step size.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(v[1].abs() > 1e-12) {
        return None;
    }

    let dr_step = (gap / RULER_STEPS as f64).abs();
    // The last stretch is where the accuracy is won or lost. Where the surface is a horizon the
    // leg's own radial rate v^r vanishes on it - for a static observer v^r is exactly sqrt(g^rr) -
    // so the arclength integrand goes like 1 / sqrt(r - r_h): convergent, but no uniform mesh
    // resolves it. Capping each step at a tenth of what is left grades the mesh into the endpoint
    // geometrically, which costs a few hundred extra steps and buys the last six digits.
    let tol = 1e-14 * r_target.abs().max(1.0);
    let mut y = [r0, v[0], v[1], v[2]];
    let mut s = 0.0f64;

    for _ in 0..(RULER_STEPS * 4) {
        let remaining = r_target - y[0];
        if remaining * sign <= 0.0 || remaining.abs() <= tol {
            return Some(s);
        }
        if !y.iter().all(|c| c.is_finite()) || y[0] <= 2.0e-4 {
            return None;
        }
        // The leg has reversed in r: it curls away before it reaches the surface.
        if y[2] * sign < 0.0 {
            return None;
        }
        let h = dr_step.min(0.1 * remaining.abs()) / y[2].abs();
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        if !(h > 0.0) || !h.is_finite() {
            return None;
        }
        let k1 = ruler_deriv(metric, &y);
        let ya: [f64; 4] = core::array::from_fn(|i| y[i] + 0.5 * h * k1[i]);
        let k2 = ruler_deriv(metric, &ya);
        let yb: [f64; 4] = core::array::from_fn(|i| y[i] + 0.5 * h * k2[i]);
        let k3 = ruler_deriv(metric, &yb);
        let yc: [f64; 4] = core::array::from_fn(|i| y[i] + h * k3[i]);
        let k4 = ruler_deriv(metric, &yc);
        for i in 0..4 {
            y[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        s += h;
    }
    None
}

#[cfg(test)]
mod tests {


    /// The ruler distance has a closed form for one observer, and this is it: the only check in
    /// the file that measures the integrator against elementary calculus rather than against
    /// itself.
    ///
    /// The Schwarzschild slice t = const is the fixed-point set of the isometry t -> -t, so it is
    /// totally geodesic - a spacelike geodesic that starts in it tangent to it stays in it. A
    /// static observer's radial leg is tangent to it, so `ruler_distance` has to return that
    /// slice's own radial proper length,
    ///
    ///     int dr / sqrt(1 - 2M/r) = sqrt(r (r - 2M)) + 2M ln(sqrt r + sqrt(r - 2M)),
    ///
    /// exactly. A sign error in the tetrad, a mis-normalised e1 or a mesh that cannot resolve the
    /// 1/sqrt(r - r+) endpoint all show up here and nowhere else.
    ///
    /// The second half is the reason the function exists. A raindrop at the same radius does *not*
    /// measure the static chain divided by its own Lorentz factor: at r = 4M that recipe gives
    /// 3.2465 M and its own rest space gives 2.2548 M. Length contraction in a gravitational field
    /// is not a rescaling of somebody else's ruler, and a read-out built that way would be wrong by
    /// most of a gravitational radius here and by a factor of two in the limit.
    #[test]
    fn test_ruler_distance_matches_the_static_chain_in_schwarzschild() {
        let m = KerrSchild::new(1.0, 0.0);
        let chain = |r: f64| (r * (r - 2.0)).sqrt() + 2.0 * (r.sqrt() + (r - 2.0).sqrt()).ln();

        for &r in &[2.5f64, 3.0, 4.0, 8.0, 20.0] {
            let u_t = 1.0 / (1.0 - 2.0 / r).sqrt();
            let got = ruler_distance(&m, r, &[u_t, 0.0, 0.0], 2.0).expect("a static leg reaches r+");
            let want = chain(r) - chain(2.0);
            assert!(
                (got - want).abs() < 1e-6 * want,
                "ruler distance to r+ from {r} M: {got} vs {want}"
            );
        }

        let x = (0.5f64).sqrt(); // sqrt(2M/r) at r = 4M
        let rain = [(1.0 + x + x * x) / (1.0 + x), -x, 0.0];
        let got = ruler_distance(&m, 4.0, &rain, 2.0).expect("the raindrop's leg reaches r+");
        let contracted = (chain(4.0) - chain(2.0)) * (1.0 - 0.5f64).sqrt();
        assert!((got - 2.2548).abs() < 1e-3, "the raindrop's ruler to r+ = {got}");
        assert!(
            (got - contracted).abs() > 0.9,
            "the raindrop's ruler must not be the static chain contracted: {got} vs {contracted}"
        );
    }


    use super::*;
    use crate::physics::geodesic::GeodesicState;
    use std::f64::consts::PI;

    fn raindrop(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let (ut, ur, up) = GeodesicState::new_infall(metric, 0.0, r, 1.0, 0.0).derivatives(metric, r);
        [ut, ur, up]
    }

    fn zamo(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g = metric.metric_components(r);
        let omega = metric.frame_dragging_omega(r);
        let n = -(g[0][0] + 2.0 * omega * g[0][2] + omega * omega * g[2][2]);
        let gamma = 1.0 / n.sqrt();
        [gamma, 0.0, gamma * omega]
    }

    fn static_obs(metric: &KerrSchild, r: f64) -> [f64; 3] {
        let g_tt = metric.metric_components(r)[0][0];
        [1.0 / (-g_tt).sqrt(), 0.0, 0.0]
    }

    /// Radii spanning every region: exterior, r+, between the horizons, r-, inside r-.
    fn probe_radii(metric: &KerrSchild) -> Vec<f64> {
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon().max(0.05);
        vec![8.0, 3.0, rp, 0.5 * (rp + rm), rm, (0.5 * rm).max(0.05)]
    }

    #[test]
    fn test_dual_tetrad_is_the_inverse_of_the_tetrad() {
        // e^a_mu e_b^mu = delta^a_b, for every admissible frame in every region.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            for &r in probe_radii(&metric).iter() {
                let mut velocities = vec![("free fall", raindrop(&metric, r))];
                if r > rp {
                    velocities.push(("ZAMO", zamo(&metric, r)));
                }
                if metric.metric_components(r)[0][0] < 0.0 {
                    velocities.push(("static", static_obs(&metric, r)));
                }
                for (name, u) in velocities {
                    for frame in [
                        LocalFrame::new(&metric, r, Tetrad::from_four_velocity(&metric, r, &u)),
                        LocalFrame::for_observer(&metric, r, &u),
                    ] {
                        let legs = [frame.tetrad.e0, frame.tetrad.e1, frame.tetrad.e2];
                        for a_idx in 0..3 {
                            for b in 0..3 {
                                let mut s = 0.0;
                                for mu in 0..3 {
                                    s += frame.dual[a_idx][mu] * legs[b][mu];
                                }
                                let expected = if a_idx == b { 1.0 } else { 0.0 };
                                assert!(
                                    (s - expected).abs() < 1e-10,
                                    "e^{a_idx}_mu e_{b}^mu = {s} (want {expected}) for {name} at r={r} (a={a})"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_observer_is_at_rest_and_light_is_isotropic_in_the_local_chart() {
        // u maps to (1, 0, 0) and the null cone maps to (1, cos alpha, sin alpha): the observer is
        // at rest at the origin and light moves at c = 1 in every direction.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in probe_radii(&metric).iter() {
                let u = raindrop(&metric, r);
                let frame = LocalFrame::for_observer(&metric, r, &u);
                let v = frame.vector_to_local(&u);
                assert!(
                    (v[0] - 1.0).abs() < 1e-9 && v[1].abs() < 1e-9 && v[2].abs() < 1e-9,
                    "u maps to {v:?} at r={r} (a={a})"
                );
                for i in 0..16 {
                    let alpha = 2.0 * PI * (i as f64) / 16.0;
                    let k = frame.tetrad().null_direction(alpha);
                    let xi = frame.vector_to_local(&k);
                    let (s, c) = alpha.sin_cos();
                    assert!(
                        (xi[0] - 1.0).abs() < 1e-9 && (xi[1] - c).abs() < 1e-9 && (xi[2] - s).abs() < 1e-9,
                        "k(alpha={alpha}) maps to {xi:?} at r={r} (a={a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_surface_r_const_passes_through_the_mapped_radial_offset() {
        // Every displacement with Delta r = r_h - r_obs lands on the plane n_a xi^a = Delta r, and
        // the drawn line is that plane's slice at xi^2 = 0.
        let metric = KerrSchild::new(1.0, 0.65);
        for &r in probe_radii(&metric).iter() {
            let frame = LocalFrame::for_observer(&metric, r, &raindrop(&metric, r));
            let n = [
                frame.tetrad().e0[1],
                frame.tetrad().e1[1],
                frame.tetrad().e2[1],
            ];
            for &r_h in &[0.0, metric.inner_horizon(), metric.outer_horizon(), 2.0] {
                let line = frame.surface_r_const(r_h);
                let d_r = r_h - r;
                let on_line = n[0] * line.point[1] + n[1] * line.point[0];
                assert!(
                    (on_line - d_r).abs() < 1e-9 * (1.0 + d_r.abs()),
                    "anchor off the plane: {on_line} vs {d_r} at r={r}, r_h={r_h}"
                );
                let along = n[0] * line.dir[1] + n[1] * line.dir[0];
                assert!(along.abs() < 1e-9, "direction leaves the plane: {along}");
                assert!((line.dir[0] * line.dir[0] + line.dir[1] * line.dir[1] - 1.0).abs() < 1e-12);
                // The whole surface really does map to that plane: sample displacements with the
                // right Delta r and arbitrary Delta t, Delta phi.
                for &(dt, dphi) in &[(0.0, 0.0), (0.3, -0.2), (-0.7, 0.4)] {
                    let xi = frame.to_local(&[dt, d_r, dphi]);
                    let s = n[0] * xi[0] + n[1] * xi[1] + n[2] * xi[2];
                    assert!(
                        (s - d_r).abs() < 1e-9 * (1.0 + d_r.abs()),
                        "n . xi = {s} vs Delta r = {d_r} at r={r}, r_h={r_h}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_surface_tilts_follow_the_regions() {
        let metric = KerrSchild::new(1.0, 0.65);
        let rp = metric.outer_horizon();
        let rm = metric.inner_horizon();

        // Region I: r = const is timelike, steeper than 45 degrees.
        let far = LocalFrame::for_observer(&metric, 4.0, &raindrop(&metric, 4.0));
        let line = far.surface_r_const(rp);
        assert!(line.slope().abs() > 1.0, "|slope| = {} at r=4", line.slope().abs());
        assert_eq!(line.character(1e-9), SurfaceCharacter::Timelike);
        // ... and it lies inward of the observer, at negative xi^1.
        assert!(line.point[0] < 0.0, "r+ must be drawn inward: {:?}", line.point);

        // On the horizon itself: the surface is null, exactly 45 degrees, through the origin.
        let on_rp = LocalFrame::for_observer(&metric, rp, &raindrop(&metric, rp));
        let line = on_rp.surface_r_const(rp);
        assert!(
            (line.slope().abs() - 1.0).abs() < 1e-8,
            "|slope| = {} at r = r+",
            line.slope().abs()
        );
        assert_eq!(line.character(1e-9), SurfaceCharacter::Null);
        assert!(line.point[0].abs() < 1e-12 && line.point[1].abs() < 1e-12);

        // Region II: both horizons are spacelike, flatter than 45 degrees; r- is in the future and
        // r+ in the past, which is precisely what "trapped" means.
        let mid = 0.5 * (rp + rm);
        let inside = LocalFrame::for_observer(&metric, mid, &raindrop(&metric, mid));
        let to_rm = inside.surface_r_const(rm);
        let to_rp = inside.surface_r_const(rp);
        assert!(to_rm.slope().abs() < 1.0, "|slope| = {} for r- at r={mid}", to_rm.slope().abs());
        assert_eq!(to_rm.character(1e-9), SurfaceCharacter::Spacelike);
        assert!(to_rm.xi0_at_axis().unwrap() > 0.0, "r- must lie in the future");
        assert!(to_rp.slope().abs() < 1.0);
        assert!(to_rp.xi0_at_axis().unwrap() < 0.0, "r+ must lie in the past");

        // Region III: r = const is timelike again.
        let core = 0.5 * rm;
        let deep = LocalFrame::for_observer(&metric, core, &raindrop(&metric, core));
        let line = deep.surface_r_const(rm);
        assert!(line.slope().abs() > 1.0, "|slope| = {} at r={core}", line.slope().abs());
        assert_eq!(line.character(1e-9), SurfaceCharacter::Timelike);
    }

    #[test]
    fn test_slope_classification_matches_g_upper_rr() {
        // |slope| > 1 iff g^rr > 0, |slope| = 1 iff Delta = 0 - for every observer, in every
        // region, at every spin. This is the whole content of the drawing rule.
        for &a in &[0.0, 0.3, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            let rm = metric.inner_horizon().max(0.05);
            let radii = [
                12.0,
                4.0,
                rp,
                0.75 * rp + 0.25 * rm,
                0.5 * (rp + rm),
                rm,
                0.5 * rm,
                (0.2 * rm).max(0.02),
            ];
            for &r in radii.iter() {
                let mut velocities = vec![raindrop(&metric, r)];
                if r > rp {
                    velocities.push(zamo(&metric, r));
                }
                for u in velocities {
                    let frame = LocalFrame::for_observer(&metric, r, &u);
                    // The line's tilt is a property of the observer's event, not of r_h.
                    for &r_h in &[0.0, rm, rp, 2.0] {
                        let slope = frame.surface_r_const(r_h).slope().abs();
                        let grr = metric.g_upper_rr(r);
                        if metric.delta(r).abs() < 1e-12 {
                            assert!(
                                (slope - 1.0).abs() < 1e-8,
                                "Delta = 0 needs 45 degrees: |slope| = {slope} at r={r} (a={a})"
                            );
                        } else if grr > 0.0 {
                            assert!(
                                slope > 1.0,
                                "g^rr = {grr} > 0 needs |slope| = {slope} > 1 at r={r} (a={a})"
                            );
                        } else {
                            assert!(
                                slope < 1.0,
                                "g^rr = {grr} < 0 needs |slope| = {slope} < 1 at r={r} (a={a})"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_static_observer_sees_vertical_constant_r_surfaces() {
        // A hovering observer's own r = const surface is his worldline: the line is vertical, and
        // every other one is vertical too, offset by the local proper distance.
        let metric = KerrSchild::new(1.0, 0.65);
        let r = 5.0;
        let u = static_obs(&metric, r);
        let frame = LocalFrame::for_observer(&metric, r, &u);
        let own = frame.surface_r_const(r);
        assert!(own.slope().is_infinite());
        assert_eq!(own.character(1e-9), SurfaceCharacter::Timelike);
        assert!(own.point[0].abs() < 1e-12 && own.xi0_at_axis().is_none());

        // Proper distance to the horizon: the anchor's xi^1 is Delta r / e1^r, and e1^r = sqrt(g^rr)
        // for an observer with u^r = 0.
        let horizon = frame.surface_r_const(metric.outer_horizon());
        let expected = (metric.outer_horizon() - r) / metric.g_upper_rr(r).sqrt();
        assert!(
            (horizon.point[0] - expected).abs() < 1e-9 * (1.0 + expected.abs()),
            "xi^1 = {} vs {expected}",
            horizon.point[0]
        );
    }

    #[test]
    fn test_other_observer_maps_into_the_chart() {
        // A second observer a little further out, at the same coordinate time: he is drawn ahead in
        // xi^1 and his worldline direction is timelike and future-directed in the chart.
        let metric = KerrSchild::new(1.0, 0.65);
        let (r_focus, r_other) = (4.0, 4.4);
        let frame = LocalFrame::for_observer(&metric, r_focus, &raindrop(&metric, r_focus));
        let xi = frame.to_local(&[0.0, r_other - r_focus, 0.1]);
        assert!(xi[1] > 0.0, "an observer further out is drawn outward: {xi:?}");

        let v = frame.vector_to_local(&raindrop(&metric, r_other));
        assert!(v[0] > 0.0, "the other worldline must run into the future: {v:?}");
        let speed = (v[1] * v[1] + v[2] * v[2]).sqrt() / v[0];
        assert!(speed < 1.0, "relative speed {speed} must be sub-luminal");
        // v^a is very nearly a unit timelike vector of the local Minkowski metric: exactly unit
        // for a vector at the observer's own event, and unit to first order in the offset for one
        // carried from another event, which is the accuracy this chart claims.
        let n = -v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        assert!((n + 1.0).abs() < 0.05, "eta(v, v) = {n}");
        let here = frame.vector_to_local(&raindrop(&metric, r_focus));
        let n_here = -here[0] * here[0] + here[1] * here[1] + here[2] * here[2];
        assert!((n_here + 1.0).abs() < 1e-9, "eta(v, v) = {n_here} at the observer's own event");
    }

    /// Step the geodesic with E = 1 and this L from r = 4.5 in fixed steps of *coordinate* time,
    /// collecting (t, r, u^mu) at every step. Coordinate time is the right pacing here because the
    /// question the distant clock grid asks is exactly "how much of the observer's own time is one
    /// unit of t worth", and that is u^t.
    fn walk(metric: &KerrSchild, l_ang: f64, dt: f64, t_end: f64) -> Vec<(f64, f64, [f64; 3])> {
        use crate::physics::observer::{Observer, WorldlineParams};
        let mut bob = Observer::new_with_phi(
            metric,
            "Bob",
            0.0,
            4.5,
            0.0,
            0.0,
            WorldlineParams::new(1.0, l_ang, false),
        );
        let mut out = Vec::new();
        let mut t = 0.0;
        while t < t_end && bob.r > 1e-3 {
            t += dt;
            bob.step(metric, t, dt);
            out.push((t, bob.r, bob.four_velocity(metric)));
        }
        out
    }

    #[test]
    fn test_surface_t_const_crosses_the_worldline_at_dt_over_u_t_and_never_tilts_past_45_degrees() {
        // The two claims the distant clock grid rests on, checked for every observer the frame view
        // can be drawn for, in every region, and at every step of the runaway worldline.
        //
        // 1. `xi0_at_axis` is exactly dt / u^t. The drawn trace is the slice xi^2 = 0 of the plane
        //    n_a xi^a = dt with n_a = e_a^t, and the observer's own worldline lies inside that
        //    slice (xi^1 = xi^2 = 0), so dropping n_2 - which is not zero in this gauge, only e2^r
        //    is - cannot move the crossing: u^t xi^0 = dt there. That is what makes the *spacing*
        //    of the grid exact even though the surface itself leaves the drawn plane.
        // 2. |d xi^0 / d xi^1| < 1: these lines are spacelike, always. eta^{ab} n_a n_b = g^tt =
        //    -(1 + 2M/r) < 0 at every r > 0 in this chart, so n_1^2 < n_0^2 - n_2^2 <= n_0^2.
        //    Unlike the surfaces r = const, which turn null at each horizon, a surface t = const
        //    never does: the grid reads the same way on both sides of r+ and of r-.
        let deltas = [-5.0, -1.0, -0.1, 0.0, 0.01, 0.1, 1.0, 5.0, 1e4];
        let mut worst_rel = 0.0f64;
        let mut worst_slope = 0.0f64;
        let mut checked = 0usize;

        fn check(
            frame: &LocalFrame,
            who: &str,
            r: f64,
            deltas: &[f64],
            worst_rel: &mut f64,
            worst_slope: &mut f64,
            checked: &mut usize,
        ) {
            let u_t = frame.tetrad().e0[0];
            assert!(u_t > 0.0, "u^t = {u_t} must be positive for {who} at r={r}");
            for &dt in deltas {
                let line = frame.surface_t_const(dt);
                let xi0 = line
                    .xi0_at_axis()
                    .expect("a line flatter than 45 degrees always meets the worldline");
                let want = dt / u_t;
                let rel = (xi0 - want).abs() / want.abs().max(f64::MIN_POSITIVE);
                assert!(
                    rel < 1e-12,
                    "xi0_at_axis = {xi0} vs dt/u^t = {want} (rel {rel:.3e}) for {who} at r={r}, dt={dt}"
                );
                let slope = line.slope().abs();
                assert!(
                    slope < 1.0,
                    "a surface t = const must be spacelike: |slope| = {slope} for {who} at r={r}"
                );
                assert_eq!(line.character(0.0), SurfaceCharacter::Spacelike);
                *worst_rel = worst_rel.max(rel);
                *worst_slope = worst_slope.max(slope);
                *checked += 1;
            }
        }

        for &a in &[0.0, 0.65, 0.90, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let rp = metric.outer_horizon();
            for &r in probe_radii(&metric).iter() {
                // Free fall exists at every radius, r+ and r- and between them included.
                let frame = LocalFrame::for_observer(&metric, r, &raindrop(&metric, r));
                check(&frame, "free fall", r, &deltas, &mut worst_rel, &mut worst_slope, &mut checked);
                if r > rp {
                    let frame = LocalFrame::for_observer(&metric, r, &zamo(&metric, r));
                    check(&frame, "ZAMO", r, &deltas, &mut worst_rel, &mut worst_slope, &mut checked);
                }
                if metric.metric_components(r)[0][0] < 0.0 {
                    let frame = LocalFrame::for_observer(&metric, r, &static_obs(&metric, r));
                    check(&frame, "static", r, &deltas, &mut worst_rel, &mut worst_slope, &mut checked);
                }
            }
        }

        // The worldline the grid is really about: E = 1, L = 2.2 at a = 0.90 has E - Omega_- L < 0,
        // so it is bound for the far branch of r- and its u^t runs away like exp(kappa_- t). The
        // exactness of the crossing has to survive u^t in the thousands, which is where a formula
        // that had leaned on the dropped n_2 would show up.
        let metric = KerrSchild::new(1.0, 0.90);
        let track = walk(&metric, 2.2, 0.1, 45.0);
        let mut u_t_max = 0.0f64;
        for &(_, r, u) in track.iter() {
            let frame = LocalFrame::for_observer(&metric, r, &u);
            check(&frame, "E=1, L=2.2 infaller", r, &deltas, &mut worst_rel, &mut worst_slope, &mut checked);
            u_t_max = u_t_max.max(u[0]);
        }
        println!(
            "surface_t_const: {checked} (observer, dt) pairs, including {} steps of the L = 2.2 \
             walk up to u^t = {u_t_max:.4e}; worst relative error in xi0_at_axis = {worst_rel:.3e}; \
             the steepest line seen still missed 45 degrees, by 1 - |slope| = {:.3e}",
            track.len(),
            1.0 - worst_slope
        );
    }

    #[test]
    fn test_the_distant_clock_piles_up_on_the_far_branch_of_r_minus_and_not_on_the_way_to_r_plus() {
        // What the grid is for, as a number. The lines of the distant clock, dt apart in the
        // chart's Killing time, cross the observer's worldline dt / u^t of their own proper time
        // apart, so the spacing of the grid *is* 1 / u^t.
        //
        // Bound for the far branch of r- (E - Omega_- L < 0, here E = 1 and L = 2.2 at a = 0.90),
        // the worldline never reaches r-: it settles onto it asymptotically, and an outgoing
        // principal null ray closes on r- like exp(-kappa_- t), so u^t grows like exp(+kappa_- t)
        // and the spacing collapses like exp(-kappa_- t). Over the last 10 M of the walk that is a
        // definite number, exp(-kappa_- * 10), and it is checked against one - not against a fit.
        // Infinitely many of the distant clock's moments are then crossed in a finite amount of the
        // observer's own time, which is the grid piling up against the origin of the frame view.
        let metric = KerrSchild::new(1.0, 0.90);
        let kappa = metric.inner_surface_gravity();
        let dt = 0.1;
        let track = walk(&metric, 2.2, dt, 45.0);
        let spacing_at = |t_want: f64| -> (f64, f64, f64) {
            let &(t, r, u) = track
                .iter()
                .min_by(|a, b| {
                    (a.0 - t_want)
                        .abs()
                        .partial_cmp(&(b.0 - t_want).abs())
                        .unwrap()
                })
                .expect("the walk has steps");
            (t, r, dt / u[0])
        };
        let (t35, r35, s35) = spacing_at(35.0);
        let (t45, r45, s45) = spacing_at(45.0);
        let ratio = s45 / s35;
        let predicted = (-kappa * (t45 - t35)).exp();
        println!(
            "L = 2.2 (far branch of r- = {:.4}): at t = {t35:.1}, r = {r35:.6}, one dt = {dt} of \
             distant time is {s35:.4e} M of proper time; at t = {t45:.1}, r = {r45:.6}, it is \
             {s45:.4e} M. Ratio {ratio:.4e} vs exp(-kappa_- * {:.1}) = {predicted:.4e} \
             (kappa_- = {kappa:.6}), off by {:.1}%",
            metric.inner_horizon(),
            t45 - t35,
            100.0 * (ratio / predicted - 1.0).abs()
        );
        assert!(r45 > metric.inner_horizon(), "the far branch is never crossed");
        assert!(
            (ratio / predicted - 1.0).abs() < 0.30,
            "the pile-up rate must be kappa_-: ratio {ratio:.4e} vs exp(-kappa_- Delta t) = {predicted:.4e}"
        );

        // The L = 0 raindrop is the control. E - Omega_- L = 1 > 0, so it crosses r+ and then the
        // near branch of r- in finite coordinate time with u^t finite the whole way: nothing piles
        // up, the grid keeps very nearly the same step, and the view through both horizons is calm.
        let rain = walk(&metric, 0.0, dt, 60.0);
        let through = rain
            .iter()
            .take_while(|&&(_, r, _)| r > metric.inner_horizon())
            .map(|&(t, r, u)| (t, r, dt / u[0]))
            .collect::<Vec<_>>();
        let crossed_rp = through.iter().any(|&(_, r, _)| r < metric.outer_horizon());
        assert!(crossed_rp, "the raindrop must actually reach r+");
        assert!(
            rain.iter().any(|&(_, r, _)| r <= metric.inner_horizon()),
            "the raindrop must actually reach r-"
        );
        let lo = through.iter().fold(f64::INFINITY, |m, &(_, _, s)| m.min(s));
        let hi = through.iter().fold(0.0f64, |m, &(_, _, s)| m.max(s));
        let first = through.first().unwrap();
        let last = through.last().unwrap();
        println!(
            "L = 0 raindrop: from t = {:.1}, r = {:.4} (spacing {:.4e} M) to t = {:.1}, r = {:.4} \
             (spacing {:.4e} M); over the whole fall through r+ = {:.4} to r- = {:.4} the spacing \
             stays within a factor {:.4}",
            first.0,
            first.1,
            first.2,
            last.0,
            last.1,
            last.2,
            metric.outer_horizon(),
            metric.inner_horizon(),
            hi / lo
        );
        assert!(
            hi / lo < 3.0,
            "nothing piles up on the way through r+ and the near branch of r-: factor {}",
            hi / lo
        );
    }

    /// The embedded chart of an observer with 4-velocity u at (t0, r0, phi0).
    fn embedded(
        metric: &KerrSchild,
        t0: f64,
        r0: f64,
        phi0: f64,
        u: &[f64; 3],
    ) -> (LocalFrame, EmbeddedFrame) {
        let frame = LocalFrame::for_observer(metric, r0, u);
        let embed = EmbeddedFrame::new(metric, &frame, t0, r0, phi0)
            .expect("a timelike tetrad at r > 0 has an invertible embedding matrix");
        (frame, embed)
    }

    #[test]
    fn test_the_embedded_frame_agrees_with_the_local_frame_to_first_order() {
        // The two charts are the same first-order chart: they are built on the same tetrad and
        // differ only in whether the displacement is written as (Delta t, Delta r, Delta phi) or
        // as (Delta t, Delta x, Delta y). The embedding's own Jacobian is what carries one into
        // the other, and the tetrad is pushed through exactly that Jacobian, so the whole
        // difference between them is the second-order departure of the embedding from its tangent
        // map - and shrinking the offset by ten has to shrink the disagreement by a hundred.
        //
        // That is the licence for swapping one for the other: no first-order statement the rest
        // frame makes changes, and the picture does, because r = const is a flat sheet in one and
        // a cylinder in the other.
        let metric = KerrSchild::new(1.0, 0.65);
        let core = (0.5 * metric.inner_horizon()).max(0.1);
        for &r0 in &[8.0, 3.0, metric.outer_horizon(), core] {
            let (phi0, t0) = (0.9, 2.0);
            let u = raindrop(&metric, r0);
            let (frame, embed) = embedded(&metric, t0, r0, phi0, &u);
            let gap = |d: f64| -> f64 {
                let here = embed.event(&metric, t0 + d, r0 + d, phi0 + d);
                let there = frame.to_local(&[d, d, d]);
                (0..3).map(|a| (here[a] - there[a]).abs()).fold(0.0f64, f64::max)
            };
            let (big, small) = (gap(1e-2), gap(1e-3));
            let ratio = big / small;
            println!(
                "r0 = {r0:.4}: the two charts differ by {big:.3e} at delta = 1e-2 and by \
                 {small:.3e} at 1e-3 (ratio {ratio:.1})"
            );
            assert!(small > 0.0, "the two charts are not literally the same map");
            assert!(
                (ratio - 100.0).abs() < 25.0,
                "the disagreement has to be second order in the offset, so the ratio must be \
                 about 100 - but it is {ratio} at r0 = {r0}"
            );
        }
    }

    #[test]
    fn test_the_focus_four_velocity_is_the_time_axis_of_the_embedded_frame() {
        // The first of the two properties the rest frame exists for, and it is exact rather than
        // nearly so: C's columns *are* the tetrad legs in embedding components, so E takes a
        // vector at the focus event back to its tetrad components, and u is e_0.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r0 in probe_radii(&metric).iter() {
                for phi0 in [0.0, 1.3, -2.7] {
                    let u = raindrop(&metric, r0);
                    let (_, embed) = embedded(&metric, 0.0, r0, phi0, &u);
                    let v = embed.vector(&metric, r0, phi0, &u);
                    assert!(
                        (v[0] - 1.0).abs() < 1e-9 && v[1].abs() < 1e-9 && v[2].abs() < 1e-9,
                        "u maps to {v:?} at r = {r0}, phi = {phi0} (a = {a})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_a_null_direction_at_the_focus_event_is_null_in_the_embedded_frame() {
        // The other property, and the hard requirement on the picture: the focus observer's cone
        // is the 45 degree cone. Nothing draws it at 45 degrees - the null directions at the event
        // are mapped by the same linear map everything else goes through, and they come out on the
        // unit cone because the tetrad is orthonormal.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r0 in probe_radii(&metric).iter() {
                let u = raindrop(&metric, r0);
                let (frame, embed) = embedded(&metric, 0.0, r0, 0.7, &u);
                let mut worst = 0.0f64;
                for i in 0..12 {
                    let alpha = 2.0 * PI * (i as f64) / 12.0;
                    let k = frame.tetrad().null_direction(alpha);
                    let v = embed.vector(&metric, r0, 0.7, &k);
                    worst = worst.max((-v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).abs());
                }
                assert!(
                    worst < 1e-9,
                    "a null direction at r = {r0} (a = {a}) came out with eta(v, v) = {worst}"
                );
            }
        }
    }

    #[test]
    fn test_the_embedded_frame_survives_the_boost_of_the_stall() {
        // The regime the whole app is about: on the approach to the far branch of r- the focus
        // observer's u^t runs away like exp(kappa_- t), and the view is looked at at a boost of a
        // million and more. The chart has to still be a chart there.
        //
        // This is the test that says why E is written down rather than computed. C's entries are
        // all of size gamma, so a cofactor of C is a difference of two numbers of size gamma^2
        // whose true value is of order one: at the gamma = 1.4e6 below, that is 2e12 with an ulp of
        // 4e-4, so a numerical inverse of C loses twelve of its sixteen digits, and by the 1e10 of
        // the stall it has lost all of them and returns noise. Factoring C = K L and reading
        // L^{-1} off the dual tetrad subtracts nothing at all.
        let metric = KerrSchild::new(1.0, 0.9);
        // A runaway u^t in closed form: a static observer a picometre outside the static limit,
        // where u^t = 1 / sqrt(1 - 2M/r) is a million. Its components are computed from numbers of
        // order one, which is the conditioning the integrator hands the view on the approach to
        // r-; gamma * (e_0 + v e_1) with v = 1 - 1e-20 is not, because in f64 that v is 1 and the
        // vector is simply null.
        let r = metric.ergosphere_equatorial() + 1e-12;
        let u = static_obs(&metric, r);
        let base = Tetrad::from_four_velocity_axial(&metric, r, &raindrop(&metric, r));
        let (frame, embed) = embedded(&metric, 0.0, r, 0.8, &u);
        let u_t = frame.tetrad().e0[0];
        assert!(u_t > 1e5, "the hovering observer really is at a runaway u^t: {u_t:e}");

        // The two charts are the same map on a vector at the focus event - E (k^t, J k) = e^a_mu
        // k^mu - so the embedded answer has to be the tetrad's own answer, to rounding. That is
        // the statement a numerical inverse of C fails: it returns cofactor noise a thousand times
        // larger than the answer, and the two disagree by four orders of magnitude rather than by
        // one part in 1e10.
        let mut worst = 0.0f64;
        let mut null_worst = 0.0f64;
        for i in 0..12 {
            let alpha = 2.0 * PI * (i as f64) / 12.0;
            // The raindrop frame's generators, which are the ones the volume samples its rim in
            // and whose components are of order one at every boost.
            let k = base.null_direction(alpha);
            let here = embed.vector(&metric, r, 0.8, &k);
            let there = frame.vector_to_local(&k);
            let scale = there.iter().fold(0.0f64, |m, c| m.max(c.abs())).max(1e-300);
            for a in 0..3 {
                worst = worst.max((here[a] - there[a]).abs() / scale);
            }
            let n = -here[0] * here[0] + here[1] * here[1] + here[2] * here[2];
            null_worst = null_worst
                .max(n.abs() / (here[0] * here[0] + here[1] * here[1] + here[2] * here[2]));
        }
        println!(
            "at u^t = {u_t:.4e} the embedded chart and the tetrad's own agree on a null direction \
             to {worst:.3e} relative, and it stays null to {null_worst:.3e}"
        );
        assert!(
            worst < 1e-9,
            "the embedded chart has to be the same map as the tetrad's at the focus event, but at \
             a boost of 1e10 they differ by {worst} relative"
        );
        assert!(null_worst < 1e-9, "and the cone is still the cone: eta / |v|^2 = {null_worst}");

        // The chart itself is of the size of the boost rather than of the size of the noise: an
        // offset of 1e-12 M - which is what is left of the surface ahead at the stall - comes back
        // as something a picture can be drawn from.
        let xi = embed.event(&metric, 0.0, r - 1e-12, 0.8);
        println!("and an offset of 1e-12 M inward maps to {xi:?}");
        assert!(xi.iter().all(|c| c.is_finite()), "the chart is finite at that boost: {xi:?}");
        let reach = xi.iter().fold(0.0f64, |m, c| m.max(c.abs()));
        assert!(
            reach > 1e-9 && reach < 1e-2,
            "1e-12 M mapped through a boost of 1e6 is of order 1e-6 M, not {reach:e}"
        );
    }

    #[test]
    fn test_the_embedded_frame_is_periodic_in_azimuth() {
        // The chart is built on a difference of azimuths, and an azimuth is an angle: an observer
        // three turns round the hole is next door. Without the wrap the far side of a pipe would
        // be drawn a whole circumference's worth of boost away from the near side of it.
        let metric = KerrSchild::new(1.0, 0.9);
        let (r0, phi0) = (3.0, 0.4);
        let (_, embed) = embedded(&metric, 1.0, r0, phi0, &raindrop(&metric, r0));
        for &phi in &[0.0, 1.1, -2.2, 4.0] {
            for turns in [-2.0, -1.0, 1.0, 3.0] {
                let here = embed.event(&metric, 1.5, 2.2, phi);
                let round = embed.event(&metric, 1.5, 2.2, phi + turns * std::f64::consts::TAU);
                for a in 0..3 {
                    assert!(
                        (here[a] - round[a]).abs() < 1e-9 * (1.0 + here[a].abs()),
                        "phi = {phi} and phi + {turns} turns are the same event, but they map to \
                         {here:?} and {round:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_event_at_height_lands_at_that_height() {
        // What a pipe is drawn from: the point of the tube r = const at a given azimuthal offset
        // whose chart time is a given height. xi^0 is affine in t at fixed (r, dphi), so the solve
        // is one division, and the answer has to come back at exactly the height asked for.
        //
        // The offsets include 1e-18 rad, which is the width of the arc that lands on the canvas at
        // the boosts of a late fall. It is a real offset here and not zero, and that is the whole
        // reason this takes an offset rather than an azimuth.
        let metric = KerrSchild::new(1.0, 0.9);
        let (rp, rm, re) = (
            metric.outer_horizon(),
            metric.inner_horizon(),
            metric.ergosphere_equatorial(),
        );
        for &r0 in &[8.0, 3.0, rp] {
            let (_, embed) = embedded(&metric, 0.0, r0, 0.3, &raindrop(&metric, r0));
            let mut worst = 0.0f64;
            for &r in &[rp, rm, re] {
                for &dphi in &[-PI, -1.0, -1e-18, 0.0, 1e-18, 1.0, PI] {
                    for &xi0 in &[-3.0f64, -0.5, 0.0, 0.25, 2.0] {
                        let xi = embed
                            .event_at_height_offset(&metric, r, dphi, xi0)
                            .expect("a future-directed observer has E > 0, so the solve exists");
                        worst = worst.max((xi[0] - xi0).abs());
                        assert!(
                            (xi[0] - xi0).abs() < 1e-9 * (1.0 + xi0.abs()),
                            "the section of r = {r} at dphi = {dphi} came back at xi^0 = {} rather \
                             than at {xi0} (observer at r = {r0})",
                            xi[0]
                        );
                    }
                }
            }
            println!(
                "observer at r = {r0:.4}: every section landed on its own height to {worst:.2e}"
            );
        }
    }

    #[test]
    fn test_far_from_the_hole_the_embedded_frame_is_the_global_chart() {
        // The sanity check on the whole construction. Far from a hole with no spin, a hovering
        // observer's tetrad is the coordinate frame, so the chart is (Delta t, Delta x, Delta y)
        // itself - which is what the global foliation draws. The rest frame is not a different
        // picture of a different spacetime; it is the same picture, boosted and sheared.
        let metric = KerrSchild::new(1.0, 0.0);
        let r0 = 1e4;
        let u = static_obs(&metric, r0);
        let (_, embed) = embedded(&metric, 0.0, r0, 0.0, &u);
        let mut worst = 0.0f64;
        for &(dt, dr, dphi) in &[(0.0, 5.0, 0.0), (3.0, 0.0, 1e-4), (-2.0, -4.0, 2e-4)] {
            let xi = embed.event(&metric, dt, r0 + dr, dphi);
            let (dx, dy) = metric.cartesian_displacement(r0, 0.0, dr, dphi);
            let want = [dt, dx, dy];
            let scale = want.iter().fold(0.0f64, |m, c| m.max(c.abs())).max(1e-12);
            for a in 0..3 {
                worst = worst.max((xi[a] - want[a]).abs() / scale);
            }
        }
        println!(
            "a static observer at r = 1e4 M, a = 0: the chart is (dt, dx, dy) to {worst:.2e} \
             relative"
        );
        assert!(worst < 1e-3, "far away the embedded chart is the global one: {worst:e} out");
    }

    #[test]
    fn test_the_horizon_pipe_is_tangent_to_the_cone_at_the_crossing() {
        // The statement the old rest-frame picture made with a plane drawn at exactly 45 degrees,
        // made here about the pipe that replaces it. The tangent plane of the tube r = r+ at the
        // focus event is spanned by the images of d/dt and d/dphi, and on the horizon that plane
        // is null: it touches the light cone along one generator, d/dt + Omega+ d/dphi, and
        // contains no timelike direction at all. That is what "the surface the observer is
        // crossing is the cone's own wall" means, and the pipe says it by being tangent rather
        // than by being drawn at an angle.
        let metric = KerrSchild::new(1.0, 0.9);
        let rp = metric.outer_horizon();
        let phi0 = 0.6;
        let (_, embed) = embedded(&metric, 0.0, rp, phi0, &raindrop(&metric, rp));
        let v_t = embed.vector(&metric, rp, phi0, &[1.0, 0.0, 0.0]);
        let v_p = embed.vector(&metric, rp, phi0, &[0.0, 0.0, 1.0]);
        let eta = |w: [f64; 3]| -w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
        let unit = |w: [f64; 3]| -> Option<[f64; 3]> {
            let len = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
            (len > 1e-12).then(|| core::array::from_fn(|k| w[k] / len))
        };

        // The generator itself: null, exactly.
        let omega = metric.frame_dragging_omega(rp);
        let generator = unit(core::array::from_fn(|i| v_t[i] + omega * v_p[i]))
            .expect("the horizon generator is not the zero vector");
        println!(
            "on r+ = {rp:.6}: d/dt maps to {v_t:?}, d/dphi to {v_p:?}, and the generator at \
             Omega+ = {omega:.6} to {generator:?} with eta = {:.3e}",
            eta(generator)
        );
        assert!(
            eta(generator).abs() < 1e-9,
            "the horizon generator has to map to a null vector, but eta = {}",
            eta(generator)
        );

        // And nothing in that plane is timelike: every direction of it is null or spacelike, so
        // the tube is tangent to the cone rather than cutting into it.
        let mut worst = f64::INFINITY;
        let mut closest_null = f64::INFINITY;
        for i in 0..360 {
            let th = 2.0 * PI * (i as f64) / 360.0;
            let (s, c) = th.sin_cos();
            let Some(w) = unit(core::array::from_fn(|k| c * v_t[k] + s * v_p[k])) else {
                continue;
            };
            worst = worst.min(eta(w));
            closest_null = closest_null.min(eta(w).abs());
        }
        println!(
            "over 360 directions of that tangent plane the Minkowski norm never fell below \
             {worst:.3e}, and touched zero to {closest_null:.3e}"
        );
        assert!(worst > -1e-9, "a timelike direction in the tangent plane of r+: eta = {worst}");
        // The scan touches zero only to the resolution it is taken at - the null direction is the
        // minimum of a quadratic, and a degree either side of it costs (half a degree)^2 - so the
        // exact statement is the generator's own eta above, and this is the shape of the dip.
        assert!(
            worst < 1e-3,
            "the plane has to come down to the cone, but its least norm over the circle is {worst}"
        );

        // Outside the horizon the same plane is timelike: it contains a hovering observer's own
        // worldline, which is what makes r = 3 a surface a rocket can stay on.
        let (_, out) = embedded(&metric, 0.0, 3.0, phi0, &raindrop(&metric, 3.0));
        let w_t = unit(out.vector(&metric, 3.0, phi0, &[1.0, 0.0, 0.0]))
            .expect("the Killing direction is not the zero vector");
        let n = eta(w_t);
        println!("at r = 3 the Killing direction in the tube's tangent plane has eta = {n:.4}");
        assert!(n < -1e-6, "outside r+ the tube is timelike, but eta = {n}");
    }
}
