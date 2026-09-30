//! The direction of travel: which way, on the observer's own sky, the observer is going, and how
//! fast, past each local reference observer that exists at the event.
//!
//! # Past whom
//!
//! "Travelling" means nothing without someone to travel past. The app quotes an observer's speed
//! past every local reference observer that exists at the event (`local_speeds` in the app's
//! `src/physics/observer.rs`), and this program keeps to the same rule:
//!
//! * the **static observer**, at rest with respect to the distant stars, u_F = d_t / sqrt(-g_tt),
//!   which exists where d_t is timelike: outside the static limit, r > 2M on the plane;
//! * the **ZAMO**, the observer of zero angular momentum, holding r and carried round by the frame
//!   dragging, u_F = (d_t + omega d_phi) / alpha, which exists outside r+;
//! * and, only where neither exists (r <= r+), the **raindrop**, the E = 1, L = 0 ingoing
//!   geodesic, which exists at every r > 0.
//!
//! Outside the static limit both hovering observers are quoted, not one of them: they disagree
//! by a great deal where the dragging is strong, and the disagreement is the dragging. Nothing is
//! ever quoted against a worldline that does not exist at the event.
//!
//! # What is quoted
//!
//! For a reference observer with 4-velocity u_F and the observer's own u, gamma = -g(u, u_F) >= 1
//! and the speed is v = sqrt(1 - 1 / gamma^2). The part of u_F in the observer's rest space is
//! w = u_F - gamma u, of length gamma v: the reference observer streams past the observer along w,
//! so the observer travels past the reference observer along -w. As a direction on the
//! observer's sky, written in the film's triad (x toward the hole, y the viewer's left, z the spin
//! axis), that is the unit vector n the mark carries. Everything lies in the equatorial plane, so
//! n_z = 0 and the mark sits on the frame's equator; the heading is the angle to the viewer's
//! right of the opening view, atan2(-n_y, n_x), in degrees in (-180, 180].
//!
//! # The hovering observers, in closed form
//!
//! Formed as it stands, u_F - gamma u subtracts numbers of the size of gamma^2 in the chart, and
//! projecting it on legs of size gamma does the same again: at gamma = 1e4 that is every digit of
//! a direction that ought to be known to thirteen. The static observer and the ZAMO exist only
//! outside r+, where the observer's velocity relative to each has a form with no such cancellation
//! in it.
//!
//! Three of the observer's numbers do not depend on the chart: u^r = dr/dtau, and the covariant
//! components u_t = -E and u_phi = L, since d_t and d_phi are the same Killing vectors in
//! Boyer-Lindquist and ingoing Kerr-Schild coordinates (the two differ by functions of r alone
//! added to t and phi). Both reference observers' frames have the same radial leg, e_r = nabla r /
//! |nabla r|, with |nabla r|^2 = g^rr = Delta / r^2 on the plane, since nabla r is orthogonal to
//! both Killing vectors; so the observer's radial celerity is the same past both,
//!
//!     u^(r) = g(u, e_r) = u^r / |nabla r| = r u^r / sqrt(Delta).
//!
//! The azimuthal leg of the ZAMO is d_phi / sqrt(g_phiphi), and of the static observer the unit
//! vector of the (t, phi) plane orthogonal to d_t, e_psi = (g_tphi d_t - g_tt d_phi) / sqrt(-g_tt
//! Delta), whose norm follows from g_tt g_phiphi - g_tphi^2 = -Delta on the plane. So
//!
//!     ZAMO:    u^(az) = L / sqrt(g_phiphi),
//!     static:  u^(az) = (g_tphi u_t - g_tt u_phi) / sqrt(-g_tt Delta)
//!                     = (Delta u^phi - a u^r) / sqrt(-g_tt Delta).
//!
//! The second form of the static one comes from writing u_t and u_phi out in the chart's
//! components: the u^t terms cancel identically, g_tphi^2 - g_tt g_phiphi = Delta multiplies u^phi,
//! and in this chart g_tphi g_tr - g_tt g_rphi = -4 h^2 a - a (1 - 4 h^2) = -a (h = M / r) multiplies
//! u^r. It has no u^t in it at all, and is exactly zero for the static observer itself.
//!
//! The celerity past the reference observer is gv = gamma v = sqrt(u^(r)^2 + u^(az)^2), a sum of
//! squares, and gamma = sqrt(1 + gv^2).
//!
//! The direction in the triad follows from the triad's own definition. x is minus the rest-space
//! part of nabla r, P nabla r = nabla r + u^r u, of length N / r with N^2 = Delta + r^2 (u^r)^2; y is
//! the unit rest-space vector along r = const on the retrograde side, which with g(y, u) = 0 and
//! y^r = 0 is -(L d_t + E d_phi) / N (its norm from the same identity, and N = r |P nabla r|). With
//! -w = (gamma u - u_F), g(-w, nabla r) = gamma u^r (u_F^r = 0) and g(-w, y) = g(u_F, L d_t + E
//! d_phi) / N = (E L_F - L E_F) / N. Divided by gv, and written with u^(r) and u^(az), both come to
//!
//!     n = -(gamma u^(r), u^(az)) / (sqrt(1 + u^(r)^2) gv),
//!
//! a unit vector (the check is one line of algebra: gamma^2 u^(r)^2 + u^(az)^2 = (1 + u^(r)^2) gv^2).
//! The radial part carries a factor gamma / sqrt(1 + u^(r)^2) that the reference observer's own
//! picture does not have: the observer's "toward the hole" is not the reference observer's, and
//! the travel is seen aberrated against it. Inward travel is +x, prograde travel is -y, the
//! viewer's right: a positive heading.
//!
//! Every quantity in it is a product, a quotient or a sum of squares of u^r, L (or u^phi) and
//! metric functions, except in three places, each handled:
//!
//! * Delta = r^2 - 2Mr + a^2 near r+, where it is small and its terms are not. It is evaluated
//!   with error-free products and sums ([`delta`]), so that it is good to its last bits at any
//!   distance from r+ that an f64 radius can have: the radius is the given number, and nothing
//!   about it is lost.
//! * -g_tt = 1 - 2M/r near the static limit, taken as (r - 2M) / r, whose subtraction is exact near
//!   r = 2M (Sterbenz: r - 2M is exact for r in [M, 4M]).
//! * L = g_phit u^t + g_phir u^r + g_phiphi u^phi for the ZAMO, and Delta u^phi - a u^r for the
//!   static observer, which cancel where the observer barely moves past the reference observer
//!   (a ZAMO against the ZAMO: L is a difference of two terms of size u^t). Nothing removes that
//!   cancellation - it is the observer's own u that is known only to its rounding - so each is
//!   carried with a bound on its rounding error, a few units of rounding of the sum of the terms'
//!   sizes.
//!
//! # The raindrop
//!
//! Between the horizons only the raindrop exists, and there N^2 = Delta + r^2 (u^r)^2 is itself a
//! difference. `kerr_sky::Triad` already holds the observer as a pure boost of the raindrop's
//! frame, which exists everywhere and has components of order one: gamma and the direction of the
//! boost come from three dot products of u with legs of order one, and the direction of travel
//! past the raindrop is `Triad::forward`, the boost's direction carried into the triad by the
//! exact turn that `Triad::look` builds the sky with. Nothing in it cancels at any boost, and it is
//! the same frame the picture is drawn in, so the mark sits where the picture says the motion is.
//! `Triad::celerity` gives its rounding bound.
//!
//! # At rest, and a direction not known
//!
//! The observer is at rest relative to a reference observer where gamma - 1 is below the rounding
//! of gamma itself, eps: the invariant the app quotes every speed from cannot tell such a motion
//! from none. With gamma - 1 = celerity^2 / 2 to first order, that is a celerity below
//! sqrt(2 eps) = 2.1e-8 ([`rest`]). It is also the size of what an observer released at rest
//! carries at the release: its u^r there is -sqrt(R) / r^2 at a zero of R, and R, a difference of
//! terms of order one, is known only to a rounding, so u^r only to about sqrt(eps). Bob and Alice
//! in the demonstration save are such observers, momentarily at rest past the ZAMO (L = 0,
//! u^r = 0) and given a motion of 1.1e-8 c straight at the hole by that root, until this floor.
//!
//! The observer is at rest too where the celerity is no larger than its rounding bound, which is
//! larger than sqrt(2 eps) only where u^t is: a ZAMO against the ZAMO a hair outside r+, say.
//! Either way there is no direction - the hover test's static observer against the static
//! observer, the raindrop against itself, and the raindrop carried by the walker's integrator,
//! which strays from the closed-form raindrop by about 5e-11 c - the speed is written as 0, and no
//! mark or heading is given.
//!
//! Where the celerity is larger than both but its rounding bound is not below it by a factor of
//! 1 / DIRECTION_TOLERANCE, the direction is known to worse than 1e-4 rad (0.0057 degrees), and
//! it is left out too: a mark in a direction the program does not know is a plausible guess, and
//! the program does not draw those. The speed is still given, and four decimals show it as
//! 0.0000. That happens only close to r+, where the observer's u^t is large and holds its own
//! motion past the ZAMO less well than that: within 1e-9 M of r+, a motion of 1e-6 c past the
//! ZAMO. The tests measure every direction that is given against an independent evaluation in
//! double-double arithmetic.

use kerr_equatorial::KerrSchild;
use kerr_sky::{Kerr, Triad};
use sky_format::{MarkDecl, ReadoutDecl};

use crate::worldline::Event;

/// The smallest celerity told from rest: sqrt(2 eps), at which gamma - 1 = celerity^2 / 2 reaches
/// the rounding of gamma (module documentation).
pub fn rest() -> f64 {
    (2.0 * f64::EPSILON).sqrt()
}

/// How well a direction must be known to be given, in radians: 1e-4, or 0.0057 degrees, inside
/// the 0.01 degrees the owner asked for. A direction whose rounding bound, divided by the
/// celerity, is larger is left out.
pub const DIRECTION_TOLERANCE: f64 = 1e-4;

/// Above this speed, in c, a read-out gives the Lorentz factor instead of the speed: four decimals
/// of a speed above it would print 1.0000 and say that something reached the speed of light.
pub const FAST: f64 = 0.9999;

/// How many decimals a heading is declared with.
pub const HEADING_DECIMALS: u32 = 1;

/// Where the first heading of a film's run is cut: a heading below this, which
/// [`HEADING_DECIMALS`] decimals show as "-180.0", is written 360 degrees up, near +180.
pub fn behind_cut() -> f64 {
    -180.0 + 0.5 * 10f64.powi(-(HEADING_DECIMALS as i32))
}

/// The unit of a heading that may be of either sign.
pub const RIGHT_OF_THE_HOLE: &str = "° right of the hole";

/// The unit of a heading of one frame that is to the viewer's left.
pub const LEFT_OF_THE_HOLE: &str = "° left of the hole";

/// A local reference observer the travel is quoted against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reference {
    Static,
    Zamo,
    Raindrop,
}

impl Reference {
    /// In the order the manifest declares them: outermost first.
    pub const ALL: [Self; 3] = [Self::Static, Self::Zamo, Self::Raindrop];

    /// The part of every id that names this observer.
    pub fn id(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::Zamo => "zamo",
            Self::Raindrop => "raindrop",
        }
    }

    /// The observer as a label names it.
    pub fn name(self) -> &'static str {
        match self {
            Self::Static => "the static observer",
            Self::Zamo => "the ZAMO",
            Self::Raindrop => "the raindrop",
        }
    }

    /// The sign the renderer draws at the direction of travel past this observer.
    pub fn shape(self) -> &'static str {
        match self {
            Self::Static => "ring",
            Self::Zamo => "diamond",
            Self::Raindrop => "triangle",
        }
    }

    /// The id of the mark.
    pub fn mark_id(self) -> String {
        format!("travel_{}", self.id())
    }

    /// The id of the speed read-out.
    pub fn speed_id(self) -> String {
        format!("speed_{}", self.id())
    }

    /// The id of the Lorentz-factor read-out.
    pub fn gamma_id(self) -> String {
        format!("gamma_{}", self.id())
    }

    /// The id of the heading read-out.
    pub fn heading_id(self) -> String {
        format!("heading_{}", self.id())
    }
}

/// The observer's motion past one reference observer, at one event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Passing {
    pub reference: Reference,
    /// The Lorentz factor between the two; 1 where the observer is at rest relative to it.
    pub gamma: f64,
    /// gamma v, the length of the observer's velocity in the reference observer's frame; 0 at
    /// rest.
    pub celerity: f64,
    /// v, in c; 0 at rest.
    pub speed: f64,
    /// The unit vector n, in the triad, along which the observer travels past the reference
    /// observer; `None` at rest, or where the direction is not known to [`DIRECTION_TOLERANCE`].
    pub direction: Option<[f64; 3]>,
    /// The bound on the rounding error of `celerity`; divided by the celerity, it bounds the
    /// error of the direction in radians.
    pub rounding: f64,
}

impl Passing {
    /// At rest relative to `reference`, as far as f64 can tell.
    fn at_rest(reference: Reference) -> Self {
        Self {
            reference,
            gamma: 1.0,
            celerity: 0.0,
            speed: 0.0,
            direction: None,
            rounding: 0.0,
        }
    }

    /// The heading of travel, in degrees in (-180, 180]: 0 toward the hole, positive to the
    /// viewer's right.
    pub fn heading(&self) -> Option<f64> {
        self.direction.map(heading_of)
    }

    /// How far the heading may be from the truth, in degrees, by the rounding bound: infinite where
    /// there is no motion to have a heading.
    pub fn heading_rounding(&self) -> f64 {
        if self.celerity > 0.0 {
            (self.rounding / self.celerity).to_degrees()
        } else {
            f64::INFINITY
        }
    }

    /// Whether the read-outs give the Lorentz factor rather than the speed.
    pub fn fast(&self) -> bool {
        self.speed > FAST
    }
}

/// The heading of a direction n in the triad, in degrees in (-180, 180].
pub fn heading_of(n: [f64; 3]) -> f64 {
    let degrees = (-n[1]).atan2(n[0]).to_degrees();
    // atan2 gives -180 for a direction straight behind on the left of the seam; the interval is
    // closed at +180. Adding 0.0 turns a -0.0 into 0.0, which is neither side.
    if degrees <= -180.0 {
        180.0
    } else {
        degrees + 0.0
    }
}

/// Delta = r^2 - 2Mr + a^2, to its last bits however near a horizon r is.
///
/// Near r+ the three terms are of order r^2 and their sum is small; formed directly, the sum
/// keeps only the digits the terms do not share, and at r - r+ = 1e-12 that is four of them. Each
/// product is split exactly into its rounded value and its rounding error (the error of a*b is
/// fma(a, b, -a*b), exactly), the three rounded values are summed with the error of each addition
/// kept (Knuth's two-sum), and the errors are added in at the end, where they are small: what is
/// left is a rounding of the result, not of its terms.
pub fn delta(m: f64, a: f64, r: f64) -> f64 {
    let two_prod = |x: f64, y: f64| {
        let p = x * y;
        (p, x.mul_add(y, -p))
    };
    let two_sum = |x: f64, y: f64| {
        let s = x + y;
        let v = s - x;
        (s, (x - (s - v)) + (y - v))
    };
    let (rr, e_rr) = two_prod(r, r);
    let (mr, e_mr) = two_prod(2.0 * m, r);
    let (aa, e_aa) = two_prod(a, a);
    let (s1, e1) = two_sum(rr, -mr);
    let (s2, e2) = two_sum(s1, aa);
    s2 + ((e1 + e2) + ((e_rr - e_mr) + e_aa))
}

/// The observer's motion past each reference observer that exists at the event, outermost first:
/// the static observer and the ZAMO where each exists, the raindrop where neither does.
///
/// `triad` is the film's triad at the event (`film::triad`), whose boost of the raindrop's frame
/// gives the travel past the raindrop.
pub fn passing(kerr: &Kerr, event: &Event, triad: &Triad) -> Vec<Passing> {
    let metric = kerr.equatorial();
    let hovering: Vec<Passing> = [Reference::Static, Reference::Zamo]
        .into_iter()
        .filter_map(|reference| past_hovering(&metric, event.r, event.u, reference))
        .collect();
    if !hovering.is_empty() {
        return hovering;
    }
    vec![past_raindrop(triad)]
}

/// Whether a reference observer exists at radius r: the app's own tests (`static_four_velocity`,
/// g_tt < 0; `zamo_four_velocity`, r > r+), and Delta > 0 besides, which they imply wherever the
/// rounding of r+ does not decide it.
pub fn exists(metric: &KerrSchild, reference: Reference, r: f64) -> bool {
    let outside = delta(metric.m, metric.a, r) > 0.0;
    match reference {
        Reference::Static => {
            metric.metric_components(r)[0][0] < 0.0 && r > 2.0 * metric.m && outside
        }
        Reference::Zamo => r > metric.outer_horizon() && outside,
        Reference::Raindrop => r > 0.0,
    }
}

/// The travel past the static observer or the ZAMO, by the closed forms of the module
/// documentation, or `None` where that observer does not exist.
pub fn past_hovering(
    metric: &KerrSchild,
    r: f64,
    u: [f64; 3],
    reference: Reference,
) -> Option<Passing> {
    if !exists(metric, reference, r) {
        return None;
    }
    let (m, a) = (metric.m, metric.a);
    let eps = f64::EPSILON;
    let delta = delta(m, a, r);
    let sqrt_delta = delta.sqrt();
    // u^(r) = r u^r / sqrt(Delta): products and a quotient, good to a few roundings of itself.
    let radial = r * u[1] / sqrt_delta;
    let (azimuthal, azimuthal_rounding) = match reference {
        Reference::Static => {
            // (Delta u^phi - a u^r) / sqrt(-g_tt Delta), -g_tt = (r - 2M) / r.
            let minus_g_tt = (r - 2.0 * m) / r;
            let terms = [delta * u[2], -a * u[1]];
            let norm = (minus_g_tt * delta).sqrt();
            (
                (terms[0] + terms[1]) / norm,
                4.0 * eps * (terms[0].abs() + terms[1].abs()) / norm,
            )
        }
        Reference::Zamo => {
            // L / sqrt(g_phiphi), L = g_phit u^t + g_phir u^r + g_phiphi u^phi.
            let g = metric.metric_components(r);
            let terms = [g[2][0] * u[0], g[2][1] * u[1], g[2][2] * u[2]];
            let norm = g[2][2].sqrt();
            (
                (terms[0] + terms[1] + terms[2]) / norm,
                8.0 * eps * terms.iter().map(|t| t.abs()).sum::<f64>() / norm,
            )
        }
        Reference::Raindrop => unreachable!("the raindrop is not a hovering observer"),
    };
    let celerity = radial.hypot(azimuthal);
    let rounding = azimuthal_rounding + 8.0 * eps * radial.abs();
    Some(passing_from(
        reference,
        celerity,
        rounding,
        |gamma| {
            let v = [-gamma * radial, -azimuthal];
            let length = v[0].hypot(v[1]);
            [v[0] / length, v[1] / length, 0.0]
        },
        None,
    ))
}

/// The travel past the raindrop, from the triad's boost of the raindrop's frame.
pub fn past_raindrop(triad: &Triad) -> Passing {
    let (celerity, rounding) = triad.celerity();
    let (gamma, _) = triad.boost();
    passing_from(
        Reference::Raindrop,
        celerity,
        rounding,
        |_| triad.forward(),
        Some(gamma),
    )
}

/// A passing from its celerity and the celerity's rounding bound: at rest, moving in a direction
/// not known well enough to give, or moving in the direction `direction` gives for the Lorentz
/// factor. `gamma` is the Lorentz factor where it is known on its own (the raindrop's, a dot
/// product); otherwise it is sqrt(1 + celerity^2).
fn passing_from(
    reference: Reference,
    celerity: f64,
    rounding: f64,
    direction: impl Fn(f64) -> [f64; 3],
    gamma: Option<f64>,
) -> Passing {
    // Written as a negation so that a NaN counts as at rest, with no direction, rather than as a
    // motion.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(celerity > rounding.max(rest())) {
        return Passing::at_rest(reference);
    }
    let gamma = gamma.unwrap_or_else(|| 1f64.hypot(celerity));
    let known = rounding <= DIRECTION_TOLERANCE * celerity;
    Passing {
        reference,
        gamma,
        celerity,
        speed: celerity / gamma,
        direction: known.then(|| direction(gamma)),
        rounding,
    }
}

/// The marks and read-outs of the direction of travel that a film declares, from the passings of
/// all its frames: for each reference observer that occurs, in the order static, ZAMO, raindrop,
/// its speed; its Lorentz factor, if some frame is faster than [`FAST`]; and its heading and mark,
/// if some frame has a direction.
///
/// Every row names its reference observer, the heading's as the speed's does - "Speed past the
/// ZAMO", "Heading past the ZAMO (diamond)" - so that a panel showing the rows of two observers
/// never leaves a heading to be matched to its speed by position. The shape in brackets names the
/// sign on the sky that the heading points at.
///
/// A film of one frame - a still, which is what the app's Look Around makes - declares its heading
/// as a magnitude, with the unit saying which side: "35.2° left of the hole" reads better on a
/// still than "-35.2° right of the hole". A heading that shows as 0.0 or 180.0 has no side, and is
/// declared to the right.
///
/// A film of more frames keeps one signed unit throughout, since a declaration's unit is the same
/// on every frame, and its heading is continuous from frame to frame ([`written_headings`]): a
/// renderer draws a read-out between two frames by interpolating it, so a heading that went from
/// 179.9 to -179.9 would be drawn sweeping back through 0. A film's heading may therefore leave
/// (-180, 180]: after 179.9, 181.0 is written rather than -179.0, and reads truly as "181.0° right
/// of the hole".
pub fn declarations(frames: &[Vec<Passing>]) -> (Vec<ReadoutDecl>, Vec<MarkDecl>) {
    let single = frames.len() == 1;
    let mut readouts = Vec::new();
    let mut marks = Vec::new();
    for reference in Reference::ALL {
        let passings: Vec<&Passing> = frames
            .iter()
            .flatten()
            .filter(|p| p.reference == reference)
            .collect();
        if passings.is_empty() {
            continue;
        }
        let name = reference.name();
        readouts.push(ReadoutDecl {
            id: reference.speed_id(),
            label: format!("Speed past {name}"),
            unit: "c".into(),
            decimals: 4,
            display: None,
        });
        if passings.iter().any(|p| p.fast()) {
            readouts.push(ReadoutDecl {
                id: reference.gamma_id(),
                label: format!("Lorentz factor past {name}"),
                unit: String::new(),
                decimals: 1,
                display: None,
            });
        }
        let headings: Vec<f64> = passings.iter().filter_map(|p| p.heading()).collect();
        if let Some(&first) = headings.first() {
            // The side as the still shows it, to its decimals: a heading of -1e-7 degrees is
            // straight at the hole, and one of -179.99 straight away from it, not to the left.
            let shown = (first * 10f64.powi(HEADING_DECIMALS as i32)).round();
            let behind = -180.0 * 10f64.powi(HEADING_DECIMALS as i32);
            let unit = if single && shown < 0.0 && shown > behind {
                LEFT_OF_THE_HOLE
            } else {
                RIGHT_OF_THE_HOLE
            };
            readouts.push(ReadoutDecl {
                id: reference.heading_id(),
                label: format!("Heading past {name} ({})", reference.shape()),
                unit: unit.into(),
                decimals: HEADING_DECIMALS,
                display: None,
            });
            marks.push(MarkDecl {
                id: reference.mark_id(),
                label: format!("Direction of travel past {name}"),
                shape: reference.shape().into(),
            });
        }
    }
    (readouts, marks)
}

/// One frame's read-out values and mark directions, each by id.
pub type Values = (Vec<(String, f64)>, Vec<(String, [f64; 3])>);

/// The heading each passing of each frame is written with, frame by frame and passing by passing
/// as `frames` holds them; `None` where a passing has no direction.
///
/// A heading whose magnitude is within its rounding bound ([`Passing::heading_rounding`]) of zero
/// is straight at the hole to the precision known, and is written as +0.0, never as -4e-7.
///
/// In a still the heading is a magnitude, the side being in the unit [`declarations`] chose.
///
/// In a film the headings past each reference observer are unwrapped: the first frame that has one
/// writes it in [-179.95, 180.05), and each later frame writes the value congruent to its heading
/// modulo 360 that is nearest the value written for the frame before, so that the written heading
/// is continuous and a heading sitting at 180 keeps one sign instead of changing sign with the
/// rounding. A frame at which the reference observer does not exist, or at which there is no
/// direction past it, breaks the run, and the next frame that has one starts again in that
/// interval. The marks' directions are not touched: a direction is the same whichever of its
/// congruent headings is written.
///
/// The interval of a run's first heading is chosen by what the read-out's one decimal shows
/// ([`behind_cut`], from [`HEADING_DECIMALS`]): a heading below -179.95 degrees would show as
/// "-180.0", so it is written 360 degrees up, near +180, and straight behind reads "180.0° right
/// of the hole", never "-180.0°". -179.95 itself, which one decimal shows as "-179.9", stays.
pub fn written_headings(frames: &[Vec<Passing>]) -> Vec<Vec<Option<f64>>> {
    let single = frames.len() == 1;
    let mut before: [Option<f64>; 3] = [None; 3];
    frames
        .iter()
        .map(|passings| {
            let mut now: [Option<f64>; 3] = [None; 3];
            let written = passings
                .iter()
                .map(|p| {
                    let slot = Reference::ALL
                        .iter()
                        .position(|&r| r == p.reference)
                        .expect("every reference is in ALL");
                    let mut heading = p.heading()?;
                    if heading.abs() <= p.heading_rounding() {
                        heading = 0.0;
                    }
                    let value = if single {
                        heading.abs()
                    } else if let Some(previous) = before[slot] {
                        // The turns that bring it nearest the value before; + 0.0 so that a
                        // heading of 0 after a small negative one is written +0.0.
                        heading + 360.0 * ((previous - heading) / 360.0).round() + 0.0
                    } else if heading < behind_cut() {
                        heading + 360.0
                    } else {
                        heading
                    };
                    now[slot] = Some(value);
                    Some(value)
                })
                .collect();
            before = now;
            written
        })
        .collect()
}

/// The read-out values and mark directions of one frame, by id: for each reference observer that
/// exists at the frame's event, its speed or (above [`FAST`]) its Lorentz factor, and its heading
/// and direction where it has one. `headings` is the frame's line of [`written_headings`].
pub fn values(passings: &[Passing], headings: &[Option<f64>]) -> Values {
    let mut readouts = Vec::new();
    let mut marks = Vec::new();
    for (p, heading) in passings.iter().zip(headings) {
        let r = p.reference;
        if p.fast() {
            readouts.push((r.gamma_id(), p.gamma));
        } else {
            readouts.push((r.speed_id(), p.speed));
        }
        if let (Some(n), Some(heading)) = (p.direction, heading) {
            readouts.push((r.heading_id(), *heading));
            marks.push((r.mark_id(), n));
        }
    }
    (readouts, marks)
}

#[cfg(test)]
mod dd;
#[cfg(test)]
mod tests;
