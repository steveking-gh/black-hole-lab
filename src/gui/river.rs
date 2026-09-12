use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use egui::{Pos2, Stroke};

/// Outer edge of the particle field, in units of M. Particles are seeded inside this disc and
/// respawned on it, so the river always has an upstream supply no matter how long it runs.
pub const R_MAX: f64 = 12.0;

/// Radius at which a drop is retired and respawned at `R_MAX`. It sits above
/// `geodesic::R_STOP`, so a particle is never carried into the regime where the closed-form
/// rates are being evaluated at the floor of `GeodesicState::derivatives`.
pub const R_MIN: f64 = 0.05;

/// Number of drops in the field. The per-particle cost is a handful of closed-form evaluations,
/// so this is negligible next to the rest of the frame.
const PARTICLE_COUNT: usize = 400;

/// Largest |dr| allowed in one integration substep, in units of M.
const MAX_DR_PER_SUBSTEP: f64 = 0.05;

/// Hard cap on substeps per particle per call, so a very large dt cannot stall a frame.
const MAX_SUBSTEPS: usize = 64;

/// Longest advance the field accepts in one call, in M of coordinate time. See `advance`.
const MAX_ADVANCE_T: f64 = 1.0;

/// Coordinate time over which a freshly spawned drop fades up to full opacity, so respawns at
/// `R_MAX` do not pop into view.
const FADE_IN_T: f64 = 1.0;

/// Proper diameter, in units of M, of a drop at the radius where it enters the field. A drop is a
/// circle of this diameter at `R_MAX` and nothing but the flow deforms it after that, so this is
/// the only size choice in the drawing, and it is a drawing choice: at the default 48 px/M a fresh
/// drop is 4.8 px across. The release spacings that produce it come from `drop_release_spacings`.
const DROP_DIAMETER_AT_SPAWN: f64 = 0.1;

/// Number of vertices in the polygon that stands in for each drop's ellipse.
const DROP_VERTICES: usize = 16;

/// Cap on the drawn semi-major axis, in pixels. When it binds, both axes are scaled by the same
/// factor, so the drop shrinks without changing shape.
const DROP_SEMI_MAJOR_MAX_PX: f32 = 40.0;

/// Floor on the drawn semi-minor axis, in pixels. A floor necessarily breaks the aspect ratio, but
/// it binds only once the width has dropped below a pixel, which happens deep inside where the
/// element really is that thin.
const DROP_SEMI_MINOR_MIN_PX: f32 = 0.6;

/// Coordinate velocity (dr/dt, dphi/dt) of the river at radius r: the E = 1, L = 0 ingoing
/// geodesic through that radius, which is the raindrop congruence the Painleve-Gullstrand and
/// Doran river pictures describe as "space flowing inward".
///
/// The rates come from the exact closed form `GeodesicState::derivatives`, which is regular
/// across both horizons, divided through by dt/dtau. For a = 0 the angular rate is identically
/// zero and the flow is purely radial; for a != 0 it is not, and the river spirals. That twist is
/// frame dragging, and it is here because the geodesic puts it here, not because it was added.
///
/// One sign to be aware of: for a > 0 the chart rate dphi/dt returned here is negative, because
/// the ingoing Kerr-Schild azimuth is twisted against Boyer-Lindquist by dphi_KS = dphi_BL +
/// (a/Delta) dr. The angle that is drawn is the polar angle psi = phi + atan2(a, r) of the
/// embedding, and dpsi/dt does carry the sign of the spin in every region, so the swirl on screen
/// is prograde. The tests pin both statements.
#[allow(dead_code)] // the field caches the congruence; this is the single-radius form the tests use
pub fn river_rates(metric: &KerrSchild, r: f64) -> (f64, f64) {
    rates_of(&raindrop_congruence(metric), metric, r)
}

/// The raindrop congruence as a geodesic state. `derivatives` reads only (E, L) off the state and
/// takes the radius as an argument, so one instance serves the whole field.
fn raindrop_congruence(metric: &KerrSchild) -> GeodesicState {
    GeodesicState::new_infall(metric, 0.0, R_MAX, 1.0, 0.0)
}

/// `river_rates` with the congruence already in hand.
fn rates_of(raindrop: &GeodesicState, metric: &KerrSchild, r: f64) -> (f64, f64) {
    let (dt_dtau, dr_dtau, dphi_dtau) = raindrop.derivatives(metric, r);
    // dt/dtau > 0 for every future-directed timelike worldline in this chart; the floor only
    // guards against a degenerate evaluation, it never binds on the raindrop.
    let inv = 1.0 / dt_dtau.max(1e-9);
    (dr_dtau * inv, dphi_dtau * inv)
}

/// Squared proper length of a chart displacement xi at radius r, measured in the rest frame of an
/// observer with 4-velocity u:
///     xi_perp = xi + (u . xi) u,   ell^2 = g(xi_perp, xi_perp) = g(xi, xi) + (u . xi)^2,
/// the cross terms collapsing because g(u, u) = -1. Only the part of xi orthogonal to u is a
/// length for that observer; the part along u is a time offset, and subtracting it is exactly what
/// turns a chart separation into a ruler reading.
fn proper_length_sq(metric: &KerrSchild, r: f64, u: &[f64; 3], xi: &[f64; 3]) -> f64 {
    let g = metric.metric_components(r);
    let mut g_xi_xi = 0.0;
    let mut u_dot_xi = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            g_xi_xi += g[i][j] * xi[i] * xi[j];
            u_dot_xi += g[i][j] * u[i] * xi[j];
        }
    }
    g_xi_xi + u_dot_xi * u_dot_xi
}

/// Proper length of a drop along the flow: the separation of two raindrops of energy `energy`
/// released `dt` of coordinate time apart, as the drop itself measures that separation.
///
/// On a slice of constant t the trailing raindrop sits at
///     xi^mu = (u^mu / u^t - delta^mu_t) dt = (0, dr/dt, dphi/dt) dt,
/// which is not orthogonal to u, so the chart norm of xi is not a ruler reading. The projection
/// `proper_length_sq` performs is, and evaluating it here from `metric_components` and the exact
/// 4-velocity of `GeodesicState::derivatives` is the general answer.
///
/// Writing that projection out shows it collapses. With u_t = -E and g(u, u) = -1,
///     g(xi, xi) = [ g(u,u)/(u^t)^2 - 2 u_t/u^t + g_tt ] dt^2 = [ -1/(u^t)^2 + 2E/u^t + g_tt ] dt^2,
///     u . xi    = [ g(u,u)/u^t - u_t ] dt                    = [ -1/u^t + E ] dt,
/// so every u^t term (and with it every g_tr and g_rphi term, which enter only through u_t and
/// g(u, u)) cancels:
///     ell^2 = (g_tt + E^2) dt^2 = (E^2 - 1 + 2M/r) dt^2.
/// `proper_drop_length_closed_form` uses that result, and this projection is the independent form
/// the tests check it against.
#[cfg(test)]
pub fn proper_drop_length(metric: &KerrSchild, r: f64, energy: f64, dt: f64) -> f64 {
    let state = GeodesicState::new_infall(metric, 0.0, r, energy, 0.0);
    let (ut, ur, uphi) = state.derivatives(metric, r);
    let u = [ut, ur, uphi];
    let xi = [0.0, dt * ur / ut, dt * uphi / ut];
    proper_length_sq(metric, r, &u, &xi).max(0.0).sqrt()
}

/// The closed form of `proper_drop_length`:
///     ell = sqrt(E^2 - 1 + 2M/r) dt,
/// which at E = 1 is `KerrSchild::doran_river_speed` times dt, i.e. sqrt(2M/r) dt. This is the
/// cheap form the drawing uses: no 4-velocity, no metric inverse, one square root per drop.
///
/// The radicand is negative only inside the turning point of a bound raindrop (E < 1 and
/// r < 2M/(1 - E^2)), where no geodesic with that energy reaches, so the floor never binds on a
/// congruence that exists; at the turning point itself ell = 0, as it must be, because the two
/// release events then sit at the same radius.
pub fn proper_drop_length_closed_form(metric: &KerrSchild, r: f64, energy: f64, dt: f64) -> f64 {
    let doran = metric.doran_river_speed(r);
    (energy * energy - 1.0 + doran * doran).max(0.0).sqrt() * dt
}

/// Proper width of a drop across the flow: the transverse gap between two raindrops of the E = 1,
/// L = 0 congruence at the same (t, r) whose azimuths differ by `dphi`, as the drop measures it.
///
/// The separation is xi^mu = (0, 0, dphi), so `proper_length_sq` gives
///     w^2 = g_phiphi dphi^2 + (u . xi)^2 = g_phiphi dphi^2 + (u_phi dphi)^2,
/// and the raindrop carries u_phi = L = 0, so the cross term vanishes identically and
/// w = sqrt(g_phiphi) dphi exactly. The projection is evaluated anyway, so the coded metric alone
/// decides the answer.
///
/// On the equator g_phiphi = r^2 + a^2 + 2 M a^2 / r. For a = 0 the width falls monotonically to
/// zero at the singularity. For a != 0 it turns around at r = (M a^2)^(1/3), which lies between
/// r- and r+, and then grows without bound as r -> 0: the ring singularity is a circle of infinite
/// proper circumference, so the flow lines fan apart again on the way in.
pub fn proper_drop_width(metric: &KerrSchild, r: f64, dphi: f64) -> f64 {
    let raindrop = raindrop_congruence(metric);
    let (ut, ur, uphi) = raindrop.derivatives(metric, r);
    let u = [ut, ur, uphi];
    let xi = [0.0, 0.0, dphi];
    proper_length_sq(metric, r, &u, &xi).max(0.0).sqrt()
}

/// Release spacings of the two raindrop pairs that bound a drop: `dt` of coordinate time between
/// the raindrop at its head and the one at its tail, and `dphi` of azimuth between the two flow
/// lines that bound it sideways.
///
/// Both are derived from `DROP_DIAMETER_AT_SPAWN` rather than chosen by hand, which defines the
/// element as a circle of that proper diameter D at the radius R_MAX where it enters the field:
///     dt   = D / sqrt(2M/R_MAX)        so  length(R_MAX) = sqrt(2M/R_MAX) dt        = D,
///     dphi = D / sqrt(g_phiphi(R_MAX)) so  width(R_MAX)  = sqrt(g_phiphi(R_MAX)) dphi = D.
/// Both depend on the metric, g_phiphi through a, so they are computed per draw rather than
/// written down as constants.
///
/// Thereafter the flow alone deforms that circle. The length scales as the ratio of Doran speeds,
///     length(r) / D = sqrt(R_MAX / r),
/// the tidal stretching along the flow of an element released from rest at infinity, and the width
/// as
///     width(r) / D = sqrt(g_phiphi(r) / g_phiphi(R_MAX)),
/// the proper convergence of neighbouring flow lines. The drawn aspect ratio length/width is
/// therefore the spaghettification factor of the fluid element itself, which is (R_MAX/r)^(3/2)
/// wherever a is negligible. Particles seeded inside R_MAX at start-up obey the same rule, both
/// sizes being functions of r alone, which is consistent with their having come from R_MAX.
fn drop_release_spacings(metric: &KerrSchild) -> (f64, f64) {
    let dt = DROP_DIAMETER_AT_SPAWN / metric.doran_river_speed(R_MAX);
    let dphi = DROP_DIAMETER_AT_SPAWN / metric.metric_components(R_MAX)[2][2].sqrt();
    (dt, dphi)
}

/// One tracer of the river, carried in chart coordinates so that the advection is exact and the
/// Cartesian embedding is applied only when drawing.
#[derive(Debug, Clone, Copy)]
pub struct RiverParticle {
    /// Chart radius r.
    pub r: f64,
    /// Chart azimuth phi (ingoing Kerr-Schild, the same angle the observers use).
    pub phi: f64,
    /// Coordinate time since this particle was spawned, used only for the opacity fade-in.
    pub age: f64,
}

/// xorshift64 pseudo-random generator. Seeding is fixed, so the field is identical from run to
/// run; nothing here needs cryptographic or statistical quality, only an even scatter.
#[derive(Debug, Clone, Copy)]
struct XorShift64(u64);

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform on [0, 1), from the top 53 bits.
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// The animated raindrop flow drawn on the equatorial view.
#[derive(Debug, Clone)]
pub struct RiverField {
    pub particles: Vec<RiverParticle>,
    rng: XorShift64,
}

impl Default for RiverField {
    fn default() -> Self {
        let mut rng = XorShift64::new(0x5EED_1E55_C0FF_EE01);
        let particles = (0..PARTICLE_COUNT)
            .map(|_| {
                // Uniform in area over the disc: r = R_MAX sqrt(u). The field starts fully faded
                // in, since these particles are not respawns but the initial state of the river.
                let r = (R_MAX * rng.next_f64().sqrt()).max(R_MIN);
                RiverParticle {
                    r,
                    phi: 2.0 * std::f64::consts::PI * rng.next_f64(),
                    age: FADE_IN_T,
                }
            })
            .collect();
        Self { particles, rng }
    }
}

impl RiverField {
    /// Advance every particle by coordinate time dt along the river.
    ///
    /// The integration is midpoint RK2 in the chart coordinates (r, phi) with the exact
    /// coordinate rates of `river_rates`, substepped so no substep moves a particle more than
    /// `MAX_DR_PER_SUBSTEP`. Integrating in the chart rather than in the Cartesian image is what
    /// keeps the flow exact: the embedding x + i y = (r + i a) e^{i phi} is applied only to draw.
    ///
    /// dr/dt < 0 everywhere on this congruence, so a particle only ever moves inward; one that
    /// reaches `R_MIN` is respawned at `R_MAX` on a fresh random azimuth with age zero, and then
    /// carried a random fraction of the same step along the flow so that respawns never line up.
    ///
    /// The step is capped at `MAX_ADVANCE_T`. The congruence is stationary, so the field after a
    /// long jump is statistically the same picture as after a short one, and a jump of hundreds
    /// of M (the distance-stepping mode asks for that once Bob has stopped) would only sweep every
    /// particle past the ring in one call and respawn all of them on the outer edge together,
    /// which is a ring of drops that has nothing to do with the flow.
    pub fn advance(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let dt = dt.min(MAX_ADVANCE_T);
        let raindrop = raindrop_congruence(metric);
        for i in 0..self.particles.len() {
            let mut p = self.particles[i];
            let reached_ring = Self::carry(&raindrop, metric, &mut p, dt);
            p.age += dt;

            if reached_ring {
                p = self.spawn_at_edge();
                let head_start = self.rng.next_f64() * dt;
                Self::carry(&raindrop, metric, &mut p, head_start);
            } else {
                p.r = p.r.min(R_MAX);
                p.phi = p.phi.rem_euclid(2.0 * std::f64::consts::PI);
            }
            self.particles[i] = p;
        }
    }

    /// Carry one particle dt along the flow. Returns true if it reached `R_MIN` on the way.
    fn carry(raindrop: &GeodesicState, metric: &KerrSchild, p: &mut RiverParticle, dt: f64) -> bool {
        if dt <= 0.0 {
            return false;
        }
        // Substep count from the local radial rate, so the strong field near the ring gets the
        // resolution and the weak field does not pay for it.
        let (dr_dt0, _) = rates_of(raindrop, metric, p.r);
        let spans = ((dr_dt0.abs() * dt) / MAX_DR_PER_SUBSTEP).ceil();
        let n = (spans.max(1.0) as usize).min(MAX_SUBSTEPS);
        let h = dt / n as f64;

        for _ in 0..n {
            let (k1_r, k1_phi) = rates_of(raindrop, metric, p.r);
            let r_mid = (p.r + 0.5 * h * k1_r).max(R_MIN);
            let (k2_r, k2_phi) = rates_of(raindrop, metric, r_mid);
            p.r += h * k2_r;
            p.phi += h * 0.5 * (k1_phi + k2_phi);
            if p.r <= R_MIN {
                return true;
            }
        }
        false
    }

    /// A replacement particle entering the field at its outer edge on a fresh azimuth.
    fn spawn_at_edge(&mut self) -> RiverParticle {
        RiverParticle {
            r: R_MAX,
            phi: 2.0 * std::f64::consts::PI * self.rng.next_f64(),
            age: 0.0,
        }
    }

    /// Draw the field: one fluid element of the river per particle, an ellipse carrying the
    /// element's own proper dimensions.
    ///
    /// The long axis is `proper_drop_length_closed_form`, the separation of two raindrops released
    /// dt of coordinate time apart as those raindrops measure it, which at E = 1 is the Doran river
    /// speed sqrt(2M/r) times dt. The short axis is `proper_drop_width`, the proper gap between two
    /// flow lines dphi apart in azimuth. Both are ruler readings in the drop's rest frame, taken
    /// from the coded metric, so the stretching along the flow and the thinning across it are the
    /// tidal deformation of the element itself.
    ///
    /// `drop_release_spacings` sets dt and dphi so that both axes come out at
    /// `DROP_DIAMETER_AT_SPAWN` at R_MAX: every element enters the field as a circle, and whatever
    /// shape it carries further in was put there by the flow.
    ///
    /// The colour stays keyed to `KerrSchild::river_speed`, the flow's speed against the local
    /// ZAMO, which reaches c at r+ rather than at 2M: the two river speeds are therefore both on
    /// screen, one as shape and one as hue.
    pub fn draw<F: Fn((f64, f64)) -> Pos2>(
        &self,
        painter: &egui::Painter,
        metric: &KerrSchild,
        to_screen: &F,
        px_per_m: f32,
    ) {
        let raindrop = raindrop_congruence(metric);
        let (drop_dt, drop_dphi) = drop_release_spacings(metric);
        for p in &self.particles {
            let fade = (p.age / FADE_IN_T).clamp(0.0, 1.0);
            let alpha = (Theme::RIVER_ALPHA as f64 * fade).round() as u8;
            if alpha == 0 {
                continue;
            }
            let colour = Theme::river_colour(metric.river_speed(p.r), alpha);

            let (dr_dt, dphi_dt) = rates_of(&raindrop, metric, p.r);
            let head = metric.cartesian_position(p.r, p.phi);
            let (vx, vy) = metric.cartesian_velocity(p.r, p.phi, dr_dt, dphi_dt);

            // `to_screen` is affine, so mapping the head and the head displaced by the Cartesian
            // velocity and differencing gives the drawn direction of the flow, y flip included.
            let centre = to_screen(head);
            let along = to_screen((head.0 + vx, head.1 + vy)) - centre;
            let along = if along.length() > 1e-9 {
                along.normalized()
            } else {
                egui::vec2(1.0, 0.0)
            };
            let across = egui::vec2(-along.y, along.x);

            // Proper sizes first, pixels second. The limits are legibility bounds on the drawing
            // alone: no integration, rate or invariant anywhere else sees them.
            let length = proper_drop_length_closed_form(metric, p.r, raindrop.energy, drop_dt);
            let width = proper_drop_width(metric, p.r, drop_dphi);
            let mut semi_major = (0.5 * length) as f32 * px_per_m;
            let mut semi_minor = (0.5 * width) as f32 * px_per_m;
            // An oversized drop is scaled on both axes at once, so the aspect ratio, which is the
            // physics on show here, survives the cap.
            if semi_major > DROP_SEMI_MAJOR_MAX_PX {
                let shrink = DROP_SEMI_MAJOR_MAX_PX / semi_major;
                semi_major *= shrink;
                semi_minor *= shrink;
            }
            // The floor on the minor axis is the one place the ratio is broken, and it binds only
            // where the width has gone sub-pixel, deep inside where the element really is that
            // thin. Nothing needs a floor on the major axis: a drop is 4.8 px across at spawn at
            // the default 48 px/M, and it only grows on the way in.
            let semi_minor = semi_minor.max(DROP_SEMI_MINOR_MIN_PX);

            let points: Vec<Pos2> = (0..DROP_VERTICES)
                .map(|k| {
                    let theta = k as f32 * std::f32::consts::TAU / DROP_VERTICES as f32;
                    centre + along * (semi_major * theta.cos()) + across * (semi_minor * theta.sin())
                })
                .collect();
            painter.add(egui::Shape::convex_polygon(points, colour, Stroke::NONE));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_river_rates_are_ingoing_everywhere() {
        // The raindrop congruence falls in at every radius and in every region, so the advection
        // can only ever carry a particle towards the ring.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            for &r in &[R_MAX, 6.0, 3.0, metric.outer_horizon(), 1.0, 0.3, R_MIN] {
                let (dr_dt, dphi_dt) = river_rates(&metric, r);
                assert!(dr_dt < 0.0, "dr/dt = {dr_dt} at r={r} (a={a}) must be ingoing");
                // dr/dt is a coordinate rate, not a speed: in this chart light itself runs at
                // dr/dt <= -1 inward. The physical statement is that the river is timelike, so it
                // must lie strictly inside the null wedge.
                let wedge = metric.null_wedge(r);
                assert!(
                    dr_dt > wedge.dr_dt_in && dr_dt < wedge.dr_dt_out,
                    "dr/dt = {dr_dt} left the null wedge [{}, {}] at r={r} (a={a})",
                    wedge.dr_dt_in,
                    wedge.dr_dt_out
                );
                assert!(dphi_dt.is_finite(), "dphi/dt = {dphi_dt} at r={r} (a={a})");
            }
        }
    }

    #[test]
    fn test_river_angular_rate_vanishes_without_spin_and_twists_with_it() {
        // a = 0: the geometry is spherically symmetric, so an L = 0 raindrop falls on a fixed
        // azimuth and dphi/dt is identically zero.
        let schwarzschild = KerrSchild::new(1.0, 0.0);
        for &r in &[10.0, 3.0, 2.0, 0.5, 0.1] {
            let (_, dphi_dt) = river_rates(&schwarzschild, r);
            assert_eq!(dphi_dt, 0.0, "dphi/dt = {dphi_dt} at r={r} without spin");
        }

        // a != 0: the rate is non-zero, and the *visible* swirl carries the sign of the spin.
        //
        // The chart rate dphi_KS/dt is negative for a > 0, which is not a contradiction: the
        // ingoing Kerr-Schild azimuth is twisted against Boyer-Lindquist by dphi_KS = dphi_BL +
        // (a/Delta) dr, and on an ingoing worldline that term over-rotates the chart. What is
        // drawn (and what is physical here) is the polar angle psi = phi + atan2(a, r) of the
        // Cartesian embedding, whose rate is
        //     dpsi/dtau = (a / r^2) S (S - 2Mr) / [Delta (r^2 + a^2)],   S = sqrt(2Mr(r^2+a^2)),
        // and S - 2Mr has the sign of Delta, so dpsi/dt carries the sign of a in every region.
        for &a in &[0.65, -0.65] {
            let metric = KerrSchild::new(1.0, a);
            let (dr_dt, dphi_dt) = river_rates(&metric, 3.0);
            assert!(dphi_dt.abs() > 1e-6, "dphi/dt = {dphi_dt} at r=3M (a={a}) must not vanish");

            // rho^2 dpsi/dt = x vy - y vx, straight from the drawn Cartesian velocity.
            let phi = 0.9;
            let (x, y) = metric.cartesian_position(3.0, phi);
            let (vx, vy) = metric.cartesian_velocity(3.0, phi, dr_dt, dphi_dt);
            let swirl = x * vy - y * vx;
            assert!(swirl * a > 0.0, "swirl = {swirl} must carry the sign of a = {a}");
        }
    }

    #[test]
    fn test_advance_keeps_every_particle_inside_the_field() {
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let mut field = RiverField::default();
            // 150 steps of 0.25 M is 37.5 M of coordinate time, comfortably more than the ~25 M a
            // raindrop needs to run from R_MAX to R_MIN, so every particle respawns at least once.
            for _ in 0..150 {
                field.advance(&metric, 0.25);
                for p in &field.particles {
                    assert!(
                        p.r >= R_MIN && p.r <= R_MAX,
                        "r = {} escaped [{R_MIN}, {R_MAX}] (a={a})",
                        p.r
                    );
                    assert!(p.r.is_finite() && p.phi.is_finite(), "non-finite particle (a={a})");
                }
            }
        }
    }

    #[test]
    fn test_advance_never_moves_the_azimuth_without_spin() {
        // With a = 0 every dphi/dt evaluated by the integrator is exactly zero, so the recorded
        // azimuths must be bit-for-bit the ones the field was seeded with.
        let metric = KerrSchild::new(1.0, 0.0);
        let mut field = RiverField::default();
        let mut moved = 0usize;
        for _ in 0..150 {
            let before: Vec<(f64, f64)> =
                field.particles.iter().map(|p| (p.phi, p.age)).collect();
            field.advance(&metric, 0.25);
            for (p, &(phi0, age0)) in field.particles.iter().zip(before.iter()) {
                // A respawn draws a fresh azimuth and resets the age, so only particles whose age
                // grew are the same particle as before the step.
                if p.age > age0 {
                    assert_eq!(p.phi, phi0, "phi moved without spin: {} vs {phi0}", p.phi);
                    moved += 1;
                }
            }
        }
        assert!(moved > 1000, "the test must actually have advanced particles, got {moved}");
    }

    #[test]
    fn test_proper_drop_length_projection_matches_the_closed_form() {
        // The projection carries u^t, g_tr and g_rphi explicitly; the closed form carries none of
        // them. Agreement to 1e-10 across every region is the statement that they all cancel.
        let mut worst = 0.0f64;
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let (dt, _) = drop_release_spacings(&metric);
            let rp = metric.outer_horizon();
            for &energy in &[1.0, 1.3] {
                for &r in &[12.0, 6.0, 3.0, 2.0, rp, 1.0, 0.5, 0.2] {
                    let projected = proper_drop_length(&metric, r, energy, dt);
                    let closed = proper_drop_length_closed_form(&metric, r, energy, dt);
                    let d = (projected - closed).abs();
                    worst = worst.max(d);
                    assert!(
                        d < 1e-10,
                        "ell = {projected} vs {closed} at r={r} (a={a}, E={energy})"
                    );
                    assert!(projected > 0.0, "ell must be positive at r={r} (a={a}, E={energy})");
                }
            }
        }
        assert!(worst < 1e-10, "worst deviation {worst}");
    }

    #[test]
    fn test_proper_drop_length_is_the_doran_river_speed_at_unit_energy() {
        // E = 1 reduces ell to sqrt(2M/r) dt, so the drawn length reads off the Doran river speed
        // directly and hits exactly dt at the static limit r = 2M, for every spin.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let (dt, _) = drop_release_spacings(&metric);
            for &r in &[12.0, 6.0, 3.0, 2.0, 1.0, 0.2] {
                let expected = (2.0_f64 / r).sqrt() * dt;
                let got = proper_drop_length_closed_form(&metric, r, 1.0, dt);
                assert!((got - expected).abs() < 1e-12, "ell = {got} vs {expected} at r={r} (a={a})");
                assert!(
                    (got - metric.doran_river_speed(r) * dt).abs() < 1e-12,
                    "ell must be doran_river_speed * dt at r={r} (a={a})"
                );
            }
            let at_static_limit = proper_drop_length_closed_form(&metric, 2.0, 1.0, dt);
            assert!(
                (at_static_limit - dt).abs() < 1e-12,
                "ell(2M) = {at_static_limit} must be dt = {dt} (a={a})"
            );
        }
    }

    #[test]
    fn test_proper_drop_width_is_the_root_of_g_phiphi() {
        // u_phi = L = 0 on the raindrop, so the projection has no cross term and the width is
        // sqrt(g_phiphi) dphi exactly, read straight off the coded metric component.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let (_, dphi) = drop_release_spacings(&metric);
            let rp = metric.outer_horizon();
            for &r in &[12.0, 6.0, 3.0, 2.0, rp, 1.0, 0.5, 0.2] {
                let g_phiphi = metric.metric_components(r)[2][2];
                let expected = g_phiphi.sqrt() * dphi;
                let got = proper_drop_width(&metric, r, dphi);
                assert!(
                    (got - expected).abs() < 1e-12 * (1.0 + expected),
                    "w = {got} vs {expected} at r={r} (a={a})"
                );
            }
        }
    }

    #[test]
    fn test_proper_drop_width_turns_around_between_the_horizons_and_diverges_at_the_ring() {
        // g_phiphi = r^2 + a^2 + 2 M a^2 / r has d/dr = 2r - 2 M a^2 / r^2, so with spin the width
        // is least at r = (M a^2)^(1/3), which lies inside r+ and outside r-, and grows again as
        // r -> 0 because g_phiphi ~ 2 M a^2 / r there: the ring has infinite proper circumference.
        for &a in &[0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let (_, dphi) = drop_release_spacings(&metric);
            let r_min_width = (metric.m * a * a).cbrt();
            assert!(
                r_min_width > metric.inner_horizon() && r_min_width < metric.outer_horizon(),
                "the width minimum at r={r_min_width} must sit between the horizons (a={a})"
            );
            let w_min = proper_drop_width(&metric, r_min_width, dphi);
            for &r in &[12.0, 3.0, 2.0, 1.5 * r_min_width, 0.5 * r_min_width, 0.05, 0.01] {
                assert!(
                    proper_drop_width(&metric, r, dphi) > w_min,
                    "w({r}) = {} must exceed the minimum {w_min} (a={a})",
                    proper_drop_width(&metric, r, dphi)
                );
            }
            // Monotone growth on the way in from the turning point to the ring.
            let mut previous = w_min;
            for &r in &[0.5 * r_min_width, 0.1 * r_min_width, 0.01 * r_min_width] {
                let w = proper_drop_width(&metric, r, dphi);
                assert!(w > previous, "w({r}) = {w} must exceed w at the larger radius {previous}");
                previous = w;
            }
        }

        // Without spin g_phiphi = r^2, so the width falls monotonically to zero instead.
        let schwarzschild = KerrSchild::new(1.0, 0.0);
        let (_, dphi) = drop_release_spacings(&schwarzschild);
        let mut previous = f64::INFINITY;
        for &r in &[12.0, 6.0, 2.0, 0.5, 0.05] {
            let w = proper_drop_width(&schwarzschild, r, dphi);
            assert!(w < previous, "w({r}) = {w} must fall without spin");
            previous = w;
        }
    }

    #[test]
    fn test_drops_are_circles_where_they_enter_the_field() {
        // The spacings are derived so that the two axes coincide at R_MAX, for every spin: a drop
        // enters the field round, and every departure from round further in is the flow's doing.
        for &a in &[0.0, 0.65, 0.95] {
            let metric = KerrSchild::new(1.0, a);
            let (dt, dphi) = drop_release_spacings(&metric);
            let length = proper_drop_length_closed_form(&metric, R_MAX, 1.0, dt);
            let width = proper_drop_width(&metric, R_MAX, dphi);
            assert!(
                (length - DROP_DIAMETER_AT_SPAWN).abs() < 1e-12,
                "length(R_MAX) = {length} must be the spawn diameter (a={a})"
            );
            assert!(
                (width - DROP_DIAMETER_AT_SPAWN).abs() < 1e-12,
                "width(R_MAX) = {width} must be the spawn diameter (a={a})"
            );
        }
    }

    #[test]
    fn test_drop_aspect_ratio_is_the_tidal_stretch() {
        // Without spin the length goes as sqrt(R_MAX/r) and the width as r/R_MAX, so the drawn
        // aspect ratio is exactly the (R_MAX/r)^(3/2) stretch of a radially infalling element.
        let schwarzschild = KerrSchild::new(1.0, 0.0);
        let (dt, dphi) = drop_release_spacings(&schwarzschild);
        for &r in &[6.0, 3.0, 2.0, 1.0, 0.5] {
            let ratio = proper_drop_length_closed_form(&schwarzschild, r, 1.0, dt)
                / proper_drop_width(&schwarzschild, r, dphi);
            let expected = (R_MAX / r).powf(1.5);
            assert!(
                (ratio - expected).abs() < 1e-12 * (1.0 + expected),
                "aspect = {ratio} vs {expected} at r={r}"
            );
        }

        // With spin g_phiphi carries a, so the ratio is no longer a pure power of r, but it still
        // grows all the way in and the element is stretched by more than a factor of ten by the
        // time it reaches the outer horizon.
        let kerr = KerrSchild::new(1.0, 0.65);
        let (dt, dphi) = drop_release_spacings(&kerr);
        let aspect = |r: f64| {
            proper_drop_length_closed_form(&kerr, r, 1.0, dt) / proper_drop_width(&kerr, r, dphi)
        };
        let rp = kerr.outer_horizon();
        assert!(aspect(rp) > 10.0, "aspect(r+) = {} must exceed 10", aspect(rp));
        let mut previous = aspect(R_MAX);
        assert!((previous - 1.0).abs() < 1e-12, "aspect(R_MAX) = {previous} must be 1");
        for &r in &[9.0, 6.0, 4.0, 3.0, 2.0, rp] {
            let ratio = aspect(r);
            assert!(ratio > previous, "aspect({r}) = {ratio} must exceed {previous} further out");
            previous = ratio;
        }
    }

    #[test]
    fn test_a_huge_step_does_not_respawn_the_field_as_a_ring() {
        // Distance stepping can ask for hundreds of M once Bob has stopped. The field must still
        // look like a stationary sample of the flow afterwards, not a ring of drops on the edge.
        let metric = KerrSchild::new(1.0, 0.65);
        let mut field = RiverField::default();
        for _ in 0..40 {
            field.advance(&metric, 500.0);
            let on_the_edge = field.particles.iter().filter(|p| p.r > 0.95 * R_MAX).count();
            let deep = field.particles.iter().filter(|p| p.r < 0.5 * R_MAX).count();
            assert!(
                on_the_edge < PARTICLE_COUNT / 4,
                "{on_the_edge} of {PARTICLE_COUNT} drops piled on the outer edge"
            );
            assert!(deep > PARTICLE_COUNT / 10, "only {deep} drops inside half the field radius");
        }
    }

    #[test]
    fn test_advance_is_a_no_op_when_paused() {
        let metric = KerrSchild::new(1.0, 0.65);
        let mut field = RiverField::default();
        let before: Vec<(f64, f64)> = field.particles.iter().map(|p| (p.r, p.phi)).collect();
        field.advance(&metric, 0.0);
        for (p, &(r, phi)) in field.particles.iter().zip(before.iter()) {
            assert_eq!((p.r, p.phi), (r, phi), "a paused river must not move");
        }
    }
}

