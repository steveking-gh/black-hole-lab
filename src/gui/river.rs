use crate::gui::theme::Theme;
use crate::physics::geodesic::GeodesicState;
use crate::physics::kerr_schild::KerrSchild;
use egui::{Pos2, Stroke};

/// Outer edge of the particle field, in units of M. Particles are seeded inside this disc and
/// respawned on it, so the river always has an upstream supply no matter how long it runs.
pub const R_MAX: f64 = 12.0;

/// Radius at which a streak is retired and respawned at `R_MAX`. It sits above
/// `geodesic::R_STOP`, so a particle is never carried into the regime where the closed-form
/// rates are being evaluated at the floor of `GeodesicState::derivatives`.
pub const R_MIN: f64 = 0.05;

/// Number of streaks in the field. The per-particle cost is a handful of closed-form evaluations,
/// so this is negligible next to the rest of the frame.
const PARTICLE_COUNT: usize = 400;

/// Largest |dr| allowed in one integration substep, in units of M.
const MAX_DR_PER_SUBSTEP: f64 = 0.05;

/// Hard cap on substeps per particle per call, so a very large dt cannot stall a frame.
const MAX_SUBSTEPS: usize = 64;

/// Coordinate time over which a freshly spawned streak fades up to full opacity, so respawns at
/// `R_MAX` do not pop into view.
const FADE_IN_T: f64 = 1.0;

/// Coordinate time the streak's tail is drawn back over: the tail marks where the particle was
/// (to first order) 0.35 M of t ago, so the streak length reads as the local river speed.
const STREAK_T: f64 = 0.35;

/// Drawing cap on the streak length in pixels. Purely a legibility limit for deep zoom levels;
/// it never touches the integration.
const STREAK_MAX_PX: f32 = 24.0;

/// Radius of the filled dot drawn at the head of each streak, in pixels.
const HEAD_RADIUS_PX: f32 = 1.7;

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
    /// reaches `R_MIN` is respawned at `R_MAX` on a fresh random azimuth with age zero.
    pub fn advance(&mut self, metric: &KerrSchild, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let raindrop = raindrop_congruence(metric);
        for i in 0..self.particles.len() {
            let mut p = self.particles[i];

            // Substep count from the local radial rate, so the strong field near the ring gets
            // the resolution and the weak field does not pay for it.
            let (dr_dt0, _) = rates_of(&raindrop, metric, p.r);
            let spans = ((dr_dt0.abs() * dt) / MAX_DR_PER_SUBSTEP).ceil();
            let n = (spans.max(1.0) as usize).min(MAX_SUBSTEPS);
            let h = dt / n as f64;

            for _ in 0..n {
                let (k1_r, k1_phi) = rates_of(&raindrop, metric, p.r);
                let r_mid = (p.r + 0.5 * h * k1_r).max(R_MIN);
                let (k2_r, k2_phi) = rates_of(&raindrop, metric, r_mid);
                p.r += h * k2_r;
                p.phi += h * 0.5 * (k1_phi + k2_phi);
                if p.r <= R_MIN {
                    break;
                }
            }
            p.age += dt;

            if p.r <= R_MIN {
                p = self.spawn_at_edge();
            } else {
                p.r = p.r.min(R_MAX);
                p.phi = p.phi.rem_euclid(2.0 * std::f64::consts::PI);
            }
            self.particles[i] = p;
        }
    }

    /// A replacement particle entering the field at its outer edge on a fresh azimuth.
    fn spawn_at_edge(&mut self) -> RiverParticle {
        RiverParticle {
            r: R_MAX,
            phi: 2.0 * std::f64::consts::PI * self.rng.next_f64(),
            age: 0.0,
        }
    }

    /// Draw the field: one streak per particle, running from where it was STREAK_T of coordinate
    /// time ago to where it is now, with a dot at the head. The colour is keyed to the invariant
    /// river speed `KerrSchild::river_speed`, so it reports the flow's speed against the local
    /// ZAMO rather than any coordinate rate.
    pub fn draw<F: Fn((f64, f64)) -> Pos2>(
        &self,
        painter: &egui::Painter,
        metric: &KerrSchild,
        to_screen: &F,
    ) {
        let raindrop = raindrop_congruence(metric);
        for p in &self.particles {
            let (dr_dt, dphi_dt) = rates_of(&raindrop, metric, p.r);
            let head = metric.cartesian_position(p.r, p.phi);
            let (vx, vy) = metric.cartesian_velocity(p.r, p.phi, dr_dt, dphi_dt);
            // First-order tail: the Cartesian point the particle came from STREAK_T ago.
            let tail = (head.0 - vx * STREAK_T, head.1 - vy * STREAK_T);

            let head_px = to_screen(head);
            let mut tail_px = to_screen(tail);
            let span = tail_px - head_px;
            if span.length() > STREAK_MAX_PX {
                tail_px = head_px + span.normalized() * STREAK_MAX_PX;
            }

            let fade = (p.age / FADE_IN_T).clamp(0.0, 1.0);
            let alpha = (Theme::RIVER_ALPHA as f64 * fade).round() as u8;
            if alpha == 0 {
                continue;
            }
            let colour = Theme::river_colour(metric.river_speed(p.r), alpha);

            painter.line_segment([tail_px, head_px], Stroke::new(1.2, colour));
            painter.circle_filled(head_px, HEAD_RADIUS_PX, colour);
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
