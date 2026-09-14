use egui::Color32;

pub struct Theme;

#[allow(dead_code)] // the palette is declared in full; not every colour is on screen yet
impl Theme {
    // UI FONT COLORS
    pub const UI_HEADING: Color32 = Color32::from_rgb(255, 100, 100);

    // Backgrounds
    pub const BG_VOID: Color32 = Color32::from_rgb(10, 12, 18);
    pub const CANVAS_BG: Color32 = Color32::from_rgb(14, 16, 24);
    pub const PANEL_BG: Color32 = Color32::from_rgb(20, 24, 34);

    // Region Fills (Translucent)
    pub const REGION_I_FILL: Color32 = Color32::from_rgba_premultiplied(15, 25, 45, 40);
    pub const ERGOSPHERE_FILL: Color32 = Color32::from_rgba_premultiplied(50, 40, 10, 45);
    pub const REGION_II_FILL: Color32 = Color32::from_rgba_premultiplied(45, 15, 55, 60);
    pub const REGION_III_FILL: Color32 = Color32::from_rgba_premultiplied(10, 50, 45, 60);
    pub const SINGULARITY_FILL: Color32 = Color32::from_rgba_premultiplied(80, 10, 20, 180);
    /// The spin arrow drawn inside the ring: `SINGULARITY_FILL` with its colour at 55% and the
    /// same alpha, so it reads as a shade of the disc it is drawn on rather than as a new colour.
    pub const SINGULARITY_SPIN: Color32 = Color32::from_rgba_premultiplied(44, 6, 11, 180);

    // Boundaries & Horizons
    pub const HORIZON_OUTER: Color32 = Color32::from_rgb(0, 230, 255); // Cyan
    pub const HORIZON_CAUCHY: Color32 = Color32::from_rgb(255, 0, 130); // Neon Pink / Magenta
    pub const ERGOSPHERE_LINE: Color32 = Color32::from_rgb(255, 180, 0); // Amber
    pub const SINGULARITY_LINE: Color32 = Color32::from_rgb(255, 40, 60); // Crimson

    // Observers
    pub const BOB_COLOR: Color32 = Color32::from_rgb(0, 255, 200); // Bright Mint / Cyan
    pub const ALICE_COLOR: Color32 = Color32::from_rgb(255, 160, 40); // Amber Orange

    /// Stroke-width scale for the *second* transmission drawn on a canvas, Bob's, against the
    /// first, Alice's. Both fields are coloured by the shift their rays carry, which is the physics
    /// and is not available as an identifying mark, so what separates them is weight: Bob's fronts
    /// are drawn at half the stroke width of Alice's, and his emission dots in `BOB_COLOR` where
    /// hers are in `ALICE_COLOR`.
    pub const SECONDARY_FRONT_WIDTH: f32 = 0.5;

    /// Fill opacity of one pulse's wedge on the (t, r) diagram, in the emitter's own colour: the
    /// band between the innermost and the outermost ray of that pulse, which is everything the
    /// projection onto (t, r) can say about a front that is really a curve in (r, phi).
    ///
    /// The number is set by the stacking, not by how one wedge looks. The wedges of a whole infall
    /// overlap - inside r+ every one of them covers the ground between the ring and r-, so a pixel
    /// down there is under all of them at once - and n layers of alpha a cover what is beneath them
    /// to a total opacity of 1 - (1 - a/255)^n. At a = 10 that is 0.18 for five layers, 0.33
    /// for ten, 0.55 for twenty and 0.80 for the forty-odd pulses of a full run: still short of
    /// flat, and the count is legible as brightness the whole way up. At 14, the top of the band
    /// worth trying, twenty layers already reach 0.68 and the last twenty add almost nothing.
    pub const WEDGE_FILL_ALPHA: u8 = 10;

    /// Opacity of the two edges of that wedge, the innermost and outermost ray. Seven times the
    /// fill, so a single pulse's reach is readable against its own interior, and low enough that
    /// the upper edges of the interior pulses - which all freeze onto r- - read as a stack of
    /// separate lines rather than as one thick line on the horizon.
    pub const WEDGE_EDGE_ALPHA: u8 = 70;

    // Light Cones, keyed to the observer rather than to the diagram, so a cone keeps its
    // identity in every reference frame. Fills sit at roughly 90% transparency; the edges use
    // the same hue at an alpha that still reads over the region shading.
    pub const CONE_FILL_ALPHA: u8 = 26;
    pub const CONE_EDGE_ALPHA: u8 = 150;

    pub const BOB_CONE_FUTURE_RGB: [u8; 3] = [140, 200, 255]; // light blue
    pub const BOB_CONE_PAST_RGB: [u8; 3] = [190, 160, 255]; // light purple
    pub const BOB_CONE_EDGE_RGB: [u8; 3] = [150, 205, 255];

    pub const ALICE_CONE_FUTURE_RGB: [u8; 3] = [255, 240, 150]; // light yellow
    pub const ALICE_CONE_PAST_RGB: [u8; 3] = [255, 195, 120]; // light orange
    pub const ALICE_CONE_EDGE_RGB: [u8; 3] = [255, 215, 130];

    /// The (future fill, past fill, edge) triple belonging to an observer, by name. Anything that
    /// is not Alice draws in Bob's blue/purple.
    pub fn cone_colours(name: &str) -> (Color32, Color32, Color32) {
        let (future, past, edge) = if name == "Alice" {
            (Self::ALICE_CONE_FUTURE_RGB, Self::ALICE_CONE_PAST_RGB, Self::ALICE_CONE_EDGE_RGB)
        } else {
            (Self::BOB_CONE_FUTURE_RGB, Self::BOB_CONE_PAST_RGB, Self::BOB_CONE_EDGE_RGB)
        };
        (
            Color32::from_rgba_unmultiplied(future[0], future[1], future[2], Self::CONE_FILL_ALPHA),
            Color32::from_rgba_unmultiplied(past[0], past[1], past[2], Self::CONE_FILL_ALPHA),
            Color32::from_rgba_unmultiplied(edge[0], edge[1], edge[2], Self::CONE_EDGE_ALPHA),
        )
    }

    // River of Space (the E = 1, L = 0 raindrop flow). The streaks are keyed to the invariant
    // river speed beta = sqrt(1 - alpha^2) relative to the local ZAMO, not to the radius, so the
    // colour says the same thing at every spin: pale blue in the weak field, the ergosphere amber
    // as beta closes on 1 at r+, the Cauchy magenta beyond it.
    pub const RIVER_SLOW_RGB: [u8; 3] = [120, 180, 235]; // cool pale blue
    pub const RIVER_MID_RGB: [u8; 3] = [255, 180, 0]; // ERGOSPHERE_LINE amber
    pub const RIVER_FAST_RGB: [u8; 3] = [255, 0, 130]; // HORIZON_CAUCHY magenta
    /// beta at or below which the streak is pure `RIVER_SLOW_RGB`.
    pub const RIVER_BETA_SLOW: f64 = 0.7;
    /// beta at which the ramp reaches `RIVER_MID_RGB`: exactly the horizon r+.
    pub const RIVER_BETA_MID: f64 = 1.0;
    /// beta at or above which the ramp is clamped to `RIVER_FAST_RGB`.
    pub const RIVER_BETA_FAST: f64 = 1.5;
    /// Opacity of a fully faded-in streak, low enough that worldlines and horizons stay legible.
    pub const RIVER_ALPHA: u8 = 130;

    /// The streak colour for a river speed beta, at opacity `alpha`.
    pub fn river_colour(beta: f64, alpha: u8) -> Color32 {
        let lerp = |lo: [u8; 3], hi: [u8; 3], t: f64| -> [u8; 3] {
            let t = t.clamp(0.0, 1.0);
            [
                (lo[0] as f64 + t * (hi[0] as f64 - lo[0] as f64)).round() as u8,
                (lo[1] as f64 + t * (hi[1] as f64 - lo[1] as f64)).round() as u8,
                (lo[2] as f64 + t * (hi[2] as f64 - lo[2] as f64)).round() as u8,
            ]
        };
        let rgb = if beta <= Self::RIVER_BETA_MID {
            let span = Self::RIVER_BETA_MID - Self::RIVER_BETA_SLOW;
            lerp(Self::RIVER_SLOW_RGB, Self::RIVER_MID_RGB, (beta - Self::RIVER_BETA_SLOW) / span)
        } else {
            let span = Self::RIVER_BETA_FAST - Self::RIVER_BETA_MID;
            lerp(Self::RIVER_MID_RGB, Self::RIVER_FAST_RGB, (beta - Self::RIVER_BETA_MID) / span)
        };
        Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], alpha)
    }

    // Grid
    /// One stroke colour for every gridline in the (t, r) diagram: the time lines and the radial
    /// lines are the same kind of thing and now look it.
    pub const GRID_LINE: Color32 = Color32::from_rgba_premultiplied(45, 52, 72, 90);
    pub const GRID_LINE_WIDTH: f32 = 0.8;
    pub const TEXT_BRIGHT: Color32 = Color32::from_rgb(230, 240, 255);
    pub const TEXT_MUTED: Color32 = Color32::from_rgb(130, 145, 170);

    // Blueshift / Warning Gradient
    pub const WARNING_RED: Color32 = Color32::from_rgb(255, 50, 80);
    pub const BLUESHIFT_BLUE: Color32 = Color32::from_rgb(60, 140, 255);
    pub const BLUESHIFT_VIOLET: Color32 = Color32::from_rgb(190, 60, 255);

    // Measured frequency shift nu_obs / nu_emit against the *emitter's* own frame, for the
    // reception dots on both canvases and the numbers the HUD quotes beside them. That ratio is
    // what a receiver actually measured on the light that reached them, so it is what an arrival is
    // reported in; it is not what the equatorial view colours its fronts by, because at the moment
    // of emission the rays of one pulse already carry every value it can take (see `front_colour`).
    // The ramp is keyed to log10 of the ratio, because the shift on a ray that has frozen onto r-
    // grows exponentially in coordinate time: only the logarithm has a usable dynamic range.
    /// log10(ratio) at or below which the colour is clamped to `SHIFT_RED_RGB`.
    pub const SHIFT_LOG_MIN: f64 = -1.0;
    /// log10(ratio) at which the ramp reaches `SHIFT_BLUE_RGB`.
    pub const SHIFT_LOG_BLUE: f64 = 1.0;
    /// log10(ratio) at or above which the colour is clamped to `SHIFT_VIOLET_RGB`.
    pub const SHIFT_LOG_MAX: f64 = 3.0;
    pub const SHIFT_RED_RGB: [u8; 3] = [255, 80, 50]; // deep orange-red, a tenfold redshift
    pub const SHIFT_WHITE_RGB: [u8; 3] = [245, 245, 250]; // unshifted
    pub const SHIFT_BLUE_RGB: [u8; 3] = [60, 140, 255]; // BLUESHIFT_BLUE, a tenfold blueshift
    pub const SHIFT_VIOLET_RGB: [u8; 3] = [190, 60, 255]; // BLUESHIFT_VIOLET, a thousandfold
    /// Opacity of a drawn signal ray, low enough to leave the horizons and worldlines legible.
    pub const SHIFT_ALPHA: u8 = 170;

    /// The colour of a measured frequency ratio nu_obs / nu_emit, at opacity `alpha`: orange-red
    /// at a tenfold redshift, white when unshifted, blue at a tenfold blueshift and violet at a
    /// thousandfold, clamped at both ends. A non-positive or non-finite ratio, which no real
    /// measurement produces, is drawn at the red end.
    pub fn shift_colour(ratio: f64, alpha: u8) -> Color32 {
        let lerp = |lo: [u8; 3], hi: [u8; 3], t: f64| -> [u8; 3] {
            let t = t.clamp(0.0, 1.0);
            [
                (lo[0] as f64 + t * (hi[0] as f64 - lo[0] as f64)).round() as u8,
                (lo[1] as f64 + t * (hi[1] as f64 - lo[1] as f64)).round() as u8,
                (lo[2] as f64 + t * (hi[2] as f64 - lo[2] as f64)).round() as u8,
            ]
        };
        let log = if ratio.is_finite() && ratio > 0.0 {
            ratio.log10().clamp(Self::SHIFT_LOG_MIN, Self::SHIFT_LOG_MAX)
        } else {
            Self::SHIFT_LOG_MIN
        };
        let rgb = if log <= 0.0 {
            lerp(Self::SHIFT_RED_RGB, Self::SHIFT_WHITE_RGB, 1.0 - log / Self::SHIFT_LOG_MIN)
        } else if log <= Self::SHIFT_LOG_BLUE {
            lerp(Self::SHIFT_WHITE_RGB, Self::SHIFT_BLUE_RGB, log / Self::SHIFT_LOG_BLUE)
        } else {
            let span = Self::SHIFT_LOG_MAX - Self::SHIFT_LOG_BLUE;
            lerp(Self::SHIFT_BLUE_RGB, Self::SHIFT_VIOLET_RGB, (log - Self::SHIFT_LOG_BLUE) / span)
        };
        Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], alpha)
    }

    // The wavefront ramp on the equatorial view: the *gain* a piece of front has picked up since it
    // was let go, nu(an infaller at this event) / nu(the infaller who was passing the emitter as it
    // left). Both observers are members of the one congruence that exists at every radius, inside
    // both horizons included - the E = 1, L = 0 raindrops - so the number is an ordinary measured
    // shift between two of them along the ray, and it means the same thing everywhere in the
    // picture. What makes it the right thing to colour a *front* by, where `shift_colour`'s
    // emitter-relative ratio is not, is that it is exactly 1 for every ray of a pulse at the moment
    // of emission: a fresh front comes out one uniform colour and then earns its way up the ramp,
    // instead of being split into a white half and a frozen half before it has gone anywhere.
    //
    // The ramp is keyed to log10 of the gain for the same reason `shift_colour` is - the frozen
    // family's gain grows like exp(kappa_- t) - but it runs much further: those rays reach 1e5
    // within a run, where the reception ramp is built for the 1e3 an actual arrival shows, so the
    // two ceilings are different numbers about different quantities and neither is the other's.
    /// log10(gain) at or below which the colour is clamped to `FRONT_MAROON_RGB`. A ray can lose
    /// frequency between two raindrops rather than gain it - one climbing outward, away from the
    /// congruence's fall - and a tenfold loss is as dark as that end gets.
    pub const FRONT_LOG_MIN: f64 = -1.0;
    /// log10(gain) of a front that has neither gained nor lost: every ray at its own emission.
    pub const FRONT_LOG_BIRTH: f64 = 0.0;
    /// log10(gain) at which the ramp reaches `FRONT_ORANGE_RGB`: a threefold blueshift.
    pub const FRONT_LOG_ORANGE: f64 = 0.5;
    /// log10(gain) at which the ramp reaches `FRONT_YELLOW_RGB`: a tenfold blueshift.
    pub const FRONT_LOG_YELLOW: f64 = 1.0;
    /// log10(gain) at which the ramp reaches `FRONT_WHITE_RGB`: about thirtyfold.
    pub const FRONT_LOG_WHITE: f64 = 1.5;
    /// log10(gain) at which the ramp reaches `FRONT_BLUE_RGB`: a thousandfold blueshift.
    pub const FRONT_LOG_BLUE: f64 = 3.0;
    /// log10(gain) at or above which the colour is clamped to `FRONT_VIOLET_RGB`: a hundred
    /// thousandfold, which is the order the stack frozen on r- reaches within a run.
    pub const FRONT_LOG_MAX: f64 = 5.0;

    pub const FRONT_MAROON_RGB: [u8; 3] = [45, 0, 12]; // a tenfold loss: nearly black maroon
    pub const FRONT_RED_RGB: [u8; 3] = [170, 25, 20]; // deep red, the colour of a newborn front
    pub const FRONT_ORANGE_RGB: [u8; 3] = [235, 120, 30];
    pub const FRONT_YELLOW_RGB: [u8; 3] = [255, 225, 90];
    pub const FRONT_WHITE_RGB: [u8; 3] = [245, 245, 250];
    pub const FRONT_BLUE_RGB: [u8; 3] = [60, 140, 255]; // BLUESHIFT_BLUE
    pub const FRONT_VIOLET_RGB: [u8; 3] = [190, 60, 255]; // BLUESHIFT_VIOLET

    /// The ramp above as (log10(gain), colour) stops, in increasing order of the first entry.
    /// `front_colour` interpolates between consecutive stops and clamps outside the two ends, so
    /// this array is the whole definition of the ramp and the consts are its labels.
    pub const FRONT_STOPS: [(f64, [u8; 3]); 7] = [
        (Self::FRONT_LOG_MIN, Self::FRONT_MAROON_RGB),
        (Self::FRONT_LOG_BIRTH, Self::FRONT_RED_RGB),
        (Self::FRONT_LOG_ORANGE, Self::FRONT_ORANGE_RGB),
        (Self::FRONT_LOG_YELLOW, Self::FRONT_YELLOW_RGB),
        (Self::FRONT_LOG_WHITE, Self::FRONT_WHITE_RGB),
        (Self::FRONT_LOG_BLUE, Self::FRONT_BLUE_RGB),
        (Self::FRONT_LOG_MAX, Self::FRONT_VIOLET_RGB),
    ];

    /// Opacity of a segment of front drawn in the frozen family's own pass, E - Omega_- L < 0.
    ///
    /// Those arcs collapse to within a fraction of a pixel of the magenta r- circle within a few M,
    /// so they are only legible drawn nearly opaque and drawn last, on top of it. The colour is the
    /// same `front_colour` every other segment gets - the frozen family is not a different kind of
    /// thing and no longer gets a flat colour of its own - and what is left of the old treatment is
    /// the weight it is drawn at, which is a legibility measure and says nothing about the physics.
    pub const FRONT_FROZEN_ALPHA: u8 = 230;

    /// The colour of a wavefront gain nu(infaller here) / nu(infaller at the emission event), at
    /// opacity `alpha`: deep red at 1, where every front is born, darkening to maroon at a tenfold
    /// loss and running deep red - orange - yellow - white - blue - violet up to a hundred
    /// thousandfold gain, clamped at both ends. A non-positive or non-finite gain, which no real
    /// measurement produces, is drawn at the dark end.
    pub fn front_colour(gain: f64, alpha: u8) -> Color32 {
        let log = if gain.is_finite() && gain > 0.0 {
            gain.log10().clamp(Self::FRONT_LOG_MIN, Self::FRONT_LOG_MAX)
        } else {
            Self::FRONT_LOG_MIN
        };
        let stops = Self::FRONT_STOPS;
        // The clamp above puts `log` inside [first stop, last stop], so the pair search always
        // finds a segment and the fallback is only there because the compiler cannot see that.
        let mut rgb = stops[stops.len() - 1].1;
        for pair in stops.windows(2) {
            let ((lo_log, lo), (hi_log, hi)) = (pair[0], pair[1]);
            if log <= hi_log {
                let t = ((log - lo_log) / (hi_log - lo_log)).clamp(0.0, 1.0);
                rgb = [
                    (lo[0] as f64 + t * (hi[0] as f64 - lo[0] as f64)).round() as u8,
                    (lo[1] as f64 + t * (hi[1] as f64 - lo[1] as f64)).round() as u8,
                    (lo[2] as f64 + t * (hi[2] as f64 - lo[2] as f64)).round() as u8,
                ];
                break;
            }
        }
        Color32::from_rgba_unmultiplied(rgb[0], rgb[1], rgb[2], alpha)
    }
}
