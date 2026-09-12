use egui::Color32;

pub struct Theme;

#[allow(dead_code)] // the palette is declared in full; not every colour is on screen yet
impl Theme {
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

    // Boundaries & Horizons
    pub const HORIZON_OUTER: Color32 = Color32::from_rgb(0, 230, 255); // Cyan
    pub const HORIZON_CAUCHY: Color32 = Color32::from_rgb(255, 0, 130); // Neon Pink / Magenta
    pub const ERGOSPHERE_LINE: Color32 = Color32::from_rgb(255, 180, 0); // Amber
    pub const SINGULARITY_LINE: Color32 = Color32::from_rgb(255, 40, 60); // Crimson

    // Observers
    pub const BOB_COLOR: Color32 = Color32::from_rgb(0, 255, 200); // Bright Mint / Cyan
    pub const ALICE_COLOR: Color32 = Color32::from_rgb(255, 160, 40); // Amber Orange

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
}
