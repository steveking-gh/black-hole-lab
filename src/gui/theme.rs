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
