use egui::Color32;

pub struct Theme;

#[allow(dead_code)]
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

    // Light Cones (Near-invisible whisper tint, reduced by another 75%)
    pub const LIGHTCONE_FUTURE_FILL: Color32 = Color32::from_rgba_premultiplied(0, 160, 200, 1);
    pub const LIGHTCONE_PAST_FILL: Color32 = Color32::from_rgba_premultiplied(120, 80, 200, 1);
    pub const LIGHTCONE_BORDER_INGOING: Color32 = Color32::from_rgba_premultiplied(120, 180, 255, 120);
    pub const LIGHTCONE_BORDER_OUTGOING: Color32 = Color32::from_rgba_premultiplied(255, 230, 100, 140);

    // Wavefronts & Grid
    pub const WAVEFRONT_PULSE: Color32 = Color32::from_rgba_premultiplied(200, 240, 255, 160);
    pub const GRID_LINE: Color32 = Color32::from_rgba_premultiplied(255, 255, 255, 15);
    pub const TEXT_BRIGHT: Color32 = Color32::from_rgb(230, 240, 255);
    pub const TEXT_MUTED: Color32 = Color32::from_rgb(130, 145, 170);

    // Blueshift / Warning Gradient
    pub const WARNING_RED: Color32 = Color32::from_rgb(255, 50, 80);
    pub const BLUESHIFT_BLUE: Color32 = Color32::from_rgb(60, 140, 255);
    pub const BLUESHIFT_VIOLET: Color32 = Color32::from_rgb(190, 60, 255);
}
