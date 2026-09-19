use crate::physics::observer::Who;
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
    /// The plate behind an axis label that has to read over whatever the chart has painted there:
    /// the canvas colour, most of the way to opaque, so that the label sits on the chart's own
    /// dark rather than on a wavefront's orange, and what is under the plate still shows faintly.
    pub const LABEL_PLATE: Color32 = Color32::from_rgba_premultiplied(12, 14, 21, 225);

    // Region Fills (Translucent). Region I has none: the exterior is the canvas background. Every
    // view paints these as disjoint strips or bands, never one over another, so that each region
    // is the same colour over the same background everywhere.
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
    /// The fill of a cone in the 2D+1 volume, three times the flat diagram's. There the cone is
    /// a fan seen through the glass of a horizon pipe and over a floor full of fronts, and at the
    /// diagram's alpha it vanished into both; this is the lowest value at which its tilt reads.
    pub const VOLUME_CONE_FILL_ALPHA: u8 = 80;

    pub const BOB_CONE_FUTURE_RGB: [u8; 3] = [140, 200, 255]; // light blue
    pub const BOB_CONE_PAST_RGB: [u8; 3] = [190, 160, 255]; // light purple
    pub const BOB_CONE_EDGE_RGB: [u8; 3] = [150, 205, 255];

    pub const ALICE_CONE_FUTURE_RGB: [u8; 3] = [255, 240, 150]; // light yellow
    pub const ALICE_CONE_PAST_RGB: [u8; 3] = [255, 195, 120]; // light orange
    pub const ALICE_CONE_EDGE_RGB: [u8; 3] = [255, 215, 130];

    /// The (future fill, past fill, edge) triple belonging to an observer. Anything that is not
    /// Alice - including an observer `Who::of` cannot place - draws in Bob's blue/purple.
    pub fn cone_colours(who: Option<Who>) -> (Color32, Color32, Color32) {
        Self::cone_colours_at(who, Self::CONE_FILL_ALPHA)
    }

    /// The same triple with the two fills at a chosen alpha: the volume view draws the cone as a
    /// surface rather than as a wedge and needs it denser, but in the same hues, so that a cone
    /// keeps its identity between the two pictures.
    pub fn cone_colours_at(who: Option<Who>, fill_alpha: u8) -> (Color32, Color32, Color32) {
        let (future, past, edge) = if who == Some(Who::Alice) {
            (Self::ALICE_CONE_FUTURE_RGB, Self::ALICE_CONE_PAST_RGB, Self::ALICE_CONE_EDGE_RGB)
        } else {
            (Self::BOB_CONE_FUTURE_RGB, Self::BOB_CONE_PAST_RGB, Self::BOB_CONE_EDGE_RGB)
        };
        (
            Color32::from_rgba_unmultiplied(future[0], future[1], future[2], fill_alpha),
            Color32::from_rgba_unmultiplied(past[0], past[1], past[2], fill_alpha),
            Color32::from_rgba_unmultiplied(edge[0], edge[1], edge[2], Self::CONE_EDGE_ALPHA),
        )
    }

    /// The colour a cone's rim is stroked in, in the volume: a brighter, opaque shade of the wall
    /// it is the lip of. The two halves have two wall colours, and the rim carries the same
    /// distinction rather than one edge colour for both, so the eye reads "future" and "past" off
    /// the lip as well as the glass. The lift is halfway from the wall's own hue to white.
    pub fn cone_rim_colour(fill: Color32) -> Color32 {
        let [r, g, b, _] = fill.to_srgba_unmultiplied();
        let lift = |c: u8| -> u8 { c.saturating_add((255 - c) / 2) };
        Color32::from_rgb(lift(r), lift(g), lift(b))
    }

    // Grid
    /// One stroke colour for every gridline in the (t, r) diagram: the time lines and the radial
    /// lines are the same kind of thing and now look it.
    pub const GRID_LINE: Color32 = Color32::from_rgba_premultiplied(45, 52, 72, 90);
    pub const GRID_LINE_WIDTH: f32 = 0.8;
    /// The same colour at `factor` of its brightness, alpha untouched. For a control that has to
    /// read as the same thing in two states without changing hue between them.
    pub fn dimmed(c: Color32, factor: f32) -> Color32 {
        let scale = |v: u8| (v as f32 * factor).round().clamp(0.0, 255.0) as u8;
        Color32::from_rgba_unmultiplied(scale(c.r()), scale(c.g()), scale(c.b()), c.a())
    }

    /// The smallest type anywhere in the app, in points before the Font Size slider scales it.
    ///
    /// It is a floor, not a size: egui's own styles and every canvas label are clamped up to it,
    /// so nothing prints smaller than the title of a telemetry box. Twelve rather than ten because
    /// egui's default Body is 12.5, and a `.small()` caption floored at ten still came out a fifth
    /// shorter than the labels either side of it on an observer card.
    pub const MIN_FONT_PT: f32 = 12.0;

    pub const TEXT_BRIGHT: Color32 = Color32::from_rgb(230, 240, 255);
    pub const TEXT_MUTED: Color32 = Color32::from_rgb(130, 145, 170);

    /// The telemetry box's measured-speed rows: what a local observer standing at the event reads
    /// off their own ruler and clock, as against the chart rates above them, which are printed in
    /// `TEXT_BRIGHT`. A warm off-white rather than the blue-white of the rest, because the whole
    /// point of those rows is that they are a different kind of statement from the lines they sit
    /// under - a measurement rather than a coordinate difference - and nothing else in the box
    /// distinguishes them.
    pub const SPEED_MEASURED: Color32 = Color32::from_rgb(255, 238, 205);

    /// Outline on the panel's chip buttons: the quick picks that set the slider beside them, and
    /// the small choices - a motion, a release, a step mode - that egui draws as bare text until
    /// they are the selected one. A dimmed blue rather than the headings' full cyan: a dozen of
    /// them sit in a row, and at full saturation a row of them reads as a row of alarms rather
    /// than as a row of choices.
    pub const CHIP_OUTLINE: Color32 = Color32::from_rgb(52, 96, 160);

    /// Outline on the chip that is currently the selected one. The fill already says which it is;
    /// this makes it say so from across the panel.
    pub const CHIP_OUTLINE_ACTIVE: Color32 = Color32::from_rgb(0, 230, 255);

    /// Fill of a transport button that is engaged: Play or Pause while it is the state the run is
    /// in, and any of the others for the moment after it is pressed.
    ///
    /// The same hue as `CHIP_OUTLINE`, carried down to something a white icon still reads clearly
    /// against. A fill at the outline's own value would be a blue button with a blue edge and no
    /// edge to see; this keeps the outline the brighter of the two, so an engaged button is a
    /// filled shape with a rim rather than a blue blob.
    pub const TRANSPORT_ENGAGED: Color32 = Color32::from_rgb(26, 50, 86);

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
    //
    // Every stop of this ramp is the same lightness, and only the hue moves. The ramp this
    // replaced ran from a near-black maroon at a loss to white at a thirtyfold gain: a factor of
    // 157 in relative luminance, which cost it in two ways. The maroon sat at a relative luminance
    // of 0.0058 against a `BG_VOID` of 0.0035, so a ray that had *lost* frequency - the one case
    // the ramp exists to distinguish from the ordinary ones - was drawn as very nearly nothing. And
    // a ramp that swings by that much in brightness stops being read as a ramp at all: the eye
    // sorts the strokes into bright ones and dim ones long before it reads a hue off them, so the
    // white band read as "this part matters" and the dark end as "ignore this", neither of which is
    // a statement about the light. Held at one lightness, what is left to see is the shift, which
    // is the only thing being measured here.
    //
    // The stops are struck in Oklab: each takes as much chroma as its hue can hold at
    // `FRONT_STOP_LIGHTNESS`, up to `FRONT_STOP_MAX_CHROMA`. The cap is not a taste either - the
    // legs between stops are interpolated in sRGB, and the further apart two neighbouring stops sit
    // in chroma the more the leg between them sags in lightness, so capping the chroma is what
    // holds the *interpolated* ramp to the same lightness as the stops. `theme::tests` measures
    // both, along the ramp rather than only at the stops.
    //
    // With lightness spent, the hue sequence is the spectrum itself: red, orange, yellow, green,
    // blue, violet, in the order a spectrograph lays them out, which is the order the quantity
    // being drawn moves in. White is gone from the middle of the ramp because white is the one
    // colour that cannot be had at a fixed lightness below its own.
    /// Oklab lightness every stop of the front ramp is struck at. 0.72 is where the chroma
    /// available across the six hues is widest: yellow runs out of gamut below it and blue above.
    pub const FRONT_STOP_LIGHTNESS: f64 = 0.72;
    /// Most chroma any stop is given, and the least the gamut allows at this lightness, which
    /// between them bound how much the stops may differ in colourfulness. See the note above on why
    /// the spread is capped rather than left at whatever each hue could hold: yellow and blue can
    /// only hold 0.147 at this lightness, and letting green take its full 0.227 beside them put a
    /// sag of 0.028 into the lightness of the legs either side of it.
    pub const FRONT_STOP_MAX_CHROMA: f64 = 0.190;
    /// See `FRONT_STOP_MAX_CHROMA`. The bottom stop is the deliberate exception.
    pub const FRONT_STOP_MIN_CHROMA: f64 = 0.145;

    /// log10(gain) at or below which the colour is clamped to `FRONT_GREY_RGB`. A ray can lose
    /// frequency between two raindrops rather than gain it - one climbing outward, away from the
    /// congruence's fall - and a tenfold loss is as far as that end goes.
    pub const FRONT_LOG_MIN: f64 = -1.0;
    /// log10(gain) of a front that has neither gained nor lost: every ray at its own emission.
    pub const FRONT_LOG_BIRTH: f64 = 0.0;
    /// log10(gain) at which the ramp reaches `FRONT_ORANGE_RGB`: a threefold blueshift.
    pub const FRONT_LOG_ORANGE: f64 = 0.5;
    /// log10(gain) at which the ramp reaches `FRONT_YELLOW_RGB`: a tenfold blueshift.
    pub const FRONT_LOG_YELLOW: f64 = 1.0;
    /// log10(gain) at which the ramp reaches `FRONT_GREEN_RGB`: about thirtyfold.
    pub const FRONT_LOG_GREEN: f64 = 1.5;
    /// log10(gain) at which the ramp reaches `FRONT_BLUE_RGB`: a thousandfold blueshift.
    pub const FRONT_LOG_BLUE: f64 = 3.0;
    /// log10(gain) at or above which the colour is clamped to `FRONT_VIOLET_RGB`: a hundred
    /// thousandfold, which is the order the stack frozen on r- reaches within a run.
    pub const FRONT_LOG_MAX: f64 = 5.0;

    // The six spectral hues below are the Oklab hues the old ramp's stops already had, carried over
    // unchanged and re-struck at the common lightness and chroma; green is the one new hue, filling
    // the place white held. A `#[test]` measures the lightness of all seven.
    /// A tenfold loss: colourless. The bottom stop is the one place the ramp leaves the spectrum,
    /// because it stands for light that has come off the red end of it, and there is no hue below
    /// red to go to. It is drawn at chroma 0.025 rather than 0 so that it reads as the end of the
    /// warm run rather than as a stroke that has lost its colour by accident, and at the same
    /// lightness as everything else so that a weak front and a losing one cannot be confused.
    pub const FRONT_GREY_RGB: [u8; 3] = [180, 159, 156];
    pub const FRONT_RED_RGB: [u8; 3] = [255, 114, 99]; // the colour of a newborn front
    pub const FRONT_ORANGE_RGB: [u8; 3] = [252, 123, 0];
    pub const FRONT_YELLOW_RGB: [u8; 3] = [192, 163, 0];
    pub const FRONT_GREEN_RGB: [u8; 3] = [67, 194, 81];
    pub const FRONT_BLUE_RGB: [u8; 3] = [104, 165, 255];
    pub const FRONT_VIOLET_RGB: [u8; 3] = [202, 124, 251];

    /// The ramp above as (log10(gain), colour) stops, in increasing order of the first entry.
    /// `front_colour` interpolates between consecutive stops and clamps outside the two ends, so
    /// this array is the whole definition of the ramp and the consts are its labels.
    pub const FRONT_STOPS: [(f64, [u8; 3]); 7] = [
        (Self::FRONT_LOG_MIN, Self::FRONT_GREY_RGB),
        (Self::FRONT_LOG_BIRTH, Self::FRONT_RED_RGB),
        (Self::FRONT_LOG_ORANGE, Self::FRONT_ORANGE_RGB),
        (Self::FRONT_LOG_YELLOW, Self::FRONT_YELLOW_RGB),
        (Self::FRONT_LOG_GREEN, Self::FRONT_GREEN_RGB),
        (Self::FRONT_LOG_BLUE, Self::FRONT_BLUE_RGB),
        (Self::FRONT_LOG_MAX, Self::FRONT_VIOLET_RGB),
    ];

    /// Opacity of a segment of front drawn in the frozen family's own pass, E - Omega_- L < 0.
    ///
    /// Those arcs collapse to within a fraction of a pixel of the magenta r- circle within a few M,
    /// so they are only legible drawn nearly opaque and drawn last, on top of it. The colour is the
    /// same `front_colour` every other segment gets - the frozen family is not a different kind of
    /// thing and no longer gets a flat colour of its own - and what is left of the old treatment is
    /// the weight it is drawn at, which is a legibility measure and says nothing about the physics:
    /// nothing on these fronts is measured in opacity.
    pub const FRONT_FROZEN_ALPHA: u8 = 230;

    /// The colour of a wavefront gain nu(infaller here) / nu(infaller at the emission event), at
    /// opacity `alpha`: red at 1, where every front is born, running red, orange, yellow, green,
    /// blue, violet up to a hundred thousandfold gain and out to a colourless grey at a tenfold
    /// loss, clamped at both ends. Every one of those stops is the same lightness, so what the
    /// colour says is the shift and only the shift; the brightness is `alpha`'s to carry. A
    /// non-positive or non-finite gain, which no real measurement produces, is drawn at the grey
    /// end.
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

#[cfg(test)]
mod tests {
    use super::Theme;

    /// The Oklab coordinates (L, a, b) of an sRGB triple, which is the space the front ramp's stops
    /// were struck in and so the space the claim about them has to be checked in.
    ///
    /// Bjorn Ottosson's transform, verbatim: sRGB -> linear light, the linear-light-to-cone matrix,
    /// cube roots, and the cone-to-Lab matrix. L is a perceptual lightness, 0 at black and 1 at
    /// white, and (a, b) are the opponent axes, so the chroma is the length of (a, b) and the hue
    /// is its angle. It lives here rather than in the drawing code because nothing the app draws
    /// needs it: the ramp is seven constants, and this is how they were computed and how they are
    /// audited.
    fn oklab(rgb: [u8; 3]) -> (f64, f64, f64) {
        let (r, g, b) = (linear(rgb[0]), linear(rgb[1]), linear(rgb[2]));
        let l = (0.412_221_470_8 * r + 0.536_332_536_3 * g + 0.051_445_992_9 * b).cbrt();
        let m = (0.211_903_498_2 * r + 0.680_699_545_1 * g + 0.107_396_956_6 * b).cbrt();
        let s = (0.088_302_461_9 * r + 0.281_718_837_6 * g + 0.629_978_700_5 * b).cbrt();
        (
            0.210_454_255_3 * l + 0.793_617_785_0 * m - 0.004_072_046_8 * s,
            1.977_998_495_1 * l - 2.428_592_205_0 * m + 0.450_593_709_9 * s,
            0.025_904_037_1 * l + 0.782_771_766_2 * m - 0.808_675_766_0 * s,
        )
    }

    /// One sRGB channel as linear light, which both colour measures below start from.
    fn linear(c: u8) -> f64 {
        let c = f64::from(c) / 255.0;
        if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
    }

    /// Relative luminance, CIE Y: what a photometer pointed at the pixel would read, and so what
    /// the alpha channel is competing with when it tries to say how strong a signal is.
    fn luminance(rgb: [u8; 3]) -> f64 {
        0.2126 * linear(rgb[0]) + 0.7152 * linear(rgb[1]) + 0.0722 * linear(rgb[2])
    }

    /// The ramp's colour at a gain, as a plain sRGB triple.
    fn stop(gain: f64) -> [u8; 3] {
        let c = Theme::front_colour(gain, 255);
        [c.r(), c.g(), c.b()]
    }

    #[test]
    fn test_the_front_ramp_carries_no_brightness_of_its_own() {
        // The one thing a wavefront's colour says is the shift its light has picked up, and a ramp
        // that swings in brightness does not say it: a viewer cannot unsee a white stroke being
        // brighter than a red one, so they sort the strokes into bright and dim before they read a
        // hue off any of them, and the ramp's own lightness gets taken for a property of the light.
        // So every stop is struck at one Oklab lightness, and this measures it along the
        // interpolated ramp rather than only at the stops: a leg between two stops of equal
        // lightness can still sag in the middle, because `front_colour` interpolates in sRGB.
        let mut lightness = (f64::INFINITY, 0.0f64);
        let mut relative = (f64::INFINITY, 0.0f64);
        for step in 0..=600 {
            let log = Theme::FRONT_LOG_MIN
                + (Theme::FRONT_LOG_MAX - Theme::FRONT_LOG_MIN) * f64::from(step) / 600.0;
            let rgb = stop(10.0_f64.powf(log));
            let (l, _, _) = oklab(rgb);
            lightness = (lightness.0.min(l), lightness.1.max(l));
            let y = luminance(rgb);
            relative = (relative.0.min(y), relative.1.max(y));
        }
        println!(
            "across the whole ramp: Oklab lightness {:.3}..{:.3} against a target of {:.2}, and \
             relative luminance {:.3}..{:.3}, a spread of {:.2}x",
            lightness.0,
            lightness.1,
            Theme::FRONT_STOP_LIGHTNESS,
            relative.0,
            relative.1,
            relative.1 / relative.0
        );
        assert!(
            (lightness.0 - Theme::FRONT_STOP_LIGHTNESS).abs() < 0.02
                && (lightness.1 - Theme::FRONT_STOP_LIGHTNESS).abs() < 0.02,
            "no colour on the ramp departs from the common lightness: {lightness:?}"
        );
        // The same claim in the units a photometer would use. The ramp this replaced spanned a
        // factor of 157 here, which is what made the maroon end vanish into the background and the
        // white band shout.
        assert!(
            relative.1 / relative.0 < 1.25,
            "and it is flat in measured luminance too, not only in Oklab: {relative:?}"
        );

        // At the stops themselves, where the chroma can be checked exactly. Each spectral stop
        // takes as much as its hue can hold at this lightness, capped so that the legs between
        // them do not sag; the bottom stop is deliberately nearly colourless.
        for (log, rgb) in Theme::FRONT_STOPS {
            let (l, a, b) = oklab(rgb);
            let chroma = a.hypot(b);
            let held = if rgb == Theme::FRONT_GREY_RGB {
                (chroma - 0.025).abs() < 0.005
            } else {
                (Theme::FRONT_STOP_MIN_CHROMA - 0.005..=Theme::FRONT_STOP_MAX_CHROMA + 0.005)
                    .contains(&chroma)
            };
            assert!(
                (l - Theme::FRONT_STOP_LIGHTNESS).abs() < 0.005 && held,
                "the stop at log10(gain) = {log} is off the ramp's lightness or chroma: \
                 L = {l:.3}, C = {chroma:.3}"
            );
        }
    }

    #[test]
    fn test_the_front_ramp_only_ever_turns_one_way_in_hue() {
        // With the lightness spent, the hue is the whole of the ramp, so it has to run one way:
        // red - orange - yellow - green - blue - violet, the spectral order, from the gain of 1
        // every front is born at up to the 1e5 the frozen stack reaches. A ramp that doubled back
        // would put two different shifts on the same colour.
        //
        // Below the birth stop it leaves the spectrum instead: that leg runs to a colourless grey,
        // which is a fall in chroma at a fixed hue rather than a turn, so it is measured on its own
        // at the end.
        let hue = |gain: f64| -> f64 {
            let (_, a, b) = oklab(stop(gain));
            let h = b.atan2(a).to_degrees();
            if h < 0.0 { h + 360.0 } else { h }
        };
        let mut previous = hue(1.0);
        let mut worst = 0.0f64;
        for step in 1..=500 {
            let log = Theme::FRONT_LOG_BIRTH
                + (Theme::FRONT_LOG_MAX - Theme::FRONT_LOG_BIRTH) * f64::from(step) / 500.0;
            let h = hue(10.0_f64.powf(log));
            worst = worst.max(previous - h);
            previous = h;
        }
        println!(
            "the ramp turns from {:.0} deg at gain 1 to {:.0} deg at 1e5, never doubling back by \
             more than {worst:.2} deg",
            hue(1.0),
            hue(1e5)
        );
        // A degree of slack for the rounding to 8-bit sRGB, which puts a wobble of about a fifth of
        // a degree into the interpolation near the blue stop; the ramp itself never turns back.
        assert!(worst < 1.0, "the hue runs one way from gain 1 to the top: it fell by {worst} deg");
        assert!(hue(1e5) - hue(1.0) > 250.0, "and it runs the length of the spectrum");

        // The losing end: the newborn red's own hue, with the colour drained out of it.
        let (_, a, b) = oklab(stop(0.1));
        assert!(
            a.hypot(b) < 0.3 * Theme::FRONT_STOP_MIN_CHROMA,
            "a tenfold loss is drawn nearly colourless rather than as another shift"
        );
    }
}
