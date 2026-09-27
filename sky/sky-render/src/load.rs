//! Reading the inputs from disk: the star map, and the bundle frames the video needs.
//!
//! One fallback lives here: a map texel that is not a finite number is read as black. That is
//! plausible, not known; a damaged texel's light is simply missing. It keeps a NaN from spreading
//! through every level of the rip-map built from it, which would spoil far more than the one
//! texel. NASA's maps have no such texel; any other map's are counted, and the run says how many
//! there were, so that a black speck in the video can be traced to its cause.

use std::path::Path;

use crate::field::{Judge, RayField};

/// A star map read from an OpenEXR file: linear RGB, row by row from the top.
pub struct MapImage {
    pub width: usize,
    pub height: usize,
    pub texels: Vec<[f32; 3]>,
    /// How many texels held a value that is not a finite number, read as black.
    pub damaged: usize,
}

/// Reads the R, G and B channels of an OpenEXR file's first layer as linear light.
///
/// NASA's maps store B, G and R as half floats, ZIP-compressed, in DECREASING_Y line order; the
/// reader places each line by its y coordinate, so the stored order does not matter, and row 0
/// is the top of the image (sky/maps/README.md). A value that is not a finite number is read as
/// black, so that a damaged texel cannot spread NaN through the pyramid built from it; the maps
/// hold values in [0, 1] and negative values are kept as they are, since the maps have none.
pub fn read_map(path: &Path) -> Result<MapImage, String> {
    use exr::prelude::*;
    if !path.is_file() {
        return Err(format!("there is no star map at {}", path.display()));
    }
    let image = read()
        .no_deep_data()
        .largest_resolution_level()
        .rgb_channels(
            |size: Vec2<usize>, _: &RgbChannels| MapImage {
                width: size.width(),
                height: size.height(),
                texels: vec![[0.0; 3]; size.width() * size.height()],
                damaged: 0,
            },
            |map: &mut MapImage, at: Vec2<usize>, (r, g, b): (f32, f32, f32)| {
                let finite = |c: f32| if c.is_finite() { c } else { 0.0 };
                if ![r, g, b].iter().all(|c| c.is_finite()) {
                    map.damaged += 1;
                }
                map.texels[at.y() * map.width + at.x()] = [finite(r), finite(g), finite(b)];
            },
        )
        .first_valid_layer()
        .all_attributes()
        .from_file(path)
        .map_err(|e| format!("could not read the star map {}: {e}", path.display()))?;
    let map = image.layer_data.channel_data.pixels;
    if map.width == 0 || map.height == 0 {
        return Err(format!("the star map {} is empty", path.display()));
    }
    Ok(map)
}

/// Reads bundle frame `index` and prepares it for sampling, judged as `judge` says when there is
/// one (`--undersampled mark`), on `threads` threads.
pub fn read_field(
    bundle: &sky_format::BundleReader,
    index: u32,
    judge: Option<Judge>,
    threads: usize,
) -> Result<RayField, String> {
    let frame = bundle
        .read_frame(index)
        .map_err(|e| format!("could not read frame {index} of the bundle: {e}"))?;
    Ok(prepare(&frame, judge, threads))
}

/// A frame prepared for sampling, judged when there is a judge: exactly what the renderer draws
/// from, for tests that build frames in memory.
pub fn prepare(frame: &sky_format::Frame, judge: Option<Judge>, threads: usize) -> RayField {
    let mut field = RayField::from_frame(frame);
    if let Some(judge) = judge {
        field.judge(judge, threads);
    }
    field
}
