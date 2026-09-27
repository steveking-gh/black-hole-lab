//! Reading the inputs from disk: the star map, and the bundle frames the video needs.

use std::path::Path;

use crate::field::RayField;

/// A star map read from an OpenEXR file: linear RGB, row by row from the top.
pub struct MapImage {
    pub width: usize,
    pub height: usize,
    pub texels: Vec<[f32; 3]>,
}

/// Reads the R, G and B channels of an OpenEXR file's first layer as linear light.
///
/// NASA's maps store B, G and R as half floats, ZIP-compressed, in DECREASING_Y line order; the
/// reader places each line by its y coordinate, so the stored order does not matter, and row 0
/// is the top of the image (assets/sky/README.md). A value that is not a finite number is read as
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
            },
            |map: &mut MapImage, at: Vec2<usize>, (r, g, b): (f32, f32, f32)| {
                let finite = |c: f32| if c.is_finite() { c } else { 0.0 };
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

/// Reads bundle frame `index` and prepares it for sampling.
pub fn read_field(bundle: &sky_format::BundleReader, index: u32) -> Result<RayField, String> {
    let frame = bundle
        .read_frame(index)
        .map_err(|e| format!("could not read frame {index} of the bundle: {e}"))?;
    Ok(RayField::from_frame(&frame))
}
