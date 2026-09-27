//! The test cases' mathematics: for a grid and a moment of the observer's watch, the frame a
//! flat-space observer records.
//!
//! Every case here is an observer at rest at the origin of flat space, so every ray reaches the far
//! sky (fate 1) unbent, unshifted (g = 1) and without winding about anything (winding 0). All that
//! differs between the cases is how the observer's triad sits against the far-sky axes, which is a
//! rotation R, the same for every pixel of a frame. A direction with triad components `n` then has
//! far-sky components
//!
//!     d = R n
//!
//! In flat space the light seen along `n` came, undeflected, from the point of the far sky that
//! `n` points at, so that is the whole of the physics. The cases exist so that the renderer can be
//! checked against a sky whose right answer is known by inspection: `still` must reproduce the star
//! map, and `turn` must slide it across the frame at a known rate.
//!
//! Nothing here touches a file. The command line and the bundle live in `main.rs`, so that the
//! tests can call these functions on a grid of a few pixels.

use sky_format::{Frame, Grid, fate};

/// A 3x3 matrix, rows first: `m[r][c]`.
pub type Matrix = [[f64; 3]; 3];

/// The rotation that leaves every vector where it is.
pub const IDENTITY: Matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// A flat-space test case and its parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Case {
    /// The observer at rest, its triad along the far-sky axes: `d = n` at every moment.
    Still,
    /// The observer turning at a constant rate about a fixed axis.
    ///
    /// `axis` is a unit vector in the far-sky frame; `degrees_per_second` is the rate of turn,
    /// right-handed about `axis`. At watch time t the triad is the far-sky axes turned by the
    /// angle `degrees_per_second * t` about `axis`, so `d = R(axis, degrees_per_second * t) n`.
    ///
    /// Right-handed about +Z means counter-clockwise seen from above: the forward axis x swings
    /// toward y, which is the observer's left. So a positive rate about +Z turns the observer to
    /// the left, and the sky, which stays put, slides to the right across the observer's frame.
    ///
    /// The rate is kept in the unit the command line takes it in, rather than converted to radians
    /// once, so that the manifest's description of the run quotes the number the person typed.
    Turn {
        axis: [f64; 3],
        degrees_per_second: f64,
    },
}

impl Case {
    /// The case's name, as `--case` takes it and the manifest's `source.name` records it.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Still => "still",
            Self::Turn { .. } => "turn",
        }
    }

    /// The frame numbered `index`, at watch time `t` seconds.
    pub fn frame(&self, grid: Grid, index: u32, t: f64) -> Frame {
        match *self {
            Self::Still => still_frame(grid, index),
            Self::Turn {
                axis,
                degrees_per_second,
            } => turn_frame(grid, index, t, axis, degrees_per_second),
        }
    }

    /// A sentence for the manifest's `observer.triad`, saying how the triad sits at each moment.
    /// It carries the case's parameters, which is also what lets a resumed run check that it is
    /// continuing the same motion: the manifest has no other field to hold them.
    pub fn triad(&self) -> String {
        match *self {
            Self::Still => "the far-sky axes themselves, at every moment: the observer is at \
                            rest and does not turn"
                .into(),
            Self::Turn {
                axis: [x, y, z],
                degrees_per_second,
            } => format!(
                "the far-sky axes turned, at watch time t seconds, by {degrees_per_second} * t \
                 degrees about the far-sky axis ({x}, {y}, {z}), right-handed: a positive rate \
                 about +Z turns the observer to the left"
            ),
        }
    }
}

/// The frame of the observer at rest with its triad along the far-sky axes: every pixel's `d` is
/// its own `n`, rounded to f32. The watch time does not matter, because nothing moves.
pub fn still_frame(grid: Grid, index: u32) -> Frame {
    sky_frame(grid, index, &IDENTITY)
}

/// The frame of the turning observer at watch time `t` seconds: the triad is the far-sky axes
/// turned by `degrees_per_second * t` degrees about `axis` (a unit vector in the far-sky frame,
/// right-handed), so `d = R n`.
///
/// The angle is computed from `t` afresh for each frame rather than accumulated, so frame 900 is
/// as exact as frame 1.
pub fn turn_frame(
    grid: Grid,
    index: u32,
    t: f64,
    axis: [f64; 3],
    degrees_per_second: f64,
) -> Frame {
    sky_frame(
        grid,
        index,
        &rotation(axis, (degrees_per_second * t).to_radians()),
    )
}

/// The rotation by `angle` radians about the unit vector `k`, right-handed (Rodrigues):
///
///     R = cos(angle) I + sin(angle) [k]x + (1 - cos(angle)) k k^T
///
/// where `[k]x v = k x v`. Right-handed means that with the thumb along `k` the fingers curl the
/// way the vectors move: about +Z, +X goes toward +Y.
///
/// At an angle of zero this is the identity to the bit (cos 0 = 1 and sin 0 = 0 exactly), which is
/// why `turn` at t = 0 is `still` exactly and not merely to rounding.
pub fn rotation(k: [f64; 3], angle: f64) -> Matrix {
    let (s, c) = angle.sin_cos();
    let v = 1.0 - c;
    let [x, y, z] = k;
    [
        [c + v * x * x, v * x * y - s * z, v * x * z + s * y],
        [v * y * x + s * z, c + v * y * y, v * y * z - s * x],
        [v * z * x - s * y, v * z * y + s * x, c + v * z * z],
    ]
}

/// `m v`.
pub fn apply(m: &Matrix, v: [f64; 3]) -> [f64; 3] {
    m.map(|row| row[0] * v[0] + row[1] * v[1] + row[2] * v[2])
}

/// A frame in which every ray reaches the far sky at `d = R n`, unshifted and unwound.
///
/// The directions are computed in f64 and rounded once, on the way into the f32 planes, so that
/// the only error a test has to allow for is that one rounding.
fn sky_frame(grid: Grid, index: u32, r: &Matrix) -> Frame {
    let mut frame = Frame::new(grid.width, grid.height, index);
    for j in 0..grid.height {
        for i in 0..grid.width {
            let k = grid.offset(i, j);
            let d = apply(r, grid.pixel_direction(i, j));
            frame.fate[k] = fate::FAR_SKY;
            for (plane, component) in frame.direction.iter_mut().zip(d) {
                plane[k] = component as f32;
            }
            frame.shift[k] = 1.0;
            // `Frame::new` leaves the winding at 0, which is what flat space has.
        }
    }
    frame
}
