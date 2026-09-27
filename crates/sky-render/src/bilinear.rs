//! Bilinear interpolation on an equirectangular grid, across its seam and over its poles.
//!
//! Two grids in this program are equirectangular: the bundle's grid of rays and every level of the
//! star map. Both are laid out as the specification's section 4.3 says, in frame coordinates
//! (u, v) with the centre of cell (i, j) at (i + 0.5, j + 0.5), and both are read between their
//! cells by the one function here, so that the two cannot disagree about where a cell is.
//!
//! **The seam.** Column W - 1 and column 0 are neighbours on the sphere: the left and right edges
//! of the frame are the same meridian. So a column index is taken modulo W, and a place between
//! the last column's centre and the right edge reads column W - 1 and column 0.
//!
//! **The poles.** Above the first row's centres (v < 0.5) there is no row of the grid on the far
//! side, and the tempting answers are both wrong. Clamping to row 0 flattens the top half-row, so
//! that everything within half a row of the pole shows the value of row 0 instead of approaching
//! the pole's own value. Reading "row -1" as the last row wraps the top of the sky onto the bottom.
//!
//! The rows really on the far side are the grid's own first row, half a turn away. A great circle
//! through the pole at longitude lambda comes out on the other side at longitude lambda + pi; walk
//! it past the pole and the latitude starts to fall again. So in frame coordinates the point
//! (u, -v) is the point (u + W/2, v): the reflection of the band 0 < v < 0.5 through the pole is
//! the same band half the frame away. The "row -1" that interpolation needs, centred at v = -0.5,
//! is therefore row 0 read at u + W/2, and the weight between the two rows is the ordinary linear
//! one. At the pole itself (v = 0) the weights are one half each: the value there is the mean of
//! row 0 at u and at u + W/2, which is what makes it one value whatever u is. The bottom pole is
//! the same with row H - 1. When W is odd, u + W/2 falls between column centres and is itself
//! interpolated, like any other u.
//!
//! For the rays this is what makes a coarse grid of directions give the right direction near the
//! pole: the direction vectors of row 0 at u and at u + W/2 have opposite horizontal parts, and
//! their weighted sum, renormalised, climbs smoothly to the pole and over it.

/// The four cells bilinear interpolation reads, with their weights. The weights are
/// non-negative and add up to one. Two entries may name the same cell, when the grid is one
/// column wide or a place is read across a pole of a grid one row high.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Taps {
    /// Offsets of the cells in row-major order: row j, column i is at `j * width + i`.
    pub index: [usize; 4],
    pub weight: [f64; 4],
}

impl Taps {
    /// The entry with the largest weight, which is the cell whose centre is nearest (u, v) in
    /// frame coordinates. A tie goes to the earlier entry, so the choice is repeatable.
    pub fn nearest(&self) -> usize {
        let mut best = 0;
        for k in 1..4 {
            if self.weight[k] > self.weight[best] {
                best = k;
            }
        }
        best
    }
}

/// The taps at frame coordinates (u, v) of a grid `width` columns by `height` rows. Any u is
/// accepted and wrapped; v is clamped to [0, height], the two poles.
pub fn taps(width: usize, height: usize, u: f64, v: f64) -> Taps {
    debug_assert!(width > 0 && height > 0);
    let half_turn = width as f64 / 2.0;
    let v = v.clamp(0.0, height as f64);
    let y = v - 0.5;
    let top = y.floor();
    let fy = y - top;
    let top = top as isize;
    let mut index = [0; 4];
    let mut weight = [0.0; 4];
    for (k, (row, wy)) in [(top, 1.0 - fy), (top + 1, fy)].into_iter().enumerate() {
        // Past a pole: the same row, half a turn round (see the module comment).
        let (row, u) = if row < 0 {
            (0, u + half_turn)
        } else if row >= height as isize {
            (height - 1, u + half_turn)
        } else {
            (row as usize, u)
        };
        let x = u - 0.5;
        let left = x.floor();
        let fx = x - left;
        // Wrapped by a compare rather than `rem_euclid`, whose integer division is a large share
        // of a lookup's cost; u within a turn either side of the frame is all this ever sees.
        let left = left as i64;
        let w = width as i64;
        let left = if (0..w).contains(&left) {
            left
        } else if (-w..0).contains(&left) {
            left + w
        } else if (w..2 * w).contains(&left) {
            left - w
        } else {
            left.rem_euclid(w)
        } as usize;
        let right = if left + 1 == width { 0 } else { left + 1 };
        index[2 * k] = row * width + left;
        weight[2 * k] = wy * (1.0 - fx);
        index[2 * k + 1] = row * width + right;
        weight[2 * k + 1] = wy * fx;
    }
    Taps { index, weight }
}
