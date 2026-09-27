//! The observer-sky grid: which direction each pixel of a frame looks along, and back.
//!
//! The grid is equirectangular, `width` columns by `height` rows, with the samples at pixel
//! centres. Column i, row j has longitude lambda and latitude beta
//!
//!     lambda = ((i + 0.5) / W - 0.5) * 2 pi
//!     beta   = (0.5 - (j + 0.5) / H) * pi
//!
//! and looks along the unit vector, in the observer's triad (x, y, z),
//!
//!     phi = -lambda
//!     n   = (cos beta cos phi, cos beta sin phi, sin beta)
//!
//! Longitude increases to the right of the frame and the azimuth phi, measured from x toward y,
//! increases to the left. The sign flip is the whole convention: x is forward, z is up and y is
//! the viewer's left, so turning right is turning toward -y, and a 360-degree player that puts the
//! frame's centre ahead and its right-hand side to the right shows the sky the way the observer
//! sees it. It is also how a star map is laid out when the sky is seen from inside.
//!
//! The formulae take frame coordinates (u, v): u runs from 0 at the frame's left edge to W at its
//! right, v from 0 at the top edge to H at the bottom, and the centre of pixel (i, j) is at
//! (i + 0.5, j + 0.5). So the same two functions serve a writer that wants the direction of pixel
//! (3, 1) and a renderer that wants the place in the frame under a direction it is about to
//! interpolate at. They are the coordinates the star maps are described in (assets/sky/README.md),
//! and a second convention with the centres on the integers would be a half-pixel error waiting
//! for whoever carried a number from one to the other.

use std::f64::consts::PI;

/// The size of an observer-sky grid, and the formulae that go with it. The manifest's
/// `GridSpec::grid` returns one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Grid {
    pub width: u32,
    pub height: u32,
}

impl Grid {
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    /// How many rays a frame of this grid holds.
    pub const fn len(self) -> usize {
        self.width as usize * self.height as usize
    }

    /// True for a grid with no rays, which no valid manifest declares.
    pub const fn is_empty(self) -> bool {
        self.len() == 0
    }

    /// Where pixel (i, j) sits in each plane of a frame: rows top to bottom, each row left to
    /// right.
    pub const fn offset(self, i: u32, j: u32) -> usize {
        j as usize * self.width as usize + i as usize
    }

    /// Longitude lambda and latitude beta, in radians, at frame coordinates (u, v): the centre of
    /// pixel (i, j) is at u = i + 0.5, v = j + 0.5.
    pub fn angles(self, u: f64, v: f64) -> (f64, f64) {
        let lambda = (u / f64::from(self.width) - 0.5) * 2.0 * PI;
        let beta = (0.5 - v / f64::from(self.height)) * PI;
        (lambda, beta)
    }

    /// The unit vector, in the observer's triad, that the observer looks along at frame
    /// coordinates (u, v). The light seen there arrives travelling along the opposite vector.
    pub fn direction(self, u: f64, v: f64) -> [f64; 3] {
        let (lambda, beta) = self.angles(u, v);
        let phi = -lambda;
        let (sin_beta, cos_beta) = beta.sin_cos();
        let (sin_phi, cos_phi) = phi.sin_cos();
        [cos_beta * cos_phi, cos_beta * sin_phi, sin_beta]
    }

    /// The direction of the centre of pixel (i, j).
    pub fn pixel_direction(self, i: u32, j: u32) -> [f64; 3] {
        self.direction(f64::from(i) + 0.5, f64::from(j) + 0.5)
    }

    /// The frame coordinates (u, v) that look along `n`, the inverse of [`Grid::direction`].
    ///
    /// `n` need not be normalised but must not be zero. u lies in [0, W): the seam directly
    /// behind the observer, phi = pi, comes out at the left edge, u = 0, and a renderer that
    /// samples across the seam wraps u itself. v lies in [0, H], the two ends being the poles,
    /// which no pixel centre reaches.
    pub fn frame_coordinates(self, n: [f64; 3]) -> (f64, f64) {
        let [nx, ny, nz] = n;
        // atan2 against the horizontal length rather than asin of nz: accurate near the poles,
        // where asin loses half its digits, and indifferent to the length of n.
        let beta = nz.atan2(nx.hypot(ny));
        let phi = ny.atan2(nx);
        let lambda = -phi;
        let u = (lambda / (2.0 * PI) + 0.5) * f64::from(self.width);
        let v = (0.5 - beta / PI) * f64::from(self.height);
        (u, v)
    }
}
