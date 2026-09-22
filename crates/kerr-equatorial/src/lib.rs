//! Kerr spacetime restricted to the equatorial plane (theta = pi/2), in ingoing Kerr-Schild
//! coordinates (t, r, phi).
//!
//! This is a slice of the full 3+1 Kerr geometry and not a lower-dimensional gravity model: every
//! quantity here is the four-dimensional one evaluated on the plane, where a worldline that starts
//! in the plane with no polar velocity stays for good. The chart is horizon-penetrating, so the
//! metric, the geodesics and the frames are all regular across the outer horizon r+ and the inner
//! horizon r-. Geometric units throughout: G = c = 1, lengths and times in units of M.
//!
//! - [`kerr_schild`]: the metric and what follows from it alone - horizons, ergosphere, circular
//!   orbits, frame dragging, Christoffel symbols, the principal null directions, curvature.
//! - [`geodesic`]: timelike geodesics, integrated in proper time or in coordinate time, and
//!   stoppable on an exact reading of the worldline's own clock.
//! - [`tetrad`]: the orthonormal frame carried by a 4-velocity.
//! - [`local_frame`]: an observer's local inertial chart, and the coordinate surfaces drawn in it.
//! - [`normal_coords`]: the same chart to all orders - Riemann normal coordinates about the
//!   observer's event, so that a surface is drawn where the geodesics actually meet it.
//!
//! It is the physics core of Black Hole Lab and knows nothing of the app: no drawing, no UI, no
//! simulation clock. Observers with modes and trails, and the signals they exchange, live in the
//! app's own `physics` module on top of this.

pub mod geodesic;
pub mod kerr_schild;
pub mod local_frame;
pub mod normal_coords;
pub mod tetrad;

pub use geodesic::GeodesicState;
pub use kerr_schild::KerrSchild;
pub use local_frame::{LocalFrame, LocalLine, SurfaceCharacter};
pub use normal_coords::{
    HorizonBranch, RadialConstants, SurfacePoint, SurfaceSampling, affine_length_between,
    affine_length_to_surface, sample_surface, sample_surface_in_frame, sample_surface_in_plane,
    surface_point_in_frame,
};
pub use tetrad::Tetrad;
