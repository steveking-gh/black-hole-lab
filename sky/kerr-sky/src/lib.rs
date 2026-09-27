//! Light in Kerr spacetime off the equatorial plane, traced backward from the eye of an observer who
//! stands on it: what the observer sees over the whole sky.
//!
//! Black Hole Lab's geometry core, `kerr_equatorial`, lives on the equatorial plane, where a ray
//! that starts in the plane stays. A sky is made of rays that leave it. This crate follows them in
//! the Cartesian form of the same chart - ingoing Kerr-Schild coordinates (t, x, y, z), whose
//! restriction to z = 0 is the app's (t, r, phi) by the app's own embedding x + iy = (r + ia) e^{i
//! phi} - and says where each came from.
//!
//! - [`metric`]: the metric g = eta + 2 H l l in Cartesian coordinates, its agreement with
//!   `kerr_equatorial` on the plane, the constants of motion E, L_z and Carter's Q, and Hamilton's
//!   equations for light.
//! - [`observer`]: an observer's event and 4-velocity as the app carries them, and the orthonormal
//!   triad on their sky.
//! - [`ray`]: one ray traced backward - Dormand-Prince 5(4) in the affine parameter - to its fate:
//!   the far sky (with the direction at infinity, the shift and the winding), the past horizon, or
//!   unresolved.
//! - [`frame`]: an equirectangular grid of rays for one observer, on scoped threads, using the
//!   reflection symmetry of the equatorial plane.
//!
//! Geometric units throughout: G = c = 1, lengths and times in units of M. The definitions of the
//! triad, the grid, the direction at infinity d, the shift g, the fate codes and the winding are
//! those of the sky bundle specification (`physics_simulation_specification.md`, sections 4.2 to
//! 4.8); this crate knows nothing of the files themselves.
//!
//! Scope: the observer is outside the outer horizon or between the horizons (r > r-). Outside r+
//! every ray traced backward either reaches the far sky or came out of the past horizon. Between
//! the horizons there is a third possibility, which the plan did not list: light with
//! E - Omega_H L_z < 0 never crossed r+ from this universe at all, and traced backward it closes
//! on the other branch of r+ (in the eternal hole, from the other exterior; in a hole formed by
//! collapse, from the collapsing star). It is given fate 2, the dark part of the sky, with a
//! `ray::Decision` of its own; see `ray`. Inside r- light also arrives through the ring from
//! negative r and from the other sheet of the inner horizon; those need fates the format does not
//! have yet, and an observer there is refused with [`observer::ScopeError`].

pub mod frame;
pub mod metric;
pub mod observer;
pub mod ray;

pub use frame::{SkyFrame, trace_frame};
pub use metric::{Constants, Kerr};
pub use observer::{Observer, ScopeError, Triad};
pub use ray::{Fate, Outcome, TraceOptions, trace_covector, trace_direction};
