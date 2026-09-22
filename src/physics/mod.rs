// The geometry core lives in the `kerr-equatorial` crate (crates/kerr-equatorial). Its modules are
// re-exported under their old names so that `crate::physics::kerr_schild::KerrSchild` and the rest
// read exactly as they did when the files sat in this directory.
pub use kerr_equatorial::{geodesic, kerr_schild, local_frame, tetrad};
// The exact normal-coordinate chart, re-exported under the app's own path like the rest of the
// geometry core. The rest-frame view draws its surfaces from it and `as_seen` takes the affine
// length of an arriving ray from it.
pub use kerr_equatorial::normal_coords;
pub mod as_seen;
pub mod observer;
pub mod wavefront;
pub mod simulation;

#[allow(unused_imports)]
pub use kerr_schild::KerrSchild;
#[allow(unused_imports)]
pub use observer::Observer;
#[allow(unused_imports)]
pub use tetrad::Tetrad;
#[allow(unused_imports)]
pub use local_frame::{LocalFrame, LocalLine, SurfaceCharacter};
#[allow(unused_imports)]
pub use wavefront::{NullRay, Pulse, Reception, SignalField};
