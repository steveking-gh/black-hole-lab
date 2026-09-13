# To do

Agreed follow-ups, in the order I would take them.

1. Draw each front segment as an arc interpolated linearly in (r, φ), subdivided before
   embedding, so chords between frozen rays never dip inside the r₋ circle. Matches the
   interpolation the reception detector already uses.
2. Remove the frame-dragging streamlines feature and its checkbox.
3. Add a test that an emitter stops transmitting once it reaches the ring (the code does this;
   nothing asserts it).
4. Label region III (inside r₋) honestly: distinct shading, a legend line and a Theory Guide
   sentence saying it is Kerr's analytic continuation, not determined by exterior data, and
   expected to be replaced by mass inflation in a real collapse. CTCs live at r < 0, which the
   app never enters.
5. Tighten the unit constants in kerr_schild.rs: derive GM☉/c² and GM☉/c³ from
   GM☉ = 1.32712440018e20 m³/s² and c = 299792458 m/s (1476.625 m, 4.925491e-6 s); AU and
   light-year to IAU values; tidal-force display to use the same source. Display and
   Distance-mode step only.

Known limitations, documented in code, not scheduled:

- A hovering observer's `Observer::four_velocity` reports the free-fall value; the signal code
  corrects for it locally. Telemetry, cones and the Distance-step estimate still read it.
- The reception detector keys a sheet by polyline-segment index; at coarse steps a fast-winding
  front can hand a sheet to a different segment between passes and a crossing is missed.
- Frozen-family classification is recomputed per ray per frame; cache at emission if it matters.

Ideas offered, not requested: a Penrose-diagram inset; merging and pushing `gr-fidelity`.
