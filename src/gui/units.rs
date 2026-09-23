//! Distances and times of a particular hole, written for the screen.
//!
//! These were methods of `KerrSchild` in the geometry crate. How a number is shown is the app's
//! business and not the geometry's - it follows the panel's "Decimal is comma" box, which the
//! geometry has no reason to know exists - so they live here now, as a trait on the metric with
//! the same names, and a call site keeps reading `metric.format_km(km)` with this trait in scope.

use crate::gui::numbers;
use kerr_equatorial::KerrSchild;

/// Kilometres in an astronomical unit, and in a light-year.
const KM_PER_AU: f64 = 1.496e8;
const KM_PER_LY: f64 = 9.461e12;

/// Seconds in a Julian year.
const SECONDS_PER_YEAR: f64 = 86400.0 * 365.25;

/// Physical distances and times of the hole `self` describes, in the current number style.
pub(crate) trait UnitLabels {
    /// A radial coordinate r, in M, as a distance in whichever of km, AU and light-years reads
    /// naturally.
    fn format_physical_distance(&self, r: f64) -> String;

    /// A coordinate time interval, in M, in whichever of µs up to years reads naturally.
    fn format_physical_time(&self, t_in_m: f64) -> String;

    /// A distance already in kilometres, with a k or M in front of the km where it helps.
    fn format_km(&self, km: f64) -> String;

    /// A kilometre grid label that differs from its neighbour `km_step` away in the last place.
    fn format_grid_km(&self, km: f64, km_step: f64) -> String;

    /// A grid label in M that differs from its neighbour `r_step` away in the last place.
    fn format_grid_m(&self, r: f64, r_step: f64) -> String;
}

impl UnitLabels for KerrSchild {
    fn format_physical_distance(&self, r: f64) -> String {
        let km = self.r_to_km(r);
        let au = km / KM_PER_AU;
        let ly = km / KM_PER_LY;
        if ly >= 0.01 {
            format!("{} ly ({} AU)", numbers::fixed(ly, 2), numbers::fixed(au, 0))
        } else if au >= 0.05 {
            format!("{} AU ({} km)", numbers::fixed(au, 1), numbers::fixed(km, 0))
        } else if km >= 1e6 {
            format!("{} M km", numbers::fixed(km / 1e6, 2))
        } else {
            format!("{} km", numbers::fixed(km, 1))
        }
    }

    fn format_physical_time(&self, t_in_m: f64) -> String {
        let secs = (t_in_m / self.m).abs() * self.t_grav_seconds();
        let sign = if t_in_m < 0.0 { "-" } else { "" };
        let (value, unit) = if secs >= SECONDS_PER_YEAR {
            (secs / SECONDS_PER_YEAR, "yr")
        } else if secs >= 86400.0 {
            (secs / 86400.0, "days")
        } else if secs >= 3600.0 {
            (secs / 3600.0, "hrs")
        } else if secs >= 60.0 {
            (secs / 60.0, "min")
        } else if secs >= 1.0 {
            (secs, "s")
        } else if secs >= 1e-3 {
            (secs * 1e3, "ms")
        } else {
            (secs * 1e6, "µs")
        };
        format!("{sign}{} {unit}", numbers::fixed(value, 2))
    }

    fn format_km(&self, km: f64) -> String {
        if km > 1e9 {
            format!("{} km", numbers::exponent(km, 2))
        } else if km >= 1e6 {
            format!("{}M km", numbers::fixed(km / 1e6, 2))
        } else if km >= 1e3 {
            format!("{}k km", numbers::fixed(km / 1e3, 1))
        } else if km >= 10.0 {
            format!("{} km", numbers::fixed(km, 1))
        } else {
            format!("{} km", numbers::fixed(km, 2))
        }
    }

    fn format_grid_km(&self, km: f64, km_step: f64) -> String {
        let km = if km.abs() < 1e-12 { 0.0 } else { km };
        let km_step = km_step.max(1e-9);
        let (unit, suffix) = if km >= 1e9 && km_step >= 1e6 {
            (1e9, "B km")
        } else if km >= 1e6 && km_step >= 1e3 {
            (1e6, "M km")
        } else if km >= 1e3 && km_step >= 1.0 {
            (1e3, "k km")
        } else {
            (1.0, " km")
        };
        format!("{}{suffix}", numbers::exact(km / unit, grid_decimals(km_step / unit)))
    }

    fn format_grid_m(&self, r: f64, r_step: f64) -> String {
        let r = if r.abs() < 1e-12 { 0.0 } else { r };
        format!("{}M", numbers::exact(r, grid_decimals(r_step.max(1e-9))))
    }
}

/// How many decimals make a label change between two ticks `step` apart.
fn grid_decimals(step: f64) -> usize {
    if step >= 0.999 { 0 } else { (-step.log10()).ceil().max(1.0) as usize }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physical_units_conversion() {
        // Solar mass: 1 M_sun gives r_g ~ 1.477 km and t_g ~ 4.93 µs.
        let ks_sun = KerrSchild::with_solar_mass(1.0, 0.0, 1.0);
        assert_eq!(ks_sun.format_physical_distance(1.0), "1.5 km");
        assert!(ks_sun.format_physical_time(1.0).ends_with("µs"));

        // Sagittarius A*: 4.15e6 M_sun gives t_g ~ 20.4 s.
        let ks_sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        assert_eq!(ks_sgr.format_physical_time(1.0), "20.45 s");
    }

    #[test]
    fn test_grid_labels_consecutive_digits_change() {
        let ks_sgr = KerrSchild::with_solar_mass(1.0, 0.9, 4.15e6);
        // Sgr A* at a 1000 km step around 6.13 M km, and at a 100 km step.
        let base_km = 6_130_000.0;
        for step_km in [1000.0, 100.0] {
            let l1 = ks_sgr.format_grid_km(base_km, step_km);
            let l2 = ks_sgr.format_grid_km(base_km + step_km, step_km);
            assert_ne!(l1, l2, "labels must differ for {step_km} km grid lines: {l1} vs {l2}");
        }
        // M units at a fine step near the Cauchy horizon, and at an extreme zoom.
        let r_base = 0.4358;
        for r_step in [0.0001, 0.00002] {
            let m1 = ks_sgr.format_grid_m(r_base, r_step);
            let m2 = ks_sgr.format_grid_m(r_base + r_step, r_step);
            assert_ne!(m1, m2, "labels must differ for {r_step} M grid lines: {m1} vs {m2}");
        }
    }

    #[test]
    fn test_large_values_are_grouped_and_follow_the_style() {
        let ks = KerrSchild::with_solar_mass(1.0, 0.9, 1e8);
        assert_eq!(ks.format_grid_km(12_345.6, 0.1), "12,345.6 km");
        assert_eq!(ks.format_physical_distance(1.0), "1.0 AU (147,700,000 km)");
        numbers::set_style(numbers::style_for(true));
        assert_eq!(ks.format_grid_km(12_345.6, 0.1), "12.345,6 km");
        assert_eq!(ks.format_physical_distance(1.0), "1,0 AU (147.700.000 km)");
        numbers::set_style(numbers::style_for(false));
    }
}
