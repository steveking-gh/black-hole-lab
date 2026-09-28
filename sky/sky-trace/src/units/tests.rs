//! The unit ladders, the decimals, and the holes the app offers.

use super::*;
use crate::bhl::GM_SUN_OVER_C3_SECONDS;

/// Seconds in one M of a hole of `solar_masses`.
fn seconds(solar_masses: f64) -> f64 {
    solar_masses * GM_SUN_OVER_C3_SECONDS
}

fn shown(display: &Display) -> (&str, u32) {
    (display.unit.as_str(), display.decimals)
}

#[test]
fn test_four_significant_figures_at_most_three_decimals_and_at_least_none() {
    for (magnitude, want) in [
        (0.0, 3),
        (0.5, 3),
        (9.99, 3),
        (9.999_999, 3),
        (10.0, 2),
        (13.89, 2),
        (99.99, 2),
        (100.0, 1),
        (188.2, 1),
        (999.9, 1),
        (1000.0, 0),
        (1234.0, 0),
        (6.6e10, 0),
    ] {
        assert_eq!(
            decimals_for(magnitude),
            want,
            "{magnitude} should be shown to {want} decimals"
        );
    }
}

#[test]
fn test_the_time_ladder_is_the_apps_with_its_thresholds() {
    // One solar mass lasts 4.93 microseconds an M; each rung starts where the app's does.
    let s = seconds(1.0);
    for (largest_seconds, unit) in [
        (5e-4, "µs"),
        (1e-3, "ms"),
        (0.999, "ms"),
        (1.0, "s"),
        (59.9, "s"),
        (60.0, "min"),
        (3599.0, "min"),
        (3600.0, "hrs"),
        (86_399.0, "hrs"),
        (86_400.0, "days"),
        (SECONDS_PER_YEAR * 0.999, "days"),
        (SECONDS_PER_YEAR, "yr"),
        (1e12, "yr"),
    ] {
        let display = time_display(largest_seconds / s, s);
        assert_eq!(
            display.unit, unit,
            "{largest_seconds} s should be shown in {unit}"
        );
        // The scale turns M into the unit: the largest value, displayed, is the seconds in it.
        let unit_seconds = TIME_LADDER.iter().find(|(u, _)| *u == unit).unwrap().1;
        let back = largest_seconds / s * display.scale.0;
        assert!(
            (back - largest_seconds / unit_seconds).abs() <= 1e-12 * back,
            "{largest_seconds} s in {unit}: {back}"
        );
    }
}

#[test]
fn test_the_length_ladder_starts_each_rung_at_one_of_its_unit() {
    // m, km, million km, billion km, ly: each from one of itself, so that every reading at or
    // above its unit has at least four significant figures. A hole of a hundredth of a solar
    // mass, so that the floor of 1 M (14.8 m) lets every rung be reached.
    let s = seconds(0.01);
    let km_per_m = s * C_KM_PER_S;
    for (largest_km, unit, decimals) in [
        (0.02, "m", 2),
        (0.999, "m", 1),
        (1.0, "km", 3),
        (999_999.0, "km", 0),
        (1e6, "million km", 3),
        (999e6, "million km", 1),
        (1e9, "billion km", 3),
        (0.999 * KM_PER_LY, "billion km", 0),
        (KM_PER_LY, "ly", 3),
        (1e18, "ly", 0),
    ] {
        // A hair above, so that a reading on a rung's threshold is not a rounding below it on
        // the way from km to M and back.
        let largest = largest_km / km_per_m * (1.0 + 1e-12);
        let display = length_display(largest, s);
        assert_eq!(
            (display.unit.as_str(), display.decimals),
            (unit, decimals),
            "{largest_km} km should be shown in {unit} to {decimals} decimals"
        );
        let shown = largest * display.scale.0;
        assert!(
            (1.0..1e7).contains(&(shown * 1.000_000_1)),
            "{largest_km} km shows as {shown} {unit}"
        );
    }
}

#[test]
fn test_a_radius_below_a_kilometre_is_shown_in_metres_only_where_one_m_is_below_one() {
    // A hole of 1 solar mass: 1 M is 1.477 km, so the unit is never chosen from less than that,
    // and a radius of half an M, 738 m, reads 0.738 km. A hole of half a solar mass: 1 M is 738 m,
    // and a film that never leaves 1 M reads in metres.
    let one = length_display(0.5, seconds(1.0));
    assert_eq!((one.unit.as_str(), one.decimals), ("km", 3));
    assert_eq!(format!("{:.3}", 0.5 * one.scale.0), "0.738");
    let half = length_display(1.0, seconds(0.5));
    assert_eq!((half.unit.as_str(), half.decimals), ("m", 1));
    assert_eq!(format!("{:.1}", half.scale.0), "738.3");
}

#[test]
fn test_a_reading_of_zero_is_shown_in_the_unit_of_one_m() {
    // A still's stopwatch reads 0 and is shown as one M of time would be, not in microseconds;
    // likewise a radius or a clock that never exceeds 1 M.
    for solar_masses in [10.0, 4.15e6, 1e8, 6.6e10] {
        let s = seconds(solar_masses);
        for small in [0.0, -0.0, 0.25, -1.0] {
            assert_eq!(
                time_display(small, s),
                time_display(1.0, s),
                "{solar_masses}: {small}"
            );
            assert_eq!(
                length_display(small, s),
                length_display(1.0, s),
                "{solar_masses}: {small}"
            );
        }
    }
}

#[test]
fn test_the_smallest_and_largest_holes_the_app_offers() {
    // 10 solar masses, Sagittarius A*, 1e8 and TON 618, each at 1 M and at 1000 M of time and of
    // radius. Worked by hand from GM_sun / c^3 = 4.925490947e-6 s and c = 299 792.458 km/s:
    //
    //   10:      1 M = 49.25 µs = 14.77 km;             1000 M = 49.25 ms = 14 766 km
    //   Sgr A*:  1 M = 20.44 s = 6.128 million km;      1000 M = 5.678 hrs = 6.128 billion km
    //   1e8:     1 M = 8.209 min = 147.7 million km;    1000 M = 5.701 days = 147.7 billion km
    //   TON 618: 1 M = 3.763 days = 97.46 billion km;   1000 M = 10.30 yr = 10.30 ly
    // Solar masses, the largest magnitude in M, and the unit and decimals of time and of length.
    type Case = (f64, f64, (&'static str, u32), (&'static str, u32));
    let cases: [Case; 8] = [
        (10.0, 1.0, ("µs", 2), ("km", 2)),
        (10.0, 1000.0, ("ms", 2), ("km", 0)),
        (4.15e6, 1.0, ("s", 2), ("million km", 3)),
        (4.15e6, 1000.0, ("hrs", 3), ("billion km", 3)),
        (1e8, 1.0, ("min", 3), ("million km", 1)),
        (1e8, 1000.0, ("days", 3), ("billion km", 1)),
        (6.6e10, 1.0, ("days", 3), ("billion km", 2)),
        (6.6e10, 1000.0, ("yr", 2), ("ly", 2)),
    ];
    for (solar_masses, largest, time, length) in cases {
        let s = seconds(solar_masses);
        let (t, l) = (time_display(largest, s), length_display(largest, s));
        println!(
            "{solar_masses:e} solar masses, {largest} M: {:.*} {} and {:.*} {}",
            t.decimals as usize,
            largest * t.scale.0,
            t.unit,
            l.decimals as usize,
            largest * l.scale.0,
            l.unit
        );
        assert_eq!(shown(&t), time, "{solar_masses}, {largest} M of time");
        assert_eq!(shown(&l), length, "{solar_masses}, {largest} M of radius");
    }
    // The values themselves, at Sagittarius A*, which the app's own test of its units has too.
    let s = seconds(4.15e6);
    assert_eq!(format!("{:.2}", time_display(1.0, s).scale.0), "20.44");
    assert_eq!(format!("{:.3}", length_display(1.0, s).scale.0), "6.128");
}

#[test]
fn test_the_second_and_the_kilometre_come_from_one_number() {
    // A radius of 1 M is c times the duration of 1 M: the kilometre display of a hole whose radius
    // is shown in km, divided by c, is the seconds an M lasts, to a rounding; and a unit of the
    // time ladder, times c, is that much light-travel distance.
    for solar_masses in [1.0, 10.0, 4.15e6, 1e8, 6.6e10] {
        let s = seconds(solar_masses);
        let km_per_m = length_display(1e-30, s);
        let per_km = match km_per_m.unit.as_str() {
            "km" => 1.0,
            "million km" => 1e6,
            "billion km" => 1e9,
            "ly" => KM_PER_LY,
            other => panic!("{other}"),
        };
        let seconds_per_m = km_per_m.scale.0 * per_km / C_KM_PER_S;
        assert!(
            (seconds_per_m - s).abs() <= 4.0 * f64::EPSILON * s,
            "{solar_masses}: {seconds_per_m} s against {s} s"
        );
    }
    // The light-year is a Julian year of light.
    assert!((KM_PER_LY - C_KM_PER_S * SECONDS_PER_YEAR).abs() <= f64::EPSILON * KM_PER_LY);
    // In integers, exactly: 299 792 458 m/s for 31 557 600 s.
    assert_eq!(299_792_458u64 * 31_557_600, 9_460_730_472_580_800);
    assert_eq!(format!("{KM_PER_LY}"), "9460730472580.8");
}
