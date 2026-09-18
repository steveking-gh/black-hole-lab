//! How one number in the performance report is arrived at: the timing loop, the calibration that
//! makes a sample long enough to mean anything, and the three statistics every benchmark reports.
//!
//! Nothing here knows what it is timing. A benchmark is a closure that is handed an iteration count
//! and gives back the time *it* decided to put on the clock, which is what lets one loop serve two
//! kinds of work that would otherwise need two harnesses. A pure kernel puts the whole loop inside
//! the timed span and measures every iteration of it; an operation that mutates state - advancing a
//! signal field, painting a canvas - rebuilds or clones its state first, outside the span, and
//! starts the clock only when the operation itself begins. The harness never sees the difference,
//! and neither statistic is contaminated by set-up.
//!
//! **Why a sample is a batch.** `Instant` on this machine resolves to about 100 ns, and
//! `geodesic_accel` takes a few tens of them, so timing one call measures the clock rather than the
//! kernel. Every benchmark is therefore calibrated to an inner iteration count large enough that
//! one sample spans at least `Budget::min_sample` (5 ms by default, ~50 000 clock ticks), and the
//! figure reported is the span divided by that count. A kernel that already takes longer than the
//! floor runs one iteration per sample and the division is by one.
//!
//! **Why the median and not the mean.** A run competing with the rest of the machine produces a
//! one-sided distribution: nothing can make an iteration faster than the hardware allows, and
//! anything at all can make one slower. The mean follows the tail, the median follows the mode, and
//! the minimum is the cleanest single figure the machine offered - so all three are kept and the
//! comparison runs on the median, with the spread as the statement of how much to trust it.
//!
//! **What the spread is.** The median absolute deviation over the median, scaled by 1.4826 so that
//! it reads as a relative standard deviation for a normal distribution. It is used rather than the
//! standard deviation because one descheduled sample moves a standard deviation by tens of percent
//! and moves this by nothing. A quiet machine gives 0.5% to 2% here; anything above about 10% means
//! the number in the same row is not worth acting on, and the compare step widens its noise band by
//! exactly this figure so that a wide spread makes a regression harder to claim rather than easier.

use std::time::{Duration, Instant};

/// How much time a benchmark is allowed to spend on itself.
///
/// The presets are a measurement and a cheaper measurement, and the difference between them is how
/// small a change they can resolve rather than whether they can resolve one at all. `full` takes 30
/// samples of 5 ms, which puts a quiet benchmark's spread at 0.5% to 2% and makes a 5% gate
/// workable. `quick` takes 15 samples at the same floor, half the wall time, and is what the quick
/// tier's 10% gate is set against.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Budget {
    /// How many samples to take once the iteration count is settled.
    pub samples: usize,
    /// The floor one sample has to reach, which is what the iteration count is calibrated against.
    pub min_sample: Duration,
    /// How long to run the calibrated batch before any sample is kept, so that the first sample is
    /// not the one that paid for the branch predictor and the caches.
    pub warmup: Duration,
}

impl Budget {
    /// The measurement: 30 samples of at least 5 ms, which is a fifth of a second per benchmark
    /// plus warm-up.
    pub fn full() -> Self {
        Self { samples: 30, min_sample: Duration::from_millis(5), warmup: Duration::from_millis(20) }
    }

    /// The everyday check: half the samples of a full run, at the same floor.
    ///
    /// The floor is deliberately not lowered with the sample count. `paint/empty-pass` builds a
    /// fresh `egui::Context` per call and so costs about 2.4 ms on its first pass and 2 µs on every
    /// pass after it - a benchmark whose cost depends on its iteration count, which the calibration
    /// warns about. Its first probe therefore lands just under a 5 ms floor and the search goes on
    /// to sixteen hundred iterations, and lands just *over* a 3 ms one perhaps one run in ten, where
    /// the search stops at one iteration and the benchmark reports the first pass - 243 µs, a
    /// hundredfold - as its per-iteration cost. The two numbers are both "correct" and a baseline
    /// that caught one against a check that caught the other is a 99% row in the compare table.
    /// Sharing the full suite's floor is what makes quick and full calibrate identically, so the
    /// only difference between them is how many samples are kept.
    pub fn quick() -> Self {
        Self { samples: 15, min_sample: Duration::from_millis(5), warmup: Duration::from_millis(10) }
    }

    /// The same for the three paint benchmarks, which are the expensive ones.
    ///
    /// A canvas paint is already several milliseconds, so it runs one iteration per sample whatever
    /// the floor says and the sample count *is* the cost: fifteen of the volume canvas is a third of
    /// the quick tier's whole time budget. Ten is what keeps all three inside about half a second
    /// between them, and a paint's own spread is narrow enough that the five samples buy little.
    pub fn quick_paint() -> Self {
        Self { samples: 10, ..Self::quick() }
    }
}

/// What one benchmark measured, per iteration of the thing being benchmarked.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub(crate) struct Stats {
    /// The fastest per-iteration time seen: the machine at its least disturbed.
    pub min_ns: f64,
    /// The middle per-iteration time, which is what `--compare` runs on.
    pub median_ns: f64,
    /// Scaled median absolute deviation over the median, as a fraction: the relative spread.
    pub spread: f64,
    /// Iterations of the benchmarked operation inside one sample.
    pub iters: u64,
    /// Samples kept.
    pub samples: usize,
}

/// A benchmark: a closure handed an iteration count, which runs that many iterations of whatever it
/// measures and returns the time it put on the clock for them.
///
/// The closure owning the clock is the whole design. It may do as much set-up as it likes before
/// calling `Instant::now`, which is what makes a benchmark of `SignalField::advance` - an operation
/// that consumes the state it is given - measurable at all: it clones a prebuilt field per call and
/// starts timing after the clone.
pub(crate) type Body<'a> = Box<dyn FnMut(u64) -> Duration + 'a>;

/// One registered benchmark: the name `--filter` and `--compare` match on, the one line the report
/// prints to say what is being measured, and the closure that measures it.
pub(crate) struct Bench<'a> {
    pub name: String,
    /// What this benchmark measures, and - where it matters - what it deliberately does not. Printed
    /// by `--list`.
    pub what: &'static str,
    pub body: Body<'a>,
}

/// Run one benchmark: calibrate, warm up, sample.
///
/// The calibration is a fixed-point search rather than a doubling, because the iteration counts
/// involved span seven orders of magnitude - 2.5 million iterations of `geodesic_accel` to one of a
/// canvas paint - and doubling from one would spend most of a benchmark's budget getting there. One
/// probe measures the operation, the count that would fill `min_sample` is computed from it with
/// 20% of headroom, and the probe is repeated until the batch is long enough. The loop is bounded
/// because a benchmark whose cost depends on its iteration count would otherwise never converge;
/// hitting the bound costs accuracy and nothing else.
pub(crate) fn measure(body: &mut dyn FnMut(u64) -> Duration, budget: &Budget) -> Stats {
    let floor = budget.min_sample.as_secs_f64();
    let mut iters: u64 = 1;
    let mut probe = body(iters).as_secs_f64();
    for _ in 0..24 {
        if probe >= floor {
            break;
        }
        // A probe that came back at or below the clock's own resolution says nothing about how many
        // iterations are needed, so the count is simply multiplied out until it does.
        let next = if probe > 1e-7 {
            ((iters as f64) * floor / probe * 1.2).ceil() as u64
        } else {
            iters.saturating_mul(64)
        };
        let next = next.clamp(iters + 1, iters.saturating_mul(256));
        iters = next;
        probe = body(iters).as_secs_f64();
    }

    let warm = Instant::now();
    loop {
        body(iters);
        if warm.elapsed() >= budget.warmup {
            break;
        }
    }

    let mut per_iter = Vec::with_capacity(budget.samples);
    for _ in 0..budget.samples {
        let elapsed = body(iters).as_secs_f64();
        per_iter.push(elapsed * 1e9 / iters as f64);
    }
    summarise(&per_iter, iters)
}

/// The three statistics of a set of per-iteration times.
pub(crate) fn summarise(per_iter_ns: &[f64], iters: u64) -> Stats {
    let mut sorted = per_iter_ns.to_vec();
    sorted.sort_by(f64::total_cmp);
    let median = median_of(&sorted);
    Stats {
        min_ns: sorted.first().copied().unwrap_or(f64::NAN),
        median_ns: median,
        spread: relative_spread(&sorted, median),
        iters,
        samples: per_iter_ns.len(),
    }
}

/// The median of an ascending slice: the middle value, or the mean of the two middle values when
/// there is an even number of them. An empty slice has no median and says so with NaN rather than
/// with a zero that would read as an infinitely fast benchmark.
pub(crate) fn median_of(sorted: &[f64]) -> f64 {
    match sorted.len() {
        0 => f64::NAN,
        n if n.is_multiple_of(2) => 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]),
        n => sorted[n / 2],
    }
}

/// The value at a fraction of the way through an ascending slice, interpolated between the two
/// neighbouring samples. p is in 0..=1; p = 0.95 is the p95 the replay report quotes.
///
/// Linear interpolation rather than nearest rank because a replay scenario is sampled at most a few
/// thousand times and the p95 of 3 600 frames otherwise steps in visible jumps between two adjacent
/// frames' times.
pub(crate) fn percentile_of(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let rank = p.clamp(0.0, 1.0) * (sorted.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    sorted[lo] + (sorted[hi] - sorted[lo]) * (rank - lo as f64)
}

/// The scaled median absolute deviation over the median, as a fraction of the median.
///
/// 1.4826 is the constant that makes the MAD of a normal distribution equal its standard deviation,
/// so the figure reads on the same scale as a relative standard deviation while being unmoved by
/// the one sample in thirty that was descheduled. A median of zero or a NaN has no relative spread
/// and reports zero, which the compare step then treats as "no noise measured" and falls back on
/// its own threshold for.
pub(crate) fn relative_spread(sorted: &[f64], median: f64) -> f64 {
    if sorted.is_empty() || !median.is_finite() || median <= 0.0 {
        return 0.0;
    }
    let mut deviations: Vec<f64> = sorted.iter().map(|v| (v - median).abs()).collect();
    deviations.sort_by(f64::total_cmp);
    1.4826 * median_of(&deviations) / median
}

/// The noise figure of a *window* of consecutive frames: the spread of its chunk medians.
///
/// The raw per-frame times of a replay are not samples of one quantity. A frame that emits a
/// wavefront costs more than one that does not, a frame that rebuilds the volume view's past cone
/// costs more again, and those differences are the workload rather than the machine: the median
/// absolute deviation over raw frames comes out at tens of percent on a window that is perfectly
/// repeatable, and a gate built on it would never fire. What *is* repeatable is a stretch of the
/// window - the emissions fall in the same places every run, because the run is the same run - so
/// the window is cut into equal chunks, each chunk is reduced to its own median ms per frame, and
/// the spread is taken over those. A quiet machine gives a percent or two; a machine that was
/// descheduled for half of one chunk shows it there and the median over the chunks does not move.
///
/// The statistic being compared is still the median over the whole window. This is only the
/// statement of how far to trust it.
pub(crate) fn chunked_spread(per_frame_ms: &[f64], chunks: usize) -> f64 {
    if per_frame_ms.is_empty() || chunks == 0 {
        return 0.0;
    }
    // Rounded up, so the chunk count is never more than asked for: 240 frames in 6 chunks is six of
    // 40, and 241 is five of 41 and one of 36 rather than seven chunks the last of which is one
    // frame long.
    let per_chunk = per_frame_ms.len().div_ceil(chunks);
    let mut medians: Vec<f64> = per_frame_ms
        .chunks(per_chunk)
        .map(|chunk| {
            let mut sorted = chunk.to_vec();
            sorted.sort_by(f64::total_cmp);
            median_of(&sorted)
        })
        .collect();
    medians.sort_by(f64::total_cmp);
    relative_spread(&medians, median_of(&medians))
}

/// A duration in nanoseconds written at a scale a human can read, three significant figures.
pub(crate) fn human_ns(ns: f64) -> String {
    if !ns.is_finite() {
        return "n/a".to_string();
    }
    if ns < 1e3 {
        format!("{ns:.1} ns")
    } else if ns < 1e6 {
        format!("{:.2} µs", ns / 1e3)
    } else if ns < 1e9 {
        format!("{:.3} ms", ns / 1e6)
    } else {
        format!("{:.3} s", ns / 1e9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_statistics_are_the_ones_the_report_claims() {
        // Odd and even medians, the p95 interpolation, and the scaling of the MAD: the three
        // numbers every row of the report is made of, on hand-checkable input.
        assert_eq!(median_of(&[1.0, 2.0, 3.0]), 2.0);
        assert_eq!(median_of(&[1.0, 2.0, 3.0, 5.0]), 2.5);
        assert!(median_of(&[]).is_nan(), "an empty set has no median");

        let ramp: Vec<f64> = (0..=100).map(f64::from).collect();
        assert!((percentile_of(&ramp, 0.95) - 95.0).abs() < 1e-9);
        assert_eq!(percentile_of(&ramp, 0.0), 0.0);
        assert_eq!(percentile_of(&ramp, 1.0), 100.0);

        // Deviations from the median 3 are 2, 1, 0, 1, 2, whose median is 1, so the scaled spread
        // is 1.4826 / 3.
        let five = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((relative_spread(&five, 3.0) - 1.4826 / 3.0).abs() < 1e-12);
        // One wild sample moves the median absolute deviation not at all, which is the whole reason
        // this is the MAD and not a standard deviation.
        let outlier = [1.0, 2.0, 3.0, 4.0, 500.0];
        assert!((relative_spread(&outlier, 3.0) - 1.4826 / 3.0).abs() < 1e-12);
        assert_eq!(relative_spread(&five, 0.0), 0.0, "no median, no measured noise");

        // `summarise` keeps the minimum as the machine's best offer rather than as a percentile.
        let stats = summarise(&[10.0, 12.0, 11.0], 7);
        assert_eq!(stats.min_ns, 10.0);
        assert_eq!(stats.median_ns, 11.0);
        assert_eq!(stats.iters, 7);
        assert_eq!(stats.samples, 3);
    }

    #[test]
    fn test_the_chunk_spread_measures_the_machine_and_not_the_workload() {
        // A window whose frames alternate 2 ms and 10 ms: a shape a real replay has, because an
        // emission frame costs more than the frames between emissions. Nothing about it varies from
        // run to run, so its noise figure has to be zero - and the raw median absolute deviation
        // over the same frames is enormous, which is exactly why the chunk medians are what is used.
        let alternating: Vec<f64> =
            (0..240).map(|i| if i % 2 == 0 { 2.0 } else { 10.0 }).collect();
        let mut sorted = alternating.clone();
        sorted.sort_by(f64::total_cmp);
        assert!(
            relative_spread(&sorted, median_of(&sorted)) > 0.9,
            "the raw spread of this window is over 90%, and all of it is workload"
        );
        assert_eq!(
            chunked_spread(&alternating, 6),
            0.0,
            "every chunk has the same median, so the machine did not move"
        );

        // Now the tail of the window costs twice what the head of it does: a stretch where the
        // machine was busy elsewhere. One bad chunk of six gives medians 1, 1, 1, 1, 1, 2, whose own
        // median is 1 and whose absolute deviations are 0, 0, 0, 0, 0, 1 - a median deviation of
        // zero. Two bad chunks give deviations 0, 0, 0, 0, 1, 1, whose median is still zero. That is
        // the median absolute deviation doing exactly what it is here for: a run has to be disturbed
        // over *half* its length before the figure that says how far to trust it moves at all.
        let window_with_bad_tail = |from: usize| {
            let mut frames: Vec<f64> = vec![1.0; 240];
            frames[from..].fill(2.0);
            chunked_spread(&frames, 6)
        };
        assert_eq!(window_with_bad_tail(200), 0.0, "one chunk of six");
        assert_eq!(window_with_bad_tail(160), 0.0, "two chunks of six");
        // Three of six: the medians are 1, 1, 1, 2, 2, 2, so the median is 1.5 and every chunk is
        // half an ms away from it.
        assert!(
            (window_with_bad_tail(120) - 1.4826 * 0.5 / 1.5).abs() < 1e-12,
            "half the window at twice the cost is a window the machine moved under"
        );

        assert_eq!(chunked_spread(&[], 6), 0.0, "no frames, no measured noise");
    }

    #[test]
    fn test_the_calibration_reaches_a_sample_long_enough_to_measure() {
        // A kernel of a fixed, tiny cost: the harness has to find an iteration count that puts a
        // sample over the floor, and then report the per-iteration cost rather than the batch.
        let budget = Budget { samples: 4, min_sample: Duration::from_millis(1), warmup: Duration::ZERO };
        let per_iteration = Duration::from_nanos(200);
        let mut body = |n: u64| per_iteration * n as u32;
        let stats = measure(&mut body, &budget);
        assert!(
            stats.iters >= 5_000,
            "a 200 ns operation needs at least 5 000 iterations to fill a millisecond, got {}",
            stats.iters
        );
        assert!(
            (stats.median_ns - 200.0).abs() < 1.0,
            "the figure reported is per iteration, not per batch: {}",
            stats.median_ns
        );
    }
}
