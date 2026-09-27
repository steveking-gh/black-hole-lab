//! Sharing an image's rows among threads, with nothing but the standard library.
//!
//! The rows are cut into bands of a few rows and the threads take bands from a shared queue until
//! it is empty. Bands rather than one slab per thread because rows do not cost the same: a row
//! through the shadow is cheap, and a row across the map's pole reads the coarse levels, so equal
//! slabs would leave threads idle waiting for the slowest.

use std::ops::Range;
use std::sync::Mutex;

/// Calls `work(rows, band)` for every band of `band_rows` rows of `image`, whose rows are
/// `row_len` items long, on `threads` scoped threads. `band` is the slice of `image` holding
/// exactly those rows.
pub fn for_each_band<T: Send>(
    image: &mut [T],
    row_len: usize,
    band_rows: usize,
    threads: usize,
    work: impl Fn(Range<usize>, &mut [T]) + Sync,
) {
    if image.is_empty() {
        return;
    }
    let band_len = row_len * band_rows.max(1);
    let threads = threads.clamp(1, image.len().div_ceil(band_len));
    let queue = Mutex::new(image.chunks_mut(band_len).enumerate());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                loop {
                    // The lock is held only to take the next band, never while working on it.
                    let next = queue
                        .lock()
                        .expect("no worker panics holding the queue")
                        .next();
                    let Some((k, band)) = next else { break };
                    let first = k * band_rows.max(1);
                    work(first..first + band.len() / row_len, band);
                }
            });
        }
    });
}
