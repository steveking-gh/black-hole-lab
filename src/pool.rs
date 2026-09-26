//! A fork-join over the machine's spare cores: a list of jobs in, their results out in the order
//! the jobs were given.
//!
//! Two kinds of work go through it. The painters build and tessellate their meshes on it (see
//! `crate::gui::mesh_pool`), and the simulation integrates the rays of a large field on it, one
//! pulse a job (see `SignalField::advance_with`). Neither knows threads exist beyond this module,
//! and the physics does not depend on the gui to reach it, which is why it lives at the root of
//! the crate rather than beside either of them.
//!
//! The threads are scoped and spawned per call. They borrow what they work on - a field of light
//! several megabytes deep - straight from the caller, with no copy, no `'static` and no `unsafe`,
//! and they are joined before the borrow ends. Spawning a handful costs about a third of a
//! millisecond on Windows, against tens of milliseconds of work shared out.
//!
//! Results come back in job order, whichever thread finished first. For the painters that order is
//! the picture, because translucent triangles do not commute. For the physics it is nothing the
//! arithmetic depends on - each pulse is stepped by the same operations on whichever thread takes
//! it - but a caller that folds the results gets them in the order the serial loop would have.
//!
//! How a persistent pool would slot in: `run_jobs` is the only place that knows threads exist, and
//! its signature is what a pool would also offer, so its callers would not change. The jobs would.
//! A pool's threads outlive the call, so a job could no longer borrow the field; a painter would
//! need its own snapshot of the fronts, about 16 bytes a ray, taken as each simulation thread
//! finishes its own advance, and the physics would need its pulses moved out and back. That
//! snapshot is also what would let the drawing of one transmission start while the other is still
//! being stepped, which scoped threads joined after the whole step cannot do.

use std::sync::Mutex;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Most threads `spare_threads` offers besides the one calling.
///
/// Past about eight the jobs of one frame are too small for another thread to pay for its spawn.
/// The pool is never shared between the simulation and the painters: the simulation's workers are
/// joined before its step returns, and painting starts after that, so each gets the whole of it in
/// turn. Within the simulation the two transmissions each ask for all of it, which with both
/// observers transmitting is one thread more than a machine of sixteen cores has; the step was
/// measured a third faster that way than with the threads divided between them, because a
/// transmission that finishes first gives its threads back to the machine (see
/// `SignalPair::advance`).
pub(crate) const MAX_WORKERS: usize = 8;

/// How many threads a caller of `run_jobs` may start besides itself: one fewer than the machine
/// has, so the caller's share is not taken from it, and never more than `MAX_WORKERS`. Zero on a
/// machine with one core, where every job runs on the caller.
pub(crate) fn spare_threads() -> usize {
    static WORKERS: OnceLock<usize> = OnceLock::new();
    *WORKERS.get_or_init(|| {
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        cores.saturating_sub(1).min(MAX_WORKERS)
    })
}

/// Run every job and return what each returned, in the order the jobs were given.
///
/// Up to `workers` scoped threads are started and the calling thread works alongside them, every
/// thread taking the next job not yet taken until none are left, so a heavy job holds up one
/// thread and not a share of the rest. A panic in a job is carried back to the caller. With one
/// job, or `workers` zero, the jobs simply run on the caller in order.
pub(crate) fn run_jobs<J, R>(jobs: Vec<J>, workers: usize) -> Vec<R>
where
    J: FnOnce() -> R + Send,
    R: Send,
{
    let workers = workers.min(jobs.len().saturating_sub(1));
    if workers == 0 {
        return jobs.into_iter().map(|job| job()).collect();
    }
    let count = jobs.len();
    let queue: Vec<Mutex<Option<J>>> = jobs.into_iter().map(|job| Mutex::new(Some(job))).collect();
    let next = AtomicUsize::new(0);
    let work = || {
        let mut done = Vec::new();
        loop {
            let k = next.fetch_add(1, Ordering::Relaxed);
            let Some(slot) = queue.get(k) else { break };
            let job = slot
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .take()
                .expect("each job is taken by exactly one thread");
            done.push((k, job()));
        }
        done
    };
    let mut done = std::thread::scope(|s| {
        let threads: Vec<_> = (0..workers).map(|_| s.spawn(work)).collect();
        let mut done = work();
        for thread in threads {
            done.extend(thread.join().unwrap_or_else(|panic| std::panic::resume_unwind(panic)));
        }
        done
    });
    done.sort_unstable_by_key(|(k, _)| *k);
    debug_assert_eq!(done.len(), count, "every job ran once");
    done.into_iter().map(|(_, result)| result).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_results_come_back_in_job_order_whatever_thread_ran_them() {
        // The jobs finish in a scrambled order - the early ones sleep longest - and the results
        // must come back in the order the jobs were given all the same, since for a painter that
        // order is the order its meshes are painted in.
        let jobs: Vec<_> = (0..24u64)
            .map(|k| {
                move || {
                    std::thread::sleep(std::time::Duration::from_micros((24 - k) * 50));
                    k * k
                }
            })
            .collect();
        let got = run_jobs(jobs, 4);
        assert_eq!(got, (0..24u64).map(|k| k * k).collect::<Vec<_>>());
    }

    #[test]
    fn test_a_panic_in_a_job_reaches_the_caller() {
        // A job that panics on a worker must not be lost with the worker: the caller has to see
        // the panic, as it would have had the job run on the caller itself. Job 5 of twelve runs
        // on whichever thread takes it, so the test holds whichever that is.
        let jobs: Vec<_> = (0..12u32)
            .map(|k| {
                move || {
                    assert!(k != 5, "job five fails");
                    k
                }
            })
            .collect();
        let caught = std::panic::catch_unwind(|| run_jobs(jobs, 3));
        assert!(caught.is_err(), "the panic in job five reaches the caller");
    }
}
