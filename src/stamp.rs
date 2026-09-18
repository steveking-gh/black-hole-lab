//! When something was written, and what it was built from.
//!
//! Four small functions with two callers: `crate::perf`, which stamps every baseline so that two of
//! them can be told apart by eye, and `crate::save`, which stamps every save file for the same
//! reason. They live here rather than in either of those because a date is not a performance
//! measurement and a commit hash is not a file format, and because the alternative - the second
//! caller copying the first's calendar arithmetic - is how two spellings of the same date get into
//! one program.
//!
//! Everything here is best effort by design. No git, no checkout, a clock the system cannot read:
//! the answer is "unknown" or the epoch, and the caller carries on. A report from an unknown commit
//! is still a useful report, and a save file whose header cannot name its build is still a save
//! file.

use std::time::{SystemTime, UNIX_EPOCH};

/// Seconds since the Unix epoch, or 0 from a clock that cannot be read.
pub(crate) fn epoch_seconds() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// The moment of the call as an ISO-8601 UTC timestamp.
pub(crate) fn now_utc_iso() -> String {
    utc_iso(epoch_seconds())
}

/// The short hash of the commit this was built from, or "unknown" if git is not there or the
/// directory is not a checkout.
pub(crate) fn git_short_hash() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .map(|out| String::from_utf8_lossy(&out.stdout).trim().to_string())
        .filter(|hash| !hash.is_empty())
        .unwrap_or_else(|| "unknown".to_string())
}

/// Whether the tree had uncommitted changes. A dirty baseline is a baseline of something that is
/// not in the history and cannot be got back to, which is worth saying on the report.
pub(crate) fn git_dirty() -> bool {
    std::process::Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .is_some_and(|out| !String::from_utf8_lossy(&out.stdout).trim().is_empty())
}

/// Seconds since the Unix epoch as an ISO-8601 UTC timestamp.
///
/// Written out rather than taken from a crate, because the crate list is deliberately short and a
/// date on a report is not worth another entry. The calendar arithmetic is Howard Hinnant's
/// `civil_from_days`, which is exact for every date the proleptic Gregorian calendar covers.
pub(crate) fn utc_iso(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rest = secs % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rest / 3600,
        (rest % 3600) / 60,
        rest % 60
    )
}

/// The civil date of a day number counted from 1970-01-01.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    // Shift the epoch to 0000-03-01, which puts the leap day at the end of the year and makes the
    // month arithmetic a single linear formula.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_timestamp_is_the_date_it_claims() {
        assert_eq!(utc_iso(0), "1970-01-01T00:00:00Z");
        assert_eq!(utc_iso(86_399), "1970-01-01T23:59:59Z");
        assert_eq!(utc_iso(86_400), "1970-01-02T00:00:00Z");
        // A leap day in a century that *is* a leap year, and the first of March in one that is
        // not, which is where a hand-rolled calendar usually goes wrong.
        assert_eq!(civil_from_days(11_016), (2000, 2, 29));
        assert_eq!(civil_from_days(47_541), (2100, 3, 1));
        assert_eq!(utc_iso(1_774_224_000), "2026-03-23T00:00:00Z");
    }
}
