//! What build this is, in one string that everything quotes.
//!
//! `build.rs` composes it once - `MAJOR.MINOR` from `Cargo.toml`, the commit count as the third
//! segment - and stamps it in, so asking for it at runtime costs nothing and cannot disagree with
//! itself. The title line shows it and every save file records it, which is the point: a screenshot
//! and a file from the same afternoon can be matched to the same build without either of them
//! having to carry a commit hash a person cannot read.

// The rule itself is `build.rs`'s to run, not the crate's: by the time this module exists the
// composing has already happened and the answer is in the environment. Compiling it here anyway,
// only for the tests, is what keeps those tests in `cargo test` with the rest of them.
#[cfg(test)]
mod compose;

/// This build's version: `MAJOR.MINOR` from the package version with the commit count behind it,
/// or the bare package version when the build did not happen in a checkout.
pub(crate) fn build_version() -> &'static str {
    env!("BLACK_HOLE_LAB_VERSION")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_stamped_version_is_three_numbers() {
        let version = build_version();
        assert!(!version.is_empty(), "the build must always stamp something");
        let segments: Vec<&str> = version.split('.').collect();
        assert_eq!(segments.len(), 3, "expected three segments, got {version:?}");
        assert!(
            segments.iter().all(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit())),
            "every segment must be a number: {version:?}"
        );
        println!("build version {version}");
    }
}
