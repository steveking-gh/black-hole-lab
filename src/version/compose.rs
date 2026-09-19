//! The rule that makes a version string out of a package version and a commit count.
//!
//! It lives in a file of its own because `build.rs` includes it by path and runs it at build time,
//! while the tests below run it inside the crate. One spelling of the rule, checked where the rest
//! of the crate's tests are, rather than a copy in the build script that nothing ever exercises.

/// `pkg_version`'s first two segments with `count` as the third: `compose("0.1.0", Some(166))` is
/// `"0.1.166"`. With no count - no git, or a directory that is not a checkout - the package version
/// comes back unchanged rather than the call failing.
pub fn compose(pkg_version: &str, count: Option<u64>) -> String {
    match count {
        Some(count) => {
            let major_minor = pkg_version.splitn(3, '.').take(2).collect::<Vec<_>>().join(".");
            format!("{major_minor}.{count}")
        }
        None => pkg_version.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_commit_count_replaces_the_patch_segment() {
        assert_eq!(compose("0.1.0", Some(707)), "0.1.707");
    }

    #[test]
    fn test_without_a_count_the_package_version_comes_back_whole() {
        assert_eq!(compose("0.1.0", None), "0.1.0");
    }

    #[test]
    fn test_the_major_and_minor_are_whatever_the_package_says() {
        assert_eq!(compose("2.5.0", Some(3)), "2.5.3");
    }
}
