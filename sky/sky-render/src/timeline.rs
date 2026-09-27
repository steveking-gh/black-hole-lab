//! Which bundle frames each video frame shows.
//!
//! Video frame k is shown k / fps seconds into the video, and the specification (section 5.7)
//! puts that at stopwatch time `k / fps * proper_time_per_video_second`. The stopwatch counts from
//! frame 0 of the bundle. The bundle's frames need not be evenly spaced and need not fall on video
//! frames, so each video frame is placed between the two bundle frames around it.
//!
//! # Holding the ends
//!
//! Before the first bundle frame and after the last, `pick` holds the nearest frame. After the
//! last it never matters: the video ends at the last bundle frame (`video_frames`), so a video
//! frame is at most a rounding error past it, and on it. Before the first it matters when the
//! stopwatch's origin, frame 0, is listed but not yet complete (a bundle still being written, or
//! damaged): the opening video frames would then show the sky of a later moment under an earlier
//! stopwatch, which is plausible and false. The renderer asks [`Timeline::before_first`] and draws
//! such a frame wholly in the unresolved colour instead: the bundle has no rays for that moment.

/// What one video frame is made of: positions in the list of bundle frames being played, not
/// frame indices.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pick {
    /// One bundle frame alone: the video frame falls on it, or before the first or after the
    /// last, where the nearest frame is held.
    One(usize),
    /// `w` of the way from the first to the second, 0 < w < 1.
    Two(usize, usize, f64),
}

/// The bundle frames' times on the stopwatch, and the playback rate.
#[derive(Debug, Clone)]
pub struct Timeline {
    /// Stopwatch time of each bundle frame, in the bundle's time unit, rising.
    times: Vec<f64>,
    frames_per_second: f64,
    proper_time_per_video_second: f64,
}

impl Timeline {
    /// `proper_times` are the bundle frames' proper times, rising; `origin` is the proper time
    /// of frame 0, from which the stopwatch counts.
    pub fn new(
        proper_times: &[f64],
        origin: f64,
        frames_per_second: f64,
        proper_time_per_video_second: f64,
    ) -> Self {
        Self {
            times: proper_times.iter().map(|t| t - origin).collect(),
            frames_per_second,
            proper_time_per_video_second,
        }
    }

    /// The stopwatch time video frame `k` shows.
    pub fn stopwatch(&self, k: u64) -> f64 {
        k as f64 / self.frames_per_second * self.proper_time_per_video_second
    }

    /// How many video frames the bundle fills: from frame 0 to the last video frame that does
    /// not pass the last bundle frame. A bundle of one frame makes a video of one frame.
    pub fn video_frames(&self) -> u64 {
        let last = self.times.last().copied().unwrap_or(0.0).max(0.0);
        let frames = last / self.proper_time_per_video_second * self.frames_per_second;
        // A last frame at 2.9999999999 video frames' worth of time is meant to be at 3.
        (frames * (1.0 + 1e-9)).floor() as u64 + 1
    }

    /// Whether video frame `k` falls before the first bundle frame, by more than a millionth of a
    /// video frame's worth of time (rounding is not a moment of its own).
    pub fn before_first(&self, k: u64) -> bool {
        let step = self.proper_time_per_video_second / self.frames_per_second;
        self.stopwatch(k) < self.times[0] - 1e-6 * step
    }

    /// The bundle frames video frame `k` is made of.
    pub fn pick(&self, k: u64) -> Pick {
        self.pick_at(self.stopwatch(k))
    }

    /// The bundle frames stopwatch time `t` is made of.
    pub fn pick_at(&self, t: f64) -> Pick {
        let times = &self.times;
        let last = times.len() - 1;
        // Within this fraction of the local spacing a time counts as falling on a frame: the
        // video frame's time and the bundle frame's are the same number computed two ways, and
        // an ulp between them must not ask for a blend with a weight of 1e-16.
        const ON: f64 = 1e-6;
        if t <= times[0] {
            return Pick::One(0);
        }
        if t >= times[last] {
            return Pick::One(last);
        }
        // The first frame later than t; it exists and is not frame 0, by the tests above.
        let after = times.partition_point(|&s| s <= t);
        let before = after - 1;
        let span = times[after] - times[before];
        let w = (t - times[before]) / span;
        if w <= ON {
            Pick::One(before)
        } else if w >= 1.0 - ON {
            Pick::One(after)
        } else {
            Pick::Two(before, after, w)
        }
    }
}
