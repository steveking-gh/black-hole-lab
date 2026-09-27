//! What a run drew that is not sky: counts per frame and over the film, and the sentences that
//! explain a marker colour to whoever sees it in the video.
//!
//! Three kinds of pixel are counted. *Unresolved*: the nearest ray's fate is 0 (or a reserved
//! code, or a far-sky ray without a usable direction), or the video frame falls before the
//! bundle's first complete frame; drawn in `--unresolved-colour`. *Under-sampled*: the rays around
//! the pixel are too far apart to say where its light came from (`crate::field`); drawn in
//! `--undersampled-colour`. *Dark*: the dark region, drawn black. Every other pixel is sky. The
//! counts are of the frame as the renderer drew it, before read-out panels are painted over it.

/// The pixels of one frame, or of many, by what they show.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Tally {
    pub pixels: u64,
    pub unresolved: u64,
    pub undersampled: u64,
    pub dark: u64,
}

impl Tally {
    pub fn add(&mut self, other: &Tally) {
        self.pixels += other.pixels;
        self.unresolved += other.unresolved;
        self.undersampled += other.undersampled;
        self.dark += other.dark;
    }

    /// The pixels drawn in a marker colour.
    pub fn marked(&self) -> u64 {
        self.unresolved + self.undersampled
    }

    /// `unresolved 12 (0.001 %), under-sampled ..., dark ...`
    pub fn describe(&self) -> String {
        let part = |name: &str, n: u64| format!("{name} {n} ({})", percent(n, self.pixels));
        format!(
            "{}, {}, {}",
            part("unresolved", self.unresolved),
            part("under-sampled", self.undersampled),
            part("dark", self.dark)
        )
    }
}

/// `n` of `of` as a percentage with three significant figures, and a plain 0 for none.
fn percent(n: u64, of: u64) -> String {
    if n == 0 || of == 0 {
        return "0 %".into();
    }
    let p = 100.0 * n as f64 / of as f64;
    let decimals = (2 - p.log10().floor() as i32).clamp(0, 6) as usize;
    format!("{p:.decimals$} %")
}

/// The counts of a run: the whole film, and its worst frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FilmTally {
    pub frames: u64,
    pub total: Tally,
    /// The video frame with the most pixels in a marker colour, the earliest of equals.
    pub worst: Option<(u64, Tally)>,
}

impl FilmTally {
    /// Adds video frame `k`.
    pub fn add(&mut self, k: u64, frame: Tally) {
        self.frames += 1;
        self.total.add(&frame);
        if self.worst.is_none_or(|(_, w)| frame.marked() > w.marked()) {
            self.worst = Some((k, frame));
        }
    }

    /// What the run prints at its end. `judged` is false for `--undersampled interpolate`, which
    /// does not look for under-sampled pixels at all.
    pub fn report(
        &self,
        unresolved: [u16; 3],
        undersampled: [u16; 3],
        judged: bool,
    ) -> Vec<String> {
        let mut out = vec![format!(
            "pixels drawn over the {} frame(s): {}",
            self.frames,
            self.total.describe()
        )];
        if let Some((k, worst)) = &self.worst
            && self.frames > 1
        {
            out.push(format!(
                "the frame with the most marked pixels, video frame {k}: {}",
                worst.describe()
            ));
        }
        if !judged {
            out.push(
                "under-sampled pixels were not looked for (--undersampled interpolate): the sky \
                 is drawn by interpolation everywhere, known or not"
                    .into(),
            );
        }
        let (a, b) = (self.total.unresolved > 0, self.total.undersampled > 0);
        let not_known = "are ones this program does not know how to draw:";
        let unresolved_why = "rays the tracer did not resolve";
        let undersampled_why = "places where the bundle's rays are too far apart to say which part \
                                of the sky the light came from";
        if a && b && unresolved == undersampled {
            out.push(format!(
                "Pixels drawn in {} {not_known} {unresolved_why}, and {undersampled_why}.",
                colour_name(unresolved)
            ));
        } else {
            if a {
                out.push(format!(
                    "Pixels drawn in {} {not_known} {unresolved_why}.",
                    colour_name(unresolved)
                ));
            }
            if b {
                out.push(format!(
                    "Pixels drawn in {} {not_known} {undersampled_why}.",
                    colour_name(undersampled)
                ));
            }
        }
        out
    }
}

/// A marker colour as its hex code, with a plain name when it has one: `#FF0000 (red)`.
pub fn colour_name(rgb: [u16; 3]) -> String {
    let [r, g, b] = rgb.map(|c| (c / 257) as u8);
    let hex = format!("#{r:02X}{g:02X}{b:02X}");
    let name = match (r, g, b) {
        (255, 0, 0) => Some("red"),
        (0, 255, 0) => Some("green"),
        (0, 0, 255) => Some("blue"),
        (255, 0, 255) => Some("magenta"),
        (0, 255, 255) => Some("cyan"),
        (255, 255, 0) => Some("yellow"),
        (255, 255, 255) => Some("white"),
        (0, 0, 0) => Some("black"),
        _ => None,
    };
    match name {
        Some(name) => format!("{hex} ({name})"),
        None => hex,
    }
}
