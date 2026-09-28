//! What a run drew that is not sky: counts per frame and over the film, and the sentences that
//! explain a marker colour to whoever sees it in the video.
//!
//! Three kinds of pixel are counted. *Unresolved*: the nearest ray's fate is 0 (or a reserved
//! code, or a far-sky ray without a usable direction), or the video frame falls before the
//! bundle's first complete frame; drawn in `--unresolved-colour`. *Under-sampled*: the rays around
//! the pixel are too far apart to say where its light came from (`crate::field`); drawn in
//! `--undersampled-colour`. *Dark*: the dark region, drawn black. Every other pixel is sky. The
//! counts are of the frame as the renderer drew it, before read-out panels are painted over it.
//!
//! The sky pixels are counted once more, by their shift g ([`ShiftCounts`]): the least and the
//! largest g, and how many fall in each class of `crate::colour::Class`, from which the run says
//! what share of the film lies where the colour model is good. A sky pixel here is one drawn as
//! light: not dark, not marked, and not unresolved for a colour that came out not known.

use crate::colour::{Class, GLOW_RANGE, NOT_STARLIGHT, STAR_RANGE};

/// The sky pixels of one frame, or of many, by their shift.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftCounts {
    /// How many sky pixels fall in each class, indexed by `Class::index`.
    pub classes: [u64; 7],
    /// The least and the largest g; +infinity and -infinity when there is no sky pixel.
    pub least: f64,
    pub largest: f64,
}

impl Default for ShiftCounts {
    fn default() -> Self {
        Self {
            classes: [0; 7],
            least: f64::INFINITY,
            largest: f64::NEG_INFINITY,
        }
    }
}

impl ShiftCounts {
    /// Counts one sky pixel seen with shift `g`.
    pub fn count(&mut self, g: f64) {
        self.classes[Class::of(g).index()] += 1;
        self.least = self.least.min(g);
        self.largest = self.largest.max(g);
    }

    pub fn add(&mut self, other: &ShiftCounts) {
        for (a, b) in self.classes.iter_mut().zip(other.classes) {
            *a += b;
        }
        self.least = self.least.min(other.least);
        self.largest = self.largest.max(other.largest);
    }

    pub fn sky(&self) -> u64 {
        self.classes.iter().sum()
    }

    fn of(&self, classes: &[Class]) -> u64 {
        classes.iter().map(|c| self.classes[c.index()]).sum()
    }

    /// The sky pixels with g in [`STAR_RANGE`].
    pub fn in_star_range(&self) -> u64 {
        self.of(&[Class::Both, Class::StarOnly])
    }

    /// The sky pixels with g in [`GLOW_RANGE`].
    pub fn in_glow_range(&self) -> u64 {
        self.of(&[Class::GlowOnly, Class::Both])
    }

    /// The sky pixels shifted beyond [`NOT_STARLIGHT`], toward the red and toward the blue.
    pub fn beyond(&self) -> (u64, u64) {
        (
            self.classes[Class::FarRed.index()],
            self.classes[Class::FarBlue.index()],
        )
    }

    /// `g from 0.61 to 389; within 0.9..2 (good for single stars) 12.3 %, ...`
    pub fn describe(&self) -> String {
        let sky = self.sky();
        if sky == 0 {
            return "no sky pixels".into();
        }
        let (red, blue) = self.beyond();
        let n = NOT_STARLIGHT;
        format!(
            "g from {} to {}; within {}..{} (good for single stars) {}, within {}..{} (good for \
             the diffuse glow) {}, beyond a shift of {n} either way (mostly not starlight) {} \
             (below 1/{n}: {}, above {n}: {}), of {sky} sky pixels",
            shift_text(self.least),
            shift_text(self.largest),
            STAR_RANGE.0,
            STAR_RANGE.1,
            percent(self.in_star_range(), sky),
            GLOW_RANGE.0,
            GLOW_RANGE.1,
            percent(self.in_glow_range(), sky),
            percent(red + blue, sky),
            percent(red, sky),
            percent(blue, sky),
        )
    }

    /// The share of sky pixels outside [`STAR_RANGE`], the wider of the ranges where the model
    /// is good; 0 with no sky.
    pub fn outside_share(&self) -> f64 {
        let sky = self.sky();
        if sky == 0 {
            0.0
        } else {
            1.0 - self.in_star_range() as f64 / sky as f64
        }
    }
}

/// A shift with four significant figures.
fn shift_text(g: f64) -> String {
    if g == 0.0 || !g.is_finite() {
        return format!("{g}");
    }
    let decimals = (3 - g.abs().log10().floor() as i32).clamp(0, 12) as usize;
    format!("{g:.decimals$}")
}

/// The pixels of one frame, or of many, by what they show.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Tally {
    pub pixels: u64,
    pub unresolved: u64,
    pub undersampled: u64,
    pub dark: u64,
    /// The sky pixels drawn as light, by their shift.
    pub shift: ShiftCounts,
}

impl Tally {
    pub fn add(&mut self, other: &Tally) {
        self.pixels += other.pixels;
        self.unresolved += other.unresolved;
        self.undersampled += other.undersampled;
        self.dark += other.dark;
        self.shift.add(&other.shift);
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
    /// The video frame with the largest share of its sky pixels outside the range where the
    /// colour model is good for single stars, the earliest of equals.
    pub worst_shift: Option<(u64, ShiftCounts)>,
}

impl FilmTally {
    /// Adds video frame `k`.
    pub fn add(&mut self, k: u64, frame: Tally) {
        self.frames += 1;
        self.total.add(&frame);
        if self.worst.is_none_or(|(_, w)| frame.marked() > w.marked()) {
            self.worst = Some((k, frame));
        }
        if self
            .worst_shift
            .is_none_or(|(_, w)| frame.shift.outside_share() > w.outside_share())
        {
            self.worst_shift = Some((k, frame.shift));
        }
    }

    /// What the run prints at its end about the shifts of the sky it drew.
    pub fn shift_report(&self) -> Vec<String> {
        let mut out = vec![format!(
            "shifts of the sky pixels drawn over the {} frame(s): {}",
            self.frames,
            self.total.shift.describe()
        )];
        if let Some((k, worst)) = &self.worst_shift
            && self.frames > 1
        {
            out.push(format!(
                "the frame with the largest share of its sky outside {}..{}, video frame {k}: {}",
                STAR_RANGE.0,
                STAR_RANGE.1,
                worst.describe()
            ));
        }
        out
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
