//! The command line.
//!
//! Parsed by hand, as the app's own `src/main.rs` does: a dozen flags do not justify a parser
//! crate, and every refusal can then be a sentence naming the flag.

use std::ops::Range;
use std::path::PathBuf;

use crate::colour::ColourRule;
use crate::encode::Codec;
use crate::panel::Placement;
use crate::sky::MapFrame;
use crate::tone::parse_hex_colour;

pub const USAGE: &str = "\
sky-render: renders a sky bundle over a star map into a 360-degree video

usage: sky-render --bundle <dir> --sky <map.exr> --out <video.mkv> [options]

  --bundle <dir>             the sky bundle (a directory holding manifest.json and frames/)
  --sky <map.exr>            the star map, an equirectangular OpenEXR file
  --out <video.mkv|video.mp4>
                             the video to write, tagged as a 360-degree equirectangular video.
                             A .mkv file carries the read-outs as its subtitle track; beside a
                             .mp4 file they go into a subtitle file of the same name ending .ass,
                             which VLC loads by itself
  --size <W>x<H>             the video's size; W must be twice H (default 8192x4096)
  --exposure <stops>         the exposure as a power of two, at most 100 either way (default:
                             2.5 stops for a map 8192 wide, two more for every doubling of the
                             map's width)
  --colour blackbody|map     how shifted light is coloured. blackbody (default): each texel of
                             the map is a blackbody at the temperature its colour implies, and a
                             shift g makes it the blackbody at g times that temperature, as an
                             eye sees it, so hue and brightness both follow the shift. map: the
                             map's own colour times g^4, the renderer's old rule
  --show-model-range         draw each sky pixel in a false colour for the range its shift g
                             falls in, showing where the blackbody model is good (the colours
                             are printed with the run); a diagnostic, off by default
  --encoder svt|nvenc|aom|none
                             SVT-AV1 on the CPU (default), NVIDIA's AV1 encoder, or libaom;
                             none renders without encoding (with --keep-frames, or for timing)
  --crf <n>                  the constant-quality level (default 28; NVENC's -cq)
  --preset <n>               the speed preset (defaults: svt 8, nvenc 5 meaning p5, aom 6
                             meaning -cpu-used 6)
  --ffmpeg <path>            the ffmpeg to run (default: ffmpeg from PATH)
  --map-frame celestial|galactic
                             the map's coordinates (default: galactic if the file name has _gal)
  --unresolved-colour <RRGGBB>
                             the flat colour of rays the tracer did not resolve (default FF0000)
  --undersampled mark|interpolate
                             where the bundle's rays are too far apart to say which part of the
                             sky the light came from (beside a black hole's dark region, where
                             light has circled the hole): mark those pixels (default), or
                             interpolate them as if the rays did say
  --undersampled-colour <RRGGBB>
                             the flat colour of such pixels (default FF0000, as unresolved rays,
                             so that everything the program does not know is one colour)
  --threads <n>              render threads (default: all)
  --frames <a>..<b>          render only video frames a to b - 1 (a.. and ..b also work)
  --keep-frames <dir>        also write each frame as a 16-bit PNG into <dir>
  --overwrite                replace --out, its .ass file and --photo if they exist

read-outs (the bundle's numbers, the observer's stopwatch first):
  --readouts overlay|panel|off
                             overlay (default): a subtitle track, which the player draws at the
                             bottom right of its window, where it stays as the viewer looks
                             around; the picture carries nothing. panel (or on): a panel painted
                             on the sky, seen only when looking that way. off: none, and the sky
                             exactly as without them
  --overlay-size <percent>   the height of one line of the overlay, as a percentage of the
                             screen's height (default 1.67: size 18 of 1080, the approved size)
  --overlay-font <name>      the overlay's typeface, which the player looks for on its own
                             machine; its digits must all be one width (default Consolas, which
                             Windows has; Courier New is on Windows and macOS, and on Linux as
                             Liberation Mono)
  --readout-at <heading>,<elevation>
                             where a panel is centred, in degrees: heading to the viewer's right
                             of the opening view (90 is a quarter turn right, 180 behind),
                             elevation up from the horizon, at most 70 either way. Repeat for more
                             panels, all showing the same numbers (default: one at 0,-30, below
                             the opening view)
  --readout-at dark          for a still: one panel inside the black hole's dark region, where
                             it hides no sky, with 1 degree of the region clear all round it;
                             made smaller, to 1 degree a line, if it does not fit, and put at
                             0,-30 if it does not fit even then (the run says where it went)
  --readout-size <degrees>   the height of one line of text, as an angle (default 2)
  --decimal-comma            write the decimal mark as a comma, as the app's own setting does

a still (one moment's view, to look around in):
  --still [<k>]              render video frame k alone, and hold it (k may be left out when the
                             bundle makes one video frame)
  --hold <seconds>           how long the held video lasts (default 60)
  --hold-rate <per second>   how often the held picture is repeated (default 0.5; rarer frames
                             are stored truly, but tools that guess a rate by probing misread it)
  --photo <file.jpg>         also write the view as a 360-degree JPEG photograph (Google Photo
                             Sphere). A photograph has no subtitle track: it shows the read-outs
                             only with --readouts panel

  --help                     this text
";

/// The largest `--exposure` either way, in stops: the gain 2^stops is then a positive finite f32
/// (2^100 is 1.3e30; f32 reaches 3.4e38), which `crate::tone::shade` relies on.
const MAX_EXPOSURE_STOPS: f64 = 100.0;

/// The largest `--readout-size`, in degrees: a line of text a sixth of the way up the sky is
/// already more than anyone needs to read it.
const MAX_READOUT_SIZE: f64 = 15.0;

/// The largest `--overlay-size`, in percent of the screen's height: ten such lines fill it, and
/// three read-outs at that size already hide a third of the view.
const MAX_OVERLAY_SIZE: f64 = 10.0;

/// A still's held video lasts a minute unless told otherwise: long enough to look all round.
pub const DEFAULT_HOLD_SECONDS: f64 = 60.0;

/// The longest `--hold`, in seconds: an hour of one picture is more than anyone needs to look
/// around in it, and the limit keeps the frame count and the subtitle times ordinary numbers.
const MAX_HOLD_SECONDS: f64 = 3600.0;

/// How many times a second a still's picture is handed to the encoder. Measured 2026-09-27 with
/// ffmpeg 8.1.2 and SVT-AV1, 60 s held at 1, 1/2, 1/3, 1/4, 1/5, 1/6, 1/10 and 1/60 frames a
/// second: every MP4 and every Matroska file remuxed from it is valid, reads back 60 s long, and
/// stores the true frame duration (the MP4's sample table; Matroska's DefaultDuration). What
/// fails below one frame in 2 s is the guess a reader makes by probing: ffprobe looks at the
/// first 5 s, sees fewer than three frames, and reports a frame rate of 1000 (the millisecond
/// timestamps' rate), or at 1/10 even with a longer look guesses 1/2. A player that estimates
/// the rate the same way could be misled, so the rate is the lowest at which the guess is right.
/// (At 8K the guess fails at any rate, for another reason: the one keyframe is 10 MB, past the
/// 5 MB ffprobe reads, so it sees one frame. The stored duration is still true.)
pub const DEFAULT_HOLD_RATE: f64 = 0.5;

/// The fastest `--hold-rate`: a film's own rate. Faster only makes more identical frames.
const MAX_HOLD_RATE: f64 = 60.0;

/// What becomes of the read-outs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadoutMode {
    /// A subtitle track, drawn by the player in screen space.
    Overlay,
    /// A panel painted on the sphere.
    Panel,
    Off,
}

/// The container `--out` names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Container {
    /// Matroska: the video and the read-outs' subtitle track in one file.
    Mkv,
    /// MP4: the video, with the read-outs in a subtitle file beside it.
    Mp4,
}

impl Container {
    /// The container a file name asks for, by its extension in any case.
    pub fn of(path: &std::path::Path) -> Result<Self, String> {
        let extension = path
            .extension()
            .map(|e| e.to_string_lossy().to_ascii_lowercase());
        match extension.as_deref() {
            Some("mkv") => Ok(Self::Mkv),
            Some("mp4") => Ok(Self::Mp4),
            _ => Err(format!(
                "--out {} ends neither in .mkv (the video with its read-outs as a subtitle track) \
                 nor in .mp4 (the video, with its read-outs in a .ass file beside it)",
                path.display()
            )),
        }
    }
}

/// What `--still` asks for.
#[derive(Debug, Clone, PartialEq)]
pub struct Still {
    /// The video frame, or None for the film's only one.
    pub frame: Option<u64>,
    /// How long the held video should last, in seconds.
    pub hold: f64,
    /// How many times a second the picture is handed to the encoder.
    pub rate: f64,
    /// Where the photograph goes, when one is asked for.
    pub photo: Option<PathBuf>,
}

impl Still {
    /// The video frame a still of a film of `total` video frames shows, or a sentence saying why
    /// there is none.
    pub fn frame_of(&self, total: u64) -> Result<u64, String> {
        match self.frame {
            Some(k) if k < total => Ok(k),
            Some(k) => Err(format!(
                "--still {k} is past the end of the film, whose {total} video frame(s) are \
                 numbered 0 to {}",
                total.saturating_sub(1)
            )),
            None if total == 1 => Ok(0),
            None => Err(format!(
                "--still needs a video frame number: the bundle makes {total} video frames, \
                 numbered 0 to {}",
                total.saturating_sub(1)
            )),
        }
    }

    /// How many times the picture is handed to the encoder: enough to fill `hold`, and at least
    /// once. The held video lasts `repeats / rate` seconds, `hold` rounded up to a whole frame.
    pub fn repeats(&self) -> u64 {
        // The tolerance keeps a hold that is a whole number of frames, but whose product comes
        // out a rounding error above it in binary, from gaining a frame.
        ((self.hold * self.rate * (1.0 - 1e-12)).ceil() as u64).max(1)
    }

    /// The held video's length in seconds.
    pub fn seconds(&self) -> f64 {
        self.repeats() as f64 / self.rate
    }
}

/// Everything the command line says.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub bundle: PathBuf,
    pub sky: PathBuf,
    /// Absent only with `--encoder none`.
    pub out: Option<PathBuf>,
    pub width: usize,
    pub height: usize,
    pub exposure: Option<f64>,
    /// `None` for `--encoder none`.
    pub codec: Option<Codec>,
    pub crf: Option<u32>,
    pub preset: Option<u32>,
    pub ffmpeg: PathBuf,
    pub map_frame: Option<MapFrame>,
    pub unresolved: [u16; 3],
    /// `--undersampled mark` (true, the default) or `interpolate`.
    pub mark_undersampled: bool,
    pub undersampled: [u16; 3],
    pub threads: usize,
    pub frames: Option<Range<u64>>,
    pub keep_frames: Option<PathBuf>,
    pub overwrite: bool,
    /// Whether the read-out panel is painted on the picture: `--readouts panel`, or `on`.
    pub readouts: bool,
    /// Whether the read-outs are written as a subtitle track: `--readouts overlay`, the default.
    /// At most one of this and `readouts` is true, and `--readouts off` makes both false.
    pub subtitles: bool,
    /// The height of a line of the overlay, in percent of the screen's height.
    pub overlay_size: f64,
    /// The overlay's typeface, by name.
    pub overlay_font: String,
    /// The container `out` names; None when there is no `out`.
    pub container: Option<Container>,
    pub still: Option<Still>,
    /// Where the read-out panels go; never empty. With `--readout-at dark`, the one default
    /// place, where the panel goes if the dark region cannot hold it.
    pub readout_at: Vec<Placement>,
    /// `--readout-at dark`: one panel, placed inside the dark region of the still's picture
    /// (`crate::shadow`).
    pub readout_in_dark: bool,
    /// The height of a line of read-out text, in degrees.
    pub readout_size: f64,
    pub decimal_comma: bool,
    /// `--colour`: how shifted light is coloured.
    pub colour: ColourRule,
    /// `--show-model-range`: the picture shows the class of each sky pixel's shift.
    pub show_model_range: bool,
}

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    Help,
    Render(Box<Options>),
}

/// Parses the arguments after the program's name.
pub fn parse(args: &[String]) -> Result<Request, String> {
    let mut bundle = None;
    let mut sky = None;
    let mut out = None;
    let mut size = (8192, 4096);
    let mut exposure = None;
    let mut codec = Some(Codec::Svt);
    let mut crf = None;
    let mut preset = None;
    let mut ffmpeg = PathBuf::from("ffmpeg");
    let mut map_frame = None;
    // Bright red for both markers: whatever the program does not know is one colour by default.
    let mut unresolved = [65_535, 0, 0];
    let mut mark_undersampled = true;
    let mut undersampled = [65_535, 0, 0];
    let mut threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let mut frames = None;
    let mut keep_frames = None;
    let mut overwrite = false;
    let mut mode = ReadoutMode::Overlay;
    let mut overlay_size = crate::subtitles::DEFAULT_PERCENT;
    let mut overlay_font = crate::subtitles::DEFAULT_FONT.to_string();
    let mut still_frame: Option<Option<u64>> = None;
    let mut hold = None;
    let mut hold_rate = None;
    let mut photo = None;
    // The first --readout-at replaces the default panel, and each after it adds one.
    let mut readout_at: Vec<Placement> = Vec::new();
    let mut readout_in_dark = false;
    let mut readout_size = 2.0;
    let mut decimal_comma = false;
    let mut colour = ColourRule::Blackbody;
    let mut show_model_range = false;

    let mut rest = args.iter().peekable();
    while let Some(flag) = rest.next() {
        let mut value = || {
            rest.next()
                .map(String::as_str)
                .ok_or_else(|| format!("{flag} needs a value; see --help"))
        };
        match flag.as_str() {
            "--help" | "-h" => return Ok(Request::Help),
            "--bundle" => bundle = Some(PathBuf::from(value()?)),
            "--sky" => sky = Some(PathBuf::from(value()?)),
            "--out" => out = Some(PathBuf::from(value()?)),
            "--size" => size = parse_size(value()?)?,
            "--exposure" => {
                let text = value()?;
                exposure = Some(
                    text.parse::<f64>()
                        .ok()
                        .filter(|s| s.is_finite() && s.abs() <= MAX_EXPOSURE_STOPS)
                        .ok_or_else(|| {
                            format!(
                                "--exposure takes a number of stops, at most \
                                 {MAX_EXPOSURE_STOPS} either way, not {text:?}"
                            )
                        })?,
                );
            }
            "--encoder" => {
                let text = value()?;
                codec = if text == "none" {
                    None
                } else {
                    Some(Codec::parse(text).ok_or_else(|| {
                        format!("--encoder is svt, nvenc, aom or none, not {text:?}")
                    })?)
                };
            }
            "--crf" => crf = Some(parse_whole(flag, value()?)?),
            "--preset" => preset = Some(parse_whole(flag, value()?)?),
            "--ffmpeg" => ffmpeg = PathBuf::from(value()?),
            "--map-frame" => {
                map_frame = Some(match value()? {
                    "celestial" => MapFrame::Celestial,
                    "galactic" => MapFrame::Galactic,
                    other => {
                        return Err(format!(
                            "--map-frame is celestial or galactic, not {other:?}"
                        ));
                    }
                });
            }
            "--unresolved-colour" => {
                let text = value()?;
                unresolved = parse_hex_colour(text).ok_or_else(|| {
                    format!("--unresolved-colour takes six hex digits such as FF0000, not {text:?}")
                })?;
            }
            "--undersampled" => {
                mark_undersampled = match value()? {
                    "mark" => true,
                    "interpolate" => false,
                    other => {
                        return Err(format!(
                            "--undersampled is mark or interpolate, not {other:?}"
                        ));
                    }
                };
            }
            "--undersampled-colour" => {
                let text = value()?;
                undersampled = parse_hex_colour(text).ok_or_else(|| {
                    format!(
                        "--undersampled-colour takes six hex digits such as FF0000, not {text:?}"
                    )
                })?;
            }
            "--threads" => {
                threads = parse_whole(flag, value()?)? as usize;
                if threads == 0 {
                    return Err("--threads needs at least one thread".into());
                }
            }
            "--frames" => frames = Some(parse_range(value()?)?),
            "--keep-frames" => keep_frames = Some(PathBuf::from(value()?)),
            "--overwrite" => overwrite = true,
            "--readouts" => {
                mode = match value()? {
                    "overlay" => ReadoutMode::Overlay,
                    // `on` asked for the panel before the overlay existed, and still does.
                    "panel" | "on" => ReadoutMode::Panel,
                    "off" => ReadoutMode::Off,
                    other => {
                        return Err(format!(
                            "--readouts is overlay, panel or off, not {other:?}"
                        ));
                    }
                };
            }
            "--overlay-size" => {
                let text = value()?;
                overlay_size = text
                    .parse::<f64>()
                    .ok()
                    .filter(|s| s.is_finite() && *s > 0.0 && *s <= MAX_OVERLAY_SIZE)
                    .ok_or_else(|| {
                        format!(
                            "--overlay-size is the height of a line in percent of the screen's \
                             height, more than 0 and at most {MAX_OVERLAY_SIZE}, not {text:?}"
                        )
                    })?;
            }
            "--overlay-font" => {
                let text = value()?.trim();
                // The name goes into a style line of comma-separated fields, which a comma
                // would end early.
                if text.is_empty() || text.chars().any(|c| c == ',' || c.is_control()) {
                    return Err(format!(
                        "--overlay-font names a typeface, with no comma or control character, \
                         not {text:?}"
                    ));
                }
                overlay_font = text.to_string();
            }
            "--still" => {
                // The frame number may be left out, so the next argument is taken as one only
                // when it is not another option.
                let k = match rest.next_if(|next| !next.starts_with("--")) {
                    Some(text) => Some(text.parse::<u64>().map_err(|_| {
                        format!("--still takes a video frame number, such as 237, not {text:?}")
                    })?),
                    None => None,
                };
                still_frame = Some(k);
            }
            "--hold" => {
                let text = value()?;
                hold = Some(
                    text.parse::<f64>()
                        .ok()
                        .filter(|s| s.is_finite() && *s > 0.0 && *s <= MAX_HOLD_SECONDS)
                        .ok_or_else(|| {
                            format!(
                                "--hold is how many seconds the still lasts, more than 0 and at \
                                 most {MAX_HOLD_SECONDS}, not {text:?}"
                            )
                        })?,
                );
            }
            "--hold-rate" => {
                let text = value()?;
                hold_rate = Some(
                    text.parse::<f64>()
                        .ok()
                        .filter(|s| s.is_finite() && *s > 0.0 && *s <= MAX_HOLD_RATE)
                        .ok_or_else(|| {
                            format!(
                                "--hold-rate is how many times a second the still's picture \
                                 is repeated, more than 0 and at most {MAX_HOLD_RATE}, not \
                                 {text:?}"
                            )
                        })?,
                );
            }
            "--photo" => {
                let path = PathBuf::from(value()?);
                let extension = path
                    .extension()
                    .map(|e| e.to_string_lossy().to_ascii_lowercase());
                if !matches!(extension.as_deref(), Some("jpg" | "jpeg")) {
                    return Err(format!(
                        "--photo names a JPEG photograph, ending .jpg or .jpeg, not {}",
                        path.display()
                    ));
                }
                photo = Some(path);
            }
            "--readout-at" => match value()? {
                "dark" => readout_in_dark = true,
                text => readout_at.push(Placement::parse(text)?),
            },
            "--readout-size" => {
                let text = value()?;
                readout_size = text
                    .parse::<f64>()
                    .ok()
                    .filter(|s| s.is_finite() && *s > 0.0 && *s <= MAX_READOUT_SIZE)
                    .ok_or_else(|| {
                        format!(
                            "--readout-size is the height of a line of text in degrees, more than \
                             0 and at most {MAX_READOUT_SIZE}, not {text:?}"
                        )
                    })?;
            }
            "--decimal-comma" => decimal_comma = true,
            "--colour" => {
                colour = match value()? {
                    "blackbody" => ColourRule::Blackbody,
                    "map" => ColourRule::Map,
                    other => {
                        return Err(format!("--colour is blackbody or map, not {other:?}"));
                    }
                };
            }
            "--show-model-range" => show_model_range = true,
            other => return Err(format!("{other:?} is not an option; see --help")),
        }
    }

    let bundle = bundle.ok_or("--bundle is required: the sky bundle to render")?;
    let sky = sky.ok_or("--sky is required: the star map to render over")?;
    if codec.is_some() && out.is_none() {
        return Err("--out is required: the video to write".into());
    }
    let container = out.as_deref().map(Container::of).transpose()?;
    let still = match still_frame {
        Some(frame) => {
            if frames.is_some() {
                return Err(
                    "--still renders one video frame and --frames a run of them; give one".into(),
                );
            }
            Some(Still {
                frame,
                hold: hold.unwrap_or(DEFAULT_HOLD_SECONDS),
                rate: hold_rate.unwrap_or(DEFAULT_HOLD_RATE),
                photo,
            })
        }
        None => {
            for (given, flag) in [
                (hold.is_some(), "--hold"),
                (hold_rate.is_some(), "--hold-rate"),
                (photo.is_some(), "--photo"),
            ] {
                if given {
                    return Err(format!(
                        "{flag} belongs to a still, and there is no --still <frame>"
                    ));
                }
            }
            None
        }
    };
    if readout_in_dark {
        if !readout_at.is_empty() {
            return Err(
                "--readout-at dark places the one panel by itself, inside the dark region, and \
                 cannot be combined with a --readout-at <heading>,<elevation>"
                    .into(),
            );
        }
        if still.is_none() {
            return Err(
                "--readout-at dark belongs to a still (--still <frame>): during a film the dark \
                 region moves and grows, and a panel keeps one place for the whole run"
                    .into(),
            );
        }
    }
    if let (Some(Codec::Nvenc), Some(p)) = (codec, preset)
        && !(1..=7).contains(&p)
    {
        return Err(format!("--preset for nvenc is 1 to 7 (p1 to p7), not {p}"));
    }
    Ok(Request::Render(Box::new(Options {
        bundle,
        sky,
        out,
        width: size.0,
        height: size.1,
        exposure,
        codec,
        crf,
        preset,
        ffmpeg,
        map_frame,
        unresolved,
        mark_undersampled,
        undersampled,
        threads,
        frames,
        keep_frames,
        overwrite,
        readouts: mode == ReadoutMode::Panel,
        subtitles: mode == ReadoutMode::Overlay,
        overlay_size,
        overlay_font,
        container,
        still,
        readout_at: if readout_at.is_empty() {
            vec![Placement::DEFAULT]
        } else {
            readout_at
        },
        readout_in_dark,
        readout_size,
        decimal_comma,
        colour,
        show_model_range,
    })))
}

fn parse_whole(flag: &str, text: &str) -> Result<u32, String> {
    text.parse()
        .map_err(|_| format!("{flag} takes a whole number, not {text:?}"))
}

/// `<W>x<H>`, W twice H: an equirectangular frame covers 360 degrees across and 180 down.
pub fn parse_size(text: &str) -> Result<(usize, usize), String> {
    let bad = || format!("--size is <width>x<height>, such as 8192x4096, not {text:?}");
    let (w, h) = text.split_once(['x', 'X']).ok_or_else(bad)?;
    let w: usize = w.trim().parse().map_err(|_| bad())?;
    let h: usize = h.trim().parse().map_err(|_| bad())?;
    if h == 0 || w != 2 * h {
        return Err(format!(
            "--size {w}x{h} is not twice as wide as it is high, and an equirectangular frame \
             spans 360 degrees across and 180 down"
        ));
    }
    Ok((w, h))
}

/// `a..b`, `a..` or `..b`: video frames a to b - 1.
pub fn parse_range(text: &str) -> Result<Range<u64>, String> {
    let bad = || format!("--frames is <first>..<end>, such as 0..90, not {text:?}");
    let (a, b) = text.split_once("..").ok_or_else(bad)?;
    let a = if a.is_empty() {
        0
    } else {
        a.parse().map_err(|_| bad())?
    };
    let b = if b.is_empty() {
        u64::MAX
    } else {
        b.parse().map_err(|_| bad())?
    };
    if a >= b {
        return Err(format!("--frames {text} holds no frames"));
    }
    Ok(a..b)
}
