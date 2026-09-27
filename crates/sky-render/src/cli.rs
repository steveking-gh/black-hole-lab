//! The command line.
//!
//! Parsed by hand, as the app's own `src/main.rs` does: a dozen flags do not justify a parser
//! crate, and every refusal can then be a sentence naming the flag.

use std::ops::Range;
use std::path::PathBuf;

use crate::encode::Codec;
use crate::panel::Placement;
use crate::sky::MapFrame;
use crate::tone::parse_hex_colour;

pub const USAGE: &str = "\
sky-render: renders a sky bundle over a star map into a 360-degree video

usage: sky-render --bundle <dir> --sky <map.exr> --out <video.mp4> [options]

  --bundle <dir>             the sky bundle (a directory holding manifest.json and frames/)
  --sky <map.exr>            the star map, an equirectangular OpenEXR file
  --out <video.mp4>          the video to write, tagged as a 360-degree equirectangular video
  --size <W>x<H>             the video's size; W must be twice H (default 8192x4096)
  --exposure <stops>         the exposure as a power of two (default: 2.5 stops for a map 8192
                             wide, two more for every doubling of the map's width)
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
                             the flat colour of rays the tracer did not resolve (default FF00FF)
  --threads <n>              render threads (default: all)
  --frames <a>..<b>          render only video frames a to b - 1 (a.. and ..b also work)
  --keep-frames <dir>        also write each frame as a 16-bit PNG into <dir>
  --overwrite                replace --out if it exists

read-outs (the bundle's numbers, the observer's stopwatch first, drawn on the sky):
  --readouts on|off          draw them (default on); off leaves the sky exactly as without them
  --readout-at <heading>,<elevation>
                             where a panel is centred, in degrees: heading to the viewer's right
                             of the opening view (90 is a quarter turn right, 180 behind),
                             elevation up from the horizon, at most 70 either way. Repeat for more
                             panels, all showing the same numbers (default: one at 0,-30, below
                             the opening view)
  --readout-size <degrees>   the height of one line of text, as an angle (default 2)
  --decimal-comma            write the decimal mark as a comma, as the app's own setting does
  --help                     this text
";

/// The largest `--readout-size`, in degrees: a line of text a sixth of the way up the sky is
/// already more than anyone needs to read it.
const MAX_READOUT_SIZE: f64 = 15.0;

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
    pub threads: usize,
    pub frames: Option<Range<u64>>,
    pub keep_frames: Option<PathBuf>,
    pub overwrite: bool,
    /// `--readouts off` makes this false.
    pub readouts: bool,
    /// Where the read-out panels go; never empty.
    pub readout_at: Vec<Placement>,
    /// The height of a line of read-out text, in degrees.
    pub readout_size: f64,
    pub decimal_comma: bool,
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
    let mut unresolved = [65_535, 0, 65_535];
    let mut threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let mut frames = None;
    let mut keep_frames = None;
    let mut overwrite = false;
    let mut readouts = true;
    // The first --readout-at replaces the default panel, and each after it adds one.
    let mut readout_at: Vec<Placement> = Vec::new();
    let mut readout_size = 2.0;
    let mut decimal_comma = false;

    let mut rest = args.iter();
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
                        .filter(|s| s.is_finite())
                        .ok_or_else(|| {
                            format!("--exposure takes a number of stops, not {text:?}")
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
                    format!("--unresolved-colour takes six hex digits such as FF00FF, not {text:?}")
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
                readouts = match value()? {
                    "on" => true,
                    "off" => false,
                    other => return Err(format!("--readouts is on or off, not {other:?}")),
                };
            }
            "--readout-at" => readout_at.push(Placement::parse(value()?)?),
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
            other => return Err(format!("{other:?} is not an option; see --help")),
        }
    }

    let bundle = bundle.ok_or("--bundle is required: the sky bundle to render")?;
    let sky = sky.ok_or("--sky is required: the star map to render over")?;
    if codec.is_some() && out.is_none() {
        return Err("--out is required: the video to write".into());
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
        threads,
        frames,
        keep_frames,
        overwrite,
        readouts,
        readout_at: if readout_at.is_empty() {
            vec![Placement::DEFAULT]
        } else {
            readout_at
        },
        readout_size,
        decimal_comma,
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
