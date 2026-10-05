//! Handing frames to ffmpeg, to PNG files, and to the JPEG encoder for the photograph.
//!
//! Frames go to ffmpeg's standard input as raw 16-bit RGB (`rgb48le`), and ffmpeg converts them
//! to 10-bit Y'CbCr 4:2:0 and encodes them. Converting in ffmpeg rather than here keeps one
//! implementation of the BT.709 matrix and the chroma filter, the one players are tested against;
//! measured 2026-09-27 it converts an 8192 x 4096 frame in under 10 ms, far below the encoder's
//! 200. The pipe carries 201 MB a frame and has a small buffer, so a slow encoder stops this
//! program's writes, and the renderer, which hands frames over through a queue of two, stops
//! behind them: that is the back-pressure that keeps memory to a few frames.
//!
//! # Colour tags
//!
//! The frames hold BT.709-primaries RGB (the maps' primaries, by the OpenEXR convention) encoded
//! with the sRGB transfer function, and the stream is tagged with exactly that:
//!
//! - primaries `bt709`;
//! - transfer `iec61966-2-1`, the sRGB curve that was applied;
//! - matrix `bt709`, the one ffmpeg is told to convert with (its default for RGB input is the
//!   BT.601 matrix, which would shift every hue a little);
//! - range `tv` (limited), what 10-bit AV1 players expect.
//!
//! Tagging the transfer as `bt709` instead would be the more common label but a wrong one: BT.709
//! names a camera curve, and a player that manages colour pairs it with a BT.1886 display (gamma
//! 2.4), which darkens the sky's faint glow relative to the sRGB decoding the frames were made
//! for. The approved clip carried `iec61966-2-1`, and VLC 3 showed it as the owner saw it; players
//! that ignore the tag show an SDR picture as sRGB anyway, which is the same thing on a desktop
//! monitor.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// Which AV1 encoder ffmpeg runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    /// SVT-AV1, on the CPU: the default, and the one the owner approved.
    Svt,
    /// NVIDIA's hardware AV1 encoder (RTX 40 series and later): about twice as fast at 8K.
    Nvenc,
    /// libaom, the reference encoder: slow at 8K, and here for when neither of the others is.
    Aom,
}

impl Codec {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "svt" => Some(Self::Svt),
            "nvenc" => Some(Self::Nvenc),
            "aom" => Some(Self::Aom),
            _ => None,
        }
    }

    pub fn default_crf(self) -> u32 {
        28
    }

    pub fn default_preset(self) -> u32 {
        match self {
            Self::Svt => 8,
            Self::Nvenc => 5,
            Self::Aom => 6,
        }
    }

    /// The ffmpeg arguments that choose and set the encoder.
    fn arguments(self, crf: u32, preset: u32) -> Vec<String> {
        match self {
            // Preset 8 and CRF 28 are the settings of the clip the owner approved.
            Self::Svt => vec![
                "-c:v".into(),
                "libsvtav1".into(),
                "-preset".into(),
                preset.to_string(),
                "-crf".into(),
                crf.to_string(),
            ],
            // Constant quality in VBR mode with no bitrate cap is NVENC's nearest to a CRF. Its
            // presets are named p1 (fastest) to p7 (best).
            Self::Nvenc => vec![
                "-c:v".into(),
                "av1_nvenc".into(),
                "-preset".into(),
                format!("p{preset}"),
                "-tune".into(),
                "hq".into(),
                "-rc".into(),
                "vbr".into(),
                "-cq".into(),
                crf.to_string(),
                "-b:v".into(),
                "0".into(),
            ],
            // `-b:v 0` makes the CRF a constant quality rather than a cap; row multithreading
            // and tiles are what let libaom use more than a few cores on a frame this large.
            Self::Aom => vec![
                "-c:v".into(),
                "libaom-av1".into(),
                "-cpu-used".into(),
                preset.to_string(),
                "-crf".into(),
                crf.to_string(),
                "-b:v".into(),
                "0".into(),
                "-row-mt".into(),
                "1".into(),
                "-tiles".into(),
                "4x2".into(),
            ],
        }
    }

    /// The pixel format the encoder is fed: NVENC takes 10-bit input as P010, the others as
    /// planar 10-bit.
    fn pixel_format(self) -> &'static str {
        match self {
            Self::Nvenc => "p010le",
            _ => "yuv420p10le",
        }
    }
}

/// What ffmpeg is asked to make.
#[derive(Debug, Clone)]
pub struct Encoding {
    pub ffmpeg: PathBuf,
    pub codec: Codec,
    pub crf: u32,
    pub preset: u32,
    pub width: usize,
    pub height: usize,
    pub frames_per_second: f64,
    /// Where ffmpeg writes the untagged MP4.
    pub out: PathBuf,
}

/// A running ffmpeg, with its standard input open for frames.
pub struct Ffmpeg {
    child: Child,
    stdin: Option<ChildStdin>,
    stderr: Option<JoinHandle<()>>,
    /// ffmpeg's last lines of standard error, kept to explain a failure.
    tail: Arc<Mutex<VecDeque<String>>>,
    program: PathBuf,
}

/// How many lines of ffmpeg's standard error are kept, and how many a failure report shows.
const TAIL: usize = 40;
const REPORT: usize = 15;

impl Ffmpeg {
    pub fn start(e: &Encoding) -> Result<Self, String> {
        // A rate as a decimal: ffmpeg turns it into the nearest fraction, which for 29.97 or
        // 23.976 is the usual NTSC one.
        let rate = format!("{}", e.frames_per_second);
        let mut args: Vec<String> = [
            "-hide_banner",
            "-nostats",
            "-loglevel",
            "warning",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb48le",
        ]
        .map(String::from)
        .to_vec();
        args.extend([
            "-video_size".into(),
            format!("{}x{}", e.width, e.height),
            "-framerate".into(),
            rate,
            "-i".into(),
            "pipe:0".into(),
            // The matrix and range of the conversion, stated rather than left to defaults.
            "-vf".into(),
            //
            // The frames' colour properties are then stamped on by `setparams`. The output
            // options further down set the same values on the stream, but ffmpeg 8 hands the
            // encoder the frames' own properties, and raw input frames have none: without this
            // SVT-AV1 writes "unknown" primaries and transfer into the bitstream.
            format!(
                "scale=in_range=full:out_range=tv:out_color_matrix=bt709,format={},\
                 setparams=range=tv:color_primaries=bt709:color_trc=iec61966-2-1:colorspace=bt709",
                e.codec.pixel_format()
            ),
        ]);
        args.extend(e.codec.arguments(e.crf, e.preset));
        args.extend(
            [
                "-color_primaries",
                "bt709",
                "-color_trc",
                "iec61966-2-1",
                "-colorspace",
                "bt709",
                "-color_range",
                "tv",
                // Named, not inferred from the file name, which ends in a temporary suffix. No
                // `-movflags +faststart`: it would put `moov` before `mdat`, which the spherical
                // tag then refuses (see `crate::mp4`).
                "-f",
                "mp4",
            ]
            .map(String::from),
        );
        args.push(e.out.to_string_lossy().into_owned());

        let mut child = Command::new(&e.ffmpeg)
            .args(&args)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|err| {
                if err.kind() == std::io::ErrorKind::NotFound {
                    format!(
                        "ffmpeg was not found (looked for {}); install it, for example with \
                         `winget install Gyan.FFmpeg`, or give its path with --ffmpeg.",
                        e.ffmpeg.display()
                    )
                } else {
                    format!("could not start {}: {err}", e.ffmpeg.display())
                }
            })?;
        let tail = Arc::new(Mutex::new(VecDeque::with_capacity(TAIL)));
        let stderr = child.stderr.take().expect("stderr was piped");
        let keep = Arc::clone(&tail);
        let stderr = std::thread::spawn(move || {
            for line in BufReader::new(stderr).lines() {
                let Ok(line) = line else { break };
                let mut tail = keep.lock().expect("the reader alone writes");
                if tail.len() == TAIL {
                    tail.pop_front();
                }
                tail.push_back(line);
            }
        });
        Ok(Self {
            stdin: child.stdin.take(),
            child,
            stderr: Some(stderr),
            tail,
            program: e.ffmpeg.clone(),
        })
    }

    /// Writes one frame of 16-bit RGB. Blocks while the encoder is behind.
    pub fn write(&mut self, frame: &[u16], scratch: &mut Vec<u8>) -> Result<(), String> {
        let stdin = self.stdin.as_mut().expect("open until finish");
        // Little-endian bytes, converted a slice at a time so the copy stays in cache.
        for part in frame.chunks(1 << 18) {
            scratch.clear();
            scratch.extend(part.iter().flat_map(|v| v.to_le_bytes()));
            if stdin.write_all(scratch).is_err() {
                // ffmpeg has gone; its own words say why.
                return Err(self.failure("stopped taking frames"));
            }
        }
        Ok(())
    }

    /// Closes ffmpeg's input and waits for it to finish the file.
    pub fn finish(mut self) -> Result<(), String> {
        drop(self.stdin.take());
        let status = self
            .child
            .wait()
            .map_err(|e| format!("could not wait for ffmpeg: {e}"))?;
        if let Some(reader) = self.stderr.take() {
            let _ = reader.join();
        }
        if status.success() {
            Ok(())
        } else {
            Err(self.failure(&format!("failed ({status})")))
        }
    }

    fn failure(&mut self, what: &str) -> String {
        // Let ffmpeg finish saying why before its words are read.
        let _ = self.stdin.take();
        let _ = self.child.wait();
        if let Some(reader) = self.stderr.take() {
            let _ = reader.join();
        }
        let tail = self.tail.lock().expect("the reader has finished");
        let lines: Vec<&str> = tail
            .iter()
            .skip(tail.len().saturating_sub(REPORT))
            .map(String::as_str)
            .collect();
        let program = self.program.display();
        if lines.is_empty() {
            format!("{program} {what}, and said nothing.")
        } else {
            format!(
                "{program} {what}. Its last words:\n    {}",
                lines.join("\n    ")
            )
        }
    }
}

impl Drop for Ffmpeg {
    /// An ffmpeg abandoned before `finish`, by a run that failed, is stopped and waited for:
    /// left alone it would go on encoding into its partial file after this program has deleted
    /// it (or, on Windows, keep it open so that it cannot be deleted).
    fn drop(&mut self) {
        if self.stdin.take().is_some() {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

/// Runs ffmpeg with `args` and `input` on its standard input, and returns its standard output, or
/// a sentence with its last words saying why it failed to `what`.
fn run_once(
    program: &Path,
    args: &[String],
    input: Vec<u8>,
    what: &str,
) -> Result<Vec<u8>, String> {
    let mut child = Command::new(program)
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("could not start {} to {what}: {e}", program.display()))?;
    // The input is fed from a thread of its own while the output is read here: a pipe holds a
    // few kilobytes, and ffmpeg may write before it has read everything.
    let mut stdin = child.stdin.take().expect("stdin was piped");
    let feeder = std::thread::spawn(move || {
        let _ = stdin.write_all(&input);
    });
    let output = child
        .wait_with_output()
        .map_err(|e| format!("could not wait for {}: {e}", program.display()))?;
    let _ = feeder.join();
    if output.status.success() {
        return Ok(output.stdout);
    }
    let words = String::from_utf8_lossy(&output.stderr);
    let lines: Vec<&str> = words.lines().collect();
    let tail = &lines[lines.len().saturating_sub(REPORT)..];
    Err(format!(
        "{} failed to {what} ({}). Its last words:\n    {}",
        program.display(),
        output.status,
        tail.join("\n    ")
    ))
}

/// The JPEG quality of the photograph, on the libjpeg scale `jpeg-encoder` takes (1 to 100).
///
/// Chosen 2026-10-05 against the photograph ffmpeg made before (`-q:v 2`, its MJPEG encoder's
/// usual "high quality"), on the same 8192 x 4096 frame of demos/near_fall.bhl, both measured
/// against the 16-bit frame they were made from: see the commit that brought this encoder in.
pub const JPEG_QUALITY: u8 = 95;

/// A frame of 16-bit RGB as a JPEG photograph at high quality.
///
/// JPEG is 8-bit Y'CbCr with the BT.601 matrix at full range (the JFIF convention every decoder
/// assumes), and the frame's codes are sRGB-encoded, which is what a JPEG without a colour profile
/// is taken to hold. The conversion is made here from the 16-bit frame (`ycbcr_of`), so the frame
/// is rounded to 8 bits once, not once to 8-bit RGB and again to Y'CbCr. The chroma is kept at
/// full resolution (4:4:4): a marker pixel one pixel wide, a red pixel the program could not
/// determine, or a coloured star would be smeared into its black neighbours by halved chroma.
/// The Huffman tables are made for the image, which costs a second pass and saves size.
///
/// The encoder is `jpeg-encoder`, in this process: ffmpeg is not needed for a photograph.
pub fn jpeg(frame: &[u16], width: usize, height: usize) -> Result<Vec<u8>, String> {
    let side = |n: usize, what: &str| {
        u16::try_from(n).map_err(|_| {
            format!("the photograph is {n} pixels {what}, more than a JPEG can be (65,535)")
        })
    };
    let (w, h) = (side(width, "wide")?, side(height, "high")?);
    let mut out = Vec::new();
    let mut encoder = jpeg_encoder::Encoder::new(&mut out, JPEG_QUALITY);
    encoder.set_sampling_factor(jpeg_encoder::SamplingFactor::F_1_1);
    encoder.set_optimized_huffman_tables(true);
    encoder
        .encode(&ycbcr_of(frame), w, h, jpeg_encoder::ColorType::Ycbcr)
        .map_err(|e| format!("could not encode the photograph: {e}"))?;
    Ok(out)
}

/// 16-bit sRGB-encoded RGB as 8-bit Y'CbCr, three bytes a pixel: the BT.601 matrix at full range,
/// JFIF's, each value rounded to the nearest code once.
pub fn ycbcr_of(frame: &[u16]) -> Vec<u8> {
    let byte = |v: f64| v.round().clamp(0.0, 255.0) as u8;
    let mut out = Vec::with_capacity(frame.len());
    for rgb in frame.as_chunks::<3>().0 {
        // 65535 is 255: a 16-bit code is the 8-bit one times 257.
        let [r, g, b] = rgb.map(|v| f64::from(v) / 257.0);
        let y = 0.299 * r + 0.587 * g + 0.114 * b;
        let cb = 128.0 - 0.168_736 * r - 0.331_264 * g + 0.5 * b;
        let cr = 128.0 + 0.5 * r - 0.418_688 * g - 0.081_312 * b;
        out.extend([byte(y), byte(cb), byte(cr)]);
    }
    out
}

/// Copies the video of the tagged MP4 `video`, and the subtitles of the ASS file `subtitles` if
/// there is one, into the Matroska file `out`, without re-encoding either.
///
/// ffmpeg reads the Spherical Video V1 box of the MP4 as the stream's spherical mapping and
/// writes it as Matroska's own `Projection` element, so the Matroska file is tagged as a
/// 360-degree video without this program writing Matroska itself. The subtitle track is marked
/// as the default one, which makes VLC show it without being asked.
pub fn remux_matroska(
    program: &Path,
    video: &Path,
    subtitles: Option<&Path>,
    out: &Path,
) -> Result<(), String> {
    let path = |p: &Path| p.to_string_lossy().into_owned();
    let mut args: Vec<String> = ["-hide_banner", "-nostats", "-loglevel", "error", "-y", "-i"]
        .map(String::from)
        .to_vec();
    args.push(path(video));
    if let Some(subtitles) = subtitles {
        args.extend(["-i".into(), path(subtitles)]);
    }
    args.extend(["-map", "0:v", "-c:v", "copy"].map(String::from));
    if subtitles.is_some() {
        args.extend(
            [
                "-map",
                "1:s",
                "-c:s",
                "ass",
                "-disposition:s:0",
                "default",
                "-metadata:s:s:0",
                "title=Read-outs",
            ]
            .map(String::from),
        );
    }
    // Named, not inferred from the file name, which ends in a temporary suffix.
    args.extend(["-f".into(), "matroska".into(), path(out)]);
    run_once(program, &args, Vec::new(), "make the Matroska file").map(|_| ())
}

/// Writes a frame as a 16-bit RGB PNG, marked as sRGB. PNG stores 16-bit samples big-endian.
pub fn write_png(path: &Path, frame: &[u16], width: usize, height: usize) -> Result<(), String> {
    let say = |e: &dyn std::fmt::Display| format!("could not write {}: {e}", path.display());
    let file = std::fs::File::create(path).map_err(|e| say(&e))?;
    let mut encoder = png::Encoder::new(
        std::io::BufWriter::new(file),
        u32::try_from(width).map_err(|e| say(&e))?,
        u32::try_from(height).map_err(|e| say(&e))?,
    );
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Sixteen);
    encoder.set_source_srgb(png::SrgbRenderingIntent::Perceptual);
    // The masters are for keeping, not for speed of reading; but at 8K the default compression
    // takes seconds a frame, and the fast setting loses a few percent of size for most of that.
    encoder.set_compression(png::Compression::Fast);
    let mut writer = encoder.write_header().map_err(|e| say(&e))?;
    let bytes: Vec<u8> = frame.iter().flat_map(|v| v.to_be_bytes()).collect();
    writer.write_image_data(&bytes).map_err(|e| say(&e))?;
    writer.finish().map_err(|e| say(&e))
}

#[cfg(test)]
/// True when `ffmpeg -version` runs: for tests that need ffmpeg and must pass without it.
pub fn ffmpeg_available(program: &Path) -> bool {
    Command::new(program)
        .arg("-version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|s| s.success())
}
