//! sky-render: a sky bundle and a star map in, a 360-degree video out.
//!
//! The bundle (physics_simulation_specification.md at the workspace root) says, for each moment
//! of an observer's watch and each direction on the observer's sky, where on the distant sky the
//! light came from, how its frequency was shifted, and whether it came from the sky at all. This
//! program knows no physics: it looks each direction up on a star map, brightens or dims it by
//! the shift, and hands the frames to ffmpeg, which encodes them as AV1. Then it marks the file as
//! an equirectangular 360-degree video, so that a player lets the viewer look around.
//!
//! The read-outs (the observer's stopwatch and the bundle's other numbers) go by default into a
//! subtitle track, which the player draws fixed at the bottom right of its window while the sky
//! turns behind it (`subtitles`); `--readouts panel` paints them on the sphere instead. A *still*
//! (`--still`) is one video frame held for a minute, for looking around in one moment, and can
//! also be written as a 360-degree photograph (`photo`).
//!
//! The pure parts, each tested on tiny inputs in `tests`:
//!
//! - `timeline`: which bundle frames each video frame shows;
//! - `bilinear` and `field`: the ray at any output pixel, from the bundle's coarser grid, across
//!   the seam and over the poles, respecting each ray's fate, and marking where the rays are too
//!   far apart to say where the light came from;
//! - `sky`: the rotation into the map's frame, and the filtered map lookup (a rip-map);
//! - `tone`: the shift's g^4, the exposure, clipping and the sRGB curve;
//! - `render`: the frame, row bands on scoped threads;
//! - `tally`: how many pixels were drawn unresolved, under-sampled and dark, and the sentence that
//!   explains a marker colour;
//! - `mp4`: the spherical-video tag, appended to the finished file;
//! - `values`: each read-out's value at a video frame, and how it is written;
//! - `subtitles`: the read-outs as an ASS subtitle script, one cue per video frame;
//! - `panel`: where a read-out panel sits on the sphere, and which pixels show it;
//! - `layout`: where each character of a panel goes, with digits that do not jitter;
//! - `photo`: the Photo Sphere metadata of a 360-degree JPEG.
//!
//! `text` rasterises the app's typefaces and `overlay` composites the panels over a finished
//! frame. The I/O is in `load` (the EXR map and the bundle's frames), `encode` (ffmpeg and PNG),
//! `output` (which files a run writes, and how each reaches its name) and here.

mod bilinear;
mod cli;
mod encode;
mod field;
mod layout;
mod load;
mod mp4;
mod output;
mod overlay;
mod panel;
mod parallel;
mod photo;
mod render;
mod sky;
mod subtitles;
mod tally;
mod text;
mod timeline;
mod tone;
mod values;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_marking;
#[cfg(test)]
mod tests_output;
#[cfg(test)]
mod tests_readouts;

use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cli::{Options, Request, USAGE};
use encode::{Encoding, Ffmpeg};
use field::{Judge, RayField};
use output::{Cleanup, Plan};
use overlay::Overlay;
use readout::Style;
use render::{Fields, Look, Scene, Size};
use sky::{MapFrame, SkyMap};
use tally::FilmTally;
use timeline::{Pick, Timeline};
use values::Series;

/// NASA's credit line, which any published video made from the Deep Star Maps must carry
/// (sky/maps/README.md, "Credit").
const CREDIT: &str = "NASA/Goddard Space Flight Center Scientific Visualization Studio. \
Gaia DR2: ESA/Gaia/DPAC. Constellation figures based on those developed for the IAU by Alan \
MacRobert of Sky and Telescope magazine (Roger Sinnott and Rick Fienberg).";

fn main() {
    // `args_os`, not `args`, which panics on an argument that is not Unicode. A path that is not
    // comes through with the offending characters replaced, and is then refused as a file that
    // cannot be found, in a sentence.
    let args: Vec<String> = std::env::args_os()
        .skip(1)
        .map(|a| a.to_string_lossy().into_owned())
        .collect();
    match cli::parse(&args) {
        Ok(Request::Help) => print!("{USAGE}"),
        Ok(Request::Render(options)) => {
            if let Err(message) = run(&options) {
                eprintln!("sky-render: {message}");
                std::process::exit(1);
            }
        }
        Err(message) => {
            eprintln!("sky-render: {message}");
            std::process::exit(2);
        }
    }
}

/// A frame on its way from the renderer to the writer.
struct Finished {
    k: u64,
    pixels: Vec<u16>,
}

/// What the writer thread spent its time on.
#[derive(Default)]
struct WriterTimes {
    /// Handing frames to ffmpeg, including the time ffmpeg made it wait.
    encode: Duration,
    png: Duration,
    photo: Duration,
}

/// What the writer does with each frame besides encoding it.
struct WriterJob {
    ffmpeg: Option<Ffmpeg>,
    /// How many times each frame is handed to ffmpeg: 1 for a film, more for a held still.
    repeats: u64,
    keep: Option<PathBuf>,
    /// The ffmpeg that makes a still's photograph, and where the photograph goes.
    photo: Option<(PathBuf, PathBuf)>,
    width: usize,
    height: usize,
}

fn run(o: &Options) -> Result<(), String> {
    let started = Instant::now();
    // The files this run gives the viewer, and those it makes on the way. The scratch files are
    // removed when `_cleanup` goes, at the end of this function however it ends; it is declared
    // before anything that can write one, so that it goes after all of them (locals are dropped
    // in reverse order), the ffmpeg of a failed run included.
    let plan = match (&o.out, o.container, o.codec) {
        (Some(out), Some(container), Some(_)) => Some(Plan::new(out, container, o.subtitles)),
        _ => None,
    };
    let _cleanup = Cleanup(plan.as_ref().map(Plan::scratch).unwrap_or_default());
    let photo_path = o.still.as_ref().and_then(|s| s.photo.clone());
    let mut finals: Vec<PathBuf> = match &plan {
        Some(plan) => plan.finals(),
        None => o.out.iter().cloned().collect(),
    };
    finals.extend(photo_path.clone());
    for path in &finals {
        if path.exists() && !o.overwrite {
            return Err(format!(
                "{} already exists; give --overwrite to replace it",
                path.display()
            ));
        }
    }
    if let Some(dir) = &o.keep_frames {
        std::fs::create_dir_all(dir)
            .map_err(|e| format!("could not create {}: {e}", dir.display()))?;
    }

    let bundle = sky_format::BundleReader::open(&o.bundle).map_err(|e| e.to_string())?;
    let manifest = bundle.manifest();
    let complete = bundle.complete();
    if complete.is_empty() {
        return Err(format!(
            "the bundle {} has no complete frames yet",
            o.bundle.display()
        ));
    }
    let entry = |index: u32| {
        let at = manifest
            .frames
            .binary_search_by_key(&index, |e| e.index)
            .expect("complete frames are listed");
        &manifest.frames[at]
    };
    let times: Vec<f64> = complete.iter().map(|&i| entry(i).proper_time.0).collect();
    // The stopwatch counts from frame 0; if frame 0 is not listed, from the first frame that is.
    let origin = manifest
        .frames
        .first()
        .filter(|e| e.index == 0)
        .map_or(times[0], |e| e.proper_time.0);
    let playback = &manifest.playback;
    let timeline = Timeline::new(
        &times,
        origin,
        playback.frames_per_second.0,
        playback.proper_time_per_video_second.0,
    );
    let total = timeline.video_frames();
    let range = match (&o.still, &o.frames) {
        (Some(still), _) => {
            let k = still.frame_of(total)?;
            k..k + 1
        }
        (None, Some(r)) => r.start.min(total)..r.end.min(total),
        (None, None) => 0..total,
    };
    if range.is_empty() {
        return Err(format!(
            "the bundle makes {total} video frames, and --frames asks for none of them"
        ));
    }

    // The read-outs are set up before the map is read, so that a panel the command line cannot
    // place is refused at once and not after the map's half-minute.
    let series = Series::new(
        &manifest.readouts,
        &complete
            .iter()
            .map(|&i| &entry(i).readouts)
            .collect::<Vec<_>>(),
    );
    let mut overlay = Overlay::for_run(
        o,
        &series,
        [
            timeline.stopwatch(range.start),
            timeline.stopwatch(range.end - 1),
        ],
    )?;
    // Each line's value at video frame k: what the panel paints and the subtitles show alike.
    let values_at = |k: u64| series.at(timeline.pick(k), timeline.stopwatch(k));
    let script = match (&plan, o.subtitles) {
        (Some(_), true) => {
            let cues = match &o.still {
                // A still's read-outs are those of its one frame, for the whole held video.
                Some(still) => vec![subtitles::still_cue(
                    &series,
                    &timeline,
                    range.start,
                    still.seconds(),
                )],
                None => subtitles::film_cues(
                    &series,
                    &timeline,
                    range.clone(),
                    playback.frames_per_second.0,
                ),
            };
            let look = subtitles::Look {
                font: o.overlay_font.clone(),
                percent: o.overlay_size,
                style: if o.decimal_comma {
                    Style::COMMA
                } else {
                    Style::POINT
                },
            };
            Some(subtitles::script(&series.lines, &cues, &look))
        }
        _ => None,
    };

    match (&overlay, &script) {
        (Some(_), _) => println!(
            "read-outs: {} line(s) on {} panel(s) painted on the sky, each line {} degrees high",
            series.lines.len(),
            o.readout_at.len(),
            o.readout_size
        ),
        (None, Some(_)) => println!(
            "read-outs: {} line(s) as a subtitle track at the bottom right of the screen, in {}, \
             each line {:.2} % of the screen's height (size {} of {}); the picture carries none",
            series.lines.len(),
            o.overlay_font,
            o.overlay_size,
            subtitles::font_size(o.overlay_size),
            subtitles::SCRIPT_HEIGHT
        ),
        (None, None) if o.subtitles => {
            println!("read-outs: none, since --encoder none makes no video to carry subtitles")
        }
        (None, None) => println!("read-outs: off"),
    }

    let clock = Instant::now();
    let image = load::read_map(&o.sky)?;
    let image_damaged = image.damaged;
    let read_seconds = clock.elapsed().as_secs_f64();
    let clock = Instant::now();
    let sky = SkyMap::new(image.width, image.height, image.texels, o.threads);
    let pyramid_seconds = clock.elapsed().as_secs_f64();
    let map_frame = o
        .map_frame
        .unwrap_or_else(|| MapFrame::from_file_name(&o.sky));
    let rotation = sky::orientation(&manifest.far_sky, map_frame);
    let stops = o
        .exposure
        .unwrap_or_else(|| tone::default_exposure_stops(sky.width()));
    let look = Look {
        gain: 2f64.powf(stops) as f32,
        unresolved: o.unresolved,
        undersampled: o.undersampled,
        encoder: tone::Encoder::new(),
    };
    let size = Size {
        width: o.width,
        height: o.height,
    };
    // What a ray grid resolves depends on how finely the video samples it and on the finest
    // detail the map holds (`field`, "Where interpolation does not know the answer").
    let judge = o.mark_undersampled.then(|| Judge {
        pixels_per_ray: (
            size.width as f64 / manifest.grid.width as f64,
            size.height as f64 / manifest.grid.height as f64,
        ),
        texel: (2.0 * std::f64::consts::PI / sky.width() as f64)
            .min(std::f64::consts::PI / sky.height() as f64),
    });
    println!(
        "bundle {}: {} frames on a {} x {} grid, far sky {:?}",
        o.bundle.display(),
        complete.len(),
        manifest.grid.width,
        manifest.grid.height,
        manifest.far_sky.name
    );
    let listed = manifest.frames.len();
    if complete.len() < listed {
        println!(
            "{} of the {listed} frames the manifest lists are not complete; the video \
             interpolates across them",
            listed - complete.len()
        );
    }
    println!(
        "map {}: {} x {} {}, read in {read_seconds:.1} s, rip-map built in {pyramid_seconds:.1} s",
        o.sky.display(),
        sky.width(),
        sky.height(),
        map_frame.name()
    );
    if image_damaged > 0 {
        println!(
            "the map has {image_damaged} texel(s) whose value is not a finite number; they are \
             read as black"
        );
    }
    println!(
        "under-sampled pixels: {}",
        if o.mark_undersampled {
            format!("marked in {}", tally::colour_name(o.undersampled))
        } else {
            "interpolated (--undersampled interpolate)".into()
        }
    );
    // A still's video runs at its own low rate, the one picture repeated (`cli::DEFAULT_HOLD_RATE`
    // says why that rate).
    let (video_rate, repeats) = match &o.still {
        Some(still) => (still.rate, still.repeats()),
        None => (playback.frames_per_second.0, 1),
    };
    match &o.still {
        Some(still) => println!(
            "still: video frame {} of {total}, held {} s ({repeats} frame(s) at {video_rate} a \
             second), {} x {}, exposure {stops:+.2} stops, {} threads",
            range.start,
            still.seconds(),
            size.width,
            size.height,
            o.threads
        ),
        None => println!(
            "video: {} x {}, {} fps, frames {}..{} of {total}, exposure {stops:+.2} stops, {} \
             threads",
            size.width, size.height, video_rate, range.start, range.end, o.threads
        ),
    }
    if photo_path.is_some() && !o.readouts {
        println!(
            "the photograph carries no read-outs: a photograph has no subtitle track{}; \
             --readouts panel paints them on it",
            if o.subtitles {
                ", and the overlay paints nothing on the picture"
            } else {
                ""
            }
        );
    }

    let ffmpeg = match (o.codec, &plan) {
        (Some(codec), Some(plan)) => Some(Ffmpeg::start(&Encoding {
            ffmpeg: o.ffmpeg.clone(),
            codec,
            crf: o.crf.unwrap_or(codec.default_crf()),
            preset: o.preset.unwrap_or(codec.default_preset()),
            width: size.width,
            height: size.height,
            frames_per_second: video_rate,
            out: plan.partial.clone(),
        })?),
        _ => None,
    };

    // Frames travel to the writer through a queue of one, and their buffers come back to be
    // reused: at most three frames exist at once (one being rendered, one queued, one being
    // written), 201 MB each at 8K.
    const BUFFERS: usize = 3;
    let (to_writer, from_renderer) = mpsc::sync_channel::<Finished>(1);
    let (to_renderer, returned) = mpsc::channel::<Vec<u16>>();
    let job = WriterJob {
        ffmpeg,
        repeats,
        keep: o.keep_frames.clone(),
        photo: photo_path.clone().map(|p| (o.ffmpeg.clone(), p)),
        width: size.width,
        height: size.height,
    };
    let writer = std::thread::spawn(move || write_frames(job, from_renderer, to_renderer));

    let mut loaded: Vec<(usize, RayField)> = Vec::new();
    let mut allocated = 0;
    let (mut render_time, mut wait_time, mut load_time, mut readout_time) = (
        Duration::ZERO,
        Duration::ZERO,
        Duration::ZERO,
        Duration::ZERO,
    );
    let mut last_report = Instant::now();
    let loop_started = Instant::now();
    let count = range.end - range.start;
    let mut sent = 0u64;
    let mut counts = FilmTally::default();
    // The loop is a closure so that a failure inside it still reaches the join below: the writer
    // and its ffmpeg are stopped before the scratch files are removed.
    let looped = (|| -> Result<(), String> {
        for k in range.clone() {
            let pick = timeline.pick(k);
            // Before the first complete bundle frame there are no rays for this moment at all.
            let before = timeline.before_first(k);
            let wanted: Vec<usize> = match pick {
                _ if before => Vec::new(),
                Pick::One(a) => vec![a],
                Pick::Two(a, b, _) => vec![a, b],
            };
            let clock = Instant::now();
            loaded.retain(|(p, _)| wanted.contains(p));
            for &p in &wanted {
                if !loaded.iter().any(|(q, _)| *q == p) {
                    loaded.push((p, load::read_field(&bundle, complete[p], judge, o.threads)?));
                }
            }
            load_time += clock.elapsed();
            let field = |p: usize| &loaded.iter().find(|(q, _)| *q == p).expect("loaded").1;

            let clock = Instant::now();
            let mut pixels = match returned.try_recv() {
                Ok(buffer) => buffer,
                Err(_) if allocated < BUFFERS => {
                    allocated += 1;
                    vec![0u16; size.width * size.height * 3]
                }
                Err(_) => match returned.recv() {
                    Ok(buffer) => buffer,
                    // The writer has stopped: its error says why.
                    Err(_) => break,
                },
            };
            wait_time += clock.elapsed();

            let clock = Instant::now();
            let tally = if before {
                render::fill_unresolved(size, &look, &mut pixels)
            } else {
                let fields = match pick {
                    Pick::One(a) => Fields::One(field(a)),
                    Pick::Two(a, b, w) => Fields::Two(field(a), field(b), w),
                };
                let scene = Scene {
                    fields,
                    rotation,
                    sky: &sky,
                };
                render::render(&scene, size, &look, o.threads, &mut pixels)
            };
            counts.add(k, tally);
            render_time += clock.elapsed();
            if let Some(overlay) = &mut overlay {
                let clock = Instant::now();
                overlay.paint(&mut pixels, &values_at(k));
                readout_time += clock.elapsed();
            }

            if to_writer.send(Finished { k, pixels }).is_err() {
                break;
            }
            sent += 1;
            if last_report.elapsed() >= Duration::from_secs(1) || sent == count {
                last_report = Instant::now();
                let elapsed = loop_started.elapsed().as_secs_f64();
                let rate = sent as f64 / elapsed;
                let left = (count - sent) as f64 / rate;
                println!(
                    "frame {} ({sent}/{count}), {rate:.2} frames/s, {} left",
                    k,
                    duration_text(left)
                );
            }
        }
        Ok(())
    })();
    drop(to_writer);
    let written = writer
        .join()
        .map_err(|_| "the writer thread panicked".to_string())?;
    looped?;
    let (ffmpeg, writer_times) = written?;
    let loop_seconds = loop_started.elapsed().as_secs_f64();

    if let (Some(ffmpeg), Some(plan)) = (ffmpeg, &plan) {
        let clock = Instant::now();
        ffmpeg.finish()?;
        let drain = clock.elapsed().as_secs_f64();
        // An old file of that name (with --overwrite) is replaced by the final rename, and only
        // then: a failure before it leaves the old file as it was.
        let said = plan.finish(&o.ffmpeg, script.as_deref())?;
        let bytes = std::fs::metadata(&plan.video).map(|m| m.len()).unwrap_or(0);
        println!(
            "wrote {} ({:.1} MB), tagged as a 360-degree equirectangular video; ffmpeg took \
             {drain:.1} s to finish after the last frame",
            plan.video.display(),
            bytes as f64 / 1e6
        );
        for line in said {
            println!("{line}");
        }
    }
    if let Some(photo) = &photo_path {
        let bytes = std::fs::metadata(photo).map(|m| m.len()).unwrap_or(0);
        println!(
            "wrote {} ({:.1} MB), a JPEG marked as a 360-degree photograph, in {:.1} s",
            photo.display(),
            bytes as f64 / 1e6,
            writer_times.photo.as_secs_f64()
        );
    }
    let per_frame = |d: Duration| d.as_secs_f64() / sent.max(1) as f64 * 1000.0;
    println!(
        "{sent} frames in {loop_seconds:.1} s: {:.2} frames/s. Per frame: rendering {:.0} ms, \
         read-outs {:.2} ms, loading bundle frames {:.0} ms, waiting for the encoder {:.0} ms; the \
         writer spent {:.0} ms handing each frame to ffmpeg{}{}. Whole run {:.1} s.",
        sent as f64 / loop_seconds,
        per_frame(render_time),
        per_frame(readout_time),
        per_frame(load_time),
        per_frame(wait_time),
        per_frame(writer_times.encode),
        if repeats > 1 {
            format!(" ({repeats} times over)")
        } else {
            String::new()
        },
        if o.keep_frames.is_some() {
            format!(" and {:.0} ms writing PNGs", per_frame(writer_times.png))
        } else {
            String::new()
        },
        started.elapsed().as_secs_f64()
    );
    for line in counts.report(o.unresolved, o.undersampled, o.mark_undersampled) {
        println!("{line}");
    }
    if sent < count {
        return Err(format!("stopped after {sent} of {count} frames"));
    }
    println!("\nA published video made from NASA's star maps must carry this credit:\n{CREDIT}");
    Ok(())
}

/// The writer thread: each frame to ffmpeg (as many times as `job.repeats` says), to a PNG
/// master, and for a still to the photograph; then the frame's buffer back to the renderer.
fn write_frames(
    mut job: WriterJob,
    frames: mpsc::Receiver<Finished>,
    back: mpsc::Sender<Vec<u16>>,
) -> Result<(Option<Ffmpeg>, WriterTimes), String> {
    let mut times = WriterTimes::default();
    let mut scratch = Vec::new();
    for frame in frames {
        if let Some(ffmpeg) = &mut job.ffmpeg {
            let clock = Instant::now();
            for _ in 0..job.repeats {
                ffmpeg.write(&frame.pixels, &mut scratch)?;
            }
            times.encode += clock.elapsed();
        }
        if let Some(dir) = &job.keep {
            let clock = Instant::now();
            let path = dir.join(format!("frame_{:06}.png", frame.k));
            encode::write_png(&path, &frame.pixels, job.width, job.height)?;
            times.png += clock.elapsed();
        }
        if let Some((program, path)) = &job.photo {
            let clock = Instant::now();
            let jpeg = encode::jpeg(program, &frame.pixels, job.width, job.height)?;
            let jpeg = photo::with_photo_sphere(&jpeg)?;
            output::write_atomically(path, &jpeg)?;
            times.photo += clock.elapsed();
        }
        // The renderer may have finished and stopped listening; the buffer is then dropped.
        let _ = back.send(frame.pixels);
    }
    Ok((job.ffmpeg, times))
}

fn duration_text(seconds: f64) -> String {
    if !seconds.is_finite() {
        return "unknown".into();
    }
    let s = seconds.round() as u64;
    if s >= 3600 {
        format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{}:{:02}", s / 60, s % 60)
    }
}
