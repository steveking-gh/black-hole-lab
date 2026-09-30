//! sky-trace: follows an observer of a Black Hole Lab save forward from the saved moment and writes,
//! frame by frame, what that observer sees of the whole sky, as a sky bundle.
//!
//! It reads the save with `sky_trace::bhl`, walks the observer with `sky_trace::worldline`, turns
//! each event into a frame with `sky_trace::film` (which traces the light with `kerr-sky`), and
//! writes the frames with `sky_format::BundleWriter`. `--info` stops after the walk and prints
//! what a film would cover. The command line is in `args` and the file handling of a film in
//! `trace`; everything with physics in it is in the library, and these only turn its answers into
//! text and files.

mod args;
#[cfg(test)]
mod tests;
mod trace;
#[cfg(test)]
mod trace_tests;

use std::fmt::Write as _;

use kerr_equatorial::KerrSchild;
use sky_trace::bhl::{self, SavedObserver};
use sky_trace::worldline::{End, Film, Motion, Worldline};

use args::{Command, Options, Who};
use trace::Failure;

fn main() {
    // `args_os`, not `args`: `args` panics on an argument that is not Unicode, and bad input is
    // answered with a sentence here, not a panic.
    let args: Result<Vec<String>, String> = std::env::args_os()
        .skip(1)
        .map(|a| {
            a.into_string().map_err(|a| {
                format!(
                    "the argument {} is not valid Unicode; give the file another name",
                    a.to_string_lossy()
                )
            })
        })
        .collect();
    let result = args
        .map_err(Failure::Usage)
        .and_then(|args| run(&args, &mut std::io::stdout()));
    if let Err(failure) = result {
        eprintln!("sky-trace: {}", failure.message());
        // 2 for a command line or a save to fix, before anything was written; 1 for a bundle
        // whose writing failed part-way, which --resume continues.
        std::process::exit(match failure {
            Failure::Usage(_) => 2,
            Failure::Write(_) => 1,
        });
    }
}

/// Runs the program on a command line (without the program's name), printing to `out`. Returns
/// what a film wrote; `None` for help or a dry run.
pub fn run(
    args: &[String],
    out: &mut (dyn std::io::Write + Send),
) -> Result<Option<trace::Summary>, Failure> {
    match args::parse(args).map_err(Failure::Usage)? {
        Command::Help => {
            // A closed stdout is not worth a failure: the help was all there was to do.
            let _ = out.write_all(args::HELP.as_bytes());
            Ok(None)
        }
        Command::Info(options) => {
            let text = info(&options).map_err(Failure::Usage)?;
            let _ = out.write_all(text.as_bytes());
            Ok(None)
        }
        Command::Trace(options) => trace::film(&options, out).map(Some),
    }
}

/// The observer a save names, or a sentence saying the save has no such observer.
fn observer_of(save: &bhl::Save, who: Who) -> Result<&SavedObserver, String> {
    match who {
        Who::Alice => save.alice.as_ref(),
        Who::Bob => save.bob.as_ref(),
    }
    .ok_or_else(|| {
        format!(
            "{} is not in the run this save holds; try --observer {}",
            who.name(),
            match who {
                Who::Alice => "bob",
                Who::Bob => "alice",
            }
        )
    })
}

/// The dry run: read, walk, describe.
fn info(options: &Options) -> Result<String, String> {
    let save =
        bhl::read_file(&options.save).map_err(|e| format!("{}: {e}", options.save.display()))?;
    let observer = observer_of(&save, options.who)?;
    let metric = save.hole.metric();
    let mut worldline = Worldline::from_saved(&metric, observer).map_err(|e| e.to_string())?;
    let film = worldline.walk(options.step, options.frames);
    let mut text = describe(&Described {
        title: options.save.display().to_string(),
        hover: false,
        step: options.step,
        frames_given: options.frames_given,
        metric: &metric,
        save: &save,
        obs: observer,
        worldline: &worldline,
        film: &film,
    });
    text.push_str("  (a dry run: nothing has been traced and nothing written)\n");
    Ok(text)
}

/// What [`describe`] reports on.
pub struct Described<'a> {
    /// The first line: the save's path, or what the hover test is.
    pub title: String,
    /// A hover test rather than a save: no saved clock, no release.
    pub hover: bool,
    /// Proper time between frames, in M.
    pub step: f64,
    /// Whether the length was asked for.
    pub frames_given: bool,
    pub metric: &'a KerrSchild,
    pub save: &'a bhl::Save,
    pub obs: &'a SavedObserver,
    pub worldline: &'a Worldline,
    pub film: &'a Film,
}

/// What `--info` prints, and what a film prints before it starts tracing.
fn describe(d: &Described) -> String {
    let (save, obs, worldline, film, metric) = (d.save, d.obs, d.worldline, d.film, d.metric);
    let hole = save.hole;
    let unit = hole.seconds_per_unit();
    let mut out = String::new();
    let mut line = |s: String| {
        out.push_str(&s);
        out.push('\n');
    };
    line(d.title.clone());
    if let Some((version, git)) = &save.written_by {
        line(format!(
            "  written by      Black Hole Lab {version} (git {git}) at {}",
            save.saved_at_utc
        ));
    }
    if !save.note.is_empty() {
        line(format!("  note            {}", save.note));
    }
    line(format!(
        "  hole            M = {}, a = {} (a/M = {:.6}), {} solar masses",
        hole.m,
        hole.a,
        hole.a / hole.m,
        hole.m_solar
    ));
    line(format!(
        "                  r+ = {:.6} M, r- = {:.6} M, static limit 2M = {} M",
        metric.outer_horizon(),
        metric.inner_horizon(),
        2.0 * metric.m
    ));
    line(format!(
        "  time unit       1 M = {unit:.6} s (from GM_sun/c^3 = {:e} s)",
        bhl::GM_SUN_OVER_C3_SECONDS
    ));
    if d.hover {
        line(format!(
            "  observer        {}, held at r and phi for good",
            obs.name
        ));
    } else {
        line(format!("  saved clock     t = {} M", save.clock));
        line(format!(
            "  observer        {}, {}; {}",
            obs.name,
            obs.mode.name(),
            obs.release.phrase()
        ));
    }
    if let Some((energy, l_ang)) = worldline.constants() {
        line(format!(
            "                  falls with E = {energy:.9}, L = {l_ang:.9} M"
        ));
    }
    if !d.hover {
        match worldline.tau_release() {
            Some(tau_release) => line(format!(
                "                  held at r = {} M until the release at t = {} M (tau = \
                 {tau_release:.6} M), then {}",
                obs.r,
                obs.release_t,
                worldline.motion_after_release().name()
            )),
            None => line(format!(
                "                  {} from the saved moment on",
                worldline.motion_after_release().name()
            )),
        }
        line(format!(
            "  saved moment    t = {:.9} M, r = {:.9} M, phi = {:.9}, tau = {:.9} M",
            obs.t, obs.r, obs.phi, obs.tau
        ));
    }

    let frames = film.events.len();
    line(format!(
        "  frames          {frames}, one every {} M of proper time ({:.6} s)",
        d.step,
        d.step * unit
    ));
    if let (Some(first), Some(last)) = (film.events.first(), film.events.last()) {
        let dtau = last.tau - first.tau;
        let dt = last.t - first.t;
        line(format!(
            "  proper time     tau = {:.9} to {:.9} M: {dtau:.6} M, {:.6} s",
            first.tau,
            last.tau,
            dtau * unit
        ));
        line(format!(
            "  coordinate time t = {:.9} to {:.9} M: {dt:.6} M, {:.6} s",
            first.t,
            last.t,
            dt * unit
        ));
        line(format!(
            "  radius          r = {:.9} M at the first frame, {:.9} M at the last",
            first.r, last.r
        ));
        if let Some(k) = film.events.iter().position(|e| e.motion != Motion::Holding)
            && k > 0
        {
            line(format!(
                "  release         between frames {} and {k}",
                k - 1
            ));
        }
    }
    let mut end = String::new();
    let _ = write!(end, "{}", film.end);
    if let End::Frames { .. } = film.end
        && !d.frames_given
    {
        end.push_str(" (the default limit; --frames sets another, and the worldline goes on)");
    }
    line(format!("  ends            {end}"));
    out
}
