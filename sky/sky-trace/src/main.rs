//! sky-trace: follows an observer of a Black Hole Lab save forward from the saved moment and will
//! write, frame by frame, what that observer sees of the whole sky.
//!
//! At this step it does a dry run only (`--info`): it reads the save with `sky_trace::bhl`, walks
//! the observer with `sky_trace::worldline`, and prints what a film would cover. The command line
//! is in `args`; everything with physics in it is in the library, and this file only turns its
//! answers into text.

mod args;
#[cfg(test)]
mod tests;

use std::fmt::Write as _;

use sky_trace::bhl::{self, SavedObserver};
use sky_trace::worldline::{End, Film, Motion, Worldline};

use args::{Command, Options, Who};

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
    let result = args.and_then(|args| match args::parse(&args)? {
        Command::Help => Ok(args::HELP.to_string()),
        Command::Info(options) => info(&options),
    });
    match result {
        Ok(text) => print!("{text}"),
        Err(message) => {
            eprintln!("sky-trace: {message}");
            std::process::exit(2);
        }
    }
}

/// The dry run: read, walk, describe.
fn info(options: &Options) -> Result<String, String> {
    let save =
        bhl::read_file(&options.save).map_err(|e| format!("{}: {e}", options.save.display()))?;
    let observer = match options.who {
        Who::Alice => save.alice.as_ref(),
        Who::Bob => save.bob.as_ref(),
    }
    .ok_or_else(|| {
        format!(
            "{} is not in the run this save holds; try --observer {}",
            options.who.name(),
            match options.who {
                Who::Alice => "bob",
                Who::Bob => "alice",
            }
        )
    })?;
    let metric = save.hole.metric();
    let mut worldline = Worldline::from_saved(&metric, observer).map_err(|e| e.to_string())?;
    let film = worldline.walk(options.step, options.frames);

    // ------------------------------------------------------------------------------------------
    // THE TRACER GOES HERE (a later step of phase 2, once kerr-sky has been reviewed).
    //
    // For each event of `film.events`, in order: build the observer's triad from the event's
    // 4-velocity (x along the inward radial direction of the observer's rest space, z along the
    // spin axis, rebuilt at every frame: the user's decisions 2 and 3), trace the frame with
    // kerr-sky, and write it with sky-format, resumably. The walk above is complete before the
    // first frame is traced, so a film that ends early is known to before any tracing is spent
    // on it. Until then this program reports and writes nothing.
    // ------------------------------------------------------------------------------------------

    Ok(describe(options, &save, observer, &worldline, &film))
}

/// What `--info` prints.
fn describe(
    options: &Options,
    save: &bhl::Save,
    obs: &SavedObserver,
    worldline: &Worldline,
    film: &Film,
) -> String {
    let hole = save.hole;
    let metric = hole.metric();
    let unit = hole.seconds_per_unit();
    let mut out = String::new();
    let mut line = |s: String| {
        out.push_str(&s);
        out.push('\n');
    };
    line(format!("{}", options.save.display()));
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
    line(format!("  saved clock     t = {} M", save.clock));

    line(format!(
        "  observer        {}, {}; {}",
        obs.name,
        obs.mode.name(),
        obs.release.phrase()
    ));
    if let Some((energy, l_ang)) = worldline.constants() {
        line(format!(
            "                  falls with E = {energy:.9}, L = {l_ang:.9} M"
        ));
    }
    match worldline.tau_release() {
        Some(tau_release) => line(format!(
            "                  held at r = {} M until the release at t = {} M (tau = {tau_release:.6} M), \
             then {}",
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

    let frames = film.events.len();
    line(format!(
        "  frames          {frames}, one every {} M of proper time ({:.6} s)",
        options.step,
        options.step * unit
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
        && !options.frames_given
    {
        end.push_str(" (the default limit; --frames sets another, and the worldline goes on)");
    }
    line(format!("  ends            {end}"));
    line("  (a dry run: nothing has been traced and nothing written)".to_string());
    out
}
