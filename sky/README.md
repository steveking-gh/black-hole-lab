# The sky pipeline

This directory holds the programs that make a 360-degree video of what an observer near a black
hole sees. It is a cargo workspace of its own. Black Hole Lab, at the repository root, does not
depend on anything here, and nothing here can change how the app builds or runs.

## The parts

| Directory | Kind | Purpose |
| --- | --- | --- |
| `sky-format` | library | Reads and writes a *sky bundle*, the files that pass from a tracer to the renderer |
| `sky-testgen` | program | Writes sky bundles for flat-space test cases whose right answer is known |
| `kerr-sky` | library | Traces light backward through Kerr spacetime, off the equatorial plane |
| `sky-trace` | program | Follows an observer from a Black Hole Lab save, or a hovering one, and writes the sky bundle of what the observer sees |
| `sky-render` | program | Turns a sky bundle and a star map into a 360-degree AV1 video |
| `maps` | data | The script that downloads NASA's star maps, and their description |

The format of a sky bundle is defined in `physics_simulation_specification.md`, at the repository
root.

## How the parts connect

```text
              sky-testgen  ─┐
                            ├─►  sky bundle  ─►  sky-render  ─►  video.mp4
save.bhl  ─►  sky-trace    ─┘                        ▲
                                                 star map
```

A *tracer* is any program that writes a sky bundle. `sky-testgen` is one, for flat space.
`sky-trace` is the one for Kerr spacetime: it reads a save of Black Hole Lab, follows the chosen
observer forward from the saved moment, and traces the light with `kerr-sky`.

## Running it

Run cargo from this directory, not from the repository root.

```sh
cd sky
cargo test                                   # every crate of the pipeline
pwsh maps/fetch-sky.ps1                      # download the default star map, once

cargo run --release -p sky-testgen -- --case turn --out ../../bundles/turn
cargo run --release -p sky-render -- --bundle ../../bundles/turn \
    --sky maps/starmap_2020_8k_gal.exr --out ../../turn.mp4
```

Each program prints its options with `--help`. `sky-render` needs `ffmpeg` on the `PATH`.

**Do not run `cargo fmt --all` here.** `--all` follows path dependencies out of this workspace:
it reformats `../crates/kerr-equatorial` and `../crates/readout`, and through their workspace the
app's own source. Format the crates of this workspace by name:

```sh
cargo fmt -p sky-format -p sky-testgen -p sky-render -p kerr-sky -p sky-trace
```

## Why a separate workspace

Cargo builds the members of a workspace together. Sharing the app's workspace would let these
crates change which features the app's dependencies are built with, add their dependencies to the
app's lock file, and make `cargo run` at the repository root ambiguous between the app and the
programs here. Kept apart, the app's manifest and lock file do not mention this directory.

Two crates in `../crates` are used from here, read-only: `kerr-equatorial`, so that the light
traced here agrees with the app's geometry, and `readout`, so that numbers on the video are
written as the app writes them.

## Licences and credit

The crates here are licensed as the app is, GPL-3.0-or-later. `THIRD-PARTY-NOTICES.md` in this
directory lists what they are built from, and gives the credit line that a published video made
from NASA's star maps must carry.
