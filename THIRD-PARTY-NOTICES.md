# Third-party notices

Black Hole Lab is licensed under the GNU General Public License, version 3 or later; the full text
is in `LICENSE.TXT`. This file records what the program is built from and on what terms — partly to
satisfy the notice conditions those terms carry (Apache-2.0 §4, the notice clauses of the MIT and
BSD licences, the attribution conditions of the SIL Open Font License and the Ubuntu Font Licence),
and partly so that anyone redistributing the program knows what they are passing on.

Nothing here restricts the GPLv3 licensing of Black Hole Lab's own source. Every Rust crate it links
is under a permissive licence, and where a crate offers a choice, the option compatible with GPLv3
is the one taken.

## Why version 3 and not version 2

Fourteen crates in the graph are Apache-2.0 **only**, with no MIT alternative offered:

`ab_glyph`, `ab_glyph_rasterizer`, `accesskit_winit`, `approx`, `codespan-reporting`, `gethostname`,
`glutin_wgl_sys`, `nalgebra`, `nalgebra-macros`, `owned_ttf_parser`, `simba`, `spirv`,
`unicode-general-category`, `winit`

Apache-2.0 can be combined into a GPLv3 work — the compatibility runs one way, and the result is
distributed under the GPL — but it is *not* compatible with GPLv2, whose terms cannot accommodate
Apache-2.0's patent-termination clause. A GPLv2 licensing of this program would therefore have been
unavailable while it depends on nalgebra and winit. GPLv3 is the earliest GPL that works here.

## Rust crates

The dependency graph resolved for all target platforms holds 397 crate versions, 360 distinct
crates, grouped below by the licence each one declares in its published manifest. An `OR` is the
crate offering a choice; an `AND` is it requiring both.

One entry deserves a note. **`self_cell`** declares `Apache-2.0 OR GPL-2.0-only`, and is used here
under **Apache-2.0**. Automated licence scanners tend to flag the GPL-2.0-only branch as
incompatible with GPLv3; that branch is not the one taken.

**MIT OR Apache-2.0** (162)
: `accesskit`, `accesskit_atspi_common`, `accesskit_consumer`, `accesskit_macos`, `accesskit_unix`, `accesskit_windows`, `ahash`, `allocator-api2`, `android-activity`, `android_system_properties`, `arboard`, `arrayvec`, `as-raw-xcb-connection`, `ash`, `async-broadcast`, `async-recursion`, `async-trait`, `bitflags`, `bumpalo`, `cfg-if`, `core-foundation`, `core-foundation-sys`, `core-graphics`, `core-graphics-types`, `crc32fast`, `crossbeam-utils`, `displaydoc`, `document-features`, `ecolor`, `eframe`, `egui`, `egui-wgpu`, `egui-winit`, `either`, `emath`, `enumflags2`, `enumflags2_derive`, `epaint`, `errno`, `euclid`, `fdeflate`, `flate2`, `font-types`, `form_urlencoded`, `futures-core`, `futures-io`, `futures-macro`, `futures-task`, `futures-util`, `getrandom`, `gpu-allocator`, `half`, `hashbrown`, `hermit-abi`, `hex`, `idna`, `image`, `itertools`, `itoa`, `jni`, `jni-macros`, `jni-sys`, `jni-sys-macros`, `js-sys`, `libc`, `litrs`, `lock_api`, `log`, `memmap2`, `naga`, `naga-types`, `ndk`, `ndk-context`, `ndk-sys`, `num-complex`, `num-integer`, `num-rational`, `num-traits`, `once_cell`, `ordered-stream`, `parking_lot`, `parking_lot_core`, `percent-encoding`, `piper`, `png`, `polycool`, `presser`, `proc-macro-crate`, `proc-macro2`, `profiling`, `quote`, `range-alloc`, `raw-window-metal`, `read-fonts`, `renderdoc-sys`, `rustversion`, `scopeguard`, `serde`, `serde_core`, `serde_derive`, `serde_json`, `serde_repr`, `signal-hook-registry`, `simdutf8`, `skrifa`, `smallvec`, `smol_str`, `stable_deref_trait`, `static_assertions`, `syn`, `tempfile`, `thiserror`, `thiserror-impl`, `toml_datetime`, `toml_edit`, `toml_parser`, `ttf-parser`, `typenum`, `unicode-segmentation`, `unicode-width`, `url`, `wasm-bindgen`, `wasm-bindgen-futures`, `wasm-bindgen-macro`, `wasm-bindgen-macro-support`, `wasm-bindgen-shared`, `web-sys`, `web-time`, `webbrowser`, `weezl`, `wgpu`, `wgpu-core`, `wgpu-core-deps-apple`, `wgpu-core-deps-emscripten`, `wgpu-core-deps-wasm`, `wgpu-core-deps-windows-linux-android`, `wgpu-hal`, `wgpu-naga-bridge`, `wgpu-types`, `windows`, `windows-collections`, `windows-core`, `windows-future`, `windows-implement`, `windows-interface`, `windows-link`, `windows-numerics`, `windows-result`, `windows-strings`, `windows-sys`, `windows-targets`, `windows-threading`, `windows_aarch64_gnullvm`, `windows_aarch64_msvc`, `windows_i686_gnu`, `windows_i686_gnullvm`, `windows_i686_msvc`, `windows_x86_64_gnu`, `windows_x86_64_gnullvm`, `windows_x86_64_msvc`, `x11rb`, `x11rb-protocol`

**MIT** (68)
: `android-properties`, `block2`, `bytes`, `calloop`, `calloop-wayland-source`, `combine`, `crunchy`, `dispatch`, `dlib`, `endi`, `fax`, `harfrust`, `libm`, `libredox`, `memoffset`, `objc-sys`, `objc2`, `objc2-app-kit`, `objc2-encode`, `objc2-foundation`, `objc2-ui-kit`, `orbclient`, `ordered-float`, `phf`, `phf_generator`, `phf_macros`, `phf_shared`, `quick-xml`, `redox_syscall`, `rfd`, `sctk-adwaita`, `simd-adler32`, `slab`, `smithay-client-toolkit`, `smithay-clipboard`, `strict-num`, `synstructure`, `tiff`, `tracing`, `tracing-attributes`, `tracing-core`, `uds_windows`, `wayland-backend`, `wayland-client`, `wayland-csd-frame`, `wayland-cursor`, `wayland-protocols`, `wayland-protocols-experimental`, `wayland-protocols-misc`, `wayland-protocols-plasma`, `wayland-protocols-wlr`, `wayland-scanner`, `wayland-sys`, `winnow`, `x11-dl`, `xcursor`, `xkbcommon-dl`, `zbus`, `zbus-lockstep`, `zbus-lockstep-macros`, `zbus_macros`, `zbus_names`, `zbus_xml`, `zcheapstr`, `zmij`, `zvariant`, `zvariant_derive`, `zvariant_utils`

**Apache-2.0 OR MIT** (41)
: `async-channel`, `async-executor`, `async-io`, `async-lock`, `async-process`, `async-signal`, `async-task`, `atomic-waker`, `atspi`, `atspi-common`, `atspi-proxies`, `bit-set`, `bit-vec`, `blocking`, `color`, `concurrent-queue`, `equivalent`, `event-listener`, `event-listener-strategy`, `fastrand`, `fearless_simd`, `futures-lite`, `idna_adapter`, `indexmap`, `kurbo`, `linebender_resource_handle`, `nohash-hasher`, `parking`, `peniko`, `pin-project`, `pin-project-internal`, `pin-project-lite`, `polling`, `portable-atomic`, `portable-atomic-util`, `rustc-hash`, `simd_cesu8`, `utf8_iter`, `uuid`, `vello_common`, `vello_cpu`

**Unicode-3.0** (18)
: `icu_collections`, `icu_locale_core`, `icu_normalizer`, `icu_normalizer_data`, `icu_properties`, `icu_properties_data`, `icu_provider`, `litemap`, `potential_utf`, `tinystr`, `writeable`, `yoke`, `yoke-derive`, `zerofrom`, `zerofrom-derive`, `zerotrie`, `zerovec`, `zerovec-derive`

**Apache-2.0** (14)
: `ab_glyph`, `ab_glyph_rasterizer`, `accesskit_winit`, `approx`, `codespan-reporting`, `gethostname`, `glutin_wgl_sys`, `nalgebra`, `nalgebra-macros`, `owned_ttf_parser`, `simba`, `spirv`, `unicode-general-category`, `winit`

**MIT/Apache-2.0** (14)
: `bitflags`, `downcast-rs`, `foreign-types`, `foreign-types-macros`, `foreign-types-shared`, `guillotiere`, `khronos-egl`, `matrixmultiply`, `plain`, `quick-error`, `rawpointer`, `scoped-tls`, `siphasher`, `type-map`

**Zlib OR Apache-2.0 OR MIT** (10)
: `bytemuck`, `bytemuck_derive`, `objc2-app-kit`, `objc2-core-foundation`, `objc2-core-graphics`, `objc2-metal`, `objc2-quartz-core`, `objc2-ui-kit`, `safe_arch`, `wide`

**MIT OR Apache-2.0 OR Zlib** (6)
: `cursor-icon`, `glow`, `raw-window-handle`, `xkeysym`, `zune-core`, `zune-jpeg`

**Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT** (4)
: `linux-raw-sys`, `rustix`, `wasip2`, `wit-bindgen`

**Apache-2.0/MIT** (2)
: `pollster`, `rustc-hash`

**BSD-2-Clause OR Apache-2.0 OR MIT** (2)
: `zerocopy`, `zerocopy-derive`

**BSD-3-Clause** (2)
: `tiny-skia`, `tiny-skia-path`

**BSD-3-Clause OR Apache-2.0** (2)
: `moxcms`, `pxfm`

**BSD-3-Clause OR MIT OR Apache-2.0** (2)
: `num_enum`, `num_enum_derive`

**BSL-1.0** (2)
: `clipboard-win`, `error-code`

**Unlicense OR MIT** (2)
: `byteorder-lite`, `memchr`

**Zlib** (2)
: `foldhash`, `slotmap`

**(MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0** (1)
: `epaint_default_fonts`

**(MIT OR Apache-2.0) AND Unicode-3.0** (1)
: `unicode-ident`

**0BSD OR MIT OR Apache-2.0** (1)
: `adler2`

**Apache-2.0 AND MIT** (1)
: `dpi`

**Apache-2.0 OR GPL-2.0-only** (1)
: `self_cell`

**BSD-2-Clause** (1)
: `arrayref`

**ISC** (1)
: `libloading`

**MIT OR Apache-2.0 OR LGPL-2.1-or-later** (1)
: `r-efi`

**MIT OR Zlib OR Apache-2.0** (1)
: `miniz_oxide`

## Fonts

Font files are data rather than code: they are not derived from this program, do not link against
it, and are distributed under their own unmodified terms alongside it. No font here is modified and
no reserved font name is reused. They arrive by two routes.

### Bundled in this repository

In `assets/fonts/`, embedded by `install_fonts` in `src/main.rs`:

| Font | Licence | Text |
| --- | --- | --- |
| Atkinson Hyperlegible, Regular and Bold | SIL Open Font License 1.1 | `assets/fonts/OFL-AtkinsonHyperlegible.txt` |
| DejaVu Sans | Bitstream Vera Fonts copyright; DejaVu changes in the public domain; Arev glyphs © Tavmjong Bah | `assets/fonts/LICENSE-DejaVu.txt` |

### Supplied by egui

The `epaint_default_fonts` 0.36.2 crate, which `egui` pulls in through its `default_fonts` feature,
compiles four more faces into the binary. Their licence texts ship inside that crate, in its
`fonts/` directory:

| Font | Licence | Text |
| --- | --- | --- |
| Ubuntu-Light | Ubuntu Font Licence 1.0 | `fonts/UFL.txt` |
| Hack Regular | MIT, © 2018 Source Foundry Authors | `fonts/Hack-Regular.txt` |
| NotoEmoji Regular | SIL Open Font License 1.1 | `fonts/OFL.txt` |
| emoji-icon-font | MIT, © 2014 John Slegers | `fonts/emoji-icon-font-mit-license.txt` |

The Ubuntu Font Licence is a free licence, but unlike the OFL it is not generally regarded as
GPL-compatible — Fedora, for instance, classifies it as acceptable for fonts while noting it is not
compatible with the GPL. It governs the Ubuntu-Light file only, on the terms above.

These four faces are not decoration that could simply be dropped. egui's default set is where the
interface gets the glyphs its buttons and headings are drawn with — ⏮ (U+23EE), ⏪ (U+23EA),
➖ (U+2796), ➕ (U+2795), 📏 (U+1F4CF), 📐 (U+1F4D0), 🔤 (U+1F524) among many others — and none of
those code points exists in Atkinson Hyperlegible or DejaVu Sans. Building with
`default-features = false` to exclude the set would leave those labels blank.

## Regenerating this list

The crate names and licences above are read from the resolved graph and the published manifests,
not maintained by hand:

```sh
cargo tree -e normal --target all --prefix none | sed 's/ (\*)$//' | sort -u
# then, for each crate, the `license` field of
# ~/.cargo/registry/src/index.crates.io-*/<name>-<version>/Cargo.toml
```
