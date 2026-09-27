# The sky bundle format, version 1

This document defines the sky bundle: the files that pass from a program that traces light near a
black hole to a program that renders what an observer sees. It is normative. The words **must**,
**must not** and **may** state requirements on writers and readers; everything else explains.

The Rust crate `crates/sky-format` implements this document. Where the crate and this document
disagree, this document is right and the crate has a bug.

## 1. Purpose

An *observer* is a person, or a camera, moving through spacetime near a black hole. The observer
carries a watch that shows the observer's *proper time*, the time the observer lives through.

A *tracer* is a program that follows light rays backward in time from the observer's eye. For each
moment of the watch it chooses a set of directions on the observer's sky, and for each direction it
records where on the distant sky the light came from, how much the light's frequency changed on
the way, and whether the light came from the distant sky at all. Some light does not: a ray
traced backward can end on the black hole's horizon, and that part of the sky is the hole's
shadow.

A *renderer* is a program that turns those records and a map of the stars into a 360-degree video.
The renderer knows no physics. Everything it needs is in the bundle.

A *sky bundle* (or *bundle*) is that record, and this document defines it. The tracer is a
*writer* of bundles; the renderer is a *reader*.

## 2. The shape of a bundle

A bundle is a directory:

```text
manifest.json                what the run is, and which frames are written
frames/000000.skyframe       frame 0
frames/000001.skyframe       frame 1
...
```

A *frame* is the record for one moment of the observer's watch. Frames are numbered from 0 by
their *index*. The *frame file* of frame `index` is named `frames/` followed by the index written
in decimal, zero-padded to six digits, followed by `.skyframe`. An index of a million or more is
written with as many digits as it needs: frame 1234567 is `frames/1234567.skyframe`.

`manifest.json` (section 5) describes the whole run and lists the frames written so far. Each frame
file (section 7) holds the rays of one frame, as planes of numbers.

A bundle is a directory, not a single file, so that a crash loses at most one frame and a run can
be resumed (section 9).

## 3. Terms

- A *ray* is one light ray arriving at the observer. The tracer follows it backward from the
  observer to where it came from.
- The *far sky* is the sphere of distant stars, far enough away that every ray that escapes the
  black hole reaches it travelling radially.
- The *observer's sky* is the sphere of directions the observer can look in.
- The *grid* is the set of directions on the observer's sky at which the tracer traces rays
  (section 4.3). It has `W` columns and `H` rows. A *pixel* is one cell of the grid, and each pixel
  has one ray, through its centre.
- A *plane* is one number per pixel, for every pixel of the grid, in the grid's order.
- The *bundle's time unit* is the unit in which every time in the bundle is written (section 4.1).

## 4. Conventions

### 4.1 Time

Every time in a bundle is a number of the bundle's time unit. The manifest declares the unit once,
in `time_unit`, as a name and the number of SI seconds in one unit:

- For a Kerr black hole of mass `M`, the unit is `M`, and one unit is `G M / c^3` seconds. For
  one solar mass that is 4.925490947e-6 s; for Sagittarius A* (4.15e6 solar masses) it is about
  20.44 s.
- For flat space the unit is the second, and `seconds` is 1.

Lengths follow from times with `c = 1`: in a Kerr bundle, a radius of 6 means 6 `G M / c^2`.

The *stopwatch* is the observer's proper time since frame 0. It is the first read-out (section 6).

### 4.2 The observer's triad

The *triad* is three unit vectors `x`, `y`, `z` in the observer's rest space: orthonormal and
right-handed, so that `x × y = z`. Every direction on the observer's sky is written in the triad.

- `z` is the pole of the observer's sky: "up".
- `x` is heading zero: the centre of the video frame, the direction the viewer faces when the video
  opens.
- `y = z × x` completes the triad. When `z` is up and the observer faces along `x`, `y` points to
  the observer's **left**.

The *azimuth* `phi` of a direction is measured about `z`, from `x` toward `y`. It therefore
increases to the observer's left.

Version 1 requires only that the triad be orthonormal and right-handed in the observer's rest
space at every frame. How the triad is carried from one frame to the next is the writer's choice,
and the choice shows in the video: a triad tied to the hole's radial direction turns with the
observer's orbit, and a triad held by gyroscopes does not.

**In Kerr** the usual choice ties the triad to the hole: `z` along the spin axis at the observer,
in the sense of the hole's angular momentum; `x` along the outward radial direction; `y` along
the prograde azimuthal direction, the sense in which the hole rotates. Even that choice is not
unique. A moving observer sees the radial, azimuthal and polar directions of a reference frame
only after a Lorentz boost, and different reference frames give triads that differ by a rotation.
No single reference frame serves everywhere: the zero-angular-momentum observer (ZAMO), for
example, exists only outside the event horizon. A Kerr writer therefore **should** state in the
manifest's `observer.triad` (section 5.9) which reference frame the triad was built from, by
which boost, and how the triad is carried between frames.

### 4.3 The observer-sky grid

The grid is *equirectangular*: longitude across, latitude down. Column `i` runs from 0 at the left
of the frame to `W - 1` at the right; row `j` runs from 0 at the top to `H - 1` at the bottom. The
ray of pixel `(i, j)` passes through the pixel's centre, which has longitude `lambda` and latitude
`beta`:

```text
lambda_i = ((i + 0.5) / W - 0.5) * 2 pi        column i, left to right
beta_j   = (0.5 - (j + 0.5) / H) * pi          row j, top row nearest +z
```

The ray's direction `n`, written in the triad, is

```text
phi = -lambda_i
n   = (cos beta_j cos phi, cos beta_j sin phi, sin beta_j)
```

`n` is the direction in which the observer **looks** to see the pixel. The light seen there arrives
travelling along `-n`.

The sign flip between `phi` and `lambda` makes longitude increase to the right of the frame while
azimuth increases to the left. The consequences are the ones a 360-degree video player expects:

- The centre of the frame, `lambda = 0`, looks along `+x`.
- Moving right in the frame turns the view toward `-y`, the observer's right.
- The top row is nearest `+z`, and the bottom row nearest `-z`.
- The left and right edges of the frame, `lambda = -pi` and `lambda = +pi`, both look along `-x`,
  behind the observer.

The same layout is the one a star map has when the sky is drawn as seen from inside the celestial
sphere, with longitude increasing to the left.

**Frame coordinates.** A place in the frame that is not a pixel centre is written in *frame
coordinates* `(u, v)`: `u` runs from 0 at the frame's left edge to `W` at its right edge, and `v`
from 0 at the top edge to `H` at the bottom edge. The centre of pixel `(i, j)` is at
`(u, v) = (i + 0.5, j + 0.5)`. The longitude and latitude at `(u, v)` are

```text
lambda = (u / W - 0.5) * 2 pi
beta   = (0.5 - v / H) * pi
```

which at a pixel centre are the formulae above. The inverse, from a direction
`n = (n_x, n_y, n_z)` (not necessarily of unit length, but not zero) to frame coordinates, is

```text
beta   = atan2(n_z, sqrt(n_x^2 + n_y^2))
phi    = atan2(n_y, n_x)                  in (-pi, pi]
lambda = -phi                             in [-pi, pi)
u      = (lambda / (2 pi) + 0.5) * W      in [0, W)
v      = (0.5 - beta / pi) * H            in [0, H]
```

The seam behind the observer comes out at the left edge, `u = 0`. A reader that samples across
the seam wraps `u` modulo `W`. No pixel centre lies on either pole.

This document uses no other continuous coordinate. In particular it never counts from a pixel's
centre, so that a number carried between this format and a star map described the same way cannot
be half a pixel out.

**Storage order.** Every plane stores its `W * H` values row by row, top row first, each row left
to right. The value of pixel `(i, j)` is at position

```text
k = j * W + i
```

The grid may be coarser than the video the renderer makes; the renderer interpolates between rays.

### 4.4 The far-sky frame

The *far-sky frame* is a right-handed set of axes `X`, `Y`, `Z` fixed to the distant stars.

- In Kerr, `Z` is the black hole's spin axis, pointing along its angular momentum, and `X` is the
  direction of azimuth zero of the writer's coordinates, at large radius.
- In flat space the writer chooses the axes.

The manifest's `far_sky` field (section 5.10) places the far-sky frame on the real sky. It gives
each far-sky axis as a unit vector in the International Celestial Reference System (ICRS). A
direction `d = (d_X, d_Y, d_Z)` in the far-sky frame is, in ICRS,

```text
d_ICRS = d_X * axes_in_icrs.x + d_Y * axes_in_icrs.y + d_Z * axes_in_icrs.z
```

The three vectors **must** be orthonormal and right-handed to within 1e-9.

**The galactic preset.** The default orientation, named `"galactic"`, puts the Milky Way on the
frame's equator: `X` points at the galactic centre, `Y` at galactic longitude 90 degrees, and `Z` at
the north galactic pole. Its axes are the rows of the matrix A_G of the Hipparcos catalogue (ESA
1997, *The Hipparcos and Tycho Catalogues*, volume 1, section 1.5.3):

```text
x = (-0.0548755604162154, -0.8734370902348850, -0.4838350155487132)
y = ( 0.4941094278755837, -0.4448296299600112,  0.7469822444972189)
z = (-0.8676661490190047, -0.1980763734312015,  0.4559837761750669)
```

### 4.5 The direction at infinity

The *direction at infinity* `d` of a ray is the unit vector, in the far-sky frame, that points from
the black hole toward the point of the far sky that the ray reaches when it is traced backward. It
is the point of the celestial sphere the light came from.

In Kerr, a ray that escapes reaches large radius at limiting polar angle `theta_inf` and azimuth
`phi_inf`, and

```text
d = (sin theta_inf cos phi_inf, sin theta_inf sin phi_inf, cos theta_inf)
```

In flat space, for an observer at rest whose triad is aligned with the far-sky axes, `d = n`.

`d` is stored as a vector, not as two angles, so that a reader can interpolate between rays
component by component and renormalise, without trouble at the poles or at the seam.

### 4.6 The shift g

The *shift* of a ray is

```text
g = nu_observed / nu_emitted
```

where `nu_emitted` is the frequency of the light as measured by an emitter at rest in the far-sky
frame, far from the hole, and `nu_observed` is its frequency as the observer measures it. `g > 1` is
a blueshift and `g < 1` a redshift.

A renderer uses `g` to scale the sky's brightness. The invariance of `I_nu / nu^3` along a ray
means that the bolometric intensity the observer sees is `g^4` times the intensity of the map, and
that the specific intensity at observed frequency `nu` is

```text
I_nu,observed(nu) = g^3 * I_nu,map(nu / g)
```

### 4.7 Fate codes

The *fate* of a ray says what the ray reached when traced backward. It is one byte:

| Code | Name | Meaning |
|---|---|---|
| 0 | unresolved | The tracer gave up before the ray reached anything. |
| 1 | far sky | The ray reached the far sky. |
| 2 | past horizon | The ray came out of the black hole's past horizon: it is part of the shadow. |
| 3 to 255 | | Reserved. A reader treats a reserved code as unresolved. |

`d` and `g` are meaningful only for fate 1. For every other fate a writer **must** store NaN in all
three components of `d` and in `g`. A writer **may** use the NaN's payload bits for its own
diagnostics; version 1 gives them no meaning, and a reader **must not** rely on them.

### 4.8 Winding

The *winding* of a ray counts the whole turns the light made about the far-sky `Z` axis on its way
from the far sky to the observer. With `Delta phi` the change in azimuth about `Z` along the ray,
measured in the direction the light travels (from the far sky to the observer) and positive in the
right-handed sense about `+Z` (prograde: the sense in which the hole rotates),

```text
winding = trunc(Delta phi / (2 pi))      rounded toward zero
```

stored as a signed 16-bit integer and clamped to [-32767, 32767].

In Kerr the azimuth is the `phi` of ingoing Kerr-Schild coordinates. That coordinate is regular
across the event and Cauchy horizons, so the winding is defined for an observer anywhere the tracer
can place one. (Boyer-Lindquist `phi` is not regular at the horizons.) Because only the change of
`phi` along the ray counts, the constant offset between the two azimuths does not matter.

Winding is meaningful only for fate 1. For every other fate a writer **must** store 0. In flat space
every winding is 0.

### 4.9 Numbers in JSON

JSON has no infinity and no NaN. In `manifest.json`:

- A finite number is written as a JSON number. A writer **should** write the shortest decimal that
  reads back as the same 64-bit float, so that the value round-trips to the bit.
- Positive infinity, negative infinity and NaN are written as the JSON strings `"inf"`, `"-inf"` and
  `"nan"`.
- A reader **must** accept those three strings wherever a number is allowed. It **should** also
  accept `"+inf"`, `"Infinity"`, `"-Infinity"` and `"NaN"`, a decimal number written as a string,
  and a JSON integer where a number is expected.

This is the rule of Black Hole Lab's `.bhl` save files, so that one reader serves both formats. In
the tables below, *number* means a value written by this rule, and *integer* means a JSON integer.

## 5. The manifest

`manifest.json` is one JSON object, encoded in UTF-8. A reader **must** ignore any field it does not
know, at any depth (section 10).

### 5.1 Top level

| Field | Type | Required | Meaning |
|---|---|---|---|
| `format` | string | yes | Always `"black-hole-lab-sky"`. A manifest with any other value is not a sky bundle's. |
| `version` | integer | yes | The format version the bundle was written in: `1`. |
| `writer` | object | yes | The program that wrote the bundle (5.2). |
| `source` | object | yes | What the bundle is a picture of (5.3). |
| `geometry` | object | yes | The spacetime the rays were traced through (5.4). |
| `time_unit` | object | yes | The bundle's time unit (5.5). |
| `observer` | object | no | Who is looking, and how the triad was built (5.9). |
| `grid` | object | yes | The observer-sky grid (5.6). |
| `far_sky` | object | yes | The far-sky frame's orientation on the real sky (5.10). |
| `playback` | object | yes | How the frames map onto a video (5.7). |
| `readouts` | array | yes | The read-outs, each declared once (section 6). |
| `labels` | array | no | Names that point sources can carry (section 8). Absent means empty. |
| `frames_planned` | integer | no | How many frames the run means to write. For progress displays only. |
| `frames` | array | yes | The frames written so far (5.8). May be empty. |

### 5.2 `writer`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `program` | string | yes | The writer's name. |
| `version` | string | yes | The writer's version. |
| `git` | string | no | The source revision the writer was built from. |

### 5.3 `source`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `kind` | string | yes | `"scenario"` for a run of Black Hole Lab, `"flat-space test"` for a generated test case. Other values are allowed. |
| `name` | string | no | The scenario's or the test case's name. |
| `state_hash` | string | no | The state hash of the `.bhl` save file the run started from. |

### 5.4 `geometry`

A renderer needs none of this. It is here so that a bundle says what it shows.

| Field | Type | Required | Meaning |
|---|---|---|---|
| `kind` | string | yes | `"flat"` or `"kerr"`. Other values are allowed, and a reader treats them as descriptive. |
| `mass` | number | for Kerr | `M`, in the bundle's time unit. In a bundle whose unit is `M`, this is 1. |
| `spin` | number | for Kerr | `a = J / M`, in the bundle's time unit. Never negative: the far-sky `Z` axis points along the angular momentum. |

### 5.5 `time_unit`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | What a person calls the unit: `"M"`, `"s"`. |
| `seconds` | number | yes | How many SI seconds one unit is. Positive and finite. |

### 5.6 `grid`

Four of the six fields have exactly one allowed value in version 1. They are written out so that a
person reading the manifest can see the layout without this document. A reader **must** refuse a
manifest in which any of them has another value.

| Field | Type | Required | Meaning |
|---|---|---|---|
| `projection` | string | yes | `"equirectangular"` (section 4.3). |
| `width` | integer | yes | `W`, the number of columns. At least 1. |
| `height` | integer | yes | `H`, the number of rows. At least 1. |
| `pole` | string | yes | `"observer-z"`: the top of the frame is the triad's `+z`. |
| `heading_zero` | string | yes | `"observer-x"`: the centre of the frame is the triad's `+x`. |
| `pixel_centres` | string | yes | `"half-integer"`: pixel `(i, j)` is sampled at `(i + 0.5, j + 0.5)` of the frame, as in the formulae of section 4.3. |

### 5.7 `playback`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `frames_per_second` | number | yes | The frame rate of the video the renderer makes. Positive. |
| `proper_time_per_video_second` | number | yes | How much of the observer's proper time, in the bundle's time unit, one second of video shows. Positive. |

At video time `s` seconds, the renderer shows the observer's sky at stopwatch time
`s * proper_time_per_video_second`. The frames of a bundle need not be evenly spaced in proper
time, and need not coincide with video frames; how the renderer chooses or blends frames is the
renderer's business.

### 5.8 `frames`

Each element records one written frame. The elements are in increasing order of `index`, and
`proper_time` increases strictly with `index`. The list may have gaps while a run is being written
or after it has been resumed.

| Field | Type | Required | Meaning |
|---|---|---|---|
| `index` | integer | yes | The frame's index. |
| `proper_time` | number | yes | The observer's proper time at this frame, in the bundle's time unit, from an origin the writer chooses. Finite. |
| `position` | object | no | The observer's event: `chart` (string) and `coords` (array of four numbers). |
| `file` | string | yes | The frame file's name relative to the bundle directory. Always the name section 2 gives for `index`; a reader **must** refuse any other. |
| `bytes` | integer | yes | The length of the frame file in bytes. |
| `readouts` | object | yes | The value of each read-out at this frame, keyed by read-out id (section 6). May be empty. |

`position.chart` names the coordinates:

- `"cartesian"`: `(t, x, y, z)`, with `x`, `y`, `z` along the far-sky axes.
- `"kerr-schild"`: ingoing Kerr-Schild `(t, r, theta, phi)`, where `r` and `theta` are the
  Boyer-Lindquist radius and polar angle.

The position is descriptive. A renderer **must not** need it.

### 5.9 `observer`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | no | The observer's name. |
| `triad` | string | no | A sentence stating how the writer built the triad (section 4.2). |

### 5.10 `far_sky`

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | A name for the orientation. `"galactic"` for the preset of section 4.4. |
| `axes_in_icrs` | object | yes | Fields `x`, `y`, `z`, each an array of three numbers: the far-sky axis as a unit vector in ICRS. |

### 5.11 Example

A complete manifest for a Kerr run of which two frames have been written. The first frame's
blueshift read-out was not computed, and is NaN.

```json
{
  "format": "black-hole-lab-sky",
  "version": 1,
  "writer": { "program": "sky-trace", "version": "0.1.0", "git": "43a2983" },
  "source": { "kind": "scenario", "name": "Bob falls from 6 M", "state_hash": "9f3c2a71d04be856" },
  "geometry": { "kind": "kerr", "mass": 1.0, "spin": 0.9 },
  "time_unit": { "name": "M", "seconds": 20.44078 },
  "observer": {
    "name": "Bob",
    "triad": "the radial, azimuthal and polar legs of the ZAMO frame, carried to Bob by the pure boost"
  },
  "grid": {
    "projection": "equirectangular",
    "width": 2048,
    "height": 1024,
    "pole": "observer-z",
    "heading_zero": "observer-x",
    "pixel_centres": "half-integer"
  },
  "far_sky": {
    "name": "galactic",
    "axes_in_icrs": {
      "x": [-0.0548755604162154, -0.8734370902348850, -0.4838350155487132],
      "y": [0.4941094278755837, -0.4448296299600112, 0.7469822444972189],
      "z": [-0.8676661490190047, -0.1980763734312015, 0.4559837761750669]
    }
  },
  "playback": { "frames_per_second": 30.0, "proper_time_per_video_second": 2.0 },
  "readouts": [
    { "id": "stopwatch", "label": "Stopwatch", "unit": "M", "decimals": 2 },
    { "id": "r", "label": "Radius", "unit": "M", "decimals": 3 },
    { "id": "blueshift", "label": "Peak blueshift", "unit": "", "decimals": 3 }
  ],
  "frames_planned": 900,
  "frames": [
    {
      "index": 0,
      "proper_time": 0.0,
      "position": { "chart": "kerr-schild", "coords": [0.0, 6.0, 1.5707963267948966, 0.0] },
      "file": "frames/000000.skyframe",
      "bytes": 21102871,
      "readouts": { "stopwatch": 0.0, "r": 6.0, "blueshift": "nan" }
    },
    {
      "index": 1,
      "proper_time": 0.06666666666666667,
      "position": { "chart": "kerr-schild", "coords": [0.0712, 5.9981, 1.5707963267948966, 0.0021] },
      "file": "frames/000001.skyframe",
      "bytes": 21099530,
      "readouts": { "stopwatch": 0.06666666666666667, "r": 5.9981, "blueshift": 1.412 }
    }
  ]
}
```

## 6. Read-outs

A *read-out* is a number the renderer draws on the video, such as the observer's stopwatch or
radius. The renderer draws read-outs without knowing what they mean: the manifest declares each
read-out once, and each frame's entry carries its values.

Each element of the manifest's `readouts` array declares one read-out:

| Field | Type | Required | Meaning |
|---|---|---|---|
| `id` | string | yes | The key under which each frame's `readouts` object gives the value. Unique within the bundle. |
| `label` | string | yes | What the renderer writes beside the value. |
| `unit` | string | yes | What the renderer writes after the value. Empty for a pure number. |
| `decimals` | integer | yes | How many decimal places the renderer should show. |

The renderer draws the read-outs in the order they are declared.

**The stopwatch comes first.** The first element of `readouts` **must** have the id `"stopwatch"`.
Its value at each frame is the observer's proper time since frame 0, in the bundle's time unit,
and its `unit` **should** be the time unit's name. When frame 0 has been written, a frame's
stopwatch value equals its `proper_time` less frame 0's `proper_time`.

**Values.** A frame's `readouts` object maps read-out ids to numbers. A frame **may** omit a
read-out, and the renderer then does not draw that read-out on that frame. A value **may** be
non-finite; a renderer draws NaN as "not available" and infinity as the infinity sign. Every key
**must** be the id of a declared read-out.

## 7. The frame file

A frame file holds the planes of one frame. Every multi-byte number in it is little-endian. `u8`,
`u32` and `u64` are unsigned integers of 1, 4 and 8 bytes; `i16` is a two's-complement signed
integer of 2 bytes; `f32` is an IEEE 754 binary32 float.

A frame file is a header, then a sequence of chunks, and nothing after the last chunk.

### 7.1 Header

| Offset | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 8 | bytes | signature | `89 53 4B 59 0D 0A 1A 0A` (hexadecimal): the byte 0x89, `SKY`, CR, LF, 0x1A, LF. |
| 8 | 4 | u32 | version | The format version: `1`. |
| 12 | 4 | u32 | header length | The header's length in bytes, `L`. 32 in version 1. |
| 16 | 4 | u32 | width | `W`. |
| 20 | 4 | u32 | height | `H`. |
| 24 | 4 | u32 | index | The frame's index. |
| 28 | 4 | u32 | chunk count | The number of chunks that follow. |

The signature follows the pattern of PNG's. A transfer that strips the high bit changes 0x89, and a
transfer that converts line endings changes the CR LF pair or the lone LF, so a damaged file is
refused as "not a frame" rather than misread.

The first chunk starts at offset `L`. A later version **may** lengthen the header by fields a
version-1 reader can ignore; a version-1 reader skips bytes 32 to `L - 1`.

### 7.2 Chunks

Each chunk is a 28-byte chunk header followed by the chunk's stored payload:

| Offset | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 4 | bytes | tag | Four ASCII characters naming the chunk (7.3). |
| 4 | 1 | u8 | codec | How the payload is stored (7.5). |
| 5 | 3 | bytes | reserved | Written as zero. A reader ignores them. |
| 8 | 4 | u32 | CRC | The CRC-32 (7.6) of the **uncompressed** payload. |
| 12 | 8 | u64 | uncompressed length | The payload's length before storing. |
| 20 | 8 | u64 | stored length | `S`, the number of payload bytes that follow. |
| 28 | `S` | bytes | payload | The stored payload. |

The next chunk starts immediately after the payload, with no padding. The file ends immediately
after the last chunk's payload.

Chunks may appear in any order. A writer **should** write them in the order of the table in 7.3.

### 7.3 Tags

| Tag | Required | Uncompressed length | Contents |
|---|---|---|---|
| `FATE` | yes | `W * H` | One u8 per pixel: the fate (section 4.7). |
| `DIRN` | yes | `12 * W * H` | Three planes of f32: every `d_X`, then every `d_Y`, then every `d_Z` (section 4.5). |
| `SHFT` | yes | `4 * W * H` | One plane of f32: the shift `g` (section 4.6). |
| `WIND` | yes | `2 * W * H` | One plane of i16: the winding (section 4.8). |
| `PNTS` | no | `8 + 24 * N` | Point sources (section 8). |
| `PTCH` | no | | Reserved for refinement patches, which a later version may define. A version-1 writer **must not** write it. |

Each plane is in the storage order of section 4.3. `DIRN` stores its components as three whole
planes, not as interleaved triples, because a plane of one quantity compresses better and a
renderer interpolates one component at a time.

NaN values are stored with their bit patterns unchanged, payload and sign included.

### 7.4 Unknown tags

A reader **must** skip a chunk whose tag it does not know, using the stored length, without
decompressing it or checking its CRC. This is what lets a later build add a chunk without a new
version (section 10). A tag is compared byte for byte; case matters.

A frame file **must not** hold two chunks with the same known tag.

### 7.5 Codecs

| Code | Codec | Stored payload |
|---|---|---|
| 0 | raw | The uncompressed payload itself. `S` equals the uncompressed length. |
| 1 | deflate | A raw DEFLATE stream (RFC 1951), with no zlib or gzip wrapper, that inflates to exactly the uncompressed length. |

A writer **may** choose either codec for each chunk. The library in `crates/sky-format` deflates each
chunk at level 1 and stores it raw when deflating does not make it shorter.

A reader **must** refuse a frame in which a chunk with a known tag uses a codec not in this table. A
new codec is a new format version.

### 7.6 The CRC

The CRC is the CRC-32 of zlib, gzip and PNG: polynomial 0x04C11DB7 in reflected form (0xEDB88320),
initial value 0xFFFFFFFF, input and output reflected, final value XORed with 0xFFFFFFFF. Its check
value, the CRC of the nine ASCII bytes `123456789`, is 0xCBF43926.

The CRC covers the uncompressed payload, so it checks the codec's output as well as the file.

### 7.7 Reading a frame

A reader of a frame from a bundle **must** check, in this order, and refuse the frame at the first
failure:

1. The first eight bytes are the signature. A file that is shorter than eight bytes and matches the
   signature as far as it goes is truncated; any other mismatch means the file is not a frame.
2. The version is no greater than the newest version the reader knows.
3. The header length is at least 32, and the file holds the whole header.
4. Width and height equal the manifest's `grid.width` and `grid.height`.
5. The index equals the index the reader asked for.
6. Each of the declared chunks is present in full. For each chunk with a known tag: the tag has not
   appeared before; the uncompressed length is the one 7.3 gives; the codec is known; the stored
   payload unpacks to exactly the uncompressed length; and the CRC matches.
7. No bytes follow the last chunk.
8. `FATE`, `DIRN`, `SHFT` and `WIND` were all present.

## 8. Point sources

The optional `PNTS` chunk lists point sources: images of individual stars that the renderer draws
as points rather than sampling from a star map. A frame without a `PNTS` chunk has no point
sources; a `PNTS` chunk with `N = 0` states that there are none. No writer produces point sources
yet; the chunk is defined now so that the first one to do so needs no new version.

The payload is an 8-byte head followed by `N` records of 24 bytes:

| Offset | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 4 | u32 | count | `N`. |
| 4 | 4 | u32 | reserved | Written as zero. A reader ignores it. |
| 8 + 24 k | 12 | 3 f32 | n | Where the observer looks to see image `k`: a unit vector in the triad, like a pixel's `n` (section 4.3). |
| 20 + 24 k | 4 | f32 | g | The shift of the image's light (section 4.6). |
| 24 + 24 k | 4 | f32 | magnification | `mu`, defined below. |
| 28 + 24 k | 4 | u32 | label | The id of a label declared in the manifest's `labels`, or 4294967295 (0xFFFFFFFF) for no label. |

The *magnification* `mu` of an image is the ratio of the solid angle the image covers on the
observer's sky to the solid angle of the patch of far sky it shows, negative when the image is
mirror-reversed. The flux the observer receives from the image is then

```text
F_observed = g^4 * |mu| * F_catalogue
```

in bolometric terms, where `F_catalogue` is the star's flux as an observer at rest in the far-sky
frame, far from the hole, would measure it.

The manifest's `labels` array names the labels. Each element has an `id` (integer, 0 to 4294967294,
unique) and a `text` (string).

## 9. Writing, crashing and resuming

### 9.1 Atomic writes

A writer **must** write every file of a bundle, frame files and the manifest alike, atomically:

1. Write the whole file under a temporary name in the same directory: the real name followed by
   `.tmp`, for example `frames/000042.skyframe.tmp`.
2. Flush the temporary file to the storage device.
3. Rename the temporary file to the real name, replacing any file of that name.

A rename within one directory is atomic, so a file under its real name is always a whole file. A
crash leaves at worst a stray `.tmp` file, which readers never open.

### 9.2 When a frame counts as written

A writer writes a frame's file first and then rewrites the manifest to list it. A frame counts as
written when the manifest lists it **and** its frame file exists with the length the manifest
records in `bytes`.

A frame file that the manifest does not list is an *orphan*: the run stopped after the frame's
rename and before the manifest's. Readers ignore orphans.

A writer **may** rewrite the manifest after every frame, which makes a crash cost at most one frame,
or less often, which makes a crash cost the frames since the last rewrite. The manifest grows with
the run, so rewriting it after every frame costs time in proportion to the frames already written.

### 9.3 Resuming

A writer that resumes a bundle:

1. Reads the manifest.
2. Drops from the manifest every frame whose file is missing or does not have the recorded length.
3. Deletes every `.tmp` file in the bundle directory and in `frames/`.
4. Treats the frames still listed as done, and writes every other frame, orphans included, again.

A writer **must not** resume a bundle of another version than its own, since that would make a
bundle of two versions.

### 9.4 Reading while a bundle is written

A reader may open a bundle while a writer is still writing it. The frames the manifest lists, with
files of the recorded length, are complete and can be read. The format defines no lock: at most one
writer may write a bundle at a time.

## 10. Versions and compatibility

The format version is a single integer, in the manifest's `version` field and in every frame
file's header. This document defines version 1.

- A build that knows version `N` **must** read every bundle of version `N` or lower.
- A reader **must** refuse a manifest or a frame file whose version is higher than any it knows,
  with a message saying that the bundle is from a newer version. It checks `format` and `version`
  before any other field, because a newer version may have changed the rest.
- Adding an optional manifest field, or an optional chunk tag, is **not** a new version. Readers
  ignore unknown JSON fields at every depth and skip unknown chunk tags, so an older reader reads a
  newer bundle of the same version, minus what it does not know.
- A change of structure or meaning **is** a new version: a field whose unit or meaning changes, a
  required chunk added or removed, a new codec, a new fate code with meaning a renderer must act on,
  or a change to the grid formulae.

## 11. Refusals

A reader **must** refuse, without crashing and with a message a person can act on, each of the
following. The names in the second column are the `sky_format::Error` variants that the library
uses.

| Condition | Library error |
|---|---|
| The path does not exist, is not a directory, has no `manifest.json`, or its manifest's `format` is not `"black-hole-lab-sky"`. | `NotABundle` |
| `manifest.json` is not valid JSON. | `ManifestUnreadable` |
| The manifest's version is higher than the reader knows. | `NewerManifest` |
| The manifest breaks a rule of section 5 or 6. | `InvalidManifest` |
| A frame file does not start with the signature. | `NotAFrame` |
| A frame file's version is higher than the reader knows. | `NewerFrame` |
| A frame file ends before its header or a chunk it declares is complete. | `Truncated` |
| A chunk's CRC does not match its payload. | `CrcMismatch` |
| A frame file's width and height differ from the manifest's grid. | `DimensionMismatch` |
| A frame file's index differs from the index its name gives. | `IndexMismatch` |
| Any other failure of section 7.7: a wrong chunk length, an unknown codec on a known tag, a DEFLATE stream that will not inflate, a duplicate or missing required chunk, bytes after the last chunk. | `Corrupt` |
| A frame is asked for that the manifest does not list. | `FrameNotWritten` |
