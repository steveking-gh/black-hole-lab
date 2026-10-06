# Star maps for the sky renderer

This directory is where a source checkout keeps the star background that `sky-render` draws
behind everything else. The maps themselves are large, so they are downloaded on demand and are
never committed: `.gitignore` excludes `*.exr` and `*.part` here, and this description is the
only file tracked.

## Where the maps come from

The maps are NASA's **Deep Star Maps 2020**, made by Ernie Wright at the NASA Goddard Space
Flight Center Scientific Visualization Studio (SVS), entry 4851:
<https://svs.gsfc.nasa.gov/4851>. They were released on 9 September 2020; the galactic-coordinate
versions were replaced on 4 January 2021 to correct a coordinate transformation, and the
`milkyway` files are dated 11 August 2021. The checksums here are of the files as served on
2026-09-27.

NASA built the maps by plotting the position, brightness and colour of about 1.7 billion stars
from three catalogues:

- **Hipparcos-2** for stars brighter than visual magnitude 8.0;
- **Tycho-2** for stars between magnitude 8.0 and 11.5;
- **Gaia DR2** for fainter stars, down to about Gaia G magnitude 21. Two holes in the Gaia DR2
  coverage, centred at (right ascension 97.9 deg, declination +57.5 deg) and (34.2 deg,
  +22.1 deg), were filled from the UCAC3 catalogue.

NASA also added 18 stars of magnitude 4.3 to 6.8 that are in the Yale Bright Star Catalogue but
missing from Hipparcos-2, among them Eta Carinae. The SVS page lists them.

A star of magnitude 21 is not visible as a separate dot at any size offered here. The light of
the faint Gaia stars adds up to the diffuse glow of the Milky Way. The magnitude of the
faintest star that shows as its own dot at a given map size has not been measured.

## Products and sizes

A **product** is one kind of map. The renderer reads two products:

- `starmap`: the full map, the Milky Way from Gaia DR2 plus the bright stars from Hipparcos-2
  and Tycho-2.
- `milkyway`: the same sky without the Hipparcos-2 and Tycho-2 stars. It is meant as a
  background layer beneath bright stars drawn separately. Its values are on a different scale
  from `starmap` (see "Colour and values" below).

Each product exists in two coordinate systems, **celestial** and **galactic**, and in five sizes.
The default is `starmap`, `galactic`, `8k`, because the owner chose to put the Milky Way along
the video frame's equator.

| Size | Pixels | Degrees per pixel | File size |
| --- | --- | ---: | --- |
| 4k | 4096 x 2048 | 0.0879 | 33 to 39 MB |
| 8k | 8192 x 4096 | 0.0439 | 119 to 153 MB |
| 16k | 16384 x 8192 | 0.0220 | 357 to 423 MB |
| 32k | 32768 x 16384 | 0.0110 | 1.2 to 1.4 GB |
| 64k | 65536 x 32768 | 0.0055 | 3.0 to 3.8 GB |

The SVS page also offers a `hiptyc` product (the bright stars alone), constellation figures,
constellation boundaries and a coordinate grid. The renderer uses none of them.

## How to fetch a map

The default map, `starmap_2020_8k_gal.exr`, is downloaded by the OK of Black Hole Lab's Look
Around dialog, or from a terminal:

```sh
sky-look --fetch-map
```

The download goes to a file ending in `.part` in the user's data folder, and is renamed only when
its byte count and its SHA-256 are the ones in the table below. On any failure the partial file is
deleted and one sentence says what went wrong.

Any other map is a download by hand. Every file is served at
`https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/<file name>`, for example:

```sh
curl --fail --location --remote-name \
    https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/milkyway_2020_4k.exr
```

Check the file against the table with `sha256sum`, or `Get-FileHash` on Windows, and name it to
the renderer with `--sky <map.exr>` or the environment variable `BLACK_HOLE_LAB_SKY_MAP`.

A checksum mismatch after a complete download most likely means that NASA replaced the file, as
it did for the galactic maps in January 2021. Look at the SVS page before changing the record.

### Files on record

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `starmap_2020_4k.exr` | 35,997,085 | `69b841cd048c2ef35543eb1c5819b322530e78aa4b0cab97ec1de3ef56a4ddcd` |
| `starmap_2020_8k.exr` | 130,530,278 | `dc6c4f413e85707a29a25a9451148154554ecca2c996f84fa8f47b65ef9ff7c4` |
| `starmap_2020_4k_gal.exr` | 40,550,793 | `bce1d2c364720fd13ebbfaf1d444404a96b7cb4d7d8ddc88d44447358552361a` |
| `starmap_2020_8k_gal.exr` | 160,735,772 | `d70924422d3e0159764b16a19658784befcf73b97e06e5c16614860c1514bb58` |
| `milkyway_2020_4k.exr` | 36,436,668 | `2eb802d6e68d170b410f766c7fec07f7518619f6b6708fdc81e9302d93e74fdb` |
| `milkyway_2020_8k.exr` | 137,307,727 | `361d1961647af073b3b3e4aea4fca85f15b3c9d915e0c8c4befb02520dadd10a` |
| `milkyway_2020_4k_gal.exr` | 34,771,211 | `fad63b36aa632c691d521cd9d8b828de9d29b4c2784276eec44e54c6cb159a49` |
| `milkyway_2020_8k_gal.exr` | 125,076,308 | `d319113f35e45f7f66f2844f088639cc07cda74a5049299a6098d41e2ade3d28` |
| `starmap_2020_16k.exr` | 443,485,587 | `19a1351f00c386a6e5eec4d67af96d5fc71edf6a1189941579b9498b52e7589a` |
| `starmap_2020_16k_gal.exr` | 384,152,397 | `20363595f309da9626111937d6d75555530937914527abbd6d81132764434aa7` |
| `milkyway_2020_16k.exr` | 434,041,232 | `9d60e58075bcbce9cd03f8115e5635533a0a910007348a485755630a3927b97a` |
| `milkyway_2020_16k_gal.exr` | 373,794,299 | `ee023f03e99466b3829117c15a001269ba32a3fecb6012d0b2d41f201c955351` |

No checksum is on record for the 32k and 64k files; they have never been downloaded here.

## Geometry

Every map is an **equirectangular** projection, which NASA calls plate carree: longitude maps
linearly to the column and latitude maps linearly to the row. "Longitude" means right ascension
in a celestial map and galactic longitude l in a galactic map; "latitude" means declination or
galactic latitude b.

| | Celestial map | Galactic map |
| --- | --- | --- |
| Frame | ICRS (NASA writes "ICRF/J2000"), geocentric | Galactic, from ICRS by the Hipparcos matrix (see below) |
| Centre of the image | Right ascension 0h, declination 0 | l = 0 (the galactic centre direction), b = 0 |
| Longitude increases | to the LEFT | to the LEFT |
| Top edge | North celestial pole (+90 deg) | North galactic pole (b = +90 deg) |
| Left and right edges | Right ascension 12h (180 deg) | l = 180 deg (the anticentre) |

Longitude increases to the left because the map shows the sky as seen from inside the sphere,
looking out, with north up: east is then on the left, as on any sky chart.

### Pixel-centre convention

Take a map W pixels wide and H pixels high, with column i counted from 0 at the left and row j
counted from 0 at the top. The centre of pixel (i, j) lies at

    longitude = 180 deg - (i + 0.5) * 360 deg / W
    latitude  =  90 deg - (j + 0.5) * 180 deg / H

So the left edge of the image (x = 0) is longitude +180 deg, the right edge is -180 deg (the same
meridian), the top edge is latitude +90 deg and the bottom edge -90 deg. Longitude 0 falls on the
boundary between columns W/2 - 1 and W/2, not on a pixel centre, and the equator falls on the
boundary between rows H/2 - 1 and H/2. Each pole is a whole row edge, not a point.

For the renderer this is the same grid as the observer-sky grid in the sky-movie plan: with
`lambda_i = ((i + 0.5) / W - 0.5) * 2 pi` and `phi = -lambda_i`, the map's longitude at column i
equals phi. A unit direction (x, y, z) in the map's own frame (x toward longitude 0 on the
equator, z toward the north pole, y toward longitude +90 deg) samples the map at the continuous
pixel coordinates

    u = W * (0.5 - atan2(y, x) / (2 pi))        column coordinate, 0 at the left edge
    v = H * (0.5 - asin(z) / pi)                row coordinate, 0 at the top edge

where the centre of pixel (i, j) is at (u, v) = (i + 0.5, j + 0.5). u wraps modulo W.

The galactic maps use the ICRS-to-galactic rotation from the Hipparcos and Gaia documentation
(NASA states this on the SVS page). As rows of the matrix that takes an ICRS unit vector to a
galactic one:

    [ -0.0548755604162154  -0.8734370902348850  -0.4838350155487132 ]
    [ +0.4941094278755837  -0.4448296299600112  +0.7469822444972189 ]
    [ -0.8676661490190047  -0.1980763734312015  +0.4559837761750669 ]

The star check below used this matrix, and the residuals confirm it.

### How the geometry was verified

The check, run on 2026-09-27, read each map into floating point with ffmpeg
(`-pix_fmt gbrpf32le -f rawvideo`), formed the luminance 0.2126 R + 0.7152 G + 0.0722 B, and
located 21 bright stars. For each star, the check took the connected region above half of the
star's peak value (measured from the local median background) and computed the region's
brightness-weighted centroid. It compared that centroid with the pixel position that the
convention above predicts from the star's catalogue coordinates (ICRS). For the galactic maps,
the coordinates were first rotated with the matrix above.

The catalogue coordinates and proper motions are the values SIMBAD gives, but they were entered
by hand and were not fetched from SIMBAD during the check. An error in one of them would show as
a residual for that star alone; the residuals below bound any such error at about 0.1 pixel. A
second check by the reviewer, with coordinates entered independently for six of the stars and a
cruder centroid, predicted the same pixel positions to 0.05 pixel and measured every star within
0.6 pixel of its prediction.

The first comparison, with positions at epoch J2000, left residuals of up to 35 arcseconds that
matched the stars' proper motions: Alpha Centauri, the fastest-moving star in the set, was worst
at every map size. Moving the catalogue positions back to epoch J1991.25, the epoch of the
Hipparcos catalogue, removed that pattern: the on-sky rms fell from 11.5 to 6.2 arcseconds on
the 16k galactic map. **The bright stars are drawn at their Hipparcos positions, epoch
J1991.25**, not propagated to 2000 or 2020. The largest shift this causes for any star here is
32 arcseconds (Alpha Centauri), which is 0.2 pixel at 8k and does not matter for the renderer.

Results, with the positions at epoch J1991.25. dx is measured minus predicted column coordinate,
dy measured minus predicted row coordinate. On the celestial maps, Polaris is left out of the
pixel statistics, because it lies 0.7 deg from the pole and its image is 80 pixels wide; the
on-sky columns include it.

| Map | Stars | Mean dx, dy (px) | Rms dx, dy (px) | Rms on the sky | Worst on the sky |
| --- | ---: | --- | --- | ---: | ---: |
| `starmap_2020_4k.exr` | 20 | -0.002, +0.023 | 0.071, 0.078 | 31" | 60" |
| `starmap_2020_8k.exr` | 20 | +0.024, -0.006 | 0.048, 0.049 | 11" | 25" |
| `starmap_2020_4k_gal.exr` | 21 | -0.009, +0.016 | 0.069, 0.076 | 30" | 70" |
| `starmap_2020_8k_gal.exr` | 21 | -0.022, -0.030 | 0.063, 0.060 | 13" | 21" |
| `starmap_2020_16k_gal.exr` | 21 | -0.001, -0.010 | 0.046, 0.067 | 6" | 15" |

A half-pixel error in the convention would show as a mean offset of 0.5 px in dx or dy, or both.
The measured means are below 0.03 px, so the convention above is right to within a few
hundredths of a pixel at every size checked.

The per-star results for the default map, `starmap_2020_8k_gal.exr` (W = 8192, H = 4096):

| Star | l (deg) | b (deg) | Predicted x, y (px) | Measured x, y (px) | dx, dy (px) | On the sky |
| --- | ---: | ---: | --- | --- | --- | ---: |
| Sirius | 227.228 | -8.888 | 7117.30, 2250.25 | 7117.21, 2250.19 | -0.09, -0.06 | 17" |
| Canopus | 261.212 | -25.292 | 6343.97, 2623.54 | 6343.88, 2623.48 | -0.09, -0.06 | 16" |
| Arcturus | 15.066 | +69.111 | 3753.17, 475.34 | 3753.17, 475.25 | -0.00, -0.09 | 14" |
| Vega | 67.447 | +19.237 | 2561.20, 1610.24 | 2561.16, 1610.20 | -0.04, -0.05 | 9" |
| Capella | 162.588 | +4.567 | 396.23, 1944.08 | 396.11, 1944.03 | -0.12, -0.04 | 21" |
| Rigel | 209.241 | -25.245 | 7526.60, 2622.47 | 7526.66, 2622.49 | +0.06, +0.02 | 10" |
| Procyon | 213.701 | +13.022 | 7425.12, 1751.68 | 7425.09, 1751.61 | -0.03, -0.06 | 11" |
| Betelgeuse | 199.787 | -8.959 | 7741.73, 2251.86 | 7741.64, 2251.77 | -0.09, -0.09 | 21" |
| Achernar | 290.842 | -58.792 | 5669.74, 3385.85 | 5669.83, 3385.78 | +0.09, -0.07 | 14" |
| Altair | 47.743 | -8.909 | 3009.59, 2250.72 | 3009.59, 2250.69 | -0.00, -0.03 | 4" |
| Aldebaran | 180.971 | -20.248 | 8169.89, 2508.76 | 8169.84, 2508.85 | -0.06, +0.09 | 17" |
| Antares | 351.947 | +15.064 | 4279.25, 1705.20 | 4279.23, 1705.14 | -0.02, -0.06 | 10" |
| Spica | 316.113 | +50.845 | 5094.68, 891.00 | 5094.58, 891.00 | -0.11, -0.00 | 11" |
| Pollux | 192.230 | +23.408 | 7913.71, 1515.34 | 7913.77, 1515.27 | +0.06, -0.07 | 15" |
| Fomalhaut | 20.489 | -64.909 | 3629.76, 3525.04 | 3629.78, 3525.01 | +0.02, -0.02 | 4" |
| Deneb | 84.285 | +1.998 | 2178.05, 2002.54 | 2178.05, 2002.60 | -0.00, +0.06 | 9" |
| Regulus | 226.428 | +48.935 | 7135.51, 934.46 | 7135.51, 934.49 | -0.00, +0.03 | 5" |
| Alpha Centauri | 315.742 | -0.684 | 5103.12, 2063.57 | 5103.11, 2063.60 | -0.01, +0.03 | 5" |
| Acrux | 300.127 | -0.363 | 5458.45, 2056.25 | 5458.41, 2056.22 | -0.04, -0.03 | 8" |
| Alcyone (Pleiades) | 166.668 | -23.455 | 303.37, 2581.74 | 303.33, 2581.73 | -0.04, -0.00 | 6" |
| Polaris | 123.280 | +26.461 | 1290.69, 1445.86 | 1290.75, 1445.74 | +0.07, -0.12 | 21" |

Three extended features were checked as well, on both 8k `starmap` maps:

- **The galactic centre.** Sagittarius A* (l = 359.944 deg, b = -0.046 deg) predicts column
  4097.3, row 2049.1 on the 8k galactic map, which is the middle of the image. A crop there shows
  the dark dust lane running horizontally through the middle of the bright galactic bulge, with
  the brightest part of the bulge just below it, as expected. On the 8k celestial map the same
  point is at column 6225.5, row 2708.1: about 3/4 of the width from the left and below the
  middle.
- **The Large Magellanic Cloud.** The check took the brightness-weighted centroid of the
  diffuse light within 5 deg of the catalogue centre (right ascension 80.894 deg, declination
  -69.756 deg), after replacing each 16 x 16 pixel block by its median to suppress single stars
  and subtracting the median of a ring 7 to 10 deg out. The centroid lies 0.44 deg from the
  catalogue centre on the celestial map and 0.46 deg on the galactic map. The two centroids,
  mapped back to ICRS, lie 0.03 deg apart, so the two maps place the LMC at the same point of the
  sky. The LMC is about 10 deg across and its light is lopsided, so an offset of 0.45 deg between
  the light centroid and the catalogue centre is within the size of the object.
- **The Pleiades.** The cluster appears at the predicted place on both maps; Alcyone, its
  brightest star, is in the table above.

## Colour and values

**Encoding.** Each file is OpenEXR, scanline, three channels named B, G and R, each a 16-bit
half float. Compression is ZIP (16 lines per block), except `starmap_2020_4k_gal.exr`, which
uses ZIPS (one line per block). The line order attribute is DECREASING_Y, which changes only
the order in which blocks are stored: row 0 is still the top of the image. ffmpeg decodes the
files correctly (ffprobe reports `gbrpf16le`, `color_transfer=linear`). The values are linear
light: there is no transfer curve to undo. The data window equals the display window, with no
offset.

**Primaries.** The files carry no `chromaticities` attribute. By the OpenEXR convention, an
image without one has the Rec. ITU-R BT.709 primaries (the same as sRGB) and a D65 white point.
That agrees with how NASA made the colours of the Hipparcos-2 and Tycho-2 stars: B-V colour
index to effective temperature, then temperature to RGB with Mitchell Charity's blackbody table,
which uses sRGB primaries and a D65 white point. The SVS page does not say whether NASA removed
the sRGB transfer curve from Charity's values before writing the linear EXR files. The colours
of the Gaia DR2 stars came from the G, G_BP and G_RP magnitudes scaled by colour-balance factors
that NASA "estimated by eye", and about a quarter of the Gaia stars have no G_BP or G_RP value
and are drawn white. So the primaries are Rec. 709 by convention, and the star colours are
plausible, not colorimetric.

**Range.** Every value lies between 0.0 and 1.0: the maps are clipped at 1.0 in each channel.
The brightest stars are flat-topped at 1.0 (Sirius, Canopus, Vega and most other stars of
magnitude 1 or brighter). On the 8k celestial map, 0.009 % of pixels (3,170) are at 1.0. A
renderer that brightens the sky, for example by g^4 for a blueshift, cannot recover the true
brightness of those stars, and a clipped star keeps its clipped colour.

**Absolute scale.** The values have no photometric calibration, and their scale depends on the
map size and the product:

- Doubling the size roughly quarters the values, as if each pixel held the light falling in
  it rather than the light per unit solid angle. A 4k pixel holds on average 0.90 of the sum of
  the four 8k pixels it covers (median per pixel 0.96). The solid-angle-weighted mean of the
  luminance is 0.0161 at 4k and 0.0046 at 8k (celestial `starmap`).
- The celestial and galactic renderings of the same product and size agree to about 5 %
  (solid-angle-weighted mean 0.0161 and 0.0153 at 4k).
- The `milkyway` product's diffuse background is brighter than the same background in `starmap`:
  the median ratio of 16 x 16 pixel block means is 1.65, and in the faintest half of the pixels
  the median ratio is 2.0. `starmap` is therefore not `milkyway` plus the bright stars on one
  scale; the two need separate exposure settings.

The renderer should therefore treat a map as a picture of relative brightness with its own
exposure setting, chosen per product and per size, and not as a measured radiance.

**Stars on the sphere.** The spots are round on the sky, not on the image: a star at latitude
beta is stretched horizontally by 1/cos(beta) in the image. Measured on the 8k celestial map,
the width-to-height ratio of the spots is 1.63 for Canopus (1/cos(beta) = 1.65), 1.91 for
Achernar (1.85) and about 89 for Polaris (78). The map is therefore a proper texture for the
sphere, and a renderer that samples it by direction sees round stars everywhere. Near the poles
a single output pixel covers many map columns, so the renderer must filter (average over the
footprint) there to avoid aliasing.

## What is in the sky and what is not

Visible, checked by cropping the 8k maps:

- the Milky Way, with dark dust lanes along it: the Coalsack beside the Southern Cross, the lane
  across the galactic centre, and the lanes from Aquila to Cygnus;
- the Large and Small Magellanic Clouds, as diffuse patches of stars;
- globular clusters, for example Omega Centauri and 47 Tucanae, as fuzzy spots;
- open clusters, for example the Pleiades and the Hyades;
- the bright stars as separate dots (in `starmap` only).

Not visible:

- **The Andromeda Galaxy (M31) is not in the map.** Its position (right ascension 0h 42.7m,
  declination +41.27 deg) shows only foreground stars, even with the image brightened 16 times.
  The Triangulum Galaxy (M33) is missing in the same way. The maps are drawn from star
  catalogues and do not show the light of other galaxies, apart from the two Magellanic Clouds,
  whose brighter stars the catalogues contain. The owner decided on 2026-09-27 to leave this for
  later, when galaxies may become a separate layer.
- **Nebulae as glowing gas.** The Orion Nebula (M42) shows as its stars only. Only starlight is
  drawn; emission and reflection nebulae are not.
- The Sun, the Moon, the planets, and anything else in the Solar System.

## Credit

Any video or image published with these maps must carry NASA's credit line, as the SVS page
gives it:

> NASA/Goddard Space Flight Center Scientific Visualization Studio. Gaia DR2: ESA/Gaia/DPAC.
> Constellation figures based on those developed for the IAU by Alan MacRobert of Sky and
> Telescope magazine (Roger Sinnott and Rick Fienberg).

The last sentence concerns the constellation figures, which the renderer does not use. Keeping
the credit line whole, as NASA gives it, is the safe choice. The SVS page links NASA's
reproduction guidelines (<https://www.nasa.gov/multimedia/guidelines/index.html>), which govern
the use of the maps; the owner should read them before publishing. The maps are not
distributed with Black Hole Lab: each user downloads them from NASA.
`THIRD-PARTY-NOTICES.md` records the same.
