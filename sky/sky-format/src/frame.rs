//! One frame file: the planes of rays for one moment of the observer's watch.
//!
//! The layout, byte by byte, is in the specification; this is its implementation. In outline: a
//! 32-byte header, then a run of chunks, each a 28-byte chunk header and a payload. Every number
//! is little-endian. The planes are stored whole and planar (all the x components, then all the
//! y, then all the z) because a plane of one quantity compresses far better than interleaved
//! records, and a renderer that interpolates wants one component at a time anyway.

use std::io::{Read, Write};
use std::path::Path;

use flate2::Compression;
use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;

use crate::{Error, Grid, VERSION};

/// The first eight bytes of every frame file. The pattern is PNG's: a byte with the high bit set,
/// which a transfer that strips the eighth bit changes; a CR LF pair and a lone LF, which a
/// transfer that converts line endings changes; and a DOS end-of-file byte, which stops `type`
/// from printing binary to a console.
pub const MAGIC: [u8; 8] = [0x89, b'S', b'K', b'Y', b'\r', b'\n', 0x1A, b'\n'];

/// Length of the header this version writes. A reader skips any header bytes beyond it, so a
/// later version can lengthen the header without a version bump for fields old readers may ignore.
pub const HEADER_LEN: u32 = 32;

/// Length of every chunk header.
pub const CHUNK_HEADER_LEN: usize = 28;

/// The chunk tags version 1 defines, as they appear in the file.
pub mod tag {
    /// One u8 per ray: its fate.
    pub const FATE: [u8; 4] = *b"FATE";
    /// Three planes of f32: the x, y and z components of the direction at infinity.
    pub const DIRN: [u8; 4] = *b"DIRN";
    /// One f32 per ray: the shift g.
    pub const SHFT: [u8; 4] = *b"SHFT";
    /// One i16 per ray: the winding.
    pub const WIND: [u8; 4] = *b"WIND";
    /// Optional: point sources.
    pub const PNTS: [u8; 4] = *b"PNTS";
    /// Reserved for refinement patches. Version 1 writes no such chunk and skips one it meets.
    pub const PTCH: [u8; 4] = *b"PTCH";
}

/// What became of a ray traced backward from the observer.
pub mod fate {
    /// The integrator gave up before the ray reached anything.
    pub const UNRESOLVED: u8 = 0;
    /// The ray reached the distant sky: its direction and shift are meaningful.
    pub const FAR_SKY: u8 = 1;
    /// The ray did not come from the far sky: traced backward it closes on r = r+ and never leaves
    /// it. Seen from outside the hole that is the past horizon and the shadow; seen from between
    /// the horizons it may be the other branch of r+. Drawn black either way.
    pub const DARK: u8 = 2;
}

/// How a chunk's payload is stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    /// The payload is the bytes themselves.
    Raw = 0,
    /// The payload is a raw DEFLATE stream (RFC 1951, no zlib or gzip wrapper).
    Deflate = 1,
}

/// One point source's image: a star drawn as a point rather than sampled from a map.
#[derive(Debug, Clone, Copy)]
pub struct PointSource {
    /// Where the observer looks to see the image, as a unit vector in the observer's triad: the
    /// same kind of vector as a pixel's direction on the grid.
    pub n: [f32; 3],
    /// The shift g of the light that forms the image.
    pub shift: f32,
    /// The ratio of the image's solid angle to the solid angle of the patch of far sky it shows,
    /// negative for a mirror-reversed image.
    pub magnification: f32,
    /// A label id declared in the manifest, or `NO_LABEL`.
    pub label: u32,
}

impl PointSource {
    const LEN: usize = 24;
}

/// The planes of one frame, in memory.
///
/// Every plane holds `width * height` values in the grid's order: rows top to bottom, each row
/// left to right (see `Grid::offset`). NaN values keep their payload through a file, bit for bit,
/// so a writer may use the payload to say why a ray has no direction; version 1 gives the payload
/// no meaning.
#[derive(Debug, Clone)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub index: u32,
    pub fate: Vec<u8>,
    /// The direction at infinity d, a unit vector in the far-sky frame, as three planes: x, y, z.
    /// NaN where the fate is not `FAR_SKY`.
    pub direction: [Vec<f32>; 3],
    /// The shift g. NaN where the fate is not `FAR_SKY`.
    pub shift: Vec<f32>,
    pub winding: Vec<i16>,
    /// `None` writes no point-source chunk; `Some` of an empty list writes an empty one.
    pub points: Option<Vec<PointSource>>,
}

impl Frame {
    /// A frame of unresolved rays: fate 0, direction and shift NaN, winding 0, no point sources.
    pub fn new(width: u32, height: u32, index: u32) -> Self {
        let n = Grid::new(width, height).len();
        Self {
            width,
            height,
            index,
            fate: vec![fate::UNRESOLVED; n],
            direction: std::array::from_fn(|_| vec![f32::NAN; n]),
            shift: vec![f32::NAN; n],
            winding: vec![0; n],
            points: None,
        }
    }

    pub fn grid(&self) -> Grid {
        Grid::new(self.width, self.height)
    }

    /// The direction at infinity of the ray at pixel (i, j).
    pub fn direction_at(&self, i: u32, j: u32) -> [f32; 3] {
        let k = self.grid().offset(i, j);
        self.direction.each_ref().map(|plane| plane[k])
    }

    /// Checks that every plane has one value per ray. The fields are public, so a caller can
    /// build a frame whose planes disagree; this is what stops it reaching a file.
    pub fn check(&self) -> Result<(), Error> {
        let n = self.grid().len();
        if n == 0 {
            return Err(Error::InvalidFrame {
                why: format!("its grid is {} x {}", self.width, self.height),
            });
        }
        let planes = [
            ("fate", self.fate.len()),
            ("direction x", self.direction[0].len()),
            ("direction y", self.direction[1].len()),
            ("direction z", self.direction[2].len()),
            ("shift", self.shift.len()),
            ("winding", self.winding.len()),
        ];
        for (name, len) in planes {
            if len != n {
                return Err(Error::InvalidFrame {
                    why: format!(
                        "its {name} plane holds {len} values and its {} x {} grid has {n} rays",
                        self.width, self.height
                    ),
                });
            }
        }
        Ok(())
    }

    /// The frame as the bytes of a frame file, each chunk stored deflated or raw, whichever is
    /// shorter.
    pub fn encode(&self) -> Result<Vec<u8>, Error> {
        self.encode_with(None)
    }

    /// [`Frame::encode`] with every chunk forced to one codec, for tests that need to know which.
    pub(crate) fn encode_with(&self, codec: Option<Codec>) -> Result<Vec<u8>, Error> {
        self.check()?;
        let chunks: Vec<Chunk> = self
            .payloads()
            .into_iter()
            .map(|(tag, raw)| Chunk::pack(tag, &raw, codec))
            .collect();
        Ok(assemble(self.width, self.height, self.index, &chunks))
    }

    /// Each chunk's tag and uncompressed payload, in the order a writer writes them.
    pub(crate) fn payloads(&self) -> Vec<([u8; 4], Vec<u8>)> {
        let mut out = vec![
            (tag::FATE, self.fate.clone()),
            (
                tag::DIRN,
                self.direction
                    .iter()
                    .flatten()
                    .flat_map(|v| v.to_le_bytes())
                    .collect(),
            ),
            (
                tag::SHFT,
                self.shift.iter().flat_map(|v| v.to_le_bytes()).collect(),
            ),
            (
                tag::WIND,
                self.winding.iter().flat_map(|v| v.to_le_bytes()).collect(),
            ),
        ];
        if let Some(points) = &self.points {
            let mut raw = Vec::with_capacity(8 + PointSource::LEN * points.len());
            let count = u32::try_from(points.len()).expect("fewer than 2^32 point sources");
            raw.extend(count.to_le_bytes());
            raw.extend(0u32.to_le_bytes());
            for p in points {
                for v in [p.n[0], p.n[1], p.n[2], p.shift, p.magnification] {
                    raw.extend(v.to_le_bytes());
                }
                raw.extend(p.label.to_le_bytes());
            }
            out.push((tag::PNTS, raw));
        }
        out
    }

    /// Reads a frame file's bytes. `file` names the file in any refusal.
    pub fn decode(bytes: &[u8], file: &Path) -> Result<Self, Error> {
        decode(bytes, file, None)
    }
}

/// The CRC-32 the format uses: the one of zlib, gzip and PNG (reflected polynomial 0xEDB88320,
/// initial value and final XOR 0xFFFFFFFF).
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = flate2::Crc::new();
    crc.update(bytes);
    crc.sum()
}

/// One chunk ready to be written.
pub(crate) struct Chunk {
    pub tag: [u8; 4],
    pub codec: Codec,
    pub crc: u32,
    pub raw_len: u64,
    pub stored: Vec<u8>,
}

impl Chunk {
    /// Packs a payload with the codec asked for, or, given `None`, with DEFLATE unless storing it
    /// raw is no longer. Planes of f32 directions are mostly mantissa noise and a small grid can
    /// come out longer deflated than raw; storing those raw costs a reader nothing.
    pub fn pack(tag: [u8; 4], raw: &[u8], codec: Option<Codec>) -> Self {
        let deflated = || {
            // Level 1: the directions barely compress at any level, and what does compress - the
            // runs of NaN in a shadow, the fate and winding planes - compresses at the fastest.
            let mut encoder = DeflateEncoder::new(Vec::new(), Compression::fast());
            encoder
                .write_all(raw)
                .expect("writing to a Vec cannot fail");
            encoder.finish().expect("writing to a Vec cannot fail")
        };
        let (codec, stored) = match codec {
            Some(Codec::Raw) => (Codec::Raw, raw.to_vec()),
            Some(Codec::Deflate) => (Codec::Deflate, deflated()),
            None => {
                let d = deflated();
                if d.len() < raw.len() {
                    (Codec::Deflate, d)
                } else {
                    (Codec::Raw, raw.to_vec())
                }
            }
        };
        Self {
            tag,
            codec,
            crc: crc32(raw),
            raw_len: raw.len() as u64,
            stored,
        }
    }
}

/// Header and chunks as the bytes of a file. The chunk list is taken as given, so a test can hand
/// it a chunk this version does not define.
pub(crate) fn assemble(width: u32, height: u32, index: u32, chunks: &[Chunk]) -> Vec<u8> {
    let body: usize = chunks
        .iter()
        .map(|c| CHUNK_HEADER_LEN + c.stored.len())
        .sum();
    let mut out = Vec::with_capacity(HEADER_LEN as usize + body);
    out.extend(MAGIC);
    let count = u32::try_from(chunks.len()).expect("fewer than 2^32 chunks");
    for word in [VERSION, HEADER_LEN, width, height, index, count] {
        out.extend(word.to_le_bytes());
    }
    for c in chunks {
        out.extend(c.tag);
        out.push(c.codec as u8);
        out.extend([0u8; 3]);
        out.extend(c.crc.to_le_bytes());
        out.extend(c.raw_len.to_le_bytes());
        out.extend((c.stored.len() as u64).to_le_bytes());
        out.extend(&c.stored);
    }
    out
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().expect("four bytes"))
}

fn u64_at(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().expect("eight bytes"))
}

/// Reads a frame, refusing it if its grid or index is not the one `expect` names. The reader
/// passes the manifest's grid and the index it asked for; checking them straight after the
/// header means a frame from another run is refused as such, before its chunks are unpacked.
pub(crate) fn decode(
    bytes: &[u8],
    file: &Path,
    expect: Option<(Grid, u32)>,
) -> Result<Frame, Error> {
    let path = || file.to_path_buf();
    let len = bytes.len();
    let truncated = |needed: u64| Error::Truncated {
        path: path(),
        needed,
        found: len as u64,
    };
    let corrupt = |why: String| Error::Corrupt { path: path(), why };

    // A file too short to hold the signature is truncated if what is there is the start of one,
    // and is not a frame at all otherwise.
    let signature = &bytes[..len.min(MAGIC.len())];
    if signature != &MAGIC[..signature.len()] {
        return Err(Error::NotAFrame { path: path() });
    }
    if len < 12 {
        return Err(truncated(u64::from(HEADER_LEN)));
    }
    // The version before anything else of the header: a newer version may have moved the rest.
    let version = u32_at(bytes, 8);
    if version > VERSION {
        return Err(Error::NewerFrame {
            path: path(),
            version,
        });
    }
    if version == 0 {
        return Err(corrupt(
            "its header says version 0, and the first version is 1".into(),
        ));
    }
    if len < HEADER_LEN as usize {
        return Err(truncated(u64::from(HEADER_LEN)));
    }
    let header_len = u32_at(bytes, 12);
    if header_len < HEADER_LEN {
        return Err(corrupt(format!(
            "its header says it is {header_len} bytes long, and a header is at least {HEADER_LEN}"
        )));
    }
    if (len as u64) < u64::from(header_len) {
        return Err(truncated(u64::from(header_len)));
    }
    let grid = Grid::new(u32_at(bytes, 16), u32_at(bytes, 20));
    let index = u32_at(bytes, 24);
    let chunk_count = u32_at(bytes, 28);
    if let Some((wanted, wanted_index)) = expect {
        if grid != wanted {
            return Err(Error::DimensionMismatch {
                path: path(),
                found: (grid.width, grid.height),
                expected: (wanted.width, wanted.height),
            });
        }
        if index != wanted_index {
            return Err(Error::IndexMismatch {
                path: path(),
                found: index,
                expected: wanted_index,
            });
        }
    }
    if grid.is_empty() {
        return Err(corrupt(format!(
            "its grid is {} x {}",
            grid.width, grid.height
        )));
    }

    let n = grid.len();
    let mut frame = Frame {
        width: grid.width,
        height: grid.height,
        index,
        fate: Vec::new(),
        direction: [Vec::new(), Vec::new(), Vec::new()],
        shift: Vec::new(),
        winding: Vec::new(),
        points: None,
    };
    let mut seen: Vec<[u8; 4]> = Vec::new();
    let mut at = header_len as usize;
    for _ in 0..chunk_count {
        if len - at < CHUNK_HEADER_LEN {
            return Err(truncated((at + CHUNK_HEADER_LEN) as u64));
        }
        let tag: [u8; 4] = bytes[at..at + 4].try_into().expect("four bytes");
        let codec = bytes[at + 4];
        let crc = u32_at(bytes, at + 8);
        let raw_len = u64_at(bytes, at + 12);
        let stored_len = u64_at(bytes, at + 20);
        at += CHUNK_HEADER_LEN;
        if stored_len > (len - at) as u64 {
            return Err(truncated(at as u64 + stored_len));
        }
        let stored = &bytes[at..at + stored_len as usize];
        at += stored_len as usize;

        let name = String::from_utf8_lossy(&tag).into_owned();
        // The payload length each known tag must have; an unknown tag is skipped unread, which
        // is the rule that lets a later build add a chunk without a new version.
        let wanted_len = match tag {
            tag::FATE => Some(n as u64),
            tag::DIRN => Some(12 * n as u64),
            tag::SHFT => Some(4 * n as u64),
            tag::WIND => Some(2 * n as u64),
            tag::PNTS => None,
            _ => continue,
        };
        if seen.contains(&tag) {
            return Err(corrupt(format!("it has two {name} chunks")));
        }
        seen.push(tag);
        if let Some(wanted_len) = wanted_len
            && raw_len != wanted_len
        {
            return Err(corrupt(format!(
                "its {name} chunk holds {raw_len} bytes and a {} x {} grid needs {wanted_len}",
                grid.width, grid.height
            )));
        }
        let raw = unpack(codec, stored, raw_len, &name).map_err(corrupt)?;
        let computed = crc32(&raw);
        if computed != crc {
            return Err(Error::CrcMismatch {
                path: path(),
                tag: name,
                stored: crc,
                computed,
            });
        }
        let f32s = |raw: &[u8]| -> Vec<f32> {
            raw.as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect()
        };
        match tag {
            tag::FATE => frame.fate = raw,
            tag::DIRN => {
                let (x, rest) = raw.split_at(4 * n);
                let (y, z) = rest.split_at(4 * n);
                frame.direction = [f32s(x), f32s(y), f32s(z)];
            }
            tag::SHFT => frame.shift = f32s(&raw),
            tag::WIND => {
                frame.winding = raw
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|b| i16::from_le_bytes(*b))
                    .collect();
            }
            tag::PNTS => frame.points = Some(point_sources(&raw).map_err(corrupt)?),
            _ => unreachable!("unknown tags were skipped above"),
        }
    }
    if at != len {
        return Err(corrupt(format!("{} bytes follow its last chunk", len - at)));
    }
    for required in [tag::FATE, tag::DIRN, tag::SHFT, tag::WIND] {
        if !seen.contains(&required) {
            return Err(corrupt(format!(
                "it has no {} chunk, and every frame carries one",
                String::from_utf8_lossy(&required)
            )));
        }
    }
    Ok(frame)
}

/// A chunk's payload as the bytes it stands for, or why it cannot be had.
fn unpack(codec: u8, stored: &[u8], raw_len: u64, name: &str) -> Result<Vec<u8>, String> {
    let raw = match codec {
        0 => stored.to_vec(),
        1 => {
            // The declared length bounds the read, so a deflate stream that expands without limit
            // stops one byte past it and is refused below rather than filling memory.
            let mut out = Vec::with_capacity(raw_len.min(1 << 26) as usize);
            DeflateDecoder::new(stored)
                .take(raw_len.saturating_add(1))
                .read_to_end(&mut out)
                .map_err(|e| format!("its {name} chunk will not decompress ({e})"))?;
            out
        }
        other => {
            return Err(format!(
                "its {name} chunk is stored with codec {other}, which version 1 does not define"
            ));
        }
    };
    if raw.len() as u64 != raw_len {
        return Err(format!(
            "its {name} chunk unpacks to {} bytes and its header says {raw_len}",
            raw.len()
        ));
    }
    Ok(raw)
}

/// The records of a point-source chunk.
fn point_sources(raw: &[u8]) -> Result<Vec<PointSource>, String> {
    if raw.len() < 8 {
        return Err(format!(
            "its PNTS chunk holds {} bytes, fewer than the 8 of its count",
            raw.len()
        ));
    }
    let count = u32_at(raw, 0) as usize;
    let wanted = 8 + count as u64 * PointSource::LEN as u64;
    if raw.len() as u64 != wanted {
        return Err(format!(
            "its PNTS chunk holds {} bytes and {count} point sources need {wanted}",
            raw.len()
        ));
    }
    Ok(raw[8..]
        .as_chunks::<{ PointSource::LEN }>()
        .0
        .iter()
        .map(|r| {
            let f = |k: usize| f32::from_le_bytes(r[4 * k..4 * k + 4].try_into().expect("four"));
            PointSource {
                n: [f(0), f(1), f(2)],
                shift: f(3),
                magnification: f(4),
                label: u32_at(r, 20),
            }
        })
        .collect())
}
