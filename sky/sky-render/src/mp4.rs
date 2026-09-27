//! Marking an MP4 as a 360-degree video: Google's Spherical Video V1 metadata.
//!
//! A player such as VLC, or YouTube, lets the viewer look around a video only when the file says
//! it is spherical. The V1 way of saying so is a `uuid` box, with the UUID
//! ffcc8263-f855-4a93-8814-587a02521fdd, holding a short XML document, placed in the video track's
//! `trak` box (github.com/google/spatial-media, docs/spherical-video-rfc.md). ffmpeg reads it back
//! as "Spherical Mapping, equirectangular".
//!
//! An MP4 is a sequence of boxes, each a 32-bit big-endian size (of the whole box, header
//! included), a four-letter type, and the contents, which for container boxes are more boxes.
//! Adding a box to the end of the video `trak` makes the `trak` and the `moov` that holds it
//! longer by the box's size, and moves every byte after the insertion point. That is harmless only
//! if no media data comes after it: the track's chunk offsets (`stco`, `co64`) are absolute
//! positions in the file, and moving `mdat` would leave them pointing at the wrong bytes. ffmpeg
//! writes `moov` after `mdat` unless asked for `faststart`, which this program never asks for, so
//! the easy case is the normal one, and the others are refused rather than mishandled:
//!
//! - a `moov` before an `mdat` (rewriting every chunk offset is a job this code does not do);
//! - a `moov` or video `trak` with a 64-bit size (not written by ffmpeg for a file this size, and
//!   growing it would need the other size field);
//! - a file already carrying the box, or with no video track.

/// The UUID of the Spherical Video V1 box.
pub const SPHERICAL_UUID: [u8; 16] = [
    0xff, 0xcc, 0x82, 0x63, 0xf8, 0x55, 0x4a, 0x93, 0x88, 0x14, 0x58, 0x7a, 0x02, 0x52, 0x1f, 0xdd,
];

/// The XML the box holds: an equirectangular, stitched, spherical video.
pub fn spherical_xml() -> String {
    format!(
        "<?xml version=\"1.0\"?>\
         <rdf:SphericalVideo xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" \
         xmlns:GSpherical=\"http://ns.google.com/videos/1.0/spherical/\">\
         <GSpherical:Spherical>true</GSpherical:Spherical>\
         <GSpherical:Stitched>true</GSpherical:Stitched>\
         <GSpherical:StitchingSoftware>Black Hole Lab sky-render {}</GSpherical:StitchingSoftware>\
         <GSpherical:ProjectionType>equirectangular</GSpherical:ProjectionType>\
         </rdf:SphericalVideo>",
        env!("CARGO_PKG_VERSION")
    )
}

/// The `uuid` box itself.
pub fn spherical_box() -> Vec<u8> {
    let xml = spherical_xml();
    let size = u32::try_from(8 + 16 + xml.len()).expect("a short box");
    let mut out = Vec::with_capacity(size as usize);
    out.extend(size.to_be_bytes());
    out.extend(b"uuid");
    out.extend(SPHERICAL_UUID);
    out.extend(xml.as_bytes());
    out
}

/// One box found in a file.
#[derive(Debug, Clone, Copy)]
struct Mp4Box {
    start: usize,
    /// The whole box, header included.
    size: usize,
    /// 8, or 16 for a box with a 64-bit size.
    header: usize,
    kind: [u8; 4],
}

impl Mp4Box {
    fn end(&self) -> usize {
        self.start + self.size
    }

    fn contents(&self) -> (usize, usize) {
        (self.start + self.header, self.end())
    }
}

/// The boxes from `start` to `end` of `file`.
fn boxes(file: &[u8], start: usize, end: usize) -> Result<Vec<Mp4Box>, String> {
    let mut out = Vec::new();
    let mut at = start;
    while at < end {
        if end - at < 8 {
            return Err(format!(
                "it has {} stray bytes at offset {at}, too few for a box",
                end - at
            ));
        }
        let size32 = u32::from_be_bytes(file[at..at + 4].try_into().expect("four bytes"));
        let kind: [u8; 4] = file[at + 4..at + 8].try_into().expect("four bytes");
        let (size, header) = match size32 {
            0 => (end - at, 8),
            1 => {
                if end - at < 16 {
                    return Err(format!("the box at offset {at} is cut short"));
                }
                let size = u64::from_be_bytes(file[at + 8..at + 16].try_into().expect("eight"));
                (usize::try_from(size).unwrap_or(usize::MAX), 16)
            }
            n => (n as usize, 8),
        };
        if size < header || size > end - at {
            return Err(format!(
                "the {} box at offset {at} says it is {size} bytes long, which does not fit",
                String::from_utf8_lossy(&kind)
            ));
        }
        out.push(Mp4Box {
            start: at,
            size,
            header,
            kind,
        });
        at += size;
    }
    Ok(out)
}

/// The file with the Spherical Video V1 box appended to its video track, or a sentence saying
/// why it cannot be done safely.
pub fn tag_spherical(file: &[u8]) -> Result<Vec<u8>, String> {
    let top = boxes(file, 0, file.len())?;
    let moov = top
        .iter()
        .find(|b| &b.kind == b"moov")
        .ok_or("it has no moov box, so it is not a finished MP4")?;
    if !top.iter().any(|b| &b.kind == b"mdat") {
        return Err("it has no mdat box, so it holds no video".into());
    }
    if top
        .iter()
        .any(|b| &b.kind == b"mdat" && b.start > moov.start)
    {
        return Err(
            "its moov box comes before its media data (a \"faststart\" file), and \
                    growing it would move the media without rewriting the chunk offsets that \
                    point into it"
                .into(),
        );
    }
    if moov.header != 8 {
        return Err("its moov box has a 64-bit size, which this tagger does not grow".into());
    }
    let (from, to) = moov.contents();
    let mut video = None;
    for trak in boxes(file, from, to)?
        .into_iter()
        .filter(|b| &b.kind == b"trak")
    {
        let (from, to) = trak.contents();
        for mdia in boxes(file, from, to)?
            .into_iter()
            .filter(|b| &b.kind == b"mdia")
        {
            let (from, to) = mdia.contents();
            for hdlr in boxes(file, from, to)?
                .into_iter()
                .filter(|b| &b.kind == b"hdlr")
            {
                // A full box: version and flags (4 bytes), pre_defined (4), then the handler.
                let at = hdlr.start + hdlr.header + 8;
                if hdlr.end() >= at + 4 && &file[at..at + 4] == b"vide" && video.is_none() {
                    video = Some(trak);
                }
            }
        }
    }
    let trak = video.ok_or("it has no video track")?;
    if trak.header != 8 {
        return Err("its video trak box has a 64-bit size, which this tagger does not grow".into());
    }
    let (from, to) = trak.contents();
    for b in boxes(file, from, to)? {
        if &b.kind == b"uuid" && b.size >= 24 && file[b.start + 8..b.start + 24] == SPHERICAL_UUID {
            return Err("its video track already carries the spherical metadata".into());
        }
    }

    let tag = spherical_box();
    let grow = |size: usize| {
        u32::try_from(size + tag.len())
            .map_err(|_| "its moov box would pass 4 GiB with the metadata added".to_string())
    };
    let trak_size = grow(trak.size)?;
    let moov_size = grow(moov.size)?;
    let mut out = Vec::with_capacity(file.len() + tag.len());
    out.extend_from_slice(&file[..trak.end()]);
    out.extend_from_slice(&tag);
    out.extend_from_slice(&file[trak.end()..]);
    out[trak.start..trak.start + 4].copy_from_slice(&trak_size.to_be_bytes());
    out[moov.start..moov.start + 4].copy_from_slice(&moov_size.to_be_bytes());
    Ok(out)
}

/// Tags the MP4 at `from` and writes the result to `to` atomically: under a temporary name,
/// flushed, then renamed, so that `to` is only ever absent, its old contents, or the whole tagged
/// file.
pub fn tag_file(from: &std::path::Path, to: &std::path::Path) -> Result<(), String> {
    use std::io::Write;
    let say = |what: &str, path: &std::path::Path, e: std::io::Error| {
        format!("could not {what} {}: {e}", path.display())
    };
    let file = std::fs::read(from).map_err(|e| say("read", from, e))?;
    let tagged = tag_spherical(&file).map_err(|why| {
        format!(
            "{} cannot be tagged as a 360-degree video: {why}",
            from.display()
        )
    })?;
    let mut name = to.as_os_str().to_owned();
    name.push(".tmp");
    let temporary = std::path::PathBuf::from(name);
    let mut out = std::fs::File::create(&temporary).map_err(|e| say("create", &temporary, e))?;
    out.write_all(&tagged)
        .map_err(|e| say("write", &temporary, e))?;
    out.sync_all().map_err(|e| say("flush", &temporary, e))?;
    drop(out);
    std::fs::rename(&temporary, to).map_err(|e| say("rename", &temporary, e))
}
