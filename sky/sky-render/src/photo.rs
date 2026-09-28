//! A still as a 360-degree photograph: a JPEG carrying Google's Photo Sphere metadata.
//!
//! A photo viewer lets the viewer look around an equirectangular JPEG only when the file says it
//! is one. Google's way of saying so, which photo viewers, Google Photos and some video players
//! read, is a set of XMP properties in the `GPano` namespace
//! (developers.google.com/streetview/spherical-metadata): the projection, and the size of the
//! full panorama and of the part of it the image holds. A still is the whole sphere, so the two
//! sizes are the image's own and the crop starts at its top-left corner.
//!
//! XMP goes into a JPEG as an APP1 segment (the XMP specification, part 3, section 1.1.3): the
//! marker FF E1, a two-byte big-endian length that counts itself but not the marker, the
//! namespace `http://ns.adobe.com/xap/1.0/` and a zero byte, then the XMP packet. A segment holds
//! at most 65,533 bytes after the marker's length, far more than these properties need. The
//! segment goes straight after the start-of-image marker, or after the JFIF APP0 segment when
//! the file begins with one, since JFIF requires its APP0 to come first. ffmpeg 8's encoder
//! writes none (its file begins with a comment segment naming the encoder), so in practice the
//! XMP comes straight after the start of image.
//!
//! The image itself is made by ffmpeg (`crate::encode::jpeg`), which already converts the
//! frame's colours for the video; everything here is bytes.

/// The namespace that opens an XMP APP1 segment, with its terminating zero.
pub const XMP_NAMESPACE: &[u8] = b"http://ns.adobe.com/xap/1.0/\0";

/// The Photo Sphere XMP packet for a whole-sphere image `width` x `height`.
pub fn xmp(width: usize, height: usize) -> String {
    format!(
        "<?xpacket begin=\"\u{feff}\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>\n\
         <x:xmpmeta xmlns:x=\"adobe:ns:meta/\">\n\
         <rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n\
         <rdf:Description rdf:about=\"\" xmlns:GPano=\"http://ns.google.com/photos/1.0/panorama/\">\n\
         <GPano:UsePanoramaViewer>True</GPano:UsePanoramaViewer>\n\
         <GPano:ProjectionType>equirectangular</GPano:ProjectionType>\n\
         <GPano:FullPanoWidthPixels>{width}</GPano:FullPanoWidthPixels>\n\
         <GPano:FullPanoHeightPixels>{height}</GPano:FullPanoHeightPixels>\n\
         <GPano:CroppedAreaImageWidthPixels>{width}</GPano:CroppedAreaImageWidthPixels>\n\
         <GPano:CroppedAreaImageHeightPixels>{height}</GPano:CroppedAreaImageHeightPixels>\n\
         <GPano:CroppedAreaLeftPixels>0</GPano:CroppedAreaLeftPixels>\n\
         <GPano:CroppedAreaTopPixels>0</GPano:CroppedAreaTopPixels>\n\
         <GPano:StitchingSoftware>Black Hole Lab sky-render {}</GPano:StitchingSoftware>\n\
         </rdf:Description>\n\
         </rdf:RDF>\n\
         </x:xmpmeta>\n\
         <?xpacket end=\"r\"?>",
        env!("CARGO_PKG_VERSION")
    )
}

/// One marker segment of a JPEG's header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    /// The byte after FF.
    pub marker: u8,
    /// Where the segment's FF is.
    pub start: usize,
    /// One past its last byte.
    pub end: usize,
}

impl Segment {
    /// What follows the marker and the length.
    pub fn payload<'a>(&self, jpeg: &'a [u8]) -> &'a [u8] {
        &jpeg[self.start + 4..self.end]
    }
}

/// The segments of a JPEG from the start of image to the start of scan, which is where the
/// headers end and the compressed data begins. The start of image itself is not listed.
pub fn segments(jpeg: &[u8]) -> Result<Vec<Segment>, String> {
    if jpeg.len() < 4 || jpeg[..2] != [0xFF, 0xD8] {
        return Err("it does not begin with a JPEG start-of-image marker".into());
    }
    let mut out = Vec::new();
    let mut at = 2;
    loop {
        if at + 4 > jpeg.len() {
            return Err(format!("it ends at byte {at}, in its headers"));
        }
        if jpeg[at] != 0xFF {
            return Err(format!(
                "byte {at} should begin a marker, and is {:#04x}",
                jpeg[at]
            ));
        }
        let marker = jpeg[at + 1];
        let length = usize::from(u16::from_be_bytes([jpeg[at + 2], jpeg[at + 3]]));
        if length < 2 || at + 2 + length > jpeg.len() {
            return Err(format!(
                "the segment at byte {at} says it is {length} bytes long"
            ));
        }
        let segment = Segment {
            marker,
            start: at,
            end: at + 2 + length,
        };
        out.push(segment);
        // Start of scan: the entropy-coded data follows, and with it the image.
        if marker == 0xDA {
            return Ok(out);
        }
        at = segment.end;
    }
}

/// The image's width and height, from its start-of-frame segment (any of the SOF markers C0 to
/// CF but C4, C8 and CC, which are tables and a reserved code).
pub fn dimensions(jpeg: &[u8]) -> Result<(u32, u32), String> {
    let sof = segments(jpeg)?
        .into_iter()
        .find(|s| (0xC0..=0xCF).contains(&s.marker) && ![0xC4, 0xC8, 0xCC].contains(&s.marker))
        .ok_or("it has no start-of-frame segment")?;
    let p = sof.payload(jpeg);
    if p.len() < 5 {
        return Err("its start-of-frame segment is cut short".into());
    }
    // Precision (1 byte), then height and width, 2 bytes each.
    let height = u32::from(u16::from_be_bytes([p[1], p[2]]));
    let width = u32::from(u16::from_be_bytes([p[3], p[4]]));
    Ok((width, height))
}

/// The XMP packet of the file's first XMP APP1 segment, if it has one.
pub fn xmp_of(jpeg: &[u8]) -> Option<String> {
    segments(jpeg).ok()?.into_iter().find_map(|s| {
        let p = s.payload(jpeg);
        (s.marker == 0xE1 && p.starts_with(XMP_NAMESPACE))
            .then(|| String::from_utf8_lossy(&p[XMP_NAMESPACE.len()..]).into_owned())
    })
}

/// The APP1 segment carrying `xmp`.
pub fn app1(xmp: &str) -> Result<Vec<u8>, String> {
    let length = 2 + XMP_NAMESPACE.len() + xmp.len();
    let length = u16::try_from(length).map_err(|_| {
        format!(
            "an XMP packet of {} bytes is too long for one segment",
            xmp.len()
        )
    })?;
    let mut out = vec![0xFF, 0xE1];
    out.extend(length.to_be_bytes());
    out.extend(XMP_NAMESPACE);
    out.extend(xmp.as_bytes());
    Ok(out)
}

/// `jpeg` with the Photo Sphere XMP for its own size added, after the start of image and any
/// JFIF APP0 segments. Refuses a file that already has an XMP segment.
pub fn with_photo_sphere(jpeg: &[u8]) -> Result<Vec<u8>, String> {
    let why = |e: String| format!("the photograph cannot be marked as a 360-degree one: {e}");
    let segments = segments(jpeg).map_err(why)?;
    if xmp_of(jpeg).is_some() {
        return Err(why("it already carries XMP".into()));
    }
    let (width, height) = dimensions(jpeg).map_err(why)?;
    let segment = app1(&xmp(width as usize, height as usize)).map_err(why)?;
    let at = segments
        .iter()
        .take_while(|s| s.marker == 0xE0)
        .last()
        .map_or(2, |s| s.end);
    let mut out = Vec::with_capacity(jpeg.len() + segment.len());
    out.extend_from_slice(&jpeg[..at]);
    out.extend(segment);
    out.extend_from_slice(&jpeg[at..]);
    Ok(out)
}
