//! From tristimulus values to the linear RGB of the star maps and the renderer, and the
//! chromaticity diagrams.
//!
//! # The RGB space
//!
//! Linear light with the primaries of Rec. ITU-R BT.709 and its D65 white, the space of the star
//! maps (sky/maps/README.md, "Colour and values": OpenEXR's default when no `chromaticities`
//! attribute is present) and of the renderer's output (BT.709, as sRGB is). BT.709-6, item 1.3 and
//! 1.4, gives the chromaticities
//!
//!     R (0.640, 0.330)    G (0.300, 0.600)    B (0.150, 0.060)    white D65 (0.3127, 0.3290).
//!
//! The matrix is derived from them, not copied, in the standard way: a primary with chromaticity
//! (x, y) and luminance 1 has XYZ = (x / y, 1, (1 - x - y) / y). Put those three columns in P; the
//! RGB of white is (1, 1, 1) by definition of the space, so the primaries' luminances S solve
//! P S = W with W = (x_w / y_w, 1, (1 - x_w - y_w) / y_w), and RGB -> XYZ is P diag(S). It is
//! evaluated at compile time (`const`) with a closed-form 3 x 3 inverse, in f64:
//!
//!     [0.412391 0.357584 0.180481]            [ 3.240970 -1.537383 -0.498611]
//!     [0.212639 0.715169 0.072192]   and      [-0.969244  1.875968  0.041555]
//!     [0.019331 0.119195 0.950532]            [ 0.055630 -0.203977  1.056972]
//!
//! The middle row of the first is the luminance of a linear BT.709 colour; BT.709 prints it
//! rounded as 0.2126, 0.7152, 0.0722. IEC 61966-2-1 (sRGB) prints the same matrix to four
//! decimals. Both are tested.
//!
//! The white here is BT.709's four-digit (0.3127, 0.3290); the CIE's D65 spectrum summed against
//! the 1 nm table gives (0.312727, 0.329023). The difference, 3e-5, is inside the rounding of the
//! standard; an RGB of (1, 1, 1) means BT.709's white, which is the white the maps and the video
//! are defined with.
//!
//! # Chromaticity diagrams
//!
//! CIE 1931 (x, y) = (X, Y) / (X + Y + Z), for reporting. CIE 1960 UCS (u, v) =
//! (4X, 6Y) / (X + 15Y + 3Z), in which the correlated colour temperature and Duv are defined
//! (CIE 015:2018, section 9.5). The CIE 1976 (u', v') is (u, 1.5 v), used for reporting colour
//! differences.

type Mat3 = [[f64; 3]; 3];

/// The chromaticities of BT.709's primaries and white, (x, y).
pub const BT709_RED: (f64, f64) = (0.640, 0.330);
pub const BT709_GREEN: (f64, f64) = (0.300, 0.600);
pub const BT709_BLUE: (f64, f64) = (0.150, 0.060);
pub const BT709_WHITE: (f64, f64) = (0.3127, 0.3290);

const fn xyz_of_chromaticity(c: (f64, f64)) -> [f64; 3] {
    [c.0 / c.1, 1.0, (1.0 - c.0 - c.1) / c.1]
}

const fn inverse(m: Mat3) -> Mat3 {
    // The adjugate over the determinant.
    let c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    let c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    [
        [
            c00 / det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det,
        ],
        [
            c01 / det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det,
        ],
        [
            c02 / det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det,
        ],
    ]
}

const fn rgb_to_xyz_matrix(r: (f64, f64), g: (f64, f64), b: (f64, f64), w: (f64, f64)) -> Mat3 {
    let (pr, pg, pb) = (
        xyz_of_chromaticity(r),
        xyz_of_chromaticity(g),
        xyz_of_chromaticity(b),
    );
    let p = [
        [pr[0], pg[0], pb[0]],
        [pr[1], pg[1], pb[1]],
        [pr[2], pg[2], pb[2]],
    ];
    let pi = inverse(p);
    let wv = xyz_of_chromaticity(w);
    let mut s = [0.0; 3];
    let mut i = 0;
    while i < 3 {
        s[i] = pi[i][0] * wv[0] + pi[i][1] * wv[1] + pi[i][2] * wv[2];
        i += 1;
    }
    [
        [p[0][0] * s[0], p[0][1] * s[1], p[0][2] * s[2]],
        [p[1][0] * s[0], p[1][1] * s[1], p[1][2] * s[2]],
        [p[2][0] * s[0], p[2][1] * s[1], p[2][2] * s[2]],
    ]
}

/// Linear BT.709 RGB to CIE XYZ (Y = 1 for RGB white).
pub const RGB_TO_XYZ: Mat3 = rgb_to_xyz_matrix(BT709_RED, BT709_GREEN, BT709_BLUE, BT709_WHITE);
/// CIE XYZ to linear BT.709 RGB.
pub const XYZ_TO_RGB: Mat3 = inverse(RGB_TO_XYZ);

/// `m v`.
#[inline]
pub fn apply(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Linear BT.709 RGB to XYZ.
#[inline]
pub fn rgb_to_xyz(rgb: [f64; 3]) -> [f64; 3] {
    apply(&RGB_TO_XYZ, rgb)
}

/// XYZ to linear BT.709 RGB.
#[inline]
pub fn xyz_to_rgb(xyz: [f64; 3]) -> [f64; 3] {
    apply(&XYZ_TO_RGB, xyz)
}

/// The luminance Y of a linear BT.709 colour (1 for white).
#[inline]
pub fn luminance(rgb: [f64; 3]) -> f64 {
    RGB_TO_XYZ[1][0] * rgb[0] + RGB_TO_XYZ[1][1] * rgb[1] + RGB_TO_XYZ[1][2] * rgb[2]
}

/// CIE 1931 chromaticity (x, y).
pub fn xy(xyz: [f64; 3]) -> (f64, f64) {
    let s = xyz[0] + xyz[1] + xyz[2];
    (xyz[0] / s, xyz[1] / s)
}

/// CIE 1960 UCS chromaticity (u, v).
#[inline]
pub fn uv(xyz: [f64; 3]) -> (f64, f64) {
    let d = xyz[0] + 15.0 * xyz[1] + 3.0 * xyz[2];
    (4.0 * xyz[0] / d, 6.0 * xyz[1] / d)
}

/// The CIE 1976 u'v' distance between two colours, a reporting measure of how different two
/// chromaticities look (0.001 to 0.004 is about the least difference an observer notices,
/// depending on the colour and the conditions).
pub fn delta_u_prime_v_prime(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (ua, va) = uv(a);
    let (ub, vb) = uv(b);
    (ua - ub).hypot(1.5 * (va - vb))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_derived_matrix_agrees_with_iec_61966_2_1_to_its_four_decimals() {
        // IEC 61966-2-1:1999, equation (7) forward and its inverse, printed to four decimals.
        let iec = [
            [0.4124, 0.3576, 0.1805],
            [0.2126, 0.7152, 0.0722],
            [0.0193, 0.1192, 0.9505],
        ];
        let iec_inverse = [
            [3.2406, -1.5372, -0.4986],
            [-0.9689, 1.8758, 0.0415],
            [0.0557, -0.2040, 1.0570],
        ];
        for i in 0..3 {
            for j in 0..3 {
                // Rounding allows 5e-5; the forward matrix is within it.
                assert!((RGB_TO_XYZ[i][j] - iec[i][j]).abs() < 5e-5, "{i}{j}");
                // IEC's inverse was computed from its rounded forward matrix and then rounded, so
                // it carries the forward rounding amplified by the inverse's size (~3): 5e-4.
                assert!(
                    (XYZ_TO_RGB[i][j] - iec_inverse[i][j]).abs() < 5e-4,
                    "{i}{j}"
                );
            }
        }
    }

    #[test]
    fn test_the_luminance_row_is_bt709s_published_weights() {
        let w = [0.2126, 0.7152, 0.0722];
        for j in 0..3 {
            assert!((RGB_TO_XYZ[1][j] - w[j]).abs() < 5e-5);
        }
        assert!((luminance([1.0, 1.0, 1.0]) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn test_the_d65_white_maps_to_equal_rgb_and_each_primary_to_its_chromaticity() {
        let rgb = xyz_to_rgb(xyz_of_chromaticity(BT709_WHITE));
        for c in rgb {
            assert!((c - 1.0).abs() < 1e-15, "{rgb:?}");
        }
        for (k, want) in [BT709_RED, BT709_GREEN, BT709_BLUE].iter().enumerate() {
            let mut e = [0.0; 3];
            e[k] = 1.0;
            let (x, y) = xy(rgb_to_xyz(e));
            assert!((x - want.0).abs() < 1e-15 && (y - want.1).abs() < 1e-15);
        }
    }

    #[test]
    fn test_the_matrix_and_its_inverse_multiply_to_the_identity() {
        for i in 0..3 {
            for j in 0..3 {
                let p: f64 = (0..3).map(|k| RGB_TO_XYZ[i][k] * XYZ_TO_RGB[k][j]).sum();
                let q: f64 = (0..3).map(|k| XYZ_TO_RGB[i][k] * RGB_TO_XYZ[k][j]).sum();
                let id = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (p - id).abs() < 1e-15 && (q - id).abs() < 1e-15,
                    "{i}{j}: {p} {q}"
                );
            }
        }
    }

    #[test]
    fn test_rgb_to_xyz_has_only_positive_entries_so_a_non_negative_pixel_has_non_negative_xyz() {
        // What the shift's domain rests on: every pixel of the map (channels in [0, 1]) is a
        // colour with X, Y, Z >= 0.
        for row in RGB_TO_XYZ {
            for v in row {
                assert!(v > 0.0);
            }
        }
    }
}
