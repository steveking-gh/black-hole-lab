//! One number of a manifest, spelled so that JSON can hold every value an f64 can.

use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// An f64 written as a JSON number where JSON can hold it and as a string where it cannot.
///
/// JSON has no NaN and no infinity, and a manifest can meet both: a read-out of a blueshift factor
/// on a ray that grazed a horizon is infinite, and a read-out the writer could not compute for one
/// frame is NaN. Left to serde_json a non-finite f64 is written as `null` and read back as an
/// error, so a bundle that recorded one could be written and never opened.
///
/// So a finite value is a JSON number, and the three others are the strings `"inf"`, `"-inf"` and
/// `"nan"`: the spelling Black Hole Lab's `.bhl` save uses, so that one reader of both formats
/// needs one rule. Reading also accepts a number that arrived quoted, and `"Infinity"` and `"NaN"`,
/// which is what lets a hand-edited manifest open.
///
/// Finite values round-trip to the bit. serde_json writes the shortest decimal that reads back as
/// the same f64, and the `float_roundtrip` feature makes the parser honour that. Negative zero
/// survives too: `-0.0` is written `-0.0` and read back with its sign bit.
#[derive(Debug, Clone, Copy, Default)]
pub struct Num(pub f64);

impl From<f64> for Num {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<Num> for f64 {
    fn from(value: Num) -> Self {
        value.0
    }
}

/// Equality by bit pattern, with every NaN equal to every other.
///
/// The bits, because that is what a round-trip test claims: a manifest that moved the last bit of
/// a proper time did not keep the run. -0.0 and 0.0 are therefore different values here, as they
/// are different f64s. NaN is the exception: the spelling `"nan"` carries no payload, so two NaNs
/// that differ only in payload come back from a manifest as the same one, and comparing them as
/// equal says what the format promises, which is that a NaN stays a NaN. (The frame files are
/// binary and do keep a NaN's payload; see `Frame`.)
impl PartialEq for Num {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits() || (self.0.is_nan() && other.0.is_nan())
    }
}

impl Serialize for Num {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if self.0.is_finite() {
            serializer.serialize_f64(self.0)
        } else if self.0.is_nan() {
            serializer.serialize_str("nan")
        } else if self.0 > 0.0 {
            serializer.serialize_str("inf")
        } else {
            serializer.serialize_str("-inf")
        }
    }
}

impl<'de> Deserialize<'de> for Num {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct NumVisitor;

        impl Visitor<'_> for NumVisitor {
            type Value = Num;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a number, or one of the strings \"inf\", \"-inf\", \"nan\"")
            }

            fn visit_f64<E: de::Error>(self, value: f64) -> Result<Num, E> {
                Ok(Num(value))
            }

            // serde_json always writes a decimal point, but a manifest written by another program,
            // or edited by hand, may say `0` where it means 0.0.
            fn visit_i64<E: de::Error>(self, value: i64) -> Result<Num, E> {
                Ok(Num(value as f64))
            }

            fn visit_u64<E: de::Error>(self, value: u64) -> Result<Num, E> {
                Ok(Num(value as f64))
            }

            fn visit_str<E: de::Error>(self, value: &str) -> Result<Num, E> {
                match value {
                    "inf" | "+inf" | "Infinity" => Ok(Num(f64::INFINITY)),
                    "-inf" | "-Infinity" => Ok(Num(f64::NEG_INFINITY)),
                    "nan" | "NaN" => Ok(Num(f64::NAN)),
                    other => other.parse::<f64>().map(Num).map_err(|_| {
                        E::custom(format!("{other:?} is not a number this format can read"))
                    }),
                }
            }
        }

        deserializer.deserialize_any(NumVisitor)
    }
}
