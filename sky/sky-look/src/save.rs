//! The three things this program reads from the save itself: the observer's name, the reading of
//! the observer's watch at the saved moment, and whether the app writes its decimal mark as a
//! comma.
//!
//! Everything else in the save is the tracer's to read, and to judge: this reading is lighter,
//! tolerant, and never a reason to refuse. A save it cannot follow is handed to the tracer all the
//! same, which then says what is wrong with it in its own sentence.

use std::io::Read;
use std::path::Path;

use crate::args::Who;

/// What the save says, as far as this program asks.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Facts {
    /// The observer's name as the save gives it; `None` when it does not.
    pub name: Option<String>,
    /// The observer's proper time at the saved moment, in M.
    pub tau: Option<f64>,
    /// The app's "Decimal is comma" setting, which the read-outs on the photograph and the numbers
    /// in this program's sentences follow. False in a save that predates the setting, as the app
    /// itself reads one.
    pub comma: bool,
}

/// Reads `path` - gzip or plain JSON, told apart by the two gzip magic bytes, as the app writes
/// either - and picks out the facts for `who`. Only reads the file: the save belongs to whoever
/// handed it over.
pub fn facts(path: &Path, who: Who) -> Facts {
    let Ok(bytes) = std::fs::read(path) else {
        return Facts::default();
    };
    let text = if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut text = Vec::new();
        if flate2::read::GzDecoder::new(bytes.as_slice())
            .read_to_end(&mut text)
            .is_err()
        {
            return Facts::default();
        }
        text
    } else {
        bytes
    };
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(&text) else {
        return Facts::default();
    };
    let observer = doc.pointer(&format!("/sim/{}", who.flag()));
    Facts {
        name: observer
            .and_then(|o| o.get("name"))
            .and_then(|n| n.as_str())
            .map(str::to_string),
        tau: observer.and_then(|o| o.get("tau")).and_then(|t| t.as_f64()),
        comma: doc
            .pointer("/controls/decimal_is_comma")
            .and_then(|c| c.as_bool())
            .unwrap_or(false),
    }
}
