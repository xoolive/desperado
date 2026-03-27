//! Timestamp and file-based time resolution helpers.
//!
//! Provides utilities for:
//! - Getting current Unix timestamp
//! - Resolving file start time from GQRX filename or filesystem metadata
//! - Parsing GQRX timestamp format (`gqrx_YYYYMMDD_HHMMSS_*`)

use chrono::NaiveDateTime;
use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

/// Get current Unix timestamp as a floating-point seconds value.
pub fn unix_now_seconds() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Resolve the start time of a file (in Unix seconds) from:
/// 1. GQRX filename pattern (`gqrx_YYYYMMDD_HHMMSS_*`)
/// 2. File creation time
/// 3. File modification time
///
/// Returns an error if none of the above succeed.
pub fn resolve_file_start_unix(path: &Path) -> Result<f64, io::Error> {
    if let Some(ts) = parse_gqrx_start_unix(path) {
        return Ok(ts);
    }

    let meta = std::fs::metadata(path)?;
    if let Ok(created) = meta.created() {
        return Ok(created
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0));
    }

    if let Ok(modified) = meta.modified() {
        return Ok(modified
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0));
    }

    Err(io::Error::other(
        "Could not determine absolute start time for input file",
    ))
}

/// Parse a GQRX timestamp from a filename.
///
/// GQRX files are named: `gqrx_YYYYMMDD_HHMMSS_freq_rate_fc.raw`
/// This function extracts the date/time components and converts to Unix timestamp.
///
/// Returns `None` if the filename doesn't match the GQRX pattern or timestamp parsing fails.
pub fn parse_gqrx_start_unix(path: &Path) -> Option<f64> {
    let name = path.file_name()?.to_str()?;
    if !name.starts_with("gqrx_") {
        return None;
    }

    let mut parts = name.split('_');
    if parts.next()? != "gqrx" {
        return None;
    }

    let date = parts.next()?;
    let time = parts.next()?;
    if date.len() != 8 || time.len() != 6 {
        return None;
    }

    let dt = NaiveDateTime::parse_from_str(&format!("{date}{time}"), "%Y%m%d%H%M%S").ok()?;
    Some(dt.and_utc().timestamp() as f64)
}
