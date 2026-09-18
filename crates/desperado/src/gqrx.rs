//! GQRX I/Q file metadata detection.
//!
//! GQRX records raw I/Q files with an encoding of the capture parameters in the
//! file name, following the convention:
//!
//! ```text
//! gqrx_<YYYYMMDD>_<HHMMSS>_<freqHz>_<sampleRateHz>_<format>.<ext>
//! ```
//!
//! For example `gqrx_20260708_153923_220936000_16000000_fc.raw` describes a
//! 16 MS/s complex-float capture centred on 220.936 MHz.
//!
//! This module parses that convention into reusable metadata so that downstream
//! decoders (dabradio, voracious, jet1090, ...) can auto-configure center
//! frequency, sample rate and I/Q format from the file name alone, with an
//! explicit CLI flag still able to override each field.
//!
//! Parsing is intentionally strict: only names that match the full convention
//! are recognised, so unrelated files fall back to the caller's defaults.

use std::path::Path;

use crate::IqFormat;

/// Capture metadata extracted from a GQRX file name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GqrxMeta {
    /// RF center frequency of the capture, in Hz.
    pub center_freq_hz: u32,
    /// Input sample rate, in samples per second.
    pub sample_rate_hz: u32,
    /// I/Q sample format.
    pub format: IqFormat,
}

/// Parse GQRX capture metadata from a file path.
///
/// Returns `None` when the file name does not follow the GQRX convention or
/// when any field cannot be parsed (including an unknown format token). The
/// file extension is not required to be `.raw`, but the stem must match the
/// full `gqrx_<date>_<time>_<freq>_<rate>_<fmt>` layout.
///
/// # Examples
///
/// ```
/// use desperado::gqrx::parse_gqrx_filename;
/// use desperado::IqFormat;
///
/// let meta = parse_gqrx_filename("gqrx_20260708_153923_220936000_16000000_fc.raw").unwrap();
/// assert_eq!(meta.center_freq_hz, 220_936_000);
/// assert_eq!(meta.sample_rate_hz, 16_000_000);
/// assert_eq!(meta.format, IqFormat::Cf32);
///
/// // Non-GQRX names are ignored (caller falls back to its own defaults).
/// assert!(parse_gqrx_filename("dab_12A_2min.iq").is_none());
/// ```
pub fn parse_gqrx_filename(path: impl AsRef<Path>) -> Option<GqrxMeta> {
    let path = path.as_ref();
    let file_name = path.file_name()?.to_str()?;
    // Compression is a transport suffix, not part of the GQRX capture name.
    // Strip it before stripping the underlying `.raw`/`.iq` extension.
    let uncompressed_name = file_name
        .strip_suffix(".zst")
        .or_else(|| file_name.strip_suffix(".ZST"))
        .unwrap_or(file_name);
    let stem = Path::new(uncompressed_name).file_stem()?.to_str()?;
    parse_gqrx_stem(stem)
}

/// Parse GQRX metadata from a file-name stem (without extension).
fn parse_gqrx_stem(stem: &str) -> Option<GqrxMeta> {
    let parts: Vec<&str> = stem.split('_').collect();
    if parts.len() < 6 {
        return None;
    }

    // The first four fields (`gqrx`, date, time, freq, rate, fmt) must all be
    // present and well-formed. We loosely validate the date/time fields as
    // all-digit runs of the expected width to reduce false positives.
    if !parts[0].eq_ignore_ascii_case("gqrx") {
        return None;
    }
    if parts[1].len() != 8 || !parts[1].bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    if parts[2].len() != 6 || !parts[2].bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }

    let center_freq_hz: u32 = parts[3].parse().ok()?;
    let sample_rate_hz: u32 = parts[4].parse().ok()?;
    let format = parse_format_token(parts[5])?;

    // Any trailing fields (rare, but tolerate extra `_` segments after the
    // format token) are ignored — only the leading layout matters.

    if center_freq_hz == 0 || sample_rate_hz == 0 {
        return None;
    }

    Some(GqrxMeta {
        center_freq_hz,
        sample_rate_hz,
        format,
    })
}

/// Map a GQRX format token to an [`IqFormat`].
///
/// GQRX labels complex-float captures `fc`; other tools sometimes use `cf32`.
/// Both are accepted, as are the bare integer variants.
fn parse_format_token(token: &str) -> Option<IqFormat> {
    match token.to_ascii_lowercase().as_str() {
        "fc" | "cf32" | "float" | "f32" => Some(IqFormat::Cf32),
        "cu8" | "u8" => Some(IqFormat::Cu8),
        "cs8" | "s8" => Some(IqFormat::Cs8),
        "cs16" | "s16" | "c16" => Some(IqFormat::Cs16),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_wideband_16msps_file() {
        let meta = parse_gqrx_filename("gqrx_20260708_153923_220936000_16000000_fc.raw")
            .expect("wideband gqrx file should parse");
        assert_eq!(meta.center_freq_hz, 220_936_000);
        assert_eq!(meta.sample_rate_hz, 16_000_000);
        assert_eq!(meta.format, IqFormat::Cf32);
    }

    #[test]
    fn parse_existing_12a_2msps_file() {
        let meta = parse_gqrx_filename("gqrx_20260314_131107_223936000_2048000_fc.raw")
            .expect("narrowband gqrx file should parse");
        assert_eq!(meta.center_freq_hz, 223_936_000);
        assert_eq!(meta.sample_rate_hz, 2_048_000);
        assert_eq!(meta.format, IqFormat::Cf32);
    }

    #[test]
    fn parses_zstd_compressed_gqrx_name() {
        let meta = parse_gqrx_filename("gqrx_20260314_131130_223936000_2048000_fc.raw.zst")
            .expect("compressed gqrx file should parse");
        assert_eq!(meta.center_freq_hz, 223_936_000);
        assert_eq!(meta.sample_rate_hz, 2_048_000);
        assert_eq!(meta.format, IqFormat::Cf32);
    }

    #[test]
    fn parse_voracious_style_gqrx_name() {
        // VOR captures in the ecosystem also follow the gqrx convention.
        let meta = parse_gqrx_filename("gqrx_20250925_144051_114647000_1800000_fc.raw").unwrap();
        assert_eq!(meta.center_freq_hz, 114_647_000);
        assert_eq!(meta.sample_rate_hz, 1_800_000);
        assert_eq!(meta.format, IqFormat::Cf32);
    }

    #[test]
    fn parse_high_frequency_field() {
        // 10-digit frequency (>1 GHz) must parse — the field is not fixed width.
        let meta = parse_gqrx_filename("gqrx_20251107_183809_1194000000_1800000_fc.raw").unwrap();
        assert_eq!(meta.center_freq_hz, 1_194_000_000);
    }

    #[test]
    fn ignores_non_gqrx_names() {
        assert!(parse_gqrx_filename("dab_12A_2min.iq").is_none());
        assert!(parse_gqrx_filename("rtlsdr_199360000_2048000_dabradio_8C.cu8").is_none());
        assert!(parse_gqrx_filename("ais_paris_162m_288k.bin").is_none());
        assert!(parse_gqrx_filename("sample.cf32.iq").is_none());
    }

    #[test]
    fn rejects_malformed_gqrx_names() {
        // Bad date/time width.
        assert!(parse_gqrx_filename("gqrx_2026070_153923_220936000_16000000_fc.raw").is_none());
        assert!(parse_gqrx_filename("gqrx_20260708_15392_220936000_16000000_fc.raw").is_none());
        // Non-numeric freq/rate.
        assert!(parse_gqrx_filename("gqrx_20260708_153923_220M_16000000_fc.raw").is_none());
        // Unknown format token.
        assert!(parse_gqrx_filename("gqrx_20260708_153923_220936000_16000000_xx.raw").is_none());
        // Too few fields.
        assert!(parse_gqrx_filename("gqrx_20260708_153923").is_none());
        // Wrong prefix.
        assert!(parse_gqrx_filename("sdruno_20260708_153923_220936000_16000000_fc.raw").is_none());
    }

    #[test]
    fn accepts_path_objects_and_alternate_extensions() {
        use std::path::Path;
        let meta = parse_gqrx_filename(Path::new(
            "/tmp/gqrx_20260708_153923_220936000_16000000_fc.iq",
        ))
        .unwrap();
        assert_eq!(meta.sample_rate_hz, 16_000_000);
    }

    #[test]
    fn format_token_aliases() {
        assert_eq!(parse_format_token("fc"), Some(IqFormat::Cf32));
        assert_eq!(parse_format_token("CF32"), Some(IqFormat::Cf32));
        assert_eq!(parse_format_token("cu8"), Some(IqFormat::Cu8));
        assert_eq!(parse_format_token("cs16"), Some(IqFormat::Cs16));
        assert_eq!(parse_format_token("unknown"), None);
    }
}
