//! Capture metadata from common SDR recording file names.
//!
//! Many recording tools encode the capture parameters in the file name. This
//! module recognises the common conventions and returns whatever each one
//! carries, so decoders can configure themselves from the file name alone.
//! Explicit user settings should still take precedence.
//!
//! | Convention | Example | Carries |
//! |---|---|---|
//! | GQRX | `gqrx_20260603_083011_134000000_8000000_fc.raw` | center, rate, format, start (UTC) |
//! | SDRuno | `SDRuno_20200908_152020Z_129535kHz.wav` | center, start (UTC) |
//! | rtl_sdr (dated) | `rtlsdr_20260728_94500000_1024000_cu8.iq` | center, rate, format |
//! | rtl_sdr | `rtlsdr_136850000_1050000_dumpvdl_6min.rtl` | center, rate, format (`cu8`) |
//!
//! A trailing `.zst` is ignored, so compressed captures are recognised too.
//!
//! ```
//! use desperado::capture::{parse_capture_filename, CaptureConvention};
//! use std::time::{Duration, UNIX_EPOCH};
//!
//! let meta = parse_capture_filename("gqrx_20260603_083011_134000000_8000000_fc.raw.zst").unwrap();
//! assert_eq!(meta.convention, CaptureConvention::Gqrx);
//! assert_eq!(meta.center_freq_hz, Some(134_000_000));
//! assert_eq!(meta.start, Some(UNIX_EPOCH + Duration::from_secs(1_780_475_411)));
//! ```

use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::IqFormat;

/// File-naming convention a capture was recognised by.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CaptureConvention {
    /// `gqrx_<YYYYMMDD>_<HHMMSS>_<freqHz>_<rateHz>_<fmt>`
    Gqrx,
    /// `SDRuno_<YYYYMMDD>_<HHMMSS>Z_<freq>kHz`
    Sdruno,
    /// `rtlsdr_[<YYYYMMDD>_]<freqHz>_<rateHz>[_...]`
    RtlSdr,
}

/// Capture parameters recovered from a file name. Fields a convention does
/// not encode are `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct CaptureMeta {
    /// Convention the name matched.
    pub convention: CaptureConvention,
    /// RF center frequency, in Hz.
    pub center_freq_hz: Option<u32>,
    /// Sample rate, in samples per second.
    pub sample_rate_hz: Option<u32>,
    /// Raw I/Q sample format. `None` for container formats such as WAV.
    pub format: Option<IqFormat>,
    /// Start of the recording, when the name carries a full UTC date and time.
    pub start: Option<SystemTime>,
}

/// Recognise a capture file name. Returns `None` for unrecognised names.
pub fn parse_capture_filename(path: impl AsRef<Path>) -> Option<CaptureMeta> {
    let path = path.as_ref();
    let stem = uncompressed_stem(path)?;
    let parts: Vec<&str> = stem.split('_').collect();
    let prefix = parts.first()?.to_ascii_lowercase();
    match prefix.as_str() {
        "gqrx" => parse_gqrx(path, &parts),
        "sdruno" => parse_sdruno(&parts),
        "rtlsdr" => parse_rtlsdr(path, &parts),
        _ => None,
    }
}

fn uncompressed_stem(path: &Path) -> Option<String> {
    let file_name = path.file_name()?.to_str()?;
    let name = file_name
        .strip_suffix(".zst")
        .or_else(|| file_name.strip_suffix(".ZST"))
        .unwrap_or(file_name);
    Some(Path::new(name).file_stem()?.to_str()?.to_string())
}

fn parse_gqrx(path: &Path, parts: &[&str]) -> Option<CaptureMeta> {
    let meta = crate::gqrx::parse_gqrx_filename(path)?;
    Some(CaptureMeta {
        convention: CaptureConvention::Gqrx,
        center_freq_hz: Some(meta.center_freq_hz),
        sample_rate_hz: Some(meta.sample_rate_hz),
        format: Some(meta.format),
        // GQRX names recordings from the current UTC date and time.
        start: utc_from_parts(parts[1], parts[2]),
    })
}

fn parse_sdruno(parts: &[&str]) -> Option<CaptureMeta> {
    if parts.len() < 4 {
        return None;
    }
    let time = parts[2].strip_suffix(['Z', 'z']).filter(|t| t.len() == 6)?;
    let start = utc_from_parts(parts[1], time)?;
    let freq = parts[3];
    let khz = freq
        .get(..freq.len().checked_sub(3)?)
        .filter(|_| freq[freq.len() - 3..].eq_ignore_ascii_case("khz"))?;
    let center_freq_hz = (khz.parse::<f64>().ok()? * 1_000.0).round();
    (center_freq_hz > 0.0 && center_freq_hz <= u32::MAX as f64).then_some(CaptureMeta {
        convention: CaptureConvention::Sdruno,
        center_freq_hz: Some(center_freq_hz as u32),
        sample_rate_hz: None,
        format: None,
        start: Some(start),
    })
}

fn parse_rtlsdr(path: &Path, parts: &[&str]) -> Option<CaptureMeta> {
    let is_date = |s: &str| s.len() == 8 && s.bytes().all(|b| b.is_ascii_digit());
    // Optional date (no time of day, so it does not give a start time).
    let rest = if parts.len() > 1 && is_date(parts[1]) {
        &parts[2..]
    } else {
        &parts[1..]
    };
    let center_freq_hz: u32 = rest.first()?.parse().ok()?;
    let sample_rate_hz: u32 = rest.get(1)?.parse().ok()?;
    if center_freq_hz == 0 || sample_rate_hz == 0 {
        return None;
    }
    // Format from the next token or the extension; rtl_sdr itself writes cu8.
    let extension = path
        .file_name()
        .and_then(|n| n.to_str())
        .map(|n| n.trim_end_matches(".zst").trim_end_matches(".ZST"))
        .and_then(|n| Path::new(n).extension().and_then(|e| e.to_str()));
    let format = rest
        .get(2)
        .and_then(|t| t.parse().ok())
        .or_else(|| extension.and_then(|e| e.parse().ok()))
        .unwrap_or(IqFormat::Cu8);
    Some(CaptureMeta {
        convention: CaptureConvention::RtlSdr,
        center_freq_hz: Some(center_freq_hz),
        sample_rate_hz: Some(sample_rate_hz),
        format: Some(format),
        start: None,
    })
}

/// Convert `YYYYMMDD` and `HHMMSS` (UTC) to a `SystemTime`.
fn utc_from_parts(date: &str, time: &str) -> Option<SystemTime> {
    let digits = |s: &str, n: usize| s.len() == n && s.bytes().all(|b| b.is_ascii_digit());
    if !digits(date, 8) || !digits(time, 6) {
        return None;
    }
    let num = |s: &str| s.parse::<u32>().ok();
    let (year, month, day) = (num(&date[..4])?, num(&date[4..6])?, num(&date[6..])?);
    let (hour, minute, second) = (num(&time[..2])?, num(&time[2..4])?, num(&time[4..])?);
    if !(1..=12).contains(&month)
        || day == 0
        || day > days_in_month(year, month)
        || hour > 23
        || minute > 59
        || second > 60
    {
        return None;
    }
    let days = days_from_civil(year as i64, month, day);
    let secs = days * 86_400 + i64::from(hour * 3_600 + minute * 60 + second);
    u64::try_from(secs)
        .ok()
        .map(|s| UNIX_EPOCH + Duration::from_secs(s))
}

fn days_in_month(year: u32, month: u32) -> u32 {
    match month {
        2 if (year.is_multiple_of(4) && !year.is_multiple_of(100)) || year.is_multiple_of(400) => {
            29
        }
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// Days since 1970-01-01 for a proleptic Gregorian date (H. Hinnant's algorithm).
fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    let y = if month <= 2 { year - 1 } else { year };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let m = i64::from(month);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + i64::from(day) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: u64) -> Option<SystemTime> {
        Some(UNIX_EPOCH + Duration::from_secs(secs))
    }

    #[test]
    fn gqrx_with_start_time() {
        let meta =
            parse_capture_filename("/data/gqrx_20260518_114025_136500000_1800000_fc.raw").unwrap();
        assert_eq!(meta.convention, CaptureConvention::Gqrx);
        assert_eq!(meta.center_freq_hz, Some(136_500_000));
        assert_eq!(meta.sample_rate_hz, Some(1_800_000));
        assert_eq!(meta.format, Some(IqFormat::Cf32));
        assert_eq!(meta.start, at(1_779_104_425));
    }

    #[test]
    fn compressed_gqrx() {
        let meta =
            parse_capture_filename("gqrx_20260603_083011_134000000_8000000_fc.raw.zst").unwrap();
        assert_eq!(meta.sample_rate_hz, Some(8_000_000));
        assert_eq!(meta.start, at(1_780_475_411));
    }

    #[test]
    fn sdruno() {
        let meta = parse_capture_filename("SDRuno_20200908_152020Z_129535kHz.wav").unwrap();
        assert_eq!(meta.convention, CaptureConvention::Sdruno);
        assert_eq!(meta.center_freq_hz, Some(129_535_000));
        assert_eq!(meta.sample_rate_hz, None);
        assert_eq!(meta.format, None);
        assert_eq!(meta.start, at(1_599_578_420));
    }

    #[test]
    fn rtlsdr_conventions() {
        let meta = parse_capture_filename("rtlsdr_136850000_1050000_dumpvdl_6min.rtl").unwrap();
        assert_eq!(meta.convention, CaptureConvention::RtlSdr);
        assert_eq!(meta.center_freq_hz, Some(136_850_000));
        assert_eq!(meta.sample_rate_hz, Some(1_050_000));
        assert_eq!(meta.format, Some(IqFormat::Cu8));
        assert_eq!(meta.start, None);

        let meta = parse_capture_filename("rtlsdr_20260728_94500000_1024000_cs16.iq.zst").unwrap();
        assert_eq!(meta.center_freq_hz, Some(94_500_000));
        assert_eq!(meta.format, Some(IqFormat::Cs16));

        let meta = parse_capture_filename("rtlsdr_199360000_2048000_dabradio_8C.cu8").unwrap();
        assert_eq!(meta.format, Some(IqFormat::Cu8));
    }

    #[test]
    fn rejects_unrecognised_or_malformed_names() {
        assert!(parse_capture_filename("capture.cu8").is_none());
        assert!(parse_capture_filename("HFDL_10081kHz.wav").is_none());
        assert!(parse_capture_filename("SDRuno_20200908_152020_129535kHz.wav").is_none());
        assert!(parse_capture_filename("SDRuno_20201308_152020Z_129535kHz.wav").is_none());
        assert!(parse_capture_filename("rtlsdr_dab_12A.iq").is_none());
        // An impossible date keeps the other GQRX fields but gives no start.
        let meta = parse_capture_filename("gqrx_20260230_083011_134000000_8000000_fc.raw").unwrap();
        assert_eq!(meta.start, None);
    }

    #[test]
    fn civil_days() {
        assert_eq!(days_from_civil(1970, 1, 1), 0);
        assert_eq!(days_from_civil(2000, 3, 1), 11_017);
        assert_eq!(days_from_civil(2026, 6, 3), 20_607);
    }
}
