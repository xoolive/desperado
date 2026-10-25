//! Input and station-tuning configuration.

use std::path::Path;
use std::str::FromStr;

use desperado::{IqFormat, gqrx::parse_gqrx_filename};
use thiserror::Error;

/// NRSC-5 FM decoder input rate after channel selection.
pub const NRSC5_FM_SAMPLE_RATE_HZ: u32 = 744_188;

/// Fully resolved raw-I/Q input configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputConfig {
    /// Capture RF centre frequency in Hz.
    pub center_frequency_hz: u32,
    /// Complex sample rate in samples/s.
    pub sample_rate_hz: u32,
    /// On-disk I/Q representation.
    pub format: IqFormat,
}

/// Fully resolved receiver configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceiverConfig {
    /// Resolved source path or future SDR URI.
    pub source: String,
    /// Raw capture configuration.
    pub input: InputConfig,
    /// Requested FM station carrier centre in Hz.
    pub station_frequency_hz: u32,
    /// HD program number (HD1 is 0).
    pub program: u8,
}

/// User-facing configuration error.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ConfigError {
    /// The source name lacks GQRX metadata and no explicit value was supplied.
    #[error("{0} is required unless it can be inferred from a GQRX file name")]
    Missing(&'static str),
    /// Frequency argument was malformed.
    #[error("invalid frequency {0:?}; use Hz, kHz, MHz, or an SI suffix such as 106.7M")]
    InvalidFrequency(String),
    /// Format was unsupported by Desperado.
    #[error("invalid I/Q format {0:?}")]
    InvalidFormat(String),
}

/// Parse a frequency with optional Hz/kHz/MHz/GHz or SI suffix.
pub fn parse_frequency_hz(value: &str) -> Result<u32, ConfigError> {
    let normalized = value.trim().to_ascii_lowercase();
    if normalized.is_empty() {
        return Err(ConfigError::InvalidFrequency(value.to_owned()));
    }

    let (number, multiplier) = if let Some(number) = normalized.strip_suffix("ghz") {
        (number, 1_000_000_000.0)
    } else if let Some(number) = normalized.strip_suffix('g') {
        (number, 1_000_000_000.0)
    } else if let Some(number) = normalized.strip_suffix("mhz") {
        (number, 1_000_000.0)
    } else if let Some(number) = normalized.strip_suffix('m') {
        (number, 1_000_000.0)
    } else if let Some(number) = normalized.strip_suffix("khz") {
        (number, 1_000.0)
    } else if let Some(number) = normalized.strip_suffix('k') {
        (number, 1_000.0)
    } else if let Some(number) = normalized.strip_suffix("hz") {
        (number, 1.0)
    } else {
        (normalized.as_str(), 1.0)
    };

    let hz = number
        .trim()
        .parse::<f64>()
        .ok()
        .map(|number| number * multiplier)
        .filter(|hz| hz.is_finite() && *hz > 0.0 && *hz <= u32::MAX as f64)
        .ok_or_else(|| ConfigError::InvalidFrequency(value.to_owned()))?;
    Ok(hz.round() as u32)
}

impl ReceiverConfig {
    /// Resolve explicit settings over GQRX filename metadata.
    pub fn resolve(
        source: String,
        station_frequency_hz: Option<u32>,
        sample_rate_hz: Option<u32>,
        format: Option<&str>,
        program: u8,
    ) -> Result<Self, ConfigError> {
        let metadata = (!source.eq("-") && !source.contains("://"))
            .then(|| parse_gqrx_filename(Path::new(&source)))
            .flatten();

        let center_frequency_hz = metadata
            .map(|metadata| metadata.center_freq_hz)
            .ok_or(ConfigError::Missing("capture centre frequency"))?;
        let station_frequency_hz = station_frequency_hz.unwrap_or(center_frequency_hz);
        let sample_rate_hz = sample_rate_hz
            .or_else(|| metadata.map(|metadata| metadata.sample_rate_hz))
            .ok_or(ConfigError::Missing("sample rate"))?;
        let format = match format {
            Some(format) => IqFormat::from_str(format)
                .map_err(|_| ConfigError::InvalidFormat(format.to_owned()))?,
            None => metadata
                .map(|metadata| metadata.format)
                .ok_or(ConfigError::Missing("I/Q format"))?,
        };

        Ok(Self {
            source,
            input: InputConfig {
                center_frequency_hz,
                sample_rate_hz,
                format,
            },
            station_frequency_hz,
            program,
        })
    }

    /// Signed baseband frequency of the station relative to the capture centre.
    pub fn station_offset_hz(&self) -> i64 {
        i64::from(self.station_frequency_hz) - i64::from(self.input.center_frequency_hz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_common_frequency_forms() {
        assert_eq!(parse_frequency_hz("106.7M").unwrap(), 106_700_000);
        assert_eq!(parse_frequency_hz("106.7 MHz").unwrap(), 106_700_000);
        assert_eq!(parse_frequency_hz("106700k").unwrap(), 106_700_000);
        assert_eq!(parse_frequency_hz("106700000").unwrap(), 106_700_000);
    }

    #[test]
    fn resolves_gqrx_capture_and_explicit_station() {
        let config = ReceiverConfig::resolve(
            "gqrx_20260916_220902_106800000_2400000_fc.raw".to_owned(),
            Some(106_700_000),
            None,
            None,
            0,
        )
        .unwrap();
        assert_eq!(config.input.center_frequency_hz, 106_800_000);
        assert_eq!(config.input.sample_rate_hz, 2_400_000);
        assert_eq!(config.input.format, IqFormat::Cf32);
        assert_eq!(config.station_offset_hz(), -100_000);
    }
}
