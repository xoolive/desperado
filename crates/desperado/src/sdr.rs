//! Shared SDR source configuration helpers.
//!
//! This module intentionally models *partial* user-provided SDR settings.  A
//! missing setting (`None`) means "not specified by the user" and should be
//! resolved by the application using protocol/device defaults.  This is
//! different from explicit values such as [`Gain::Auto`], which means the user
//! requested device automatic gain control.

use crate::{Gain, IqFormat};
use serde::{Deserialize, Serialize};

/// Return true if `input` uses a live SDR URI scheme supported by desperado.
pub fn is_device_uri(input: &str) -> bool {
    matches!(
        device_scheme(input),
        Some("rtlsdr" | "airspy" | "hackrf" | "soapy" | "pluto")
    )
}

/// Return the lowercase SDR URI scheme, if present.
pub fn device_scheme(input: &str) -> Option<&str> {
    input.split_once("://").map(|(scheme, _)| scheme)
}

/// Common gain-step tables suitable for interactive gain adjustment.
pub fn gain_steps_for_source(source: &str) -> Vec<f64> {
    match device_scheme(source) {
        Some("rtlsdr") => vec![
            0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7, 16.6, 19.7, 20.7, 22.9, 25.4,
            28.0, 29.7, 32.8, 33.8, 36.4, 37.2, 38.6, 40.2, 42.1, 43.4, 43.9, 44.5, 48.0, 49.6,
        ],
        Some("airspy") => (0..=21).map(|v| v as f64 * 50.0 / 21.0).collect(),
        Some("hackrf") => (0..=51).map(|v| v as f64 * 2.0).collect(),
        _ => (0..=50).map(|v| v as f64).collect(),
    }
}

/// User-provided SDR settings common to live receivers and I/Q files.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct SdrSettings {
    /// Center frequency in Hz.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center_freq: Option<u64>,
    /// Sample rate in samples per second.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sample_rate: Option<u32>,
    /// Gain setting. `None` means use application default; `Some(Gain::Auto)`
    /// means explicit AGC/auto gain.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gain: Option<Gain>,
    /// Enable antenna bias tee where supported.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bias_tee: Option<bool>,
    /// Enable HackRF RF amplifier. This is a boolean, not a numeric gain stage.
    #[serde(default, alias = "rf_amp", skip_serializing_if = "Option::is_none")]
    pub amp_enable: Option<bool>,
    /// RTL-SDR frequency correction in parts per million.
    #[serde(
        default,
        alias = "ppm",
        alias = "freq_correction",
        alias = "freq-correction",
        skip_serializing_if = "Option::is_none"
    )]
    pub freq_correction_ppm: Option<i32>,
    /// File I/Q format.
    #[serde(
        default,
        alias = "format",
        alias = "iq_format",
        skip_serializing_if = "Option::is_none"
    )]
    pub iq_format: Option<IqFormat>,
}

impl SdrSettings {
    pub fn center_freq_or(&self, default: u64) -> u64 {
        self.center_freq.unwrap_or(default)
    }

    pub fn sample_rate_or(&self, default: u32) -> u32 {
        self.sample_rate.unwrap_or(default)
    }

    pub fn gain_or(&self, default: Gain) -> Gain {
        self.gain.clone().unwrap_or(default)
    }

    pub fn bias_tee_or(&self, default: bool) -> bool {
        self.bias_tee.unwrap_or(default)
    }

    pub fn amp_enable_or(&self, default: bool) -> bool {
        self.amp_enable.unwrap_or(default)
    }

    pub fn freq_correction_ppm_or(&self, default: i32) -> i32 {
        self.freq_correction_ppm.unwrap_or(default)
    }

    pub fn iq_format_or(&self, default: IqFormat) -> IqFormat {
        self.iq_format.unwrap_or(default)
    }

    /// Merge user overrides into this settings object. Values present in
    /// `overrides` replace values in `self`; missing override fields leave the
    /// current value unchanged.
    pub fn merge_overrides(&mut self, overrides: &SdrSettings) {
        if overrides.center_freq.is_some() {
            self.center_freq = overrides.center_freq;
        }
        if overrides.sample_rate.is_some() {
            self.sample_rate = overrides.sample_rate;
        }
        if overrides.gain.is_some() {
            self.gain = overrides.gain.clone();
        }
        if overrides.bias_tee.is_some() {
            self.bias_tee = overrides.bias_tee;
        }
        if overrides.amp_enable.is_some() {
            self.amp_enable = overrides.amp_enable;
        }
        if overrides.freq_correction_ppm.is_some() {
            self.freq_correction_ppm = overrides.freq_correction_ppm;
        }
        if overrides.iq_format.is_some() {
            self.iq_format = overrides.iq_format;
        }
    }
}

#[cfg(feature = "rtlsdr")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RtlSdrPath {
    #[serde(flatten)]
    pub config: RtlSdrDeviceConfig,
}

#[cfg(feature = "rtlsdr")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RtlSdrDeviceConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serial: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub product: Option<String>,
}

#[cfg(feature = "airspy")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct AirspyPath {
    #[serde(flatten)]
    pub config: AirspyDeviceConfig,
}

#[cfg(feature = "airspy")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirspyDeviceConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serial: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lna_gain: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mixer_gain: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vga_gain: Option<u8>,
}

#[cfg(feature = "hackrf")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct HackrfPath {
    #[serde(flatten)]
    pub config: HackrfDeviceConfig,
}

#[cfg(feature = "hackrf")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HackrfDeviceConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lna_gain: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vga_gain: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub amp_enable: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub freq_offset_hz: Option<i32>,
}

#[cfg(feature = "soapy")]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SoapyPath {
    pub soapy: String,
}

/// Structured file path for TOML source configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct FilePath {
    pub file: String,
}

/// Parse an Airspy serial number from decimal or `0x` hexadecimal notation.
pub fn parse_airspy_serial(serial: &str) -> Result<u64, String> {
    if let Some(hex) = serial
        .strip_prefix("0x")
        .or_else(|| serial.strip_prefix("0X"))
    {
        u64::from_str_radix(hex, 16).map_err(|e| format!("invalid Airspy serial {serial}: {e}"))
    } else {
        serial
            .parse::<u64>()
            .map_err(|e| format!("invalid Airspy serial {serial}: {e}"))
    }
}

#[cfg(feature = "airspy")]
pub fn validate_airspy_gain_conflict(
    gain: &Option<Gain>,
    config: &AirspyDeviceConfig,
) -> Result<(), &'static str> {
    if gain.is_some()
        && (config.lna_gain.is_some() || config.mixer_gain.is_some() || config.vga_gain.is_some())
    {
        Err(
            "Cannot specify both `gain` (source level) and per-element gains (`lna_gain`, `mixer_gain`, `vga_gain`) inside `airspy = { ... }`. Use either `gain = ...` or per-element gains, not both.",
        )
    } else {
        Ok(())
    }
}

#[cfg(feature = "hackrf")]
pub fn validate_hackrf_gain_conflict(
    gain: &Option<Gain>,
    config: &HackrfDeviceConfig,
) -> Result<(), &'static str> {
    if gain.is_some() && (config.lna_gain.is_some() || config.vga_gain.is_some()) {
        Err(
            "Cannot specify both `gain` (source level) and per-element gains (`lna_gain`, `vga_gain`) inside `hackrf = { ... }`. Use either `gain = ...` or per-element gains, not both.",
        )
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_gain_is_distinct_from_auto_gain() {
        let missing = SdrSettings::default();
        assert_eq!(missing.gain, None);
        assert_eq!(missing.gain_or(Gain::Manual(49.6)), Gain::Manual(49.6));

        let explicit_auto = SdrSettings {
            gain: Some(Gain::Auto),
            ..SdrSettings::default()
        };
        assert_eq!(explicit_auto.gain_or(Gain::Manual(49.6)), Gain::Auto);
    }

    #[test]
    fn recognizes_device_uri_schemes() {
        assert!(is_device_uri("rtlsdr://0"));
        assert!(is_device_uri("airspy://0"));
        assert!(is_device_uri("hackrf://0"));
        assert!(is_device_uri("soapy://driver=rtlsdr"));
        assert!(is_device_uri("pluto://ip:192.168.2.1"));
        assert!(!is_device_uri("capture.cu8"));
    }

    #[test]
    fn parses_airspy_serial_forms() {
        assert_eq!(parse_airspy_serial("1234").unwrap(), 1234);
        assert_eq!(parse_airspy_serial("0x4d2").unwrap(), 1234);
    }
}
