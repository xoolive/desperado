//! Device URI parsing and source building.
//!
//! This module handles CLI-level concerns of parsing device URIs
//! (rtlsdr://, airspy://, soapy://) and building IQ sources from them.

#[cfg(feature = "airspy")]
use desperado::Gain;
use desperado::{DeviceConfig, IqSource as BaseIqSource};
use std::io;
use std::str::FromStr;

/// Check if input string is a device URI.
pub fn is_device_uri(input: &str) -> bool {
    input.starts_with("rtlsdr://")
        || input.starts_with("soapy://")
        || input.starts_with("airspy://")
}

/// Build an IQ source from a device URI.
pub fn build_device_source(
    uri: &str,
    center_freq_hz: u32,
    sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    if uri.starts_with("airspy://") {
        return build_airspy_source(uri, center_freq_hz, sample_rate);
    }

    let configured_uri = ensure_tuning_query(uri, center_freq_hz, sample_rate);
    let config =
        DeviceConfig::from_str(&configured_uri).map_err(|e| io::Error::other(e.to_string()))?;
    BaseIqSource::from_device_config(config).map_err(|e| io::Error::other(e.to_string()))
}

/// Add missing frequency and sample rate to device URI query string.
fn ensure_tuning_query(uri: &str, center_freq_hz: u32, sample_rate: u32) -> String {
    let has_query = uri.contains('?');
    let has_freq = uri.contains("freq=") || uri.contains("frequency=");
    let has_rate = uri.contains("rate=") || uri.contains("sample_rate=");

    let mut out = uri.to_string();
    if !has_query {
        out.push('?');
    }
    if !has_freq {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("freq={center_freq_hz}"));
    }
    if !has_rate {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("rate={sample_rate}"));
    }
    out
}

/// Build Airspy device source from URI.
#[cfg(feature = "airspy")]
fn build_airspy_source(
    uri: &str,
    center_freq_hz: u32,
    sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    let rest = &uri["airspy://".len()..];
    let (device_part, query) = if let Some(pos) = rest.find('?') {
        (&rest[..pos], &rest[pos + 1..])
    } else {
        (rest, "")
    };

    let device_index = if device_part.is_empty() {
        0
    } else {
        device_part
            .parse::<usize>()
            .map_err(|_| io::Error::other(format!("Invalid airspy device index: {device_part}")))?
    };

    let mut gain = Gain::Auto;

    for param in query.split('&') {
        if param.is_empty() {
            continue;
        }
        let kv: Vec<&str> = param.splitn(2, '=').collect();
        if kv.len() != 2 {
            continue;
        }
        if kv[0] == "gain" {
            gain = if kv[1].eq_ignore_ascii_case("auto") {
                Gain::Auto
            } else {
                Gain::Manual(
                    kv[1]
                        .parse::<f64>()
                        .map_err(|_| io::Error::other(format!("Invalid airspy gain: {}", kv[1])))?,
                )
            };
        }
    }

    let config =
        desperado::airspy::AirspyConfig::new(device_index, center_freq_hz, sample_rate, gain);
    let source = desperado::airspy::AirspySdrReader::new(&config)
        .map_err(|e| io::Error::other(e.to_string()))?;

    Ok(BaseIqSource::Airspy(source))
}

#[cfg(not(feature = "airspy"))]
fn build_airspy_source(
    _uri: &str,
    _center_freq_hz: u32,
    _sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    Err(io::Error::other(
        "airspy:// is not enabled. Rebuild with --features airspy",
    ))
}
