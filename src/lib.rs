#![doc = include_str!("../readme.md")]
//!
//! # API Naming Conventions
//!
//! This library follows consistent naming conventions:
//! - **Synchronous types**: `IqSource`, `IqRead`, `RtlSdrReader`, etc.
//! - **Asynchronous types**: `IqAsyncSource`, `IqAsyncRead`, `AsyncRtlSdrReader`, etc.
//! - Async types are prefixed with `Async` or use `IqAsync` naming
//! - All async operations return `Future`s or implement the `Stream` trait

use std::path::PathBuf;
use std::pin::Pin;
use std::task::{Context, Poll};

use futures::Stream;
use num_complex::Complex;

pub mod dsp;
pub mod error;
pub mod iqread;
#[cfg(feature = "pluto")]
pub mod pluto;
#[cfg(feature = "rtlsdr")]
pub mod rtlsdr;
#[cfg(feature = "soapy")]
pub mod soapy;

// Re-export commonly used types
pub use error::{Error, Result};

/// Expand tilde (~) in path to home directory
///
/// This function expands paths starting with `~` to the user's home directory.
/// If the path doesn't start with `~` or the home directory cannot be determined,
/// the original path is returned unchanged.
///
/// # Examples
///
/// ```
/// use std::path::PathBuf;
/// use desperado::expanduser;
///
/// let path = PathBuf::from("~/Documents/data.iq");
/// let expanded = expanduser(path);
/// // On Unix: /home/username/Documents/data.iq
/// ```
pub fn expanduser(path: PathBuf) -> PathBuf {
    // Check if the path starts with "~"
    if let Some(stripped) = path.to_str().and_then(|p| p.strip_prefix("~"))
        && let Some(home_dir) = dirs::home_dir()
    {
        // Join the home directory with the rest of the path
        return home_dir.join(stripped.trim_start_matches('/'));
    }
    path
}

/// Parse a numeric value with optional SI suffix (k, M, G)
///
/// Supports both integer and floating-point values with SI multipliers:
/// - `k` or `K`: multiply by 1,000 (kilo)
/// - `M`: multiply by 1,000,000 (mega)
/// - `G`: multiply by 1,000,000,000 (giga)
///
/// # Examples
///
/// ```
/// use desperado::parse_si_value;
///
/// assert_eq!(parse_si_value::<u32>("1090M").unwrap(), 1090000000);
/// assert_eq!(parse_si_value::<u32>("2400k").unwrap(), 2400000);
/// assert_eq!(parse_si_value::<u32>("1090000000").unwrap(), 1090000000);
/// assert_eq!(parse_si_value::<f64>("2.4M").unwrap(), 2400000.0);
/// assert_eq!(parse_si_value::<i64>("1090M").unwrap(), 1090000000);
/// ```
pub fn parse_si_value<T>(s: &str) -> Result<T>
where
    T: std::str::FromStr,
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    let s = s.trim();

    // Check for SI suffix
    let (num_str, multiplier) = if let Some(stripped) = s.strip_suffix('G') {
        (stripped, 1_000_000_000.0)
    } else if let Some(stripped) = s.strip_suffix('M') {
        (stripped, 1_000_000.0)
    } else if let Some(stripped) = s.strip_suffix(['k', 'K']) {
        (stripped, 1_000.0)
    } else {
        // No suffix - try to parse directly
        return s
            .parse()
            .map_err(|e| Error::other(format!("Invalid numeric value '{}': {}", s, e)));
    };

    // Parse as f64 first to handle both integers and floats with suffixes
    let value_f64: f64 = num_str
        .parse()
        .map_err(|e| Error::other(format!("Invalid numeric value '{}': {}", num_str, e)))?;

    // Apply multiplier
    let result = value_f64 * multiplier;

    // Convert to string and parse as target type (handles rounding for integers)
    let result_str = format!("{:.0}", result);
    result_str.parse().map_err(|e| {
        Error::other(format!(
            "Failed to convert '{}' to target type: {}",
            result_str, e
        ))
    })
}

/// Unified gain configuration across all SDR devices
///
/// This enum provides a consistent interface for configuring tuner gain
/// across different SDR hardware (RTL-SDR, SoapySDR, PlutoSDR).
///
/// # Examples
///
/// ```
/// use desperado::Gain;
///
/// // Automatic gain control
/// let auto_gain = Gain::Auto;
///
/// // Manual gain of 49.6 dB
/// let manual_gain = Gain::Manual(49.6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gain {
    /// Automatic gain control (AGC)
    Auto,
    /// Manual gain in dB
    Manual(f64),
}

/// I/Q Data Format
///
/// Represents the different formats for I/Q sample data. Each variant corresponds
/// to a specific data type and encoding used for representing complex samples.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum IqFormat {
    /// Complex unsigned 8-bit (Cu8)
    ///
    /// Two bytes per sample: I (unsigned 8-bit), Q (unsigned 8-bit)
    /// Values range from 0-255, with 127.5 representing zero
    Cu8,
    /// Complex signed 8-bit (Cs8)
    ///
    /// Two bytes per sample: I (signed 8-bit), Q (signed 8-bit)
    /// Values range from -128 to 127
    Cs8,
    /// Complex signed 16-bit little-endian (Cs16)
    ///
    /// Four bytes per sample: I (signed 16-bit LE), Q (signed 16-bit LE)
    /// Values range from -32768 to 32767
    Cs16,
    /// Complex 32-bit float little-endian (Cf32)
    ///
    /// Eight bytes per sample: I (32-bit float LE), Q (32-bit float LE)
    /// Normalized float values
    Cf32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DeviceConfig {
    #[cfg(feature = "pluto")]
    Pluto(pluto::PlutoConfig),
    #[cfg(feature = "rtlsdr")]
    RtlSdr(rtlsdr::RtlSdrConfig),
    #[cfg(feature = "soapy")]
    Soapy(soapy::SoapyConfig),
}

impl std::str::FromStr for DeviceConfig {
    type Err = Error;

    /// Parse device configuration from URL-style string
    ///
    /// Supported formats:
    /// - `rtlsdr://[device_index]?freq=<hz>&rate=<hz>&gain=<db|auto>&bias_tee=<bool>`
    /// - `soapy://<driver>?freq=<hz>&rate=<hz>&gain=<db|auto>&bias_tee=<bool>`
    /// - `pluto://<uri>?freq=<hz>&gain=<db|auto>`
    ///
    /// For RTL-SDR, if `device_index` is omitted, it defaults to 0 (first device).
    ///
    /// For PlutoSDR, the URI can be:
    /// - An IP address: `pluto://192.168.2.1`
    /// - With ip: prefix: `pluto://ip:192.168.2.1`
    /// - USB device: `pluto://usb:` or `pluto:///usb:1.18.5` (use triple slash for URIs with colons/dots)
    ///
    /// Frequency and sample rate values support SI suffixes (k/K, M, G):
    /// - `1090M` = 1,090,000,000 Hz
    /// - `2.4M` = 2,400,000 Hz
    /// - `1090000000` (raw Hz values also supported)
    ///
    /// # Examples
    ///
    /// ```
    /// # #[cfg(feature = "rtlsdr")]
    /// # {
    /// use desperado::DeviceConfig;
    /// use std::str::FromStr;
    ///
    /// // RTL-SDR with SI suffixes (most convenient)
    /// let config = DeviceConfig::from_str("rtlsdr://0?freq=1090M&rate=2.4M&gain=auto").unwrap();
    ///
    /// // RTL-SDR with raw Hz values
    /// let config = DeviceConfig::from_str("rtlsdr://0?freq=1090000000&rate=2400000&gain=auto").unwrap();
    ///
    /// // RTL-SDR first available device (implicit device 0)
    /// let config = DeviceConfig::from_str("rtlsdr://?freq=1090M&rate=2.4M&gain=49.6").unwrap();
    /// # }
    /// ```
    ///
    /// ```
    /// # #[cfg(feature = "pluto")]
    /// # {
    /// use desperado::DeviceConfig;
    /// use std::str::FromStr;
    ///
    /// // PlutoSDR with IP address
    /// let config = DeviceConfig::from_str("pluto://192.168.2.1?freq=1090M&rate=2.4M&gain=40").unwrap();
    ///
    /// // PlutoSDR with explicit ip: prefix
    /// let config = DeviceConfig::from_str("pluto://ip:192.168.2.1?freq=1090M&rate=2.4M&gain=40").unwrap();
    ///
    /// // PlutoSDR via USB (use triple slash for USB URIs with version numbers)
    /// let config = DeviceConfig::from_str("pluto:///usb:1.18.5?freq=1090M&rate=2.4M&gain=40").unwrap();
    /// # }
    /// ```
    fn from_str(s: &str) -> Result<Self> {
        // Parse URL scheme
        let parts: Vec<&str> = s.splitn(2, "://").collect();
        if parts.len() != 2 {
            return Err(Error::other(
                "Invalid device URL: missing '://' separator".to_string(),
            ));
        }

        let scheme = parts[0];
        #[allow(unused_variables)]
        let rest = parts[1];

        match scheme {
            #[cfg(feature = "rtlsdr")]
            "rtlsdr" => {
                // Parse: rtlsdr://[device_index]?freq=...&rate=...&gain=...&bias_tee=...
                let (device_part, query) = if let Some(q_pos) = rest.find('?') {
                    (&rest[..q_pos], &rest[q_pos + 1..])
                } else {
                    (rest, "")
                };

                let device_index = if device_part.is_empty() {
                    0
                } else {
                    device_part.parse::<usize>().map_err(|_| {
                        Error::other(format!("Invalid device index: {}", device_part))
                    })?
                };

                // Parse query parameters
                let mut center_freq: Option<u32> = None;
                let mut sample_rate: Option<u32> = None;
                let mut gain = Gain::Auto;
                let mut bias_tee = false;

                for param in query.split('&') {
                    if param.is_empty() {
                        continue;
                    }
                    let kv: Vec<&str> = param.splitn(2, '=').collect();
                    if kv.len() != 2 {
                        continue;
                    }
                    match kv[0] {
                        "freq" | "frequency" => {
                            center_freq = Some(parse_si_value(kv[1])?);
                        }
                        "rate" | "sample_rate" => {
                            sample_rate = Some(parse_si_value(kv[1])?);
                        }
                        "gain" => {
                            if kv[1].to_lowercase() == "auto" {
                                gain = Gain::Auto;
                            } else {
                                let gain_db: f64 = kv[1].parse().map_err(|_| {
                                    Error::other(format!("Invalid gain: {}", kv[1]))
                                })?;
                                gain = Gain::Manual(gain_db);
                            }
                        }
                        "bias_tee" | "bias-tee" => {
                            bias_tee = kv[1].to_lowercase() == "true" || kv[1] == "1";
                        }
                        _ => {} // Ignore unknown parameters
                    }
                }

                let center_freq = center_freq
                    .ok_or_else(|| Error::other("Missing freq parameter".to_string()))?;
                let sample_rate = sample_rate
                    .ok_or_else(|| Error::other("Missing rate parameter".to_string()))?;

                Ok(DeviceConfig::RtlSdr(rtlsdr::RtlSdrConfig {
                    device: rtlsdr::DeviceSelector::Index(device_index),
                    center_freq,
                    sample_rate,
                    gain,
                    bias_tee,
                }))
            }
            #[cfg(feature = "soapy")]
            "soapy" => {
                // Parse: soapy://<args>?freq=...&rate=...&gain=...
                let (args_part, query) = if let Some(q_pos) = rest.find('?') {
                    (&rest[..q_pos], &rest[q_pos + 1..])
                } else {
                    (rest, "")
                };

                let args = args_part.to_string();

                // Parse query parameters
                let mut center_freq: Option<f64> = None;
                let mut sample_rate: Option<f64> = None;
                let mut gain = Gain::Auto;
                let channel = 0;
                let gain_element = "TUNER".to_string();
                let mut bias_tee = false;

                for param in query.split('&') {
                    if param.is_empty() {
                        continue;
                    }
                    let kv: Vec<&str> = param.splitn(2, '=').collect();
                    if kv.len() != 2 {
                        continue;
                    }
                    match kv[0] {
                        "freq" | "frequency" => {
                            center_freq = Some(parse_si_value(kv[1])?);
                        }
                        "rate" | "sample_rate" => {
                            sample_rate = Some(parse_si_value(kv[1])?);
                        }
                        "gain" => {
                            if kv[1].to_lowercase() == "auto" {
                                gain = Gain::Auto;
                            } else {
                                let gain_db: f64 = kv[1].parse().map_err(|_| {
                                    Error::other(format!("Invalid gain: {}", kv[1]))
                                })?;
                                gain = Gain::Manual(gain_db);
                            }
                        }
                        "bias_tee" | "bias-tee" => {
                            bias_tee = kv[1].to_lowercase() == "true" || kv[1] == "1";
                        }
                        _ => {} // Ignore unknown parameters
                    }
                }

                let center_freq = center_freq
                    .ok_or_else(|| Error::other("Missing freq parameter".to_string()))?;
                let sample_rate = sample_rate
                    .ok_or_else(|| Error::other("Missing rate parameter".to_string()))?;

                Ok(DeviceConfig::Soapy(soapy::SoapyConfig {
                    args,
                    center_freq,
                    sample_rate,
                    channel,
                    gain,
                    gain_element,
                    bias_tee,
                }))
            }
            #[cfg(feature = "pluto")]
            "pluto" => {
                // Parse: pluto://<uri>?freq=...&rate=...&gain=...
                // URI can be: ip:192.168.2.1, usb:, usb:1, usb:1.18.5, or plain IP like 192.168.2.1
                // Also supports path format: pluto:///usb:1.18.5 (triple slash for URIs with colons)

                // Strip leading slash if present (from pluto:///uri format)
                let rest_trimmed = rest.strip_prefix('/').unwrap_or(rest);

                let (uri_part, query) = if let Some(q_pos) = rest_trimmed.find('?') {
                    (&rest_trimmed[..q_pos], &rest_trimmed[q_pos + 1..])
                } else {
                    (rest_trimmed, "")
                };

                let uri = uri_part.to_string();

                // Parse query parameters
                let mut center_freq: Option<i64> = None;
                let mut sample_rate: Option<i64> = None;
                let mut gain = Gain::Manual(40.0); // Default manual gain for Pluto

                for param in query.split('&') {
                    if param.is_empty() {
                        continue;
                    }
                    let kv: Vec<&str> = param.splitn(2, '=').collect();
                    if kv.len() != 2 {
                        continue;
                    }
                    match kv[0] {
                        "freq" | "frequency" => {
                            center_freq = Some(parse_si_value(kv[1])?);
                        }
                        "rate" | "sample_rate" => {
                            sample_rate = Some(parse_si_value(kv[1])?);
                        }
                        "gain" => {
                            if kv[1].to_lowercase() == "auto" {
                                gain = Gain::Auto;
                            } else {
                                let gain_db: f64 = kv[1].parse().map_err(|_| {
                                    Error::other(format!("Invalid gain: {}", kv[1]))
                                })?;
                                gain = Gain::Manual(gain_db);
                            }
                        }
                        _ => {} // Ignore unknown parameters
                    }
                }

                let center_freq = center_freq
                    .ok_or_else(|| Error::other("Missing freq parameter".to_string()))?;
                let sample_rate = sample_rate
                    .ok_or_else(|| Error::other("Missing rate parameter".to_string()))?;

                Ok(DeviceConfig::Pluto(pluto::PlutoConfig {
                    uri,
                    center_freq,
                    sample_rate,
                    gain,
                }))
            }
            _ => Err(Error::other(format!("Unknown device scheme: {}", scheme))),
        }
    }
}

impl std::fmt::Display for IqFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IqFormat::Cu8 => write!(f, "cu8"),
            IqFormat::Cs8 => write!(f, "cs8"),
            IqFormat::Cs16 => write!(f, "cs16"),
            IqFormat::Cf32 => write!(f, "cf32"),
        }
    }
}

impl std::str::FromStr for IqFormat {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "cu8" => Ok(IqFormat::Cu8),
            "cs8" => Ok(IqFormat::Cs8),
            "cs16" => Ok(IqFormat::Cs16),
            "cf32" => Ok(IqFormat::Cf32),
            _ => Err(Error::format(format!("Invalid IQ format: '{}'", s))),
        }
    }
}

/// Synchronous I/Q Data Source (iterable)
///
/// An enum representing different sources of I/Q data that can be read synchronously.
/// Implements [`Iterator`] to yield chunks of I/Q samples as [`Complex<f32>`] vectors.
pub enum IqSource {
    /// File-based IQ source
    IqFile(iqread::IqRead<std::io::BufReader<std::fs::File>>),
    /// Stdin-based IQ source
    IqStdin(iqread::IqRead<std::io::BufReader<std::io::Stdin>>),
    /// TCP-based IQ source
    IqTcp(iqread::IqRead<std::io::BufReader<std::net::TcpStream>>),
    /// Adalm Pluto-based IQ source (requires "pluto" feature)
    #[cfg(feature = "pluto")]
    PlutoSdr(pluto::PlutoSdrReader),
    /// RTL-SDR-based IQ source (requires "rtlsdr" feature)
    #[cfg(feature = "rtlsdr")]
    RtlSdr(rtlsdr::RtlSdrReader),
    /// SoapySDR-based IQ source (requires "soapy" feature)
    #[cfg(feature = "soapy")]
    SoapySdr(soapy::SoapySdrReader),
}

impl Iterator for IqSource {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            IqSource::IqFile(source) => source.next(),
            IqSource::IqStdin(source) => source.next(),
            IqSource::IqTcp(source) => source.next(),
            #[cfg(feature = "pluto")]
            IqSource::PlutoSdr(source) => source.next(),
            #[cfg(feature = "rtlsdr")]
            IqSource::RtlSdr(source) => source.next(),
            #[cfg(feature = "soapy")]
            IqSource::SoapySdr(source) => source.next(),
        }
    }
}
impl IqSource {
    /// Create a new file-based I/Q source
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::{IqSource, IqFormat};
    ///
    /// let source = IqSource::from_file(
    ///     "samples.iq",
    ///     100_000_000,  // 100 MHz center frequency
    ///     2_048_000,    // 2.048 MS/s sample rate
    ///     16384,        // 16K samples per chunk
    ///     IqFormat::Cu8
    /// )?;
    ///
    /// for chunk in source {
    ///     let samples = chunk?;
    ///     println!("Read {} samples", samples.len());
    /// }
    /// # Ok::<(), desperado::Error>(())
    /// ```
    pub fn from_file<P: AsRef<std::path::Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let source =
            iqread::IqRead::from_file(path, center_freq, sample_rate, chunk_size, iq_format)?;
        Ok(IqSource::IqFile(source))
    }

    /// Create a new stdin-based I/Q source
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::{IqSource, IqFormat};
    ///
    /// // Read from piped input: cat samples.iq | my_program
    /// let source = IqSource::from_stdin(
    ///     100_000_000,  // 100 MHz
    ///     2_048_000,    // 2.048 MS/s
    ///     16384,        // 16K samples per chunk
    ///     IqFormat::Cu8
    /// )?;
    /// # Ok::<(), desperado::Error>(())
    /// ```
    pub fn from_stdin(
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<IqSource> {
        let source = iqread::IqRead::from_stdin(center_freq, sample_rate, chunk_size, iq_format);
        Ok(IqSource::IqStdin(source))
    }

    pub fn from_tcp(
        addr: &str,
        port: u16,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let source =
            iqread::IqRead::from_tcp(addr, port, center_freq, sample_rate, chunk_size, iq_format)?;
        Ok(IqSource::IqTcp(source))
    }

    /// Create a new I/Q source from a device configuration
    pub fn from_device_config(config: DeviceConfig) -> error::Result<Self> {
        match config {
            #[cfg(feature = "pluto")]
            DeviceConfig::Pluto(cfg) => {
                let source = pluto::PlutoSdrReader::new(&cfg)?;
                Ok(IqSource::PlutoSdr(source))
            }
            #[cfg(feature = "rtlsdr")]
            DeviceConfig::RtlSdr(cfg) => {
                let source = rtlsdr::RtlSdrReader::new(&cfg)?;
                Ok(IqSource::RtlSdr(source))
            }
            #[cfg(feature = "soapy")]
            DeviceConfig::Soapy(cfg) => {
                let source = soapy::SoapySdrReader::new(&cfg)?;
                Ok(IqSource::SoapySdr(source))
            }
        }
    }

    #[cfg(feature = "pluto")]
    /// Create a new Adalm Pluto-based I/Q source
    pub fn from_pluto(
        uri: &str,
        center_freq: i64,
        sample_rate: i64,
        gain: f64,
    ) -> error::Result<Self> {
        let config = pluto::PlutoConfig {
            uri: uri.to_string(),
            center_freq,
            sample_rate,
            gain: Gain::Manual(gain),
        };
        let source = pluto::PlutoSdrReader::new(&config)?;
        Ok(IqSource::PlutoSdr(source))
    }

    #[cfg(feature = "rtlsdr")]
    /// Create a new RTL-SDR-based I/Q source
    pub fn from_rtlsdr(
        device_index: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<i32>,
    ) -> error::Result<Self> {
        let config = rtlsdr::RtlSdrConfig {
            device: rtlsdr::DeviceSelector::Index(device_index),
            center_freq,
            sample_rate,
            gain: match gain {
                Some(g) => Gain::Manual((g as f64) / 10.0),
                None => Gain::Auto,
            },
            bias_tee: false,
        };
        let source = rtlsdr::RtlSdrReader::new(&config)?;
        Ok(IqSource::RtlSdr(source))
    }

    #[cfg(feature = "soapy")]
    /// Create a new SoapySDR-based I/Q source
    pub fn from_soapy(
        args: &str,
        channel: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<f64>,
        gain_element: &str,
    ) -> error::Result<Self> {
        let config = soapy::SoapyConfig {
            args: args.to_string(),
            center_freq: center_freq as f64,
            sample_rate: sample_rate as f64,
            channel,
            gain: match gain {
                Some(g) => Gain::Manual(g),
                None => Gain::Auto,
            },
            gain_element: gain_element.to_string(),
            bias_tee: false,
        };
        let source = soapy::SoapySdrReader::new(&config)?;
        Ok(IqSource::SoapySdr(source))
    }
}

/// Asynchronous I/Q Data Source (streamable)
///
/// An enum representing different sources of I/Q data that can be read asynchronously.
/// Implements [`Stream`] to yield chunks of I/Q samples as [`Complex<f32>`] vectors.
///
/// Use this when you need non-blocking I/O operations in an async runtime.
pub enum IqAsyncSource {
    /// File-based IQ source
    IqAsyncFile(iqread::IqAsyncRead<tokio::io::BufReader<tokio::fs::File>>),
    /// Stdin-based IQ source
    IqAsyncStdin(iqread::IqAsyncRead<tokio::io::BufReader<tokio::io::Stdin>>),
    /// TCP-based IQ source
    IqAsyncTcp(iqread::IqAsyncRead<tokio::io::BufReader<tokio::net::TcpStream>>),
    /// Adalm Pluto-based IQ source (requires "pluto" feature)
    #[cfg(feature = "pluto")]
    PlutoSdr(pluto::AsyncPlutoSdrReader),
    /// RTL-SDR-based IQ source (requires "rtlsdr" feature)
    #[cfg(feature = "rtlsdr")]
    RtlSdr(rtlsdr::AsyncRtlSdrReader),
    /// SoapySDR-based IQ source (requires "soapy" feature)
    #[cfg(feature = "soapy")]
    SoapySdr(soapy::AsyncSoapySdrReader),
}

impl IqAsyncSource {
    /// Create a new file-based asynchronous I/Q source
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::{IqAsyncSource, IqFormat};
    /// use futures::StreamExt;
    ///
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), desperado::Error> {
    /// let mut source = IqAsyncSource::from_file(
    ///     "samples.iq",
    ///     100_000_000,  // 100 MHz
    ///     2_048_000,    // 2.048 MS/s
    ///     16384,        // 16K samples per chunk
    ///     IqFormat::Cu8
    /// ).await?;
    ///
    /// while let Some(chunk) = source.next().await {
    ///     let samples = chunk?;
    ///     println!("Read {} samples", samples.len());
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn from_file<P: AsRef<std::path::Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let source =
            iqread::IqAsyncRead::from_file(path, center_freq, sample_rate, chunk_size, iq_format)
                .await?;
        Ok(IqAsyncSource::IqAsyncFile(source))
    }

    /// Create a new stdin-based asynchronous I/Q source
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::{IqAsyncSource, IqFormat};
    ///
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), desperado::Error> {
    /// let source = IqAsyncSource::from_stdin(
    ///     100_000_000,  // 100 MHz
    ///     2_048_000,    // 2.048 MS/s
    ///     16384,
    ///     IqFormat::Cu8
    /// );
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_stdin(
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Self {
        let source =
            iqread::IqAsyncRead::from_stdin(center_freq, sample_rate, chunk_size, iq_format);
        IqAsyncSource::IqAsyncStdin(source)
    }

    /// Create a new TCP-based asynchronous I/Q source
    pub async fn from_tcp(
        addr: &str,
        port: u16,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let source = iqread::IqAsyncRead::from_tcp(
            addr,
            port,
            center_freq,
            sample_rate,
            chunk_size,
            iq_format,
        )
        .await?;
        Ok(IqAsyncSource::IqAsyncTcp(source))
    }

    #[cfg(feature = "pluto")]
    /// Create a new Adalm Pluto-based asynchronous I/Q source
    pub async fn from_pluto(
        uri: &str,
        center_freq: i64,
        sample_rate: i64,
        gain: f64,
    ) -> error::Result<Self> {
        let config = pluto::PlutoConfig {
            uri: uri.to_string(),
            center_freq,
            sample_rate,
            gain: Gain::Manual(gain),
        };
        let source = pluto::AsyncPlutoSdrReader::new(&config).await?;
        Ok(IqAsyncSource::PlutoSdr(source))
    }

    #[cfg(feature = "rtlsdr")]
    /// Create a new RTL-SDR-based asynchronous I/Q source
    pub async fn from_rtlsdr(
        device_index: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<i32>,
    ) -> error::Result<Self> {
        let config = rtlsdr::RtlSdrConfig {
            device: rtlsdr::DeviceSelector::Index(device_index),
            center_freq,
            sample_rate,
            gain: match gain {
                Some(g) => Gain::Manual((g as f64) / 10.0),
                None => Gain::Auto,
            },
            bias_tee: false,
        };
        let async_reader = rtlsdr::AsyncRtlSdrReader::new(&config)?;
        Ok(IqAsyncSource::RtlSdr(async_reader))
    }

    #[cfg(feature = "soapy")]
    /// Create a new SoapySDR-based asynchronous I/Q source
    pub async fn from_soapy(
        args: &str,
        channel: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<f64>,
        gain_element: &str,
    ) -> error::Result<Self> {
        let config = soapy::SoapyConfig {
            args: args.to_string(),
            center_freq: center_freq as f64,
            sample_rate: sample_rate as f64,
            channel,
            gain: match gain {
                Some(g) => Gain::Manual(g),
                None => Gain::Auto,
            },
            gain_element: gain_element.to_string(),
            bias_tee: false,
        };
        let async_reader = soapy::AsyncSoapySdrReader::new(&config)?;
        Ok(IqAsyncSource::SoapySdr(async_reader))
    }

    /// Create a new asynchronous I/Q source from a DeviceConfig
    ///
    /// This is a convenience method that dispatches to the appropriate device-specific
    /// constructor based on the DeviceConfig variant.
    #[cfg(any(feature = "rtlsdr", feature = "pluto", feature = "soapy"))]
    pub async fn from_device_config(config: &DeviceConfig) -> error::Result<Self> {
        match config {
            #[cfg(feature = "rtlsdr")]
            DeviceConfig::RtlSdr(cfg) => {
                let async_reader = rtlsdr::AsyncRtlSdrReader::new(cfg)?;
                Ok(IqAsyncSource::RtlSdr(async_reader))
            }
            #[cfg(feature = "pluto")]
            DeviceConfig::Pluto(cfg) => {
                let async_reader = pluto::AsyncPlutoSdrReader::new(cfg).await?;
                Ok(IqAsyncSource::PlutoSdr(async_reader))
            }
            #[cfg(feature = "soapy")]
            DeviceConfig::Soapy(cfg) => {
                let async_reader = soapy::AsyncSoapySdrReader::new(cfg)?;
                Ok(IqAsyncSource::SoapySdr(async_reader))
            }
        }
    }
}

impl Stream for IqAsyncSource {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        match self.get_mut() {
            IqAsyncSource::IqAsyncFile(source) => Pin::new(source).poll_next(cx),
            IqAsyncSource::IqAsyncStdin(source) => Pin::new(source).poll_next(cx),
            IqAsyncSource::IqAsyncTcp(source) => Pin::new(source).poll_next(cx),
            #[cfg(feature = "pluto")]
            IqAsyncSource::PlutoSdr(source) => Pin::new(source).poll_next(cx),
            #[cfg(feature = "rtlsdr")]
            IqAsyncSource::RtlSdr(source) => Pin::new(source).poll_next(cx),
            #[cfg(feature = "soapy")]
            IqAsyncSource::SoapySdr(source) => Pin::new(source).poll_next(cx),
        }
    }
}

fn convert_bytes_to_complex(format: IqFormat, buffer: &[u8]) -> Vec<Complex<f32>> {
    match format {
        IqFormat::Cu8 => buffer
            .chunks_exact(2)
            .map(|c| Complex::new((c[0] as f32 - 127.5) / 128.0, (c[1] as f32 - 127.5) / 128.0))
            .collect(),
        IqFormat::Cs8 => buffer
            .chunks_exact(2)
            .map(|c| Complex::new((c[0] as i8) as f32 / 128.0, (c[1] as i8) as f32 / 128.0))
            .collect(),
        IqFormat::Cs16 => buffer
            .chunks_exact(4)
            .map(|c| {
                Complex::new(
                    i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                    i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
                )
            })
            .collect(),
        IqFormat::Cf32 => buffer
            .chunks_exact(8)
            .map(|c| {
                Complex::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iqformat_display() {
        assert_eq!(IqFormat::Cu8.to_string(), "cu8");
        assert_eq!(IqFormat::Cs8.to_string(), "cs8");
        assert_eq!(IqFormat::Cs16.to_string(), "cs16");
        assert_eq!(IqFormat::Cf32.to_string(), "cf32");
    }

    #[test]
    fn test_iqformat_fromstr() {
        assert_eq!("cu8".parse::<IqFormat>().unwrap(), IqFormat::Cu8);
        assert_eq!("cs8".parse::<IqFormat>().unwrap(), IqFormat::Cs8);
        assert_eq!("cs16".parse::<IqFormat>().unwrap(), IqFormat::Cs16);
        assert_eq!("cf32".parse::<IqFormat>().unwrap(), IqFormat::Cf32);
    }

    #[test]
    fn test_iqformat_fromstr_case_insensitive() {
        assert_eq!("CU8".parse::<IqFormat>().unwrap(), IqFormat::Cu8);
        assert_eq!("Cs8".parse::<IqFormat>().unwrap(), IqFormat::Cs8);
        assert_eq!("CS16".parse::<IqFormat>().unwrap(), IqFormat::Cs16);
        assert_eq!("CF32".parse::<IqFormat>().unwrap(), IqFormat::Cf32);
    }

    #[test]
    fn test_iqformat_fromstr_invalid() {
        assert!("invalid".parse::<IqFormat>().is_err());
        assert!("cu16".parse::<IqFormat>().is_err());
        assert!("".parse::<IqFormat>().is_err());
    }

    #[test]
    fn test_iqformat_roundtrip() {
        let formats = [IqFormat::Cu8, IqFormat::Cs8, IqFormat::Cs16, IqFormat::Cf32];
        for format in formats {
            let s = format.to_string();
            let parsed: IqFormat = s.parse().unwrap();
            assert_eq!(parsed, format);
        }
    }

    #[test]
    #[cfg(feature = "pluto")]
    fn test_pluto_uri_parsing() {
        use std::str::FromStr;

        // Test IP address
        let config = DeviceConfig::from_str("pluto://192.168.2.1?freq=1090M&rate=2.4M").unwrap();
        if let DeviceConfig::Pluto(pluto_config) = config {
            assert_eq!(pluto_config.uri, "192.168.2.1");
        } else {
            panic!("Expected PlutoSDR config");
        }

        // Test explicit ip: prefix
        let config = DeviceConfig::from_str("pluto://ip:192.168.2.1?freq=1090M&rate=2.4M").unwrap();
        if let DeviceConfig::Pluto(pluto_config) = config {
            assert_eq!(pluto_config.uri, "ip:192.168.2.1");
        } else {
            panic!("Expected PlutoSDR config");
        }

        // Test USB with triple slash (for URIs containing colons)
        let config = DeviceConfig::from_str("pluto:///usb:1.18.5?freq=1090M&rate=2.4M").unwrap();
        if let DeviceConfig::Pluto(pluto_config) = config {
            assert_eq!(pluto_config.uri, "usb:1.18.5");
        } else {
            panic!("Expected PlutoSDR config");
        }

        // Test simple USB
        let config = DeviceConfig::from_str("pluto:///usb:?freq=1090M&rate=2.4M").unwrap();
        if let DeviceConfig::Pluto(pluto_config) = config {
            assert_eq!(pluto_config.uri, "usb:");
        } else {
            panic!("Expected PlutoSDR config");
        }
    }
}
