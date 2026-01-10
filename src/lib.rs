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

pub enum DeviceConfig {
    #[cfg(feature = "pluto")]
    Pluto(pluto::PlutoConfig),
    #[cfg(feature = "rtlsdr")]
    RtlSdr(rtlsdr::RtlSdrConfig),
    #[cfg(feature = "soapy")]
    Soapy(soapy::SoapyConfig),
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
            device_index,
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
            device_index,
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
        };
        let async_reader = soapy::AsyncSoapySdrReader::new(&config)?;
        Ok(IqAsyncSource::SoapySdr(async_reader))
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
}
