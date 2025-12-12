//! Error handling for the desperado library
//!
//! This module provides a unified error type for all operations in the desperado
//! library, including I/O operations, device errors, and format conversions.

use std::fmt;
use std::io;

/// A specialized Result type for desperado operations
pub type Result<T> = std::result::Result<T, Error>;

/// Error type for desperado operations
#[derive(Debug)]
pub enum Error {
    /// I/O error (file operations, network, etc.)
    Io(io::Error),

    /// Device configuration or initialization error
    Device(String),

    /// Invalid I/Q format or conversion error
    Format(String),

    /// RTL-SDR specific error (requires "rtlsdr" feature)
    #[cfg(feature = "rtlsdr")]
    RtlSdr(rtl_sdr_rs::error::RtlsdrError),

    /// SoapySDR specific error (requires "soapy" feature)
    #[cfg(feature = "soapy")]
    SoapySdr(soapysdr::Error),

    /// Pluto SDR specific error (requires "pluto" feature)
    #[cfg(feature = "pluto")]
    PlutoSdr(String),

    /// Generic error with custom message
    Other(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(err) => write!(f, "I/O error: {}", err),
            Error::Device(msg) => write!(f, "Device error: {}", msg),
            Error::Format(msg) => write!(f, "Format error: {}", msg),
            #[cfg(feature = "rtlsdr")]
            Error::RtlSdr(err) => write!(f, "RTL-SDR error: {}", err),
            #[cfg(feature = "soapy")]
            Error::SoapySdr(err) => write!(f, "SoapySDR error: {}", err),
            #[cfg(feature = "pluto")]
            Error::PlutoSdr(msg) => write!(f, "Pluto SDR error: {}", msg),
            Error::Other(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(err) => Some(err),
            #[cfg(feature = "soapy")]
            Error::SoapySdr(err) => Some(err),
            _ => None,
        }
    }
}

// From conversions for common error types

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Error::Io(err)
    }
}

impl From<String> for Error {
    fn from(msg: String) -> Self {
        Error::Other(msg)
    }
}

impl From<&str> for Error {
    fn from(msg: &str) -> Self {
        Error::Other(msg.to_string())
    }
}

#[cfg(feature = "rtlsdr")]
impl From<rtl_sdr_rs::error::RtlsdrError> for Error {
    fn from(err: rtl_sdr_rs::error::RtlsdrError) -> Self {
        Error::RtlSdr(err)
    }
}

#[cfg(feature = "soapy")]
impl From<soapysdr::Error> for Error {
    fn from(err: soapysdr::Error) -> Self {
        Error::SoapySdr(err)
    }
}

// Helper constructors for common error scenarios

impl Error {
    /// Create a device error with a custom message
    pub fn device<S: Into<String>>(msg: S) -> Self {
        Error::Device(msg.into())
    }

    /// Create a format error with a custom message
    pub fn format<S: Into<String>>(msg: S) -> Self {
        Error::Format(msg.into())
    }

    /// Create a generic error with a custom message
    pub fn other<S: Into<String>>(msg: S) -> Self {
        Error::Other(msg.into())
    }

    #[cfg(feature = "pluto")]
    /// Create a Pluto SDR error with a custom message
    pub fn pluto<S: Into<String>>(msg: S) -> Self {
        Error::PlutoSdr(msg.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::error::Error as StdError;

    #[test]
    fn test_io_error_conversion() {
        let io_err = io::Error::new(io::ErrorKind::NotFound, "file not found");
        let err: Error = io_err.into();
        assert!(matches!(err, Error::Io(_)));
        assert!(err.to_string().contains("I/O error"));
    }

    #[test]
    fn test_string_conversion() {
        let err: Error = "test error".into();
        assert!(matches!(err, Error::Other(_)));
        assert_eq!(err.to_string(), "test error");
    }

    #[test]
    fn test_device_error_constructor() {
        let err = Error::device("initialization failed");
        assert!(matches!(err, Error::Device(_)));
        assert!(err.to_string().contains("Device error"));
    }

    #[test]
    fn test_format_error_constructor() {
        let err = Error::format("invalid format");
        assert!(matches!(err, Error::Format(_)));
        assert!(err.to_string().contains("Format error"));
    }

    #[test]
    fn test_error_display() {
        let err = Error::Device("test device error".to_string());
        assert_eq!(err.to_string(), "Device error: test device error");
    }

    #[test]
    fn test_error_source() {
        let io_err = io::Error::new(io::ErrorKind::NotFound, "file not found");
        let err = Error::Io(io_err);
        assert!(err.source().is_some());
    }
}
