//! Error types for rs-spy operations.

use thiserror::Error;

/// Result type for rs-spy operations.
pub type Result<T> = std::result::Result<T, Error>;

/// Error codes matching libairspy enum values (for cross-compatibility).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum AirspyErrorCode {
    /// Operation successful.
    Success = 0,
    /// True (for boolean returns).
    True = 1,
    /// Parameter invalid.
    InvalidParam = -2,
    /// No Airspy device found.
    NotFound = -5,
    /// Device is busy.
    Busy = -6,
    /// Out of memory.
    NoMem = -11,
    /// Operation not supported.
    Unsupported = -12,
    /// LibUSB error.
    Libusb = -1000,
    /// Thread error.
    Thread = -1001,
    /// Streaming thread error.
    StreamingThreadErr = -1002,
    /// Streaming stopped.
    StreamingStopped = -1003,
    /// Other error.
    Other = -9999,
}

impl AirspyErrorCode {
    /// Get a human-readable name for the error code.
    pub fn name(self) -> &'static str {
        match self {
            AirspyErrorCode::Success => "AIRSPY_SUCCESS",
            AirspyErrorCode::True => "AIRSPY_TRUE",
            AirspyErrorCode::InvalidParam => "AIRSPY_ERROR_INVALID_PARAM",
            AirspyErrorCode::NotFound => "AIRSPY_ERROR_NOT_FOUND",
            AirspyErrorCode::Busy => "AIRSPY_ERROR_BUSY",
            AirspyErrorCode::NoMem => "AIRSPY_ERROR_NO_MEM",
            AirspyErrorCode::Unsupported => "AIRSPY_ERROR_UNSUPPORTED",
            AirspyErrorCode::Libusb => "AIRSPY_ERROR_LIBUSB",
            AirspyErrorCode::Thread => "AIRSPY_ERROR_THREAD",
            AirspyErrorCode::StreamingThreadErr => "AIRSPY_ERROR_STREAMING_THREAD_ERR",
            AirspyErrorCode::StreamingStopped => "AIRSPY_ERROR_STREAMING_STOPPED",
            AirspyErrorCode::Other => "AIRSPY_ERROR_OTHER",
        }
    }
}

/// Errors that can occur during Airspy operations.
#[derive(Debug, Error)]
pub enum Error {
    /// Failed to open USB device.
    #[error("failed to open USB device: {0}")]
    OpenFailed(#[source] nusb::Error),

    /// Failed to claim USB interface.
    #[error("failed to claim USB interface: {0}")]
    ClaimFailed(#[source] nusb::Error),

    /// USB control transfer failed.
    #[error("control transfer failed: {0}")]
    ControlTransfer(#[source] nusb::transfer::TransferError),

    /// USB bulk transfer failed.
    #[error("bulk transfer failed: {0}")]
    BulkTransfer(#[source] nusb::transfer::TransferError),

    /// No Airspy device found.
    #[error("No Airspy device found")]
    DeviceNotFound,

    /// Device configuration failed.
    #[error("Configuration failed: {0}")]
    ConfigFailed(String),

    /// Control transfer failed (with context message).
    #[error("Control transfer failed: {0}")]
    ControlTransferFailed(String),

    /// Invalid response from device.
    #[error("Invalid device response: {0}")]
    InvalidResponse(String),

    /// Streaming/bulk transfer error.
    #[error("Streaming error: {0}")]
    StreamingError(String),

    /// Timeout waiting for device response.
    #[error("Device timeout")]
    Timeout,
}
