/// Error types for the rs-rtl crate.
///
/// All errors in this crate flow through the [`Error`] enum, which covers
/// USB communication failures, device identification issues, tuner problems,
/// and streaming errors. A convenience [`Result<T>`] alias is provided.
use thiserror::Error;

/// Convenience result type alias for rs-rtl operations.
pub type Result<T> = std::result::Result<T, Error>;

/// All possible errors from RTL-SDR operations.
#[derive(Debug, Error)]
pub enum Error {
    /// USB device not found during enumeration.
    #[error("RTL-SDR device not found")]
    DeviceNotFound,

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

    /// No supported tuner detected on the I2C bus.
    #[error("no supported tuner found (checked R820T at 0x34, R828D at 0x74)")]
    TunerNotFound,

    /// PLL failed to lock at the requested frequency.
    #[error("PLL failed to lock at {freq_hz} Hz")]
    PllLockFailed {
        /// The frequency (Hz) that was requested when the PLL failed to lock.
        freq_hz: u64,
    },

    /// Requested sample rate is outside the valid range.
    #[error("invalid sample rate {rate} Hz (valid: 225001-300000 or 900001-3200000)")]
    InvalidSampleRate {
        /// The requested sample rate that was out of range.
        rate: u32,
    },

    /// The streaming session is already active.
    #[error("streaming already active")]
    AlreadyStreaming,

    /// The streaming session has been stopped or never started.
    #[error("streaming not active")]
    NotStreaming,

    /// A timeout occurred waiting for data.
    #[error("timeout waiting for USB data")]
    Timeout,

    /// Invalid parameter supplied by the caller.
    #[error("invalid parameter: {0}")]
    InvalidParam(String),
}
