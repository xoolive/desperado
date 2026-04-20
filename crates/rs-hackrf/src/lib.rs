//! Pure-Rust driver for HackRF One Software Defined Radio.
//!
//! # Overview
//!
//! `rs-hackrf` provides a zero-C dependency Rust interface to HackRF SDR devices.
//! It supports device discovery, configuration, and high-throughput RX streaming.
//!
//! # Sample Format
//!
//! HackRF outputs interleaved 8-bit signed I/Q samples directly over USB.
//! No DSP conversion is needed (unlike Airspy which outputs real samples).
//! Each sample pair is `[I, Q]` where both I and Q are `i8` values (-128 to 127).
//!
//! # Example
//!
//! ```no_run
//! use rs_hackrf::HackRf;
//!
//! let device = HackRf::open_first()?;
//! let version = device.version()?;
//! println!("Firmware: {}", version);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod error;
pub mod transport;

pub use error::{Error, HackRfErrorCode, Result};
pub use transport::{
    AsyncReadControlHandle, AsyncReadHandle, HackRf, RECOMMENDED_BUFFER_SIZE, TRANSFER_BUFFER_SIZE,
};

/// HackRF USB Vendor ID (OpenMoko Inc, shared VID).
pub const HACKRF_VID: u16 = 0x1d50;

/// HackRF One USB Product ID.
pub const HACKRF_ONE_PID: u16 = 0x6089;

/// HackRF Jawbreaker USB Product ID.
pub const HACKRF_JAWBREAKER_PID: u16 = 0x604b;

/// rad1o USB Product ID.
pub const RAD1O_PID: u16 = 0xcc15;
