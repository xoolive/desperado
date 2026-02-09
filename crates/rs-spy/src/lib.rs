//! Pure-Rust driver for Airspy R2, Mini, and HF+ Software Defined Radios.
//!
//! # Overview
//!
//! `rs-spy` provides a zero-C dependency Rust interface to Airspy SDR devices.
//! It supports device discovery, configuration, and high-throughput streaming.
//!
//! # Example
//!
//! ```no_run
//! use rs_spy::Airspy;
//!
//! let device = Airspy::open_first()?;
//! let version = device.version()?;
//! println!("Firmware: {}", version);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod error;
pub mod transport;

pub use error::{AirspyErrorCode, Error, Result};
pub use transport::Airspy;

// Airspy device identifiers (USB VID/PID)
pub const AIRSPY_VID: u16 = 0x1d50;
pub const AIRSPY_PID: u16 = 0x60a1;
