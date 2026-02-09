//! Pure-Rust driver for Airspy R2, Mini, and HF+ Software Defined Radios.
//!
//! # Overview
//!
//! `rs-spy` provides a zero-C dependency Rust interface to Airspy SDR devices.
//! It supports device discovery, configuration, and high-throughput streaming.
//!
//! # Sample Format
//!
//! Airspy hardware outputs REAL samples from a single ADC. This crate provides
//! an [`IqConverter`] to convert those real samples to Complex I/Q format using
//! Fs/4 frequency translation (matching libairspy behavior).
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
pub mod iqconverter;
pub mod transport;

pub use error::{AirspyErrorCode, Error, Result};
pub use iqconverter::IqConverter;
pub use transport::{Airspy, RECOMMENDED_BUFFER_SIZE, SAMPLES_PER_BUFFER};

// Airspy device identifiers (USB VID/PID)
pub const AIRSPY_VID: u16 = 0x1d50;
pub const AIRSPY_PID: u16 = 0x60a1;
