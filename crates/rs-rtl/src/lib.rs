//! Pure Rust RTL-SDR driver using nusb.
//!
//! This crate provides a complete RTL-SDR device driver built on [`nusb`] for
//! native async multi-transfer USB bulk I/O. It supports the R820T and R828D
//! tuners (including RTL-SDR Blog V4).
//!
//! # Key Features
//!
//! - **No libusb dependency** — pure Rust USB via nusb
//! - **Multi-transfer streaming** — keeps 15 USB transfers in-flight simultaneously,
//!   eliminating the inter-transfer gap that causes RTL2832U FIFO overflow
//! - **Backpressure-safe** — blocking channel delivery ensures no IQ samples are
//!   silently dropped
//! - **Full R82xx support** — frequency tuning, gain control, bandwidth selection,
//!   bias-T, Blog V4 upconversion
//!
//! # Quick Start
//!
//! ```no_run
//! use rs_rtl::RtlSdr;
//!
//! let mut sdr = RtlSdr::open(0)?;
//! sdr.set_center_freq(100_000_000)?;  // 100 MHz
//! sdr.set_sample_rate(2_048_000)?;    // 2.048 MS/s
//! sdr.set_gain_manual(496)?;          // 49.6 dB
//!
//! let reader = sdr.start_streaming()?;
//! while let Some(data) = reader.recv() {
//!     // data contains interleaved u8 I/Q samples: [I0, Q0, I1, Q1, ...]
//!     println!("received {} bytes", data.len());
//! }
//! # Ok::<(), rs_rtl::Error>(())
//! ```
pub mod device;
pub mod error;
pub mod rtlsdr;
pub mod tuner;

pub use error::{Error, Result};
pub use rtlsdr::{
    AsyncReadControlHandle, AsyncReadHandle, DEF_RTL_XTAL_FREQ, DeviceInfo, NUM_TRANSFERS,
    RECOMMENDED_QUEUE_DEPTH, RtlSdr, TRANSFER_BUF_SIZE, list_devices,
};
pub use tuner::{GAIN_VALUES, R82XX_IF_FREQ, TunerType};

/// RTL-SDR USB Vendor ID (Realtek).
pub const RTL_USB_VID: u16 = 0x0bda;

/// Common RTL-SDR USB Product IDs.
pub const RTL_USB_PIDS: &[u16] = &[0x2832, 0x2838];
