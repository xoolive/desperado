//! VOR (VHF Omnidirectional Range) and ILS (Instrument Landing System) signal decoder library
//!
//! This library provides I/Q streaming, signal processing, and decoding for VOR navigation signals
//! and ILS localizer signals.

pub mod audio;
pub mod decoders;
pub mod dsp;
pub mod error;
pub mod filter_config;
pub mod metrics;
pub mod sources;

pub use error::{Error, Result};

pub use decoders::VorDemodulator;
pub use decoders::VorRadial;
pub use desperado::IqFormat;
pub use sources::iq::{IlsSource, VorSource};
pub use sources::wav::{WavIlsSource, WavVorSource};
