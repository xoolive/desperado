//! VOR (VHF Omnidirectional Range) and ILS (Instrument Landing System) signal decoder library
//!
//! This library provides I/Q streaming, signal processing, and decoding for VOR navigation signals
//! and ILS localizer signals.

pub mod audio;
pub mod decoders;
pub mod sources;

pub use decoders::{
    VorDemodulator, VorRadial,
    error::{Error, Result},
};
pub use desperado::IqFormat;
pub use sources::iq::{IlsSource, VorSource};
pub use sources::wav::{WavIlsSource, WavVorSource};
