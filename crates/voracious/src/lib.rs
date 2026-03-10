//! VOR (VHF Omnidirectional Range) signal decoder library
//!
//! This library provides DSP processing and decoding for VOR navigation signals.

pub mod decode;
pub mod dsp;
pub mod metrics;
pub mod source;

pub use decode::VorRadial;
pub use desperado::IqFormat;
pub use dsp::VorDemodulator;
pub use source::IqSource;
