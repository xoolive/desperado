//! VOR (VHF Omnidirectional Range) signal decoder library
//!
//! This library provides DSP processing and decoding for VOR navigation signals.

pub mod decode;
pub mod dsp;
pub mod sources;

pub use decode::VorRadial;
pub use dsp::VorDemodulator;
