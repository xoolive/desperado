//! Radio beacon decoders
//!
//! This module contains implementations for decoding various radio navigation signals.
//! Each decoder is self-contained with its own demodulation, signal processing, and output logic.
//!
//! ## Currently Implemented
//! - [`vor`]: VOR (VHF Omnidirectional Range) decoder
//! - [`morse`]: Generic Morse code parsing (reusable for VOR, NDB, ILS, DME)
//!
//! ## Future Decoders
//! - ILS (Instrument Landing System) localizer decoder
//! - DME (Distance Measuring Equipment) decoder

pub mod morse;
pub mod vor;

// Re-export public types and functions for convenient access
pub use morse::MorseDecodeAttempt;
pub use vor::{
    MorseCandidate, MorseDebugInfo, SignalQualityMetrics, VOR_SAMPLE_RATE_1_8M, VorDemodulator,
    VorRadial, calculate_radial, calculate_radial_vortrack, decode_morse_ident,
};
