//! Radio beacon decoders
//!
//! This module contains implementations for decoding various radio navigation signals.
//! Each decoder is self-contained with its own demodulation, signal processing, and output logic.
//!
//! ## Currently Implemented
//! - [`vor`]: VOR (VHF Omnidirectional Range) decoder
//! - [`ils`]: ILS (Instrument Landing System) localizer decoder
//! - [`morse`]: Generic Morse code parsing (reusable for VOR, NDB, ILS, DME)
//!
//! ## Future Decoders
//! - DME (Distance Measuring Equipment) decoder

pub mod ils;
pub mod morse;
pub mod vor;

// Re-export public types and functions for convenient access
pub use ils::{
    ILS_AUDIO_RATE, ILS_SAMPLE_RATE_1_8M, IlsDemodulator, IlsFrame, IlsMorseCandidate,
    IlsMorseDebugInfo, IlsSide, compute_ddm,
};
pub use morse::MorseDecodeAttempt;
pub use vor::{
    MorseCandidate, MorseDebugInfo, SignalQualityMetrics, VOR_SAMPLE_RATE_1_8M, VorDemodulator,
    VorRadial, calculate_radial, calculate_radial_vortrack, decode_morse_ident,
};
