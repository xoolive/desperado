//! Radio beacon decoders
//!
//! This module contains implementations for decoding various radio navigation signals.
//! Each decoder is self-contained with its own demodulation, signal processing, and output logic.
//!
//! ## Currently Implemented
//! - [`vor`]: VOR (VHF Omnidirectional Range) decoder
//! - [`ils_loc`]: ILS Localizer (LOC/LLZ) (Instrument Landing System lateral guidance) decoder
//! - [`morse`]: Generic Morse code parsing (reusable for VOR, NDB, ILS, DME)
//!
//! ## Future Decoders
//! - ILS Glide Slope (GS) decoder (vertical guidance)
//! - DME (Distance Measuring Equipment) decoder

pub mod error;
pub mod ils_loc;
pub mod metrics;
pub mod morse;
pub mod vor;

// Re-export public types and functions for convenient access
pub use error::{Error, Result};
pub use ils_loc::{
    ILS_AUDIO_RATE, ILS_MORSE_AUDIO_BPF_HIGH, ILS_MORSE_AUDIO_BPF_LOW, ILS_MORSE_AUDIO_BPF_ORDER,
    ILS_MORSE_BPF_HIGH, ILS_MORSE_BPF_LOW, ILS_MORSE_BPF_ORDER, ILS_SAMPLE_RATE_1_8M, IlsFrame,
    IlsLocalizerDemodulator, IlsMorseCandidate, IlsMorseDebugInfo, IlsSide, compute_ddm,
};
pub use metrics::compute_signal_quality;
pub use morse::MorseDecodeAttempt;
pub use vor::{
    MorseCandidate, MorseDebugInfo, SignalQualityMetrics, VOR_30HZ_LPF_CUTOFF, VOR_30HZ_LPF_ORDER,
    VOR_MORSE_AUDIO_BPF_HIGH, VOR_MORSE_AUDIO_BPF_LOW, VOR_MORSE_AUDIO_BPF_ORDER,
    VOR_MORSE_BPF_HIGH, VOR_MORSE_BPF_LOW, VOR_MORSE_BPF_ORDER, VOR_REF_SUB_BPF_HIGH,
    VOR_REF_SUB_BPF_LOW, VOR_REF_SUB_BPF_ORDER, VOR_SAMPLE_RATE_1_8M, VOR_VAR_SUB_BPF_HIGH,
    VOR_VAR_SUB_BPF_LOW, VOR_VAR_SUB_BPF_ORDER, VorDemodulator, VorProcessor, VorProcessorOutput,
    VorRadial, calculate_radial, calculate_radial_vortrack, decode_morse_ident,
};
