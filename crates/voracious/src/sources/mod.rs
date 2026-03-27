//! Radio navigation source handling for I/Q and WAV inputs.
//!
//! This module provides unified interfaces for processing radio navigation signals
//! from different input sources:
//!
//! - **I/Q Sources** (`iq/`): Raw complex samples from SDRs or files
//!   - Raw I/Q binary files (cu8, cs8, cs16, cf32)
//!   - GQRX recordings (auto-detected sample rate from filename)
//!   - Live device input (RTL-SDR, Airspy, SoapySDR)
//!
//! - **WAV Sources** (`wav/`): Pre-recorded audio (already demodulated)
//!   - AM-demodulated audio files from GQRX AM mode
//!   - User-specified demodulation mode validation
//!   - Clear error messages for unsupported formats
//!
//! ## Architecture
//!
//! ```text
//! I/Q Path (1.8 MSps)                      WAV Path (48 kHz)
//! ├─ Frequency shift                       ├─ Read mono PCM
//! ├─ Baseband LPF                          ├─ Detect IF carrier (optional)
//! ├─ AM demodulation (magnitude)           ├─ Extract subcarriers
//! ├─ Decimation (→ 9 kHz or 48 kHz)        └─ Shared DSP pipeline
//! └─ Shared DSP (common.rs)
//!    ├─ Bandpass filters
//!    ├─ Hilbert transforms
//!    ├─ Envelope/phase extraction
//!    └─ Modulation index calculation
//! ```
//!
//! ## Shared DSP Pipeline
//!
//! All source types feed into a unified DSP pipeline in `common.rs`:
//! - **ILS Processing**: Extract 90/150 Hz envelopes, compute DDM
//! - **VOR Processing**: Extract 30 Hz modulation, compute radial
//! - **Morse Processing**: Extract 1020 Hz envelope, decode ident

pub mod common;
pub mod iq;
pub mod wav;

// Re-export main types for public API
pub use iq::{IlsLocalizerSource, VorSource};
pub use wav::{WavIlsLocalizerSource, WavVorSource};
