//! WAV file source handling.
//!
//! This submodule provides WAV-specific signal processing:
//! - Reading and validating WAV files
//! - Detecting and removing IF carriers
//! - Format validation (mono, 48 kHz, AM-demodulated)
//! - Feeding into shared DSP pipeline
//!
//! # Phase 1 (Current)
//!
//! Wrappers around existing `crate::wav_source` implementations. Provides
//! consistent API within the `sources/` module hierarchy.
//!
//! # Phase 2 (Future)
//!
//! Refactor to use shared DSP functions from `sources/common.rs`, eliminating
//! code duplication between I/Q and WAV processing paths.

pub mod ils_loc;
pub mod vor;

pub use ils_loc::WavIlsLocalizerSource;
pub use vor::WavVorSource;
