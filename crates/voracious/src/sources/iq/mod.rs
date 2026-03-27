//! I/Q source handling for navigation signals
//!
//! This module provides unified input handling for both file and live device sources,
//! with integrated VOR and ILS Localizer demodulation and signal quality analysis.

pub mod ils_loc;
pub mod vor;

pub use ils_loc::IlsLocalizerSource;
pub use vor::VorSource;
