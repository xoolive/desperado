//! I/Q source handling for navigation signals
//!
//! This module provides unified input handling for both file and live device sources,
//! with integrated VOR and ILS demodulation and signal quality analysis.

pub mod ils;
pub mod vor;

pub use ils::IlsSource;
pub use vor::VorSource;
