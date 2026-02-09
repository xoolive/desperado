//! Radio DSP and FM demodulation library.
//!
//! Provides building blocks for FM (Frequency Modulation) radio reception and decoding:
//! - FM demodulation (phase extraction)
//! - De-emphasis filtering for FM broadcast audio
//! - RDS (Radio Data System) decoding for station name and radiotext
//! - Adaptive audio resampling for real-time streaming

pub mod fm;
pub mod rds;
pub mod resampler;

// Re-export main types for convenience
pub use fm::{DeemphasisFilter, PhaseExtractor};
pub use rds::RdsParser;
pub use resampler::AdaptiveResampler;
