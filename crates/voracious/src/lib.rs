//! VOR (VHF Omnidirectional Range) signal decoder library
//!
//! This library provides I/Q streaming, signal processing, and decoding for VOR navigation signals.

pub mod decoders;
pub mod metrics;
pub mod source;

pub use decoders::VorDemodulator;
pub use decoders::VorRadial;
pub use desperado::IqFormat;
pub use source::IqSource;
