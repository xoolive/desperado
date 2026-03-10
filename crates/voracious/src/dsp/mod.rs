//! DSP processing for VOR signals

pub mod filter;
pub mod iir;
pub mod vor;

pub use vor::{VOR_SAMPLE_RATE_1_8M, VorDemodulator, calculate_radial};
