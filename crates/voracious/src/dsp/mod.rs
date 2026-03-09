//! DSP processing for VOR signals

pub mod filter;
pub mod iir;
pub mod vor;

pub use vor::{calculate_radial, VorDemodulator, VOR_SAMPLE_RATE_1_8M};
