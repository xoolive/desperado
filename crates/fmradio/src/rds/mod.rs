//! RDS (Radio Data System) decoder
//!
//! This module provides a complete RDS decoder implementation including:
//! - DSP chain for signal processing (PLL, AGC, symbol sync)
//! - Protocol parser for RDS groups and station information
//! - Constants and lookup tables for the RDS standard

pub mod constants;
pub mod dsp;
pub mod parser;

#[cfg(test)]
mod tests;

// Re-export public API
pub use dsp::{RdsDecoder, RdsResamplerCustom, StereoDecoderPLL};
pub use parser::{
    ClockTimeInfo, DIFlags, EONInfo, ODAInfo, ProgramItemInfo, RdsGroupJson, RdsParser,
    StationInfo, rds_data_from_word, rds_offset_for_syndrome, rds_syndrome,
};
