//! Forward Error Correction for DAB.
//!
//! Implements the three FEC stages applied (in reverse) to recover data bits
//! from noisy OFDM symbols:
//!
//! 1. **[`eep`]** — Equal Error Protection depuncturing (rate 1/4 mother code)
//! 2. **[`viterbi`]** — Constraint-length 7 Viterbi convolutional decoder
//! 3. **[`energy_dispersal`]** — PRBS de-scrambling

pub mod eep;
pub mod energy_dispersal;
pub mod viterbi;
