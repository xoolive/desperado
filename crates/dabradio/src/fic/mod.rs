//! Fast Information Channel (FIC) decoder.
//!
//! The FIC occupies the first 3 OFDM data symbols in each DAB Mode I frame
//! and carries the ensemble configuration: service list, subchannel layout,
//! programme types, and service labels.
//!
//! - **[`handler`]** — orchestrates depuncturing → Viterbi → FIB parsing per frame
//! - **[`fib`]** — parses Fast Information Blocks (FIGs) into structured metadata

pub mod fib;
pub mod handler;
