//! OFDM front-end for DAB Mode I.
//!
//! Converts a continuous stream of I/Q samples into frequency-domain OFDM
//! symbols ready for FIC and MSC decoding.
//!
//! - **[`processor`]** — null-symbol detection, coarse/fine time-sync, FFT
//! - **[`decoder`]** — DQPSK differential decoding and frequency de-interleaving

pub mod decoder;
pub mod processor;
