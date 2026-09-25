//! A DAB/DAB+ (ETSI EN 300 401) decoder, as a library.
//!
//! Feed it raw I/Q at 2.048 Msps and it decodes a DAB Mode I ensemble: the
//! sync and OFDM front end, the Fast Information Channel (service list, labels,
//! programme types), the Main Service Channel and its FEC, the DAB+
//! super-frame/Reed–Solomon layer, and Programme Associated Data (Dynamic
//! Label, MOT slide-show). The `dabradio` binary in this crate is a thin TUI
//! around it.
//!
//! # Architecture
//!
//! The processing pipeline is split into the following stages:
//!
//! ```text
//! IQ stream ─► OFDM sync & FFT ─► DQPSK decode ─┬─► FIC ─► ensemble metadata
//!                                               └─► MSC ─► subchannel frames
//!                                                              │
//!                                                     FEC (Viterbi + RS)
//!                                                              │
//!                                                     AAC decode ─► audio
//!                                                              │
//!                                                     PAD ─► DLS / MOT
//! ```
//!
//! # Modules
//!
//! | Module | Purpose |
//! |---|---|
//! | [`ofdm`] | Frame synchronisation, FFT, DQPSK differential decoding |
//! | [`fic`] | Fast Information Channel — ensemble & service metadata |
//! | [`msc`] | Main Service Channel — subchannel extraction |
//! | [`fec`] | Forward Error Correction (EEP depuncturing, Viterbi, energy dispersal) |
//! | [`audio`] | DAB+ super-frame assembly, Reed–Solomon, AAC decoding |
//! | [`pad`] | Programme Associated Data (Dynamic Label Segment, MOT slide-show) |
//! | [`constants`] | DAB Mode I parameters and Band III channel table |
//! | [`charsets`] | EBU Latin → UTF-8 conversion for service labels |
//!
//! # Audio codecs
//!
//! DAB (not DAB+) carries MP2, decoded here in pure Rust. DAB+ carries
//! HE-AAC, which the [`audio::AacDecoder`] and [`audio::DabPlusDecoder`] types
//! decode through `fdk-aac` — behind the **`fdk-aac` feature**, which is on by
//! default. With it off, the decoder still syncs, decodes the ensemble and the
//! services, and hands over the DAB+ super-frames (Reed–Solomon corrected,
//! Access Units extracted) via [`audio::SuperframeDecoder`] for a caller that
//! has its own AAC decoder.

pub mod audio;
pub mod charsets;
pub mod constants;
pub mod fec;
pub mod fic;
pub mod msc;
pub mod ofdm;
pub mod pad;
