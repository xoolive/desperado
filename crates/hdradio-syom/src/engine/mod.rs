//! Original AAC-LC / HE-AAC engine (ISO/IEC 14496-3 / 13818-7).
//!
//! Product entry is [`decode::StreamDecoder`].

pub mod adts;
pub mod asc;
pub mod bits;
pub(crate) mod channel_map;
pub mod crc;
pub mod decode;
pub(crate) mod decode_cpe;
pub(crate) mod det_math;
pub(crate) mod enc_frame;
pub(crate) mod enc_huff;
pub(crate) mod enc_ms;
pub(crate) mod enc_psy;
pub(crate) mod enc_quant;
pub(crate) mod enc_section;
pub(crate) mod enc_short;
pub(crate) mod enc_tns;
pub(crate) mod error;
pub mod extension_payload;
pub mod filterbank;
pub mod huff;
pub mod huff_esc;
pub mod huff_pair;
pub mod huff_quad;
pub mod ics;
pub mod ics_body;
pub mod imdct;
pub mod latm;
pub(crate) mod mdct;
pub mod pns;
pub mod ps_data;
pub mod ps_decoder;
pub mod ps_decorr;
pub mod ps_huffman;
pub mod ps_hybrid;
pub mod ps_map;
pub mod ps_stereo;
pub mod raw_data_block;
pub mod sbr_decoder;
pub mod sbr_dequant;
pub mod sbr_element;
pub mod sbr_env_adjust;
pub mod sbr_envelope;
pub mod sbr_extension;
pub mod sbr_freq_bands;
pub mod sbr_grid;
pub mod sbr_header;
pub mod sbr_hf_gen;
pub mod sbr_huffman;
pub mod sbr_limiter;
pub mod sbr_noise_table;
pub mod sbr_qmf;
pub mod sbr_reconstruct;
pub mod sbr_time_grid;
pub mod section;
pub mod sf;
pub mod sf_tab;
pub mod skip;
pub mod spectrum;
pub mod stereo;
pub mod swb;
pub mod tns;

pub(crate) use error::Error;
pub use error::Result;

#[cfg(test)]
#[path = "huff_tests.rs"]
mod huff_tests;

#[cfg(test)]
#[path = "ics_tests.rs"]
mod ics_tests;

#[cfg(test)]
#[path = "tools_tests.rs"]
mod tools_tests;

#[cfg(test)]
#[path = "channel_map_tests.rs"]
mod channel_map_tests;

#[cfg(test)]
#[path = "golden_tests.rs"]
mod golden_tests;

#[cfg(test)]
#[path = "extension_payload_tests.rs"]
mod extension_payload_tests;
