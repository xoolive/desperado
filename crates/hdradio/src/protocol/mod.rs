//! NRSC-5 configuration and primary data transport decoding.

pub mod config;
pub mod pids;

pub use config::{InputConfig, ReceiverConfig, parse_frequency_hz};
pub use pids::{PidsDecoder, PidsFrame};
