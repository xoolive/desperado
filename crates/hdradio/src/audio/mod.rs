//! HDC audio transport and pure-Rust PCM decoding.

pub mod hdc;
pub mod transport;

pub use hdc::HdcCoreAudio;
pub use transport::{AudioTransport, HdcPdu};
