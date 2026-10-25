//! Pure-Rust NRSC-5 / HD Radio receiver primitives.
//!
//! The crate is organized by receiver stage: [`front_end`] handles I/Q input,
//! [`ofdm`] performs physical-layer synchronization, [`protocol`] decodes
//! primary data, [`metadata`] handles station/service information, and
//! [`audio`] extracts and decodes HDC audio. It never links to the GPL
//! `nrsc5` reference decoder; that program is used only as an offline oracle
//! for test artifacts.

pub mod audio;
pub mod fec;
pub mod front_end;
pub mod metadata;
pub mod ofdm;
pub mod p1;
pub mod protocol;

pub use audio::{AudioTransport, HdcCoreAudio, HdcPdu};
pub use front_end::FrontEnd;
pub use metadata::model::{FrontEndReport, ReceiverSnapshot};
pub use protocol::config::{InputConfig, ReceiverConfig, parse_frequency_hz};
