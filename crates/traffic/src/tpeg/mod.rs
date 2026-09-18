//! TPEG application layer: transport frames and TEC decoding.

pub mod tec;
pub mod transport;

pub use tec::{TEC_AID, decode_frame, decode_payload};
pub use transport::{
    SYNC_WORD, ServiceFrame, TransportFrame, encode_frame, find_frames, parse_tpeg1_service,
};
