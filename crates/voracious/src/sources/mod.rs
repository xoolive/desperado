//! Data sources for VOR signal processing

pub mod iq;

pub use desperado::IqFormat;
pub use iq::{IqAsyncSource, IqSource};
