//! Streaming frame types: the borrowed [`Frame`] delivered to the callback
//! and the [`StreamInfo`] tallies returned at end of stream.

/// One decoded AAC frame: planar f32 in [-1, 1], one plane per channel.
///
/// The planes borrow decoder scratch and are valid **only for the duration
/// of the frame callback** — copy them out to keep them.
pub struct Frame<'a> {
    /// Native sample rate after SBR (2× the core rate for HE-AAC).
    pub sample_rate: u32,
    /// Per-channel sample count in this frame.
    pub samples: usize,
    /// Planes in the same channel order as [`crate::DecodedAac`].
    pub planar: &'a [&'a [f32]],
}

/// Tallies from a finished stream decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamInfo {
    /// Native sample rate of the stream (0 if nothing decodable was seen).
    pub sample_rate: u32,
    /// Channels per emitted frame (0 if nothing decodable was seen).
    pub channels: usize,
    /// Payload frames decoded, including edit-list-skipped ones.
    pub aac_frames: u64,
    /// Output samples per channel after any `elst` skip.
    pub samples: u64,
}
