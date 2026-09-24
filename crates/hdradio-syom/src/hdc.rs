//! NRSC-5 HDC bitstream adapter for the syom AAC core.

use crate::engine::decode::StreamDecoder;
use crate::{AacError, Result};

/// Stateful pure-Rust decoder for the AAC-LC core of NRSC-5 HDC PDUs.
///
/// HDC's DRM-derived SBR syntax is distinct from ordinary HE-AAC. Compatible
/// frames are synthesized at 44.1 kHz; unsupported SBR tables safely return
/// the 22.05 kHz AAC-LC core.
#[derive(Debug, Default)]
pub struct HdcCoreDecoder {
    decoder: StreamDecoder,
}

/// Decoded PCM, planar f32 at either 22,050 Hz core rate or 44,100 Hz with SBR.
#[derive(Debug, Clone)]
pub struct DecodedHdcCore {
    /// The HDC AAC-LC core sample rate.
    pub sample_rate: u32,
    /// One PCM plane per decoded channel.
    pub channels: Vec<Vec<f32>>,
}

impl HdcCoreDecoder {
    /// Decode a single HDC packet data unit's AAC core.
    pub fn decode(&mut self, payload: &[u8]) -> Result<DecodedHdcCore> {
        let sample_rate = self
            .decoder
            .decode_hdc_core_scaled(payload)
            .map_err(AacError::from)?;
        Ok(DecodedHdcCore {
            sample_rate,
            channels: self.decoder.hdc_planes().to_vec(),
        })
    }
}
