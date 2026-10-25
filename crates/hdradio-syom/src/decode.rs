//! One-shot decode: a collecting frame callback over the streaming core
//! (`stream` module owns the container state machines and caps).

use crate::engine::adts::AdtsHeader;
use crate::error::Result;
use crate::isomp4::sniff_is_isobmff;
use crate::options::{ChannelMode, DecodeOptions};
use crate::stream::{decode_streaming, decode_streaming_mono_into};

/// Decoded AAC at native sample rate (planar f32, mono-mixed or split).
///
/// Split channel order: mono/stereo as-is; multichannel AAC-LC
/// (`channel_configuration` 3–6) follows the libavcodec layout order
/// (5.1 = FL FR FC LFE BL BR); PCE streams follow PCE declaration order
/// (front, side, back, LFE). Speech mono is the mean of the non-LFE planes.
#[derive(Debug, Clone)]
pub struct DecodedAac {
    pub sample_rate: u32,
    pub channels: Vec<Vec<f32>>,
}

/// Decode ADTS, M4A, or LATM/LOAS bytes with speech-ingest defaults.
#[inline]
pub fn decode(data: &[u8]) -> Result<DecodedAac> {
    decode_with(data, &DecodeOptions::speech())
}

/// Alias of [`decode`].
#[inline]
pub fn decode_bytes(data: &[u8]) -> Result<DecodedAac> {
    decode(data)
}

/// Read a file and [`decode`] it.
pub fn read(path: impl AsRef<std::path::Path>) -> Result<DecodedAac> {
    decode(&std::fs::read(path)?)
}

/// Read a file and [`decode_with`] it.
pub fn read_with(path: impl AsRef<std::path::Path>, opts: &DecodeOptions) -> Result<DecodedAac> {
    decode_with(&std::fs::read(path)?, opts)
}

/// Decode ADTS, M4A, or LATM/LOAS bytes under `opts`.
pub fn decode_with(data: &[u8], opts: &DecodeOptions) -> Result<DecodedAac> {
    let est_frames = adts_frame_estimate(data);
    if matches!(opts.channel_mode, ChannelMode::Mono) && !sniff_is_isobmff(data) {
        // Mono fast path: frames decode straight into the output plane, no
        // per-frame scratch or callback copy.
        let mut track: Vec<f32> = Vec::new();
        let info = decode_streaming_mono_into(data, opts, est_frames, &mut track)?;
        return Ok(DecodedAac {
            sample_rate: info.sample_rate,
            channels: vec![track],
        });
    }
    let mut tracks: Vec<Vec<f32>> = Vec::new();
    let info = decode_streaming(data, opts, |f| {
        if tracks.is_empty() {
            tracks.resize_with(f.planar.len(), Vec::new);
            // Pre-size once from the frame-count estimate instead of growing
            // geometrically per frame (each growth reallocs + recopies).
            if let Some(frames) = est_frames {
                let per = (frames * f.samples).min(opts.max_frames(f.sample_rate));
                for t in &mut tracks {
                    t.reserve(per);
                }
            }
        }
        for (dst, src) in tracks.iter_mut().zip(f.planar.iter()) {
            dst.extend_from_slice(src);
        }
        Ok(())
    })?;
    Ok(DecodedAac {
        sample_rate: info.sample_rate,
        channels: tracks,
    })
}

/// ADTS frame-count estimate for output pre-sizing: walk the frame headers
/// (payloads are skipped, so this is far cheaper than decoding). Stops at
/// the first unparseable or partial header — a hint only, never an error.
/// `None` when the input does not start with ADTS (M4A, LATM, garbage).
fn adts_frame_estimate(data: &[u8]) -> Option<usize> {
    let mut pos = 0usize;
    let mut frames = 0usize;
    while let Ok((hdr, _)) = AdtsHeader::parse(&data[pos..]) {
        let len = usize::from(hdr.aac_frame_length);
        if len == 0 || pos + len > data.len() {
            break;
        }
        pos += len;
        frames += 1;
    }
    (frames > 0).then_some(frames)
}
