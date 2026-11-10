//! One-shot encode: planar f32 → AAC-LC in ADTS or M4A.
//!
//! The encoder writes AAC-LC only (no SBR/PS), mono or stereo, with block
//! switching (long windows for steady content, short windows on detected
//! attacks). CBR-ish via a per-frame global scalefactor offset search plus
//! a bounded one-frame bit credit. Output is conformant enough that
//! libavcodec decodes it; the lavc-decoded goldens under `src/goldens/`
//! are the committed proof.

use crate::engine::adts::{ADTS_SAMPLE_RATES_HZ, AdtsHeader};
use crate::engine::enc_frame::LcEncoder;
use crate::engine::swb::LONG_WINDOW_LEN as FRAME;
use crate::error::{AacError, Result};
use crate::m4a_write;
use crate::options::{EncodeContainer, EncodeOptions};

/// Encode planar f32 PCM (~[-1, 1]) to an ADTS stream: AAC-LC at 128 kbps.
///
/// One plane per channel (mono or stereo), all planes the same length; the
/// last frame is zero-padded to 1024 samples. The stream carries the usual
/// 1024-sample codec priming (first decoded frame is a fade-in).
#[inline]
pub fn encode(pcm: &[Vec<f32>], sample_rate: u32) -> Result<Vec<u8>> {
    encode_with(pcm, sample_rate, &EncodeOptions::default())
}

/// Encode planar f32 PCM under `opts` (container + bitrate + lookahead).
///
/// With [`EncodeOptions::lookahead`] on, the attack detector runs one
/// frame ahead (better pre-echo suppression on early-in-frame onsets) at
/// one extra frame of internal latency; the output frame count and the
/// ADTS priming are unchanged.
pub fn encode_with(pcm: &[Vec<f32>], sample_rate: u32, opts: &EncodeOptions) -> Result<Vec<u8>> {
    validate(pcm, sample_rate, opts)?;
    let channels = pcm.len();
    let mut enc =
        LcEncoder::new(sample_rate, channels, opts.bitrate_bps)?.with_lookahead(opts.lookahead);
    let n_samples = pcm[0].len();
    let n_frames = n_samples.div_ceil(FRAME);
    let mut payloads: Vec<Vec<u8>> = Vec::with_capacity(n_frames);
    let mut bufs = [[0.0f32; FRAME]; 2];
    for f in 0..n_frames {
        let start = f * FRAME;
        let end = (start + FRAME).min(n_samples);
        for (ch, plane) in pcm.iter().enumerate() {
            bufs[ch] = [0.0; FRAME];
            bufs[ch][..end - start].copy_from_slice(&plane[start..end]);
        }
        let planes: Vec<&[f32]> = bufs[..channels].iter().map(|b| &b[..]).collect();
        if opts.lookahead {
            if let Some(payload) = enc.push_frame(&planes)? {
                payloads.push(payload);
            }
        } else {
            payloads.push(enc.encode_frame(&planes)?);
        }
    }
    if opts.lookahead
        && let Some(payload) = enc.flush()?
    {
        payloads.push(payload);
    }
    match opts.container {
        EncodeContainer::Adts => Ok(wrap_adts(&payloads, enc.fs_index(), channels)),
        EncodeContainer::M4a => {
            m4a_write::mux_aac_lc(&payloads, enc.fs_index(), channels, sample_rate)
        }
    }
}

/// Encode to ADTS and write the file.
pub fn write(path: impl AsRef<std::path::Path>, pcm: &[Vec<f32>], sample_rate: u32) -> Result<()> {
    write_with(path, pcm, sample_rate, &EncodeOptions::default())
}

/// Encode under `opts` and write the file.
pub fn write_with(
    path: impl AsRef<std::path::Path>,
    pcm: &[Vec<f32>],
    sample_rate: u32,
    opts: &EncodeOptions,
) -> Result<()> {
    let bytes = encode_with(pcm, sample_rate, opts)?;
    std::fs::write(path, bytes)?;
    Ok(())
}

fn validate(pcm: &[Vec<f32>], sample_rate: u32, opts: &EncodeOptions) -> Result<()> {
    if !ADTS_SAMPLE_RATES_HZ.contains(&sample_rate) {
        return Err(AacError::encode(format!(
            "encode: unsupported sample rate {sample_rate}Hz (not in the AAC table)"
        )));
    }
    if !(1..=2).contains(&pcm.len()) {
        return Err(AacError::encode(format!(
            "encode: channels must be 1 or 2, got {}",
            pcm.len()
        )));
    }
    if opts.bitrate_bps == 0 {
        return Err(AacError::encode("encode: bitrate must be > 0"));
    }
    let n = pcm[0].len();
    if n == 0 {
        return Err(AacError::encode("encode: empty input"));
    }
    if pcm.iter().any(|p| p.len() != n) {
        return Err(AacError::encode("encode: channel planes differ in length"));
    }
    if pcm.iter().any(|p| p.iter().any(|x| !x.is_finite())) {
        return Err(AacError::encode("encode: non-finite sample"));
    }
    Ok(())
}

/// ADTS-wrap each raw_data_block (no CRC, VBR fullness marker).
fn wrap_adts(payloads: &[Vec<u8>], fs_index: u8, channels: usize) -> Vec<u8> {
    let total: usize = payloads
        .iter()
        .map(|p| p.len() + crate::engine::adts::ADTS_HEADER_BYTES_NO_CRC)
        .sum();
    let mut out = Vec::with_capacity(total);
    for p in payloads {
        adts_frame_into(p, fs_index, channels, &mut out);
    }
    out
}

/// Append one ADTS-wrapped `raw_data_block` to `out` (no CRC, VBR fullness
/// marker). Shared by one-shot [`encode_with`] and the push [`crate::Encoder`].
pub(crate) fn adts_frame_into(payload: &[u8], fs_index: u8, channels: usize, out: &mut Vec<u8>) {
    let hdr = AdtsHeader {
        mpeg_version_mpeg2: false,
        protection_absent: true,
        profile: 1, // LC
        sampling_frequency_index: fs_index,
        channel_configuration: channels as u8,
        aac_frame_length: (crate::engine::adts::ADTS_HEADER_BYTES_NO_CRC + payload.len()) as u16,
        adts_buffer_fullness: 0x7FF,
        number_of_raw_data_blocks_in_frame: 1,
    };
    out.extend_from_slice(&hdr.write());
    out.extend_from_slice(payload);
}
