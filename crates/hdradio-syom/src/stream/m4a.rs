//! M4A/ISOBMFF frame walk for `decode_streaming` (slice-only: the frame
//! index needs `moov`). The `elst` encoder delay is honoured as a skip
//! counter *before* emission — frames fully inside the skip region are
//! still decoded (SBR/IMDCT overlap state) but not emitted; a partially
//! skipped frame emits `plane[skip..]`.

use crate::engine::asc::AudioSpecificConfig;
use crate::engine::decode::StreamDecoder;
use crate::error::{AacError, Result};
use crate::isomp4;
use crate::options::{ChannelMode, DecodeOptions};
use crate::stream::{Frame, StreamInfo};

pub(crate) fn stream_m4a<F>(
    data: &[u8],
    opts: &DecodeOptions,
    mut on_frame: F,
) -> Result<StreamInfo>
where
    F: FnMut(Frame<'_>) -> Result<()>,
{
    let track = isomp4::parse_aac_track(data).map_err(|e| {
        if e.is_format_class() {
            AacError::NotAac
        } else {
            e
        }
    })?;
    let (asc, _) = AudioSpecificConfig::parse(&track.asc)
        .map_err(|e| AacError::format(format!("aac: bad AudioSpecificConfig: {e:?}")))?;
    let out_rate = asc.output_sample_rate;
    if out_rate == 0 || out_rate > opts.max_sample_rate {
        return Err(AacError::sample_rate(out_rate, opts.max_sample_rate));
    }
    let max_samples = opts.max_frames(out_rate);
    if track.total_samples > max_samples as u64 {
        let observed_s = track.total_samples as f64 / out_rate.max(1) as f64;
        return Err(AacError::too_long(observed_s, opts.max_duration_secs));
    }

    let mono = matches!(opts.channel_mode, ChannelMode::Mono);
    let mut dec = StreamDecoder::new();
    dec.mix_down_mono = mono;
    let mut scratch: Vec<f32> = Vec::new();
    let mut skip_left = track.skip_samples(out_rate);
    let mut locked: Option<(u32, usize)> = None;
    let mut aac_frames = 0u64;
    let mut emitted = 0u64;
    let mut decoded = 0u64; // per-channel, pre-skip (cap counter)
    let mut samples_out = 0u64;

    for (idx, &(off, len)) in track.frames.iter().enumerate() {
        let payload = sample_payload(data, off, len)?;
        let rate = if mono {
            scratch.clear();
            dec.decode_raw_mono_f32(
                asc.aot,
                asc.sampling_frequency_index,
                asc.sample_rate,
                asc.channel_configuration,
                1,
                payload,
                &mut scratch,
            )
            .map_err(|e| AacError::decode(format!("aac: decode failed at frame {idx}: {e:?}")))?
        } else {
            dec.decode_frame_scaled(
                asc.aot,
                asc.sampling_frequency_index,
                asc.sample_rate,
                asc.channel_configuration,
                payload,
            )
            .map_err(|e| AacError::decode(format!("aac: decode failed at frame {idx}: {e}")))?
        };
        aac_frames += 1;
        let (n_ch, n) = if mono {
            (1, scratch.len())
        } else {
            let planes = dec.frame_planes();
            (planes.len(), planes.first().map_or(0, Vec::len))
        };
        if n_ch == 0 {
            continue; // channel-less frame: consumed, not emitted
        }
        match locked {
            None => {
                if rate == 0 || rate > opts.max_sample_rate {
                    return Err(AacError::sample_rate(rate, opts.max_sample_rate));
                }
                locked = Some((rate, n_ch));
            }
            Some((r, c)) => {
                if rate != r {
                    return Err(AacError::decode(format!(
                        "aac: sample rate changed mid-stream ({r}Hz → {rate}Hz)"
                    )));
                }
                if c != n_ch {
                    return Err(AacError::decode(format!(
                        "aac: channel count changed mid-stream ({c} → {n_ch})"
                    )));
                }
            }
        }
        decoded += n as u64;
        if decoded > max_samples as u64 {
            let observed_s = decoded as f64 / f64::from(rate.max(1));
            return Err(AacError::too_long(observed_s, opts.max_duration_secs));
        }
        if skip_left >= n {
            skip_left -= n; // fully inside the edit-list skip region
            continue;
        }
        let start = skip_left;
        skip_left = 0;
        emitted += 1;
        samples_out += (n - start) as u64;
        if mono {
            let plane: [&[f32]; 1] = [&scratch[start..]];
            on_frame(Frame {
                sample_rate: rate,
                samples: n - start,
                planar: &plane,
            })?;
        } else {
            let planes: Vec<&[f32]> = dec.frame_planes().iter().map(|ch| &ch[start..]).collect();
            on_frame(Frame {
                sample_rate: rate,
                samples: n - start,
                planar: &planes,
            })?;
        }
    }

    if emitted == 0 {
        return Err(AacError::NotAac);
    }
    let (sample_rate, channels) = locked.unwrap_or((out_rate, 0));
    Ok(StreamInfo {
        sample_rate,
        channels,
        aac_frames,
        samples: samples_out,
    })
}

fn sample_payload(data: &[u8], off: u64, len: u32) -> Result<&[u8]> {
    let end = off
        .checked_add(u64::from(len))
        .ok_or_else(|| AacError::format("aac: sample range overflow"))? as usize;
    data.get(off as usize..end)
        .ok_or_else(|| AacError::format("aac: sample range past end of file"))
}
