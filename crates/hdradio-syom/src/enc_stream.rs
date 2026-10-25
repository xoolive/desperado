//! Streaming encode: a resumable push [`Encoder`] for planar f32 PCM →
//! AAC-LC in ADTS.
//!
//! Feed PCM in chunks of any size; the callback fires once per completed
//! AAC frame (1024 samples per channel) with the ADTS-wrapped access unit.
//! Leftover samples are buffered across feeds; [`Encoder::finish`] encodes
//! the zero-padded tail frame, exactly like one-shot [`crate::encode_with`]
//! — same frame sequence, same bytes. M4A is rejected at construction
//! (`stco` / sample sizes need the finished totals; use
//! [`crate::encode_with`] with [`EncodeContainer::M4a`]), mirroring how the
//! push [`crate::Decoder`] rejects ISOBMFF input.
//!
//! With [`EncodeOptions::lookahead`] on, the callback trails the input by
//! one frame (the attack detector sees the next frame before a frame is
//! encoded) and `finish` flushes the held frame; byte-parity with one-shot
//! holds for the same options.
//!
//! ```
//! use syom::{DecodeOptions, EncodeOptions, Encoder};
//! let mut enc = Encoder::new(48_000, 1, &EncodeOptions::adts())?;
//! let pcm = vec![0.0f32; 3000];
//! let mut adts = Vec::new();
//! for chunk in pcm.chunks(777) {
//!     enc.feed(&[chunk], |f| {
//!         adts.extend_from_slice(f.au);
//!         Ok(())
//!     })?;
//! }
//! let info = enc.finish(|f| {
//!     adts.extend_from_slice(f.au);
//!     Ok(())
//! })?;
//! assert_eq!(info.samples, 3000);
//! assert_eq!(info.aac_frames, 3); // tail frame zero-padded to 1024
//! assert_eq!(info.bytes as usize, adts.len());
//! let dec = syom::decode_with(&adts, &DecodeOptions::unbounded())?;
//! assert_eq!(dec.channels[0].len(), 3 * 1024);
//! # Ok::<(), syom::AacError>(())
//! ```

use crate::engine::adts::ADTS_SAMPLE_RATES_HZ;
use crate::engine::enc_frame::LcEncoder;
use crate::engine::swb::LONG_WINDOW_LEN as FRAME;
use crate::error::{AacError, Result};
use crate::options::{EncodeContainer, EncodeOptions};

/// One encoded AAC frame: an ADTS-wrapped access unit.
///
/// The bytes borrow encoder scratch and are valid **only for the duration
/// of the callback** — copy them out to keep them.
pub struct EncodedFrame<'a> {
    /// Input samples per channel this frame carries: 1024, or the tail
    /// remainder at [`Encoder::finish`] (that frame is zero-padded to 1024
    /// in the bitstream, like one-shot encode).
    pub samples: usize,
    /// One ADTS frame (7-byte header + `raw_data_block`).
    pub au: &'a [u8],
}

/// Tallies from a finished streaming encode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncodeInfo {
    /// Input sample rate.
    pub sample_rate: u32,
    /// Channels per frame (1 or 2).
    pub channels: usize,
    /// AAC frames emitted, including the zero-padded tail frame.
    pub aac_frames: u64,
    /// Input samples per channel consumed.
    pub samples: u64,
    /// Bytes handed to the callback (ADTS headers included).
    pub bytes: u64,
}

/// Resumable push encoder: planar f32 chunks in, ADTS frames out.
///
/// Byte-exact with one-shot [`crate::encode_with`] on the same PCM, for any
/// feed chunking. Peak RAM is one frame of PCM plus the current access
/// unit. Validation matches one-shot encode: the ADTS sample-rate table,
/// 1–2 channels, finite samples, equal plane lengths.
pub struct Encoder {
    enc: LcEncoder,
    sample_rate: u32,
    channels: usize,
    /// Samples buffered across feeds, one plane per channel.
    pending: [Vec<f32>; 2],
    pending_len: usize,
    /// Reused access-unit scratch the callback borrows.
    scratch: Vec<u8>,
    samples: u64,
    aac_frames: u64,
    bytes: u64,
}

impl Encoder {
    /// New encoder for `channels` planes at `sample_rate` under `opts`.
    ///
    /// Errors [`AacError::Encode`] on an unsupported rate, channel count,
    /// zero bitrate, or [`EncodeContainer::M4a`] (M4A cannot be streamed
    /// incrementally; use [`crate::encode_with`]).
    pub fn new(sample_rate: u32, channels: usize, opts: &EncodeOptions) -> Result<Self> {
        if !ADTS_SAMPLE_RATES_HZ.contains(&sample_rate) {
            return Err(AacError::encode(format!(
                "encode: unsupported sample rate {sample_rate}Hz (not in the AAC table)"
            )));
        }
        if !(1..=2).contains(&channels) {
            return Err(AacError::encode(format!(
                "encode: channels must be 1 or 2, got {channels}"
            )));
        }
        if opts.bitrate_bps == 0 {
            return Err(AacError::encode("encode: bitrate must be > 0"));
        }
        if opts.container != EncodeContainer::Adts {
            return Err(AacError::encode(
                "encode: M4A/ISOBMFF needs finish-time sizes; use encode_with",
            ));
        }
        Ok(Self {
            enc: LcEncoder::new(sample_rate, channels, opts.bitrate_bps)?
                .with_lookahead(opts.lookahead),
            sample_rate,
            channels,
            pending: [Vec::new(), Vec::new()],
            pending_len: 0,
            scratch: Vec::new(),
            samples: 0,
            aac_frames: 0,
            bytes: 0,
        })
    }

    /// Feed PCM planes (any length, equal lengths per channel, plane count
    /// as passed to [`Encoder::new`]). `on_frame` fires per completed AAC
    /// frame; return an error from it to abort. Returns the number of
    /// samples consumed per channel (all of them; partial frames are
    /// buffered internally).
    pub fn feed<F>(&mut self, planes: &[&[f32]], on_frame: F) -> Result<usize>
    where
        F: FnMut(EncodedFrame<'_>) -> Result<()>,
    {
        if planes.len() != self.channels {
            return Err(AacError::encode(format!(
                "encode: expected {} channel plane(s), got {}",
                self.channels,
                planes.len()
            )));
        }
        let n = planes.first().map_or(0, |p| p.len());
        if planes.iter().any(|p| p.len() != n) {
            return Err(AacError::encode("encode: channel planes differ in length"));
        }
        if planes.iter().any(|p| p.iter().any(|x| !x.is_finite())) {
            return Err(AacError::encode("encode: non-finite sample"));
        }
        let mut cb = on_frame;
        // Take the scratch so `cb` can borrow it while `self` mutates.
        let mut scratch = std::mem::take(&mut self.scratch);
        let mut off = 0usize;
        while self.pending_len + (n - off) >= FRAME {
            let take = FRAME - self.pending_len;
            let mut bufs = [[0.0f32; FRAME]; 2];
            for (ch, buf) in bufs.iter_mut().enumerate().take(self.channels) {
                buf[..self.pending_len].copy_from_slice(&self.pending[ch][..self.pending_len]);
                buf[self.pending_len..].copy_from_slice(&planes[ch][off..off + take]);
            }
            self.emit(&bufs, FRAME, &mut scratch, &mut cb)?;
            off += take;
            self.pending_len = 0;
        }
        self.scratch = scratch;
        if off < n {
            if self.pending_len == 0 {
                // Pending was fully consumed (or never held anything).
                for (ch, plane) in planes.iter().enumerate() {
                    self.pending[ch].clear();
                    self.pending[ch].extend_from_slice(&plane[off..]);
                }
                self.pending_len = n - off;
            } else {
                // Not enough to complete a frame: append to pending.
                for (ch, plane) in planes.iter().enumerate() {
                    self.pending[ch].extend_from_slice(&plane[off..]);
                }
                self.pending_len += n - off;
            }
        }
        self.samples += n as u64;
        Ok(n)
    }

    /// End of input: encode the zero-padded tail frame (if any) and report
    /// tallies. With lookahead on, the tail push first flushes the
    /// previously held full frame and the lookahead flush then emits the
    /// tail itself; an exact-multiple input still has one held frame to
    /// flush. Errors [`AacError::Encode`] when no samples were ever fed,
    /// matching one-shot encode's empty-input rule.
    pub fn finish<F>(mut self, on_frame: F) -> Result<EncodeInfo>
    where
        F: FnMut(EncodedFrame<'_>) -> Result<()>,
    {
        if self.samples == 0 {
            return Err(AacError::encode("encode: empty input"));
        }
        let mut cb = on_frame;
        let mut scratch = std::mem::take(&mut self.scratch);
        if self.pending_len > 0 {
            let tail = self.pending_len;
            let mut bufs = [[0.0f32; FRAME]; 2];
            for (ch, buf) in bufs.iter_mut().enumerate().take(self.channels) {
                buf[..tail].copy_from_slice(&self.pending[ch][..tail]);
            }
            if self.enc.lookahead_enabled() {
                let planes: Vec<&[f32]> = bufs[..self.channels].iter().map(|b| &b[..]).collect();
                if let Some(au) = self.enc.push_frame(&planes)? {
                    // The tail push encodes the previously held FULL frame.
                    self.deliver(&au, FRAME, &mut scratch, &mut cb)?;
                }
                if let Some(au) = self.enc.flush()? {
                    self.deliver(&au, tail, &mut scratch, &mut cb)?;
                }
            } else {
                self.emit(&bufs, tail, &mut scratch, &mut cb)?;
            }
        } else if self.enc.lookahead_enabled()
            && let Some(au) = self.enc.flush()?
        {
            self.deliver(&au, FRAME, &mut scratch, &mut cb)?;
        }
        self.scratch = scratch;
        Ok(EncodeInfo {
            sample_rate: self.sample_rate,
            channels: self.channels,
            aac_frames: self.aac_frames,
            samples: self.samples,
            bytes: self.bytes,
        })
    }

    /// Encode one full frame out of `bufs`, ADTS-wrap it into `scratch`,
    /// deliver it to `cb`, and tally. With lookahead on, the push encodes
    /// the previously held (full) frame instead — one frame of latency —
    /// and the first push emits nothing.
    fn emit<F>(
        &mut self,
        bufs: &[[f32; FRAME]; 2],
        samples: usize,
        scratch: &mut Vec<u8>,
        cb: &mut F,
    ) -> Result<()>
    where
        F: FnMut(EncodedFrame<'_>) -> Result<()>,
    {
        let planes: Vec<&[f32]> = bufs[..self.channels].iter().map(|b| &b[..]).collect();
        if self.enc.lookahead_enabled() {
            if let Some(au) = self.enc.push_frame(&planes)? {
                self.deliver(&au, samples, scratch, cb)?;
            }
            return Ok(());
        }
        let au = self.enc.encode_frame(&planes)?;
        self.deliver(&au, samples, scratch, cb)
    }

    /// ADTS-wrap `au` into `scratch`, deliver it to `cb`, and tally.
    fn deliver<F>(
        &mut self,
        au: &[u8],
        samples: usize,
        scratch: &mut Vec<u8>,
        cb: &mut F,
    ) -> Result<()>
    where
        F: FnMut(EncodedFrame<'_>) -> Result<()>,
    {
        scratch.clear();
        crate::encode::adts_frame_into(au, self.enc.fs_index(), self.channels, scratch);
        cb(EncodedFrame {
            samples,
            au: scratch,
        })?;
        self.aac_frames += 1;
        self.bytes += scratch.len() as u64;
        Ok(())
    }
}

#[cfg(test)]
#[path = "enc_stream_tests.rs"]
mod enc_stream_tests;
