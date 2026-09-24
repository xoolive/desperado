//! Streaming decode: a resumable push [`Decoder`] for ADTS and LATM/LOAS
//! byte streams, plus [`decode_streaming`] for complete in-memory buffers
//! of any supported container (ADTS, LATM/LOAS, M4A/ISOBMFF).
//!
//! Peak PCM memory is one AAC frame: decoded planes are borrowed by the
//! frame callback and never accumulated by the decoder itself. The push
//! decoder's input buffer holds at most one partial frame plus the bytes
//! fed but not yet pumped (compacted past 64 KiB of consumed prefix).
//!
//! ```
//! use syom::{DecodeOptions, Decoder};
//! let adts = include_bytes!("goldens/sine48.adts");
//! let mut dec = Decoder::new(DecodeOptions::speech());
//! let mut samples = 0usize;
//! for chunk in adts.chunks(7) {
//!     dec.feed(chunk, |f| {
//!         samples += f.samples;
//!         Ok(())
//!     })?;
//! }
//! let info = dec.finish(|f| {
//!     samples += f.samples;
//!     Ok(())
//! })?;
//! assert_eq!(info.sample_rate, 48_000);
//! assert_eq!(samples as u64, info.samples);
//! # Ok::<(), syom::AacError>(())
//! ```

use crate::engine::decode::StreamDecoder;
use crate::engine::error::Error as EngineError;
use crate::engine::latm::MuxCfg;
use crate::error::{AacError, Result};
use crate::isomp4::sniff_is_isobmff;
use crate::options::{ChannelMode, DEFAULT_MAX_INPUT_BYTES, DecodeOptions};

mod frame;
mod m4a;
mod pump;

pub use frame::{Frame, StreamInfo};

/// Container the push decoder has locked onto.
enum Container {
    Unknown,
    Adts,
    Latm,
}

/// Resumable push decoder for ADTS and LATM/LOAS byte streams. Peak PCM RAM
/// is O(frame). M4A/ISOBMFF input is rejected (`moov` needs random access;
/// use [`decode_streaming`] on the full slice).
///
/// Bytes are buffered until the container is sniffable (~8 bytes) and until
/// a whole compressed frame has arrived; partial trailing frames are
/// dropped by [`Decoder::finish`], matching one-shot decode. Whether input
/// is undecodable is likewise decided at `finish` ([`AacError::NotAac`];
/// LATM/LOAS keeps its decode-class error), not mid-`feed`.
pub struct Decoder {
    opts: DecodeOptions,
    buf: Vec<u8>,
    pos: usize,
    bytes_fed: u64,
    container: Container,
    dec: StreamDecoder,
    /// Sticky LATM `StreamMuxConfig()`; persists across feeds.
    mux: Option<MuxCfg>,
    /// Mono fast-path target, cleared per frame.
    mono_scratch: Vec<f32>,
    /// `(sample_rate, channels)` locked by the first emitted frame.
    locked: Option<(u32, usize)>,
    max_samples: usize,
    aac_frames: u64,
    /// Per-channel samples decoded (cap counter, pre-skip).
    samples_decoded: u64,
    /// Per-channel samples handed to the callback.
    samples_out: u64,
    emitted: u64,
}

impl Decoder {
    pub fn new(opts: DecodeOptions) -> Self {
        let mut dec = StreamDecoder::new();
        dec.mix_down_mono = matches!(opts.channel_mode, ChannelMode::Mono);
        Self {
            opts,
            buf: Vec::new(),
            pos: 0,
            bytes_fed: 0,
            container: Container::Unknown,
            dec,
            mux: None,
            mono_scratch: Vec::new(),
            locked: None,
            max_samples: 0,
            aac_frames: 0,
            samples_decoded: 0,
            samples_out: 0,
            emitted: 0,
        }
    }

    /// Feed bytes; `on_frame` fires once per completed AAC frame. Returns
    /// the number of bytes consumed (all of them; partial frames are
    /// buffered internally).
    pub fn feed<F>(&mut self, bytes: &[u8], on_frame: F) -> Result<usize>
    where
        F: FnMut(Frame<'_>) -> Result<()>,
    {
        self.bytes_fed = self.bytes_fed.saturating_add(bytes.len() as u64);
        if self.bytes_fed > DEFAULT_MAX_INPUT_BYTES {
            return Err(AacError::too_long(
                self.bytes_fed as f64 / 40_000.0, // rough lower bound, like decode_with
                self.opts.max_duration_secs,
            ));
        }
        // Take the buffer so `pump` can borrow it while mutating `self`.
        let mut buf = std::mem::take(&mut self.buf);
        buf.extend_from_slice(bytes);
        let mut cb = on_frame;
        let res = self.pump(&buf, None, None, &mut cb, false);
        // Compact the consumed prefix so `buf` stays O(frame + fresh input).
        if res.is_ok() && self.pos >= 64 * 1024 && self.pos >= buf.len() / 2 {
            buf.drain(..self.pos);
            self.pos = 0;
        }
        self.buf = buf;
        res?;
        Ok(bytes.len())
    }

    /// End of input: flush, drop a partial trailing frame, and report
    /// tallies. Errors [`AacError::NotAac`] when nothing decodable was seen.
    pub fn finish<F>(mut self, on_frame: F) -> Result<StreamInfo>
    where
        F: FnMut(Frame<'_>) -> Result<()>,
    {
        let buf = std::mem::take(&mut self.buf);
        let mut cb = on_frame;
        self.pump(&buf, None, None, &mut cb, true)?;
        self.tallies()
    }

    /// One-shot over a complete slice (ADTS/LATM): sniffs and pumps without
    /// copying the input into the push buffer. Equivalent to one `feed` of
    /// the whole slice plus `finish`.
    fn decode_slice<F>(mut self, data: &[u8], on_frame: F) -> Result<StreamInfo>
    where
        F: FnMut(Frame<'_>) -> Result<()>,
    {
        let mut cb = on_frame;
        self.pump(data, None, None, &mut cb, true)?;
        self.tallies()
    }

    /// One-shot mono over a complete slice: decode straight into `dst`,
    /// skipping the per-frame scratch + callback copy. Caller guarantees
    /// [`ChannelMode::Mono`]; `est_frames` pre-sizes `dst` after frame 1.
    fn decode_slice_mono(
        mut self,
        data: &[u8],
        est_frames: Option<usize>,
        dst: &mut Vec<f32>,
    ) -> Result<StreamInfo> {
        fn ignore(_: Frame<'_>) -> Result<()> {
            Ok(())
        }
        self.pump(data, Some(dst), est_frames, &mut ignore, true)?;
        self.tallies()
    }

    /// Terminal tallies; [`AacError::NotAac`] when nothing decodable was seen.
    fn tallies(&self) -> Result<StreamInfo> {
        if self.emitted == 0 {
            return Err(match self.container {
                // One-shot LATM reports an empty LOAS stream as decode-class.
                Container::Latm => AacError::from(EngineError::LoasSyncInvalid),
                _ => AacError::NotAac,
            });
        }
        let (sample_rate, channels) = self.locked.unwrap_or((0, 0));
        Ok(StreamInfo {
            sample_rate,
            channels,
            aac_frames: self.aac_frames,
            samples: self.samples_out,
        })
    }
}

/// Stream-decode a complete in-memory buffer (any container, incl. M4A).
///
/// `on_frame` fires once per decoded AAC frame; return an error from it to
/// abort. Peak PCM RAM is O(frame); M4A still holds the input slice (the
/// frame index needs `moov`).
pub fn decode_streaming<F>(data: &[u8], opts: &DecodeOptions, on_frame: F) -> Result<StreamInfo>
where
    F: FnMut(Frame<'_>) -> Result<()>,
{
    if data.len() as u64 > DEFAULT_MAX_INPUT_BYTES {
        return Err(AacError::too_long(
            data.len() as f64 / 40_000.0, // rough lower bound only for message
            opts.max_duration_secs,
        ));
    }
    if sniff_is_isobmff(data) {
        return m4a::stream_m4a(data, opts, on_frame);
    }
    Decoder::new(opts.clone()).decode_slice(data, on_frame)
}

/// One-shot mono fast path for [`crate::decode_with`]: append each frame's
/// mixed-mono samples straight into `dst` — no per-frame scratch, no
/// callback copy. ADTS/LATM only; M4A keeps the [`decode_streaming`] path.
pub(crate) fn decode_streaming_mono_into(
    data: &[u8],
    opts: &DecodeOptions,
    est_frames: Option<usize>,
    dst: &mut Vec<f32>,
) -> Result<StreamInfo> {
    if data.len() as u64 > DEFAULT_MAX_INPUT_BYTES {
        return Err(AacError::too_long(
            data.len() as f64 / 40_000.0, // rough lower bound only for message
            opts.max_duration_secs,
        ));
    }
    Decoder::new(opts.clone()).decode_slice_mono(data, est_frames, dst)
}
