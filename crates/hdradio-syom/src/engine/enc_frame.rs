//! `LcEncoder` — per-frame LC pipeline: attack detector → window sequence
//! state machine → window + forward MDCT → TNS analysis (`enc_tns`) →
//! per-band M/S (`enc_ms`) → psy model (per-band precision targets) →
//! quantize → section plan → rate loop (one global sf offset, `rate`
//! submodule) → `raw_data_block()` bytes.
//!
//! Block switching: the causal state machine (default — no lookahead, no
//! added latency) emits OnlyLong → LongStart → EightShort → LongStop; an
//! attack makes THIS frame a LongStart (its window zeroes the tail around
//! the attack) and the next an EightShort, so the transient is coded on
//! 128-bin short windows. Transitions are shape-legal (`{OnlyLong,
//! LongStop}` ↔ long family, `{LongStart, EightShort}` ↔ short family), so
//! overlap-add stays TDAC-perfect. Stereo (CPE `common_window = 1`) shares
//! the sequence: the attack decision is the OR of both channels'
//! detectors. TNS runs on the raw L/R spectra before M/S — the decoder
//! applies the inverse filter after the M/S undo (`decode_cpe`); long
//! frames only (short frames emit `tns_data_present = 0`, v1). No
//! PNS/intensity, no bit reservoir (`adts_buffer_fullness = 0x7FF` VBR
//! marker), no `scale_factor_grouping` (8 groups of 1 window).
//!
//! Known causal weakness, and the opt-in fix: the LongStart window stays
//! flat for the first 1024 + 448 taps, so an attack landing in the first
//! ~448 NEW samples of a frame is still coded by the flat-region long
//! transform — reduced, not eliminated, pre-echo. With one-frame lookahead
//! on (`crate::EncodeOptions::with_lookahead`; entry points `push_frame` /
//! `flush` in the `lookahead` child module), the attack detectors run one
//! frame ahead of the encode: frame N's sequence is decided by
//! attack(N) || attack(N+1), so an attack anywhere in frame N+1 makes
//! frame N a LongStart — its start-window slope covers the pre-attack
//! tail — and frame N+1 an EightShort, coding early attacks entirely on
//! short windows. Mid-stream the attack(N) half of the OR is unreachable
//! (attack(N) already made frame N−1 a LongStart, which forces N short);
//! it matters only for frame 0 and for the final frame flushed without a
//! successor. Cost: one frame (1024 samples) of latency; `flush` emits the
//! held frame at end of input.

use super::adts::ADTS_SAMPLE_RATES_HZ;
use super::enc_ms::{self, MsBands};
use super::enc_psy::{AttackDetector, Psy};
use super::enc_quant::{MAX_BANDS, MAX_FLAT_SHORT, QuantChannel, QuantShort};
use super::enc_short::{self, ShortWindows};
use super::enc_tns::{self, EncTns};
use super::error::{Error, Result};
use super::filterbank::window_left;
use super::ics::{WindowSequence, WindowShape};
use super::mdct::mdct_into_f32;
use super::swb::{LONG_WINDOW_LEN, long_offsets, short_offsets};

/// ADTS `raw_data_block` payload ceiling (8191-byte frame − 7-byte header).
pub const MAX_PAYLOAD_BYTES: usize = 8184;
/// Flat quality target: band peaks quantize to about this magnitude before
/// the rate loop trades it against the bit budget.
pub(crate) const TARGET_Q: f32 = 2048.0;

/// AAC-LC encoder state (push-shaped: one `raw_data_block` per 1024
/// samples, so a streaming wrapper is additive later).
pub struct LcEncoder {
    fs_index: u8,
    sample_rate: u32,
    channels: usize,
    bitrate_bps: u32,
    offsets: &'static [u16],
    short_offsets: &'static [u16],
    /// Full 2048-tap KBD window pre-scaled by 32768 (decoder filterbank
    /// output is s16-scaled before the public 1/32768 mapping).
    window: Box<[f32; 2 * LONG_WINDOW_LEN]>,
    /// Start/stop/short analysis windows (same scaling).
    windows: ShortWindows,
    prev: [[f32; LONG_WINDOW_LEN]; 2],
    chans: Box<[QuantChannel; 2]>,
    books: [[u8; MAX_BANDS]; 2],
    gains: [u8; 2],
    psy: Psy,
    target_q: [[f32; MAX_BANDS]; 2],
    /// Unspent bits carried forward (bounded at one frame's budget): the
    /// integer sf-offset step undershoots by up to ~11%, so frames may
    /// spend accumulated savings. ADTS carries the VBR fullness marker.
    credit: i64,
    /// This frame's per-band M/S decision (`ms_mask_present` 0/1/2).
    ms: MsBands,
    /// This frame's per-channel TNS decision (`enc_tns`; off on short).
    tns: [EncTns; 2],
    /// Previously emitted sequence (legal-transition state).
    prev_seq: WindowSequence,
    /// This frame's sequence (set by `encode_frame`).
    seq: WindowSequence,
    /// Per-channel attack detectors (OR'd for the shared CPE decision).
    detectors: [AttackDetector; 2],
    /// One-frame attack lookahead: the detectors run one frame ahead of
    /// the encode (`push_frame` / `flush` in the `lookahead` child module).
    lookahead: bool,
    /// The frame held for the lookahead decision (a private copy — the
    /// caller's buffers are reused between pushes).
    held: Option<Box<lookahead::HeldFrame>>,
    /// Short-path state (touched only on EightShort frames).
    chans_s: Box<[QuantShort; 2]>,
    books_s: [[u8; MAX_FLAT_SHORT]; 2],
    target_q_s: [[f32; MAX_FLAT_SHORT]; 2],
    psy_short: Psy,
    /// Test hook: force OnlyLong every frame (pre-echo A/B measurement).
    #[cfg(test)]
    block_switching: bool,
    /// Test hook: whole-pair M/S A/B (per-band when true).
    #[cfg(test)]
    ms_per_band: bool,
    /// Test hook: TNS A/B (long frames, on when true).
    #[cfg(test)]
    tns_enabled: bool,
}

impl LcEncoder {
    /// `channels` must be 1 or 2; `sample_rate` must be in the ADTS table.
    pub fn new(sample_rate: u32, channels: usize, bitrate_bps: u32) -> Result<Self> {
        let fs_index = ADTS_SAMPLE_RATES_HZ
            .iter()
            .position(|&r| r == sample_rate)
            .ok_or(Error::UnsupportedSampleRateIndex(0xFF))? as u8;
        if !(1..=2).contains(&channels) {
            return Err(Error::Format("LC encoder: channels must be 1 or 2"));
        }
        if bitrate_bps == 0 {
            return Err(Error::Format("LC encoder: bitrate must be > 0"));
        }
        let offsets = long_offsets(fs_index)?;
        let short_offsets = short_offsets(fs_index)?;
        // KBD both halves (alpha 4, matching the decoder table): far better
        // sidelobe rejection than sine, so masked-band decisions aren't
        // fighting analysis leakage. The ics_info shape bit matches.
        let half = window_left(2 * LONG_WINDOW_LEN, WindowShape::Kbd);
        let mut window = Box::new([0.0f32; 2 * LONG_WINDOW_LEN]);
        for i in 0..LONG_WINDOW_LEN {
            window[i] = half[i] * 32768.0;
            window[2 * LONG_WINDOW_LEN - 1 - i] = half[i] * 32768.0;
        }
        let n_bands = offsets.len() - 1;
        Ok(Self {
            fs_index,
            sample_rate,
            channels,
            bitrate_bps,
            offsets,
            short_offsets,
            window,
            windows: ShortWindows::new(),
            prev: [[0.0; LONG_WINDOW_LEN]; 2],
            chans: Box::new([QuantChannel::new(n_bands), QuantChannel::new(n_bands)]),
            books: [[0; MAX_BANDS]; 2],
            gains: [100; 2],
            psy: Psy::new(offsets, sample_rate),
            target_q: [[0.0; MAX_BANDS]; 2],
            credit: 0,
            ms: MsBands::off(),
            tns: [EncTns::off(), EncTns::off()],
            prev_seq: WindowSequence::OnlyLong,
            seq: WindowSequence::OnlyLong,
            detectors: [AttackDetector::new(), AttackDetector::new()],
            lookahead: false,
            held: None,
            chans_s: Box::new([
                QuantShort::new(short_offsets.len() - 1),
                QuantShort::new(short_offsets.len() - 1),
            ]),
            books_s: [[0; MAX_FLAT_SHORT]; 2],
            target_q_s: [[0.0; MAX_FLAT_SHORT]; 2],
            psy_short: Psy::new_short(short_offsets, sample_rate),
            #[cfg(test)]
            block_switching: true,
            #[cfg(test)]
            ms_per_band: true,
            #[cfg(test)]
            tns_enabled: true,
        })
    }

    /// Wire `sampling_frequency_index` (for the ADTS header / ASC).
    #[must_use]
    pub fn fs_index(&self) -> u8 {
        self.fs_index
    }

    /// Opt into one-frame attack lookahead (see the module docs). Encode
    /// via `push_frame` / `flush` (child module `lookahead`) instead of
    /// `encode_frame`; one-shot `encode_with` and the push `Encoder` wire
    /// this from `EncodeOptions::lookahead`.
    #[must_use]
    pub fn with_lookahead(mut self, on: bool) -> Self {
        self.lookahead = on;
        self
    }

    /// Is one-frame attack lookahead enabled?
    pub(crate) fn lookahead_enabled(&self) -> bool {
        self.lookahead
    }

    /// Test-only A/B switch for pre-echo measurements.
    #[cfg(test)]
    pub fn set_block_switching(&mut self, on: bool) {
        self.block_switching = on;
    }

    /// Test-only A/B switch: whole-pair M/S (off) vs per-band (on).
    #[cfg(test)]
    pub fn set_ms_per_band(&mut self, on: bool) {
        self.ms_per_band = on;
    }

    /// Test-only A/B switch: TNS on long frames.
    #[cfg(test)]
    pub fn set_tns(&mut self, on: bool) {
        self.tns_enabled = on;
    }

    /// Test hook: did the last frame emit TNS on any channel?
    #[cfg(test)]
    pub(crate) fn tns_on(&self) -> bool {
        self.tns[..self.channels].iter().any(EncTns::is_on)
    }

    /// Per-frame bit budget from the target bitrate (no reservoir).
    fn budget_bits(&self) -> usize {
        let bits = u64::from(self.bitrate_bps) * 1024 / u64::from(self.sample_rate);
        (bits as usize).min(MAX_PAYLOAD_BYTES * 8)
    }

    /// Window sequence state machine (see the module docs). Causal: the
    /// decision uses only frames already pushed.
    fn next_seq(&self, attack: bool) -> WindowSequence {
        match (self.prev_seq, attack) {
            (WindowSequence::OnlyLong | WindowSequence::LongStop, true) => {
                WindowSequence::LongStart
            }
            (WindowSequence::OnlyLong | WindowSequence::LongStop, false) => {
                WindowSequence::OnlyLong
            }
            (WindowSequence::LongStart, _) => WindowSequence::EightShort,
            (WindowSequence::EightShort, true) => WindowSequence::EightShort,
            (WindowSequence::EightShort, false) => WindowSequence::LongStop,
        }
    }

    /// Frame-shape check shared by the causal and lookahead entry points.
    fn check_shape(&self, pcm: &[&[f32]]) -> Result<()> {
        if pcm.len() != self.channels || pcm.iter().any(|c| c.len() != LONG_WINDOW_LEN) {
            return Err(Error::Format("LC encoder: bad frame shape"));
        }
        Ok(())
    }

    /// Run the per-channel attack detectors on `pcm` (OR'd for the shared
    /// CPE window decision).
    fn detect(&mut self, pcm: &[&[f32]]) -> bool {
        let attack = (0..self.channels).any(|ch| self.detectors[ch].push(pcm[ch]));
        #[cfg(test)]
        let attack = attack && self.block_switching;
        attack
    }

    /// Encode 1024 samples per channel into one `raw_data_block`. Causal
    /// path (lookahead off): the attack decision comes from THIS frame's
    /// samples.
    pub fn encode_frame(&mut self, pcm: &[&[f32]]) -> Result<Vec<u8>> {
        self.check_shape(pcm)?;
        let attack = self.detect(pcm);
        self.encode_with_attack(pcm, attack)
    }

    /// Core pipeline — window + forward MDCT → TNS → per-band M/S → psy →
    /// quantize → section plan → rate loop → emit — given the attack
    /// decision driving this frame's window sequence. The lookahead path
    /// supplies a decision taken one frame ahead (`lookahead` child
    /// module); everything downstream of the sequence choice is identical.
    fn encode_with_attack(&mut self, pcm: &[&[f32]], attack: bool) -> Result<Vec<u8>> {
        self.seq = self.next_seq(attack);
        let mut specs = [[0.0f32; LONG_WINDOW_LEN]; 2];
        for ((spec, prev), plane) in specs.iter_mut().zip(self.prev.iter()).zip(pcm.iter()) {
            let mut buf = [0.0f32; 2 * LONG_WINDOW_LEN];
            buf[..LONG_WINDOW_LEN].copy_from_slice(prev);
            buf[LONG_WINDOW_LEN..].copy_from_slice(plane);
            match self.seq {
                WindowSequence::EightShort => enc_short::spectra(&buf, &self.windows, spec),
                seq => {
                    let window: &[f32] = match seq {
                        WindowSequence::LongStart => &self.windows.start[..],
                        WindowSequence::LongStop => &self.windows.stop[..],
                        _ => &self.window[..],
                    };
                    for (b, &w) in buf.iter_mut().zip(window.iter()) {
                        *b *= w;
                    }
                    mdct_into_f32(&buf, spec);
                }
            }
        }
        for (prev, plane) in self.prev.iter_mut().zip(pcm.iter()) {
            prev.copy_from_slice(plane);
        }
        // TNS analysis on the raw L/R spectra, before the M/S decision:
        // the decoder applies the inverse filter after the M/S undo
        // (`decode_cpe`), so the analysis must mirror that order. The psy
        // model works on the ORIGINAL (pre-TNS) spectrum: whitening drops
        // the loudest band by the prediction gain, which would sink the
        // −60 dB coded-band floor with it. The TNS span is the coded-band
        // span (`enc_tns` module docs), so the masks are computed here on
        // the L/R spectra; `enc_frame_rate` snapshots for the build psy.
        let long = !self.seq.is_eight_short();
        let mut psy_specs = [[0.0f32; LONG_WINDOW_LEN]; 2];
        let mut coded = [[false; MAX_BANDS]; 2];
        if long {
            psy_specs = specs;
            for ch in 0..self.channels {
                let mut tq = [0.0f32; MAX_BANDS];
                self.psy
                    .analyze(&specs[ch], self.offsets, TARGET_Q, &mut coded[ch], &mut tq);
            }
        }
        #[cfg(test)]
        let tns_enabled = self.tns_enabled;
        #[cfg(not(test))]
        let tns_enabled = true;
        self.tns = enc_tns::decide_frame(
            &mut specs,
            self.channels,
            self.seq,
            self.offsets,
            self.fs_index,
            &coded,
            tns_enabled,
        );
        // Per-band M/S decision, once per frame before the rate loop
        // (`enc_ms` module docs); chosen bands are transformed in place.
        self.ms = if self.channels == 2 {
            #[cfg(test)]
            let per_band = self.ms_per_band;
            #[cfg(not(test))]
            let per_band = true;
            if self.seq.is_eight_short() {
                enc_ms::decide_short(&mut specs, self.short_offsets, per_band)
            } else {
                enc_ms::decide_long(&mut specs, self.offsets, per_band)
            }
        } else {
            MsBands::off()
        };
        if long && self.channels == 2 {
            self.ms.apply_long_to(&mut psy_specs, self.offsets);
        }
        let budget = self.budget_bits();
        let spend = budget + (self.credit.min(budget as i64 / 2)) as usize;
        let offset = self.search_offset(&specs, &psy_specs, spend);
        let bits = self.build(&specs, &psy_specs, offset);
        if bits > spend {
            self.drop_bands_until(spend);
        }
        let out = self.emit();
        self.credit =
            (self.credit + budget as i64 - (out.len() * 8) as i64).clamp(0, budget as i64);
        self.prev_seq = self.seq;
        Ok(out)
    }
}

/// Rate loop + emission (`enc_frame_rate.rs`): split out for the line cap.
/// A child module, so the impl keeps using `LcEncoder`'s private fields.
#[path = "enc_frame_rate.rs"]
mod rate;

/// One-frame attack lookahead (`enc_frame_lookahead.rs`): `push_frame` /
/// `flush`, split out for the line cap. A child module, like `rate`.
#[path = "enc_frame_lookahead.rs"]
mod lookahead;

#[cfg(test)]
#[path = "enc_frame_tests.rs"]
mod enc_frame_tests;

#[cfg(test)]
#[path = "enc_block_tests.rs"]
mod enc_block_tests;

#[cfg(test)]
#[path = "enc_lookahead_tests.rs"]
mod enc_lookahead_tests;
