//! Rate loop + `raw_data_block` emission for [`LcEncoder`] — split from
//! `enc_frame.rs` for the line cap. A child module of `enc_frame`, so the
//! `impl` below keeps using the encoder's private fields; no new
//! abstraction boundary.

use super::{LcEncoder, MAX_PAYLOAD_BYTES, TARGET_Q};
use crate::engine::enc_quant::{self, MAX_BANDS};
use crate::engine::enc_section;
use crate::engine::enc_short;
use crate::engine::swb::LONG_WINDOW_LEN;

/// Rate-loop search bounds for the global scalefactor offset. Lower offset
/// = larger quantized values = finer quantization = more bits; the floor
/// stays clear of the escape-saturation plateau.
const OFFSET_LO: i32 = -8;
const OFFSET_HI: i32 = 80;

impl LcEncoder {
    /// Smallest global sf offset whose frame fits the bit budget (frame
    /// bits decrease as the offset grows; the smallest fitting offset is
    /// the finest quantization we can afford).
    pub(super) fn search_offset(
        &mut self,
        specs: &[[f32; LONG_WINDOW_LEN]; 2],
        psy_specs: &[[f32; LONG_WINDOW_LEN]; 2],
        spend: usize,
    ) -> i32 {
        let budget = spend.saturating_sub(32);
        if self.build(specs, psy_specs, OFFSET_LO) <= budget {
            return OFFSET_LO; // maximum quality fits
        }
        if self.build(specs, psy_specs, OFFSET_HI) > budget {
            return OFFSET_HI; // over budget even at ceiling: cap logic takes over
        }
        let (mut lo, mut hi) = (OFFSET_LO, OFFSET_HI);
        while hi - lo > 1 {
            let mid = lo + (hi - lo) / 2;
            if self.build(specs, psy_specs, mid) <= budget {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        hi
    }

    /// Quantize + plan both channels at `offset`; returns total frame bits.
    /// Long windows: psy thresholds come from `psy_specs` (the pre-TNS
    /// snapshot, M/S-transformed) while `specs` (the coded, TNS-filtered
    /// spectra) feed peaks and quantization.
    pub(super) fn build(
        &mut self,
        specs: &[[f32; LONG_WINDOW_LEN]; 2],
        psy_specs: &[[f32; LONG_WINDOW_LEN]; 2],
        offset: i32,
    ) -> usize {
        let standalone = self.channels == 1;
        let mut total = 3 + 4 + 3; // element id + tag + END
        if self.seq.is_eight_short() {
            if self.channels == 2 {
                total += 1 + 15 + self.ms.overhead_bits(); // cw + ics_info + ms_mask
            }
            for (ch, spec) in specs.iter().enumerate().take(self.channels) {
                total += enc_short::channel_build(
                    &mut self.psy_short,
                    self.short_offsets,
                    spec,
                    offset,
                    &mut self.chans_s[ch],
                    &mut self.target_q_s[ch],
                    &mut self.books_s[ch],
                    &mut self.gains[ch],
                    standalone,
                );
            }
            return total + 7; // byte-align pad ceiling
        }
        if self.channels == 2 {
            // common_window + ics_info + ms_mask
            total += 1 + 11 + self.ms.overhead_bits();
        }
        let channels = self
            .chans
            .iter_mut()
            .zip(self.target_q.iter_mut())
            .zip(specs.iter())
            .zip(psy_specs.iter())
            .zip(self.tns.iter())
            .zip(self.books.iter_mut())
            .zip(self.gains.iter_mut())
            .take(self.channels);
        for ((((((q, tq), spec), psy_spec), tns), books), gain) in channels {
            self.psy
                .analyze(psy_spec, self.offsets, TARGET_Q, &mut q.coded, tq);
            // TNS whitens its span flat: every band in it carries residual
            // energy the decoder's recursion needs — force them coded.
            if let Some((tns_lo, tns_hi)) = tns.band_range() {
                let span = tq.iter_mut().enumerate().take(tns_hi.min(q.n_bands));
                for (b, tq_b) in span.skip(tns_lo) {
                    if !q.coded[b] {
                        q.coded[b] = true;
                        *tq_b = TARGET_Q;
                    }
                }
            }
            let mut peaks = [0.0f32; MAX_BANDS];
            enc_quant::band_peaks(spec, self.offsets, &mut peaks);
            enc_quant::raw_scalefactors(&peaks, tq, offset, q);
            *gain = enc_quant::normalize_sf(q);
            enc_quant::quantize(spec, self.offsets, q);
            *books = enc_section::plan_books(q);
            total += enc_section::channel_body_bits(books, q, *gain, standalone, tns);
        }
        total + 7 // byte-align pad ceiling
    }

    /// Hard cap: drop the highest coded bands until the frame fits
    /// `limit_bits` (the bit budget, and never above the ADTS payload
    /// limit). Quality collapse is legal, an oversize frame is not.
    pub(super) fn drop_bands_until(&mut self, limit_bits: usize) {
        let cap = limit_bits.min(MAX_PAYLOAD_BYTES * 8);
        if self.seq.is_eight_short() {
            enc_short::drop_bands_until(
                &mut self.chans_s[..],
                &mut self.books_s[..],
                &self.gains,
                self.channels,
                cap,
                self.ms.overhead_bits(),
            );
            return;
        }
        let standalone = self.channels == 1;
        loop {
            let mut total = 3 + 4 + 3 + 7;
            if self.channels == 2 {
                total += 1 + 11 + self.ms.overhead_bits();
            }
            let channels = self
                .chans
                .iter()
                .zip(self.books.iter())
                .zip(self.gains.iter())
                .zip(self.tns.iter())
                .take(self.channels);
            for (((q, books), gain), tns) in channels {
                total += enc_section::channel_body_bits(books, q, *gain, standalone, tns);
            }
            if total <= cap {
                return;
            }
            // Find the highest coded band across channels and silence it.
            let mut hit: Option<(usize, usize)> = None;
            for (ch, q) in self.chans.iter().enumerate().take(self.channels) {
                if let Some(b) = q.coded[..q.n_bands].iter().rposition(|&c| c) {
                    hit = Some((ch, b));
                    break;
                }
            }
            let Some((ch, b)) = hit else { return };
            self.chans[ch].coded[b] = false;
            self.books[ch][b] = 0;
            self.books[ch] = enc_section::plan_books(&self.chans[ch]);
        }
    }

    /// Emit the `raw_data_block` from the built state.
    pub(super) fn emit(&self) -> Vec<u8> {
        if self.seq.is_eight_short() {
            return enc_short::emit_frame(
                self.short_offsets,
                &self.chans_s[..],
                &self.books_s[..],
                &self.gains,
                &self.ms,
                self.channels,
            );
        }
        enc_section::emit_frame(
            self.offsets,
            self.seq,
            &self.chans[..],
            &self.books[..],
            &self.gains,
            &self.ms,
            &self.tns,
            self.channels,
        )
    }
}
