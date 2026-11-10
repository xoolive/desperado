//! TNS (Temporal Noise Shaping) analysis for the LC encoder — the forward
//! twin of [`super::tns`] (ISO/IEC 14496-3 §4.6.9, Tables 4.54 / 4.103).
//!
//! TNS runs an all-pole LPC analysis over a span of the long-window MDCT
//! spectrum and quantizes the prediction *residual*: the decoder's inverse
//! filter then shapes the quantization noise after the signal's time
//! envelope inside the frame. What fires it is intra-frame envelope
//! movement (tremolo-like content); steady tones and sweeps barely whiten
//! (their KBD-MDCT spectra are already near-white along frequency — lavc
//! keeps TNS off there too).
//!
//! v1 design:
//!
//! - Long-family frames only (OnlyLong / LongStart / LongStop). Short
//!   frames keep `tns_data_present = 0` (TODO: order ≤ 7 short windows).
//! - The filter span is the coded-band span reported by the psy model on
//!   the *original* spectrum, `[first_coded, last_coded+1)` clamped to
//!   `tns_max_bands` — never the whole table. A whitening filter is
//!   unity-gain on average (Σ log|A| = 0), so it *boosts* the quiet
//!   sidelobe bands around a notch it carves; any band inside the filter
//!   span the quantizer drops would inject that boosted residual error
//!   straight into the decoder's recursion. The build therefore force-codes
//!   every band in the span (`enc_frame_rate`), and the span is emitted as
//!   up to two filters: an order-0 spacer (no-op — skips the uncovered top
//!   bands) plus the active filter. Direction upward, `coef_res = 4` bits,
//!   no coefficient compression.
//! - Order: Levinson–Durbin to the long-window maximum (12), trimmed above
//!   the last reflection coefficient with `|k| >= K_MIN`.
//! - Gate: the filter is emitted only when the *quantized* filter whitens
//!   the span by at least [`PG_MIN`] (2 dB) — silence and noise-like
//!   spectra stay off.
//!
//! Pipeline placement: the decoder applies the TNS inverse *after* the M/S
//! undo (`decode_cpe`), so the analysis filter runs on the raw L/R spectra
//! *before* [`super::enc_ms`] — the mirror of the decoder's tool order.
//! The psy model's coded-band decisions and masking thresholds come from
//! the *original* (pre-TNS) spectrum (`enc_frame` keeps a snapshot and
//! replays the M/S transform onto it): whitening drops the loudest band by
//! the prediction gain, which would sink the −60 dB coded-band floor with
//! it and spend the budget on boosted sidelobe bands. Peaks and
//! quantization run on the filtered spectra, so scalefactors adapt to the
//! residual.
//!
//! Determinism: the decision path is `+ - * / sqrt` (IEEE-exact) plus
//! [`det_math`](super::det_math) transcendentals — no libm — so TNS keeps
//! the encoder's cross-platform byte-exactness.

use super::bits::BitWriter;
use super::det_math;
use super::enc_quant::MAX_BANDS;
use super::ics::WindowSequence;
use super::swb::LONG_WINDOW_LEN;

/// All-pole order ceiling for long windows (Table 4.54; the decoder clamps
/// long-window orders to 12).
pub const MAX_ORDER: usize = 12;
/// Trailing reflection coefficients under this magnitude extend the filter
/// without measurable gain — the order is trimmed above them.
const K_MIN: f64 = 0.1;
/// Prediction-gain gate (linear, ≈ 2 dB): TNS is emitted only when the
/// quantized filter whitens the covered range by at least this factor.
const PG_MIN: f64 = 1.585;
/// `tns_max_bands` without PQF, long windows — mirror of the decoder's
/// private `MAX_BANDS_LONG` ([`super::tns`], Table 4.103). Index 12
/// (7350 Hz) has no entry there (its parse errors), so TNS stays off.
const TNS_MAX_BANDS_LONG: [u8; 12] = [31, 31, 34, 40, 42, 51, 46, 46, 42, 42, 42, 39];

/// One channel's TNS decision for a frame; `on == false` emits only the
/// `tns_data_present = 0` flag bit.
#[derive(Debug, Clone, Copy)]
pub struct EncTns {
    on: bool,
    order: u8,
    /// Active filter span in scalefactor bands `[start, end)`.
    start: u8,
    end: u8,
    /// Wire `length` of the order-0 spacer filter skipping the uncovered
    /// top bands; 0 when the active filter reaches the TNS band ceiling
    /// (then the decoder's clamp ends it — no spacer needed).
    spacer: u8,
    /// Wire `length` of the active filter (bands counted down from the
    /// spacer's bottom / from `num_swb`).
    length: u8,
    /// 4-bit wire coefficients (two's complement in the low nibble).
    coef: [u8; MAX_ORDER],
}

impl EncTns {
    /// TNS off for this channel.
    pub fn off() -> Self {
        Self {
            on: false,
            order: 0,
            start: 0,
            end: 0,
            spacer: 0,
            length: 0,
            coef: [0; MAX_ORDER],
        }
    }

    /// `true` when a filter is emitted.
    #[cfg(test)]
    pub fn is_on(&self) -> bool {
        self.on
    }

    /// The active filter's scalefactor band span when on — every band in
    /// it must be coded (the residual carries real energy across the whole
    /// whitened span; a dropped band feeds zeros into the decoder's
    /// recursion).
    pub fn band_range(&self) -> Option<(usize, usize)> {
        if self.on {
            Some((usize::from(self.start), usize::from(self.end)))
        } else {
            None
        }
    }

    /// Bits occupied in the channel body: the present flag plus, when on,
    /// the Table 4.54 payload (one window; order-0 spacer + active filter).
    pub fn bits(&self) -> usize {
        if !self.on {
            return 1;
        }
        // n_filt(2) + coef_res(1) + [spacer: length(6) + order(5)] +
        // length(6) + order(5) + direction(1) + coef_compress(1) + 4·order.
        let spacer = if self.spacer > 0 { 6 + 5 } else { 0 };
        1 + 2 + 1 + spacer + 6 + 5 + 2 + 4 * self.order as usize
    }

    /// Emit `tns_data_present` and, when on, the `tns_data()` payload — the
    /// exact mirror of [`super::tns::TnsData::parse`] (long windows: 2-bit
    /// `n_filt`, 6-bit `length`, 5-bit `order`).
    pub fn emit(&self, w: &mut BitWriter) {
        w.write_bit(self.on);
        if !self.on {
            return;
        }
        w.write(if self.spacer > 0 { 2 } else { 1 }, 2); // n_filt
        w.write_bit(true); // coef_res: 4-bit
        if self.spacer > 0 {
            w.write(u32::from(self.spacer), 6);
            w.write(0, 5); // order 0: no-op spacer
        }
        w.write(u32::from(self.length), 6);
        w.write(u32::from(self.order), 5);
        w.write_bit(false); // direction: upward
        w.write_bit(false); // coef_compress
        for &c in self.coef.iter().take(self.order as usize) {
            w.write(u32::from(c), 4);
        }
    }
}

/// Autocorrelation of `x` at lags 0..=MAX_ORDER (f64 sums, fixed order).
fn autocorrelate(x: &[f32]) -> [f64; MAX_ORDER + 1] {
    let mut r = [0.0f64; MAX_ORDER + 1];
    for (lag, slot) in r.iter_mut().enumerate() {
        let mut acc = 0.0f64;
        for i in 0..x.len() - lag {
            acc += f64::from(x[i]) * f64::from(x[i + lag]);
        }
        *slot = acc;
    }
    r
}

/// Levinson–Durbin on `r`: reflection coefficients (`k[0]` is k₁). `None`
/// when the recursion breaks down (`|k| >= 1` — degenerate spectra). The
/// recursion matches the decoder's `decode_lpc` step-up, so emitting the
/// quantized `k`s makes the decoder rebuild the same direct-form filter.
fn levinson(r: &[f64; MAX_ORDER + 1]) -> Option<[f64; MAX_ORDER]> {
    let mut k = [0.0f64; MAX_ORDER];
    let mut a = [0.0f64; MAX_ORDER];
    let mut e = r[0];
    for m in 1..=MAX_ORDER {
        let mut acc = r[m];
        for i in 1..m {
            acc += a[i - 1] * r[m - i];
        }
        let km = -acc / e;
        if !(-1.0..1.0).contains(&km) {
            return None;
        }
        k[m - 1] = km;
        let old = a;
        for i in 1..m {
            a[i - 1] = old[i - 1] + km * old[m - i - 1];
        }
        a[m - 1] = km;
        e *= 1.0 - km * km;
    }
    Some(k)
}

/// Quantize one reflection coefficient to its 4-bit wire value — the
/// inverse of `decode_lpc`'s `sin(t / iqfac)` reconstruction (4-bit,
/// uncompressed: `t >= 0` divides by 7.5/(π/2), `t < 0` by 8.5/(π/2)).
/// `asin(k) = atan(k/√(1−k²))` — [`det_math`] + IEEE `sqrt` only.
fn quantize_coef(k: f64) -> u8 {
    let k = (k as f32).clamp(-0.999, 0.999);
    let angle = det_math::atan(k / (1.0 - k * k).sqrt()) * std::f32::consts::FRAC_2_PI;
    let t = if k >= 0.0 {
        (angle * 7.5).round() as i32
    } else {
        (angle * 8.5).round() as i32
    };
    (t.clamp(-8, 7) & 0xF) as u8
}

/// Dequantize a 4-bit wire coefficient — the decoder's `sin(t / iqfac)`
/// through [`det_math`] (f32; ~1 ulp from the decoder's f64 `sin`, far
/// inside the golden tolerance, and libm-free for the decision path).
fn dequantize_coef(wire: u8) -> f32 {
    let t = (i32::from(wire) << 28) >> 28; // sign-extend the low nibble
    let scaled = if t >= 0 {
        t as f32 / 7.5
    } else {
        t as f32 / 8.5
    };
    let s = det_math::sincos(scaled.abs() * std::f32::consts::FRAC_PI_2).0;
    if scaled < 0.0 { -s } else { s }
}

/// Step-up from quantized reflection coefficients to the direct-form
/// A(z) = 1 + Σ aₖ z⁻ᵏ — the decoder's `decode_lpc` recursion on the
/// dequantized coefficients, so the analysis filter below is the exact
/// inverse of what the decoder applies.
fn step_up(kq: &[f32]) -> [f64; MAX_ORDER] {
    let mut a = [0.0f64; MAX_ORDER];
    for (m0, &km) in kq.iter().enumerate() {
        let m = m0 + 1;
        let k = f64::from(km);
        let old = a;
        for i in 1..m {
            a[i - 1] = old[i - 1] + k * old[m - i - 1];
        }
        a[m - 1] = k;
    }
    a
}

/// Analysis (whitening) filter, upward direction: `e[n] = x[n] + Σ aₖ·x[n−k]`.
/// The decoder's `ar_filter` computes `y = x − Σ aₖ·y[n−k]`, i.e.
/// X = Y·A(z); this applies A(z). f64 accumulator, fixed op order.
fn analysis_filter(x: &mut [f32], orig: &[f32], a: &[f64]) {
    for (n, slot) in x.iter_mut().enumerate() {
        let mut e = f64::from(orig[n]);
        for (j, &ak) in a.iter().enumerate() {
            if n > j {
                e += ak * f64::from(orig[n - j - 1]);
            }
        }
        *slot = e as f32;
    }
}

/// Decide TNS for one long-family channel spectrum, filtering it in place
/// when the gate passes. `offsets` is the rate's long-window swb table;
/// `coded` is the psy model's coded-band mask on this original spectrum —
/// the filter span is the coded span (module docs).
pub fn decide_long(
    spec: &mut [f32; LONG_WINDOW_LEN],
    offsets: &[u16],
    fs_index: u8,
    coded: &[bool],
) -> EncTns {
    let n_swb = offsets.len() - 1;
    let Some(&max_bands) = TNS_MAX_BANDS_LONG.get(fs_index as usize) else {
        return EncTns::off();
    };
    let max_bands = n_swb.min(usize::from(max_bands));
    let coded = &coded[..n_swb.min(coded.len())];
    let Some(first) = coded.iter().position(|&c| c) else {
        return EncTns::off(); // silence / nothing coded
    };
    let Some(last) = coded.iter().rposition(|&c| c) else {
        return EncTns::off();
    };
    let start = first.min(max_bands);
    let end = (last + 1).min(max_bands);
    let (lo, hi) = (usize::from(offsets[start]), usize::from(offsets[end]));
    if hi - lo <= MAX_ORDER {
        return EncTns::off();
    }
    let r = autocorrelate(&spec[lo..hi]);
    if r[0] <= 0.0 {
        return EncTns::off();
    }
    let Some(k) = levinson(&r) else {
        return EncTns::off();
    };
    let Some(order) = (1..=MAX_ORDER).rev().find(|&m| k[m - 1].abs() >= K_MIN) else {
        return EncTns::off(); // flat spectrum: no prediction gain to buy
    };
    // The decoder walks filters down from `num_swb` and clamps each filter's
    // top to `tns_max_bands`: when the span reaches the ceiling the active
    // filter alone suffices (`length = num_swb - start`); otherwise an
    // order-0 spacer first skips the uncovered top bands.
    let (spacer, length) = if end == max_bands {
        (0u8, (n_swb - start) as u8)
    } else {
        ((n_swb - end) as u8, (end - start) as u8)
    };
    let mut out = EncTns {
        on: true,
        order: order as u8,
        start: start as u8,
        end: end as u8,
        spacer,
        length,
        coef: [0; MAX_ORDER],
    };
    let mut kq = [0.0f32; MAX_ORDER];
    for i in 0..order {
        out.coef[i] = quantize_coef(k[i]);
        kq[i] = dequantize_coef(out.coef[i]);
    }
    let a = step_up(&kq[..order]);
    // Gate on the quantized filter's measured prediction gain over the span.
    let mut filtered = [0.0f32; LONG_WINDOW_LEN];
    filtered[lo..hi].copy_from_slice(&spec[lo..hi]);
    analysis_filter(&mut filtered[lo..hi], &spec[lo..hi], &a[..order]);
    let (mut e_in, mut e_out) = (0.0f64, 0.0f64);
    for i in lo..hi {
        e_in += f64::from(spec[i]) * f64::from(spec[i]);
        e_out += f64::from(filtered[i]) * f64::from(filtered[i]);
    }
    if e_in < PG_MIN * e_out {
        return EncTns::off();
    }
    spec[lo..hi].copy_from_slice(&filtered[lo..hi]);
    out
}

/// Per-frame entry: TNS analysis on the long-family spectra of `channels`
/// channels, in place, before the M/S decision. `coded` holds each
/// channel's psy coded-band mask on the original (pre-TNS) spectrum.
/// Short frames and the test-only A/B off switch keep every channel off.
#[allow(clippy::too_many_arguments)]
pub fn decide_frame(
    specs: &mut [[f32; LONG_WINDOW_LEN]; 2],
    channels: usize,
    seq: WindowSequence,
    offsets: &[u16],
    fs_index: u8,
    coded: &[[bool; MAX_BANDS]; 2],
    enabled: bool,
) -> [EncTns; 2] {
    if !enabled || seq.is_eight_short() {
        return [EncTns::off(), EncTns::off()];
    }
    let mut out = [EncTns::off(), EncTns::off()];
    for ch in 0..channels {
        out[ch] = decide_long(&mut specs[ch], offsets, fs_index, &coded[ch]);
    }
    out
}

#[cfg(test)]
#[path = "enc_tns_tests.rs"]
mod enc_tns_tests;
