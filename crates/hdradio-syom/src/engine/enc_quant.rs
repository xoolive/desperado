//! Forward quantization, per-band scalefactors, and codebook cost table —
//! the inverse of [`super::spectrum::invquant`] / [`super::spectrum::sf_gain`]
//! (ISO/IEC 14496-3 §4.6.2–4.6.3).

use super::det_math;
use super::enc_huff::spectral_bits;
use super::spectrum::SF_OFFSET;
use super::swb::{LONG_WINDOW_LEN, SHORT_WINDOW_LEN};

/// Largest quantized magnitude the bitstream carries (§4.6.2).
pub const QUANT_MAX: i32 = 8191;
/// Spectral books 1..=11; slot 0 is unused.
pub const BOOKS: usize = 12;
/// Long-window band ceiling (the 32 kHz table has 51 bands).
pub const MAX_BANDS: usize = 51;
/// `bits` sentinel: book cannot represent the band.
pub const UNREPRESENTABLE: u32 = u32::MAX;

/// One quantized long-window channel.
#[derive(Clone)]
pub struct QuantChannel {
    pub quant: [i32; 1024],
    /// `bits[band][book]` — spectral bits under each book, or
    /// [`UNREPRESENTABLE`].
    pub bits: [[u32; BOOKS]; MAX_BANDS],
    /// Absolute scalefactors actually quantized with (the transmitted ones).
    pub sf: [i32; MAX_BANDS],
    /// Bands with signal worth coding (the rest get ZERO_HCB).
    pub coded: [bool; MAX_BANDS],
    pub n_bands: usize,
}

impl QuantChannel {
    pub fn new(n_bands: usize) -> Self {
        Self {
            quant: [0; 1024],
            bits: [[UNREPRESENTABLE; BOOKS]; MAX_BANDS],
            sf: [0; MAX_BANDS],
            coded: [false; MAX_BANDS],
            n_bands,
        }
    }
}

/// Short-window band ceiling (the 8 kHz table has 15 bands).
pub const MAX_SFB_SHORT: usize = 15;
/// Window groups in a short frame: v1 emits no `scale_factor_grouping`, so
/// each of the 8 short windows is its own group (simplest conformant
/// grouping — sections and scalefactors cost a few extra bits per frame,
/// the spectral walk stays contiguous per window).
pub const MAX_GROUPS: usize = 8;
/// Flattened (group, band) ceiling: index `g * n_sfb + b`.
pub const MAX_FLAT_SHORT: usize = MAX_GROUPS * MAX_SFB_SHORT;

/// One quantized short-window channel: 8 windows × 128 bins, window-major,
/// with books / scalefactors per (group, band) flattened as `g * n_sfb + b`
/// (group `g` is window `g` — no grouping).
#[derive(Clone)]
pub struct QuantShort {
    pub quant: [i32; LONG_WINDOW_LEN],
    /// `bits[g * n_sfb + band][book]`.
    pub bits: [[u32; BOOKS]; MAX_FLAT_SHORT],
    /// Absolute scalefactors actually quantized with (the transmitted ones).
    pub sf: [i32; MAX_FLAT_SHORT],
    /// Bands with signal worth coding (the rest get ZERO_HCB).
    pub coded: [bool; MAX_FLAT_SHORT],
    /// Scalefactor bands per window (the rate's short-table size).
    pub n_sfb: usize,
}

impl QuantShort {
    pub fn new(n_sfb: usize) -> Self {
        Self {
            quant: [0; LONG_WINDOW_LEN],
            bits: [[UNREPRESENTABLE; BOOKS]; MAX_FLAT_SHORT],
            sf: [0; MAX_FLAT_SHORT],
            coded: [false; MAX_FLAT_SHORT],
            n_sfb,
        }
    }
}

/// Per-band peak |coefficient| of a 1024-line long-window spectrum.
pub fn band_peaks(spec: &[f32; 1024], offsets: &[u16], out: &mut [f32; MAX_BANDS]) {
    let n_bands = offsets.len() - 1;
    for b in 0..n_bands {
        let lo = usize::from(offsets[b]);
        let hi = usize::from(offsets[b + 1]);
        out[b] = spec[lo..hi].iter().map(|x| x.abs()).fold(0.0f32, f32::max);
    }
}

/// Desired scalefactor for a band peak so the peak quantizes to about
/// `target_q`: `q = |x|^0.75 · 2^(−0.1875·(sf−100))` solved for `sf`. The
/// log2s are [`det_math`](super::det_math)'s: libm `log2` is not bit-identical
/// across platforms, and a 1-ulp drift can flip the rounded scalefactor.
pub fn sf_for_peak(peak: f32, target_q: f32) -> i32 {
    if peak <= 0.0 || target_q <= 0.0 {
        return 0;
    }
    let sf =
        SF_OFFSET as f32 + 4.0 * det_math::log2(peak) - (16.0 / 3.0) * det_math::log2(target_q);
    sf.round() as i32
}

/// Raw per-band scalefactors from peaks and the psy model's per-band
/// precision targets. `out.coded` is set by the psy model (a coded band
/// always has `target_q >= 1` and a nonzero peak).
pub fn raw_scalefactors(
    peaks: &[f32; MAX_BANDS],
    target_q: &[f32; MAX_BANDS],
    global_offset: i32,
    out: &mut QuantChannel,
) {
    let bands = peaks
        .iter()
        .zip(target_q.iter())
        .zip(out.coded.iter())
        .zip(out.sf.iter_mut())
        .take(out.n_bands);
    for (((&peak, &tq), &coded), sf) in bands {
        *sf = if coded && tq >= 1.0 {
            sf_for_peak(peak, tq) + global_offset
        } else {
            0
        };
    }
}

/// Clamp `out.sf` onto the wire format: global_gain is 8-bit, DPCM deltas
/// are ±60. Returns `global_gain`. Values actually transmitted are written
/// back into `out.sf` so quantization matches what the decoder applies.
pub fn normalize_sf(out: &mut QuantChannel) -> u8 {
    let mut global_gain = 100u8;
    let mut prev: Option<i32> = None;
    let bands = out.coded.iter().zip(out.sf.iter_mut()).take(out.n_bands);
    for (&coded, sf) in bands {
        if !coded {
            continue;
        }
        let want = (*sf).clamp(0, 255);
        let v = match prev {
            None => {
                global_gain = want as u8;
                want
            }
            Some(p) => (p + (want - p).clamp(-60, 60)).clamp(0, 255),
        };
        *sf = v;
        prev = Some(v);
    }
    global_gain
}

/// Quantize `spec` with the (normalized) `out.sf` and fill the per-band,
/// per-book bit-cost table.
pub fn quantize(spec: &[f32; 1024], offsets: &[u16], out: &mut QuantChannel) {
    out.quant = [0; 1024];
    let n_bands = out.n_bands.min(offsets.len() - 1);
    let bands = out
        .coded
        .iter()
        .zip(out.sf.iter())
        .zip(out.bits.iter_mut())
        .zip(offsets.windows(2))
        .take(n_bands);
    for (((&coded, &sf), bits), w) in bands {
        let (lo, hi) = (usize::from(w[0]), usize::from(w[1]));
        if !coded {
            *bits = [UNREPRESENTABLE; BOOKS];
            continue;
        }
        // Deterministic gain + |x|^0.75 (det_math): libm exp2/powf are not
        // bit-identical across platforms and would drift the encoded bytes.
        let gain = det_math::exp2(-0.1875f32 * (sf - SF_OFFSET) as f32);
        for (q, &x) in out.quant[lo..hi].iter_mut().zip(spec[lo..hi].iter()) {
            let mag = (det_math::pow_three_quarter(x) * gain).round() as i32;
            let mag = mag.min(QUANT_MAX);
            *q = if x < 0.0 { -mag } else { mag };
        }
        fill_bits(bits, &out.quant, lo, hi);
    }
}

/// Cost of one band under every book: quad books 1–4 walk 4-tuples, pair
/// books 5–11 walk pairs (long-window band widths are multiples of 4).
fn fill_bits(bits: &mut [u32; BOOKS], quant: &[i32; 1024], lo: usize, hi: usize) {
    for (cb, slot) in bits.iter_mut().enumerate().skip(1) {
        let step = if cb <= 4 { 4 } else { 2 };
        let mut total = 0u32;
        let mut i = lo;
        while i < hi {
            let tuple = &quant[i..i + step];
            let Some(n) = spectral_bits(cb as u8, tuple) else {
                total = UNREPRESENTABLE;
                break;
            };
            total = total.saturating_add(n as u32);
            i += step;
        }
        *slot = total;
    }
}

/// Per-(window, band) peak |coefficient| of a window-major short spectrum,
/// flattened as `w * n_sfb + band`.
pub fn band_peaks_short(
    spec: &[f32; LONG_WINDOW_LEN],
    offsets: &[u16],
    n_sfb: usize,
    out: &mut [f32; MAX_FLAT_SHORT],
) {
    for w in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            let lo = w * SHORT_WINDOW_LEN + usize::from(offsets[b]);
            let hi = w * SHORT_WINDOW_LEN + usize::from(offsets[b + 1]);
            out[w * n_sfb + b] = spec[lo..hi].iter().map(|x| x.abs()).fold(0.0f32, f32::max);
        }
    }
}

/// Raw per-(window, band) scalefactors, short-window twin of
/// [`raw_scalefactors`] (`out.coded` is set by the psy model).
pub fn raw_scalefactors_short(
    peaks: &[f32; MAX_FLAT_SHORT],
    target_q: &[f32; MAX_FLAT_SHORT],
    global_offset: i32,
    out: &mut QuantShort,
) {
    let n = MAX_GROUPS * out.n_sfb;
    let bands = peaks
        .iter()
        .zip(target_q.iter())
        .zip(out.coded.iter())
        .zip(out.sf.iter_mut())
        .take(n);
    for (((&peak, &tq), &coded), sf) in bands {
        *sf = if coded && tq >= 1.0 {
            sf_for_peak(peak, tq) + global_offset
        } else {
            0
        };
    }
}

/// Clamp `out.sf` onto the wire format, short windows: the DPCM chain runs
/// over the flattened (group, band) order — the decoder's `last_sf`
/// continues across groups (ISO/IEC 14496-3 Table 4.53).
pub fn normalize_sf_short(out: &mut QuantShort) -> u8 {
    let mut global_gain = 100u8;
    let mut prev: Option<i32> = None;
    let n = MAX_GROUPS * out.n_sfb;
    let bands = out.coded.iter().zip(out.sf.iter_mut()).take(n);
    for (&coded, sf) in bands {
        if !coded {
            continue;
        }
        let want = (*sf).clamp(0, 255);
        let v = match prev {
            None => {
                global_gain = want as u8;
                want
            }
            Some(p) => (p + (want - p).clamp(-60, 60)).clamp(0, 255),
        };
        *sf = v;
        prev = Some(v);
    }
    global_gain
}

/// Quantize a window-major short spectrum with the (normalized) `out.sf`
/// and fill the per-(window, band) bit-cost table.
pub fn quantize_short(spec: &[f32; LONG_WINDOW_LEN], offsets: &[u16], out: &mut QuantShort) {
    out.quant = [0; LONG_WINDOW_LEN];
    for w in 0..MAX_GROUPS {
        for b in 0..out.n_sfb {
            let idx = w * out.n_sfb + b;
            let lo = w * SHORT_WINDOW_LEN + usize::from(offsets[b]);
            let hi = w * SHORT_WINDOW_LEN + usize::from(offsets[b + 1]);
            if !out.coded[idx] {
                out.bits[idx] = [UNREPRESENTABLE; BOOKS];
                continue;
            }
            // Same deterministic gain + |x|^0.75 as the long path.
            let gain = det_math::exp2(-0.1875f32 * (out.sf[idx] - SF_OFFSET) as f32);
            for (q, &x) in out.quant[lo..hi].iter_mut().zip(spec[lo..hi].iter()) {
                let mag = (det_math::pow_three_quarter(x) * gain).round() as i32;
                let mag = mag.min(QUANT_MAX);
                *q = if x < 0.0 { -mag } else { mag };
            }
            fill_bits(&mut out.bits[idx], &out.quant, lo, hi);
        }
    }
}

#[cfg(test)]
#[path = "enc_quant_tests.rs"]
mod enc_quant_tests;
