//! TNS — ISO/IEC 14496-3 §4.6.9 Tables 4.54 / 4.102 / 4.103.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::ics::{IcsInfo, WindowSequence};
use super::swb::{long_offsets, short_offsets};

/// Table 4.103 without PQF, long windows, fs 0..=11.
const MAX_BANDS_LONG: [u8; 12] = [31, 31, 34, 40, 42, 51, 46, 46, 42, 42, 42, 39];
/// Table 4.103 without PQF, short windows.
const MAX_BANDS_SHORT: [u8; 12] = [9, 9, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14];

/// One TNS filter (Table 4.54).
#[derive(Debug, Clone)]
pub struct TnsFilter {
    /// Scalefactor bands covered, counted down from the previous top.
    pub length: u8,
    /// All-pole order (clamped later by TNS_MAX_ORDER).
    pub order: u8,
    /// `true` = downward (inc = −1).
    pub direction: bool,
    /// Shrinks `coef` field by one bit.
    pub coef_compress: bool,
    /// Unsigned wire coefficients.
    pub coef: Vec<u8>,
}

/// Per-window TNS payload.
#[derive(Debug, Clone)]
pub struct TnsWindow {
    /// `coef_res`: false → 3-bit, true → 4-bit.
    pub coef_res: bool,
    /// Filters in wire order (top of spectrum first).
    pub filters: Vec<TnsFilter>,
}

/// Parsed `tns_data()`.
#[derive(Debug, Clone)]
pub struct TnsData {
    /// One slot per transform window.
    pub windows: Vec<TnsWindow>,
}

impl TnsData {
    /// Table 4.54.
    pub fn parse(br: &mut BitReader<'_>, ics: &IcsInfo) -> Result<Self> {
        let short = ics.window_sequence.is_eight_short();
        let n_filt_bits = if short { 1 } else { 2 };
        let len_bits = if short { 4 } else { 6 };
        let order_bits = if short { 3 } else { 5 };
        let mut windows = Vec::with_capacity(ics.num_windows as usize);
        for _ in 0..ics.num_windows {
            let n_filt = br.read(n_filt_bits)? as usize;
            let mut coef_res = false;
            let mut filters = Vec::with_capacity(n_filt);
            if n_filt > 0 {
                coef_res = br.read_bit()?;
            }
            for _ in 0..n_filt {
                let length = br.read(len_bits)? as u8;
                let order = br.read(order_bits)? as u8;
                let mut direction = false;
                let mut coef_compress = false;
                let mut coef = Vec::new();
                if order > 0 {
                    direction = br.read_bit()?;
                    coef_compress = br.read_bit()?;
                    let coef_res_bits = if coef_res { 4 } else { 3 };
                    let coef_bits = coef_res_bits - u32::from(coef_compress);
                    for _ in 0..order {
                        coef.push(br.read(coef_bits)? as u8);
                    }
                }
                filters.push(TnsFilter {
                    length,
                    order,
                    direction,
                    coef_compress,
                    coef,
                });
            }
            windows.push(TnsWindow { coef_res, filters });
        }
        Ok(TnsData { windows })
    }

    /// Parse HDC TNS. Long HDC windows implicitly carry one filter instead of
    /// the ordinary two-bit `n_filt` field.
    pub(crate) fn parse_hdc(br: &mut BitReader<'_>, ics: &IcsInfo) -> Result<Self> {
        let short = ics.window_sequence.is_eight_short();
        let len_bits = if short { 4 } else { 6 };
        let order_bits = if short { 3 } else { 5 };
        let mut windows = Vec::with_capacity(ics.num_windows as usize);
        for _ in 0..ics.num_windows {
            let n_filt = if short { br.read(1)? as usize } else { 1 };
            let mut coef_res = false;
            let mut filters = Vec::with_capacity(n_filt);
            if n_filt > 0 {
                coef_res = br.read_bit()?;
            }
            for _ in 0..n_filt {
                let length = br.read(len_bits)? as u8;
                let order = br.read(order_bits)? as u8;
                let mut direction = false;
                let mut coef_compress = false;
                let mut coef = Vec::new();
                if order > 0 {
                    direction = br.read_bit()?;
                    coef_compress = br.read_bit()?;
                    let coef_res_bits = if coef_res { 4 } else { 3 };
                    let coef_bits = coef_res_bits - u32::from(coef_compress);
                    for _ in 0..order {
                        coef.push(br.read(coef_bits)? as u8);
                    }
                }
                filters.push(TnsFilter {
                    length,
                    order,
                    direction,
                    coef_compress,
                    coef,
                });
            }
            windows.push(TnsWindow { coef_res, filters });
        }
        Ok(TnsData { windows })
    }
}

fn max_order(seq: WindowSequence) -> u8 {
    if seq.is_eight_short() { 7 } else { 12 }
}

fn max_bands(fs_index: u8, seq: WindowSequence) -> Result<u8> {
    let idx = fs_index as usize;
    let table = if seq.is_eight_short() {
        &MAX_BANDS_SHORT
    } else {
        &MAX_BANDS_LONG
    };
    table
        .get(idx)
        .copied()
        .ok_or(Error::UnsupportedSampleRateIndex(fs_index))
}

fn sign_extend(coef: u8, bits: u32) -> i32 {
    let s_mask = 1u8 << (bits - 1);
    if coef & s_mask != 0 {
        (coef as i32) | !((1i32 << bits) - 1)
    } else {
        i32::from(coef)
    }
}

fn decode_lpc(order: usize, coef_res_bits: u32, compress: bool, coef: &[u8]) -> Result<Vec<f64>> {
    let coef_res2 = coef_res_bits - u32::from(compress);
    if !(2..=4).contains(&coef_res2) {
        return Err(Error::SpectrumInvalid);
    }
    let iqfac = ((1u32 << (coef_res_bits - 1)) as f64 - 0.5) / (std::f64::consts::PI / 2.0);
    let iqfac_m = ((1u32 << (coef_res_bits - 1)) as f64 + 0.5) / (std::f64::consts::PI / 2.0);
    let mut tmp2 = vec![0.0f64; order];
    if coef.len() < order {
        return Err(Error::SpectrumInvalid);
    }
    for (slot, &c) in tmp2.iter_mut().zip(coef.iter().take(order)) {
        let t = sign_extend(c, coef_res2);
        *slot = (t as f64 / if t >= 0 { iqfac } else { iqfac_m }).sin();
    }
    let mut a = vec![0.0f64; order + 1];
    a[0] = 1.0;
    for m in 1..=order {
        let mut b = vec![0.0f64; m];
        for i in 1..m {
            b[i] = a[i] + tmp2[m - 1] * a[m - i];
        }
        a[1..m].copy_from_slice(&b[1..m]);
        a[m] = tmp2[m - 1];
    }
    Ok(a)
}

fn ar_filter(spec: &mut [f32], start: usize, size: usize, inc: i32, lpc: &[f64]) {
    let order = lpc.len().saturating_sub(1);
    if size == 0 || order == 0 {
        return;
    }
    let mut hist = [0.0f64; 12];
    let mut pos = 0usize;
    let mut idx = start as isize;
    for _ in 0..size {
        let x = f64::from(spec[idx as usize]);
        let mut y = x;
        let mut hidx = pos;
        for &coeff in lpc.iter().take(order + 1).skip(1) {
            if hidx == 0 {
                hidx = order;
            }
            hidx -= 1;
            y -= coeff * hist[hidx];
        }
        hist[pos] = y;
        spec[idx as usize] = y as f32;
        pos += 1;
        if pos == order {
            pos = 0;
        }
        idx += inc as isize;
    }
}

/// §4.6.9.3 `tns_decode_frame` on a window-major spectrum.
pub fn apply(spec: &mut [f32], tns: &TnsData, ics: &IcsInfo, fs_index: u8) -> Result<()> {
    let win_len = ics.window_len();
    let offsets = if ics.window_sequence.is_eight_short() {
        short_offsets(fs_index)?
    } else {
        long_offsets(fs_index)?
    };
    let tns_max_order = max_order(ics.window_sequence) as usize;
    let tns_max_bands = max_bands(fs_index, ics.window_sequence)? as usize;
    let max_sfb = ics.max_sfb as usize;
    for (w, win) in tns.windows.iter().enumerate() {
        let mut bottom = ics.num_swb as usize;
        for filt in &win.filters {
            let top = bottom;
            bottom = top.saturating_sub(filt.length as usize);
            let order = (filt.order as usize).min(tns_max_order);
            if order == 0 {
                continue;
            }
            let coef_res_bits = if win.coef_res { 4 } else { 3 };
            let lpc = decode_lpc(order, coef_res_bits, filt.coef_compress, &filt.coef)?;
            let start_sfb = bottom.min(tns_max_bands).min(max_sfb);
            let end_sfb = top.min(tns_max_bands).min(max_sfb);
            let start = *offsets.get(start_sfb).unwrap_or(&0) as usize;
            let end = *offsets.get(end_sfb).unwrap_or(&0) as usize;
            if end <= start {
                continue;
            }
            let size = end - start;
            let base = w * win_len;
            let (inc, origin) = if filt.direction {
                (-1i32, base + end - 1)
            } else {
                (1i32, base + start)
            };
            ar_filter(spec, origin, size, inc, &lpc);
        }
    }
    Ok(())
}
