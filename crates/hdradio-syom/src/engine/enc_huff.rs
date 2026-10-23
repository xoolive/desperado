//! Spectral Huffman encode — the inverse of [`super::huff`]
//! (ISO/IEC 14496-3 §4.6.3, Tables 4.A.2–4.A.12) plus the Table 4.A.1
//! scalefactor DPCM code from [`super::sf_tab`].

use super::bits::BitWriter;
use super::sf_tab::{SF_CODE, SF_LEN};
use super::{huff_esc, huff_pair, huff_quad};

/// `(len, code)` tables for spectral book `cb` (1..=11).
fn tables(cb: u8) -> Option<(&'static [u8], &'static [u16])> {
    match cb {
        1 => Some((&huff_quad::H1_LEN, &huff_quad::H1_CODE)),
        2 => Some((&huff_quad::H2_LEN, &huff_quad::H2_CODE)),
        3 => Some((&huff_quad::H3_LEN, &huff_quad::H3_CODE)),
        4 => Some((&huff_quad::H4_LEN, &huff_quad::H4_CODE)),
        5 => Some((&huff_pair::H5_LEN, &huff_pair::H5_CODE)),
        6 => Some((&huff_pair::H6_LEN, &huff_pair::H6_CODE)),
        7 => Some((&huff_pair::H7_LEN, &huff_pair::H7_CODE)),
        8 => Some((&huff_pair::H8_LEN, &huff_pair::H8_CODE)),
        9 => Some((&huff_pair::H9_LEN, &huff_pair::H9_CODE)),
        10 => Some((&huff_pair::H10_LEN, &huff_pair::H10_CODE)),
        11 => Some((&huff_esc::H11_LEN, &huff_esc::H11_CODE)),
        _ => None,
    }
}

/// `true` for the unsigned books (3, 4, 7..=11) that trail sign bits.
fn unsigned_book(cb: u8) -> bool {
    matches!(cb, 3 | 4 | 7 | 8 | 9 | 10 | 11)
}

/// Ordinal of tuple `v` in book `cb`, or `None` when the tuple is not
/// representable (magnitude above the book's LAV; book 11 escapes any
/// magnitude). Inverses of the decoders in [`super::huff`].
pub fn tuple_index(cb: u8, v: &[i32]) -> Option<usize> {
    let in_range = |limit: i32| v.iter().all(|&x| x.abs() <= limit);
    match cb {
        1 | 2 if v.len() == 4 && in_range(1) => {
            Some(((v[0] + 1) * 27 + (v[1] + 1) * 9 + (v[2] + 1) * 3 + (v[3] + 1)) as usize)
        }
        3 | 4 if v.len() == 4 && in_range(2) => {
            Some((v[0].abs() * 27 + v[1].abs() * 9 + v[2].abs() * 3 + v[3].abs()) as usize)
        }
        5 | 6 if v.len() == 2 && in_range(4) => Some(((v[0] + 4) * 9 + (v[1] + 4)) as usize),
        7 | 8 if v.len() == 2 && in_range(7) => Some((v[0].abs() * 8 + v[1].abs()) as usize),
        9 | 10 if v.len() == 2 && in_range(12) => Some((v[0].abs() * 13 + v[1].abs()) as usize),
        11 if v.len() == 2 => {
            let a = v[0].abs().min(16);
            let b = v[1].abs().min(16);
            Some((a * 17 + b) as usize)
        }
        _ => None,
    }
}

/// Escape-sequence bit count for one magnitude ≥ 16 (§4.6.3.3):
/// `n` = floor(log2(mag)) ≥ 4, then `n − 4` one-bits, a zero stop bit, and
/// the `n`-bit offset.
fn esc_bits(mag: u32) -> usize {
    let n = 31 - mag.leading_zeros() as usize; // floor(log2(mag)), mag ≥ 16 → n ≥ 4
    (n - 4) + 1 + n
}

/// Total bits to emit tuple `v` under book `cb`, or `None` when the book
/// cannot represent it.
pub fn spectral_bits(cb: u8, v: &[i32]) -> Option<usize> {
    let idx = tuple_index(cb, v)?;
    let (len, _) = tables(cb)?;
    let mut bits = usize::from(*len.get(idx)?);
    if unsigned_book(cb) {
        bits += v.iter().filter(|&&x| x != 0).count();
    }
    if cb == 11 {
        for &x in v {
            let mag = x.unsigned_abs();
            if mag >= 16 {
                bits += esc_bits(mag);
            }
        }
    }
    Some(bits)
}

/// Emit tuple `v` under book `cb`: codeword, then per-nonzero sign bits
/// (unsigned books), then book-11 escape sequences — the exact order
/// [`super::huff::decode_tuple`] reads them. Unrepresentable tuples are a
/// caller bug; nothing is written.
pub fn spectral_emit(cb: u8, v: &[i32], w: &mut BitWriter) {
    let Some(idx) = tuple_index(cb, v) else {
        return;
    };
    let Some((len, code)) = tables(cb) else {
        return;
    };
    let Some((&l, &c)) = len.get(idx).zip(code.get(idx)) else {
        return;
    };
    w.write(u32::from(c), u32::from(l));
    if unsigned_book(cb) {
        for &x in v {
            if x != 0 {
                w.write_bit(x < 0);
            }
        }
    }
    if cb == 11 {
        for &x in v {
            let mag = x.unsigned_abs();
            if mag >= 16 {
                let n = 31 - mag.leading_zeros(); // ≥ 4
                for _ in 4..n {
                    w.write_bit(true);
                }
                w.write_bit(false);
                w.write(mag - (1 << n), n);
            }
        }
    }
}

/// Bits for one scalefactor DPCM delta (Table 4.A.1; clamped to ±60, the
/// codebook's range).
pub fn sf_delta_bits(delta: i32) -> usize {
    usize::from(SF_LEN[(delta.clamp(-60, 60) + 60) as usize])
}

/// Emit one scalefactor DPCM delta (clamped to ±60).
pub fn sf_emit_delta(w: &mut BitWriter, delta: i32) {
    let idx = (delta.clamp(-60, 60) + 60) as usize;
    w.write(SF_CODE[idx], u32::from(SF_LEN[idx]));
}

#[cfg(test)]
#[path = "enc_huff_tests.rs"]
mod enc_huff_tests;
