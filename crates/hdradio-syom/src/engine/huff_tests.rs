//! Huffman 1–11 and Table 4.A.1 scalefactor codebook.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::bits::{BitReader, BitWriter};
use super::huff;
use super::sf::decode_dpcm;

fn emit(len: u8, code: u32) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.write(code, u32::from(len));
    w.finish()
}

#[test]
fn book1_zero_tuple_is_single_bit_zero() -> Result<(), super::Error> {
    // Table 4.A.2 index 40 = (0,0,0,0), 1-bit `0`.
    let bytes = emit(1, 0);
    let mut br = BitReader::new(&bytes);
    let mut out = [0i32; 4];
    let n = huff::decode_tuple(&mut br, 1, &mut out)?;
    assert_eq!(n, 4);
    assert_eq!(out, [0, 0, 0, 0]);
    Ok(())
}

#[test]
fn book1_roundtrip_index_zero() -> Result<(), super::Error> {
    // Index 0 = (−1,−1,−1,−1). Length/code from Table 4.A.2.
    let (len, code) = (super::huff_quad::H1_LEN[0], super::huff_quad::H1_CODE[0]);
    let bytes = emit(len, u32::from(code));
    let mut br = BitReader::new(&bytes);
    let mut out = [0i32; 4];
    huff::decode_tuple(&mut br, 1, &mut out)?;
    assert_eq!(out, [-1, -1, -1, -1]);
    Ok(())
}

#[test]
fn scalefactor_delta_zero_is_one_bit() -> Result<(), super::Error> {
    // Table 4.A.1 index 60 (delta 0) is the single bit `0`.
    let bytes = emit(1, 0);
    let mut br = BitReader::new(&bytes);
    assert_eq!(decode_dpcm(&mut br)?, 0);
    Ok(())
}

fn decode_cb(cb: u8, idx: usize, signs: &[bool]) -> Result<(usize, [i32; 4], u64), super::Error> {
    let (len, code) = match cb {
        1 => (
            super::huff_quad::H1_LEN[idx],
            super::huff_quad::H1_CODE[idx],
        ),
        2 => (
            super::huff_quad::H2_LEN[idx],
            super::huff_quad::H2_CODE[idx],
        ),
        3 => (
            super::huff_quad::H3_LEN[idx],
            super::huff_quad::H3_CODE[idx],
        ),
        4 => (
            super::huff_quad::H4_LEN[idx],
            super::huff_quad::H4_CODE[idx],
        ),
        5 => (
            super::huff_pair::H5_LEN[idx],
            super::huff_pair::H5_CODE[idx],
        ),
        6 => (
            super::huff_pair::H6_LEN[idx],
            super::huff_pair::H6_CODE[idx],
        ),
        7 => (
            super::huff_pair::H7_LEN[idx],
            super::huff_pair::H7_CODE[idx],
        ),
        8 => (
            super::huff_pair::H8_LEN[idx],
            super::huff_pair::H8_CODE[idx],
        ),
        9 => (
            super::huff_pair::H9_LEN[idx],
            super::huff_pair::H9_CODE[idx],
        ),
        10 => (
            super::huff_pair::H10_LEN[idx],
            super::huff_pair::H10_CODE[idx],
        ),
        11 => (
            super::huff_esc::H11_LEN[idx],
            super::huff_esc::H11_CODE[idx],
        ),
        _ => unreachable!(),
    };
    let mut w = BitWriter::new();
    w.write(u32::from(code), u32::from(len));
    for &s in signs {
        w.write_bit(s);
    }
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let mut out = [0i32; 4];
    let n = huff::decode_tuple(&mut br, cb, &mut out)?;
    Ok((n, out, br.bit_position()))
}

#[test]
fn books_1_through_11_exact_tuples() -> Result<(), super::Error> {
    // Signed quads (1–2): index maps to −1..=1, no extra bits.
    let (_, t, pos) = decode_cb(2, 0, &[])?;
    assert_eq!(&t[..4], &[-1, -1, -1, -1]);
    assert_eq!(pos, u64::from(super::huff_quad::H2_LEN[0]));

    // Unsigned quads (3–4): magnitudes then one sign per non-zero.
    let (_, t, pos) = decode_cb(3, 1, &[true])?;
    assert_eq!(&t[..4], &[0, 0, 0, -1]);
    assert_eq!(pos, u64::from(super::huff_quad::H3_LEN[1]) + 1);

    let (_, t, pos) = decode_cb(4, 1, &[false])?;
    assert_eq!(&t[..4], &[0, 0, 0, 1]);
    assert_eq!(pos, u64::from(super::huff_quad::H4_LEN[1]) + 1);

    // Signed pairs (5–6).
    let (_, t, pos) = decode_cb(5, 0, &[])?;
    assert_eq!(&t[..2], &[-4, -4]);
    assert_eq!(pos, u64::from(super::huff_pair::H5_LEN[0]));

    let (_, t, _) = decode_cb(6, 40, &[])?;
    assert_eq!(&t[..2], &[0, 0]);

    // Unsigned pairs (7–10).
    let (_, t, pos) = decode_cb(7, 1, &[true])?;
    assert_eq!(&t[..2], &[0, -1]);
    assert_eq!(pos, u64::from(super::huff_pair::H7_LEN[1]) + 1);

    let (_, t, pos) = decode_cb(8, 1, &[false])?;
    assert_eq!(&t[..2], &[0, 1]);
    assert_eq!(pos, u64::from(super::huff_pair::H8_LEN[1]) + 1);

    let (_, t, pos) = decode_cb(9, 1, &[true])?;
    assert_eq!(&t[..2], &[0, -1]);
    assert_eq!(pos, u64::from(super::huff_pair::H9_LEN[1]) + 1);

    let (_, t, pos) = decode_cb(10, 1, &[false])?;
    assert_eq!(&t[..2], &[0, 1]);
    assert_eq!(pos, u64::from(super::huff_pair::H10_LEN[1]) + 1);

    // Book 11 unsigned pair, no ESC.
    let (_, t, pos) = decode_cb(11, 1, &[true])?;
    assert_eq!(&t[..2], &[0, -1]);
    assert_eq!(pos, u64::from(super::huff_esc::H11_LEN[1]) + 1);
    Ok(())
}

#[test]
fn book11_signs_before_escape() -> Result<(), super::Error> {
    // ISO/IEC 14496-3 §4.6.3.3: book 11 is unsigned. After the Huffman
    // pair, sign bits for each non-zero (including the 16 flag), then
    // escape_sequence for each magnitude-16. Encoder (lavc) writes the
    // same order. Reversing it desyncs the next channel.
    // Pair (16, 0): idx = 16*17 = 272. Sign=1 → negative. ESC n=4, off=0 → 16.
    let idx = 16 * 17;
    let (len, code) = (
        super::huff_esc::H11_LEN[idx],
        super::huff_esc::H11_CODE[idx],
    );
    let mut w = BitWriter::new();
    w.write(u32::from(code), u32::from(len));
    w.write_bit(true); // sign of 16
    w.write_bit(false); // ESC prefix ends (n stays 4)
    w.write(0, 4); // offset → magnitude 16
    w.write(0b1010, 4); // must remain unread
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let mut out = [0i32; 4];
    huff::decode_tuple(&mut br, 11, &mut out)?;
    assert_eq!(out[0], -16);
    assert_eq!(out[1], 0);
    let expect = u64::from(len) + 1 + 1 + 4;
    assert_eq!(
        br.bit_position(),
        expect,
        "sign-then-ESC must consume {expect} bits"
    );
    Ok(())
}
