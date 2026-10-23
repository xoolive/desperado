//! enc_huff emit → huff decode roundtrips (decoder-as-oracle), exhaustive
//! over every book's magnitude tuples plus book-11 escapes, and the SF DPCM
//! code over its full ±60 range.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::bits::{BitReader, BitWriter};
use super::super::huff::decode_tuple;
use super::super::sf::decode_dpcm;
use super::{sf_delta_bits, sf_emit_delta, spectral_bits, spectral_emit, tuple_index};

/// Emit `tuples` under `cb`, decode them back, assert identity.
fn roundtrip(cb: u8, dim: usize, tuples: &[Vec<i32>]) {
    let mut w = BitWriter::new();
    let mut declared = 0usize;
    for t in tuples {
        let bits = spectral_bits(cb, t).expect("representable");
        declared += bits;
        spectral_emit(cb, t, &mut w);
    }
    assert_eq!(
        w.bit_len() as usize,
        declared,
        "cb {cb}: spectral_bits disagrees with the emit"
    );
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let mut out = [0i32; 4];
    for t in tuples {
        let n = decode_tuple(&mut br, cb, &mut out).expect("decode");
        assert_eq!(n, dim, "cb {cb} tuple dim");
        assert_eq!(&out[..dim], t.as_slice(), "cb {cb} tuple {t:?}");
    }
}

/// All magnitude tuples for `cb`, with a deterministic sign pattern.
fn all_tuples(cb: u8, dim: usize, lav: i32) -> Vec<Vec<i32>> {
    let mut out = vec![vec![]];
    for _ in 0..dim {
        let mut next = Vec::new();
        for prefix in &out {
            for m in 0..=lav {
                let mut t = prefix.clone();
                // Deterministic signs: negate when (m + position sum) is odd.
                let sign = if (m + prefix.iter().sum::<i32>()) % 2 == 0 {
                    1
                } else {
                    -1
                };
                t.push(m * sign);
                next.push(t);
            }
        }
        out = next;
    }
    let _ = cb;
    out
}

#[test]
fn signed_quad_books_roundtrip() {
    for cb in [1u8, 2] {
        roundtrip(cb, 4, &all_tuples(cb, 4, 1));
    }
}

#[test]
fn unsigned_quad_books_roundtrip() {
    for cb in [3u8, 4] {
        roundtrip(cb, 4, &all_tuples(cb, 4, 2));
    }
}

#[test]
fn pair_books_roundtrip() {
    for (cb, lav) in [(5u8, 4), (6, 4), (7, 7), (8, 7), (9, 12), (10, 12)] {
        roundtrip(cb, 2, &all_tuples(cb, 2, lav));
    }
}

#[test]
fn esc_book_roundtrip_including_escapes() {
    let mut tuples = all_tuples(11, 2, 16);
    // Escape territory: magnitudes ≥ 16 use escape_sequence.
    for &m in &[16, 17, 31, 32, 100, 255, 1024, 8191] {
        tuples.push(vec![m, -3]);
        tuples.push(vec![-m, m]);
        tuples.push(vec![0, m]);
    }
    roundtrip(11, 2, &tuples);
}

#[test]
fn out_of_range_tuples_are_rejected() {
    assert_eq!(tuple_index(1, &[2, 0, 0, 0]), None);
    assert_eq!(tuple_index(3, &[0, 3, 0, 0]), None);
    assert_eq!(tuple_index(5, &[5, 0]), None);
    assert_eq!(tuple_index(7, &[0, 8]), None);
    assert_eq!(tuple_index(9, &[13, 0]), None);
    assert!(tuple_index(11, &[8191, -8191]).is_some());
    assert_eq!(spectral_bits(2, &[0, 0, 0, 2]), None);
}

#[test]
fn sf_dpcm_full_range_roundtrip() {
    let mut w = BitWriter::new();
    let mut declared = 0usize;
    for delta in -60..=60 {
        declared += sf_delta_bits(delta);
        sf_emit_delta(&mut w, delta);
    }
    assert_eq!(w.bit_len() as usize, declared);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    for delta in -60..=60i8 {
        let got = decode_dpcm(&mut br).expect("sf decode");
        assert_eq!(got, delta);
    }
}
