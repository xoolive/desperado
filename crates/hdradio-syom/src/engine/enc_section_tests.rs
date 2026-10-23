//! Section planning + emission, parsed back by the shipped section/sf/sf_tab
//! machinery.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::bits::{BitReader, BitWriter};
use super::super::ics::{IcsInfo, WindowSequence};
use super::super::section::SectionData;
use super::super::sf::{self, ScaleFactors};
use super::super::swb::long_offsets;
use super::{channel_body_bits, emit_channel_body, emit_ics_info, emit_section_data, plan_books};
use crate::engine::enc_quant::QuantChannel;

/// A channel with alternating loud/quiet bands so planning is non-trivial.
fn sample_channel(n_bands: usize) -> QuantChannel {
    let mut q = QuantChannel::new(n_bands);
    for b in 0..n_bands {
        q.coded[b] = b % 3 != 2;
        q.sf[b] = 100 + (b as i32 % 7) * 8;
        if q.coded[b] {
            for (cb, slot) in q.bits[b].iter_mut().enumerate().skip(1) {
                *slot = (20 + b * 3 + cb * cb) as u32;
            }
        }
    }
    q
}

#[test]
fn plan_books_never_picks_unrepresentable() {
    let q = sample_channel(49);
    let books = plan_books(&q);
    for (b, (&book, &coded)) in books.iter().zip(q.coded.iter()).enumerate().take(49) {
        if coded {
            let cb = usize::from(book);
            assert!((1..=11).contains(&cb));
            assert_ne!(
                q.bits[b][cb],
                crate::engine::enc_quant::UNREPRESENTABLE,
                "band {b} planned on an unrepresentable book"
            );
        } else {
            assert_eq!(book, 0);
        }
    }
}

#[test]
fn section_data_parses_back() {
    let q = sample_channel(49);
    let books = plan_books(&q);
    let mut w = BitWriter::new();
    emit_ics_info(&mut w, WindowSequence::OnlyLong, 49);
    emit_section_data(&mut w, &books, 49);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let ics = IcsInfo::parse(&mut br, 3, false).expect("ics_info");
    assert_eq!(ics.max_sfb, 49);
    assert!(!ics.window_sequence.is_eight_short());
    let sections = SectionData::parse(&mut br, &ics).expect("section_data");
    assert_eq!(sections.sfb_cb.len(), 1, "long window: one group");
    assert_eq!(&sections.sfb_cb[0], &books[..49]);
}

#[test]
fn long_run_escapes_at_31() {
    // All bands one book: a single 49-band run must use the 31 escape.
    let mut q = QuantChannel::new(49);
    for b in 0..49 {
        q.coded[b] = true;
        q.sf[b] = 120;
        q.bits[b] = [crate::engine::enc_quant::UNREPRESENTABLE; 12];
        q.bits[b][3] = 10; // book 3 cheapest everywhere
    }
    let books = plan_books(&q);
    assert!(books[..49].iter().all(|&b| b == 3));
    let mut w = BitWriter::new();
    emit_section_data(&mut w, &books, 49);
    let bytes = w.finish();
    let mut w2 = BitWriter::new();
    emit_ics_info(&mut w2, WindowSequence::OnlyLong, 49);
    let ics_bytes = w2.finish();
    let mut br2 = BitReader::new(&ics_bytes);
    let ics = IcsInfo::parse(&mut br2, 3, false).expect("ics");
    let mut br3 = BitReader::new(&bytes);
    let sections = SectionData::parse(&mut br3, &ics).expect("sections");
    assert_eq!(sections.sfb_cb[0], vec![3u8; 49]);
}

#[test]
fn channel_body_bits_match_emit() {
    let offsets = long_offsets(3).expect("offsets");
    let mut q = sample_channel(49);
    // Real quant values so the spectral walk is exercised.
    for (i, v) in q.quant.iter_mut().enumerate() {
        *v = ((i * 7) % 5) as i32 - 2;
    }
    for b in 0..49 {
        if q.coded[b] {
            // Recompute the cost table against the actual tuples.
            for cb in 1..12usize {
                let step = if cb <= 4 { 4 } else { 2 };
                let (lo, hi) = (usize::from(offsets[b]), usize::from(offsets[b + 1]));
                let mut total = 0u32;
                let mut i = lo;
                let mut ok = true;
                while i < hi {
                    match crate::engine::enc_huff::spectral_bits(cb as u8, &q.quant[i..i + step]) {
                        Some(bits) => total += bits as u32,
                        None => {
                            ok = false;
                            break;
                        }
                    }
                    i += step;
                }
                q.bits[b][cb] = if ok {
                    total
                } else {
                    crate::engine::enc_quant::UNREPRESENTABLE
                };
            }
        }
    }
    let books = plan_books(&q);
    let mut w = BitWriter::new();
    emit_channel_body(
        &mut w,
        offsets,
        WindowSequence::OnlyLong,
        &books,
        &q,
        100,
        true,
        &crate::engine::enc_tns::EncTns::off(),
    );
    let counted = channel_body_bits(
        &books,
        &q,
        100,
        true,
        &crate::engine::enc_tns::EncTns::off(),
    );
    assert_eq!(
        w.bit_len() as usize,
        counted,
        "counted bits must match emitted bits exactly"
    );
    // And the emitted body must parse: sf DPCM over the same plan.
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let gg = br.read(8).expect("gg") as u8;
    assert_eq!(gg, 100);
    let ics = IcsInfo::parse(&mut br, 3, false).expect("ics");
    let sections = SectionData::parse(&mut br, &ics).expect("sections");
    let mut sf = ScaleFactors::default();
    sf::parse_into(&mut br, &ics, &sections.sfb_cb, gg, &mut sf).expect("sf");
    for b in 0..49 {
        if q.coded[b] {
            assert_eq!(sf.sf[0][b], q.sf[b], "band {b} scalefactor");
        }
    }
}
