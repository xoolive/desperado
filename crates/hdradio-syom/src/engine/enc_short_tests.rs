//! Short-window section planning + emission, parsed back by the shipped
//! ics/section/sf/spectrum machinery (the exact decode path our bits hit).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::bits::{BitReader, BitWriter};
use super::super::enc_quant::{MAX_GROUPS, QuantShort, UNREPRESENTABLE};
use super::super::ics::IcsInfo;
use super::super::section::SectionData;
use super::super::sf::{self, ScaleFactors};
use super::super::spectrum;
use super::super::swb::short_offsets;
use super::{channel_body_bits_short, emit_channel_body_short, plan_books_short};

/// A short channel with alternating loud/quiet bands per window so planning
/// is non-trivial, and real quant values so the spectral walk is exercised.
fn sample_short_channel(n_sfb: usize) -> QuantShort {
    let mut q = QuantShort::new(n_sfb);
    for g in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            let idx = g * n_sfb + b;
            q.coded[idx] = (g + b) % 3 != 2;
            q.sf[idx] = 90 + ((g * 5 + b) as i32 % 9) * 6;
        }
    }
    for (i, v) in q.quant.iter_mut().enumerate() {
        *v = ((i * 7) % 5) as i32 - 2;
    }
    q
}

/// Recompute the cost table against the actual tuples (like `fill_bits`).
fn fill_costs(q: &mut QuantShort, offsets: &[u16]) {
    for w in 0..MAX_GROUPS {
        for b in 0..q.n_sfb {
            let idx = w * q.n_sfb + b;
            if !q.coded[idx] {
                continue;
            }
            let lo = w * 128 + usize::from(offsets[b]);
            let hi = w * 128 + usize::from(offsets[b + 1]);
            for cb in 1..12usize {
                let step = if cb <= 4 { 4 } else { 2 };
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
                q.bits[idx][cb] = if ok { total } else { UNREPRESENTABLE };
            }
        }
    }
}

#[test]
fn short_channel_body_parses_back() {
    let offsets = short_offsets(3).expect("48k short offsets");
    let n_sfb = offsets.len() - 1;
    let mut q = sample_short_channel(n_sfb);
    fill_costs(&mut q, offsets);
    let books = plan_books_short(&q);
    // Planning stays per group and never picks an unrepresentable book.
    for (idx, (&book, &coded)) in books.iter().zip(q.coded.iter()).enumerate() {
        if coded {
            let cb = usize::from(book);
            assert!((1..=11).contains(&cb), "flat band {idx}: bad book {cb}");
            assert_ne!(q.bits[idx][cb], UNREPRESENTABLE, "flat band {idx}");
        } else {
            assert_eq!(book, 0);
        }
    }
    let mut w = BitWriter::new();
    emit_channel_body_short(&mut w, offsets, &books, &q, 100, true);
    let counted = channel_body_bits_short(&books, &q, 100, true);
    assert_eq!(
        w.bit_len() as usize,
        counted,
        "counted bits must match emitted bits exactly"
    );
    // Parse the body back through the shipped decode path.
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let gg = br.read(8).expect("gg") as u8;
    assert_eq!(gg, 100);
    let ics = IcsInfo::parse(&mut br, 3, false).expect("ics_info");
    assert!(ics.window_sequence.is_eight_short());
    assert_eq!(ics.num_windows, 8);
    assert_eq!(ics.num_window_groups, 8, "v1: no scale_factor_grouping");
    assert!(ics.window_group_length.iter().all(|&l| l == 1));
    assert_eq!(ics.max_sfb as usize, n_sfb);
    let sections = SectionData::parse(&mut br, &ics).expect("section_data");
    assert_eq!(sections.sfb_cb.len(), 8);
    for g in 0..MAX_GROUPS {
        assert_eq!(
            &sections.sfb_cb[g][..],
            &books[g * n_sfb..(g + 1) * n_sfb],
            "group {g} codebooks"
        );
    }
    let mut sfs = ScaleFactors::default();
    sf::parse_into(&mut br, &ics, &sections.sfb_cb, gg, &mut sfs).expect("sf");
    for g in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            if q.coded[g * n_sfb + b] {
                assert_eq!(sfs.sf[g][b], q.sf[g * n_sfb + b], "g{g} band {b} sf");
            }
        }
    }
    // The spectral walk: skipped flags, then the grouped quant parse.
    assert!(!br.read_bit().expect("pulse"));
    assert!(!br.read_bit().expect("tns"));
    assert!(!br.read_bit().expect("gain"));
    let mut quant = Vec::new();
    spectrum::parse_quant_into(&mut br, &ics, &sections, 3, &mut quant).expect("quant");
    assert_eq!(quant.len(), 1024);
    for g in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            let idx = g * n_sfb + b;
            let lo = g * 128 + usize::from(offsets[b]);
            let hi = g * 128 + usize::from(offsets[b + 1]);
            if q.coded[idx] {
                assert_eq!(
                    &quant[lo..hi],
                    &q.quant[lo..hi],
                    "g{g} band {b} coefficients"
                );
            } else {
                assert!(
                    quant[lo..hi].iter().all(|&v| v == 0),
                    "g{g} band {b} uncoded but nonzero"
                );
            }
        }
    }
}

#[test]
fn short_sf_dpcm_continues_across_groups() {
    // All bands coded with a rising sf ramp: the decoder must accumulate
    // one continuous DPCM chain across the 8 groups (not restart per group).
    let offsets = short_offsets(3).expect("48k short offsets");
    let n_sfb = offsets.len() - 1;
    let mut q = QuantShort::new(n_sfb);
    for i in 0..MAX_GROUPS * n_sfb {
        q.coded[i] = true;
        q.sf[i] = 80 + i as i32 % 40;
    }
    for (i, v) in q.quant.iter_mut().enumerate() {
        *v = ((i * 3) % 3) as i32 - 1;
    }
    fill_costs(&mut q, offsets);
    let books = plan_books_short(&q);
    let mut w = BitWriter::new();
    emit_channel_body_short(&mut w, offsets, &books, &q, 100, true);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let gg = br.read(8).expect("gg") as u8;
    let ics = IcsInfo::parse(&mut br, 3, false).expect("ics");
    let sections = SectionData::parse(&mut br, &ics).expect("sections");
    let mut sfs = ScaleFactors::default();
    sf::parse_into(&mut br, &ics, &sections.sfb_cb, gg, &mut sfs).expect("sf");
    for g in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            assert_eq!(sfs.sf[g][b], q.sf[g * n_sfb + b], "g{g} band {b}");
        }
    }
}
