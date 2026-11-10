//! Quantizer / scalefactor-normalization unit tests.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::swb::long_offsets;
use super::{
    MAX_BANDS, QuantChannel, band_peaks, normalize_sf, quantize, raw_scalefactors, sf_for_peak,
};

#[test]
fn sf_for_peak_hits_target() {
    for &peak in &[1.0f32, 100.0, 1e4, 1e6, 3e7] {
        for &tq in &[256.0f32, 2048.0, 4095.0] {
            let sf = sf_for_peak(peak, tq);
            let q = peak.powf(0.75) * (-0.1875f32 * (sf - 100) as f32).exp2();
            let ratio = q / tq;
            assert!(
                (0.85..=1.15).contains(&ratio),
                "peak {peak} tq {tq}: sf {sf} gives q {q}"
            );
        }
    }
}

#[test]
fn normalize_sf_respects_wire_bounds() {
    // Deterministic pseudo-random raw scalefactors, including wild swings.
    let mut state = 0x1234_5678u32;
    let mut rng = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 8) as i32
    };
    for _ in 0..50 {
        let n_bands = 49;
        let mut q = QuantChannel::new(n_bands);
        for b in 0..n_bands {
            q.coded[b] = rng() % 4 != 0;
            q.sf[b] = (rng() % 400) - 80;
        }
        let gg = normalize_sf(&mut q);
        let mut prev = i32::from(gg);
        let mut first = true;
        for b in 0..n_bands {
            if !q.coded[b] {
                continue;
            }
            let sf = q.sf[b];
            assert!((0..=255).contains(&sf), "sf {sf} out of 8-bit range");
            if first {
                assert_eq!(sf, i32::from(gg), "first coded sf is global_gain");
                first = false;
            } else {
                assert!(
                    (sf - prev).abs() <= 60,
                    "DPCM delta {} exceeds ±60",
                    sf - prev
                );
            }
            prev = sf;
        }
    }
}

#[test]
fn silent_band_quantizes_to_zero() {
    let offsets = long_offsets(3).expect("48k long offsets");
    let n_bands = offsets.len() - 1;
    let mut q = QuantChannel::new(n_bands);
    let mut peaks = [0.0f32; MAX_BANDS];
    peaks[10] = 1e6; // one live band, rest silent
    let tq = [2048.0f32; MAX_BANDS];
    q.coded[10] = true; // the psy model's decision, mocked
    raw_scalefactors(&peaks, &tq, 0, &mut q);
    let gg = normalize_sf(&mut q);
    let mut spec = [0.0f32; 1024];
    spec[usize::from(offsets[10])] = 1e6;
    quantize(&spec, offsets, &mut q);
    assert!(q.quant[..usize::from(offsets[10])].iter().all(|&v| v == 0));
    assert!(q.quant[usize::from(offsets[11])..].iter().all(|&v| v == 0));
    let peak_q = q.quant[usize::from(offsets[10])].abs();
    assert!(peak_q > 1000, "peak band quantized to {peak_q}");
    let _ = gg;
}

#[test]
fn band_peaks_tracks_spectrum() {
    let offsets = long_offsets(3).expect("48k long offsets");
    let mut spec = [0.0f32; 1024];
    spec[100] = -7.5;
    spec[600] = 3.0;
    let mut peaks = [0.0f32; MAX_BANDS];
    band_peaks(&spec, offsets, &mut peaks);
    let b100 = offsets.iter().position(|&o| o > 100).expect("band") - 1;
    assert_eq!(peaks[b100], 7.5);
    assert_eq!(peaks[0], 0.0);
}
