//! Per-band M/S decision + bitmask emission, parsed back by the shipped
//! `ics` / `stereo::MsInfo` machinery and decoded by the shipped decoder.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::bits::{BitReader, BitWriter};
use super::super::decode::StreamDecoder;
use super::super::error::Result;
use super::super::ics::{IcsInfo, WindowSequence};
use super::super::stereo::{MsInfo, MsMask};
use super::super::swb::{LONG_WINDOW_LEN, long_offsets, short_offsets};
use super::{MaskForm, MsBands, decide_long};
use crate::engine::enc_frame::LcEncoder;

/// Parse the CPE header of one `raw_data_block` payload: element id, tag,
/// `common_window`, `ics_info`, then the M/S mask.
fn parse_ms(payload: &[u8], fs_index: u8) -> Result<(IcsInfo, MsInfo, u64)> {
    let mut br = BitReader::new(payload);
    let id = br.read(3)?;
    assert_eq!(id, 1, "stereo frame must start with a CPE");
    br.read(4)?; // tag
    assert!(br.read_bit()?, "common_window");
    let ics = IcsInfo::parse(&mut br, fs_index, true)?;
    let pre = br.bit_position();
    let ms = MsInfo::parse(&mut br, &ics)?;
    Ok((ics, ms, br.bit_position() - pre))
}

/// Deterministic LCG noise frame, decorrelated per `seed`.
fn noise_frame(t: usize, seed: u32, amp: f32) -> Vec<f32> {
    let mut state = seed.wrapping_add((t as u32).wrapping_mul(2_654_435_761));
    (0..LONG_WINDOW_LEN)
        .map(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            amp * (((state >> 9) as f32 / (1u32 << 23) as f32) * 2.0 - 1.0)
        })
        .collect()
}

/// Split-spectrum frame: channels share a 200 Hz sine (correlated lows) but
/// carry independent high-frequency sine beds (decorrelated highs).
fn split_frame(t: usize, hf_amp: f32) -> (Vec<f32>, Vec<f32>) {
    let mut l = Vec::with_capacity(LONG_WINDOW_LEN);
    let mut r = Vec::with_capacity(LONG_WINDOW_LEN);
    for i in 0..LONG_WINDOW_LEN {
        let n = (t * LONG_WINDOW_LEN + i) as f32;
        let low = 0.4 * (2.0 * std::f32::consts::PI * 200.0 * n / 48_000.0).sin();
        let mut hl = 0.0f32;
        let mut hr = 0.0f32;
        for k in 0..16 {
            let f = 8_000.0 + 500.0 * k as f32;
            let w = 2.0 * std::f32::consts::PI * f / 48_000.0;
            hl += (w * n + 0.7 * k as f32).sin();
            hr += (w * n + 0.7 * k as f32 + 1.9).sin();
        }
        l.push(low + hf_amp * hl / 4.0);
        r.push(low + hf_amp * hr / 4.0);
    }
    (l, r)
}

#[test]
fn correlated_pair_decides_all_ms() -> Result<()> {
    // Same content in both channels (scaled): every loud band prefers M/S
    // and the quiet-band fold keeps the 2-bit whole-pair mask.
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    for t in 0..4 {
        let l = noise_frame(t, 0x1234_5678, 0.3);
        let r: Vec<f32> = l.iter().map(|&x| x * 0.9).collect();
        let payload = enc.encode_frame(&[&l, &r])?;
        if t < 2 {
            continue; // priming
        }
        let (_, ms, mask_bits) = parse_ms(&payload, enc.fs_index())?;
        assert_eq!(ms.mask, MsMask::All, "frame {t}: correlated must be All");
        assert_eq!(mask_bits, 2, "whole-pair mask costs 2 bits");
        let mut dec = StreamDecoder::new();
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 2, 1, &payload)?;
        assert_eq!(frame.planar.len(), 2);
    }
    Ok(())
}

#[test]
fn decorrelated_pair_decides_off() -> Result<()> {
    // Independent noise with the right channel 20 dB down: even perfect
    // band covariance cannot satisfy 3·e_side < min(e_l, e_r), so every
    // band stays L/R and the mask collapses to Off. (Equal-energy
    // decorrelated content sits closer to the border: a narrow band whose
    // instantaneous covariance happens high may genuinely go M/S — that is
    // the metric finding real correlation, and it is cost-neutral.)
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    for t in 0..4 {
        let l = noise_frame(t, 0x0BAD_F00D, 0.3);
        let r = noise_frame(t, 0x5EED_1234, 0.03);
        let payload = enc.encode_frame(&[&l, &r])?;
        if t < 2 {
            continue;
        }
        let (_, ms, mask_bits) = parse_ms(&payload, enc.fs_index())?;
        assert_eq!(ms.mask, MsMask::Off, "frame {t}: decorrelated must be Off");
        assert_eq!(mask_bits, 2);
    }
    Ok(())
}

#[test]
fn split_spectrum_emits_per_band_bitmask() -> Result<()> {
    let offsets = long_offsets(3)?; // 48 kHz
    let n_bands = offsets.len() - 1;
    let band_hz = |b: usize| {
        f32::from(offsets[b]) * 48_000.0 / 2048.0 // band lower edge
    };
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let mut dec = StreamDecoder::new();
    let mut saw_per_band = false;
    for t in 0..6 {
        let (l, r) = split_frame(t, 0.5);
        let payload = enc.encode_frame(&[&l, &r])?;
        if t < 2 {
            continue;
        }
        let (ics, ms, mask_bits) = parse_ms(&payload, enc.fs_index())?;
        assert!(!ics.window_sequence.is_eight_short());
        assert_eq!(ms.mask, MsMask::PerBand, "frame {t}: split must be PerBand");
        assert_eq!(mask_bits, 2 + n_bands as u64, "one bit per long band");
        saw_per_band = true;
        for b in 0..n_bands {
            if band_hz(b) >= 8_000.0 && band_hz(b) < 16_000.0 {
                assert!(!ms.used(0, b), "frame {t} band {b}: decorrelated HF is L/R");
            }
        }
        // The band holding the shared 200 Hz sine is M/S.
        let b200 = (0..n_bands)
            .find(|&b| usize::from(offsets[b]) <= 8 && 8 < usize::from(offsets[b + 1]))
            .expect("200 Hz band");
        assert!(ms.used(0, b200), "frame {t}: correlated low band is M/S");
        // The emitted mask decodes.
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 2, 1, &payload)?;
        assert_eq!(frame.planar.len(), 2);
    }
    assert!(saw_per_band);
    Ok(())
}

#[test]
fn short_window_bitmask_is_per_group() -> Result<()> {
    // Correlated 300 Hz tone bed + decorrelated broadband click: the
    // EightShort frame must emit ms_mask_present = 1 with 8 × n_sfb bits.
    let offsets = short_offsets(3)?;
    let n_sfb = offsets.len() - 1;
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let mut dec = StreamDecoder::new();
    let mut saw_short_per_band = false;
    for t in 0..6 {
        // Base: correlated 300 Hz tone; click frames add decorrelated noise.
        let mut l = Vec::with_capacity(LONG_WINDOW_LEN);
        let mut r = Vec::with_capacity(LONG_WINDOW_LEN);
        for i in 0..LONG_WINDOW_LEN {
            let n = (t * LONG_WINDOW_LEN + i) as f32;
            let v = 0.3 * (2.0 * std::f32::consts::PI * 300.0 * n / 48_000.0).sin();
            l.push(v);
            r.push(v);
        }
        if t >= 2 {
            // Independent per channel, right 20 dB down: L/R is
            // mathematically forced even for the narrow mid bands (flip
            // would need band covariance above the Cauchy bound).
            let nl = noise_frame(t, 0xC1C4_0001, 0.4);
            let nr = noise_frame(t, 0xC1C4_0002, 0.04);
            for i in 0..LONG_WINDOW_LEN {
                l[i] += nl[i];
                r[i] += nr[i];
            }
        }
        let payload = enc.encode_frame(&[&l, &r])?;
        let (ics, ms, mask_bits) = parse_ms(&payload, enc.fs_index())?;
        if ics.window_sequence != WindowSequence::EightShort {
            continue;
        }
        saw_short_per_band = true;
        assert_eq!(ics.num_window_groups, 8, "8 groups of 1 window");
        assert_eq!(ms.mask, MsMask::PerBand, "short split must be PerBand");
        assert_eq!(mask_bits, 2 + (8 * n_sfb) as u64, "8 groups × n_sfb bits");
        assert_eq!(ms.used.len(), 8);
        for g in 0..8 {
            assert!(ms.used(g, 0), "g{g}: correlated tone band is M/S");
            for (b, &off) in offsets.iter().enumerate().take(n_sfb) {
                let hz = f32::from(off) * 48_000.0 / 256.0;
                if (4_000.0..16_000.0).contains(&hz) {
                    assert!(!ms.used(g, b), "g{g} b{b}: decorrelated HF is L/R");
                }
            }
        }
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 2, 1, &payload)?;
        assert_eq!(frame.planar.len(), 2);
    }
    assert!(
        saw_short_per_band,
        "fixture must produce an EightShort frame"
    );
    Ok(())
}

#[test]
fn emit_parses_back_bit_exact() -> Result<()> {
    // Hand-built split spectrum → decide → emit → parse with the shipped
    // MsInfo: the bitmask round-trips exactly (long layout).
    let offsets = long_offsets(3)?;
    let n_bands = offsets.len() - 1;
    let mut specs = [[0.0f32; LONG_WINDOW_LEN]; 2];
    {
        let [l, r] = &mut specs;
        for (b, win) in offsets.windows(2).enumerate() {
            let (lo, hi) = (usize::from(win[0]), usize::from(win[1]));
            let bins = l.iter_mut().zip(r.iter_mut()).enumerate().take(hi).skip(lo);
            for (i, (lv, rv)) in bins {
                if b < n_bands / 2 {
                    *lv = 100.0;
                    *rv = 90.0; // correlated lows
                } else {
                    // Decorrelated-ish highs: alternating signs differ.
                    *lv = if i % 2 == 0 { 100.0 } else { -100.0 };
                    *rv = if i % 3 == 0 { 100.0 } else { -90.0 };
                }
            }
        }
    }
    let ms = decide_long(&mut specs, offsets, true);
    assert_eq!(ms.form(), MaskForm::PerBand);
    let mut w = BitWriter::new();
    ms.emit(&mut w);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let ics = IcsInfo {
        window_sequence: WindowSequence::OnlyLong,
        window_shape: super::super::ics::WindowShape::Kbd,
        max_sfb: n_bands as u8,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
        num_swb: n_bands as u8,
    };
    let parsed = MsInfo::parse(&mut br, &ics)?;
    assert_eq!(parsed.mask, MsMask::PerBand);
    for b in 0..n_bands {
        assert_eq!(parsed.used(0, b), ms.used(b), "band {b} mirror");
    }
    Ok(())
}

/// Asymmetric split: shared 200 Hz sine (correlated lows) + independent
/// HF content with the right channel 20 dB down (L/R genuinely cheaper:
/// only the left HF needs coding). `hf_amp` scales the left HF level.
fn ab_frame(t: usize, hf_amp: f32, tonal: bool) -> (Vec<f32>, Vec<f32>) {
    let mut l = Vec::with_capacity(LONG_WINDOW_LEN);
    let mut r = Vec::with_capacity(LONG_WINDOW_LEN);
    let (nl, nr) = if tonal {
        (vec![0.0f32; LONG_WINDOW_LEN], vec![0.0f32; LONG_WINDOW_LEN])
    } else {
        (
            noise_frame(t, 0xAB00_0001, hf_amp),
            noise_frame(t, 0xAB00_0002, hf_amp * 0.1),
        )
    };
    for i in 0..LONG_WINDOW_LEN {
        let n = (t * LONG_WINDOW_LEN + i) as f32;
        let low = 0.3 * (2.0 * std::f32::consts::PI * 200.0 * n / 48_000.0).sin();
        let (mut hl, mut hr) = (0.0f32, 0.0f32);
        for (k, &f) in [8_000.0f32, 10_000.0, 12_000.0].iter().enumerate() {
            let w = 2.0 * std::f32::consts::PI * f / 48_000.0;
            hl += (w * n + 0.9 * k as f32).sin();
            hr += (w * n + 0.9 * k as f32 + 2.1).sin();
        }
        l.push(low + hf_amp * hl + nl[i]);
        r.push(low + hf_amp * 0.1 * hr + nr[i]);
    }
    (l, r)
}

/// Decode payloads through the shipped streaming decoder; returns per-channel
/// SNR (dB) against the input with the one-frame encoder delay accounted for.
fn ab_snr(payloads: &[Vec<u8>], frames: &[(Vec<f32>, Vec<f32>)], fs_index: u8) -> Result<[f64; 2]> {
    let mut dec = StreamDecoder::new();
    let (mut ps, mut pe) = ([0.0f64; 2], [0.0f64; 2]);
    for (t, payload) in payloads.iter().enumerate() {
        let frame = dec.decode_raw_data_block(2, fs_index, 48_000, 2, 1, payload)?;
        if t < 2 || t + 1 >= frames.len() {
            continue; // priming + one-frame delay; drop the tail frame
        }
        let (l, r) = &frames[t - 1];
        for (ch, want) in [l, r].into_iter().enumerate() {
            for (&g, &w) in frame.planar[ch].iter().zip(want.iter()) {
                // DecodedFrame is in the engine's s16 domain (×32768).
                let w = f64::from(w) * 32768.0;
                ps[ch] += w * w;
                let e = w - f64::from(g);
                pe[ch] += e * e;
            }
        }
    }
    Ok([
        10.0 * (ps[0] / pe[0].max(1.0)).log10(),
        10.0 * (ps[1] / pe[1].max(1.0)).log10(),
    ])
}

#[test]
fn per_band_never_worse_than_whole_pair() -> Result<()> {
    // A/B against the whole-pair fallback. Whole-pair must either code the
    // quiet right HF via M/S (All) or forgo the shared-sine side collapse
    // (Off); per-band gets both.
    let encode_all = |per_band: bool, frames: &[(Vec<f32>, Vec<f32>)]| -> Result<Vec<Vec<u8>>> {
        let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
        enc.set_ms_per_band(per_band);
        frames
            .iter()
            .map(|(l, r)| enc.encode_frame(&[l, r]))
            .collect()
    };
    // Tonal content fits under the 128k budget, so coded bytes reflect the
    // decision directly: per-band must be strictly smaller here.
    let tonal: Vec<_> = (0..8).map(|t| ab_frame(t, 0.2, true)).collect();
    let whole = encode_all(false, &tonal)?;
    let per_band = encode_all(true, &tonal)?;
    let (bw, bp): (usize, usize) = (
        whole.iter().map(Vec::len).sum(),
        per_band.iter().map(Vec::len).sum(),
    );
    eprintln!("asymmetric tonal split @128k: whole-pair {bw} B, per-band {bp} B");
    assert!(bp < bw, "per-band {bp} B should beat whole-pair {bw} B");
    // Noisy content saturates the 128k budget on both sides, so the only
    // structural byte difference is the per-band mask tax
    // ((n_bands − 2) bits/frame vs the 2-bit whole-pair mask); quality
    // stays within psy noise of the whole-pair fallback.
    let n_bands = long_offsets(3)?.len() - 1;
    let noisy: Vec<_> = (0..10).map(|t| ab_frame(t, 0.3, false)).collect();
    let whole = encode_all(false, &noisy)?;
    let per_band = encode_all(true, &noisy)?;
    let (bw, bp): (usize, usize) = (
        whole.iter().map(Vec::len).sum(),
        per_band.iter().map(Vec::len).sum(),
    );
    let sw = ab_snr(&whole, &noisy, 3)?;
    let sp = ab_snr(&per_band, &noisy, 3)?;
    eprintln!(
        "asymmetric noisy split @128k: whole {bw} B SNR {:.1}/{:.1} dB, \
         per-band {bp} B SNR {:.1}/{:.1} dB",
        sw[0], sw[1], sp[0], sp[1]
    );
    let tax_bytes = noisy.len() * (n_bands - 2).div_ceil(8) + 16;
    assert!(
        bp <= bw + tax_bytes,
        "per-band {bp} B exceeds whole-pair {bw} B by more than the mask tax"
    );
    for ch in 0..2 {
        assert!(sp[ch] > 0.0, "ch{ch}: per-band SNR {:.1} dB", sp[ch]);
        assert!(
            sp[ch] >= sw[ch] - 1.5,
            "ch{ch}: per-band SNR {:.1} regressed vs whole-pair {:.1}",
            sp[ch],
            sw[ch]
        );
    }
    Ok(())
}

/// Silence: no loud bands, mask folds to Off and the frame stays tiny.
#[test]
fn silence_stays_off() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let pcm = [0.0f32; LONG_WINDOW_LEN];
    let payload = enc.encode_frame(&[&pcm, &pcm])?;
    let (_, ms, mask_bits) = parse_ms(&payload, enc.fs_index())?;
    assert_eq!(ms.mask, MsMask::Off);
    assert_eq!(mask_bits, 2);
    Ok(())
}

/// `MsBands::off` (mono) is never emitted but must have a sane form.
#[test]
fn mono_mask_form_is_off() {
    let ms = MsBands::off();
    assert_eq!(ms.form(), MaskForm::Off);
    assert_eq!(ms.overhead_bits(), 2);
}
