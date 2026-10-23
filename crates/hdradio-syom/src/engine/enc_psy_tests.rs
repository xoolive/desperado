//! Psy model behavior: tones concentrate precision, noise spreads it,
//! silence codes nothing.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::swb::LONG_WINDOW_LEN;
use super::super::swb::{long_offsets, short_offsets};
use super::{AttackDetector, Psy};
use crate::engine::enc_quant::MAX_BANDS;

fn offsets_48k() -> (&'static [u16], usize) {
    let o = long_offsets(3).expect("48k offsets");
    (o, o.len() - 1)
}

/// MDCT-shaped spectrum with a single tone at `bin`.
fn tone_spec(bin: usize) -> [f32; LONG_WINDOW_LEN] {
    let mut spec = [0.0f32; LONG_WINDOW_LEN];
    spec[bin] = 1e6;
    spec[bin + 1] = 3e5;
    spec
}

#[test]
fn tone_concentrates_coding() {
    let (offsets, n_bands) = offsets_48k();
    let mut psy = Psy::new(offsets, 48_000);
    let spec = tone_spec(100);
    let mut coded = [false; MAX_BANDS];
    let mut tq = [0.0f32; MAX_BANDS];
    psy.analyze(&spec, offsets, 2048.0, &mut coded, &mut tq);
    let n_coded = coded[..n_bands].iter().filter(|&&c| c).count();
    assert!(n_coded <= 10, "one tone coded {n_coded} bands");
    // The tone's own band gets full precision.
    let tone_band = offsets.iter().position(|&o| o > 100).expect("band") - 1;
    assert!(coded[tone_band]);
    assert!(
        (tq[tone_band] - 2048.0).abs() < 1.0,
        "tone band target {}",
        tq[tone_band]
    );
}

#[test]
fn flat_noise_codes_everywhere() {
    let (offsets, n_bands) = offsets_48k();
    let mut psy = Psy::new(offsets, 48_000);
    let mut spec = [0.0f32; LONG_WINDOW_LEN];
    // Deterministic flat-ish spectrum.
    for (i, v) in spec.iter_mut().enumerate() {
        *v = if i % 2 == 0 { 1e4 } else { -1e4 };
    }
    let mut coded = [false; MAX_BANDS];
    let mut tq = [0.0f32; MAX_BANDS];
    psy.analyze(&spec, offsets, 2048.0, &mut coded, &mut tq);
    let n_coded = coded[..n_bands].iter().filter(|&&c| c).count();
    assert!(
        n_coded >= n_bands * 3 / 4,
        "noise coded only {n_coded}/{n_bands}"
    );
}

#[test]
fn silence_codes_nothing() {
    let (offsets, n_bands) = offsets_48k();
    let mut psy = Psy::new(offsets, 48_000);
    let spec = [0.0f32; LONG_WINDOW_LEN];
    let mut coded = [true; MAX_BANDS];
    let mut tq = [1.0f32; MAX_BANDS];
    psy.analyze(&spec, offsets, 2048.0, &mut coded, &mut tq);
    assert!(
        coded[..n_bands].iter().all(|&c| !c),
        "silence must code no bands"
    );
}

#[test]
fn short_psy_uses_short_bin_width() {
    // A short-window tone at bin k sits 8× higher in Hz than the same long
    // bin: the short psy must code the short band containing it.
    let short = short_offsets(3).expect("48k short offsets");
    let mut psy = Psy::new_short(short, 48_000);
    let mut spec = [0.0f32; 128];
    spec[30] = 1e6;
    let n_bands = short.len() - 1;
    let mut coded = [false; MAX_BANDS];
    let mut tq = [0.0f32; MAX_BANDS];
    psy.analyze(&spec, short, 2048.0, &mut coded, &mut tq);
    let band = short.iter().position(|&o| o > 30).expect("band") - 1;
    assert!(coded[band], "short tone band not coded");
    assert!(
        coded[..n_bands].iter().filter(|&&c| c).count() <= 8,
        "short tone spread too wide"
    );
}

fn frame_of(f: impl Fn(usize) -> f32) -> Vec<f32> {
    (0..LONG_WINDOW_LEN).map(f).collect()
}

#[test]
fn detector_is_quiet_on_tones_and_silence() {
    let mut det = AttackDetector::new();
    for f in 0..20 {
        let tone = frame_of(|i| {
            0.5 * (2.0 * std::f32::consts::PI * 440.0 * (f * LONG_WINDOW_LEN + i) as f32 / 48_000.0)
                .sin()
        });
        assert!(!det.push(&tone), "sine frame {f} flagged as attack");
    }
    let silence = vec![0.0f32; LONG_WINDOW_LEN];
    for _ in 0..4 {
        assert!(!det.push(&silence), "silence flagged as attack");
    }
}

#[test]
fn detector_flags_click_after_steady_tone() {
    let mut det = AttackDetector::new();
    for f in 0..8 {
        let tone = frame_of(|i| {
            0.2 * (2.0 * std::f32::consts::PI * 440.0 * (f * LONG_WINDOW_LEN + i) as f32 / 48_000.0)
                .sin()
        });
        assert!(!det.push(&tone));
    }
    // Castanet-like click: a 32-sample burst mid-frame over the tone.
    let mut click = frame_of(|i| {
        0.2 * (2.0 * std::f32::consts::PI * 440.0 * (8 * LONG_WINDOW_LEN + i) as f32 / 48_000.0)
            .sin()
    });
    let mut lcg = 0x1234_5678u32;
    for v in &mut click[600..632] {
        lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let u = (lcg >> 9) as f32 / (1u32 << 23) as f32;
        *v += 0.8 * (2.0 * u - 1.0);
    }
    assert!(det.push(&click), "click not detected");
}

#[test]
fn detector_flags_onset_from_silence_but_not_first_frame() {
    let mut det = AttackDetector::new();
    let loud =
        frame_of(|i| 0.5 * (2.0 * std::f32::consts::PI * 1000.0 * i as f32 / 48_000.0).sin());
    // The very first frame has no history to surge against: no flag.
    assert!(!det.push(&loud), "first-ever frame must not flag");
    let mut det = AttackDetector::new();
    let silence = vec![0.0f32; LONG_WINDOW_LEN];
    for _ in 0..3 {
        assert!(!det.push(&silence));
    }
    assert!(det.push(&loud), "onset from silence must flag");
}

#[test]
fn detector_ignores_slow_swell() {
    let mut det = AttackDetector::new();
    // 10 dB/s linear swell: per-sub-block growth stays under the ratio.
    for f in 0..20i32 {
        let gain = 0.02 * 1.12f32.powi(f);
        let frame = frame_of(|i| {
            gain * (2.0 * std::f32::consts::PI * 700.0 * (f as usize * LONG_WINDOW_LEN + i) as f32
                / 48_000.0)
                .sin()
        });
        assert!(!det.push(&frame), "swell frame {f} flagged");
    }
}
