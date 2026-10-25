//! Public encode API: roundtrips through the shipped decoder, error paths.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use crate::{AacError, EncodeOptions, decode_with, encode, encode_with};

/// SNR (dB) of `got` vs `want` over their common prefix.
fn snr_db(want: &[f32], got: &[f32]) -> f64 {
    let n = want.len().min(got.len());
    let mut ps = 0.0f64;
    let mut pe = 0.0f64;
    for i in 0..n {
        let s = f64::from(want[i]);
        let e = s - f64::from(got[i]);
        ps += s * s;
        pe += e * e;
    }
    if pe == 0.0 {
        return 200.0;
    }
    10.0 * (ps / pe).log10()
}

fn sine(rate: u32, secs: f64, freq: f32, amp: f32) -> Vec<f32> {
    let n = (f64::from(rate) * secs) as usize;
    (0..n)
        .map(|i| amp * (2.0 * std::f32::consts::PI * freq * i as f32 / rate as f32).sin())
        .collect()
}

/// Encoder delay: decoded sample `i` corresponds to input `i − 1024` past
/// the windowed fade-in (the first decoded frame). Compare from frame 2 on.
const PRIME: usize = 1024;

/// SNR of the roundtrip with the one-frame encoder delay accounted for.
fn snr_aligned(want: &[f32], got: &[f32]) -> f64 {
    snr_db(
        &want[PRIME..want.len() - PRIME],
        &got[2 * PRIME..want.len()],
    )
}

#[test]
fn sine_roundtrip_mono() {
    let pcm = vec![sine(48_000, 0.25, 440.0, 0.5)];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    assert_eq!(dec.sample_rate, 48_000);
    assert_eq!(dec.channels.len(), 1);
    let got = &dec.channels[0];
    let want = &pcm[0];
    assert!(got.len() >= want.len());
    let snr = snr_aligned(want, got);
    assert!(snr >= 45.0, "mono sine roundtrip SNR {snr:.1} dB");
}

#[test]
fn sine_roundtrip_stereo() {
    let l = sine(48_000, 0.25, 440.0, 0.5);
    let r = sine(48_000, 0.25, 660.0, 0.25);
    let pcm = vec![l, r];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    assert_eq!(dec.channels.len(), 2);
    for (ch, (want, got)) in pcm.iter().zip(dec.channels.iter()).enumerate() {
        let snr = snr_aligned(want, got);
        assert!(snr >= 40.0, "stereo ch{ch} roundtrip SNR {snr:.1} dB");
    }
}

#[test]
fn silence_stays_silent() {
    let pcm = vec![vec![0.0f32; 8192]];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    let peak = dec.channels[0]
        .iter()
        .map(|x| x.abs())
        .fold(0.0f32, f32::max);
    assert!(peak < 1e-4, "silence decoded with peak {peak}");
}

#[test]
fn partial_tail_frame_is_padded() {
    let pcm = vec![sine(48_000, 0.05, 440.0, 0.5)[..1000].to_vec()];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    assert_eq!(dec.channels[0].len(), 1024, "one padded frame");
}

#[test]
fn bitrate_knob_changes_size() {
    let pcm = vec![sine(48_000, 1.0, 440.0, 0.5)];
    let lo = encode_with(
        &pcm,
        48_000,
        &EncodeOptions::adts().with_bitrate_bps(32_000),
    )
    .expect("32k");
    let hi = encode_with(
        &pcm,
        48_000,
        &EncodeOptions::adts().with_bitrate_bps(192_000),
    )
    .expect("192k");
    assert!(
        hi.len() > lo.len(),
        "192k ({} B) should beat 32k ({} B)",
        hi.len(),
        lo.len()
    );
}

#[test]
fn error_paths_are_encode_errors() {
    let pcm = vec![sine(48_000, 0.05, 440.0, 0.5)];
    let cases: Vec<AacError> = vec![
        encode(&pcm, 47_000).unwrap_err(),
        encode(&[], 48_000).unwrap_err(),
        encode(&[vec![], vec![]], 48_000).unwrap_err(),
        encode(&[pcm[0].clone(), pcm[0].clone(), pcm[0].clone()], 48_000).unwrap_err(),
        encode(&[vec![f32::NAN; 2048]], 48_000).unwrap_err(),
        encode_with(&pcm, 48_000, &EncodeOptions::adts().with_bitrate_bps(0)).unwrap_err(),
    ];
    for e in cases {
        assert!(
            matches!(e, AacError::Encode(_)),
            "expected Encode, got {e:?}"
        );
        assert!(!e.to_string().is_empty(), "stable Display");
    }
}

#[test]
fn write_roundtrip_via_file() {
    let pcm = vec![sine(48_000, 0.05, 440.0, 0.5)];
    let path = std::env::temp_dir().join(format!("syom-enc-test-{}.adts", std::process::id()));
    crate::write(&path, &pcm, 48_000).expect("write");
    let dec = crate::read_with(&path, &crate::DecodeOptions::unbounded()).expect("read");
    let _ = std::fs::remove_file(&path);
    assert_eq!(dec.sample_rate, 48_000);
    let snr = snr_aligned(&pcm[0], &dec.channels[0]);
    assert!(snr >= 45.0, "file roundtrip SNR {snr:.1} dB");
}

/// Deterministic white noise (LCG), full-band, moderate level.
fn noise(rate: u32, secs: f64, amp: f32) -> Vec<f32> {
    let n = (f64::from(rate) * secs) as usize;
    let mut state = 0x2F6E_2B1Du32;
    (0..n)
        .map(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let u = (state >> 9) as f32 / (1u32 << 23) as f32; // [0, 1)
            amp * (2.0 * u - 1.0)
        })
        .collect()
}

/// Band-limited noise: sum of random-phase sines up to `fmax` (LCG phases).
fn band_noise(rate: u32, secs: f64, amp: f32, fmax: f32) -> Vec<f32> {
    let n = (f64::from(rate) * secs) as usize;
    let mut state = 0x1A2B_3C4Du32;
    let mut phase = [0.0f32; 96];
    let mut freq = [0.0f32; 96];
    for k in 0..96 {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        phase[k] = (state >> 9) as f32 / (1u32 << 23) as f32 * 2.0 * std::f32::consts::PI;
        freq[k] = fmax * (k as f32 + 0.5) / 96.0;
    }
    let scale = amp / 96f32.sqrt();
    (0..n)
        .map(|i| {
            let t = i as f32 / rate as f32;
            let mut acc = 0.0f32;
            for k in 0..96 {
                acc += (2.0 * std::f32::consts::PI * freq[k] * t + phase[k]).sin();
            }
            acc * scale
        })
        .collect()
}

#[test]
fn bitrate_accuracy_on_noise() {
    // White noise is budget-limited: achieved bitrate tracks the target.
    let l = noise(48_000, 1.0, 0.3);
    let r = noise(48_000, 1.0, 0.3);
    let stereo = vec![l.clone(), r];
    for (pcm, target) in [
        (vec![l], 32_000u32),
        (stereo.clone(), 64_000),
        (stereo, 128_000),
    ] {
        let opts = EncodeOptions::adts().with_bitrate_bps(target);
        let adts = encode_with(&pcm, 48_000, &opts).expect("encode");
        let achieved = (adts.len() as u64) * 8;
        let ratio = achieved as f64 / f64::from(target);
        assert!(
            (0.90..=1.10).contains(&ratio),
            "target {target}, achieved {achieved} bits/s"
        );
    }
}

#[test]
fn band_noise_quality_floor_stereo_128k() {
    let l = band_noise(48_000, 0.5, 0.3, 4_000.0);
    let r = band_noise(48_000, 0.5, 0.3, 4_000.0);
    let pcm = vec![l, r];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    for (ch, (want, got)) in pcm.iter().zip(dec.channels.iter()).enumerate() {
        let snr = snr_aligned(want, got);
        assert!(snr >= 15.0, "noise ch{ch} SNR {snr:.1} dB");
    }
}

#[test]
fn correlated_stereo_uses_ms_and_saves_bits() {
    // Nearly identical channels: M/S collapses the side channel.
    let l = sine(48_000, 0.25, 440.0, 0.5);
    let r: Vec<f32> = l.iter().map(|&x| x * 0.98).collect();
    let corr = encode(&[l.clone(), r], 48_000).expect("correlated");
    // Per-band M/S keeps genuinely decorrelated bands on L/R, so the old
    // separate-sines contrast no longer isolates the M/S saving (L/R codes
    // that pair cheaply too). Independent full-band noise — which no stereo
    // tool can help — is the honest expensive baseline.
    let mut s1 = 0x0BAD_F00Du32;
    let mut s2 = 0x5EED_1234u32;
    let n = 12_000usize;
    let (mut n1, mut n2) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for _ in 0..n {
        s1 = s1.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        s2 = s2.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        n1.push(0.5 * (((s1 >> 9) as f32 / (1u32 << 23) as f32) * 2.0 - 1.0));
        n2.push(0.5 * (((s2 >> 9) as f32 / (1u32 << 23) as f32) * 2.0 - 1.0));
    }
    let decorr = encode(&[n1, n2], 48_000).expect("decorrelated");
    assert!(
        corr.len() * 5 < decorr.len() * 4,
        "correlated {} B should save >20% vs decorrelated {} B",
        corr.len(),
        decorr.len()
    );
    let dec = decode_with(&corr, &crate::DecodeOptions::unbounded()).expect("decode");
    let want_r: Vec<f32> = dec.channels[0].iter().map(|&x| x * 0.98).collect();
    // 0.98x differs from 1.0x by -34 dB, so a faithful M/S decode of r
    // tracks 0.98 * decoded-l closely (decoder outputs: no delay shift).
    let snr = snr_db(&want_r[2048..], &dec.channels[1][2048..]);
    assert!(snr >= 45.0, "M/S right-channel tracking {snr:.1} dB");
}

#[test]
fn decorrelated_stereo_stays_lr() {
    let l = sine(48_000, 0.25, 440.0, 0.5);
    let r = sine(48_000, 0.25, 2_997.0, 0.4);
    let pcm = vec![l, r];
    let adts = encode(&pcm, 48_000).expect("encode");
    let dec = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode");
    for (ch, (want, got)) in pcm.iter().zip(dec.channels.iter()).enumerate() {
        let snr = snr_aligned(want, got);
        assert!(snr >= 30.0, "decorrelated ch{ch} SNR {snr:.1} dB");
    }
}

/// Deterministic lavc-oracle fixture: 0.6 s stereo 48 kHz — a 200→4000 Hz
/// sweep (0.3 s), a decorrelated noise burst (0.2 s), an 880 Hz tremolo
/// (8 Hz full-depth AM, 0.1 s). The sweep's sine goes through
/// `det_math::sincos` (libm `sin` is not bit-identical across platforms)
/// with the phase kept in [0, 2π) — the golden streams are byte-exact only
/// if this signal is, too. The tremolo's intra-frame envelope movement is
/// what exercises TNS (steady tones/sweeps barely whiten, so TNS stays off
/// there — matching lavc's behavior on pure tones).
fn lavc_fixture() -> Vec<Vec<f32>> {
    let n = 28_800usize;
    let mut l = vec![0.0f32; n];
    let mut r = vec![0.0f32; n];
    // Sweep: integrate a linearly rising frequency.
    let mut phase = 0.0f32;
    for i in 0..14_400 {
        let t = i as f32 / 14_400.0;
        let f = 200.0 + 3_800.0 * t;
        phase += 2.0 * std::f32::consts::PI * f / 48_000.0;
        if phase >= std::f32::consts::TAU {
            phase -= std::f32::consts::TAU;
        }
        let v = 0.4 * crate::engine::det_math::sincos(phase).0;
        l[i] = v;
        r[i] = 0.8 * v;
    }
    // Noise burst: independent LCGs per channel (decorrelated).
    let (mut sl, mut sr) = (0x0BAD_F00Du32, 0x5EED_1234u32);
    for i in 14_400..24_000 {
        sl = sl.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        sr = sr.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        l[i] = 0.25 * (((sl >> 9) as f32 / (1u32 << 23) as f32) * 2.0 - 1.0);
        r[i] = 0.25 * (((sr >> 9) as f32 / (1u32 << 23) as f32) * 2.0 - 1.0);
    }
    // Tremolo: correlated channels like the sweep.
    let (mut cphase, mut ephase) = (0.0f32, 0.0f32);
    for i in 24_000..28_800 {
        cphase += 2.0 * std::f32::consts::PI * 880.0 / 48_000.0;
        if cphase >= std::f32::consts::TAU {
            cphase -= std::f32::consts::TAU;
        }
        ephase += 2.0 * std::f32::consts::PI * 8.0 / 48_000.0;
        if ephase >= std::f32::consts::TAU {
            ephase -= std::f32::consts::TAU;
        }
        let env = 0.5 + 0.5 * crate::engine::det_math::sincos(ephase).0;
        let v = 0.4 * env * crate::engine::det_math::sincos(cphase).0;
        l[i] = v;
        r[i] = 0.8 * v;
    }
    vec![l, r]
}

fn to_s16(v: f32) -> i16 {
    (v * 32768.0).round().clamp(-32768.0, 32767.0) as i16
}

/// The oracle tolerance check, shared with `encode_transient_tests`.
pub(crate) fn assert_decode_matches_lavc_pub(stream: &[u8], lavc: &[u8]) {
    assert_decode_matches_lavc(stream, lavc);
}

/// Offline mint: `MINT_GOLDENS=1 cargo test --lib mint_lavc_adts_golden`,
/// then decode the written ADTS with ffmpeg to `enc48.lavc.s16`:
/// `ffmpeg -y -i src/goldens/enc48.adts -f s16le src/goldens/enc48.lavc.s16`
/// Tests never spawn ffmpeg; the committed s16 is the oracle.
#[test]
fn mint_lavc_adts_golden() {
    if std::env::var("MINT_GOLDENS").is_err() {
        return;
    }
    let pcm = lavc_fixture();
    let adts = encode(&pcm, 48_000).expect("encode");
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/goldens");
    std::fs::write(dir.join("enc48.adts"), adts).expect("write golden");
}

#[test]
fn lavc_matches_our_decode_of_our_adts() {
    let adts = include_bytes!("goldens/enc48.adts");
    let lavc = include_bytes!("goldens/enc48.lavc.s16");
    let pcm = lavc_fixture();
    let fresh = encode(&pcm, 48_000).expect("encode");
    // Layer 1 — byte-exactness tripwire. The encoder's decision path is
    // platform-deterministic (every transcendental goes through
    // `engine::det_math`; the fixture's sine as well), so the fresh encode
    // must equal the committed golden on macOS and Linux alike. This is a
    // drift tripwire, not a product guarantee: if the encoder changes
    // intentionally, re-mint (MINT_GOLDENS=1 + ffmpeg); if a new platform
    // ever breaks it, relax to a length band and rely on layer 2.
    assert_eq!(
        fresh.as_slice(),
        &adts[..],
        "encoder output drifted from the committed golden; re-mint with \
         MINT_GOLDENS=1 and refresh the lavc s16"
    );
    // Layer 2 — the contract that actually matters: our decode of the fresh
    // encode matches ffmpeg's decode within the oracle tolerance.
    assert_decode_matches_lavc(&fresh, lavc);
}

/// The oracle tolerance check: our decode of `stream` vs the committed
/// ffmpeg-decoded s16 — ≤ 2 LSB s16 max error, ≥ 55 dB SNR per channel.
fn assert_decode_matches_lavc(stream: &[u8], lavc: &[u8]) {
    let dec = decode_with(stream, &crate::DecodeOptions::unbounded()).expect("decode");
    assert_eq!(dec.channels.len(), 2);
    let frames = lavc.len() / 4; // s16 stereo interleaved
    assert_eq!(dec.channels[0].len(), frames, "frame count vs lavc");
    for (ch, got) in dec.channels.iter().enumerate() {
        let mut max_lsb = 0u32;
        let mut ps = 0.0f64;
        let mut pe = 0.0f64;
        for (i, &g) in got.iter().enumerate() {
            let lav = i16::from_le_bytes([lavc[(i * 2 + ch) * 2], lavc[(i * 2 + ch) * 2 + 1]]);
            let ours = to_s16(g);
            max_lsb = max_lsb.max((i32::from(lav) - i32::from(ours)).unsigned_abs());
            let s = f64::from(lav);
            ps += s * s;
            pe += (s - f64::from(ours)).powi(2);
        }
        let snr = 10.0 * (ps / pe.max(1.0)).log10();
        assert!(max_lsb <= 2, "ch{ch}: max {max_lsb} LSB vs lavc");
        assert!(snr >= 55.0, "ch{ch}: SNR vs lavc {snr:.1} dB");
    }
}

#[test]
fn m4a_roundtrip_matches_adts_minus_priming() {
    let pcm = lavc_fixture();
    let adts = encode(&pcm, 48_000).expect("adts");
    let m4a = encode_with(&pcm, 48_000, &EncodeOptions::m4a()).expect("m4a");
    assert!(crate::sniff_is_isobmff(&m4a));
    let da = decode_with(&adts, &crate::DecodeOptions::unbounded()).expect("decode adts");
    let dm = decode_with(&m4a, &crate::DecodeOptions::unbounded()).expect("decode m4a");
    assert_eq!(dm.sample_rate, 48_000);
    assert_eq!(dm.channels.len(), 2);
    // elst media_time = 1024: the M4A drops exactly the priming frame.
    assert_eq!(dm.channels[0].len() + 1024, da.channels[0].len());
    for ch in 0..2 {
        let a = &da.channels[ch];
        let m = &dm.channels[ch];
        let mut max_diff = 0.0f32;
        for (i, &mv) in m.iter().enumerate() {
            max_diff = max_diff.max((mv - a[i + 1024]).abs());
        }
        assert_eq!(max_diff, 0.0, "ch{ch}: M4A != ADTS after the 1024 shift");
    }
}

/// Offline mint: `MINT_GOLDENS=1 cargo test --lib mint_lavc_m4a_golden`,
/// then `ffmpeg -y -i src/goldens/enc48m.m4a -f s16le src/goldens/enc48m.lavc.s16`.
#[test]
fn mint_lavc_m4a_golden() {
    if std::env::var("MINT_GOLDENS").is_err() {
        return;
    }
    let pcm = lavc_fixture();
    let m4a = encode_with(&pcm, 48_000, &EncodeOptions::m4a()).expect("encode m4a");
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/goldens");
    std::fs::write(dir.join("enc48m.m4a"), m4a).expect("write golden");
}

#[test]
fn lavc_matches_our_decode_of_our_m4a() {
    let m4a = include_bytes!("goldens/enc48m.m4a");
    let lavc = include_bytes!("goldens/enc48m.lavc.s16");
    let pcm = lavc_fixture();
    let fresh = encode_with(&pcm, 48_000, &EncodeOptions::m4a()).expect("encode");
    // Same two layers as the ADTS oracle: byte-exactness tripwire (the
    // encoder is platform-deterministic by construction), then the real
    // contract — decode-equivalence with ffmpeg within tolerance.
    assert_eq!(
        fresh.as_slice(),
        &m4a[..],
        "encoder output drifted from the committed golden; re-mint"
    );
    // ffmpeg honours elst too: both sides skipped the 1024-sample priming.
    assert_decode_matches_lavc(&fresh, lavc);
}
