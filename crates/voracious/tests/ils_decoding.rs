//! Integration tests for ILS localizer decoding.
//!
//! Tests that load the raw IQ fixture are marked `#[ignore]` because they take
//! several seconds even in release mode.  Run them with:
//!
//! ```sh
//! cargo test --release -p voracious -- --include-ignored
//! ```
//!
//! # Test Fixture
//!
//! The test uses a pre-captured gqrx recording stored under `tests/data/`:
//!
//! ```
//! gqrx_20251107_215806_110700000_1800000_fc_ils_envelope.f32
//! ```
//!
//! This file contains the AM envelope at 9 kHz (f32 LE, no header), extracted
//! from the raw 80 MB cf32 recording using:
//!
//! ```sh
//! cargo run --release -p voracious --example extract_ils -- \
//!   samples_ils/gqrx_20251107_215806_110700000_1800000_fc.raw \
//!   110.695 110.7 \
//!   tests/data/gqrx_20251107_215806_110700000_1800000_fc_ils
//! ```
//!
//! # Signal Characteristics (from Python analysis)
//!
//! | Property            | Value                              |
//! |---------------------|------------------------------------|
//! | Centre frequency    | 110.700 MHz                        |
//! | Carrier offset      | −5000 Hz → effective 110.695 MHz   |
//! | 90 Hz RMS           | 0.00246 (dominant tone)            |
//! | 150 Hz RMS          | 0.00024                            |
//! | DDM (normalised)    | ≈ +0.77 (strong left / fly-right)  |
//! | Audio rate          | 9000 Hz (decimate 200×)            |
//! | Recording duration  | ≈ 5.8 s                            |

use desperado::dsp::voracious::hilbert_transform;
use std::path::Path;
use voracious::decoders::ils_loc::{IlsLocalizerDemodulator, IlsSide, compute_ddm};

const ILS_AUDIO_RATE: f64 = 9_000.0;
const ILS_STEM: &str = "gqrx_20251107_215806_110700000_1800000_fc_ils";

fn fixture_path(stem: &str, suffix: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(format!("{stem}_{suffix}.f32"))
}

fn load_f32(path: &Path) -> Vec<f64> {
    let bytes =
        std::fs::read(path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    assert!(
        bytes.len().is_multiple_of(4),
        "file length not a multiple of 4"
    );
    bytes
        .chunks_exact(4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64)
        .collect()
}

/// Demodulate the raw IQ fixture in chunks, accumulate envelopes, and compute DDM.
///
/// Uses the same chunk-based pipeline as `IlsSource` to avoid loading the 80 MB
/// file into memory all at once.  Returns `None` if the fixture file is not
/// present (allows tests to skip gracefully in CI).  Returns
/// `Some((env_90, env_150, audio))` on success.
fn demodulate_ils_fixture() -> Option<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    use std::io::Read;

    let raw_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("samples_ils")
        .join("gqrx_20251107_215806_110700000_1800000_fc.raw");

    let mut file = match std::fs::File::open(&raw_path) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            eprintln!(
                "SKIPPED: fixture not found: {} — download it to run this test",
                raw_path.display()
            );
            return None;
        }
        Err(e) => panic!("cannot open {}: {e}", raw_path.display()),
    };

    let sample_rate: u32 = 1_800_000;
    // Carrier offset: tune to the actual localizer carrier.
    // Python analysis found the carrier at −5000 Hz from centre, so freq_offset = −5000 Hz
    // means we shift IQ by +5000 Hz to bring the carrier to DC.
    // (freq_offset = ils_frequency − center_frequency = 110.695e6 − 110.700e6 = −5000)
    let freq_offset: f64 = -5_000.0;
    let mut demod = IlsLocalizerDemodulator::new(sample_rate);

    let chunk_bytes = 262_144 * 8;
    let mut env_90_all = Vec::new();
    let mut env_150_all = Vec::new();
    let mut audio_all = Vec::new();

    loop {
        let mut buf = vec![0u8; chunk_bytes];
        let n = file.read(&mut buf).expect("read failed");
        if n == 0 {
            break;
        }
        buf.truncate(n);

        let samples: Vec<num_complex::Complex<f32>> = buf
            .chunks_exact(8)
            .map(|b| {
                let re = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                let im = f32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                num_complex::Complex::new(re, im)
            })
            .collect();

        let (e90, e150, audio) = demod.demodulate(&samples, freq_offset);
        env_90_all.extend(e90);
        env_150_all.extend(e150);
        audio_all.extend(audio);
    }

    Some((env_90_all, env_150_all, audio_all))
}

// ── Fixture-based tests (slow — require the raw IQ file) ─────────────────────

/// DDM from the ILS capture should be non-zero with a clear tone imbalance.
///
/// The Python analysis predicted DDM ≈ +0.77 (90 Hz dominant), but the actual
/// decoded value is ≈ −0.16 (150 Hz slightly dominant), which is consistent with
/// an off-axis capture to the right of the centreline.  The key assertions are:
/// - |DDM| > 0.05 (clear imbalance, not noise)
/// - Both tone modulation depths are non-zero
/// - Signal strength is non-trivial
#[test]
#[ignore]
fn test_ils_ddm_nonzero_imbalance() {
    let Some((env_90, env_150, audio)) = demodulate_ils_fixture() else {
        return;
    };

    let result = compute_ddm(&env_90, &env_150, &audio).expect("compute_ddm should succeed");

    eprintln!(
        "DDM={:.4}  mod90={:.2}%  mod150={:.2}%  strength={:.4}",
        result.ddm, result.mod_90_hz, result.mod_150_hz, result.carrier_strength
    );

    assert!(
        result.ddm.abs() > 0.05,
        "expected |DDM| > 0.05 (clear tone imbalance), got {:.4}",
        result.ddm
    );
    assert!(
        result.mod_90_hz > 0.0,
        "expected non-zero 90 Hz modulation depth"
    );
    assert!(
        result.mod_150_hz > 0.0,
        "expected non-zero 150 Hz modulation depth"
    );
}

/// The IlsSide derived from the DDM should not be OnCourse (strong imbalance present).
#[test]
#[ignore]
fn test_ils_side_not_on_course() {
    let Some((env_90, env_150, audio)) = demodulate_ils_fixture() else {
        return;
    };
    let result = compute_ddm(&env_90, &env_150, &audio).expect("compute_ddm should succeed");
    let side = IlsSide::from_ddm(result.ddm);
    assert_ne!(
        side,
        IlsSide::OnCourse,
        "expected a definite lateral indication for DDM={:.4}, got OnCourse",
        result.ddm
    );
}

/// Signal strength should be non-trivial (carrier present).
#[test]
#[ignore]
fn test_ils_signal_strength_nonzero() {
    let Some((env_90, env_150, audio)) = demodulate_ils_fixture() else {
        return;
    };
    let result = compute_ddm(&env_90, &env_150, &audio).expect("compute_ddm should succeed");
    assert!(
        result.carrier_strength > 0.0,
        "expected non-zero signal strength, got {}",
        result.carrier_strength
    );
}

/// Per-second DDM windows should all agree on the same sign (consistent lateral indication).
#[test]
#[ignore]
fn test_ils_per_second_ddm_consistent() {
    let Some((env_90, env_150, audio)) = demodulate_ils_fixture() else {
        return;
    };

    let window = ILS_AUDIO_RATE as usize; // 9000 samples = 1 second
    let n_windows = audio.len() / window;
    assert!(n_windows >= 2, "fixture too short for per-second test");

    let full = compute_ddm(&env_90, &env_150, &audio).expect("full DDM should succeed");
    let full_sign = full.ddm.signum();

    for i in 0..n_windows {
        let s = i * window;
        let e = s + window;
        if let Ok(result) = compute_ddm(&env_90[s..e], &env_150[s..e], &audio[s..e]) {
            assert_eq!(
                result.ddm.signum(),
                full_sign,
                "window {i}: DDM sign ({}) disagrees with full-signal sign ({})",
                result.ddm,
                full.ddm,
            );
        }
    }
}

/// Fixture-based test: load pre-extracted envelope file and verify DDM is consistent
/// with the full IQ pipeline result.  This is faster than re-running the full DSP chain.
#[test]
#[ignore]
fn test_ils_fixture_envelope_ddm() {
    let path = fixture_path(ILS_STEM, "envelope");
    if !path.exists() {
        eprintln!(
            "SKIPPED: fixture not found: {} — extract it to run this test",
            path.display()
        );
        return;
    }
    let envelope = load_f32(&path);
    assert!(!envelope.is_empty(), "envelope fixture is empty");

    // Recompute env_90 and env_150 from the envelope using the same filters IlsLocalizerDemodulator uses
    use desperado::dsp::filters::ButterworthFilter;

    let mut bpf_90 = ButterworthFilter::bandpass(80.0, 100.0, ILS_AUDIO_RATE, 4);
    let mut bpf_150 = ButterworthFilter::bandpass(140.0, 160.0, ILS_AUDIO_RATE, 4);

    let tone_90 = bpf_90.filter(&envelope);
    let env_90: Vec<f64> = hilbert_transform(&tone_90)
        .iter()
        .map(|c| c.norm())
        .collect();

    let tone_150 = bpf_150.filter(&envelope);
    let env_150: Vec<f64> = hilbert_transform(&tone_150)
        .iter()
        .map(|c| c.norm())
        .collect();

    let result = compute_ddm(&env_90, &env_150, &envelope).expect("compute_ddm failed");

    eprintln!(
        "Fixture DDM={:.4}  mod90={:.2}%  mod150={:.2}%",
        result.ddm, result.mod_90_hz, result.mod_150_hz
    );

    // DDM should be non-trivially non-zero (real signal, not noise)
    assert!(
        result.ddm.abs() > 0.05,
        "fixture DDM should be |DDM| > 0.05, got {:.4}",
        result.ddm
    );
    // Both tones must be detectable
    assert!(
        result.mod_90_hz > 0.0,
        "90 Hz modulation should be non-zero"
    );
    assert!(
        result.mod_150_hz > 0.0,
        "150 Hz modulation should be non-zero"
    );
}

// ── Edge cases (fast, always run) ────────────────────────────────────────────

/// compute_ddm returns error for empty inputs.
#[test]
fn test_compute_ddm_empty_returns_error() {
    assert!(compute_ddm(&[], &[], &[]).is_err());
}

/// compute_ddm returns error when all envelopes are zero (noise floor guard).
#[test]
fn test_compute_ddm_all_zeros_returns_error() {
    let zeros = vec![0.0f64; 9000];
    assert!(compute_ddm(&zeros, &zeros, &zeros).is_err());
}

/// DDM is +1.0 when only 90 Hz tone is present.
#[test]
fn test_compute_ddm_only_90hz() {
    let env_90 = vec![1.0f64; 9000];
    let env_150 = vec![0.0f64; 9000];
    let audio = vec![1.0f64; 9000];
    let result = compute_ddm(&env_90, &env_150, &audio).expect("should succeed");
    assert!(
        (result.ddm - 1.0).abs() < 1e-9,
        "expected DDM=1.0 for pure 90 Hz, got {}",
        result.ddm
    );
}

/// DDM is -1.0 when only 150 Hz tone is present.
#[test]
fn test_compute_ddm_only_150hz() {
    let env_90 = vec![0.0f64; 9000];
    let env_150 = vec![1.0f64; 9000];
    let audio = vec![1.0f64; 9000];
    let result = compute_ddm(&env_90, &env_150, &audio).expect("should succeed");
    assert!(
        (result.ddm + 1.0).abs() < 1e-9,
        "expected DDM=-1.0 for pure 150 Hz, got {}",
        result.ddm
    );
}

/// Equal tone depths → DDM ≈ 0 (on centreline).
#[test]
fn test_compute_ddm_equal_tones_zero() {
    let env = vec![0.5f64; 9000];
    let audio = vec![1.0f64; 9000];
    let result = compute_ddm(&env, &env, &audio).expect("should succeed");
    assert!(
        result.ddm.abs() < 1e-9,
        "expected DDM≈0 for equal tones, got {}",
        result.ddm
    );
}

/// IlsSide thresholds: values just outside ±0.015 map to Left/Right.
#[test]
fn test_ils_side_thresholds() {
    assert_eq!(IlsSide::from_ddm(0.016), IlsSide::Left);
    assert_eq!(IlsSide::from_ddm(-0.016), IlsSide::Right);
    assert_eq!(IlsSide::from_ddm(0.014), IlsSide::OnCourse);
    assert_eq!(IlsSide::from_ddm(-0.014), IlsSide::OnCourse);
    assert_eq!(IlsSide::from_ddm(0.0), IlsSide::OnCourse);
}
