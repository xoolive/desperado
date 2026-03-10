//! Integration tests for VOR decoding using pre-demodulated audio fixtures.
//!
//! Tests that load fixture files are marked `#[ignore]` because they are slow
//! in a debug build.  Run them with:
//!
//! ```sh
//! cargo test --release -p voracious -- --include-ignored
//! ```
//!
//! # Test Fixtures
//!
//! Fixtures are stored in `tests/data/` as pre-demodulated f32 binary files,
//! extracted from real gqrx captures at 1.8 MSps (cf32 format) using the Rust
//! `VorDemodulator` (see `examples/extract_audio.rs`).
//!
//! Each fixture set contains three files derived from the same source capture:
//!
//! - `*_audio.f32`: mono audio at ~47368 Hz (AM-demodulated, decimated 38×)
//! - `*_var30.f32`: 30 Hz variable signal (envelope of 9960 Hz subcarrier)
//! - `*_ref30.f32`: 30 Hz reference signal (FM-demodulated 9960 Hz subcarrier)
//!
//! All signals are f32 little-endian, no header.
//!
//! # To regenerate fixtures
//!
//! ```sh
//! cargo run --release --example extract_audio -- \
//!   samples/gqrx_20250925_144051_114647000_1800000_fc.raw \
//!   114.85 114.647 tests/data/gqrx_20250925_144051_114647000_1800000_fc
//!
//! cargo run --release --example extract_audio -- \
//!   samples/gqrx_20251107_182558_116000000_1800000_fc.raw \
//!   116.0 116.0 tests/data/gqrx_20251107_182558_116000000_1800000_fc 26
//! ```
//!
//! # Sources
//!
//! | File stem                                       | Station | Freq (MHz) | Duration |
//! |-------------------------------------------------|---------|------------|----------|
//! | `gqrx_20250925_144051_114647000_1800000_fc`     | KLO     | 114.85     | ~18 s    |
//! | `gqrx_20251107_182558_116000000_1800000_fc`     | ARL     | 116.00     | ~26 s    |

use std::path::Path;
use voracious::decoders::{calculate_radial, calculate_radial_vortrack, decode_morse_ident};

// Audio rate matches VorDemodulator: round(1_800_000 / 48_000) = 38, rate = 1_800_000 / 38
const AUDIO_RATE: f64 = 1_800_000.0 / 38.0; // ≈ 47368 Hz

// Morse window: 15 seconds of audio, matching production default (source.rs)
const MORSE_WINDOW_SAMPLES: usize = (AUDIO_RATE as usize) * 15;

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

fn fixture_path(stem: &str, suffix: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(format!("{stem}_{suffix}.f32"))
}

/// Try Morse decoding over sliding windows of `MORSE_WINDOW_SAMPLES`, stepping by half a window.
/// Returns the first decoded ident and all accumulated tokens.
/// Mirrors the sliding-window strategy used by `IqSource` in `source.rs`.
fn decode_morse_sliding(audio: &[f64]) -> (Option<String>, Vec<String>) {
    let step = MORSE_WINDOW_SAMPLES / 2;
    let mut all_tokens: Vec<String> = Vec::new();
    let mut best_ident: Option<String> = None;

    let mut start = 0;
    while start + MORSE_WINDOW_SAMPLES <= audio.len() {
        let window = &audio[start..start + MORSE_WINDOW_SAMPLES];
        let (ident, tokens, _) = decode_morse_ident(window, AUDIO_RATE);
        all_tokens.extend(tokens);
        if best_ident.is_none() {
            best_ident = ident;
        }
        start += step;
    }

    (best_ident, all_tokens)
}

// ── KLO (114.85 MHz) ─────────────────────────────────────────────────────────

const KLO_STEM: &str = "gqrx_20250925_144051_114647000_1800000_fc";
const ARL_STEM: &str = "gqrx_20251107_182558_116000000_1800000_fc";

/// VORtrack radial should be in the expected range for KLO (~119°).
#[test]
#[ignore]
fn test_klo_vortrack_radial_in_range() {
    let audio = load_f32(&fixture_path(KLO_STEM, "audio"));

    let radial = calculate_radial_vortrack(&audio, AUDIO_RATE)
        .expect("calculate_radial_vortrack should return a value for KLO");

    assert!(
        (115.0..=125.0).contains(&radial),
        "KLO vortrack radial {radial:.1}° out of expected range 115–125°"
    );
}

/// VORtrack radial on 3-second windows should be consistent with the full-signal result.
/// This mirrors production behaviour where radial is computed per 3-second window.
#[test]
#[ignore]
fn test_klo_vortrack_windowed_consistent() {
    let audio = load_f32(&fixture_path(KLO_STEM, "audio"));

    let full_radial =
        calculate_radial_vortrack(&audio, AUDIO_RATE).expect("full signal radial failed");

    let window = (AUDIO_RATE * 3.0) as usize;
    let mut radials: Vec<f64> = Vec::new();
    for i in 0..3 {
        let start = i * window;
        let end = (start + window).min(audio.len());
        if let Some(r) = calculate_radial_vortrack(&audio[start..end], AUDIO_RATE) {
            radials.push(r);
        }
    }

    assert!(!radials.is_empty(), "no windowed radials computed");
    for r in &radials {
        let diff = ((r - full_radial + 540.0) % 360.0) - 180.0;
        assert!(
            diff.abs() < 5.0,
            "KLO windowed radial {r:.1}° deviates {diff:.1}° from full-signal {full_radial:.1}°"
        );
    }
}

/// Morse decoder should identify the KLO ident from audio using 15-second sliding windows.
#[test]
#[ignore]
fn test_klo_morse_ident() {
    let audio = load_f32(&fixture_path(KLO_STEM, "audio"));

    let (ident, tokens) = decode_morse_sliding(&audio);

    assert!(
        !tokens.is_empty(),
        "KLO: expected Morse tokens but got none"
    );
    assert_eq!(
        ident.as_deref(),
        Some("KLO"),
        "KLO: expected ident 'KLO', got {ident:?} (tokens: {tokens:?})"
    );
}

// ── ARL (116.00 MHz) ──────────────────────────────────────────────────────────

/// VORtrack radial should be in the expected range for ARL (~115°).
#[test]
#[ignore]
fn test_arl_vortrack_radial_in_range() {
    let audio = load_f32(&fixture_path(ARL_STEM, "audio"));

    let radial = calculate_radial_vortrack(&audio, AUDIO_RATE)
        .expect("calculate_radial_vortrack should return a value for ARL");

    assert!(
        (110.0..=120.0).contains(&radial),
        "ARL vortrack radial {radial:.1}° out of expected range 110–120°"
    );
}

/// VORtrack radial on 3-second windows should be consistent with the full-signal result.
#[test]
#[ignore]
fn test_arl_vortrack_windowed_consistent() {
    let audio = load_f32(&fixture_path(ARL_STEM, "audio"));

    let full_radial =
        calculate_radial_vortrack(&audio, AUDIO_RATE).expect("full signal radial failed");

    let window = (AUDIO_RATE * 3.0) as usize;
    let mut radials: Vec<f64> = Vec::new();
    for i in 0..5 {
        let start = i * window;
        let end = (start + window).min(audio.len());
        if let Some(r) = calculate_radial_vortrack(&audio[start..end], AUDIO_RATE) {
            radials.push(r);
        }
    }

    assert!(!radials.is_empty(), "no windowed radials computed");
    for r in &radials {
        let diff = ((r - full_radial + 540.0) % 360.0) - 180.0;
        assert!(
            diff.abs() < 5.0,
            "ARL windowed radial {r:.1}° deviates {diff:.1}° from full-signal {full_radial:.1}°"
        );
    }
}

/// Morse decoder should identify the ARL ident using sliding 15-second windows.
/// ARL has strong signal with ident bursts at t≈0–3 s, 8–11 s, and 23–26 s.
#[test]
#[ignore]
fn test_arl_morse_ident() {
    let audio = load_f32(&fixture_path(ARL_STEM, "audio"));

    let (ident, tokens) = decode_morse_sliding(&audio);

    assert!(
        !tokens.is_empty(),
        "ARL: expected Morse tokens but got none"
    );
    assert_eq!(
        ident.as_deref(),
        Some("ARL"),
        "ARL: expected ident 'ARL', got {ident:?} (tokens: {tokens:?})"
    );
}

// ── Edge cases ────────────────────────────────────────────────────────────────

/// VORtrack should return None gracefully for inputs shorter than 1 second.
#[test]
fn test_vortrack_returns_none_for_short_input() {
    let short = vec![0.0f64; (AUDIO_RATE * 0.5) as usize];
    assert!(
        calculate_radial_vortrack(&short, AUDIO_RATE).is_none(),
        "expected None for input shorter than 1 second"
    );
}

/// calculate_radial (FFT method) should return None for inputs shorter than 1 second.
#[test]
fn test_calculate_radial_returns_none_for_short_input() {
    let short = vec![0.0f64; (AUDIO_RATE * 0.5) as usize];
    assert!(
        calculate_radial(&short, &short, AUDIO_RATE).is_none(),
        "expected None for input shorter than 1 second"
    );
}

/// Morse decoder should return nothing for pure silence.
#[test]
fn test_morse_returns_none_for_silence() {
    let silence = vec![0.0f64; MORSE_WINDOW_SAMPLES];
    let (ident, tokens, _) = decode_morse_ident(&silence, AUDIO_RATE);
    assert!(
        ident.is_none(),
        "expected no ident from silence, got {ident:?}"
    );
    assert!(tokens.is_empty(), "expected no tokens from silence");
}
