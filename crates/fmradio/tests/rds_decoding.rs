//! Integration tests for RDS decoding from a real RTL-SDR capture.
//!
//! # Test Fixture
//!
//! `tests/data/rtlsdr_20210215_103300000_1102500_cu8.bin`:
//!   6-second RTL-SDR capture in cu8 format (unsigned 8-bit IQ),
//!   recorded Feb 15 2021, center 103.3 MHz, 1,102,500 samples/s.
//!   Contains two FM stations:
//!   - FIP     at 103.5 MHz (offset +200 kHz), PI = 0xF204
//!   - CLASSIQ at 103.1 MHz (offset −200 kHz), PI = 0xF221
//!
//! # Running
//!
//! These tests are marked `#[ignore]` because the DSP chain is too slow for
//! an unoptimised debug build.  Run them in release mode:
//!
//! ```sh
//! cargo test --release -p fmradio --features test-fixtures -- --include-ignored
//! ```

#![cfg(feature = "test-fixtures")]

use desperado::dsp::{DspBlock, decimator::Decimator, filters::LowPassFir, rotate::Rotate};
use fmradio::fm::PhaseExtractor;
use fmradio::rds::{RdsDecoder, RdsGroupJson, RdsResamplerCustom, StereoDecoderPLL};
use std::f32::consts::PI;
use std::io::Read;
use std::path::Path;
use tracing::trace;

const SAMPLE_RATE: u32 = 1_102_500;
const FM_BANDWIDTH: f32 = 240_000.0;
const RDS_TARGET_RATE: f32 = 171_000.0;

fn fixture_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data/rtlsdr_20210215_103300000_1102500_cu8.bin")
}

/// Run the full FM+RDS DSP chain on the fixture and return all decoded groups.
fn decode_rds(offset_hz: i32) -> Vec<RdsGroupJson> {
    let path = fixture_path();
    let rotate_freq = -2.0 * PI * offset_hz as f32 / SAMPLE_RATE as f32;
    let mut rotate = Rotate::new(rotate_freq);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (SAMPLE_RATE as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);
    let actual_mpx_rate = SAMPLE_RATE as f32 / factor as f32;

    let mut stereo = StereoDecoderPLL::new(actual_mpx_rate);
    let mut rds_resampler = RdsResamplerCustom::new(actual_mpx_rate, RDS_TARGET_RATE);
    let mut rds = RdsDecoder::new(RDS_TARGET_RATE, true);
    rds.set_print_json_output(false);

    // Lowpass not used in stereo path but keeps the chain consistent with main.rs
    let _lowpass_fir = LowPassFir::new(15_000.0, actual_mpx_rate, 256);

    let mut input = std::fs::File::open(&path)
        .unwrap_or_else(|e| panic!("cannot open {}: {e}", path.display()));

    let chunk_samples = 65_536usize;
    let chunk_bytes = chunk_samples * 2; // cu8: 2 bytes per IQ sample
    let mut groups: Vec<RdsGroupJson> = Vec::new();
    let mut chunk_idx = 0usize;

    trace!(
        "[DIAG] factor={}, actual_mpx_rate={}",
        factor, actual_mpx_rate
    );

    loop {
        let mut buf = vec![0u8; chunk_bytes];
        let n = input.read(&mut buf).expect("read error");
        if n == 0 {
            break;
        }
        buf.truncate(n);

        let samples: Vec<num_complex::Complex<f32>> = buf
            .chunks_exact(2)
            .map(|b| {
                let i = (b[0] as f32 - 127.5) / 127.5;
                let q = (b[1] as f32 - 127.5) / 127.5;
                num_complex::Complex::new(i, q)
            })
            .collect();

        let shifted = rotate.process(&samples);
        let decimated = decimator.process(&shifted);
        let phase = phase_extractor.process(&decimated);
        let (_, _, pilot_phases) = stereo.process(&phase);
        let (rds_i, rds_q) = rds_resampler.process_with_pilot(&phase, &pilot_phases);

        if chunk_idx < 3 {
            let phase_rms =
                (phase.iter().map(|x| x * x).sum::<f32>() / phase.len().max(1) as f32).sqrt();
            // Check 19 kHz pilot energy via pilot_phases rate of change
            let mut pilot_freq_sum = 0.0_f64;
            let mut pilot_count = 0usize;
            for i in 1..pilot_phases.len().min(1000) {
                let mut delta = pilot_phases[i] - pilot_phases[i - 1];
                if delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                }
                if delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                pilot_freq_sum += delta;
                pilot_count += 1;
            }
            let pilot_freq = if pilot_count > 0 {
                pilot_freq_sum / pilot_count as f64 * actual_mpx_rate as f64
                    / (2.0 * std::f64::consts::PI)
            } else {
                0.0
            };

            let rds_i_rms = if !rds_i.is_empty() {
                (rds_i.iter().map(|x| x * x).sum::<f32>() / rds_i.len() as f32).sqrt()
            } else {
                0.0
            };
            let rds_q_rms = if !rds_q.is_empty() {
                (rds_q.iter().map(|x| x * x).sum::<f32>() / rds_q.len() as f32).sqrt()
            } else {
                0.0
            };

            trace!(
                "[DIAG] chunk {}: samples={}, decimated={}, phase={}, pilot_phases={}, pilot_freq={:.1}Hz, phase_rms={:.4}, rds_i_len={}, rds_i_rms={:.6}, rds_q_rms={:.6}",
                chunk_idx,
                samples.len(),
                decimated.len(),
                phase.len(),
                pilot_phases.len(),
                pilot_freq,
                phase_rms,
                rds_i.len(),
                rds_i_rms,
                rds_q_rms
            );
        }

        if !rds_i.is_empty() {
            rds.process_iq(&rds_i, &rds_q);
        }
        groups.extend(rds.take_json_outputs());

        // Print intermediate stats after chunk 50
        if chunk_idx == 50 {
            let (bits_pushed, blocks_received, groups_decoded, _) = rds.stats();
            trace!(
                "[DIAG] After chunk 50: bits_pushed={}, blocks={}, groups={}",
                bits_pushed, blocks_received, groups_decoded
            );
        }

        chunk_idx += 1;
    }

    let (bits_pushed, blocks_received, groups_decoded, _) = rds.stats();
    trace!(
        "[DIAG] Final: {} chunks, {} groups, bits_pushed={}, blocks={}, groups_decoded={}",
        chunk_idx,
        groups.len(),
        bits_pushed,
        blocks_received,
        groups_decoded
    );

    groups
}

// ── FIP (103.5 MHz, offset +200 kHz) ─────────────────────────────────────────

#[test]
#[ignore]
fn test_fip_pi_code() {
    let groups = decode_rds(200_000);
    let pi = groups
        .iter()
        .map(|g| g.pi.as_str())
        .next()
        .expect("no groups decoded for FIP");
    assert_eq!(pi, "0xF204", "FIP PI mismatch: got {pi}");
}

#[test]
#[ignore]
fn test_fip_station_name() {
    let groups = decode_rds(200_000);
    let ps = groups
        .iter()
        .rev()
        .find_map(|g| g.ps.as_deref())
        .expect("no PS decoded for FIP");
    assert_eq!(ps.trim(), "F I P", "FIP PS mismatch: got '{ps}'");
}

#[test]
#[ignore]
fn test_fip_radiotext_full() {
    let groups = decode_rds(200_000);
    let rt = groups
        .iter()
        .rev()
        .find_map(|g| g.rt.as_deref())
        .expect("no RT decoded for FIP");
    assert_eq!(
        rt, "MOON RIVER - MELODY GARDOT (2020) - FIP",
        "FIP RT mismatch: got '{rt}'"
    );
}

#[test]
#[ignore]
fn test_fip_tp_flag() {
    let groups = decode_rds(200_000);
    let tp = groups
        .iter()
        .find_map(|g| g.tp)
        .expect("no TP field in FIP groups");
    assert!(tp, "FIP TP should be true");
}

#[test]
#[ignore]
fn test_fip_af_includes_own_frequency() {
    let groups = decode_rds(200_000);
    let af = groups
        .iter()
        .rev()
        .find_map(|g| g.af.as_ref().filter(|v| !v.is_empty()))
        .expect("no AF list decoded for FIP");
    assert!(
        af.iter().any(|f| f == "103.5"),
        "FIP AF should contain 103.5 MHz, got: {af:?}"
    );
}

// ── CLASSIQ (103.1 MHz, offset −200 kHz) ─────────────────────────────────────

#[test]
#[ignore]
fn test_classiq_pi_code() {
    let groups = decode_rds(-200_000);
    let pi = groups
        .iter()
        .map(|g| g.pi.as_str())
        .next()
        .expect("no groups decoded for CLASSIQ");
    assert_eq!(pi, "0xF221", "CLASSIQ PI mismatch: got {pi}");
}

#[test]
#[ignore]
fn test_classiq_station_name() {
    let groups = decode_rds(-200_000);
    let ps = groups
        .iter()
        .rev()
        .find_map(|g| g.ps.as_deref())
        .expect("no PS decoded for CLASSIQ");
    assert_eq!(ps.trim(), "CLASSIQ", "CLASSIQ PS mismatch: got '{ps}'");
}

#[test]
#[ignore]
fn test_classiq_radiotext_full() {
    let groups = decode_rds(-200_000);
    let rt = groups
        .iter()
        .rev()
        .find_map(|g| g.rt.as_deref())
        .expect("no RT decoded for CLASSIQ");
    assert_eq!(
        rt, "Satie: Gnossienne n 1",
        "CLASSIQ RT mismatch: got '{rt}'"
    );
}

#[test]
#[ignore]
fn test_classiq_tp_flag() {
    let groups = decode_rds(-200_000);
    let tp = groups
        .iter()
        .find_map(|g| g.tp)
        .expect("no TP field in CLASSIQ groups");
    assert!(tp, "CLASSIQ TP should be true");
}

#[test]
#[ignore]
fn test_classiq_af_includes_own_frequency() {
    let groups = decode_rds(-200_000);
    let af = groups
        .iter()
        .rev()
        .find_map(|g| g.af.as_ref().filter(|v| !v.is_empty()))
        .expect("no AF list decoded for CLASSIQ");
    assert!(
        af.iter().any(|f| f == "103.1"),
        "CLASSIQ AF should contain 103.1 MHz, got: {af:?}"
    );
}
