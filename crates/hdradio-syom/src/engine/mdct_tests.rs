//! Fast forward MDCT vs the naive analysis sum, plus the TDAC roundtrip
//! (window → MDCT → IMDCT → window + overlap-add reconstructs the input).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::filterbank::window_left;
use super::super::ics::WindowShape;
use super::super::imdct::imdct;
use super::{mdct_into_f32, mdct_naive};

const N: usize = 2048;

/// Deterministic smooth-ish signal in [-1, 1].
fn test_signal(len: usize) -> Vec<f64> {
    (0..len)
        .map(|i| {
            let t = i as f64;
            0.6 * (t * 0.017).sin() + 0.3 * (t * 0.111).cos() + 0.1 * (t * 0.005).sin()
        })
        .collect()
}

fn sine_window() -> Vec<f64> {
    let half = window_left(N, WindowShape::Sine);
    (0..N)
        .map(|i| {
            if i < N / 2 {
                f64::from(half[i])
            } else {
                f64::from(half[N - 1 - i])
            }
        })
        .collect()
}

fn mdct_fast_f64(time: &[f64]) -> Vec<f64> {
    let t32: Vec<f32> = time.iter().map(|&x| x as f32).collect();
    let mut spec = vec![0.0f32; time.len() / 2];
    mdct_into_f32(&t32, &mut spec);
    spec.iter().map(|&x| f64::from(x)).collect()
}

#[test]
fn fast_matches_naive_long() {
    let sig = test_signal(N);
    let a = mdct_naive(&sig, N);
    let b = mdct_fast_f64(&sig);
    let err = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max);
    assert!(err < 1e-3, "fast MDCT drifted from the naive sum: {err}");
}

#[test]
fn tdac_reconstructs_input() {
    let sig = test_signal(3 * (N / 2));
    let w = sine_window();
    let frame = |off: usize| -> Vec<f64> { (0..N).map(|i| sig[off + i] * w[i]).collect() };
    let spec0 = mdct_fast_f64(&frame(0));
    let spec1 = mdct_fast_f64(&frame(N / 2));
    let y0 = imdct(&spec0, N);
    let y1 = imdct(&spec1, N);
    let mut err = 0.0f64;
    for i in 0..N / 2 {
        let rec = y0[N / 2 + i] * w[N / 2 + i] + y1[i] * w[i];
        err = err.max((rec - sig[N / 2 + i]).abs());
    }
    assert!(err < 1e-3, "TDAC reconstruction error {err}");
}

#[test]
fn zero_input_is_silence() {
    let spec = mdct_fast_f64(&vec![0.0; N]);
    assert!(spec.iter().all(|&x| x.abs() < 1e-9));
}

const N_S: usize = 256;

fn short_window() -> Vec<f64> {
    let half = window_left(N_S, WindowShape::Sine);
    (0..N_S)
        .map(|i| {
            if i < N_S / 2 {
                f64::from(half[i])
            } else {
                f64::from(half[N_S - 1 - i])
            }
        })
        .collect()
}

#[test]
fn fast_matches_naive_short() {
    let sig = test_signal(N_S);
    let a = mdct_naive(&sig, N_S);
    let b = mdct_fast_f64(&sig);
    let err = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max);
    assert!(
        err < 1e-3,
        "fast short MDCT drifted from the naive sum: {err}"
    );
}

#[test]
fn tdac_reconstructs_input_short() {
    // Eight-short hop is 128: each output sample is the overlap of two
    // 256-sample windows (like the decoder's `short_windowed`).
    let sig = test_signal(N_S + 3 * (N_S / 2));
    let w = short_window();
    let frame = |off: usize| -> Vec<f64> { (0..N_S).map(|i| sig[off + i] * w[i]).collect() };
    let mut prev = imdct(&mdct_fast_f64(&frame(0)), N_S);
    let mut err = 0.0f64;
    for hop in 1..=3 {
        let cur = imdct(&mdct_fast_f64(&frame(hop * N_S / 2)), N_S);
        for i in 0..N_S / 2 {
            let rec = prev[N_S / 2 + i] * w[N_S / 2 + i] + cur[i] * w[i];
            err = err.max((rec - sig[(hop - 1) * (N_S / 2) + N_S / 2 + i]).abs());
        }
        prev = cur;
    }
    assert!(err < 1e-3, "short TDAC reconstruction error {err}");
}
