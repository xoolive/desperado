//! Fast IMDCT vs the naive §4.6.11.3.1 sum.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{imdct, imdct_naive};

fn max_err(n: usize, spec: &[f64]) -> f64 {
    let a = imdct_naive(spec, n);
    let b = imdct(spec, n);
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

#[test]
fn fast_matches_naive_short() {
    let spec: Vec<f64> = (0..128)
        .map(|k| ((k * 13) % 17) as f64 * 0.1 - 0.8)
        .collect();
    assert!(
        max_err(256, &spec) < 1e-3,
        "short IMDCT drifted from §4.6.11.3.1"
    );
}

#[test]
fn fast_matches_naive_long_impulse() {
    let mut spec = vec![0.0f64; 1024];
    spec[4] = 1.0;
    assert!(
        max_err(2048, &spec) < 1e-3,
        "long IMDCT drifted from §4.6.11.3.1"
    );
}

#[test]
fn naive_zero_is_silence() {
    let out = imdct_naive(&[0.0; 1024], 2048);
    assert!(out.iter().all(|&x| x.abs() < 1e-18));
}
