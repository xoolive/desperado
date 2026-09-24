//! Forward MDCT — analysis side of ISO/IEC 14496-3 §4.6.11.3.1, built as the
//! adjoint of the [`super::imdct`] fast path (same twiddle angles, computed
//! via [`super::det_math`] for cross-platform bit-identical output, and
//! conjugated; the FFT stage runs with negated imaginary twiddles).
//!
//! `X[k] = 2 · Σ_n x[n] · cos((2π/N)(n + n0)(k + 1/2))`, `n0 = N/4 + 1/2`.
//! The factor 2 makes window → MDCT → IMDCT → window + overlap-add
//! reconstruct the input (TDAC); the scale is pinned by `mdct_tests`.

use super::det_math;
use super::imdct::{bitrev_table, ifft_soa};
use std::sync::LazyLock;

struct Plan {
    n: usize,
    /// cos/sin of `2π/N·(k + 1/8)` — the imdct pre/post twiddle angles.
    tw_c: Vec<f32>,
    tw_s: Vec<f32>,
    bitrev: Vec<u16>,
    /// Forward-FFT twiddles (`twiddle_table` with imaginary parts negated).
    fft_re: Vec<f32>,
    fft_im: Vec<f32>,
}

static PLAN_2048: LazyLock<Plan> = LazyLock::new(|| Plan::new(2048));
static PLAN_256: LazyLock<Plan> = LazyLock::new(|| Plan::new(256));

impl Plan {
    fn new(n: usize) -> Self {
        let n4 = n / 4;
        let two_pi_n = 2.0 * std::f32::consts::PI / n as f32;
        let mut tw_c = Vec::with_capacity(n4);
        let mut tw_s = Vec::with_capacity(n4);
        for k in 0..n4 {
            let a = two_pi_n * (k as f32 + 0.125);
            // det_math twiddles: libm sin/cos differ by 1 ulp across
            // platforms, which would drift the encoded bytes.
            let (s, c) = det_math::sincos(a);
            tw_c.push(c);
            tw_s.push(s);
        }
        let (fft_re, fft_im) = det_math::twiddle_table(n4);
        let fft_im = fft_im.iter().map(|&x| -x).collect();
        Self {
            n,
            tw_c,
            tw_s,
            bitrev: bitrev_table(n4),
            fft_re,
            fft_im,
        }
    }

    /// `time` (N windowed samples) → `spec` (N/2 coefficients).
    fn apply(&self, time: &[f32], spec: &mut [f32]) {
        let n = self.n;
        let n2 = n / 2;
        let n4 = n / 4;
        let h = n4 / 2;
        let mut re = vec![0.0f32; n4];
        let mut im = vec![0.0f32; n4];
        // Adjoint of the imdct output fold/scatter, with the TDAC factor 2
        // in place of the imdct's 2/N.
        for p in 0..h {
            let u_even = time[2 * p] - time[n2 - 1 - 2 * p];
            let u_odd = time[2 * p + 1] - time[n2 - 2 - 2 * p];
            let v_even = time[n2 + 2 * p] + time[n - 1 - 2 * p];
            let v_odd = time[n2 + 2 * p + 1] + time[n - 2 - 2 * p];
            re[h + p] = 2.0 * v_even;
            im[h + p] = 2.0 * u_even;
            re[h - 1 - p] = -2.0 * u_odd;
            im[h - 1 - p] = -2.0 * v_odd;
        }
        // Adjoint of the post-twiddle: multiply by conj(e^{i·a}).
        for k in 0..n4 {
            let (c, s) = (self.tw_c[k], self.tw_s[k]);
            let (r, i) = (re[k], im[k]);
            re[k] = r * c + i * s;
            im[k] = i * c - r * s;
        }
        ifft_soa(&mut re, &mut im, &self.bitrev, &self.fft_re, &self.fft_im);
        // Adjoint of the pre-twiddle + input gather.
        for k in 0..n4 {
            let (c, s) = (self.tw_c[k], self.tw_s[k]);
            let (r, i) = (re[k], im[k]);
            spec[n2 - 1 - 2 * k] = r * c + i * s;
            spec[2 * k] = i * c - r * s;
        }
    }
}

/// Naive §4.6.11.3.1 analysis sum (f64), `X[k] = 2·Σ x[n]·cos(...)`.
#[cfg(test)]
#[must_use]
pub fn mdct_naive(time: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n / 2];
    naive_f64_into(time, &mut out);
    out
}

fn naive_f64_into(time: &[f64], spec: &mut [f64]) {
    let n = time.len();
    if n == 0 {
        return;
    }
    let n0 = (n / 2 + 1) as f64 / 2.0;
    let step = 2.0 * std::f64::consts::PI / n as f64;
    for (k, slot) in spec.iter_mut().enumerate() {
        let mut acc = 0.0f64;
        let kf = k as f64 + 0.5;
        for (n_i, &x) in time.iter().enumerate() {
            acc += x * (step * (n_i as f64 + n0) * kf).cos();
        }
        *slot = 2.0 * acc;
    }
}

fn naive_f32(time: &[f32], spec: &mut [f32]) {
    let t64: Vec<f64> = time.iter().copied().map(f64::from).collect();
    let mut tmp = vec![0.0f64; spec.len()];
    naive_f64_into(&t64, &mut tmp);
    for (d, s) in spec.iter_mut().zip(tmp) {
        *d = s as f32;
    }
}

/// Forward MDCT: `time` (N windowed samples) → `spec` (N/2 coefficients).
pub fn mdct_into_f32(time: &[f32], spec: &mut [f32]) {
    if time.len() == 2048 && spec.len() == 1024 {
        PLAN_2048.apply(time, spec);
    } else if time.len() == 256 && spec.len() == 128 {
        PLAN_256.apply(time, spec);
    } else {
        naive_f32(time, spec);
    }
}

#[cfg(test)]
#[path = "mdct_tests.rs"]
mod mdct_tests;
