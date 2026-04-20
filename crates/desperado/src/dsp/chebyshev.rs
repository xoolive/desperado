use std::f32::consts::PI;

/// Stateful 2-pole Chebyshev low-pass filter for streaming samples.
#[derive(Debug, Clone)]
pub struct Chebyshev2Lpf {
    a: [f32; 3],
    b: [f32; 3],
    xn: [f32; 3],
    yn: [f32; 2],
}

impl Chebyshev2Lpf {
    /// Build a 2-pole type-I Chebyshev LPF.
    ///
    /// `cutoff_freq` is normalized to sample rate (0.0..0.5).
    /// `ripple_pct` is passband ripple in percent.
    pub fn new(cutoff_freq: f32, ripple_pct: f32) -> Self {
        let (a, b) = chebyshev_lpf_init(cutoff_freq, ripple_pct, 2);
        Self {
            a,
            b,
            xn: [0.0; 3],
            yn: [0.0; 2],
        }
    }

    /// Process one real sample and return the filtered sample.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        self.xn = [x, self.xn[0], self.xn[1]];
        let y = lpf_step(&self.a, &self.b, &self.xn, &self.yn);
        self.yn = [y, self.yn[0]];
        y
    }

    /// Reset filter memory.
    pub fn reset(&mut self) {
        self.xn = [0.0; 3];
        self.yn = [0.0; 2];
    }
}

/// Compute feed-forward (A) and feedback (B) coefficients for a Chebyshev
/// type-I lowpass IIR filter with `npoles` poles.
///
/// `cutoff_freq` is normalized to sample rate (0.0..0.5).
fn chebyshev_lpf_init(cutoff_freq: f32, ripple_pct: f32, npoles: usize) -> ([f32; 3], [f32; 3]) {
    assert!(npoles == 2, "only 2-pole filter implemented");
    assert!((npoles & 1) == 0);

    let mut a = [0.0f32; 5];
    let mut b = [0.0f32; 5];
    a[2] = 1.0;
    b[2] = 1.0;

    for p in 1..=(npoles / 2) {
        let (aa, bb) = chebyshev_calc_pole(p, cutoff_freq, ripple_pct, npoles);
        let ta = a;
        let tb = b;
        for i in 2..5 {
            a[i] = aa[0] * ta[i] + aa[1] * ta[i - 1] + aa[2] * ta[i - 2];
            b[i] = tb[i] - bb[1] * tb[i - 1] - bb[2] * tb[i - 2];
        }
    }

    b[2] = 0.0;
    let mut out_a = [0.0f32; 3];
    let mut out_b = [0.0f32; 3];
    for i in 0..3 {
        out_a[i] = a[i + 2];
        out_b[i] = -b[i + 2];
    }
    out_b[0] = 0.0;

    let sa: f32 = out_a.iter().sum();
    let sb: f32 = out_b.iter().sum();
    let gain = sa / (1.0 - sb);
    for x in &mut out_a {
        *x /= gain;
    }
    (out_a, out_b)
}

/// 2-pole Chebyshev IIR filter applied to a real-valued signal sample.
///
/// `xn` = input history [x(n), x(n-1), x(n-2)]
/// `yn` = output history [y(n-1), y(n-2)]
#[inline]
fn lpf_step(a: &[f32; 3], b: &[f32; 3], xn: &[f32; 3], yn: &[f32; 2]) -> f32 {
    a[0] * xn[0] + a[1] * xn[1] + a[2] * xn[2] + b[1] * yn[0] + b[2] * yn[1]
}

fn chebyshev_calc_pole(p: usize, cutoff: f32, ripple: f32, npoles: usize) -> ([f32; 3], [f32; 3]) {
    let angle = PI / (2.0 * npoles as f32) + (p as f32 - 1.0) * PI / npoles as f32;
    let (ip_raw, rp_raw) = angle.sin_cos();
    let mut rp = -rp_raw;
    let mut ip = ip_raw;

    if ripple != 0.0 {
        let es = ((100.0 / (100.0 - ripple)).powi(2) - 1.0).sqrt();
        let vx = (1.0 / npoles as f32) * ((1.0 / es) + ((1.0 / (es * es)) + 1.0).sqrt()).ln();
        let kx = (1.0 / npoles as f32) * ((1.0 / es) + ((1.0 / (es * es)) - 1.0).sqrt()).ln();
        let kx_val = (kx.exp() + (-kx).exp()) / 2.0;
        let sinh_vx = (vx.exp() - (-vx).exp()) / 2.0;
        let cosh_vx = (vx.exp() + (-vx).exp()) / 2.0;
        rp = -rp_raw * sinh_vx / kx_val;
        ip = ip_raw * cosh_vx / kx_val;
    }

    let t = 2.0 * (0.5f32).tan();
    let w = 2.0 * PI * cutoff;
    let m = rp * rp + ip * ip;
    let d = 4.0 - 4.0 * rp * t + m * t * t;
    let x0 = t * t / d;
    let x1 = 2.0 * x0;
    let x2 = x0;
    let y1 = (8.0 - 2.0 * m * t * t) / d;
    let y2 = (-4.0 - 4.0 * rp * t - m * t * t) / d;

    let k = (0.5 - w / 2.0).sin() / (0.5 + w / 2.0).sin();
    let d2 = 1.0 + y1 * k - y2 * k * k;

    let aa = [
        (x0 - x1 * k + x2 * k * k) / d2,
        (-2.0 * x0 * k + x1 + x1 * k * k - 2.0 * x2 * k) / d2,
        (x0 * k * k - x1 * k + x2) / d2,
    ];
    let bb = [
        0.0,
        (2.0 * k + y1 + y1 * k * k - 2.0 * y2 * k) / d2,
        (-(k * k) - y1 * k + y2) / d2,
    ];
    (aa, bb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chebyshev_coeffs_are_finite() {
        let (a, b) = chebyshev_lpf_init(8_000.0 / 1_050_000.0, 0.5, 2);
        assert!(a.iter().all(|v| v.is_finite()));
        assert!(b.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn chebyshev_filter_stateful_stream() {
        let mut f = Chebyshev2Lpf::new(8_000.0 / 1_050_000.0, 0.5);
        let mut last = 0.0f32;
        for _ in 0..1024 {
            last = f.step(1.0);
            assert!(last.is_finite());
        }
        assert!(last > 0.0);
        f.reset();
        let y = f.step(0.0);
        assert!(y.abs() < 1e-6);
    }

    #[test]
    fn chebyshev_dc_converges_to_unity() {
        // A low-pass filter fed DC should converge to 1.0
        let mut f = Chebyshev2Lpf::new(0.1, 0.5); // generous cutoff
        let mut last = 0.0f32;
        for _ in 0..2000 {
            last = f.step(1.0);
        }
        assert!((last - 1.0).abs() < 0.05, "Expected ~1.0, got {}", last);
    }

    #[test]
    fn chebyshev_rejects_above_cutoff() {
        // Very low cutoff — a fast oscillation should be attenuated
        let cutoff = 0.01; // normalized
        let mut f = Chebyshev2Lpf::new(cutoff, 0.5);
        // Feed a tone well above cutoff (0.4 Nyquist)
        let mut max_out = 0.0f32;
        for i in 0..2000 {
            let x = (std::f32::consts::PI * 0.8 * i as f32).sin();
            let y = f.step(x);
            if i > 200 {
                max_out = max_out.max(y.abs());
            }
        }
        assert!(
            max_out < 0.1,
            "High-freq tone should be attenuated, got {}",
            max_out
        );
    }
}
