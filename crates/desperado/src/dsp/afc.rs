//! Adaptive Frequency Correction using Squared Signal FFT
//!

use std::f32::consts::PI;

use num_complex::Complex;

use crate::dsp::DspBlock;

/// Adaptive frequency correction block using squared-signal FFT peak detection.
///
/// Corrects residual carrier frequency offsets in OFDM-like signals by squaring
/// the input (to double the carrier offset), performing an FFT, locating the
/// peak pair, and applying a compensating complex rotation.
///
/// Used in FM radio for fine frequency offset correction after initial tuning.
pub struct SquareFreqOffsetCorrection {
    /// Buffer for squared input samples for FFT
    fft_data: Vec<Complex<f32>>,
    /// Buffer for output samples after correction
    output: Vec<Complex<f32>>,
    /// Cumulative sum buffer for wide search
    cumsum: Vec<f32>,
    /// Current complex rotation factor
    rot: Complex<f32>,
    /// FFT size (number of samples per window)
    n: usize,
    /// log2(n), used for bit-reversal and FFT
    log_n: usize,
    /// Current sample count in the window
    count: usize,
    /// Search window size for peak detection
    window: usize,
    /// Enables wide search mode for larger offsets
    wide: bool,
}

impl SquareFreqOffsetCorrection {
    /// Create a new frequency correction block.
    ///
    /// # Arguments
    /// * `n` - FFT size (number of samples per correction window, must be a power of 2)
    /// * `window` - Search window size for peak detection (bins to exclude at edges)
    /// * `wide` - Enable wide search mode for larger frequency offsets
    pub fn with_params(n: usize, window: usize, wide: bool) -> Self {
        let log_n = (n as f32).log2() as usize;
        Self {
            fft_data: vec![Complex::new(0.0, 0.0); n],
            output: vec![Complex::new(0.0, 0.0); n],
            cumsum: vec![0.0; n],
            rot: Complex::new(1.0, 0.0),
            n,
            log_n,
            count: 0,
            window,
            wide,
        }
    }

    fn correct_frequency(&mut self) -> f32 {
        let n = self.n;

        // Perform FFT on squared signal
        fft_in_place(&mut self.fft_data);

        let delta = (9600.0 / 48000.0 * n as f32) as usize;
        let mut wi: isize = 0;

        if self.wide {
            if self.cumsum.len() < n {
                self.cumsum.resize(n, 0.0);
            }

            let m = (12500.0 / 48000.0 * n as f32) as usize;
            let ofs = (m - delta) / 2;
            let mut wm = -1.0f32;

            self.cumsum[0] = 0.0;
            for i in 1..n {
                let p = self.fft_data[(i + n / 2) % n].norm();
                self.cumsum[i] = self.cumsum[i - 1] + p;
            }

            let mut best_i: usize = 0;
            for i in 0..(n - m) {
                let v = self.cumsum[i + m] - self.cumsum[i]
                    + 0.6
                        * (self.fft_data[(i + ofs + n / 2) % n].norm()
                            + self.fft_data[(i + ofs + delta + n / 2) % n].norm());
                if v > wm {
                    wm = v;
                    best_i = i;
                }
            }
            wi = best_i as isize + (m / 2) as isize - (n / 2) as isize;
        }

        let mut max_val = 0.0f32;
        let mut fz = -1.0f32;

        // Search range uses signed arithmetic; clamp to valid [0, n) range.
        let lo = (wi + self.window as isize).max(0) as usize;
        let hi = (wi + n as isize - self.window as isize - delta as isize).max(0) as usize;
        for i in lo..hi.min(n) {
            let h = self.fft_data[(i + n / 2) % n].norm()
                + self.fft_data[(i + delta + n / 2) % n].norm();

            if h > max_val {
                max_val = h;
                fz = (n / 2) as f32 - (i as f32 + delta as f32 / 2.0);
            }
        }

        let f = fz / 2.0 / n as f32;
        let rot_step = Complex::new(0.0, f * 2.0 * PI).exp();

        for i in 0..n {
            self.rot *= rot_step;
            self.output[i] *= self.rot;
        }

        self.rot /= self.rot.norm();

        f * 48000.0 / 162.0
    }
}

impl DspBlock for SquareFreqOffsetCorrection {
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut result = Vec::new();
        self.correct_frequency();

        if self.fft_data.len() < self.n {
            self.fft_data.resize(self.n, Complex::new(0.0, 0.0));
        }
        if self.output.len() < self.n {
            self.output.resize(self.n, Complex::new(0.0, 0.0));
        }

        for &sample in data {
            // Store squared sample at bit-reversed position
            let pos = reverse_bits(self.count, self.log_n);
            self.fft_data[pos] = sample * sample;
            self.output[self.count] = sample;

            self.count += 1;

            if self.count == self.n {
                result.extend_from_slice(&self.output);
                self.count = 0;
            }
        }

        result
    }
}

fn reverse_bits(mut n: usize, bits: usize) -> usize {
    let mut result = 0;
    for _ in 0..bits {
        result = (result << 1) | (n & 1);
        n >>= 1;
    }
    result
}

fn fft_in_place(data: &mut [Complex<f32>]) {
    let n = data.len();
    if n <= 1 {
        return;
    }

    let log_n = (n as f32).log2() as usize;

    // Pre-compute twiddle factors
    let mut omega: Vec<Complex<f32>> = Vec::with_capacity(n);
    for s in 0..n {
        let angle = -2.0 * PI * s as f32 / n as f32;
        omega.push(Complex::new(angle.cos(), angle.sin()));
    }

    // Radix-2 FFT
    let mut m = 2;
    let mut m2 = 1;
    let mut r = n;

    for _ in 0..log_n {
        let mut w = 0;
        r >>= 1;

        for j in 0..m2 {
            let o = omega[w];

            let mut k = 0;
            while k < n {
                let t = o * data[k + j + m2];
                data[k + j + m2] = data[k + j] - t;
                data[k + j] += t;
                k += m;
            }

            w += r;
        }

        m2 = m;
        m <<= 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_afc_creation() {
        let afc = SquareFreqOffsetCorrection::with_params(1024, 10, false);
        assert_eq!(afc.n, 1024);
        assert_eq!(afc.window, 10);
        assert!(!afc.wide);
        assert_eq!(afc.count, 0);
    }

    #[test]
    fn test_afc_process_empty() {
        let mut afc = SquareFreqOffsetCorrection::with_params(1024, 10, false);
        let result = afc.process(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_afc_process_partial_window() {
        let mut afc = SquareFreqOffsetCorrection::with_params(1024, 10, false);
        // Feed fewer samples than the FFT window — no output expected
        let input: Vec<Complex<f32>> = (0..512)
            .map(|i| Complex::new((i as f32 * 0.1).cos(), (i as f32 * 0.1).sin()))
            .collect();
        let result = afc.process(&input);
        assert!(result.is_empty());
    }

    #[test]
    fn test_afc_process_full_window() {
        let mut afc = SquareFreqOffsetCorrection::with_params(1024, 10, false);
        // Feed exactly one FFT window — should produce output
        let input: Vec<Complex<f32>> = (0..1024)
            .map(|i| Complex::new((i as f32 * 0.1).cos(), (i as f32 * 0.1).sin()))
            .collect();
        let result = afc.process(&input);
        assert_eq!(result.len(), 1024);
        // Output should contain finite values
        assert!(result.iter().all(|c| c.re.is_finite() && c.im.is_finite()));
    }

    #[test]
    fn test_afc_wide_mode() {
        let mut afc = SquareFreqOffsetCorrection::with_params(1024, 10, true);
        let input: Vec<Complex<f32>> = (0..1024)
            .map(|i| Complex::new((i as f32 * 0.1).cos(), (i as f32 * 0.1).sin()))
            .collect();
        let result = afc.process(&input);
        assert_eq!(result.len(), 1024);
        assert!(result.iter().all(|c| c.re.is_finite() && c.im.is_finite()));
    }

    #[test]
    fn test_fft_in_place_dc() {
        // DC signal — all energy in bin 0
        let n = 8;
        let mut data = vec![Complex::new(1.0, 0.0); n];
        fft_in_place(&mut data);
        // Bin 0 should have magnitude n, rest should be ~0
        assert!((data[0].norm() - n as f32).abs() < 1e-4);
        for bin in &data[1..] {
            assert!(bin.norm() < 1e-4);
        }
    }

    #[test]
    fn test_reverse_bits() {
        assert_eq!(reverse_bits(0b000, 3), 0b000);
        assert_eq!(reverse_bits(0b001, 3), 0b100);
        assert_eq!(reverse_bits(0b010, 3), 0b010);
        assert_eq!(reverse_bits(0b110, 3), 0b011);
    }
}
