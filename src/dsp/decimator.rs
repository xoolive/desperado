/// Decimator with anti-aliasing filter.
///
/// This module provides a decimator that reduces the sample rate by an integer factor,
/// applying a low-pass FIR filter before decimation to prevent aliasing.
///
/// The decimator uses a Hamming-windowed sinc filter with a cutoff frequency
/// set to half the Nyquist frequency of the decimated rate.
///
/// # Example
///
/// ```rust
/// use desperado::dsp::decimator::Decimator;
/// use desperado::dsp::DspBlock;
/// use num_complex::Complex;
///
/// // Decimate by a factor of 8
/// let mut decimator = Decimator::new(8);
///
/// let input: Vec<Complex<f32>> = (0..1024)
///     .map(|i| Complex::new(i as f32, 0.0))
///     .collect();
///
/// let output = decimator.process(&input);
/// // Output length is approximately input.len() / factor
/// assert!(output.len() >= 120 && output.len() <= 130);
/// ```
use num_complex::Complex;
use std::f32::consts::PI;

use super::DspBlock;

/// A decimator that reduces the sample rate by an integer factor.
///
/// The decimator applies a Hamming-windowed sinc low-pass filter
/// before downsampling to prevent aliasing artifacts.
///
/// # Fields
/// - `factor`: The decimation factor (output rate = input rate / factor)
/// - `fir`: The FIR filter coefficients
/// - `buffer`: Internal buffer for maintaining state between process() calls
pub struct Decimator {
    factor: usize,
    fir: Vec<f32>,
    buffer: Vec<Complex<f32>>,
}

impl Decimator {
    /// Creates a new decimator with the specified decimation factor.
    ///
    /// # Arguments
    /// * `factor` - The decimation factor (must be > 0)
    ///
    /// # Panics
    /// Panics if `factor` is 0.
    ///
    /// # Example
    /// ```rust
    /// use desperado::dsp::decimator::Decimator;
    ///
    /// let decimator = Decimator::new(4);
    /// ```
    pub fn new(factor: usize) -> Self {
        assert!(factor > 0, "Decimation factor must be greater than 0");

        // Design anti-aliasing filter
        let taps = 31;
        let cutoff = 0.5 / factor as f32; // Normalized cutoff (Nyquist = 0.5)
        let mut fir = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;

        // Hamming-windowed sinc filter
        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * cutoff
            } else {
                (2.0 * cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            // Hamming window
            let window = 0.54 - 0.46 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();
            fir.push(sinc * window);
        }

        // Normalize filter to unity gain at DC
        let norm: f32 = fir.iter().sum();
        for v in fir.iter_mut() {
            *v /= norm;
        }

        Self {
            factor,
            fir,
            buffer: Vec::new(),
        }
    }

    /// Creates a new decimator with custom filter parameters.
    ///
    /// # Arguments
    /// * `factor` - The decimation factor (must be > 0)
    /// * `taps` - The number of FIR filter taps (should be odd)
    /// * `cutoff` - The normalized cutoff frequency (0.0 to 0.5)
    ///
    /// # Panics
    /// Panics if `factor` is 0 or if `cutoff` is not in (0.0, 0.5].
    ///
    /// # Example
    /// ```rust
    /// use desperado::dsp::decimator::Decimator;
    ///
    /// // Custom decimator with 63 taps and lower cutoff
    /// let decimator = Decimator::with_params(8, 63, 0.05);
    /// ```
    pub fn with_params(factor: usize, taps: usize, cutoff: f32) -> Self {
        assert!(factor > 0, "Decimation factor must be greater than 0");
        assert!(
            cutoff > 0.0 && cutoff <= 0.5,
            "Cutoff must be in range (0.0, 0.5]"
        );

        let mut fir = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;

        // Hamming-windowed sinc filter
        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * cutoff
            } else {
                (2.0 * cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            let window = 0.54 - 0.46 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();
            fir.push(sinc * window);
        }

        let norm: f32 = fir.iter().sum();
        for v in fir.iter_mut() {
            *v /= norm;
        }

        Self {
            factor,
            fir,
            buffer: Vec::new(),
        }
    }

    /// Returns the decimation factor.
    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Returns the number of FIR filter taps.
    pub fn taps(&self) -> usize {
        self.fir.len()
    }

    /// Clears the internal state buffer.
    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

impl DspBlock for Decimator {
    /// Processes input samples, applying anti-aliasing filter and decimation.
    ///
    /// The method maintains internal state to handle filtering across chunk boundaries.
    ///
    /// # Arguments
    /// * `data` - Input samples
    ///
    /// # Returns
    /// Decimated output samples (length ≈ input.len() / factor)
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>> {
        // Append new data to buffer
        self.buffer.extend_from_slice(data);

        let taps = self.fir.len();
        let mid = taps / 2;
        let mut output = Vec::new();

        // Process with filtering and decimation
        let mut i = mid;
        while i < self.buffer.len() {
            let mut acc = Complex::new(0.0, 0.0);

            // Apply FIR filter
            for j in 0..taps {
                let buf_idx = i + j - mid;
                if buf_idx < self.buffer.len() {
                    acc += self.buffer[buf_idx] * self.fir[j];
                }
            }

            output.push(acc);
            i += self.factor;
        }

        // Keep only the samples needed for the next iteration
        // We need to keep at least `taps` samples for the filter state
        let keep = self.buffer.len().saturating_sub(mid);
        self.buffer.drain(0..keep);

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_decimator_new() {
        let dec = Decimator::new(4);
        assert_eq!(dec.factor(), 4);
        assert_eq!(dec.taps(), 31);
    }

    #[test]
    fn test_decimator_with_params() {
        let dec = Decimator::with_params(8, 63, 0.06);
        assert_eq!(dec.factor(), 8);
        assert_eq!(dec.taps(), 63);
    }

    #[test]
    #[should_panic(expected = "Decimation factor must be greater than 0")]
    fn test_decimator_zero_factor() {
        let _ = Decimator::new(0);
    }

    #[test]
    #[should_panic(expected = "Cutoff must be in range")]
    fn test_decimator_invalid_cutoff() {
        let _ = Decimator::with_params(4, 31, 0.6);
    }

    #[test]
    fn test_decimator_length() {
        let mut dec = Decimator::new(4);
        let input: Vec<Complex<f32>> = (0..1024).map(|i| Complex::new(i as f32, 0.0)).collect();
        let output = dec.process(&input);

        // Output length should be approximately input.len() / factor
        // (may vary slightly due to filter edge effects)
        assert!(output.len() >= 240 && output.len() <= 260);
    }

    #[test]
    fn test_decimator_dc_signal() {
        let mut dec = Decimator::new(4);

        // DC signal (constant value) - need more samples for filter to settle
        let input = vec![Complex::new(1.0, 0.0); 4096];
        let output = dec.process(&input);

        // After the filter settles (skip more initial samples), output should be close to 1.0
        let settled_samples: Vec<_> = output.iter().skip(20).take(output.len() - 30).collect();
        assert!(!settled_samples.is_empty(), "Should have settled samples");

        for sample in settled_samples {
            assert_relative_eq!(sample.re, 1.0, epsilon = 0.15);
            assert_relative_eq!(sample.im, 0.0, epsilon = 0.15);
        }
    }

    #[test]
    fn test_decimator_stateful() {
        let mut dec = Decimator::new(8);

        // Process in two chunks
        let chunk1: Vec<Complex<f32>> = (0..512).map(|i| Complex::new(i as f32, 0.0)).collect();
        let chunk2: Vec<Complex<f32>> =
            (512..1024).map(|i| Complex::new(i as f32, 0.0)).collect();

        let out1 = dec.process(&chunk1);
        let out2 = dec.process(&chunk2);

        // Should produce some output from both chunks
        assert!(!out1.is_empty());
        assert!(!out2.is_empty());
    }

    #[test]
    fn test_decimator_reset() {
        let mut dec = Decimator::new(4);

        let input: Vec<Complex<f32>> = (0..512).map(|i| Complex::new(i as f32, 0.0)).collect();
        let _ = dec.process(&input);

        // Reset should clear buffer
        dec.reset();
        assert_eq!(dec.buffer.len(), 0);
    }

    #[test]
    fn test_decimator_filter_normalization() {
        let dec = Decimator::new(4);

        // Filter coefficients should sum to approximately 1.0
        let sum: f32 = dec.fir.iter().sum();
        assert_relative_eq!(sum, 1.0, epsilon = 1e-6);
    }
}
