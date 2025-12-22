//! Digital filter implementations.
//!
//! This module provides various digital filter implementations for signal processing:
//! - [`LowPassFir`]: Finite Impulse Response (FIR) low-pass filter
//!
//! # Example
//!
//! ```
//! use desperado::dsp::filters::LowPassFir;
//!
//! // Create a low-pass filter with 15kHz cutoff at 240kHz sample rate
//! let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
//!
//! // Filter audio samples
//! let input = vec![0.5; 100];
//! let output = filter.process(&input);
//! assert_eq!(output.len(), 100);
//! ```

use std::f32::consts::PI;

/// Finite Impulse Response (FIR) low-pass filter.
///
/// Implements a low-pass FIR filter using a windowed-sinc design with a Blackman window.
/// This filter provides good stopband attenuation and minimal passband ripple, making it
/// suitable for audio and RF signal processing applications.
///
/// The filter uses a centered FIR implementation where the output is computed by convolving
/// the input signal with the filter coefficients. Edge handling is performed by zero-padding
/// (samples outside the input range are treated as zero).
///
/// # Design Method
///
/// The filter is designed using the windowed-sinc method:
/// 1. Ideal sinc function for the desired cutoff frequency
/// 2. Blackman window for sidelobe suppression
/// 3. Normalization to maintain unity gain in passband
///
/// # Example
///
/// ```
/// use desperado::dsp::filters::LowPassFir;
///
/// // Create a filter to isolate mono audio (0-15kHz) from FM baseband
/// let cutoff = 15_000.0;      // 15 kHz cutoff
/// let sample_rate = 240_000.0; // FM baseband sample rate
/// let taps = 256;              // Filter length (more taps = sharper transition)
///
/// let filter = LowPassFir::new(cutoff, sample_rate, taps);
///
/// // Process audio samples
/// let audio = vec![0.1, 0.2, 0.3, 0.4, 0.5];
/// let filtered = filter.process(&audio);
/// ```
pub struct LowPassFir {
    /// Filter coefficients (impulse response)
    fir: Vec<f32>,
}

impl LowPassFir {
    /// Create a new low-pass FIR filter.
    ///
    /// Designs a windowed-sinc FIR filter with Blackman window. The filter will have
    /// linear phase (symmetric impulse response) and approximately unity gain in the passband.
    ///
    /// # Arguments
    ///
    /// * `cutoff_freq` - Cutoff frequency in Hz (e.g., 15_000.0 for 15 kHz)
    /// * `sample_rate` - Sample rate in Hz (e.g., 240_000.0 for FM baseband)
    /// * `taps` - Number of filter taps (must be > 0). More taps provide a sharper
    ///   transition band but increase computation. Typical values: 64-512.
    ///
    /// # Panics
    ///
    /// Panics if `taps` is 0 or if `sample_rate` is 0.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::filters::LowPassFir;
    ///
    /// // Sharper filter (more taps)
    /// let sharp = LowPassFir::new(15_000.0, 240_000.0, 512);
    ///
    /// // Faster filter (fewer taps)
    /// let fast = LowPassFir::new(15_000.0, 240_000.0, 64);
    /// ```
    pub fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        assert!(taps > 0, "Number of taps must be greater than 0");
        assert!(sample_rate > 0.0, "Sample rate must be greater than 0");

        let mut fir = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;
        let norm_cutoff = cutoff_freq / (sample_rate / 2.0);

        // Design windowed-sinc filter
        for n in 0..taps {
            let x = n as isize - mid;

            // Sinc function: sinc(x) = sin(πx) / (πx), with sinc(0) = 1
            let sinc = if x == 0 {
                2.0 * norm_cutoff
            } else {
                (2.0 * norm_cutoff * PI * x as f32).sin() / (PI * x as f32)
            };

            // Blackman window: w(n) = 0.42 - 0.5*cos(2πn/N) + 0.08*cos(4πn/N)
            let window = 0.42 - 0.5 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos()
                + 0.08 * ((4.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();

            fir.push(sinc * window);
        }

        // Normalize to unity gain
        let norm: f32 = fir.iter().sum();
        for v in fir.iter_mut() {
            *v /= norm;
        }

        Self { fir }
    }

    /// Process a block of samples through the filter.
    ///
    /// Applies the FIR filter by convolving the input with the filter coefficients.
    /// Samples outside the input range are treated as zero (zero-padding).
    ///
    /// # Arguments
    ///
    /// * `samples` - Input samples to filter
    ///
    /// # Returns
    ///
    /// A vector of filtered samples with the same length as the input.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::filters::LowPassFir;
    ///
    /// let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
    /// let input = vec![1.0, 0.5, 0.0, -0.5, -1.0];
    /// let output = filter.process(&input);
    /// assert_eq!(output.len(), 5);
    /// ```
    pub fn process(&self, samples: &[f32]) -> Vec<f32> {
        let taps = self.fir.len();
        let mid = taps / 2;
        let mut out = vec![0.0f32; samples.len()];

        for (i, out_elem) in out.iter_mut().enumerate() {
            let mut acc = 0.0f32;
            for j in 0..taps {
                let idx = i as isize + j as isize - mid as isize;
                if idx >= 0 && (idx as usize) < samples.len() {
                    acc += samples[idx as usize] * self.fir[j];
                }
            }
            *out_elem = acc;
        }
        out
    }

    /// Get the number of filter taps.
    ///
    /// # Returns
    ///
    /// The number of coefficients in the filter.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::filters::LowPassFir;
    ///
    /// let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
    /// assert_eq!(filter.taps(), 256);
    /// ```
    pub fn taps(&self) -> usize {
        self.fir.len()
    }

    /// Get the filter coefficients.
    ///
    /// # Returns
    ///
    /// A slice containing the filter's impulse response coefficients.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::filters::LowPassFir;
    ///
    /// let filter = LowPassFir::new(15_000.0, 240_000.0, 64);
    /// let coeffs = filter.coefficients();
    /// assert_eq!(coeffs.len(), 64);
    /// ```
    pub fn coefficients(&self) -> &[f32] {
        &self.fir
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_lowpass_fir_creation() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
        assert_eq!(filter.taps(), 256);
        assert_eq!(filter.coefficients().len(), 256);
    }

    #[test]
    #[should_panic(expected = "Number of taps must be greater than 0")]
    fn test_lowpass_fir_zero_taps() {
        let _ = LowPassFir::new(15_000.0, 240_000.0, 0);
    }

    #[test]
    #[should_panic(expected = "Sample rate must be greater than 0")]
    fn test_lowpass_fir_zero_sample_rate() {
        let _ = LowPassFir::new(15_000.0, 0.0, 256);
    }

    #[test]
    fn test_lowpass_fir_coefficients_normalized() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
        let sum: f32 = filter.coefficients().iter().sum();

        // Coefficients should sum to 1 for unity gain
        assert_relative_eq!(sum, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_lowpass_fir_dc_gain() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);

        // DC signal (constant value) should pass through with unity gain
        let dc_value = 1.0;
        let input = vec![dc_value; 1000];
        let output = filter.process(&input);

        // Check samples away from edges (to avoid edge effects)
        for &sample in output.iter().skip(200).take(600) {
            assert_relative_eq!(sample, dc_value, epsilon = 0.01);
        }
    }

    #[test]
    fn test_lowpass_fir_impulse_response() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);

        // Create impulse: [0, 0, ..., 1, 0, 0, ...]
        let mut input = vec![0.0; 500];
        input[250] = 1.0;

        let output = filter.process(&input);

        // Output should be centered around the impulse position
        // and should be the filter's impulse response
        let max_val = output.iter().fold(0.0f32, |a, &b| a.max(b));
        assert!(
            max_val > 0.0,
            "Impulse response should have non-zero output"
        );

        // The peak should be around the impulse position (±taps/2)
        let peak_idx = output
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(idx, _)| idx)
            .unwrap();

        assert!(
            (peak_idx as isize - 250).abs() < 10,
            "Peak should be near impulse position"
        );
    }

    #[test]
    fn test_lowpass_fir_zero_input() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
        let input = vec![0.0; 100];
        let output = filter.process(&input);

        assert_eq!(output.len(), 100);
        for &sample in &output {
            assert_eq!(sample, 0.0);
        }
    }

    #[test]
    fn test_lowpass_fir_empty_input() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);
        let input: Vec<f32> = vec![];
        let output = filter.process(&input);

        assert_eq!(output.len(), 0);
    }

    #[test]
    fn test_lowpass_fir_output_length() {
        let filter = LowPassFir::new(15_000.0, 240_000.0, 256);

        // Output should have same length as input
        for len in [1, 10, 100, 1000] {
            let input = vec![0.5; len];
            let output = filter.process(&input);
            assert_eq!(output.len(), len);
        }
    }

    #[test]
    fn test_lowpass_fir_different_tap_counts() {
        // Test that different tap counts produce valid filters
        for taps in [16, 32, 64, 128, 256, 512] {
            let filter = LowPassFir::new(15_000.0, 240_000.0, taps);
            assert_eq!(filter.taps(), taps);

            let sum: f32 = filter.coefficients().iter().sum();
            assert_relative_eq!(sum, 1.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_lowpass_fir_cutoff_frequency_effect() {
        // Lower cutoff should have narrower passband
        let low_cutoff = LowPassFir::new(5_000.0, 240_000.0, 256);
        let high_cutoff = LowPassFir::new(50_000.0, 240_000.0, 256);

        // The maximum coefficient should be higher for lower cutoff
        // (narrower filter = more attenuation needed)
        let low_max = low_cutoff
            .coefficients()
            .iter()
            .fold(0.0f32, |a, &b| a.max(b.abs()));
        let high_max = high_cutoff
            .coefficients()
            .iter()
            .fold(0.0f32, |a, &b| a.max(b.abs()));

        assert!(
            low_max < high_max,
            "Lower cutoff should have smaller peak coefficient"
        );
    }
}
