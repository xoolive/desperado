//! Digital filter implementations.
//!
//! This module provides various digital filter implementations for signal processing:
//! - [`LowPassFir`]: Finite Impulse Response (FIR) low-pass filter
//! - [`ButterworthFilter`]: Butterworth IIR filter (low-pass and bandpass)
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

use super::window::{WindowType, design_fir_filter};
use num_complex::Complex;

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

        let norm_cutoff = cutoff_freq / sample_rate;
        let fir = design_fir_filter(norm_cutoff, taps, WindowType::Blackman);

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

/// Stateful FIR low-pass filter that maintains inter-block continuity.
///
/// Unlike [`LowPassFir`] which zero-pads at block boundaries, this filter keeps a ring
/// buffer of past samples so that consecutive `process()` calls produce the same output
/// as filtering the concatenation of all input blocks in one pass. This is essential for
/// any application where a PLL or other feedback loop depends on the filtered output,
/// since zero-padding artifacts at block edges would disturb the loop.
///
/// # Example
///
/// ```
/// use desperado::dsp::filters::StatefulLowPassFir;
///
/// let mut filter = StatefulLowPassFir::new(15_000.0, 240_000.0, 257);
/// let block1 = vec![1.0_f32; 1024];
/// let block2 = vec![1.0_f32; 1024];
/// let out1 = filter.process(&block1);
/// let out2 = filter.process(&block2); // no edge artifact between blocks
/// ```
pub struct StatefulLowPassFir {
    fir: Vec<f32>,
    /// Ring buffer holding past input samples (length = number of taps).
    buffer: Vec<f32>,
    /// Write position in ring buffer.
    write_pos: usize,
}

impl StatefulLowPassFir {
    /// Create a new stateful low-pass FIR filter.
    ///
    /// # Arguments
    ///
    /// * `cutoff_freq` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `taps` - Number of filter taps (must be > 0)
    pub fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        assert!(taps > 0, "Number of taps must be greater than 0");
        assert!(sample_rate > 0.0, "Sample rate must be greater than 0");

        let norm_cutoff = cutoff_freq / sample_rate;
        let fir = super::window::design_fir_filter(
            norm_cutoff,
            taps,
            super::window::WindowType::Blackman,
        );

        Self {
            fir,
            buffer: vec![0.0; taps],
            write_pos: 0,
        }
    }

    /// Process a block of samples through the filter (maintains state between calls).
    pub fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        let mut out = Vec::with_capacity(samples.len());
        for &x in samples {
            self.buffer[self.write_pos] = x;
            self.write_pos = (self.write_pos + 1) % self.buffer.len();

            let mut acc = 0.0_f32;
            let len = self.fir.len();
            for i in 0..len {
                let buf_idx = (self.write_pos + len - 1 - i) % len;
                acc += self.buffer[buf_idx] * self.fir[i];
            }
            out.push(acc);
        }
        out
    }

    /// Get the number of filter taps.
    pub fn taps(&self) -> usize {
        self.fir.len()
    }

    /// Get the filter coefficients.
    pub fn coefficients(&self) -> &[f32] {
        &self.fir
    }
}

/// Butterworth filter for low-pass and bandpass filtering.
///
/// Implements a Butterworth filter using FIR coefficients derived from a low-pass
/// prototype. Supports both real and complex signal filtering.
///
/// The filter operates as a simple FIR filter with a circular buffer state,
/// suitable for real-time processing of scalar and complex samples.
///
/// # Example
///
/// ```
/// use desperado::dsp::filters::ButterworthFilter;
/// use num_complex::Complex;
///
/// // Low-pass filter at 10 kHz with 4th order cutoff
/// let mut filter = ButterworthFilter::lowpass(10_000.0, 48_000.0, 4);
/// let input = vec![0.1, 0.2, 0.3, 0.4];
/// let output = filter.filter(&input);
/// assert_eq!(output.len(), 4);
///
/// // Bandpass filter between 900 Hz and 1100 Hz
/// let mut bp_filter = ButterworthFilter::bandpass(900.0, 1100.0, 48_000.0, 4);
/// ```
#[derive(Debug, Clone)]
pub struct ButterworthFilter {
    coeffs: Vec<f64>,
    real_state: Vec<f64>,
    imag_state: Vec<f64>,
    real_pos: usize,
    imag_pos: usize,
}

impl ButterworthFilter {
    /// Create a low-pass Butterworth filter.
    ///
    /// # Arguments
    ///
    /// * `cutoff` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `order` - Filter order (e.g., 4 for 4th order)
    pub fn lowpass(cutoff: f64, sample_rate: f64, order: usize) -> Self {
        let taps = taps_from_order(order);
        let fir = LowPassFir::new(cutoff as f32, sample_rate as f32, taps);
        let coeffs = fir.coefficients().iter().map(|&c| c as f64).collect();
        Self::from_coeffs(coeffs)
    }

    /// Create a bandpass Butterworth filter.
    ///
    /// # Arguments
    ///
    /// * `low` - Lower cutoff frequency in Hz
    /// * `high` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `order` - Filter order (e.g., 4 for 4th order)
    pub fn bandpass(low: f64, high: f64, sample_rate: f64, order: usize) -> Self {
        let taps = taps_from_order(order);
        let lp_high = LowPassFir::new(high as f32, sample_rate as f32, taps);
        let lp_low = LowPassFir::new(low as f32, sample_rate as f32, taps);

        let coeffs = lp_high
            .coefficients()
            .iter()
            .zip(lp_low.coefficients().iter())
            .map(|(h, l)| (*h - *l) as f64)
            .collect();

        Self::from_coeffs(coeffs)
    }

    fn from_coeffs(coeffs: Vec<f64>) -> Self {
        let len = coeffs.len();
        Self {
            coeffs,
            real_state: vec![0.0; len],
            imag_state: vec![0.0; len],
            real_pos: 0,
            imag_pos: 0,
        }
    }

    /// Filter a block of real-valued samples.
    ///
    /// # Arguments
    ///
    /// * `input` - Input samples to filter
    ///
    /// # Returns
    ///
    /// Filtered output samples with the same length as input
    pub fn filter(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(fir_step(
                x,
                &self.coeffs,
                &mut self.real_state,
                &mut self.real_pos,
            ));
        }
        output
    }

    /// Filter a block of complex samples.
    ///
    /// # Arguments
    ///
    /// * `input` - Complex input samples to filter
    ///
    /// # Returns
    ///
    /// Filtered complex output samples with the same length as input
    pub fn filter_complex(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let r = fir_step(
                x.re as f64,
                &self.coeffs,
                &mut self.real_state,
                &mut self.real_pos,
            );
            let i = fir_step(
                x.im as f64,
                &self.coeffs,
                &mut self.imag_state,
                &mut self.imag_pos,
            );
            output.push(Complex::new(r as f32, i as f32));
        }
        output
    }
}

fn taps_from_order(order: usize) -> usize {
    let base = (order.max(1) * 16) + 1;
    if base % 2 == 1 { base } else { base + 1 }
}

fn fir_step(x: f64, coeffs: &[f64], state: &mut [f64], pos: &mut usize) -> f64 {
    state[*pos] = x;

    let mut y = 0.0;
    let mut idx = *pos;
    for &c in coeffs {
        y += c * state[idx];
        idx = if idx == 0 { state.len() - 1 } else { idx - 1 };
    }

    *pos += 1;
    if *pos == state.len() {
        *pos = 0;
    }

    y
}
