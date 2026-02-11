//! FM demodulation blocks.
//!
//! This module provides building blocks for FM (Frequency Modulation) demodulation:
//! - [`PhaseExtractor`]: Extracts instantaneous phase from complex samples for FM demodulation
//! - [`DeemphasisFilter`]: Applies de-emphasis filtering to restore audio signal balance
//!
//! # Example
//!
//! ```
//! use fmradio::fm::{PhaseExtractor, DeemphasisFilter};
//! use num_complex::Complex;
//!
//! let mut phase_extractor = PhaseExtractor::new();
//! let mut deemphasis = DeemphasisFilter::new(240_000.0, 50e-6);
//!
//! // Demodulate FM signal
//! let iq_samples = vec![Complex::new(0.5, 0.5); 100];
//! let phase = phase_extractor.process(&iq_samples);
//! let audio = deemphasis.process(&phase);
//! ```

use num_complex::Complex;

/// Phase extractor for FM demodulation.
///
/// Extracts the instantaneous phase from a stream of complex samples by computing
/// the phase difference between consecutive samples. This is the core operation
/// for FM demodulation, where information is encoded in frequency (phase rate of change).
///
/// The output is normalized to [-1.0, 1.0] range to prevent clipping in downstream
/// audio processing.
///
/// # Example
///
/// ```
/// use fmradio::fm::PhaseExtractor;
/// use num_complex::Complex;
///
/// let mut extractor = PhaseExtractor::new();
/// let samples = vec![
///     Complex::new(1.0, 0.0),
///     Complex::new(0.707, 0.707),
///     Complex::new(0.0, 1.0),
/// ];
/// let phase = extractor.process(&samples);
/// assert_eq!(phase.len(), 3);
/// ```
pub struct PhaseExtractor {
    /// Last complex sample for phase difference calculation
    last: Complex<f32>,
}

impl PhaseExtractor {
    /// Create a new phase extractor.
    ///
    /// Initializes with a unit complex number (1+0j) as the starting reference.
    pub fn new() -> Self {
        Self {
            last: Complex::new(1.0, 0.0),
        }
    }

    /// Process a block of complex samples and extract phase information.
    ///
    /// For each sample, computes the phase difference from the previous sample
    /// using complex conjugate multiplication and angle extraction. The output
    /// is normalized to the range [-1.0, 1.0] by dividing by the maximum absolute value.
    ///
    /// # Arguments
    ///
    /// * `samples` - Input complex samples to demodulate
    ///
    /// # Returns
    ///
    /// A vector of phase values (real numbers) representing the FM demodulated signal.
    pub fn process(&mut self, samples: &[Complex<f32>]) -> Vec<f32> {
        let mut phases = Vec::with_capacity(samples.len());
        for &sample in samples {
            // Compute phase difference: arg(sample * conj(last))
            let d = (sample * self.last.conj()).arg();
            phases.push(d);
            self.last = sample;
        }

        // Return raw phase values (no normalization)
        // FM deviation is typically ±75 kHz for broadcast FM
        // At 240 kHz sample rate, phase change per sample = 2π × 75k/240k ≈ 1.96 radians max
        // The RDS subcarrier needs the actual phase values, not normalized
        phases
    }

    /// Reset the internal state.
    ///
    /// Resets the last sample to the initial state (1+0j).
    pub fn reset(&mut self) {
        self.last = Complex::new(1.0, 0.0);
    }
}

impl Default for PhaseExtractor {
    fn default() -> Self {
        Self::new()
    }
}

/// De-emphasis filter for FM broadcast audio.
///
/// FM broadcast stations apply pre-emphasis to the audio signal before transmission
/// to improve signal-to-noise ratio (boost high frequencies). This filter applies
/// the inverse operation (de-emphasis) to restore the original audio balance.
///
/// Common time constants:
/// - North America, South Korea: 75 μs (75e-6)
/// - Europe, rest of world: 50 μs (50e-6)
///
/// Implements a first-order IIR low-pass filter:
/// `y[n] = b*x[n] + a*y[n-1]`
///
/// # Example
///
/// ```
/// use fmradio::fm::DeemphasisFilter;
///
/// // European FM broadcast (50 μs time constant)
/// let mut filter = DeemphasisFilter::new(240_000.0, 50e-6);
///
/// let input = vec![0.1, 0.2, 0.3, 0.4, 0.5];
/// let output = filter.process(&input);
/// assert_eq!(output.len(), 5);
/// ```
pub struct DeemphasisFilter {
    /// Coefficient for previous output (feedback)
    a: f32,
    /// Coefficient for current input (feedforward)
    b: f32,
    /// Previous output sample
    prev_y: f32,
}

impl DeemphasisFilter {
    /// Create a new de-emphasis filter.
    ///
    /// # Arguments
    ///
    /// * `sample_rate` - Sample rate in Hz (e.g., 240_000.0 for FM bandwidth)
    /// * `tau` - Time constant in seconds (e.g., 50e-6 for European standard, 75e-6 for North American)
    ///
    /// # Example
    ///
    /// ```
    /// use fmradio::fm::DeemphasisFilter;
    ///
    /// // North American FM broadcast
    /// let filter_na = DeemphasisFilter::new(240_000.0, 75e-6);
    ///
    /// // European FM broadcast
    /// let filter_eu = DeemphasisFilter::new(240_000.0, 50e-6);
    /// ```
    pub fn new(sample_rate: f32, tau: f32) -> Self {
        let dt = 1.0 / sample_rate;
        let decay = (-dt / tau).exp();
        let b = 1.0 - decay;
        let a = decay;
        Self { a, b, prev_y: 0.0 }
    }

    /// Process a block of audio samples through the de-emphasis filter.
    ///
    /// Applies the IIR filter equation: `y[n] = b*x[n] + a*y[n-1]`
    ///
    /// # Arguments
    ///
    /// * `samples` - Input audio samples to filter
    ///
    /// # Returns
    ///
    /// A vector of filtered audio samples with de-emphasis applied.
    pub fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        let mut y = Vec::with_capacity(samples.len());
        for &x in samples {
            let out = self.b * x + self.a * self.prev_y;
            y.push(out);
            self.prev_y = out;
        }
        y
    }

    /// Reset the internal state.
    ///
    /// Clears the previous output sample, useful when starting a new signal stream.
    pub fn reset(&mut self) {
        self.prev_y = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_phase_extractor_new() {
        let extractor = PhaseExtractor::new();
        assert_eq!(extractor.last, Complex::new(1.0, 0.0));
    }

    #[test]
    fn test_phase_extractor_constant_signal() {
        let mut extractor = PhaseExtractor::new();
        let samples = vec![Complex::new(1.0, 0.0); 10];
        let phase = extractor.process(&samples);

        // Constant phase should produce near-zero phase differences (after normalization)
        assert_eq!(phase.len(), 10);
        for &p in &phase {
            assert!(
                p.abs() < 0.1,
                "Phase difference should be near zero for constant signal"
            );
        }
    }

    #[test]
    fn test_phase_extractor_rotating_signal() {
        let mut extractor = PhaseExtractor::new();

        // Create a signal rotating at constant frequency
        let n = 100;
        let mut samples = Vec::with_capacity(n);
        let freq = 0.1; // Normalized frequency
        for i in 0..n {
            let phase = 2.0 * std::f32::consts::PI * freq * i as f32;
            samples.push(Complex::new(phase.cos(), phase.sin()));
        }

        let phase_diff = extractor.process(&samples);
        assert_eq!(phase_diff.len(), n);

        // All phase differences should be similar for constant rotation
        // (after ignoring the first few samples for settling)
        if n > 10 {
            let mean_phase: f32 = phase_diff[5..].iter().sum::<f32>() / (n - 5) as f32;
            for &p in &phase_diff[10..] {
                assert_relative_eq!(p, mean_phase, epsilon = 0.2);
            }
        }
    }

    #[test]
    fn test_phase_extractor_reset() {
        let mut extractor = PhaseExtractor::new();
        let samples = vec![Complex::new(0.5, 0.5); 5];
        let _ = extractor.process(&samples);

        extractor.reset();
        assert_eq!(extractor.last, Complex::new(1.0, 0.0));
    }

    #[test]
    fn test_deemphasis_new() {
        let filter = DeemphasisFilter::new(240_000.0, 50e-6);

        // Coefficients should sum to approximately 1 for DC gain of 1
        assert_relative_eq!(filter.a + filter.b, 1.0, epsilon = 1e-6);
        assert_eq!(filter.prev_y, 0.0);
    }

    #[test]
    fn test_deemphasis_dc_signal() {
        let mut filter = DeemphasisFilter::new(240_000.0, 50e-6);
        let dc_value = 0.5;
        let samples = vec![dc_value; 100];
        let output = filter.process(&samples);

        // DC signal should eventually settle to input value
        assert_eq!(output.len(), 100);
        let final_value = output[output.len() - 1];
        assert_relative_eq!(final_value, dc_value, epsilon = 0.01);
    }

    #[test]
    fn test_deemphasis_impulse_response() {
        let mut filter = DeemphasisFilter::new(240_000.0, 50e-6);

        // Impulse: [1.0, 0.0, 0.0, ...]
        let mut samples = vec![0.0; 10];
        samples[0] = 1.0;

        let output = filter.process(&samples);

        // Impulse response should decay exponentially
        assert!(output[0] > 0.0, "First sample should be positive");
        assert!(output[1] < output[0], "Should decay");
        assert!(output[2] < output[1], "Should continue to decay");
    }

    #[test]
    fn test_deemphasis_reset() {
        let mut filter = DeemphasisFilter::new(240_000.0, 50e-6);
        let samples = vec![1.0; 5];
        let _ = filter.process(&samples);

        filter.reset();
        assert_eq!(filter.prev_y, 0.0);
    }

    #[test]
    fn test_deemphasis_time_constants() {
        // Test both common time constants
        let filter_eu = DeemphasisFilter::new(240_000.0, 50e-6);
        let filter_na = DeemphasisFilter::new(240_000.0, 75e-6);

        // European (50μs) should have faster decay (smaller 'a')
        assert!(
            filter_eu.a < filter_na.a,
            "50μs should decay faster than 75μs"
        );
    }

    #[test]
    fn test_phase_extractor_empty_input() {
        let mut extractor = PhaseExtractor::new();
        let samples: Vec<Complex<f32>> = vec![];
        let phase = extractor.process(&samples);
        assert_eq!(phase.len(), 0);
    }

    #[test]
    fn test_deemphasis_empty_input() {
        let mut filter = DeemphasisFilter::new(240_000.0, 50e-6);
        let samples: Vec<f32> = vec![];
        let output = filter.process(&samples);
        assert_eq!(output.len(), 0);
    }
}
