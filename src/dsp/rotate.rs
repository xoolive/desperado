/// A DSP block that applies a complex rotation to a sequence of samples.
///
/// The `Rotate` struct performs frequency shifting by applying a complex rotation
/// (phasor multiplication) to each sample. This is commonly used for:
/// - Frequency offset correction
/// - Tuning to a different center frequency
/// - Doppler shift compensation
///
/// The rotation is applied incrementally, with each sample rotated by an additional
/// phase angle. The internal state is periodically normalized to prevent numerical
/// drift from accumulating over long processing runs.
///
/// # Implementation
///
/// Uses optimized complex multiplication by manually expanding the formula:
/// `(a + bi) * (c + di) = (ac - bd) + (ad + bc)i`
///
/// # Example
/// ```
/// use num_complex::Complex;
/// use desperado::dsp::rotate::Rotate;
/// use desperado::dsp::DspBlock;
///
/// // Rotate by 0.1 radians per sample (frequency shift)
/// let mut rotator = Rotate::new(0.1);
/// let input = vec![Complex::new(1.0, 0.0); 100];
/// let output = rotator.process(&input);
/// assert_eq!(output.len(), 100);
/// ```
use num_complex::Complex;

use crate::dsp::DspBlock;

/// Complex rotation DSP block for frequency shifting.
///
/// Maintains internal state consisting of:
/// - Current rotation phasor (`rot`)
/// - Per-sample rotation step (`mult`)
///
/// The rotation is normalized periodically to prevent numerical drift.
pub struct Rotate {
    /// Current complex rotation factor
    rot: Complex<f32>,
    /// Per-sample rotation multiplier
    mult: Complex<f32>,
}

impl Rotate {
    /// Create a new Rotate DSP block with the specified rotation angle.
    ///
    /// The angle represents the rotation applied *per sample*. For frequency
    /// shifting, this is typically computed as:
    ///
    /// `angle = -2π × freq_offset / sample_rate`
    ///
    /// # Arguments
    ///
    /// * `angle` - Rotation angle in radians per sample
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::rotate::Rotate;
    /// use std::f32::consts::PI;
    ///
    /// // Shift by 200 kHz at 2 MHz sample rate
    /// let sample_rate = 2_000_000.0;
    /// let freq_offset = 200_000.0;
    /// let angle = -2.0 * PI * freq_offset / sample_rate;
    /// let rotator = Rotate::new(angle);
    /// ```
    pub fn new(angle: f32) -> Self {
        Self {
            rot: Complex::new(1.0, 0.0),
            mult: Complex::new(angle.cos(), angle.sin()),
        }
    }

    /// Reset the internal rotation state.
    ///
    /// Resets the rotation phasor to the initial state (1 + 0i).
    /// Useful when starting a new signal or when you want to reset phase accumulation.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::rotate::Rotate;
    /// use desperado::dsp::DspBlock;
    /// use num_complex::Complex;
    ///
    /// let mut rotator = Rotate::new(0.1);
    /// let input = vec![Complex::new(1.0, 0.0); 10];
    /// let _ = rotator.process(&input);
    ///
    /// // Reset for new signal
    /// rotator.reset();
    /// ```
    pub fn reset(&mut self) {
        self.rot = Complex::new(1.0, 0.0);
    }
}

impl DspBlock for Rotate {
    /// Process a slice of complex samples, applying the rotation.
    ///
    /// Each sample is rotated by the current phase, and the phase is incremented
    /// by the rotation angle. After processing all samples, the rotation phasor
    /// is normalized to prevent drift.
    ///
    /// # Arguments
    ///
    /// * `data` - Input complex samples
    ///
    /// # Returns
    ///
    /// A vector of rotated complex samples with the same length as the input.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::rotate::Rotate;
    /// use desperado::dsp::DspBlock;
    /// use num_complex::Complex;
    ///
    /// let mut rotator = Rotate::new(0.5);
    /// let input = vec![Complex::new(1.0, 0.0); 5];
    /// let output = rotator.process(&input);
    /// assert_eq!(output.len(), 5);
    /// ```
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut output = Vec::with_capacity(data.len());

        for &sample in data {
            // Optimized complex multiplication: (a + bi) * (c + di) = (ac - bd) + (ad + bc)i
            let rr = sample.re * self.rot.re;
            let ii = sample.im * self.rot.im;
            let ri = sample.re * self.rot.im;
            let ir = sample.im * self.rot.re;

            output.push(Complex::new(rr - ii, ir + ri));

            // Update rotation phasor
            self.rot *= self.mult;
        }

        // Normalize to prevent drift
        let norm = self.rot.norm();
        if norm > 0.0 {
            self.rot /= norm;
        }

        output
    }
}

impl Default for Rotate {
    fn default() -> Self {
        Self::new(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_rotate_new() {
        let rotator = Rotate::new(0.1);
        assert_eq!(rotator.rot, Complex::new(1.0, 0.0));
        assert_relative_eq!(rotator.mult.re, 0.1_f32.cos(), epsilon = 1e-6);
        assert_relative_eq!(rotator.mult.im, 0.1_f32.sin(), epsilon = 1e-6);
    }

    #[test]
    fn test_rotate_zero_angle() {
        let mut rotator = Rotate::new(0.0);
        let input = vec![
            Complex::new(1.0, 0.0),
            Complex::new(0.5, 0.5),
            Complex::new(0.0, 1.0),
        ];
        let output = rotator.process(&input);

        // Zero rotation should not change the signal
        for (inp, out) in input.iter().zip(output.iter()) {
            assert_relative_eq!(inp.re, out.re, epsilon = 1e-6);
            assert_relative_eq!(inp.im, out.im, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_rotate_90_degrees() {
        let mut rotator = Rotate::new(std::f32::consts::FRAC_PI_2);
        let input = vec![Complex::new(1.0, 0.0)];
        let output = rotator.process(&input);

        // First sample rotated by 90 degrees: 1+0i rotated once → remains 1+0i
        // (rotation happens AFTER multiplication for the first sample)
        // Actually: sample * rot, where rot starts at 1+0i
        // So first output = 1+0i * 1+0i = 1+0i
        // But then rot is updated to 0+1i for next sample
        assert_relative_eq!(output[0].re, 1.0, epsilon = 1e-6);
        assert_relative_eq!(output[0].im, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_rotate_180_degrees() {
        let mut rotator = Rotate::new(std::f32::consts::PI);
        let input = vec![Complex::new(1.0, 0.0), Complex::new(1.0, 0.0)];
        let output = rotator.process(&input);

        // First sample: 1+0i * 1+0i = 1+0i
        assert_relative_eq!(output[0].re, 1.0, epsilon = 1e-6);
        assert_relative_eq!(output[0].im, 0.0, epsilon = 1e-6);

        // Second sample: 1+0i * (-1+0i) = -1+0i
        assert_relative_eq!(output[1].re, -1.0, epsilon = 1e-6);
        assert_relative_eq!(output[1].im, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_rotate_accumulation() {
        let angle = 0.1;
        let mut rotator = Rotate::new(angle);
        let input = vec![Complex::new(1.0, 0.0); 10];
        let output = rotator.process(&input);

        // Each sample should be rotated by the accumulated rotation
        // Sample 0: rot = 1+0i (initial), output = input * rot
        // Sample 1: rot = cos(angle) + i*sin(angle), output = input * rot
        // Sample i: rot has accumulated i*angle rotation
        for (i, &sample) in output.iter().enumerate() {
            let expected_angle = angle * i as f32;
            let expected = Complex::new(expected_angle.cos(), expected_angle.sin());
            assert_relative_eq!(sample.re, expected.re, epsilon = 1e-4);
            assert_relative_eq!(sample.im, expected.im, epsilon = 1e-4);
        }
    }

    #[test]
    fn test_rotate_normalization() {
        // Process many samples to verify normalization prevents drift
        let mut rotator = Rotate::new(0.01);
        let input = vec![Complex::new(1.0, 0.0); 10000];
        let _ = rotator.process(&input);

        // After many rotations, the internal state should still be normalized
        let norm = rotator.rot.norm();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-4);
    }

    #[test]
    fn test_rotate_reset() {
        let mut rotator = Rotate::new(0.5);
        let input = vec![Complex::new(1.0, 0.0); 10];
        let _ = rotator.process(&input);

        rotator.reset();
        assert_eq!(rotator.rot, Complex::new(1.0, 0.0));
    }

    #[test]
    fn test_rotate_empty_input() {
        let mut rotator = Rotate::new(0.1);
        let input: Vec<Complex<f32>> = vec![];
        let output = rotator.process(&input);
        assert_eq!(output.len(), 0);
    }

    #[test]
    fn test_rotate_output_length() {
        let mut rotator = Rotate::new(0.1);
        for len in [1, 10, 100, 1000] {
            let input = vec![Complex::new(1.0, 0.0); len];
            let output = rotator.process(&input);
            assert_eq!(output.len(), len);
        }
    }

    #[test]
    fn test_rotate_frequency_shift() {
        // Simulate frequency shift scenario
        let sample_rate = 1_000_000.0;
        let freq_offset = 100_000.0; // 100 kHz offset
        let angle = -2.0 * std::f32::consts::PI * freq_offset / sample_rate;

        let mut rotator = Rotate::new(angle);

        // Generate a tone at the offset frequency
        let n_samples = 100;
        let mut input = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let phase = 2.0 * std::f32::consts::PI * freq_offset * i as f32 / sample_rate;
            input.push(Complex::new(phase.cos(), phase.sin()));
        }

        let output = rotator.process(&input);

        // After rotation, the tone should be at DC (near constant)
        // Check that output is relatively constant
        let mean_re = output.iter().map(|c| c.re).sum::<f32>() / output.len() as f32;
        let mean_im = output.iter().map(|c| c.im).sum::<f32>() / output.len() as f32;

        for sample in &output[10..] {
            // Skip first few samples for settling
            assert_relative_eq!(sample.re, mean_re, epsilon = 0.1);
            assert_relative_eq!(sample.im, mean_im, epsilon = 0.1);
        }
    }
}
