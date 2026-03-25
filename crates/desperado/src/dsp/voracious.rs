//! Shared DSP utilities for radio navigation decoders (voracious).
//!
//! This module provides common signal processing functions used across
//! VOR, ILS, and other radio beacon decoders to avoid code duplication.

use num_complex::Complex;
use rustfft::FftPlanner;

/// Compute the complex analytic signal (Hilbert transform) of a real signal.
///
/// Uses FFT to compute the one-sided Hilbert transform: the positive
/// frequencies are kept (scaled by 2), and the negative frequencies are zeroed.
/// This gives the complex envelope of the original signal.
///
/// # Arguments
/// * `signal` - Real-valued input signal
///
/// # Returns
/// Complex analytic signal where the magnitude represents the envelope
/// and the phase represents the instantaneous phase.
///
/// # Example
/// ```ignore
/// let real_signal = vec![1.0, 2.0, 3.0, 2.0, 1.0];
/// let analytic = hilbert_transform(&real_signal);
/// let envelope: Vec<f64> = analytic.iter().map(|c| c.norm()).collect();
/// ```
pub fn hilbert_transform(signal: &[f64]) -> Vec<Complex<f64>> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut spectrum: Vec<Complex<f64>> = signal
        .iter()
        .map(|&x| Complex::<f64>::new(x, 0.0))
        .collect();

    fft.process(&mut spectrum);

    // Zero out negative frequencies and scale positive frequencies by 2
    if n.is_multiple_of(2) {
        for x in spectrum.iter_mut().take(n / 2).skip(1) {
            *x *= 2.0;
        }
        for x in spectrum.iter_mut().skip(n / 2 + 1) {
            *x = Complex::new(0.0, 0.0);
        }
    } else {
        for x in spectrum.iter_mut().take(n.div_ceil(2)).skip(1) {
            *x *= 2.0;
        }
        for x in spectrum.iter_mut().skip(n.div_ceil(2)) {
            *x = Complex::new(0.0, 0.0);
        }
    }

    ifft.process(&mut spectrum);

    let scale = 1.0 / n as f64;
    for x in &mut spectrum {
        *x *= scale;
    }

    spectrum
}

/// Compute the envelope (magnitude) of a complex signal.
///
/// For each complex sample, returns its magnitude (absolute value).
/// Typically applied to the output of [`hilbert_transform`] to get
/// the amplitude modulation envelope.
///
/// # Arguments
/// * `signal` - Complex-valued input signal
///
/// # Returns
/// Real-valued envelope signal
///
/// # Example
/// ```ignore
/// let analytic = hilbert_transform(&real_signal);
/// let env = envelope(&analytic);
/// ```
pub fn envelope(signal: &[Complex<f64>]) -> Vec<f64> {
    signal.iter().map(|c| c.norm()).collect()
}

/// Decimate a signal by an integer factor (simple downsampling via averaging).
///
/// Takes every `factor`-th sample and averages groups to reduce sample rate.
/// This is a simple approach suitable for post-processing; for anti-aliasing,
/// filter before decimation.
///
/// # Arguments
/// * `signal` - Input signal
/// * `factor` - Decimation factor (must be > 0)
///
/// # Returns
/// Decimated signal with length ≈ original / factor
pub fn decimate(signal: &[f64], factor: usize) -> Vec<f64> {
    if factor == 0 || signal.is_empty() {
        return Vec::new();
    }
    if factor == 1 {
        return signal.to_vec();
    }

    signal
        .chunks(factor)
        .map(|chunk| {
            let sum: f64 = chunk.iter().sum();
            sum / chunk.len() as f64
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hilbert_transform_empty() {
        let result = hilbert_transform(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_hilbert_transform_single_sample() {
        let result = hilbert_transform(&[1.0]);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_envelope_empty() {
        let result = envelope(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_envelope_constant() {
        let signal = vec![Complex::new(3.0, 4.0), Complex::new(3.0, 4.0)];
        let result = envelope(&signal);
        assert_eq!(result.len(), 2);
        assert!((result[0] - 5.0).abs() < 1e-10); // |3+4j| = 5
        assert!((result[1] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_decimate_empty() {
        let result = decimate(&[], 2);
        assert!(result.is_empty());
    }

    #[test]
    fn test_decimate_factor_1() {
        let input = vec![1.0, 2.0, 3.0];
        let result = decimate(&input, 1);
        assert_eq!(result, input);
    }

    #[test]
    fn test_decimate_factor_2() {
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let result = decimate(&input, 2);
        assert_eq!(result.len(), 2);
        assert!((result[0] - 1.5).abs() < 1e-10); // (1+2)/2
        assert!((result[1] - 3.5).abs() < 1e-10); // (3+4)/2
    }

    #[test]
    fn test_decimate_factor_zero() {
        let input = vec![1.0, 2.0, 3.0];
        let result = decimate(&input, 0);
        assert!(result.is_empty());
    }
}
