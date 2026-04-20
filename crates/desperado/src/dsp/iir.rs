//! IIR filters with zero-phase filtering (filtfilt).
//!
//! This module provides zero-phase filtering implementations using the `biquad` crate
//! for Butterworth IIR filter design. Zero-phase filtering applies the filter both
//! forward and backward to eliminate phase distortion.
//!
//! # Example
//!
//! ```
//! use desperado::dsp::iir::filtfilt_lowpass;
//!
//! // Apply zero-phase low-pass filter at 100 Hz cutoff, 1 kHz sample rate, 4th order
//! let signal = vec![0.5; 100];
//! let filtered = filtfilt_lowpass(&signal, 100.0, 1000.0, 4);
//! assert_eq!(filtered.len(), 100);
//! ```
//!
//! # WASM Compatibility
//!
//! The biquad crate used here is compatible with WebAssembly platforms.
//! All operations use standard Rust arithmetic without platform-specific intrinsics.
//! This module should work correctly in WASM environments.

use biquad::*;

/// Apply zero-phase (forward + backward) filtering with the given biquad coefficients.
///
/// The filter is applied `passes` times forward, then the signal is reversed
/// and filtered `passes` times backward, then reversed again. This eliminates
/// phase distortion and doubles the effective filter order.
fn filtfilt_core(x: &[f64], coeffs: Coefficients<f64>, passes: usize) -> Vec<f64> {
    // Forward pass
    let mut y = x.to_vec();
    for _ in 0..passes {
        let mut df1 = DirectForm1::<f64>::new(coeffs);
        for sample in y.iter_mut() {
            *sample = df1.run(*sample);
        }
    }

    // Reverse
    y.reverse();

    // Backward pass
    for _ in 0..passes {
        let mut df1 = DirectForm1::<f64>::new(coeffs);
        for sample in y.iter_mut() {
            *sample = df1.run(*sample);
        }
    }

    // Reverse back
    y.reverse();
    y
}

/// Apply zero-phase filtering (forward + backward pass) using Butterworth low-pass IIR filter.
///
/// Zero-phase filtering applies the filter twice (forward and backward) to eliminate
/// phase distortion. This doubles the effective filter order.
///
/// # Arguments
///
/// * `x` - Input signal samples
/// * `cutoff` - Cutoff frequency in Hz
/// * `fs` - Sample rate in Hz
/// * `order` - Filter order per pass (effective order will be 2x due to forward+backward)
///
/// # Returns
///
/// Filtered signal with the same length as input
///
/// # Example
///
/// ```
/// use desperado::dsp::iir::filtfilt_lowpass;
///
/// let signal = vec![1.0, 0.8, 0.6, 0.4, 0.2];
/// let filtered = filtfilt_lowpass(&signal, 10.0, 100.0, 2);
/// assert_eq!(filtered.len(), signal.len());
/// ```
pub fn filtfilt_lowpass(x: &[f64], cutoff: f64, fs: f64, order: usize) -> Vec<f64> {
    let hz = cutoff.hz();
    let sample_rate = fs.hz();

    let coeffs =
        Coefficients::<f64>::from_params(Type::LowPass, sample_rate, hz, Q_BUTTERWORTH_F64)
            .unwrap();

    filtfilt_core(x, coeffs, order.div_ceil(2))
}

/// Apply zero-phase filtering for bandpass IIR filter.
///
/// Zero-phase filtering applies the filter twice (forward and backward) to eliminate
/// phase distortion. This doubles the effective filter order.
///
/// # Arguments
///
/// * `x` - Input signal samples
/// * `lowcut` - Lower cutoff frequency in Hz
/// * `highcut` - Upper cutoff frequency in Hz
/// * `fs` - Sample rate in Hz
/// * `order` - Filter order per pass (effective order will be 2x due to forward+backward)
///
/// # Returns
///
/// Filtered signal with the same length as input
///
/// # Example
///
/// ```
/// use desperado::dsp::iir::filtfilt_bandpass;
///
/// let signal = vec![0.1, 0.2, 0.3, 0.4, 0.5];
/// let filtered = filtfilt_bandpass(&signal, 100.0, 200.0, 1000.0, 2);
/// assert_eq!(filtered.len(), signal.len());
/// ```
pub fn filtfilt_bandpass(x: &[f64], lowcut: f64, highcut: f64, fs: f64, order: usize) -> Vec<f64> {
    let center = (lowcut + highcut) / 2.0;
    let bandwidth = highcut - lowcut;

    let hz = center.hz();
    let sample_rate = fs.hz();
    let q = center / bandwidth;

    let coeffs = Coefficients::<f64>::from_params(Type::BandPass, sample_rate, hz, q).unwrap();

    filtfilt_core(x, coeffs, order.div_ceil(2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filtfilt_lowpass_length() {
        let signal = vec![1.0; 100];
        let filtered = filtfilt_lowpass(&signal, 100.0, 1000.0, 2);
        assert_eq!(filtered.len(), 100);
    }

    #[test]
    fn test_filtfilt_lowpass_dc_passthrough() {
        // DC signal should pass through a low-pass filter unchanged
        let signal = vec![1.0; 1000];
        let filtered = filtfilt_lowpass(&signal, 100.0, 1000.0, 4);
        // Check interior samples (away from edges)
        for &v in &filtered[100..900] {
            assert!((v - 1.0).abs() < 0.05, "DC passthrough failed: {}", v);
        }
    }

    #[test]
    fn test_filtfilt_bandpass_length() {
        let signal = vec![0.5; 100];
        let filtered = filtfilt_bandpass(&signal, 100.0, 200.0, 1000.0, 2);
        assert_eq!(filtered.len(), 100);
    }

    #[test]
    fn test_filtfilt_bandpass_rejects_dc() {
        // DC signal should be rejected by a bandpass filter
        let signal = vec![1.0; 1000];
        let filtered = filtfilt_bandpass(&signal, 100.0, 200.0, 1000.0, 4);
        // Interior samples should be near zero
        for &v in &filtered[200..800] {
            assert!(v.abs() < 0.1, "DC should be rejected: {}", v);
        }
    }

    #[test]
    fn test_filtfilt_zero_phase() {
        // Zero-phase filtering should produce no group delay.
        // Create an impulse and verify the peak stays at the same position.
        let mut signal = vec![0.0; 500];
        signal[250] = 1.0;
        let filtered = filtfilt_lowpass(&signal, 100.0, 1000.0, 2);
        // Peak should be at or very near index 250
        let peak_idx = filtered
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert!(
            (peak_idx as i64 - 250).abs() <= 1,
            "Peak shifted to {}, expected ~250",
            peak_idx
        );
    }
}
