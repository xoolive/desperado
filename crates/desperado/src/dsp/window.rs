//! Window functions for filter design (incl. Hamming, Blackman and Hann).
//!
//! This module provides common window functions used in digital filter design, particularly
//! for windowed-sinc FIR filter implementations. Windows are used to minimize spectral leakage
//! and control the tradeoff between main lobe width and sidelobe attenuation.

use std::f32::consts::PI;

/// Enumeration of available window types for filter design.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowType {
    /// Hamming window: good all-around characteristics
    /// Main lobe width: 8π/N, First sidelobe: -43 dB
    Hamming,
    /// Blackman window: excellent sidelobe attenuation
    /// Main lobe width: 12π/N, First sidelobe: -58 dB
    Blackman,
    /// Hann (Hanning) window: smoother rolloff than Hamming
    /// Main lobe width: 8π/N, First sidelobe: -32 dB
    Hann,
}

/// Compute a window value for the given tap index.
///
/// # Arguments
/// * `n` - Tap index (0 to taps-1)
/// * `taps` - Total number of taps
/// * `window_type` - Type of window to compute
///
/// # Returns
/// The window coefficient for this tap
pub fn compute_window(n: usize, taps: usize, window_type: WindowType) -> f32 {
    let normalized_n = n as f32 / (taps as f32 - 1.0);

    match window_type {
        WindowType::Hamming => {
            // Hamming: w(n) = 0.54 - 0.46*cos(2πn/N)
            0.54 - 0.46 * (2.0 * PI * normalized_n).cos()
        }
        WindowType::Blackman => {
            // Blackman: w(n) = 0.42 - 0.5*cos(2πn/N) + 0.08*cos(4πn/N)
            0.42 - 0.5 * (2.0 * PI * normalized_n).cos() + 0.08 * (4.0 * PI * normalized_n).cos()
        }
        WindowType::Hann => {
            // Hann (Hanning): w(n) = 0.5 - 0.5*cos(2πn/N)
            0.5 - 0.5 * (2.0 * PI * normalized_n).cos()
        }
    }
}

/// Normalize filter coefficients to unity gain (sum of coefficients = 1.0).
///
/// This is typically applied after windowing and sinc computation to ensure
/// the filter has approximately unity gain in the passband.
///
/// # Arguments
/// * `coeffs` - Mutable slice of filter coefficients
pub fn normalize_coeffs(coeffs: &mut [f32]) {
    let sum: f32 = coeffs.iter().sum();
    if sum != 0.0 {
        for coeff in coeffs.iter_mut() {
            *coeff /= sum;
        }
    }
}

/// Design a windowed-sinc FIR low-pass filter.
///
/// Creates FIR filter coefficients using the windowed-sinc method with the specified window type.
/// This is the primary interface for designing standard low-pass filters.
///
/// # Arguments
/// * `cutoff` - Normalized cutoff frequency (0.0 to 0.5, where 0.5 is the Nyquist frequency)
/// * `taps` - Number of filter taps (should be odd for symmetric filters)
/// * `window_type` - Type of window to apply
///
/// # Returns
/// Vector of normalized FIR filter coefficients
///
/// # Panics
/// Panics if `cutoff` is not in (0.0, 0.5], or if `taps` is 0.
///
/// # Example
/// ```
/// use desperado::dsp::window::{design_fir_filter, WindowType};
///
/// // Design a 31-tap Hamming window filter with cutoff at Fs/16
/// let filter = design_fir_filter(0.0625, 31, WindowType::Hamming);
/// assert_eq!(filter.len(), 31);
/// ```
pub fn design_fir_filter(cutoff: f32, taps: usize, window_type: WindowType) -> Vec<f32> {
    assert!(taps > 0, "Number of taps must be greater than 0");
    assert!(
        cutoff > 0.0 && cutoff <= 0.5,
        "Cutoff must be in range (0.0, 0.5]"
    );

    let mut fir = Vec::with_capacity(taps);
    let mid = (taps / 2) as isize;

    // Design windowed-sinc filter
    for n in 0..taps {
        let x = n as isize - mid;

        // Sinc function: sinc(x) = sin(πx) / (πx), with sinc(0) = 1
        let sinc = if x == 0 {
            2.0 * cutoff
        } else {
            (2.0 * cutoff * PI * x as f32).sin() / (PI * x as f32)
        };

        // Apply window
        let window = compute_window(n, taps, window_type);
        fir.push(sinc * window);
    }

    // Normalize to unity gain
    normalize_coeffs(&mut fir);
    fir
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_hamming_window() {
        // Hamming window should start and end near zero
        assert_relative_eq!(
            compute_window(0, 31, WindowType::Hamming),
            0.08,
            epsilon = 0.01
        );
        assert_relative_eq!(
            compute_window(30, 31, WindowType::Hamming),
            0.08,
            epsilon = 0.01
        );
        // Peak at center
        assert_relative_eq!(
            compute_window(15, 31, WindowType::Hamming),
            1.0,
            epsilon = 0.01
        );
    }

    #[test]
    fn test_blackman_window() {
        // Blackman window should start and end very close to zero
        assert_relative_eq!(
            compute_window(0, 31, WindowType::Blackman),
            0.0,
            epsilon = 0.001
        );
        assert_relative_eq!(
            compute_window(30, 31, WindowType::Blackman),
            0.0,
            epsilon = 0.001
        );
        // Peak at center
        assert_relative_eq!(
            compute_window(15, 31, WindowType::Blackman),
            1.0,
            epsilon = 0.01
        );
    }

    #[test]
    fn test_hann_window() {
        // Hann window should start and end at zero
        assert_relative_eq!(compute_window(0, 31, WindowType::Hann), 0.0, epsilon = 1e-6);
        assert_relative_eq!(
            compute_window(30, 31, WindowType::Hann),
            0.0,
            epsilon = 1e-6
        );
        // Peak at center
        assert_relative_eq!(
            compute_window(15, 31, WindowType::Hann),
            1.0,
            epsilon = 0.01
        );
    }

    #[test]
    fn test_normalize_coeffs() {
        let mut coeffs = vec![1.0, 2.0, 1.0];
        normalize_coeffs(&mut coeffs);
        assert_relative_eq!(coeffs.iter().sum::<f32>(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_design_fir_filter_hamming() {
        let filter = design_fir_filter(0.125, 31, WindowType::Hamming);
        assert_eq!(filter.len(), 31);
        // Coefficients should sum to 1.0 (unity gain)
        let sum: f32 = filter.iter().sum();
        assert_relative_eq!(sum, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_design_fir_filter_blackman() {
        let filter = design_fir_filter(0.0625, 63, WindowType::Blackman);
        assert_eq!(filter.len(), 63);
        let sum: f32 = filter.iter().sum();
        assert_relative_eq!(sum, 1.0, epsilon = 1e-6);
    }

    #[test]
    #[should_panic(expected = "Cutoff must be in range")]
    fn test_design_fir_invalid_cutoff_zero() {
        design_fir_filter(0.0, 31, WindowType::Hamming);
    }

    #[test]
    #[should_panic(expected = "Cutoff must be in range")]
    fn test_design_fir_invalid_cutoff_too_high() {
        design_fir_filter(0.6, 31, WindowType::Hamming);
    }

    #[test]
    #[should_panic(expected = "Number of taps must be greater than 0")]
    fn test_design_fir_zero_taps() {
        design_fir_filter(0.125, 0, WindowType::Hamming);
    }
}
