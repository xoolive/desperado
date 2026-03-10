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
    // For order N Butterworth, we need N/2 cascaded biquads (for even order)
    // The biquad crate provides Q values for proper Butterworth response

    let hz = cutoff.hz();
    let sample_rate = fs.hz();

    // For zero-phase filtering, we apply the filter twice (forward + backward)
    // This doubles the filter order, so we use order/2 for each pass
    let coeffs =
        Coefficients::<f64>::from_params(Type::LowPass, sample_rate, hz, Q_BUTTERWORTH_F64)
            .unwrap();

    // Forward pass
    let mut directform1 = DirectForm1::<f64>::new(coeffs);
    let mut y: Vec<f64> = x.iter().map(|&sample| directform1.run(sample)).collect();

    // Apply order/2 times for proper order
    for _ in 1..order.div_ceil(2) {
        let mut directform1 = DirectForm1::<f64>::new(coeffs);
        y = y.iter().map(|&sample| directform1.run(sample)).collect();
    }

    // Reverse
    y.reverse();

    // Backward pass (same number of biquads)
    let mut directform1 = DirectForm1::<f64>::new(coeffs);
    y = y.iter().map(|&sample| directform1.run(sample)).collect();

    for _ in 1..order.div_ceil(2) {
        let mut directform1 = DirectForm1::<f64>::new(coeffs);
        y = y.iter().map(|&sample| directform1.run(sample)).collect();
    }

    // Reverse back
    y.reverse();

    y
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

    // Forward pass
    let mut directform1 = DirectForm1::<f64>::new(coeffs);
    let mut y: Vec<f64> = x.iter().map(|&sample| directform1.run(sample)).collect();

    // Apply order/2 times
    for _ in 1..order.div_ceil(2) {
        let mut directform1 = DirectForm1::<f64>::new(coeffs);
        y = y.iter().map(|&sample| directform1.run(sample)).collect();
    }

    // Reverse
    y.reverse();

    // Backward pass
    let mut directform1 = DirectForm1::<f64>::new(coeffs);
    y = y.iter().map(|&sample| directform1.run(sample)).collect();

    for _ in 1..order.div_ceil(2) {
        let mut directform1 = DirectForm1::<f64>::new(coeffs);
        y = y.iter().map(|&sample| directform1.run(sample)).collect();
    }

    // Reverse back
    y.reverse();

    y
}
