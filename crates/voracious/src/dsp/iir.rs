//! IIR filters with zero-phase filtering (filtfilt)
//!
//! Uses the biquad crate for proper Butterworth filter implementation

use biquad::*;

/// Apply zero-phase filtering (forward + backward pass) using biquad filter
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

/// Apply zero-phase bandpass filtering
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
