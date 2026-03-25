//! Signal quality metrics and analysis for VOR signals
//!
//! This module provides functions to compute and format various signal quality
//! metrics that characterize the strength, clarity, and stability of VOR reception.
//!
//! # Metrics Computed
//!
//! - **SNR at 9960 Hz:** Signal-to-noise ratio of the main VOR identification tone
//! - **SNR at 30 Hz:** Signal-to-noise ratio of the reference/variable 30 Hz subcarrier
//! - **Phase Lock Quality:** Correlation between variable and reference signals
//! - **Radial Variance:** Circular variance of measured radial headings (stability indicator)
//! - **Clipping Ratio:** Percentage of I/Q samples that saturate (overload detector)
//!
//! # Algorithm Notes
//!
//! - **Tone Power Measurement:** Uses I/Q correlation at specific frequencies via Goertzel-like algorithm
//! - **Phase Lock:** Computes normalized dot product of var_30 and ref_30 signals
//! - **Circular Variance:** Converts degree-based radials to unit vectors, measures spread
//! - **Clipping Detection:** Tracks when |I| ≥ 0.98 or |Q| ≥ 0.98 during sample collection

use super::vor::SignalQualityMetrics;

/// Compute all signal quality metrics for a VOR measurement window.
///
/// This is the main entry point for quality analysis. It orchestrates measurements
/// of SNR, phase lock, clipping, and radial stability.
///
/// # Arguments
///
/// - `clipping_ratio`: Optional measured clipping ratio (0.0 - 1.0)
/// - `audio`: Full audio output from demodulator (3 kHz sample rate)
/// - `var_30`: Variable 30 Hz subcarrier signal
/// - `ref_30`: Reference 30 Hz subcarrier signal
/// - `sample_rate`: Audio sample rate (Hz)
/// - `window_iq_count`: Total I/Q samples collected in this window (for clipping resolution)
/// - `radial_history`: Recent radial measurements (degrees)
///
/// # Returns
///
/// A `SignalQualityMetrics` struct with formatted strings for display.
pub fn compute_signal_quality(
    clipping_ratio: Option<f64>,
    audio: &[f64],
    var_30: &[f64],
    ref_30: &[f64],
    sample_rate: f64,
    window_iq_count: usize,
    radial_history: &[f64],
) -> SignalQualityMetrics {
    // VOR identification tone at 9960 Hz with noise bands above/below
    let snr_9960_db = tone_snr_db(
        audio,
        sample_rate,
        9960.0,
        &[9600.0, 9700.0, 9800.0, 10100.0, 10200.0, 10300.0],
    );

    // 30 Hz subcarrier SNR (reference + variable combined)
    let p30_var = rms_power(var_30);
    let p30_ref = rms_power(ref_30);
    let p30_min = p30_var.min(p30_ref);
    let n30 = avg_tone_power(var_30, sample_rate, &[15.0, 20.0, 25.0, 35.0, 40.0, 45.0])
        .min(avg_tone_power(
            ref_30,
            sample_rate,
            &[15.0, 20.0, 25.0, 35.0, 40.0, 45.0],
        ))
        .max(1e-12);
    let snr_30_db = 10.0 * ((p30_min + 1e-12) / n30).log10();

    // Correlation between var and ref 30 Hz signals (phase lock indicator)
    let lock_score = phase_lock_score(var_30, ref_30);

    // Circular variance of recent radials (lower = more stable)
    let radial_var = circular_variance(radial_history);

    // Format clipping ratio, showing measurement floor if zero
    let clipping_ratio_raw = clipping_ratio.unwrap_or(0.0).clamp(0.0, 1.0);
    let clipping_ratio_display = if clipping_ratio_raw == 0.0 {
        // If no clipped sample is observed in this output window,
        // report the measurement floor instead of a hard zero.
        let resolution = 1.0 / (window_iq_count.max(1) as f64);
        format!("<{:.2e}", resolution)
    } else {
        format_scientific(clipping_ratio_raw, 2)
    };

    SignalQualityMetrics {
        clipping_ratio: clipping_ratio_display,
        snr_30hz_db: round_decimals(snr_30_db, 2),
        snr_9960hz_db: round_decimals(snr_9960_db, 2),
        lock_quality: format_scientific(lock_score, 2),
        radial_variance: format_scientific(radial_var, 2),
    }
}

// ============================================================================
// Formatting Utilities
// ============================================================================

/// Round a floating-point value to a specified number of decimal places.
pub fn round_decimals(value: f64, decimals: u32) -> f64 {
    let factor = 10_f64.powi(decimals as i32);
    (value * factor).round() / factor
}

/// Format a floating-point number in scientific notation with specified precision.
fn format_scientific(value: f64, decimals: usize) -> String {
    format!("{value:.decimals$e}")
}

// ============================================================================
// Power and Tone Measurement
// ============================================================================

/// Compute RMS (root mean square) power of a signal.
///
/// Power = mean(x^2)
fn rms_power(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64
}

/// Compute the power of a specific frequency tone in a signal.
///
/// Uses a Goertzel-like approach: correlates signal with sin/cos at target frequency,
/// returns the magnitude squared of the complex result.
///
/// # Arguments
///
/// - `signal`: Input samples
/// - `sample_rate`: Sample rate in Hz
/// - `freq`: Target frequency in Hz
fn tone_power(signal: &[f64], sample_rate: f64, freq: f64) -> f64 {
    if signal.len() < 4 || sample_rate <= 0.0 {
        return 0.0;
    }

    let n = signal.len() as f64;
    let mut i_sum = 0.0;
    let mut q_sum = 0.0;
    for (k, &x) in signal.iter().enumerate() {
        let phase = 2.0 * std::f64::consts::PI * freq * k as f64 / sample_rate;
        i_sum += x * phase.cos();
        q_sum += x * phase.sin();
    }

    (i_sum * i_sum + q_sum * q_sum) / (n * n)
}

/// Compute average tone power across multiple frequencies.
fn avg_tone_power(signal: &[f64], sample_rate: f64, freqs: &[f64]) -> f64 {
    if freqs.is_empty() {
        return 0.0;
    }
    let sum: f64 = freqs
        .iter()
        .map(|&f| tone_power(signal, sample_rate, f))
        .sum();
    sum / freqs.len() as f64
}

/// Compute SNR in dB for a specific tone against noise at other frequencies.
///
/// SNR = 10 * log10(P_signal / P_noise)
fn tone_snr_db(signal: &[f64], sample_rate: f64, tone_freq: f64, noise_freqs: &[f64]) -> f64 {
    let p_sig = tone_power(signal, sample_rate, tone_freq).max(1e-12);
    let p_noise = avg_tone_power(signal, sample_rate, noise_freqs).max(1e-12);
    10.0 * (p_sig / p_noise).log10()
}

// ============================================================================
// Signal Quality Indicators
// ============================================================================

/// Compute phase lock quality between variable and reference 30 Hz signals.
///
/// Returns the normalized correlation (0.0 = no correlation, 1.0 = perfect alignment).
///
/// This measures how well the variable signal (which rotates at the aircraft's bearing)
/// is synchronized with the reference signal. High lock quality indicates stable reception.
fn phase_lock_score(var_30: &[f64], ref_30: &[f64]) -> f64 {
    let n = var_30.len().min(ref_30.len());
    if n < 8 {
        return 0.0;
    }

    let (mut dot, mut v2, mut r2) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let v = var_30[i];
        let r = ref_30[i];
        dot += v * r;
        v2 += v * v;
        r2 += r * r;
    }

    let denom = (v2 * r2).sqrt();
    if denom <= 1e-12 {
        0.0
    } else {
        (dot.abs() / denom).clamp(0.0, 1.0)
    }
}

/// Compute circular variance of a set of bearings/radials.
///
/// Converts degrees to unit vectors on the unit circle, measures the
/// spread of the resulting distribution. Returns 0.0 for perfect alignment,
/// 1.0 for completely uniform distribution.
fn circular_variance(radials_deg: &[f64]) -> f64 {
    if radials_deg.len() < 2 {
        return 1.0;
    }
    let mut c = 0.0;
    let mut s = 0.0;
    for &deg in radials_deg {
        let r = deg.to_radians();
        c += r.cos();
        s += r.sin();
    }
    let n = radials_deg.len() as f64;
    let r = (c * c + s * s).sqrt() / n;
    (1.0 - r).clamp(0.0, 1.0)
}

// ============================================================================
// Morse Decoding Utilities
// ============================================================================

/// Estimate the repeat interval of Morse identification sequence.
///
/// Examines inter-hit intervals and attempts to identify the underlying
/// Morse cycle time, accounting for potential overlapping cycles.
///
/// # Algorithm
///
/// 1. Compute all intervals between consecutive hits (≥ 7 seconds)
/// 2. Try "folding" intervals by dividing by 1-5 (for harmonic relationships)
/// 3. Keep candidates in the normal VOR ident range (6-14 seconds)
/// 4. If folding produces good candidates, use median
/// 5. Otherwise, use median of raw deltas
pub fn estimate_repeat_interval_seconds(hits: &[f64]) -> Option<f64> {
    if hits.len() < 2 {
        return None;
    }
    let mut deltas: Vec<f64> = hits
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|d| *d >= 7.0)
        .collect();
    if deltas.is_empty() {
        return None;
    }

    let mut folded_candidates = Vec::new();
    for d in &deltas {
        for n in 1..=5 {
            let candidate = *d / n as f64;
            if (6.0..=14.0).contains(&candidate) {
                folded_candidates.push(candidate);
            }
        }
    }

    if !folded_candidates.is_empty() {
        folded_candidates.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        return Some(folded_candidates[folded_candidates.len() / 2]);
    }

    deltas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some(deltas[deltas.len() / 2])
}
