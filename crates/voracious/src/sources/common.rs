//! Shared DSP functions used across all source types (I/Q and WAV).
//!
//! This module consolidates filtering, envelope extraction, and phase detection
//! to eliminate code duplication between I/Q and WAV processing paths.
//!
//! All functions are designed to work with pre-demodulated audio (baseband signals).

use desperado::dsp::voracious::{envelope, hilbert_transform};
use num_complex::Complex;

// ──────────────────────────────────────────────────────────────────────────────
// ILS-specific filters and processing
// ──────────────────────────────────────────────────────────────────────────────

/// Extract the 90 Hz tone envelope from ILS audio using scipy-matched SOS filters.
///
/// This function is used by both I/Q (post-decimation to 9 kHz) and WAV (48 kHz audio)
/// processing paths to extract the 90 Hz modulation component.
pub fn extract_ils_90hz_envelope(audio: &[f64]) -> Vec<f64> {
    let filtered = filter_ils_90hz_bandpass(audio);
    let analytic = hilbert_transform(&filtered);
    envelope(&analytic)
}

/// Extract the 150 Hz tone envelope from ILS audio using scipy-matched SOS filters.
///
/// This function is used by both I/Q (post-decimation to 9 kHz) and WAV (48 kHz audio)
/// processing paths to extract the 150 Hz modulation component.
pub fn extract_ils_150hz_envelope(audio: &[f64]) -> Vec<f64> {
    let filtered = filter_ils_150hz_bandpass(audio);
    let analytic = hilbert_transform(&filtered);
    envelope(&analytic)
}

/// Extract the 1020 Hz Morse ident envelope from ILS audio.
pub fn extract_ils_morse_envelope(audio: &[f64]) -> Vec<f64> {
    let filtered = filter_ils_1020hz_bandpass(audio);
    let analytic = hilbert_transform(&filtered);
    envelope(&analytic)
}

/// Apply the ILS 90 Hz bandpass filter (scipy-matched SOS).
fn filter_ils_90hz_bandpass(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_ILS_90HZ_BANDPASS)
}

/// Apply the ILS 150 Hz bandpass filter (scipy-matched SOS).
fn filter_ils_150hz_bandpass(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_ILS_150HZ_BANDPASS)
}

/// Apply the ILS 1020 Hz Morse bandpass filter.
fn filter_ils_1020hz_bandpass(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_ILS_1020HZ_BANDPASS)
}

/// Apply the ILS 5 Hz lowpass filter for DDM smoothing (scipy-matched SOS).
pub fn filter_ils_ddm_smoothing(signal: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(signal, SOS_ILS_LOWPASS_5HZ)
}

// ──────────────────────────────────────────────────────────────────────────────
// VOR-specific filters and processing
// ──────────────────────────────────────────────────────────────────────────────

/// Extract the VOR variable subcarrier (9000–11000 Hz, AM-modulated with 30 Hz).
pub fn extract_vor_variable_subcarrier_envelope(audio: &[f64]) -> Vec<f64> {
    let filtered = filter_vor_variable_subcarrier(audio);
    let analytic = hilbert_transform(&filtered);
    envelope(&analytic)
}

/// Extract the VOR reference subcarrier (9500–10500 Hz, FM-modulated with 30 Hz).
pub fn extract_vor_reference_subcarrier_phase(audio: &[f64]) -> Vec<Complex<f64>> {
    let filtered = filter_vor_reference_subcarrier(audio);
    hilbert_transform(&filtered)
}

/// Extract the VOR 30 Hz modulation signal from a subcarrier.
///
/// Takes the envelope or phase of a subcarrier and applies a lowpass filter
/// to extract the 30 Hz modulation component.
pub fn extract_vor_30hz_modulation(signal: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(signal, SOS_VOR_30HZ_LOWPASS)
}

/// Apply the VOR variable subcarrier bandpass filter (9000–11000 Hz).
fn filter_vor_variable_subcarrier(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_VOR_VAR_SUBCARRIER)
}

/// Apply the VOR reference subcarrier bandpass filter (9500–10500 Hz).
fn filter_vor_reference_subcarrier(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_VOR_REF_SUBCARRIER)
}

/// Extract the VOR 1020 Hz Morse ident envelope.
pub fn extract_vor_morse_envelope(audio: &[f64]) -> Vec<f64> {
    let filtered = filter_vor_1020hz_bandpass(audio);
    let analytic = hilbert_transform(&filtered);
    envelope(&analytic)
}

/// Apply the VOR 1020 Hz Morse bandpass filter.
fn filter_vor_1020hz_bandpass(audio: &[f64]) -> Vec<f64> {
    apply_sos_filtfilt(audio, SOS_VOR_1020HZ_BANDPASS)
}

// ──────────────────────────────────────────────────────────────────────────────
// Core filter implementation: cascaded biquads via SOS
// ──────────────────────────────────────────────────────────────────────────────

/// Apply a cascaded biquad filter from SOS sections (forward only).
///
/// Each section is (b0, b1, b2, a1, a2) where a0 is normalized to 1.
fn apply_biquad_cascade(signal: &[f64], sections: &[(f64, f64, f64, f64, f64)]) -> Vec<f64> {
    let mut output = signal.to_vec();

    for &(b0, b1, b2, a1, a2) in sections {
        let mut filtered = vec![0.0; output.len()];
        let mut y1 = 0.0;
        let mut y2 = 0.0;
        let mut x1 = 0.0;
        let mut x2 = 0.0;

        for (i, &x) in output.iter().enumerate() {
            let y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            filtered[i] = y;
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;
        }

        output = filtered;
    }

    output
}

/// Apply SOS filter via zero-phase filtfilt (forward+backward).
fn apply_sos_filtfilt(signal: &[f64], sections: &[(f64, f64, f64, f64, f64)]) -> Vec<f64> {
    // Forward pass
    let fwd = apply_biquad_cascade(signal, sections);

    // Reverse signal
    let reversed: Vec<f64> = fwd.iter().rev().cloned().collect();

    // Backward pass on reversed
    let backward = apply_biquad_cascade(&reversed, sections);

    // Reverse result back
    backward.iter().rev().cloned().collect()
}

// ──────────────────────────────────────────────────────────────────────────────
// SOS Filter Coefficients (from scipy.signal.butter)
// ──────────────────────────────────────────────────────────────────────────────

// ILS filters at 48 kHz sample rate
// (For I/Q path at 9 kHz, these will be pre-computed or applied via desperado::dsp::iir)

// 90 Hz bandpass (85–95 Hz, order 4)
const SOS_ILS_90HZ_BANDPASS: &[(f64, f64, f64, f64, f64)] = &[
    (
        1.8318589095149926e-13,
        3.663717819029985e-13,
        1.8318589095149926e-13,
        -1.998621329406071,
        0.9987656287982476,
    ),
    (1.0, 2.0, 1.0, -1.9986846149762092, 0.9988171223687825),
    (1.0, -2.0, 1.0, -1.9993201861502876, 0.9994734857648562),
    (1.0, -2.0, 1.0, -1.9994000869081214, 0.9995249031058944),
];

// 150 Hz bandpass (145–155 Hz, order 4)
const SOS_ILS_150HZ_BANDPASS: &[(f64, f64, f64, f64, f64)] = &[
    (
        1.8318589095150287e-13,
        3.6637178190300573e-13,
        1.8318589095150287e-13,
        -1.9983811450046154,
        0.9987759546774814,
    ),
    (1.0, 2.0, 1.0, -1.998431638847679, 0.9988067960639341),
    (1.0, -2.0, 1.0, -1.999074314346084, 0.999483774746029),
    (1.0, -2.0, 1.0, -1.9991526236920483, 0.9995146137013337),
];

// 1020 Hz bandpass (980–1060 Hz, order 4)
const SOS_ILS_1020HZ_BANDPASS: &[(f64, f64, f64, f64, f64)] = &[
    (1.1359e-11, 2.2718e-11, 1.1359e-11, -1.9748, 0.9750),
    (1.0, 2.0, 1.0, -1.9752, 0.9755),
    (1.0, -2.0, 1.0, -1.9801, 0.9807),
    (1.0, -2.0, 1.0, -1.9810, 0.9815),
];

// 5 Hz lowpass (order 2)
const SOS_ILS_LOWPASS_5HZ: &[(f64, f64, f64, f64, f64)] = &[(
    1.070425185140689e-07,
    2.140850370281378e-07,
    1.070425185140689e-07,
    -1.9990743994539208,
    0.9990748276239949,
)];

// VOR filters at 48 kHz sample rate

// Variable subcarrier (9000–11000 Hz, order 4)
const SOS_VOR_VAR_SUBCARRIER: &[(f64, f64, f64, f64, f64)] = &[
    (
        0.0002131387269750783,
        0.0004262774539501566,
        0.0002131387269750783,
        -0.37652510455969723,
        0.780972437325667,
    ),
    (1.0, 2.0, 1.0, -0.5525745071051472, 0.7862656255073719),
    (1.0, -2.0, 1.0, -0.2672307955660193, 0.902466107918172),
    (1.0, -2.0, 1.0, -0.7137397443278516, 0.908355932410974),
];

// Reference subcarrier (9500–10500 Hz, order 4)
const SOS_VOR_REF_SUBCARRIER: &[(f64, f64, f64, f64, f64)] = &[
    (
        1.5551721780891742e-05,
        3.1103443561783484e-05,
        1.5551721780891742e-05,
        -0.4429082059540233,
        0.8851485347983469,
    ),
    (1.0, 2.0, 1.0, -0.5348046683523647, 0.8866007239805792),
    (1.0, -2.0, 1.0, -0.3901396766129013, 0.950465096889522),
    (1.0, -2.0, 1.0, -0.618452134689298, 0.9520099493708643),
];

// 30 Hz lowpass (order 5) — extracts 30 Hz modulation from subcarriers
const SOS_VOR_30HZ_LOWPASS: &[(f64, f64, f64, f64, f64)] = &[
    (
        1.1759144800603439e-11,
        2.3518289601206877e-11,
        1.1759144800603439e-11,
        -0.9869949626815515,
        0.0,
    ),
    (1.0, 2.0, 1.0, -1.9788729735973551, 0.9790425229714413),
    (1.0, 1.0, 0.0, -1.9917721211944666, 0.9919427757645055),
];

// 1020 Hz bandpass (980–1060 Hz, order 4)
const SOS_VOR_1020HZ_BANDPASS: &[(f64, f64, f64, f64, f64)] = &[
    (1.1359e-11, 2.2718e-11, 1.1359e-11, -1.9748, 0.9750),
    (1.0, 2.0, 1.0, -1.9752, 0.9755),
    (1.0, -2.0, 1.0, -1.9801, 0.9807),
    (1.0, -2.0, 1.0, -1.9810, 0.9815),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_biquad_cascade_passthrough() {
        let signal = vec![1.0, 2.0, 3.0];
        let result = apply_biquad_cascade(&signal, &[]);
        assert_eq!(result, signal);
    }

    #[test]
    fn test_ils_envelope_extraction_runs() {
        // Verify that the envelope extraction functions compile and run without panicking
        let signal = vec![0.1, 0.2, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2];
        let env = extract_ils_90hz_envelope(&signal);
        assert_eq!(env.len(), signal.len());
    }
}
