//! Filter configuration constants for VOR and ILS decoders.
//!
//! This module centralizes all DSP filter parameters, threshold values, and
//! signal processing constants used across the crate. Each constant includes
//! documentation explaining its purpose, typical range, and the algorithm that
//! uses it.

// ── Sample rates ─────────────────────────────────────────────────────────────

/// VOR receiver input I/Q sample rate (Hz).
///
/// Typical capture from SDR hardware (gqrx, etc.). Provides sufficient bandwidth
/// to capture the full VOR signal (30 Hz reference + 9.96 kHz subcarrier modulation).
pub const VOR_SAMPLE_RATE_1_8M: u32 = 1_800_000;

/// VOR audio target rate (Hz).
///
/// After decimation from 1.8 MSps, this rate is sufficient to capture the 30 Hz
/// baseband signals and audio/ident modulation. Decimation factor = 1_800_000 / 48_000 ≈ 37.5.
pub const VOR_AUDIO_TARGET_RATE: f64 = 48_000.0;

/// ILS receiver input I/Q sample rate (Hz).
///
/// Typical capture from SDR hardware. Provides sufficient bandwidth
/// for 90 Hz and 150 Hz tone detection plus 1020 Hz Morse ident.
pub const ILS_SAMPLE_RATE_1_8M: u32 = 1_800_000;

/// ILS audio target rate (Hz).
///
/// Chosen as 9000 Hz to provide integer decimation factor from 1.8 MSps.
/// Nyquist = 4500 Hz, sufficient for both 90 Hz and 150 Hz tones.
/// Decimation factor = 1_800_000 / 9_000 = 200.
pub const ILS_AUDIO_RATE: f64 = 9_000.0;

// ── VOR filter parameters ────────────────────────────────────────────────────

/// VOR baseband lowpass filter cutoff frequency (Hz).
///
/// Removes out-of-band RF noise before further signal processing.
/// Typical range: 50–250 kHz for AM-modulated signals.
/// Value chosen to preserve the 30 Hz variable signal and voice modulation.
pub const VOR_BASEBAND_LPF_CUTOFF: f64 = 200_000.0;

/// VOR baseband lowpass filter order.
///
/// Higher order = steeper roll-off but more group delay.
/// Order 5 is a good compromise for VOR (VHF band noise attenuation).
pub const VOR_BASEBAND_LPF_ORDER: usize = 5;

/// VOR audio lowpass filter cutoff frequency (Hz).
///
/// Applied after baseband filter to remove RF noise that survives AM demodulation.
/// Typical range: 10–30 kHz. Chosen to preserve voice (300–3400 Hz) and Morse ident (1020 Hz).
pub const VOR_AUDIO_LPF_CUTOFF: f64 = 20_000.0;

/// VOR audio lowpass filter order.
pub const VOR_AUDIO_LPF_ORDER: usize = 5;

/// VOR variable subcarrier bandpass filter low cutoff (Hz).
///
/// The variable signal is AM-modulated on an ~9960 Hz subcarrier.
/// Bandpass range captures the subcarrier with sidebands.
pub const VOR_VAR_SUB_BPF_LOW: f64 = 9_000.0;

/// VOR variable subcarrier bandpass filter high cutoff (Hz).
pub const VOR_VAR_SUB_BPF_HIGH: f64 = 11_000.0;

/// VOR variable subcarrier bandpass filter order.
pub const VOR_VAR_SUB_BPF_ORDER: usize = 4;

/// VOR reference subcarrier bandpass filter low cutoff (Hz).
///
/// The reference signal is FM-modulated on a 9960 Hz subcarrier.
/// Narrower than variable to isolate pure reference tone.
pub const VOR_REF_SUB_BPF_LOW: f64 = 9_500.0;

/// VOR reference subcarrier bandpass filter high cutoff (Hz).
pub const VOR_REF_SUB_BPF_HIGH: f64 = 10_500.0;

/// VOR reference subcarrier bandpass filter order.
pub const VOR_REF_SUB_BPF_ORDER: usize = 4;

/// VOR 30 Hz lowpass filter cutoff frequency (Hz).
///
/// Applied after subcarrier extraction to isolate the 30 Hz modulation.
/// Value is higher than 30 Hz to account for filter roll-off and provide
/// good transient response to capture the underlying 30 Hz sine wave.
pub const VOR_30HZ_LPF_CUTOFF: f64 = 100.0;

/// VOR 30 Hz lowpass filter order.
pub const VOR_30HZ_LPF_ORDER: usize = 5;

/// VOR Morse ident bandpass filter low cutoff (Hz).
///
/// Isolates the 1020 Hz Morse ident tone before envelope extraction.
/// Typical Morse ident is 900–1100 Hz.
pub const VOR_MORSE_BPF_LOW: f64 = 900.0;

/// VOR Morse ident bandpass filter high cutoff (Hz).
pub const VOR_MORSE_BPF_HIGH: f64 = 1_100.0;

/// VOR Morse ident bandpass filter order.
pub const VOR_MORSE_BPF_ORDER: usize = 4;

/// VOR Morse ident lowpass filter cutoff frequency (Hz).
///
/// Applied to the envelope of the 1020 Hz Morse tone to smooth it.
/// Typical Morse ident transmission is ~12 WPM = ~100 ms dot duration,
/// so 40 Hz cutoff is sufficient to resolve individual dots while
/// suppressing carrier ripple.
pub const VOR_MORSE_ENV_LPF_CUTOFF: f64 = 40.0;

/// VOR Morse ident lowpass filter order.
pub const VOR_MORSE_ENV_LPF_ORDER: usize = 4;

/// VOR RMS threshold for signal normalization (Hz).
///
/// If normalized signal RMS falls below this value, it's considered
/// too weak and normalization is skipped to avoid amplifying noise.
pub const VOR_RMS_MIN: f64 = 1e-10;

// ── ILS filter parameters ────────────────────────────────────────────────────

/// ILS baseband lowpass filter cutoff frequency (Hz).
///
/// Pre-decimation filter removes out-of-band RF noise.
/// Typical range: 250–1000 Hz at full rate (1.8 MSps).
/// At post-decimation Nyquist of 4500 Hz (9 kHz rate), 500 Hz cutoff
/// preserves DC envelope and low-frequency modulation.
pub const ILS_BASEBAND_LPF_CUTOFF: f64 = 500.0;

/// ILS baseband lowpass filter order.
pub const ILS_BASEBAND_LPF_ORDER: usize = 5;

/// ILS 90 Hz tone bandpass filter low cutoff (Hz).
///
/// ILS 90 Hz localizer tone: deeper on the left of runway centreline.
/// Bandpass range: 80–100 Hz captures the tone with sidebands.
pub const ILS_90HZ_BPF_LOW: f64 = 80.0;

/// ILS 90 Hz tone bandpass filter high cutoff (Hz).
pub const ILS_90HZ_BPF_HIGH: f64 = 100.0;

/// ILS 90 Hz tone bandpass filter order.
pub const ILS_90HZ_BPF_ORDER: usize = 4;

/// ILS 150 Hz tone bandpass filter low cutoff (Hz).
///
/// ILS 150 Hz localizer tone: deeper on the right of runway centreline.
/// Bandpass range: 140–160 Hz captures the tone with sidebands.
pub const ILS_150HZ_BPF_LOW: f64 = 140.0;

/// ILS 150 Hz tone bandpass filter high cutoff (Hz).
pub const ILS_150HZ_BPF_HIGH: f64 = 160.0;

/// ILS 150 Hz tone bandpass filter order.
pub const ILS_150HZ_BPF_ORDER: usize = 4;

/// ILS Morse ident bandpass filter low cutoff (Hz).
///
/// Isolates the 1020 Hz Morse ident tone from the AM-modulated carrier.
/// Bandpass range: 900–1100 Hz.
pub const ILS_MORSE_BPF_LOW: f64 = 900.0;

/// ILS Morse ident bandpass filter high cutoff (Hz).
pub const ILS_MORSE_BPF_HIGH: f64 = 1_100.0;

/// ILS Morse ident bandpass filter order (IIR).
///
/// ILS uses zero-phase IIR filtfilt for sharper, better-normalized
/// bandpass response compared to FIR. Order 2 with filtfilt (4 biquads total,
/// 2 forward + 2 backward) provides Q ≈ 5 at 1020 Hz center.
pub const ILS_MORSE_BPF_ORDER: usize = 2;

/// ILS Morse ident lowpass filter cutoff frequency (Hz).
///
/// Applied to the envelope of the 1020 Hz Morse tone.
/// The Hilbert envelope of a narrow-band 1020 Hz tone has residual
/// oscillation at ~36 Hz (from noise beating). 16 Hz stated cutoff
/// applied via cascaded-biquad filtfilt with order=4 has an effective
/// -3 dB at ~0.5 × stated = ~8 Hz, which suppresses the 36 Hz ripple
/// while resolving 100 ms dots at 9 kHz audio rate.
pub const ILS_MORSE_ENV_LPF_CUTOFF: f64 = 16.0;

/// ILS Morse ident lowpass filter order (IIR).
///
/// Applied via cascaded-biquad filtfilt; order 4 means 2 biquads
/// each direction (4 total) for sharper roll-off.
pub const ILS_MORSE_ENV_LPF_ORDER: usize = 4;

// ── ILS thresholds ──────────────────────────────────────────────────────────

/// ILS DDM threshold for on-course (Hz).
///
/// |DDM| ≤ 0.015 → on centreline
/// |DDM| > 0.015 → off centreline (left or right)
pub const ILS_DDM_ON_COURSE_THRESHOLD: f64 = 0.015;

/// ILS DDM computation threshold (minimum sum of 90 + 150 Hz).
///
/// If (mean_90 + mean_150) < this value, both tones are near zero
/// (noise floor) and DDM computation is unreliable. Fail gracefully.
pub const ILS_DDM_SUM_MIN: f64 = 1e-9;

/// ILS signal strength normalization factor.
///
/// Carrier strength = mean_envelope / this factor, clamped to [0, 1].
/// Typical normalized envelope magnitude is ~0.5, so dividing by 0.5
/// maps 0.5 → 1.0 (nominal), <0.1 → <0.2 (weak), >1.0 → 1.0 (clipped).
pub const ILS_CARRIER_NORMALIZATION: f64 = 0.5;

// ── Common detection thresholds ──────────────────────────────────────────────

/// Minimum RMS value before signal is considered noise.
///
/// Used in signal normalization across both VOR and ILS.
/// Below this threshold, RMS-based normalization is skipped
/// to avoid amplifying noise floor.
pub const SIGNAL_RMS_MIN: f64 = 1e-10;

/// Minimum sum of modulation depths for ILS DDM calculation.
///
/// If both 90 Hz and 150 Hz tones are below noise floor,
/// DDM is undefined. This threshold gates the calculation.
pub const MODULATION_DEPTH_MIN: f64 = 1e-9;

/// Minimum FFT bin magnitude before considering signal present.
///
/// Used when checking if a frequency component has sufficient power.
pub const FFT_BIN_MIN: f64 = 1e-12;

// ── Signal processing constants ──────────────────────────────────────────────

/// VOR reference signal frequency (Hz).
///
/// The 30 Hz reference modulation is a fundamental constant
/// of the VOR system definition (ICAO Annex 10).
pub const VOR_REFERENCE_FREQ: f64 = 30.0;

/// VOR reference subcarrier nominal frequency (Hz).
///
/// The reference signal is FM-modulated on approximately 9960 Hz.
/// Nominal design value; actual Airspy capture may vary ±10 Hz.
pub const VOR_REFERENCE_SUBCARRIER_FREQ: f64 = 9_960.0;

/// ILS baseband downsampling target audio rate (Hz).
///
/// Decimation factor from I/Q rate is calculated as
/// ceil(input_rate / this_value).
pub const ILS_DECIMATION_TARGET_RATE: f64 = 9_000.0;

/// VOR baseband downsampling target audio rate (Hz).
///
/// Decimation factor from I/Q rate is calculated as
/// ceil(input_rate / this_value).
pub const VOR_DECIMATION_TARGET_RATE: f64 = 48_000.0;

/// Minimum sample count for reliable radial calculation.
///
/// VOR radial is estimated from at least 1 second of filtered
/// 30 Hz phase data. This constant ensures FFT-based phase
/// estimation has enough data points for accuracy.
pub const VOR_MIN_SAMPLES_FOR_RADIAL: f64 = 1.0; // seconds
