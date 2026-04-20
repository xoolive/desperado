//! VOR (VHF Omnidirectional Range) decoder.
//!
//! VOR is a radio navigation system providing bearing information to VOR ground stations.
//! This module demodulates VOR signals to extract:
//! - **30 Hz reference signal**: FM-modulated on a 9960 Hz subcarrier, provides phase reference
//! - **30 Hz variable signal**: AM-modulated on the same 9960 Hz subcarrier, rotates with station's
//!   mechanical or electronic phase shifter
//! - **Radial bearing**: Phase difference between variable and reference signals (0–360°)
//! - **Morse ident**: 4-letter station identifier on 1020 Hz tone
//!
//! ## Demodulation Process
//!
//! The [`VorDemodulator`] performs the following steps on each I/Q chunk:
//!
//! 1. **Frequency shifting**: Translates RF carrier to baseband (DC)
//! 2. **Baseband filtering**: 200 kHz lowpass removes out-of-band RF noise
//! 3. **AM demodulation**: Extracts baseband envelope via magnitude (|I² + Q²|)
//! 4. **Audio filtering**: 20 kHz lowpass removes RF artifacts
//! 5. **Decimation**: Reduces sample rate from 1.8 MSps to 48 kHz for efficient processing
//! 6. **Subcarrier extraction**:
//!    - Variable subcarrier: 9–11 kHz bandpass, then AM demodulation via Hilbert envelope
//!    - Reference subcarrier: 9.5–10.5 kHz bandpass, then FM demodulation via phase tracking
//! 7. **30 Hz extraction**: 100 Hz lowpass filters both signals to isolate the 30 Hz modulation
//!
//! ## Radial Calculation
//!
//! Two algorithms are provided:
//! - [`calculate_radial`]: FFT-based phase estimation at 30 Hz frequency bin (robust, requires ~1 sec data)
//! - [`calculate_radial_vortrack`]: Vortrack algorithm (reference implementation, real-time capable)
//!
//! ## Filter Design
//!
//! All filter parameters (cutoff frequencies, orders) are defined in the filter configuration.
//! Key design choices:
//! - Butterworth IIR filters chosen for phase linearity and stability
//! - Higher filter orders (4–5) for sharp roll-off in congested VHF band
//! - 30 Hz lowpass cutoff set to 100 Hz to account for filter group delay and capture transients
//!
//! Morse ident decoding is handled by the generic [`super::morse`] module.
//!
//! ## References
//!
//! - VOR stations transmit on 108–117.95 MHz (odd tenth-MHz channels)
//! - 30 Hz reference/variable subcarriers derived from ICAO Annex 10 standard
//! - Vortrack algorithm based on [Junzi Sun's Mode S decoder](https://github.com/junzis/pyModeS)

use num_complex::Complex;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ── VOR Filter & Processing Constants ────────────────────────────────────────

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

/// VOR Morse audio output bandpass filter low cutoff (Hz).
///
/// Matches analysis filter band (980-1060 Hz) for consistent audio output.
/// Uses stateful FIR implementation for streaming audio compatibility.
pub const VOR_MORSE_AUDIO_BPF_LOW: f64 = 980.0;

/// VOR Morse audio output bandpass filter high cutoff (Hz).
pub const VOR_MORSE_AUDIO_BPF_HIGH: f64 = 1_060.0;

/// VOR Morse audio output bandpass filter order.
///
/// Order 4 (65 taps FIR) provides smooth passband for audio output.
/// Applied to audio output only, not to signal measurements.
pub const VOR_MORSE_AUDIO_BPF_ORDER: usize = 4;

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

/// Minimum FFT bin magnitude before considering signal present.
///
/// Used when checking if a frequency component has sufficient power.
pub const FFT_BIN_MIN: f64 = 1e-12;

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

use super::metrics;
use super::morse;
use crate::dsp_utils::{envelope, hilbert_transform};
use desperado::dsp::filters::ButterworthFilter;
use desperado::dsp::iir::filtfilt_lowpass;

/// Sample rate constants for VOR decoding
pub const VOR_SAMPLE_RATE_1_8M: u32 = 1_800_000;
pub const VOR_AUDIO_TARGET_RATE: f64 = 48_000.0;

pub struct VorDemodulator {
    pub sample_rate: f64,
    audio_rate: f64,
    decim_factor: usize,

    // Filters
    baseband_lpf: ButterworthFilter,
    audio_lpf: ButterworthFilter,
    var_sub_bpf: ButterworthFilter,
    ref_sub_bpf: ButterworthFilter,
    var_30_lpf: ButterworthFilter,
    ref_30_lpf: ButterworthFilter,

    // State for phase continuity
    last_ref_phase: Option<f64>,
}

impl VorDemodulator {
    pub fn new(sample_rate: u32) -> Self {
        let sample_rate_f = sample_rate as f64;
        let decim_factor = ((sample_rate_f / VOR_DECIMATION_TARGET_RATE).round() as usize).max(1);
        let audio_rate = sample_rate_f / decim_factor as f64;

        Self {
            sample_rate: sample_rate_f,
            audio_rate,
            decim_factor,

            baseband_lpf: ButterworthFilter::lowpass(
                VOR_BASEBAND_LPF_CUTOFF,
                sample_rate_f,
                VOR_BASEBAND_LPF_ORDER,
            ),

            audio_lpf: ButterworthFilter::lowpass(
                VOR_AUDIO_LPF_CUTOFF,
                sample_rate_f,
                VOR_AUDIO_LPF_ORDER,
            ),

            var_sub_bpf: ButterworthFilter::bandpass(
                VOR_VAR_SUB_BPF_LOW,
                VOR_VAR_SUB_BPF_HIGH,
                audio_rate,
                VOR_VAR_SUB_BPF_ORDER,
            ),

            ref_sub_bpf: ButterworthFilter::bandpass(
                VOR_REF_SUB_BPF_LOW,
                VOR_REF_SUB_BPF_HIGH,
                audio_rate,
                VOR_REF_SUB_BPF_ORDER,
            ),

            var_30_lpf: ButterworthFilter::lowpass(
                VOR_30HZ_LPF_CUTOFF,
                audio_rate,
                VOR_30HZ_LPF_ORDER,
            ),
            ref_30_lpf: ButterworthFilter::lowpass(
                VOR_30HZ_LPF_CUTOFF,
                audio_rate,
                VOR_30HZ_LPF_ORDER,
            ),

            last_ref_phase: None,
        }
    }

    pub fn audio_rate(&self) -> f64 {
        self.audio_rate
    }

    /// Demodulate VOR signal and extract 30 Hz components.
    ///
    /// Performs the full demodulation pipeline on a chunk of I/Q samples:
    /// frequency shift → baseband/audio filtering → AM demodulation →
    /// decimation → subcarrier extraction → 30 Hz envelope extraction.
    ///
    /// # Arguments
    ///
    /// - `iq_samples`: Raw I/Q samples at the configured input sample rate
    /// - `freq_offset`: Frequency shift to apply before baseband filtering (Hz)
    ///
    /// # Returns
    ///
    /// A tuple of:
    /// - `variable_30hz`: Envelope of the 30 Hz variable signal (AM subcarrier)
    /// - `reference_30hz`: Phase signal from the 30 Hz reference (FM subcarrier)
    /// - `audio_samples`: Full AM envelope decimated to audio rate
    ///
    /// All outputs are at the audio rate (see [`audio_rate`](Self::audio_rate)).
    /// Returns empty vectors if input is empty.
    pub fn demodulate(
        &mut self,
        iq_samples: &[Complex<f32>],
        freq_offset: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if iq_samples.is_empty() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        // 1. Frequency shift to baseband
        let t_start = 0.0;
        let iq_shifted: Vec<Complex<f32>> = iq_samples
            .iter()
            .enumerate()
            .map(|(i, &sample)| {
                let t = t_start + (i as f64 / self.sample_rate);
                let shift = Complex::new(
                    (-2.0 * PI * freq_offset * t).cos() as f32,
                    (-2.0 * PI * freq_offset * t).sin() as f32,
                );
                sample * shift
            })
            .collect();

        // 2. Lowpass filter
        let iq_filtered = self.baseband_lpf.filter_complex(&iq_shifted);

        // 3. AM demodulation from carrier envelope
        let am_signal: Vec<f64> = iq_filtered.iter().map(|c| c.norm() as f64).collect();

        // Remove DC from AM envelope before audio filtering
        let am_mean = am_signal.iter().sum::<f64>() / am_signal.len() as f64;
        let am_centered: Vec<f64> = am_signal.iter().map(|x| x - am_mean).collect();

        // 4. Audio lowpass filter
        let audio = self.audio_lpf.filter(&am_centered);

        // 5. Decimate to audio rate
        let audio_decimated = decimate(&audio, self.decim_factor);

        // 6 & 7. Extract subcarriers and 30 Hz components
        let (var_30, ref_30) = self.extract_subcarriers_and_30hz(&audio_decimated);

        (var_30, ref_30, audio_decimated)
    }

    /// Extract subcarrier signals and their 30 Hz modulation from audio samples.
    ///
    /// This is the core subcarrier extraction logic, factored out so it can be
    /// reused by both I/Q demodulation and direct audio processing paths.
    ///
    /// # Arguments
    ///
    /// - `audio`: Audio samples already at the target audio rate (e.g., 48 kHz)
    ///
    /// # Returns
    ///
    /// A tuple of:
    /// - `variable_30hz`: Envelope of the 30 Hz variable signal (AM subcarrier)
    /// - `reference_30hz`: Phase signal from the 30 Hz reference (FM subcarrier)
    ///
    /// Both outputs are at the audio rate. Returns empty vectors if input is empty.
    pub fn extract_subcarriers_and_30hz(&mut self, audio: &[f64]) -> (Vec<f64>, Vec<f64>) {
        if audio.is_empty() {
            return (Vec::new(), Vec::new());
        }

        // Extract subcarriers (bandpass filtered at audio rate)
        let var_sub = self.var_sub_bpf.filter(audio);
        let ref_sub = self.ref_sub_bpf.filter(audio);

        // Variable: AM demodulation (envelope)
        let var_analytic = hilbert_transform(&var_sub);
        let var_envelope = envelope(&var_analytic);

        // Remove DC from variable signal
        if var_envelope.is_empty() || ref_sub.is_empty() {
            return (Vec::new(), Vec::new());
        }

        let var_envelope_mean = var_envelope.iter().sum::<f64>() / var_envelope.len() as f64;
        let var_centered: Vec<f64> = var_envelope.iter().map(|x| x - var_envelope_mean).collect();
        let var_30 = self.var_30_lpf.filter(&var_centered);

        // Reference: FM demodulation of subcarrier
        let mut ref_phase_signal = Vec::with_capacity(ref_sub.len());
        let ref_analytic = hilbert_transform(&ref_sub);

        let mut prev_ref_phase = self.last_ref_phase.unwrap_or_else(|| {
            if !ref_analytic.is_empty() {
                ref_analytic[0].arg() as f64
            } else {
                0.0
            }
        });

        for sample in &ref_analytic {
            let phase = sample.arg();
            let mut diff = phase - prev_ref_phase;

            // Unwrap phase
            while diff > PI {
                diff -= 2.0 * PI;
            }
            while diff < -PI {
                diff += 2.0 * PI;
            }

            ref_phase_signal.push(diff);
            prev_ref_phase = phase;
        }

        self.last_ref_phase = Some(prev_ref_phase);

        // Remove DC from reference signal
        if ref_phase_signal.is_empty() {
            return (var_30, Vec::new());
        }

        let ref_mean = ref_phase_signal.iter().sum::<f64>() / ref_phase_signal.len() as f64;
        let ref_centered: Vec<f64> = ref_phase_signal.iter().map(|x| x - ref_mean).collect();
        let ref_30 = self.ref_30_lpf.filter(&ref_centered);

        (var_30, ref_30)
    }

    /// Demodulate pre-decimated audio to extract VOR 30 Hz components.
    ///
    /// This method is used when audio is already available (e.g., from a WAV file)
    /// and doesn't need frequency shifting or the full I/Q demodulation pipeline.
    ///
    /// # Arguments
    ///
    /// - `audio_samples`: Audio samples at the configured audio rate
    ///
    /// # Returns
    ///
    /// A tuple of:
    /// - `variable_30hz`: Envelope of the 30 Hz variable signal
    /// - `reference_30hz`: Phase signal from the 30 Hz reference
    /// - `audio_out`: The input audio (returned unchanged for pipeline compatibility)
    ///
    /// All outputs are at the audio rate. Returns empty vectors if input is empty.
    pub fn demodulate_audio(&mut self, audio_samples: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if audio_samples.is_empty() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        let (var_30, ref_30) = self.extract_subcarriers_and_30hz(audio_samples);
        (var_30, ref_30, audio_samples.to_vec())
    }
}

fn decimate(input: &[f64], factor: usize) -> Vec<f64> {
    if factor <= 1 {
        return input.to_vec();
    }
    input.iter().step_by(factor).copied().collect()
}

/// Calculate VOR radial from phase difference using FFT
pub fn calculate_radial(var_30: &[f64], ref_30: &[f64], sample_rate: f64) -> Option<f64> {
    let n = var_30.len().min(ref_30.len());
    let min_samples = (sample_rate * VOR_MIN_SAMPLES_FOR_RADIAL) as usize;
    if n < min_samples {
        return None;
    }

    let var_normalized = normalize_signal(&var_30[..n]);
    let ref_normalized = normalize_signal(&ref_30[..n]);

    let phi_var = estimate_phase_at_freq(&var_normalized, sample_rate, 30.0);
    let phi_ref = estimate_phase_at_freq(&ref_normalized, sample_rate, 30.0);
    Some(((phi_var - phi_ref).to_degrees() + 360.0) % 360.0)
}

pub fn calculate_radial_vortrack(signal: &[f64], sample_rate: f64) -> Option<f64> {
    if signal.len() < sample_rate as usize {
        return None;
    }

    let mut flt_r = VortrackFilter::default();
    let mut flt_s = VortrackFilter::default();
    let mut flt_f = VortrackFilter::default();

    let mut phase = 0.0f64;
    let mut sum = 0.0f64;
    let mut uw = 0.0f64;
    let mut prev_a = 0.0f64;
    let mut prev_fmcar = Complex::new(0.0, 0.0);
    let mut n: isize = -((sample_rate / 10.0) as isize);

    let w30 = 2.0 * PI * VOR_REFERENCE_FREQ / sample_rate;
    let f_limit = 2.0 * PI * 510.0 / sample_rate;
    let phase_bias = 26.0 * 2.0 * PI * VOR_REFERENCE_FREQ / sample_rate;

    for &s in signal {
        phase += w30;
        if phase > PI {
            phase -= 2.0 * PI;
        }

        let ref30 = flt_r.filterlow(Complex::from_polar(s, -phase));

        let phase9960 = -(VOR_REFERENCE_SUBCARRIER_FREQ / VOR_REFERENCE_FREQ) * phase;
        let fmcar = flt_f.filter510(Complex::from_polar(s, phase9960));

        let mut f = if prev_fmcar.norm_sqr() > FFT_BIN_MIN {
            (fmcar * prev_fmcar.conj()).arg()
        } else {
            0.0
        };
        prev_fmcar = fmcar;

        if f > f_limit {
            f = f_limit;
        }
        if f < -f_limit {
            f = -f_limit;
        }

        let sig30 = flt_s.filterlow(Complex::from_polar(f, -phase));

        let a = (sig30 * ref30.conj()).arg() + phase_bias;

        if n > 0 {
            if (a - prev_a) > PI {
                uw -= 2.0 * PI;
            }
            if (a - prev_a) < -PI {
                uw += 2.0 * PI;
            }
            sum += a + uw;
        }
        prev_a = a;
        n += 1;
    }

    if n <= 0 {
        return None;
    }

    let mut avg = (180.0 / PI) * (sum / n as f64);
    avg = avg.rem_euclid(360.0);
    Some(avg)
}

fn normalize_signal(signal: &[f64]) -> Vec<f64> {
    let mean = signal.iter().sum::<f64>() / signal.len() as f64;
    let centered: Vec<f64> = signal.iter().map(|x| x - mean).collect();

    let rms = (centered.iter().map(|x| x * x).sum::<f64>() / centered.len() as f64).sqrt();

    if rms > VOR_RMS_MIN {
        centered.iter().map(|x| x / rms).collect()
    } else {
        centered
    }
}

fn estimate_phase_at_freq(signal: &[f64], sample_rate: f64, target_freq: f64) -> f64 {
    let n = signal.len();
    if n < 2 {
        return 0.0;
    }

    let mut real_sum = 0.0;
    let mut imag_sum = 0.0;

    for (i, &x) in signal.iter().enumerate() {
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
        let phase = 2.0 * PI * target_freq * i as f64 / sample_rate;
        let xw = x * w;
        real_sum += xw * phase.cos();
        imag_sum -= xw * phase.sin();
    }

    imag_sum.atan2(real_sum)
}

#[derive(Debug, Clone)]
struct VortrackFilter {
    xv: [Complex<f64>; 8],
    yv: [Complex<f64>; 8],
}

impl Default for VortrackFilter {
    fn default() -> Self {
        Self {
            xv: [Complex::new(0.0, 0.0); 8],
            yv: [Complex::new(0.0, 0.0); 8],
        }
    }
}

impl VortrackFilter {
    fn shift(&mut self, v: Complex<f64>) {
        self.xv[0] = self.xv[1];
        self.xv[1] = self.xv[2];
        self.xv[2] = self.xv[3];
        self.xv[3] = self.xv[4];
        self.xv[4] = v;

        self.yv[0] = self.yv[1];
        self.yv[1] = self.yv[2];
        self.yv[2] = self.yv[3];
        self.yv[3] = self.yv[4];
    }

    fn filter510(&mut self, v: Complex<f64>) -> Complex<f64> {
        self.shift(v);
        self.yv[4] = (self.xv[0] + self.xv[4])
            + 1.372_412_796_2 * (self.xv[1] + self.xv[3])
            + 0.744_825_592_5 * self.xv[2]
            + (-0.913_351_229_9 * self.yv[2])
            + (1.909_423_187_8 * self.yv[3]);
        self.yv[4]
    }

    fn filterlow(&mut self, v: Complex<f64>) -> Complex<f64> {
        self.shift(v);
        self.yv[4] =
            (self.xv[0] + self.xv[4]) + 0.000_014_212_2 * self.xv[1] + 0.000_014_212_2 * self.xv[3]
                - 1.999_971_575_6 * self.xv[2]
                + (-0.997_235_202_6 * self.yv[2])
                + (1.997_232_651_1 * self.yv[3]);
        self.yv[4]
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VorRadial {
    pub timestamp: f64,
    pub radial_deg: f64,
    pub frequency_mhz: f64,
    pub signal_quality: Option<SignalQualityMetrics>,
    pub ident: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub morse_debug: Option<MorseDebugInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalQualityMetrics {
    pub clipping_ratio: String,
    pub snr_30hz_db: f64,
    pub snr_9960hz_db: f64,
    pub lock_quality: String,
    pub radial_variance: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MorseDebugInfo {
    pub candidates: Vec<MorseCandidate>,
    pub total_tokens: usize,
    pub windows_total: usize,
    pub ident_hits_seconds: Vec<f64>,
    pub repeat_interval_seconds: Option<f64>,
    pub next_expected_seconds: Option<f64>,
    pub decode_attempts: Vec<morse::MorseDecodeAttempt>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MorseCandidate {
    pub token: String,
    pub count: usize,
    pub confidence: f64,
}

impl VorRadial {
    pub fn new(timestamp: f64, radial_deg: f64, frequency_mhz: f64) -> Self {
        Self {
            timestamp,
            radial_deg,
            frequency_mhz,
            signal_quality: None,
            ident: None,
            morse_debug: None,
        }
    }

    pub fn with_quality(mut self, quality: SignalQualityMetrics) -> Self {
        self.signal_quality = Some(quality);
        self
    }

    pub fn with_ident(mut self, ident: Option<String>) -> Self {
        self.ident = ident;
        self
    }

    pub fn with_morse_debug(mut self, debug: Option<MorseDebugInfo>) -> Self {
        self.morse_debug = debug;
        self
    }
}

/// VOR-specific Morse ident decoding wrapper.
///
/// Applies VOR-specific signal processing (bandpass 900-1100 Hz for VOR ident)
/// then delegates generic Morse parsing to [`morse::decode_morse_ident`].
pub fn decode_morse_ident(
    audio: &[f64],
    sample_rate: f64,
) -> (Option<String>, Vec<String>, Vec<morse::MorseDecodeAttempt>) {
    if audio.len() < (sample_rate as usize / 2) {
        return (None, Vec::new(), Vec::new());
    }

    // VOR-specific processing: bandpass + Hilbert envelope + lowpass
    // Python-proven path: bandpass + Hilbert envelope + aggressive lowpass around 1020 Hz
    // Use FIR bandpass (reliable) + IIR lowpass with zero-phase filtering for envelope
    let mut bpf = ButterworthFilter::bandpass(
        VOR_MORSE_BPF_LOW,
        VOR_MORSE_BPF_HIGH,
        sample_rate,
        VOR_MORSE_BPF_ORDER,
    );
    let tone = bpf.filter(audio);
    let analytic = hilbert_transform(&tone);
    let env_raw = envelope(&analytic);

    // Use proper IIR Butterworth lowpass with zero-phase filtering (filtfilt) via biquad crate
    let env = filtfilt_lowpass(
        &env_raw,
        VOR_MORSE_ENV_LPF_CUTOFF,
        sample_rate,
        VOR_MORSE_ENV_LPF_ORDER,
    );

    // Delegate to generic Morse decoding
    morse::decode_morse_ident(&env, sample_rate)
}

// ──────────────────────────────────────────────────────────────────────────────
// VOR Window-based Processor
// ──────────────────────────────────────────────────────────────────────────────

/// Stateful processor for window-based VOR analysis.
///
/// This struct encapsulates the common processing logic shared between I/Q and WAV VOR sources:
/// - Audio windowing and accumulation
/// - Radial calculation at window boundaries
/// - Morse ident detection and voting
/// - Signal quality metrics
///
/// Both I/Q (streaming) and WAV (batch) paths use this processor to ensure identical
/// windowing, voting, and output generation logic.
pub struct VorProcessor {
    // Configuration
    window_samples: usize,
    morse_window_samples: usize,
    debug_morse: bool,

    // Sliding windows for analysis
    audio_buffer: Vec<f64>,
    var_30_buffer: Vec<f64>,
    ref_30_buffer: Vec<f64>,
    morse_audio_buffer: Vec<f64>,

    // Ident tracking state
    ident_votes: std::collections::HashMap<String, usize>,
    ident_hit_timestamps: std::collections::HashMap<String, Vec<f64>>,
    all_tokens_history: Vec<String>,

    // Radial tracking
    radial_history: Vec<f64>,
    windows_total: usize,
}

/// Output from [`VorProcessor::emit()`] containing all data needed to construct a [`VorRadial`].
pub struct VorProcessorOutput {
    pub radial: f64,
    pub signal_quality: SignalQualityMetrics,
    pub ident: Option<String>,
    pub morse_debug: Option<MorseDebugInfo>,
}

impl VorProcessor {
    /// Create a new processor with given window sizes.
    ///
    /// # Arguments
    ///
    /// - `window_seconds`: Window duration for radial calculation (typically 3.0 s)
    /// - `morse_window_seconds`: Window duration for Morse ident accumulation (typically 15.0 s)
    /// - `audio_rate`: Sample rate of audio/var_30/ref_30 signals (Hz)
    /// - `debug_morse`: Whether to include Morse candidate list in output
    pub fn new(
        window_seconds: f64,
        morse_window_seconds: f64,
        audio_rate: f64,
        debug_morse: bool,
    ) -> Self {
        let window_samples = (window_seconds * audio_rate) as usize;
        let morse_window_samples = (morse_window_seconds * audio_rate) as usize;

        Self {
            window_samples,
            morse_window_samples,
            debug_morse,
            audio_buffer: Vec::new(),
            var_30_buffer: Vec::new(),
            ref_30_buffer: Vec::new(),
            morse_audio_buffer: Vec::new(),
            ident_votes: std::collections::HashMap::new(),
            ident_hit_timestamps: std::collections::HashMap::new(),
            all_tokens_history: Vec::new(),
            radial_history: Vec::new(),
            windows_total: 0,
        }
    }

    /// Accumulate audio and 30 Hz signals into the processor.
    ///
    /// Returns `true` if a radial window boundary has been reached and [`emit()`] should be called.
    ///
    /// [`emit()`]: Self::emit
    pub fn accumulate(&mut self, audio: &[f64], var_30: &[f64], ref_30: &[f64]) -> bool {
        self.audio_buffer.extend_from_slice(audio);
        self.var_30_buffer.extend_from_slice(var_30);
        self.ref_30_buffer.extend_from_slice(ref_30);
        self.morse_audio_buffer.extend_from_slice(audio);

        self.audio_buffer.len() >= self.window_samples
    }

    /// Emit a radial measurement if enough data has accumulated.
    ///
    /// Call this after [`accumulate()`] returns `true`. Returns `Some` with complete output data,
    /// or `None` if processing should continue.
    ///
    /// # Arguments
    ///
    /// - `timestamp`: Unix timestamp (seconds) for this measurement
    /// - `clipping_ratio`: Optional clipping ratio from I/Q signal (I/Q sources only)
    /// - `sample_count`: Total samples processed so far (for I/Q sources)
    /// - `audio_rate`: Sample rate of audio signals
    ///
    /// [`accumulate()`]: Self::accumulate
    pub fn emit(
        &mut self,
        timestamp: f64,
        clipping_ratio: Option<f64>,
        sample_count: usize,
        audio_rate: f64,
    ) -> Option<VorProcessorOutput> {
        if self.audio_buffer.len() < self.window_samples {
            return None;
        }

        // Calculate radial from current window
        let radial = calculate_radial_vortrack(&self.audio_buffer, audio_rate)?;
        self.radial_history.push(radial);
        if self.radial_history.len() > 20 {
            self.radial_history.remove(0);
        }

        self.windows_total += 1;
        let mut ident_detected_now: Option<String> = None;

        // Try Morse decoding if we have enough accumulated audio
        let morse_debug = if self.morse_audio_buffer.len() >= self.morse_window_samples {
            let (candidate, tokens, attempts) =
                decode_morse_ident(&self.morse_audio_buffer, audio_rate);
            self.all_tokens_history.extend(tokens);

            if let Some(ref id) = candidate {
                *self.ident_votes.entry(id.clone()).or_insert(0) += 1;
                let hits = self.ident_hit_timestamps.entry(id.clone()).or_default();
                let is_new_cycle = hits
                    .last()
                    .map(|last| timestamp - *last >= 7.0)
                    .unwrap_or(true);
                if is_new_cycle {
                    hits.push(timestamp);
                    ident_detected_now = Some(id.clone());
                }
            }

            // Keep only recent audio for Morse (sliding window)
            let keep_samples = (self.morse_window_samples as f64 * 0.5) as usize;
            if self.morse_audio_buffer.len() > self.morse_window_samples + keep_samples {
                self.morse_audio_buffer.drain(0..keep_samples);
            }

            // Build debug info if requested
            if self.debug_morse {
                let mut counts: std::collections::HashMap<String, usize> =
                    std::collections::HashMap::new();
                for token in &self.all_tokens_history {
                    let t = token.to_uppercase();
                    if t.len() == 3 {
                        *counts.entry(t).or_insert(0) += 1;
                    }
                }

                let total_count: usize = counts.values().sum();
                let mut candidates: Vec<MorseCandidate> = counts
                    .into_iter()
                    .map(|(token, count)| MorseCandidate {
                        token,
                        count,
                        confidence: if total_count > 0 {
                            count as f64 / total_count as f64
                        } else {
                            0.0
                        },
                    })
                    .collect();

                candidates
                    .sort_by(|a, b| b.count.cmp(&a.count).then_with(|| a.token.cmp(&b.token)));

                // Get timestamp hits for top ident candidate
                let top_ident = candidates.first().map(|c| c.token.clone());
                let ident_hits_seconds = top_ident
                    .and_then(|id| self.ident_hit_timestamps.get(&id).cloned())
                    .unwrap_or_default();

                // Estimate repeat interval and next expected time
                let (repeat_interval_seconds, next_expected_seconds) =
                    if ident_hits_seconds.len() >= 2 {
                        let intervals: Vec<f64> =
                            ident_hits_seconds.windows(2).map(|w| w[1] - w[0]).collect();
                        let avg_interval = intervals.iter().sum::<f64>() / intervals.len() as f64;
                        let next_expected = ident_hits_seconds.last().map(|&t| t + avg_interval);
                        (Some(avg_interval), next_expected)
                    } else {
                        (None, None)
                    };

                Some(MorseDebugInfo {
                    candidates,
                    total_tokens: self.all_tokens_history.len(),
                    windows_total: self.windows_total,
                    ident_hits_seconds,
                    repeat_interval_seconds,
                    next_expected_seconds,
                    decode_attempts: attempts,
                })
            } else {
                None
            }
        } else {
            None
        };

        // Calculate signal quality
        let signal_quality = metrics::compute_signal_quality(
            clipping_ratio,
            &self.audio_buffer,
            &self.var_30_buffer,
            &self.ref_30_buffer,
            audio_rate,
            sample_count,
            &self.radial_history,
        );

        // Clear buffers for next window
        self.audio_buffer.clear();
        self.var_30_buffer.clear();
        self.ref_30_buffer.clear();

        Some(VorProcessorOutput {
            radial,
            signal_quality,
            ident: ident_detected_now,
            morse_debug,
        })
    }
}
