//! VOR (VHF Omnidirectional Range) decoder.
//!
//! VOR (VHF Omnidirectional Range) demodulation and radial calculation.
//! - Signal demodulation (frequency shift, filtering, subcarrier extraction)
//! - 30 Hz reference and variable signal extraction
//! - Radial calculation (bearing to station)
//!
//! Morse ident decoding is handled by the generic [`super::morse`] module.
//!
//! VOR stations transmit on 108-117.95 MHz with:
//! - Carrier with voice/ident modulation
//! - 30 Hz reference signal (FM modulated on 9960 Hz subcarrier)
//! - 30 Hz variable signal (AM modulated, rotates mechanically or electronically)
//!
//! The bearing (radial) to the station is determined by the phase difference
//! between the 30 Hz variable and reference signals.

use num_complex::Complex;
use rustfft::FftPlanner;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

use super::morse;
use desperado::dsp::filters::ButterworthFilter;
use desperado::dsp::iir::filtfilt_lowpass;

pub const VOR_SAMPLE_RATE_1_8M: u32 = 1_800_000;

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
        let audio_target = 48_000.0;
        let decim_factor = ((sample_rate_f / audio_target).round() as usize).max(1);
        let audio_rate = sample_rate_f / decim_factor as f64;

        Self {
            sample_rate: sample_rate_f,
            audio_rate,
            decim_factor,

            // Baseband filter (200 kHz)
            baseband_lpf: ButterworthFilter::lowpass(200_000.0, sample_rate_f, 5),

            // Audio filter (20 kHz)
            audio_lpf: ButterworthFilter::lowpass(20_000.0, sample_rate_f, 5),

            // Variable subcarrier
            var_sub_bpf: ButterworthFilter::bandpass(9_000.0, 11_000.0, audio_rate, 4),

            // Reference subcarrier (9.5-10.5 kHz)
            ref_sub_bpf: ButterworthFilter::bandpass(9_500.0, 10_500.0, audio_rate, 4),

            // 30 Hz lowpass filters - wider for better capture
            var_30_lpf: ButterworthFilter::lowpass(100.0, audio_rate, 5),
            ref_30_lpf: ButterworthFilter::lowpass(100.0, audio_rate, 5),

            last_ref_phase: None,
        }
    }

    pub fn audio_rate(&self) -> f64 {
        self.audio_rate
    }

    /// Demodulate VOR signal and extract 30 Hz components
    /// Returns (variable_30hz, reference_30hz, audio_samples)
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

        // 6. Extract subcarriers
        let audio_f64: Vec<f64> = audio_decimated.to_vec();

        let var_sub = self.var_sub_bpf.filter(&audio_f64);
        let ref_sub = self.ref_sub_bpf.filter(&audio_f64);

        // 7. Extract 30 Hz components
        // Variable: AM demodulation (envelope)
        let var_analytic = hilbert_transform(&var_sub);
        let var_envelope = envelope(&var_analytic);

        // Remove DC from variable signal
        if var_envelope.is_empty() || ref_sub.is_empty() {
            return (Vec::new(), Vec::new(), audio_decimated);
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
            return (Vec::new(), Vec::new(), audio_decimated);
        }

        let ref_mean = ref_phase_signal.iter().sum::<f64>() / ref_phase_signal.len() as f64;
        let ref_centered: Vec<f64> = ref_phase_signal.iter().map(|x| x - ref_mean).collect();
        let ref_30 = self.ref_30_lpf.filter(&ref_centered);

        (var_30, ref_30, audio_decimated)
    }
}

fn decimate(input: &[f64], factor: usize) -> Vec<f64> {
    if factor <= 1 {
        return input.to_vec();
    }
    input.iter().step_by(factor).copied().collect()
}

fn hilbert_transform(signal: &[f64]) -> Vec<Complex<f64>> {
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

fn envelope(signal: &[Complex<f64>]) -> Vec<f64> {
    signal.iter().map(|c| c.norm()).collect()
}

/// Calculate VOR radial from phase difference using FFT
pub fn calculate_radial(var_30: &[f64], ref_30: &[f64], sample_rate: f64) -> Option<f64> {
    let n = var_30.len().min(ref_30.len());
    let min_samples = (sample_rate * 1.0) as usize;
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

    let w30 = 2.0 * PI * 30.0 / sample_rate;
    let f_limit = 2.0 * PI * 510.0 / sample_rate;
    let phase_bias = 26.0 * 2.0 * PI * 30.0 / sample_rate;

    for &s in signal {
        phase += w30;
        if phase > PI {
            phase -= 2.0 * PI;
        }

        let ref30 = flt_r.filterlow(Complex::from_polar(s, -phase));

        let phase9960 = -(9960.0 / 30.0) * phase;
        let fmcar = flt_f.filter510(Complex::from_polar(s, phase9960));

        let mut f = if prev_fmcar.norm_sqr() > 1e-12 {
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

    if rms > 1e-10 {
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
    let mut bpf = ButterworthFilter::bandpass(900.0, 1100.0, sample_rate, 4);
    let tone = bpf.filter(audio);
    let analytic = hilbert_transform(&tone);
    let env_raw = envelope(&analytic);

    // Use proper IIR Butterworth lowpass with zero-phase filtering (filtfilt) via biquad crate
    let env = filtfilt_lowpass(&env_raw, 40.0, sample_rate, 4);

    // Delegate to generic Morse decoding
    morse::decode_morse_ident(&env, sample_rate)
}
