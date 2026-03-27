//! ILS (Instrument Landing System) localizer decoder.
//!
//! The ILS localizer provides lateral (left-right) guidance for precision aircraft landings.
//! This module demodulates ILS localizer signals to extract:
//! - **90 Hz tone**: Amplitude modulation deeper on left side of centreline
//! - **150 Hz tone**: Amplitude modulation deeper on right side of centreline
//! - **DDM (Difference in Depth of Modulation)**: Lateral deviation indicator (−1.0 to +1.0)
//! - **Morse ident**: 4-letter station identifier on 1020 Hz tone
//!
//! ## Demodulation Process
//!
//! The [`IlsDemodulator`] processes raw I/Q samples or WAV audio through these steps:
//!
//! 1. **Frequency shifting** (I/Q path): Translates RF carrier to baseband
//! 2. **Baseband filtering**: 500 Hz lowpass removes RF noise before decimation
//! 3. **AM envelope detection**: Extracts baseband signal via magnitude (|I² + Q²|)
//! 4. **Decimation**: Reduces from 1.8 MSps to 9 kHz (decimation factor = 200)
//! 5. **Dual bandpass filtering**:
//!    - 90 Hz tone: 80–100 Hz bandpass filter
//!    - 150 Hz tone: 140–160 Hz bandpass filter
//! 6. **Envelope extraction**: Hilbert transform → magnitude → envelope
//!
//! ## DDM Calculation
//!
//! The [`compute_ddm`] function computes:
//!
//! ```text
//! DDM = (depth_90 - depth_150) / (depth_90 + depth_150)
//! ```
//!
//! Where `depth_90` and `depth_150` are the mean envelope amplitudes of the respective tones.
//!
//! **Interpretation:**
//! - **DDM > +0.015**: 90 Hz dominant → left of centreline → fly right
//! - **|DDM| ≤ 0.015**: on centreline → maintain
//! - **DDM < −0.015**: 150 Hz dominant → right of centreline → fly left
//!
//! The threshold of 0.015 is the FAA standard for "on-course" guidance.
//!
//! ## Filter Design
//!
//! All filter parameters (cutoff frequencies, orders, thresholds) are centralized in
//! [`filter_config`](crate::filter_config). Key design choices:
//! - **Baseband LPF (500 Hz)**: Preserves both 90 Hz and 150 Hz tones while rejecting RF noise
//! - **90/150 Hz BPF (order 4)**: Narrow passband (20 Hz wide) for tone isolation
//! - **Morse envelope LPF (16 Hz stated, ~8 Hz effective)**: Suppresses ~36 Hz Hilbert ripple
//!   while resolving 100 ms dots at 9 kHz audio rate
//! - **DDM sum threshold (1e-9)**: Prevents division by near-zero when both tones are at noise floor
//!
//! ## WAV File Processing
//!
//! For pre-recorded audio files, use [`IlsDemodulator::new_at_audio_rate`] to skip the I/Q
//! decimation path. For Morse ident extraction over long files, use
//! [`IlsDemodulator::precompute_morse_envelope`] to apply zero-phase filtering across the
//! entire file at once, avoiding edge artifacts from rolling windowed filtering.
//!
//! Morse ident decoding is handled by the generic [`super::morse`] module.

use num_complex::Complex;
use serde::{Deserialize, Serialize};

// ── ILS Filter & Processing Constants ────────────────────────────────────────

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

/// ILS Morse audio output bandpass filter low cutoff (Hz).
///
/// Matches analysis filter band (980-1060 Hz) for consistent audio output.
/// Uses stateful FIR implementation for streaming audio compatibility.
pub const ILS_MORSE_AUDIO_BPF_LOW: f64 = 980.0;

/// ILS Morse audio output bandpass filter high cutoff (Hz).
pub const ILS_MORSE_AUDIO_BPF_HIGH: f64 = 1_060.0;

/// ILS Morse audio output bandpass filter order.
///
/// Order 4 (65 taps FIR) provides smooth passband for audio output.
/// Applied to audio output only, not to signal measurements.
pub const ILS_MORSE_AUDIO_BPF_ORDER: usize = 4;

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

/// ILS baseband downsampling target audio rate (Hz).
///
/// Decimation factor from I/Q rate is calculated as
/// ceil(input_rate / this_value).
pub const ILS_DECIMATION_TARGET_RATE: f64 = 9_000.0;

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
use std::f64::consts::PI;

use super::error::{Error, IlsDdmResult, Result};
use super::morse;
use desperado::dsp::filters::ButterworthFilter;
use desperado::dsp::iir::{filtfilt_bandpass, filtfilt_lowpass};
use desperado::dsp::voracious::{envelope, hilbert_transform};

/// Sample rate constants for ILS decoding
pub const ILS_SAMPLE_RATE_1_8M: u32 = 1_800_000;
pub const ILS_AUDIO_RATE: f64 = 9_000.0;

/// ILS side-of-centreline indication derived from DDM.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum IlsSide {
    /// DDM > +0.015: 90 Hz dominant, aircraft is left of centreline → fly right
    Left,
    /// |DDM| ≤ 0.015: on centreline
    OnCourse,
    /// DDM < −0.015: 150 Hz dominant, aircraft is right of centreline → fly left
    Right,
}

impl IlsSide {
    /// Classify lateral position from a DDM value.
    ///
    /// - `|ddm| ≤ 0.015` → [`OnCourse`](Self::OnCourse)
    /// - `ddm > 0.015`  → [`Left`](Self::Left) (90 Hz dominant, fly right)
    /// - `ddm < −0.015` → [`Right`](Self::Right) (150 Hz dominant, fly left)
    pub fn from_ddm(ddm: f64) -> Self {
        if ddm > ILS_DDM_ON_COURSE_THRESHOLD {
            IlsSide::Left
        } else if ddm < -ILS_DDM_ON_COURSE_THRESHOLD {
            IlsSide::Right
        } else {
            IlsSide::OnCourse
        }
    }
}

/// One decoded ILS localizer measurement covering approximately one second of audio.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IlsFrame {
    /// Unix timestamp (seconds) of the end of this measurement window.
    pub timestamp: f64,
    /// Localizer centre frequency in MHz (derived from carrier offset + centre freq).
    pub frequency_mhz: f64,
    /// Difference in Depth of Modulation: (env90 − env150) / (env90 + env150).
    /// Bounded −1.0..+1.0; positive = left of centreline.
    pub ddm: f64,
    /// 90 Hz modulation depth as a percentage of total signal (0–100).
    pub mod_90_pct: f64,
    /// 150 Hz modulation depth as a percentage of total signal (0–100).
    pub mod_150_pct: f64,
    /// Mean signal strength (normalised 0–1), derived from the AM envelope.
    pub signal_strength: f64,
    /// Decoded Morse ident if present (e.g. `"IKLO"`).
    pub ident: Option<String>,
    /// Optional debug details about Morse decoding attempts and candidates.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub morse_debug: Option<IlsMorseDebugInfo>,
    /// Lateral position interpretation derived from DDM.
    pub side: IlsSide,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IlsMorseDebugInfo {
    pub candidates: Vec<IlsMorseCandidate>,
    pub total_tokens: usize,
    pub decode_attempts: Vec<morse::MorseDecodeAttempt>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IlsMorseCandidate {
    pub token: String,
    pub count: usize,
    pub confidence: f64,
}

/// ILS localizer demodulator.
///
/// Processes raw cf32 I/Q samples at 1.8 MSps and extracts 90 Hz / 150 Hz
/// tone envelopes needed to compute DDM.
pub struct IlsLocalizerDemodulator {
    /// Input I/Q sample rate in Hz.
    pub sample_rate: f64,
    /// Decimated audio sample rate in Hz (= sample_rate / decim_factor).
    pub audio_rate: f64,
    decim_factor: usize,

    // Filters applied at the full I/Q sample rate
    baseband_lpf: ButterworthFilter,

    // Filters applied at the decimated audio rate
    bpf_90: ButterworthFilter,
    bpf_150: ButterworthFilter,
}

impl IlsLocalizerDemodulator {
    /// Create a new demodulator for the given I/Q sample rate.
    ///
    /// With the default 1.8 MSps input the decimation factor is 200,
    /// yielding a 9 kHz audio rate.
    pub fn new(sample_rate: u32) -> Self {
        let sample_rate_f = sample_rate as f64;
        // Keep the audio rate at exactly 9 kHz regardless of rounding
        let decim_factor = ((sample_rate_f / ILS_AUDIO_RATE).round() as usize).max(1);
        let audio_rate = sample_rate_f / decim_factor as f64;

        Self {
            sample_rate: sample_rate_f,
            audio_rate,
            decim_factor,

            baseband_lpf: ButterworthFilter::lowpass(
                ILS_BASEBAND_LPF_CUTOFF,
                sample_rate_f,
                ILS_BASEBAND_LPF_ORDER,
            ),

            bpf_90: ButterworthFilter::bandpass(
                ILS_90HZ_BPF_LOW,
                ILS_90HZ_BPF_HIGH,
                audio_rate,
                ILS_90HZ_BPF_ORDER,
            ),
            bpf_150: ButterworthFilter::bandpass(
                ILS_150HZ_BPF_LOW,
                ILS_150HZ_BPF_HIGH,
                audio_rate,
                ILS_150HZ_BPF_ORDER,
            ),
            // Note: 900–1100 Hz Morse ident filtering is done with filtfilt_bandpass
            // in decode_ident(), not with a stateful BPF here.
        }
    }

    /// Create a demodulator that operates directly on audio-rate samples (e.g. from a WAV file).
    ///
    /// Bypasses the I/Q decimation path: all filters are built at `audio_rate` directly,
    /// and the pre-decimation lowpass is a trivial placeholder.
    pub fn new_at_audio_rate(audio_rate: f64) -> Self {
        Self {
            sample_rate: audio_rate,
            audio_rate,
            decim_factor: 1,
            // Not used in the WAV path, but must be initialised
            baseband_lpf: ButterworthFilter::lowpass(audio_rate / 2.0 * 0.9, audio_rate, 1),
            bpf_90: ButterworthFilter::bandpass(
                ILS_90HZ_BPF_LOW,
                ILS_90HZ_BPF_HIGH,
                audio_rate,
                ILS_90HZ_BPF_ORDER,
            ),
            bpf_150: ButterworthFilter::bandpass(
                ILS_150HZ_BPF_LOW,
                ILS_150HZ_BPF_HIGH,
                audio_rate,
                ILS_150HZ_BPF_ORDER,
            ),
        }
    }

    /// Expose the audio (post-decimation) sample rate.
    pub fn audio_rate(&self) -> f64 {
        self.audio_rate
    }

    /// Process pre-demodulated audio samples (e.g. from a WAV file) and return the
    /// 90 Hz and 150 Hz tone envelopes, bypassing the I/Q decimation path.
    ///
    /// The input is the raw signed AM audio (bipolar signal).
    /// Returns `(env_90, env_150)`.
    pub fn filter_audio_envelopes(&mut self, audio: &[f64]) -> (Vec<f64>, Vec<f64>) {
        // Use IIR filtfilt bandpass instead of the broken ButterworthFilter::bandpass.
        //
        // The FIR bandpass (ButterworthFilter::bandpass) has only ~65 taps at 48 kHz;
        // it subtracts two independently-normalised lowpass filters, producing near-zero
        // response (~0.0001 instead of 0.04–0.10) due to normalization mismatch.
        // The IIR filtfilt_bandpass correctly extracts the 90/150 Hz tones.
        let tone_90 = filtfilt_bandpass(
            audio,
            ILS_90HZ_BPF_LOW,
            ILS_90HZ_BPF_HIGH,
            self.audio_rate,
            ILS_90HZ_BPF_ORDER,
        );
        let analytic_90 = hilbert_transform(&tone_90);
        let env_90 = envelope(&analytic_90);

        let tone_150 = filtfilt_bandpass(
            audio,
            ILS_150HZ_BPF_LOW,
            ILS_150HZ_BPF_HIGH,
            self.audio_rate,
            ILS_150HZ_BPF_ORDER,
        );
        let analytic_150 = hilbert_transform(&tone_150);
        let env_150 = envelope(&analytic_150);

        (env_90, env_150)
    }

    /// Decode Morse ident from pre-demodulated audio samples (WAV path).
    pub fn decode_ident_audio(
        &mut self,
        audio: &[f64],
    ) -> (Option<String>, Vec<String>, Vec<morse::MorseDecodeAttempt>) {
        self.decode_ident(audio)
    }

    /// Pre-compute the full 1020 Hz Morse envelope for an entire WAV file.
    ///
    /// **Note**: This method is deprecated. For WAV file processing, prefer using
    /// the `sources::wav` module which handles carrier removal, timestamp parsing,
    /// and uses the shared DSP pipeline from the `sources::common` module.
    ///
    /// Because the Morse envelope lowpass (`filtfilt`) needs context on both
    /// sides of each sample to work correctly (zero-phase filtering), processing
    /// the entire file at once produces far better results than filtering
    /// successive 15-second chunks.  Call this once during source initialisation
    /// and slice the returned vector per-window during decoding.
    ///
    /// Returns a vector of the same length as `audio` containing the smoothed
    /// 1020 Hz tone envelope at the WAV sample rate.
    #[allow(dead_code)]
    pub fn precompute_morse_envelope(&mut self, audio: &[f64]) -> Vec<f64> {
        // Use IIR filtfilt bandpass instead of the FIR bpf_ident.
        //
        // The FIR bandpass (ButterworthFilter::bandpass) has only ~65 taps at
        // 48 kHz; its transition bands are far too wide to isolate 1020 Hz, and
        // the passband output is dominated by out-of-band noise.  The IIR
        // filtfilt_bandpass has a Q = 1000/200 = 5, giving a sharp, narrow band
        // centred on 1000 Hz that correctly extracts the 1020 Hz ident tone even
        // at modest SNR.
        let tone = filtfilt_bandpass(
            audio,
            ILS_MORSE_BPF_LOW,
            ILS_MORSE_BPF_HIGH,
            self.audio_rate,
            ILS_MORSE_BPF_ORDER,
        );
        let analytic = hilbert_transform(&tone);
        let env_raw = envelope(&analytic);
        // The Morse envelope lowpass with stated 16 Hz cutoff.
        // The cascaded-biquad filtfilt in this codebase with order=4 has an
        // effective -3 dB at approximately 0.5 × stated cutoff (~8 Hz effective).
        // This is aggressive enough to suppress the ~36 Hz Hilbert envelope ripple
        // while still resolving individual 100 ms dots and 120 ms inter-element gaps.
        // Processing the full file at once (rather than rolling chunks) avoids filtfilt edge artefacts.
        filtfilt_lowpass(
            &env_raw,
            ILS_MORSE_ENV_LPF_CUTOFF,
            self.audio_rate,
            ILS_MORSE_ENV_LPF_ORDER,
        )
    }

    /// Demodulate a chunk of I/Q samples.
    ///
    /// Performs the full demodulation pipeline on I/Q samples:
    /// frequency shift → baseband filtering → AM demodulation →
    /// decimation → dual bandpass filtering → envelope extraction.
    ///
    /// # Arguments
    ///
    /// - `iq_samples`: Raw I/Q samples at the configured input sample rate
    /// - `freq_offset`: Frequency shift to apply before baseband filtering (Hz)
    ///
    /// # Returns
    ///
    /// A tuple of three vectors, all at the audio rate (see [`audio_rate`](Self::audio_rate)):
    ///
    /// - `env_90`: Instantaneous envelope of the 90 Hz tone (Hilbert magnitude)
    /// - `env_150`: Instantaneous envelope of the 150 Hz tone (Hilbert magnitude)
    /// - `audio_envelope`: Full AM envelope at audio rate (used for signal strength + ident)
    ///
    /// Returns empty vectors if input is empty.
    pub fn demodulate(
        &mut self,
        iq_samples: &[Complex<f32>],
        freq_offset: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if iq_samples.is_empty() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        // 1. Frequency-shift the carrier to DC
        let iq_shifted: Vec<Complex<f32>> = iq_samples
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let t = i as f64 / self.sample_rate;
                let shift = Complex::new(
                    (-2.0 * PI * freq_offset * t).cos() as f32,
                    (-2.0 * PI * freq_offset * t).sin() as f32,
                );
                s * shift
            })
            .collect();

        // 2. Lowpass filter to remove off-channel energy before decimation
        let iq_filtered = self.baseband_lpf.filter_complex(&iq_shifted);

        // 3. AM envelope detection: |IQ|
        let am_envelope: Vec<f64> = iq_filtered.iter().map(|c| c.norm() as f64).collect();

        // 4. Decimate to audio rate
        let audio: Vec<f64> = am_envelope
            .iter()
            .step_by(self.decim_factor)
            .copied()
            .collect();

        if audio.is_empty() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        // 5 & 6. Extract 90 Hz and 150 Hz tone envelopes
        let (env_90, env_150) = self.extract_tones_and_audio(&audio);

        (env_90, env_150, audio)
    }

    /// Extract 90 Hz and 150 Hz tone envelopes from audio samples.
    ///
    /// This is the core tone extraction logic, factored out so it can be
    /// reused by both I/Q demodulation and direct audio processing paths.
    ///
    /// # Arguments
    ///
    /// - `audio`: Audio samples already at the target audio rate (e.g., 48 kHz)
    ///
    /// # Returns
    ///
    /// A tuple of:
    /// - `env_90`: Envelope of the 90 Hz modulation tone
    /// - `env_150`: Envelope of the 150 Hz modulation tone
    ///
    /// Both outputs are at the audio rate. Returns empty vectors if input is empty.
    pub fn extract_tones_and_audio(&mut self, audio: &[f64]) -> (Vec<f64>, Vec<f64>) {
        if audio.is_empty() {
            return (Vec::new(), Vec::new());
        }

        // Extract 90 Hz tone envelope
        let tone_90 = self.bpf_90.filter(audio);
        let analytic_90 = hilbert_transform(&tone_90);
        let env_90 = envelope(&analytic_90);

        // Extract 150 Hz tone envelope
        let tone_150 = self.bpf_150.filter(audio);
        let analytic_150 = hilbert_transform(&tone_150);
        let env_150 = envelope(&analytic_150);

        (env_90, env_150)
    }

    /// Demodulate pre-decimated audio to extract ILS 90/150 Hz tone envelopes.
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
    /// - `env_90`: Envelope of the 90 Hz modulation tone
    /// - `env_150`: Envelope of the 150 Hz modulation tone
    /// - `audio_out`: The input audio (returned unchanged for pipeline compatibility)
    ///
    /// All outputs are at the audio rate. Returns empty vectors if input is empty.
    pub fn demodulate_audio(&mut self, audio_samples: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if audio_samples.is_empty() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        let (env_90, env_150) = self.extract_tones_and_audio(audio_samples);
        (env_90, env_150, audio_samples.to_vec())
    }

    /// Decode the 1020 Hz Morse ident from audio-rate samples.
    ///
    /// Applies a zero-phase IIR 900–1100 Hz bandpass (`filtfilt_bandpass`),
    /// Hilbert envelope, and zero-phase lowpass, then delegates to
    /// [`morse::decode_morse_ident`].
    ///
    /// Using `filtfilt_bandpass` (IIR, Q ≈ 5, 0 dB peak gain) instead of the
    /// FIR `bpf_ident` is critical: the FIR filter has too few taps at 9 kHz to
    /// adequately narrow the passband, so its output is dominated by out-of-band
    /// noise.  The IIR filtfilt applies twice (forward + backward) but with the
    /// corrected 0 dB normalisation, so the total gain stays near unity.
    pub fn decode_ident(
        &mut self,
        audio: &[f64],
    ) -> (Option<String>, Vec<String>, Vec<morse::MorseDecodeAttempt>) {
        if audio.len() < (self.audio_rate as usize / 2) {
            return (None, Vec::new(), Vec::new());
        }

        // IIR zero-phase bandpass: sharper and better-normalised than the FIR path.
        let tone = filtfilt_bandpass(
            audio,
            ILS_MORSE_BPF_LOW,
            ILS_MORSE_BPF_HIGH,
            self.audio_rate,
            ILS_MORSE_BPF_ORDER,
        );
        let analytic = hilbert_transform(&tone);
        let env_raw = envelope(&analytic);

        // LP cutoff with stated value; actual effective cutoff is ~0.5 × stated
        // with cascaded-biquad filtfilt order=4.
        let env = filtfilt_lowpass(
            &env_raw,
            ILS_MORSE_ENV_LPF_CUTOFF,
            self.audio_rate,
            ILS_MORSE_ENV_LPF_ORDER,
        );

        morse::decode_morse_ident(&env, self.audio_rate)
    }
}

/// Compute DDM from mean envelopes over a window.
///
/// Returns `(ddm, mod_90_pct, mod_150_pct, signal_strength)`.
/// Returns `None` if the window is empty or the sum is too small (noise floor).
/// Compute ILS Difference in Depth of Modulation (DDM) from demodulated signals.
///
/// Returns structured result containing DDM, modulation depths, and carrier strength,
/// with proper error handling for edge cases.
pub fn compute_ddm(
    env_90: &[f64],
    env_150: &[f64],
    audio_envelope: &[f64],
) -> Result<IlsDdmResult> {
    let n = env_90.len().min(env_150.len()).min(audio_envelope.len());
    if n == 0 {
        return Err(Error::EmptyInput);
    }

    // Validate input signals don't contain NaN/Inf
    if env_90[..n].iter().any(|&x| !x.is_finite())
        || env_150[..n].iter().any(|&x| !x.is_finite())
        || audio_envelope[..n].iter().any(|&x| !x.is_finite())
    {
        return Err(Error::NonFiniteSignal);
    }

    let mean_90 = env_90[..n].iter().sum::<f64>() / n as f64;
    let mean_150 = env_150[..n].iter().sum::<f64>() / n as f64;
    let mean_env = audio_envelope[..n].iter().sum::<f64>() / n as f64;

    let sum_90_150 = mean_90 + mean_150;
    if sum_90_150 < ILS_DDM_SUM_MIN {
        return Err(Error::DdmComputationFailed {
            reason: "both 90 Hz and 150 Hz modulation depths are near zero".to_string(),
        });
    }

    let ddm = (mean_90 - mean_150) / sum_90_150;

    // For WAV files with pre-demodulated audio, the envelope values from
    // filtfilt_bandpass may have a different scale than expected. Clamp modulation
    // percentages to a reasonable range [0, 100].
    let (mod_90_pct, mod_150_pct) = if mean_env > MODULATION_DEPTH_MIN {
        let pct_90 = (mean_90 / mean_env * 100.0).min(100.0);
        let pct_150 = (mean_150 / mean_env * 100.0).min(100.0);
        (pct_90, pct_150)
    } else {
        (0.0, 0.0)
    };

    // Signal strength: normalised carrier level (clamp to 0..1)
    let carrier_strength = (mean_env / ILS_CARRIER_NORMALIZATION).clamp(0.0, 1.0);

    Ok(IlsDdmResult {
        ddm,
        mod_90_hz: mod_90_pct,
        mod_150_hz: mod_150_pct,
        carrier_strength,
    })
}

// ── Internal DSP helpers ──────────────────────────────────────────────────────

// ── Public constructor for IlsFrame ──────────────────────────────────────────

impl IlsFrame {
    /// Build an `IlsFrame` from pre-computed DDM components.
    pub fn new(
        timestamp: f64,
        frequency_mhz: f64,
        ddm: f64,
        mod_90_pct: f64,
        mod_150_pct: f64,
        signal_strength: f64,
        ident: Option<String>,
    ) -> Self {
        let side = IlsSide::from_ddm(ddm);
        Self {
            timestamp,
            frequency_mhz,
            ddm,
            mod_90_pct,
            mod_150_pct,
            signal_strength,
            ident,
            morse_debug: None,
            side,
        }
    }

    pub fn with_morse_debug(mut self, debug: Option<IlsMorseDebugInfo>) -> Self {
        self.morse_debug = debug;
        self
    }
}
